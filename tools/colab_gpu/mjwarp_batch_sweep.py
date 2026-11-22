"""Batched C++/CUDA Force Writers + mujoco_warp Parameter Sweep & Tyre Backend Benchmark.

Executes two core GPU benchmarks on NVIDIA CUDA (e.g. Google Colab L4):
  1. Experiment A (`run_parameter_sweep`):
     Runs `nworld = 2048` heterogeneous worlds in parallel (`fork_psi_scale` in [0.8, 1.25],
     `rebound_scale` in [0.7, 1.4], `shock_rate_scale` in [0.85, 1.20], `tire_stiff_scale` in
     [0.85, 1.20]) inside a single `wp.ScopedCapture()` CUDA Graph executing the `clean_split`
     schedule (`fwd_kinematics -> crb -> tendon_armature -> factor_m -> collision -> transmission
     -> fwd_velocity -> bike_gpu_launch_writers -> make_constraint -> fwd_actuation ->
     fwd_acceleration(factorize=False) -> solve -> sensor_acc -> implicit`).
  2. Experiment B (`run_tyre_backend_scaling_benchmark`):
     Benchmarks the fused C++/CUDA writers + `clean_split` at `nworld = 2048` across the unified
     `tyre_gpu_step.hpp` backends (`compliant_2d`, `distributed_2d_reference` at N=32/64/128).
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable

import numpy as np

try:
    from tools.colab_gpu.build_cuda_writers import (
        DEFAULT_GPU_SRC_DIR,
        DEFAULT_OUT_SO_PATH,
        GpuModelIndicesStruct,
        GpuSp3ConfigStruct,
        apply_physics_config_json_to_sp3_config,
        build_sp3_config_struct_from_mjmodel,
        build_sp3_sweep_configs_bytes,
        build_spindle_params_bytes,
        build_sweep_configs_bytes,
        build_tyre_and_terrain_buffers,
        compile_cuda_writers,
        expand_mjwarp_tendon_range_and_frictionloss,
        materialize_model_12gears_xml,
        resolve_gpu_model_indices,
        unpack_sp4_world_metrics,
    )
    from tools.colab_gpu.model_assets import (
        load_captured_initial_state,
        load_model_with_assets,
        materialize_welded_assets_dir,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_cuda_writers import (  # type: ignore[no-redef]
        DEFAULT_GPU_SRC_DIR,
        DEFAULT_OUT_SO_PATH,
        GpuModelIndicesStruct,
        GpuSp3ConfigStruct,
        apply_physics_config_json_to_sp3_config,
        build_sp3_config_struct_from_mjmodel,
        build_sp3_sweep_configs_bytes,
        build_spindle_params_bytes,
        build_sweep_configs_bytes,
        build_tyre_and_terrain_buffers,
        compile_cuda_writers,
        expand_mjwarp_tendon_range_and_frictionloss,
        materialize_model_12gears_xml,
        resolve_gpu_model_indices,
        unpack_sp4_world_metrics,
    )
    from model_assets import (  # type: ignore[no-redef]
        load_captured_initial_state,
        load_model_with_assets,
        materialize_welded_assets_dir,
    )

try:
    import mujoco
except Exception:  # pragma: no cover
    mujoco = None  # type: ignore[assignment]

try:
    import warp as wp

    wp.config.enable_mathdx_gemm = False
    wp.config.enable_mathdx_solver = False
    import mujoco_warp as mjwarp
except Exception:  # pragma: no cover - imported on CUDA/Colab runtime
    mjwarp = None  # type: ignore[assignment]
    wp = None  # type: ignore[assignment]

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL_XML = REPO_ROOT / "tools" / "fp32_probe" / "artifacts" / "common" / "model.xml"

TYRE_BACKEND_TIERS: tuple[dict[str, Any], ...] = (
    {
        "tier_id": "compliant_2d",
        "label": "Compliant 2D (polyline O(1))",
        "backend": "compliant_2d",
        "station_count": 0,
        "units_per_wheel": 1,
        "units_per_world": 2,
    },
    {
        "tier_id": "distributed_32",
        "label": "Distributed 2D (N=32 stations)",
        "backend": "distributed_2d_reference",
        "station_count": 32,
        "units_per_wheel": 32,
        "units_per_world": 64,
    },
    {
        "tier_id": "distributed_64",
        "label": "Distributed 2D (N=64 stations)",
        "backend": "distributed_2d_reference",
        "station_count": 64,
        "units_per_wheel": 64,
        "units_per_world": 128,
    },
    {
        "tier_id": "distributed_128",
        "label": "Distributed 2D (N=128 stations)",
        "backend": "distributed_2d_reference",
        "station_count": 128,
        "units_per_wheel": 128,
        "units_per_world": 256,
    },
)


def _require_gpu_stack() -> None:
    if mujoco is None or wp is None or mjwarp is None:
        raise RuntimeError(
            "mujoco, warp-lang, and mujoco_warp are required to run the GPU batch benchmarks."
        )
    wp.config.enable_mathdx_gemm = False
    wp.config.enable_mathdx_solver = False
    wp.init()


def _allocate_and_copy_device_bytes(raw_bytes: bytes, device: str = "cuda:0") -> Any:
    """Allocate a `wp.zeros(..., dtype=wp.uint8)` device buffer and copy `raw_bytes` via `wp.copy`."""
    host_np = np.frombuffer(raw_bytes, dtype=np.uint8).copy()
    host_wp = wp.array(host_np, dtype=wp.uint8, device="cpu")
    dev_wp = wp.zeros(len(raw_bytes), dtype=wp.uint8, device=device)
    wp.copy(dev_wp, host_wp)
    return dev_wp


def _ensure_newton_solver(m: Any) -> None:
    """Enforce Newton solver (CG is rejected on GPU sweeps per simulation-decisions.md)."""
    if mujoco is not None and hasattr(m, "opt") and hasattr(mujoco, "mjtSolver"):
        m.opt.solver = int(mujoco.mjtSolver.mjSOL_NEWTON)


def _polyline_slope_from_desc_and_f32(
    desc_bytes: bytes,
    f32_np: np.ndarray,
    x_m: float,
) -> float:
    """Evaluate terrain polyline slope ``dz/dx`` at ``x_m`` from packed ``(desc, f32)``."""
    import struct

    if len(desc_bytes) < 12 or f32_np.size < 4:
        return 0.0
    n_vert, px_off, pz_off = struct.unpack_from("<3i", desc_bytes, 0)
    if n_vert < 2 or px_off < 0 or pz_off < 0:
        return 0.0
    if px_off + n_vert > f32_np.size or pz_off + n_vert > f32_np.size:
        return 0.0
    px = f32_np[px_off : px_off + n_vert]
    pz = f32_np[pz_off : pz_off + n_vert]
    idx = int(np.clip(int(np.searchsorted(px, float(x_m), side="right")) - 1, 0, n_vert - 2))
    dx = float(px[idx + 1]) - float(px[idx])
    dz = float(pz[idx + 1]) - float(pz[idx])
    return (dz / dx) if abs(dx) > 1e-12 else 0.0


def _upload_tyre_and_terrain_device(
    host_lib: ctypes.CDLL,
    xml_path: Path,
    backend: str,
    station_count: int,
    idx_bytes: bytes,
    device: str,
    surface_label: int = -2,
) -> tuple[Any, Any, Any, Any, int, int]:
    """Build host tyre/terrain tables and upload ``(tyre_model, desc, f32, i32)`` to ``device``."""
    model_b, desc_b, f32_np, i32_np = build_tyre_and_terrain_buffers(
        host_lib=host_lib,
        xml_path=xml_path,
        backend=backend,
        station_count=station_count,
        idx_bytes=idx_bytes,
        surface_label=surface_label,
    )
    d_tyre_model = _allocate_and_copy_device_bytes(model_b, device=device)
    d_terrain_desc = _allocate_and_copy_device_bytes(desc_b, device=device)
    d_terrain_f32 = (
        wp.array(f32_np, dtype=wp.float32, device=device)
        if f32_np.size > 0
        else wp.zeros(1, dtype=wp.float32, device=device)
    )
    d_terrain_i32 = (
        wp.array(i32_np, dtype=wp.int32, device=device)
        if i32_np.size > 0
        else wp.zeros(1, dtype=wp.int32, device=device)
    )
    return (
        d_tyre_model,
        d_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        int(d_terrain_f32.ptr),
        int(d_terrain_i32.ptr),
    )


def _resolve_clean_split_ops(mjw_mod: Any) -> dict[str, Any]:
    """Resolve ``mujoco_warp`` submodules/functions for the SP1 ``clean_split`` step."""
    src = getattr(mjw_mod, "_src", None)
    fwd = getattr(mjw_mod, "forward", None) or getattr(src, "forward", None) or mjw_mod
    smooth = getattr(mjw_mod, "smooth", None) or getattr(src, "smooth", None) or mjw_mod
    coll = (
        getattr(mjw_mod, "collision_driver", None)
        or getattr(src, "collision_driver", None)
        or mjw_mod
    )
    cons = getattr(mjw_mod, "constraint", None) or getattr(src, "constraint", None) or mjw_mod
    sol = getattr(mjw_mod, "solver", None) or getattr(src, "solver", None) or mjw_mod
    sens = getattr(mjw_mod, "sensor", None) or getattr(src, "sensor", None) or mjw_mod

    def _pick(primary: Any, *names: str) -> Any:
        for obj in (primary, mjw_mod, fwd, smooth, coll, cons, sol, sens):
            if obj is None:
                continue
            for name in names:
                fn = getattr(obj, name, None)
                if callable(fn):
                    return fn
        raise RuntimeError(f"clean_split could not resolve any of {names} on mujoco_warp")

    kin_fn = getattr(fwd, "fwd_kinematics", None) or getattr(mjw_mod, "fwd_kinematics", None)
    if callable(kin_fn):
        kin_stage = lambda m_wp, d_b: kin_fn(m_wp, d_b)
    else:
        k1 = _pick(smooth, "kinematics")
        k2 = _pick(smooth, "com_pos")
        k3 = _pick(smooth, "camlight")
        k4 = _pick(smooth, "flex")
        k5 = _pick(smooth, "tendon")

        def kin_stage(m_wp: Any, d_b: Any) -> None:
            k1(m_wp, d_b)
            k2(m_wp, d_b)
            k3(m_wp, d_b)
            k4(m_wp, d_b)
            k5(m_wp, d_b)

    return {
        "fwd_kinematics": kin_stage,
        "crb": _pick(smooth, "crb"),
        "tendon_armature": _pick(smooth, "tendon_armature"),
        "factor_m": _pick(smooth, "factor_m"),
        "collision": _pick(coll, "collision"),
        "transmission": _pick(smooth, "transmission"),
        "fwd_velocity": _pick(fwd, "fwd_velocity"),
        "sensor_vel": _pick(sens, "sensor_vel"),
        "make_constraint": _pick(cons, "make_constraint"),
        "fwd_actuation": _pick(fwd, "fwd_actuation"),
        "fwd_acceleration": _pick(fwd, "fwd_acceleration"),
        "solve": _pick(sol, "solve"),
        "sensor_acc": _pick(sens, "sensor_acc"),
        "implicit": _pick(fwd, "implicit"),
    }


def _mjwarp_clean_split_step(
    ops: dict[str, Any],
    m_wp: Any,
    d_b: Any,
    launch_writers_fn: Callable[[], None],
) -> None:
    """Execute one SP1 ``clean_split`` step (``fwd_velocity`` before ``make_constraint``)."""
    ops["fwd_kinematics"](m_wp, d_b)
    ops["crb"](m_wp, d_b)
    ops["tendon_armature"](m_wp, d_b)
    ops["factor_m"](m_wp, d_b)
    ops["collision"](m_wp, d_b)
    ops["transmission"](m_wp, d_b)
    ops["fwd_velocity"](m_wp, d_b)
    launch_writers_fn()
    ops["make_constraint"](m_wp, d_b)
    ops["fwd_actuation"](m_wp, d_b)
    ops["fwd_acceleration"](m_wp, d_b, factorize=False)
    ops["solve"](m_wp, d_b)
    ops["sensor_acc"](m_wp, d_b)
    ops["implicit"](m_wp, d_b)


def _mjwarp_sp3_clean_split_step(
    ops: dict[str, Any],
    m_wp: Any,
    d_b: Any,
    launch_pre_solve_fn: Callable[[], None],
    launch_post_solve_fn: Callable[[], None],
) -> None:
    """Execute one autonomous SP3 ``clean_split`` step (pre_solve -> solve/implicit -> post_solve)."""
    ops["fwd_kinematics"](m_wp, d_b)
    ops["crb"](m_wp, d_b)
    ops["tendon_armature"](m_wp, d_b)
    ops["factor_m"](m_wp, d_b)
    ops["collision"](m_wp, d_b)
    ops["transmission"](m_wp, d_b)
    ops["fwd_velocity"](m_wp, d_b)
    ops["sensor_vel"](m_wp, d_b)
    launch_pre_solve_fn()
    ops["make_constraint"](m_wp, d_b)
    ops["fwd_actuation"](m_wp, d_b)
    ops["fwd_acceleration"](m_wp, d_b, factorize=False)
    ops["solve"](m_wp, d_b)
    ops["sensor_acc"](m_wp, d_b)
    ops["implicit"](m_wp, d_b)
    launch_post_solve_fn()


_SP4_XZ_SHIFT_KERNEL: Any = None
_SP4_EXACT_TENDON_LENGTH_KERNEL: Any = None


def _get_sp4_xz_shift_kernel() -> Any:
    """Lazily construct the Warp kernel that centers (root_x, root_z) around (0, 0)."""
    global _SP4_XZ_SHIFT_KERNEL
    if _SP4_XZ_SHIFT_KERNEL is not None:
        return _SP4_XZ_SHIFT_KERNEL
    if wp is None:
        raise RuntimeError("warp-lang is required for SP4 XZ coordinate rebasing")

    @wp.kernel
    def _sp4_shift_xz_kernel(
        qpos: wp.array2d[float],
        xpos: wp.array2d[wp.vec3],
        xanchor: wp.array2d[wp.vec3],
        site_xpos: wp.array2d[wp.vec3],
        subtree_com: wp.array2d[wp.vec3],
        xz_off: wp.array[wp.vec2],
        saved_xz: wp.array2d[float],
        saved_xpos: wp.array2d[wp.vec3],
        saved_xanchor: wp.array2d[wp.vec3],
        saved_site_xpos: wp.array2d[wp.vec3],
        saved_subtree_com: wp.array2d[wp.vec3],
        rx0: int,
        rz0: int,
        rx1: int,
        rz1: int,
        sign: float,
        mode: int,
    ) -> None:
        w = wp.tid()
        if mode == 1:
            dx = wp.trunc(qpos[w, rx0])
            dz = wp.trunc(qpos[w, rz0])
            xz_off[w] = wp.vec2(dx, dz)
            qpos[w, rx0] = qpos[w, rx0] - dx
            qpos[w, rz0] = qpos[w, rz0] - dz
            if rx1 >= 0:
                qpos[w, rx1] = qpos[w, rx1] - dx
            if rz1 >= 0:
                qpos[w, rz1] = qpos[w, rz1] - dz
            return
        if mode == 3:
            qpos[w, rx0] = saved_xz[w, 0]
            qpos[w, rz0] = saved_xz[w, 1]
            if rx1 >= 0:
                qpos[w, rx1] = saved_xz[w, 2]
            if rz1 >= 0:
                qpos[w, rz1] = saved_xz[w, 3]
            for i in range(1, xpos.shape[1]):
                xpos[w, i] = saved_xpos[w, i]
            for i in range(xanchor.shape[1]):
                xanchor[w, i] = saved_xanchor[w, i]
            for i in range(site_xpos.shape[1]):
                site_xpos[w, i] = saved_site_xpos[w, i]
            for i in range(1, subtree_com.shape[1]):
                subtree_com[w, i] = saved_subtree_com[w, i]
            return
        if mode == 2:
            saved_xz[w, 0] = qpos[w, rx0]
            saved_xz[w, 1] = qpos[w, rz0]
            if rx1 >= 0:
                saved_xz[w, 2] = qpos[w, rx1]
            if rz1 >= 0:
                saved_xz[w, 3] = qpos[w, rz1]
            for i in range(1, xpos.shape[1]):
                saved_xpos[w, i] = xpos[w, i]
            for i in range(xanchor.shape[1]):
                saved_xanchor[w, i] = xanchor[w, i]
            for i in range(site_xpos.shape[1]):
                saved_site_xpos[w, i] = site_xpos[w, i]
            for i in range(1, subtree_com.shape[1]):
                saved_subtree_com[w, i] = subtree_com[w, i]
        off = xz_off[w]
        s = wp.vec3(sign * off[0], 0.0, sign * off[1])
        qpos[w, rx0] = qpos[w, rx0] + s[0]
        qpos[w, rz0] = qpos[w, rz0] + s[2]
        if rx1 >= 0:
            qpos[w, rx1] = qpos[w, rx1] + s[0]
        if rz1 >= 0:
            qpos[w, rz1] = qpos[w, rz1] + s[2]
        for i in range(1, xpos.shape[1]):
            xpos[w, i] = xpos[w, i] + s
        for i in range(xanchor.shape[1]):
            xanchor[w, i] = xanchor[w, i] + s
        for i in range(site_xpos.shape[1]):
            site_xpos[w, i] = site_xpos[w, i] + s
        for i in range(1, subtree_com.shape[1]):
            subtree_com[w, i] = subtree_com[w, i] + s

    _SP4_XZ_SHIFT_KERNEL = _sp4_shift_xz_kernel
    return _SP4_XZ_SHIFT_KERNEL


def _get_sp4_exact_tendon_length_kernel() -> Any:
    """Lazily construct the Warp kernel computing 2-joint tendon lengths in df32."""
    global _SP4_EXACT_TENDON_LENGTH_KERNEL
    if _SP4_EXACT_TENDON_LENGTH_KERNEL is not None:
        return _SP4_EXACT_TENDON_LENGTH_KERNEL
    if wp is None:
        raise RuntimeError("warp-lang is required for SP4 exact tendon length kernel")

    @wp.func
    def _sp4_df32_two_sum(a: float, b: float) -> wp.vec2:
        s = a + b
        bb = s - a
        e = (a - (s - bb)) + (b - bb)
        return wp.vec2(s, e)

    @wp.func
    def _sp4_df32_two_prod(a: float, b: float) -> wp.vec2:
        ca = float(4097.0) * a
        a_hi = ca - (ca - a)
        a_lo = a - a_hi
        cb = float(4097.0) * b
        b_hi = cb - (cb - b)
        b_lo = b - b_hi
        p = a * b
        e = ((a_hi * b_hi - p) + a_hi * b_lo + a_lo * b_hi) + a_lo * b_lo
        return wp.vec2(p, e)

    @wp.func
    def _sp4_df32_add(x: wp.vec2, y: wp.vec2) -> wp.vec2:
        s = _sp4_df32_two_sum(x[0], y[0])
        t = _sp4_df32_two_sum(x[1], y[1])
        v = _sp4_df32_two_sum(s[0], s[1] + t[0])
        return _sp4_df32_two_sum(v[0], t[1] + v[1])

    @wp.kernel
    def _sp4_exact_tendon_length_kernel(
        qpos: wp.array2d[float],
        ten_ids: wp.array[int],
        qadr0: wp.array[int],
        qadr1: wp.array[int],
        coef0: wp.array[float],
        coef1: wp.array[float],
        ten_length: wp.array2d[float],
    ) -> None:
        w, i = wp.tid()
        tid = ten_ids[i]
        p0 = _sp4_df32_two_prod(coef0[i], qpos[w, qadr0[i]])
        p1 = _sp4_df32_two_prod(coef1[i], qpos[w, qadr1[i]])
        s = _sp4_df32_add(p0, p1)
        ten_length[w, tid] = s[0] + s[1]

    _SP4_EXACT_TENDON_LENGTH_KERNEL = _sp4_exact_tendon_length_kernel
    return _SP4_EXACT_TENDON_LENGTH_KERNEL


def _build_sp4_xz_rebase_ctx(m_mj: Any, nworld: int, device: str) -> dict[str, Any] | None:
    """Build per-world XZ coordinate rebase and df32 2-joint tendon state for FP32 mjwarp."""
    if mujoco is None or wp is None:
        return None
    def _jnt_qadr(name: str) -> int:
        jid = int(mujoco.mj_name2id(m_mj, mujoco.mjtObj.mjOBJ_JOINT, name))
        return int(m_mj.jnt_qposadr[jid]) if jid >= 0 else -1

    rx0 = _jnt_qadr("root_x")
    rz0 = _jnt_qadr("root_z")
    if rx0 < 0 or rz0 < 0:
        return None
    rx1 = _jnt_qadr("rider_root_x")
    rz1 = _jnt_qadr("rider_root_z")

    ten_ids_list: list[int] = []
    qadr0_list: list[int] = []
    qadr1_list: list[int] = []
    coef0_list: list[np.float32] = []
    coef1_list: list[np.float32] = []
    wrap_joint = int(mujoco.mjtWrap.mjWRAP_JOINT)
    for t in range(int(m_mj.ntendon)):
        if int(m_mj.tendon_num[t]) != 2:
            continue
        adr = int(m_mj.tendon_adr[t])
        if (
            int(m_mj.wrap_type[adr]) != wrap_joint
            or int(m_mj.wrap_type[adr + 1]) != wrap_joint
        ):
            continue
        j0 = int(m_mj.wrap_objid[adr])
        j1 = int(m_mj.wrap_objid[adr + 1])
        if j0 < 0 or j1 < 0:
            continue
        qa0 = int(m_mj.jnt_qposadr[j0])
        qa1 = int(m_mj.jnt_qposadr[j1])
        if qa0 < 0 or qa1 < 0:
            continue
        ten_ids_list.append(t)
        qadr0_list.append(qa0)
        qadr1_list.append(qa1)
        coef0_list.append(np.float32(m_mj.wrap_prm[adr]))
        coef1_list.append(np.float32(m_mj.wrap_prm[adr + 1]))

    n_exact_tendons = len(ten_ids_list)
    exact_tendon_ctx: dict[str, Any] = {"n_exact_tendons": n_exact_tendons}
    if n_exact_tendons > 0:
        exact_tendon_ctx.update(
            {
                "exact_tendon_kernel": _get_sp4_exact_tendon_length_kernel(),
                "ten_ids": wp.array(np.array(ten_ids_list, dtype=np.int32), dtype=wp.int32, device=device),
                "qadr0": wp.array(np.array(qadr0_list, dtype=np.int32), dtype=wp.int32, device=device),
                "qadr1": wp.array(np.array(qadr1_list, dtype=np.int32), dtype=wp.int32, device=device),
                "coef0": wp.array(np.array(coef0_list, dtype=np.float32), dtype=wp.float32, device=device),
                "coef1": wp.array(np.array(coef1_list, dtype=np.float32), dtype=wp.float32, device=device),
            }
        )

    return {
        "kernel": _get_sp4_xz_shift_kernel(),
        "xz_off": wp.zeros(int(nworld), dtype=wp.vec2, device=device),
        "saved_xz": wp.zeros((int(nworld), 4), dtype=wp.float32, device=device),
        "saved_xpos": wp.zeros((int(nworld), int(m_mj.nbody)), dtype=wp.vec3, device=device),
        "saved_xanchor": wp.zeros((int(nworld), int(m_mj.njnt)), dtype=wp.vec3, device=device),
        "saved_site_xpos": wp.zeros((int(nworld), int(m_mj.nsite)), dtype=wp.vec3, device=device),
        "saved_subtree_com": wp.zeros((int(nworld), int(m_mj.nbody)), dtype=wp.vec3, device=device),
        "rx0": rx0,
        "rz0": rz0,
        "rx1": rx1,
        "rz1": rz1,
        "nworld": int(nworld),
        "device": device,
        **exact_tendon_ctx,
    }


def _launch_sp4_xz_shift(
    xz_ctx: dict[str, Any] | None,
    d_b: Any,
    sign: float,
    mode: int,
) -> None:
    if xz_ctx is None:
        return
    wp.launch(
        xz_ctx["kernel"],
        dim=xz_ctx["nworld"],
        inputs=[
            d_b.qpos,
            d_b.xpos,
            d_b.xanchor,
            d_b.site_xpos,
            d_b.subtree_com,
            xz_ctx["xz_off"],
            xz_ctx["saved_xz"],
            xz_ctx["saved_xpos"],
            xz_ctx["saved_xanchor"],
            xz_ctx["saved_site_xpos"],
            xz_ctx["saved_subtree_com"],
            xz_ctx["rx0"],
            xz_ctx["rz0"],
            xz_ctx["rx1"],
            xz_ctx["rz1"],
            float(sign),
            int(mode),
        ],
        device=xz_ctx["device"],
    )


def _launch_sp4_exact_tendon_length(
    xz_ctx: dict[str, Any] | None,
    d_b: Any,
) -> None:
    if xz_ctx is None or int(xz_ctx.get("n_exact_tendons", 0)) <= 0:
        return
    wp.launch(
        xz_ctx["exact_tendon_kernel"],
        dim=(xz_ctx["nworld"], int(xz_ctx["n_exact_tendons"])),
        inputs=[
            d_b.qpos,
            xz_ctx["ten_ids"],
            xz_ctx["qadr0"],
            xz_ctx["qadr1"],
            xz_ctx["coef0"],
            xz_ctx["coef1"],
            d_b.ten_length,
        ],
        device=xz_ctx["device"],
    )


def _mjwarp_sp4_clean_split_step(
    ops: dict[str, Any],
    m_wp: Any,
    d_b: Any,
    launch_pre_solve_fn: Callable[[], None],
    launch_pass1_fn: Callable[[], None],
    launch_pass2_fn: Callable[[], None],
    launch_pass3_fn: Callable[[], None],
    launch_post_solve_fn: Callable[[], None],
    run_collision: bool = False,
    xz_ctx: dict[str, Any] | None = None,
) -> None:
    """Execute one full SP4 closed-loop 4x-solve ``clean_split`` step."""
    # Center (root_x, root_z) near (0, 0) during mjwarp kinematics/constraints/solver
    # so FP32 xpos subtraction in _equality_connect does not lose precision at x=20..100m.
    _launch_sp4_xz_shift(xz_ctx, d_b, 0.0, 1)
    ops["fwd_kinematics"](m_wp, d_b)
    _launch_sp4_exact_tendon_length(xz_ctx, d_b)
    ops["crb"](m_wp, d_b)
    ops["tendon_armature"](m_wp, d_b)
    ops["factor_m"](m_wp, d_b)
    ops["transmission"](m_wp, d_b)
    ops["fwd_velocity"](m_wp, d_b)
    ops["sensor_vel"](m_wp, d_b)
    # Restore true world (xpos, xanchor, site_xpos, subtree_com) BEFORE collision and pre_solve
    # so static world geoms (terrain hfield, catch_plane) collide at true world coordinates.
    _launch_sp4_xz_shift(xz_ctx, d_b, 1.0, 2)
    if run_collision:
        ops["collision"](m_wp, d_b)
    launch_pre_solve_fn()
    _launch_sp4_xz_shift(xz_ctx, d_b, -1.0, 3)
    ops["make_constraint"](m_wp, d_b)
    ops["fwd_actuation"](m_wp, d_b)
    ops["fwd_acceleration"](m_wp, d_b, factorize=False)
    ops["solve"](m_wp, d_b)
    launch_pass1_fn()
    ops["fwd_actuation"](m_wp, d_b)
    ops["fwd_acceleration"](m_wp, d_b, factorize=False)
    ops["solve"](m_wp, d_b)
    launch_pass2_fn()
    ops["fwd_actuation"](m_wp, d_b)
    ops["fwd_acceleration"](m_wp, d_b, factorize=False)
    ops["solve"](m_wp, d_b)
    launch_pass3_fn()
    ops["fwd_actuation"](m_wp, d_b)
    ops["fwd_acceleration"](m_wp, d_b, factorize=False)
    ops["solve"](m_wp, d_b)
    ops["sensor_acc"](m_wp, d_b)
    ops["implicit"](m_wp, d_b)
    _launch_sp4_xz_shift(xz_ctx, d_b, 1.0, 0)
    launch_post_solve_fn()


REPO_ROOT = DEFAULT_GPU_SRC_DIR.parents[2]
DEFAULT_PHYSICS_TOML = REPO_ROOT / "examples" / "research" / "viewer_physics_welded.toml"
DEFAULT_STRENGTH_JSON = REPO_ROOT / "examples" / "research" / "rider_strength_reference.json"
DEFAULT_ENVELOPE_JSON = (
    REPO_ROOT / "examples" / "research" / "rider_joint_envelope_anatomical.json"
)
# Authoritative M0 FP64 distances from native/tests/gpu/test_sp4_s1s4_cpu_acceptance.cpp
M0_S1S4_REFERENCE_DISTANCES_M = {
    "S1_flat": 28.9273,
    "S2_climb15": 96.2017,
    "S3_steady90": 50.0476,
    "S4_start_uphill15_60nm": 26.1543,
}


def _find_bike_gpu_export_bin() -> Path:
    candidates = (
        REPO_ROOT / "build" / "native-release" / "bin" / "bike-gpu-export",
        REPO_ROOT / "build" / "native-release" / "bike-gpu-export",
    )
    for cand in candidates:
        if cand.is_file():
            return cand
    found = shutil.which("bike-gpu-export")
    if found:
        return Path(found)
    raise FileNotFoundError(
        "bike-gpu-export not found; build target bike-gpu-export in build/native-release"
    )


def _load_step_closure_row0(bundle_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    trace_path = bundle_dir / "step_closure.bin"
    with trace_path.open("rb") as f:
        hdr_bytes = f.read(80)
        if len(hdr_bytes) != 80:
            raise RuntimeError(f"Short step_closure.bin header in {trace_path}")
        hdr = struct.unpack("<8sIIQ6I5iIQ", hdr_bytes)
        magic, version, header_bytes, n_steps, row_doubles, nq, nv, na = hdr[:8]
        if magic != b"BGSC0001" or version != 1 or header_bytes != 80 or n_steps < 1:
            raise RuntimeError(f"Invalid step_closure.bin header in {trace_path}: {hdr[:5]}")
        row0 = np.fromfile(f, dtype="<f8", count=int(row_doubles))
    if row0.size != int(row_doubles):
        raise RuntimeError(f"Short row0 in {trace_path}")
    off = 3
    qpos0 = row0[off : off + int(nq)].copy()
    off += int(nq)
    qvel0 = row0[off : off + int(nv)].copy()
    off += int(nv) + int(na)
    qacc_ws0 = row0[off : off + int(nv)].copy()
    return qpos0, qvel0, qacc_ws0


def _export_sp4_scenario_bundle(
    track_rel: str,
    v0: float,
    out_dir: Path,
    physics_toml: Path = DEFAULT_PHYSICS_TOML,
    sp4_bundle_dir: Path | None = None,
    scenario_id: str | None = None,
) -> tuple[Path, np.ndarray, np.ndarray, np.ndarray]:
    if sp4_bundle_dir is not None:
        base_dir = Path(sp4_bundle_dir).resolve()
        bundle_sub = (
            base_dir / scenario_id
            if scenario_id is not None and (base_dir / scenario_id / "model_12gears.xml").is_file()
            else (
                base_dir
                if (base_dir / "model_12gears.xml").is_file()
                else base_dir / (scenario_id or "S1_flat")
            )
        )
        if not (bundle_sub / "model_12gears.xml").is_file():
            raise FileNotFoundError(
                f"Pre-exported SP4 bundle missing model_12gears.xml in {bundle_sub}"
            )
        qpos0, qvel0, qacc_ws0 = _load_step_closure_row0(bundle_sub)
        return bundle_sub / "model_12gears.xml", qpos0, qvel0, qacc_ws0

    bin_path = _find_bike_gpu_export_bin()
    cmd = [
        str(bin_path),
        "--physics-config",
        str(physics_toml.resolve()),
        "--track",
        str((REPO_ROOT / track_rel).resolve()),
        "--out",
        str(out_dir.resolve()),
        "--steps",
        "1",
        "--initial-speed-mps",
        repr(float(v0)),
    ]
    proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"bike-gpu-export failed (exit {proc.returncode}):\n"
            f"CMD: {' '.join(cmd)}\nSTDERR:\n{proc.stderr}"
        )
    qpos0, qvel0, qacc_ws0 = _load_step_closure_row0(out_dir)
    return out_dir / "model_12gears.xml", qpos0, qvel0, qacc_ws0


def _compute_steady90_v0(
    physics_toml: Path = DEFAULT_PHYSICS_TOML,
    sp4_bundle_dir: Path | None = None,
) -> float:
    if sp4_bundle_dir is not None:
        manifest_path = Path(sp4_bundle_dir).resolve() / "manifest.json"
        if manifest_path.is_file():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if "steady90_v0" in manifest:
                return float(manifest["steady90_v0"])
        xml12, qpos0, _, _ = _export_sp4_scenario_bundle(
            "tests/fixtures/tracks/flat.toml",
            0.0,
            Path(sp4_bundle_dir),
            physics_toml=physics_toml,
            sp4_bundle_dir=sp4_bundle_dir,
            scenario_id="S1_flat",
        )
        m = load_model_with_assets(xml12)
        d = mujoco.MjData(m)
        d.qpos[:] = qpos0
        mujoco.mj_forward(m, d)
        rw_bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "rear_wheel"))
        tg = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "terrain"))
        hf_id = int(m.geom_dataid[tg])
        z_road = float(m.geom_pos[tg, 2]) + float(m.hfield_data[0]) * float(m.hfield_size[hf_id, 2])
        r_eff = float(d.xpos[rw_bid, 2]) - z_road
        phys_cfg = json.loads((xml12.parent / "physics_config.json").read_text(encoding="utf-8"))
        ft = int(phys_cfg["drive"]["gearing"]["front_teeth"])
        rt = int(phys_cfg["drive"]["gearing"]["rear_teeth"])
        return 90.0 * 2.0 * math.pi / 60.0 * (float(ft) / float(rt)) * r_eff

    with tempfile.TemporaryDirectory(prefix="bike-s3-v0-") as td:
        td_path = Path(td)
        xml12, qpos0, _, _ = _export_sp4_scenario_bundle(
            "tests/fixtures/tracks/flat.toml", 0.0, td_path, physics_toml=physics_toml
        )
        m = load_model_with_assets(xml12)
        d = mujoco.MjData(m)
        d.qpos[:] = qpos0
        mujoco.mj_forward(m, d)
        rw_bid = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "rear_wheel"))
        tg = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "terrain"))
        hf_id = int(m.geom_dataid[tg])
        z_road = float(m.geom_pos[tg, 2]) + float(m.hfield_data[0]) * float(m.hfield_size[hf_id, 2])
        r_eff = float(d.xpos[rw_bid, 2]) - z_road
        phys_cfg = json.loads((td_path / "physics_config.json").read_text(encoding="utf-8"))
        ft = int(phys_cfg["drive"]["gearing"]["front_teeth"])
        rt = int(phys_cfg["drive"]["gearing"]["rear_teeth"])
        return 90.0 * 2.0 * math.pi / 60.0 * (float(ft) / float(rt)) * r_eff


def _configure_sp4_mjwarp_model(m: Any, device: str, *, disable_contacts: bool = False) -> Any:
    """Configure `mujoco_warp` model solver tolerances (keeping crash geom contacts active by default)."""
    from mujoco_warp._src import types as mjw_types

    if disable_contacts:
        m.geom_contype[:] = 0
        m.geom_conaffinity[:] = 0
    m_wp = mjwarp.put_model(m)
    m_wp.opt.disableflags |= int(mjw_types.DisableBit.MULTICCD)
    if disable_contacts:
        m_wp.opt.disableflags |= int(mjw_types.DisableBit.CONTACT)
        m_wp.opt.run_collision_detection = False
    else:
        m_wp.opt.run_collision_detection = True
    # Keep ConeType matching the MuJoCo XML (PYRAMIDAL) across both Metal and CUDA so
    # contact monitor forces and solver Hessian assembly match CPU & Metal.
    # Tighten Newton line-search directional derivative tolerance from 1e-2 to 1e-3
    # while keeping opt.tolerance=1e-6 above the FP32 roundoff floor so stiff
    # closed-loop drivetrain/pedal constraints resolve cleanly across S1-S4.
    ls_tol = getattr(m_wp.opt, "ls_tolerance", None)
    if ls_tol is not None:
        target_ls_tol = min(float(m.opt.ls_tolerance), 1e-3)
        if isinstance(ls_tol, wp.array):
            ls_tol.assign(
                wp.array(
                    np.full(ls_tol.shape, target_ls_tol, dtype=np.float32),
                    dtype=wp.float32,
                    device=device,
                )
            )
        else:
            m_wp.opt.ls_tolerance = float(target_ls_tol)
    return m_wp


def _disable_contacts_for_sp4(m: Any, device: str) -> Any:
    """Compatibility alias forwarding to ``_configure_sp4_mjwarp_model`` with crash contacts enabled."""
    return _configure_sp4_mjwarp_model(m, device=device, disable_contacts=False)


def _build_sp4_config_bytes(
    lib: ctypes.CDLL,
    m_mj: Any,
    nworld: int,
    surface_label: int = 1,
    v0: float = 0.0,
    human_torque_nm: float | None = None,
    motor_torque_nm: float | None = None,
    fork_psi_scale: np.ndarray | None = None,
    rebound_scale: np.ndarray | None = None,
    shock_rate_scale: np.ndarray | None = None,
    tire_stiff_scale: np.ndarray | None = None,
    physics_config_json: dict[str, Any] | Path | None = None,
) -> bytes:
    phys_dict: dict[str, Any] | None = None
    if isinstance(physics_config_json, Path):
        if physics_config_json.is_file():
            phys_dict = json.loads(physics_config_json.read_text(encoding="utf-8"))
    elif isinstance(physics_config_json, dict):
        phys_dict = physics_config_json

    if (
        fork_psi_scale is not None
        or rebound_scale is not None
        or shock_rate_scale is not None
        or tire_stiff_scale is not None
    ):
        raw = build_sp3_sweep_configs_bytes(
            lib=lib,
            nworld=nworld,
            m_mj=m_mj,
            fork_psi_scale=fork_psi_scale,
            rebound_scale=rebound_scale,
            shock_rate_scale=shock_rate_scale,
            tire_stiff_scale=tire_stiff_scale,
        )
        buf = ctypes.create_string_buffer(raw)
        arr = (GpuSp3ConfigStruct * int(nworld)).from_buffer(buf)
        if phys_dict is not None:
            for w in range(int(nworld)):
                apply_physics_config_json_to_sp3_config(arr[w], phys_dict)
    else:
        base = build_sp3_config_struct_from_mjmodel(lib, m_mj)
        if phys_dict is not None:
            apply_physics_config_json_to_sp3_config(base, phys_dict)
        cfg_size = int(lib.bike_gpu_sizeof_sp3_config())
        buf = ctypes.create_string_buffer(int(nworld) * cfg_size)
        arr = (GpuSp3ConfigStruct * int(nworld)).from_buffer(buf)
        for w in range(int(nworld)):
            arr[w] = base

    for w in range(int(nworld)):
        arr[w].default_surface_label = int(surface_label)
        arr[w].apply_direct_crank_torque = 0
        arr[w].arm_standstill_start = 1 if float(v0) <= 0.0 else 0
        arr[w].seed_lean_from_launch = 1 if float(v0) <= 0.0 else 0
        if human_torque_nm is not None:
            arr[w].caller_human_torque_nm = float(human_torque_nm)
        if motor_torque_nm is not None:
            arr[w].caller_motor_torque_nm = float(motor_torque_nm)
    return buf.raw


def _resolve_tendon_range_binding(m_wp: Any, nworld: int) -> tuple[int, int]:
    """Return `(tendon_range_ptr, tendon_range_world_stride)` for `m_wp.tendon_range`."""
    tendon_range = getattr(m_wp, "tendon_range", None)
    if tendon_range is None:
        return 0, 0
    ptr = int(tendon_range.ptr)
    shape = getattr(tendon_range, "shape", ())
    if len(shape) >= 2 and int(shape[0]) >= nworld:
        return ptr, int(shape[1]) * 2
    if len(shape) == 1 and int(shape[0]) >= nworld:
        return ptr, 2
    if nworld == 1:
        return ptr, 0
    return 0, 0


def _resolve_dof_frictionloss_binding(m_wp: Any, nworld: int) -> tuple[int, int]:
    """Return `(dof_frictionloss_ptr, world_stride)` for `m_wp.dof_frictionloss`."""
    fl = getattr(m_wp, "dof_frictionloss", None)
    if fl is None:
        return 0, 0
    shape = getattr(fl, "shape", ())
    if len(shape) >= 2 and int(shape[0]) >= nworld:
        return int(fl.ptr), int(shape[1])
    if nworld == 1 and len(shape) >= 2:
        return int(fl.ptr), int(shape[1])
    if nworld == 1 and len(shape) == 1 and int(shape[0]) > 0:
        return int(fl.ptr), 0
    return 0, 0


def _wp_ptr(arr: Any) -> int:
    if arr is None:
        return 0
    ptr = getattr(arr, "ptr", None)
    return int(ptr) if ptr is not None else 0


def _resolve_sp3_post_solve_bindings(d_b: Any) -> dict[str, int]:
    """Extract device pointers and exact strides from a `mujoco_warp.Data` instance."""
    sd = getattr(d_b, "sensordata", None)
    nsensordata = int(sd.shape[1]) if sd is not None and len(getattr(sd, "shape", ())) >= 2 else 0
    sensordata_ptr = _wp_ptr(sd) if nsensordata > 0 else 0

    efc = getattr(d_b, "efc", None)
    nefc = getattr(d_b, "nefc", None)
    efc_type = getattr(efc, "type", None) if efc is not None else None
    efc_id = getattr(efc, "id", None) if efc is not None else None
    efc_J = getattr(efc, "J", None) if efc is not None else None
    efc_force = getattr(efc, "force", None) if efc is not None else None

    njmax = int(efc_type.shape[1]) if efc_type is not None and len(efc_type.shape) >= 2 else 0
    efc_J_row_stride = int(efc_J.shape[2]) if efc_J is not None and len(efc_J.shape) >= 3 else 0

    contact = getattr(d_b, "contact", None)
    nacon = getattr(d_b, "nacon", None)
    c_wid = getattr(contact, "worldid", None) if contact is not None else None
    c_geom = getattr(contact, "geom", None) if contact is not None else None
    c_adr = getattr(contact, "efc_address", None) if contact is not None else None
    c_dim = getattr(contact, "dim", None) if contact is not None else None
    nconmax = int(c_geom.shape[0]) if c_geom is not None and len(c_geom.shape) >= 1 else 0
    contact_efc_stride = (
        int(c_adr.shape[1]) if c_adr is not None and len(c_adr.shape) >= 2 else 1
    )

    return {
        "nsensordata": nsensordata,
        "sensordata_ptr": sensordata_ptr,
        "nefc_ptr": _wp_ptr(nefc),
        "njmax": njmax,
        "efc_J_row_stride": efc_J_row_stride,
        "efc_type_ptr": _wp_ptr(efc_type),
        "efc_id_ptr": _wp_ptr(efc_id),
        "efc_J_ptr": _wp_ptr(efc_J),
        "efc_force_ptr": _wp_ptr(efc_force),
        "nacon_ptr": _wp_ptr(nacon),
        "nconmax": nconmax,
        "contact_efc_stride": contact_efc_stride,
        "contact_worldid_ptr": _wp_ptr(c_wid),
        "contact_geom_ptr": _wp_ptr(c_geom),
        "contact_efc_address_ptr": _wp_ptr(c_adr),
        "contact_dim_ptr": _wp_ptr(c_dim),
    }


def _prepare_initial_qpos_qvel(m: Any, xml_path: Path | str) -> tuple[np.ndarray, np.ndarray]:
    """Return the captured initial `(qpos0, qvel0)` for `m` from `captured_bootstrap.json`.

    `xml_path` must be the model XML that `m` was loaded from. Errors (missing capture,
    nq/nv mismatch) propagate; there is no fallback state.
    """
    return load_captured_initial_state(xml_path, int(m.nq), int(m.nv))


def _prepare_initial_mjdata(m: Any, xml_path: Path | str) -> Any:
    """Create an initial `mujoco.MjData` at the captured state and run forward kinematics."""
    d = mujoco.MjData(m)
    qpos0, qvel0 = _prepare_initial_qpos_qvel(m, xml_path)
    d.qpos[:] = qpos0
    d.qvel[:] = qvel0
    mujoco.mj_forward(m, d)
    return d


def stage_welded_fixture_assets(welded_xml_path: Path | str, out_dir: Path | str) -> Path:
    """Stage `welded.xml` (`nq = 27, dt = 1.25 ms`) with `hfield_data.bin` and kinematics-valid `captured_bootstrap.json`.

    Matches `TempWeldedAssetDir` + `build_welded_initial_state` in `native/tests/gpu/test_gpu_batch_runners.cpp`:
      - Exports the model's flat `z = 0` normalized heightfield (`m.hfield_data`) as little-endian float64 `hfield_data.bin`.
      - Advances both `root_x` and `rider_root_x` to `2.0 m` (preserving bike-rider `connect` equality constraints),
        compresses both tyres by `15 mm` against `z = 0`, and initializes forward road/wheel/crank/drive-shaft speeds.
    """
    import shutil

    src_xml = Path(welded_xml_path).resolve()
    dst_dir = Path(out_dir).resolve()
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst_xml = dst_dir / src_xml.name
    shutil.copy2(src_xml, dst_xml)

    m = mujoco.MjModel.from_xml_path(str(dst_xml))
    if int(m.nhfielddata) > 0:
        hf_id = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_HFIELD, "road"))
        if hf_id < 0:
            hf_id = 0
        terrain_geom = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "terrain"))
        pos_z = float(m.geom_pos[terrain_geom, 2]) if terrain_geom >= 0 else 0.0
        sz = float(m.hfield_size[hf_id, 2])
        flat_norm = float(np.float32((0.0 - pos_z) / sz)) if sz > 0.0 else 0.0
        m.hfield_data[:] = flat_norm
        hf_f64 = np.asarray(m.hfield_data, dtype="<f8")
        (dst_dir / "hfield_data.bin").write_bytes(hf_f64.tobytes())

    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)

    def _jid(name: str) -> int:
        return int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name))

    rx = _jid("root_x")
    rz = _jid("root_z")
    rrx = _jid("rider_root_x")
    rrz = _jid("rider_root_z")
    fw_spin = _jid("front_wheel_spin")
    rw_spin = _jid("rear_wheel_spin")
    cr_spin = _jid("crank_spin")
    ds_spin = _jid("drive_shaft_spin")

    if rx >= 0:
        d.qpos[int(m.jnt_qposadr[rx])] = 2.0
    if rrx >= 0:
        d.qpos[int(m.jnt_qposadr[rrx])] = 2.0
    mujoco.mj_forward(m, d)

    f_body = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "front_wheel"))
    r_body = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "rear_wheel"))
    f_geom = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "front_contact"))
    if f_geom < 0:
        f_geom = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "geom_front_contact"))
    r_geom = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "rear_contact"))
    if r_geom < 0:
        r_geom = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "geom_rear_contact"))

    f_r = float(m.geom_size[f_geom, 0])
    r_r = float(m.geom_size[r_geom, 0])
    min_clear = min(float(d.xpos[f_body, 2]) - f_r, float(d.xpos[r_body, 2]) - r_r)
    dz = -0.015 - min_clear
    if rz >= 0:
        d.qpos[int(m.jnt_qposadr[rz])] += dz
    if rrz >= 0:
        d.qpos[int(m.jnt_qposadr[rrz])] += dz
    if rx >= 0:
        d.qvel[int(m.jnt_dofadr[rx])] = 5.0
    if rrx >= 0:
        d.qvel[int(m.jnt_dofadr[rrx])] = 5.0
    if fw_spin >= 0:
        d.qvel[int(m.jnt_dofadr[fw_spin])] = 5.0 / f_r
    if rw_spin >= 0:
        d.qvel[int(m.jnt_dofadr[rw_spin])] = 5.0 / r_r
    if cr_spin >= 0:
        d.qvel[int(m.jnt_dofadr[cr_spin])] = 6.0
    if ds_spin >= 0:
        d.qvel[int(m.jnt_dofadr[ds_spin])] = 6.0
    mujoco.mj_forward(m, d)

    bootstrap = {
        "state": {
            "model_dims": {"nq": int(m.nq), "nv": int(m.nv)},
            "integration_state": [0.0] + d.qpos.tolist() + d.qvel.tolist(),
        }
    }
    (dst_dir / "captured_bootstrap.json").write_text(
        json.dumps(bootstrap, indent=2) + "\n", encoding="utf-8"
    )
    return dst_xml


def _summarize_distribution(values: np.ndarray) -> dict[str, float]:
    arr = np.asarray(values, dtype=np.float64)
    return {
        "min": float(np.min(arr)),
        "p10": float(np.percentile(arr, 10.0)),
        "median": float(np.percentile(arr, 50.0)),
        "mean": float(np.mean(arr)),
        "p90": float(np.percentile(arr, 90.0)),
        "max": float(np.max(arr)),
        "std": float(np.std(arr)),
    }


def generate_parameter_sweep_scales(
    nworld: int = 2048,
    seed: int = 42,
) -> dict[str, np.ndarray]:
    """Generate heterogeneous per-world parameter scales across `nworld` worlds.

    Ranges:
      - `fork_psi_scale` in `[0.8, 1.25]`
      - `rebound_scale` in `[0.7, 1.4]`
      - `shock_rate_scale` in `[0.85, 1.20]`
      - `tire_stiff_scale` in `[0.85, 1.20]`
    """
    rng = np.random.default_rng(seed)
    fork_psi_scale = rng.uniform(0.80, 1.25, size=nworld).astype(np.float32)
    rebound_scale = rng.uniform(0.70, 1.40, size=nworld).astype(np.float32)
    shock_rate_scale = rng.uniform(0.85, 1.20, size=nworld).astype(np.float32)
    tire_stiff_scale = rng.uniform(0.85, 1.20, size=nworld).astype(np.float32)

    if nworld >= 1:
        # World 0: nominal baseline
        fork_psi_scale[0] = 1.0
        rebound_scale[0] = 1.0
        shock_rate_scale[0] = 1.0
        tire_stiff_scale[0] = 1.0
    if nworld >= 2:
        # World 1: softest corner
        fork_psi_scale[1] = 0.80
        rebound_scale[1] = 0.70
        shock_rate_scale[1] = 0.85
        tire_stiff_scale[1] = 0.85
    if nworld >= 3:
        # World 2: stiffest corner
        fork_psi_scale[2] = 1.25
        rebound_scale[2] = 1.40
        shock_rate_scale[2] = 1.20
        tire_stiff_scale[2] = 1.20

    return {
        "fork_psi_scale": np.ascontiguousarray(fork_psi_scale, dtype=np.float32),
        "rebound_scale": np.ascontiguousarray(rebound_scale, dtype=np.float32),
        "shock_rate_scale": np.ascontiguousarray(shock_rate_scale, dtype=np.float32),
        "tire_stiff_scale": np.ascontiguousarray(tire_stiff_scale, dtype=np.float32),
    }


def run_mjwarp_point(
    xml_path: Path | str = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    warmup_steps: int = 25,
    seed: int = 42,
    peak_sample_stride: int = 25,
    lib: ctypes.CDLL | None = None,
    device: str = "cuda:0",
) -> dict[str, Any]:
    """Run a single mujoco_warp batch benchmark point after loading `MjModel` + `hfield_data.bin`."""
    resolved_xml = Path(xml_path)
    _ = load_model_with_assets(resolved_xml)
    return run_parameter_sweep(
        model_xml_path=resolved_xml,
        nworld=nworld,
        n_steps=n_steps,
        warmup_steps=warmup_steps,
        seed=seed,
        peak_sample_stride=peak_sample_stride,
        lib=lib,
        device=device,
    )


def run_parameter_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    warmup_steps: int = 25,
    seed: int = 42,
    peak_sample_stride: int = 25,
    lib: ctypes.CDLL | None = None,
    host_lib: ctypes.CDLL | None = None,
    device: str = "cuda:0",
    backend: str = "compliant_2d",
    station_count: int = 64,
) -> dict[str, Any]:
    """Experiment A: Heterogeneous 2048-world parameter sweep via clean_split CUDA Graph."""
    _require_gpu_stack()
    if lib is None:
        lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)
    if host_lib is None:
        try:
            from tools.colab_gpu.cpu_batch_sweep import compile_cpu_batch_runner
        except ImportError:
            from cpu_batch_sweep import compile_cpu_batch_runner  # type: ignore[no-redef]
        host_lib = compile_cpu_batch_runner()

    m = load_model_with_assets(model_xml_path)
    _ensure_newton_solver(m)
    d0 = _prepare_initial_mjdata(m, model_xml_path)
    dt_s = float(m.opt.timestep)
    ops = _resolve_clean_split_ops(mjwarp)

    idx_bytes = resolve_gpu_model_indices(m)
    idx_struct = GpuModelIndicesStruct.from_buffer_copy(idx_bytes)
    scales = generate_parameter_sweep_scales(nworld=nworld, seed=seed)
    configs_bytes = build_sweep_configs_bytes(
        lib=lib,
        nworld=nworld,
        fork_psi_scale=scales["fork_psi_scale"],
        rebound_scale=scales["rebound_scale"],
        shock_rate_scale=scales["shock_rate_scale"],
        tire_stiff_scale=scales["tire_stiff_scale"],
    )
    sizeof_state = int(lib.bike_gpu_sizeof_world_state())

    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_configs = _allocate_and_copy_device_bytes(configs_bytes, device=device)
    d_states = wp.zeros(nworld * sizeof_state, dtype=wp.uint8, device=device)
    (
        d_tyre_model,
        d_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        terrain_f32_ptr,
        terrain_i32_ptr,
    ) = _upload_tyre_and_terrain_device(
        host_lib, model_xml_path, backend, station_count, idx_bytes, device=device
    )

    m_wp = mjwarp.put_model(m)
    if m.ntendon > 0 and hasattr(m_wp, "tendon_range"):
        tr_np = np.tile(m.tendon_range.astype(np.float32)[None, :, :], (nworld, 1, 1))
        m_wp.tendon_range = wp.array(tr_np, dtype=wp.vec2f, device=device)
    d_b = mjwarp.put_data(m, d0, nworld=nworld)
    mjwarp.forward(m_wp, d_b)

    cuda_stream = wp.get_stream().cuda_stream
    tendon_range_ptr, tendon_range_stride = _resolve_tendon_range_binding(m_wp, nworld)

    def _init_states(data_b: Any) -> None:
        rc = int(
            lib.bike_gpu_launch_init_states(
                ctypes.c_void_p(cuda_stream),
                int(nworld),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_configs.ptr),
                1,
                ctypes.c_void_p(d_states.ptr),
                ctypes.c_void_p(data_b.qpos.ptr),
            )
        )
        if rc != 0:
            raise RuntimeError(f"bike_gpu_launch_init_states failed with code {rc}")

    def _make_writer_fn(data_b: Any) -> Callable[[], None]:
        def _launch_writers() -> None:
            rc = int(
                lib.bike_gpu_launch_writers(
                    ctypes.c_void_p(cuda_stream),
                    int(nworld),
                    ctypes.c_float(dt_s),
                    ctypes.c_void_p(d_idx.ptr),
                    ctypes.c_void_p(d_tyre_model.ptr),
                    ctypes.c_void_p(d_terrain_desc.ptr),
                    ctypes.c_void_p(terrain_f32_ptr),
                    ctypes.c_void_p(terrain_i32_ptr),
                    ctypes.c_void_p(d_configs.ptr),
                    1,
                    ctypes.c_void_p(d_states.ptr),
                    ctypes.c_void_p(data_b.qpos.ptr),
                    ctypes.c_void_p(data_b.qvel.ptr),
                    ctypes.c_void_p(data_b.xpos.ptr),
                    ctypes.c_void_p(data_b.xmat.ptr),
                    ctypes.c_void_p(data_b.xanchor.ptr),
                    ctypes.c_void_p(data_b.xaxis.ptr),
                    ctypes.c_void_p(data_b.qfrc_applied.ptr),
                    ctypes.c_void_p(data_b.ctrl.ptr),
                    ctypes.c_void_p(tendon_range_ptr),
                    int(tendon_range_stride),
                )
            )
            if rc != 0:
                raise RuntimeError(f"bike_gpu_launch_writers failed with code {rc}")

        return _launch_writers

    def _capture_step(data_b: Any) -> Any:
        with wp.ScopedCapture() as capture:
            _mjwarp_clean_split_step(ops, m_wp, data_b, _make_writer_fn(data_b))
        return capture.graph

    # Run 1 uncaptured step prior to CUDA Graph capture so eager device struct +
    # monotonic terrain validation executes outside stream capture.
    _init_states(d_b)
    wp.synchronize()
    _mjwarp_clean_split_step(ops, m_wp, d_b, _make_writer_fn(d_b))
    wp.synchronize()

    d_b = mjwarp.put_data(m, d0, nworld=nworld)
    mjwarp.forward(m_wp, d_b)
    _init_states(d_b)
    wp.synchronize()
    graph = _capture_step(d_b)

    for _ in range(warmup_steps):
        wp.capture_launch(graph)
    wp.synchronize()

    d_b = mjwarp.put_data(m, d0, nworld=nworld)
    mjwarp.forward(m_wp, d_b)
    _init_states(d_b)
    graph = _capture_step(d_b)
    wp.synchronize()

    t0 = time.perf_counter()
    for _ in range(n_steps):
        wp.capture_launch(graph)
    wp.synchronize()
    wall_time_s = time.perf_counter() - t0

    d_b = mjwarp.put_data(m, d0, nworld=nworld)
    mjwarp.forward(m_wp, d_b)
    _init_states(d_b)
    graph = _capture_step(d_b)

    fork_qpos_idx = int(idx_struct.fork_travel_qpos)
    shock_qpos_idx = int(idx_struct.shock_stroke_qpos)
    root_x_qpos_idx = int(idx_struct.root_x_qpos)

    peak_fork_mm = np.zeros(nworld, dtype=np.float32)
    peak_shock_mm = np.zeros(nworld, dtype=np.float32)
    stride = max(1, int(peak_sample_stride))

    steps_done = 0
    while steps_done < n_steps:
        chunk = min(stride, n_steps - steps_done)
        for _ in range(chunk):
            wp.capture_launch(graph)
        wp.synchronize()
        qpos_host = d_b.qpos.numpy()
        peak_fork_mm = np.maximum(peak_fork_mm, qpos_host[:, fork_qpos_idx] * 1000.0)
        peak_shock_mm = np.maximum(peak_shock_mm, qpos_host[:, shock_qpos_idx] * 1000.0)
        steps_done += chunk

    final_qpos = d_b.qpos.numpy()
    forward_distance_m = final_qpos[:, root_x_qpos_idx] - float(d0.qpos[root_x_qpos_idx])
    finite_all = bool(np.all(np.isfinite(final_qpos)))

    total_world_steps = int(nworld) * int(n_steps)
    us_per_step_per_world = (wall_time_s * 1e6) / float(total_world_steps)
    steps_per_s = float(total_world_steps) / max(wall_time_s, 1e-12)

    return {
        "experiment": "A_parameter_sweep",
        "schedule": "clean_split",
        "backend": backend,
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "wall_time_ms": wall_time_s * 1000.0,
        "us_per_step_per_world": us_per_step_per_world,
        "steps_per_s": steps_per_s,
        "all_finite": finite_all,
        "sweep_ranges": {
            "fork_psi_scale": [0.80, 1.25],
            "rebound_scale": [0.70, 1.40],
            "shock_rate_scale": [0.85, 1.20],
            "tire_stiff_scale": [0.85, 1.20],
        },
        "metrics": {
            "peak_fork_travel_mm": _summarize_distribution(peak_fork_mm),
            "peak_shock_stroke_mm": _summarize_distribution(peak_shock_mm),
            "forward_distance_root_x_m": _summarize_distribution(forward_distance_m),
        },
    }


def run_tyre_backend_scaling_benchmark(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 500,
    warmup_steps: int = 20,
    lib: ctypes.CDLL | None = None,
    host_lib: ctypes.CDLL | None = None,
    device: str = "cuda:0",
) -> dict[str, Any]:
    """Experiment B: Tyre backend scaling across compliant_2d and distributed N=32/64/128."""
    _require_gpu_stack()
    if lib is None:
        lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)
    if host_lib is None:
        try:
            from tools.colab_gpu.cpu_batch_sweep import compile_cpu_batch_runner
        except ImportError:
            from cpu_batch_sweep import compile_cpu_batch_runner  # type: ignore[no-redef]
        host_lib = compile_cpu_batch_runner()

    m = load_model_with_assets(model_xml_path)
    _ensure_newton_solver(m)
    d0 = _prepare_initial_mjdata(m, model_xml_path)
    dt_s = float(m.opt.timestep)
    ops = _resolve_clean_split_ops(mjwarp)

    idx_bytes = resolve_gpu_model_indices(m)
    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    sizeof_state = int(lib.bike_gpu_sizeof_world_state())

    m_wp = mjwarp.put_model(m)
    if m.ntendon > 0 and hasattr(m_wp, "tendon_range"):
        tr_np = np.tile(m.tendon_range.astype(np.float32)[None, :, :], (nworld, 1, 1))
        m_wp.tendon_range = wp.array(tr_np, dtype=wp.vec2f, device=device)
    cuda_stream = wp.get_stream().cuda_stream
    tendon_range_ptr, tendon_range_stride = _resolve_tendon_range_binding(m_wp, nworld)

    configs_bytes = build_sweep_configs_bytes(lib=lib, nworld=1)
    d_configs = _allocate_and_copy_device_bytes(configs_bytes, device=device)
    d_states = wp.zeros(nworld * sizeof_state, dtype=wp.uint8, device=device)

    tier_results: list[dict[str, Any]] = []
    baseline_us: float | None = None

    for tier in TYRE_BACKEND_TIERS:
        backend = str(tier["backend"])
        station_count = int(tier["station_count"])
        units_per_world = int(tier["units_per_world"])
        (
            d_tyre_model,
            d_terrain_desc,
            _d_terrain_f32,
            _d_terrain_i32,
            terrain_f32_ptr,
            terrain_i32_ptr,
        ) = _upload_tyre_and_terrain_device(
            host_lib, model_xml_path, backend, station_count, idx_bytes, device=device
        )

        d_b = mjwarp.put_data(m, d0, nworld=nworld)
        mjwarp.forward(m_wp, d_b)

        rc_init = int(
            lib.bike_gpu_launch_init_states(
                ctypes.c_void_p(cuda_stream),
                int(nworld),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_configs.ptr),
                0,
                ctypes.c_void_p(d_states.ptr),
                ctypes.c_void_p(d_b.qpos.ptr),
            )
        )
        if rc_init != 0:
            raise RuntimeError(f"bike_gpu_launch_init_states failed with code {rc_init}")
        wp.synchronize()

        def _make_tier_writer_fn(data_b: Any) -> Callable[[], None]:
            def _launch_writers() -> None:
                rc = int(
                    lib.bike_gpu_launch_writers(
                        ctypes.c_void_p(cuda_stream),
                        int(nworld),
                        ctypes.c_float(dt_s),
                        ctypes.c_void_p(d_idx.ptr),
                        ctypes.c_void_p(d_tyre_model.ptr),
                        ctypes.c_void_p(d_terrain_desc.ptr),
                        ctypes.c_void_p(terrain_f32_ptr),
                        ctypes.c_void_p(terrain_i32_ptr),
                        ctypes.c_void_p(d_configs.ptr),
                        0,
                        ctypes.c_void_p(d_states.ptr),
                        ctypes.c_void_p(data_b.qpos.ptr),
                        ctypes.c_void_p(data_b.qvel.ptr),
                        ctypes.c_void_p(data_b.xpos.ptr),
                        ctypes.c_void_p(data_b.xmat.ptr),
                        ctypes.c_void_p(data_b.xanchor.ptr),
                        ctypes.c_void_p(data_b.xaxis.ptr),
                        ctypes.c_void_p(data_b.qfrc_applied.ptr),
                        ctypes.c_void_p(data_b.ctrl.ptr),
                        ctypes.c_void_p(tendon_range_ptr),
                        int(tendon_range_stride),
                    )
                )
                if rc != 0:
                    raise RuntimeError(f"bike_gpu_launch_writers failed with code {rc}")

            return _launch_writers

        # Run 1 uncaptured step prior to CUDA Graph capture for eager struct + terrain validation.
        _mjwarp_clean_split_step(ops, m_wp, d_b, _make_tier_writer_fn(d_b))
        wp.synchronize()

        d_b = mjwarp.put_data(m, d0, nworld=nworld)
        mjwarp.forward(m_wp, d_b)
        rc_init = int(
            lib.bike_gpu_launch_init_states(
                ctypes.c_void_p(cuda_stream),
                int(nworld),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_configs.ptr),
                0,
                ctypes.c_void_p(d_states.ptr),
                ctypes.c_void_p(d_b.qpos.ptr),
            )
        )
        if rc_init != 0:
            raise RuntimeError(f"bike_gpu_launch_init_states failed with code {rc_init}")
        wp.synchronize()

        with wp.ScopedCapture() as capture:
            _mjwarp_clean_split_step(ops, m_wp, d_b, _make_tier_writer_fn(d_b))
        graph = capture.graph

        for _ in range(warmup_steps):
            wp.capture_launch(graph)
        wp.synchronize()

        t0 = time.perf_counter()
        for _ in range(n_steps):
            wp.capture_launch(graph)
        wp.synchronize()
        wall_time_s = time.perf_counter() - t0

        qpos_host = d_b.qpos.numpy()
        finite_all = bool(np.all(np.isfinite(qpos_host)))

        total_world_steps = int(nworld) * int(n_steps)
        us_per_step_per_world = (wall_time_s * 1e6) / float(total_world_steps)
        steps_per_s = float(total_world_steps) / max(wall_time_s, 1e-12)

        if baseline_us is None:
            baseline_us = us_per_step_per_world
        delta_us = us_per_step_per_world - baseline_us

        tier_results.append(
            {
                "tier_id": tier["tier_id"],
                "label": tier["label"],
                "backend": backend,
                "station_count": station_count,
                "units_per_wheel": int(tier["units_per_wheel"]),
                "units_per_world": units_per_world,
                "wall_time_ms": wall_time_s * 1000.0,
                "us_per_step_per_world": us_per_step_per_world,
                "delta_us_vs_compliant": delta_us,
                "steps_per_s": steps_per_s,
                "all_finite": finite_all,
            }
        )

    return {
        "experiment": "B_tyre_backend_scaling",
        "schedule": "clean_split",
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "tiers": tier_results,
    }


def run_mjwarp_sp3_autonomous_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 200,
    warmup_steps: int = 10,
    seed: int = 42,
    use_12gears: bool = True,
    lib: ctypes.CDLL | None = None,
    host_lib: ctypes.CDLL | None = None,
    device: str = "cuda:0",
    nconmax: int = 64,
    njmax: int = 128,
    backend: str = "compliant_2d",
    station_count: int = 64,
) -> dict[str, Any]:
    """Autonomous SP3-C closed-loop sweep inside `wp.ScopedCapture()` on CUDA."""
    _require_gpu_stack()
    if lib is None:
        lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)
    if host_lib is None:
        try:
            from tools.colab_gpu.cpu_batch_sweep import compile_cpu_batch_runner
        except ImportError:
            from cpu_batch_sweep import compile_cpu_batch_runner  # type: ignore[no-redef]
        host_lib = compile_cpu_batch_runner()

    with tempfile.TemporaryDirectory(prefix="bike-sp3-cuda-") as sp3_tmp:
        resolved_xml = Path(model_xml_path).resolve()
        if use_12gears:
            xml12 = Path(sp3_tmp) / "model_12gears.xml"
            m_src = load_model_with_assets(resolved_xml)
            if int(m_src.nhfielddata) > 0:
                (Path(sp3_tmp) / "hfield_data.bin").write_bytes(
                    np.asarray(m_src.hfield_data, dtype="<f8").tobytes()
                )
            cb = resolved_xml.parent / "captured_bootstrap.json"
            if cb.is_file():
                (Path(sp3_tmp) / "captured_bootstrap.json").write_text(
                    cb.read_text(encoding="utf-8"), encoding="utf-8"
                )
            resolved_xml = materialize_model_12gears_xml(resolved_xml, xml12)

        m = load_model_with_assets(resolved_xml)
        _ensure_newton_solver(m)
        d0 = _prepare_initial_mjdata(
            m, model_xml_path if not use_12gears else resolved_xml
        )
        dt_s = float(m.opt.timestep)
        ops = _resolve_clean_split_ops(mjwarp)

        idx_bytes = resolve_gpu_model_indices(m)
        idx_struct = GpuModelIndicesStruct.from_buffer_copy(idx_bytes)
        scales = generate_parameter_sweep_scales(nworld=nworld, seed=seed)
        sp3_configs_bytes = build_sp3_sweep_configs_bytes(
            lib=lib,
            nworld=nworld,
            m_mj=m,
            fork_psi_scale=scales["fork_psi_scale"],
            rebound_scale=scales["rebound_scale"],
            shock_rate_scale=scales["shock_rate_scale"],
            tire_stiff_scale=scales["tire_stiff_scale"],
        )
        sizeof_ws = int(lib.bike_gpu_sizeof_world_state())
        sizeof_sp3 = int(lib.bike_gpu_sizeof_sp3_world_state())

        d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
        d_sp3_configs = _allocate_and_copy_device_bytes(sp3_configs_bytes, device=device)
        d_world_states = wp.zeros(nworld * sizeof_ws, dtype=wp.uint8, device=device)
        d_sp3_states = wp.zeros(nworld * sizeof_sp3, dtype=wp.uint8, device=device)
        (
            d_tyre_model,
            d_terrain_desc,
            _d_tf32,
            _d_ti32,
            terrain_f32_ptr,
            terrain_i32_ptr,
        ) = _upload_tyre_and_terrain_device(
            host_lib, resolved_xml, backend, station_count, idx_bytes, device=device
        )

        m_wp = mjwarp.put_model(m)
        expand_mjwarp_tendon_range_and_frictionloss(m, m_wp, nworld, device=device)
        init_tr_np = m_wp.tendon_range.numpy().copy()
        init_fl_np = m_wp.dof_frictionloss.numpy().copy()
        eff_nconmax = max(int(nconmax), int(d0.ncon) + 32)
        eff_njmax = max(int(njmax), int(d0.nefc) + 64)
        d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=eff_nconmax, njmax=eff_njmax)
        mjwarp.forward(m_wp, d_b)
        wp.synchronize()

        cuda_stream = wp.get_stream().cuda_stream
        tr_ptr, tr_stride = _resolve_tendon_range_binding(m_wp, nworld)
        fl_ptr, fl_stride = _resolve_dof_frictionloss_binding(m_wp, nworld)
        post_b = _resolve_sp3_post_solve_bindings(d_b)

        def _init_sp3_states() -> None:
            d_world_states.zero_()
            d_sp3_states.zero_()
            m_wp.tendon_range.assign(
                wp.array(init_tr_np, dtype=m_wp.tendon_range.dtype, device=device)
            )
            m_wp.dof_frictionloss.assign(
                wp.array(init_fl_np, dtype=m_wp.dof_frictionloss.dtype, device=device)
            )
            assert int(m_wp.tendon_range.ptr) == tr_ptr
            assert int(m_wp.dof_frictionloss.ptr) == fl_ptr
            d_b.qpos.assign(
                wp.array(
                    np.tile(d0.qpos.astype(np.float32)[None, :], (nworld, 1)),
                    dtype=wp.float32,
                    device=device,
                )
            )
            d_b.qvel.assign(
                wp.array(
                    np.tile(d0.qvel.astype(np.float32)[None, :], (nworld, 1)),
                    dtype=wp.float32,
                    device=device,
                )
            )
            d_b.qacc_warmstart.zero_()
            d_b.ctrl.zero_()
            d_b.qfrc_applied.zero_()
            d_b.time.zero_()
            if int(m.na) > 0:
                d_b.act.zero_()
            mjwarp.forward(m_wp, d_b)
            wp.synchronize()
            rc = int(
                lib.bike_gpu_launch_sp3_init(
                    ctypes.c_void_p(cuda_stream),
                    int(nworld),
                    ctypes.c_void_p(d_idx.ptr),
                    ctypes.c_void_p(d_sp3_configs.ptr),
                    1,
                    ctypes.c_void_p(d_sp3_states.ptr),
                    ctypes.c_void_p(d_world_states.ptr),
                    ctypes.c_void_p(d_b.qpos.ptr),
                    ctypes.c_float(0.0),
                    int(m.ntendon),
                    ctypes.c_void_p(tr_ptr),
                    int(tr_stride),
                )
            )
            if rc != 0:
                raise RuntimeError(f"bike_gpu_launch_sp3_init failed with code {rc}")
            wp.synchronize()

        _init_sp3_states()

        def _pre() -> None:
            r = int(
                lib.bike_gpu_launch_sp3_pre_solve(
                    ctypes.c_void_p(cuda_stream),
                    int(nworld),
                    ctypes.c_float(dt_s),
                    ctypes.c_void_p(d_idx.ptr),
                    ctypes.c_void_p(d_tyre_model.ptr),
                    ctypes.c_void_p(d_terrain_desc.ptr),
                    ctypes.c_void_p(terrain_f32_ptr),
                    ctypes.c_void_p(terrain_i32_ptr),
                    ctypes.c_void_p(d_sp3_configs.ptr),
                    1,
                    ctypes.c_void_p(d_sp3_states.ptr),
                    ctypes.c_void_p(d_world_states.ptr),
                    ctypes.c_void_p(d_b.qpos.ptr),
                    ctypes.c_void_p(d_b.qvel.ptr),
                    ctypes.c_void_p(d_b.xpos.ptr),
                    ctypes.c_void_p(d_b.xmat.ptr),
                    ctypes.c_void_p(d_b.xanchor.ptr),
                    ctypes.c_void_p(d_b.xaxis.ptr),
                    ctypes.c_void_p(d_b.qfrc_applied.ptr),
                    ctypes.c_void_p(d_b.ctrl.ptr),
                    int(m.ntendon),
                    ctypes.c_void_p(tr_ptr),
                    int(tr_stride),
                    ctypes.c_void_p(fl_ptr),
                    int(fl_stride),
                )
            )
            if r != 0:
                raise RuntimeError(f"bike_gpu_launch_sp3_pre_solve failed with code {r}")

        def _post() -> None:
            r = int(
                lib.bike_gpu_launch_sp3_post_solve(
                    ctypes.c_void_p(cuda_stream),
                    int(nworld),
                    ctypes.c_float(dt_s),
                    ctypes.c_void_p(d_idx.ptr),
                    ctypes.c_void_p(d_sp3_configs.ptr),
                    1,
                    ctypes.c_void_p(d_sp3_states.ptr),
                    ctypes.c_void_p(d_b.qpos.ptr),
                    ctypes.c_void_p(d_b.qvel.ptr),
                    ctypes.c_void_p(d_b.xmat.ptr),
                    int(post_b["nsensordata"]),
                    ctypes.c_void_p(post_b["sensordata_ptr"]),
                    ctypes.c_void_p(post_b["nefc_ptr"]),
                    int(post_b["njmax"]),
                    int(post_b["efc_J_row_stride"]),
                    ctypes.c_void_p(post_b["efc_type_ptr"]),
                    ctypes.c_void_p(post_b["efc_id_ptr"]),
                    ctypes.c_void_p(post_b["efc_J_ptr"]),
                    ctypes.c_void_p(post_b["efc_force_ptr"]),
                    ctypes.c_void_p(post_b["nacon_ptr"]),
                    int(post_b["nconmax"]),
                    int(post_b["contact_efc_stride"]),
                    ctypes.c_void_p(post_b["contact_worldid_ptr"]),
                    ctypes.c_void_p(post_b["contact_geom_ptr"]),
                    ctypes.c_void_p(post_b["contact_efc_address_ptr"]),
                    ctypes.c_void_p(post_b["contact_dim_ptr"]),
                )
            )
            if r != 0:
                raise RuntimeError(f"bike_gpu_launch_sp3_post_solve failed with code {r}")

        _mjwarp_sp3_clean_split_step(ops, m_wp, d_b, _pre, _post)
        wp.synchronize()

        _init_sp3_states()
        with wp.ScopedCapture() as capture:
            _mjwarp_sp3_clean_split_step(ops, m_wp, d_b, _pre, _post)
        graph = capture.graph
        for _ in range(warmup_steps):
            wp.capture_launch(graph)
        wp.synchronize()

        _init_sp3_states()
        t0 = time.perf_counter()
        for _ in range(n_steps):
            wp.capture_launch(graph)
        wp.synchronize()
        wall_time_s = time.perf_counter() - t0

        final_qpos = d_b.qpos.numpy()
        final_qvel = d_b.qvel.numpy()
        all_finite = bool(np.all(np.isfinite(final_qpos)) and np.all(np.isfinite(final_qvel)))
        forward_distance_m = final_qpos[:, int(idx_struct.root_x_qpos)] - float(
            d0.qpos[int(idx_struct.root_x_qpos)]
        )
        total_world_steps = int(nworld) * int(n_steps)
        return {
            "experiment": "SP3_C_autonomous",
            "schedule": "clean_split",
            "ntendon": int(m.ntendon),
            "nworld": int(nworld),
            "n_steps": int(n_steps),
            "dt_s": dt_s,
            "sizeof_sp3_world_state_bytes": sizeof_sp3,
            "wall_time_ms": wall_time_s * 1000.0,
            "us_per_step_per_world": (wall_time_s * 1e6) / float(total_world_steps),
            "steps_per_s": float(total_world_steps) / max(wall_time_s, 1e-12),
            "all_finite": all_finite,
            "forward_distance_root_x_m": _summarize_distribution(forward_distance_m),
        }


def _build_sp4_s1s4_scenarios(
    sp4_bundle_dir: Path | None = None,
) -> list[dict[str, Any]]:
    steady90_v0 = _compute_steady90_v0(sp4_bundle_dir=sp4_bundle_dir)
    return [
        {
            "id": "S1_flat",
            "track": "tests/fixtures/tracks/flat.toml",
            "duration_s": 6.0,
            "v0": 0.0,
            "surface_label": 1,  # hardpack
            "human_torque_nm": None,
            "motor_torque_nm": None,
            "m0_distance_m": M0_S1S4_REFERENCE_DISTANCES_M["S1_flat"],
        },
        {
            "id": "S2_climb15",
            "track": "tests/fixtures/tracks/climb_15pct.toml",
            "duration_s": 25.0,
            "v0": 0.0,
            "surface_label": 0,  # asphalt
            "human_torque_nm": None,
            "motor_torque_nm": None,
            "m0_distance_m": M0_S1S4_REFERENCE_DISTANCES_M["S2_climb15"],
        },
        {
            "id": "S3_steady90",
            "track": "tests/fixtures/tracks/flat.toml",
            "duration_s": 8.0,
            "v0": float(steady90_v0),
            "surface_label": 1,  # hardpack
            "human_torque_nm": 24.0,
            "motor_torque_nm": None,
            "m0_distance_m": M0_S1S4_REFERENCE_DISTANCES_M["S3_steady90"],
        },
        {
            "id": "S4_start_uphill15_60nm",
            "track": "tests/fixtures/tracks/climb_15pct.toml",
            "duration_s": 10.0,
            "v0": 0.0,
            "surface_label": 0,  # asphalt
            "human_torque_nm": 60.0,
            "motor_torque_nm": 0.0,
            "m0_distance_m": M0_S1S4_REFERENCE_DISTANCES_M["S4_start_uphill15_60nm"],
        },
    ]


def _run_cuda_sp4_batch(
    *,
    lib: ctypes.CDLL,
    host_lib: ctypes.CDLL,
    xml_path: Path,
    qpos0: np.ndarray,
    qvel0: np.ndarray,
    qacc_ws0: np.ndarray | None,
    nworld: int,
    n_steps: int,
    warmup_steps: int,
    surface_label: int,
    backend: str,
    station_count: int,
    control_period_steps: int,
    cfg_bytes: bytes,
    nconmax: int = 32,
    njmax: int = 64,
    device: str = "cuda:0",
    init_grade: float | None = None,
) -> dict[str, Any]:
    m = load_model_with_assets(xml_path)
    _ensure_newton_solver(m)
    d0 = mujoco.MjData(m)
    d0.qpos[:] = qpos0
    d0.qvel[:] = qvel0
    mujoco.mj_forward(m, d0)
    if qacc_ws0 is not None and qacc_ws0.size == int(m.nv):
        d0.qacc_warmstart[:] = qacc_ws0
    dt_s = float(m.opt.timestep)
    ops = _resolve_clean_split_ops(mjwarp)

    idx_bytes = resolve_gpu_model_indices(m)
    idx_struct = GpuModelIndicesStruct.from_buffer_copy(idx_bytes)
    spindle_bytes = build_spindle_params_bytes(
        host_lib=host_lib,
        xml_path=xml_path,
        strength_json_path=DEFAULT_STRENGTH_JSON,
        envelope_json_path=DEFAULT_ENVELOPE_JSON,
        control_period_steps=control_period_steps,
    )
    sizeof_ws = int(lib.bike_gpu_sizeof_world_state())
    sizeof_sp3 = int(lib.bike_gpu_sizeof_sp3_world_state())
    sizeof_sp4 = int(lib.bike_gpu_sizeof_sp4_world_state())

    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_sp3_configs = _allocate_and_copy_device_bytes(cfg_bytes, device=device)
    d_spindle_params = _allocate_and_copy_device_bytes(spindle_bytes, device=device)
    d_world_states = wp.zeros(nworld * sizeof_ws, dtype=wp.uint8, device=device)
    d_sp3_states = wp.zeros(nworld * sizeof_sp3, dtype=wp.uint8, device=device)
    d_sp4_states = wp.zeros(nworld * sizeof_sp4, dtype=wp.uint8, device=device)
    (
        d_tyre_model,
        d_terrain_desc,
        _d_tf32,
        _d_ti32,
        terrain_f32_ptr,
        terrain_i32_ptr,
    ) = _upload_tyre_and_terrain_device(
        host_lib,
        xml_path,
        backend,
        station_count,
        idx_bytes,
        device=device,
        surface_label=surface_label,
    )

    m_wp = _configure_sp4_mjwarp_model(m, device=device, disable_contacts=False)
    expand_mjwarp_tendon_range_and_frictionloss(m, m_wp, nworld, device=device)
    init_tr_np = m_wp.tendon_range.numpy().copy()
    init_fl_np = m_wp.dof_frictionloss.numpy().copy()
    eff_nconmax = max(int(nconmax), int(d0.ncon) + 32)
    eff_njmax = max(int(njmax), int(d0.nefc) + 32)
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=eff_nconmax, njmax=eff_njmax)
    mjwarp.forward(m_wp, d_b)
    if qacc_ws0 is not None and qacc_ws0.size == int(m.nv):
        ws_np = np.tile(qacc_ws0.astype(np.float32)[None, :], (nworld, 1))
        d_b.qacc_warmstart.assign(wp.array(ws_np, dtype=wp.float32, device=device))
    wp.synchronize()

    cuda_stream = wp.get_stream().cuda_stream
    tr_ptr, tr_stride = _resolve_tendon_range_binding(m_wp, nworld)
    fl_ptr, fl_stride = _resolve_dof_frictionloss_binding(m_wp, nworld)
    post_b = _resolve_sp3_post_solve_bindings(d_b)
    submass_arr = getattr(m_wp, "body_subtreemass", None)
    submass_ptr = int(submass_arr.ptr) if submass_arr is not None else 0
    m_rowadr_ptr = int(m_wp.M_rowadr.ptr)
    m_rownnz_ptr = int(m_wp.M_rownnz.ptr)
    m_colind_ptr = int(m_wp.M_colind.ptr)
    m_world_stride = int(d_b.M.shape[1])
    xz_ctx = _build_sp4_xz_rebase_ctx(m, nworld, device)
    cfg0 = GpuSp3ConfigStruct.from_buffer_copy(cfg_bytes)
    resolved_init_grade = (
        float(init_grade)
        if init_grade is not None
        else _polyline_slope_from_desc_and_f32(
            d_terrain_desc.numpy().tobytes(),
            _d_tf32.numpy(),
            float(qpos0[int(idx_struct.root_x_qpos)]) + float(cfg0.road_lookahead_m),
        )
    )

    def _init_states() -> None:
        if xz_ctx is not None:
            xz_ctx["xz_off"].zero_()
        d_world_states.zero_()
        d_sp3_states.zero_()
        d_sp4_states.zero_()
        m_wp.tendon_range.assign(
            wp.array(init_tr_np, dtype=m_wp.tendon_range.dtype, device=device)
        )
        m_wp.dof_frictionloss.assign(
            wp.array(init_fl_np, dtype=m_wp.dof_frictionloss.dtype, device=device)
        )
        assert int(m_wp.tendon_range.ptr) == tr_ptr
        assert int(m_wp.dof_frictionloss.ptr) == fl_ptr
        d_b.qpos.assign(
            wp.array(np.tile(qpos0.astype(np.float32)[None, :], (nworld, 1)), dtype=wp.float32, device=device)
        )
        d_b.qvel.assign(
            wp.array(np.tile(qvel0.astype(np.float32)[None, :], (nworld, 1)), dtype=wp.float32, device=device)
        )
        if qacc_ws0 is not None and qacc_ws0.size == int(m.nv):
            d_b.qacc_warmstart.assign(
                wp.array(
                    np.tile(qacc_ws0.astype(np.float32)[None, :], (nworld, 1)),
                    dtype=wp.float32,
                    device=device,
                )
            )
        else:
            d_b.qacc_warmstart.zero_()
        d_b.ctrl.zero_()
        d_b.qfrc_applied.zero_()
        d_b.time.zero_()
        if int(m.na) > 0:
            d_b.act.zero_()
        mjwarp.forward(m_wp, d_b)
        wp.synchronize()
        rc = int(
            lib.bike_gpu_launch_sp4_init(
                ctypes.c_void_p(cuda_stream),
                int(nworld),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_sp3_configs.ptr),
                1,
                ctypes.c_void_p(d_sp3_states.ptr),
                ctypes.c_void_p(d_sp4_states.ptr),
                ctypes.c_void_p(d_world_states.ptr),
                ctypes.c_void_p(d_b.qpos.ptr),
                ctypes.c_float(resolved_init_grade),
                int(m.ntendon),
                ctypes.c_void_p(tr_ptr),
                int(tr_stride),
            )
        )
        if rc != 0:
            raise RuntimeError(f"bike_gpu_launch_sp4_init failed with code {rc}")
        wp.synchronize()

    def _pre() -> None:
        r = int(
            lib.bike_gpu_launch_sp4_pre_solve_and_pass0(
                ctypes.c_void_p(cuda_stream),
                int(nworld),
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_tyre_model.ptr),
                ctypes.c_void_p(d_terrain_desc.ptr),
                ctypes.c_void_p(terrain_f32_ptr),
                ctypes.c_void_p(terrain_i32_ptr),
                ctypes.c_void_p(d_sp3_configs.ptr),
                1,
                ctypes.c_void_p(d_spindle_params.ptr),
                ctypes.c_void_p(d_sp3_states.ptr),
                ctypes.c_void_p(d_sp4_states.ptr),
                ctypes.c_void_p(d_world_states.ptr),
                ctypes.c_void_p(d_b.qpos.ptr),
                ctypes.c_void_p(d_b.qvel.ptr),
                ctypes.c_void_p(d_b.qfrc_bias.ptr),
                ctypes.c_void_p(d_b.xpos.ptr),
                ctypes.c_void_p(d_b.xmat.ptr),
                ctypes.c_void_p(d_b.xanchor.ptr),
                ctypes.c_void_p(d_b.xaxis.ptr),
                ctypes.c_void_p(d_b.site_xpos.ptr),
                ctypes.c_void_p(d_b.subtree_com.ptr),
                ctypes.c_void_p(submass_ptr),
                0,
                int(m_world_stride),
                ctypes.c_void_p(m_rowadr_ptr),
                ctypes.c_void_p(m_rownnz_ptr),
                ctypes.c_void_p(m_colind_ptr),
                ctypes.c_void_p(d_b.M.ptr),
                ctypes.c_void_p(d_b.qfrc_applied.ptr),
                ctypes.c_void_p(d_b.ctrl.ptr),
                int(m.ntendon),
                ctypes.c_void_p(tr_ptr),
                int(tr_stride),
                ctypes.c_void_p(fl_ptr),
                int(fl_stride),
                ctypes.c_void_p(d_b.qacc_warmstart.ptr),
            )
        )
        if r != 0:
            raise RuntimeError(f"bike_gpu_launch_sp4_pre_solve_and_pass0 failed: {r}")

    def _make_pass(pass_fn: Any) -> Callable[[], None]:
        def _run() -> None:
            r = int(
                pass_fn(
                    ctypes.c_void_p(cuda_stream),
                    int(nworld),
                    int(m.nq),
                    int(m.nv),
                    int(m.nu),
                    ctypes.c_void_p(d_spindle_params.ptr),
                    ctypes.c_void_p(d_sp4_states.ptr),
                    ctypes.c_void_p(d_b.qpos.ptr),
                    ctypes.c_void_p(d_b.qvel.ptr),
                    ctypes.c_void_p(d_b.qfrc_bias.ptr),
                    ctypes.c_void_p(d_b.qacc.ptr),
                    0,
                    int(m_world_stride),
                    ctypes.c_void_p(m_rowadr_ptr),
                    ctypes.c_void_p(m_rownnz_ptr),
                    ctypes.c_void_p(m_colind_ptr),
                    ctypes.c_void_p(d_b.M.ptr),
                    ctypes.c_void_p(d_b.ctrl.ptr),
                    ctypes.c_void_p(d_b.qacc_warmstart.ptr),
                )
            )
            if r != 0:
                raise RuntimeError(f"bike_gpu_launch_sp4_pass failed: {r}")

        return _run

    p1 = _make_pass(lib.bike_gpu_launch_sp4_pass1)
    p2 = _make_pass(lib.bike_gpu_launch_sp4_pass2)
    p3 = _make_pass(lib.bike_gpu_launch_sp4_pass3)

    def _post() -> None:
        r = int(
            lib.bike_gpu_launch_sp4_post_solve(
                ctypes.c_void_p(cuda_stream),
                int(nworld),
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_sp3_configs.ptr),
                1,
                ctypes.c_void_p(d_spindle_params.ptr),
                ctypes.c_void_p(d_sp3_states.ptr),
                ctypes.c_void_p(d_sp4_states.ptr),
                ctypes.c_void_p(d_b.qpos.ptr),
                ctypes.c_void_p(d_b.qvel.ptr),
                ctypes.c_void_p(d_b.xpos.ptr),
                ctypes.c_void_p(d_b.xmat.ptr),
                ctypes.c_void_p(d_b.xanchor.ptr),
                int(post_b["nsensordata"]),
                ctypes.c_void_p(post_b["sensordata_ptr"]),
                ctypes.c_void_p(post_b["nefc_ptr"]),
                int(post_b["njmax"]),
                int(post_b["efc_J_row_stride"]),
                ctypes.c_void_p(post_b["efc_type_ptr"]),
                ctypes.c_void_p(post_b["efc_id_ptr"]),
                ctypes.c_void_p(post_b["efc_J_ptr"]),
                ctypes.c_void_p(post_b["efc_force_ptr"]),
                ctypes.c_void_p(post_b["nacon_ptr"]),
                int(post_b["nconmax"]),
                int(post_b["contact_efc_stride"]),
                ctypes.c_void_p(post_b["contact_worldid_ptr"]),
                ctypes.c_void_p(post_b["contact_geom_ptr"]),
                ctypes.c_void_p(post_b["contact_efc_address_ptr"]),
                ctypes.c_void_p(post_b["contact_dim_ptr"]),
            )
        )
        if r != 0:
            raise RuntimeError(f"bike_gpu_launch_sp4_post_solve failed: {r}")

    _init_states()
    _mjwarp_sp4_clean_split_step(
        ops, m_wp, d_b, _pre, p1, p2, p3, _post, run_collision=True, xz_ctx=xz_ctx
    )
    wp.synchronize()

    _init_states()
    with wp.ScopedCapture() as capture:
        _mjwarp_sp4_clean_split_step(
            ops, m_wp, d_b, _pre, p1, p2, p3, _post, run_collision=True, xz_ctx=xz_ctx
        )
    graph = capture.graph
    for _ in range(warmup_steps):
        wp.capture_launch(graph)
    wp.synchronize()

    _init_states()
    t0 = time.perf_counter()
    for _ in range(n_steps):
        wp.capture_launch(graph)
    wp.synchronize()
    wall_time_s = time.perf_counter() - t0

    final_qpos = d_b.qpos.numpy()
    final_qvel = d_b.qvel.numpy()
    all_finite = bool(np.all(np.isfinite(final_qpos)) and np.all(np.isfinite(final_qvel)))
    forward_distance_m = final_qpos[:, int(idx_struct.root_x_qpos)] - float(
        qpos0[int(idx_struct.root_x_qpos)]
    )
    sp4_raw = d_sp4_states.numpy().tobytes()
    sp3_raw = d_sp3_states.numpy().tobytes()
    world0_metrics = unpack_sp4_world_metrics(sp4_raw[:sizeof_sp4])
    sp3_buf0 = ctypes.create_string_buffer(sp3_raw[:sizeof_sp3], sizeof_sp3)
    has_failure0 = int(lib.bike_gpu_sp3_state_has_failure(ctypes.cast(sp3_buf0, ctypes.c_void_p)))
    failure_flags0 = (
        int(lib.bike_gpu_sp3_state_failure_flags(ctypes.cast(sp3_buf0, ctypes.c_void_p)))
        if hasattr(lib, "bike_gpu_sp3_state_failure_flags")
        else (1 if has_failure0 != 0 else 0)
    )
    total_world_steps = int(nworld) * int(n_steps)
    sim_time_s = float(n_steps) * dt_s
    return {
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "backend": backend,
        "station_count": int(station_count),
        "control_period_steps": int(control_period_steps),
        "wall_time_ms": wall_time_s * 1000.0,
        "ms_per_step": (wall_time_s * 1000.0) / max(1, int(n_steps)),
        "us_per_step_per_world": (wall_time_s * 1e6) / max(1, total_world_steps),
        "steps_per_s": float(total_world_steps) / max(wall_time_s, 1e-12),
        "realtime_factor_x": (sim_time_s * float(nworld)) / max(wall_time_s, 1e-12),
        "all_finite": all_finite,
        "forward_distance_root_x_m": _summarize_distribution(forward_distance_m),
        "world0_metrics": world0_metrics,
        "world0_first_failure": "violation" if has_failure0 != 0 else "none",
        "world0_failure_flags": failure_flags0,
        "world0_balance_lost": bool((failure_flags0 & 1) != 0),
        "world0_crash_detected": bool((failure_flags0 & 6) != 0),
    }


def export_sp4_bundles(
    out_dir: Path,
    physics_toml: Path = DEFAULT_PHYSICS_TOML,
) -> Path:
    """Export S1-S4 scenario bundles (`model_12gears.xml`, `hfield_data.bin`, `step_closure_bundle.bin`, `manifest.json`)."""
    out_root = Path(out_dir).resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    _export_sp4_scenario_bundle(
        "tests/fixtures/tracks/flat.toml",
        0.0,
        out_root / "S1_flat",
        physics_toml=physics_toml,
    )
    steady90_v0 = _compute_steady90_v0(physics_toml=physics_toml, sp4_bundle_dir=out_root)
    scenarios = _build_sp4_s1s4_scenarios(sp4_bundle_dir=out_root)
    for sc in scenarios:
        if sc["id"] != "S1_flat":
            _export_sp4_scenario_bundle(
                sc["track"],
                float(sc["v0"]),
                out_root / sc["id"],
                physics_toml=physics_toml,
            )
    manifest = {
        "physics_toml": str(physics_toml.resolve()),
        "steady90_v0": float(steady90_v0),
        "scenarios": scenarios,
    }
    (out_root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return out_root


def run_mjwarp_sp4_closed_loop_sweep(
    nworld: int = 2048,
    n_steps: int = 200,
    warmup_steps: int = 5,
    seed: int = 42,
    backend: str = "both",
    station_count: int = 128,
    control_period_steps: int = 1,
    lib: ctypes.CDLL | None = None,
    host_lib: ctypes.CDLL | None = None,
    device: str = "cuda:0",
    sp4_bundle_dir: Path | None = None,
) -> dict[str, Any]:
    """Full SP4 closed-loop 4x-solve CUDA Graph throughput sweep."""
    _require_gpu_stack()
    if lib is None:
        lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)
    if host_lib is None:
        try:
            from tools.colab_gpu.cpu_batch_sweep import compile_cpu_batch_runner
        except ImportError:
            from cpu_batch_sweep import compile_cpu_batch_runner  # type: ignore[no-redef]
        host_lib = compile_cpu_batch_runner()

    backends = (
        ("compliant_2d", "distributed_2d_reference")
        if backend == "both"
        else (backend,)
    )
    scales = generate_parameter_sweep_scales(nworld=nworld, seed=seed)
    runs = []
    with tempfile.TemporaryDirectory(prefix="bike-sp4-cuda-") as td:
        xml12, qpos0, qvel0, qacc_ws0 = _export_sp4_scenario_bundle(
            "tests/fixtures/tracks/flat.toml",
            0.0,
            Path(td),
            sp4_bundle_dir=sp4_bundle_dir,
            scenario_id="S1_flat",
        )
        m = load_model_with_assets(xml12)
        cfg_bytes = _build_sp4_config_bytes(
            lib=lib,
            m_mj=m,
            nworld=nworld,
            surface_label=1,
            v0=0.0,
            fork_psi_scale=scales["fork_psi_scale"],
            rebound_scale=scales["rebound_scale"],
            shock_rate_scale=scales["shock_rate_scale"],
            tire_stiff_scale=scales["tire_stiff_scale"],
            physics_config_json=xml12.parent / "physics_config.json",
        )
        for bk in backends:
            res = _run_cuda_sp4_batch(
                lib=lib,
                host_lib=host_lib,
                xml_path=xml12,
                qpos0=qpos0,
                qvel0=qvel0,
                qacc_ws0=qacc_ws0,
                nworld=nworld,
                n_steps=n_steps,
                warmup_steps=warmup_steps,
                surface_label=1,
                backend=bk,
                station_count=station_count if bk == "distributed_2d_reference" else 0,
                control_period_steps=control_period_steps,
                cfg_bytes=cfg_bytes,
                device=device,
            )
            runs.append(res)
    return {
        "experiment": "SP4_closed_loop_4solve_cuda",
        "device": device,
        "runs": runs,
    }


def run_mjwarp_sp4_s1s4_acceptance(
    lib: ctypes.CDLL | None = None,
    host_lib: ctypes.CDLL | None = None,
    device: str = "cuda:0",
    sp4_bundle_dir: Path | None = None,
) -> dict[str, Any]:
    """Run closed-loop S1-S4 acceptance scenarios on CUDA (`4x solve`)."""
    _require_gpu_stack()
    if lib is None:
        lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)
    if host_lib is None:
        try:
            from tools.colab_gpu.cpu_batch_sweep import compile_cpu_batch_runner
        except ImportError:
            from cpu_batch_sweep import compile_cpu_batch_runner  # type: ignore[no-redef]
        host_lib = compile_cpu_batch_runner()

    scenario_results = []
    all_passed = True
    for sc in _build_sp4_s1s4_scenarios(sp4_bundle_dir=sp4_bundle_dir):
        with tempfile.TemporaryDirectory(prefix=f"bike-s1s4-cuda-{sc['id']}-") as td:
            xml12, qpos0, qvel0, qacc_ws0 = _export_sp4_scenario_bundle(
                sc["track"],
                sc["v0"],
                Path(td),
                sp4_bundle_dir=sp4_bundle_dir,
                scenario_id=sc["id"],
            )
            m = load_model_with_assets(xml12)
            cfg_bytes = _build_sp4_config_bytes(
                lib=lib,
                m_mj=m,
                nworld=1,
                surface_label=sc["surface_label"],
                v0=sc["v0"],
                human_torque_nm=sc["human_torque_nm"],
                motor_torque_nm=sc["motor_torque_nm"],
                physics_config_json=xml12.parent / "physics_config.json",
            )
            n_steps = int(round(sc["duration_s"] / float(m.opt.timestep)))
            res = _run_cuda_sp4_batch(
                lib=lib,
                host_lib=host_lib,
                xml_path=xml12,
                qpos0=qpos0,
                qvel0=qvel0,
                qacc_ws0=qacc_ws0,
                nworld=1,
                n_steps=n_steps,
                warmup_steps=2,
                surface_label=sc["surface_label"],
                backend="compliant_2d",
                station_count=0,
                control_period_steps=1,
                cfg_bytes=cfg_bytes,
                device=device,
            )
            wm = res["world0_metrics"]
            dist = float(wm["distance_m"])
            m0_dist = float(sc["m0_distance_m"])
            rel_err = abs(dist - m0_dist) / max(1.0, abs(m0_dist))
            min_press = min(float(wm["min_press_front_n"]), float(wm["min_press_rear_n"]))
            max_rise_f = wm["max_rising_front_excess_n"]
            max_rise_r = wm["max_rising_rear_excess_n"]
            max_rise = (
                max(float(max_rise_f), float(max_rise_r))
                if (max_rise_f is not None and max_rise_r is not None)
                else (
                    float(max_rise_f)
                    if max_rise_f is not None
                    else (float(max_rise_r) if max_rise_r is not None else None)
                )
            )
            min_saddle = float(wm["min_saddle_force_n"])
            max_saddle = float(wm["max_saddle_force_n"])
            passed = (
                res["all_finite"]
                and res["world0_first_failure"] == "none"
                and min_press >= -6.0
                and (max_rise is not None and max_rise <= 6.0)
                and min_saddle >= -6.0
                and max_saddle <= 1750.0
                and rel_err <= 0.002
            )
            if not passed:
                all_passed = False
            scenario_results.append(
                {
                    "id": sc["id"],
                    "passed": bool(passed),
                    "first_failure": res["world0_first_failure"],
                    "distance_m": dist,
                    "m0_distance_m": m0_dist,
                    "distance_rel_err": rel_err,
                    "end_speed_mps": float(wm["end_speed_mps"]),
                    "min_press_front_n": float(wm["min_press_front_n"]),
                    "min_press_rear_n": float(wm["min_press_rear_n"]),
                    "min_press_n": min_press,
                    "max_rising_front_excess_n": max_rise_f,
                    "max_rising_rear_excess_n": max_rise_r,
                    "max_rising_excess_n": max_rise,
                    "min_saddle_force_n": min_saddle,
                    "max_saddle_force_n": max_saddle,
                    "wall_time_ms": res["wall_time_ms"],
                    "all_finite": res["all_finite"],
                }
            )
    return {
        "experiment": "SP4_S1S4_cuda",
        "device": device,
        "all_passed": bool(all_passed),
        "scenarios": scenario_results,
    }


def _print_report(results: dict[str, Any]) -> None:
    if "parameter_sweep" in results:
        sw = results["parameter_sweep"]
        print("\n=== Experiment A: Batched Parameter Sweep (clean_split CUDA Graph) ===")
        print(
            f"nworld = {sw['nworld']} | n_steps = {sw['n_steps']} | "
            f"wall = {sw['wall_time_ms']:.2f} ms | "
            f"{sw['us_per_step_per_world']:.3f} us/step/world | "
            f"{sw['steps_per_s']:,.0f} steps/s | finite = {sw['all_finite']}"
        )
        for metric_name, stats in sw["metrics"].items():
            print(
                f"  {metric_name}: "
                f"min={stats['min']:.3f}, p10={stats['p10']:.3f}, "
                f"median={stats['median']:.3f}, mean={stats['mean']:.3f}, "
                f"p90={stats['p90']:.3f}, max={stats['max']:.3f}"
            )

    if "tyre_backend_scaling" in results:
        tr = results["tyre_backend_scaling"]
        print("\n=== Experiment B: Tyre Backend Scaling Benchmark (clean_split CUDA Graph) ===")
        for tier in tr["tiers"]:
            print(
                f"  {tier['label']:<35} | "
                f"stations={tier['station_count']:3d} | "
                f"wall={tier['wall_time_ms']:7.2f} ms | "
                f"{tier['us_per_step_per_world']:6.3f} us/step/world "
                f"(+{tier['delta_us_vs_compliant']:.3f} us) | "
                f"{tier['steps_per_s']:10,.0f} steps/s"
            )

    if "sp4_closed_loop" in results:
        sp4 = results["sp4_closed_loop"]
        print("\n=== SP4 Closed-Loop 4x-Solve Throughput ===")
        for r in sp4["runs"]:
            print(
                f"  {r['backend']:<26} | nworld={r['nworld']} | n_steps={r['n_steps']} | "
                f"{r['ms_per_step']:.3f} ms/step | {r['steps_per_s']:10,.0f} ws/s | "
                f"RTF={r['realtime_factor_x']:.1f}x | finite={r['all_finite']}"
            )

    if "sp4_s1s4" in results:
        s1s4 = results["sp4_s1s4"]
        print(f"\n=== SP4 S1-S4 Closed-Loop Acceptance (all_passed={s1s4['all_passed']}) ===")
        for sc in s1s4["scenarios"]:
            mr = sc["max_rising_excess_n"]
            mr_str = f"{mr:+.2f}" if mr is not None else "None"
            print(
                f"  [{sc['id']}] pass={sc['passed']} fail={sc['first_failure']} "
                f"dist={sc['distance_m']:.4f} m0={sc['m0_distance_m']:.4f} "
                f"rel={sc['distance_rel_err'] * 100.0:.4f}% "
                f"min_press={sc['min_press_n']:+.2f} max_rise={mr_str} "
                f"saddle={sc['min_saddle_force_n']:+.1f}..{sc['max_saddle_force_n']:.1f}"
            )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run batched C++/CUDA force writers + clean_split mujoco_warp benchmarks."
    )
    parser.add_argument(
        "--model-xml",
        type=Path,
        default=DEFAULT_MODEL_XML,
        help="Path to the MuJoCo XML model.",
    )
    parser.add_argument(
        "--nworld",
        type=int,
        default=2048,
        help="Number of parallel worlds in the GPU batch (default: 2048).",
    )
    parser.add_argument(
        "--n-steps",
        type=int,
        default=1000,
        help="Number of steps for Experiment A parameter sweep (default: 1000).",
    )
    parser.add_argument(
        "--tier-steps",
        type=int,
        default=500,
        help="Number of steps per tier for Experiment B tyre backend scaling (default: 500).",
    )
    parser.add_argument(
        "--experiment",
        choices=("all", "sweep", "tire_scaling", "sp3_autonomous", "sp4_closed_loop", "sp4_s1s4", "sp4_all"),
        default="all",
        help="Which benchmark experiment to execute.",
    )
    parser.add_argument(
        "--backend",
        choices=("both", "compliant_2d", "distributed_2d_reference"),
        default="both",
        help="Tyre backend for sp4_closed_loop sweep (default: both).",
    )
    parser.add_argument(
        "--sp4-bundle-dir",
        type=Path,
        default=None,
        help="Directory containing pre-exported S1-S4 scenario bundles (avoids bike-gpu-export on remote hosts).",
    )
    parser.add_argument(
        "--export-sp4-bundles",
        type=Path,
        default=None,
        help="Export standalone S1-S4 scenario bundles + manifest.json to DIR and exit.",
    )
    parser.add_argument(
        "--use-12gears",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Expand freehub tendon into 12 static cassette tendons for SP3 autonomous run.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path to write JSON benchmark report.",
    )
    parser.add_argument(
        "--synthesize-welded-assets",
        action="store_true",
        help="Synthesize flat-road hfield_data.bin and settled welded initial state when missing.",
    )
    args = parser.parse_args(argv)

    if args.export_sp4_bundles is not None:
        out_root = export_sp4_bundles(args.export_sp4_bundles)
        print(f"Exported SP4 S1-S4 bundles to {out_root}")
        return 0

    lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)
    results: dict[str, Any] = {}

    with tempfile.TemporaryDirectory(prefix="bike-cuda-assets-") as tmp_dir:
        model_xml_path = (
            materialize_welded_assets_dir(args.model_xml, tmp_dir)
            if args.synthesize_welded_assets
            else args.model_xml
        )
        if args.experiment in ("all", "sweep"):
            results["parameter_sweep"] = run_parameter_sweep(
                model_xml_path=model_xml_path,
                nworld=args.nworld,
                n_steps=args.n_steps,
                lib=lib,
            )

        if args.experiment in ("all", "tire_scaling"):
            results["tyre_backend_scaling"] = run_tyre_backend_scaling_benchmark(
                model_xml_path=model_xml_path,
                nworld=args.nworld,
                n_steps=args.tier_steps,
                lib=lib,
            )

        if args.experiment == "sp3_autonomous":
            results["sp3_autonomous"] = run_mjwarp_sp3_autonomous_sweep(
                model_xml_path=model_xml_path,
                nworld=args.nworld,
                n_steps=args.n_steps,
                use_12gears=args.use_12gears,
                lib=lib,
            )

        if args.experiment in ("sp4_closed_loop", "sp4_all"):
            results["sp4_closed_loop"] = run_mjwarp_sp4_closed_loop_sweep(
                nworld=args.nworld,
                n_steps=args.n_steps,
                backend=args.backend,
                lib=lib,
                sp4_bundle_dir=args.sp4_bundle_dir,
            )

        if args.experiment in ("sp4_s1s4", "sp4_all"):
            results["sp4_s1s4"] = run_mjwarp_sp4_s1s4_acceptance(
                lib=lib,
                sp4_bundle_dir=args.sp4_bundle_dir,
            )

    _print_report(results)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
