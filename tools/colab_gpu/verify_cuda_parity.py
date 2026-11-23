"""End-to-end NVIDIA CUDA vs Host-float (Level A) and CPU FP64 (Level C) Parity Verification.

Executes on an NVIDIA CUDA environment (e.g., Google Colab L4/T4/A100) with ``mujoco``, ``warp``,
and ``mujoco_warp`` installed:
  1. Compiles ``libbike_cuda_writers.so`` (``nvcc -O3 --fmad=false``) and
     ``libbike_cpu_batch_runner.so`` (``g++ -O3 -ffp-contract=off``).
  2. Level A — Exact CUDA vs Host-float kernel parity (``0``-bit / ``<= 1 ULP``) across
     ``nworld = 32`` heterogeneous worlds and all 3 tyre backends (``compliant_2d``,
     ``distributed_2d_reference`` with ``station_count = 128``, and ``pneumatic`` with
     ``station_count = 64``):
       - SP1/SP2 ``bike_gpu_launch_writers`` (5 steps)
       - SP3 ``init`` + ``pre_solve`` + ``post_solve`` (5 steps)
       - SP4 ``M2K2`` + ``energy_post_step`` (5 steps, interleaved with ``mujoco_warp`` 4x solve passes)
       - SP4 ``SpindleM0`` Armijo Newton loop + ``energy_post_step`` (5 steps)
  3. Level C — Closed-loop ``mujoco_warp`` + CUDA (``m2k2`` & ``m0``, crash contacts active)
     acceptance across all 10 scenarios (``S1..S4`` + ``G1..G6``) compared
     against CPU FP64 (``M0``) reference metrics in ``manifest.json`` and verifying
     ``GpuEnergyLedger`` (``max_rel_energy_residual <= 0.05`` & ``energy_contract_ok == 1``).
  4. Batched GPU throughput sanity at ``nworld = 2048, n_steps = 200`` across ``compliant_2d``,
     ``distributed_2d_reference``, and ``pneumatic``.
"""

from __future__ import annotations

import argparse
import ctypes
import json
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any

import mujoco
import numpy as np
import warp as wp
import mujoco_warp as mjwarp


def _require_mujoco_3x() -> None:
    if int(str(mujoco.__version__).split(".", 1)[0]) != 3:
        raise RuntimeError(
            f"Unsupported mujoco major version {mujoco.__version__}; expected mujoco>=3.11.0,<4.0.0"
        )

try:
    from tools.colab_gpu.build_cuda_writers import (
        DEFAULT_GPU_SRC_DIR,
        DEFAULT_OUT_SO_PATH,
        GpuModelIndicesStruct,
        build_spindle_params_bytes,
        build_sweep_configs_bytes,
        build_tyre_and_terrain_buffers,
        compile_cuda_writers,
        configure_cuda_writers_cdll,
        expand_mjwarp_tendon_range_and_frictionloss,
        resolve_gpu_model_indices,
        unpack_sp3_energy_ledger,
        unpack_sp4_world_metrics,
    )
    from tools.colab_gpu.cpu_batch_sweep import compile_cpu_batch_runner
    from tools.colab_gpu.mjwarp_batch_sweep import (
        DEFAULT_ENVELOPE_JSON,
        DEFAULT_STRENGTH_JSON,
        _allocate_and_copy_device_bytes,
        _build_sp4_config_bytes,
        _configure_sp4_mjwarp_model,
        _ensure_newton_solver,
        _load_step_closure_row0,
        _resolve_clean_split_ops,
        _resolve_dof_frictionloss_binding,
        _resolve_sp3_post_solve_bindings,
        _resolve_tendon_range_binding,
        _run_cuda_sp4_batch,
        _upload_tyre_and_terrain_device,
        generate_parameter_sweep_scales,
    )
    from tools.colab_gpu.model_assets import load_model_with_assets
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_cuda_writers import (  # type: ignore[no-redef]
        DEFAULT_GPU_SRC_DIR,
        DEFAULT_OUT_SO_PATH,
        GpuModelIndicesStruct,
        build_spindle_params_bytes,
        build_sweep_configs_bytes,
        build_tyre_and_terrain_buffers,
        compile_cuda_writers,
        configure_cuda_writers_cdll,
        expand_mjwarp_tendon_range_and_frictionloss,
        resolve_gpu_model_indices,
        unpack_sp3_energy_ledger,
        unpack_sp4_world_metrics,
    )
    from cpu_batch_sweep import compile_cpu_batch_runner  # type: ignore[no-redef]
    from mjwarp_batch_sweep import (  # type: ignore[no-redef]
        DEFAULT_ENVELOPE_JSON,
        DEFAULT_STRENGTH_JSON,
        _allocate_and_copy_device_bytes,
        _build_sp4_config_bytes,
        _configure_sp4_mjwarp_model,
        _ensure_newton_solver,
        _load_step_closure_row0,
        _resolve_clean_split_ops,
        _resolve_dof_frictionloss_binding,
        _resolve_sp3_post_solve_bindings,
        _resolve_tendon_range_binding,
        _run_cuda_sp4_batch,
        _upload_tyre_and_terrain_device,
        generate_parameter_sweep_scales,
    )
    from model_assets import load_model_with_assets  # type: ignore[no-redef]


def _f32_ulp_stats(a: np.ndarray, b: np.ndarray) -> dict[str, Any]:
    """Compute exact 0-bit match fraction, max ULP distance, and max abs diff on float32 arrays."""
    af = np.ascontiguousarray(a, dtype=np.float32).ravel()
    bf = np.ascontiguousarray(b, dtype=np.float32).ravel()
    if af.size != bf.size:
        raise ValueError(f"Shape mismatch in ULP comparison: {af.size} vs {bf.size}")
    if af.size == 0:
        return {"count": 0, "zero_bit_fraction": 1.0, "max_ulp": 0, "max_abs_diff": 0.0}

    # Normalize +0.0 and -0.0 before sign-magnitude ULP conversion (while keeping raw bit count too)
    raw_a = af.view(np.uint32)
    raw_b = bf.view(np.uint32)
    exact_bits = (raw_a == raw_b) | ((af == 0.0) & (bf == 0.0))

    def _to_ordered_int(u: np.ndarray) -> np.ndarray:
        i = u.view(np.int32).astype(np.int64)
        return np.where(i < 0, np.int64(-0x80000000) - i, i)

    both_nan = np.isnan(af) & np.isnan(bf)
    any_unmatched_nan = np.isnan(af) ^ np.isnan(bf)
    if bool(np.any(any_unmatched_nan)):
        return {
            "count": int(af.size),
            "zero_bit_fraction": float(np.mean(exact_bits)),
            "max_ulp": 2**31,
            "max_abs_diff": float("nan"),
        }

    ord_a = _to_ordered_int(np.where(af == 0.0, np.uint32(0), raw_a))
    ord_b = _to_ordered_int(np.where(bf == 0.0, np.uint32(0), raw_b))
    ulp = np.where(both_nan, 0, np.abs(ord_a - ord_b))
    abs_diff = np.where(both_nan, 0.0, np.abs(af.astype(np.float64) - bf.astype(np.float64)))
    return {
        "count": int(af.size),
        "zero_bit_fraction": float(np.mean(exact_bits | both_nan)),
        "max_ulp": int(np.max(ulp)),
        "max_abs_diff": float(np.max(abs_diff)),
    }


def _ptr_f32(arr: np.ndarray) -> ctypes.c_void_p:
    return ctypes.c_void_p(arr.ctypes.data)


def _ptr_buf(buf: ctypes.Array[ctypes.c_char]) -> ctypes.c_void_p:
    return ctypes.cast(buf, ctypes.c_void_p)


def _collect_provenance() -> dict[str, Any]:
    def _cmd_out(cmd: list[str]) -> str:
        try:
            p = subprocess.run(cmd, check=False, capture_output=True, text=True, timeout=10)
            return (p.stdout or p.stderr).strip()
        except Exception as exc:
            return f"error: {exc}"

    wp.init()
    dev = wp.get_device("cuda:0")
    return {
        "platform": platform.platform(),
        "python": sys.version,
        "gpu_name": str(getattr(dev, "name", dev)),
        "nvidia_smi": _cmd_out(
            ["nvidia-smi", "--query-gpu=name,driver_version,compute_cap,memory.total", "--format=csv,noheader"]
        ),
        "nvcc_version": _cmd_out(["nvcc", "--version"]),
        "gxx_version": _cmd_out(["g++", "--version"]),
        "mujoco_version": str(getattr(mujoco, "__version__", "unknown")),
        "warp_version": str(getattr(wp, "__version__", "unknown")),
        "mujoco_warp_version": str(getattr(mjwarp, "__version__", "unknown")),
    }


def run_level_a_sp1_writers_parity(
    *,
    cuda_lib: ctypes.CDLL,
    host_lib: ctypes.CDLL,
    xml_path: Path,
    qpos0: np.ndarray,
    qvel0: np.ndarray,
    backend: str,
    station_count: int,
    nworld: int = 32,
    n_steps: int = 5,
    device: str = "cuda:0",
) -> dict[str, Any]:
    """Compare ``bike_gpu_launch_writers`` on CUDA vs Host-float across 32 heterogeneous worlds."""
    m = load_model_with_assets(xml_path)
    _ensure_newton_solver(m)
    d0 = mujoco.MjData(m)
    d0.qpos[:] = qpos0
    d0.qvel[:] = qvel0
    mujoco.mj_forward(m, d0)
    dt_s = float(m.opt.timestep)

    idx_bytes = resolve_gpu_model_indices(m)
    scales = generate_parameter_sweep_scales(nworld=nworld, seed=42)
    cfg_bytes = build_sweep_configs_bytes(
        cuda_lib,
        nworld=nworld,
        fork_psi_scale=scales["fork_psi_scale"],
        rebound_scale=scales["rebound_scale"],
        shock_rate_scale=scales["shock_rate_scale"],
        tire_stiff_scale=scales["tire_stiff_scale"],
    )
    model_b, desc_b, f32_np, i32_np = build_tyre_and_terrain_buffers(
        host_lib=host_lib,
        xml_path=xml_path,
        backend=backend,
        station_count=station_count,
        idx_bytes=idx_bytes,
        surface_label=1,
    )
    sizeof_ws = int(cuda_lib.bike_gpu_sizeof_world_state())

    # Device allocations
    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_cfg = _allocate_and_copy_device_bytes(cfg_bytes, device=device)
    d_tyre = _allocate_and_copy_device_bytes(model_b, device=device)
    d_desc = _allocate_and_copy_device_bytes(desc_b, device=device)
    d_tf32 = wp.array(f32_np if f32_np.size > 0 else np.zeros(1, dtype=np.float32), dtype=wp.float32, device=device)
    d_ti32 = wp.array(i32_np if i32_np.size > 0 else np.zeros(1, dtype=np.int32), dtype=wp.int32, device=device)
    d_ws = wp.zeros(nworld * sizeof_ws, dtype=wp.uint8, device=device)

    m_wp = _configure_sp4_mjwarp_model(m, device=device, disable_contacts=False)
    expand_mjwarp_tendon_range_and_frictionloss(m, m_wp, nworld, device=device)
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=64, njmax=128)
    mjwarp.forward(m_wp, d_b)
    wp.synchronize()
    cuda_stream = wp.get_stream().cuda_stream
    tr_ptr, tr_stride = _resolve_tendon_range_binding(m_wp, nworld)

    # Host allocations
    h_idx = ctypes.create_string_buffer(idx_bytes, len(idx_bytes))
    h_cfg = ctypes.create_string_buffer(cfg_bytes, len(cfg_bytes))
    h_tyre = ctypes.create_string_buffer(model_b, len(model_b))
    h_desc = ctypes.create_string_buffer(desc_b, len(desc_b))
    h_tf32 = np.ascontiguousarray(f32_np if f32_np.size > 0 else np.zeros(1, dtype=np.float32), dtype=np.float32)
    h_ti32 = np.ascontiguousarray(i32_np if i32_np.size > 0 else np.zeros(1, dtype=np.int32), dtype=np.int32)
    h_ws = ctypes.create_string_buffer(nworld * sizeof_ws)
    h_tr = np.ascontiguousarray(m_wp.tendon_range.numpy().reshape(-1), dtype=np.float32)

    # Init both states identically
    rc_gpu = int(
        cuda_lib.bike_gpu_launch_init_states(
            ctypes.c_void_p(cuda_stream),
            nworld,
            ctypes.c_void_p(d_idx.ptr),
            ctypes.c_void_p(d_cfg.ptr),
            1,
            ctypes.c_void_p(d_ws.ptr),
            ctypes.c_void_p(d_b.qpos.ptr),
        )
    )
    h_qpos0 = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)
    rc_cpu = int(
        host_lib.bike_gpu_launch_init_states(
            None,
            nworld,
            _ptr_buf(h_idx),
            _ptr_buf(h_cfg),
            1,
            _ptr_buf(h_ws),
            _ptr_f32(h_qpos0),
        )
    )
    wp.synchronize()
    if rc_gpu != 0 or rc_cpu != 0:
        raise RuntimeError(f"init_states failed: gpu={rc_gpu}, cpu={rc_cpu}")

    max_qfrc_ulp = 0
    max_ctrl_ulp = 0
    qfrc_stats_last: dict[str, Any] = {}
    ctrl_stats_last: dict[str, Any] = {}

    for step in range(n_steps):
        if step > 0:
            # Perturb kinematics deterministically across worlds/steps to exercise hysteretic state evolution
            qp_np = d_b.qpos.numpy()
            qv_np = d_b.qvel.numpy()
            qp_np[:, 0] += 0.004 * (1.0 + 0.02 * np.arange(nworld, dtype=np.float32))
            qv_np[:, 0] = 3.5 + 0.1 * np.arange(nworld, dtype=np.float32)
            d_b.qpos.assign(wp.array(qp_np, dtype=wp.float32, device=device))
            d_b.qvel.assign(wp.array(qv_np, dtype=wp.float32, device=device))
            mjwarp.forward(m_wp, d_b)
            wp.synchronize()

        h_qpos = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)
        h_qvel = np.ascontiguousarray(d_b.qvel.numpy(), dtype=np.float32)
        h_xpos = np.ascontiguousarray(d_b.xpos.numpy(), dtype=np.float32)
        h_xmat = np.ascontiguousarray(d_b.xmat.numpy(), dtype=np.float32)
        h_xanc = np.ascontiguousarray(d_b.xanchor.numpy(), dtype=np.float32)
        h_xaxs = np.ascontiguousarray(d_b.xaxis.numpy(), dtype=np.float32)
        h_qfrc = np.zeros((nworld, int(m.nv)), dtype=np.float32)
        h_ctrl = np.zeros((nworld, int(m.nu)), dtype=np.float32)
        d_b.qfrc_applied.zero_()
        d_b.ctrl.zero_()
        wp.synchronize()

        rc_g = int(
            cuda_lib.bike_gpu_launch_writers(
                ctypes.c_void_p(cuda_stream),
                nworld,
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_tyre.ptr),
                ctypes.c_void_p(d_desc.ptr),
                ctypes.c_void_p(d_tf32.ptr),
                ctypes.c_void_p(d_ti32.ptr),
                ctypes.c_void_p(d_cfg.ptr),
                1,
                ctypes.c_void_p(d_ws.ptr),
                ctypes.c_void_p(d_b.qpos.ptr),
                ctypes.c_void_p(d_b.qvel.ptr),
                ctypes.c_void_p(d_b.xpos.ptr),
                ctypes.c_void_p(d_b.xmat.ptr),
                ctypes.c_void_p(d_b.xanchor.ptr),
                ctypes.c_void_p(d_b.xaxis.ptr),
                ctypes.c_void_p(d_b.qfrc_applied.ptr),
                ctypes.c_void_p(d_b.ctrl.ptr),
                ctypes.c_void_p(tr_ptr),
                int(tr_stride),
            )
        )
        wp.synchronize()
        rc_c = int(
            host_lib.bike_gpu_launch_writers(
                None,
                nworld,
                ctypes.c_float(dt_s),
                _ptr_buf(h_idx),
                _ptr_buf(h_tyre),
                _ptr_buf(h_desc),
                _ptr_f32(h_tf32),
                ctypes.c_void_p(h_ti32.ctypes.data),
                _ptr_buf(h_cfg),
                1,
                _ptr_buf(h_ws),
                _ptr_f32(h_qpos),
                _ptr_f32(h_qvel),
                _ptr_f32(h_xpos),
                _ptr_f32(h_xmat),
                _ptr_f32(h_xanc),
                _ptr_f32(h_xaxs),
                _ptr_f32(h_qfrc),
                _ptr_f32(h_ctrl),
                _ptr_f32(h_tr),
                int(tr_stride),
            )
        )
        if rc_g != 0 or rc_c != 0:
            raise RuntimeError(f"writers failed at step {step}: gpu={rc_g}, cpu={rc_c}")

        qfrc_stats_last = _f32_ulp_stats(d_b.qfrc_applied.numpy(), h_qfrc)
        ctrl_stats_last = _f32_ulp_stats(d_b.ctrl.numpy(), h_ctrl)
        max_qfrc_ulp = max(max_qfrc_ulp, int(qfrc_stats_last["max_ulp"]))
        max_ctrl_ulp = max(max_ctrl_ulp, int(ctrl_stats_last["max_ulp"]))

    passed = max_qfrc_ulp <= 1 and max_ctrl_ulp <= 1
    return {
        "stage": "SP1_SP2_writers",
        "backend": backend,
        "station_count": station_count,
        "nworld": nworld,
        "n_steps": n_steps,
        "passed": bool(passed),
        "max_qfrc_ulp": max_qfrc_ulp,
        "max_ctrl_ulp": max_ctrl_ulp,
        "last_step_qfrc": qfrc_stats_last,
        "last_step_ctrl": ctrl_stats_last,
    }


def run_level_a_sp3_sp4_parity(
    *,
    cuda_lib: ctypes.CDLL,
    host_lib: ctypes.CDLL,
    xml_path: Path,
    qpos0: np.ndarray,
    qvel0: np.ndarray,
    qacc_ws0: np.ndarray | None,
    backend: str,
    station_count: int,
    nworld: int = 32,
    n_steps: int = 5,
    device: str = "cuda:0",
) -> dict[str, Any]:
    """Compare SP3 & SP4 6-kernel pipelines on CUDA vs Host-float across 32 heterogeneous worlds."""
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
    scales = generate_parameter_sweep_scales(nworld=nworld, seed=42)
    manifest_path = xml_path.parent / "manifest.json"
    cfg_bytes = _build_sp4_config_bytes(
        lib=cuda_lib,
        m_mj=m,
        nworld=nworld,
        surface_label=1,
        v0=0.0,
        fork_psi_scale=scales["fork_psi_scale"],
        rebound_scale=scales["rebound_scale"],
        shock_rate_scale=scales["shock_rate_scale"],
        tire_stiff_scale=scales["tire_stiff_scale"],
        physics_config_json=xml_path.parent / "physics_config.json",
        manifest_json=manifest_path if manifest_path.is_file() else None,
    )
    spindle_bytes = build_spindle_params_bytes(
        host_lib=host_lib,
        xml_path=xml_path,
        strength_json_path=DEFAULT_STRENGTH_JSON,
        envelope_json_path=DEFAULT_ENVELOPE_JSON,
        control_period_steps=1,
    )
    model_b, desc_b, f32_np, i32_np = build_tyre_and_terrain_buffers(
        host_lib=host_lib,
        xml_path=xml_path,
        backend=backend,
        station_count=station_count,
        idx_bytes=idx_bytes,
        surface_label=1,
    )

    sizeof_ws = int(cuda_lib.bike_gpu_sizeof_world_state())
    sizeof_sp3 = int(cuda_lib.bike_gpu_sizeof_sp3_world_state())
    sizeof_sp4 = int(cuda_lib.bike_gpu_sizeof_sp4_world_state())

    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_cfg = _allocate_and_copy_device_bytes(cfg_bytes, device=device)
    d_spindle = _allocate_and_copy_device_bytes(spindle_bytes, device=device)
    d_tyre, d_desc, _d_tf32, _d_ti32, tf32_ptr, ti32_ptr = _upload_tyre_and_terrain_device(
        host_lib, xml_path, backend, station_count, idx_bytes, device=device, surface_label=1
    )
    d_ws = wp.zeros(nworld * sizeof_ws, dtype=wp.uint8, device=device)
    d_sp3 = wp.zeros(nworld * sizeof_sp3, dtype=wp.uint8, device=device)
    d_sp4 = wp.zeros(nworld * sizeof_sp4, dtype=wp.uint8, device=device)

    m_wp = _configure_sp4_mjwarp_model(m, device=device, disable_contacts=False)
    expand_mjwarp_tendon_range_and_frictionloss(m, m_wp, nworld, device=device)
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=max(32, int(d0.ncon) + 32), njmax=max(64, int(d0.nefc) + 32))
    mjwarp.forward(m_wp, d_b)
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

    # Host shadow buffers
    h_idx = ctypes.create_string_buffer(idx_bytes, len(idx_bytes))
    h_cfg = ctypes.create_string_buffer(cfg_bytes, len(cfg_bytes))
    h_spindle = ctypes.create_string_buffer(spindle_bytes, len(spindle_bytes))
    h_tyre = ctypes.create_string_buffer(model_b, len(model_b))
    h_desc = ctypes.create_string_buffer(desc_b, len(desc_b))
    h_tf32 = np.ascontiguousarray(f32_np if f32_np.size > 0 else np.zeros(1, dtype=np.float32), dtype=np.float32)
    h_ti32 = np.ascontiguousarray(i32_np if i32_np.size > 0 else np.zeros(1, dtype=np.int32), dtype=np.int32)
    h_ws = ctypes.create_string_buffer(nworld * sizeof_ws)
    h_sp3 = ctypes.create_string_buffer(nworld * sizeof_sp3)
    h_sp4 = ctypes.create_string_buffer(nworld * sizeof_sp4)
    h_tr = np.ascontiguousarray(m_wp.tendon_range.numpy().reshape(-1), dtype=np.float32)
    h_fl = np.ascontiguousarray(m_wp.dof_frictionloss.numpy().reshape(-1), dtype=np.float32)
    h_submass = (
        np.ascontiguousarray(submass_arr.numpy().reshape(-1), dtype=np.float32)
        if submass_arr is not None
        else np.ascontiguousarray(m.body_subtreemass, dtype=np.float32)
    )
    h_m_rowadr = np.ascontiguousarray(m_wp.M_rowadr.numpy().reshape(-1), dtype=np.int32)
    h_m_rownnz = np.ascontiguousarray(m_wp.M_rownnz.numpy().reshape(-1), dtype=np.int32)
    h_m_colind = np.ascontiguousarray(m_wp.M_colind.numpy().reshape(-1), dtype=np.int32)

    h_qpos0 = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)
    rc_g = int(
        cuda_lib.bike_gpu_launch_sp4_init(
            ctypes.c_void_p(cuda_stream),
            nworld,
            ctypes.c_void_p(d_idx.ptr),
            ctypes.c_void_p(d_cfg.ptr),
            1,
            ctypes.c_void_p(d_sp3.ptr),
            ctypes.c_void_p(d_sp4.ptr),
            ctypes.c_void_p(d_ws.ptr),
            ctypes.c_void_p(d_b.qpos.ptr),
            ctypes.c_float(0.0),
            int(m.ntendon),
            ctypes.c_void_p(tr_ptr),
            int(tr_stride),
        )
    )
    wp.synchronize()
    rc_c = int(
        host_lib.bike_gpu_launch_sp4_init(
            None,
            nworld,
            _ptr_buf(h_idx),
            _ptr_buf(h_cfg),
            1,
            _ptr_buf(h_sp3),
            _ptr_buf(h_sp4),
            _ptr_buf(h_ws),
            _ptr_f32(h_qpos0),
            ctypes.c_float(0.0),
            int(m.ntendon),
            _ptr_f32(h_tr),
            int(tr_stride),
        )
    )
    if rc_g != 0 or rc_c != 0:
        raise RuntimeError(f"sp4_init failed: gpu={rc_g}, cpu={rc_c}")

    max_ulp_by_array = {
        "qfrc_applied": 0,
        "ctrl_pass0": 0,
        "ctrl_pass1": 0,
        "ctrl_pass2": 0,
        "ctrl_pass3": 0,
        "tendon_range": 0,
        "dof_frictionloss": 0,
        "qacc_warmstart": 0,
        "sp4_metrics": 0,
    }
    last_stats: dict[str, Any] = {}

    def _solve_once() -> None:
        ops["fwd_actuation"](m_wp, d_b)
        ops["fwd_acceleration"](m_wp, d_b, factorize=False)
        ops["solve"](m_wp, d_b)
        wp.synchronize()

    for _step in range(n_steps):
        ops["fwd_kinematics"](m_wp, d_b)
        ops["crb"](m_wp, d_b)
        ops["tendon_armature"](m_wp, d_b)
        ops["factor_m"](m_wp, d_b)
        ops["collision"](m_wp, d_b)
        ops["transmission"](m_wp, d_b)
        ops["fwd_velocity"](m_wp, d_b)
        ops["sensor_vel"](m_wp, d_b)
        wp.synchronize()

        h_qpos = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)
        h_qvel = np.ascontiguousarray(d_b.qvel.numpy(), dtype=np.float32)
        h_bias = np.ascontiguousarray(d_b.qfrc_bias.numpy(), dtype=np.float32)
        h_xpos = np.ascontiguousarray(d_b.xpos.numpy(), dtype=np.float32)
        h_xmat = np.ascontiguousarray(d_b.xmat.numpy(), dtype=np.float32)
        h_xanc = np.ascontiguousarray(d_b.xanchor.numpy(), dtype=np.float32)
        h_xaxs = np.ascontiguousarray(d_b.xaxis.numpy(), dtype=np.float32)
        h_site = np.ascontiguousarray(d_b.site_xpos.numpy(), dtype=np.float32)
        h_scom = np.ascontiguousarray(d_b.subtree_com.numpy(), dtype=np.float32)
        h_M = np.ascontiguousarray(d_b.M.numpy(), dtype=np.float32)
        h_qfrc = np.zeros_like(d_b.qfrc_applied.numpy(), dtype=np.float32)
        h_ctrl = np.zeros_like(d_b.ctrl.numpy(), dtype=np.float32)
        h_qws = np.ascontiguousarray(d_b.qacc_warmstart.numpy(), dtype=np.float32)

        rc_g = int(
            cuda_lib.bike_gpu_launch_sp4_pre_solve_and_pass0(
                ctypes.c_void_p(cuda_stream),
                nworld,
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_tyre.ptr),
                ctypes.c_void_p(d_desc.ptr),
                ctypes.c_void_p(tf32_ptr),
                ctypes.c_void_p(ti32_ptr),
                ctypes.c_void_p(d_cfg.ptr),
                1,
                ctypes.c_void_p(d_spindle.ptr),
                ctypes.c_void_p(d_sp3.ptr),
                ctypes.c_void_p(d_sp4.ptr),
                ctypes.c_void_p(d_ws.ptr),
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
        wp.synchronize()
        rc_c = int(
            host_lib.bike_gpu_launch_sp4_pre_solve_and_pass0(
                None,
                nworld,
                ctypes.c_float(dt_s),
                _ptr_buf(h_idx),
                _ptr_buf(h_tyre),
                _ptr_buf(h_desc),
                _ptr_f32(h_tf32),
                ctypes.c_void_p(h_ti32.ctypes.data),
                _ptr_buf(h_cfg),
                1,
                _ptr_buf(h_spindle),
                _ptr_buf(h_sp3),
                _ptr_buf(h_sp4),
                _ptr_buf(h_ws),
                _ptr_f32(h_qpos),
                _ptr_f32(h_qvel),
                _ptr_f32(h_bias),
                _ptr_f32(h_xpos),
                _ptr_f32(h_xmat),
                _ptr_f32(h_xanc),
                _ptr_f32(h_xaxs),
                _ptr_f32(h_site),
                _ptr_f32(h_scom),
                _ptr_f32(h_submass),
                0,
                int(m_world_stride),
                ctypes.c_void_p(h_m_rowadr.ctypes.data),
                ctypes.c_void_p(h_m_rownnz.ctypes.data),
                ctypes.c_void_p(h_m_colind.ctypes.data),
                _ptr_f32(h_M),
                _ptr_f32(h_qfrc),
                _ptr_f32(h_ctrl),
                int(m.ntendon),
                _ptr_f32(h_tr),
                int(tr_stride),
                _ptr_f32(h_fl),
                int(fl_stride),
                _ptr_f32(h_qws),
            )
        )
        if rc_g != 0 or rc_c != 0:
            raise RuntimeError(f"sp4_pre_solve_and_pass0 failed: gpu={rc_g}, cpu={rc_c}")

        for key, g_arr, c_arr in (
            ("qfrc_applied", d_b.qfrc_applied.numpy(), h_qfrc),
            ("ctrl_pass0", d_b.ctrl.numpy(), h_ctrl),
            ("tendon_range", m_wp.tendon_range.numpy().reshape(-1), h_tr),
            ("dof_frictionloss", m_wp.dof_frictionloss.numpy().reshape(-1), h_fl),
            ("qacc_warmstart", d_b.qacc_warmstart.numpy(), h_qws),
        ):
            st = _f32_ulp_stats(g_arr, c_arr)
            max_ulp_by_array[key] = max(max_ulp_by_array[key], int(st["max_ulp"]))
            last_stats[key] = st

        ops["make_constraint"](m_wp, d_b)
        _solve_once()

        for pass_idx, pass_name in ((1, "bike_gpu_launch_sp4_pass1"), (2, "bike_gpu_launch_sp4_pass2"), (3, "bike_gpu_launch_sp4_pass3")):
            h_qacc = np.ascontiguousarray(d_b.qacc.numpy(), dtype=np.float32)
            h_ctrl = np.ascontiguousarray(d_b.ctrl.numpy(), dtype=np.float32)
            h_qws = np.ascontiguousarray(d_b.qacc_warmstart.numpy(), dtype=np.float32)
            rc_g = int(
                getattr(cuda_lib, pass_name)(
                    ctypes.c_void_p(cuda_stream),
                    nworld,
                    int(m.nq),
                    int(m.nv),
                    int(m.nu),
                    ctypes.c_void_p(d_spindle.ptr),
                    ctypes.c_void_p(d_sp4.ptr),
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
            wp.synchronize()
            rc_c = int(
                getattr(host_lib, pass_name)(
                    None,
                    nworld,
                    int(m.nq),
                    int(m.nv),
                    int(m.nu),
                    _ptr_buf(h_spindle),
                    _ptr_buf(h_sp4),
                    _ptr_f32(h_qpos),
                    _ptr_f32(h_qvel),
                    _ptr_f32(h_bias),
                    _ptr_f32(h_qacc),
                    0,
                    int(m_world_stride),
                    ctypes.c_void_p(h_m_rowadr.ctypes.data),
                    ctypes.c_void_p(h_m_rownnz.ctypes.data),
                    ctypes.c_void_p(h_m_colind.ctypes.data),
                    _ptr_f32(h_M),
                    _ptr_f32(h_ctrl),
                    _ptr_f32(h_qws),
                )
            )
            if rc_g != 0 or rc_c != 0:
                raise RuntimeError(f"{pass_name} failed: gpu={rc_g}, cpu={rc_c}")

            ctrl_key = f"ctrl_pass{pass_idx}"
            st_c = _f32_ulp_stats(d_b.ctrl.numpy(), h_ctrl)
            st_w = _f32_ulp_stats(d_b.qacc_warmstart.numpy(), h_qws)
            max_ulp_by_array[ctrl_key] = max(max_ulp_by_array[ctrl_key], int(st_c["max_ulp"]))
            max_ulp_by_array["qacc_warmstart"] = max(max_ulp_by_array["qacc_warmstart"], int(st_w["max_ulp"]))
            last_stats[ctrl_key] = st_c
            _solve_once()

        ops["sensor_acc"](m_wp, d_b)
        ops["implicit"](m_wp, d_b)
        wp.synchronize()

        # Copy post-solve inputs from device to host so post_solve sees identical solver outputs
        h_qpos_post = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)
        h_qvel_post = np.ascontiguousarray(d_b.qvel.numpy(), dtype=np.float32)
        h_xpos_post = np.ascontiguousarray(d_b.xpos.numpy(), dtype=np.float32)
        h_xmat_post = np.ascontiguousarray(d_b.xmat.numpy(), dtype=np.float32)
        h_xanc_post = np.ascontiguousarray(d_b.xanchor.numpy(), dtype=np.float32)
        h_sd = (
            np.ascontiguousarray(d_b.sensordata.numpy(), dtype=np.float32)
            if post_b["sensordata_ptr"] != 0
            else np.zeros(1, dtype=np.float32)
        )
        efc = getattr(d_b, "efc", None)
        h_nefc = np.ascontiguousarray(d_b.nefc.numpy(), dtype=np.int32) if post_b["nefc_ptr"] != 0 else np.zeros(nworld, dtype=np.int32)
        h_etype = np.ascontiguousarray(efc.type.numpy(), dtype=np.int32) if efc is not None and post_b["efc_type_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_eid = np.ascontiguousarray(efc.id.numpy(), dtype=np.int32) if efc is not None and post_b["efc_id_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_eJ = np.ascontiguousarray(efc.J.numpy(), dtype=np.float32) if efc is not None and post_b["efc_J_ptr"] != 0 else np.zeros(1, dtype=np.float32)
        h_ef = np.ascontiguousarray(efc.force.numpy(), dtype=np.float32) if efc is not None and post_b["efc_force_ptr"] != 0 else np.zeros(1, dtype=np.float32)
        con = getattr(d_b, "contact", None)
        h_nacon = np.ascontiguousarray(d_b.nacon.numpy(), dtype=np.int32) if post_b["nacon_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_cwid = np.ascontiguousarray(con.worldid.numpy(), dtype=np.int32) if con is not None and post_b["contact_worldid_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_cgeom = np.ascontiguousarray(con.geom.numpy(), dtype=np.int32) if con is not None and post_b["contact_geom_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_cadr = np.ascontiguousarray(con.efc_address.numpy(), dtype=np.int32) if con is not None and post_b["contact_efc_address_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_cdim = np.ascontiguousarray(con.dim.numpy(), dtype=np.int32) if con is not None and post_b["contact_dim_ptr"] != 0 else np.zeros(1, dtype=np.int32)

        rc_g = int(
            cuda_lib.bike_gpu_launch_sp4_post_solve(
                ctypes.c_void_p(cuda_stream),
                nworld,
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_cfg.ptr),
                1,
                ctypes.c_void_p(d_spindle.ptr),
                ctypes.c_void_p(d_sp3.ptr),
                ctypes.c_void_p(d_sp4.ptr),
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
        wp.synchronize()
        rc_c = int(
            host_lib.bike_gpu_launch_sp4_post_solve(
                None,
                nworld,
                ctypes.c_float(dt_s),
                _ptr_buf(h_idx),
                _ptr_buf(h_cfg),
                1,
                _ptr_buf(h_spindle),
                _ptr_buf(h_sp3),
                _ptr_buf(h_sp4),
                _ptr_f32(h_qpos_post),
                _ptr_f32(h_qvel_post),
                _ptr_f32(h_xpos_post),
                _ptr_f32(h_xmat_post),
                _ptr_f32(h_xanc_post),
                int(post_b["nsensordata"]),
                _ptr_f32(h_sd) if post_b["sensordata_ptr"] != 0 else None,
                ctypes.c_void_p(h_nefc.ctypes.data) if post_b["nefc_ptr"] != 0 else None,
                int(post_b["njmax"]),
                int(post_b["efc_J_row_stride"]),
                ctypes.c_void_p(h_etype.ctypes.data) if post_b["efc_type_ptr"] != 0 else None,
                ctypes.c_void_p(h_eid.ctypes.data) if post_b["efc_id_ptr"] != 0 else None,
                _ptr_f32(h_eJ) if post_b["efc_J_ptr"] != 0 else None,
                _ptr_f32(h_ef) if post_b["efc_force_ptr"] != 0 else None,
                ctypes.c_void_p(h_nacon.ctypes.data) if post_b["nacon_ptr"] != 0 else None,
                int(post_b["nconmax"]),
                int(post_b["contact_efc_stride"]),
                ctypes.c_void_p(h_cwid.ctypes.data) if post_b["contact_worldid_ptr"] != 0 else None,
                ctypes.c_void_p(h_cgeom.ctypes.data) if post_b["contact_geom_ptr"] != 0 else None,
                ctypes.c_void_p(h_cadr.ctypes.data) if post_b["contact_efc_address_ptr"] != 0 else None,
                ctypes.c_void_p(h_cdim.ctypes.data) if post_b["contact_dim_ptr"] != 0 else None,
            )
        )
        if rc_g != 0 or rc_c != 0:
            raise RuntimeError(f"sp4_post_solve failed: gpu={rc_g}, cpu={rc_c}")

        if hasattr(cuda_lib, "bike_gpu_launch_sp4_energy_post_step") and hasattr(
            host_lib, "bike_gpu_launch_sp4_energy_post_step"
        ):
            ops["fwd_kinematics"](m_wp, d_b)
            ops["crb"](m_wp, d_b)
            wp.synchronize()
            h_M_post = np.ascontiguousarray(d_b.M.numpy(), dtype=np.float32)
            h_xpos_e = np.ascontiguousarray(d_b.xpos.numpy(), dtype=np.float32)
            h_bmass = np.ascontiguousarray(m_wp.body_mass.numpy(), dtype=np.float32)
            h_xipos = np.ascontiguousarray(d_b.xipos.numpy(), dtype=np.float32)
            h_grav = np.ascontiguousarray(m_wp.opt.gravity.numpy(), dtype=np.float32)
            h_ctrl_e = np.ascontiguousarray(d_b.ctrl.numpy(), dtype=np.float32)
            rc_eg = int(
                cuda_lib.bike_gpu_launch_sp4_energy_post_step(
                    ctypes.c_void_p(cuda_stream),
                    nworld,
                    ctypes.c_float(dt_s),
                    ctypes.c_void_p(d_idx.ptr),
                    ctypes.c_void_p(d_tyre.ptr),
                    ctypes.c_void_p(d_desc.ptr),
                    ctypes.c_void_p(tf32_ptr),
                    ctypes.c_void_p(d_cfg.ptr),
                    1,
                    ctypes.c_void_p(d_spindle.ptr),
                    ctypes.c_void_p(d_sp3.ptr),
                    ctypes.c_void_p(d_sp4.ptr),
                    ctypes.c_void_p(d_ws.ptr),
                    0,
                    int(m_world_stride),
                    ctypes.c_void_p(m_rowadr_ptr),
                    ctypes.c_void_p(m_rownnz_ptr),
                    ctypes.c_void_p(m_colind_ptr),
                    ctypes.c_void_p(d_b.M.ptr),
                    ctypes.c_void_p(d_b.qpos.ptr),
                    ctypes.c_void_p(d_b.qvel.ptr),
                    ctypes.c_void_p(d_b.xpos.ptr),
                    ctypes.c_void_p(d_b.ctrl.ptr),
                    ctypes.c_void_p(int(m_wp.body_mass.ptr)),
                    ctypes.c_void_p(int(d_b.xipos.ptr)),
                    ctypes.c_void_p(int(m_wp.opt.gravity.ptr)),
                    ctypes.c_void_p(post_b["nefc_ptr"]),
                    int(post_b["njmax"]),
                    int(post_b["efc_J_row_stride"]),
                    ctypes.c_void_p(post_b["efc_type_ptr"]),
                    ctypes.c_void_p(post_b["efc_J_ptr"]),
                    ctypes.c_void_p(post_b["efc_force_ptr"]),
                )
            )
            wp.synchronize()
            rc_ec = int(
                host_lib.bike_gpu_launch_sp4_energy_post_step(
                    None,
                    nworld,
                    ctypes.c_float(dt_s),
                    _ptr_buf(h_idx),
                    _ptr_buf(h_tyre),
                    _ptr_buf(h_desc),
                    _ptr_f32(h_tf32),
                    _ptr_buf(h_cfg),
                    1,
                    _ptr_buf(h_spindle),
                    _ptr_buf(h_sp3),
                    _ptr_buf(h_sp4),
                    _ptr_buf(h_ws),
                    0,
                    int(m_world_stride),
                    ctypes.c_void_p(h_m_rowadr.ctypes.data),
                    ctypes.c_void_p(h_m_rownnz.ctypes.data),
                    ctypes.c_void_p(h_m_colind.ctypes.data),
                    _ptr_f32(h_M_post),
                    _ptr_f32(h_qpos_post),
                    _ptr_f32(h_qvel_post),
                    _ptr_f32(h_xpos_e),
                    _ptr_f32(h_ctrl_e),
                    _ptr_f32(h_bmass),
                    _ptr_f32(h_xipos),
                    _ptr_f32(h_grav),
                    ctypes.c_void_p(h_nefc.ctypes.data) if post_b["nefc_ptr"] != 0 else None,
                    int(post_b["njmax"]),
                    int(post_b["efc_J_row_stride"]),
                    ctypes.c_void_p(h_etype.ctypes.data) if post_b["efc_type_ptr"] != 0 else None,
                    _ptr_f32(h_eJ) if post_b["efc_J_ptr"] != 0 else None,
                    _ptr_f32(h_ef) if post_b["efc_force_ptr"] != 0 else None,
                )
            )
            if rc_eg != 0 or rc_ec != 0:
                raise RuntimeError(f"sp4_energy_post_step failed: gpu={rc_eg}, cpu={rc_ec}")

    # Compare all float tail metrics (including energy ledger) + failure flags across all worlds
    g_sp4_raw = d_sp4.numpy().tobytes()
    c_sp4_raw = h_sp4.raw
    g_sp3_raw = d_sp3.numpy().tobytes()
    c_sp3_raw = h_sp3.raw

    float_keys = (
        "last_posture_min_lean_rad",
        "min_press_front_n",
        "min_press_rear_n",
        "min_saddle_force_n",
        "max_saddle_force_n",
        "sum_delivered_crank_torque_nm",
        "sum_commanded_crank_torque_nm",
        "start_root_x_m",
        "end_root_x_m",
        "end_speed_mps",
        "max_speed_mps",
        "energy_residual_j",
        "energy_scale_j",
        "max_rel_energy_residual",
    )
    g_vals = []
    c_vals = []
    flags_match = True
    for w in range(nworld):
        gm = unpack_sp4_world_metrics(g_sp4_raw[w * sizeof_sp4 : (w + 1) * sizeof_sp4])
        cm = unpack_sp4_world_metrics(c_sp4_raw[w * sizeof_sp4 : (w + 1) * sizeof_sp4])
        for fk in float_keys:
            g_vals.append(float(gm[fk]))
            c_vals.append(float(cm[fk]))
        if int(gm["energy_contract_ok"]) != int(cm["energy_contract_ok"]):
            flags_match = False
        gb = ctypes.create_string_buffer(g_sp3_raw[w * sizeof_sp3 : (w + 1) * sizeof_sp3], sizeof_sp3)
        cb = ctypes.create_string_buffer(c_sp3_raw[w * sizeof_sp3 : (w + 1) * sizeof_sp3], sizeof_sp3)
        gf = int(cuda_lib.bike_gpu_sp3_state_failure_flags(_ptr_buf(gb)))
        cf = int(host_lib.bike_gpu_sp3_state_failure_flags(_ptr_buf(cb)))
        if gf != cf:
            flags_match = False

    st_metrics = _f32_ulp_stats(np.asarray(g_vals, dtype=np.float32), np.asarray(c_vals, dtype=np.float32))
    max_ulp_by_array["sp4_metrics"] = int(st_metrics["max_ulp"])
    last_stats["sp4_metrics"] = st_metrics

    overall_max_ulp = max(max_ulp_by_array.values())
    passed = overall_max_ulp <= 1 and flags_match
    return {
        "stage": "SP3_SP4_6kernels",
        "backend": backend,
        "station_count": station_count,
        "nworld": nworld,
        "n_steps": n_steps,
        "passed": bool(passed),
        "flags_match": bool(flags_match),
        "overall_max_ulp": int(overall_max_ulp),
        "max_ulp_by_array": max_ulp_by_array,
        "last_step_stats": last_stats,
    }


def run_level_a_sp4_m0_parity(
    *,
    cuda_lib: ctypes.CDLL,
    host_lib: ctypes.CDLL,
    xml_path: Path,
    qpos0: np.ndarray,
    qvel0: np.ndarray,
    qacc_ws0: np.ndarray | None,
    backend: str,
    station_count: int,
    nworld: int = 32,
    n_steps: int = 5,
    device: str = "cuda:0",
) -> dict[str, Any]:
    """Compare SP4 ``SpindleM0`` Armijo Newton + ``energy_post_step`` on CUDA vs Host-float."""
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
    scales = generate_parameter_sweep_scales(nworld=nworld, seed=42)
    manifest_path = xml_path.parent / "manifest.json"
    cfg_bytes = _build_sp4_config_bytes(
        lib=cuda_lib,
        m_mj=m,
        nworld=nworld,
        surface_label=1,
        v0=0.0,
        fork_psi_scale=scales["fork_psi_scale"],
        rebound_scale=scales["rebound_scale"],
        shock_rate_scale=scales["shock_rate_scale"],
        tire_stiff_scale=scales["tire_stiff_scale"],
        physics_config_json=xml_path.parent / "physics_config.json",
        manifest_json=manifest_path if manifest_path.is_file() else None,
    )
    spindle_bytes = build_spindle_params_bytes(
        host_lib=host_lib,
        xml_path=xml_path,
        strength_json_path=DEFAULT_STRENGTH_JSON,
        envelope_json_path=DEFAULT_ENVELOPE_JSON,
        control_period_steps=1,
    )
    model_b, desc_b, f32_np, i32_np = build_tyre_and_terrain_buffers(
        host_lib=host_lib,
        xml_path=xml_path,
        backend=backend,
        station_count=station_count,
        idx_bytes=idx_bytes,
        surface_label=1,
    )
    sizeof_ws = int(cuda_lib.bike_gpu_sizeof_world_state())
    sizeof_sp3 = int(cuda_lib.bike_gpu_sizeof_sp3_world_state())
    sizeof_sp4 = int(cuda_lib.bike_gpu_sizeof_sp4_world_state())
    sizeof_m0 = int(cuda_lib.bike_gpu_sizeof_spindle_m0_scratch())

    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_cfg = _allocate_and_copy_device_bytes(cfg_bytes, device=device)
    d_spindle = _allocate_and_copy_device_bytes(spindle_bytes, device=device)
    d_tyre = _allocate_and_copy_device_bytes(model_b, device=device)
    d_desc = _allocate_and_copy_device_bytes(desc_b, device=device)
    d_tf32 = wp.array(f32_np if f32_np.size > 0 else np.zeros(1, dtype=np.float32), dtype=wp.float32, device=device)
    d_ti32 = wp.array(i32_np if i32_np.size > 0 else np.zeros(1, dtype=np.int32), dtype=wp.int32, device=device)
    d_ws = wp.zeros(nworld * sizeof_ws, dtype=wp.uint8, device=device)
    d_sp3 = wp.zeros(nworld * sizeof_sp3, dtype=wp.uint8, device=device)
    d_sp4 = wp.zeros(nworld * sizeof_sp4, dtype=wp.uint8, device=device)
    d_m0 = wp.zeros(nworld * sizeof_m0, dtype=wp.uint8, device=device)
    d_any = wp.zeros(1, dtype=wp.int32, device=device)

    m_wp = _configure_sp4_mjwarp_model(m, device=device, disable_contacts=False)
    expand_mjwarp_tendon_range_and_frictionloss(m, m_wp, nworld, device=device)
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=64, njmax=128)
    mjwarp.forward(m_wp, d_b)
    if qacc_ws0 is not None and qacc_ws0.size == int(m.nv):
        d_b.qacc_warmstart.assign(wp.array(np.tile(qacc_ws0.astype(np.float32)[None, :], (nworld, 1)), dtype=wp.float32, device=device))
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

    h_idx = ctypes.create_string_buffer(idx_bytes, len(idx_bytes))
    h_cfg = ctypes.create_string_buffer(cfg_bytes, len(cfg_bytes))
    h_spindle = ctypes.create_string_buffer(spindle_bytes, len(spindle_bytes))
    h_tyre = ctypes.create_string_buffer(model_b, len(model_b))
    h_desc = ctypes.create_string_buffer(desc_b, len(desc_b))
    h_tf32 = np.ascontiguousarray(f32_np if f32_np.size > 0 else np.zeros(1, dtype=np.float32), dtype=np.float32)
    h_ti32 = np.ascontiguousarray(i32_np if i32_np.size > 0 else np.zeros(1, dtype=np.int32), dtype=np.int32)
    h_ws = ctypes.create_string_buffer(nworld * sizeof_ws)
    h_sp3 = ctypes.create_string_buffer(nworld * sizeof_sp3)
    h_sp4 = ctypes.create_string_buffer(nworld * sizeof_sp4)
    h_m0 = ctypes.create_string_buffer(nworld * sizeof_m0)
    h_any = np.zeros(1, dtype=np.int32)
    h_tr = np.ascontiguousarray(m_wp.tendon_range.numpy().reshape(-1), dtype=np.float32)
    h_fl = np.ascontiguousarray(m_wp.dof_frictionloss.numpy().reshape(-1), dtype=np.float32)
    h_rowadr = np.ascontiguousarray(m_wp.M_rowadr.numpy(), dtype=np.int32)
    h_rownnz = np.ascontiguousarray(m_wp.M_rownnz.numpy(), dtype=np.int32)
    h_colind = np.ascontiguousarray(m_wp.M_colind.numpy(), dtype=np.int32)
    h_submass = np.ascontiguousarray(submass_arr.numpy(), dtype=np.float32) if submass_arr is not None else np.zeros(1, dtype=np.float32)
    h_qpos0 = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)

    rc_g = int(
        cuda_lib.bike_gpu_launch_sp4_m0_init(
            ctypes.c_void_p(cuda_stream),
            nworld,
            ctypes.c_void_p(d_idx.ptr),
            ctypes.c_void_p(d_cfg.ptr),
            1,
            ctypes.c_void_p(d_sp3.ptr),
            ctypes.c_void_p(d_sp4.ptr),
            ctypes.c_void_p(d_m0.ptr),
            ctypes.c_void_p(d_ws.ptr),
            ctypes.c_void_p(d_b.qpos.ptr),
            ctypes.c_float(0.0),
            int(m.ntendon),
            ctypes.c_void_p(tr_ptr),
            int(tr_stride),
        )
    )
    wp.synchronize()
    rc_c = int(
        host_lib.bike_gpu_launch_sp4_m0_init(
            None,
            nworld,
            _ptr_buf(h_idx),
            _ptr_buf(h_cfg),
            1,
            _ptr_buf(h_sp3),
            _ptr_buf(h_sp4),
            _ptr_buf(h_m0),
            _ptr_buf(h_ws),
            _ptr_f32(h_qpos0),
            ctypes.c_float(0.0),
            int(m.ntendon),
            _ptr_f32(h_tr),
            int(tr_stride),
        )
    )
    if rc_g != 0 or rc_c != 0:
        raise RuntimeError(f"sp4_m0_init failed: gpu={rc_g}, cpu={rc_c}")

    max_ulp_by_array = {"qfrc_applied": 0, "ctrl": 0, "sp4_metrics": 0}
    last_stats: dict[str, Any] = {}

    def _solve_once() -> None:
        ops["fwd_actuation"](m_wp, d_b)
        ops["fwd_acceleration"](m_wp, d_b, factorize=False)
        ops["solve"](m_wp, d_b)
        wp.synchronize()

    for _step in range(n_steps):
        ops["fwd_kinematics"](m_wp, d_b)
        ops["crb"](m_wp, d_b)
        ops["tendon_armature"](m_wp, d_b)
        ops["factor_m"](m_wp, d_b)
        ops["collision"](m_wp, d_b)
        ops["transmission"](m_wp, d_b)
        ops["fwd_velocity"](m_wp, d_b)
        ops["sensor_vel"](m_wp, d_b)
        wp.synchronize()

        h_qpos = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)
        h_qvel = np.ascontiguousarray(d_b.qvel.numpy(), dtype=np.float32)
        h_bias = np.ascontiguousarray(d_b.qfrc_bias.numpy(), dtype=np.float32)
        h_xpos = np.ascontiguousarray(d_b.xpos.numpy(), dtype=np.float32)
        h_xmat = np.ascontiguousarray(d_b.xmat.numpy(), dtype=np.float32)
        h_xanc = np.ascontiguousarray(d_b.xanchor.numpy(), dtype=np.float32)
        h_xaxs = np.ascontiguousarray(d_b.xaxis.numpy(), dtype=np.float32)
        h_site = np.ascontiguousarray(d_b.site_xpos.numpy(), dtype=np.float32)
        h_scom = np.ascontiguousarray(d_b.subtree_com.numpy(), dtype=np.float32)
        h_M = np.ascontiguousarray(d_b.M.numpy(), dtype=np.float32)
        h_qfrc = np.zeros_like(d_b.qfrc_applied.numpy(), dtype=np.float32)
        h_ctrl = np.zeros_like(d_b.ctrl.numpy(), dtype=np.float32)
        h_qws = np.ascontiguousarray(d_b.qacc_warmstart.numpy(), dtype=np.float32)
        d_any.zero_()
        h_any.fill(0)

        rc_g = int(
            cuda_lib.bike_gpu_launch_sp4_m0_pre_solve_and_begin(
                ctypes.c_void_p(cuda_stream),
                nworld,
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_tyre.ptr),
                ctypes.c_void_p(d_desc.ptr),
                ctypes.c_void_p(d_tf32.ptr),
                ctypes.c_void_p(d_ti32.ptr),
                ctypes.c_void_p(d_cfg.ptr),
                1,
                ctypes.c_void_p(d_spindle.ptr),
                ctypes.c_void_p(d_sp3.ptr),
                ctypes.c_void_p(d_sp4.ptr),
                ctypes.c_void_p(d_m0.ptr),
                ctypes.c_void_p(d_ws.ptr),
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
                ctypes.c_void_p(d_any.ptr),
            )
        )
        wp.synchronize()
        rc_c = int(
            host_lib.bike_gpu_launch_sp4_m0_pre_solve_and_begin(
                None,
                nworld,
                ctypes.c_float(dt_s),
                _ptr_buf(h_idx),
                _ptr_buf(h_tyre),
                _ptr_buf(h_desc),
                _ptr_f32(h_tf32),
                ctypes.c_void_p(h_ti32.ctypes.data),
                _ptr_buf(h_cfg),
                1,
                _ptr_buf(h_spindle),
                _ptr_buf(h_sp3),
                _ptr_buf(h_sp4),
                _ptr_buf(h_m0),
                _ptr_buf(h_ws),
                _ptr_f32(h_qpos),
                _ptr_f32(h_qvel),
                _ptr_f32(h_bias),
                _ptr_f32(h_xpos),
                _ptr_f32(h_xmat),
                _ptr_f32(h_xanc),
                _ptr_f32(h_xaxs),
                _ptr_f32(h_site),
                _ptr_f32(h_scom),
                _ptr_f32(h_submass) if submass_ptr != 0 else None,
                0,
                int(m_world_stride),
                ctypes.c_void_p(h_rowadr.ctypes.data),
                ctypes.c_void_p(h_rownnz.ctypes.data),
                ctypes.c_void_p(h_colind.ctypes.data),
                _ptr_f32(h_M),
                _ptr_f32(h_qfrc),
                _ptr_f32(h_ctrl),
                int(m.ntendon),
                _ptr_f32(h_tr) if tr_ptr != 0 else None,
                int(tr_stride),
                _ptr_f32(h_fl) if fl_ptr != 0 else None,
                int(fl_stride),
                _ptr_f32(h_qws),
                ctypes.c_void_p(h_any.ctypes.data),
            )
        )
        if rc_g != 0 or rc_c != 0:
            raise RuntimeError(f"sp4_m0_pre_solve_and_begin failed: gpu={rc_g}, cpu={rc_c}")

        st_q = _f32_ulp_stats(d_b.qfrc_applied.numpy(), h_qfrc)
        st_c = _f32_ulp_stats(d_b.ctrl.numpy(), h_ctrl)
        max_ulp_by_array["qfrc_applied"] = max(max_ulp_by_array["qfrc_applied"], int(st_q["max_ulp"]))
        max_ulp_by_array["ctrl"] = max(max_ulp_by_array["ctrl"], int(st_c["max_ulp"]))

        ops["make_constraint"](m_wp, d_b)
        _solve_once()

        for _p in range(37):
            g_need = int(d_any.numpy()[0])
            c_need = int(h_any[0])
            if g_need != c_need:
                raise RuntimeError(f"M0 any_needs_solve divergence: gpu={g_need}, cpu={c_need}")
            if g_need == 0:
                break
            d_any.zero_()
            h_any.fill(0)
            h_qpos_s = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)
            h_qvel_s = np.ascontiguousarray(d_b.qvel.numpy(), dtype=np.float32)
            h_bias_s = np.ascontiguousarray(d_b.qfrc_bias.numpy(), dtype=np.float32)
            h_qacc_s = np.ascontiguousarray(d_b.qacc.numpy(), dtype=np.float32)
            h_M_s = np.ascontiguousarray(d_b.M.numpy(), dtype=np.float32)
            h_ctrl_s = np.ascontiguousarray(d_b.ctrl.numpy(), dtype=np.float32)
            h_qws_s = np.ascontiguousarray(d_b.qacc_warmstart.numpy(), dtype=np.float32)
            rc_sg = int(
                cuda_lib.bike_gpu_launch_sp4_m0_step(
                    ctypes.c_void_p(cuda_stream),
                    nworld,
                    int(m.nq),
                    int(m.nv),
                    int(m.nu),
                    ctypes.c_void_p(d_idx.ptr),
                    ctypes.c_void_p(d_cfg.ptr),
                    1,
                    ctypes.c_void_p(d_sp3.ptr),
                    ctypes.c_void_p(d_spindle.ptr),
                    ctypes.c_void_p(d_sp4.ptr),
                    ctypes.c_void_p(d_m0.ptr),
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
                    ctypes.c_void_p(d_any.ptr),
                )
            )
            wp.synchronize()
            rc_sc = int(
                host_lib.bike_gpu_launch_sp4_m0_step(
                    None,
                    nworld,
                    int(m.nq),
                    int(m.nv),
                    int(m.nu),
                    _ptr_buf(h_idx),
                    _ptr_buf(h_cfg),
                    1,
                    _ptr_buf(h_sp3),
                    _ptr_buf(h_spindle),
                    _ptr_buf(h_sp4),
                    _ptr_buf(h_m0),
                    _ptr_f32(h_qpos_s),
                    _ptr_f32(h_qvel_s),
                    _ptr_f32(h_bias_s),
                    _ptr_f32(h_qacc_s),
                    0,
                    int(m_world_stride),
                    ctypes.c_void_p(h_rowadr.ctypes.data),
                    ctypes.c_void_p(h_rownnz.ctypes.data),
                    ctypes.c_void_p(h_colind.ctypes.data),
                    _ptr_f32(h_M_s),
                    _ptr_f32(h_ctrl_s),
                    _ptr_f32(h_qws_s),
                    ctypes.c_void_p(h_any.ctypes.data),
                )
            )
            if rc_sg != 0 or rc_sc != 0:
                raise RuntimeError(f"sp4_m0_step failed: gpu={rc_sg}, cpu={rc_sc}")
            st_c = _f32_ulp_stats(d_b.ctrl.numpy(), h_ctrl_s)
            max_ulp_by_array["ctrl"] = max(max_ulp_by_array["ctrl"], int(st_c["max_ulp"]))
            if int(d_any.numpy()[0]) != int(h_any[0]):
                raise RuntimeError(f"M0 post-step any_needs_solve divergence: gpu={int(d_any.numpy()[0])}, cpu={int(h_any[0])}")
            if int(d_any.numpy()[0]) == 0:
                break
            _solve_once()

        ops["sensor_acc"](m_wp, d_b)
        ops["implicit"](m_wp, d_b)
        wp.synchronize()

        h_qpos_post = np.ascontiguousarray(d_b.qpos.numpy(), dtype=np.float32)
        h_qvel_post = np.ascontiguousarray(d_b.qvel.numpy(), dtype=np.float32)
        h_xpos_post = np.ascontiguousarray(d_b.xpos.numpy(), dtype=np.float32)
        h_xmat_post = np.ascontiguousarray(d_b.xmat.numpy(), dtype=np.float32)
        h_xanc_post = np.ascontiguousarray(d_b.xanchor.numpy(), dtype=np.float32)
        h_sd = np.ascontiguousarray(d_b.sensordata.numpy(), dtype=np.float32) if post_b["sensordata_ptr"] != 0 else np.zeros(1, dtype=np.float32)
        efc = getattr(d_b, "efc", None)
        h_nefc = np.ascontiguousarray(d_b.nefc.numpy(), dtype=np.int32) if post_b["nefc_ptr"] != 0 else np.zeros(nworld, dtype=np.int32)
        h_etype = np.ascontiguousarray(efc.type.numpy(), dtype=np.int32) if efc is not None and post_b["efc_type_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_eid = np.ascontiguousarray(efc.id.numpy(), dtype=np.int32) if efc is not None and post_b["efc_id_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_eJ = np.ascontiguousarray(efc.J.numpy(), dtype=np.float32) if efc is not None and post_b["efc_J_ptr"] != 0 else np.zeros(1, dtype=np.float32)
        h_ef = np.ascontiguousarray(efc.force.numpy(), dtype=np.float32) if efc is not None and post_b["efc_force_ptr"] != 0 else np.zeros(1, dtype=np.float32)
        con = getattr(d_b, "contact", None)
        h_nacon = np.ascontiguousarray(d_b.nacon.numpy(), dtype=np.int32) if post_b["nacon_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_cwid = np.ascontiguousarray(con.worldid.numpy(), dtype=np.int32) if con is not None and post_b["contact_worldid_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_cgeom = np.ascontiguousarray(con.geom.numpy(), dtype=np.int32) if con is not None and post_b["contact_geom_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_cadr = np.ascontiguousarray(con.efc_address.numpy(), dtype=np.int32) if con is not None and post_b["contact_efc_address_ptr"] != 0 else np.zeros(1, dtype=np.int32)
        h_cdim = np.ascontiguousarray(con.dim.numpy(), dtype=np.int32) if con is not None and post_b["contact_dim_ptr"] != 0 else np.zeros(1, dtype=np.int32)

        rc_g = int(
            cuda_lib.bike_gpu_launch_sp4_m0_post_solve(
                ctypes.c_void_p(cuda_stream),
                nworld,
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_cfg.ptr),
                1,
                ctypes.c_void_p(d_spindle.ptr),
                ctypes.c_void_p(d_sp3.ptr),
                ctypes.c_void_p(d_sp4.ptr),
                ctypes.c_void_p(d_m0.ptr),
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
        wp.synchronize()
        rc_c = int(
            host_lib.bike_gpu_launch_sp4_m0_post_solve(
                None,
                nworld,
                ctypes.c_float(dt_s),
                _ptr_buf(h_idx),
                _ptr_buf(h_cfg),
                1,
                _ptr_buf(h_spindle),
                _ptr_buf(h_sp3),
                _ptr_buf(h_sp4),
                _ptr_buf(h_m0),
                _ptr_f32(h_qpos_post),
                _ptr_f32(h_qvel_post),
                _ptr_f32(h_xpos_post),
                _ptr_f32(h_xmat_post),
                _ptr_f32(h_xanc_post),
                int(post_b["nsensordata"]),
                _ptr_f32(h_sd) if post_b["sensordata_ptr"] != 0 else None,
                ctypes.c_void_p(h_nefc.ctypes.data) if post_b["nefc_ptr"] != 0 else None,
                int(post_b["njmax"]),
                int(post_b["efc_J_row_stride"]),
                ctypes.c_void_p(h_etype.ctypes.data) if post_b["efc_type_ptr"] != 0 else None,
                ctypes.c_void_p(h_eid.ctypes.data) if post_b["efc_id_ptr"] != 0 else None,
                _ptr_f32(h_eJ) if post_b["efc_J_ptr"] != 0 else None,
                _ptr_f32(h_ef) if post_b["efc_force_ptr"] != 0 else None,
                ctypes.c_void_p(h_nacon.ctypes.data) if post_b["nacon_ptr"] != 0 else None,
                int(post_b["nconmax"]),
                int(post_b["contact_efc_stride"]),
                ctypes.c_void_p(h_cwid.ctypes.data) if post_b["contact_worldid_ptr"] != 0 else None,
                ctypes.c_void_p(h_cgeom.ctypes.data) if post_b["contact_geom_ptr"] != 0 else None,
                ctypes.c_void_p(h_cadr.ctypes.data) if post_b["contact_efc_address_ptr"] != 0 else None,
                ctypes.c_void_p(h_cdim.ctypes.data) if post_b["contact_dim_ptr"] != 0 else None,
            )
        )
        if rc_g != 0 or rc_c != 0:
            raise RuntimeError(f"sp4_m0_post_solve failed: gpu={rc_g}, cpu={rc_c}")

        ops["fwd_kinematics"](m_wp, d_b)
        ops["crb"](m_wp, d_b)
        wp.synchronize()
        h_M_post = np.ascontiguousarray(d_b.M.numpy(), dtype=np.float32)
        h_xpos_e = np.ascontiguousarray(d_b.xpos.numpy(), dtype=np.float32)
        h_bmass = np.ascontiguousarray(m_wp.body_mass.numpy(), dtype=np.float32)
        h_xipos = np.ascontiguousarray(d_b.xipos.numpy(), dtype=np.float32)
        h_grav = np.ascontiguousarray(m_wp.opt.gravity.numpy(), dtype=np.float32)
        h_ctrl_e = np.ascontiguousarray(d_b.ctrl.numpy(), dtype=np.float32)
        rc_eg = int(
            cuda_lib.bike_gpu_launch_sp4_energy_post_step(
                ctypes.c_void_p(cuda_stream),
                nworld,
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_tyre.ptr),
                ctypes.c_void_p(d_desc.ptr),
                ctypes.c_void_p(d_tf32.ptr),
                ctypes.c_void_p(d_cfg.ptr),
                1,
                ctypes.c_void_p(d_spindle.ptr),
                ctypes.c_void_p(d_sp3.ptr),
                ctypes.c_void_p(d_sp4.ptr),
                ctypes.c_void_p(d_ws.ptr),
                0,
                int(m_world_stride),
                ctypes.c_void_p(m_rowadr_ptr),
                ctypes.c_void_p(m_rownnz_ptr),
                ctypes.c_void_p(m_colind_ptr),
                ctypes.c_void_p(d_b.M.ptr),
                ctypes.c_void_p(d_b.qpos.ptr),
                ctypes.c_void_p(d_b.qvel.ptr),
                ctypes.c_void_p(d_b.xpos.ptr),
                ctypes.c_void_p(d_b.ctrl.ptr),
                ctypes.c_void_p(int(m_wp.body_mass.ptr)),
                ctypes.c_void_p(int(d_b.xipos.ptr)),
                ctypes.c_void_p(int(m_wp.opt.gravity.ptr)),
                ctypes.c_void_p(post_b["nefc_ptr"]),
                int(post_b["njmax"]),
                int(post_b["efc_J_row_stride"]),
                ctypes.c_void_p(post_b["efc_type_ptr"]),
                ctypes.c_void_p(post_b["efc_J_ptr"]),
                ctypes.c_void_p(post_b["efc_force_ptr"]),
            )
        )
        wp.synchronize()
        rc_ec = int(
            host_lib.bike_gpu_launch_sp4_energy_post_step(
                None,
                nworld,
                ctypes.c_float(dt_s),
                _ptr_buf(h_idx),
                _ptr_buf(h_tyre),
                _ptr_buf(h_desc),
                _ptr_f32(h_tf32),
                _ptr_buf(h_cfg),
                1,
                _ptr_buf(h_spindle),
                _ptr_buf(h_sp3),
                _ptr_buf(h_sp4),
                _ptr_buf(h_ws),
                0,
                int(m_world_stride),
                ctypes.c_void_p(h_rowadr.ctypes.data),
                ctypes.c_void_p(h_rownnz.ctypes.data),
                ctypes.c_void_p(h_colind.ctypes.data),
                _ptr_f32(h_M_post),
                _ptr_f32(h_qpos_post),
                _ptr_f32(h_qvel_post),
                _ptr_f32(h_xpos_e),
                _ptr_f32(h_ctrl_e),
                _ptr_f32(h_bmass),
                _ptr_f32(h_xipos),
                _ptr_f32(h_grav),
                ctypes.c_void_p(h_nefc.ctypes.data) if post_b["nefc_ptr"] != 0 else None,
                int(post_b["njmax"]),
                int(post_b["efc_J_row_stride"]),
                ctypes.c_void_p(h_etype.ctypes.data) if post_b["efc_type_ptr"] != 0 else None,
                _ptr_f32(h_eJ) if post_b["efc_J_ptr"] != 0 else None,
                _ptr_f32(h_ef) if post_b["efc_force_ptr"] != 0 else None,
            )
        )
        if rc_eg != 0 or rc_ec != 0:
            raise RuntimeError(f"sp4_energy_post_step (M0) failed: gpu={rc_eg}, cpu={rc_ec}")

    g_sp4_raw = d_sp4.numpy().tobytes()
    c_sp4_raw = h_sp4.raw
    g_sp3_raw = d_sp3.numpy().tobytes()
    c_sp3_raw = h_sp3.raw
    float_keys = (
        "last_posture_min_lean_rad",
        "min_press_front_n",
        "min_press_rear_n",
        "min_saddle_force_n",
        "max_saddle_force_n",
        "sum_delivered_crank_torque_nm",
        "sum_commanded_crank_torque_nm",
        "start_root_x_m",
        "end_root_x_m",
        "end_speed_mps",
        "max_speed_mps",
        "energy_residual_j",
        "energy_scale_j",
        "max_rel_energy_residual",
    )
    g_vals = []
    c_vals = []
    flags_match = True
    for w in range(nworld):
        gm = unpack_sp4_world_metrics(g_sp4_raw[w * sizeof_sp4 : (w + 1) * sizeof_sp4])
        cm = unpack_sp4_world_metrics(c_sp4_raw[w * sizeof_sp4 : (w + 1) * sizeof_sp4])
        for fk in float_keys:
            g_vals.append(float(gm[fk]))
            c_vals.append(float(cm[fk]))
        if (
            int(gm["energy_contract_ok"]) != int(cm["energy_contract_ok"])
            or int(gm["max_m0_passes_used"]) != int(cm["max_m0_passes_used"])
        ):
            flags_match = False
        gb = ctypes.create_string_buffer(g_sp3_raw[w * sizeof_sp3 : (w + 1) * sizeof_sp3], sizeof_sp3)
        cb = ctypes.create_string_buffer(c_sp3_raw[w * sizeof_sp3 : (w + 1) * sizeof_sp3], sizeof_sp3)
        gf = int(cuda_lib.bike_gpu_sp3_state_failure_flags(_ptr_buf(gb)))
        cf = int(host_lib.bike_gpu_sp3_state_failure_flags(_ptr_buf(cb)))
        if gf != cf:
            flags_match = False
    st_metrics = _f32_ulp_stats(np.asarray(g_vals, dtype=np.float32), np.asarray(c_vals, dtype=np.float32))
    max_ulp_by_array["sp4_metrics"] = int(st_metrics["max_ulp"])
    last_stats["sp4_metrics"] = st_metrics
    overall_max_ulp = max(max_ulp_by_array.values())
    return {
        "stage": "SP4_M0_Armijo",
        "backend": backend,
        "station_count": station_count,
        "nworld": nworld,
        "n_steps": n_steps,
        "passed": bool(overall_max_ulp <= 1 and flags_match),
        "flags_match": bool(flags_match),
        "overall_max_ulp": int(overall_max_ulp),
        "max_ulp_by_array": max_ulp_by_array,
        "last_step_stats": last_stats,
    }


def run_level_c_closed_loop_10scenarios(
    *,
    cuda_lib: ctypes.CDLL,
    host_lib: ctypes.CDLL,
    bundle_dir: Path,
    device: str = "cuda:0",
    spindle_solver: str = "both",
) -> dict[str, Any]:
    """Run all 10 closed-loop scenarios (S1..S4 + G1..G6) on CUDA mujoco_warp (``m2k2`` & ``m0``)."""
    manifest_path = bundle_dir / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"Missing manifest.json in {bundle_dir}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenarios = list(manifest["scenarios"])
    solvers = ("m2k2", "m0") if spindle_solver == "both" else (spindle_solver,)

    results = []
    all_passed = True
    for solver_mode in solvers:
        for sc in scenarios:
            sc_id = str(sc["id"])
            sub_dir = bundle_dir / sc_id
            xml12 = sub_dir / "model_12gears.xml"
            qpos0, qvel0, qacc_ws0 = _load_step_closure_row0(sub_dir)
            m = load_model_with_assets(xml12)
            sub_mf = sub_dir / "manifest.json"
            cfg_bytes = _build_sp4_config_bytes(
                lib=cuda_lib,
                m_mj=m,
                nworld=1,
                surface_label=int(sc["surface_label"]),
                v0=float(sc["v0"]),
                human_torque_nm=sc.get("human_torque_nm"),
                motor_torque_nm=sc.get("motor_torque_nm"),
                physics_config_json=sub_dir / "physics_config.json",
                manifest_json=sub_mf if sub_mf.is_file() else None,
            )
            n_steps = int(sc["target_steps"])
            backend = str(sc.get("tyre_backend", sc.get("backend", "compliant_2d")))
            station_count = int(sc["station_count"])
            init_grade = float(sc.get("init_grade", 0.0))

            res = _run_cuda_sp4_batch(
                lib=cuda_lib,
                host_lib=host_lib,
                xml_path=xml12,
                qpos0=qpos0,
                qvel0=qvel0,
                qacc_ws0=qacc_ws0,
                nworld=1,
                n_steps=n_steps,
                warmup_steps=2,
                surface_label=int(sc["surface_label"]),
                backend=backend,
                station_count=station_count,
                control_period_steps=1,
                cfg_bytes=cfg_bytes,
                device=device,
                init_grade=init_grade,
                spindle_solver=solver_mode,
            )
            wm = res["world0_metrics"]
            dist = float(wm["distance_m"])
            m0_dist_raw = sc.get("m0_distance_m")
            m0_dist = float(m0_dist_raw) if m0_dist_raw is not None else None
            rel_err = abs(dist - m0_dist) / max(1.0, abs(m0_dist)) if m0_dist is not None else 0.0
            max_rel_err = float(sc.get("max_rel_dist_err", 0.01))
            diag_rough = bool(sc.get("diagnostic_rough", False))
            min_press = min(float(wm["min_press_front_n"]), float(wm["min_press_rear_n"]))
            max_rise_f = wm["max_rising_front_excess_n"]
            max_rise_r = wm["max_rising_rear_excess_n"]
            max_rise = (
                max(float(max_rise_f), float(max_rise_r))
                if (max_rise_f is not None and max_rise_r is not None)
                else (float(max_rise_f) if max_rise_f is not None else (float(max_rise_r) if max_rise_r is not None else 0.0))
            )
            min_saddle = float(wm["min_saddle_force_n"])
            max_saddle = float(wm["max_saddle_force_n"])
            crash_det = bool(res["world0_crash_detected"])
            bal_lost = bool(res["world0_balance_lost"])
            m0_crash = bool(sc.get("m0_crash_detected", False))
            m0_bal = bool(sc.get("m0_balance_lost", False))
            max_rel_e = float(wm["max_rel_energy_residual"])
            e_ok = int(wm["energy_contract_ok"])

            bounds_ok = (
                (max_saddle <= 1750.0)
                if diag_rough
                else (
                    min_press >= -6.0
                    and max_rise <= 6.0
                    and min_saddle >= -6.0
                    and max_saddle <= 1750.0
                )
            )
            passed = (
                bool(res["all_finite"])
                and (crash_det == m0_crash)
                and (bal_lost == m0_bal)
                and bounds_ok
                and (m0_dist is None or rel_err <= max_rel_err)
                and max_rel_e <= 0.05
                and e_ok == 1
            )
            if not passed:
                all_passed = False

            row = {
                "id": sc_id,
                "spindle_solver": solver_mode,
                "backend": backend,
                "station_count": station_count,
                "n_steps": n_steps,
                "passed": bool(passed),
                "distance_m": dist,
                "m0_distance_m": m0_dist,
                "gpu_host_f32_distance_m": float(sc.get("gpu_host_f32_distance_m", 0.0)),
                "distance_rel_err": rel_err,
                "max_rel_dist_err": max_rel_err,
                "end_speed_mps": float(wm["end_speed_mps"]),
                "m0_end_speed_mps": float(sc.get("m0_end_speed_mps", 0.0)),
                "min_press_n": min_press,
                "max_rising_excess_n": max_rise,
                "min_saddle_force_n": min_saddle,
                "max_saddle_force_n": max_saddle,
                "energy_residual_j": float(wm["energy_residual_j"]),
                "energy_scale_j": float(wm["energy_scale_j"]),
                "max_rel_energy_residual": max_rel_e,
                "energy_contract_ok": e_ok,
                "solver_mode": int(wm["solver_mode"]),
                "max_m0_passes_used": int(wm["max_m0_passes_used"]),
                "current_m0_passes": int(wm["current_m0_passes"]),
                "crash_detected": crash_det,
                "m0_crash_detected": m0_crash,
                "balance_lost": bal_lost,
                "m0_balance_lost": m0_bal,
                "wall_time_ms": float(res["wall_time_ms"]),
                "realtime_factor_x": float(res["realtime_factor_x"]),
                "all_finite": bool(res["all_finite"]),
            }
            results.append(row)
            m0_s = f"{m0_dist:.4f}" if m0_dist is not None else "N/A"
            print(
                f"  [{sc_id} {solver_mode}] pass={row['passed']} dist={dist:.4f} m0={m0_s} "
                f"rel={rel_err * 100.0:.4f}% (lim {max_rel_err * 100.0:.2f}%) "
                f"e_rel={max_rel_e * 100.0:.4f}% e_ok={e_ok} m0_passes={wm['max_m0_passes_used']} "
                f"crash={crash_det}/{m0_crash} bal={bal_lost}/{m0_bal} "
                f"saddle={min_saddle:+.1f}..{max_saddle:.1f} N wall={res['wall_time_ms']:.1f} ms",
                flush=True,
            )

    return {
        "all_passed": bool(all_passed),
        "scenarios": results,
    }


run_level_c_closed_loop_8scenarios = run_level_c_closed_loop_10scenarios


def run_batched_throughput_sanity(
    *,
    cuda_lib: ctypes.CDLL,
    host_lib: ctypes.CDLL,
    bundle_dir: Path,
    nworld: int = 2048,
    n_steps: int = 200,
    device: str = "cuda:0",
) -> list[dict[str, Any]]:
    """Run nworld=2048 SP4 4x-solve CUDA Graph throughput across all 3 tyre backends."""
    scales = generate_parameter_sweep_scales(nworld=nworld, seed=42)
    runs = []
    for bk, st_cnt, sc_id in (
        ("compliant_2d", 0, "S1_flat"),
        ("distributed_2d_reference", 128, "G3_distributed_flat_30s"),
        ("pneumatic", 64, "G5_pneumatic_flat_30s"),
    ):
        sub_dir = bundle_dir / sc_id
        if not (sub_dir / "model_12gears.xml").is_file():
            sub_dir = bundle_dir / "S1_flat"
        xml12 = sub_dir / "model_12gears.xml"
        qpos0, qvel0, qacc_ws0 = _load_step_closure_row0(sub_dir)
        m = load_model_with_assets(xml12)
        sub_mf = sub_dir / "manifest.json"
        cfg_bytes = _build_sp4_config_bytes(
            lib=cuda_lib,
            m_mj=m,
            nworld=nworld,
            surface_label=1,
            v0=0.0,
            fork_psi_scale=scales["fork_psi_scale"],
            rebound_scale=scales["rebound_scale"],
            shock_rate_scale=scales["shock_rate_scale"],
            tire_stiff_scale=scales["tire_stiff_scale"],
            physics_config_json=sub_dir / "physics_config.json",
            manifest_json=sub_mf if sub_mf.is_file() else None,
        )
        res = _run_cuda_sp4_batch(
            lib=cuda_lib,
            host_lib=host_lib,
            xml_path=xml12,
            qpos0=qpos0,
            qvel0=qvel0,
            qacc_ws0=qacc_ws0,
            nworld=nworld,
            n_steps=n_steps,
            warmup_steps=5,
            surface_label=1,
            backend=bk,
            station_count=st_cnt,
            control_period_steps=1,
            cfg_bytes=cfg_bytes,
            device=device,
        )
        runs.append(res)
        print(
            f"  [Throughput {bk} (stations={st_cnt})] nworld={nworld} n_steps={n_steps} "
            f"steps/s={res['steps_per_s']:,.0f} RTF={res['realtime_factor_x']:.1f}x finite={res['all_finite']}",
            flush=True,
        )
    return runs


def main(argv: list[str] | None = None) -> int:
    _require_mujoco_3x()
    parser = argparse.ArgumentParser(description="Verify CUDA vs Host-float (Level A) and CPU FP64 (Level C) parity.")
    parser.add_argument("--bundle-dir", type=Path, required=True, help="Directory with pre-exported S1..S4 + G1..G6 scenarios + manifest.json")
    parser.add_argument("--out-json", type=Path, default=None, help="Output JSON report path")
    parser.add_argument("--nworld-throughput", type=int, default=2048)
    parser.add_argument("--steps-throughput", type=int, default=200)
    parser.add_argument(
        "--spindle-solver",
        choices=("m2k2", "m0", "both"),
        default="both",
        help="Spindle solver mode(s) for Level A and Level C parity verification (default: both).",
    )
    parser.add_argument(
        "--skip-compile",
        action="store_true",
        help="Load pre-compiled libbike_cuda_writers.so and libbike_cpu_batch_runner.so if already present.",
    )
    args = parser.parse_args(argv)

    host_so_default = DEFAULT_GPU_SRC_DIR.parents[2] / "build" / "libbike_cpu_batch_runner.so"
    if args.skip_compile and DEFAULT_OUT_SO_PATH.is_file() and host_so_default.is_file():
        print("=== Loading pre-compiled CUDA & Host-float libraries ===", flush=True)
        cuda_lib = configure_cuda_writers_cdll(ctypes.CDLL(str(DEFAULT_OUT_SO_PATH)))
        host_lib = configure_cuda_writers_cdll(ctypes.CDLL(str(host_so_default)))
    else:
        print("=== Compiling CUDA (nvcc --fmad=false) & Host-float (g++ -ffp-contract=off) libraries ===", flush=True)
        cuda_lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)
        host_lib = compile_cpu_batch_runner(DEFAULT_GPU_SRC_DIR)
    prov = _collect_provenance()
    print(f"GPU: {prov['gpu_name']} | nvidia-smi: {prov['nvidia_smi']}", flush=True)

    bundle_dir = args.bundle_dir.resolve()
    s1_dir = bundle_dir / "S1_flat"
    g3_dir = bundle_dir / "G3_distributed_flat_30s"
    g5_dir = bundle_dir / "G5_pneumatic_flat_30s"
    if not (g5_dir / "model_12gears.xml").is_file():
        g5_dir = s1_dir
    s1_qpos, s1_qvel, s1_qws = _load_step_closure_row0(s1_dir)
    g3_qpos, g3_qvel, g3_qws = _load_step_closure_row0(g3_dir)
    g5_qpos, g5_qvel, g5_qws = _load_step_closure_row0(g5_dir)

    print("\n=== Level A: Exact CUDA vs Host-float Kernel Parity (nworld=32) ===", flush=True)
    level_a = []
    for bk, st_cnt, sub, qp, qv, qws in (
        ("compliant_2d", 0, s1_dir, s1_qpos, s1_qvel, s1_qws),
        ("distributed_2d_reference", 128, g3_dir, g3_qpos, g3_qvel, g3_qws),
        ("pneumatic", 64, g5_dir, g5_qpos, g5_qvel, g5_qws),
    ):
        r_sp1 = run_level_a_sp1_writers_parity(
            cuda_lib=cuda_lib,
            host_lib=host_lib,
            xml_path=sub / "model_12gears.xml",
            qpos0=qp,
            qvel0=qv,
            backend=bk,
            station_count=st_cnt,
            nworld=32,
            n_steps=5,
        )
        print(
            f"  [Level A SP1/SP2 {bk}] pass={r_sp1['passed']} max_qfrc_ulp={r_sp1['max_qfrc_ulp']} "
            f"max_ctrl_ulp={r_sp1['max_ctrl_ulp']} zero_bit_qfrc={r_sp1['last_step_qfrc']['zero_bit_fraction']:.6f}",
            flush=True,
        )
        level_a.append(r_sp1)

        if args.spindle_solver in ("m2k2", "both"):
            r_sp4 = run_level_a_sp3_sp4_parity(
                cuda_lib=cuda_lib,
                host_lib=host_lib,
                xml_path=sub / "model_12gears.xml",
                qpos0=qp,
                qvel0=qv,
                qacc_ws0=qws,
                backend=bk,
                station_count=st_cnt,
                nworld=32,
                n_steps=5,
            )
            print(
                f"  [Level A SP3/SP4 M2K2 {bk}] pass={r_sp4['passed']} overall_max_ulp={r_sp4['overall_max_ulp']} "
                f"flags_match={r_sp4['flags_match']} ulps={r_sp4['max_ulp_by_array']}",
                flush=True,
            )
            level_a.append(r_sp4)

        if args.spindle_solver in ("m0", "both") and hasattr(
            cuda_lib, "bike_gpu_launch_sp4_m0_init"
        ):
            r_m0 = run_level_a_sp4_m0_parity(
                cuda_lib=cuda_lib,
                host_lib=host_lib,
                xml_path=sub / "model_12gears.xml",
                qpos0=qp,
                qvel0=qv,
                qacc_ws0=qws,
                backend=bk,
                station_count=st_cnt,
                nworld=32,
                n_steps=5,
            )
            print(
                f"  [Level A SP4 M0 {bk}] pass={r_m0['passed']} overall_max_ulp={r_m0['overall_max_ulp']} "
                f"flags_match={r_m0['flags_match']} ulps={r_m0['max_ulp_by_array']}",
                flush=True,
            )
            level_a.append(r_m0)

    level_a_all_passed = all(bool(x["passed"]) for x in level_a)

    print("\n=== Level C: Closed-Loop 10-Scenario Acceptance (mujoco_warp + CUDA m2k2/m0) ===", flush=True)
    level_c = run_level_c_closed_loop_10scenarios(
        cuda_lib=cuda_lib,
        host_lib=host_lib,
        bundle_dir=bundle_dir,
        spindle_solver=args.spindle_solver,
    )

    print(f"\n=== Batched Throughput Sanity (nworld={args.nworld_throughput}, n_steps={args.steps_throughput}) ===", flush=True)
    throughput = run_batched_throughput_sanity(
        cuda_lib=cuda_lib,
        host_lib=host_lib,
        bundle_dir=bundle_dir,
        nworld=args.nworld_throughput,
        n_steps=args.steps_throughput,
    )
    throughput_ok = all(bool(r["all_finite"]) for r in throughput)

    report = {
        "provenance": prov,
        "level_a_all_passed": bool(level_a_all_passed),
        "level_a": level_a,
        "level_c_all_passed": bool(level_c["all_passed"]),
        "level_c": level_c,
        "throughput_all_finite": bool(throughput_ok),
        "throughput": throughput,
    }
    if args.out_json is not None:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"\nWrote JSON report to {args.out_json}", flush=True)

    overall_ok = bool(level_a_all_passed and level_c["all_passed"] and throughput_ok)
    print(f"\n=== OVERALL CUDA PARITY RESULT: {'PASS' if overall_ok else 'FAIL'} ===", flush=True)
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
