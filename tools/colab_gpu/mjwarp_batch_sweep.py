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
from pathlib import Path
import sys
import time
from typing import Any, Callable

import numpy as np

try:
    from tools.colab_gpu.build_cuda_writers import (
        DEFAULT_GPU_SRC_DIR,
        DEFAULT_OUT_SO_PATH,
        GpuModelIndicesStruct,
        build_sweep_configs_bytes,
        build_tyre_and_terrain_buffers,
        compile_cuda_writers,
        resolve_gpu_model_indices,
    )
    from tools.colab_gpu.model_assets import (
        load_captured_initial_state,
        load_model_with_assets,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_cuda_writers import (  # type: ignore[no-redef]
        DEFAULT_GPU_SRC_DIR,
        DEFAULT_OUT_SO_PATH,
        GpuModelIndicesStruct,
        build_sweep_configs_bytes,
        build_tyre_and_terrain_buffers,
        compile_cuda_writers,
        resolve_gpu_model_indices,
    )
    from model_assets import (  # type: ignore[no-redef]
        load_captured_initial_state,
        load_model_with_assets,
    )

try:
    import mujoco
except Exception:  # pragma: no cover
    mujoco = None  # type: ignore[assignment]

try:
    import mujoco_warp as mjwarp
    import warp as wp
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


def _upload_tyre_and_terrain_device(
    host_lib: ctypes.CDLL,
    xml_path: Path,
    backend: str,
    station_count: int,
    idx_bytes: bytes,
    device: str,
) -> tuple[Any, Any, Any, Any, int, int]:
    """Build host tyre/terrain tables and upload ``(tyre_model, desc, f32, i32)`` to ``device``."""
    model_b, desc_b, f32_np, i32_np = build_tyre_and_terrain_buffers(
        host_lib=host_lib,
        xml_path=xml_path,
        backend=backend,
        station_count=station_count,
        idx_bytes=idx_bytes,
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
        choices=("all", "sweep", "tire_scaling"),
        default="all",
        help="Which benchmark experiment to execute.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path to write JSON benchmark report.",
    )
    args = parser.parse_args(argv)

    lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)
    results: dict[str, Any] = {}

    if args.experiment in ("all", "sweep"):
        results["parameter_sweep"] = run_parameter_sweep(
            model_xml_path=args.model_xml,
            nworld=args.nworld,
            n_steps=args.n_steps,
            lib=lib,
        )

    if args.experiment in ("all", "tire_scaling"):
        results["tyre_backend_scaling"] = run_tyre_backend_scaling_benchmark(
            model_xml_path=args.model_xml,
            nworld=args.nworld,
            n_steps=args.tier_steps,
            lib=lib,
        )

    _print_report(results)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
