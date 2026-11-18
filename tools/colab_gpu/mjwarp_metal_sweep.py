"""Scenario 2: Full Apple Metal GPU Pipeline (`warp-metal` + `mujoco_warp` + Custom Metal Writers).

Executes the full GPU simulation loop on Apple Silicon (`device="metal:0"`) via SP1 `clean_split`:
  - Custom bicycle force writers (`bike_wp_writers_metal_kernel`) compiled via
    `wp_metal_load_library` and dispatched via `wp_metal_launch_kernel` with `MTLMathModeSafe`
    and `#pragma METAL fp contract(off)`.
  - Rigid-body constraint dynamics, kinematics, and collision (`clean_split` schedule:
    `fwd_kinematics -> crb -> tendon_armature -> factor_m -> collision -> transmission ->
    fwd_velocity -> writers -> make_constraint -> fwd_actuation -> fwd_acceleration ->
    solve -> sensor_acc -> implicit`) compiled and dispatched on Apple Metal GPU via `warp-metal`.
  - All stages fused inside a single `wp.ScopedCapture()` Metal Graph (`wp.capture_launch(graph)`).

Benchmarks:
  1. Experiment A (`run_mjwarp_metal_parameter_sweep`):
     Heterogeneous `nworld = 2048` parameter sweep across `fork_psi_scale`, `rebound_scale`,
     `shock_rate_scale`, and `tire_stiff_scale`.
  2. Experiment B (`run_mjwarp_metal_tyre_backend_scaling_benchmark`):
     Shared `tyre_gpu_step.hpp` backend scaling across `compliant_2d` and
     `distributed_2d_reference` (`N=32/64/128`) at `nworld = 2048` and `nworld = 1`.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
from pathlib import Path
import platform
import sys
import time
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WARP_CACHE_DIR = REPO_ROOT / "build" / "warp_cache"

try:
    import mujoco
except Exception:  # pragma: no cover
    mujoco = None  # type: ignore[assignment]

try:
    import warp_metal  # noqa: F401
    import warp as wp
    import mujoco_warp as mjwarp
except Exception:  # pragma: no cover
    wp = None  # type: ignore[assignment]
    mjwarp = None  # type: ignore[assignment]

try:
    from tools.colab_gpu.build_cuda_writers import (
        DEFAULT_GPU_SRC_DIR,
        GpuModelIndicesStruct,
        build_sweep_configs_bytes,
        resolve_gpu_model_indices,
    )
    from tools.colab_gpu.metal_hybrid_sweep import compile_metal_batch_runner
    from tools.colab_gpu.model_assets import load_model_with_assets
    from tools.colab_gpu.mjwarp_batch_sweep import (
        DEFAULT_MODEL_XML,
        TYRE_BACKEND_TIERS,
        _allocate_and_copy_device_bytes,
        _ensure_newton_solver,
        _mjwarp_clean_split_step,
        _prepare_initial_mjdata,
        _resolve_clean_split_ops,
        _resolve_tendon_range_binding,
        _summarize_distribution,
        _upload_tyre_and_terrain_device,
        generate_parameter_sweep_scales,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_cuda_writers import (  # type: ignore[no-redef]
        DEFAULT_GPU_SRC_DIR,
        GpuModelIndicesStruct,
        build_sweep_configs_bytes,
        resolve_gpu_model_indices,
    )
    from metal_hybrid_sweep import compile_metal_batch_runner  # type: ignore[no-redef]
    from model_assets import load_model_with_assets  # type: ignore[no-redef]
    from mjwarp_batch_sweep import (  # type: ignore[no-redef]
        DEFAULT_MODEL_XML,
        TYRE_BACKEND_TIERS,
        _allocate_and_copy_device_bytes,
        _ensure_newton_solver,
        _mjwarp_clean_split_step,
        _prepare_initial_mjdata,
        _resolve_clean_split_ops,
        _resolve_tendon_range_binding,
        _summarize_distribution,
        _upload_tyre_and_terrain_device,
        generate_parameter_sweep_scales,
    )


class _BikeWpBounds(ctypes.Structure):
    _fields_ = [
        ("shape0", ctypes.c_int32),
        ("_pad0", ctypes.c_int32),
        ("size", ctypes.c_size_t),
        ("coord_mult", ctypes.c_size_t),
    ]


class _BikeWpInitArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("per_world_config", ctypes.c_int32),
    ]


class _BikeWpWritersArgs(ctypes.Structure):
    _fields_ = [
        ("idx", ctypes.c_uint64),
        ("tyre_model", ctypes.c_uint64),
        ("terrain_desc", ctypes.c_uint64),
        ("terrain_f32", ctypes.c_uint64),
        ("terrain_i32", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("qvel", ctypes.c_uint64),
        ("xpos", ctypes.c_uint64),
        ("xmat", ctypes.c_uint64),
        ("xanchor", ctypes.c_uint64),
        ("xaxis", ctypes.c_uint64),
        ("qfrc_applied", ctypes.c_uint64),
        ("ctrl", ctypes.c_uint64),
        ("tendon_range", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("dt_s", ctypes.c_float),
        ("per_world_config", ctypes.c_int32),
        ("tendon_range_world_stride", ctypes.c_int32),
    ]


_INIT_PTR_OFFSETS = (ctypes.c_size_t * 4)(0, 8, 16, 24)
_WRITERS_PTR_OFFSETS_15 = (ctypes.c_size_t * 15)(*range(0, 120, 8))
_WRITERS_PTR_OFFSETS_16 = (ctypes.c_size_t * 16)(*range(0, 128, 8))


class WarpMetalWritersRuntime:
    """Loads custom bicycle Metal force writer kernels into `warp-metal`'s native runtime."""

    def __init__(self, lib: ctypes.CDLL, ordinal: int = 0) -> None:
        self.lib = lib
        self.ordinal = int(ordinal)
        self.core = wp._src.context.runtime.core

        msl_bytes = self.lib.bike_metal_get_shader_source(
            str(DEFAULT_GPU_SRC_DIR).encode("utf-8")
        )
        if not msl_bytes:
            raise RuntimeError("bike_metal_get_shader_source returned empty source")

        self.mtl_lib = self.core.wp_metal_load_library(self.ordinal, msl_bytes)
        if not self.mtl_lib:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_load_library failed:\n{err}")

        self.k_init = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_init_states_kernel")
        self.k_writers = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_writers_metal_kernel")
        if not self.k_init or not self.k_writers:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_get_kernel failed:\n{err}")

    def launch_init_states(
        self,
        nworld: int,
        d_idx_ptr: int,
        d_configs_ptr: int,
        per_world_config: int,
        d_states_ptr: int,
        d_qpos_ptr: int,
    ) -> None:
        if int(nworld) <= 0:
            raise ValueError(f"nworld must be > 0, got {nworld}")
        for name, ptr in (
            ("d_idx_ptr", d_idx_ptr),
            ("d_configs_ptr", d_configs_ptr),
            ("d_states_ptr", d_states_ptr),
            ("d_qpos_ptr", d_qpos_ptr),
        ):
            if int(ptr) == 0:
                raise ValueError(f"{name} must be non-zero")
        bounds = _BikeWpBounds(shape0=int(nworld), _pad0=0, size=int(nworld), coord_mult=1)
        args = _BikeWpInitArgs(
            idx=int(d_idx_ptr),
            configs=int(d_configs_ptr),
            states=int(d_states_ptr),
            qpos=int(d_qpos_ptr),
            nworld=int(nworld),
            per_world_config=int(per_world_config),
        )
        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_init,
            int(nworld),
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            _INIT_PTR_OFFSETS,
            4,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_launch_kernel(bike_wp_init_states_kernel) failed: {err}")

    def launch_writers(
        self,
        nworld: int,
        dt_s: float,
        d_idx_ptr: int,
        d_tyre_model_ptr: int,
        d_terrain_desc_ptr: int,
        d_terrain_f32_ptr: int,
        d_terrain_i32_ptr: int,
        d_configs_ptr: int,
        per_world_config: int,
        d_states_ptr: int,
        d_qpos_ptr: int,
        d_qvel_ptr: int,
        d_xpos_ptr: int,
        d_xmat_ptr: int,
        d_xanchor_ptr: int,
        d_xaxis_ptr: int,
        d_qfrc_ptr: int,
        d_ctrl_ptr: int,
        tendon_range_ptr: int,
        tendon_range_stride: int,
    ) -> None:
        if int(nworld) <= 0:
            raise ValueError(f"nworld must be > 0, got {nworld}")
        if not math.isfinite(float(dt_s)) or float(dt_s) <= 0.0:
            raise ValueError(f"dt_s must be finite and > 0, got {dt_s}")
        if int(tendon_range_stride) < 0:
            raise ValueError(f"tendon_range_stride must be >= 0, got {tendon_range_stride}")
        for name, ptr in (
            ("d_idx_ptr", d_idx_ptr),
            ("d_tyre_model_ptr", d_tyre_model_ptr),
            ("d_terrain_desc_ptr", d_terrain_desc_ptr),
            ("d_terrain_f32_ptr", d_terrain_f32_ptr),
            ("d_terrain_i32_ptr", d_terrain_i32_ptr),
            ("d_configs_ptr", d_configs_ptr),
            ("d_states_ptr", d_states_ptr),
            ("d_qpos_ptr", d_qpos_ptr),
            ("d_qvel_ptr", d_qvel_ptr),
            ("d_xpos_ptr", d_xpos_ptr),
            ("d_xmat_ptr", d_xmat_ptr),
            ("d_xanchor_ptr", d_xanchor_ptr),
            ("d_xaxis_ptr", d_xaxis_ptr),
            ("d_qfrc_ptr", d_qfrc_ptr),
            ("d_ctrl_ptr", d_ctrl_ptr),
        ):
            if int(ptr) == 0:
                raise ValueError(f"{name} must be non-zero")
        num_threads = int(nworld)
        bounds = _BikeWpBounds(shape0=num_threads, _pad0=0, size=num_threads, coord_mult=1)
        args = _BikeWpWritersArgs(
            idx=int(d_idx_ptr),
            tyre_model=int(d_tyre_model_ptr),
            terrain_desc=int(d_terrain_desc_ptr),
            terrain_f32=int(d_terrain_f32_ptr),
            terrain_i32=int(d_terrain_i32_ptr),
            configs=int(d_configs_ptr),
            states=int(d_states_ptr),
            qpos=int(d_qpos_ptr),
            qvel=int(d_qvel_ptr),
            xpos=int(d_xpos_ptr),
            xmat=int(d_xmat_ptr),
            xanchor=int(d_xanchor_ptr),
            xaxis=int(d_xaxis_ptr),
            qfrc_applied=int(d_qfrc_ptr),
            ctrl=int(d_ctrl_ptr),
            tendon_range=int(tendon_range_ptr),
            nworld=int(nworld),
            dt_s=float(dt_s),
            per_world_config=int(per_world_config),
            tendon_range_world_stride=int(tendon_range_stride),
        )
        if tendon_range_ptr != 0:
            offsets = _WRITERS_PTR_OFFSETS_16
            n_offsets = 16
        else:
            offsets = _WRITERS_PTR_OFFSETS_15
            n_offsets = 15

        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            self.k_writers,
            num_threads,
            64,
            ctypes.byref(bounds),
            ctypes.sizeof(bounds),
            ctypes.byref(args),
            ctypes.sizeof(args),
            offsets,
            n_offsets,
            0,
        )
        if rc != 0:
            err = wp._src.context.runtime.get_error_string()
            raise RuntimeError(f"wp_metal_launch_kernel(bike_wp_writers_metal_kernel) failed: {err}")


def _init_metal_device() -> str:
    if wp is None or mujoco is None or mjwarp is None:
        raise RuntimeError("warp-lang, warp-metal, mujoco, and mujoco_warp are required.")
    DEFAULT_WARP_CACHE_DIR.mkdir(parents=True, exist_ok=True)
    wp.config.kernel_cache_dir = str(DEFAULT_WARP_CACHE_DIR.resolve())
    wp.init()
    wp.set_device("metal:0")
    return str(wp.get_device("metal:0").name)


def run_mjwarp_metal_parameter_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    warmup_steps: int = 25,
    seed: int = 42,
    peak_sample_stride: int = 25,
    lib: ctypes.CDLL | None = None,
    wm_rt: WarpMetalWritersRuntime | None = None,
    device: str = "metal:0",
    nconmax: int = 64,
    njmax: int = 128,
    backend: str = "compliant_2d",
    station_count: int = 64,
) -> dict[str, Any]:
    """Experiment A: Heterogeneous parameter sweep on Apple Metal (`clean_split`)."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

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
        _d_terrain_f32,
        _d_terrain_i32,
        terrain_f32_ptr,
        terrain_i32_ptr,
    ) = _upload_tyre_and_terrain_device(
        lib, model_xml_path, backend, station_count, idx_bytes, device=device
    )

    m_wp = mjwarp.put_model(m)
    if m.ntendon > 0 and hasattr(m_wp, "tendon_range"):
        tr_np = np.tile(m.tendon_range.astype(np.float32)[None, :, :], (nworld, 1, 1))
        m_wp.tendon_range = wp.array(tr_np, dtype=wp.vec2f, device=device)

    eff_nconmax = max(int(nconmax), int(d0.ncon) + 32)
    eff_njmax = max(int(njmax), int(d0.nefc) + 64)

    def _init_data() -> Any:
        data_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=eff_nconmax, njmax=eff_njmax)
        mjwarp.forward(m_wp, data_b)
        tr_ptr, tr_stride = _resolve_tendon_range_binding(m_wp, nworld)
        wm_rt.launch_init_states(
            nworld=nworld,
            d_idx_ptr=int(d_idx.ptr),
            d_configs_ptr=int(d_configs.ptr),
            per_world_config=1,
            d_states_ptr=int(d_states.ptr),
            d_qpos_ptr=int(data_b.qpos.ptr),
        )
        wp.synchronize()
        return data_b, tr_ptr, tr_stride

    def _make_writer_fn(data_b: Any, tr_ptr: int, tr_stride: int) -> Any:
        def _launch_writers() -> None:
            wm_rt.launch_writers(
                nworld=nworld,
                dt_s=dt_s,
                d_idx_ptr=int(d_idx.ptr),
                d_tyre_model_ptr=int(d_tyre_model.ptr),
                d_terrain_desc_ptr=int(d_terrain_desc.ptr),
                d_terrain_f32_ptr=terrain_f32_ptr,
                d_terrain_i32_ptr=terrain_i32_ptr,
                d_configs_ptr=int(d_configs.ptr),
                per_world_config=1,
                d_states_ptr=int(d_states.ptr),
                d_qpos_ptr=int(data_b.qpos.ptr),
                d_qvel_ptr=int(data_b.qvel.ptr),
                d_xpos_ptr=int(data_b.xpos.ptr),
                d_xmat_ptr=int(data_b.xmat.ptr),
                d_xanchor_ptr=int(data_b.xanchor.ptr),
                d_xaxis_ptr=int(data_b.xaxis.ptr),
                d_qfrc_ptr=int(data_b.qfrc_applied.ptr),
                d_ctrl_ptr=int(data_b.ctrl.ptr),
                tendon_range_ptr=tr_ptr,
                tendon_range_stride=tr_stride,
            )

        return _launch_writers

    # Warmup 1 uncaptured step to ensure all mjwarp kernels are compiled before capture
    d_b, tendon_range_ptr, tendon_range_stride = _init_data()
    _mjwarp_clean_split_step(ops, m_wp, d_b, _make_writer_fn(d_b, tendon_range_ptr, tendon_range_stride))
    wp.synchronize()

    d_b, tendon_range_ptr, tendon_range_stride = _init_data()
    writer_fn = _make_writer_fn(d_b, tendon_range_ptr, tendon_range_stride)
    with wp.ScopedCapture() as capture:
        _mjwarp_clean_split_step(ops, m_wp, d_b, writer_fn)
    graph = capture.graph

    for _ in range(warmup_steps):
        wp.capture_launch(graph)
    wp.synchronize()

    d_b, tendon_range_ptr, tendon_range_stride = _init_data()
    writer_fn = _make_writer_fn(d_b, tendon_range_ptr, tendon_range_stride)
    with wp.ScopedCapture() as capture:
        _mjwarp_clean_split_step(ops, m_wp, d_b, writer_fn)
    graph = capture.graph
    wp.synchronize()

    t0 = time.perf_counter()
    for _ in range(n_steps):
        wp.capture_launch(graph)
    wp.synchronize()
    wall_time_s = time.perf_counter() - t0

    d_b, tendon_range_ptr, tendon_range_stride = _init_data()
    writer_fn = _make_writer_fn(d_b, tendon_range_ptr, tendon_range_stride)
    with wp.ScopedCapture() as capture:
        _mjwarp_clean_split_step(ops, m_wp, d_b, writer_fn)
    graph = capture.graph

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
    rtf = (float(total_world_steps) * dt_s) / max(wall_time_s, 1e-12)

    return {
        "experiment": "A_mjwarp_metal_parameter_sweep",
        "backend": "metal_mjwarp_full_gpu_fp32",
        "schedule": "clean_split",
        "tyre_backend": backend,
        "gpu_device": gpu_name,
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "wall_time_ms": wall_time_s * 1000.0,
        "us_per_step_per_world": us_per_step_per_world,
        "steps_per_s": steps_per_s,
        "realtime_factor_x": rtf,
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


def run_mjwarp_metal_tyre_backend_scaling_benchmark(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 500,
    warmup_steps: int = 20,
    lib: ctypes.CDLL | None = None,
    wm_rt: WarpMetalWritersRuntime | None = None,
    device: str = "metal:0",
    nconmax: int = 64,
    njmax: int = 128,
) -> dict[str, Any]:
    """Experiment B: Shared tyre backend scaling on Apple Metal (`clean_split`)."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

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
    tendon_range_ptr, tendon_range_stride = _resolve_tendon_range_binding(m_wp, nworld)

    configs_bytes = build_sweep_configs_bytes(lib=lib, nworld=1)
    d_configs = _allocate_and_copy_device_bytes(configs_bytes, device=device)
    d_states = wp.zeros(nworld * sizeof_state, dtype=wp.uint8, device=device)

    tier_results: list[dict[str, Any]] = []
    baseline_us: float | None = None

    eff_nconmax = max(int(nconmax), int(d0.ncon) + 32)
    eff_njmax = max(int(njmax), int(d0.nefc) + 64)

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
            lib, model_xml_path, backend, station_count, idx_bytes, device=device
        )

        d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=eff_nconmax, njmax=eff_njmax)
        mjwarp.forward(m_wp, d_b)

        wm_rt.launch_init_states(
            nworld=nworld,
            d_idx_ptr=int(d_idx.ptr),
            d_configs_ptr=int(d_configs.ptr),
            per_world_config=0,
            d_states_ptr=int(d_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
        )
        wp.synchronize()

        def _launch_writers() -> None:
            wm_rt.launch_writers(
                nworld=nworld,
                dt_s=dt_s,
                d_idx_ptr=int(d_idx.ptr),
                d_tyre_model_ptr=int(d_tyre_model.ptr),
                d_terrain_desc_ptr=int(d_terrain_desc.ptr),
                d_terrain_f32_ptr=terrain_f32_ptr,
                d_terrain_i32_ptr=terrain_i32_ptr,
                d_configs_ptr=int(d_configs.ptr),
                per_world_config=0,
                d_states_ptr=int(d_states.ptr),
                d_qpos_ptr=int(d_b.qpos.ptr),
                d_qvel_ptr=int(d_b.qvel.ptr),
                d_xpos_ptr=int(d_b.xpos.ptr),
                d_xmat_ptr=int(d_b.xmat.ptr),
                d_xanchor_ptr=int(d_b.xanchor.ptr),
                d_xaxis_ptr=int(d_b.xaxis.ptr),
                d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
                d_ctrl_ptr=int(d_b.ctrl.ptr),
                tendon_range_ptr=tendon_range_ptr,
                tendon_range_stride=tendon_range_stride,
            )

        _mjwarp_clean_split_step(ops, m_wp, d_b, _launch_writers)
        wp.synchronize()

        with wp.ScopedCapture() as capture:
            _mjwarp_clean_split_step(ops, m_wp, d_b, _launch_writers)
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
        rtf = (float(total_world_steps) * dt_s) / max(wall_time_s, 1e-12)

        if baseline_us is None:
            baseline_us = us_per_step_per_world
        delta_us = us_per_step_per_world - baseline_us
        pct_over = ((us_per_step_per_world / baseline_us) - 1.0) * 100.0 if baseline_us > 0.0 else 0.0

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
                "pct_over_compliant": pct_over,
                "steps_per_s": steps_per_s,
                "realtime_factor_x": rtf,
                "all_finite": finite_all,
            }
        )

    return {
        "experiment": "B_mjwarp_metal_tyre_backend_scaling",
        "backend": "metal_mjwarp_full_gpu_fp32",
        "schedule": "clean_split",
        "gpu_device": gpu_name,
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "tiers": tier_results,
    }


def _print_report(results: dict[str, Any]) -> None:
    if "parameter_sweep" in results:
        sw = results["parameter_sweep"]
        print(
            f"\n=== Scenario 2 Experiment A: Full Metal GPU Parameter Sweep ({sw['gpu_device']}) ==="
        )
        print(
            f"nworld = {sw['nworld']} | n_steps = {sw['n_steps']} | "
            f"wall = {sw['wall_time_ms']:.2f} ms | "
            f"{sw['us_per_step_per_world']:.3f} us/step/world | "
            f"{sw['steps_per_s']:,.0f} steps/s | "
            f"RTF = {sw['realtime_factor_x']:.1f}x | finite = {sw['all_finite']}"
        )
        for metric_name, stats in sw["metrics"].items():
            print(
                f"  {metric_name}: "
                f"min={stats['min']:.3f}, p10={stats['p10']:.3f}, "
                f"median={stats['median']:.3f}, mean={stats['mean']:.3f}, "
                f"p90={stats['p90']:.3f}, max={stats['max']:.3f}"
            )

    for key in ("tyre_backend_scaling_batch", "tyre_backend_scaling_single"):
        if key in results:
            tr = results[key]
            print(
                f"\n=== Scenario 2 Experiment B: Tyre Backend Scaling "
                f"(nworld={tr['nworld']}, {tr['gpu_device']}) ==="
            )
            for tier in tr["tiers"]:
                print(
                    f"  {tier['label']:<35} | "
                    f"stations={tier['station_count']:3d} | "
                    f"wall={tier['wall_time_ms']:7.2f} ms | "
                    f"{tier['us_per_step_per_world']:6.3f} us/step/world "
                    f"({tier['pct_over_compliant']:+6.1f}%) | "
                    f"{tier['steps_per_s']:10,.0f} steps/s | "
                    f"RTF={tier['realtime_factor_x']:6.1f}x"
                )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run Scenario 2: Full Apple Metal GPU (warp-metal + clean_split mujoco_warp) benchmarks."
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
        "--skip-single-world",
        action="store_true",
        help="Skip the nworld=1 single-world scaling run.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path to write JSON benchmark report.",
    )
    args = parser.parse_args(argv)

    _init_metal_device()
    lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    wm_rt = WarpMetalWritersRuntime(lib)

    results: dict[str, Any] = {
        "host_info": {
            "platform": platform.platform(),
            "processor": platform.processor(),
            "machine": platform.machine(),
            "gpu_device": str(wp.get_device("metal:0").name),
            "warp_version": wp.__version__,
            "mujoco_warp_version": getattr(mjwarp, "__version__", "3.11.0"),
        }
    }

    if args.experiment in ("all", "sweep"):
        results["parameter_sweep"] = run_mjwarp_metal_parameter_sweep(
            model_xml_path=args.model_xml,
            nworld=args.nworld,
            n_steps=args.n_steps,
            lib=lib,
            wm_rt=wm_rt,
        )

    if args.experiment in ("all", "tire_scaling"):
        results["tyre_backend_scaling_batch"] = run_mjwarp_metal_tyre_backend_scaling_benchmark(
            model_xml_path=args.model_xml,
            nworld=args.nworld,
            n_steps=args.tier_steps,
            lib=lib,
            wm_rt=wm_rt,
        )
        if not args.skip_single_world:
            results["tyre_backend_scaling_single"] = run_mjwarp_metal_tyre_backend_scaling_benchmark(
                model_xml_path=args.model_xml,
                nworld=1,
                n_steps=args.tier_steps,
                lib=lib,
                wm_rt=wm_rt,
            )

    _print_report(results)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
