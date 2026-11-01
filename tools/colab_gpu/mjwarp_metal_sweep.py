"""Scenario 2: Full Apple Metal GPU Pipeline (`warp-metal` + `mujoco_warp` + Custom Metal Writers).

Executes the full GPU simulation loop on Apple Silicon (`device="metal:0"`):
  - Custom bicycle force writers (`bike_wp_writers_metal_kernel` / `bike_wp_writers_simd32_kernel`)
    compiled via `wp_metal_load_library` and dispatched via `wp_metal_launch_kernel`.
  - Rigid-body constraint dynamics, kinematics, and collision (`mjwarp.step(m_wp, d_b)`)
    compiled and dispatched on Apple Metal GPU via `warp-metal` (`1.17.0.3`).
  - Both stages fused inside a single `wp.ScopedCapture()` Metal Graph (`wp.capture_launch(graph)`).

Benchmarks:
  1. Experiment A (`run_mjwarp_metal_parameter_sweep`):
     Heterogeneous `nworld = 2048` parameter sweep across `fork_psi_scale`, `rebound_scale`,
     `shock_rate_scale`, and `tire_stiff_scale`.
  2. Experiment B (`run_mjwarp_metal_3d_tire_ray_scaling_benchmark`):
     3D tire ray-grid scaling across `64`, `900`, and `7700` rays/world at `nworld = 2048`
     and `nworld = 1`.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import platform
import sys
import time
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WARP_CACHE_DIR = REPO_ROOT / "build" / "warp_cache"

try:
    import warp_metal  # noqa: F401
    import warp as wp
    import mujoco
    import mujoco_warp as mjwarp
except ImportError:  # pragma: no cover
    wp = None  # type: ignore[assignment]
    mujoco = None  # type: ignore[assignment]
    mjwarp = None  # type: ignore[assignment]

try:
    from tools.colab_gpu.build_cuda_writers import (
        DEFAULT_GPU_SRC_DIR,
        GpuModelIndicesStruct,
        build_sweep_configs_bytes,
        resolve_gpu_model_indices,
    )
    from tools.colab_gpu.metal_hybrid_sweep import compile_metal_batch_runner
    from tools.colab_gpu.mjwarp_batch_sweep import (
        DEFAULT_MODEL_XML,
        TIRE_RAY_SCALING_TIERS,
        _allocate_and_copy_device_bytes,
        _extract_heightfield_1d,
        _prepare_initial_mjdata,
        _resolve_tendon_range_binding,
        _summarize_distribution,
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
    from mjwarp_batch_sweep import (  # type: ignore[no-redef]
        DEFAULT_MODEL_XML,
        TIRE_RAY_SCALING_TIERS,
        _allocate_and_copy_device_bytes,
        _extract_heightfield_1d,
        _prepare_initial_mjdata,
        _resolve_tendon_range_binding,
        _summarize_distribution,
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
        ("hf_heights", ctypes.c_uint64),
        ("configs", ctypes.c_uint64),
        ("states", ctypes.c_uint64),
        ("qpos", ctypes.c_uint64),
        ("qvel", ctypes.c_uint64),
        ("xpos", ctypes.c_uint64),
        ("qfrc_applied", ctypes.c_uint64),
        ("ctrl", ctypes.c_uint64),
        ("tendon_range", ctypes.c_uint64),
        ("nworld", ctypes.c_int32),
        ("dt_s", ctypes.c_float),
        ("hf_num_samples", ctypes.c_int32),
        ("hf_x_min", ctypes.c_float),
        ("hf_dx", ctypes.c_float),
        ("per_world_config", ctypes.c_int32),
        ("tendon_range_world_stride", ctypes.c_int32),
        ("_pad", ctypes.c_int32),
    ]


_INIT_PTR_OFFSETS = (ctypes.c_size_t * 4)(0, 8, 16, 24)
_WRITERS_PTR_OFFSETS_9 = (ctypes.c_size_t * 9)(0, 8, 16, 24, 32, 40, 48, 56, 64)
_WRITERS_PTR_OFFSETS_10 = (ctypes.c_size_t * 10)(0, 8, 16, 24, 32, 40, 48, 56, 64, 72)


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
        self.k_writers_1 = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_writers_metal_kernel")
        self.k_writers_32 = self.core.wp_metal_get_kernel(self.mtl_lib, b"bike_wp_writers_simd32_kernel")
        if not self.k_init or not self.k_writers_1 or not self.k_writers_32:
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
        d_hf_ptr: int,
        hf_num_samples: int,
        hf_x_min: float,
        hf_dx: float,
        d_configs_ptr: int,
        per_world_config: int,
        d_states_ptr: int,
        d_qpos_ptr: int,
        d_qvel_ptr: int,
        d_xpos_ptr: int,
        d_qfrc_ptr: int,
        d_ctrl_ptr: int,
        tendon_range_ptr: int,
        tendon_range_stride: int,
        use_simd32: bool = False,
    ) -> None:
        num_threads = int(nworld) * 32 if use_simd32 else int(nworld)
        block_dim = 128 if use_simd32 else 64
        kernel_handle = self.k_writers_32 if use_simd32 else self.k_writers_1
        bounds = _BikeWpBounds(shape0=num_threads, _pad0=0, size=num_threads, coord_mult=1)
        args = _BikeWpWritersArgs(
            idx=int(d_idx_ptr),
            hf_heights=int(d_hf_ptr),
            configs=int(d_configs_ptr),
            states=int(d_states_ptr),
            qpos=int(d_qpos_ptr),
            qvel=int(d_qvel_ptr),
            xpos=int(d_xpos_ptr),
            qfrc_applied=int(d_qfrc_ptr),
            ctrl=int(d_ctrl_ptr),
            tendon_range=int(tendon_range_ptr),
            nworld=int(nworld),
            dt_s=float(dt_s),
            hf_num_samples=int(hf_num_samples),
            hf_x_min=float(hf_x_min),
            hf_dx=float(hf_dx),
            per_world_config=int(per_world_config),
            tendon_range_world_stride=int(tendon_range_stride),
            _pad=0,
        )
        if tendon_range_ptr != 0:
            offsets = _WRITERS_PTR_OFFSETS_10
            n_offsets = 10
        else:
            offsets = _WRITERS_PTR_OFFSETS_9
            n_offsets = 9

        rc = self.core.wp_metal_launch_kernel(
            self.ordinal,
            kernel_handle,
            num_threads,
            block_dim,
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
            raise RuntimeError(f"wp_metal_launch_kernel(bike_wp_writers) failed: {err}")


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
) -> dict[str, Any]:
    """Experiment A: Heterogeneous parameter sweep on Apple Metal (`warp-metal` + `mujoco_warp`)."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

    m = mujoco.MjModel.from_xml_path(str(model_xml_path))
    d0 = _prepare_initial_mjdata(m)
    dt_s = float(m.opt.timestep)

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
        num_stations=32,
        num_lateral_ribs=1,
    )
    sizeof_state = int(lib.bike_gpu_sizeof_world_state())

    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_configs = _allocate_and_copy_device_bytes(configs_bytes, device=device)
    d_states = wp.zeros(nworld * sizeof_state, dtype=wp.uint8, device=device)
    d_hf_heights, hf_num_samples, hf_x_min, hf_dx = _extract_heightfield_1d(m, device=device)

    m_wp = mjwarp.put_model(m)
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=nconmax, njmax=njmax)
    mjwarp.forward(m_wp, d_b)
    tendon_range_ptr, tendon_range_stride = _resolve_tendon_range_binding(m_wp, nworld)

    wm_rt.launch_init_states(
        nworld=nworld,
        d_idx_ptr=int(d_idx.ptr),
        d_configs_ptr=int(d_configs.ptr),
        per_world_config=1,
        d_states_ptr=int(d_states.ptr),
        d_qpos_ptr=int(d_b.qpos.ptr),
    )
    wp.synchronize()

    # Warmup 1 uncaptured step to ensure all mjwarp kernels are compiled and loaded before capture
    wm_rt.launch_writers(
        nworld=nworld,
        dt_s=dt_s,
        d_idx_ptr=int(d_idx.ptr),
        d_hf_ptr=int(d_hf_heights.ptr),
        hf_num_samples=hf_num_samples,
        hf_x_min=hf_x_min,
        hf_dx=hf_dx,
        d_configs_ptr=int(d_configs.ptr),
        per_world_config=1,
        d_states_ptr=int(d_states.ptr),
        d_qpos_ptr=int(d_b.qpos.ptr),
        d_qvel_ptr=int(d_b.qvel.ptr),
        d_xpos_ptr=int(d_b.xpos.ptr),
        d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
        d_ctrl_ptr=int(d_b.ctrl.ptr),
        tendon_range_ptr=tendon_range_ptr,
        tendon_range_stride=tendon_range_stride,
        use_simd32=False,
    )
    mjwarp.step(m_wp, d_b)
    wp.synchronize()

    # Reset to clean initial state and capture Metal Graph
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=nconmax, njmax=njmax)
    mjwarp.forward(m_wp, d_b)
    wm_rt.launch_init_states(
        nworld=nworld,
        d_idx_ptr=int(d_idx.ptr),
        d_configs_ptr=int(d_configs.ptr),
        per_world_config=1,
        d_states_ptr=int(d_states.ptr),
        d_qpos_ptr=int(d_b.qpos.ptr),
    )
    wp.synchronize()

    with wp.ScopedCapture() as capture:
        wm_rt.launch_writers(
            nworld=nworld,
            dt_s=dt_s,
            d_idx_ptr=int(d_idx.ptr),
            d_hf_ptr=int(d_hf_heights.ptr),
            hf_num_samples=hf_num_samples,
            hf_x_min=hf_x_min,
            hf_dx=hf_dx,
            d_configs_ptr=int(d_configs.ptr),
            per_world_config=1,
            d_states_ptr=int(d_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
            d_qvel_ptr=int(d_b.qvel.ptr),
            d_xpos_ptr=int(d_b.xpos.ptr),
            d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
            d_ctrl_ptr=int(d_b.ctrl.ptr),
            tendon_range_ptr=tendon_range_ptr,
            tendon_range_stride=tendon_range_stride,
            use_simd32=False,
        )
        mjwarp.step(m_wp, d_b)
    graph = capture.graph

    for _ in range(warmup_steps):
        wp.capture_launch(graph)
    wp.synchronize()

    # Reset for timed rollout + metric sampling
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=nconmax, njmax=njmax)
    mjwarp.forward(m_wp, d_b)
    wm_rt.launch_init_states(
        nworld=nworld,
        d_idx_ptr=int(d_idx.ptr),
        d_configs_ptr=int(d_configs.ptr),
        per_world_config=1,
        d_states_ptr=int(d_states.ptr),
        d_qpos_ptr=int(d_b.qpos.ptr),
    )
    with wp.ScopedCapture() as capture:
        wm_rt.launch_writers(
            nworld=nworld,
            dt_s=dt_s,
            d_idx_ptr=int(d_idx.ptr),
            d_hf_ptr=int(d_hf_heights.ptr),
            hf_num_samples=hf_num_samples,
            hf_x_min=hf_x_min,
            hf_dx=hf_dx,
            d_configs_ptr=int(d_configs.ptr),
            per_world_config=1,
            d_states_ptr=int(d_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
            d_qvel_ptr=int(d_b.qvel.ptr),
            d_xpos_ptr=int(d_b.xpos.ptr),
            d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
            d_ctrl_ptr=int(d_b.ctrl.ptr),
            tendon_range_ptr=tendon_range_ptr,
            tendon_range_stride=tendon_range_stride,
            use_simd32=False,
        )
        mjwarp.step(m_wp, d_b)
    graph = capture.graph
    wp.synchronize()

    t0 = time.perf_counter()
    for _ in range(n_steps):
        wp.capture_launch(graph)
    wp.synchronize()
    wall_time_s = time.perf_counter() - t0

    # Sample physical metrics across rollout
    d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=nconmax, njmax=njmax)
    mjwarp.forward(m_wp, d_b)
    wm_rt.launch_init_states(
        nworld=nworld,
        d_idx_ptr=int(d_idx.ptr),
        d_configs_ptr=int(d_configs.ptr),
        per_world_config=1,
        d_states_ptr=int(d_states.ptr),
        d_qpos_ptr=int(d_b.qpos.ptr),
    )
    with wp.ScopedCapture() as capture:
        wm_rt.launch_writers(
            nworld=nworld,
            dt_s=dt_s,
            d_idx_ptr=int(d_idx.ptr),
            d_hf_ptr=int(d_hf_heights.ptr),
            hf_num_samples=hf_num_samples,
            hf_x_min=hf_x_min,
            hf_dx=hf_dx,
            d_configs_ptr=int(d_configs.ptr),
            per_world_config=1,
            d_states_ptr=int(d_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
            d_qvel_ptr=int(d_b.qvel.ptr),
            d_xpos_ptr=int(d_b.xpos.ptr),
            d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
            d_ctrl_ptr=int(d_b.ctrl.ptr),
            tendon_range_ptr=tendon_range_ptr,
            tendon_range_stride=tendon_range_stride,
            use_simd32=False,
        )
        mjwarp.step(m_wp, d_b)
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


def run_mjwarp_metal_3d_tire_ray_scaling_benchmark(
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
    """Experiment B: 3D tire ray-grid scaling on Apple Metal (`warp-metal` + `mujoco_warp`)."""
    gpu_name = _init_metal_device()
    if lib is None:
        lib = compile_metal_batch_runner(DEFAULT_GPU_SRC_DIR)
    if wm_rt is None:
        wm_rt = WarpMetalWritersRuntime(lib)

    m = mujoco.MjModel.from_xml_path(str(model_xml_path))
    d0 = _prepare_initial_mjdata(m)
    dt_s = float(m.opt.timestep)

    idx_bytes = resolve_gpu_model_indices(m)
    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_hf_heights, hf_num_samples, hf_x_min, hf_dx = _extract_heightfield_1d(m, device=device)
    sizeof_state = int(lib.bike_gpu_sizeof_world_state())

    m_wp = mjwarp.put_model(m)
    tendon_range_ptr, tendon_range_stride = _resolve_tendon_range_binding(m_wp, nworld)

    tier_results: list[dict[str, Any]] = []
    baseline_us: float | None = None

    for tier in TIRE_RAY_SCALING_TIERS:
        num_stations = int(tier["num_stations"])
        num_lateral_ribs = int(tier["num_lateral_ribs"])
        rays_per_world = int(tier["rays_per_world"])
        use_simd32 = bool(num_lateral_ribs > 1 or nworld < 64)

        configs_bytes = build_sweep_configs_bytes(
            lib=lib,
            nworld=1,
            fork_psi_scale=None,
            rebound_scale=None,
            shock_rate_scale=None,
            tire_stiff_scale=None,
            num_stations=num_stations,
            num_lateral_ribs=num_lateral_ribs,
        )
        d_configs = _allocate_and_copy_device_bytes(configs_bytes, device=device)
        d_states = wp.zeros(nworld * sizeof_state, dtype=wp.uint8, device=device)

        d_b = mjwarp.put_data(m, d0, nworld=nworld, nconmax=nconmax, njmax=njmax)
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

        # 1 uncaptured step to ensure all modules are loaded
        wm_rt.launch_writers(
            nworld=nworld,
            dt_s=dt_s,
            d_idx_ptr=int(d_idx.ptr),
            d_hf_ptr=int(d_hf_heights.ptr),
            hf_num_samples=hf_num_samples,
            hf_x_min=hf_x_min,
            hf_dx=hf_dx,
            d_configs_ptr=int(d_configs.ptr),
            per_world_config=0,
            d_states_ptr=int(d_states.ptr),
            d_qpos_ptr=int(d_b.qpos.ptr),
            d_qvel_ptr=int(d_b.qvel.ptr),
            d_xpos_ptr=int(d_b.xpos.ptr),
            d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
            d_ctrl_ptr=int(d_b.ctrl.ptr),
            tendon_range_ptr=tendon_range_ptr,
            tendon_range_stride=tendon_range_stride,
            use_simd32=use_simd32,
        )
        mjwarp.step(m_wp, d_b)
        wp.synchronize()

        with wp.ScopedCapture() as capture:
            wm_rt.launch_writers(
                nworld=nworld,
                dt_s=dt_s,
                d_idx_ptr=int(d_idx.ptr),
                d_hf_ptr=int(d_hf_heights.ptr),
                hf_num_samples=hf_num_samples,
                hf_x_min=hf_x_min,
                hf_dx=hf_dx,
                d_configs_ptr=int(d_configs.ptr),
                per_world_config=0,
                d_states_ptr=int(d_states.ptr),
                d_qpos_ptr=int(d_b.qpos.ptr),
                d_qvel_ptr=int(d_b.qvel.ptr),
                d_xpos_ptr=int(d_b.xpos.ptr),
                d_qfrc_ptr=int(d_b.qfrc_applied.ptr),
                d_ctrl_ptr=int(d_b.ctrl.ptr),
                tendon_range_ptr=tendon_range_ptr,
                tendon_range_stride=tendon_range_stride,
                use_simd32=use_simd32,
            )
            mjwarp.step(m_wp, d_b)
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
        mrays_per_s = (float(total_world_steps) * float(rays_per_world)) / max(wall_time_s, 1e-12) / 1e6
        rtf = (float(total_world_steps) * dt_s) / max(wall_time_s, 1e-12)

        if baseline_us is None:
            baseline_us = us_per_step_per_world
        delta_us_vs_2d = us_per_step_per_world - baseline_us
        pct_over_2d = ((us_per_step_per_world / baseline_us) - 1.0) * 100.0 if baseline_us > 0.0 else 0.0

        tier_results.append(
            {
                "tier_id": tier["tier_id"],
                "label": tier["label"],
                "num_stations": num_stations,
                "num_lateral_ribs": num_lateral_ribs,
                "rays_per_wheel": int(tier["rays_per_wheel"]),
                "rays_per_world": rays_per_world,
                "total_rays_per_step": int(nworld) * rays_per_world,
                "wall_time_ms": wall_time_s * 1000.0,
                "us_per_step_per_world": us_per_step_per_world,
                "delta_us_vs_2d_planar": delta_us_vs_2d,
                "pct_over_2d_planar": pct_over_2d,
                "steps_per_s": steps_per_s,
                "mrays_per_s": mrays_per_s,
                "realtime_factor_x": rtf,
                "all_finite": finite_all,
            }
        )

    return {
        "experiment": "B_mjwarp_metal_3d_tire_ray_scaling",
        "backend": "metal_mjwarp_full_gpu_fp32",
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

    for key in ("tire_ray_scaling_batch", "tire_ray_scaling_single"):
        if key in results:
            tr = results[key]
            print(
                f"\n=== Scenario 2 Experiment B: 3D Tire Ray-Grid Scaling "
                f"(nworld={tr['nworld']}, {tr['gpu_device']}) ==="
            )
            for tier in tr["tiers"]:
                print(
                    f"  {tier['label']:<35} | "
                    f"grid={tier['num_stations']}x{tier['num_lateral_ribs']} | "
                    f"wall={tier['wall_time_ms']:7.2f} ms | "
                    f"{tier['us_per_step_per_world']:6.3f} us/step/world "
                    f"({tier['pct_over_2d_planar']:+6.1f}%) | "
                    f"{tier['steps_per_s']:10,.0f} steps/s | "
                    f"{tier['mrays_per_s']:8.1f} Mrays/s | "
                    f"RTF={tier['realtime_factor_x']:6.1f}x"
                )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run Scenario 2: Full Apple Metal GPU (warp-metal + mujoco_warp) benchmarks."
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
        "--ray-steps",
        type=int,
        default=500,
        help="Number of steps per tier for Experiment B 3D tire ray scaling (default: 500).",
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
        results["tire_ray_scaling_batch"] = run_mjwarp_metal_3d_tire_ray_scaling_benchmark(
            model_xml_path=args.model_xml,
            nworld=args.nworld,
            n_steps=args.ray_steps,
            lib=lib,
            wm_rt=wm_rt,
        )
        if not args.skip_single_world:
            results["tire_ray_scaling_single"] = run_mjwarp_metal_3d_tire_ray_scaling_benchmark(
                model_xml_path=args.model_xml,
                nworld=1,
                n_steps=args.ray_steps,
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
