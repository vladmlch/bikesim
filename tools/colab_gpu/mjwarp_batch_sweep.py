"""Batched C++/CUDA Force Writers + mujoco_warp Parameter Sweep & 3D Tire Ray Scaling Benchmark.

Executes two core GPU benchmarks on NVIDIA CUDA (e.g. Google Colab L4):
  1. Experiment A (`run_parameter_sweep`):
     Runs `nworld = 2048` heterogeneous worlds in parallel (`fork_psi_scale` in [0.8, 1.25],
     `rebound_scale` in [0.7, 1.4], `shock_rate_scale` in [0.85, 1.20], `tire_stiff_scale` in
     [0.85, 1.20]) inside a single `wp.ScopedCapture()` CUDA Graph fusing
     `lib.bike_gpu_launch_writers(...)` + `mjwarp.step(m_wp, d_b)`.
  2. Experiment B (`run_3d_tire_ray_scaling_benchmark`):
     Benchmarks the fused C++/CUDA writers + `mjwarp.step` at `nworld = 2048` across three
     tire ray-grid resolutions:
       - 2D Planar Tier (64 rays/world):     `num_stations = 32,  num_lateral_ribs = 1`
       - 3D Fast Tier (900 rays/world):      `num_stations = 45,  num_lateral_ribs = 10`
       - 3D Detailed Tier (7700 rays/world): `num_stations = 110, num_lateral_ribs = 35`
"""

from __future__ import annotations

import argparse
import ctypes
import json
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

try:
    from tools.colab_gpu.build_cuda_writers import (
        DEFAULT_GPU_SRC_DIR,
        DEFAULT_OUT_SO_PATH,
        GpuModelIndicesStruct,
        build_sweep_configs_bytes,
        compile_cuda_writers,
        resolve_gpu_model_indices,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_cuda_writers import (  # type: ignore[no-redef]
        DEFAULT_GPU_SRC_DIR,
        DEFAULT_OUT_SO_PATH,
        GpuModelIndicesStruct,
        build_sweep_configs_bytes,
        compile_cuda_writers,
        resolve_gpu_model_indices,
    )

try:
    import mujoco
    import mujoco_warp as mjwarp
    import warp as wp
except ImportError:  # pragma: no cover - imported on CUDA/Colab runtime
    mujoco = None  # type: ignore[assignment]
    mjwarp = None  # type: ignore[assignment]
    wp = None  # type: ignore[assignment]

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL_XML = REPO_ROOT / "tools" / "fp32_probe" / "artifacts" / "common" / "model.xml"

TIRE_RAY_SCALING_TIERS: tuple[dict[str, Any], ...] = (
    {
        "tier_id": "2d_planar_64",
        "label": "2D Planar Tier (64 rays/world)",
        "num_stations": 32,
        "num_lateral_ribs": 1,
        "rays_per_wheel": 32,
        "rays_per_world": 64,
    },
    {
        "tier_id": "3d_fast_900",
        "label": "3D Fast Tier (900 rays/world)",
        "num_stations": 45,
        "num_lateral_ribs": 10,
        "rays_per_wheel": 450,
        "rays_per_world": 900,
    },
    {
        "tier_id": "3d_detailed_7700",
        "label": "3D Detailed Tier (7700 rays/world)",
        "num_stations": 110,
        "num_lateral_ribs": 35,
        "rays_per_wheel": 3850,
        "rays_per_world": 7700,
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


def _extract_heightfield_1d(
    m: Any,
    device: str = "cuda:0",
) -> tuple[Any, int, float, float]:
    """Extract the 1D terrain elevation slice from `MjModel` and upload it to GPU memory."""
    if getattr(m, "nhfield", 0) > 0:
        ncol = int(m.hfield_ncol[0])
        size_x_half = float(m.hfield_size[0, 0])
        scale_z = float(m.hfield_size[0, 2])
        hf_x_min = 0.0
        z_offset = 0.0
        for g_id in range(int(getattr(m, "ngeom", 0))):
            if int(m.geom_type[g_id]) == int(mujoco.mjtGeom.mjGEOM_HFIELD):
                hf_x_min = float(m.geom_pos[g_id, 0]) - size_x_half
                z_offset = float(m.geom_pos[g_id, 2])
                break
        raw_row = (
            np.asarray(m.hfield_data[:ncol], dtype=np.float32) * np.float32(scale_z)
            + np.float32(z_offset)
        )
        total_length_x = 2.0 * size_x_half if size_x_half > 0.0 else float(ncol - 1) * 0.005
        hf_dx = total_length_x / float(max(ncol - 1, 1))
    else:
        ncol = 4000
        hf_dx = 0.005
        hf_x_min = 0.0
        x_coords = np.arange(ncol, dtype=np.float32) * np.float32(hf_dx)
        raw_row = (
            0.015 * np.sin(2.5 * x_coords)
            + 0.008 * np.sin(9.0 * x_coords)
            + 0.004 * np.cos(21.0 * x_coords)
        ).astype(np.float32)

    d_hf_heights = wp.array(np.ascontiguousarray(raw_row, dtype=np.float32), dtype=wp.float32, device=device)
    return d_hf_heights, int(ncol), float(hf_x_min), float(hf_dx)


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


_SETTLED_QPOS0: tuple[float, ...] = (
    2.0163204663006615,
    -0.05357401639909831,
    0.004284780880975736,
    0.060060019575332066,
    -0.00010761676353484251,
    -0.00010761678414200608,
    0.05987889648695992,
    0.021245897379749926,
    0.12725704340599134,
    -0.06889657594618108,
    0.025439328588473524,
    0.22395495463507642,
    -0.18923705383804443,
    0.01470640089285262,
    2.0283590907841074,
    -0.05351113740715276,
    0.1493965439850205,
    -0.14185170640949385,
    0.08528710301997564,
    -0.14365190627622126,
    0.08528710301997607,
    -0.14365190627622199,
    -0.11940117644925781,
    -0.020246245278860982,
    -0.15553302755866683,
    0.03599510121499223,
)

_SETTLED_QVEL0: tuple[float, ...] = (
    0.05801761406260024,
    -0.02434270696902437,
    0.09131975316303595,
    1.1072967848667608,
    -0.16297306446955506,
    -0.16297306446951973,
    0.0813597266915952,
    0.08118260076454009,
    -0.051013560249722076,
    0.02620219528909801,
    0.45931837864051583,
    -0.08781622504775156,
    0.07560567276947935,
    -0.006060887475190563,
    0.019576748171044888,
    0.0063482260239148105,
    -1.7666562179517296,
    1.6702495436871208,
    -1.548304405566199,
    2.923274710289271,
    -1.5483044055659094,
    2.923274710288845,
    2.3325064033150946,
    -0.8498340111344352,
    1.3470393158296798,
    0.5931204424545525,
)


def _prepare_initial_mjdata(m: Any) -> Any:
    """Create and settle an initial `mujoco.MjData` state on the terrain surface."""
    d = mujoco.MjData(m)
    if d.qpos.size == len(_SETTLED_QPOS0) and d.qvel.size == len(_SETTLED_QVEL0):
        d.qpos[:] = np.asarray(_SETTLED_QPOS0, dtype=np.float64)
        d.qvel[:] = np.asarray(_SETTLED_QVEL0, dtype=np.float64)
        v_fwd = 5.0
        d.qvel[0] = v_fwd
        d.qvel[7] = v_fwd / 0.369
        d.qvel[10] = v_fwd / 0.348
        d.qvel[3] = d.qvel[10] * (32.0 / 36.0)
    else:
        if d.qpos.size >= 2:
            d.qpos[0] = 2.0
        if d.qvel.size >= 1:
            d.qvel[0] = 5.0
    mujoco.mj_forward(m, d)
    return d


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


def run_parameter_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    warmup_steps: int = 25,
    seed: int = 42,
    peak_sample_stride: int = 25,
    lib: ctypes.CDLL | None = None,
    device: str = "cuda:0",
) -> dict[str, Any]:
    """Experiment A: Heterogeneous 2048-world parameter sweep in a single CUDA Graph."""
    _require_gpu_stack()
    if lib is None:
        lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)

    m = mujoco.MjModel.from_xml_path(str(model_xml_path))
    d0 = _prepare_initial_mjdata(m)
    dt_s = float(m.opt.timestep)

    # 1. Build host byte buffers for GpuModelIndices and heterogeneous GpuWorldConfig[nworld]
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

    # 2. Allocate device buffers with wp.zeros(..., dtype=wp.uint8) and copy host bytes via wp.copy
    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_configs = _allocate_and_copy_device_bytes(configs_bytes, device=device)
    d_states = wp.zeros(nworld * sizeof_state, dtype=wp.uint8, device=device)
    d_hf_heights, hf_num_samples, hf_x_min, hf_dx = _extract_heightfield_1d(m, device=device)

    # 3. Upload model & batched data to mujoco_warp and initialize states
    m_wp = mjwarp.put_model(m)
    d_b = mjwarp.put_data(m, d0, nworld=nworld)
    mjwarp.forward(m_wp, d_b)

    cuda_stream = wp.get_stream().cuda_stream
    tendon_range_ptr, tendon_range_stride = _resolve_tendon_range_binding(m_wp, nworld)

    lib.bike_gpu_launch_init_states(
        ctypes.c_void_p(cuda_stream),
        int(nworld),
        ctypes.c_void_p(d_idx.ptr),
        ctypes.c_void_p(d_configs.ptr),
        1,
        ctypes.c_void_p(d_states.ptr),
        ctypes.c_void_p(d_b.qpos.ptr),
    )
    wp.synchronize()

    # 4. Capture [lib.bike_gpu_launch_writers(...) + mjwarp.step(m_wp, d_b)] in ONE CUDA Graph
    with wp.ScopedCapture() as capture:
        lib.bike_gpu_launch_writers(
            ctypes.c_void_p(cuda_stream),
            int(nworld),
            ctypes.c_float(dt_s),
            ctypes.c_void_p(d_idx.ptr),
            ctypes.c_void_p(d_hf_heights.ptr),
            int(hf_num_samples),
            ctypes.c_float(hf_x_min),
            ctypes.c_float(hf_dx),
            ctypes.c_void_p(d_configs.ptr),
            1,
            ctypes.c_void_p(d_states.ptr),
            ctypes.c_void_p(d_b.qpos.ptr),
            ctypes.c_void_p(d_b.qvel.ptr),
            ctypes.c_void_p(d_b.xpos.ptr),
            ctypes.c_void_p(d_b.qfrc_applied.ptr),
            ctypes.c_void_p(d_b.ctrl.ptr),
            ctypes.c_void_p(tendon_range_ptr),
            int(tendon_range_stride),
        )
        mjwarp.step(m_wp, d_b)
    graph = capture.graph

    # 5. Warmup graph execution, then reset to clean initial state before timing
    for _ in range(warmup_steps):
        wp.capture_launch(graph)
    wp.synchronize()

    d_b = mjwarp.put_data(m, d0, nworld=nworld)
    mjwarp.forward(m_wp, d_b)
    lib.bike_gpu_launch_init_states(
        ctypes.c_void_p(cuda_stream),
        int(nworld),
        ctypes.c_void_p(d_idx.ptr),
        ctypes.c_void_p(d_configs.ptr),
        1,
        ctypes.c_void_p(d_states.ptr),
        ctypes.c_void_p(d_b.qpos.ptr),
    )
    # Re-capture on the fresh d_b buffer pointers
    with wp.ScopedCapture() as capture:
        lib.bike_gpu_launch_writers(
            ctypes.c_void_p(cuda_stream),
            int(nworld),
            ctypes.c_float(dt_s),
            ctypes.c_void_p(d_idx.ptr),
            ctypes.c_void_p(d_hf_heights.ptr),
            int(hf_num_samples),
            ctypes.c_float(hf_x_min),
            ctypes.c_float(hf_dx),
            ctypes.c_void_p(d_configs.ptr),
            1,
            ctypes.c_void_p(d_states.ptr),
            ctypes.c_void_p(d_b.qpos.ptr),
            ctypes.c_void_p(d_b.qvel.ptr),
            ctypes.c_void_p(d_b.xpos.ptr),
            ctypes.c_void_p(d_b.qfrc_applied.ptr),
            ctypes.c_void_p(d_b.ctrl.ptr),
            ctypes.c_void_p(tendon_range_ptr),
            int(tendon_range_stride),
        )
        mjwarp.step(m_wp, d_b)
    graph = capture.graph
    wp.synchronize()

    # 6. Timed uninterrupted CUDA Graph execution
    t0 = time.perf_counter()
    for _ in range(n_steps):
        wp.capture_launch(graph)
    wp.synchronize()
    wall_time_s = time.perf_counter() - t0

    # 7. Collect physical sweep metrics across all nworld worlds
    #    Replay in chunks of `peak_sample_stride` to record exact peak fork travel and shock stroke
    d_b = mjwarp.put_data(m, d0, nworld=nworld)
    mjwarp.forward(m_wp, d_b)
    lib.bike_gpu_launch_init_states(
        ctypes.c_void_p(cuda_stream),
        int(nworld),
        ctypes.c_void_p(d_idx.ptr),
        ctypes.c_void_p(d_configs.ptr),
        1,
        ctypes.c_void_p(d_states.ptr),
        ctypes.c_void_p(d_b.qpos.ptr),
    )
    with wp.ScopedCapture() as capture:
        lib.bike_gpu_launch_writers(
            ctypes.c_void_p(cuda_stream),
            int(nworld),
            ctypes.c_float(dt_s),
            ctypes.c_void_p(d_idx.ptr),
            ctypes.c_void_p(d_hf_heights.ptr),
            int(hf_num_samples),
            ctypes.c_float(hf_x_min),
            ctypes.c_float(hf_dx),
            ctypes.c_void_p(d_configs.ptr),
            1,
            ctypes.c_void_p(d_states.ptr),
            ctypes.c_void_p(d_b.qpos.ptr),
            ctypes.c_void_p(d_b.qvel.ptr),
            ctypes.c_void_p(d_b.xpos.ptr),
            ctypes.c_void_p(d_b.qfrc_applied.ptr),
            ctypes.c_void_p(d_b.ctrl.ptr),
            ctypes.c_void_p(tendon_range_ptr),
            int(tendon_range_stride),
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

    return {
        "experiment": "A_parameter_sweep",
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


def run_3d_tire_ray_scaling_benchmark(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 500,
    warmup_steps: int = 20,
    lib: ctypes.CDLL | None = None,
    device: str = "cuda:0",
) -> dict[str, Any]:
    """Experiment B: 3D tire ray-grid scaling benchmark across 64, 900, and 7700 rays/world."""
    _require_gpu_stack()
    if lib is None:
        lib = compile_cuda_writers(DEFAULT_GPU_SRC_DIR, DEFAULT_OUT_SO_PATH)

    m = mujoco.MjModel.from_xml_path(str(model_xml_path))
    d0 = _prepare_initial_mjdata(m)
    dt_s = float(m.opt.timestep)

    idx_bytes = resolve_gpu_model_indices(m)
    d_idx = _allocate_and_copy_device_bytes(idx_bytes, device=device)
    d_hf_heights, hf_num_samples, hf_x_min, hf_dx = _extract_heightfield_1d(m, device=device)
    sizeof_state = int(lib.bike_gpu_sizeof_world_state())

    m_wp = mjwarp.put_model(m)
    cuda_stream = wp.get_stream().cuda_stream
    tendon_range_ptr, tendon_range_stride = _resolve_tendon_range_binding(m_wp, nworld)

    tier_results: list[dict[str, Any]] = []
    baseline_us: float | None = None

    for tier in TIRE_RAY_SCALING_TIERS:
        num_stations = int(tier["num_stations"])
        num_lateral_ribs = int(tier["num_lateral_ribs"])
        rays_per_world = int(tier["rays_per_world"])

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

        d_b = mjwarp.put_data(m, d0, nworld=nworld)
        mjwarp.forward(m_wp, d_b)

        lib.bike_gpu_launch_init_states(
            ctypes.c_void_p(cuda_stream),
            int(nworld),
            ctypes.c_void_p(d_idx.ptr),
            ctypes.c_void_p(d_configs.ptr),
            0,
            ctypes.c_void_p(d_states.ptr),
            ctypes.c_void_p(d_b.qpos.ptr),
        )
        wp.synchronize()

        with wp.ScopedCapture() as capture:
            lib.bike_gpu_launch_writers(
                ctypes.c_void_p(cuda_stream),
                int(nworld),
                ctypes.c_float(dt_s),
                ctypes.c_void_p(d_idx.ptr),
                ctypes.c_void_p(d_hf_heights.ptr),
                int(hf_num_samples),
                ctypes.c_float(hf_x_min),
                ctypes.c_float(hf_dx),
                ctypes.c_void_p(d_configs.ptr),
                0,
                ctypes.c_void_p(d_states.ptr),
                ctypes.c_void_p(d_b.qpos.ptr),
                ctypes.c_void_p(d_b.qvel.ptr),
                ctypes.c_void_p(d_b.xpos.ptr),
                ctypes.c_void_p(d_b.qfrc_applied.ptr),
                ctypes.c_void_p(d_b.ctrl.ptr),
                ctypes.c_void_p(tendon_range_ptr),
                int(tendon_range_stride),
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

        if baseline_us is None:
            baseline_us = us_per_step_per_world
        delta_us_vs_2d = us_per_step_per_world - baseline_us

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
                "steps_per_s": steps_per_s,
                "mrays_per_s": mrays_per_s,
                "all_finite": finite_all,
            }
        )

    return {
        "experiment": "B_3d_tire_ray_scaling",
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "dt_s": dt_s,
        "tiers": tier_results,
    }


def _print_report(results: dict[str, Any]) -> None:
    if "parameter_sweep" in results:
        sw = results["parameter_sweep"]
        print("\n=== Experiment A: Batched Parameter Sweep (CUDA Graph) ===")
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

    if "tire_ray_scaling" in results:
        tr = results["tire_ray_scaling"]
        print("\n=== Experiment B: 3D Tire Ray-Grid Scaling Benchmark (CUDA Graph) ===")
        for tier in tr["tiers"]:
            print(
                f"  {tier['label']:<35} | "
                f"grid={tier['num_stations']}x{tier['num_lateral_ribs']} | "
                f"wall={tier['wall_time_ms']:7.2f} ms | "
                f"{tier['us_per_step_per_world']:6.3f} us/step/world "
                f"(+{tier['delta_us_vs_2d_planar']:.3f} us) | "
                f"{tier['steps_per_s']:10,.0f} steps/s | "
                f"{tier['mrays_per_s']:8.1f} Mrays/s"
            )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run batched C++/CUDA force writers + mujoco_warp benchmarks."
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
        results["tire_ray_scaling"] = run_3d_tire_ray_scaling_benchmark(
            model_xml_path=args.model_xml,
            nworld=args.nworld,
            n_steps=args.ray_steps,
            lib=lib,
        )

    _print_report(results)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
