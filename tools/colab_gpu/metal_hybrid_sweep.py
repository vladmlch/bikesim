"""Scenario 3: Apple Metal Unified-Memory Hybrid Batch Sweep & 3D Tire Ray-Grid Scaling Runner.

Compiles `native/src/gpu/metal_batch_runner.mm` against `Metal.framework`, `Foundation.framework`,
and the `libmujoco` shared library bundled inside the `mujoco` Python wheel, then executes:
  1. Experiment A (`run_metal_hybrid_parameter_sweep`):
     Runs `nworld = 2048` heterogeneous worlds in parallel (`fork_psi_scale` in [0.80, 1.25],
     `rebound_scale` in [0.70, 1.40], `shock_rate_scale` in [0.85, 1.20], `tire_stiff_scale` in
     [0.85, 1.20]) using zero-copy Unified Memory (`MTLResourceStorageModeShared`):
     40-core Apple Metal GPU force writers (`FP32`) overlapped with 16-thread C++ CPU `mj_step` (`FP64`).
  2. Experiment B (`run_metal_hybrid_3d_tire_ray_scaling_benchmark`):
     Benchmarks the hybrid Metal GPU writers + 16-thread CPU `mj_step` at `nworld = 2048` (and
     optional `nworld = 1`) across three tire ray-grid resolutions (`64`, `900`, and `7700` rays/world),
     plus pure Metal GPU writers kernel timings.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any

import mujoco
import numpy as np

try:
    from tools.colab_gpu.build_cuda_writers import (
        DEFAULT_GPU_SRC_DIR,
        build_sweep_configs_bytes,
        configure_cuda_writers_cdll,
        resolve_gpu_model_indices,
    )
    from tools.colab_gpu.cpu_batch_sweep import _find_mujoco_wheel_lib_and_include
    from tools.colab_gpu.model_assets import (
        extract_heightfield_1d,
        load_captured_initial_state,
        load_model_with_assets,
    )
    from tools.colab_gpu.mjwarp_batch_sweep import (
        DEFAULT_MODEL_XML,
        TIRE_RAY_SCALING_TIERS,
        _summarize_distribution,
        generate_parameter_sweep_scales,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_cuda_writers import (  # type: ignore[no-redef]
        DEFAULT_GPU_SRC_DIR,
        build_sweep_configs_bytes,
        configure_cuda_writers_cdll,
        resolve_gpu_model_indices,
    )
    from cpu_batch_sweep import _find_mujoco_wheel_lib_and_include  # type: ignore[no-redef]
    from model_assets import (  # type: ignore[no-redef]
        extract_heightfield_1d,
        load_captured_initial_state,
        load_model_with_assets,
    )
    from mjwarp_batch_sweep import (  # type: ignore[no-redef]
        DEFAULT_MODEL_XML,
        TIRE_RAY_SCALING_TIERS,
        _summarize_distribution,
        generate_parameter_sweep_scales,
    )


def compile_metal_batch_runner(
    gpu_src_dir: Path = DEFAULT_GPU_SRC_DIR,
    out_so_path: Path | None = None,
) -> ctypes.CDLL:
    """Compile `native/src/gpu/metal_batch_runner.mm` and bind its C API."""
    if sys.platform != "darwin":
        raise RuntimeError("compile_metal_batch_runner requires macOS with Metal.framework.")

    gpu_src_dir = Path(gpu_src_dir).resolve()
    mm_path = gpu_src_dir / "metal_batch_runner.mm"
    if not mm_path.is_file():
        raise FileNotFoundError(f"Metal batch runner source not found: {mm_path}")

    mj_lib_path, mj_inc_dir = _find_mujoco_wheel_lib_and_include()
    if out_so_path is None:
        out_so_path = gpu_src_dir.parents[2] / "build" / "libbike_metal_batch_runner.dylib"
    out_so_path = Path(out_so_path).resolve()
    out_so_path.parent.mkdir(parents=True, exist_ok=True)
    obj_path = out_so_path.with_suffix(".o")

    compile_cmd = [
        "clang++",
        "-std=c++20",
        "-O3",
        "-ffast-math",
        "-mcpu=native",
        "-fobjc-arc",
        "-fPIC",
        f"-I{gpu_src_dir}",
        f"-I{mj_inc_dir}",
        "-c",
        str(mm_path),
        "-o",
        str(obj_path),
    ]
    proc = subprocess.run(compile_cmd, check=False, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"Objective-C++ compile failed (exit {proc.returncode}):\n"
            f"Command: {' '.join(compile_cmd)}\n"
            f"STDOUT:\n{proc.stdout}\n"
            f"STDERR:\n{proc.stderr}"
        )

    link_cmd = [
        "clang++",
        "-dynamiclib",
        "-fobjc-arc",
        "-framework",
        "Metal",
        "-framework",
        "Foundation",
        str(obj_path),
        str(mj_lib_path),
        f"-Wl,-rpath,{mj_lib_path.parent}",
        "-o",
        str(out_so_path),
    ]
    proc_link = subprocess.run(link_cmd, check=False, capture_output=True, text=True)
    if proc_link.returncode != 0:
        raise RuntimeError(
            f"Objective-C++ link failed (exit {proc_link.returncode}):\n"
            f"Command: {' '.join(link_cmd)}\n"
            f"STDOUT:\n{proc_link.stdout}\n"
            f"STDERR:\n{proc_link.stderr}"
        )

    lib = configure_cuda_writers_cdll(ctypes.CDLL(str(out_so_path)))

    lib.bike_metal_init.argtypes = [ctypes.c_char_p]
    lib.bike_metal_init.restype = ctypes.c_int

    lib.bike_metal_device_name.argtypes = []
    lib.bike_metal_device_name.restype = ctypes.c_char_p

    lib.bike_metal_get_shader_source.argtypes = [ctypes.c_char_p]
    lib.bike_metal_get_shader_source.restype = ctypes.c_char_p

    lib.bike_metal_launch_init_states.argtypes = [
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    lib.bike_metal_launch_init_states.restype = ctypes.c_int

    lib.bike_metal_launch_writers.argtypes = [
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_int,
    ]
    lib.bike_metal_launch_writers.restype = ctypes.c_int

    lib.bike_metal_benchmark_writers_only.argtypes = [
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.bike_metal_benchmark_writers_only.restype = ctypes.c_int

    lib.bike_metal_run_hybrid_sweep.argtypes = [
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.bike_metal_run_hybrid_sweep.restype = ctypes.c_int

    rc = lib.bike_metal_init(str(gpu_src_dir).encode("utf-8"))
    if rc != 0:
        raise RuntimeError(f"bike_metal_init failed with code {rc}")
    return lib


def run_metal_hybrid_parameter_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    num_threads: int = 0,
    num_stations: int = 32,
    num_lateral_ribs: int = 1,
    lib: ctypes.CDLL | None = None,
) -> dict[str, Any]:
    """Run Experiment A (`nworld` heterogeneous parameter sweep) in Scenario 3 Hybrid mode."""
    if lib is None:
        lib = compile_metal_batch_runner()

    model_xml_path = Path(model_xml_path).resolve()
    m = load_model_with_assets(model_xml_path)
    dt_us = float(m.opt.timestep) * 1e6
    hw_threads = os.cpu_count() or 1
    used_threads = num_threads if num_threads > 0 else hw_threads
    gpu_name = lib.bike_metal_device_name().decode("utf-8")

    idx_bytes = resolve_gpu_model_indices(m)
    idx_buf = ctypes.create_string_buffer(idx_bytes)
    hf_arr, hf_x_min, hf_dx = extract_heightfield_1d(m)
    hf_ncol = int(hf_arr.size)
    init_qpos, init_qvel = load_captured_initial_state(model_xml_path, int(m.nq), int(m.nv))

    scales = generate_parameter_sweep_scales(nworld=nworld, seed=42)
    configs_bytes = build_sweep_configs_bytes(
        lib,
        nworld=nworld,
        fork_psi_scale=scales["fork_psi_scale"],
        rebound_scale=scales["rebound_scale"],
        shock_rate_scale=scales["shock_rate_scale"],
        tire_stiff_scale=scales["tire_stiff_scale"],
        num_stations=num_stations,
        num_lateral_ribs=num_lateral_ribs,
    )
    cfg_buf = ctypes.create_string_buffer(configs_bytes)

    out_fork_mm = np.zeros(nworld, dtype=np.float32)
    out_shock_mm = np.zeros(nworld, dtype=np.float32)
    out_root_x_m = np.zeros(nworld, dtype=np.float32)
    all_finite_c = ctypes.c_int(0)
    elapsed_ms_c = ctypes.c_double(0.0)

    rc = lib.bike_metal_run_hybrid_sweep(
        str(model_xml_path).encode("utf-8"),
        int(nworld),
        int(n_steps),
        int(used_threads),
        ctypes.cast(idx_buf, ctypes.c_void_p),
        hf_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        int(hf_ncol),
        ctypes.c_float(hf_x_min),
        ctypes.c_float(hf_dx),
        ctypes.cast(cfg_buf, ctypes.c_void_p),
        1,
        init_qpos.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        init_qvel.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        out_fork_mm.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        out_shock_mm.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        out_root_x_m.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        ctypes.byref(all_finite_c),
        ctypes.byref(elapsed_ms_c),
    )
    if rc != 0:
        raise RuntimeError(f"bike_metal_run_hybrid_sweep failed with return code {rc}")

    elapsed_ms = float(elapsed_ms_c.value)
    total_world_steps = float(nworld) * float(n_steps)
    us_per_step_per_world = (elapsed_ms * 1000.0) / total_world_steps
    steps_per_sec = total_world_steps / (elapsed_ms * 1e-3)
    batch_rtf = dt_us / us_per_step_per_world

    return {
        "experiment": "metal_hybrid_parameter_sweep_2048",
        "platform": f"{platform.system()}-{platform.machine()}",
        "gpu_device": gpu_name,
        "num_threads": int(used_threads),
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "num_stations": int(num_stations),
        "num_lateral_ribs": int(num_lateral_ribs),
        "rays_per_world": int(2 * num_stations * num_lateral_ribs),
        "wall_time_ms": round(elapsed_ms, 3),
        "us_per_step_per_world": round(us_per_step_per_world, 4),
        "total_steps_per_sec": round(steps_per_sec, 1),
        "batch_rtf": round(batch_rtf, 2),
        "all_finite": bool(all_finite_c.value != 0),
        "metrics": {
            "peak_fork_travel_mm": _summarize_distribution(out_fork_mm),
            "peak_shock_stroke_mm": _summarize_distribution(out_shock_mm),
            "final_root_x_m": _summarize_distribution(out_root_x_m),
        },
    }


def run_metal_hybrid_3d_tire_ray_scaling_benchmark(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 500,
    num_threads: int = 0,
    lib: ctypes.CDLL | None = None,
) -> dict[str, Any]:
    """Run Experiment B (3D tire ray-grid scaling across 64, 900, and 7700 rays/world) in Scenario 3."""
    if lib is None:
        lib = compile_metal_batch_runner()

    model_xml_path = Path(model_xml_path).resolve()
    m = load_model_with_assets(model_xml_path)
    dt_s = float(m.opt.timestep)
    dt_us = dt_s * 1e6
    hw_threads = os.cpu_count() or 1
    used_threads = num_threads if num_threads > 0 else hw_threads
    gpu_name = lib.bike_metal_device_name().decode("utf-8")

    idx_bytes = resolve_gpu_model_indices(m)
    idx_buf = ctypes.create_string_buffer(idx_bytes)
    hf_arr, hf_x_min, hf_dx = extract_heightfield_1d(m)
    hf_ncol = int(hf_arr.size)
    init_qpos, init_qvel = load_captured_initial_state(model_xml_path, int(m.nq), int(m.nv))

    tiers_report: dict[str, Any] = {}
    for tier in TIRE_RAY_SCALING_TIERS:
        tier_id = str(tier["tier_id"])
        n_stat = int(tier["num_stations"])
        n_ribs = int(tier["num_lateral_ribs"])
        rays_per_world = int(tier["rays_per_world"])

        configs_bytes = build_sweep_configs_bytes(
            lib,
            nworld=nworld,
            num_stations=n_stat,
            num_lateral_ribs=n_ribs,
        )
        cfg_buf = ctypes.create_string_buffer(configs_bytes)

        # Measure pure Metal GPU writers kernel timings (single CB vs per-step CB)
        single_cb_ms_c = ctypes.c_double(0.0)
        per_step_cb_ms_c = ctypes.c_double(0.0)
        rc = lib.bike_metal_benchmark_writers_only(
            int(nworld),
            int(n_steps),
            ctypes.c_float(dt_s),
            ctypes.cast(idx_buf, ctypes.c_void_p),
            hf_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            int(hf_ncol),
            ctypes.c_float(hf_x_min),
            ctypes.c_float(hf_dx),
            ctypes.cast(cfg_buf, ctypes.c_void_p),
            1,
            ctypes.byref(single_cb_ms_c),
            ctypes.byref(per_step_cb_ms_c),
        )
        if rc != 0:
            raise RuntimeError(f"bike_metal_benchmark_writers_only failed with code {rc}")
        total_world_steps = float(nworld) * float(n_steps)
        writers_single_cb_us = (float(single_cb_ms_c.value) * 1000.0) / total_world_steps
        writers_per_step_cb_us = (float(per_step_cb_ms_c.value) * 1000.0) / total_world_steps

        out_fork_mm = np.zeros(nworld, dtype=np.float32)
        out_shock_mm = np.zeros(nworld, dtype=np.float32)
        out_root_x_m = np.zeros(nworld, dtype=np.float32)
        all_finite_c = ctypes.c_int(0)
        elapsed_ms_c = ctypes.c_double(0.0)

        rc = lib.bike_metal_run_hybrid_sweep(
            str(model_xml_path).encode("utf-8"),
            int(nworld),
            int(n_steps),
            int(used_threads),
            ctypes.cast(idx_buf, ctypes.c_void_p),
            hf_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            int(hf_ncol),
            ctypes.c_float(hf_x_min),
            ctypes.c_float(hf_dx),
            ctypes.cast(cfg_buf, ctypes.c_void_p),
            1,
            init_qpos.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            init_qvel.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            out_fork_mm.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            out_shock_mm.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            out_root_x_m.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            ctypes.byref(all_finite_c),
            ctypes.byref(elapsed_ms_c),
        )
        if rc != 0:
            raise RuntimeError(f"bike_metal_run_hybrid_sweep ({tier_id}) failed with code {rc}")

        elapsed_ms = float(elapsed_ms_c.value)
        us_per_step_per_world = (elapsed_ms * 1000.0) / total_world_steps
        steps_per_sec = total_world_steps / (elapsed_ms * 1e-3)
        mrays_per_sec = (steps_per_sec * float(rays_per_world)) / 1e6
        batch_rtf = dt_us / us_per_step_per_world

        tiers_report[tier_id] = {
            "label": str(tier["label"]),
            "num_stations": n_stat,
            "num_lateral_ribs": n_ribs,
            "rays_per_wheel": int(tier["rays_per_wheel"]),
            "rays_per_world": rays_per_world,
            "total_rays_per_step_batch": int(nworld * rays_per_world),
            "wall_time_ms": round(elapsed_ms, 3),
            "us_per_step_per_world": round(us_per_step_per_world, 4),
            "writers_only_single_cb_us_per_world": round(writers_single_cb_us, 4),
            "writers_only_per_step_cb_us_per_world": round(writers_per_step_cb_us, 4),
            "total_steps_per_sec": round(steps_per_sec, 1),
            "mrays_per_sec": round(mrays_per_sec, 2),
            "batch_rtf": round(batch_rtf, 2),
            "all_finite": bool(all_finite_c.value != 0),
        }

    base_us = float(tiers_report["2d_planar_64"]["us_per_step_per_world"])
    for tier_id, info in tiers_report.items():
        cur_us = float(info["us_per_step_per_world"])
        info["overhead_us_vs_2d"] = round(cur_us - base_us, 4)
        info["overhead_pct_vs_2d"] = (
            round(100.0 * (cur_us - base_us) / base_us, 2) if base_us > 0.0 else 0.0
        )

    return {
        "experiment": "metal_hybrid_3d_tire_ray_scaling",
        "platform": f"{platform.system()}-{platform.machine()}",
        "gpu_device": gpu_name,
        "num_threads": int(used_threads),
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "tiers": tiers_report,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run Scenario 3 (Metal GPU Writers + Multi-Threaded C++ CPU mj_step) benchmarks."
    )
    parser.add_argument("--model-xml", type=Path, default=DEFAULT_MODEL_XML)
    parser.add_argument("--nworld", type=int, default=2048)
    parser.add_argument("--n-steps", type=int, default=1000)
    parser.add_argument("--ray-steps", type=int, default=500)
    parser.add_argument("--num-threads", type=int, default=0)
    parser.add_argument("--out-so", type=Path, default=None)
    parser.add_argument("--json-out", type=Path, default=None)
    args = parser.parse_args(argv)

    lib = compile_metal_batch_runner(out_so_path=args.out_so)
    sweep_report = run_metal_hybrid_parameter_sweep(
        model_xml_path=args.model_xml,
        nworld=args.nworld,
        n_steps=args.n_steps,
        num_threads=args.num_threads,
        lib=lib,
    )
    ray_report = run_metal_hybrid_3d_tire_ray_scaling_benchmark(
        model_xml_path=args.model_xml,
        nworld=args.nworld,
        n_steps=args.ray_steps,
        num_threads=args.num_threads,
        lib=lib,
    )
    ray_single_world = run_metal_hybrid_3d_tire_ray_scaling_benchmark(
        model_xml_path=args.model_xml,
        nworld=1,
        n_steps=args.ray_steps,
        num_threads=1,
        lib=lib,
    )

    payload = {
        "parameter_sweep": sweep_report,
        "tire_ray_scaling": ray_report,
        "tire_ray_scaling_single_world": ray_single_world,
    }
    rendered = json.dumps(payload, indent=2)
    print(rendered)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
