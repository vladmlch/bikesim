"""Multi-threaded C++ CPU Batch Sweep & 3D Tire Ray-Grid Scaling Runner.

Compiles `native/src/gpu/cpu_batch_runner.cpp` against the `libmujoco` shared library bundled inside
the `mujoco` Python wheel and executes:
  1. Experiment A (`run_cpu_parameter_sweep`):
     Runs `nworld = 2048` heterogeneous worlds in parallel across all CPU hardware threads
     (`fork_psi_scale` in [0.80, 1.25], `rebound_scale` in [0.70, 1.40], `shock_rate_scale` in
     [0.85, 1.20], `tire_stiff_scale` in [0.85, 1.20]) inside an allocation-free C++ thread pool.
  2. Experiment B (`run_cpu_3d_tire_ray_scaling_benchmark`):
     Benchmarks the multi-threaded C++ force writers + `mj_step` at `nworld = 2048` across three
     tire ray-grid resolutions (`64`, `900`, and `7700` rays/world).
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

REPO_ROOT = Path(__file__).resolve().parents[2]


def _find_mujoco_wheel_lib_and_include() -> tuple[Path, Path]:
    mj_dir = Path(mujoco.__file__).resolve().parent
    inc_dir = mj_dir / "include"
    if not (inc_dir / "mujoco" / "mujoco.h").is_file():
        raise FileNotFoundError(f"mujoco.h not found under {inc_dir}")

    candidates = sorted(mj_dir.glob("libmujoco*.dylib")) + sorted(mj_dir.glob("libmujoco*.so*"))
    if not candidates:
        raise FileNotFoundError(f"libmujoco shared library not found in {mj_dir}")
    return candidates[0], inc_dir


def compile_cpu_batch_runner(
    gpu_src_dir: Path = DEFAULT_GPU_SRC_DIR,
    out_so_path: Path | None = None,
) -> ctypes.CDLL:
    """Compile `native/src/gpu/cpu_batch_runner.cpp` and bind its C API."""
    gpu_src_dir = Path(gpu_src_dir).resolve()
    cpp_path = gpu_src_dir / "cpu_batch_runner.cpp"
    if not cpp_path.is_file():
        raise FileNotFoundError(f"C++ batch runner source not found: {cpp_path}")

    mj_lib_path, mj_inc_dir = _find_mujoco_wheel_lib_and_include()
    is_macos = sys.platform == "darwin"
    if out_so_path is None:
        ext = ".dylib" if is_macos else ".so"
        out_so_path = gpu_src_dir.parents[2] / "build" / f"libbike_cpu_batch_runner{ext}"
    out_so_path = Path(out_so_path).resolve()
    out_so_path.parent.mkdir(parents=True, exist_ok=True)

    if is_macos:
        cmd = [
            "clang++",
            "-std=c++20",
            "-O3",
            "-ffast-math",
            "-mcpu=native",
            "-dynamiclib",
            "-fPIC",
            f"-I{gpu_src_dir}",
            f"-I{mj_inc_dir}",
            str(cpp_path),
            str(mj_lib_path),
            f"-Wl,-rpath,{mj_lib_path.parent}",
            "-o",
            str(out_so_path),
        ]
    else:
        cmd = [
            "g++",
            "-std=c++20",
            "-O3",
            "-ffast-math",
            "-march=native",
            "-shared",
            "-fPIC",
            "-pthread",
            f"-I{gpu_src_dir}",
            f"-I{mj_inc_dir}",
            str(cpp_path),
            str(mj_lib_path),
            f"-Wl,-rpath,{mj_lib_path.parent}",
            "-o",
            str(out_so_path),
        ]

    proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"C++ compilation failed (exit {proc.returncode}):\n"
            f"Command: {' '.join(cmd)}\n"
            f"STDOUT:\n{proc.stdout}\n"
            f"STDERR:\n{proc.stderr}"
        )

    lib = configure_cuda_writers_cdll(ctypes.CDLL(str(out_so_path)))
    lib.bike_cpu_run_batch_sweep.argtypes = [
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
    lib.bike_cpu_run_batch_sweep.restype = ctypes.c_int
    return lib


def run_cpu_point(
    xml_path: Path | str = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    num_threads: int = 0,
    num_stations: int = 32,
    num_lateral_ribs: int = 1,
    lib: ctypes.CDLL | None = None,
) -> dict[str, Any]:
    """Run a single CPU batch benchmark point after loading `MjModel` + `hfield_data.bin`."""
    resolved_xml = Path(xml_path) if xml_path is not None else DEFAULT_MODEL_XML
    _ = load_model_with_assets(resolved_xml)
    return run_cpu_parameter_sweep(
        model_xml_path=resolved_xml,
        nworld=nworld,
        n_steps=n_steps,
        num_threads=num_threads,
        num_stations=num_stations,
        num_lateral_ribs=num_lateral_ribs,
        lib=lib,
    )


def run_cpu_parameter_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    num_threads: int = 0,
    num_stations: int = 32,
    num_lateral_ribs: int = 1,
    lib: ctypes.CDLL | None = None,
) -> dict[str, Any]:
    """Run Experiment A (`nworld` heterogeneous parameter sweep) across all CPU threads."""
    if lib is None:
        lib = compile_cpu_batch_runner()

    resolved_xml = Path(model_xml_path) if model_xml_path is not None else DEFAULT_MODEL_XML
    model_xml_path = resolved_xml.resolve()
    m = load_model_with_assets(model_xml_path)
    dt_us = float(m.opt.timestep) * 1e6
    hw_threads = os.cpu_count() or 1
    used_threads = num_threads if num_threads > 0 else hw_threads

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

    rc = lib.bike_cpu_run_batch_sweep(
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
        raise RuntimeError(f"bike_cpu_run_batch_sweep failed with return code {rc}")

    elapsed_ms = float(elapsed_ms_c.value)
    total_world_steps = float(nworld) * float(n_steps)
    us_per_step_per_world = (elapsed_ms * 1000.0) / total_world_steps
    steps_per_sec = total_world_steps / (elapsed_ms * 1e-3)
    batch_rtf = dt_us / us_per_step_per_world

    return {
        "experiment": "cpu_parameter_sweep_2048",
        "platform": f"{platform.system()}-{platform.machine()}",
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


def run_cpu_3d_tire_ray_scaling_benchmark(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 500,
    num_threads: int = 0,
    lib: ctypes.CDLL | None = None,
) -> dict[str, Any]:
    """Run Experiment B (3D tire ray-grid scaling across 64, 900, and 7700 rays/world) on CPU."""
    if lib is None:
        lib = compile_cpu_batch_runner()

    resolved_xml = Path(model_xml_path) if model_xml_path is not None else DEFAULT_MODEL_XML
    model_xml_path = resolved_xml.resolve()
    m = load_model_with_assets(model_xml_path)
    dt_us = float(m.opt.timestep) * 1e6
    hw_threads = os.cpu_count() or 1
    used_threads = num_threads if num_threads > 0 else hw_threads

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

        out_fork_mm = np.zeros(nworld, dtype=np.float32)
        out_shock_mm = np.zeros(nworld, dtype=np.float32)
        out_root_x_m = np.zeros(nworld, dtype=np.float32)
        all_finite_c = ctypes.c_int(0)
        elapsed_ms_c = ctypes.c_double(0.0)

        rc = lib.bike_cpu_run_batch_sweep(
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
            raise RuntimeError(f"bike_cpu_run_batch_sweep ({tier_id}) failed with code {rc}")

        elapsed_ms = float(elapsed_ms_c.value)
        total_world_steps = float(nworld) * float(n_steps)
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
        "experiment": "cpu_3d_tire_ray_scaling",
        "platform": f"{platform.system()}-{platform.machine()}",
        "num_threads": int(used_threads),
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "tiers": tiers_report,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run multi-threaded C++ CPU parameter sweep and 3D tire ray scaling benchmarks."
    )
    parser.add_argument(
        "--model-xml",
        "--xml-path",
        dest="model_xml",
        type=Path,
        default=None,
    )
    parser.add_argument("--nworld", type=int, default=2048)
    parser.add_argument("--n-steps", type=int, default=1000)
    parser.add_argument("--ray-steps", type=int, default=500)
    parser.add_argument("--num-threads", type=int, default=0)
    parser.add_argument(
        "--out-so",
        type=Path,
        default=None,
        help="Optional output path for compiled shared library.",
    )
    parser.add_argument("--json-out", type=Path, default=None)
    args = parser.parse_args(argv)

    model_xml = args.model_xml if args.model_xml is not None else DEFAULT_MODEL_XML
    lib = compile_cpu_batch_runner(out_so_path=args.out_so)
    sweep_report = run_cpu_parameter_sweep(
        model_xml_path=model_xml,
        nworld=args.nworld,
        n_steps=args.n_steps,
        num_threads=args.num_threads,
        lib=lib,
    )
    ray_report = run_cpu_3d_tire_ray_scaling_benchmark(
        model_xml_path=model_xml,
        nworld=args.nworld,
        n_steps=args.ray_steps,
        num_threads=args.num_threads,
        lib=lib,
    )

    payload = {
        "parameter_sweep": sweep_report,
        "tire_ray_scaling": ray_report,
    }
    rendered = json.dumps(payload, indent=2)
    print(rendered)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
