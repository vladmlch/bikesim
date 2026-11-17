"""Multi-threaded C++ CPU Batch Sweep & Tyre Backend Scaling Runner.

Compiles `native/src/gpu/cpu_batch_runner.cpp`, `cuda_kernels_host.cpp`, and
`native/src/physics_core/tyre_upload.cpp` against the `libmujoco` shared library bundled inside
the `mujoco` Python wheel (`-ffp-contract=off`, no `-ffast-math`) and executes:
  1. Experiment A (`run_cpu_parameter_sweep`):
     Runs `nworld = 2048` heterogeneous worlds in parallel across all CPU hardware threads
     (`fork_psi_scale` in [0.80, 1.25], `rebound_scale` in [0.70, 1.40], `shock_rate_scale` in
     [0.85, 1.20], `tire_stiff_scale` in [0.85, 1.20]) inside an allocation-free C++ thread pool.
  2. Experiment B (`run_cpu_tyre_backend_scaling_benchmark`):
     Benchmarks the multi-threaded C++ force writers + `mj_step` at `nworld = 2048` across the
     shared `tyre_gpu_step.hpp` backends (`compliant_2d`, `distributed_2d_reference` N=32/64/128).
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
        build_tyre_and_terrain_buffers,
        configure_cuda_writers_cdll,
        resolve_gpu_model_indices,
    )
    from tools.colab_gpu.model_assets import (
        load_captured_initial_state,
        load_model_with_assets,
    )
    from tools.colab_gpu.mjwarp_batch_sweep import (
        DEFAULT_MODEL_XML,
        TYRE_BACKEND_TIERS,
        _summarize_distribution,
        generate_parameter_sweep_scales,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_cuda_writers import (  # type: ignore[no-redef]
        DEFAULT_GPU_SRC_DIR,
        build_sweep_configs_bytes,
        build_tyre_and_terrain_buffers,
        configure_cuda_writers_cdll,
        resolve_gpu_model_indices,
    )
    from model_assets import (  # type: ignore[no-redef]
        load_captured_initial_state,
        load_model_with_assets,
    )
    from mjwarp_batch_sweep import (  # type: ignore[no-redef]
        DEFAULT_MODEL_XML,
        TYRE_BACKEND_TIERS,
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
    """Compile `cpu_batch_runner.cpp`, `cuda_kernels_host.cpp`, and `tyre_upload.cpp`."""
    gpu_src_dir = Path(gpu_src_dir).resolve()
    native_src_dir = gpu_src_dir.parent
    cpp_path = gpu_src_dir / "cpu_batch_runner.cpp"
    host_kernels_cpp = gpu_src_dir / "cuda_kernels_host.cpp"
    tyre_upload_cpp = native_src_dir / "physics_core" / "tyre_upload.cpp"
    for p in (cpp_path, host_kernels_cpp, tyre_upload_cpp):
        if not p.is_file():
            raise FileNotFoundError(f"Required C++ source not found: {p}")

    mj_lib_path, mj_inc_dir = _find_mujoco_wheel_lib_and_include()
    is_macos = sys.platform == "darwin"
    if out_so_path is None:
        ext = ".dylib" if is_macos else ".so"
        out_so_path = gpu_src_dir.parents[2] / "build" / f"libbike_cpu_batch_runner{ext}"
    out_so_path = Path(out_so_path).resolve()
    out_so_path.parent.mkdir(parents=True, exist_ok=True)

    sources = [str(cpp_path), str(host_kernels_cpp), str(tyre_upload_cpp)]
    if is_macos:
        cmd = [
            "clang++",
            "-std=c++20",
            "-O3",
            "-ffp-contract=off",
            "-mcpu=native",
            "-dynamiclib",
            "-fPIC",
            f"-I{native_src_dir}",
            f"-I{gpu_src_dir}",
            f"-I{mj_inc_dir}",
            *sources,
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
            "-ffp-contract=off",
            "-march=native",
            "-shared",
            "-fPIC",
            "-pthread",
            f"-I{native_src_dir}",
            f"-I{gpu_src_dir}",
            f"-I{mj_inc_dir}",
            *sources,
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
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_int32),
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
    backend: str = "compliant_2d",
    station_count: int = 64,
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
        backend=backend,
        station_count=station_count,
        lib=lib,
    )


def run_cpu_parameter_sweep(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 1000,
    num_threads: int = 0,
    backend: str = "compliant_2d",
    station_count: int = 64,
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
    model_b, desc_b, f32_arr, i32_arr = build_tyre_and_terrain_buffers(
        lib, model_xml_path, backend=backend, station_count=station_count, idx_bytes=idx_bytes
    )
    model_buf = ctypes.create_string_buffer(model_b)
    desc_buf = ctypes.create_string_buffer(desc_b)
    f32_ptr = f32_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float)) if f32_arr.size > 0 else None
    i32_ptr = i32_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)) if i32_arr.size > 0 else None
    init_qpos, init_qvel = load_captured_initial_state(model_xml_path, int(m.nq), int(m.nv))

    scales = generate_parameter_sweep_scales(nworld=nworld, seed=42)
    configs_bytes = build_sweep_configs_bytes(
        lib,
        nworld=nworld,
        fork_psi_scale=scales["fork_psi_scale"],
        rebound_scale=scales["rebound_scale"],
        shock_rate_scale=scales["shock_rate_scale"],
        tire_stiff_scale=scales["tire_stiff_scale"],
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
        ctypes.cast(model_buf, ctypes.c_void_p),
        ctypes.cast(desc_buf, ctypes.c_void_p),
        f32_ptr,
        i32_ptr,
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
        "backend": backend,
        "station_count": int(station_count) if backend == "distributed_2d_reference" else 0,
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


def run_cpu_tyre_backend_scaling_benchmark(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 500,
    num_threads: int = 0,
    lib: ctypes.CDLL | None = None,
) -> dict[str, Any]:
    """Run Experiment B (shared tyre_gpu_step backend scaling across compliant_2d and distributed)."""
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
    init_qpos, init_qvel = load_captured_initial_state(model_xml_path, int(m.nq), int(m.nv))

    configs_bytes = build_sweep_configs_bytes(lib, nworld=nworld)
    cfg_buf = ctypes.create_string_buffer(configs_bytes)

    tiers_report: dict[str, Any] = {}
    for tier in TYRE_BACKEND_TIERS:
        tier_id = str(tier["tier_id"])
        backend = str(tier["backend"])
        station_count = int(tier["station_count"])

        model_b, desc_b, f32_arr, i32_arr = build_tyre_and_terrain_buffers(
            lib, model_xml_path, backend=backend, station_count=station_count, idx_bytes=idx_bytes
        )
        model_buf = ctypes.create_string_buffer(model_b)
        desc_buf = ctypes.create_string_buffer(desc_b)
        f32_ptr = f32_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float)) if f32_arr.size > 0 else None
        i32_ptr = i32_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)) if i32_arr.size > 0 else None

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
            ctypes.cast(model_buf, ctypes.c_void_p),
            ctypes.cast(desc_buf, ctypes.c_void_p),
            f32_ptr,
            i32_ptr,
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
        batch_rtf = dt_us / us_per_step_per_world

        tiers_report[tier_id] = {
            "label": str(tier["label"]),
            "backend": backend,
            "station_count": station_count,
            "units_per_wheel": int(tier["units_per_wheel"]),
            "units_per_world": int(tier["units_per_world"]),
            "wall_time_ms": round(elapsed_ms, 3),
            "us_per_step_per_world": round(us_per_step_per_world, 4),
            "total_steps_per_sec": round(steps_per_sec, 1),
            "batch_rtf": round(batch_rtf, 2),
            "all_finite": bool(all_finite_c.value != 0),
        }

    base_us = float(tiers_report["compliant_2d"]["us_per_step_per_world"])
    for tier_id, info in tiers_report.items():
        cur_us = float(info["us_per_step_per_world"])
        info["overhead_us_vs_compliant"] = round(cur_us - base_us, 4)
        info["overhead_pct_vs_compliant"] = (
            round(100.0 * (cur_us - base_us) / base_us, 2) if base_us > 0.0 else 0.0
        )

    return {
        "experiment": "cpu_tyre_backend_scaling",
        "platform": f"{platform.system()}-{platform.machine()}",
        "num_threads": int(used_threads),
        "nworld": int(nworld),
        "n_steps": int(n_steps),
        "tiers": tiers_report,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run multi-threaded C++ CPU parameter sweep and tyre backend scaling benchmarks."
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
    parser.add_argument("--tier-steps", type=int, default=500)
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
    backend_report = run_cpu_tyre_backend_scaling_benchmark(
        model_xml_path=model_xml,
        nworld=args.nworld,
        n_steps=args.tier_steps,
        num_threads=args.num_threads,
        lib=lib,
    )

    payload = {
        "parameter_sweep": sweep_report,
        "tyre_backend_scaling": backend_report,
    }
    rendered = json.dumps(payload, indent=2)
    print(rendered)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
