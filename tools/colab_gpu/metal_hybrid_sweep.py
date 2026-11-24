"""Scenario 3: Apple Metal Unified-Memory Hybrid Batch Sweep & Tyre Backend Scaling Runner.

Compiles `native/src/gpu/metal_batch_runner.mm`, `cuda_kernels_host.cpp`, and
`native/src/physics_core/tyre_upload.cpp` (`-ffp-contract=off`, no `-ffast-math`,
`MTLMathModeSafe`) against `Metal.framework`, `Foundation.framework`, and the `libmujoco`
shared library bundled inside the `mujoco` Python wheel, then executes:
  1. Experiment A (`run_metal_hybrid_parameter_sweep`):
     Runs `nworld = 2048` heterogeneous worlds in parallel (`fork_psi_scale` in [0.80, 1.25],
     `rebound_scale` in [0.70, 1.40], `shock_rate_scale` in [0.85, 1.20], `tire_stiff_scale` in
     [0.85, 1.20]) using zero-copy Unified Memory (`MTLResourceStorageModeShared`):
     Apple Metal GPU force writers (`FP32`) overlapped with multi-threaded C++ CPU `mj_step` (`FP64`).
  2. Experiment B (`run_metal_hybrid_tyre_backend_scaling_benchmark`):
     Benchmarks the hybrid Metal GPU writers + CPU `mj_step` at `nworld = 2048` (and optional
     `nworld = 1`) across the shared `tyre_gpu_step.hpp` backends (`compliant_2d`,
     `distributed_2d_reference` N=32/64/128), plus pure Metal GPU writers kernel timings.
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
    from tools.colab_gpu.cpu_batch_sweep import _find_mujoco_wheel_lib_and_include
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
    from cpu_batch_sweep import _find_mujoco_wheel_lib_and_include  # type: ignore[no-redef]
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


def compile_metal_batch_runner(
    gpu_src_dir: Path = DEFAULT_GPU_SRC_DIR,
    out_so_path: Path | None = None,
) -> ctypes.CDLL:
    """Compile `metal_batch_runner.mm` and `tyre_upload.cpp`."""
    if sys.platform != "darwin":
        raise RuntimeError("compile_metal_batch_runner requires macOS with Metal.framework.")

    gpu_src_dir = Path(gpu_src_dir).resolve()
    native_src_dir = gpu_src_dir.parent
    mm_path = gpu_src_dir / "metal_batch_runner.mm"
    tyre_upload_cpp = native_src_dir / "physics_core" / "tyre_upload.cpp"
    spindle_upload_cpp = native_src_dir / "physics_core" / "spindle_m2k2_upload.cpp"
    for p in (mm_path, tyre_upload_cpp, spindle_upload_cpp):
        if not p.is_file():
            raise FileNotFoundError(f"Required Metal batch runner source not found: {p}")

    mj_lib_path, mj_inc_dir = _find_mujoco_wheel_lib_and_include()
    if out_so_path is None:
        out_so_path = gpu_src_dir.parents[2] / "build" / "libbike_metal_batch_runner.dylib"
    out_so_path = Path(out_so_path).resolve()
    out_so_path.parent.mkdir(parents=True, exist_ok=True)
    obj_mm = out_so_path.with_suffix(".mm.o")
    obj_tu = out_so_path.with_suffix(".tu.o")
    obj_su = out_so_path.with_suffix(".su.o")

    colab_gpu_dir = Path(__file__).resolve().parent
    common_flags = [
        "clang++",
        "-std=c++20",
        "-O3",
        "-ffp-contract=off",
        "-mcpu=native",
        "-fPIC",
        "-DBIKE_GPU_STANDALONE_RUNNER=1",
        f"-I{native_src_dir}",
        f"-I{gpu_src_dir}",
        f"-I{colab_gpu_dir}",
        f"-I{mj_inc_dir}",
    ]
    compile_jobs = [
        ([*common_flags, "-fobjc-arc", "-c", str(mm_path), "-o", str(obj_mm)], "Objective-C++"),
        ([*common_flags, "-c", str(tyre_upload_cpp), "-o", str(obj_tu)], "tyre_upload"),
        ([*common_flags, "-c", str(spindle_upload_cpp), "-o", str(obj_su)], "spindle_upload"),
    ]
    for compile_cmd, label in compile_jobs:
        proc = subprocess.run(compile_cmd, check=False, capture_output=True, text=True)
        if proc.returncode != 0:
            raise RuntimeError(
                f"{label} compile failed (exit {proc.returncode}):\n"
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
        str(obj_mm),
        str(obj_tu),
        str(obj_su),
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
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
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
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_int32),
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
    backend: str = "compliant_2d",
    station_count: int = 64,
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

    rc = lib.bike_metal_run_hybrid_sweep(
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


def run_metal_hybrid_tyre_backend_scaling_benchmark(
    model_xml_path: Path = DEFAULT_MODEL_XML,
    nworld: int = 2048,
    n_steps: int = 500,
    num_threads: int = 0,
    lib: ctypes.CDLL | None = None,
) -> dict[str, Any]:
    """Run Experiment B (tyre backend scaling across compliant_2d and distributed N=32/64/128)."""
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

        # Measure pure Metal GPU writers kernel timings (single CB vs per-step CB)
        single_cb_ms_c = ctypes.c_double(0.0)
        per_step_cb_ms_c = ctypes.c_double(0.0)
        rc = lib.bike_metal_benchmark_writers_only(
            int(nworld),
            int(n_steps),
            ctypes.c_float(dt_s),
            ctypes.cast(idx_buf, ctypes.c_void_p),
            ctypes.cast(model_buf, ctypes.c_void_p),
            ctypes.cast(desc_buf, ctypes.c_void_p),
            f32_ptr,
            i32_ptr,
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
            raise RuntimeError(f"bike_metal_run_hybrid_sweep ({tier_id}) failed with code {rc}")

        elapsed_ms = float(elapsed_ms_c.value)
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
            "writers_only_single_cb_us_per_world": round(writers_single_cb_us, 4),
            "writers_only_per_step_cb_us_per_world": round(writers_per_step_cb_us, 4),
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
        "experiment": "metal_hybrid_tyre_backend_scaling",
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
    parser.add_argument("--tier-steps", type=int, default=500)
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
    backend_report = run_metal_hybrid_tyre_backend_scaling_benchmark(
        model_xml_path=args.model_xml,
        nworld=args.nworld,
        n_steps=args.tier_steps,
        num_threads=args.num_threads,
        lib=lib,
    )
    backend_single_world = run_metal_hybrid_tyre_backend_scaling_benchmark(
        model_xml_path=args.model_xml,
        nworld=1,
        n_steps=args.tier_steps,
        num_threads=1,
        lib=lib,
    )

    payload = {
        "parameter_sweep": sweep_report,
        "tyre_backend_scaling": backend_report,
        "tyre_backend_scaling_single_world": backend_single_world,
    }
    rendered = json.dumps(payload, indent=2)
    print(rendered)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
