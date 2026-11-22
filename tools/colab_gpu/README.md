# Batched C++/CUDA & Apple Metal Force Writers and `mujoco_warp` Runner

This directory contains the Python build and benchmark tools for the GPU-resident bicycle and rider force writers.
These tools compile the single-source C++/CUDA/MSL kernels in `native/src/physics_core/` and `native/src/gpu/` and run them inside the SP1 `clean_split` `mujoco_warp` schedule inside a single CUDA or Metal Graph.

---

## 1. Purpose and Scope

The headers in `native/src/physics_core/` and `native/src/gpu/` implement all custom force writers in flat, allocation-free C++/CUDA/Metal code, with both `compliant_2d` (O(1) polyline contact) and `distributed_2d_reference` (`N <= 128` signed-profile stations) sharing the exact same single-source `tyre_gpu_step.hpp` step function as CPU `FP64` reference writers.
During a batched rollout (`nworld = 2048`), all state data, tyre models, terrain tables, and force arrays stay in GPU memory.

All runners disable FMA contraction and fast-math approximations across targets:
- CPU host runners (`cpu_batch_sweep.py`, `metal_hybrid_sweep.py`): `-ffp-contract=off` (no `-ffast-math`).
- NVIDIA CUDA (`build_cuda_writers.py`): `--fmad=false` (no `--use_fast_math`).
- Apple Metal MSL (`metal_batch_runner.mm`): `#pragma METAL fp contract(off)` + `MTLMathModeSafe` + `MTLMathFloatingPointFunctionsPrecise`.

---

## 2. C++/CUDA/MSL Kernel Architecture

All shared headers use `BIKE_HD` (`__host__ __device__ __forceinline__` on CUDA, `inline` on host/MSL) with `POD` structures and zero heap allocations.

| File | Description |
|---|---|
| `../physics_core/tyre_gpu_step.hpp` | Single-source tri-target (Host, CUDA, Metal MSL) step for both `compliant_2d` and `distributed_2d_reference` wheel backends (`gpu_step_both_wheels_tyre`) plus `GpuTerrainTableDesc` validation. |
| `../physics_core/{suspension,resistance,brake_hold,joint_envelope,ratchet}.hpp` | Single-source tri-target (Host, CUDA, Metal MSL) force-law headers for suspension (`eval_suspension`), rolling/aero resistance (`eval_resistance`), brake hold (`eval_brake_hold_wheel` / `apply_static_brake`), rider joint ROM envelope (`eval_rider_joint_envelope`), and dual ratchets (`ratchet_prepare_bound` / `fill_cassette_tendon_ranges`). |
| `../physics_core/{shifting,pedaling,assist,launch_plan,front_lift,rider_intent,efc_monitors,spindle_m2k2}.hpp` | Single-source tri-target (Host, CUDA, Metal MSL) autonomous drivetrain/rider controllers, EFC reaction monitors, and full 4-pass `SpindleM2K2` (`M2_K2`) Newton leg-press/upper-body controller. |
| `../physics_core/{tyre_upload,spindle_m2k2_upload}.{hpp,cpp}` | Host-side MuJoCo upload builders (`build_gpu_tyre_model`, `build_spindle_m2k2_params_from_json_files`). |
| `common.hpp` | Defines `BIKE_HD`, vector types (`GpuVec2`, `GpuVec3`), `GpuModelIndices` (`240` bytes for `nq = 27`), and root force projection (`project_planar_force_to_root`). |
| `terrain_tables.hpp` / `runner_tyre_setup.hpp` / `runner_spindle_setup.hpp` | Host-side terrain table packer (`GpuTerrainTableDesc` + flat `f32`/`i32` spans), per-surface `GpuTyreModel` resolution, and `SpindleM2K2Params` loader. |
| `batch_step.hpp` / `sp3_autonomous_step.hpp` / `sp4_closed_loop_step.hpp` | Defines `SP1–SP4` per-world configs/states and the tri-target closed-loop 4-pass `clean_split` step (`gpu_sp4_pre_solve_and_pass0_world`, `gpu_sp4_pass{1,2,3}_world`, `gpu_sp4_post_solve_world`). |
| `cuda_kernels.cu` | Defines the `__global__` CUDA kernels (`SP1–SP4`) and the `extern "C"` host/CUDA launch entry points returning status codes. |
| `cpu_batch_runner.cpp` | Implements the multi-threaded C++ CPU batch runner (`bike_cpu_run_batch`, `bike_cpu_run_batch_sweep`, `bike_cpu_build_tyre_and_terrain{,_for_surface}`, `bike_cpu_build_spindle_params`) linking directly against `libmujoco`. |
| `metal_batch_runner.mm` | Stitches `physics_core/*.hpp` + `gpu/*.hpp` into Apple Metal Shading Language (`MSL`) compute kernels (`1` thread/world, `MTLMathModeSafe`, `#pragma METAL fp contract(off)`) for **Scenario 3** (Hybrid Metal GPU Writers + CPU `mj_step` over Unified Memory), **Scenario 2** (`warp-metal` + `clean_split` `mujoco_warp`), and **SP4 Closed-Loop `4x solve`** (`bike_wp_sp4_*_kernel`). |

---

## 3. Shared Library Builder (`build_cuda_writers.py`)

`build_cuda_writers.py` compiles `native/src/gpu/cuda_kernels.cu` into a shared library and binds its `extern "C"` functions with `ctypes`.

### 3.1 Compilation Command

`compile_cuda_writers(gpu_src_dir, out_so_path)` runs the NVIDIA CUDA compiler:

```bash
nvcc -std=c++20 -O3 --fmad=false --shared -Xcompiler -fPIC \
  -Inative/src -Inative/src/gpu \
  native/src/gpu/cuda_kernels.cu -o /tmp/libbike_cuda_writers.so
```

### 3.2 Bound C API Functions

`configure_cuda_writers_cdll` verifies struct sizes (`GpuModelIndicesStruct`, `GpuTyreModelStruct`, `GpuTerrainTableDescStruct`) and configures `.argtypes` and `.restype = ctypes.c_int` for:

1. `bike_gpu_sizeof_world_config() -> int` / `bike_gpu_sizeof_world_state() -> int` / `bike_gpu_sizeof_model_indices() -> int` / `bike_gpu_sizeof_tyre_model() -> int` / `bike_gpu_sizeof_terrain_desc() -> int`
2. `bike_gpu_init_default_config(void *cfg) -> int`: Initializes one default `GpuWorldConfig` structure on the host.
3. `bike_gpu_build_sweep_configs(...) -> int`: Builds `nworld` heterogeneous `GpuWorldConfig` structures on the host from parameter scale arrays.
4. `bike_gpu_launch_init_states(...) -> int`: Launches `bike_gpu_init_states_kernel` on the given `cudaStream_t` to initialize wheel states and freehub bounds.
5. `bike_gpu_launch_writers(...) -> int`: Launches `bike_gpu_writers_kernel` on the given `cudaStream_t` using `d_tyre_model`, `d_terrain_desc`, `d_terrain_f32`, `d_terrain_i32`, `d_xmat`, `d_xanchor`, and `d_xaxis` to write `qfrc_applied`, `ctrl`, and `tendon_range`.

### 3.3 Model Index & Tyre Resolution

`resolve_gpu_model_indices(m)` populates `GpuModelIndicesStruct` (`240` bytes matching `common.hpp::GpuModelIndices`, supporting `nq = 27` including `drive_shaft_spin` and `crank_clutch`).
`build_tyre_and_terrain_buffers(host_lib, xml_path, backend, station_count)` builds `(tyre_model, terrain_desc, terrain_f32, terrain_i32)` via host C++ (`tyre_upload.cpp` + `terrain_tables.hpp`).

---

## 4. `clean_split` Graph Fusion and Benchmarks (`mjwarp_batch_sweep.py`)

`mjwarp_batch_sweep.py` fuses the C++/CUDA force writers with `mujoco_warp` (`SolverType.NEWTON`) inside one CUDA Graph using the SP1 `clean_split` schedule so `fwd_velocity` executes before `make_constraint` and kinematics (`xmat`, `xanchor`, `xaxis`) match step entry before writers run.

### 4.1 Zero-Copy Device Pointer Sharing & `clean_split` Schedule

1. Host builds `d_idx`, `d_configs`, `d_tyre_model`, `d_terrain_desc`, `d_terrain_f32`, and `d_terrain_i32`, copies them to GPU memory, and allocates `d_states`.
2. Inside `with wp.ScopedCapture() as capture:`, each step executes:
   - `fwd_kinematics` -> `smooth.crb` -> `smooth.tendon_armature` -> `smooth.factor_m` -> `collision_driver.collision` -> `smooth.transmission` -> `fwd_velocity`
   - `bike_gpu_launch_writers(...)` (reading `qpos`, `qvel`, `xpos`, `xmat`, `xanchor`, `xaxis` and writing `qfrc_applied`, `ctrl`, `tendon_range`)
   - `constraint.make_constraint` -> `fwd_actuation` -> `fwd_acceleration(factorize=False)` -> `solver.solve` -> `sensor.sensor_acc` -> `implicit`

### 4.2 Experiment A: Heterogeneous Parameter Sweep (`run_parameter_sweep`)

Experiment A runs `nworld = 2048` worlds in parallel with heterogeneous per-world scales (`fork_psi_scale` in `[0.80, 1.25]`, `rebound_scale` in `[0.70, 1.40]`, `shock_rate_scale` in `[0.85, 1.20]`, `tire_stiff_scale` in `[0.85, 1.20]`) and summarizes `peak_fork_travel_mm`, `peak_shock_stroke_mm`, and `forward_distance_root_x_m`.

### 4.3 Experiment B: Tyre Backend Scaling (`run_tyre_backend_scaling_benchmark`)

Experiment B benchmarks all unified `tyre_gpu_step.hpp` tiers at `nworld = 2048`:

| Tier ID | Backend (`TyreBackendKind`) | Stations per Wheel |
|---|---|---:|
| `compliant_2d` | `compliant_2d` (O(1) polyline) | `0` (polyline) |
| `distributed_32` | `distributed_2d_reference` | `32` |
| `distributed_64` | `distributed_2d_reference` | `64` |
| `distributed_128` | `distributed_2d_reference` | `128` |

---

## 5. How to Run on Google Colab (NVIDIA GPU)

1. Install `mujoco`, `warp-lang`, and `mujoco_warp` in the Colab runtime.
2. Compile the shared library and verify struct sizes:

```bash
python3 tools/colab_gpu/build_cuda_writers.py
```

3. Run both Experiment A (parameter sweep) and Experiment B (tyre backend scaling):

```bash
python3 tools/colab_gpu/mjwarp_batch_sweep.py \
  --nworld 2048 \
  --n-steps 1000 \
  --tier-steps 500 \
  --json-out /tmp/colab_gpu_sweep_results.json

# SP4 Closed-Loop 4x-Solve (M2_K2) Throughput & S1-S4 Acceptance
# Option A (local build tree with build/native-release/bin/bike-gpu-export):
python3 tools/colab_gpu/mjwarp_batch_sweep.py \
  --experiment sp4_all \
  --nworld 2048 \
  --n-steps 200 \
  --json-out /tmp/colab_sp4_all.json

# Option B (remote Linux Colab L4 VM without native CMake build / bike-gpu-export):
# 1. On the dev host with bike-gpu-export built, export the self-contained S1-S4 bundles once:
python3 tools/colab_gpu/mjwarp_batch_sweep.py --export-sp4-bundles /tmp/sp4_bundles
# 2. Copy /tmp/sp4_bundles to Colab and run with --sp4-bundle-dir:
python3 tools/colab_gpu/mjwarp_batch_sweep.py \
  --experiment sp4_all \
  --nworld 2048 \
  --n-steps 200 \
  --sp4-bundle-dir /tmp/sp4_bundles \
  --json-out /tmp/colab_sp4_all.json
```

---

## 6. How to Run Local Apple Silicon Benchmarks (`CPU`, `Scenario 3 Hybrid`, `Scenario 2 Full Metal`)

### 6.1 Pure Multi-Threaded C++ CPU Batch Sweep (`cpu_batch_sweep.py`)

```bash
uv run python tools/colab_gpu/cpu_batch_sweep.py \
  --nworld 2048 \
  --n-steps 1000 \
  --tier-steps 500
```

### 6.2 Scenario 3: Hybrid Apple Metal GPU Writers (`FP32`, `MTLMathModeSafe`) + CPU `mj_step` (`FP64`) (`metal_hybrid_sweep.py`)

```bash
uv run python tools/colab_gpu/metal_hybrid_sweep.py \
  --nworld 2048 \
  --n-steps 1000 \
  --tier-steps 500 \
  --json-out build/metal_hybrid_sweep_results.json
```

### 6.3 Scenario 2: Full Apple Metal GPU Pipeline (`warp-metal` + `clean_split` `mujoco_warp`, `mjwarp_metal_sweep.py`)

```bash
uv run --no-config --no-project --python .venv-metal/bin/python \
  tools/colab_gpu/mjwarp_metal_sweep.py \
  --nworld 2048 \
  --n-steps 1000 \
  --tier-steps 500 \
  --json-out build/mjwarp_metal_sweep_results.json

# Full SP4 Closed-Loop 4x-Solve Throughput (compliant_2d + distributed_2d_reference) + S1-S4 GPU Acceptance
# (supports --export-sp4-bundles DIR and --sp4-bundle-dir DIR just like mjwarp_batch_sweep.py)
.venv-metal/bin/python tools/colab_gpu/mjwarp_metal_sweep.py \
  --experiment sp4_all \
  --nworld 2048 \
  --n-steps 200 \
  --sp4-bundle-dir /tmp/sp4_bundles \
  --json-out build/mjwarp_metal_sp4_verification.json
```


