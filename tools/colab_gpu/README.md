# Batched C++/CUDA Force Writers and `mujoco_warp` Runner

This directory contains the Python build and benchmark tools for the GPU-resident bicycle and rider force writers.
These tools compile the C++/CUDA kernels in `native/src/gpu/` and fuse them with `mujoco_warp` (`mjwarp.step`) inside one CUDA Graph.

---

## 1. Purpose and Scope

On a single CPU core, custom force writers take more than 95% of the physics step time (`~1060 µs` out of `1100 µs`).
If the force writers stay on the CPU during a GPU batch run, memory copies between the CPU and the GPU remove the GPU speedup.

The headers in `native/src/gpu/` implement all custom force writers in flat, allocation-free C++/CUDA code.
During a batched rollout (`nworld = 2048`), all state data and force arrays stay in GPU memory (`VRAM`).

---

## 2. C++/CUDA Kernel Architecture (`native/src/gpu/`)

All headers in `native/src/gpu/` use the `BIKE_HD` (`__host__ __device__ __forceinline__`) macro.
They use plain-old-data (`POD`) structures and do not allocate heap memory.
Because of this design, the same code compiles with `clang++ -std=c++23` on macOS (for unit tests) and with `nvcc` on Linux/CUDA.

| File | Description |
|---|---|
| `common.hpp` | Defines `BIKE_HD`, vector types (`GpuVec2`, `GpuVec3`), `GpuModelIndices`, and root force projection (`project_planar_force_to_root`). |
| `suspension_kernel.hpp` | Computes the polytropic dual-chamber fork air spring, Charger3 hydraulic bottom-out (`HBO`) damper, rear coil spring, elastomer bumper, and top-out stops. |
| `tire_kernel.hpp` | Evaluates the distributed multi-rib tire contact grid over the 1D heightfield and updates tangential bristle deflections. |
| `drivetrain_resistance_kernel.hpp` | Computes aerodynamic drag, rolling resistance, bearing drag, dynamic braking, rider crank torque, and unilateral freehub ratcheting. |
| `rider_kernel.hpp` | Computes 6-DOF anatomical range-of-motion (`ROM`) soft-edge torques and posture PD plus pedaling actuator commands. |
| `batch_step.hpp` | Defines `GpuWorldConfig`, `GpuWorldState`, `gpu_build_sweep_configs`, and the unified per-world step function `gpu_step_world_writers`. |
| `cuda_kernels.cu` | Defines the `__global__` CUDA kernels (`bike_gpu_init_states_kernel` and `bike_gpu_writers_kernel`) and the `extern "C"` host functions. |
| `cpu_batch_runner.cpp` | Implements the multi-threaded C++ CPU batch runner (`bike_cpu_run_batch_sweep`) linking directly against `libmujoco`. |
| `metal_batch_runner.mm` | Compiles `native/src/gpu/*.hpp` as Apple Metal Shading Language (`MSL` / C++14) compute kernels (`1` thread/world and `32`-lane SIMD-group cooperative/world) for **Scenario 3** (Hybrid Metal GPU Writers + 16-thread CPU `mj_step` over zero-copy Unified Memory) and **Scenario 2** (`warp-metal` + `mujoco_warp` + MSL Writers fused inside one Metal Graph). |

---

## 3. Shared Library Builder (`build_cuda_writers.py`)

`build_cuda_writers.py` compiles `native/src/gpu/cuda_kernels.cu` into a shared library and binds its `extern "C"` functions with `ctypes`.

### 3.1 Compilation Command

`compile_cuda_writers(gpu_src_dir, out_so_path)` runs the NVIDIA CUDA compiler:

```bash
nvcc -std=c++20 -O3 --use_fast_math --shared -Xcompiler -fPIC \
  native/src/gpu/cuda_kernels.cu -o /tmp/libbike_cuda_writers.so
```

### 3.2 Bound C API Functions

`compile_cuda_writers` configures `.argtypes` and `.restype` on the `ctypes.CDLL` handle for seven functions:

1. `bike_gpu_sizeof_world_config() -> int`: Returns `sizeof(GpuWorldConfig)` in bytes.
2. `bike_gpu_sizeof_world_state() -> int`: Returns `sizeof(GpuWorldState)` in bytes.
3. `bike_gpu_sizeof_model_indices() -> int`: Returns `sizeof(GpuModelIndices)` in bytes.
4. `bike_gpu_init_default_config(void *cfg)`: Initializes one default `GpuWorldConfig` structure on the host.
5. `bike_gpu_build_sweep_configs(...)`: Builds `nworld` heterogeneous `GpuWorldConfig` structures on the host from parameter scale arrays and tire grid dimensions.
6. `bike_gpu_launch_init_states(...)`: Launches `bike_gpu_init_states_kernel` on the given `cudaStream_t` to zero tire bristles and set initial freehub bounds.
7. `bike_gpu_launch_writers(...)`: Launches `bike_gpu_writers_kernel` on the given `cudaStream_t` to write `qfrc_applied`, `ctrl`, and `tendon_range` for all `nworld` worlds.

### 3.3 Model Index Resolution

`resolve_gpu_model_indices(m)` reads body IDs, joint `qpos` addresses, joint `dof` addresses, and actuator IDs from a `mujoco.MjModel`.
It returns the packed bytes of `GpuModelIndicesStruct` (`164 bytes`), which match `common.hpp::GpuModelIndices`.

---

## 4. CUDA Graph Fusion and Benchmarks (`mjwarp_batch_sweep.py`)

`mjwarp_batch_sweep.py` fuses the C++/CUDA force writers with `mujoco_warp` inside one CUDA Graph.

### 4.1 Zero-Copy Device Pointer Sharing

1. The script allocates GPU byte buffers (`wp.zeros(..., dtype=wp.uint8)`) for `d_idx`, `d_configs`, and `d_states`.
2. It copies host configuration bytes to the GPU with `wp.copy`.
3. It extracts raw device pointers from Warp arrays through the `.ptr` attribute (`d_b.qpos.ptr`, `d_b.qvel.ptr`, `d_b.xpos.ptr`, `d_b.qfrc_applied.ptr`, `d_b.ctrl.ptr`, and `m_wp.tendon_range.ptr`).
4. It gets the raw CUDA stream handle with `wp.get_stream().cuda_stream`.
5. Inside a `with wp.ScopedCapture() as capture:` block, it calls `lib.bike_gpu_launch_writers(...)` on that stream and then calls `mjwarp.step(m_wp, d_b)`.
6. CUDA captures both the C++/CUDA writer kernel and the `mjwarp.step` solver kernels into one `capture.graph`.

### 4.2 Experiment A: Heterogeneous Parameter Sweep (`run_parameter_sweep`)

Experiment A runs `nworld = 2048` worlds in parallel with different physical parameters in each world:
- `fork_psi_scale` in `[0.80, 1.25]`
- `rebound_scale` in `[0.70, 1.40]`
- `shock_rate_scale` in `[0.85, 1.20]`
- `tire_stiff_scale` in `[0.85, 1.20]`

The benchmark measures wall-clock time, `µs/step/world`, and total `steps/s`.
It also reports the distribution (`min`, `p10`, `median`, `mean`, `p90`, `max`) across all 2048 worlds for:
- Peak fork travel (`mm`)
- Peak rear shock stroke (`mm`)
- Forward distance `root_x` (`m`)

### 4.3 Experiment B: 3D Tire Ray-Grid Scaling (`run_3d_tire_ray_scaling_benchmark`)

Experiment B measures the speed of the fused C++/CUDA writers and `mjwarp.step` at `nworld = 2048` across three tire contact resolutions:

| Tier Name | Stations (`num_stations`) | Lateral Ribs (`num_lateral_ribs`) | Rays per Wheel | Rays per World (2 Wheels) | Total Rays per Step (`nworld = 2048`) |
|---|---:|---:|---:|---:|---:|
| **2D Planar Tier** | 32 | 1 | 32 | 64 | 131,072 |
| **3D Fast Tier** | 45 | 10 | 450 | 900 | 1,843,200 |
| **3D Detailed Tier** | 110 | 35 | 3,850 | 7,700 | 15,769,600 |

---

## 5. How to Run on Google Colab (NVIDIA GPU)

1. Install `mujoco`, `warp-lang`, and `mujoco_warp` in the Colab runtime.
2. Compile the shared library and verify struct sizes:

```bash
python3 tools/colab_gpu/build_cuda_writers.py
```

3. Run both Experiment A (parameter sweep) and Experiment B (3D tire ray scaling):

```bash
python3 tools/colab_gpu/mjwarp_batch_sweep.py \
  --nworld 2048 \
  --n-steps 1000 \
  --ray-steps 500 \
  --json-out /tmp/colab_gpu_sweep_results.json
```

---

## 6. How to Run Local Apple Silicon Benchmarks (`CPU`, `Scenario 3 Hybrid Metal+CPU`, and `Scenario 2 Full Metal GPU`)

### 6.1 Pure Multi-Threaded C++ CPU Batch Sweep (`cpu_batch_sweep.py`)

To run the `nworld = 2048` parameter sweep and 3D tire ray-grid scaling benchmark across all local CPU threads (`16` threads on `Apple M4 Max`, `FP32` writers + `FP64` `mj_step`):

```bash
uv run python tools/colab_gpu/cpu_batch_sweep.py \
  --nworld 2048 \
  --n-steps 1000 \
  --ray-steps 500
```

### 6.2 Scenario 3: Hybrid Apple Metal GPU Writers (`FP32`) + 16-Thread CPU `mj_step` (`FP64`) Over Unified Memory (`metal_hybrid_sweep.py`)

`metal_batch_runner.mm` and `metal_hybrid_sweep.py` allocate zero-copy `MTLResourceStorageModeShared` Unified Memory buffers and overlap our custom bicycle force writers on the 40-core `Apple M4 Max` Metal GPU (`FP32`, using 32-lane SIMD-group cooperative reduction for 3D tires) with 16-thread C++ CPU `mj_step` (`FP64`):

```bash
uv run python tools/colab_gpu/metal_hybrid_sweep.py \
  --nworld 2048 \
  --n-steps 1000 \
  --ray-steps 500 \
  --json-out build/metal_hybrid_sweep_results.json
```

### 6.3 Scenario 2: Full Apple Metal GPU Pipeline (`warp-metal` + `mujoco_warp` + MSL Writers in One Metal Graph, `mjwarp_metal_sweep.py`)

`mjwarp_metal_sweep.py` uses the community `warp-metal` (`1.17.0.3` overlaying `warp-lang==1.17.0`) and `mujoco-warp==3.11.0` inside `.venv-metal` to compile and fuse both our custom MSL force writers (`bike_wp_writers_metal_kernel` / `bike_wp_writers_simd32_kernel`) and all ~30 `mjwarp.step` solver kernels inside a single `wp.ScopedCapture()` Metal Graph on `"metal:0"` (`100%` GPU execution in `FP32`, `0%` CPU load):

```bash
uv run --no-config --no-project --python .venv-metal/bin/python \
  tools/colab_gpu/mjwarp_metal_sweep.py \
  --nworld 2048 \
  --n-steps 1000 \
  --ray-steps 500 \
  --json-out build/mjwarp_metal_sweep_results.json
```

---

## 7. Measured Five-Way Benchmark Matrix: Apple M4 Max (`Pure CPU`, `Scenario 3 Hybrid Metal+CPU`, `Scenario 2 Full Metal GPU`) vs. Google Colab (`AMD EPYC 7B12` CPU & `NVIDIA L4` GPU)

For `rough_uphill_savage.toml` (`model.xml`), the integration timestep is `dt = 0.0005 s` (`500 µs`).
The Real-Time Factor (`RTF`) is `500 µs / t_step_us`.

| Workload and Tire Tier | Mac Pure CPU (`Apple M4 Max`, C++ `FP64` `mj_step`) | Mac Scenario 3: Hybrid (`Metal GPU` Writers `FP32` + `16T CPU` `mj_step` `FP64`) | Mac Scenario 2: Full Metal GPU (`warp-metal` + `mjwarp` + `MSL` Graph `FP32`) | Colab CPU (`AMD EPYC 7B12`, 1 Core C++ `FP64`) | Colab GPU (`NVIDIA L4`, CUDA Graph `FP32`) | Fastest Target and Speedup |
|---|---|---|---|---|---|---|
| **Single World (`nworld = 1`) — Solver-Only** | **`22.03 µs/step` (`RTF = 22.70x`, 1c)** | `—` | `—` | `47.73 µs/step` (`RTF = 10.48x`, 1c) | `462.66 µs/step` (`RTF = 1.08x`) | **M4 Max 1c CPU (`2.17x` vs. EPYC, `21.0x` vs. L4)** |
| **Single World (`nworld = 1`) — 2D Planar (`64` rays)** | **`20.47 – 22.41 µs/step` (`RTF = 22.31x – 24.42x`, 1c)** | `146.06 µs/step` (`RTF = 3.42x`) | `1,783.33 µs/step` (`RTF = 0.28x`) | `49.18 – 52.77 µs/step` (`RTF = 9.48x – 10.17x`, 1c) | `692.32 µs/step` (`RTF = 0.72x`) | **M4 Max 1c CPU (`2.38x` vs. EPYC, `32x` vs. L4, `83x` vs. Metal Graph)** |
| **Single World (`nworld = 1`) — 3D Fast (`900` rays)** | **`24.39 – 25.96 µs/step` (`RTF = 19.26x – 20.50x`, 1c)** | `153.68 µs/step` (`RTF = 3.25x`) | `1,815.77 µs/step` (`RTF = 0.28x`) | `60.36 – 62.06 µs/step` (`RTF = 8.06x – 8.28x`, 1c) | `~695 µs/step` (`RTF ≈ 0.72x`) | **M4 Max 1c CPU (`2.43x` vs. EPYC, `27.6x` vs. L4)** |
| **Single World (`nworld = 1`) — 3D Detailed (`7,700` rays)** | **`52.22 – 52.74 µs/step` (`RTF = 9.48x – 9.58x`, 1c)** | `214.51 µs/step` (`RTF = 2.33x`) | `1,919.99 µs/step` (`RTF = 0.26x`) | `143.59 – 146.58 µs/step` (`RTF = 3.41x – 3.48x`, 1c) | `~710 µs/step` (`RTF ≈ 0.70x`) | **M4 Max 1c CPU (`2.76x` vs. EPYC, `4.08x` vs. Hybrid, `13.5x` vs. L4)** |
| **Experiment A Sweep (`nworld = 2048`, `1000` steps, `64` rays)** | **`1.332 µs/step/world` (`750.9k steps/s`, `RTF = 375.5x`, 16T)** | **`1.864 µs/step/world` (`536.6k steps/s`, `RTF = 268.3x`)** | **`1.801 µs/step/world` (`555.1k steps/s`, `RTF = 277.6x`, 0% CPU)** | `49.18 – 52.77 µs/step/world` (`RTF = 9.48x – 10.17x`, 1c) | **`1.426 µs/step/world` (`701.1k steps/s`, `RTF = 350.5x`)** | **M4 Max 16T CPU (`375.5x` in FP64) ≈ L4 GPU (`350.5x`) > M4 Max Metal GPU (`277.6x`)** |
| **Experiment B (`nworld = 2048`) — 2D Planar (`64` rays)** | **`1.453 µs/step/world` (`688.0k steps/s`, `RTF = 344.0x`, 16T)** | `2.008 µs/step/world` (`497.9k steps/s`, `RTF = 248.9x`; writers `0.029 µs`) | **`1.780 µs/step/world` (`561.7k steps/s`, `RTF = 280.9x`)** | `49.18 – 52.77 µs/step/world` (`RTF = 9.48x – 10.17x`, 1c) | **`1.400 µs/step/world` (`714.2k steps/s`, `RTF = 357.1x`)** | **L4 GPU (`357.1x`) ≈ M4 Max 16T CPU (`344.0x`) > M4 Max Metal GPU (`280.9x`)** |
| **Experiment B (`nworld = 2048`) — 3D Fast (`900` rays)** | **`1.719 µs/step/world` (`581.6k steps/s`, `RTF = 290.8x`, 16T)** | `2.170 µs/step/world` (`460.9k steps/s`, `RTF = 230.4x`; writers `0.094 µs`) | **`1.889 µs/step/world` (`529.5k steps/s`, `RTF = 264.7x`)** | `60.36 – 62.06 µs/step/world` (`RTF = 8.06x – 8.28x`, 1c) | **`1.445 µs/step/world` (`691.9k steps/s`, `RTF = 346.0x`)** | **L4 GPU (`346.0x`) > M4 Max 16T CPU (`290.8x`) > M4 Max Metal GPU (`264.7x`)** |
| **Experiment B (`nworld = 2048`) — 3D Detailed (`7,700` rays)** | `4.343 µs/step/world` (`230.3k steps/s`, `1.77 Grays/s`, `RTF = 115.1x`, 16T) | **`2.148 µs/step/world` (`465.5k steps/s`, `3.58 Grays/s`, `RTF = 232.8x`; writers `0.191 µs` = `40.25 Grays/s`!)** | **`2.075 µs/step/world` (`481.9k steps/s`, `3.71 Grays/s`, `RTF = 240.9x`)** | `143.59 – 146.58 µs/step/world` (`RTF = 3.41x – 3.48x`, 1c) | **`1.885 µs/step/world` (`530.4k steps/s`, `4.08 Grays/s`, `RTF = 265.2x`)** | **L4 GPU (`265.2x`) ≈ M4 Max Metal Scenario 2 (`240.9x`, `2.09x` vs. 16T CPU) ≈ M4 Max Hybrid Scenario 3 (`232.8x` in FP64, `2.02x` vs. 16T CPU)** |


