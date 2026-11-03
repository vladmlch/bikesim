# GPU Shared Physics — Stage 1: Toolkit and Tyre Core — Implementation Plan

> **Imported into main3 (2026-11-03), not started.** Written in the colab-gpu
> worktree against `research/colab-gpu` `b25120af`, before main3 merged
> passive-joint-damping and colab-gpu, ported kinetic-damping and removed the
> battery energy model (`54ee44f8`). Re-verify every file:line fact against
> main3 before executing. Known differences:
> - `tools/fp32_probe/artifacts/common/model.xml` (with `captured_bootstrap.json`
>   and `hfield_data.bin`) is tracked in main3 (`70f72783`).
> - `docs/superpowers/audits/` does not exist in main3 (`072f0002`); research
>   records live in `docs/research/`.
> - Recording channels `battery_energy_j`, `battery_empty`, `energy_limited`,
>   `electrical_power_w` and energy terms `electrical_work_j`,
>   `electrical_residual_j` no longer exist.
> - Open question Q23 from the planning session (record every per-step model
>   mutation: brake joint friction, motor tendon limit, solver warmstart) was
>   never answered.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

## Review amendments (2026-11-03, override the body where they conflict)

- **A1.** No test may depend on an untracked file. `tools/fp32_probe/artifacts/common/model.xml` is untracked, so the Task 11 batch-runner tests use the tracked `tests/fixtures/golden/darwin-arm64/native_distributed_tire/episodes/stand40/model.mjb` instead, the same model Task 8 already uses. If the CPU batch runner cannot load an `.mjb`, or `build_tyre_upload` rejects stand40, stop (S9) and report the exception text. Do not commit or reference the fp32_probe artifacts.
- **A2.** This plan runs only after Stage 0 has landed. G1 needs `golden_episode_digest` and `bike-recording-digest` from `2026-11-03-gpu-shared-s0-goldens.md`.
- **A3.** Physics stage: the final gate is `bash tools/run_tests.sh native` **and** `bash tools/run_tests.sh full` (master plan, Global Constraints).

**Goal:** The distributed tyre law (`distributed_2d_reference`) runs from one header-only source in `native/src/physics_core/`. The CPU instantiates it as `<double, CpuExactToolkit>` and its output stays bit-identical. The GPU instantiates it as `<float, GpuToolkit>`, with 128 rotating stations, the exact dof-chain Jacobian and the CPU-built terrain polyline. `gpu/tire_kernel.hpp` and the lateral ribs are deleted.

**Architecture:**
- Each per-step tyre law is moved verbatim from `writers/`, `tyre/` and `contact/` into a template over `<class Real, class TK>`. `TK` is a toolkit of primitives (dot, gemv, ring lift, interp segment, div, sqrt, abs, exp, isfinite, isnan).
- The CPU wrappers keep every diagnostic, ledger, snapshot and exception. They map the core's `Status` to the exception thrown today.
- The GPU gets a thin adapter, `gpu/tyre_step.hpp`. It is fed engine kinematics (`xpos`, `xmat`, `xanchor`, `xaxis`) and a host-built upload (`gpu/tyre_upload.cpp`).
- Metal compiles everything in a measured safe math mode (Task 1). One test requires Metal output to be bitwise equal to the host-`float` instantiation.

**Tech Stack:**
- C++23 host, CMake presets `release` → `build/native-release`.
- Accelerate CBLAS on macOS; NumPy-bundled OpenBLAS on Linux aarch64.
- MSL compiled at runtime by `newLibraryWithSource` in `native/src/gpu/metal_batch_runner.mm`.
- CUDA via `cuda_kernels.cu`: compiled for host only here. nvcc is Stage 8.
- Python via `uv` for `tools/colab_gpu/`.

**Spec:** `docs/superpowers/plans/2026-11-03-gpu-shared-physics-master.md` (decisions D1–D22, risks R1–R5). Also read `AGENTS.md`, `docs/agents/native-engineering.md` and `docs/TESTING.md`.

## Global Constraints

Copied from the master plan:
- Python only via `uv` (`uv run ...`). GPU Python uses `.venv-metal` (`uv run --python .venv-metal/bin/python ...`).
- Read `docs/agents/native-engineering.md` before any native change.
- The stage completion gate is `bash tools/run_tests.sh native`, exit 0, run **once** at the end of the stage (Task 14). The inner loop uses targeted CTest nodes.
- **D2:** CPU bit identity is checked with the Stage 0 end-to-end digest (`ctest --test-dir build/native-release -R golden_episode_digest --output-on-failure`, label `slow`). Run it after **every** task that touches `native/src/writers/`, `native/src/tyre/`, `native/src/contact/`, `native/src/drivetrain/` or `native/src/physics_core/`. Gate G1 below includes it.
- No golden fixture under `tests/fixtures/golden/` is re-captured or edited. New tests only *read* fixtures.
- Never forge sealed recording containers.
- `physics_core/` headers:
  - compile as MSL, CUDA and host C++;
  - no `std::` containers, no allocation, no exceptions, no virtuals, no recursion;
  - fixed-size arrays sized by `constexpr` capacities;
  - address-space macros (`BIKE_HD`, `BIKE_THREAD`, `BIKE_DEVICE`, `BIKE_CONSTANT`, plus the new `BIKE_CORE_TABLE`).
- `-ffp-contract=off` stays on for CPU. The core never depends on FMA contraction; explicit fma exists only inside a toolkit primitive (`CpuExactToolkit::interp_segment`, through `numeric::np_interp_segment`).
- T63: `tools/colab_gpu/**` keeps its recorded Python exemption.
- D3: `compliant_2d` and `pneumatic` stay CPU-only. Where they share an extracted helper (`contactlaw::brush_step`, `tirecommon::surface_at/surface_mu/effective_friction`, `biketyre::np_interp`), the helper becomes a wrapper over the core with **unchanged bits and exceptions**. Their CTest nodes join the gate (G2).
- Stage-specific rules:
  - Every literal inside `physics_core/` is either an integer converted with `Real(k)` or a `Lit<Real>::name()` constant (see `scalar.hpp`). Reason: host-`float` and MSL must round decimal literals identically, and MSL has no `double`.
  - Every division in `physics_core/` goes through `TK::div` and every square root through `TK::sqrt`. `CpuExactToolkit` implements them as the plain IEEE operations, so the CPU bits do not change. `GpuToolkit` can be swapped to a correctly rounded software routine if Task 1 finds Metal division or sqrt are not correctly rounded.
  - Commits follow the repo style (`feat(scope): …`, `refactor(scope): …`, `test(scope): …`) and end with the trailer line `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

---

## Decisions this plan takes (deviations from the brief, each justified)

| # | Decision | Why |
|---|---|---|
| P1 | The core takes **both** halves of the centre Jacobian (`jp` and `jr`), not only the translational 3×nv block D10 mentions. | The per-station cross product reads `jr_` (`distributed_tire.cpp:673-681`). Without it the CPU bits cannot be reproduced. |
| P2 | GPU wheel rotation comes from the engine's `xmat`, and the GPU Jacobian from the engine's `xanchor`/`xaxis` along the host-resolved dof chain. It is **not** recomputed from `qpos`. | (a) The CPU reads `d->xmat` (`distributed_tire.cpp:440-450`), so this is the same input. (b) Recomputing from `qpos` needs device `sin`/`cos`, whose Metal and host-libm results are not guaranteed to agree bitwise. That would break the D19 Metal==host-float test. (c) Both MuJoCo CPU and mujoco_warp already provide `xmat`, `xanchor` and `xaxis` (`.venv-metal/.../mujoco_warp/_src/types.py:2239-2243`). The chain formula is the exact hinge/slide Jacobian. Task 8 proves it ≤ 1e-12 against `mj_jac` (R3). |
| P3 | The core splits the per-wheel step into `classify_stations` (ring lift, heights, active ids) and `wheel_contacts` (station loop). | The CPU throws a height-domain error *before* `point_jacobian` (`distributed_tire.cpp:631` precedes `:634`). One fused core call would reorder exceptions. `classify_stations` is also what `stored_energy` needs (`:482`). |
| P4 | Per-station diagnostics reach the CPU wrapper through an `Io` template callback (`old_state(sid)` and `station(result)`). | The CPU interleaves its ledgers, patch validation and state staging with the station loop. Doing that inside the loop keeps every accumulation and exception in its original order. `make_patch` may throw from inside the callback; the core itself contains no `throw`. |
| P5 | `GpuToolkit::exp` is a portable Cephes-style `expf` built from `+ - * floor ldexp`. It is not `metal::exp` or libm `expf`. | `surface_mu` calls `exp` on every active station (`tire_common.cpp:15-16`, `surface_mode = "track"`). Two independent transcendental implementations cannot be relied on to agree bitwise. A polynomial over correctly rounded basic ops agrees by construction, once Task 1 confirms those basic ops agree. |
| P6 | The Metal `simd32` kernels are deleted. So are the never-looked-up `bike_metal_step_writers*_kernel` MSL entry points and `bike_metal_benchmark_writers_only`. | The `simd32` path reduces with `simd_sum` (`metal_batch_runner.mm:180-192`) in a different order from the host, which breaks D19. Its lane-parallel ray loop exists only for the deleted rib grid. The host looks up only five functions (`:887-891`). The benchmark fabricates `xpos` and ignores caller heights (`:1310`), so it cannot drive the exact Jacobian. D15 already schedules re-measurement. |
| P7 | GPU tyre geometry (ring, weights, radius, chain, surface map) lives in one per-model device buffer, `GpuTyreModel`. The terrain lives in a second packed float buffer. Tunable per-world tyre parameters (density, tangent stiffness, μ, relaxation length) stay in `GpuWorldConfig`. | Keeps the existing per-world `tire_stiff_scale` sweep. Replaces the four heightfield arguments with two pointers. Adds no field to `GpuModelIndices`, so the `static_assert(... == 228)` and the Python mirror stay as they are. `njnt` travels inside `GpuTyreModel`. |
| P8 | Stage 1 ships the upload builder (`gpu/tyre_upload.cpp`) and the C++ runner ABI. It does **not** ship a CLI that writes an upload file for the Python sweeps. | A new installed binary touches packaging. The Stage 2 bundle exporter (master plan "Model bundle export") is the natural owner. The Python mirrors are updated to the new ABI and take a `--gpu-tyre` side-car path. They fail closed until Stage 2 provides it. The report lists this. |
| P9 | `-ffp-contract=off` reaches every GPU test through `bike_apply_native_policy`. The root graph already applies it (`CMakeLists.txt:680-689`). No new flag is needed in the gate graph. | Verified. The legacy `native/CMakeLists.txt:564-585` `bike_add_gpu_test` does not apply it. That graph only builds with Python bindings (`CMakeLists.txt:756-764`), so this plan leaves it untouched apart from deleting the `test_gpu_tire` line. |

---

## Verified facts (HEAD `b25120af8f777e496c7d96155c0557380a589b25`, read 2026-11-03)

**CPU tyre**
- `native/src/writers/distributed_tire.cpp`:
  - `point_jacobian` :59-79
  - `effective_radius` :135-155 (`numeric::PythonSum`, `dot3` = `blas::ddot`)
  - local ring built with `numeric::np_cos/np_sin` :228-238
  - `station_points` :432-467:
    - `blas::dgemm(row_major, no, yes, n, 3, 3, …)` :446-451, then `+= center` :454-456
    - `height` over all stations :459-460
    - active ids :462-465
  - `stored_energy` :473-511
  - `accumulate_validated` :587-899:
    - centre `geom_xpos` :628-630
    - `station_points` :631, `point_jacobian` :634, centre velocity `dgemv(no)` :636-638
    - loop :650-772: cross product :673-681, `dgemv(no)` :684-687, rate :690-693, transport :709-719, `effective_friction` :723-727, `brush_step` :732-736, shear energy `pyfloat::pow` :738, projection `dgemv(yes)` + `+=` :743-752, `make_patch` :767-769
    - detached loss :776-781, reductions :785-801, `distinct` :806-819, `separate` :823-833, snapshot :838-854, diagnostics :858-878, commit :888-898
- `native/src/writers/distributed_tire.hpp`: `StationState` :44-48; private section :208-268.
- `native/src/tyre/station.cpp`: `station_angles` :25-50 (16..512, weight `(2π)/count`); `station_density` :52-73; `station_response` :77-95.
- `native/src/tyre/distance.cpp`:
  - `search_right/left` :35-44, `clip01` :48
  - constructor :52-103, `height` :105-112, `candidate` :114-143, `query` :145-258
  - literals `2e-15` :187, `1e-10` :209/:252, `1e-12` :229/:241-243
- `native/src/tyre/distance.hpp`: class :48-85.
- `native/src/tyre/profile.cpp`: `np_interp` :98-140 (calls `numeric::np_interp_segment` :138); 7 external callers of `biketyre::np_interp` (`grep` 2026-11-03: `runtime/research_metrics.cpp:92-93`, `runtime/control.cpp:412`, `legacy/ride.cpp:377`, `initialization/initial_pose.cpp:56`, `tyre/distance.cpp:111`, `binding.cpp:1762`).
- `native/src/numeric_numpy.hpp`: `np_interp_eval` :314-322 (`std::fma` iff `BIKE_NUMPY_INTERP_CONTRACTED`); `np_interp_segment` :329-340. The define is directory-wide (`native/cmake/BikeNumerics.cmake:433-434`).
- `native/src/contact/laws.hpp`: `finite_fail`/`finite_result` :13-22; `brush_step` :43-74. Callers: `writers/tire_common.hpp:105-116` and `writers/rider_contacts.cpp:549`.
- `native/src/writers/tire_common.cpp`: `surface_mu` :14-18 (`std::exp`), `surface_at` :22-30, `effective_friction` :34-42. Callers: `writers/tire.cpp:750,753`, `writers/distributed_tire.cpp:724,733`, `writers/pneumatic_tire.cpp:766,1032-1033,1044`.

**CPU primitives**
- `native/src/cblas_abi.hpp`: `blas::ddot` returns numpy's `0. +` seeded value :90-91; `dgemv` :103-105; `dgemm` :107-110.
- `numeric_sum.hpp`: `PythonSum` :48-62, `numpy_sum` :108-111.
- `numeric_sincos.hpp`: volatile `np_sin/np_cos` :69-89.
- `writers/pyfloat.hpp`: `pyfloat::pow` through a volatile function pointer :16-19.
- libc++ `std::max` is `comp(a,b) ? b : a` and `std::min` is `comp(b,a) ? b : a` (`MacOSX.sdk/usr/include/c++/v1/__algorithm/max.h:30`, `min.h:30`).
- `-ffp-contract=off` is a required policy flag (`native/cmake/BikeCompilerPolicy.cmake:446-456`), applied by `bike_apply_native_policy` (:1337-1344).

**Config**
- The `env_request/physics.toml` fixture matches `examples/research/viewer_physics_distributed.toml` except two JSON paths (`diff`, lines 115 and 118).
- The resolved tyre wire (`tests/fixtures/golden/darwin-arm64/native_distributed_tire/env/wire.json`, `.tire`):
  - `station_count` 128;
  - density knots `[0, .004, .012]`, stiffness `[300000, 0, 0]`, damping 2000;
  - front/rear `mu` 1.1, `tangent_k_n_m` 20000, `relaxation_length_m` 0.2;
  - `surface_mode` `"track"`: default `hardpack` (0.8/0.6/4.5) plus 3 `scree` sections (0.45/0.38/4.5);
  - `distinct_normal_deg` 20.
- Defaults: `native/src/configuration/physical_config.hpp:54-84,184-189`.
- `nativecfg::WriterConfig::tire` is `std::optional<TireConfig>` (`native/src/writer_config.hpp:49-58`). The test helper `tyre_reference::load_config(dir)` decodes `wire.json` (`native/tests/reference/tyre_reference_support.hpp:257-261`). `platform_fixture(bundle, name)` is at :237-241.
- `HingeDensity` validation has no knot-count limit (`native/src/config_validation.hpp:288-298`).
- The `stand40` fixture model (`tests/fixtures/golden/<platform>/native_distributed_tire/episodes/stand40/model.mjb`, tracked) contains these names:
  - `root_x`, `root_z`, `root_pitch`, `fork_travel`, `main_pivot`, `horst_pivot`;
  - `front_wheel_spin`, `rear_wheel_spin`;
  - `geom_front_contact`, `steer`, `chainstay`, `seatstay` (`strings`).

**MuJoCo**
- `mjJNT_SLIDE = 2`, `mjJNT_HINGE = 3` (`.deps/mujoco-3.12.0/include/mujoco/mjtype.h:92-95`).
- `mj_jac(m, d, jacp, jacr, point, body)` (`mujoco.h:566-567`).

**GPU**
- `native/src/gpu/common.hpp`:
  - macros :10-40, shared-code pragmas :42-50
  - `kMaxNq/Nv/Nu/Nbody = 32/32/16/32` :57-60, `kMaxTreadStations = 64` :61, `kMaxHingeKnots = 8` :62
  - `static_assert(sizeof(GpuModelIndices) == 228)` :145
- `native/src/gpu/batch_step.hpp`:
  - `GpuWorldConfig` :17-27, `GpuWorldState` :29-33
  - `gpu_build_sweep_configs` :44-109, `gpu_init_world_state` :112-123, `gpu_step_world_writers` :126-221
  - tyre outputs feed drivetrain/resistance :188-206
- `native/src/gpu/metal_batch_runner.mm`:
  - `build_msl_shader_source` :101-821; header list :103-110; `strip_includes_and_pragmas` :84-99
  - `newLibraryWithSource … options:nil` :878-880
  - function lookups :887-891
  - `use_simd32` selection :1166, :1318, :1609
  - `BikeWpWritersArgs` :576-594
- Python loads the same MSL through warp-metal's `wp_metal_load_library` (`tools/colab_gpu/mjwarp_metal_sweep.py:146-160`). warp-metal's `libwarp.dylib` references `setMathMode:` and `setMathFloatingPointFunctions:` (`strings`), but the values are not visible.
- Test registration:
  - root graph `bike_add_gpu_unit_test` `CMakeLists.txt:680-689`, `bike_test_gpu_tire` :696-697
  - Metal runner object :720-736
  - `native_distance_tests`/`native_station_tests` :574-596
  - `bike_core` sources :113-171 (`tyre/station.cpp` at :129)
- CTest names:
  - `reference.native_distributed_tire`, `reference.distributed_tire_reference`, `reference.native_distributed_stands` (`native/tests/reference/Phase3Ports.cmake:116-129`; prefix `reference.` from `native/tests/reference/CMakeLists.txt:24,34`);
  - `native_distance_tests`, `native_station_tests` (`CMakeLists.txt:595-596`; legacy twin `native/CMakeLists.txt:431-447`).
- Suppression register row for the GPU kernels: `docs/TESTING.md:1062`.

**Metal SDK** (`/Library/Developer/CommandLineTools/SDKs/MacOSX27.0.sdk`, identical to `Xcode.app/.../MacOSX.sdk` for `MTLLibrary.h:250-432`):
- `MTLMathMode { Safe = 0, Relaxed = 1, Fast = 2 }` :269-274. Safe "Disables unsafe floating-point optimizations".
- `MTLMathFloatingPointFunctions { Fast = 0, Precise = 1 }` :286-290. The default is Fast.
- `mathMode` (macOS 15+, "Default depends on the language standard version") :332.
- `mathFloatingPointFunctions` (macOS 15+) :338.
- `fastMathEnabled` is deprecated since macOS 15 and "defaults to YES" :323-326.
- `preserveInvariance` "compile shaders to preserve invariance", default false :383-386. This is a vertex-invariance switch, not an FP-contraction switch.
- `floatingPointConversionRoundingMode` (macOS 27) :429.
- Host OS is macOS 27.0.1. No deployment target is set in CMake (`grep`), so the macOS 15 properties are usable without `@available`.

## Facts that could NOT be verified locally

- **MSL FP-contraction control.** The offline Metal toolchain is not installed (`xcrun metal` reports "missing Metal Toolchain"). `metal_stdlib` and the MSL specification are not on disk. So these are unverified until Task 1 measures them:
  - whether `#pragma METAL fp contract(off)` or `#pragma clang fp contract(off)` exist or are honoured;
  - whether `MTLMathModeSafe` disables contraction;
  - whether Metal fp32 `/` and `sqrt` are correctly rounded;
  - whether denormals are flushed.
- **warp-metal's compile options** for `wp_metal_load_library`. Only the selector names were seen.
- **`nv`/`nq`/`nbody`/`njnt` of the `stand40` model.** No code was run. Task 8 prints them, and Step 8.2 handles `nv > kMaxNv`.
- **The Stage 0 artifacts** (`golden_episode_digest`, `bike-recording-digest`) are planned in parallel and do not exist at HEAD. This plan assumes the brief's names.
- **CUDA device compilation** of the changed `cuda_kernels.cu` kernels. The host build never compiles `#if defined(__CUDACC__)` blocks; that waits for Stage 8.
- **libstdc++ `std::min`/`std::max` on Linux aarch64.** Only libc++ was read. The standard pins the same `a < b` comparisons.
- **MSL acceptance of the core's C++ shapes:**
  - `thread`-qualified pointer members in local structs (`WheelScratch`, `WheelGeometry`, `GpuTyreIo`);
  - member functions called through a template `Io`/`Map` parameter;
  - `static_assert` and namespace-scope `namespace pc = …` in a stitched header.

  The MSL is never compiled offline, so the first proof is the Metal runner compile in Task 12 (Task 2's smoke covers only the toolkit). Step 12.4 names the fallback, which changes signatures only.
- **The GPU runner test model.** `tools/fp32_probe/artifacts/common/model.xml` is **untracked** in this worktree: `git ls-files tools/fp32_probe` is empty, though the file is present and not ignored. Nothing beyond the names was checked: that `grep` finds `geom_front_contact`, `geom_rear_contact` and `name="terrain"`. Whether `build_tyre_upload` accepts it (geom at the body origin, slide/hinge chain only) is unverified until Task 11, which leads to S9.
- **stand40's contact geoms at their wheel body origin** (`geom_pos == 0`), which P7's body-`xpos` centre relies on. The model builder writes `pos="0 0 0"` for both (`native/src/model/linkage.cpp:22`, `native/src/model/drivetrain.cpp:62`), and so does the probe XML (`model.xml:116,149`). That the binary `stand40/model.mjb` came from this builder is assumed, not read. Task 9 asserts it, which leads to S10.
- **`Stepper::set_state` leaves the tyre writer's station state untouched.** Read `stepper.cpp:570-593`: it does validation, snapshots, then `mj_resetData`, buffer writes and `mj_forward`. The rest of the body was not read. Tasks 10 and 12 rely on it.

---

## File structure

**Create (`native/src/physics_core/`, header-only, MSL + CUDA + host):**
- `platform.hpp`: address-space and inline macros (moved from `gpu/common.hpp:10-50`), plus the new `BIKE_CORE_TABLE`.
- `status.hpp`: `enum class Status`.
- `capacity.hpp`: `constexpr` capacities.
- `scalar.hpp`: `Lit<Real>`, `max_std`, `min_std`, `clip01`, `clamp_int`.
- `toolkit.hpp`: the toolkit contract (documentation, host-only `concept`, `BIKE_CORE_TOOLKIT_CHECK`).
- `gpu_toolkit.hpp`: `GpuToolkit` (float, plain loops, software `exp`).
- `cpu_exact_toolkit.hpp`: `CpuExactToolkit` (double, BLAS / `numeric::` wrappers). Host only, and `#error` under MSL/CUDA.
- `tyre_station.hpp`: station density and response (Task 3).
- `terrain.hpp`: polyline view, `np_interp`, height, closest-point query (Task 4).
- `tyre_bristle.hpp`: brush step, surface μ, effective friction, tangent transport (Task 5).
- `kinematics.hpp`: ring lift, station Jacobian, projection (Task 6), dof-chain Jacobian (Task 8).
- `tyre.hpp`: `classify_stations`, `wheel_contacts` (Task 7).

**Create (CPU side):**
- `native/src/physics_core/host_status.hpp`: `throw_core_status(Status)`, the D13 mapping to today's exceptions. Host only (`#error` under MSL/CUDA), like `cpu_exact_toolkit.hpp`; it sits in `physics_core/` because `tyre/`, `contact/` and `writers/` all include it.

**Create (GPU side):**
- `native/src/gpu/tyre_model.hpp`: GPU POD types (`GpuWheelChain`, `GpuTyreParams`, `GpuTyreModel`, states, outputs, `polyline_floats`) (Tasks 8–9).
- `native/src/gpu/tyre_step.hpp`: `gpu_polyline_view`, `GpuSurfaceMapAccess`, `GpuTyreIo`, `gpu_tyre_step` (Task 10).
- `native/src/gpu/tyre_upload.hpp` / `.cpp`: host-only builder `build_tyre_upload` and helpers (Task 8 chain, Task 9 the rest). Compiled into `bike_core`.

**Create (tests):**
- `native/tests/physics_core/test_physics_core.cpp`: CTest `physics_core_tests`. Frozen-copy differential tests for every extraction, plus toolkit tests.
- `native/tests/gpu/test_metal_math_mode.cpp`: CTest `bike_test_metal_math_mode` (Apple).
- `native/tests/reference/test_gpu_tyre_core.cpp`: CTest `reference.gpu_tyre_core`. Chain Jacobian vs `mj_jac`, upload, host-float vs CPU-double.
- `native/tests/gpu/test_metal_tyre_bitwise.cpp` + `native/tests/gpu/tyre_probe_abi.hpp`: CTest `reference.metal_tyre_bitwise` (Apple, registered in `GpuSharedCore.cmake`). Metal vs host-float.
- `native/tests/physics_core/frozen/{station,terrain,bristle,kinematics}.hpp`: verbatim pre-extraction copies (the D2 unit oracles).
- `native/tests/reference/GpuSharedCore.cmake`: registration of the reference-style GPU tests.

**Modify:**
- `native/src/gpu/common.hpp`: include `platform.hpp`, remove moved macros, retire `kMaxTreadStations`.
- `native/src/tyre/{station,distance,profile}.{hpp,cpp}`, `native/src/contact/laws.hpp`, `native/src/writers/{tire_common,distributed_tire}.{hpp,cpp}`: become wrappers over the core.
- `native/src/gpu/{batch_step.hpp,cpu_batch_runner.cpp,cuda_kernels.cu,metal_batch_runner.mm}`.
- `native/tests/gpu/{test_gpu_batch_step,test_gpu_batch_runners}.cpp`.
- `CMakeLists.txt`, `native/CMakeLists.txt`, `native/tests/reference/CMakeLists.txt`.
- `tools/colab_gpu/*.py`, `tools/colab_gpu/README.md`, `docs/TESTING.md`.

**Delete:** `native/src/gpu/tire_kernel.hpp`, `native/tests/gpu/test_gpu_tire.cpp`.

---

## Standard gates (copy these commands verbatim)

**Prerequisite (once per worktree):** `cmake --preset release`. Expected: the configure ends with `-- Build files have been written to: …/build/native-release`.

**G1 — D2 bit identity (after every task that touches CPU physics):**
```bash
cmake --build build/native-release -j
ctest --test-dir build/native-release --output-on-failure \
  -R '^(reference\.(native_distributed_tire|distributed_tire_reference|native_distributed_stands)|native_distance_tests|native_station_tests)$'
ctest --test-dir build/native-release -R golden_episode_digest --output-on-failure
```
Expected: `100% tests passed, 0 tests failed out of 5`, then `100% tests passed, 0 tests failed out of 1`.

**G2 — shared-helper users (after Tasks 4 and 5, which rewire helpers used by D3 backends):**
```bash
ctest --test-dir build/native-release --output-on-failure \
  -R '^reference\.(native_tire|native_pneumatic_tire|native_rider_contacts|native_rider_contact_math|tyre_road_profile)$'
```
Expected: `100% tests passed, 0 tests failed out of 5`.

**G3 — GPU unit tests:** `ctest --test-dir build/native-release -L gpu --output-on-failure`. Expected: `100% tests passed`.

**G4 — header sweep (after any task adding a header):** `cmake --build build/native-release --target check_headers`. Expected: exit 0.

## Stop conditions (report to the user and wait; never work around)

- **S1 (D2/R1).** G1's digest or a bitwise node fails after an extraction, and the frozen-copy unit test of that task passes. Stop. Report:
  - the failing node;
  - the first differing step and component (`bike-recording-digest` output for the digest);
  - the diff of the task.

  Do not adjust tolerances or re-capture.
- **S2 (D2, unit level).** A frozen-copy test of Tasks 3–6 reports a bit difference that cannot be traced to a typo against the frozen copy. Stop. Report:
  - the case, the inputs (as hex bit patterns), both outputs;
  - which toolkit primitive is involved (BLAS order, `np_interp_eval` fma, `exp`, volatile sincos/pow).
- **S3 (R2).** Task 1's probe finds no compile variant whose ops `{0,1,2,3,4,5,6,8,9,11}` are bitwise equal to host `float`. Report the full table and the max ulp distance per op. If contraction (ops 0–2) is the only failure and the max ulp distance is ≤ 1, D19's agreed fallback ("tolerance is 1 ulp") applies. Record it and continue, with Task 12 comparing in ulps. If ops 3/4 (div/sqrt) fail, stop: `GpuToolkit::div`/`sqrt` need a correctly rounded software routine, which is a user decision.
- **S4 (R3).** Task 8's chain Jacobian differs from `mj_jac` by more than 1e-12 (double, host) at any pose. Stop: the dof-chain assumption is wrong. Report the body, the dof, both columns and the pose.
- **S5.** Task 10's host-float vs CPU-double comparison exceeds its documented tolerance. Stop. Report:
  - per-dof values for both and the relative error;
  - the active station sets of both (a set difference is expected only at the penetration boundary).
- **S6.** Task 12's Metal vs host-float tyre step is not bitwise equal (or exceeds 1 ulp in D19 fallback mode). Stop. Report:
  - the first differing field, step and station;
  - whether a denormal or a transcendental is involved (Task 1 table).
- **S7.** Any pre-existing GPU test fails after Task 2 moves the macros. A failing `-L gpu` node not explained by a compile error in the moved code means the macro move changed semantics. Stop and report.
- **S8.** `stand40` has `nv > 64` or `nbody > 64` (Step 8.2). Raising GPU capacities beyond 64 changes thread-memory budgets materially. Report the dimensions.
- **S9.** The runner test model (`tools/fp32_probe/artifacts/common/model.xml`, untracked) is missing, or `build_tyre_upload` rejects it (Task 11). The batch-runner tests then have no tracked, upload-compatible model. Choosing or adding one is a user decision. Report the exception text.
- **S10.** `build_tyre_upload` rejects stand40 with "GPU tyre needs the contact geom at the wheel body origin" (Task 9). P7's body-`xpos` centre is then not the CPU's `geom_xpos` centre. The fix (a per-world `geom_xpos` input) changes the runner ABI again and needs approval.

---

## Task 1: Metal safe-math fact and production compile options (D19, R2)

**Files:**
- Modify: `native/src/gpu/metal_batch_runner.mm`:
  - includes near :21–36: add `#include <cstring>`;
  - anonymous namespace: insert the variant helpers directly above `build_msl_shader_source` (:101);
  - `build_msl_shader_source` :112 (`std::string combined = R"MSL(`) prologue;
  - `newLibraryWithSource` :878–880;
  - `extern "C"` block after `bike_metal_get_shader_source` (:1024–1039).
- Create: `native/tests/gpu/test_metal_math_mode.cpp`
- Modify: `CMakeLists.txt`: insert between :752 (`endif()` of the APPLE link block) and :753 (`endif()` of `BUILD_TESTING`).
- Create: `docs/superpowers/audits/2026-11-03-metal-safe-math-probe.md` (the recorded table; required by D19 "that fact is established first").

**Interfaces:**
- Produces:
  - `extern "C" int bike_metal_production_compile_variant();`
  - `extern "C" int bike_metal_run_probe(const char *source, const char *kernel_name, int variant, const void *in, std::size_t in_bytes, void *out, std::size_t out_bytes, int thread_count, char *err, std::size_t err_capacity);`
  - Meaning:
    - `variant = -1` selects the production options and prologue. Tasks 2 and 12 reuse this.
    - Kernel ABI: `buffer(0)` = input bytes, `buffer(1)` = output bytes, one thread per element.
    - Return codes: 0 ok; -1 compile error (text in `err`); -2 lookup/pipeline; -3 command buffer; -4 bad arguments; -5 no device.
  - Anonymous-namespace `MTLCompileOptions *make_compile_options(int variant)`, `const char *msl_prologue(int variant)` and `constexpr int kProductionCompileVariant`.

- [ ] **Step 1.1: Write the failing probe test.** Create `native/tests/gpu/test_metal_math_mode.cpp`:

```cpp
// test_metal_math_mode.cpp — D19/R2 probe. Compiles one MSL kernel under each
// compile-option variant of metal_batch_runner.mm, runs adversarial float
// cases on the device and compares bits with this host build
// (-ffp-contract=off via bike_apply_native_policy). Gated ops must be bitwise
// equal under the production variant; ops 7 (denormals) and 10 (precise::exp)
// are recorded only. The printed table is the evidence recorded in
// docs/superpowers/audits/2026-11-03-metal-safe-math-probe.md.
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

extern "C" int bike_metal_production_compile_variant();
extern "C" int bike_metal_run_probe(const char *source, const char *kernel_name,
                                    int variant, const void *in,
                                    std::size_t in_bytes, void *out,
                                    std::size_t out_bytes, int thread_count,
                                    char *err, std::size_t err_capacity);

namespace {

struct ProbeIn {
    float a;
    float b;
    float c;
    int op;
};
static_assert(sizeof(ProbeIn) == 16, "ProbeIn mirrors the MSL struct");

constexpr int kOps = 12;
constexpr int kVariants = 5;
// Ops whose bits must match under the production variant.
constexpr std::array<bool, kOps> kGated = {true, true, true, true, true, true,
                                           true, false, true, true, false, true};

constexpr const char *kProbeSource = R"MSL(
#include <metal_stdlib>
using namespace metal;
struct ProbeIn { float a; float b; float c; int op; };
kernel void bike_probe(device const ProbeIn *in [[buffer(0)]],
                       device float *out [[buffer(1)]],
                       uint gid [[thread_position_in_grid]]) {
    const ProbeIn p = in[gid];
    float r = 0.0f;
    switch (p.op) {
        case 0: r = p.a * p.b + p.c; break;
        case 1: r = p.c - p.a * p.b; break;
        case 2: r = p.a * p.b + p.c * p.a; break;
        case 3: r = p.a / p.b; break;
        case 4: r = sqrt(p.a); break;
        case 5: r = isnan(p.a) ? 1.0f : 0.0f; break;
        case 6: r = (p.a < p.b) ? p.b : p.a; break;
        case 7: r = p.a * p.b; break;
        case 8: r = p.a + p.b; break;
        case 9: r = fma(p.a, p.b, p.c); break;
        case 10: r = precise::exp(p.a); break;
        case 11: r = isfinite(p.a) ? 1.0f : 0.0f; break;
        default: r = 0.0f; break;
    }
    out[gid] = r;
}
)MSL";

float host_op(const ProbeIn &p) {
    switch (p.op) {
        case 0: return p.a * p.b + p.c;
        case 1: return p.c - p.a * p.b;
        case 2: return p.a * p.b + p.c * p.a;
        case 3: return p.a / p.b;
        case 4: return std::sqrt(p.a);
        case 5: return std::isnan(p.a) ? 1.0f : 0.0f;
        case 6: return (p.a < p.b) ? p.b : p.a;
        case 7: return p.a * p.b;
        case 8: return p.a + p.b;
        case 9: return std::fma(p.a, p.b, p.c);
        case 10: return std::exp(p.a);
        case 11: return std::isfinite(p.a) ? 1.0f : 0.0f;
        default: return 0.0f;
    }
}

std::vector<ProbeIn> make_corpus() {
    std::vector<ProbeIn> v;
    std::mt19937 rng(20261103u);
    std::uniform_real_distribution<float> unit(-4.0f, 4.0f);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const float tiny = std::numeric_limits<float>::denorm_min();
    for (int k = 0; k < 2048; ++k) {
        const float a = unit(rng);
        float b = unit(rng);
        if (b == 0.0f) b = 0.5f;
        const float prod = a * b; // rounded: -ffp-contract=off on the host
        v.push_back({a, b, -prod, 0});        // 0 unless fused
        v.push_back({a, b, prod, 1});         // 0 unless fused
        v.push_back({a, b, -b, 2});           // 0 unless fused
        v.push_back({a, b, 0.0f, 3});
        v.push_back({std::fabs(a) * 1000.0f, 0.0f, 0.0f, 4});
        v.push_back({a, b, 0.0f, 8});
        v.push_back({a, b, -prod, 9});
        v.push_back({a * 20.0f, 0.0f, 0.0f, 10});
    }
    for (const float x : {nan, -nan, inf, -inf, 1.0f, -0.0f}) {
        v.push_back({x, 0.0f, 0.0f, 5});
        v.push_back({x, 0.0f, 0.0f, 11});
        v.push_back({x, 1.0f, 0.0f, 6});
        v.push_back({1.0f, x, 0.0f, 6});
    }
    v.push_back({-0.0f, 0.0f, 0.0f, 6});
    v.push_back({0.0f, -0.0f, 0.0f, 6});
    v.push_back({1e-20f, 1e-20f, 0.0f, 7});
    v.push_back({tiny, 1.0f, 0.0f, 7});
    v.push_back({1.1754942e-38f, 0.5f, 0.0f, 7});
    v.push_back({tiny, 0.0f, 0.0f, 4});
    return v;
}

bool same(float x, float y) {
    if (std::isnan(x) && std::isnan(y)) return true;
    return std::bit_cast<std::uint32_t>(x) == std::bit_cast<std::uint32_t>(y);
}

std::int64_t ulps(float x, float y) {
    if (same(x, y)) return 0;
    if (!std::isfinite(x) || !std::isfinite(y)) return -1;
    const auto key = [](float f) {
        const auto u = static_cast<std::int64_t>(std::bit_cast<std::uint32_t>(f));
        return (u & 0x80000000LL) != 0 ? 0x80000000LL - u : u;
    };
    return std::llabs(key(x) - key(y));
}

} // namespace

int main() {
    const std::vector<ProbeIn> corpus = make_corpus();
    const int count = static_cast<int>(corpus.size());
    std::vector<float> host(corpus.size());
    for (std::size_t i = 0; i < corpus.size(); ++i) host[i] = host_op(corpus[i]);

    std::array<bool, kVariants> passes{};
    std::cout << "variant | mismatches per op 0..11 (max ulp) | gated\n";
    for (int variant = 0; variant < kVariants; ++variant) {
        std::vector<float> dev(corpus.size(), 0.0f);
        std::array<char, 4096> err{};
        const int rc = bike_metal_run_probe(
            kProbeSource, "bike_probe", variant, corpus.data(),
            corpus.size() * sizeof(ProbeIn), dev.data(), dev.size() * sizeof(float),
            count, err.data(), err.size());
        if (rc != 0) {
            std::cout << variant << " | run failed rc=" << rc << ": " << err.data() << '\n';
            continue;
        }
        std::array<int, kOps> bad{};
        std::array<std::int64_t, kOps> worst{};
        for (std::size_t i = 0; i < corpus.size(); ++i) {
            const auto op = static_cast<std::size_t>(corpus[i].op);
            if (!same(dev[i], host[i])) {
                bad[op] += 1;
                const std::int64_t u = ulps(dev[i], host[i]);
                worst[op] = (u < 0 || worst[op] < 0) ? -1 : std::max(worst[op], u);
            }
        }
        bool gated_ok = true;
        std::cout << variant << " |";
        for (std::size_t op = 0; op < kOps; ++op) {
            std::cout << ' ' << bad[op] << '(' << worst[op] << ')';
            if (kGated[op] && bad[op] != 0) gated_ok = false;
        }
        std::cout << " | " << (gated_ok ? "PASS" : "FAIL") << '\n';
        passes[static_cast<std::size_t>(variant)] = gated_ok;
    }
    int recommended = -1;
    for (int variant = 1; variant < kVariants; ++variant)
        if (passes[static_cast<std::size_t>(variant)]) { recommended = variant; break; }
    const int production = bike_metal_production_compile_variant();
    std::cout << "RECOMMENDED VARIANT " << recommended << "; PRODUCTION VARIANT "
              << production << '\n';
    if (recommended < 0) {
        std::cerr << "FAIL NO SAFE VARIANT (stop condition S3)\n";
        return 1;
    }
    if (!passes[static_cast<std::size_t>(production)]) {
        std::cerr << "FAIL production variant " << production
                  << " is not bitwise; set kProductionCompileVariant = "
                  << recommended << '\n';
        return 1;
    }
    std::cout << "PASS metal_math_mode\n";
    return 0;
}
```

Register it in `CMakeLists.txt` between :752 and :753:

```cmake
  if(APPLE)
    # D19 probe: links the Metal runner object for bike_metal_run_probe.
    bike_add_gpu_unit_test(bike_test_metal_math_mode
      "${_bike_native_tests_dir}/gpu/test_metal_math_mode.cpp"
      $<TARGET_OBJECTS:bike_gpu_metal_runner_obj>)
    target_link_libraries(bike_test_metal_math_mode PRIVATE
      Threads::Threads bike::mujoco ${CMAKE_DL_LIBS}
      "-framework Metal" "-framework Foundation")
    bike_rewrite_mujoco_load_name(bike_test_metal_math_mode "${BIKE_DEP_mujoco_LIBRARY}")
  endif()
```

- [ ] **Step 1.2: Run it to verify it fails.** Run:
  ```bash
  cmake --preset release && cmake --build build/native-release --target bike_test_metal_math_mode
  ```
  Expected: a link error with `undefined symbol: _bike_metal_run_probe`.

- [ ] **Step 1.3: Implement the probe entry and the production options.** Edit `native/src/gpu/metal_batch_runner.mm`:

  1. Add `#include <cstring>` after `#include <cstdint>` (:27).
  2. Insert directly above `std::string build_msl_shader_source(` (:101), inside the anonymous namespace (these helpers depend on nothing defined later; the probe entry in item 5 also uses `wait_command_buffer_ok`, defined at :824, before the `extern "C"` block at :1003):

```objc
// D19 compile-option variants. The production choice is measured by
// bike_test_metal_math_mode and recorded in
// docs/superpowers/audits/2026-11-03-metal-safe-math-probe.md:
//   0 — nil options (Metal default: fast math; the pre-Stage-1 behaviour)
//   1 — mathMode = Safe, mathFloatingPointFunctions = Precise
//   2 — variant 1 + "#pragma METAL fp contract(off)" prologue
//   3 — variant 1 + "#pragma clang fp contract(off)" prologue
//   4 — variant 1 + preserveInvariance = YES
constexpr int kProductionCompileVariant = 1;

MTLCompileOptions *make_compile_options(int variant) {
    if (variant == 0) {
        return nil;
    }
    MTLCompileOptions *options = [MTLCompileOptions new];
    options.mathMode = MTLMathModeSafe;
    options.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;
    if (variant == 4) {
        options.preserveInvariance = YES;
    }
    return options;
}

// Source-level prologue. It reaches warp-metal's wp_metal_load_library too,
// whose MTLCompileOptions this repository cannot set.
const char *msl_prologue(int variant) {
    switch (variant) {
        case 2: return "#pragma METAL fp contract(off)\n";
        case 3: return "#pragma clang fp contract(off)\n";
        default: return "";
    }
}
```

  3. In `build_msl_shader_source`, change the initialisation at :112 from `std::string combined = R"MSL(` to the following, then close the raw string unchanged:

```cpp
    std::string combined = std::string(msl_prologue(kProductionCompileVariant)) + R"MSL(
```

  The prologue precedes the existing `#pragma clang diagnostic ignored "-Wc++17-extensions"` line (:113); `strip_includes_and_pragmas` only processes the stitched headers, never `combined`'s own text.

  4. Replace `options:nil` at :879 with `options:make_compile_options(kProductionCompileVariant)`.
  5. Add these functions to the `extern "C"` block, after `bike_metal_get_shader_source`:

```objc
int bike_metal_production_compile_variant() {
    return kProductionCompileVariant;
}

// Diagnostic entry point (tests only): compile `source` under `variant`
// (-1 = production), run `kernel_name` over `thread_count` threads with
// buffer(0) = in and buffer(1) = out, and copy `out` back.
int bike_metal_run_probe(const char *source, const char *kernel_name, int variant,
                         const void *in, std::size_t in_bytes, void *out,
                         std::size_t out_bytes, int thread_count, char *err,
                         std::size_t err_capacity) {
    const std::span<char> err_span =
        err != nullptr ? boundary_span(err, err_capacity) : std::span<char>{};
    const auto report = [&](const std::string &msg) {
        if (err_span.empty()) {
            return;
        }
        const std::size_t k = std::min(err_span.size() - 1, msg.size());
        std::copy_n(msg.data(), k, err_span.begin());
        err_span[k] = '\0';
    };
    if (source == nullptr || kernel_name == nullptr || in == nullptr || out == nullptr ||
        in_bytes == 0 || out_bytes == 0 || thread_count <= 0) {
        report("invalid probe arguments");
        return -4;
    }
    @autoreleasepool {
        const int effective = variant < 0 ? kProductionCompileVariant : variant;
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (!device) {
            report("no Metal device");
            return -5;
        }
        const std::string full = std::string(msl_prologue(effective)) + source;
        NSError *error = nil;
        id<MTLLibrary> library =
            [device newLibraryWithSource:[NSString stringWithUTF8String:full.c_str()]
                                 options:make_compile_options(effective)
                                   error:&error];
        if (!library) {
            report(error ? [[error localizedDescription] UTF8String] : "unknown compile error");
            return -1;
        }
        id<MTLFunction> fn = [library newFunctionWithName:[NSString stringWithUTF8String:kernel_name]];
        if (!fn) {
            report("kernel not found");
            return -2;
        }
        id<MTLComputePipelineState> pso = [device newComputePipelineStateWithFunction:fn error:&error];
        if (!pso) {
            report(error ? [[error localizedDescription] UTF8String] : "pipeline error");
            return -2;
        }
        id<MTLCommandQueue> queue = [device newCommandQueue];
        id<MTLBuffer> b_in = [device newBufferWithBytes:in length:in_bytes
                                                options:MTLResourceStorageModeShared];
        id<MTLBuffer> b_out = [device newBufferWithLength:out_bytes
                                                  options:MTLResourceStorageModeShared];
        id<MTLCommandBuffer> cb = [queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
        [enc setComputePipelineState:pso];
        [enc setBuffer:b_in offset:0 atIndex:0];
        [enc setBuffer:b_out offset:0 atIndex:1];
        const NSUInteger tg = std::min<NSUInteger>(pso.maxTotalThreadsPerThreadgroup, 64);
        [enc dispatchThreads:MTLSizeMake(static_cast<NSUInteger>(thread_count), 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cb commit];
        if (!wait_command_buffer_ok(cb, "bike_metal_run_probe")) {
            report("command buffer failed");
            return -3;
        }
        std::memcpy(out, [b_out contents], out_bytes);
        return 0;
    }
}
```

- [ ] **Step 1.4: Run the probe.** Run:
  ```bash
  cmake --build build/native-release --target bike_test_metal_math_mode && \
    ctest --test-dir build/native-release -R '^bike_test_metal_math_mode$' -V
  ```
  Expected: a table of 5 rows. Variant 0 most likely shows non-zero counts in ops 0–2 (contraction). This is not asserted: it is the fact being measured.

  Decision rule, as printed by the test:
  - If the production variant passes, the test ends with `PASS metal_math_mode`.
  - If it fails but another variant passes, set `kProductionCompileVariant` to the printed `RECOMMENDED VARIANT` and rerun until `PASS`.
  - If `FAIL NO SAFE VARIANT` is printed → **S3**. In D19 fallback mode (only ops 0–2 failing, max ulp ≤ 1), set `kProductionCompileVariant = 1`. Change `kGated[0..2]` to `false` with the comment `// D19 fallback: contraction cannot be disabled; Task 12 compares in ulps`. Record the mode in the audit file, and set Task 12 to ulp mode (Step 12.5).

- [ ] **Step 1.5: Record the fact.** Create `docs/superpowers/audits/2026-11-03-metal-safe-math-probe.md` containing:
  - the date and the `sw_vers` output;
  - the Metal device name (`bike_metal_device_name`, printed by `bike_test_gpu_batch_runners -V` or by adding `std::cout` in the test);
  - the full table from Step 1.4;
  - the chosen variant and the reason;
  - the denormal (op 7) and `precise::exp` (op 10) mismatch counts, recorded as facts. Op 10 justifies plan decision P5. If op 7 mismatches, write "Metal flushes fp32 denormals under the production variant" and reference stop condition S6.
  - the explicit note: "warp-metal's own compile options remain unverified. A source prologue covers them only if the production variant is 2 or 3."

- [ ] **Step 1.6: Run the existing runner test.** It now compiles the production MSL with the new options. Run:
  ```bash
  ctest --test-dir build/native-release -R '^bike_test_gpu_batch_runners$' --output-on-failure
  ```
  Expected: PASS. If it fails only on a pre-existing numeric expectation that relied on fast math, report it under S7 with the failing assertion.

- [ ] **Step 1.7: Commit.**
```bash
git add native/src/gpu/metal_batch_runner.mm native/tests/gpu/test_metal_math_mode.cpp CMakeLists.txt docs/superpowers/audits/2026-11-03-metal-safe-math-probe.md
git commit -m "feat(gpu): measure Metal safe math and compile writers in it

D19/R2: the probe compares Metal fp32 against host float bit for bit per
compile-option variant; production uses the measured safe variant instead
of the options:nil fast-math default.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Task 2: `physics_core/` scaffolding — platform, status, scalars, toolkits

**Files:**
- Create: `native/src/physics_core/{platform,status,capacity,scalar,toolkit,gpu_toolkit,cpu_exact_toolkit}.hpp`
- Create: `native/src/physics_core/host_status.hpp`
- Modify: `native/src/gpu/common.hpp:10-50` (replace with an include), `:57-62` (capacities)
- Modify: `native/src/gpu/metal_batch_runner.mm:103-110` (header list), plus a new `extern "C" bike_metal_get_core_source`
- Create: `native/tests/physics_core/test_physics_core.cpp`; CTest `physics_core_tests`
- Modify: `CMakeLists.txt`: after :596 (register `physics_core_tests`) and in the APPLE block from Task 1 (add a core MSL smoke to `bike_test_metal_math_mode`)

**Interfaces (produced; later tasks use exactly these names):**
- Macros: `BIKE_HD`, `BIKE_INLINE`, `BIKE_THREAD`, `BIKE_DEVICE`, `BIKE_CONSTANT`, `BIKE_GLOBAL`, `BIKE_CONSTEXPR`, `BIKE_CORE_TABLE`, `BIKE_GPU_SHARED_CODE_BEGIN/END`.
- `namespace bike::physics_core`:
  - `enum class Status : int`;
  - `Lit<Real>::{half, tie_abs, normal_tol, length_floor, one_minus_floor}()`;
  - `max_std`, `min_std`, `clip01`, `clamp_int`;
  - `struct GpuToolkit` and `struct CpuExactToolkit`, each with `using Real` plus static `dot`, `gemv_n3`, `gemv_t3`, `ring_lift`, `interp_segment`, `div`, `sqrt`, `abs`, `exp`, `isfinite`, `isnan`;
  - capacities `kMaxTyreStations = 512`, `kMaxGpuTyreStations = 128`, `kMaxChainJoints = 8`, `kMaxSurfaceSections = 16`, `kMaxHingeKnots = 8`.
- `[[noreturn]] void throw_core_status(bike::physics_core::Status)` in `physics_core/host_status.hpp`.

- [ ] **Step 2.1: Write the failing toolkit tests.** Create `native/tests/physics_core/test_physics_core.cpp` with the harness of `native/tests/test_station.cpp`: copy its lines 22–48 (`TestFunction`, `TestCase`, `require`, `require_throws_invalid_argument`) into the anonymous namespace, its lines 218–228 (`run_case`) after the `cases` array, and its `main` (lines 231–269) after the namespace. In the usage string, change `native_station_tests` to `physics_core_tests` and `native station case` to `physics core case`. Then add:

```cpp
// test_physics_core.cpp — physics_core/ contracts. Each extraction task adds
// a frozen-copy differential case: the pre-extraction code is copied verbatim
// from HEAD into namespace frozen_* and compared bit for bit with the core
// instantiated as <double, CpuExactToolkit> (D2).
#include "../../src/physics_core/cpu_exact_toolkit.hpp"
#include "../../src/physics_core/gpu_toolkit.hpp"
#include "../../src/physics_core/scalar.hpp"
#include "../../src/physics_core/toolkit.hpp"
#include "../../src/cblas_abi.hpp"
#include "../../src/numeric_numpy.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace pc = bike::physics_core;

namespace {
    // (harness copied from test_station.cpp:21-48 goes here)

    [[nodiscard]] bool same_bits(double a, double b) {
        return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
    }
    [[nodiscard]] bool same_bits(float a, float b) {
        return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b);
    }

    static_assert(pc::Toolkit<pc::CpuExactToolkit>);
    static_assert(pc::Toolkit<pc::GpuToolkit>);

    // CpuExactToolkit forwards to the very BLAS / numeric entry points the
    // CPU writers call today — same arguments, same bits.
    void test_cpu_exact_forwards() {
        std::mt19937_64 rng(7);
        std::uniform_real_distribution<double> u(-3.0, 3.0);
        constexpr int nv = 13;
        std::array<double, 3 * nv> a{};
        std::array<double, nv> x{};
        std::array<double, 3> f{};
        for (double &v : a) v = u(rng);
        for (double &v : x) v = u(rng);
        for (double &v : f) v = u(rng);
        std::array<double, 3> y_core{}, y_ref{};
        pc::CpuExactToolkit::gemv_n3(nv, a.data(), x.data(), y_core.data());
        blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3, nv, 1.0, a.data(), nv,
                    x.data(), 1, 0.0, y_ref.data(), 1);
        for (std::size_t k = 0; k < 3; ++k)
            require(same_bits(y_core[k], y_ref[k]), "gemv_n3 forwards dgemv(no)");
        std::array<double, nv> t_core{}, t_ref{};
        pc::CpuExactToolkit::gemv_t3(nv, a.data(), f.data(), t_core.data());
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, nv, 1.0, a.data(), nv,
                    f.data(), 1, 0.0, t_ref.data(), 1);
        for (std::size_t k = 0; k < nv; ++k)
            require(same_bits(t_core[k], t_ref[k]), "gemv_t3 forwards dgemv(yes)");
        require(same_bits(pc::CpuExactToolkit::dot(3, f.data(), x.data()),
                          blas::ddot(3, f.data(), 1, x.data(), 1)),
                "dot forwards ddot");
        std::array<double, 30> local{};
        std::array<double, 9> rot{};
        for (double &v : local) v = u(rng);
        for (double &v : rot) v = u(rng);
        std::array<double, 30> p_core{}, p_ref{};
        pc::CpuExactToolkit::ring_lift(10, local.data(), rot.data(), p_core.data());
        blas::dgemm(blas::Order::row_major, blas::Transpose::no, blas::Transpose::yes, 10, 3,
                    3, 1.0, local.data(), 3, rot.data(), 3, 0.0, p_ref.data(), 3);
        for (std::size_t k = 0; k < 30; ++k)
            require(same_bits(p_core[k], p_ref[k]), "ring_lift forwards dgemm(no, yes)");
        require(same_bits(pc::CpuExactToolkit::interp_segment(1.25, 0.3, 0.1, -2.0, 0.9, 4.0),
                          numeric::np_interp_segment(1.25, 0.3, 0.1, -2.0, 0.9, 4.0)),
                "interp_segment forwards np_interp_segment");
    }

    // std::max / std::min semantics, including NaN and signed zero.
    void test_scalar_semantics() {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const std::array<double, 6> vals{nan, -0.0, 0.0, 1.0, -1.0, 3.5};
        for (const double a : vals)
            for (const double b : vals) {
                require(same_bits(pc::max_std(a, b), std::max(a, b)) ||
                            (std::isnan(pc::max_std(a, b)) && std::isnan(std::max(a, b))),
                        "max_std == std::max");
                require(same_bits(pc::min_std(a, b), std::min(a, b)) ||
                            (std::isnan(pc::min_std(a, b)) && std::isnan(std::min(a, b))),
                        "min_std == std::min");
            }
        require(same_bits(pc::Lit<double>::one_minus_floor(), 1.0 - 1e-12),
                "Lit<double>::one_minus_floor");
        require(same_bits(pc::Lit<float>::tie_abs(), 2e-15f), "Lit<float>::tie_abs");
    }

    // GpuToolkit::exp: Cephes-style; relative error below 4 float ulps on
    // the domain the tyre uses (x <= 0), and exact special values.
    void test_gpu_exp() {
        require(pc::GpuToolkit::exp(0.0f) == 1.0f, "exp(0) == 1");
        require(pc::GpuToolkit::exp(-100.0f) == 0.0f, "exp below cutoff is 0");
        require(std::isnan(pc::GpuToolkit::exp(std::numeric_limits<float>::quiet_NaN())),
                "exp(NaN) is NaN");
        for (int i = 0; i <= 87000; ++i) {
            const float x = -static_cast<float>(i) * 1e-3f;
            const double want = std::exp(static_cast<double>(x));
            const double got = pc::GpuToolkit::exp(x);
            require(std::fabs(got - want) <= 4.0 * 1.1920929e-7 * want, "exp accuracy");
        }
    }
```

Then register the cases in the `cases` array:
```cpp
    constexpr std::array<TestCase, 3> cases{{
        {.name = "cpu_exact_forwards", .run = test_cpu_exact_forwards},
        {.name = "scalar_semantics", .run = test_scalar_semantics},
        {.name = "gpu_exp", .run = test_gpu_exp},
    }};
```
Later tasks append cases and update the array size.

Register the test in `CMakeLists.txt` directly after :596 (`add_test(NAME native_station_tests …)`), inside the same `if(BUILD_TESTING)`:
```cmake
  # physics_core/ contracts and frozen-copy differential tests (Stage 1).
  add_executable(physics_core_tests "${_bike_native_tests_dir}/physics_core/test_physics_core.cpp")
  bike_apply_native_policy(physics_core_tests)
  target_include_directories(physics_core_tests SYSTEM PRIVATE
    "${_bike_native_dir}/src" "${BIKE_DEP_mujoco_ROOT}/include")
  target_link_libraries(physics_core_tests PRIVATE bike_core bike_engine bike_rider_math)
  bike_rewrite_mujoco_load_name(physics_core_tests "${BIKE_DEP_mujoco_LIBRARY}")
  add_test(NAME physics_core_tests COMMAND physics_core_tests)
  set_tests_properties(physics_core_tests PROPERTIES LABELS "unit;tyre")
```

- [ ] **Step 2.2: Run to verify it fails.** Run `cmake --preset release && cmake --build build/native-release --target physics_core_tests`. Expected: `fatal error: '../../src/physics_core/cpu_exact_toolkit.hpp' file not found`.

- [ ] **Step 2.3: Create `physics_core/platform.hpp`.**
  - Move `native/src/gpu/common.hpp` lines 10–50 verbatim into it, under a `#pragma once` and a header comment.
  - In each of the three branches add one line after `BIKE_CONSTEXPR`:
    - `#define BIKE_CORE_TABLE device` in the Metal branch;
    - `#define BIKE_CORE_TABLE` (empty) in the CUDA and host branches.
  - Comment above the macros:

```cpp
// BIKE_CORE_TABLE: address space of read-only model tables the cores index
// (terrain polyline, station ring, weights, chain joints) and per-world
// engine kinematics (xmat, xanchor, xaxis).
// GPU callers keep these tables in device memory; scratch and per-call
// state are BIKE_THREAD.
```

  In `common.hpp`, replace lines 10–50 with `#include "../physics_core/platform.hpp"`.

- [ ] **Step 2.4: Create `physics_core/status.hpp`, `capacity.hpp`, `scalar.hpp`.**

```cpp
// physics_core/status.hpp — D13: cores never throw; they return a Status.
// CPU wrappers map each code to the exception thrown before Stage 1
// (physics_core/host_status.hpp); GPU worlds store it and stop.
#pragma once
#include "platform.hpp"

namespace bike::physics_core {
enum class Status : int {
    ok = 0,
    terrain_query_nonfinite = 1,      // distance.cpp:149-151
    terrain_outside_domain = 2,       // distance.cpp:108-110
    terrain_no_closest_segment = 3,   // distance.cpp:213-218 (logic_error)
    station_penetration_nonfinite = 4,// station.cpp:56-58
    station_invalid_quadrature = 5,   // station.cpp:83-86
    material_query_nonfinite = 6,     // tire_common.cpp:24-25
    brush_nonfinite = 7,              // laws.hpp:13-22 (overflow_error)
    brush_passivity = 8,              // laws.hpp:70-72 (overflow_error)
    capacity_exceeded = 9,            // GPU fixed arrays
    chain_unsupported_joint = 10,     // dof chain holds a non hinge/slide joint
    invalid_interval = 11,            // GPU: dt <= 0 (CPU keeps its own check)
};
} // namespace bike::physics_core
```

```cpp
// physics_core/capacity.hpp — compile-time capacities of the shared cores.
#pragma once
#include "platform.hpp"

namespace bike::physics_core {
BIKE_CONSTEXPR int kMaxTyreStations = 512;    // station.cpp:32 domain bound
BIKE_CONSTEXPR int kMaxGpuTyreStations = 128; // >= viewer_physics_distributed.toml station_count
BIKE_CONSTEXPR int kMaxChainJoints = 8;       // root(3) + pivots(2) + spin(1) fits
BIKE_CONSTEXPR int kMaxSurfaceSections = 16;  // alpine_rocky track uses 3
BIKE_CONSTEXPR int kMaxHingeKnots = 8;        // GPU density table (CPU is unbounded)
} // namespace bike::physics_core
```

```cpp
// physics_core/scalar.hpp — scalar helpers whose semantics equal the std::
// calls the CPU code makes (libc++ max.h:30 / min.h:30), and literal tables
// so host float and MSL round every decimal constant identically.
#pragma once
#include "platform.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL core. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class T> BIKE_HD T max_std(T a, T b) noexcept { return (a < b) ? b : a; }
template <class T> BIKE_HD T min_std(T a, T b) noexcept { return (b < a) ? b : a; }
// np.clip(t, 0., 1.) const-bound path (distance.cpp:48).
template <class Real> BIKE_HD Real clip01(Real t) noexcept {
    return t < Real(0) ? Real(0) : (t > Real(1) ? Real(1) : t);
}
// std::clamp on ints (distance.cpp:157-158).
BIKE_HD int clamp_int(int v, int lo, int hi) noexcept { return v < lo ? lo : (hi < v ? hi : v); }

template <class Real> struct Lit;
template <> struct Lit<float> {
    static BIKE_HD float half() noexcept { return 0.5f; }
    static BIKE_HD float tie_abs() noexcept { return 2e-15f; }
    static BIKE_HD float normal_tol() noexcept { return 1e-10f; }
    static BIKE_HD float length_floor() noexcept { return 1e-12f; }
    static BIKE_HD float one_minus_floor() noexcept { return 1.0f - 1e-12f; }
};
#if !defined(__METAL_VERSION__)
template <> struct Lit<double> {
    static BIKE_HD double half() noexcept { return 0.5; }
    static BIKE_HD double tie_abs() noexcept { return 2e-15; }      // distance.cpp:187
    static BIKE_HD double normal_tol() noexcept { return 1e-10; }   // distance.cpp:209,252
    static BIKE_HD double length_floor() noexcept { return 1e-12; } // distance.cpp:229,241-242
    static BIKE_HD double one_minus_floor() noexcept { return 1.0 - 1e-12; } // :243
};
#endif

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
```

Every later `physics_core/*.hpp` opens with the same NOLINTBEGIN line plus `BIKE_GPU_SHARED_CODE_BEGIN`, and closes with the matching END pair. This is stated once here; the code blocks below omit the two wrapper lines for brevity, and they must be present in the files.

- [ ] **Step 2.5: Create `toolkit.hpp`, `gpu_toolkit.hpp`, `cpu_exact_toolkit.hpp`.**

```cpp
// physics_core/toolkit.hpp — the primitive-operation contract (D9). A toolkit
// is a struct with `using Real` and these static members:
//   dot(n, a, b)             Σ a[i]*b[i] (CPU: blas::ddot, numpy-seeded)
//   gemv_n3(nv, A, x, y)     y(3) = A(3×nv, row-major) · x
//   gemv_t3(nv, A, x, y)     y(nv) = Aᵀ · x(3)
//   ring_lift(n, L, R, out)  out(n×3) = L(n×3) · Rᵀ (R row-major 3×3)
//   interp_segment(s, x, x0, y0, x1, y1)
//                            np.interp's segment evaluation with its
//                            right-endpoint retry and constant fallback
//                            (CPU: numeric::np_interp_segment, the single
//                            implementation; its fma lowering included)
//   div, sqrt, abs, exp, isfinite, isnan
// Pointer parameters are BIKE_THREAD except ring_lift's L (BIKE_CORE_TABLE).
#pragma once
#include "platform.hpp"

#if !defined(__METAL_VERSION__) && !defined(__CUDACC__)
#include <concepts>
namespace bike::physics_core {
template <class TK>
concept Toolkit = requires(int n, const typename TK::Real *c, typename TK::Real *m,
                           typename TK::Real r) {
    { TK::dot(n, c, c) } -> std::same_as<typename TK::Real>;
    { TK::gemv_n3(n, c, c, m) } -> std::same_as<void>;
    { TK::gemv_t3(n, c, c, m) } -> std::same_as<void>;
    { TK::ring_lift(n, c, c, m) } -> std::same_as<void>;
    { TK::interp_segment(r, r, r, r, r, r) } -> std::same_as<typename TK::Real>;
    { TK::div(r, r) } -> std::same_as<typename TK::Real>;
    { TK::sqrt(r) } -> std::same_as<typename TK::Real>;
    { TK::abs(r) } -> std::same_as<typename TK::Real>;
    { TK::exp(r) } -> std::same_as<typename TK::Real>;
    { TK::isfinite(r) } -> std::same_as<bool>;
    { TK::isnan(r) } -> std::same_as<bool>;
};
} // namespace bike::physics_core
#include <type_traits>
#define BIKE_CORE_TOOLKIT_CHECK(Real, TK)                                      \
    static_assert(::bike::physics_core::Toolkit<TK> &&                         \
                  std::is_same_v<Real, typename TK::Real>,                     \
                  "core instantiated with a mismatched toolkit")
#else
#define BIKE_CORE_TOOLKIT_CHECK(Real, TK) static_assert(true, "")
#endif
```

```cpp
// physics_core/gpu_toolkit.hpp — float toolkit for Metal, CUDA and the
// host-float reference: plain sequential loops, contraction-free
// expressions, and a portable exp (plan decision P5).
#pragma once
#include "platform.hpp"
#if !defined(__METAL_VERSION__)
#include <cmath>
#endif

namespace bike::physics_core {
struct GpuToolkit {
    using Real = float;

    static BIKE_HD float dot(int n, const BIKE_THREAD float *a, const BIKE_THREAD float *b) noexcept {
        float s = 0.0f;
        for (int i = 0; i < n; ++i) s += a[i] * b[i];
        return s;
    }
    static BIKE_HD void gemv_n3(int nv, const BIKE_THREAD float *a, const BIKE_THREAD float *x,
                                BIKE_THREAD float *y) noexcept {
        for (int r = 0; r < 3; ++r) {
            float s = 0.0f;
            for (int c = 0; c < nv; ++c) s += a[r * nv + c] * x[c];
            y[r] = s;
        }
    }
    static BIKE_HD void gemv_t3(int nv, const BIKE_THREAD float *a, const BIKE_THREAD float *x,
                                BIKE_THREAD float *y) noexcept {
        for (int c = 0; c < nv; ++c) {
            float s = 0.0f;
            for (int r = 0; r < 3; ++r) s += a[r * nv + c] * x[r];
            y[c] = s;
        }
    }
    static BIKE_HD void ring_lift(int n, const BIKE_CORE_TABLE float *local,
                                  const BIKE_THREAD float *rotation, BIKE_THREAD float *out) noexcept {
        for (int i = 0; i < n; ++i)
            for (int k = 0; k < 3; ++k) {
                float s = 0.0f;
                for (int j = 0; j < 3; ++j) s += local[3 * i + j] * rotation[3 * k + j];
                out[3 * i + k] = s;
            }
    }
    // numeric::np_interp_segment's float twin: left-knot evaluation,
    // right-endpoint retry, constant-segment fallback; unfused.
    static BIKE_HD float interp_segment(float slope, float x, float x0, float y0, float x1,
                                        float y1) noexcept {
        float result = slope * (x - x0) + y0;
        if (isnan(result)) {
            result = slope * (x - x1) + y1;
            if (isnan(result) && y0 == y1) {
                result = y0;
            }
        }
        return result;
    }
    static BIKE_HD float div(float a, float b) noexcept { return a / b; }
#if defined(__METAL_VERSION__)
    static BIKE_HD float sqrt(float x) noexcept { return metal::sqrt(x); }
    static BIKE_HD float abs(float x) noexcept { return metal::fabs(x); }
    static BIKE_HD bool isfinite(float x) noexcept { return metal::isfinite(x); }
    static BIKE_HD bool isnan(float x) noexcept { return metal::isnan(x); }
    static BIKE_HD float floor_(float x) noexcept { return metal::floor(x); }
    static BIKE_HD float ldexp_(float x, int e) noexcept { return metal::ldexp(x, e); }
    static BIKE_HD float inf_() noexcept { return INFINITY; }
#else
    static BIKE_HD float sqrt(float x) noexcept { return ::sqrtf(x); }
    static BIKE_HD float abs(float x) noexcept { return ::fabsf(x); }
    static BIKE_HD bool isfinite(float x) noexcept { return std::isfinite(x); }
    static BIKE_HD bool isnan(float x) noexcept { return std::isnan(x); }
    static BIKE_HD float floor_(float x) noexcept { return ::floorf(x); }
    static BIKE_HD float ldexp_(float x, int e) noexcept { return ::ldexpf(x, e); }
    static BIKE_HD float inf_() noexcept { return HUGE_VALF; }
#endif
    // Cephes expf (S. Moshier): split-ln2 reduction, degree-5 minimax
    // polynomial, ldexp. Below -126·ln2 the result would be subnormal,
    // which Metal may flush, so it returns 0 (Task 1 op 7).
    static BIKE_HD float exp(float x) noexcept {
        if (isnan(x)) return x;
        if (x > 88.72283905206835f) return inf_();
        if (x < -87.33654475055310f) return 0.0f;
        const float fx = floor_(1.44269504088896341f * x + 0.5f);
        const int n = static_cast<int>(fx);
        float r = x - fx * 0.693359375f;
        r = r - fx * -2.12194440e-4f;
        const float z = r * r;
        float p = 1.9875691500e-4f;
        p = p * r + 1.3981999507e-3f;
        p = p * r + 8.3334519073e-3f;
        p = p * r + 4.1665795894e-2f;
        p = p * r + 1.6666665459e-1f;
        p = p * r + 5.0000001201e-1f;
        const float y = (p * z + r) + 1.0f;
        return ldexp_(y, n);
    }
};
} // namespace bike::physics_core
```

```cpp
// physics_core/cpu_exact_toolkit.hpp — CPU-only toolkit: forwards to the
// exact entry points the pre-Stage-1 CPU writers call, so every
// <double, CpuExactToolkit> instantiation keeps the CPU bits (D2, D9).
// Never part of the MSL stitch.
#pragma once
#if defined(__METAL_VERSION__) || defined(__CUDACC__)
#error "cpu_exact_toolkit.hpp is host-only"
#endif
#include "../cblas_abi.hpp"
#include "../numeric_numpy.hpp"
#include <cmath>

namespace bike::physics_core {
struct CpuExactToolkit {
    using Real = double;
    static double dot(int n, const double *a, const double *b) noexcept {
        return blas::ddot(n, a, 1, b, 1);
    }
    static void gemv_n3(int nv, const double *a, const double *x, double *y) noexcept {
        blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3, nv, 1.0, a, nv, x, 1, 0.0, y, 1);
    }
    static void gemv_t3(int nv, const double *a, const double *x, double *y) noexcept {
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, nv, 1.0, a, nv, x, 1, 0.0, y, 1);
    }
    static void ring_lift(int n, const double *local, const double *rotation, double *out) noexcept {
        blas::dgemm(blas::Order::row_major, blas::Transpose::no, blas::Transpose::yes, n, 3, 3,
                    1.0, local, 3, rotation, 3, 0.0, out, 3);
    }
    static double interp_segment(double slope, double x, double x0, double y0, double x1,
                                 double y1) noexcept {
        return numeric::np_interp_segment(slope, x, x0, y0, x1, y1);
    }
    static double div(double a, double b) noexcept { return a / b; }
    static double sqrt(double x) noexcept { return std::sqrt(x); }
    static double abs(double x) noexcept { return std::abs(x); }
    static double exp(double x) noexcept { return std::exp(x); }
    static bool isfinite(double x) noexcept { return std::isfinite(x); }
    static bool isnan(double x) noexcept { return std::isnan(x); }
};
} // namespace bike::physics_core
```

Create `native/src/physics_core/host_status.hpp`:
```cpp
// physics_core/host_status.hpp — D13 mapping from a core Status to the
// exception (type and message) the CPU code threw before the extraction.
// Host only; never part of the MSL stitch.
#pragma once
#if defined(__METAL_VERSION__) || defined(__CUDACC__)
#error "host_status.hpp is host-only"
#endif
#include "status.hpp"
#include <stdexcept>

[[noreturn]] inline void throw_core_status(bike::physics_core::Status s) {
    using S = bike::physics_core::Status;
    switch (s) {
        case S::terrain_query_nonfinite:
            throw std::invalid_argument("distance queries must be a finite (N, 2) array");
        case S::terrain_outside_domain:
            throw std::invalid_argument("query lies outside the compiled road domain");
        case S::terrain_no_closest_segment:
            throw std::logic_error("signed-distance window produced no closest segment");
        case S::station_penetration_nonfinite:
            throw std::invalid_argument("contact penetration must be finite");
        case S::station_invalid_quadrature:
            throw std::invalid_argument("invalid contact quadrature");
        case S::material_query_nonfinite:
            throw std::invalid_argument("material query must be finite");
        case S::brush_nonfinite:
            throw std::overflow_error("tire calculation exceeds finite floating-point range");
        case S::brush_passivity:
            throw std::overflow_error("brush update violates discrete passivity");
        case S::capacity_exceeded:
            throw std::invalid_argument("physics core capacity exceeded");
        case S::chain_unsupported_joint:
            throw std::invalid_argument("wheel chain has an unsupported joint type");
        case S::invalid_interval:
            throw std::invalid_argument("invalid distributed tire interval");
        case S::ok:
            break;
    }
    throw std::logic_error("throw_core_status called without an error");
}
```

In `common.hpp`, replace `:61-62` (`kMaxTreadStations`, `kMaxHingeKnots`) with:
```cpp
// Tyre capacities moved to physics_core/capacity.hpp (Stage 1).
using ::bike::physics_core::kMaxHingeKnots;
```
and add `#include "../physics_core/capacity.hpp"` under the platform include. `kMaxTreadStations` stays in place until Task 11 deletes its last users. So for this task keep `BIKE_CONSTEXPR int kMaxTreadStations = 64;` and replace only the `kMaxHingeKnots` line.

- [ ] **Step 2.6: Teach the MSL stitcher about `physics_core/`.** In `metal_batch_runner.mm` :103–110, make the list:
```cpp
    const std::vector<std::string> headers = {
        "../physics_core/platform.hpp",
        "../physics_core/status.hpp",
        "../physics_core/capacity.hpp",
        "../physics_core/scalar.hpp",
        "../physics_core/toolkit.hpp",
        "../physics_core/gpu_toolkit.hpp",
        "common.hpp",
        "suspension_kernel.hpp",
        "tire_kernel.hpp",
        "drivetrain_resistance_kernel.hpp",
        "rider_kernel.hpp",
        "batch_step.hpp"
    };
```
`toolkit.hpp`'s host-only `#include <concepts>` sits inside `#if !defined(__METAL_VERSION__)`, and the stitcher strips the `#include` line. That is safe because the guarded block is dead under MSL.

Add a core-only bundle for tests. Split the reading loop into a helper `std::string stitch(const std::string &base, const std::vector<std::string> &headers)` that holds the existing `read_file_str` / `strip_includes_and_pragmas` loop. Then add to the `extern "C"` block:
```objc
// Stitched physics_core/ headers only, under the production prologue (tests).
const char *bike_metal_get_core_source(const char *gpu_src_dir) {
    static std::mutex core_mutex;
    static std::string cached_core;
    std::lock_guard<std::mutex> lock(core_mutex);
    std::string dir_str;
    if (gpu_src_dir != nullptr && gpu_src_dir[0] != '\0') {
        dir_str = gpu_src_dir;
    } else {
        const std::string file_path = __FILE__;
        const std::size_t slash = file_path.find_last_of("/\\");
        dir_str = (slash != std::string::npos) ? file_path.substr(0, slash) : ".";
    }
    cached_core = std::string(msl_prologue(kProductionCompileVariant)) +
                  "#include <metal_stdlib>\nusing namespace metal;\n" +
                  stitch(dir_str, {"../physics_core/platform.hpp", "../physics_core/status.hpp",
                                   "../physics_core/capacity.hpp", "../physics_core/scalar.hpp",
                                   "../physics_core/toolkit.hpp", "../physics_core/gpu_toolkit.hpp"});
    return cached_core.c_str();
}
```

- [ ] **Step 2.7: Add the MSL smoke for `GpuToolkit` to `test_metal_math_mode.cpp`.** Append before `main` and call `run_core_smoke()` at the end of `main` (fail → return 1).

```cpp
extern "C" const char *bike_metal_get_core_source(const char *gpu_src_dir);
#include "../../src/physics_core/gpu_toolkit.hpp"

namespace {
// One thread per case: x in, eight GpuToolkit results out. Host float runs
// the same functions from the same header.
constexpr const char *kCoreKernel = R"MSL(
kernel void bike_core_smoke(device const float *in [[buffer(0)]],
                            device float *out [[buffer(1)]],
                            uint gid [[thread_position_in_grid]]) {
    using bike::physics_core::GpuToolkit;
    const float x = in[gid];
    float a[3] = {x, x * 0.5f - 1.0f, 2.0f - x};
    float b[3] = {0.25f, -x, x * x};
    float m[6] = {x, 1.0f, -x, 0.5f, x * 3.0f, -2.0f};
    float y3[3]; float y2[2]; float pts[3];
    GpuToolkit::gemv_n3(2, m, b, y3);
    GpuToolkit::gemv_t3(2, m, a, y2);
    GpuToolkit::ring_lift(1, in + gid, b, pts);   // rows in[gid..gid+2]; the buffer has 2 pad floats
    device float *o = out + 8 * gid;
    o[0] = GpuToolkit::dot(3, a, b);
    o[1] = y3[0] + y3[1] + y3[2];
    o[2] = y2[0] - y2[1];
    o[3] = GpuToolkit::exp(-GpuToolkit::abs(x) / 4.5f);
    o[4] = GpuToolkit::div(x, 3.0f + x * x);
    o[5] = GpuToolkit::sqrt(x * x + 1.0f);
    o[6] = GpuToolkit::interp_segment(x, 0.75f, 0.25f, -0.125f, 1.5f, 2.0f);
    o[7] = pts[0];
}
)MSL";

bool run_core_smoke() {
    using bike::physics_core::GpuToolkit;
    std::vector<float> xs;
    for (int i = -400; i <= 400; ++i) xs.push_back(static_cast<float>(i) * 0.0371f);
    const std::size_t cases = xs.size();
    xs.push_back(0.0f);   // two pad floats: thread g reads the ring row xs[g..g+2]
    xs.push_back(0.0f);
    std::vector<float> dev(cases * 8, 0.0f), host(cases * 8, 0.0f);
    for (std::size_t g = 0; g < cases; ++g) {
        const float x = xs[g];
        float a[3] = {x, x * 0.5f - 1.0f, 2.0f - x};
        float b[3] = {0.25f, -x, x * x};
        float m[6] = {x, 1.0f, -x, 0.5f, x * 3.0f, -2.0f};
        float y3[3]; float y2[2]; float pts[3]; const float ring[3] = {xs[g], xs[g + 1], xs[g + 2]};
        GpuToolkit::gemv_n3(2, m, b, y3);
        GpuToolkit::gemv_t3(2, m, a, y2);
        GpuToolkit::ring_lift(1, ring, b, pts);
        const std::array<float, 8> r = {GpuToolkit::dot(3, a, b), y3[0] + y3[1] + y3[2],
                                        y2[0] - y2[1], GpuToolkit::exp(-GpuToolkit::abs(x) / 4.5f),
                                        GpuToolkit::div(x, 3.0f + x * x), GpuToolkit::sqrt(x * x + 1.0f),
                                        GpuToolkit::interp_segment(x, 0.75f, 0.25f, -0.125f, 1.5f, 2.0f), pts[0]};
        std::copy(r.begin(), r.end(), host.begin() + static_cast<std::ptrdiff_t>(8 * g));
    }
    const std::string source = std::string(bike_metal_get_core_source(nullptr)) + kCoreKernel;
    std::array<char, 4096> err{};
    const int rc = bike_metal_run_probe(source.c_str(), "bike_core_smoke", -1, xs.data(),
                                        xs.size() * sizeof(float), dev.data(),
                                        dev.size() * sizeof(float), static_cast<int>(cases),
                                        err.data(), err.size());
    if (rc != 0) {
        std::cerr << "FAIL core smoke rc=" << rc << ": " << err.data() << '\n';
        return false;
    }
    int bad = 0;
    for (std::size_t i = 0; i < dev.size(); ++i) bad += same(dev[i], host[i]) ? 0 : 1;
    std::cout << "core smoke mismatches: " << bad << " of " << dev.size() << '\n';
    return bad == 0;
}
} // namespace
```

`ring_lift`'s table argument in the MSL kernel is a `device` pointer (`in + gid`), because `BIKE_CORE_TABLE` is `device` there. The host passes the same three floats as a plain array. In the host-only test file, `ring[3]` triggers `-Wunsafe-buffer-usage` only through the header's pointer arithmetic, and the header suppresses that. Add `#include <algorithm>` to the test includes. In D19 fallback mode (Task 1), tolerate up to 1 ulp here with the same `ulps` helper and note it in the audit file.

- [ ] **Step 2.8: Run the new tests and the header sweep.** Run:
```bash
cmake --build build/native-release --target physics_core_tests bike_test_metal_math_mode && \
  ctest --test-dir build/native-release -R '^(physics_core_tests|bike_test_metal_math_mode)$' --output-on-failure
cmake --build build/native-release --target check_headers
```
Expected: both tests `Passed`; `check_headers` exits 0. The sweep compiles `cpu_exact_toolkit.hpp` standalone, and `BIKE_NUMPY_INTERP_CONTRACTED` is directory-wide.

- [ ] **Step 2.9: Run G3 for S7.** Run `ctest --test-dir build/native-release -L gpu --output-on-failure`. Expected: all pass. Any failure → S7.

- [ ] **Step 2.9a: Register the suppressions.** The new headers carry the GPU-kernel NOLINT set and `BIKE_GPU_SHARED_CODE_BEGIN`. Add their paths to the existing register row, `docs/TESTING.md:1062`. In the row's file column, append `` `native/src/physics_core/{platform,status,capacity,scalar,toolkit,gpu_toolkit}.hpp` `` after the `gpu/` list. Tasks 3–8 extend the brace list as each header lands: `tyre_station`, `terrain`, `tyre_bristle`, `kinematics`, `tyre`. In the verification column, append `` `physics_core_tests` ``. `cpu_exact_toolkit.hpp` and `host_status.hpp` carry no suppression and are not listed.

- [ ] **Step 2.10: Commit.**
```bash
git add native/src/physics_core native/src/gpu/common.hpp docs/TESTING.md \
  native/src/gpu/metal_batch_runner.mm native/tests/physics_core native/tests/gpu/test_metal_math_mode.cpp CMakeLists.txt
git commit -m "feat(physics_core): add platform macros, status, toolkits

GpuToolkit (float, plain loops, portable exp) and CpuExactToolkit
(BLAS/numeric forwarders) are the two D9 primitive sets; the MSL smoke
proves GpuToolkit is bitwise equal on Metal and host float.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---
## Frozen-copy protocol (Tasks 3–7)

Every extraction keeps a verbatim copy of the pre-extraction function as the bitwise oracle of its unit test. The base for every frozen copy is the plan's HEAD, `b25120af`.

**Precondition, checked once before Task 3.** Run:
```bash
git diff --stat b25120af -- native/src/tyre native/src/contact/laws.hpp native/src/writers/tire_common.cpp native/src/writers/tire_common.hpp native/src/writers/distributed_tire.cpp native/src/writers/distributed_tire.hpp
```
Expected: empty output. Stage 0 must not have touched these files. If the output is not empty, re-verify every line anchor of Tasks 3–7 against the new HEAD before continuing. Use the new HEAD as the frozen base, not `b25120af`.

Frozen code goes into `native/tests/physics_core/frozen/*.hpp`, in a `namespace frozen_<law>`:
- Copy it with `git show b25120af:<path> | sed -n '<A>,<B>p'`.
- Rename only the enclosing class or namespace, as each task states.
- These files are test inputs and are never edited afterwards. Header comment: `// Frozen verbatim copy of <path>:<A>-<B> at b25120af — the D2 oracle of test_physics_core.cpp. Never edit.`

---

## Task 3: Station law (`tyre_station.hpp`)

**Files:**
- Create: `native/src/physics_core/tyre_station.hpp`
- Create: `native/tests/physics_core/frozen/station.hpp`
- Modify: `native/src/tyre/station.cpp:52-95` (bodies become wrappers)
- Modify: `native/tests/physics_core/test_physics_core.cpp`

**Interfaces (produced):**
```cpp
namespace bike::physics_core {
template <class Real> struct HingeDensityView {
    const BIKE_THREAD Real *knots_m;
    const BIKE_THREAD Real *stiffness_n_m;
    int knot_count;            // the zip bound min(|knots|, |stiffness|)
    Real damping_ns_m;
};
template <class Real> struct StationDensity { Real force; Real energy; };
template <class Real> struct StationResponse { Real load; Real energy; };
template <class Real, class TK>
BIKE_HD Status station_density(Real delta, const BIKE_THREAD HingeDensityView<Real> &d,
                               BIKE_THREAD StationDensity<Real> &out) noexcept;
template <class Real, class TK>
BIKE_HD Status station_response(Real delta, Real rate, Real weight,
                                const BIKE_THREAD HingeDensityView<Real> &d,
                                BIKE_THREAD StationResponse<Real> &out) noexcept;
}
```
Why `BIKE_THREAD`: on the GPU the knots live in `GpuTyreParams`, inside the per-world `GpuWorldConfig`. The Metal scalar path copies that config into thread memory (`metal_batch_runner.mm:258`).

- [ ] **Step 3.1: Freeze the oracle.** Run:
```bash
mkdir -p native/tests/physics_core/frozen
{ printf '%s\n' '// Frozen verbatim copy of native/src/tyre/station.cpp:52-95 at b25120af — the D2 oracle of test_physics_core.cpp. Never edit.' \
  '#pragma once' '#include "../../../src/config_types.hpp"' '#include <algorithm>' '#include <cmath>' '#include <stdexcept>' '#include <utility>' \
  'namespace frozen_station {'; \
  git show b25120af:native/src/tyre/station.cpp | sed -n '52,95p'; printf '%s\n' '} // namespace frozen_station'; } > native/tests/physics_core/frozen/station.hpp
```
In that file:
- prefix both function definitions with `inline`;
- delete the trailing `} // namespace biketyre` line that `sed` copied (line 95);
- change nothing else.

- [ ] **Step 3.2: Write the failing differential test.** In `test_physics_core.cpp` add `#include "frozen/station.hpp"`, `#include "../../src/physics_core/tyre_station.hpp"` and `#include "../../src/tyre/station.hpp"`, plus:

```cpp
    // A deterministic corpus over the hinge law: knots straddling delta,
    // negative/zero/positive delta, damped rates of both signs, and the
    // domain errors (non-finite delta, invalid quadrature, negative weight).
    std::vector<nativecfg::HingeDensity> station_materials() {
        return {
            {.knots_m = {0., .004, .012}, .stiffness_n_m = {300000., 0., 0.},
             .damping_ns_m = 2000., .provenance = "fixture"},
            {.knots_m = {0., .008, .018}, .stiffness_n_m = {55000., 85000., 140000.},
             .damping_ns_m = 420., .provenance = "gpu-default"},
            {.knots_m = {-.001, .0021, .0033, .007}, .stiffness_n_m = {1e5, 3.3e4, 7.7e4, 1.9e5},
             .damping_ns_m = 0., .provenance = "four-knot"},
        };
    }

    void test_station_law_frozen() {
        std::mt19937_64 rng(20261103u);
        std::uniform_real_distribution<double> delta(-.01, .03), rate(-3., 3.), weight(0., .2);
        for (const nativecfg::HingeDensity &mat : station_materials()) {
            for (int i = 0; i < 4096; ++i) {
                const double dl = i % 97 == 0 ? mat.knots_m[static_cast<std::size_t>(i) % mat.knots_m.size()] : delta(rng);
                const double rt = rate(rng), w = weight(rng);
                const auto want_d = frozen_station::station_density(dl, mat);
                const auto got_d = biketyre::station_density(dl, mat);
                require(same_bits(want_d.first, got_d.first) && same_bits(want_d.second, got_d.second),
                        "station_density bits");
                const auto want_r = frozen_station::station_response(dl, rt, w, mat);
                const auto got_r = biketyre::station_response(dl, rt, w, mat);
                require(same_bits(want_r.first, got_r.first) && same_bits(want_r.second, got_r.second),
                        "station_response bits");
            }
            const double nan = std::numeric_limits<double>::quiet_NaN();
            for (const double bad : {nan, std::numeric_limits<double>::infinity()}) {
                require_same_throw([&] { (void)frozen_station::station_density(bad, mat); },
                                   [&] { (void)biketyre::station_density(bad, mat); },
                                   "station_density domain error");
                require_same_throw([&] { (void)frozen_station::station_response(.001, bad, .1, mat); },
                                   [&] { (void)biketyre::station_response(.001, bad, .1, mat); },
                                   "station_response quadrature error");
            }
            require_same_throw([&] { (void)frozen_station::station_response(.001, 0., -.1, mat); },
                               [&] { (void)biketyre::station_response(.001, 0., -.1, mat); },
                               "station_response negative weight");
        }
    }
```
Add the shared helper once, above the first test. Later tasks reuse it.
```cpp
    // Both callables must throw the same exception type with the same
    // message (D13: statuses map back to today's exceptions exactly).
    template <class A, class B>
    void require_same_throw(const A &want, const B &got, std::string_view what) {
        const auto capture = [](const auto &f) -> std::pair<std::string, std::string> {
            try { f(); } catch (const std::overflow_error &e) { return {"overflow_error", e.what()}; }
            catch (const std::invalid_argument &e) { return {"invalid_argument", e.what()}; }
            catch (const std::logic_error &e) { return {"logic_error", e.what()}; }
            catch (const std::exception &e) { return {"exception", e.what()}; }
            return {"none", ""};
        };
        const auto w = capture(want);
        const auto g = capture(got);
        require(w.first != "none", std::string(what) + ": oracle did not throw");
        require(w == g, std::string(what) + ": got " + g.first + " '" + g.second + "', want " +
                            w.first + " '" + w.second + "'");
    }
```
`overflow_error` is caught before `invalid_argument` and `logic_error`. That order is deliberate: `overflow_error` derives from `runtime_error`, and `invalid_argument` derives from `logic_error`. Register `{.name = "station_law_frozen", .run = test_station_law_frozen}`.

- [ ] **Step 3.3: Run to verify it fails.** Run `cmake --build build/native-release --target physics_core_tests`. Expected: `fatal error: '../../src/physics_core/tyre_station.hpp' file not found`.

- [ ] **Step 3.4: Create `physics_core/tyre_station.hpp`.** The NOLINT and `BIKE_GPU_SHARED_CODE` wrapper lines from Step 2.4 go around the namespace.

```cpp
// physics_core/tyre_station.hpp — distributed_tire.py:125-148 station law,
// moved verbatim from native/src/tyre/station.cpp:52-95 (D9). The knot
// fold is sequential; every literal is Real(k) or Lit<Real>.
#pragma once
#include "platform.hpp"
#include "scalar.hpp"
#include "status.hpp"
#include "toolkit.hpp"

namespace bike::physics_core {

template <class Real> struct HingeDensityView {
    const BIKE_THREAD Real *knots_m;
    const BIKE_THREAD Real *stiffness_n_m;
    int knot_count;
    Real damping_ns_m;
};
template <class Real> struct StationDensity { Real force; Real energy; };
template <class Real> struct StationResponse { Real load; Real energy; };

template <class Real, class TK>
BIKE_HD Status station_density(Real delta, const BIKE_THREAD HingeDensityView<Real> &d,
                               BIKE_THREAD StationDensity<Real> &out) noexcept {
    BIKE_CORE_TOOLKIT_CHECK(Real, TK);
    if (!TK::isfinite(delta)) {
        return Status::station_penetration_nonfinite;
    }
    Real force = Real(0);
    Real energy = Real(0);
    for (int i = 0; i < d.knot_count; ++i) {
        const Real x = max_std(delta - d.knots_m[i], Real(0));
        force += d.stiffness_n_m[i] * x;
        energy += d.stiffness_n_m[i] * x * x;
    }
    out.force = force;
    out.energy = Lit<Real>::half() * energy;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status station_response(Real delta, Real rate, Real weight,
                                const BIKE_THREAD HingeDensityView<Real> &d,
                                BIKE_THREAD StationResponse<Real> &out) noexcept {
    BIKE_CORE_TOOLKIT_CHECK(Real, TK);
    if (!(TK::isfinite(delta) && TK::isfinite(rate) && TK::isfinite(weight)) ||
        weight < Real(0)) {
        return Status::station_invalid_quadrature;
    }
    StationDensity<Real> dens{};
    const Status s = station_density<Real, TK>(delta, d, dens);
    if (s != Status::ok) {
        return s;
    }
    const Real force = delta > Real(0) ? max_std(Real(0), dens.force + d.damping_ns_m * rate)
                                       : Real(0);
    out.load = weight * force;
    out.energy = weight * dens.energy;
    return Status::ok;
}

} // namespace bike::physics_core
```

- [ ] **Step 3.5: Make `station.cpp` a wrapper.** Replace the bodies at `station.cpp:52-95` with the following, keeping both signatures and the NOLINT lines around `station_response`. Add the includes `#include "../physics_core/cpu_exact_toolkit.hpp"`, `#include "../physics_core/host_status.hpp"`, `#include "../physics_core/tyre_station.hpp"` and `#include <limits>`.

```cpp
    namespace {
        namespace pc = bike::physics_core;
        // zip(knots_m, stiffness_n_m) truncates to the shorter column.
        [[nodiscard]] pc::HingeDensityView<double> density_view(const nativecfg::HingeDensity &d) {
            const std::size_t knots = std::min(d.knots_m.size(), d.stiffness_n_m.size());
            if (knots > static_cast<std::size_t>(std::numeric_limits<int>::max()))
                throw std::invalid_argument("hinge density knot table exceeds int range");
            return {.knots_m = d.knots_m.data(), .stiffness_n_m = d.stiffness_n_m.data(),
                    .knot_count = static_cast<int>(knots), .damping_ns_m = d.damping_ns_m};
        }
    } // namespace

    std::pair<double, double>
    station_density(double delta, const nativecfg::HingeDensity &density) {
        // distributed_tire.py:131-139 — physics_core::station_density.
        pc::StationDensity<double> out{};
        const pc::HingeDensityView<double> view = density_view(density);
        if (const pc::Status s = pc::station_density<double, pc::CpuExactToolkit>(delta, view, out);
            s != pc::Status::ok)
            throw_core_status(s);
        return {out.force, out.energy};
    }

    std::pair<double, double>
    station_response(double delta, double rate, double weight,
                     const nativecfg::HingeDensity &density) {
        // distributed_tire.py:144-148 — physics_core::station_response.
        pc::StationResponse<double> out{};
        const pc::HingeDensityView<double> view = density_view(density);
        if (const pc::Status s = pc::station_response<double, pc::CpuExactToolkit>(
                delta, rate, weight, view, out);
            s != pc::Status::ok)
            throw_core_status(s);
        return {out.load, out.energy};
    }
```
`station.cpp` is also `#include`d by `native/tests/gpu/test_gpu_tire.cpp:10`. That test is deleted in Task 11. Until then it must still compile, and it does: the new includes are relative to `station.cpp`.

- [ ] **Step 3.6: Run the unit tests.**
```bash
cmake --build build/native-release --target physics_core_tests native_station_tests && \
  ctest --test-dir build/native-release -R '^(physics_core_tests|native_station_tests)$' --output-on-failure
```
Expected: `100% tests passed, 0 tests failed out of 2`. A bit difference → **S2**.

- [ ] **Step 3.7: Run G1, then G3 and G4.** G1 as written in "Standard gates". Expected: as stated there. A failure → **S1**. Then run `ctest --test-dir build/native-release -L gpu --output-on-failure` and `cmake --build build/native-release --target check_headers`. Expected: both pass.

- [ ] **Step 3.8: Commit.**
```bash
git add native/src/physics_core/tyre_station.hpp native/src/tyre/station.cpp native/tests/physics_core
git commit -m "refactor(tyre): move the station law into physics_core

station_density/station_response become <Real, TK> templates returning a
Status; biketyre keeps its signatures and exceptions (D2, D13), proven by
a frozen-copy bitwise test and the golden digest.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Task 4: Terrain closest-point query (`terrain.hpp`)

**Files:**
- Create: `native/src/physics_core/terrain.hpp`
- Create: `native/tests/physics_core/frozen/terrain.hpp`
- Modify:
  - `native/src/tyre/distance.hpp:48-85`: add `view()`; remove `Candidate` and `candidate`.
  - `native/src/tyre/distance.cpp:33-49`: remove the anonymous-namespace helpers.
  - `native/src/tyre/distance.cpp:105-258`: `height` and `query` become wrappers; delete `candidate`.
  - `native/src/tyre/profile.cpp:98-140`: `np_interp` keeps its boundary check, then calls the core.
- Modify: `native/tests/physics_core/test_physics_core.cpp`

**Interfaces (produced):**
```cpp
namespace bike::physics_core {
template <class Real> struct PolylineView {
    const BIKE_CORE_TABLE Real *x, *z;                  // vertex_count each
    const BIKE_CORE_TABLE Real *edge_x, *edge_z;        // vertex_count - 1 each
    const BIKE_CORE_TABLE Real *length2;
    const BIKE_CORE_TABLE Real *normal_x, *normal_z;
    int vertex_count;                                   // >= 2
};
template <class Real> struct PointDistance { Real distance; Real normal[2]; Real point[2]; int segment; bool nonsmooth; };
template <class Real> BIKE_HD int search_right(const BIKE_CORE_TABLE Real *x, int n, Real v) noexcept;
template <class Real> BIKE_HD int search_left(const BIKE_CORE_TABLE Real *x, int n, Real v) noexcept;
template <class Real, class TK> BIKE_HD Real np_interp(Real xq, const BIKE_CORE_TABLE Real *xs, const BIKE_CORE_TABLE Real *ys, int n) noexcept;
template <class Real, class TK> BIKE_HD Status terrain_height(const BIKE_THREAD PolylineView<Real> &pl, Real x, BIKE_THREAD Real &out) noexcept;
template <class Real, class TK> BIKE_HD Status terrain_query(const BIKE_THREAD PolylineView<Real> &pl, Real px, Real pz, BIKE_THREAD PointDistance<Real> &out) noexcept;
}
// biketyre::SignedProfile gains:
[[nodiscard]] bike::physics_core::PolylineView<double> view() const noexcept;
```
D11: building the polyline stays in `SignedProfile`'s constructor (`distance.cpp:52-103`), and the GPU reads exactly these arrays (Task 9).

- [ ] **Step 4.1: Freeze the oracle.** Create `native/tests/physics_core/frozen/terrain.hpp` from three sources, in this order:
  - `git show b25120af:native/src/tyre/distance.hpp | sed -n '36,85p'`: `PointDistance` and the class.
  - `git show b25120af:native/src/tyre/profile.cpp | sed -n '98,140p'`: `np_interp`.
  - `git show b25120af:native/src/tyre/distance.cpp | sed -n '33,258p'`: helpers and members.

  Wrap everything in `namespace frozen_terrain { … }`, with these edits:
  - replace each `namespace biketyre {` / `} // namespace biketyre` pair by nothing (the outer namespace replaces it);
  - rename `SignedProfile` to `Profile` (`sed -i '' 's/SignedProfile/Profile/g'`);
  - mark every out-of-class definition `inline`;
  - turn the anonymous namespace into `namespace detail` and qualify its three uses (`detail::search_right`, `detail::search_left`, `detail::clip01`).

  Includes: `../../../src/cblas_abi.hpp`, `../../../src/numeric_numpy.hpp`, `<algorithm>`, `<array>`, `<cmath>`, `<span>`, `<stdexcept>`, `<utility>`, `<vector>`. Change nothing else.

- [ ] **Step 4.2: Write the failing differential test.**

```cpp
    // Polylines that exercise the window logic: a smooth ramp, a kinked
    // ridge (nonsmooth vertices), a long flat run (ties, constant-segment
    // interp), and a dense random walk.
    std::vector<std::vector<std::array<double, 2>>> terrain_polylines() {
        std::vector<std::vector<std::array<double, 2>>> out;
        out.push_back({{-5., 0.}, {0., 0.}, {5., 1.25}, {12., 1.25}});
        out.push_back({{0., 0.}, {1., 1.}, {2., 0.}, {3., 1.}, {4., 0.}});
        out.push_back({{-1., .2}, {0., .2}, {1., .2}, {2., .2}, {3., .2}});
        std::mt19937_64 rng(42);
        std::uniform_real_distribution<double> step(.01, .3), dz(-.05, .05);
        std::vector<std::array<double, 2>> walk{{-2., 0.}};
        for (int i = 0; i < 400; ++i)
            walk.push_back({walk.back()[0] + step(rng), walk.back()[1] + dz(rng)});
        out.push_back(walk);
        return out;
    }

    void test_terrain_frozen() {
        std::mt19937_64 rng(20261103u);
        for (const auto &verts : terrain_polylines()) {
            const frozen_terrain::Profile want(verts);
            const biketyre::SignedProfile got(verts);
            const double x0 = verts.front()[0], x1 = verts.back()[0];
            std::uniform_real_distribution<double> ux(x0, x1), uz(-.5, 1.5);
            std::vector<std::array<double, 2>> queries;
            for (int i = 0; i < 8192; ++i) queries.push_back({ux(rng), uz(rng)});
            for (const auto &v : verts) queries.push_back(v);                   // on vertices
            for (std::size_t i = 0; i + 1 < verts.size(); ++i)                  // on segment midpoints
                queries.push_back({.5 * (verts[i][0] + verts[i + 1][0]), .5 * (verts[i][1] + verts[i + 1][1])});
            for (const auto &q : queries) {
                require(same_bits(want.height(q[0]), got.height(q[0])), "height bits");
                const auto a = want.query(q);
                const auto b = got.query(q);
                require(same_bits(a.distance, b.distance) && same_bits(a.normal[0], b.normal[0]) &&
                            same_bits(a.normal[1], b.normal[1]) && same_bits(a.point[0], b.point[0]) &&
                            same_bits(a.point[1], b.point[1]) && a.segment == b.segment &&
                            a.nonsmooth == b.nonsmooth,
                        "query bits");
            }
            const double nan = std::numeric_limits<double>::quiet_NaN();
            require_same_throw([&] { (void)want.query({nan, 0.}); }, [&] { (void)got.query({nan, 0.}); },
                               "query non-finite");
            require_same_throw([&] { (void)want.query({x1 + 1., 0.}); }, [&] { (void)got.query({x1 + 1., 0.}); },
                               "query outside domain");
            require_same_throw([&] { (void)want.height(x0 - 1.); }, [&] { (void)got.height(x0 - 1.); },
                               "height outside domain");
        }
        // np_interp: the exported table path, including NaN query and the
        // constant-segment fallback.
        const std::vector<double> xs{0., 1., 2., 4.}, ys{1., 1., -3., 5.};
        for (const double q : {-1., 0., .5, 1., 1.5, 2., 3.999, 4., 9., std::numeric_limits<double>::quiet_NaN()}) {
            const double a = frozen_terrain::np_interp(q, xs, ys), b = biketyre::np_interp(q, xs, ys);
            require(same_bits(a, b) || (std::isnan(a) && std::isnan(b)), "np_interp bits");
        }
        require_same_throw([&] { (void)frozen_terrain::np_interp(0., {}, {}); },
                           [&] { (void)biketyre::np_interp(0., {}, {}); }, "np_interp empty table");
    }
```
Register `terrain_frozen`. Add includes `frozen/terrain.hpp`, `../../src/physics_core/terrain.hpp`, `../../src/tyre/distance.hpp` and `../../src/tyre/profile.hpp`. Check the frozen `np_interp` declaration: it takes `std::span<const double>`. `{}` converts to an empty span, as in the CPU header.

- [ ] **Step 4.3: Run to verify it fails.** `cmake --build build/native-release --target physics_core_tests`. Expected: `fatal error: '../../src/physics_core/terrain.hpp' file not found` (the test includes it for a `static_assert` placed after the cases: `static_assert(sizeof(pc::PointDistance<float>) > 0);`).

- [ ] **Step 4.4: Create `physics_core/terrain.hpp`.** Each statement mirrors the cited `distance.cpp` / `profile.cpp` line.

```cpp
// physics_core/terrain.hpp — profile_distance.py SignedProfile height and
// closest-point query, moved from native/src/tyre/distance.cpp:33-258 and
// the np.interp port of native/src/tyre/profile.cpp:98-140 (D9, D11). The
// polyline is built by biketyre::SignedProfile on the CPU; this file only
// reads it.
#pragma once
#include "platform.hpp"
#include "scalar.hpp"
#include "status.hpp"
#include "toolkit.hpp"

namespace bike::physics_core {

template <class Real> struct PolylineView {
    const BIKE_CORE_TABLE Real *x;
    const BIKE_CORE_TABLE Real *z;
    const BIKE_CORE_TABLE Real *edge_x;
    const BIKE_CORE_TABLE Real *edge_z;
    const BIKE_CORE_TABLE Real *length2;
    const BIKE_CORE_TABLE Real *normal_x;
    const BIKE_CORE_TABLE Real *normal_z;
    int vertex_count;
};

template <class Real> struct PointDistance {
    Real distance;
    Real normal[2];
    Real point[2];
    int segment;
    bool nonsmooth;
};

// std::ranges::upper_bound with the same probe sequence and predicate
// (v < x[mid]): np.searchsorted(x, v, 'right'). NaN v runs to n, as before.
template <class Real>
BIKE_HD int search_right(const BIKE_CORE_TABLE Real *x, int n, Real v) noexcept {
    int first = 0;
    int count = n;
    while (count > 0) {
        const int step = count / 2;
        const int mid = first + step;
        if (!(v < x[mid])) {
            first = mid + 1;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    return first;
}

// std::ranges::lower_bound (x[mid] < v): np.searchsorted(x, v, 'left').
template <class Real>
BIKE_HD int search_left(const BIKE_CORE_TABLE Real *x, int n, Real v) noexcept {
    int first = 0;
    int count = n;
    while (count > 0) {
        const int step = count / 2;
        const int mid = first + step;
        if (x[mid] < v) {
            first = mid + 1;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    return first;
}

// compiled_base.c np_interp, lenxp > 1 (profile.cpp:111-139). The segment
// evaluation with its retry is the toolkit's interp_segment
// (CPU: numeric::np_interp_segment — the single implementation).
template <class Real, class TK>
BIKE_HD Real np_interp(Real xq, const BIKE_CORE_TABLE Real *xs, const BIKE_CORE_TABLE Real *ys,
                       int n) noexcept {
    BIKE_CORE_TOOLKIT_CHECK(Real, TK);
    const Real lval = ys[0];
    const Real rval = ys[n - 1];
    int j = 0;
    if (xq < xs[0]) {
        j = -1;
    } else if (xq > xs[n - 1]) {
        j = n;
    } else {
        j = search_right<Real>(xs, n, xq) - 1;
    }
    if (j == -1) {
        return lval;
    }
    if (j == n) {
        return rval;
    }
    if (j == n - 1) {
        return rval;
    }
    if (xs[j] == xq) {
        return ys[j];
    }
    const Real slope = TK::div(ys[j + 1] - ys[j], xs[j + 1] - xs[j]);
    return TK::interp_segment(slope, xq, xs[j], ys[j], xs[j + 1], ys[j + 1]);
}

// SignedProfile::height (distance.cpp:105-112).
template <class Real, class TK>
BIKE_HD Status terrain_height(const BIKE_THREAD PolylineView<Real> &pl, Real x,
                              BIKE_THREAD Real &out) noexcept {
    if (!TK::isfinite(x) || x < pl.x[0] || x > pl.x[pl.vertex_count - 1]) {
        return Status::terrain_outside_domain;
    }
    out = np_interp<Real, TK>(x, pl.x, pl.z, pl.vertex_count);
    return Status::ok;
}

template <class Real> struct SegmentCandidate {
    Real fraction;
    Real projection[2];
    Real vector[2];
    Real squared;
};

// SignedProfile::candidate (distance.cpp:114-143): +0.0-seeded folds,
// unfused multiply-then-add per component.
template <class Real, class TK>
BIKE_HD SegmentCandidate<Real> terrain_candidate(const BIKE_THREAD PolylineView<Real> &pl, int i,
                                                 Real px, Real pz) noexcept {
    const Real ox = px - pl.x[i];
    const Real oz = pz - pl.z[i];
    Real num = Real(0);
    num += ox * pl.edge_x[i];
    num += oz * pl.edge_z[i];
    SegmentCandidate<Real> c{};
    c.fraction = clip01(TK::div(num, pl.length2[i]));
    c.projection[0] = pl.x[i] + c.fraction * pl.edge_x[i];
    c.projection[1] = pl.z[i] + c.fraction * pl.edge_z[i];
    c.vector[0] = px - c.projection[0];
    c.vector[1] = pz - c.projection[1];
    Real squared = Real(0);
    squared += c.vector[0] * c.vector[0];
    squared += c.vector[1] * c.vector[1];
    c.squared = squared;
    return c;
}

// SignedProfile::query (distance.cpp:145-258).
template <class Real, class TK>
BIKE_HD Status terrain_query(const BIKE_THREAD PolylineView<Real> &pl, Real px, Real pz,
                             BIKE_THREAD PointDistance<Real> &out) noexcept {
    BIKE_CORE_TOOLKIT_CHECK(Real, TK);
    if (!TK::isfinite(px) || !TK::isfinite(pz)) {
        return Status::terrain_query_nonfinite;
    }
    Real h = Real(0);
    const Status hs = terrain_height<Real, TK>(pl, px, h);
    if (hs != Status::ok) {
        return hs;
    }
    const int n = pl.vertex_count;
    const int nseg = n - 1;
    const int first = clamp_int(search_right<Real>(pl.x, n, px) - 1, 0, nseg - 1);
    const Real edge[2] = {pl.edge_x[first], pl.edge_z[first]};
    const Real offset[2] = {px - pl.x[first], pz - pl.z[first]};
    const Real t = clip01(TK::div(TK::dot(2, offset, edge), pl.length2[first]));
    const Real projected[2] = {pl.x[first] + t * edge[0], pl.z[first] + t * edge[1]};
    const Real upper_d[2] = {px - projected[0], pz - projected[1]};
    const Real upper = TK::sqrt(TK::dot(2, upper_d, upper_d));
    const int left = max_std(0, search_left<Real>(pl.x, n, px - upper) - 1);
    const int right = min_std(nseg, search_right<Real>(pl.x, n, px + upper));
    const int hi = max_std(left + 1, right);
    Real best = terrain_candidate<Real, TK>(pl, left, px, pz).squared;
    for (int i = left + 1; i < hi; ++i) {
        best = min_std(best, terrain_candidate<Real, TK>(pl, i, px, pz).squared);
    }
    const Real bound = best + Lit<Real>::tie_abs() * max_std(Real(1), best);
    int sid = -1;
    SegmentCandidate<Real> winner{};
    bool nonsmooth = false;
    for (int i = left; i < hi; ++i) {
        const SegmentCandidate<Real> row = terrain_candidate<Real, TK>(pl, i, px, pz);
        if (row.squared <= bound) {
            if (sid < 0) {
                sid = i;
                winner = row;
            }
            const Real dnx = pl.normal_x[i] - pl.normal_x[sid];
            const Real dnz = pl.normal_z[i] - pl.normal_z[sid];
            if (TK::sqrt(dnx * dnx + dnz * dnz) > Lit<Real>::normal_tol()) {
                nonsmooth = true;
            }
        }
    }
    if (sid < 0) {
        return Status::terrain_no_closest_segment;
    }
    const Real length = TK::sqrt(winner.squared);
    const Real sign = pz >= h ? Real(1) : Real(-1);
    out.distance = sign * length;
    if (length > Lit<Real>::length_floor()) {
        out.normal[0] = TK::div(sign * winner.vector[0], length);
        out.normal[1] = TK::div(sign * winner.vector[1], length);
    } else {
        out.normal[0] = pl.normal_x[sid];
        out.normal[1] = pl.normal_z[sid];
    }
    out.point[0] = winner.projection[0];
    out.point[1] = winner.projection[1];
    out.segment = sid;
    out.nonsmooth = nonsmooth;
    if (length <= Lit<Real>::length_floor() &&
        (winner.fraction <= Lit<Real>::length_floor() ||
         winner.fraction >= Lit<Real>::one_minus_floor())) {
        const int vertex = winner.fraction <= Lit<Real>::length_floor() ? sid : sid + 1;
        if (0 < vertex && vertex < n - 1) {
            const Real d[2] = {pl.normal_x[vertex - 1] - pl.normal_x[vertex],
                               pl.normal_z[vertex - 1] - pl.normal_z[vertex]};
            if (TK::sqrt(TK::dot(2, d, d)) > Lit<Real>::normal_tol()) {
                out.nonsmooth = true;
            }
        }
    }
    return Status::ok;
}

} // namespace bike::physics_core
```
Equivalence notes, checked against `distance.cpp`:
- `std::clamp(v, 0, nseg-1)` is `clamp_int`.
- `std::max(0, a)`, `std::min(nseg, b)`, `std::max(left+1, right)`, `std::min(best, c)` and `std::max(1.0, best)` all have `max_std`/`min_std` argument order identical to the libc++ predicates.
- `TK::dot(2, …)` replaces each `blas::ddot(2, …)` (:166, :174, :250). The `0.0`-seeded folds of `candidate` stay plain code, exactly as today.

- [ ] **Step 4.5: Rewire `SignedProfile`.**

  In `distance.hpp`:
  - add `#include "../physics_core/terrain.hpp"`;
  - delete the `Candidate` struct and `candidate` declaration (:66-79);
  - add, public, after `query`:
```cpp
        // The core's read-only view of the compiled polyline (D11): the
        // CPU query/height below and the GPU upload read the same arrays.
        [[nodiscard]] bike::physics_core::PolylineView<double> view() const noexcept {
            return {.x = x_.data(), .z = z_.data(), .edge_x = edge_x_.data(),
                    .edge_z = edge_z_.data(), .length2 = length2_.data(),
                    .normal_x = normal_x_.data(), .normal_z = normal_z_.data(),
                    .vertex_count = static_cast<int>(x_.size())};
        }
```
  In `distance.cpp`:
  - delete the anonymous namespace (:33-49) and `candidate` (:114-143);
  - add the includes `../physics_core/cpu_exact_toolkit.hpp` and `../physics_core/host_status.hpp`;
  - replace `height` and `query` with:
```cpp
    double SignedProfile::height(double x) const {
        // profile_distance.py:34-38 — physics_core::terrain_height.
        namespace pc = bike::physics_core;
        double h = 0.0;
        if (const pc::Status s = pc::terrain_height<double, pc::CpuExactToolkit>(view(), x, h);
            s != pc::Status::ok)
            throw_core_status(s);
        return h;
    }

    PointDistance SignedProfile::query(std::array<double, 2> p) const {
        // profile_distance.py:40-75 — physics_core::terrain_query.
        namespace pc = bike::physics_core;
        pc::PointDistance<double> r{};
        if (const pc::Status s =
                pc::terrain_query<double, pc::CpuExactToolkit>(view(), p[0], p[1], r);
            s != pc::Status::ok)
            throw_core_status(s);
        return {.distance = r.distance, .normal = {r.normal[0], r.normal[1]},
                .point = {r.point[0], r.point[1]}, .segment = r.segment,
                .nonsmooth = r.nonsmooth};
    }
```
  `view()` binds a temporary to `const PolylineView<double> &`. The temporary lives until the end of the full expression, which covers the call.

  Then drop the `#include "../cblas_abi.hpp"` from `distance.cpp` if `clang-tidy`'s misc-include-cleaner flags it. Keep it otherwise.

  In `profile.cpp` keep lines 98-109 (the signature and the `xs.empty() || xs.size() != ys.size()` throw). Replace lines 110-139 with:
```cpp
        if (xs.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::invalid_argument("interp tables must be nonempty and equally wide");
        namespace pc = bike::physics_core;
        return pc::np_interp<double, pc::CpuExactToolkit>(xq, xs.data(), ys.data(),
                                                          static_cast<int>(xs.size()));
```
  Add the includes `../physics_core/cpu_exact_toolkit.hpp` and `../physics_core/terrain.hpp`.

  The added size guard can never trigger for any table built today, which are compiled hfield rows. It exists so the narrowing is proven rather than assumed. The message reuses the existing rejection text.

- [ ] **Step 4.6: Run the unit tests and the D3 users.**
```bash
cmake --build build/native-release -j && \
  ctest --test-dir build/native-release -R '^(physics_core_tests|native_distance_tests)$' --output-on-failure
```
Expected: `100% tests passed … out of 2`. A frozen-copy difference → **S2**. Then run **G1** (a failure → **S1**), **G2** (expected: as stated), **G3** and **G4**.

- [ ] **Step 4.7: Commit.**
```bash
git add native/src/physics_core/terrain.hpp native/src/tyre/distance.hpp native/src/tyre/distance.cpp native/src/tyre/profile.cpp native/tests/physics_core
git commit -m "refactor(tyre): move terrain height and query into physics_core

SignedProfile keeps building the polyline and exposes it as a POD view;
height, query and np_interp run the shared <Real, TK> core, bit-identical
to the frozen copy and the golden digest.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Task 5: Brush step, effective friction, tangent transport (`tyre_bristle.hpp`)

**Files:**
- Create: `native/src/physics_core/tyre_bristle.hpp`
- Create: `native/tests/physics_core/frozen/bristle.hpp`
- Modify: `native/src/contact/laws.hpp:43-74` (`brush_step` becomes a wrapper; `finite_fail`/`finite_result` stay, since `normal_contact` uses them)
- Modify: `native/src/writers/tire_common.cpp:14-42`, `native/src/writers/tire_common.hpp` (add `SurfaceMapAccess`, `spec_pod`)
- Modify: `native/tests/physics_core/test_physics_core.cpp`

**Interfaces (produced):**
```cpp
namespace bike::physics_core {
template <class Real> struct BrushStep { Real xi; Real force; Real loss; };
template <class Real, class TK> BIKE_HD Status brush_step(Real xi, Real u, Real v_roll, Real fn, Real k, Real mu, Real length, Real dt, BIKE_THREAD BrushStep<Real> &out) noexcept;
template <class Real> struct SurfaceSpecPod { Real mu_peak; Real mu_slide; Real stribeck_speed_mps; };
template <class Real, class TK> BIKE_HD Real surface_mu(const BIKE_THREAD SurfaceSpecPod<Real> &s, Real slip) noexcept;
BIKE_CONSTEXPR int kFrictionConfigured = -2;   // no surface map: tire mu
BIKE_CONSTEXPR int kSurfaceDefault = -1;       // map default material
template <class Real> struct Friction { Real mu; int section; };
// Map: section_count(), section_start(i), section_end(i), section_spec(i), default_spec()
template <class Real, class TK, class Map> BIKE_HD Status surface_section(const BIKE_THREAD Map &map, Real x, BIKE_THREAD int &section) noexcept;
template <class Real, class TK, class Map> BIKE_HD Status effective_friction(Real tire_mu, const BIKE_THREAD Map *map, Real contact_x, Real slip, BIKE_THREAD Friction<Real> &out) noexcept;
template <class Real> struct Transport { Real xi; Real loss; };
template <class Real, class TK> BIKE_HD Transport<Real> transport_bristle(Real xi, Real k_station, const BIKE_THREAD Real *old_tangent, const BIKE_THREAD Real *tangent) noexcept;
}
namespace tirecommon {
struct SurfaceMapAccess { const nativecfg::SurfaceMap *map; /* the five accessors */ };
[[nodiscard]] bike::physics_core::SurfaceSpecPod<double> spec_pod(const nativecfg::SurfaceSpec &s) noexcept;
}
```

- [ ] **Step 5.1: Freeze the oracle.** Create `native/tests/physics_core/frozen/bristle.hpp` in `namespace frozen_bristle`. Copy:
  - `git show b25120af:native/src/contact/laws.hpp | sed -n '13,74p'`: `finite_fail`, `finite_result`, `normal_contact`, `brush_step`.
  - `git show b25120af:native/src/writers/tire_common.cpp | sed -n '14,42p'`: `surface_mu`, `surface_at`, `effective_friction`. Mark them `inline`.

  Includes: `../../../src/config_types.hpp`, `<algorithm>`, `<cmath>`, `<initializer_list>`, `<stdexcept>`, `<string_view>`, `<tuple>`, `<utility>`.

  The transport expression has no function to freeze: it is inline in the loop at `distributed_tire.cpp:709-719`. Freeze it as this helper, copied from those lines with `dot3` spelled as its `blas::ddot(3, …)` body:
```cpp
    // distributed_tire.cpp:709-719 at b25120af, as a function.
    inline std::pair<double, double> transport(double xi, double k_station,
                                               const std::array<double, 3> &old_t,
                                               const std::array<double, 3> &t) {
        const double transported = xi * blas::ddot(3, old_t.data(), 1, t.data(), 1);
        const double loss = std::max(0.0, .5 * k_station * (xi * xi - transported * transported));
        return {transported, loss};
    }
```
  (add `#include "../../../src/cblas_abi.hpp"` and `<array>`). Verify that `dot3` really is `blas::ddot(3, a.data(), 1, b.data(), 1)`. Run `grep -n "dot3" native/src/writers/distributed_tire.cpp | head -3` and read the definition. If it differs, freeze its real body instead.

- [ ] **Step 5.2: Write the failing differential test.**

```cpp
    void test_bristle_frozen() {
        std::mt19937_64 rng(20261103u);
        std::uniform_real_distribution<double> u(-2., 2.), pos(0., 3000.), small(1e-4, .5);
        for (int i = 0; i < 20000; ++i) {
            const double xi = u(rng) * 1e-3, slip = u(rng), vr = u(rng) * 5.;
            const double fn = i % 11 == 0 ? 0. : pos(rng), k = 20000. * small(rng);
            const double mu = i % 13 == 0 ? 0. : small(rng) * 2., len = small(rng), dt = .00125;
            const auto [wx, wf, wl] = frozen_bristle::brush_step(xi, slip, vr, fn, k, mu, len, dt);
            const auto [gx, gf, gl] = contactlaw::brush_step(xi, slip, vr, fn, k, mu, len, dt);
            require(same_bits(wx, gx) && same_bits(wf, gf) && same_bits(wl, gl), "brush_step bits");
        }
        const double big = std::numeric_limits<double>::max();
        require_same_throw([&] { (void)frozen_bristle::brush_step(big, 0., 0., 1., big, 1., 1., 1.); },
                           [&] { (void)contactlaw::brush_step(big, 0., 0., 1., big, 1., 1., 1.); },
                           "brush_step non-finite");
        // Surface map: default + two sections, queries inside, on the
        // half-open edges, outside, and non-finite.
        nativecfg::SurfaceMap map{
            .surface = {.name = "hardpack", .mu_peak = .8, .mu_slide = .6, .slip_stiffness_per_load = 0., .stribeck_speed_mps = 4.5},
            .sections = {{.start_m = 2., .end_m = 4., .surface = {.name = "scree", .mu_peak = .45, .mu_slide = .38, .slip_stiffness_per_load = 0., .stribeck_speed_mps = 4.5}},
                         {.start_m = 7., .end_m = 7.5, .surface = {.name = "mud", .mu_peak = .3, .mu_slide = .2, .slip_stiffness_per_load = 0., .stribeck_speed_mps = 1.5}}}};
        for (const double x : {0., 2., 3.9, 4., 7., 7.49, 7.5, 100.})
            for (const double slip : {-3., -0., 0., .01, 2.5, 40.}) {
                const auto a = frozen_bristle::effective_friction(1.1, &map, x, slip);
                const auto b = tirecommon::effective_friction(1.1, &map, x, slip);
                require(same_bits(a.first, b.first) && a.second == b.second, "effective_friction");
                const auto c = frozen_bristle::effective_friction(1.1, nullptr, x, slip);
                const auto d = tirecommon::effective_friction(1.1, nullptr, x, slip);
                require(same_bits(c.first, d.first) && c.second == d.second, "configured friction");
                require(same_bits(frozen_bristle::surface_mu(map.surface, slip),
                                  tirecommon::surface_mu(map.surface, slip)), "surface_mu");
            }
        require_same_throw([&] { (void)frozen_bristle::surface_at(map, std::numeric_limits<double>::quiet_NaN()); },
                           [&] { (void)tirecommon::surface_at(map, std::numeric_limits<double>::quiet_NaN()); },
                           "surface_at non-finite");
        // Transport vs the frozen loop expression.
        for (int i = 0; i < 4096; ++i) {
            const std::array<double, 3> a{u(rng), 0., u(rng)}, b{u(rng), 0., u(rng)};
            const double xi = u(rng) * 1e-3, ks = 20000. * small(rng);
            const auto [wt, wl] = frozen_bristle::transport(xi, ks, a, b);
            const auto t = pc::transport_bristle<double, pc::CpuExactToolkit>(xi, ks, a.data(), b.data());
            require(same_bits(wt, t.xi) && same_bits(wl, t.loss), "transport bits");
        }
    }
```
Before writing the initialisers, check the `SurfaceSpec` field order at `config_types.hpp:237-241`: `name, mu_peak, mu_slide, slip_stiffness_per_load, stribeck_speed_mps`. The designated initialisers above follow it. Register `bristle_frozen`. Add includes for `frozen/bristle.hpp`, `../../src/contact/laws.hpp`, `../../src/writers/tire_common.hpp` and `../../src/physics_core/tyre_bristle.hpp`.

- [ ] **Step 5.3: Run to verify it fails.** Build `physics_core_tests`. Expected: `'../../src/physics_core/tyre_bristle.hpp' file not found`.

- [ ] **Step 5.4: Create `physics_core/tyre_bristle.hpp`.**

```cpp
// physics_core/tyre_bristle.hpp — tire.py _brush_step (contact/laws.hpp:43-74),
// surface.py SurfaceSpec.mu / SurfaceMap.at (writers/tire_common.cpp:14-42)
// and the distributed tangent transport (distributed_tire.cpp:709-719).
#pragma once
#include "platform.hpp"
#include "scalar.hpp"
#include "status.hpp"
#include "toolkit.hpp"

namespace bike::physics_core {

template <class Real> struct BrushStep { Real xi; Real force; Real loss; };

// contactlaw::brush_step: finite_result groups become Status::brush_nonfinite
// at the same points; the passivity throw becomes Status::brush_passivity.
template <class Real, class TK>
BIKE_HD Status brush_step(Real xi, Real u, Real v_roll, Real fn, Real k, Real mu, Real length,
                          Real dt, BIKE_THREAD BrushStep<Real> &out) noexcept {
    BIKE_CORE_TOOLKIT_CHECK(Real, TK);
    const Real old_energy = Lit<Real>::half() * k * xi * xi;
    if (!TK::isfinite(old_energy)) {
        return Status::brush_nonfinite;
    }
    if (fn == Real(0) || mu == Real(0)) {
        out.xi = Real(0);
        out.force = Real(0);
        out.loss = old_energy;
        return Status::ok;
    }
    const Real relaxation = dt * TK::div(TK::abs(v_roll), length);
    const Real denominator = Real(1) + relaxation;
    const Real trial_numerator = xi + dt * u;
    const Real friction_limit = mu * fn;
    if (!(TK::isfinite(relaxation) && TK::isfinite(denominator) &&
          TK::isfinite(trial_numerator) && TK::isfinite(friction_limit))) {
        return Status::brush_nonfinite;
    }
    const Real trial = TK::div(trial_numerator, denominator);
    const Real limit = TK::div(friction_limit, k);
    if (!(TK::isfinite(trial) && TK::isfinite(limit))) {
        return Status::brush_nonfinite;
    }
    const Real new_xi = max_std(-limit, min_std(limit, trial));
    const Real force = -k * new_xi;
    const Real new_energy = Lit<Real>::half() * k * new_xi * new_xi;
    const Real change = new_xi - xi;
    const Real implicit_loss = Lit<Real>::half() * k * change * change;
    const Real relaxation_loss = Real(2) * relaxation * new_energy;
    const Real projection_loss = (k * new_xi) * (denominator * (trial - new_xi));
    const Real loss = implicit_loss + relaxation_loss + projection_loss;
    if (!(TK::isfinite(force) && TK::isfinite(new_energy) && TK::isfinite(loss))) {
        return Status::brush_nonfinite;
    }
    if (loss < Real(0)) {
        return Status::brush_passivity;
    }
    out.xi = new_xi;
    out.force = force;
    out.loss = loss;
    return Status::ok;
}

template <class Real> struct SurfaceSpecPod { Real mu_peak; Real mu_slide; Real stribeck_speed_mps; };

// SurfaceSpec.mu (tire_common.cpp:14-18).
template <class Real, class TK>
BIKE_HD Real surface_mu(const BIKE_THREAD SurfaceSpecPod<Real> &s, Real slip) noexcept {
    const Real decay = TK::exp(TK::div(-TK::abs(slip), s.stribeck_speed_mps));
    return s.mu_slide + (s.mu_peak - s.mu_slide) * decay;
}

BIKE_CONSTEXPR int kFrictionConfigured = -2;
BIKE_CONSTEXPR int kSurfaceDefault = -1;

template <class Real> struct Friction { Real mu; int section; };

// SurfaceMap.at (tire_common.cpp:22-30): first half-open section wins.
template <class Real, class TK, class Map>
BIKE_HD Status surface_section(const BIKE_THREAD Map &map, Real x, BIKE_THREAD int &section) noexcept {
    if (!TK::isfinite(x)) {
        return Status::material_query_nonfinite;
    }
    for (int i = 0; i < map.section_count(); ++i) {
        if (map.section_start(i) <= x && x < map.section_end(i)) {
            section = i;
            return Status::ok;
        }
    }
    section = kSurfaceDefault;
    return Status::ok;
}

// effective_friction (tire_common.cpp:34-42).
template <class Real, class TK, class Map>
BIKE_HD Status effective_friction(Real tire_mu, const BIKE_THREAD Map *map, Real contact_x, Real slip,
                                  BIKE_THREAD Friction<Real> &out) noexcept {
    if (map == nullptr) {
        out.mu = tire_mu;
        out.section = kFrictionConfigured;
        return Status::ok;
    }
    int section = kSurfaceDefault;
    const Status s = surface_section<Real, TK, Map>(*map, contact_x, section);
    if (s != Status::ok) {
        return s;
    }
    const SurfaceSpecPod<Real> spec = section >= 0 ? map->section_spec(section) : map->default_spec();
    out.mu = min_std(tire_mu, surface_mu<Real, TK>(spec, slip));
    out.section = section;
    return Status::ok;
}

template <class Real> struct Transport { Real xi; Real loss; };

// distributed_tire.cpp:709-719: the stored brush transports through the
// tangent change; released stretch is dissipated.
template <class Real, class TK>
BIKE_HD Transport<Real> transport_bristle(Real xi, Real k_station, const BIKE_THREAD Real *old_tangent,
                                          const BIKE_THREAD Real *tangent) noexcept {
    const Real transported = xi * TK::dot(3, old_tangent, tangent);
    Transport<Real> t{};
    t.loss = max_std(Real(0), Lit<Real>::half() * k_station * (xi * xi - transported * transported));
    t.xi = transported;
    return t;
}

} // namespace bike::physics_core
```

- [ ] **Step 5.5: Rewire the CPU helpers.**

  `contact/laws.hpp`: add includes `../physics_core/cpu_exact_toolkit.hpp`, `../physics_core/host_status.hpp` and `../physics_core/tyre_bristle.hpp`. Replace the body of `brush_step` (:48-73) with:
```cpp
        namespace pc = bike::physics_core;
        pc::BrushStep<double> r{};
        if (const pc::Status s = pc::brush_step<double, pc::CpuExactToolkit>(
                xi, u, v_roll, Fn, k, mu, length, dt, r);
            s != pc::Status::ok)
            throw_core_status(s);
        return {r.xi, r.force, r.loss};
```
  `throw_core_status(brush_nonfinite)` throws `std::overflow_error("tire calculation exceeds finite floating-point range")`, the same as `finite_fail` (`laws.hpp:13-17`). `brush_passivity` maps to the `:70-72` message.

  `tire_common.hpp`: inside `namespace tirecommon`, after `BrushResult`, add:
```cpp
    // physics_core Map accessor over the sorted nativecfg::SurfaceMap.
    [[nodiscard]] inline bike::physics_core::SurfaceSpecPod<double>
    spec_pod(const nativecfg::SurfaceSpec &s) noexcept {
        return {.mu_peak = s.mu_peak, .mu_slide = s.mu_slide, .stribeck_speed_mps = s.stribeck_speed_mps};
    }
    struct SurfaceMapAccess {
        const nativecfg::SurfaceMap *map;
        [[nodiscard]] int section_count() const noexcept { return static_cast<int>(map->sections.size()); }
        [[nodiscard]] double section_start(int i) const noexcept { return map->sections[static_cast<std::size_t>(i)].start_m; }
        [[nodiscard]] double section_end(int i) const noexcept { return map->sections[static_cast<std::size_t>(i)].end_m; }
        [[nodiscard]] bike::physics_core::SurfaceSpecPod<double> section_spec(int i) const noexcept {
            return spec_pod(map->sections[static_cast<std::size_t>(i)].surface);
        }
        [[nodiscard]] bike::physics_core::SurfaceSpecPod<double> default_spec() const noexcept {
            return spec_pod(map->surface);
        }
    };
```
  Include `../physics_core/tyre_bristle.hpp` and `<cstddef>`. `sections.size()` comes from config validation and is far below `INT_MAX`; the narrowing matches the existing `static_cast<int>` sites in the writer.

  `tire_common.cpp:14-42`: replace the three bodies with:
```cpp
    double surface_mu(const nativecfg::SurfaceSpec &s, double slip) {
        namespace pc = bike::physics_core;
        const pc::SurfaceSpecPod<double> pod = spec_pod(s);
        return pc::surface_mu<double, pc::CpuExactToolkit>(pod, slip);
    }

    const nativecfg::SurfaceSpec &
    surface_at(const nativecfg::SurfaceMap &map, double x_m) {
        namespace pc = bike::physics_core;
        const SurfaceMapAccess access{.map = &map};
        int section = pc::kSurfaceDefault;
        if (const pc::Status s = pc::surface_section<double, pc::CpuExactToolkit>(access, x_m, section);
            s != pc::Status::ok)
            throw_core_status(s);
        return section >= 0 ? map.sections[static_cast<std::size_t>(section)].surface : map.surface;
    }

    std::pair<double, std::string_view>
    effective_friction(double tire_mu, const nativecfg::SurfaceMap *map,
                       double contact_x, double slip) {
        namespace pc = bike::physics_core;
        const SurfaceMapAccess access{.map = map};
        pc::Friction<double> f{};
        if (const pc::Status s = pc::effective_friction<double, pc::CpuExactToolkit>(
                tire_mu, map != nullptr ? &access : nullptr, contact_x, slip, f);
            s != pc::Status::ok)
            throw_core_status(s);
        if (f.section == pc::kFrictionConfigured)
            return {f.mu, "configured"};
        return {f.mu, f.section >= 0 ? std::string_view(map->sections[static_cast<std::size_t>(f.section)].surface.name)
                                     : std::string_view(map->surface.name)};
    }
```
  Keep the NOLINT comment lines of `effective_friction`. Add the includes `../physics_core/cpu_exact_toolkit.hpp` and `../physics_core/host_status.hpp`.

- [ ] **Step 5.6: Run the tests.** Build all, then `ctest -R '^physics_core_tests$'`. Expected: Passed (a difference → **S2**). Then **G1** (→ **S1**), **G2** (includes `native_rider_contacts`, the other `contactlaw::brush_step` caller at `rider_contacts.cpp:549`), **G3** and **G4**.

- [ ] **Step 5.7: Commit.**
```bash
git add native/src/physics_core/tyre_bristle.hpp native/src/contact/laws.hpp native/src/writers/tire_common.hpp native/src/writers/tire_common.cpp native/tests/physics_core
git commit -m "refactor(tyre): move brush step and friction into physics_core

brush_step, surface_mu, surface_at, effective_friction and the bristle
transport run the shared <Real, TK> core; compliant, pneumatic and rider
contact callers keep their bits and exceptions (D3).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Task 6: Station kinematics (`kinematics.hpp`, part 1)

**Files:**
- Create: `native/src/physics_core/kinematics.hpp`
- Modify: `native/src/writers/distributed_tire.cpp`:
  - `:446-456`: the ring lift in `station_points`;
  - `:636-638`: the centre velocity;
  - `:673-687`: the station Jacobian and velocity;
  - `:743-752`: the projection.
- Modify: `native/tests/physics_core/test_physics_core.cpp`

**Interfaces (produced):**
```cpp
namespace bike::physics_core {
template <class Real, class TK> BIKE_HD void lift_ring(int n, const BIKE_CORE_TABLE Real *local, const BIKE_THREAD Real *rotation, const BIKE_THREAD Real *center, BIKE_THREAD Real *points) noexcept;
template <class Real> BIKE_HD void station_jacobian(int nv, const BIKE_THREAD Real *jp, const BIKE_THREAD Real *jr, const BIKE_THREAD Real *arm, BIKE_THREAD Real *jac) noexcept;
template <class Real, class TK> BIKE_HD void project_station_force(int nv, const BIKE_THREAD Real *jac, const BIKE_THREAD Real *force, BIKE_THREAD Real *tmp, BIKE_THREAD Real *out) noexcept;
}
```

- [ ] **Step 6.1: Freeze the oracle.** Create `native/tests/physics_core/frozen/kinematics.hpp` in `namespace frozen_kinematics`. It holds three functions, each the body of the cited `b25120af` lines with members replaced by parameters, so that the expressions are character-identical:
```cpp
    // distributed_tire.cpp:446-456 at b25120af.
    inline void lift(std::size_t n, const double *local, const double *rot, const double *center, double *points) {
        blas::dgemm(blas::Order::row_major, blas::Transpose::no, blas::Transpose::yes, static_cast<int>(n), 3, 3,
                    1.0, local, 3, rot, 3, 0.0, points, 3);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t k = 0; k < 3; ++k)
                points[3 * i + k] += center[k];
    }
    // :673-681.
    inline void jac(std::size_t nv, const double *jac_center, const double *jr, const double *arm, double *out) {
        for (std::size_t k = 0; k < 3; ++k) {
            const std::size_t a = (k + 1) % 3, b = (k + 2) % 3;
            for (std::size_t i = 0; i < nv; ++i)
                out[k * nv + i] = jac_center[k * nv + i] + (jr[a * nv + i] * arm[b] - jr[b * nv + i] * arm[a]);
        }
    }
    // :743-752.
    inline void project(int nv, const double *jac_station, const double *force, double *tmp, double *out) {
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, nv, 1.0, jac_station, nv, force, 1, 0.0, tmp, 1);
        for (std::size_t i = 0; i < static_cast<std::size_t>(nv); ++i)
            out[i] += tmp[i];
    }
```

- [ ] **Step 6.2: Write the failing differential test.**

  In `test_physics_core.cpp`, add `test_kinematics_frozen`. Data:
  - random `nv ∈ {1, 7, 13, 32}`;
  - `n ∈ {16, 128, 512}`;
  - random rotation entries in `[-1, 1]` (BLAS does not care about orthogonality);
  - random `jp`, `jr`, `arm` and `force`.

  Compare bitwise:
  - `pc::lift_ring<double, pc::CpuExactToolkit>` vs `frozen_kinematics::lift`;
  - `pc::station_jacobian<double>` vs `frozen_kinematics::jac`;
  - `pc::project_station_force<double, pc::CpuExactToolkit>` vs `frozen_kinematics::project`. Start `out` from a non-zero random vector so the `+=` order is exercised.

  Register `kinematics_frozen`.

- [ ] **Step 6.3: Run to verify it fails.** Expected: `'../../src/physics_core/kinematics.hpp' file not found`.

- [ ] **Step 6.4: Create `physics_core/kinematics.hpp`.**

```cpp
// physics_core/kinematics.hpp — distributed tyre station kinematics
// (distributed_tire.cpp:446-456, :673-687, :743-752) and, from Task 8, the
// dof-chain point Jacobian used by the GPU (D10).
#pragma once
#include "platform.hpp"
#include "scalar.hpp"
#include "status.hpp"
#include "toolkit.hpp"

namespace bike::physics_core {

// center + local @ rotation.T (distributed_tire_forces.py:112).
template <class Real, class TK>
BIKE_HD void lift_ring(int n, const BIKE_CORE_TABLE Real *local, const BIKE_THREAD Real *rotation,
                       const BIKE_THREAD Real *center, BIKE_THREAD Real *points) noexcept {
    TK::ring_lift(n, local, rotation, points);
    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < 3; ++k) {
            points[3 * i + k] += center[k];
        }
    }
}

// jp + cross(jr, arm) row-wise (distributed_tire_forces.py:163-165), the
// unrolled mul-mul-sub then add per element.
template <class Real>
BIKE_HD void station_jacobian(int nv, const BIKE_THREAD Real *jp, const BIKE_THREAD Real *jr,
                              const BIKE_THREAD Real *arm, BIKE_THREAD Real *jac) noexcept {
    for (int k = 0; k < 3; ++k) {
        const int a = (k + 1) % 3;
        const int b = (k + 2) % 3;
        for (int i = 0; i < nv; ++i) {
            jac[k * nv + i] = jp[k * nv + i] + (jr[a * nv + i] * arm[b] - jr[b * nv + i] * arm[a]);
        }
    }
}

// result += jac.T @ force, materialized into tmp first (:183-184).
template <class Real, class TK>
BIKE_HD void project_station_force(int nv, const BIKE_THREAD Real *jac, const BIKE_THREAD Real *force,
                                   BIKE_THREAD Real *tmp, BIKE_THREAD Real *out) noexcept {
    TK::gemv_t3(nv, jac, force, tmp);
    for (int i = 0; i < nv; ++i) {
        out[i] += tmp[i];
    }
}

} // namespace bike::physics_core
```

- [ ] **Step 6.5: Wire the writer.** In `distributed_tire.cpp` add `#include "../physics_core/cpu_exact_toolkit.hpp"` and `#include "../physics_core/kinematics.hpp"`, plus `namespace pc = bike::physics_core;` in the file's anonymous namespace. Then:

  1. **`:446-456`** becomes:
```cpp
    pc::lift_ring<double, pc::CpuExactToolkit>(
        static_cast<int>(n), &local_[side][0][0], xmat.subspan(b9, 9).data(), center.data(),
        points_.data());
```
  2. **`:636-638`** becomes `pc::CpuExactToolkit::gemv_n3(nv_, jac_center_.data(), qvel.data(), center_velocity.data());`.
  3. **`:673-681`** (the `for k` loop) becomes `pc::station_jacobian<double>(nv_, jac_center_.data(), jr_.data(), arm.data(), jac_station_.data());`.
  4. **`:684-687`** becomes `pc::CpuExactToolkit::gemv_n3(nv_, jac_station_.data(), qvel.data(), velocity.data());`.
  5. **`:747-752`** (the `dgemv` plus `+=`) becomes `pc::project_station_force<double, pc::CpuExactToolkit>(nv_, jac_station_.data(), force.data(), tmp_.data(), out.data());`.

  Keep every comment that cites the Python line. `local_[side]` is a `std::vector<std::array<double, 3>>` (`distributed_tire.cpp:233-238`), so `&local_[side][0][0]` addresses `n*3` contiguous doubles, exactly as the replaced `dgemm` call did.

- [ ] **Step 6.6: Run.** Build, then `physics_core_tests` (→ **S2**), then **G1** (→ **S1**), **G3** and **G4**.

- [ ] **Step 6.7: Commit** (`refactor(tyre): move station kinematics into physics_core`, same body style and trailer).

---

## Task 7: The per-wheel step (`tyre.hpp`)

**Files:**
- Create: `native/src/physics_core/tyre.hpp`
- Modify:
  - `native/src/writers/distributed_tire.hpp:208-268`: declare the private nested `StationIo`.
  - `native/src/writers/distributed_tire.cpp:432-467`: `station_points` calls `classify_stations`.
  - `native/src/writers/distributed_tire.cpp:650-772`: the station loop becomes one `wheel_contacts` call.
- Modify: `native/tests/physics_core/test_physics_core.cpp`: status-path tests.

**Interfaces (produced):**
```cpp
namespace bike::physics_core {
template <class Real> struct StationState { bool active; Real xi; bool has_tangent; Real tangent[3]; };
template <class Real> struct StationResult {
    int j, sid;
    Real point[3], normal[3], tangent[3];
    Real load, elastic_energy, penetration, radial_power_term;
    bool nonsmooth, transported;
    Real transport_loss, slip, tangent_force, brush_loss, xi_new, k_station;
};
template <class Real> struct WheelGeometry { const BIKE_CORE_TABLE Real *ring_local; int station_count; const BIKE_THREAD Real *center; const BIKE_THREAD Real *rotation; };
template <class Real> struct WheelScratch { BIKE_THREAD Real *points; BIKE_THREAD Real *heights; BIKE_THREAD int *active_ids; BIKE_THREAD Real *jac_station; BIKE_THREAD Real *tmp; };
template <class Real> struct WheelLaw { const BIKE_CORE_TABLE Real *weights; HingeDensityView<Real> density; Real tangent_k_n_m; Real tire_mu; Real relaxation_length_m; };
template <class Real> struct WheelJacobian { const BIKE_THREAD Real *jp; const BIKE_THREAD Real *jr; const BIKE_THREAD Real *qvel; int nv; };
// Io: StationState<Real> old_state(int sid) const; void station(const StationResult<Real> &r);
template <class Real, class TK> BIKE_HD Status classify_stations(const BIKE_THREAD WheelGeometry<Real> &g, const BIKE_THREAD PolylineView<Real> &pl, const BIKE_THREAD WheelScratch<Real> &s, BIKE_THREAD int &count) noexcept;
template <class Real, class TK, class Map, class Io> BIKE_HD Status wheel_contacts(const BIKE_THREAD WheelGeometry<Real> &g, const BIKE_THREAD PolylineView<Real> &pl, const BIKE_THREAD WheelLaw<Real> &law, const BIKE_THREAD WheelJacobian<Real> &jac, const BIKE_THREAD Map *map, Real dt, const BIKE_THREAD WheelScratch<Real> &s, int count, BIKE_THREAD Real *qfrc, BIKE_THREAD Io &io) noexcept;
}
```
The station loop at `distributed_tire.cpp:650-772` does five kinds of work in one order. The core keeps each operation at its place:
1. query;
2. kinematics;
3. response;
4. transport, friction and brush;
5. projection.

The CPU-only work runs inside `Io::station` and keeps its relative order. It covers:
- the ledgers (`radial_power`, `losses`, `shear_energy` with `pyfloat::pow`);
- the artifacts (`loads_`, `energies_`, `depths_`, `normals_`, `nonsmooth`);
- the patch-capacity guard and `make_patch`, which may throw;
- the state stage.

The `out += tmp` projection precedes `make_patch` in both versions. When `make_patch` throws, `out` is discarded by the caller in both versions. Nothing observable can differ.

- [ ] **Step 7.1: Write the failing status-path test.** The bitwise oracle for this task is the golden replay: G1's three reference nodes plus the digest. The extracted loop is too entangled with writer members to freeze as a function. The unit test pins the core's status ordering, which the CPU exceptions depend on:

```cpp
    struct RecordingIo {
        int calls = 0;
        pc::StationState<double> old_state(int) const { return {false, 0., false, {0., 0., 0.}}; }
        void station(const pc::StationResult<double> &) { ++calls; }
    };
    struct NoMap {
        int section_count() const { return 0; }
        double section_start(int) const { return 0.; }
        double section_end(int) const { return 0.; }
        pc::SurfaceSpecPod<double> section_spec(int) const { return {0., 0., 1.}; }
        pc::SurfaceSpecPod<double> default_spec() const { return {0., 0., 1.}; }
    };

    // A flat road at z=0 and a wheel of radius 0.3 m whose centre sits at
    // z=0.29: stations near the bottom penetrate. Statuses must surface in
    // the CPU's exception order: domain (classify) before anything else,
    // quadrature before material, material before brush.
    void test_tyre_status_order() {
        const biketyre::SignedProfile road({{-10., 0.}, {10., 0.}});
        const auto view = road.view();
        constexpr int n = 16;
        std::array<double, 3 * n> local{};
        for (int i = 0; i < n; ++i) {
            const double a = 2. * std::numbers::pi * i / n;
            local[3 * i] = .3 * std::cos(a);
            local[3 * i + 2] = .3 * std::sin(a);
        }
        const std::array<double, 9> rot{1., 0., 0., 0., 1., 0., 0., 0., 1.};
        std::array<double, 3> center{0., 0., .29};
        std::array<double, 3 * n> points{};
        std::array<double, n> heights{};
        std::array<int, n> ids{};
        std::array<double, 3> jac_station{}, tmp{};
        const pc::WheelScratch<double> s{points.data(), heights.data(), ids.data(), jac_station.data(), tmp.data()};
        pc::WheelGeometry<double> g{local.data(), n, center.data(), rot.data()};
        int count = -1;
        require(pc::classify_stations<double, pc::CpuExactToolkit>(g, view, s, count) == pc::Status::ok && count > 0,
                "bottom stations penetrate");
        center[0] = 9.9; // ring reaches x > 10: outside the road domain
        require(pc::classify_stations<double, pc::CpuExactToolkit>(g, view, s, count) ==
                    pc::Status::terrain_outside_domain, "domain error from classify");
        center[0] = 0.;
        require(pc::classify_stations<double, pc::CpuExactToolkit>(g, view, s, count) == pc::Status::ok, "reclassify");
        const std::array<double, 3> jp{1., 0., 0.}, jr{0., 1., 0.}, qvel{.5};
        std::array<double, n> weights{};
        weights.fill(2. * std::numbers::pi / n);
        const std::array<double, 3> knots{0., .004, .012}, stiff{300000., 0., 0.};
        pc::WheelLaw<double> law{weights.data(), {knots.data(), stiff.data(), 3, 2000.}, 20000., 1.1, .2};
        const pc::WheelJacobian<double> jac{jp.data(), jr.data(), qvel.data(), 1};
        std::array<double, 1> qfrc{0.};
        RecordingIo io;
        require(pc::wheel_contacts<double, pc::CpuExactToolkit, NoMap, RecordingIo>(
                    g, view, law, jac, nullptr, .00125, s, count, qfrc.data(), io) == pc::Status::ok &&
                    io.calls == count, "every active station reaches Io once");
        weights[static_cast<std::size_t>(ids[0])] = -1.;
        RecordingIo io2;
        require(pc::wheel_contacts<double, pc::CpuExactToolkit, NoMap, RecordingIo>(
                    g, view, law, jac, nullptr, .00125, s, count, qfrc.data(), io2) ==
                    pc::Status::station_invalid_quadrature && io2.calls == 0,
                "quadrature error before Io of the failing station");
    }
```
Register `tyre_status_order`. Add includes `../../src/physics_core/tyre.hpp` and `<numbers>`.

- [ ] **Step 7.2: Run to verify it fails.** Expected: `'../../src/physics_core/tyre.hpp' file not found`.

- [ ] **Step 7.3: Create `physics_core/tyre.hpp`.**

```cpp
// physics_core/tyre.hpp — the distributed_2d_reference per-wheel step
// (distributed_tire_forces.py:150-189; native/src/writers/distributed_tire.cpp
// :432-467 and :650-772). classify_stations is _stations; wheel_contacts is
// the station loop. Ledgers, artifacts and state staging live in the
// caller's Io (plan P3, P4).
#pragma once
#include "kinematics.hpp"
#include "platform.hpp"
#include "scalar.hpp"
#include "status.hpp"
#include "terrain.hpp"
#include "toolkit.hpp"
#include "tyre_bristle.hpp"
#include "tyre_station.hpp"

namespace bike::physics_core {

template <class Real> struct StationState {
    bool active;
    Real xi;
    bool has_tangent;
    Real tangent[3];
};

template <class Real> struct StationResult {
    int j;
    int sid;
    Real point[3];
    Real normal[3];
    Real tangent[3];
    Real load;
    Real elastic_energy;
    Real penetration;
    Real radial_power_term;
    bool nonsmooth;
    bool transported;
    Real transport_loss;
    Real slip;
    Real tangent_force;
    Real brush_loss;
    Real xi_new;
    Real k_station;
};

template <class Real> struct WheelGeometry {
    const BIKE_CORE_TABLE Real *ring_local;
    int station_count;
    const BIKE_THREAD Real *center;
    const BIKE_THREAD Real *rotation;
};

template <class Real> struct WheelScratch {
    BIKE_THREAD Real *points;
    BIKE_THREAD Real *heights;
    BIKE_THREAD int *active_ids;
    BIKE_THREAD Real *jac_station;
    BIKE_THREAD Real *tmp;
};

template <class Real> struct WheelLaw {
    const BIKE_CORE_TABLE Real *weights;
    HingeDensityView<Real> density;
    Real tangent_k_n_m;
    Real tire_mu;
    Real relaxation_length_m;
};

template <class Real> struct WheelJacobian {
    const BIKE_THREAD Real *jp;
    const BIKE_THREAD Real *jr;
    const BIKE_THREAD Real *qvel;
    int nv;
};

// _stations (:108-115): world lift, the height sweep over EVERY station
// (a domain error aborts before any station is judged active), ascending
// penetrating ids.
template <class Real, class TK>
BIKE_HD Status classify_stations(const BIKE_THREAD WheelGeometry<Real> &g,
                                 const BIKE_THREAD PolylineView<Real> &pl,
                                 const BIKE_THREAD WheelScratch<Real> &s,
                                 BIKE_THREAD int &count) noexcept {
    BIKE_CORE_TOOLKIT_CHECK(Real, TK);
    const int n = g.station_count;
    lift_ring<Real, TK>(n, g.ring_local, g.rotation, g.center, s.points);
    for (int i = 0; i < n; ++i) {
        const Status st = terrain_height<Real, TK>(pl, s.points[3 * i], s.heights[i]);
        if (st != Status::ok) {
            return st;
        }
    }
    int c = 0;
    for (int i = 0; i < n; ++i) {
        if (s.points[3 * i + 2] < s.heights[i]) {
            s.active_ids[c] = i;
            c += 1;
        }
    }
    count = c;
    return Status::ok;
}

// The station loop (:156-189). qfrc accumulates (the caller zeroes it).
template <class Real, class TK, class Map, class Io>
BIKE_HD Status wheel_contacts(const BIKE_THREAD WheelGeometry<Real> &g,
                              const BIKE_THREAD PolylineView<Real> &pl,
                              const BIKE_THREAD WheelLaw<Real> &law,
                              const BIKE_THREAD WheelJacobian<Real> &jac,
                              const BIKE_THREAD Map *map, Real dt,
                              const BIKE_THREAD WheelScratch<Real> &s, int count,
                              BIKE_THREAD Real *qfrc, BIKE_THREAD Io &io) noexcept {
    BIKE_CORE_TOOLKIT_CHECK(Real, TK);
    // :152 — center_velocity = jp_center @ qvel.
    Real center_velocity[3];
    TK::gemv_n3(jac.nv, jac.jp, jac.qvel, center_velocity);
    for (int j = 0; j < count; ++j) {
        StationResult<Real> r{};
        r.j = j;
        r.sid = s.active_ids[j];
        const int sid = r.sid;
        r.point[0] = s.points[3 * sid];
        r.point[1] = s.points[3 * sid + 1];
        r.point[2] = s.points[3 * sid + 2];
        // :157-160 — signed distance, X-Z normal and tangent.
        PointDistance<Real> geometry{};
        Status st = terrain_query<Real, TK>(pl, r.point[0], r.point[2], geometry);
        if (st != Status::ok) {
            return st;
        }
        r.normal[0] = geometry.normal[0];
        r.normal[1] = Real(0);
        r.normal[2] = geometry.normal[1];
        r.tangent[0] = r.normal[2];
        r.tangent[1] = Real(0);
        r.tangent[2] = -r.normal[0];
        // :161-166 — arm, station Jacobian, velocity.
        const Real arm[3] = {r.point[0] - g.center[0], r.point[1] - g.center[1],
                             r.point[2] - g.center[2]};
        station_jacobian<Real>(jac.nv, jac.jp, jac.jr, arm, s.jac_station);
        Real velocity[3];
        TK::gemv_n3(jac.nv, s.jac_station, jac.qvel, velocity);
        // :167-171 — rate, penetration, radial response and power term.
        const Real rate = -((velocity[0] * r.normal[0] + velocity[1] * r.normal[1]) +
                            velocity[2] * r.normal[2]);
        r.penetration = -geometry.distance;
        const Real weight = law.weights[sid];
        StationResponse<Real> resp{};
        st = station_response<Real, TK>(r.penetration, rate, weight, law.density, resp);
        if (st != Status::ok) {
            return st;
        }
        StationDensity<Real> dens{};
        st = station_density<Real, TK>(r.penetration, law.density, dens);
        if (st != Status::ok) {
            return st;
        }
        r.load = resp.load;
        r.elastic_energy = resp.energy;
        r.radial_power_term = max_std(Real(0), (r.load - weight * dens.force) * rate);
        // :172-177 — transport the stored brush through the tangent change.
        const StationState<Real> old = io.old_state(sid);
        Real xi = old.xi;
        r.k_station = law.tangent_k_n_m * weight;
        if (old.has_tangent) {
            const Transport<Real> t = transport_bristle<Real, TK>(xi, r.k_station, old.tangent, r.tangent);
            r.transported = true;
            r.transport_loss = t.loss;
            xi = t.xi;
        }
        // :178-182 — slip, effective mu, brush step.
        r.slip = TK::dot(3, velocity, r.tangent);
        Friction<Real> friction{};
        st = effective_friction<Real, TK, Map>(law.tire_mu, map, geometry.point[0], r.slip, friction);
        if (st != Status::ok) {
            return st;
        }
        BrushStep<Real> brush{};
        st = brush_step<Real, TK>(xi, r.slip, TK::dot(3, center_velocity, r.tangent), r.load,
                                  r.k_station, friction.mu, law.relaxation_length_m, dt, brush);
        if (st != Status::ok) {
            return st;
        }
        r.xi_new = brush.xi;
        r.tangent_force = brush.force;
        r.brush_loss = brush.loss;
        // :183-184 — force and result += jac.T @ force.
        const Real force[3] = {r.load * r.normal[0] + r.tangent_force * r.tangent[0],
                               r.load * r.normal[1] + r.tangent_force * r.tangent[1],
                               r.load * r.normal[2] + r.tangent_force * r.tangent[2]};
        project_station_force<Real, TK>(jac.nv, s.jac_station, force, s.tmp, qfrc);
        r.nonsmooth = geometry.nonsmooth;
        // :185-189 — the caller's artifacts, patch and state stage.
        io.station(r);
    }
    return Status::ok;
}

} // namespace bike::physics_core
```
`std::max(0.0, x)` is `max_std(Real(0), x)` (`:704-706`). `.5 * k_station * …` and `.5 * tk * …` stay in the CPU `Io`, because `pyfloat::pow` is host-only.

- [ ] **Step 7.4: Rewire the writer.**

  In `distributed_tire.hpp`:
  - add `#include "../physics_core/tyre.hpp"`;
  - in the private section, declare `struct StationIo;` (a nested class already has access to the private members, so no `friend` is needed).

  In `distributed_tire.cpp`, add `#include "../physics_core/host_status.hpp"`, then rewrite `station_points` (:432-467). Keep the signature. Replace the body from `blas::dgemm` / `lift_ring` to the end with:
```cpp
    const pc::WheelGeometry<double> g{.ring_local = &local_[side][0][0], .station_count = static_cast<int>(n),
                                      .center = center.data(), .rotation = xmat.subspan(b9, 9).data()};
    // classify_stations reads only points/heights/ids — the three mutable
    // scratch members (distributed_tire.hpp:244-246); jac_station_/tmp_
    // are not mutable and stay out of this const method.
    const pc::WheelScratch<double> s{.points = points_.data(), .heights = heights_.data(),
                                     .active_ids = active_ids_.data(), .jac_station = nullptr,
                                     .tmp = nullptr};
    int count = 0;
    if (const pc::Status st = pc::classify_stations<double, pc::CpuExactToolkit>(
            g, engaged(profile_).view(), s, count);
        st != pc::Status::ok)
        throw_core_status(st);
    return static_cast<std::size_t>(count);
```
  `station_points` is `const`. Only `points_`, `heights_` and `active_ids_` are `mutable` (verified: `distributed_tire.hpp:244-246`), which is why the scratch above passes `nullptr` for the two buffers `classify_stations` never touches.

  Define the Io in `distributed_tire.cpp` at file scope, after the anonymous namespace that holds `make_patch`. It is the nested class declared in the header, so it cannot live inside the anonymous namespace:
```cpp
// The CPU side of the core's station loop (plan P4): every ledger, artifact,
// patch and state stage of distributed_tire.cpp:704-771, in their original
// order. make_patch and the capacity guard may throw from here.
struct DistributedTireWriter::StationIo {
    DistributedTireWriter &w;
    std::size_t s;
    std::size_t n;
    TireSnapshot &snap;
    double &losses;
    double &shear_energy;
    double &radial_power;
    int &nonsmooth;

    [[nodiscard]] pc::StationState<double> old_state(int sid) const {
        const StationState &old = w.states_[s][static_cast<std::size_t>(sid)];
        pc::StationState<double> r{.active = old.active, .xi = old.xi,
                                   .has_tangent = old.tangent.has_value(), .tangent = {0., 0., 0.}};
        if (old.tangent) {
            r.tangent[0] = (*old.tangent)[0];
            r.tangent[1] = (*old.tangent)[1];
            r.tangent[2] = (*old.tangent)[2];
        }
        return r;
    }

    void station(const pc::StationResult<double> &r) {
        constexpr std::size_t kDim = model_access::kXYZ;
        const std::size_t j = static_cast<std::size_t>(r.j);
        const std::size_t sid = static_cast<std::size_t>(r.sid);
        radial_power += r.radial_power_term;                                  // :704-706
        if (r.transported)
            losses += r.transport_loss;                                       // :715-717
        losses += r.brush_loss;                                               // :737
        shear_energy += .5 * r.k_station * pyfloat::pow(r.xi_new, 2.0);       // :738
        w.loads_[j] = r.load;                                                 // :755-761
        w.energies_[j] = r.elastic_energy;
        w.depths_[j] = r.penetration;
        nonsmooth += r.nonsmooth ? 1 : 0;
        const Vec3 point = {r.point[0], r.point[1], r.point[2]};
        const Vec3 normal = {r.normal[0], r.normal[1], r.normal[2]};
        const Vec3 tangent = {r.tangent[0], r.tangent[1], r.tangent[2]};
        w.normals_[kDim * j] = normal[0];
        w.normals_[kDim * j + 1] = normal[1];
        w.normals_[kDim * j + 2] = normal[2];
        if (snap.patches.size() >= n)                                         // :764-766
            throw std::invalid_argument("tire snapshot patch capacity exceeded");
        snap.patches.push_back(make_patch(point, normal, r.load, r.tangent_force, r.slip));
        w.states_stage_[s][sid] = StationState{.active = true, .xi = r.xi_new, .tangent = tangent};
    }
};
```
  Before writing this, re-read `distributed_tire.cpp:699-771` and check each cited sub-line. Every statement above must correspond 1:1, in the same relative order, to a statement there. In particular, `radial_power +=` precedes the transport `losses +=` (`:704` vs `:715`) and `losses += brush_loss` precedes `shear_energy +=` (`:737-738`). The only reordering is that `radial_power +=` now runs after the brush step and the projection. `radial_power` is a local that nothing reads before the loop ends, so its value and bits are unchanged.

  Replace the loop `:650-772` (`for (std::size_t j = 0; j < count; ++j) { … }`) with:
```cpp
        const pc::WheelGeometry<double> g{
            .ring_local = &local_[s][0][0], .station_count = static_cast<int>(n),
            .center = center.data(),
            .rotation = model_access::readonly_buffer(d->xmat, 9 * m_->nbody)
                            .subspan(9 * static_cast<std::size_t>(body), 9).data()};
        const pc::WheelScratch<double> scratch{
            .points = points_.data(), .heights = heights_.data(), .active_ids = active_ids_.data(),
            .jac_station = jac_station_.data(), .tmp = tmp_.data()};
        const pc::WheelLaw<double> law{
            .weights = stations_.weights.data(),
            .density = biketyre::density_view(material),
            .tangent_k_n_m = tk, .tire_mu = pcfg.mu, .relaxation_length_m = pcfg.relaxation_length_m};
        const pc::WheelJacobian<double> wj{.jp = jac_center_.data(), .jr = jr_.data(),
                                           .qvel = qvel.data(), .nv = nv_};
        const tirecommon::SurfaceMapAccess map_access{.map = surface_map_ ? &*surface_map_ : nullptr};
        StationIo io{.w = *this, .s = s, .n = n, .snap = snap, .losses = losses,
                     .shear_energy = shear_energy, .radial_power = radial_power, .nonsmooth = nonsmooth};
        if (const pc::Status st = pc::wheel_contacts<double, pc::CpuExactToolkit,
                                                     tirecommon::SurfaceMapAccess, StationIo>(
                g, engaged(profile_).view(), law, wj, surface_map_ ? &map_access : nullptr, dt,
                scratch, static_cast<int>(count), out.data(), io);
            st != pc::Status::ok)
            throw_core_status(st);
```
  Keep `center_velocity` out: the core computes it from `jac_center_` (`:636-638` moves into the core). Delete the now-unused `Vec3 center_velocity` and its `gemv_n3` line from Task 6. Keep `point_jacobian` (`:634`) before this block.

  `biketyre::density_view` is Task 3's helper. Move it from `station.cpp`'s anonymous namespace into `namespace biketyre`, and declare it in `tyre/station.hpp`, so the zip bound and its narrowing guard exist once:
```cpp
    // zip(knots_m, stiffness_n_m) as the core's density view (station.cpp).
    [[nodiscard]] bike::physics_core::HingeDensityView<double>
    density_view(const nativecfg::HingeDensity &d);
```
  `station.hpp` then includes `../physics_core/tyre_station.hpp`.

  `stored_energy` (`:473-511`) keeps calling `station_points` and `biketyre::station_density`, unchanged.

- [ ] **Step 7.5: Run the tests.** Build, then `physics_core_tests`. Expected: Passed. Then run **G1**. This is the load-bearing gate for Task 7: the three reference nodes replay the golden episodes bitwise, and the digest covers the full ride. A failure → **S1**. Then run **G3** and **G4**.

- [ ] **Step 7.6: Commit.**
```bash
git add native/src/physics_core/tyre.hpp native/src/writers/distributed_tire.hpp native/src/writers/distributed_tire.cpp native/tests/physics_core
git commit -m "refactor(tyre): run the distributed wheel step from physics_core

classify_stations and wheel_contacts are the station loop; the writer's
StationIo keeps ledgers, patches and state staging in their original order,
so the golden replay and the episode digest stay bit-identical (D2).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---
## Task 8: Exact dof-chain point Jacobian (D10, R3)

**Files:**
- Modify: `native/src/physics_core/kinematics.hpp` (add `chain_point_jacobian`, joint constants)
- Create: `native/src/gpu/tyre_model.hpp` (GPU POD types; this task adds `GpuWheelChain`, Task 9 the rest)
- Create: `native/src/gpu/tyre_upload.hpp`, `native/src/gpu/tyre_upload.cpp` (this task: `build_wheel_chain`)
- Modify: `CMakeLists.txt:113-171` (add `native/src/gpu/tyre_upload.cpp` to `bike_core`, after `tyre/station.cpp` at :129)
- Create: `native/tests/reference/test_gpu_tyre_core.cpp`, `native/tests/reference/GpuSharedCore.cmake`
- Modify: `native/tests/reference/CMakeLists.txt` (include `GpuSharedCore.cmake` after `Phase3Ports.cmake`, :45)

**Interfaces (produced):**
```cpp
namespace bike::physics_core {
BIKE_CONSTEXPR int kJointSlide = 2;   // mjJNT_SLIDE (mjtype.h:93)
BIKE_CONSTEXPR int kJointHinge = 3;   // mjJNT_HINGE (mjtype.h:94)
template <class Real>
BIKE_HD Status chain_point_jacobian(int joint_count, const BIKE_CORE_TABLE int *type,
                                    const BIKE_CORE_TABLE int *joint_id, const BIKE_CORE_TABLE int *dof_id,
                                    const BIKE_CORE_TABLE Real *xanchor, const BIKE_CORE_TABLE Real *xaxis,
                                    const BIKE_THREAD Real *point, int nv,
                                    BIKE_THREAD Real *jp, BIKE_THREAD Real *jr) noexcept;
}
namespace bike::gpu {
struct GpuWheelChain { int joint_count; int type[kMaxChainJoints]; int joint_id[kMaxChainJoints]; int dof_id[kMaxChainJoints]; };
[[nodiscard]] GpuWheelChain build_wheel_chain(const mjModel *m, int body);   // host only, throws
}
```
The `xanchor`/`xaxis` pointers are per world and in device memory on the GPU, so they are `BIKE_CORE_TABLE`. These are MuJoCo's `d->xanchor`/`d->xaxis` (CPU) or mujoco_warp's `Data.xanchor`/`Data.xaxis` (`types.py:2242-2243`).

- [ ] **Step 8.1: Write the failing reference test.** Create `native/tests/reference/test_gpu_tyre_core.cpp`:

```cpp
// test_gpu_tyre_core.cpp — GPU shared-core contracts on the real stand40
// model (Stage 1): the exact dof-chain Jacobian against mj_jac (D10, R3),
// the tyre upload (Task 9), and host-float vs CPU-double (Task 10).
#include "tyre_reference_support.hpp"

#include "gpu/tyre_upload.hpp"
#include "physics_core/kinematics.hpp"

#include <cmath>
#include <iostream>
#include <random>

namespace {

using namespace tyre_reference;

[[nodiscard]] std::filesystem::path bundle(std::string_view name) {
    return platform_fixture("native_distributed_tire", name);
}

[[nodiscard]] int wheel_body(const mjModel *m, const char *geom) {
    const int g = mj_name2id(m, mjOBJ_GEOM, geom);
    bt::require(g >= 0, std::string("stand40 has ") + geom);
    return m->geom_bodyid[g];
}

// mj_jac vs the dof-chain formula at the wheel centre and at an offset
// contact-like point, over episode rows and chain-joint perturbations.
[[gnu::noinline]] void test_chain_jacobian_matches_mj_jac() {
    const auto ep = load_episode_bundle(bundle("episodes/stand40"));
    Stepper st((bundle("episodes/stand40") / "model.mjb").string(), load_config(bundle("env")));
    const mjModel *m = st.model();
    const mjData *d = st.data();
    std::cout << "stand40 nq=" << m->nq << " nv=" << m->nv << " nbody=" << m->nbody
              << " njnt=" << m->njnt << '\n';
    const std::array<int, 2> bodies{wheel_body(m, "geom_front_contact"),
                                    wheel_body(m, "geom_rear_contact")};
    std::array<bike::gpu::GpuWheelChain, 2> chains{};
    for (std::size_t s = 0; s < 2; ++s) chains[s] = bike::gpu::build_wheel_chain(m, bodies[s]);
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    std::vector<double> jp_ref(3 * nv), jr_ref(3 * nv), jp(3 * nv), jr(3 * nv);
    std::mt19937_64 rng(20261103u);
    std::uniform_real_distribution<double> jitter(-.05, .05);
    const std::size_t rows = ep.artifact.state_qpos.shape[0];
    double worst = 0.;
    for (const std::size_t k : {std::size_t{0}, rows / 2, rows - 1}) {
        for (int rep = 0; rep < 4; ++rep) {
            std::vector<double> qpos = state_row(ep.artifact.state_qpos, k, ep.nq);
            if (rep > 0)
                for (const auto &chain : chains)
                    for (int c = 0; c < chain.joint_count; ++c)
                        qpos[static_cast<std::size_t>(m->jnt_qposadr[chain.joint_id[c]])] += jitter(rng);
            st.set_state(qpos, state_row(ep.artifact.state_qvel, k, ep.nv),
                         state_row(ep.artifact.state_act, k, ep.na),
                         state_row(ep.artifact.state_warmstart, k, ep.nv), ep.time[k]);
            for (std::size_t s = 0; s < 2; ++s) {
                const int b = bodies[s];
                const std::array<double, 3> centre{d->xpos[3 * b], d->xpos[3 * b + 1], d->xpos[3 * b + 2]};
                for (const std::array<double, 3> off : {std::array<double, 3>{0., 0., 0.},
                                                        std::array<double, 3>{.1, .02, -.3}}) {
                    const std::array<double, 3> p{centre[0] + off[0], centre[1] + off[1], centre[2] + off[2]};
                    mj_jac(m, d, jp_ref.data(), jr_ref.data(), p.data(), b);
                    const auto &ch = chains[s];
                    bt::require(bike::physics_core::chain_point_jacobian<double>(
                                    ch.joint_count, ch.type, ch.joint_id, ch.dof_id, d->xanchor,
                                    d->xaxis, p.data(), m->nv, jp.data(), jr.data()) ==
                                    bike::physics_core::Status::ok,
                                "chain Jacobian status");
                    for (std::size_t i = 0; i < 3 * nv; ++i)
                        worst = std::max({worst, std::abs(jp[i] - jp_ref[i]), std::abs(jr[i] - jr_ref[i])});
                }
            }
        }
    }
    std::cout << "max |chain - mj_jac| = " << worst << '\n';
    bt::require(worst <= 1e-12, "chain Jacobian within 1e-12 of mj_jac (R3, S4)");
}

} // namespace

int main() {
    return bt::run([] { test_chain_jacobian_matches_mj_jac(); });
}
```
Create `native/tests/reference/GpuSharedCore.cmake`:
```cmake
# Stage 1 GPU shared-core contracts on the stand40 fixture model.
bike_add_reference_test(gpu_tyre_core ENGINE
  SOURCES test_gpu_tyre_core.cpp LIBRARIES bike_app
  LABELS tyre gpu)
```
Then add `include("${CMAKE_CURRENT_SOURCE_DIR}/GpuSharedCore.cmake")` after `native/tests/reference/CMakeLists.txt:45`.

The `gpu` label puts the node into G3.

- [ ] **Step 8.2: Run to verify it fails.** Run `cmake --preset release && cmake --build build/native-release --target reference.gpu_tyre_core`. Expected: `fatal error: 'gpu/tyre_upload.hpp' file not found`.

- [ ] **Step 8.3: Implement.**

  Append to `physics_core/kinematics.hpp`, before the closing namespace:
```cpp
BIKE_CONSTEXPR int kJointSlide = 2;
BIKE_CONSTEXPR int kJointHinge = 3;

// Exact point Jacobian of `point` on a body whose ancestor dof chain is
// (type, joint_id, dof_id): slide → jp = axis; hinge → jp = axis × (p -
// anchor), jr = axis. Equal to mj_jac up to rounding (R3, Task 8); needs no
// sin/cos, so Metal and host float agree bitwise (plan P2).
template <class Real>
BIKE_HD Status chain_point_jacobian(int joint_count, const BIKE_CORE_TABLE int *type,
                                    const BIKE_CORE_TABLE int *joint_id,
                                    const BIKE_CORE_TABLE int *dof_id,
                                    const BIKE_CORE_TABLE Real *xanchor,
                                    const BIKE_CORE_TABLE Real *xaxis,
                                    const BIKE_THREAD Real *point, int nv,
                                    BIKE_THREAD Real *jp, BIKE_THREAD Real *jr) noexcept {
    for (int i = 0; i < 3 * nv; ++i) {
        jp[i] = Real(0);
        jr[i] = Real(0);
    }
    for (int c = 0; c < joint_count; ++c) {
        const int j = joint_id[c];
        const int v = dof_id[c];
        const Real ax = xaxis[3 * j];
        const Real ay = xaxis[3 * j + 1];
        const Real az = xaxis[3 * j + 2];
        if (type[c] == kJointSlide) {
            jp[v] = ax;
            jp[nv + v] = ay;
            jp[2 * nv + v] = az;
        } else if (type[c] == kJointHinge) {
            const Real rx = point[0] - xanchor[3 * j];
            const Real ry = point[1] - xanchor[3 * j + 1];
            const Real rz = point[2] - xanchor[3 * j + 2];
            jp[v] = ay * rz - az * ry;
            jp[nv + v] = az * rx - ax * rz;
            jp[2 * nv + v] = ax * ry - ay * rx;
            jr[v] = ax;
            jr[nv + v] = ay;
            jr[2 * nv + v] = az;
        } else {
            return Status::chain_unsupported_joint;
        }
    }
    return Status::ok;
}
```
  Create `native/src/gpu/tyre_model.hpp`:
```cpp
// native/src/gpu/tyre_model.hpp — POD layout of the per-model GPU tyre
// upload (plan P7). Built on the host by gpu/tyre_upload.cpp; read by the
// shared core through gpu/tyre_step.hpp on Metal, CUDA and host float.
#pragma once
#include "common.hpp"
#include "../physics_core/capacity.hpp"
#include "../physics_core/kinematics.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

using ::bike::physics_core::kMaxChainJoints;

// The wheel body's ancestor dof chain, root first (build_wheel_chain).
struct GpuWheelChain {
    int joint_count = 0;
    int type[kMaxChainJoints] = {};
    int joint_id[kMaxChainJoints] = {};
    int dof_id[kMaxChainJoints] = {};
};

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
```
  Create `native/src/gpu/tyre_upload.hpp`:
```cpp
// native/src/gpu/tyre_upload.hpp — host-only builder of the GPU tyre upload
// from the same mjModel/mjData/TireConfig the CPU writer is built from
// (D11, plan P7, P8). Throws std::invalid_argument on a model the GPU core
// cannot represent.
#pragma once
#include "tyre_model.hpp"
#include <mujoco/mujoco.h>

namespace bike::gpu {
[[nodiscard]] GpuWheelChain build_wheel_chain(const mjModel *m, int body);
} // namespace bike::gpu
```
  Create `native/src/gpu/tyre_upload.cpp`:
```cpp
// native/src/gpu/tyre_upload.cpp — see tyre_upload.hpp.
#include "tyre_upload.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>
#include <stdexcept>

namespace bike::gpu {

static_assert(::bike::physics_core::kJointSlide == mjJNT_SLIDE);
static_assert(::bike::physics_core::kJointHinge == mjJNT_HINGE);

GpuWheelChain build_wheel_chain(const mjModel *m, int body) {
    if (m == nullptr || body <= 0 || body >= m->nbody)
        throw std::invalid_argument("wheel chain needs a physical body");
    // Ancestors from the wheel body up to (excluding) the world body,
    // then reversed so the chain is root first — the dof order mj_jac uses.
    std::array<int, kMaxChainJoints> types{}, joints{}, dofs{};
    int count = 0;
    const std::span<const int> parent(m->body_parentid, static_cast<std::size_t>(m->nbody));
    const std::span<const int> jntadr(m->body_jntadr, static_cast<std::size_t>(m->nbody));
    const std::span<const int> jntnum(m->body_jntnum, static_cast<std::size_t>(m->nbody));
    const std::span<const int> jnt_type(m->jnt_type, static_cast<std::size_t>(m->njnt));
    const std::span<const int> jnt_dof(m->jnt_dofadr, static_cast<std::size_t>(m->njnt));
    for (int b = body; b > 0; b = parent[static_cast<std::size_t>(b)]) {
        const int first = jntadr[static_cast<std::size_t>(b)];
        // A body's own joints in reverse, so the final reversal restores
        // MuJoCo's within-body order.
        for (int k = jntnum[static_cast<std::size_t>(b)] - 1; k >= 0; --k) {
            const int j = first + k;
            const int t = jnt_type[static_cast<std::size_t>(j)];
            if (t != mjJNT_SLIDE && t != mjJNT_HINGE)
                throw std::invalid_argument("wheel chain has a free or ball joint");
            if (count == kMaxChainJoints)
                throw std::invalid_argument("wheel chain exceeds kMaxChainJoints");
            types[static_cast<std::size_t>(count)] = t;
            joints[static_cast<std::size_t>(count)] = j;
            dofs[static_cast<std::size_t>(count)] = jnt_dof[static_cast<std::size_t>(j)];
            ++count;
        }
    }
    GpuWheelChain chain{};
    chain.joint_count = count;
    for (int c = 0; c < count; ++c) {
        const std::size_t src = static_cast<std::size_t>(count - 1 - c);
        chain.type[c] = types[src];
        chain.joint_id[c] = joints[src];
        chain.dof_id[c] = dofs[src];
    }
    return chain;
}

} // namespace bike::gpu
```
  Add `native/src/gpu/tyre_upload.cpp` to the `bike_core` source list right after `tyre/station.cpp` (`CMakeLists.txt:129`). Use the same `"${_bike_native_dir}/src/…"` spelling as its neighbours. The legacy `native/CMakeLists.txt:206-214` `bike_native_contract_core` list compiles `tyre/{profile,distance,station}.cpp` for the binding build. Nothing there calls the upload, so `tyre_upload.cpp` is not added to it.

- [ ] **Step 8.4: Run.** Run:
  ```bash
  cmake --build build/native-release --target reference.gpu_tyre_core && \
    ctest --test-dir build/native-release -R '^reference\.gpu_tyre_core$' -V
  ```
  Expected: `max |chain - mj_jac| = <x>` with x ≤ 1e-12, and the test passes.
  - Record the printed `nq/nv/nbody/njnt` line in the commit body.
  - If `nv > 32` or `nbody > 32`, raise `kMaxNq`/`kMaxNv`/`kMaxNbody` in `gpu/common.hpp:57-60` to 64. Rerun G3; the `GpuModelIndices` layout is unaffected because these are capacities, not fields. Above 64 → **S8**.
  - `x > 1e-12` → **S4**.
  - A `"wheel chain has a free or ball joint"` throw is the same stop (**S4**): the dof-chain assumption does not hold for this model.

- [ ] **Step 8.5: Commit.** Run G3 and G4 first.
```bash
git add native/src/physics_core/kinematics.hpp native/src/gpu/tyre_model.hpp native/src/gpu/tyre_upload.hpp native/src/gpu/tyre_upload.cpp CMakeLists.txt native/tests/reference
git commit -m "feat(gpu): exact dof-chain point Jacobian for the tyre core

D10/R3: slide/hinge chain from xanchor/xaxis, proven against mj_jac on
stand40 (<= 1e-12); no device sin/cos, so Metal and host float agree.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Task 9: Tyre upload — polyline, 128-station ring, model buffer (D11, D12)

**Files:**
- Modify: `native/src/gpu/tyre_model.hpp` (add the remaining POD types)
- Modify: `native/src/gpu/tyre_upload.{hpp,cpp}` (add `build_tyre_upload`, `make_gpu_tyre_params`, `pack_polyline`)
- Modify:
  - `native/src/tyre/station.{hpp,cpp}`: add `biketyre::ring_local`;
  - `native/src/writers/distributed_tire.cpp:228-238`: the writer constructor uses `ring_local`.
- Modify: `native/tests/reference/test_gpu_tyre_core.cpp`

**Interfaces (produced):**
```cpp
namespace biketyre {
// local[i] = (r*np.cos(angle_i), 0, r*np.sin(angle_i)) — distributed_tire_forces.py:50-51.
[[nodiscard]] std::vector<std::array<double, 3>> ring_local(double radius, const std::vector<double> &angles);
}
namespace bike::gpu {
struct GpuTyreParams { int knot_count; float knots_m[kMaxHingeKnots]; float stiffness_n_m[kMaxHingeKnots];
                       float damping_ns_m; float tangent_k_n_m; float mu; float relaxation_length_m; };
struct GpuSurfaceSection { float start_m, end_m, mu_peak, mu_slide, stribeck_speed_mps; };
struct GpuTyreWheel { int body; int station_count; float radius_m; float station_weight[kMaxGpuTyreStations];
                      float ring_local[3 * kMaxGpuTyreStations]; GpuWheelChain chain; };
struct GpuTyreModel { GpuTyreWheel wheels[2]; int njnt; int surface_map_enabled; GpuSurfaceSection default_surface;
                      int section_count; GpuSurfaceSection sections[kMaxSurfaceSections]; int polyline_vertex_count; };
struct GpuTyreStationState { float xi; float tangent[3]; int flags; };
struct GpuTyreWheelState { GpuTyreStationState stations[kMaxGpuTyreStations]; };
struct GpuTyreWheelOutput { float normal_load_n; float effective_radius_m; int active_stations; };
BIKE_CONSTEXPR int kStationActive = 1, kStationHasTangent = 2, kStationWritten = 4;
[[nodiscard]] inline int polyline_floats(int vertex_count) { return 2 * vertex_count + 5 * (vertex_count - 1); }

struct TyreUpload { GpuTyreModel model; std::vector<float> polyline; };
[[nodiscard]] std::vector<float> pack_polyline(const biketyre::SignedProfile &profile);
[[nodiscard]] GpuTyreParams make_gpu_tyre_params(const nativecfg::TireConfig &cfg, int side);
[[nodiscard]] TyreUpload build_tyre_upload(const mjModel *m, const mjData *d, const nativecfg::TireConfig &cfg);
}
```
The packed polyline layout is `[x(n) z(n) edge_x(n-1) edge_z(n-1) length2(n-1) normal_x(n-1) normal_z(n-1)]`. Each entry is `static_cast<float>` of the `SignedProfile` array (D11: no geometry is recomputed in float).

- [ ] **Step 9.1: Write the failing upload test.** Append to `test_gpu_tyre_core.cpp` (includes `tyre/distance.hpp`, `tyre/profile.hpp`, `tyre/station.hpp`, `<bit>`):

```cpp
// The upload is the CPU writer's own geometry, float-cast: same polyline
// arrays, same ring (np_cos/np_sin), same weights, same bodies/radii, and the
// default GpuTyreParams equal the env fixture's resolved tyre config.
[[gnu::noinline]] void test_upload_is_the_cpu_geometry() {
    Stepper st((bundle("episodes/stand40") / "model.mjb").string(), load_config(bundle("env")));
    const nativecfg::TireConfig cfg = *load_config(bundle("env")).tire;
    const bike::gpu::TyreUpload up = bike::gpu::build_tyre_upload(st.model(), st.data(), cfg);
    const biketyre::SignedProfile profile(biketyre::compiled_profile_vertices(st.model(), st.data(), "terrain"));
    const auto v = profile.view();
    const int n = v.vertex_count;
    bt::require(up.model.polyline_vertex_count == n, "vertex count");
    bt::require(static_cast<int>(up.polyline.size()) == bike::gpu::polyline_floats(n), "packed size");
    const std::array<const double *, 7> cols{v.x, v.z, v.edge_x, v.edge_z, v.length2, v.normal_x, v.normal_z};
    std::size_t at = 0;
    for (std::size_t c = 0; c < cols.size(); ++c) {
        const int len = c < 2 ? n : n - 1;
        for (int i = 0; i < len; ++i, ++at)
            bt::require(std::bit_cast<std::uint32_t>(up.polyline[at]) ==
                            std::bit_cast<std::uint32_t>(static_cast<float>(cols[c][i])),
                        "polyline float cast");
    }
    const biketyre::StationAngles stations = biketyre::station_angles(cfg.distributed->station_count);
    for (int s = 0; s < 2; ++s) {
        const auto &w = up.model.wheels[s];
        bt::require(w.station_count == cfg.distributed->station_count && w.station_count <= 128, "station count");
        const auto ring = biketyre::ring_local(static_cast<double>(st.model()->geom_size[3 * mj_name2id(
                              st.model(), mjOBJ_GEOM, s == 0 ? "geom_front_contact" : "geom_rear_contact")]),
                          stations.angles);
        for (int i = 0; i < w.station_count; ++i) {
            bt::require(w.station_weight[i] == static_cast<float>(stations.weights[static_cast<std::size_t>(i)]), "weight");
            for (int k = 0; k < 3; ++k)
                bt::require(w.ring_local[3 * i + k] == static_cast<float>(ring[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)]), "ring");
        }
    }
    bt::require(up.model.njnt == st.model()->njnt, "njnt travels in the model");
    bt::require(up.model.surface_map_enabled == 1 && up.model.section_count == 3, "track surface map (3 scree sections)");
    for (int s = 0; s < 2; ++s) {
        const bike::gpu::GpuTyreParams got = bike::gpu::make_gpu_tyre_params(cfg, s);
        const bike::gpu::GpuTyreParams want{};
        bt::require(std::memcmp(&got, &want, sizeof got) == 0, "default GpuTyreParams equal the env config");
    }
}
```
Add it to `main`'s `bt::run` lambda. `memcmp` is valid here because `GpuTyreParams` is all `int`/`float` with no padding; a `static_assert(sizeof(GpuTyreParams) == 4 * (1 + 2 * kMaxHingeKnots + 4))` in `tyre_model.hpp` proves it.

- [ ] **Step 9.2: Run to verify it fails.** Build `reference.gpu_tyre_core`. Expected: `no member named 'build_tyre_upload' in namespace 'bike::gpu'`.

- [ ] **Step 9.3: Implement.**

  `tyre/station.hpp` and `tyre/station.cpp`: add `ring_local`. Its body is the writer's construction loop (`distributed_tire.cpp:233-238`) verbatim:
```cpp
    std::vector<std::array<double, 3>> ring_local(double radius, const std::vector<double> &angles) {
        // distributed_tire_forces.py:50-51 — column_stack((r*cos, 0, r*sin));
        // numeric::np_cos/np_sin keep the two libm calls separate.
        std::vector<std::array<double, 3>> local(angles.size());
        for (std::size_t i = 0; i < angles.size(); ++i)
            local[i] = {radius * numeric::np_cos(angles[i]), 0.0, radius * numeric::np_sin(angles[i])};
        return local;
    }
```
  `station.cpp` gains `#include "../numeric_sincos.hpp"` and `<array>`. In `distributed_tire.cpp:233-238` replace the `for s … local_[s].resize(n); for i …` loop with `for (std::size_t s = 0; s < 2; ++s) local_[s] = biketyre::ring_local(radii_[s], stations_.angles);`. Same expressions, same bits; G1 proves it.

  `tyre_model.hpp`: add after `GpuWheelChain`:
```cpp
using ::bike::physics_core::kMaxGpuTyreStations;
using ::bike::physics_core::kMaxSurfaceSections;

// Tunable per-world tyre parameters (GpuWorldConfig). Defaults equal the
// resolved env fixture (tests/fixtures/golden/<platform>/native_distributed_tire/env/wire.json)
// and nativecfg defaults (configuration/physical_config.hpp:54-58).
struct GpuTyreParams {
    int knot_count = 3;
    float knots_m[kMaxHingeKnots] = {0.0f, 0.004f, 0.012f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float stiffness_n_m[kMaxHingeKnots] = {300000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float damping_ns_m = 2000.0f;
    float tangent_k_n_m = 20000.0f;
    float mu = 1.1f;
    float relaxation_length_m = 0.2f;
};
static_assert(sizeof(GpuTyreParams) == 4 * (1 + 2 * kMaxHingeKnots + 4), "no padding");

struct GpuSurfaceSection {
    float start_m = 0.0f;
    float end_m = 0.0f;
    float mu_peak = 0.0f;
    float mu_slide = 0.0f;
    float stribeck_speed_mps = 1.0f;
};

struct GpuTyreWheel {
    int body = 0;
    int station_count = 0;
    float radius_m = 0.0f;
    float station_weight[kMaxGpuTyreStations] = {};
    float ring_local[3 * kMaxGpuTyreStations] = {};
    GpuWheelChain chain{};
};

struct GpuTyreModel {
    GpuTyreWheel wheels[2]{};
    int njnt = 0;
    int surface_map_enabled = 0;
    GpuSurfaceSection default_surface{};
    int section_count = 0;
    GpuSurfaceSection sections[kMaxSurfaceSections]{};
    int polyline_vertex_count = 0;
};

BIKE_CONSTEXPR int kStationActive = 1;
BIKE_CONSTEXPR int kStationHasTangent = 2;
BIKE_CONSTEXPR int kStationWritten = 4;

struct GpuTyreStationState {
    float xi = 0.0f;
    float tangent[3] = {0.0f, 0.0f, 0.0f};
    int flags = 0;
};

struct GpuTyreWheelState {
    GpuTyreStationState stations[kMaxGpuTyreStations]{};
};

struct GpuTyreWheelOutput {
    float normal_load_n = 0.0f;
    float effective_radius_m = 0.0f;
    int active_stations = 0;
};

// Floats in the packed polyline buffer for `vertex_count` vertices.
BIKE_HD int polyline_floats(int vertex_count) noexcept {
    return 2 * vertex_count + 5 * (vertex_count - 1);
}
```
  `static_assert` with a message is valid MSL (C++14). `kMaxHingeKnots` is visible through `common.hpp` (Task 2 re-exports it).

  `tyre_upload.hpp`: add `#include "../config_types.hpp"`, `#include "../tyre/distance.hpp"` and `<vector>`, plus the `TyreUpload` struct and three declarations from Interfaces. `tyre_upload.cpp`:
```cpp
std::vector<float> pack_polyline(const biketyre::SignedProfile &profile) {
    const auto v = profile.view();
    const int n = v.vertex_count;
    std::vector<float> out;
    out.reserve(static_cast<std::size_t>(polyline_floats(n)));
    const auto push = [&out](const double *col, int len) {
        for (const double x : std::span<const double>(col, static_cast<std::size_t>(len)))
            out.push_back(static_cast<float>(x));
    };
    push(v.x, n);
    push(v.z, n);
    push(v.edge_x, n - 1);
    push(v.edge_z, n - 1);
    push(v.length2, n - 1);
    push(v.normal_x, n - 1);
    push(v.normal_z, n - 1);
    return out;
}

GpuTyreParams make_gpu_tyre_params(const nativecfg::TireConfig &cfg, int side) {
    if (!cfg.distributed || cfg.backend != "distributed_2d_reference")
        throw std::invalid_argument("GPU tyre needs the distributed_2d_reference backend");
    const nativecfg::DistributedMaterial &mat = side == 0 ? cfg.distributed->front : cfg.distributed->rear;
    const nativecfg::TireParams &p = side == 0 ? cfg.front : cfg.rear;
    const std::size_t knots = std::min(mat.density.knots_m.size(), mat.density.stiffness_n_m.size());
    if (knots > static_cast<std::size_t>(kMaxHingeKnots))
        throw std::invalid_argument("GPU tyre density exceeds kMaxHingeKnots");
    GpuTyreParams out{};
    out.knot_count = static_cast<int>(knots);
    for (std::size_t i = 0; i < static_cast<std::size_t>(kMaxHingeKnots); ++i) {
        out.knots_m[i] = i < knots ? static_cast<float>(mat.density.knots_m[i]) : 0.0f;
        out.stiffness_n_m[i] = i < knots ? static_cast<float>(mat.density.stiffness_n_m[i]) : 0.0f;
    }
    out.damping_ns_m = static_cast<float>(mat.density.damping_ns_m);
    out.tangent_k_n_m = static_cast<float>(p.tangent_k_n_m);
    out.mu = static_cast<float>(p.mu);
    out.relaxation_length_m = static_cast<float>(p.relaxation_length_m);
    return out;
}

TyreUpload build_tyre_upload(const mjModel *m, const mjData *d, const nativecfg::TireConfig &cfg) {
    // The writer's own construction checks (distributed_tire.cpp:168-231),
    // restricted to what the GPU reads; nativecfg::validate stays the CPU
    // writer's (the same cfg must also build the CPU writer).
    (void)make_gpu_tyre_params(cfg, 0);
    TyreUpload up{};
    const biketyre::SignedProfile profile(biketyre::compiled_profile_vertices(m, d, "terrain"));
    up.polyline = pack_polyline(profile);
    up.model.polyline_vertex_count = profile.view().vertex_count;
    up.model.njnt = static_cast<int>(m->njnt);
    const biketyre::StationAngles stations = biketyre::station_angles(cfg.distributed->station_count);
    const int n = static_cast<int>(stations.angles.size());
    if (n > kMaxGpuTyreStations)
        throw std::invalid_argument("GPU tyre supports at most kMaxGpuTyreStations stations");
    constexpr std::array<const char *, 2> kGeoms = {"geom_front_contact", "geom_rear_contact"};
    for (std::size_t s = 0; s < 2; ++s) {
        const int g = mj_name2id(m, mjOBJ_GEOM, kGeoms[s]);
        if (g < 0)
            throw std::invalid_argument("GPU tyre needs the wheel contact geoms");
        const std::size_t g3 = 3 * static_cast<std::size_t>(g);
        // The GPU centre is the body xpos; it equals geom_xpos only for a
        // geom at the body origin (plan P7).
        if (m->geom_pos[g3] != 0.0 || m->geom_pos[g3 + 1] != 0.0 || m->geom_pos[g3 + 2] != 0.0)
            throw std::invalid_argument("GPU tyre needs the contact geom at the wheel body origin");
        GpuTyreWheel &w = up.model.wheels[s];
        w.body = m->geom_bodyid[g];
        w.station_count = n;
        const double radius = m->geom_size[g3];
        w.radius_m = static_cast<float>(radius);
        const auto ring = biketyre::ring_local(radius, stations.angles);
        for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) {
            w.station_weight[i] = static_cast<float>(stations.weights[i]);
            for (std::size_t k = 0; k < 3; ++k) w.ring_local[3 * i + k] = static_cast<float>(ring[i][k]);
        }
        w.chain = build_wheel_chain(m, w.body);
    }
    const auto spec = [](const nativecfg::SurfaceSpec &sp, double a, double b) {
        return GpuSurfaceSection{static_cast<float>(a), static_cast<float>(b), static_cast<float>(sp.mu_peak),
                                 static_cast<float>(sp.mu_slide), static_cast<float>(sp.stribeck_speed_mps)};
    };
    if (cfg.surface_mode == "track") {
        if (!cfg.surface_map)
            throw std::invalid_argument("track material mode needs the actual track surface map");
        auto sections = cfg.surface_map->sections;
        std::ranges::sort(sections, {}, [](const auto &sec) { return sec.start_m; });
        if (sections.size() > static_cast<std::size_t>(kMaxSurfaceSections))
            throw std::invalid_argument("GPU tyre supports at most kMaxSurfaceSections sections");
        up.model.surface_map_enabled = 1;
        up.model.default_surface = spec(cfg.surface_map->surface, 0.0, 0.0);
        up.model.section_count = static_cast<int>(sections.size());
        for (std::size_t i = 0; i < sections.size(); ++i)
            up.model.sections[i] = spec(sections[i].surface, sections[i].start_m, sections[i].end_m);
    }
    return up;
}
```
  The writer's sort is `std::ranges::sort(… sections, {}, start_m)` (`distributed_tire.cpp:190-191`). `nativecfg::validate` forbids overlapping sections (`config_validation.hpp:344-358`), so equal `start_m` keys cannot occur and the unstable sort's order is unique. Add the includes `../tyre/profile.hpp`, `../tyre/station.hpp`, `<algorithm>`, `<cstring>`, `<string>`, `<vector>`.

- [ ] **Step 9.4: Run.** Build all. Then run `ctest -R '^reference\.gpu_tyre_core$' -V`: both cases pass. Then **G1**: `ring_local` is now the writer's ring, so a failure here → **S1**. Then **G3** and **G4**.

- [ ] **Step 9.5: Commit.** Message: `feat(gpu): build the GPU tyre upload from the CPU writer geometry`. Body: "D11/D12: the polyline is SignedProfile's arrays float-cast, the ring is biketyre::ring_local (shared with the writer), 128 stations, the dof chain, the surface map; tunable params stay per world." Then the trailer.

---

## Task 10: GPU tyre step `<float, GpuToolkit>` (`gpu/tyre_step.hpp`)

**Files:**
- Create: `native/src/gpu/tyre_step.hpp`
- Modify: `native/tests/reference/test_gpu_tyre_core.cpp`

**Interfaces (produced):**
```cpp
namespace bike::gpu {
BIKE_HD physics_core::PolylineView<float> gpu_polyline_view(const BIKE_CORE_TABLE float *poly, int vertex_count) noexcept;
struct GpuSurfaceMapAccess { const BIKE_CORE_TABLE GpuTyreModel *model; /* five accessors */ };
struct GpuTyreIo { /* in-place state writes + load/effective-radius accumulators */ };
BIKE_HD physics_core::Status gpu_tyre_wheel_step(const BIKE_CORE_TABLE GpuTyreModel &model, int side,
    const BIKE_CORE_TABLE float *polyline, const BIKE_THREAD GpuTyreParams &p,
    const BIKE_THREAD float *xpos_w, const BIKE_CORE_TABLE float *xmat_w,
    const BIKE_CORE_TABLE float *xanchor_w, const BIKE_CORE_TABLE float *xaxis_w,
    const BIKE_THREAD float *qvel_w, int nv, float dt, BIKE_THREAD GpuTyreWheelState &state,
    BIKE_THREAD float *qfrc, BIKE_THREAD GpuTyreWheelOutput &out) noexcept;
BIKE_HD physics_core::Status gpu_tyre_step(/* same, both wheels: front then rear; zeroes qfrc_tyre first */) noexcept;
}
```

- [ ] **Step 10.1: Write the failing host-float vs CPU-double test.** Append to `test_gpu_tyre_core.cpp` (include `gpu/tyre_step.hpp`):

```cpp
// Host float (the exact code Metal runs) against the CPU double writer on
// the stand40 episode rows, both advancing their own bristle state from
// empty. Tolerance per dof: |gpu - cpu| <= 1e-2 * max|cpu| + 1e-3 N (S5).
[[gnu::noinline]] void test_host_float_tracks_cpu_double() {
    const auto ep = load_episode_bundle(bundle("episodes/stand40"));
    const nativecfg::WriterConfig wcfg = load_config(bundle("env"));
    Stepper st((bundle("episodes/stand40") / "model.mjb").string(), wcfg);
    const mjModel *m = st.model();
    const mjData *d = st.data();
    const bike::gpu::TyreUpload up = bike::gpu::build_tyre_upload(m, d, *wcfg.tire);
    const std::array<bike::gpu::GpuTyreParams, 2> params{bike::gpu::make_gpu_tyre_params(*wcfg.tire, 0),
                                                         bike::gpu::make_gpu_tyre_params(*wcfg.tire, 1)};
    std::array<bike::gpu::GpuTyreWheelState, 2> state{};
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    const auto f32 = [](const mjtNum *src, std::size_t count) {
        std::vector<float> v(count);
        for (std::size_t i = 0; i < count; ++i) v[i] = static_cast<float>(src[i]);
        return v;
    };
    const double dt = .00125;
    double worst_ratio = 0.;
    for (std::size_t k = 0; k < ep.artifact.state_qpos.shape[0]; ++k) {
        st.set_state(state_row(ep.artifact.state_qpos, k, ep.nq), state_row(ep.artifact.state_qvel, k, ep.nv),
                     state_row(ep.artifact.state_act, k, ep.na),
                     state_row(ep.artifact.state_warmstart, k, ep.nv), ep.time[k]);
        const std::vector<double> cpu = st.tire_qfrc(dt);
        const auto xpos = f32(d->xpos, 3 * static_cast<std::size_t>(m->nbody));
        const auto xmat = f32(d->xmat, 9 * static_cast<std::size_t>(m->nbody));
        const auto xanchor = f32(d->xanchor, 3 * static_cast<std::size_t>(m->njnt));
        const auto xaxis = f32(d->xaxis, 3 * static_cast<std::size_t>(m->njnt));
        const auto qvel = f32(d->qvel, nv);
        std::vector<float> gpu(nv, 0.f);
        std::array<bike::gpu::GpuTyreWheelOutput, 2> out{};
        bt::require(bike::gpu::gpu_tyre_step(up.model, up.polyline.data(), params[0], params[1], xpos.data(),
                                             xmat.data(), xanchor.data(), xaxis.data(), qvel.data(), m->nv,
                                             static_cast<float>(dt), state.data(), gpu.data(), out.data()) ==
                        bike::physics_core::Status::ok,
                    "gpu tyre step status at row " + std::to_string(k));
        double scale = 0.;
        for (const double c : cpu) scale = std::max(scale, std::abs(c));
        for (std::size_t i = 0; i < nv; ++i) {
            const double err = std::abs(static_cast<double>(gpu[i]) - cpu[i]);
            const double tol = 1e-2 * scale + 1e-3;
            worst_ratio = std::max(worst_ratio, err / tol);
            if (err > tol)
                std::cerr << "row " << k << " dof " << i << " cpu " << cpu[i] << " gpu " << gpu[i] << '\n';
        }
    }
    std::cout << "worst |gpu-cpu|/tol = " << worst_ratio << '\n';
    bt::require(worst_ratio <= 1., "host float within tolerance of CPU double (S5)");
}
```
Add it to `main`. `Stepper::set_state` runs `mj_resetData` plus `mj_forward` (`stepper.cpp:570-577`). It does not touch the tyre writer's station state, so both sides advance their own state. That is the property under test. If the CPU throws "tire state can advance only once per increasing timestamp", the rows' `time` is not strictly increasing; report it as **S5** with the row times.

- [ ] **Step 10.2: Run to verify it fails.** Expected: `'gpu/tyre_step.hpp' file not found`.

- [ ] **Step 10.3: Create `gpu/tyre_step.hpp`.**

```cpp
// native/src/gpu/tyre_step.hpp — the GPU instantiation of the shared tyre
// core: <float, GpuToolkit> over the per-model GpuTyreModel, the packed
// polyline and engine kinematics (xpos, xmat, xanchor, xaxis). No tyre law
// lives here; this file adapts storage to physics_core/tyre.hpp (D9).
#pragma once
#include "common.hpp"
#include "tyre_model.hpp"
#include "../physics_core/gpu_toolkit.hpp"
#include "../physics_core/kinematics.hpp"
#include "../physics_core/status.hpp"
#include "../physics_core/terrain.hpp"
#include "../physics_core/tyre.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

namespace pc = ::bike::physics_core;

BIKE_HD pc::PolylineView<float> gpu_polyline_view(const BIKE_CORE_TABLE float *poly,
                                                  int vertex_count) noexcept {
    const int n = vertex_count;
    const int s = n - 1;
    pc::PolylineView<float> v{};
    v.x = poly;
    v.z = poly + n;
    v.edge_x = poly + 2 * n;
    v.edge_z = poly + 2 * n + s;
    v.length2 = poly + 2 * n + 2 * s;
    v.normal_x = poly + 2 * n + 3 * s;
    v.normal_z = poly + 2 * n + 4 * s;
    v.vertex_count = n;
    return v;
}

struct GpuSurfaceMapAccess {
    const BIKE_CORE_TABLE GpuTyreModel *model;
    BIKE_HD int section_count() const noexcept { return model->section_count; }
    BIKE_HD float section_start(int i) const noexcept { return model->sections[i].start_m; }
    BIKE_HD float section_end(int i) const noexcept { return model->sections[i].end_m; }
    BIKE_HD pc::SurfaceSpecPod<float> section_spec(int i) const noexcept {
        const pc::SurfaceSpecPod<float> r{model->sections[i].mu_peak, model->sections[i].mu_slide,
                                          model->sections[i].stribeck_speed_mps};
        return r;
    }
    BIKE_HD pc::SurfaceSpecPod<float> default_spec() const noexcept {
        const pc::SurfaceSpecPod<float> r{model->default_surface.mu_peak, model->default_surface.mu_slide,
                                          model->default_surface.stribeck_speed_mps};
        return r;
    }
};

// Writes each active station's new state in place (each sid is visited
// once, after its old_state read) and accumulates the two outputs the
// drivetrain consumes. The effective radius is the CPU's load-weighted
// mean contact drop (distributed_tire.cpp:135-155) in plain float sums.
struct GpuTyreIo {
    BIKE_THREAD GpuTyreWheelState *state;
    const BIKE_THREAD float *axis;
    float load_sum;
    float terms_loaded;
    float weights_loaded;
    float terms_unit;
    float weights_unit;
    bool any_loaded;

    BIKE_HD pc::StationState<float> old_state(int sid) const noexcept {
        const BIKE_THREAD GpuTyreStationState &s = state->stations[sid];
        pc::StationState<float> r{};
        r.active = (s.flags & kStationActive) != 0;
        r.xi = s.xi;
        r.has_tangent = (s.flags & kStationHasTangent) != 0;
        r.tangent[0] = s.tangent[0];
        r.tangent[1] = s.tangent[1];
        r.tangent[2] = s.tangent[2];
        return r;
    }

    BIKE_HD void station(const BIKE_THREAD pc::StationResult<float> &r) noexcept {
        BIKE_THREAD GpuTyreStationState &s = state->stations[r.sid];
        s.xi = r.xi_new;
        s.tangent[0] = r.tangent[0];
        s.tangent[1] = r.tangent[1];
        s.tangent[2] = r.tangent[2];
        s.flags = kStationActive | kStationHasTangent | kStationWritten;
        load_sum += r.load;
        const float drop = -(((r.point[0] - axis[0]) * r.normal[0] + (r.point[1] - axis[1]) * r.normal[1]) +
                             (r.point[2] - axis[2]) * r.normal[2]);
        terms_loaded += r.load * drop;
        weights_loaded += r.load;
        terms_unit += drop;
        weights_unit += 1.0f;
        any_loaded = any_loaded || r.load > 0.0f;
    }
};

BIKE_HD pc::Status gpu_tyre_wheel_step(const BIKE_CORE_TABLE GpuTyreModel &model, int side,
                                       const BIKE_CORE_TABLE float *polyline,
                                       const BIKE_THREAD GpuTyreParams &p,
                                       const BIKE_THREAD float *xpos_w, const BIKE_CORE_TABLE float *xmat_w,
                                       const BIKE_CORE_TABLE float *xanchor_w,
                                       const BIKE_CORE_TABLE float *xaxis_w,
                                       const BIKE_THREAD float *qvel_w, int nv, float dt,
                                       BIKE_THREAD GpuTyreWheelState &state, BIKE_THREAD float *qfrc,
                                       BIKE_THREAD GpuTyreWheelOutput &out) noexcept {
    using pc::GpuToolkit;
    const BIKE_CORE_TABLE GpuTyreWheel &wheel = model.wheels[side];
    const int n = wheel.station_count;
    if (n < 1 || n > kMaxGpuTyreStations || nv < 0 || nv > kMaxNv || p.knot_count < 0 ||
        p.knot_count > kMaxHingeKnots || wheel.chain.joint_count > kMaxChainJoints ||
        model.polyline_vertex_count < 2) {
        return pc::Status::capacity_exceeded;
    }
    if (!(dt > 0.0f)) {
        return pc::Status::invalid_interval;
    }
    float center[3] = {xpos_w[3 * wheel.body], xpos_w[3 * wheel.body + 1], xpos_w[3 * wheel.body + 2]};
    float rotation[9];
    for (int k = 0; k < 9; ++k) {
        rotation[k] = xmat_w[9 * wheel.body + k];
    }
    float points[3 * kMaxGpuTyreStations];
    float heights[kMaxGpuTyreStations];
    int active[kMaxGpuTyreStations];
    float jp[3 * kMaxNv];
    float jr[3 * kMaxNv];
    float jac_station[3 * kMaxNv];
    float tmp[kMaxNv];
    const pc::PolylineView<float> pl = gpu_polyline_view(polyline, model.polyline_vertex_count);
    pc::WheelGeometry<float> g{};
    g.ring_local = wheel.ring_local;
    g.station_count = n;
    g.center = center;
    g.rotation = rotation;
    pc::WheelScratch<float> s{};
    s.points = points;
    s.heights = heights;
    s.active_ids = active;
    s.jac_station = jac_station;
    s.tmp = tmp;
    int count = 0;
    pc::Status st = pc::classify_stations<float, GpuToolkit>(g, pl, s, count);
    if (st != pc::Status::ok) {
        return st;
    }
    // CPU order: the height sweep (station_points) precedes point_jacobian.
    st = pc::chain_point_jacobian<float>(wheel.chain.joint_count, wheel.chain.type, wheel.chain.joint_id,
                                         wheel.chain.dof_id, xanchor_w, xaxis_w, center, nv, jp, jr);
    if (st != pc::Status::ok) {
        return st;
    }
    pc::WheelLaw<float> law{};
    law.weights = wheel.station_weight;
    law.density.knots_m = p.knots_m;
    law.density.stiffness_n_m = p.stiffness_n_m;
    law.density.knot_count = p.knot_count;
    law.density.damping_ns_m = p.damping_ns_m;
    law.tangent_k_n_m = p.tangent_k_n_m;
    law.tire_mu = p.mu;
    law.relaxation_length_m = p.relaxation_length_m;
    pc::WheelJacobian<float> jac{};
    jac.jp = jp;
    jac.jr = jr;
    jac.qvel = qvel_w;
    jac.nv = nv;
    GpuSurfaceMapAccess map{};
    map.model = &model;
    GpuTyreIo io{};
    io.state = &state;
    io.axis = center;
    st = pc::wheel_contacts<float, GpuToolkit, GpuSurfaceMapAccess, GpuTyreIo>(
        g, pl, law, jac, model.surface_map_enabled != 0 ? &map : nullptr, dt, s, count, qfrc, io);
    if (st != pc::Status::ok) {
        return st;
    }
    // new_states starts empty (distributed_tire.cpp:648): unwritten
    // stations detach.
    for (int i = 0; i < n; ++i) {
        BIKE_THREAD GpuTyreStationState &ss = state.stations[i];
        if ((ss.flags & kStationWritten) != 0) {
            ss.flags &= ~kStationWritten;
        } else {
            ss.xi = 0.0f;
            ss.tangent[0] = 0.0f;
            ss.tangent[1] = 0.0f;
            ss.tangent[2] = 0.0f;
            ss.flags = 0;
        }
    }
    out.normal_load_n = io.load_sum;
    out.active_stations = count;
    const float radius = io.any_loaded ? pc::GpuToolkit::div(io.terms_loaded, io.weights_loaded)
                                       : pc::GpuToolkit::div(io.terms_unit, io.weights_unit);
    out.effective_radius_m = count == 0 ? 0.0f : pc::max_std(0.0f, radius);
    return pc::Status::ok;
}

// Both wheels, front then rear, into one zeroed qfrc_tyre — the CPU's
// `out` accumulation order (distributed_tire.cpp:601, :615).
BIKE_HD pc::Status gpu_tyre_step(const BIKE_CORE_TABLE GpuTyreModel &model,
                                 const BIKE_CORE_TABLE float *polyline,
                                 const BIKE_THREAD GpuTyreParams &front,
                                 const BIKE_THREAD GpuTyreParams &rear,
                                 const BIKE_THREAD float *xpos_w, const BIKE_CORE_TABLE float *xmat_w,
                                 const BIKE_CORE_TABLE float *xanchor_w,
                                 const BIKE_CORE_TABLE float *xaxis_w,
                                 const BIKE_THREAD float *qvel_w, int nv, float dt,
                                 BIKE_THREAD GpuTyreWheelState *state, BIKE_THREAD float *qfrc_tyre,
                                 BIKE_THREAD GpuTyreWheelOutput *out) noexcept {
    for (int i = 0; i < nv; ++i) {
        qfrc_tyre[i] = 0.0f;
    }
    const pc::Status sf = gpu_tyre_wheel_step(model, 0, polyline, front, xpos_w, xmat_w, xanchor_w, xaxis_w,
                                              qvel_w, nv, dt, state[0], qfrc_tyre, out[0]);
    if (sf != pc::Status::ok) {
        return sf;
    }
    return gpu_tyre_wheel_step(model, 1, polyline, rear, xpos_w, xmat_w, xanchor_w, xaxis_w, qvel_w, nv, dt,
                               state[1], qfrc_tyre, out[1]);
}

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
```
Notes:
- `kMaxNv`, `kMaxHingeKnots`, `kMaxGpuTyreStations` and `kMaxChainJoints` are visible in `bike::gpu` through `common.hpp` / `tyre_model.hpp`.
- `pc::GpuToolkit` is used for `div`, so the radius division follows the same rule as every other core division.
- `namespace pc = …` at namespace scope inside a header is legal in MSL and in C++.

- [ ] **Step 10.4: Run.** Build `reference.gpu_tyre_core`, then `ctest -R '^reference\.gpu_tyre_core$' -V`. Expected: three cases pass and the output prints `worst |gpu-cpu|/tol = <r>` with r ≤ 1. r > 1 → **S5**: report the printed rows/dofs and the active-station counts (`out[s].active_stations` vs `st`'s `diagnostics`). Then run G3 and G4.

- [ ] **Step 10.5: Commit.** Message: `feat(gpu): instantiate the shared tyre core as <float, GpuToolkit>`. Body: "gpu_tyre_step adapts GpuTyreModel/polyline/engine kinematics to physics_core/tyre.hpp; host float tracks the CPU double writer on stand40 within 1e-2 relative." Then the trailer.

---
## Task 11: Batch step, CPU runner and CUDA on the shared tyre (D12)

**Files:**
- Modify: `native/src/gpu/batch_step.hpp` (whole file: config/state types, sweep builder, init, step)
- Modify:
  - `native/src/gpu/cpu_batch_runner.cpp`: ABI :81-155 and :157-260, step loop :334-420;
  - `native/src/gpu/cuda_kernels.cu`: kernel :43-94, C API :101-215 and :260-340.
- Modify: `native/src/gpu/common.hpp:61` (delete `kMaxTreadStations`), `:319-322` (comment)
- Modify: `native/tests/gpu/test_gpu_batch_step.cpp`, `native/tests/gpu/test_gpu_batch_runners.cpp`
- Delete:
  - `native/src/gpu/tire_kernel.hpp`;
  - `native/tests/gpu/test_gpu_tire.cpp`;
  - its registrations `CMakeLists.txt:696-697` and `native/CMakeLists.txt:588`.
- Modify: `CMakeLists.txt:740-752` (link `bike_core` into `bike_test_gpu_batch_runners`)

The MSL stitch list still names `tire_kernel.hpp` until Task 12. So this task removes `tire_kernel.hpp` from `metal_batch_runner.mm:103-110` too, and adds `tyre_model.hpp` and `tyre_step.hpp` plus the core headers in its place. That keeps the Apple build compiling in between. The Metal kernels themselves change in Task 12. Until then, the Metal runner object would not compile against the new `batch_step.hpp`, so **Tasks 11 and 12 are committed together**. Task 11's commit step is folded into Task 12. On Linux, Task 11 alone is a consistent state.

**Interfaces (produced):**
```cpp
namespace bike::gpu {
struct GpuWorldConfig { GpuSuspensionParams suspension; GpuTyreParams front_tyre, rear_tyre; GpuResistanceParams resistance;
                        GpuDrivetrainParams drivetrain; GpuRiderParams rider; float pedal_demand, front_brake_demand, rear_brake_demand; };
struct GpuWorldState { GpuTyreWheelState tyre[2]; GpuTyreWheelOutput tyre_out[2]; float tyre_qfrc[kMaxNv];
                       GpuDrivetrainState drivetrain; int status; int failed_step; int step_index; };
BIKE_HD void gpu_build_sweep_configs(BIKE_THREAD GpuWorldConfig *configs, int nworld, const BIKE_THREAD float *fork_psi_scale,
    const BIKE_THREAD float *rebound_scale, const BIKE_THREAD float *shock_rate_scale, const BIKE_THREAD float *tire_stiff_scale) noexcept;
BIKE_HD void gpu_step_world_writers(int w, float dt_s, const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_CORE_TABLE GpuTyreModel &tyre_model, const BIKE_CORE_TABLE float *polyline,
    const BIKE_THREAD GpuWorldConfig *configs, int per_world_config, BIKE_THREAD GpuWorldState *states,
    const BIKE_THREAD float *qpos_batch, const BIKE_THREAD float *qvel_batch, const BIKE_THREAD float *xpos_batch,
    const BIKE_CORE_TABLE float *xmat_batch, const BIKE_CORE_TABLE float *xanchor_batch, const BIKE_CORE_TABLE float *xaxis_batch,
    BIKE_THREAD float *qfrc_batch, BIKE_THREAD float *ctrl_batch, BIKE_THREAD float *tendon_range_batch,
    int tendon_range_world_stride = 2) noexcept;
}
```
There is one C ABI rule for every runner entry point, applied in this task and the next. The four heightfield parameters `const float *d_hf_heights, int hf_num_samples, float hf_x_min, float hf_dx` become three:
```cpp
const bike::gpu::GpuTyreModel *d_tyre_model, const float *d_polyline, int polyline_floats
```
Entry points that receive caller kinematics (`*_launch_writers`, `*_launch_step_writers`) also gain `const float *d_xmat, const float *d_xanchor, const float *d_xaxis` right after `d_xpos`. Sizes: `nworld*nbody*9` for `d_xmat`, and `nworld*njnt*3` for the other two, with `njnt` taken from `d_tyre_model->njnt`. `bike_gpu_build_sweep_configs` loses its trailing `int num_stations, int num_lateral_ribs`.

- [ ] **Step 11.1: Update the tests first.**

  In `native/tests/gpu/test_gpu_batch_step.cpp`:
  1. Replace `#include` of `tire_kernel.hpp` (if any) with `../../src/gpu/tyre_step.hpp`, and the header comment's `test_gpu_tire.cpp/station.cpp` mention (:9) with `tyre_step.hpp`.
  2. In `BatchScenario` (:73-96), replace `hf_heights` and `heightfield()` with:
```cpp
    std::vector<float> road_x, road_z;    // the rough profile as polyline vertices
    std::vector<float> polyline;          // packed (tyre_model.hpp layout)
    GpuTyreModel tyre_model{};
    std::vector<float> xmat_batch, xanchor_batch, xaxis_batch;
    float ground(float x) const {
        return bike::physics_core::np_interp<float, bike::physics_core::GpuToolkit>(
            x, road_x.data(), road_z.data(), static_cast<int>(road_x.size()));
    }
```
  3. Add these helpers above `make_sweep_configs`:
```cpp
// Synthetic packed polyline: the scenario's own float geometry (this test
// has no CPU SignedProfile; the CPU-built path is reference.gpu_tyre_core).
std::vector<float> pack_synthetic_polyline(const std::vector<float> &x, const std::vector<float> &z) {
    const int n = static_cast<int>(x.size());
    std::vector<float> out(static_cast<std::size_t>(polyline_floats(n)), 0.0f);
    const std::size_t un = x.size(), us = un - 1;
    for (std::size_t i = 0; i < un; ++i) { out[i] = x[i]; out[un + i] = z[i]; }
    for (std::size_t i = 0; i < us; ++i) {
        const float ex = x[i + 1] - x[i], ez = z[i + 1] - z[i];
        const float len2 = ex * ex + ez * ez, len = std::sqrt(len2);
        out[2 * un + i] = ex;
        out[2 * un + us + i] = ez;
        out[2 * un + 2 * us + i] = len2;
        out[2 * un + 3 * us + i] = -ez / len;
        out[2 * un + 4 * us + i] = ex / len;
    }
    return out;
}

// Synthetic joints mirroring the model topology (GpuModelIndices defaults):
// 0 root_x slide, 1 root_z slide, 2 root_pitch hinge, 3 fork_travel slide,
// 4 front spin hinge, 5 main_pivot hinge, 6 horst_pivot hinge, 7 rear spin hinge.
constexpr int kSynthJoints = 8;
GpuTyreModel make_synthetic_tyre_model(const GpuModelIndices &idx, int vertex_count) {
    GpuTyreModel tm{};
    tm.njnt = kSynthJoints;
    tm.polyline_vertex_count = vertex_count;
    const std::array<std::array<int, 3>, kSynthJoints> j = {{
        {0, bike::physics_core::kJointSlide, idx.root_x_dof}, {1, bike::physics_core::kJointSlide, idx.root_z_dof},
        {2, bike::physics_core::kJointHinge, idx.root_pitch_dof}, {3, bike::physics_core::kJointSlide, idx.fork_travel_dof},
        {4, bike::physics_core::kJointHinge, idx.front_wheel_spin_dof}, {5, bike::physics_core::kJointHinge, idx.main_pivot_dof},
        {6, bike::physics_core::kJointHinge, idx.horst_pivot_dof}, {7, bike::physics_core::kJointHinge, idx.rear_wheel_spin_dof}}};
    const std::array<std::vector<int>, 2> chains = {std::vector<int>{0, 1, 2, 3, 4}, std::vector<int>{0, 1, 2, 5, 6, 7}};
    const std::array<int, 2> bodies = {idx.front_wheel_body, idx.rear_wheel_body};
    constexpr int kStations = 128;
    for (std::size_t s = 0; s < 2; ++s) {
        GpuTyreWheel &w = tm.wheels[s];
        w.body = bodies[s];
        w.station_count = kStations;
        w.radius_m = 0.351f;
        for (int i = 0; i < kStations; ++i) {
            const double a = 2.0 * std::numbers::pi * i / kStations;
            w.station_weight[i] = static_cast<float>(2.0 * std::numbers::pi / kStations);
            w.ring_local[3 * i] = static_cast<float>(0.351 * std::cos(a));
            w.ring_local[3 * i + 2] = static_cast<float>(0.351 * std::sin(a));
        }
        w.chain.joint_count = static_cast<int>(chains[s].size());
        for (std::size_t c = 0; c < chains[s].size(); ++c) {
            const auto &jj = j[static_cast<std::size_t>(chains[s][c])];
            w.chain.joint_id[c] = jj[0];
            w.chain.type[c] = jj[1];
            w.chain.dof_id[c] = jj[2];
        }
    }
    return tm;
}

// xmat = identity for every body; joint anchors/axes from the world's xpos.
void refresh_joint_frames(BatchScenario &sc, std::size_t wi) {
    const GpuModelIndices &idx = sc.idx;
    const std::size_t nb = to_index(idx.nbody);
    for (std::size_t b = 0; b < nb; ++b)
        for (std::size_t k = 0; k < 9; ++k)
            sc.xmat_batch[wi * nb * 9 + b * 9 + k] = (k % 4 == 0) ? 1.0f : 0.0f;
    const auto body = [&](int b, int k) { return sc.xpos_batch[wi * nb * 3 + 3 * to_index(b) + to_index(k)]; };
    const std::array<std::array<float, 6>, kSynthJoints> frames = {{
        {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 0, 1},
        {body(idx.frame_body, 0), 0, body(idx.frame_body, 2), 0, 1, 0},
        {0, 0, 0, 0, 0, 1},
        {body(idx.front_wheel_body, 0), 0, body(idx.front_wheel_body, 2), 0, 1, 0},
        {body(idx.frame_body, 0) + kMainPivotLocalDxM, 0, body(idx.frame_body, 2) + kMainPivotLocalDzM, 0, 1, 0},
        {body(idx.rear_wheel_body, 0) + 0.05f, 0, body(idx.rear_wheel_body, 2) + 0.10f, 0, 1, 0},
        {body(idx.rear_wheel_body, 0), 0, body(idx.rear_wheel_body, 2), 0, 1, 0}}};
    for (std::size_t jn = 0; jn < kSynthJoints; ++jn)
        for (std::size_t k = 0; k < 3; ++k) {
            sc.xanchor_batch[wi * kSynthJoints * 3 + 3 * jn + k] = frames[jn][k];
            sc.xaxis_batch[wi * kSynthJoints * 3 + 3 * jn + k] = frames[jn][3 + k];
        }
}
```
  4. `make_sweep_configs` (:104-138):
     - call `bike_gpu_build_sweep_configs(configs.data(), kNWorld, fork_psi_scale.data(), rebound_scale.data(), shock_rate_scale.data(), tire_stiff_scale.data());`;
     - delete the `num_stations`/`num_lateral_ribs` loop (:131-136) and its comment lines (:98-103 "32x1 / 45x10");
     - the comment becomes "64 heterogeneous configs: fork pressure, rebound, shock rate and tyre stiffness sweeps".
  5. In `make_batch_scenario`:
     - fill `sc.road_x/road_z` from the existing rough formula (:155-158), with `x = kHfXMin + i*kHfDx`;
     - then `sc.polyline = pack_synthetic_polyline(sc.road_x, sc.road_z); sc.tyre_model = make_synthetic_tyre_model(idx, kHfSamples);`;
     - size `xmat_batch` (`kNWorld*nbody*9`) and `xanchor_batch`/`xaxis_batch` (`kNWorld*kSynthJoints*3`);
     - replace every `gpu_sample_heightfield_1d(hf, x).x` with `sc.ground(x)`, and every `sc.configs[wi].front_tire.nominal_radius_m` / `rear_tire.nominal_radius_m` with `sc.tyre_model.wheels[0].radius_m` / `[1].radius_m`;
     - call `refresh_joint_frames(sc, wi)` after each block that writes `xpos_w` (:186-235 and :281-298).
  6. Every `gpu_step_world_writers(` call (:257, :525, :530) becomes:
```cpp
            gpu_step_world_writers(
                w, kDt, idx, sc.tyre_model, sc.polyline.data(), sc.configs.data(), /*per_world_config=*/1,
                sc.states.data(), sc.qpos_batch.data(), sc.qvel_batch.data(), sc.xpos_batch.data(),
                sc.xmat_batch.data(), sc.xanchor_batch.data(), sc.xaxis_batch.data(),
                sc.qfrc_batch.data(), sc.ctrl_batch.data(), sc.tendon_range_batch.data());
```
     Apply the same change at the two `test_drivetrain_torque_path` calls, keeping their own state/qfrc/tendon arguments.
  7. `test_stepped_world_outputs` (:394-500): the assertions stay. Add:
```cpp
        require(sc.states[wi].status == 0, "no world stopped on a tyre status");
        require(sc.states[wi].tyre_out[0].active_stations > 0 && sc.states[wi].tyre_out[1].active_stations > 0,
                "both tyres touch the synthetic road");
```
  8. `test_sweep_world_differences` (:556-580): replace the ray-grid comparison (world 31 vs 32) with tyre stiffness. Worlds 0 and 63 see identical kinematics and differ by `tire_stiff_scale` 0.85 vs 1.20:
```cpp
    require(std::fabs(sc.states[63].tyre_qfrc[to_index(idx.root_z_dof)] -
                      sc.states[0].tyre_qfrc[to_index(idx.root_z_dof)]) > 1.0f,
            "tyre stiffness sweep changes the tyre force");
```
     Update the section comment accordingly.
  9. `test_launch_wrappers_and_bounds` (:582-670): apply the C ABI rule to each `bike_gpu_launch_writers` / `bike_gpu_build_sweep_configs` call and to the `extern "C"` declarations at the top of the file. For the null-argument cases, pass `nullptr, nullptr, 0` for the tyre triple.
  10. `test_api_sizes_and_model_defaults`: keep. `bike_gpu_sizeof_world_state() == sizeof(GpuWorldState)` follows the new layout.

  Add includes `<numbers>`, `../../src/gpu/tyre_step.hpp`, `../../src/physics_core/terrain.hpp` and `../../src/physics_core/gpu_toolkit.hpp`.

  In `native/tests/gpu/test_gpu_batch_runners.cpp`:
  - apply the C ABI rule to the declarations (:29-110) and every call;
  - build the upload once per test from the probe model with `bike::gpu::build_tyre_upload` (include `gpu/tyre_upload.hpp`), using this config, which is the `nativecfg` default distributed tyre (`config_types.hpp:230-283`, values as in the env fixture):
```cpp
nativecfg::TireConfig default_distributed_tire() {
    nativecfg::TireConfig c{};
    c.backend = "distributed_2d_reference";
    c.surface_mode = "configured";
    c.front = {.material = std::nullopt, .tangent_k_n_m = 20000., .mu = 1.1, .relaxation_length_m = .2};
    c.rear = c.front;
    c.distinct_normal_deg = 20.;
    nativecfg::HingeDensity hd{.knots_m = {0., .004, .012}, .stiffness_n_m = {300000., 0., 0.},
                               .damping_ns_m = 2000., .provenance = "synthetic density; not fitted or measured"};
    nativecfg::DistributedMaterial mat{.density = hd, .fitting_dataset_id = "none-synthetic",
                                       .valid_load_range_n = {0., 2000.}, .valid_deflection_range_m = {0., .04},
                                       .calibration_status = "experimental_unvalidated"};
    c.distributed = nativecfg::DistributedConfig{.station_count = 128, .density = hd, .front = mat, .rear = mat};
    return c;
}
```
    The upload needs an `mjData` for `compiled_profile_vertices`. Load the probe model with `mj_loadXML` plus `bike::gpu::assets::load_hfield_bin` (the runner's own sequence, `cpu_batch_runner.cpp:183-201`), then `mj_makeData` and `mj_forward`.
  - The Metal "45x10 rays" case (:243-276) becomes a single 128-station launch.

  In `CMakeLists.txt`, add `bike_core` to `target_link_libraries(bike_test_gpu_batch_runners PRIVATE …)` (:745-746). The runner test includes no first-party `.cpp`, so the link edge the `:691-693` comment warns about does not apply.

- [ ] **Step 11.2: Run to verify it fails.** Run `cmake --build build/native-release --target bike_test_gpu_batch_step`. Expected: errors such as `no member named 'polyline' …` and `no matching function for call to 'gpu_step_world_writers'`, because `batch_step.hpp` is still old.

- [ ] **Step 11.3: Rewrite `batch_step.hpp`.**
  - Header comment: "…combining suspension, the shared distributed tyre core (gpu/tyre_step.hpp), drivetrain/freehub/resistance/braking, and the 9-DOF rider…".
  - Includes: replace `#include "tire_kernel.hpp"` with `#include "tyre_step.hpp"`.

  Then:
```cpp
struct GpuWorldConfig {
    GpuSuspensionParams suspension{};
    GpuTyreParams front_tyre{};
    GpuTyreParams rear_tyre{};
    GpuResistanceParams resistance{};
    GpuDrivetrainParams drivetrain{};
    GpuRiderParams rider{};
    float pedal_demand = 0.65f;
    float front_brake_demand = 0.0f;
    float rear_brake_demand = 0.0f;
};

// D13: a world whose tyre core reports a status stops applying writer
// forces from that step on; `status` keeps the first physics_core::Status.
struct GpuWorldState {
    GpuTyreWheelState tyre[2]{};
    GpuTyreWheelOutput tyre_out[2]{};
    float tyre_qfrc[kMaxNv] = {};
    GpuDrivetrainState drivetrain{};
    int status = 0;
    int failed_step = -1;
    int step_index = 0;
};
```
  In `gpu_build_sweep_configs`:
  - drop the two trailing parameters and their blocks (:94-104);
  - change the comment's last clause to "…and tyre density stiffness";
  - the stiffness loop becomes:
```cpp
        if (tire_stiff_scale != nullptr) {
            const float t_scale = gpu_max(0.0f, tire_stiff_scale[w]);
            for (int k = 0; k < kMaxHingeKnots; ++k) {
                cfg.front_tyre.stiffness_n_m[k] *= t_scale;
                cfg.rear_tyre.stiffness_n_m[k] *= t_scale;
            }
        }
```
  `gpu_init_world_state`: replace the bristle loop (:118-121) with:
```cpp
    for (int s = 0; s < 2; ++s) {
        for (int i = 0; i < kMaxGpuTyreStations; ++i) {
            st.tyre[s].stations[i] = GpuTyreStationState{};
        }
        st.tyre_out[s] = GpuTyreWheelOutput{};
    }
    for (int i = 0; i < kMaxNv; ++i) {
        st.tyre_qfrc[i] = 0.0f;
    }
    st.status = 0;
    st.failed_step = -1;
    st.step_index = 0;
```
  `gpu_step_world_writers`: use the signature from Interfaces. After `qfrc_w`/`ctrl_w` are computed (:155-156), add:
```cpp
    const BIKE_CORE_TABLE float *xmat_w = xmat_batch + w64 * idx.nbody * 9;
    const BIKE_CORE_TABLE float *xanchor_w = xanchor_batch + w64 * tyre_model.njnt * 3;
    const BIKE_CORE_TABLE float *xaxis_w = xaxis_batch + w64 * tyre_model.njnt * 3;
```
  Replace the block from `for (int i = 0; i < idx.nv; ++i) { qfrc_w[i] = 0.0f; }` through step 2 (:158-185) with:
```cpp
    for (int i = 0; i < idx.nv; ++i) {
        qfrc_w[i] = 0.0f;
    }
    for (int i = 0; i < idx.nu; ++i) {
        ctrl_w[i] = 0.0f;
    }
    const int step_index = st.step_index;
    st.step_index = step_index + 1;
    if (st.status != 0) {
        return;
    }

    // 1. Fork air spring + Charger3 HBO damper + rear coil shock + bumper + top-out stops
    gpu_write_suspension_qfrc(cfg.suspension, idx, qpos_w, qvel_w, qfrc_w);

    // 2. The shared distributed tyre core (physics_core/tyre.hpp), front then rear.
    const ::bike::physics_core::Status tyre_status = gpu_tyre_step(
        tyre_model, polyline, cfg.front_tyre, cfg.rear_tyre, xpos_w, xmat_w, xanchor_w, xaxis_w,
        qvel_w, idx.nv, dt_s, st.tyre, st.tyre_qfrc, st.tyre_out);
    if (tyre_status != ::bike::physics_core::Status::ok) {
        st.status = static_cast<int>(tyre_status);
        st.failed_step = step_index;
        for (int i = 0; i < idx.nv; ++i) {
            qfrc_w[i] = 0.0f;
        }
        return;
    }
    for (int i = 0; i < idx.nv; ++i) {
        qfrc_w[i] += st.tyre_qfrc[i];
    }
    const GpuTyreWheelOutput front_out = st.tyre_out[0];
    const GpuTyreWheelOutput rear_out = st.tyre_out[1];
```
  Step 3 (drivetrain) keeps reading `front_out.normal_load_n`, `rear_out.normal_load_n`, `front_out.effective_radius_m` and `rear_out.effective_radius_m` (:198-201). Those field names are unchanged.

  In `common.hpp`, delete `:61` (`kMaxTreadStations`). Replace the last two sentences of the `:319-322` comment with: "No production caller passes it to gpu_rear_pivot_anchor_world; the seatstay uses gpu_horst_pivot_anchor_world."

- [ ] **Step 11.4: Rewire the CPU runner and CUDA.**

  `cpu_batch_runner.cpp`:
  - Apply the C ABI rule to the three declarations (:83-142) and definitions (:157, :523, :566) and to the forwarding calls (:549, :592).
  - Document return code `-7` in the table at :144-155: "the tyre upload is missing or does not match the model: null `d_tyre_model`/`d_polyline`, `polyline_floats != polyline_floats(vertex_count)`, `njnt != m->njnt`, or a wheel body outside `[1, nbody)`".
  - Delete the heightfield block (:225-257, `model_hf` / `hf`). Keep `load_hfield_bin` (:183-192): MuJoCo's own contacts read `hfield_data`. After the dimension checks, insert:
```cpp
    if (d_tyre_model == nullptr || d_polyline == nullptr ||
        polyline_floats != bike::gpu::polyline_floats(d_tyre_model->polyline_vertex_count) ||
        d_tyre_model->njnt != static_cast<int>(m_base->njnt) ||
        d_tyre_model->wheels[0].body <= 0 || d_tyre_model->wheels[0].body >= m_base->nbody ||
        d_tyre_model->wheels[1].body <= 0 || d_tyre_model->wheels[1].body >= m_base->nbody) {
        std::cerr << "bike_cpu_run_batch: tyre upload does not match the model\n";
        mj_deleteModel(m_base);
        return -7;
    }
    const int njnt3 = 3 * static_cast<int>(m_base->njnt);
```
  - In `worker_fn` (:334), next to `d_xpos` add:
```cpp
        const std::span<mjtNum> d_xmat = boundary_span(d_t->xmat, 9 * to_index(nbody3 / 3));
        const std::span<mjtNum> d_xanchor = boundary_span(d_t->xanchor, to_index(njnt3));
        const std::span<mjtNum> d_xaxis = boundary_span(d_t->xaxis, to_index(njnt3));
        std::vector<float> xanchor_f32(to_index(njnt3)), xaxis_f32(to_index(njnt3));
```
  - In `step_world` add `alignas(64) std::array<float, 9 * to_index(kMaxNbody)> xmat_f32{};`. Fill all three by the same `static_cast<float>` loop as `xpos_f32` (:402-404).
  - Pass `*d_tyre_model, d_polyline` after `idx` and `xmat_f32.data(), xanchor_f32.data(), xaxis_f32.data()` after `xpos_f32.data()` in the `gpu_step_world_writers` call (:406-421). Delete its `hf` argument.
  - After the call, add `if (st_w.status != 0) all_finite_flag.store(0);`. A stopped world is a failed rollout for `out_all_finite`.

  `cuda_kernels.cu`:
  - `bike_gpu_writers_kernel` (:43-94): replace the five `hf` parameters with `const GpuTyreModel *d_tyre_model, const float *d_polyline` and add `const float *d_xmat, const float *d_xanchor, const float *d_xaxis` after `d_xpos`;
  - delete the `GpuHeightfield1D hf{…}` construction and pass `*d_tyre_model, d_polyline` and the three pointers in the call;
  - apply the C ABI rule to `bike_gpu_build_sweep_configs` (:107-116, :185-205) and `bike_gpu_launch_writers` (:255-340, including its host loop);
  - add a `d_tyre_model == nullptr || d_polyline == nullptr` early return next to the existing `d_configs == nullptr` check;
  - delete `hf_inv_dx`.

  The CUDA device branch (`#if defined(__CUDACC__)`) is not compiled by this host build. Read it once more after editing: every `hf` identifier must be gone (`grep -n "hf_\|GpuHeightfield1D" native/src/gpu/cuda_kernels.cu` prints nothing).

- [ ] **Step 11.5: Delete the rib kernel.** Run:
```bash
git rm native/src/gpu/tire_kernel.hpp native/tests/gpu/test_gpu_tire.cpp
```
  Then:
  - delete `CMakeLists.txt:696-697` (`bike_add_gpu_unit_test(bike_test_gpu_tire …)`) and `native/CMakeLists.txt:588`;
  - in `native/CMakeLists.txt:543-546`, drop `test_gpu_tire.cpp` from the comment's list;
  - in `metal_batch_runner.mm:103-110` (the list as extended by Task 2), replace `"tire_kernel.hpp"` with `"../physics_core/tyre_station.hpp", "../physics_core/terrain.hpp", "../physics_core/tyre_bristle.hpp", "../physics_core/kinematics.hpp", "../physics_core/tyre.hpp", "tyre_model.hpp", "tyre_step.hpp"`.

  Then confirm nothing outside `docs/` and `tools/colab_gpu/` still names the deleted symbols:
```bash
grep -rn -e tire_kernel -e GpuTireParams -e num_lateral_ribs -e kMaxTreadStations -e GpuWheelStationState -e GpuHeightfield1D -e gpu_sample_heightfield_1d native CMakeLists.txt
```
  Expected: hits only in `native/src/gpu/metal_batch_runner.mm`, which Task 12 rewrites. Any other hit gets fixed now.

- [ ] **Step 11.6: Run the host GPU tests.** On Linux, or with the Apple runner object temporarily excluded, run:
```bash
cmake --build build/native-release --target bike_test_gpu_batch_step bike_test_gpu_suspension bike_test_gpu_drivetrain_resistance bike_test_gpu_rider && \
  ctest --test-dir build/native-release -R '^bike_test_gpu_(batch_step|suspension|drivetrain_resistance|rider)$' --output-on-failure
```
  Expected: `100% tests passed … out of 4`. On macOS `bike_test_gpu_batch_runners` cannot build until Task 12. Continue directly with Task 12 and run G3 there.

---

## Task 12: Metal runner on the shared tyre; Metal == host float (D19)

**Files:**
- Modify: `native/src/gpu/metal_batch_runner.mm`:
  - MSL: :127-200, :230-305, :307-492, :493-565, :576-594, :625-745, :747-820;
  - context: :840-930;
  - C API: :1121-1280, :1280-1515 (delete), :1516-2014, :2020-2073.
- Create: `native/tests/gpu/tyre_probe_abi.hpp`, `native/tests/gpu/test_metal_tyre_bitwise.cpp`
- Modify: `native/tests/reference/GpuSharedCore.cmake`

**Interfaces (produced):**
- Remaining MSL entry points: `bike_gpu_init_states_metal_kernel`, `bike_gpu_writers_metal_kernel`, `bike_wp_init_states_kernel` and `bike_wp_writers_metal_kernel`.
- `MetalWritersContext` keeps `pso_init_states`, `pso_1thread` and `pso_step_writers_fused`.
- `BikeWpWritersArgs` (the field order is the Python ctypes mirror's, Task 13):
```cpp
struct BikeWpWritersArgs {
    device const bike::gpu::GpuModelIndices *idx;
    device const bike::gpu::GpuTyreModel *tyre_model;
    device const float *polyline;
    device const bike::gpu::GpuWorldConfig *configs;
    device bike::gpu::GpuWorldState *states;
    device const float *qpos;
    device const float *qvel;
    device const float *xpos;
    device const float *xmat;
    device const float *xanchor;
    device const float *xaxis;
    device float *qfrc_applied;
    device float *ctrl;
    device float *tendon_range;
    int nworld;
    float dt_s;
    int per_world_config;
    int tendon_range_world_stride;
};
```
- `bike_gpu_writers_metal_kernel` buffer indices:

  | Index | Argument |
  |---|---|
  | 0 | world_offset |
  | 1 | world_count |
  | 2 | dt |
  | 3 | idx |
  | 4 | tyre_model |
  | 5 | polyline |
  | 6 | xmat |
  | 7 | xanchor |
  | 8 | configs |
  | 9 | per_world_config |
  | 10 | states |
  | 11 | qpos |
  | 12 | qvel |
  | 13 | xpos |
  | 14 | qfrc |
  | 15 | ctrl |
  | 16 | tendon |
  | 17 | stride |
  | 18 | xaxis |

  Indices 8–17 are unchanged; 4–7 reuse the four heightfield slots.

- [ ] **Step 12.1: Write the failing Metal-vs-host test.**

  Create `native/tests/gpu/tyre_probe_abi.hpp`. It is compiled as host C++ by the test and stitched as MSL text by the same test:
```cpp
// tyre_probe_abi.hpp — test-only layout for bike_tyre_probe: one
// gpu_tyre_step call per thread over a shared GpuTyreModel + polyline.
#pragma once
#include "../../src/gpu/tyre_step.hpp"

namespace bike::gpu::probe {
BIKE_CONSTEXPR int kMaxProbeJoints = 64;
struct TyreProbeHeader {
    int case_count;
    int nv;
    int model_offset;
    int polyline_offset;
    int case_offset;
    int case_stride;
    int result_stride;
};
struct TyreProbeCase {
    GpuTyreParams params[2];
    float xpos[3 * kMaxNbody];
    float xmat[9 * kMaxNbody];
    float xanchor[3 * kMaxProbeJoints];
    float xaxis[3 * kMaxProbeJoints];
    float qvel[kMaxNv];
    float dt;
    GpuTyreWheelState state[2];
};
struct TyreProbeResult {
    GpuTyreWheelState state[2];
    float qfrc[kMaxNv];
    GpuTyreWheelOutput out[2];
    int status;
    int sizes[3];   // sizeof(TyreProbeHeader/Case/Result) as MSL sees them
};
} // namespace bike::gpu::probe
```
  Create `native/tests/gpu/test_metal_tyre_bitwise.cpp`:
```cpp
// test_metal_tyre_bitwise.cpp — D19: the production MSL of the shared tyre
// core (gpu_tyre_step, <float, GpuToolkit>) is bitwise equal to the same
// header compiled as host float, on stand40 kinematics with an evolving
// bristle state (40 rows → 40 independent Metal threads).
#include "../reference/tyre_reference_support.hpp"
#include "tyre_probe_abi.hpp"
#include "gpu/tyre_upload.hpp"

#include <bit>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

extern "C" const char *bike_metal_get_shader_source(const char *gpu_src_dir);
extern "C" int bike_metal_run_probe(const char *source, const char *kernel_name, int variant,
                                    const void *in, std::size_t in_bytes, void *out,
                                    std::size_t out_bytes, int thread_count, char *err,
                                    std::size_t err_capacity);

namespace {
using namespace tyre_reference;
namespace gp = bike::gpu::probe;

constexpr const char *kProbeKernel = R"MSL(
kernel void bike_tyre_probe(device const uchar *in [[buffer(0)]], device uchar *out [[buffer(1)]],
                            uint gid [[thread_position_in_grid]]) {
    using namespace bike::gpu;
    using namespace bike::gpu::probe;
    device const TyreProbeHeader &h = *(device const TyreProbeHeader *)in;
    if (static_cast<int>(gid) >= h.case_count) return;
    device const GpuTyreModel &model = *(device const GpuTyreModel *)(in + h.model_offset);
    device const float *poly = (device const float *)(in + h.polyline_offset);
    device const TyreProbeCase &c = *(device const TyreProbeCase *)(in + h.case_offset + gid * h.case_stride);
    device TyreProbeResult &r = *(device TyreProbeResult *)(out + gid * h.result_stride);
    const GpuTyreParams pf = c.params[0];
    const GpuTyreParams pr = c.params[1];
    float xpos[3 * kMaxNbody];
    for (int i = 0; i < 3 * kMaxNbody; ++i) xpos[i] = c.xpos[i];
    float qvel[kMaxNv];
    for (int i = 0; i < kMaxNv; ++i) qvel[i] = c.qvel[i];
    GpuTyreWheelState st[2];
    st[0] = c.state[0];
    st[1] = c.state[1];
    float qfrc[kMaxNv];
    for (int i = 0; i < kMaxNv; ++i) qfrc[i] = 0.0f;
    GpuTyreWheelOutput o[2];
    const int status = static_cast<int>(gpu_tyre_step(model, poly, pf, pr, xpos, c.xmat, c.xanchor,
                                                      c.xaxis, qvel, h.nv, c.dt, st, qfrc, o));
    r.state[0] = st[0];
    r.state[1] = st[1];
    for (int i = 0; i < kMaxNv; ++i) r.qfrc[i] = qfrc[i];
    r.out[0] = o[0];
    r.out[1] = o[1];
    r.status = status;
    r.sizes[0] = static_cast<int>(sizeof(TyreProbeHeader));
    r.sizes[1] = static_cast<int>(sizeof(TyreProbeCase));
    r.sizes[2] = static_cast<int>(sizeof(TyreProbeResult));
}
)MSL";

std::string abi_as_msl() {
    std::ifstream f(repo_file("native/tests/gpu/tyre_probe_abi.hpp"));
    bt::require(f.good(), "tyre_probe_abi.hpp readable");
    std::ostringstream o;
    for (std::string line; std::getline(f, line);)
        if (line.rfind("#include", 0) != 0 && line.rfind("#pragma once", 0) != 0) o << line << '\n';
    return o.str();
}

std::size_t align16(std::size_t v) { return (v + 15) & ~std::size_t{15}; }

[[gnu::noinline]] void test_metal_equals_host_float() {
    const auto bundle = [](std::string_view n) { return platform_fixture("native_distributed_tire", n); };
    const auto ep = load_episode_bundle(bundle("episodes/stand40"));
    const nativecfg::WriterConfig wcfg = load_config(bundle("env"));
    Stepper st((bundle("episodes/stand40") / "model.mjb").string(), wcfg);
    const mjModel *m = st.model();
    const mjData *d = st.data();
    bt::require(m->njnt <= gp::kMaxProbeJoints && m->nbody <= bike::gpu::kMaxNbody && m->nv <= bike::gpu::kMaxNv,
                "stand40 fits the probe capacities");
    const bike::gpu::TyreUpload up = bike::gpu::build_tyre_upload(m, d, *wcfg.tire);
    const int rows = static_cast<int>(ep.artifact.state_qpos.shape[0]);
    std::vector<gp::TyreProbeCase> cases(static_cast<std::size_t>(rows));
    std::vector<gp::TyreProbeResult> host(static_cast<std::size_t>(rows));
    std::array<bike::gpu::GpuTyreWheelState, 2> state{};
    for (int k = 0; k < rows; ++k) {
        const std::size_t uk = static_cast<std::size_t>(k);
        st.set_state(state_row(ep.artifact.state_qpos, uk, ep.nq), state_row(ep.artifact.state_qvel, uk, ep.nv),
                     state_row(ep.artifact.state_act, uk, ep.na),
                     state_row(ep.artifact.state_warmstart, uk, ep.nv), ep.time[uk]);
        gp::TyreProbeCase &c = cases[uk];
        c = gp::TyreProbeCase{};
        c.params[0] = bike::gpu::make_gpu_tyre_params(*wcfg.tire, 0);
        c.params[1] = bike::gpu::make_gpu_tyre_params(*wcfg.tire, 1);
        for (int i = 0; i < 3 * m->nbody; ++i) c.xpos[i] = static_cast<float>(d->xpos[i]);
        for (int i = 0; i < 9 * m->nbody; ++i) c.xmat[i] = static_cast<float>(d->xmat[i]);
        for (int i = 0; i < 3 * m->njnt; ++i) {
            c.xanchor[i] = static_cast<float>(d->xanchor[i]);
            c.xaxis[i] = static_cast<float>(d->xaxis[i]);
        }
        for (int i = 0; i < m->nv; ++i) c.qvel[i] = static_cast<float>(d->qvel[i]);
        c.dt = 0.00125f;
        c.state[0] = state[0];
        c.state[1] = state[1];
        gp::TyreProbeResult &r = host[uk];
        r = gp::TyreProbeResult{};
        std::array<bike::gpu::GpuTyreWheelState, 2> s = state;
        std::array<bike::gpu::GpuTyreWheelOutput, 2> o{};
        std::array<float, bike::gpu::kMaxNv> qfrc{};
        r.status = static_cast<int>(bike::gpu::gpu_tyre_step(up.model, up.polyline.data(), c.params[0], c.params[1],
                                                             c.xpos, c.xmat, c.xanchor, c.xaxis, c.qvel, m->nv, c.dt,
                                                             s.data(), qfrc.data(), o.data()));
        r.state[0] = s[0];
        r.state[1] = s[1];
        std::memcpy(r.qfrc, qfrc.data(), sizeof r.qfrc);
        r.out[0] = o[0];
        r.out[1] = o[1];
        state = s;   // the next row starts from this row's state
    }
    // Pack: header | model | polyline | cases.
    gp::TyreProbeHeader h{};
    h.case_count = rows;
    h.nv = m->nv;
    h.model_offset = static_cast<int>(align16(sizeof h));
    h.polyline_offset = static_cast<int>(align16(static_cast<std::size_t>(h.model_offset) + sizeof up.model));
    h.case_offset = static_cast<int>(align16(static_cast<std::size_t>(h.polyline_offset) + up.polyline.size() * sizeof(float)));
    h.case_stride = static_cast<int>(align16(sizeof(gp::TyreProbeCase)));
    h.result_stride = static_cast<int>(align16(sizeof(gp::TyreProbeResult)));
    std::vector<unsigned char> in(static_cast<std::size_t>(h.case_offset) + cases.size() * static_cast<std::size_t>(h.case_stride));
    std::memcpy(in.data(), &h, sizeof h);
    std::memcpy(in.data() + h.model_offset, &up.model, sizeof up.model);
    std::memcpy(in.data() + h.polyline_offset, up.polyline.data(), up.polyline.size() * sizeof(float));
    for (std::size_t k = 0; k < cases.size(); ++k)
        std::memcpy(in.data() + h.case_offset + k * static_cast<std::size_t>(h.case_stride), &cases[k], sizeof cases[k]);
    std::vector<unsigned char> out(cases.size() * static_cast<std::size_t>(h.result_stride), 0);
    const std::string source = std::string(bike_metal_get_shader_source(nullptr)) + abi_as_msl() + kProbeKernel;
    std::array<char, 8192> err{};
    const int rc = bike_metal_run_probe(source.c_str(), "bike_tyre_probe", -1, in.data(), in.size(), out.data(),
                                        out.size(), rows, err.data(), err.size());
    bt::require(rc == 0, std::string("Metal probe ran: ") + err.data());
    int mismatches = 0;
    for (std::size_t k = 0; k < cases.size(); ++k) {
        gp::TyreProbeResult dev{};
        std::memcpy(&dev, out.data() + k * static_cast<std::size_t>(h.result_stride), sizeof dev);
        if (k == 0)
            bt::require(dev.sizes[0] == static_cast<int>(sizeof(gp::TyreProbeHeader)) &&
                            dev.sizes[1] == static_cast<int>(sizeof(gp::TyreProbeCase)) &&
                            dev.sizes[2] == static_cast<int>(sizeof(gp::TyreProbeResult)),
                        "MSL and host agree on the probe layout");
        const gp::TyreProbeResult &want = host[k];
        if (dev.status != want.status) {
            std::cerr << "row " << k << " status metal " << dev.status << " host " << want.status << '\n';
            ++mismatches;
        }
        const auto cmp = [&](const char *what, const float *a, const float *b, std::size_t count) {
            for (std::size_t i = 0; i < count; ++i)
                if (std::bit_cast<std::uint32_t>(a[i]) != std::bit_cast<std::uint32_t>(b[i])) {
                    std::cerr << "row " << k << ' ' << what << '[' << i << "] metal " << a[i] << " host " << b[i] << '\n';
                    ++mismatches;
                    return;
                }
        };
        cmp("qfrc", dev.qfrc, want.qfrc, static_cast<std::size_t>(m->nv));
        for (int s = 0; s < 2; ++s) {
            cmp("normal_load", &dev.out[s].normal_load_n, &want.out[s].normal_load_n, 1);
            cmp("effective_radius", &dev.out[s].effective_radius_m, &want.out[s].effective_radius_m, 1);
            for (int i = 0; i < up.model.wheels[s].station_count; ++i) {
                cmp("xi", &dev.state[s].stations[i].xi, &want.state[s].stations[i].xi, 1);
                cmp("tangent", dev.state[s].stations[i].tangent, want.state[s].stations[i].tangent, 3);
                if (dev.state[s].stations[i].flags != want.state[s].stations[i].flags) ++mismatches;
            }
        }
    }
    std::cout << "metal vs host-float mismatches: " << mismatches << " over " << rows << " rows\n";
    bt::require(mismatches == 0, "Metal tyre step is bitwise host float (D19, S6)");
}
} // namespace

int main() {
    return bt::run([] { test_metal_equals_host_float(); });
}
```
  Append to `GpuSharedCore.cmake`:
```cmake
if(APPLE)
  # D19: Metal (production MSL, measured safe math) == host float for the
  # shared tyre core. Links the Metal runner object for bike_metal_run_probe.
  bike_add_reference_test(metal_tyre_bitwise ENGINE
    SOURCES ../gpu/test_metal_tyre_bitwise.cpp $<TARGET_OBJECTS:bike_gpu_metal_runner_obj>
    LIBRARIES bike_app "-framework Metal" "-framework Foundation"
    LABELS tyre gpu)
endif()
```
  Before relying on it, check that `bike_gpu_metal_runner_obj` exists whenever this file is included. It is created under `if(BUILD_TESTING)` at `CMakeLists.txt:720-736`, and `native/tests/reference/CMakeLists.txt:3-5` returns early without `BUILD_TESTING`, so the two guards agree.

  In D19 fallback mode (Task 1), replace the bit comparison in `cmp` with `ulps(a[i], b[i]) <= 1`, copying the helper from `test_metal_math_mode.cpp`, and say so in the test comment.

- [ ] **Step 12.2: Run to verify it fails.** Run `cmake --build build/native-release --target reference.metal_tyre_bitwise`. Expected: the Metal runner object fails to compile against the Task 11 `batch_step.hpp` (`no matching function for call to 'gpu_step_world_writers'` in `metal_batch_runner.mm`).

- [ ] **Step 12.3: Rewrite the Metal runner.**

  1. **MSL, delete:**
     - `bike_metal_wheel_contact_simd32` (the raw string after the header loop, from :127 to the end of that function);
     - `bike_metal_step_writers_simd32_world`;
     - `bike_metal_step_writers_simd32_impl`;
     - `bike_metal_step_writers_fused_simd32_impl`;
     - the kernels `bike_gpu_writers_simd32_kernel`, `bike_wp_writers_simd32_kernel`, `bike_metal_step_writers_kernel`, `bike_metal_step_writers_simd32_kernel`, `bike_metal_step_writers_fused_kernel` and `bike_metal_step_writers_fused_simd32_kernel` (:701-820 region).

     Also drop `#include <metal_simdgroup>` (:115).
  2. **`bike_metal_step_writers_scalar_world`** (:233-305):
     - Replace the `hf` parameter with `device const bike::gpu::GpuTyreModel &tyre_model, device const float *polyline`.
     - Add `device const float *xmat_batch, device const float *xanchor_batch, device const float *xaxis_batch` after `xpos_batch`.
     - In the call to `gpu_step_world_writers` (which runs with `w = 0` on thread copies), pass:
```cpp
        tyre_model, polyline, &cfg, 0, &st, qpos_w, qvel_w, xpos_w,
        xmat_batch + w * idx.nbody * 9, xanchor_batch + w * tyre_model.njnt * 3,
        xaxis_batch + w * tyre_model.njnt * 3, qfrc_w, ctrl_w,
```
       These are pre-offset device pointers, because `BIKE_CORE_TABLE` is `device` and `gpu_step_world_writers` offsets them by `w = 0`.
  3. **`bike_metal_step_writers_impl` and `bike_gpu_writers_metal_kernel`:** use the buffer table in Interfaces.
     - Delete the `GpuHeightfield1D hf{…}` and `inv_dx` lines.
     - Pass `*tyre_model_ptr` (from `device const bike::gpu::GpuTyreModel *tyre_model [[buffer(4)]]`) and `polyline [[buffer(5)]]`.
  4. **`BikeWpWritersArgs`:** as in Interfaces. In `bike_metal_step_writers_fused_impl`, delete the `hf` lines and pass `*args->tyre_model, args->polyline, …, args->xmat, args->xanchor, args->xaxis`.
  5. **`MetalWritersContext`** (:840-930):
     - delete `pso_simd32`, `pso_step_writers_simd32` and `pso_step_writers_fused_simd32`, their `newFunctionWithName` lookups and the `threadExecutionWidth != 32` check;
     - keep `fn_init`, `fn1` and `fn_wp1`;
     - wherever the C API tests `!ctx.pso_simd32`, drop that clause.
  6. **`bike_metal_launch_writers`** (:1121-1280):
     - apply the C ABI rule;
     - delete `use_simd32` (:1166) and every branch on it, keeping the scalar branch;
     - replace the `hf_bytes` buffer with three buffers: `[device newBufferWithBytes:d_tyre_model length:sizeof(bike::gpu::GpuTyreModel) …]`, the polyline (`polyline_floats * sizeof(float)`) and `xmat`/`xanchor`/`xaxis` (`nworld*nbody*9`, `nworld*njnt*3` floats, `njnt = d_tyre_model->njnt`);
     - bind them at 4, 5, 6, 7 and 18;
     - add `d_tyre_model`, `d_polyline`, `d_xmat`, `d_xanchor` and `d_xaxis` to the null-argument check returning `-1`.
  7. **Delete `bike_metal_benchmark_writers_only`** (:1280-1515), as decided in P6.
  8. **`bike_metal_run_hybrid_sweep`** (:1516-2014) and **`bike_metal_run_hybrid_batch`** (:2047-2073):
     - apply the C ABI rule;
     - delete the heightfield extraction and `use_simd32` (:1609);
     - validate the tyre upload against `m_base` exactly as `cpu_batch_runner.cpp` does in Step 11.4, with the same `-7` code;
     - each step, copy `d_t->xmat`, `d_t->xanchor` and `d_t->xaxis` to the shared float buffers next to the existing `xpos` copy.
  9. **`bike_metal_launch_step_writers`** (:2020-2045): apply the rule and forward.

  Use `grep -n "simd32\|hf_\|GpuHeightfield1D\|kMaxTreadStations\|num_lateral_ribs\|benchmark_writers_only\|kTwoPi" native/src/gpu/metal_batch_runner.mm` to drive the edit. Expected after the edit: no output.

  Update the `native/tests/gpu/test_gpu_batch_runners.cpp` Metal calls with the C ABI rule (Step 11.1 already built the upload). Remove its `bike_metal_benchmark_writers_only` declaration and call, if any.

- [ ] **Step 12.4: Run the Metal tests and G3.**
```bash
cmake --build build/native-release -j && \
  ctest --test-dir build/native-release -R '^(reference\.metal_tyre_bitwise|bike_test_gpu_batch_runners|bike_test_metal_math_mode)$' -V
ctest --test-dir build/native-release -L gpu --output-on-failure
```
  Expected: `metal vs host-float mismatches: 0 over 40 rows`, then all `-L gpu` nodes pass.
  - Any mismatch → **S6**: report the first printed row/field and the Task 1 table row for the ops involved.
  - A compile error inside the MSL (the probe's `err` text) on a `thread` pointer member of `WheelScratch`/`WheelGeometry`, or on `GpuTyreIo`, is the unverified MSL fact listed below. Fix it by passing the scratch pointers as separate parameters in `tyre.hpp` (signature change only, same statements), then rerun G1 to prove the CPU bits did not move.

- [ ] **Step 12.5: Run G1 and G4, then commit Tasks 11 and 12 together.**
```bash
git add -A native/src/gpu native/tests/gpu native/tests/reference CMakeLists.txt native/CMakeLists.txt
git commit -m "feat(gpu): run every GPU runner on the shared tyre core

D12: tire_kernel.hpp, the lateral ribs and the simd32 kernels are gone;
CPU/CUDA/Metal runners take the GpuTyreModel + polyline upload and engine
xmat/xanchor/xaxis. Metal equals host float bitwise on stand40 (D19).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Task 13: Python mirrors in `tools/colab_gpu/` (T63 exemption)

**Files:**
- Modify:
  - `tools/colab_gpu/build_cuda_writers.py`: :89-150 argtypes, :425-455 `build_sweep_configs`;
  - `tools/colab_gpu/cpu_batch_sweep.py`: :146-170, :170-380;
  - `tools/colab_gpu/metal_hybrid_sweep.py`: :141-215, :225-380;
  - `tools/colab_gpu/mjwarp_metal_sweep.py`: :90-125, :150-240, :300-750;
  - `tools/colab_gpu/mjwarp_batch_sweep.py`: :140-150, :300-600.

Plan P8: Stage 1 ships no upload exporter, so every sweep entry point fails closed. The ctypes mirrors still follow the new ABI exactly, so Stage 2 only has to supply the upload.

- [ ] **Step 13.1: Write the failing check.** Run:
```bash
grep -rn -e num_lateral_ribs -e num_stations -e simd32 -e hf_heights -e hf_num_samples -e rays_per_world -e benchmark_writers_only tools/colab_gpu/*.py
```
  Expected now: hits in all five files, for example `cpu_batch_sweep.py:175`, `metal_hybrid_sweep.py:230`, `mjwarp_metal_sweep.py:113` and `:159`, `build_cuda_writers.py:431`. After this task the same command must print nothing.

- [ ] **Step 13.2: Edit the mirrors.**
  - **`build_cuda_writers.py`:**
    - `bike_gpu_build_sweep_configs.argtypes`: drop the last two `ctypes.c_int`.
    - `bike_gpu_launch_writers.argtypes` and `bike_gpu_launch_step_writers.argtypes` (if present): replace `(c_void_p, c_int, c_float, c_float)` for the heightfield with `(c_void_p, c_void_p, c_int)`, and insert three `c_void_p` after the `xpos` pointer.
    - The `build_sweep_configs(...)` helper drops `num_stations`/`num_lateral_ribs`.
  - **`cpu_batch_sweep.py` and `metal_hybrid_sweep.py`:**
    - Apply the same `argtypes` rule to `bike_cpu_run_batch_sweep` / `bike_metal_run_hybrid_sweep` / `bike_metal_launch_writers`.
    - Delete the `bike_metal_benchmark_writers_only` argtypes and every call to it.
    - Delete the `num_stations`/`num_lateral_ribs` parameters, the tier keys and `rays_per_world`.
    - At the top of each `main`/sweep entry, add:
```python
def _require_gpu_tyre_upload() -> None:
    """The shared tyre core needs the CPU-built GpuTyreModel + polyline upload.

    Stage 1 ships only the C++ builder (native/src/gpu/tyre_upload.cpp); the
    Stage 2 bundle exporter writes it for these sweeps (plan P8).
    """
    raise SystemExit(
        "GPU tyre upload unavailable: the Stage 2 bundle exporter provides it "
        "(docs/superpowers/plans/2026-11-03-gpu-shared-s1-tyre.md, P8)"
    )
```
  - **`mjwarp_metal_sweep.py`:**
    - Make `_BikeWpWritersArgs._fields_` follow Task 12's `BikeWpWritersArgs` order exactly: `idx, tyre_model, polyline, configs, states, qpos, qvel, xpos, xmat, xanchor, xaxis, qfrc_applied, ctrl, tendon_range` (all `c_uint64`), then `nworld c_int, dt_s c_float, per_world_config c_int, tendon_range_world_stride c_int`.
    - Delete `k_writers_32` (:159) and all `use_simd32` parameters and branches (:218-222, :361, :398, :437, :479, :579, :626, :650).
    - Pass `d_b.xmat.ptr`, `d_b.xanchor.ptr` and `d_b.xaxis.ptr`. These mujoco_warp `Data` fields were verified in Part 1.
    - Call `_require_gpu_tyre_upload()` at the sweep entry.
  - **`mjwarp_batch_sweep.py`:**
    - Apply the CUDA launch rule.
    - Delete `_extract_heightfield_1d`'s use for the writers. Keep it if it is still used for something else; check with `grep -n _extract_heightfield_1d tools/colab_gpu/*.py`.
    - Call `_require_gpu_tyre_upload()`.

- [ ] **Step 13.3: Verify.** Run:
```bash
grep -rn -e num_lateral_ribs -e num_stations -e simd32 -e hf_heights -e hf_num_samples -e rays_per_world -e benchmark_writers_only tools/colab_gpu/*.py
uv run --with numpy python -m py_compile tools/colab_gpu/*.py
uv run --with numpy --with pytest pytest -q tools/colab_gpu/test_model_assets.py
```
  Expected: the `grep` prints nothing, `py_compile` exits 0, and pytest reports `passed`.

- [ ] **Step 13.4: Commit.** Message: `feat(colab_gpu): mirror the shared-tyre runner ABI`. Body: "ctypes argtypes and BikeWpWritersArgs follow the GpuTyreModel/polyline/xmat ABI; sweeps fail closed until the Stage 2 upload exporter (P8)." Then the trailer.

---

## Task 14: Documentation and the stage gate

**Files:**
- Modify:
  - `docs/TESTING.md:1062`: the GPU kernel suppression row;
  - `docs/TESTING.md`: wherever `native_station_tests` / `bike_test_gpu_tire` are listed (`grep -n`);
  - `tools/colab_gpu/README.md`: :26-60 and :95-110.

- [ ] **Step 14.1: Update the suppression register row** (`docs/TESTING.md:1062`). The paths become:
  - `native/src/gpu/{common,suspension_kernel,drivetrain_resistance_kernel,rider_kernel,tyre_model,tyre_step,batch_step}.hpp`;
  - `native/src/physics_core/{platform,status,capacity,scalar,toolkit,gpu_toolkit,tyre_station,terrain,tyre_bristle,kinematics,tyre}.hpp`;
  - `cuda_kernels.cu`.

  The verification column adds `physics_core_tests`, `reference.gpu_tyre_core`, `bike_test_metal_math_mode` and `reference.metal_tyre_bitwise`. `cpu_exact_toolkit.hpp` and `host_status.hpp` are host-only and carry no suppression. Task 2 already added the `physics_core/` paths, so this step only removes `tire_kernel` and adds the two `gpu/tyre_*` headers.

- [ ] **Step 14.2: Update the other docs.**
  - Where `docs/TESTING.md` lists GPU nodes, replace `bike_test_gpu_tire` with the four new nodes.
  - In `tools/colab_gpu/README.md`, replace the `tire_kernel.hpp` row (:28) with:
```
| `tyre_model.hpp` / `tyre_step.hpp` | The per-model GPU tyre upload (128-station ring, dof chain, surface map) and the `<float, GpuToolkit>` instantiation of the shared distributed tyre core in `native/src/physics_core/`. |
```
  - Delete the rib-tier table (:101-110). Add one sentence: "Sweeps fail closed until the Stage 2 bundle exporter provides the tyre upload."
  - Fix the `simd32` kernel names (:157).

- [ ] **Step 14.3: Run the stage gate once.** Run `bash tools/run_tests.sh native`. Expected: exit 0.
  - Compare any failure with the documented baseline list in `docs/TESTING.md`.
  - A failure that the baseline does not explain and that lies in a touched area (tyre, GPU, headers, sweeps) is fixed in place, then the gate is rerun.
  - A digest or bitwise failure here → **S1**.

- [ ] **Step 14.4: Commit.** Message: `docs(gpu): document the shared tyre core and its gates`, with the trailer.

---

## Self-review record

- **Brief coverage:**
  - Metal math mode (Task 1);
  - `physics_core/` scaffolding with toolkits, status enum, capacities (stations ≥ 128) and the stitcher (Task 2);
  - one CPU law per task with a bitwise test (Tasks 3–7);
  - exact Jacobian (Task 8);
  - polyline upload and the 128-station ring (Task 9);
  - GPU tyre step (Task 10);
  - runners switched, rib kernel and ribs deleted, params from the CPU config (Tasks 11–12);
  - Python mirrors (Task 13);
  - Metal == host float (Task 12) and host float vs double (Task 10);
  - final gate and docs (Task 14).
- **Exception order:** each `return st` in the cores is at the position of the CPU `throw` it replaces (Tasks 3–7). The `Io` callbacks keep the CPU-only throws (`make_patch`, patch capacity) after the projection, as today.
- **Address spaces:**
  - `BIKE_CORE_TABLE` (device on Metal): polyline, ring, weights, chain tables, per-world `xmat`/`xanchor`/`xaxis`.
  - `BIKE_THREAD`: density knots (inside the thread-copied config), scratch, state, `qvel`, `xpos`.
  - In `metal_batch_runner.mm` the scalar world copies `cfg`, `st`, `qpos`, `qvel` and `xpos` to thread memory (`:258-273`), which is where those thread pointers come from.
- **Names used consistently across tasks:**
  - `Status`, `Lit`, `GpuToolkit`, `CpuExactToolkit`;
  - `PolylineView`, `HingeDensityView`, `WheelGeometry`, `WheelScratch`, `WheelLaw`, `WheelJacobian`, `StationState`, `StationResult`;
  - `GpuTyreParams`, `GpuTyreModel`, `GpuTyreWheelState`, `GpuTyreWheelOutput`;
  - `polyline_floats`, `build_tyre_upload`, `make_gpu_tyre_params`, `gpu_tyre_step`, `throw_core_status`.
