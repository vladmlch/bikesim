# Subproject 1 Design Specification: Shared GPU/CPU Tyre & Terrain Force Core (`native/src/physics_core/`)

**Date:** 2026-11-17  
**Target Branch / Baseline:** `feat/gpu-shared-tyre-core` (`HEAD 0502660d`)  
**Supersedes / Updates:** Stale assumptions in `docs/superpowers/plans/2026-11-03-gpu-shared-s1-tyre.md` (written against pre-`5f23a658` / pre-`motor_clutch` commit `b25120af`), `docs/agents/simulation-decisions.md` Item 29 & Rejected Item 13, and Master Plan `D3`/`D21` (`2026-11-03-gpu-shared-physics-master.md`).

---

## 1. Goal, Scope, & 4-Subproject Program Context

### 1.1 Subproject 1 Goal & Deliverables

Subproject 1 extracts a single-source, allocation-free, header-only C++ template force core under `native/src/physics_core/` that runs bitwise-identically on **CPU FP64** and deterministically on **Apple GPU FP32** (Metal MSL in `native/src/gpu/metal_batch_runner.mm` + `warp-metal`) and **NVIDIA GPU FP32** (`native/src/gpu/cuda_kernels.cu` + `warp` CUDA) for **both** production tyre backends on the live `0502660d` clutch plant (`nq = 27, nv = 27, nu = 12, nbody = 29, ntendon = 2, dt = 1.25 ms`):

1. **Slice 1 — `compliant_2d` (`TireWriter`, `native/src/writers/tire.{hpp,cpp}`):** Primary `welded` profile tyre (`examples/research/viewer_physics_welded.toml`), including `O(1)`-register `ProfileQuery::contact` terrain scanner (`native/src/tyre/profile.cpp`), Ban #39 rate-independent `tanh` radial hysteresis (`native/src/contact/laws.hpp:48-62`), 3-gate branch continuity (`tire.cpp:729-760`), airborne persistent `segment` tracking (`:843`), `jacp`-only kinematics (`jr == nullptr`), and `BrushState` persistence (`~112 B/world`).
2. **Slice 2 — `distributed_2d_reference` (`DistributedTireWriter`, `native/src/writers/distributed_tire.{hpp,cpp}`):** `distributed` profile tyre (`examples/research/viewer_physics_distributed.toml`), including 128-station `xmat`-rotated wheel ring, `SignedProfile::height`/`query` (`native/src/tyre/distance.cpp`), `station_response` `tanh` radial hysteresis (`native/src/tyre/station.cpp:77-101`), and per-station `StationState` (`~5,120 B/world`).
3. **Engine-Free Schema v1 `bike-recording-digest` Gate & Policy Update:** Updates `docs/agents/simulation-decisions.md` (Item 29 + Rejected Item 13), adds standalone engine-free `bike-recording-digest` over untouched `schema_version = 1` `physics` + `forces` DuckDB tables, freezes 30 s `darwin-arm64` golden digests (recorded via headless `bike-ride`) for both `viewer_physics_welded.toml` and `viewer_physics_distributed.toml` on `flat` and `rough_uphill_extreme`, and removes legacy `native/src/gpu/tire_kernel.hpp`, `num_lateral_ribs`, and Metal `simd32` tyre kernels.

### 1.2 Explicit Interfaces with Subprojects 2–4

| Subproject | Shared Contracts & Interfaces Owned Across Boundaries |
|---|---|
| **SP1 (This Spec)** | `native/src/physics_core/{common,exact_math,gpu_math,cpu_exact_toolkit,gpu_toolkit,tyre_types,kinematics,terrain_scanner,terrain_distance,tyre_contact,tyre_compliant,tyre_distributed}.hpp` + `tyre_upload.{hpp,cpp}`, `O(1)` `ProfileQuery::contact`, `GpuTyreModel` + `GpuWheelChain` (`nq = 27`), `clean_split` `mujoco_warp` driver (`fwd_velocity` before `make_constraint`, ending in `implicit`), Metal + CUDA dual-backend wiring, `bike-recording-digest` & `golden_episode_digest`. |
| **SP2 (Remaining Force Laws & Golden Bundle)** | Consumes `physics_core/{common,exact_math,gpu_math,cpu_exact_toolkit,gpu_toolkit,kinematics}.hpp` to extract `suspension` (`shock_upper_stop`, `shock_hbo`, `lockout_firm`, dual `v_knee`), `resistance` (`chain_angular_jacobian` `jr` + frame `jp` at `point_body_m`), `StaticBrake::hold_qfrc`, `rider_joint_envelope` (`soft_edge_scalar`, zero damping), and dual ratchets (`freehub` + `crank_clutch`). Extends `GoldenForceObservation` to `StepClosureSnapshot` (`dof_frictionloss`, `wrap_prm`, `tendon_range`, `qacc_warmstart`, `mjINT_IMPLICITFAST`), replaces Master Plan D21 `release-f32` with standalone `bundle_runner_f32` against `.worktrees/fp32-physics/third-party/mujoco-312-install-f32`, and adds `bike-replay` ghost. |
| **SP3 (Autonomous Controllers & Monitors)** | Consumes SP1's `nworld`-expanded `(m.dof_frictionloss, m.tendon_range, m.tendon_invweight0)` buffers and `GpuWheelState`. Owns `12,308 B/world` controller/monitor POD state, 12-tendon static cassette (`ntendon = 13` with discrete 1-step cog switch and zero `mujoco_warp` fork, per user decision 2026-11-17), synchronized `2π` epoch re-basing across `{qpos, freehub_boundary, clutch_boundary, brake_latch, target_phase_rad}`, `int32` tick timers replacing `1e-12` countdowns, and single-pass `efc.type == mjEQ_CONNECT` readout. |
| **SP4 (GPU `SpindleController`)** | Consumes SP1's `clean_split` `mjwarp` schedule (`4× solve` per step, `wp.ScopedCapture()`). Implements GPU `M2_K2` (`1` Newton step, `alpha = 1`, `kEps = 0.1 N`, `|det| > 1e-6`, fixed step-entry `qacc_warmstart`), analytical 2-link `J`/`dJ/dphi`, CSR `d.M` (`smooth.mul_m`), dense `d.efc.J`, and `body_subtreemass`/`subtree_com` `leg_weights_at_pedal`. |

---

## 2. Policy Update (`simulation-decisions.md`) & Schema v1 CPU Bit-Identity Gate

### 2.1 Amendment to `docs/agents/simulation-decisions.md`

Update **Item 29 («Точность», `:268-274`)** and **Rejected Item 13 («Физика на видеокарте Apple», `:824-825`)**:

- **Authoritative Oracle Unchanged:** CPU `double` (`FP64`) remains the sole physics-acceptance and model-calibration oracle. Zero bits of CPU FP64 output may change during extraction (`0` changed bits gated by `bike-recording-digest` and existing golden tests).
- **Authorized Parallel Batch Contours:** **Both Apple Silicon Metal (`FP32`, `metal:0` via `metal_batch_runner.mm` + `warp-metal`) and NVIDIA CUDA (`FP32`, via `cuda_kernels.cu` + `warp`)** are authorized first-class parallel batch execution contours for `dt = 1.25 ms` closed-loop sweeps (`SolverType.NEWTON` only; `CG` rejected due to ~220×–1000× repeat spread).
- **Required `clean_split` Pipeline & Documented `1.25 ms` Tolerances:**
  - `mujoco_warp 3.11.0` `fwd_position` (`forward.py:668`) invokes `constraint.make_constraint` before `fwd_velocity` (`:1346`), leaving `d.cvel`/`d.cdof_dot` stale by 1 step (`+951.1 rad/s²` `qacc` error on Step 1). All GPU batch drivers MUST execute the **`clean_split`** schedule:
    1. `fwd_kinematics` → `smooth.crb` → `smooth.tendon_armature` → `smooth.factor_m` → `collision_driver.collision` → `smooth.transmission` → `fwd_velocity`;
    2. GPU writers + per-world `(dof_frictionloss, tendon_range)` patches;
    3. `constraint.make_constraint`;
    4. Pass 1 (`fwd_actuation → fwd_acceleration(factorize=False) → solver.solve`) → Rider controller (`SP4`) → Pass 2 (`fwd_actuation → fwd_acceleration(factorize=False) → solver.solve → sensor.sensor_acc → implicit`).
  - Documented `1.25 ms` parity & throughput baselines on `welded.xml` (`nq = 27, nv = 27`, `clean_split` vs CPU MuJoCo FP64, with per-world consistent `tendon_range` slack):

| Contour / Backend | 1-Step `|dqpos|` (p50 / max) | 1-Step `|dqvel|` (p50 / max, rad/s) | 80-Step Repeat `|dqpos|` / `|dqvel|` | 4-Solve Capture Throughput (`nworld = 2048–4096`, `ncon=0`, vs 16T C++ `~15.0k`) |
|---|---|---|---|---|
| **Apple M4 Max (`metal:0`, `FP32`)** | `7.85e-7` / `9.07e-6` | `5.25e-4` / `7.25e-3` | `9.06e-6` / `1.21e-3` | `132.5k – 133.7k` ws/s `welded` (`8.9×`), `108.9k` `distributed` (`7.2×`) |
| **NVIDIA CUDA (`L4`, `FP32`)** | `8.41e-7` / `8.92e-6` | `5.43e-4` / `7.13e-3` | `6.20e-6` / `9.20e-4` | `600.7k – 652.2k` ws/s `welded` (`43.4×`), `632.9k` `distributed` (`42.0×`), `182.7k` `hfield_on` (`12.2×`) |

### 2.2 Engine-Free `bike-recording-digest` & `golden_episode_digest`

- **Schema v1 Invariant:** `bike::recording::schema_version == 1` (`native/src/recording/catalog.hpp:16`), `catalog_data.inc`, and `native/resources/recording/catalog-v1.json` are **untouched**. Digests read only the committed `physics` (`schema.cpp:80`, `catalog-v1.json:11`) and `forces` (`schema.cpp:79`, `catalog-v1.json:160`) DuckDB tables.
- **Engine-Free Architecture:**
  - `native/src/recording/digest.{hpp,cpp}` links only against `bike_recording_reader` + `bike_io` (no MuJoCo symbol dependency, preserving `bike_recording_reader`'s engine-free contract in `RecordingApplication.cmake:193`).
  - Emits canonical JSON containing per-table row count, SHA-256 over exact IEEE-754 little-endian column bit patterns (`physics` state/kinematics + `forces` component rows), and min/max/L2 diagnostics so any 1-ULP CPU drift fails immediately with the first differing table, column, and `interval_id`.
  - **Standalone Engine-Free CLI (NOT via `bike-recording`):** `bike-recording` (`cli/recording.cpp`) links `bike_recording_cli -> bike_golden -> bike_app -> libmujoco` (`RecordingApplication.cmake:200-212`). Therefore, expose a separate engine-free library `bike_recording_digest_cli` (`native/src/cli/recording_digest.{hpp,cpp}`, linking ONLY `bike_recording_reader` + `bike_io`) and standalone uninstalled executable `bin/bike-recording-digest` (`native/src/cli/recording_digest_main.cpp`, implementing `--run <run.duckdb> [--out <digest.json>] [--verify <golden.json>]` plus `--help` for `reference.native_platform_linkage`, with no `bike_rewrite_mujoco_load_name`).
  - Unit tests (`recording_digest` CTest): `native/tests/recording/test_recording_digest.cpp` wired in `native/tests/recording/CMakeLists.txt` alongside `recording_catalog` (`:6-14`), linking ONLY `bike_test_fixtures` + `bike_recording_reader` + `bike_recording_digest_cli` (no `bike_recording`, no MuJoCo load-name rewrite), testing bit-flip sensitivity, column ordering invariance, and missing-table error handling.
- **Golden Episode Digests (`golden_episode_digest`):**
  - `bike-recording golden` (`cli/recording_golden.cpp:19-24`) writes binary `EpisodeArtifact` (`episode.bin`), NOT DuckDB `run.duckdb`. Instead, record the **4 × 30 s (`24,000`-step at `dt = 1.25 ms`)** sealed `schema_version = 1` DuckDB episodes (`{viewer_physics_welded.toml, viewer_physics_distributed.toml}` × `{flat, rough_uphill_extreme}`) via headless **`bike-ride`** (`--duration 30 --check-policy diagnostic`, `recording_retention = "full"`, deterministic equilibrium cache) and hash them into `tests/fixtures/episode-digest/darwin-arm64/` with **`bike-recording-digest`**.
  - Registered under the `slow` CTest label (`golden_episode_digest`), asserting **0 bit diffs** between baseline and refactored `physics_core` runs.

---

## 3. `native/src/physics_core/` Architecture

### 3.1 Header Layout & Tri-Target Compilation Discipline

```text
native/src/physics_core/
  common.hpp             # Tri-target: BIKE_HD / BIKE_THREAD / BIKE_DEVICE / BIKE_CONSTANT, Status, Lit<Real>, fixed kMax*
  gpu_math.hpp           # Tri-target: portable FP32 expf / logf / powf / tanhf / sqrtf / np_interp_segment over IEEE + - * floor ldexp
  gpu_toolkit.hpp        # Tri-target: GpuToolkit (float) deterministic across Host / Metal MSL / NVIDIA CUDA
  tyre_types.hpp         # Tri-target: pure POD structs (GpuWheelChain, GpuTyreModel, GpuPolylineView, GpuCompliantWheelState, StationState)
  kinematics.hpp         # Tri-target: chain_point_jacobian<Real, TK, bool ComputeJr>(...) using d->xanchor / d->xaxis
  terrain_scanner.hpp    # Tri-target: allocation-free O(1)-register PolylineView + scan_polyline_contact<Real, TK, MultiSupportPolicy>
  terrain_distance.hpp   # Tri-target: allocation-free SignedProfileView + height<Real, TK> + signed_distance<Real, TK>
  tyre_contact.hpp       # Tri-target: shared surface_mu, effective_friction, brush_step, transport_bristle (replaces tyre_bristle plan name)
  tyre_compliant.hpp     # Tri-target: step_compliant_wheel<Real, TK, Io> (TireWriter + Slice 1 GPU kernel)
  tyre_distributed.hpp   # Tri-target: step_distributed_wheel<Real, TK, Io> (DistributedTireWriter + Slice 2 GPU kernel)
  exact_math.hpp         # Host-only CPU FP64 wrappers (pyfloat::pow, np_interp_segment, blas::ddot/dgemv/dgemm, PythonSum/NeumaierSum)
  cpu_exact_toolkit.hpp  # Host-only CpuExactToolkit (double) matching native/src 0-bit semantics
  tyre_upload.{hpp,cpp}  # Host-only mjModel*/ProfileQuery builders (build_gpu_tyre_model, build_wheel_chain) + strict bounds checks
```

**Header Inclusion & MSL Stitcher Rules:**
- **MSL Stitcher Exact Order:** `exact_math.hpp`, `cpu_exact_toolkit.hpp`, and `tyre_upload.hpp` (which include `<cmath>`, BLAS, or `mjModel*`) are **never** stitched into MSL. `build_msl_shader_source()` (`metal_batch_runner.mm:101-120`) stitches ONLY the ordered device headers `../physics_core/common.hpp`, `../physics_core/gpu_math.hpp`, `../physics_core/gpu_toolkit.hpp`, `../physics_core/tyre_types.hpp`, `../physics_core/kinematics.hpp`, `../physics_core/terrain_scanner.hpp`, `../physics_core/terrain_distance.hpp`, `../physics_core/tyre_contact.hpp`, `../physics_core/tyre_compliant.hpp`, `../physics_core/tyre_distributed.hpp` before the remaining `gpu/*.hpp` kernels.
- **Standalone Header Policy:** Every `native/src/physics_core/*.hpp` header is self-contained (includes its own dependencies with `#pragma once`, compiles standalone for `check_headers`, with `exact_math.hpp` / `cpu_exact_toolkit.hpp` / `tyre_upload.hpp` guarded or compiled purely on host).

**FP Contraction & Precision Rules (Tri-Target):**
1. **`Lit<Real>(v)` Helper:** Every floating literal inside `physics_core/` uses `Lit<Real>(...)` (`static_cast<Real>(...)` — double when `Real = double`, exact single-precision suffix when `Real = float`) so `Real = float` kernels never suffer accidental `double` promotion under `-Wdouble-promotion` or MSL (which forbids `double`).
2. **CPU (`clang++`/`g++`) & Python C++ Runners:** Compiled with `-ffp-contract=off` (`BikeCompilerPolicy.cmake`). Exact `fma` sites (`np_interp_segment` when `BIKE_NUMPY_INTERP_CONTRACTED=1`, `py_hypot` `DoubleLength`) call `std::fma` explicitly via `CpuExactToolkit`. **Hidden `-ffast-math` & missing include fixes:** Remove `-ffast-math` (replace with `-ffp-contract=off`) and add `-I<native/src>` in **`tools/colab_gpu/metal_hybrid_sweep.py:96,100`** and **`tools/colab_gpu/cpu_batch_sweep.py:105,109,122,127`**.
3. **Apple Metal (`metal_batch_runner.mm` + `warp-metal`):**
   - `make_compile_options()` in `metal_batch_runner.mm:879` replaces `options:nil` with `MTLCompileOptions` setting `options.mathMode = MTLMathModeSafe` and `options.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise`.
   - `build_msl_shader_source()` prepends `#pragma METAL fp contract(off)\n` before the stitched MSL source (`:112`), reaching `warp-metal`'s `wp_metal_load_library` (`tools/colab_gpu/mjwarp_metal_sweep.py:152`) where `MTLCompileOptions` cannot be injected externally.
4. **NVIDIA CUDA (`cuda_kernels.cu` + `build_cuda_writers.py`):**
   - **Remove `--use_fast_math`** from `tools/colab_gpu/build_cuda_writers.py:174` (which forced `--ftz=true --prec-div=false --prec-sqrt=false --fmad=true`). Compile with `--fmad=false` (`-std=c++20 -O3` + `-I<native/src>`), giving IEEE division/sqrt and unfused multiply-add matching CPU `-ffp-contract=off` and Metal `#pragma METAL fp contract(off)`.

### 3.2 `CpuExactToolkit` vs `GpuToolkit` Primitive Contract & 3-Tier Numerical Contract

| Primitive | `CpuExactToolkit` (`double`, 0-bit CPU parity) | `GpuToolkit` (`float`, Host / MSL / CUDA) | Notes |
|---|---|---|---|
| `dot(n, a, b)` | `blas::ddot(n, a, 1, b, 1)` (`cblas_abi.hpp:90-91`) | Sequential `float s = 0.0f; for (i) s += a[i]*b[i];` | Verified `0 / 10,000` bit diffs vs `dgemv(10000,2)` on arm64 Accelerate. |
| `gemv_n3(nv, A, x, y)` | `blas::dgemv(RowMajor, NoTrans, 3, nv, 1, A, nv, x, 1, 0, y, 1)` | Plain row-major `3 × nv` scalar loop | Used for `velocity`, `center_velocity`. |
| `gemv_t3(nv, A, x, y)` | `blas::dgemv(RowMajor, Trans, 3, nv, 1, A, nv, x, 1, 0, y, 1)` | Plain transposed `nv × 3` scalar loop | Used for `jac.T @ force_world`. |
| `ring_lift(n, L, R, out)` | `blas::dgemm(RowMajor, NoTrans, Trans, n, 3, 3, 1, L, 3, R, 3, 0, out, 3)` | Plain `(n,3) @ (3,3).T` loop (`distributed_2d_reference`) | Avoids `dgemm` dependency on device. |
| `interp_segment(s, x, x0, y0, x1, y1)` | `numeric::np_interp_segment` (`std::fma` when `BIKE_NUMPY_INTERP_CONTRACTED=1`, `numeric_numpy.hpp:314-340`) | **Unconditional unfused** `s * (x - x0) + y0` (no FMA) + right-endpoint NaN retry + `y0 == y1` constant fallback | Identical across Host `float`, Metal, and CUDA without macro skew. |
| `hypot2(a, b)` | `biketyre::py_hypot(a, b)` (`numeric_norm.hpp:36-130`, DoubleLength FMA) | Scaled `sqrt(m*m * (1 + (n/m)*(n/m)))` (or `sqrt(a*a + b*b)` avoiding `std::hypot` libcall) | Required by `ProfileQuery` (`profile.hpp:11-13`). |
| `div(a, b)` / `sqrt(x)` / `abs(x)` | `a / b`, `std::sqrt(x)`, `std::abs(x)` | IEEE `a / b`, `metal::sqrt`/`::sqrtf`, `metal::fabs`/`::fabsf` | Under Safe math / no `--use_fast_math`. |
| `isfinite(x)` / `isnan(x)` | `std::isfinite(x)`, `std::isnan(x)` | `metal::isfinite`/`::isfinite`, `metal::isnan`/`::isnan` | — |
| `exp(x)` | `std::exp(x)` (`tire_common.cpp:16`) | Portable Cephes-style `expf_portable` over `+ - * floor ldexp` | Zero `metal::exp`/`__expf` divergence. |
| `tanh(x)` | `std::tanh(x)` (`laws.hpp:56`, `station.cpp:98`) | Portable `tanhf_portable` built on `expf_portable` (`\|x\|>=10 -> ±1`, `\|x\|<1e-4 -> x*(1-x²/3)`, else `(e^(2\|x\|)-1)/(e^(2\|x\|)+1)`) | **Replaces non-portable `metal::tanh`/`::tanhf` in `gpu/common.hpp:173,225`!** Verified `max_rel = 1.69e-7` in `normal_contact_hysteresis`. |
| `sq(x)` / `pow(a, b)` | `x * x` in `tire.cpp:751,757`; `pyfloat::pow(xi_new, 2.0)` in `distributed_tire.cpp:507,738,780` & `tire.cpp:904,909` | `x * x` (GPU never runs `pyfloat::pow`) | `pyfloat::pow(x,2.0)` differs on 115 / 100k floats vs `x*x`. |
| `sum` (`NeumaierSum` / `PythonSum` / `numpy_sum`) | `NeumaierSum` (`tire.cpp:95-111`), `numeric::PythonSum` (`distributed_tire.cpp:144`), `numeric::numpy_sum` (`:785-788`) | Sequential `float` register accumulation | On 128-patch sums, float fold vs `PythonSum` measured `max_rel = 5.11e-7`. |
| `cos_deg(deg)` | `numeric::np_cos(deg * (pi/180))` (`tire.cpp:678`, `profile.cpp:355`) | Host-precomputed `float` in `GpuTyreModel` / `PolylineView` | Zero runtime device `cos`. |

**3-Tier Numerical Contract:**
- **Tier 1 (CPU `FP64`):** `0` bits difference (`memcmp == 0`) across `TireWriter`, `DistributedTireWriter`, `ProfileQuery`, `SignedProfile`, and `bike-recording-digest`.
- **Tier 2 (Isolated GPU `FP32` Kernel vs CPU `FP64`):** `<= 1–2 ULP` on `expf`/`tanhf`/`sqrt` + expected Apple GPU FTZ behavior near denormals (`< 1e-38`); zero `NaN`/`Inf` on valid states; host `GpuToolkit` and Metal/CUDA device outputs agree within Safe IEEE FP32 tolerances.
- **Tier 3 (Closed-Loop `1.25 ms` Multi-Step Rollouts):** Bounded by the Section 2.1 `clean_split` parity table (`1-step |dqvel| <= 1e-2 rad/s`, `80-step |dqpos| <= 2e-5` on healthy trajectories, `<= 1e-5` 1-step repeat spread under `NEWTON`).

### 3.3 Exception-Free `Status` & Zero-Cost `Io` Sink

All `physics_core` entry points are `noexcept` (`BIKE_NONBLOCKING`-safe) and return `physics_core::Status`:

```cpp
enum class Status : std::uint8_t {
    ok = 0,
    invalid_argument,   // non-finite input, bad radius/prev_seg, outside domain, solid road, degenerate <=1e-12, nonunit normal, bad tangent/segment/point state
    overflow,           // derived force/energy non-finite -> CPU maps to std::overflow_error (contact/laws.hpp:13-22, 94-96)
    out_of_bounds,      // nv/njnt/chain/station capacity exceeded
    logic_error         // internal invariant violation -> CPU maps to std::logic_error (distance.cpp:217-218, tire.cpp:667-669)
};
```

- **Exact Exception Preservation:** Every validation gate runs in the exact CPU source order (`profile.cpp:487-501`, `tire.cpp:653-853`, `distributed_tire.cpp:592-854`, covering `:592-598` dt/clock gates, `:631`/`:459-460` pre-Jacobian `height()` domain gate, and `:844-853` snapshot checks), setting a compact error tag (`ErrCode`) alongside `Status` so CPU wrappers (`TireWriter`, `DistributedTireWriter`) throw the **exact existing exception type (`std::invalid_argument` / `std::overflow_error` / `std::logic_error`) and message** on failure without any heap allocation on the `Status::ok` path.
- **`Io` Callback / Sink (`Io` template parameter):**
  - GPU passes `NoOpIo{}` (compiles away at `-O2`, zero registers/writes for `TireDiagnostics`, `TireSnapshot.patches`, `multi_support`, `radial_loss_power`, `energy`, `loss`, and `ProfileContact::multi_support`).
  - CPU passes `CompliantCpuIo` / `DistributedCpuIo` receiving the exact scalar telemetry, patch structs, `multi_support` flag, and `pyfloat::pow` shear energy needed for bit-identical `TireSnapshot`, `TireDiagnostics`, and `effective_radius`.

### 3.4 Two-Commit `O(1)` Polyline Contact Scanner (`terrain_scanner.hpp`)

`biketyre::ProfileQuery::contact` (`native/src/tyre/profile.cpp:483-656`) currently populates 5 `24,001`-element `std::vector` scratch buffers (`cand_scratch_`, `keep_scratch_`, `near_scratch_`, `normals_scratch_`, `dots_scratch_` in `profile.hpp:114-118`). Migration happens in two strictly verified commits:

1. **Commit A (Verbatim Extract):** Move `ProfileQuery::contact` mechanics into `physics_core/` preserving 0-bit CPU parity via `CpuExactToolkit`.
2. **Commit B (`O(1)`-Register Two-Pass Scanner):** Replace all 5 `24,001`-element scratch buffers with a stateless streaming scan (`scan_polyline_contact<Real, TK, MultiSupportPolicy>`) keeping only `O(1)` thread registers (`winner_id`, `winner_pt[2]`, `winner_dist`, `has_prev_tie`, `prev_pt[2]`, `prev_dist`, `min_dist`), with **five load-bearing invariants** proved against `34,000` test queries (`0` bit diffs in `0.104 s`):
   - **Flat-Window Fast Path (`profile.cpp:504-557`):** Evaluated in `O(1)` registers whenever `height < radius && height_changes[hi] == height_changes[lo]`, using `TK::hypot2` and `TK::dot(2, ...)` (`endpoint` `closer` check, `<= 1e-12` `previous_segment` tie-break).
   - **Pass 1 & Airborne Expansion (`:561-583`):** Scans `[lo, hi)` first; ONLY if empty (`hi <= lo`) or `min_dist >= radius` (airborne) does it compute `vertical_lower = max(0, c_z - maximum_z)`, `reach = sqrt(max(0, best^2 - vertical_lower^2)) + 1e-8`, `[search_lo, search_hi)`, and re-scan via `scan_window_o1`. *(Note: on `rough_uphill_extreme`, `maximum_z ~ 20 m` at the crest causes airborne wheels near the bottom to scan up to `1,348` segments (`mean 629.54`); still strictly `O(1)` memory.)*
   - **Asymmetric `endpoint_keep` (`:584-601`):** `argmin` (`dist < winner_dist`) MUST NOT filter by `endpoint_keep` (`111` adversarial queries had an unkept segment win `argmin`!). `endpoint_keep` (`eval_endpoint_keep`) is computed on the fly in registers **only** at `seg == *previous_segment` (for the `<= 1e-12` tie-break) and inside the optional CPU `multi_support` pass.
   - **Degenerate Gate Order:** Check `min_dist <= 1e-12` (`:581`) before committing `winner`, matching CPU `min_elt(distances) <= 1e-12`.
   - **Separated `multi_support` Policy:** GPU uses `NoMultiSupport` (single pass, zero `normals_scratch_`/`dots_scratch_`). CPU uses `O1CpuMultiSupport` scanning `[win_lo, win_hi)` once in registers with `TK::dot(2, normal_i, normal_winner)` (`0 / 10,000` bit diffs vs `blas::dgemv(10000, 2)` on arm64 Accelerate) and `sqrt(ex*ex + ez*ez) > 1e-10`.

---

## 4. Shared Tyre Models, `nq = 27` Kinematics, Dual-Backend Wiring, & Verification

### 4.1 `GpuTyreModel`, `GpuWheelChain` (`nq = 27`), Bounds Safety, & `TabulatedRadial` Rejection

- **`nq = 27` Clutch Topology (`welded.xml:95-120, 192-193, 417-426`):** Live research models include `drive_shaft` (`nbody = 29`) + `drive_shaft_spin` hinge (`nq = 27, nv = 27, njnt = 27, ntendon = 2`). Note `drive_shaft` is a child of `frame` on the crank path, NOT an ancestor of `front_wheel` (`frame -> steer -> fork_lower -> front_wheel`, where `steer` has `0` joints -> **5 joints**: `root_x, root_z, root_pitch, fork_travel, front_wheel_spin`) or `rear_wheel` (`frame -> chainstay -> seatstay -> rear_wheel`, **6 joints**: `root_x, root_z, root_pitch, main_pivot, horst_pivot, rear_wheel_spin`); both are `<= kMaxChainJoints = 8`.
- **Host-Built `GpuWheelChain` & `GpuTyreModel` (`tyre_types.hpp` + `tyre_upload.{hpp,cpp}`):**
  - Built once from `const mjModel *m`: walks `m->body_parentid` from each wheel body (`front_wheel`, `rear_wheel`) to world (`0`), recording **`{joint_id, joint_type, dof_adr}`** up to `kMaxChainJoints = 8` plus `njnt` in `GpuTyreModel`, and validating `0 <= joint_id < njnt`. Storing `joint_id` is mandatory because `chain_point_jacobian` indexes live world-frame `d->xanchor[3 * joint_id]` and `d->xaxis[3 * joint_id]` (`m->jnt_axis` is body-local and stale once `frame` pitches or `fork_travel` slides).
  - **Wheel Geom Alignment Gate:** CPU reads `d->geom_xpos[geom]` (`tire.cpp:691`, `distributed_tire.cpp:629`), whereas GPU reads `d->xpos[wheel_body]`. `build_gpu_tyre_model` validates `m->geom_bodyid[geom] == wheel_body`, `m->geom_pos[geom] == (0, 0, 0)`, and identity `m->geom_quat[geom]` at upload time so both paths coincide identically.
  - **`TabulatedRadial` Rejection:** Unreachable in `viewer_physics_{welded,distributed}.toml`, `rider_type_*.toml`, built-in presets, and `reference.native_tire`. `TireWriter` retains `radial_tables_` as a CPU-only fallback; `build_gpu_tyre_model` / `tyre_upload` rejects `mat.table.has_value()` with an explicit error.
- **Closing Nov 16 Audit Findings (`R1-2.3b/c, R2-4a/b, BD1–BD4`) & Strict Bounds Checks:**
  - **R1-2.3c & BD4 (`GpuModelIndices` `nq = 27` & Freehub Driver):** Add `drive_shaft_spin_qpos = 14`, `drive_shaft_spin_dof = 14`, and `crank_clutch_tendon = 1` to `GpuModelIndices` (`gpu/common.hpp:97-145`) and `GpuModelIndicesStruct` (`tools/colab_gpu/build_cuda_writers.py:18-61,349-350`), shifting `rider_qpos`/`rider_dof` defaults `+1` (`{23, 24, 25, 26, 18, 19, 20, 21, 22}`) and updating `static_assert(sizeof(GpuModelIndices) == 240)` (`60` ints). Switch `test_gpu_model_geometry.cpp:89` from `model.xml` (`nq = 26`) to `tests/fixtures/model/welded.xml` (`nq = 27`) as the default `GpuModelIndices{}` drift-guard target, update `test_gpu_rider.cpp:198-209` for the shifted defaults, compute `freehub_boundary_rad` / `phi` from `drive_shaft_spin_qpos >= 0 ? drive_shaft_spin_qpos : crank_spin_qpos` (`drivetrain_resistance_kernel.hpp:117,131`, `metal_batch_runner.mm:220-228,612-620`, `cuda_kernels.cu:39-41`, `cpu_batch_runner.cpp:376-384`), and resolve `idx` dynamically from `mjModel` when loading legacy `nq = 26` fixtures (`test_gpu_batch_runners.cpp:168`, `tools/colab_gpu/test_model_assets.py:39-40`).
  - **R2-4a & BD2 (`tendon_range_batch` Indexing):** Fix `batch_step.hpp:216` and `metal_batch_runner.mm:301` from `int` `w * tendon_range_world_stride + 1` to `static_cast<long>(w) * tendon_range_world_stride + 2 * idx.freehub_tendon + 1` (validating `2 * idx.freehub_tendon + 1 < tendon_range_world_stride`).
  - **R2-4b & BD3 (Legacy Ban Violations):** Deleting `gpu/tire_kernel.hpp` removes `bristle_cx_ns_m = 35` (`R2-4b(1–3)`), and SP1 zeroes `.c_edge_nm_s_rad = 0.0f` across **`rider_kernel.hpp:31` AND every entry in `GpuRiderParams::joints[0..8]` (`rider_kernel.hpp:62-111`)**, plus removes/zeroes the `:197` `torque -= c * ramp * qd_rad_s` damping term (updating `test_gpu_rider.cpp:130` as needed) to close Ban #26 / BD3 completely; full extraction of `suspension_kernel.hpp` and `rider_kernel.hpp` remains scoped to SP2 (`§1.2`).
  - **Exhaustive Upload & Launch Bounds Gates (`R1-2.3b`, `BD1`):** Every CPU, Metal, and CUDA entry point (`cuda_kernels.cu`, `metal_batch_runner.mm`, `cpu_batch_runner.cpp`, `build_cuda_writers.py`, `mjwarp_metal_sweep.py`) MUST validate: `nworld > 0`, `idx.{nq,nv,nu,nbody}` bounded by `kMax*` AND equal to loaded `m->{nq,nv,nu,nbody}`, every `qpos`/`dof`/`body`/`tendon` index in `[0, max)`, `0 <= chain[i].joint_id < njnt`, `wheel.chain.joint_count <= kMaxChainJoints`, `station_count <= kMaxGpuTyreStations (128)` (when distributed), and `polyline.num_vertices >= 2` with monotonic `px`. Invalid bounds return an explicit nonzero status code / error (no silent no-op `return;`).

### 4.2 `chain_point_jacobian` (`kinematics.hpp`)

Computes MuJoCo-exact analytical 3×`nv` point Jacobians directly from world-frame `(xpos, xanchor[3 * joint_id], xaxis[3 * joint_id])` along the 5/6-DOF ancestor chain (note: `Stepper::set_state`, `native/src/stepper.cpp:556-609`, sets `posvel_fresh_ = false` at `:594` and defers `mj_forward`, so tests reading `xpos`/`xmat`/`xanchor`/`xaxis` after `set_state` MUST call `forward()` first):
- **Compliant Mode (`jr == nullptr`):** For `compliant_2d`, `TireWriter` (`tire.cpp:697-698`) calls `mj_jac(m, d, jacp, nullptr, pt, body)` twice (`p` and `center`). Template parameter `bool ComputeJr = false` avoids allocating `3 * kMaxNv` (`96`) registers for `jr` and skips angular cross terms (`cross(axis, pt - anchor)` only for `jp`).
- **Distributed Mode (`ComputeJr = true`):** `DistributedTireWriter` (`distributed_tire.cpp:634`) calls `mj_jac` once at `center` (`jp_center`, `jr`), rotates `128` local ring points via `TK::ring_lift(128, local, xmat[body], points)`, and computes `jac_station[k] = jp_center[k] + (jr[a] * arm[b] - jr[b] * arm[a])` (`:673-681`).

### 4.3 Slice 1 (`compliant_2d`) & Slice 2 (`distributed_2d_reference`) Execution & State

| Property | Slice 1: `compliant_2d` (`tyre_compliant.hpp`) | Slice 2: `distributed_2d_reference` (`tyre_distributed.hpp`) |
|---|---|---|
| **Terrain Query** | `O(1)` `scan_polyline_contact` (`O1CpuMultiSupport` on CPU; `NoMultiSupport` on GPU) | `SignedProfile` `height` sweep + `signed_distance` (`128` stations) |
| **Normal Law** | `normal_contact_hysteresis(delta, delta_dot, rk, eta, eps)` (`laws.hpp:48-62`) with `delta_dot = -TK::dot(3, vel, n)` (`tire.cpp:709`) | `station_response(pen, rate, w, density)` (`station.cpp:77-101`) with plain fold `rate = -((v0*n0 + v1*n1) + v2*n2)` (`:690-693`) |
| **Continuity / Transport** | **3-gate rule** (`adjacent \|\| coincident \|\| continuous` with `center`, `tire.cpp:729-760`), `release_loss = 0.5*tk*(xi^2 - transp^2)` (`x * x`) | Unconditional transport when active (`:709-719`); detached stations reset `{false, 0, no_tangent}`, CPU logs `0.5*k_s*pyfloat::pow(old.xi, 2.0)` (`:776-781`) |
| **Airborne Behavior** | **`GpuCompliantWheelState` updated even when `delta <= 0`** (`tire.cpp:843`) so `segment` persists for `ProfileQuery::contact` tie-breaking! | Detached stations clear `StationState` (`active = false`) |
| **Per-Wheel Persistent State** | `~56 B/wheel` (`xi`, `tangent[3]`, `point[3]`, `center[3]`, `int32 segment`, `uint8 flags{has_tangent, has_center}`) = **`~112 B/world`** (`448 KiB` @ 4,096 worlds) | `128 × {active, xi, tangent[3]}` = `~2,560 B/wheel` = **`~5,120 B/world`** (`20.0 MiB` @ 4,096 worlds) |
| **`effective_radius_m` Output** | `delta > 0 ? max(0, -dot3(p - center, n)) : 0` (exact single-patch equivalence to `NeumaierSum`, `tire.cpp:162-177`) | Load-weighted mean over `loads_` via `PythonSum` (CPU) / sequential `float` (GPU), fallback unweighted mean (`distributed_tire.cpp:135-155`) |

*(Note: Both slices share `surface_mu`, `effective_friction`, and `brush_step` in `physics_core/tyre_contact.hpp`.)*

### 4.4 Simultaneous Apple Metal + NVIDIA CUDA Wiring, `TESTING.md`, & Legacy Removal

- **Apple Metal (`metal_batch_runner.mm` & `tools/colab_gpu/{mjwarp_metal_sweep,metal_hybrid_sweep}.py`):**
  - Stitch ONLY the ordered tri-target `physics_core` device headers (`../physics_core/common.hpp`, `gpu_math.hpp`, `gpu_toolkit.hpp`, `tyre_types.hpp`, `kinematics.hpp`, `terrain_scanner.hpp`, `terrain_distance.hpp`, `tyre_contact.hpp`, `tyre_compliant.hpp`, `tyre_distributed.hpp`) in `build_msl_shader_source()` (`metal_batch_runner.mm:103-110`) with `#pragma METAL fp contract(off)` + `MTLCompileOptions` (`MTLMathModeSafe` + `MTLMathFloatingPointFunctionsPrecise`).
  - Expose uniform `bike_wp_writers_metal_kernel` / `bike_metal_step_writers_kernel` calling `physics_core` Slice 1 (`compliant_2d`) or Slice 2 (`distributed_2d_reference`) depending on `GpuTyreConfig.backend` (`1 thread = 1 world`, reading `d.xpos`/`d.xmat`/`d.xanchor`/`d.xaxis`/`d.qvel`/`d.qpos` directly inside `clean_split` + `wp.ScopedCapture()`), and update `bike_metal_benchmark_writers_only` (`metal_batch_runner.mm:1280`, `metal_hybrid_sweep.py:181`) for the unified scalar `physics_core` path.
- **NVIDIA CUDA (`cuda_kernels.cu`, `cuda_kernels_abi.hpp`, `tools/colab_gpu/{build_cuda_writers,mjwarp_batch_sweep}.py`):**
  - Include the identical `physics_core/*.hpp` device headers in `cuda_kernels.cu` (`#include` guarded under `__CUDACC__` and host `cuda_kernels_host.cpp`).
  - Remove `--use_fast_math` from `build_cuda_writers.py:174`, replace with `--fmad=false` (`-std=c++20 -O3 -I<native/src>`).
  - Pass `xmat` (`(nworld, nbody, 3, 3)`), `xanchor`, and `xaxis` (`(nworld, njnt, 3)`) alongside `xpos`/`qpos`/`qvel` in both CUDA and Metal ABIs (`distributed_2d_reference` requires `xmat`, and `kinematics.hpp` reads `xanchor`/`xaxis` from `mjData`).
- **CPU Batch Runner (`native/src/gpu/cpu_batch_runner.cpp` & `tools/colab_gpu/cpu_batch_sweep.py`):**
  - Migrated to the same `physics_core` tyre entry points (`GpuToolkit` FP32 parity lane + host verification), compiled with `-ffp-contract=off -I<native/src>` (`-ffast-math` removed).
- **`docs/TESTING.md:1054-1060` Suppression Register Sync:**
  - Update the suppression table (`docs/TESTING.md:1054-1056`) to remove deleted `tire_kernel.hpp`, register the new tri-target `physics_core` raw-pointer / C-array / positional-signature device headers (`common`, `gpu_math`, `gpu_toolkit`, `tyre_types`, `kinematics`, `terrain_*`, `tyre_*`), and update `static_assert(sizeof(GpuModelIndices) == 240)` so `run_native_checks.sh` (`diagnostic-controls` / `contracts`) stays green.
- **Legacy Deletions (Zero Dual Legacy Maintenance):**
  - Delete `native/src/gpu/tire_kernel.hpp` completely.
  - Delete `num_lateral_ribs` from `cuda_kernels_abi.hpp`, `batch_step.hpp`, `cuda_kernels.cu`, `metal_batch_runner.mm`, `build_cuda_writers.py`, `mjwarp_batch_sweep.py`, `mjwarp_metal_sweep.py`, `metal_hybrid_sweep.py`, `cpu_batch_sweep.py` (replaced by 2D planar `compliant_2d` and `distributed_2d_reference`).
  - Delete `bike_gpu_writers_simd32_kernel`, `bike_wp_writers_simd32_kernel`, `bike_metal_step_writers_simd32_kernel`, `bike_metal_step_writers_fused_simd32_kernel`, and `use_simd32` dispatch from `metal_batch_runner.mm` and `mjwarp_metal_sweep.py`.

---

## 5. Verification Matrix & 3-Level Test Budget (`AGENTS.md`)

| Check / Target | Scope & Pass Criterion | Cadence (`AGENTS.md` Budget) |
|---|---|---|
| `recording_digest` | Bit-flip sensitivity, `physics` + `forces` SHA-256 over `schema_version = 1`, engine-free linkage (`bike_test_fixtures` + `bike_recording_reader` + `bike_recording_digest_cli`), plus `reference.native_platform_linkage` `--help` check on `bin/bike-recording-digest` | **Level 1 (`mise run test` per task + `check_change.sh` per stage)** |
| `test_physics_core_math` | `GpuToolkit` `exp`/`tanh`/`hypot2`/`interp` <= `1–2 ULP` vs CPU, `CpuExactToolkit` 0-bit parity, Metal `MTLMathModeSafe` vs CUDA `--fmad=false` parity | **Level 1** |
| `test_terrain_scanner` | `O1PolylineScanner` (`>= 20,000` adversarial + `flat` + `rough_uphill_extreme` queries) **0-bit identical** to legacy `ProfileQuery::contact` + 0-alloc `BIKE_NONBLOCKING` RTSan pass | **Level 1** |
| `native_distance_tests`, `native_station_tests`, `reference.tyre_road_profile`, `reference.native_tire`, `reference.distributed_tire_reference`, `reference.native_distributed_tire`, `reference.native_distributed_stands` | **0-bit CPU identity** (`TireWriter` & `DistributedTireWriter` wired through `physics_core` `<double, CpuExactToolkit>`) | **Level 1 (`check_change.sh` runs ordinary suite; slow reference tyre tests run targeted during tyre tasks + Level 3)** |
| `bike_test_gpu_tire` (rewritten in-place), `bike_test_gpu_batch_step`, `bike_test_gpu_batch_runners`, `bike_test_gpu_model_geometry` (`CMakeLists.txt:688`), `bike_test_gpu_rider`, `bike_test_gpu_drivetrain_resistance` | `nq = 27` `GpuWheelChain` (`forward()` called after `Stepper::set_state`, `stepper.cpp:594`), `sizeof(GpuModelIndices) == 240` (`welded.xml` default drift guard at `test_gpu_model_geometry.cpp:89`, shifted `test_gpu_rider.cpp:198-209`, zeroed `c_edge_nm_s_rad` at `rider_kernel.hpp:31,62-111,197`) + `drive_shaft_spin` bounds/rejection tests (`R1-2.3b/c, R2-4a/b, BD1–BD4`), `TabulatedRadial` GPU rejection, Metal `metal:0` (`clean_split` + `ScopedCapture`) & CPU runner parity (`~1e-6` 1-step) | **Level 1** |
| `golden_episode_digest` (`slow` label) | 30 s (`24,000`-step) `bike-ride` digests for `viewer_physics_{welded,distributed}.toml` × `{flat, rough_uphill_extreme}`: **0 changed bits** | **Targeted during tyre tasks + Level 3 (`run_tests.sh full`) at completion** |
| `bash tools/run_tests.sh native` + `full` | Full static-analysis sweeps (`clang-tidy`/`cppcheck`/frontends/contracts with synced `docs/TESTING.md:1054-1060`), ASan/RTSan, and all `slow`/`package` gates | **Once per merge (`native` + `full` final proof)** |
