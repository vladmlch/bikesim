# Subproject 1: Shared GPU/CPU Tyre & Terrain Force Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract a single-source, allocation-free, header-only C++ template force core (`native/src/physics_core/`) shared bitwise-identically by CPU FP64 (`TireWriter` `compliant_2d`, `DistributedTireWriter` `distributed_2d_reference`, `ProfileQuery` `O(1)` scanner, `SignedProfile`) and deterministically by Apple Metal FP32 and NVIDIA CUDA FP32 on the live `nq = 27` clutch model, gated by an engine-free Schema v1 digest oracle.

**Architecture:** All shared physics laws and terrain queries live as `<Real, TK>` header-only templates inside `native/src/physics_core/` parameterized over `CpuExactToolkit` (`double`, 0-bit Accelerate BLAS / `std::fma` / `pyfloat::pow` / `NeumaierSum` / `PythonSum`) and `GpuToolkit` (`float`, sequential loops, unconditional unfused `interp_segment`, portable Cephes `expf`/`tanhf`). Host-only `mjModel*`/BLAS/cmath headers (`exact_math.hpp`, `cpu_exact_toolkit.hpp`, `tyre_upload.{hpp,cpp}`) stay isolated from the ordered MSL-stitched/CUDA-included device headers (`common`, `gpu_math`, `gpu_toolkit`, `tyre_types`, `kinematics`, `terrain_scanner`, `terrain_distance`, `tyre_contact`, `tyre_compliant`, `tyre_distributed`).

**Tech Stack:** C++23 host (`clang++`/`g++`, `-ffp-contract=off`), Objective-C++ / Apple Metal Shading Language (`MTLMathModeSafe` + `MTLMathFloatingPointFunctionsPrecise` + `#pragma METAL fp contract(off)`), NVIDIA CUDA (`nvcc --fmad=false`), MuJoCo `3.12.0`, `mujoco_warp 3.11.0` (`clean_split` + `wp.ScopedCapture()`), DuckDB (`bike_recording_reader`), CMake/CTest (`build/native-release`), `mise`.

**Spec:** `docs/superpowers/specs/2026-11-17-gpu-shared-tyre-core-design.md` (commit `594cf4b6`)

## Global Constraints

- **CPU FP64 Bit-Identity Invariant:** Zero bits of CPU FP64 output may change across `TireWriter`, `DistributedTireWriter`, `ProfileQuery`, `SignedProfile`, `contactlaw::`, and Schema v1 DuckDB `physics` + `forces` tables; gated by `bike-recording-digest` + existing reference goldens.
- **Schema v1 Freeze:** `bike::recording::schema_version == 1` (`native/src/recording/catalog.hpp:16`), `catalog_data.inc`, and `native/resources/recording/catalog-v1.json` MUST remain untouched.
- **Engine-Free Recording Digest Boundary:** `bike_recording_digest_cli`, `bin/bike-recording-digest`, and `recording_digest` CTest MUST link ONLY `bike_recording_reader` + `bike_io` (+ `bike_test_fixtures` in test), with zero `libmujoco` edge.
- **Tri-Target FP Contraction Parity:** CPU builds compile with `-ffp-contract=off`; Metal compiles with `MTLMathModeSafe` + `MTLMathFloatingPointFunctionsPrecise` + `#pragma METAL fp contract(off)`; CUDA compiles with `--fmad=false` (never `--use_fast_math`); Python C++ runners replace `-ffast-math` with `-ffp-contract=off` and add `-I<native/src>`.
- **Contracts & Compiler Policy Compliance:**
  - Never write bare `mj_step(`, `mj_forward(`, `mj_setConst(` (even in comments) in `native/src/**/*.cpp` outside `engine_call` (`check_contracts` rejects them).
  - Never construct raw `(ptr, size)` `std::span` outside explicitly `docs/TESTING.md`-scoped boundary helpers (`-Wunsafe-buffer-usage`).
  - All `native/src/physics_core/*.hpp` headers MUST be standalone-compilable (`check_headers`), and every raw-pointer / C-array / positional-signature device header MUST be registered in `docs/TESTING.md:1054-1060`.

---

## `AGENTS.md` Test Budget

Baseline timings from `.git/bike-gate/timings.tsv` on `physics-2026-11-05`:
- `2e456d24` (`native`, 16 workers): **676 s** (~11.3 min: `configure=9, build=18, ctest=67, cppcheck=12, context=53, diagnostic-controls=2, contracts=4, frontends=59, headers=32, odr=152, tidy=268`).
- `a96033ce` (`full`, 16 workers): **2077 s** (~34.6 min: `native` + `slow=1237, package=101`).

Budget allocation across 3 stages and 9 tasks:
- **Per-task inner loop (Tasks 1–8):** `mise run test '<pattern>'` only (~5–45 s per task; `golden_episode_digest` and slow tyre reference tests run targeted in Tasks 2, 4, 6, 7 only).
- **Stage 1 gate (end of Task 2):** `bash tools/check_change.sh <stage1_base>` (`<= 5 min`, covers policy + `bike-recording-digest` + golden fixtures).
- **Stage 2 gate (end of Task 5):** `bash tools/check_change.sh <stage2_base>` (`<= 5 min`, covers `physics_core` math + `O(1)` scanner + `nq = 27` kinematics + audit closures).
- **Merge completion proof (Task 9):** `1 × bash tools/run_tests.sh native` (~11–12 min) followed by `1 × bash tools/run_tests.sh full` (~35 min) after Task 8 edits complete. Zero redundant `native`/`full` sweeps mid-branch.

---

## File Structure

| Path | Action | Responsibility |
|---|---|---|
| `docs/agents/simulation-decisions.md` | Modify (`:268-274, :824-825`) | Update Item 29 & Rejected Item 13 for `clean_split` Metal + CUDA FP32 parity |
| `native/src/recording/digest.{hpp,cpp}` | Create | Engine-free DuckDB `physics` + `forces` SHA-256 & column/interval diff digest |
| `native/src/cli/recording_digest.{hpp,cpp}` | Create | Engine-free CLI library `bike_recording_digest_cli` |
| `native/src/cli/recording_digest_main.cpp` | Create | Standalone uninstalled `bin/bike-recording-digest` (`--help` for `reference.native_platform_linkage`) |
| `native/cmake/RecordingApplication.cmake` | Modify (`:212`) | Register `bike_recording_digest_cli` & `bin/bike-recording-digest` without `libmujoco` |
| `native/tests/recording/CMakeLists.txt` & `test_recording_digest.cpp` | Modify / Create | Register engine-free `recording_digest` CTest alongside `recording_catalog` |
| `CMakeLists.txt` | Modify | Wire `test_physics_core_math`, `test_terrain_scanner`, `golden_episode_digest` (`slow`) |
| `tests/fixtures/episode-digest/darwin-arm64/*.json` | Create | 4 × 30 s (`24,000`-step) `bike-ride` Schema v1 golden digests (`welded`/`distributed` × `flat`/`rough_uphill_extreme`) |
| `native/tests/recording/test_golden_episode_digest.cpp` | Create | `golden_episode_digest` (`slow`) regression verifying 0-bit digest match |
| `native/src/physics_core/common.hpp` | Create | Tri-target macros, `Status`, `ErrCode`, `Lit<Real>`, `kMax*` bounds constants |
| `native/src/physics_core/gpu_math.hpp` | Create | Portable FP32 Cephes `expf_portable`, `tanhf_portable`, `interp_segment_unfused` |
| `native/src/physics_core/gpu_toolkit.hpp` | Create | Tri-target `GpuToolkit` (`float`, sequential reductions, zero FMA contraction) |
| `native/src/physics_core/tyre_types.hpp` | Create | Templated tri-target POD views/states (`PolylineView<Real>`, `SignedProfileView<Real>`, `StationRingView<Real>`, `TyreModel<Real>`, `CompliantWheelState<Real>`, `StationStatePod<Real>`) |
| `native/src/physics_core/exact_math.hpp` | Create | Host-only FP64 `pyfloat::pow`, `np_interp_segment`, `py_hypot`, BLAS, `PythonSum`/`NeumaierSum` |
| `native/src/physics_core/cpu_exact_toolkit.hpp` | Create | Host-only `CpuExactToolkit` (`double`) preserving 0-bit CPU parity |
| `native/src/physics_core/terrain_scanner.hpp` | Create | Tri-target `O(1)`-register `scan_polyline_contact<Real, TK, MultiSupportPolicy>` + `NoMultiSupport` / `O1CpuMultiSupport` |
| `native/src/physics_core/terrain_distance.hpp` | Create | Tri-target allocation-free `profile_height<Real, TK>` & `signed_profile_query<Real, TK>` |
| `native/src/physics_core/kinematics.hpp` | Create | Tri-target `chain_point_jacobian<Real, TK, bool ComputeJr>` over world `xanchor`/`xaxis` |
| `native/src/physics_core/tyre_upload.{hpp,cpp}` | Create | Host-only `build_wheel_chain` & `build_gpu_tyre_model` (`TabulatedRadial` + `geom_pos==0` gates) |
| `native/src/physics_core/tyre_contact.hpp` | Create | Tri-target `surface_mu`, `effective_friction`, `normal_contact_hysteresis`, `brush_step`, `transport_bristle` |
| `native/src/physics_core/tyre_compliant.hpp` | Create | Tri-target `step_compliant_wheel<Real, TK, Io>` + `NoOpIo` |
| `native/src/physics_core/tyre_distributed.hpp` | Create | Tri-target `station_density`, `station_response`, `step_distributed_wheel<Real, TK, Io>` |
| `native/src/tyre/{profile,distance,station}.{hpp,cpp}` | Modify | Delegate to `physics_core`, deleting `ProfileQuery`'s 5 `24,001`-element scratch vectors |
| `native/src/contact/laws.hpp` & `native/src/writers/{tire_common,tire,distributed_tire}.{hpp,cpp}` | Modify | Delegate CPU `compliant_2d` & `distributed_2d_reference` through `physics_core` `<double, CpuExactToolkit>` |
| `native/src/gpu/{common.hpp,batch_step.hpp,drivetrain_resistance_kernel.hpp,rider_kernel.hpp}` | Modify | `GpuModelIndices` `sizeof == 240` (`drive_shaft_spin`, `crank_clutch_tendon`), `long` tendon stride, zero `c_edge` |
| `native/src/gpu/tire_kernel.hpp` | Delete | Remove legacy `bristle_cx_ns_m=35` / `num_lateral_ribs` tyre kernel |
| `native/src/gpu/{cuda_kernels_abi.hpp,cuda_kernels.cu,metal_batch_runner.mm,cpu_batch_runner.cpp}` | Modify | Wire ordered `physics_core` headers, `xmat`/`xanchor`/`xaxis`, `MTLMathModeSafe` + `#pragma METAL fp contract(off)`, delete `simd32` & `num_lateral_ribs`, strict bounds gates |
| `tools/colab_gpu/{build_cuda_writers,mjwarp_batch_sweep,mjwarp_metal_sweep,metal_hybrid_sweep,cpu_batch_sweep}.py` | Modify | `--fmad=false` / `-ffp-contract=off`, `-I<native/src>`, `clean_split`, `GpuModelIndicesStruct` 240B, delete `num_lateral_ribs` & `use_simd32` |
| `docs/TESTING.md` | Modify (`:1054-1060`) | Remove `tire_kernel.hpp`, add tri-target `physics_core` headers, update `sizeof(GpuModelIndices) == 240` |
| `native/tests/physics_core/` & `native/tests/gpu/` | Create / Modify | `test_physics_core_math`, `test_terrain_scanner`, updated `test_gpu_{model_geometry,rider,drivetrain_resistance,batch_step,tire,batch_runners}.cpp` |

---

### Task 1: Policy Update & Standalone Engine-Free `bike-recording-digest`

**Files:**
- Modify: `docs/agents/simulation-decisions.md:268-274, :824-825`
- Create: `native/src/recording/digest.hpp`, `native/src/recording/digest.cpp`
- Create: `native/src/cli/recording_digest.hpp`, `native/src/cli/recording_digest.cpp`, `native/src/cli/recording_digest_main.cpp`
- Modify: `native/cmake/RecordingApplication.cmake:200-224`
- Modify: `native/tests/recording/CMakeLists.txt:5-15`
- Create: `native/tests/recording/test_recording_digest.cpp`

**Interfaces:**
- Consumes: `bike::recording::Recording` (`native/src/recording/reader.hpp:24-48`), `bike::io::Value`, `bike::io::sha256_hex` (NO `libmujoco`, NO change to `schema_version == 1`).
- Produces (namespace `bike::recording` / `bike::cli`):
  - `struct ColumnDigest { std::string name; std::uint64_t rows; std::string sha256; double min_val; double max_val; double l2_norm; };`
  - `struct TableDigest { std::string table; std::uint64_t row_count; std::vector<ColumnDigest> columns; };`
  - `struct DigestReport { std::int64_t schema_version; std::vector<TableDigest> tables; std::string overall_sha256; };`
  - `struct DigestDiff { bool equal; std::string table; std::string column; std::int64_t interval_id; std::string detail; };`
  - `DigestReport compute_recording_digest(const Recording &rec);`
  - `DigestDiff compare_recording_digests(const DigestReport &lhs, const DigestReport &rhs);`
  - `int bike::cli::run_recording_digest(int argc, const char *const *argv, std::ostream &out, std::ostream &err);`
  - Engine-free static library `bike_recording_digest_cli` and uninstalled executable `bin/bike-recording-digest`.

- [ ] **Step 1: Update `docs/agents/simulation-decisions.md` Item 29 & Rejected Item 13**

Edit `docs/agents/simulation-decisions.md:268-274` (Item 29) and `:824-825` (Rejected Item 13) to lock in: CPU `FP64` remains the sole physics-acceptance oracle (`0` changed bits gated by `bike-recording-digest` and golden tests), while both **Apple Silicon Metal (`FP32`)** and **NVIDIA CUDA (`FP32`)** are authorized first-class parallel batch contours for `dt = 1.25 ms` `SolverType.NEWTON` rollouts under `clean_split` (`fwd_velocity` before `make_constraint`, ending in `implicit`) with documented Wave 3 baselines (`M4 Metal`: `|dqpos| max 9.07e-6`, `|dqvel| max 7.25e-3`, `133.7k` ws/s `8.9×` 16T C++; `L4 CUDA`: `|dqpos| max 8.92e-6`, `|dqvel| max 7.13e-3`, `652.2k` ws/s `43.4×` 16T C++).

- [ ] **Step 2: Write the failing `test_recording_digest.cpp` & wire `native/tests/recording/CMakeLists.txt`**

```cpp
// native/tests/recording/test_recording_digest.cpp
#include "cli/recording_digest.hpp"
#include "recording/digest.hpp"
#include "recording/reader.hpp"
#include "support/fixture_test.hpp"
#include <cassert>
#include <sstream>

int main() {
    // Fixture creates base_run.duckdb and mutated_1ulp.duckdb (physics.root_z_m at interval_id=42 flipped +1 ULP)
    // plus missing_forces.duckdb (no forces table).
    const auto rec_a = bike::recording::open_recording(base_run_path);
    const auto rec_b = bike::recording::open_recording(mutated_1ulp_path);
    const auto d_a = bike::recording::compute_recording_digest(*rec_a);
    const auto d_b = bike::recording::compute_recording_digest(*rec_b);
    assert(d_a.schema_version == 1);
    const auto diff = bike::recording::compare_recording_digests(d_a, d_b);
    assert(!diff.equal);
    assert(diff.table == "physics");
    assert(diff.column == "root_z_m");
    assert(diff.interval_id == 42);

    std::ostringstream out, err;
    const char *help_argv[] = {"bike-recording-digest", "--help"};
    assert(bike::cli::run_recording_digest(2, help_argv, out, err) == 0);
    return 0;
}
```

In `native/tests/recording/CMakeLists.txt` right after `recording_catalog` (`:14`):

```cmake
add_executable(recording_digest test_recording_digest.cpp)
bike_startup_target_policy(recording_digest "${_test_repo_root}/native")
target_link_libraries(recording_digest PRIVATE
  bike_test_fixtures bike_recording_reader bike_recording_digest_cli)
if(BUILD_TESTING)
  add_test(NAME recording_digest COMMAND recording_digest)
  set_tests_properties(recording_digest PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:recording_digest>")
endif()
```

- [ ] **Step 3: Run `recording_digest` target (`RED`)**

Run: `cmake --build build/native-release --target recording_digest`
Expected: FAIL — `recording/digest.hpp` and `bike_recording_digest_cli` not found.

- [ ] **Step 4: Implement `digest.{hpp,cpp}`, `recording_digest.{hpp,cpp}`, `recording_digest_main.cpp`, and `RecordingApplication.cmake`**

```cpp
// native/src/recording/digest.hpp
#pragma once
#include "recording/reader.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace bike::recording {
struct ColumnDigest {
    std::string name;
    std::uint64_t rows{};
    std::string sha256;
    double min_val{};
    double max_val{};
    double l2_norm{};
};
struct TableDigest {
    std::string table;
    std::uint64_t row_count{};
    std::vector<ColumnDigest> columns;
};
struct DigestReport {
    std::int64_t schema_version = 1;
    std::vector<TableDigest> tables;
    std::string overall_sha256;
};
struct DigestDiff {
    bool equal = true;
    std::string table;
    std::string column;
    std::int64_t interval_id = -1;
    std::string detail;
};
[[nodiscard]] DigestReport compute_recording_digest(const Recording &rec);
[[nodiscard]] DigestDiff compare_recording_digests(const DigestReport &lhs, const DigestReport &rhs);
[[nodiscard]] io::Value digest_to_json(const DigestReport &report);
[[nodiscard]] DigestReport digest_from_json(const io::Value &json);
} // namespace bike::recording
```

In `digest.cpp`, stream `"physics"` (`ORDER BY interval_id`) and `"forces"` (`ORDER BY interval_id, side, component`), hashing every scalar's exact LE IEEE-754 bits (`std::bit_cast<std::uint64_t>(val)`) alongside per-column `min_val`/`max_val`/`l2_norm` and per-interval chunk hashes (`interval_id -> row_hash`) so `compare_recording_digests` locates the exact first differing `(table, column, interval_id)`.
In `native/cmake/RecordingApplication.cmake` (after `:212`):

```cmake
add_library(bike_recording_digest_cli STATIC
  "${native_dir}/src/recording/digest.cpp"
  "${native_dir}/src/cli/recording_digest.cpp")
bike_startup_target_policy(bike_recording_digest_cli "${native_dir}")
target_link_libraries(bike_recording_digest_cli PUBLIC bike_recording_reader bike_io)

add_executable(bike-recording-digest
  "${native_dir}/src/cli/recording_digest_main.cpp")
bike_startup_target_policy(bike-recording-digest "${native_dir}")
target_link_libraries(bike-recording-digest PRIVATE bike_recording_digest_cli)
set_target_properties(bike-recording-digest PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
```

- [ ] **Step 5: Run `recording_digest` & `reference.native_platform_linkage` (`GREEN`)**

Run: `mise run test '^(recording_catalog|recording_digest|reference\.native_platform_linkage)$'`
Expected: PASS (`3/3` green; `otool -L build/native-release/bin/bike-recording-digest` has no `libmujoco`).

- [ ] **Step 6: Commit Task 1**

```bash
git add docs/agents/simulation-decisions.md native/src/recording/digest.{hpp,cpp} \
  native/src/cli/recording_digest.{hpp,cpp} native/src/cli/recording_digest_main.cpp \
  native/cmake/RecordingApplication.cmake native/tests/recording/CMakeLists.txt \
  native/tests/recording/test_recording_digest.cpp
git commit -m "feat(sp1): add engine-free Schema v1 bike-recording-digest and update policy"
```

---

### Task 2: Freeze 4 × 30 s Golden Episode Digests + Stage 1 Gate

**Files:**
- Create: `tests/fixtures/episode-digest/darwin-arm64/{welded_flat,welded_rough_uphill_extreme,distributed_flat,distributed_rough_uphill_extreme}.json`
- Create: `native/tests/recording/test_golden_episode_digest.cpp`
- Modify: `CMakeLists.txt` (register `golden_episode_digest`, `LABELS "recording;golden;slow"`)

**Interfaces:**
- Consumes: `bin/bike-ride`, `bin/bike-recording-digest`, `bike::recording::{compute_recording_digest, compare_recording_digests, digest_from_json}`.
- Produces: Frozen baseline 24,000-step digest JSONs + CTest `golden_episode_digest` (`[--episode <all|welded|distributed>]`).

- [ ] **Step 1: Generate the 4 baseline 30 s DuckDB episodes via `bike-ride` and digest them**

```bash
mkdir -p /tmp/sp1_golden_runs tests/fixtures/episode-digest/darwin-arm64
for pair in \
  "welded_flat:flat:examples/research/viewer_physics_welded.toml" \
  "welded_rough_uphill_extreme:examples/research/rough_uphill_extreme.toml:examples/research/viewer_physics_welded.toml" \
  "distributed_flat:flat:examples/research/viewer_physics_distributed.toml" \
  "distributed_rough_uphill_extreme:examples/research/rough_uphill_extreme.toml:examples/research/viewer_physics_distributed.toml"
do
  IFS=':' read -r name track phys <<< "$pair"
  rm -rf "/tmp/sp1_golden_runs/$name"
  mkdir -p "/tmp/sp1_golden_runs/$name/cache" "/tmp/sp1_golden_runs/$name/out"
  BIKE_SIM_EQUILIBRIUM_CACHE_DIR="/tmp/sp1_golden_runs/$name/cache" \
    ./build/native-release/bin/bike-ride --physics-config "$phys" --track "$track" \
    --headless --duration 30 --check-policy diagnostic --record \
    --out "/tmp/sp1_golden_runs/$name/out" --no-plots || [ "$?" -eq 2 ]
  rec_dir=$(dirname "$(ls "/tmp/sp1_golden_runs/$name/out/"*/manifest.json)")
  ./build/native-release/bin/bike-recording-digest \
    --run "$rec_dir/run.duckdb" \
    --out "tests/fixtures/episode-digest/darwin-arm64/${name}.json"
done
rm -rf /tmp/sp1_golden_runs
```

- [ ] **Step 2: Implement `test_golden_episode_digest.cpp` & register in `CMakeLists.txt`**

```cpp
// native/tests/recording/test_golden_episode_digest.cpp
int main(int argc, char **argv) {
    // argv[1] = path to bike-ride binary; optional --episode welded|distributed|all
    for (const auto &ep : selected_episodes(filter)) {
        run_headless_bike_ride(bike_ride_bin, ep.track_toml, ep.physics_toml, tmp_dir);
        const auto live = bike::recording::compute_recording_digest(*bike::recording::open_recording(tmp_dir / "run.duckdb"));
        const auto golden = bike::recording::digest_from_json(bike::io::parse_json_file(ep.golden_json));
        const auto diff = bike::recording::compare_recording_digests(golden, live);
        if (!diff.equal) {
            throw std::runtime_error(ep.name + " drift at " + diff.table + "." + diff.column +
                                     " interval=" + std::to_string(diff.interval_id) + ": " + diff.detail);
        }
    }
    return 0;
}
```

Register `golden_episode_digest` in `CMakeLists.txt` with `LABELS "recording;golden;slow"` (outside `native/tests/recording/` loop so it uses `bike_test_fixtures` + `bike_recording_digest_cli` + `bike_platform` without `libmujoco`).

- [ ] **Step 3: Run targeted `golden_episode_digest` & Stage-1 `check_change.sh` (`GREEN`)**

Run:
1. `mise run test '^golden_episode_digest$'` -> PASS (`4/4` episodes match `0` bit diffs).
2. Commit Task 2 and run Stage-1 `check_change.sh` against the commit before Task 1:

```bash
git add tests/fixtures/episode-digest/darwin-arm64/*.json \
  native/tests/recording/test_golden_episode_digest.cpp CMakeLists.txt
git commit -m "test(sp1): freeze 30s Schema v1 golden episode digests and slow gate"
bash tools/check_change.sh HEAD~2
```
Expected: exits `0` (`<= 5 min`).

---

### Task 3: `physics_core` Foundations (`common`, `gpu_math`, `gpu_toolkit`, `tyre_types`, `exact_math`, `cpu_exact_toolkit`)

**Files:**
- Create: `native/src/physics_core/{common,gpu_math,gpu_toolkit,tyre_types,exact_math,cpu_exact_toolkit}.hpp`
- Create: `native/tests/physics_core/{CMakeLists.txt,test_physics_core_math.cpp}`
- Modify: `CMakeLists.txt` (`add_subdirectory(native/tests/physics_core)`)

**Interfaces:**
- Consumes: `numeric/{numeric_numpy,numeric_norm,cblas_abi}.hpp` (host-only headers ONLY).
- Produces (namespace `bike::physics_core`):
  - `enum class Status : std::uint8_t { ok = 0, invalid_argument, overflow, out_of_bounds, logic_error };`
  - `enum class ErrCode : std::uint16_t { none = 0, nonfinite_center, invalid_radius, invalid_prev_seg, outside_domain, solid_road, degenerate_contact, nonunit_normal, logic_no_closest, overflow_nonfinite, overflow_passivity, missing_material, bad_dt, bad_time, out_of_bounds_chain, out_of_bounds_station, nonfinite_tangent, invalid_station_quad };`
  - `template <class Real> BIKE_HD constexpr Real Lit(double v) noexcept;`
  - `kMaxNq = 32`, `kMaxNv = 32`, `kMaxNu = 16`, `kMaxNbody = 32`, `kMaxNjnt = 32`, `kMaxChainJoints = 8`, `kMaxGpuTyreStations = 128`, `kMaxHingeKnots = 8`.
  - `struct CpuExactToolkit` & `struct GpuToolkit`: `div`, `sqrt`, `abs`, `isfinite`, `isnan`, `exp`, `tanh`, `pow_py`, `dot2`, `dot3`, `dot`, `gemv_n3`, `gemv_t3`, `ring_lift`, `interp_segment`, `hypot2`, `max`, `min`, `clamp`.
  - Templated tri-target POD views/states in `tyre_types.hpp`: `GpuChainJoint`, `GpuWheelChain`, `PolylineView<Real>`, `SignedProfileView<Real>`, `StationRingView<Real>`, `HingeDensityView<Real>`, `CompliantMaterialView<Real>`, `SurfaceSectionView<Real>`, `SurfaceMapView<Real>`, `TyreWheelSpec<Real>`, `TyreModel<Real>`, `CompliantWheelStatePod<Real>`, `StationStatePod<Real>` (with `using GpuTyreModel = TyreModel<float>;` etc.).

- [ ] **Step 1: Write failing `test_physics_core_math.cpp` covering CPU, Host-FP32, & Apple Metal parity**

```cpp
// native/tests/physics_core/test_physics_core_math.cpp (core assertions)
#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/gpu_math.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/tyre_types.hpp"

void test_portable_math_and_metal_parity() {
    for (int i = -2000; i <= 2000; ++i) {
        const float x = static_cast<float>(i) * 0.01f;
        const float g_tanh = bike::physics_core::GpuToolkit::tanh(x);
        const double c_tanh = bike::physics_core::CpuExactToolkit::tanh(static_cast<double>(x));
        require(std::abs(static_cast<double>(g_tanh) - c_tanh) <= 3e-7, "tanh ULP bound");
    }
    // Compile MSL snippet on MTLCreateSystemDefaultDevice() with #pragma METAL fp contract(off) +
    // MTLMathModeSafe + MTLMathFloatingPointFunctionsPrecise, run 1024 inputs, assert 0-bit match with Host GpuToolkit.
}
```

- [ ] **Step 2: Run `test_physics_core_math` (`RED`)**

Run: `cmake --build build/native-release --target test_physics_core_math`
Expected: FAIL — `physics_core/` headers do not exist yet.

- [ ] **Step 3: Implement `common.hpp`, `gpu_math.hpp`, `gpu_toolkit.hpp`, `tyre_types.hpp`, `exact_math.hpp`, `cpu_exact_toolkit.hpp`**

```cpp
// native/src/physics_core/gpu_math.hpp (key portable FP32 primitives)
BIKE_HD float expf_portable(float x) noexcept {
    // Cephes range reduction x = n * ln2 + r using HI/LO Cody-Waite split (strict + - * floor ldexp)
    if (x > 88.3762626647949f) return 3.402823466e+38f * 2.0f;
    if (x < -103.97207708399179f) return 0.0f;
    const float z = gpu_floor(1.44269504088896341f * x + 0.5f);
    const int n = static_cast<int>(z);
    x -= z * 0.693359375f;
    x -= z * (-2.12194440e-4f);
    const float xx = x * x;
    float px = 1.9875691500e-4f;
    px = px * x + 1.3981999507e-3f;
    px = px * x + 8.3334519073e-3f;
    px = px * x + 4.1665795894e-2f;
    px = px * x + 1.6666665459e-1f;
    px = px * x + 5.0000001201e-1f;
    const float ex = 1.0f + x + px * xx;
    return gpu_ldexp(ex, n);
}

BIKE_HD float tanhf_portable(float x) noexcept {
    const float ax = gpu_abs(x);
    const float sign = (x < 0.0f) ? -1.0f : 1.0f;
    if (ax >= 10.0f) return sign;
    if (ax < 1e-4f) return x * (1.0f - x * x * (1.0f / 3.0f));
    const float e2x = expf_portable(2.0f * ax);
    return sign * ((e2x - 1.0f) / (e2x + 1.0f));
}

BIKE_HD float interp_segment_unfused(float s, float x, float x0, float y0, float x1, float y1) noexcept {
    if (y0 == y1) return y0;
    float res = y0 + s * (x - x0);
    if (gpu_isnan(res)) res = y1 + s * (x - x1);
    return res;
}
```

Protect tri-target raw pointers/C-arrays in `common.hpp`, `gpu_math.hpp`, `gpu_toolkit.hpp`, `tyre_types.hpp` with `BIKE_GPU_SHARED_CODE_BEGIN`/`_END` + `NOLINTBEGIN` matching `gpu/common.hpp:52`, ensure standalone `#include` cleanliness for `check_headers`, and keep `exact_math.hpp` & `cpu_exact_toolkit.hpp` host-only (`#if defined(__METAL_VERSION__) || defined(__CUDA_ARCH__) #error`).

- [ ] **Step 4: Run `test_physics_core_math` (`GREEN`)**

Run: `mise run test '^test_physics_core_math$'`
Expected: PASS (`0` ULP diffs Host `GpuToolkit` vs Apple Metal device, `<= 2 ULP` vs `CpuExactToolkit`).

- [ ] **Step 5: Commit Task 3**

```bash
git add native/src/physics_core/{common,gpu_math,gpu_toolkit,tyre_types,exact_math,cpu_exact_toolkit}.hpp \
  native/tests/physics_core/{CMakeLists.txt,test_physics_core_math.cpp} CMakeLists.txt
git commit -m "feat(sp1): add tri-target physics_core math, toolkits, and POD views"
```

---

### Task 4: Two-Commit `O(1)` Polyline Contact Scanner & `SignedProfile` Extraction

**Files:**
- Create: `native/src/physics_core/terrain_scanner.hpp`, `native/src/physics_core/terrain_distance.hpp`
- Modify: `native/src/tyre/profile.hpp:43-119`, `native/src/tyre/profile.cpp:355-656`
- Modify: `native/src/tyre/distance.hpp`, `native/src/tyre/distance.cpp:150-259`
- Create: `native/tests/physics_core/test_terrain_scanner.cpp`

**Interfaces:**
- Consumes: `Status`, `ErrCode`, `PolylineView<Real>`, `SignedProfileView<Real>` (`tyre_types.hpp`), `CpuExactToolkit`, `GpuToolkit`.
- Produces (namespace `bike::physics_core`):
  - `struct NoMultiSupport;` and `struct O1CpuMultiSupport;`
  - `template <class Real> struct PolylineContactOut { Real point[2]; Real normal[2]; Real delta; int segment_id; bool multi_support; };`
  - `template <class Real, class TK, class MultiSupportPolicy = NoMultiSupport> BIKE_HD Status scan_polyline_contact(const PolylineView<Real> &pl, Real cx, Real cz, Real radius, bool has_prev_seg, int prev_seg, PolylineContactOut<Real> &out, ErrCode &err) noexcept;`
  - `template <class Real, class TK> BIKE_HD Status profile_height(const SignedProfileView<Real> &sp, Real x, Real &out_h, ErrCode &err) noexcept;`
  - `template <class Real, class TK> BIKE_HD Status signed_profile_query(const SignedProfileView<Real> &sp, Real px, Real pz, Real &out_dist, Real out_normal[2], Real out_pt[2], int &out_seg, bool &out_nonsmooth, ErrCode &err) noexcept;`

- [ ] **Step 1: Write failing `test_terrain_scanner.cpp` (`>= 20,000` queries + RTSan `BIKE_NONBLOCKING`)**

Test `scan_polyline_contact<double, CpuExactToolkit, O1CpuMultiSupport>` against frozen reference outputs of `ProfileQuery::contact` across actual `flat.toml` and `rough_uphill_extreme.toml` polylines plus `>= 20,000` adversarial queries (airborne expansion `min_dist >= radius`, exact/near `1e-12` ties with `previous_segment`, unkept `endpoint_keep == false` interior vertices winning `argmin`, exact endpoints, steep corners, all invalid-argument throws), asserting **0-bit `std::memcmp` parity** and zero heap allocations.

- [ ] **Step 2: Run `test_terrain_scanner` (`RED`)**

Run: `cmake --build build/native-release --target test_terrain_scanner`
Expected: FAIL — `terrain_scanner.hpp` does not exist yet.

- [ ] **Step 3: Commit 4a — Verbatim Extract of `SignedProfile` & `ProfileQuery` Mechanics**

Implement `terrain_distance.hpp` (`profile_height` + `signed_profile_query` preserving `< 1e-12` floor, `std::logic_error` via `ErrCode::logic_no_closest` -> `Status::logic_error`, `nonsmooth` check) and initial `terrain_scanner.hpp`. Delegate `biketyre::SignedProfile` and `biketyre::ProfileQuery::contact` to `<double, CpuExactToolkit>`, run `native_distance_tests`, `reference.tyre_road_profile`, and `test_terrain_scanner`, then commit Commit 4a:

```bash
git add native/src/physics_core/{terrain_scanner,terrain_distance}.hpp \
  native/src/tyre/{profile,distance}.{hpp,cpp} native/tests/physics_core/{CMakeLists.txt,test_terrain_scanner.cpp}
git commit -m "refactor(sp1): verbatim extract ProfileQuery and SignedProfile to physics_core"
```

- [ ] **Step 4: Commit 4b — Replace 5 Scratch Vectors with Stateless `O(1)` Scanner**

Update `terrain_scanner.hpp` with the verified stateless two-pass `O(1)`-register loop (matching our verified `o1_polyline_scanner.hpp`):

```cpp
// native/src/physics_core/terrain_scanner.hpp (5 load-bearing invariants)
WindowScanSummary<Real> sum = scan_window_o1<Real, TK>(pl, lo, hi, cx, cz, has_prev_seg, prev_seg);
int win_lo = lo, win_hi = hi;
if (!sum.nonempty || sum.min_dist >= radius) {
    if (sum.nonempty) {
        const Real v_lo = max_val(Lit<Real>(0), cz - pl.maximum_z);
        const Real reach = TK::sqrt(max_val(Lit<Real>(0), sum.min_dist * sum.min_dist - v_lo * v_lo)) + Lit<Real>(1e-8);
        win_lo = max_val(0, search_right(pl.px, pl.vertex_count, cx - reach) - 2);
        win_hi = min_val(count, search_right(pl.px, pl.vertex_count, cx + reach) + 1);
    } else {
        win_lo = 0; win_hi = count;
    }
    sum = scan_window_o1<Real, TK>(pl, win_lo, win_hi, cx, cz, has_prev_seg, prev_seg);
}
if (!sum.nonempty || sum.min_dist <= Lit<Real>(1e-12)) { err = ErrCode::degenerate_contact; return Status::invalid_argument; }
// Asymmetric endpoint_keep: applied ONLY to prev_seg tie-break (never argmin) and CPU multi_support pass!
if (sum.has_prev && sum.prev_keep && TK::abs(sum.prev_dist - winner_dist) <= Lit<Real>(1e-12)) { /* adopt prev */ }
const bool multi = MultiSupportPolicy::template evaluate<Real, TK>(pl, win_lo, win_hi, cx, cz, radius,
                                                                   winner_ptx, winner_ptz, delta, normal_w);
```

Delete `detail::Candidates`, `candidates`, `candidates_into`, `endpoint_keep_into`, `cand_scratch_`, `keep_scratch_`, `near_scratch_`, `normals_scratch_`, `dots_scratch_` from `native/src/tyre/profile.{hpp,cpp}` (`:43-99, :114-118`).

- [ ] **Step 5: Verify `test_terrain_scanner` & CPU parity (`GREEN`)**

Run: `mise run test '^(test_terrain_scanner|native_distance_tests|reference\.tyre_road_profile)$'`
Expected: PASS (`0` bit diffs across all `>= 20,000` + track queries; zero allocations).

- [ ] **Step 6: Commit 4b**

```bash
git add native/src/physics_core/terrain_scanner.hpp native/src/tyre/profile.{hpp,cpp}
git commit -m "perf(sp1): replace 24k scratch vectors in ProfileQuery::contact with O(1) scanner"
```

---

### Task 5: `nq = 27` Kinematics, Host Upload Gates, Nov 16 Audit Closures, & Stage 2 Gate

**Files:**
- Create: `native/src/physics_core/kinematics.hpp`, `native/src/physics_core/tyre_upload.hpp`, `native/src/physics_core/tyre_upload.cpp`
- Modify: `native/src/gpu/{common.hpp,batch_step.hpp,drivetrain_resistance_kernel.hpp,rider_kernel.hpp}`
- Modify: `native/src/gpu/{cuda_kernels.cu,cpu_batch_runner.cpp,metal_batch_runner.mm}`
- Modify: `tools/colab_gpu/build_cuda_writers.py:18-62, 349-350`, `tools/colab_gpu/test_model_assets.py:39-40`
- Modify: `native/tests/gpu/test_gpu_{model_geometry,rider,drivetrain_resistance,batch_step}.cpp`

**Interfaces:**
- Consumes: `GpuWheelChain`, `TyreModel<Real>`, `Status`, `ErrCode` (`tyre_types.hpp`, `common.hpp`, Task 3).
- Produces (namespace `bike::physics_core` & `bike::gpu`):
  - `template <class Real, class TK, bool ComputeJr> BIKE_HD Status chain_point_jacobian(const GpuWheelChain &chain, const Real *body_xpos, const Real *jnt_xanchor, const Real *jnt_xaxis, int wheel_body_id, int nv, const Real pt[3], Real *out_jp, Real *out_jr, ErrCode &err) noexcept;`
  - `GpuWheelChain build_wheel_chain(const mjModel *m, int wheel_body_id);`
  - `HostTyreUploadBundle build_gpu_tyre_model(const mjModel *m, const mjData *d, const nativecfg::TireConfig &cfg, int front_geom, int rear_geom);` (owning backing vectors for `TyreModel<double>` / `TyreModel<float>` views)
  - Updated `bike::gpu::GpuModelIndices` (`sizeof == 240`, `60` ints).

- [ ] **Step 1: Update GPU unit tests (`RED`) for `nq = 27`, `c_edge = 0`, upload rejections, & `long` stride**

1. Update `native/tests/gpu/test_gpu_model_geometry.cpp:89` from `tools/fp32_probe/artifacts/common/model.xml` (`nq = 26`) to `tests/fixtures/model/welded.xml` (`nq = 27`), asserting `sizeof(GpuModelIndices) == 240`, `drive_shaft_spin_qpos == 14`, `drive_shaft_spin_dof == 14`, `crank_clutch_tendon == 1`, `front_wheel` (`5`-DOF chain: `root_x, root_z, root_pitch, fork_travel, front_wheel_spin`), `rear_wheel` (`6`-DOF chain), and `chain_point_jacobian` exact match vs `mj_jac` (after calling `forward()` post `Stepper::set_state`, `stepper.cpp:594`).
2. Assert `build_gpu_tyre_model` rejects `TabulatedRadial` (`mat.table.has_value()`), non-zero `geom_pos`, non-identity `geom_quat`, `geom_bodyid != wheel_body`, `joint_id >= njnt`, and `station_count > 128`.
3. Update `test_gpu_rider.cpp:130, 198-209` (`rider_qpos`/`rider_dof` `{23,24,25,26,18,19,20,21,22}`, zero damping when `c_edge` zeroed), `test_gpu_drivetrain_resistance.cpp` (`drive_shaft_spin_qpos` freehub driver), and `test_gpu_batch_step.cpp` (`sizeof == 240`, `2 * idx.freehub_tendon + 1` stride).

- [ ] **Step 2: Run updated GPU tests (`RED`)**

Run: `mise run test '^bike_test_gpu_(model_geometry|rider|drivetrain_resistance|batch_step)$'`
Expected: FAIL — `sizeof(GpuModelIndices) == 228 != 240`, `c_edge != 0`, `kinematics.hpp` absent.

- [ ] **Step 3: Implement `kinematics.hpp`, `tyre_upload.{hpp,cpp}`, and Nov 16 Audit closures**

```cpp
// native/src/physics_core/kinematics.hpp (world-frame xanchor/xaxis ancestor Jacobian)
template <class Real, class TK, bool ComputeJr>
BIKE_HD Status chain_point_jacobian(
    const GpuWheelChain &chain, const Real *body_xpos, const Real *jnt_xanchor,
    const Real *jnt_xaxis, int wheel_body_id, int nv, const Real pt[3],
    Real *out_jp, Real *out_jr, ErrCode &err) noexcept {
    if (chain.joint_count > kMaxChainJoints || nv <= 0 || nv > kMaxNv) {
        err = ErrCode::out_of_bounds_chain; return Status::out_of_bounds;
    }
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < nv; ++c) {
            out_jp[r * nv + c] = Lit<Real>(0);
            if constexpr (ComputeJr) out_jr[r * nv + c] = Lit<Real>(0);
        }
    (void)body_xpos; (void)wheel_body_id;
    for (int k = 0; k < chain.joint_count; ++k) {
        const GpuChainJoint &j = chain.joints[k];
        if (j.joint_id < 0 || j.joint_id >= chain.njnt || j.dof_adr < 0 || j.dof_adr >= nv) {
            err = ErrCode::out_of_bounds_chain; return Status::out_of_bounds;
        }
        const int a3 = 3 * j.joint_id;
        const int d = j.dof_adr;
        if (j.joint_type == 3 /* mjJNT_HINGE */) {
            const Real ax[3] = {jnt_xaxis[a3], jnt_xaxis[a3 + 1], jnt_xaxis[a3 + 2]};
            const Real arm[3] = {pt[0] - jnt_xanchor[a3], pt[1] - jnt_xanchor[a3 + 1], pt[2] - jnt_xanchor[a3 + 2]};
            out_jp[0 * nv + d] = ax[1] * arm[2] - ax[2] * arm[1];
            out_jp[1 * nv + d] = ax[2] * arm[0] - ax[0] * arm[2];
            out_jp[2 * nv + d] = ax[0] * arm[1] - ax[1] * arm[0];
            if constexpr (ComputeJr) {
                out_jr[0 * nv + d] = ax[0]; out_jr[1 * nv + d] = ax[1]; out_jr[2 * nv + d] = ax[2];
            }
        } else if (j.joint_type == 2 /* mjJNT_SLIDE */) {
            out_jp[0 * nv + d] = jnt_xaxis[a3]; out_jp[1 * nv + d] = jnt_xaxis[a3 + 1]; out_jp[2 * nv + d] = jnt_xaxis[a3 + 2];
        } // mjJNT_FREE handled similarly for 6 DOFs if encountered
    }
    return Status::ok;
}
```

Apply all Nov 16 Audit fixes verbatim:
- **`GpuModelIndices` (`gpu/common.hpp:97-145` & `build_cuda_writers.py:18-61,349-350`)**: Add `drive_shaft_spin_qpos = 14`, `drive_shaft_spin_dof = 14`, `crank_clutch_tendon = 1`, shift `rider_{qpos,dof}` to `{23,24,25,26,18,19,20,21,22}`, `static_assert(sizeof(GpuModelIndices) == 240)`.
- **Freehub driver**: Use `const int fh_qpos = (idx.drive_shaft_spin_qpos >= 0) ? idx.drive_shaft_spin_qpos : idx.crank_spin_qpos;` in `drivetrain_resistance_kernel.hpp:117,131`, `metal_batch_runner.mm:220-228,612-620`, `cuda_kernels.cu:39-41`, and `cpu_batch_runner.cpp:376-384` (resolving `idx` dynamically from `mjModel` when `test_gpu_batch_runners.cpp:168` loads `nq = 26`).
- **`tendon_range_batch` indexing (`batch_step.hpp:216` & `metal_batch_runner.mm:301`)**: Use `static_cast<long>(w) * tendon_range_world_stride + 2 * idx.freehub_tendon + 1` with bounds gate `2 * idx.freehub_tendon + 1 < tendon_range_world_stride`.
- **`rider_kernel.hpp` `c_edge` zeroing**: Set `.c_edge_nm_s_rad = 0.0f` at `:31` AND across all 9 joints in `GpuRiderParams::joints[0..8]` (`:62-111`), and remove `:197` `torque -= c * ramp * qd_rad_s`.

- [ ] **Step 4: Run GPU tests & Stage-2 `check_change.sh` (`GREEN`)**

```bash
mise run test '^bike_test_gpu_(model_geometry|rider|drivetrain_resistance|batch_step)$'
git add native/src/physics_core/{kinematics.hpp,tyre_upload.hpp,tyre_upload.cpp} \
  native/src/gpu/{common.hpp,batch_step.hpp,drivetrain_resistance_kernel.hpp,rider_kernel.hpp,cuda_kernels.cu,cpu_batch_runner.cpp,metal_batch_runner.mm} \
  tools/colab_gpu/build_cuda_writers.py native/tests/gpu/test_gpu_{model_geometry,rider,drivetrain_resistance,batch_step}.cpp
git commit -m "feat(sp1): add nq=27 kinematics, tyre upload gates, and Nov 16 audit closures"
bash tools/check_change.sh HEAD~4
```
Expected: all 4 GPU tests pass and `check_change.sh` exits `0` (`<= 5 min`).

---

### Task 6: Shared `tyre_contact.hpp` & Slice 1 (`compliant_2d`) CPU/Shared Extraction

**Files:**
- Create: `native/src/physics_core/tyre_contact.hpp`, `native/src/physics_core/tyre_compliant.hpp`
- Modify: `native/src/contact/laws.hpp:40-98`
- Modify: `native/src/writers/tire_common.{hpp,cpp}`
- Modify: `native/src/writers/tire.{hpp,cpp}`
- Modify: `native/tests/physics_core/test_physics_core_math.cpp`

**Interfaces:**
- Consumes: `CpuExactToolkit`, `GpuToolkit` (Task 3), `scan_polyline_contact` (Task 4), `chain_point_jacobian<..., false>` (Task 5), `CompliantWheelStatePod<Real>`.
- Produces (namespace `bike::physics_core`):
  - `template <class Real, class TK> BIKE_HD Real surface_mu(const SurfaceSectionView<Real> &s, Real slip) noexcept;`
  - `template <class Real, class TK> BIKE_HD void effective_friction(Real tire_mu, const SurfaceMapView<Real> *map, Real contact_x, Real slip, Real &out_mu, int &out_section_idx) noexcept;`
  - `template <class Real, class TK> BIKE_HD Status normal_contact_hysteresis(Real delta, Real delta_dot, Real k, Real eta, Real eps, Real &out_force, Real &out_energy, ErrCode &err) noexcept;`
  - `template <class Real, class TK> BIKE_HD Status brush_step(Real xi, Real u, Real v_roll, Real fn, Real k, Real mu, Real length, Real dt, Real &out_xi, Real &out_force, Real &out_loss, ErrCode &err) noexcept;`
  - `struct NoOpIo { using MultiSupportPolicy = NoMultiSupport; /* inline no-op record hooks */ };`
  - `template <class Real, class TK, class Io = NoOpIo> BIKE_HD Status step_compliant_wheel(const TyreModel<Real> &model, int side, Real dt, const Real *qvel, const Real *body_xpos, const Real *jnt_xanchor, const Real *jnt_xaxis, CompliantWheelStatePod<Real> &state, Real *out_qfrc, Io &io, ErrCode &err) noexcept;`

- [ ] **Step 1: Add `test_compliant_step_fp32_vs_fp64` to `test_physics_core_math.cpp` (`RED`)**

Drive `step_compliant_wheel<double, CpuExactToolkit, NoOpIo>` and `step_compliant_wheel<float, GpuToolkit, NoOpIo>` across `20` loaded + airborne + cornering ticks (`welded.xml`), asserting `0` nonfinite outputs and `<= 5e-5` relative normal/tangent force agreement.

- [ ] **Step 2: Implement `tyre_contact.hpp` & `tyre_compliant.hpp`**

```cpp
// native/src/physics_core/tyre_compliant.hpp (3-gate continuity + airborne segment persistence)
const bool same_branch = state.has_center && state.has_segment &&
    ((seg_abs_diff <= 1) || coincident_point || continuous_with_center);
Real release_loss = Lit<Real>(0);
Real old_xi = Lit<Real>(0);
if (same_branch && state.has_tangent) {
    transport_bristle<Real, TK>(old_tangent, tangent, state.xi, xi_transp);
    old_xi = state.xi;
    release_loss = Lit<Real>(0.5) * tk * (state.xi * state.xi - xi_transp * xi_transp);
} else if (state.has_tangent) {
    release_loss = Lit<Real>(0.5) * tk * (state.xi * state.xi); // airborne / branch break release
}
// Even when delta <= 0, preserve contact.segment_id (tire.cpp:843) for tie-breaking!
state.segment = contact.segment_id;
state.has_segment = true;
```

- [ ] **Step 3: Wire `contact/laws.hpp`, `writers/tire_common.cpp`, and `writers/tire.cpp` through `<double, CpuExactToolkit>`**

Keep `TireWriter`'s atomic dual-wheel staging, `radial_tables_` fallback when `mat.table.has_value()`, `pyfloat::pow(xi_new, 2.0)` inside `CompliantCpuIo` (`tire.cpp:904, 909`), exact exception order (`Status::logic_error` -> `std::logic_error` at `tire.cpp:667-669`), and zero bare MuJoCo C-API call tokens in comments.

- [ ] **Step 4: Run `compliant_2d` parity & `welded` golden digest (`GREEN`)**

Run: `mise run test '^(test_physics_core_math|reference\.native_tire|reference\.native_pneumatic_tire)$'`
And run: `./build/native-release/native/tests/recording/golden_episode_digest ./build/native-release/bin/bike-ride --episode welded`
Expected: PASS (`0` bit diffs across all `24,000` steps of `welded_flat` & `welded_rough_uphill_extreme`).

- [ ] **Step 5: Commit Task 6**

```bash
git add native/src/physics_core/{tyre_contact,tyre_compliant}.hpp \
  native/src/contact/laws.hpp native/src/writers/tire_common.{hpp,cpp} \
  native/src/writers/tire.{hpp,cpp} native/tests/physics_core/test_physics_core_math.cpp
git commit -m "feat(sp1): extract compliant_2d tyre physics into physics_core with 0-bit CPU parity"
```

---

### Task 7: Slice 2 (`distributed_2d_reference`) Extraction + 4-Episode 0-Bit Proof

**Files:**
- Create: `native/src/physics_core/tyre_distributed.hpp`
- Modify: `native/src/tyre/station.{hpp,cpp}`
- Modify: `native/src/writers/distributed_tire.{hpp,cpp}`
- Modify: `native/tests/physics_core/test_physics_core_math.cpp`

**Interfaces:**
- Consumes: `CpuExactToolkit`, `GpuToolkit`, `profile_height`, `signed_profile_query` (`terrain_distance.hpp`), `chain_point_jacobian<..., true>` (`kinematics.hpp`), `tyre_contact.hpp`, `StationRingView<Real>`, `StationStatePod<Real>`.
- Produces (namespace `bike::physics_core`):
  - `template <class Real, class TK> BIKE_HD Status station_density(Real delta, const HingeDensityView<Real> &density, Real &out_force, Real &out_energy, ErrCode &err) noexcept;`
  - `template <class Real, class TK> BIKE_HD Status station_response(Real delta, Real rate, Real weight, const HingeDensityView<Real> &density, Real &out_force, Real &out_energy, ErrCode &err) noexcept;`
  - `template <class Real, class TK, class Io = NoOpIo> BIKE_HD Status step_distributed_wheel(const TyreModel<Real> &model, int side, Real dt, const Real *qvel, const Real *body_xpos, const Real *body_xmat, const Real *jnt_xanchor, const Real *jnt_xaxis, StationStatePod<Real> *states, Real *out_qfrc, Io &io, ErrCode &err) noexcept;`

- [ ] **Step 1: Add `test_distributed_step_fp32_vs_fp64` to `test_physics_core_math.cpp` (`RED`)**

Step all `128` tread stations over `20` ticks (`distributed.toml`), verifying matching active station bitmasks and `<= 5e-5` normal/shear load parity between `<double, CpuExactToolkit>` and `<float, GpuToolkit>`.

- [ ] **Step 2: Implement `tyre_distributed.hpp` & delegate `station.cpp` / `distributed_tire.cpp`**

Follow exact `distributed_tire.cpp:592-854` sequence:
1. `chain_point_jacobian<Real, TK, true>` once at `center` (`jp_center`, `jr`).
2. `TK::ring_lift` -> full-ring `profile_height` domain sweep FIRST (`distributed_tire.cpp:459-460`) before testing `z < height`.
3. Per active station: `signed_profile_query`, `vel = jp_center @ qvel + cross(jr @ qvel, arm)`, plain fold `rate = -((v0*n0 + v1*n1) + v2*n2)` (`:690-693`), `station_response`, unconditional `transport_bristle`, `brush_step`, `jac_station^T @ f_world` (`:673-681`).
4. Detached stations reset `{active = false, xi = 0}`; `DistributedCpuIo` receives `0.5 * k_s * pyfloat::pow(old.xi, 2.0)` (`:507, 738, 780`), `numpy_sum`, `PythonSum` `effective_radius_m`, `distinct`/`separate` checks, preserving exact validation order (`:592-854`).

- [ ] **Step 3: Run distributed tests & all 4 `golden_episode_digest` episodes (`GREEN`)**

Run: `mise run test '^(test_physics_core_math|native_station_tests|reference\.distributed_tire_reference|reference\.native_distributed_tire|reference\.native_distributed_stands|golden_episode_digest)$'`
Expected: PASS (`0` bit diffs across all 4 × 30 s golden episodes and all distributed reference tests).

- [ ] **Step 4: Commit Task 7**

```bash
git add native/src/physics_core/tyre_distributed.hpp native/src/tyre/station.{hpp,cpp} \
  native/src/writers/distributed_tire.{hpp,cpp} native/tests/physics_core/test_physics_core_math.cpp
git commit -m "feat(sp1): extract distributed_2d_reference into physics_core with 4-episode 0-bit parity"
```

---

### Task 8: Simultaneous Metal + CUDA + CPU Runner Wiring, `TESTING.md` Sync, & Legacy Removal

**Files:**
- Delete: `native/src/gpu/tire_kernel.hpp`
- Modify: `native/src/gpu/{cuda_kernels_abi.hpp,cuda_kernels.cu,batch_step.hpp,metal_batch_runner.mm,cpu_batch_runner.cpp}`
- Modify: `tools/colab_gpu/{build_cuda_writers,mjwarp_batch_sweep,mjwarp_metal_sweep,metal_hybrid_sweep,cpu_batch_sweep}.py`
- Modify: `docs/TESTING.md:1054-1060`
- Modify: `native/tests/gpu/{test_gpu_tire,test_gpu_batch_runners}.cpp`

**Interfaces:**
- Consumes: Ordered MSL/CUDA device headers (`common`, `gpu_math`, `gpu_toolkit`, `tyre_types`, `kinematics`, `terrain_scanner`, `terrain_distance`, `tyre_contact`, `tyre_compliant`, `tyre_distributed`), `tyre_upload` (Task 5), `step_compliant_wheel` (Task 6), `step_distributed_wheel` (Task 7).
- Produces: Unified `bike_gpu_writers_kernel` / `bike_wp_writers_metal_kernel` / `bike_metal_step_writers_kernel` / `bike_metal_benchmark_writers_only` supporting `compliant_2d` & `distributed_2d_reference` (`xmat`, `xanchor`, `xaxis`, `d_tyre_model`), with `num_lateral_ribs` and all `simd32` kernels deleted.

- [ ] **Step 1: Rewrite `test_gpu_tire.cpp` & update `test_gpu_batch_runners.cpp` (`RED`)**

Rewrite `test_gpu_tire.cpp` to assert FP32 vs FP64 parity for `step_compliant_wheel` + `step_distributed_wheel` (`welded.xml` `nq = 27`, `rough_uphill_extreme`), and update `test_gpu_batch_runners.cpp` to pass `(xmat, xanchor, xaxis, TyreModel<float>)`, test BOTH `compliant_2d` and `distributed_2d_reference` on CPU & `metal:0` (`<= 2e-5` 1-step), and assert nonzero error returns for invalid bounds (`nworld <= 0`, `idx.nq != m->nq`, `joint_id >= njnt`, `station_count > 128`).

- [ ] **Step 2: Delete `gpu/tire_kernel.hpp`, strip `num_lateral_ribs` & `simd32`, and wire Metal + CUDA + CPU**

1. Update `build_msl_shader_source()` (`metal_batch_runner.mm:101-120`) to prepend `#pragma METAL fp contract(off)\n` and stitch ONLY the ordered `../physics_core/` device headers (`common.hpp`, `gpu_math.hpp`, `gpu_toolkit.hpp`, `tyre_types.hpp`, `kinematics.hpp`, `terrain_scanner.hpp`, `terrain_distance.hpp`, `tyre_contact.hpp`, `tyre_compliant.hpp`, `tyre_distributed.hpp`), and set `MTLMathModeSafe` + `MTLMathFloatingPointFunctionsPrecise` in `make_compile_options()` (`:879`).
2. Update `tools/colab_gpu/build_cuda_writers.py:174` (`--use_fast_math` -> `--fmad=false`, add `-I<native/src>`), `metal_hybrid_sweep.py:96,100` (`-ffast-math` -> `-ffp-contract=off`, add `-I<native/src>`), and `cpu_batch_sweep.py:105,109,122,127` (`-ffast-math` -> `-ffp-contract=off`, add `-I<native/src>`).
3. Enforce `clean_split` (`fwd_velocity` before `constraint.make_constraint`, ending in `implicit`) in `mjwarp_metal_sweep.py` and `mjwarp_batch_sweep.py`, delete `num_lateral_ribs` & all `simd32` kernels (`use_simd32`), update `bike_metal_benchmark_writers_only` (`metal_batch_runner.mm:1280`, `metal_hybrid_sweep.py:181`), and pass `xmat`, `xanchor`, `xaxis`, and `TyreModel<float>` through all runner ABIs with explicit nonzero status on bounds violation.

- [ ] **Step 3: Sync `docs/TESTING.md:1054-1060` & verify GPU CTests (`GREEN`)**

Update `docs/TESTING.md:1054-1056` to replace `tire_kernel.hpp` with the 10 tri-target `physics_core` device headers and `sizeof(GpuModelIndices) == 240`.

Run: `mise run test '^bike_test_gpu_(tire|batch_step|batch_runners|model_geometry|rider|drivetrain_resistance)$'`
Expected: PASS (`6/6` tests green).

- [ ] **Step 4: Commit Task 8**

```bash
git rm native/src/gpu/tire_kernel.hpp
git add native/src/gpu/ native/tests/gpu/ tools/colab_gpu/ docs/TESTING.md
git commit -m "feat(sp1): wire physics_core across Metal, CUDA, and CPU runners and remove legacy kernels"
```

---

### Task 9: Final Merge Verification (`native` + `full` + Optional CUDA Smoke)

**Files:**
- Verify only (no edits unless fixing a gate failure)

**Interfaces:**
- Consumes: All Task 1–8 commits on `feat/gpu-shared-tyre-core`.
- Produces: Recorded `native` + `full` green gate rows in `.git/bike-gate/timings.tsv`.

- [ ] **Step 1: Run Level-2 `native` gate (compile + all static-analysis sweeps + ordinary CTest)**

Run: `bash tools/run_tests.sh native`
Expected: exit `0` (`~11–12 min`; `configure`, `build`, `ctest`, `scripts`, `cppcheck`, `context`, `diagnostic-controls`, `contracts`, `frontends`, `headers`, `odr`, `tidy` all green).

- [ ] **Step 2: Run Level-3 `full` gate (`native` + `slow` + `package` labels)**

Run: `bash tools/run_tests.sh full`
Expected: exit `0` (`~35 min`; includes `golden_episode_digest`, `reference.native_tire`, `reference.native_distributed_tire`, `reference.distributed_tire_reference`, `reference.native_distributed_stands`).

- [ ] **Step 3: Optional Colab CUDA `build_cuda_writers.py --fmad=false` + `clean_split` parity smoke**

If executing on Colab NVIDIA CUDA, compile `cuda_kernels.cu` via `tools/colab_gpu/build_cuda_writers.py` (`--fmad=false`), run 1-step & 80-step `mjwarp_batch_sweep.py` (`clean_split`), **immediately execute `colab stop`**, and verify `colab sessions` reports `0` active VMs.
