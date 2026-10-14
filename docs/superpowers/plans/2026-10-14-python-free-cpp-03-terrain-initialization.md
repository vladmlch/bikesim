# Randomness, terrain and Python-free initialization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T19 through T25 with source-backed parity and independently reviewed test cycles.

**Architecture:** Preserve the existing native execution path and replace only this subsystem's Python-owned construction, behavior or orchestration. Downstream consumers use the declared owned interfaces; Python reference code stays isolated until final removal.

**Tech Stack:** C++23, CMake >=3.24, pinned native MuJoCo 3.12.0/DuckDB 1.5.6, existing numeric kernels, native IO/GUI/test tools.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`. Also read `migration/interface-contracts.md`, `migration/api-index.md`, and the assigned source cards.

## Global Constraints

- Preserve all required first-party behavior; no Python fallback, embedded interpreter, subprocess interpreter or mandatory Python build/test tool in the final product.
- Preserve existing C++ kernels and the single authoritative physical stepping path; boundary extraction is not permission to change equations or operation order.
- Use C++23 and CMake >=3.24. Retain strict warnings, stack protection, numerical flags, hardening and native safety checks.
- Initially retain MuJoCo 3.12.0 with the verified private TLS error ABI and DuckDB 1.5.6; lock actual native dependency hashes before use.
- Maintain named schema/config/CLI/recording/replay contracts and every supported mode, or obtain explicit user approval for a documented compatibility break.
- Keep existing same-provider bitwise guarantees; no global epsilon, golden regeneration to hide failures, or silent cross-provider equivalence claims.
- Recording remains opt-in, RAM-retained during execution and final-only publication; no streaming redesign, silent history loss or automatic recording from --out.
- Every Python deletion requires a ledger owner, native replacement or approved retirement, assertion/capability coverage and passing evidence; final removal is T63.
- Changes to shared headers/CMake/registries go through one integration owner. Run full native gates serially per machine and record the exact build identity.
- Source examples and API snippets in this plan are proposed implementation contracts, not a claim that code was compiled or tests passed during planning.

---

The named execution sub-skills refer to the downstream agent's environment. When unavailable, follow the same task-by-task red/green/independent-review protocol; no capability is assumed in this handoff. All statuses start planned. The code examples are contracts/regressions to implement and validate, not patches already applied to the application.

## Package map

| Task | Deliverable | Dependencies |
|---|---|---|
| T19 | Reproduce seeded random streams and distribution transforms | T05, T13 |
| T20 | Port terrain primitives, profiles, surfaces and heightfields | T14, T19 |
| T21 | Port all track generators, presets and track-file round trips | T20, T13 |
| T22 | Port physical settling without a reference RideSimulation | T18, T20, T09 |
| T23 | Port bounded least squares, equilibrium refinement and sag fitting | T22, T15 |
| T24 | Replace Python equilibrium cache/state files and retain old readers | T22, T23, T06 |
| T25 | Construct standalone native rides from user configuration | T13, T16, T18, T21, T24 |

## Task T19: Reproduce seeded random streams and distribution transforms

**Dependencies:** T05, T13

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/random/numpy_compat.hpp` |
| Create | `native/src/random/numpy_compat.cpp` |
| Create | `native/src/random/seed_sequence.cpp` |
| Create | `native/src/random/distributions.cpp` |
| Create | `native/src/random/noise_tape.cpp` |
| Create | `native/tests/migration/test_t19.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/research/rider_random.py:1-51`
- `src/bike_sim/sim/research/sensor_noise.py:1-70`

Declaration-level card: `migration/cards/T19.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T05` / `migration/api-index.md#t05`: Create strict lossless data and JSON boundaries.
- `T13` / `migration/api-index.md#t13`: Port TOML/JSON loading, merge precedence and asset resolution.

**Produces:**

```cpp
namespace bike::random {
class NumpyRng {
public:
  explicit NumpyRng(std::uint64_t seed);
  NumpyRng(std::string_view nonnegative_decimal_entropy,
           std::span<const std::uint32_t> spawn_key);
  ~NumpyRng();
  NumpyRng(NumpyRng&&) noexcept;
  NumpyRng& operator=(NumpyRng&&) noexcept;
  NumpyRng(const NumpyRng&) = delete;
  NumpyRng& operator=(const NumpyRng&) = delete;
  std::uint64_t raw64();
  double uniform01();
  double normal01();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
// Additional SeedSequence entropy/spawn-key inputs are required for the source's
// spawned sensor streams; preserve arbitrary integer seed input at the IO boundary.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t19.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "random/numpy_compat.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    bike::random::NumpyRng a(1234), b(1234);
    for (int i=0; i<64; ++i)
      bt::require(a.raw64() == b.raw64(), "deterministic native stream");
    bike::random::NumpyRng n(1234);
    std::array<double,64> values{};
    for (auto& x : values) x = n.normal01();
    bt::golden_f64("random/seed1234-normal64.f64", values);
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T19
ctest --test-dir build/native-release -R '^migration_T19$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T19` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T19` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Inventory actual random calls in terrain generator/road/alpine/obstacles and research rider_random/sensor_noise. Freeze the exact locked NumPy version and draw shapes/order from the reference.
  - [ ] 2. Port or vendor the necessary native algorithm sources with preserved license: SeedSequence mixing/spawn behavior, the selected PCG64 generator, uniform integer/float conversion, normal transform, choice and needed vector-draw semantics. Do not substitute std::mt19937 or std::normal_distribution.
  - [ ] 3. Mirror separate noise/dropout child streams. Startup draws, sensor count formula, distribution batching and skipped/dropout draw consumption must all match.
  - [ ] 4. Keep replay assets containing old noise tapes sufficient to replay without regenerating random numbers. Native fresh generation still needs seed compatibility; storing old tapes is not a replacement for this task.
  - [ ] 5. If a controlled new RNG algorithm is ever introduced, use a distinct execution identity and explicit migration decision. It is not the default outcome of this plan.

### Mandatory additional regression cases

- [ ] Raw bitstream, spawned streams, uniforms, bounded integers, normals and choice against reference fixtures.
- [ ] Seeds zero, one, 1234, large seed values, empty draws and mixed draw sequences.
- [ ] Terrain and research results do not change with batch worker count.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T19$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/random/numpy_compat.hpp native/src/random/numpy_compat.cpp native/src/random/seed_sequence.cpp native/src/random/distributions.cpp native/src/random/noise_tape.cpp native/tests/migration/test_t19.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T19 reproduce seeded random streams and distribution transforms"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T20: Port terrain primitives, profiles, surfaces and heightfields

**Dependencies:** T14, T19

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/terrain/types.hpp` |
| Create | `native/src/terrain/profile.cpp` |
| Create | `native/src/terrain/obstacles.cpp` |
| Create | `native/src/terrain/surface.cpp` |
| Create | `native/src/terrain/heightfield.cpp` |
| Create | `native/src/terrain/contact_safety.cpp` |
| Create | `native/src/terrain/wheelpath.cpp` |
| Create | `native/src/terrain/profile.hpp` |
| Create | `native/tests/migration/test_t20.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/terrain/contact_profile.py:1-178`
- `src/bike_sim/terrain/contact_safety.py:1-96`
- `src/bike_sim/terrain/grade.py:1-55`
- `src/bike_sim/terrain/heightfield.py:1-285`
- `src/bike_sim/terrain/profile.py:1-159`
- `src/bike_sim/terrain/profile_distance.py:1-80`
- `src/bike_sim/terrain/road.py:1-384`
- `src/bike_sim/terrain/surface.py:1-266`
- `src/bike_sim/terrain/wheelpath.py:1-141`

Declaration-level card: `migration/cards/T20.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T14` / `migration/api-index.md#t14`: Port geometry and Horst-link kinematics.
- `T19` / `migration/api-index.md#t19`: Reproduce seeded random streams and distribution transforms.

**Produces:**

```cpp
namespace bike::terrain {
struct TrackSpec;
TrackSpec flat(double length_m);
std::vector<std::array<double,2>> profile(const TrackSpec& track, double resolution_m);
}
// Existing biketyre::SignedProfile and native tyre/distance.cpp are reused
// wherever they implement the same nearest-profile contract.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t20.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "terrain/profile.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    const auto track = bike::terrain::flat(10.0);
    const auto vertices = bike::terrain::profile(track, 0.1);
    bt::require(vertices.size() >= 2, "profile has an interval");
    for (const auto& p : vertices) bt::require(p[1] == 0.0, "flat elevation");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T20
ctest --test-dir build/native-release -R '^migration_T20$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T20` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T20` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port every obstacle class, grade/profile composition, surface registry/sections, contact safety and wheel-center path calculation. Preserve endpoint inclusion, obstacle overlap order, slopes, rounding and units.
  - [ ] 2. Port Gaussian smoothing used by obstacles with the original kernel construction, sigma/truncation and boundary mode. A moving-average replacement changes terrain and is not acceptable.
  - [ ] 3. Preserve heightfield layout/scaling and engine world-to-field mappings, contact query state and material resolution. Test terrain placement regression cases explicitly.
  - [ ] 4. Reuse native distance/station/profile algorithms already present; supply new generation/storage adapters instead of creating competing geometric kernels.
  - [ ] 5. Do not implement the separate spatial-3D or soft-scree design plans during this migration. Preserve the currently implemented geometry.

### Mandatory additional regression cases

- [ ] Flat/square edge/pothole/roots/washboard/drop/stepped climb and near-boundary queries.
- [ ] Surface-segment boundaries and out-of-range behavior.
- [ ] Strictly increasing profile coordinates; duplicate or nonfinite vertices reject.
- [ ] Compiled heightfield samples and wheel path match reference.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T20$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/terrain/types.hpp native/src/terrain/profile.cpp native/src/terrain/obstacles.cpp native/src/terrain/surface.cpp native/src/terrain/heightfield.cpp native/src/terrain/contact_safety.cpp native/src/terrain/wheelpath.cpp native/src/terrain/profile.hpp native/tests/migration/test_t20.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T20 port terrain primitives, profiles, surfaces and heightfields"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T21: Port all track generators, presets and track-file round trips

**Dependencies:** T20, T13

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/terrain/generator.cpp` |
| Create | `native/src/terrain/road.cpp` |
| Create | `native/src/terrain/alpine.cpp` |
| Create | `native/src/terrain/placement.cpp` |
| Create | `native/src/terrain/trackfile.hpp` |
| Create | `native/src/terrain/trackfile.cpp` |
| Create | `native/src/terrain/presets.cpp` |
| Create | `native/tests/migration/test_t21.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/terrain/__init__.py:1-140`
- `src/bike_sim/terrain/alpine.py:1-336`
- `src/bike_sim/terrain/generator.py:1-194`
- `src/bike_sim/terrain/obstacles.py:1-555`
- `src/bike_sim/terrain/placement.py:1-77`
- `src/bike_sim/terrain/presets.py:1-243`
- `src/bike_sim/terrain/research.py:1-46`
- `src/bike_sim/terrain/trackfile.py:1-527`

Declaration-level card: `migration/cards/T21.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T20` / `migration/api-index.md#t20`: Port terrain primitives, profiles, surfaces and heightfields.
- `T13` / `migration/api-index.md#t13`: Port TOML/JSON loading, merge precedence and asset resolution.

**Produces:**

```cpp
namespace bike::terrain {
TrackSpec load_track(const std::filesystem::path& file);
std::string dump_track(const TrackSpec& track);
TrackSpec generate(const bike::io::Value& specification, std::uint64_t seed);
std::vector<std::string> available_presets();
}
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t21.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "terrain/trackfile.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    const auto track = bike::terrain::load_track(bt::fixture("tracks/rough_uphill_savage.toml"));
    bt::golden_text("tracks/rough_uphill_savage.canonical.toml",
                    bike::terrain::dump_track(track));
    bt::invalid([] { bike::terrain::load_track(bt::fixture("tracks/unknown_obstacle.toml")); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T21
ctest --test-dir build/native-release -R '^migration_T21$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T21` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T21` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port terrain/generator.py, road.py, alpine.py, placement.py, presets.py and trackfile.py rather than supporting only the supplied hand-written tracks.
  - [ ] 2. Preserve seed, generation specification, unit conversion, key aliases, free interval selection, obstacle budgets, roughness carving and deterministic placement ordering.
  - [ ] 3. Use the native random compatibility layer. Compare both semantic TrackSpec output and sampled/compiled geometry for fixed seeds.
  - [ ] 4. Support CLI list/dump/preview paths without building a running simulation. Round-trip source track schemas and explicit surface maps.
  - [ ] 5. Validate malicious/unknown class names as data; never reintroduce Python import-based class resolution.

### Mandatory additional regression cases

- [ ] All preset names, alpine_scree/alpine_rocky, rough uphill variants and road generator presets.
- [ ] Fixed-seed equality, impossible placement budgets, bad ranges and negative lengths.
- [ ] All supplied example TOMLs load and round-trip without lost fields.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T21$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/terrain/generator.cpp native/src/terrain/road.cpp native/src/terrain/alpine.cpp native/src/terrain/placement.cpp native/src/terrain/trackfile.hpp native/src/terrain/trackfile.cpp native/src/terrain/presets.cpp native/tests/migration/test_t21.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T21 port all track generators, presets and track-file round trips"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T22: Port physical settling without a reference RideSimulation

**Dependencies:** T18, T20, T09

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/initialization/equilibrium.hpp` |
| Create | `native/src/initialization/equilibrium.cpp` |
| Create | `native/src/initialization/initial_pose.cpp` |
| Create | `native/src/initialization/settle_contacts.cpp` |
| Create | `native/src/initialization/setup_owner.hpp` |
| Create | `native/src/initialization/setup_owner.cpp` |
| Create | `native/tests/migration/test_t22.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/ride/physical_equilibrium.py:1-153`
- `src/bike_sim/sim/ride/physical_runtime.py:1-951`

Declaration-level card: `migration/cards/T22.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T18` / `migration/api-index.md#t18`: Resolve model names into typed native writer/runtime configuration.
- `T20` / `migration/api-index.md#t20`: Port terrain primitives, profiles, surfaces and heightfields.
- `T09` / `migration/api-index.md#t09`: Extract owned ride runtime and bootstrap into a pure C++ API.

**Produces:**

```cpp
namespace bike::initialization {
struct EquilibriumOptions { std::optional<std::int64_t> max_steps; double tolerance = 0.05; };
struct EquilibriumResult { std::int64_t steps; double residual_qacc; bool cache_hit; };
class SetupOwner; // complete PImpl interface in migration/interface-contracts.md
std::unique_ptr<SetupOwner> make_setup(const std::filesystem::path& request_toml);
EquilibriumResult settle(SetupOwner& setup, const EquilibriumOptions& options);
}
// SetupOwner owns a model/Stepper and reuses the same physical writer path.
// A NativeRideRuntime is constructed only after a completed t=0 bootstrap exists.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t22.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "initialization/setup_owner.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    bike::initialization::EquilibriumOptions options;
    options.tolerance = -1.0;
    auto setup = bike::initialization::make_setup(bt::fixture("requests/welded_flat.toml"));
    bt::invalid([&] { bike::initialization::settle(*setup, options); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T22
ctest --test-dir build/native-release -R '^migration_T22$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T22` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T22` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port initial road slope/interpolation, wheel clearance, crank/pedal phase, articulated rider placement and contact/controller reset from physical_equilibrium.py.
  - [ ] 2. Confine velocity zeroing/damping and equilibrium force application to initialization. Never introduce a running-step root stabilizer or silently change force coefficients to settle.
  - [ ] 3. Preserve max(40000, round(80.0/dt)) default budget, 0.02-second cycles, specified refinement timing, finite-state checks, top-five residual diagnostics and acceptance at residual <= tolerance.
  - [ ] 4. Expose initialization-only typed apply-forces/forward/settle-welds operations reusing existing C++ writers. Reset all clocks, work/energy baselines, contact memories and generation state consistently after successful initialization.
  - [ ] 5. Initially run with refinement disabled only in a dedicated test fixture that settles without it. Public startup is not complete until T23 refinement and T24 cache behavior are connected.
  - [ ] 6. Do not settle through a live NativeRideRuntime that requires an already initialized bootstrap. Use SetupOwner with the shared Stepper/writer context, apply initialization-only velocity/pose changes there, then extract the final typed bootstrap for T25.

### Mandatory additional regression cases

- [ ] Flat/slope starts, high and low timestep budget equivalence, initial brake demands and crank phase.
- [ ] Failure to converge is an error, never a success flag with a large residual.
- [ ] A running runtime cannot enter refinement or settling mode.
- [ ] No reference Python object is required for typed initialization.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T22$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/initialization/equilibrium.hpp native/src/initialization/equilibrium.cpp native/src/initialization/initial_pose.cpp native/src/initialization/settle_contacts.cpp native/src/initialization/setup_owner.hpp native/src/initialization/setup_owner.cpp native/tests/migration/test_t22.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T22 port physical settling without a reference ridesimulation"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T23: Port bounded least squares, equilibrium refinement and sag fitting

**Dependencies:** T22, T15

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/numerics/least_squares.hpp` |
| Create | `native/src/numerics/least_squares.cpp` |
| Create | `native/src/initialization/refine.cpp` |
| Create | `native/src/initialization/sag_fit.cpp` |
| Create | `native/tests/migration/test_t23.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/equilibrium.py:1-205`
- `src/bike_sim/sim/ride/equilibrium_refine.py:1-96`
- `src/bike_sim/sim/ride/sag_fit.py:1-101`

Declaration-level card: `migration/cards/T23.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T22` / `migration/api-index.md#t22`: Port physical settling without a reference RideSimulation.
- `T15` / `migration/api-index.md#t15`: Port mass properties, suspension setup and rider geometry.

**Produces:**

```cpp
namespace bike::numerics {
struct LeastSquaresResult { std::vector<double> x; double cost; int evaluations; bool converged; };
LeastSquaresResult bounded_least_squares(
  const std::function<std::vector<double>(std::span<const double>)>& residual,
  std::span<const double> initial, std::span<const double> lower,
  std::span<const double> upper, int max_evaluations);
}
// Solver-specific tolerances/scaling/finite-difference modes live in an explicit
// options overload; initialize them from the exact calls in the Python sources.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t23.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "numerics/least_squares.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    const std::array<double,1> x{0.}, lo{-3.}, hi{3.};
    const auto result = bike::numerics::bounded_least_squares(
      [](std::span<const double> a) { return std::vector<double>{a[0]-2.}; },
      x, lo, hi, 80);
    bt::require(std::abs(result.x[0]-2.) < 1e-7, "bounded least-squares control");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T23
ctest --test-dir build/native-release -R '^migration_T23$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T23` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T23` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Freeze the exact scipy.optimize.least_squares call configurations in equilibrium_refine.py, sag_fit.py and drive_suspension_rig.py. Port the bounded trust-region reflective algorithm and required linear algebra, including three-point finite differences and x_scale=jac where selected.
  - [ ] 2. Do not claim arbitrary new optimizers are bitwise equivalent. The existing supported path is gated by captured initial states/first-step outcomes and unchanged residual thresholds; any provider/solver divergence must be isolated and reviewed, not hidden behind a global epsilon.
  - [ ] 3. Port refinement auxiliary scalar tire shear, foot/saddle/hand contact shear and freehub candidate staging. Restore rejected trial states and preserve the incoming freehub boundary.
  - [ ] 4. Keep the nq==nv planar constraint explicit until a separate topology project changes it. Preserve zero-velocity residual measurement and the source choice of best physically evaluated candidate.
  - [ ] 5. Port the eight re-centered refinement attempts and improvement/stagnation checks, and sag fitting parameter bounds, evaluation limits and config output. No SciPy subprocess or runtime FFI is permitted.

### Mandatory additional regression cases

- [ ] Analytical linear/bounded controls, singular Jacobian, constrained optimum and failed evaluation.
- [ ] Actual compiled acceleration residual matches source fixtures without relaxed 0.05 acceptance.
- [ ] Repeated rejected candidates do not mutate the accepted state.
- [ ] Sag target success and unreachable target failures.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T23$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/numerics/least_squares.hpp native/src/numerics/least_squares.cpp native/src/initialization/refine.cpp native/src/initialization/sag_fit.cpp native/tests/migration/test_t23.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T23 port bounded least squares, equilibrium refinement and sag fitting"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T24: Replace Python equilibrium cache/state files and retain old readers

**Dependencies:** T22, T23, T06

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/initialization/state_file.hpp` |
| Create | `native/src/initialization/state_file.cpp` |
| Create | `native/src/initialization/equilibrium_cache.cpp` |
| Create | `native/src/io/npy.hpp` |
| Create | `native/src/io/npy.cpp` |
| Create | `native/src/io/npz.cpp` |
| Create | `native/tests/migration/test_t24.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/native/artifact.py:1-248`
- `src/bike_sim/sim/ride/equilibrium_cache.py:1-391`
- `src/bike_sim/sim/ride/initial_state.py:1-213`

Declaration-level card: `migration/cards/T24.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T22` / `migration/api-index.md#t22`: Port physical settling without a reference RideSimulation.
- `T23` / `migration/api-index.md#t23`: Port bounded least squares, equilibrium refinement and sag fitting.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.

**Produces:**

```cpp
namespace bike::initialization {
struct StateFile;
StateFile read_state_file(const std::filesystem::path& file);
void write_state_file(const std::filesystem::path& file, const StateFile& state);
std::string fingerprint(const StateFile& state);
}
// Version any new writer format explicitly; retain read-only import of the
// archived NPZ/NPY variants actually used by equilibrium_cache.py.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t24.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "initialization/state_file.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    const auto s = bike::initialization::read_state_file(bt::fixture("equilibrium/legacy_state.npz"));
    bt::require(!bike::initialization::fingerprint(s).empty(), "state identity");
    bt::invalid([] { bike::initialization::read_state_file(bt::fixture("equilibrium/truncated.npz")); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T24
ctest --test-dir build/native-release -R '^migration_T24$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T24` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T24` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port the complete source fingerprint and state inventory, not only qpos. Include material/contact/controller state, named addresses, dimensions, config/terrain identity and settle evidence.
  - [ ] 2. Support the source NPZ/NPY variants using bounded ZIP and NPY readers or pinned native libraries. Never enable pickle or execute archive members. Validate compression ratio, dimensions, dtype/endian and member path before allocation.
  - [ ] 3. Preserve the explicit-state-file precedence and read-only semantics. Re-measure its residual and reject wrong identity/shape/residual with a filename-specific error.
  - [ ] 4. Characterize the auto-cache path separately: the source trusts certain cached residual metadata differently from explicit files. Keep that behavior for parity or make a separately reviewed validation change, not a hidden migration fix.
  - [ ] 5. Atomic cache writes use an exclusive temporary sibling and durable publication; a corrupt cache cannot clobber a valid user-managed state.

### Mandatory additional regression cases

- [ ] Cache hit/miss, explicit-file precedence, changed tire/controller/config hash.
- [ ] Round-trip all hidden state; same qpos with wrong material state rejects.
- [ ] Truncated ZIP, wrong dtype, oversized dimensions, symlink and foreign model.
- [ ] Legacy fixture import does not require NumPy.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T24$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/initialization/state_file.hpp native/src/initialization/state_file.cpp native/src/initialization/equilibrium_cache.cpp native/src/io/npy.hpp native/src/io/npy.cpp native/src/io/npz.cpp native/tests/migration/test_t24.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T24 replace python equilibrium cache/state files and retain old readers"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T25: Construct standalone native rides from user configuration

**Dependencies:** T13, T16, T18, T21, T24

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/app/prepare.hpp` |
| Create | `native/src/app/prepare.cpp` |
| Create | `native/src/app/physical_session.hpp` |
| Create | `native/src/app/physical_session.cpp` |
| Create | `native/src/app/setup.hpp` |
| Create | `native/tests/migration/test_t25.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/native/runtime.py:1-232`
- `src/bike_sim/native/setup.py:1-419`
- `src/bike_sim/sim/ride_sim.py:1-852`

Declaration-level card: `migration/cards/T25.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T13` / `migration/api-index.md#t13`: Port TOML/JSON loading, merge precedence and asset resolution.
- `T16` / `migration/api-index.md#t16`: Generate and compile physical MJCF entirely in C++.
- `T18` / `migration/api-index.md#t18`: Resolve model names into typed native writer/runtime configuration.
- `T21` / `migration/api-index.md#t21`: Port all track generators, presets and track-file round trips.
- `T24` / `migration/api-index.md#t24`: Replace Python equilibrium cache/state files and retain old readers.

**Produces:**

```cpp
namespace bike::app {
struct PreparedSimulation { runtime::RuntimeBootstrap bootstrap; bike::io::Value metadata; };
PreparedSimulation prepare(const std::filesystem::path& request_toml);
std::unique_ptr<runtime::NativeRideRuntime> make_ride(PreparedSimulation prepared);
}
// Request files carry explicit physics profile, track, rider, initial conditions
// and output-independent options. They never reference a Python callable.
// No create_ride alias: callers use prepare() followed by make_ride().
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t25.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "app/setup.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    auto prepared = bike::app::prepare(bt::fixture("requests/welded_flat.toml"));
    auto run = bike::app::make_ride(std::move(prepared));
    bt::require(run->snapshot().step == 0, "native startup seals t=0");
    bt::golden_f64("bootstrap/native_start_welded/integration.f64",
                   run->snapshot().integration_state);
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T25
ctest --test-dir build/native-release -R '^migration_T25$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T25` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T25` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Replace the entire build_physical_simulation -> Python reset/settle -> capture_bootstrap chain with native load/generate/compile/project/settle/capture.
  - [ ] 2. Introduce a typed initialization context where necessary instead of constructing a partially valid public NativeRideRuntime and mutating it through test hooks.
  - [ ] 3. Build metadata, preview state, initial frame and immutable reset bootstrap once. No .mjb from a Python generator is needed on a clean installation.
  - [ ] 4. Verify startup in an environment where python, python3, uv, mjpython and the old src/bike_sim tree are unavailable. Trace child processes and dynamic libraries, not just PATH.
  - [ ] 5. A failed preparation leaves no recording directory or successful manifest. Persistent recording remains opt-in and final-only.
  - [ ] 6. Internal request TOML schema and defaults are fixed in migration/interface-contracts.md. This is a fixture/application assembly format; it does not replace existing user-facing config files or CLI flags. prepare() calls SetupOwner, completes initialization and serializes the in-memory model image, then make_ride() constructs the sole live owner.

### Mandatory additional regression cases

- [ ] Each shipped native profile plus track-config combinations runs from TOML on a clean native installation.
- [ ] Native startup/reset agree with the original startup fixture under the pinned provider.
- [ ] Wrong configuration/model/asset errors name the originating file.
- [ ] Running preparation twice does not leak or cross-contaminate controller state.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T25$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/app/prepare.hpp native/src/app/prepare.cpp native/src/app/physical_session.hpp native/src/app/physical_session.cpp native/src/app/setup.hpp native/tests/migration/test_t25.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T25 construct standalone native rides from user configuration"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
