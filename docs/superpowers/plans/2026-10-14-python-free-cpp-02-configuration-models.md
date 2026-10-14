# Configuration, geometry and native model construction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T12 through T18 with source-backed parity and independently reviewed test cycles.

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
| T12 | Port configuration and parameter schemas without changing defaults | T05, T07 |
| T13 | Port TOML/JSON loading, merge precedence and asset resolution | T12, T06 |
| T14 | Port geometry and Horst-link kinematics | T12 |
| T15 | Port mass properties, suspension setup and rider geometry | T14, T12 |
| T16 | Generate and compile physical MJCF entirely in C++ | T13, T14, T15, T04 |
| T17 | Port standard, stand, playground and legacy MJCF exports | T16 |
| T18 | Resolve model names into typed native writer/runtime configuration | T07, T08, T13, T16 |

## Task T12: Port configuration and parameter schemas without changing defaults

**Dependencies:** T05, T07

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/configuration/model_config.hpp` |
| Create | `native/src/configuration/physical_config.hpp` |
| Create | `native/src/configuration/rider_spec.hpp` |
| Create | `native/src/configuration/suspension_spec.hpp` |
| Create | `native/src/configuration/validate.cpp` |
| Create | `native/src/configuration/schema.cpp` |
| Create | `native/tests/migration/test_t12.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/native/schema.py:1-384`
- `src/bike_sim/physics/__init__.py:1-46`
- `src/bike_sim/physics/checks.py:1-105`
- `src/bike_sim/physics/domain_validation.py:1-90`
- `src/bike_sim/physics/joint_strength.py:1-218`
- `src/bike_sim/physics/model_config.py:1-136`
- `src/bike_sim/physics/physical_config.py:1-474`
- `src/bike_sim/physics/rider_envelope.py:1-139`
- `src/bike_sim/physics/rider_posture.py:1-39`
- `src/bike_sim/physics/rider_program.py:1-134`
- `src/bike_sim/physics/suspension_config.py:1-41`

Declaration-level card: `migration/cards/T12.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T05` / `migration/api-index.md#t05`: Create strict lossless data and JSON boundaries.
- `T07` / `migration/api-index.md#t07`: Separate typed writer construction from nanobind parsing.

**Produces:**

```cpp
namespace bike::configuration {
struct SimulationPhysicsConfig; // full field list: migration/dataclass-contracts.md
struct RiderSpecs;
SimulationPhysicsConfig default_physics();
void validate(const SimulationPhysicsConfig& config);
bike::io::Value schema();
}
// Reuse native typed subrecords when they are semantically identical;
// input-facing defaults are not borrowed from zero-initialized wire structs.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t12.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "configuration/model_config.hpp"
#include "io/json.hpp"
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
    auto cfg = bike::configuration::default_physics();
    cfg.timestep_s = 0.0;
    bt::invalid([&] { bike::configuration::validate(cfg); });
    bt::golden_text("config/schema.json",
      bike::io::canonical_json(bike::configuration::schema()));
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T12
ctest --test-dir build/native-release -R '^migration_T12$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T12` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T12` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Transcribe every dataclass field, default/default factory, enum, optionality and __post_init__ domain from physics/model_config.py, physical_config.py and the parameter/spec modules listed in the owner ledger.
  - [ ] 2. Keep physics-mode versus drive-mode distinctions and mode-dependent restrictions. The current native subset is not the full product schema.
  - [ ] 3. Define integer versus bool explicitly; reject booleans where Python contracts require an integer. Preserve explicit infinities only on documented bounds.
  - [ ] 4. Generate the public schema from the same typed declarations/field tables used by loading, validation and provenance. Do not maintain a second independent defaults table.
  - [ ] 5. Maintain version floors and all supplied example-profile effective values, including 0.00125-second welded timestep. Do not copy that profile value into unrelated defaults.

### Mandatory additional regression cases

- [ ] All constructor-domain tests, field-name diagnostics and default factory ownership.
- [ ] All shipped TOML/JSON profile fields are covered exactly once.
- [ ] Round-trip resolved config retains optional/missing distinctions and aliases.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T12$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/configuration/model_config.hpp native/src/configuration/physical_config.hpp native/src/configuration/rider_spec.hpp native/src/configuration/suspension_spec.hpp native/src/configuration/validate.cpp native/src/configuration/schema.cpp native/tests/migration/test_t12.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T12 port configuration and parameter schemas without changing defaults"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T13: Port TOML/JSON loading, merge precedence and asset resolution

**Dependencies:** T12, T06

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/configuration/load.hpp` |
| Create | `native/src/configuration/load.cpp` |
| Create | `native/src/configuration/paths.cpp` |
| Create | `native/src/configuration/profile_writer.cpp` |
| Create | `native/tests/migration/test_t13.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/physics/plant_profile.py:1-87`
- `src/bike_sim/physics/resolution.py:1-119`

Declaration-level card: `migration/cards/T13.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T12` / `migration/api-index.md#t12`: Port configuration and parameter schemas without changing defaults.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.

**Produces:**

```cpp
namespace bike::configuration {
SimulationPhysicsConfig load_physics(const std::filesystem::path& file);
SimulationPhysicsConfig resolve_physics(const bike::io::Value& overrides,
  const SimulationPhysicsConfig& base);
std::string write_physics_toml(const SimulationPhysicsConfig& config);
}
// Relative referenced assets resolve against the declaring configuration file.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t13.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "configuration/load.hpp"
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
    const auto cfg = bike::configuration::load_physics(
      bt::fixture("config/viewer_physics_welded.toml"));
    bt::require(cfg.timestep_s == 0.00125, "shipped timestep");
    bt::invalid([] { bike::configuration::load_physics(bt::fixture("config/unknown_key.toml")); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T13
ctest --test-dir build/native-release -R '^migration_T13$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T13` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T13` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port physics/resolution.py recursive merge and cli/research.py explicit-value handling. Defaults, file settings and explicitly supplied CLI flags must have the same precedence; an absent flag is not an explicit default override.
  - [ ] 2. Preserve rider-strength/envelope/tire-calibration asset path resolution and ownership. Resolve once, retain original user spelling for diagnostics and absolute/content identity for execution.
  - [ ] 3. Use a pinned native TOML parser; reject unknown keys, malformed nested sections, wrong arrays/scalars, nonfinite prohibited values and unsupported schemas.
  - [ ] 4. Port plant_profile.py configuration writing and configuration_metadata/canonicalization behavior needed by recording and experiments.
  - [ ] 5. Do not automatically read arbitrary environment variables or silently look in a previous Python package resource directory.

### Mandatory additional regression cases

- [ ] Nested override precedence; explicit --dt/--timestep versus file value.
- [ ] Equivalent relative and absolute paths resolve to the same asset identity.
- [ ] Writing then reading TOML preserves effective config.
- [ ] Paths with spaces and nonexistent named assets have stable diagnostic paths.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T13$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/configuration/load.hpp native/src/configuration/load.cpp native/src/configuration/paths.cpp native/src/configuration/profile_writer.cpp native/tests/migration/test_t13.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T13 port toml/json loading, merge precedence and asset resolution"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T14: Port geometry and Horst-link kinematics

**Dependencies:** T12

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/geometry/specs.hpp` |
| Create | `native/src/geometry/hardpoints.cpp` |
| Create | `native/src/geometry/cockpit.cpp` |
| Create | `native/src/geometry/validation.cpp` |
| Create | `native/src/kinematics/linkage_math.hpp` |
| Create | `native/src/kinematics/linkage_math.cpp` |
| Create | `native/src/kinematics/solver.hpp` |
| Create | `native/src/kinematics/solver.cpp` |
| Create | `native/src/kinematics/curves.cpp` |
| Create | `native/tests/migration/test_t14.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/geometry/__init__.py:1-29`
- `src/bike_sim/geometry/cockpit.py:1-233`
- `src/bike_sim/geometry/hardpoints.py:1-237`
- `src/bike_sim/geometry/specs.py:1-112`
- `src/bike_sim/geometry/validation.py:1-31`
- `src/bike_sim/kinematics/__init__.py:1-17`
- `src/bike_sim/kinematics/curves.py:1-63`
- `src/bike_sim/kinematics/linkage_math.py:1-145`
- `src/bike_sim/kinematics/solver.py:1-361`

Declaration-level card: `migration/cards/T14.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T12` / `migration/api-index.md#t12`: Port configuration and parameter schemas without changing defaults.

**Produces:**

```cpp
namespace bike::kinematics {
double brentq(const std::function<double(double)>& f, double a, double b,
              double xtol = 1e-12);
class HorstLinkageSolver; // public operations mirror solver.py's named solves
}
// Geometry source units remain mm; conversions to engine meters stay explicit.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t14.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "kinematics/linkage_math.hpp"
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
    const double root = bike::kinematics::brentq(
      [](double x) { return x*x - 2.0; }, 0.0, 2.0, 1e-12);
    bt::require(std::abs(root*root - 2.0) < 1e-10, "bracketed root");
    bt::invalid([] { bike::kinematics::brentq([](double x) { return x*x+1.; }, -1., 1.); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T14
ctest --test-dir build/native-release -R '^migration_T14$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T14` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T14` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port the existing brentq_pure algorithm and arithmetic order rather than substituting a new generic root solver. Preserve bracket search, selected linkage branch and failure diagnostics.
  - [ ] 2. Implement fixed frame points, trail, front/rear axles, cockpit validation, circle intersections, instant centers, leverage and travel/stroke maps in focused files.
  - [ ] 3. Keep zero travel, maximum legal travel and singular/tangent cases explicit. A physically impossible geometry fails rather than being clamped into a plausible model.
  - [ ] 4. Compare source-generated numeric arrays across a dense travel grid and the exact public JSON geometry export. Use matching reduction/trigonometry functions when the reference contract is bitwise.
  - [ ] 5. Add independent geometric residual checks for link lengths and inverse travel/stroke consistency so a shared golden error is not the only oracle.

### Mandatory additional regression cases

- [ ] Endpoint travel and branch continuity.
- [ ] Unbracketed roots, degenerate geometry, NaN and reversed intervals.
- [ ] Units and public point names match geometry exports.
- [ ] Dense leverage curve and inverse solve fixtures.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T14$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/geometry/specs.hpp native/src/geometry/hardpoints.cpp native/src/geometry/cockpit.cpp native/src/geometry/validation.cpp native/src/kinematics/linkage_math.hpp native/src/kinematics/linkage_math.cpp native/src/kinematics/solver.hpp native/src/kinematics/solver.cpp native/src/kinematics/curves.cpp native/tests/migration/test_t14.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T14 port geometry and horst-link kinematics"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T15: Port mass properties, suspension setup and rider geometry

**Dependencies:** T14, T12

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/model/mass.hpp` |
| Create | `native/src/model/mass.cpp` |
| Create | `native/src/model/rider_geometry.hpp` |
| Create | `native/src/model/rider_geometry.cpp` |
| Create | `native/src/model/suspension_setup.hpp` |
| Create | `native/src/model/suspension_setup.cpp` |
| Create | `native/tests/migration/test_t15.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/physics/air_spring.py:1-297`
- `src/bike_sim/physics/coil_shock.py:1-129`
- `src/bike_sim/physics/component_masses.py:1-30`
- `src/bike_sim/physics/damper.py:1-476`
- `src/bike_sim/physics/inertia.py:1-99`
- `src/bike_sim/physics/mass.py:1-264`
- `src/bike_sim/physics/rider.py:1-1021`
- `src/bike_sim/physics/rider_segments.py:1-94`
- `src/bike_sim/physics/seated_climb.py:1-170`
- `src/bike_sim/physics/tuning.py:1-166`
- `src/bike_sim/sim/ride/mass_properties.py:1-35`

Declaration-level card: `migration/cards/T15.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T14` / `migration/api-index.md#t14`: Port geometry and Horst-link kinematics.
- `T12` / `migration/api-index.md#t12`: Port configuration and parameter schemas without changing defaults.

**Produces:**

```cpp
namespace bike::model {
std::array<double,9> ring_inertia(double mass_kg, double inner_m,
                                double outer_m, double width_m);
std::array<double,9> parallel_axis(double mass_kg, std::array<double,3> offset);
}
// Row-major full 3x3 tensors, not just diagonal moments. Cylinder axis is Y.
// RiderSpecs/SeatedPose/LegChain/ArticulatedPose retain the source field contracts
// documented in migration/dataclass-contracts.md.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t15.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "model/mass.hpp"
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
    const auto inertia = bike::model::ring_inertia(2.0, 0.1, 0.5, 0.06);
    bt::golden_f64("model/ring_2kg_0p1_0p5_0p06.f64", inertia);
    bt::invalid([] { bike::model::ring_inertia(-1.0, 0.1, 0.5, 0.06); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T15
ctest --test-dir build/native-release -R '^migration_T15$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T15` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T15` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port BikeMassSpecs component accounting, component centers, wheel inertias, parallel-axis assembly, static system CG and compiled-model mass observations.
  - [ ] 2. Port setup-time AirSpring/CoilShock/Damper specs, damper presets/click behavior, fork/shock tuning and suspension construction. Runtime force kernels already in C++ are reused, not forked.
  - [ ] 3. Port all none/lumped/seated/articulated_planar rider geometry, two-link IK, leg forward kinematics/Jacobians, anatomical envelopes/strength loaders and seated apparent-mass routines.
  - [ ] 4. Keep legacy rider and physical articulated geometry separate where the Python code does. Reject non-fitting rider configurations and preserve leg-branch choices.
  - [ ] 5. Preserve component registry uniqueness and every mass-bearing geometry assignment; mass reassignment must update inertial properties coherently.

### Mandatory additional regression cases

- [ ] Total mass and each component match; no unregistered or twice-registered mass.
- [ ] Seated fit, impossible reach and anatomical limits.
- [ ] Independent parallel-axis and IK/FK residual tests.
- [ ] Damper/air/coil setup fixtures use exact units and legacy_behavior settings.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T15$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/model/mass.hpp native/src/model/mass.cpp native/src/model/rider_geometry.hpp native/src/model/rider_geometry.cpp native/src/model/suspension_setup.hpp native/src/model/suspension_setup.cpp native/tests/migration/test_t15.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T15 port mass properties, suspension setup and rider geometry"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T16: Generate and compile physical MJCF entirely in C++

**Dependencies:** T13, T14, T15, T04

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/model/builder.hpp` |
| Create | `native/src/model/builder.cpp` |
| Create | `native/src/model/xml_writer.cpp` |
| Create | `native/src/model/physical_topology.cpp` |
| Create | `native/src/model/articulated_rider.cpp` |
| Create | `native/src/model/drivetrain.cpp` |
| Create | `native/src/model/linkage.cpp` |
| Create | `native/src/model/terrain.cpp` |
| Create | `native/tests/migration/test_t16.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/mujoco/_xml_format.py:1-167`
- `src/bike_sim/mujoco/actuators.py:1-155`
- `src/bike_sim/mujoco/articulated_rider.py:1-91`
- `src/bike_sim/mujoco/builder.py:1-412`
- `src/bike_sim/mujoco/drivetrain.py:1-326`
- `src/bike_sim/mujoco/linkage.py:1-23`
- `src/bike_sim/mujoco/physical_topology.py:1-246`
- `src/bike_sim/mujoco/rear_linkage.py:1-397`
- `src/bike_sim/mujoco/steering_fork.py:1-201`
- `src/bike_sim/mujoco/terrain.py:1-54`

Declaration-level card: `migration/cards/T16.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T13` / `migration/api-index.md#t13`: Port TOML/JSON loading, merge precedence and asset resolution.
- `T14` / `migration/api-index.md#t14`: Port geometry and Horst-link kinematics.
- `T15` / `migration/api-index.md#t15`: Port mass properties, suspension setup and rider geometry.
- `T04` / `migration/api-index.md#t04`: Preserve numerical providers and fatal-engine containment.

**Produces:**

```cpp
namespace bike::model {
struct BuildRequest; // resolved bike, rider, masses, mode, terrain and physics
std::string generate_mjcf(const BuildRequest& request);
struct ModelDeleter { void operator()(mjModel*) const noexcept; };
using ModelOwner = std::unique_ptr<mjModel, ModelDeleter>;
ModelOwner compile_mjcf(std::string_view xml, const std::filesystem::path& asset_root);
}
// Reuse engine containment for compilation; report XML diagnostics with context.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t16.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "model/builder.hpp"
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
    bt::invalid([] {
      bike::model::compile_mjcf("<mujoco><broken></mujoco>", ".");
    });
    const auto xml = bt::text(bt::fixture("model/welded.xml"));
    auto model = bike::model::compile_mjcf(xml, bt::fixture("model/assets"));
    bt::require(model != nullptr && model->nq > 0, "native model compilation");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T16
ctest --test-dir build/native-release -R '^migration_T16$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T16` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T16` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port the physical branches of all 18 mujoco modules, preserving body/joint/site/geom/equality/tendon/sensor names and deterministic insertion order. The source files remain the topology authority, not historical 3D plans.
  - [ ] 2. Use escaped, locale-independent XML output. Explicitly preserve numeric precision and attribute defaults; structural/compiled-array equality is the gate unless byte-level XML identity is specifically required by an existing hash contract.
  - [ ] 3. Compile from generated XML and native assets through the pinned MuJoCo API, without a Python-created .mjb. Compare nq/nv/nu/na and all named IDs, joint addresses, inertia, geometry, equality and actuator arrays against captured models.
  - [ ] 4. Preserve physical rider spindle/pin/connect topology, crank/freehub topology, tire backend contact disabling, mass registry, COM marker nonzero initialization and correct integration options.
  - [ ] 5. Provide isolated ownership for model/data creation; compilation failure cannot leak or expose a partially compiled model.

### Mandatory additional regression cases

- [ ] Each shipped welded/distributed/pneumatic physical profile compiles with matching structural inventory.
- [ ] Wrong mode/rider/tire combinations reject before allocating a running environment.
- [ ] Names/ordering and downstream address mapping match the reference.
- [ ] Compiled-array differences are diagnosed field-by-field, not dismissed as XML formatting.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T16$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/model/builder.hpp native/src/model/builder.cpp native/src/model/xml_writer.cpp native/src/model/physical_topology.cpp native/src/model/articulated_rider.cpp native/src/model/drivetrain.cpp native/src/model/linkage.cpp native/src/model/terrain.cpp native/tests/migration/test_t16.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T16 generate and compile physical mjcf entirely in c++"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T17: Port standard, stand, playground and legacy MJCF exports

**Dependencies:** T16

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/model/legacy_topology.cpp` |
| Create | `native/src/model/export.hpp` |
| Create | `native/src/model/export.cpp` |
| Create | `native/src/model/assets.cpp` |
| Create | `native/src/model/environment.cpp` |
| Create | `native/src/model/sensors.cpp` |
| Create | `native/tests/migration/test_t17.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/mujoco/__init__.py:1-39`
- `src/bike_sim/mujoco/assets.py:1-74`
- `src/bike_sim/mujoco/builder.py:1-412`
- `src/bike_sim/mujoco/environment.py:1-137`
- `src/bike_sim/mujoco/exporter.py:1-285`
- `src/bike_sim/mujoco/frame.py:1-311`
- `src/bike_sim/mujoco/reference_rider.py:1-40`
- `src/bike_sim/mujoco/rider.py:1-232`
- `src/bike_sim/mujoco/sensors.py:1-56`

Declaration-level card: `migration/cards/T17.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T16` / `migration/api-index.md#t16`: Generate and compile physical MJCF entirely in C++.

**Produces:**

```cpp
namespace bike::model {
void export_mjcf(const BuildRequest& request, const std::filesystem::path& file);
void export_geometry_json(const BuildRequest& request, const std::filesystem::path& file);
void export_playground_models(const std::filesystem::path& directory);
}
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t17.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "model/export.hpp"
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
    bike::model::export_playground_models("migration-test-output/models");
    bt::require(std::filesystem::is_directory("migration-test-output/models"),
                "playground export creates output directory");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T17
ctest --test-dir build/native-release -R '^migration_T17$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T17` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T17` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port standard/stand/playground/ride branches, lumped/seated/rigid/articulated legacy rider selection, sphere and legacy pneumatic geometry, debug markers and visual assets.
  - [ ] 2. Preserve builder timestep choices by mode and the include_rider/rider resolution rules. Do not route nonphysical callers into the native physical subset.
  - [ ] 3. Reproduce exporter.py file naming, JSON schema and options used by bike-export/bike-sim. Write through explicit output paths without changing the current directory.
  - [ ] 4. Capture and verify each exported artifact file name, required content and compiled model inventory. Existing outputs may be overwritten only where the original command explicitly allowed it.
  - [ ] 5. Keep model generation and exporting usable in a headless build without GLFW.

### Mandatory additional regression cases

- [ ] All four mode families; include_rider override precedence.
- [ ] JSON-only/XML-only flags and generated playground model names.
- [ ] A headless install exports models without loading a GUI library.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T17$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/model/legacy_topology.cpp native/src/model/export.hpp native/src/model/export.cpp native/src/model/assets.cpp native/src/model/environment.cpp native/src/model/sensors.cpp native/tests/migration/test_t17.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T17 port standard, stand, playground and legacy mjcf exports"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T18: Resolve model names into typed native writer/runtime configuration

**Dependencies:** T07, T08, T13, T16

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/model/mapping.hpp` |
| Create | `native/src/model/mapping.cpp` |
| Create | `native/src/runtime/project_config.hpp` |
| Create | `native/src/runtime/project_config.cpp` |
| Create | `native/tests/migration/test_t18.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/native/config.py:1-468`
- `src/bike_sim/native/setup.py:1-419`
- `src/bike_sim/sim/ride/physical_mapping.py:1-81`

Declaration-level card: `migration/cards/T18.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T07` / `migration/api-index.md#t07`: Separate typed writer construction from nanobind parsing.
- `T08` / `migration/api-index.md#t08`: Remove binding types from rider and drivetrain state wires.
- `T13` / `migration/api-index.md#t13`: Port TOML/JSON loading, merge precedence and asset resolution.
- `T16` / `migration/api-index.md#t16`: Generate and compile physical MJCF entirely in C++.

**Produces:**

```cpp
namespace bike::model {
int required_id(const mjModel& model, mjtObj kind, std::string_view name);
}
namespace runtime {
RuntimeConfig project_runtime(const bike::configuration::SimulationPhysicsConfig& config,
  const mjModel& model);
}
// Full writer projection includes resolved tire material/terrain and rider geometry;
// input-facing config and model-specific IDs must not become a single mutable object.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t18.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "model/mapping.hpp"
#include "model/builder.hpp"
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
    auto model = bike::model::compile_mjcf(
      bt::text(bt::fixture("model/welded.xml")), bt::fixture("model/assets"));
    bt::invalid([&] { bike::model::required_id(*model, mjOBJ_JOINT, "not_a_joint"); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T18
ctest --test-dir build/native-release -R '^migration_T18$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T18` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T18` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port native/config.py, native/setup.py geometry/rider-controller projection and physical_mapping.py operations. Retain exact schema paths and named-joint DOF references.
  - [ ] 2. Project tire backend-dependent material records, configured versus track surfaces, drive policies, strength/envelope tables, damping metadata and pose once.
  - [ ] 3. Never validate model compatibility by matching dimensions alone: bind resolved references to the actual model owner and content identity.
  - [ ] 4. Keep structural supported-capability validation distinct from configuration parsing. Later T27-T31 expand supported capabilities only with working implementations and tests.
  - [ ] 5. Compare the new projection to captured Python projections before connecting it to new equilibrium construction.

### Mandatory additional regression cases

- [ ] Missing/wrong object names and same-dimension wrong model reject.
- [ ] All tire surface modes produce the expected resolved data.
- [ ] Runtime config projection does not depend on a Python RideSimulation object.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T18$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/model/mapping.hpp native/src/model/mapping.cpp native/src/runtime/project_config.hpp native/src/runtime/project_config.cpp native/tests/migration/test_t18.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T18 resolve model names into typed native writer/runtime configuration"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
