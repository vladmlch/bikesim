# Complete remaining physics, rider and legacy capabilities Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T26 through T33 with source-backed parity and independently reviewed test cycles.

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
| T26 | Close parity for already-ported kernels and retire their Python mirrors | T11, T25 |
| T27 | Implement remaining drivetrain capabilities | T26, T18 |
| T28 | Port constrained allocation, linear feasibility and calibration solvers | T23, T15 |
| T29 | Port remaining rider dynamics and coupled task controllers | T28, T26 |
| T30 | Implement flat/weld/spring rider supports and missing physical modes | T29, T27, T16 |
| T31 | Port legacy tire backends and nonphysical ride integration | T20, T27, T30 |
| T32 | Port remaining legacy rider/control/measurement behavior | T31, T15 |
| T33 | Port suspension playground and stand controllers | T17, T32 |

## Task T26: Close parity for already-ported kernels and retire their Python mirrors

**Dependencies:** T11, T25

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/tests/migration/kernel_parity.cpp` |
| Create | `native/tests/migration/kernel_properties.cpp` |
| Create | `migration/kernel-reuse-map.json` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/physics/attachment_budget.py:1-55`
- `src/bike_sim/physics/battery.py:1-74`
- `src/bike_sim/physics/distributed_tire.py:1-185`
- `src/bike_sim/physics/drivetrain.py:1-329`
- `src/bike_sim/physics/energy_ledger.py:1-42`
- `src/bike_sim/physics/external_resistance.py:1-42`
- `src/bike_sim/physics/freehub.py:1-37`
- `src/bike_sim/physics/grip_release.py:1-15`
- `src/bike_sim/physics/motor.py:1-165`
- `src/bike_sim/physics/motor_profile.py:1-99`
- `src/bike_sim/physics/pedaling.py:1-133`
- `src/bike_sim/physics/rider_activation.py:1-28`
- `src/bike_sim/physics/shifting.py:1-111`
- `src/bike_sim/physics/stops.py:1-63`
- `src/bike_sim/physics/tire.py:1-252`
- `src/bike_sim/sim/ride/_step_oracle.py:1-213`
- `src/bike_sim/sim/ride/attachment_wrench.py:1-307`
- `src/bike_sim/sim/ride/balance_monitor.py:1-84`
- `src/bike_sim/sim/ride/braking.py:1-124`
- `src/bike_sim/sim/ride/constraint_forces.py:1-88`
- `src/bike_sim/sim/ride/contact_filter.py:1-42`
- `src/bike_sim/sim/ride/contact_state.py:1-201`
- `src/bike_sim/sim/ride/contacts.py:1-518`
- `src/bike_sim/sim/ride/control_clock.py:1-35`
- `src/bike_sim/sim/ride/crank_split.py:1-67`
- `src/bike_sim/sim/ride/cruise.py:1-244`
- `src/bike_sim/sim/ride/distributed_tire_forces.py:1-218`
- `src/bike_sim/sim/ride/drivetrain_forces.py:1-582`
- `src/bike_sim/sim/ride/force_accumulator.py:1-54`
- `src/bike_sim/sim/ride/geometric_freehub.py:1-193`
- `src/bike_sim/sim/ride/ideal_freehub.py:1-107`
- `src/bike_sim/sim/ride/lean_limit.py:1-69`
- `src/bike_sim/sim/ride/pedal_recovery.py:1-59`
- `src/bike_sim/sim/ride/physical_crash.py:1-60`
- `src/bike_sim/sim/ride/physical_driver.py:1-420`
- `src/bike_sim/sim/ride/physical_energy.py:1-134`
- `src/bike_sim/sim/ride/physical_observations.py:1-193`
- `src/bike_sim/sim/ride/physical_resistance.py:1-47`
- `src/bike_sim/sim/ride/physical_runtime.py:1-951`
- `src/bike_sim/sim/ride/pneumatic_tire_forces.py:1-478`
- `src/bike_sim/sim/ride/reference_monitor.py:1-31`
- `src/bike_sim/sim/ride/rider_contacts.py:1-721`
- `src/bike_sim/sim/ride/rider_effort.py:1-122`
- `src/bike_sim/sim/ride/rider_forces.py:1-217`
- `src/bike_sim/sim/ride/rider_intent.py:1-97`
- `src/bike_sim/sim/ride/rider_state.py:1-118`
- `src/bike_sim/sim/ride/rim_events.py:1-44`
- `src/bike_sim/sim/ride/static_braking.py:1-37`
- `src/bike_sim/sim/ride/termination.py:1-269`
- `src/bike_sim/sim/ride/tyre/__init__.py:1-16`
- `src/bike_sim/sim/ride/tyre/applier.py:1-195`
- `src/bike_sim/sim/ride/tyre/brush.py:1-275`
- `src/bike_sim/sim/ride/tyre/carcass.py:1-373`
- `src/bike_sim/sim/ride/tyre/geometry.py:1-288`
- `src/bike_sim/sim/ride/tyre/model.py:1-492`
- `src/bike_sim/sim/ride/tyre/profile.py:1-42`
- `src/bike_sim/sim/ride/virtual_rider.py:1-271`
- `src/bike_sim/sim/ride/wheel_kinematics.py:1-40`

Declaration-level card: `migration/cards/T26.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T11` / `migration/api-index.md#t11`: Separate native samples and column pieces from their Python boxers.
- `T25` / `migration/api-index.md#t25`: Construct standalone native rides from user configuration.

**Produces:**

```text
The reuse map records {python_symbol, native_symbol, source_locations,
 test_ids, comparison_class, remaining_branches}. A symbol is deletable only
 when every reachable branch and named input domain has native coverage.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -R 'migration_T26|native_contract_tests|native_distance_tests|native_station_tests' --output-on-failure --no-tests=error
# During migration, also run the unchanged source parity tests:
uv run --frozen --group native python -m pytest tests/reference/test_native_tire.py tests/reference/test_native_drivetrain.py tests/reference/test_native_suspension.py -q
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T26
ctest --test-dir build/native-release -R '^migration_T26$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T26` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T26` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Map physics and ride mirrors to existing writers/, pneumatic/, tyre/, drivetrain/, rider/ and runtime/ implementations by symbol and branch, not by matching basename.
  - [ ] 2. Port any residual setup/measurement/helper functions from mirrored modules into their owning native subsystem. Keep equations, operation ordering and intentional saturations unchanged.
  - [ ] 3. Freeze pure-Python-oracle fixtures for every existing bitwise test, including parameterized domains, invalid inputs, array aliases and empty/degenerate cases. Add independently derived conservation/monotonicity/bound tests.
  - [ ] 4. Do not remove a mirror because the default native profile passes. Mark the module eligible only after the complete function/branch inventory is mapped.
  - [ ] 5. Keep actual deletion centralized in T63 so other migration branches retain readable oracles.

### Mandatory additional regression cases

- [ ] No kernel is rewritten solely to modernize style.
- [ ] Every Python mirror has a concrete native symbol and mapped tests.
- [ ] The pneumatic backend is included even though older prose docs describe a narrower native subset.
- [ ] Bitwise comparison runs on the matched provider; cross-platform tests use their explicit class.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T26$' --no-tests=error --output-on-failure
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
git add -- native/tests/migration/kernel_parity.cpp native/tests/migration/kernel_properties.cpp migration/kernel-reuse-map.json migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T26 close parity for already-ported kernels and retire their python mirrors"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T27: Implement remaining drivetrain capabilities

**Dependencies:** T26, T18

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/drivetrain/elastic_chain.cpp` |
| Create | `native/src/drivetrain/motor_clutch.cpp` |
| Modify | `native/src/drivetrain/transmission.cpp` |
| Modify | `native/src/writers/drivetrain.cpp` |
| Create | `native/src/runtime/project_config.cpp` |
| Create | `native/tests/migration/test_t27.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/physics/chain.py:1-105`
- `src/bike_sim/physics/physical_config.py:1-474`
- `src/bike_sim/physics/transmission_constraint.py:1-63`
- `src/bike_sim/sim/ride/drivetrain.py:1-580`
- `native/src/writers/drivetrain.hpp:1-554`

Declaration-level card: `migration/cards/T27.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T26` / `migration/api-index.md#t26`: Close parity for already-ported kernels and retire their Python mirrors.
- `T18` / `migration/api-index.md#t18`: Resolve model names into typed native writer/runtime configuration.

**Produces:**

```cpp
namespace drivetrain {
struct Capability { bool elastic_chain; bool motor_clutch; bool reflected_rotor; };
Capability implemented_capabilities();
}
// Extend existing DriveConfig/DrivetrainWriter; do not create a parallel drive engine.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t27.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "writers/drivetrain.hpp"
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
    const auto c = drivetrain::implemented_capabilities();
    bt::require(c.elastic_chain && c.motor_clutch && c.reflected_rotor,
                "remaining drivetrain modes have native implementations");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T27
ctest --test-dir build/native-release -R '^migration_T27$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T27` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T27` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port Python-supported elastic_chain behavior, optional motor clutch and nonzero reflected rotor inertia, plus any legacy/freehub/geometric variants not covered by current native writers.
  - [ ] 2. Preserve chain geometry derivatives, freewheel topology, active constraints, stiffness/damping energy and coefficient updates. Use source tests for locked-chain, motor freewheel, momentum and drive-suspension coupling.
  - [ ] 3. Remove one native capability rejection only in the commit that adds its implementation and positive/negative tests. Keep unrelated rejections until their owner tasks complete.
  - [ ] 4. Validate torque limits, assist, shifting, battery accounting, crank phase/ripple and reset for every newly supported transmission family.
  - [ ] 5. Use existing force accumulation/transaction boundaries so mixed-mode changes cannot partially mutate model coefficients.

### Mandatory additional regression cases

- [ ] Elastic-chain energy/reaction and stiffness tests.
- [ ] Clutch engage/disengage and zero/nonzero rotor inertia.
- [ ] Motor freewheel topology and no spurious external momentum.
- [ ] Unsupported parameter combinations still fail explicitly.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T27$' --no-tests=error --output-on-failure
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
git add -- native/src/drivetrain/elastic_chain.cpp native/src/drivetrain/motor_clutch.cpp native/src/drivetrain/transmission.cpp native/src/writers/drivetrain.cpp native/src/runtime/project_config.cpp native/tests/migration/test_t27.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T27 implement remaining drivetrain capabilities"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T28: Port constrained allocation, linear feasibility and calibration solvers

**Dependencies:** T23, T15

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/numerics/slsqp.hpp` |
| Create | `native/src/numerics/slsqp.cpp` |
| Create | `native/src/numerics/linear_feasibility.cpp` |
| Create | `native/src/numerics/nnls.cpp` |
| Create | `native/src/rider/allocation.hpp` |
| Create | `native/src/rider/allocation.cpp` |
| Create | `native/tests/migration/test_t28.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/physics/distributed_tire.py:1-185`
- `src/bike_sim/physics/rider_allocation.py:1-189`
- `src/bike_sim/sim/ride/rider_response_allocation.py:1-274`

Declaration-level card: `migration/cards/T28.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T23` / `migration/api-index.md#t23`: Port bounded least squares, equilibrium refinement and sag fitting.
- `T15` / `migration/api-index.md#t15`: Port mass properties, suspension setup and rider geometry.

**Produces:**

```cpp
namespace bike::rider {
struct Allocation { std::vector<double> solution; bool feasible; double violation; };
struct AllocationProblem; // explicit matrices, bounds and differentiable extras
AllocationProblem allocation_problem_from_json(const bike::io::Value& value);
Allocation allocate(const AllocationProblem& problem);
}
// Port the exact nearest-feasible-point objective and source residual check;
// optimizer success alone never determines feasibility.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t28.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "rider/allocation.hpp"
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
    const auto problem = bike::rider::allocation_problem_from_json(
      bike::io::parse_json(bt::text(bt::fixture("allocation/box_projection.json"))));
    const auto result = bike::rider::allocate(problem);
    bt::require(result.feasible && result.violation <= 1e-7, "certified allocation");
    bt::golden_f64("allocation/box_projection.f64", result.solution);
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T28
ctest --test-dir build/native-release -R '^migration_T28$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T28` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T28` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Implement AllocationProblem and allocation_problem_from_json in allocation.hpp/cpp as a test/data boundary. Production callers build typed matrices directly.
  - [ ] 2. Port bounds projection fast path, residual against every equality/inequality/nonlinear family, equality-first constraint order, SLSQP objective/gradient, four restarts, ftol=1e-12, maxiter=500 and feasibility <=1e-7.
  - [ ] 3. Port the LP feasibility prefilter used by rider_response_allocation.py. An unknown/numerically failed solve is not an infeasibility certificate; it must not prune a valid contact mode.
  - [ ] 4. Port NonlinearConstraint/LinearConstraint translation, activation delivery and grip cone/branch constraints. Native callbacks own their captured data and cannot outlive a runtime.
  - [ ] 5. Port NNLS used by distributed_tire.fit_density and source SciPy brentq uses in validation rigs. Use pinned native solver source or a faithful C++ implementation; no Python, no runtime SciPy bridge, no silent solver substitution.
  - [ ] 6. Maintain exact defaults/order first. Solver replacement or a new feasible solution changes numeric execution identity unless bitwise equivalence is demonstrated.

### Mandatory additional regression cases

- [ ] Analytic box projection, active equalities, infeasible region and nonlinear grip examples.
- [ ] NaN matrix/bounds, reversed bounds and mismatched warm start reject.
- [ ] LP uncertainty never rejects a mode.
- [ ] NNLS nonnegativity, density fit residual and independent physical bounds.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T28$' --no-tests=error --output-on-failure
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
git add -- native/src/numerics/slsqp.hpp native/src/numerics/slsqp.cpp native/src/numerics/linear_feasibility.cpp native/src/numerics/nnls.cpp native/src/rider/allocation.hpp native/src/rider/allocation.cpp native/tests/migration/test_t28.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T28 port constrained allocation, linear feasibility and calibration solvers"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T29: Port remaining rider dynamics and coupled task controllers

**Dependencies:** T28, T26

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/rider/constraint_dynamics.hpp` |
| Create | `native/src/rider/constraint_dynamics.cpp` |
| Create | `native/src/rider/response_allocation.cpp` |
| Create | `native/src/rider/activation_allocation.cpp` |
| Create | `native/src/rider/mode_search.cpp` |
| Create | `native/src/rider/legacy_controller.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/ride/rider_activation_allocation.py:1-55`
- `src/bike_sim/sim/ride/rider_balance.py:1-27`
- `src/bike_sim/sim/ride/rider_control.py:1-1600`
- `src/bike_sim/sim/ride/rider_dynamics.py:1-198`
- `src/bike_sim/sim/ride/rider_response_allocation.py:1-274`

Declaration-level card: `migration/cards/T29.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T28` / `migration/api-index.md#t28`: Port constrained allocation, linear feasibility and calibration solvers.
- `T26` / `migration/api-index.md#t26`: Close parity for already-ported kernels and retire their Python mirrors.

**Produces:**

```cpp
namespace bike::rider {
struct ModeSearchResult { bool complete; std::size_t modes_tried; Allocation allocation; };
// Controller-specific typed interfaces mirror rider_dynamics.py,
// rider_response_allocation.py and rider_control.py and consume the same
// owned model context as existing SpindleController, never a foreign mjData*. 
}
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -R 'migration_T29' --output-on-failure --no-tests=error
uv run --frozen --group native python -m pytest tests/reference/test_rider_dynamics.py tests/reference/test_rider_allocation.py tests/reference/test_coupled_rider_task.py project/tests/reference/test_constraint_mode_search.py -q
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T29
ctest --test-dir build/native-release -R '^migration_T29$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T29` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T29` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port current soft-constraint response extraction, inverse dynamics rows, support wrench mapping, active contact mode enumeration and complete/incomplete mode-search evidence.
  - [ ] 2. Port coupled crank torque tasks, strength/power/speed envelopes, activation latent/delivered state, warm starts, grip branch selection and fallback reporting.
  - [ ] 3. Preserve the invariant that a fallback may be diagnostic/infeasible; do not convert it into a valid controller state merely because native code returns a vector.
  - [ ] 4. Keep the existing closed-form spindle controller unchanged for its current mode and dispatch explicitly to the newly ported modes.
  - [ ] 5. Add direct C++ tests corresponding to project/tests/reference/test_constraint_mode_search.py; do not lose it because it lives outside the primary tests directory.

### Mandatory additional regression cases

- [ ] Mode-search completeness, legal prefilter pruning and infeasible fallback diagnostics.
- [ ] Independent force/momentum residuals, torque/power/strength bounds.
- [ ] No privileged plant data enters public research observations.
- [ ] Controller state restoration/reset and failure atomicity.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T29$' --no-tests=error --output-on-failure
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
git add -- native/src/rider/constraint_dynamics.hpp native/src/rider/constraint_dynamics.cpp native/src/rider/response_allocation.cpp native/src/rider/activation_allocation.cpp native/src/rider/mode_search.cpp native/src/rider/legacy_controller.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T29 port remaining rider dynamics and coupled task controllers"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T30: Implement flat/weld/spring rider supports and missing physical modes

**Dependencies:** T29, T27, T16

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/rider/support_dispatch.cpp` |
| Create | `native/src/rider/flat_contacts.cpp` |
| Create | `native/src/rider/weld_settle.cpp` |
| Create | `native/src/rider/grip_springs.cpp` |
| Create | `native/src/runtime/capabilities.hpp` |
| Create | `native/src/runtime/capabilities.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/native/setup.py:1-419`
- `src/bike_sim/physics/physical_config.py:1-474`
- `src/bike_sim/sim/ride/control.py:1-43`
- `src/bike_sim/sim/ride/rider_support.py:1-117`
- `src/bike_sim/sim/ride/support_geometry.py:1-347`
- `src/bike_sim/sim/ride/weld_pedals.py:1-208`

Declaration-level card: `migration/cards/T30.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T29` / `migration/api-index.md#t29`: Port remaining rider dynamics and coupled task controllers.
- `T27` / `migration/api-index.md#t27`: Implement remaining drivetrain capabilities.
- `T16` / `migration/api-index.md#t16`: Generate and compile physical MJCF entirely in C++.

**Produces:**

```cpp
namespace runtime {
struct CapabilityReport { bool supported; std::vector<std::string> reasons; };
CapabilityReport capabilities(const bike::configuration::SimulationPhysicsConfig& config,
                              const bike::configuration::RiderSpecs& rider);
}
// Successful parsing is distinct from supported execution; each mode has a
// tested writer/controller/topology combination before capabilities returns true.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -R 'migration_T30' --output-on-failure --no-tests=error
uv run --frozen --group native python -m pytest tests/test_rider_welds.py tests/reference/test_pinned_topology.py tests/reference/test_seated_topology.py tests/reference/test_planar_arms.py -q
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T30
ctest --test-dir build/native-release -R '^migration_T30$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T30` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T30` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port physical flat/weld pedal and saddle attachment behavior and spring grips, including settle-only weld adjustment, release/overload budgets and per-attachment energy.
  - [ ] 2. Support Python-accepted rider variants, drive modes and optional pitch-assist branches through explicit capability combinations; never erase the old validator wholesale.
  - [ ] 3. Keep force application points, support normals, friction/constraint reactions and model topology coherent. A config parser accepting a string does not constitute implemented support.
  - [ ] 4. Use model-builder tests for each topology and runtime tests for reset, airborne/released contacts, overload, initial settling and active pedaling.
  - [ ] 5. Update the capability matrix to distinguish implemented, physically invalid/diagnostic, and invalid configuration.

### Mandatory additional regression cases

- [ ] All source-accepted pedal/saddle/grip combinations exercised or source-rejected identically.
- [ ] Contact energy and separation/release behavior.
- [ ] No hidden pitch stabilization added to the already-native profile.
- [ ] Every former native setup rejection has a mapped implementation/test or an unchanged source-level product rejection.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T30$' --no-tests=error --output-on-failure
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
git add -- native/src/rider/support_dispatch.cpp native/src/rider/flat_contacts.cpp native/src/rider/weld_settle.cpp native/src/rider/grip_springs.cpp native/src/runtime/capabilities.hpp native/src/runtime/capabilities.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T30 implement flat/weld/spring rider supports and missing physical modes"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T31: Port legacy tire backends and nonphysical ride integration

**Dependencies:** T20, T27, T30

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/legacy/tyre.hpp` |
| Create | `native/src/legacy/tyre.cpp` |
| Create | `native/src/legacy/ride.hpp` |
| Create | `native/src/legacy/ride.cpp` |
| Create | `native/src/legacy/force_accumulator.cpp` |
| Create | `native/src/legacy/recorder.cpp` |
| Create | `native/tests/migration/test_t31.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/physics/tyre.py:1-319`
- `src/bike_sim/sim/ride/__init__.py:1-50`
- `src/bike_sim/sim/ride/forces.py:1-229`
- `src/bike_sim/sim/ride/resistance.py:1-161`
- `src/bike_sim/sim/ride/tire_forces.py:1-278`
- `src/bike_sim/sim/ride/wheels.py:1-130`
- `src/bike_sim/sim/ride_sim.py:1-852`

Declaration-level card: `migration/cards/T31.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T20` / `migration/api-index.md#t20`: Port terrain primitives, profiles, surfaces and heightfields.
- `T27` / `migration/api-index.md#t27`: Implement remaining drivetrain capabilities.
- `T30` / `migration/api-index.md#t30`: Implement flat/weld/spring rider supports and missing physical modes.

**Produces:**

```cpp
namespace bike::legacy {
class RideSimulation; // owned model/data and explicit control/step/snapshot/reset
std::unique_ptr<RideSimulation> make_ride(const std::filesystem::path& request);
}
// Do not relabel the old legacy pneumatic model as the newer native pneumatic
// carcass/brush backend; they are separate source-supported behaviors.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t31.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "legacy/ride.hpp"
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
    auto sim = bike::legacy::make_ride(bt::fixture("requests/legacy_flat.toml"));
    bt::require(sim != nullptr, "legacy path has a native owner");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T31
ctest --test-dir build/native-release -R '^migration_T31$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T31` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T31` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port RideSimulation nonphysical branches and sim/ride/tyre/{profile,geometry,carcass,brush,model,applier}. Preserve tire tiers/pressure and native_reference/tabulated-material branches not covered by current native writers.
  - [ ] 2. Keep source integration timestep, suspension force order, force accumulation, cruise/resistance/leg-drive/rider-force behavior and termination rules for legacy paths.
  - [ ] 3. Port legacy telemetry/CSV capture and physical-versus-legacy checks without forcing every mode into new full recording. Legacy/preview rejection of --record remains unless separately approved.
  - [ ] 4. Reuse model/geometry helpers and native kernels where truly equivalent; retain distinct equations where legacy_behavior intentionally changes physics.
  - [ ] 5. Provide deterministic reset and snapshots for viewer/validation use.

### Mandatory additional regression cases

- [ ] Sphere and each legacy pneumatic tier/pressure configuration.
- [ ] Legacy speed/cruise and drive behavior, suspension damping and force timing.
- [ ] CSV columns and terminal outcomes match captured runs.
- [ ] Native physical pneumatic fixtures remain unchanged.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T31$' --no-tests=error --output-on-failure
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
git add -- native/src/legacy/tyre.hpp native/src/legacy/tyre.cpp native/src/legacy/ride.hpp native/src/legacy/ride.cpp native/src/legacy/force_accumulator.cpp native/src/legacy/recorder.cpp native/tests/migration/test_t31.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T31 port legacy tire backends and nonphysical ride integration"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T32: Port remaining legacy rider/control/measurement behavior

**Dependencies:** T31, T15

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/legacy/controls.cpp` |
| Create | `native/src/legacy/rider.cpp` |
| Create | `native/src/legacy/legs.cpp` |
| Create | `native/src/legacy/metrics.cpp` |
| Create | `native/src/numerics/filter.hpp` |
| Create | `native/src/numerics/filter.cpp` |
| Create | `native/tests/migration/test_t32.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/ride/energy.py:1-87`
- `src/bike_sim/sim/ride/leg_drive.py:1-221`
- `src/bike_sim/sim/ride/leg_loop.py:1-89`
- `src/bike_sim/sim/ride/metrics.py:1-739`
- `src/bike_sim/sim/ride/wheelie.py:1-240`
- `src/bike_sim/sim/telemetry.py:1-220`

Declaration-level card: `migration/cards/T32.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T31` / `migration/api-index.md#t31`: Port legacy tire backends and nonphysical ride integration.
- `T15` / `migration/api-index.md#t15`: Port mass properties, suspension setup and rider geometry.

**Produces:**

```cpp
namespace bike::numerics {
std::vector<double> legacy_ride_filter(std::span<const double> samples,
                                     double sample_rate_hz);
}
// The exact order/cutoff/padding settings come from sim/ride/metrics.py.
// Low sample counts use the same source behavior, not an invented filter fallback.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t32.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "numerics/filter.hpp"
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
    const auto input = bt::read_f64(bt::fixture("metrics/filter_input.f64"));
    const auto filtered = bike::numerics::legacy_ride_filter(input, 1000.0);
    bt::golden_f64("metrics/filter_expected.f64", filtered);
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T32
ctest --test-dir build/native-release -R '^migration_T32$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T32` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T32` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Add bt::read_f64(path) to the bounded binary-array IO layer with documented endian/length contract; production exports may reuse it.
  - [ ] 2. Port legacy leg loop, pedal recovery/weld mechanics, rider support, virtual rider, crash/wheelie handling, momentum/energy observations and recorder reductions assigned to this task.
  - [ ] 3. Port scipy.signal butter/filtfilt behavior from ride metrics: coefficient design, padding, initial conditions and forward/backward ordering. Preserve small-input errors/fallbacks exactly.
  - [ ] 4. Preserve speed/effort/brake control semantics and missing telemetry channels. No simplification to a single default rider is allowed.
  - [ ] 5. Cross-check metric integrals and event boundaries independently of plot output.

### Mandatory additional regression cases

- [ ] Filter impulse/constant/short-vector tests and captured metric series.
- [ ] Leg kinematics and crank split consistency.
- [ ] Crash, wheelie and termination boundaries.
- [ ] Legacy metrics do not affect authoritative stepping.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T32$' --no-tests=error --output-on-failure
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
git add -- native/src/legacy/controls.cpp native/src/legacy/rider.cpp native/src/legacy/legs.cpp native/src/legacy/metrics.cpp native/src/numerics/filter.hpp native/src/numerics/filter.cpp native/tests/migration/test_t32.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T32 port remaining legacy rider/control/measurement behavior"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T33: Port suspension playground and stand controllers

**Dependencies:** T17, T32

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/legacy/playground.hpp` |
| Create | `native/src/legacy/playground.cpp` |
| Create | `native/src/legacy/suspension_controller.cpp` |
| Create | `native/src/legacy/static_equilibrium.cpp` |
| Create | `native/src/legacy/telemetry.cpp` |
| Create | `native/tests/migration/test_t33.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/controllers.py:1-55`
- `src/bike_sim/sim/playground.py:1-398`

Declaration-level card: `migration/cards/T33.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T17` / `migration/api-index.md#t17`: Port standard, stand, playground and legacy MJCF exports.
- `T32` / `migration/api-index.md#t32`: Port remaining legacy rider/control/measurement behavior.

**Produces:**

```cpp
namespace bike::legacy {
struct Playground;
std::unique_ptr<Playground> make_playground();
void step(Playground& playground);
void reset(Playground& playground);
}
// The GUI adapter is T48; this task delivers the testable non-windowed model/controller.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t33.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "legacy/playground.hpp"
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
    auto playground = bike::legacy::make_playground();
    bike::legacy::step(*playground);
    bike::legacy::reset(*playground);
    bt::require(playground != nullptr, "playground controller lifecycle");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T33
ctest --test-dir build/native-release -R '^migration_T33$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T33` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T33` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port sim/playground.py core, controllers.py, equilibrium.py and telemetry.py; leave window management to the frontend task.
  - [ ] 2. Preserve static equilibrium, compression/rebound controls, damper presets/click adjustments, reset and model switching.
  - [ ] 3. Publish an immutable telemetry view with the existing labels/units and data needed by playground HUD and camera controls.
  - [ ] 4. Remove ensure_macos_mjpython from the native application design; Cocoa/GLFW main-thread ownership is handled by the native entrypoint.
  - [ ] 5. Keep playground construction usable in a headless test.

### Mandatory additional regression cases

- [ ] Static equilibrium, model switch, damper change and reset.
- [ ] A controller step produces the same captured telemetry prefix.
- [ ] No GUI dependency is loaded by the controller-only test.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T33$' --no-tests=error --output-on-failure
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
git add -- native/src/legacy/playground.hpp native/src/legacy/playground.cpp native/src/legacy/suspension_controller.cpp native/src/legacy/static_equilibrium.cpp native/src/legacy/telemetry.cpp native/tests/migration/test_t33.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T33 port suspension playground and stand controllers"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
