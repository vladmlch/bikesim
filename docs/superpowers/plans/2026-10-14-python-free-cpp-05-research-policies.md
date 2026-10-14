# Research construction, policy ABI and lifecycle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T34 through T37 with source-backed parity and independently reviewed test cycles.

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
| T34 | Construct research environments, programs and sensor tapes natively | T25, T19, T10 |
| T35 | Replace Python motor policies with built-ins and a versioned native ABI | T34, T06 |
| T36 | Port policy sessions, pacing boundaries and lifecycle coordination | T35 |
| T37 | Port research metrics, validity and experiment evidence | T34, T36 |

## Task T34: Construct research environments, programs and sensor tapes natively

**Dependencies:** T25, T19, T10

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/research/environment.hpp` |
| Create | `native/src/research/environment.cpp` |
| Create | `native/src/research/configuration.cpp` |
| Create | `native/src/research/program_loader.cpp` |
| Create | `native/src/research/rider_random.cpp` |
| Create | `native/tests/migration/test_t34.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/runtime/programs.hpp:1-26`
- `native/src/runtime/sensors.hpp:1-55`
- `src/bike_sim/native/research.py:1-506`
- `src/bike_sim/sim/research/__init__.py:1-5`
- `src/bike_sim/sim/research/backend.py:1-49`
- `src/bike_sim/sim/research/configuration.py:1-86`
- `src/bike_sim/sim/research/demand.py:1-65`
- `src/bike_sim/sim/research/environment.py:1-524`
- `src/bike_sim/sim/research/observations.py:1-19`
- `src/bike_sim/sim/research/rider_behavior.py:1-40`
- `src/bike_sim/sim/research/rider_program.py:1-119`
- `src/bike_sim/sim/research/sensors.py:1-164`

Declaration-level card: `migration/cards/T34.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T25` / `migration/api-index.md#t25`: Construct standalone native rides from user configuration.
- `T19` / `migration/api-index.md#t19`: Reproduce seeded random streams and distribution transforms.
- `T10` / `migration/api-index.md#t10`: Make the research runtime consume only native owned records.

**Produces:**

```cpp
namespace bike::research {
std::unique_ptr<runtime::NativeResearchRuntime> make_environment(
  const std::filesystem::path& request_toml);
}
// Existing runtime::ResearchConfig, NoiseTape, SensorState, programs and sensor
// kernels remain the runtime contract; construction no longer takes ResearchEnvironment.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t34.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "research/environment.hpp"
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
    auto environment = bike::research::make_environment(
      bt::fixture("requests/research_delay.toml"));
    bt::require(environment->snapshot().step == 0, "startup boundary");
    environment->close();
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T34
ctest --test-dir build/native-release -R '^migration_T34$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T34` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T34` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Replace create_native_research(reference: ResearchEnvironment) with native resolution/model preparation, research integer-step validation, sensor pipeline startup and noise-tape construction.
  - [ ] 2. Port ExperimentConfig, rider randomization, RiderProgram/DemandProgram file loading, posture interpolation, check-policy resolution and scenario generation. Reuse already-native runtime/programs.cpp and sensors.cpp for advancement.
  - [ ] 3. Preserve sensor/control/actuator-delay scheduling, maximum step computation, initial held brake/control state and one startup observation. The first sample must not be generated twice.
  - [ ] 4. Retain the sensor-only observation contract, finite domains, termination/validity/energy-quality classification and configured capture mode.
  - [ ] 5. Verify fixed-seed equality across headless, viewer and batch frontends with nonbinding wall limits.

### Mandatory additional regression cases

- [ ] Initial observation, all-zero delays, nonzero delays, incompatible period ratios and long control windows.
- [ ] Rider random same seed/different seed and invalid range checks.
- [ ] Program boundaries/interpolation and metadata identities.
- [ ] No Python source or precomputed Python bootstrap required.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T34$' --no-tests=error --output-on-failure
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
git add -- native/src/research/environment.hpp native/src/research/environment.cpp native/src/research/configuration.cpp native/src/research/program_loader.cpp native/src/research/rider_random.cpp native/tests/migration/test_t34.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T34 construct research environments, programs and sensor tapes natively"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T35: Replace Python motor policies with built-ins and a versioned native ABI

**Dependencies:** T34, T06

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/research/policy.hpp` |
| Create | `native/src/research/policy.cpp` |
| Create | `native/src/research/builtin_policies.cpp` |
| Create | `native/include/bike/policy_v1.h` |
| Create | `native/src/research/plugin_policy.cpp` |
| Create | `examples/policies/traction_control.cpp` |
| Create | `docs/NATIVE_POLICIES.md` |
| Create | `native/tests/migration/test_t35.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/research/backend.py:1-49`
- `src/bike_sim/sim/research/policies.py:1-176`

Declaration-level card: `migration/cards/T35.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T34` / `migration/api-index.md#t34`: Construct research environments, programs and sensor tapes natively.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.

**Produces:**

```cpp
// Stable plugin entrypoint; only C-compatible scalars/arrays and explicit sizes.
// The host owns observation/control/error memory; no STL object crosses this ABI.
extern "C" const BikePolicyV1* bike_policy_v1(void);
// BikePolicyV1 defines version/struct_size plus create, reset, step and destroy.
// create/reset/act return a status and writes bounded error text; no exception
// crosses the ABI. Observation fields come only from SensorObservation.
namespace bike::research {
std::unique_ptr<Policy> load_policy(std::string_view specification);
}
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t35.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "research/policy.hpp"
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
    auto zero = bike::research::load_policy("builtin:zero");
    bt::require(zero != nullptr, "built-in native policy");
    bt::invalid([] { bike::research::load_policy("unknown_module:factory"); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T35
ctest --test-dir build/native-release -R '^migration_T35$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T35` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T35` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port passthrough, zero, fixed-limit-40 and traction-control policy equations/state from policies.py. Keep known historical built-in module:factory strings as aliases only if their identity is unambiguous.
  - [ ] 2. Define Policy plus the entire C ABI in policy_v1.h, including ABI version, struct_size, optional validity masks, sensor schema version, reset seed and error ownership. Match existing sensor/control fields exactly.
  - [ ] 3. Replace arbitrary Python module:factory with builtin:name or a native plugin specification. Arbitrary external Python source cannot remain executable in a Python-free release; provide explicit migration documentation and port every policy discovered at T01.
  - [ ] 4. Hash plugin binaries/configuration in execution provenance. Replay never imports or executes a recorded policy. Invalid ABI, missing symbol, null callback or thrown exception from a plugin produces a clear policy failure.
  - [ ] 5. Construct/call a policy only at the existing external control boundary. Do not invoke user plugins inside marked zero-allocation/realtime force loops.
  - [ ] 6. Do not claim dynamically loaded native plugins are sandboxed. They carry the same trust requirement as application code; process isolation is a separate feature.

### Mandatory additional regression cases

- [ ] Built-in output/state matches captured policy traces.
- [ ] Counting policy invoked once per accepted external interval even across repeated budget yields.
- [ ] Wrong ABI, load failure, invalid/NaN output and throwing plugin are controlled failures.
- [ ] Policy cannot access privileged ground-truth fields through the public observation ABI.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T35$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/research/policy.hpp native/src/research/policy.cpp native/src/research/builtin_policies.cpp native/include/bike/policy_v1.h native/src/research/plugin_policy.cpp examples/policies/traction_control.cpp docs/NATIVE_POLICIES.md native/tests/migration/test_t35.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T35 replace python motor policies with built-ins and a versioned native abi"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T36: Port policy sessions, pacing boundaries and lifecycle coordination

**Dependencies:** T35

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/research/policy_session.hpp` |
| Create | `native/src/research/policy_session.cpp` |
| Create | `native/src/research/lifecycle.cpp` |
| Create | `native/src/research/exit_codes.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/runtime/research.hpp:1-135`
- `src/bike_sim/sim/research/policy_session.py:1-205`
- `src/bike_sim/sim/research/recording_lifecycle.py:1-199`
- `src/bike_sim/sim/ride/physical_session.py:1-225`

Declaration-level card: `migration/cards/T36.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T35` / `migration/api-index.md#t35`: Replace Python motor policies with built-ins and a versioned native ABI.

**Produces:**

```cpp
namespace bike::research {
class PolicySession {
public:
  PolicySession(std::unique_ptr<runtime::NativeResearchRuntime> environment,
                std::unique_ptr<Policy> policy);
  ~PolicySession();
  PolicySession(const PolicySession&) = delete;
  PolicySession& operator=(const PolicySession&) = delete;
  void begin_advance();
  std::optional<runtime::ResearchTransition> advance_pending(
    std::optional<double> wall_budget_s, std::optional<std::int64_t> target_step);
  void request_stop();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
// Constructor takes an owned environment and Policy; pending command ownership
// survives pauses/yields and is cleared only at its documented boundary.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -R migration_T36 --output-on-failure --no-tests=error
uv run --frozen --group native python -m pytest tests/reference/test_research_frontend.py tests/reference/test_native_research_frontend.py tests/reference/test_frontend_windows.py -q
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T36
ctest --test-dir build/native-release -R '^migration_T36$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T36` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T36` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port PolicySession, load_policy metadata, synchronous/run-until APIs and exact episode_exit_code classifications.
  - [ ] 2. Keep operator pause/brake/reset/stop requests separate from accepted controls. Requests become effective at the source-defined external control boundary.
  - [ ] 3. Stopping on close finishes only an already accepted command. Repeated advance_pending calls never request another policy action, truncate a long window or flush a synthetic transition.
  - [ ] 4. Use actual completed physics steps to decrement pacing debt. Pause/reset/time-scale changes rebase debt without changing dt or sensor/controller periods.
  - [ ] 5. Preserve primary simulation/policy exceptions and attach secondary close/publication failures as structured diagnostics.

### Mandatory additional regression cases

- [ ] Instrumented policy call count for yield, pause, stop and reset.
- [ ] Control period longer than debt cap, partial window at engine failure and time budget exhaustion.
- [ ] All current research frontend exit-code tests.
- [ ] No successful transition for a failed physical suffix.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T36$' --no-tests=error --output-on-failure
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
git add -- native/src/research/policy_session.hpp native/src/research/policy_session.cpp native/src/research/lifecycle.cpp native/src/research/exit_codes.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T36 port policy sessions, pacing boundaries and lifecycle coordination"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T37: Port research metrics, validity and experiment evidence

**Dependencies:** T34, T36

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/research/metrics.cpp` |
| Create | `native/src/research/quality.cpp` |
| Create | `native/src/research/evidence.cpp` |
| Create | `native/src/research/metadata.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/research/environment.py:1-524`
- `src/bike_sim/sim/research/metrics.py:1-36`
- `src/bike_sim/sim/research/quality.py:1-34`
- `src/bike_sim/sim/research/validity.py:1-41`
- `src/bike_sim/sim/ride/model_status.py:1-45`

Declaration-level card: `migration/cards/T37.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T34` / `migration/api-index.md#t34`: Construct research environments, programs and sensor tapes natively.
- `T36` / `migration/api-index.md#t36`: Port policy sessions, pacing boundaries and lifecycle coordination.

**Produces:**

```cpp
namespace bike::research {
bike::io::Value summarize(const runtime::OwnedResearchEpisode& episode);
}
// Reuse runtime/research_metrics.cpp and status.cpp where equivalent;
// this API assembles complete frontend/recording metadata without Python views.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Use this exact command/process contract as the first native regression and add the matching CTest/process assertions. The mandatory cases below are additional required tests, not optional follow-ups.

```bash
ctest --test-dir build/native-release -R migration_T37 --output-on-failure --no-tests=error
uv run --frozen --group native python -m pytest tests/reference/test_research_capture.py tests/reference/test_research_recording_lifecycle.py tests/reference/test_rider_information_boundary.py -q
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T37
ctest --test-dir build/native-release -R '^migration_T37$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T37` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T37` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port summary assembly, energy quality, model/channel violations, running metrics and research asset recipes from the Python environment/view adapter.
  - [ ] 2. Keep strict versus diagnostic policies independent from recording. Diagnostic continuation retains sticky invalid status and the first failure; it does not grant training/replay validity.
  - [ ] 3. Retain separate completed-control and physical/accounted extents, terminal cause, achieved/requested timing and observation/request IDs.
  - [ ] 4. Evidence must be computable from sealed owned episodes after live runtime destruction. Missing fields remain missing, not zero-filled.
  - [ ] 5. Compare source metadata while allowing only explicitly volatile fields such as new run UUID/wall timestamps to differ.

### Mandatory additional regression cases

- [ ] No-record mode still computes required live checks/metrics but does not grow optional histories.
- [ ] Correct first failure and terminal reason under strict/diagnostic runs.
- [ ] Summary stability after reset/close.
- [ ] Research observation boundary remains sensor-only.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T37$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/research/metrics.cpp native/src/research/quality.cpp native/src/research/evidence.cpp native/src/research/metadata.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T37 port research metrics, validity and experiment evidence"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
