# Separate reusable C++ cores from Python bindings Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T07 through T11 with source-backed parity and independently reviewed test cycles.

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
| T07 | Separate typed writer construction from nanobind parsing | T03, T04, T05 |
| T08 | Remove binding types from rider and drivetrain state wires | T07 |
| T09 | Extract owned ride runtime and bootstrap into a pure C++ API | T07, T08 |
| T10 | Make the research runtime consume only native owned records | T09 |
| T11 | Separate native samples and column pieces from their Python boxers | T09, T10 |

## Task T07: Separate typed writer construction from nanobind parsing

**Dependencies:** T03, T04, T05

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/writer_config.hpp` |
| Create | `native/src/writer_config.cpp` |
| Modify | `native/src/config.hpp` |
| Modify | `native/src/config_types.hpp` |
| Modify | `native/src/stepper.hpp` |
| Modify | `native/src/stepper.cpp` |
| Modify | `native/src/binding.cpp` |
| Create | `native/tests/migration/test_t07.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/config.hpp:1-746`
- `native/src/config_types.hpp:1-368`
- `native/src/stepper.hpp:1-507`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T03` / `migration/api-index.md#t03`: Build the application without finding Python or wheel directories.
- `T04` / `migration/api-index.md#t04`: Preserve numerical providers and fatal-engine containment.
- `T05` / `migration/api-index.md#t05`: Create strict lossless data and JSON boundaries.

**Produces:**

```cpp
namespace nativecfg {
struct WriterConfig; // optional typed sections for the current nine writer families
WriterConfig decode(const bike::io::Value& value);
}
// Add, without changing the owned Stepper lifetime:
// Stepper(const std::string& mjb_path, const nativecfg::WriterConfig& config);
// Adapter-only nanobind constructors delegate to this constructor.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t07.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "writer_config.hpp"
#include "stepper.hpp"
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
    const auto raw = bike::io::parse_json(R"({"schema":2})");
    const auto cfg = nativecfg::decode(raw);
    Stepper stepper(bt::fixture("models/minimal.mjb").string(), cfg);
    bt::require(stepper.qpos().size() > 0, "standalone typed constructor");
    bt::invalid([] { nativecfg::decode(bike::io::parse_json(R"({"schema":2,"unexpected":0})")); });
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T07
ctest --test-dir build/native-release -R '^migration_T07$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T07` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T07` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Inventory every parser in native/src/config.hpp, including schema-1 migration semantics and schema-2 exact-key rules. Move the owned typed result into writer_config.hpp; no live nanobind object is permitted in it.
  - [ ] 2. Keep existing drivetrain/rider config structs and writer constructors. Decode JSON/Value or optional adapter inputs into the same typed form and invoke one shared validation implementation.
  - [ ] 3. Move all nb::handle/dict constructors, exception translators and ndarray boxing to binding-only translation units. Make stepper.hpp independently compilable without Python/nanobind headers.
  - [ ] 4. Do not replace hand-coded reduction/math kernels, writer compute order, force accumulation, snapshot contents or transaction staging. This task changes the boundary, not the equations.
  - [ ] 5. Compare both the optional binding path and direct C++ construction on the same model, config, state and force inputs, including allocation failure controls.

### Mandatory additional regression cases

- [ ] Empty writer configuration remains valid for bare stepping; unknown sections reject.
- [ ] Schema 1 and 2 fixtures decode to identical effective writer inputs where documented.
- [ ] Public include closure for Stepper has zero Python dependencies.
- [ ] Invalid config cannot partially install writers or mutate a previously valid owner.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T07$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/writer_config.hpp native/src/writer_config.cpp native/src/config.hpp native/src/config_types.hpp native/src/stepper.hpp native/src/stepper.cpp native/src/binding.cpp native/tests/migration/test_t07.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T07 separate typed writer construction from nanobind parsing"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T08: Remove binding types from rider and drivetrain state wires

**Dependencies:** T07

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Modify | `native/src/rider/attachment_wire.hpp` |
| Modify | `native/src/rider/intent_wire.hpp` |
| Modify | `native/src/rider/spindle_wire.hpp` |
| Modify | `native/src/rider/rider_contact_binding.hpp` |
| Modify | `native/src/drivetrain/drive_binding.hpp` |
| Create | `native/src/rider/state_codec.hpp` |
| Create | `native/src/rider/state_codec.cpp` |
| Create | `native/src/drivetrain/state_codec.hpp` |
| Create | `native/src/drivetrain/state_codec.cpp` |
| Create | `native/tests/migration/test_t08.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/drivetrain/drive_binding.hpp:1-13`
- `native/src/rider/attachment_wire.hpp:1-84`
- `native/src/rider/intent_wire.hpp:1-41`
- `native/src/writers/drivetrain.hpp:1-554`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T07` / `migration/api-index.md#t07`: Separate typed writer construction from nanobind parsing.

**Produces:**

```cpp
namespace rider {
SpindleConfig decode_spindle_config(const bike::io::Value& value);
}
namespace drivetrain {
DriveSnapshot decode_drive_snapshot(const bike::io::Value& value);
bike::io::Value encode_drive_snapshot(const DriveSnapshot& state);
}
// SpindleConfig already lives in rider/spindle_controller.hpp.
// DriveSnapshot already lives in writers/drivetrain.hpp; reuse it.
// Binding-only decoder declarations must not enter the pure core include graph.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t08.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "drivetrain/state_codec.hpp"
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
    const auto value = bike::io::parse_json(bt::text(bt::fixture("state/drive.json")));
    const auto state = drivetrain::decode_drive_snapshot(value);
    bt::golden_text("state/drive.canonical.json",
      bike::io::canonical_json(drivetrain::encode_drive_snapshot(state)));
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T08
ctest --test-dir build/native-release -R '^migration_T08$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T08` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T08` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Audit each binding header before changing ownership. Reuse DriveSnapshot in writers/drivetrain.hpp and RiderContactsState/Probe in the existing pure writer headers. Relocate only declarations genuinely owned by a binding header; keep existing field names and optional-value distinctions and do not duplicate types.
  - [ ] 2. Provide cold-path Value codecs for config/bootstrap/replay only. Existing hot paths keep typed arrays/spans and fixed records.
  - [ ] 3. Preserve read-only span ownership, alias validation, dimensions, state labels, and restoration atomicity. State inspection may not advance a controller or normalize away signed zero.
  - [ ] 4. Make adapters thin: convert input, invoke core, box output. Delete duplicate validation branches only after both adapters and direct tests prove the same domain errors.
  - [ ] 5. Test copy/move lifetimes and independent runtime instances; never introduce shared mutable scratch to reduce allocation.

### Mandatory additional regression cases

- [ ] Round-trip drive, spindle, contact, attachment and intent state, including absent optional state.
- [ ] Bad array width/label/order rejects before commit.
- [ ] No core header transitively includes a *_binding.hpp merely to obtain a data type.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T08$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/rider/attachment_wire.hpp native/src/rider/intent_wire.hpp native/src/rider/spindle_wire.hpp native/src/rider/rider_contact_binding.hpp native/src/drivetrain/drive_binding.hpp native/src/rider/state_codec.hpp native/src/rider/state_codec.cpp native/src/drivetrain/state_codec.hpp native/src/drivetrain/state_codec.cpp native/tests/migration/test_t08.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T08 remove binding types from rider and drivetrain state wires"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T09: Extract owned ride runtime and bootstrap into a pure C++ API

**Dependencies:** T07, T08

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/runtime/runtime.hpp` |
| Create | `native/src/runtime/runtime.cpp` |
| Modify | `native/src/runtime/runtime_binding.hpp` |
| Modify | `native/src/runtime/runtime_binding.cpp` |
| Modify | `native/src/runtime/config.hpp` |
| Modify | `native/src/runtime/bootstrap.hpp` |
| Create | `native/src/runtime/bootstrap.cpp` |
| Create | `native/src/runtime/bootstrap_io.hpp` |
| Create | `native/src/runtime/bootstrap_io.cpp` |
| Modify | `native/src/stepper.hpp` |
| Modify | `native/src/stepper.cpp` |
| Create | `native/tests/migration/test_t09.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/runtime/bootstrap.hpp:1-86`
- `native/src/runtime/config.hpp:1-78`
- `native/src/runtime/runtime_binding.cpp:1-1549`
- `native/src/runtime/runtime_binding.hpp:1-156`
- `src/bike_sim/native/runtime.py:1-232`

Declaration-level card: `migration/cards/T09.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T07` / `migration/api-index.md#t07`: Separate typed writer construction from nanobind parsing.
- `T08` / `migration/api-index.md#t08`: Remove binding types from rider and drivetrain state wires.

**Produces:**

```cpp
namespace runtime {
struct ModelImage { std::vector<std::byte> mjb; std::string sha256; };
struct RuntimeBootstrap {
  ModelImage model;
  RuntimeConfig config;
  BootstrapState state;
};
RuntimeBootstrap read_bootstrap(const std::filesystem::path& directory);
// Replace binding-shaped constructor/method inputs in NativeRideRuntime:
// explicit NativeRideRuntime(RuntimeBootstrap bootstrap);
// AdvanceResult advance(int64_t target_step, const RideControl& control,
//   double front, double rear, std::optional<double> wall_budget_s);
}
// Existing RuntimeSnapshot, SampleBatch and owned episode types are retained.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t09.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "runtime/runtime.hpp"
#include "runtime/bootstrap_io.hpp"
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
    auto seed = runtime::read_bootstrap(bt::fixture("bootstrap/welded"));
    runtime::NativeRideRuntime run(std::move(seed));
    const auto initial = run.snapshot();
    run.reset();
    const auto reset = run.snapshot();
    bt::require(reset.step == 0 && reset.generation == initial.generation + 1,
                "reset generation and step");
    bt::golden_f64("bootstrap/welded/integration.f64", reset.integration_state);
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T09
ctest --test-dir build/native-release -R '^migration_T09$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T09` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T09` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Move the core implementation out of runtime_binding.cpp and replace RuntimeConfig/GeometryConfig/BootstrapState nanobind::object members with the existing typed controller, pose, intent, signal and step-state records.
  - [ ] 2. Validate model digest, model dimensions, full integration-state width, named IDs, mutable coefficients, t=0 requirements, measurement cadence and retention mode before committing ownership.
  - [ ] 3. Preserve single-owner exclusion, close idempotence, stale-generation checks, invalid-input atomicity, committed-prefix snapshots, poison semantics and reset inventory.
  - [ ] 4. Maintain the six mutable render-model arrays separately from bootstrap-specific model mutations. Do not assume mjSTATE_INTEGRATION includes mutable mjModel coefficients.
  - [ ] 5. Keep preparation and acknowledgment APIs where throwing consumers could fail; C++ serialization/allocation failures can still happen after the Python boundary is gone.
  - [ ] 6. Add a standalone smoke executable loading the frozen bootstrap. This is the first complete Python-free simulation from a prebuilt fixture, not yet Python-free initialization from TOML.
  - [ ] 7. Read fixture model paths into an owned ModelImage at the IO boundary. Add a validated in-memory model-image Stepper constructor so T25 can initialize natively without writing a transient .mjb. Reuse the same engine load/ownership and byte/digest validation logic as the file constructor.

### Mandatory additional regression cases

- [ ] advance/reset/close/snapshot reentrancy failures match their direct-test contracts.
- [ ] Budget yielding preserves pending work; target_step never rewinds or partially commits.
- [ ] Failure at a chosen physics step exposes exactly the committed prefix.
- [ ] Reset restores hidden tire, battery, freehub, sensor-independent controller and accounting state.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T09$' --no-tests=error --output-on-failure
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
git add -- native/src/runtime/runtime.hpp native/src/runtime/runtime.cpp native/src/runtime/runtime_binding.hpp native/src/runtime/runtime_binding.cpp native/src/runtime/config.hpp native/src/runtime/bootstrap.hpp native/src/runtime/bootstrap.cpp native/src/runtime/bootstrap_io.hpp native/src/runtime/bootstrap_io.cpp native/src/stepper.hpp native/src/stepper.cpp native/tests/migration/test_t09.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T09 extract owned ride runtime and bootstrap into a pure c++ api"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T10: Make the research runtime consume only native owned records

**Dependencies:** T09

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Modify | `native/src/runtime/research.hpp` |
| Modify | `native/src/runtime/research.cpp` |
| Modify | `native/src/runtime/research_binding.cpp` |
| Modify | `native/src/runtime/research_binding.hpp` |
| Create | `native/src/runtime/research_io.hpp` |
| Create | `native/src/runtime/research_io.cpp` |
| Create | `native/tests/migration/test_t10.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/runtime/research.cpp:1-550`
- `native/src/runtime/research.hpp:1-135`
- `src/bike_sim/native/research.py:1-506`
### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T09` / `migration/api-index.md#t09`: Extract owned ride runtime and bootstrap into a pure C++ API.

**Produces:**

```cpp
namespace runtime {
struct ResearchBootstrap {
  RuntimeBootstrap ride;
  ResearchConfig config;
  NoiseTape tape;
  SensorState startup;
};
ResearchBootstrap read_research_bootstrap(const std::filesystem::path& directory);
std::unique_ptr<NativeResearchRuntime> make_research(ResearchBootstrap bootstrap);
}
// Existing begin_control/advance_control/acknowledge_control semantics are retained.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t10.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "runtime/research.hpp"
#include "runtime/research_io.hpp"
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
    auto run = runtime::make_research(
      runtime::read_research_bootstrap(bt::fixture("research/delayed_bootstrap")));
    const auto s = run->snapshot();
    bt::require(s.step == 0, "research starts at t=0");
    run->close(); bt::require(run->closed(), "closed research owner");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T10
ctest --test-dir build/native-release -R '^migration_T10$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T10` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T10` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Remove the runtime_binding.hpp include from research.hpp and use the new pure runtime.hpp. Retain already-native sensors/programs/research_metrics/accounting logic.
  - [ ] 2. Convert warning emission, metadata and status serialization to C++ sinks. A warning sink must not execute during a marked realtime section.
  - [ ] 3. Preserve separate requested/applied commands, delayed controls, observation sequence IDs, pending accepted window, completed-control prefix and physical failure suffix.
  - [ ] 4. Keep begin-control exactly once and acknowledgment after successful result consumption. An allocation failure during delivery must not issue an extra policy action or advance an extra interval.
  - [ ] 5. Verify standalone research can run from the frozen research bootstrap without importing a Python reference environment. New startup construction remains T25/T34.

### Mandatory additional regression cases

- [ ] Control periods larger than presentation debt caps; yield before/after observation boundaries.
- [ ] Stop drains only an already accepted window.
- [ ] reset with a noise tape restores correct startup sensor state without double-consuming a sample.
- [ ] Engine errors do not fabricate successful transitions.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T10$' --no-tests=error --output-on-failure
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
git add -- native/src/runtime/research.hpp native/src/runtime/research.cpp native/src/runtime/research_binding.cpp native/src/runtime/research_binding.hpp native/src/runtime/research_io.hpp native/src/runtime/research_io.cpp native/tests/migration/test_t10.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T10 make the research runtime consume only native owned records"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T11: Separate native samples and column pieces from their Python boxers

**Dependencies:** T09, T10

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Modify | `native/src/runtime/encode.hpp` |
| Modify | `native/src/runtime/encode.cpp` |
| Create | `native/src/runtime/encode_binding.cpp` |
| Modify | `native/src/runtime/samples_binding.cpp` |
| Modify | `native/src/runtime/provenance.hpp` |
| Modify | `native/src/runtime/provenance.cpp` |
| Create | `native/tests/migration/test_t11.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/runtime/encode.cpp:1-1953`
- `native/src/runtime/encode.hpp:1-86`
- `native/src/runtime/samples.hpp:1-125`
- `src/bike_sim/sim/ride/period_buffer.py:1-342`
- `src/bike_sim/sim/ride/physical_samples.py:1-130`

Declaration-level card: `migration/cards/T11.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T09` / `migration/api-index.md#t09`: Extract owned ride runtime and bootstrap into a pure C++ API.
- `T10` / `migration/api-index.md#t10`: Make the research runtime consume only native owned records.

**Produces:**

```cpp
namespace runtime {
struct EncodedTable { std::int64_t rows; std::map<std::string, Piece> columns; };
EncodePlan parse_encode_plan(const bike::io::Value& plan);
EncodedTable encode_table_native(const OwnedNativeEpisode& episode,
  const EncodePlan& plan, std::int64_t start, std::int64_t limit);
}
// Piece remains move-only; its member_slots must not alias another Piece's children.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t11.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "runtime/encode.hpp"
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
    runtime::Piece first;
    first.kind = runtime::Piece::Kind::scalar_d;
    first.d = {-0.0, 1.25};
    auto second = std::move(first);
    bt::require(second.d.size() == 2 && std::signbit(second.d.front()),
                "move-only piece preserves binary64");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T11
ctest --test-dir build/native-release -R '^migration_T11$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T11` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T11` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Reuse the existing encoder algorithm and Piece layout. Split plan parsing/emission from nanobind boxing so DuckDB ingestion can consume C++ buffers directly.
  - [ ] 2. Return native records for applied-command differences and derived transitions. Preserve chunk start seeding and event boundary identities.
  - [ ] 3. Replace provenance nb::dict results with a pure struct/Value and keep binding conversion optional.
  - [ ] 4. Test nested Piece moves, map/list/struct offsets, optional parent validity and all-valid masks. Never reintroduce per-row Python or stringify all values.
  - [ ] 5. After T07-T11, bike_core must compile with bindings OFF and have no Python/nanobind symbols in its transitive headers or linked dependency closure. The optional oracle adapter is the only remaining native Python bridge.

### Mandatory additional regression cases

- [ ] One full chunk equals arbitrary chunk partitions.
- [ ] Sealed episode buffers survive runtime reset/close.
- [ ] Nested child pointers remain valid after moves.
- [ ] Null parent, absent child and explicit-null child remain distinct.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T11$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/runtime/encode.hpp native/src/runtime/encode.cpp native/src/runtime/encode_binding.cpp native/src/runtime/samples_binding.cpp native/src/runtime/provenance.hpp native/src/runtime/provenance.cpp native/tests/migration/test_t11.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T11 separate native samples and column pieces from their python boxers"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
