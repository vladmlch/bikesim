# A4 Native Interval Accounting Implementation Plan

> **For agentic workers:** Use the source task map below; unchecked entries are not verified gates. The user explicitly prohibited executing the project, tests, analyzers, or compilation in this delivery.

**Goal:** Account every captured native interval and publish owned schema-2 samples, work history, model status, and native recorder columns.

**Architecture:** PhysicalStep remains the sole physics owner. A native PeriodAccounting owner consumes its RawStep values, stages a whole period before publication, and retains unacknowledged output independently of the live engine. Python only boxes owned values at API boundaries.

**Tech Stack:** Existing C++23/Stepper/nanobind implementation and Python dataclass contracts; no new dependency.

**Spec:** `docs/superpowers/specs/2026-10-08-native-visual-runtime-design.md`, sections 3.2/3.3; A4 in `2026-10-08-native-visual-runtime-core.md`.

## Global Constraints

- Do not run or compile anything from the project. Runtime, test, sanitizer, and compiler evidence remains pending.
- Preserve schema-2 keys, physical interval ordering, every-step accounting, and the existing diagnostic/strict distinction.
- Display/budget/target boundaries do not implicitly flush a partial period.
- Account and publish the entire period before strict rejection, using the originating interval end time.
- Python boxing must precede acknowledgement; failed boxing leaves native rows available.
- Reset first closes the outgoing period; strict rejection preserves that generation and its drainable output. Successful reset clears accounting and increments generation. Retained samples, batches, snapshots, and columns must survive reset/close.
- Source-supported review is not executed verification; do not check off the A4 phase gate.

## Task 1: Owned samples, period capture, and status policies

**Files:** Create `native/src/runtime/{samples,period_buffer,status}.{hpp,cpp}`.

**Interfaces:** `PeriodBuffer::push(RawStep)`, `WorkHistory::add(const PhysicalSampleData&)`, `ModelStatus::observe`, `ReferenceMonitor::observe`; immutable `SamplePtr` and native `SampleColumns`.

- [ ] Define owned sample fields, schema-2 export, flattened native columns and recorder aliases.
- [ ] Enforce capacity, consecutive IDs and continuous times across period clears.
- [ ] Port the attachment/work budgets and model applicability/energy-quality policies with unchanged thresholds.
- [ ] Add independent regressions for opposing signed work, zero-source budget, missing/duplicate IDs, and malformed contact evidence.

## Task 2: Complete period accounting

**Files:** Create `native/src/runtime/accounting.{hpp,cpp}`; extend diagnostic publication in `rider/spindle_controller.hpp` and `writers/rider_contacts.hpp`.

**Interfaces:** `PeriodAccounting::push`, `flush`, `prepare_samples`, `acknowledge_samples`, `state_wire`, and `recorded_columns`.

- [ ] Reconstruct attachment and solved effort evidence from every incoming capture, never from endpoint scratch.
- [ ] Accumulate all six work lanes, losses, external/electrical work, component history and airtime.
- [ ] Add integral constraint rejection only to the final interval of each explicitly closed period.
- [ ] Build mandatory channels on every row and detail on decimated or closing rows; retain latest full sample.
- [ ] Stage history/status/rows/recorder blocks and diagnostic updates before committing publication.

## Task 3: Runtime and binding integration

**Files:** Modify `runtime_binding.{hpp,cpp}`, add `samples_binding.{hpp,cpp}`, update `native/CMakeLists.txt`, `src/bike_sim/native/{runtime,contracts}.py`.

**Interfaces:** `flush() -> None`, `drain_samples() -> SampleBatch`, `reset() -> FrameSnapshot`, owned latest sample/view/first failure, recorder columns and accounting state.

- [ ] Replace discarded RawSteps with native accounting; keep partial periods across budget/target calls.
- [ ] Cache the last committed integration vector for failure-safe snapshots.
- [ ] Publish the committed prefix on engine failure without replacing the original engine exception.
- [ ] Restore through the supported Stepper recovery path and clear output on reset.
- [ ] Register distinct native reference errors and translate them to the existing Python `InvalidReferenceRun`.
- [ ] Use prepare/box/ack extraction; retain rows after Python constructor/export failure.
- [ ] Recursively freeze Python values and own all array storage.

## Task 4: Regression source and handoff

**Files:** Add `tests/reference/test_native_runtime_{accounting,rollout,ownership}.py`; update the existing execution audit.

- [ ] Add complete-row comparison for a 41-step nonaligned tail and identical explicit flush schedules.
- [ ] Add deterministic chunking, decimation-independent history and an 800-step golden episode.
- [ ] Add strict/diagnostic first-failure equivalence, failed boxing/retry, generation-safe acknowledgement, retained objects and bootstrap lifetime tests.
- [ ] Add process-isolated fatal-engine committed-prefix/recovery coverage using existing native test hooks.
- [ ] Inspect edited source, package the changed files and full supplied project, and record that all execution gates are intentionally unrun.

## Verification (not executed)

The existing A4 phase commands in the core plan, including the selected Release build, focused tests, `full`, and the documented ASan/UBSan artifact recipe, remain required before an unqualified verified-completion claim.


## Delivery status - 2026-10-09

The source files and regression files named in Tasks 1-4 have been added or
updated, and the source was compared with the corresponding Python ownership,
accounting and flush paths. The existing audit now contains an explicitly
source-only A4 entry. Checkboxes deliberately remain open: neither source
inspection nor writing a test demonstrates its runtime completion gate.

No project code, tests, compiler, configuration step, analyzer or sanitizer was
executed. The final A4 gate in `2026-10-08-native-visual-runtime-core.md` remains
unchecked. See the audit's A4 section for the file/scenario inventory and the
unverified DGELSD-versus-Python-pinv numerical comparison.
