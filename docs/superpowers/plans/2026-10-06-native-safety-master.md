# Native Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:
> executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make native failures recoverable, malformed inputs consistently rejected, state changes atomic, and
verification trustworthy.

**Architecture:** Five subsystem plans share one behavioral specification and explicit interfaces. Establish trustworthy
gates first, then implement independent safety changes, then remove runtime allocation obstacles and repair the
benchmark. Keep Python/native behavior synchronized through shared contract tests.

**Tech Stack:** Python through uv, pytest, C++23, CMake/CTest, nanobind, MuJoCo 3.12.0, Apple Accelerate,
clang-tidy/CSA/cppcheck, ASan/UBSan/RTSan/LLVM coverage.

**Spec:** [Native safety specification](../specs/2026-10-06-native-safety.md).

## Global Constraints

- Use `uv` for every Python command.
- Retain Python >=3.13, CMake >=3.24, and nanobind >=3.1.0; native platform scope remains Apple arm64/Accelerate.
- Preserve valid-input numerical parity, including operation order, intentional saturation, and documented legacy
  behavior.
- Reject malformed inputs consistently in Python and C++; update both implementations in the same task.
- Compile first-party native code as ISO C++23 with extensions disabled.
- Use the existing MuJoCo 3.12.0 dependency; fail configuration if the error-interception ABI is unavailable.
- Keep alpha analyzer findings report-only; report analyzer execution failures separately from findings.
- Run `native` for native code or shell tooling changes; also run `full` for physics or realtime changes.
- Preserve existing AGENTS.md behavioral-stability requirements.
- Keep allocation, Python boxing, logging, and exception construction outside marked realtime regions.
- Make each task independently reviewable; record exact commands, selected binary, tool health, and test results.

---

## Read before execution

Read the specification and [agent rules](../../agents/native-engineering.md). The user approved this **plan and agent
guidance**, not a claim that its proposed production checks or fixes are already installed. This document contains 20
reviewable implementation tasks. Execute focused regressions before broad gates; commit each completed task separately
when using the execution skill. Preserve unrelated working-tree changes.

Baseline evidence: the audited checkout's native profile passed 697 tests. The supplied review's immediate ndarray UAF,
truthy bool, unordered-set acceptance, discarded-result suppression of throws, and blanket noexcept arguments were
disproved. Other issues are internal C++ preconditions rather than exposed Python crashes. The specification's A01–A33
ledger records these distinctions and additional issues.

## File and module boundaries

| Plan                                                                | Ownership                                                                                               | New modules/interfaces                                                                                                             |
|---------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------|
| [Verification](2026-10-06-native-safety-verification.md)            | CMake source manifest/context; sweep orchestration; selected extension loading; coverage/profile health | `tools/native_checks.py`, `tests/reference/native_loader.py`, `native/tests/test_contracts.cpp`, `tools/check_native_contracts.py` |
| [FFI and validation](2026-10-06-native-safety-validation.md)        | Config projection, strict wire readers, shared domains, ownership, numeric clocks                       | `native/src/config_types.hpp`, `validation.hpp`, `binding_readers.hpp`, `binding_arrays.hpp`, schema-2 projection                  |
| [Errors and transactions](2026-10-06-native-safety-transactions.md) | Fatal engine boundary; staged transmission, compute/probe, and settlement                               | `native/src/engine_call.hpp/.cpp`, prepared updates and public commit boundary                                                     |
| [C++ contracts](2026-10-06-native-safety-cpp.md)                    | Typed constructor invariants, geometry extents, shared numerical ABI/helpers, header contracts          | `cblas_abi.hpp`, `model_access.hpp`, `numeric_norm.hpp`, direct C++ contract cases                                                 |
| [Runtime and benchmark](2026-10-06-native-safety-runtime.md)        | Construction-owned scratch, typed diagnostics, allocation budget, reproducible benchmark                | `writer_types.hpp`, benchmark CMake target, versioned state artifacts                                                              |

Avoid a universal helper header. Wire readers depend on nanobind; numeric validation and config types do not.
Engine-call code owns the pinned private ABI; business logic does not declare private symbols. Shared numerical helpers
record the exact oracle operation they reproduce.

## Order and dependency gates

| Task | Deliverable                                                              | Dependencies | Spec/evidence coverage     |
|------|--------------------------------------------------------------------------|--------------|----------------------------|
| V1   | Complete source selection and fail-closed sweeps                         | None         | S1; A20,A21,A27,A33        |
| V2   | Portable header analysis, ISO build, scoped suppressions, hardening      | V1           | S1,S7; A21,A25,A26,A27     |
| V3   | Selected-artifact loader, native/full preflight, C++ contract executable | V1,V2        | S2; A30                    |
| V4   | Coverage selection and contract-policy gates                             | V3           | S1,S2; A30,A33             |
| F1   | Typed configs, strict wire readers, schema migration                     | V3           | S3; A16,A18,A31            |
| F2   | Shared numeric and topology domains                                      | F1           | S3,S4; A14,A15,A16,A28,A31 |
| F3   | Runtime/state types and relational restore                               | F1,F2        | S3,S4,S6; A15,A18,A32      |
| F4   | Ownership and private hook contract                                      | V3,F1        | S7; A02,A13,A19            |
| F5   | Tire rounding/range and live cruise timestep                             | F2,F3        | S4; A17,A28                |
| E1   | Recoverable pinned engine boundary                                       | V3           | S5; A01                    |
| E2   | Staged transmission and geometry update                                  | F2,F3,E1     | S6; A10                    |
| E3   | Atomic compute/probe and Python boxing                                   | E2,F3,F4     | S6; A11                    |
| E4   | Exactly-once battery settlement under allocation failure                 | E3           | S6; A12                    |
| C1   | Direct typed constructor and lifecycle contracts                         | F1,F2        | S3,S7; A09,A25             |
| C2   | Checked geometry/profile extents and read-only spans                     | C1           | S7; A05,A06,A07,A08        |
| C3   | Shared ABI/numeric/model helpers                                         | C2           | S7; A22                    |
| C4   | Header/API hygiene, local enums and attributes                           | C3,F4        | S7; A16,A24,A25            |
| R1   | Allocation-free owning core with typed staged diagnostics                | E4,C4,F5     | S8; A23,A24                |
| R2   | Safe built benchmark and malformed-input tests                           | V3,C3,E1     | S8; A03,A04,A29            |
| R3   | Compatible artifact bundle and comparable workload                       | R2           | S8; A29                    |

E1 may proceed while F1 is developed; it is urgent and independent once V3 exists. Benchmark hardening can proceed
before runtime refactoring. Shared edits to CMake, binding.cpp, config.hpp, and drivetrain.cpp require sequential
integration even if execution uses agents. Use the dependency table rather than assuming all plans are independent.

## Required check policy

| Check                    | Blocking condition                                                       | Evidence retained                                            |
|--------------------------|--------------------------------------------------------------------------|--------------------------------------------------------------|
| Source manifest          | Missing/duplicate/unexpected first-party TU or empty selection           | Canonical expected/selected sets                             |
| Build/dependency context | Missing required headers/libraries/tools or unsupported pinned error ABI | Compiler, SDK, Python/nanobind/MuJoCo locations and versions |
| Stable static analysis   | Any execution failure or unsuppressed stable finding                     | Exit status, full logs, checker/origin for exclusions        |
| Alpha analysis           | Execution failure fails health; findings alone are report-only           | Distinct health/finding statuses and complete logs           |
| Native artifact          | Import differs from selected build or zero native tests collected        | Resolved extension path and test collection counts           |
| C++ contracts            | Constructor, bounds, failure-atomicity, or ownership regression          | CTest output with checks active in Release                   |
| Valid numerical behavior | Valid-input bitwise/golden regression                                    | Native/Python paired results; no unexplained golden edits    |
| Invalid-input behavior   | Acceptance or mismatched class/state rollback                            | Paired negative cases with API/field messages                |
| Sanitizers               | ASan/UBSan error; runtime/preload mismatch                               | Child runtime loaded, selected extension, logs               |
| Realtime core            | Any warm core allocation or RTSan finding in declared regions            | Backend/path matrix, allocations, p50/p95/p99                |
| Coverage                 | Tested/report artifact mismatch or empty profiles                        | Build and binary provenance; line/branch deltas              |

Coverage is a visibility gate, not an invented global percentage target. Require coverage of new failure branches and
report uncovered paths; set a numeric floor only after measuring the repository baseline. Performance measurements
record model, compiler, hardening, backend, warmup, and sample count; avoid asserting an unmeasured universal slowdown
budget.

## Completion and review

- [x] V1–V4 are green on normal, relocated-checkout, and malformed-tool-context controls.
- [x] F1–F5 pass strict Python/native invalid-input matrices and existing valid-input golden tests.
- [x] E1–E4 pass subprocess fatal-error and every-allocation-position rollback/retry tests.
- [x] C1–C4 pass independent-header compilation, Release CTest, and ASan/UBSan contract tests.
- [x] R1 passes allocation/RTSan path matrices; R2–R3 pass safe benchmark/state artifact tests.
- [ ] `native` and `full` pass on final code; selected ASan/UBSan and RTSan builds exercise the changed safety/realtime
  paths. — `native` is green (1966 passed); ASan/UBSan and RTSan builds exercise the changed paths with zero
  sanitizer reports. `full` reports 24 failures, all in pure-Python rider/equilibrium/realtime-gate tests that do
  not import `bike_native`; the identical failures reproduce on the pre-program base `fa7a7f0` (and intermediate
  baselines), so they are pre-existing defects outside this program's scope rather than regressions.
- [x] TESTING.md and agent rules describe installed behavior, and implementation status identifies any remaining planned
  work.
- [x] Review each A01–A33 disposition against code/tests; preserve qualified and rejected findings as such.

Out of scope: broad C++23 feature conversion, new physics laws, changing valid mathematical results, making a Python
tick allocation-free, a new realtime scheduler, and asserting arbitrary foreign `mjData*` ownership from dimensions.
These require separate evidence and design.

## Plan self-review

All S1–S8 requirements and A01–A33 dispositions have task owners in the dependency table. Local document validation
checked all links, identical global constraints, complete/unique task IDs, evidence coverage, and Python snippet syntax.
Implementation snippets describe the proposed interfaces; they have not been compiled as production changes. The
original behavioral-stability text in AGENTS.md is preserved; its native-profile trigger now includes Python native
tooling/projection/tests. Only documentation, the agent-rule pointer, and this profile rule are installed by the
planning change; the check targets, schema2, and runtime fixes remain implementation work.
