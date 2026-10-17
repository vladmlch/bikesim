# Linux and macOS portability - master Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. If those skills are not installed in the execution environment, follow the same task/review gates manually; do not claim to have invoked unavailable skills.

**Goal:** Add Linux compilation, runtime, and development support without removing or weakening macOS support.

**Architecture:** Retain the shared Python/C++ runtime and isolate platform SDK, ABI, linker, numerical-provider and toolchain concerns. Deliver source-first native development/offline workflows and require explicit Linux plus macOS qualification before advertising support.

**Tech Stack:** Python/uv, NumPy, MuJoCo 3.12.0, nanobind, CMake, C++23, Accelerate on macOS, the selected NumPy OpenBLAS provider on Linux, pytest, CTest, LLVM/GCC analysis tools.

**Spec:** `docs/superpowers/specs/2026-10-14-linux-macos-portability-design.md`. Read the source audit and master plan as well as this file.

## Global Constraints

- Add Linux support; retain macOS support and existing macOS behavior.
- Keep Python >=3.13, CMake >=3.24, and strict ISO C++23 without compiler language extensions.
- Keep MuJoCo exactly 3.12.0 for native execution until its private error ABI is deliberately migrated in a separate change.
- Preserve the macOS Accelerate backend, SDK override/fallback behavior, mjpython launcher, and current application commands.
- Preserve same-environment native/Python bitwise tests, exception safety, source/artifact identity, and fail-closed verification.
- Do not promise bitwise-identical results between different operating systems, CPUs, BLAS implementations, or dependency builds.
- Do not change physics, fixed integration periods, controller behavior, recording opt-in/final-only semantics, or replay tolerances for this port.
- Use uv for project Python environments. Never silently regenerate the production lock or upgrade dependencies during a build.
- No implicit native compilation on application import; no silent native-to-Python fallback; no required GUI for headless execution.
- Keep quick/native/full profile meanings; missing required analysis tools remain errors, not successful skips.
- Treat the supplied Repomix export as a source snapshot, not a complete Git checkout or a runnable distribution.
- Native builds are the initial delivery target. Cross-compilation, universal binaries, Windows, and musl are outside this change.

---

## 1. Handoff status and reading order

This is an implementation plan, not an implemented port. The source snapshot was unpacked and audited; the existing Linux configure failure was reproduced. No project implementation source was changed and no native build, simulation suite, macOS session or GUI acceptance was claimed as passing.

Read START-HERE.md, the design spec, the source audit, this master plan, then the assigned work-package/task. task-index.json is the scheduling/ownership interface. The code blocks are concrete proposed test/implementation patterns to adapt against the recovered real checkout; they are not represented as already executed patches. Keep the original uploaded archive unchanged.

The export omits uv.lock, .python-version, .github, binary assets and Git metadata. Recover the actual repository state first. Never interpret a file absent from this export as authorization to delete or recreate its real-checkout counterpart. Exact dependency/tool versions and historical baseline outcomes come from that recovered state, with source comments and declared constraints cross-checked.

## 2. Primary decisions

The initial required Linux target is Ubuntu 24.04 x86_64/glibc with qualified Clang and GCC. The existing macOS Apple Silicon environment remains required, including its actual compiler/SDK/interpreter/numerical backend, not merely a convenient replacement hosted runner. Python 3.13 stays supported alongside the real baseline interpreter. Additional Linux ARM64/macOS Intel support is preserved wherever already established and otherwise remains explicitly unqualified until tested.

Do not rewrite the application. Keep MuJoCo 3.12.0 private ABI, native/Python interfaces, macOS Accelerate, one selected extension, existing error handling, source fingerprints, controller/physics timing and opt-in final-only recording. Numerical compatibility is the highest-risk work: Linux must match its own pinned Python/NumPy oracle at the existing exact boundaries. A generic OpenBLAS link, allclose assertions or native-to-Python fallback is not the solution.

Build metadata and persisted execution identity are separately versioned. Cross-platform recording inspection/playback can work while strict physical replay correctly rejects differing numerical identity. A pure Python wheel is not a native binary distribution; explicit source/editable builds are the initial native installation route on both systems.

## 3. Work packages

| Package | Task IDs | Deliverable | Document |
|---|---|---|---|
| Environment | T01-T03 | Baseline, frozen dependency policy, explicit dev driver | 2026-10-14-linux-macos-01-environment.md |
| Native build | T04-T10 | SDK isolation, exact engine/provider targets, dual ABI adapters, compiler/linker policy | 2026-10-14-linux-macos-02-native-build.md |
| Numerical correctness | T11-T12 | Local exact parity, versioned execution identity and replay diagnostics | 2026-10-14-linux-macos-03-numerics.md |
| Development checks | T13-T17 | Platform context, real analysis, ASan/RTSan, LLVM coverage | 2026-10-14-linux-macos-04-development-tools.md |
| Runtime | T18-T22 | Headless/GUI/workers, recording lifecycle, inspection/playback/checked replay | 2026-10-14-linux-macos-05-runtime.md |
| Delivery | T23-T26 | Source-native install, full/slim offline bundles and guides | 2026-10-14-linux-macos-06-delivery.md |
| Qualification | T27-T30 | Exact baseline comparison, CI, real-host rehearsal and release acceptance | 2026-10-14-linux-macos-07-validation.md |

Each task includes exact files, consumed/produced interfaces, a regression/control, red-test command, implementation instructions/code, verification, a commit and an independent review gate. Some compile-dependent tests cannot run until the build graph exists; the task identifies those integration gates rather than allowing a fake early success.

## 4. Task dependency and ownership index

Task numbers are stable identifiers, not a command to ignore dependencies. In particular, T13 must be developed with T10 rather than after an incompatible context writer has already shipped.

| Task | Owner | Prerequisites | Deliverable |
|---|---|---|---|
| T01 | integration | None | Recover the real checkout and freeze a truthful platform baseline |
| T02 | environment | T01 | Make native dependency resolution consistent and qualify wheels |
| T03 | environment | T01, T02 | Add a cross-platform explicit development driver |
| T04 | native-build | T01, T03 | Isolate pre-project Apple SDK discovery and preserve every macOS control |
| T05 | native-build | T02, T03, T04 | Discover the selected Python dependencies and MuJoCo shared library |
| T06 | numerics | T02, T05 | Qualify the actual NumPy BLAS/LAPACK provider and integer ABI |
| T07 | numerics | T06 | Introduce typed dual-platform CBLAS adapters without changing callers |
| T08 | numerics | T06, T07 | Isolate DGELSD and preserve least-squares workspace semantics |
| T09 | native-build | T04, T05 | Make compiler, architecture and standard-library policies explicit |
| T10 | native-build | T05, T07, T08, T09, T13 | Integrate portable linkage, RPATH and the complete native target graph |
| T11 | numerics | T10 | Qualify every platform-sensitive numerical lowering against the local oracle |
| T12 | runtime | T06, T10, T11 | Version numerical provenance and give replay precise incompatibility errors |
| T13 | native-build | T04, T05, T06, T09 | Version build context without weakening source and toolchain validation |
| T14 | development-tools | T10, T13 | Discover analysis tools and preserve real negative controls on both systems |
| T15 | development-tools | T09, T10, T13, T14 | Load compiler-matched sanitizer runtimes and preserve instrumentation failures |
| T16 | development-tools | T15 | Qualify RTSan explicitly and keep realtime contracts intact |
| T17 | development-tools | T14, T15 | Generate coverage with matching LLVM tools and selected-artifact provenance |
| T18 | runtime | T03, T10, T11, T12 | Prove headless operation does not require a display or initialize rendering |
| T19 | runtime | T18 | Preserve mjpython and qualify real Linux desktop viewers |
| T20 | runtime | T03, T11 | Make worker selection portable and qualify asynchronous process startup |
| T21 | runtime | T12, T18, T19, T20 | Qualify final-only recording and error lifecycle on both platforms |
| T22 | runtime | T12, T21 | Separate portable recording inspection and playback from checked physical replay |
| T23 | environment | T03, T10, T12 | Make source-first installation and native loader diagnostics explicit |
| T24 | delivery | T02, T23 | Build manifest-driven offline source bundles for each qualified target |
| T25 | delivery | T10, T11, T14, T21, T24 | Install full and slim bundles offline with local native compilation |
| T26 | delivery | T14, T15, T16, T17, T19, T20, T22, T23, T25 | Document supported workflows and remove machine-specific development references |
| T27 | integration | All prior implementation/qualification tasks | Preserve test profile meaning and compare failures by exact provenance |
| T28 | integration | T26, T27 | Add mandatory Linux and macOS CI without replacing existing workflows |
| T29 | integration | T19, T20, T21, T22, T25, T27, T28 | Rehearse real desktop, performance, and offline release workflows |
| T30 | integration | All prior implementation/qualification tasks | Close every requirement with evidence and perform a no-regression release review |

### Integration grouping and parallelism

T10 and T13 form one integration group: the native target graph/context generator and its validator must land together. T13's data-only unit tests can pass first; its real-build acceptance completes with T10. Record this as implementation-ready/awaiting-integration, not as a completed platform gate. One native-build owner resolves their CMake changes in one integration worktree.

A practical serial spine is T01 -> T02/T03 -> T04/T05 -> T06 -> T07/T08 plus T09 -> T13/T10 -> T11 -> T12. Tooling work T14-T17 and runtime work T18-T23 can then proceed on separate owned files. Delivery T24-T26 follows the validated source-install/dependency contract. T27-T30 close integration after all contributing gates are available.

Do not let agents concurrently rewrite native/CMakeLists.txt. The native-build owner integrates reviewed patches for numerical targets, sanitizer controls and context identity. The numerics owner owns tools/native_dependencies.py after T05 handoff. The runtime owner serializes T12 and later artifact/replay edits. The development-tools owner serializes native_checks.py context/tool changes. The delivery owner handles shared README/offline edits after the corresponding implementation interface is stable.

Parallel agents may prepare failing tests/design evidence before dependencies finish, but must not invent neighboring APIs. Required interfaces are in the spec and task-index.json. Any interface/schema change first updates its defining task/spec and all consumers in one reviewed change.

## 5. File/responsibility map

| Boundary | Existing files retained | New focused files |
|---|---|---|
| Platform/SDK | native/CMakeLists.txt | native/cmake/BikePlatform.cmake, BikeAppleSDK.cmake |
| Interpreter/engine discovery | native/CMakeLists.txt engine probe | tools/native_dependencies.py, native/cmake/BikeDependencies.cmake |
| Numerical provider | native/src/cblas_abi.hpp, rider/least_squares.* | native/cmake/BikeNumerics.cmake, native/src/platform/blas_*.cpp, lapack.hpp, lapack_*.cpp, numerics_symbols.hpp.in |
| Compiler/link graph | Existing object/core/engine/module/tests/benchmark targets | native/cmake/BikeCompilerPolicy.cmake, BikeLinkage.cmake |
| Exact numerical lowering | numeric_numpy.hpp, numeric_norm.hpp, numeric_sincos.hpp, numeric_sum.hpp | tools/numeric_portability_probe.py and demonstrated platform math implementation only |
| Build check context | tools/native_checks.py, check_native_contracts.py | native/cmake/BikeCheckContext.cmake |
| Runtime identity | native/src/runtime/provenance.cpp, src/bike_sim/native/artifact.py | src/bike_sim/native/execution_identity.py |
| Development orchestration | tools/run_tests.sh and analysis/coverage wrappers | tools/dev.py, native_toolchain.py, native_runtime.py |
| Runtime/UI/processes | Existing CLI, viewer, async_close and batch modules | Targeted tests, not a parallel Linux runtime |
| Recording/replay | Existing recording and research lifecycle/read/replay modules | Versioned fixtures and platform regression tests |
| Offline/install | Existing full/slim shells, install_offline.py, source validator | tools/offline_bundle.py and network-blocked qualification probe |
| Release evidence | Existing tests/audits/workflows | tools/portability_qualification.py, ci_portability.py, task/result evidence |

Follow existing ownership and naming. A numerical lowering file is created only when an actual failing operation requires it; do not introduce a large abstraction framework simply to replace a few guarded calls. Test/control files are first-party sources and must appear in the appropriate source/analysis manifests when compiled.

## 6. Milestones and hard stop conditions

**M0 - Recovered baseline.** Real checkout, lock and missing metadata recovered; old macOS behavior and Linux failure captured. Stop changes to dependency pins when the original environment is not established; use a separately labeled reconstructed candidate, never silently replace the original lock.

**M1 - Linux configure/build graph.** Linux no longer requires Apple tools; exact MuJoCo/private ABI, provider metadata, typed adapters and compile/link policies qualify. macOS configures/builds through the retained SDK/Accelerate path. No undefined private symbol, guessed ABI or changed source ownership is accepted.

**M2 - Numerical correctness.** Both Linux compilers and required interpreters pass the exact local native oracles, while macOS results remain unchanged. Counterexamples are retained. A numerical mismatch blocks support; do not fix it with skips, tolerance changes, system BLAS substitution or physics edits.

**M3 - Development/runtime workflows.** Analysis/source/ODR/hardening/sanitizer/coverage and headless/desktop/worker/recording/replay paths have actual evidence. Unsupported requested tools fail explicitly. No-display success and real GUI success are separate rows.

**M4 - Installation/delivery.** Fresh source bootstrap and network-blocked full/slim native source installs work on each required target. A moved source bundle rebuilds locally. No host virtualenv/cache/binary is treated as portable.

**M5 - Release decision.** T30 aggregates exact evidence for the reviewed source. Missing macOS evidence, missing qualified numerical profile, absent required GUI/offline checks, or new test failures yields blocked. Optional targets remain unqualified rather than being mislabeled supported.

## 7. Requirement-to-task coverage

| Requirement | Owning tasks | Required acceptance evidence |
|---|---|---|
| Keep macOS fully supported | T01,T04,T06-T12,T14-T19,T26-T30 | Real original baseline and current macOS build, exact oracles, tools, launcher/viewer, runtime/install checks |
| Compile on Linux | T02-T10,T13 | Fresh Debug/Release with Clang and GCC, exact engine/private ABI, no Apple commands, valid ELF dependencies |
| Develop on Linux | T03,T09,T13-T17,T23,T26-T28 | Editable env, editor database, all analysis controls, ASan, qualified RTSan, coverage, CI |
| Run Python/native headlessly | T11-T12,T18,T20-T21 | Valid short fixture steps, no GL init, proper worker shutdown, no fallback |
| Run interactive viewers | T19,T21-T22,T29 | Actual Linux/macOS desktop sessions, pacing/controls/reset/close, retained mjpython |
| Preserve numerical behavior | T06-T08,T11-T12,T27 | Local bitwise corpus/kernel/rollout tests, profile identity and first-divergence evidence |
| Preserve recording/replay safety | T12,T21-T22 | Final-only lifecycle, legacy/foreign data read, checked identity rejection before stepping |
| Offline run/build/develop | T02,T23-T26,T29 | Complete target manifests, full/slim network-blocked local native builds and smoke tests |
| Transfer to independent agents | Every task; task-index.json | Stable APIs, exact paths/commands, dependency graph, stop conditions and review records |

## 8. Agent execution and review protocol

Before editing, read the actual checkout's AGENTS.md and native-engineering safety documents, inspect git status and preserve user changes. Work in an isolated branch/worktree using the execution environment's worktree skill when available. The plan does not authorize destructive resets, overwriting a real lock, modifying unrelated spatial-3D work or adding credentials to artifacts.

For each task: write the regression/control; run it and retain the expected failure; implement only the bounded change; run the narrow tests plus dependent gates; review the diff; commit owned hunks. The provided git add examples assume a clean isolated task worktree. When a named file contains other user work, stage only task hunks with git add -p and inspect git diff --cached; do not stage whole directories indiscriminately.

Every task review record contains: task ID, source/base commit, host/CPU/OS/interpreter, dependency/provider/compiler identities, changed files, exact commands and raw exit codes, test counts, red-to-green evidence, negative-control result, macOS impact, remaining host-dependent gates, interface changes, and reviewed commit hash. Status values are not_started, in_progress, implementation_ready, verified, blocked. A local Linux agent cannot mark an unrun macOS gate verified.

Use two reviews where possible: requirement/correctness review first, then maintainability/safety review. Reviewers reject code that only compiles but violates local oracle, error ABI, source identity, realtime allocation, recording or replay contracts. Green tests on an accidentally imported old extension are not accepted; verify the selected artifact and source fingerprint.

## 9. Evidence and uncertainty rules

A library/header/API assumption is not measured support until its probe passes on the selected host. Exact provider symbols, transitive wheel availability, compiler/stdlib feature set, GUI drivers and preserved macOS behavior are implementation-time measurements. The plan supplies concrete discovery/tests and failure boundaries; it does not fabricate their outcomes.

Do not broaden the support claim to all Linux distributions or architectures. A glibc deployment floor is the intersection of the full dependency/toolchain set, not one wheel tag. Do not require cross-OS bitwise replay when numerical environments differ; do require same-environment exact checks and honest foreign-environment diagnostics.

No plan step changes physics to compensate for a porting issue. A known invalid-controller prefix remains invalid and cannot become a throughput result. Any historical macOS full-suite exception must match the independently captured exact ledger; it is not a blanket exception for Linux.

## 10. First executable handoff

Give an implementation lead the original repository/archive plus this entire handoff directory. Start at T01. Recover the real checkout and capture its current environment before running a synchronization command. Then establish T02's frozen profile and use T03's explicit driver. Do not start by replacing CMakeLists.txt, installing arbitrary OpenBLAS, or disabling failing tests.

The handoff is complete when implementation agents can identify the next ready task from task-index.json, reproduce its failure/control, implement its exact interfaces, and produce the evidence required by its review gate. The project is complete only when T30 reports a supported Linux and retained macOS configuration with all required release records.
