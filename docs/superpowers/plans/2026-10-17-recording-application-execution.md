# Recording and application implementation plan

> **For agentic workers:** Follow the existing T38–T51 source cards and the task-by-task implementation/review workflow. The user forbids compilation and project/test execution in this session; all runtime gates remain pending.

**Goal:** Implement the native recording and application layer T38–T51 against the uploaded source snapshot.

**Architecture:** Typed native column pieces feed an immutable in-memory capture and a final-only DuckDB publisher. A separate read-only analyst library validates published recordings; checked replay owns a simulation separately. Native command dispatch joins the existing setup/research/legacy runtimes with owned render replicas, reports and process-isolated batch jobs.

**Tech Stack:** C++23, existing CMake dependency locks, MuJoCo, DuckDB C API, GLFW/Cairo when configured.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`; detailed steps and required cases are in `2026-10-14-python-free-cpp-06-recording-replay.md`, `2026-10-14-python-free-cpp-07-cli-viewer-batch.md` and `migration/cards/T38.md` through `T51.md`.

## Global constraints

- No compilation, project execution, test execution or dependency downloads in this session.
- Preserve canonical source field inventory, native numeric kernels and owned interfaces.
- Recording is opt-in, retained in RAM and published exclusively at finalization; reset alone never publishes.
- Read-only analyst operations must not initialize a simulation, load policies or require a display.
- Preserve Python references until T63; do not mark runtime or parity gates passed without execution.
- One integration owner updates CMake, dispatch and migration evidence after interface review.

## Task 1: Catalog, encoding, capture and publication (T38–T42)

**Files:** `native/resources/recording/catalog-v1.json`, `native/src/recording/{catalog,coordinates,schema,encoding,plan,duckdb_chunks,contracts,capture,database,publish}.*`, and `native/tests/migration/test_t38.cpp` through `test_t42.cpp`.

**Interfaces:** Consume `runtime::EncodePlan`, `runtime::Piece`, `runtime::OwnedNativeEpisode`, `runtime::OwnedResearchEpisode`, `bike::io::Value` and `bike::platform` exclusive-file primitives. Produce one authoritative catalog, `RunCapture`, native DuckDB schema/writer and explicit final publication.

- [x] Transcribe complete schema descriptors directly from the supplied source without importing or executing it.
- [x] Implement typed plans, nested data chunks, owned capture and transactional publication with retry/error semantics.
- [x] Add regression sources covering invalid input, null/empty distinctions and publication failures.
- [x] Review source and call-site contracts; record unresolved runtime checks separately.

## Task 2: Reader, replay, reports and analyst command (T43–T46)

**Files:** `native/src/recording/{reader,analysis,report,export}.*`, `native/src/replay/*`, `native/src/cli/recording.*`, and migration tests T43–T46.

**Interfaces:** Consume the catalog/schema/database contract; expose read-only `Recording`, explicit reports/exports and checked replay that compares actual simulated state against retained evidence.

- [x] Implement validation of the manifest, closed-file digests, table schemas and episode selection before queries.
- [x] Implement analyst operations, versioned replay, CSV/JSONL and legacy export without a Python fallback.
- [x] Add unrun regression cases and review malformed recordings, mutation attempts and mismatches.

## Task 3: Native commands and batch ownership (T47, T51)

**Files:** `native/src/cli/{arguments,application,ride,research,settle,export,playground,replay,main}.*`, `native/src/batch/*`, `native/src/platform/process.*`, migration tests T47/T51.

**Interfaces:** `bike::cli::dispatch(executable, arguments)` selects nine installed entrypoints. The same research parser validates batch jobs before workers spawn. Batch workers use explicit argv arrays and independent process ownership.

- [x] Port argument inventory/defaults/validation and assemble existing typed setup/physics/research requests.
- [x] Wire physical/research capture and finalization; distinguish saved-state playback from checked replay.
- [x] Implement deterministic TOML grid expansion, worker cancellation/reaping and ordered durable result records.
- [x] Add unrun parser/grid/error regressions and statically inspect all public paths.

## Task 4: Rendering and artifacts (T48–T50)

**Files:** `native/src/viewer/*`, `native/src/viz/*`, narrowly scoped `native/src/runtime/runtime.*` additions, migration tests T48–T50.

**Interfaces:** Immutable `runtime::RenderFrame` snapshots feed an independent model/data owner. Input, pacing and reset callbacks remain outside the physics loop. Plot/report functions consume typed series or validated recording data.

- [x] Implement owned render state, GLFW controls, HUD/cameras and cooperative bounded pacing.
- [x] Port native plot generation and offline interactive artifact output. The first-party HTML backend and optional local Plotly seam are implemented; the upstream Plotly bundle/license and qualified Cairo binary remain external acquisition/acceptance work.
- [x] Add unrun ownership/input/plot regressions and source-level no-stepping checks.

## Task 5: Integration and deliverables

**Files:** `native/cmake/RecordingApplication.cmake`, root CMake integration, `native/tests/migration/CMakeLists.txt`, `migration/recording-application-*`, patch and archives.

- [x] Reconcile producer/consumer signatures and headless/GUI link boundaries.
- [x] Review changed source, schemas and declared resources; repair concrete findings.
- [x] Record implementation status without promoting unexecuted gates to complete.
- [x] Produce patches relative to the uploaded snapshot, an archive containing every changed project file and the complete reconstructed source snapshot.

## Verification after this session

The existing plans retain their exact native build, focused CTest, oracle and full gate requirements. They must run on the final integrated source in an environment where execution is permitted. No red/green, compiler, CTest, GUI, database round-trip or replay result is asserted by this plan.
