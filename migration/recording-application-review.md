# Recording and application: T38–T51 implementation review

Date: 2026-10-17. Input: `repomix-output.xml(20261017-200614).zip`.

The native source implementation for **all fourteen packages T38–T51** is
present in this delivery. Its source-card scope is **64 Python modules and
621 inventoried declarations**. Those numbers describe the reviewed work
inventory; they do not mean that every source branch has passed a native test.

**Status: `implemented_unverified`. No package acceptance gate is closed.**
The user explicitly prohibited compilation and execution. No CMake configure,
compiler, linker, project tool, project Python module, simulator, policy,
database round trip, replay, viewer or test executable was run. File utilities
were used to unpack, read, edit, inspect and package the supplied source text.

## What was implemented

| Packages | Implemented boundary | Authored checks, not run |
|---|---|---|
| T38–T39 | Complete field/coordinate catalog; container schema 1; reviewed aliases; typed native plans, nested pieces, validation and DuckDB chunk ingestion | Descriptor inventory; acquired coordinate catalog; primitive and nested domains; exact integers, nonfinite values, signed zero, empty/null/absent distinctions; invalid piece layouts |
| T40–T42 | Owned RAM capture; reset-separated episodes/events; physical and research adapters; setup assets and final identity receipts; database transaction/validation; exclusive final publication; actual asynchronous period-close worker | Ownership and identity conflicts; record-off behavior; persisted typed shape/count/hash cases; twenty-four publication fault stages; retries, concurrency, signals, worker drain/detach/strict failure |
| T43–T46 | Read-only analyst; bounded queries; reports and CSV/JSONL exports; native checked replay; saved-state playback; versioned legacy export; standalone `bike-recording` | Corrupt/unsealed inputs; read-only SQL; schema/precision boundaries; report/export failures; identity and replay mismatches; analyst exits and argument forms |
| T47 | Native dispatch and command contracts for simulation, export, ride, research, settle, playground and replay; physical/research recording lifecycle | Native defaults; recording opt-in; parser domains; command help/error dispatch; independent configuration defaults; actual compiled-terrain and setup metadata contracts |
| T48–T49 | Owned render replica; six mutable model fields; main-thread GLFW lifecycle; cameras, HUD, controls, pause/reset/terminal hold; bounded cooperative pacing | Render state isolation and generation checks; no-display state manipulation; wrong-thread rejection; input mappings; camera parameters; wall debt and achieved-rate semantics |
| T50 | Native semantic plot/report model; PNG and SVG; offline interactive HTML; ride CSV reader, annotations and tables | Output format structure; nonfinite gaps; Unicode/HTML escaping; marker/series semantics; CSV quoting; canonical physical plot names |
| T51 | Deterministic TOML grid/explicit-job expansion; decimal seed handling; native process pool; cancellation/reaping; durable results; research sweep | Ordering, flags, wide integers, ranges, process exits, literal argv, exclusive output, missing child, queued/active cancellation |

The per-task file lists, source IDs, public symbols and test registrations are
in `recording-application-implementation.json`. The larger source assertion
migration remains in T52/T53. Existing reference fixtures and old source files
were retained.

## Integration and ownership decisions

### Typed columns and the analyst

`runtime/encoding_values.hpp` contains the existing move-only `Piece` and
`EncodedTable` data records. The numerical projector remains in
`runtime/encode.hpp/.cpp`. The split prevents the analyst's piece validator
from transitively including runtime control, MuJoCo or rider/LAPACK headers.
It does not introduce a second encoder or change the physical arithmetic.

The existing source-style binary64 formatter was factored from
`configuration/schema.cpp` into `io/float_format.hpp`. Configuration and analyst
JSON/CSV now share its shortest-round-trip and exponent-format rules without
giving the analyst a configuration/runtime link dependency. This edit to a T12
source also needs requalification with the final integrated tree.

The CMake edge for the installed analyst is:

`bike-recording → bike_recording_cli → bike_recording_reader → bike_io / bike_platform / DuckDB`.

The reader validates the sealing manifest, closed-file hashes, exact database
shape, catalog and relationships before exposing queries. Its implementation
does not initialize a simulator, policy or display. SQL is constrained to a
single read-only statement, and convenience operations use bounded results;
explicit exports stream all requested rows. The other application commands
link the runtime, recording, replay and viewer owners separately.

### Capture and publication

`RunCapture` stores immutable evidence in memory. A reset seals the old episode
in RAM and starts a new episode; it does not publish a directory. Finalization
writes and validates the database transaction, commits, checkpoints, closes
and syncs it, renders a report, hashes closed payloads, creates a new exclusive
destination and links the manifest last. Existing destinations and symlinks
are never adopted as this run's output. Before the sealing marker, a failure
retains owned evidence and permits a retry to a fresh destination. After
successful publication, repeated finalization returns the committed path.

Report failure has a separate status and diagnostic. A valid database may be
sealed with `report_status=failed`; a missing or failed report never becomes a
false database-validation failure or a false complete report. Postcommit
cleanup and directory sync cannot retroactively change a committed success
into a retryable precommit error. Crash and durability behavior still require
the authored OS-level tests.

Ordinary physical and research recordings capture the actual model image and
state, resolved configuration, compiled terrain vertices and referenced profile
bytes. The terrain digest covers the declared little-endian binary64 vertex
payload, separately from the NPY file header. Captured profile bytes must match
the hashes established during setup. Finalization records unchanged, changed
or unavailable input identities without replacing the captured evidence.

The physical launch journal uses a monotonic clock from launch. The episode
wall watchdog has a separate origin which resets with the episode. Operator
events, derived events and terminal evidence keep their source classifications.

### Asynchronous telemetry

`async_telemetry` now enables a real `AsyncCloseRunner`. It receives owned raw
periods and a copied effort-evaluation context. It never receives live
`mjModel`, `mjData`, controller, policy or filesystem owners. It uses the shared
period-close evaluator and returns ordered result objects to the main owner.
There are at most two unacknowledged periods before backpressure drains them.

The main owner stages allocations and state copies before applying a result.
It commits evidence before acknowledging the worker result. Strict failure
therefore preserves the committed failing period and original failure time.
Ordinary worker errors propagate; a lost transport may detach and replay
unacknowledged originals from the last committed main-owner state. Flush,
seal, reset and teardown drain or join through explicit ownership boundaries.

The runtime projection emits the optional `async_telemetry` boolean. The
runtime-schema-1 decoder accepts its absence for existing bootstraps and rejects
unknown keys as before. This is an additive envelope change, not a new Python
bootstrap. The earlier physics-wave note describing async as unsupported is
historical; this delivery supersedes that limitation without requalifying its
old execution evidence.

Asynchronous observations can arrive later. Numerical equality of the period
evaluator does not establish equality of trajectories under policies which
consume delayed observations. No such equivalence is claimed here.

## Explicit source correction

The source encoder intentionally represents a present, valid claimed-root
container with an empty routed remainder as an absent/null optional terminal
leaf. The original storage validator rejected this state by requiring all
root/leaf masks to be equal. Empty rider-control and component maps can reach
that path, so copying both source rules would make otherwise valid native
recordings fail publication.

The C++ validator accepts exactly the relevant optional map/struct/struct-list
terminal state: root present and valid, terminal absent and NULL. It retains
the encoder's original bits and does not synthesize an empty value. Primitive
roots, required leaves, invalid roots, null-parent contradictions and malformed
nonempty leaves retain their checks. Positive and negative cases were added to
T39/T41. The detailed evidence and source references are in
`recording-core-static-review.md`.

## Replay and command behavior

`bike-replay --viewer` visualizes checked re-simulation. `--view` uses stored
positions and velocities for inspection, with pause, seek, restart, loop and
speed controls. Inspection updates only a detached render model. Its synthetic
generation changes permit seeking backward without weakening the generation
rules of live physical/research runtimes.

A checked `ReplaySession` reports success only after final verification.
Stopping early remains incomplete; a mismatch remains a mismatch with the last
verified frame and diagnostic. No extra research control interval is issued
because a window closes. Native replay qualification checks the recorded
execution profile, source identity and numerical provider. Historical
Python/extension execution identities cannot certify a different native
executable profile and are rejected for checked native execution. Stored-state
inspection remains a separate capability. No Python executable or extension
hash is fabricated to bridge the difference.

The ordinary command parser transcribes source/captured option inventories.
Native is the only execution backend. `--out` alone never enables physical or
research recording; `--record` selects full retained evidence. Legacy ride
outputs use the existing legacy recorder and artifact flow. Research built-ins
and plugins remain behind the T35 policy owner. The raw fixed-command research
path preserves rider inputs separately from motor-only policy callbacks.

Independent static integration review found that five captured research choice
sets initially retained the quotes in Python `repr` metadata. The static
transcription now decodes those literal values before embedding the C++
contract. Explicit scenario, rider, check-policy, posture and transmission
arguments therefore use their actual spellings. T47 includes regression source
for this boundary; it has not been executed.

All installed command targets are declared in
`native/cmake/RecordingApplication.cmake`. The shared `main.cpp` is compiled
once as an object target, avoiding duplicate translation-unit entries for the
eight dispatching executables. The analyst and research sweep have independent
entry points. The test-child program is a bounded, test-only executable and is
not installed.

## Viewer, plots and known output differences

The render owner has its own model/data and checks model identity, generation,
state extent and the six declared model-field extents before applying an owned
snapshot. Rendering uses the source-compatible kinematic refresh and never
steps the simulation. Window/context work stays on the main OS thread. The
headless build keeps GLFW initialization disabled.

The frontend has separate simulation and render clocks. Pause and reset rebase
wall debt; only completed steps debit it. Achieved real-time factor derives
from observed progress. Window close, an explicit stop and Ctrl-C pass through
the owning lifecycle hooks even while paused or holding a terminal frame.
The physical viewer preview is produced only when recording was requested.

T50 has a native raster PNG path and an optional explicit Cairo SDK for Unicode
raster text. SVG and HTML retain Unicode labels, series, axes, optional gaps
and annotations. No graphical artifact was rendered or visually inspected
during this task because executing those generators was prohibited.

**The snapshot does not contain a genuine Plotly JavaScript bundle.** The
default HTML therefore identifies its real backend as `native-html-v1` and
provides local SVG charts, hover, legend controls, pan/zoom and SVG/JSON export.
A supplied genuine local Plotly bundle can be embedded through
`set_plotly_resource()`. No CDN access or fake Plotly API is used. The default
renderer is an explicit implementation difference from the planned Plotly
backend; Plotly output parity and a qualified Cairo binary distribution remain
open. See `native/resources/plotly/README.md` and
`viewer-plots-static-review.md`.

## Batch/process behavior

Grid expansion preserves source ordering: base × grid first, then explicit
job overrides. Parameters produce literal argv tokens, with booleans following
the source flag rules. Seed lists and integer ranges retain exact decimal
identities. The research parser validates job arguments before workers start.

Each job runs in a separately owned native child process and process group,
including `--serial`, which uses one child at a time. This replaces the source's
Python in-process worker while keeping deterministic input/result identities.
The parent never changes its working directory and never evaluates a shell
command. It drains stdout and stderr independently, bounds output, handles
spawn/nonzero failures and delivers completed results to one writer. The
results log is flushed/synced as each result arrives. A cancellation prevents
queued launches, signals active child groups, applies a bounded grace interval
and reaps children before returning. A worker or result-consumer exception also
joins owned workers and child processes.

Static review repaired a descriptor-ownership race: after the drain loop
closes a pipe, its RAII owner is invalidated so it cannot close a descriptor
number later reused by a sibling thread. Pipe creation/CLOEXEC/spawn setup is
serialized to prevent one concurrent child from inheriting another child's
temporarily unmarked pipe. These changes require the authored process tests;
no child was launched in this delivery.

Sweep logs concatenate the separately captured stdout and stderr streams;
their cross-stream byte ordering is not an interleaved timeline. The two raw
streams remain separately available for batch jobs. A zero process exit is
never used as a physical-validity certificate; recorded outcomes and metrics
keep their own meaning.

## Static evidence and remaining acceptance

`recording-application-static-audit.json` is a file-inspection report. It covers
JSON readability, embedded catalog agreement, local include resolution,
declared new CMake sources, authored test presence, analyst include closure,
and unchanged original Python/fixture content. It is not a C++ syntax check,
link check, numerical comparison, database test or clean-build result.

The delivery still needs the complete relevant gates from the two owner plans:
strict compilation/linking, all focused T38–T51 tests, upstream regression
impact, source-specific oracles, exact persistence/replay comparisons, signals
and crash tests, GUI interaction, and plot semantics/appearance. The current
dependency lock is the supplied macOS baseline; Linux and autonomous installer
qualification remain in T58/T59. Missing external SDKs and binary fixtures must
be acquired according to their existing manifests before execution.

T11 after its last change, T14 edge cases and T56/T57 evidence are not promoted
by this source-only wave. T61/T62 acceptance and T63 deletion remain unchanged.
The supplied text snapshot is not a live Git checkout, so no current HEAD,
commit, clean repository state or successful build is claimed.
