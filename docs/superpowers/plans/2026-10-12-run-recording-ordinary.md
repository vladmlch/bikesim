# Ordinary Native Run Recording Implementation Plan — Stage A

> **2026-10-13 delivery:** ordinary and research source changes and regression sources are supplied. See the [task/R01-R24 source-completion ledger](../audits/2026-10-13-run-recording-source-completion.md) and [usage/migration guide](../../RECORDING.md). No builds, imports, tests, analyzers, simulator/GUI runs or benchmarks were executed. Execution-dependent completion checkboxes below remain open; this is not an accepted A6/B4 runtime gate. Stage-B source was completed under the no-execution instruction, not after a claimed green stage-A run.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Save full ordinary native ride evidence in RAM and publish an analyzable DuckDB recording at launch completion.

**Architecture:** A small shared capture module owns immutable episode sources. Native accounting retains one full sample index independently of measurement scheduling. Frontends journal input and finish episodes; a final-only writer produces typed tables, a manifest and a report.

**Tech Stack:** Existing native runtime, Python >=3.13 through uv, DuckDB 1.5.6, PyArrow 26.0.0, NumPy and pytest.

**Spec:** [Run recording design](../specs/2026-10-12-run-recording-design.md), R01–R12/R14–R19/R22–R24. Begin after [A0](2026-10-12-run-recording.md#a0-establish-implementation-baseline).

## Global Constraints

- Invoke Python and Python tools through `uv`.
- Use Python >=3.13, C++23, nanobind, and MuJoCo 3.12.0 for native execution.
- Preserve valid-input binary64 operation order, physics timestep, controller/sensor scheduling, and existing comparison tolerances.
- Store physical samples at every accounted physics interval when recording is enabled; do not downsample or quantize the canonical recording.
- Collect the whole launch in RAM and write only during launch finalization.
- Make persistent recording opt-in with `--record`, default false.
- Keep database access, Python boxing, report generation, and disk I/O outside realtime kernels.
- Preserve user-owned changes, unrelated plans, skills, and output; do not delete or overwrite an existing run.

## File map

| Path | Responsibility |
|---|---|
| `src/bike_sim/recording/contracts.py` | Immutable run/episode/event/asset values and evidence-source protocol |
| `recording/capture.py` | One launch, event ordering, retained episodes and idempotent finalization |
| `recording/catalog.py`, `recording/encoding.py` | Reviewed field/coordinate catalog, presence/nonfinite and typed batch conversion |
| `recording/native.py` | Immutable native episode source adapter, lazy batch export |
| `recording/storage.py`, `recording/reader.py` | DuckDB schema/ingestion, staging/publication and validated read-only opening |
| `recording/analysis.py`, `recording/report.py` | Typed queries/comparison and deterministic Markdown report |
| `src/bike_sim/cli/recording.py` | Agent-friendly CLI and bounded output |
| Existing physical driver/frontends | Capture policy, episode lifecycle, requested-input journal and one finalization |
| Existing native accounting/samples/runtime | Measurement separation, one retained sample index and owned sealed episode |

Paths abbreviated as `recording/...` above are under `src/bike_sim/recording/`. Create its `__init__.py` with only the intentionally public contract/owner exports; keep storage imports lazy. Keep legacy `native/recording.py` as research compatibility until stage B.

## A1. Capture contracts, catalog and dependency seam

**Files:** Create the package, `contracts.py`, `capture.py`, `catalog.py`, `encoding.py`; modify `pyproject.toml`, `uv.lock`; create `tests/reference/_recording_support.py`, `test_recording_capture.py`, `test_recording_catalog.py`.

**Interfaces:** Define these shared types before later tasks consume them:

```python
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Protocol
from uuid import UUID
import numpy as np

@dataclass(frozen=True)
class RunMetadata:
    run_id: UUID
    started_at: datetime
    backend: str
    mode: str
    values: Mapping[str, Any]

@dataclass(frozen=True)
class InputEvent:
    episode_id: int
    event_id: int
    kind: str
    received_wall_s: float
    processed_step: int
    applied_step: int | None
    values: Mapping[str, Any]

@dataclass(frozen=True)
class Asset:
    name: str
    media_type: str
    sha256: str
    content: bytes

@dataclass(frozen=True)
class TableBatch:
    table: str
    columns: Mapping[str, Sequence[Any] | np.ndarray]
    row_count: int

class PhysicalSource(Protocol):
    row_count: int
    def tables(self, *, batch_rows: int) -> Iterator[TableBatch]: ...

@dataclass(frozen=True)
class EpisodeEvidence:
    episode_id: int
    generation: int
    physical: PhysicalSource
    summary: Mapping[str, Any]
    assets: tuple[Asset, ...]
    research: Any | None = None

class RunCapture:
    def __init__(self, metadata: RunMetadata, *, enabled: bool): ...
    def event(self, event: InputEvent) -> None: ...
    def add_episode(self, evidence: EpisodeEvidence) -> None: ...
    def finalize(self, out_root: Path) -> Path | None: ...
```

The method bodies above are contract declarations, not implementation stubs to check in. Freeze nested metadata as well as dataclass attributes. Type `research` as a forward-declared `ResearchEvidence` under `TYPE_CHECKING` once B1 defines it; stage A accepts only `None`. `RunCapture` implementation is in `capture.py`, not duplicated in `contracts.py`. Dependencies are injected at the private publication seam for failure tests.

- [ ] Add pinned storage dependencies and resolve with `uv lock`; verify frozen installation on the actual project Python. Do not use scratch environment results as proof of project compatibility. New imports are absent from native advancement and ordinary disabled CLI startup.
- [ ] Write capture tests first: disabled capture creates no path; conflicting episode IDs reject atomically; identical re-addition is idempotent; event IDs/order are validated; zero-interval evidence is valid; nested mutable source input cannot modify captured metadata.

```python
from pathlib import Path
from bike_sim.recording.capture import RunCapture
from _recording_support import run_metadata

def test_disabled_capture_has_no_persistent_side_effect(tmp_path):
    target = tmp_path / 'not-created'
    capture = RunCapture(run_metadata(), enabled=False)
    assert capture.finalize(target) is None
    assert not target.exists()
```

In `_recording_support.py`, implement `run_metadata()` using UUID `00000000-0000-0000-0000-000000000001`, UTC `2026-10-12T00:00:00+00:00`, backend `native`, mode `ride`, and explicit synthetic provenance. Add `TreePhysicalSource` there for independent hand-authored rows; its table conversion goes through the production catalog, not through the production native adapter.
- [ ] Implement `RunCapture` ownership/identity validation and a finalization state machine (`collecting`, `publishing`, `published`, failed publication retry). Only `finalize()` opens a storage destination; `event()` and `add_episode()` stay RAM-only.
- [ ] Inventory the supported runtime's typed channels from source and existing schema-2 records. Populate the finite field catalog with source/units/phase/optionality/measurement metadata. Enumerate aliases from `PhysicalRecorder`/`PhysicalSampleData::flat_columns`; canonicalize by meaning, never by coincidentally equal numeric values. Add coordinate catalogs from actual model joint/qpos/DOF maps.
- [ ] Add pure round-trip tests for all R10 cases, nested parent presence, empty lists, vector units/address maps and unknown-field rejection. Use an intentionally unrecognized field and require a named error, not silent dropping. Add source-backed catalog cases for control ceiling, incoming state, specific acceleration and crank torque.
- [ ] Verify and review A1:

```bash
bash tools/run_tests.sh quick tests/reference/test_recording_capture.py tests/reference/test_recording_catalog.py
```

Completion: a disabled run has no persistent side effect; the reviewed catalog and contract types exist; the resolved dependency lock works on the project platform. Checkpoint: `feat: define typed in-memory run recording contracts`.

## A2. Native retention independent of measurement and safe episode sealing

**Files:** Modify `native/src/runtime/accounting.hpp`, `accounting.cpp`, `samples.hpp`, `samples.cpp`, `step.hpp`, `step.cpp`, `config.hpp`, `runtime_binding.hpp`, `runtime_binding.cpp`; native configuration decoder in `native/src/runtime/`; `src/bike_sim/native/setup.py`, `runtime.py`, `contracts.py`; mirrored `src/bike_sim/sim/ride/physical_runtime.py`; create `recording/native.py`, `tests/reference/test_native_recording_retention.py`; extend native accounting/runtime tests.

**Interfaces:**

```text
RetentionMode = off | full | legacy
NativeRideDriver.seal_episode() -> OwnedNativeEpisode
OwnedNativeEpisode.generation: int
OwnedNativeEpisode.row_count: int
OwnedNativeEpisode.batch(start: int, limit: int) -> owned structured sample batch
NativePhysicalSource(episode: OwnedNativeEpisode, catalog) implements PhysicalSource
```

`OwnedNativeEpisode` is a binding-owned immutable value containing `SamplePtr` references, committed extents and needed catalog/config copies, never a borrowed runtime/model pointer. It remains exportable after the runtime resets/closes. Ordinary programmatic legacy constructors keep their existing decimated-recorder defaults; CLI integration explicitly passes `off`/`full`. This avoids breaking legacy explicit `save()/export()` while changing the CLI default.

- [ ] Add a red test that full retention contains every interval even with legacy measurement decimation 80, and a valid legacy control that still reports its old decimated row count. The source must expose forces, scalar types and fresh/forwarded measurement provenance rather than relying on float-only aliases.
- [ ] Add the passivity regression before changing retention. Adapt the existing independent `environment_factory` pattern:

```python
from _bits import assert_bitwise_equal
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_driver import PhysicalRideDriver

def test_recording_does_not_change_committed_trajectory(environment_factory):
    drivers = [PhysicalRideDriver(environment_factory(decimation=80).sim,
        backend='native', duration_s=.01, decimation=80, record=enabled)
        for enabled in (False, True)]
    try:
        for target in (1, 4, 8):
            for driver in drivers:
                driver.advance(target, RideControl(0.))
            assert_bitwise_equal(drivers[0].snapshot().integration_state,
                                 drivers[1].snapshot().integration_state)
        assert drivers[0].reason == drivers[1].reason
        assert drivers[0].snapshot().model_status == drivers[1].snapshot().model_status
    finally:
        for driver in drivers:
            driver.close()
```

The new driver `record` argument is introduced in this task as retention-policy plumbing; lifecycle/publication belongs to A3/A4. Use a nonbinding wall limit. Also compare fixed-step applied inputs, energy/check histories and first failure; final state alone cannot prove all earlier transitions matched.
- [ ] Split the legacy scheduling field into explicit measurement scheduling and retention policy in both Python and C++. Preserve baseline authoritative measurement/check cadence for the same old argument. Make the adapter's `--record` select full retention only. Record forwarded measurement provenance. Preserve effective public legacy projection, update bridge validation/defaults and document its mapping.
- [ ] Retain one index of immutable samples; build old numeric column projections lazily on explicit legacy reads. Eliminate eager parallel column blocks where they duplicate the new retained history. Off mode keeps latest/necessary pending consumer evidence and running metrics without an optional growing history. Do not suppress samples needed by research consumers.
- [ ] Implement owned episode sealing from already committed retained evidence. Expose component generalized forces via a recording-specific batch view; leave legacy schema-2 dictionaries readable. Validate batch range/type domains before boxing and keep the source reusable after failed conversion. Allocate/copy before reset or delivery acknowledgement, consistent with native safety rules.
- [ ] Test empty run, partial control period, strict flush rejection, allocation/boxing fault and retry, acknowledgement, reset then export, close then export, integer identity and missing/nonfinite values. Failed sealing must leave the active evidence available. Use existing native test fault-injection seams rather than invoking undefined engine states in-process.
- [ ] Verify focused native behavior and the native intermediate gate:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh native -k 'recording_retention or runtime_accounting or ride_frontend'
```

Completion: retained output is consecutive and complete, old explicit consumers still behave as documented, on/off physical/control/check sequences agree, and an owned episode survives reset/close. Checkpoint: `feat: separate native recording retention from physical measurement`.

## A3. Ordinary CLI, lifecycle, input journal and check policy

**Files:** Modify `src/bike_sim/cli/ride.py`, `sim/ride/physical_driver.py`, `physical_frontend.py`, `physical_session.py`; extend `recording/capture.py`, `recording/native.py`; create `tests/reference/test_recording_cli.py`, `test_native_recording_lifecycle.py`; extend `test_native_ride_frontend.py`.

**Interfaces:** `PhysicalRideDriver(..., record: bool | None = None)` maps explicit CLI booleans to off/full and `None` to legacy programmatic compatibility. `PhysicalRideDriver.seal_episode(episode_id: int, *, reason: str) -> EpisodeEvidence` captures owned summary/source/assets once; it does not persist. The frontend owns one `RunCapture` across resets. Add `--record` and independent `--check-policy strict|diagnostic`; parser resolution preserves existing headless/viewer defaults.

- [ ] Write parser tests for flag defaults, native supported paths, unsupported ordinary Python plus `--record`, physical/nonphysical combinations and independent stop-policy selection. `--out` alone does not enable recording. `--time-scale` remains an interactive option and is absent from headless examples.
- [ ] Write a viewer-pump lifecycle test with fake input/clock and a real driver adapter test separately: two reset-separated episodes, duplicate terminal notification, Ctrl-C and caught-error paths must retain exact episode IDs without invoking a writer before finalization. Use a publisher that raises if called from advance/reset/render.
- [ ] Refactor `export_generation()` into RAM-only episode sealing. Keep the whole launch owner alive through viewer termination. On reset, capture the outgoing owned episode before resetting the runtime; if a strict flush rejects, preserve its committed prefix/first failure and avoid a partially installed new episode. Finalize once before teardown discards retained evidence.
- [ ] Record raw queued inputs and processed step association before mutating local strength/braking/playback state. Add explicit requested/applied records so rollback-hold effective braking does not overwrite requested braking. Preserve canceling toggles and strength changes while brakes are off. Timestamp receive and process separately; no wall-time field is used as a physics input.
- [ ] Capture existing source events and actual controller branch diagnostics. For missing reason instrumentation, add compact raw branch/gate facts with staged publication outside marked kernels. Derived state transitions have a rule/version and `derived_transition` classification. Tests cover contact on/off, shift/coast, limiter/validity transitions and a missing-reason case; inferred reasons cannot be labeled source events.
- [ ] Resolve check policy before runtime construction; existing fatal engine/poison behavior remains terminal. Disable ordinary preview file writers and automatic recording-derived plots when CLI recording is off. When on, report generation happens after final DB publication, not every render. Preserve explicit legacy `--html` reading for old preview files.
- [ ] Test caught error plus failed export preserves the original failure, while a successful simulation plus failed requested export becomes nonzero. Test Ctrl-C without simulating another hidden control action, terminal viewer hold then close, reset retention, and zero-step initialization failure where owned evidence exists.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh native -k 'recording_lifecycle or recording_cli or ride_frontend'
```

The native profile's collection covers direct native tests; run the Python-only parser tests separately through `quick` when they are not selected by the profile.

Completion: recording is genuinely opt-in, every episode remains available until launch end, all input events are ordered, and ordinary headless/viewer finalize through one owner. Checkpoint: `feat: journal ordinary native rides until launch finalization`.

## A4. Typed DuckDB storage, reader and final publication

**Files:** Create `recording/storage.py`, `reader.py`; extend catalog/encoding/capture/native adapters; create `tests/reference/test_recording_storage.py`, `test_recording_publication.py`, `test_recording_values.py`.

**Interfaces:** `write_recording(capture: RunCapture, out_root: Path) -> Path`; `open_recording(path: Path) -> RecordingReader`; reader is a context manager exposing `connection` opened read-only, `metadata`, `episode_ids` and schema/catalog information. Add `RunCapture.iter_episodes() -> Iterator[EpisodeEvidence]` and `iter_events() -> Iterator[InputEvent]` as read-only writer accessors; they do not consume data or expose mutable containers.

- [ ] Add independent synthetic payload fixtures for zero/one/multiple episodes, vectors/forces, optional parent/leaf keys, NaN/infinities/signed zero, categories/booleans and forwarded measurement metadata. `synthetic_capture()` in `_recording_support.py` builds a reviewed `RunCapture` using `TreePhysicalSource` and hand-computed powers/work; it never runs a simulator.
- [ ] Write public reader round-trip tests and publication failure cases before the writer. SQL assertions include types, presence, counts, event order and foreign references, not just byte equality between two uses of the same encoder.

```python
import math
from bike_sim.recording.reader import open_recording
from _recording_support import synthetic_capture

def test_round_trip_keeps_nan_distinct_from_null(tmp_path):
    saved = synthetic_capture(value_cases=True).finalize(tmp_path)
    with open_recording(saved) as recording:
        rows = recording.connection.execute(
            'SELECT residual_j, residual_j_present '
            'FROM channel_energy ORDER BY interval_id'
        ).fetchall()
    assert math.isnan(rows[0][0]) and rows[0][1] is True
    assert rows[1] == (None, True)
    assert rows[2] == (None, False)
```

The synthetic fixture emits three consecutive intervals whose optional `energy.residual_j` is respectively NaN, explicit null and absent, with the corresponding source/measurement provenance. The reviewed energy catalog declares its optional presence column; no test-only production table or encoder branch is introduced.
- [ ] Implement versioned core tables from spec R15 and catalog-generated domain tables with presence columns. Use quoted/validated identifiers and Arrow record batches at bounded export batch size, default 4096. Keys are INTEGER/BIGINT/UUID, not float64 conversions inherited from CSV. Store research-free ordinary observations as absent streams, not fabricated rows. Provide canonical `telemetry` and compatibility alias views.

```python
connection.execute('BEGIN TRANSACTION')
try:
    create_schema(connection, catalog)
    ingest_capture(connection, capture, batch_rows=4096)
    validate_recording_tables(connection)
    connection.execute('COMMIT')
except BaseException:
    connection.execute('ROLLBACK')
    raise
connection.execute('CHECKPOINT')
connection.close()
```

Private `create_schema(connection, catalog)`, `ingest_capture(connection, capture, *, batch_rows)`, and `validate_recording_tables(connection)` are defined in `storage.py`; validation checks per-episode consecutive interval counts/extents, presence/catalog completeness and all declared relationships. Preserve a primary insert failure if rollback/close also fails.
- [ ] Implement final-only staging, file closure/hash verification, exclusive destination reservation and manifest-last publication per R16. Never create persistent destinations from capture append/reset. Preserve a failed in-process capture for a retry to a fresh exclusive directory; repeated successful `finalize` returns the original path. Distinguish incomplete publication from a sealed failed simulation. Reader rejects missing manifest, unknown schema, wrong hash and dangling asset declarations.
- [ ] Add fault injection at ingestion, COMMIT, checkpoint, report generation, file move and manifest publication. Verify old runs, unrelated files and an existing empty destination remain untouched. A report failure still publishes valid data with report status/error; a payload failure never creates a successful seal. Assert no optional history is silently dropped when export batches are small.
- [ ] Verify this isolated Python work:

```bash
bash tools/run_tests.sh quick tests/reference/test_recording_capture.py tests/reference/test_recording_catalog.py tests/reference/test_recording_storage.py tests/reference/test_recording_publication.py tests/reference/test_recording_values.py
```

Completion: all supported typed evidence round-trips through the public reader, publication is sealed/unique/retryable, and reader errors are explicit. Checkpoint: `feat: publish typed DuckDB recordings at run completion`.

## A5. Report and analyst CLI

**Files:** Create `recording/analysis.py`, `report.py`, `src/bike_sim/cli/recording.py`; modify `pyproject.toml` for `bike-recording`; extend reader; create `tests/reference/test_recording_analysis.py`, `test_recording_report.py`.

**Interfaces:** `overview(reader)`, `fields(reader, *, pattern=None)`, `events(reader, *, episode_id, kind=None)`, `window(reader, *, episode_id, start_s, end_s, fields)`, `compare(left, right, *, left_episode, right_episode, mode, fields, atol=1e-9, rtol=1e-9)` produce typed mapping/list results; `render_report(reader) -> str` renders these facts. CLI `main(argv=None) -> int` maps exactly to spec R18; JSON uses the tagged nonfinite codec, and SQL is read-only with default `--limit 200`.

- [ ] Write synthetic independent tests for half-open interval selection and overlapping intervals, first mismatch, prefix mismatch, incompatible units/phases/grid/DOF identity, numeric tolerance, null versus missing, matching invalid NaNs, categorical changes and summary-mode parameter changes.
- [ ] Implement these queries with bound values and catalog-validated field selection. Do not compare by position or interpolate silently. First mismatch uses stable episode/interval keys, and differences reference exact saved evidence. Emit all comparison tolerances and rejected-compatibility reasons in machine-readable output.
- [ ] Implement deterministic report sections from R17. Identify events/limits with source references, list unmeasured/unavailable diagnostics and invalid physical status. Report field extrema only over the declared measurement domain; a carried closing energy measurement is not an instantaneous peak at every repeated row.
- [ ] Add CLI parser/read-only/truncation tests, including empty results, unknown field, multiple episodes, attempted write SQL, huge SELECT and explicit CSV/JSONL export. SQL output is bounded with a truncation indicator. Metadata never causes a Python module or recorded policy to be imported.
- [ ] Connect report rendering to the A4 writer's finalization seam. The manifest includes report success/error; regeneration reads an existing sealed DB and does not rewrite it.

```bash
bash tools/run_tests.sh quick tests/reference/test_recording_analysis.py tests/reference/test_recording_report.py tests/reference/test_recording_storage.py
```

Completion: an agent can start from `overview`, find an event, inspect its time window and compare saved runs without reading the entire payload. Checkpoint: `feat: add post-run recording queries and evidence report`.

## A6. Ordinary integration acceptance and documentation

**Files:** Update `docs/RIDE.md`, create `docs/RECORDING.md`, update the A0 audit; extend `test_native_recording_lifecycle.py` and `test_native_recording_retention.py` where final integration exposes gaps.

- [ ] Run a supported short fixed-input native headless episode with recording off/on; compare all captured checkpoint states/commands/check outcomes using the independent passivity tests. Include normal completion, diagnostic invalid continuation, strict rejection and an interrupted viewer generation test. Keep application/test selectors consistent.
- [ ] Run an actual viewer manually on the supported host: record off, then on, brake/strength/playback changes, reset twice and close. Assert no recording output is published before close; inspect three episodes and requested/applied event order afterward. A mocked viewer is not GUI verification.
- [ ] Document the proposed ordinary invocation and analysis workflow, with the supported reference profile clearly named:

```bash
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --headless --duration 1 --record
uv run --frozen bike-recording overview output/ride/SELECTED_RUN --json
```

The documentation's final example replaces `SELECTED_RUN` with the actual published path from that verification; do not claim a missing distributed profile was tested. Cover final-only RAM behavior, Ctrl-C/error evidence, explicit check policy, unique run/episode IDs, SQL field lookup and legacy exports. Explain opt-in as a user-visible default change.
- [ ] Measure off/on capture wall time and peak RSS separately from final export and query time; use the same fixed-input profile, timestep, duration, strictness and artifact. This is one bounded performance observation, not a reason to launch many physics campaigns or weaken tests.
- [ ] Run final stage-A verification once the code stabilizes:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
```

- [ ] Review correctness, sample ownership/exception safety, numerical passivity, field semantics, tool health and measured cost separately. Record selected extension/library identity, exact failures/counts, viewer evidence and dataset paths in the audit. A required failure remains a red gate. Proceed to stage B after the accepted stage-A gate; do not claim the entire project complete at this checkpoint.

Completion: ordinary native headless/viewer produce sealed full-step recordings only when requested, preserve execution behavior, and pass the required integration gate. Checkpoint: `docs: document and verify ordinary native run recording`.
