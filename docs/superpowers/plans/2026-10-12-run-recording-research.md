# Research Recording and Replay Implementation Plan — Stage B

> **2026-10-13 delivery:** ordinary and research source changes and regression sources are supplied. See the [task/R01-R24 source-completion ledger](../audits/2026-10-13-run-recording-source-completion.md) and [usage/migration guide](../../RECORDING.md). No builds, imports, tests, analyzers, simulator/GUI runs or benchmarks were executed. Execution-dependent completion checkboxes below remain open; this is not an accepted A6/B4 runtime gate. Stage-B source was completed under the no-execution instruction, not after a claimed green stage-A run.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate both research backends to the common final-only recording and preserve old/new checked replay while reducing duplicate retention and repeated history materialization.

**Architecture:** Research keeps its sample-and-hold scheduler, sensor pipeline and control acknowledgement. Its adapter seals physical and research evidence into the stage-A launch owner. Versioned replay-input adapters provide the same verifier with logical records from legacy bundles or the new database.

**Tech Stack:** Stage-A recording package, existing native/Python research environments and ReplaySession, DuckDB 1.5.6, PyArrow 26.0.0, C++23/nanobind/MuJoCo 3.12.0, pytest through uv.

**Spec:** [Run recording design](../specs/2026-10-12-run-recording-design.md), especially R02–R07/R13–R16/R20–R24. Begin after [stage A](2026-10-12-run-recording-ordinary.md) is accepted. Both stages belong to one implementation objective.

## Global Constraints

- Invoke Python and Python tools through `uv`.
- Use Python >=3.13, C++23, nanobind, and MuJoCo 3.12.0 for native execution.
- Preserve valid-input binary64 operation order, physics timestep, controller/sensor scheduling, and existing comparison tolerances.
- Store physical samples at every accounted physics interval when recording is enabled; do not downsample or quantize the canonical recording.
- Collect the whole launch in RAM and write only during launch finalization.
- Make persistent recording opt-in with `--record`, default false.
- Keep database access, Python boxing, report generation, and disk I/O outside realtime kernels.
- Preserve user-owned changes, unrelated plans, skills, and output; do not delete or overwrite an existing run.

## B1. One research evidence owner and suffix access

**Files:** Modify `native/src/runtime/research.hpp`, `research.cpp`, `research_binding.cpp`, `recording.hpp`, `recording.cpp`, accounting/runtime retention interfaces from A2; `src/bike_sim/native/research.py`, `src/bike_sim/sim/research/environment.py`, `backend.py`; create `src/bike_sim/recording/research.py`; extend `recording/contracts.py`, `catalog.py`; create `tests/reference/test_native_research_capture.py`, `test_research_capture.py`; extend `test_native_research_failures.py`, `test_native_research_recording.py`.

**Interfaces:** Define the forward type referenced by A1 and preserve a small shared evidence seam:

```python
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Protocol
import numpy as np
from bike_sim.recording.contracts import EpisodeEvidence, TableBatch

class ResearchSource(Protocol):
    def tables(self, *, batch_rows: int) -> Iterator[TableBatch]: ...

@dataclass(frozen=True)
class ResearchEvidence:
    history: ResearchSource
    initial_integration_state: np.ndarray
    final_integration_state: np.ndarray
    replay_contract_version: int
    complete_control_steps: int
    replay_eligible: bool

# Implement on both ResearchEnvironment and NativeResearchEnvironment:
# seal_episode(episode_id: int, *, reason: str) -> EpisodeEvidence
# applied_since(cursor: int) -> tuple[Sequence[Mapping[str, Any]], int]
```

`ResearchEvidence` is defined in `recording/contracts.py`; adapter/source implementations live in `recording/research.py`. Integration arrays own immutable backing; history owns native backing or frozen Python data, not the environment. `applied_since` returns only the newly available suffix and a cursor scoped to one episode. Invalid cursor type/range fails before mutation. Replay uses a transient verification policy that retains required newly applied events without enabling persistent run recording. In verification mode, prepare and box the complete return value before acknowledging the consumed suffix; retain the absolute cursor base so stale cursors reject explicitly. Full/legacy capture retains its canonical journal independently of that transient acknowledgement.

- [ ] Pin an independent Python/native logical trace before refactoring: requested/applied command order, startup input, initial observation, acquisition/delivery times, delayed input, truth, metrics, termination, validity and integration state. Use existing `environment_pair`, `CountingPolicy`, sensor and program fixtures. Include decimation 80 with full recording on and off, explicit strict/diagnostic settings and nonzero sensor noise/dropout.

```python
from bike_sim.sim.ride.control import RideControl

def test_applied_suffix_is_once_only(environment_pair):
    reference, native = environment_pair(duration=.04)
    for env in (reference, native):
        env.step(RideControl(0.))
        first, cursor = env.applied_since(0)
        again, unchanged = env.applied_since(cursor)
        assert len(first) >= 1
        assert len(again) == 0 and unchanged == cursor
```

Also assert actual applied payload/times against the independent expected startup and actuator-delay sequence; the count test alone is insufficient. Existing explicit programmatic fixtures keep legacy retention unless a test requests the new mode.
- [ ] Add capture policy plumbing to both backends without altering noise tape, clocks, sensing or command composition. The native physical runtime retains the single full sample index from A2; research evaluation consumes all required pending intervals but does not build another copy of full columns/samples. Off mode retains running metrics, active window and sensor/actuator queues; a replay verifier requests bounded transient applied-event evidence explicitly.
- [ ] Replace duplicate `ResearchRecorder` columns/index on the new capture path with the shared immutable physical source. Keep legacy `recorded_columns()/recorded_intervals()` as lazy compatibility projections while their explicit users remain supported. Do not claim the old code duplicated every payload: its samples already shared `shared_ptr`; the improvement is redundant indices/columns and conversion work.
- [ ] Implement one-time owned research sealing at a completed control seam or a failure prefix. Retain already committed physical suffix separately from completed transitions. Preserve `delivery_pending` and boxing-before-ack behavior: a failed Python result conversion is retryable and must not repeat physics, policy or history append. Sealing a pending external control window does not invent a complete transition or invoke another action.
- [ ] Implement `applied_since` in native binding and Python environment with stable episode-local cursor semantics. Keep full-history properties for explicit compatibility calls, but final export and ReplaySession use the new evidence/suffix paths. The native reader uses the same ownership/busy guard as current history reads and does no I/O.
- [ ] Add source-lifetime and failure regressions: seal/reset/close then export; all initial/final arrays remain unchanged; forced result-boxing failure then retry; requested-input staging failure; interrupted control window; policy error; poisoned engine prefix; empty episode. Distinguish committed samples, completed control transitions and replay eligibility in assertions.
- [ ] Verify B1:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh native -k 'research_capture or research_failures or research_recording or research_sensors or research_programs'
bash tools/run_tests.sh quick tests/reference/test_research_capture.py
```

Completion: both backends expose equivalent owned evidence, current retry/clock/sensor contracts pass, no duplicate full recorder exists on the new path, and replay can consume an applied suffix. Checkpoint: `refactor: share research evidence retention and expose applied suffixes`.

## B2. Research CLI finalization and normalized database streams

**Files:** Modify `src/bike_sim/cli/ride.py`, `cli/research.py`, `sim/research/viewer.py`, `policy_session.py`, `configuration.py`; adapt `src/bike_sim/native/recording.py` for explicit legacy export; extend shared capture/catalog/storage/report/research adapters; create `tests/reference/test_research_recording_cli.py`, `test_native_research_recording_lifecycle.py`; extend `test_research_frontend.py`, `test_native_research_frontend.py`.

**Interfaces:** Both CLI entrypoints pass explicit off/full capture and resolved `--check-policy` before environment construction. `make_environment(args, *, capture_mode='legacy')` and `build_environment(..., capture_mode='legacy')` keep direct programmatic users/fixtures on their legacy retention default; CLI `main`/viewer construction explicitly supplies `'full' if args.record else 'off'`. `PolicySession.seal_episode(episode_id: int, *, reason: str) -> EpisodeEvidence` adds its policy/run metadata to environment evidence without a new action. The stage-A `RunCapture` is the sole persistent launch owner. Explicit legacy `save(directory, overwrite=False)` delegates to a compatibility exporter over one sealed/read-only evidence view, not repeated native full-history properties.

- [ ] Write opt-in tests for `bike-ride --research` headless/viewer and separate `bike-research`, both backends. Without `--record`, no old bundle or new run directory is written. `--out` alone does not enable persistence. Programmatic explicit save remains tested separately. Existing `--diagnostic-model-limits` maps to the resolved check policy; conflicting explicit old/new policy flags fail clearly before stepping.
- [ ] Write finalization regressions for both entrypoints: success, Ctrl-C, caught policy/engine error, viewer reset, terminal frame hold and viewer close. The current separate `bike-research` interrupt gap must be covered. Complete only the already active control interval where existing stop policy requires it, then capture; do not call a policy to obtain a fresh command solely for saving.
- [ ] Implement one `RunCapture` over all research episodes. Resets capture the outgoing episode into RAM and preserve its initial/final state/assets; final launch shutdown writes once. Defer optional history teardown until owned sealing succeeds. Error handling preserves original exceptions and adds cleanup/publication notes rather than replacing the first error.
- [ ] Normalize database streams. Store each observation once with an integer observation ID; transitions reference the delivered observation and carry distinct evaluation truth. Store commands separately with request/application times and original/effective brakes. Preserve startup applied event, synthetic demand/program metadata, stale validity, source/delivery times and existing metrics. The catalog names privileged truth separately from policy inputs.

```sql
SELECT t.episode_id, t.control_step,
       o.source_time_s, o.time_s AS delivered_time_s,
       t.reason, t.model_valid
FROM transitions t
JOIN observations o
  ON o.episode_id = t.episode_id AND o.observation_id = t.observation_id
ORDER BY t.episode_id, t.control_step;
```

Publish this analytic view without storing a second observation payload per transition. Validate `observations = completed transitions + 1` for a normally sealed episode, with separately declared failed/incomplete-prefix exceptions; requested commands may include an unfinished window and must not be forced into a false equal-count invariant. Requested/completed/replay extents are independently recorded.
- [ ] Move legacy CSV/JSONL projections behind explicit export calls. The compatibility exporter reads a single owned source per call, stages the old fixed file set and publishes its old replay manifest last. Legacy programmatic users can still request an old bundle; normal new `--record` uses only the new container. Test one-shot source iteration to prevent accidental repeated conversion of growing histories.
- [ ] Record original and final source/artifact identities, policy identity and input bytes consistently. Bundle replay-required assets in `assets`; avoid reopening arbitrary original paths at replay. A changed source/asset invalidates exact replay eligibility without erasing the evidence needed for analysis.
- [ ] Verify focused lifecycle and schema behavior:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh native -k 'research_recording_lifecycle or research_recording or research_frontend or research_capture'
bash tools/run_tests.sh quick tests/reference/test_research_recording_cli.py tests/reference/test_research_capture.py
```

Completion: all research entrypoints follow explicit opt-in and end-only saving; one normalized recording contains all episodes and exact research timing; legacy explicit exports remain valid. Checkpoint: `feat: finalize research launches into shared DuckDB recordings`.

## B3. Versioned replay input and backward format compatibility

**Files:** Create `src/bike_sim/recording/replay_input.py`; modify `src/bike_sim/sim/research/replay.py`, `replay_session.py`, `src/bike_sim/cli/replay.py`; extend shared reader; create `tests/reference/test_replay_recording_v3.py`, `test_native_replay_recording_v3.py`; preserve/extend `test_replay_manifest_v2.py`, `test_replay_session.py`, `test_native_replay_session.py`.

**Interfaces:**

```text
load_replay_input(path: Path, *, episode_id: int | None = None) -> ReplayInput
LegacyReplayInput: existing schema-1/2 fixed bundle validation and logical input
DuckDBReplayInput: sealed container schema 1 + replay contract 3 logical input
ReplaySession(path: Path, *, episode_id: int | None = None)
replay_episode(path: Path, *, episode_id: int | None = None) -> dict
```

`ReplayInput` owns immutable metadata/recipe/assets, ordered requested/applied records, observations/transitions, metrics, initial/final state and replay extents. Define its typed fields in `recording/replay_input.py` from the existing verifier's actual accesses. Its loader parses/validates data only. Plant rebuilding remains in `sim/research/replay.py`, consuming a validated recipe, never importing the recorded policy. Add `--episode` to the CLI; omit it only for a single new episode or an existing legacy episode directory.

- [ ] Write red tests for new recording selection, sole/multiple episode rules, missing manifest/hash/asset, unsupported container/replay versions, initial/final state and completed-prefix extent. New failure recordings remain analyzable but reject successful completed replay. A nonresearch ordinary recording reports that it lacks checked-replay evidence.
- [ ] Preserve independent legacy schema-1/2 fixtures and existing checksum/path/identity failures. Format compatibility means old data can be parsed/validated; physical replay still requires its recorded source/runtime/artifact identity. Do not weaken source hashes merely to replay an older file after modifying the implementation. Generate current-identity legacy fixtures through the explicit compatibility exporter and verify them; retained historical fixtures test format/hash validation independently of current-source replay.
- [ ] Implement both input adapters. The legacy adapter delegates to the existing strict fixed-file validators. The new adapter validates the outer DB hash, asset hashes, backend/library/source identity, schema/catalog and episode extents before rebuilding. Distinguish metadata describing a valid physical prefix from a complete replayable transition prefix. Data-only paths never open referenced executable modules or unbundled original recipe paths.
- [ ] Refactor `ReplaySession._load()` to consume `ReplayInput` rather than forcing new data through temporary legacy JSON files. Rebuild from source-backed recipe/assets; compare the saved initial integration vector instead of treating it as a universal state-restore checkpoint. Retain current startup/reset logic, sensor noise sequence and policy-free requested command playback.
- [ ] Keep every verifier stage: initial observation, each transition/truth/observation, applied suffix, source/backend checks, outcome, validity/metrics and final integration state. Use the B1 cursor for applied events instead of reloading the entire native applied vector at each boundary. Final tolerances remain `atol=rtol=1e-9`; matching partial data never produces a completed-success report. Existing viewer early-close/error/success exit semantics remain.

```python
from bike_sim.sim.research.replay_session import ReplaySession

def test_new_recording_requires_full_verification_before_report(saved_research_run):
    path, episode_id = saved_research_run
    session = ReplaySession(path, episode_id=episode_id)
    try:
        while not session.done:
            session.advance()
        assert session.report()['passed'] is True
    finally:
        session.close()
```

Create `saved_research_run` in `_recording_support.py`/test fixtures during this task: use existing `environment_pair`, execute a fixed short step-indexed command sequence, seal one episode through `RunCapture`, finalize to `tmp_path`, return its path and ID, and close the original environment. Also add an early report test that must fail before full advancement. Current source exposes `advance(wall_budget_s=None, *, target_step=None)` and `report()`; the example calls these existing methods rather than introducing a second verifier.
- [ ] Verify old/new and native/Python replay:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh native -k 'replay_recording_v3 or replay_session or research_recording'
bash tools/run_tests.sh quick tests/reference/test_replay_recording_v3.py tests/reference/test_replay_manifest_v2.py tests/reference/test_replay_session.py
```

Completion: one verifier reads either supported container, old formats retain strict validation, no policy is executed during replay, and new success requires all current comparisons. Checkpoint: `feat: verify DuckDB research recordings with legacy replay compatibility`.

## B4. Cross-mode acceptance, measured optimization and handoff

**Files:** Update `docs/RIDE.md`, `docs/RECORDING.md`, the A0 audit; extend focused research/replay tests if final integration reveals gaps.

- [ ] Run sequential same-input on/off captures for research Python and native with nonzero sensor bias/noise/dropout and actuator delay. Compare trajectory, RNG cursor, observations, requested/applied command chronology, truth, first failure, outcome and metrics. Keep physical duration and measurement/check policy identical; wall caps must not bind.
- [ ] Test two reset-separated research episodes, caught policy/engine failure and Ctrl-C for both headless entrypoints. Verify output is absent until launch finalization and no post-stop policy call is introduced. After close, inspect complete-control versus physical extents and owned initial/final states.
- [ ] Verify actual native research viewer recording and new checked visual replay on the host. Include pause, brake, reset, speed change, early close, mismatch and completed success. A mocked GUI test or a legacy replay result alone does not satisfy the new GUI path.
- [ ] Measure the supported short research workload with off/on capture and final export separately. Record peak RSS, captured counts, normalized observation/transition storage, one-pass export behavior and replay time with suffix reads. Compare to the execution baseline under identical source-supported settings where possible. Source-level deduplication alone is not a measured speedup; report regressions honestly and fix unbounded repeated-copy behavior.
- [ ] Document `--record` default changes for both research CLIs, failure evidence versus replay eligibility, observation source/delivery clocks, episode selection and old format/runtime constraints. Update existing examples that rely on automatic recording to pass `--record` explicitly. Keep recording disabled during replay's own execution unless explicitly requested; verification's transient evidence is not another persistent run.
- [ ] Execute final project gate after all relevant changes:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
```

Reinspect native preflight integration and analyzer tool health. Record exact output, selected artifact, versions, counts and any failures. Required failing/absent checks remain open; old accepted platform failures are not automatically waived.
- [ ] Review the spec coverage table in the master against final implementation, ensure all R01–R24 have implementation/test or explicitly unresolved evidence, and verify docs/examples/CLI agree. Preserve unrelated work and show the final code/document diff. Mark task/stage checkboxes only after their actual gates pass.

Completion: both research backends/entrypoints share the final-only store and query/report tools; existing experiment chronology and checked replay contracts hold; old formats remain supported under their identity rules; the full required gate and claimed GUI/performance paths have current evidence. Checkpoint: `docs: verify shared research recording and replay workflow`.
