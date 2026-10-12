# Run-recording source completion and handoff - 2026-10-13

## Scope and evidence level

This delivery implements the source changes for both stages of
[the master plan](../plans/2026-10-12-run-recording.md) and
[design R01-R24](../specs/2026-10-12-run-recording-design.md). The user explicitly
requested **no compilation and no execution**. Project code, import probes,
linters/analyzers, syntax compilation, tests, simulations, GUI and benchmarks were
not executed. Only repository unpacking, text inspection, file editing and
packaging utilities were used.

**Source implementation is not runtime acceptance.** A1-A5 and B1-B3 have source
and regression changes; A6/B4 documentation and acceptance instructions are
provided, but all execution-dependent criteria remain **NOT RUN**. The original
plan checkboxes are not converted into false green gates. Work on stage-B source
was performed under the no-execution instruction, not after a claimed green A6.

Input: `repomix-output.xml(3).zip`, containing `repomix-output.xml`. Unpacking
restored 1,071 original text files. The immutable extraction was kept separately
from the edited tree. No Git metadata, dependency lock, compiled native artifact
or otherwise omitted asset was manufactured. Existing `output/` and unrelated
plans/skills were preserved. `README.md` was absent despite being referenced by
`pyproject.toml`; a navigation/status README is supplied in this delivery.

The [2026-10-12 audit](2026-10-12-run-recording.md) is historical evidence from the
original checkout. Its 201 failed / 2825 passed / 1 skipped result is preserved,
not rerun, repaired or attributed to this delivery. No unrelated numeric-domain
failure or platform-baseline test was weakened.

## Implementation checkpoints

| Task | Source deliverable | Authored/adapted regression sources | Execution status |
| --- | --- | --- | --- |
| A0 | Preserve supplied baseline; identify repomix/checkout limitations | Original A0 audit retained | No new baseline run |
| A1 | Immutable contracts, transactional capture/journal, reviewed catalog, explicit nested/presence encoding; pinned dependencies retained | `test_recording_capture.py`, `test_recording_catalog.py`, `test_recording_values.py` | NOT RUN |
| A2 | Independent retention policy, all-interval native owner, lazy unflattened bounded `sample_batch`, sealed lifetime | `test_native_recording_retention.py`, `test_recording_values.py` | NOT RUN |
| A3 | Ordinary opt-in CLI, check policy, request/application events, setup-time assets, final-only lifecycle, primary-error preservation | `test_recording_cli.py`, `test_native_recording_lifecycle.py`, `test_frontend_windows.py` | NOT RUN |
| A4 | DuckDB schema/Arrow ingestion, metadata/catalog/coordinate/reference/presence validation, read-only reader, exclusive manifest-last publication/retry | `test_recording_storage.py`, `test_recording_publication.py`, `test_recording_values.py` | NOT RUN |
| A5 | Bounded analyst CLI, windows, exact/summary comparison, explicit export and deterministic report | `test_recording_analysis.py`, `test_recording_report.py` | NOT RUN |
| A6 | `docs/RECORDING.md`, updated `docs/RIDE.md`, this ledger | Ordinary tests above; real GUI and full acceptance still required | Source/docs supplied; acceptance NOT RUN |
| B1 | Python/native off/full/legacy/verify, immutable research evidence, completed/physical prefixes, prepare-before-ack suffix | `test_research_capture.py`, `test_native_research_capture.py` | NOT RUN |
| B2 | Both entrypoints/backends use one launch owner; reset/terminal RAM capture, stop drains only accepted window; normalized streams | `test_research_recording_lifecycle.py`, `test_research_capture.py`, parser/window tests | NOT RUN |
| B3 | Independent legacy/new replay-input adapters, explicit episode selection, unchanged checked verifier, compact verify mode, one-snapshot legacy export | `test_replay_recording_v3.py`, `test_native_replay_recording_v3.py`, adapted legacy session tests, existing legacy recording tests | NOT RUN |
| B4 | Cross-mode source integration, compatibility/default-change docs, full requirement map and deferred acceptance ledger | All focused families and repository `full` profile | Source/docs supplied; acceptance/performance NOT RUN |

Tests in this table are under `tests/reference/`. A filename is a source reference,
not evidence of a passing test. Existing test families remain included rather than
being replaced by a smaller synthetic acceptance claim.

## R01-R24 coverage map

| Requirement | Implementation location and concrete behavior | Remaining evidence |
| --- | --- | --- |
| R01 | `recording/contracts.py`, `capture.py`; launch UUID/UTC/monotonic origin, consecutive episode IDs and separate generation | Identity/reset regressions NOT RUN |
| R02 | `cli/ride.py`, `cli/research.py`, physical/research frontends; off by default, no implicit output/history, explicit unsupported-mode errors | CLI/off-mode regressions NOT RUN |
| R03 | `physical_frontend.py`, `research/recording_lifecycle.py`, both viewers; outgoing seal before reset, finalization at launch end only | Mocked and real-viewer evidence NOT RUN |
| R04 | Capture state machine, transactional publisher, zero-step startup/policy failures, physical versus completed-control extents, failure events and exception notes | Fault injection/error/interrupt tests NOT RUN |
| R05 | Capture branches separated from existing integration/control/sensor arithmetic; fixed-input on/off tests authored for Python/native | Numerical passivity NOT RUN; no equality claim based solely on source |
| R06 | Existing measurement cadence unchanged; independent native retention; fresh/forwarded energy provenance and mass scope in catalog/channels | Nontrivial-decimation passivity/coverage NOT RUN |
| R07 | Native shared sample payloads, one canonical physical index, owned episode values, lazy bounded boxing; Python immutable byte-backed arrays; suffix acknowledgement after boxing | Reset/close/allocation/boxing tests NOT RUN |
| R08 | Catalog-directed physics/forces/domain tables preserve incoming vectors, component forces/powers and diagnostic channels | Full payload/native round trips NOT RUN |
| R09 | `recording/catalog.py`, `schema.py`; finite reviewed fields, explicit units/frames/phases/children/shapes, qpos/DOF mapping, aliases as views | Catalog/coordinate fixtures NOT RUN |
| R10 | `encoding.py`, `json_codec.py`, schema masks and reader presence checks; NULL/missing/false/zero/empty/nonfinite/-0 kept distinct | Typed SQL and JSON boundary tests NOT RUN |
| R11 | Raw receipt/processing/application journal plus requested/applied control streams; derived facts use `channel_value_change.v1`, never invented controller reason codes | Operator/control ordering tests NOT RUN |
| R12 | `recording/assets.py`; setup-owned track/terrain/config/rider bytes and recipe, final identity-change checks, hashes and strict bundle allowlist | Asset/source-change and replay checks NOT RUN |
| R13 | `recording/research.py`, Python/native environment adapters; separate request/applied/observation/transition extents and state pair | Sensor/noise/control-prefix equivalence NOT RUN |
| R14 | `RunCapture` owns immutable `EpisodeEvidence`; physical and research sources expose bounded typed batches independent of engine lifetime | Owner lifetime/failure retry NOT RUN |
| R15 | `recording/schema.py`, `storage.py`; typed DuckDB streams, normalized observation references, field/coordinate/alias catalogs, assets, metrics, views | SQL types/relationships/presence round trips NOT RUN |
| R16 | Final-only staging, transaction/COMMIT/CHECKPOINT/close, exclusive destination/hardlinks, hash checks, manifest last; report failure distinguished | Insert/commit/checkpoint/close/payload/manifest injection NOT RUN |
| R17 | `recording/report.py`; deterministic saved-data report, extent/failure/event references, measured extrema, final cumulative values and coverage | Report reproducibility/integral fixtures NOT RUN |
| R18 | `cli/recording.py`; overview/fields/events/window/sql/compare/export, 200-row default, tags, explicit streaming export and read-only external-access guard | CLI/readonly/truncation/export tests NOT RUN |
| R19 | `recording/analysis.py`; exact grid/coordinate/phase/provenance compatibility, explicit tolerances, invalid matches, first/max deviations/unmatched extents; separate summary mode | Independent comparison fixtures NOT RUN |
| R20 | `recording/replay_input.py`, `research/replay.py`, `replay_session.py`, `cli/replay.py`; container 1 / replay 3, schema-1/2 adapter, episode selection, all shared checks and no policy execution | Old/new Python/native end-to-end replay NOT RUN |
| R21 | Native full mode omits second research sample/column history; one bounded sample read feeds all export domains; explicit legacy export owns one snapshot; verify consumes applied suffix | Retention/one-pass/suffix tests authored; performance NOT MEASURED |
| R22 | Explicit strict/diagnostic resolution in both CLIs; historical defaults, fatal handling and sticky invalid evidence retained | Parser and stop-policy passivity NOT RUN |
| R23 | Regression sources and acceptance commands supplied; no test/analyzer skip or tolerance relaxation introduced | All required execution gates OPEN |
| R24 | Reviewable full-tree archive/patch/docs; measurement conditions and required outputs documented below | Capture/export/RSS/size/query benchmark NOT RUN |

Paths without `src/bike_sim/` above are relative to that package, except native
files and explicitly named tests/docs. Native changes remain in the existing
runtime translation units; no build graph execution was needed to edit them.

## Static review corrections included

The final source review corrected repeated native sample boxing across domains,
late reads of mutable input files, source-history copies in legacy export,
repeated full applied-vector reads in replay, source advancement after sealing,
policy-load failure before launch ownership, zero-step ordinary startup failure,
immutable NumPy backing, parent/leaf presence contradictions, requested/applied
versus completed-transition extents, and stale tests expecting automatic CSV/HTML.

Old viewer/export tests were updated only where the approved opt-in contract
changes the expected behavior. Explicit legacy exporter tests and the original
source/artifact/replay checks remain. CLI default changes are documented rather
than hidden behind automatic native selection or implicit recording.

Controller/sensor clocks, RNG algorithm, physical timestep and final-state
comparison tolerances are not intentionally changed. That statement is a source
review assessment; it is not a substitute for the required numerical test runs.

## Acceptance commands retained for the complete supported checkout

These commands are **references only and were not executed**. Respect the
repository's sequential execution/tool-health rules and the selected artifact.

```bash
# Isolated storage, values, publication, report and CLI parsing.
bash tools/run_tests.sh quick \
  tests/reference/test_recording_capture.py \
  tests/reference/test_recording_catalog.py \
  tests/reference/test_recording_cli.py \
  tests/reference/test_recording_storage.py \
  tests/reference/test_recording_publication.py \
  tests/reference/test_recording_values.py \
  tests/reference/test_recording_analysis.py \
  tests/reference/test_recording_report.py

# Python research, launch lifecycle and current-identity old/new replay.
bash tools/run_tests.sh quick \
  tests/reference/test_research_capture.py \
  tests/reference/test_research_recording_lifecycle.py \
  tests/reference/test_replay_session.py \
  tests/reference/test_replay_recording_v3.py

# Selected-artifact native intermediate scope.
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh native \
  -k 'recording or research_capture or replay_session'

# Final native/realtime integration gate after the focused failures are resolved.
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
```

Neither dependencies nor a matching extension were loaded as a shortcut to these
gates. The supplied lock is absent, so dependency resolution and lock consistency
are also unverified. Repository test commands include build/tool phases; they were
not invoked under the no-execution instruction.

Real GUI acceptance must separately exercise off/on recording, brake/strength/
playback changes, two resets, terminal hold, Ctrl-C/window close and post-close
three-episode publication. Mock-window tests do not establish actual GUI behavior.

## Performance evidence still absent

For each backend/mode being accepted, preserve the same supported reference
profile, explicit duration, seed, measurement cadence, check policy, timestep and
nonbinding wall cap. Capture fixed step-indexed input, not wall-time interaction.
Record exact source/artifact/compiler/engine/dependency identity and host CPU/RAM.
Measure setup, capture-off and capture-on advancement, final export and
representative bounded queries separately. Record peak RSS, interval/control
counts, physical validity/outcome, artifact size and the actual published path.
Do not infer a speedup, zero-copy claim or acceptance threshold from old exploratory
CSV timing. None of these measurements has been made for this delivery.

## Handoff state

Source and documentation changes are delivered for review. Original A6/B4 full,
GUI and performance gates remain open. Historical failed counts remain historical;
new pass/fail counts, runtime reproduction logs and timing numbers do not exist.
The archive is an edited reconstruction of the supplied text snapshot, not a
newly verified native release or a replacement for omitted binary/ignored assets.
