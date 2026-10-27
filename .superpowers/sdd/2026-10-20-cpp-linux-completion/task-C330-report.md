# Task C330 — test-port 3.30 report (recording lifecycle, retention, capture, publication)

**Batch**: 3.30 — `tests/reference/test_native_recording_lifecycle.py`,
`test_native_recording_retention.py`, `test_recording_capture.py`,
`test_recording_publication.py` (74 records / 78 collected nodes;
two parametrized records expand the count).
CTest ids `reference.{native_recording_lifecycle,native_recording_retention,
recording_capture,recording_publication}`; labels `t53 recording`;
port-map rows carry `owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e`, base `f65906c7`
(F5 merge-back). No merge, no push.

## Status: landed — 67 `port`, 7 `drop`, 0 `planned`

67 of 74 records ported into four new CTest executables plus one shared
fixture header. Every ported check compares native state against native
state, native contract text, or literal structure — **no golden fixtures
were needed or created** (zero `_golden` / live-oracle comparison sites
in the four source files). Seven records are `drop` (`python-only`) —
Python-frontend-only emission kinds, fake-driver-only injection seams,
a console-trailer text pin, the Python adapter's export file bundle and
a Python type-state rejection. No record stays `planned`: the source
suite is fully green (78/78) and no live-oracle red family exists in
this batch.

## Triage

| Source file | records | collected nodes | port | planned | drop |
|---|---|---|---|---|---|
| `test_native_recording_lifecycle.py` | 31 | 32 | 26 | 0 | 5 |
| `test_native_recording_retention.py` | 17 | 17 | 16 | 0 | 1 |
| `test_recording_capture.py` | 24 | 24 | 23 | 0 | 1 |
| `test_recording_publication.py` | 2 | 5 | 2 | 0 | 0 |
| **total** | **74** | **78** | **67** | **0** | **7** |

## What the ports reproduce

- **`reference.native_recording_lifecycle`** (26 tests): derived-
  transition contracts (`derived_transition` kind,
  `channel_value_change.v1` rule, flattened member paths, journal cursor
  continuation), journal sequencing/validation, Python-backend record
  rejection, viewer two-episode publish-once, `ResearchRun` seal dedupe
  for duplicate terminal notices, native pump reset-generation sealing
  with real channel diffs, interrupt-shielded deferred interrupts
  (seal/finalize/normal windows), `finalize_guarded` re-raise after
  commit, strict failure-prefix sealing, and two real
  `cli::ride_command` headless runs (welded recipe publication; the
  `weld_foot_min_normal_n` violating recipe's sticky first failure).
- **`reference.native_recording_retention`** (16 tests): full retention
  under `measurement_decimation=80` (every committed interval),
  legacy/off projections, provenance (`measured`/`source_interval_id`/
  `source_time_s` forwarding), partial-period and empty-run sealing,
  `episode.batch` range validation, typed sample fields and
  `powers_w == forces·qvel`, `channel_mass`/`channel_energy`/
  `forces` table emission with presence-marker NULLs, batch/channel-root
  validation, non-finite encoding, pending-acknowledgement independence,
  strict-rejection retained evidence, reset/close ownership, and the
  record-on/record-off trajectory-identity invariant.
- **`reference.recording_capture`** (23 tests): contiguous journal ids,
  `emit_all` bulk emission, wrong-episode/skipped-id rejection, metadata
  ownership (post-capture caller mutation cannot reach stored values),
  disabled capture no-op publication, publication caching vs retry,
  state machine (`collecting`→`published`/`publication_failed`),
  interrupt-shield note/deferral semantics.
- **`reference.recording_publication`** (2 tests): `FinalizeOptions::
  before_stage` fault injection at `insert`/`COMMIT`/`CHECKPOINT`/
  `manifest_link` stages — failure leaves evidence intact and the
  capture retryable into a fresh destination, and unrelated files
  survive a failure at `link_run.duckdb`.

## Port-only support

`native/tests/reference/recording_test_support.hpp` — synthetic
8-joint catalog/rows/episodes (`OwnedNativeEpisode::seal`),
deterministic `run_metadata()`, the welded-bootstrap staging helper
(`welded_runtime`), storage-valid full `drive`/`tires` channel builders
(`storable_channels`), `violating_physics` TOML staging, and
evidence/source builders shared by the four suites.

## Confirmed defects / staging traps found during the port

1. **Staged bootstrap layout (fixture semantics).**
   `bootstrap/welded/state.json` declares array payloads
   capture-root-relative (`bootstrap/welded/state.arrays/*.f64`) and
   `resolve_declared` climbs ancestors. A flat temp-dir copy breaks
   resolution; the staged copy must preserve the
   `<root>/bootstrap/welded` layout (`recording_test_support.hpp`).
2. **config.json ↔ state.json agreement.** Re-staged
   `measurement_decimation`/`strict` config values must be mirrored in
   `state.json` (`state.runtime.measurement_decimation`,
   `state.runtime.monitor.strict`) — `NativeRideRuntime` rejects
   disagreement at load.
3. **Dangling reference into a temporary (test bug, real UB).**
   `sw::object(sw::required(runtime->accounting_state(), "history"))`
   bound a reference into a destroyed `WireObject` — nondeterministic
   `missing field 'last_id'`. Fixed by naming the `accounting_state()`
   result (`test_native_recording_retention.cpp`).
4. **Struct presence encoding.** `channel_mass.endpoint_mass` is a
   struct column; Python row-membership `'endpoint_mass' in row` maps to
   the `endpoint_mass_present` marker column (`Piece::b`), not the
   struct `valid` mask — the ported assertion reads the marker.
5. **`record=False` is not a retention mode.** The oracle's
   `PhysicalRideDriver(record=...)` toggles driver-level capture; the
   ported trajectory-identity check uses two identical full-retention
   runtimes, one drained through `prepare_samples`/`acknowledge_samples`
   like the `RunCapture` consumer.
6. **Published synthetic episodes need storage-valid channels.**
   `channel_drive`/`channel_tires` claim their channel roots wholesale —
   a row carrying `channels.drive` must carry all 51 non-optional fields
   (and `tires.front/rear` their struct members). `storable_channels`
   keeps only the plan-watched fields varying so the derived-transition
   change sets stay exact.
7. **`ResearchRun` requires `capture_mode == full`.** The
   `delayed_bootstrap` fixture is `legacy`; the port prepares a
   `full`-capture environment from `requests/research_delay.toml` (the
   t36 path).
8. **Welded recipe always latches `energy.constraint_work`.** Welded
   feet do real constraint work → `constraint_work_ok` fails at the
   first 5 ms checkpoint → `model_valid=false` → exit 2 on every welded
   physical run. The "healthy" headless port asserts the honest native
   outcome (seal+publish, `duration_reached`, exit 2); no physical
   recipe exits 0 (pneumatic needs NPZ provenance staging, distributed
   emits catalog-unknown tire members).
9. **`violating_physics` siblings.** The staged TOML's
   `joint_envelope_path`/`joint_strength_path` are relative to its own
   directory — the JSON profiles are copied alongside (`platform::
   open_regular` failure otherwise). `--seed` was also dropped from the
   headless args (the track TOML has no `[generator]` block).

## Verification evidence

- **Oracle green (darwin-arm64)**: `uv run pytest
  tests/reference/test_native_recording_{lifecycle,retention}.py
  test_recording_{capture,publication}.py` — **78 passed**, 4 warnings
  (49.77 s), `BIKE_NATIVE_BUILD_PATH=native/build`. The linux oracle run
  was not required: no record compares against the live oracle and no
  golden capture sites exist in this batch.
- **CTest darwin-arm64**: `ctest -R
  'reference\.(native_recording_lifecycle|native_recording_retention|
  recording_capture|recording_publication)'` — **4/4 Passed**
  (18.18 s; lifecycle 11.85 s carries the two real `ride_command` runs).
- **CTest linux-aarch64** (`bike-dev-pfc-lane-e`, committed tree synced
  via `git archive`): same scope — **4/4 Passed** (22.73 s).
- **Red→green proof (no goldens in batch → byte-flip N/A)**: flipped the
  disabled-capture expectation `State::published`→`State::collecting`
  (`test_recording_capture.cpp:44`) — `FAIL disabled capture reports
  published without a path`; reverted — `PASS 61 checks`. Asserts fire.
- **Static registration**: `Phase3Ports.cmake` — four
  `bike_add_reference_test` entries, `LABELS t53 recording`.

## Ledger

- `migration/test-port-map.json`: 67 `port` (contract), 7 `drop`
  (python-only) — measured `collected_node_ids`, `assertions_capture`
  and per-record evidence for all 74 rows.
- `migration/python-removal.json`: the four source rows →
  `status: replacement_landed`, `verification_ids` set to the CTest ids;
  deletion stays gated at T63.

## Commits

| SHA | Unit |
|---|---|
| `f291acf0` | 3.30 T53: native recording lifecycle/retention/capture/publication ports — 67 port + 7 drop, darwin-arm64 green |
| (this commit) | Phase3Ports comment correction (no red family exists), ledger linux-qualification evidence, C3.30 report |
