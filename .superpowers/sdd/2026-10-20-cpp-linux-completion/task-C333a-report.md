# Task C333a — test-port 3.33a report (native research)

**Batch**: 3.33a — `tests/reference/test_native_research_capture.py`,
`test_native_research_environment.py`,
`test_native_research_failures.py`,
`test_native_research_frontend.py`,
`test_native_research_policy.py`,
`test_native_research_programs.py`,
`test_native_research_recording.py`,
`test_native_research_sensors.py`
(41 records / 78 collected nodes; parametrized records expand the
count). CTest ids `reference.native_research_{capture,environment,
failures,frontend,policy,programs,recording,sensors}`; labels `t53
research` (+ `golden` on the two golden targets); port-map rows carry
`owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-a`, branch `lane/a`, base `fe5edb2a`
(lane/a 3.31 merge-back tip). New shared seam
`native/tests/reference/research_reference_support.{hpp,cpp}` maps the
Python `environment_factory` knobs onto the established fast
flat-track recipe (dt .0005 physical + crank_effort + lumped rider +
compliant_2d tires + equilibrium cache off) and carries the staged
reseed, sensor flatteners, `CountingPolicy`, `Directory`, and the
`invalid_with`/`throws_runtime`/`thrown_message` fragment helpers.

## Status: landed — 24 `port`, 5 `golden`, 12 `planned`

Every recipe-agnostic contract in the eight files is now a native CTest.
The five oracle-comparison records replay per-platform goldens
captured through new `BIKE_GOLDEN_CAPTURE` sites in the two source
files that have oracle legs (`test_native_research_programs.py`,
`test_native_research_sensors.py`). The twelve `planned` records are
live-oracle parity legs gated on the documented welded-physics
baseline drift or Python-adapter-only surfaces with no native hook;
each carries a row-specific evidence string in
`migration/test-port-map.json`.

## Triage

| Source file | records | collected nodes | port | golden | planned |
|---|---|---|---|---|---|
| `test_native_research_capture.py` | 2 | 2 | 1 | 0 | 1 |
| `test_native_research_environment.py` | 14 | 20 | 4 | 0 | 10 |
| `test_native_research_failures.py` | 1 | 1 | 1 | 0 | 0 |
| `test_native_research_frontend.py` | 1 | 1 | 1 | 0 | 0 |
| `test_native_research_policy.py` | 7 | 14 | 7 | 0 | 0 |
| `test_native_research_programs.py` | 3 | 8 | 2 | 1 | 0 |
| `test_native_research_recording.py` | 7 | 9 | 6 | 0 | 1 |
| `test_native_research_sensors.py` | 6 | 23 | 2 | 4 | 0 |
| **total** | **41** | **78** | **24** | **5** | **12** |

- **`reference.native_research_capture`** (1 test): the `verify`
  capture cursor protocol — `prepare_applied_since(0)` delivers the
  startup row at cursor 1, an un-acked prepare re-delivers the
  identical committed prefix (the boxing-failure analog: the native
  read is pure const, so an un-acked suffix cannot be lost by
  construction), `acknowledge_applied` erases the prefix, a stale
  cursor is rejected, a later step produces a suffix, verify retention
  holds no stream history, `last_requested` keeps the submitted motor
  torque.
- **`reference.native_research_environment`** (4 tests): one external
  step commits exactly `control_steps` physics steps (the retained
  half of `test_native_loop_never_calls_python_interval_or_rng` — the
  no-Python-calls half is vacuous in pure C++); rejected boundary
  calls (out-of-domain brake, double begin, negative/NaN budgets,
  reset while pending) never touch state and the queued stop resolves
  to `operator_stop` — the source's `close()`-while-pending
  `RuntimeError` leg is dropped: `NativeResearch.close` guards
  `control_pending` adapter-side (research.py:450), while
  `NativeResearchRuntime::close()` (research.cpp:606-611) has no
  pending guard and permits deferred close by design; the zero-budget
  poll keeps the window latched
  with `step == 0`, `recorded_rows == 0`, one requested + one
  observation and no tail flush, then the window commits and a
  further `advance_control` is refused; a stop after a
  duration-ended episode preserves the `duration` reason.
- **`reference.native_research_failures`** (1 test): the injected
  engine failure (`test_fail_at_step(3)`) preserves the committed
  prefix — `step == recorded_rows == 3`, reason `simulation_error`,
  error string carrying the `RuntimeError: ` prefix, no pending
  control and no fabricated transition — and a staged `reseed`
  recovers a full window at a bumped generation.
- **`reference.native_research_frontend`** (1 test): a partial tick
  delivers to step 1 and keeps the pending latch, zero-budget polls
  are pure, queued brakes apply to the *next* requested command (the
  latched first command keeps `front_brake_demand == 0.`), and a
  queued stop drains only the active window to `operator_stop`.
- **`reference.native_research_policy`** (7 tests): one `act` per
  window across unbounded and budgeted yields, queued pause/brakes
  latching at the boundary, pause/resume never advancing simulated
  time, stop/reset deferred to the external boundary with policy
  reseeding `{"19","47","53"}`, rejected operator values never
  reaching the policy, `act` failure terminating as `policy_error`
  with no manufactured transition, and a policy command that smuggles
  rider inputs rejected as `policy_error`.
- **`reference.native_research_programs`** (3 tests): the rider
  program knot matrix — smoothstep interpolation with reaction delay
  across `effort ∈ {None, 0., 12.5} × offsets ∈ {None, (0,0)}` at ten
  probe times including `nextafter(.035, ±inf)` — replays a (10,18)
  flattened control/demand golden per param case; no-program
  preservation of `None` fields vs explicit zeros; mixed
  effort/posture ownership rejection plus a malformed nonuniform
  ownership wire.
- **`reference.native_research_recording`** (6 tests): empty episode
  export + no-overwrite refusal leaving the manifest untouched;
  failed staging (blocking file + destination symlink) preserving an
  existing recording byte-for-byte; `replay.json` written last as the
  publication seal; opaque recorded policy provenance surviving
  `load_legacy_input` without module resolution; pending external
  window export refused (`std::runtime_error`, "finish the external
  control interval" — the adapter's `...at its boundary` wording is
  Python-side); changed execution identity flagged by
  `identity_receipt` across `execution_identity` /
  `executable_sha256` / `mujoco_library_sha256` with the stored
  startup identity preserved in summary and manifest.
- **`reference.native_research_sensors`** (5 tests): the
  noise/latency/dropout/stale grid — 18 param cases replaying
  flattened 12-wide observation rows + state rows (50 recorded values
  per node); imported startup state replayed from the *recorded*
  Python `state_dict` through `unflatten_state` → `import_state`
  (cursor not re-drawn); irregular delivery schedule; same-seed reset
  repeatability + different-seed divergence; the native-side tape
  ownership and invalid-operation exhaustion contracts.

## Confirmed traps found during the port

1. **The pending-window export refusal lives in two places.** The
   Python adapter guards `env.control_pending` first and raises its
   own `...at its boundary` `RuntimeError`; the native twin reaches
   `NativeResearchRuntime::legacy_snapshot()`, which throws a plain
   `std::runtime_error` (`finish the external control interval before
   exporting`) *before* `capture_legacy`'s `RecordingError` check is
   reached. The port asserts the native class + native fragment; the
   `boundary` wording is adapter-side.
2. **Failed staging needs a real filesystem failure.** A missing
   parent is auto-created by `export_legacy`'s `create_directories`;
   the injection uses a regular file as a parent component plus a
   destination symlink (`destination_check` refuses), and asserts the
   pre-existing destination survives byte-for-byte.
3. **`execution_changed_during_run` compares a named field set.**
   The Python adapter flags `current_execution != self.execution` on
   the whole `execution_provenance` dict (params
   `native_source_sha256`/`extension_sha256`/`mujoco_library_sha256`);
   the native `identity_receipt` compares `execution_identity`,
   `executable_sha256` (the adapter's extension image digest),
   `mujoco_library_sha256`, `numeric_provider`, and reports
   source drift as `source_changed_during_run`. Export also
   consistency-checks `recipe.json`'s embedded execution, so the
   injection rewrites metadata *and* the recipe asset + sha.
4. **Policy failure status stores the raw message.** The adapter's
   `env.error` decorates to `RuntimeError: <msg>`; the native status
   `error` field stores `<msg>` bare for policy failures while engine
   failures do carry the prefix — both surfaces pinned.
5. **`advance_pending(budget, target_step)` slicing.** The frontend
   partial tick maps to `advance_pending(.008, 1)`; `advance_pending(0.)`
   is the pure-poll spelling used by the budget-yield contracts.
6. **`-Wweak-vtables` on the shared test double.** `CountingPolicy`
   lives in the support header and is subclassed by two tests; the
   out-of-line `~CountingPolicy` anchor is compiled from
   `research_reference_support.cpp` into the three targets that
   instantiate it (matching the T36 per-TU convention).
7. **clang-18 Linux strictness.** `-Wmissing-field-initializers`
   (omitted on the AppleClang profile) required complete
   `IntentControl`/`StagedReset` designated initializers;
   `-Wunsafe-buffer-usage-in-container` required `subspan` slicing in
   `unflatten_state` rather than iterator+count `std::span`.
8. **Golden state import is literal, not substituted.** The imported-
   startup record freezes the Python `state_dict` in the golden as
   `input.state` (a `f64_input` entry); `unflatten_state` rebuilds the
   identical `SensorState` the binding's decoder produces, so
   `import_state` is exercised on genuine oracle state.

## Verification evidence

- **Oracle capture (darwin-arm64)**: `BIKE_GOLDEN_CAPTURE=
  tests/fixtures/golden NATIVE_TEST_BUILD_PATH=$PWD/native/build
  PYTHONPATH=tests/reference uv run --frozen python -m pytest
  tests/reference/test_native_research_programs.py
  tests/reference/test_native_research_sensors.py -q` — **31
  passed**; 27 golden node pairs written under
  `tests/fixtures/golden/darwin-arm64/tests_reference_test_native_
  research_{programs,sensors}.py/` (6 program nodes × `program_at`
  (10,18); 21 sensor nodes: 18 grid + imported-startup (3 entries
  incl. the `input.state` input) + irregular + same-seed).
- **Oracle capture (linux-aarch64, `bike-dev-pfc-lane-a`)**: same two
  files under `/opt/bike-venv` against a container-built
  `bike_native.cpython-313-aarch64-linux-gnu.so` — **31 passed**; the
  `linux-aarch64` golden dirs are `docker cp`'d back and committed.
- **CTest darwin-arm64** (`build/native-release`, AppleClang):
  `ctest -R reference.native_research_` — **8/8 Passed** (8.69 s).
- **CTest linux-aarch64** (`bike-dev-pfc-lane-a`, clang++-18): same
  scope — **8/8 Passed** (6.59 s).
- **Red→green proof (golden byte-flip, both platforms)**:
  darwin `test_program_knots_...[None-None].bin` byte 100 flipped →
  `FAIL ... label 'program_at' element 12: actual=...0000
  expected=...0100`, exit 1; restored → `PASS 1255 checks`, exit 0.
  linux `test_same_seed_....bin` byte 40 flipped → `FAIL ... label
  'sequence' element 5: actual=0x402393795f305f89
  expected=0x402393795f305f8b`, exit 1; restored → `PASS 27839
  checks`, exit 0.
- **Static**: all eight TUs compile warning-free under AppleClang and
  clang++-18 (`-Wweak-vtables`, `-Wmissing-field-initializers`,
  `-Wunsafe-buffer-usage-in-container`, `-Wdangling-gsl` all resolved
  in-source, no suppressions).

## Ledger

- `migration/test-port-map.json`: 41 rows touched in place — 24
  `port`, 5 `golden`, 12 `planned` with row-specific blocker
  evidence; stale `ported::tests::reference::...` ids corrected to
  `reference.native_research_*` on all 41; measured
  `collected_node_ids` (78 nodes) populated. Counters:
  `port` 1007→1031, `golden` 351→356, `planned` 920→891, `drop` 115.
- `migration/python-removal.json`: five source rows →
  `status: replacement_landed` with `verification_ids` set to the
  CTest ids (failures, frontend, policy, programs, sensors); capture,
  environment and recording stay `planned` while their live-parity
  records are outstanding; `sha256`/`line_count` refreshed on the
  three files edited since inventory (capture-site edits on
  programs/sensors, an earlier recording.py refresh). Deletion stays
  gated at T63.

## Commits

| SHA | Unit |
|---|---|
| `bbd2a6c1` | 3.33a T53 native research ports — 8 CTests + support seam + capture sites, darwin-arm64 green |
| `a9029caf` | 3.33a golden captures darwin-arm64 + linux-aarch64 (27 node pairs/platform) |
| (this section) | 3.33a ledger dispositions + dual-platform evidence + task report |
