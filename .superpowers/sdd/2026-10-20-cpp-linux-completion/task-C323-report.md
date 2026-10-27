# Task C323 — test-port 3.23 report (monitors, energy, acceptance)

**Batch**: 3.23 — eight `tests/reference/` files, 53 records. Labels
`t52 physics` (+ `golden` for `reference.sensor_noise`, `slow` for
`reference.realistic_pedelec_acceptance`); port-map rows carry
`owner_task: T52`. The handoff title's "T53 integration" conflicts with
the plan table — T52 owns 3.12–3.24, T53 owns 3.25–3.36 — so the batch
ships `t52` labels.

**Lane**: `.worktrees/pfc-lane-c`, branch `lane/c`, base `934fc9da`.
No merge, no push.

## Status: 49 of 53 records landed green; 4 pedelec records `planned` under the Task-3.09 physics stop rule

Commits on `lane/c`:

- `6fe7ab22` — `refactor(runtime): publish constraint partition +
  numerical powers for oracle parity` (step.cpp/.hpp only; the
  energy-ledger port audits the published grouping instead of a
  test-side copy).
- `117bdb89` — `test(cpp): T52 port monitors, energy, acceptance
  (3.23)`: all eight ports, two shared headers, registration, both
  platforms' sensor-noise fixtures, ledger rows, oracle-harness repairs.
- `<report commit>` — this report.

## Triage

| File | records | port | golden | planned |
|---|---|---|---|---|
| `test_energy_ledger.py` | 14 | 14 | 0 | 0 |
| `test_balance_monitor.py` | 9 | 9 | 0 | 0 |
| `test_monitor_strictness.py` | 10 | 10 | 0 | 0 |
| `test_reference_monitor.py` | 4 | 4 | 0 | 0 |
| `test_sensor_noise.py` | 8 | 6 | 2 | 0 |
| `test_acceptance_interval_capture.py` | 2 | 2 | 0 | 0 |
| `test_realistic_pedelec_acceptance.py` | 4 | 0 | 0 | 4 |
| `test_lean_limit.py` | 2 | 2 | 0 | 0 |
| **total** | **53** | **47** | **2** | **4** |

No `drop` records. The four `planned` records are all of
`test_realistic_pedelec_acceptance.py`, blocked on the documented
Task-3.09 physics stop-rule decision — the bars are preserved verbatim
under `#if 0` with always-on measurands (the
`test_closed_form_rider.cpp` precedent), never weakened.

## What the port reproduces

- **`test_reference_monitor.cpp`** (4): strict/non-strict
  `ReferenceMonitor` boundaries — `first_failure` latch, warning
  propagation via `take_warning`, and `ReferenceFailure::message()`
  f-string parity including the singleton trailing comma.
- **`test_balance_monitor.cpp`** (9): grace/dwell/recovery sequences,
  the summary and metrics legs, event `state()/restore()` round-trips
  on the welded environment at the `.005 s` sensor grid.
- **`test_monitor_strictness.cpp`** (10): accounting flush boundaries,
  model-status propagation, runtime failure/reset semantics over
  neutral raws and the delayed research fixture; the bootstrap document
  keeps `measurement_decimation` and persisted monitor strictness
  synchronized with the physical runtime.
- **`test_sensor_noise.cpp`** (8 records / 10 golden nodes): RNG draw
  order for the legacy call sequence, the nine-value tape contract
  across three seeds (0, 19, 2**63-117) and three configs, the
  `{60,12}` pipeline trace and sample counters against the frozen
  pre-refactor reference.
- **`test_energy_ledger.cpp`** (14): `step_work` five-argument
  surface (muscle, motor, constraint, closure, dt), the F1
  equality/closure rescope, `PeriodAccounting` flush detail channels
  (`full||last`), `endpoint_mass`, `energy.constraint_work` vs the
  published `constraint_components`/`numerical_constraint_powers`
  helpers, and `constraint_work_ok` round-off tolerance.
- **`test_acceptance_interval_capture.cpp`** (2): per-interval
  violation extraction over `attachment_violations` channels.
- **`test_lean_limit.cpp`** (2): `rider::lean_limit_rad` geometric
  contract.
- **`test_realistic_pedelec_acceptance.cpp`** (4, slow): the four
  welded episodes (flat 0–300 m, shift, 15% climb, savage) with
  always-on measurands; live structural checks only where the oracle
  itself is not baseline-red.

## Physics stop-rule records (Task 3.09, undecided)

Measured on darwin-arm64 / linux-aarch64 (`reason=` is the episode
outcome):

| scenario | measure | bar | native | oracle baseline |
|---|---|---|---|---|
| flat | reach 25 km/h | <10 s | 8.28 s | passes |
| flat | cadence in-band fraction | >=0.8 | 0.6063 / 0.6132 | 0.760–0.776, baseline-red |
| flat | late violations | 0 | 538 / 557 `rider_strength.*` | 0 — new native parity gap |
| shift | windows checked | >=1 | 6 / 7 | — |
| shift | worst motor/human ratio | >=2.5 | 0.2303 / 0.3295 | ~0.094, baseline-red |
| climb | steady min speed | >=12 km/h | 12.038 / 12.931 | passes natively |
| climb | late violations | 0 | 763 / 869 | oracle chronic, same family |
| savage | episode | finish/x>=120 m | darwin: aborts `attachment normal has no planar component` at ~t0; linux: x=58.3 m stall, `crash:loop_out` | x~53 m stall, `duration`, balance_lost |

Two **new native-vs-oracle parity gaps** found by this batch (recorded
in ledger evidence, not silently absorbed):

1. The native flat/shift rides emit `rider_strength.*` (knee_rear,
   torso_hinge, hip_rear) violations from interval ~406 where the
   oracle's flat ride emits none — the same per-tick-held-command
   family the oracle shows only on the climb.
2. The darwin-arm64 savage episode throws `attachment normal has no
   planar component` inside `decompose_wrench` (a purely vertical
   contact normal); the oracle and linux-aarch64 complete the episode.
   The native test pins that exact failure mode so a different abort is
   red.

All four records stay `planned` pending the user decision required by
the stop rule (approved model fix, threshold change, or documented
`drop`).

## Oracle-harness repairs (in-batch, test-side only)

- `test_energy_ledger.py`, `test_monitor_strictness.py`: refreshed
  stale call sites to the post-F1 surfaces — five-argument
  `step_work(muscle,motor,constraint,closure,dt)` and the
  `closure_signed_j`/`closure_absolute_j` fields on the
  `runtime_for_raws` fake. Pre-existing red at batch start: 16 of 84
  collected; after repair the batch subset runs 54 green (the remaining
  fails were already-documented pedelec baselines).
- `test_sensor_noise.py`: `_golden.record` capture sites added
  (noise/uniform draws, the `{60,12}` reads trace, counters).

## Verification

- **Focused CTest, darwin-arm64** (`build/native-release`): all 8
  binaries pass — `reference_monitor` 19, `balance_monitor` 120,
  `monitor_strictness` 174, `sensor_noise` 12905, `energy_ledger` 340,
  `acceptance_interval_capture` 2, `lean_limit` 3,
  `realistic_pedelec_acceptance` 7859 checks (9.66 s).
- **Focused CTest, linux-aarch64** (`/build/root-clang`,
  `bike-dev-pfc-lane-c`, clang++-18): the 7 fast targets pass;
  `realistic_pedelec_acceptance` passes 13850 checks.
- **Golden capture, both platforms**: 10 node pairs (.json+.bin) under
  `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/
  tests_reference_test_sensor_noise.py/`. The linux fixtures were
  re-captured inside the container
  (`BIKE_GOLDEN_CAPTURE=tests/fixtures/golden
  NATIVE_TEST_BUILD_PATH=/build/root-clang uv run --frozen --no-sync
  pytest -q tests/reference/test_sensor_noise.py` → 30 passed) and
  pulled via `container.sh pull`. Every `.bin` payload is
  byte-identical across platforms — the contracted RNG lowering is
  platform-deterministic.
- **Byte-flip proof**: byte 0 of the darwin-arm64
  `test_pipeline_matches_frozen_pre_refactor_sensor_trace.bin` flipped
  → FAIL naming node, comparison index and `reads` label; restored →
  PASS 12905.
- **Oracle pytest**: the batch's non-slow files pass on darwin-arm64
  (54 nodes); the pedelec file's four nodes are the documented
  baseline-red records.

## Final gate — `bash tools/run_tests.sh native` (darwin-arm64)

Run on the final tree (`117bdb89`), host shared with lane-e's `full`
gate:

- **Preflight** on `native/build` (AppleClang, Release): configure +
  build + produced `bike-native-checks` orchestrator rebuilt; all nine
  static-analysis sweeps green — frontends/tidy/cppcheck/odr zero
  findings, analyzer alpha findings report-only, shellcheck clean;
  produced kinds `headers` (263 headers / 9,198 TUs, 0 errors),
  `diagnostic-controls`, `context` all passed.
- **CTest** `native/build`: **60/62 in 1,136 s** — the two reds are the
  documented frozen-oracle baselines `migration_T09` (`FAIL post-advance
  integration vector matches the oracle`) and `migration_T10` (`FAIL
  first transition matches the Python oracle`); `set -e` exits 8 at the
  ctest step — the same gate shape recorded on every lane since F2.
- **Manual tail** (profile flags, `MallocScribble`/`MallocPreScribble`/
  `MallocGuardEdges`, `NATIVE_TEST_BUILD_PATH=native/build`):
  `bike-native-checks --kind contracts` → **12/12 controls passed**;
  `pytest tests/reference/test_native_*.py test_golden_episode*.py -p
  native_test_reporter` → **43 failed / 3,197 passed / 1 skipped /
  60 warnings in 6,160 s**. All 43 reds reproduce the documented
  task-F2 baseline tail (concentration: `research_environment` 17,
  `runtime_*` 14, `spindle_controller` 2 = `_damped_config` defect,
  `distributed_tire` 2, `numeric_completion` 2, `viewer_snapshot` 2,
  `research_capture`/`research_recording`/`ride_frontend`/
  `pneumatic_tire` singletons) — C++↔oracle bitwise-drift signatures
  (±ulp) on unported/planned records; the set shrank 64→43 as sibling
  batches landed. **Zero failures in this batch's surface**: the 3.23
  ports are ctest-registered binaries (all green above) and the three
  repaired oracle files are outside the `test_native_*` glob. The tail
  embeds the designed ~80-min re-run of the 9,198-TU headers sweep
  inside
  `test_native_check_tools.py::test_standalone_header_sweep_compiles_first_party_headers`
  — the documented dominant tail cost, not a stall.

## Ledger

- `migration/test-port-map.json`: all 53 rows dispositioned — 47
  `port` (`native contract port`/`analytic`), 2 `golden`
  (`bitwise-oracle`, `golden_platforms: [darwin-arm64,
  linux-aarch64]`), 4 `planned` with dated two-platform evidence;
  `native_test_id`/`native_source`/`collected_node_ids`/`line` refreshed.
- `migration/python-removal.json`: 7 rows → `replacement_landed`
  (replacement = the new `test_*.cpp`, verification = the CTest name;
  `snapshot_sha256`/`snapshot_line_count` refreshed to the live
  sources). `test_realistic_pedelec_acceptance.py` stays `planned`
  with its replacement path recorded — the file cannot be deleted while
  its records await the 3.09 decision.
- `native/tests/reference/Phase3Ports.cmake`: eight
  `bike_add_reference_test` registrations, `LABELS t52 physics`
  (+`golden`, +`slow` as above); `sensor_noise`/`balance_monitor` link
  `bike_random`/`bike_cli`; pedelec links `bike_validation`.
- `migration/cpp-linux-completion-evidence.md`: see C323 section.

## Concerns / notes for merge-back

1. The pedelec `#if 0` region is deliberately verbose — every
   suppressed bar keeps its oracle assertion text and a measurand
   `fprintf` so the 3.09 decision can be made from test output alone.
   Do not "clean up" the disabled blocks before the decision lands.
2. `rider_strength.*` violations on the flat ride and the darwin
   planar-normal abort are new parity evidence for the 3.09 decision;
   they were not present in the 3.09-era measurements (which only
   documented the oracle's baseline-red bars).
3. `constraint_components`/`numerical_constraint_powers` were already
   production helpers — `PhysicalStep::solve_tail` calls both on every
   step (step.cpp:1674 fills `RawStep::constraint_powers`; step.cpp:1721
   builds the constraint ForceMap). `6fe7ab22` changed only linkage:
   anonymous namespace → exported via `step.hpp` beside
   `evaluate_attachments`, so the energy-ledger port audits the identical
   grouping production computes instead of a test-side copy. Read-only
   on `mjModel`/`mjData`; they mutate only caller scratch and return
   fresh vectors.
4. `tools/run_tests.sh` ctest selection note: the batch's slow pedelec
   node is excluded from default `-L` sweeps by its `slow` label per
   the established convention.
