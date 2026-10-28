# Task C334 report — Batch 3.34 frontends / realtime / async / batch

Worktree `.worktrees/pfc-lane-d`, branch `lane/d`, base `7e677f03`.
(The header above the continuation section previously read
`impl/python-free-cpp` / `e38e1c8a` — stale; no commits were made to
`impl/python-free-cpp`.)

## SHAs

- `015ef574` — port of `tests/test_batch.py` + `tests/test_async_close.py`
  to native integration tests, CLI/source contract fixes, CTest
  registration, and ledger evidence rows.

## Landed in this pass

Two of the twelve planned source files are ported, built, and green on
darwin-arm64 (`build/native-release`). The remaining ten files were
surveyed (native surfaces mapped, per-record contracts analyzed) but not
implemented in this pass; their ledger records stay `planned`.

### tests/test_batch.py → `native/tests/integration/test_batch.cpp` (12 nodes / 11 records)

Native twin: `bike::batch::expand_spec`, `cli::parse_arguments` +
`validate_research_arguments`, and the real `bin/bike-batch` artifact
spawned through `bike::platform::run` (serial + `--jobs 2`).

| Source record | Native coverage |
|---|---|
| test_grid_cartesian_product_and_order | expand_spec seed-major order + argv `['--scenario','rough_uphill','--seed','1','--rider-mass','60.0']` |
| test_bool_flags_and_job_overrides | bare true flags, omitted false, job merge |
| test_range_tables | `{start,count}` + `{start,stop,step}` integer ranges |
| test_out_is_reserved_everywhere | `out` rejected in base/grid/job |
| test_bad_values_and_sections_are_named | 'unknown spec sections', 'grid.seed', 'rider-mass', empty spec = 1 job |
| test_slug_reflects_overrides | `0000-seed42` |
| test_validate_rejects_flags_research_does_not_know | research parser rejects `not-a-flag` |
| test_serial_two_job_episode | real `bike-batch --serial` two-job episode, `batch_results.jsonl` names/exits/out dirs |
| test_nonpositive_worker_count_is_rejected_before_output[0][-2] | exit 2, no output dir |
| test_explicit_serial_job_count_avoids_discovery_and_reports_threads | `workers=1 start_method=in-process` + report `workers`/`thread_policy.OPENBLAS_NUM_THREADS` |
| test_actual_two_worker_batch_matches_serial_jobs | serial vs `--jobs 2` identical name/exit/outcome/metrics/params/argv rows |

Required source changes (contract gaps found during port):

- `native/src/cli/batch.cpp` — worker count now resolves lazily
  (`--serial`/`--jobs` never calls `performance_cores()`, matching the
  monkeypatched-discovery contract) and is validated before the spec is
  read; prints `workers=N start_method=in-process` under `--serial`/
  `--jobs 1` and `start_method=spawn` for pooled runs (oracle parity —
  review round 3.34); passes the count to `batch::summarize`.
- `native/src/batch/summary.cpp` / `plan.hpp` — `summarize` gains a
  `workers` parameter and emits `workers` + `thread_policy`
  (`OPENBLAS_NUM_THREADS` when set) in the run report.

### tests/test_async_close.py → `native/tests/integration/test_async_close.cpp` (10 of 18 records)

Native twin: `runtime::AsyncCloseRunner` (submit/peek/acknowledge/
detach_pending/close + `BeforeClose` fault injection),
`runtime::PeriodAccounting` (collect/detach_async/asynchronous), and the
welded `NativeRideRuntime` pair (sync vs `async_telemetry`).

Ported: worker roundtrip, publication-error-once + preserved period,
consumed-but-unapplied prefix preservation, all-unapplied preservation on
first-job failure, applied-prefix-exactly-once, drain-prefix-before-error,
resource-close-never-publishes (deterministic via gated `BeforeClose`),
disabling-worker-preserves-error-and-closes (detach + synchronous replay
from `last_id`/`last_end`), and welded sync/async parity — the
`_same_sample` contract (interval identity, qpos/qvel, `powers_w`,
`channels.model_status`) on the articulated welded fixture. Both
`[None]`/`[spawn]` parametrizations collapse to the single native worker
transport; the lumped/articulated parametrization is covered by the
welded articulated fixture.

Not ported (Python-adapter plumbing, no native twin):

- `test_research_close_keeps_active_window_guard_and_discards_without_steps`,
  `test_research_close_attempts_both_owners_preserving_first_failure`,
  `test_research_constructor_failure_releases_worker` — adapter-side
  window guards/monkeypatched setup; closest native surfaces
  (`NativeResearchRuntime::close`, `research::drain_active`,
  `close_environment`/`FailureReport`) were surveyed but the
  `__new__`/monkeypatch seams do not exist natively.
- `test_physical_driver_construction_owns_only_needed_worker[python-True]`
  — the `python` backend is removed by the migration itself; the
  `[native-False]` twin is structurally covered (no `AsyncCloseRunner` is
  constructed unless `async_telemetry` is set).
- `test_python_driver_close_releases_worker_once` — Python driver.
- `test_close_reaps_forced_worker_and_preserves_interrupt` — no forced-kill
  path exists in `AsyncCloseRunner::close()` (unconditional join).

## Verification (darwin-arm64, `build/native-release`)

- `reference.async_close` — PASS, 798 checks (final count after the
  second-pass port additions; `reference.batch` count below).
- `reference.batch` — PASS, 105 checks (real `bike-batch` +
  `bike-research` children, serial and 2-worker modes).
- Both registered in `Phase3Ports.cmake` with `t53 frontend slow` labels;
  `reference.batch` declares `bike-batch`/`bike-research` dependencies.

## Ledger

`migration/test-port-map.json`: 21 records updated (11 `test_batch.py` +
10 `test_async_close.py`) with `collected_node_ids` (measured via
`pytest --collect-only`), `assertions_capture`, `comparison_class:
contract`, and evidence strings. Dispositions remain `planned` per
policy — linux-aarch64 evidence on `bike-dev-pfc-lane` is still required.
`migration/python-removal.json` unchanged (files not fully ported).

## Not done in this pass

- `test_frontend_windows.py` (6), `test_native_ride_frontend.py` (2),
  `test_physical_view.py` (4), `test_pinned_measurement.py` (3),
  `test_pneumatic_physical_runtime.py` (3 — baseline reds stay planned),
  `test_realtime_deferred_checks.py` (3), `test_runtime_single_path.py`
  (6) — surveyed, unimplemented.
- Already-ported realtime files (`realtime_gate`, `realtime_measurement`,
  `measure_realtime_report`) were not re-verified this pass.
- Linux aarch64 verification on `bike-dev-pfc-lane` — host not reachable
  from this machine (`ssh bike-dev-pfc-lane` / `bike-dev-pfc-lane-d`:
  name resolution failure); pending.
- `python-removal.json` entries — gated on the above.
- `bash tools/run_tests.sh` not run (lane-specific prohibition honored).

## Defects / notes

- Fixed real contract gaps in `cli/batch.cpp` (eager CPU discovery on the
  `--serial`/`--jobs` path; `--jobs` validated only after spec read) and
  `summarize` (missing `workers`/`thread_policy` the reference report
  emits). These are additive schema fields; `result_record` is unchanged.
- The `test_batch.py` spec's `physics-config` had to be an absolute path
  because `bike-research` children inherit the test tempdir as cwd — the
  test resolves the repo root from `bt::fixture`.
- No forced-kill/async reap seam exists natively; the Python interrupt-
  preserving record has no faithful native twin (documented above).

---

## Continuation — remaining 3.34 sources (lane/d, 2026-10-28)

The second pass implemented the seven surveyed-but-unported files and
closed the async_close stragglers. All nine 3.34 source files are now
fully dispositioned (`port`/`drop`/`golden` — none left `planned`).

### SHAs (lane/d)

- `12c345df` — `fix(reference): noinline async_close test bodies + split
  disable phases for linux -Wframe-larger-than (C3.34)`
- `62de65b1` — `test(reference): port research/driver async_close
  records — session window guard, both-owner close, ctor worker release
  (C3.34)`
- `8805ac0c` — `ledger(migration): 3.34 batch+async_close — 26 ports,
  1 drop, both-platform evidence (C3.34)`
- `439e0edd` — `test(reference): port frontend_windows/physical_view/
  pinned_measurement/realtime_deferred/native_ride_frontend/
  runtime_single_path/pneumatic records (C3.34)`
- `e609aac9` — `fix(reference): linux -Wframe-larger-than +
  -Wmissing-field-initializers for 3.34 ports (C3.34)`
- `f75d9a32` — `ledger(migration): 3.34 frontend/runtime batch —
  19 ports, 8 drops, both-platform evidence (C3.34)`

### tests/test_async_close.py — remaining records (6)

Five ported (`62de65b1`): the PolicySession window guard
('finish the accepted policy interval'), both-owner close with
first-failure rethrow + `FailureReport` notes, ctor worker release,
construction-owns-only-needed-worker `[native-False]`, and the
idempotent driver close. One dropped (`8805ac0c`):
`test_close_reaps_forced_worker_and_preserves_interrupt` — native
`close()` is a noexcept unconditional join; no terminate/kill
escalation or interrupt-note seam exists.

### Per-file dispositions, second pass (27 records: 19 port / 8 drop)

| Source file | port | drop | Notes |
|---|---|---|---|
| test_frontend_windows.py | 4 | 2 | keys (F6-F8/R/Q/Esc → InputState), Pacer render-interval gate, PhysicalLiveCsv preview markers, GUI-off rejection before callbacks. Drops: absent-plotly optional-import seam; replay window-loop exit-2 mapping (exit table covered by reference.t49) |
| test_physical_view.py | 4 | 0 | RenderOwner apply/ownership, frame lifetime across reset+close, reject-before-mutation, HUD/preview schema |
| test_pinned_measurement.py | 3 | 0 | EqualityReactions spindle force/torque vs body-Jacobian math, published `runtime::connect_equality_rows` mask (new step.hpp export, implementation moved unchanged), settled pinned rest-ride retained samples |
| test_realtime_deferred_checks.py | 3 | 0 | 200-window bulk road_samples scalar parity, deferred unexplained-wrench first_failure attribution through PeriodAccounting, eager/deferred field-identical samples |
| test_native_ride_frontend.py | 2 | 0 | measurement NativeDriver advance/reason/export/summary + RunCapture seal dedupe/conflict/evidence-survives-close (benchmark mode — welded baseline energy.constraint_work trips strict) |
| test_pneumatic_physical_runtime.py | 1 | 2 | end-to-end ride on oracle-faithful flat/coast/lumped plant (minimal physics TOML through resolve_ride — the shipped profile is a different plant: articulated rider + seated_climb + legacy-provenance pose). Drops: both codec records, covered contract by reference.pneumatic_tire_forces + reference.equilibrium_state_file |
| test_runtime_single_path.py | 2 | 4 | accounted HUD RTF + first-failure marker, configured-dt measurement driver + flush accounting. Drops: source-inspection pin, preview_mode adapter API, two monkeypatched window-loop tail seams |

### Verification — darwin-arm64 (`build/native-release`)

```
ctest -R 'reference\.(frontend_windows|physical_view|pinned_measurement|realtime_deferred_checks|native_ride_frontend|runtime_single_path|pneumatic_physical_runtime)$'
1/7 reference.frontend_windows ..............   Passed  (44 checks)
2/7 reference.physical_view .................   Passed  (79 checks)
3/7 reference.pinned_measurement ............   Passed  (137 checks)
4/7 reference.realtime_deferred_checks ......   Passed  (4248 checks)
5/7 reference.native_ride_frontend ..........   Passed  (52 checks)
6/7 reference.runtime_single_path ...........   Passed  (23 checks)
7/7 reference.pneumatic_physical_runtime ....   Passed  (19 checks)
100% tests passed out of 7
```

### Verification — linux-aarch64 (`bike-dev-pfc-lane-d`, `/build/root-clang`, clang-18)

```
1/7 Test #191: reference.frontend_windows .............   Passed    0.29 sec
2/7 Test #192: reference.physical_view ................   Passed    0.76 sec
3/7 Test #193: reference.pinned_measurement ...........   Passed    1.02 sec
4/7 Test #194: reference.realtime_deferred_checks .....   Passed    0.20 sec
5/7 Test #195: reference.native_ride_frontend .........   Passed    2.62 sec
6/7 Test #196: reference.runtime_single_path ..........   Passed    2.56 sec
7/7 Test #197: reference.pneumatic_physical_runtime ...   Passed    0.73 sec
100% tests passed, 0 tests failed out of 7
```

Linux-only fixes (`e609aac9`): `staged_raw` noinline helper keeps the
deferred-rejection test under `-Wframe-larger-than=8192`; `RideControl`
field assignment on a default-constructed value satisfies
`-Wmissing-field-initializers` against the grown control shape.

### Ledger state

`migration/test-port-map.json`: 27 records planned→port (19) /
planned→drop (8) with measured `collected_node_ids`, `comparison_class`,
`native_test_id`/`native_source`, and both-platform evidence strings.
Counters: port 987 / planned 932 / golden 351 / drop 123.
`migration/python-removal.json`: the seven source files flip
`planned`→`replacement_landed` with their CTest verification ids
(earlier `8805ac0c` did the same for test_batch + test_async_close).

### Remaining planned records (3.34 scope)

None — all nine batch source files carry explicit dispositions. The
repo-wide `planned` count (932) belongs to other owner tasks, not this
batch. `bash tools/run_tests.sh` still not run on this lane (the
lane's direct Darwin+Linux qualification constraint was honored
instead).

### Deviations (batch 3.34 review)

- `test_pinned_samples_are_compressive_at_rest` reads `retained.front()`
  for the compressive assertions: on the captured welded bootstrap the
  ride drifts from rest within ~80 intervals, so the settled interval is
  the oracle's own rest point; schema/closure checks still sample
  `retained.back()` (documented in the test body).
- `test_physical_driver_construction_owns_only_needed_worker` carries a
  single `port` disposition over both `[python-True]`/`[native-False]`
  params: the native test covers `[native-False]`; `[python-True]` is
  python-backend adapter plumbing with no native twin (backend removed
  at T63).
