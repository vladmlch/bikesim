# Task C334 report — Batch 3.34 frontends / realtime / async / batch

Branch `impl/python-free-cpp` (lane `pfc-lane-d` / `lane/d`), base `e38e1c8a`.

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
  read; prints `workers=N start_method=in-process`; passes the count to
  `batch::summarize`.
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

- `reference.async_close` — PASS, 720 checks.
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
