# C320 / T52 batch 3.20 — brakes, cruise, suspension, clocks (`runtime`)

Worktree: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/pfc-lane-d`
Branch: `lane/d` · Integration base: `45d35190` · Date: 2026-10-26

## Scope

`migration/test-port-map.json` batch 3.20: 66 records across eight files.
28 already had source successors (static brake 5, native clock contracts 13,
control clock 3, playback clock 7). This task closed the remaining 38:

| File | Records | Disposition |
|---|---|---|
| `tests/reference/test_native_cruise.py` | 24 | 8 golden replay + 13 contract ports + 3 binding-contract drops |
| `tests/reference/test_native_brake_resistance.py` | 5 | 2 golden + 2 contract + 1 drop |
| `tests/reference/test_native_suspension.py` | 4 | 2 golden + 2 contract |
| `tests/reference/test_native_stepper.py` | 1 | 1 golden |
| `tests/reference/test_native_clock_contracts.py` | 3 | 3 contract (tire snapshot interval path) |
| `tests/reference/test_control_clock.py` | 1 | 1 golden (solved-effort runtime replay) |

## Native successors

- `native/tests/reference/test_native_cruise.cpp` — sequential 515-case PI
  oracle replay (15 literal head cases + 500 `bike::random::NumpyRng(728)`
  tail, PCG64 mirror of `np.random.default_rng(728)`), signed-zero,
  snapshot/mjData restore, exact-saturation and restored-target parametrized
  leaves, plus typed-decoder contract ports (config validation, missing keys,
  root-joint kinds, nonfinite setter/restore atomicity, projection).
- `native/tests/reference/test_native_brake_resistance.cpp` — replays the
  frozen `episode/pinned_savage_40_f2` bundle: per state row the Stepper is
  restored/forwarded, demand sweep {0,.3,.7,1,-.2,1.5} through
  `brake_torques`/`apply_brake`, resistance components over the manifest's
  tire-snapshot rows; require-config and missing-key contract ports.
- `native/tests/reference/test_native_suspension.cpp` — suspension component
  replay over the episode and `native_suspension/legacy_env` bundles plus
  require-config/missing-key ports.
- `native/tests/reference/test_native_stepper.cpp` — 200 `Stepper::step()`
  vs raw `mj_step()` bitwise over qpos/qvel/act/warmstart.
- `native/tests/reference/test_native_clock_contracts.cpp` — adds the three
  `TestTireSnapshotIntervalPath` ports (id/helper agreement over a literal
  table, overflow_error + bitwise state atomicity + re-entry, foreign
  rounding modes) beside the earlier interval/cruise-timed ports.
- `native/tests/reference/test_control_clock.cpp` — adds the solved-effort
  replay over the `control_clock/env_request` bundle: 403 controlled steps,
  held zero command between ticks, rider effort/power telemetry scan, reset,
  ten-step tail.
- `native/tests/reference/test_native_static_brake.cpp` — IEEE-754 literal
  fixes (`-1000.*(.03-.02)` style) in the pre-existing successor.

All registered in `native/tests/reference/Phase3Ports.cmake` (`t52 runtime
golden [slow]`); `reference.control_clock` gained the `bike_golden` link for
the env-request bundle seam.

## Recorded binding-contract drops (no native analog)

- `test_missing_state_field_preserves_state`,
  `test_extra_state_field_rejected`,
  `test_nonnumeric_restore_preserves_state` (cruise) — nanobind dict-shape /
  dict-value-type rejections; typed `CruiseState` cannot carry them. The
  numeric halves are covered by `invalid_restore_preserves_state` and
  `nonfinite_restore_preserves_state`; the two `('engaged', ...)` params of
  `test_invalid_restore_preserves_state` are likewise binding-only.
- `test_resistance_snapshot_array_conversion_lifetime` — ndarray
  lifetime/dtype duck-typing at the binding boundary; native
  `TireSideInput` spans are statically typed views.

## Fixtures / goldens

- New recipe `tests/fixtures/golden/episode/pinned_savage_40_f2/`
  (`physics.toml`, `request.toml`, `freeze.py` one-off capture helper):
  40 direct physical steps, `human_torque_nm=35`, `speed_kph=0.0`
  (`initial_speed_mps=0` — native request parsing unconditionally overrides
  from `speed_kph`), dt 0.00125, `capture_mode="legacy"`.
- Per-platform artifacts under `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/`:
  `episode/pinned_savage_40_f2/` (episode.npz, manifest.json, model.mjb,
  wire.json), `control_clock/env_request/` (request/physics/track/initial
  state), `native_suspension/legacy_env/` (meta, model.mjb, wire), and the
  `tests_reference_test_native_{cruise,brake_resistance,suspension,stepper,
  clock_contracts,static_brake}.py/` golden leaf families.

## Results

Python oracle (capture-mode runs double as the parity gate):

- darwin-arm64: batch subset 164 passed; brake/suspension subset 6 passed
  (3 known `energy.constraint_work` runtime warnings); control clock 7
  passed (1 known warning).
- linux-aarch64 (container `bike-dev-pfc-lane-d`, `/opt/bike-venv`):
  180 passed + 14 passed, same warnings only.

Native CTest:

- darwin-arm64 (`build/native-release`, clang): `reference.native_stepper`
  PASS 81 checks; `reference.native_cruise` 13989;
  `reference.native_clock_contracts` 12662;
  `reference.native_brake_resistance` 45722; `reference.native_suspension`
  11660; `reference.control_clock` 1598; `reference.native_static_brake`
  111.
- linux-aarch64 (`/build/root-clang`, clang++-18, `-ffp-contract=off`,
  separate NumPy lowerings per the platform record): all 7 T52 reference
  tests pass (`ctest -R 'reference\.(native_cruise|native_brake_resistance|
  native_suspension|native_stepper|native_clock_contracts|native_static_
  brake|control_clock)'` → 7/7, plus `reference.playback_clock` green).

## Byte-flip proof

`test_native_stepper.py` golden `.bin`: flipped one byte at offset 208 →
`test_native_step_matches_python_bitwise` failed with a qvel bit mismatch;
restored → PASS 81. Golden payload is verified bitwise, not advisory.

## Baseline / unrelated failures

The pre-existing Darwin full-CTest reds (`reference.golden_episode_tool`,
`reference.pneumatic_tire_oracle`, `reference.native_drive_policies`,
`reference.native_drivetrain`) are missing/stale-fixture failures owned by
other lanes/batches, unrelated to this batch; `reference.native_static_brake`
was in scope and is now green on both platforms.

## Ledger updates

- `migration/test-port-map.json`: 38 records closed (14 golden, 20 port,
  4 drop) with measured collected node IDs and dated 3.20 lane/d evidence.
- `migration/python-removal.json`: the eight batch file rows moved to
  `replacement_landed` with their `reference.*` verification ids.
- `migration/cpp-linux-phase3-source-status.json` + `-source-review.md`:
  3.20 marked `source_implemented` / complete.

## Out-of-batch drive-by fix

`native/src/terrain/alpine.cpp`: `faceted` (template, internal linkage by
use) was wrapped in an anonymous namespace to clear a blocking
`-Wmissing-declarations`-family clang-tidy finding
(`function 'faceted' can be made static or moved into an anonymous
namespace`) that stopped the Darwin `check_tidy` sweep. The edit is
behavior-preserving (anonymous namespace is exactly what the checker
asked for); the file is otherwise untouched by this batch.

## Final gate

`bash tools/run_tests.sh native` (darwin-arm64, 2026-10-26,
`native/build`, host shared with three sibling lane gates):

- Build green; produced orchestrator rebuilt.
- All nine static-analysis sweeps + the three produced-binary kinds
  green: `frontends`, `tidy`, `cppcheck`, `odr`, `scripts`,
  `headers` (259 headers / ~8,950 probes), `diagnostic-controls`,
  `context` — findings `none`; `analyzer` passed report-only (alpha).
- CTest `native/build`: **59/62** at gate time — `migration_T09`
  (`integration vector resolves bitwise`) and `migration_T29`
  (`config.rider_controller.weld_servo_gain: unknown key`), the
  documented baseline pair verbatim, plus `migration_T03`, which failed
  environmentally (`No space left on device` during its nested
  scratch-build configure — the host disk hit 100% under concurrent
  lane gates). `ctest -R '^migration_T03$'` rerun standalone:
  **Passed 158.35 s** — effective 60/62 = the documented baseline
  exactly. Gate exited 8 at the ctest step, the documented shape for
  this profile (the baseline pair ends the script before the
  contracts+pytest tail).
- Contracts check (run per the profile's tail stages):
  `bike-native-checks --kind contracts` — **12/12 passed**
  (provider_binding_boundary … hot_kernel_dependency_free).
- Pytest tail (`tests/reference/test_native_*.py +
  test_golden_episode*.py`, `-p native_test_reporter`, Darwin allocator
  debugging per the profile): **3161 passed / 78 failed / 1 skipped /
  0 errors** in 6851 s. Of the 78: **75 are the documented
  C++↔oracle-gap baseline verbatim** — same file concentration as the
  F2 (75 failed) and 3.28 lane/a (81 failed) records: spindle 22,
  research_environment 17, drivetrain 7, runtime_* 15, drive_policies 4,
  numeric_completion 2, distributed_tire 2, pneumatic_tire 1,
  research_capture/recording 2, ride_frontend 1,
  replay_recording_v3 1, viewer_snapshot 1. The other 3
  (`platform_recording::test_viewer_reset_twice_…`,
  `replay_session::test_native_session_gate_invoked_…`,
  `replay_session::test_native_session_rejects_foreign_identity_…`)
  were disk-full flakes (`run.duckdb` checkpoint `ENOSPC`) — all three
  **pass** on a targeted rerun (38.96 s). **Zero failures in the eight
  3.20 batch files**; an earlier disk-full-poisoned attempt had shown
  88F/346E, all traced to `Errno 28`/`tmp_path` exhaustion and
  superseded by this clean run.

`bash tools/linux/container.sh` native profile (linux-aarch64,
`bike-dev-pfc-lane-d`, `/build/native-clang`):

- Stops at `check_tidy`: 317 unsuppressed findings across 102
  translation units (setup_owner, test_t08, initial_state, lifecycle,
  validate, golden/capture, research/*, viewer/research …). Verified
  from `native_check_summaries/tidy.json`: **zero unsuppressed
  diagnostics reference any file this batch touched** — pre-existing
  Linux baseline red, matching the documented pattern where lane gates
  reach `check_tidy` and stop on pre-existing warnings.
- Batch-required evidence all green on linux-aarch64: oracle capture
  runs 180 + 14 passed; CTest **8/8** T52 reference tests re-verified
  (`playback_clock`, `control_clock`, `native_static_brake`,
  `native_cruise`, `native_brake_resistance`, `native_suspension`,
  `native_stepper`, `native_clock_contracts`, 2.41 s).

Per the documented Phase-3 batch procedure the required gates — per-file
pytest greens, two-platform goldens, and the new CTest nodes on both
platforms — are all green; the branch-level `check_tidy` (Linux) and
`migration_T09/T29` (Darwin) reds are baseline and not authored here.
