# Remove joint damping everywhere

Date: 2026-11-04. Branch/worktree: `no-joint-damping` at `.worktrees/no-joint-damping`.

## Decision (user, 2026-11-04)

- Joint damping must not exist in the project in any form: rider joints
  and bike joints.
- Old recordings and states that carry damping fields need not stay
  readable.
- Stored fixture models are rebuilt without damping and every dependent
  golden is re-recorded by the current native program, on both platforms
  (Linux through `tools/linux/container.sh`). The user accepted that the
  re-recorded goldens then prove "native today equals native at capture",
  no longer "native equals the removed Python oracle".

## Measured starting point (darwin-arm64, commit 48c30c8c)

- Baseline: 185/185 ordinary tests pass (60 s).
- Experiment: bike damping values set to "0" in `model/drivetrain.cpp`
  and `model/linkage.cpp`. Five tests change: `legacy_ride`
  (`bootstrap/legacy_flat/integration.f64`), `model_mjcf`
  (`tests/fixtures/model/welded.xml` dof_damping), and slow
  `reference.native_pneumatic_tire`, `reference.period_buffer`,
  `reference.golden_episode_tool`. All other 40 slow physics tests stay
  green because they run on stored models.
- 33 stored models carry damping (probe: compile/load, list nonzero
  `dof_damping`): all physical-mode bundles carry `main_pivot=0.5
  horst_pivot=0.3 rocker_frame_pivot=0.3 yoke_pivot=0.2`;
  `native_suspension/legacy_env` also `steer_joint=1000` and wheel spins
  0.01. Plus `tests/fixtures/model/welded.xml`.
- Removed-field names still recorded: `joint_passive_damping_nms_rad`
  (27 files), `passive_damping_nm` (124), `solved_passive_nm` (39),
  `rider_passive_damping` (38), `rider_passive_power_w` (123); includes 34
  golden `.bin`, 7 DuckDB recordings, bootstrap/state JSON.
- Golden `.values` arrays store numeric leaves without names, which is why
  `ft2_reference_support.hpp::consume_tree` guesses removed positions.
  Re-recording removes the need for the guess.

## Scope

In scope (joint damping):

1. Bike joint `damping` attributes in `model/drivetrain.cpp`,
   `model/linkage.cpp`; the now-redundant `set("damping","0")` overrides
   and `{"damping","0"}` attributes in `model/physical_topology.cpp`,
   `model/articulated_rider.cpp`, `model/linkage.cpp` (slide joints).
   The locked steer keeps its range and stiffness; only damping goes.
2. `rider_passive_damping` force component and its energy/power terms
   (`runtime/step.cpp`, `validation/evidence.cpp`).
3. Legacy readers: `replay/rebuild.cpp::drop_removed_passive_damping`,
   `rider/state_codec.cpp` legacy keys, `recording/database_validate.cpp`
   `kLegacyFields`/`kLegacyStructChildren`,
   `initialization/initial_state.cpp` `legacy_damping`.
4. Comparator guessing and named-set filters in tests
   (`ft2_reference_support.hpp` `kRemovedPassiveLeaves`, tier/families/
   try_tail/try_scalar_rows; `test_native_encode_columns.cpp`,
   `test_period_buffer.cpp`, `test_native_spindle_controller.cpp`,
   `test_state_codecs.cpp`, `test_catalog.cpp`,
   `test_native_runtime_rollout.cpp`, `test_replay_recording_v3.cpp`,
   `test_welded_profile_config.cpp`).
5. `leg_track_damping_nms_rad` (rider controller viscous term on hip/knee
   velocity error, default 0, never set nonzero in tracked configs).

Out of scope (not joint damping): suspension dampers (fork/shock), tire
and contact damping, freehub compliance, brake hold, saddle interface,
controller `joint_kd_nms_rad` (PD posture gain), seated-variant tissue
`damping_ratio` used only by the apparent-mass analysis. Listed in the
final report so the user can widen scope explicitly.

## Phases

### Phase 1 — code (no fixture edits)

- Remove items 1–5 above. Add a regression test: every builder mode's
  compiled model has all `dof_damping == 0`.
- Expected red after this phase: the goldens listed in Phase 3.

### Phase 2 — record mode for goldens

- `bt::oracle::Node`: `BIKE_GOLDEN_RECORD=<dir>` makes `f64`,
  `f64_close`, `bytes` store the actual payload (label, kind and
  tolerances kept; shape taken from actual), and `done()` writes the
  node's `.json`/`.bin`. Unchanged nodes rewrite byte-identically.
- Custom comparators that read expected values through
  `f64_close_input`/`bytes_input` get an explicit record path.
- Bootstrap writer: native tool that runs setup and writes a
  `capture_bootstrap` directory (config.json, state.json, state.arrays,
  model.mjb, integration.f64, meta.json) in the format
  `runtime/bootstrap_io` reads.

### Phase 3 — re-record darwin-arm64

- Rebuild the 33 stored models and their bundles; regenerate
  `model/welded.xml`; episode goldens through
  `bike-recording golden freeze`; DuckDB recordings through the native
  recorder; then every golden test in record mode.
- Review: `git diff --stat` must touch only model-dependent nodes; per
  ADR-0003 rule 3 keep before/after `bike-recording golden compare`
  output for the episode goldens.

### Phase 4 — re-record linux-aarch64 (container)

### Phase 5 — verification and docs

- `tools/run_tests.sh native` and `full` on darwin; the same in the
  Linux container.
- ADR-0003 consequence note: goldens re-recorded natively on 2026-11-xx
  because the model changed.

## Revised approach (2026-11-04, after measurement)

Two measurements make the bootstrap writer and per-bundle provenance
reconstruction unnecessary:

1. `tests/fixtures/model/welded.xml` (Python-captured) with every
   `damping` attribute removed matches the current builder output in every
   compiled array `model_mjcf` compares (test passes). Zeroing
   `dof_damping` in a stored model is therefore the same model the builder
   now produces — no rebuild from configs is needed.
2. In all 11 stored bootstrap states that ship a model, every damped
   joint has `qvel == 0` at t=0 (`max |damping*qvel| = 0`). Damping does not
   enter the static equilibrium, so the stored states stay exact.

New steps:

- Patch the 33 stored `model.mjb` files: load, set `dof_damping = 0`,
  save; update the digests that name them (`state.json` `model_digest`,
  `meta.json` `model_sha256`, episode manifests). Linux `model.mjb`
  files are patched the same way on macOS.
- Re-run `bike-settle` for `tests/fixtures/initial_state/*` (their
  contract hash covers the physics config, which lost
  `leg_track_damping_nms_rad`).
- Give record paths to the few comparators that read the expected side
  themselves (`consume_tree` family, encode_columns, period_buffer,
  spindle_controller, runtime_rollout), then delete the guessing and the
  named-set filters.
- Run the suite in record mode; review which goldens changed and that the
  changes are physically plausible (energy/realtime checks stay green).
- DuckDB recordings: decide per fixture (2 embed the damped model and
  replay physics; the others only carry removed columns).
- Delete tests whose only purpose was legacy damping tolerance.
- Linux: only the record-mode run needs the container.

## Order agreed with the user (2026-11-04, revised)

1. Remove only `coupled_task_control` and its two slow tests
   (`reference.coupled_rider_task`, `reference.native_coupled_rider_task`).
2. Finish this damping work.
3. Next task, own plan: remove the whole old rider control path (closed-form
   plan Task 11). Measured 2026-11-04: it is spread across the legacy
   controller/allocator, the flat-pad and spring-grip branches of
   `writers/rider_contacts.cpp` and `rider/support_dispatch.cpp`, weld
   builder branches, ~15 config fields; at least 19 tests use the old
   attachments explicitly or by default, and `flat` doubles as "no
   attachments" for riderless configs, so the three attachment selector
   fields must be deleted rather than restricted. Record mode makes a
   second re-record cheap, so this does not block step 2.
4. Separate task: the `energy.constraint_work` violation in the first three
   intervals (t <= 0.005 s) of every ride, which stops every strict run.

All 33 stored models already use the new rider topology (no ankles, point
connects; "welded" in fixture names is historical).

Linux is deferred by the user (2026-11-04: «линукс пока не делай») — no container
work in this task. When it is done, it is three runs, not one: re-settle the two
`initial_state/linux-aarch64` files, the record-mode run, and the final
`native` + `full` verification.

## Status

- [~] Phase 1 — code done, not committed (2026-11-04):
  - builders emit no `damping` attribute; physical_topology overrides gone;
  - `rider_passive_damping` component removed (`human_joint_power_w` now
    equals active power; ledger `passive_joint_work_j` removed);
  - legacy readers removed (rebuild.cpp, state_codec.cpp, initial_state.cpp;
    `recording/database_validate.cpp` restored to its pre-389959e7 text —
    every later change there was the legacy tolerance);
  - `leg_track_damping_nms_rad` removed (config, controller, codec);
  - text input fixtures stripped of the removed keys with
    `/tmp/njd_tools/strip_keys.py` / `strip_arrays.py` (format-preserving,
    semantic check against json.loads) — 84 files;
  - NOT yet done: test-side filters/guessing (they go with re-recording),
    `test_replay_recording_v3` legacy gate, spindle legacy-knob test,
    test_schema count comment, regression test "all dof_damping == 0".
  - Ordinary suite after these edits: 28/185 fail, all on fixtures/goldens
    built with the removed fields or the damped model.
- [~] Phase 2 — record mode in `native/tests/support/golden.{hpp,cpp}`
  (`BIKE_GOLDEN_RECORD=1`, `record_consumed`). Self-check: 6 unaffected
  suites rewrote 1316 files byte-identically.
  - Open: no native writer for `capture_bootstrap` bundles exists. Needed:
    expose the runtime-config envelope (`runtime/project_config.cpp:520`)
    and the state document (`initialization/capture.cpp`
    `capture_initialized`) and write config.json/state.json/model.mjb/
    meta.json/integration.f64; reconstruct each bundle's setup inputs.
  - Initial-state files: `bike-settle` writes them
    (`tests/fixtures/initial_state/<platform>/*.json`).
  - Episode bundles: `bike-recording golden freeze`.
- [x] Step 1 (revised order) — `coupled_task_control` removed: config
  field + FieldDecl, validation list, codec keys, coupled branches in
  `rider/legacy_controller.cpp` (allocator crank-task internals left for
  the old-path removal task), both coupled tests and their CMake entries,
  the light-init `braked_relaxation=false` rig branch, 43 config fixtures
  + schema.json stripped. config_schema articulated count 69 -> 67.
- [x] Step 2 — 25 darwin/shared `model.mjb` checked to round-trip
  byte-identically, then patched (`/tmp/njd_probe/patch.cpp`: 22 x 4 pivot
  dofs, legacy_env x 7, 2 riderless rigs unchanged); 17 old->new digests
  replaced in 27 bundle meta/state docs and in place (same length) inside
  4 darwin golden .bin inputs. Linux files untouched (deferred).
- [x] Step 3 — no re-settle needed: the contract only changed because the
  zero-valued `leg_track_damping_nms_rad` left the config. New contract
  hashes obtained by a temporary diagnostic print (reverted) and written
  into the 5 darwin initial-state documents (2 in initial_state/, 3 golden
  env inputs).
- [x] Step 4 — comparators without guessing or named-set filters:
  `consume_tree` (ft2_reference_support), encode_columns, period_buffer,
  spindle_controller, state_codecs, catalog, runtime_rollout; record paths
  in `golden.{hpp,cpp}` (`drop_consumed`, within-tolerance entries kept),
  `bt::recording()`/`golden_f64`/`golden_text` in fixture_test, research and
  runtime bootstrap oracles. Legacy-tolerance tests deleted (replay_v3
  damping gate, spindle activation+damping node and its goldens); the
  spindle/welded-profile rejection tests stay (removed keys reject).
  Embedded bundles inside 4 darwin goldens migrated
  (`/tmp/njd_tools/migrate_bundles.py`: keys stripped, embedded model
  patched, manifest sizes); `restored_state` input stripped; stale
  `assets.json` digests (ft2 research_flat/savage, both platforms) and
  `replay/legacy_v1/replay.json` refreshed.
- [x] Step 5 — `replay/legacy_v2` migrated (intervals.jsonl keys,
  telemetry.csv 894 -> 869 columns, replay.json digests). The 7 DuckDB
  fixture dirs (`recording/{good,bad_presence,no_manifest,report_failed,
  value_cases}`, `replay/{native_v3,tampered}`) are used by no test: the
  whole suite (232 tests incl. slow) passed with them moved away; deleted.
- [x] Step 6 — record-mode run on darwin-arm64, then compare-mode run of
  all 232 tests: green. Changed goldens are only model-dependent nodes;
  episode golden `pinned_savage_40` re-frozen (`bike-recording golden
  freeze`), before/after compare in
  `docs/superpowers/audits/2026-11-04-no-joint-damping-episode-*.txt`.
- [x] Step 7 (part) — regression test `model_mjcf`: every builder mode and
  profile compiles with all `dof_damping == 0`; ADR-0003 note.
- [x] Gates: `tools/run_tests.sh full` on darwin exit 0 (2026-11-04: 185
  ordinary + 47 slow + 1 package tests, all sweeps). Two earlier runs failed
  on analyzer findings in the new test code (cppcheck dangling-view false
  positive from a path member in `Node`, tidy style findings) — fixed.
- [x] 5 Python-frozen episode goldens (`episode/pinned_savage_40_f2`,
  `native_distributed_tire/episodes/{probe5,stand40}`,
  `native_tire/episodes/stand40`, `pneumatic_tire/episodes/fast40`): the 22
  removed channels dropped from `episode.npz` and the manifest `absent`
  lists (`/tmp/njd_tools/migrate_episode.py`; every kept value and every
  other array byte-identical; their tests pass).
- [x] GPU shared-physics plans (master, s0, s1): baseline note added — their
  goldens and digests are captured after this branch merges.
- [x] AGENTS.md: "Демпфирование" section (no joint damping; fork/shock
  dampers, tyre damping and the freehub always stay).
- Open (asked the user): the `seated` rider variant (physics_mode
  "legacy", `bike-ride` default `--rider seated`) applies spring-damper
  forces on its slide joints (`writers/rider_forces.cpp`; saddle
  k=101000 N/m, c=2762 N s/m; torso/arms/legs from `damping_ratio`). The
  earlier note that `damping_ratio` is used only by the apparent-mass
  analysis was wrong.
- Linux fixtures not re-recorded (deferred).
