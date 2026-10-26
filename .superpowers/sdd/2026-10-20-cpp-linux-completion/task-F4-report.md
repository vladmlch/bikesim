# Task F4 — coupled-rider planned-vs-solved weld torque parity (native fix)

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e`, base `0f63db5e`.
No merge, no push. lane/c untouched.

**Invariant** (`tests/reference/test_coupled_rider_task.py`::
`test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step`):
for the first physics step of every control period in the fully coupled
rider task, the allocator's planned crank torque
(`solution_crank_torque_nm` → `predicted_crank_torque_nm`) equals the
solved pedal-weld torque (`rider_welds.crank_torque_nm`) within
`1e-3 * max(1, |planned|)`, over ≥ 20 checked periods. Python passed;
native diverged.

## Root cause — runtime forward-state sequencing, not the allocator

The oracle's control tick in `physical_runtime.py:381-384` is three
ordered statements:

```python
d.qfrc_applied[:] = acc.total()
mujoco.mj_forward(m, d)
torques = self.rider_control.compute(m, d, ...)
```

`apply_forces_body` in `native/src/runtime/step.cpp` staged the same
`qfrc_applied` copy but went straight to `compute()`. The controller's
`ConstraintDynamics` snapshots `qfrc_actuator`, `qfrc_bias`,
`qfrc_passive` and the efc arena out of `mjData` — so natively it read
the *previous* interval's post-step solve outputs (the zero-input
forward the oracle comment calls out as insufficient). `ensure_posvel_
fresh` could not repair this: the defect was the force/constraint arena,
not stale kinematics.

## Evidence trail

- First false lead eliminated: an early probe compared the plan against
  `delivered_crank_torque_nm`, which `evaluate()` deliberately
  republishes one step stale from the previous settle latch
  (`rider_contacts.cpp`). The correct read is the fresh
  `settled_welds->crank_torque_nm` — the value the `rider_welds` wire
  emits (`wire_settled_welds`).
- First real divergence after the corrected read: **step 148,
  t = 0.1863 s** — planned `10.452644` N·m vs solved `9.808642` N·m
  (gap 0.644 N·m). It coincided with `nefc` growing 31 → 32, initially
  suggesting an active-set port defect.
- Three-way same-solve replay disproved that:
  - `ConstraintDynamics` rebuilt on the live post-step arena, evaluated
    at the allocator's torque: worst deviation vs the engine's
    `efc_force` ≈ 1.8e-12 — the model matches the solve.
  - The *stale* `qfrc_actuator` buffer (captured before the tick's
    forward) replayed through the same model: worst deviation vs the
    allocator's stored `constraint_forces` = **0.0** — the allocator
    correctly solved stale inputs.
  - One actuator DOF's force differed by ≈ 1.245280 N between the stale
    and refreshed buffers — the propagated weld-projection error.
- Fix: `refresh_forward(m, d)` between the `qfrc_applied` copy and
  `compute()` (`native/src/runtime/step.cpp`, `apply_forces_body`,
  control-tick branch). `refresh_forward` is the established
  `mj_forward` idiom (`forward_skip` when posvel is fresh, full
  `forward` otherwise); the same helper already mirrors the oracle's
  later forward at the interval end.

## Commits

- `df16bd4f` — `fix(runtime): refresh forward state before rider
  control-tick compute` (10 insertions incl. the port-mapping comment).
- `cbf3ffe6` — `test(reference): port coupled-rider planned-vs-solved
  weld invariant` — new CTest `reference.native_coupled_rider_task`
  (labels `t52 rider slow`).

## Regression coverage

`native/tests/reference/test_native_coupled_rider_task.cpp` rebuilds
the oracle rig exactly: `welded_flat.toml` + weld pedals/saddle/grips +
`coupled_task_control = true` + `joint_passive_damping_nms_rad = 0.5`,
0.6 s at `timestep_s` per `control_clock`. Each control tick it reads
`predicted_crank_torque_nm` from the post-advance allocation
diagnostics and the published `raw.diagnostics['rider_welds']
['crank_torque_nm']` wire (the channel value accounting copies at
period close), applies the oracle's relative bound with the same skip
rules (no prediction / `effort_scale == 0`), and requires ≥ 20 checked
periods — matching the oracle's contract bit for bit, including reading
the fresh settle rather than the stale `delivered_crank_torque_nm`
sensor channel.

## Verification

- Darwin arm64, `build/native-release` (Apple clang, Release):
  - `reference.native_coupled_rider_task` — **PASS 280 checks**
    (57 checked control periods; typical |planned−solved| ~1e-14…1e-12;
    active-set transitions incl. `nefc` 31→32 at t=0.1863 s all inside
    bound; formerly failing step 148 now equal within noise).
  - `ctest -R 'rider|golden|stepper|brake|suspension'` — 14/15 pass;
    `reference.golden_episode_tool` red with the documented baseline
    signature (`FAIL native recorder rows reproduce Python oracle` —
    pre-existing recorder drift, C318/C320 reports).
- Python oracle (unchanged): `uv run pytest
  tests/reference/test_coupled_rider_task.py -x -q` → **5 passed**
  (141 s).
- Golden families inspected: `pinned_savage_40` /
  `pinned_savage_40_f2` (articulated rider episodes — rider-forces,
  brake-resistance, suspension, episode-regression and codec/tool CTest
  consumers) and the seated-pedaling family (`test_native_rider_forces`
  seated-variant + `test_native_drive_policies` pedaling-branch
  goldens). All CTest consumers pass on Darwin except the documented
  `golden_episode_tool` baseline; pytest golden consumers run under the
  full gate below.
- Linux aarch64 (`bike-dev-pfc-lane-e` container created from
  `bike-dev-linux:24.04`, cmake 3.28.3, `cmake --preset release` +
  `cmake --build --preset release`, 596 targets clean):
  - `reference.native_coupled_rider_task` — **PASS 275 checks**
    (56 checked control periods; one fewer than Darwin — one tick fell
    into the oracle's skip rule (`effort_scale == 0` or unset
    prediction), i.e. fewer comparisons, never a failing one).
  - `ctest -R 'rider|golden|stepper|brake|suspension'` — 14/15 pass;
    `reference.golden_episode_tool` red with the same documented
    baseline signature as Darwin.

## Full gate

`bash tools/run_tests.sh full` (darwin-arm64, once, log
`output/full_gate.log`): **exit 8** — matching the documented baseline
gate verbatim (F1-era evidence entry: exit 8, CTest 60/62, reds
`migration_T09` + `migration_T10`, "zero unexpected reds").

- Extension build at `native/build`: clean (mujoco 3.12.0 pin).
- All nine analysis stages green: `check_frontends` none, `check_tidy`
  none, `check_analyzer` report-only alpha (unchanged pre-existing
  patterns), `check_cppcheck` none, `check_odr` (gcc-16 -flto -Wodr)
  none, `check_scripts` shellcheck clean; produced checker kinds
  `headers` (9012 header TUs, findings none), `diagnostic-controls`
  (6/6), `context` all green.
- `ctest --test-dir native/build`: **60/62** — reds `migration_T09`
  (`post-advance integration vector matches the oracle`) and
  `migration_T10` (`first transition matches the Python oracle`), both
  the documented frozen-oracle family (immutable T01-era captures;
  documented baseline pair, unchanged signature shape). No other reds.
- Profile stops at the first red stage per documented gate-abort
  behavior (pytest phase never reached — same as the recorded baseline
  gate). Contracts checker run standalone per that procedure:
  `bike-native-checks --kind contracts` → **all assertions passed**
  (`loader_provenance` seeded by a provenance-recording `load_native()`
  import, as the gate's own preflight does).

## Supplementary parity evidence (pytest, extension `native/build`)

The gate-abort left the pytest phase unexercised, so the affected
families were run directly against the fixed extension
(`NATIVE_TEST_BUILD_PATH`/`BIKE_NATIVE_BUILD_PATH` =
`native/build`):

- `test_native_rider_forces.py` + `test_native_drive_policies.py` —
  **120 passed** (pinned_savage_40 episode-force goldens + pedaling
  parameter/sequence branches).
- `test_golden_episode.py` + `test_seated_pedaling_cycle.py` +
  `test_seated_posture_program.py` + `test_seated_climb_surge.py` —
  **31 passed** (255 s; seated-pedaling family green).
- `test_native_rider_contacts.py` + `test_native_stepper.py` —
  all passed inside the combined 68-pass run.
- `test_native_runtime_step.py` — 6 reds
  (`first_step_uses_settled_material_state`,
  `step_sequence_matches_python_state`,
  `force_fold_matches_python_recorded_inputs`,
  `advance_tracks_python_under_load`,
  `sensor_channels_read_sensordata_rows`,
  `intent_lanes_match_python_past_reaction_delay`).
  **A/B-verified pre-existing**: the same extension built with the
  pre-fix `step.cpp` (`git show df16bd4f^:...`) fails the identical six
  nodes with identical bitwise signatures (e.g. first diff
  `5.66934492e-03` vs `5.66934491e-03`). These live-oracle parity nodes
  sit past the gate's ctest abort, so their redness is not visible in
  gate output; it predates F4 and is unrelated to this change (the
  same-shape frozen-parity drift family as the T09/T10 pair).

## Scope limits

- `tools/run_tests.sh full` was run exactly once as required; its exit
  8 + {T09, T10} red set equals the documented baseline gate set, so
  no re-run was triggered per the baseline-comparison rule.
- The `test_native_runtime_step.py` live-parity reds are recorded as
  pre-existing (A/B-verified) — fixing that drift is outside the F4
  surgical-fix scope.
