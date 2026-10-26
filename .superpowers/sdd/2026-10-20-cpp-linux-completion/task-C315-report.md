# Task C315 — batch 3.15 rider control (T52), lane/c

Worktree `.worktrees/pfc-lane-c`, branch `lane/c`, base `2c24dc43`.
Batch: 103 records across 8 source files. **Batch NOT complete — clean FILE split recorded below.**

## Commits

- `7ffc881c` — port `test_crank_split.py` (9 records) + `test_leg_loop.py` (7 records); new shared support `native/tests/reference/rider_control_reference_support.hpp` (pinned model/rig, `idle_control`, `advance_env_steps`); Phase3Ports registration.
- `02cd6cfa` — port `test_coupled_rider_task.py` (4/5 records); env.step fidelity fixes in the shared support header; registration `reference.coupled_rider_task` (`t52 rider slow`); ledger updates for the touched records.
- `538629f4` — port `test_closed_form_rider.py` (7/8 records); `pinned_rig` now runs the real `init::settle_context` braked relaxation (was bare `initialize_pose`); `SpindleController::upper_targets` moved public for direct `_upper_targets` parity; ledger updates.
- `d0474e0b` — port `test_joint_strength.py` (15 records): curve validation, fixture ordering/velocity limits, isometric/concentric/eccentric interpolation, direction sign, positive power budget, profile coverage, provenance gate, profile rejection, anatomical range, strength/envelope coordinate agreement, pure-motor + dof_damping split, stepped effort, directional capacity, 120 RPM envelope.
- `97838b78` — port `test_planar_arms.py` (11 records): paired-arm mass split (total mass / no added mass or strength), COM + sagittal inertia, hand mass inside forearm, 11-joint registry without shared `rider_shoulder`, envelope schema requires two-arm topology + fail-closed malformed data, compiled topology (per-side arm bodies/joints/grip sites/connect equalities, no paired arm or midline grip), per-hand Jacobian isolation, welded-hand reactions, initial-state capture/restore round-trip of both `grip_anchor_local` anchors. Raw MuJoCo arrays accessed via `model_access::readonly_buffer` spans to satisfy `-Werror=unsafe-buffer-usage`.

## Dispositions

| Source file | Records | Disposition |
|---|---|---|
| tests/reference/test_crank_split.py | 9 | port — `reference.crank_split`, PASS 23 checks darwin-arm64 |
| tests/reference/test_leg_loop.py | 7 | port — `reference.leg_loop`, PASS 65 checks darwin-arm64 |
| tests/reference/test_coupled_rider_task.py | 5 | **4 port / 1 deferred** — `reference.coupled_rider_task`, PASS 19 checks darwin-arm64 |
| tests/reference/test_closed_form_rider.py | 8 | **7 port / 1 deferred** — `reference.closed_form_rider`, PASS 751 checks darwin-arm64 |
| tests/reference/test_joint_strength.py | 15 | port — `reference.joint_strength`, PASS 408 checks darwin-arm64 |
| tests/reference/test_planar_arms.py | 11 | port — `reference.planar_arms`, PASS 187 checks darwin-arm64 |
| tests/reference/test_native_rider_intent.py | 26 | **remaining** — oracle-parity file, needs `_golden` instrumentation + darwin/linux golden capture |
| tests/reference/test_native_spindle_controller.py | 22 | **remaining** — same |

Deferred record 1: `test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step` — **native/oracle parity gap, not a harness bug**: `predicted_crank_torque_nm` (wire `solution_crank_torque_nm`) matches `diagnostics.rider_welds.crank_torque_nm` bitwise for ~18 control periods, then diverges at t≈0.19 s (10.310 vs 9.689 N·m; oracle bound 1e-3 rel). Python oracle itself passes 5/5 in 200 s. Function kept in the .cpp under `#if 0` with an explanatory comment; ledger record stays `planned` with evidence.

Deferred record 2: `test_flat_launch_delivers_crank_torque` — **same delivered-torque parity family**: native pedelec assist engages on 279/401 = 69.6% of late intervals vs the oracle's 351/400 = 87.8%. Both sides cut the motor exactly when `human_sensor_nm` dips below ~4 N·m or during shifts — the divergence is the measurand: native dead spots run deeper/longer and native shift + post-shift re-engagement windows last ~2–3× longer (122 vs 49 off-ticks). The `>=80%` gate stays verbatim under `#if 0`; the other three asserts live and pass. Ledger record stays `planned`.

## Qualification matrix

| Test | darwin-arm64 | linux-aarch64 |
|---|---|---|
| reference.crank_split | PASS 23 | **pending** |
| reference.leg_loop | PASS 65 | **pending** |
| reference.coupled_rider_task | PASS 19 (~16 min — faithful settle dominates) | **pending** |
| reference.closed_form_rider | PASS 751 | **pending** |
| reference.joint_strength | PASS 408 | **pending** |
| reference.planar_arms | PASS 187 | **pending** |
| Oracle pytest | green — joint_strength 15/15 in 17.58 s; planar_arms 11/11 in 3.93 s (earlier: closed-form 11/11, coupled 5/5 slow) | — |

Ledger evidence strings state "linux-aarch64 qualification pending" — `disposition=port` here means landed+darwin-green only; two-platform sign-off still owed. The final `bash tools/run_tests.sh native` gate has NOT yet been run for this batch.

## Golden-capture convention (for the two remaining files)

`test_native_*.py` files use `tests/reference/_golden.py`: `import _golden`, then `_golden.record(np.asarray(v), label)` per compared leaf (`input.*` prefix = values the C++ port replays, `expected.*`/output = compared). Capture runs the pytest file with `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden` on each platform lane; each node writes `<platform>/<file>/<test>.{json,bin}` in call order. The C++ port consumes entries via `bt::oracle::Node` (`native/tests/support/golden.hpp`): `f64_input` for inputs, `f64`/`f64_close`/`bytes` for outputs, `peek_label`/`peek_shape` to select output-vs-error capture branches. `test_native_rider_intent.py` and `test_native_spindle_controller.py` are NOT yet instrumented — porting them requires adding `_golden.record` calls at every `_compare`/`pytest.raises` boundary. Byte-flip proof: flip one golden byte, confirm the C++ gate fails, restore.

Native API surface for the intent port: `native/src/rider/intent.hpp` (`RiderIntent`, `SeatedClimbPolicy`, `SeatedPostureProgram`), `native/src/rider/intent_wire.hpp` + `state_codec.cpp` (the adapter's exact wire encode/decode for control/signals/state_dict). Adapter calls exercised by the file: `intent_resolve`, `intent_state`, `intent_update`, `intent_power_target`, `intent_program_update`, `intent_schedule_pulse`, `intent_reset`, `intent_restore` — each maps onto the C++ members the adapter wraps.

## Deviations / findings

1. **`idle_control()` fixed**: Python `RideControl()` leaves all torque channels `None`; pinning `human_torque_nm=0.` disables `automatic_effort` (step.cpp gate), producing ~10 W mean positive power instead of the oracle's >20 W pedalling floor. Now default-constructed (nullopt channels, `rider_enabled=true`).
2. **Brake demands fixed**: `env.step(control)` defaults `front/rear_brake_demand=0.`; the shared advance used `(1.,1.)` = full brakes. Corrected to `(0.,0.)`.
3. **`rider_welds` lookup**: it is a `RawStep.diagnostics` key; `channels` only carries it after PeriodAccounting folds it in.
4. **Diag capture timing**: allocation diagnostics must be read right after the *first* interval of a control period.
5. `bt::run` is fail-closed; a parity-failing function cannot be half-registered.
6. **`pinned_rig` was missing the physical settle** (fixed): ran `initialize_pose` + `finalize_initial_state` only, so every env rig started airborne (`normal_load_n` = 0 → `front_load_share` never published, launch dynamics differed from the oracle's `make_environment` construction settle). `pinned_rig` now calls `init::settle_context` with `equilibrium_cache_enabled=false`. Wheels now carry load at t=0 (front 429 N / rear 595 N on the savage rig).
7. **`upper_targets` moved public** in `native/src/rider/spindle_controller.hpp` — reference-parity surface for `controller._upper_targets(...)`.
8. Settle cost warning: `reference.coupled_rider_task` runs ~16 min on darwin. Keep `slow` labels; budget Linux validation accordingly.
9. `unsafe-buffer-usage/access` sweeps apply to tests: all raw MuJoCo buffers (`eq_obj1id`, `site_xpos`, `xpos`, `jnt_dofadr`, Jacobian buffers) go through `model_access::readonly_buffer` spans.
10. `SegmentMasses` is an ordered vector of name/value pairs; `segment_masses(total_kg, helmet_kg)` preserves Python insertion order; `split_paired_arm_masses` removes `upper_arm_pair`/`forearm_pair` and appends halved left/right entries; `ARM_LATERAL_OFFSET_M` = 0.10.
11. `attachment_samples` are latched/published during period close: `PeriodAccounting::close_period` → `evaluate_attachments` (raw spatial Jacobians + qfrc → rider/bike wrench solves, observability/Newton-third-law/planar checks, errors are entries not zeros) → writer samples.
12. `PhysicalInitialState::capture` serializes per-hand anchors under `grip_anchor_local`; `restore` stages anchors transactionally through native material state (decode+validate all fields, then commit).
13. `migration/test-port-map.json` schema uses `source_path`/`source_symbol` (no `source` key); `python-removal.json` files end without a trailing newline — preserve that when editing ledgers.
14. Known pre-existing issue not touched: duplicate `activation_tau_s` TOML baseline (out of scope).

## Clean FILE split — work handed to the next agent

**Done (6 files, 53 records landed, darwin green):** `test_crank_split.py` (9), `test_leg_loop.py` (7), `test_coupled_rider_task.py` (4/5), `test_closed_form_rider.py` (7/8), `test_joint_strength.py` (15), `test_planar_arms.py` (11). Two records stay `planned` pending the delivered-torque parity fix (same root-cause family, likely one fix).

**Remaining (2 files, 48 records, both `planned`):**
- `tests/reference/test_native_rider_intent.py` — 26 records → `native/tests/reference/test_native_rider_intent.cpp`. Record list (order in file): `test_resolve_matches_oracle_over_mixed_steps`, `test_resolve_disabled_passthrough`, `test_resolve_merges_only_unset_fields`, `test_resolve_partial_merge`, `test_resolve_rider_disabled_keeps_control_but_ticks`, `test_resolve_inactive_returns_control`, `test_probe_advance_preserves_state`, `test_resolve_step_validation`, `test_resolve_skipped_acquisition`, `test_resolve_rewind_requires_reset`, `test_period_must_be_integer_multiple`, `test_config_validation`, `test_signals_validation`, `test_control_validation`, `test_road_grade_validation_is_lazy`, `test_update_validates_before_enabled_check`, `test_policy_update_parity_with_delays`, `test_resolve_enabled_delayed_none_front_load`, `test_policy_inclination_norm_gate`, `test_surge_budget_spend_and_recovery`, `test_program_update_parity`, `test_program_validation`, `test_schedule_pulse_and_reset`, `test_state_roundtrip`, `test_state_rejects_malformed`, `test_state_unknown_signals_keys_rejected` — 26 functions but only 25 named here; recount against `grep -c "def test_"` before ledger update (file has 26 ledger records).
- `tests/reference/test_native_spindle_controller.py` — 22 records → `test_native_spindle_controller.cpp` (`SpindleController` = native facade over `ArticulatedRiderController`; activation/reset, strength/power, posture, crank split, leg loop, diagnostics, state/config encode/decode — adapter calls `controller_*`; check the file's import block for the exact adapter section name).

**Also owed for all 6 landed files:** bounded linux-aarch64 validation via `tools/linux/container.sh` (clang build; run the six landed binaries plus any new ones), one byte-flip proof on a golden fixture, flipping `test_coupled_rider_task.py`/`test_closed_form_rider.py` `python-removal.json` entries to `replacement_landed` only once their deferred records land, and the final `bash tools/run_tests.sh native` gate (must exit 0; ~15–30 min). `full` vs `native` profile decision still owes: scope is test-only + one private-to-public member move — if the launcher treats `native` as including golden/reference tests, `native` suffices; otherwise check `tools/run_tests.sh` profile definitions.
