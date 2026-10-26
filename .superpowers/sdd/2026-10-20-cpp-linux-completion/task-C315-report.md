# Task C-3.15 — rider control (T52, label `t52 rider`) — report

Worktree `.worktrees/pfc-lane-c`, branch `lane/c`, base `2c24dc43`.
Batch: 103 records across 8 source files. **Batch NOT complete — clean FILE split recorded below.**

## Commits

- `7ffc881c` — port `test_crank_split.py` (9 records) + `test_leg_loop.py` (7 records); new shared support `native/tests/reference/rider_control_reference_support.hpp` (pinned model/rig, `idle_control`, `advance_env_steps`); Phase3Ports registration.
- `02cd6cfa` — port `test_coupled_rider_task.py` (4/5 records); env.step fidelity fixes in the shared support header; registration `reference.coupled_rider_task` (`t52 rider slow`); ledger updates for the 21 touched records; `python-removal.json`: `test_crank_split.py` + `test_leg_loop.py` → `replacement_landed`.

## Dispositions (this batch so far)

| Source file | Records | Disposition |
|---|---|---|
| tests/reference/test_crank_split.py | 9 | port — `reference.crank_split`, PASS 23 checks darwin-arm64 |
| tests/reference/test_leg_loop.py | 7 | port — `reference.leg_loop`, PASS 65 checks darwin-arm64 |
| tests/reference/test_coupled_rider_task.py | 5 | **4 port / 1 deferred** — `reference.coupled_rider_task`, PASS 19 checks darwin-arm64 |

Deferred record: `test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step` — **native/oracle parity gap, not a harness bug**: `predicted_crank_torque_nm` (wire `solution_crank_torque_nm`) matches `diagnostics.rider_welds.crank_torque_nm` bitwise for ~18 control periods, then diverges at t≈0.19 s (10.310 vs 9.689 N·m; oracle bound 1e-3 rel). Python oracle itself passes 5/5 in 200 s. Function kept in the .cpp under `#if 0` with an explanatory comment; ledger record stays `planned` with evidence.

## Qualification matrix

| Test | darwin-arm64 | linux-aarch64 |
|---|---|---|
| reference.crank_split | PASS 23 | **pending** |
| reference.leg_loop | PASS 65 | **pending** |
| reference.coupled_rider_task | PASS 19 | **pending** |
| Oracle pytest (3 files) | green (coupled 5/5 slow, 200 s) | — |

Ledger evidence strings state "linux-aarch64 qualification pending" — `disposition=port` here means landed+darwin-green only; two-platform sign-off still owed.

## Deviations / findings

1. **`idle_control()` fixed**: Python `RideControl()` leaves all torque channels `None`; pinning `human_torque_nm=0.` disables `automatic_effort` (step.cpp gate), producing ~10 W mean positive power instead of the oracle's >20 W pedalling floor. Now default-constructed (nullopt channels, `rider_enabled=true`).
2. **Brake demands fixed**: `env.step(control)` defaults `front/rear_brake_demand=0.` (research environment.py:357); the shared advance used `(1.,1.)` = full brakes. Both corrected to `(0.,0.)`.
3. **`rider_welds` lookup**: it is a `RawStep.diagnostics` key; `channels` only carries it after PeriodAccounting folds it in — reading `samples[0].channels` threw `accounting: missing field 'rider_welds'`.
4. **Diag capture timing**: allocation diagnostics must be read right after the *first* interval of a control period (control tick fires on `step % steps_per_period == 0`, i.e. at period head); reading after the whole period pairs a next-period prediction with this period's weld.
5. `bt::run` is fail-closed; no skip/xfail mechanism — a parity-failing function cannot be half-registered.
6. Known pre-existing issue not touched: duplicate `activation_tau_s` TOML baseline (out of scope per instructions).

## Clean FILE split — work handed to the next agent

**Done (3 files, 21 records):** `test_crank_split.py`, `test_leg_loop.py`, `test_coupled_rider_task.py` (4/5; 1 record deferred pending parity fix above).

**Remaining (5 files, 82 records, all `planned`):**
- `tests/reference/test_native_rider_intent.py` — 26 records → `native/tests/reference/test_native_rider_intent.cpp`
- `tests/reference/test_native_spindle_controller.py` — 22 records → `test_native_spindle_controller.cpp`
- `tests/reference/test_joint_strength.py` — 15 records → `test_joint_strength.cpp`
- `tests/reference/test_planar_arms.py` — 11 records → `test_planar_arms.cpp`
- `tests/reference/test_closed_form_rider.py` — 8 records → `test_closed_form_rider.cpp`

Also owed for the landed 3 files: bounded linux-aarch64 validation via `tools/linux/container.sh` (clang build), golden-fixture capture if any new port needs goldens, one byte-flip proof on a golden, and flipping `test_coupled_rider_task.py`'s `python-removal.json` entry to `replacement_landed` once the deferred record lands. Do NOT flip it before then.

Support API ready in `native/tests/reference/rider_control_reference_support.hpp` (`pinned_model`, `pinned_rig` with physics-override lambda + track, `idle_control`, `advance_env_steps`, `expect_invalid`).
