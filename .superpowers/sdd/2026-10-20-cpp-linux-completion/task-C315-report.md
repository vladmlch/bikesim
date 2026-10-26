# Task C315 — batch 3.15 rider control (T52), lane/c

Worktree `.worktrees/pfc-lane-c`, branch `lane/c`, base `2c24dc43`.
Batch: 103 records across 8 source files. **Batch NOT complete — see the
qualification matrix and the deferred-record list.**

Update 2026-10-26/27 (this session): the intent consumer landed
(`c77380a8`, 23,309 checks over 26 nodes, darwin) and is now
linux-qualified; the spindle file is instrumented and its 20 oracle-green
nodes are replayed by `test_native_spindle_controller.cpp` (1,971 checks)
on both platforms. The remaining 22 spindle nodes are deferred on two
documented grounds below; `reference.coupled_rider_task` additionally
fails linux-aarch64 on the tissue-damping episode record.

## Commits

- `7ffc881c` — port `test_crank_split.py` (9 records) + `test_leg_loop.py` (7 records); new shared support `native/tests/reference/rider_control_reference_support.hpp` (pinned model/rig, `idle_control`, `advance_env_steps`); Phase3Ports registration.
- `02cd6cfa` — port `test_coupled_rider_task.py` (4/5 records); env.step fidelity fixes in the shared support header; registration `reference.coupled_rider_task` (`t52 rider slow`); ledger updates for the touched records.
- `538629f4` — port `test_closed_form_rider.py` (7/8 records); `pinned_rig` now runs the real `init::settle_context` braked relaxation (was bare `initialize_pose`); `SpindleController::upper_targets` moved public for direct `_upper_targets` parity; ledger updates.
- `d0474e0b` — port `test_joint_strength.py` (15 records): curve validation, fixture ordering/velocity limits, isometric/concentric/eccentric interpolation, direction sign, positive power budget, profile coverage, provenance gate, profile rejection, anatomical range, strength/envelope coordinate agreement, pure-motor + dof_damping split, stepped effort, directional capacity, 120 RPM envelope.
- `97838b78` — port `test_planar_arms.py` (11 records): paired-arm mass split (total mass / no added mass or strength), COM + sagittal inertia, hand mass inside forearm, 11-joint registry without shared `rider_shoulder`, envelope schema requires two-arm topology + fail-closed malformed data, compiled topology (per-side arm bodies/joints/grip sites/connect equalities, no paired arm or midline grip), per-hand Jacobian isolation, welded-hand reactions, initial-state capture/restore round-trip of both `grip_anchor_local` anchors. Raw MuJoCo arrays accessed via `model_access::readonly_buffer` spans to satisfy `-Werror=unsafe-buffer-usage`.
- `a60ce0ea` — `test_native_rider_intent.py` `_golden` instrumentation + darwin-arm64 capture (26 records). Oracle file still 26/26 green with records inert.
- `c77380a8` — `test_native_rider_intent.cpp` golden consumer + Phase3Ports registration (`reference.native_rider_intent`, `t52 rider golden`): replays `input.<op>.k` JSON docs through `decode_intent_*` and the native intent stack, `peek_label()` selects output-vs-error branches, `output.*` compared byte-exact (canonical JSON) or as f64 scalars. PASS 23,309 checks on darwin-arm64.
- this session (uncommitted at report time) — `test_native_spindle_controller.py` `_golden` instrumentation (same contract: `input.<op>.k` f64-packed canonical-JSON docs, `output.*` bitwise-JSON or close-f64 leaves, `error.<op>.k` empty |u1 markers, `_SEQ` per-node counters) + darwin-arm64 capture of the 20 oracle-green nodes; `test_native_spindle_controller.cpp` consumer + `reference.native_spindle_controller` registration; linux-aarch64 captures for both files.

## Dispositions

| Source file | Records | Disposition |
|---|---|---|
| tests/reference/test_crank_split.py | 9 | port — `reference.crank_split`, PASS 23 checks darwin-arm64 |
| tests/reference/test_leg_loop.py | 7 | port — `reference.leg_loop`, PASS 65 checks darwin-arm64 |
| tests/reference/test_coupled_rider_task.py | 5 | **4 port / 1 deferred** — `reference.coupled_rider_task`, PASS 19 checks darwin-arm64 |
| tests/reference/test_closed_form_rider.py | 8 | **7 port / 1 deferred** — `reference.closed_form_rider`, PASS 751 checks darwin-arm64 |
| tests/reference/test_joint_strength.py | 15 | port — `reference.joint_strength`, PASS 408 checks darwin-arm64 |
| tests/reference/test_planar_arms.py | 11 | port — `reference.planar_arms`, PASS 187 checks darwin-arm64 |
| tests/reference/test_native_rider_intent.py | 26 | port — `reference.native_rider_intent`, PASS 23,309 checks darwin-arm64 + PASS linux-aarch64 (26/26 node goldens both platforms) |
| tests/reference/test_native_spindle_controller.py | 22 records / 42 nodes | **14 records port (20 nodes) / 8 records deferred (22 nodes)** — `reference.native_spindle_controller`, PASS 1,971 checks darwin-arm64 + PASS linux-aarch64 |

Deferred record 1: `test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step` — **native/oracle parity gap, not a harness bug**: `predicted_crank_torque_nm` (wire `solution_crank_torque_nm`) matches `diagnostics.rider_welds.crank_torque_nm` bitwise for ~18 control periods, then diverges at t≈0.19 s (10.310 vs 9.689 N·m; oracle bound 1e-3 rel). Python oracle itself passes 5/5 in 200 s. Function kept in the .cpp under `#if 0` with an explanatory comment; ledger record stays `planned` with evidence.

Deferred record 2: `test_flat_launch_delivers_crank_torque` — **same delivered-torque parity family**: native pedelec assist engages on 279/401 = 69.6% of late intervals vs the oracle's 351/400 = 87.8%. Both sides cut the motor exactly when `human_sensor_nm` dips below ~4 N·m or during shifts — the divergence is the measurand: native dead spots run deeper/longer and native shift + post-shift re-engagement windows last ~2–3× longer (122 vs 49 off-ticks). The `>=80%` gate stays verbatim under `#if 0`; the other three asserts live and pass. Ledger record stays `planned`.

Deferred records 3–8 (spindle file, 20 nodes over 6 records): `test_spindle_compute_matches_python` (12 parametrized nodes), `test_spindle_steady_state_bypasses_activation`, `test_spindle_state_round_trip`, `test_spindle_solved_effort_matches_python`, `test_spindle_compute_posture_matches_python` (4 nodes), `test_spindle_coasting_command_matches_python` — **native/oracle enabled-compute parity defect**: every enabled `compute` diverges in the torque map; the rear-leg chain collapses natively (`rider_hip_rear`/`rider_knee_rear` read 0.0 vs oracle −1.310/−0.261 N·m on `compute[0.0-0.0]`; front hip/knee off ~35%: −0.968 vs −1.541, 0.293 vs 0.351; torso/shoulder/elbow match). Allocation diagnostics diverge in the rear chain (solution excitation `[0.,−0.]` vs oracle `[−26.87,−5.34]`). Identical 22-failure set reproduces on darwin-arm64 and linux-aarch64, so it is a delivered implementation defect, not a platform or harness issue. Instrumented input streams exist; fixtures for these nodes were deliberately not committed (unconsumable — replaying them would fail closed). Records stay `planned` with evidence.

Deferred records 9–10 (spindle file, 2 nodes): `test_spindle_activation_rejects_repeated_timestamp`, `test_spindle_activation_and_passive_damping_match_python` — **`_damped_config` TOML baseline**: the helper rewrites the pinned TOML in place and trips `tomllib.TOMLDecodeError: Cannot overwrite a value (at line 112, column 25)` inside `_plant` before any golden event. No fixture exists for these nodes. Records stay `planned` until the pinned-config baseline is re-anchored.

Deferred record 11 (coupled file): `test_tissue_damping_does_not_eat_the_muscle_budget` — **linux-aarch64 only**: the oracle-side pytest passes on the linux container (894 s) and the native binary passes darwin-arm64 (1,199 s), but the native 1.5 s episode on linux lands `passive >= 0.3*positive` — a native-side platform drift plausibly in the delivered-power parity family (records 1–2). Record flipped back to `planned` with evidence; the binary is red on linux until fixed.

## Stale-extension diagnosis (this session)

The first instrumented spindle run reported 32/42 failures of the form
`adapter.spindle_controller.config.weld_foot_min_normal_n: unknown key`
— a schema error that source inspection could not reproduce
(`kConfigRequiredKeys` already accepted the key). Cause: the Darwin
`bike_native` extension predated the current `spindle_wire.cpp`; rebuilding
the .so replaced all schema failures with the true baseline: 20 passed /
22 failed, the failures being the real compute defect plus the two damped
TOML nodes. Lesson: on this lane always rebuild the extension before
reading parity results off it.

## Gate-blocking tidy regression (this session)

The first `run_tests.sh native` attempt died at `check_tidy`: `native/src/drivetrain/shifting.cpp:66` — `modernize-loop-convert`, introduced earlier today by lane commit `5bbea70a` ("mirror multi-cog shift throw selection", reverse-iterator loop over the cassette). Fixed in place: `for (int const teeth : config_.cassette | std::views::reverse)` — identical descending iteration order and body (C++20 `<ranges>` already used tree-wide). The nanobind `clang-analyzer-cplusplus.Move` diagnostic in `contact_binding.cpp` is an allowlisted suppression, not a blocker.

## Byte-flip proof (this session)

`tests/fixtures/golden/darwin-arm64/tests_reference_test_native_spindle_controller.py/test_spindle_disabled_command_matches_python.bin`, byte 98,400 inside the `output.torques.0` |u1 JSON payload flipped `0x46→0x47`:
`reference.native_spindle_controller` → `FAIL ... comparison 4 label 'output.torques.0': bytes differ`, exit 1. Restored byte → PASS 1,971 checks, exit 0. (A byte flip inside an `input.*` lane was also tried first and did NOT fail — inputs are f64-packed one-ASCII-byte-per-lane, so a low-mantissa-bit flip truncates to the same character; the meaningful corruption targets are `output.*`/`error.*` payloads or sidecar metadata.)

## Qualification matrix

| Test | darwin-arm64 | linux-aarch64 |
|---|---|---|
| reference.crank_split | PASS 23 | PASS |
| reference.leg_loop | PASS 65 | PASS |
| reference.coupled_rider_task | PASS 19 (~16 min — faithful settle dominates) | **FAIL** — `test_tissue_damping_does_not_eat_the_muscle_budget` (960 s) |
| reference.closed_form_rider | PASS 751 | PASS |
| reference.joint_strength | PASS 408 | PASS |
| reference.planar_arms | PASS 187 | PASS |
| reference.native_rider_intent | PASS 23,309 | PASS (26/26 goldens) |
| reference.native_spindle_controller | PASS 1,971 | PASS (20/20 committed goldens) |
| Oracle pytest | green | test_native_rider_intent 26/26; test_native_spindle_controller 20/22 blocked as documented; coupled oracle-side tissue-damping green (894 s) |

Ledger flips (`migration/test-port-map.json`): all 26 intent records and the 14 qualified spindle records → `collected_node_ids` measured, `assertions_capture` dated "oracle green darwin-arm64 + linux-aarch64", `comparison_class=bitwise-oracle`, `disposition=port` with evidence. The 8 spindle blocked records + `test_tissue_damping_does_not_eat_the_muscle_budget` stay `planned` with defect evidence. `migration/python-removal.json`: `test_native_rider_intent.py` → `replacement_landed` (verification id `reference.native_rider_intent`, sha refreshed to the instrumented file); `test_native_spindle_controller.py` stays `planned` (22/42 nodes unqualified); `test_coupled_rider_task.py`/`test_closed_form_rider.py` stay `planned` (deferred records outstanding).

## Golden-capture convention

`test_native_*.py` files use `tests/reference/_golden.py`: `import _golden`, then `_golden.record(np.asarray(v), label)` per compared leaf (`input.*` prefix = values the C++ port replays, `output.*`/`error.*` = compared branches). Capture runs the pytest file with `BIKE_GOLDEN_CAPTURE=tests/fixtures/golden` on each platform lane; each node writes `<platform>/<file>/<test>.{json,bin}` in call order. The C++ port consumes entries via `bt::oracle::Node` (`native/tests/support/golden.hpp`): `f64_input` for inputs, `f64`/`f64_close`/`bytes` for outputs, `peek_label`/`peek_shape` to select output-vs-error capture branches. Both `test_native_*.py` files in this batch are instrumented and captured on both platforms. Byte-flip proof documented above.

Native API surface for the intent port: `native/src/rider/intent.hpp` (`RiderIntent`, `SeatedClimbPolicy`, `SeatedPostureProgram`), `native/src/rider/state_codec.hpp` + `native/src/io/json.hpp` (`parse_recording_json`, `recording_json`, `canonical_json`). Adapter-call → C++ mapping: `intent_resolve`→`RiderIntent::resolve`, `intent_state`→`RiderIntent::state`+`encode_intent_state`, `intent_update`→`policy().update`, `intent_power_target`→`policy().power_target_w`, `intent_program_update`→`policy().program().update`, `intent_schedule_pulse`→`RiderIntent::schedule_pulse`, `intent_reset`→`reset`, `intent_restore`→`decode_intent_state`+`restore`. Decoders for replayed inputs: `decode_intent_config/signals/step/control/state`, `decode_intent_step` enforces the `type(step) is int` gate (rejects bool/float/string/`{"npint":N}` objects).

Consumer contract (verified against captured fixtures): entry order per node is `input.adapter.0` first, then per call one `input.<op>.k` doc followed by either `output.*` entries or an `error.<op>.k` empty-u1 marker — `peek_label()` picks the branch. Input docs are `parse_recording_json(decode_f64_text(f64_input))`; scalar args (`road_grade`,`preview_grade`,`lean_limit_rad`,`dt_s`,...) may be `Tagged{float64,value:"nan"|"+inf"|"-inf"}` — read via the codec's `real()`-equivalent (or replicate: Tagged→nonfinite, Number→lexeme, null→nullopt). `output.control`/`output.state`/`output.intent` compare `recording_json(encode_*(result))` bytes via `Node::bytes` ("|u1"); `output.intent` doc shape `{"posture":encode_rider_posture(...),"effort_ceiling_nm":Number}`; `output.power`/`output.program` are plain f64 scalars. Counters are per-node (`input.resolve.0`, `input.resolve.1`, ...), reset per test — mirror the Python `_SEQ` scheme. Both platforms captured; byte-flip proof documented above.

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

## Status for the next agent

**Fully qualified (darwin-arm64 + linux-aarch64):** `test_crank_split.py` (9), `test_leg_loop.py` (7), `test_joint_strength.py` (15), `test_planar_arms.py` (11), `test_native_rider_intent.py` (26 — consumer + goldens on both platforms, `python-removal` row now `replacement_landed`), plus the 14 oracle-green records of `test_native_spindle_controller.py` (20 nodes, replayed by `reference.native_spindle_controller` on both platforms).

**Still blocked (records stay `planned` with evidence):**
- `test_coupled_rider_task.py`: `test_planned_crank_torque_...` (delivered-torque parity) + `test_tissue_damping_does_not_eat_the_muscle_budget` (linux-aarch64 episode drift — oracle green on linux, native red).
- `test_closed_form_rider.py`: `test_flat_launch_delivers_crank_torque` (same parity family).
- `test_native_spindle_controller.py`: 6 enabled-compute records / 20 nodes (native rear-leg chain torque defect — described above) + 2 `_damped_config` records (TOMLDecodeError baseline).

**Still owed before batch completion:** the three defect fixes above (or an explicit waiver path), then `python-removal.json` flips for the affected files, and the final `bash tools/run_tests.sh native` gate — this session has NOT yet run it; it must exit 0.
