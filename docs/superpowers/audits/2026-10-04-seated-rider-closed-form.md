# Closed-form seated rider audit — NOT ACCEPTED

Plan dated 2026-10-04; final verification completed 2026-10-05. Plan: `docs/superpowers/plans/2026-10-04-seated-rider-closed-form.md`; specification: `docs/superpowers/specs/2026-10-03-v2-seated-plant.md`. Implementation branch: `implementation/seated-plant-v2`, starting source `3ce230e`; implementation through Task12: `92b124e`.

The default plant now has nine rider hinges, locked ankles and saddle/spindle/bar point connects. The pinned controller uses closed-chain virtual work, directional strength curves, active power clipping and pose PD with model bias compensation. This is implemented component behavior, not physical qualification: road, strict-cycle and realtime gates remain red. Legacy removal Task11 is **OPEN**, because the explicit condition “after acceptance” has not been met.

## Source and component evidence

| Component | Evidence |
|---|---|
| Asphalt savage, original obstacles/extreme preserved | `test_savage_track_surface.py`; 120m/asphalt/no loose sections |
| Configured attachment budget; grip weld alias normalized to connect | `test_articulated_config.py`, `test_attachment_budget.py`; every runtime interval receives its configured budget |
| Nine joints, no ankle actuators, five point connects, unlimited pure motors | `test_pinned_topology.py`; joint profiles accept canonical ankle supersets and reject missing present joints |
| Solved foot forces/torque, point-only moments, linkage closure exclusions | `test_pinned_measurement.py`, `test_attachment_wrench.py`; native Jᵀλ equals paired `mj_applyFT`, asymmetric foot loads give nonzero sensor torque |
| Soft-connect measurement at each body's own anchor | Nonzero1mm-gap regression; explanation, Newton-pair and planar checks retained |
| Leg IK/build offsets/frame pitch, directional torque split/preload/power | `test_leg_loop.py`, `test_crank_split.py`; native pose/phase checks and settled IK check |
| Delayed front-load trim, preview endpoint, surge and geometric ROM bound | Posture/surge/lean tests; native runtime producer proves load share reaches intent while external sensor map stays unchanged |
| Closed-form path; strength/power/activation/probe behavior | `test_closed_form_rider.py`, `test_period_buffer.py`; every held interval independently checks strength, 250W/joint, 450W total and 20rad/s |
| Balance latch/first-low dwell/braking/reset; summary and research fields | `test_balance_monitor.py`; event time/position remains latched and does not itself stop integration |
| Arm effort, geometric lean limit and balance HUD | `test_hud_columns.py`; formatter/colors tested, GUI not launched |
| Actual-crank coast EMA; wheel-informed mash preserved | Root commit `be56ccd`; three narrow regressions, original coast/shifter deadlock repaired |
| Exact profile-led performance changes | `test_realtime_deferred_checks.py`;200-step eager/deferred native oracle,200 scalar preview windows, and individual non-flat200-step published-state/force/channel/gate oracles for each additional optimization, rtol/atol1e-12 |

Torque curves and Bosch numbers remain recalled/provisional, `verified=false` / `mechanically_checked` as applicable. No internet verification was performed. S5 still literally names a snapshot-only executor signature; the approved Task6 interface retains `compute(model,data)` for permitted kinematics/transforms/model `qfrc_bias`. No solved-force/telemetry reads were added to the pinned controller. Capability isolation by a snapshot-only signature is **not claimed**; this wording/interface mismatch remains explicit.

Native torso sign evidence corrected the plan's reaction sign: at qpos0 torso torques -1/0/+1Nm give pelvis accelerations +8.718597/-0.014405/-8.747408rad/s². Positive torso torque restores positive pelvis pitch. Foot compression is local **+z** with the authored ankle-above-spindle geometry; native action/Jacobian identity proves the sign. Downward preload contributes **+preload×spindle_x** about native +y; rising-leg torque is negative. These are evidence-led convention corrections, not gate changes.

## Road acceptance

Every physical interval is preserved in `interval_rows.csv`; control rows remain100Hz for the original20-row shift windows. Timestamped interval errors prevent a clean period endpoint from hiding an earlier rejected step. Counts below filter each interval's own incoming time >0.5s. Multiple reasons can occur in one invalid interval. Diagnostic recording decimation is1, unlike the realtime probe's80; decimated snapshots cannot certify intermediate-step power/strength.

| Ride | Max km/h | First25km/h s |25%plateau mean km/h | x_max m | Balance events | Invalid intervals after0.5s | Gate violations after0.5s | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| Flat12s |25.61667 |9.50 |— |60.11678 |0 |6455 |22073 |Speed requirement passes; cadence fraction0.760234<0.8 and physical gates fail |
| Shift12s |25.61667 |9.50 |— |60.11678 |0 |6455 |22073 |10shifts; first measured motor/human window0.09373<2.5 |
| Synthetic15%,25s |14.36827 |— |— |88.02854 |0 |13270 |30394 |Late minimum12.15393km/h passes12; physical gates fail |
| Savage |16.03661 |— |5.64277 |58.31396 |1 |20997 |58563 |Crashes `loop_out` at31.17875s;120m and6km/h plateau criteria fail |

Savage's event latches at x=50.06613m. Its final position58.10276m differs from the recorded maximum58.31396m because the bike rolls back before termination. The45%wall is not reached; no inference about passing that wall is justified.

| Ride | Peak actual positive rider power W | Absolute constraint work / positive sources |
|---|---:|---:|
| Flat/shift |1004.61531 |2.396512% |
|15% |967.53516 |1.164545% |
| Savage |1570.41957 |2.135590% |

These exceed450W /1%. Incoming commands clipped at200Hz are not a certificate for the four subsequent held physical intervals: changing velocity/angle can exceed strength/power before the next command tick. Support normal/friction failures are genuine post-hoc findings; requested virtual-work moment/preload is not proof of solved compression.

The first red shift window has sensor≤4Nm in72.727% of physical intervals and cadence≥180rpm in11.039%; no manual braking, backward rotation or coasting occurred in that window. These close legitimate Bosch permission/envelope gates. The inherited mean-window oracle omits instantaneous gate eligibility, so the red ratio alone does **not** establish an extra shift-specific motor cut. Threshold2.5 and the requirement for at least five shifts were retained.

Persistent acceptance evidence: `.superpowers/sdd/2026-10-04-seated-rider-closed-form/task9-{flat,shift,fifteen,savage}/test_*/out/`; summaries/complete violation counts: `task9-summary.json`; shift gating: `task9-shift-gating.json`. All four original runs report `source_changed_during_run=false` at source `be56ccd`. The required final reruns at clean source `92b124e` reproduced every road number and interval count in these tables, with unchanged source during each run. Root independently streamed their CSV and summary artifacts into `root-final-acceptance-evidence.json`; complete final artifacts are under `final-checks/test_realistic_pedelec_acceptance/` in the recovery ledger. Code-only performance changes also passed the per-change 200-step numerical/gate oracles.

## Strict cycles

Strict motor-off20/40/60Nm and free-coast cases all stop at their first closing period:0.005s, four captured intervals, `energy.constraint_work`. Absolute work0.004478444247J / positive source work0.027149520780J = **16.495481756%**, against1%. Their initial slew is identical, explaining identical first-period evidence. Requested8s cycles, ≥80%torque delivery and three revolutions remain unqualified; no diagnostic bypass or startup waiver was added.

Evidence: `task12-strict/test_*/out/{summary.json,strict_failure.json,intervals.jsonl}` and `task12-summary.json` in the same recovery ledger. Legacy flat/default preload cycles were replaced by the plan's stricter pinned cases; standalone legacy implementations/tests remain.

## Realtime

Both final probes run all16000steps/20s with dt1.25ms and controller period5ms. Accounted headless timing includes final flush and excludes separately reported startup. It does not measure GUI rendering. Source fingerprint, git SHA, machine, dirty flag and independent model/numerical validity are stored in each report.

| Track | Before factor | First optimized factor | Final clean-source factor | Final accounted wall s | Sim s / steps | Model valid |
|---|---:|---:|---:|---:|---:|---|
| Savage |0.47623 |0.58882 |0.41084 |48.68132 |20 /16000 |false |
| Extreme |0.50000 |0.58766 |0.53025 |37.71816 |20 /16000 |false |

Machine: `macOS-27.0.1-arm64-arm-64bit-Mach-O`. The first optimization reports name git SHA `1a75443` and `working_tree_dirty=true`; their exact source hash is retained. Optimizations are committed in `795f56c`. The required final repeats name `92b124e`, `working_tree_dirty=false`, source SHA256 `4bf2ce19c4f6c165b6e573def3b5b60bdac8ce5f3fdf1a1244f03b474dcbd570`. Timing varied between runs; neither series meets ≥1×. No timestep, solver, control frequency, gate or physical step was changed or skipped. Profiler-overhead factor 0.28636 is not a qualification number. Evidence: `task10-final/test_realtime_factor_for_full_{0,1}/realtime.json`, `final-checks/test_realtime_gate/test_realtime_factor_for_full_{0,1}/realtime.json`, `task10-summary.json`, `task10-profile.prof`, and individual references in `task10-oracles/`.

An independent review rejected a raw λ-only shortcut because it would make force explanation and Newton checks tautological. A topology-certified analytic wrench solve was discussed, but not adopted: it changes recovery arithmetic, needs additional corruption/boundary parity proofs, and does not guarantee ≥1×. Current optimizations retain the original inverse-wrench checks.

## Verification and remaining scope

Final fast suite: **398 passed, 49 deselected**; root independently reran `uv run python -m pytest tests -m 'not slow' -q` after the audit edits (8.79 s, no warnings). Required selected slow files ran sequentially at `92b124e`: **3 passed, 11 failed, 19 deselected**. Source/tree checks do not substitute for physical acceptance. The complete repository slow suite and GUI have not been claimed green.

| Final slow file | Passed | Failed | Deselected |
|---|---:|---:|---:|
| `test_closed_form_rider.py` |1 |1 |9 |
| `test_pinned_topology.py` |1 |0 |1 |
| `test_pinned_measurement.py` |1 |0 |3 |
| `test_seated_pedaling_cycle.py` |0 |4 |6 |
| `test_realistic_pedelec_acceptance.py` |0 |4 |0 |
| `test_realtime_gate.py` |0 |2 |0 |

Results/logs: `.superpowers/sdd/2026-10-04-seated-rider-closed-form/final-checks/results.json` and each named log/directory. The final launch still rejects motor engagement 0.5875 against 0.8. The measurement test passes its force/sensor assertions while emitting the existing energy warning; this is measurement verification, not acceptance.

Independent `gpt-6.1-sol/high` review of `3ce230e..92b124e` found no new Important/Critical implementation defects. It checked deferred and partial-period validation, snapshot ownership, all-interval acceptance capture, strict failure persistence, balance publication and compatibility. It did not repeat plant runs or claim the red physical/realtime gates passed. Two agents were used in total; all Python ran through `uv` and all plant runs were sequential.

Task11 conditional deletion remains **OPEN**: no acceptance was achieved, so SLSQP allocator/clutch/config fields and standalone legacy tests were preserved. Only obsolete readers of the default topology/damping were updated to actual connects/nine joints/zero passive damping. Original R4/R7 paths are superseded by this slice; their historical evidence is preserved. User-owned originating/current plans remain untracked after narrow annotations/checklists.

R8 `ease_pedals` and `rear_brake_tap` are explicitly outside this slice; no anti-wheelie policy or estimator was developed. The posture lean path is implemented, but savage's event/crash is not hidden by an added reflex. Offline bundle, wheel/source parity, Linux rebuild and physiological/strength-data validation are excluded. Physical support/work/held-interval limits, motor engagement and full≥1×realtime remain unresolved release blockers.
