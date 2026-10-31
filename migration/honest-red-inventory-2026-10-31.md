# Honest-red inventory + Task-3.09 ruling proposal — 2026-10-31

Investigation at `d20ea74c` (read-only agent). Ledger: 593 `planned` records
total; the honest-red blocked subset is **36 records in 8 families**. All
citations verified at that commit.

## Defect families

| Family | Records | Signature | Root-cause hypothesis |
|---|---|---|---|
| R1 runtime_step/commands bitwise parity | 10 | First printed diff ~1 ulp in MuJoCo integration vector; identical darwin+linux; unchanged pre/post-F4 | Staged `mjData` inputs (`qfrc_applied`/`xfrc_applied`/`ctrl`/`act`/warmstart) — same engine image both sides, oracle pinned to `*_py` twins via `_pin_oracle_kernels` → true arithmetic divergence in `step.cpp::apply_forces_body` staging chain |
| R2 runtime_rollout weld parity | 4 | `sensors.frame_specific_force_body_mps2[0]` diverges from interval 0; 800-interval flips `rider_torso_hinge` (categorical, not noise) | Same FT-1 staged-input family; sensor read is downstream observable |
| R3 research welded live-oracle | 10 | All external-transition/recorder-row/episode-metric compares on `viewer_physics_welded.toml` drift from interval 0 | FT-1 downstream; `NativeResearchRuntime` clean where parity holds |
| R4 research adapter seams | 2 | Tests monkeypatch `bike_sim.native.research._transition`/`rider_behavior` — no native hook exists | Contract decision, not a physics defect (FT-2): implement seams or `drop` with coverage note |
| R5 closed-form delivered torque | 2 | flat_launch engagement 69.6% vs oracle 87.8%; tissue_damping linux-only `passive_power` drift in `solved_effort` | F5-family port defect (FT-3): delivered/sensed-torque bisect through dead spots + shift windows; linux leg-chain ledger |
| R6 realistic pedelec | 4 | See ruling §2 — stacked oracle-baseline defect + native-only gaps (538 late violations vs oracle 0; darwin savage abort on contact normal) | FT-1 family + Task-3.09 ruling |
| R7 pneumatic tire | 2 | `event_rich`: 64k-step equilibrium non-convergence (oracle-side freeze family); `probe_and_clock`: probe-vs-live step-1 bitwise divergence | FT-5 freeze non-convergence; FT-6 probe staging |
| R8 viewer snapshot cross-oracle | 2 | Ownership/pacing halves green; parity legs planned pending goldens | Almost certainly FT-1 downstream — recheck after FT-1 |

Excluded (already dispositioned): platform_replay ×6 (port, lane/d green),
realtime_gate ×2 (F3 re-aimed), seated pedaling/posture ×5 (F1 resolved),
joint_strength/rider_allocation/drivetrain/spindle/planar_arms/period_buffer/
rider_welds (golden/port).

## Task-3.09 pedelec ruling proposal

Gap = two stacked layers:

- **Layer A (oracle baseline, Task-3.09 stop rule):** sensed crank torque
  collapses ~50→0.2 N·m during shift unloads against held ~6-7 N·m command
  (delivery regression in the closed-form leg law); monitor contract gap
  (per-raw-interval evaluation of per-tick held commands); savage capability
  bound (stall x≈53 m on 25% plateau vs 6 km/h bar).
- **Layer B (native-only parity gaps):** 538 late `rider_strength.*` violations
  vs oracle 0; shift motor-ratio 0.230 vs 0.094; engagement 69.6% vs 87.8%;
  darwin savage abort `attachment normal has no planar component` (preceding
  trajectory/contact divergence).

Recommendation: **Option 1 — approve the rider-plant delivery fix** as the
disposition for all four nodes, with explicit measured fallback to Option 2
(threshold/scope re-scoping) only for any sub-bar the fix measurably cannot
reach. Reject Option 3 (drop): the dominant cause is a genuine defect, and
honest two-platform qualification rides the same fix surface. Fallback needs
written physical argument + user approval per the stop rule.

## Fix tasks

| Task | Covers | Class |
|---|---|---|
| **FT-1** bitwise staged-input bisect at interval 0 (weld plant) | R1+R2+R3+R8 + R6 native gaps | Native compute defect — highest leverage (24/36) |
| **FT-2** adapter-seam contract decision | R4 | implement seams or drop w/ coverage |
| **FT-3** delivered/sensed-torque bisect + linux passive-power ledger | R5 | F5-family port defect |
| **FT-4** Task-3.09 ruling execution | R6 | Option-1 fix + measured Option-2 fallback |
| **FT-5** freeze/equilibrium non-convergence (committed physics.toml; pneumatic event_rich; pinned_savage_40) | R7 partial + golden_episode_tool | oracle-side baseline/config |
| **FT-6** probe-vs-live step-1 bitwise bisect | `probe_and_clock` | probe staging defect |

Note: port-map `line` fields are stale for pneumatic records (ledger
L372/L832 vs current L558/L1052) — candidate for a ledger re-sync pass.
