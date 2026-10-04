# Realistic pedelec drive: implementation and acceptance


Продолжение: [closed-form seated rider audit](2026-10-04-seated-rider-closed-form.md), 2026-10-04. Новый pin/connect профиль и приёмка 120 м supersede старую allocator+saddle-contact траекторию; физическая приёмка остаётся открытой до зелёных гейтов.

Plan: `docs/superpowers/plans/2026-10-04-realistic-pedelec-drive.md`.
Branch: `implementation/seated-plant-v2`; initial source: `aadcba1`.
All Python commands use `uv run`. Plant runs were sequential. Bosch values
remain recalled, unverified data; no web lookup was performed.

**The slice is not physically accepted.** Component contracts have been
implemented and tested. Solved rider torque, strict support/energy checks and
road criteria remain red. Diagnostic rides preserve `model_valid=false`.

## Component verification

| Change | Verification |
|---|---|
| Motor profile and permission | 32 tests; immutable gains, finite data, modes, binary gate |
| Assist controller | 59 profile/controller tests; lag, immediate gate/reset, external ceiling |
| Rotor topology | 8 unit/config tests and 3 slow builds; default, rotor and legacy clutch |
| Reposition removal | 4 policy tests; no remaining production/example references |
| Shifter | 9 tests; crank decision, landing/safety gates, no additional motor cut |
| EMA, ripple, force requests | 29 tests after the net preload correction |
| Leg strength curves | Full module: 15 passed, with a diagnostic energy warning |
| Welded default config | 3 tests; requested topology and numbers |
| Airborne momentum consumer | 2 slow tests; actual motor work, passive zero work, original conservation limits |

Final fast suite: **316 passed, 41 deselected**, in 1.52 s. Selected slow
group: **3 passed, 11 failed, 14 deselected**, 4 warnings, in 698.68 s.
The slow group ran at physical source `09b04cd`; later replay-only refusal
changes do not alter the tested plant. No passed physical acceptance is claimed.
The fast command is `uv run python -m pytest tests -m 'not slow' -q -x`.

## Physical failures and source attribution

Strict preload and 120 rpm cycle tests stop at `saddle.cop` at incoming time
0.0015 s, in the first closing 0.005 s period. No support or energy gate was
relaxed. Matching 80/120 rpm, 20 Nm requests on a disposable `git archive aadcba1` snapshot reproduced the same first failure without the new preload.
The snapshot's previously untracked `console.py` was required for package
import, so current UI plumbing was copied into the disposable snapshot.
No baseline physics module was substituted; the import path was asserted.

The new default's torque sensor has correct one-step identity (<1e-9 Nm), but
its load threshold fails:

| 400-step diagnostic | Late mean solved crank torque |
|---|---:|
| Baseline `aadcba1` | 16.505119 Nm |
| Approved new default before net compensation | 6.903736 Nm |
| After net compensation | 5.815129 Nm |
| Required by the unchanged smoke test | >10 Nm |

Both baseline and new diagnostic had all 200 late samples in `pedaling`.
Coasting does not explain the loss. Baseline late command was 51.38–54.27 Nm;
new default command was 51.67–54.00 Nm. This is an unresolved new delivery
regression, not a sensor-delay defect or a baseline-only failure.

The first 20 s flat ride, before net compensation, reached **2.95118 km/h**
against a 20 km/h criterion. Mean sensor torque was 1.98598 Nm despite a
57.3535 Nm command; mean motor torque was 1.03578 Nm. Positive sensor fraction
was 0.757; motor engagement fraction 0.1735; no shifts occurred from 51T.
All 2,000 rows reported `allocation_invalid=false`, while the model was invalid
throughout. Recorded failures include support normal/friction/COP budgets,
unobservable attachment wrenches, constraint work, joint strength and positive
power. A recorded 459.697 W muscle-power sample exceeds the 450 W limit and
remains marked invalid.

## Corrected request and validation defects

1. At a horizontal power stroke, `0 * infinity` produced NaN in the existing
   force-request capacity. This discarded the useful pedal and requested zero
   force. Explicit unbounded/finite capacity cases fix the arithmetic without
   changing a physical budget. Force-derived regressions cover both pedals,
   including zero friction.
2. The original preload snippet overwrote return-leg force after distributing
   the net waveform. At phase zero, mean 40 Nm and ripple 0.5, a declared
   60 Nm target became 53 Nm after the return leg's -7 Nm moment. Preload and
   its actual phase/normal signed moment now enter the pure force helper before
   active-foot allocation. Recovering feet can be excluded; infeasible geometry
   cannot request negative active drive. All allocator and power budgets stay
   unchanged. This fixes intent, **not solved delivery**.
3. The active airborne momentum stand used gain=0, sensor=0 and rider intent=0,
   so the new gate silently turned it into a passive stand. It now declares
   synthetic 10 Nm intent and sensor input. Its independent joint controller
   still requests zero pedal effort; no rider force or production gate bypass
   was added. Active work must be positive and passive work zero.

## Rulings and remaining gates

- Work in the named checkout and use two agents in total, both
  `gpt-6.1-sol/high`: one motor/topology implementer, one reused reviewer.
  Cost: less per-task context isolation; stable commit groups were reviewed.
- Amend release W3 now so it cannot restore `motor_clutch=true` later.
  Cost: its handoff text needs reconciliation with later release work.
- Correct the standalone freewheel oracle: a loaded soft tendon has finite
  penetration, not necessarily backlash. The original 1e-6 rad assertion under
  30 Nm saw 8.8288e-5 rad penetration with co-rotation and 26.08696 Nm transfer.
  Tests retain loaded transfer, ratchet tracking, unloaded boundary agreement
  and prompt re-engagement. Production solver settings/boundaries were unchanged.
  Cost: the corrected unit oracle still does not qualify engagement energy.
- Disable obsolete reposition in topology-only fixtures until its removal;
  make their raw 1 Nm/profile 85 Nm conflict independent of the default TOML.
  Cost: fixtures isolate topology rather than validate the complete default.
- Continue without G5 acceptance as explicitly authorized by Q15; retain red
  preload/cycle gates. Cost: a repaired plant may expose additional drive issues.
- Keep the specified vmax=22/hill_c=0.35. The 120 rpm failure is a support gate,
  not proof that strength needs another increase. Cost: actual cadence capacity
  is still unverified.
- Preserve the configured net torque when accounting for recovery preload.
  Cost: the stronger positive-leg wish still needs physically valid allocation;
  measured delivery and rides have not been made green by this change.
- Adapt necessary validation consumers instead of silently qualifying zero
  motor work. Cost: artificial momentum input qualifies internal momentum only.

D1 assist source/unit work is implemented; D1 sensors, D2 and D4 remain outside
this slice. D3' unit/topology work is implemented; detailed engagement-energy
loss is pending. W3 reposition removal is implemented; initializer/release work
is pending. R9 source/curve work is implemented; solved compression, full 120
rpm cycling and road acceptance are pending. G5 constraint-work attribution
must be rerun on the rigid topology before W6. Legacy clutch remains for A/B.
No full physical qualification, GUI, realtime factor or vendor calibration is
claimed.

## Evidence

- Motor/topology handoff: `.superpowers/sdd/2026-10-04-realistic-pedelec-drive/motor-report.md`.
- Recovery ledger: `.superpowers/sdd/2026-10-04-realistic-pedelec-drive/progress.md`.
- Strict failures: `/tmp/pedelec-preload.log`, `/tmp/pedelec-120.log`.
- Baseline comparison: `/tmp/pedelec-baseline80.log`, `/tmp/pedelec-baseline120.log`;
  `/private/tmp/pedelec-baseline-cycle-{80,120}/evidence/`.
- Strength: `/tmp/pedelec-strength.log`.
- Sensor: `/tmp/pedelec-sensor.log`, `/tmp/pedelec-sensor-compensated.log`,
  `/tmp/pedelec-sensor-{diag,baseline}.log` and
  `/private/tmp/pedelec-sensor-{current,baseline}/sensor.csv`.
- Initial flat: `/tmp/pedelec-flat.log` and
  `/private/tmp/pedelec-flat-n2qcwygw/test_flat_start_reaches_20_kmh0/out/`.
- Final selected slow group: `/tmp/pedelec-final-slow.log`; complete artifacts
  `/private/tmp/pedelec-final-sjkdsx7s/`.


## Final road and cycle acceptance

All four road tests ran sequentially on `09b04cd`; their saved summaries report
`source_changed_during_run=false`, `model_valid=false` and no passed qualification.

| Scenario | Actual failure |
|---|---|
| Flat 0 to 20 km/h | Maximum 2.981850 km/h; zero shifts; 51T retained |
| Motor through a shift | Zero shifts against at least five required |
| Synthetic 15% climb | Late minimum -0.469034 km/h; maximum x=12.5531 m, before the full 15% plateau at 14 m |
| Savage 15% plateau | Maximum x=12.5926 m; the checked 17–33 m plateau was not reached |

All seven strict cycle tests failed. The first failures were:

| Cycle | First rejected incoming time | Reason |
|---|---:|---|
| 40 rpm | .0025 s | foot_front:unobservable_attachment_wrench |
| 80 rpm | .0010 s | saddle.cop |
| 110 rpm | .0020 s | foot_front:unobservable_attachment_wrench |
| 120 rpm | .0020 s | foot_front:unobservable_attachment_wrench |
| Flat pedals | .0050 s | energy.constraint_work |
| Free coast | .0050 s | energy.constraint_work |
| Recovery preload | .0010 s | saddle.cop |

Three topology builds passed. The full selected group therefore returned
11 failed / 3 passed; failures are retained without loosening criteria.

## Final review consumer correction

The reviewer found another old motor-only path: `open_loop_schedule` explicitly
commanded zero rider effort, making its advertised motor ramp always motor-off
under the new gate. It is now explicitly unsupported. Replay rejects before
building, retains the requested mode, writes an incomplete/error report and
returns nonzero CLI status. Fidelity rejects before reading/building fixtures;
its series/comparisons cannot report a passed motor ramp. The factor matrix
retains a visibly failed unsupported entry. No substitute rider experiment or
production gate bypass was introduced.

Refusal regressions: 8 passed. The test's initial sweep axes were corrected to
its existing three-distinct-values contract; production convergence requirements
were retained. Necessary consumer ruling: reject an unrepresentable motor-only
experiment rather than quietly qualify zero actuation. Cost: that old experiment
requires a separately declared motor stand to be available again.

Tasks 9 and 10 have executed their checks and recorded evidence; physical
acceptance and slice closure remain **OPEN**. A failing mean-torque smoke is a
new delivery blocker, while baseline COP/energy defects remain G5 prerequisites.
No single physical knob has been proved responsible for the whole delivery loss.

A final source check rules out one narrower measurement hypothesis:
`WeldedPedals.delivered_crank_torque_nm` projects the actual selected pedal
equality multipliers through the native Jacobian transpose onto `crank_spin`.
It does not substitute zero when an individual attachment wrench is
unobservable. The low measured torque is therefore not that proposed fallback;
the underlying delivery/support problem remains unresolved.

## Independent verification and delivery investigation (2026-10-04, second session)

Verified: 13 commits `3385623..09c6ac4` implement Tasks 0–10 at component
level; non-slow suite 316 passed (one test needs `python -m pytest` for the
`tools` import); no pre-existing threshold loosened; no removed-knob leftovers.
Flat acceptance reproduced on the default welded profile: **2.98 km/h**, zero
shifts. Slice remains **NOT ACCEPTED**.

### Where the rider's power goes (default profile, flat launch, t > 2 s)

| Quantity | Value |
|---|---:|
| Net muscle power (11 actuators) | 161 W |
| Dissipated in rider hinge damping (`joint_kd_nms_rad` = 15 compiled as DOF damping) | 158 W |
| Delivered to the crank (sensor × crank rate) | 3–7 W |
| Mean solved weld crank torque vs command | 1.3 vs 58 N·m |

Pedal forces are large (≈185 N mean each, peaks 700 N) but uncorrelated
with the request (corr −0.03); the crank lags the wheel 65 % of the time
(freehub open), so the pedelec gate correctly sees no rider torque and the
motor stays off. The old boost/stall motor had masked this.

### Ablation (6 s launches, one knob each; sensor mean in N·m)

reference 1.7 · legacy clutch 1.7 · ripple .35/no preload 2.1 · no preload 2.0
· baseline strength curves 1.5 · 20 N·m effort 2.0 · damping 1.5 → 0.6 ·
damping 3 → 2.0. Neither the drive topology nor any Task 6–8 parameter is the
cause; the loss is in the rider controller/plant.

### Mechanism (probes `/tmp/pedelec_alloc_probe.py`, `/tmp/pedelec_binding_probe.py`)

1. The leg command is `PD(IK targets) + Jᵀ·F_request`; in the welded
   pelvis–leg–pedal–crank chain a Jacobian-transpose feedforward assumes a
   free foot and saturates the ±100 N·m actuators (intents 400–1000 N·m)
   instead of turning the crank. The allocator's nearest-intent objective is
   then dominated by unreachable wishes.
2. Welded pedal platforms spin freely; the ankle target "level foot" with
   kp = 600 produces ~900 N·m ankle intents once the platform pitches, and the
   hip/knee IK (aimed at a level-foot ankle position) fights the chain.
3. With hands welded, the torso target is unreachable statically: the trunk
   collapsed to its ROM stop in the initial equilibrium (torso q 0.567 at the
   0.567 limit, elbows at 105°), demanding −300 N·m forever; the trunk
   reaction then binds the saddle friction/COP cone, which the binding probe
   shows as the family that makes a 30–60 N·m crank task infeasible
   (`saddle_mu` alone unlocks it).
4. At the control-period level the planned first-step weld torque is exact
   (|planned − solved| = 0.00, corr 1.000) but drifts within the 5–10 ms
   period; the pelvis bounces on the saddle weld (0–1900 N) under pedalling.

### Implemented, opt-in (`[articulated] coupled_task_control = true`), default OFF

- `rider_response_allocation.allocate_response(..., crank_torque_nm)`: the
  predicted weld torque on `crank_spin` (same multipliers and Jᵀ row as the
  sensor) is one more heavily weighted coordinate (`CRANK_TASK_SCALE_NM` = 1
  N·m ≡ 50 N·m joint deviation); `linear_region_infeasible` honours an
  equality block; diagnostics `crank_torque_target_nm`,
  `solution_crank_torque_nm`.
- `rider_control`: for the fully coupled rider the crank intent (pedalling
  waveform or coasting brake) is the task; pedal/support Jᵀ feedforwards are
  dropped; the hands carry the trunk's gravity moment on the bar; arm IK is
  solved about the shoulder the *target* torso places (stiff arms).
- `ArticulatedConfig.joint_passive_damping_nms_rad` (None = legacy coupling to
  `joint_kd_nms_rad`): tissue damping compiled into the model and the energy
  ledger apart from the controller gain. Not changed in any profile.
- Tests: `tests/reference/test_coupled_rider_task.py` (planned = solved
  first-step weld torque; damping < 30 % of positive power with 0.5 N·m·s/rad;
  config contracts). Default profile tests unchanged: held rider 4 passed,
  sensor smoke still 5.815 < 10 N·m (pre-existing).

Measured with the mode on (damping 0.5): launch reaches 5–7 km/h within 1 s
(vs 1.3 before), then the trunk still migrates to its ROM stop, the saddle
cone binds, the task is dropped 12 % of ticks, 20 s flat end speed 6.3 km/h.
An ankle variant (hip/knee IK at the actual ankle anchor, bounded ankle wish)
removed the 900 N·m intents but left the free pedal platform without a static
attractor: the initial equilibrium stopped converging (residual 0.73 at
`rider_ankle_rear`/`pedal_rear_spin`), so it was reverted.

### Ruling and next slice

"Pedalling like reality" is blocked by the rider plant, not by the drive:
(1) trunk posture/arm support in the doubly welded chain, (2) the welded
pelvis COP/friction budget for a planar two-leg rider, (3) foot/pedal pitch
control through the pedal pressure centre, (4) neuromuscular smoothing
(`activation_tau_s` = 0 today). These are R/G5 plant items (spec S4/S5), to be
done as one slice with the saddle-contact model, after which the four
acceptance rides re-run unchanged. No threshold, budget or track was changed.
