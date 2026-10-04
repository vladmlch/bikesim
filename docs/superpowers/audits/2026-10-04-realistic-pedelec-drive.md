# Realistic pedelec drive: implementation and acceptance

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

Final fast suite: **316 passed, 41 deselected**, in 1.33 s. Selected slow
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
