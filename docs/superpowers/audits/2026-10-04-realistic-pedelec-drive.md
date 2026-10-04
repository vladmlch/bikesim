# Realistic pedelec drive — implementation and acceptance evidence

Source baseline: `aadcba1`, branch `implementation/seated-plant-v2`.
Approved plan: `docs/superpowers/plans/2026-10-04-realistic-pedelec-drive.md`.
Python commands use `uv run`; all plant runs are sequential. Bosch data are
declared recalled/unverified; no web lookup was performed.

## Task 6 force-request failure

The prescribed ripple integration test failed after adding the waveform:
`test_force_requests_honour_the_ripple_argument` received zero forces for
both `ripple=0` and `ripple=.5` at phase zero. Focused result: 1 failed,
11 passed; the full fast run stopped at 1 failed, 157 passed, 34 deselected.
Both reported `RuntimeWarning: invalid value encountered in scalar multiply`
at the capacity calculation in `rider_control.py`.

At the horizontal driving pedal, normal lever arm is positive, tangential
lever arm and tangential/normal ratio are zero, and the compression upper
bound is infinity. Computing `ratio * compression_limit` generates NaN;
the zero tangential term then contaminates the positive infinite capacity.
`max(0, NaN)` silently chooses zero, so the useful pedal is discarded.

Ruling: compute unbounded-capacity cases explicitly: positive normal lever
means an unbounded *request* capacity; zero normal lever can only use bounded
tangential friction. Finite return-side bounds retain the existing formula.
This fixes the arithmetic without increasing a physical force/power budget.
Cost if wrong: pedaling requests near singular geometry need re-evaluation;
the independent allocator/support limits and physical acceptance still apply.

## Acceptance status

Pending. Component tests and topology builds do not qualify the assembled
plant, G5 energy gate, GUI, realtime performance, or vendor calibration.

## Task 6 verification and baseline comparison

After the capacity fix, focused policy/waveform tests: **18 passed** in .37s;
canonical non-slow: **293 passed, 34 deselected** in 1.14s, with no NaN warning.
The horizontal-stroke regression computes crank torque from the actual force
vectors for both pedals, including zero friction.

The strict preload test is **red**: `saddle.cop` at incoming interval time
`.0015 s` (first closing `.005 s` period), 1 failed/11 deselected in28.08s.
No support/energy gate or assertion was relaxed.

A disposable `git archive aadcba1` baseline with Python import path explicitly
asserted to be `/private/tmp/pedelec-baseline-src/src` reproduced the same
`saddle.cop` at `.0015 s` for the matching80rpm/20Nm wish **without preload**.
Baseline state:10steps, simulation time.005s, reason invalid_controller,
model_valid=false. Its source snapshot lacked the then-untrackedconsole.py
required for package import; current console.py was copied into the disposable
snapshot as UI plumbing only. No baseline physics file was substituted.
Command: `PYTHONPATH=/private/tmp/pedelec-baseline-src/src uv run python /tmp/pedelec-baseline-cycle.py 80`.
Evidence: `/private/tmp/pedelec-baseline-cycle-80/evidence/` and
`/tmp/pedelec-baseline80.log`; current failure `/tmp/pedelec-preload.log`.

Ruling: continue the authorized drive slice without G5 acceptance, as Q15
explicitly requires. Keep the strict preload test red and mark solved preload
compression pending. The reproduced startup failure is outside the drive
slice; declaring the preload physically accepted would be unsupported.
Cost if wrong: fixing G5 may reveal an additional preload/allocator failure.

## Task 7

Exactly12leg direction curves changed: vmax22rad/s, hill_c.35, provenance
marked engineering/unverified. Upper body, angle/torque knots, and verification
flags unchanged; compact formatting retained (12lines changed).
RED envelope test1failed/1passed, missing vmax. Full joint-strength module
GREEN15passed in20.41s; its diagnostic episode still warns energy.constraint_work.
The added120rpm case requests20Nm (other pre-existing cases retain30Nm).
Strict full-cycle case:1failed/12deselected in28.74s, saddle.cop at.0015s.
Disposable aadcba1 with same120rpm/20Nm also reports saddle.cop at.0015s.
Actual120rpm/no-saturation pedaling is therefore PENDING, not qualified by the
pure directional-capacity tests. No vmax25escalation: failure is a support gate,
not evidence of insufficient directional strength. Logs: /tmp/pedelec-120.log,
/tmp/pedelec-strength.log, /tmp/pedelec-baseline120.log.
