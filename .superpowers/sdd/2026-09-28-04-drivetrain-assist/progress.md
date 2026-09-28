# SDD ledger — plan: docs/superpowers/plans/2026-09-28-04-drivetrain-assist.md

## Target and preflight

- Phase follows A1-A6, B1-B2, and C1/C6; implementation has not started.
- Important source drift: current branch already has `crank_joint`, `crank_spin`, rotating `pedal_front`/`pedal_rear`, pedal sites, and a crank-to-wheel gear equality. The recent `physics/drivetrain.py` also defines `DrivetrainSpecs`, 12-speed cassette options, shifting, torque ripple, and assist settings. The old baseline references in the plan do not describe current HEAD.
- Ruling: extend the existing optional pedalized path for physical mode; do not build duplicate cranks/pedals or add a second competing `DrivetrainSpecs`. Add the missing cassette state and physical chain/freehub paths while retaining legacy/ideal-speed behavior. Keep the active physical gear fixed during a run as required by the spec. Cost if wrong: a second drivetrain authority or duplicated mass/actuation would invalidate physical and legacy parity.

## Pairwise dependency/conflict scan

| Tasks | Shared file/interface | Finding and ruling |
|---|---|---|
| D1 / D2 | crank/cassette names, `physics/chain.py`, `drivetrain_forces.py` | D1 creates the bodies/config; D2 derives a full Jacobian from those exact names. |
| D1 / B1-B2 | crank, cassette, wheel mass/inertia | Existing crank/pedal bodies already have masses; D1 must reassign, not duplicate, budgets. B2 owns wheel tensor; cassette mass transfer must preserve wheel+cassette total. |
| D1 / current `physics/drivetrain.py` | `DrivetrainSpecs`, active ratio, cassette | Plan proposes a conflicting class/default. Ruling: one existing public drivetrain config, extended/re-exported as needed; physical run locks selected gear. |
| D2 / D3 | chain pull and cassette-wheel coupling | Chain tension applies to cassette; freehub torque is a separate equal/opposite wheel/cassette pair. Do not apply either torque twice. |
| D2 / A2 | generalized-force accumulation | Chain contributes `-T*J` once under a stable component name. |
| D3 / D5 | stored coupling energy and release | D5/F1 energy ledger needs freehub storage and loss on disengagement; expose the decrease as a release/dissipation event rather than silently dropping it. |
| D4 / E4 | human crank torque and articulated rider force | `crank_effort` owns a crank actuator; `articulated_effort` measures foot/pedal work and forbids the human crank actuator. |
| D4 / D5 | motor command and energy budget | Apply torque caps and battery budget before the same-step actuator command; telemetry reads that same sample. |
| D4 / D6 | brake priority | Brake disables motor and ideal-speed torque immediately; default command cancels human target while explicit research input remains identifiable. |
| D1 / D6 | wheel joint names/DOF addresses | Static brake resolves joints by name after new bodies alter DOF order. |

## Task self-consistency scan

| Task | Result |
|---|---|
| D1 | The “no dynamic crank/pedals yet” premise is false on current HEAD, while cassette is still integrated into the rear wheel. Extend current topology and preserve component masses/legacy equality. |
| D2 | Tangent-length/Jacobian equations match DRIVE-02. Keep unwrap state fixed during finite differences and test world/parent motion; require physical model `nq==nv` only if verified at this topology. |
| D3 | One-way spring coupling and equal/opposite torque are specified. Boundary retreat can drop stored energy; publish that drop to the energy ledger. |
| D4 | Synthetic defaults match DRIVE-05. Add an explicit nonnegative human-command contract and preserve a configurable cadence torque table; avoid unbounded torque at zero cadence. |
| D5 | Battery is in joules and torque limit precedes application. If idle loss exceeds the budget, do not consume the remaining energy while reporting the motor off. |
| D6 | `dof_frictionloss` is the static constraint path; identify only friction rows for brake work and cancel positive drive on the current step. Do not attribute tire/contact constraint work to the brake. |

## Recorded rulings

- Ruling: reconcile D1 with current source by extending `physics/drivetrain.DrivetrainSpecs` or a single owned subconfig and reusing the already-built crank/pedals. — Current HEAD implements the feature the plan says is absent. — Cost if wrong: conflicting APIs, topology and mass ownership.
- Ruling: add a freehub release-loss event when stored energy is discarded. — SYS-04 forbids untracked energy disappearance. — Cost if wrong: energy residual cannot distinguish implementation loss from physical dissipation.
- Ruling: validate brake work from frictionloss rows only and use relative wheel speed. — DRIVE-07 excludes contact forces from brake torque. — Cost if wrong: reported brake power includes road contact.
- Active workstream `#1153`: initialize the selected crank phase and matching pedal/rider interfaces before static equilibrium, not only after equilibrium has been solved.

## Task ledger

- [ ] D1: pending Phase 03 gate and current-topology audit
- [ ] D2: pending D1
- [ ] D3: pending D2
- [ ] D4: pending D1-D3
- [ ] D5: pending D4
- [ ] D6: pending D4
- [ ] Gate D review: pending
