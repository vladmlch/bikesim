# Joint strength evidence — `examples/research/rider_strength_reference.json`

## Status: PROVISIONAL — every curve is `verified: false`

This profile is an engineering working sheet for the R3 effort limiter. It is
**not** a biomechanically validated human strength model. Loading it with
`require_verified=True` fails by construction; that gate, not this file,
decides when the numbers are release-acceptable.

## What each field means

- `coordinate`: the q→anatomical convention (`neutral_anatomical_rad`,
  `direction`), identical to `rider_joint_envelope_anatomical.json`. The
  runtime cross-checks it against the envelope file; a mismatch is an error.
- `used_anatomical_range_rad`: the anatomical range this rider actually uses.
  Every torque curve's knots must cover this range — evaluating outside the
  knots raises, it never extrapolates.
- `directions.positive` / `directions.negative`: the curve bounding a
  **positive** / **negative** q-torque. `label` gives the anatomical motion
  that torque produces (e.g. a positive knee torque flexes the knee).
- `angles_rad`, `torques_nm`: isometric torque-vs-angle knots (N·m,
  absolute, not mass-normalized).
- `vmax_rad_s`, `hill_c`, `eccentric_ratio`: Hill-type force–velocity
  envelope parameters (concentric zero-crossing, curvature, eccentric cap).
- `source`, `verified`: provenance string and the release-gate flag.

## Sources (planned extractions — all pending)

| Joint direction        | Placeholder anchor                                        | Status  |
|------------------------|-----------------------------------------------------------|---------|
| hip extension          | Anderson et al. 2007 (cycling hip extensor domain)        | pending |
| knee extension         | Anderson et al. 2007 (knee extensor domain)               | pending |
| ankle plantarflexion   | Anderson et al. 2007 (plantarflexor domain)               | pending |
| hip flexion            | dynamometer table TBD                                     | pending |
| knee flexion           | dynamometer table TBD                                     | pending |
| ankle dorsiflexion     | dynamometer table TBD                                     | pending |
| shoulder flex/ext      | dynamometer table TBD                                     | pending |
| elbow flex/ext         | dynamometer table TBD                                     | pending |
| trunk flex/ext         | dynamometer table TBD                                     | pending |

To verify a curve: replace the torques with the cited table values, record
table/row in `source`, set `verified: true`.

## The 450 W whole-body cap

`active_positive_power_limit_w = 450` is an **engineering budget** chosen
inside the user-requested 400–500 W band. It is not a physiological claim
about this (or any) rider. It caps `Σ max(τ_active · qdot, 0)` over the 11
internal joints; passive damping and eccentric (negative) work never count.

## Modeling boundary

Hill-type here is a mechanical torque envelope: finite isometric capacity,
falling concentric capacity to zero at `vmax`, bounded eccentric capacity
that can only absorb — never produce — power. No activation dynamics,
fatigue, metabolism, or reflexes are modeled beyond the existing first-order
activation filter.
