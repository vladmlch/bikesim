# Longitudinal research plant completion

## Goal
Complete the supplied planar bicycle/rider/road plant as an offline, repeatable
software-in-the-loop testbed for future motor-torque anti-wheelie controllers.
The existing articulated multibody model, suspension, drivetrain, unilateral
rider contacts, graded/material-zoned roads and energy gates remain authoritative.

## Requirements
- Work against the supplied source snapshot, not a replacement downloaded project.
- Python 3.13, existing NumPy/SciPy/Matplotlib/Pytest, supplied MuJoCo wheels only.
- Preserve legacy ride modes and existing force ownership and work accounting.
- Add range-checked nonlinear radial tire curves with exact elastic energy.
  Synthetic example data must never be called measured data.
- Sample sensors on their own physics-aligned clock, not on policy calls; expose
  bias, missing samples and stale data, with deterministic reset and no truth leak.
- Add time-programmed rider posture/effort independent of motor-policy cadence,
  continuous interpolation, no qpos/qvel prescription and explicit ownership.
- Make road contact/model-range violations distinguishable from numerical errors.
  Single equivalent contact cannot certify multi-support geometry; do not silently
  accept it as valid learning data. Preserve diagnostic runs on explicit request.
- Expose physical configuration and road mesh refinement from the research CLI.
- Save enough self-contained configuration and controls for checked deterministic
  replay, including brake commands, sensor seeds and rider programs. Verify source,
  environment, terrain, initial state and resulting trajectories, not just exit status.
- Ship tested source patch, rebuilt project wheel and the original offline dependency
  wheels; do not ship environment directories or machine-specific caches.

## Non-goals and validation boundary
This is a longitudinal X-Z model, not a new lateral balancing simulator or a
validated human/soil model. Hardware parameters require measurements. A low
energy residual is necessary, not evidence of real-bicycle safety. No controller
may receive simulator-only contact or terrain truth through the sensor interface.
