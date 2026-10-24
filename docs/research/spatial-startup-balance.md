# Spatial startup, steering and terminal outcomes

Status: implementation and acceptance cases authored for W29–W33.
Verification: **pending V06**. No configuration, compilation, executable,
simulation or test run was performed while preparing this change.

## Equilibrium and release

The spatial initializer queries the physical terrain at the wheel hubs. It
uses their heights and outer radii to seed height and pitch, then transports
the rider roots with the same rigid transform. The initial compression is a
geometric seed; the equilibrium solver determines the physical sag.

`spatial_refine` varies a tangent displacement and calls `mj_integratePos` to
construct each candidate. Free-joint angular tangent coordinates are local to
the body, so the allowed world roll and pitch directions are rotated into
that coordinate system. World x and yaw remain fixed. Scalar joints and ball
joints retain their native position/velocity widths. The finite difference
step is `1e-6`; the existing bounded least-squares provider owns the solve.

Every candidate restores the integration inventory, mutable model rows,
drive, tyre, rider and spatial causal owners before force evaluation. Tyre
evaluation is a probe. A one-evaluation report restores the original
integration bytes, including the warm start, after deriving its report.

The report separates translational acceleration in m/s² from angular
acceleration in rad/s². It also records total/valid tyre contacts, vertical
load error relative to total weight, roll and pitch external moments, the
three largest acceleration DOFs, evaluation count and explicit errors.

The bootstrap hold acts along world Y and about world X/Z. It leaves height,
pitch and physical suspension compression free. Its state stores the world
wrench and potential energy. Accepted initialization intervals integrate
signed `F·v + M·ω` into the separate hold-work lane.

Release proceeds through `settling`, `release_check`, `armed`, then `scored`.
It requires valid contacts and residual reports, force below 1 N and moment
below 1 N·m. Timeout or a required residual hold throws a descriptive error.
Armed and scored phases contribute exact zero hold wrench and preserve the
accumulated initialization work. They cannot reactivate a restored hold.

Moving initialization sets root velocity from the physical heading and
terrain tangent, wheel speed from each loaded rolling radius, and drive
rates where pedaling requires them. It resets accepted tyre/brake interval
numbers and the steering delay clock before scored interval zero. Initial
mechanical and stored energy are captured after these velocities are set.

## Complete startup transfer

Fresh, cached and explicitly imported startup use the same resolved physical
configuration. Identities bind topology, physical terrain, tyre calibration,
steering profile, model dimensions, native provider and the applicable
integration/solver configuration. Runtime seed and generator provenance are
reported separately from the physical terrain identity.

Optional cache corruption records a rejection reason and starts a fresh
solve. An explicit imported-state mismatch is an error. A planar bootstrap
cannot supply the spatial producer inventory.

The spatial `PhysicalInitialState` payload carries `mjSTATE_INTEGRATION` and
all seven mutable model input rows, in addition to the material, drive,
rider and spatial codecs. Validation stages the integration vector in
independent engine storage and checks its qpos/qvel/warm-start envelope
before a live owner changes. The planar transfer document remains on its
existing layout.

The startup-path comparison includes integration bytes, mutable model rows,
tyre bytes, drive, rider contacts/controller/intent, control state, contact
filters and queries, energy/accounting and hold/brake/steer/outcome owners.
Only origin/cache/source metadata is outside this comparison. It also
compares the first observation and first force-stage components bitwise,
then hashes 1,000 accepted continuation intervals on the same binary.

## Steering and outcome semantics

The `roll-heading-v1` profile has speed nodes 1/3/5/8 m/s. At 3 m/s the six
gains are `[-40,-8,-6,-2,-1,-0.3]`; the other nodes use the specified
1.6/0.8/0.6 scales. Its status remains `provisional`. The canonical profile
payload, including cap and requested delay, determines its content digest.

Steering consumes one integer control tick at a time. Linear interpolation
and edge clamping select gains; the strength cap precedes the delay queue.
At 200 Hz a 0.1 s delay is exactly 20 ticks. Disabled ticks advance the queue
with zero and apply zero even if the ignored observation is nonfinite.
Duplicate ticks cannot advance state. Restore validates the complete queue,
profile identity, cap and finite counters before copying. Setup metadata
reports the resolved delay in ticks and seconds. Runtime work uses the
actual solved actuator torque over accepted physics intervals.

Spatial outcomes latch once. The priority is numerical failure, invalid
terrain query, outside-domain, capsize/body contact, corridor crossing,
low-speed dwell, then requested completion. A singular Euler attitude alone
is not capsize. Corridor comparison includes the bicycle half-width. Invalid
queries cannot appear as a valid airborne interval.

The accepted outcome is attached to each spatial sample. Research/status
consumers use that sample's latch rather than reading a later live runtime
state. Metrics distinguish requested duration, valid accepted duration,
completed distance and terminal reason. Duplicate publications do not
increase progress; gaps are errors; an invalid endpoint does not add its
distance or duration; the first terminal freezes the episode. Spatial
invalidity propagates to model status, evidence summaries and exit status.
Planar records do not acquire spatial-only keys. This work does not remove
the separate W37 gate on the spatial research-observation frontend.

A failed force assembly retains the previous committed engine and tyre
prefix. It publishes only the new invalid outcome beside that prefix and
restores live brake/steering history and brake bounds. Further advancement
requires reset. The startup suite induces a real degenerate tyre geometry
after a brake/steering tick to check this boundary. Research failure cleanup
consumes already accepted samples before freezing its invalid terminal
metrics, preserving the valid trailing prefix.

## Authored acceptance

| Area | Authored checks | Evidence |
| --- | --- | --- |
| W29 equilibrium | constrained/free roots, absent/articulated rider, 0.05 acceleration limits, 0.5% weight error, 1 N·m moment limits, unit quaternions, q/−q equivalence, independent `−k/m` finite difference, unconditional one-evaluation failure, unchanged probe material/integration | pending V06 |
| W30 hold | 90° yaw with world-Y support, offset rejection, 10° bank rejection at zero speed, zero-speed scored startup, exact zero armed/scored force, accepted hold work, scored restore | pending V06 |
| W31 paths | fast/detailed × constrained/free × rigid/articulated, fresh/cache/import bytes, first observation/force parity, 1,000-step continuation, corrupt cache, explicit mismatch, invalid attempted-step prefix and reset recovery | pending V06 |
| W32 controller | zero/sign/cap, 20-tick delay, duplicate tick, disable/reset, profile hash, active queue restore/replay; dynamic perturbations in integrated ride suites | pending V06 |
| W33 outcomes | exact corridor/roll thresholds, invalid priority, immutable terminal tick, recovered dwell, deterministic replay, accepted-prefix metrics | pending V06 |

Measured convergence, ride recovery, throughput and qualification are not
claimed by this document. Numerical results belong to the V06 evidence
manifest and must retain the resolved profile and terrain digests.
