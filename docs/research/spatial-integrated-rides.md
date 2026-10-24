# Spatial integrated ride qualification: F / B / R / G / C

Status: W36 source and strict acceptance checks written; execution and numerical
evidence are **pending V06**. The implementation has not been configured,
compiled, built, run or tested during this authoring phase. No success rate,
force error, speed, energy residual, timing or continuation result is asserted
from an unexecuted suite.

## Runner and source identity

`native/tests/spatial/target_rides.cpp` runs the actual `NativeRideRuntime` and
`PhysicalStep` through an owned settled bootstrap. Its input is an authored
`[demand_trace]` TOML array of `[time_s, torque_fraction, brake_fraction]` rows.
The demand is piecewise constant and starts at zero time; timestamps must be
strictly increasing. Fractions are bounded in `[0,1]`. The motor command and
limit are 85 Nm times the fraction and 85 Nm respectively; human torque is
zero, while the configured articulated rider and its physical attachments
remain active. Front and rear brake fractions use the same trace.

The trace is evaluated against absolute physical time. Changing measurement
decimation does not change force application or the integer control clock.
The runner records valid duration, completed distance, first terminal reason,
invalid intervals, scored hold intervals, peak/mean normal load and physical
contact events. It checks each actual material patch's combined friction ellipse
using that material's Stribeck coefficient and the corresponding endpoint
kinematics. A peak-grip upper bound is not substituted for the current friction
coefficient.

The four files `examples/research/spatial_flat.toml`,
`spatial_cross_slope.toml`, `spatial_rough_uphill.toml` and
`spatial_split_grip.toml` are low-level qualification inputs. They do not pretend
to be the later W40 research-session or observation-V2 interface.

The coefficient identity is independently pinned in
`native/tests/spatial/data/ride_profile_pin.json`; the controller's declared rows
are pinned by `data/steer_profiles.json`. The tyre pin contains the complete
projected tyre coefficient record, with two declared derived quantities:
Maxwell stiffness from its pinned rate factor and initial torus volume from
its pinned geometry. Tier is supplied by the scenario. Numeric JSON spelling
is normalized without changing the values. The full canonical record must
match the live cold projection before its SHA-256 is computed.

Consequently each report's tyre and controller content hashes must equal the
manifest-derived source hashes. A runtime selecting another coefficient,
geometry, pressure, delay or gain row cannot silently update both expected and
actual observations. These content hashes identify the authored source; they
are separate from a measured calibration hash or a completed qualification
hash. Measurement and qualification hashes remain null/pending V06.

## Scenario matrix

| Scenario | Physical case | Required completion and acceptance |
|---|---|---|
| F | Flat, initial 3/5/8 m/s; roll ±3 degrees; heading ±2 degrees; articulated rider; both tiers | All 20 s valid; no physical terminal; lateral position within 0.5 m; no scored bootstrap hold |
| B | Smooth cross-slope ramps to ±5/±10 degrees and back to flat; both tiers | All 20 s valid; plateau sampled; return beyond x=68 m; lateral position within 0.75 m; signed load and steering response |
| R | Fixed-seed savage rough/uphill producer; baseline articulated rider; both tiers | At least 100 m within a 120 s budget; no invalid/hold intervals; rim penetration zero; complete energy residual at most 0.5% |
| G | Split-grip braking while turning, stop, hold on 5 degree slope, gradual release and restart; both tiers | All 12 s valid; per-patch shared friction budget; held speed at most 0.03 m/s and drift at most 0.01 m; restart above 0.5 m/s |
| C | Same-process capture and restore during a turn with braking and a material patch split; both tiers | Every continuation state/observable equals the uninterrupted branch; full pending accounting state preserved |

### F: balance recovery and negative control

The complete flat matrix also carries the W32 dynamic recovery gate. At 3 m/s,
the boundary nearest three seconds must have absolute roll below 0.5 degrees
and lateral position below 0.3 m. Across recovery, continuous steering saturation
may not exceed forty 200 Hz control ticks. Both roll and heading signs are tested,
with the same physical strengths, delay and source profile at every speed.

A separate controller-off episode prints its observed fall/terminal time and
maximum absolute roll. It has no asserted fall time. It is a diagnostic control,
not evidence that any particular fall duration has been reproduced.

### B: cross-slope and friction domain

The analytic bank uses quintic entrance and exit ramps. On the settled plateau,
tangent contact load must point uphill, opposing the lateral component of the
road normal. The signed plateaus at the two bank signs must reverse lateral
load and steering response. The test also verifies actual traversal back onto
the flat section, so a ride stopping before the ramp cannot satisfy the case.

A separate 10 degree bank with `mu_peak=0.02` deliberately lies outside the
friction domain. It must terminate physically as capsize, corridor loss or
balance loss with valid physics. Terrain/numerical invalidity is not accepted
as a successful demonstration of an impossible bank.

### R: requested course, not a surviving prefix

The terrain is loaded from the existing
`rough_uphill_savage_3d_genspec.toml`, preserving seed `0` and its complete base
track. The test generates that actual producer result and retains its immutable
owner and digest. It reports integrated load impulse, peak normal load and
maximum suspension travel, in addition to distance and energy closure.

There is a known producer qualification dependency: the authored W09 savage
recipe contains legacy square edges that can violate the grid's maximum slope
at 5 mm resolution. The strict generator is allowed to reject that recipe.
The test prints a blocked acquisition row and fails; it does not replace the
course with a smoothed or easier surface and then label the ride "savage".
Resolution of that producer issue belongs to W09/V02 and is a prerequisite for
R's V06 numerical qualification.

Actual steering and motor torques remain below 40 and 85 Nm respectively.
The solved hinge-friction force plus elastic brake bite must obey the one total
brake torque budget. The rough-uphill acceptance rejects any rim penetration
and requires `completed_distance_m >= requested_distance_m`; an early valid
fall or a shortened run is insufficient.

### G: intentional stop and physical feasibility

The test explicitly configures a 20 second low-speed balance dwell for the
12 second intentional stop/hold/restart manoeuvre. This is a scenario policy:
the ordinary half-second low-speed outcome would otherwise end the requested
stopped manoeuvre before release. Geometric capsizing, corridor, terrain-domain
and numerical guards remain active. No bootstrap hold, artificial roll support,
extra contact friction or tyre braking torque is introduced.

Circular-reference errors are supplied through the private W35 steering-input
seam. As speed decreases, the demanded lean tends to zero. The declared braking
trace increases demand gradually, holds the brakes, then releases through four
levels before applying drive again. The shared per-patch friction ellipse is
checked with the actual dry/wet material coefficient throughout.

A separate deliberately impossible slope-hold case uses `mu_peak=0.02` and full
brake demand. It must exhibit longitudinal drift or physical capsize while
remaining a valid simulation. A locked wheel is not evidence that ground grip
can exceed the tyre friction budget.

### C: complete private continuation checkpoint

The frozen public runtime has no arbitrary mid-episode restore API; the generic
persisted spatial checkpoint is W42. C uses an explicitly private test friend,
`bike::validation::SpatialRideAccess`, and restores only into the matching
native model/runtime. It does not expose an engine-vector-only restore to users.

The checkpoint is taken one physical step after a 200 Hz control boundary, while
steering and braking are nonzero and the resolved material field has split the
contact into multiple patches. The accounting period must still contain an
unpublished interval. The fixture then records 0.4 s of uninterrupted evolution,
restores the checkpoint and repeats exactly the same absolute-time input trace.
The cold checkpoint inventory lives on the heap; it is not a physical-step
workspace or a warm-path allocation requirement.

Its inventory includes:

- Full copied `mjData`, including derived solver buffers and all warmstarts.
- Mutable model friction-loss, contact-site, tendon and wrap parameters.
- Complete tyre material/patch state, brake latch and angle history, controller
  FIFO/tick/work, hold phase and latched outcome.
- Drivetrain and rider/contact/controller/intent/filter owners.
- Physical interval force/constraint/brake snapshots and acceleration-input cache.
- Pending period buffer with its interval/time markers, accumulated accounting,
  publication indices, retained observations, committed boundary and presentation.

Restore retains the copied derived engine data and does not execute a forward
solve that would consume or alter a warmstart. The final complete physics bytes,
steering codec bytes and every sampled position/orientation/load/work observable
must match the uninterrupted continuation. Immutable model/terrain/profile
identities are checked before restore. The diagnostic observation probes do not
advance material or causal owners.

## Evidence skeleton and remaining gates

All numerical cells below are intentionally empty:

```json
{
  "schema": 1,
  "status": "source_written_pending_V06",
  "binary_identity": null,
  "engine_identity": null,
  "resolved_source_profile_digests": null,
  "F": {"qualified": null, "maximum_lateral_m": null, "roll_at_3s_rad": null},
  "B": {"qualified": null, "signed_loads_n": null, "physical_outside_mu_terminal": null},
  "R": {"qualified": null, "completed_distance_m": null, "maximum_rim_penetration_m": null, "energy_residual_fraction": null},
  "G": {"qualified": null, "maximum_ellipse": null, "hold_drift_m": null, "restart_speed_mps": null},
  "C": {"qualified": null, "bitwise_continuation_equal": null},
  "qualification_digest": null
}
```

The current coefficient and steering profiles remain provisional. Material
fractional advection and previous-tangent transport are limited by the frozen
W00 state. Normal contact entry/exit work closure has an explicit unresolved
acceptance row. These are physical qualification issues, not reasons to remove
a failing check, silently widen its limit, or mark the capability qualified.
`ride:spatial_free` must therefore retain `qualification_pending=true` until V06
has supplied reproducible evidence and addressed any failed gates.
