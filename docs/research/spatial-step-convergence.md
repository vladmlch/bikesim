# Spatial cornering, reflection and timestep qualification

Status: W35 source and acceptance suites written; numerical verification and
selection of production timesteps are **pending V06**. No project configure,
compiler, build, executable, simulation or test was run while authoring this
change. The numbers below are acceptance limits, not observed performance.

## Physical owners and reference input

The full-model tests construct `SetupOwner`, settle and seal its actual native
bootstrap, and advance `NativeRideRuntime` one accepted physical interval at a
time. The private `ride_qualification.hpp` fixture retains the immutable terrain
owner, complete material state, rider, drivetrain, steering, brake and accounting
owners. It does not reconstruct a pneumatic model from a rendered road or from
an engine integration vector alone.

`arena_rig` and `mirror_rig` are private test helpers returning an owned runtime.
The frozen `startup_fixture.hpp` interface is unchanged: its `RigModel` contains
only engine pointers and topology and cannot own a complete physical episode.
The arena is 120 × 120 m, flat, with no terrain features. Its authored input is
`examples/benchmarks/spatial_cornering.toml`.

The production roll/heading controller has a straight-line zero reference.
Cornering therefore supplies circular-reference errors through the private test
friend at the same integer control tick. It calls the actual `SteerBalance`;
the normal runtime call observes an already-consumed tick and cannot advance
the delay FIFO a second time. The fixture preserves the previous work total;
the accepted physical solve accounts actual actuator work. Gains, reaction
delay, torque cap and physical steering actuator are unchanged. This test seam
is not a new public trajectory-controller API.

Initial perturbations transform every independent free root, including the
articulated rider, by the same rigid transform. Angular and linear velocities
include the imposed yaw motion. They do not stretch attachments by rotating
only the bicycle frame. The changed initial energy is rebased before t=0
accounting starts; no initial perturbation is charged as unrecorded ride work.

## Cornering matrix and independent reference

The matrix contains radii 10, 20 and 40 m at 3, 5 and 8 m/s. A combination with
`v²/(gR) > 0.8*mu_peak` is explicitly printed as excluded before solving. Such
an exclusion is a declared friction domain, not a passing failed simulation.

The rigid thin-contact reference uses the native four-body Whipple model and
independent knife-edge rolling constraints from W34. A seventh, workless
constraint holds steering angle. A two-variable nonlinear solve finds lean
and steering for a relative equilibrium at the requested **COM** speed and
radius. The reference does not call a pneumatic or brush kernel and does not
use a large tyre stiffness as a surrogate for an ideal constraint.

For the following five seconds, the fixture uses the rotational symmetry of
the solved relative equilibrium to place native states on its orbit. At each
sample it solves the full native mass-matrix KKT system again and requires the
acceleration to match the prescribed centripetal acceleration, with constant
body-frame angular and hinge rates. Thus the kinematic orbit is checked by
native dynamics at every point. It is not a captured expected trajectory.
An independent least-squares fit to the resulting COM points supplies radius.
The support-force-weighted ground point supplies COM lean.

The reference limits are:

| Quantity | Acceptance |
|---|---:|
| KKT relative residual | `<= 1e-10`, full rank |
| Native acceleration along relative-equilibrium orbit | `<= 2e-5` in each generalized acceleration component |
| COM lean versus `atan(v_COM²/(g*R_COM))` | `<= 1 degree` |
| Mean lateral reaction versus `m*v_COM²/R_COM` | `<= 2%` |

The complete MTB uses its actual compliant tyres, suspension and articulated
rider. A bounded PI speed demand maintains the target during the cornering
qualification; it is separate from W36's prerecorded demand programs. Before
scoring, yaw rate, roll and total normal load must each have a coefficient of
variation below 1% over two seconds. The acquisition budget is 30 seconds.
After this condition the test scores a fixed five-second interval.

Failure to acquire or retain a complete scored interval prints `inconclusive`
with the terminal/acquisition reason and fails the test. It is never counted as
a valid short episode. On the scored interval, the fitted COM radius must be
within 5%, mean COM speed within 2%, every contact query valid, and bootstrap
hold and hold work absent.

Endpoint force probes are non-advancing and use the actual immutable material
registry. The total external force includes gravity and all tyre forces; the
moment is reduced about the complete-system COM, retaining each intrinsic
couple. Central differences of COM velocity and whole-system angular momentum
provide the independent dynamics checks. The momentum includes the spinning
wheels and articulated rider.

| Full-model balance | Acceptance |
|---|---:|
| `|F_ext - m*a_COM|/(m*g)` | `<= 1e-3` |
| `|M_ext - dH_COM/dt| / max(1 Nm, |M_ext|, |yaw_rate|*|H_COM|)` | `<= 1%` |

The endpoint probe and finite-difference bandwidth are deliberately explicit.
V06 must investigate a failed balance or refine the observation bandwidth;
it must not replace the physical force by `m*a` to make the check tautological.

## Reflection

The integrated mirror fixture reverses the grid's lateral elevation rows and
material-cell rows. Its asymmetric smooth bump and material zone therefore
become `h(x,-y)` and the corresponding mirrored material map. Initial lateral
position, roll, heading and steer angle also change sign. The IMU's mounting
position and orientation are reflected consistently.

Two complete two-second native episodes use identical longitudinal demand.
Positions are compared as polar vectors; orientation uses the reflected
quaternion `(w,-x,y,-z)` and geodesic angular distance. Acceleration sensor axes
are polar, while gyro axes are axial. The test checks mirrored steering torque
and equal steering work as well as the trajectory.

| Mirror comparison | Acceptance |
|---|---:|
| Position | `<= 1e-7 m` |
| Orientation | `<= 1e-7 rad` |
| Reflected accelerometer/gyro components | `<= 1e-6` in their native SI units |
| Steering torque and work symmetry | `<= 1e-7` |

Separate isolated tests use `std::bit_cast<uint64_t>` for the nonzero odd lateral
components and even brush state/energy, and for contact coordinates, indentation
and cell areas on an independent analytic plane. Symmetric torus midpoint
coordinates and integrated areas are paired exactly. These tests do not claim
that every arithmetic zero has the same sign bit under a general rigid transform;
the complete integrated trajectory uses the stated numerical tolerances.

## Timestep matrix

| Tier | Coarse `dt` | Half `dt` | Quarter `dt` | Episode |
|---|---:|---:|---:|---:|
| fast | 0.00125 s | 0.000625 s | 0.0003125 s | 10 s |
| detailed | 0.0005 s | 0.00025 s | 0.000125 s | 10 s |

All cases use the same seed, articulated rider, initial perturbation, coefficient
source pin, control period and TOML demand timestamps. The control period is
0.005 s, an integer number of physical steps at every listed `dt`. Orientation
is compared on this common observation clock. Each tier pins one closure time
constant for all three members (fast 0.0025 s, detailed 0.001 s, at least `2*dt`
at the coarsest `dt` of the tier), so only `dt` differs between the coarse, half
and quarter runs; the selected production `dt` is qualified with that closure
constant (`native/tests/spatial/timestep.cpp`). Contact topology, landing,
airborne and rim events, and load extrema are inspected at every physical step.

Each coarse and half-step result is compared against its tier's quarter-step
result. Event identity/order must agree and event time may differ by at most
one step of the coarser member. Mean load may differ by 2%, peak load by 5%, and
orientation by 0.5 degrees. Each run must close its energy ledger within 0.5%
and satisfy `valid_duration_s == requested_duration_s == 10`. Invalid or early
terminal runs fail before an accuracy comparison is reported as qualified.

## Evidence record

The suites print JSON observations when deliberately executed later. There are
no results to populate yet:

```json
{
  "schema": 1,
  "verification": "pending V06",
  "compiler_and_engine_identity": null,
  "cornering_ideal": null,
  "cornering_mtb": null,
  "mirror": null,
  "timestep_fast": null,
  "timestep_detailed": null,
  "selected_fast_dt_s": null,
  "selected_detailed_dt_s": null,
  "qualification_digest": null
}
```

The candidate timesteps in the source are not newly qualified defaults. The
normal-contact topology/work and material-transport C10 gaps described in the
tyre-law documents can affect these tests and remain explicit unresolved gates.
