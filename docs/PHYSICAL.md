# Physical longitudinal ride model

This guide describes `--physics physical`. The historical model remains available
as `legacy`; an unchanged command line still selects it. See [RIDE.md](RIDE.md)
for legacy behavior, tracks and general viewer usage. The implementation contract
and acceptance criteria are in
[the physics specification](superpowers/specs/2026-09-28-physics-correctness.md).

## Scope and provenance

The model is planar: X forward, Z upward, wheel rotation about +Y. It is not a
three-dimensional balance model, a soil model or a measured reproduction of a
particular bicycle. Defaults for tires, anatomical inertias, contact compliance,
transmission, motor losses and postural control are **synthetic**. Passing
mechanics tests does not constitute experimental calibration. Run metadata keeps
`calibration_status=parameterized_unvalidated` until independent measurement and
holdout evidence exists; component names cannot upgrade this status.

The native tire backend is an explicit solver reference. Its `solref`/`solimp`
are not material stiffness/damping in N/m and N*s/m. The separate `compliant_2d`
backend uses SI material laws and the actual compiled heightfield raster. Native
wheel/terrain contacts are disabled for that backend; crash contacts remain.
There is no hidden fallback to the other tire backend.

One equivalent contact region is supported per wheel. Distinct simultaneous
support regions set `multi_support`. Such intervals remain in the data but do
not establish calibrated peak-load predictions. A wheel center below the solid
profile, an unsupported surface transform or departure from the working surface
is an explicit failure, not a reflected normal or a silently substituted road.

## Running

In a normal project checkout install the unchanged locked dependencies:

```sh
uv sync --locked
uv run --locked bike-ride --physics physical --drive coast --initial-speed 20 \
  --rider lumped --headless --duration 5 --no-plots --out output/physical-coast
uv run --locked bike-ride --physics physical --drive crank_effort \
  --human-torque 20 --assist-gain 2 --rider lumped --headless --duration 5 \
  --no-plots --out output/physical-effort
uv run --locked bike-ride --physics physical --drive articulated_effort \
  --human-torque 20 --rider articulated_planar --headless --duration 5 \
  --no-plots --out output/physical-articulated
```

`--initial-speed` is an initial condition in **km/h**. `--speed` is only the ideal
regulator's target and is rejected for coast and either effort mode. A stationary
headless coast stand requires `--duration`; not reaching a track finish is not a
failure when a time-limited stand was requested. The CLI returns nonzero for a
crash, numerical/geometry error or an exhausted safety limit, and records the
reason when it can save a run.

Four drive modes are distinct:

| Mode | Physical source |
|---|---|
| `coast` | No human or motor propulsion. |
| `ideal_speed_control` | Explicit external rear-wheel test actuator; metadata identifies it as an external speed controller, not motor assist. |
| `crank_effort` | Human crank moment and mid-drive moment. The transmission model is selected under `[drive]`. |
| `articulated_effort` | Bounded internal rider joint actuators act through unilateral feet and pedals. No second human-crank actuator exists. The mid-drive remains a separate source. |

Unloaded rear contact does not disable effort input. Wheelspin is possible: tire
force, not motor torque, is friction-limited. Either brake request cancels motor
assistance and positive ideal-regulator torque on the current interval. Physical
braking uses the engine's bounded static friction constraint, not a tapered
constant moment. Reported brake work comes from its identified constraint rows.

A zero/low-cadence unassisted start can stall near a dead center: delivered human
moment is not forcibly made equal to its requested phase average. Contact loss,
reach limits and actuator saturation remain observable rather than being fixed
by prescribing the crank or foot coordinates.

## Physical pedaling and climbing regression checks

A rolling effort-mode initial condition now initializes the crank, cassette,
platform and leg velocities consistently with wheel speed and gearing. These
velocity writes occur **only at reset**; an unpowered elastic drivetrain still
starts freewheeling. The ideal transmission also permits stationary cranks during
overrun. A stationary start does not inject crank, pedal, wheel or rider velocity;
the configured human effort starts pedaling through the existing actuators and contacts.

Moving leg targets include a central-difference acceleration feedforward through
the compiled mass matrix, in addition to position/velocity feedback and bias
compensation. All terms remain inside the existing internal joint torque,
positive-power and speed limits; `tracking_nm` exposes the scaled inertial term.
The desired downward stance force is no longer suppressed by the very absence
of contact that the foot needs to recover. Its tangential part is still limited
by measured normal load, and actual foot/pedal contacts remain unilateral.

Arm IK allows the grip spring's requested deflection (`force_on_bike / grip_k`)
instead of demanding both a nonzero support force and zero spring deformation.
Only actuator goals change: no hand, foot, pelvis, crank or bike pose is imposed
while stepping. Terrain geometry, tire forces, support release rules, motor
assistance and anti-wheelie logic are not changed by these corrections.

Use the installed offline environment and explicitly select the patched source
checkout, rather than an older copy of the bundled wheel:

```sh
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
python -m pytest -q tests/test_physical_pedaling_regression.py
python examples/research/check_pedaling.py --initial-speed-mps 0 --duration 5
python examples/research/check_pedaling.py --track rough_uphill --duration 13
```

The last two commands are **opt-in integration checks**, not part of the default
unit test suite. They use the unchanged `viewer_physics_fast.toml`, ordinary
static initialization and real physical stepping. They report progress, actual
crank rotation, measured pedal work, contact gaps, tire loads and crash status;
a zero front-wheel load is allowed, not concealed. The rough check traverses the
22% climb, the 6 cm bump and the 4 cm step when the reported final X exceeds
16 m. Timing and command values here are SI (m/s and seconds).

These checks use physical preview to omit expensive scientific accounting, not
to simplify the applied forces. They do **not** establish energy convergence,
calibration, a guaranteed climbing capability at every power setting, or a
validated strict-profile (`viewer_physics.toml`) trajectory. In particular, a
physically underpowered climb may still stall; no artificial speed regulator or
front-wheel hold-down force is introduced.

## TOML and API

Resolution is defaults, then TOML, then explicitly supplied CLI values. Unknown
keys and invalid combinations are errors. The top-level TOML matches
`SimulationPhysicsConfig`; nested tables match its immutable component configs.
All TOML dimensional values use SI unless a field name explicitly states another
unit. For example:

```toml
physics_mode = "physical"
drive_mode = "crank_effort"
timestep_s = 0.0005
closure_time_constant_s = 0.001
initial_speed_mps = 0.0

[tires]
backend = "compliant_2d"

[tires.front]
mu = 0.8

[tires.rear]
mu = 0.8

[drive]
transmission_model = "elastic_chain"
human_torque_nm = 20.0
crank_phase_rad = 0.0

[drive.gearing]
front_teeth = 34
rear_teeth = 24
chain_pitch_m = 0.0127

[drive.assist]
gain = 2.0

[resistance]
crr = 0.015
cda_m2 = 0.5
wind_world_mps = [0.0, 0.0, 0.0]
```

```sh
uv run --locked bike-ride --physics-config physical.toml --rider lumped \
  --headless --duration 5 --no-plots
```

```python
from bike_sim.physics.resolution import load_physics_config
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.sim.ride.physical_recorder import PhysicalRecorder
from bike_sim.terrain import get_preset

config = load_physics_config("physical.toml")
sim = RideSimulation(track=get_preset("flat"), rider="lumped", physics_config=config)
recorder = PhysicalRecorder(sim, decimate=10)
for _ in range(round(1.0 / config.timestep_s)):
    sim.step()
    recorder.record(sim)
recorder.write_csv("output/physical.csv")
```

Do not pass the legacy `TyreConfig(pneumatic...)`, old drive-mode selector,
visual-pedaling or leg-weld flags to physical mode. They select different models,
not aliases. `articulated_effort` requires `articulated_planar`.

`closure_time_constant_s` must be at least twice `timestep_s`. Otherwise MuJoCo's
safety rule changes the effective closure stiffness as dt changes. The default
1 ms reference is held fixed for the 0.5/0.25/0.125 ms refinement matrix.

`drive.transmission_model = "elastic_chain"` retains the research drivetrain with
separate cassette and freehub dynamics. `"ideal_mid_drive"` removes those high-frequency
states and uses a massless one-way gear-ratio constraint. The wheel can overrun stationary
cranks; a ratchet boundary follows that overrun and re-engagement is solved without
rewriting joint positions or velocities. Motor torque acts at the crank, not directly
at the rear wheel, so it cannot bypass the freewheel. Solved transmission forces and
their constraint work are recorded separately. It is intended for interactive plant/control development,
not for chain, freehub or drivetrain-energy studies.

`equilibrium_cache_enabled = true` enables a validated initial-pose cache for
interactive runs. The cache is keyed by source, compiled model, road, rider and
physics configuration; a mismatch falls back to the full equilibrium solve. The
default is `false`, so research runs do not silently reuse an old initial state.
Files are kept under the system temporary directory; set
`BIKE_SIM_EQUILIBRIUM_CACHE_DIR` to choose another location or remove old entries.

## Initialization and rider behavior

Crank phase and independent rider pose are established **before** coupled static
equilibrium. The solve checks actual compiled accelerations, not a desired load
split. Its physical relaxation-time budget is independent of dt. Refinement and
velocity resets occur only during initialization. The running physical path
never assigns qpos/qvel to repair dynamics and never applies external pitch
stabilization or a hand-computed load-transfer force.

`--sag` fits pressure and coil rate against repeated compiled equilibria. Both
achieved wheel travels must be within 0.5 mm of the request; an unattainable fit
is an error. Resolved metadata records the final active components, including
explicit API overrides, rather than just the geometry's original defaults.

The articulated pelvis has independent X/Z/pitch coordinates. Anatomical masses
do not depend on desired support shares. Saddle and pedal patches can push but
not pull; both feet can leave the platforms and the bilateral hand grip can be
released. Equal/opposite interface forces act at a common point. Selected body
crash geoms collide with the terrain. No slider or foot weld survives release.

Postural contact-force requests are targets for joint actuators, not replacements
for actual contact forces. They balance the current anatomical gravity wrench
where the available supports permit it; infeasibility is reported. Swing legs
follow reachable pedal IK targets, while stance joints oppose unwanted pelvis
pitch through physical contact reactions. Requests are limited after summing
posture, gravity/support and pedaling terms. Motor work and joint work are
recorded separately from moment/power actually delivered to the crank.

## Outputs and accounting

A headless run writes a drive/config-hashed directory containing `summary.json`,
`telemetry.csv`, `intervals.jsonl`, `terrain_vertices.npy`, and optional plots.
Different physical drive settings therefore cannot collide merely because their
track and nominal speed are the same. Metadata includes the exact terrain hash,
resolved configuration, source-content hash, available git revision/dirty state,
actual library versions, numerical solver settings and parameter provenance.

Schema 2 records incoming state time and force interval explicitly. Force power
uses the same incoming velocity; work uses a left rectangle at **every** substep,
even when rows are decimated. Endpoint mass/energy channels are labeled as such.
Tire normal load, total vertical force, normal-only vertical force and controller
`grounded` are separate. Physical airtime uses raw working-road load, with several
load thresholds retained; missing snapshots are not treated as flight. Catch-plane
contact is an emergency event, not restored working-road support.

Elastic energy, actuator work, external work, dissipative losses and unexplained
numerical residual are separate. Aerodynamic work is a signed external channel
(wind can supply energy), not also added as a loss. Native contact/closure work is
reported as a solver channel without inventing a material spring energy. Battery
work is a separate electrical ledger; only actual energy-limited shaft effort is
applied. Reset starts a new generation and requires a new recorder.

Headless runs and physical plots consume these immutable samples. The interactive
viewer instead uses a preview path with the same force laws, controller updates,
material-state advancement, battery settling and crash checks, but without the
per-step scientific samples and energy/work audit. Its HUD labels this as
`PHYSICAL PREVIEW`, reports measured simulation/wall-time `RTF`, and shows
`energy audit: off`. Closing the viewer restores the preview flag; a successful
reset is required before research stepping or recording can resume.

The interactive physical viewer also writes `preview.csv` beside the hashed run
artifacts under `--out` (by default `output/ride/`). It flushes one row per
0.1 s of simulated time, including speed, wheel loads, pitch, fork/shock state,
cadence, human/motor torque, rider grip/pedal contact and IK
saturation. It also records crank phase and front/rear stance, rider-root/pelvis/torso angles, motor request versus
delivered torque, assist latch state, grade, nearby obstacle and tire slip. The
`event` column carries `reset`, `run ended` or `viewer closed` markers. The file
can be followed live with `tail -f` or loaded directly with `pandas.read_csv`.

`examples/research/viewer_physics_fast.toml` uses the `ideal_mid_drive` transmission,
a 1.25 ms step, a 2.5 ms closure time constant and 100 N s/m pedal damping. These are
explicit synthetic preview parameters, not a convergence-validated replacement for
the strict configuration. The road, tires, suspension, bicycle body and articulated
rider remain in the physical preview path; only drivetrain internal dynamics are
removed.

The fast profile starts at `initial_speed_mps = 0.0`, with the rider requesting
`human_torque_nm = 20.0` from the first running interval. The initial 34x51 gear is
the easiest gear of its configured cassette. Motor assistance still uses the measured
pedal torque and configured cadence/response gates; no startup speed or motor impulse
is injected. Initialization brakes hold the equilibrium pose only, not the running ride.

Automatic shifting is enabled for this profile under `[drive.shifting]`:

```toml
[drive.shifting]
enabled = true
cassette = [10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51]
target_cadence_min_rpm = 65.0
target_cadence_max_rpm = 85.0
shift_cooldown_s = 0.4
shift_cut_duration_s = 0.2
torque_factor = 0.3
```

Below 65 rpm the rider selects the next larger rear sprocket (an easier gear);
above 85 rpm the next smaller sprocket (a harder gear). Decisions use the larger
of actual forward crank cadence and wheel-required cadence in the current gear,
both smoothed by `cadence_smoothing_tau_s` so a sub-100 ms transient cannot
trigger a shift a rider would never perceive. An upshift is also refused while
the rear tire slips faster than `upshift_slip_limit_mps` (default 0.5 m/s):
a spinning wheel inflates the implied cadence without accelerating the bike.
This allows selecting a usable gear while the wheel overruns stationary pedals.
Only one sprocket is selected per shift, with at least 0.4 s between changes;
braking, zero requested human effort, a disabled rider or an airborne rear wheel
prevent new shifts. The initial rear sprocket must belong to the cassette.

For 0.2 s after a change the human effort request is multiplied by 0.3 and motor
output is capped at 0.3 of its delivered torque immediately before the shift.
The motor cap is latched for that interval, not multiplied into the previous output
on every step. Existing motor lag, safety ceilings and battery limits still apply,
including for an external research motor request. Foot support/posture actuation
remains active during the relief interval.

The native transmission ratio and solver constants are updated at each change,
reanchoring the ratchet without a crank/wheel position or velocity rewrite. The
solver handles the resulting speed transition; shift transients and real-time
performance still require runtime validation. Automatic physical shifting is
opt-in outside this profile and currently requires `ideal_mid_drive`; the detailed
`elastic_chain` drivetrain retains its fixed sprocket geometry.

The fast profile enables cadence coasting under `[drive.pedaling]`:

```toml
[drive.pedaling]
enabled = true
coast_above_rpm = 110.0
resume_below_rpm = 90.0
stop_time_s = 0.35
```

Above the upper cadence threshold the rider stops requesting pedaling effort. The
decision uses both actual crank cadence and the cadence required by rear-wheel speed
in the selected gear, so stopping the crank does not immediately restart pedaling.
The lower threshold supplies hysteresis. `stop_time_s` ramps the rider's crank-position
goal to rest; it does not overwrite the actual crank velocity. Both feet receive
stance/support goals, realized only through bounded internal limb actuation and the
existing unilateral pedal contacts. Saddle, hand and pedal forces still determine
the actual load transfer to the bicycle. This is a synthetic rider policy, not a
cadence calibration or an automatic gear-shift controller. Cadence coasting is disabled
by default outside the fast profile; zero requested effort or braking also requests coasting.
Gear selection runs before the cadence-coasting decision. If a harder gear brings
wheel-required cadence below the resume threshold, the rider can resume pedaling;
if the cassette is exhausted, high-cadence coasting remains available.

While coasting, automatic assistance receives zero pedaling demand, even if passive
foot support produces a positive raw torque-sensor reading. The raw sensor remains
logged. The configured motor response and stop delay still apply; an explicit
research motor setpoint retains its separate command path. Preview logs include
`rider_mode`, `coast_reason`, `required_cadence`, `crank_goal`, `human_cmd`,
`assist_input`, `freehub` and `freehub_torque`. Logs also include `gear`, `last_shift`
(direction, old/new sprocket, simulation timestamp and counter) and `shift_cut`.
The last event persists across 0.1 s log samples so a shift is not lost between samples.

Mid-drive response parameters are overridden under `[drive.assist]` in any physical
TOML file. For example, `gain`, `tau`, `stop_delay`, `slew`, `max_torque`,
`max_power` and `torque_curve` (a piecewise-linear rpm/torque ceiling, end values
held outside its range) are accepted there and override their `AssistConfig`
defaults. `[drive.battery]` accepts `enabled`: `false` removes the energy store --
electrical power is still metered, but motor torque is never energy-limited and
the reserve never depletes.
Assist engages on pedal torque (`engage_torque_nm`), not crank rotation, so a
rider pressing a pedal at standstill is assisted like on torque-sensing
mid-drives. `stall_timeout_s` bounds sustained torque while |crank rpm| stays
below `spin_rpm`; the latch clears as soon as the shaft rotates again. `boost_s`
holds the last assist target briefly after pedal torque drops while the cranks
still turn forward. Reverse crank rotation is backdrive, not a lockout: sensed
rider torque still produces assist, which brakes the rollback through the
engaged drivetrain. `[drive.pedaling]` additionally accepts `mash_cadence_rpm`/
`mash_torque_nm` (the low-cadence effort ramp toward the isometric ceiling),
`effort_slew_nm_s` (rider force-development rate bound), and
the hill-hold reflex `rollback_brake`/`rollback_engage_mps`/
`rollback_release_mps`/`rollback_demand`, which physically grabs the wheel
brakes on sustained rollback without gating pedaling or assist intent.
`[drive.shifting]` accepts `cadence_smoothing_tau_s`, the filter constant on
measured crank cadence and wheel-required cadence used for shift decisions,
and `upshift_slip_limit_mps`, the rear tire slip speed above which upshifts
are refused.
The viewer synchronizes at at most 60 Hz without skipping integration steps.
Real-time capacity depends on the machine; `RTF` exposes slowdowns rather than
relabelling elapsed wall time as simulated time. An articulated rider can still
fall or roll backwards on a hill; preview mode does not stabilize its roots.

Runtime keys
that would change physical parameters or inject unrecorded pose/velocity changes
are blocked; camera, pause, reset, brakes and the explicitly ideal speed target
remain available. Restart with a new configuration to change material parameters.

Legacy output keeps schema 1 and its old physical meanings. Legacy pneumatic rim
crossings are still recorded raw; a separate impact-episode counter groups crossings
within explicit 1 ms/1 cm limits. Episode grouping does not hold or modify force.

## Validation and release evidence

```sh
uv run --locked pytest -q
uv run --locked python tools/validate_physics.py --out output/physics-validation \
  --dt 0.0005 0.00025 0.000125 --jobs 4 --require-lock
```

The registry executes independent rigs and full-bike cases; exceptions fail a
case instead of silently skipping it. Reports retain measured metrics and each
criterion, timestep and spatial refinement comparisons, actual versions and
source hashes. A subset of cases is not the complete mechanics gate. Source
changes during a run invalidate its gate.

`passed` describes numerical cases; `mechanics_gate_passed` additionally requires
the complete registry, required refinement grid and unchanged source.
`release_gate_passed` also requires the checked locked environment. The environment
check covers declared direct dependencies and Python, not a claim that every
transitive wheel has been independently authenticated. `--require-lock` makes an
environment mismatch fail the command even when numerical tests pass.

The supplied offline slim bundle used during development did not contain all
locked scientific-library wheels. Consult the accompanying verification report
for the actually tested versions. A successful run in that provided environment
must not be described as a successful `uv run --locked` run.

Measured calibration data require declared units, experiment conditions,
uncertainty, source/date and configuration identity. Fit and holdout are split by
whole experiments. Synthetic fixtures cannot establish V2 calibration. See
`validation/datasets.py`; no measurement data are fabricated by the test suite.
