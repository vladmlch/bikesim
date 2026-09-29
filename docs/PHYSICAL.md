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
chain/freehub, motor losses and postural control are **synthetic**. Passing
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
| `crank_effort` | Human crank moment and mid-drive moment, transmitted through the chain, cassette and freehub. |
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

The viewer and physical plots consume these same immutable samples. Runtime keys
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
