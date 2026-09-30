# Anti-wheelie research plant

This extension supplies the **simulation and experiment interface**, not a trained
anti-wheelie controller. It is a longitudinal X-Z multibody plant with pitch,
articulated rider, suspension, chain/freehub, compliant tyres, road geometry and
causal sensor inputs. The existing legacy visual/speed-controlled modes are kept
for compatibility; they are not the research plant.

## Start from the supplied offline archive

The archive is the original **SLIM** distribution with updated source and project
wheel. It does not download packages. It reuses CPython 3.13, NumPy >=1.24,
SciPy **1.17.0**, Matplotlib >=3.7 and pytest >=7 from the selected interpreter.
The rest of its dependencies remain in `wheelhouse/`. The previous installer
incorrectly required SciPy >=1.18 even though the source pinned 1.17.0; this is
now consistent. The installer can inherit the scientific stack of an active
virtual environment, not just the base interpreter's site-packages.

From the bundle root:

```bash
PYTHON=/path/to/existing/python3.13 ./install.sh
source .venv/bin/activate
cd project
export PYTHONPATH="$PWD/src"

uv run --offline --no-project --python ../.venv/bin/python \
  python -m bike_sim.cli.research --scenario rough_uphill \
  --rider articulated_planar --duration 3 --motor-torque 80 --out output/rough
```

`bike-research` is also installed as a console command. `PYTHONPATH=src` makes
local edits authoritative without downloading/reinstalling anything. To rebuild
the project wheel after editing, use `PYTHON=/path/to/python tools/build_offline_wheel.sh`;
this requires an existing setuptools >=70 (or an older setuptools plus wheel).
No new runtime dependency was introduced.

A repository patched outside this bundle can use its existing compatible Python:

```bash
PYTHONPATH=src uv run --offline --no-project --python .venv/bin/python \
  python -m bike_sim.cli.research --scenario flat --rider lumped --duration 1
```

Choose a new `--out` directory for every run. Replacing files requires explicit
`--overwrite`. The headless research command does not require a viewer or display.
Tests that exercise rendering need a working EGL/OSMesa backend, for example
`MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg`.

## What is physically controlled

`RideSimulation.step(..., control=RideControl(...))` accepts an immutable command.
It is supported only by physical mode. Direct motor/human requests require
`crank_effort` or `articulated_effort`, not coast or an ideal speed controller.

| Command | Meaning |
| --- | --- |
| `motor_torque_nm=None` | Keep the existing torque/cadence pedelec-assist demand. |
| `motor_torque_nm=number` | External nonnegative **crank-side** motor torque setpoint. It can start at zero cadence without human demand. |
| `motor_limit_nm=number` | A safety ceiling after motor lag/slew, before the battery budget. Zero cuts delivered torque immediately **once the command reaches the plant**. |
| `human_torque_nm=number` | Mean human torque request. With the articulated rider it becomes bounded limb effort, not a hidden crank actuator. |
| `posture=RiderPosture(...)` | Bounded torso/pelvis/hip goals realized through internal joint motors and actual contacts. |
| `rider_enabled=False` | Disable rider joint effort; physical mass, limbs and contacts remain. |

Motor requests retain the configured torque/power curve, speed cut-off/taper,
reverse-cadence inhibition, lag, slew, brake priority and battery-energy limits.
A zero *request* normally ramps down through lag; a zero *limit* cuts immediately.
Braking overrides motor torque. The torque is not an ideal rear-wheel force: it
passes through the crank, compliant chain and passive freehub, with frame/carrier
reaction. This models a crank-coupled mid-drive, not every possible internal
motor/chainring clutch architecture. Defaults (80 Nm, 500 W) are synthetic,
not a measured motor or a legal class.

Both initial holding brakes default to one in `bike-research`, allowing static
initialization on a slope. They are released at episode time zero. They are not
an invisible riding aid. The initial speed is assigned along the supported road
tangent and wheel rolling velocities are initialized consistently. Commands never
teleport a running bike or rider. Static initialization is the only stage allowed
to adjust coordinates/reset velocities to find equilibrium.

## Controller interface and time semantics

```python
from bike_sim.cli.research import parser, make_environment
from bike_sim.sim.ride.control import RideControl

args = parser().parse_args([
    '--scenario', 'rough_uphill', '--duration', '3', '--seed', '17',
])
env = make_environment(args)
observation = env.observation
while not env.done:
    # Demonstration input only. A policy can replace this expression using
    # observation, without access to env.sim or result.truth.
    requested_nm = 80.0 if observation.valid else 0.0
    result = env.step(RideControl(motor_torque_nm=requested_nm,
                                 motor_limit_nm=80.0,
                                 human_torque_nm=0.0))
    observation = result.observation
    if not result.numerically_valid:
        break  # Discard invalid dynamics; do not train on this transition.
env.save('output/my_policy')
```

The independent example `examples/research/controller_loop.py` is executable.
`env.reset(seed=17)` resets the complete plant, contacts, drivetrain, battery,
recorder, command queue, metrics and sensor RNG. Same configuration, command
sequence, seed and numerical environment reproduce the episode; equality across
different engine/BLAS versions is not promised.

Defaults are physics `dt=0.0005 s` (2 kHz) for the ideal transmissions and
`0.000125 s` (8 kHz) for `--transmission elastic_chain` (see below), control
period `0.01 s` (100 Hz), actuator transport delay `0.005 s`, sensor delay `0.01 s`. Control period, actuator
delay and duration must be integer multiples of the physics timestep. Commands
are sample-held and queued by physics-step index, avoiding floating-point event
drift. A final partial control interval stops exactly at the physics-step budget.
The initial pending command is motor zero/human zero. Brakes supplied to `env.step`
are immediate out-of-band safety inputs, not delayed transport commands.

Force samples and raw sensors describe the **incoming state of the last physics
interval**, not the integrated endpoint. With ideal sensors the newest observation
at t=0.01 s has source time 0.009875 s at the default dt. Transport delay adds to
that age. Every observation includes delivery time, source time and a validity
flag; the first delayed observations are invalid warm-up samples. Delay selection
is causal, repeated reads do not redraw noise, and the delivery clock cannot go
backwards.

Policy inputs deliberately contain only body-frame proper acceleration, nose-up
pitch rate, signed front/rear wheel and crank encoders, and motor/human torque
sensors. Proper acceleration already includes the engine accelerometer convention;
gravity is not subtracted again. There is no perfect pitch angle, terrain gradient,
true chassis speed, normal load, clearance or CoM in `SensorObservation`.
Noise is independently seeded Gaussian noise with configurable standard deviations;
this is a synthetic sensor model, without calibrated bias drift or temperature
effects. `--ideal-sensors` removes both noise and sensor transport delay.

`ResearchStep.truth` and the trace's `truth_*` columns are privileged evaluation
outputs. Keep them out of the policy, state estimator and training features unless
you explicitly label an oracle experiment. Evaluation callbacks may use them for
labels and metrics.

## Rider posture

`articulated_planar` has an independent pelvis and articulated torso, arms and
legs. Saddle/feet are unilateral physical supports; feet have finite platforms
and frictional slip; hands are finite, releasable grips. Ground/crash contacts and
joint torque/speed/power limits remain active. The lumped rider is a useful
controlled rigid-mass baseline, not a dynamically moving human.

`RiderPosture` supplies offsets from the neutral posture. Positive torso/pelvis
pitch means forward lean (engine +Y); `pelvis_offset_m=(forward, upward)` is in the
bike frame. `None` retains the original support-following inverse kinematics;
explicit `(0, 0)` requests the nominal hip position. `use_saddle=False` asks the
controller to stop allocating saddle support; it **does not remove the saddle
collision or guarantee lift-off**. Unreachable targets saturate limb goals and
actuators. A rider can fail to reach the target, lose support, or crash.

The command-line `--posture forward|crouched|standing` uses a smooth transition
between t=0.5 and 1.0 s. A standing target raises the requested hip position by
0.16 m; actual displacement is measured in the acceptance run rather than assumed.
These are low-level posture targets, not a human balance/reflex/perception model.
`--rider-mass`, `--rider-height`, drive/gearing parameters and the source-level
`BikeSpecs`/`RiderSpecs` interfaces support parameter sweeps without extra packages.

## Road and tyre material

Six repeatable authored scenarios are available: `flat`, `uphill`,
`rough_uphill`, `crest`, `low_grip`, and `step_up`. The supplied TOML examples use
seed 17; `--track-file` preserves the file's authored geometry/seeds. On generated
scenarios `--seed` sets road and sensor reproducibility. The same seed with a
changed generation specification is a different road.

Independent `GradeProfile` specifies station and **dimensionless grade** pairs.
Grade is interpolated linearly; elevation is its exact continuous integral and
is added to ordinary bumps/roughness/ledges. Thus roughness no longer competes
with an overlapping 'slope obstacle'. A grade of 0.22 means 22 percent, not 22
degrees. Obstacle elevation/datum-shift semantics remain unchanged.

```toml
name = "mixed_climb"
length_m = 20.0
surface = "hardpack"

[grade_profile]
knots = [[0.0, 0.0], [4.0, 0.0], [8.0, 0.22], [20.0, 0.22]]

[[surface_sections]]
start_m = 10.0
end_m = 13.0
surface = "wet"

[[obstacles]]
type = "bump"
start_m = 9.0
height_m = 0.06
bump_length_m = 0.7
```

Material zones are validated nonoverlapping half-open intervals [start, end).
The base material applies outside zones. Physical research tyres use
`surface_mode='track'`: the actual contact station and slip select the surface's
Stribeck friction, capped by the tyre's configured mu. Front and rear contacts
can therefore have different friction at the same time. The older
`surface_mode='configured'` constant-mu path remains available for old tests.
The native-reference tyre backend rejects track-material mode instead of silently
ignoring it. Legacy pneumatic tyres also use zoned `SurfaceMap`.

The compliant model still uses its configured radial/brush stiffness and damping.
Road names do not invent measured soil stiffness, mud sinkage or rut deformation.
The road is a rigid heightfield extruded across Y. Force queries use its exact
compiled longitudinal polyline, not a separate smoother road. Multiple significant
candidate supports at sharp geometry are flagged as `multi_support`; one equivalent
support is applied per wheel. This limitation matters for rocks/vertical faces.
Authored tracks, compiled vertices, friction selection and configuration hashes
are saved so the geometry used by a run is inspectable.

## Detecting front lift without mistaking a climb for wheelie

Ground truth uses raw working-road normal load and circle-to-compiled-road
clearance. The reference pitch is the axle line relative to the local road chord,
including the different 29/27.5-inch wheel radii. Absolute frame pitch alone is
never a wheelie label.

Contact states distinguish two-wheel support, front unloading, front lift,
rear lift/unloading, both-wheel flight, and zero-force near-road unsupported
states. A wheelie candidate requires rear support, front load at most 5 N,
front clearance above 0.01 m and relative pitch above 0.035 rad. Confirmation
requires 0.02 s persistence, with exit hysteresis; loss of rear support clears
confirmation immediately. Thresholds are explicit synthetic event definitions,
not calibrated detection probabilities.

A terrain crest or a rider impulse can also produce rear-supported front lift.
The label is not proof of motor causation. Use paired runs with identical initial
conditions/terrain/seed and different torque policies. Both-wheel flight has its
own label and metric, rather than being misreported as a sustained wheelie.
Metrics are integrated every physics interval independently of CSV decimation.

## Numerical quality and timestep refinement

Do not judge the model by a plausible-looking chassis trajectory alone. The
34/51 low gear exposed an explicit chain/freehub oscillation at 0.5 ms. Cassette
speed and inferred dissipative loss became very large even while the bicycle
appeared to ride normally. That is why `elastic_chain` keeps 0.125 ms; the ideal
transmissions have no chain spring and use a validated 0.5 ms (see "Transmission
modes"). Legacy mode retains its old timestep. Static equilibrium refinement starts at 0.1 s for a lumped rider and at 3 s
for an articulated rider, whose unilateral supports must first settle. The
**same actual acceleration-residual acceptance test** remains in force; no
riding force or mass is changed to speed initialization.

Every research interval checks the mechanical energy residual, normalized by
initial kinetic/elastic energy and absolute source/road/constraint work. Arbitrary
gravitational datum and inferred losses are not used to inflate the denominator.
The default maximum ratio is 0.05. Electrical energy is checked independently.
A failed check returns `reason='numerical_quality'`, `truncated=True` and
`numerically_valid=False`, and preserves the failed run. It is not a valid
training transition or a successful controller demonstration. Nonfinite dynamics
and engine failures still raise exceptions and retain an explicit error outcome.

A small residual is necessary but not sufficient for accuracy. Refine the timestep
and contact resolution, compare event onset, load, slip, clearance and work, and
check material applicability and multi-support flags. `--energy-tolerance` is an
explicit diagnostic budget, not a tool for relabelling a bad run as correct.
Stronger impacts, changed stiffness/inertia/gearing or rider settings may require
a smaller timestep than the default. The acceptance tool tests stable fine steps
and deliberately verifies rejection of the unstable coarse case.

```bash
PYTHONPATH=src MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg \
  uv run --offline --no-project --python ../.venv/bin/python \
  python tools/validate_antiwheelie.py --cases wheelie limited reject_coarse \
  --dt 0.000125 0.0000625 --jobs 2 --out verification/my_timestep_check

PYTHONPATH=src MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  uv run --offline --no-project --python ../.venv/bin/python python -m pytest -q
```

The deliberately overpowered 300 Nm / 10 kW stress motor in the acceptance suite
is a plant test, not a suggested e-bike specification. A 60 Nm cap is an A/B
control input, not a universal safe torque. The suite also exercises articulated
riding/standing, wet material transitions, a crest, and incline-vs-wheelie labels.
The delivered archive's `verification/antiwheelie*` directories contain actual
run data and acceptance results; `verification/DELIVERY_VERIFICATION.json`
summarizes executed checks.

## Refocused plant: rough-terrain front-load margin

The plant is refocused on one question: how does a mid-drive e-bike with an
articulated rider keep its front wheel loaded while climbing rough, procedurally
generated terrain under a torque request? The deliverable is infrastructure and a
Python policy hook, `policy(observation, demand_nm) -> RideControl`
(`examples/research/controller_loop.py`). The anti-wheelie algorithm is the
user's; the demo policies in `bike_sim.sim.research.policies` are plumbing, not
solutions. The plant stays planar (X-Z): no roll, yaw, steering or hub motor.

### Transmission modes

`--transmission` selects how crank torque reaches the rear wheel:

| Value | Role |
| --- | --- |
| `ideal_mid_drive` (default) | One-way tendon constraint, no chain/freehub spring. Cheap; omits chain-growth/suspension coupling (`omits_suspension_coupling`). Validated at `dt = 0.0005 s`. |
| `geometric_ideal_mid_drive` | **Experimental** (`transmission_reference_status: experimental_geometric_reduction`): the same tendon linearized from the chain geometry. |
| `elastic_chain` | Frozen reference model for A/B comparison; default `dt = 0.000125 s`. `--chain-stiffness`/`--freehub-stiffness` apply only here and raise `ValueError` otherwise. |

`--dt` defaults to `0.0005` (ideal modes) or `0.000125` (`elastic_chain`).
`tools/validate_antiwheelie.py --transmission ideal_mid_drive --dt ...` passes all of
`wheelie limited rough crest low_grip incline` at 0.125, 0.25 and 0.5 ms with a
maximum energy residual ratio of 0.0009 (`verification/dt_sweep_ideal/report.json`).
1 ms is not a candidate: `SimulationPhysicsConfig` refuses it
(`closure_time_constant_s` must be at least `2*dt`). The `reject_coarse` acceptance
case (0.5 ms must be rejected) tests the chain's instability, so
`validate_antiwheelie.py` always runs it on `elastic_chain`, whatever
`--transmission` says.

**A/B parity is not fully met.** `tools/compare_transmissions.py` runs identical
episodes (seed 17, 5 s, 3 m/s, 80 N*m) on `elastic_chain` and each candidate and
checks wheelie onset (50 ms), minimum front load (15 %), peak shock stroke (10 %)
and mean speed (5 %):

| Scenario | `ideal_mid_drive` @ 0.5 ms | Note |
| --- | --- | --- |
| `flat` | pass | |
| `step_up` | not comparable | both transmissions end in `model_violation`; a pair with a truncated run is never a pass |
| `uphill`, `rough_uphill` | **fail**: minimum front load 24-31 % lower, front-load fraction 22-25 % lower (tolerance 15 %) | shock stroke and speed agree (<= 1.1 %) |
| `crest` | not comparable | the chain reference itself ends in `model_violation` |

The climb deviation is the same at 0.125, 0.25 and 0.5 ms (31/31/30 % on
`uphill`), so it comes from the missing chain/suspension coupling, not the
timestep. The ideal plant has *less* front-load margin on climbs than the chain
reference, i.e. it errs toward earlier front unloading. `geometric_ideal_mid_drive`
does worse (energy gate at 0.5 ms; even at 0.125 ms the rider falls on `uphill`),
so it is not a replacement. Reports are in `verification/ab_transmission*/`
(re-derived with `--reuse`; their `mean_speed_mps` figures predate the fix that
measures distance from the start position instead of absolute x, so treat the
speed check as indicative). `compare_transmissions.py` runs the reference at
`0.000125 s` unless `--reference-dt` says otherwise.
Whether to ship `ideal_mid_drive` or `elastic_chain` as the research default is
the owner's call; `--transmission elastic_chain` restores the reference.

### Margin metrics, episodes and `episode_metrics.json`

`WheelieTracker` reports, besides the existing counters, a continuous margin
`front_load_fraction_min` / `front_load_fraction_mean` (front normal load over
total, counted only while the rear wheel carries load; 0 means unloaded),
`max_pitch_rate_up_rad_s`, and `wheelie_episode_records`: one record per candidate
bout `{start_s, end_s, confirmed, max_relative_pitch_rad, max_front_clearance_m,
min_front_load_n, onset}`. `onset` holds the state when the bout began
(`delivered_motor_nm`, `applied_motor_nm`, `road_pitch_rad`, `pitch_rate_up_rad_s`,
`speed_mps`), so a lift can be attributed to torque, grade or terrain. Terrain
micro-lifts are acceptable; crash outcomes (`loop_out`, `endo`,
`rider_ground_contact`, ...) end the episode.

Every run also writes `episode_metrics.json`: `outcome`, `duration_s`,
`progress_m` (distance travelled from the start position), `mean_speed_mps`, `finish_time_s`, `torque_delivered_nms` and
`torque_requested_nms` (integrals of the solved and commanded motor torque),
`motor_pass_fraction` (their ratio; `null` when the policy never commanded a
torque), `loop_out`, `endo`, `max_shock_stroke_m`, `max_fork_travel_m` (peaks at
physics rate), `numerically_valid`, `max_energy_residual_ratio`, `model_status`
(`model_valid`, `first_model_violation`, counts) and the full `wheelie` block.
These are privileged evaluation outputs, like `truth_*`.

### Generated terrain and the eval set

`--scenario generated [--gen-spec spec.toml] --seed N` draws a seeded track from
`TerrainGenSpec` (defaults in `examples/research/gen_spec.toml`): 60-120 m,
uphill grade 0-25 % with an optional stretch up to 30 %, roughness sections,
bumps and root sections of 2-10 cm, wet/loose friction zones, and a flat obstacle-free
lead-in (4 m) and lead-out (5 m). `(spec, seed)` fully determines the track, and
the spec hash is stored in the track description. `--track-file` still wins over
`--scenario`.

Vertical faces (`SquareEdge`, `Drop`) are opt-in (`feature_types`), and bump
lengths are floored so the crest radius stays above the wheel: sharper geometry
makes the tyre touch two places and the run ends in `model_violation`
(`front:multi_support`). A smoke run of the first eval set ended 4 of 10 tracks that way
before the ranges were narrowed; the committed set then ran 15 s with no
`model_violation` (7 reached `duration`, 3 ended `crash:rider_ground_contact`
under the non-solution `passthrough` policy).

```bash
PYTHONPATH=src uv run python tools/generate_eval_set.py \
  --gen-spec examples/research/gen_spec.toml --n 10 --seed0 1000 --out examples/research/eval
```

`examples/research/eval/` holds the committed set (`eval_00..09.toml`, loadable
with `--track-file`) and `manifest.json` (seeds, spec hash, date). Regenerating
refuses to overwrite a non-empty directory, because a policy is compared across
revisions on the same tracks. `examples/research/long_climb.toml` is a 118 m
track for 10-30 s episodes; use `--record-decimation 80` or more for long runs.

### Demand channel, rider and sensors

* **Demand.** `--demand NM` or `--demand-file demand.toml` (`[[keyframes]]` with
  `time_s`, `torque_nm`, quintic smoothstep between knots) gives the policy the
  torque "the bike wants" as `env.demand_nm` (`None` without a program). It is an
  advisory command echo, not a sensor and not enforced: metrics compare what was
  delivered with what the policy actually requested. The CLI's own loop passes the
  demand through as the motor setpoint. `ResearchStep.demand_nm` and the
  `trace.csv` column `demand_nm` hold the value the policy saw for that command.
* **Random rider.** `--rider-random [--rider-seed N]` samples mass (60-100 kg),
  height (1.60-1.85 m, inside the frame's seatpost window), torso lean, pelvis
  pitch and pedalling effort once per episode (`rider_random.py`). The rider
  program then owns posture and human effort, so the policy must leave
  `posture` and `human_torque_nm` as `None`. The program is saved in
  `summary.json` (`research.rider_program`).
* **Reactive-rider hook.** `ResearchEnvironment(..., rider_behavior=...)` calls
  `act(time_s, RiderSignals) -> RiderPosture | None` once per control step when the
  policy does not set a posture. `RiderSignals` carries pitch rate, proper
  acceleration and saddle/bar/pedal loads only. It is an interface: there is no
  reflex or balance model, and it is exclusive with a rider program.
* **No-IMU variant.** `SensorConfig(imu_enabled=False)` reports exact zeros for
  proper acceleration and pitch rate (not noise) and keeps the encoder and torque
  noise streams unchanged for a given seed. There is no CLI flag yet.

### Batch evaluation

```bash
PYTHONPATH=src uv run python tools/research_batch.py \
  --spec examples/research/batch_demo.toml --jobs 4 --out output/batch_demo
```

A `[grid]` of tracks and/or scenarios, seeds, `demand_nm` (or `demand_files`),
`policy` (`passthrough`, `zero`, `fixed_limit_40`; a list makes it an axis),
`transmission`, `dt`, `duration` and verbatim `extra_args` is expanded into runs.
Each run gets its own directory with the usual artifacts;
`batch_report.json`/`.csv` aggregate `episode_metrics`. `outcome_counts` counts
`model_violation` and `numerical_quality` separately from `crash:*`, `finish` and
`duration`, so out-of-scope runs are not read as policy failures. A failing run is
an `error` record and the exit code is 1.

## Saved evidence and outcomes

Each run saves `summary.json`, requested/applied command streams, timestamped
sensor observations, `trace.csv`, decimated physical telemetry/intervals,
`track.toml` and exact `terrain_vertices.npy`. Summary metadata includes source,
configuration, terrain and command hashes, resolved parameters, versions, seed,
energy accounts and whether source files changed during execution. Do not merge
results with different hashes as though they were one identical experiment.
Physical interval files explicitly use incoming-state timing.

`duration` is a normal time-budget truncation; `finish` is a course completion;
crash outcomes are actual terminated episodes; `numerical_quality` is invalid
dynamics. `crash:loop_out` (backward, nose-up past the pitch limit) and
`crash:endo` (forward) replace the old `crash:pitch_over`; other crash causes
(`handlebar_contact`, `rider_ground_contact`, `catch_plane_contact`) are
unchanged. `model_violation` and `numerical_quality` are *truncations*, not
crashes: the run left the model's declared scope (or its energy gate), so it
says nothing about the bike. `ExperimentConfig.stop_on_model_violation`
(default on) enables the former; never switch it off to make a run "work".
The command-line tool returns nonzero for crash/invalid/error outcomes.
A validation *case* may intentionally expect wheelie or numerical rejection; its
acceptance result documents that expectation rather than calling the ride safe.

## Scope before a real bicycle

The intended use is software-in-the-loop development of longitudinal torque
policies, estimators, failure cases and test infrastructure. It is not a verified
real-bicycle safety predictor. There is no roll/yaw/steering/lateral balance,
deformable soil, flexible frame, independently calibrated human reflex controller,
or measured tyre/motor/suspension/sensor parameter set. A simple moving rider is
not a physiological model. Match hardware measurements, actuator authority,
sensor placement, delays and uncertainty before hardware-in-the-loop testing;
a simulation-only policy must not be treated as safe for a person to ride.

For the existing interactive viewer (requires a display), the supplied file can
be used with:

```bash
bike-ride --physics-config examples/research/viewer_physics.toml \
  --track examples/research/rough_uphill.toml --rider articulated_planar
```

This viewer example uses the original human-driven assist path. It is not replay
of a saved external torque-policy episode and does not provide the research
wrapper's numerical-quality termination gate. Use the headless research interface
for policy evaluation and recorded acceptance results.

The standalone `GradeProfile` extrapolates endpoint grade. A `TrackSpec` clips
that evaluation to the authored track domain: compiled heightfield runout is
flat at the final elevation, consistent with existing obstacle tracks. Use a
zero-grade lead-out before the finish to make the slope continuous there.
