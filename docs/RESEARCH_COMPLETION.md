# Longitudinal anti-wheelie plant: completion package (0.3.0)

This release completes the **longitudinal software-in-the-loop experiment path**,
not a certified controller or a measured digital twin. It extends the supplied
physical bicycle, articulated rider and compiled rough-road model without
introducing new third-party dependencies. Read `ANTI_WHEELIE.md` for the original
force paths and `PHYSICAL.md` for mechanical and electrical accounting.

## What changed

1. Radial tires may use explicit monotone force/deflection tables, with exact
   force-integral elastic energy, passive damping, provenance, pressure metadata
   and an applicability range. No extrapolation beyond the last table point is
   allowed. The original linear material and tangential brush remain supported.
2. Sensor acquisition has its own integer physics clock, independent of policy
   calls. Fixed biases, seeded sample dropout, latency, age and validity are
   modeled without leaking contact truth into observations.
3. A rider program drives posture/effort intent at the physics rate, independently
   of motor command transport. Quintic interpolation has zero first and second
   derivatives at keyframes. Existing bounded internal joint actuation realizes
   the requested motion; no qpos/qvel update or support-force shortcut occurs.
4. Research accepts a full physical TOML and an explicit road-mesh spacing. Only
   explicitly supplied physical CLI flags override the file. Force calculations
   use the actual compiled polyline, also exported with each experiment.
5. A separate model-applicability gate detects significant distinct supports,
   out-of-range loaded tires, excessive radial compression, catch-plane contact
   and excessive linkage closure error. It never clips or repairs a trajectory.
6. Each recording contains a self-contained, checksum-checked replay recipe.
   `bike-replay` rebuilds the same initial equilibrium, replays motor and brake
   requests and checks every saved observation/transition, applied command,
   event metric and final MuJoCo integration state.
7. The offline installer verifies both actual dependency versions and the
   project wheel's source content. Hash-pinned requirements include the rebuilt
   project wheel instead of silently retaining the original 0.2.0 package.

## Installation without network access

From the **bundle root**, using CPython 3.13 on Linux x86_64:

```bash
PYTHON=/path/to/python3.13 ./install.sh
source .venv/bin/activate
bike-research --help
bike-replay --help
```

This is the same **slim** distribution as the input: NumPy, SciPy **1.17.0**,
Matplotlib and Pytest must already be available to the chosen interpreter.
The original input README incorrectly required SciPy >=1.18. The installer now
reads requirements directly from `pyproject.toml`, fails before installation on
an incompatible base environment and never fetches packages. MuJoCo, GLFW and
their bundled dependencies come only from `wheelhouse/` and are hash verified.
The created environment contains `bikesim-install-report.json` with actual
versions and paths. A headless physics run does not need a graphical viewer.

After applying the source patch to a separate local checkout, the existing
configured environment may run the updated source directly:

```bash
PYTHONPATH=src uv run --offline --no-project --python /path/to/venv/bin/python \
  python -m bike_sim.cli.research --scenario flat --rider lumped \
  --duration 0.2 --motor-torque 0 --out output/smoke
```

To rebuild the wheel in an offline bundle after editing source:

```bash
PYTHON=/path/to/prepared/python tools/build_offline_wheel.sh ../wheelhouse
# Keep exactly one bike_sim wheel: remove only an obsolete project wheel.
uv run --offline --no-project --python /path/to/prepared/python \
  python tools/refresh_offline_wheels.py ..
../install.sh
```

## A reproducible rough-terrain rider experiment

Run from `project/` after activating the installed environment:

```bash
bike-research --scenario rough_uphill --rider articulated_planar \
  --initial-speed 3 --duration 3 --seed 17 \
  --rider-program examples/research/rider_shift.toml \
  --sensor-period 0.001 --sensor-delay 0.01 --sensor-dropout 0.02 \
  --out output/rider_shift
bike-replay output/rider_shift
```

Do not pass `--human-torque` or a separate `--posture` when the program owns
those inputs. `use_saddle=false` changes the internal support-allocation request;
it does not delete the saddle or force the pelvis away from it. Reaction delay
shifts rider intent in time, not the motor controller's command queue. Rider
keyframes describe **desired**, not guaranteed, body positions; feasibility,
loss of foot/grip support and actual CoM transfer belong to the plant response.

The force-curve example can be added with:

```bash
bike-research --scenario flat --rider lumped --duration 0.2 --motor-torque 0 \
  --physics-config examples/research/nonlinear_tires.toml \
  --out output/nonlinear
bike-replay output/nonlinear
```

`nonlinear_tires.toml` is deliberately labeled synthetic. Replace the ordinates,
pressure, damping and validity range with measured data before drawing real-tire
conclusions. Pressure is metadata for a particular curve, not an unvalidated
automatic stiffness multiplier. The simulator rejects a deflection outside the
table (`model_domain`); it never extends the final slope silently.

For mesh and timestep convergence, repeat a fixed seeded scenario at
`--road-resolution 0.005` and `0.0025`, and at `--dt 0.000125` and `0.0000625`.
The mesh spacing must divide the field extent; control, actuator and acquisition
periods must be integer multiples of dt. Compare onset, minimum front load,
clearance, slip and energy, not just whether the bicycle finishes.

## Policy interface and timing contract

`ResearchEnvironment.step(RideControl(...))` holds motor intent for one control
period. Motor transport delay is queued; brakes are immediate safety inputs.
Rider-program intent is evaluated independently each physics step. Acquisition
samples the **incoming solved state**, not an endpoint with forces from the
previous state. At a 1 ms acquisition period and a 10 ms policy period, a policy
call does not force an additional acquisition. Noise/dropout consume RNG only
on scheduled acquisitions; reading an observation does not change future noise.

`SensorObservation.valid` is false before a delayed measurement arrives, for an
invalid input sample, or when the latest delivered sample exceeds the configured
maximum age. Missing acquisitions retain the previous measurement; if all initial
acquisitions are lost, the placeholder is zero and invalid. `--ideal-sensors`
sets zero noise and latency; explicit bias/dropout and the chosen sample period
still apply. Bias is an experiment parameter, not random walk or a full sensor
hardware model.

Policies should use `env.observation` only. Ground-truth front/rear load,
clearance, road-relative pitch and contacts live in `ResearchStep.truth` and
telemetry for scoring. `examples/research/controller_loop.py` demonstrates the
wiring, but its ramp is **not** a completed anti-wheelie algorithm.

## Rejecting invalid training data

Use `transition.valid_for_learning` (and the corresponding summary field), not
only `reason == 'duration'`. Numerical validity and model validity are separate,
latched flags. An energy check may pass while contact geometry is out of scope.
A diagnostic run may continue with `--diagnostic-model-limits`, but it remains
invalid for training and the CLI returns nonzero. A configured finite table
cannot continue beyond its domain, even in diagnostic mode.

The single-equivalent-support tire is suitable only while that approximation
holds. Closely spaced smooth-road normals are combined; significant different
supports on a sharp rock/step are flagged and rejected. This release does **not**
pretend to solve deformable multi-contact tires or soil. Large impacts may
require smaller timesteps and different measured materials. Neither gate proves
accuracy; they prevent several known kinds of unusable data being labeled valid.

## Saved state, replay and A/B comparisons

In addition to telemetry and exact road vertices, recordings contain:

- `states.npz`: initial/final `mjSTATE_INTEGRATION`, without pickle;
- `transitions.jsonl`: all policy-boundary observations, truth and outcomes;
- `replay.json`: fixed-file checksums, field specification and initial placement.

Replay requires the same source hash and exact Python/MuJoCo/NumPy/SciPy versions.
It reconstructs tire shear, rider contacts, drivetrain and equilibrium rather
than overwriting qpos over uninitialized force-component state. It refuses
changed or incomplete recordings and compares numerical states with 1e-9
absolute/relative tolerance. A failed force solve can be preserved for diagnosis
but is not claimed to be a fully replayable successful transition. Successful
replay is distinct from physical/numerical validity.

`rebuild_environment(path)` in `bike_sim.sim.research.replay` returns a fresh
checked copy at the recorded start for running a different policy. Do not compare
controllers with different terrain seeds or rider programs unintentionally.
Exact replay of a separately injected custom suspension/runtime object is not
promised; reconstruction rejects a mismatched resolved configuration.

## Evidence and boundaries

Actual commands and results for this delivery are in
`verification/completion/DELIVERY_VERIFICATION.json`. The existing older
`verification/` files are retained from the input and are **not** results of this
release. Test pass counts do not establish experimentally measured accuracy.

The plane of motion is longitudinal X-Z: there is no lateral balance, roll,
steering control or sideways tire force. Defaults are synthetic. A deployable
anti-wheelie controller still needs calibrated masses/CoM, rider behavior,
suspension/tire curves, motor dynamics and sensors, plus independent bench and
controlled real-bicycle validation. No universal safe torque is asserted.
