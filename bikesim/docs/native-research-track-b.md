# Native research: Track B source delivery

Historical Track B scope. The later [Track C source audit](native-visual-track-c.md)
and [ride usage](RIDE.md) supersede the frontend/replay work marked future below;
neither source delivery establishes executable acceptance.

Status: B1-B3 implementation and regression sources are supplied. They were not
compiled, imported, executed or tested during this delivery. A4 verification was
already pending in the supplied audit. This is not a verified release artifact.

## Backend selection

`bike_sim.sim.research.configuration.build_environment` accepts `backend="python"`
(the unchanged default) or `backend="native"`. It retains the existing named
recipe arguments and now also forwards `rider_behavior`. There is no automatic
fallback to Python when native setup fails.

An existing fresh Python research setup can instead be captured explicitly:

```python
from bike_sim.native.research import create_native_research

# reference is a ResearchEnvironment at its initial t=0 external boundary.
# The factory captures its setup into an independent native owner.
env = create_native_research(reference)
```

Only the existing supported A-track physical topology is accepted: articulated
spindle/pin/connect control, track-material compliant tires and the supported
ideal mid-drive configuration. This work does not add another integrator, tire
model, support topology, speed controller or physical stabilizer.

Artifact selection continues to use an absolute `BIKE_NATIVE_BUILD_PATH`, or the
existing checkout default `native/build/release`. Construction verifies that the
loaded extension's compiled source fingerprint matches the current native
source tree. A stale extension must not be used for the new API.

No native frontend command-line switch or viewer is added here. Those are
separate Track C work. The existing research CLI still constructs Python by
default; explicit backend selection is at the factory/API boundary.

## External control transactions

`env.step(control, front_brake_demand=..., rear_brake_demand=...)` is synchronous.
The equivalent incremental operations are:

```python
env.begin_control(control, front_brake_demand=0., rear_brake_demand=0.)
result = env.advance_control(wall_budget_s=0.)
# A zero budget yields without advancing or closing a physical period.
assert result is None
result = env.advance_control(wall_budget_s=None)
```

Only one window may be active. A budget yield neither re-enqueues the command nor
publishes another observation/transition, closes an accounting period, or calls a
Python policy/rider callback. A completed native result remains available until
the adapter has boxed it successfully and acknowledged it. Retrying publication
after a Python boxing exception does not repeat physics.

`PolicySession.begin_advance()` calls the motor policy once;
`advance_pending(wall_budget_s=...)` resumes the same window. `advance()` uses this
same lifecycle synchronously. Stop, pause, brake changes and reset requests are
queued until the current external boundary. Stopping an already ended episode
preserves its original reason.

A custom `RiderBehavior` runs at the external boundary, not in the GIL-released
loop. Native construction deep-copies its already initialized state so a paired
Python environment does not share mutable callback state. Such callback state
must support `copy.deepcopy`; resource-owning callbacks can provide an appropriate
`__deepcopy__` implementation. Reset resets the owned callback once with the new
seed.

## Sensor determinism

NumPy generates an owned float64 tape before native advancement. Each row uses
three acceleration draws, one scalar gyro draw, three encoder draws and two
torque draws in the original four `Generator.normal` calls. A separately spawned
stream supplies dropout uniforms. Zero deviations and disabled IMU channels do
not skip normal draws.

The native pipeline owns the delayed queue, dropout decisions, clocks and cursor.
Importing the Python t=0 setup starts at cursor one: startup noise is not applied
twice. There are `1 + floor(max_steps / sensor_steps)` rows, including the spare
final acquisition. Exhaustion is an explicit error, not a seed change.

## Views, recording and provenance

Native `env.sim` is a read-only view: time, step count, position, copied setup
metadata and owned snapshots. It exposes neither `mjModel`, `mjData` nor a plant
`step` method. Diagnostic maps and snapshot arrays are owned/read-only. Calling
`env.snapshot()` or `env.sim.snapshot()` does not flush a physical tail.

C++ records selected numeric columns and detailed intervals while accounting,
quality checks and metrics still examine every completed physical interval.
`env.save(directory, overwrite=False)` uses the same exporter for both backends.
It requires an external boundary, stages the complete output, hashes the final
file bytes, and publishes `replay.json` last. A failure during staging leaves an
older recording untouched; an interrupted publication cannot leave a stale
manifest describing mixed files.

New manifests use schema 2 with exactly these execution fields: backend, runtime
schema, Python source hash, native source hash, extension hash, loaded MuJoCo
version, loaded MuJoCo library hash, and compiled build context. Native-only
fields are null in Python recordings. Source hashes use stable relative paths.
Build context is compiled into the selected extension, not taken from an editable
sidecar. Save compares current execution identity with the startup identity and
records any change without relabelling the run.

Both referenced rider parameter files are bundled under fixed names:
`rider_joint_envelope.json` and `rider_joint_strength.json`. Validation checks the
exact schema-specific checksum set, execution shape, bundle declarations and
content fingerprints. It does not load a recorded extension path or policy
factory. Schema 1 retains its original file set. A legacy recipe that requires
an unbundled external parameter file is not reconstructed from that untrusted
path.

Checked native replay is intentionally not implemented in B. Rebuilding a native
schema-2 recording reports the missing C3 replay path rather than silently
reconstructing a Python backend.

## Verification handoff

The new sources in `tests/reference` cover the frozen pre-refactor sensor oracle,
noise tapes, native sensor delivery, program interpolation, paired rollouts,
policy/operator lifecycle, publication retry, read-only ownership, schema-1/2
validation, export parity, tampering and changed identity. The fatal-engine probe
is isolated in a subprocess by the test source.

None of these tests has been run for this delivery. In particular, the one-second
`atol=rtol=1e-9` test is a future acceptance gate, not a measured result. The full
checkout must supply its missing root packaging/lock files, tools and original
Python tests before the documented upstream native/full gates can be performed.
See the dated delivery audit for scope and outstanding checks.
