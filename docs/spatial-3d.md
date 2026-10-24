# Native spatial 3D workflows

**Status: implemented in source; verification pending V08.** These commands,
native reports, install rules and regression sources were authored in W45–W49.
No configure, build, compilation, test, application, installation or GUI was
run for this delivery. The physics qualification belongs to V07 and the
performance measurements to V09. Source availability is not evidence of a
successful ride or a qualified calibration.

The existing `bike-ride`, `bike-research`, `bike-replay`, `bike-recording` and
`bike-research-sweep` executables own these paths. With no spatial flag the
authored physics TOML retains its mode; an empty configuration remains planar.
`--spatial 3d` explicitly selects spatial physics and `--spatial planar`
explicitly removes the extension. The latter still requires a valid planar
tyre backend in the resulting physics configuration.

## Capability and evidence

| Capability | Source status | Required verification |
| --- | --- | --- |
| Planar defaults and schema1 reader | Existing path retained; regression authored | V08 compatibility |
| Constrained spatial root with steering off | Implemented in source | V07 physics; V08 CLI |
| Free spatial root, `roll-heading-v1` steering | Implemented in source | V07 balance/domain; V08 application |
| Native grid and terrain genspec requests | Implemented in source | V04 terrain; V08 installed inputs |
| Profile extrusion request | Recognized, capability rejection with a named reason | C10/V04 material-registry limitation |
| Sensor schema2, `noise2`, policy ABI2 | Implemented in source | V07 causal/sensor gates; V08 application |
| Pose-aware camera, render copy and HUD | Implemented in source | V08 real-display smoke and interaction |
| Container2, recipe4, offline reports and exports | Implemented in source | V07 recording; V08 relocated replay/report |
| Process sweep and canonical job identities | Implemented in source | V08 workers 1/2/4 and crash isolation |
| `spatial-fast-v1` realtime targets | Targets only; no measurement | V09 |

The command registry is
[`examples_manifest.json`](../examples/spatial/examples_manifest.json).
`spatial.documented_examples` parses every producer through the actual CLI
and resolves the typed request without preparing a model, stepping, creating
a viewer or writing output. Recording and sweep commands use their actual
cold parsers; analysis commands reference the producing example's mode,
root, schema and duration. Process/GUI acceptance is a separate gate.

## Installed inputs and common settings

Put the installed `bin` directory on your shell's `PATH`. In the commands
below, `EXAMPLES` is the absolute path to the installed
`share/bike_sim/examples/spatial` directory, `OUT` is a fresh output root, and
`RUN` is the **sealed run directory printed by the noise2 research command**.
The output root itself is not a recording. These are shell substitutions for
the manifest's `${EXAMPLES}`, `${OUT}` and `${RUN}` placeholders.

Both spatial physics files choose physical `crank_effort`, `pneumatic_3d`
`fast`, `dt=0.00125 s`, physical control period `0.005 s`, initial longitudinal
speed `2 m/s`, no initial brake demand, no pitch assistance, no drag and no
rolling resistance. The examples use the lumped rider. The free example uses
`roll-heading-v1`, a `40 N m` steering cap, a `60 degree` steering range and
`0.1 s` reaction delay. The constrained file explicitly selects steering off.
These are controlled diagnostic cases, not a road-bike calibration.

The terrain genspecs are `60 m` long, have a `1.5 m` half-width, `dx=0.005 m`,
`dy=0.01 m`, seed `0`, flat lead-in `4 m` and lead-out `5 m`. The cross-slope
has a `5 degree` bank after a `1 m` transition. The material uses the installed
`mtb-provisional-v1` lateral coefficients. The terrain seed is authored in the
genspec; a research seed selects research noise and does not silently replace
that terrain seed. The corridor half-width is `1.5 m` and the capsize roll
threshold is `pi/3 rad` in both physics files.

The `.terrain3d.manifest.json` file is explicitly pending: its binary hash is
null. V08 must generate/review/pin the binary asset. The commands use the
installed genspec directly through the native terrain owner, so they do not
pretend that a generated asset already exists.

## 1. Planar compatibility

```sh
bike-ride --physics physical --duration 1 --headless --no-plots
```

This bounded control uses the existing planar physical request, planar root
and schema1. Recording is off. The expected successful terminal is
`duration_reached`; strict model failure stays a failure. The unchanged empty
CLI default is legacy planar and intentionally does not accept `--duration`
without a physical selection. Existing legacy workflows remain in
[RIDE.md](RIDE.md).

## 2. Constrained spatial control

```sh
bike-ride --spatial 3d --terrain3d "${EXAMPLES}/flat.toml" --physics-config "${EXAMPLES}/constrained_fast.toml" --duration 2 --headless --no-plots
```

This is a constrained root control with 3D tyre/contact data and steering off.
It exercises the spatial request and schema2 channels, and does not test free
balance. Recording is off. Completion should be `completed`; a domain,
model or numerical terminal must retain its recorded classification.

## 3. Straight free-root ride

```sh
bike-ride --spatial 3d --terrain3d "${EXAMPLES}/flat.toml" --physics-config "${EXAMPLES}/free_fast.toml" --duration 2 --headless --no-plots
```

The free root now carries world position/quaternion and spatial tangent
velocities. The hold belongs to bootstrap and must be released before scored
intervals. The intended terminal is `completed`, with no scored hold.
V07/V08 must establish that result. Public classifications `crash:capsize`,
`crash:off_corridor`, `balance_lost`, `invalid:terrain`, `invalid:outside` and
`invalid:numerical` identify distinct failures; reducing limits or relabeling
one as a completed run is not a pass.

## 4. Cross-slope viewer

```sh
bike-ride --spatial 3d --terrain3d "${EXAMPLES}/cross_slope.toml" --physics-config "${EXAMPLES}/free_fast.toml" --duration 2 --no-plots --close-on-finish
```

The same physical request feeds the viewer. `C` cycles camera modes, `1` and
`2` select side/chase, and `R` requests the ordinary reset lifecycle. The
camera follows world Y and unwraps heading across `-pi/+pi`. A new generation
clears smoothing. Quaternion interpolation, camera movement and pacing alter
only the render copy. They do not step physics, run sensors or invoke a policy.

HUD roll is `N/A` when attitude is unavailable. Steering angle/torque use
rad/N m; lateral slips use m/s and respect front/rear contact validity. `HOLD`
is displayed while the accepted state says the hold is active. A real display
is mandatory for `spatial.viewer_smoke`; an absent display is a failing gate.
Recording is off in this example.

## 5. Explicit ride recording

```sh
bike-ride --spatial 3d --terrain3d "${EXAMPLES}/cross_slope.toml" --physics-config "${EXAMPLES}/free_fast.toml" --duration 2 --headless --no-plots --record --out "${OUT}/ride"
```

`--record` enables retained evidence and final publication. `--out` only names
the destination; it does not imply recording. Spatial evidence uses container2
and includes the spatial assets/checkpoints. Physical ride inspection is
available, while completed **research** episodes own the checked replay
contract demonstrated below. Interrupted/failure prefixes remain analyzable
with their actual extents and eligibility, without being certified complete.

## 6. Noise2 research recording

```sh
bike-research --spatial 3d --terrain3d "${EXAMPLES}/flat.toml" --physics-config "${EXAMPLES}/free_fast.toml" --duration 2 --rider lumped --motor-torque 0 --seed 0 --sensor-period 0.01 --sensor-delay 0.01 --record --out "${OUT}/research"
```

Research selects sensor schema2 and noise2 from the spatial mode. The eleven
components are specific-force XYZ, angular-rate XYZ, front/rear/crank angular
speed and motor/human torque. The IMU axes are sensor-frame axes. The seed is
`0`; sample period and latency are `0.01 s`; the existing research control
period remains the CLI default. Default component noise is enabled. The
`--ideal-sensors` flag selects zero stochastic standard deviations and zero
latency when explicitly requested; it does not bypass masks or validation.
Spatial external policies use ABI2; a V1-only plugin is rejected at loading.

The intended completed spatial research terminal is `completed` at its bounded duration;
the stored `outcome`, `model_status`, spatial reason and validity are
authoritative. The command is headless unless the existing viewer path is
explicitly requested. Save the printed sealed directory as `RUN`.

## 7. Inspection

```sh
bike-recording inspect "${RUN}"
```

Inspection validates and reads the sealed container without constructing a
simulator or importing a policy. It can remain available when the current
machine's execution identity cannot reproduce the run. Viewer inspection is
the existing `bike-replay --view` path; it displays saved poses and does not
certify them by integrating a new trajectory.

## 8. Checked research replay

```sh
bike-replay --checked "${RUN}"
```

Container2 is interpreted through recipe4. A complete eligible recording and
the exact numerical execution identity are required. Checked replay compares
the retained causal state/evidence and reports the first mismatch. A different
execution identity produces a named `incompatible-execution` diagnostic and a
nonzero exit; it must not fall back to visual inspection or relaxed tolerances.
`--checked` and `--view` are mutually exclusive. A successful inspection alone
is not a checked result.

## 9. Offline native report

```sh
bike-recording report "${RUN}" --episode 0 --output "${OUT}/spatial.html"
```

The native report reads accepted data and uses local SVG/HTML/PNG backends,
without a CDN or a second rollout. `--kind` can select `attitude`, `contact`,
`path`, `imu` or `energy`; `all` is the default. Changing the output extension
to `.svg` or `.png` selects the corresponding existing backend.

| Data | Units/frame | Time/availability |
| --- | --- | --- |
| Recorded root pose | m, quaternion wxyz; world/body | Accepted interval solve-stage pose, before endpoint forward |
| Roll/heading | rad; world ZYX convention | `attitude_valid`; unavailable samples split lines |
| Steering | rad, N m, J | Interval angle/torque; cumulative work on interval-end clock |
| Contact | m/s, N; wheel/contact frame | Per-wheel `contact_valid` |
| Axle paths | m; actual front/rear world transforms | Accepted force-stage geometry; steering/suspension retained |
| IMU | m/s² and rad/s; sensor frame | Delivery clock; source time retained; `valid` and mask bits |
| Hold/tyre energy | J; cumulative signed hold work / cumulative tyre dissipation | Dedicated accepted schema2 auxiliary channels, interval-end clock |
| Spatial outcome | reason, terminal/valid flags, tick, s, m | Accepted `channel_spatial_outcome` row; preserves the stop classification and its measured position |

The live camera uses the already-forwarded committed endpoint pose, keeping
the render skeleton and camera in phase. It does not recompute the recorded
interval force/IMU truth. No line bridges an unavailable interval. Missing channels produce a capability
diagnostic or an explicit N/A note; numeric zero never stands in for missing
evidence. Spatial queries use `spatial.*`; requesting them from a schema1
recording reports `recording schema 1 has no spatial channels`. Explicit
CSV/JSONL exports retain array shapes/masks and add an `.fields.json` sidecar
containing units, frames, phases and presence/validity descriptors.
The explicit spatial export tables are `channel_spatial`,
`channel_spatial_energy`, `channel_spatial_geometry` and
`channel_spatial_outcome`.

## 10. Process sweep

```sh
bike-research-sweep --out "${OUT}/sweep" --seeds 0-3 --jobs 2 -- --spatial 3d --terrain3d "${EXAMPLES}/flat.toml" --physics-config "${EXAMPLES}/free_fast.toml" --duration 2 --rider lumped --motor-torque 0 --record
```

Each child owns a separate simulation context and output destination. Job
identity covers canonical resolved configuration, actual terrain digest,
tyre calibration digest, sensor schema, seed and the internal thread count
(`1` by default). W52 adds `--threads N` for spatial tyre workers, with
`N` in `[1,64]`; the resolved value is retained in each job identity.
Process worker count is scheduling, so workers 1/2/4 must preserve job keys and
canonical ordered outcomes. Wall times and throughput remain measured,
scheduling-dependent fields. Crashed or model-invalid jobs are excluded from
speed statistics; their exit/outcome evidence remains in the summary and
other children continue.

## Verification handoff

The source-authored gates are `spatial.cli`, `spatial.viewer_pose`,
`spatial.reports_batch`, `spatial.documented_examples`,
`spatial.viewer_smoke` and `spatial.installed_product`. Install/package smokes
invoke the same relocated-product script with a native fixture helper. It
creates a schema1 reader control and a checksum-valid recording with a changed
execution identity, so compatibility rejection is not confused with corrupt
container rejection. It clears Python and loader path overrides, executes
from a fresh directory outside the checkout and preserves logs.

All execution results, artifact hashes and platforms remain **pending V08** in
the [application evidence ledger](superpowers/audits/2026-10-22-spatial-3d-application-evidence.md).
No speed, calibration, safety or physics qualification follows from these
written examples. See [TESTING.md](TESTING.md) for mandatory non-default gates
and [perf.md](perf.md) for V09 targets.
