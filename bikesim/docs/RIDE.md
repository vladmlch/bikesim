# Physical ride, research and checked replay

## Delivery status and prerequisites

This is the Track C **source overlay**, not a built or verified release. No
commands in this document were executed for the 2026-10-10 delivery. The supplied
base omits root packaging/lock files, upstream verification scripts and most
older tests. Apply the overlay to the corresponding full BikeSim checkout before
using its installed commands or verification profiles. Keep the existing pinned
NumPy/MuJoCo environment and native numerical flags.

The native source currently targets the project's macOS/Accelerate toolchain.
It requires a freshly built extension: an older Track B binary does not implement
the additional presentation and pacing interfaces. In a later, explicitly
permitted build session, the existing full-checkout workflow is:

```bash
uv run --frozen --group native cmake -S native -B native/build/release -DCMAKE_BUILD_TYPE=Release
uv run --frozen --group native cmake --build native/build/release
```

Application loading does not run these commands. It selects
`native/build/release`, or the absolute directory in `BIKE_NATIVE_BUILD_PATH`,
and verifies the actual imported extension path, required dependency versions,
compiled source stamp and loaded MuJoCo identity. Unsupported configuration,
missing/stale binary or a conflicting prior import fails; it never falls back to
Python.

## Physical backend and fixed physics

`bike-ride --backend python|native` defaults to `python`. `--time-scale` is accepted
only for interactive physical/research runs, with values 1, 2, 4 and 8; absent
means 1. Supplying it to a headless, legacy or track-preview run is an error,
including an explicit `--time-scale 1`.

The supplied welded profile remains at **0.00125 s**. Playback never edits the
integration timestep, physical/controller/sensor periods, control inputs,
numerical tolerances or the model. Higher requested scales are not a promise of
higher achieved throughput. The HUD/preview distinguish requested scale from
measured simulated-time/wall-time RTF.

Exact profile/track examples, for a later authorized launch:

```bash
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 1
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 2
```

Use `--backend python` for the reference path with the same profile. A headless
physical run uses `--headless --duration 30 --no-plots`; it has no playback scale.
Headless physical runs use strict checks; interactive physical runs retain
model-validity/failure diagnostics rather than hiding invalid intervals.

Native capability remains the A/B subset: physical `articulated_effort`, the
`articulated_planar` rider without pitch assist, spindle pedals / saddle pin /
grip connects, track-material analytic `compliant_2d` tires, `ideal_mid_drive`,
no motor clutch and zero reflected rotor inertia. Native selection does not add
elastic-chain, distributed-tire, alternative-support or legacy drive support.

## Interactive controls and rendering

F6 lowers the scale, F7 raises it, F8 restores 1x. C cycles the camera, 1/2 select
2D/3D, T toggles terminal telemetry and G toggles model markers. Q/Escape closes.
Physical Space toggles braking; comma/period lower/raise brake strength. Physical
R exports the current generation and resets the same compiled setup.

The physical viewer writes a root `preview.csv`, including generation/reset
markers and requested/achieved pacing values. Full accounted data are exported
under `generation-0001`, `generation-0002`, etc., with `summary.json`,
`telemetry.csv`, `intervals.jsonl` and `terrain_vertices.npy`. The run directory
includes the track, drive mode and configuration hash. Headless physical output
uses the same evidence files directly in that run directory. Physical reset
starts fresh accounting and recorder state; finalization exports before close.

The window holds an independent render model/data pair. An owning frame carries
full `mjSTATE_INTEGRATION`, the six declared mutable model arrays, native/Python
endpoint values and the latest accounted interval. Applying it performs geometry
updates only, not `mj_step`, `mj_forward`, force accounting or sensor acquisition.
Changes to the render model cannot steer authoritative physics. Existing HUD
labels, units and missing-channel blanks are retained.

Playback debt is reduced by actual completed physics steps. A budget yield keeps
unfinished work; scale/pause/reset rebases discard stale pacing debt. A normal
compute slice has an 8 ms budget checked between physics steps and frame requests
are capped at 60 Hz. These are cooperative boundaries, not a preemptive deadline:
a single solve, external Python policy call, export or GUI operation can exceed
8 ms. No new zero-allocation/real-time-safe C++ region is claimed.

## Research and Python motor policies

`bike-ride --research` supports both interactive and headless execution with the
same `--backend` selection. `bike-research --backend python|native` remains
headless and has no playback-scale switch. Example:

```bash
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --research --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 1 --time-scale 1 --out output/track-c-research
```

Use a new/empty research output directory. Episodes are saved as
`episode-0000`, `episode-0001`, etc. Space pauses/resumes; B toggles brakes; R
saves and resets. Pause/brake/stop/reset requests take effect at the external
control boundary. Closing the research window finishes only its already active
control interval, saves it, then closes; it does not request another policy
command.

A `module:factory` motor policy stays in Python. `PolicySession.begin_advance()`
invokes it once; repeated `advance_pending(wall_budget_s=..., target_step=...)`
continues that exact command. The optional absolute `target_step` limits physics
progress without truncating the external window, flushing a partial period or
publishing an extra transition. This also allows control periods longer than the
pacing debt cap. Default no-target synchronous APIs remain compatible.

## Checked replay

```bash
uv run --frozen --group native bike-replay output/track-c-research/episode-0000
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-replay output/track-c-research/episode-0000 --viewer --time-scale 2
```

There is no replay backend override. Schema 1 selects Python; schema 2 selects
its recorded backend and execution identity. File hashes, source/runtime,
recorded binary/library identity and supported native capabilities are checked
before constructing the replay plant. Bundled rider parameter files replace
untrusted original recipe paths. Recorded policy metadata is data only: no
recorded policy factory is imported or called.

Headless and viewer share `ReplaySession`: initial state/observation, requested
commands and times, transitions, observations, applied events, outcome, model
status, event/episode metrics and final integration state are checked with the
existing `1e-9` tolerances. A saved `not_finished` prefix is valid replay input:
passing verifies that recorded prefix, not a completed physical ride.

Viewer Space pauses without completing a pending command. R constructs a fresh,
checked replay; it does not reuse controller or sensor state. F6/F7/F8 and camera,
telemetry and marker controls remain available. On mismatch the last verified
control-boundary frame is held. On success the verified final frame is held.
Q/Escape or closing the window returns **0** only after final verification,
**1** for mismatch/error, **2** for early close. An incomplete replay cannot
produce a successful `report()`, and its close does not simulate hidden work.

## Throughput evidence

`tools/measure_realtime.py` takes the profile's timestep unless `--dt` is
explicit. It does not build native code. Run Python and native sequentially with
the same settings; one warm-up and three measured repetitions are defaults:

```bash
uv run --frozen --group native python tools/measure_realtime.py --backend python --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 30 --warmup 1 --runs 3 --out output/track-c-rtf/python
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python tools/measure_realtime.py --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 30 --warmup 1 --runs 3 --compare output/track-c-rtf/python/report.json --out output/track-c-rtf/native
```

Measured wall time surrounds the full advancement loop and final flush. Setup
and export have separate times. Percentiles describe whole `advance` calls in
seconds, with actual completed-step counts; no per-step p95 is inferred from a
packet. Reports retain effective configuration, terrain/source/runtime/artifact
identity, clocks, strictness, decimation, errors, validity, terminal reason and
prefix length. The default recording decimation is 80 for both measured backends.

Different settings or divergent classified prefixes are not comparable. Equally
classified invalid prefixes remain diagnostic, never calibrated valid rides.
Short terminal prefixes require explicit review. The headless RTF threshold
field alone cannot establish Track C acceptance: full parity/replay tests,
sanitisers/inspections, actual macOS UI checks and 1x mean achieved RTF >= 0.95
remain separate gates. This delivery contains **no measurement results**.
