# Physical ride, research and checked replay

## Status and prerequisites

The 2026-10-13 run-recording delivery changes normal physical/research output
to opt-in, final-only DuckDB recording. See [RECORDING](RECORDING.md) for the
verification results and the
[source-completion ledger](superpowers/audits/2026-10-13-run-recording-source-completion.md)
for the requirement map. The visual-runtime results below are historical and
predate recording; the recording behavior described in this document was
separately exercised on this branch.

### Platform support — measured 2026-10-17/18

This section is current evidence; the dated sections further down remain
historical macOS results.

| Lane | Toolchain | Native artifact | Gate evidence |
|---|---|---|---|
| macOS arm64 | AppleClang 21.0.0, CPython 3.14.3 (uv `.venv`), Xcode 26.6 `MacOSX.sdk` | `native/build/bike_native.cpython-314-darwin.so` | All static-analysis sweeps + CTest 10/10 green; native+golden pytest ~3,203 items |
| Linux aarch64 (Ubuntu 24.04 container) | Clang 18.1.3 and GCC 13.3.0, CPython 3.13.16 (`/opt/bike-venv`) | `native/build-linux-clang/bike_native.cpython-313-aarch64-linux-gnu.so` | All sweeps green on both compiler lanes; CTest 10/10; native+golden pytest ~3,221 items |

Setup and per-lane workflows: [linux.md](development/linux.md) and
[macos.md](development/macos.md); failure diagnosis:
[troubleshooting.md](development/troubleshooting.md). Machine evidence logs
(including the retained pytest failures recorded verbatim in each guide's
limits note) live under `output/portability/`; the consolidated
[portability matrix](development/portability-test-matrix.md) tracks the
release-level lane status. Wheel/provider qualification:
[dependency-matrix.md](development/dependency-matrix.md) and
[numerical-profiles.md](development/numerical-profiles.md).

Not qualified: Linux x86_64 (the spec's release-blocking lane — no host yet),
macOS Intel, native Wayland/EGL/OSMesa, cross-compilation. Interactive
desktop sessions are pending real acceptance per
[viewer-platform-acceptance.md](development/viewer-platform-acceptance.md);
the macOS GUI results below are the 2026-10-10 historical record, not a new
measurement.

### Historical visual-runtime acceptance: 2026-10-10

The original acceptance session (branch `impl/cpp-port-p2`) recorded:

- The native extension builds clean under the documented Release workflow below;
  `bash tools/run_tests.sh native` is green and the `full` profile fails only
  on the recorded platform-baseline failure set (see
  [the execution audit](superpowers/audits/2026-10-08-native-visual-runtime.md)).
- The macOS GUI paths below were exercised for real: `bike-ride` (physical,
  native backend, 1x and 8x), `bike-ride --research` (native, `--duration 1`),
  and `bike-replay --viewer` on a recorded native episode. Windowed native
  stepping, pacing and checked replay all work under the required `mjpython`
  re-exec (the launcher re-execs automatically when `mjpython` is on PATH).
- Measured results are reported in
  `output/native-visual-acceptance/{python,native}/report.json` and in the audit:
  strict headless runs terminate at step 4 on this platform's documented
  `energy.constraint_work` baseline violation, so the reported medians describe
  a 0.005 s diagnostic prefix, not steady-state throughput. Sustained
  interactive evidence: native held `achieved_rtf` mean 0.9999 at requested 1x
  (visual 1x gate >= 0.95 satisfied) and ~1.8x at requested 8x.

The native source targets the project's macOS/Accelerate toolchain. It requires
a freshly built extension: a Track B binary does not implement the presentation
and pacing interfaces, and the loaded artifact must carry the current compiled
source stamp. One-time build:

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
higher achieved throughput — the interactive 8x run above achieved a sustained
~1.8x. The HUD/preview distinguish requested scale from measured
simulated-time/wall-time RTF, and physics is scale-invariant by construction
(the 1x and 8x runs produced byte-identical run outcomes: `crash` at
t=31.178750 s, 24943 steps, identical position).

Exact profile/track examples:

```bash
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 1
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --time-scale 2
```

Use `--backend python` for the reference path with the same profile. A headless
physical run uses `--headless --duration 30 --no-plots`; it has no playback scale.
Headless physical runs use strict checks — on this platform they terminate at
the documented `energy.constraint_work` baseline violation (t=0.005 s) as
`invalid_controller`, a retained diagnostic prefix rather than a defect;
interactive physical runs retain model-validity/failure diagnostics in the HUD
(`[FIRST FAILURE]` banner) and exported summaries rather than hiding invalid
intervals.

Native capability remains the A/B subset: physical `articulated_effort`, the
`articulated_planar` rider without pitch assist, spindle pedals / saddle pin /
grip connects, track-material analytic `compliant_2d` tires and the
experimental multi-contact `distributed_2d_reference` tyre (unvalidated
material; see
[the native distributed tyre audit](superpowers/audits/2026-10-11-native-distributed-tire.md)),
`ideal_mid_drive`, no motor clutch and zero reflected rotor inertia. Native
selection does not add elastic-chain, alternative-support or legacy drive
support.

Held brakes distinguish rolling from standstill. While a wheel turns, physical
braking is the solved static-friction bound (`dof_frictionloss`) and the
frictional torque fades to zero as ω→0 — a rolling held brake never reverses
the bike. At a standstill under a pulled lever, the `brake_hold_*` drive
fields add a separate pad bite: the wheel angle latches when |ω| drops at or
below `brake_hold_engage_radps`, and a bounded spring–damper torque
`−brake_hold_stiffness_nm_rad·(θ−θ_latch) − brake_hold_damping_nms_rad·ω`
(clipped to `brake_hold_torque_nm·demand`) holds it until |ω| exceeds
`brake_hold_release_radps` or the lever releases. The distinct engage/release
thresholds give hysteresis, the hold reports as its own `brake_hold` force
component and folds into energy-loss accounting, and it resets with each
episode. Setting the hold torque or stiffness to zero disables the bite and
preserves the legacy fade-only behaviour.

The `pneumatic` tyre's model contract, fitted constants and literature targets —
the sections the code cites as `RIDE.md §3.1`, `§4.1`, `§11.2` and `§12 item 10`,
which this file no longer carries — live in
[docs/reference/tyre-calibration.md](reference/tyre-calibration.md) under those
same section numbers.

## Interactive controls and rendering

F6 lowers the scale, F7 raises it, F8 restores 1x. C cycles the camera, 1/2 select
2D/3D, T toggles terminal telemetry and G toggles model markers. Q/Escape closes.
Physical Space toggles braking; comma/period lower/raise brake strength. Physical
R seals the current generation in RAM and resets the same compiled setup.

Physical/research persistence now requires `--record`; `--out` alone does not
enable it. The ordinary new recording adapter requires `--backend native`.
Without recording there are no automatic preview CSV, per-generation legacy
bundles or recording-derived HTML files. With recording, every accounted interval
and all reset-separated episodes remain in RAM until launch close. Finalization
publishes one exclusive run directory containing `run.duckdb`, `manifest.json` and
a successful `summary.md`; the CLI prints the actual directory. A terminal
outcome announces `[RUN ENDED]`, seals its episode in RAM, then holds the final
frame without publishing until the window closes. See [RECORDING](RECORDING.md)
for failure/retry behavior, check policy, analysis commands and explicit legacy exports.

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

On macOS the passive viewer installs its own `mjcb_time` timer from
`mujoco._simulate` at launch; the native stepper whitelists exactly the
`mujoco._structs`/`mujoco._simulate` vendor timers and still rejects
Python-installed or foreign C callbacks before stepping.

## Research and Python motor policies

`bike-ride --research` supports both interactive and headless execution with the
same `--backend` selection. `bike-research --backend python|native` remains
headless and has no playback-scale switch. Example:

```bash
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --research --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_savage.toml --duration 1 --time-scale 1 --record --out output/track-c-research
```

Space pauses/resumes; B toggles brakes; R seals the outgoing episode in RAM and
resets. With `--record`, finalization creates a unique run directory beneath
`--out` without replacing existing runs. Episodes are rows in that run, not
automatically exported `episode-0000` directories. Pause/brake/stop/reset requests
take effect at the external control boundary. Closing the window finishes only
its already accepted control interval and then publishes; it does not request
another policy command. Both Python and native research support the new recorder.
Explicit programmatic legacy `save()` remains available.

A `module:factory` motor policy stays in Python. `PolicySession.begin_advance()`
invokes it once; repeated `advance_pending(wall_budget_s=..., target_step=...)`
continues that exact command. The optional absolute `target_step` limits physics
progress without truncating the external window, flushing a partial period or
publishing an extra transition. This also allows control periods longer than the
pacing debt cap. Default no-target synchronous APIs remain compatible.

## Checked replay

```bash
# Set RUN to the actual unique directory printed by a recorded research launch.
RUN=/absolute/path/to/published/run
uv run --frozen --group native bike-replay "$RUN" --episode 0
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-replay "$RUN" --episode 0 --viewer --time-scale 2
```

There is no replay backend override. Legacy replay schema 1 selects Python;
schema 2 selects its recorded backend and execution identity. New DuckDB
containers use container schema 1 and separate replay contract 3; `--episode` is
required when selection is ambiguous. Old directories retain their original
reader and are not upgraded in place. File hashes, source/runtime,
recorded binary/library identity and supported native capabilities are checked
before constructing the replay plant. Bundled rider parameter files replace
untrusted original recipe paths. Recorded policy metadata is data only: no
recorded policy factory is imported or called.

Headless and viewer share `ReplaySession`: initial state/observation, requested
commands and times, transitions, observations, applied events, outcome, model
status, event/episode metrics and final integration state are checked with the
existing `1e-9` tolerances. A saved `not_finished` prefix is valid replay input:
passing verifies that recorded prefix, not a completed physical ride. New v3
error/incomplete/invalid evidence remains analyzable but is not eligible to claim
completed successful replay. The following is historical legacy evidence only:
the 2026-10-10 native episode (truncated `numerical_quality`, 104 steps) replayed
with `passed=true` and `final_state_max_abs_error=0.0` in the real viewer.

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
Short terminal prefixes require explicit review — on this platform every strict
run is short because the documented `energy.constraint_work` baseline violation
at t=0.005 s ends strict progression at step 4 for both backends; the reported
medians (python 0.4736, native 2.0196 RTF in the linked `report.json`
artifacts) measure boundary cost on that prefix,
not sustained throughput. Sustained evidence comes from the interactive runs
above. The headless RTF threshold field alone cannot establish Track C
acceptance: full parity/replay tests, sanitisers/inspections, actual macOS UI
checks and 1x mean achieved RTF >= 0.95 remain separate gates.

## Spatial terrain authoring status

Native 3D terrain is **implemented in source; qualification is pending**.
W06–W12 supply the immutable grid/asset, bounded queries, native generation,
exact-profile height extrusion and separate render raster. Spatial runtime
capability remains gated until its tyre/startup/integration producers and V
acceptance are complete; this section does not advertise a runnable 3D ride.

The source recipes `examples/research/alpine_rocky_3d_genspec.toml` and
`rough_uphill_savage_3d_genspec.toml` refer to existing longitudinal tracks and
add explicitly authored lateral geometry. Their manifest hashes and execution
evidence are pending. Sharp edges in the savage base can violate the generator's
60-degree slope bound and are rejected; the source delivery neither smooths
those edges nor claims successful generation. See the terrain audit for V02/V05
acceptance and the remaining contract decisions.
