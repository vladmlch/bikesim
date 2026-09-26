# Rough Road: Finishing Ride Mode Plan

**Goal:** Bring the existing `ride` mode to a usable state — a `bike-ride` CLI, a
declarative TOML track file with hand-placed and procedurally generated road
defects (potholes and bumps of configurable shape and size), a CSV telemetry
recorder with summary metrics and plots, tests, and the documentation that was
left unwritten when the 2026-08-25 plan halted after Task 4.

**Not a new MJCF mode.** Ride mode already provides planar (sagittal-plane)
rolling over a heightfield road with cruise control, brakes, rolling resistance
and a virtual rider. A "road with bumps and potholes" is a *track*, and
`TrackSpec` is the abstraction built for exactly that. Everything below is a
track-file, CLI, telemetry and documentation layer on top of what exists.

**Predecessor:** [`2026-08-25-ride-mode.md`](2026-08-25-ride-mode.md), Tasks 1–4
landed (Task 4 uncommitted at the halt, committed as Step 0 of this plan).
Its Task 5 is absorbed here in full.

**Spec:** [`docs/RIDE.md`](../../RIDE.md) — the Physics Contract remains the
binding authority for the physics. This plan does not change the physics.

## Decisions (settled 2026-09-26)

| # | Decision |
|---|---|
| D1 | Finish the existing `ride` mode; no fifth `mode` string in the MJCF builder. |
| D2 | Road content is a procedural generator that materialises into an ordinary `TrackSpec`, so HUD, markers, plots and validation work unchanged. |
| D3 | Road-defect scale by default (potholes 40–120 mm deep, 0.3–0.8 m long; bumps 30–80 mm high, 0.3–0.6 m long; background roughness a few mm). Trail scale stays in `enduro_aggressive`. All sizes user-configurable. |
| D4 | Stays planar (2D). No roll, no steering, no avoidance. |
| D5 | Primary output is measurement: headless run → CSV + `summary.json` + plots. The interactive viewer keeps working as a bonus. |
| D6 | Task 4 is committed as-is first (268 tests green), so the diff of this work stays readable. Substantive review of Task 4 is out of scope. |
| D7 | All eight items of the predecessor's Task 5 are in scope, one commit per step. |
| D8 | Geometry has one source of truth: the **TOML track file**. The CLI carries only run-level knobs (`--seed`, `--length`, `--speed`, …). |
| D9 | Pothole shapes: `sharp` (existing `Pothole`), `sloped` (chamfer `edge_m`), `bowl` (cosine dip). Bump shapes: `cosine`, `trapezoid` (`ramp_m` + `plateau_m`). |
| D10 | Heightfield size is derived from `length_m`; the default field (120 m) and `tests/golden/baseline_bike_ride.xml` are unchanged. |
| D11 | Cruise limits 15–45 km/h and default 25 km/h unchanged. |
| D12 | Acceleration metrics: 100 Hz low-pass (Butterworth, zero-phase) for RMS/peak, **plus** the raw peak labelled `raw, incl. solver transients`. CSV keeps raw channels. |
| D13 | Viewer does not switch tracks at runtime; the track is chosen on the command line. |
| D14 | `DEFAULT_PRESET` in `terrain/presets.py` and the shipped `BikeSpecs` defaults do not change. The CLI's own default track is `road_worn`. |

### Physical fact recorded for the generator

A wheel of radius `r` (0.372 m front / 0.352 m rear) cannot reach the floor of a
hole shorter than `2r ≈ 0.74 m`. Its drop is bounded by geometry:
`drop = r − sqrt(r² − (w/2)²)` — 0.3 m → ~32 mm, 0.5 m → ~100 mm, 0.6 m → ~150 mm.
A short pothole's declared depth beyond that bound changes nothing. The
generator, the preview and the summary therefore report the **effective drop**
of each pothole alongside its declared depth.

## Global Constraints

- `uv` for every Python invocation.
- Every existing golden baseline stays byte-identical. Ride mode's default
  heightfield geometry is part of `baseline_bike_ride.xml`; a non-default
  `length_m` produces a different in-memory XML, never a regenerated baseline.
- `bike_sim.terrain` imports no MuJoCo and no other `bike_sim` package.
  `tomllib` (stdlib, Python ≥ 3.12) is allowed there for reading; writing TOML
  is done with a small hand-rolled emitter, no new dependency.
- No pandas. CSV via the standard library.
- No source file over 500 lines.
- Tolerances in tests are measured, then recorded in the docstring — never
  invented.
- Seeded generators only; `np.random.default_rng(seed)`.
- Units: `bike_sim.terrain` is in metres. The track file uses **mm** for
  heights/depths and **m** for lengths/positions because that is how people
  describe road defects; the loader converts.

## Track file format (TOML)

```toml
name = "my_road"
length_m = 150
description = "optional"

[[obstacles]]             # hand-placed; any type from the catalogue
type = "pothole"
start_m = 20.0
depth_mm = 80
length_m = 0.5
edge = "sharp"            # sharp | sloped | bowl   (sloped: edge_m = 0.10)

[[obstacles]]
type = "bump"
start_m = 35.0
height_mm = 50
length_m = 0.4
shape = "cosine"          # cosine | trapezoid     (trapezoid: ramp_m, plateau_m)

[generator]               # optional procedural fill around the hand-placed ones
seed = 0
runup_m = 10.0            # reserved: bike starts at 2 m and needs ~8 m to reach speed
runout_m = 5.0
potholes_per_100m = 4
pothole_depth_mm = [40, 120]
pothole_length_m = [0.3, 0.8]
pothole_edge = { sharp = 1.0 }               # weights; a single key = one shape
bumps_per_100m = 6
bump_height_mm = [30, 80]
bump_length_m = [0.3, 0.6]
bump_shape = { cosine = 1.0 }
roughness_mm = 3                              # 0 disables
roughness_correlation_m = 0.2
```

Existing catalogue types are addressable as `square_edge`, `washboard`,
`g_out`, `drop`, `kicker`, `roots`, `rock_garden` with their dataclass field
names (metres, as today). A two-element list means a uniform range drawn per
obstacle from the seed; a scalar means a fixed value.

`bike-ride --dump-track <preset>` writes any built-in preset in this format.
The three road levels `road_smooth`, `road_worn`, `road_broken` are built-in
presets defined as `[generator]` blocks.

## CLI

```
bike-ride [--track NAME|PATH] [--seed N] [--length M] [--speed KMH]
          [--headless] [--sag PCT] [--out DIR] [--decimate N]
          [--preview] [--dump-track NAME]
```

Defaults: `--track road_worn`, `--speed 25`, seed and length from the file.
`--seed`/`--length` override the file for the run. Artifacts go to
`output/ride/<track>_<speed>_s<seed>/` (`telemetry.csv`, `summary.json`,
`*.png`). `--preview` renders the road profile with markers and effective
drops and exits without simulating. `length_m > 500` prints a warning about
viewer triangle count; headless runs are unaffected.

## Summary metrics (`summary.json` + console table)

Per end (fork, shock): max travel (mm, %), 95th percentile, bottom-out count
(fork: within 5 mm of 180 mm; shock: bumper engaged, > 55 mm), top-out count.
Accelerations (bar, saddle): RMS and peak of the 100 Hz-filtered vertical
channel, raw peak alongside. Airborne events. Mean speed over the window
excluding the first 8 m. Crash flag and reason. Per-pothole effective drop.

## Tasks

### Step 0 — Baseline commit and hygiene
Commit Task 4's untracked work (`sim/ride/{hud,input,livery,session,termination,viewer}.py`,
`tests/test_ride_track.py`, modified `ride_sim.py`, `sim/ride/__init__.py`,
`README.md`), plus `uv.lock`, the predecessor plan and this plan,
`.claude/settings.json`. Add to `.gitignore`: `.agents/`, `.ai/`, `.codex/`,
`.junie/`, `skills-lock.json`, `.claude/settings.local.json`.
**Verify:** `uv run python -m pytest -q` (268 passed before this plan).

### Step 1 — Obstacle shapes
`terrain/obstacles.py`: `Bump` (cosine hump: `height_m`, `length_m`),
`TrapezoidBump` (`height_m`, `ramp_m`, `plateau_m`), `SlopedPothole`
(`depth_m`, `hole_length_m`, `edge_m`, flat floor), `BowlPothole` (`depth_m`,
`hole_length_m`, cosine). `effective_wheel_drop_m(r)` helper for pothole-like
obstacles. Tests in `tests/test_terrain.py` style: shape, datum return,
grid independence.

### Step 2 — Track file and generator
`terrain/trackfile.py`: `load_track(path) -> TrackSpec`, `dump_track(track) -> str`,
type registry, unit conversion, range parsing. `terrain/road.py`:
`RoadGeneratorSpec` dataclass + `generate_road(spec, length_m, reserved) -> List[Obstacle]`
(seeded, fills free intervals, honours shape weights, adds a `RoadRoughness`
obstacle spanning free ground when `roughness_mm > 0`). Presets
`road_smooth/road_worn/road_broken` registered in `PRESETS`. Tests: seed
determinism, ranges honoured, `validate()` passes, no overlap with hand-placed
obstacles, dump→load round-trip equals the source `TrackSpec`.

### Step 3 — Heightfield from track length
`HeightFieldSpec.for_track_length(length_m)` keeping 5 mm resolution and the
vertical envelope; thread an optional `HeightFieldSpec` through
`RideSimulation → generate_mujoco_xml → build_terrain`. Default path is
byte-identical to today (golden test proves it). Test: a 300 m track compiles
and a short traverse runs.

### Step 4 — `bike-ride` CLI
`cli/ride.py`, console script in `pyproject.toml`, `--sag` wired to
`compute_suspension_tuning_for_sag(front_weight_fraction=None)` applying fork
pressure and coil rate to the run without touching `BikeSpecs` defaults.

### Step 5 — Recorder and metrics
`sim/ride/recorder.py` (channels per the predecessor's Task 5 list, plus the
two accelerometers), `sim/ride/metrics.py` (summary as above; `scipy.signal`
Butterworth 4th order, 100 Hz, `filtfilt`).

### Step 6 — Plots and preview
`viz/ride_plots.py`: travel vs X with `TrackSpec.markers`, per-end
shaft-velocity histogram, filtered accelerations with raw peaks annotated;
`plot_track_profile()` for `--preview`. Match `viz/theme.py`.

### Step 7 — Tests
`tests/test_ride_telemetry.py` (header, one row per recorded step, bit-identical
repeat, plots render headless), `tests/test_ride_invariants.py` on `road_worn`
(completes, sag within tolerance, travel within measured tolerance, speed
tracks target excluding ramp, energy bounded with rider work as a source),
CLI smoke (`--headless` on a 40 m track, `--preview`, `--dump-track`).

### Step 8 — Documentation
`docs/RIDE.md`: §9 sag table corrected (40.6 % / 22.7 % solved vs 42.0 % /
25.3 % first-order, both columns, which one the simulation produces and why);
contact-debounce note in §6/§7/§12; new sections Usage, CLI, Track file,
Telemetry reference, reading the shaft-velocity histogram; TOC. `README.md`:
Ride Mode section, TOC, Repository Structure, Provenance (track layout, `Crr`,
tyre stiffness, rider ceiling, omitted aerodynamics). `docs/ARCHITECTURE.md`:
directory tree and subsystem table refreshed for `terrain/`, `sim/ride/`,
`physics/coil_shock.py`, `physics/tuning.py`, `mujoco/terrain.py`.

**Final verification:** `uv run python -m pytest -q`;
`uv run bike-ride --headless`; `uv run bike-ride --track road_broken --preview`;
`uv run bike-ride --dump-track enduro_aggressive`.
