# `bike_sim` Architecture & Developer Guide

## 1. Overview & Purpose

`bike_sim` is a modular, high-fidelity Python and MuJoCo simulation package for enduro/downhill mountain bike suspension analysis. It models:
- Complete analytical bicycle frame geometry (Reach, Stack, Head Angle, Mullet wheels, BB Drop, Fork Offset, Mechanical Trail).
- Closed-form 4-bar Horst-Link suspension kinematic solving with zero phantom stroke and strict link invariant conservation ($< 10^{-12}\text{ mm}$).
- Non-linear thermodynamic pneumatic fork spring (DebonAir+ dual-chamber adiabatic air spring with volume token tuning).
- Multi-circuit hydrodynamic dampers (RockShox Charger 3 RC2 fork damper & Super Deluxe Ultimate RC2T rear shock with Hydraulic Bottom-Out and Threshold Lockout).
- Physical mass and inertia profiles (full-power eMTB 24.4 kg frame + rider 80 kg toggle).
- MuJoCo MJCF XML procedural compiler across four simulation modes (Standard, Stand, Playground, Ride).
- Live interactive 2D suspension test stand playground with position servo control, auto-sweep, multi-camera tracking, and live HUD telemetry.
- **Ride mode**: a planar free-rolling whole-bike model over a heightfield road — obstacle catalogue at trail and road scale, procedural rough-road generator, TOML track files, force-based suspension (air fork, coil shock with bumper, click-tuned dampers), PI cruise control, brakes, rolling resistance, a bounded virtual rider, crash detection, and a headless telemetry / summary / plotting pipeline behind the `bike-ride` command (`docs/RIDE.md`).

---

## 2. Directory Layout & Subsystem Decomposition

```
mujoco_sim_new2/
├── pyproject.toml              # Build config, package metadata, dependencies & CLI entrypoints
├── output/                     # Isolated directory for all generated artifacts (gitignored)
│   ├── models/                 # Generated .xml and .json models
│   └── plots/                  # Generated .png figures
├── src/
│   └── bike_sim/               # Core Python Package (`bike-sim`)
│       ├── __init__.py         # Top-level public API exports
│       ├── config.py           # Unified composite BikeConfig dataclass
│       ├── geometry/           # Frame geometry, hardpoints, angles, trail & validation
│       │   ├── __init__.py
│       │   ├── hardpoints.py   # Fixed & kinematic frame hardpoints (P0-P12, BB, P_FA, etc.)
│       │   ├── specs.py        # BikeSpecs, FrameGeometrySpecs, SuspensionHardwareSpecs
│       │   └── validation.py   # Geometry sanity checks
│       ├── kinematics/         # Linkage solving, leverage ratio curves & instant centers
│       │   ├── __init__.py
│       │   ├── curves.py       # Instant center & kinematic curve utilities
│       │   ├── linkage_math.py # Circle-circle intersection and planar helpers
│       │   └── solver.py       # Analytical HorstLinkageSolver (closed-loop loop closure)
│       ├── physics/            # Pure calculators: springs, dampers, mass, tuning (never touch mjModel/mjData)
│       │   ├── __init__.py
│       │   ├── air_spring.py   # ForkAirSpring thermodynamic pneumatic model
│       │   ├── coil_shock.py   # CoilShock: linear coil + progressive bottom-out bumper
│       │   ├── damper.py       # Charger3Damper, SuperDeluxeDamper, BikeSuspensionSystem, presets
│       │   ├── mass.py         # BikeMassSpecs, RiderSpecs, CoM & static axle loads
│       │   └── tuning.py       # compute_suspension_tuning_for_sag (fork psi / coil rate for a sag target)
│       ├── terrain/            # Ride road geometry — pure NumPy/SciPy, imports no MuJoCo
│       │   ├── __init__.py
│       │   ├── obstacles.py    # Catalogue: SquareEdge, Pothole, Washboard, GOut, Drop, Kicker, Roots, RockGarden,
│       │   │                   #   Bump, TrapezoidBump, SlopedPothole, BowlPothole, RoadRoughness
│       │   ├── wheelpath.py    # Rolling-wheel envelope: effective drop of a rigid wheel into a hole
│       │   ├── profile.py      # TrackSpec, build_profile (datum carry, overlap rejection), markers
│       │   ├── road.py         # RoadGeneratorSpec, generate_road/build_road, road_smooth/worn/broken
│       │   ├── trackfile.py    # TOML track files: load_track / dump_track / save_track
│       │   ├── presets.py      # Registry of named tracks (authored + generated)
│       │   └── heightfield.py  # HeightFieldSpec (default 120 m, derived for longer tracks), rasterization
│       ├── mujoco/             # Modular MuJoCo MJCF XML Builder & Exporter
│       │   ├── __init__.py
│       │   ├── _xml_format.py  # Pretty-printing
│       │   ├── actuators.py    # Position servos (stand), rear drive + brakes (ride)
│       │   ├── assets.py       # Textures, materials, visuals XML builder
│       │   ├── builder.py      # Main orchestrator (generate_mujoco_xml; mode and optional heightfield spec)
│       │   ├── environment.py  # Lighting, floor, stand fixtures, ride catch plane
│       │   ├── terrain.py      # Ride hfield asset + terrain geom (data filled from Python after compile)
│       │   ├── exporter.py     # File exporter (export_mujoco, export_playground_models, export_json)
│       │   ├── frame.py        # Frame body, tubes, battery, casting, saddle; planar root joints in ride mode
│       │   ├── linkage.py      # Loop-closure constraints and contact exclusions
│       │   ├── rear_linkage.py # Chainstay, seatstay, rocker, yoke, shock bodies
│       │   ├── steering_fork.py# Steerer, fork, front wheel; contact sphere in ride mode
│       │   ├── drivetrain.py   # Wheels, spin joints, rear contact sphere
│       │   ├── rider.py        # 3D Rider capsules XML builder
│       │   └── sensors.py      # Telemetry sensors (+ bar/saddle accelerometers in ride mode)
│       ├── sim/                # Simulation runtime — the only layer that mutates mjData
│       │   ├── __init__.py
│       │   ├── camera.py       # CameraManager (2D tracking & 3D isometric chase)
│       │   ├── controllers.py  # SuspensionController (fork air + damping; stand shock spring)
│       │   ├── equilibrium.py  # solve_static_equilibrium for ride-mode runs
│       │   ├── hud.py, input_handler.py, telemetry.py   # Test-stand HUD, keys, telemetry dict
│       │   ├── playground.py   # SuspensionPlayground & run_interactive_playground
│       │   ├── ride_sim.py     # RideSimulation: compiles ride model, rasterizes track, sequences per-step writers
│       │   └── ride/           # Ride-mode writers and tooling
│       │       ├── forces.py         # SuspensionForceApplier (fork + coil shock into qfrc_applied)
│       │       ├── contacts.py       # TerrainContactQuery / TerrainContacts (debounced gates, raw support)
│       │       ├── resistance.py     # RollingResistance (Crr·N·r)
│       │       ├── cruise.py         # CruiseController (PI on chassis velocity, contact-gated)
│       │       ├── braking.py        # BrakeController (sign-aware, tapered)
│       │       ├── virtual_rider.py  # PitchStabilizer, CrashDetector
│       │       ├── wheels.py         # Wheel-spin handles
│       │       ├── termination.py    # RunLimits, RunTerminator, RunOutcome
│       │       ├── session.py, viewer.py, input.py, hud.py, livery.py   # Interactive ride
│       │       ├── recorder.py       # RideRecorder: per-step channels, stdlib CSV
│       │       └── metrics.py        # RideSummary: travel, bottom-outs, filtered accelerations, effective drops
│       ├── viz/                # Publication Plotting & Terminal ASCII Tables
│       │   ├── __init__.py
│       │   ├── dyno_plot.py    # Damper F-v & HBO dyno curves plotter
│       │   ├── plots.py        # Linkage geometry, leverage ratio, compression comparison
│       │   ├── ride_plots.py   # Ride telemetry figures and track profile preview
│       │   └── tables.py       # Formatted ASCII tables & coordinate summaries
│       └── cli/                # CLI Entrypoints
│           ├── __init__.py
│           ├── export.py       # `bike-export` entrypoint
│           ├── main.py         # `bike-sim` entrypoint
│           ├── playground.py   # `bike-playground` entrypoint
│           └── ride.py         # `bike-ride` entrypoint
├── docs/
│   ├── RIDE.md                 # Ride mode physics contract + usage / track file / telemetry reference
│   ├── ARCHITECTURE.md         # This file
│   └── superpowers/plans/      # Implementation plans
├── tests/                      # Pytest Test Suite (403 tests)
│   ├── golden/                 # Baseline golden XML & JSON snapshots (incl. baseline_bike_ride.xml)
│   ├── test_*.py               # Geometry, kinematics, physics, MJCF, playground, photo fit
│   ├── test_terrain.py, test_road_shapes.py, test_road_generator.py      # terrain package
│   └── test_ride_*.py          # model, field sizing, equilibrium, controllers, full traverse,
│                               #   telemetry, invariants, plots, CLI
└── tools/                      # Offline Calibration & Photographic Fitting Tools
    ├── fit_hardpoints.py       # Photographic hardpoint optimizer
    ├── photo_reference.py      # Photo contour analyzer
    └── render_comparison.py    # MuJoCo vs photograph optical alignment checker
```

---

## 3. Subsystems & Module Descriptions

| Subsystem | Modules | Description |
|---|---|---|
| **Geometry** | `specs.py`, `hardpoints.py`, `validation.py` | Defines bike specs and fixed hardpoints ($P_0, P_5, P_7, P_8, P_9, P_{10}, P_{11}, P_{\text{HT\_bot}}, P_{\text{FA}}, BB$) in MuJoCo coordinate frame (+X forward, +Z up). |
| **Kinematics** | `solver.py`, `curves.py` | Solves the 4-bar linkage ($P_0 \to P_2 \to P_3 \to P_5$) analytically for chainstay angle, rocker rotation, yoke displacement, rear axle path, and leverage ratio with zero phantom stroke at rest ($S_0 = 0$). |
| **Physics** | `air_spring.py`, `coil_shock.py`, `damper.py`, `mass.py`, `tuning.py` | Pure calculators: thermodynamic air spring, linear coil with bottom-out bumper, dual-stage hydraulic dampers with blow-off and HBO, component masses / CG / axle loads, sag-target spring fitting. Never touch `mjModel`/`mjData`. |
| **Terrain** | `obstacles.py`, `wheelpath.py`, `profile.py`, `road.py`, `trackfile.py`, `presets.py`, `heightfield.py` | Road geometry for ride mode: obstacle catalogue at trail and road scale, rolling-wheel envelope, `TrackSpec` assembly with datum carry and overlap rejection, seeded rough-road generator, TOML track files, preset registry, heightfield rasterization with a fixed default grid and length-derived larger grids. Imports no MuJoCo and no other `bike_sim` package. |
| **MuJoCo** | `assets.py`, `frame.py`, `linkage.py`, `rear_linkage.py`, `steering_fork.py`, `drivetrain.py`, `environment.py`, `terrain.py`, `rider.py`, `actuators.py`, `sensors.py`, `builder.py`, `exporter.py` | Decomposed XML generator creating valid MJCF models for four modes; every ride-mode difference is gated on `mode == "ride"` so the three older golden baselines stay byte-identical. |
| **Simulation** | `playground.py`, `camera.py`, `controllers.py`, `equilibrium.py`, `ride_sim.py`, `ride/*` | Test-stand runner; ride orchestrator sequencing suspension → rolling resistance → cruise → brakes → virtual rider → crash check → `mj_step` → contact query, with solved static equilibrium, interactive session/viewer, telemetry recorder and summary metrics. The only layer that writes `mjData`. |
| **Visualization** | `plots.py`, `tables.py`, `dyno_plot.py`, `ride_plots.py` | Publication-ready matplotlib diagrams, dyno curves, ride telemetry figures, track preview, ASCII tables. Agg backend, headless-safe. |
| **CLI** | `main.py`, `playground.py`, `export.py`, `ride.py` | `bike-sim`, `bike-playground`, `bike-export`, `bike-ride` console scripts wired in `pyproject.toml`. |

---

## 4. CLI Commands & Usage

All commands should be executed with `uv run`:

```bash
# 1. Run full kinematics and geometry analysis + print tables:
uv run bike-sim --print-table

# 2. Generate publication-quality plots:
uv run bike-sim --plot --plot-damper

# 3. Export MuJoCo XML models and JSON coordinates to output/models/:
uv run bike-export

# 4. Launch interactive 2D suspension test stand playground:
uv run bike-playground

# 5. Launch test stand playground via main CLI:
uv run bike-sim --playground

# 6. Ride mode: viewer on the default rough road / headless measured run / track tooling
uv run bike-ride
uv run bike-ride --headless --track road_broken --speed 30 --seed 3
uv run bike-ride --track my_road.toml --preview
uv run bike-ride --dump-track road_worn > my_road.toml
```

---

## 5. Development & Testing Rules for AI Agents

1. **Always use `uv`**:
   - Run tests: `uv run pytest`
   - Run commands: `uv run python ...`
2. **Never break existing tests**:
   - All tests in `tests/` must pass at 100% parity.
3. **Keep output clean**:
   - All generated `.png` plots belong in `output/plots/`.
   - All generated `.xml` and `.json` model exports belong in `output/models/`.
   - Never write ephemeral artifacts into the repo root.
4. **Link Invariant Conservation**:
   - Rigid link errors between $P_0, P_1, P_2, P_3, P_4, P_5, P_6, P_{12}$ must remain $< 10^{-10}\text{ mm}$.
   - Shock stroke at 0 mm travel must remain strictly $0.0\text{ mm}$ (no phantom stroke).
5. **Layering** (enforced by review):
   - `bike_sim.terrain` imports no MuJoCo and no other `bike_sim` package.
   - `bike_sim.physics` modules are pure calculators; only `bike_sim.sim` mutates `mjData`.
   - Golden baselines are never regenerated to make a test pass; a shared-builder change that alters one was not gated on its mode.
6. **Ride-mode numbers are measured, then recorded**: test tolerances and documented figures (sag, travel limits, contact dropout rates) come from a run whose value is written into the docstring, never invented to pass. Heightfield contact is chaotic, so ride tests assert bounds and invariants, not traces.
7. **Ride artifacts** go to `output/ride/<track>_<speed>_s<seed>/`, gitignored.
