# `bike_sim` Architecture & Developer Guide

## 1. Overview & Purpose

`bike_sim` is a modular, high-fidelity Python and MuJoCo simulation package for enduro/downhill mountain bike suspension analysis. It models:
- Complete analytical bicycle frame geometry (Reach, Stack, Head Angle, Mullet wheels, BB Drop, Fork Offset, Mechanical Trail).
- Closed-form 4-bar Horst-Link suspension kinematic solving with zero phantom stroke and strict link invariant conservation ($< 10^{-12}\text{ mm}$).
- Non-linear thermodynamic pneumatic fork spring (DebonAir+ dual-chamber adiabatic air spring with volume token tuning).
- Multi-circuit hydrodynamic dampers (RockShox Charger 3 RC2 fork damper & Super Deluxe Ultimate RC2T rear shock with Hydraulic Bottom-Out and Threshold Lockout).
- Physical mass and inertia profiles (full-power eMTB 24.4 kg frame + rider 80 kg toggle).
- MuJoCo MJCF XML procedural compiler across simulation modes (Standard, Stand, Playground).
- Live interactive 2D suspension test stand playground with position servo control, auto-sweep, multi-camera tracking, and live HUD telemetry.

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
│       │   └── solver.py       # Analytical HorstLinkageSolver (closed-loop loop closure)
│       ├── physics/            # Air spring, damper models & mass distribution
│       │   ├── __init__.py
│       │   ├── air_spring.py   # ForkAirSpring thermodynamic pneumatic model
│       │   ├── damper.py       # Charger3Damper, SuperDeluxeDamper, BikeSuspensionSystem
│       │   └── mass.py         # BikeMassSpecs, RiderSpecs, CoM & static axle loads
│       ├── mujoco/             # Modular MuJoCo MJCF XML Builder & Exporter
│       │   ├── __init__.py
│       │   ├── actuators.py    # Actuator XML builder (position servos for stand)
│       │   ├── assets.py       # Textures, materials, visuals XML builder
│       │   ├── builder.py      # Main orchestrator (generate_mujoco_xml)
│       │   ├── environment.py  # Lighting, floor, stand fixtures XML builder
│       │   ├── exporter.py     # File exporter (export_mujoco, export_playground_models, export_json)
│       │   ├── frame.py        # Frame body, tubes, battery, casting, saddle XML builder
│       │   ├── linkage.py      # Steering, fork, chainstay, seatstay, rocker, yoke, shock, constraints
│       │   ├── rider.py        # 3D Rider capsules XML builder
│       │   └── sensors.py      # Telemetry sensors XML builder
│       ├── sim/                # Simulation Runtime & Interactive Playground
│       │   ├── __init__.py
│       │   ├── camera.py       # CameraManager (2D tracking & 3D isometric chase)
│       │   ├── controllers.py  # SuspensionController (qfrc_applied force calculation)
│       │   └── playground.py   # SuspensionPlayground & run_interactive_playground
│       ├── viz/                # Publication Plotting & Terminal ASCII Tables
│       │   ├── __init__.py
│       │   ├── dyno_plot.py    # Damper F-v & HBO dyno curves plotter
│       │   ├── plots.py        # Linkage geometry, leverage ratio, compression comparison
│       │   └── tables.py       # Formatted ASCII tables & coordinate summaries
│       └── cli/                # CLI Entrypoints
│           ├── __init__.py
│           ├── export.py       # `bike-export` entrypoint
│           ├── main.py         # `bike-sim` entrypoint
│           └── playground.py   # `bike-playground` entrypoint
├── tests/                      # Pytest Test Suite
│   ├── golden/                 # Baseline golden XML & JSON snapshots
│   ├── test_air_spring.py
│   ├── test_fitted_hardpoints.py
│   ├── test_frame_visuals.py
│   ├── test_golden_baselines.py
│   ├── test_kinematics.py
│   ├── test_mass_distribution.py
│   ├── test_photo_reference.py
│   ├── test_playground.py
│   └── test_render_comparison.py
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
| **Physics** | `air_spring.py`, `damper.py`, `mass.py` | Thermodynamic pneumatic air spring with DebonAir+ equalization, dual-stage hydraulic dampers with blow-off and HBO, component masses, CG, and axle loads. |
| **MuJoCo** | `assets.py`, `frame.py`, `linkage.py`, `environment.py`, `rider.py`, `actuators.py`, `sensors.py`, `builder.py`, `exporter.py` | Decomposed XML generator creating valid MJCF models without monolithic files. |
| **Simulation** | `playground.py`, `camera.py`, `controllers.py` | Live interactive test stand runner with suspension travel control, auto-sweep, damper tuning, and HUD telemetry. |
| **Visualization** | `plots.py`, `tables.py`, `dyno_plot.py` | Publication-ready matplotlib diagrams, dyno curves, and clean ASCII tabular summaries. |
| **CLI** | `main.py`, `playground.py`, `export.py` | Command-line interfaces wired into `pyproject.toml` console scripts. |

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
