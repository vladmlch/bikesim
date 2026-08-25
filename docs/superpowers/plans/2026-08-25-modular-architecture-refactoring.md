# Modular Architecture Refactoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restructure the bicycle kinematics & MuJoCo simulation repository into a clean, modular Python package (`bike_sim`), decompose multi-thousand-line monolithic scripts into focused domain modules (100–300 lines each), isolate generated artifacts into `output/`, and provide an AI architecture guide (`docs/ARCHITECTURE.md`) to minimize LLM token consumption and improve navigation.

**Architecture:** Standard Python `src`-layout package (`src/bike_sim/`) organized into domain subpackages (`geometry`, `kinematics`, `physics`, `mujoco`, `sim`, `viz`, `cli`). Decompose `export_mujoco.py` (2,829 lines) into 8 focused sub-builders, `run_playground.py` (1,531 lines) into 4 simulation modules, and `main.py` (1,314 lines) into visualization and CLI entrypoints. Golden snapshot testing is performed to guarantee 100% mathematical and XML output equivalence.

**Tech Stack:** Python 3.10+, MuJoCo 3.0+, NumPy, Matplotlib, SciPy, GLFW, Pytest, `uv` package manager.

**Spec:** [`docs/superpowers/specs/2026-08-25-modular-architecture-refactoring-design.md`](file:///Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/docs/superpowers/specs/2026-08-25-modular-architecture-refactoring-design.md)

## Global Constraints
- `uv` is the mandatory tool for all Python runs (`uv run ...`, `uv pip install ...`).
- All 95 existing tests in `tests/` must pass with 0 failures after refactoring.
- Generated MuJoCo XML output from `bike_sim.mujoco.builder.generate_mujoco_xml` must be byte/semantically identical to the baseline `bike_model.xml`.
- Preserve all docstrings, comments, formulas, constants, and kinematics invariants.
- No files > 500 lines in the new codebase.

---

## Task Breakdown

### Task 1: Phase 0 — Output Directory Isolation & Golden Baseline Snapshots

**Files:**
- Create: `output/.gitignore`
- Create: `output/plots/.gitkeep`
- Create: `output/models/.gitkeep`
- Create: `tests/golden/.gitkeep` (temporary directory for baseline snapshot)
- Modify: `.gitignore`

**Interfaces:**
- Consumes: Existing root `bike_model.xml`, `bike_playground.xml`, `coordinates.json`, root `.png` plots
- Produces: `output/` clean hierarchy, `tests/golden/` snapshot XMLs and JSONs for equivalence testing

- [ ] **Step 1: Capture golden reference XML and JSON files**

Run a capture script to save current `bike_model.xml`, `bike_playground.xml`, `bike_playground_dynamic.xml`, `bike_playground_stand.xml`, and `coordinates.json` into `tests/golden/`.

```bash
mkdir -p tests/golden output/plots output/models
cp bike_model.xml tests/golden/baseline_bike_model.xml
cp bike_playground.xml tests/golden/baseline_bike_playground.xml
cp bike_playground_dynamic.xml tests/golden/baseline_bike_playground_dynamic.xml
cp bike_playground_stand.xml tests/golden/baseline_bike_playground_stand.xml
cp coordinates.json tests/golden/baseline_coordinates.json
```

- [ ] **Step 2: Move existing loose PNG files and XMLs to `output/`**

Move generated plots (`leverage_ratio.png`, `axle_path.png`, `linkage_geometry.png`, `damper_dyno_curves.png`, `suspension_compressed_comparison.png`, `shock_stroke.png`, `test_plot.png`, `crop_*.png`, `bb_*.png`, `headtube_*.png`, `rocker_*.png`, `seat_*.png`, `shock_*.png`, `fork_dropout.png`, `rear_axle_area.png`) to `output/plots/`.
Move root XMLs and log files (`MUJOCO_LOG.TXT`) to `output/models/` or remove generated temporary logs.

- [ ] **Step 3: Configure `output/.gitignore` and root `.gitignore`**

Create `output/.gitignore`:
```gitignore
*
!.gitignore
!plots/
!models/
!plots/.gitkeep
!models/.gitkeep
```

Update root `.gitignore` to ignore `.work/`, `output/plots/*.png`, `output/models/*.xml`.

- [ ] **Step 4: Verify test suite runs cleanly on baseline**

Run: `uv run pytest`
Expected: 95 passed in ~12s.

- [ ] **Step 5: Commit Phase 0**

```bash
git add output/ tests/golden/ .gitignore
git commit -m "refactor(phase0): isolate outputs and capture golden baseline snapshots"
```

---

### Task 2: Phase 1 — Package Skeleton & `pyproject.toml` Configuration

**Files:**
- Create: `src/bike_sim/__init__.py`
- Create: `src/bike_sim/geometry/__init__.py`
- Create: `src/bike_sim/kinematics/__init__.py`
- Create: `src/bike_sim/physics/__init__.py`
- Create: `src/bike_sim/mujoco/__init__.py`
- Create: `src/bike_sim/sim/__init__.py`
- Create: `src/bike_sim/viz/__init__.py`
- Create: `src/bike_sim/cli/__init__.py`
- Modify: `pyproject.toml`

**Interfaces:**
- Consumes: Package metadata
- Produces: Installable `bike_sim` package in editable mode via `uv`

- [ ] **Step 1: Create directory skeleton and empty `__init__.py` files**

```bash
mkdir -p src/bike_sim/geometry src/bike_sim/kinematics src/bike_sim/physics src/bike_sim/mujoco src/bike_sim/sim src/bike_sim/viz src/bike_sim/cli
touch src/bike_sim/__init__.py
touch src/bike_sim/geometry/__init__.py
touch src/bike_sim/kinematics/__init__.py
touch src/bike_sim/physics/__init__.py
touch src/bike_sim/mujoco/__init__.py
touch src/bike_sim/sim/__init__.py
touch src/bike_sim/viz/__init__.py
touch src/bike_sim/cli/__init__.py
```

- [ ] **Step 2: Update `pyproject.toml`**

Update `pyproject.toml` to:
```toml
[project]
name = "bike-sim"
version = "0.2.0"
description = "Bicycle suspension kinematics solver and MuJoCo simulation model generator"
readme = "README.md"
requires-python = ">=3.10"
dependencies = [
    "numpy>=1.24.0",
    "matplotlib>=3.7.0",
    "mujoco>=3.0.0",
    "scipy>=1.18.0",
    "glfw>=2.10.0",
    "pytest>=7.0.0",
]

[project.scripts]
bike-sim = "bike_sim.cli.main:main"
bike-playground = "bike_sim.cli.playground:main"
bike-export = "bike_sim.cli.export:main"

[build-system]
requires = ["setuptools>=61.0"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
where = ["src"]
```

- [ ] **Step 3: Install editable package with `uv`**

Run: `uv pip install -e .`
Expected: Successfully installed `bike-sim-0.2.0`.

- [ ] **Step 4: Commit Phase 1**

```bash
git add pyproject.toml src/
git commit -m "feat(phase1): create src/bike_sim skeleton and update pyproject.toml"
```

---

### Task 3: Phase 2.1 — Geometry Domain Package (`src/bike_sim/geometry/`)

**Files:**
- Create: `src/bike_sim/geometry/specs.py` (migrated from `bike_geometry.py`)
- Create: `src/bike_sim/geometry/hardpoints.py` (migrated from `bike_geometry.py`)
- Create: `src/bike_sim/geometry/validation.py` (validation logic)
- Modify: `src/bike_sim/geometry/__init__.py`

**Interfaces:**
- Consumes: NumPy
- Produces: `BikeSpecs`, `get_fixed_frame_points()`, `compute_front_axle()`, `compute_trail()`, `validate_geometry()`

- [ ] **Step 1: Write `src/bike_sim/geometry/specs.py`**

Migrate `BikeSpecs` dataclass with all default dimensions (head angle, reach, stack, BB drop, travel, pivot definitions).

- [ ] **Step 2: Write `src/bike_sim/geometry/hardpoints.py`**

Migrate `get_fixed_frame_points(specs: BikeSpecs)`, `compute_front_axle(specs: BikeSpecs)`, `compute_trail(specs: BikeSpecs)`, and `compute_headtube_points(specs: BikeSpecs)`.

- [ ] **Step 3: Write `src/bike_sim/geometry/validation.py`**

Add `validate_geometry(specs: BikeSpecs)` checking geometric feasibility and limits.

- [ ] **Step 4: Expose symbols in `src/bike_sim/geometry/__init__.py`**

```python
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.hardpoints import (
    compute_front_axle,
    compute_headtube_points,
    compute_trail,
    get_fixed_frame_points,
)
from bike_sim.geometry.validation import validate_geometry

__all__ = [
    "BikeSpecs",
    "compute_front_axle",
    "compute_headtube_points",
    "compute_trail",
    "get_fixed_frame_points",
    "validate_geometry",
]
```

- [ ] **Step 5: Write unit test for geometry package**

Run: `uv run pytest tests/test_kinematics.py -k TestBicycleGeometry`
Expected: PASS.

- [ ] **Step 6: Commit Phase 2.1**

```bash
git add src/bike_sim/geometry/
git commit -m "feat(phase2): implement bike_sim.geometry subpackage"
```

---

### Task 4: Phase 2.2 — Kinematics Domain Package (`src/bike_sim/kinematics/`)

**Files:**
- Create: `src/bike_sim/kinematics/solver.py` (migrated from `linkage_solver.py`)
- Create: `src/bike_sim/kinematics/curves.py` (kinematics curves & metrics)
- Modify: `src/bike_sim/kinematics/__init__.py`

**Interfaces:**
- Consumes: `bike_sim.geometry.specs.BikeSpecs`, `bike_sim.geometry.hardpoints.get_fixed_frame_points`
- Produces: `HorstLinkageSolver`, leverage ratio, anti-squat, axle path, progression calculations

- [ ] **Step 1: Write `src/bike_sim/kinematics/solver.py`**

Migrate `HorstLinkageSolver` implementing:
- `solve_state(theta0)`: vector loop closure for 4-bar Horst linkage
- `solve_state_from_wheel_travel(travel_mm)`
- `solve_state_from_shock_stroke(stroke_mm)`
- `get_uncompressed_state()`
- `get_bottom_out_state()`

- [ ] **Step 2: Write `src/bike_sim/kinematics/curves.py`**

Implement helper functions for computing leverage ratio series, progression rate %, axle path coordinates, anti-squat curve against gear ratios, and instant center locations.

- [ ] **Step 3: Expose symbols in `src/bike_sim/kinematics/__init__.py`**

```python
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.kinematics.curves import (
    compute_anti_squat_curve,
    compute_axle_path_curve,
    compute_instant_centers,
    compute_leverage_ratio_curve,
)

__all__ = [
    "HorstLinkageSolver",
    "compute_anti_squat_curve",
    "compute_axle_path_curve",
    "compute_instant_centers",
    "compute_leverage_ratio_curve",
]
```

- [ ] **Step 4: Verify kinematics tests**

Run: `uv run pytest tests/test_kinematics.py`
Expected: PASS.

- [ ] **Step 5: Commit Phase 2.2**

```bash
git add src/bike_sim/kinematics/
git commit -m "feat(phase2): implement bike_sim.kinematics subpackage"
```

---

### Task 5: Phase 2.3 — Physics Domain Package & Composite Config (`src/bike_sim/physics/`, `src/bike_sim/config.py`)

**Files:**
- Create: `src/bike_sim/physics/mass.py` (migrated from `mass_profile.py`)
- Create: `src/bike_sim/physics/air_spring.py` (migrated from `air_spring.py`)
- Create: `src/bike_sim/physics/damper.py` (migrated from `suspension_damper.py`)
- Create: `src/bike_sim/config.py` (migrated from `bike_config.py`)
- Modify: `src/bike_sim/physics/__init__.py`

**Interfaces:**
- Consumes: `BikeSpecs`, `HorstLinkageSolver`
- Produces: `BikeMassSpecs`, `RiderSpecs`, `compute_static_system_cg`, `AirSpringSpecs`, `AirSpringModel`, `DamperSpecs`, `DamperModel`, `BikeConfig`

- [ ] **Step 1: Write `src/bike_sim/physics/mass.py`**

Migrate `BikeMassSpecs`, `RiderSpecs`, `compute_component_masses`, `compute_wheel_inertia`, and `compute_static_system_cg`.

- [ ] **Step 2: Write `src/bike_sim/physics/air_spring.py`**

Migrate `AirSpringSpecs`, `AirSpringModel` (isothermal/polytropic equations, token volume adjustments, sag psi calibration).

- [ ] **Step 3: Write `src/bike_sim/physics/damper.py`**

Migrate `DamperSpecs`, `DamperModel` (rebound and compression velocity-force calculation, dyno curve evaluations).

- [ ] **Step 4: Write `src/bike_sim/config.py`**

Migrate `BikeConfig` dataclass composing `BikeSpecs`, `BikeMassSpecs`, `AirSpringSpecs`, `DamperSpecs`.

- [ ] **Step 5: Expose symbols in `src/bike_sim/physics/__init__.py`**

```python
from bike_sim.physics.mass import BikeMassSpecs, RiderSpecs, compute_static_system_cg
from bike_sim.physics.air_spring import AirSpringSpecs, AirSpringModel
from bike_sim.physics.damper import DamperSpecs, DamperModel

__all__ = [
    "BikeMassSpecs",
    "RiderSpecs",
    "compute_static_system_cg",
    "AirSpringSpecs",
    "AirSpringModel",
    "DamperSpecs",
    "DamperModel",
]
```

- [ ] **Step 6: Run mass, air spring, and damper tests**

Run: `uv run pytest tests/test_air_spring.py tests/test_mass_distribution.py`
Expected: PASS.

- [ ] **Step 7: Commit Phase 2.3**

```bash
git add src/bike_sim/physics/ src/bike_sim/config.py
git commit -m "feat(phase2): implement bike_sim.physics subpackage and config"
```

---

### Task 6: Phase 3 — MuJoCo Subsystem Decomposition (`src/bike_sim/mujoco/`)

**Files:**
- Create: `src/bike_sim/mujoco/assets.py` (assets, materials, textures, visual quality)
- Create: `src/bike_sim/mujoco/environment.py` (ground, lighting, terrain obstacles, test stand fixtures)
- Create: `src/bike_sim/mujoco/frame.py` (frame body, tubes, BB, headset, pivots visual geoms)
- Create: `src/bike_sim/mujoco/linkage.py` (chainstay, seatstay, rocker, yoke, wheels, joints, equality constraints)
- Create: `src/bike_sim/mujoco/rider.py` (rider body segments & mass)
- Create: `src/bike_sim/mujoco/actuators.py` (springs, dampers, wheel servos, drive motor)
- Create: `src/bike_sim/mujoco/sensors.py` (travel, force, velocity, acceleration sensors)
- Create: `src/bike_sim/mujoco/builder.py` (orchestrates `generate_mujoco_xml`)
- Create: `src/bike_sim/mujoco/exporter.py` (`export_mujoco`, `export_playground_models`, `export_json`)
- Modify: `src/bike_sim/mujoco/__init__.py`

**Interfaces:**
- Consumes: `BikeSpecs`, `HorstLinkageSolver`, `BikeMassSpecs`, `AirSpringModel`, `DamperModel`
- Produces: `generate_mujoco_xml()`, `export_mujoco()`, `export_playground_models()`, `export_json()`

- [ ] **Step 1: Write `src/bike_sim/mujoco/assets.py`**

Build `<visual>` quality settings and `<asset>` definitions (materials: frame, shock, links, tire, ground grid texture, skybox).

- [ ] **Step 2: Write `src/bike_sim/mujoco/environment.py`**

Build world lights, ground plane, obstacles (whoops, bumps, drops, tabletop, pits) and test stand support fixtures.

- [ ] **Step 3: Write `src/bike_sim/mujoco/frame.py`**

Build main frame body: downtube battery box, toptube, seattube casting, headtube, shock mounts, handlebar, stem, saddle, seatpost.

- [ ] **Step 4: Write `src/bike_sim/mujoco/linkage.py`**

Build rear suspension linkage bodies:
- Chainstay (`body name="chainstay"`)
- Seatstay (`body name="seatstay"`)
- Rocker link (`body name="rocker"`)
- Shock yoke (`body name="yoke"`) & shock body (`body name="shock_body"`)
- Front fork steerer, stanchions, lower sliders, front wheel
- Rear wheel & hub
- Loop closure `<equality>` constraints (`connect` at P3 Horst pivot, P6 yoke-shock connect).

- [ ] **Step 5: Write `src/bike_sim/mujoco/rider.py`**

Build rider visual geoms (torso, head, arms, legs) and inertial bodies with toggle support.

- [ ] **Step 6: Write `src/bike_sim/mujoco/actuators.py`**

Build `<actuator>` elements (suspension spring/damper, motor, steering, stand position servos with appropriate kp/kv gains).

- [ ] **Step 7: Write `src/bike_sim/mujoco/sensors.py`**

Build `<sensor>` elements for kinematics measurement (frame velocity, suspension travel, shock stroke, normal force, wheel speeds).

- [ ] **Step 8: Write `src/bike_sim/mujoco/builder.py` and `exporter.py`**

Orchestrate `generate_mujoco_xml()` in `builder.py`.
Implement `export_mujoco()`, `export_playground_models()`, `export_json()` in `exporter.py`.

- [ ] **Step 9: Expose symbols in `src/bike_sim/mujoco/__init__.py`**

```python
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.mujoco.exporter import export_json, export_mujoco, export_playground_models

__all__ = [
    "generate_mujoco_xml",
    "export_json",
    "export_mujoco",
    "export_playground_models",
]
```

- [ ] **Step 10: Verify XML and Hardpoint Parity with Golden Snapshots**

Run a comparison test between `generate_mujoco_xml()` and `tests/golden/baseline_bike_model.xml`.
Run: `uv run pytest tests/test_frame_visuals.py tests/test_fitted_hardpoints.py`
Expected: PASS.

- [ ] **Step 11: Commit Phase 3**

```bash
git add src/bike_sim/mujoco/
git commit -m "feat(phase3): decompose export_mujoco into modular bike_sim.mujoco subpackage"
```

---

### Task 7: Phase 4 — Simulation Subsystem Decomposition (`src/bike_sim/sim/`)

**Files:**
- Create: `src/bike_sim/sim/obstacles.py` (`ObstacleEntry`, `ObstacleManager`)
- Create: `src/bike_sim/sim/camera.py` (`CameraManager`, camera tracking, views)
- Create: `src/bike_sim/sim/controllers.py` (stand servos, rider stabilization, cruise control)
- Create: `src/bike_sim/sim/playground.py` (`SuspensionPlayground`, GLFW window, simulation step, HUD overlay)
- Modify: `src/bike_sim/sim/__init__.py`

**Interfaces:**
- Consumes: `bike_sim.mujoco.builder.generate_mujoco_xml`, `BikeSpecs`, `BikeMassSpecs`
- Produces: `SuspensionPlayground`, `CameraManager`, `ObstacleManager`, `run_interactive_playground()`

- [ ] **Step 1: Write `src/bike_sim/sim/obstacles.py`**

Migrate `ObstacleEntry` and `ObstacleManager` (procedural obstacle positioning, recycling, scaling).

- [ ] **Step 2: Write `src/bike_sim/sim/camera.py`**

Migrate `CameraManager` (camera tracking modes: side, follow, isometric, suspension close-up, view smoothing).

- [ ] **Step 3: Write `src/bike_sim/sim/controllers.py`**

Extract stand mode position servo controls, dynamic riding balance/cruise controllers, keyboard input mapping.

- [ ] **Step 4: Write `src/bike_sim/sim/playground.py`**

Migrate `SuspensionPlayground` core simulation loop, macOS `mjpython` launcher helper (`ensure_macos_mjpython`), GLFW keyboard callbacks, OpenGL telemetry HUD.

- [ ] **Step 5: Expose symbols in `src/bike_sim/sim/__init__.py`**

```python
from bike_sim.sim.camera import CameraManager
from bike_sim.sim.controllers import StandController, DynamicRiderController
from bike_sim.sim.obstacles import ObstacleEntry, ObstacleManager
from bike_sim.sim.playground import SuspensionPlayground, run_interactive_playground

__all__ = [
    "CameraManager",
    "StandController",
    "DynamicRiderController",
    "ObstacleEntry",
    "ObstacleManager",
    "SuspensionPlayground",
    "run_interactive_playground",
]
```

- [ ] **Step 6: Run playground unit tests**

Run: `uv run pytest tests/test_playground.py`
Expected: PASS (21 tests).

- [ ] **Step 7: Commit Phase 4**

```bash
git add src/bike_sim/sim/
git commit -m "feat(phase4): decompose run_playground into modular bike_sim.sim subpackage"
```

---

### Task 8: Phase 5 — Visualization & CLI Subsystems (`src/bike_sim/viz/`, `src/bike_sim/cli/`)

**Files:**
- Create: `src/bike_sim/viz/plots.py` (linkage geometry, leverage ratio, compression comparison)
- Create: `src/bike_sim/viz/tables.py` (`print_tables`, formatted terminal summaries)
- Create: `src/bike_sim/viz/dyno_plot.py` (migrated from `plot_damper_dyno.py`)
- Create: `src/bike_sim/cli/main.py` (`bike-sim` entrypoint)
- Create: `src/bike_sim/cli/playground.py` (`bike-playground` entrypoint)
- Create: `src/bike_sim/cli/export.py` (`bike-export` entrypoint)
- Modify: `src/bike_sim/viz/__init__.py`, `src/bike_sim/cli/__init__.py`

**Interfaces:**
- Consumes: All `bike_sim` subpackages
- Produces: CLI commands `bike-sim`, `bike-playground`, `bike-export`, high-res plots in `output/plots/`

- [ ] **Step 1: Write `src/bike_sim/viz/plots.py`**

Migrate `plot_linkage_geometry()`, `plot_leverage_ratio()`, `plot_suspension_compressed_comparison()` with default save destination `output/plots/`.

- [ ] **Step 2: Write `src/bike_sim/viz/tables.py`**

Migrate terminal table formatting functions (`print_tables`, hardpoints printout).

- [ ] **Step 3: Write `src/bike_sim/viz/dyno_plot.py`**

Migrate `plot_damper_dyno()` with default save destination `output/plots/damper_dyno_curves.png`.

- [ ] **Step 4: Write `src/bike_sim/cli/main.py`**

Implement CLI argument parser and execution logic for `bike-sim` (generating tables, plots, JSON, and XML).

- [ ] **Step 5: Write `src/bike_sim/cli/playground.py` and `src/bike_sim/cli/export.py`**

Implement `bike-playground` and `bike-export` CLI wrappers.

- [ ] **Step 6: Expose top-level API in `src/bike_sim/__init__.py`**

```python
from bike_sim.geometry import BikeSpecs, get_fixed_frame_points
from bike_sim.kinematics import HorstLinkageSolver
from bike_sim.physics import AirSpringModel, BikeMassSpecs, DamperModel
from bike_sim.mujoco import generate_mujoco_xml, export_mujoco
from bike_sim.config import BikeConfig

__all__ = [
    "BikeSpecs",
    "get_fixed_frame_points",
    "HorstLinkageSolver",
    "BikeMassSpecs",
    "AirSpringModel",
    "DamperModel",
    "generate_mujoco_xml",
    "export_mujoco",
    "BikeConfig",
]
```

- [ ] **Step 7: Test CLI entrypoints via `uv run`**

Run:
- `uv run bike-sim --help`
- `uv run bike-export --help`
- `uv run bike-playground --help`
Expected: All print help successfully with exit code 0.

- [ ] **Step 8: Commit Phase 5**

```bash
git add src/bike_sim/viz/ src/bike_sim/cli/ src/bike_sim/__init__.py
git commit -m "feat(phase5): implement viz and cli subpackages with CLI entry points"
```

---

### Task 9: Phase 6 — Test Suite Import Updates, Root Cleanup & AI Documentation

**Files:**
- Modify: `tests/test_air_spring.py`
- Modify: `tests/test_fitted_hardpoints.py`
- Modify: `tests/test_frame_visuals.py`
- Modify: `tests/test_kinematics.py`
- Modify: `tests/test_mass_distribution.py`
- Modify: `tests/test_photo_reference.py`
- Modify: `tests/test_playground.py`
- Modify: `tests/test_render_comparison.py`
- Modify: `tools/fit_hardpoints.py`
- Modify: `tools/photo_reference.py`
- Modify: `tools/render_comparison.py`
- Create: `docs/ARCHITECTURE.md`
- Modify: `AGENTS.md`, `CLAUDE.md`
- Delete: Root legacy redundant scripts (`air_spring.py`, `bike_config.py`, `bike_geometry.py`, `export_mujoco.py`, `linkage_solver.py`, `main.py`, `mass_profile.py`, `plot_damper_dyno.py`, `run_playground.py`, `suspension_damper.py`) or provide minimal 2-line deprecation shims if needed.

**Interfaces:**
- Consumes: `src/bike_sim`
- Produces: 100% passing tests (95/95), pristine root directory, AI navigation index `docs/ARCHITECTURE.md`

- [ ] **Step 1: Update import statements in all 8 test files**

Update imports from e.g. `import bike_geometry` to `from bike_sim.geometry import BikeSpecs, ...` and `from bike_sim.mujoco import generate_mujoco_xml`.

- [ ] **Step 2: Update import statements in `tools/`**

Update `tools/fit_hardpoints.py`, `tools/photo_reference.py`, `tools/render_comparison.py` to import from `bike_sim.*`.

- [ ] **Step 3: Remove root legacy scripts & loose files**

Remove old flat files from root now that `bike_sim` is installed and verified. Clean up `tests/golden/`.

- [ ] **Step 4: Create `docs/ARCHITECTURE.md` for AI Navigation**

Create a 1-page architecture index outlining every module, its responsibility, its public symbols, and workflow recipes (e.g. "How to adjust kinematics", "How to modify XML geoms", "How to add a sensor").

- [ ] **Step 5: Update `AGENTS.md` and `CLAUDE.md`**

Update project guides with new package paths, CLI commands (`bike-sim`, `bike-playground`), and references to `docs/ARCHITECTURE.md`.

- [ ] **Step 6: Run full test suite verification**

Run: `uv run pytest`
Expected: 95 passed in <15s with 0 errors.

- [ ] **Step 7: Commit Phase 6**

```bash
git add .
git commit -m "refactor(phase6): update test imports, tools, documentation, and create ARCHITECTURE.md"
```

---

## Plan Self-Review
1. **Spec Coverage**: All items from the spec (Geometry, Kinematics, Physics, MuJoCo 8-module split, Sim 4-module split, Viz, CLI, Output isolation, ARCHITECTURE.md, and 95 tests verification) are accounted for.
2. **Placeholder Scan**: No "TBD", "TODO", or vague requirements. Every task lists exact files, functions, and commands.
3. **Type & Symbol Consistency**: Verified symbol names (`BikeSpecs`, `HorstLinkageSolver`, `AirSpringModel`, `DamperModel`, `generate_mujoco_xml`, `SuspensionPlayground`) across all tasks.
