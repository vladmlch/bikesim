# Uphill Climb and Wheelspin Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an uphill climbing simulation (`climb_steps` on `hardpack`) with dynamic multi-speed cassette shifting (10–51T) and a dedicated wheelspin analysis tool using the pneumatic tyre model.

**Architecture:** A new continuous obstacle `SteppedClimb` models progressive grades ($5\% \to 25\%$) with smoothed transitions; `HeightFieldSpec.for_track` is generalized to vertically size heightfields for climbing; `PedalDrivetrain` is augmented with a 12-speed cassette and an adaptive cadence-based auto-shifter; and `tools/analyze_wheelspin.py` captures and visualizes cadence-synchronized wheelspin.

**Tech Stack:** Python 3.12, MuJoCo, NumPy, SciPy, Matplotlib, `uv`, pytest.

**Spec:** [docs/superpowers/specs/2026-09-28-uphill-climb-wheelspin-design.md](file:///Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2/docs/superpowers/specs/2026-09-28-uphill-climb-wheelspin-design.md)

## Global Constraints

- Always use `uv` for running tests and commands (`uv run ...`).
- Pure geometry stays pure: `obstacles.py`, `profile.py`, `heightfield.py` do not import MuJoCo.
- Existing golden baselines and default presets must remain bit-identical.
- All tests in `tests/` must pass without regressions.
- Preserve all existing comments and docstrings.

---

### Task 1: Heightfield Dynamic Vertical Sizing

**Files:**
- Modify: `src/bike_sim/terrain/heightfield.py:188-215`
- Test: `tests/test_ride_field_sizing.py`

**Interfaces:**
- `HeightFieldSpec.for_track(track: TrackSpec) -> HeightFieldSpec`: currently only sizes `track_length_m`. Must also size `elevation_m` and `datum_z_m` if `profile_extent(track)` exceeds the default envelope.
- `FIELD = HeightFieldSpec()` remains unchanged as the default 120m field.

- [ ] **Step 1: Write the failing unit test**

In `tests/test_ride_field_sizing.py`, add `test_heightfield_sizes_vertical_extent_for_climb`:
```python
def test_heightfield_sizes_vertical_extent_for_climb():
    # A track that climbs 10 metres
    track = TrackSpec(
        name="climb_test",
        length_m=100.0,
        obstacles=[SquareEdge(start_m=10.0, height_m=10.0, ledge_length_m=80.0)],
    )
    spec = HeightFieldSpec.for_track(track)
    assert not spec.is_default
    extent = profile_extent(track)
    assert spec.fits(extent)
    assert spec.max_profile_m >= 10.0
```

- [ ] **Step 2: Run test to confirm it fails**

Run `uv run pytest tests/test_ride_field_sizing.py` and verify failure.

- [ ] **Step 3: Implement dynamic vertical sizing in `HeightFieldSpec.for_track`**

In `src/bike_sim/terrain/heightfield.py`:
Update `for_track` to inspect `profile_extent(track, resolution_m=default.resolution_m)`:
If `extent[0] < default.min_profile_m` or `extent[1] > default.max_profile_m`:
Compute `margin_below = 1.0`, `margin_above = 1.0`.
Compute `datum_z_m = max(default.datum_z_m, math.ceil((-extent[0] + margin_below) * 10) / 10)`.
Compute `needed_elevation = math.ceil((datum_z_m + extent[1] + margin_above) * 10) / 10`.
Return a field with adjusted `radius_x_m`, `ncol`, `datum_z_m`, and `elevation_m`.

- [ ] **Step 4: Verify tests pass**

Run `uv run pytest tests/test_ride_field_sizing.py tests/test_terrain.py`.

---

### Task 2: Obstacle Primitive `SteppedClimb` and Track Preset `climb_steps`

**Files:**
- Modify: `src/bike_sim/terrain/obstacles.py`
- Modify: `src/bike_sim/terrain/presets.py`
- Modify: `src/bike_sim/terrain/__init__.py`
- Test: `tests/test_road_shapes.py`, `tests/test_terrain.py`

**Interfaces:**
- `SteppedClimb(Obstacle)`:
  - `steps: Tuple[Tuple[float, float], ...]`: list of `(grade_pct, length_m)`
  - `transition_m: float = 3.0`: transition zone between grades
  - `elevation(s: np.ndarray) -> np.ndarray`: continuous monotonic climb
  - `datum_shift_m: float`: total vertical elevation gained
- `climb_steps() -> TrackSpec`: factory in `presets.py` with surface `"hardpack"`.

- [ ] **Step 1: Write unit tests for `SteppedClimb`**

In `tests/test_road_shapes.py`, add tests for `SteppedClimb`:
- Endpoint continuity: `elevation(0) == 0.0`, `elevation(length) == datum_shift_m`.
- Monotonic slope and smoothed transition derivatives.
- Integration in `build_profile`.

- [ ] **Step 2: Run test to confirm failure**

Run `uv run pytest tests/test_road_shapes.py -k SteppedClimb`.

- [ ] **Step 3: Implement `SteppedClimb` and `climb_steps` preset**

In `src/bike_sim/terrain/obstacles.py`:
Implement `SteppedClimb` with cubic Hermite blending across transition zones.
In `src/bike_sim/terrain/presets.py`:
Implement `climb_steps()`:
- Length: 115 m.
- Surface: `hardpack`.
- Steps: 5% (15m) $\to$ 10% (15m) $\to$ 15% (15m) $\to$ 20% (15m) $\to$ 25% (15m).
Register in `PRESETS`.

- [ ] **Step 4: Verify tests pass**

Run `uv run pytest tests/test_road_shapes.py tests/test_terrain.py`.

---

### Task 3: 12-Speed Cassette and Adaptive Cadence Auto-Shifter

**Files:**
- Modify: `src/bike_sim/physics/drivetrain.py`
- Modify: `src/bike_sim/sim/ride/drivetrain.py`
- Create: `tests/test_drivetrain_shifting.py`

**Interfaces:**
- `CASSETTE_12S_TEETH = (10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51)`
- `DrivetrainSpecs`:
  - `cassette: Tuple[int, ...] = CASSETTE_12S_TEETH`
  - `auto_shift: bool = True`
  - `target_cadence_min_rpm: float = 65.0`
  - `target_cadence_max_rpm: float = 85.0`
  - `shift_cooldown_s: float = 0.40`
  - `shift_cut_duration_s: float = 0.20`
- `PedalDrivetrain`:
  - Updates `model.eq_data[self.eq_id, 1] = 1.0 / new_gear_ratio` on shift.
  - Updates crank velocity $\omega_{\text{crank}} = \omega_{\text{wheel}} / R_{\text{new}}$ and calls `_redatum`.
  - Attenuates torque during `shift_cut`.
  - `CrankCommand.gear_teeth: int` recorded for telemetry.

- [ ] **Step 1: Write unit tests for dynamic shifting**

In `tests/test_drivetrain_shifting.py`:
- Test that low cadence under pedal demand triggers downshift (cog teeth increase).
- Test that high cadence triggers upshift (cog teeth decrease).
- Test that shifting re-datums chain equality with zero residual error.
- Test that fixed gearing (`auto_shift=False`) preserves single gear.

- [ ] **Step 2: Run test to confirm failure**

Run `uv run pytest tests/test_drivetrain_shifting.py`.

- [ ] **Step 3: Implement cassette auto-shifting in `physics/drivetrain.py` and `sim/ride/drivetrain.py`**

Implement shifting logic, torque attenuation, equality parameter update, and telemetry channel `gear_teeth`.

- [ ] **Step 4: Verify tests pass**

Run `uv run pytest tests/test_drivetrain_shifting.py tests/test_ride_controllers.py`.

---

### Task 4: CLI Integration and Track Defaults for `climb_steps`

**Files:**
- Modify: `src/bike_sim/cli/ride.py`
- Modify: `src/bike_sim/sim/ride/recorder.py`
- Test: `tests/test_ride_cli.py`

**Interfaces:**
- When `--track climb_steps` is selected:
  - Default `--speed 16`
  - Default `--tyre-model pneumatic`
  - Default `--drive-mode pedelec`
  - Default `--assist turbo`
- `recorder.py`: records `gear_teeth` and `slip_ratio` in `telemetry.csv`.

- [ ] **Step 1: Write CLI tests for `climb_steps`**

In `tests/test_ride_cli.py`:
Assert that `climb_steps` is listed in `available_presets()`, and can be loaded.

- [ ] **Step 2: Implement CLI defaults and recorder channels**

Update `src/bike_sim/cli/ride.py` and `src/bike_sim/sim/ride/recorder.py`.

- [ ] **Step 3: Verify CLI tests pass**

Run `uv run pytest tests/test_ride_cli.py`.

---

### Task 5: Wheelspin Analysis Tool (`tools/analyze_wheelspin.py`)

**Files:**
- Create: `tools/analyze_wheelspin.py`
- Create: `tests/test_analyze_wheelspin.py`

**Interfaces:**
- `run_wheelspin_analysis(track_name: str = "climb_steps", out_dir: Path) -> WheelspinReport`
- Plots:
  - `output/wheelspin/climb_kinematics.png`: 4-panel figure ($V_x$ vs $\omega R$, slip $\kappa$, gear & cadence, slope & torque).
  - `output/wheelspin/summary.json`: summary statistics.

- [ ] **Step 1: Write smoke test for `analyze_wheelspin`**

In `tests/test_analyze_wheelspin.py`:
Run analysis on a short climb track, assert output plots and JSON report are generated.

- [ ] **Step 2: Implement `tools/analyze_wheelspin.py`**

Implement simulation loop, telemetry processing, matplotlib figure rendering, and summary metrics.

- [ ] **Step 3: Verify test passes**

Run `uv run pytest tests/test_analyze_wheelspin.py`.

---

### Task 6: Full Regression Testing & Experiment Run

- [ ] **Step 1: Run complete test suite**

Run `uv run pytest` to ensure all existing and new tests pass.

- [ ] **Step 2: Execute wheelspin analysis experiment**

Run `uv run python -m tools.analyze_wheelspin` and inspect output metrics.
