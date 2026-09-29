# Anti-wheelie Research Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the physical bike, articulated rider and road into an offline torque-control research plant.

**Architecture:** Keep one physical force owner and add immutable commands at its input. Compose road grade/material independently of obstacles. A separate research wrapper owns control timing, causal sensor delivery and contact-based experiment metrics without adding stabilizing forces.

**Tech Stack:** Python 3.13, NumPy, SciPy 1.17, MuJoCo 3.12, pytest; existing offline wheels only.

**Spec:** `docs/superpowers/specs/2026-09-29-antiwheelie-research.md`

## Global Constraints
- No dependency downloads.
- No external pitch stabilizer, speed servo, pose repair or load-transfer force in the research plant.
- Existing legacy behavior and old physical API defaults remain compatible.
- Inputs and outputs use SI; positive research pitch is nose-up (opposite the engine's +Y hinge coordinate).
- Default model parameters remain synthetic/unvalidated.

---

### Task 1: Graded, material-zoned roads
**Files:** Create `src/bike_sim/terrain/grade.py`, `src/bike_sim/terrain/research.py`, `tests/test_research_terrain.py`; modify `terrain/profile.py`, `terrain/surface.py`, `terrain/trackfile.py`, `physics/physical_config.py`, `sim/ride/tire_forces.py`, `sim/ride/physical_runtime.py`, `sim/ride/physical_session.py`.
**Interfaces:** `GradeProfile(knots)` integrates piecewise-linear grades; `SurfaceSection(start_m, end_m, surface)` defines half-open material intervals; `TrackSpec.grade_profile` and `.surface_sections` serialize losslessly; `TireBackendConfig.surface_mode='track'` opts into material-resolved compliant contact.
- [ ] Write failing tests for additive elevation, C1 grade transitions, TOML round trips, material boundaries and tire friction selection.
```python
profile = GradeProfile(((0., 0.), (2., .2), (4., .2)))
assert profile.elevation([0., 2., 4.]).tolist() == [0., .2, .6]
```
- [ ] Run `pytest -q tests/test_research_terrain.py` and confirm missing interfaces fail.
- [ ] Implement integrated grade, validated material intervals, codec fields and contact-point material resolution; preserve the configured-friction mode.
- [ ] Rerun new terrain tests plus `test_surface.py`, `test_terrain.py`, `test_road_generator.py`, `test_ride_track.py`, `test_tire_brush.py`.
- [ ] Commit the independently tested road component.

### Task 2: Motor and rider commands
**Files:** Create `physics/rider_posture.py`, `sim/ride/control.py`, `tests/test_research_control.py`, `tests/test_rider_posture_commands.py`; modify `physics/motor.py`, `sim/ride/drivetrain_forces.py`, `sim/ride/rider_control.py`, `sim/ride/physical_runtime.py`, `sim/ride_sim.py`.
**Interfaces:** `RideSimulation.step(control=RideControl(...))`; commands carry optional crank-side `motor_torque_nm`, `motor_limit_nm`, `human_torque_nm`, and `RiderPosture`. `None` retains configured assistance. A posture supplies joint goals, never state assignments or floating-root actuation.
- [ ] Write failing tests for finite/range validation, motor ceilings, brake priority, command-only probes, and unchanged legacy rejection.
```python
command = RideControl(motor_torque_nm=40., motor_limit_nm=12.)
sim.step(control=command)
assert 0. <= sim.physical.drive.last['motor_torque_nm'] <= 12.
```
- [ ] Run `pytest -q tests/test_research_control.py tests/test_rider_posture_commands.py` and confirm absent command support fails.
- [ ] Thread commands through the single force owner; apply existing motor lag, slew, speed/power/torque and battery limits; record requested/limited/delivered torque separately.
- [ ] Add bounded torso/pelvis goals and optional frame-relative hip target to rider IK. Standing removes saddle only from requested support, never from actual contact physics.
- [ ] Rerun command tests plus existing drive, battery, rider and force-ledger regressions.
- [ ] Commit command support.

### Task 3: Contact-based truth and causal observations
**Files:** Create `sim/ride/wheelie.py`, `sim/research/sensors.py`, `sim/research/observations.py`, `tests/test_wheelie_detection.py`, `tests/test_research_sensors.py`.
**Interfaces:** Immutable truth separates road-relative axle pitch, loads, clearances and contact state. Sensor observations contain timestamped inertial/encoder/motor measurements only; contact state and terrain normals are not policy inputs. Sensor delivery advances once per control tick.
- [ ] Test grade-only, front unload, rear lift, flight, sustained wheelie and threshold hysteresis independently.
```python
assert classify_contact(front_load_n=0., rear_load_n=0.,
    front_clearance_m=.1, rear_clearance_m=.1,
    relative_pitch_rad=.2) == 'flight'
```
- [ ] Test delay causality, deterministic noise, immutable samples and physics-rate metrics.
- [ ] Implement pure contact classification, episode counters, analytic quasi-static diagnostic and timestamped sensor buffering.
- [ ] Run the new tests and existing physical telemetry/contact regressions.
- [ ] Commit observation support.

### Task 4: Reproducible control environment and executable experiments
**Files:** Create `sim/research/environment.py`, `sim/research/__init__.py`, `cli/research.py`, `tools/validate_antiwheelie.py`, `tests/test_research_environment.py`; modify the project script entry points.
**Interfaces:** `ResearchEnvironment.reset()` and `.step(RideControl)` hold a command over an integral number of physics steps, with explicit command/sensor delays. Return observations separately from truth and termination. Save commands, observations, metrics and complete physical run metadata.
- [ ] Test integral timing, action hold, reset determinism, no speed/pitch servo, bounded duration and geometry-failure propagation.
- [ ] Implement CLI scenarios for flat acceleration, graded rough road, crest and low grip, with articulated or lumped rider selection.
- [ ] Run real open-loop and torque-limited experiments; retain measured outcomes rather than assumed wheelie claims.
- [ ] Commit the environment and verified experiments.

### Task 5: Regression, packaging and delivery
**Files:** Create `docs/ANTI_WHEELIE.md`, scenario TOML examples and verification reports; update README links and the outer offline installer/wheel hashes.
- [ ] Run the complete pytest suite and targeted numerical/time-step checks in the supplied environment.
- [ ] Build the updated project wheel without isolation or dependency resolution; replace the old wheel and update its SHA256.
- [ ] Install the finished bundle into a clean test environment with network disabled and execute its CLI smoke test.
- [ ] Generate a source patch and check it against a fresh extraction of the original archive.
- [ ] Package the modified project and local wheelhouse without machine-specific virtual environments, caches or Git internals.
- [ ] Document exact commands, coordinate conventions, actuator authority, synthetic-parameter limits and actual test outcomes.

## Numerical validation finding

The first dynamic A/B run exposed an unstable explicit drivetrain limit cycle at 0.5 ms with 34/51 gearing. Root motion alone hid cassette spin oscillations and a >10 kJ mechanical energy residual. Final research defaults must use a verified smaller timestep and report a normalized energy residual; unstable coarse-step examples are not validation evidence. The original legacy timestep remains unchanged for compatibility.
