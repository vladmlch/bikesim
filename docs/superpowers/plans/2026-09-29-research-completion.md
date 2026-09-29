# Longitudinal Research Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver an offline, physically auditable and repeatable longitudinal anti-wheelie development plant.

**Architecture:** Extend the existing single-owner physical runtime rather than adding surrogate forces. Keep tire laws, rider programs, sensor timing, applicability checks and replay in focused modules. Retain independent numerical-energy and model-applicability results.

**Tech Stack:** Python 3.13, MuJoCo 3.12.0, NumPy, SciPy, pytest; existing local wheels.

**Spec:** `docs/superpowers/specs/2026-09-29-research-completion.md`

## Global Constraints
- Source baseline: uploaded offline project, read incrementally.
- No dependency downloads; execute Python using uv with --offline --no-project.
- No hidden pitch/speed stabilizer, body teleportation or duplicated tire forces.
- Preserve old public interfaces where possible; add validated optional configuration.
- Explicitly distinguish synthetic material data, model applicability and numerical stability.

---

### Task 1: Nonlinear tire material
**Files:** create `src/bike_sim/physics/tire_curve.py`, `tests/test_tire_curve.py`; modify `physics/tire.py`, `physics/physical_config.py`, `physics/resolution.py`, `sim/ride/tire_forces.py`.
**Interfaces:** `TabulatedTireSpec.elastic_response(delta) -> (force, energy)` and `normal_contact(delta, rate)`; same elastic-response interface on linear `TireSpec`. TOML dispatch uses `deflection_m` to select the table class, rejects unknown/mixed fields.
- [ ] Write a derivative-of-energy regression:
```python
curve = TabulatedTireSpec((0., .01, .03), (0., 1000., 5000.), 100., 200000., 'synthetic', (0., 5000.))
x, eps = .02, 1e-7
force, energy = curve.elastic_response(x)
assert abs((curve.elastic_response(x+eps)[1]-curve.elastic_response(x-eps)[1])/(2*eps)-force) < 1e-4
```
- [ ] Run `python -m pytest -q tests/test_tire_curve.py`; verify missing-interface failure.
- [ ] Implement monotone piecewise-linear force, exact trapezoidal primitive, unilateral damping, immutable parameters, range rejection, and update both runtime energy calculations.
- [ ] Run new tests plus `test_tire_radial.py`, `test_tire_brush.py`, `test_physics_resolution.py`.
- [ ] Commit `feat: add passive nonlinear tire curves`.

### Task 2: Independent sensor acquisition
**Files:** modify `sim/research/sensors.py`, `sim/research/environment.py`; create `tests/test_sensor_acquisition.py`.
**Interfaces:** `SensorConfig.sample_period_s` (zero = every physics step), fixed bias parameters, dropout probability and optional maximum age; existing immutable `SensorObservation` is unchanged.
- [ ] Test that the same fixed physical trace pushed at 1 ms and read at 10/20 ms yields identical shared observations; test unavailable/stale values remain invalid.
```python
assert pipeline.read(.1) == pipeline.read(.1)
assert not pipeline.read(1.).valid
```
- [ ] Run new tests and confirm the missing configuration fails.
- [ ] Acquire only incoming-state physical samples on the acquisition grid; preserve source timestamps, pure reads, deterministic noise and invalidity.
- [ ] Run `test_research_sensors.py`, `test_research_environment.py` and new tests.
- [ ] Commit `fix: decouple sensor acquisition from policy timing`.

### Task 3: Rider programs and plant applicability
**Files:** create `sim/research/rider_program.py`, `sim/research/validity.py`, corresponding tests; modify `sim/research/environment.py`.
**Interfaces:** `RiderProgram.from_dict`, `.at(time_s)`, `.apply(control, time_s)`; `model_violations(sample, maximum_compression_fraction) -> tuple[str, ...]`.
- [ ] Test strict time ordering, bounds, continuity, command ownership and finite parameters; test explicit multi-support/load/compression violations without modifying forces.
```python
assert program.at(0.).posture == program.at(-0.1).posture
assert 'front:multi_support' in model_violations(sample, .15)
```
- [ ] Run new tests to verify the interfaces are absent.
- [ ] Implement immutable C2 keyframes evaluated every physical step after their own reaction delay; retain internal joint effort and actual support contacts.
- [ ] Add latched model validity, recorded reasons and strict termination, separate from numerical-energy validity.
- [ ] Run targeted research, rider and contact regressions.
- [ ] Commit `feat: add rider programs and model applicability gates`.

### Task 4: Configurable scenarios and checked replay
**Files:** create `sim/research/replay.py`, `cli/replay.py`, tests and TOML examples; modify `cli/research.py`, `sim/research/environment.py`, `pyproject.toml`.
**Interfaces:** `rebuild_environment(directory)`, `replay_episode(directory) -> report`; CLI `bike-replay RUN`; research `--physics-config`, `--rider-program`, `--road-resolution`.
- [ ] Save a short run, replay it, compare initial/final states, commands, sensor observations and truth; corrupt a command and require a checksum error.
```python
report = replay_episode(run_path)
assert report['passed']
```
- [ ] Add strict config precedence, mesh-size validation and self-contained scenario manifests; never execute code or deserialize pickle from a recording.
- [ ] Include initial/final integration states and all program/config hashes; reject incompatible versions/source or unsupported overrides rather than silently approximating replay.
- [ ] Run CLI, replay, config, terrain and research tests.
- [ ] Commit `feat: add reproducible scenario configuration and replay`.

### Task 5: Acceptance, offline installation and delivery
**Files:** update offline build/install tools, README and research documentation; add `verification/research_completion/` evidence.
- [ ] Run the complete regression suite and physical A/B, rough-road, moving-rider, crest, material-transition and coarse-step rejection cases; retain actual failures as failures.
- [ ] Test fine-timestep and terrain-resolution comparisons, nonlinear tires, independent sensing and saved-run replay.
- [ ] Build the wheel locally with `setuptools.build_meta.build_wheel`, replace the stale project wheel, regenerate hashes and test an installation without index access.
- [ ] Create `git diff --binary BASELINE`, apply-check against a clean baseline and compare resulting source hashes.
- [ ] Archive updated source, tests, evidence and original dependency wheels, excluding virtual environments and caches; provide exact execution and patch commands.
- [ ] Commit verified release documentation.
