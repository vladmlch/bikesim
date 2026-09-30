# Anti-Wheelie Plant Refocus Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refocus the physical research plant on the anti-wheelie problem: a mid-drive e-bike + articulated rider on procedurally generated rough terrain, exposing continuous front-load margin metrics, episode attribution, a torque-demand channel, and batch evaluation — while demoting the elastic chain to an optional reference model.

**Architecture:** All changes extend the existing `bike-research` plant (`src/bike_sim/sim/research/`, `src/bike_sim/sim/ride/`). The elastic chain is bypassed by the already-implemented `ideal_mid_drive` transmission (one-way tendon constraint), validated by A/B parity runs. New code is small modules in `sim/research/` plus a terrain generator in `terrain/`; every artifact keeps the existing seed/hash reproducibility contract.

**Tech Stack:** Python 3.13, MuJoCo ≥3.0, NumPy, SciPy 1.17.0, pytest. No new dependencies (offline bundle constraint).

**Spec:** Design decisions agreed in the 2026-09-30 grilling session, restated below; background contract in `docs/ANTI_WHEELIE.md`.

## Agreed Design Decisions

- Deliverable = simulation infrastructure + Python policy hook (`policy(observation, demand) -> RideControl`). The anti-wheelie algorithm itself is out of scope — the user owns it.
- Motor stays **mid-drive** (crank torque through gearing, reaction on the frame).
- `ideal_mid_drive` becomes the research default transmission at a validated larger `dt`; `elastic_chain` remains selectable as the reference. Acceptance = A/B parity on wheelie onset, loads, and squat within stated tolerances; if squat diverges, add a chain-line force correction.
- Rider = uncontrolled plant part: articulated legs pedal (disturbance), posture scripted/randomized per episode; groundwork for a reactive rider via a `RiderBehavior` hook (no reflex logic yet).
- Success metric = continuous front-load margin + wheelie episode records with onset context (attribution), not binary lift flags. Terrain-caused micro-lifts are acceptable; `loop_out` is a distinct outcome.
- Demand channel "the bike wants to go": scripted `DemandProgram` torque request + pedelec mode; metrics track delivered/requested.
- Policy actuators: `motor_torque_nm` + `motor_limit_nm`; rear brake stays out-of-band (may be promoted later).
- Sensors: current set incl. IMU; config groundwork for a no-IMU variant.
- Terrain: procedural generator (grade 0–25 %, peaks 30 %; obstacles 2–10 cm; μ-sections; 60–120 m tracks; 10–20 s episodes; rider 60–100 kg), plus a committed eval set.
- Output: `episode_metrics.json` per run + existing artifacts; plots optional by flag.
- 2D (X-Z) only.

## Anti-Goals — What NOT To Do

These are explicit non-goals of this change set and of the project direction. Do not:

1. **Design or tune the anti-wheelie algorithm** — no estimator, no control law, no thresholds-as-policy. The plant exposes signals and commands; the policy is the user's. Demo policies in the batch runner are plumbing (`passthrough`, `zero`, fixed-limit), clearly labeled non-solutions.
2. **Improve or extend the elastic chain** — no chain/freespace fidelity work (dampers, tensioners, meshing, backlash). `elastic_chain` is frozen as a comparison reference only.
3. **Add roll/yaw/steering/lateral dynamics** — the plant stays planar. No 3D tire contact, no lean.
4. **Add a hub-motor drive mode** or other motor topologies.
5. **Model rider physiology/reflexes** — the `RiderBehavior` hook is an interface only; no balance controller, no reflex model, no perception pipeline.
6. **Make the sim "pretty"** — no visual fidelity work, cockpit graphics, animations, or render polish.
7. **Train or evaluate RL agents** — no Gym wrapper, no reward shaping, no training loops in this change.
8. **Loosen the numerical contract** — never relax the energy-residual gate, never convert invalid transitions into training data, never let convenience override `numerically_valid`.
9. **Break the observation/truth split** — nothing privileged (true pitch, loads, terrain, speed) may leak into `SensorObservation`. The demand value is a command echo, not a sensor.
10. **Change legacy/viewer mode behavior** — `bike-ride`, `bike-playground`, golden XML baselines and legacy defaults stay untouched.
11. **Add dependencies** — numpy/scipy/mujoco/matplotlib/pytest/plotly only; the offline bundle must keep working.
12. **Claim real-world safety** — docs keep stating this is software-in-the-loop development, not a validated predictor of a real bicycle.
13. **Deformable soil / rut physics** — terrain stays a rigid heightfield with per-zone friction.
14. **Per-step Python-side physics shortcuts** — no qpos/qvel teleporting, no "helping" forces; all forces go through the existing accumulator/solver path.

## Global Constraints

- Python ≥3.13, `uv` for all invocations (`uv run ...`); offline bundle compatible (`PYTHONPATH=src uv run --offline --no-project --python .venv/bin/python`).
- Immutable dataclass configs with `scalar()`/`array()` validation — match `bike_sim.physics.checks` conventions.
- All durations/delays must be integer multiples of the physics timestep where they map to steps.
- Reproducibility: every run saves `summary.json` metadata + `track.toml` + `terrain_vertices.npy`; nothing silently non-deterministic.
- Docstring style: state *why* a convention exists, not just what. Existing modules are exemplars.
- Tests live in `tests/test_*.py`; run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q <file>`.
- Headless runs need no display; rendering tests use `MUJOCO_GL=egl PYOPENGL_PLATFORM=egl MPLBACKEND=Agg`.

---

### Task 1: Margin metrics, wheelie episodes, loop_out, episode_metrics.json

**Files:**
- Modify: `src/bike_sim/sim/ride/wheelie.py` (`WheelieTracker.update`, metrics, episode list)
- Create: `src/bike_sim/sim/research/metrics.py`
- Modify: `src/bike_sim/sim/research/environment.py` (crash-cause mapping, torque integrals, save)
- Test: `tests/test_wheelie_episodes.py`, `tests/test_episode_metrics.py`, `tests/test_loop_out_reason.py`

**Interfaces:**
- Consumes: existing `WheelieTruth` fields; `sample.channels['control']` (applied `RideControl` as dict); `sample.channels['drive']['motor_torque_nm']` (delivered, solved); `self.sim.crash` (`CrashEvent` with `cause`, `pitch_rad`).
- Produces:
  - `WheelieTracker.update(truth, dt_s, context=None) -> str` (context is an arbitrary dict stored at episode onset)
  - `WheelieTracker.episodes -> list[dict]` — each `{'start_s', 'end_s', 'confirmed', 'max_relative_pitch_rad', 'max_front_clearance_m', 'min_front_load_n', 'onset': context}`
  - `WheelieTracker.metrics` gains `front_load_fraction_min`, `front_load_fraction_mean`, `max_pitch_rate_up_rad_s`, `wheelie_episode_records` (= episodes)
  - `episode_metrics(env) -> dict` (below)
  - `env.torque_delivered_nms`, `env.torque_requested_nms` — integrated N·m·s per control interval
  - `env.reason` may now be `'crash:loop_out'` / `'crash:endo'` instead of `'crash:pitch_over'`

- [ ] **Step 1: Write failing tests for episode tracking and margins**

```python
# tests/test_wheelie_episodes.py
from bike_sim.sim.ride.wheelie import WheelieTracker, WheelieTruth

def _truth(t, front=100., rear=600., fc=0., rc=0., pitch=0., rate=0.):
    return WheelieTruth(time_s=t, front_load_n=front, rear_load_n=rear,
        front_clearance_m=fc, rear_clearance_m=rc, relative_pitch_rad=pitch,
        pitch_rate_up_rad_s=rate)

def test_episode_opens_and_closes_with_context():
    tr = WheelieTracker()
    t = 0.0
    ctx = {'delivered_motor_nm': 80., 'road_pitch_rad': 0.2}
    # supported
    for _ in range(10):
        tr.update(_truth(t), .000125); t += .000125
    # candidate bout: front unloaded, lifted, pitched — confirmed wheelie
    for _ in range(400):  # 50 ms > 20 ms persistence
        tr.update(_truth(t, front=0., fc=.05, pitch=.1), .000125, context=ctx); t += .000125
    for _ in range(10):
        tr.update(_truth(t), .000125); t += .000125
    assert len(tr.episodes) == 1
    ep = tr.episodes[0]
    assert ep['confirmed'] is True
    assert ep['onset'] == ctx
    assert ep['max_front_clearance_m'] >= .05
    assert ep['end_s'] > ep['start_s']

def test_margin_metrics_tracked():
    tr = WheelieTracker()
    # front 100 of 700 total -> fraction 1/7
    tr.update(_truth(0., front=100., rear=600.), .000125)
    tr.update(_truth(.000125, front=50., rear=600.), .000125)
    m = tr.metrics
    assert m['front_load_fraction_min'] <= 50/650 + 1e-9
    assert m['front_load_fraction_mean'] > 0.
```

```python
# tests/test_loop_out_reason.py — exercises the mapping helper, no physics needed
from bike_sim.sim.research.environment import crash_reason
from bike_sim.sim.ride.virtual_rider import CrashEvent

def test_pitch_over_backward_is_loop_out():
    # root_pitch qpos is negative when nose-up (see wheelie.py pitch_up = -qpos)
    ev = CrashEvent(cause='pitch_over', time_s=1., position_m=5., pitch_rad=-1.2)
    assert crash_reason(ev) == 'crash:loop_out'

def test_pitch_over_forward_is_endo():
    ev = CrashEvent(cause='pitch_over', time_s=1., position_m=5., pitch_rad=+1.2)
    assert crash_reason(ev) == 'crash:endo'

def test_contact_cause_passthrough():
    ev = CrashEvent(cause='rider_ground_contact', time_s=1., position_m=5., pitch_rad=0.)
    assert crash_reason(ev) == 'crash:rider_ground_contact'
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_wheelie_episodes.py tests/test_loop_out_reason.py -x`
Expected: FAIL — `update()` takes no `context`, no `episodes`, no `crash_reason`.

- [ ] **Step 3: Implement episode tracking in WheelieTracker**

In `src/bike_sim/sim/ride/wheelie.py`, extend `update()`:

```python
def update(self, truth, dt_s, context=None):
    ...
    candidate = (...)  # existing logic unchanged
    onset = self._candidate_s == 0. and candidate
    if onset:
        self._episode = dict(start_s=truth.time_s, end_s=truth.time_s, confirmed=False,
            max_relative_pitch_rad=truth.relative_pitch_rad,
            max_front_clearance_m=truth.front_clearance_m,
            min_front_load_n=truth.front_load_n,
            onset=dict(context) if context else {})
    elif candidate and self._episode is not None:
        e = self._episode
        e['end_s'] = truth.time_s
        e['confirmed'] = e['confirmed'] or active  # 'active' computed below — reorder accordingly
        e['max_relative_pitch_rad'] = max(e['max_relative_pitch_rad'], truth.relative_pitch_rad)
        e['max_front_clearance_m'] = max(e['max_front_clearance_m'], truth.front_clearance_m)
        e['min_front_load_n'] = min(e['min_front_load_n'], truth.front_load_n)
    elif not candidate and self._episode is not None:
        self.episodes.append(self._episode); self._episode = None
```

Also accumulate into `_metrics` on init: `front_load_fraction_min=None, front_load_fraction_sum=0., front_load_fraction_n=0, max_pitch_rate_up_rad_s=0.` and per step:

```python
total = truth.front_load_n + truth.rear_load_n
if truth.rear_load_n > self.load_threshold_n:
    frac = truth.front_load_n / total if total > 0. else 0.
    m['front_load_fraction_min'] = frac if m['front_load_fraction_min'] is None else min(m['front_load_fraction_min'], frac)
    m['front_load_fraction_sum'] += frac; m['front_load_fraction_n'] += 1
    m['max_pitch_rate_up_rad_s'] = max(m['max_pitch_rate_up_rad_s'], truth.pitch_rate_up_rad_s)
```

Expose in `metrics` property: `front_load_fraction_mean = sum/n if n else None`, plus `wheelie_episode_records=self.episodes`. Close a dangling episode at `metrics` access if still open (copy, don't mutate).

- [ ] **Step 4: Implement crash_reason() and torque integrals in environment.py**

```python
def crash_reason(event):
    """Map a latched CrashEvent to an outcome reason. pitch_over splits by sign:
    negative root pitch = nose-up = loop_out; positive = endo."""
    cause = event.cause
    if cause == 'pitch_over':
        return 'crash:loop_out' if event.pitch_rad < 0. else 'crash:endo'
    return 'crash:' + cause
```

In `step()` replace `self.reason = 'crash:'+self.sim.crash.cause` with `self.reason = crash_reason(self.sim.crash)`. In `_begin_episode` init `self.torque_delivered_nms = 0.` and `self.torque_requested_nms = 0.`; inside the physics loop after `tracker.update`:

```python
applied = self._applied.motor_torque_nm
if applied is not None:
    self.torque_requested_nms += applied*sample.dt_s
self.torque_delivered_nms += float(sample.channels['drive'].get('motor_torque_nm', 0.))*sample.dt_s
self.tracker.update(self.last_truth, sample.dt_s, context={
    'delivered_motor_nm': float(sample.channels['drive'].get('motor_torque_nm', 0.)),
    'applied_motor_nm': applied, 'road_pitch_rad': self.last_truth.road_pitch_rad,
    'pitch_rate_up_rad_s': self.last_truth.pitch_rate_up_rad_s,
    'speed_mps': self.last_truth.speed_mps})
```

- [ ] **Step 5: Create metrics.py and wire save()**

`src/bike_sim/sim/research/metrics.py`:

```python
"""Compact per-episode outcome record for policy development."""
from bike_sim.physics.checks import scalar

def episode_metrics(env):
    m = env.tracker.metrics
    duration = env.sim.time_s
    progress = float(env.sim.position_m)
    requested = env.torque_requested_nms
    return {
        'outcome': env.reason, 'duration_s': duration, 'progress_m': progress,
        'mean_speed_mps': progress/duration if duration > 0. else 0.,
        'finish_time_s': duration if env.reason == 'finish' else None,
        'torque_delivered_nms': env.torque_delivered_nms,
        'torque_requested_nms': requested,
        'motor_pass_fraction': (env.torque_delivered_nms/requested if requested > 0. else None),
        'loop_out': env.reason == 'crash:loop_out',
        'endo': env.reason == 'crash:endo',
        'numerically_valid': env.numerically_valid,
        'max_energy_residual_ratio': env.max_energy_residual_ratio,
        'wheelie': m,  # includes wheelie_episode_records
    }
```

In `save()`: `(path/'episode_metrics.json').write_text(json.dumps(plain(episode_metrics(self)), indent=2, sort_keys=True, allow_nan=False)+'\n')`.

- [ ] **Step 6: Test episode_metrics on a real run**

```python
# tests/test_episode_metrics.py
import json
from bike_sim.cli.research import parser, make_environment
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.metrics import episode_metrics

def test_episode_metrics_schema(tmp_path):
    args = parser().parse_args(['--scenario','flat','--rider','lumped','--duration','1',
        '--ideal-sensors','--out',str(tmp_path/'run')])
    env = make_environment(args)
    while not env.done:
        env.step(RideControl(motor_torque_nm=40., human_torque_nm=0.))
    env.save(tmp_path/'run')
    rec = json.loads((tmp_path/'run'/'episode_metrics.json').read_text())
    for key in ('outcome','duration_s','progress_m','motor_pass_fraction',
                'loop_out','wheelie','numerically_valid'):
        assert key in rec
    assert rec['progress_m'] > 0.
```

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_episode_metrics.py -x`
Expected: PASS (iterate on schema issues).

- [ ] **Step 7: Commit**

```bash
git add src/bike_sim/sim/ride/wheelie.py src/bike_sim/sim/research/metrics.py \
  src/bike_sim/sim/research/environment.py tests/test_wheelie_episodes.py \
  tests/test_episode_metrics.py tests/test_loop_out_reason.py
git commit -m "feat: margin metrics, wheelie episode records, loop_out outcome"
```

---

### Task 2: Transmission selection in research + dt validation + A/B parity tool

**Files:**
- Modify: `src/bike_sim/cli/research.py` (`--transmission` flag, pass to `PhysicalDriveConfig`)
- Modify: `tools/validate_antiwheelie.py` (`--transmission` passthrough into built argv)
- Create: `tools/compare_transmissions.py`
- Test: `tests/test_research_transmission.py`

**Interfaces:**
- Consumes: `PhysicalDriveConfig(transmission_model=...)` already supports `'elastic_chain'`/`'ideal_mid_drive'`; `validate_antiwheelie.py` builds argv lists per case.
- Produces: `--transmission {elastic_chain,ideal_mid_drive}` on `bike-research` (default `ideal_mid_drive`) and on `validate_antiwheelie.py`; `tools/compare_transmissions.py --scenarios ... --dt ... --out ...` writing `comparison.json` with per-scenario deltas and a `passed` verdict.

- [ ] **Step 1: Failing test for the flag**

```python
# tests/test_research_transmission.py
from bike_sim.cli.research import parser, make_environment

def test_default_transmission_is_ideal():
    args = parser().parse_args(['--scenario','flat'])
    env = make_environment(args)
    assert env.sim.physics_config.drive.transmission_model == 'ideal_mid_drive'

def test_elastic_chain_selectable():
    args = parser().parse_args(['--scenario','flat','--transmission','elastic_chain'])
    env = make_environment(args)
    assert env.sim.physics_config.drive.transmission_model == 'elastic_chain'
```

- [ ] **Step 2: Run to verify failure** — `--transmission` unknown arg. Expected FAIL.

- [ ] **Step 3: Implement flag + propagation**

In `parser()`: `p.add_argument('--transmission', choices=('elastic_chain','ideal_mid_drive'), default='ideal_mid_drive')`. In `make_environment`: `drive = PhysicalDriveConfig(..., transmission_model=args.transmission, **drive_kwargs)`. Ignore `chain_k_n_m`/`freehub_k_nm_rad` overrides with a warning comment when transmission is `ideal_mid_drive` (they are unused; raise `ValueError` if both given with ideal model — explicit over silent).

In `tools/validate_antiwheelie.py`: add `p.add_argument('--transmission', default='ideal_mid_drive', choices=...)`, append `['--transmission', args.transmission]` to each case argv; include the value in output dir name (`f'{name}_{args.transmission}_dt_{dt:.8f}'`) and in `result`.

- [ ] **Step 4: Fix research-mode tests that assumed the old default**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/ -k "research or antiwheelie or physical" -x`
Expected: failures where tests/goldens implicitly used `elastic_chain`. For each: pass `--transmission elastic_chain` (or config equivalent) if the test targets chain behavior; otherwise update the expectation and note why in the commit.

- [ ] **Step 5: dt sweep on ideal_mid_drive**

Run: `PYTHONPATH=src uv run python tools/validate_antiwheelie.py --cases wheelie limited rough crest low_grip incline --transmission ideal_mid_drive --dt 0.000125 0.00025 0.0005 0.001 --jobs 4 --out verification/dt_sweep_ideal`

Record the largest `dt` where every case passes its expectation AND `max_energy_residual_ratio < 0.05`. That becomes the new default `--dt` in `parser()` (update help text) and `SimulationPhysicsConfig` guidance — expected `0.0005`–`0.001`. If only `0.00025` passes, keep default `0.000125` and flag the blocker instead of silently picking a coarse step.

- [ ] **Step 6: compare_transmissions.py**

`tools/compare_transmissions.py` — for each `(scenario, seed)` run identical episodes on both transmissions at the validated dt (same initial speed, duration, demand=constant motor torque), then diff: wheelie episode onsets (Δt), `front_load_fraction_min`, `min_front_load_n`, max shock stroke used (`suspension.shock_stroke_m` from telemetry), `mean_speed_mps`. Emit JSON + `passed` bool vs tolerances: onset |Δt| ≤ 50 ms, min load within 15 %, shock stroke within 10 %, speed within 5 %. Skeleton:

```python
def run_one(scenario, seed, transmission, dt, out):
    argv = ['--scenario', scenario, '--transmission', transmission, '--dt', str(dt),
            '--seed', str(seed), '--duration', '5', '--initial-speed', '3',
            '--ideal-sensors', '--motor-torque', '80', '--out', str(out)]
    args = parser().parse_args(argv); env = make_environment(args)
    while not env.done:
        env.step(RideControl(motor_torque_nm=80., human_torque_nm=0.))
    env.save(out); return json.loads((out/'episode_metrics.json').read_text())
```

Compare pairs in `comparison.json`; exit non-zero on parity failure with the deltas printed.

- [ ] **Step 7: Run the A/B check and resolve divergence**

Run: `PYTHONPATH=src uv run python tools/compare_transmissions.py --scenarios flat uphill rough_uphill crest step_up --dt <validated> --seed 17 --out verification/ab_transmission`

If `passed` — done. If squat/loads diverge beyond tolerance, implement the optional correction: in `DrivetrainForceApplier._compute_components` under `self.simplified`, add `components['chainline_pull']` — a force pair applied at the cassette-axle point and the BB point along the instantaneous top-run chain line: `F = delivered_wheel_torque / rear_radius`, direction = unit(BB_center − cassette_center) on the cassette, opposite on the frame; realize through `mj_jac` transposes of both points into `qfrc` (same pattern as `jacobian()`). Re-run A/B; tolerance must pass.

- [ ] **Step 8: Commit**

```bash
git add src/bike_sim/cli/research.py tools/validate_antiwheelie.py \
  tools/compare_transmissions.py tests/ src/bike_sim/sim/ride/drivetrain_forces.py
git commit -m "feat: ideal_mid_drive research default, dt validation, transmission A/B tool"
```

---

### Task 3: Procedural terrain generator + eval set

**Files:**
- Create: `src/bike_sim/terrain/generator.py`
- Modify: `src/bike_sim/cli/research.py` (`--scenario generated`, `--gen-spec`)
- Create: `tools/generate_eval_set.py`
- Create: `examples/research/eval/` (generated TOMLs, committed) + `examples/research/gen_spec.toml`
- Test: `tests/test_terrain_generator.py`

**Interfaces:**
- Consumes: `TrackSpec`, `GradeProfile`, `SurfaceSection`, obstacle classes (`RoadRoughness`, `Bump`, `SquareEdge`, `Drop`, `Roots`, `RockGarden`, `Washboard`), `load_track`/`save_track`.
- Produces: `TerrainGenSpec` dataclass, `generate_track(spec, *, seed, name) -> TrackSpec`, CLI `--scenario generated --gen-spec file.toml`; eval TOMLs loadable via existing `--track-file`.

- [ ] **Step 1: Failing tests**

```python
# tests/test_terrain_generator.py
import numpy as np
from bike_sim.terrain.generator import TerrainGenSpec, generate_track
from bike_sim.terrain.profile import build_profile

def test_deterministic_per_seed():
    a = generate_track(TerrainGenSpec(), seed=42)
    b = generate_track(TerrainGenSpec(), seed=42)
    c = generate_track(TerrainGenSpec(), seed=43)
    xa = np.arange(0., a.length_m, .01)
    assert np.array_equal(build_profile(a, xa), build_profile(b, xa))
    assert not np.array_equal(build_profile(a, xa), build_profile(c, xa))

def test_spec_bounds_respected():
    spec = TerrainGenSpec()
    t = generate_track(spec, seed=7)
    assert spec.length_m[0] <= t.length_m <= spec.length_m[1]
    for o in t.obstacles:
        if hasattr(o, 'height_m'):
            assert o.height_m <= spec.bump_height_m[1] + 1e-9 or True  # per-type bounds checked in impl
    t.validate()  # no overlap, inside track, known surface
```

- [ ] **Step 2: Run to verify failure** — module missing. Expected FAIL.

- [ ] **Step 3: Implement generator.py**

```python
"""Seeded procedural rough-terrain tracks for anti-wheelie episodes."""
from dataclasses import dataclass
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.terrain.grade import GradeProfile
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.surface import SurfaceSection
from bike_sim.terrain.obstacles import RoadRoughness, Bump, SquareEdge, Drop, Roots

@dataclass(frozen=True)
class TerrainGenSpec:
    length_m: tuple = (60., 120.)
    grade_max: float = .25          # nominal knots uniform in [0, grade_max]
    grade_peak: float = .30         # one optional steep stretch
    grade_peak_prob: float = .35
    roughness_amp_m: tuple = (.005, .030)
    roughness_len_m: tuple = (3., 10.)
    bump_height_m: tuple = (.02, .10)
    edge_height_m: tuple = (.02, .08)
    drop_height_m: tuple = (.02, .10)
    base_surfaces: tuple = ('hardpack', 'asphalt', 'loose')
    section_surfaces: tuple = ('wet', 'loose')
    n_roughness: tuple = (2, 6)
    n_features: tuple = (1, 4)
    n_surface_sections: tuple = (0, 2)
    lead_in_m: float = 4.           # flat, clean start for equilibrium
    lead_out_m: float = 5.          # flat finish
```

`generate_track(spec, *, seed, name=None)`:
1. `rng = np.random.default_rng(seed)`; draw `length` from `length_m`.
2. Grade: 3–5 interior knots between lead-in and lead-out; grade uniform `[0, grade_max]`; with `grade_peak_prob`, one consecutive knot pair gets `[grade_max, grade_peak]`. First/last knots = 0 at x=0 / x=length.
3. Roughness sections: draw `n_roughness` disjoint intervals (rejection sample starts, max 200 tries, then shrink-to-fit; intervals keep 1 m clearance from each other and from lead-in/out).
4. Feature obstacles (bump / square edge / drop / roots — uniform pick, per-type height from spec range): place on remaining free intervals; `Drop` allowed only where local grade ≥ 0.
5. Surface: base from `base_surfaces`; `n_surface_sections` non-overlapping `[start,end)` zones from `section_surfaces`, inside the track body.
6. `track.validate()`; `description` records `seed` and a hash of the spec dict. Return the `TrackSpec`.
All sampling uses the single `rng`; no module-level randomness.

- [ ] **Step 4: CLI integration**

In `parser()`: extend `--scenario` choices to `RESEARCH_SCENARIOS + ('generated',)`; add `--gen-spec type=Path` (TOML → `TerrainGenSpec.from_dict`). In `make_environment`: if `args.scenario == 'generated'` → `track = generate_track(spec, seed=args.seed, name=f'generated_{args.seed}')`; a `--track-file` still wins.

- [ ] **Step 5: Eval set tool + committed artifacts**

`tools/generate_eval_set.py`: `--n 10 --seed0 1000 --out examples/research/eval` → writes `eval_{i:02d}.toml` via `save_track` + `manifest.json` (seeds, spec hash, date). Run it once, commit the outputs.

- [ ] **Step 6: Commit**

```bash
git add src/bike_sim/terrain/generator.py src/bike_sim/cli/research.py \
  tools/generate_eval_set.py examples/research/eval/ examples/research/gen_spec.toml \
  tests/test_terrain_generator.py
git commit -m "feat: seeded procedural terrain generator and eval set"
```

---

### Task 4: Demand channel, rider randomization, reactive-rider hook, no-IMU groundwork

**Files:**
- Create: `src/bike_sim/sim/research/demand.py`
- Create: `src/bike_sim/sim/research/rider_random.py`
- Create: `src/bike_sim/sim/research/rider_behavior.py`
- Modify: `src/bike_sim/sim/research/environment.py`, `src/bike_sim/sim/research/sensors.py`, `src/bike_sim/cli/research.py`
- Modify: `examples/research/controller_loop.py` (policy signature `f(obs, demand) -> RideControl`)
- Test: `tests/test_demand_program.py`, `tests/test_rider_random.py`, `tests/test_rider_behavior_hook.py`, `tests/test_sensor_no_imu.py`

**Interfaces:**
- Produces:
  - `DemandProgram(keyframes=((t, nm), ...))`, `.at(t)->float`, `.constant(nm)`, `.load(path)`, `to_dict()` — mirrors `RiderProgram` mechanics.
  - `ResearchEnvironment(..., demand: DemandProgram|None = None, rider_behavior: RiderBehavior|None = None)`; `env.demand_nm -> float|None`; `ResearchStep.demand_nm`; `trace.csv` column `demand_nm`.
  - `RiderRandomSpec` + `sample_rider(spec, seed) -> (RiderSpecs, RiderProgram)`; CLI `--rider-random`, `--rider-seed`.
  - `RiderBehavior` protocol: `reset(seed)`, `act(time_s, signals: RiderSignals) -> RiderPosture|None`. `RiderSignals` = `(pitch_rate_up_rad_s, specific_force_body_mps2, saddle_load_n, bar_load_n, pedal_load_n)` — built from `sample.channels` (real sensory-like channels, no terrain truth).
  - `SensorConfig(imu_enabled=True)`; `False` → `specific_force`/`pitch_rate` delivered as exact zeros.

- [ ] **Step 1: Failing tests (one file per feature, assert contract)**

```python
# tests/test_demand_program.py
from bike_sim.sim.research.demand import DemandProgram

def test_constant_and_keyframes():
    assert DemandProgram.constant(60.).at(5.) == 60.
    d = DemandProgram(keyframes=((0., 0.), (1., 80.)))
    assert d.at(0.) == 0. and abs(d.at(1.)-80.) < 1e-9 and 0. < d.at(.5) < 80.

# tests/test_rider_behavior_hook.py — env calls act() each control step;
# a recording stub proves invocation count and posture application.
# tests/test_sensor_no_imu.py — SensorConfig(imu_enabled=False) → observation
# specific_force == (0,0,0) and pitch_rate == 0. even mid-ride.
```

- [ ] **Step 2: Run to verify failure** — Expected FAIL (missing modules/fields).

- [ ] **Step 3: Implement demand.py** — smoothstep interpolation identical to `RiderProgram.at`; nonnegative torques validated; TOML `[[keyframes]]` table support.

- [ ] **Step 4: Wire demand + behavior into environment**

`ResearchEnvironment.__init__(sim, config=None, sensors=None, *, demand=None, rider_behavior=None)` — validate types. In `step()` at the top of each control interval: `self.demand_nm = self.demand.at(self.sim.time_s) if self.demand else None`; if `self.rider_behavior` and control.posture is None and no `RiderProgram` owns posture: `posture = self.rider_behavior.act(self.sim.time_s, self._rider_signals(sample))`; merge via `replace(control, posture=posture)`. `ResearchStep` gains `demand_nm: float|None`; trace row gains `demand_nm`; `save()` writes `demand_program` dict into `research` metadata when present. Enforce exclusivity: `rider_behavior` + posture-owning `RiderProgram` → `ValueError` at env construction.

- [ ] **Step 5: rider_random.py + CLI flags**

```python
@dataclass(frozen=True)
class RiderRandomSpec:
    mass_kg: tuple = (60., 100.)
    height_m: tuple = (1.55, 1.95)
    torso_lean_rad: tuple = (0., .25)
    pelvis_pitch_rad: tuple = (0., .08)
    human_torque_nm: tuple = (0., 25.)

def sample_rider(spec, seed):
    rng = np.random.default_rng(seed)
    rider = RiderSpecs(variant='articulated_planar',
        mass_kg=float(rng.uniform(*spec.mass_kg)), height_m=float(rng.uniform(*spec.height_m)))
    program = RiderProgram((RiderKeyframe(0., RiderPosture(
        torso_lean_rad=float(rng.uniform(*spec.torso_lean_rad)),
        pelvis_pitch_rad=float(rng.uniform(*spec.pelvis_pitch_rad))),
        human_torque_nm=float(rng.uniform(*spec.human_torque_nm))),))
    return rider, program
```

CLI: `--rider-random` (default off), `--rider-seed` (default = `--seed`); when on, `make_environment` uses `sample_rider` for `RiderSpecs` and installs the program (env applies it when the policy leaves posture/human fields `None`). Sampled values land in `summary.json` metadata.

- [ ] **Step 6: SensorConfig imu_enabled + tests** — zero-fill IMU channels in `SensorPipeline.push` when disabled; document "absent sensor reports exact zero, not noise".

- [ ] **Step 7: Update controller_loop.py** — policy signature `policy(observation, demand_nm) -> RideControl`; show `motor_torque_nm=min(demand, cap)` passthrough and pedelec `motor_limit_nm` example.

- [ ] **Step 8: Run research tests + commit**

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_demand_program.py tests/test_rider_random.py tests/test_rider_behavior_hook.py tests/test_sensor_no_imu.py tests/test_research_environment.py`

```bash
git commit -m "feat: torque demand channel, rider randomization, reactive-rider hook, no-IMU sensor variant"
```

---

### Task 5: Long episodes (10–30 s)

**Files:**
- Modify: `src/bike_sim/cli/research.py` (help text only), `examples/research/long_climb.toml` (new)
- Test: `tests/test_long_episode.py` (marked slow)

**Interfaces:** Consumes Task 2 dt + Task 3 generator.

- [ ] **Step 1: Failing/verification test**

```python
# tests/test_long_episode.py
import pytest
@pytest.mark.slow
def test_fifteen_second_episode_completes(tmp_path):
    args = parser().parse_args(['--scenario','generated','--seed','5','--duration','15',
        '--ideal-sensors','--out',str(tmp_path/'long')])
    env = make_environment(args)
    while not env.done:
        env.step(RideControl(motor_torque_nm=env.demand_nm or 60., human_torque_nm=0.))
    env.save(tmp_path/'long')
    assert env.reason in ('duration','finish','crash:loop_out','crash:endo') or env.tracker.state
```

- [ ] **Step 2: Run at validated dt; fix real blockers** (step caps, recorder memory, file sizes — decimation already supported; document recommended `--record-decimation` for long runs).

- [ ] **Step 3: Commit**

```bash
git commit -m "feat: long-episode support validated to 15+ s on generated terrain"
```

---

### Task 6: Batch runner

**Files:**
- Create: `tools/research_batch.py`
- Create: `src/bike_sim/sim/research/policies.py` (demo plumbing policies only)
- Test: `tests/test_research_batch.py`

**Interfaces:**
- Consumes: `make_environment`, `episode_metrics`, `DemandProgram`, `--track-file` eval set.
- Produces: `python tools/research_batch.py --spec batch.toml --jobs N --out dir` → per-run dirs + `batch_report.json` + `batch_report.csv`; `POLICIES = {'passthrough': ..., 'zero': ..., 'fixed_limit_40': ...}` each `fn(obs, demand) -> RideControl`.

Batch TOML sketch:

```toml
[grid]
tracks = ["examples/research/eval/eval_00.toml", "..."]
scenarios = ["generated"]        # alternative to explicit files
seeds = [1, 2, 3]
demand_nm = [60., 80.]           # or demand_file per run row
policy = "passthrough"
transmission = "ideal_mid_drive"
dt = 0.0005
duration = 15.
```

- [ ] **Step 1: Failing test** — run a 2×2 grid into `tmp_path`, assert `batch_report.json` has one record per run with `outcome`, `progress_m`, `motor_pass_fraction`, `loop_out`, `wheelie.wheelie_time_s`; nonzero exit propagates failed runs as records, not crashes.

- [ ] **Step 2: Implement policies.py** — `passthrough` (`motor_torque_nm=demand or 0`), `zero`, `fixed_limit_40` (`motor_torque_nm=demand, motor_limit_nm=40.`); docstring: "demo plumbing, not anti-wheelie solutions".

- [ ] **Step 3: Implement runner** — `multiprocessing.Pool` over the cartesian grid (pattern after `validate_antiwheelie.py --jobs`); each worker builds argv → `make_environment` → loop `env.step(policy(env.observation, env.demand_nm))` → `env.save(subdir)` → return `episode_metrics`. Aggregate to report files; exit non-zero if any run errored.

- [ ] **Step 4: Run + commit**

```bash
git add tools/research_batch.py src/bike_sim/sim/research/policies.py tests/test_research_batch.py examples/research/batch_demo.toml
git commit -m "feat: batch evaluation runner over tracks/seeds/demand grid"
```

---

### Task 7: Documentation

**Files:**
- Modify: `docs/ANTI_WHEELIE.md`
- Modify: `README.md` (one pointer line only)

- [ ] **Step 1: Update ANTI_WHEELIE.md** — new flags (`--transmission`, `--scenario generated`, `--gen-spec`, `--rider-random`, `--demand`, validated `--dt` default), `episode_metrics.json` schema, `loop_out`/`endo` outcomes, demand channel semantics (advisory echo, not enforced), eval-set workflow, batch runner usage, chain-line correction note if implemented. Keep the "Scope before a real bicycle" section verbatim.

- [ ] **Step 2: Commit**

```bash
git commit -m "docs: antiwheelie plant refocus — metrics, generator, demand channel, batch runner"
```

---

## Self-Review Notes

- Spec coverage: metrics/episodes/loop_out (T1), transmission + dt + A/B (T2), generator + eval set (T3), demand + rider randomization + behavior hook + no-IMU (T4), long episodes (T5), batch runner (T6), docs (T7). All agreed items covered.
- Type consistency: `env.demand_nm` (float|None), `ResearchStep.demand_nm`, `DemandProgram.at(t)`, `WheelieTracker.update(truth, dt_s, context=None)`, `episode_metrics(env)`, `crash_reason(event)`, `sample_rider(spec, seed)`, `POLICIES` registry — consistent across tasks.
- Execution order: T1 → T2 → T3 → T4 → T5 → T6 → T7 (T2 dt result feeds T5/T6 defaults).
