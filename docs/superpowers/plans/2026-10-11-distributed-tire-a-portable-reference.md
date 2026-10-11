# Distributed Tyre Plan A: Portable Python Reference Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Prerequisite:** Phase 0 of `docs/superpowers/plans/2026-10-11-multi-contact-tire.md` finished with Outcome C, and `docs/superpowers/plans/2026-10-11-alpine-scree-track.md` landed.

**Goal:** Rewrite the experimental `distributed_2d_reference` tyre into a form a C++ writer can reproduce bit-for-bit — scalar per-station loop with an explicit operation order, a trimmed per-station state with a canonical wire schema, scalar-only diagnostics — and freeze the files the port's golden and performance gates will reference, without changing the model's physics beyond ulp-level reordering (documented before/after).

**Architecture:** `sim/ride/distributed_tire_forces.py` is rewritten around `_StationState(xi, tangent)` and a station loop that mirrors spec §4.3 step by step; scalar `station_density`/`station_response` join the vectorised functions in `physics/distributed_tire.py` (stands keep the vectorised ones). State serialisation in `equilibrium_cache`, `initial_state` and `tools/golden_episode.py` moves to the new `state_dict()`/`set_state()` wire. A pinned `alpine_rocky` preset, its frozen TOML and a distributed physics profile are added. Before any rewrite, the current implementation's stand traces are captured and pinned as the before/after evidence.

**Tech Stack:** Python 3.12, numpy, MuJoCo 3.12.0 (stands only), pytest; `uv run --frozen …`.

**Spec:** `docs/superpowers/specs/2026-10-11-native-distributed-tire-design.md` (§4, §6, §7)

## Global Constraints

- Branch: `feat/native-distributed-tire` from the HEAD that contains the alpine track work.
- Python only; nothing under `native/` changes in Plan A. `bike_sim/native/config.py` still rejects the backend (Plan B changes that).
- Verification profile: `bash tools/run_tests.sh quick` plus the stand runs named in Task 1/Task 7.
- The vectorised `density_response`, `normal_station_response`, `flat_load`, `fit_density` and the stand rigs keep their signatures and results.
- `calibration_status` stays `'experimental_unvalidated'`; `capability_status` string stays `'experimental_reference_not_release_qualified'`.
- Diagnostics keys emitted by the rewritten applier are exactly (spec §2): `penetration_m, normal_load_n, multi_support, supports_multiple_contacts, capability_status, calibration_status, station_count, active_station_count, nonsmooth_station_count, max_patch_count, outside_material_load_range, outside_material_deflection_range, radial_energy_j, shear_energy_j, brush_loss_j`.
- Operation order in `compute_qfrc` follows spec §4.3 literally; any deviation must be recorded in the spec before the code changes.
- Commit after every task with the project trailer:

```
Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
```

---

## File Structure

| File | Responsibility |
|---|---|
| Create `tests/reference/test_distributed_tire_reference.py` | Before/after pin, scalar-vs-vector formulas, state wire, determinism, multi-contact behaviour |
| Modify `src/bike_sim/physics/distributed_tire.py` | `station_density`, `station_response`; `station_count ≤ 512` |
| Modify `src/bike_sim/sim/ride/distributed_tire_forces.py` | `_StationState`, portable `compute_qfrc`, `state_dict`/`set_state`, trimmed diagnostics |
| Modify `src/bike_sim/sim/ride/equilibrium_cache.py:94-120, 180-247` | Distributed state arrays = `sides, xi, tangent, tangent_valid` |
| Modify `src/bike_sim/sim/ride/initial_state.py:83-134` | `project_stations` emits the same four arrays |
| Modify `tools/golden_episode.py:50-54` | `_tire_state_row` via `state_dict()` |
| Modify `src/bike_sim/terrain/alpine.py`, `presets.py` | `alpine_rocky` preset (seed 0, rocky spec) |
| Create `examples/research/alpine_rocky.toml` | Frozen dump |
| Create `examples/research/viewer_physics_distributed.toml` | Welded profile with the distributed backend |
| Create `docs/superpowers/audits/<date>-distributed-reference-rewrite.md` | Before/after stand and sweep diff |

---

### Task 1: Pin the current behaviour (before)

**Files:**
- Create: `tests/reference/test_distributed_tire_reference.py`
- Create: `output/dist-ref/before/` (not committed) with `contact_matrix.json`, `fidelity/`

**Interfaces:**
- Produces: the pinned step-rig trace `BEFORE_STEP_TRACE` used by Task 4 to bound the rewrite's drift.

- [ ] **Step 1: Capture the step-rig trace of the current implementation**

```bash
uv run --frozen python - <<'EOF'
from bike_sim.validation.contact_manifold_rigs import distributed_road_trace
rows, m = distributed_road_trace(.00125, 128, .005, case='step', duration_s=.1)
print('NORMAL =', [round(r['normal_load_n'], 9) for r in rows[::8]])
print('VERTICAL =', [round(r['vertical_force_n'], 9) for r in rows[::8]])
print('PATCHES =', [r['patch_count'] for r in rows[::8]])
print('METRICS =', {k: round(float(v), 12) for k, v in m.items()})
EOF
```

- [ ] **Step 2: Write the test file with those numbers pasted in**

```python
# tests/reference/test_distributed_tire_reference.py
"""The portable distributed tyre reference: pinned behaviour, scalar formulas, state wire."""
import numpy as np
import pytest

from bike_sim.physics.distributed_tire import HingeDensity, density_response, normal_station_response
from bike_sim.validation.contact_manifold_rigs import distributed_road_trace, wheel_stand

# Captured from the vectorised implementation before the portable rewrite (Task 1).
BEFORE_NORMAL = [...]      # paste NORMAL
BEFORE_VERTICAL = [...]    # paste VERTICAL
BEFORE_PATCHES = [...]     # paste PATCHES
BEFORE_MAX_PATCHES = ...   # paste METRICS['max_patch_count']


def test_step_rig_matches_the_pinned_trace_to_reordering_tolerance():
    rows, m = distributed_road_trace(.00125, 128, .005, case='step', duration_s=.1)
    got_n = [r['normal_load_n'] for r in rows[::8]]
    got_v = [r['vertical_force_n'] for r in rows[::8]]
    assert [r['patch_count'] for r in rows[::8]] == BEFORE_PATCHES
    np.testing.assert_allclose(got_n, BEFORE_NORMAL, rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(got_v, BEFORE_VERTICAL, rtol=1e-9, atol=1e-9)
    assert m['max_patch_count'] == BEFORE_MAX_PATCHES >= 2
    assert m['energy_residual_ratio'] <= .05
    assert m['native_wheel_contact_count'] == 0
```

- [ ] **Step 3: Run it — it must pass on the untouched code (tolerance is loose enough for identity)**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py -q`
Expected: 1 passed.

- [ ] **Step 4: Capture the full before-evidence (not committed)**

```bash
mkdir -p output/dist-ref/before
uv run --frozen python - <<'EOF'
import json
from bike_sim.validation.contact_acceptance import contact_matrix
rows = contact_matrix(dts=(.00125, .000625), station_counts=(128, 256), flat_loads=(300., 600., 900.), step_dxs=(.01, .005))
json.dump(rows, open('output/dist-ref/before/contact_matrix.json', 'w'), indent=1, default=float)
print(sum(r['accepted'] for r in rows), 'of', len(rows))
EOF
uv run --frozen python -m bike_sim.validation.fidelity_sweep --backend distributed_2d_reference --stations --output output/dist-ref/before/fidelity --jobs 3
```

Record the "accepted of total" line; keep the directory for Task 7.

- [ ] **Step 5: Commit the test**

```bash
git add tests/reference/test_distributed_tire_reference.py
git commit -m "$(cat <<'EOF'
test: pin the distributed tyre step-rig trace before the portable rewrite

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 2: Scalar station formulas and the station-count ceiling

**Files:**
- Modify: `src/bike_sim/physics/distributed_tire.py:95-116`
- Test: `tests/reference/test_distributed_tire_reference.py`

**Interfaces:**
- Produces: `station_density(delta: float, material: HingeDensity) -> tuple[float, float]` (elastic force per radian, energy per radian), `station_response(delta: float, rate: float, weight: float, material: HingeDensity) -> tuple[float, float]` (normal load N, energy J); `station_angles(count)` rejects `count > 512`.

- [ ] **Step 1: Failing tests (append)**

```python
def test_scalar_station_formulas_equal_the_vectorised_ones_bitwise():
    material = HingeDensity()
    deltas = np.array([-.01, 0., 1e-6, .003, .004, .0041, .011, .012, .02])
    rates = np.array([-.3, 0., .2, -1., .5, 0., .1, -.1, 2.])
    weights = np.full(len(deltas), 2 * np.pi / 128)
    vec_force, vec_energy = normal_station_response(deltas, rates, weights, material)
    vec_density, vec_density_energy = density_response(deltas, material)
    from bike_sim.physics.distributed_tire import station_density, station_response
    for i in range(len(deltas)):
        f, e = station_response(float(deltas[i]), float(rates[i]), float(weights[i]), material)
        d, de = station_density(float(deltas[i]), material)
        assert (f, e) == (float(vec_force[i]), float(vec_energy[i]))
        assert (d, de) == (float(vec_density[i]), float(vec_density_energy[i]))


def test_station_count_ceiling():
    from bike_sim.physics.distributed_tire import station_angles, DistributedTireConfig
    station_angles(512)
    with pytest.raises(ValueError, match='512'):
        station_angles(513)
    with pytest.raises(ValueError):
        DistributedTireConfig(station_count=1024)
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py -q`
Expected: 2 failed (`ImportError` for `station_density`; `station_angles(513)` does not raise).

- [ ] **Step 3: Implement**

In `physics/distributed_tire.py` replace `station_angles` and add the scalar pair after `normal_station_response`:

```python
MAX_STATION_COUNT = 512
"""Upper bound the native writer sizes its allocation-free scratch against."""


def station_angles(count: int):
    if isinstance(count, bool) or not isinstance(count, int) or count < 16:
        raise ValueError('at least sixteen tread stations required')
    if count > MAX_STATION_COUNT:
        raise ValueError(f'at most {MAX_STATION_COUNT} tread stations supported')
    return 2*np.pi*np.arange(count)/count, np.full(count, 2*np.pi/count)


def station_density(delta: float, material: HingeDensity) -> tuple[float, float]:
    """One station's elastic force and energy per radian: the scalar form of density_response.

    The knot sum is sequential, which is what numpy's pairwise sum does below eight terms,
    so the result is bitwise the vectorised value for the same station.
    """
    if not np.isfinite(delta):
        raise ValueError('contact penetration must be finite')
    force = 0.
    energy = 0.
    for knot, k in zip(material.knots_m, material.stiffness_n_m):
        x = max(delta - knot, 0.)
        force += k * x
        energy += k * x * x
    return force, .5 * energy


def station_response(delta: float, rate: float, weight: float, material: HingeDensity) -> tuple[float, float]:
    """One station's normal load and stored energy: the scalar form of normal_station_response."""
    if not (np.isfinite(delta) and np.isfinite(rate) and np.isfinite(weight)) or weight < 0:
        raise ValueError('invalid contact quadrature')
    elastic, energy = station_density(delta, material)
    force = max(0., elastic + material.damping_ns_m * rate) if delta > 0 else 0.
    return weight * force, weight * energy
```

Check the bitwise claim against numpy: `density_response` computes `np.sum(k*x, axis=-1)` over three knots. numpy's reduction for `n < 8` is a sequential fold starting from `-0.0` (see `native/src/numeric_sum.hpp`), i.e. `((-0.0 + a) + b) + c`; `0. + a` and `-0. + a` are identical for every finite `a ≠ 0`, and for `a = 0` both give `+0.0` under the test's `==`. `.5 * np.sum(k*x*x)` vs `.5 * energy` — same order (`k*x*x` is evaluated left to right in both). If the test still fails on a row, print both values with `.hex()` and compare the operation order against `density_response` rather than loosening the assertion.

- [ ] **Step 4: Run**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py -q`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/physics/distributed_tire.py tests/reference/test_distributed_tire_reference.py
git commit -m "$(cat <<'EOF'
feat(physics): scalar station response for the portable distributed tyre

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 3: `_StationState` and the state wire

**Files:**
- Modify: `src/bike_sim/sim/ride/distributed_tire_forces.py` (add the dataclass and the wire methods; `compute_qfrc` still vectorised until Task 4)
- Test: `tests/reference/test_distributed_tire_reference.py`

**Interfaces:**
- Produces: `_StationState(xi: float = 0., tangent: np.ndarray | None = None)`; `DistributedTireForceApplier.state_names() -> list[str]` (`f'{side}.s{index:03d}.{leaf}'`, sides `front, rear`, `index 0..N-1`, leaves `xi, tangent.0, tangent.1, tangent.2`); `state_dict() -> {'names': [...], 'row': np.ndarray}`; `set_state(names, row)`; `states: dict[tuple[str, int], _StationState]`.

- [ ] **Step 1: Failing tests (append)**

```python
def _stand():
    m, d, tire = wheel_stand(.00125, 32)
    return m, d, tire


def test_state_wire_names_and_round_trip():
    from bike_sim.sim.ride.distributed_tire_forces import _StationState
    m, d, tire = _stand()
    names = tire.state_names()
    assert len(names) == 2 * 32 * 4
    assert names[:4] == ['front.s000.xi', 'front.s000.tangent.0', 'front.s000.tangent.1', 'front.s000.tangent.2']
    assert names[-1] == 'rear.s031.tangent.2'
    tire.states = {('front', 3): _StationState(.002, np.array([1., 0., 0.])), ('rear', 7): _StationState(-.001, None)}
    wire = tire.state_dict()
    assert wire['names'] == names and wire['row'].shape == (len(names),)
    assert wire['row'][names.index('front.s003.xi')] == .002
    assert np.isnan(wire['row'][names.index('rear.s007.tangent.0')])
    assert np.isnan(wire['row'][names.index('front.s000.xi')])
    tire.states = {}
    tire.set_state(wire['names'], wire['row'])
    assert set(tire.states) == {('front', 3), ('rear', 7)}
    assert tire.states[('front', 3)].xi == .002 and tire.states[('rear', 7)].tangent is None
    assert np.array_equal(tire.states[('front', 3)].tangent, [1., 0., 0.])
    with pytest.raises(ValueError, match='rear.s031.tangent.2'):
        tire.set_state(wire['names'][:-1], wire['row'][:-1])
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py -q -k wire`
Expected: FAIL with `ImportError: cannot import name '_StationState'`.

- [ ] **Step 3: Implement**

Add to `distributed_tire_forces.py` (module level, after imports):

```python
@dataclass
class _StationState:
    """Per-station brush state: only what the next interval reads (spec section 4.1)."""
    xi: float = 0.0
    tangent: np.ndarray | None = None


_STATION_LEAVES = ('xi', 'tangent.0', 'tangent.1', 'tangent.2')
```

and `from dataclasses import dataclass` to the imports. Add the methods to the class:

```python
    def state_names(self) -> list[str]:
        """Canonical wire columns: side-major, station-major, then xi and the tangent components."""
        return [f'{side}.s{index:03d}.{leaf}' for side in ('front', 'rear')
                for index in range(len(self.angles)) for leaf in _STATION_LEAVES]

    def state_dict(self) -> dict:
        """{'names': [...], 'row': ndarray}: NaN for an inactive station or an unset tangent."""
        row = np.full(2 * len(self.angles) * 4, np.nan)
        for (side, index), state in self.states.items():
            base = (0 if side == 'front' else 1) * len(self.angles) * 4 + index * 4
            row[base] = float(state.xi)
            if state.tangent is not None:
                row[base + 1:base + 4] = np.asarray(state.tangent, float)
        return {'names': self.state_names(), 'row': row}

    def set_state(self, names, row) -> None:
        """Inverse of state_dict; every canonical column is required."""
        index = {str(n): i for i, n in enumerate(names)}
        row = np.asarray(row, float)
        states = {}
        for side in ('front', 'rear'):
            for station in range(len(self.angles)):
                keys = [f'{side}.s{station:03d}.{leaf}' for leaf in _STATION_LEAVES]
                for key in keys:
                    if key not in index:
                        raise ValueError(f'tire state is missing column {key}')
                values = np.array([row[index[k]] for k in keys])
                if np.isnan(values[0]):
                    continue
                tangent = None if np.isnan(values[1:]).all() else values[1:].copy()
                states[(side, station)] = _StationState(float(values[0]), tangent)
        self.states = states
        self.last_time_s = None
```

- [ ] **Step 4: Run**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py -q`
Expected: 4 passed (the vectorised `compute_qfrc` still constructs `_BrushState`; that is replaced in Task 4).

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/sim/ride/distributed_tire_forces.py tests/reference/test_distributed_tire_reference.py
git commit -m "$(cat <<'EOF'
feat(tire): per-station state wire for the distributed tyre

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 4: Portable `compute_qfrc`

**Files:**
- Modify: `src/bike_sim/sim/ride/distributed_tire_forces.py` (replace `_geometry`, `stored_energy`, `compute_qfrc`)
- Test: `tests/reference/test_distributed_tire_reference.py`

**Interfaces:**
- Consumes: `station_density`, `station_response`, `_brush_step`, `effective_friction`, `point_jacobian`, `SignedProfile.query/height`.
- Produces: `compute_qfrc(model, data, dt, *, advance=True) -> np.ndarray` with the diagnostics keys of Global Constraints; `snapshots[side]` is a `WheelContactSnapshot` with one `ContactPatch` per active station in ascending station order.

- [ ] **Step 1: Failing tests (append)**

```python
def test_diagnostics_are_scalar_only_and_multi_contact_is_informational():
    rows, m = distributed_road_trace(.00125, 64, .005, case='two_support', duration_s=.05)
    assert m['max_patch_count'] >= 2
    m_, d_, tire = wheel_stand(.00125, 64)
    tire.compute_qfrc(m_, d_, .00125)
    diag = tire.diagnostics['front']
    assert set(diag) == {'penetration_m', 'normal_load_n', 'multi_support', 'supports_multiple_contacts',
                         'capability_status', 'calibration_status', 'station_count', 'active_station_count',
                         'nonsmooth_station_count', 'max_patch_count', 'outside_material_load_range',
                         'outside_material_deflection_range', 'radial_energy_j', 'shear_energy_j', 'brush_loss_j'}
    assert diag['supports_multiple_contacts'] is True and diag['station_count'] == 64
    assert all(not isinstance(v, (list, tuple, np.ndarray)) for v in diag.values())


def test_compute_is_deterministic_and_states_are_station_states():
    from bike_sim.sim.ride.distributed_tire_forces import _StationState
    import mujoco
    a = wheel_stand(.00125, 64)
    b = wheel_stand(.00125, 64)
    for m, d, tire in (a, b):
        d.qpos[1] = -.006
        mujoco.mj_forward(m, d)
    fa, fb = a[2].compute_qfrc(a[0], a[1], .00125), b[2].compute_qfrc(b[0], b[1], .00125)
    assert np.array_equal(fa, fb)
    assert a[2].states and all(isinstance(s, _StationState) for s in a[2].states.values())
    assert a[2].state_dict()['row'].tolist() == b[2].state_dict()['row'].tolist()


def test_probe_leaves_state_and_clock_untouched():
    import mujoco
    m, d, tire = wheel_stand(.00125, 64)
    d.qpos[1] = -.006
    mujoco.mj_forward(m, d)
    tire.compute_qfrc(m, d, .00125)
    before = tire.state_dict()['row'].copy(); clock = tire.last_time_s
    tire.compute_qfrc(m, d, .00125, advance=False)
    assert np.array_equal(np.nan_to_num(tire.state_dict()['row']), np.nan_to_num(before))
    assert tire.last_time_s == clock and tire.probe_snapshots
    with pytest.raises(ValueError, match='once per increasing timestamp'):
        tire.compute_qfrc(m, d, .00125)
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py -q`
Expected: the diagnostics test fails on the key set (`active_station_ids` etc. present); the state test fails on `isinstance(_StationState)`.

- [ ] **Step 3: Rewrite the applier body**

Replace `_geometry`, `stored_energy` and `compute_qfrc` with (imports: add `from bike_sim.physics.distributed_tire import station_angles, station_density, station_response`, drop `normal_station_response, density_response`; keep `_brush_step`, `effective_friction`, `point_jacobian`, `ContactPatch`, `WheelContactSnapshot`; remove the `_BrushState` import):

```python
    def _stations(self, data, side):
        """Centre, rotation, every station's world point, and the ascending ids of active stations."""
        center = data.geom_xpos[self.geoms[side]].copy()
        rotation = data.xmat[self.bodies[side]].reshape(3, 3)
        points = center + self.local[side] @ rotation.T
        heights = self.profile.height(points[:, 0])
        ids = [int(i) for i in np.flatnonzero(points[:, 2] < heights)]
        return center, points, ids

    def stored_energy(self, model, data):
        energy = 0.
        for side in self.geoms:
            _, points, ids = self._stations(data, side)
            material = self.materials[side].density
            for sid in ids:
                geometry = self.profile.query(points[sid][None, [0, 2]])
                energy += self.weights[sid] * station_density(-float(geometry.distance[0]), material)[1]
        # A detached station keeps its old energy until the next force interval
        # releases it as loss. Endpoint accounting must not discard it twice.
        for (side, sid), state in sorted(self.states.items()):
            energy += .5 * getattr(self.config, side).tangent_k_n_m * self.weights[sid] * state.xi ** 2
        return energy

    def compute_qfrc(self, model, data, dt, *, advance=True):
        if not advance:
            probe = copy.copy(self)
            probe.states = copy.deepcopy(self.states)
            probe.last_time_s = None
            force = probe.compute_qfrc(model, data, dt)
            self.probe_snapshots = probe.snapshots
            return force
        dt = scalar(dt, 'distributed tire interval', positive=True)
        time = float(data.time)
        if self.last_time_s is not None and time <= self.last_time_s:
            raise ValueError('tire state can advance only once per increasing timestamp')
        result = np.zeros(model.nv)
        snapshots = {}; diagnostics = {}; new_states = {}
        energy = total_loss = radial_loss_power = 0.
        cos_distinct = np.cos(np.radians(self.config.distinct_normal_deg))
        for side, body in self.bodies.items():
            cfg = getattr(self.config, side)
            domain = self.materials[side]; material = domain.density
            center, points, ids = self._stations(data, side)
            jp_center, jr = point_jacobian(model, data, body, center)
            center_velocity = jp_center @ data.qvel
            loads = np.zeros(len(ids)); energies = np.zeros(len(ids)); depths = np.zeros(len(ids))
            normals = []; patches = []
            losses = 0.; shear_energy = 0.; radial_power = 0.; nonsmooth = 0
            for j, sid in enumerate(ids):
                geometry = self.profile.query(points[sid][None, [0, 2]])
                n2 = geometry.normal[0]
                normal = np.array([n2[0], 0., n2[1]])
                tangent = np.array([normal[2], 0., -normal[0]])
                arm = points[sid] - center
                # J(p) = J(c) + omega x (p - c), component by component (the unrolled np.cross).
                jac = np.empty_like(jp_center)
                for k in range(3):
                    jac[k] = jp_center[k] + (jr[(k + 1) % 3] * arm[(k + 2) % 3] - jr[(k + 2) % 3] * arm[(k + 1) % 3])
                velocity = jac @ data.qvel
                rate = -((velocity[0] * normal[0] + velocity[1] * normal[1]) + velocity[2] * normal[2])
                penetration = -float(geometry.distance[0])
                load, elastic_energy = station_response(penetration, rate, float(self.weights[sid]), material)
                elastic_density = station_density(penetration, material)[0]
                radial_power += max(0., (load - self.weights[sid] * elastic_density) * rate)
                old = self.states.get((side, sid), _StationState())
                xi = old.xi; k_station = cfg.tangent_k_n_m * self.weights[sid]
                if old.tangent is not None:
                    transported = xi * float(old.tangent @ tangent)
                    losses += max(0., .5 * k_station * (xi * xi - transported * transported))
                    xi = transported
                slip = float(velocity @ tangent)
                mu, _ = effective_friction(cfg, self.surface_map, float(geometry.point[0, 0]), slip)
                xi_new, tangent_force, brush_loss = _brush_step(
                    xi, slip, float(center_velocity @ tangent), load, k_station, mu, cfg.relaxation_length_m, dt)
                losses += brush_loss; shear_energy += .5 * k_station * xi_new ** 2
                force = load * normal + tangent_force * tangent
                result += jac.T @ force
                loads[j] = load; energies[j] = elastic_energy; depths[j] = penetration
                nonsmooth += int(geometry.nonsmooth[0])
                normals.append(normal)
                patches.append(ContactPatch(points[sid], normal, load, tangent_force, slip))
                new_states[(side, sid)] = _StationState(xi_new, tangent)
            for (old_side, sid), old in sorted(self.states.items()):
                if old_side == side and (side, sid) not in new_states:
                    losses += .5 * cfg.tangent_k_n_m * self.weights[sid] * old.xi ** 2
            normal_total = float(np.sum(loads)); radial_energy = float(np.sum(energies))
            max_depth = float(depths.max()) if ids else 0.
            center_clearance = float(self.profile.query(center[None, [0, 2]]).distance[0]) - self.radii[side]
            signed_overlap = max_depth if ids else -max(0., center_clearance)
            distinct = any(float(a @ b) < cos_distinct for a in normals for b in normals) if len(ids) > 1 else False
            # Separate material patches across a missing tread station are also
            # simultaneous supports, even when road normals happen to agree.
            separate = (len(ids) > 1 and any(b - a > 1 for a, b in zip(ids, ids[1:]))
                        and not (ids[0] == 0 and ids[-1] == len(self.angles) - 1))
            snapshots[side] = WheelContactSnapshot(time, tuple(patches), bool(ids), round(time / dt),
                                                   'distributed_2d_reference', center)
            diagnostics[side] = {
                'penetration_m': signed_overlap, 'normal_load_n': normal_total,
                'multi_support': bool(distinct or separate), 'supports_multiple_contacts': True,
                'capability_status': 'experimental_reference_not_release_qualified',
                'calibration_status': domain.calibration_status,
                'station_count': len(self.angles), 'active_station_count': len(ids),
                'nonsmooth_station_count': nonsmooth, 'max_patch_count': len(patches),
                'outside_material_load_range': not (domain.valid_load_range_n[0] <= normal_total <= domain.valid_load_range_n[1]),
                'outside_material_deflection_range': not (domain.valid_deflection_range_m[0] <= max_depth <= domain.valid_deflection_range_m[1]),
                'radial_energy_j': radial_energy, 'shear_energy_j': shear_energy, 'brush_loss_j': losses}
            energy += radial_energy + shear_energy; total_loss += losses; radial_loss_power += radial_power
        self.states, self.snapshots, self.diagnostics = new_states, snapshots, diagnostics
        self.elastic_energy_j, self.brush_loss_step_j = energy, total_loss
        self.radial_dissipation_power_w = radial_loss_power; self.last_time_s = time
        return result
```

Notes that matter for Plan B (do not "simplify" them away):
- `np.sum(loads)` / `np.sum(energies)` stay numpy reductions over the active-station vector: the native side reproduces them with `numeric::numpy_pairwise_sum`.
- `rate` is written as `((v0 n0 + v1 n1) + v2 n2)` — numpy's `np.sum(velocities*normals, axis=1)` for three terms is a sequential fold; `n1 = 0` so the middle term is exactly zero.
- `jac @ data.qvel`, `jp_center @ data.qvel`, `old.tangent @ tangent`, `velocity @ tangent`, `center_velocity @ tangent`, `jac.T @ force` are BLAS calls (`dgemv`/`ddot`) in numpy; the C++ port calls the same CBLAS entry points.
- Detached-station losses iterate `sorted(self.states.items())` — the order the C++ writer uses.
- Update the module docstring: replace "material-station tire/road force path" paragraph with a sentence that the loop order is the native contract and points at the spec.

- [ ] **Step 4: Run the whole file, then the stands**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py -q`
Expected: 7 passed. `test_step_rig_matches_the_pinned_trace_to_reordering_tolerance` is the drift bound: a failure there at `rtol=1e-9` means the rewrite changed more than operation order — diff the per-station `load`/`rate` against the vectorised formulas on the failing row before touching the tolerance.

Run: `uv run --frozen python -m pytest tests/reference/test_native_runtime_config.py -q` (it mentions the distributed backend at line 55 — it must still pass).

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/sim/ride/distributed_tire_forces.py tests/reference/test_distributed_tire_reference.py
git commit -m "$(cat <<'EOF'
refactor(tire): portable per-station distributed tyre loop

Scalar station order the native writer reproduces bit-for-bit; list
diagnostics dropped (no consumers), station state trimmed to xi+tangent.

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 5: State arrays in `equilibrium_cache` and `initial_state`, golden capture

**Files:**
- Modify: `src/bike_sim/sim/ride/equilibrium_cache.py:94-120` (`_state_arrays`), `:180-247` (`restore_state`)
- Modify: `src/bike_sim/sim/ride/initial_state.py:83-134` (`project_stations`)
- Modify: `tools/golden_episode.py:50-54`
- Test: `tests/reference/test_distributed_tire_reference.py`

**Interfaces:**
- Produces: for the distributed backend the state arrays are exactly `state_tire_sides, state_tire_xi, state_tire_tangent, state_tire_tangent_valid`; `restore_state` accepts that set for distributed and the full nine-array set for `compliant_2d` unchanged; `_tire_state_row(tire)` returns `dict(zip(names, row))` from `tire.state_dict()` for either backend.

- [ ] **Step 1: Failing tests (append)**

```python
def test_distributed_state_arrays_round_trip_through_the_cache_codec():
    from types import SimpleNamespace
    from bike_sim.sim.ride.equilibrium_cache import _state_arrays, restore_state
    from bike_sim.sim.ride.distributed_tire_forces import _StationState
    m, d, tire = wheel_stand(.00125, 32)
    tire.states = {('front', 5): _StationState(.001, np.array([0., 0., 1.])), ('rear', 9): _StationState(-.002, None)}
    runtime = SimpleNamespace(drive=SimpleNamespace(ideal_hub=None, clutch=None), rider_contacts=None, tire=tire)
    arrays = _state_arrays(runtime)
    assert set(arrays) == {'state_tire_sides', 'state_tire_xi', 'state_tire_tangent', 'state_tire_tangent_valid'}
    tire.states = {}
    assert restore_state(runtime, arrays) is True
    assert tire.states[('front', 5)].xi == .001 and tire.states[('rear', 9)].tangent is None
    bad = dict(arrays); bad['state_tire_xi'] = np.array([float('nan'), -.002])
    assert restore_state(runtime, bad) is False
```

Also check how `restore_state` is invoked (its exact signature and return convention) in `equilibrium_cache.py` around line 150 and adapt the test call accordingly before running — the test above assumes `restore_state(runtime, state_dict) -> bool`.

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py -q -k cache`
Expected: FAIL (`_state_arrays` reads `.point` on `_StationState` → `AttributeError`, or the key-set assertion fails).

- [ ] **Step 3: Implement**

In `_state_arrays`, replace the `tire` block:

```python
    tire = runtime.tire
    if tire is not None:
        distributed = tire.config.backend == 'distributed_2d_reference'
        sides = tuple(sorted(tire.states)) if distributed else tuple(tire.states)
        state['state_tire_sides'] = np.asarray([json.dumps(key) for key in sides])
        state['state_tire_xi'] = np.asarray([tire.states[side].xi for side in sides], dtype=float)
        state['state_tire_tangent'] = np.asarray([
            value.tangent if value.tangent is not None else np.zeros(3)
            for value in (tire.states[side] for side in sides)
        ], dtype=float).reshape(-1,3)
        state['state_tire_tangent_valid'] = np.asarray([
            value.tangent is not None for value in (tire.states[side] for side in sides)
        ], dtype=bool)
        if not distributed:
            state['state_tire_point'] = ...   # keep the existing three point/segment/center blocks verbatim here
```

(Move the existing `state_tire_point`, `state_tire_point_valid`, `state_tire_segment`, `state_tire_center`, `state_tire_center_valid` assignments unchanged under `if not distributed:`.)

In `restore_state`, inside `if tire is not None:` make `required` depend on the backend:

```python
        distributed = tire.config.backend == 'distributed_2d_reference'
        required = {'state_tire_sides', 'state_tire_xi', 'state_tire_tangent', 'state_tire_tangent_valid'}
        if not distributed:
            required |= {'state_tire_point', 'state_tire_point_valid', 'state_tire_segment',
                         'state_tire_center', 'state_tire_center_valid'}
        if not required.issubset(state):
            return False
```

then keep the `sides` decoding and the key validation as they are (the `distributed` variable now exists earlier — delete its later duplicate definition), and split the array decoding:

```python
        try:
            xi = np.asarray(state['state_tire_xi'],dtype=float)
            tangent = np.asarray(state['state_tire_tangent'],dtype=float)
            tangent_valid = np.asarray(state['state_tire_tangent_valid'],dtype=bool)
        except (ValueError,TypeError):
            return False
        count = len(sides)
        if xi.shape != (count,) or tangent.shape != (count, 3) or tangent_valid.shape != (count,):
            return False
        if not np.isfinite(xi).all() or not np.isfinite(tangent[tangent_valid]).all():
            return False
        if distributed:
            from bike_sim.sim.ride.distributed_tire_forces import _StationState
            updates = {side: _StationState(float(xi[i]), tangent[i].copy() if tangent_valid[i] else None)
                       for i, side in enumerate(sides)}
        else:
            ...  # the existing point/segment/center decoding, shape and finiteness checks, and the
                 # `_BrushState(...)` construction loop verbatim, minus the `distributed` branches
        tire_update=(distributed,updates)
```

In `initial_state.project_stations`: keep the interpolation; the `arrays` dict becomes
`{name: [] for name in ('sides', 'xi', 'tangent', 'tangent_valid')}`, the per-station append writes only those four, and the dtype line drops the `segment` case:

```python
            for j,index in enumerate(ids):
                tangent=target_tangent[index];norm=float(np.linalg.norm(tangent))
                if norm>1e-12:tangent=tangent/norm
                arrays['sides'].append(json.dumps([side,int(index)]));arrays['xi'].append(target_xi[index])
                arrays['tangent'].append(tangent);arrays['tangent_valid'].append(norm>1e-12)
        for name,values in arrays.items():
            dtype=(bool if name.endswith('_valid') else str if name=='sides' else float)
            array=np.asarray(values,dtype=dtype)
            if name=='tangent':array=array.reshape(-1,3)
            p['material']['state_tire_'+name]={'dtype':str(array.dtype),'shape':list(array.shape),'values':array.tolist()}
```

(`geometry = tire.profile.query(...)` in that function is then unused — delete that line too.)

In `tools/golden_episode.py` replace `_tire_state_row`:

```python
def _tire_state_row(tire) -> dict:
    """One flat row of the tyre's wire state — `state_dict()` for either backend."""
    if tire is None:
        return {}
    wire = tire.state_dict()
    return dict(zip(wire['names'], (float(v) for v in wire['row'])))
```

Check the surrounding lines 50–54 for the exact `None` handling the current function has and keep it.

- [ ] **Step 4: Run the file plus the golden and native-config suites that read tyre state**

Run: `uv run --frozen python -m pytest tests/reference/test_distributed_tire_reference.py tests/reference/test_golden_episode.py tests/reference/test_golden_episode_v2.py -q -m "not slow"`
Expected: pass. Then `bash tools/run_tests.sh quick` — green. (`test_native_tire.py` is a `native`-profile test; Plan B re-runs it. It projects artifact rows through `_canonical_row`, so the compliant column names are unchanged by the `state_dict` capture.)

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/sim/ride/equilibrium_cache.py src/bike_sim/sim/ride/initial_state.py tools/golden_episode.py tests/reference/test_distributed_tire_reference.py
git commit -m "$(cat <<'EOF'
refactor(tire): distributed state arrays and golden capture use the station wire

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 6: `alpine_rocky` preset, frozen track, distributed physics profile

**Files:**
- Modify: `src/bike_sim/terrain/alpine.py` (add `ROCKY_PRESET_SEED`, `alpine_rocky`)
- Modify: `src/bike_sim/terrain/presets.py` (`PRESETS`, `SEEDED_PRESETS`)
- Create: `examples/research/alpine_rocky.toml`
- Create: `examples/research/viewer_physics_distributed.toml`
- Test: `tests/reference/test_alpine_track.py`

**Interfaces:**
- Produces: `alpine_rocky(seed=ROCKY_PRESET_SEED) -> TrackSpec` built from `AlpineGenSpec.from_dict(tomllib.loads(Path('examples/research/alpine_rocky_genspec.toml').read_text()))`; `SEEDED_PRESETS['alpine_rocky'] = (alpine_rocky, 0)`; the physics profile `examples/research/viewer_physics_distributed.toml`.

- [ ] **Step 1: Failing tests (append to `tests/reference/test_alpine_track.py`)**

```python
def test_alpine_rocky_preset_is_frozen_and_hazardous_by_design():
    from bike_sim.terrain.alpine import ROCKY_PRESET_SEED, alpine_rocky
    track = get_preset('alpine_rocky')
    assert track.name == 'alpine_rocky' and SEEDED_PRESETS['alpine_rocky'] == (alpine_rocky, ROCKY_PRESET_SEED)
    assert (ROOT / 'examples/research/alpine_rocky.toml').read_text(encoding='utf-8') == dump_track(track)
    assert single_contact_hazards(track)


def test_distributed_physics_profile_loads():
    from bike_sim.physics.resolution import load_physics_config
    cfg = load_physics_config(ROOT / 'examples/research/viewer_physics_distributed.toml')
    assert cfg.tires.backend == 'distributed_2d_reference' and cfg.tires.surface_mode == 'track'
    assert 16 <= cfg.tires.distributed.station_count <= 512
```

(Confirm the loader's name and signature in `src/bike_sim/physics/resolution.py` before running; `test_pinned_topology.py:36` imports `load_physics_config` from there.)

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q -k "rocky or distributed_physics"`
Expected: FAIL (`ImportError: cannot import name 'alpine_rocky'`; missing TOML).

- [ ] **Step 3: Add the preset**

In `terrain/alpine.py`:

```python
ROCKY_PRESET_SEED = 0
ROCKY_GENSPEC_PATH = Path(__file__).resolve().parents[3] / 'examples/research/alpine_rocky_genspec.toml'


def alpine_rocky(seed=ROCKY_PRESET_SEED):
    """Builds the ``alpine_rocky`` preset: the multi-contact gen-spec at the pinned seed.

    Deliberately violates the single-contact guards; it is the multi-contact tyre's track.
    """
    import tomllib
    spec = AlpineGenSpec.from_dict(tomllib.loads(ROCKY_GENSPEC_PATH.read_text(encoding='utf-8')))
    name = 'alpine_rocky' if seed == ROCKY_PRESET_SEED else f'alpine_rocky_{seed}'
    return generate_alpine_track(spec, seed=seed, name=name)
```

with `from pathlib import Path` in the imports and `'ROCKY_PRESET_SEED', 'alpine_rocky'` in `__all__`. In `presets.py` import `ROCKY_PRESET_SEED, alpine_rocky`, add `"alpine_rocky": alpine_rocky` to `PRESETS` and `"alpine_rocky": (alpine_rocky, ROCKY_PRESET_SEED)` to `SEEDED_PRESETS`, and to `__all__`.

- [ ] **Step 4: Freeze the track**

```bash
uv run --frozen python -c "from bike_sim.terrain.presets import get_preset; from bike_sim.terrain.trackfile import save_track; save_track(get_preset('alpine_rocky'), 'examples/research/alpine_rocky.toml')"
```

- [ ] **Step 5: Write the physics profile**

Copy `examples/research/viewer_physics_welded.toml` to `examples/research/viewer_physics_distributed.toml`, change the header comment's first line to `# Distributed-tyre research profile: the welded rider on the experimental multi-contact`, and replace the `[tires]` block with:

```toml
[tires]
backend = "distributed_2d_reference"
surface_mode = "track"

[tires.distributed]
# Station count from the Phase 0 spike (docs/superpowers/audits/*multi-contact-tire-spike.md);
# the native writer sizes its scratch from this value (16..512).
station_count = 128

[tires.front]
mu = 1.1

[tires.rear]
mu = 1.1
```

Set `station_count` to the value the spike recommended if it differs from 128; keep the rest of the profile identical to the welded one.

- [ ] **Step 6: Run**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: pass.

Smoke: `uv run --frozen bike-research --physics-config examples/research/viewer_physics_distributed.toml --track-file examples/research/alpine_rocky.toml --duration 0.5 --diagnostic-model-limits --out output/dist-ref/smoke --overwrite` → exit 0; `summary.json` shows `tires.front.supports_multiple_contacts: true`. Delete `output/dist-ref/smoke`.

- [ ] **Step 7: Commit**

```bash
git add src/bike_sim/terrain/alpine.py src/bike_sim/terrain/presets.py examples/research/alpine_rocky.toml examples/research/viewer_physics_distributed.toml tests/reference/test_alpine_track.py
git commit -m "$(cat <<'EOF'
feat: alpine_rocky preset and the distributed-tyre physics profile

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 7: After-evidence and the rewrite audit

**Files:**
- Create: `docs/superpowers/audits/<date>-distributed-reference-rewrite.md`

- [ ] **Step 1: Capture the after-evidence with the same commands as Task 1 Step 4**, writing to `output/dist-ref/after/`.

- [ ] **Step 2: Diff**

```bash
uv run --frozen python - <<'EOF'
import json
b = json.load(open('output/dist-ref/before/contact_matrix.json')); a = json.load(open('output/dist-ref/after/contact_matrix.json'))
assert len(a) == len(b)
worst = 0.
for rb, ra in zip(b, a):
    assert rb['accepted'] == ra['accepted'], (rb['case'], rb['dt_s'], rb['station_count'])
    for k, vb in rb['metrics'].items():
        va = ra['metrics'][k]
        if isinstance(vb, (int, float)) and vb:
            worst = max(worst, abs(va - vb) / max(abs(vb), 1e-12))
print('max relative metric drift:', worst)
EOF
```

Expected: no assertion; drift ≤ 1e-9. Do the same comparison for the `fidelity/` summary JSONs (compare `metrics` of matching case/dt/dx/station rows).

- [ ] **Step 3: Write the audit**

Sections: Commit range; What changed (state trim, scalar loop, diagnostics trim) with the spec reference; Stand matrix before/after (accepted counts, max drift); Fidelity sweep before/after (per-case max drift table); Step-rig pinned trace (`test_step_rig_matches_the_pinned_trace_to_reordering_tolerance` tolerance and result); Residuals (none expected; list any drift above 1e-9 with its cause).

- [ ] **Step 4: Final quick profile and commit**

Run: `bash tools/run_tests.sh quick` → green.

```bash
git add docs/superpowers/audits/*distributed-reference-rewrite.md
git commit -m "$(cat <<'EOF'
docs: before/after evidence for the portable distributed tyre reference

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

## Self-review against the spec

- §4.1 state → Tasks 3, 5. §4.2 scalar formulas → Task 2. §4.3 loop order → Task 4 (each numbered step maps to a code block; reductions kept as numpy where the spec says so). §4.4 files → all tasks. §2 "Заморозка" → Task 6. §2 "Сдвиг референса" → Tasks 1, 7. §6 `station_count ≤ 512` → Task 2.
- Names consistent: `_StationState`, `state_names/state_dict/set_state`, `station_density/station_response`, `alpine_rocky/ROCKY_PRESET_SEED`, `viewer_physics_distributed.toml`.
- Nothing native changes; `test_native_tire.py` is re-run in Plan B.
