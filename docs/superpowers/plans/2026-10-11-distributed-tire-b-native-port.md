# Distributed Tyre Plan B: Native C++ Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Prerequisites:** Plan A (`docs/superpowers/plans/2026-10-11-distributed-tire-a-portable-reference.md`) merged; the native engineering rules file referenced by `AGENTS.md` (`docs/agents/native-engineering.md`) located or its absence resolved with the user — do not start native work without it.

**Goal:** A `DistributedTireWriter` in the native extension that is bitwise-equal, per stored golden state, to the portable Python `DistributedTireForceApplier`, selectable through `tire.backend = "distributed_2d_reference"`, allocation-free in its warm core, with multi-contact validity semantics on the wire and a performance contract that proves interactive 1× on `alpine_rocky`.

**Architecture:** Three new native units mirror the three Python modules: `tyre/distance` (SignedProfile), `tyre/station` (scalar station response), `writers/distributed_tire` (the writer). Brush step, friction resolution and `SurfaceMap::at` are lifted out of `writers/tire.cpp` into `writers/tire_common` so both writers share one implementation without touching `TireWriter`'s proved warm core. `Stepper` owns a `std::variant` of the two writers and dispatches the existing tyre API; the config layer gains `tire.distributed`. Tests follow `test_native_tire.py` exactly: golden captured in-test with the distributed profile on `alpine_rocky`, per-state bitwise gate, state round-trip, probe discipline, then trajectory, stands, sanitizers and the performance contract.

**Tech Stack:** C++23 (Apple clang primary, GCC 16 sweep), nanobind, CMake, MuJoCo 3.12.0, Accelerate CBLAS; Python 3.12 + pytest for gates; `uv run --frozen --group native …`.

**Spec:** `docs/superpowers/specs/2026-10-11-native-distributed-tire-design.md` (§5, §6, §7); numeric contract: `docs/adr/0001-native-port-mujoco-core.md` §"Численный контракт P2".

## Global Constraints

- Branch: `feat/native-distributed-tire` (continues Plan A).
- Verification: `bash tools/run_tests.sh native` after every native task (build, all `check_*` sweeps, CTest, native/golden tests); `full` at the end. A missing sweep tool fails the run — install it, never skip.
- Numeric contract: `-ffp-contract=off` (already in `native/CMakeLists.txt`); `a @ b` → `cblas_ddot`, `(3,nv) @ qvel` → `cblas_dgemv`, `J.T @ f` → `cblas_dgemv(CblasTrans)` via `cblas_abi.hpp`; `np.sum` → `numeric::numpy_pairwise_sum`; `np.interp` → `biketyre::np_interp`; `np.linalg.norm(v)` → `std::sqrt(cblas_ddot(v, v))`; `np.cos(np.radians(x))` → `std::cos(x * (M_PI / 180.))` only if numpy's `radians` lowers to the same multiply — verify against `numeric_sincos.hpp` conventions already used in the tree. Any component that cannot be made bitwise gets a per-component ulp bound in the test and a line in the audit.
- Warm core: `try_compute_into` is `noexcept BIKE_NONBLOCKING` and allocation-free; all scratch sized in the constructor from `station_count` (validated `16..512`). RTSan build (`NATIVE_RTSAN=ON`) must pass the new writer's tick.
- `TireWriter` (`writers/tire.cpp`) behaviour, state schema and bitwise results do not change; `test_native_tire.py` stays green at every task.
- No changes to the tyre model (Plan A fixed the reference); no HUD changes; `docs/RIDE.md` capability sentence is the only doc edit.
- Two compilers, clang-tidy/analyzer/cppcheck/ODR/scripts sweeps as configured — `check_<name>` targets.
- Commit after every task with the project trailer:

```
Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
```

---

## File Structure

| File | Responsibility |
|---|---|
| Create `native/src/tyre/distance.hpp/.cpp` | `SignedProfile` port: `height`, single-point `query` |
| Create `native/src/tyre/station.hpp/.cpp` | `station_angles`, `station_density`, `station_response` |
| Create `native/src/writers/tire_common.hpp/.cpp` | `brush_step`, `effective_friction`, `surface_at`, shared snapshot/diagnostic structs |
| Modify `native/src/writers/tire.hpp/.cpp` | Use `tire_common`; no behavioural change |
| Create `native/src/writers/distributed_tire.hpp/.cpp` | `DistributedTireWriter` |
| Modify `native/src/config_types.hpp`, `config_validation.hpp`, `config.hpp` | `DistributedConfig`, backend acceptance, section rules |
| Modify `native/src/stepper.hpp/.cpp` | `std::variant<TireWriter, DistributedTireWriter>` dispatch |
| Modify `native/src/runtime/step.cpp` | `supports_multiple_contacts` per backend; distributed diagnostics on the wire |
| Modify `native/src/binding.cpp` | No new entry points; existing `tire_*` route through the variant |
| Modify `native/CMakeLists.txt` | New sources in the library and the sweeps |
| Modify `src/bike_sim/native/config.py`, `schema.py` | Project `tire.distributed`; accept the backend |
| Create `tests/reference/test_native_distributed_tire.py` | Gates (Tasks 4–8) |
| Modify `tests/reference/test_native_runtime_config.py` | Backend acceptance |
| Modify `tests/reference/test_native_runtime_performance_contract.py` | Distributed contract |
| Modify `docs/RIDE.md:74-78` | Capability sentence |
| Create `docs/superpowers/audits/<date>-native-distributed-tire.md` | Parity table, ulp residuals, RTF evidence |

---

### Task 1: Config projection and native config acceptance

**Files:**
- Modify: `src/bike_sim/native/config.py:32-38, 106-148`, `src/bike_sim/native/schema.py:23-28, 212-215`
- Modify: `native/src/config_types.hpp:242-250`, `native/src/config_validation.hpp:267`, `native/src/config.hpp:394-398`
- Test: `tests/reference/test_native_runtime_config.py`, `tests/reference/test_native_distributed_tire.py`

**Interfaces:**
- Produces (wire): `config['tire']` for the distributed backend =
  `{'backend': 'distributed_2d_reference', 'surface_mode', 'significant_delta_m', 'significance_fraction', 'distinct_normal_deg', 'front': {'tangent_k_n_m', 'mu', 'relaxation_length_m'}, 'rear': {...}, 'surface_map'?: {...}, 'distributed': {'station_count': int, 'density': {'knots_m': [..], 'stiffness_n_m': [..], 'damping_ns_m': float, 'provenance': str}, 'front': {'density': {...}, 'fitting_dataset_id': str, 'valid_load_range_n': [lo, hi], 'valid_deflection_range_m': [lo, hi], 'calibration_status': str}, 'rear': {...}}}`.
- Produces (C++): `nativecfg::HingeDensity {std::vector<double> knots_m, stiffness_n_m; double damping_ns_m; std::string provenance;}`, `nativecfg::DistributedMaterial {HingeDensity density; std::string fitting_dataset_id; std::array<double,2> valid_load_range_n, valid_deflection_range_m; std::string calibration_status;}`, `nativecfg::DistributedConfig {int station_count; HingeDensity density; DistributedMaterial front, rear;}`, `std::optional<DistributedConfig> TireConfig::distributed`.

- [x] **Step 1: Failing tests**

In `tests/reference/test_native_distributed_tire.py` (new):

```python
"""Native distributed tyre writer: config, bitwise per-state parity, state wire, probe, stands."""
import re
from pathlib import Path

import mujoco
import numpy as np
import pytest

from native_loader import load_native
bike_native = load_native()

from _bits import assert_bitwise_equal
from test_native_brake_resistance import _restore

ROOT = Path(__file__).resolve().parents[2]
DISTRIBUTED = ROOT / 'examples/research/viewer_physics_distributed.toml'
ROCKY = 'examples/research/alpine_rocky.toml'


def _distributed_config(tmp_path, station_count=None):
    text = DISTRIBUTED.read_text()
    for key in ('joint_envelope_path', 'joint_strength_path'):
        text = re.sub(rf'{key} = "([^"]+)"', lambda m: f'{key} = "{(DISTRIBUTED.parent / m.group(1)).resolve()}"', text)
    if station_count is not None:
        text = re.sub(r'station_count = \d+', f'station_count = {station_count}', text)
    path = tmp_path / 'physics.toml'
    path.write_text(text)
    return path


def _env(tmp_path, duration='1', station_count=None):
    from bike_sim.cli import research as research_cli
    args = research_cli.parser().parse_args([
        '--physics-config', str(_distributed_config(tmp_path, station_count)),
        '--track-file', ROCKY, '--duration', duration, '--dt', '.00125',
        '--diagnostic-model-limits', '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


def _golden(tmp_path, steps=40):
    from tools.golden_episode import capture_episode, save
    from bike_sim.sim.ride.control import RideControl
    env = _env(tmp_path)
    ep = capture_episode(env, steps, RideControl(human_torque_nm=35.))
    save(ep, tmp_path / 'g', env.sim.model)
    return env, tmp_path / 'g'


def test_config_projection_carries_the_distributed_section(tmp_path):
    from tools.native_config import project, validate_config
    env = _env(tmp_path, duration='.01')
    cfg = project(env)
    tire = cfg['tire']
    assert tire['backend'] == 'distributed_2d_reference'
    assert 'material' not in tire['front'] and set(tire['front']) == {'tangent_k_n_m', 'mu', 'relaxation_length_m'}
    d = tire['distributed']
    assert d['station_count'] == env.sim.physics_config.tires.distributed.station_count
    assert list(d['density']['knots_m']) == [0., .004, .012] and d['front']['calibration_status'] == 'experimental_unvalidated'
    validate_config(cfg)
    bike_native.Stepper(str(_model_path(tmp_path, env)), cfg)   # construction accepts the backend


def _model_path(tmp_path, env):
    path = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(env.sim.model, str(path), None)
    return path


def test_config_rejects_bad_distributed_sections(tmp_path):
    from tools.native_config import project
    import copy
    env = _env(tmp_path, duration='.01')
    good = project(env)
    model = str(_model_path(tmp_path, env))
    for mutate in (lambda c: c['tire'].pop('distributed'),
                   lambda c: c['tire']['distributed'].__setitem__('station_count', 8),
                   lambda c: c['tire']['distributed'].__setitem__('station_count', 513),
                   lambda c: c['tire']['distributed']['density'].__setitem__('knots_m', [0., .012, .004]),
                   lambda c: c['tire']['distributed']['density'].__setitem__('stiffness_n_m', [-1., 0., 0.])):
        bad = copy.deepcopy(good)
        mutate(bad)
        with pytest.raises((ValueError, RuntimeError)):
            bike_native.Stepper(model, bad)
```

(Confirm how other native tests obtain a `.mjb` for a fresh environment — `_golden` saves one via `save(...)`; if a helper exists in `_native_runtime_support.py`, use it instead of `_model_path`.)

In `test_native_runtime_config.py` locate the assertion at line 55 that currently expects the distributed backend to be rejected and invert it to expect acceptance **with** a `distributed` section and rejection **without** one.

- [x] **Step 2: Run to verify failure**

Run: `uv run --frozen --group native python -m pytest tests/reference/test_native_distributed_tire.py -q -k config`
Expected: FAIL — `project()` raises `native tire port requires TireSpec material` / backend rejected.

- [x] **Step 3: Python projection**

In `config.py`: `_tire_side(params, *, distributed)` skips `material` when `distributed`; `_tire_section` adds:

```python
    if cfg_tires.backend == 'distributed_2d_reference':
        dist = cfg_tires.distributed
        def density(h):
            return {'knots_m': list(h.knots_m), 'stiffness_n_m': list(h.stiffness_n_m),
                    'damping_ns_m': h.damping_ns_m, 'provenance': h.provenance}
        def material(side):
            m = dist.for_wheel(side)
            return {'density': density(m.density), 'fitting_dataset_id': m.fitting_dataset_id,
                    'valid_load_range_n': list(m.valid_load_range_n),
                    'valid_deflection_range_m': list(m.valid_deflection_range_m),
                    'calibration_status': m.calibration_status}
        out['distributed'] = {'station_count': dist.station_count, 'density': density(dist.density),
                              'front': material('front'), 'rear': material('rear')}
```

Update the module docstring paragraph (lines 32–38) to describe both backends. In `schema.py` add the field lists for `tire.distributed`, `tire.distributed.density`, `tire.distributed.front/rear` and the rule "`distributed` required iff backend is distributed, forbidden otherwise", mirroring the `surface_map` rule at line 214.

- [x] **Step 4: C++ config**

`config_types.hpp`: add the three structs above and `std::optional<DistributedConfig> distributed;` to `TireConfig`; `TireParams` material becomes `std::optional<TireMaterial>` (present iff compliant). `config.hpp:394-398`: accept both backends; read `distributed` when the backend is distributed (reader helpers already exist for arrays/strings). `config_validation.hpp:267`: `backend ∈ {compliant_2d, distributed_2d_reference}`; when distributed: `distributed` present, `16 ≤ station_count ≤ 512`, knots strictly increasing from 0, stiffness ≥ 0, damping ≥ 0, load/deflection ranges `0 ≤ lo < hi`, `calibration_status == "experimental_unvalidated"`, `material` absent on both sides; when compliant: `distributed` absent, `material` present.

- [x] **Step 5: Run native profile**

Run: `bash tools/run_tests.sh native tests/reference/test_native_distributed_tire.py tests/reference/test_native_runtime_config.py tests/reference/test_native_input_contracts.py`
Expected: config tests pass; `test_native_input_contracts.py` (recursive key contract) still passes — if it enumerates `tire.front.material` as required, update its optional-key set at line 70 to include `material` and `distributed`.

Note: `Stepper` construction with the distributed backend will fail at this point in the writer factory (`step.cpp:962` "requires the compliant_2d tire writer") — mark `test_config_projection_carries_the_distributed_section`'s last line with `pytest.xfail('writer lands in Task 5')` until Task 5 removes it.

- [x] **Step 6: Commit**

```bash
git add src/bike_sim/native/config.py src/bike_sim/native/schema.py native/src/config_types.hpp native/src/config_validation.hpp native/src/config.hpp tests/reference/test_native_distributed_tire.py tests/reference/test_native_runtime_config.py tests/reference/test_native_input_contracts.py
git commit -m "$(cat <<'EOF'
feat(native): tire.distributed config section and backend acceptance

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 2: `tyre/distance` — SignedProfile port

**Files:**
- Create: `native/src/tyre/distance.hpp`, `native/src/tyre/distance.cpp`
- Create: `native/tests/test_distance.cpp` (CTest, next to `test_contracts.cpp`)
- Test (Python oracle): `tests/reference/test_native_distributed_tire.py`

**Interfaces:**
```cpp
namespace biketyre {
struct PointDistance { double distance; std::array<double,2> normal; std::array<double,2> point; int segment; bool nonsmooth; };
class SignedProfile {
public:
    explicit SignedProfile(std::vector<std::array<double,2>> vertices_xz);   // validation as SignedProfile.__init__
    [[nodiscard]] double height(double x) const;                              // np.interp via np_interp; domain check
    [[nodiscard]] PointDistance query(std::array<double,2> p) const;          // profile_distance.py:40-75 for one point
private:
    std::vector<double> x_, z_, edge_x_, edge_z_, length2_, normal_x_, normal_z_;
};
}
```

- [ ] **Step 1: Failing oracle test (Python side)**

Expose `SignedProfile` through a tiny debug binding `bike_native._debug_signed_distance(vertices, points) -> (distance, normal, point, segment, nonsmooth)` in `binding.cpp` (debug-only entry, documented as test surface), then:

```python
def test_signed_distance_matches_python_bitwise():
    from bike_sim.terrain.profile_distance import SignedProfile
    rng = np.random.default_rng(3)
    x = np.linspace(0., 12., 2401)
    z = .05 * np.sin(x) + np.where((x > 5.) & (x < 5.3), .04, 0.) + rng.normal(0., .002, len(x))
    vertices = np.column_stack([x, z])
    pts = np.column_stack([rng.uniform(.1, 11.9, 400), rng.uniform(-.1, .5, 400)])
    want = SignedProfile(vertices).query(pts)
    got = bike_native._debug_signed_distance(vertices, pts)
    assert_bitwise_equal(np.asarray(got[0]), want.distance, 'distance')
    assert_bitwise_equal(np.asarray(got[1]), want.normal, 'normal')
    assert_bitwise_equal(np.asarray(got[2]), want.point, 'point')
    assert np.array_equal(np.asarray(got[3]), want.segment)
    assert np.array_equal(np.asarray(got[4]), want.nonsmooth)
```

- [ ] **Step 2: Run to verify failure**

Run: `bash tools/run_tests.sh native tests/reference/test_native_distributed_tire.py -k signed_distance`
Expected: `AttributeError: module 'bike_native' has no attribute '_debug_signed_distance'`.

- [ ] **Step 3: Implement**

Port `profile_distance.py:40-75` for one point, operation by operation:
- `height`: `np_interp(x, x_, z_)`; domain check raises `std::invalid_argument("query lies outside the compiled road domain")`.
- `local = clip(searchsorted(x_, px, right) - 1, 0, nseg-1)`; `t = clip(dot(p - v[first], edge)/length2, 0, 1)` where `dot` of two 2-vectors is `np.dot` → `cblas_ddot(2, …)`; `upper = norm(p - (v + t*edge))` → `std::sqrt(cblas_ddot(2, d, d))`.
- `left/right` via `searchsorted(side=left/right)`; `ids = arange(left, max(left+1, right))`.
- Per candidate: `fractions`, `projections`, `vectors`, `squared = np.sum(vectors*vectors, axis=1)` → for 2 terms: `vx*vx + vz*vz` (sequential fold from `-0.0`: `((-0.0 + a) + b)` — write it literally as `(-0.0 + vx*vx) + vz*vz` to keep signed zero).
- `best = min(squared)`; ties mask `squared <= best + 2e-15*max(1., best)`; winner = first tie; `length = sqrt(squared[winner])`; sign by `p.z >= height`; `normal = sign*vector/length` if `length > 1e-12` else `normals[sid]`.
- `nonsmooth`: `norm(tie_normals - normals[sid]) > 1e-10` for any tie (norm = `sqrt(ddot)` on the 2-vector difference), plus the vertex rule at lines 71–74.
- `normals_` precomputed as `(-edge_z, edge_x)/sqrt(length2)` — note Python divides the 2-vector by `np.sqrt(length2)[:, None]`: two independent divisions by the same scalar.

Add `native/tests/test_distance.cpp` with: constructor rejections (non-monotone x, NaN), `height` domain error, a flat-road query (`distance == z`, `segment` of the containing edge, `nonsmooth == false`), a vertex-on-corner query (`nonsmooth == true`), tie rule (point equidistant to two collinear subdivisions → `nonsmooth == false`).

Register the source in `native/CMakeLists.txt` (library + sweeps) and the test with CTest.

- [ ] **Step 4: Run**

Run: `bash tools/run_tests.sh native tests/reference/test_native_distributed_tire.py -k signed_distance`
Expected: pass bitwise. If `normal` differs by 1 ulp on some rows, check that the division `vector/length` is two scalar divisions (numpy broadcasts the division, no reciprocal-multiply) and that `-ffp-contract=off` is in effect for the TU.

- [ ] **Step 5: Commit**

```bash
git add native/src/tyre/distance.hpp native/src/tyre/distance.cpp native/tests/test_distance.cpp native/CMakeLists.txt native/src/binding.cpp tests/reference/test_native_distributed_tire.py
git commit -m "$(cat <<'EOF'
feat(native): SignedProfile port with bitwise oracle gate

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 3: `tyre/station` and `writers/tire_common`

**Files:**
- Create: `native/src/tyre/station.hpp/.cpp`, `native/src/writers/tire_common.hpp/.cpp`
- Modify: `native/src/writers/tire.cpp` (call the shared helpers; delete the moved bodies)
- Create: `native/tests/test_station.cpp`
- Test: `tests/reference/test_native_tire.py` (must stay green), `tests/reference/test_native_distributed_tire.py`

**Interfaces:**
```cpp
namespace biketyre {
struct StationAngles { std::vector<double> angles, weights; };       // station_angles(count): 2*pi*i/count, 2*pi/count
[[nodiscard]] StationAngles station_angles(int count);                // 16..512 else std::invalid_argument
[[nodiscard]] std::pair<double,double> station_density(double delta, const nativecfg::HingeDensity&);   // (force/rad, energy/rad)
[[nodiscard]] std::pair<double,double> station_response(double delta, double rate, double weight, const nativecfg::HingeDensity&);  // (load N, energy J)
}
namespace tirecommon {
struct BrushResult { double xi, force, loss; };
[[nodiscard]] BrushResult brush_step(double xi, double u, double v_roll, double fn, double k, double mu, double length, double dt);  // moved verbatim from tire.cpp
[[nodiscard]] std::pair<double, std::string_view> effective_friction(double tire_mu, const nativecfg::SurfaceMap*, double contact_x, double slip);  // moved verbatim
}
```

- [x] **Step 1: Failing C++ test**

`native/tests/test_station.cpp`: `station_angles(16)`/`(512)` ok, `(15)`/`(513)` throw; `station_density(-.01) == (0,0)`; `station_density(.005)` with default density `(300000*(.005-0) + 0 + 0, .5*300000*.005*.005)`; `station_response(.005, -.3, w)` = `w*max(0, 1500 + 2000*(-.3))`; `station_response(0., 1., w) == (0,0)` (gate `delta > 0`).

- [x] **Step 2: Run** — CTest fails to configure the missing target.

- [x] **Step 3: Implement** `station.cpp` exactly as Plan A's Python (`for knot, k: x = max(delta-knot, 0); force += k*x; energy += k*x*x; return (force, .5*energy)`; response: `delta > 0 ? max(0., elastic + damping*rate) : 0.`, times weight). Create `tire_common` by moving `brush_step`/`effective_friction`/`surface_at` out of `tire.cpp` without changing a character of their bodies; `tire.cpp` includes the header. `TirePatch`, `TireSnapshot`, `TireDiagnostics` move to `tire_common.hpp`; `TireDiagnostics` gains the distributed scalar fields with zero defaults: `bool supports_multiple_contacts{}; int station_count{}, active_station_count{}, nonsmooth_station_count{}, max_patch_count{}; bool outside_material_deflection_range{}; std::string calibration_status, capability_status;`.

- [x] **Step 4: Run** `bash tools/run_tests.sh native tests/reference/test_native_tire.py` → green (bitwise, unchanged), CTest green.

- [x] **Step 5: Commit**

```bash
git add native/src/tyre/station.hpp native/src/tyre/station.cpp native/src/writers/tire_common.hpp native/src/writers/tire_common.cpp native/src/writers/tire.hpp native/src/writers/tire.cpp native/tests/test_station.cpp native/CMakeLists.txt
git commit -m "$(cat <<'EOF'
refactor(native): shared tyre helpers and scalar station response

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 4: `DistributedTireWriter` — construction, state wire, stored energy

**Files:**
- Create: `native/src/writers/distributed_tire.hpp/.cpp`
- Test: `tests/reference/test_native_distributed_tire.py` (state round-trip through a debug binding until Task 5 wires `Stepper`)

**Interfaces:**
```cpp
struct StationState { bool active = false; double xi = 0.0; std::optional<std::array<double,3>> tangent; };
class DistributedTireWriter {
public:
    DistributedTireWriter(const mjModel*, const mjData*, nativecfg::TireConfig);  // geoms/bodies/radii, local_[side] = r*(cos a, 0, sin a), scratch sized by station_count, collision gate
    void reset(); void restart_clock() noexcept;
    [[nodiscard]] std::vector<double> qfrc(const mjData*, double dt);
    void compute_into(const mjData*, double dt, std::span<double> out);
    [[nodiscard]] CoreStatus try_compute_into(const mjData*, double dt, std::span<double> out) noexcept BIKE_NONBLOCKING;
    void set_state(std::span<const std::string> names, std::span<const double> row);   // all 2*N*4 columns required; NaN xi => inactive
    [[nodiscard]] const std::vector<std::string>& state_names() const;                 // f'{side}.s{index:03d}.{leaf}'
    [[nodiscard]] std::vector<double> state() const;                                   // NaN for inactive / unset tangent
    [[nodiscard]] std::vector<double> probe_qfrc(const mjData*, double dt); void probe_compute_into(...);
    [[nodiscard]] double stored_energy(const mjData*) const;
    snapshots()/diagnostics()/probe_snapshots()/snapshot_storage()/... — same shapes as TireWriter
    elastic_energy_j()/brush_loss_step_j()/radial_dissipation_power_w()/radii()
private:
    biketyre::SignedProfile profile_; std::optional<nativecfg::SurfaceMap> surface_map_;
    biketyre::StationAngles stations_; std::array<std::vector<std::array<double,3>>,2> local_;
    std::array<std::vector<StationState>,2> states_;              // N per side
    // scratch: points_ (N x 3), heights_ (N), active_ids_ (N), jac_center_/jr_ (3 x nv), jac_station_ (3 x nv),
    //          loads_, energies_, depths_ (N), normals_ (N x 3), tmp_ (nv), qfrc_out_ (nv), patches reserve N
};
```

- [ ] **Step 1: Failing test** (uses a debug factory `bike_native._debug_distributed_writer(model_path, config)` returning an object with `state_names/state/set_state/stored_energy(qpos, qvel)`; removed in Task 5 once `Stepper` dispatches):

```python
def test_distributed_state_wire_round_trip(tmp_path):
    from tools.native_config import project
    env = _env(tmp_path, duration='.01')
    writer = bike_native._debug_distributed_writer(str(_model_path(tmp_path, env)), project(env))
    names = list(writer.state_names)
    N = env.sim.physics_config.tires.distributed.station_count
    assert names == env.sim.physical.tire.state_names() and len(names) == 2 * N * 4
    row = np.full(len(names), np.nan)
    row[names.index('front.s003.xi')] = .002
    row[names.index('front.s003.tangent.0'):names.index('front.s003.tangent.0') + 3] = [1., 0., 0.]
    row[names.index('rear.s007.xi')] = -.001
    writer.set_state(names, row)
    assert_bitwise_equal(np.asarray(writer.state()), row, 'state round trip')
    with pytest.raises(RuntimeError, match='rear.s'):
        writer.set_state(names[:-1], row[:-1])
```

- [ ] **Step 2: Run** → `AttributeError`.

- [ ] **Step 3: Implement** constructor (mirror `TireWriter`'s ctor order: surface-map gate, profile from `compiled_profile_vertices`, geoms `geom_front_contact`/`geom_rear_contact`, bodies, radii = `geom_size[g,0]`, collision gate, `local_` from `station_angles`), `state_names` (built once), `state`/`set_state` (NaN conventions as Plan A §Task 3), `stored_energy` (Plan A's `stored_energy`: active stations' `station_density(-distance).second * weight` summed with `+=` in station order, then detached-state shear terms in `(side, station)` ascending order — Plan A iterates `sorted(self.states.items())`, which is that order), `reset`, `restart_clock`, and the debug binding.

- [ ] **Step 4: Run** `bash tools/run_tests.sh native tests/reference/test_native_distributed_tire.py -k wire` → pass; RTSan not yet required (no tick).

- [ ] **Step 5: Commit** (`feat(native): DistributedTireWriter skeleton with the station state wire`).

---

### Task 5: `DistributedTireWriter::accumulate_validated` and `Stepper` dispatch — the bitwise gate

**Files:**
- Modify: `native/src/writers/distributed_tire.cpp` (tick), `native/src/stepper.hpp/.cpp` (`std::variant<TireWriter, DistributedTireWriter> tire_`; `tire_qfrc/set_tire_state/tire_state/tire_state_names/tire_snapshots/tire_writer()` dispatch with `std::visit`), `native/src/runtime/step.cpp:962` (accept either writer), `native/src/binding.cpp` (remove debug factory)
- Test: `tests/reference/test_native_distributed_tire.py`

**Interfaces:**
- Consumes: Plan A's `compute_qfrc` as the oracle, step by step (spec §4.3).
- Produces: `Stepper.tire_qfrc(dt)` for the distributed backend equal bitwise to the golden `forces[k]['tires']`.

- [x] **Step 1: Failing test**

```python
def _live_applier(env, ref):
    from bike_sim.sim.ride.distributed_tire_forces import DistributedTireForceApplier
    return DistributedTireForceApplier(ref, env.sim.physical.vertices, env.sim.physics_config.tires, env.sim.track.surface_map)


@pytest.mark.slow
def test_distributed_qfrc_bitwise(tmp_path):
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g / 'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g / 'model.mjb'), project(env))
    applier = _live_applier(env, ref)
    tire_i = ep.force_names.index('tires')
    names = list(ep.tire_state_names)
    assert tuple(st.tire_state_names) == tuple(applier.state_names())
    dt = float(ref.opt.timestep)
    for k in range(len(ep.state_qpos)):
        _restore(ref, dref, ep, k)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k], ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        st.set_tire_state(names, ep.tire_state[k])
        got = st.tire_qfrc(dt)
        assert_bitwise_equal(got, ep.forces[k][tire_i], f'tires golden step {k}')
        applier.set_state(names, ep.tire_state[k])
        want = applier.compute_qfrc(ref, dref, dt)
        assert_bitwise_equal(got, want, f'tires live-oracle step {k}')
        assert_bitwise_equal(np.asarray(st.tire_state()), applier.state_dict()['row'], f'tire_state evolve step {k}')
        if k + 1 < len(ep.tire_state):
            assert_bitwise_equal(np.asarray(st.tire_state()), ep.tire_state[k + 1], f'tire_state[k+1] step {k}')
        native_snaps = st.tire_snapshots()
        for side in ('front', 'rear'):
            snap, want_snap = native_snaps[side], applier.snapshots[side]
            assert snap['backend'] == 'distributed_2d_reference'
            assert len(snap['patches']) == len(want_snap.patches)
            for p, w in zip(snap['patches'], want_snap.patches):
                for f in ('normal_load_n', 'tangent_force_n', 'slip_mps'):
                    assert_bitwise_equal(p[f], float(getattr(w, f)), f'{side}.{f} step {k}')
            diag = st.tire_diagnostics()[side] if hasattr(st, 'tire_diagnostics') else None
            if diag is not None:
                for key in ('multi_support', 'active_station_count', 'max_patch_count', 'nonsmooth_station_count'):
                    assert diag[key] == applier.diagnostics[side][key], f'{side}.{key} step {k}'
                for key in ('radial_energy_j', 'shear_energy_j', 'brush_loss_j', 'penetration_m', 'normal_load_n'):
                    assert_bitwise_equal(diag[key], applier.diagnostics[side][key], f'{side}.{key} step {k}')
        res = st.resistance_components(native_snaps)
        for name, want_r in res.items():
            assert_bitwise_equal(want_r, ep.forces[k][ep.force_names.index(name)], f'{name} via snapshots step {k}')
```

(Check whether `Stepper` exposes diagnostics today — `binding.cpp:448` writes `multi_support` into a dict; use the actual accessor name.)

- [x] **Step 2: Run** → fails at `Stepper` construction (`requires the compliant_2d tire writer`).

- [x] **Step 3: Implement the tick** following spec §4.3 one numbered step per code block, with these lowerings:
  1. `points_[s] = center + R @ local_[s]` — use the same `dgemm/dgemv` call that `compiled_profile_vertices` uses for `local @ R.T + xpos` (one `cblas_dgemv(RowMajor, NoTrans, 3, 3, 1., R, 3, local, 1, 0., out, 1)` then `+ center` component-wise, matching numpy's `center + (local @ rotation.T)` evaluation: the matmul first, then the add).
  2. `heights_[s] = profile_.height(points_[s][0])`; active iff `points_[s][2] < heights_[s]`; ids ascending.
  3. `mj_jac(m, d, jac_center_, jr_, center, body)`; `center_velocity = cblas_dgemv(RowMajor, NoTrans, 3, nv, 1., jac_center_, nv, qvel, 1, 0., …)`.
  4. Per active station: `query = profile_.query({x, z})`; `normal = {n.x, 0, n.z}`, `tangent = {normal.z, 0, -normal.x}`; `arm = point - center`; `jac_station_[k][j] = jac_center_[k][j] + (jr_[(k+1)%3][j]*arm[(k+2)%3] - jr_[(k+2)%3][j]*arm[(k+1)%3])` (the product-then-subtract order of `np.cross`); `velocity = dgemv(jac_station_, qvel)`; `rate = -((v0*n0 + v1*n1) + v2*n2)`.
  5. `station_response(-distance, rate, weight, density)`, `station_density(-distance).first`, `radial_power += max(0., (load - weight*elastic_density)*rate)`.
  6. Brush: `transported = xi * cblas_ddot(3, old_tangent, tangent)`; `losses += max(0., .5*k*(xi*xi - transported*transported))`; `slip = ddot(velocity, tangent)`; `effective_friction(cfg.mu, surface_map, point.x, slip)`; `brush_step(xi, slip, ddot(center_velocity, tangent), load, k, mu, relaxation, dt)`; `shear += .5*k*xi_new*xi_new`.
  7. `force = load*normal + tangent_force*tangent` (component-wise: `load*n[k] + tf*t[k]`); `result += jac_station_.T @ force` → `cblas_dgemv(RowMajor, CblasTrans, 3, nv, 1., jac_station_, nv, force, 1, 0., tmp_, 1)` then `result[j] += tmp_[j]` (numpy evaluates the matmul into a temporary and then `+=`).
  8. Detached stations in `(side, station)` ascending order.
  9. `normal_total = numeric::numpy_pairwise_sum(loads_[0:n])`, `radial_energy = numpy_pairwise_sum(energies_)`; `max_depth`; `center_clearance = profile_.query({cx, cz}).distance - radius`; `distinct` with `std::cos(distinct_normal_deg * pi/180)` — confirm the lowering numpy uses for `np.cos(np.radians(x))` against `numeric_sincos.hpp`; `separate` as Plan A.
  10. Stage snapshot (patches in station order, `backend = "distributed_2d_reference"`, `interval_id = round(time/dt)` as `TireWriter` does) and diagnostics; commit after both wheels; energies and `last_time_s_`.

  `Stepper`: replace `std::unique_ptr<TireWriter>` by `std::variant<std::monostate, TireWriter, DistributedTireWriter>` (or two optionals) and dispatch every `tire_*` method; `step.cpp:962` accepts either alternative; `step.cpp` tick calls `try_compute_into` through the visitor. Remove the Task 4 debug factory.

- [x] **Step 4: Run** `bash tools/run_tests.sh native tests/reference/test_native_distributed_tire.py tests/reference/test_native_tire.py`
Expected: both bitwise suites green. Triage order for a mismatch: (1) `tire_state` evolve — brush path; (2) `qfrc` but states equal — Jacobian/`dgemv` ordering or the `+=` temporary; (3) patches equal but `radial_energy_j` differs — the pairwise sum was replaced by a loop; (4) first mismatch at the first step with a hairpin of >1 active stations — `np.cross` component order.

- [x] **Step 5: Commit** (`feat(native): DistributedTireWriter tick and Stepper dispatch, bitwise against the portable reference`).

---

### Task 6: Probe, clock discipline, reset, and the wire semantics

**Files:**
- Modify: `native/src/writers/distributed_tire.cpp` (`probe_*`), `native/src/runtime/step.cpp:1905-1941` (`supports_multiple_contacts` from the writer's diagnostics; new scalar fields on the wire), `native/src/runtime/research_metrics.cpp:76`, `native/src/runtime/control.hpp:206` (both reference "compliant_2d always publishes …" — verify the distributed path publishes `penetration_m` and the grounded flag the same way)
- Test: `tests/reference/test_native_distributed_tire.py`, `tests/reference/test_native_numeric_completion.py` (observation channel parity across backends)

- [x] **Step 1: Failing tests**

```python
@pytest.mark.slow
def test_distributed_probe_and_clock(tmp_path):
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path, steps=5)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g / 'model.mjb'))
    st = bike_native.Stepper(str(g / 'model.mjb'), project(env))
    names = list(ep.tire_state_names); dt = float(ref.opt.timestep)
    st.set_state(ep.state_qpos[2], ep.state_qvel[2], ep.state_act[2], ep.state_warmstart[2], float(ep.state_time[2]))
    st.forward(); st.set_tire_state(names, ep.tire_state[2])
    before = np.asarray(st.tire_state())
    st.tire_probe_qfrc(dt)                       # advance=False analogue; use the Stepper's actual probe entry point name
    assert_bitwise_equal(np.asarray(st.tire_state()), before, 'probe leaves state')
    st.tire_qfrc(dt)
    with pytest.raises(RuntimeError, match='increasing timestamp'):
        st.tire_qfrc(dt)


def test_distributed_observations_declare_multi_contact(tmp_path):
    from bike_sim.native.research import create_native_research
    env = _env(tmp_path, duration='.05')
    native = create_native_research(env)
    try:
        obs = native.observe() if hasattr(native, 'observe') else native.initial_observation
        tires = obs['tires'] if isinstance(obs, dict) else obs.tires
        for side in ('front', 'rear'):
            assert tires[side]['supports_multiple_contacts'] is True
            assert tires[side]['backend'] == 'distributed_2d_reference'
    finally:
        native.close()
```

(Resolve the exact observation accessor from `tests/reference/test_native_research_environment.py` before running.)

- [x] **Step 2: Run** → probe entry missing / `supports_multiple_contacts` is `False`.

- [x] **Step 3: Implement** probe as `TireWriter` does (detached copy, cleared clock, only `probe_snapshots_` published); `step.cpp` reads `supports_multiple_contacts`, `station_count`, `active_station_count`, `nonsmooth_station_count`, `max_patch_count`, `outside_material_deflection_range`, `calibration_status`, `capability_status` from the diagnostics struct and emits them for the distributed backend (compliant wire unchanged: emit the new keys only when the backend is distributed, so compliant channel parity tests are untouched).

- [x] **Step 4: Run** `bash tools/run_tests.sh native` (whole profile) → green, including `test_native_numeric_completion.py` and the research parity suite.

- [x] **Step 5: Commit** (`feat(native): distributed tyre probe, clock and multi-contact wire semantics`).

---

### Task 7: Stands on the native stepper, trajectory equivalence, sanitizers

**Files:**
- Create: `tests/reference/test_native_distributed_stands.py`
- Test: `tests/reference/test_native_distributed_tire.py` (trajectory)

- [x] **Step 1: Trajectory test** — reuse the pattern of the existing `compliant_2d` trajectory/episode-equivalence test (`test_native_research_environment.py` / `environment_pair` fixture in `conftest.py`): `environment_pair(...)` with the distributed physics config and `alpine_rocky`, advance 1 s, assert channel-wise bitwise equality of observations/transitions as the existing suite does for compliant (the fixture's `create()` takes the arguments the welded recipe uses; extend `environment_factory` with a `physics_config` keyword defaulting to the welded profile so the distributed profile can be passed without changing existing callers).

- [x] **Step 2: Stands** — `test_native_distributed_stands.py`: build the `wheel_stand` model (`validation/contact_manifold_rigs.py::wheel_stand`) and a minimal config dict for the stepper (only the `tire` section plus whatever root sections the config reader requires — see `test_native_input_contracts.py::resolved` for the minimal valid root), run `_run_stand`'s loop with `st.tire_qfrc(dt)` as the force source, and assert the same bounds `distributed_flat_rig`/`distributed_step_rig` declare (`force_relative_error ≤ .02`, `energy_residual_ratio ≤ .02/.05`, `max_patch_count ≥ 2` on the step, `native_wheel_contact_count == 0`). If the stepper cannot be constructed on the bare stand model (missing bodies the root sections require), gate this file on a stand-capable constructor documented in the audit and keep the Python-stand evidence from Plan A as the model's acceptance — do not fake a stand.

- [x] **Step 3: Sanitizers** — configure and run the ASan and RTSan builds per `docs/TESTING.md` (`NATIVE_TEST_BUILD_PATH` to `native/build/asan` and `native/build/rtsan`); RTSan must pass `test_distributed_qfrc_bitwise`'s tick path (`try_compute_into` is the marked region).

- [x] **Step 4: Commit** (`test(native): distributed tyre trajectory, stands and sanitizer evidence`).

---

### Task 8: Performance contract and RTF gate

**Files:**
- Modify: `tests/reference/test_native_runtime_performance_contract.py`
- Evidence: `output/native-distributed-rtf/` (not committed), audit

- [x] **Step 1: Extend the contract** with a distributed case mirroring the existing compliant case (same duration, same thresholds structure) on `alpine_rocky` + `viewer_physics_distributed.toml`; the threshold is the spec's gate expressed in the contract's own units — if the contract measures headless RTF, set the floor so that the interactive ≥ 0.95 at 1× is implied with the same margin the compliant case uses (read how the compliant threshold was derived in the test's docstring and copy the derivation).

- [x] **Step 2: Measure**

```bash
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native python tools/measure_realtime.py --backend native --physics-config examples/research/viewer_physics_distributed.toml --track examples/research/alpine_rocky.toml --duration 30 --warmup 1 --runs 3 --out output/native-distributed-rtf/native
uv run --frozen --group native python tools/measure_realtime.py --backend python --physics-config examples/research/viewer_physics_distributed.toml --track examples/research/alpine_rocky.toml --duration 30 --warmup 1 --runs 3 --out output/native-distributed-rtf/python
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_distributed.toml --track alpine_rocky --time-scale 1
```

Record headless medians (both backends) and the HUD's mean achieved RTF after 20 s at 1×. Gate: ≥ 0.95. If it fails at the default `station_count`, measure 96 and 64; a lower default is a **spec change** (update `viewer_physics_distributed.toml`, the spec, and re-run Plan A's Task 6 test) — not a silent edit.

- [x] **Step 3: Commit** (`test(native): performance contract for the distributed tyre`).

---

### Task 9: Docs and audit

**Files:**
- Modify: `docs/RIDE.md:74-78`
- Create: `docs/superpowers/audits/<date>-native-distributed-tire.md`

- [ ] **Step 1: RIDE.md** — replace "track-material analytic `compliant_2d` tires" with "track-material analytic `compliant_2d` tires and the experimental multi-contact `distributed_2d_reference` tyre (unvalidated material; see the native distributed tyre audit)", and remove "distributed-tire" from the "does not add" list.

- [ ] **Step 2: Audit** — sections: commit range; per-component parity table (component → bitwise / ulp bound with reason); state schema; stands (native vs Python metrics); sanitizer runs (build ids, outcomes); RTF table (python/native headless medians, interactive mean at 1×, station count); residuals and follow-ups (calibration remains out of scope).

- [ ] **Step 3: Final gates** — `bash tools/run_tests.sh native` then `bash tools/run_tests.sh full`; both green or the documented platform-baseline failure set only (as recorded in `docs/RIDE.md`).

- [ ] **Step 4: Commit** (`docs: native distributed tyre capability and acceptance audit`).

---

## Self-review against the spec

- §5.1 files → Tasks 1–6, 9. §5.2 warm-core invariants → Task 4 (scratch), Task 5 (`try_compute_into`), Task 7 (RTSan). §5.3 numeric contract → Global Constraints + Task 5 lowerings. §5.4 tests → Tasks 5 (per-state, state round-trip), 6 (probe, wire), 7 (trajectory, stands, sanitizers), 8 (performance). §6 errors → Task 1 (config), Task 4 (`set_state`), Task 6 (validity). §7 verification → Tasks 7–9.
- Interfaces consistent with Plan A: `state_names()` format `side.sNNN.leaf`, `state_dict()['row']`, `set_state(names, row)`, diagnostics key set, `supports_multiple_contacts = True`, `(side, station)` ascending order for detached stations.
- Uncertain facts are flagged where the executor must look them up (`numeric_sincos.hpp` lowering of `np.cos(np.radians())`, observation accessor names, probe entry-point name, stand constructibility) rather than invented.
