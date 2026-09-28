# Task B1 brief — unified component mass budget and compiled CoM

Source: `docs/superpowers/plans/2026-09-28-02-mass-inertia.md`, Task B1. The physics-correctness specification is authoritative when a conflict appears.

## Global constraints

- Базовая ревизия: 70623815b98788018bcdbd8eef347d778f9bb3f3.
- Python >=3.12; использовать существующий uv.lock, не обновлять зависимости в PR физики.
- Новые численные зависимости не добавлять: NumPy, SciPy, MuJoCo и pytest уже объявлены проектом.
- Новая физика использует SI: m, s, kg, N, N*m, rad; mm и km/h допустимы только на совместимых внешних границах.
- Новая физика остаётся плоской X-Z; боковое сцепление и баланс по крену не заявляются.
- В режиме physical запрещены внешняя стабилизация тангажа, ручной перенос веса и присваивание qpos/qvel после шага для исправления физики.
- legacy и physical имеют разные physics_revision и разные численные эталоны.
- Параметры без измерений помечаются synthetic; прохождение синтетических тестов не считается экспериментальной валидацией.
- Все новые численные допуски являются критериями приёмки, а не результатами уже выполненных испытаний.

## Task B1: unified mass budget and dynamic CoM

**Files:** Create `src/bike_sim/physics/component_masses.py`, `src/bike_sim/sim/ride/mass_properties.py`; modify `src/bike_sim/physics/mass.py:62-89`, builders `mujoco/frame.py`, `mujoco/steering_fork.py:114-139`, `mujoco/rear_linkage.py`, `mujoco/drivetrain.py:30-103`, `mujoco/builder.py`, `sim/ride_sim.py`; test `tests/test_compiled_mass_contract.py`.

**Interfaces:** Produce `assign_component_mass(geoms,total_kg)->None`, `compiled_center_of_mass(model,data)->np.ndarray`. Add keyword-only `mass_specs=None` to `RideSimulation` and pass it through without loss into the MJCF builder. Existing builders return/accumulate a Python registry `component_id -> list[ET.Element]`; do not add invalid custom attributes to XML.

### Required behavior and acceptance

1. Add the mass configuration and total-mass tests. The requested independent examples are:

```python
import mujoco
import pytest
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.mujoco.builder import generate_mujoco_xml

@pytest.mark.parametrize("field", ["front_wheel_mass", "rear_wheel_mass", "frame_structure_mass",
                                   "chainstay_mass", "seatstay_mass", "fork_lowers_mass"])
def test_changing_component_mass_changes_compiled_total(field):
    mass = BikeMassSpecs()
    setattr(mass, field, getattr(mass, field)+0.7)
    xml = generate_mujoco_xml(specs=BikeSpecs(), mode="ride", mass_specs=mass,
                              rider="none", physics_config=SimulationPhysicsConfig(physics_mode="physical"))
    model = mujoco.MjModel.from_xml_string(xml)
    assert model.body_mass.sum() == pytest.approx(mass.total_bike_mass, abs=1e-8)
```

2. Implement `assign_component_mass(geoms,total_kg)` by treating each geom's existing XML `mass` as a relative weight and distributing `total_kg` proportionally. Reject non-finite/non-positive totals; reject an empty list, invalid/negative weights, or non-positive total weight. Write mass values with 17 significant digits (`.17g`).

3. Each builder registers its component geoms in physical-component groups. Groups include:
   - frame: `motor`, `battery`, `crank_pedals`, `saddle_post`, and `frame_structure` separately;
   - fork: `steer_assembly`, `stanchions`, `fork_lowers`, and `front_wheel` separately;
   - rear linkage and shock damper each have their own groups.
   - Zero-mass debug geoms receive no mass.

   No measured component-mass dataset was supplied. Mark these physical component budgets/distributions as machine-readable `synthetic`, keyed by stable component ID, so the resolved release metadata can preserve provenance; do not present them as measured.

   Do not scale every frame geom with one coefficient: a frame-structure override must not change motor or battery mass. In physical mode populate all groups from the matching `BikeMassSpecs` fields. In legacy mode retain existing absolute geom masses. B1 owns mass distribution; B2 owns wheel inertial geometry, so do not assign a second wheel mass owner or explicit wheel inertial in this task.

4. `BikeMassSpecs` is mutable. Its 15 mass-budget fields are `motor_mass`, `battery_mass`, `frame_structure_mass`, `saddle_post_mass`, `crank_pedals_mass`, `steer_assembly_mass`, `stanchions_mass`, `fork_lowers_mass`, `chainstay_mass`, `seatstay_mass`, `rocker_mass`, `shock_yoke_mass`, `shock_damper_mass`, `front_wheel_mass`, and `rear_wheel_mass`. Exercise all actual mass fields, not just the six named in the first example, and do not treat `wheel_rim_tire_fraction` or `wheel_hub_core_fraction` as mass fields. Ensure every mass-bearing geom is included exactly once and debug geoms are excluded. `model.body_mass` includes world body mass zero, so summing it is valid.

5. Implement `compiled_center_of_mass(model,data)` as the mass-weighted sum of `data.xipos` divided by total compiled body mass. Reject a model with no positive physical mass. Add mass/CoM to telemetry through this helper and update MuJoCo kinematics before reading it. Preserve the old analytic CoM under a separate explicit name; do not use it as the dynamic marker. Check static load using actual contact points after sag, not unloaded axle locations.

6. Add keyword-only `mass_specs=None` to `RideSimulation` and preserve it through the single model-build call to the MJCF builder. Keep `physics_config` behavior from Task A1 and preserve legacy defaults.

7. Preserve geometric invariants: axle positions, hardpoints, and linkage lengths must not change when physical masses are modified. Add the requested test that translating `root_x` by 1 m moves compiled CoM by exactly 1 m without changing mass/inertia. Repeat override checks for remaining `BikeMassSpecs` fields.

### Verification and commit

- Before code changes, record the geometry invariant and retain tests proving it.
- Red: `uv run --locked pytest tests/test_compiled_mass_contract.py -q`; the prior behavior leaves some overrides disconnected from compiled masses.
- Focused: `uv run --locked pytest tests/test_compiled_mass_contract.py tests/test_geometry.py tests/test_linkage_kinematics.py -q`.
- Commit only Task B1 changes with message: `fix: drive compiled component masses from one budget`.

## Prior interfaces and decisions

- Task A1 provides `SimulationPhysicsConfig` and keyword-only `physics_config` plumbing in `RideSimulation` and MJCF builder.
- Phase A is reviewed/complete before dispatching B1. Use its current code, not the old spec-base implementation.
- B2 will convert wheel visual geoms to zero mass and transfer that same budget to one explicit body inertial. Do not implement B2 early.
- E1/E2 add rider bodies later; they must not become frame-mass registry entries or duplicate `BikeMassSpecs`.
- D1 may split crank/cassette bodies later while preserving this component mass budget.
- F2 consumes compiled CoM and must not substitute unloaded analytic CoM.
