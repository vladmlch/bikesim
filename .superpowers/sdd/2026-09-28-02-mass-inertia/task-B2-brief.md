# Task B2 brief — explicit physical wheel inertia tensors

Source: `docs/superpowers/plans/2026-09-28-02-mass-inertia.md`, Task B2. The physics-correctness specification is authoritative when a conflict appears.

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

## Task B2: explicit physical wheel tensors

**Files:** Create `src/bike_sim/physics/inertia.py`; modify `src/bike_sim/mujoco/steering_fork.py:114-139`, `src/bike_sim/mujoco/drivetrain.py` (`build_rear_wheel`), and `src/bike_sim/physics/mass.py:80-89`; test `tests/test_wheel_inertia_contract.py`.

**Interfaces:** Produce `ring_inertia(mass_kg,inner_m,outer_m,width_m)->np.ndarray`, `parallel_axis(mass_kg,offset_m)->np.ndarray`, `add_body_inertial(body,mass_kg,com_m,tensor)->None`. All tensors are in the local axes of the relevant body, with wheel axis = Y.

### Required behavior and acceptance

1. Start with tests for the annular-cylinder formula and compiled response:

```python
import numpy as np
import mujoco
import pytest
from bike_sim.physics.inertia import ring_inertia

def test_ring_inertia_has_mass_at_the_rim():
    tensor = ring_inertia(1.0, 0.30, 0.35, 0.06)
    assert tensor[1,1] == pytest.approx(0.10625)
    assert np.linalg.eigvalsh(tensor).min() > 0
    assert tensor[0,0]+tensor[2,2] >= tensor[1,1]

def test_fixed_axis_acceleration_uses_declared_inertia():
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <option gravity="0 0 0"/>
      <worldbody><body><joint type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.053425 0.10625 0.053425"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qfrc_applied[0] = 1.0625
    mujoco.mj_forward(model, data)
    assert data.qacc[0] == pytest.approx(10.0, rel=1e-3)
```

Before changing generated wheels, add an assertion comparing their compiled tensor against the selected ring model and confirm it fails. Run `uv run --locked pytest tests/test_wheel_inertia_contract.py -q` for the absent API red test.

2. Implement these formula/validation contracts:

```python
def ring_inertia(mass_kg, inner_m, outer_m, width_m):
    if not all(math.isfinite(v) for v in (mass_kg,inner_m,outer_m,width_m)):
        raise ValueError("non-finite inertia input")
    if mass_kg <= 0 or inner_m < 0 or outer_m <= inner_m or width_m <= 0:
        raise ValueError("invalid annular cylinder")
    radial = inner_m**2+outer_m**2
    transverse = mass_kg*(3*radial+width_m**2)/12
    return np.diag([transverse, mass_kg*radial/2, transverse])

def parallel_axis(mass_kg, offset_m):
    r = np.asarray(offset_m, dtype=float)
    if r.shape != (3,) or not np.isfinite(r).all() or not math.isfinite(mass_kg) or mass_kg < 0:
        raise ValueError("invalid parallel-axis input")
    return mass_kg*(float(r@r)*np.eye(3)-np.outer(r,r))

def add_body_inertial(body, mass_kg, com_m, tensor):
    com_m = np.asarray(com_m, dtype=float)
    tensor = np.asarray(tensor, dtype=float)
    if not math.isfinite(mass_kg) or mass_kg <= 0 or com_m.shape != (3,) or tensor.shape != (3,3):
        raise ValueError("invalid body inertial dimensions or mass")
    if not np.isfinite(com_m).all() or not np.isfinite(tensor).all():
        raise ValueError("non-finite body inertial")
    if not np.allclose(tensor, tensor.T, rtol=0, atol=1e-12):
        raise ValueError("inertia tensor must be symmetric")
    eig = np.linalg.eigvalsh(tensor)
    if eig[0] <= 0 or eig[2] > eig[0]+eig[1]+1e-12:
        raise ValueError("inertia violates positivity or triangle inequality")
    if body.find("inertial") is not None:
        raise ValueError("body already has an explicit inertial")
    values = (tensor[0,0],tensor[1,1],tensor[2,2],tensor[0,1],tensor[0,2],tensor[1,2])
    ET.SubElement(body, "inertial", {
        "mass": format(mass_kg,".17g"),
        "pos": " ".join(format(float(x),".17g") for x in com_m),
        "fullinertia": " ".join(format(float(x),".17g") for x in values),
    })
```

3. For each wheel, combine ring/core components, sum component masses, and transfer each tensor to the common CoM. Use this initial **synthetic, not measured** profile:
   - 75% wheel mass in ring; 25% in core;
   - front ring `(inner=0.320, outer=0.372, width=0.060)` m;
   - rear ring `(inner=0.300, outer=0.352, width=0.064)` m;
   - both core radii `0.045` m; front width `0.110` m, rear width `0.148` m;
   - all offsets are zero for this low-order profile.

   Store the components for each wheel in one immutable tuple of `(mass_fraction,inner_radius,outer_radius,width,offset)`, with fractions summing to 1. Pass one shared configuration to the generator and report. After D1, separate cassette mass/inertia from rear core without changing total budget. An eventual measured profile replaces this approximation as a whole.

4. In physical mode, visual wheel geoms have `mass=0`, and the wheel body has one explicit `<inertial>`. Preserve non-wheel body masses. Check that core/cassette stay on the correct body; D1 later moves the cassette to a separate body with the same budget. Legacy wheel behavior remains reproducible.

5. After compilation, compare the full tensor expressed in body frame: `R @ diag(model.body_inertia[id]) @ R.T`; obtain R from `model.body_iquat[id]` using `mju_quat2Mat`. Do not compare sorted/principal inertia indices without undoing the principal-axis rotation. Either redirect the old analytic helper to this selected physical calculation or leave it clearly legacy-only.

6. Verify energy for known angular speed on a fixed-axis stand and prove changing decorative geometry does not change inertia.

### Verification and commit

- `uv run --locked pytest tests/test_wheel_inertia_contract.py tests/test_compiled_mass_contract.py tests/test_mujoco_export.py -q`.
- Commit only B2 implementation/tests with message: `fix: assign physical wheel inertia independent of visual cylinders`.

## Prior interfaces and decisions

- B1 provides the physical wheel mass budget and mass registry. Replace each physical wheel visual-geom mass assignment with exactly one explicit body inertia; never double-count the wheel mass. Keep the B1 legacy path unchanged.
- Keep physical wheel profile/provenance synthetic until measured data exists.
- B2 does not split cassette body yet; D1 owns that follow-up while preserving total mass and the selected inertia convention.
- Preserve all geometric hardpoints and axle positions; inertia edits must not change geometry.

## Controller ruling on necessary integration plumbing

B2 may extend to `mujoco/builder.py`, `mujoco/rear_linkage.py`, and the B1 mass-contract test to plumb one shared wheel profile and let the component registry account for each explicit wheel inertial as its sole mass owner. The B1 registry validates geom groups, while B2 removes wheel geom masses; without this narrow extension, the builder would reject or omit wheel mass. Cost if wrong: mass can be dropped/duplicated or the physical build can fail at assembly.
