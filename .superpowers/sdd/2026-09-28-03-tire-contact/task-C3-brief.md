# Task C3 brief — normal tire compliance in physical units

Source: `docs/superpowers/plans/2026-09-28-03-tire-contact.md`, Task C3. TIRE-02/TIRE-03 in the physics-correctness spec are authoritative.

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

## Task C3: radial compliance with material units

**Files:** Create `src/bike_sim/physics/tire.py`; test `tests/test_tire_radial.py`.

**Interfaces:** Produce `normal_contact(delta,delta_dot,k,c)->tuple[force_n,energy_j]`. `TireSpec` has positive `radial_k_n_m`, nonnegative `radial_c_ns_m`, `pressure_pa_gauge`, `provenance`, and `valid_load_range_n`. Native `solref`/`solimp` are stored separately from these values.

### Required behavior and acceptance

1. Tests must check force/material stiffness, stored elastic energy, and unilateral lift-off behavior:

```python
import pytest
from bike_sim.physics.tire import normal_contact

def test_material_stiffness_gives_known_deflection():
    force, energy = normal_contact(500/130000, 0.0, 130000.0, 800.0)
    assert force == pytest.approx(500.0)
    assert energy == pytest.approx(0.5*500*(500/130000))

def test_approaching_airborne_wheel_has_no_premature_damping():
    assert normal_contact(-0.001, 2.0, 130000.0, 800.0) == (0.0,0.0)
```

2. Implement the unilateral physical law:

```python
def normal_contact(delta, delta_dot, k, c):
    if not all(isfinite(x) for x in (delta,delta_dot,k,c)) or k <= 0 or c < 0:
        raise ValueError("invalid radial contact parameters")
    if delta <= 0:
        return 0.0, 0.0
    return max(0.0,k*delta+c*delta_dot), 0.5*k*delta**2
```

   Inputs use SI: penetration m, penetration rate m/s, stiffness N/m, damping N*s/m; output N and J. At `delta<=0`, force and energy are exactly zero independent of velocity. Do not generate tensile force. With unload/negative `delta_dot`, clamp force nonnegative.

3. `130000 N/m` and `800 N*s/m` are synthetic rig values only. They are not an interpretation or copy of native MuJoCo `solref`/`solimp`; keep native solver configuration separate and do not call its raw values tire material stiffness/damping.

4. Provide an explicit `TireSpec` consumed by the law/configuration path, not hidden defaults. It carries positive radial stiffness, nonnegative damping, gauge pressure in Pa, provenance, and an explicit valid load range. Synthetic values are labelled `synthetic`; no experimental calibration is implied.

5. If a calibrated force-deflection characteristic is represented, accept a monotone table with physical units, pressure/load range and experiment conditions; interpolate only inside the valid range and raise outside it in calibrated mode. Do not add an unsupported universal `k proportional to pressure` relation. Linear mode remains a valid simple law.

6. Tests must check linear material behavior for 100/300/600/1000 N on the same `k`, unload with negative `delta_dot`, no stretch outside contact, and equality of elastic energy to the force-displacement integral. Native reference effective response is measured on a separate stand, not used to silently overwrite the material law.

### Verification and commit

- Red: `uv run --locked pytest tests/test_tire_radial.py -q`; expected missing API.
- Focused: `uv run --locked pytest tests/test_tire_radial.py -q`.
- Commit only C3 implementation/tests with message: `feat: add unilateral radial tire compliance in SI units`.

## Prior interfaces and decisions

- C1 now supplies immutable contact snapshots and world point kinematics. C2 will separate controller grounded filtering; neither changes the pure radial law.
- C3 introduces only pure radial law/configuration; C4 extends the same `physics/tire.py` with brush tangential law; C5 owns geometry and force application.
- Keep native solver settings and compliant material settings as separate explicit backends. A failed calibrated range never silently selects another backend.
