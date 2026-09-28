# Task C4 brief — passive tangential brush return-map

Source: `docs/superpowers/plans/2026-09-28-03-tire-contact.md`, Task C4. TIRE-03 in the physics-correctness spec is authoritative.

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

## Task C4: tangential compliance, stick/slip, and energy

**Files:** Modify `src/bike_sim/physics/tire.py`; test `tests/test_tire_brush.py`.

**Interface:** Produce `brush_step(xi,u,v_roll,Fn,k,mu,length,dt)->tuple[xi_new,Fx,dissipation_step_j]`. The final output is a nonnegative discrete work residual under the chosen quadrature, including numerical dissipation; do not label it wholly as measured tire-rubber hysteresis.

### Required behavior and acceptance

1. Add the force-limit and discrete-passivity tests:

```python
import pytest
from bike_sim.physics.tire import brush_step

def test_drive_slip_creates_forward_force_within_friction_limit():
    xi, force, loss = brush_step(0.0,-1.0,5.0,100.0,20000.0,0.5,0.2,0.01)
    assert 0 < force <= 50.0
    assert force*(-1.0)*0.01+0.5*20000*xi**2+loss == pytest.approx(0.0, abs=1e-12)
    assert loss >= 0.0

def test_static_shear_can_hold_without_wheel_rotation():
    xi, force, loss = brush_step(0.001,0.0,0.0,100.0,20000.0,0.5,0.2,0.01)
    assert force == pytest.approx(-20.0)
    assert loss == pytest.approx(0.0)
```

2. Implement the pure return-map with validation:

```python
def brush_step(xi, u, v_roll, Fn, k, mu, length, dt):
    if not all(isfinite(x) for x in (xi,u,v_roll,Fn,k,mu,length,dt)):
        raise ValueError("non-finite brush input")
    if Fn < 0 or k <= 0 or mu < 0 or length <= 0 or dt <= 0:
        raise ValueError("invalid brush parameters")
    old_energy = 0.5*k*xi*xi
    if Fn == 0:
        return 0.0, 0.0, old_energy
    decay = abs(v_roll)/length
    trial = (xi+dt*u)/(1+dt*decay)
    limit = mu*Fn/k
    new_xi = max(-limit,min(limit,trial))
    force = -k*new_xi
    new_energy = 0.5*k*new_xi*new_xi
    loss = -force*u*dt-(new_energy-old_energy)
    if loss < -1e-9:
        raise ArithmeticError("brush update violates discrete passivity")
    return new_xi, force, max(loss,0.0)
```

   Units: `xi/length` m; `u/v_roll` m/s; `Fn` N; `k` N/m; `mu` dimensionless; `dt` s; outputs m, N, J. State `xi` is meters. Clamp output force to `mu*Fn`. The force opposes relative surface velocity `u` at the tire contact; `u` is not merely root-x speed minus relative hinge speed. At zero roll speed, static shear may persist.

3. On `Fn==0`, reset shear and report the removed spring energy as nonnegative loss; never silently drop stored energy. This is a pure return-map only: C5 owns per-wheel persistent state, tangent-frame transport, contact-branch release and separate loss events. Never carry `xi` front-to-rear.

4. Add load-drop and velocity-reversal checks, and exercise a grid of velocities, loads and initial `xi` for nonnegative discrete loss/passivity. A force limit must not become an input motor-torque limiter; wheelspin must remain possible.

### Verification and commit

- Red: `uv run --locked pytest tests/test_tire_brush.py -q`; expected missing `brush_step`.
- Focused: `uv run --locked pytest tests/test_tire_brush.py tests/test_tire_radial.py -q`.
- Add/run a dynamic launch without motor-torque limiting to confirm physically possible slip. Do not claim calibration from this synthetic test.
- Commit only C4 implementation/tests with message: `feat: add passive tangential tire state and friction saturation`.

## Prior interfaces and decisions

- C3 provides the SI radial law and explicit `TireSpec`; keep the pure radial and tangential laws in the same `physics/tire.py` and preserve its force/energy sign conventions.
- C1 contact snapshots provide absolute point velocity and exact world force; use the actual relative surface velocity at the contact in any integration/launch test.
- C5 owns persistent brush state and transforms the tangent frame across normal changes; C4 stays a pure stateless return-map.
