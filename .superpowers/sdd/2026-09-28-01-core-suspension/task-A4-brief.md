# Task A4 brief — exact requirements

## Global Constraints

- Базовая ревизия: 70623815b98788018bcdbd8eef347d778f9bb3f3.
- Python >=3.12; использовать существующий uv.lock, не обновлять зависимости в PR физики.
- Новые численные зависимости не добавлять: NumPy, SciPy, MuJoCo и pytest уже объявлены проектом.
- Новая физика использует SI: m, s, kg, N, N*m, rad; mm и km/h допустимы только на совместимых внешних границах.
- Новая физика остаётся плоской X-Z; боковое сцепление и баланс по крену не заявляются.
- В режиме physical запрещены внешняя стабилизация тангажа, ручной перенос веса и присваивание qpos/qvel после шага для исправления физики.
- legacy и physical имеют разные physics_revision и разные численные эталоны.
- Параметры без измерений помечаются synthetic; прохождение синтетических тестов не считается экспериментальной валидацией.
- Все новые численные допуски являются критериями приёмки, а не результатами уже выполненных испытаний.

## Task body

### Task A4: общий HBO и непрерывный Firm

**Files:** Modify `src/bike_sim/physics/damper.py:314-330`; Test `tests/test_damper_physics.py`.

**Interfaces:** Consumes настроенный `SuperDeluxeDamper`; Produces исправленный `compute_damping_force(v,stroke_mm)` и отдельный совместимый legacy branch.

- [ ] **Step 1: добавить тесты существующего дефекта.**

```python
import pytest
from bike_sim.physics.damper import SuperDeluxeDamper

@pytest.mark.parametrize("firm", [False, True])
def test_hbo_adds_force_in_both_modes(firm):
    damper = SuperDeluxeDamper(lockout_firm=firm)
    assert damper.compute_damping_force(1.0, 65.0) > damper.compute_damping_force(1.0, 20.0)

def test_firm_force_is_continuous():
    damper = SuperDeluxeDamper(lockout_firm=True)
    left = damper.compute_damping_force(0.03-1e-8, 20.0)
    right = damper.compute_damping_force(0.03+1e-8, 20.0)
    assert abs(right-left) < 1e-3
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_damper_physics.py -q`; ожидаются fail для Firm/HBO и скачка силы.
- [ ] **Step 3: заменить только исправленную ветвь вычисления.**

```python
v = float(velocity_mps)
coeffs = self.get_effective_coefficients()
if self.lockout_firm and v > 0.0:
    knee = 0.03
    low = self.lockout_stiffness
    high = 1.8 * coeffs["c_hsc"]
    f_damp = low*v if v < knee else low*knee + high*(v-knee)
else:
    f_damp = self._compute_base_damping(v, coeffs["c_lsc"], coeffs["c_hsc"], coeffs["c_reb"])
if stroke_mm > self.hbo_start_mm and v > 0.0:
    fraction = min(1.0, (stroke_mm-self.hbo_start_mm)/(self.total_stroke_mm-self.hbo_start_mm))
    f_damp += coeffs["c_hbo"] * fraction**2 * v
return float(f_damp)
```

Старый body метода переместить без изменений в `_compute_legacy_damping_force`; вызов только при `self.legacy_behavior`. Новый параметр `legacy_behavior=False` добавить в конструктор SuperDeluxeDamper и передать из фабрики. Это сохраняет возможность воспроизведения старой модели, но не закрепляет ошибку в physical. В тесте пассивности перебрать velocities `[-2,-0.2,-1e-6,0,1e-6,0.03,0.5,2]` и strokes `[0,20,52,65]` для обоих режимов.

- [ ] **Step 4:** `uv run --locked pytest tests/test_damper_physics.py tests/test_ride_controllers.py -q`.
- [ ] **Step 5:** `git add src/bike_sim/physics/damper.py tests/test_damper_physics.py && git commit -m "fix: preserve HBO in firm mode and remove force jump"`.


