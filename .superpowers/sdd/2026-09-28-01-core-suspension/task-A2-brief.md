# Task A2 brief — exact requirements

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

### Task A2: аккумулятор сил и однозначный момент выборки

**Files:** Create `src/bike_sim/sim/ride/force_accumulator.py`; Modify `src/bike_sim/sim/ride_sim.py:192-223`, `src/bike_sim/sim/ride/forces.py:86-124`; Test `tests/test_force_accumulator.py`.

**Interfaces:** Consumes `SimulationPhysicsConfig`; Produces `ForceAccumulator(nv)`, `clear()`, `add(name,qfrc)`, `total()`, read-only копии `components` для F1.

- [ ] **Step 1: failing test против потери/удвоения вклада.**

```python
import numpy as np
import pytest
from bike_sim.sim.ride.force_accumulator import ForceAccumulator

def test_two_writers_on_one_dof_add_without_aliasing():
    acc = ForceAccumulator(2)
    source = np.array([3.0, 0.0])
    acc.add("spring", source)
    source[0] = 1000.0
    acc.add("chain", np.array([-1.0, 2.0]))
    np.testing.assert_array_equal(acc.total(), [2.0, 2.0])
    with pytest.raises(ValueError, match="duplicate"):
        acc.add("chain", np.zeros(2))
    acc.clear()
    np.testing.assert_array_equal(acc.total(), [0.0, 0.0])
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_force_accumulator.py -q`; ожидается отсутствующий модуль.
- [ ] **Step 3: реализация.**

```python
import numpy as np

class ForceAccumulator:
    def __init__(self, nv: int):
        self.nv = nv
        self._components = {}

    def clear(self):
        self._components.clear()

    def add(self, name: str, qfrc: np.ndarray):
        value = np.asarray(qfrc, dtype=float)
        if value.shape != (self.nv,) or not np.isfinite(value).all():
            raise ValueError("invalid generalized force")
        if name in self._components:
            raise ValueError("duplicate force component")
        self._components[name] = value.copy()

    @property
    def components(self):
        return {name: value.copy() for name, value in self._components.items()}

    def total(self):
        result = np.zeros(self.nv)
        for value in self._components.values():
            result += value
        return result
```

В physical writer подвески добавить метод `compute_qfrc(model,data)->np.ndarray`: прежняя математика, но запись в локальный нулевой вектор. Старый `apply` для legacy остаётся адаптером. Physical step: `mj_forward` → `acc.clear` → сбор всех вкладов → `data.qfrc_applied[:]=acc.total()` → копия `(time,qpos,qvel,components)` → `mj_step`. Внешние силы, если нужны, передавать явно в `step(external_qfrc=...)`, валидировать форму и добавлять компонентом `external`. Нельзя очищать пользовательский input до его регистрации.

- [ ] **Step 4:** `uv run --locked pytest tests/test_force_accumulator.py tests/test_ride_controllers.py -q`. Проверить два последовательных physical шага без накопления силы прошлого шага.
- [ ] **Step 5:** `git add src/bike_sim/sim/ride/force_accumulator.py src/bike_sim/sim/ride_sim.py src/bike_sim/sim/ride/forces.py tests/test_force_accumulator.py && git commit -m "refactor: accumulate named physical force contributions"`.


