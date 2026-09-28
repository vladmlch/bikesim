# Task A1 brief — exact requirements

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

### Task A1: разделить режимы и запретить внешнюю помощь в physical

**Files:** Create `src/bike_sim/physics/model_config.py`; Modify `src/bike_sim/sim/ride_sim.py:130-223`, `src/bike_sim/mujoco/builder.py` (сигнатура `generate_mujoco_xml`), `src/bike_sim/sim/ride/virtual_rider.py:111-128`; Test `tests/test_physics_config.py`.

**Interfaces:** Consumes существующий `RideSimulation`; Produces `SimulationPhysicsConfig`, keyword-only `RideSimulation(..., physics_config=None)` и `generate_mujoco_xml(..., physics_config=None)`.

- [ ] **Step 1: добавить failing tests.**

```python
import pytest
from bike_sim.physics.model_config import SimulationPhysicsConfig

def test_physical_defaults_to_coast_without_external_pitch():
    cfg = SimulationPhysicsConfig(physics_mode="physical")
    assert cfg.drive_mode == "coast"
    assert cfg.pitch_assist is False

def test_external_pitch_is_rejected_in_physical():
    with pytest.raises(ValueError, match="pitch"):
        SimulationPhysicsConfig(physics_mode="physical", pitch_assist=True)
```

- [ ] **Step 2: подтвердить красный тест.** `uv run --locked pytest tests/test_physics_config.py -q`. Ожидается ошибка импорта нового модуля.
- [ ] **Step 3: реализовать контракт и развести ветви оркестратора.**

```python
from dataclasses import dataclass
from math import isfinite

@dataclass(frozen=True)
class SimulationPhysicsConfig:
    physics_mode: str = "legacy"
    drive_mode: str | None = None
    timestep_s: float = 0.0005
    pitch_assist: bool | None = None

    def __post_init__(self):
        if self.physics_mode not in {"legacy", "physical"}:
            raise ValueError("unknown physics_mode")
        if not isfinite(self.timestep_s) or self.timestep_s <= 0:
            raise ValueError("timestep_s must be finite and positive")
        legacy = self.physics_mode == "legacy"
        if self.drive_mode is None:
            object.__setattr__(self, "drive_mode", "ideal_speed_control" if legacy else "coast")
        if self.pitch_assist is None:
            object.__setattr__(self, "pitch_assist", legacy)
        if self.drive_mode not in {"coast", "ideal_speed_control", "crank_effort", "articulated_effort"}:
            raise ValueError("unknown drive_mode")
        if not legacy and self.pitch_assist:
            raise ValueError("external pitch assist is forbidden in physical")
```

Сохранить существующий `step` как `_step_legacy`. В `_step_physical` не вызывать `PitchStabilizer.apply`; в coast записывать нулевые drive controls каждый шаг. На этой стадии effort-режимы валидны как конфигурация, но создание симуляции с ними до D1–D4 должно выдавать `NotImplementedError` с именем режима, а не включать wheel cruise. После D4 этот временный guard удаляется, после E4 снимается guard articulated. Инициализация `physics_config=None` идёт строго в legacy. Записать `physics_revision` в объекте симуляции.

- [ ] **Step 4: зелёный прогон.** `uv run --locked pytest tests/test_physics_config.py tests/test_ride_model.py -q`. Дополнительно создать physical/coast, поднять его над поверхностью, сделать шаг и проверить нулевой вклад внешнего root-pitch.
- [ ] **Step 5: commit.** `git add src/bike_sim/physics/model_config.py src/bike_sim/sim/ride_sim.py src/bike_sim/mujoco/builder.py src/bike_sim/sim/ride/virtual_rider.py tests/test_physics_config.py && git commit -m "feat: separate legacy and physical simulation modes"`.


