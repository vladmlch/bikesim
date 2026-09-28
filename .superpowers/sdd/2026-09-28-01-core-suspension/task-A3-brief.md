# Task A3 brief — exact requirements

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

### Task A3: единая фабрика подвески

**Files:** Create `src/bike_sim/physics/suspension_config.py`; Modify `src/bike_sim/sim/ride_sim.py:136-141,342-351`, `src/bike_sim/physics/damper.py:350-372`; Test `tests/test_suspension_config.py`.

**Interfaces:** Consumes `BikeSpecs`; Produces `build_suspension_components(specs, *, preload_mm=0.0)->tuple[SuspensionController,CoilShock]`. `BikeSuspensionSystem` получает keyword-only `fork_travel_mm=180.0`, `shock_stroke_mm=65.0`, `legacy_behavior=False`.

- [ ] **Step 1: тест параметров, которые сейчас не доходят до runtime.**

```python
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.suspension_config import build_suspension_components

def test_public_spec_reaches_running_components():
    specs = BikeSpecs(fork_lsc=2, shock_hsc=4, shock_stiffness=90000.0,
                      fork_travel=170.0, shock_stroke=60.0)
    controller, coil = build_suspension_components(specs)
    assert controller.suspension_system.fork_damper.lsc_clicks == 2
    assert controller.suspension_system.shock_damper.hsc_clicks == 4
    assert coil.specs.rate_n_m == 90000.0
    assert coil.specs.stroke_mm == 60.0
    assert controller.suspension_system.fork_damper.total_travel_mm == 170.0
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_suspension_config.py -q`; ожидается ошибка импорта.
- [ ] **Step 3: построить компоненты одной функцией.**

```python
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs
from bike_sim.physics.damper import BikeSuspensionSystem, DamperClickConfig
from bike_sim.sim.controllers import SuspensionController

def build_suspension_components(specs, *, preload_mm=0.0):
    clicks = DamperClickConfig(**{
        name: getattr(specs, name) for name in (
            "fork_hsc", "fork_lsc", "fork_rebound", "shock_hsc", "shock_lsc",
            "shock_rebound", "shock_hbo", "shock_lockout")
    })
    dampers = BikeSuspensionSystem(clicks, fork_travel_mm=specs.fork_travel,
                                  shock_stroke_mm=specs.shock_stroke)
    air = ForkAirSpring(AirSpringSpecs(total_travel_mm=specs.fork_travel),
                        num_tokens=specs.fork_air_tokens,
                        gauge_pressure_psi=specs.fork_initial_psi)
    coil = CoilShock(CoilShockSpecs(rate_n_m=specs.shock_stiffness,
                                   preload_mm=preload_mm, stroke_mm=specs.shock_stroke))
    return SuspensionController(specs, air, dampers), coil
```

В конструкторах демпферов передать ходы и вычислить зоны HBO по SUS-02. Проверять конечные положительные ходы и `0<zone<=travel`. Проверить overrides до начала симуляции. Для legacy оставить прежнюю фабрику и явно установить `legacy_behavior=True` в демпфере/coil после добавления соответствующих flags в A4–A5. Default нового standalone компонента — исправленный закон.

- [ ] **Step 4:** `uv run --locked pytest tests/test_suspension_config.py -q`. Добавить параметризацию всех девяти кликов/переключателей из тестируемого mapping и конфликт хода override.
- [ ] **Step 5:** `git add src/bike_sim/physics/suspension_config.py src/bike_sim/physics/damper.py src/bike_sim/sim/ride_sim.py tests/test_suspension_config.py && git commit -m "fix: resolve suspension parameters into active components"`.


