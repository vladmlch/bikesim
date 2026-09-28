# 01 — ядро и подвеска Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Изолировать новую физику, исключить перезапись сил и исправить активные законы подвески.

**Architecture:** Legacy остаётся отдельным путём фабрики. Physical получает единый resolved config, аккумулятор сил и параметризованные компоненты подвески. Чистые силовые функции проверяются до интеграции в MuJoCo.

**Tech Stack:** Python >=3.12, MuJoCo, NumPy, SciPy, pytest; existing uv.lock.

**Spec:** [2026-09-28-physics-correctness.md](../specs/2026-09-28-physics-correctness.md)

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

---

## Файлы и границы

A1 владеет `physics/model_config.py`; A2 — `sim/ride/force_accumulator.py`; A3 — `physics/suspension_config.py`; A4 изменяет `physics/damper.py`; A5 — `physics/coil_shock.py` и новый `physics/stops.py`; A6 — `physics/tuning.py` и новый `sim/ride/sag_fit.py`. Все пути ниже относительно корня репозитория.

Референсы: R01–R06, R15, R18 в спецификации. Строки после первого коммита могут сместиться; искать указанный символ, не перечитывать весь файл.

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

### Task A5: coil без растяжения и отдельный top-out

**Files:** Modify `src/bike_sim/physics/coil_shock.py:59-83`, `src/bike_sim/sim/ride/forces.py:106-124`; Create `src/bike_sim/physics/stops.py`; Test `tests/test_end_stops.py`.

**Interfaces:** Produces `end_stop(q,v,lo,hi,k,c)->tuple[float,float]` = generalized force и potential energy. Coil получает `legacy_behavior=False`.

- [ ] **Step 1: failing tests.**

```python
import pytest
from bike_sim.physics.coil_shock import CoilShock
from bike_sim.physics.stops import end_stop

def test_unloaded_compression_coil_does_not_pull():
    assert CoilShock().compute_spring_force(-4.0) == 0.0

def test_top_out_pushes_into_range_and_stores_energy():
    force, energy = end_stop(-0.002, -0.1, 0.0, 0.065, 200000.0, 500.0)
    assert force == pytest.approx(450.0)
    assert energy == pytest.approx(0.4)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_end_stops.py -q`; сначала отсутствует module, затем legacy coil-тест должен воспроизводимо краснеть до исправления.
- [ ] **Step 3: реализовать два разных элемента.**

```python
# Inside CoilShock.compute_spring_force, physical branch:
compression_m = max(0.0, (float(stroke_mm)+self.specs.preload_mm)/1000.0)
return self.specs.rate_n_m * compression_m
```

```python
from math import isfinite

def end_stop(q, v, lo, hi, k, c):
    if not all(isfinite(x) for x in (q,v,lo,hi,k,c)) or hi <= lo or k <= 0 or c < 0:
        raise ValueError("invalid end-stop parameters")
    low_depth = max(lo-q, 0.0)
    high_depth = max(q-hi, 0.0)
    force = 0.0
    if low_depth > 0:
        force += max(0.0, k*low_depth-c*v)
    if high_depth > 0:
        force -= max(0.0, k*high_depth+c*v)
    return force, 0.5*k*(low_depth**2+high_depth**2)
```

Не прибавлять новый upper stop поверх существующего bumper в том же диапазоне. В writer использовать отдельно top-out ниже нуля, bumper в своей зоне и только аварийный upper stop за рабочим ходом. Изменить пределы MJCF physical совместно с аварийной деформацией; legacy ranges не менять. Записать каждый компонент энергии/силы в named contribution. Проверять preload>=0, rate>0, stroke>0 и `0<bumper_length<=stroke`.

- [ ] **Step 4:** `uv run --locked pytest tests/test_end_stops.py tests/test_coil_shock.py -q`. Добавить release-сценарий с negative stroke и обратной скоростью, проверяя отсутствие генерации энергии.
- [ ] **Step 5:** `git add src/bike_sim/physics/coil_shock.py src/bike_sim/physics/stops.py src/bike_sim/sim/ride/forces.py tests/test_end_stops.py && git commit -m "fix: separate compression spring and end-stop forces"`.

### Task A6: исправить приведённую массу и численно подбирать sag

**Files:** Modify `src/bike_sim/physics/tuning.py:75-94`; Create `src/bike_sim/sim/ride/sag_fit.py`; Test `tests/test_sag_physics.py`; существующий `tests/test_mass_distribution.py` менять только в assertions исправленного аналитического расчёта.

**Interfaces:** Produces `reflected_shock_mass(mass_kg,leverage_ratio)->float`; `fit_sag(evaluate,target_mm,initial,bounds)->np.ndarray`, где `evaluate(psi,rate)->tuple[float,float]` возвращает фактические front/rear wheel travel в mm.

- [ ] **Step 1: failing tests с независимыми ожидаемыми значениями.**

```python
import numpy as np
import pytest
from bike_sim.physics.tuning import reflected_shock_mass
from bike_sim.sim.ride.sag_fit import fit_sag

def test_reflected_mass_follows_kinetic_energy():
    assert reflected_shock_mass(50.0, 3.0) == 450.0

def test_sag_fit_solves_two_positive_parameters():
    def evaluate(psi, rate):
        return 5000.0/psi, 5e6/rate
    result = fit_sag(evaluate, (50.0,50.0), (80.0,90000.0),
                     ((20.0,20000.0),(200.0,300000.0)))
    np.testing.assert_allclose(result, (100.0,100000.0), rtol=1e-4)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_sag_physics.py -q`; ожидается отсутствующий API.
- [ ] **Step 3: реализовать преобразование и ограниченный поиск.**

```python
from math import isfinite

def reflected_shock_mass(mass_kg, leverage_ratio):
    if not all(isfinite(v) for v in (mass_kg, leverage_ratio)) or mass_kg <= 0 or leverage_ratio <= 0:
        raise ValueError("mass and leverage ratio must be positive")
    return mass_kg * leverage_ratio**2
```

```python
import numpy as np
from scipy.optimize import least_squares

def fit_sag(evaluate, target_mm, initial, bounds):
    target = np.asarray(target_mm, dtype=float)
    lower, upper = map(lambda x: np.asarray(x, dtype=float), bounds)
    x0 = np.asarray(initial, dtype=float)
    arrays = (target, lower, upper, x0)
    if any(a.shape != (2,) or not np.isfinite(a).all() for a in arrays):
        raise ValueError("expected finite two-element sag arrays")
    if np.any(target <= 0) or np.any(lower <= 0) or np.any(upper <= lower):
        raise ValueError("invalid sag target or bounds")
    if np.any(x0 < lower) or np.any(x0 > upper):
        raise ValueError("initial sag parameters lie outside bounds")
    def residual(log_values):
        measured = np.asarray(evaluate(*np.exp(log_values)), dtype=float)
        if measured.shape != (2,) or not np.isfinite(measured).all():
            raise ValueError("invalid equilibrium output")
        return measured-target
    result = least_squares(residual, np.log(x0), bounds=(np.log(lower),np.log(upper)))
    if not result.success or np.max(np.abs(residual(result.x))) > 0.5:
        raise RuntimeError("requested sag is not achievable within parameter bounds")
    return np.exp(result.x)
```

Заменить division by LR² в `_compute_shock_tuning` на новый helper. В реальном `evaluate` создавать конфигурацию с пробными pressure/rate, активной A3-фабрикой, теми же массами/райдером/terrain и читать результат `solve_static_equilibrium`. Все остальные настройки фиксировать. Не подставлять целевой sag в `qpos`.

Проверки NaN/inf для helper и массивов target/initial/bounds выполняются в приведённом ядре до логарифма; добавить тесты, передающие NaN и отрицательные значения в каждый из этих аргументов и ожидающие ValueError. Зафиксировать отдельно тест недостижимого target. Старое ожидаемое damping-число заменить расчётом из энергии, не увеличивать tolerance.

- [ ] **Step 4:** `uv run --locked pytest tests/test_sag_physics.py tests/test_mass_distribution.py tests/test_ride_equilibrium.py -q`.
- [ ] **Step 5:** `git add src/bike_sim/physics/tuning.py src/bike_sim/sim/ride/sag_fit.py tests/test_sag_physics.py tests/test_mass_distribution.py && git commit -m "fix: reflect shock mass correctly and solve requested sag"`.

## Review gate A

Проверить, что physical не пишет внешнюю стабилизацию root; legacy действительно воспроизводится своей фабрикой; ни один новый writer не затирает чужой вклад; HBO проходит обе ветви; изменение публичных параметров доходит до runtime. Новый sag fitter не превращается в источник сил во время заезда.