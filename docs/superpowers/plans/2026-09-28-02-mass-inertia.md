# 02 — массы и инерции Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Сделать конфигурацию масс источником скомпилированной динамики и отделить инерцию колёс от их внешнего вида.

**Architecture:** Массы задаются бюджетами компонентов; геометрии получают нормированные доли либо явный inertial. Динамический CoM читается из MuJoCo. Тензоры рассчитываются в одном физическом модуле и используются генератором и тестами.

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

## Файлы и зависимости

Требуется A1. B1 изменяет только массу/её передачу в builders, B2 — расчёт инерций колёс. Перед каждым изменением зафиксировать invariant геометрии: положения осей, hardpoints и длины звеньев не должны изменяться от корректировки массы.

### Task B1: единый массовый бюджет и динамический CoM

**Files:** Create `src/bike_sim/physics/component_masses.py`, `src/bike_sim/sim/ride/mass_properties.py`; Modify `src/bike_sim/physics/mass.py:62-89`, builders `mujoco/frame.py`, `mujoco/steering_fork.py:114-139`, `mujoco/rear_linkage.py`, `mujoco/drivetrain.py:30-103`, `mujoco/builder.py`, `sim/ride_sim.py`; Test `tests/test_compiled_mass_contract.py`.

**Interfaces:** Produces `assign_component_mass(geoms,total_kg)->None`, `compiled_center_of_mass(model,data)->np.ndarray`. Добавить keyword-only `mass_specs=None` в `RideSimulation` и передать без потери в MJCF builder. Existing builders возвращают/накапливают Python-registry `component_id -> list[ET.Element]`, не добавляя невалидные custom attributes в XML.

- [ ] **Step 1: failing tests конфигурации и суммарной массы.**

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

- [ ] **Step 2:** `uv run --locked pytest tests/test_compiled_mass_contract.py -q`. До исправления часть overrides не меняет скомпилированную массу.
- [ ] **Step 3: нормировать массы геомов внутри каждого компонента.**

```python
import math

def assign_component_mass(geoms, total_kg):
    weights = [float(geom.get("mass", "0")) for geom in geoms]
    if not math.isfinite(total_kg) or total_kg <= 0:
        raise ValueError("component mass must be finite and positive")
    if not weights or any(not math.isfinite(w) or w < 0 for w in weights) or sum(weights) <= 0:
        raise ValueError("component has no valid mass weights")
    denominator = sum(weights)
    for geom, weight in zip(geoms, weights):
        geom.set("mass", format(total_kg*weight/denominator, ".17g"))
```

Каждый builder регистрирует свои геомы в группе физического компонента. У рамы отдельны motor, battery, crank_pedals, saddle_post и frame_structure. У fork отдельны steer_assembly, stanchions, fork_lowers и front_wheel. Задние звенья и shock damper имеют собственные группы. Нулевые debug-геомы не получают массу. Нельзя масштабировать все геомы frame одним коэффициентом: это изменит мотор/батарею при правке структуры.

В physical заполнить все группы из соответствующих полей `BikeMassSpecs`; в legacy оставить прежние абсолютные геом-массы. Для массы, записываемой в XML, использовать 17 значащих цифр, а не трёхзначное округление.

```python
import numpy as np

def compiled_center_of_mass(model, data):
    masses = np.asarray(model.body_mass)
    total = float(masses.sum())
    if total <= 0:
        raise ValueError("model has no physical mass")
    return (masses[:, None]*data.xipos).sum(axis=0)/total
```

Добавить mass/CoM к телеметрии через этот helper; перед чтением актуализировать кинематику. Старый analytic CoM оставить под отдельным именем, не подменять им динамический marker. Проверку static load выполнять по фактическим точкам контакта после sag, а не по ненагруженным осям.

- [ ] **Step 4:** `uv run --locked pytest tests/test_compiled_mass_contract.py tests/test_geometry.py tests/test_linkage_kinematics.py -q`. Добавить тест переноса `root_x` на 1 м: CoM перемещается ровно на 1 м, масса/инерция не меняются. Повторить overrides для остальных полей BikeMassSpecs.
- [ ] **Step 5:** `git add src/bike_sim/physics/component_masses.py src/bike_sim/sim/ride/mass_properties.py src/bike_sim/physics/mass.py src/bike_sim/mujoco src/bike_sim/sim/ride_sim.py tests/test_compiled_mass_contract.py && git commit -m "fix: drive compiled component masses from one budget"`.

### Task B2: явные физические тензоры колёс

**Files:** Create `src/bike_sim/physics/inertia.py`; Modify `src/bike_sim/mujoco/steering_fork.py:114-139`, `src/bike_sim/mujoco/drivetrain.py` (`build_rear_wheel`), `src/bike_sim/physics/mass.py:80-89`; Test `tests/test_wheel_inertia_contract.py`.

**Interfaces:** Produces `ring_inertia(mass_kg,inner_m,outer_m,width_m)->np.ndarray`, `parallel_axis(mass_kg,offset_m)->np.ndarray`, `add_body_inertial(body,mass_kg,com_m,tensor)->None`. Все тензоры выражены в локальных осях соответствующего body, wheel axis=Y.

- [ ] **Step 1: проверка кольца и наблюдаемого ускорения.**

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

- [ ] **Step 2:** `uv run --locked pytest tests/test_wheel_inertia_contract.py -q`; сначала отсутствует ring_inertia. До изменения generated wheels добавить assertion compiled tensor vs selected ring model и подтвердить его красный результат.
- [ ] **Step 3: единая математика и явный inertial.**

```python
import math
import numpy as np
import xml.etree.ElementTree as ET

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

Для каждого wheel собрать кольца/ядро по одной физической конфигурации, суммировать массы и перенести тензоры к общему CoM. Начальный synthetic-профиль: 75% массы в кольце и 25% в ядре; front ring `(inner=0.320, outer=0.372, width=0.060)` m, rear ring `(0.300,0.352,0.064)` m; front core — сплошной цилиндр радиусом 0.045 m и шириной 0.110 m, rear core — радиусом 0.045 m и шириной 0.148 m. Offsets в этом низкопорядковом synthetic-профиле равны нулю. Это не измеренные инерции. Хранить компоненты в одном immutable tuple `(mass_fraction,inner_radius,outer_radius,width,offset)` на колесо; сумма fractions=1, одна конфигурация передаётся генератору и отчёту. После D1 отделить массу/инерцию кассеты от rear core без изменения суммарного бюджета, а измеренный профиль должен заменять эту аппроксимацию целиком.

В physical визуальные wheel-geoms получают `mass=0`, а body — `<inertial>`. Не стирать ненулевую массу других body. Проверять, что корпус/кассета остаются на правильном теле; D1 позже перенесёт кассету в отдельный body с тем же бюджетом.

После компиляции восстановить тензор в body-frame: `R @ diag(model.body_inertia[id]) @ R.T`, где R получен из `model.body_iquat[id]` через `mju_quat2Mat`. Не сравнивать индекс principal inertia без учёта поворота главных осей. Аналитический старый helper перенаправить на тот же выбранный physical расчёт либо оставить явно legacy-only.

- [ ] **Step 4:** `uv run --locked pytest tests/test_wheel_inertia_contract.py tests/test_compiled_mass_contract.py tests/test_mujoco_export.py -q`. Проверить энергию при известном omega на закреплённом стенде и неизменность инерции при изменении декоративной геометрии.
- [ ] **Step 5:** `git add src/bike_sim/physics/inertia.py src/bike_sim/physics/mass.py src/bike_sim/mujoco/steering_fork.py src/bike_sim/mujoco/drivetrain.py tests/test_wheel_inertia_contract.py && git commit -m "fix: assign physical wheel inertia independent of visual cylinders"`.

## Review gate B

Сверить полную таблицу mass fields и compiled masses, а не только total. В отчёте должны различаться ненагруженный analytic CoM и текущий compiled CoM. Любые новые кольцевые параметры подписаны synthetic до измерений; предыдущие 0.250/0.261 kg*m² из аналитического приближения не являются обязательными целями.