# 03 — контакт покрышек и сопротивления Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Разделить реальные контактные силы и управляющие признаки, добавить проверяемую податливость шины и корректные внешние сопротивления.

**Architecture:** Native contact сохраняется отдельным reference backend. Новый compliant_2d получает геометрию из того же дискретного профиля, рассчитывает пассивные силы и применяет их один раз через Cartesian-to-generalized mapping. Управляющий фильтр не участвует в физическом контакте.

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

Требуются A1–A2 и B2 для динамических стендов. C1 задаёт контракт, C2 — независимый фильтр, C3–C4 — чистые законы, C5 — геометрию и интеграцию, C6 — внешние сопротивления. Референсы текущего поведения: R07, R09, R10, R16.

### Task C1: физический snapshot и абсолютная скорость точки

**Files:** Create `src/bike_sim/sim/ride/contact_state.py`, `src/bike_sim/sim/ride/wheel_kinematics.py`; Modify `src/bike_sim/sim/ride/contacts.py:217-231`; Test `tests/test_contact_kinematics.py`.

**Interfaces:** Produces `ContactPatch(point_m,normal,normal_load_n,tangent_force_n,slip_mps)` и `point_velocity(v_center,omega_world,offset)`. `WheelContactSnapshot` содержит timestamp, patches, geometric_contact и вычисляемые агрегаты; frozen snapshot не хранит references на изменяемые буферы MuJoCo.

- [ ] **Step 1: failing tests знаков и разложения силы.**

```python
import numpy as np
import pytest
from bike_sim.sim.ride.wheel_kinematics import point_velocity
from bike_sim.sim.ride.contact_state import ContactPatch

def test_no_slip_uses_world_wheel_velocity():
    v = point_velocity(np.array([3.5,0.,0.]), np.array([0.,10.,0.]), np.array([0.,0.,-0.35]))
    np.testing.assert_allclose(v, np.zeros(3), atol=1e-12)

def test_slope_keeps_normal_and_vertical_force_distinct():
    n = np.array([-0.5,0.,np.sqrt(3)/2])
    patch = ContactPatch(np.zeros(3), n, 100.0, 20.0, 0.0)
    assert patch.normal_load_n == 100.0
    assert patch.world_force_n[2] == pytest.approx(100*np.sqrt(3)/2+10)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_contact_kinematics.py -q`; ожидается отсутствие модулей.
- [ ] **Step 3: реализовать векторные контракты.**

```python
from dataclasses import dataclass
import numpy as np

def point_velocity(v_center, omega_world, offset):
    values = [np.asarray(x, dtype=float) for x in (v_center,omega_world,offset)]
    if any(x.shape != (3,) or not np.isfinite(x).all() for x in values):
        raise ValueError("expected finite 3D vectors")
    return values[0]+np.cross(values[1],values[2])

@dataclass(frozen=True)
class ContactPatch:
    point_m: np.ndarray
    normal: np.ndarray
    normal_load_n: float
    tangent_force_n: float
    slip_mps: float

    def __post_init__(self):
        point = np.array(self.point_m, dtype=float, copy=True)
        normal = np.array(self.normal, dtype=float, copy=True)
        if any(a.shape != (3,) or not np.isfinite(a).all() for a in (point, normal)):
            raise ValueError("contact needs finite 3D vectors")
        if abs(np.linalg.norm(normal)-1.0) > 1e-8 or abs(normal[1]) > 1e-8:
            raise ValueError("expected a unit normal in the X-Z plane")
        scalars = (self.normal_load_n, self.tangent_force_n, self.slip_mps)
        if not all(np.isfinite(v) for v in scalars) or self.normal_load_n < 0:
            raise ValueError("invalid contact load or slip")
        point.setflags(write=False)
        normal.setflags(write=False)
        object.__setattr__(self, "point_m", point)
        object.__setattr__(self, "normal", normal)

    @property
    def tangent(self):
        return np.array([self.normal[2],0.,-self.normal[0]])

    @property
    def world_force_n(self):
        return self.normal_load_n*self.normal+self.tangent_force_n*self.tangent
```

Приведённый конструктор копирует и защищает векторы, проверяет единичность нормали, `normal_y=0` в 2D, конечность скаляров и Fn>=0. `WheelContactSnapshot` определить как frozen dataclass с полями `time_s: float`, `patches: tuple[ContactPatch,...]`, `geometric_contact: bool`; свойства `normal_load_n`, `normal_vertical_n`, `vertical_resultant_n` и `force_world_n` суммируют соответственно Fn, Fn*nz, world_force_n[2] и world_force_n. Пустой tuple возвращает нулевые агрегаты. В __post_init__ требовать finite time_s>=0 и приводить patches к tuple. В native query читать все компоненты `mj_contactForce`, преобразовывать `contact.frame.reshape(3,3).T @ force[:3]` в world, с корректным знаком tracked geom. Не терять вертикальную компоненту касательной силы. Для world wheel velocity использовать `mj_objectVelocity` с world orientation и известным центром объекта; затем пересчитывать скорость к оси/контакту, если объектный центр не совпадает с ними. Relative hinge speed оставить отдельным значением.

- [ ] **Step 4:** `uv run --locked pytest tests/test_contact_kinematics.py tests/test_ride_controllers.py -q`. Отдельно проверить swapped geom1/geom2 и wheel parent, который вращается вместе с колесом при нулевом relative hinge speed.
- [ ] **Step 5:** `git add src/bike_sim/sim/ride/contact_state.py src/bike_sim/sim/ride/wheel_kinematics.py src/bike_sim/sim/ride/contacts.py tests/test_contact_kinematics.py && git commit -m "feat: expose physical contact wrench and wheel kinematics"`.

### Task C2: фильтр grounded по времени, без подмены нагрузки

**Files:** Create `src/bike_sim/sim/ride/contact_filter.py`; Modify `src/bike_sim/sim/ride/contacts.py:234-269`; Test `tests/test_contact_filter_time.py`.

**Interfaces:** Produces `GroundedFilter(hold_s).update(raw_grounded,time_s)->bool`, `reset()`. Не принимает и не возвращает normal load.

- [ ] **Step 1: тест времени и идемпотентного чтения.**

```python
from bike_sim.sim.ride.contact_filter import GroundedFilter

def test_filter_does_not_advance_when_read_twice():
    f = GroundedFilter(0.005)
    assert f.update(True, 0.0)
    assert f.update(False, 0.004)
    assert f.update(False, 0.004)
    assert not f.update(False, 0.006)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_contact_filter_time.py -q`; ожидается отсутствующий модуль.
- [ ] **Step 3: заменить step-count на timestamp.**

```python
from math import isfinite

class GroundedFilter:
    def __init__(self, hold_s):
        if not isfinite(hold_s) or hold_s < 0:
            raise ValueError("hold_s must be finite and nonnegative")
        self.hold_s = hold_s
        self.reset()

    def reset(self):
        self.last_time = None
        self.last_loaded = None
        self.value = False

    def update(self, raw_grounded, time_s):
        if not isfinite(time_s):
            raise ValueError("non-finite timestamp")
        if self.last_time is not None and time_s < self.last_time:
            raise ValueError("timestamp moved backwards; reset required")
        if time_s == self.last_time:
            return self.value
        self.last_time = time_s
        if raw_grounded:
            self.last_loaded = time_s
        self.value = bool(raw_grounded or (self.last_loaded is not None and
                          time_s-self.last_loaded < self.hold_s))
        return self.value
```

Physical rolling resistance и контактные силы получают только raw Fn. Native reference controller может использовать отфильтрованный bool, подписанный `controller_grounded`. Краш catch-plane не проходит через grounded-filter. При reset симуляции reset фильтра обязателен.

- [ ] **Step 4:** `uv run --locked pytest tests/test_contact_filter_time.py -q`. Повторить одну временную последовательность при dt 0.5/0.25/0.125 ms; длительность удержания совпадает в пределах одного шага.
- [ ] **Step 5:** `git add src/bike_sim/sim/ride/contact_filter.py src/bike_sim/sim/ride/contacts.py tests/test_contact_filter_time.py && git commit -m "fix: separate time-based grounded filter from physical loads"`.

### Task C3: нормальная податливость с материальными единицами

**Files:** Create `src/bike_sim/physics/tire.py`; Test `tests/test_tire_radial.py`.

**Interfaces:** Produces `normal_contact(delta,delta_dot,k,c)->tuple[force_n,energy_j]`. Параметры TireSpec: positive `radial_k_n_m`, nonnegative `radial_c_ns_m`, `pressure_pa_gauge`, `provenance`, `valid_load_range_n`. Native solref/solimp хранятся отдельно от этих значений.

- [ ] **Step 1: тест зависимости силы от деформации и нулевой силы вне контакта.**

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

- [ ] **Step 2:** `uv run --locked pytest tests/test_tire_radial.py -q`; ожидается отсутствующий API.
- [ ] **Step 3: реализовать односторонний закон.**

```python
from math import isfinite

def normal_contact(delta, delta_dot, k, c):
    if not all(isfinite(x) for x in (delta,delta_dot,k,c)) or k <= 0 or c < 0:
        raise ValueError("invalid radial contact parameters")
    if delta <= 0:
        return 0.0, 0.0
    return max(0.0,k*delta+c*delta_dot), 0.5*k*delta**2
```

130000/800 используются только как synthetic-стенд в физических единицах, не копируются как интерпретация прежнего solref. Конфигурация calibrated принимает таблицу давления и монотонной force-deflection кривой; интерполяция внутри диапазона, ошибка за его пределами. Для линейной версии проверить 100/300/600/1000 N на одном и том же k. Для native reference тот же стенд измеряет эффективную характеристику; несоответствие материальной кривой сохраняется в отчёте, а не подавляется.

- [ ] **Step 4:** `uv run --locked pytest tests/test_tire_radial.py -q`. Проверить unload с отрицательной delta_dot, отсутствие растяжения и равенство энергии интегралу силы пружины.
- [ ] **Step 5:** `git add src/bike_sim/physics/tire.py tests/test_tire_radial.py && git commit -m "feat: add unilateral radial tire compliance in SI units"`.

### Task C4: касательная податливость, stick/slip и энергетическая проверка

**Files:** Modify `src/bike_sim/physics/tire.py`; Test `tests/test_tire_brush.py`.

**Interfaces:** Produces `brush_step(xi,u,v_roll,Fn,k,mu,length,dt)->tuple[xi_new,Fx,dissipation_step_j]`. Последняя величина — неотрицательный дискретный остаток работы элемента при принятой квадратуре, включая численную диссипацию; не выдавать её целиком за измеренный гистерезис резины.

- [ ] **Step 1: тест предела силы и дискретной пассивности.**

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

- [ ] **Step 2:** `uv run --locked pytest tests/test_tire_brush.py -q`; ожидается отсутствующий brush_step.
- [ ] **Step 3: реализовать пассивный return-map.**

```python
from math import isfinite

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

Проверить случай уменьшения Fn при ненулевом xi и разворот u. Состояние хранит tangent frame; при смене нормали переносить вектор shear проекцией в новую касательную и учитывать уменьшение его энергии. При переходе на несвязанную контактную область сбросить shear с отдельным loss event. Не переносить xi между front/rear wheel.

- [ ] **Step 4:** `uv run --locked pytest tests/test_tire_brush.py tests/test_tire_radial.py -q`. Перебрать сетку скоростей, нагрузок и начального xi; выполнить динамический launch без лимитера motor torque, подтвердить возможную пробуксовку.
- [ ] **Step 5:** `git add src/bike_sim/physics/tire.py tests/test_tire_brush.py && git commit -m "feat: add passive tangential tire state and friction saturation"`.

### Task C5: контакт с профилем и единственный силовой backend

**Files:** Create `src/bike_sim/terrain/contact_profile.py`, `src/bike_sim/sim/ride/tire_forces.py`; Modify `src/bike_sim/mujoco/builder.py`, `src/bike_sim/mujoco/steering_fork.py:114-139`, `src/bike_sim/mujoco/drivetrain.py` (`build_rear_wheel`), `src/bike_sim/sim/ride_sim.py:192-223`; Test `tests/test_profile_contact.py`, `tests/test_tire_force_path.py`.

**Interfaces:** Produces `ProfileContact(point,normal,delta,segment_id,multi_support)` и `closest_profile_contact(center_xz,radius,vertices_xz,previous_segment=None)->ProfileContact`; `TireForceApplier.compute_qfrc(model,data,dt)->np.ndarray` и текущий contact snapshot.

- [ ] **Step 1: геометрический тест дробления поверхности.**

```python
import numpy as np
import pytest
from bike_sim.terrain.contact_profile import closest_profile_contact

def test_flat_contact_does_not_depend_on_segment_count():
    center = np.array([0.1,0.34])
    coarse = np.array([[-1.,0.],[1.,0.]])
    fine = np.column_stack([np.linspace(-1,1,101),np.zeros(101)])
    a = closest_profile_contact(center,0.35,coarse)
    b = closest_profile_contact(center,0.35,fine)
    assert a.delta == pytest.approx(0.01)
    assert b.delta == pytest.approx(a.delta,abs=1e-12)
    np.testing.assert_allclose(a.normal,[0.,1.],atol=1e-12)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_profile_contact.py -q`; ожидается отсутствующий модуль.
- [ ] **Step 3: реализовать геометрию и mapping силы.**

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class ProfileContact:
    point: np.ndarray
    normal: np.ndarray
    delta: float
    segment_id: int
    multi_support: bool

def closest_profile_contact(center_xz,radius,vertices_xz,previous_segment=None):
    c = np.asarray(center_xz,dtype=float)
    vertices = np.asarray(vertices_xz,dtype=float)
    if c.shape != (2,) or vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 2:
        raise ValueError("invalid planar profile")
    if not np.isfinite(vertices).all() or not np.isfinite(c).all() or not np.isfinite(radius) or radius <= 0:
        raise ValueError("invalid contact geometry")
    a, b = vertices[:-1], vertices[1:]
    d = b-a
    length2 = np.sum(d*d,axis=1)
    if np.any(length2 <= 0) or np.any(np.diff(vertices[:,0]) <= 0):
        raise ValueError("profile x must increase strictly")
    t = np.clip(np.sum((c-a)*d,axis=1)/length2,0.,1.)
    points = a+t[:,None]*d
    distances = np.linalg.norm(c-points,axis=1)
    i = int(np.argmin(distances))
    if previous_segment is not None and 0 <= previous_segment < len(distances):
        if abs(distances[previous_segment]-distances[i]) <= 1e-12:
            i = previous_segment
    if distances[i] <= 1e-12:
        raise ValueError("wheel center reached the surface")
    normals = (c-points)/np.maximum(distances[:,None],1e-12)
    distinct = np.sum(normals*normals[i],axis=1) < np.cos(np.deg2rad(20.0))
    multi = bool(np.any((distances < radius) & distinct))
    return ProfileContact(points[i].copy(),normals[i].copy(),
                          float(radius-distances[i]),i,multi)
```

Рабочий adapter предварительно выбирает локальные segments по X через `searchsorted`, но сохраняет глобальный segment_id; крайние кандидаты включать. Тесты сначала используют полный массив как независимый эталон локального поиска. Если центр оказывается внутри твёрдого грунта или профиль выходит за supported envelope, останавливать сценарий с причиной, не отражать колесо произвольной нормалью.

В physics step world contact point = `[point[0],0,point[1]]`, world normal = `[normal[0],0,normal[1]]`. Сила из C3–C4 применяется один раз:

```python
qfrc = np.zeros(model.nv)
force_world = Fn*normal_world + Fx*tangent_world
mujoco.mj_applyFT(model,data,force_world,np.zeros(3),point_world,wheel_body_id,qfrc)
```

Не добавлять отдельно `-Fx*R` в wheel hinge: mapping уже учитывает плечо. При `compliant_2d` все wheel–terrain geoms отключены для native contacts; crash-geoms рамы не отключаются. Для native_reference tire writer инертен, нагрузки поступают только из `mj_contactForce`. Проверка `test_tire_force_path.py` должна сравнить `qfrc@qvel` с `force_world@point_velocity` и убедиться в нулевом числе native wheel–terrain rows для compliant backend.

- [ ] **Step 4:** `uv run --locked pytest tests/test_profile_contact.py tests/test_tire_force_path.py -q`. Добавить inclined plane, convex edge, wide gap, local/full search equality и multi_support-flag. Выполнить A20–A21 на простом стенде до дорожных метрик.
- [ ] **Step 5:** `git add src/bike_sim/terrain/contact_profile.py src/bike_sim/sim/ride/tire_forces.py src/bike_sim/mujoco src/bike_sim/sim/ride_sim.py tests/test_profile_contact.py tests/test_tire_force_path.py && git commit -m "feat: integrate explicit planar tire contact without native double counting"`.

### Task C6: rolling resistance как внешнее взаимодействие и аэродинамика

**Files:** Create `src/bike_sim/physics/external_resistance.py`; Modify `src/bike_sim/sim/ride/resistance.py:128-152`; Test `tests/test_external_resistance.py`.

**Interfaces:** Produces `rolling_moment(crr,Fn,radius,omega_abs,taper)->float`, `drag_force(velocity_relative,rho,cda)->np.ndarray`. Внутреннее bearing damping остаётся separate contribution на relative hinge.

- [ ] **Step 1: тест нормальной нагрузки и пассивности воздуха.**

```python
import numpy as np
import pytest
from bike_sim.physics.external_resistance import rolling_moment, drag_force

def test_rolling_uses_normal_load_not_vertical_projection():
    assert rolling_moment(0.015,500.0,0.35,10.0,0.2) == pytest.approx(-2.625)
    assert rolling_moment(0.015,0.0,0.35,10.0,0.2) == 0.0

def test_still_air_drag_is_passive():
    v = np.array([7.,0.,1.])
    force = drag_force(v,1.2,0.5)
    assert force@v < 0
    np.testing.assert_array_equal(drag_force(np.zeros(3),1.2,0.5),np.zeros(3))
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_external_resistance.py -q`.
- [ ] **Step 3: законы и правильное тело реакции.**

```python
import math
import numpy as np

def rolling_moment(crr,Fn,radius,omega_abs,taper):
    if not all(math.isfinite(x) for x in (crr,Fn,radius,omega_abs,taper)):
        raise ValueError("non-finite rolling input")
    if crr < 0 or Fn < 0 or radius <= 0 or taper <= 0:
        raise ValueError("invalid rolling parameters")
    return -crr*Fn*radius*math.tanh(omega_abs/taper)

def drag_force(velocity_relative,rho,cda):
    v = np.asarray(velocity_relative,dtype=float)
    if v.shape != (3,) or not np.isfinite(v).all() or not math.isfinite(rho) or not math.isfinite(cda):
        raise ValueError("invalid aerodynamic input")
    if rho < 0 or cda < 0:
        raise ValueError("negative aerodynamic coefficient")
    return -0.5*rho*cda*np.linalg.norm(v)*v
```

Rolling torque применить world-вектором вдоль фактической оси колеса через `mj_applyFT`; не присваивать только wheel hinge. Drag применить в заданной аэродинамической точке тела. Записать работу этих воздействий как внешнее взаимодействие с неподвижной дорогой/воздухом. Для ветра использовать относительную скорость, а world work сохранять со знаком; ветер может отдавать энергию.

Отдельно проверить virtual work world torque против абсолютной wheel angular velocity. На наклоне использовать Fn, а не normal_vertical_n. Сохранить прежнюю логику rolling only в legacy. Сопоставить плоский выбег с суммой bearing, tire и Crr losses, чтобы не откалибровать одну потерю дважды.

- [ ] **Step 4:** `uv run --locked pytest tests/test_external_resistance.py tests/test_tire_force_path.py -q`.
- [ ] **Step 5:** `git add src/bike_sim/physics/external_resistance.py src/bike_sim/sim/ride/resistance.py tests/test_external_resistance.py && git commit -m "fix: apply road resistance externally and add aerodynamic force"`.

## Review gate C

Проверить, что ни один controller snapshot не становится физической нагрузкой. У compliant backend нет native двойного контакта, у native backend нет дополнительной шины поверх solver. Контактная геометрия читает тот же дискретный профиль. Multi-support не скрыт: результаты таких интервалов не получают статус V2 без отдельной проверки.