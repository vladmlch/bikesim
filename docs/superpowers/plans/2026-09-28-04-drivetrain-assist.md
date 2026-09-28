# 04 — трансмиссия, педалирование и ассист Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Заменить подмену центрального привода моментом на заднем колесе наблюдаемой механической цепью передачи энергии.

**Architecture:** Шатуны, кассета и колесо имеют отдельные состояния; цепь создаёт обобщённые силы через производную удлинения, freehub передаёт односторонний момент. Источники человеческой и моторной работы отделены от потерь и батареи. Тормоз — статическое ограниченное трение, не taper-мотор.

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

Требуются A1–A3, B1–B2, C1 и C6; дорожные испытания D4–D6 требуют C3–C5 либо явно выбранного native_reference. Референсы текущего поведения: R01, R13–R14 и R16. До D1 в коде нет динамических crank/cassette; не выводить наличие трансмиссии из имён визуальных геомов.

### Task D1: отдельные crank, pedals и cassette без удвоения масс

**Files:** Modify `src/bike_sim/mujoco/drivetrain.py:30-103` и `build_rear_wheel`, `src/bike_sim/mujoco/actuators.py`, `src/bike_sim/mujoco/frame.py:240-253`, `src/bike_sim/sim/ride_sim.py:150-154`; Create `src/bike_sim/physics/chain.py` с конфигурацией; Test `tests/test_drivetrain_topology.py`.

**Interfaces:** Produces MJCF body/joint names `crank/crank_spin`, `cassette/cassette_spin`, `pedal_front/pedal_front_spin`, `pedal_rear/pedal_rear_spin`; sites `site_pedal_front`, `site_pedal_rear`; actuators `mid_drive`, `human_crank` только в соответствующих режимах. `DrivetrainSpecs(front_teeth=34,rear_teeth=24,chain_pitch_m=0.0127)` задаёт средние радиусы.

- [ ] **Step 1: failing test скомпилированной топологии.**

```python
import mujoco
import pytest
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig

def test_physical_effort_has_crank_and_cassette_dofs_without_extra_mass():
    cfg = SimulationPhysicsConfig(physics_mode="physical",drive_mode="crank_effort")
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="ride",rider="none",physics_config=cfg))
    for name in ("crank_spin","cassette_spin","rear_wheel_spin"):
        assert mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name) >= 0
    assert model.body_mass.sum() == pytest.approx(BikeMassSpecs().total_bike_mass,abs=1e-8)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_drivetrain_topology.py -q`; ожидается отсутствие динамических joints.
- [ ] **Step 3: перенести, а не скопировать физические части.**

```python
from dataclasses import dataclass
from math import isfinite, pi

@dataclass(frozen=True)
class DrivetrainSpecs:
    front_teeth: int = 34
    rear_teeth: int = 24
    chain_pitch_m: float = 0.0127

    def __post_init__(self):
        if any(type(z) is not int or z < 3 for z in (self.front_teeth, self.rear_teeth)):
            raise ValueError("sprockets need integer tooth counts of at least three")
        if not isfinite(self.chain_pitch_m) or self.chain_pitch_m <= 0:
            raise ValueError("invalid chain pitch")

    @property
    def front_radius_m(self):
        return self.chain_pitch_m*self.front_teeth/(2*pi)

    @property
    def rear_radius_m(self):
        return self.chain_pitch_m*self.rear_teeth/(2*pi)
```

Ядро новой MJCF-структуры:

```python
crank = ET.SubElement(frame,"body",name="crank",pos="0 0 0")
ET.SubElement(crank,"joint",name="crank_spin",type="hinge",axis="0 1 0",damping="0")
for side, sign in (("front",1.0),("rear",-1.0)):
    pedal = ET.SubElement(crank,"body",name="pedal_"+side,
                          pos=f"{sign*crank_length_m:.17g} 0 0")
    ET.SubElement(pedal,"joint",name="pedal_"+side+"_spin",type="hinge",axis="0 1 0",damping="0")
    ET.SubElement(pedal,"site",name="site_pedal_"+side,pos="0 0 0")
```

Существующие геомы arms/spindle переносить на crank, платформы — на pedal bodies; мотор и BB shell оставить на frame. Каждое подвижное тело получает положительную массу и допустимый тензор из B2. Кассету перенести из wheel в отдельный child seatstay на той же оси. Вычитать её массу и инерцию из wheel через компонентную модель B1–B2. Добавить явные межтелесные collision exclusions для crank/pedal/cassette и велосипеда; rider–pedal взаимодействие обеспечит E3.

`mid_drive` действует на `crank_spin`, его ctrlrange только неотрицательный. `human_crank` существует/работает только в crank_effort. В articulated_effort его вклад строго нулевой/actuator отсутствует. `rear_drive` создаётся только для legacy или явно выбранного ideal_speed_control. Убрать безусловное разрешение handle `rear_drive` из конструктора физического effort-режима.

- [ ] **Step 4:** `uv run --locked pytest tests/test_drivetrain_topology.py tests/test_compiled_mass_contract.py tests/test_wheel_inertia_contract.py -q`. Проверить перемещение pedal sites при четверти оборота crank и постоянство полного бюджета массы.
- [ ] **Step 5:** `git add src/bike_sim/mujoco src/bike_sim/physics/chain.py src/bike_sim/sim/ride_sim.py tests/test_drivetrain_topology.py && git commit -m "feat: add dynamic crank pedals and cassette bodies"`.

### Task D2: цепная координата и силы через виртуальную работу

**Files:** Modify `src/bike_sim/physics/chain.py`; Create `src/bike_sim/sim/ride/drivetrain_forces.py`; Test `tests/test_chain_virtual_work.py`.

**Interfaces:** Produces `chain_extension(cf,cr,rf,rr,theta_f,theta_r,reference,*,up_xz=None,psi_reference=None)->float`; `chain_jacobian(q,evaluate,epsilon=1e-7)->np.ndarray`. `evaluate(q)` — чистое вычисление extension при фиксированной ветви развёрнутого угла. Runtime applier добавляет named `chain` в A2.

- [ ] **Step 1: failing tests геометрии и общей вращательной инвариантности.**

```python
import numpy as np
import pytest
from bike_sim.physics.chain import chain_extension

def test_equal_sprockets_reduce_to_straight_length_and_phase():
    e = chain_extension(np.array([0.,0.]),np.array([-0.45,0.]),0.05,0.05,0.2,0.0,0.45)
    assert e == pytest.approx(0.01)

def test_rigid_rotation_does_not_stretch_chain():
    cf, cr = np.array([0.,0.]),np.array([-0.45,0.])
    reference = chain_extension(cf,cr,0.07,0.04,0.0,0.0,0.0)
    angle = 0.2
    R = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    e = chain_extension(R@cf,R@cr,0.07,0.04,-angle,-angle,reference,up_xz=R@np.array([0.,1.]))
    assert e == pytest.approx(0.0,abs=1e-12)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_chain_virtual_work.py -q`.
- [ ] **Step 3: реализовать касательную и force mapping.**

```python
from math import atan2, cos, sin, sqrt
import numpy as np

def chain_extension(cf,cr,rf,rr,theta_f,theta_r,reference,*,up_xz=None,psi_reference=None):
    cf, cr = np.asarray(cf,dtype=float),np.asarray(cr,dtype=float)
    up = np.array([0.,1.]) if up_xz is None else np.asarray(up_xz,dtype=float)
    if cf.shape != (2,) or cr.shape != (2,) or up.shape != (2,):
        raise ValueError("chain centers and up vector must be planar")
    values = np.r_[cf,cr,up,rf,rr,theta_f,theta_r,reference]
    if not np.isfinite(values).all() or rf <= 0 or rr <= 0 or np.linalg.norm(up) == 0:
        raise ValueError("invalid chain input")
    delta = cr-cf
    distance = float(np.linalg.norm(delta))
    difference = rf-rr
    if distance <= abs(difference):
        raise ValueError("sprockets have no valid external tangent")
    direction = delta/distance
    perpendicular = np.array([direction[1],-direction[0]])
    ratio = difference/distance
    n1 = ratio*direction+sqrt(1-ratio*ratio)*perpendicular
    n2 = ratio*direction-sqrt(1-ratio*ratio)*perpendicular
    normal = n1 if n1@up >= n2@up else n2
    psi = atan2(normal[1],normal[0])
    if psi_reference is not None:
        if not np.isfinite(psi_reference):
            raise ValueError("invalid angular unwrap reference")
        psi = psi_reference+atan2(sin(psi-psi_reference),cos(psi-psi_reference))
    length = sqrt(distance*distance-difference*difference)+difference*psi
    return length+rf*theta_f-rr*theta_r-reference

def chain_jacobian(q,evaluate,epsilon=1e-7):
    q = np.asarray(q,dtype=float)
    if q.ndim != 1 or not np.isfinite(q).all() or not np.isfinite(epsilon) or epsilon <= 0:
        raise ValueError("invalid generalized coordinates or difference step")
    result = np.zeros_like(q)
    for i in range(len(q)):
        step = np.zeros_like(q)
        step[i] = epsilon
        plus, minus = float(evaluate(q+step)), float(evaluate(q-step))
        if not np.isfinite(plus) or not np.isfinite(minus):
            raise ValueError("non-finite chain extension")
        result[i] = (plus-minus)/(2*epsilon)
    return result
```

Ядро `chain_jacobian` проверяет координаты, шаг и результат evaluator; production adapter дополнительно требует `nq==nv` для поддержанной плоской модели. Начальные коэффициенты тестового профиля задать явно: `k_chain=200000.0 N/m`, `c_chain=10.0 N*s/m`, provenance=synthetic. Они допускаются только после теста устойчивости и сходимости; не выдавать их за измеренные свойства конкретной цепи. Вычисления возмущённых поз проводить на отдельном `MjData`, не мутировать live state и не обновлять управляющие фильтры. Freeze psi_reference на весь расчёт J; обновлять unwrap state ровно один раз после оценки текущего состояния.

В applier:

```python
e_dot = float(J@data.qvel)
T = max(0.0,k_chain*e+c_chain*e_dot) if e > 0.0 else 0.0
qfrc_chain = -T*J
chain_energy_j = 0.5*k_chain*max(e,0.0)**2
```

Извлекать theta_f/theta_r как абсолютные развёрнутые углы соответствующих тел, не только `qpos[crank_spin]` и `qpos[cassette_spin]`. Повороты рамы и заднего звена входят в зависимость. Расчёт по всем q автоматически переносит chain pull в подвеску. Не добавлять после этого отдельно torque=T*r: он уже содержится в J. Сначала использовать finite-difference J как проверяемую реализацию; аналитический J — отдельная оптимизация с сопоставлением на сетке поз.

- [ ] **Step 4:** `uv run --locked pytest tests/test_chain_virtual_work.py -q`. Добавить `Q@v == -T*(J@v)`, finite rigid translation, незамкнутую/натянутую цепь, энергетический тест и изменение реакции shock при блокированном crank. На колесном стенде проверить отношение по зубьям при выключенных потерях.
- [ ] **Step 5:** `git add src/bike_sim/physics/chain.py src/bike_sim/sim/ride/drivetrain_forces.py tests/test_chain_virtual_work.py && git commit -m "feat: transmit chain load through full kinematic Jacobian"`.

### Task D3: пассивный freehub между кассетой и колесом

**Files:** Create `src/bike_sim/physics/freehub.py`; Modify `src/bike_sim/sim/ride/drivetrain_forces.py`; Test `tests/test_freehub_physics.py`.

**Interfaces:** Produces `Freehub(stiffness_nm_rad,damping_nms_rad).update(phi_c,phi_w,omega_c,omega_w)->float`, `reset()`, `energy_j`. Углы и скорости двух siblings измеряются в одной системе относительно их общего несущего тела.

- [ ] **Step 1: failing test свободного хода и зацепления.**

```python
import pytest
from bike_sim.physics.freehub import Freehub

def test_wheel_can_overrun_then_cassette_can_engage():
    hub = Freehub(1000.0,0.5)
    assert hub.update(0.0,0.0,0.0,10.0) == 0.0
    assert hub.update(0.0,1.0,0.0,10.0) == 0.0
    assert hub.update(0.02,1.0,2.0,0.0) == pytest.approx(21.0)
    assert hub.energy_j == pytest.approx(0.2)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_freehub_physics.py -q`.
- [ ] **Step 3: односторонняя угловая связь.**

```python
from math import isfinite

class Freehub:
    def __init__(self,stiffness_nm_rad,damping_nms_rad):
        if not isfinite(stiffness_nm_rad) or not isfinite(damping_nms_rad):
            raise ValueError("non-finite freehub parameter")
        if stiffness_nm_rad <= 0 or damping_nms_rad < 0:
            raise ValueError("invalid freehub parameter")
        self.k = stiffness_nm_rad
        self.c = damping_nms_rad
        self.reset()

    def reset(self):
        self.boundary = None
        self.energy_j = 0.0

    def update(self,phi_c,phi_w,omega_c,omega_w):
        if not all(isfinite(x) for x in (phi_c,phi_w,omega_c,omega_w)):
            raise ValueError("non-finite freehub state")
        relative = phi_c-phi_w
        if self.boundary is None or relative < self.boundary:
            self.boundary = relative
        deflection = max(relative-self.boundary,0.0)
        self.energy_j = 0.5*self.k*deflection**2
        return max(0.0,self.k*deflection+self.c*(omega_c-omega_w))
```

Нулевой backlash — explicit synthetic assumption. Applier добавляет +T в wheel spin и -T в cassette spin; поскольку parent общий, реакции на него сокращаются. Для другой топологии использовать пару world moments. Показатель мощности муфты `T*(omega_w-omega_c)` плюс изменение её энергии должен быть неположительным с учётом дискретной ошибки. Вызов update идёт один раз на шаг; telemetry только читает.

- [ ] **Step 4:** `uv run --locked pytest tests/test_freehub_physics.py tests/test_chain_virtual_work.py -q`. Проверить обратное педалирование и отсутствие forced qvel resets; совместно с chain stiffness выполнить dt-refinement до release.
- [ ] **Step 5:** `git add src/bike_sim/physics/freehub.py src/bike_sim/sim/ride/drivetrain_forces.py tests/test_freehub_physics.py && git commit -m "feat: add passive one-way cassette wheel coupling"`.

### Task D4: человеческое усилие и контроллер motor assist

**Files:** Create `src/bike_sim/physics/pedaling.py`, `src/bike_sim/physics/motor.py`; Modify `src/bike_sim/sim/ride/drivetrain_forces.py`, `src/bike_sim/sim/ride_sim.py`; Test `tests/test_motor_assist.py`.

**Interfaces:** Produces `human_crank_torque(mean_nm,phase_rad,ripple=0.35)->float`; `AssistController.step(human_nm,cadence_rpm,speed_mps,braking,dt)->float`, `reset()`. Поля controller задаются keyword-only, defaults точно из DRIVE-05.

- [ ] **Step 1: тесты brake priority и мгновенного power cap.**

```python
from math import pi
from bike_sim.physics.motor import AssistController

def test_brake_cancels_motor_without_filter_tail():
    motor = AssistController()
    for _ in range(200):
        torque = motor.step(40.0,60.0,2.0,False,0.005)
    assert torque > 0
    assert motor.step(40.0,60.0,2.0,True,0.005) == 0.0

def test_power_cap_is_applied_after_filter_even_after_cadence_jump():
    motor = AssistController()
    for _ in range(200):
        motor.step(100.0,20.0,2.0,False,0.005)
    torque = motor.step(100.0,500.0,2.0,False,0.005)
    assert torque*(500*2*pi/60) <= 500.0+1e-9
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_motor_assist.py -q`.
- [ ] **Step 3: ограниченная динамика помощника, не PI по скорости рамы.**

```python
from math import cos, isfinite

def human_crank_torque(mean_nm,phase_rad,ripple=0.35):
    if not all(isfinite(x) for x in (mean_nm,phase_rad,ripple)) or not 0 <= ripple < 1:
        raise ValueError("invalid pedal torque profile")
    return mean_nm*(1-ripple*cos(2*phase_rad))
```

```python
from math import expm1, isfinite, pi

class AssistController:
    def __init__(self,*,gain=2.0,max_torque=80.0,max_power=500.0,tau=0.05,
                 slew=400.0,stop_delay=0.1,on_rpm=10.0,off_rpm=5.0,
                 cutoff_mps=25/3.6,taper_width_mps=2/3.6):
        values = (gain,max_torque,max_power,tau,slew,stop_delay,on_rpm,off_rpm,cutoff_mps,taper_width_mps)
        if not all(isfinite(x) for x in values) or min(values) < 0:
            raise ValueError("invalid assist parameters")
        if tau == 0 or slew == 0 or taper_width_mps == 0 or on_rpm <= off_rpm:
            raise ValueError("invalid assist time constant or thresholds")
        self.gain,self.max_torque,self.max_power = gain,max_torque,max_power
        self.tau,self.slew,self.stop_delay = tau,slew,stop_delay
        self.on_rpm,self.off_rpm = on_rpm,off_rpm
        self.cutoff,self.width = cutoff_mps,taper_width_mps
        self.reset()

    def reset(self):
        self.torque = 0.0
        self.pedaling = False
        self.age = float("inf")

    def step(self,human_nm,cadence_rpm,speed_mps,braking,dt):
        if not all(isfinite(x) for x in (human_nm,cadence_rpm,speed_mps,dt)) or dt <= 0:
            raise ValueError("invalid assist sample")
        if braking or cadence_rpm < 0:
            self.reset()
            return 0.0
        threshold = self.off_rpm if self.pedaling else self.on_rpm
        self.pedaling = human_nm > 0 and cadence_rpm >= threshold
        self.age = 0.0 if self.pedaling else self.age+dt
        if self.age >= self.stop_delay and not self.pedaling:
            self.torque = 0.0
            return 0.0
        omega = abs(cadence_rpm)*2*pi/60
        ceiling = self.max_torque if omega <= 1e-12 else min(self.max_torque,self.max_power/omega)
        taper = max(0.0,min(1.0,(self.cutoff-abs(speed_mps))/self.width))
        target = min(self.gain*max(human_nm,0.0),ceiling)*taper
        candidate = self.torque+(-expm1(-dt/self.tau))*(target-self.torque)
        candidate = max(self.torque-self.slew*dt,min(self.torque+self.slew*dt,candidate))
        self.torque = max(0.0,min(candidate,ceiling*taper))
        return self.torque
```

Валидацию таблицы `T_max(cadence)` добавить в конфигурацию как монотонную по X piecewise-linear таблицу; приведённый код использует её плоский synthetic случай 80 Nm. Обнуление motor при brake выше приоритета smooth slew. Speed taper применяется к target и ceiling, а не многократно к уже отфильтрованному состоянию.

В crank_effort human actuator действует на crank; в articulated_effort sensor human_nm приходит из фактических pedal forces, не из второго torque source. Remove временный constructor guard crank_effort после интеграции D1–D4. Контакт rear wheel не выключает motor автоматически: разгруженное колесо может раскручиваться согласно цепи и инерции.

- [ ] **Step 4:** `uv run --locked pytest tests/test_motor_assist.py tests/test_drivetrain_topology.py -q`. Добавить `human<=0`, stop delay, negative cadence, cutoff continuity и среднее human torque за фазовый цикл.
- [ ] **Step 5:** `git add src/bike_sim/physics/pedaling.py src/bike_sim/physics/motor.py src/bike_sim/sim/ride/drivetrain_forces.py src/bike_sim/sim/ride_sim.py tests/test_motor_assist.py && git commit -m "feat: add cadence torque based pedal assist with hard limits"`.

### Task D5: электрическая мощность и ограничение по запасу батареи

**Files:** Create `src/bike_sim/physics/battery.py`; Modify `src/bike_sim/physics/motor.py`, `src/bike_sim/sim/ride/drivetrain_forces.py`; Test `tests/test_motor_energy.py`.

**Interfaces:** Produces `Battery(energy_j).draw(requested_power_w,dt)->delivered_power_w`; `motor_electrical_power(torque,omega,a,b,idle,enabled)->float`; `limit_torque_by_energy(request,omega,a,b,idle,budget_w)->float`. Поля потерь — synthetic до измерений.

- [ ] **Step 1: failing tests расхода и потерь при нулевой механической мощности.**

```python
import pytest
from bike_sim.physics.battery import Battery, motor_electrical_power

def test_battery_cannot_deliver_more_than_remaining_energy():
    battery = Battery(10.0)
    assert battery.draw(100.0,0.2) == pytest.approx(50.0)
    assert battery.energy_j == 0.0

def test_stall_torque_still_has_electrical_losses():
    assert motor_electrical_power(20.0,0.0,0.02,0.0,5.0,True) == pytest.approx(13.0)
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_motor_energy.py -q`.
- [ ] **Step 3: реализовать учёт и ограничитель ДО приложения torque.**

```python
from math import isfinite, sqrt

class Battery:
    def __init__(self,energy_j):
        if not isfinite(energy_j) or energy_j < 0:
            raise ValueError("invalid stored energy")
        self.energy_j = energy_j

    def draw(self,requested_power_w,dt):
        if not isfinite(requested_power_w) or not isfinite(dt) or requested_power_w < 0 or dt <= 0:
            raise ValueError("invalid battery demand")
        delivered_j = min(self.energy_j,requested_power_w*dt)
        self.energy_j -= delivered_j
        return delivered_j/dt

def motor_electrical_power(torque,omega,a,b,idle,enabled):
    if not all(isfinite(x) for x in (torque,omega,a,b,idle)) or min(a,b,idle) < 0:
        raise ValueError("invalid motor loss input")
    if not enabled:
        return 0.0
    return max(torque*omega,0.0)+a*torque**2+b*omega**2+idle

def limit_torque_by_energy(request,omega,a,b,idle,budget_w):
    if not all(isfinite(x) for x in (request,omega,a,b,idle,budget_w)) or min(request,a,b,idle,budget_w) < 0:
        raise ValueError("invalid energy-limited torque input")
    available = budget_w-(b*omega**2+idle)
    if available <= 0:
        return 0.0
    w = max(omega,0.0)
    if a > 0:
        cap = 2*available/(w+sqrt(w*w+4*a*available))
    elif w > 0:
        cap = available/w
    else:
        cap = request
    return min(request,cap)
```

На шаге сначала получить assist request, затем ограничить его при `budget_w=battery.energy_j/dt`, только потом приложить момент и записать фактический P_elec. Если даже idle loss не помещается в остаток, мотор выключается: остаток энергии ненулевой, статус `energy_limited`, а не ложное `empty`. Для точного расходования последнего остатка можно разбить последний подшаг в момент отключения; это отдельная проверенная ветвь, не обязательное линейное масштабирование torque.

Не вызывать Battery.draw с прежним P_request после ограничения torque. Не применять коэффициент КПД второй раз к моменту, заданному на выходном валу. Механические потери передачи задаются отдельным пассивным моментом/силой и отдельной работой; для ratio-тестов они нулевые.

- [ ] **Step 4:** `uv run --locked pytest tests/test_motor_energy.py tests/test_motor_assist.py -q`. Проверить, что фактически запрошенная после ограничения мощность не больше budget, что сохранены единицы J/Wh и нет отрицательной энергии.
- [ ] **Step 5:** `git add src/bike_sim/physics/battery.py src/bike_sim/physics/motor.py src/bike_sim/sim/ride/drivetrain_forces.py tests/test_motor_energy.py && git commit -m "feat: account for motor losses and energy limited torque"`.

### Task D6: статический тормоз и правильная фактическая работа

**Files:** Modify `src/bike_sim/sim/ride/braking.py`, `src/bike_sim/mujoco/steering_fork.py` (wheel joint), `src/bike_sim/mujoco/drivetrain.py` (wheel joint), `src/bike_sim/sim/ride_sim.py`; Test `tests/test_static_brake.py`.

**Interfaces:** Produces `StaticBrakeApplier.apply(model,data,front_demand,rear_demand)->None`. Physical использует `dof_frictionloss`; legacy сохраняет старый `BrakeController.compute`.

- [ ] **Step 1: сначала одноосный стенд удержания.**

```python
import mujoco

def test_frictionloss_holds_against_subthreshold_torque():
    model = mujoco.MjModel.from_xml_string('''<mujoco><option timestep="0.0005" gravity="0 0 0"/>
    <worldbody><body><joint type="hinge" axis="0 1 0" frictionloss="20"
    solreffriction="0.005 1" solimpfriction="0.9999 0.9999 0.001 0.5 2"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qfrc_applied[0] = 10.0
    for _ in range(2000):
        mujoco.mj_step(model,data)
    assert abs(data.qpos[0]) < 1e-4
```

Добавить integration test `StaticBrakeApplier` на physical model: demand=1 меняет обе `dof_frictionloss` на ceiling, demand=0 снимает их, никакой дополнительный taper control не включается.

- [ ] **Step 2:** `uv run --locked pytest tests/test_static_brake.py -q`. Сам engine-стенд проверяет предпосылку; integration test до нового adapter должен краснеть.
- [ ] **Step 3: в adapter разрешить DOF-адреса и применять предел, а не постоянный момент.**

```python
from math import isfinite

class StaticBrakeApplier:
    def __init__(self,front_dofadr,rear_dofadr,ceiling_nm=200.0):
        if not isfinite(ceiling_nm) or ceiling_nm < 0:
            raise ValueError("invalid brake torque ceiling")
        self.front = front_dofadr
        self.rear = rear_dofadr
        self.ceiling = ceiling_nm

    def apply(self,model,data,front_demand,rear_demand):
        for dof,demand in ((self.front,front_demand),(self.rear,rear_demand)):
            if not isfinite(demand):
                raise ValueError("non-finite brake demand")
            model.dof_frictionloss[dof] = self.ceiling*max(0.0,min(1.0,demand))
```

DOF-адреса разрешать по именам joints, не полагаться на порядок индексов после добавления crank/rider. Параметры solver frictionloss из стенда — начальная численная конфигурация, не свойства гидравлического тормоза. Проверить A15 и refinement на полной системе.

Фактическую силу ограничения извлекать из friction-DOF rows `efc_type` и соответствующего Jacobian mapping после solve; не использовать весь `qfrc_constraint` на wheel DOF как «тормоз», потому что там присутствуют контактные силы. Проверить идентификацию rows на одноосном стенде в lock-версии API. Brake power = actual brake torque * relative wheel speed. Любой brake demand отключает motor и положительный ideal-speed torque до шага.

- [ ] **Step 4:** `uv run --locked pytest tests/test_static_brake.py tests/test_motor_assist.py -q`. Полный inclined-plane hold 5 s, затем torque выше предела — колесо должно начать двигаться, а не сохранять искусственную фиксацию.
- [ ] **Step 5:** `git add src/bike_sim/sim/ride/braking.py src/bike_sim/mujoco/steering_fork.py src/bike_sim/mujoco/drivetrain.py src/bike_sim/sim/ride_sim.py tests/test_static_brake.py && git commit -m "fix: support static wheel braking and cancel motor assistance"`.

## Review gate D

Проверить общие массу, энергию и момент импульса после переноса кассеты/шатунов. Цепной torque не приложен дважды; freehub имеет пару реакций; physical assist не держит скорость на спуске и не исчезает из-за фильтра контакта. Кратковременные power caps проверяются в те же моменты, когда вычислен и приложен torque, а не смешанными pre/post-step выборками.