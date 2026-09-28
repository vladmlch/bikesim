# 05 — физически связанный райдер Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Заменить независимые направляемые массы связанным плоским телом с настоящим размыканием опор и суставным педалированием.

**Architecture:** Новый rider variant не меняет legacy seated. Анатомические массы постоянны, таз имеет собственный плоский root в worldbody, конечности образуют дерево. Взаимодействие с велосипедом проходит через наблюдаемые контактные пары; управление создаёт только внутренние суставные моменты.

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

E1 можно выполнять после B1–B2. E2 требует D1 для движущихся pedal sites. В physical builder не вызывать legacy `compute_static_system_cg` с `articulated_planar` до компиляции: этот путь использует старую seated/load-path модель. Геометрию нового тела строить geometry-only adapter, а CoM marker заполнять по compiled mass-properties B1 после mj_forward. Новый райдер создаётся один раз в worldbody; старая ветвь build_seated_rider при этом не вызывается. E3 использует C1, C3–C4 и A2. E4 требует D1–D4 и E3. Текущие референсы: R11–R12, R15 и R19.

### Task E1: постоянные анатомические массы

**Files:** Create `src/bike_sim/physics/rider_segments.py`; Modify `src/bike_sim/physics/rider.py` (`RIDER_VARIANTS`, геометрический adapter нового варианта); Test `tests/test_rider_anatomy.py`.

**Interfaces:** Produces `segment_masses(total_kg,helmet_kg)->dict[str,float]`. Имена: pelvis, torso, head, upper_arm_pair, forearm_pair, thigh_front/shank_front/foot_front, thigh_rear/shank_rear/foot_rear. Опорные shares не входят в аргументы.

- [ ] **Step 1: failing test суммы и сегментов.**

```python
import pytest
from bike_sim.physics.rider_segments import segment_masses

def test_anatomical_mass_includes_helmet_once():
    masses = segment_masses(80.0,0.4)
    assert sum(masses.values()) == pytest.approx(80.0,abs=1e-10)
    assert masses["head"] == pytest.approx(0.0694*79.6+0.4)
    assert masses["thigh_front"] == masses["thigh_rear"]
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_rider_anatomy.py -q`.
- [ ] **Step 3: использовать анатомические fractions, не пути нагрузки.**

```python
from math import isfinite
from bike_sim.physics.rider import DE_LEVA_MASS_FRACTIONS

def segment_masses(total_kg,helmet_kg):
    if not isfinite(total_kg) or not isfinite(helmet_kg) or not 0 <= helmet_kg < total_kg:
        raise ValueError("invalid rider or helmet mass")
    f = DE_LEVA_MASS_FRACTIONS
    body = total_kg-helmet_kg
    result = {
        "pelvis": f["trunk_lower"]*body,
        "torso": (f["trunk_upper"]+f["trunk_middle"])*body,
        "head": f["head"]*body+helmet_kg,
        "upper_arm_pair": 2*f["upper_arm"]*body,
        "forearm_pair": 2*(f["forearm"]+f["hand"])*body,
    }
    for side in ("front","rear"):
        result["thigh_"+side] = f["thigh"]*body
        result["shank_"+side] = f["shank"]*body
        result["foot_"+side] = f["foot"]*body
    return result
```

Новый variant `articulated_planar` получает geometry-only pose. Разделить получение joint centers из существующего solve_seated_pose и последующее создание старых load-path bodies; новая геометрическая функция не принимает saddle/pedal/bar shares. Если для первого коммита используется adapter к прежнему solver, передавать фиксированные default shares и брать только joint centers, не `pose.bodies` и не `path_masses_kg`; затем покрыть тестом неизменность новой анатомии при изменении desired support distribution.

- [ ] **Step 4:** `uv run --locked pytest tests/test_rider_anatomy.py tests/test_ride_equilibrium.py -q`. Проверить 60/80/100 kg и helmet 0/0.4 kg; legacy seated сохраняет старую ветвь.
- [ ] **Step 5:** `git add src/bike_sim/physics/rider_segments.py src/bike_sim/physics/rider.py tests/test_rider_anatomy.py && git commit -m "feat: separate anatomical rider mass from support shares"`.

### Task E2: независимый плоский root и связанное дерево тела

**Files:** Create `src/bike_sim/mujoco/articulated_rider.py`; Modify `src/bike_sim/mujoco/builder.py`, `src/bike_sim/mujoco/frame.py:240-253`, `src/bike_sim/sim/equilibrium.py` (инициализация rider root); Test `tests/test_articulated_rider_topology.py`.

**Interfaces:** Produces `build_articulated_rider(worldbody,pose,masses)->dict[str,ET.Element]`. Body names `rider_pelvis`, `rider_torso`, `rider_head`, `rider_upper_arm_pair`, `rider_forearm_pair`, `rider_thigh_front`, `rider_shank_front`, `rider_foot_front` и rear counterparts. Root joints `rider_root_x`, `rider_root_z`, `rider_root_pitch`.

- [ ] **Step 1: failing topology test без ссылки на внешний вид.**

```python
import mujoco
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig

def test_pelvis_is_not_a_vertical_slider_attached_to_bike():
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode="ride",rider="articulated_planar",
        physics_config=SimulationPhysicsConfig(physics_mode="physical")))
    obj = mujoco.mjtObj.mjOBJ_BODY
    pelvis = mujoco.mj_name2id(model,obj,"rider_pelvis")
    thigh = mujoco.mj_name2id(model,obj,"rider_thigh_front")
    shank = mujoco.mj_name2id(model,obj,"rider_shank_front")
    assert model.body_parentid[pelvis] == 0
    assert model.body_parentid[thigh] == pelvis
    assert model.body_parentid[shank] == thigh
    assert model.body_jntnum[pelvis] == 3
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_articulated_rider_topology.py -q`; ожидается неподдержанный variant/topology.
- [ ] **Step 3: построить дерево в worldbody.**

```python
import xml.etree.ElementTree as ET

def planar_root(worldbody,hip):
    pelvis = ET.SubElement(worldbody,"body",name="rider_pelvis",
                           pos=" ".join(format(float(x),".17g") for x in hip))
    for name,kind,axis in (("rider_root_x","slide","1 0 0"),
                           ("rider_root_z","slide","0 0 1"),
                           ("rider_root_pitch","hinge","0 1 0")):
        ET.SubElement(pelvis,"joint",name=name,type=kind,axis=axis)
    return pelvis

def articulated_child(parent,name,joint,offset):
    body = ET.SubElement(parent,"body",name=name,
                         pos=" ".join(format(float(x),".17g") for x in offset))
    ET.SubElement(body,"joint",name=joint,type="hinge",axis="0 1 0",damping="0")
    return body
```

Торс — child pelvis в hip; head — fixed child torso. Thigh каждой стороны — child pelvis в hip, shank — child thigh в knee, foot — child shank в ankle. Upper_arm_pair — child torso в shoulder, forearm_pair — child upper arm в elbow. Все исходные тела могут иметь identity orientation: геометрические векторы исходной позы задаются endpoints, joints первоначально q=0. Для каждого child offset вычисляется как разность координат его joint center и joint center parent в исходной BB-системе.

Каждый сегмент получает положительную массу из E1 и отдельный physical inertial. Для synthetic цилиндрического сегмента длины L, радиуса r и unit axis u: `I_parallel=m*r*r/2`, `I_transverse=m*(3*r*r+L*L)/12`, `I=I_transverse*eye(3)+(I_parallel-I_transverse)*outer(u,u)`. Центр — объявленный CoM сегмента, изначально midpoint; head — sphere `I=0.4*m*r*r*eye(3)`. Визуальные геомы mass=0. Размеры physical сегментов и их provenance хранить отдельно от mesh/цвета.

На equilibrium reset rider_root_x/z и исходную позу согласовать с начальным положением велосипеда. Это установка initial condition, не ежешаговая коррекция. Старый `RiderForceApplier` для нового variant инертен; новые опоры подключит E3. Не запускать статический solve нового rider до E3: отдельный topology test компилирует модель без попытки выдать unsupported equilibrium за результат.

- [ ] **Step 4:** `uv run --locked pytest tests/test_articulated_rider_topology.py tests/test_rider_anatomy.py -q`. Проверить массу всех rider bodies, родительские связи и неизменность длины сегментов при произвольных допустимых joint angles.
- [ ] **Step 5:** `git add src/bike_sim/mujoco/articulated_rider.py src/bike_sim/mujoco/builder.py src/bike_sim/mujoco/frame.py src/bike_sim/sim/equilibrium.py tests/test_articulated_rider_topology.py && git commit -m "feat: build independently rooted articulated planar rider"`.

### Task E3: опоры с реакциями и настоящим размыканием

**Files:** Create `src/bike_sim/sim/ride/rider_contacts.py`; Modify `src/bike_sim/sim/ride_sim.py`, `src/bike_sim/sim/equilibrium.py`; Test `tests/test_rider_contact_reactions.py`.

**Interfaces:** Produces `apply_internal_force(model,data,body_a,body_b,point,force,qfrc)->None`, `RiderContactApplier.compute_qfrc(model,data,dt)->np.ndarray`. Записываются контакты saddle, front_pedal, rear_pedal, grip и их полные wrench.

- [ ] **Step 1: тест пары реакций в общей точке.**

```python
import mujoco
import numpy as np
from bike_sim.sim.ride.rider_contacts import apply_internal_force

def test_internal_force_has_no_net_force_or_world_moment():
    model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
    <worldbody>
      <body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/>
      <joint type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
      <body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/>
      <joint type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    qfrc = np.zeros(model.nv)
    apply_internal_force(model,data,1,2,np.array([0.5,0.,0.5]),np.array([5.,0.,3.]),qfrc)
    assert abs(qfrc[0]+qfrc[3]) < 1e-12
    assert abs(qfrc[1]+qfrc[4]) < 1e-12
    assert abs(qfrc[2]+qfrc[5]-qfrc[4]) < 1e-12
```

Последнее равенство включает orbital moment силы на body b в x=1; сумма одних hinge torques не является полным world moment.

- [ ] **Step 2:** `uv run --locked pytest tests/test_rider_contact_reactions.py -q`.
- [ ] **Step 3: парное приложение и опорные законы.**

```python
import mujoco
import numpy as np

def apply_internal_force(model,data,body_a,body_b,point,force,qfrc):
    p, f = np.asarray(point,dtype=float),np.asarray(force,dtype=float)
    if p.shape != (3,) or f.shape != (3,) or not np.isfinite(p).all() or not np.isfinite(f).all():
        raise ValueError("invalid interface force")
    if body_a <= 0 or body_b <= 0 or body_a == body_b:
        raise ValueError("internal reaction needs two distinct physical bodies")
    mujoco.mj_applyFT(model,data,f,np.zeros(3),p,body_a,qfrc)
    mujoco.mj_applyFT(model,data,-f,np.zeros(3),p,body_b,qfrc)
```

Saddle/pedal normal force вычислять C3 по signed gap фактических поверхностей. Для педали проекция стопы должна лежать внутри платформы; выход за край размыкает опору. Tangential force — C4 при Fn>0, без способности тянуть плоскую педаль вверх. Для symmetric hand-pair применять суммарный wrench в выбранной точке grip на `steer`, а не в произвольной точке frame.

Bilateral grip задавать состоянием упругого смещения в общей точке контакта, обновляемым относительной скоростью в этой же точке. Для isotropic grip: `xi_new=xi+dt*u`, `F=-k*xi_new-c*u`, `U=k*dot(xi_new,xi_new)/2`. Применить равные противоположные F в одной точке; записывать согласованную работу. Не сочетать geometric spring energy между двумя разными anchors с force power, вычисленной в несогласованных точках. Ограничение достижимости руки проверять по геометрии; при отпускании grip сбросить его state с учётом удалённой энергии.

Все joint stabilizing torques в тесте release отключены. Поднять bike/rider над terrain, разомкнуть все четыре опоры, приложить внешний горизонтальный импульс только к bike: rider должен остаться без горизонтального ускорения от этого импульса. Этот тест отличает реальное размыкание от нулевого значения видимой saddle spring при скрытой slider-связи.

- [ ] **Step 4:** `uv run --locked pytest tests/test_rider_contact_reactions.py tests/test_articulated_rider_topology.py -q`. Проверить односторонность saddle/pedals, взаимную работу пары, отсутствие изменения масс при release и общий Ly в тесте F2.
- [ ] **Step 5:** `git add src/bike_sim/sim/ride/rider_contacts.py src/bike_sim/sim/ride_sim.py src/bike_sim/sim/equilibrium.py tests/test_rider_contact_reactions.py && git commit -m "feat: add physical rider support contacts and balanced reactions"`.

### Task E4: внутренние суставные моменты и педалирование без двойной мощности

**Files:** Create `src/bike_sim/sim/ride/rider_control.py`; Modify `src/bike_sim/mujoco/articulated_rider.py` (joint actuators), `src/bike_sim/sim/ride/drivetrain_forces.py`, `src/bike_sim/sim/ride_sim.py`; Test `tests/test_articulated_pedaling.py`.

**Interfaces:** Produces `stance_force(phase,mean_nm,crank_m)->np.ndarray`, `bounded_joint_torque(q,qd,target,kp,kd,limit)->np.ndarray`, `ArticulatedRiderController.compute(model,data,command)->dict[joint_name,float]`. Команда содержит средний requested crank torque и целевые углы позы; delivered torque измеряется отдельно.

- [ ] **Step 1: тест направлений опорной фазы и ограничения суставов.**

```python
import numpy as np
from bike_sim.sim.ride.rider_control import stance_force, bounded_joint_torque

def test_front_stance_pushes_down_and_return_stroke_does_not_pull():
    force = stance_force(0.0,20.0,0.165)
    assert force[2] < 0
    np.testing.assert_allclose(stance_force(np.pi,20.0,0.165),np.zeros(3),atol=1e-12)

def test_joint_controller_cannot_exceed_torque_limit():
    torque = bounded_joint_torque(np.array([0.]),np.array([0.]),np.array([2.]),100.,10.,30.)
    np.testing.assert_allclose(torque,[30.])
```

- [ ] **Step 2:** `uv run --locked pytest tests/test_articulated_pedaling.py -q`.
- [ ] **Step 3: реализовать requested forces и внутренние actuator.**

```python
from math import cos, sin, pi, isfinite
import numpy as np

def stance_force(phase,mean_nm,crank_m):
    if not all(isfinite(x) for x in (phase,mean_nm,crank_m)) or mean_nm < 0 or crank_m <= 0:
        raise ValueError("invalid pedal stance request")
    weight = max(cos(phase),0.0)*pi/2
    tangent = np.array([-sin(phase),0.,-cos(phase)])
    return (mean_nm/crank_m)*weight*tangent

def bounded_joint_torque(q,qd,target,kp,kd,limit):
    q,qd,target = map(lambda x:np.asarray(x,dtype=float),(q,qd,target))
    if q.shape != qd.shape or q.shape != target.shape or not np.isfinite(np.r_[q,qd,target]).all():
        raise ValueError("invalid joint states")
    if not all(isfinite(x) for x in (kp,kd,limit)) or min(kp,kd,limit) < 0:
        raise ValueError("invalid joint gains or limit")
    return np.clip(kp*(target-q)-kd*qd,-limit,limit)
```

Для второй ноги phase сдвинут на pi. Сумма requested torque от двух фаз имеет требуемое среднее; фактически переданный torque может быть меньше из-за контакта, трения и ограничений суставов. Контроллер не меняет Fn и не заставляет контакт выполнять невозможную тягу.

Для stance получить foot Jacobian относительно трёх joint DOF данной ноги, `tau=J_foot.T@F_target`, затем добавить ограниченный postural PD. Не использовать колонки root DOF как actuator: это создало бы внешнюю силу. Ограничить уже сумму feedforward+PD, а не каждый компонент отдельно до сложения. В swing фазе выбрать доступную IK-позу к фактическому положению следующей педали; проверить forward-kinematics целевой позы и насыщение при недостижимой точке. Использовать существующую двухзвенную геометрию как чистый solver, не записывать IK-углы напрямую в qpos.

`human_crank` actuator отключён в articulated_effort. Human work = сумма actual joint actuator force * соответствующая joint speed. Доставленная в crank мощность считается отдельно по pedal forces. Mid-drive остаётся единственным отдельным моторным моментом на crank. Remove guard articulated после прохождения E1–E4.

- [ ] **Step 4:** `uv run --locked pytest tests/test_articulated_pedaling.py tests/test_rider_contact_reactions.py tests/test_motor_assist.py -q`. Добавить тест отсутствия human_crank contribution, работу суставов, конечность в нижней мёртвой точке и частичную потерю foot contact без удержания ноги невидимой связью.
- [ ] **Step 5:** `git add src/bike_sim/sim/ride/rider_control.py src/bike_sim/mujoco/articulated_rider.py src/bike_sim/sim/ride/drivetrain_forces.py src/bike_sim/sim/ride_sim.py tests/test_articulated_pedaling.py && git commit -m "feat: drive pedals with bounded internal rider actuation"`.

## Review gate E

Новый райдер не получает физические массы из support shares. Таз не привязан к раме по X/pitch. Все четыре опоры можно разомкнуть. В полёте управление сохраняет общий Ly и меняет его распределение, а не создаёт внешний root torque. Режим articulated_effort не оплачивает одну человеческую работу дважды.