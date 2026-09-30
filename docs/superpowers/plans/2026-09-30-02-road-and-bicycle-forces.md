# Контакт дороги и силовая механика велосипеда — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Получить проверяемую reference-модель нескольких опор шины и геометрического воздействия привода на подвеску без высокочастотной упругой цепи.

**Architecture:** Сохранить текущий single-support backend как быстрый и явно ограниченный. Новый distributed contact первоначально является экспериментальным стендовым backend с энергией, заданной до сил. Для привода использовать существующую геометрию цепи и редуцировать её в согласованное ограничение полного виртуального перемещения, а не накладывать дополнительную силу поверх прежнего tendon.

**Tech Stack:** Python 3.13, MuJoCo 3.12.0, NumPy, SciPy, pytest, XML, JSON/TOML; только локальные зависимости.

**Spec:** `docs/superpowers/specs/2026-09-30-bikesim-fidelity-audit.md`, F4–F5; монитор и replay из плана `2026-09-30-01-rider-and-validity.md`.

## Global Constraints

- Не реализовывать алгоритм anti-wheelie и не менять момент по признакам подъёма переднего колеса.
- Сохранить основной CLI, `articulated_planar` и пользовательские TOML; новые режимы должны быть явно названы.
- Не скачивать зависимости; использовать имеющиеся Python, NumPy, SciPy, MuJoCo, pytest и стандартную библиотеку.
- Не прикладывать скрытые силы/моменты к корню велосипеда или райдера. Не править `qpos`/`qvel` во время езды для восстановления позы, контакта или устойчивости. Инициализационная установка состояния разрешена и должна оставаться отдельной.
- Силы взаимодействия райдера и велосипеда должны быть внутренними и попарно противоположными; контакт дороги — односторонним.
- Разделять запрос, фактически переданное усилие, численную достоверность, применимость модели и экспериментальную валидацию.
- Сохранять единственную физическую временную шкалу и неизменяемые интервальные samples; диагностический просмотр не должен второй раз продвигать состояние силовых моделей.
- Плоская модель относится к продольной езде по жёсткому рельефу. Повороты, боковой уклон, колеи, рулевой баланс и деформируемый грунт не объявлять поддержанными этой моделью.
- Любые предложенные численные допуски и новые параметры человека/шины первоначально имеют статус инженерных, синтетических, а не измеренных.

---

## Проектное решение и границы уверенности

Наличие двух различных нормалей на целевой трассе подтверждено геометрическими пробами, но погрешность результирующей нагрузки ещё не измерена. Поэтому новый backend сначала должен стать **проверяемой reference-гипотезой**, а не немедленной заменой всех шин.

Выбрана минимальная распределённая penalty-модель: конечное число безмассовых material stations по окружности шины с фиксированными квадратурными весами; потенциальная энергия каждого контакта определяет его нормальную силу. Это не FEM и не заявление о точном пневматическом каркасе. Количество станций уточняется отдельно от сетки дороги. Если модель не воспроизводит радиальную кривую/контакт со ступенью в заданной области, она остаётся experimental и не получает production capability. Старый backend продолжает маркировать multi-support как invalid.

## Карта файлов

| Файл | Ответственность |
|---|---|
| `src/bike_sim/terrain/profile_distance.py` — новый | Signed distance точки до того же скомпилированного профиля |
| `src/bike_sim/physics/distributed_tire.py` — новый | Плотность potential/force и фиксированные квадратурные веса |
| `src/bike_sim/sim/ride/distributed_tire_forces.py` — новый | MuJoCo Jacobian mapping, отдельные brush states, snapshots |
| `src/bike_sim/physics/physical_config.py`, `src/bike_sim/physics/resolution.py` | Явный новый backend и параметры материала |
| `src/bike_sim/mujoco/physical_topology.py`, `src/bike_sim/sim/ride/physical_runtime.py` | Выбор одного backend и отключение двойного контакта |
| `src/bike_sim/sim/research/environment.py` | Capability-based допуск, не прежнее сравнение строки backend |
| `src/bike_sim/validation/contact_manifold_rigs.py` — новый | Статика, ступень, потеря/смена контактов, сходимость |
| `src/bike_sim/physics/transmission_constraint.py` — новый | Линеаризация полного геометрического ограничения |
| `src/bike_sim/sim/ride/geometric_freehub.py` — новый | Учет границы одностороннего привода и solver reaction |
| `src/bike_sim/sim/ride/drivetrain_forces.py` | Повторное использование chain geometry/Jacobian, новый режим |
| `src/bike_sim/validation/drive_suspension_rig.py` — новый | Проверка виртуальной работы и влияния на подвеску |

## Task B1: Signed-distance и potential kernel распределённой шины

**Files:** Create `src/bike_sim/terrain/profile_distance.py`, `src/bike_sim/physics/distributed_tire.py`, `tests/test_distributed_tire_kernel.py`, `tests/test_profile_distance.py`; Reuse `src/bike_sim/terrain/contact_profile.py:32–75`, `src/bike_sim/sim/ride/tire_forces.py:18–44`, `src/bike_sim/physics/tire_curve.py:17–92`.

**Interfaces:**
- `signed_profile_distance(vertices_xz: ndarray, points_xz: ndarray) -> tuple[ndarray, ndarray, ndarray, ndarray]`: signed distance >0 над поверхностью, outward normals, closest points, stable segment IDs; точки вне горизонтальной области профиля — явный ValueError.
- `station_angles(count: int) -> tuple[ndarray, ndarray]`: углы и веса с суммой 2π; веса не зависят от числа дорожных сегментов.
- `HingeDensity(knots_m: tuple[float, ...], stiffness_n_m: tuple[float, ...], damping_ns_m: float, provenance: str)` — плотность нормального закона на единицу угловой меры, **не** прежняя суммарная radial TireSpec.
- `density_response(delta: ndarray, material: HingeDensity) -> tuple[ndarray, ndarray]`: force density и energy density.
- `normal_station_response(delta: ndarray, delta_rate: ndarray, weights: ndarray, material: HingeDensity) -> tuple[ndarray, ndarray]`: неотрицательные нормальные силы и elastic energy станций.

- [ ] **Step 1: Написать test force = derivative of potential и независимости от повторения квадратуры.**

```python
import numpy as np
import pytest
from bike_sim.physics.distributed_tire import HingeDensity, density_response, station_angles

def test_normal_force_is_derivative_of_declared_energy():
    material = HingeDensity((0., .004), (10000., 20000.), 0., 'synthetic-test')
    d, eps = np.array([.003, .007]), 1e-7
    f, u = density_response(d, material)
    _, plus = density_response(d+eps, material)
    _, minus = density_response(d-eps, material)
    np.testing.assert_allclose(f, (plus-minus)/(2*eps), rtol=1e-6)
    assert np.all(u >= 0)

def test_quadrature_weight_is_not_contact_count():
    for count in (128, 256, 512):
        angles, weights = station_angles(count)
        assert len(angles) == count
        assert weights.sum() == pytest.approx(2*np.pi)
```

- [ ] **Step 2: Run** `python -m pytest tests/test_distributed_tire_kernel.py tests/test_profile_distance.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать kernel с фиксированной энергией.**

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class HingeDensity:
    knots_m: tuple[float, ...]
    stiffness_n_m: tuple[float, ...]
    damping_ns_m: float
    provenance: str

    def __post_init__(self):
        a, k = np.asarray(self.knots_m, float), np.asarray(self.stiffness_n_m, float)
        if a.ndim != 1 or len(a) == 0 or a.shape != k.shape:
            raise ValueError('invalid density table shape')
        if not np.isfinite([*a, *k, self.damping_ns_m]).all():
            raise ValueError('density parameters must be finite')
        if a[0] != 0 or np.any(np.diff(a) <= 0) or np.any(k < 0) or self.damping_ns_m < 0:
            raise ValueError('density law must be monotone and dissipative')
        if not self.provenance.strip():
            raise ValueError('material provenance is required')

def station_angles(count):
    if isinstance(count, bool) or not isinstance(count, int) or count < 16:
        raise ValueError('at least sixteen tread stations required')
    return 2*np.pi*np.arange(count)/count, np.full(count, 2*np.pi/count)

def density_response(delta, material):
    x = np.maximum(np.asarray(delta, float)[..., None]-np.asarray(material.knots_m), 0.)
    k = np.asarray(material.stiffness_n_m)
    return np.sum(k*x, axis=-1), .5*np.sum(k*x*x, axis=-1)

def normal_station_response(delta, delta_rate, weights, material):
    delta, rate, w = np.broadcast_arrays(delta, delta_rate, weights)
    if not np.isfinite([*delta.ravel(), *rate.ravel(), *w.ravel()]).all() or np.any(w < 0):
        raise ValueError('invalid contact quadrature')
    elastic, energy = density_response(delta, material)
    force = np.where(delta > 0, np.maximum(0., elastic+material.damping_ns_m*rate), 0.)
    return w*force, w*energy
```

Signed distance: ограничить поиск близкими сегментами по x, выполнить clamped projection, выбрать минимальную евклидову дистанцию; знак определить положением точки относительно piecewise-linear height в её x. Для точки на линии использовать нормаль сегмента `[-dz, dx] / length`; под линией outward normal направлен к ближайшей границе твёрдой области, над линией — от неё. При равных расстояниях использовать стабильное правило segment id, сохранить информацию о негладкой точке. Не сглаживать ступень скрыто. На профиле строго возрастающий x; вертикальная стенка представляется именно имеющимся raster, а не новым невидимым аналитическим препятствием.

Материальные станции `p_i(q) = wheel_center + R_wheel(q) @ [R*cos(theta_i), 0, R*sin(theta_i)]` вращаются вместе с колесом. `delta_i=max(-signed_distance(p_i),0)`. Elastic generalized force — `-∂U/∂q`, где `U=Σ w_i U_density(delta_i)`. В реализации через point Jacobian это `Σ J_i.T @ (N_i*n_i)`; проверить эквивалентность finite differences, включая wheel spin.

Форма закона hinge-density — осознанное ограничение. Суммарную flat-road load-deflection кривую вычислять отдельным стендом и подбирать неотрицательные density coefficients `scipy.optimize.nnls` по нескольким нагрузкам. Старый `radial_k_n_m` нельзя напрямую присваивать каждой станции. Если shape данной модели не аппроксимирует целевую кривую с ошибкой ≤5% в заданном диапазоне, fit считается rejected, а backend остаётся experimental. Порог 5% — первоначальное инженерное требование.

- [ ] **Step 4: Run** tests. Добавить signed-distance tests плоскости, наклонной, вогнутого сопряжения, ступени, outside-domain; finite differences по x/z/spin. Expected: forces nonnegative, potential gradient consistent вдали от геометрических негладкостей; суммарная radial load сходится при 128→256→512 станциях. В точке излома проверять одностороннюю производную/интегральную работу, не требовать несуществующей гладкой производной.
- [ ] **Step 5: Commit** `git add src/bike_sim/terrain/profile_distance.py src/bike_sim/physics/distributed_tire.py tests/test_distributed_tire_kernel.py tests/test_profile_distance.py && git commit -m "feat: add energy based distributed tire reference kernel"`.

## Task B2: Подключить backend и проверить многоточечный контакт на стенде

**Files:** Create `src/bike_sim/sim/ride/distributed_tire_forces.py`, `src/bike_sim/validation/contact_manifold_rigs.py`, `tests/test_distributed_tire_integration.py`; Modify `src/bike_sim/physics/physical_config.py`, `src/bike_sim/physics/resolution.py`, `src/bike_sim/mujoco/physical_topology.py:97–109`, `src/bike_sim/sim/ride/physical_runtime.py`, `src/bike_sim/sim/ride/physical_observations.py`, `src/bike_sim/sim/research/environment.py:74–78`, `src/bike_sim/validation/benchmarks.py:89–115`.

**Interfaces:**
- Новый `TireParameters.backend='distributed_2d_reference'`, config содержит count, density law, fitting dataset id, valid load/deflection domain, `calibration_status`.
- `DistributedTireForceApplier` предоставляет тот же runtime-протокол, что `TireForceApplier`: `compute_qfrc`, `reset`, `stored_energy`, `diagnostics`, `radii`, snapshots; точная сигнатура — `compute_qfrc(model, data, dt, *, advance=True) -> ndarray`; `reset() -> None`, `stored_energy(model, data) -> float`. Не менять callers других backend.
- State key `(wheel_side, station_id)`; segment ID — наблюдение геометрии, не идентичность material state.
- `distributed_flat_rig(dt_s: float, station_count: int, load_n: float) -> tuple[dict, dict]`, `distributed_step_rig(dt_s: float, station_count: int, terrain_dx_m: float) -> tuple[dict, dict]` возвращают metrics/bounds в формате существующего benchmark registry.
- Snapshot сохраняет физическую сумму `world_force_n`, `wheel_axis_moment_nm`, `patches`, а не среднее направлений, используемое как ложная единственная сила.

- [ ] **Step 1: Написать интеграционные tests capability и отсутствия double contact.**

```python
import pytest
from bike_sim.validation.contact_manifold_rigs import distributed_flat_rig

@pytest.mark.parametrize('count', [128, 256, 512])
def test_static_multicontact_supports_one_external_load(count):
    metrics, bounds = distributed_flat_rig(.000625, count, 600.)
    assert metrics['normal_force_n'] == pytest.approx(600., rel=.02)
    assert metrics['native_wheel_contact_count'] == 0
    assert metrics['energy_residual_ratio'] < .02
```

- [ ] **Step 2: Run** `python -m pytest tests/test_distributed_tire_integration.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать адаптер, следуя силовому ledger.** В каждом wheel step вычислить stations, signed distances, relative normal rates; затем использовать kernel B1. Для каждого активного patch применять силу и соответствующий момент через Jacobian. Нельзя назначать полный normal/tangent stiffness всем stations: и tangent elastic energy, и damping масштабируются теми же фиксированными весами.

```python
# Integration core inside the new applier; jp_i maps a world-point velocity.
qfrc = np.zeros(model.nv)
for station_id in active_station_ids:
    normal_force = normal_forces[station_id] * normals_world[station_id]
    tangent_force = tangent_forces[station_id] * tangents_world[station_id]
    force = normal_force + tangent_force
    qfrc += point_jacobians[station_id].T @ force
    # The snapshot stores this force and its moment about the wheel axis.
```

В этом ядре массивы являются локальными результатами `normal_station_response`, signed distance и существующего brush law; не вводить вторую силу из native geometry. Нормальную составляющую проверить через gradient potential B1. Tangent slip и точка приложения должны соответствовать одному Jacobian, иначе `Q·v` не совпадёт с контактной мощностью. Точки проекции normal-force лежат на одной нормали со station, поэтому перенос normal force к дороге не добавляет произвольного couple; для tangent force плечо задаётся явно и проверяется work-тестом.

При отрыве station обнулять shear state и записывать сброшенную energy как dissipation. При смене tangent переводить состояние объективно в новый базис. Не копировать один brush state сразу в несколько patches. `advance=False` не меняет время, xi и losses. Новые native exclusions применить во всех местах, где сейчас условие равно только `compliant_2d`.

Stands: плоскость 100/300/600/1000 Н; наклон без проскальзывания; ступень 4 см с подъездом; две опоры одновременно; уход переднего колеса с кромки; разгрузка и повторный контакт; малый bump при выключенном моторе. Heightfield и signed-distance profile всегда одинаковые. В registry добавить отдельные reference cases; новый backend не заменяет старые benchmark baselines.

- [ ] **Step 4: Run** `python -m pytest tests/test_distributed_tire_integration.py tests/test_tire_radial.py tests/test_tire_brush.py tests/test_profile_contact.py tests/test_model_status.py -q`. Expected: сохранены legacy tests; reference passes power/mass/no-double-contact checks. Обязателен comparison 128/256/512 stations и 10/5/2.5 мм профиля; force impulse между двумя наиболее точными расчётами отличается ≤2%, время потери опоры ≤5 мс. Если не достигнуто, не повышать capability/status и не ослаблять test без отчёта о причине.
- [ ] **Step 5: Commit** `git add src/bike_sim tests/test_distributed_tire_integration.py && git commit -m "feat: integrate auditable multipatch tire reference backend"`.

## Task B3: Проверить и восстановить геометрическую связь привода и подвески

**Files:** Create `src/bike_sim/physics/transmission_constraint.py`, `src/bike_sim/sim/ride/geometric_freehub.py`, `src/bike_sim/validation/drive_suspension_rig.py`, `tests/test_geometric_transmission.py`; Modify `src/bike_sim/mujoco/physical_topology.py:132–145`, `src/bike_sim/sim/ride/drivetrain_forces.py:80–106,258–283`, `src/bike_sim/physics/physical_config.py`, `src/bike_sim/physics/resolution.py`.

**Interfaces:**
- `linearized_upper_bound(phi_m: float, jacobian_m: ndarray, q: ndarray, boundary_m: float) -> tuple[ndarray, float]` для единого scalar-joint planar q.
- `constraint_reaction(tension_n: float, jacobian_m: ndarray) -> ndarray` возвращает `-T * J`, **включая** spin и suspension составляющие.
- `transmission_geometry(model, data, gearing) -> tuple[float, ndarray]`: значение полного chain-extension-like геометрического ограничения и его аналитический Jacobian. Для ideal режима вместо независимой cassette координаты используется привязанная к колесу передающая координата; копировать только spin terms из detailed branch недостаточно.
- `GeometricFreehubConstraint.reset/prepare/set_ratio/solved_qfrc` — тот же контракт жизненного цикла, что `IdealFreehubConstraint`; новый режим называется `geometric_ideal_mid_drive`.
- `drive_suspension_rig(dt_s, mode, imposed_torque_nm) -> tuple[dict, dict]`, где mode — `ideal_mid_drive`, `geometric_ideal_mid_drive` или существующая elastic-reference конфигурация.

- [ ] **Step 1: Написать тест tangent constraint и виртуальной работы.**

```python
import numpy as np
import pytest
from bike_sim.physics.transmission_constraint import linearized_upper_bound, constraint_reaction

def test_linearized_bound_preserves_gap_at_current_state():
    q = np.array([.3, .5, .02])
    j = np.array([.08, -.12, .4])
    phi, boundary = .003, .005
    coeff, upper = linearized_upper_bound(phi, j, q, boundary)
    assert upper-coeff@q == pytest.approx(boundary-phi)

def test_reaction_uses_full_virtual_displacement():
    j = np.array([.08, -.12, .4]); v = np.array([2., 1., .1])
    force = constraint_reaction(100., j)
    assert force@v == pytest.approx(-100.*(j@v))
    assert force[2] == pytest.approx(-40.)
```

- [ ] **Step 2: Run** `python -m pytest tests/test_geometric_transmission.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать power-consistent reduction и сравнение до переключения default.**

```python
import numpy as np

def linearized_upper_bound(phi_m, jacobian_m, q, boundary_m):
    j, q = np.asarray(jacobian_m, float), np.asarray(q, float)
    if j.ndim != 1 or j.shape != q.shape:
        raise ValueError('transmission Jacobian requires scalar planar coordinates')
    if not np.isfinite([phi_m, boundary_m, *j, *q]).all():
        raise ValueError('non-finite transmission linearization')
    return j.copy(), float(boundary_m-phi_m+j@q)

def constraint_reaction(tension_n, jacobian_m):
    j = np.asarray(jacobian_m, float)
    if j.ndim != 1 or not np.isfinite(j).all() or not np.isfinite(tension_n) or tension_n < 0:
        raise ValueError('invalid tensile transmission reaction')
    return -tension_n*j
```

Сначала выделить полную геометрию из существующих `_geometry`, `chain_extension`, `jacobian` без изменения знаков/frames. Finite-difference test должен покрывать **каждую** scalar q, особенно linkage DOFs. Проверить инвариантность при общей трансляции и вращении велосипеда. Никаких ручных констант «anti-squat force».

Для реализации нового ideal constraint использовать locally linearized fixed tendon по scalar hinge/slide coordinates: коэффициенты `J_n`, верхняя граница `b - phi(q_n) + J_n q_n`. Это один совмещённый solver constraint; прежний двухугловой ideal tendon в этом режиме не активен. Update coefficients/range выполняется до solve, используется отдельный scratch MjData для обновления constants, не reset рабочего состояния. Применимость этой линеаризации ограничена шагом: после solve обязательно записывать `phi(q_{n+1}) - phi(q_n) - J_n @ (q_{n+1}-q_n)`. Нельзя скрывать большую нелинейную ошибку уменьшением output precision.

Односторонняя freehub-boundary обновляется по полному phi, а не только crank/wheel angle. При смене передачи сохранить физический gap, отдельно записать переключательную работу; не применять геометрию новой передачи задним числом к прошлому интервалу. Solver reaction целиком идёт в `ideal_transmission` ledger. Не добавлять второй раз `-tension*J` к applied forces поверх solved constraint — helper нужен для проверки/эталона знаков.

Стенд сравнения: зафиксированные на начальной стадии углы корпуса/внешняя нагрузка, три положения подвески, одинаковый доставленный момент на валу 0/20/40 Н·м; затем короткое свободное движение подвески. Сначала это локализованный механический эксперимент, не полноразмерная экстремальная трасса. Сравнить rear travel, shock force, front normal load, transmitted shaft power с elastic reference при достаточно малом dt. Differences старого ideal — измеренный результат стенда, а не заранее заданный процент.

- [ ] **Step 4: Run** `python -m pytest tests/test_geometric_transmission.py tests/test_compiled_mass_contract.py tests/test_physical_topology_xml.py -q`. Expected: helper/Jacobian/work tests PASS. Дополнительно: нулевая передача тяги при overrun, ненулевая suspension составляющая при геометрическом chain growth, отсутствие внешнего CoM импульса в воздухе, сходимость constraint defect при dt/2. При сравнении в диапазоне 20/40 Н·м нормированная ошибка средних сил относительно elastic reference ≤5% — проектный начальный gate, не доказанный результат. Если gate не пройден, новый режим остаётся experimental; текущий ideal сохраняет пометку о пропущенной геометрической связи.
- [ ] **Step 5: Commit** `git add src/bike_sim/physics/transmission_constraint.py src/bike_sim/sim/ride/geometric_freehub.py src/bike_sim/sim/ride/drivetrain_forces.py src/bike_sim/mujoco/physical_topology.py src/bike_sim/validation/drive_suspension_rig.py src/bike_sim/physics/physical_config.py src/bike_sim/physics/resolution.py tests/test_geometric_transmission.py && git commit -m "feat: add geometric reduced mid drive reference constraint"`.

## Task B4: Закрепить контракт момента и отделить стенд от логики езды

**Files:** Create `tests/test_plant_torque_contract.py`, `examples/research/plant_reference_open_loop.toml`; Modify `src/bike_sim/validation/rider_replay.py` из A3 и `src/bike_sim/validation/benchmarks.py`.

**Interfaces:**
- Использовать **существующий** `RideControl(motor_torque_nm=..., human_torque_nm=..., posture=...)`, не вводить второй API двигателя.
- `open_loop_schedule(time_s: float) -> RideControl`: момент зависит только от времени.
- Диагностика разделяет `motor_setpoint_nm`, `motor_request_nm`, `motor_torque_nm`, power/energy limits, human delivered torque, gear ratio, brake demands.

- [ ] **Step 1: Написать test открытой программы без обратной связи.**

```python
from bike_sim.validation.rider_replay import open_loop_schedule

def test_torque_schedule_uses_only_time():
    assert open_loop_schedule(0.).motor_torque_nm == 0.
    assert open_loop_schedule(1.5).motor_torque_nm == 20.
    assert open_loop_schedule(2.5).motor_torque_nm == 40.
    assert open_loop_schedule(5.).motor_torque_nm == 0.
```

- [ ] **Step 2: Run** `python -m pytest tests/test_plant_torque_contract.py -q`. Expected: missing schedule.

- [ ] **Step 3: Реализовать расписание и отдельный стендовый preset.**

```python
from bike_sim.sim.ride.control import RideControl

def open_loop_schedule(time_s):
    if time_s < 1. or time_s >= 4.:
        torque = 0.
    elif time_s < 2.:
        torque = 40.*(time_s-1.)
    else:
        torque = 40.
    return RideControl(motor_torque_nm=torque, human_torque_nm=0.)
```

В `plant_reference_open_loop.toml` явно выключить automatic shifting и rollback brake; передачу фиксировать 34/51. Сохранить physical actuator lag, torque/power/speed envelope и батарейные ограничения выбранного профиля. Numeric external setpoint уже обходит только demand gating, не физику двигателя. Выбор backend, закона подвески, rider posture и массы пишется в report. Профиль не должен наследовать невидимый anti-wheelie или pitch assist.

Проверить отдельно нулевой момент, ramp и отпускание. Ground-truth wheelie может записываться, но не используется входом `open_loop_schedule`. Human-only replay A3 и motor-only replay этого task дают разные причинные стенды; их нельзя смешать и объявить изменение front load эффектом только двигателя.

- [ ] **Step 4: Run** `python -m pytest tests/test_plant_torque_contract.py tests/test_rider_replay.py tests/test_wheelie_detection.py -q`. Expected: PASS. На forward и airborne rigs убедиться, что выдаваемый shaft power соответствует actual torque × shaft rate, а передаточное отношение применяется ровно один раз. Для motor-only в воздухе общий CoM движется только под внешними силами; внутренний обмен угловым моментом разрешён.
- [ ] **Step 5: Commit** `git add tests/test_plant_torque_contract.py examples/research/plant_reference_open_loop.toml src/bike_sim/validation/rider_replay.py src/bike_sim/validation/benchmarks.py && git commit -m "test: separate physical torque plant from riding automation"`.

## Приёмка плана B

Новое сложное представление контакта не считается улучшением только потому, что содержит больше patches. Обязательны force/potential consistency, static-load fit, event/mesh convergence, отсутствие double counting и passivity при смене состояния. Редуцированная трансмиссия должна сохранять виртуальную работу **всей** силовой связи. Если эти требования не выполняются, результат task остаётся диагностическим reference-прототипом и не разрешает пользователю получать «валидные» данные на неподдержанном участке.

План C определяет численные/экспериментальные gates. До их прохождения основной пользовательский preset сохраняет читаемую маркировку `parameterized_unvalidated` и ограничения single-support/ideal drive.