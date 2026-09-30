# Проверка, калибровка и выпуск reference-профиля — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Сделать точность и область применимости симуляции проверяемыми по механическим инвариантам, численной сходимости и независимым измерениям.

**Architecture:** Расширить существующие `src/bike_sim/validation/benchmarks.py` и `src/bike_sim/validation/datasets.py`, а не создавать второй validation framework. Разделить три результата: корректность механики, численная сходимость, соответствие реальному велосипеду/человеку. Быстрый профиль выбирается по пройденным допускам относительно reference, а не по красивому viewer-проезду.

**Tech Stack:** Python 3.13, MuJoCo 3.12.0, NumPy, SciPy, pytest, JSON/CSV/TOML, существующие benchmark и recording utilities; только offline-окружение.

**Spec:** `docs/superpowers/specs/2026-09-30-bikesim-fidelity-audit.md`, F8, разделы 6–9; планы A и B описывают проверяемые компоненты.

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

## Карта файлов

| Файл | Ответственность |
|---|---|
| `src/bike_sim/validation/load_transfer.py` — новый | Аналитический частный случай и механические residuals |
| `src/bike_sim/validation/system_momentum.py` — новый | Суммарный импульс и угловой момент всей системы |
| `src/bike_sim/validation/fidelity_sweep.py` — новый | Отдельные time/mesh/station sweeps и comparison report |
| `src/bike_sim/validation/benchmarks.py` | Регистрация новых cases и использование существующего runner |
| `src/bike_sim/validation/datasets.py` | Дополнительные rig types, единицы, uncertainty/provenance |
| `src/bike_sim/validation/experiment_acceptance.py` — новый | Разделение fit/holdout и честный статус калибровки |
| `src/bike_sim/validation/reference_release.py` — новый | Машиночитаемые release gates и допустимый профиль |
| `examples/research/plant_reference_open_loop.toml` | Нейтральный вход на вал, не anti-wheelie policy |
| `examples/research/plant_reference_validated.toml` — создаётся только при gates | Проверенный набор параметров с provenance |
| `docs/validation/bikesim_measurement_protocol.md` — новый | Измерительный протокол и границы доверия |
| `docs/validation/reference_profile.md` — новый | Поддержанные сценарии и явные исключения |

## Task C1: Добавить эталон переноса нагрузки и баланс импульса системы

**Files:** Create `src/bike_sim/validation/load_transfer.py`, `src/bike_sim/validation/system_momentum.py`, `tests/test_load_transfer_reference.py`, `tests/test_system_momentum.py`; Modify `src/bike_sim/validation/benchmarks.py:89–115`; Reuse existing airborne/passive rigs.

**Interfaces:**
- `quasistatic_front_load(mass_kg, wheelbase_m, com_x_m, com_h_m, slope_rad, tangent_accel_mps2=0., gravity_mps2=9.81) -> float`.
- `system_momentum(model, data) -> tuple[ndarray, ndarray, ndarray]`: общий CoM, линейный импульс и угловой момент относительно общего CoM.
- `load_transfer_rig(dt_s, slope_rad, com_x_m, com_h_m) -> tuple[dict, dict]`: standalone rigid-equivalent rig, не полный динамический rider.
- `momentum_rig(dt_s, active: bool) -> tuple[dict, dict]`: полностью airborne system с выключенными аэродинамикой/rolling drag; внутреннее включение/выключение двигателя и суставов не меняет внешний импульс.

- [ ] **Step 1: Написать аналитические tests.**

```python
import pytest
from bike_sim.validation.load_transfer import quasistatic_front_load

def test_static_centered_bike_has_half_weight_on_front():
    assert quasistatic_front_load(100., 1.2, .6, .9, 0.) == pytest.approx(490.5)

def test_increasing_tangent_acceleration_unloads_front():
    resting = quasistatic_front_load(100., 1.2, .6, .9, 0.)
    accelerating = quasistatic_front_load(100., 1.2, .6, .9, 0., 1.)
    assert resting-accelerating == pytest.approx(75.)

def test_negative_reference_load_is_not_silently_clamped():
    assert quasistatic_front_load(100., 1.2, .2, 1., 0., 3.) < 0.
```

- [ ] **Step 2: Run** `python -m pytest tests/test_load_transfer_reference.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать эталон и общий momentum diagnostic.**

```python
from math import sin, cos, isfinite

def quasistatic_front_load(mass_kg, wheelbase_m, com_x_m, com_h_m,
                          slope_rad, tangent_accel_mps2=0., gravity_mps2=9.81):
    values = (mass_kg, wheelbase_m, com_x_m, com_h_m, slope_rad,
              tangent_accel_mps2, gravity_mps2)
    if not all(isfinite(v) for v in values):
        raise ValueError('load transfer inputs must be finite')
    if mass_kg <= 0 or wheelbase_m <= 0 or com_h_m < 0 or gravity_mps2 <= 0:
        raise ValueError('invalid rigid-equivalent geometry')
    return mass_kg/wheelbase_m * (
        gravity_mps2*(com_x_m*cos(slope_rad)-com_h_m*sin(slope_rad))
        -tangent_accel_mps2*com_h_m)
```

Отрицательный результат означает недопустимость двухопорного квазистатического состояния; не превращать его в «измеренное отрицательное Fn». Формула не учитывает изменение углового момента вращающихся колёс, pitch acceleration, движение райдера, аэродинамический момент и разные нормали на препятствии. Поэтому сравнивать с ней только соответствующий rigid-equivalent fixture; полный велосипед проверяется общим балансом momentum.

```python
import mujoco
import numpy as np

def system_momentum(model, data):
    bodies = np.flatnonzero(model.body_mass > 0)
    masses = model.body_mass[bodies]
    positions = data.xipos[bodies]
    total_mass = float(masses.sum())
    if total_mass <= 0:
        raise ValueError('system must contain positive mass')
    com = np.sum(masses[:, None]*positions, axis=0)/total_mass
    linear, angular = np.zeros(3), np.zeros(3)
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for body in bodies:
        mujoco.mj_jacBodyCom(model, data, jp, jr, int(body))
        velocity, omega = jp@data.qvel, jr@data.qvel
        p = float(model.body_mass[body])*velocity
        rotation = data.ximat[body].reshape(3, 3)
        inertia = rotation@np.diag(model.body_inertia[body])@rotation.T
        linear += p
        angular += inertia@omega + np.cross(data.xipos[body]-com, p)
    return com, linear, angular
```

Нативные solver constraints, привод и контакты райдера не должны вносить внешний линейный импульс в airborne rig. Для углового баланса учитывать torque/couple контактных patches, внешний aero/drag и гравитацию относительно выбранной точки. В двухопорной статике сравнить сумму вертикальных сил с весом и правильное плечо общего CoM; варьировать положение груза и наклон, не фиксировать pitch внешним constraint с незаписанным реактивным моментом.

- [ ] **Step 4: Run** `python -m pytest tests/test_load_transfer_reference.py tests/test_system_momentum.py tests/test_compiled_mass_contract.py -q`. Expected: PASS. Начальные engineering gates: static forces ≤1% от веса; passive/active airborne impulse residual ≤0.1% от характерного импульса `m*g*duration`; threshold floors записать явно. Отдельно проверить offset invariance при общей трансляции на 100 м.
- [ ] **Step 5: Commit** `git add src/bike_sim/validation/load_transfer.py src/bike_sim/validation/system_momentum.py src/bike_sim/validation/benchmarks.py tests/test_load_transfer_reference.py tests/test_system_momentum.py && git commit -m "test: validate axle load transfer and whole system momentum"`.

## Task C2: Измерить сходимость по времени, дороге и контактной квадратуре

**Files:** Create `src/bike_sim/validation/fidelity_sweep.py`, `tests/test_fidelity_sweep.py`; Modify `src/bike_sim/validation/benchmarks.py:143–175`; Reuse `src/bike_sim/sim/research/configuration.py:46–53` и recorder.

**Interfaces:**
- `scalar_relative_error(value: float, reference: float, floor: float) -> float`.
- `interval_integral(rows: list[dict], key: str) -> float`: интеграл по duration каждого **входящего** интервала, не trapezoid на смешанных endpoint samples.
- `compare_metrics(coarse: dict, reference: dict) -> dict`: impulse error, mean/min load, contact-event time error, peak pitch rate, suspension travel, model-validity agreement.
- `run_fidelity_sweep(output_dir: str, cases: tuple[str, ...]) -> dict`; CLI `python -m bike_sim.validation.fidelity_sweep --output results/fidelity`.

- [ ] **Step 1: Написать tests корректного интервала и finite validation.**

```python
import pytest
from bike_sim.validation.fidelity_sweep import interval_integral, scalar_relative_error

def test_load_impulse_uses_interval_duration():
    rows = [{'time_s': 0., 'end_time_s': .01, 'load_n': 100.},
            {'time_s': .01, 'end_time_s': .03, 'load_n': 200.}]
    assert interval_integral(rows, 'load_n') == pytest.approx(5.)

def test_near_zero_reference_has_explicit_scale_floor():
    assert scalar_relative_error(.1, 0., 1.) == pytest.approx(.1)
```

- [ ] **Step 2: Run** `python -m pytest tests/test_fidelity_sweep.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать интеграл и orthogonal sweeps.**

```python
from math import isfinite

def scalar_relative_error(value, reference, floor):
    if not all(isfinite(v) for v in (value, reference, floor)) or floor <= 0:
        raise ValueError('finite comparison and positive floor required')
    return abs(value-reference)/max(abs(reference), floor)

def interval_integral(rows, key):
    result, previous_end = 0., None
    for row in rows:
        t, end, value = row['time_s'], row['end_time_s'], row[key]
        if not all(isfinite(v) for v in (t, end, value)) or end <= t:
            raise ValueError('invalid force interval')
        if previous_end is not None and abs(t-previous_end) > 1e-9:
            raise ValueError('gap or overlap in force history')
        result += value*(end-t)
        previous_end = end
    return result
```

Временной sweep: `dt = (0.00125, 0.000625, 0.0003125)` с **одинаковыми** материалами, motor/rider settings, сеткой 5 мм и closure time constant **0.0025 с**. Инициализировать одну и ту же физическую позу/скорость/упругие состояния; reset/restore допустим только до t=0. Не подменять совпадение начальных состояний некорректным кешем с чужим hash конфигурации. Если equilibrium различается, отдельно записать init residual и не приписывать его ошибке интегратора.

Пространственный sweep: `dx = (0.01, 0.005, 0.0025)` на фиксированном самом точном dt; использовать существующий `research_field`. Точки ступеней и входные траектории неизменны, но честно учитывать отличие скомпилированного raster. Для distributed backend отдельно `stations=(128,256,512)`, не менять одновременно dx и count.

Cases: static sag, motor ramp на гладком уклоне, coasting по одному bump/crest, одна ступень 4 см, повторный контакт после полёта, articulated coast/resume. Для старого single-support backend ступень может законно закончиться model-invalid; такой результат не включать в «passed physics convergence».

Метрики на каждом physics step: передняя и задняя patch-суммы/вертикальные силы, импульсы, время unload/recontact, pitch и pitch-rate, COM x/z, fork/shock travel, фактические human/motor torques, суставная положительная работа, contact/solver energy losses. Для классификации event переиспользовать WheelieTracker; не переопределять output label в anti-wheelie policy.

Начальные gates между двумя наиболее точными расчётами: normal impulse ≤2%; static/mean load ≤2% с floor 10 Н; suspension travel ≤1 мм; время устойчивого contact transition ≤5 мс; peak pitch-rate ≤5% с floor 0.05 рад/с. Events без соответствующей пары сравнивать как categorical mismatch, а не заменять временем 0. Gate по энергетическому residual остаётся отдельным; результат с NaN, ранним numerical abort или model-invalid не считается converged.

Report содержит все пары разрешений, maxima, failed criterion, source/config/terrain hashes и фактические versions. Runtime/performance измеряется отдельно от pass/fail физики. Не утверждать, что 1.25 мс достаточно, до измерений.

- [ ] **Step 4: Run** `python -m pytest tests/test_fidelity_sweep.py -q`, затем `python -m bike_sim.validation.fidelity_sweep --output results/fidelity`. Expected: unit PASS; полный report может содержать failed gates. В таком случае выбрать более точную конфигурацию либо исправить причину, не стирать неуспешные серии.
- [ ] **Step 5: Commit** `git add src/bike_sim/validation/fidelity_sweep.py src/bike_sim/validation/benchmarks.py tests/test_fidelity_sweep.py && git commit -m "test: certify time mesh and contact resolution separately"`.

## Task C3: Расширить измерительные контракты до велосипеда и райдера

**Files:** Modify `src/bike_sim/validation/datasets.py:9–105`; Create `src/bike_sim/validation/experiment_acceptance.py`, `tests/test_experiment_acceptance.py`, `docs/validation/bikesim_measurement_protocol.md`.

**Interfaces:**
- Дополнительные rig types: `axle_loads`, `rider_pose`, `suspension_kinematics`, `full_bike_run`; существующие tire/damper/motor сохраняются.
- Обязательные каналы/единицы: axle loads — `front_load_n`, `rear_load_n`, `slope_rad`; rider pose — `pelvis_x_m`, `pelvis_z_m`, `torso_pitch_rad`, `com_x_m`, `com_z_m`; suspension kinematics — `rear_travel_m`, `shock_stroke_m`; full bike — `time_s`, `pitch_rad`, `pitch_rate_rad_s`, `speed_mps`, `fork_travel_m`, `shock_stroke_m`, `motor_torque_nm`, `crank_rad_s`.
- Масса, pressure, temperature, gear, bike/rider config hashes, sensor calibration, units, uncertainty, synchronisation method и experiment ID сохраняются в metadata. Непосредственно не измеряемые каналы отмечаются estimate с отдельной uncertainty, не выдаются за sensor truth.
- `validate_split(fit_experiment_ids: list[str], holdout_experiment_ids: list[str]) -> None`.
- `accepted_calibration_status(*, synthetic: bool, holdout_passed: bool, converged: bool, within_scope: bool) -> str`.

- [ ] **Step 1: Написать tests запрета утечки и ложной валидации.**

```python
import pytest
from bike_sim.validation.experiment_acceptance import validate_split, accepted_calibration_status

def test_one_experiment_cannot_be_both_fit_and_holdout():
    with pytest.raises(ValueError):
        validate_split(['run-01'], ['run-01'])

def test_synthetic_data_never_produces_real_validated_status():
    result = accepted_calibration_status(synthetic=True, holdout_passed=True,
                                        converged=True, within_scope=True)
    assert result == 'parameterized_unvalidated'
```

- [ ] **Step 2: Run** `python -m pytest tests/test_experiment_acceptance.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать contract gates и измерительный протокол.**

```python
def validate_split(fit_experiment_ids, holdout_experiment_ids):
    if not fit_experiment_ids or not holdout_experiment_ids:
        raise ValueError('fit and holdout experiments are both required')
    if any(not isinstance(v, str) or not v.strip()
           for v in fit_experiment_ids+holdout_experiment_ids):
        raise ValueError('experiment identifiers must be nonempty strings')
    if set(fit_experiment_ids) & set(holdout_experiment_ids):
        raise ValueError('fit/holdout leakage at experiment level')

def accepted_calibration_status(*, synthetic, holdout_passed, converged, within_scope):
    if not all(type(v) is bool for v in (synthetic, holdout_passed, converged, within_scope)):
        raise ValueError('calibration gates require boolean evidence')
    if synthetic or not (holdout_passed and converged and within_scope):
        return 'parameterized_unvalidated'
    return 'validated_within_declared_scope'
```

Протокол измерений содержит последовательность, а не обещание отсутствующих данных:

1. Взвесить велосипед целиком и нагрузки осей с известной геометрией; оценить CoM по нескольким статическим положениям с uncertainty. Отделить sprung/unsprung masses и inertia колёс. Свести mass budget ровно к измеренному total mass.
2. Получить tire load-deflection при фиксированном pressure и несколько динамических compression/rebound серий; измерить применимый traction range на выбранном hardpack. Не переносить hardpack-параметры на loose soil.
3. Измерить rear travel/shock stroke relation по ходу, sag под известной нагрузкой, отдельные compression/rebound характеристики и отбойники. Не сводить всё к одному совпавшему sag.
4. Зафиксировать рост/массу райдера и несколько seated/standing/forward/rearward поз. Указать метод оценки segment masses/CoM; неопределённость anthropometric regression не равна нулю. Калибровать joint coordinate origins для A4.
5. Снять синхронные full-bike прогоны с безопасным для испытательного стенда open-loop моментом, измеренным рельефом и повторениями. Контактные нагрузки записывать, когда доступны датчики; при их отсутствии не объявлять точность Fn подтверждённой только по IMU.

Синтетические fixtures тестируют schema/фиттер, но не заменяют этапы 1–5. Data не предоставлены в задаче, поэтому deliverable этого task — работающий import/validation contract и протокол; статус `validated` появляется лишь после фактического добавления реальных holdout experiments. Это явный release gate, не пустой пункт плана.

Для initial holdout agreement отдельно задать измерительные uncertainty и allowable model discrepancy: static axle loads 3% веса; travel RMS 3 мм; pitch RMS 1 градус; pitch-rate RMS 0.05 рад/с. Эти допуски проектные и должны быть пересмотрены по диапазону будущего применения и точности датчиков. Не подбирать их после просмотра holdout ошибок ради PASS. Оценку эпизодов отрыва делать по измеренному контакту/видео с известной временной погрешностью, не только по увеличению pitch.

- [ ] **Step 4: Run** `python -m pytest tests/test_experiment_acceptance.py -q` и существующие dataset tests, найденные по `rg -l 'validate_dataset|RIG_UNITS' tests`. Expected: новые rig schemas и старые tire/damper/motor совместимы; synthetic/неполные/no-holdout datasets не дают real validation.
- [ ] **Step 5: Commit** `git add src/bike_sim/validation/datasets.py src/bike_sim/validation/experiment_acceptance.py tests/test_experiment_acceptance.py docs/validation/bikesim_measurement_protocol.md && git commit -m "feat: validate full bicycle and rider measurement datasets"`.

## Task C4: Выпустить раздельные preview/reference-профили по явным gates

**Files:** Create `src/bike_sim/validation/reference_release.py`, `tests/test_reference_release.py`, `docs/validation/reference_profile.md`; Modify `examples/research/viewer_physics_fast.toml` только с сохранением совместимости и явной маркировкой; output calibrated preset создаётся из реально одобренных данных, не заранее.

**Interfaces:**
- `release_gates(report: dict) -> tuple[bool, tuple[str, ...]]`.
- Report keys: `known_tests_passed`, `coast_resume_resolved`, `mechanics_passed`, `numerics_converged`, `model_scope_passed`, `holdout_passed`, `synthetic`, `scenarios`.
- У release два уровня: `numerically_verified_synthetic` и `validated_within_declared_scope`. Первый пригоден для разработки экспериментов, но не заявляет переносимость на реальную систему.

- [ ] **Step 1: Написать test запрета выпуска по одной красивой траектории.**

```python
from bike_sim.validation.reference_release import release_gates

def test_full_course_finish_is_not_a_substitute_for_physics_gates():
    report = dict(known_tests_passed=True, coast_resume_resolved=False,
        mechanics_passed=True, numerics_converged=False, model_scope_passed=True,
        holdout_passed=False, synthetic=True, scenarios={'extreme_finished': True})
    ok, reasons = release_gates(report)
    assert not ok
    assert 'coast_resume_resolved' in reasons
    assert 'numerics_converged' in reasons
```

- [ ] **Step 2: Run** `python -m pytest tests/test_reference_release.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать release gate и матрицу сценариев.**

```python
REQUIRED_NUMERICAL_GATES = (
    'known_tests_passed', 'coast_resume_resolved', 'mechanics_passed',
    'numerics_converged', 'model_scope_passed')

def release_gates(report):
    missing = tuple(key for key in REQUIRED_NUMERICAL_GATES if report.get(key) is not True)
    return not missing, missing
```

Сценарии reference: плоскость; гладкий подъём 5/12/20/28/35 **процентов**; одиночные bump, crest и ступени 4/6 см; две опоры; возврат из полёта; педалирование-накат-восстановление; несколько явно заданных поз; исходная 100-метровая extreme трасса. Уклон 35% — не 35 градусов; использовать `atan(grade)`.

На каждом сценарии outcome — `completed`, `physical_stall`, `loss_of_traction`, `contact_loss`, `crash`, `model_invalid`, `numerical_failure`, `duration_limit` с подтверждающими channels. Финиш не обязателен при физически недостаточной тяге, но причины должны быть наблюдаемы. Outcome не устанавливать только по знаку скорости одного sample. Сохранить first cause, время/координату, конфигурацию и входные команды.

Для preview можно оставить старое быстрое приближение с постоянным предупреждением об ограничениях. Для numerically verified profile выбрать максимальный timestep и минимальное количество contact stations из реально прошедших C2 вариантов. В reference documentation указать поддержанные load/pressure/grade/speed/posture domains и не поддержанные terrain types. Файл с суффиксом `validated` создавать только когда `accepted_calibration_status` из C3 вернул `validated_within_declared_scope`; synthetic report не может породить такой файл.

Не подменять отсутствие real-data валидации ручным флагом. В output сохранить calibration dataset IDs, configuration fingerprint, engine versions и результаты holdout. Исходный пользовательский CLI остаётся рабочим; новый профиль задаётся тем же `--physics-config`.

- [ ] **Step 4: Run** `python -m pytest tests/test_reference_release.py tests/test_experiment_acceptance.py tests/test_fidelity_sweep.py -q`, затем полный pytest-suite и reference matrix. Expected: unit PASS; выдача reference manifest только при всех соответствующих gates. Отсутствие натурных данных оставляет честный numerically-verified synthetic level и перечень недостающих измерений.
- [ ] **Step 5: Commit** `git add src/bike_sim/validation/reference_release.py tests/test_reference_release.py docs/validation/reference_profile.md examples/research/viewer_physics_fast.toml && git commit -m "docs: gate bicycle simulation profiles on declared evidence"`.

## Проверка покрытия спецификации и порядок исполнения

| Требование аудита | Task |
|---|---|
| F1: незавершённый balance API | A1 |
| F2: неприменяемый validity и потерянный radius | A2 |
| F3: восстановление педалирования | A3 + release gate C4 |
| F4: несколько опор и физические материалы | B1–B2 + C2–C3 |
| F5: привод и подвеска без подробной цепи | B3 + C1 |
| F6: суставные пределы/хват | A4–A5 + C3 |
| F7: динамика активного человека | A6 + C3 |
| F8: численная и экспериментальная достоверность | C1–C4 |
| Отделение объекта управления от алгоритма | B4; исключение anti-wheelie во всех plans |

Порядок: A1/A2 → A3 → C1; затем A4–A6 и B1–B4; C2 проверяет завершённые варианты, C3 начинается с протокола и реально доступных измерений; C4 выпускает только подтверждённый уровень. Не требуется ждать реальных данных, чтобы исправить шесть failures или подключить validity, но нельзя назвать результат экспериментально валидированным без этих данных.

Численные gates в этом плане являются начальными проектными требованиями, а не результатами выполненного аудита. Все задачи — изменения симуляции и её проверки; ни одна не подбирает anti-wheelie управляющее воздействие.