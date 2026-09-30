# Работоспособность райдера и достоверность наблюдений — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Завершить физический контракт райдера, сделать ограничения модели видимыми и воспроизводимо диагностировать восстановление педалирования.

**Architecture:** Сохранить MuJoCo-дерево тел и внутренние суставные приводы. Запросы баланса идут только через существующий allocator опор; монитор применимости читает те же входящие контактные состояния, что силовой расчёт. Отдельные маленькие модули отвечают за replay, суставные конверты и динамику активного усилия.

**Tech Stack:** Python 3.13, MuJoCo 3.12.0, NumPy, SciPy, pytest, стандартные dataclasses/JSON/TOML; зависимости из предоставленного wheelhouse.

**Spec:** `docs/superpowers/specs/2026-09-30-bikesim-fidelity-audit.md`, разделы 1–5, F1–F3, F6–F7.

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

Все пути `src/` ниже относительны корню проекта. Номера строк — до изменений, из исследованного архива.

| Файл | Ответственность |
|---|---|
| `src/bike_sim/physics/physical_config.py` | Численные параметры articulated-модели и их валидация |
| `src/bike_sim/sim/ride/rider_balance.py` | Только желаемая внутренняя реакция на ошибку положения |
| `src/bike_sim/sim/ride/rider_support.py` | Разложение запрошенного wrench по доступным опорам |
| `src/bike_sim/sim/ride/rider_control.py` | Внутренние суставные запросы, не внешние силы |
| `src/bike_sim/sim/research/validity.py` | Чистая проверка применимости модели |
| `src/bike_sim/sim/ride/model_status.py` — новый | Накопление причин invalid без изменения сил |
| `src/bike_sim/sim/ride/physical_observations.py` | Сериализация радиуса и контактных признаков |
| `src/bike_sim/sim/ride/physical_runtime.py` | Один вызов мониторинга за физический интервал |
| `src/bike_sim/sim/ride/physical_session.py`, `src/bike_sim/sim/ride/hud.py` | Сохранённый статус и показ предупреждения |
| `src/bike_sim/sim/research/environment.py` | Политика остановки или маркировки research episode |
| `src/bike_sim/validation/rider_replay.py` — новый | Повторяемый ввод, трассировка и диагноз coast/resume |
| `src/bike_sim/physics/rider_envelope.py` — новый | Явные суставные координаты, диапазоны и происхождение |
| `src/bike_sim/physics/rider_activation.py` — новый | Фильтр активного усилия и общий бюджет мощности |
| `src/bike_sim/mujoco/articulated_rider.py`, `builder.py` | Применение опционального joint-envelope profile |
| `src/bike_sim/sim/ride/rider_contacts.py` | Конечный хват и пассивное освобождение накопленной энергии |

Имеющиеся большие runtime/controller файлы не переписывать целиком. Новую математику выносить в указанные небольшие модули. Каждый task завершается отдельно проверяемым commit.

## Task A1: Завершить balance API и подключить его к внутренним приводам

**Files:** Modify `src/bike_sim/physics/physical_config.py:249–291`, `src/bike_sim/sim/ride/rider_support.py:78–109`, `src/bike_sim/sim/ride/rider_control.py:335–399`, `src/bike_sim/physics/rider_posture.py:7–15`; Test `tests/test_rider_balance.py` и `tests/test_rider_posture_tracking.py`.

**Interfaces:**
- Consumes: существующие `balance_force_request(controller, model, data, posture)`, `RiderCommand`, `RiderPosture`.
- Produces: `pedaling_support_targets(..., *, pitch_moment_nm=0., balance_force_on_rider_n=(0., 0., 0.)) -> tuple[dict, dict]`.
- В диагностике: `requested_balance_force_on_rider_n`, `total_requested_force_on_rider_n`, `horizontal_force_error_n`, `feasible`. Все силы этой диагностики — на райдера; возвращаемый словарь сил — на велосипед.

- [ ] **Step 1: Закрепить падающие тесты.** Существующие шесть tests уже воспроизводят отсутствие API. Добавить в тот же файл проверку вызова из controller и отсутствия root actuation; используется его существующая fixture `rig`:

```python
def test_compute_routes_balance_through_support_targets(rig, monkeypatch):
    from bike_sim.sim.ride.rider_control import RiderCommand
    import bike_sim.sim.ride.rider_balance as balance
    m, d, c = rig
    calls = []
    def request(*args):
        calls.append(True)
        return np.array([50., 0., 20.])
    monkeypatch.setattr(balance, 'balance_force_request', request)
    before_q = d.qpos.copy()
    before_v = d.qvel.copy()
    before_f = d.qfrc_applied.copy()
    torques = c.compute(m, d, RiderCommand(),
        contact_loads={'saddle': 440., 'front': 130., 'rear': 130., 'grip': 100.},
        support_available={n: True for n in ('saddle', 'front', 'rear', 'grip')})
    assert calls == [True]
    assert c.support_diagnostics['requested_balance_force_on_rider_n'] == [50., 0., 20.]
    assert not any(name.startswith('rider_root_') for name in torques)
    np.testing.assert_array_equal(d.qpos, before_q)
    np.testing.assert_array_equal(d.qvel, before_v)
    np.testing.assert_array_equal(d.qfrc_applied, before_f)
```

- [ ] **Step 2: Увидеть failure.** Run `python -m pytest tests/test_rider_balance.py -q --tb=short`. Expected: прежние AttributeError/TypeError, новый тест не видит balance call.

- [ ] **Step 3: Завершить контракт.** В `ArticulatedConfig` добавить синтетические параметры:

```python
posture_translation_k_n_m: float = 1500.
posture_translation_d_ns_m: float = 150.
posture_translation_limit_n: float = 400.
```

Нулевой stiffness/damping/limit разрешает выключить соответствующий запрос. Сохранить проверку finite/nonnegative. В `pedaling_support_targets` заменить расчёт остаточной силы следующим ядром; переменные `extra`, `arms`, `pedal`, `points`, `com` остаются из существующей функции:

```python
balance = array(balance_force_on_rider_n, 'balance force', (3,))
if abs(balance[1]) > 1e-12:
    raise ValueError('balance request must be planar')
target_force = np.array([balance[0], 0., weight_n + balance[2]])
if enabled[3]:
    extra[3, 0] = target_force[0] - np.sum(extra[:3, 0])
extra_moment = float(np.sum(arms[:, 2]*extra[:, 0] - arms[:, 0]*extra[:, 2]))
loads, diagnostics = gravity_support_targets(
    weight_n, com[0], points[:, 0], crank_x_m,
    pedal_fraction, bar_fraction, enabled,
    pitch_moment_nm=pitch_moment_nm-extra_moment,
    vertical_target_n=target_force[2]-float(np.sum(extra[:, 2])))
reactions = extra.copy()
reactions[:, 2] += np.array(list(loads.values()))
error = np.sum(reactions, axis=0)-target_force
# Existing vertical/moment/coasting residuals remain in diagnostics.
diagnostics.update(
    requested_balance_force_on_rider_n=balance.tolist(),
    requested_vertical_forces_n=loads,
    total_requested_force_on_rider_n=np.sum(reactions, axis=0).tolist(),
    horizontal_force_error_n=float(error[0]),
    feasible=bool(diagnostics['feasible'] and np.max(np.abs(error)) <= 1e-8))
```

Сохранить существующие поля моментной диагностики. Недоступной опоре не назначать усилие; при несовместимых требованиях сохранять ошибку, не создавать силу корня. В `compute` импортировать `balance_force_request`, вызвать ровно один раз и передать результат новым keyword в строке вызова allocator. `disabled`-ветвь по-прежнему возвращает нулевые суставные команды.

Документировать: `pelvis_offset_m=None` по-прежнему относится к support-following IK, но включённый translational balance отдельно запрашивает nominal support wrench. Для пассивного эксперимента translational gains равны нулю; это не скрытая кинематическая фиксация. Добавить в описание пресета явную отметку о новом feedback и его synthetic-параметрах.

- [ ] **Step 4: Проверить контракт и сохранение внутренности сил.** Run `python -m pytest tests/test_rider_balance.py tests/test_rider_posture_tracking.py tests/test_rider_contact_laws.py tests/test_physical_pedaling_regression.py -q`. Expected: PASS, включая семь balance tests. Затем replay задачи A3 — без требования воспроизвести старую неправильную траекторию побитно.
- [ ] **Step 5: Commit.**

```bash
git add src/bike_sim/physics/physical_config.py src/bike_sim/physics/rider_posture.py src/bike_sim/sim/ride/rider_control.py src/bike_sim/sim/ride/rider_support.py tests/test_rider_balance.py
git commit -m "fix: complete internal rider balance support contract"
```

## Task A2: Включить применимость модели во все режимы без второго силового расчёта

**Files:** Modify `src/bike_sim/sim/research/validity.py:8–24`, `src/bike_sim/sim/ride/physical_observations.py:8–43`, `src/bike_sim/sim/ride/physical_runtime.py:408–496`, `src/bike_sim/sim/ride/physical_session.py:181–214`, `src/bike_sim/sim/research/environment.py:151–185`, `src/bike_sim/sim/ride/hud.py`; Create `src/bike_sim/sim/ride/model_status.py`; Test `tests/test_model_status.py`.

**Interfaces:**
- Produces `channel_violations(channels: Mapping, maximum_compression_fraction: float = .15, maximum_linkage_error_m: float = .002) -> tuple[str, ...]`; старый `model_violations(sample, ...)` остаётся совместимой оболочкой.
- Produces `ModelStatus.observe(interval_id: int, time_s: float, channels: Mapping) -> None`, `ModelStatus.as_dict() -> dict`.
- `tires[side]` всегда содержит `unloaded_radius_m`, `multi_support`, `supports_multiple_contacts`, `outside_material_load_range`, `patches`. Последний capability для текущего backend равен false.
- Разделить `model_valid`, `numerically_valid`, `calibration_status`; отсутствие energy audit в preview означает `not_evaluated`, а не PASS.

- [ ] **Step 1: Добавить чистые tests для тихо пропускаемых случаев.**

```python
from bike_sim.sim.research.validity import channel_violations

def channels(**front):
    tire = dict(unloaded_radius_m=.35, penetration_m=.001,
                normal_load_n=400., multi_support=False,
                supports_multiple_contacts=False, patches=())
    return {'tires': {'front': dict(tire, **front), 'rear': dict(tire)}}

def test_compression_uses_unloaded_radius():
    assert 'front:tire_compression' in channel_violations(
        channels(penetration_m=.06))

def test_multisupport_is_invalid_for_single_support_backend():
    assert 'front:multi_support' in channel_violations(channels(multi_support=True))

def test_missing_radius_is_explicit_not_silently_accepted():
    c = channels()
    del c['tires']['front']['unloaded_radius_m']
    assert 'front:missing_radius' in channel_violations(c)
```

- [ ] **Step 2: Run** `python -m pytest tests/test_model_status.py -q`. Expected: ImportError для ещё отсутствующего `channel_violations`.

- [ ] **Step 3: Реализовать pure checker и accumulator.** Перенести имеющиеся проверки без потери configurable thresholds. Ветку multi-support сделать условной по capability; неизвестный backend не объявлять multi-contact capable. Новое обязательное поле радиуса брать из `runtime.tire.radii[side]`, а для native reference — из сохранённого номинального radius в topology, не из нулевого airborne effective radius.

```python
from dataclasses import dataclass, field
from collections import Counter
from bike_sim.sim.research.validity import channel_violations

@dataclass
class ModelStatus:
    counts: Counter = field(default_factory=Counter)
    first: dict | None = None
    last_interval: int = -1

    def observe(self, interval_id, time_s, channels):
        if interval_id <= self.last_interval:
            raise ValueError('model status requires one advancing interval')
        self.last_interval = interval_id
        reasons = channel_violations(channels)
        self.counts.update(reasons)
        if reasons and self.first is None:
            self.first = {'interval_id': interval_id, 'time_s': time_s,
                          'reasons': list(reasons)}

    def as_dict(self):
        return {'model_valid': not bool(self.counts),
                'first_model_violation': self.first,
                'model_violation_counts': dict(self.counts)}
```

В обычном runtime собрать channels как сейчас, затем вызвать monitor **перед** замораживанием `PhysicalSample` и включить status в channels. Для preview использовать сохранённые входящие tire diagnostics/contact snapshots и solved linkage residual до `mj_forward` следующего состояния; не смешивать `q_n` с endpoint normal loads. Не вызывать `compute_qfrc(..., advance=True)` ради HUD. Продвигать monitor ровно один раз, общий interval id — `sim.steps` до increment. Сбросить monitor вместе с episode/reset.

Research environment: добавить явный `stop_on_model_violation` в ExperimentConfig, default true для reference; при false продолжать с `model_valid=false`. Headless: сохранить статус и first cause в output; нарушение модели возвращает отдельный nonzero exit status 2, numerical exception — 1, нормальное завершение — 0. Preview предупреждает, но не меняет силы. Автономное продолжение invalid episode разрешено только как явно диагностическое.

- [ ] **Step 4: Run** `python -m pytest tests/test_model_status.py tests/test_research_quality.py tests/test_profile_contact.py tests/test_wheelie_detection.py -q`. Expected: PASS. Добавить smoke-тест сопоставления q/v между монитором on/off при одинаковом входе: `np.testing.assert_array_equal`; проверить reject повторного interval и очистку counts после reset.
- [ ] **Step 5: Commit** `git add src/bike_sim/sim tests/test_model_status.py && git commit -m "fix: enforce model applicability without changing dynamics"`.

## Task A3: Сделать coast/resume воспроизводимым диагностическим стендом

**Files:** Create `src/bike_sim/validation/rider_replay.py`, `tests/test_rider_replay.py`, `examples/research/rider_resume_flat.toml`, `examples/research/rider_resume_incline.toml`, `examples/research/rider_resume_physics.toml`; Modify `src/bike_sim/sim/ride/physical_runtime.py:449–456` только для недостающих диагностических полей; результаты не фиксировать как большие бинарные fixtures.

**Interfaces:**
- `build_sim(physics_path: str, track_path: str, timestep_s: float | None = None)` возвращает существующий RideSimulation через CLI resolve/build.
- `replay(sim, schedule: Callable[[float], RideControl], duration_s: float) -> list[dict]` использует `sim.step(control=...)`.
- `diagnostic_row(sample) -> dict` переносит данные одного интервала; `diagnose_resume(rows) -> str` возвращает один из `resumed`, `no_effort_request`, `support_unavailable`, `actuator_saturated`, `assist_gated`, `mechanically_stalled`, `mixed_or_unresolved` вместе с сохранёнными численными доказательствами в отчёте replay. Последняя категория честно запрещает считать root cause установленной.

- [ ] **Step 1: Добавить тест расписания и неизменности входов.**

```python
from bike_sim.sim.ride.control import RideControl
from bike_sim.validation.rider_replay import resume_schedule

def test_replay_has_explicit_drive_coast_resume_intervals():
    assert resume_schedule(.5).human_torque_nm == 20.
    assert resume_schedule(2.5).human_torque_nm == 0.
    assert resume_schedule(4.5).human_torque_nm == 20.
    assert resume_schedule(.5).motor_torque_nm == 0.
```

- [ ] **Step 2: Run** `python -m pytest tests/test_rider_replay.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать replay и sample adapter, не менять controller по догадке.**

```python
from bike_sim.sim.ride.control import RideControl
from bike_sim.cli.ride import parse_args, resolve_track, resolve_rider
from bike_sim.sim.ride.physical_session import build_physical_simulation
from bike_sim.sim.ride.physical_samples import plain

def resume_schedule(time_s):
    human = 20. if time_s < 2. or time_s >= 4. else 0.
    return RideControl(human_torque_nm=human, motor_torque_nm=0.)

def build_sim(physics_path, track_path, timestep_s=None):
    argv = ['--physics-config', physics_path, '--rider', 'articulated_planar',
            '--track', track_path]
    if timestep_s is not None:
        argv += ['--timestep', str(timestep_s)]
    args = parse_args(argv)
    return build_physical_simulation(resolve_track(args.track), args, resolve_rider(args))

def diagnostic_row(sample):
    c = sample.channels
    return {'time_s': sample.time_s, 'end_time_s': sample.end_time_s,
            'drive': plain(c['drive']), 'rider': plain(c['rider']),
            'support': plain(c['rider_support_targets']),
            'joint_terms': plain(c['rider_control']),
            'ik_saturation': plain(c['rider_ik_saturation']),
            'tires': plain(c['tires']), 'energy': plain(c['energy'])}

def replay(sim, schedule, duration_s):
    dt = float(sim.model.opt.timestep)
    count = round(duration_s/dt)
    if count < 1 or abs(count*dt-duration_s) > 1e-9:
        raise ValueError('duration must be a positive integer number of steps')
    rows = []
    for _ in range(count):
        sim.step(control=schedule(sim.time_s))
        rows.append(diagnostic_row(sim.physical.sample))
        if sim.crash is not None:
            break
    return rows
```

В `src/bike_sim/validation/rider_replay.py` добавить `main()` с argparse: `--physics-config`, `--track`, `--duration`, `--output`; JSON содержит source/config hashes, версию движка, все rows и диагноз. Вызов `python -m bike_sim.validation.rider_replay ...`. Track fixtures: 30 м, hardpack, плоскость и постоянный уклон 0.05; без препятствий. Диагностический physics profile — копия fast с `drive.shifting.enabled=false`, `drive.pedaling.rollback_brake=false`, сохранёнными массами/материалами и motor envelope.

Матрица причин: human-only; тот же прогон с `motor_torque_nm=40.`; затем torque-sensing assist; затем отдельно shifting; отдельно rollback brake. Не включать несколько новых факторов одновременно. Сохранять реальные `human_sensor_nm` и `motor_torque_nm`, а не выдавать `human_command_nm` за доставленный момент.

Критерий `resumed`: в окне 4–8 с интеграл фактической положительной мощности на шатуне > 0 и развёрнутый угол шатуна увеличился не менее чем на 2π. Этот критерий — тест работы педалирования, не требование финиша или заданного ускорения. `support_unavailable` требует одновременно запроса и отсутствия доступной опоры/gap; `assist_gated` — наличия передаваемого human torque и записанного штатного gate. Для mixed cases нельзя автоматически выбирать «виновника».

Обнаруженную причину исправлять отдельным commit в уже локализованном модуле. До этого не утверждать, что A1 автоматически устраняет откат. Если физически заданное усилие недостаточно, корректный результат — документированный stall, а не программная подкрутка колеса.

- [ ] **Step 4: Run** unit tests и два 8-секундных replay с fixed gear. Expected: тест adapter PASS; интеграционные результаты сохраняют диагноз, даже если физический сценарий ещё stall. Затем добавить конкретный regression test подтверждённой причины и добиться его PASS до выпуска reference-профиля. Без этого gate release закрыт.
- [ ] **Step 5: Commit** `git add src/bike_sim/validation/rider_replay.py tests/test_rider_replay.py examples/research/rider_resume_flat.toml examples/research/rider_resume_incline.toml examples/research/rider_resume_physics.toml && git commit -m "test: add traceable articulated rider coast resume rig"`.

## Task A4: Добавить явный профиль допустимых суставных углов

**Files:** Create `src/bike_sim/physics/rider_envelope.py`, `tests/test_rider_joint_envelope.py`; Modify `src/bike_sim/mujoco/articulated_rider.py:20–31`, `src/bike_sim/mujoco/builder.py`, `src/bike_sim/physics/physical_config.py`, `src/bike_sim/physics/resolution.py`; Create `examples/research/rider_joint_envelope_synthetic.json`.

**Interfaces:**
- `JointEnvelope(neutral_anatomical_rad: float, direction: int, minimum_anatomical_rad: float, maximum_anatomical_rad: float, provenance: str)`.
- `joint_q_range(envelope: JointEnvelope) -> tuple[float, float]`.
- `load_joint_envelopes(path: str) -> dict[str, JointEnvelope]`; profile обязан покрыть ровно девять внутренних rider joints, не root.
- `ArticulatedConfig.joint_envelope_path: str | None = None`; строку проверять отдельно от текущего цикла scalar-полей. None сохраняет legacy-профиль, metadata сообщает `joint_envelope_status='unspecified'`.

- [ ] **Step 1: Написать проверку смены знака и запрета root.**

```python
import pytest
from bike_sim.physics.rider_envelope import JointEnvelope, joint_q_range

def test_joint_zero_and_direction_are_not_assumed_anatomical():
    a = JointEnvelope(1.0, 1, 0.1, 2.4, 'synthetic-test')
    b = JointEnvelope(1.0, -1, 0.1, 2.4, 'synthetic-test')
    assert joint_q_range(a) == pytest.approx((-.9, 1.4))
    assert joint_q_range(b) == pytest.approx((-1.4, .9))
```

- [ ] **Step 2: Run** `python -m pytest tests/test_rider_joint_envelope.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать преобразование и интеграцию XML.**

```python
from dataclasses import dataclass
from math import isfinite

@dataclass(frozen=True)
class JointEnvelope:
    neutral_anatomical_rad: float
    direction: int
    minimum_anatomical_rad: float
    maximum_anatomical_rad: float
    provenance: str

    def __post_init__(self):
        values = (self.neutral_anatomical_rad,
                  self.minimum_anatomical_rad, self.maximum_anatomical_rad)
        if not all(isfinite(v) for v in values) or self.direction not in (-1, 1):
            raise ValueError('invalid joint coordinate convention')
        if not self.minimum_anatomical_rad < self.maximum_anatomical_rad:
            raise ValueError('empty joint range')
        if not self.provenance.strip():
            raise ValueError('joint range provenance is required')

def joint_q_range(envelope):
    a = (envelope.minimum_anatomical_rad-envelope.neutral_anatomical_rad)/envelope.direction
    b = (envelope.maximum_anatomical_rad-envelope.neutral_anatomical_rad)/envelope.direction
    return min(a, b), max(a, b)
```

JSON fixture использует только проверочные, не физиологически заявленные значения; пример выше назначается суставу в отдельном тестовом rig. Для реального reference профиль должен содержать анатомическую калибровку `q=0`, полученную из геометрии сегментов/позы, и диапазоны из измерений/явно названного набора антропометрии. Не распространять один угол колена на таз/плечо.

В XML ставить `limited=true`, `range=...` только для joints из profile. Проверить используемые единицы compiler и форматировать диапазоны в них, не передавать радианы в degree-compiler. Добавить мягкое пассивное сопротивление около края как производную сохраняемой potential; solver limit остаётся последним ограничителем. Его работу включить в существующий `joint_limits` ledger. IK должен отмечать clipping к envelope, а не молча менять anatomical range.

- [ ] **Step 4: Run** `python -m pytest tests/test_rider_joint_envelope.py tests/test_physical_topology_xml.py tests/test_compiled_mass_contract.py tests/test_rider_posture_tracking.py -q`. Expected: профиль ограничивает девять внутренних joints, root остаётся свободным, масса/инерция не меняется. Обязательные tests: зеркальная нога, выход target за диапазон, невалидный/неполный профиль, нулевая работа неподвижного limit.
- [ ] **Step 5: Commit** `git add src/bike_sim/physics src/bike_sim/mujoco tests/test_rider_joint_envelope.py examples/research/rider_joint_envelope_synthetic.json && git commit -m "feat: support explicit rider joint coordinate envelopes"`.

## Task A5: Сделать разрыв хвата силовым и пассивным

**Files:** Modify `src/bike_sim/sim/ride/rider_contacts.py:244–275`, `src/bike_sim/physics/physical_config.py:273–275`; Create `src/bike_sim/physics/grip_release.py`, `tests/test_grip_release.py`.

**Interfaces:**
- `release_if_overloaded(force_n: ndarray, old_energy_j: float, limit_n: float) -> tuple[ndarray, float, bool]` возвращает разрешённую силу, dissipated energy, released.
- `ArticulatedConfig.grip_pair_force_limit_n`, `grip_capture_distance_m`, `grip_capture_speed_mps`; единица силы относится к **паре** рук текущей planar-модели.
- Повторный capture допускается только по явному запросу программы райдера/контактов, при конечном расстоянии и скорости; разрыв не означает автоматическое приклеивание на следующем шаге.

- [ ] **Step 1: Написать test отсутствия отрицательных потерь и неподконтрольного хвата.**

```python
import numpy as np
from bike_sim.physics.grip_release import release_if_overloaded

def test_overload_breaks_grip_and_accounts_stored_energy():
    force, loss, released = release_if_overloaded(np.array([500., 0., 0.]), 3., 400.)
    np.testing.assert_array_equal(force, np.zeros(3))
    assert loss == 3.
    assert released
```

- [ ] **Step 2: Run** `python -m pytest tests/test_grip_release.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать минимальный cohesive break, не простое clipping силы.**

```python
import numpy as np

def release_if_overloaded(force_n, old_energy_j, limit_n):
    force = np.asarray(force_n, dtype=float)
    if force.shape != (3,) or not np.isfinite(force).all():
        raise ValueError('grip force must be a finite 3-vector')
    if not np.isfinite(old_energy_j) or old_energy_j < 0:
        raise ValueError('invalid stored grip energy')
    if not np.isfinite(limit_n) or limit_n <= 0:
        raise ValueError('grip limit must be positive')
    released = bool(np.linalg.norm(force) > limit_n)
    return (np.zeros(3), old_energy_j, True) if released else (force.copy(), 0., False)
```

Интеграция: сначала trial `_grip_step`; до применения qfrc проверить trial force. При break не использовать trial elastic state/energy; сохранить нулевую силу и xi, учесть **предыдущее** накопленное energy как release loss, защёлкнуть `enabled['grip']=False`. Не прибавлять одновременно trial loss и полную release loss. Повторный захват начинается с xi=0, выполняется только при расстоянии/скорости ниже порогов и явном enable. Для preview/probe новые состояния копируются, как существующие contact states. Ввести синтетический test profile с пределом 400 Н, capture distance 0.02 м, capture speed 0.2 м/с; эти числа не выдавать за измеренную способность человека.

- [ ] **Step 4: Run** `python -m pytest tests/test_grip_release.py tests/test_rider_contact_laws.py -q`. Expected: PASS. Дополнительно test probe не изменяет latch; capture не создаёт energy; equal/opposite wrench сохраняется; чрезмерная относительная скорость не даёт regrasp.
- [ ] **Step 5: Commit** `git add src/bike_sim/physics/grip_release.py src/bike_sim/physics/physical_config.py src/bike_sim/sim/ride/rider_contacts.py tests/test_grip_release.py && git commit -m "feat: add passive force limited rider grip release"`.

## Task A6: Отделить активное усилие от пассивной механики человека

**Files:** Create `src/bike_sim/physics/rider_activation.py`, `tests/test_rider_activation.py`; Modify `src/bike_sim/sim/ride/rider_control.py:444–512`, `src/bike_sim/physics/physical_config.py`, `src/bike_sim/sim/ride/physical_runtime.py:441–455`.

**Interfaces:**
- `activation_step(previous: ndarray, target: ndarray, dt_s: float, tau_s: float) -> ndarray`.
- `limit_positive_power(torque: ndarray, velocity: ndarray, limit_w: float) -> ndarray`.
- Активное состояние контроллера сбрасывается вместе с episode; повторный `advance=False` его не меняет.
- В channels добавляются `rider_active_request_nm`, `rider_active_delivered_nm`, `rider_positive_power_w`, `rider_activation_saturated`; passive damping/work записываются отдельно.

- [ ] **Step 1: Написать unit tests общего бюджета и полугруппы фильтра.**

```python
import numpy as np
from bike_sim.physics.rider_activation import activation_step, limit_positive_power

def test_total_positive_power_cap_does_not_cancel_braking():
    tau = np.array([20., 20., -5.]); speed = np.array([10., 10., 10.])
    result = limit_positive_power(tau, speed, 200.)
    assert np.maximum(result*speed, 0.).sum() <= 200. + 1e-10
    assert result[2] == -5.

def test_filter_depends_on_time_not_number_of_calls():
    zero, target = np.zeros(2), np.ones(2)
    whole = activation_step(zero, target, .01, .05)
    half = activation_step(zero, target, .005, .05)
    np.testing.assert_allclose(whole, activation_step(half, target, .005, .05))
```

- [ ] **Step 2: Run** `python -m pytest tests/test_rider_activation.py -q`. Expected: ImportError.

- [ ] **Step 3: Реализовать active-only filter и budget projection.**

```python
import numpy as np

def activation_step(previous, target, dt_s, tau_s):
    previous, target = np.asarray(previous, float), np.asarray(target, float)
    if previous.shape != target.shape or not np.isfinite([*previous, *target, dt_s, tau_s]).all():
        raise ValueError('invalid activation state')
    if dt_s <= 0 or tau_s < 0:
        raise ValueError('invalid activation time constants')
    return target.copy() if tau_s == 0 else target+(previous-target)*np.exp(-dt_s/tau_s)

def limit_positive_power(torque, velocity, limit_w):
    torque, velocity = np.asarray(torque, float), np.asarray(velocity, float)
    if torque.shape != velocity.shape or torque.ndim != 1:
        raise ValueError('power budget vector mismatch')
    if not np.isfinite([*torque, *velocity, limit_w]).all() or limit_w < 0:
        raise ValueError('invalid power budget')
    result = torque.copy()
    positive = result*velocity > 0
    power = float(np.sum(result[positive]*velocity[positive]))
    if power > limit_w:
        result[positive] *= limit_w/power
    return result
```

Сначала формируется target активных суставных усилий; затем activation filter; затем индивидуальные torque/speed и общий positive-power лимиты. Пассивный damper не фильтровать и не считать активной положительной мощностью. Текущая affine-damping запись actuator требует отдельного теста: величина, ограниченная этим helper, должна соответствовать фактическому активному усилию, а не только `data.ctrl`. Проверять solved actuator force и интервальную работу, поскольку изменение скорости внутри шага может нарушить оценку cap по входящей скорости. Превышение лимита считать diagnostic, а не исправлять endpoint qvel.

Ввести явно синтетический profile с tau=0.05 с, общим active-positive-power budget 600 Вт; сохранить legacy tau=0 как сравниваемый контроль. Эти параметры не являются рекомендацией физиологической мощности. Не добавлять feedback от wheelie label к человеку или мотору.

- [ ] **Step 4: Run** `python -m pytest tests/test_rider_activation.py tests/test_physical_pedaling_regression.py tests/test_rider_posture_tracking.py -q`. Expected: PASS. Повторить A3 со старым и новым профилем; документировать изменение движения и фактической мощности, не калибровать по желаемому финишу.
- [ ] **Step 5: Commit** `git add src/bike_sim/physics/rider_activation.py src/bike_sim/physics/physical_config.py src/bike_sim/sim/ride/rider_control.py src/bike_sim/sim/ride/physical_runtime.py tests/test_rider_activation.py && git commit -m "feat: model bounded rider active effort dynamics"`.

## Приёмка плана A

A1 и A2 — обязательные первые исправления. A3 даёт диагностический инструмент и отдельный gate исправления установленной причины; он не обещает заранее, что известный 8-секундный stall устранится одной правкой. A4–A6 дают включаемые физические ограничения, но их synthetic-параметры требуют калибровки из плана C.

В конце запустить весь pytest-suite в offline-окружении; отдельно сохранить названия пропущенных/долгих tests. Ни один новый тест не должен требовать подключения к сети или графического дисплея. Нельзя выпускать reference profile при неразобранном coast/resume regression, даже если unit tests новых helper проходят.