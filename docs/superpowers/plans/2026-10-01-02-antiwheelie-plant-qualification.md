# Anti-Wheelie Plant Qualification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Устранить необъяснённую потерю педалей и выпустить проверенный синтетический профиль велосипеда с райдером для исследования управления моторным моментом на целевой пересечённой трассе.

**Architecture:** Сначала воспроизвести пользовательский coast/contact failure на обычном physical runtime и исправить доказанную причину. Затем квалифицировать имеющиеся модели контакта и редуцированного привода независимыми механическими и численными проверками, после чего запускать один policy runner из плана A в viewer/headless/batch. Детальная цепь не развивается и не становится единственным эталоном.

**Tech Stack:** Python >=3.13, uv, MuJoCo, NumPy, SciPy, pytest, существующие JSON/CSV/replay инструменты; без новых runtime dependencies.

**Spec:** `docs/superpowers/specs/2026-10-01-antiwheelie-development-bench-design.md`

## Global Constraints

- Python >=3.13; every Python command uses `uv`; no new runtime dependencies.
- Work only in this checkout; preserve existing dirty/staged/untracked files and previous verification artifacts.
- Do not commit, change branches, or change default profiles without explicit user authorization.
- Do not implement or tune the user's anti-wheelie algorithm.
- Retain planar X-Z dynamics, articulated rider, mid-drive torque units, and the sensor/truth boundary.
- Do not extend detailed-chain physics; do not add external pitch stabilization or runtime qpos/qvel corrections.
- Keep energy residual limit 0.05 and existing model-scope rejection; failed/incomplete runs cannot pass release gates.
- Preserve the existing preview command; research runs opt in explicitly and use the same core as headless runs.
- All newly created evidence uses a new output directory, source/config hashes and runtime versions.
- Never describe synthetic verification as measured real-bicycle validation.

---

## Порядок и доказательства на старте

Выполнить **B1 первым**, до плана A. Затем A1–A5, затем B2–B4. Исходная ревизия планирования — `3f94874`; runtime в этой сессии не проверялся.

Новый пользовательский дефект: ноги уходят с педалей, велосипед не доезжает наверх. Последний найденный matching preview:

`output/ride/research_rough_uphill_extreme_articulated_effort_43d5c804a9146790/preview.csv`

Он имеет mtime 2026-09-30 23:53, но рядом нет source/config manifest. Поэтому это диагностическая зацепка, а не доказательство воспроизведения текущей ревизии:

| Время | Позиция | Наблюдение |
|---|---|---|
| 1.289 с | 3.890 м | Начинается накат, обе ноги ещё нагружены |
| 1.726 с | 5.016 м | Каденс −43 rpm, обе нагрузки нулевые, зазоры 21/69 мм |
| 2.998 с | 8.150 м | Обе нагрузки нулевые, зазоры 257/242 мм |
| 3.494–3.725 с | 9.281–9.763 м | Педалирование возвращается, контакты восстанавливаются |
| 27.811 с | 55.984 м | Максимальная достигнутая координата на уклоне 28%; обе ноги нагружены |
| 33.899 с | 55.776 м | Откат, затем закрытие viewer |

В этом логе переключений не было: всё время 34/51. Первый отрыв начинается на плоскости до начала roughness в 8 м. Не объявлять его следствием ступеней, переключений или «навсегда выключенного» контакта. Не считать последующий stall доказанным следствием раннего отрыва.

## Карта файлов

| Файл | Ответственность |
|---|---|
| `sim/ride/rider_control.py` | Физически реализуемые цели ног, posture/effort и диагностика |
| `physics/pedaling.py` | Намерение pedaling/coasting и остановки кривошипа |
| `sim/ride/rider_contacts.py` | Конечная платформа, односторонний контакт, трение |
| `validation/rider_replay.py` | Автоматический coast regression и существующий scripted resume |
| `validation/rider_factor_matrix.py` | Изоляция rider/motor/shift/rollback факторов |
| `physics/transmission_constraint.py`, `sim/ride/geometric_freehub.py` | Геометрия и единственная реакция идеального привода |
| `physics/distributed_tire.py`, `sim/ride/distributed_tire_forces.py` | Контакт нескольких опор |
| `validation/fidelity_sweep.py`, `validation/reference_release.py` | Сходимость и выпуск профиля |
| новый `validation/antiwheelie_bench.py` | Оркестрация существующих стендов, доказательства, paired summary |
| новые `examples/research/antiwheelie_candidate.toml`, `antiwheelie_reference.toml` | Кандидат и только после успешных gates — точный release config |

Все сокращённые пути в таблице начинаются с `src/bike_sim/`. Production model files изменять только по доказанным regression failures; не переписывать уже работающие баланс, суставные ограничения или multi-contact backend.

### Task B1: Воспроизвести и исправить потерю педалей, отдельно объяснить stall

**Files:**
- Modify: `src/bike_sim/validation/rider_replay.py:23`
- Modify: `src/bike_sim/sim/ride/rider_control.py:316`
- Modify if demonstrated necessary: `src/bike_sim/physics/pedaling.py:80`
- Modify if demonstrated necessary: `src/bike_sim/sim/ride/rider_contacts.py:157`
- Modify: `src/bike_sim/sim/ride/physical_runtime.py` — только экспорт дополнительной диагностики controller в sample
- Modify: `tests/test_physical_pedaling_regression.py:105`
- Modify: `tests/test_pedal_contact_stability.py:10`
- Modify: `tests/test_rider_replay.py`
- Create: `tests/test_coasting_contact_recovery.py`

**Interfaces:**
- Consumes: `build_sim(physics_path, track_path, timestep_s=None, *, physics_overrides=None)`, `replay(sim, schedule, duration_s)`, `RideControl()` с исходным pedelec/rider поведением.
- Produces: `automatic_schedule(time_s: float) -> RideControl`, CLI `rider_replay --mode automatic`, неизменяющий ни torque intent, ни automatic pedaling настройки.
- Produces: `run_automatic_coast_case(*, dt_s: float, duration_s: float = 5.) -> dict` в `validation/rider_replay.py`.
- Produces report keys: `duration_s`, `coast_entered`, `pedaling_resumed`, `max_both_unloaded_coast_s`, `max_coasting_gap_m`, `resume_crank_turns`, `resume_positive_work_j`, `model_valid`, `numerically_valid`, `first_support_loss`, `rows`.
- Produces per-side diagnostics: `actual_sole_position_m`, `target_sole_position_m`, `actual_crank_phase_rad`, `target_crank_phase_rad`, `crank_phase_error_rad`, actual/desired rate; дополнительно существующие gap/load/normal/tangent/stance/saturation.

- [ ] **Step 1: Зафиксировать baseline и запустить существующие узкие проверки.**

```bash
git status --short
git rev-parse HEAD
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_physical_pedaling_regression.py tests/test_pedal_contact_stability.py tests/test_rider_posture_tracking.py tests/test_rider_replay.py
```

Сохранить stdout, runtime versions и hashes профиля/трассы в `verification/antiwheelie-20261001/pedal-baseline/`. Не менять default profile и `dt` до reproduction. Старый тест `test_coasting_rider_holds_both_feet_using_only_internal_actuators` проверяет один `compute`, а не динамическое удержание; его green не закрывает дефект.

Запустить существующий headless без новой команды policy:

```bash
uv run bike-ride \
  --physics-config examples/research/viewer_physics_fast.toml \
  --rider articulated_planar \
  --track examples/research/rough_uphill_extreme.toml \
  --headless --duration 40 --decimate 1 --no-plots \
  --out output/pedal-loss-current-strict
```

Этот текущий путь пишет accounting, но пока не останавливается строго по всем research gates. Проверить summary/intervals, а не только exit code. Если gates нарушены раньше 1.7 с, сначала локализовать это нарушение; не интерпретировать дальнейшие силы как достоверные.

- [ ] **Step 2: Добавить physics-rate диагностику и точный automatic сценарий.**

```python
def automatic_schedule(time_s):
    _time(time_s)
    return RideControl()
```

`--mode automatic` использует этот schedule и переданный `physics-config` без overrides. Существующие `human-only`, `motor40`, `assist`, `shifting`, `rollback`, `open-loop` оставить как отдельные диагностические режимы. В `ArticulatedRiderController` сохранять target/actual координаты и rates в diagnostics после вычисления целей, не изменяя `data`. Для actual sole брать `data.site_xpos[self.soles[side]]`; для target sole брать результат той же forward-kinematics геометрии, которой получен leg target, в scratch data. Не восстанавливать target position из UI.

Считать support available как `in_platform and normal_load_n > 1.`. Записывать все bouts, подтверждать sustained loss после 0.20 с; это diagnostic persistence, не новая контактная сила. Для coasting обе ноги — stance; при pedaling одиночная ненагруженная return-foot нормальна. Отдельно считать оба unloaded во время active effort. Зазоры не заменять одним boolean `in_platform`.

Для `run_automatic_coast_case` использовать fast TOML и первые 5 с **исходной extreme-трассы**: ранний отрыв находится на плоскости, а последующее снижение скорости на начале подъёма возвращает pedaling. Не начинать сразу в готовом coast-state. Дополнительный isolated flat run проверяет удержание при накате, но не требует автоматического resume в фиксированный срок: на ровной дороге скорость может остаться выше resume threshold. Если automatic coast не наступает, report имеет `coast_entered=False`, и regression не проходит. Initial state/hash сохранять вместе с report.

Минимальная реализация runner и метрик в `rider_replay.py`:

```python
def run_automatic_coast_case(*, dt_s, duration_s=5.):
    from bike_sim.sim.research.quality import energy_quality
    from bike_sim.sim.ride.physical_session import configuration_metadata
    sim = build_sim(
        'examples/research/viewer_physics_fast.toml',
        'examples/research/rough_uphill_extreme.toml', timestep_s=dt_s)
    metadata = configuration_metadata(sim)
    rows = replay(sim, automatic_schedule, duration_s)
    coast_entered = False
    resumed = False
    loss_duration = 0.
    maximum_loss = 0.
    maximum_gap = 0.
    rotation = 0.
    positive_work = 0.
    first_loss = None
    for row in rows:
        interval = row['end_time_s'] - row['time_s']
        drive = row['drive']
        coasting = drive['rider_mode'] == 'coasting'
        coast_entered = coast_entered or coasting
        pedals = [row['rider'][side + '_pedal'] for side in ('front', 'rear')]
        available = any(pedal['in_platform'] and pedal['normal_load_n'] > 1.
                        for pedal in pedals)
        if coasting:
            maximum_gap = max(maximum_gap, *(pedal['gap_m'] for pedal in pedals))
        loss_duration = loss_duration + interval if coasting and not available else 0.
        maximum_loss = max(maximum_loss, loss_duration)
        if loss_duration >= .20 and first_loss is None:
            first_loss = row['end_time_s'] - loss_duration
        if coast_entered and drive['rider_mode'] == 'pedaling':
            resumed = True
            rate = drive['crank_rad_s']
            rotation += rate * interval
            positive_work += max(0., drive['human_sensor_nm'] * rate) * interval
    return {
        'metadata': metadata, 'rows': rows, 'duration_s': sim.time_s,
        'coast_entered': coast_entered, 'pedaling_resumed': resumed,
        'max_both_unloaded_coast_s': maximum_loss,
        'max_coasting_gap_m': maximum_gap,
        'resume_crank_turns': rotation / (2. * pi),
        'resume_positive_work_j': positive_work,
        'first_support_loss': first_loss,
        'model_valid': sim.physical.model_status.as_dict()['model_valid'],
        'numerically_valid': bool(rows) and all(
            energy_quality(row['energy']).acceptable for row in rows),
    }
```

- [ ] **Step 3: Написать интеграционный red-test на наблюдаемую проблему.**

```python
import pytest
from bike_sim.validation.rider_replay import run_automatic_coast_case


@pytest.mark.slow
def test_automatic_coasting_keeps_reachable_pedals_and_resumes_work():
    report = run_automatic_coast_case(dt_s=.0003125, duration_s=5.)
    assert report['duration_s'] == pytest.approx(5.)
    assert report['coast_entered']
    assert report['max_both_unloaded_coast_s'] <= .20
    assert report['max_coasting_gap_m'] <= .02
    assert report['pedaling_resumed']
    assert report['resume_positive_work_j'] > 0.
    assert report['model_valid']
    assert report['numerically_valid']
```

Порог 20 мм отделяет observed 0.24–0.26 м от необходимой миллиметровой податливости; это authored criterion раннего coast-эпизода. Не переносить его на прыжки или активный return stroke. Fine-dt integration test отделяет логическую ошибку от пригодности fast timestep; исходный 1.25 мс обязательно воспроизводится Step 1 и проверяется отдельной сходимостью. Для scripted resume отдельно требовать `resume_crank_turns >= 1.0` и положительную работу, используя уже существующий `resume_evidence` и `tests/test_rider_replay.py::test_diagnosis_requires_actual_power_and_full_crank_turn`.

Если тот же случай при 1.25 мс не проходит numerical gate, red сохраняется как доказательство непригодного timestep; дополнительно выполнить 0.625/0.3125 мс, не увеличивать damping/силы для сокрытия ошибки. Расширить `test_pedal_contact_stability` этими тремя `dt`, отдельно фиксируя config `pedal_c_ns_m=100` из fast profile. Его fixture должен принимать config параметром, иначе сравнивается другой закон контакта.

- [ ] **Step 4: Проверить первую конкретную гипотезу и сделать один минимальный fix.**

Гипотеза: `PedalingPolicy` замораживает goal phase остановки, фактический crank успевает развернуться, а `_coasting_target_state` переносит воображаемые педали в замороженную фазу; ноги следуют им вместо физических педалей. Проверка требует одновременного роста phase/Cartesian target error **до** потери нагрузки. Нулевые saturation flags в старом CSV не доказывают достижимость правильной цели.

Сравнить диагностические траектории actual/desired phase, target-vs-actual sole, pedal angular motion, limb torque, friction saturation и motor/freehub work за 1.2–2.1 с. Если подтверждается именно target mismatch, первым проверить такую локальную замену `_coasting_target_state`:

```python
def _coasting_target_state(self, model, data, command):
    import mujoco
    target = self.coasting_target_data
    target.qpos[:] = data.qpos
    target.qvel[:] = data.qvel
    target.qvel[self.crank_spin_dof] = command.crank_target_rate_rad_s
    target.qvel[self.pedal_spin_dofs] = -command.crank_target_rate_rad_s
    mujoco.mj_kinematics(model, target)
    return target
```

Здесь цели положения используют текущую геометрию, а конечное торможение кривошипа выполняют внутренние суставные усилия через velocity tracking. Замороженную desired phase оставить диагностическим намерением, не перемещать ею физическую платформу или goal далеко от неё. Это **кандидат исправления при подтверждённой гипотезе**, не установленная причина и не разрешение принять patch без интеграционного теста.

Дополнить regression проверкой, что scratch `qpos` соответствует actual crank, а live `qpos/qvel` не изменились после `compute`. Проверить остановку cranks, отсутствие неограниченного reverse spin, сохранение внутренних реакций/energy. Если вместо target mismatch доказан contact instability, исправлять только контактный integration/force defect, подтверждённый isolated pedal fixture. Если доказана недостижимая поза, корректировать bounded posture/leg target с существующими IK/envelope; не повышать torque/gap limits произвольно. Не выполнять несколько гипотетических исправлений одновременно.

- [ ] **Step 5: Отдельно разобрать позднюю остановку около 56 м.**

Для нового exact run построить окно 26–34 с: actual crank/human/motor torque, реальная human work, support loads, rear slip, brake/rollback state, assist stall timeout, grade и COM/posture. Сравнить конфиг с одной отключённой policy за раз через `rider_factor_matrix`: rollback, затем automatic shifting; не изменять одновременно torque curve и rider strength.

Классифицировать результат existing `reference_release.classify_outcome`. Для steady supported segment использовать фактические силу тяги, grade resistance и скорость; не применять к удару о корень статическую формулу как полный закон. Если delivered rider torque исчезает при reachable loaded stance — сделать второй минимальный regression на этот interval и исправлять источник передачи усилия. Если тяги/сцепления недостаточно — показать численный balance и честный `physical_stall`/`loss_of_traction`. Исправление раннего coast не считается доказательством проезда подъёма.

- [ ] **Step 6: Подтвердить исправление исходным пользовательским запуском.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_physical_pedaling_regression.py tests/test_pedal_contact_stability.py tests/test_rider_posture_tracking.py tests/test_rider_replay.py tests/test_coasting_contact_recovery.py tests/test_system_momentum.py
uv run bike-ride --physics-config examples/research/viewer_physics_fast.toml --rider articulated_planar --track examples/research/rough_uphill_extreme.toml
```

Сначала targeted tests, затем GUI с сохранением новой записи. Если fast timestep физически непригоден, показать это по convergence, предложить новый явный candidate profile и не менять старый default без разрешения. На плоском/допустимом подъёме обязателен работающий rider и восстановление positive work; на extreme фиксируется физически объяснимый исход. Незакрытую причину не обозначать «expected rider fall» ради завершения задачи.

Deliverable: воспроизводимый regression, доказанная причина и локальный fix, проверенный coast/resume и отдельная диагностика climb stall. После B1 выполнить план A.

### Task B2: Проверить существующие модели сил и замкнуть сбор доказательств

**Files:**
- Create: `src/bike_sim/validation/antiwheelie_bench.py`
- Create: `tests/test_antiwheelie_bench.py`
- Modify: `src/bike_sim/validation/rider_factor_matrix.py`
- Modify: `src/bike_sim/validation/drive_suspension_rig.py`
- Modify: `tools/compare_transmissions.py:92`
- Source fixes only when demonstrated: `src/bike_sim/physics/transmission_constraint.py`, `src/bike_sim/sim/ride/geometric_freehub.py`, `src/bike_sim/physics/distributed_tire.py`, `src/bike_sim/sim/ride/distributed_tire_forces.py`

**Interfaces:**
- Consumes: existing `load_transfer_rig`, `shaft_ratio_rig`, `drive_suspension_rig`, `run_contact_resolution`, `run_matrix`, source/config metadata.
- Produces: `evidence_matches(record: dict, expected: dict) -> bool`, `run_mechanics(output_dir: Path, physics_profile: Path) -> dict` в `antiwheelie_bench.py`.
- Report schema: `schema_version=1`, `source_sha256`, `configuration_sha256`, `versions`, `checks: list[dict]`, `passed: bool`; check содержит `name`, `passed`, `artifact`, `sha256`, `error`.

- [ ] **Step 1: Запретить reuse старых результатов другой конфигурации.**

```python
from bike_sim.validation.antiwheelie_bench import evidence_matches


def test_evidence_requires_source_config_and_runtime_match():
    expected = {
        'source_sha256': 'source-a',
        'configuration_sha256': 'config-a',
        'versions': {'python': '3.13.5', 'mujoco': '3.12.0'},
    }
    assert evidence_matches(dict(expected), expected)
    assert not evidence_matches(dict(expected, source_sha256='old'), expected)
    assert not evidence_matches(dict(expected, configuration_sha256='old'), expected)
    assert not evidence_matches({}, expected)
```

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_antiwheelie_bench.py tests/test_compare_transmissions.py
```

Реализовать:

```python
def evidence_matches(record, expected):
    keys = ('source_sha256', 'configuration_sha256', 'versions')
    return all(expected.get(key) is not None
               and record.get(key) == expected[key] for key in keys)
```

Canonical source hash брать `validation.environment.source_fingerprint` от того же `src/bike_sim`, что `configuration_metadata`; не использовать разные roots. В `compare_transmissions --reuse` проверять также track, seed, declared horizon и inputs hashes. Несовпадение — отказ reuse с перечнем полей, новый run в новой директории. Test: метрики старого run не принимаются даже если названия сценария/передачи совпадают.

- [ ] **Step 2: Выполнить механические проверки без усложнения цепи.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_compiled_mass_contract.py tests/test_physical_topology_xml.py tests/test_system_momentum.py tests/test_load_transfer_reference.py tests/test_plant_torque_contract.py tests/test_geometric_transmission.py tests/test_drive_suspension_rig.py tests/test_distributed_tire_kernel.py tests/test_distributed_tire_integration.py
uv run python -m bike_sim.validation.drive_suspension_rig --output verification/antiwheelie-20261001/mechanics/drive-suspension.json
uv run python -m bike_sim.validation.contact_resolution --output verification/antiwheelie-20261001/mechanics/contact
uv run python -m bike_sim.validation.rider_factor_matrix --output verification/antiwheelie-20261001/mechanics/rider
```

Стенды обязаны подтверждать: массу/CoM, свободный pitch, равные внутренние реакции, отсутствие ускорения общего CoM в воздухе от внутренних усилий, правильный torque ratio/freewheel, статические нагрузки на уклоне, отсутствие duplicate native tire contact, конечный grip и joint/power budgets.

Для static rigid-equivalent ориентир уже реализован `quasistatic_front_load`:

```python
from bike_sim.validation.load_transfer import quasistatic_front_load


def test_rigid_reference_reacts_to_acceleration_and_forward_com():
    baseline = quasistatic_front_load(100., 1.2, .45, .9, 0.)
    accelerating = quasistatic_front_load(100., 1.2, .45, .9, 0., 1.)
    forward = quasistatic_front_load(100., 1.2, .55, .9, 0.)
    assert accelerating < baseline < forward
```

Это проверка знака/баланса в ограниченном стенде; full-rider/rough transient не обязан совпадать с rigid formula.

- [ ] **Step 3: Локализовать geometric failure по первому плохому интервалу.**

Если fresh reproduction вновь даёт numerical failure, сохранить окно ±0.1 с с существующими `transmission_gap_m`, `transmission_tension_n`, `transmission_constraint_defect_m`, `transmission_reaction_error_n`, `transmission_interval_work_j`, energy breakdown и rider/contact events. Диагностика не добавляет новую force component.

Порядок решения, каждая строка требует отдельного red/green regression:

| Наблюдение | Следующее действие | Критерий принятия |
|---|---|---|
| Jacobian не совпадает с finite-difference virtual displacement | Исправить geometry/sign в `transmission_constraint.py` | Existing geometric tests + новые позы/gear ratios проходят |
| Constraint reaction отличается от `-tension * Jacobian` | Исправить selection/sign/index mapping в `geometric_freehub.py` | Reaction oracle error убывает до floating-point/solver tolerance |
| Discrete defect/work убывает при dt refinement | Проверить меньший точный dt без изменения closure/material | Полные declared windows и energy gate проходят |
| Failure совпадает с rider/contact event и сохраняется без drive torque | Исправить доказанный rider/contact defect в его модуле | Isolated fixture + полный исходный repro проходят |
| Reduction не имеет сходимости после исключения implementation defects | Зафиксировать model-form rejection и разработать отдельную power-consistent reduction в рамках этой незакрытой задачи | Новый mathematical contract и regression должны пройти те же gates; профиль пока не выпускается |

Последняя ветка — явный риск плана: по статическому аудиту нельзя честно написать универсальный исправляющий patch. Не подставлять ad hoc anti-squat force, не ослаблять residual, не объявлять detailed chain измеренным эталоном. В реализации сначала получить минимальный witness, затем согласованно обновить математический контракт задачи; задача остаётся незавершённой до рабочего редуцированного привода.

- [ ] **Step 4: Проверить целевые contact cases и составить report.**

Для 4/6 см ступеней, двух опор, edge unload и recontact использовать существующий distributed backend. Plain compliant backend допускается как fast approximation только если показывает те же целевые выходы в declared scope; `multi_support` invalid нельзя подавлять. Дорожное `hardpack` имеет собственные peak/sliding friction, TOML `mu=1.1` не означает фактическое постоянное сцепление 1.1.

`run_mechanics` вызывает существующие стенды, сохраняет их результаты без переупаковки ошибок в PASS и строит checks. Непустой список и `all(check['passed'] is True for check in checks)` обязательны; отсутствие обязательного именованного стенда — fail. Перед записью SHA проверять существование файла. Сохранить traceback и продолжить независимые стенды при ошибке.

- [ ] **Step 5: Targeted regressions и review checkpoint.**

Повторять только затронутые Step 2 tests плюс новый witness. Принять задачу, когда работают подходящие модели сил, известны их границы и создан source-matched report. Нельзя «закрыть» задачу лишь отчётом, в котором все физические кандидаты отказали.

### Task B3: Выбрать точный профиль по сходимости и сценариям

**Files:**
- Create: `examples/research/antiwheelie_candidate.toml`
- Modify: `src/bike_sim/validation/fidelity_sweep.py`
- Modify: `src/bike_sim/validation/reference_release.py`
- Modify: `src/bike_sim/validation/antiwheelie_bench.py`
- Modify: `tests/test_fidelity_sweep.py`
- Modify: `tests/test_reference_release.py`
- Modify: `tests/test_long_episode.py`

**Interfaces:**
- Consumes: candidate profile, existing `compare_metrics`, `select_verified_resolution`, `release_gates`, mechanics evidence B2.
- Produces: `assess_run(summary: dict, *, required_duration_s: float, minimum_progress_m: float = 0.) -> dict` в `antiwheelie_bench.py`.
- Produces CLI: `uv run python -m bike_sim.validation.antiwheelie_bench --physics-config PATH --output DIR --phase {mechanics,qualification,paired,release}`.
- Produces exact tested candidate tuples `(dt_s, dx_m, station_count, tire_backend, transmission_model)` with separate time/road/stations pass flags.

- [ ] **Step 1: Не позволять раннему падению проходить как 15-секундный прогон.**

```python
from bike_sim.validation.antiwheelie_bench import assess_run


def test_early_valid_crash_is_not_a_completed_long_run():
    summary = {
        'duration_s': .3, 'progress_m': .1,
        'outcome': 'crash:loop_out', 'numerically_valid': True,
        'model_status': {'model_valid': True},
    }
    result = assess_run(summary, required_duration_s=15., minimum_progress_m=5.)
    assert not result['accepted']
    assert 'incomplete_horizon' in result['reasons']
```

Implement `assess_run` без inference из exit code:

```python
def assess_run(summary, *, required_duration_s, minimum_progress_m=0.):
    from bike_sim.physics.checks import scalar
    required_duration_s = scalar(required_duration_s, 'required duration', positive=True)
    minimum_progress_m = scalar(minimum_progress_m, 'minimum progress', minimum=0.)
    duration = scalar(summary.get('duration_s', 0.), 'actual duration', minimum=0.)
    progress = scalar(summary.get('progress_m', 0.), 'actual progress')
    reasons = []
    if summary.get('numerically_valid') is not True:
        reasons.append('numerical_invalid')
    if summary.get('model_status', {}).get('model_valid') is not True:
        reasons.append('model_invalid')
    if duration + 1e-9 < required_duration_s:
        reasons.append('incomplete_horizon')
    if progress < minimum_progress_m:
        reasons.append('insufficient_progress')
    if (summary.get('outcome') or '').startswith('crash:'):
        reasons.append('crash')
    if summary.get('operator_intervention') is True:
        reasons.append('operator_intervention')
    return {'accepted': not reasons, 'reasons': reasons}
```

Добавить тесты missing keys, NaN/inf, negative required horizon. Это long-run gate, не общий запрет исследовать физические crashes.

Заменить слабый `test_fifteen_second_episode_completes`: использовать фиксированную длинную плоскую трассу, articulated rider, умеренную тягу и подтверждённый профиль, требовать **15 с**, отсутствие crash/invalid, progress >=5 м и положительную actual rider work при заявленном pedaling. Существующий generated stress test сохранить отдельно и переименовать по проверяемому свойству (valid termination), а не по недоказанной длине run.

- [ ] **Step 2: Создать явный candidate, включающий ограничения райдера.**

Взять полный `reference_candidate_experimental.toml` как основу, а не сокращённый TOML с неявными default differences. Из user fast сохранить 0 speed, 34/51, human 20 Nm, 85 Nm/600 W motor, coasting/effort settings. В candidate включить существующие joint envelope, grip force limit, activation и общий power limit из experimental reference; их происхождение — synthetic. Первую квалификацию проводить fixed gear/без rollback, комбинированный пользовательский режим — отдельный фактор после неё.

Ключевой блок candidate:

```toml
physics_mode = "physical"
drive_mode = "articulated_effort"
timestep_s = 0.000625
closure_time_constant_s = 0.0025
initial_speed_mps = 0.0

[drive]
transmission_model = "geometric_ideal_mid_drive"
human_torque_nm = 20.0

[tires]
backend = "distributed_2d_reference"
surface_mode = "track"

[tires.distributed]
station_count = 256
calibration_status = "experimental_unvalidated"

[articulated]
activation_tau_s = 0.05
active_positive_power_limit_w = 600.0
grip_pair_force_limit_n = 400.0
joint_envelope_path = "rider_joint_envelope_synthetic.json"
```

Это блок изменений в **полной** копии config, не готовый released profile и не назначение выбранных цифр правильными. Если B1/B2 доказали другое допустимое сочетание, переносить именно измеренное сочетание, записав основание. Старый `viewer_physics_fast.toml` не изменять.

- [ ] **Step 3: Выполнить orthogonal convergence с одинаковыми входами.**

```bash
uv run python -m bike_sim.validation.fidelity_sweep \
  --physics-config examples/research/antiwheelie_candidate.toml \
  --backend distributed_2d_reference --transmission geometric_ideal_mid_drive \
  --stations --jobs 3 --output verification/antiwheelie-20261001/fidelity
```

Existing time grid: 1.25/0.625/0.3125 мс; road grid: 10/5/2.5 мм; stations: 128/256/512. Сначала читать реальный CLI и подтвердить, что report делает **независимые** axes, а не только заменяет backend. Физический closure 2.5 мс, геометрия, stiffness, rider/motor programs, seed и начальное состояние неизменны внутри time sweep.

Если finest pair не сходится, добавить один следующий уровень **только для проваленной оси/случая**: dt 0.15625 мс, dx 1.25 мм либо 1024 stations; не перемножать сразу всю матрицу. Ввести CLI `--time-steps`, `--road-steps`, `--station-counts` с проверкой положительности, не менее трёх упорядоченных уникальных значений, фиксированием остальных параметров. Каждый axis report должен содержать фактические значения, не константы по умолчанию.

Критерии уже заданы в `compare_metrics`: loads/impulses 2%, travel 1 мм, peak pitch rate 5%, event time 5 мс, совпадение event sequence, full horizon, scope и energy. На физически terminal сценариях сравнить совпадение причины и времени события; не выдавать обрезанные траектории за полную интеграторную сходимость.

State fingerprints в temporal axis должны включать qpos/qvel, passive material memory, drivetrain boundary/state и initialization inputs. Проверить, что существующая сборка не приравнивает два независимых equilibrium solve только по initial-speed. Не передавать material state между разными tire models; их сравнивать через отдельную physical initialization agreement.

- [ ] **Step 4: Выполнить лестницу сценариев до exact 100 м.**

```bash
uv run python -m bike_sim.validation.reference_release \
  --physics-config examples/research/antiwheelie_candidate.toml \
  --scenarios flat grade_5pct grade_12pct grade_20pct grade_28pct grade_35pct bump crest step_4cm step_6cm two_support recontact coast_resume posture_seated posture_standing posture_forward posture_rearward extreme_100m \
  --output verification/antiwheelie-20261001/scenarios
```

Existing reference matrix использует собственный controlled torque schedule; дополнительно обязательно exact assist workflow через A:

```bash
uv run bike-ride --research --headless \
  --physics-config examples/research/antiwheelie_candidate.toml \
  --rider articulated_planar --track examples/research/rough_uphill_extreme.toml \
  --duration 90 --sensor-period .005 --seed 17 \
  --out output/antiwheelie-qualification-exact
```

Повторить exact input поведение с automatic shifting/rollback, включёнными как в исходном пользовательском профиле, в отдельном candidate variant. Профиль нельзя выпускать с неподтверждённой комбинацией, проверив лишь fixed-gear subset.

Для каждого препятствия сохранить достигнутую координату и traversal witness: step test считается выполненным только если колесо достигло/пересекло declared edge, а не остановилось раньше. Для grade case нужно действительно достичь участка уклона. `extreme_100m` обязан получить finish или доказанный физический outcome; `duration_limit` без объяснения остановки, contact bug, model invalid и numerical error — незавершённая квалификация. Реальный физический crash допустим как scenario result, но не заменяет long stable gate.

- [ ] **Step 5: Проверить воспроизводимость и выбрать самый быстрый прошедший tuple.**

Запустить два одинаковых seed/input runs и replay, сравнить observations/commands/outcomes/state. Нужен fresh new output для каждого. Использовать existing `select_verified_resolution`, передавая только measured candidates со всеми тремя axes и scope coverage. Если station axis неприменим к single-contact backend, это явно объявленный другой scope, не фиктивный station pass для rough-track release.

Измерить CPU wall time, simulated seconds, real-time factor, peak memory и размер artifacts. Rendering 15/30/60 FPS не меняет outputs. Сначала уменьшать rendering/logging frequency; после оптимизаций, затронувших execution, повторить parity regression. Не обещать real time до измерения.

Deliverable: хотя бы один точный прошедший tuple для целевой области, successful 15-секундный rider run и объяснимый результат exact extreme scenario. Если пока нет прошедшего tuple, B3 остаётся незавершённой; `experimental` нельзя переименовать в `reference`.

### Task B4: Парное сравнение политик, отчёт и выпуск рабочего стенда

**Files:**
- Modify: `src/bike_sim/validation/antiwheelie_bench.py`
- Modify: `src/bike_sim/validation/reference_release.py:55`
- Modify: `src/bike_sim/sim/research/metrics.py`
- Modify: `src/bike_sim/sim/research/environment.py`
- Modify: `tools/research_batch.py`
- Modify: `tests/test_antiwheelie_bench.py`
- Modify: `tests/test_episode_metrics.py`
- Modify: `tests/test_reference_release.py`
- Create after gates: `examples/research/antiwheelie_reference.toml`
- Create after gates: `verification/antiwheelie-20261001/release.json`
- Modify: `README.md`, `docs/ANTI_WHEELIE.md`, `docs/RIDE.md`, `docs/RESEARCH_COMPLETION.md`

**Interfaces:**
- Produces `pair_key(record: dict) -> tuple[str, int, str]`, `paired_summary(records: list[dict]) -> dict` в `antiwheelie_bench.py`.
- Pair record fields: `scenario_id`, `seed`, `plant_and_inputs_sha256`, `policy_id`, `metrics`, `operator_intervention`.
- Pair key исключает policy identity, но включает exact plant, terrain, initial state, demand/rider programs и sensor clocks/noise seed через `plant_and_inputs_sha256`.
- Metrics сохраняют старые поля и добавляют `demand_integral_nms`, `operator_intervention`, per-event counts/durations и outcome class. `motor_pass_fraction` остаётся совместимым, но не используется как общий score.

- [ ] **Step 1: Написать тест, что нулевой момент не выигрывает за счёт отсутствия движения.**

```python
from bike_sim.validation.antiwheelie_bench import paired_summary


def test_pair_report_keeps_progress_and_does_not_rank_zero_as_success():
    shared = {
        'scenario_id': 'flat_start', 'seed': 17,
        'plant_and_inputs_sha256': 'same', 'operator_intervention': False,
    }
    records = [
        dict(shared, policy_id='passthrough', metrics={
            'progress_m': 10., 'duration_s': 15., 'outcome': 'duration',
            'numerically_valid': True, 'model_status': {'model_valid': True},
            'wheelie': {'wheelie_time_s': .2}}),
        dict(shared, policy_id='zero', metrics={
            'progress_m': 0., 'duration_s': 15., 'outcome': 'duration',
            'numerically_valid': True, 'model_status': {'model_valid': True},
            'wheelie': {'wheelie_time_s': 0.}}),
    ]
    report = paired_summary(records)
    assert report['pairs'][0]['policies']['zero']['progress_m'] == 0.
    assert report['pairs'][0]['policies']['passthrough']['progress_m'] == 10.
    assert 'winner' not in report['pairs'][0]
```

Существующее имя duration в `WheelieTracker.metrics` — `wheelie_time_s`; сохранить его без дублирующего alias. Добавить tests mismatched input hashes, missing baseline, unequal coverage, invalid model и operator brake; ни один такой pair не попадает в comparable count.

- [ ] **Step 2: Реализовать report и независимый demand denominator.**

```python
def pair_key(record):
    return (record['scenario_id'], record['seed'],
            record['plant_and_inputs_sha256'])
```

`paired_summary` группирует по этому ключу; сохраняет все policies и отдельный список exclusions с причинами, не удаляет неудачные runs. Пропускной candidate не сравнивается с baseline другой длины, конфигурации или terrain. Не вводить composite reward и не подбирать anti-wheelie thresholds.

Для внешнего `DemandProgram` интеграл вычислять по физическим временам независимо от команды policy:

```python
if self.demand is not None:
    self.demand_integral_nms += self.demand.at(sample.time_s) * sample.dt_s
```

Инициализировать в `_begin_episode`, добавить в `episode_metrics` и reset tests. Для pedelec — `None`, а не придуманный постоянный запрос; `drive.motor_request_nm`, policy setpoint/ceiling и delivered torque сохранять раздельно. При наличии rotor lag delivered/requested может кратковременно превышать единицу; не clip telemetry ради score.

Для оценки причин использовать парные external-demand и time-based rider programs, а исходный automatic assist run показывать отдельно: его внутренние coasting/rollback transitions могут изменяться из-за траектории политики. Это допустимая closed-loop реакция, а не строго одинаковая delivered human input.

- [ ] **Step 3: Выполнить paired holdout без разработки алгоритма.**

Через `antiwheelie_bench --phase paired` выполнить passthrough/zero/fixed-limit plumbing на committed `examples/research/eval/manifest.json` и exact extreme track. Сохранять seed/config/source, attempted/completed/valid counts, progress, load margin, front-lift/wheelie/flight events, loop-out/endo, rider-contact loss, stall/rollback, actuator delay и torque. Fixed-limit demo нужен для проверки причинной связи command→torque→motion, не для заявления «anti-wheelie работает».

Часть case/seed выбрать holdout до изменений: committed eval seeds оставить evaluation-only, локальный flat/coast witness использовать для fix. Не менять evaluation terrain после просмотра результата, чтобы «улучшить» статистику. Articulated rider cases 60/80/100 кг и front/neutral/rear posture задать существующим RiderSpecs/RiderProgram, отклоняя недопустимую геометрию явно. Сформировать `report.json` и читаемый `report.md` из тех же данных.

- [ ] **Step 4: Выпустить профиль только с проверенным evidence.**

`--phase release` читает fresh mechanics/qualification/paired reports, проверяет их hashes и source/config/runtime, сверяет обязательные case witnesses и повторно вычисляет gates. Не принимать пользовательские booleans как доказательство. Вызвать существующий `write_release_manifest` с `known_tests_passed`, `coast_resume_resolved`, `mechanics_passed`, `numerics_converged`, `model_scope_passed` только на основании соответствующих artifacts.

`antiwheelie_reference.toml` — точная копия прошедшего resolved candidate с выбранным dt/contact/drive/rider набором. `release.json` имеет `numerically_verified_synthetic`; measured status не присваивается. Existing default fast profile остаётся прежним, новый reference выбирается явным флагом файла.

Ожидаемая пользовательская команда после реализации и выпуска:

```bash
uv run bike-ride --research \
  --physics-config examples/research/antiwheelie_reference.toml \
  --rider articulated_planar \
  --track examples/research/rough_uphill_extreme.toml \
  --policy bike_sim.sim.research.policies:passthrough_factory \
  --duration 90 --sensor-period .005 --seed 17 \
  --out output/antiwheelie-reference-viewer
```

Для пользователя замена только `--policy package.module:factory` подключает его алгоритм; `--headless` выполняет идентичную физику без окна. Флаги road resolution/stations записать в документированную команду, если они отличаются от defaults: точный release tuple нельзя потерять на CLI границе.

- [ ] **Step 5: Финальная проверка и документация.**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_coasting_contact_recovery.py tests/test_sensor_acquisition.py tests/test_research_configuration.py tests/test_research_replay.py tests/test_policy_session.py tests/test_research_viewer.py tests/test_antiwheelie_bench.py tests/test_fidelity_sweep.py tests/test_reference_release.py tests/test_episode_metrics.py tests/test_long_episode.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q
git diff --check
```

Сначала targeted suite, затем полный suite один раз после завершения. При renderer/display setup error сохранить точное сообщение и отдельно классифицировать environment failure; не объявлять полный suite green и не менять physics ради renderer. GUI проверяется отдельно на доступном display, replay — по fresh recording того же source hash.

README начинает research quick start с пользовательского сценария, статуса модели, интерфейса policy и места отчёта. Указать, как увидеть отличие terrain lift от rear-supported wheelie и как определить причину остановки. `docs/RESEARCH_COMPLETION.md` не должна утверждать работающие acquisition/replay/configuration до фактического прохождения проверок A.

## Self-review и закрытие

- [ ] B1 объясняет и исправляет именно coast/contact failure; late stall имеет независимый разбор.
- [ ] Ноги не приварены к педалям, нет tensile normal force, root stabilization или qpos/qvel correction.
- [ ] Целевая детализация сосредоточена на контакте, CoM, rider effort, pitch, suspension и actuator timing; подробная цепь не расширена.
- [ ] Полный 15-секундный run проверен по времени/прогрессу; extreme не обязан финишировать при физической невозможности, но причина не остаётся необъяснённой.
- [ ] Нет выпуска по stale evidence, одинаковым ранним отказам или subset без ступеней.
- [ ] Новый профиль проходит все заявленные gates; иначе задача ещё не завершена.
- [ ] Пользователь может запустить свою policy factory в GUI/headless и повторить запись.
- [ ] Итоговый отчёт явно различает synthetic verification, unrun GUI и отсутствие натурной калибровки.
