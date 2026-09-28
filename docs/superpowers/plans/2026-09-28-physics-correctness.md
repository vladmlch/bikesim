# bikesim: программа исправления физики Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Выполнить шесть проверяемых подпроектов без смешения legacy, новой механики и экспериментально калиброванной модели.

**Architecture:** Чистые расчёты остаются в physics, сборка тел — в mujoco, состояние и приложение сил — в sim/ride. Новые модули вводятся по физической ответственности, существующие файлы не реорганизуются без необходимости. Контрольные сценарии независимы от дорожных эталонов.

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

## Как исполнять

Это ведущий план и карта контрактов, а не одна гигантская задача. Исполнитель читает спецификацию и только текущий подплан. Каждый Task содержит отдельный red/green цикл и заканчивается коммитом. Не выполнять команды коммита при подготовке/обсуждении плана.

Примеры кода в подпланах задают обязательное ядро алгоритма и конкретные тестовые случаи. Интеграционные изменения описаны рядом с этим кодом. Это не применённый diff и не утверждение о прошедших тестах. До проверки в lock-окружении их статус — проект реализации.

Если указанные agentic sub-skills недоступны исполнителю, выполнять те же задачи последовательно вручную с теми же review-gates; этот пакет не предоставляет и не устанавливает инструменты для параллельных агентов.

## Последовательность

1. [01 — ядро и подвеска](2026-09-28-01-core-suspension.md): A1–A6. Получается изолированный physical-путь, корректная фабрика и пассивная подвеска.
2. [02 — массы и инерции](2026-09-28-02-mass-inertia.md): B1–B2. Получается единая физическая массовая модель.
3. [03 — шины и сопротивления](2026-09-28-03-tire-contact.md): C1–C6. Получаются наблюдаемые контакты и отдельный compliant backend.
4. [04 — трансмиссия и ассист](2026-09-28-04-drivetrain-assist.md): D1–D6. Получается центральный привод с цепными реакциями, freehub и тормозом.
5. [05 — связанный райдер](2026-09-28-05-articulated-rider.md): E1–E4. Получается режим articulated_effort без скрытых направляющих и двойной работы человека.
6. [06 — проверка и выпуск](2026-09-28-06-validation-release.md): F1–F4. Получаются синхронная телеметрия, набор стендов, калибровочный протокол и CLI.

B1–B2 не зависят от алгоритма шины. E1 можно делать после B2, но E3–E4 требуют C3–C4 и D1–D3. F1 начинается после A2 и дополняется каждым силовым компонентом, а не откладывается до конца. F2–F4 — обязательный gate перед публикацией physical как поддержанного режима.

## Структура файлов и владельцы

| Файл | Ответственность | Task |
|---|---|---|
| `src/bike_sim/physics/model_config.py` | Разделение режимов, общая валидация | A1 |
| `src/bike_sim/sim/ride/force_accumulator.py` | Вклады обобщённых сил без перезаписи | A2 |
| `src/bike_sim/physics/suspension_config.py` | Единая фабрика активной подвески | A3 |
| `src/bike_sim/physics/damper.py` | Firm/HBO и параметры ходов | A3–A4 |
| `src/bike_sim/physics/coil_shock.py`, `physics/stops.py` | Компрессионная coil и отдельные ограничители | A5 |
| `src/bike_sim/physics/tuning.py`, `sim/ride/sag_fit.py` | Приведённая масса и численный sag | A6 |
| `src/bike_sim/physics/component_masses.py` | Распределение единого массового бюджета | B1 |
| `src/bike_sim/sim/ride/mass_properties.py` | Динамический CoM и внешнее равновесие | B1 |
| `src/bike_sim/physics/inertia.py` | Кольца, тензоры и перенос осей | B2 |
| `src/bike_sim/sim/ride/contact_state.py` | Неизменяемые контактные величины и timestamp | C1 |
| `src/bike_sim/sim/ride/wheel_kinematics.py` | World/relative speeds, скорость точки | C1 |
| `src/bike_sim/sim/ride/contact_filter.py` | Только управляющий grounded-фильтр | C2 |
| `src/bike_sim/physics/tire.py` | Нормальная и касательная податливость | C3–C4 |
| `src/bike_sim/terrain/contact_profile.py` | Контакт с дискретным продольным профилем | C5 |
| `src/bike_sim/sim/ride/tire_forces.py` | Применение выбранного backend | C5 |
| `src/bike_sim/physics/external_resistance.py` | Чистые законы Crr и аэродинамики | C6 |
| `src/bike_sim/sim/ride/resistance.py` | Внешний rolling moment и drag | C6 |
| `src/bike_sim/mujoco/drivetrain.py` | Crank, cassette, pedals и их массы | D1 |
| `src/bike_sim/physics/chain.py` | Конфигурация передачи, касательная, удлинение, якобиан и натяжение | D1–D2 |
| `src/bike_sim/sim/ride/drivetrain_forces.py` | Силы цепи/муфты и моменты источников | D2–D6 |
| `src/bike_sim/physics/freehub.py` | Односторонняя связь кассета–колесо | D3 |
| `src/bike_sim/physics/pedaling.py`, `physics/motor.py` | Усилие человека и контроллер motor assist | D4 |
| `src/bike_sim/physics/battery.py` | Электрические потери и энергия батареи | D5 |
| `src/bike_sim/sim/ride/braking.py` | Статический тормоз и фактическая работа | D6 |
| `src/bike_sim/physics/rider_segments.py` | Неизменные анатомические массы | E1 |
| `src/bike_sim/mujoco/articulated_rider.py` | Связанное плоское дерево райдера | E2 |
| `src/bike_sim/sim/ride/rider_contacts.py` | Опоры rider–bike и их реакции | E3 |
| `src/bike_sim/sim/ride/rider_control.py` | Ограниченные суставные моменты | E4 |
| `src/bike_sim/sim/ride/energy.py`, `telemetry_v2.py` | Энергия, Ly и интервальные snapshots | F1 |
| `src/bike_sim/validation/benchmarks.py`, `tools/validate_physics.py` | Стенды и проверка сходимости | F2 |
| `src/bike_sim/validation/datasets.py` | Калибровочные данные и holdout | F3 |
| `src/bike_sim/cli/ride.py`, `sim/ride/session.py` | Передача resolved config из CLI | F4 |

`sim/ride_sim.py` изменяется в A1–A3, C5–C6, D1–D6, E2–E4 и F1; это место интеграции, не новый склад физических формул. `mujoco/builder.py` получает конфигурации до вызова subsystem builders. Добавление нового пакета `validation` включает пустой `__init__.py` в F2.

Не создавать несколько независимых реализаций contact state, battery power или инерции. Владельцы выше определяют, где вносить последующие изменения.

## Контракты между подпланами

| Контракт | Производитель | Потребители |
|---|---|---|
| `SimulationPhysicsConfig(physics_mode, drive_mode, timestep_s, pitch_assist)` | A1 | Все фабрики и CLI |
| `RideSimulation(..., physics_config=None, mass_specs=None)`; оба новых аргумента keyword-only | A1/B1 | Стенды, CLI |
| `ForceAccumulator.add(name, qfrc)` / `.total()` | A2 | Все явные writers |
| `build_suspension_components(specs)` → controller, coil | A3 | RideSimulation, sag fitter |
| `ring_inertia(mass_kg, inner_m, outer_m, width_m)` → ndarray (3,3), ось Y | B2 | Колёса, трансмиссия |
| `ContactPatch(point_m, normal, normal_load_n, tangent_force_n, slip_mps)` | C1 | Шины, сопротивления, telemetry |
| `point_velocity(v_center, omega_world, offset)` → ndarray (3,) | C1 | Шины, rider interfaces |
| `GroundedFilter.update(raw_grounded,time_s)` → bool | C2 | Только управляющая логика |
| `normal_contact(delta,delta_dot,k,c)` → force_N, energy_J | C3 | Compliant tire, rider normal contacts |
| `brush_step(xi,u,v_roll,Fn,k,mu,length,dt)` → xi_new, Fx, dissipation_step_j | C4 | Compliant tire, rider tangent contacts |
| `closest_profile_contact(center_xz,radius,vertices_xz,previous_segment=None)` → ProfileContact(point,normal,delta,segment_id,multi_support) | C5 | TireForceApplier |
| `chain_extension(cf,cr,rf,rr,theta_f,theta_r,reference)` → float | D2 | DrivetrainForceApplier |
| `Freehub.update(phi_c,phi_w,omega_c,omega_w)` → torque_Nm | D3 | DrivetrainForceApplier |
| `AssistController.step(human_nm,cadence_rpm,speed_mps,braking,dt)` → Nm | D4 | RideSimulation |
| `Battery.draw(requested_power_w,dt)` → delivered_power_w | D5 | Motor loss accounting |
| `segment_masses(total_kg,helmet_kg)` → dict[str,float] | E1 | Articulated builder |
| `apply_internal_force(model,data,body_a,body_b,point,force,qfrc)` | E3 | Rider contacts |
| `EnergyLedger.residual(energy_j,active_work_j,external_work_j,loss_j)` → float | F1 | Все стенды и отчёт |
| `run_suite(output_dir,dt_values)` → проверяемый JSON-отчёт | F2 | CLI release gate |

Расширения контрактов допускаются только согласованным изменением производителя и потребителей в одном PR или через версионированный адаптер. Имена в этой таблице используются в подпланах буквально.

## Матрица покрытия спецификации

| Требование | Задачи |
|---|---|
| SYS-01, SYS-02 | A1, A3, F4 |
| SYS-03 | A2, C5, D2, E3, F1 |
| SYS-04 | A5, C3–C4, D2–D5, F1–F2 |
| SUS-01 | A4 |
| SUS-02 | A3 |
| SUS-03 | A6 |
| SUS-04 | A5 |
| SUS-05 | A6, F2–F3 |
| MASS-01, MASS-03 | B1, F2 |
| MASS-02 | B2, D1 |
| TIRE-01 | C1–C2, C5, F1 |
| TIRE-02 | C3, C5, F2–F3 |
| TIRE-03 | C3–C4 |
| TIRE-04 | C5, F2 |
| TIRE-05 | C1, F1 |
| TIRE-06 | C6, F2–F3 |
| DRIVE-01 | D1 |
| DRIVE-02 | D2 |
| DRIVE-03 | D3 |
| DRIVE-04, DRIVE-05 | D4, E4 |
| DRIVE-06 | D5, F1 |
| DRIVE-07 | D6 |
| RIDER-01 | E1 |
| RIDER-02 | E2 |
| RIDER-03 | E3 |
| RIDER-04 | E4, F2 |
| DATA-01 | A1, F3–F4 |
| DATA-02 | C1, F1, F4 |
| A01–A23 | Локальные red/green тесты + F2 + F4 |

## Команды рабочего окружения

В корне рабочей копии базовой ревизии:

```bash
uv sync --locked
uv run --locked python -c "import sys,mujoco; print(sys.version); print(mujoco.__version__)"
uv run --locked pytest -q
```

Первый полный прогон фиксирует фактическое исходное состояние. Если установка по lock невозможна в окружении исполнителя, не переписывать lock ради продолжения физического PR: оформить отдельное изменение окружения, затем повторить базовый прогон. Текущий пакет не утверждает, что этот прогон уже выполнен.

## Gates

**G1:** A1–A6 и B1–B2. Физические параметры достигают работающих компонентов, инерции скомпилированы правильно, старые legacy-тесты сохранены.

**G2:** C1–C6. Пройдены отдельные стенды контакта, геометрии и пассивности; backend имеет явную область применимости.

**G3:** D1–D6. Пройдены передаточное отношение, виртуальная работа цепи, freehub, статический тормоз и ограничения ассиста. `crank_effort` уже пригоден для механических исследований в своей области.

**G4:** E1–E4. Пройдены масса, размыкание опор, сохранение Ly и отсутствие двойной человеческой мощности. Только после этого доступен `articulated_effort`.

**G5:** F1–F4. JSON/CLI/CSV согласованы; выполнены сходимость и воспроизводимость. V2 дополнительно требует независимого holdout-отчёта измерений.