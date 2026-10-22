# Spatial 3D Native — 04. Startup и баланс Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Если эти skills недоступны, сохраняйте тот же порядок реализации, независимого ревью и доказательной приёмки.

**Goal:** Надёжно подготовить свободную пространственную модель, отпускать bootstrap hold до измеряемого заезда, управлять только внутренним рулевым моментом и подтвердить динамику независимыми benchmark.

**Architecture:** Расширяется уже существующий `bike::initialization`, а не создаётся второй `runtime/equilibrium`. Позиции и кватернионы обновляются в касательном пространстве MuJoCo; полное состояние включает историю шины, тормоза и руления. Эталон Whipple отделён от целевой MTB, чтобы применимость сравнения была проверяемой.

**Tech Stack:** C++23, существующие native setup/cache, MuJoCo из закреплённого qualification manifest, `bike::numerics::bounded_least_squares`/`dense`, существующий control clock, CTest и низкоуровневый native ride runner.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`, §§5,8–11; `2026-10-22-spatial-3d-native-contracts.md`; master — `2026-10-22-spatial-3d-native-master.md`.

## Global Constraints

- Текущая работа только пишет план: build/tests/simulation/CMake/syntax-only не выполняются. Все команды и ожидаемые результаты относятся к будущей реализации.
- `G1-core`/`G2-startup` должны подтвердить готовность производителей PFC до интеграции; сейчас они не проверены. S29–S36 не закрывают upstream gates своими тестами: их собственный результат — spatial physics milestone M3. Ранним автономным kernels финальный G6 не требуется.
- Существующий planar dispatch и его byte contracts сохраняются по ADR0003. Spatial refactor не удаляет scalar guards, оставляя ошибочное `qpos[i]==qvel[i]`.
- Root descriptors строятся по `mjModel`: `nq` и `nv` различны, free/ball joints учитываются отдельно. Нет общего предположения `nq=nv+1` или `nq=nv+2`.
- Bootstrap phase — явное состояние, не знак engine time. После `scored` hold force и hold work точно отсутствуют; внутренний steer actuator не является внешней опорой.
- Ни benchmark stiffness×1000, ни Python fit, ни только решение эталонной матрицы не закрывают native динамический gate.
- Числа acceptance ниже — предложенные предварительно фиксируемые критерии. Из этой документации не следует, что конкретные коэффициенты контроллера, timestep или стартовая зона уже прошли проверку.

## Существующие производители и новые типы

`native/src/initialization/setup_owner.hpp` уже содержит `SetupRequest`, `make_setup(SetupRequest)`, `SetupOwner::seal_bootstrap()`, capture initial/state-file. `equilibrium.hpp` уже содержит `settle`, `settle_context`, `initialize_pose`, `refine_equilibrium`. Они остаются едиными входами. В снимке `refine.cpp:24–43` требует `nq==nv` и использует scalar indices; новый spatial branch заменяет именно эту логику, а не условие ошибки.

| Новый тип | Владелец и данные |
|---|---|
| `spatial::BootstrapPhase` | Создан S17 в `spatial/bootstrap_state.hpp`: `settling`, `release_check`, `armed`, `scored`; S30 реализует активные переходы |
| `spatial::HoldState` | S17 declarations/codec; опорные y/yaw/roll, энергия, работа, force/moment, phase, valid mask; S30 подключает поведение |
| `spatial::SteerInput` | Control tick, roll/roll-rate, ошибки курса/поперечного положения, угол/скорость руля, знаковая скорость хода, enabled |
| `spatial::SteerState` | S17 declarations/codec; следующий tick, очередь задержки, profile digest, последняя команда/момент, насыщение, работа; S32 подключает поведение |
| `spatial::Outcome`, `OutcomeState`, `SpatialReason` | S17 declarations/codec; причина, tick/time, положение, диагностика, valid/invalid, terminal latch; S33 подключает classifier |
| `spatial::QualificationMetrics` | Результаты именованных fixtures, допуски, достоверность, identity исходников/модели/config; только cold artifact |
| `spatial::EquilibriumReport` | max_linear_accel_mps2, max_angular_accel_radps2, невязки силы/момента, контакты, ошибки; spatial-секция существующего EquilibriumResult |

`BodyPose` из общих contracts содержит `position_world_m`, `quat_world_from_body_wxyz`, `linear_velocity_world_mps`, `angular_velocity_world_rad_s`. Углы Эйлера извлекаются только для наблюдаемых величин в объявленной области корректности. Контроллер не получает неверный «roll rate» через прямое чтение произвольной компоненты free-root qvel.

Тесты используют `native/tests/support/migration_test.hpp`, существующий native model fixture и общий `native/tests/spatial/startup_fixture.hpp`. Каждый slug даёт executable `bike_spatial_<slug>` и CTest `spatial.<slug>` через `native/tests/spatial/CMakeLists.txt`; production source зарегистрированы в корневом `CMakeLists.txt`. Transitional binding graph сохраняется только для собственных gates PFC.

```bash
cmake --build --preset release --target bike_spatial_equilibrium
ctest --test-dir build/native-release --no-tests=error -R '^spatial\.equilibrium$' --output-on-failure
```

Preset/build directory сверяются с квалифицированным manifest. Команды сейчас не запускаются.

## S29. Native equilibrium в tangent coordinates

**Depends on:** S13–S19, S25–S27; `G1-core`.

**Files:** Modify `native/src/initialization/{context.hpp,equilibrium.hpp,equilibrium.cpp,initial_pose.cpp,refine.cpp,settle_contacts.cpp}`; Create `native/src/initialization/spatial_equilibrium.hpp`, `spatial_equilibrium.cpp`, `native/tests/spatial/startup_fixture.hpp`, `native/tests/spatial/equilibrium.cpp`.

**Consumes:** Существующий `SetupContext`, `spatial::Topology`/`RootBinding`, выбранный terrain, полное механическое состояние, native least-squares provider.

**Produces:** Existing `settle_context(SetupContext&,const EquilibriumOptions&) -> EquilibriumResult` получает spatial dispatch и optional `spatial::EquilibriumReport`; `spatial_refine(SetupContext&,int max_evaluations,double acceleration_tolerance) -> double`. Новый публичный setup service не создаётся.

- [ ] Начальную позу строить по линии контактов колёс и terrain sample: расположить ступицы с учётом реальных радиусов/sag, курса рамы и допустимых roll/pitch. Тела и суставы райдера сохраняют преобразования относительно родителей. Spatial root не требует имён `root_z`/`root_pitch`; невалидная начальная геометрия получает причину до solver.
- [ ] Вектор оптимизации содержит tangent increment `dq` размера `nv` по разрешённым DOF и явно выбранные неизвестные материала/контакта. Каждую позу-кандидат строить заново из замороженного q0 через `mj_integratePos(model,q_candidate,dq,1.0)`; разности проверять через `mj_differentiatePos`. Эквивалентные кватернионы q/−q дают одинаковые невязки.
- [ ] Устранить свободы выбора системы отсчёта: world x/yaw и нужные bootstrap coordinates закрепляются явной маской; связи рук, стоп и седла учитываются действующими constraints. Free/ball increments не нарезаются по qpos indices. Масштабы DOF (метры/радианы) и невязок документировать отдельно; разные units не скрывать одной нормой.
- [ ] Перед каждым независимо выбираемым кандидатом восстанавливать drivetrain, tyre, rider contacts, brake, steer и свежесть engine stage. Оценка кандидата — только probe: отвергнутая точка не продвигает материал, ratchet или latch. Существующий `bounded_least_squares` сохраняет provider/rank policy; новая Accelerate-specific ветка не нужна.
- [ ] Приёмка проверяет ускорения, невязки constraints, нагрузки/проникновение и баланс сил/моментов, а не status оптимизатора. Прежний scalar tolerance .05 сохраняется; spatial report дополнительно разделяет m/s² и rad/s². Для плоского старта: невязка поддержки веса≤0.5% mg, момента pitch/roll≤1 N·m; normals/contacts обязаны быть valid.
- [ ] Будущий `spatial.equilibrium`: constrained/free, rider/no-rider, extra ball fixture, q↔−q, finite-difference perturbation, deliberate insufficient budget; commit `feat(startup): refine spatial equilibrium on the configuration manifold`.

```cpp
bt::require(std::abs(quaternion_norm(qpos, root)-1.0) <= 1e-12,
            "refinement preserves a unit root quaternion");
bt::require(result.spatial->max_linear_accel_mps2 <= 0.05
         && result.spatial->max_angular_accel_radps2 <= 0.05,
            "spatial residuals use separate physical units");
```

**Acceptance:** Допустимый startup сходится в установленном бюджете с повторяемым результатом; ошибка показывает худшие DOF и оставляет доступный для анализа owner, который нельзя seal. Порядок probes и отвергнутые кандидаты не меняют принятую историю. Новые поля отчёта зависят от schema и отсутствуют в прежнем planar canonical output.

## S30. Bootstrap hold, отпускание и старт без скрытой опоры

**Depends on:** S29, S18/S26; hold structure может разрабатываться параллельно S29 на fixtures.

**Files:** Create `native/src/initialization/bootstrap_hold.hpp`, `bootstrap_hold.cpp`, `native/tests/spatial/bootstrap_hold.cpp`; Modify `native/src/initialization/{context.hpp,equilibrium.cpp,finalize.cpp,capture.cpp}`, `native/src/runtime/accounting.{hpp,cpp}`, `native/src/spatial/bootstrap_state.{hpp,cpp}` из S17 только для согласованного active-state validation. `BootstrapPhase`/`HoldState` не объявляются повторно.

**Consumes:** `BootstrapPhase`, `RootBinding`, pose тела/скорость точки, hold config, начальная скорость и невязка S29.

**Produces:** `BootstrapHold::try_compute_into(const mjModel&,const mjData&,BootstrapPhase,std::span<double> qfrc) noexcept -> CoreStatus`; немутирующий `state() -> const HoldState&`; явная проверка переходов стадии.

- [ ] Hold стабилизирует поперечное положение, yaw и roll, оставляя вертикальное движение, pitch и sag подвески свободными. Ошибка вращения вычисляется через quaternion/log-map; момент применяется rotational Jacobian. Вычитание углов Эйлера через±π не используется. Параметры пружин/демпферов принадлежат bootstrap config и доступны диагностике.
- [ ] Автомат стадий строго `settling → release_check → armed → scored`. В `settling` разрешена диссипативная релаксация и сброс скорости по существующему циклу; `release_check` не выполняет скрытых повторных сбросов. В `armed` hold уже нулевой, но scored accounting ещё не начат. Отрицательное engine time не требуется.
- [ ] Отпускание проверяет достоверность нагрузки, невязки и контактов, абсолютную support force<1 N и moment<1 N·m. Ненулевая «полезная» поддержка означает плохую стартовую зону или невыполнимую позу. Старт на поперечном уклоне допустим только при существующем статическом балансе; обнуление крупной силы не доказывает equilibrium.
- [ ] Для старта в движении задать согласованные root/wheel/drivetrain velocities: скорости колёс без проскальзывания, а не произвольный world vx при повернутой раме. Начальная энергия явно присутствует в state. Старт с нуля — честная проверка малой скорости с пределами контроллера и возможным balance_lost; невидимый толчок/авторазгон не добавляется.
- [ ] В scored evaluator ветка hold недоступна по phase; вклад qfrc точно нулевой, накопленная работа hold постоянна. Возврат к свежему startup возможен только через lifecycle API; replay restore сохраняет phase и не активирует опору заново.
- [ ] Будущий `spatial.bootstrap_hold`: rotated startup, bad bank, nonzero hold at release, zero-speed fall, moving start, restore scored; commit `feat(startup): enforce explicit spatial hold and release lifecycle`.

```cpp
bt::require(std::ranges::all_of(scored.hold_qfrc,
                              [](double x){ return x == 0.0; }),
            "scored state has no bootstrap force");
bt::require(scored.hold_work_j == armed.hold_work_j,
            "hold work cannot grow after release");
```

**Acceptance:** Каждый scored sample сообщает hold_active=false и точный ноль приращения силы/работы. Несошедшаяся или удерживаемая опорой конфигурация не может seal bootstrap. Порог отпускания проверяется до удаления силы, не после записи нулей.

## S31. Native setup, fresh/cache/import и полнота bootstrap

**Depends on:** S11, S27, S29–S30; подтверждённый upstream `G2-startup`. Задача доказывает spatial startup и входит в M3, но не закрывает PFC readiness gate.

**Files:** Modify `native/src/initialization/{setup_owner.hpp,setup_owner.cpp,request.cpp,equilibrium_cache.cpp,artifact_identity.cpp,state_file.cpp,capture.cpp,research_startup.cpp}`, `native/src/runtime/{bootstrap.hpp,bootstrap.cpp,bootstrap_io.cpp}`; Create `native/tests/spatial/startup_paths.cpp`.

**Consumes:** Существующие `SetupRequest`/`make_setup`, spatial config/root descriptor, terrain owner/digest, полное tyre/brake state и объявленный steering-state layout.

**Produces:** Существующий `SetupOwner::seal_bootstrap() -> runtime::RuntimeBootstrap` поддерживает spatial state version; `read_setup_request` принимает объявленные spatial inputs без Python. Cache identity учитывает каждую влияющую на физику версию отдельно от общей schema.

- [ ] Расширить request/model/terrain projection полями S05/S11; переиспользовать разрешение относительных путей assets. Fresh-start override может не открывать игнорируемый initial-state файл, но не пропускает проверку актуальных config/terrain. Capability требует успешного make_setup→settle→seal→первого принятого tick, а не одного parser.
- [ ] Cache key строить из canonical resolved config, tyre profile/schema, topology/model digest, terrain geometry/material digest, начальной позы/скорости, solver/integrator/dt, креплений райдера, bootstrap policy и нужных параметров датчика. Runtime seed и происхождение генератора отражают реальную семантику materialized asset.
- [ ] Capture охватывает integration vector, mutable frictionloss/site inputs, tyre/brake/rider/drive states, clocks/phase/energy. Steering/hold/outcome layout и codecs уже объявлены S17; до S32 используется настоящий disabled canonical steering state, не null/stub. Active controller/queue continuation проверяется в S32/S36. `runtime/step.cpp:2745` не стирает spatial latch; observation V2 подключается в S37–S39.
- [ ] Проверить identity импортируемого state, schema/tier/model dimensions и конечность значений до мутации. Неподдержанный planar↔spatial import получает явную ошибку; projection выполняется отдельным осознанным инструментом, не обрезкой массива. Повреждённый cache помечается с причиной, после чего действует настроенная fresh-start policy; mismatch явно заданного initial-state не должен молча вызывать fallback.
- [ ] Независимо проверить fresh settle, cache hit и явно захваченный state. Сравнить нормализованные physics/checkpoint bytes, исключая только поля происхождения, явно отмеченные изменчивыми; затем продолжить1000 шагов на том же binary. Первая observation/force stage должна совпасть: правильный qpos при несвежем sensor stage не означает эквивалентный startup.
- [ ] Будущий `spatial.startup_paths` и public native setup smoke на квалифицированном окружении; commit `feat(startup): support complete spatial fresh and cached bootstrap`; evidence включается в spatial M3.

```cpp
bt::require(fresh.physics_state_bytes == cached.physics_state_bytes,
            "cache restores the same complete physics state");
bt::require(imported.continuation_digest == fresh.continuation_digest,
            "explicit native import preserves continuation");
```

**Acceptance:** Ни один новый путь не требует Python; неверные identity/schema отклоняются до мутации live state. Cache metadata различает происхождение fresh/cache без изменения физического состояния. Матрица startup покрывает все поддержанные tyre tiers и rider root kinds.

## S32. Внутренний steer controller, ticks, delay и restore

**Depends on:** S14–S19, S25, S30–S31; проверка кинематического замыкания рук/руля S16 обязательна.

**Files:** Create `native/src/rider/steer_balance.hpp`, `steer_balance.cpp`, `native/tests/spatial/steer_controller.cpp`, `native/tests/spatial/data/steer_profiles.json`; Modify `native/src/runtime/{step.hpp,step.cpp,control.cpp,accounting.cpp}`, `configuration/{physical_config.hpp,validate.cpp}`, `native/src/spatial/bootstrap_state.{hpp,cpp}` из S17 для active-state validation. `SteerState` переиспользуется, `SteerInput` объявляется в новом owner header.

**Consumes:** `spatial::SteerInput`, root/body pose, действительные угол/скорость steer hinge, скорость хода, проверенный профиль, существующий control clock. Контроллер моделирует райдера; research policy не получает truth fields через этот API.

**Produces:** `SteerBalance::tick(const spatial::SteerInput&) noexcept -> double`, `state() -> const spatial::SteerState&`, `restore(const SteerState&) -> CoreStatus`; результат — внутренний рулевой момент в N·m.

- [ ] Задать знаки/единицы вектора состояния: roll, roll-rate, приведённая ошибка курса, поперечная ошибка, угол/скорость руля. Закон `tau_raw=-K(v)*x+tau_reference`; знаковые коэффициенты K, узлы скорости и интерполяция записываются в versioned profile. Знак момента выводится из теста оси S14, не из прежней интуиции world-Y.
- [ ] Получить стартовые коэффициенты cold-линеаризацией native модели и малыми возмущениями, затем настроить по объявленным цели/пределам существующими numerics. Профиль хранит точные gains, диапазон скорости и provenance; недоказанная оценка40 N·m и старая таблица не становятся измеренной силой райдера. Числовой user cap проходит проверяемый config.
- [ ] Обновлять на integer control tick; длина/округление FIFO следует общему clock contract. Для200 Hz и0.1 s — ровно20 ticks. Непредставимая задержка разрешается явно, фактическое значение записывается в metadata. Queue/reset/disable входят в checkpoint и не выделяют память warm.
- [ ] Ограничивать момент cap и range-stop policy; работу считать по реально приложенному `τ*steer_rate`. Внешняя мировая сила на раме не помогает рулению. Связи хвата и shoulder swivel обязаны допускать выбранные±60°; блокирование constraints — дефект модели, не основание поднять cap.
- [ ] Unit tests: ноль, знак, задержка, cap, disable/reset/restore. Динамические случаи: v=3/5/8 m/s, начальный roll±3° и ошибки курса±2°. Цель при3 m/s: |roll|<0.5° к3 s, |y|<0.3 m, непрерывное насыщение≤0.2 s. Это будущая область приёмки профиля, не гарантия для произвольных gains.
- [ ] Будущий `spatial.steer_controller`; commit `feat(rider): add stateful spatial steering balance on native control ticks` вместе с digest профилей, границей truth/policy и независимыми динамическими результатами.

```cpp
bt::require(first_nonzero_tick == input_tick + 20,
            "100 ms reaction delay is exactly twenty 200 Hz ticks");
bt::require(std::abs(applied_torque_nm) <= cap_nm+1e-12,
            "steering cannot exceed the configured rider strength");
```

**Acceptance:** Повторяемые ticks на одном binary, продолжение после restore, ограниченный внутренний момент и полная telemetry работы. Поведение на нулевой/неподдержанной скорости задано явно. Один тест знака не закрывает динамическую приёмку баланса.

## S33. Терминальные исходы, приоритет и достоверные метрики

**Depends on:** S19, S25, S30–S32; consumers research/recording подключаются в позднейших фазах.

**Files:** Create `native/src/spatial/outcomes.hpp`, `outcomes.cpp`, `native/tests/spatial/outcomes.cpp`; Modify `native/src/runtime/{status.cpp,research.cpp,research_metrics.hpp,research_metrics.cpp,samples.cpp}`, `native/src/research/{metrics.cpp,evidence.cpp,quality.cpp,exit_codes.cpp}`, `native/src/spatial/bootstrap_state.{hpp,cpp}` из S17 для active-state validation.

**Consumes:** Принятый tick, BodyPose/достоверность ориентации, реальные контакты/нагрузки, corridor/domain, стадия lifecycle и существующий статус завершения/ошибки эпизода.

**Produces:** `evaluate_outcome(const SpatialOutcomeInput&,OutcomeState&) noexcept -> spatial::Outcome`; новый input объявляется в `outcomes.hpp` и содержит пороги/измерения/validity текущего tick. Общие `OutcomeState`, `Outcome`, `SpatialReason` и их strict codecs уже созданы S17; classifier использует их без повторного определения. Поздние research/recording consumers не создают второй classifier.

- [ ] Добавить причины с явным смыслом: capsize, off_corridor, balance_lost, terrain_invalid/outside, numerical failure, requested completion. Сохранить существующий wire namespace; labels `crash:capsize`/`crash:off_corridor` не перезаписывают прежние значения. Невалидная симуляция не считается успешным коротким эпизодом или обычным падением райдера.
- [ ] Закрепить приоритет в одном tick: engine/nonfinite/contact-query invalidity; затем физическое опрокидывание/столкновение; выход из corridor; потеря баланса; нормальное завершение. Хранить первый сработавший tick и зафиксированную причину; повторный poll не сдвигает время и не увеличивает счётчики.
- [ ] Порог capsize применяется к ориентации относительно объявленного up/reference frame в корректной области углов; default60° из spec. Контакт корпуса с дорогой и превышение roll — разные причины. Sensor dropout не делает truth невалидной; лежащий на боку велосипед не считается успешным front lift.
- [ ] Corridor check учитывает реальный габарит шины/велосипеда и margin, а не только центр рамы внутри |y|≤W. Physical grid domain может отличаться от scoring corridor; пересечения получают разные причины. Исчерпание ray budget — численная invalidity, а не airborne/off-corridor без геометрического доказательства.
- [ ] Fixtures синтезируют точные пересечения границ и приоритеты. Динамический тест без контроллера задаёт явную неустойчивую скорость/возмущение; он не утверждает, что любой велосипед обязан упасть за3 s. Метрики разделяют запрошенную/валидную длительность, пройденное расстояние и причину завершения.
- [ ] Будущий `spatial.outcomes`; commit `feat(research): classify spatial ride outcomes and preserve validity`; передать schema в S38/S41/S44.

```cpp
bt::require(outcome.reason == SpatialReason::terrain_invalid,
            "query exhaustion cannot masquerade as airborne success");
bt::require(second_poll.tick == first_poll.tick,
            "terminal crossing is latched at the first accepted tick");
```

**Acceptance:** Все boundary/priority fixtures детерминированы; невалидные случаи не засчитываются как успех в performance/accuracy. Прежние planar outcome bytes неизменны. Пороги corridor/roll имеют SI/radian metadata и не зависят от камеры.

## S34. Отдельный физический benchmark M: Meijaard/Whipple

**Depends on:** S13–S15, S19, S29–S31; калибровка production-шины S28 не подменяет эталон идеальных контактов.

**Files:** Create `native/src/model/benchmark_whipple.hpp`, `benchmark_whipple.cpp`, `native/tests/spatial/meijaard.cpp`, `native/tests/spatial/meijaard_constraints.hpp`, `native/tests/spatial/meijaard_reference.hpp`, `native/tests/spatial/benchmark_linalg.hpp`, `examples/benchmarks/meijaard2007.toml`, `docs/research/spatial-meijaard-qualification.md`; Modify `model/{builder.hpp,builder.cpp}` benchmark dispatch.

**Consumes:** Native spatial body/joint machinery; параметры Table1 и собственные значения Table2 из [первичной статьи Meijaard et al. 2007 на сайте автора](https://arendschwab.com/assets/pdf/meijaard2007linearized.pdf); подвеска и дерево райдера MTB в модель эталона не входят.

**Produces:** `make_whipple_request(const WhippleParameters&) -> model::BuildRequest`, только тестовая динамика с идеальными constraints и приведённый native Jacobian4×4; независимые reference matrices `M,C1,K0,K2` из Appendix A. `WhippleParameters` содержит все body/inertia/geometry значения Table1, g и скорость.

- [ ] Построить четыре жёстких симметричных тела: задняя рама+райдер, передняя сборка, заднее/переднее идеальные knife-edge колёса. Явно преобразовать z-down координаты и произведения инерции статьи в z-up репозитория. Проверить compiled masses/COM/full inertia и steer axis/trail. Подвеска, податливость шин, drivetrain, хват, steer damping и стабилизатор отсутствуют.
- [ ] Реализовать только тестовые точные rolling constraints `A(q)*v=0`: нормальная, продольная и боковая скорость в каждом из двух аналитических knife-edge контактов. MuJoCo даёт M и bias; система `[M,-Aᵀ;A,0]*[vdot;λ]=[-c;-Adot*v]` решается существующим dense provider с проверкой ранга/невязки. Адаптер недоступен обычным production configs.
- [ ] Построить установившееся движение и касательный chart с выполненными contact-height constraints, фиксированной скоростью и исключёнными свободами x/y/yaw/phase. Возмущать roll/steer/rates через manifold, проецировать допустимые скорости и оценивать приведённые ускорения разностями h/h2/h4. Сравниваемый Jacobian обязан происходить от native тел/constraints; один matrix solver статьи этого не доказывает.
- [ ] Независимый reference вычисляет корни `det(λ²M+λ*v*C1+g*K0+v²*K2)`. Небольшой test-only eigen solver проверяет собственные невязки/известные матрицы, не вводя общий новый numerics provider. Сверить Table2: v=2 weave `2.68234517512745 ±1.68066296590675i`; v=6 weave `−1.52644486584142 ±5.87673060598709i`, capsize `−0.00406690076970`. Прежнее +1.7 при v=2 ошибочно.
- [ ] Критерии benchmark M: скорости 2/4/6/8/10 m/s, ошибка eigenvalues≤3% с absolute floor .05 s⁻¹; границы weave/capsize в пределах .2 m/s от 4.29238253634111/6.02426201538837. Для |λ|<.05 s⁻¹ дополнительно требовать absolute error≤5e−4 s⁻¹ и изменение при h-refinement≤1e−4 s⁻¹; корень границы локализовать до .01 m/s с проверкой знаков. Независимый малый rollout подтверждает ненейтральные моды; почти нейтральную v=6 capsize нельзя достоверно оценить за 3 s.
- [ ] Будущий `spatial.meijaard`; опубликовать model/Jacobian/reference/eigenvalues/step study и commit `test(spatial): validate native Whipple dynamics against Meijaard`. Penalty approximation допустима только с доказанной сходимостью stiffness/width к идеальному пределу; одно увеличение жёсткости×1000 не проходит benchmark M.

```cpp
bt::require(std::abs(native.weave_boundary_mps-4.29238253634111) <= 0.2,
            "native constrained model matches the weave boundary");
bt::require(kkt.relative_residual <= 1e-10,
            "benchmark contacts enforce the ideal rolling equations");
```

**Acceptance:** Совпадают и физические параметры, и динамика native модели; код только формул статьи не принимается как полное доказательство. Benchmark M проверяет topology/rigid-body conventions в своей области; точность полной MTB-шины, подвески, райдера и контроллера проверяется отдельно. Буква M здесь — имя физического benchmark, не milestone M0–M6.

## S35. Cornering, mirror invariance и выбор timestep

**Depends on:** S28, S31–S34; площадка cornering S09/S11 и квалифицированные преобразования root/forces.

**Files:** Create `native/tests/spatial/cornering.cpp`, `mirror.cpp`, `timestep.cpp`, `examples/benchmarks/spatial_cornering.toml`, `docs/research/spatial-step-convergence.md`; Modify `native/tests/spatial/startup_fixture.hpp`, `native/tests/spatial/CMakeLists.txt`, root `CMakeLists.txt`.

**Consumes:** Полная spatial модель и отдельно объявленный идеализированный steady-cornering fixture, resolved controller/tyre profile, полная telemetry энергии и внешних wrenches.

**Produces:** Именованные qualification metrics для cornering, symmetry и dt; наибольший принятый production timestep для каждого tier/profile/solver с evidence manifest. Универсальный новый default не объявляется.

- [ ] Использовать отдельную плоскую площадку достаточного размера для радиусов10/20/40 m и дуг разгона/измерения; прямой corridor шириной3 m эти окружности не вмещает. Проверить v=3/5/8 m/s; комбинации вне заявленной области friction/controller исключаются заранее с причиной, а не после неудачного заезда.
- [ ] В steady reference с тонкими контактами сравнить наклон combined COM с `atan(v²/(gR))` в пределах1°, суммарную боковую силу с `m*v²/R` в пределах2%; R/v оценивать по COM trajectory. Для полной MTB дополнительно проверить `sum(Fexternal)=m*aCOM` и баланс angular momentum с wheel gyro, шириной шин и креплениями райдера. Roll рамы не равен автоматически наклону combined COM.
- [ ] Разгон заканчивается по объявленным steady criteria: изменчивость yaw rate/roll/load. Затем измеряется фиксированный интервал. Цели: средний радиус в5%, скорость в2%, valid contacts, нулевой внешний hold. Недостигнутое установившееся состояние — failure/inconclusive с причиной, а не сравнение произвольного последнего кадра.
- [ ] Согласованно отразить `h(x,-y)`, боковые начальные условия, асимметричные features/materials, steer command и оси датчика. Изолированные symmetry-preserving kernels могут требовать точных signed bytes. Для MuJoCo trajectory зарегистрировать допуски1e−7 m и1e−7 rad на гладком2 s fixture; длинные rough runs получают анализ чувствительности. Cross-platform bitwise mirror не обещается.
- [ ] Серия dt fast: `.00125,.000625,.0003125 s`; detailed: `.0005,.00025,.000125 s`. Сравнить одинаковые control times/seeds и полные valid10 s episodes: события, импульсы нагрузок, roll/position, энергию. Цели: событие≤одного грубого шага, средняя/пиковая нагрузка≤2/5%, ориентация≤0.5°, energy residual≤0.5% масштаба. Вблизи бифуркации нужна дополнительная сходимость, не один совпавший endpoint.
- [ ] Будущие `spatial.cornering`, `spatial.mirror`, `spatial.timestep`; commit `test(spatial): qualify turning symmetry and timestep convergence` с выбранным dt и измеренными ограничениями. Квалифицированный dt fast/detailed может различаться.

```cpp
bt::require(std::abs(mean_lateral_force_n-mass*speed*speed/radius)
            <= 0.02*mass*speed*speed/radius,
            "steady ideal fixture balances lateral acceleration");
bt::require(report.valid_duration_s == report.requested_duration_s,
            "dt comparison uses complete valid episodes");
```

**Acceptance:** Применимость benchmark объявлена явно; допуски не расширяются молча после mismatch. Сравниваемые эпизоды имеют одинаковые config/domain и объявленный solver. Перестройка контактов может менять точечные пики, поэтому импульсы и сходимость state дополняют сравнение samples.

## S36. Целевые заезды и spatial physical milestone M3

**Depends on:** S06–S35, upstream G1/G2. Spatial research/recording/viewer подключаются позднее с их G3/G4/G5; S36 от этих будущих consumers не зависит.

**Files:** Create `native/tests/spatial/target_rides.cpp`, `examples/research/spatial_flat.toml`, `spatial_cross_slope.toml`, `spatial_rough_uphill.toml`, `spatial_split_grip.toml`, `docs/research/spatial-integrated-rides.md`; Modify `native/src/runtime/capabilities.cpp` только после полного path evidence, root `CMakeLists.txt`, `native/tests/spatial/CMakeLists.txt`.

**Consumes:** Квалифицированные native config/model/setup/runtime, profile identity, выбранный dt, начальная скорость, S32 controller и S33 outcomes. Использует низкоуровневый C++ runner на существующих `NativeRideRuntime`/`PhysicalStep` с заранее заданным demand trace. Observation V2, research policy/session и публичный research runner не требуются: их end-to-end путь квалифицируют S39/S44.

**Produces:** Детерминированная матрица сценариев и пакет end-to-end evidence: resolved input/model/terrain digests, startup result, trajectory/forces/energy, outcome, запрошенная/валидная длительность, статус калибровки и данные воспроизводимости.

- [ ] Сценарий F: плоскость, v=3/5/8 m/s, roll±3°, райдер включён, оба tyre tiers. Требовать полный20 s заезд без capsize/corridor loss, |y|≤.5 m и нулевой hold. Выключенный контроллер исследуется отдельно с заданными скоростью/возмущением; несовпадение заранее придуманного времени падения не опровергает F.
- [ ] Сценарий B: поперечный уклон±5°/±10°, длинный плавный переход к bank, отсутствие неожиданной world-frame spring. Проверить знаки боковой нагрузки/steer, доступное сцепление, footprint и возврат на плоскость. Цель устойчивого контроля |y|≤.75 m; режимы вне μ-domain завершаются физически и остаются valid negative tests.
- [ ] Сценарий R:100 m rough/uphill terrain при закреплённом seed, камнях/корнях/колеях, программе speed/demand и массе райдера baseline. Стартовая зона явно плоская и принятая. Qualification profile должен пройти всю дистанцию; exploratory runs могут завершаться с записанной причиной. Проверить импульсы нагрузки, подвеску, torque caps, проникновение к ободу и полный energy ledger.
- [ ] Сценарий G: split-grip торможение в повороте и остановка около нулевой скорости. Требовать общий friction budget, удержание на допустимом slope, скольжение на недопустимом, плавное отпускание/повторный старт. Сценарий C: capture/restore при повороте, тормозе и patch split с продолжением на том же binary; identity совпадает, fresh reset имеет другой смысл.
- [ ] Опубликовать пределы применимости: coefficient domain, roughness amplitude/grid, стартовую скорость, gains/delay/cap и выбранный dt. Provisional lateral data позволяют закрыть engineering integration, но не experimental MTB accuracy. RTF только валидного префикса, output-only replay, скрытый hold и неправильный writer dispatch — явные failures.
- [ ] Будущий `spatial.target_rides`; commit `test(spatial): add native end-to-end target ride qualification`. Передать M3 evidence позднейшим research/recording/viewer задачам. S54 принимает полный набор artifacts, не заменяя его коротким заездом.

```cpp
bt::require(ride.invalid_ticks == 0 && ride.bootstrap_hold_scored_ticks == 0,
            "target ride remains physical throughout the scored episode");
bt::require(ride.completed_distance_m >= ride.requested_distance_m,
            "rough-uphill acceptance requires the full requested ride");
bt::require(ride.profile_digest == manifest.profile_digest,
            "qualification cannot drift to a different tyre/controller profile");
```

**Acceptance:** S31/S36 доказывают spatial startup и полные engineering rides заявленного domain, закрывая M3. Финальная spatial qualification S54 дополнительно принимает research, recording/replay, viewer, performance и physical calibration evidence из позднейших фаз. Это не переопределяет upstream G0–G6. Конечные числа, правдоподобная анимация или один короткий flat clip не заменяют matrix.

## Handoff

S31 передаёт startup/полное состояние владельцам recording/research; S32 — внутреннюю steering telemetry/state; S33 — устойчивую семантику завершения. Физический benchmark M в S34 и сходимость S35 независимы от успеха контроллера S36: gains нельзя подстроить так, чтобы скрыть неверные инерции/контакты. Итоговый отчёт отделяет подтверждённые результаты от ожидающих проверки критериев и сохраняет source/config/model provenance.
