# План: муфта свободного хода в моторе

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Цель.** Сделать муфту в моторе рабочей схемой всех трёх профилей. Педаль вперёд — зацепление, назад — свободно.
Мотор может крутить звезду, пока педали стоят. В передаче остаются ровно две муфты свободного хода: втулка заднего
колеса и муфта в моторе. Муфта ротора удаляется. Работа решателя при зацеплении муфт не считается потерей.

**Устройство.** Муфта уже есть в коде как `drive.motor_clutch = true` (`native/src/model/physical_topology.cpp:127-130`):
- ограничение `crank_clutch` с диапазоном `-1e12 0` и коэффициентами `crank_spin +1`, `drive_shaft_spin -1`;
- шатун не может обгонять вал мотора: если обгоняет, вал тянется за ним;
- если вал обгоняет шатун или шатун крутится назад, ограничение не действует;
- мотор приложен к `drive_shaft_spin` (`:136`), втулка связывает вал мотора с задним колесом (`:122-126`).

План делает четыре вещи:
1. удаляет муфту ротора;
2. убирает двойной учёт зацепления;
3. включает муфту в профилях;
4. проверяет, что шатуны не замирают в мёртвой точке, как 2026-10-03.

**Инструменты.** C++23, MuJoCo 3.12, CMake/CTest, `mise`, `bike-ride`, `bike-recording sql`, `tools/run_tests.sh`.

**Источник требований.**
- Решения пользователя 2026-11-05:
  - «я хотел 2 муфты: 1 в заднем колесе и 1 в моторе»;
  - «щелчки никакие не нужны, ни в моторе ни в заднем колесе»;
  - «при кручении педали вперед зацепление есть, при кручении назад — нет»;
  - муфту ротора — «удалить»;
  - КПД 100 % (AGENTS.md, «Передача»).
- `docs/agents/simulation-decisions.md`, «Решено — не сделано» 1, «Пробовали — не сработало» 1.
- Прочитай `docs/agents/native-engineering.md` перед задачей 1.

## Общие ограничения

- **Порядок.** Этот план выполняется **после** `docs/superpowers/plans/2026-11-05-lossless-drivetrain.md`. Там
  удаляется трение подшипников, в том числе на `drive_shaft_spin` (`native/src/writers/drivetrain.cpp:127-139`). Если
  его ещё нет, остановись (О1).
- **Проверка фактов.** Если отчёт `docs/research/2026-11-05-mid-drive-freewheel-topology.md` уже есть и противоречит
  схеме «вперёд — зацепление, назад — свободно, мотор может крутить звезду при стоящих педалях», остановись и спроси
  пользователя (О2). Если отчёта нет, работай по решению пользователя и отметь это в отчёте задачи 4.
- **Профили.** В плане меняются только `examples/research/viewer_physics_{welded,pneumatic,distributed}.toml`, строка
  `motor_clutch` и комментарии к ней. Копии в `tests/fixtures/config/viewer_physics_*.toml` — отдельные фикстуры с
  другим содержимым, их не трогать.
- **Без подстройки.** Если в задаче 4 шатуны замирают, ничего не подстраивать: ни пределы, ни фильтры мотора, ни
  ритм. Доложи числа (О3). Решение о пределах — только пользователя (`simulation-decisions.md`, пункт 34).
- **Нумерация в записях.** Порядковые номера в записях не сдвигаются (как Р3 плана передачи без потерь).
  `writer_types.hpp:30-31` требует «extend at the end only». Удаляемые поля `TelemetryField` (`motor_freewheel_*`)
  удаляются только если они не хранятся по номеру. Если хранятся, поле остаётся с пометкой `// reserved: removed
  2026-11-05, never produced`. Проверь, как кодируется `TelemetryField`: `rg -n 'TelemetryField' native/src/writers/writer_types.hpp`.
- **Эталонные записи.** Перезаписываются только darwin, Linux ждёт контейнера (решение пользователя 2026-11-05 для
  плана передачи без потерь). Если оба плана идут в одной ветке, перезапись делается один раз, после этого плана.
- **Коммиты** в стиле `git log`. В конце сообщения: `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- **Сборка** — только с разрешения пользователя. 2026-11-05 он сказал «пока не надо».
- **Бюджет тестов** (по `timings.tsv`: `5461865e native 598 s`, `dbc6ab7e full 1413 s`):
  - **Внутри задач.** Узкие тесты 1–3 мин.
  - **Конец задач 1–3.** `check_change.sh`, около 15 мин.
  - **Задача 4.** 12 прогонов по 30 с, около 10 мин.
  - **Задача 5.** Перезапись около 35 мин и итоговое `run_tests.sh full` около 25 мин.
  - **Итого** около 85 мин. Если перезапись общая с планом передачи без потерь, около 60 мин сверх него.

## Условия остановки

- **О1.** `rg -n 'bearing_c_nms_rad|drive_bearings' native/src` что-то находит: план передачи без потерь не сделан.
- **О2.** Отчёт о проверке фактов противоречит схеме.
- **О3.** В задаче 4 с муфтой шатуны замирают заметно чаще, чем без неё (признак ниже), или растёт число остановок.
- **О4.** После задачи 2 энергетическая проверка `energy.constraint_work` срабатывает на старте или на
  зацеплении. Значит, строки ограничения муфты не попадают в упругую статью: проверь
  `native/src/runtime/accounting.cpp:93`.

## Известно до начала (обзор кода 2026-11-05)

- **Двойной учёт зацепления.**
  - Работа ограничения `tendon_limit:crank_clutch` уже идёт в упругую статью (`accounting.cpp:93`, `214b671f`).
  - Писатель одновременно считает положительную часть той же силы потерей: `crank_clutch_dissipation_power_w`
    (`drivetrain.cpp:1247-1270`) складывается в `loss` в `native/src/runtime/step.cpp:2458`.
  - То же с втулкой при включённой муфте: `freehub_dissipation_power_w` (`drivetrain.cpp:1279-1281`, `step.cpp:2457`).
  - И с `motor_freewheel_dissipation_power_w` (`step.cpp:2459`).
- **Мотор при открытой муфте** не работает без педалей: без момента на педали или при скорости шатуна ниже
  5°/с выход 0 (`native/src/drivetrain/motor.cpp:75-78`). Скорость вала задаёт только верхний предел
  (`motor.cpp:68`).
  - Задержка мотора сейчас 40 мс (`physical_config.hpp:177`), 2026-10-03 было около 0,3–0,4 с.
  - Значит, «мотор крутит звезду, пока педали стоят» длится ровно столько, сколько гаснет момент: порядка
    40 мс плюс инерция вала.
- **«Нужный» каденс** считается от скорости колеса через передаточное число втулки: `required = wheel_speed /
  hub_ratio` (`drivetrain.cpp:746, 814`). Ведущим звеном втулки с муфтой является `drive_shaft_spin`. Этот каденс
  читают:
  - нижняя граница наката (`native/src/drivetrain/pedaling.cpp:42-43`);
  - желание педалирования (`pedaling.cpp:79-81`);
  - переключатель передач (`drivetrain.cpp:761-763`);
  - проверка прижатия педали (`native/src/rider/spindle_controller.cpp`, около 1149-1165).
  Явного «шатун == вал» в коде нет, но эти места его подразумевают. Задача 4 проверяет, не отсюда ли замирание.
- **Тест на жёсткую связь.** `native/tests/reference/test_welded_profile_config.cpp:34, 43-46`
  (`test_rigid_crank_topology_is_the_default`) требует `motor_clutch == false`. Профиль `welded` — схема поездки по
  умолчанию (`native/src/runtime/realtime_options.cpp:74`).
- **Тесты, которые включают муфту или ротор:** около 11 файлов (список в задаче 1).
- **Эталонные записи.** Ни в одной ротор не ненулевой. `motor_freewheel_*` встречается в 137 JSON на платформу,
  `crank_clutch_*` — в 11.

---

### Задача 1: Удалить муфту ротора

**Файлы:**
- **настройки:** `native/src/configuration/physical_config.hpp:300-301, 795`; `native/src/config_types.hpp:90`;
- **проверки:** `native/src/config_validation.hpp:197, 204-206`; `native/src/configuration/validate.cpp:629-635,
  1089-1092`;
- **провод:** `native/src/writer_config.hpp:1073, 1092-1093, 1113-1121`; `native/src/configuration/schema.cpp:597`;
  `native/src/runtime/project_config.cpp:299`;
- **модель:** `native/src/model/physical_topology.cpp:54-59, 131-134, 136`; `native/src/initialization/finalize.cpp:52`;
- **возможности:** `native/src/drivetrain/capabilities.hpp:12, 21`; `native/src/drivetrain/motor_clutch.cpp:11-16`;
- **писатель:** `native/src/writers/drivetrain.cpp`
  - `:36, 55, 130-131, 153, 188-194`;
  - `307, 357-362, 386-388, 462-463, 499-500, 510-511`;
  - `671, 712-713, 848, 876-880`;
  - `1062-1064, 1132-1134, 1254-1259, 1321-1322`;
  - `1389, 1427, 1443-1444`;
  `native/src/writers/drivetrain.hpp:69, 167-169, 197, 212, 421-430, 436`;
- **состояние:** `native/src/drivetrain/state_codec.cpp:500, 530-531, 570` (ключ `"freewheel"`);
- **телеметрия:** `native/src/writers/writer_types.hpp:108-110, 317-326`; ключи `motor_freewheel_engaged`,
  `_torque_nm`, `_dissipation_power_w` в `native/resources/recording/catalog-v1.json` (около 2286-2326) и
  `native/src/recording/catalog_data.inc` (около 2331-2371); `native/tests/recording/test_catalog.cpp:43`;
- **учёт:** `native/src/runtime/step.cpp:2459`;
- **тесты:**
  - `native/tests/reference/test_motor_freewheel_topology.cpp` (тесты ротора `:63, 161, 213` удалить; проверку
    старой муфты шатуна `:197-210` оставить);
  - `native/tests/model/test_mjcf.cpp:111, 178-183`;
  - `native/tests/reference/drivetrain_reference_support.hpp:30-32, 45, 62, 84`;
  - `native/tests/test_contracts.cpp:1685-1718, 1788, 2059-2073`;
  - `native/tests/reference/test_native_atomicity.cpp:251-258, 503-504, 533`;
  - `native/tests/reference/test_native_state_restore.cpp:810-813`;
  - `native/tests/io/test_state_codecs.cpp:178, 234, 315`;
  - `native/tests/reference/test_native_encode_columns.cpp:228-233`;
  - `native/tests/reference/recording_test_support.hpp:176-178`;
  - `native/tests/reference/test_native_drivetrain.cpp:42, 371-377, 423-425`;
  - `native/tests/config/test_schema.cpp:635-636, 883-892, 910-914`;
  - `native/tests/initialization/test_pipeline.cpp:117`;
  - `native/tests/reference/test_native_runtime_config.cpp:139-150`;
  - `native/tests/reference/test_native_numeric_completion.cpp:613`;
- **фикстуры:**
  - ключ `"freewheel"` в 32 JSON вне эталонов (`tests/fixtures/{bootstrap,ft2,replay,research,state}`);
  - `rotor_inertia_kgm2` в 65 файлах под `tests/`, везде `0.0`;
  - строка `rotor_inertia_kgm2` и комментарий к ней в трёх профилях `examples/research/viewer_physics_*.toml`.

**Интерфейсы.**
- Даёт: в модели нет тела `motor_rotor`, шарнира `rotor_spin` и ограничения `motor_freewheel`. У мотора две схемы:
  на `crank_spin` (без муфты) и на `drive_shaft_spin` (с муфтой). Ключа `rotor_inertia_kgm2` нет; ключ в старом
  документе отвергается как «unknown key».

- [ ] **Шаг 1: Записать базу**

Выполни: `git rev-parse HEAD > /tmp/clutch_base_t1`

- [ ] **Шаг 2: Падающий тест**

В `native/tests/model/test_mjcf.cpp` рядом с проверкой списка ограничений (строка 111):
```cpp
// Two one-way clutches only (user, 2026-11-05): the rear freehub and the
// motor clutch. The rotor freewheel and its body are gone, and an old
// document naming rotor_inertia_kgm2 is rejected.
void drivetrain_has_two_one_way_clutches() {
    auto request = physical("viewer_physics_welded.toml");
    request->physics->drive.motor_clutch = true;
    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    bt::require(mj_name2id(model.get(), mjOBJ_TENDON, "ideal_mid_drive_freehub") >= 0, "freehub present");
    bt::require(mj_name2id(model.get(), mjOBJ_TENDON, "crank_clutch") >= 0, "motor clutch present");
    bt::require(mj_name2id(model.get(), mjOBJ_TENDON, "motor_freewheel") < 0, "no rotor freewheel");
    bt::require(mj_name2id(model.get(), mjOBJ_BODY, "motor_rotor") < 0, "no rotor body");
    bt::require(mj_name2id(model.get(), mjOBJ_JOINT, "rotor_spin") < 0, "no rotor joint");
}
```
`physical(...)` и путь к полю `motor_clutch` возьми из соседних тестов этого файла: `rg -n 'motor_clutch' native/tests/model/test_mjcf.cpp`.
Зарегистрируй функцию в `bt::run`. В `native/tests/config/test_schema.cpp` добавь проверку, что TOML с
`[drive] rotor_inertia_kgm2 = 0.0` отвергается. Сделай её по образцу проверки неизвестного ключа в этом файле:
`rg -n 'unknown key' native/tests/config/test_schema.cpp`.

- [ ] **Шаг 3: Убедиться, что тесты падают**

Выполни: `mise run test 'model_mjcf|config\.schema|test_schema'`
Ожидается: FAIL: тест на неизвестный ключ проходит разбор, а модель с ротором собирается только при `rotor > 0`. Если
шаг 2 проходит сразу, проверь, что тест вызван.

- [ ] **Шаг 4: Удалить**

Иди по списку «Файлы»:
- **модель:** ветку тела ротора и ограничения удали; привод `mid_drive` вешай на `motor_clutch ? "drive_shaft_spin" :
  "crank_spin"`;
- **писатель:** удали `freewheel_` и все ветки по нему;
- **телеметрия:** `motor_freewheel_engaged`, `_torque_nm` и `_dissipation_power_w` писались без муфты как
  «передано» (`drivetrain.cpp:1062-1064`). Удали их из писателя, из каталогов и из `TelemetryField`. Если номера
  хранятся, оставь резерв, см. «Общие ограничения». В `test_catalog.cpp:43` уменьши число на 3 и допиши комментарий
  «minus the rotor freewheel's three leaves (2026-11-05)»;
- **учёт:** удали строку `step.cpp:2459`;
- **возможности:** удали `reflected_rotor` и `motor_freewheel` из `capabilities.hpp`, а проверку `reflected_rotor` из
  `native/tests/drivetrain/test_transmission.cpp:73-74`;
- **состояние:** ключ `"freewheel"` в `drivetrain/state_codec.cpp` удали из записи и из разбора. Старые документы с
  этим ключом отвергаются: старые записи читать не нужно (как 2026-11-04);
- **профили:** удали строку `rotor_inertia_kgm2 = 0.0` и комментарий над ней в трёх профилях.

- [ ] **Шаг 5: Тесты и фикстуры**

- Поправь тесты из списка: удали проверки ротора, оставь проверки муфты шатуна.
- Из фикстур удали ключи `rotor_inertia_kgm2` и `"freewheel"`. Формат файлов сохраняй
  (`json.dumps(doc, indent=2, sort_keys=True) + "\n"`). Каждый скрипт до записи проверяет, что файл уже в этом
  формате.

Проверь:
`rg -n 'rotor_inertia|rotor_spin|motor_rotor|motor_freewheel|reflected_rotor' native/src native/tests examples tests/fixtures --glob '!tests/fixtures/golden/**'`
Ожидается: только резервные строки, если они понадобились.

- [ ] **Шаг 6: Узкие тесты**

Выполни: `mise run test 'model_mjcf|schema|resolution|motor_freewheel_topology|native_drivetrain|transmission|test_contracts|atomicity|state_restore|state_codecs|encode_columns|recording_catalog|pipeline|runtime_config'`
Ожидается: PASS, кроме узлов с эталонными записями. Список упавших запиши в `/tmp/clutch_failing_t1.txt`.

- [ ] **Шаг 7: Коммит**

```bash
git add -A -- native/src native/tests native/resources examples/research tests/fixtures ':!tests/fixtures/golden'
git commit -m "refactor(drive): delete the rotor freewheel

The drivetrain has exactly two one-way clutches (user, 2026-11-05): the
rear freehub and the motor clutch. Removes rotor_inertia_kgm2, the rotor
body, the motor_freewheel tendon and its telemetry. Goldens are
re-recorded at the end of this branch.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 2: Зацепление муфт — не потеря

**Файлы:**
- Изменить: `native/src/runtime/step.cpp:2456-2460` (сумма потерь), `native/src/writers/drivetrain.cpp:1247-1281`
  (запись `crank_clutch_dissipation_power_w` и `freehub_dissipation_power_w`), `native/src/writers/writer_types.hpp`
  (поля и спецификации), оба каталога, `native/tests/recording/test_catalog.cpp:43`
- Тест: `native/tests/reference/test_native_drivetrain.cpp` или тест учёта, найденный по
  `rg -ln 'loss_increment|loss_step_j' native/tests`

**Интерфейсы.**
- Даёт: в `loss_j` и `loss_step_j` нет вклада муфт и втулки. Столбцов `crank_clutch_dissipation_power_w` и
  `freehub_dissipation_power_w` нет. Работа ограничений муфт видна только в упругой статье `closure_signed_j` и
  `closure_absolute_j`.

- [ ] **Шаг 1: Падающий тест**

Найди тест, который проверяет приращение потерь (`rg -n 'loss_step_j|loss_increment' native/tests`). Если его нет,
напиши проверку на уровне прогона: welded-профиль с `motor_clutch = true`, ровная дорога 30 м, 2 с, желание
райдера 40 Н·м:
```cpp
// AGENTS.md «Передача» (2026-11-05): one-way clutch engagement is not a
// loss. With the motor clutch on, the per-step loss equals the material
// losses (suspension dampers, tyres) alone: the clutch and freehub add none,
// and their solver work shows up only in the closure channel.
void test_clutch_engagement_is_not_a_loss() {
    auto env = clutch_flat_env(/*human_torque_nm=*/40.);   // build like cycle_env in test_seated_pedaling_cycle.cpp:170-229
    double clutch_closure = 0.;
    for (int i = 0; i < 1600; ++i) {
        const auto row = env.step();
        bt::require(!row.has("crank_clutch_dissipation_power_w"), "no clutch loss column");
        bt::require(!row.has("freehub_dissipation_power_w"), "no freehub loss column");
        clutch_closure = row.number("closure_absolute_j");
    }
    bt::require(clutch_closure > 0., "clutch and freehub engagement work lands in the closure channel");
}
```
Имена `clutch_flat_env`, `step`, `has` и `number` замени на средства, которыми `test_seated_pedaling_cycle.cpp`
читает столбцы (`cycle_env`, строки 170-229, чтение строк 299-389).

- [ ] **Шаг 2: Убедиться, что тест падает**

Выполни: `mise run test 'reference\.(native_drivetrain|seated_pedaling_cycle)'` (или узел, в который добавлен тест).
Ожидается: FAIL «no clutch loss column».

- [ ] **Шаг 3: Реализация**

- В `step.cpp` удали из суммы потерь `crank_clutch_dissipation_power_w` и `freehub_dissipation_power_w`.
  `motor_freewheel_*` удалена в задаче 1. Над суммой оставь комментарий: «one-way clutch engagement is solver work on
  a tendon limit, charged to the closure channel (accounting.cpp raw_work), never a loss — AGENTS.md «Передача»,
  2026-11-05».
- В писателе удали расчёт и запись обоих столбцов (`drivetrain.cpp:1260-1266, 1279-1281`). Поля
  `crank_clutch_engaged` и `crank_clutch_torque_nm` оставь: это не потери, их показывает задача 4.
- Каталоги и `TelemetryField`: убери два ключа, при необходимости оставь резерв. В `test_catalog.cpp:43` уменьши
  число на 2 и допиши комментарий.

- [ ] **Шаг 4: Тест проходит**

Выполни ту же команду, что в шаге 2.
Ожидается: PASS. Если срабатывает `energy.constraint_work` на зацеплении, это О4.

- [ ] **Шаг 5: Проверка этапа и коммит**

`bash tools/check_change.sh "$(cat /tmp/clutch_base_t1)"` должен дать выход 0 по сборке и анализу. Падать могут
только узлы с эталонными записями из списков задач. Затем:
```bash
git add -A native/src native/tests native/resources
git commit -m "fix(runtime): stop charging one-way clutch engagement as a loss

The clutch and freehub tendon-limit work was already charged to the
closure channel (214b671f) and was counted a second time as loss through
the writer's dissipation columns. The drive is lossless (AGENTS.md
«Передача», 2026-11-05).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 3: Включить муфту в профилях

**Файлы:**
- Изменить: `examples/research/viewer_physics_welded.toml:26-37, 70-72`, `viewer_physics_pneumatic.toml:38-48` и
  соответствующие строки комментариев, `viewer_physics_distributed.toml:31-41`
- Изменить: `native/tests/reference/test_welded_profile_config.cpp:34-46`
- Тест: `native/tests/model/test_mjcf.cpp` (поведение муфты на модели)

**Интерфейсы.**
- Использует: задачи 1–2.
- Даёт: во всех трёх профилях `motor_clutch = true`.

- [ ] **Шаг 1: Падающие тесты**

В `test_welded_profile_config.cpp` замени `test_rigid_crank_topology_is_the_default` на:
```cpp
// The motor clutch is the default topology (user, 2026-11-05): the pedals
// drive the chainring through a one-way clutch, so they turn backward freely
// and the motor may turn the chainring while they stand.
void test_motor_clutch_topology_is_the_default() {
    const auto config = load_welded_example();                 // existing helper at :34
    bt::require(config.drive.motor_clutch, "the welded example uses the motor clutch");
    bt::require(config.drive.transmission_model == "ideal_mid_drive", "ideal lossless drive");
}
```
Обнови вызов в `bt::run`. В `native/tests/model/test_mjcf.cpp` добавь проверку поведения на скомпилированной модели:
```cpp
// Forward the crank pulls the motor shaft along; backward (or slower) it is
// free and the shaft keeps its speed.
void motor_clutch_engages_forward_only() {
    auto request = physical("viewer_physics_welded.toml");
    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(mj_makeData(model.get()), mj_deleteData);
    const int crank = mj_name2id(model.get(), mjOBJ_JOINT, "crank_spin");
    const int shaft = mj_name2id(model.get(), mjOBJ_JOINT, "drive_shaft_spin");
    bt::require(crank >= 0 && shaft >= 0, "crank and motor shaft joints exist");
    const auto run = [&](double crank_rate, double shaft_rate) {
        mj_resetData(model.get(), data.get());
        model->opt.gravity[2] = 0.;                              // isolate the drive from the bike's fall
        data->qvel[model->jnt_dofadr[crank]] = crank_rate;
        data->qvel[model->jnt_dofadr[shaft]] = shaft_rate;
        for (int i = 0; i < 20; ++i) mj_step(model.get(), data.get());
        return std::pair{data->qvel[model->jnt_dofadr[crank]], data->qvel[model->jnt_dofadr[shaft]]};
    };
    const auto [c_back, s_back] = run(-3., 5.);
    bt::require(c_back < 0. && s_back > 4.5, "backpedalling leaves the shaft running");
    const auto [c_fwd, s_fwd] = run(8., 2.);
    bt::require(std::abs(c_fwd - s_fwd) < .2, "a faster forward crank drives the shaft");
}
```
Тела райдера, колёса и втулка тоже связаны с шатуном. Если без райдера модель не собирается, возьми запрос без
райдера (вариант `none`), как это делают соседние тесты: `rg -n 'variant' native/tests/model/test_mjcf.cpp`. Числа 4,5
и 0,2 — допуск на втулку и ведомое колесо. Если тест падает только из-за них, напечатай фактические скорости и подбери
проверку по смыслу (вал не тормозится шатуном; вал догоняет шатун), не меняя саму муфту.

- [ ] **Шаг 2: Убедиться, что тесты падают**

Выполни: `mise run test 'welded_profile_config|model_mjcf'`
Ожидается: FAIL «the welded example uses the motor clutch». `motor_clutch_engages_forward_only` падает, потому что
профиль ещё без муфты.

- [ ] **Шаг 3: Переключить профили и переписать комментарии**

В трёх профилях раздел `[drive]` должен стать таким (шапка `# Welded-rider research profile…` остаётся):
```toml
[drive]
# Mid-drive with a one-way motor clutch (user, 2026-11-05): pedalling
# forward engages it and drives the chainring; backward, or slower than the
# motor shaft, it is free — the pedals turn back freely and the motor may
# keep turning the chainring while they stand. The rear freehub is the only
# other one-way element; the drive is lossless (AGENTS.md «Передача»).
# RU: педаль вперёд — зацепление, назад — свободно; муфт две: в моторе и втулка.
transmission_model = "ideal_mid_drive"
motor_clutch = true
```
Комментарий в `[drive.assist]` «Torque envelope vs crank rpm (crank == motor shaft with the rigid chainring)»
замени. Предел момента считается по скорости вала мотора (`native/src/drivetrain/motor.cpp:68`):
```toml
# Torque envelope vs motor-shaft rpm (motor.cpp ceiling). With the clutch
# engaged the shaft turns at crank rpm.
```

- [ ] **Шаг 4: Тесты проходят**

Выполни: `mise run test 'welded_profile_config|model_mjcf|rider_welds|realistic_pedelec|seated_pedaling_cycle'`
Ожидается: PASS для двух новых тестов. Тесты, читающие профили (`rg -l 'examples/research' native/tests`, около 30
файлов), могут поменять результат. Запиши упавшие в `/tmp/clutch_failing_t3.txt` и разбери каждый: эталонная
запись (перезапишется в задаче 5) или поведение (задача 4).

- [ ] **Шаг 5: Проверка этапа и коммит**

`bash tools/check_change.sh <коммит задачи 2>`. Затем:
```bash
git add examples/research/viewer_physics_*.toml native/tests/reference/test_welded_profile_config.cpp native/tests/model/test_mjcf.cpp
git commit -m "feat(drive): make the one-way motor clutch the profile default

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 4: Проверка замирания шатунов и педалирования

**Файлы:**
- Создать: `docs/superpowers/audits/2026-11-05-motor-clutch-rides.md`

- [ ] **Шаг 1: Прогоны: с муфтой и без, по одному**

Временные копии профилей сделай в `/tmp/clutch/`. Вариант `off` получается одной строкой `motor_clutch = false`:
```bash
mkdir -p /tmp/clutch/runs
for p in welded pneumatic; do
  cp examples/research/viewer_physics_$p.toml /tmp/clutch/${p}_on.toml
  sed 's/^motor_clutch = true$/motor_clutch = false/' /tmp/clutch/${p}_on.toml > /tmp/clutch/${p}_off.toml
done
cp examples/research/rider_*.json /tmp/clutch/ 2>/dev/null   # profile paths resolve next to the TOML
for p in welded pneumatic; do for v in on off; do for t in rough_uphill rough_uphill_savage rough_uphill_extreme; do
  build/native-release/bin/bike-ride --backend native --physics-config /tmp/clutch/${p}_$v.toml \
    --track examples/research/$t.toml --headless --duration 30 --no-plots --check-policy diagnostic \
    --record --out /tmp/clutch/runs/${p}_${v}_$t; echo "$p $v $t rc=$?"; done; done; done
```

- [ ] **Шаг 2: Признак замирания и сравнение**

Для каждого прогона посчитай через `bike-recording sql "$R" --json --query …` (столбцы: `channel_drive.crank_rad_s`,
`cadence_rpm`, `drive_shaft_rad_s`, `crank_clutch_engaged`, `crank_clutch_torque_nm`, `motor_torque_nm`,
`coasting_reason`, `required_cadence_rpm`; точные имена таблиц — `bike-recording fields "$R"`):
- **доля замирания:** доля времени после 3 с, когда райдер педалирует (`coasting_reason` пусто), а `crank_rad_s <
  2` рад/с. Порог 2 рад/с — из замеров 2026-10-03: без муфты шатун не падал ниже 2 рад/с на уклонах 15–25 %;
- **размыкания:** число размыканий муфты и их средняя длительность;
- **момент мотора:** средний момент мотора при разомкнутой муфте;
- **результат прогона:** пройденная дистанция, число событий `balance_lost`, первое нарушение и его время.

В отчёте сведи таблицу «профиль × трасса × муфта». Отдельно проверь, нет ли замирания, когда
`required_cadence_rpm` больше каденса шатуна. Это догадка из обзора кода (раздел «Известно до начала»); проверь её
сопоставлением во времени.

- [ ] **Шаг 3: Решение по О3**

Если доля замирания с муфтой больше, чем без неё, более чем на 5 процентных пунктов, или с муфтой меньше дистанция
или больше остановок — это О3. Остановись, покажи пользователю отчёт и варианты. Ничего не подстраивай.

- [ ] **Шаг 4: Показать отчёт пользователю**

Даже без О3 покажи таблицу. Пользователь решает, идти ли дальше к перезаписи.

### Задача 5: Эталонные записи, документы, итоговая проверка

**Файлы:** `tests/fixtures/golden/darwin-arm64/**` и общие фикстуры, `docs/agents/simulation-decisions.md`,
`docs/RIDE.md`, `docs/TESTING.md` (разделы, где сказано, что шатун жёстко связан со звездой: `rg -n -i 'rigid
crank|crank/chainring|motor_clutch|rotor' docs/*.md`).

- [ ] **Шаг 1: Перезапись**

Повтори задачу 4 плана `docs/superpowers/plans/2026-11-05-lossless-drivetrain.md` (шаги 1–6): режим
`BIKE_GOLDEN_RECORD=1`, производные хэши, самопроверка, сравнение эпизодов. Сравнение сохрани в
`docs/superpowers/audits/2026-11-05-motor-clutch-episode-{before,after}.txt`. Если оба плана в одной ветке, сделай
одну перезапись после задачи 4 этого плана. Тогда задача 4 плана передачи без потерь пропускается.

- [ ] **Шаг 2: Документы**

- **`simulation-decisions.md`:** пункт 1 «Решено — не сделано» перенеси в «Действует» с коммитами и ссылкой на отчёт
  задачи 4. В пункт 3 (переставка) допиши: «муфта в моторе включена, план переставки — после ревизии 2026-11-05».
- **`RIDE.md`, `TESTING.md`:** замени описания жёсткого шатуна. Исторические разделы не трогай.

- [ ] **Шаг 3: Итоговое доказательство**

Выполни: `bash tools/run_tests.sh full`. Ничего не запускать параллельно.
Ожидается: выход 0.

- [ ] **Шаг 4: Коммиты**

```bash
git add -A tests/fixtures docs/superpowers/audits/2026-11-05-motor-clutch-*
git commit -m "test(golden): re-record darwin goldens for the motor clutch

linux-aarch64 goldens are stale until the container re-record.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
git add docs
git commit -m "docs: record the motor clutch as the default drive

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
