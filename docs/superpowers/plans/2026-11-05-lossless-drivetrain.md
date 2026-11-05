# План: передача без потерь (КПД 100 %)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Цель.** Привести код к правилу AGENTS.md, раздел «Передача»: КПД 100 % во всех механических частях, цепь не
моделируется. Для этого:
1. убрать вязкое трение подшипников (`bearing_c_nms_rad`, составляющая силы `drive_bearings`);
2. целиком удалить вариант передачи с упругой цепью (`transmission_model = "elastic_chain"`), вместе с упругой
   втулкой и телом кассеты;
3. убрать трение и КПД 0,96 из ядра видеокарты;
4. перезаписать эталонные записи darwin.

**Устройство.** Работа идёт по образцу удаления демпфирования суставов (`14e9c054`, план
`docs/superpowers/plans/2026-11-04-no-joint-damping.md`):
- сначала код и тесты;
- затем перезапись эталонных записей в режиме `BIKE_GOLDEN_RECORD=1` и обновление производных хэшей;
- в конце полная проверка.

Старые записи с удалёнными ключами читать не нужно: строгие разборщики отвергнут их как «unknown key».

**Инструменты.** C++23, MuJoCo 3.12, CMake/CTest, `mise`, `bike-recording golden freeze|compare`,
`tools/run_tests.sh`.

**Источник требований.**
- Решения пользователя 2026-11-05: «я вообще хочу чтобы везде было 100% кпд»; «цепь не моделируется»; «не
  моделируются потери в звездах и любых других механических частях»; вариант с упругой цепью — «удалить»;
  перезапись эталонных записей — «ок»; потери в шинах остаются.
- `docs/agents/simulation-decisions.md`, «Действует» 20 и «Решено — не сделано» 10.
- Прочитай `docs/agents/native-engineering.md` перед задачей 1.

## Общие ограничения

- **Что остаётся.**
  - `ideal_mid_drive` и `geometric_ideal_mid_drive`;
  - ограничение `ideal_mid_drive_freehub` (`native/src/model/physical_topology.cpp:123`) — это втулка идеального
    пути, жёсткое ограничение без демпфирования;
  - `motor_clutch`, `rotor_inertia_kgm2` и их ограничения;
  - тормоза, демпферы вилки и амортизатора, демпфирование шин и контактов, удержание тормоза, сопротивление воздуха и
    качению;
  - производная составляющая регулятора райдера `joint_kd_nms_rad`: это активное управление, а не пассивная потеря.
- **Эталонные записи.**
  - Перезаписываются только эталонные записи darwin (`tests/fixtures/golden/darwin-arm64/**`) и общие фикстуры.
  - Linux не трогать (Р2).
  - Каждый изменившийся узел должен объясняться удалением трения или удалением столбцов. Сравни `golden compare` до и
    после, как в `docs/superpowers/audits/2026-11-04-no-joint-damping-episode-{before,after}.txt`.
- **Профили** `examples/research/*.toml` уже используют `ideal_mid_drive` и не задают удаляемых ключей. Они не должны
  требовать правок; если требуют — остановись (О1).
- **Коммиты** в стиле `git log`: `refactor(drive): …`, `test(golden): …`. В конце сообщения:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- **Сборка.** Сборок `build/native-release` сейчас нет. Перед задачей 1 спроси пользователя о разрешении на сборку и
  о месте работы (Р6).
- **Бюджет тестов** по `<git-common-dir>/bike-gate/timings.tsv` (последние: `5461865e native 598 s`,
  `dbc6ab7e full 1413 s`):
  - **Внутри задачи.** `mise run test '<шаблон>'`, 1–3 мин.
  - **Конец задач 1–3.** `bash tools/check_change.sh <base>`, не больше 5 мин каждая, около 15 мин.
  - **Задача 4.** Перезапись: обычный набор и медленный набор в режиме записи, затем повтор без записи — около
    35 мин.
  - **Задача 5.** Итоговое доказательство, уровень 3: `bash tools/run_tests.sh full`, около 25 мин.
  - **Итого** около 75 мин плюс сборка.
  - Никаких двух проверок одновременно.

## Решения (ответы пользователя 2026-11-05)

Ответы:
- Р1 — да; Р2 — да; Р3 — да.
- Р5 решено иначе, чем предлагалось:
  - муфт в передаче ровно две: втулка и муфта в моторе;
  - щелчков и потерь при зацеплении нет;
  - муфта ротора `motor_freewheel` удаляется;
  - численная работа при зацеплении муфты в моторе идёт в упругую статью.
  Всё это делается в отдельном плане включения муфты в моторе, а не здесь. В этом плане `motor_clutch`,
  `crank_clutch` и `motor_freewheel` не трогать.
- Р6 — «пока не надо»: сборку и рабочее дерево не создавать. До разрешения сборки план не начинать.

- **Р1. Видеокарта.** В `native/src/gpu/drivetrain_resistance_kernel.hpp` убрать:
  - `wheel_bearing_viscous_nm_s_rad = 0.004f`, `wheel_bearing_coulomb_nm = 0.02f`, `drivetrain_viscous_nm_s_rad =
    0.015f` (строки 20-22) и места их применения (`:99-106`, `:194-196`, `:231`, `:334-336`);
  - поставить `efficiency = 1.0f` или убрать поле (строка 28);
  - убрать демпфирование трещотки `freehub_damping_nm_s_rad = 4.5f` (строка 30), если трещотка не заменяется
    жёстким ограничением.
  Ядро видеокарты — опытная линия, но правило КПД 100 % действует везде. Рекомендация: убрать.
- **Р2. Linux.** Эталонные записи linux-aarch64 не перезаписываются, как 2026-11-04 («линукс пока не делай»). Они
  останутся устаревшими до прогона в контейнере, это записывается в сообщение коммита.
- **Р3. Порядковые номера.** `ForceKind::bearing` (`native/src/writers/writer_types.hpp:36`) и метка
  `"elastic_chain"` в `transmission_model_labels` (`:159-160`) хранятся в записях по порядковому номеру. Комментарий
  требует «extend at the end only». Рекомендация: не удалять эти элементы, а пометить `// reserved: removed
  2026-11-05, never produced`, чтобы номера `suspension`, `tire`, `rider`, `resistance` и `ideal_mid_drive` не
  сдвинулись. Разбор конфигурации значение `"elastic_chain"` отвергает.
- **Р4. Столбцы.** Удаляются столбцы, которые в идеальном пути всегда нули: `chain_extension_m`,
  `chain_extension_rate_mps`, `chain_tension_n`, `chain_energy_j`, `chain_dissipation_power_w`, `chain_power_w`,
  `freehub_deflection_rad`, `freehub_energy_j`. Остаются `freehub_torque_nm`, `freehub_engaged`, `freehub_power_w`
  (идеальная втулка) и `freehub_dissipation_power_w` (ненулевой только при `motor_clutch = true`).
- **Р5. Потери муфт** `crank_clutch` и `motor_freewheel` (`native/src/runtime/step.cpp:2458-2459`) и
  `freehub_dissipation_power_w` при `motor_clutch = true` (`native/src/writers/drivetrain.cpp:1279-1281`) — это
  работа решателя на ограничениях, а не заданное трение. Рабочие профили их не включают. В этом плане их не трогать.
  Вопрос записывается в `simulation-decisions.md`, «Не решено».
- **Р6. Место работы.** Новое рабочее дерево `.worktrees/lossless-drivetrain` с веткой `lossless-drivetrain` от
  `main`. Зависимости: `mise run deps`. Сборка: `cmake --workflow --preset release` (около 10 ГБ).

## Условия остановки

- **О1.** Рабочий профиль `examples/research/viewer_physics_*.toml` перестаёт загружаться.
- **О2.** После задачи 1 меняется эталонная запись узла, не связанного с передачей. Например, узлы без
  `drive_bearings`, без колёс или шатуна: `suspension`-only стенды, равновесие. Сначала найди причину.
- **О3.** Самопроверка записи: не затронутые узлы при перезаписи должны совпасть байт в байт. Если не совпали,
  остановись.
- **О4.** В задаче 4 меняется хэш, которого нет в списке задачи 4, шаг 3. Найди его потребителя, прежде чем менять.

## Карта файлов (точные строки — из обзора 2026-11-05, проверить перед правкой)

| Область | Файлы |
|---|---|
| настройки | `native/src/configuration/physical_config.hpp:281-289, 783-787`; `native/src/config_types.hpp:89-90` |
| проверки | `native/src/configuration/validate.cpp:625-628, 647-654`; `native/src/config_validation.hpp:194-205` |
| схема | `native/src/configuration/schema.cpp:594-597, 604, 1235-1237` |
| провод | `native/src/writer_config.hpp:921, 1059-1060, 1073, 1080-1082, 1089-1091, 1115-1117` |
| проекция | `native/src/runtime/project_config.cpp:293-300` |
| CLI и прочее | `native/src/cli/configuration.cpp:213-217`; `native/src/cli/argument_contracts.inc:1423, 1433-1447`; `native/src/research/configuration.cpp:211`; `native/src/drivetrain/capabilities.hpp:10`; `CMakeLists.txt:120-126` |
| модель | `native/src/model/physical_topology.cpp:69-81, 110`; `native/src/model/drivetrain.cpp:62` |
| писатель | `native/src/writers/drivetrain.cpp:28, 95, 114-174, 197-200, 303, 528, 726-727, 912-952, 990-993, 1021-1033, 1351-1376`; `native/src/writers/drivetrain.hpp:87, 97, 102-103, 156, 365-373, 404-429, 461-470, 501-506, 513-514` |
| удаляемые файлы | `native/src/drivetrain/chain.cpp`, `elastic_chain.cpp`, `freehub.cpp`, `freehub.hpp` (если `policy_set.hpp:19-70` больше их не строит) |
| учёт | `native/src/runtime/step.cpp:1824-1836, 2430, 2456-2457` |
| телеметрия | `native/src/writers/writer_types.hpp:36, 81-85, 94-98, 159-160, 206-223, 258-275`; `native/resources/recording/catalog-v1.json` и `native/src/recording/catalog_data.inc` (8 ключей из Р4); `native/tests/recording/test_catalog.cpp:43` (число 487) |
| стенды проверки | `native/src/validation/drivetrain.cpp:17, 63-72, 139-147`; `rigs.cpp:82-106`; `rigs.hpp:25-28`; `registry.cpp:31-33, 157`; `benchmark.cpp:35, 40, 73, 204-207, 220`; `cli_options.hpp:46` |
| видеокарта (Р1) | `native/src/gpu/drivetrain_resistance_kernel.hpp:20-30, 99-106, 194-196, 231, 245-249, 334-336`; `native/tests/gpu/test_gpu_drivetrain_resistance.cpp:595, 748`; `tools/colab_gpu/README.md:29` |
| тесты | `native/tests/reference/test_native_numeric_completion.cpp`, `native/tests/test_contracts.cpp`, `native/tests/config/test_schema.cpp`, `native/tests/reference/test_native_drivetrain.cpp`, `native/tests/drivetrain/test_transmission.cpp` и ещё 10 файлов (`rg -l 'elastic_chain\|chain_k_n_m\|chain_c_ns_m\|bearing_c_nms_rad\|drive_bearings\|hub_stiffness_nm_rad\|hub_damping_nm_s' native/tests`) |
| фикстуры | `tests/fixtures/config/variant_elastic_chain{.toml,.resolved.json,_shifting_rejected.toml}`; `tests/fixtures/config/schema.json`, `resolved/*.json`; 257 файлов под `tests/fixtures/golden` (darwin около половины) |

---

### Задача 1: Убрать трение подшипников

**Файлы:** строки «настройки», «проверки», «схема», «провод», «проекция» (только `bearing_c_nms_rad`); писатель
`drivetrain.cpp:114-140, 950-952`, `drivetrain.hpp:87, 97, 102-103, 156, 418`; учёт `step.cpp:2430`; `writer_types.hpp:36`
(Р3); тест `native/tests/reference/test_native_drivetrain.cpp`.

**Интерфейсы.**
- Даёт: в `DrivetrainWriter` нет `drive_bearings`. Массив составляющих силы сокращается с 4 до 3 элементов
  (`drivetrain.hpp:97, 156`). Поле `bearing_c_nms_rad` и ключ провода удалены.

- [ ] **Шаг 1: Записать хэш базы**

Выполни: `git rev-parse HEAD > /tmp/lossless_base_t1`

- [ ] **Шаг 2: Написать падающий тест**

В `native/tests/reference/test_native_drivetrain.cpp` найди тест, который строит писатель на welded-профиле
(`rg -n 'DrivetrainWriter|ideal_mid_drive' native/tests/reference/test_native_drivetrain.cpp`). Рядом добавь:
```cpp
// AGENTS.md «Передача» (2026-11-05): the drivetrain has no mechanical loss.
// A spinning wheel and crank with zero human and motor torque receive no
// drive force at all — no bearing component exists.
void test_ideal_drive_applies_no_bearing_drag() {
    auto rig = welded_ideal_rig();            // the helper the neighbouring tests use
    rig.set_velocity("rear_wheel_spin", 10.);
    rig.set_velocity("front_wheel_spin", 10.);
    rig.set_velocity("crank_spin", 8.);
    const auto bank = rig.prepare_zero_effort();
    for (const auto& component : bank.component_names())
        bt::require(component != "drive_bearings", "no bearing component is emitted");
    bt::require(rig.drive_force_norm() == 0., "zero effort gives zero drive force");
}
```
`welded_ideal_rig`, `set_velocity`, `prepare_zero_effort` и `drive_force_norm` замени на средства, которые уже есть
в этом файле или в `native/tests/reference/drivetrain_reference_support.hpp`. Если подходящих нет, проверь через
`ForceComponentView` из `drivetrain.hpp:156`, что составляющей с именем `drive_bearings` нет.

- [ ] **Шаг 3: Убедиться, что тест падает**

Выполни: `mise run test 'reference\.native_drivetrain'`
Ожидается: FAIL «no bearing component is emitted».

- [ ] **Шаг 4: Удалить трение**

- `physical_config.hpp:289` и строка отражения `REAL(PhysicalDriveConfig, bearing_c_nms_rad)` (около 787):
  удалить.
- `config_types.hpp:90`: удалить `bearing_c_nms_rad{}`.
- `validate.cpp:654`, `config_validation.hpp:196`: удалить проверку.
- `schema.cpp:597`: удалить из обязательных ключей.
- `writer_config.hpp:1073` (список `extra_keys`) и `:1091` (разбор): удалить.
- `project_config.cpp:298`: удалить из списка проходящих ключей.
- `drivetrain.cpp`: удалить `bearing_joints_.push_back(j);` (125, 139) и цикл 950-952.
- `drivetrain.hpp:418`: удалить член `bearing_joints_`.
- `drivetrain.hpp`: убрать `drive_bearings` из `ForceComponent`, из таблицы спецификаций (102-103), размер массива
  4 → 3 (97, 156).
- `step.cpp:2430`: список станет `{"fork_damper", "shock_damper", "shock_hbo"}`.
- `writer_types.hpp:36`: `bearing,  // reserved: removed 2026-11-05 (AGENTS.md «Передача»), never produced` (Р3).
- Текст каталога «cumulative material dissipation (dampers, brush, bearings)» (`catalog-v1.json:831`,
  `catalog_data.inc:835`): убрать слово «bearings».

- [ ] **Шаг 5: Собрать и прогнать тест**

Выполни: `mise run test 'reference\.native_drivetrain'`
Ожидается: PASS.

- [ ] **Шаг 6: Найти и исправить остальные упоминания**

Выполни: `rg -n 'bearing_c_nms_rad|drive_bearings|bearing_joints' native/src native/tests tools`
Ожидается: только комментарий о резерве в `writer_types.hpp` и тесты, которые надо поправить. В тестах удали
ключ из ожидаемых конфигураций и списков составляющих. Отрицательный тест
`mutated_drive_config_rejected_before_model_writes/bearing_c_nms_rad--1.0` удали вместе с его эталонной записью: ключа
больше нет.

- [ ] **Шаг 7: Фикстуры конфигурации**

Убери `bearing_c_nms_rad` из `tests/fixtures/config/schema.json`, `resolved/*.json`, `variant_*.resolved.json`,
`tests/fixtures/research/delayed_bootstrap/config.json`. Сохрани формат: `json.dumps(doc, indent=2, sort_keys=True) +
"\n"`. Проверь до записи, что файл в нём.

- [ ] **Шаг 8: Узкие тесты без эталонных записей**

Выполни: `mise run test 'config\.|schema|resolution|native_drivetrain|test_contracts|transmission'`
Ожидается: PASS. Узлы с эталонными записями могут падать: они перезаписываются в задаче 4. Запиши список упавших в
`/tmp/lossless_failing_t1.txt`. В нём не должно быть узлов из О2.

- [ ] **Шаг 9: Коммит**

```bash
git add -A native/src native/tests tests/fixtures/config tests/fixtures/research
git commit -m "refactor(drive): remove the viscous bearing drag

AGENTS.md «Передача» (user, 2026-11-05): no mechanical loss anywhere in
the drivetrain. Goldens are re-recorded in a later commit of this branch.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 2: Удалить вариант с упругой цепью

**Файлы:** все строки карты файлов, относящиеся к `elastic_chain`, `chain_*`, `freehub_k/c`, `hub_stiffness_nm_rad`,
`hub_damping_nm_s`, телу и шарниру `cassette`; удаляемые файлы; стенды проверки; столбцы Р4.

**Интерфейсы.**
- Использует: задачу 1 (массив из 3 составляющих).
- Даёт:
  - `PhysicalDriveConfig::transmission_model` по умолчанию `"ideal_mid_drive"`;
  - допустимые значения `{"ideal_mid_drive", "geometric_ideal_mid_drive"}`;
  - в писателе нет `simplified_`: путь всегда идеальный;
  - в политиках нет `Freehub`.

- [ ] **Шаг 1: Падающий тест — `elastic_chain` отвергается**

В `native/tests/config/test_schema.cpp`:
```cpp
// The elastic-chain transmission was removed (user, 2026-11-05: 100 % drive
// efficiency, the chain is not modelled). The value and its keys are rejected.
void test_elastic_chain_is_rejected() {
    bt::require(bt::throws([] { (void)load_physics_text(R"(
physics_mode = "physical"
[drive]
transmission_model = "elastic_chain"
)"); }), "elastic_chain is no longer a transmission model");
    bt::require(configuration::PhysicalDriveConfig{}.transmission_model == "ideal_mid_drive",
                "the declared default is the ideal drive");
}
```
`load_physics_text` замени на средство разбора TOML из этого файла. Если для полного документа нужны другие
обязательные ключи, возьми `tests/fixtures/config/variant_elastic_chain.toml` целиком и проверь, что он отвергается.

- [ ] **Шаг 2: Убедиться, что тест падает**

Выполни: `mise run test 'config\.schema|test_schema'`
Ожидается: FAIL.

- [ ] **Шаг 3: Удалить вариант**

Иди по карте файлов сверху вниз:
- **настройки:** `chain_k_n_m`, `chain_c_ns_m`, `freehub_k_nm_rad`, `freehub_c_nms_rad`; значение по умолчанию
  `"ideal_mid_drive"`;
- **проверки:** удалить ветку `elastic_chain` и её исключения (`config_validation.hpp:204-205`);
- **схема и провод:** удалить ключи `chain_*`, `hub_stiffness_nm_rad`, `hub_damping_nm_s`, значение из списков
  допустимых;
- **проекция:** `project_config.cpp:293-300`;
- **CLI:** удалить `--chain-stiffness`, `--freehub-stiffness` и проверку «только elastic»;
- **исследования:** удалить переопределение шага `.000125` (`research/configuration.cpp:211`);
- **`capabilities.hpp:10`:** удалить поле `elastic_chain` и места, где оно читается
  (`rg -n 'elastic_chain' native/src`);
- **модель:** удалить ветку `physical_topology.cpp:69-81` и `:110`. Колесо несёт полную массу, как сейчас в
  идеальном пути. `geom_rear_cassette` (`model/drivetrain.cpp:62`) оставить на колесе;
- **писатель:** удалить `simplified_` и все ветки `!simplified_`: шарнир кассеты, `hub_`, `hub_scratch_`, расчёт цепи
  `912-945`, `chain_loss`/`hub_loss` `990-993`, запись столбцов `1021-1033` для удаляемых ключей, `stored_energy`
  `1362-1376` (если `simplified` возвращал `{}`, функция становится пустой — удали её и её вызовы). Ветки «простого
  пути» оставь без условия;
- **политики:** `drivetrain/policy_set.hpp` — убрать построение `Freehub`. После этого удалить `drivetrain/freehub.{hpp,cpp}`,
  `drivetrain/chain.cpp`, `drivetrain/elastic_chain.cpp`, если `rg` не находит других пользователей. Из
  `CMakeLists.txt:120-126` удалить соответствующие исходники;
- **учёт:** `step.cpp:1824-1826` (`chain_power_w`) удалить; в `1827-1836` оставить идеальную часть `freehub_power_w`;
  в `2456-2457` убрать `chain_dissipation_power_w`. `freehub_dissipation_power_w` остаётся (Р4);
- **телеметрия:** удалить 8 полей Р4 из `writer_types.hpp` (поля и строки спецификаций) и из обоих каталогов.
  `"elastic_chain"` в `transmission_model_labels` пометить резервом (Р3). В `test_catalog.cpp:43` число уменьшится на
  число удалённых листов: посчитай по каталогу. Допиши строку в комментарий над проверкой: «N = 487 minus the
  elastic-chain leaves …»;
- **стенды:** удалить `chain_locked_rig`, `chain_geometry_rig`, `compiled_chain_motion_rig` и режим `elastic_chain` в
  `validation/drivetrain.cpp:17` и `benchmark.cpp` вместе с регистрацией и параметрами CLI.

- [ ] **Шаг 4: Проверить, что ничего не осталось**

Выполни:
`rg -n 'elastic_chain|chain_k_n_m|chain_c_ns_m|freehub_k_nm_rad|freehub_c_nms_rad|hub_stiffness_nm_rad|hub_damping_nm_s|cassette_spin|simplified_' native/src CMakeLists.txt native/cmake`
Ожидается: только резервная метка в `writer_types.hpp`.

- [ ] **Шаг 5: Тесты и фикстуры конфигурации**

- Удали `tests/fixtures/config/variant_elastic_chain.toml`, `variant_elastic_chain.resolved.json`,
  `variant_elastic_chain_shifting_rejected.toml` и тесты, которые их читают.
- Поправь остальные тесты по списку `rg -l` из карты файлов.
- Убери удалённые ключи из `schema.json` и `resolved/*.json`, как в задаче 1, шаг 7.
- Отрицательный тест `mutated_drive_config_rejected_before_model_writes/chain_k_n_m-0.0` удали вместе с его эталонной
  записью.

- [ ] **Шаг 6: Узкие тесты**

Выполни: `mise run test 'config\.|schema|resolution|native_drivetrain|test_contracts|transmission|recording_catalog|validation'`
Ожидается: PASS, кроме узлов с эталонными записями. Список упавших запиши в `/tmp/lossless_failing_t2.txt`.

- [ ] **Шаг 7: Проверка этапа**

Выполни: `bash tools/check_change.sh "$(cat /tmp/lossless_base_t1)"`
Ожидается: сборка и анализ чистые. Падения обычного набора допустимы только для узлов из списков задач 1–2:
сравни. Иначе найди причину.

- [ ] **Шаг 8: Коммит**

```bash
git add -A native/src native/tests native/resources CMakeLists.txt native/cmake tests/fixtures/config
git commit -m "refactor(drive): delete the elastic-chain transmission

The chain is not modelled and the drive is lossless (user, 2026-11-05).
Removes the chain spring/damper, the elastic freehub, the cassette body
and the zero-valued chain_*/freehub_{deflection,energy} columns. The
transmission and force-kind ordinals keep reserved slots.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 3: Ядро видеокарты без потерь (только если пользователь подтвердил Р1)

**Файлы:** `native/src/gpu/drivetrain_resistance_kernel.hpp`, `native/tests/gpu/test_gpu_drivetrain_resistance.cpp`,
`tools/colab_gpu/README.md`.

- [ ] **Шаг 1: Падающий тест**

В `native/tests/gpu/test_gpu_drivetrain_resistance.cpp`:
```cpp
// AGENTS.md «Передача»: the GPU kernel matches the lossless CPU drive — a
// spinning wheel and crank with no torque inputs see no drive drag.
void test_gpu_drive_has_no_bearing_or_efficiency_loss() {
    GpuResistanceParams rp{};
    GpuDrivetrainParams dp{};
    bt::require(dp.efficiency == 1.0f, "drive efficiency is 100 %");
    const auto out = gpu_drivetrain_resistance_cpu_reference(rp, dp, /*wheel_rad_s=*/10.f, /*crank_rad_s=*/8.f,
                                                             /*human_nm=*/0.f, /*motor_nm=*/0.f);
    bt::require(out.wheel_bearing_nm == 0.f && out.crank_drag_nm == 0.f, "no bearing or crank drag");
}
```
Имена функций и полей результата возьми у соседних тестов (строки 595, 748). Если результата нет по отдельным
составляющим, проверь полную силу при нулевых входах и выключенном сопротивлении воздуха и качению.

- [ ] **Шаг 2: Удалить потери**

- Удалить поля `wheel_bearing_viscous_nm_s_rad`, `wheel_bearing_coulomb_nm`, `drivetrain_viscous_nm_s_rad` и их
  применение (`:99-106`, `:194-196`, `:231`, `:334-336`).
- Поставить `efficiency` в 1 или удалить поле вместе с умножением: найди его через `rg -n 'efficiency'
  native/src/gpu`.
- Трещотка (`:245-249`): если ей нужен демпфер для устойчивости, это вопрос пользователю. Не удаляй молча; запиши в
  «Не решено». Если демпфер не нужен для устойчивости (тест с нулём проходит), поставь
  `freehub_damping_nm_s_rad = 0`.
- В `tools/colab_gpu/README.md:29` убрать упоминание трения подшипников.

- [ ] **Шаг 3: Тесты видеокарты на процессоре**

Выполни: `mise run test 'gpu'`
Ожидается: PASS. Металл/CUDA в этом плане не запускаются.

- [ ] **Шаг 4: Проверка этапа и коммит**

`bash tools/check_change.sh <коммит задачи 2>` должен дать выход 0 по сборке и анализу. Затем:
```bash
git add native/src/gpu native/tests/gpu tools/colab_gpu/README.md
git commit -m "refactor(gpu): drop the kernel's bearing drag and 96 % drive efficiency

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 4: Перезаписать эталонные записи darwin

**Файлы:** `tests/fixtures/golden/darwin-arm64/**`, общие фикстуры (`tests/fixtures/{bootstrap,ft2,replay,recording,state,research}/**`).

- [ ] **Шаг 1: Снимок эпизодов до перезаписи**

Этот шаг выполняется **до задачи 1**, сразу после первой сборки базы. Он снимает «до» для сравнения:
```bash
build/native-release/bin/bike-recording golden compare > /tmp/lossless_episode_before.txt 2>&1; echo rc=$?
```
Если его пропустили, «до» берётся из `docs/superpowers/audits/2026-11-04-no-joint-damping-episode-after.txt`, и это
отмечается в коммите.
Точные аргументы `golden compare` и `golden freeze` возьми из `docs/superpowers/plans/2026-11-04-no-joint-damping.md`
(раздел Status, шаг 5) и `native/src/cli/recording.cpp:139-176`.

- [ ] **Шаг 2: Перезапись**

```bash
BIKE_GOLDEN_RECORD=1 ctest --test-dir build/native-release -LE 'package' -j8 --output-on-failure 2>&1 | tail -40
BIKE_GOLDEN_RECORD=1 ctest --test-dir build/native-release -L slow --output-on-failure 2>&1 | tail -40
git status --short tests/fixtures | wc -l
```
Режим записи хранит фактическое значение и сохраняет записи, которые и так в допуске (`native/tests/support/golden.cpp:233`,
`fixture_test.cpp:137`). Узел `golden_support` проверяет отказы, его в режиме записи не запускай: добавь
`-E golden_support`, если он попадает в выборку.

- [ ] **Шаг 3: Производные хэши**

Это шаги 2–5 плана 2026-11-04, раздел Status:
- **дайджесты моделей:** если изменились сохранённые `model.mjb`, замени старые дайджесты на новые в мета-файлах
  пакетов и документах состояния. Найди их: `rg -l '<старый дайджест>' tests/fixtures`;
- **контракт начального состояния** (`tests/fixtures/initial_state/darwin-arm64/*.json`): пересчитай хэши, если он
  меняется. Удалённые ключи входили в документ физики (`native/src/initialization/initial_state.cpp`);
- **`assets.json`** в `tests/fixtures/ft2/darwin-arm64/{research_flat,research_savage}`;
- **`replay.json`** в `tests/fixtures/replay/**`;
- **эпизоды**, замороженные ещё из Python (`episode.npz` и списки отсутствующих столбцов): удалить столбцы Р4 и
  `drive_bearings`.
Если меняется хэш, не перечисленный здесь, это условие О4.

- [ ] **Шаг 4: Самопроверка**

Выполни: `git diff --stat tests/fixtures | tail -5` и `git diff --name-only tests/fixtures | grep -v -E
'darwin-arm64|config/|bootstrap|ft2|replay|recording|state|research' | head`
Ожидается: второй вывод пуст, linux-aarch64 нет. Затем без режима записи:
`ctest --test-dir build/native-release -LE 'package' -j8` и `ctest --test-dir build/native-release -L slow`.
Ожидается: всё зелёное.

- [ ] **Шаг 5: Сравнение эпизодов после**

Выполни: `build/native-release/bin/bike-recording golden compare … > docs/superpowers/audits/2026-11-05-lossless-drivetrain-episode-after.txt`.
Если шаг 1 удался, сохрани и «до»: `…-episode-before.txt`. Каждое изменение должно объясняться удалением трения: чуть
больше скорость или работа, нет `drive_bearings` и столбцов Р4.

- [ ] **Шаг 6: Коммит**

```bash
git add -A tests/fixtures docs/superpowers/audits/2026-11-05-lossless-drivetrain-episode-*.txt
git commit -m "test(golden): re-record darwin goldens for the lossless drivetrain

Bearing drag and the elastic chain are removed (user, 2026-11-05).
linux-aarch64 goldens are stale until the container re-record.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 5: Документы и итоговое доказательство

**Файлы:** `docs/agents/simulation-decisions.md`, `docs/TESTING.md`, `docs/RIDE.md`,
`docs/adr/0003-native-authoritative-python-removal.md` (заметка о перезаписи, как в `14e9c054`).

- [ ] **Шаг 1: Найти устаревший текст**

Выполни:
`rg -n -i 'elastic.chain|bearing|chain_k|chain_c|freehub_k|freehub_c|96 ?%|efficiency' docs/*.md docs/agents docs/adr`
Каждое место про текущую модель исправь. Исторические записи в аудитах и планах не трогай.

- [ ] **Шаг 2: Файл решений**

В `docs/agents/simulation-decisions.md`:
- пункт 10 «Решено — не сделано» перенеси в «Действует», пункт 20, с коммитами;
- если трещотку на видеокарте оставили, добавь её в «Не решено».
Муфты (Р5) уже решены пользователем и записаны в «Решено — не сделано», пункт 1, здесь их не трогать.

- [ ] **Шаг 3: Итоговое доказательство**

Выполни: `bash tools/run_tests.sh full`. Ничего не запускать параллельно.
Ожидается: выход 0.

- [ ] **Шаг 4: Коммит документов, затем слияние — по решению пользователя**

```bash
git add docs
git commit -m "docs: record the lossless drivetrain

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
