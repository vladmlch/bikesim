# План старта, часть 2: таблица опыта и подготовка — план реализации

> **Для исполнителей-агентов:** обязательный навык — `superpowers:subagent-driven-development` (рекомендуется) или
> `superpowers:executing-plans`. Шаги отмечаются флажками (`- [ ]`). Исполнитель — Devin (`devin-orchestration`);
> финальное ревью ветки делает оркестратор.

Редакция 3 — после двух ревью codex: `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan2-review/review.md` и
`review-2.md` (ответы в конце).

**Цель.** Автор профиля задаёт старт только таблицей опыта райдера `[rider_launch]`. Звезда и угол педали на старте
вычисляются из неё по уклону в точке старта. Наклон корпуса из таблицы держится при успокоении позы и в первой позе
езды. Поведение поездок не меняется: во всех профилях таблица даёт сегодняшние значения.

**Архитектура.**
- Таблица — запись `rider::LaunchConfig` (раздел профиля `[rider_launch]`) и чистая функция
  `rider::prepare_launch(table, grade)`.
- Поля `drive.gearing.rear_teeth` и `drive.crank_phase_rad` остаются в структуре физики, их по-прежнему читают
  модель, писатели, проекция писателя и запись поездки. Автор их больше не пишет:
  - авторские документы с этими ключами отклоняются (файл профиля, документ поправок, флаги командной строки);
  - при загрузке и у стендов, которые обходят подготовку, оба поля вычисляются из таблицы для ровного старта
    (уклон 0);
  - всякий путь, который собирает модель или контракт из `SetupRequest`, сначала вызывает
    `initialization::prepare_request`. Она вычисляет оба поля для настоящего уклона старта и возвращает результат
    подготовки.
- Результат подготовки (`rider::LaunchPreparation`: звезда, угол, наклон) хранится в `SetupContext::preparation`.
  Через него наклон попадает в успокоение позы, в сброс намерений и в контракт переноса начального состояния.

**Стек.** C++23, MuJoCo, CTest (`bike_add_reference_test`), таблицы полей `FieldDecl` и загрузчик
`native/src/configuration/load.cpp`.

**Спецификация.** `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md`, разделы «Таблица опыта» и
«Устройство в коде» (подготовка). Дорожная карта — `docs/superpowers/plans/2026-11-11-rider-launch-plan.md` (её
задачи 6–7 заменены этим планом). Ревью всей работы — `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan-review/review.md`,
пункты I6, I7, I9, I13 и доля I17. Часть 1 — `docs/superpowers/plans/2026-11-11-launch-1-channels.md`.

## Решения плана

Спецификация задаёт цель; эти решения задают способ. Каждое опирается на код.

1. **В таблице части 2 только поля подготовки:** `start_cog_grade_breaks`, `start_cog_teeth`,
   `start_pedal_angle_rad`, `start_lean_grades`, `start_lean_rad`. Остальные поля таблицы и раздел реакции добавит
   часть 3 вместе с кодом, который их читает.
2. **Вычисленные поля остаются в структуре.** Их читают проекция писателя (`project_config.cpp:287-296`, схема
   провода `schema.cpp:593-605`), модель (`physical_topology.cpp:117`), начальная поза (`initial_pose.cpp:77`),
   запись и ключи. Меняется только источник значений.
3. **Отказ — на авторских входах, а не в `construct_config`.** Входы: `load_physics` (файл профиля),
   `resolve_physics` (документ поправок), `research::resolve_research_physics` (файл профиля исследования вместе с
   поправками флагов). Через них идут обычные запросы (`initialization/request.cpp:64-68`), запросы исследований
   (`research/configuration.cpp:245-249`), командная строка и проверки. Запись поездки — уже вычисленный документ,
   её читает `construct_config` (`replay/rebuild.cpp:145`), и оба поля в ней законны.
4. **Ровный старт там, где уклона нет.**
   - Загрузчик перед сборкой записи вычисляет оба поля из таблицы самого документа для уклона 0
     (`detail::with_flat_start`). Поэтому проверка «звезда есть в кассете» (`validate.cpp:648-652`) видит
     вычисленную звезду, а загруженная конфигурация несёт осмысленные значения.
   - Стенды проверок строят модель без `make_setup` (`validation/mechanics.hpp:21-30`). Они вызывают
     `configuration::resolve_flat_start(config)` — тот же расчёт для типизированной записи.
5. **Подготовка до первой проверки.** `prepare_request` вызывается до `post_init`:
   - в `make_setup` (`setup_owner.cpp:165`);
   - в среде исследований (`research/environment.cpp:18-19`);
   - при снятии начального состояния через `RuntimeRigAccess::initial_state` (`validation/runtime_access.cpp:80-85`):
     там на копии запроса, потому что `validation::Simulation` хранит исходный запрос.

   Тогда устаревшее значение поля по умолчанию (24) не проверяется против кассеты раньше, чем таблица даст своё.
   Среда исследований готовит сам запрос, поэтому его физика в записи (`research/metadata.cpp:125`) совпадает с
   физикой подготовки.
6. **Уклон подготовки** — `track.grade_profile->slope(start_x_m + articulated.road_lookahead_m)`, без профиля
   уклона — 0. Это тот же взгляд вперёд, что у ощущения `grade_ahead`, только от точки старта: до сборки модели
   положений колёс нет. Препятствия не учитываются: обычные поездки начинаются на ровном участке.
7. **Флаги командной строки пишут таблицу.**
   - `--crank-phase` → `rider_launch.start_pedal_angle_rad`;
   - `--rear-teeth` → таблица из одной звезды (`start_cog_teeth = [N]`, `start_cog_grade_breaks = []`).

   Запись хранит таблицу, поэтому видно, что задал пользователь.
8. **Файл профиля исследования — поправка поверх значений `bike-research` по умолчанию**, как сейчас для всех
   разделов (`research/configuration.cpp:178-202`). Файл без `[rider_launch]` берёт таблицу значений по умолчанию,
   так же как файл без `[monitors]` берёт их мониторы. Строгая загрузка полного профиля — это `load_physics`.
   Запрет двух вычисленных полей действует и здесь.
9. **Наклон при успокоении — только у шарнирного райдера и только если он не 0.**
   - Шаг отклоняет любую позу, если райдер не шарнирный (`control.cpp:40-44`).
   - Нулевой наклон оставляет пустую команду, которую успокоение посылало всегда.
   - У упрощённого райдера или без райдера наклона нет: корпуса нет.
10. **Ключи повторного использования.**
    - **Ключ кэша равновесия** строится из всего документа физики (`equilibrium_cache.cpp:163-181`), значит включает
      таблицу, а через неё — звезду, угол и наклон. Он и так меняется при каждой правке исходников
      (`source_identity`). Поэтому смена любого узла таблицы — промах кэша, даже если результат подготовки тот же:
      осторожное правило.
    - **Исторический ключ файлов NPZ** (ветка `legacy`, `:172`) убирает `rider_launch`, как он убирает
      `equilibrium_relaxation`: исторические данные таблицы не знали, а звезда и угол привязаны через `drive`.
      Ненулевой наклон добавляется в этот ключ членом `launch_lean_rad` (задача 3). Все исторические позы
      успокоены прямо, поэтому наклонный старт их не возьмёт, а ключ прямого старта остаётся историческим байт в
      байт.
    - **Контракт переноса начального состояния** (`initial_state.cpp:51-84`) от исходников не зависит. Сохранённые
      переносы должны оставаться годными:
      - раздел `rider_launch` из него убирается, как `research` и `monitors`;
      - звезда и угол остаются в нём через `drive`;
      - наклон входит отдельным членом `launch_lean_rad`, только если он не 0. Нет члена — значит прямой старт, с
        которым сняты все прежние переносы. Разные наклоны дают разные контракты; разные таблицы с одним
        результатом подготовки дают один контракт.
11. **Записи до части 2.** Документ физики без `[rider_launch]` отклоняется везде, в том числе при воспроизведении
    старых записей: читателей старых форм нет. Это касается входа `tests/fixtures/replay/legacy_v2/summary.json`
    теста `reference.replay_recording_v3`. Он уже красный и падает раньше, на контрольной сумме; это долг части 1
    (журнал очереди), он закрывается при финальном ревью.
12. **Значения «как сегодня»:**
    - 16 файлов физики, все сейчас пишут `rear_teeth = 51` → `start_cog_teeth = [51]`;
    - угол 0,0 — его не пишет никто, по умолчанию 0;
    - наклон `[0.0]` по уклонам `[0.0]` — успокоение сейчас шлёт пустую позу (`settle_contacts.cpp:14`), намерения
      после сброса начинают с 0 (`intent.cpp:106-112,246-257`);
    - `default_physics()` → `[24]` (34/24, `physical_config.hpp:149-152`);
    - программные конфигурации → их нынешняя звезда (опись ниже).

## Общие ограничения

- Лимиты, пороги, полосы, фильтры, задержки, силы, `strength`, `pedal_torque_ripple`, запас 25 Н и допуски
  тестов не меняются.
- У полей профиля нет значений по умолчанию в коде; читателей старых форм нет. Профиль без раздела или поля не
  загружается и называет ключ.
- Мотор, автомат переключений, тормоза и закон нажима не меняются.
- Успокоенная поза, собранная модель и начальное состояние не меняются бит в бит. Доказательство —
  сравнение с базовой сборкой (`ab_settle.sh`: все массивы файла позы и хэш модели совпадают). Поездки после
  старта этим сравнением не покрыты бит в бит. Для них доказательство другое: код езды не меняется (кроме
  `seed_lean(0)`, который пишет те же нули, что сброс), и прежние тесты поездок проходят. Отчёт называет
  доказательство именно так, без утверждения «траектории совпадают бит в бит».
- **Перезапись эталонов** (правило части 1; для части 2 пользователь подтвердил его 2026-11-11):
  - перезаписываются только выходы **зелёных** тестов, и только если единственное отличие — новые ключи, а все
    прежние значения совпадают бит в бит. Проверка —
    `python3 /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch1/check_rerecord.py --repo . --ref HEAD <файлы>`
    (список файлов через `xargs`), каждый файл `OK`;
  - если изменилось хоть одно прежнее значение, в том числе хэш (`configuration_sha256`, ключ профиля) — стоп,
    файл и отличие в отчёт, без перезаписи;
  - записанные выходы **красных** тестов не перезаписываются; входные файлы физики (`*.toml`) переводятся на новую
    форму во всех тестах, иначе отказ загрузчика сменит причину падения;
  - опись хэшей над документом физики, которые лежат в эталонах:
    - `configuration_sha256`: `tests/fixtures/ft2/*/research_*/provenance.json`, `.bin` тестов
      `native_research_environment` и `native_research_recording`, `tests/fixtures/replay/*/summary.json`;
    - все они принадлежат красным тестам и остаются как долг. Новый красный тест из-за хэша — стоп.
- Известные красные тесты: 22 обычных (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-10-delays-t5b-check/reds.txt`) и
  10 медленных (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-09-reds-final/slow_reds.txt`). Их причины сравниваются строго
  (`causes.py`: все строки отказа, нормализуются только пути) с журналом
  `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch1/task7/reds_output.log`, где есть все 32 теста. Допустимое
  отличие — только счётчик ключей или столбцов, выросший ровно на ключи этой части (как
  `native_runtime_rollout` 615→623 в части 1); каждое такое отличие — в отчёт с объяснением.
- Коммит: автор — git-пользователь; последняя строка ровно `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`;
  без строк про Devin; без push; без `git stash`; не трогать `.claude/` и `.agents/`; не требовать чистого
  `git status`.
- Без отладчиков. Каждая команда сборки или тестов — с ограничением времени (`perl -e 'alarm N; exec @ARGV' --`).
- Новые тестовые функции с большими локальными объектами (`SetupRequest`, `RuntimeBootstrap`) держат их в куче
  (`std::make_unique`) или в отдельных вспомогательных функциях. Файл `test_launch_preparation.cpp` снимает
  предупреждение о кадре так же, как `native/tests/initialization/test_setup.cpp:14-17,143`.

## Проверка задачи (одинакова для всех)

1. Целевые тесты задачи: `perl -e 'alarm 1800; exec @ARGV' -- mise run test '<шаблон>'`.
2. Коммит задачи, затем одна команда:

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
bash /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/check_task.sh <база задачи> \
  /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/verify-task<N> [ab]
```

   Скрипт делает три вещи:
   - запускает `check_change.sh` и отдельные проходы `cppcheck`/`frontends`/`tidy` по изменённым исходникам;
   - перепрогоняет все 32 известных красных с `--output-on-failure` и сравнивает причины (`causes.py`);
   - с `ab` — сравнивает успокоенную позу с базовой сборкой `9c9d2696`. Она собрана в
     `/Volumes/SSD_1TB_M2/work/wt/lane-flip`; оркестратор держит её собранной.

   Код выхода 0 только если: нет новых и нет починившихся красных, у каждого анализатора `findings: none`, причины
   совпадают, позы совпадают. Любое `STOP:` — исправление отдельным коммитом и повтор. Если `STOP` — допустимое
   отличие из «Общих ограничений», в отчёте приводится строка `cause_compare.txt` и объяснение.
3. В отчёт — вывод скрипта целиком и путь к каталогу.

Итоговая проверка всей части — задача 4.

## Бюджет тестов

| Что | Время |
|---|---|
| целевые тесты одной задачи (с тремя–шестью успокоениями позы) | 5–12 мин |
| `check_task.sh`: `check_change` + анализаторы | около 20 мин |
| причины 32 красных (медленные в их числе) | около 6 мин |
| `ab`: шесть успокоений через `bike-settle` | около 6 мин |
| задача 4: анализаторы целиком, обычные, медленные, пакетные | около 25 мин |

Итого три задачи по 40 мин плюс 25 мин ≈ 2 ч 25 мин машинного времени. База — прогон задачи 7 части 1
(`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch1/task7/`, код `5e0fe3a0`; после него до `9c9d2696` менялись только
документы). Повторять её не нужно.

## Опись мест, которые меняются

Карта — коммит мягкого старта `e8d082c3` (`git show --stat e8d082c3`): он добавил обязательные поля в каждый **полный
документ физики**. Каждый полный документ физики получает `[rider_launch]` (в коде — `rider_launch = {...}`).
Места, где менялся только `IntentConfig` (`seated_climb` без остальной физики), не трогаются.

- **Файлы физики (16)** — все пишут `rear_teeth = 51`, никто не пишет `crank_phase_rad`.
  `git grep -l '^\[monitors\]' -- '*.toml'`:
  - `examples/research/viewer_physics_{welded,pneumatic,distributed}.toml`;
  - `tests/fixtures/config/{viewer_physics_welded,viewer_physics_pneumatic,viewer_physics_distributed,variant_alt_support,variant_grip_weld_coercion}.toml`;
  - `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/{control_clock/env_request,native_distributed_tire/env_request,pneumatic_tire/env_fast}/physics.toml`;
  - `tests/fixtures/golden/episode/{pinned_savage_40,pinned_savage_40_f2}/physics.toml`.
- **Программные конфигурации в коде:**

  | Место | Звезда | Что ещё |
  |---|---|---|
  | `native/src/configuration/validate.cpp:1292` `default_physics()` | 24 | — |
  | `native/src/cli/configuration.cpp:208` (умолчания `bike-research`) | 51 | задача 2 убирает `rear_teeth = 51` |
  | `native/src/validation/drivetrain.cpp:23-27` `fixture_config` (стенд) | 51 | задача 2: `gearing.front_teeth=34` и `resolve_flat_start(c)` |
  | `native/src/validation/rider.cpp:21-24` `momentum_config` (стенд) | 51 | то же |
  | `native/src/validation/rider.cpp:34-36` `airborne_config` | 24 | задача 2: `resolve_flat_start(c)` |
  | `native/src/validation/scenarios.cpp:102-113` | 24 | звезду не задаёт; идёт через `make_setup` |
  | `native/src/validation/scenarios.cpp:168` | 24 | то же |

- **Программные конфигурации и встроенные документы в тестах:**
  - структуры: `native/tests/reference/replay_reference_support.hpp:103`,
    `native/tests/reference/research_reference_support.hpp:81`, `native/tests/reference/test_control_clock.cpp:29`,
    `native/tests/replay/test_session.cpp:48`, `native/tests/config/test_resolution.cpp:1437`, места
    `native/tests/config/test_schema.cpp` из `git show e8d082c3 -- native/tests/config/test_schema.cpp`. Звезда —
    та, что конфигурация задаёт сама; без явной звезды — 24;
  - встроенные TOML-строки: в каждом файле теста число `[rider_launch]` должно равняться числу `[monitors]` (полный
    документ физики всегда несёт `[monitors]`). Проверка в задаче 1, шаг 4.
- **Запись вычисленных полей в коде, которую убирает задача 2:**
  - `cli/configuration.cpp:89,208,221`;
  - `validation/drivetrain.cpp:26`, `validation/rider.cpp:23` (`rear_teeth` в `gearing={...}`);
  - `native/tests/initialization/test_setup.cpp:94` (`crank_phase_rad=.75`).

  Записи `rear_teeth` в `test_schema.cpp:232,944,1080` проверяют `post_init` самой структуры и через подготовку не
  проходят — не трогать.
- **Прямые конструкторы `SetupContext`:**
  - `native/src/initialization/setup_owner.cpp` (`make_setup`) — задача 2;
  - `native/src/validation/runtime_access.cpp:82` — задача 2;
  - тестовые `native/tests/reference/rider_control_reference_support.hpp:261-265` и
    `native/tests/initialization/test_sag.cpp:73-81` строят контекст для своих проверок без подготовки, с наклоном 0,
    как сейчас; не трогать.

---

### Задача 1. Таблица опыта в каждом профиле (поведение не меняется)

**Файлы:**
- Создать: `native/src/rider/launch_config.hpp`, `native/src/rider/launch_config.cpp`,
  `native/tests/reference/test_launch_preparation.cpp`
- Изменить:
  - `CMakeLists.txt:426-438` (`launch_config.cpp` в `bike_configuration`);
  - `native/tests/reference/Phase3Ports.cmake` (регистрация после `static_brake_latch`, `:307-309`);
  - `native/src/configuration/model_config.hpp`: псевдоним, член `rider_launch` после `seated_climb` (`:80`),
    объявление `post_init` (`:139-144`), таблица полей, корневой `FieldDecl` после `seated_climb` (`:292-295`);
  - `native/src/configuration/load.cpp:589-607` (`BIKE_T13_RECORD`);
  - `native/src/configuration/validate.cpp`: `detail::post_init`, проверки между записями в корневом `post_init`
    (`:1078-1092`), `default_physics()` (`:1292`);
  - `native/src/initialization/initial_state.cpp:77` (контракт без `rider_launch`);
  - `native/src/initialization/equilibrium_cache.cpp:172` (исторический ключ без `rider_launch`);
  - 16 файлов физики и программные конфигурации из описи (только добавление таблицы; `rear_teeth` пока остаётся).
- Тест: `native/tests/reference/test_launch_preparation.cpp`, `native/tests/reference/test_welded_profile_config.cpp`

**Интерфейсы:**
- Создаёт (`native/src/rider/launch_config.hpp`):

```cpp
// rider/launch_config.hpp — the rider's launch experience table (launch
// plan, part 2): the [rider_launch] profile section and the pure start
// preparation it drives. The profile authors only this table; setup
// derives drive.gearing.rear_teeth and drive.crank_phase_rad from it for
// the start grade, and the settle and the first running posture hold its
// lean. Part 3 adds the hold/release/effort/handover fields here.
#pragma once

#include <limits>
#include <vector>

namespace rider {

struct LaunchConfig {
    // Stepwise start cog: start_cog_teeth[i] applies for grades in
    // [start_cog_grade_breaks[i-1], start_cog_grade_breaks[i]); the breaks
    // ascend strictly and number one fewer than the cogs.
    std::vector<double> start_cog_grade_breaks;
    std::vector<int> start_cog_teeth;
    // The crank angle the rider sets the working pedal to, rad — the angle
    // drive.crank_phase_rad carries. NaN until a profile sets it: the
    // record has no code default.
    double start_pedal_angle_rad = std::numeric_limits<double>::quiet_NaN();
    // Torso lean by grade, rad: piecewise linear over strictly ascending
    // knots, held at the end values outside them.
    std::vector<double> start_lean_grades;
    std::vector<double> start_lean_rad;
};

// The start the table gives for one grade.
struct LaunchPreparation {
    int rear_teeth = 0;
    double crank_phase_rad = 0.;
    double lean_rad = 0.;
};

// The table's own shape and domain; throws std::invalid_argument naming the
// rider_launch field. Rules that need other records (cassette, posture
// bounds) live in configuration post_init.
void validate_launch_table(const LaunchConfig &table);

// Validates the table and the grade, then returns the stepwise cog, the
// fixed pedal angle and the interpolated lean for that grade.
[[nodiscard]] LaunchPreparation prepare_launch(const LaunchConfig &table,
                                               double grade);

} // namespace rider
```

- Создаёт в `bike::configuration`: `using RiderLaunchConfig = ::rider::LaunchConfig;`,
  `SimulationPhysicsConfig::rider_launch`, `detail::kRiderLaunchConfigSchema`,
  `detail::post_init(const RiderLaunchConfig&)`.
- Задачи 2 и 3 используют `rider::prepare_launch`, `rider::LaunchPreparation` и `physics.rider_launch`.

- [ ] **Шаг 1. Тесты (падают).** Новый файл `native/tests/reference/test_launch_preparation.cpp`:

```cpp
// Launch part 2 — the rider's launch table and the start preparation it
// drives: the pure table rules (task 1), the setup wiring of the cog and
// pedal angle (task 2) and of the lean (task 3).
#include "support/fixture_test.hpp"
#include "diag.hpp"
#include "rider/launch_config.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

NATIVE_DIAG_PUSH
// NATIVE_DIAG_IGNORE is the project diagnostic-pragma macro from diag.hpp; cppcheck cannot resolve it.
// cppcheck-suppress unknownMacro
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

namespace {

// Runs `action`, which must throw std::invalid_argument naming `needle`.
template <typename Action>
void rejects(Action &&action, std::string_view needle, std::string_view what) {
    bool threw = false;
    try {
        action();
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(std::string_view(failure.what()).find(needle) != std::string_view::npos,
                    std::string(what) + " names " + std::string(needle) + " (" + failure.what() + ")");
    }
    bt::require(threw, std::string(what) + " is rejected");
}

// A valid three-cog table with a two-knot lean ramp — the control every
// rejection below changes in exactly one place.
rider::LaunchConfig ramp_table() {
    return {.start_cog_grade_breaks = {.03, .08},
            .start_cog_teeth = {24, 33, 51},
            .start_pedal_angle_rad = .6,
            .start_lean_grades = {0., .2},
            .start_lean_rad = {0., .2}};
}

// The cog is stepwise and each interval is closed below: a grade equal to a
// break already takes the next cog; below the first break the first cog,
// above the last break the last one.
void test_cog_is_stepwise_by_grade() {
    const auto table = ramp_table();
    bt::require(rider::prepare_launch(table, -.05).rear_teeth == 24, "downhill takes the first cog");
    bt::require(rider::prepare_launch(table, .0299).rear_teeth == 24, "below the first break");
    bt::require(rider::prepare_launch(table, .03).rear_teeth == 33, "a break belongs to the next cog");
    bt::require(rider::prepare_launch(table, .0799).rear_teeth == 33, "below the second break");
    bt::require(rider::prepare_launch(table, .08).rear_teeth == 51, "the second break takes the last cog");
    bt::require(rider::prepare_launch(table, .3).rear_teeth == 51, "steep takes the last cog");
    rider::LaunchConfig single = table;
    single.start_cog_grade_breaks = {};
    single.start_cog_teeth = {51};
    bt::require(rider::prepare_launch(single, -.2).rear_teeth == 51 &&
                    rider::prepare_launch(single, .2).rear_teeth == 51,
                "a one-cog table starts on that cog at every grade");
}

// The lean interpolates linearly between knots and holds the end values
// outside them; the pedal angle is the table's, whatever the grade.
void test_lean_interpolates_and_angle_is_fixed() {
    const auto table = ramp_table();
    bt::require(std::abs(rider::prepare_launch(table, .05).lean_rad - .05) < 1e-15, "a quarter of the way up the ramp");
    bt::require(rider::prepare_launch(table, -.1).lean_rad == 0., "below the first knot holds the first lean");
    bt::require(rider::prepare_launch(table, .5).lean_rad == .2, "above the last knot holds the last lean");
    bt::require(rider::prepare_launch(table, .15).crank_phase_rad == .6, "the pedal angle comes from the table");
    rider::LaunchConfig flat = table;
    flat.start_lean_grades = {0.};
    flat.start_lean_rad = {.04};
    bt::require(rider::prepare_launch(flat, .3).lean_rad == .04, "one knot is a constant lean");
}

// Every shape or domain error names its field; the valid control passes.
void test_table_rules_name_the_field() {
    (void)rider::prepare_launch(ramp_table(), 0.);
    auto with = [](auto change) { auto t = ramp_table(); change(t); return t; };
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_teeth = {}; t.start_cog_grade_breaks = {}; })); },
            "start_cog_teeth", "an empty cog list");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_teeth = {24, 2, 51}; })); },
            "start_cog_teeth", "a two-tooth cog");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.03}; })); },
            "start_cog_grade_breaks", "one break too few");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.03, .03}; })); },
            "start_cog_grade_breaks", "equal breaks");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.08, .03}; })); },
            "start_cog_grade_breaks", "descending breaks");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_cog_grade_breaks = {std::numeric_limits<double>::quiet_NaN(), .08}; })); },
            "start_cog_grade_breaks", "a NaN break");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_pedal_angle_rad = std::numeric_limits<double>::infinity(); })); },
            "start_pedal_angle_rad", "an infinite pedal angle");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_grades = {}; t.start_lean_rad = {}; })); },
            "start_lean_grades", "no lean knot");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_rad = {0.}; })); },
            "start_lean_rad", "a lean count that differs from the knots");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_grades = {.2, 0.}; })); },
            "start_lean_grades", "descending lean knots");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_lean_rad = {0., std::numeric_limits<double>::quiet_NaN()}; })); },
            "start_lean_rad", "a NaN lean");
    rejects([&] { (void)rider::prepare_launch(ramp_table(), std::numeric_limits<double>::infinity()); },
            "grade", "an infinite preparation grade");
    const rider::LaunchConfig unset;
    rejects([&] { rider::validate_launch_table(unset); }, "start_cog_teeth", "a default-constructed table");
}

} // namespace

int main() {
    return bt::run([] {
        test_cog_is_stepwise_by_grade();
        test_lean_interpolates_and_angle_is_fixed();
        test_table_rules_name_the_field();
    });
}
NATIVE_DIAG_POP
```

  `Phase3Ports.cmake`, после `static_brake_latch`:

```cmake
# Launch part 2 — the rider's launch table and the start preparation:
# the pure table rules, the cog/pedal angle the setup computes from it and
# the lean the settle and the intent reset hold (tasks 1-3).
bike_add_reference_test(launch_preparation ENGINE
  SOURCES test_launch_preparation.cpp LIBRARIES bike_app bike_validation bike_cli
  LABELS launch rider)
```

  В `test_welded_profile_config.cpp` — помощники и тесты профиля (добавить `#include <functional>`):

```cpp
// Runs `action`, which must throw std::invalid_argument naming `needle`.
template <typename Action>
void rejects(Action &&action, std::string_view needle, std::string_view what) {
    bool threw = false;
    try {
        action();
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(std::string_view(failure.what()).find(needle) != std::string_view::npos,
                    std::string(what) + " names " + std::string(needle) + " (" + failure.what() + ")");
    }
    bt::require(threw, std::string(what) + " is rejected");
}

// Loads the shipped welded profile after `edit` changed its text. The two
// rider data files the profile names travel along, so only the edit
// decides whether the load succeeds.
[[gnu::noinline]] cfg::SimulationPhysicsConfig load_edited_profile(
    const std::function<void(std::string &)> &edit) {
    std::string text = bt::text(repo_file("examples/research/viewer_physics_welded.toml"));
    edit(text);
    const bt::TemporaryDirectory dir("welded-edited-profile");
    for (const char *name : {"rider_joint_envelope_anatomical.json", "rider_strength_reference.json"})
        std::filesystem::copy_file(repo_file(std::string("examples/research/") + name), dir.path() / name);
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << text; }
    return cfg::load_physics(path);
}

// Replaces the one occurrence of `from`; a missing or repeated line fails.
void replace_once(std::string &text, std::string_view from, std::string_view to) {
    const auto at = text.find(from);
    bt::require(at != std::string::npos && text.find(from, at + 1) == std::string::npos,
                "the profile text occurs exactly once: " + std::string(from));
    text.replace(at, from.size(), to);
}

// The welded profile carries today's start as its launch table: the 51T
// cog at every grade, the pedal at angle 0 and no lean. The built-in
// default profile keeps its own 34/24 drivetrain.
[[gnu::noinline]] void test_launch_table_reproduces_todays_start() {
    const auto &launch = profile().rider_launch;
    bt::require(launch.start_cog_grade_breaks.empty() &&
                    launch.start_cog_teeth == std::vector<int>{51},
                "one 51T start cog");
    bt::require(launch.start_pedal_angle_rad == 0., "pedal angle 0");
    bt::require(launch.start_lean_grades == std::vector<double>{0.} &&
                    launch.start_lean_rad == std::vector<double>{0.},
                "no start lean");
    const auto defaults = cfg::default_physics();
    bt::require(defaults.rider_launch.start_cog_teeth == std::vector<int>{24} &&
                    defaults.drive.gearing.front_teeth == 34 &&
                    defaults.drive.gearing.rear_teeth == 24,
                "the built-in profile starts on 34/24");
}

// Every [rider_launch] key and the section itself are required: dropping
// one fails the load and the error names it (no code default).
[[gnu::noinline]] void test_launch_table_keys_are_required() {
    for (const char *line : {"start_cog_grade_breaks = []\n", "start_cog_teeth = [51]\n",
                             "start_pedal_angle_rad = 0.0\n", "start_lean_grades = [0.0]\n",
                             "start_lean_rad = [0.0]\n"}) {
        const std::string key(std::string_view(line).substr(0, std::string_view(line).find(' ')));
        rejects([&] { (void)load_edited_profile([&](std::string &t) { replace_once(t, line, ""); }); },
                key, "a profile without " + key);
    }
    rejects([] {
        (void)load_edited_profile([](std::string &t) {
            const auto from = t.find("[rider_launch]\n");
            const std::string_view last = "start_lean_rad = [0.0]\n";
            const auto to = t.find(last, from);
            bt::require(from != std::string::npos && to != std::string::npos, "the profile has the launch block");
            t.erase(from, to + last.size() - from);
        });
    }, "rider_launch", "a profile without [rider_launch]");
}

// Rules that need other records: a start cog must be on the cassette while
// shifting runs (any cog when it does not), and a start lean must stay
// inside the seated posture bounds of [seated_climb].
[[gnu::noinline]] void test_launch_table_cross_record_rules() {
    auto config = profile();
    config.rider_launch.start_cog_teeth = {50};
    rejects([&] { cfg::validate(config); }, "start_cog_teeth", "a start cog off the cassette");
    config.drive.shifting.enabled = false;
    cfg::validate(config);   // control: any cog >= 3 without shifting
    config = profile();
    config.rider_launch.start_lean_rad = {config.seated_climb.max_forward_lean_rad + 1e-3};
    rejects([&] { cfg::validate(config); }, "start_lean_rad", "a lean beyond the forward bound");
    config.rider_launch.start_lean_rad = {-config.seated_climb.max_backward_lean_rad - 1e-3};
    rejects([&] { cfg::validate(config); }, "start_lean_rad", "a lean beyond the backward bound");
    config.rider_launch.start_lean_rad = {config.seated_climb.max_forward_lean_rad};
    cfg::validate(config);   // control: the bound itself is a valid lean
}
```

  Вызовы трёх новых тестов добавить в `main` этого файла.
- [ ] **Шаг 2.** `mise run test 'launch_preparation|welded_profile_config'` — сборка падает: нет
  `rider/launch_config.hpp` и `rider_launch`.
- [ ] **Шаг 3. Код.**
  - `launch_config.cpp`:

```cpp
// rider/launch_config.cpp — the launch table rules and the pure start
// preparation (launch plan, part 2).
#include "launch_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rider {
namespace {

[[noreturn]] void reject(std::string_view field, std::string_view why) {
    throw std::invalid_argument("rider_launch." + std::string(field) + ": " + std::string(why));
}

void strictly_ascending(const std::vector<double> &values, std::string_view field) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i])) reject(field, "values must be finite");
        if (i > 0 && !(values[i - 1] < values[i])) reject(field, "values must ascend strictly");
    }
}

} // namespace

void validate_launch_table(const LaunchConfig &table) {
    if (table.start_cog_teeth.empty()) reject("start_cog_teeth", "needs at least one cog");
    for (const int teeth : table.start_cog_teeth)
        if (teeth < 3) reject("start_cog_teeth", "a cog has at least three teeth");
    if (table.start_cog_grade_breaks.size() + 1 != table.start_cog_teeth.size())
        reject("start_cog_grade_breaks", "needs exactly one break fewer than start_cog_teeth");
    strictly_ascending(table.start_cog_grade_breaks, "start_cog_grade_breaks");
    if (!std::isfinite(table.start_pedal_angle_rad)) reject("start_pedal_angle_rad", "must be finite");
    if (table.start_lean_grades.empty()) reject("start_lean_grades", "needs at least one knot");
    if (table.start_lean_rad.size() != table.start_lean_grades.size())
        reject("start_lean_rad", "needs one lean per start_lean_grades knot");
    strictly_ascending(table.start_lean_grades, "start_lean_grades");
    for (const double lean : table.start_lean_rad)
        if (!std::isfinite(lean)) reject("start_lean_rad", "values must be finite");
}

LaunchPreparation prepare_launch(const LaunchConfig &table, double grade) {
    validate_launch_table(table);
    if (!std::isfinite(grade)) throw std::invalid_argument("launch preparation grade must be finite");
    // upper_bound counts the breaks at or below the grade: a grade equal to
    // a break already takes the next cog.
    const auto &breaks = table.start_cog_grade_breaks;
    const auto cog = static_cast<std::size_t>(
        std::distance(breaks.begin(), std::ranges::upper_bound(breaks, grade)));
    const auto &knots = table.start_lean_grades;
    const auto &leans = table.start_lean_rad;
    double lean = leans.front();
    if (grade >= knots.back()) {
        lean = leans.back();
    } else if (grade > knots.front()) {
        const auto upper = static_cast<std::size_t>(
            std::distance(knots.begin(), std::ranges::upper_bound(knots, grade)));
        const double t = (grade - knots[upper - 1]) / (knots[upper] - knots[upper - 1]);
        lean = leans[upper - 1] + t * (leans[upper] - leans[upper - 1]);
    }
    return {.rear_teeth = table.start_cog_teeth[cog],
            .crank_phase_rad = table.start_pedal_angle_rad,
            .lean_rad = lean};
}

} // namespace rider
```

  - `CMakeLists.txt`, в список `bike_configuration` после `profile_writer.cpp`:
    `"${_bike_native_dir}/src/rider/launch_config.cpp"`. Загрузчику в задаче 2 нужна `prepare_launch`, а
    `bike_configuration` не зависит от `bike_core`.
  - `model_config.hpp`:
    - `#include "../rider/launch_config.hpp"`;
    - после `using SeatedClimbConfig = ...`:

```cpp
// Launch part 2 — the rider's launch experience table ([rider_launch]):
// the record is the rider-layer LaunchConfig, like SeatedClimbConfig.
using RiderLaunchConfig = ::rider::LaunchConfig;
```

    - в `SimulationPhysicsConfig` после `SeatedClimbConfig seated_climb;` — `RiderLaunchConfig rider_launch;`;
    - в блок `namespace detail` объявлений (`:139-144`) — `void post_init(const RiderLaunchConfig&);`;
    - перед `kDeclaredSimulationPhysicsConfig`:

```cpp
// Launch part 2 — [rider_launch]: every key is required and the record has
// no declared-default instance, like [monitors].
inline constexpr auto kRiderLaunchConfigFields = std::to_array<FieldDecl>({
    {.name="start_cog_grade_breaks", .type="tuple[float, ...]", .kind=FieldKind::RealTuple,
     .offset=offsetof(RiderLaunchConfig, start_cog_grade_breaks), .required=true, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="start_cog_teeth", .type="tuple[int, ...]", .kind=FieldKind::IntegerTuple,
     .offset=offsetof(RiderLaunchConfig, start_cog_teeth), .required=true, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="start_pedal_angle_rad", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(RiderLaunchConfig, start_pedal_angle_rad), .required=true, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="start_lean_grades", .type="tuple[float, ...]", .kind=FieldKind::RealTuple,
     .offset=offsetof(RiderLaunchConfig, start_lean_grades), .required=true, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="start_lean_rad", .type="tuple[float, ...]", .kind=FieldKind::RealTuple,
     .offset=offsetof(RiderLaunchConfig, start_lean_rad), .required=true, .factory="",
     .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kRiderLaunchConfigSchema{
    .path="bike_sim.physics.rider_launch.RiderLaunchConfig",
    .fields=kRiderLaunchConfigFields, .declared_default=nullptr};
```

    - в `kSimulationPhysicsConfigFields` после записи `seated_climb` (`:292-295`):

```cpp
    {.name="rider_launch",
     .type="<class 'bike_sim.physics.rider_launch.RiderLaunchConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(SimulationPhysicsConfig, rider_launch),
     .required=true, .factory="", .nested=&kRiderLaunchConfigSchema, .nested2=nullptr},
```

  - `load.cpp`, в `construct_record` после строки `SeatedClimbConfig`:
    `BIKE_T13_RECORD(RiderLaunchConfig, kRiderLaunchConfigSchema)`.
  - `validate.cpp`:
    - в `namespace detail`, рядом с `post_init(const MonitorsConfig &)` (`:886`):

```cpp
void post_init(const RiderLaunchConfig &config) {
    // Launch part 2 — the table's own rules; the cassette and posture
    // bounds need other records and run in the root post_init.
    ::rider::validate_launch_table(config);
}
```

    - в корневом `post_init` после `detail::post_init(config.research);`:

```cpp
    detail::post_init(config.rider_launch);
    // The start cogs are cassette cogs while shifting runs (the computed
    // start cog must pass the same membership rule as drive.gearing), and
    // the start leans stay inside the seated posture bounds the intent
    // clamps to.
    if (config.drive.shifting.enabled)
        for (const int teeth : config.rider_launch.start_cog_teeth)
            if (std::ranges::find(config.drive.shifting.cassette, teeth) ==
                config.drive.shifting.cassette.end())
                fail("rider_launch.start_cog_teeth: cog " + std::to_string(teeth) +
                     " is not on the cassette");
    for (const double lean : config.rider_launch.start_lean_rad)
        if (lean > config.seated_climb.max_forward_lean_rad ||
            lean < -config.seated_climb.max_backward_lean_rad)
            fail("rider_launch.start_lean_rad: lean outside the seated posture bounds");
```

    - в `default_physics()` после блока `seated_climb` (в комментарии над функцией дописать «and the 24T launch
      table»):

```cpp
    // Launch part 2 — the built-in profile starts on its own 34/24
    // drivetrain at angle 0 without lean (today's start).
    config.rider_launch = {.start_cog_grade_breaks = {},
                           .start_cog_teeth = {24},
                           .start_pedal_angle_rad = 0.,
                           .start_lean_grades = {0.},
                           .start_lean_rad = {0.}};
```

  - `initial_state.cpp:77`, в той же строке, что убирает `research` и `monitors`, убрать и `rider_launch`:
    `std::erase_if(config,[](const auto& p){return p.first=="research"||p.first=="monitors"||p.first=="rider_launch";});`.
    Над ней дописать в комментарий: «[rider_launch] (launch part 2) shapes the start only through the computed
    drive members, bound above, and the prepared lean, bound explicitly when nonzero — so the table itself is
    dropped and every transfer recorded before it keeps validating».
  - `equilibrium_cache.cpp:172`: строку исторической ветки
    `if(legacy)std::erase_if(fields,[](const auto& field){return field.first=="equilibrium_relaxation";});` заменить
    на `if(legacy)std::erase_if(fields,[](const auto& field){return field.first=="equilibrium_relaxation"||field.first=="rider_launch";});`
    и дописать в комментарий над ней: «historical payloads also predate [rider_launch]; their cog and angle stay
    bound through drive».
  - 16 файлов физики — одним скриптом (блок перед `[monitors]`):

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
python3 - <<'EOF'
import pathlib, subprocess
block = '''# Rider launch experience table (launch plan, part 2): start cog by grade,
# working pedal angle, torso lean by grade. Preliminary values that keep
# today's start (51T cog, pedal angle 0, no lean); the launch plan
# calibrates them later.
[rider_launch]
start_cog_grade_breaks = []
start_cog_teeth = [51]
start_pedal_angle_rad = 0.0
start_lean_grades = [0.0]
start_lean_rad = [0.0]

'''
files = subprocess.run(['git', 'grep', '-l', r'^\[monitors\]', '--', '*.toml'],
                       capture_output=True, text=True, check=True).stdout.split()
assert len(files) == 16, files
for name in files:
    path = pathlib.Path(name)
    text = path.read_text()
    assert text.count('\n[monitors]\n') == 1 and '[rider_launch]' not in text, name
    assert text.count('\nrear_teeth = 51\n') == 1, name
    path.write_text(text.replace('\n[monitors]\n', '\n' + block + '[monitors]\n', 1))
print('updated', len(files))
EOF
```

  - Программные конфигурации из описи (звезда — по описи; в коде — сразу после установки `seated_climb`):

```cpp
    c.rider_launch = {.start_cog_grade_breaks = {}, .start_cog_teeth = {51},
                      .start_pedal_angle_rad = 0., .start_lean_grades = {0.},
                      .start_lean_rad = {0.}};
```

    В умолчаниях `bike-research` (`cli/configuration.cpp:208`) — то же с `{51}` (`defaults->rider_launch = ...`).
    Во встроенных TOML-строках тестов — тот же блок `[rider_launch]` перед `[monitors]`, со звездой конфигурации
    (`[51]`, если строка пишет `rear_teeth = 51`, иначе `[24]`).
- [ ] **Шаг 4. Тесты проходят.**
  - `perl -e 'alarm 2400; exec @ARGV' -- mise run test 'launch_preparation|welded_profile_config|seated_pedaling_cycle|realistic_pedelec_acceptance|equilibrium_state_file|initialization_setup|initialization_transfer|initialization_sag|cli_capture|assist_controller|pneumatic_tire_config|pneumatic_physical_runtime|articulated_config|validation|replay_session'`.
    Новые тесты зелёные, прежние — как в базе.
  - Встроенные документы переведены все — команда печатает пустой список:

```bash
for f in $(git grep -l '\[monitors\]' -- native/tests); do
  m=$(grep -o '\[monitors\]' "$f" | wc -l); r=$(grep -o '\[rider_launch\]' "$f" | wc -l)
  [ "$m" -eq "$r" ] || echo "$f monitors=$m rider_launch=$r"
done
```

- [ ] **Шаг 5. Перезаписи.** Если зелёный тест падает только из-за нового раздела в записанном выходе —
  перезаписать выход способом перезаписи этого теста и проверить `check_rerecord.py`. Список и результат — в отчёт;
  если таких нет — так и написать.
- [ ] **Шаг 6. Коммит и проверка.** `feat(config): the rider launch table in every physics profile`, затем
  `check_task.sh 9c9d2696 .../verify-task1 ab`.

### Задача 2. Звезда и угол педали вычисляются из таблицы

Одна задача, потому что отказ старых ключей, удаление `rear_teeth = 51` из профилей и подготовка должны появиться
вместе: иначе профили либо не загрузятся, либо поедут на 24.

**Файлы:**
- Создать: `native/src/initialization/launch_preparation.hpp`, `native/src/initialization/launch_preparation.cpp`
- Изменить:
  - `native/cmake/NativeStartup.cmake:92` (`launch_preparation.cpp` в `bike_initialization`, после `initial_state.cpp`);
  - `native/src/configuration/load.hpp`, `native/src/configuration/load.cpp` (`load_physics`, `resolve_physics`,
    `resolve_flat_start`, `detail::reject_computed_members`, `detail::with_flat_start`,
    `detail::erase_computed_members`);
  - `native/src/configuration/profile_writer.cpp:195-204`;
  - `native/src/research/configuration.cpp:195-202`;
  - `native/src/research/environment.cpp:18-19`;
  - `native/src/initialization/context.hpp` (поле `preparation`);
  - `native/src/initialization/setup_owner.cpp:162-195`;
  - `native/src/validation/runtime_access.cpp:80-85`;
  - `native/src/cli/configuration.cpp:81-99,195-225`;
  - `native/src/validation/drivetrain.cpp:23-27`, `native/src/validation/rider.cpp:21-24,34-36`;
  - 16 файлов физики (убрать строку `rear_teeth = 51`);
  - `native/tests/initialization/test_setup.cpp:94`.
- Тест: `test_launch_preparation.cpp`, `test_welded_profile_config.cpp`, `native/tests/cli/test_capture.cpp`

**Интерфейсы:**
- Использует: `rider::prepare_launch`, `rider::LaunchPreparation`, `physics.rider_launch` (задача 1).
- Создаёт:

```cpp
// initialization/launch_preparation.hpp
namespace bike::initialization {
[[nodiscard]] double rider_start_grade(const terrain::TrackSpec &track,
                                       double start_x_m, double lookahead_m);
::rider::LaunchPreparation prepare_start(configuration::SimulationPhysicsConfig &physics,
                                         const terrain::TrackSpec &track, double start_x_m);
::rider::LaunchPreparation prepare_request(SetupRequest &request);
}
// initialization/context.hpp, SetupContext (after start_x_m)
::rider::LaunchPreparation preparation{};
// configuration/load.hpp
void resolve_flat_start(SimulationPhysicsConfig &config);           // public
void detail::reject_computed_members(const bike::io::Value &authored);
[[nodiscard]] bike::io::Value detail::with_flat_start(bike::io::Value document);
void detail::erase_computed_members(bike::io::Value &document);
```

- Задача 3 читает `SetupContext::preparation.lean_rad`.

- [ ] **Шаг 1. Тесты (падают).**
  - `test_welded_profile_config.cpp`:

```cpp
// The two computed drive members are never authored: a profile that still
// names either one is rejected, and the error points at the table field
// that replaced it. An override document is held to the same rule.
[[gnu::noinline]] void test_computed_drive_members_are_not_authored() {
    rejects([] { (void)load_edited_profile([](std::string &t) {
                replace_once(t, "[drive.gearing]\n", "[drive.gearing]\nrear_teeth = 51\n"); }); },
            "start_cog_teeth", "a profile that authors drive.gearing.rear_teeth");
    rejects([] { (void)load_edited_profile([](std::string &t) {
                replace_once(t, "[drive]\n", "[drive]\ncrank_phase_rad = 0.0\n"); }); },
            "start_pedal_angle_rad", "a profile that authors drive.crank_phase_rad");
    using bike::io::Number;
    using bike::io::Value;
    const Value cog{Value::Object{{"rear_teeth", Value{Number{"39"}}}}};
    const Value gearing{Value::Object{{"gearing", cog}}};
    const Value gear{Value::Object{{"drive", gearing}}};
    rejects([&] { (void)cfg::resolve_physics(gear, profile()); },
            "start_cog_teeth", "an override that authors the rear cog");
}

// Loading resolves the computed members for a flat start from the
// document's own table: the shipped profile carries 51T at angle 0, and a
// stepwise table puts grade 0 on the cog of its middle interval. An
// override of the table re-resolves them after the merge.
[[gnu::noinline]] void test_load_resolves_a_flat_start() {
    bt::require(profile().drive.gearing.rear_teeth == 51 && profile().drive.crank_phase_rad == 0.,
                "the shipped profile resolves to 51T at angle 0");
    const auto stepwise = load_edited_profile([](std::string &t) {
        replace_once(t, "start_cog_grade_breaks = []", "start_cog_grade_breaks = [-0.01, 0.05]");
        replace_once(t, "start_cog_teeth = [51]", "start_cog_teeth = [21, 39, 51]");
        replace_once(t, "start_pedal_angle_rad = 0.0", "start_pedal_angle_rad = 0.5");
    });
    bt::require(stepwise.drive.gearing.rear_teeth == 39 && stepwise.drive.crank_phase_rad == .5,
                "grade 0 takes the middle cog and the table's angle");
    using bike::io::Value;
    const Value teeth{Value::Array{Value{bike::io::Number{"45"}}}};
    const Value launch{Value::Object{{"start_cog_teeth", teeth}}};
    const Value table{Value::Object{{"rider_launch", launch}}};
    bt::require(cfg::resolve_physics(table, profile()).drive.gearing.rear_teeth == 45,
                "an overridden table re-resolves the start cog");
}

// A written profile authors only the table (round trip normalized to a
// flat start): write_physics_toml leaves the two computed members out, the
// text loads back with the same table, and the computed members come back
// as the table's flat start — for a stepwise table too.
[[gnu::noinline]] void test_written_profile_authors_only_the_table() {
    auto stepwise = profile();
    stepwise.rider_launch.start_cog_grade_breaks = {-.01, .05};
    stepwise.rider_launch.start_cog_teeth = {21, 39, 51};
    stepwise.drive.gearing.rear_teeth = 51;   // a prepared value for a steep start
    const auto text = cfg::write_physics_toml(stepwise);
    bt::require(text.find("rear_teeth") == std::string::npos &&
                    text.find("crank_phase_rad") == std::string::npos,
                "the writer leaves out the computed members");
    const bt::TemporaryDirectory dir("welded-written-profile");
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << text; }
    const auto reread = cfg::load_physics(path);
    bt::require(reread.rider_launch.start_cog_teeth == stepwise.rider_launch.start_cog_teeth &&
                    reread.rider_launch.start_cog_grade_breaks == stepwise.rider_launch.start_cog_grade_breaks,
                "the written table loads back unchanged");
    bt::require(reread.drive.gearing.rear_teeth == 39 && reread.drive.crank_phase_rad == 0.,
                "the computed members come back as the flat start");
}

// A recorded (resolved) physics document keeps its prepared members: the
// reader for recordings does not re-resolve them. A document without the
// launch table — a recording from before part 2 — is rejected and named.
[[gnu::noinline]] void test_recorded_document_keeps_the_prepared_start() {
    auto prepared = profile();
    prepared.rider_launch.start_cog_grade_breaks = {.03};
    prepared.rider_launch.start_cog_teeth = {24, 39};
    prepared.rider_launch.start_pedal_angle_rad = .6;
    prepared.drive.gearing.rear_teeth = 39;   // what setup prepares on a 4 % start
    prepared.drive.crank_phase_rad = .6;
    const auto recorded = cfg::detail::plain_document(cfg::to_value(prepared));
    const auto rebuilt = cfg::detail::construct_config(recorded);
    bt::require(rebuilt.drive.gearing.rear_teeth == 39 && rebuilt.drive.crank_phase_rad == .6,
                "the recorded prepared start survives the recording reader");
    auto old = recorded;
    std::erase_if(std::get<bike::io::Value::Object>(old.data),
                  [](const auto &member) { return member.first == "rider_launch"; });
    rejects([&] { (void)cfg::detail::construct_config(old); }, "rider_launch",
            "a recorded document from before the launch table");
}
```

    Строки `[drive]\n` и `[drive.gearing]\n` встречаются в профиле ровно один раз (`viewer_physics_welded.toml:56,120`).
  - `test_launch_preparation.cpp` (добавить `#include "initialization/setup_owner.hpp"`,
    `#include "initialization/launch_preparation.hpp"`, `#include "research/environment.hpp"` (там же
    `resolve_research_physics`, `:102`), `#include "configuration/load.hpp"`, `#include "launch_ride_support.hpp"`,
    `#include "validation/scenarios.hpp"`, `#include "cli/application.hpp"`, `#include "io/value.hpp"`,
    `#include "support/temporary_directory.hpp"`, `<filesystem>`, `<fstream>`, `<memory>`, `<vector>`):

```cpp
namespace init = bike::initialization;
namespace cfg = bike::configuration;

// A member of a JSON-like document; a missing member fails the test.
const bike::io::Value &member(const bike::io::Value &object, std::string_view key) {
    const auto *found = bike::io::find_member(bike::io::as_object(object, key), key);
    bt::require(found != nullptr, "the document names " + std::string(key));
    return *found;
}

// The welded flat request with a real settle on a constant grade: no
// transferred initial state, no cache, held brakes (the
// slope_brake_and_crank recipe of test_setup.cpp). Heap-owned: a
// SetupRequest is several KB.
std::unique_ptr<init::SetupRequest> settle_request(double grade) {
    auto request = std::make_unique<init::SetupRequest>(
        init::read_setup_request(bt::fixture("requests/welded_flat.toml")));
    auto &physics = request->model.physics.value(); // NOLINT(bugprone-unchecked-optional-access) the fixture request carries physics
    physics.equilibrium_cache_enabled = false;
    physics.equilibrium_state_file.reset();
    physics.initial_speed_mps = 0.;
    physics.initial_front_brake = 1.;
    physics.initial_rear_brake = 1.;
    request->initial_state.reset();
    request->track.grade_profile = bike::terrain::GradeProfile{{{0., grade}, {request->track.length_m, grade}}};
    request->model.field = bike::terrain::HeightFieldSpec::for_track(request->track);
    return request;
}

// The 24/39 table switching at 3 %, pedal at .6 rad.
void stepwise_cog(init::SetupRequest &request) {
    auto &launch = request.model.physics->rider_launch; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    launch.start_cog_grade_breaks = {.03};
    launch.start_cog_teeth = {24, 39};
    launch.start_pedal_angle_rad = .6;
}

// The rider judges the grade road_lookahead_m ahead of the start point on
// the track's grade profile; a track without one is flat.
void test_start_grade_looks_ahead_of_the_start() {
    auto track = bike::terrain::flat(40.);
    bt::require(init::rider_start_grade(track, 2., 2.) == 0., "no grade profile is flat");
    track.grade_profile = bike::terrain::GradeProfile{{{0., 0.}, {10., 0.}, {10.5, .1}, {40., .1}}};
    bt::require(init::rider_start_grade(track, 2., 2.) == 0., "flat ahead of a flat start");
    bt::require(init::rider_start_grade(track, 9., 2.) == .1, "the climb two metres ahead");
    rejects([&] { (void)init::rider_start_grade(track, 2., -1.); }, "lookahead", "a negative lookahead");
}

// Setup prepares the start from the table at the start grade before the
// model exists: on a 4 % start the 24/39 table compiles the 39T cog, and
// cog and angle reach the setup metadata and the sealed bootstrap. The
// crank coordinate itself follows the table: the same start with the pedal
// at angle 0 settles to a different crank phase.
[[gnu::noinline]] void test_setup_prepares_cog_and_angle_for_the_start_grade() {
    auto request = settle_request(.04);
    stepwise_cog(*request);
    auto owner = init::make_setup(*request);
    const auto result = init::settle(*owner);
    bt::require(result.initial_crank_phase_rad == .6, "the settle starts from the table's pedal angle");
    const auto metadata = owner->metadata();
    const auto &drive = member(member(metadata, "physics"), "drive");
    bt::require(bike::io::as_int64(member(member(drive, "gearing"), "rear_teeth"), "rear_teeth") == 39,
                "the setup physics names the prepared cog");
    const auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(owner->seal_bootstrap());
    bt::require(bootstrap->state.drive.shifting.rear_teeth == 39, "the ride starts on the prepared cog");
    request->model.physics->rider_launch.start_pedal_angle_rad = 0.; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    auto upright_pedal = init::make_setup(*request);
    bt::require(init::settle(*upright_pedal).settled_crank_phase_rad != result.settled_crank_phase_rad,
                "the pedal angle moves the actual crank coordinate");
}

// A programmatic configuration need not repair the computed cog: with a
// cassette that lacks the stale default 24, setup prepares the table's cog
// before any cassette check runs.
[[gnu::noinline]] void test_setup_prepares_before_validating() {
    auto request = settle_request(0.);
    auto &physics = request->model.physics.value(); // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    physics.drive.shifting.cassette = {39, 51};
    physics.drive.gearing.rear_teeth = 24;   // the stale struct default
    physics.rider_launch.start_cog_teeth = {51};
    auto owner = init::make_setup(*request);
    const auto metadata = owner->metadata();   // owned: member() borrows from it
    const auto &drive = member(member(metadata, "physics"), "drive");
    bt::require(bike::io::as_int64(member(member(drive, "gearing"), "rear_teeth"), "rear_teeth") == 51,
                "setup prepared 51T from the table");
}

// The research recording and the research ride name the prepared cog: the
// research environment prepares its own request before anything reads it.
[[gnu::noinline]] void test_research_records_the_prepared_cog() {
    auto owner = std::make_unique<bike::research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = cfg::load_physics(
        launch_ride::repo_file("examples/research/viewer_physics_welded.toml"));
    auto &physics = *request.setup.model.physics;
    physics.initial_speed_mps = 0.;
    physics.rider_launch.start_cog_grade_breaks = {.03};
    physics.rider_launch.start_cog_teeth = {24, 39};
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider = bike::model::rider_from_args("articulated_planar");
    request.setup.track = launch_ride::track("[[0.0, 0.04], [60.0, 0.04]]", 60.);
    request.experiment.duration_s = .01;
    request.experiment.record_decimation = 1;
    request.experiment.maximum_energy_residual_ratio = 1e9;
    request.experiment.stop_on_model_violation = false;
    request.sensors = bike::research::SensorConfig{};
    request.experiment.control_period_s = physics.research.control_period_s;
    request.experiment.actuator_delay_s = physics.research.actuator_delay_s;
    request.sensors.delivery.latency_s = physics.research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s = physics.research.sensor_maximum_age_s;
    request.sensors.sample_period_s = .005;
    const auto prepared = bike::research::prepare_environment(std::move(owner));
    const auto &resolved = member(prepared->metadata, "resolved_config");
    const auto &drive = member(member(resolved, "physics"), "drive");
    bt::require(bike::io::as_int64(member(member(drive, "gearing"), "rear_teeth"), "rear_teeth") == 39,
                "the recording names the prepared cog");
    const auto &table = member(member(resolved, "physics"), "rider_launch");
    bt::require(bike::io::as_array(member(table, "start_cog_teeth"), "start_cog_teeth").size() == 2,
                "the recording keeps the authored table");
    bt::require(prepared->bootstrap.ride.state.drive.shifting.rear_teeth == 39,
                "the research ride starts on it");
}

// The validation rig captures the initial state of the start that was
// actually prepared: a transfer captured through validation::Simulation on
// the 4 % stepwise start validates when the same request reuses it.
[[gnu::noinline]] void test_validation_capture_matches_the_prepared_start() {
    auto request = settle_request(.04);
    stepwise_cog(*request);
    auto simulation = std::make_unique<bike::validation::Simulation>(*request);
    auto transfer = std::make_unique<init::SetupRequest>(*request);
    transfer->initial_state = simulation->initial_state();
    auto reused = init::make_setup(*transfer);
    bt::require(init::settle(*reused).initial_state_sha256.has_value(),
                "the rig capture validates for the same prepared start");
}

// The research profile path holds the same rule as load_physics: a profile
// file that authors a computed member is rejected.
[[gnu::noinline]] void test_research_profile_cannot_author_the_cog() {
    const bt::TemporaryDirectory dir("research-authored-cog");
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << "[drive.gearing]\nrear_teeth = 39\n"; }
    rejects([&] { (void)bike::research::resolve_research_physics(
                cfg::default_physics(), path, bike::io::Value{bike::io::Value::Object{}}); },
            "start_cog_teeth", "a research profile that authors the rear cog");
}
```

    Вызовы — в `main`. `EquilibriumResult::initial_state_sha256` — `std::optional<std::string>`
    (`native/src/initialization/equilibrium.hpp:33`).
  - `native/tests/cli/test_capture.cpp`:

```cpp
// The start flags author the launch table, never the computed drive
// members: --crank-phase sets the table's pedal angle, --rear-teeth a
// one-cog table, so the recorded physics shows what the user asked for.
void start_flags_author_the_launch_table() {
    const auto ride = bike::cli::resolve_ride(parse("bike-ride", {"--physics", "physical", "--crank-phase", "30"}));
    const auto &ride_physics = ride->setup.model.physics.value(); // NOLINT(bugprone-unchecked-optional-access) resolve_ride sets physics
    bt::require(ride_physics.rider_launch.start_pedal_angle_rad == 30. * std::numbers::pi / 180. &&
                    ride_physics.drive.crank_phase_rad == ride_physics.rider_launch.start_pedal_angle_rad,
                "--crank-phase is the table's pedal angle");
    // ... and the prepared setup records it: the setup physics names the
    // table's angle as both the authored value and the computed member.
    auto owner = bike::initialization::make_setup(ride->setup);
    const auto metadata = owner->metadata();
    const auto &physics = std::get<bike::io::Value::Object>(metadata.data);
    const auto *recorded = bike::io::find_member(physics, "physics");
    bt::require(recorded != nullptr, "setup metadata carries the physics");
    const auto &launch_doc = *bike::io::find_member(bike::io::as_object(*recorded, "physics"), "rider_launch");
    bt::require(bike::io::as_double(*bike::io::find_member(bike::io::as_object(launch_doc, "rider_launch"),
                                                           "start_pedal_angle_rad"), "angle") ==
                    ride_physics.rider_launch.start_pedal_angle_rad,
                "the recorded physics names the flag's angle");
    const auto research = bike::cli::resolve_research(parse("bike-research", {"--rear-teeth", "39"}));
    const auto &launch = research->setup.model.physics.value().rider_launch; // NOLINT(bugprone-unchecked-optional-access) resolve_research sets physics
    bt::require(launch.start_cog_teeth == std::vector<int>{39} && launch.start_cog_grade_breaks.empty() &&
                    research->setup.model.physics->drive.gearing.rear_teeth == 39,
                "--rear-teeth is a one-cog table");
}
```

    Добавить `#include <numbers>`, `#include "initialization/setup_owner.hpp"`, `#include "io/value.hpp"` и вызов в
    `main` этого файла. `make_setup` здесь без `settle`: метаданные готовы сразу после сборки.
- [ ] **Шаг 2.** `mise run test 'launch_preparation|welded_profile_config|cli_capture'` — падает.
- [ ] **Шаг 3. Код.**
  - `launch_preparation.hpp`:

```cpp
// initialization/launch_preparation.hpp — the rider's start preparation for
// one setup request (launch plan, part 2). Before the model exists the
// rider judges the grade ahead of the start point and takes the cog, pedal
// angle and lean the launch table gives for it; the cog and angle become
// the two computed drive members every later stage reads.
#pragma once

#include "../configuration/model_config.hpp"
#include "../rider/launch_config.hpp"
#include "../terrain/types.hpp"
#include "setup_owner.hpp"

namespace bike::initialization {

// The grade the rider sees before the start: the track's grade profile
// lookahead_m ahead of the start point, 0 without a grade profile.
// Obstacles are not part of it — the heightfield does not exist yet.
[[nodiscard]] double rider_start_grade(const terrain::TrackSpec &track,
                                       double start_x_m, double lookahead_m);

// Prepares the start for this physics/track/start point and writes the cog
// and pedal angle into drive.gearing.rear_teeth / drive.crank_phase_rad.
::rider::LaunchPreparation prepare_start(configuration::SimulationPhysicsConfig &physics,
                                         const terrain::TrackSpec &track, double start_x_m);

// prepare_start on the request's own physics, track and start point. Pure
// and idempotent: every path that builds a model or a contract from a
// request calls it first (make_setup, the research environment, the
// validation rig capture), and all of them get the same start.
::rider::LaunchPreparation prepare_request(SetupRequest &request);

} // namespace bike::initialization
```

  - `launch_preparation.cpp`:

```cpp
#include "launch_preparation.hpp"

#include <cmath>
#include <stdexcept>

namespace bike::initialization {

double rider_start_grade(const terrain::TrackSpec &track, double start_x_m, double lookahead_m) {
    if (!std::isfinite(start_x_m) || !std::isfinite(lookahead_m) || lookahead_m < 0.)
        throw std::invalid_argument("rider start grade needs a finite start and a nonnegative lookahead");
    return track.grade_profile ? track.grade_profile->slope(start_x_m + lookahead_m) : 0.;
}

::rider::LaunchPreparation prepare_start(configuration::SimulationPhysicsConfig &physics,
                                         const terrain::TrackSpec &track, double start_x_m) {
    const auto start = ::rider::prepare_launch(
        physics.rider_launch,
        rider_start_grade(track, start_x_m, physics.articulated.road_lookahead_m));
    physics.drive.gearing.rear_teeth = start.rear_teeth;
    physics.drive.crank_phase_rad = start.crank_phase_rad;
    return start;
}

::rider::LaunchPreparation prepare_request(SetupRequest &request) {
    if (!request.model.physics)
        throw std::invalid_argument("launch preparation requires an explicit physical configuration");
    return prepare_start(*request.model.physics, request.track, request.start_x_m);
}

} // namespace bike::initialization
```

    В `NativeStartup.cmake` в список `bike_initialization` после `initial_state.cpp` (`:92`):
    `"${native_dir}/src/initialization/launch_preparation.cpp"`.
  - `context.hpp`: `#include "../rider/launch_config.hpp"`; в `SetupContext` сразу после `start_x_m`:

```cpp
    // Launch part 2 — the start the rider prepared for this request (cog,
    // pedal angle, lean); make_setup fills it before the model is built.
    ::rider::LaunchPreparation preparation{};
```

  - `setup_owner.cpp`, `make_setup`: строку `configuration::post_init(*request.model.physics);` (`:165`) заменить на

```cpp
    // Launch part 2: the rider prepares the start before anything validates
    // or reads the computed drive members — the cog and pedal angle become
    // drive.gearing.rear_teeth / drive.crank_phase_rad for the model, the
    // writers and the recording; the lean waits in the context for the
    // settle and the intent reset.
    const auto preparation = prepare_request(request);
    configuration::post_init(*request.model.physics);
```

    после `context.start_x_m = ...` — `context.preparation = preparation;`; добавить
    `#include "launch_preparation.hpp"`.
  - `research/environment.cpp:18-19`: перед `configuration::post_init(physics);` вставить
    `(void)initialization::prepare_request(request.setup);` с комментарием «launch part 2: the research request
    itself carries the prepared start, so its recorded physics and assets name it» и
    `#include "../initialization/launch_preparation.hpp"`.
  - `validation/runtime_access.cpp:80-85`, `RuntimeRigAccess::initial_state`: перед построением `context`:

```cpp
    // Launch part 2: capture the start that was actually prepared — the
    // caller's request is the unprepared original (Simulation keeps it).
    const auto prepared=std::make_unique<initialization::SetupRequest>(request);
    const auto preparation=initialization::prepare_request(*prepared);
    // The same normalization make_setup applied to the physics it compiled
    // (drive_mode/pitch_assist resolution, cassette sort): the contract must
    // hash that physics, not the caller's unnormalized copy.
    configuration::post_init(*prepared->model.physics); // NOLINT(bugprone-unchecked-optional-access) prepare_request checked it
```

    и в инициализаторе `context`: `.physics=*prepared->model.physics,.start_x_m=prepared->start_x_m,.preparation=preparation,`
    (порядок членов — как в `SetupContext`); `#include "../initialization/launch_preparation.hpp"`.
  - `load.hpp`, публично рядом с `resolve_physics`:

```cpp
// Launch part 2 — resolves the two computed drive members of a typed
// configuration for a flat start from its own launch table, for model
// builders that bypass setup (the validation stands); setup re-resolves
// them for the real start grade.
void resolve_flat_start(SimulationPhysicsConfig& config);
```

    и в `namespace detail` рядом с `merge_values`:

```cpp
// Launch part 2 — drive.gearing.rear_teeth and drive.crank_phase_rad are
// computed from [rider_launch], never authored. Throws
// std::invalid_argument naming the table field when an authored document
// (profile file, override document) names either member.
void reject_computed_members(const bike::io::Value& authored);
// Resolves the two computed members for a flat start (grade 0) from the
// document's own [rider_launch], before construction; setup re-resolves
// them for the real start grade. A document without the table is
// returned unchanged (construction then names the missing section).
[[nodiscard]] bike::io::Value with_flat_start(bike::io::Value document);
// Removes the two computed members from a document a profile is written
// from.
void erase_computed_members(bike::io::Value& document);
```

  - `load.cpp`, в `namespace detail` после `construct_config` (ниже определений `construct_record` `:581`,
    `construct_error` `:123`, `kRiderLaunchConfigSchema`; `Value` и `Number` — `using` в `:39-40`):

```cpp
namespace {
const Value::Object* object_of(const Value& value) { return std::get_if<Value::Object>(&value.data); }
Value::Object* object_of(Value& value) { return std::get_if<Value::Object>(&value.data); }
Value& child_object(Value::Object& parent, std::string_view key) {
    for (auto& [name, member] : parent)
        if (name == key) return member;
    parent.emplace_back(std::string(key), Value{Value::Object{}});
    return parent.back().second;
}
void put(Value::Object& parent, std::string_view key, Value value) {
    for (auto& [name, member] : parent)
        if (name == key) { member = std::move(value); return; }
    parent.emplace_back(std::string(key), std::move(value));
}
} // namespace

void reject_computed_members(const Value& authored) {
    const auto* root = object_of(authored);
    const Value* drive = root ? io::find_member(*root, "drive") : nullptr;
    const auto* drive_object = drive ? object_of(*drive) : nullptr;
    if (drive_object == nullptr) return;   // shape errors stay with construction
    if (io::find_member(*drive_object, "crank_phase_rad") != nullptr)
        construct_error("drive.crank_phase_rad is computed from [rider_launch]; "
                        "author rider_launch.start_pedal_angle_rad instead");
    const Value* gearing = io::find_member(*drive_object, "gearing");
    const auto* gearing_object = gearing ? object_of(*gearing) : nullptr;
    if (gearing_object != nullptr && io::find_member(*gearing_object, "rear_teeth") != nullptr)
        construct_error("drive.gearing.rear_teeth is computed from [rider_launch]; "
                        "author rider_launch.start_cog_teeth instead");
}

Value with_flat_start(Value document) {
    auto* root = object_of(document);
    const Value* table = root ? io::find_member(*root, "rider_launch") : nullptr;
    const auto* table_object = table ? object_of(*table) : nullptr;
    if (table_object == nullptr) return document;
    RiderLaunchConfig launch{};
    construct_record(kRiderLaunchConfigSchema, *table_object, &launch, "rider_launch");
    const auto start = ::rider::prepare_launch(launch, 0.);
    auto* drive = object_of(child_object(*root, "drive"));
    if (drive == nullptr) return document;   // construction names the bad shape
    auto* gearing = object_of(child_object(*drive, "gearing"));
    if (gearing == nullptr) return document;
    put(*gearing, "rear_teeth", Value{Number{std::to_string(start.rear_teeth)}});
    put(*drive, "crank_phase_rad", Value{Number{repr_double(start.crank_phase_rad)}});
    return document;
}

void erase_computed_members(Value& document) {
    auto* root = object_of(document);
    if (root == nullptr) return;
    for (auto& [name, drive] : *root) {
        if (name != "drive") continue;
        auto* drive_object = object_of(drive);
        if (drive_object == nullptr) return;
        std::erase_if(*drive_object, [](const auto& p) { return p.first == "crank_phase_rad"; });
        for (auto& [key, gearing] : *drive_object)
            if (auto* gearing_object = key == "gearing" ? object_of(gearing) : nullptr)
                std::erase_if(*gearing_object, [](const auto& p) { return p.first == "rear_teeth"; });
    }
}
```

    и вне `detail`, рядом с `resolve_physics`:

```cpp
void resolve_flat_start(SimulationPhysicsConfig& config) {
    const auto start = ::rider::prepare_launch(config.rider_launch, 0.);
    config.drive.gearing.rear_teeth = start.rear_teeth;
    config.drive.crank_phase_rad = start.crank_phase_rad;
}
```

  - `load.cpp`, точки входа (прежние комментарии над функциями сохранить):

```cpp
SimulationPhysicsConfig
resolve_physics(const bike::io::Value& overrides,
                const SimulationPhysicsConfig& base) {
    detail::reject_computed_members(overrides);
    const Value base_doc = detail::plain_document(to_value(base));
    const Value override_doc = detail::plain_document(overrides);
    return construct_simulation(detail::with_flat_start(
        detail::merge_values(base_doc, override_doc)));
}

SimulationPhysicsConfig load_physics(const std::filesystem::path& file) {
    Value values = detail::parse_toml_file(file);
    detail::resolve_config_paths(values, file.parent_path());
    detail::reject_computed_members(values);
    return construct_simulation(detail::with_flat_start(std::move(values)));
}
```

  - `research/configuration.cpp:195-202`: после `overrides = configuration::detail::merge_values(overrides, explicit_overrides);`
    вставить `configuration::detail::reject_computed_members(overrides);`, последнюю строку заменить на
    `return configuration::detail::construct_config(configuration::detail::with_flat_start(configuration::detail::merge_values(base, overrides)));`.
  - `profile_writer.cpp`, `write_physics_toml`: `const Value document` сделать неконстантным и перед `emit_table`
    вызвать `detail::erase_computed_members(document);` с комментарием «a written profile authors only the launch
    table; the two computed drive members would be rejected on reload».
  - `cli/configuration.cpp`:
    - `ride_physics`: объявить `launch{Object{}}` рядом с `drive`; строку `:89` заменить на

```cpp
    // --crank-phase authors the launch table's pedal angle (launch part 2);
    // setup computes drive.crank_phase_rad from it.
    if (args.has("crank_phase")) set(launch, "start_pedal_angle_rad", real(args.real("crank_phase") * std::numbers::pi / 180.));
```

      и перед `auto physics = load_ride_physics(args);` добавить
      `if (!io::as_object(launch, "rider_launch").empty()) set(overrides, "rider_launch", launch);`;
    - `resolve_research`: в строке `:208` убрать `defaults->drive.gearing.rear_teeth = 51;` (таблица `[51]` стоит с
      задачи 1); строку `:221` заменить на

```cpp
    // --rear-teeth authors a one-cog launch table (launch part 2); setup
    // computes drive.gearing.rear_teeth from it at every grade.
    if (args.present("rear_teeth")) {
        set(launch, "start_cog_teeth", Value{Value::Array{integer(args.integer("rear_teeth"))}});
        set(launch, "start_cog_grade_breaks", Value{Value::Array{}});
    }
```

      объявить `launch{Object{}}` рядом с `gearing`, перед вызовом `resolve_research_physics` добавить
      `if (!io::as_object(launch, "rider_launch").empty()) set(overrides, "rider_launch", std::move(launch));`.
  - Стенды:
    - `validation/drivetrain.cpp:26` и `validation/rider.cpp:23`: `c.drive.gearing={.front_teeth=34,.rear_teeth=51};`
      → `c.drive.gearing.front_teeth=34;`;
    - в `fixture_config`, `momentum_config` и `airborne_config` перед `return c;` —
      `configuration::resolve_flat_start(c);`; в оба файла — `#include "../configuration/load.hpp"` (сейчас его там
      нет: `drivetrain.cpp:1-11`, `rider.cpp:1-5`). Стенд строит модель без подготовки
      (`validation/mechanics.hpp:21-30`), поэтому звезда 51 (или 24) берётся из таблицы здесь.
  - 16 файлов физики — убрать строку `rear_teeth = 51`:

```bash
python3 - <<'EOF'
import pathlib, subprocess
files = subprocess.run(['git', 'grep', '-l', r'^\[rider_launch\]', '--', '*.toml'],
                       capture_output=True, text=True, check=True).stdout.split()
assert len(files) == 16, files
for name in files:
    path = pathlib.Path(name)
    text = path.read_text()
    assert text.count('\nrear_teeth = 51\n') == 1, name
    path.write_text(text.replace('\nrear_teeth = 51\n', '\n', 1))
print('updated', len(files))
EOF
```

  - `test_setup.cpp:94`: `request.model.physics.value().drive.crank_phase_rad=.75;` →
    `request.model.physics.value().rider_launch.start_pedal_angle_rad=.75;` (тот же комментарий NOLINT).
- [ ] **Шаг 4. Тесты проходят.** Шаблон задачи 1. Стенды проверок (`validation`) и поездки зелёные как в базе.
- [ ] **Шаг 5. Остатки.** Обе команды печатают пустой список:

```bash
git grep -nE 'gearing\.rear_teeth *=[^=]|crank_phase_rad *=[^=]' -- native/src ':!native/src/initialization/launch_preparation.cpp' ':!native/src/configuration/load.cpp' ':!native/src/drivetrain' ':!native/src/writers' ':!native/src/runtime'
git grep -n '^rear_teeth\|^crank_phase_rad' -- '*.toml'
```

- [ ] **Шаг 6. Перезаписи** — как в задаче 1, шаг 5.
- [ ] **Шаг 7. Коммит и проверка.** `feat(setup): the start cog and pedal angle come from the launch table`, затем
  `check_task.sh <коммит задачи 1> .../verify-task2 ab`.

### Задача 3. Наклон подготовки при успокоении и в первой позе

**Файлы:**
- Изменить:
  - `native/src/rider/intent.hpp`, `native/src/rider/intent.cpp` (`seed_lean` у программы позы, политики и
    `RiderIntent`);
  - `native/src/initialization/settle_contacts.cpp:10-15`;
  - `native/src/initialization/finalize.cpp:70`;
  - `native/src/initialization/initial_state.cpp:51-84` (наклон в контракте переноса).
- Тест: `test_launch_preparation.cpp`

**Интерфейсы:**
- Использует: `SetupContext::preparation` (задача 2).
- Создаёт:

```cpp
void SeatedPostureProgram::seed_lean(double lean_rad);
void SeatedClimbPolicy::seed_lean(double lean_rad);
// Starts the posture from the prepared start lean after reset(): the
// policy, its posture program and the held intent continue from it.
void RiderIntent::seed_lean(double lean_rad);
```

- Часть 3 (план старта) опирается на то, что после сброса намерения начинают с наклона подготовки.

- [ ] **Шаг 1. Тесты (падают)** в `test_launch_preparation.cpp` (добавить `#include "rider/intent.hpp"`,
  `#include "configuration/model_config.hpp"`, `#include "initialization/state_file.hpp"`, `<algorithm>`, `<cstddef>`,
  `<optional>`, `<tuple>`, `<utility>`):

```cpp
// After reset the intent starts from the seeded lean and slews from it at
// lean_rate_rad_s: on flat ground with no front-load sample yet the first
// tick moves the lean exactly one rate step toward upright.
void test_intent_slews_from_the_seeded_lean() {
    auto config = cfg::default_physics().seated_climb;
    config.enabled = true;
    rider::RiderIntent intent(config, .00125);
    intent.seed_lean(.05);
    bt::require(intent.state().policy.lean_rad == .05 && intent.state().program.lean_rad == .05 &&
                    intent.intent().posture.torso_lean_rad == .05,
                "the seed reaches the policy, the program and the held intent");
    const auto control = intent.resolve(rider::IntentControl{}, rider::IntentSignals{}, 0, 0., 0.);
    const double step = config.lean_rate_rad_s * config.period_s;
    bt::require(control.posture.has_value() && control.posture->torso_lean_rad == .05 - step,
                "the first tick slews one rate step from the seed");
    rejects([&] { intent.seed_lean(config.max_forward_lean_rad + 1e-3); }, "lean", "a seed beyond the forward bound");
    intent.reset();
    bt::require(intent.state().program.lean_rad == 0., "reset still starts upright before a seed");
}

// Bootstrap rider-intent lean, from the sealed state document.
double intent_lean(const runtime::RuntimeBootstrap &bootstrap, std::string_view part) {
    return bike::io::as_double(member(member(bootstrap.state.rider_intent, part), "lean_rad"), "lean_rad");
}

std::vector<std::filesystem::path> cache_entries(const std::filesystem::path &directory) {
    std::vector<std::filesystem::path> out;
    for (const auto &entry : std::filesystem::directory_iterator(directory))
        if (entry.path().extension() == ".npz") out.push_back(entry.path());
    return out;
}

// The complete settled state of a completed setup, as the state file
// stores it — coordinates, integration state, drive, rider, tire and model
// arrays — by array name, each with its dtype, shape and bytes.
using StateArrays = std::vector<std::pair<std::string, std::tuple<std::string, std::vector<std::size_t>, std::vector<std::byte>>>>;
StateArrays settled_state(const init::SetupOwner &owner) {
    StateArrays out;
    for (const auto &[name, array] : owner.capture_state_file().arrays)
        out.emplace_back(name, std::tuple{array.dtype, array.shape, array.bytes});
    return out;
}
std::vector<std::byte> qpos_of(const StateArrays &state) {
    for (const auto &[name, array] : state)
        if (name == "qpos") return std::get<2>(array);
    bt::require(false, "the state file stores qpos");
    return {};
}

// One completed setup of `request`: its result, settled state and sealed lean.
struct Settled {
    init::EquilibriumResult result;
    StateArrays state;
    std::vector<std::byte> qpos;
    double policy_lean = 0., program_lean = 0.;
};
[[gnu::noinline]] Settled settle_once(const init::SetupRequest &request) {
    auto owner = init::make_setup(request);
    Settled out{.result = init::settle(*owner), .state = {}, .qpos = {}};
    out.state = settled_state(*owner);
    out.qpos = qpos_of(out.state);
    const auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(owner->seal_bootstrap());
    out.policy_lean = intent_lean(*bootstrap, "policy");
    out.program_lean = intent_lean(*bootstrap, "program");
    return out;
}

// The prepared lean is part of every way a settled start is reused. A
// cache miss settles holding it; a cache hit and an explicit state file
// restore the very same coordinates and lean; an explicit file settled
// with another lean is refused; a different lean is a different cache
// entry and a different rest pose.
[[gnu::noinline]] void test_prepared_lean_survives_cache_and_state_file() {
    const bt::TemporaryDirectory cache("launch-lean-cache");
    auto request = settle_request(0.);
    request->model.physics->equilibrium_cache_enabled = true; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    request->model.physics->rider_launch.start_lean_rad = {.05}; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    request->cache_directory = cache.path();
    const auto miss = settle_once(*request);
    bt::require(!miss.result.cache_hit && miss.policy_lean == .05 && miss.program_lean == .05,
                "a cache miss settles and seals the prepared lean");
    const auto hit = settle_once(*request);
    bt::require(hit.result.cache_hit && hit.state == miss.state && hit.policy_lean == .05,
                "a cache hit restores the same complete state and lean");
    const auto entries = cache_entries(cache.path());
    bt::require(entries.size() == 1, "one lean is one cache entry");
    auto pinned = std::make_unique<init::SetupRequest>(*request);
    pinned->model.physics->equilibrium_cache_enabled = false; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    pinned->model.physics->equilibrium_state_file = entries.front().string(); // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    const auto from_file = settle_once(*pinned);
    bt::require(from_file.result.cache_hit && from_file.state == miss.state && from_file.policy_lean == .05,
                "an explicit state file restores the same complete state and lean");
    pinned->model.physics->rider_launch.start_lean_rad = {0.}; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    rejects([&] { (void)settle_once(*pinned); }, "fingerprint", "a state file settled with another lean");
    pinned->model.physics->rider_launch.start_lean_rad = {.05}; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    pinned->model.physics->rider_launch.start_pedal_angle_rad = .3; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    rejects([&] { (void)settle_once(*pinned); }, "fingerprint", "a state file settled with another pedal angle");
    pinned->model.physics->rider_launch.start_pedal_angle_rad = 0.; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    pinned->model.physics->rider_launch.start_cog_teeth = {45}; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    rejects([&] { (void)settle_once(*pinned); }, "fingerprint", "a state file settled on another cog");
    auto upright = std::make_unique<init::SetupRequest>(*request);
    upright->model.physics->rider_launch.start_lean_rad = {0.}; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    const auto settled_upright = settle_once(*upright);
    bt::require(!settled_upright.result.cache_hit && cache_entries(cache.path()).size() == 2,
                "another lean is another cache entry");
    bt::require(settled_upright.qpos != miss.qpos, "the settle held the prepared lean: the rest pose differs");
}

// The transfer contract binds the prepared lean, not the table: a transfer
// validates for the same prepared start (also from a different table that
// prepares the same lean), and is refused by a start with another lean.
[[gnu::noinline]] void test_transfer_contract_binds_the_prepared_lean() {
    auto request = settle_request(0.);
    request->model.physics->rider_launch.start_lean_rad = {.05}; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    auto source = init::make_setup(*request);
    (void)init::settle(*source);
    const auto captured = source->capture_initial_state();
    const auto source_qpos = qpos_of(settled_state(*source));
    auto same = std::make_unique<init::SetupRequest>(*request);
    same->initial_state = captured;
    auto reused = init::make_setup(*same);
    bt::require(init::settle(*reused).initial_state_sha256 == std::optional<std::string>(captured.sha256()) &&
                    qpos_of(settled_state(*reused)) == source_qpos,
                "the same prepared start reuses the transfer and lands on its coordinates");
    same->model.physics->rider_launch.start_lean_grades = {-.5, .5}; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    same->model.physics->rider_launch.start_lean_rad = {.05, .05}; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    auto retabled = init::make_setup(*same);
    bt::require(init::settle(*retabled).initial_state_sha256 == std::optional<std::string>(captured.sha256()) &&
                    qpos_of(settled_state(*retabled)) == source_qpos,
                "a different table with the same prepared lean reuses it too");
    auto upright = std::make_unique<init::SetupRequest>(*request);
    upright->initial_state = captured;
    upright->model.physics->rider_launch.start_lean_rad = {0.}; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    auto refused = init::make_setup(*upright);
    rejects([&] { (void)init::settle(*refused); }, "contract differs", "a leaning transfer into an upright start");
}

// The validation rig captures the prepared lean too: a capture through
// validation::Simulation of a leaning start is refused by an upright one.
[[gnu::noinline]] void test_validation_capture_binds_the_lean() {
    auto request = settle_request(0.);
    request->model.physics->rider_launch.start_lean_rad = {.05}; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    auto simulation = std::make_unique<bike::validation::Simulation>(*request);
    auto upright = std::make_unique<init::SetupRequest>(*request);
    upright->initial_state = simulation->initial_state();
    upright->model.physics->rider_launch.start_lean_rad = {0.}; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    auto refused = init::make_setup(*upright);
    rejects([&] { (void)init::settle(*refused); }, "contract differs", "a rig capture with another lean");
}

// A lumped rider has no torso: a nonzero table lean leaves its settle
// command empty (the step rejects any posture for it) and setup completes.
[[gnu::noinline]] void test_lumped_rider_ignores_the_prepared_lean() {
    const std::vector<std::string_view> args{"--physics", "physical", "--rider", "lumped", "--track", "flat"};
    auto ride = bike::cli::resolve_ride(bike::cli::parse_arguments("bike-ride", args));
    ride->setup.model.physics->rider_launch.start_lean_rad = {.05}; // NOLINT(bugprone-unchecked-optional-access) resolve_ride sets physics
    auto owner = init::make_setup(ride->setup);
    bt::require(std::isfinite(init::settle(*owner).pitch_rad), "a lumped rider settles with a table lean");
}
```

  Вызовы — в `main`. Если сообщение отказа явного файла с другим наклоном не содержит `fingerprint`, взять
  подстроку из `native/src/initialization/equilibrium_cache.cpp:235-238` и записать её в отчёт.
- [ ] **Шаг 2.** `mise run test 'launch_preparation'` — сборка падает: нет `seed_lean`.
- [ ] **Шаг 3. Код.**
  - `intent.hpp`: объявления `seed_lean` из блока «Интерфейсы» — в публичной части каждого класса, рядом с
    `reset()`, с комментарием над `RiderIntent::seed_lean`.
  - `intent.cpp`:

```cpp
void SeatedPostureProgram::seed_lean(double lean_rad) {
    if (!std::isfinite(lean_rad) || lean_rad > config_.max_forward_lean_rad ||
        lean_rad < -config_.max_backward_lean_rad)
        throw std::invalid_argument("seeded lean must be finite and inside the seated posture bounds");
    lean_rad_ = lean_rad;
}

void SeatedClimbPolicy::seed_lean(double lean_rad) {
    program_.seed_lean(lean_rad);
    lean_rad_ = lean_rad;
}

void RiderIntent::seed_lean(double lean_rad) {
    policy_.seed_lean(lean_rad);
    intent_.posture.torso_lean_rad = lean_rad;
}
```

  - `settle_contacts.cpp`, `apply_initial_forces`:

```cpp
void apply_initial_forces(SetupContext& context,bool advance) {
    require_initializing(context);
    // Launch part 2: an articulated rider holds the prepared start lean
    // through the settle. A zero lean keeps the empty command the settle
    // always sent, and a lumped or absent rider has no torso to lean — the
    // step rejects any posture for it (control.cpp:40-44).
    runtime::RideControl control{};
    if(context.preparation.lean_rad!=0.&&context.physical.rider_present())
        control.posture=::rider::RiderPosture{.torso_lean_rad=context.preparation.lean_rad};
    (void)context.physical.apply_forces(false,advance,
        context.physics.initial_front_brake,context.physics.initial_rear_brake,
        std::nullopt,control);
}
```

  - `finalize.cpp:70`: после `physical.rider_intent_.reset();` —
    `physical.rider_intent_.seed_lean(context.preparation.lean_rad);` с комментарием «launch part 2: the first
    running posture continues from the prepared lean».
  - `initial_state.cpp`, функция `contract`: после строки, которая убирает `research`, `monitors` и `rider_launch`:

```cpp
    // The prepared start lean shapes the rest pose (launch part 2). It is
    // bound only when nonzero: an absent member is the upright start every
    // transfer before the launch table was captured with, so those stay
    // valid while two different leans never share a contract.
    if(context.preparation.lean_rad!=0.)put(config,"launch_lean_rad",real(context.preparation.lean_rad));
```

    (`put` и `real` — помощники этого файла, они уже используются в той же функции.)
  - `equilibrium_cache.cpp`, историческая ветка ключа (строка с `equilibrium_relaxation`/`rider_launch` из
    задачи 1): сразу после неё добавить

```cpp
    // Historical payloads were all settled upright: a nonzero prepared lean
    // is bound so it can never reuse one, while the upright key stays the
    // historical bytes.
    if(legacy&&context.preparation.lean_rad!=0.)fields.emplace_back("launch_lean_rad",number(context.preparation.lean_rad));
```

- [ ] **Шаг 4. Тесты проходят.** Шаблон задачи 1. Успокоенная поза, модель и начальное состояние бит в бит
  прежние (`ab`): наклон в профилях 0, и команда успокоения осталась пустой.
- [ ] **Шаг 5. Коммит и проверка.** `feat(setup): the settle and the first posture hold the prepared lean`, затем
  `check_task.sh <коммит задачи 2> .../verify-task3 ab`.

### Задача 4. Итоговая проверка части 2 (оркестратор)

По образцу задачи 7 части 1, своими руками, без исполнителя. Каталог —
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/final`; лог и код выхода каждой стадии сохраняются.

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
F=/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/final; T=/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2; mkdir -p $F
R=/Volumes/SSD_1TB_M2/work/tmp/2026-11-10-delays-t5b-check/reds.txt; S=/Volumes/SSD_1TB_M2/work/tmp/2026-11-09-reds-final/slow_reds.txt
t() { perl -e "alarm $1; exec @ARGV" -- "${@:2}"; }
t 3600 cmake --build build/native-release -j 12 > $F/build.log 2>&1; echo "build $?" | tee $F/stages.txt
t 3600 bash tools/run_native_checks.sh build/native-release > $F/checks.log 2>&1; echo "checks $?" | tee -a $F/stages.txt
t 1800 ctest --test-dir build/native-release -LE 'slow|package' -j 12 --output-on-failure > $F/ordinary.log 2>&1; echo "ordinary $?" | tee -a $F/stages.txt
t 2400 ctest --test-dir build/native-release -L slow --no-tests=error -j 8 --output-on-failure > $F/slow.log 2>&1; echo "slow $?" | tee -a $F/stages.txt
t 1200 ctest --test-dir build/native-release -L package --no-tests=error > $F/package.log 2>&1; echo "package $?" | tee -a $F/stages.txt
cat $F/ordinary.log $F/slow.log > $F/reds_output.log
python3 $T/causes.py /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch1/task7/reds_output.log $F/reds_output.log $R $S > $F/cause_compare.txt; echo "causes $?" | tee -a $F/stages.txt
bash $T/ab_settle.sh /Volumes/SSD_1TB_M2/work/wt/lane-flip "$PWD" $F/ab > $F/ab.txt 2>&1; echo "ab $?" | tee -a $F/stages.txt
```

  Ожидание: `build 0`, `checks 0` (у каждого вида `tool health: ok` и `findings: none`), `ordinary 8` и `slow 8`
  (тесты упали — это ровно известные красные, что подтверждает `causes 0`), `package 0`, `ab 0`.
- [ ] Отчёт называет итог честно: проверка относительно базы с известными красными, а не зелёный `full`.
- [ ] Строка состояния в `docs/superpowers/plans/2026-11-11-rider-launch-plan.md` и запись в журнал
  `.scratch/devin/2026-11-05-physics-queue/ledger.md`. Коммит `docs(plan): launch part 2 is in the code`.

## Ответы на ревью

**Ревью всей работы (`2026-11-11-plan-review`), доля части 2:**

| Пункт | Где в плане |
|---|---|
| I6 — авторская и вычисленная формы | Решения 2–4, 7, 8; задача 2: отказ на трёх авторских входах, ровный старт при загрузке и у стендов, писатель профиля без вычисленных полей, флаги пишут таблицу, записанный документ сохраняет подготовленные значения |
| I7 — перевод всех ключей и значения «как сегодня» | Решение 12 и опись с таблицей звёзд; проверка числа `[rider_launch]` = `[monitors]`; `ab_settle.sh` сравнивает позу и модель с базой |
| I9 — путь подготовки | Решение 5: `prepare_request` в `make_setup`, в среде исследований и при снятии состояния стендом; задача 3: успокоение, `seed_lean`, кэш, явный файл, перенос, тесты с ненулевым наклоном и нестандартной звездой |
| I13 — проверки таблиц | Задача 1: форма, конечность, строгий рост, число переломов, звезда ≥ 3, звезда на кассете, наклон в пределах позы, у каждой правильный контроль. Полей времени в части 2 нет; новых восстанавливаемых состояний тоже (`seed_lean` пишет существующие поля) |
| I17 — заглушки | Код и тела тестов полные; имена помощников взяты из кода |

**Ревью этого плана (`2026-11-11-plan2-review`):**

| Пункт | Ответ |
|---|---|
| C1 — поза при успокоении у нешарнирного райдера | Решение 9; поза только при `rider_present()` и наклоне ≠ 0; тест упрощённого райдера с наклоном |
| I1 — стенды теряли 51 | Стенды вызывают `resolve_flat_start(c)`; звезда из таблицы `[51]` |
| I2 — проверка устаревшей звезды до подготовки | Решение 5: подготовка до `post_init` в `make_setup` и в среде исследований; тест с кассетой без 24 |
| I3 — снятие состояния стендом без подготовки | `RuntimeRigAccess::initial_state` готовит копию запроса; тесты через `validation::Simulation` (звезда — задача 2, наклон — задача 3) |
| I4 — файл исследования без таблицы | Решение 8: файл исследования — поправка поверх умолчаний, как для всех разделов; запрет вычисленных полей действует |
| I5 — старые записи при воспроизведении | Решение 11: отклоняются, читателя старых форм нет; тест записанного документа с подготовленной звездой и отказ документа без таблицы; `replay_recording_v3` — долг |
| I6 — лишняя скобка, кадр | Документы-значения в тестах собраны по частям; запросы и загрузка в куче; файл теста снимает предупреждение о кадре по образцу `test_setup.cpp` |
| I7 — слабые тесты повторного использования | Побайтное сравнение `qpos` для промаха, попадания и явного файла; отказ явного файла с другим наклоном; перенос при той же подготовке и при другой таблице с тем же результатом; координата шатуна; таблица в записи исследования |
| I8 — хэши и красные тесты | Опись хэшей в «Общих ограничениях»: все в красных тестах, остаются долгом; новый красный из-за хэша — стоп |
| I9 — честность итоговой проверки | `check_task.sh` останавливается на новых или починившихся красных, грязных анализаторах, расхождении причин всех 32 красных (строгое сравнение `causes.py`) и позы; в задаче 4 у медленных и пакетных `--no-tests=error`, логи и коды сохраняются |
| M1 — указатели | Исправлены: `model_config.hpp:292-295`, `initial_state.cpp:77`, `validate.cpp:1292`, `NativeStartup.cmake:92`; гадание об имени `posture` убрано; опись со звёздами по местам |
| M2 — смысл «туда-обратно» для писателя | Тест: таблица сохраняется, вычисленные поля возвращаются как ровный старт, проверено на ступенчатой таблице |
| M3 — политика ключей | Решение 10: кэш осторожен (любая правка таблицы — промах), перенос привязан к результату подготовки; исторический ключ NPZ без таблицы |

**Повторное ревью (`review-2.md`):**

| Пункт | Ответ |
|---|---|
| R1 — нет заголовка у стендов | `#include "../configuration/load.hpp"` в оба файла стендов |
| R2 — ссылка на временный документ | Метаданные сохраняются в переменную до `member()` |
| R3 — исторический ключ терял наклон | Ненулевой наклон добавляется в исторический ключ (задача 3); ключ прямого старта прежний |
| R4 — анализатор с ошибкой проходил | `check_task.sh` требует код 0 и `tool health: ok` у каждого прохода и строку итога CTest у `check_change` |
| R5 — сравнение причин обрезалось и зависело от порядка | `causes.py`: блок теста закрывается на следующем тесте или итоге CTest, строки не обрезаются, обязательный список 32 тестов, код перепрогона должен быть 8 |
| R6 — старые позы при сбое | `ab_settle.sh` удаляет каталог случая перед запуском, сбой или пустой файл — `STOP`, проверяется число массивов |
| R7 — заявления шире тестов | Сравнивается всё состояние файла (все массивы с типом и формой); отказ явного файла при другом угле и звезде; перенос садится на те же координаты; флаг попадает в физику подготовки. Заявление о поездках сужено: бит в бит — поза, модель, начальное состояние |
| R8 — тип и команды | Тип `initial_state_sha256` назван; команды задачи 4 с ограничением времени, логами и кодами |
| I3 (остаток) — нормализация копии | `RuntimeRigAccess::initial_state` вызывает `post_init` для копии после подготовки, как `make_setup` |

## Что остаётся частям 3–4

Без изменений по `docs/superpowers/plans/2026-11-11-launch-1-channels.md`, раздел «Что остаётся частям 2–4». Часть 3
добавляет в `rider::LaunchConfig` поля удержания, отпускания тормоза, предела нажима, переключения и конца старта, а
также раздел реакции; числа калибрует она же. Встроенные таблицы трёх типов райдера ставит часть 4.
