# План старта, часть 3a: ощущения в шаге и реакция на подъём колеса — план реализации

> **Для исполнителей-агентов:** обязательный навык — `superpowers:subagent-driven-development` (рекомендуется) или
> `superpowers:executing-plans`. Шаги отмечаются флажками (`- [ ]`). Исполнитель — Devin (`devin-orchestration`);
> финальное ревью ветки делает оркестратор.

Редакция 2 — после ревью codex `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan3a-review/review.md` (ответы в конце).

**Цель.** Подготовить всё, на что опирается план старта (часть 3b), кроме самого плана. Поведение поездок не
меняется:
- райдер получает свои ощущения с задержкой реакции прямо в шаге: адаптер `rider::Perception` из части 1 живёт в
  `RiderIntent`;
- реакция на подъём переднего колеса — отдельный узел на всю поездку («Решено — не сделано», пункт 2). В
  профилях оба её действия выключены, а признак и фазы пишутся в запись каждого интервала;
- на педалях появляется порядок «предел райдера → срез переключения» для случая, когда нажим ограничивает сам
  райдер. Обычный путь не меняется.

**Архитектура.**
- `RiderIntent::resolve_full` получает ощущения шага (`RiderSenses`) и возвращает выход вместе с признаком
  `effort_limited`. Пробный шаг берёт признак из той же копии намерений, что и выход.
- На тике намерений сначала проверяется новая выборка. Затем `Perception` выдаёт задержанное восприятие, по нему
  `FrontLiftReaction` решает, как действовать, и только после этого обновляется обычная политика.
- «Наклониться» входит в цель программы позы добавкой (`extra_target_rad`). Поэтому общий наклон подчиняется
  прежнему ограничению скорости и пределам позы, а нулевая добавка не меняет арифметику.
- «Ослабить» умножает выход «предел нажима»; множитель 1 не применяется.
- Состояния восприятия и реакции входят в `state.rider_intent`. Восстановление намерений сначала собирает
  кандидата, потом подменяет состояние целиком.

**Стек.** C++23, MuJoCo, CTest (`bike_add_reference_test`), таблицы полей `FieldDecl`, строгие кодеки, каталог
записи.

**Спецификация.** `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md`, разделы «Ощущения райдера»,
«Реакция на подъём переднего колеса», «Таблица опыта» (строка реакции), «Кто чем управляет». Ревью всей работы —
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan-review/review.md`: в этой части пункты I4, I12, I13 и доля I14 для
реакции. C2, C3, I10, I11 и события плана — в части 3b. Части 1 и 2 — `2026-11-11-launch-1-channels.md`,
`2026-11-11-launch-2-preparation.md`.

## Решения плана

1. **Решение пользователя 2026-11-11.** Мягкий старт уходит в части 4, в той же задаче, где включается план
   старта. Части 3a и 3b строят узлы за выключателями, которые в профилях выключены.
2. **Реакция — раздел профиля `[front_lift_reaction]`** (`rider::FrontLiftReactionConfig`, 13 обязательных полей,
   значений по умолчанию в коде нет):
   - `hand_unload_n`, `unload_dwell_s` — порог и выдержка разгрузки рук;
   - `pitch_rate_up_rad_s`, `pitch_rad` — пороги скорости подъёма и накопленного наклона;
   - `decision_s` — время решимости;
   - `ease_enabled`, `lean_enabled`, `rear_brake_tap_enabled` — три выключателя; последний обязан быть `false`
     (решение пользователя 4);
   - `ease_fraction`, `ease_rate_per_s` — на какую долю и с какой скоростью ослабить предел нажима;
   - `lean_add_rad`, `lean_rate_rad_s` — на сколько и с какой скоростью сдвинуть цель наклона;
   - `clear_s` — сколько признак должен отсутствовать, чтобы реакция закончилась.
3. **Времена — в тиках намерений.** `unload_dwell_s`, `decision_s` и `clear_s` — целые кратные
   `seated_climb.period_s`, не больше 2^31 тиков. «Дольше выдержки» значит больше `unload_dwell_s / period_s`
   выборок подряд. Проверки сетки и предела наклона (`lean_add_rad ≤ seated_climb.max_forward_lean_rad`) делают оба
   входа:
   - корневой `post_init` профиля;
   - конструктор `RiderIntent`, который строится из документа запуска и `post_init` не проходит
     (`runtime.cpp:350-354`).
4. **Признак и фазы на задержанном восприятии.**
   - Признак: руки разгружены (`hand_press_n < hand_unload_n`) дольше выдержки, и при этом скорость подъёма
     `> pitch_rate_up_rad_s` или накопленный наклон `> pitch_rad`.
   - Фазы:
     - `idle` → `deciding` в тик появления признака;
     - `deciding` → `acting`, когда признак держится `decision_s`; если он пропал раньше — назад в `idle`;
     - `acting` → `releasing`, когда признака нет дольше `clear_s`;
     - `releasing` → `acting`, если признак вернулся;
     - `releasing` → `idle`, когда множитель вернулся к 1, а добавка к 0.
   - Пока восприятия нет (первые `reaction_delay_s`), реакция стоит в `idle`.
   - **Время выборки.** Шаг кладёт в восприятие ощущения начала интервала, который только что закончился
     (`step.cpp:1854-1860`). Поэтому к задержке реакции добавляется один физический шаг (1,25 мс в рабочем
     профиле). Это записывается, а не исправляется: вторая задержка не вводится.
5. **Действия.**
   - «Ослабить»: выход «предел нажима» умножается на множитель реакции, если предел не задал вызывающий. Множитель
     1 не применяется.
   - «Наклониться»: добавка реакции входит в цель программы позы (`SeatedPostureProgram::update`, параметр
     `extra_target_rad`) вместе с импульсами корпуса (`intent.cpp:175-180`). Дальше её ограничивают прежние
     скорость наклона, передний предел и предел по сгибу локтя. Нулевая добавка не прибавляется.
   - При передаче управления в части 3b добавка остаётся в цели программы, поэтому реакция идёт через передачу
     без перезапуска.
6. **Признак `effort_limited`.** Он истинен, когда предел нажима выдают намерения, а множитель реакции < 1. В части
   3b к нему добавится план. Признак возвращается из `resolve_full` вместе с выходом той же (пробной или
   настоящей) копии, и шаг передаёт его на педали.
7. **Порядок на педалях (I4).** При `cut_after_ceiling` цель строится так: `min(цель с прибавкой mash, предел) ×
   срез`, затем прежний темп. Иначе порядок прежний: `(цель × срез)`, затем `min(предел)` (`pedaling.cpp:68-74`).
8. **Запись.** Канал `rider_reaction` (`phase`, `predicate`, `effort_factor`, `lean_add_rad`) кладётся в каналы
   каждого интервала там же, где `rider_senses` (`step.cpp:2031-2049`), а не в подробные каналы. Поэтому он есть и
   на прореженных интервалах. Если намерения выключены, значение — `null`. Моменты появления признака и начала
   действия видны как смена `predicate` и `phase` между интервалами.
9. **Ключи повторного использования.** Реакция не меняет успокоенную позу. Поэтому `front_lift_reaction`
   убирается из контракта переноса (`initial_state.cpp:80`) и из ключа кэша для обеих ветвей — одной строкой
   рядом с общими исключениями (`equilibrium_cache.cpp:168`).
10. **Документы.**
    - В документ запуска добавляется ключ верхнего уровня `rider_reaction`, только в список ключей конфигурации
      (`bootstrap.cpp:249-255`). Его значение копируется в `RuntimeConfig` рядом с `rider_intent`
      (`bootstrap.cpp:300-302`).
    - Состояние меняется только внутри `rider_intent`: ключи `perception` и `reaction`. Список ключей верхнего
      уровня состояния (`bootstrap.cpp:358-363`) не трогается.

## Общие ограничения

Как в части 2 (`docs/superpowers/plans/2026-11-11-launch-2-preparation.md`, раздел «Общие ограничения»), с этими
уточнениями:
- **Доказательство неизменности — по частям, и отчёт называет его так же:**
  - успокоенная поза и модель — `ab_settle.sh`;
  - выход намерений при выключенной реакции — тест-оракул задачи 3: побайтно против голой обычной политики;
  - порядок педалей при `cut_after_ceiling = false` — тест задачи 1;
  - поездки — неизменный код езды и прежние тесты поездок, без побайтного сравнения траекторий.
- **Перезапись эталонов** — правило, подтверждённое пользователем для части 2:
  - только выходы зелёных тестов, только новые ключи;
  - проверка `check_rerecord.py --ref <коммит до задачи>`;
  - изменение прежнего значения (хэши включительно) — стоп;
  - выходы красных тестов не трогаются, входные документы переводятся на новую форму везде.
- **Красные тесты.** Известные красные — 22 обычных и 10 медленных. Нельзя переставлять или ослаблять проверки в
  красном тесте ради прежней причины падения. Новая причина допустима, только если её вызвала задача; она
  приводится и объясняется.
  - `reference.native_rider_intent` уже падает при построении адаптера (`.../2026-11-11-launch2/final/reds_output.log:38-39`).
    Его двоичные входы (`input.restore.*` с документами состояния намерений) не переводятся: тест красный, а
    проверка `check_rerecord.py` двоичные файлы не читает. Это долг; он записывается в журнал.
- Без отладчиков, с ограничением времени у длинных команд, без `git stash`, без push, `.claude/` и `.agents/` не
  трогать. Последняя строка коммита ровно `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Проверка задачи

После коммита задачи:

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
BASELOG=/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/baseline-before-task<N>.log \
  bash /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/check_task.sh <база задачи> \
  /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/verify-task<N> ab
```

Оркестратор кладёт `baseline-before-task<N>.log` перед запуском: это `reds_output.log` проверки предыдущей задачи;
для задачи 1 — `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/final/reds_output.log`. Конец вывода —
`check_task exit 0`. Иначе каждое `STOP` приводится и объясняется. Исправления — отдельным коммитом, после чего
проверка повторяется на итоговой вершине.

## Бюджет тестов

| Что | Время |
|---|---|
| целевые тесты задачи | 3–8 мин |
| `check_task.sh` с `ab` | около 32 мин |
| задача 4: итоговая проверка | около 25 мин |

Итого около 2 ч 20 мин машинного времени без повторов.

## Опись

**Полные документы физики** получают `[front_lift_reaction]` — те же места, что получили `[rider_launch]` в части 2:
- 16 файлов `*.toml` (`git grep -l '^\[monitors\]' -- '*.toml'`);
- `default_physics()` (`validate.cpp`), умолчания `bike-research` (`cli/configuration.cpp`, рядом с
  `defaults->rider_launch`);
- `fixture_config`, `momentum_config`, `airborne_config` (`validation/drivetrain.cpp`, `validation/rider.cpp`), две
  конфигурации `validation/scenarios.cpp`;
- тестовые структуры `replay_reference_support.hpp`, `research_reference_support.hpp`, `test_control_clock.cpp`,
  `replay/test_session.cpp`, `test_resolution.cpp` (`unresolved_base`), `test_schema.cpp` (`welded_profile()`);
- подстановка недостающих разделов в `test_pneumatic_tire_config.cpp` (`resolve()`);
- встроенные TOML-строки: проверка «`[front_lift_reaction]` столько же, сколько `[monitors]`» по образцу шага 4
  задачи 1 части 2.

Найти все программные места: `git grep -n 'rider_launch *=' -- native` — у каждого такого места появляется
`front_lift_reaction`.

**Значения в профилях** — предварительные, реакция выключена:
- `ease_enabled = false`, `lean_enabled = false`, `rear_brake_tap_enabled = false`;
- `hand_unload_n = 20.0`, `unload_dwell_s = 0.05`, `pitch_rate_up_rad_s = 0.5`, `pitch_rad = 0.17`;
- `decision_s = 0.1`, `ease_fraction = 0.7`, `ease_rate_per_s = 5.0`;
- `lean_add_rad = 0.1`, `lean_rate_rad_s = 0.5`, `clear_s = 0.3`.

**Документы, которые получают новые ключи** (перезапись по правилу, проверка `check_rerecord.py`):

| Где | Что добавляется | Кто читает |
|---|---|---|
| `tests/fixtures/bootstrap/**/config.json` (включая `faults/*`, `welded/reset/*`) | `rider_reaction` | `runtime_bootstrap` (красный), тесты захвата и загрузки |
| `tests/fixtures/bootstrap/**/state.json` | `rider_intent.perception`, `rider_intent.reaction` | то же |
| `tests/fixtures/research/delayed_bootstrap/{config,state}.json` | то же | исследования |
| `tests/fixtures/ft2/{darwin-arm64,linux-aarch64}/{ride,research_flat,research_savage}/{config,state}.json` | то же | `native_runtime_*` (красные) — их выходы не трогаются |
| `tests/fixtures/state/rider_intent.json`, `rider_intent.canonical.json` | `perception`, `reaction` | `io_state_codecs` (`test_state_codecs.cpp:361-368`) |
| `tests/fixtures/recording/catalog_welded.json` | 5 описателей канала `rider_reaction` | `recording_catalog` (`test_catalog.cpp:72`: 506 → 511) |

Перед перезаписью каждый файл относится к одному из двух видов: вход теста или выход теста. Негативные документы
(`faults/*`) сохраняют свою намеренную ошибку: новый ключ им добавляется, а испорченное поле остаётся испорченным.

**Вызовы `resolve` и конструктора `RiderIntent`:**
- `native/src/rider/intent.cpp:466` (пробная рекурсия);
- `native/src/runtime/step.cpp:1024` (конструктор), `:1264-1269` (вызов);
- `native/tests/reference/test_launch_preparation.cpp:289,294`;
- `native/tests/reference/test_seated_posture_program.cpp:87-89,94-95,136,145-147,166`;
- `native/tests/reference/test_native_rider_intent.cpp:170-173` (`plant.intent.emplace(...)`), `:204-205`.

Поиск, чтобы не пропустить: `git grep -nE 'RiderIntent\(|intent\.emplace\(|\.resolve\(' -- native`.

---

### Задача 1. Порядок «предел райдера → срез» на педалях

**Файлы:**
- Изменить: `native/src/drivetrain/pedaling.hpp:82-86`, `native/src/drivetrain/pedaling.cpp:68-74`,
  `native/src/writers/drivetrain.hpp:119-134,301-306`, `native/src/writers/drivetrain.cpp:476-488,751-755`,
  `native/src/runtime/step.cpp:1328-1337`.
- Создать: `native/tests/reference/test_pedaling_cut_order.cpp`; регистрация в `native/tests/reference/Phase3Ports.cmake`
  после `launch_preparation`:

```cmake
# Launch part 3a, task 1 — the shift cut after a rider-limited effort
# ceiling (review I4): ordinary and rider-limited target construction.
bike_add_reference_test(pedaling_cut_order ENGINE
  SOURCES test_pedaling_cut_order.cpp LIBRARIES bike_app
  LABELS launch drive)
```

**Интерфейсы:**
- Создаёт: последний параметр `bool cut_after_ceiling = false` у `PedalingPolicy::update` (`pedaling.hpp:83-86`);
  поле `bool cut_after_ceiling = false;` в `DrivetrainWriter::PrepareInputs`; последний параметр
  `bool cut_after_ceiling = false` у `DrivetrainWriter::prepare`. Значения по умолчанию сохраняют всех нынешних
  вызывающих; это параметры кода, а не профиля.
- Задача 3 передаёт признак `effort_limited` из `resolve_full`.

- [ ] **Шаг 1. Тест (падает):**

```cpp
// Launch part 3a — the shift cut after a rider-limited effort ceiling
// (review I4): the ordinary path keeps (target x cut) then min(ceiling);
// a rider-limited effort takes min(target, ceiling) first and cuts after,
// so a small ceiling cannot erase the cut.
#include "support/fixture_test.hpp"
#include "drive_reference_support.hpp"
#include "drivetrain/pedaling.hpp"

#include <cmath>
#include <optional>

namespace {

// The shared reference pedaling config (mash 60 N·m below 45 rpm, no
// slew): a crank at rest raises a 10 N·m effort toward 60 before any cap.
drivetrain::PedalingPolicy mash_policy() {
    return drivetrain::PedalingPolicy(drive_reference::pedaling_config());
}

// Ordinary order: (60 x .3 = 18) then min(10) = 10 — the cut is erased.
void test_ordinary_order_is_unchanged() {
    auto policy = mash_policy();
    const auto out = policy.update(0., 0., 0., 10., .00125, true, false, std::nullopt, 10., .3);
    bt::require(out.effort_nm == 10., "ordinary order keeps min(target x cut, ceiling)");
}

// Rider-limited order: min(60, 10) = 10, then x .3 = 3.
void test_rider_limited_effort_cuts_after_the_ceiling() {
    auto policy = mash_policy();
    const auto out = policy.update(0., 0., 0., 10., .00125, true, false, std::nullopt, 10., .3, true);
    bt::require(std::abs(out.effort_nm - 3.) < 1e-12, "the cut applies after the rider's ceiling");
}

// Without a cut the flag changes nothing, bit for bit.
void test_orders_agree_without_a_cut() {
    auto a = mash_policy();
    auto b = mash_policy();
    const auto ordinary = a.update(0., 0., 0., 10., .00125, true, false, std::nullopt, 10., 1.);
    const auto limited = b.update(0., 0., 0., 10., .00125, true, false, std::nullopt, 10., 1., true);
    bt::require(ordinary.effort_nm == limited.effort_nm, "factor 1 gives the same effort in both orders");
}

} // namespace

int main() {
    return bt::run([] {
        test_ordinary_order_is_unchanged();
        test_rider_limited_effort_cuts_after_the_ceiling();
        test_orders_agree_without_a_cut();
    });
}
```

- [ ] **Шаг 2.** `mise run test 'pedaling_cut_order'` — сборка падает (нет параметра).
- [ ] **Шаг 3. Код.** В `pedaling.cpp` строки `:68-74` (от `double target = effort;` до `target = std::min(target,
  *ceiling_nm);`) заменить на

```cpp
            double target = effort;
            if (config_.mash_torque_nm > target && cadence < config_.mash_cadence_rpm)
                target = effort + (config_.mash_torque_nm - effort) * (1. - cadence / config_.mash_cadence_rpm);
            if (cut_after_ceiling) {
                // Launch part 3a (review I4): a rider-limited effort is
                // capped first and the shift cut applies once to the cap.
                if (ceiling_nm)
                    target = std::min(target, *ceiling_nm);
                target = validation::derived(target * torque_factor, "PedalingPolicy.target_effort");
            } else {
                target = validation::derived(target * torque_factor, "PedalingPolicy.target_effort");
                if (ceiling_nm)
                    target = std::min(target, *ceiling_nm);
            }
```

  и протянуть параметр:
  - объявление и определение `update(..., double torque_factor = 1., bool cut_after_ceiling = false)`;
  - `PrepareInputs::cut_after_ceiling`;
  - `prepare(..., RiderShift rider_shift = {}, bool cut_after_ceiling = false)` → `.cut_after_ceiling =
    cut_after_ceiling` в инициализаторе `stage_prepare` (`drivetrain.cpp:481-484`);
  - вызов `pedaling.update(..., in.ceiling, torque_factor, in.cut_after_ceiling)` (`drivetrain.cpp:751-755`).

  В `step.cpp:1328-1334` вызов `stepper_->drive().prepare(...)` получает последним аргументом `false` с
  комментарием «launch part 3a, task 3 passes the resolver's effort_limited».
- [ ] **Шаг 4.** `mise run test 'pedaling_cut_order|native_drivetrain|native_drive_policies|seated_pedaling_cycle|realistic_pedelec_acceptance'`
  — новые зелёные, прежние как в базе.
- [ ] **Шаг 5. Коммит и проверка.** `feat(drivetrain): cut a rider-limited effort after its ceiling`, затем
  `check_task.sh` с базой `50217951` и `ab`.

### Задача 2. Таблица реакции и узел реакции

**Файлы:**
- Создать: `native/src/rider/front_lift_reaction.hpp`, `native/src/rider/front_lift_reaction.cpp`,
  `native/tests/reference/test_front_lift_reaction.cpp`
- Изменить:
  - `native/src/rider/senses_codec.hpp`, `native/src/rider/senses_codec.cpp` — кодеки реакции рядом с кодеком
    восприятия, теми же помощниками (`VNode`, `ncd::exact`);
  - `CMakeLists.txt:426-438` — `front_lift_reaction.cpp` в `bike_configuration`, после `launch_config.cpp`;
  - `native/src/configuration/model_config.hpp`, `native/src/configuration/load.cpp`,
    `native/src/configuration/validate.cpp`;
  - `native/src/initialization/initial_state.cpp:80`, `native/src/initialization/equilibrium_cache.cpp:168`;
  - полные документы физики из описи;
  - `native/tests/reference/Phase3Ports.cmake`.
- Тест: `test_front_lift_reaction.cpp`, `test_welded_profile_config.cpp`

**Интерфейсы** (`native/src/rider/front_lift_reaction.hpp`):

```cpp
// rider/front_lift_reaction.hpp — the rider's reaction to a rising front
// wheel (launch plan, part 3a; «Решено — не сделано», item 2). One reaction
// for the whole ride: on the delayed percepts, unloaded hands that last
// longer than a dwell, together with a fast pitch-up or a held pitch, make
// the predicate; after a decision time the rider eases the effort ceiling
// and/or shifts the lean target forward, each behind its own switch, and
// recovers once the predicate has been absent for clear_s. Times count
// intention ticks.
#pragma once

#include <cstdint>
#include <limits>
#include <optional>

#include "perception.hpp"

namespace rider {

struct FrontLiftReactionConfig {
    double hand_unload_n = std::numeric_limits<double>::quiet_NaN();
    double unload_dwell_s = std::numeric_limits<double>::quiet_NaN();
    double pitch_rate_up_rad_s = std::numeric_limits<double>::quiet_NaN();
    double pitch_rad = std::numeric_limits<double>::quiet_NaN();
    double decision_s = std::numeric_limits<double>::quiet_NaN();
    bool ease_enabled = false;
    bool lean_enabled = false;
    bool rear_brake_tap_enabled = false;   // must stay false: not implemented
    double ease_fraction = std::numeric_limits<double>::quiet_NaN();
    double ease_rate_per_s = std::numeric_limits<double>::quiet_NaN();
    double lean_add_rad = std::numeric_limits<double>::quiet_NaN();
    double lean_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
    double clear_s = std::numeric_limits<double>::quiet_NaN();
};

// The table's own domain; throws std::invalid_argument naming the
// front_lift_reaction field.
void validate_reaction_table(const FrontLiftReactionConfig &table);
// The rules that need the seated-climb record: the three times on the
// intention period grid (at most 2^31 ticks) and the lean add inside the
// forward lean bound. Both the profile post_init and RiderIntent call it.
void validate_reaction_context(const FrontLiftReactionConfig &table, double period_s,
                               double max_forward_lean_rad);

enum class ReactionPhase : std::uint8_t { idle, deciding, acting, releasing };
[[nodiscard]] const char *reaction_phase_name(ReactionPhase phase);          // throws on a value outside the enum
[[nodiscard]] ReactionPhase reaction_phase(std::string_view name);           // throws on an unknown name

struct ReactionCommand {
    ReactionPhase phase = ReactionPhase::idle;
    bool predicate = false;
    double effort_factor = 1.;   // multiplies the effort ceiling
    double lean_add_rad = 0.;    // joins the posture program's lean target
};

class FrontLiftReaction {
public:
    struct State {
        ReactionPhase phase = ReactionPhase::idle;
        bool predicate = false;
        std::int64_t unload_ticks = 0;     // saturates at dwell + 1
        std::int64_t decision_ticks = 0;   // < decision ticks while deciding, else 0
        std::int64_t clear_ticks = 0;      // <= clear ticks while acting, else 0
        double effort_factor = 1.;
        double lean_add_rad = 0.;
    };
    FrontLiftReaction(const FrontLiftReactionConfig &config, double period_s);
    void reset();
    // Rejects nonfinite percept fields without changing anything.
    static void validate(const std::optional<Percepts> &percepts);
    // One intention tick; percepts are absent until the reaction delay has
    // passed and the reaction then stays idle.
    ReactionCommand update(const std::optional<Percepts> &percepts);
    [[nodiscard]] ReactionCommand command() const;
    [[nodiscard]] State state() const;
    // Validates the whole state against the configuration (phase domain,
    // phase/counter/predicate consistency, switch-dependent outputs),
    // then commits; a rejected state leaves the reaction unchanged.
    void restore(const State &state);

private:
    FrontLiftReactionConfig config_;
    double period_s_;
    std::int64_t dwell_ticks_ = 0, decision_ticks_ = 0, clear_ticks_ = 0;
    State s_;
};

} // namespace rider
```

`senses_codec.hpp`, после кодека восприятия:

```cpp
// The reaction's restorable document: {"phase", "predicate",
// "unload_ticks", "decision_ticks", "clear_ticks", "effort_factor",
// "lean_add_rad"} — exact keys, phase one of idle/deciding/acting/releasing,
// int64 counters, finite reals. Reachability belongs to
// FrontLiftReaction::restore.
[[nodiscard]] bike::io::Value encode_reaction_state(const FrontLiftReaction::State &state);
[[nodiscard]] FrontLiftReaction::State decode_reaction_state(const bike::io::Value &v, std::string_view path);
// The runtime document of [front_lift_reaction]: the 13 keys, exact, then
// validate_reaction_table.
[[nodiscard]] bike::io::Value encode_reaction_config(const FrontLiftReactionConfig &config);
[[nodiscard]] FrontLiftReactionConfig decode_reaction_config(const bike::io::Value &v, std::string_view path);
```

(`senses_codec.hpp` включает `front_lift_reaction.hpp`.)

- [ ] **Шаг 1. Тесты (падают).** `test_front_lift_reaction.cpp`:

```cpp
// Launch part 3a — the front-lift reaction node: its timing on the delayed
// percepts, the two independent actions, recovery, the codec round trip
// with bitwise continuation, atomic restore and the table rules.
#include "support/fixture_test.hpp"
#include "io/json.hpp"
#include "rider/front_lift_reaction.hpp"
#include "rider/senses_codec.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr double kPeriod = .00125;

template <typename Action>
void rejects(Action &&action, std::string_view needle, std::string_view what) {
    bool threw = false;
    try {
        std::forward<Action>(action)();
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(std::string_view(failure.what()).find(needle) != std::string_view::npos,
                    std::string(what) + " names " + std::string(needle) + " (" + failure.what() + ")");
    }
    bt::require(threw, std::string(what) + " is rejected");
}

// Dwell 4 ticks, decision 8 ticks, clear 16 ticks; both actions on.
rider::FrontLiftReactionConfig table() {
    return {.hand_unload_n = 20., .unload_dwell_s = 4 * kPeriod, .pitch_rate_up_rad_s = .5,
            .pitch_rad = .17, .decision_s = 8 * kPeriod, .ease_enabled = true, .lean_enabled = true,
            .rear_brake_tap_enabled = false, .ease_fraction = .7, .ease_rate_per_s = 5.,
            .lean_add_rad = .1, .lean_rate_rad_s = .5, .clear_s = 16 * kPeriod};
}

rider::Percepts percept(double hand_press_n, double pitch_rate, double pitch) {
    rider::Percepts p;
    p.senses.hand_press_n = hand_press_n;
    p.pitch_rate_up_rad_s = pitch_rate;
    p.pitch_rad = pitch;
    return p;
}

std::string text(const rider::FrontLiftReaction &r) {
    return bike::io::canonical_json(rider::encode_reaction_state(r.state()));
}

// Unloaded hands must last longer than the dwell (4 samples) before the
// predicate rises; the action starts after the decision time (8 ticks);
// the first acting tick already moves both outputs one rate step.
void test_predicate_dwell_and_decision_timing() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    const auto lifting = percept(5., 1., 0.);
    std::vector<rider::ReactionCommand> out;
    for (int tick = 0; tick < 20; ++tick) out.push_back(reaction.update(lifting));
    for (std::size_t tick = 0; tick < 4; ++tick)
        bt::require(!out[tick].predicate, "no predicate within the dwell");
    bt::require(out[4].predicate && out[4].phase == rider::ReactionPhase::deciding,
                "the predicate rises when the unload outlasts the dwell");
    for (std::size_t tick = 4; tick < 12; ++tick)
        bt::require(out[tick].phase == rider::ReactionPhase::deciding && out[tick].effort_factor == 1.,
                    "the rider is still deciding");
    bt::require(out[12].phase == rider::ReactionPhase::acting, "the action starts after the decision time");
    bt::require(std::abs(out[12].effort_factor - (1. - 5. * kPeriod)) < 1e-15 &&
                    std::abs(out[12].lean_add_rad - .5 * kPeriod) < 1e-15,
                "acting moves both outputs one rate step");
}

// A held-high wheel triggers through the accumulated pitch with a small
// rate; light hands without pitch never trigger; no percepts keep idle.
void test_held_pitch_triggers_and_light_hands_alone_do_not() {
    rider::FrontLiftReaction held(table(), kPeriod);
    rider::ReactionCommand last;
    for (int tick = 0; tick < 5; ++tick) last = held.update(percept(5., .01, .3));
    bt::require(last.predicate, "a held pitch above the threshold makes the predicate");
    rider::FrontLiftReaction light(table(), kPeriod);
    for (int tick = 0; tick < 40; ++tick) last = light.update(percept(5., .01, .01));
    bt::require(!last.predicate && last.phase == rider::ReactionPhase::idle, "light hands alone never trigger");
    rider::FrontLiftReaction blind(table(), kPeriod);
    for (int tick = 0; tick < 40; ++tick) last = blind.update(std::nullopt);
    bt::require(last.phase == rider::ReactionPhase::idle && !last.predicate, "no percepts, no reaction");
}

// A predicate that vanishes during the decision returns to idle without
// any action.
void test_short_predicate_is_dropped_while_deciding() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    rider::ReactionCommand last;
    for (int tick = 0; tick < 6; ++tick) last = reaction.update(percept(5., 1., 0.));
    bt::require(last.phase == rider::ReactionPhase::deciding, "deciding");
    last = reaction.update(percept(80., 0., 0.));
    bt::require(last.phase == rider::ReactionPhase::idle && last.effort_factor == 1. && last.lean_add_rad == 0.,
                "a short lift is dropped before any action");
}

// Each action has its own switch; with both off the phases still run and
// both outputs stay exactly neutral.
void test_each_action_has_its_own_switch() {
    for (const auto &[ease, lean] : {std::pair{true, false}, std::pair{false, true}, std::pair{false, false}}) {
        auto t = table();
        t.ease_enabled = ease;
        t.lean_enabled = lean;
        rider::FrontLiftReaction reaction(t, kPeriod);
        rider::ReactionCommand last;
        for (int tick = 0; tick < 200; ++tick) last = reaction.update(percept(5., 1., 0.));
        bt::require(last.phase == rider::ReactionPhase::acting, "acting with any switch setting");
        bt::require((last.effort_factor < 1.) == ease && (ease || last.effort_factor == 1.), "ease follows its switch");
        bt::require((last.lean_add_rad > 0.) == lean && (lean || last.lean_add_rad == 0.), "lean follows its switch");
        if (ease) bt::require(std::abs(last.effort_factor - .3) < 1e-12, "the ease settles at 1 - ease_fraction");
        if (lean) bt::require(last.lean_add_rad == .1, "the lean add settles at lean_add_rad");
    }
}

// The reaction ends when the predicate has been absent longer than
// clear_s; both outputs return at their rates and the phase becomes idle.
void test_recovery_after_clear() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    rider::ReactionCommand last;
    for (int tick = 0; tick < 200; ++tick) last = reaction.update(percept(5., 1., 0.));
    for (int tick = 0; tick < 16; ++tick) {
        last = reaction.update(percept(80., 0., 0.));
        bt::require(last.phase == rider::ReactionPhase::acting, "still acting within clear_s");
    }
    last = reaction.update(percept(80., 0., 0.));
    bt::require(last.phase == rider::ReactionPhase::releasing, "releasing after clear_s");
    for (int tick = 0; tick < 400 && last.phase != rider::ReactionPhase::idle; ++tick)
        last = reaction.update(percept(80., 0., 0.));
    bt::require(last.phase == rider::ReactionPhase::idle && last.effort_factor == 1. && last.lean_add_rad == 0.,
                "idle again with both outputs back exactly");
}

// Serialize → read → continue: restored mid-decision and mid-release, the
// reaction continues bitwise (whole state compared as canonical JSON).
void test_codec_round_trip_and_continuation() {
    const auto input = [](int tick) { return tick < 200 ? percept(5., 1., 0.) : percept(80., 0., 0.); };
    for (const int split : {7, 230}) {
        rider::FrontLiftReaction original(table(), kPeriod);
        for (int tick = 0; tick < split; ++tick) (void)original.update(input(tick));
        rider::FrontLiftReaction copy(table(), kPeriod);
        copy.restore(rider::decode_reaction_state(rider::encode_reaction_state(original.state()), "reaction"));
        for (int tick = split; tick < split + 300; ++tick) {
            (void)original.update(input(tick));
            (void)copy.update(input(tick));
            bt::require(text(original) == text(copy), "the restored reaction continues bitwise");
        }
    }
}

// Restore rejects unreachable states atomically, including actions a
// switch forbids, a phase outside the enum and inconsistent counters; a
// reachable state from the same configuration is accepted.
void test_restore_rejects_unreachable_states() {
    auto off = table();
    off.ease_enabled = false;
    off.lean_enabled = false;
    rider::FrontLiftReaction target(off, kPeriod);
    for (int tick = 0; tick < 6; ++tick) (void)target.update(percept(5., 1., 0.));
    const std::string before = text(target);
    auto bad = target.state();
    bad.phase = rider::ReactionPhase::acting;
    bad.decision_ticks = 0;
    bad.effort_factor = .5;
    rejects([&] { target.restore(bad); }, "effort_factor", "an ease the switch forbids");
    bad.effort_factor = 1.;
    bad.lean_add_rad = .05;
    rejects([&] { target.restore(bad); }, "lean_add_rad", "a lean the switch forbids");
    bad = target.state();
    bad.phase = static_cast<rider::ReactionPhase>(9);
    rejects([&] { target.restore(bad); }, "phase", "a phase outside the enum");
    bad = target.state();
    bad.predicate = false;
    rejects([&] { target.restore(bad); }, "predicate", "deciding without the predicate");
    bad = target.state();
    bad.unload_ticks = std::numeric_limits<std::int64_t>::max();
    rejects([&] { target.restore(bad); }, "unload_ticks", "an unload counter past saturation");
    bt::require(text(target) == before, "rejected restores change nothing");
    target.restore(target.state());   // control: its own reachable state
    rejects([&] { (void)rider::decode_reaction_state(bike::io::Value{bike::io::Value::Object{}}, "reaction"); },
            "reaction", "an empty document");
}

// The table's own rules and the period/posture context, each with the
// valid control passing.
void test_table_rules() {
    rider::validate_reaction_table(table());
    rider::validate_reaction_context(table(), kPeriod, .35);
    auto with = [](auto change) { auto t = table(); change(t); return t; };
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.rear_brake_tap_enabled = true; })); },
            "not implemented", "the rear brake tap");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.ease_fraction = 1.2; })); },
            "ease_fraction", "an ease beyond one");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.ease_rate_per_s = 0.; })); },
            "ease_rate_per_s", "a zero ease rate");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.lean_add_rad = -.1; })); },
            "lean_add_rad", "a backward lean add");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.lean_rate_rad_s = 0.; })); },
            "lean_rate_rad_s", "a zero lean rate");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.pitch_rate_up_rad_s = 0.; })); },
            "pitch_rate_up_rad_s", "a zero pitch-rate threshold");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) {
                t.pitch_rad = std::numeric_limits<double>::quiet_NaN(); })); },
            "pitch_rad", "a NaN pitch threshold");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.clear_s = -kPeriod; })); },
            "clear_s", "a negative clear time");
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.decision_s = 1.5 * kPeriod; }), kPeriod, .35); },
            "decision_s", "a decision time off the tick grid");
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.clear_s = 0x1p40 * kPeriod; }), kPeriod, .35); },
            "clear_s", "a clear time beyond 2^31 ticks");
    rejects([&] { rider::validate_reaction_context(table(), kPeriod, .05); },
            "lean_add_rad", "a lean add beyond the forward bound");
}

} // namespace

int main() {
    return bt::run([] {
        test_predicate_dwell_and_decision_timing();
        test_held_pitch_triggers_and_light_hands_alone_do_not();
        test_short_predicate_is_dropped_while_deciding();
        test_each_action_has_its_own_switch();
        test_recovery_after_clear();
        test_codec_round_trip_and_continuation();
        test_restore_rejects_unreachable_states();
        test_table_rules();
    });
}
```

  Регистрация в `Phase3Ports.cmake` после `pedaling_cut_order`:

```cmake
# Launch part 3a, tasks 2-3 — the front-lift reaction node and its place
# in the rider intent.
bike_add_reference_test(front_lift_reaction ENGINE
  SOURCES test_front_lift_reaction.cpp LIBRARIES bike_app
  LABELS launch rider)
```

  В `test_welded_profile_config.cpp` (помощники `rejects`, `load_edited_profile`, `replace_once` там уже есть):

```cpp
// The welded profile carries the switched-off reaction with the
// preliminary numbers of the inventory.
[[gnu::noinline]] void test_reaction_table_is_switched_off() {
    const auto &r = profile().front_lift_reaction;
    bt::require(!r.ease_enabled && !r.lean_enabled && !r.rear_brake_tap_enabled, "both actions off");
    bt::require(r.hand_unload_n == 20. && r.unload_dwell_s == .05 && r.pitch_rate_up_rad_s == .5 &&
                    r.pitch_rad == .17 && r.decision_s == .1 && r.ease_fraction == .7 &&
                    r.ease_rate_per_s == 5. && r.lean_add_rad == .1 && r.lean_rate_rad_s == .5 && r.clear_s == .3,
                "the preliminary numbers");
}

// Every [front_lift_reaction] key and the section are required; the period
// grid and the forward bound are checked against [seated_climb].
[[gnu::noinline]] void test_reaction_keys_and_context_rules() {
    for (const char *line : {"hand_unload_n = 20.0\n", "unload_dwell_s = 0.05\n", "pitch_rate_up_rad_s = 0.5\n",
                             "pitch_rad = 0.17\n", "decision_s = 0.1\n", "ease_enabled = false\n",
                             "lean_enabled = false\n", "rear_brake_tap_enabled = false\n",
                             "ease_fraction = 0.7\n", "ease_rate_per_s = 5.0\n", "lean_add_rad = 0.1\n",
                             "lean_rate_rad_s = 0.5\n", "clear_s = 0.3\n"}) {
        const std::string key(std::string_view(line).substr(0, std::string_view(line).find(' ')));
        rejects([&] { (void)load_edited_profile([&](std::string &t) { replace_once(t, line, ""); }); },
                key, "a profile without " + key);
    }
    rejects([] {
        (void)load_edited_profile([](std::string &t) {
            const auto from = t.find("[front_lift_reaction]\n");
            const std::string_view last = "clear_s = 0.3\n";
            const auto to = t.find(last, from);
            bt::require(from != std::string::npos && to != std::string::npos, "the profile has the reaction block");
            t.erase(from, to + last.size() - from);
        });
    }, "front_lift_reaction", "a profile without [front_lift_reaction]");
    auto config = profile();
    config.front_lift_reaction.decision_s = .1 + .0005;
    rejects([&] { cfg::validate(config); }, "decision_s", "a decision time off the period grid");
    config = profile();
    config.front_lift_reaction.lean_add_rad = config.seated_climb.max_forward_lean_rad + .01;
    rejects([&] { cfg::validate(config); }, "lean_add_rad", "a lean add beyond the forward bound");
    cfg::validate(profile());   // control
}
```

  Оба вызова добавить в `main` этого файла.
- [ ] **Шаг 2.** `mise run test 'front_lift_reaction|welded_profile_config'` — сборка падает.
- [ ] **Шаг 3. Код.**
  - `front_lift_reaction.cpp`:

```cpp
#include "front_lift_reaction.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rider {
namespace {

[[noreturn]] void reject(std::string_view field, std::string_view why) {
    throw std::invalid_argument("front_lift_reaction." + std::string(field) + ": " + std::string(why));
}

void finite_at_least(double value, double minimum, std::string_view field) {
    if (!std::isfinite(value) || value < minimum) reject(field, "must be finite and not below its minimum");
}

void finite_positive(double value, std::string_view field) {
    if (!std::isfinite(value) || value <= 0.) reject(field, "must be finite and positive");
}

// A time on the intention grid, at most 2^31 ticks (exactly representable,
// far beyond any ride, and safe to count in int64).
std::int64_t ticks(double seconds, double period_s, std::string_view field) {
    const double n = std::rint(seconds / period_s);
    if (!(n >= 0. && n <= 0x1p31) || std::abs(n * period_s - seconds) > 1e-9)
        reject(field, "must be a whole multiple of the intention period, at most 2^31 ticks");
    return static_cast<std::int64_t>(n);
}

double step_toward(double value, double target, double step) {
    if (std::abs(target - value) <= step) return target;
    return value + (target > value ? step : -step);
}

} // namespace

void validate_reaction_table(const FrontLiftReactionConfig &t) {
    if (!std::isfinite(t.hand_unload_n)) reject("hand_unload_n", "must be finite");
    finite_at_least(t.unload_dwell_s, 0., "unload_dwell_s");
    finite_positive(t.pitch_rate_up_rad_s, "pitch_rate_up_rad_s");
    finite_positive(t.pitch_rad, "pitch_rad");
    finite_at_least(t.decision_s, 0., "decision_s");
    if (t.rear_brake_tap_enabled) reject("rear_brake_tap_enabled", "the rear brake tap is not implemented");
    if (!std::isfinite(t.ease_fraction) || t.ease_fraction < 0. || t.ease_fraction > 1.)
        reject("ease_fraction", "must lie in [0, 1]");
    finite_positive(t.ease_rate_per_s, "ease_rate_per_s");
    finite_at_least(t.lean_add_rad, 0., "lean_add_rad");
    finite_positive(t.lean_rate_rad_s, "lean_rate_rad_s");
    finite_at_least(t.clear_s, 0., "clear_s");
}

void validate_reaction_context(const FrontLiftReactionConfig &t, double period_s, double max_forward_lean_rad) {
    validate_reaction_table(t);
    if (!std::isfinite(period_s) || period_s <= 0.)
        throw std::invalid_argument("front-lift reaction needs a positive intention period");
    (void)ticks(t.unload_dwell_s, period_s, "unload_dwell_s");
    (void)ticks(t.decision_s, period_s, "decision_s");
    (void)ticks(t.clear_s, period_s, "clear_s");
    if (t.lean_add_rad > max_forward_lean_rad) reject("lean_add_rad", "beyond the seated forward lean bound");
}

const char *reaction_phase_name(ReactionPhase phase) {
    switch (phase) {
    case ReactionPhase::idle: return "idle";
    case ReactionPhase::deciding: return "deciding";
    case ReactionPhase::acting: return "acting";
    case ReactionPhase::releasing: return "releasing";
    }
    throw std::invalid_argument("front-lift reaction phase outside the enum");
}

ReactionPhase reaction_phase(std::string_view name) {
    if (name == "idle") return ReactionPhase::idle;
    if (name == "deciding") return ReactionPhase::deciding;
    if (name == "acting") return ReactionPhase::acting;
    if (name == "releasing") return ReactionPhase::releasing;
    throw std::invalid_argument("front-lift reaction phase: unknown name");
}

FrontLiftReaction::FrontLiftReaction(const FrontLiftReactionConfig &config, double period_s)
    : config_(config), period_s_(period_s) {
    validate_reaction_table(config_);
    if (!std::isfinite(period_s_) || period_s_ <= 0.)
        throw std::invalid_argument("front-lift reaction needs a positive intention period");
    dwell_ticks_ = ticks(config_.unload_dwell_s, period_s_, "unload_dwell_s");
    decision_ticks_ = ticks(config_.decision_s, period_s_, "decision_s");
    clear_ticks_ = ticks(config_.clear_s, period_s_, "clear_s");
}

void FrontLiftReaction::reset() { s_ = {}; }

void FrontLiftReaction::validate(const std::optional<Percepts> &p) {
    if (p && (!std::isfinite(p->senses.hand_press_n) || !std::isfinite(p->pitch_rate_up_rad_s) ||
              !std::isfinite(p->pitch_rad)))
        throw std::invalid_argument("front-lift reaction percepts must be finite");
}

ReactionCommand FrontLiftReaction::update(const std::optional<Percepts> &percepts) {
    validate(percepts);
    State n = s_;
    if (!percepts) {
        n.unload_ticks = 0;
        n.predicate = false;
    } else {
        n.unload_ticks = percepts->senses.hand_press_n < config_.hand_unload_n
                             ? std::min(n.unload_ticks + 1, dwell_ticks_ + 1)
                             : 0;
        n.predicate = n.unload_ticks > dwell_ticks_ &&
                      (percepts->pitch_rate_up_rad_s > config_.pitch_rate_up_rad_s ||
                       percepts->pitch_rad > config_.pitch_rad);
    }
    switch (n.phase) {
    case ReactionPhase::idle:
        if (n.predicate) { n.phase = ReactionPhase::deciding; n.decision_ticks = 0; }
        break;
    case ReactionPhase::deciding:
        if (!n.predicate) { n.phase = ReactionPhase::idle; n.decision_ticks = 0; }
        else ++n.decision_ticks;
        break;
    case ReactionPhase::acting:
        n.clear_ticks = n.predicate ? 0 : n.clear_ticks + 1;
        if (n.clear_ticks > clear_ticks_) { n.phase = ReactionPhase::releasing; n.clear_ticks = 0; }
        break;
    case ReactionPhase::releasing:
        if (n.predicate) n.phase = ReactionPhase::acting;
        break;
    }
    if (n.phase == ReactionPhase::deciding && n.decision_ticks >= decision_ticks_) {
        n.phase = ReactionPhase::acting;
        n.decision_ticks = 0;
        n.clear_ticks = 0;
    }
    const bool acting = n.phase == ReactionPhase::acting;
    n.effort_factor = step_toward(n.effort_factor,
                                  acting && config_.ease_enabled ? 1. - config_.ease_fraction : 1.,
                                  config_.ease_rate_per_s * period_s_);
    n.lean_add_rad = step_toward(n.lean_add_rad, acting && config_.lean_enabled ? config_.lean_add_rad : 0.,
                                 config_.lean_rate_rad_s * period_s_);
    if (n.phase == ReactionPhase::releasing && n.effort_factor == 1. && n.lean_add_rad == 0.)
        n.phase = ReactionPhase::idle;
    s_ = n;
    return command();
}

ReactionCommand FrontLiftReaction::command() const {
    return {.phase = s_.phase, .predicate = s_.predicate, .effort_factor = s_.effort_factor,
            .lean_add_rad = s_.lean_add_rad};
}

FrontLiftReaction::State FrontLiftReaction::state() const { return s_; }

void FrontLiftReaction::restore(const State &s) {
    (void)reaction_phase_name(s.phase);   // throws on a phase outside the enum
    const auto bad = [](std::string_view field, std::string_view why) {
        throw std::invalid_argument("front-lift reaction " + std::string(field) + ": " + std::string(why));
    };
    if (s.unload_ticks < 0 || s.unload_ticks > dwell_ticks_ + 1) bad("unload_ticks", "outside [0, dwell + 1]");
    if (s.predicate && s.unload_ticks <= dwell_ticks_) bad("predicate", "set without an unload past the dwell");
    if (s.decision_ticks < 0 || (s.phase == ReactionPhase::deciding ? s.decision_ticks >= decision_ticks_
                                                                     : s.decision_ticks != 0))
        bad("decision_ticks", "outside the decision window of its phase");
    if (s.clear_ticks < 0 || (s.phase == ReactionPhase::acting ? s.clear_ticks > clear_ticks_
                                                                : s.clear_ticks != 0))
        bad("clear_ticks", "outside the clear window of its phase");
    if (s.phase == ReactionPhase::idle && s.predicate) bad("predicate", "idle with the predicate");
    if (s.phase == ReactionPhase::deciding && !s.predicate) bad("predicate", "deciding without the predicate");
    if (!std::isfinite(s.effort_factor) || s.effort_factor > 1. ||
        s.effort_factor < (config_.ease_enabled ? 1. - config_.ease_fraction : 1.))
        bad("effort_factor", "outside the range its switch allows");
    if (!std::isfinite(s.lean_add_rad) || s.lean_add_rad < 0. ||
        s.lean_add_rad > (config_.lean_enabled ? config_.lean_add_rad : 0.))
        bad("lean_add_rad", "outside the range its switch allows");
    if ((s.phase == ReactionPhase::idle || s.phase == ReactionPhase::deciding) &&
        (s.effort_factor != 1. || s.lean_add_rad != 0.))
        bad("phase", "no action may be in force before acting");
    s_ = s;
}

} // namespace rider
```

  - Кодеки в `senses_codec.cpp`, в пространстве `rider` после кодека восприятия:

```cpp
bio::Value encode_reaction_state(const FrontLiftReaction::State &s) {
    bio::Value::Object out;
    out.emplace_back("phase", bio::Value{std::string(reaction_phase_name(s.phase))});
    out.emplace_back("predicate", bio::Value{s.predicate});
    out.emplace_back("unload_ticks", integer_value(s.unload_ticks));
    out.emplace_back("decision_ticks", integer_value(s.decision_ticks));
    out.emplace_back("clear_ticks", integer_value(s.clear_ticks));
    out.emplace_back("effort_factor", real_value(s.effort_factor));
    out.emplace_back("lean_add_rad", real_value(s.lean_add_rad));
    return bio::Value{std::move(out)};
}

FrontLiftReaction::State decode_reaction_state(const bio::Value &value, std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d, keys("phase", "predicate", "unload_ticks", "decision_ticks", "clear_ticks",
                       "effort_factor", "lean_add_rad"));
    FrontLiftReaction::State s;
    try {
        s.phase = reaction_phase(d["phase"].string(std::string(path) + ".phase"));
    } catch (const std::invalid_argument &) {
        invalid(std::string(path) + ".phase", "unknown reaction phase");
    }
    s.predicate = d["predicate"].boolean(path);
    s.unload_ticks = d["unload_ticks"].integer(path);
    s.decision_ticks = d["decision_ticks"].integer(path);
    s.clear_ticks = d["clear_ticks"].integer(path);
    s.effort_factor = d["effort_factor"].finite_real(path);
    s.lean_add_rad = d["lean_add_rad"].finite_real(path);
    return s;
}

bio::Value encode_reaction_config(const FrontLiftReactionConfig &c) {
    bio::Value::Object out;
    out.emplace_back("hand_unload_n", real_value(c.hand_unload_n));
    out.emplace_back("unload_dwell_s", real_value(c.unload_dwell_s));
    out.emplace_back("pitch_rate_up_rad_s", real_value(c.pitch_rate_up_rad_s));
    out.emplace_back("pitch_rad", real_value(c.pitch_rad));
    out.emplace_back("decision_s", real_value(c.decision_s));
    out.emplace_back("ease_enabled", bio::Value{c.ease_enabled});
    out.emplace_back("lean_enabled", bio::Value{c.lean_enabled});
    out.emplace_back("rear_brake_tap_enabled", bio::Value{c.rear_brake_tap_enabled});
    out.emplace_back("ease_fraction", real_value(c.ease_fraction));
    out.emplace_back("ease_rate_per_s", real_value(c.ease_rate_per_s));
    out.emplace_back("lean_add_rad", real_value(c.lean_add_rad));
    out.emplace_back("lean_rate_rad_s", real_value(c.lean_rate_rad_s));
    out.emplace_back("clear_s", real_value(c.clear_s));
    return bio::Value{std::move(out)};
}

FrontLiftReactionConfig decode_reaction_config(const bio::Value &value, std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d, keys("hand_unload_n", "unload_dwell_s", "pitch_rate_up_rad_s", "pitch_rad", "decision_s",
                       "ease_enabled", "lean_enabled", "rear_brake_tap_enabled", "ease_fraction",
                       "ease_rate_per_s", "lean_add_rad", "lean_rate_rad_s", "clear_s"));
    FrontLiftReactionConfig c;
    c.hand_unload_n = d["hand_unload_n"].finite_real(path);
    c.unload_dwell_s = d["unload_dwell_s"].finite_real(path);
    c.pitch_rate_up_rad_s = d["pitch_rate_up_rad_s"].finite_real(path);
    c.pitch_rad = d["pitch_rad"].finite_real(path);
    c.decision_s = d["decision_s"].finite_real(path);
    c.ease_enabled = d["ease_enabled"].boolean(path);
    c.lean_enabled = d["lean_enabled"].boolean(path);
    c.rear_brake_tap_enabled = d["rear_brake_tap_enabled"].boolean(path);
    c.ease_fraction = d["ease_fraction"].finite_real(path);
    c.ease_rate_per_s = d["ease_rate_per_s"].finite_real(path);
    c.lean_add_rad = d["lean_add_rad"].finite_real(path);
    c.lean_rate_rad_s = d["lean_rate_rad_s"].finite_real(path);
    c.clear_s = d["clear_s"].finite_real(path);
    validate_reaction_table(c);
    return c;
}
```

    `real_value`, `integer_value`, `invalid`, `mapping`, `Dict` — помощники этого файла (их используют кодеки
    ощущений `senses_codec.cpp:248-300`).
  - `model_config.hpp`, по образцу `rider_launch`:
    - `#include "../rider/front_lift_reaction.hpp"`, `using FrontLiftReactionConfig = ::rider::FrontLiftReactionConfig;`;
    - член `FrontLiftReactionConfig front_lift_reaction;` после `rider_launch`;
    - `void post_init(const FrontLiftReactionConfig&);` в блоке объявлений `detail`;
    - таблица полей перед `kDeclaredSimulationPhysicsConfig`:

```cpp
// Launch part 3a — [front_lift_reaction]: every key is required and the
// record has no declared-default instance.
#define FLR_REAL(f) \
    {.name=#f, .type="<class 'float'>", .kind=FieldKind::Real, \
     .offset=offsetof(FrontLiftReactionConfig, f), .required=true, .factory="", .nested=nullptr, .nested2=nullptr}
#define FLR_BOOL(f) \
    {.name=#f, .type="<class 'bool'>", .kind=FieldKind::Boolean, \
     .offset=offsetof(FrontLiftReactionConfig, f), .required=true, .factory="", .nested=nullptr, .nested2=nullptr}
inline constexpr auto kFrontLiftReactionConfigFields = std::to_array<FieldDecl>({
    FLR_REAL(hand_unload_n), FLR_REAL(unload_dwell_s), FLR_REAL(pitch_rate_up_rad_s), FLR_REAL(pitch_rad),
    FLR_REAL(decision_s), FLR_BOOL(ease_enabled), FLR_BOOL(lean_enabled), FLR_BOOL(rear_brake_tap_enabled),
    FLR_REAL(ease_fraction), FLR_REAL(ease_rate_per_s), FLR_REAL(lean_add_rad), FLR_REAL(lean_rate_rad_s),
    FLR_REAL(clear_s),
});
#undef FLR_REAL
#undef FLR_BOOL
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kFrontLiftReactionConfigSchema{
    .path="bike_sim.physics.front_lift_reaction.FrontLiftReactionConfig",
    .fields=kFrontLiftReactionConfigFields, .declared_default=nullptr};
```

    - в `kSimulationPhysicsConfigFields` после `rider_launch`:

```cpp
    {.name="front_lift_reaction",
     .type="<class 'bike_sim.physics.front_lift_reaction.FrontLiftReactionConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(SimulationPhysicsConfig, front_lift_reaction),
     .required=true, .factory="", .nested=&kFrontLiftReactionConfigSchema, .nested2=nullptr},
```

  - `load.cpp`, `construct_record`: `BIKE_T13_RECORD(FrontLiftReactionConfig, kFrontLiftReactionConfigSchema)`
    после строки `RiderLaunchConfig`.
  - `validate.cpp`: `detail::post_init(const FrontLiftReactionConfig &c)` вызывает
    `::rider::validate_reaction_table(c)`. В корневом `post_init` после проверок `rider_launch`:

```cpp
    detail::post_init(config.front_lift_reaction);
    // Launch part 3a: the reaction counts intention ticks and leans inside
    // the seated forward bound — the same context check RiderIntent runs on
    // runtime documents.
    ::rider::validate_reaction_context(config.front_lift_reaction, config.seated_climb.period_s,
                                       config.seated_climb.max_forward_lean_rad);
```

    `default_physics()` и программные конфигурации описи:

```cpp
    config.front_lift_reaction = {.hand_unload_n = 20., .unload_dwell_s = .05, .pitch_rate_up_rad_s = .5,
                                  .pitch_rad = .17, .decision_s = .1, .ease_enabled = false,
                                  .lean_enabled = false, .rear_brake_tap_enabled = false,
                                  .ease_fraction = .7, .ease_rate_per_s = 5., .lean_add_rad = .1,
                                  .lean_rate_rad_s = .5, .clear_s = .3};
```

    У конфигурации с выключенными намерениями и заглушкой `seated_climb.period_s` (если такая встретится в описи)
    проверка контекста не проходит. Тогда проверка контекста делается только при `config.seated_climb.enabled`,
    а это ветвление — в отчёт с найденным местом.
  - 16 файлов физики — скрипт по образцу шага 3 задачи 1 части 2; блок вставляется перед `[monitors]`:

```toml
# Front-lift reaction (launch plan, part 3a): the rider eases the effort
# and/or leans forward when the hands unload and the bike pitches up.
# Both actions are off until the launch plan is switched on (part 4);
# preliminary numbers, calibrated with the launch plan.
[front_lift_reaction]
hand_unload_n = 20.0
unload_dwell_s = 0.05
pitch_rate_up_rad_s = 0.5
pitch_rad = 0.17
decision_s = 0.1
ease_enabled = false
lean_enabled = false
rear_brake_tap_enabled = false
ease_fraction = 0.7
ease_rate_per_s = 5.0
lean_add_rad = 0.1
lean_rate_rad_s = 0.5
clear_s = 0.3
```

  - `initial_state.cpp:80`: в той же строке, что `research`/`monitors`/`rider_launch`, убирать и
    `front_lift_reaction`; в комментарий над ней — «and the front-lift reaction, which steers the ride, never the
    rest pose».
  - `equilibrium_cache.cpp:168`: в общее исключение полей (то, что убирает `equilibrium_cache_enabled` и
    `equilibrium_state_file`) добавить `field.first=="front_lift_reaction"`, с тем же комментарием.
- [ ] **Шаг 4.** Шаблон тестов шага 4 задачи 1 части 2 плюс `front_lift_reaction`. Проверка «`[front_lift_reaction]`
  столько же, сколько `[monitors]`» печатает пустой список.
- [ ] **Шаг 5. Перезаписи** — по правилу; список или «нет».
- [ ] **Шаг 6. Коммит и проверка.** `feat(rider): the front-lift reaction table and node`; `check_task.sh`, база —
  коммит задачи 1, с `ab`.

### Задача 3. Восприятие и реакция в намерениях райдера

**Файлы:**
- Создать: `native/src/rider/intent_signals.hpp` (перенос `IntentSignals` из `intent.hpp:58-66`).
- Изменить:
  - `native/src/rider/perception.hpp` (`#include "intent_signals.hpp"` вместо `intent.hpp`; `percepts()`;
    `static void validate(const SenseSample&)`), `native/src/rider/perception.cpp`;
  - `native/src/rider/intent.hpp`, `native/src/rider/intent.cpp` (программа позы, политика, `RiderIntent`);
  - `native/src/rider/state_codec.cpp:2278-2400` (ключи `perception`, `reaction`);
  - `native/src/runtime/config.hpp:57-87` (`RuntimeConfig::rider_reaction`), `native/src/runtime/step.hpp:50-66`
    (`StepConfig::reaction`);
  - `native/src/runtime/project_config.cpp:505`, `native/src/runtime/bootstrap.cpp:249-255,300-302`,
    `native/src/runtime/runtime.cpp:350-354`;
  - `native/src/runtime/step.cpp:1024,1264-1269,1328-1337,2031-2049`;
  - `native/resources/recording/catalog-v1.json`, `native/src/recording/catalog_data.inc` (описатели канала по
    образцу `rider_senses`, коммит `c5ceece3`);
  - `native/tests/recording/test_catalog.cpp:68-72` (511 и комментарий);
  - вызовы из описи; документы из описи.
- Тест: `test_front_lift_reaction.cpp` (намерения), `test_rider_senses.cpp` (канал в поездке).

**Интерфейсы:**

```cpp
// intent_signals.hpp — seated_climb.py SeatedClimbSignals (moved from
// intent.hpp so perception.hpp and intent.hpp do not include each other).
struct IntentSignals { /* the five members, unchanged */ };

// perception.hpp
static void Perception::validate(const SenseSample &sample);     // the update() domain check, public
[[nodiscard]] std::optional<Percepts> Perception::percepts() const;  // from the state

// intent.hpp
double SeatedPostureProgram::update(double time_s, double road_grade, double dt_s,
    std::optional<double> front_load_share = std::nullopt, std::optional<double> lean_limit_rad = std::nullopt,
    std::optional<double> dead_time_s = std::nullopt, std::optional<double> min_lean_rad = std::nullopt,
    double extra_target_rad = 0.);
Intent SeatedClimbPolicy::update(const IntentSignals &signals, double dt_s, double road_grade,
    std::optional<double> preview_grade = std::nullopt, std::optional<double> lean_limit_rad = std::nullopt,
    std::optional<double> min_lean_rad = std::nullopt, double extra_lean_rad = 0.);

struct IntentResolution {
    IntentControl control;
    bool effort_limited = false;   // the rider's own reduction is in force
};
RiderIntent(IntentConfig config, FrontLiftReactionConfig reaction, double dt_s);
IntentResolution resolve_full(IntentControl control, const IntentSignals &signals, const RiderSenses &senses,
    std::int64_t step, double road_grade, std::optional<double> preview_grade = std::nullopt,
    std::optional<double> lean_limit_rad = std::nullopt, bool active = true, bool advance = true,
    std::optional<double> min_lean_rad = std::nullopt);
IntentControl resolve(/* same arguments */);   // resolve_full(...).control
[[nodiscard]] ReactionCommand reaction() const;
struct RiderIntent::State {
    std::int64_t last_tick_step = -1;
    Intent intent;
    SeatedClimbPolicy::State policy;
    SeatedPostureProgram::State program;
    std::optional<Perception::State> perception;        // engaged exactly when the intent is enabled
    std::optional<FrontLiftReaction::State> reaction;   // engaged exactly when the intent is enabled
};
```

- [ ] **Шаг 1. Тесты (падают)** в `test_front_lift_reaction.cpp` (добавить `#include "rider/intent.hpp"`,
  `#include "rider/state_codec.hpp"`, `#include "configuration/model_config.hpp"`, `<bit>`, `<cstring>`):

```cpp
rider::IntentConfig enabled_intent() {
    auto config = bike::configuration::default_physics().seated_climb;
    config.enabled = true;
    return config;
}

rider::FrontLiftReactionConfig switched_off() {
    auto t = table();
    t.ease_enabled = false;
    t.lean_enabled = false;
    return t;
}

bool same_bits(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }

bool same_posture(const rider::RiderPosture &a, const rider::RiderPosture &b) {
    return same_bits(a.torso_lean_rad, b.torso_lean_rad) && same_bits(a.pelvis_pitch_rad, b.pelvis_pitch_rad) &&
           a.pelvis_offset_m.has_value() == b.pelvis_offset_m.has_value() && a.use_saddle == b.use_saddle;
}

// The varying inputs every intent test feeds: a pitch-up pulse with light
// hands after 0.2 s on a 6 % preview, crank and load-share signals that
// exercise the policy's queues and trim.
std::pair<rider::IntentSignals, rider::RiderSenses> inputs(std::int64_t step) {
    rider::IntentSignals s;
    s.pitch_rate_up_rad_s = step > 160 && step < 400 ? 1. : 0.;
    s.specific_force_body_mps2 = {.3, 0., 9.81};
    s.crank_rate_rad_s = 6.;
    s.human_crank_torque_nm = 20.;
    s.front_load_share = .35;
    rider::RiderSenses senses;
    senses.hand_press_n = step > 160 && step < 400 ? 5. : 60.;
    senses.grade_ahead = .06;
    return {s, senses};
}

// Oracle (review M3): with both actions off the resolver output is the
// bare ordinary policy's, bit for bit, at every tick — effort and every
// posture field — and probes between ticks change no resolver state.
void test_switched_off_resolver_equals_the_bare_policy() {
    const auto config = enabled_intent();
    rider::RiderIntent intent(config, switched_off(), kPeriod);
    rider::SeatedClimbPolicy policy(config);
    for (std::int64_t step = 0; step < 600; ++step) {
        const auto [signals, senses] = inputs(step);
        const std::string before = bike::io::canonical_json(rider::encode_intent_state(intent.state()));
        (void)intent.resolve_full(rider::IntentControl{}, signals, senses, step, .06, .06, .35, true, false);
        bt::require(bike::io::canonical_json(rider::encode_intent_state(intent.state())) == before,
                    "a probe changes no resolver state");
        const auto out = intent.resolve_full(rider::IntentControl{}, signals, senses, step, .06, .06, .35);
        const auto bare = policy.update(signals, config.period_s, .06, .06, .35);
        bt::require(out.control.human_torque_nm.has_value() &&
                        same_bits(*out.control.human_torque_nm, bare.effort_ceiling_nm) &&
                        out.control.posture.has_value() && same_posture(*out.control.posture, bare.posture) &&
                        !out.effort_limited,
                    "switched-off actions leave the resolver output bitwise the policy's");
    }
}

// With both actions on, the resolver eases the effort exactly at
// reaction_delay + dwell + decision ticks after the first lifting sample,
// shifts the lean target forward through the posture program (whose own
// rate limits the total lean motion), reports effort_limited from the very
// copy that produced the output (probe and advance agree), and continues
// bitwise after encode → decode → restore mid-decision and mid-release.
void test_reacting_resolver_timing_slew_probe_and_restore() {
    const auto config = enabled_intent();
    rider::RiderIntent reacting(config, table(), kPeriod);
    rider::RiderIntent inert(config, switched_off(), kPeriod);
    const std::int64_t delay = std::llround(config.reaction_delay_s / config.period_s);
    const std::int64_t onset = 161 + delay + 4 + 8;   // first lifting sample 161, dwell 4, decision 8
    double previous_lean = 0.;
    std::optional<rider::RiderIntent> restored;
    for (std::int64_t step = 0; step < 900; ++step) {
        const auto [signals, senses] = inputs(step);
        const auto probe = reacting.resolve_full(rider::IntentControl{}, signals, senses, step, .06, .06, .35, true, false);
        const auto a = reacting.resolve_full(rider::IntentControl{}, signals, senses, step, .06, .06, .35);
        const auto b = inert.resolve_full(rider::IntentControl{}, signals, senses, step, .06, .06, .35);
        bt::require(probe.effort_limited == a.effort_limited &&
                        same_bits(*probe.control.human_torque_nm, *a.control.human_torque_nm),
                    "the probe previews the advancing output and its cut order");
        if (step < onset)
            bt::require(same_bits(*a.control.human_torque_nm, *b.control.human_torque_nm) && !a.effort_limited,
                        "no ease before delay + dwell + decision");
        if (step == onset)
            bt::require(*a.control.human_torque_nm < *b.control.human_torque_nm && a.effort_limited,
                        "the ease starts on the computed tick");
        const double lean = a.control.posture->torso_lean_rad;
        bt::require(std::abs(lean - previous_lean) <= config.lean_rate_rad_s * config.period_s + 1e-15,
                    "the total lean never outruns the posture rate");
        previous_lean = lean;
        if (step == onset - 3 || step == 700) {
            restored.emplace(config, table(), kPeriod);
            restored->restore(rider::decode_intent_state(rider::encode_intent_state(reacting.state()), "intent"));
        } else if (restored) {
            const auto r = restored->resolve_full(rider::IntentControl{}, signals, senses, step, .06, .06, .35);
            bt::require(same_bits(*r.control.human_torque_nm, *a.control.human_torque_nm) &&
                            same_posture(*r.control.posture, *a.control.posture) &&
                            r.effort_limited == a.effort_limited,
                        "the restored resolver continues bitwise");
        }
    }
    bt::require(reacting.reaction().phase == rider::ReactionPhase::idle, "the reaction ended after the pulse");
}

// A bad sense rejects before anything advances, and a bad restore changes
// nothing — policy, perception and reaction stay as they were.
void test_resolver_failures_are_atomic() {
    const auto config = enabled_intent();
    rider::RiderIntent intent(config, table(), kPeriod);
    for (std::int64_t step = 0; step < 10; ++step) {
        const auto [signals, senses] = inputs(step);
        (void)intent.resolve_full(rider::IntentControl{}, signals, senses, step, .06, .06, .35);
    }
    const std::string before = bike::io::canonical_json(rider::encode_intent_state(intent.state()));
    auto [signals, senses] = inputs(10);
    senses.hand_press_n = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { (void)intent.resolve_full(rider::IntentControl{}, signals, senses, 10, .06, .06, .35); },
            "hand_press_n", "a NaN hand press");
    bt::require(bike::io::canonical_json(rider::encode_intent_state(intent.state())) == before,
                "a rejected sense advances nothing");
    auto bad = intent.state();
    bad.reaction->effort_factor = .5;   // the decoded state of another configuration
    bad.policy.time_s += 1.;            // a change that must not leak if the restore fails
    rejects([&] { intent.restore(bad); }, "effort_factor", "an unreachable reaction state");
    bt::require(bike::io::canonical_json(rider::encode_intent_state(intent.state())) == before,
                "a rejected restore changes nothing");
    auto absent = intent.state();
    absent.perception.reset();
    rejects([&] { intent.restore(absent); }, "perception", "an enabled intent without perception state");
}
```

  Все три вызова добавить в `main` файла `test_front_lift_reaction.cpp`. Если сообщение отказа для NaN называет
  поле иначе (проверка `Perception::validate`), взять его подстроку и записать в отчёт.

  В `test_rider_senses.cpp`: тест канала, вызов в `main`. `launch_ride::environment` получает перегрузку с
  прореживанием записи (`launch_ride_support.hpp`: параметр `int record_decimation`; старая сигнатура вызывает новую
  с 1):

```cpp
// The front-lift reaction is published on every interval, measured or
// not: with record decimation 8 every recorded interval still carries the
// rider_reaction channel, inert with the shipped switched-off table.
[[gnu::noinline]] void test_reaction_channel_is_on_every_interval() {
    auto env = launch_ride::environment(launch_ride::track("[[0.0, 0.0], [60.0, 0.0]]", 60.), 1., 8);
    runtime::RideControl control;
    control.human_torque_nm = 10.;
    std::int64_t last_id = -1;
    std::size_t rows = 0;
    for (std::size_t w = 0; w < 20; ++w) {
        const auto result = env.runtime->advance(
            (static_cast<std::int64_t>(w) + 1) * env.control_steps, control, 0., 0., std::nullopt);
        if (result.outcome.has_value()) break;
        for (const auto &sample : env.runtime->recorded_intervals()) {
            if (sample->interval_id <= last_id) continue;
            last_id = sample->interval_id;
            const auto *wire = sw::find(sample->channels, "rider_reaction");
            bt::require(wire != nullptr, "every interval publishes the rider_reaction channel");
            const auto &reaction = sw::object(*wire);
            const auto &phase = sw::string(sw::required(reaction, "phase"));
            bt::require(phase == "idle" || phase == "deciding" || phase == "acting" || phase == "releasing",
                        "the phase is one of the four names");
            (void)sw::boolean(sw::required(reaction, "predicate"));
            bt::require(sw::number(sw::required(reaction, "effort_factor")) == 1. &&
                            sw::number(sw::required(reaction, "lean_add_rad")) == 0.,
                        "switched-off actions never move the outputs");
            ++rows;
        }
    }
    bt::require(rows > 8, "the ride recorded more intervals than one decimation window");
}
```

- [ ] **Шаг 2.** `mise run test 'front_lift_reaction|rider_senses'` — сборка падает.
- [ ] **Шаг 3. Код.**
  - `intent_signals.hpp`: структура `IntentSignals` из `intent.hpp:58-66` без изменений (с комментарием над ней),
    `#include <array>`, `<optional>`; `intent.hpp` и `perception.hpp` включают её. `intent.hpp` дополнительно
    включает `perception.hpp` и `front_lift_reaction.hpp`.
  - `Perception`:
    - `validate(sample)` — это нынешняя проверка `check_sample(sample, "perception sample")`, ставшая открытой;
    - `percepts()` строит `Percepts` из `s_.delayed` и `s_.pitch_rad` той же формулой, что `update`;
    - `update` после изменения состояния возвращает `percepts()`.
  - Программа позы: параметр `extra_target_rad`; в `update` после суммы импульсов (`intent.cpp:175-178`)
    `if (extra_target_rad != 0.) extra += extra_target_rad;` с комментарием «launch part 3a: the reaction's
    forward lean joins the target; zero leaves the arithmetic unchanged». Политика: параметр `extra_lean_rad`
    передаётся в `program_.update(..., min_lean_rad, extra_lean_rad)` (`intent.cpp:350-352`).
  - `RiderIntent`:
    - члены `std::optional<Perception> perception_; std::optional<FrontLiftReaction> reaction_;`;
    - конструктор при `config_.enabled` вызывает `validate_reaction_context(reaction, config_.period_s,
      config_.max_forward_lean_rad)` и строит оба, иначе оставляет пустыми;
    - `reset()` сбрасывает оба;
    - `reaction()` — `reaction_ ? reaction_->command() : ReactionCommand{}`.
  - `resolve_full` — нынешнее тело `resolve` (`intent.cpp:431-484`) с тремя изменениями:

```cpp
    if (!advance) {
        RiderIntent probe(*this);
        return probe.resolve_full(control, signals, senses, step, road_grade, preview_grade,
                                  lean_limit_rad, true, true, min_lean_rad);
    }
    if (step % period_steps_ == 0 && step != last_tick_step_) {
        // ... the existing on_schedule check, unchanged ...
        // Launch part 3a: one acquisition per intention tick. The new
        // sample is checked before anything advances; then the delayed
        // perception and the front-lift reaction move, and the policy
        // updates with the reaction's lean add in its posture target.
        const SenseSample sample{.signals = signals, .senses = senses};
        if (perception_) Perception::validate(sample);
        double lean_add = 0.;
        if (perception_ && reaction_)
            lean_add = reaction_->update(perception_->update(sample)).lean_add_rad;
        intent_ = policy_.update(signals, config_.period_s, road_grade, preview_grade, lean_limit_rad,
                                 min_lean_rad, lean_add);
        last_tick_step_ = step;
    }
    if (!control.rider_enabled)
        return {.control = control, .effort_limited = false};
    if (!control.posture.has_value())
        control.posture = intent_.posture;
    bool limited = false;
    if (!control.human_torque_nm.has_value()) {
        control.human_torque_nm = intent_.effort_ceiling_nm;
        // The rider eases the ceiling while reacting; a factor of one is
        // not applied at all (bitwise unchanged).
        const double factor = reaction().effort_factor;
        if (factor != 1.) {
            control.human_torque_nm = intent_.effort_ceiling_nm * factor;
            limited = true;
        }
    }
    return {.control = control, .effort_limited = limited};
```

    Остальное тело (проверки `step`, `on_schedule`, ранний выход `!config_.enabled || !active`) не меняется.
    Ранний выход возвращает `{.control = control, .effort_limited = false}`. `resolve(...)` вызывает
    `resolve_full(...)` и возвращает `.control`.
  - `state()` заполняет `perception`/`reaction` из членов. `restore()` собирает кандидата:

```cpp
void RiderIntent::restore(const State &state) {
    if (state.perception.has_value() != perception_.has_value())
        throw std::invalid_argument("rider intent perception state presence must match the enabled intent");
    if (state.reaction.has_value() != reaction_.has_value())
        throw std::invalid_argument("rider intent reaction state presence must match the enabled intent");
    RiderIntent next(*this);   // staged: every component restores into the copy first
    next.policy_.restore(state.policy);
    next.policy_.program().restore(state.program);
    if (next.perception_) next.perception_->restore(*state.perception);
    if (next.reaction_) next.reaction_->restore(*state.reaction);
    next.last_tick_step_ = state.last_tick_step;
    next.intent_ = state.intent;
    *this = std::move(next);
}
```

    Если у класса нет копирующего присваивания (`commands_` и прочие члены копируются), оно объявляется
    `= default`; пробная копия `RiderIntent probe(*this)` уже требует копирования.
  - Кодек состояния (`state_codec.cpp:2278-2400`): ключи `perception` и `reaction` с `encode_perception_state` /
    `encode_reaction_state` или `null`; декодирование — в `std::optional`; точный набор ключей
    `last_tick_step, intent, policy, program, perception, reaction`.
  - Документ запуска:
    - `project_config.cpp` после `rider_intent`: `{"rider_reaction", rider::encode_reaction_config(config.front_lift_reaction)}`;
    - `config.hpp`: `bike::io::Value rider_reaction;` рядом с `rider_intent`;
    - `bootstrap.cpp:249-255`: `"rider_reaction"` в списке ключей **конфигурации**, и
      `out.rider_reaction = owned_mapping(root, "rider_reaction");` рядом с `:301`. Список ключей **состояния**
      (`:358-363`) не меняется;
    - `step.hpp`: `rider::FrontLiftReactionConfig reaction{};` в `StepConfig`;
    - `runtime.cpp:350-354`: `out.reaction = rider::decode_reaction_config(config.rider_reaction, "config.rider_reaction");`;
    - `step.cpp:1024`: `rider_intent_(config_.intent, config_.reaction, config_.timestep_s)`.
  - Шаг (`step.cpp:1264-1269`):

```cpp
            const auto resolution = rider_intent_.resolve_full(
                control, intent_signals_, rider_senses_, step_,
                road_grade_for_posture(road), road_grade_preview(road),
                rider_control_.has_value() ? rider_control_->lean_limit_rad()
                                           : std::optional<double>{},
                active, advance, min_lean_rad);
            control = resolution.control;
            effort_limited = resolution.effort_limited;
```

    `bool effort_limited = false;` объявляется перед `if (config_.intent.enabled)`. Последний аргумент
    `prepare(...)` — `effort_limited` вместо `false` из задачи 1.
  - Канал (`step.cpp:2031-2049`), после блока `rider_senses`:

```cpp
        // Launch part 3a — the front-lift reaction on every interval
        // (measured or not), so its onsets are never hidden by decimation.
        if (config_.intent.enabled) {
            const rider::ReactionCommand reaction = rider_intent_.reaction();
            WireObject fields;
            fields.emplace_back("phase", Wire(std::string(rider::reaction_phase_name(reaction.phase))));
            fields.emplace_back("predicate", Wire(reaction.predicate));
            fields.emplace_back("effort_factor", Wire(reaction.effort_factor));
            fields.emplace_back("lean_add_rad", Wire(reaction.lean_add_rad));
            channels.emplace_back("rider_reaction", Wire(std::move(fields)));
        } else {
            channels.emplace_back("rider_reaction", Wire(nullptr));
        }
```

  - Каталог: канал `rider_reaction` — структура из четырёх полей (строка, логическое, два вещественных),
    описатели по образцу `rider_senses` в обоих файлах каталога (`git show c5ceece3 -- native/resources/recording/catalog-v1.json`).
    `test_catalog.cpp:72`: 511, комментарий «511 = 506 plus channel_rider_reaction (struct + phase, predicate,
    effort_factor, lean_add_rad)». `tests/fixtures/recording/catalog_welded.json` перезаписывается по правилу.
  - Вызовы из описи: `resolve(...)` получает `senses` третьим аргументом (в тестах — `rider::RiderSenses{}` или
    смысловое значение теста); конструкторы — выключенная реакция с числами описи (в тестах — константа файла).
- [ ] **Шаг 4.** `mise run test 'front_lift_reaction|rider_senses|rider_perception|launch_preparation|seated_posture_program|seated_pedaling_cycle|realistic_pedelec_acceptance|io_state_codecs|recording_catalog|pedaling_cut_order'`
  — новые зелёные, прежние как в базе.
- [ ] **Шаг 5. Перезаписи** документов описи — по правилу, с выводом `check_rerecord.py --ref <коммит задачи 2>`;
  выходы красных тестов — не трогать, перечислить.
- [ ] **Шаг 6. Коммит и проверка.** `feat(rider): perception and the front-lift reaction in the intent`;
  `check_task.sh`, база — коммит задачи 2, с `ab`.

### Задача 4. Итоговая проверка части 3a (оркестратор)

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
F=/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/final; T=/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2; mkdir -p $F
R=/Volumes/SSD_1TB_M2/work/tmp/2026-11-10-delays-t5b-check/reds.txt; S=/Volumes/SSD_1TB_M2/work/tmp/2026-11-09-reds-final/slow_reds.txt
t() { perl -e "alarm $1; exec @ARGV" -- "${@:2}"; }
git log --oneline -1 > $F/head.txt
t 3600 cmake --build build/native-release -j 12 > $F/build.log 2>&1; echo "build $?" | tee $F/stages.txt
t 3600 bash tools/run_native_checks.sh build/native-release > $F/checks.log 2>&1; echo "checks $?" | tee -a $F/stages.txt
t 1800 ctest --test-dir build/native-release -LE 'slow|package' -j 12 --output-on-failure > $F/ordinary.log 2>&1; echo "ordinary $?" | tee -a $F/stages.txt
t 2400 ctest --test-dir build/native-release -L slow --no-tests=error -j 8 --output-on-failure > $F/slow.log 2>&1; echo "slow $?" | tee -a $F/stages.txt
t 1200 ctest --test-dir build/native-release -L package --no-tests=error > $F/package.log 2>&1; echo "package $?" | tee -a $F/stages.txt
cat $F/ordinary.log $F/slow.log > $F/reds_output.log
python3 $T/causes.py $T/final/reds_output.log $F/reds_output.log $R $S > $F/cause_compare.txt; echo "causes $?" | tee -a $F/stages.txt
bash $T/ab_settle.sh /Volumes/SSD_1TB_M2/work/wt/lane-flip "$PWD" $F/ab > $F/ab.txt 2>&1; echo "ab $?" | tee -a $F/stages.txt
```

Ожидание:
- `build 0`, `checks 0` (у каждого вида `tool health: ok`, `findings: none`);
- `ordinary 8` (22 известных красных), `slow 8` (10 известных);
- `package 0`, `causes 0` относительно итога части 2, `ab 0`.

Отчёт называет доказательство по частям, как в «Общих ограничениях». Затем строка состояния в дорожной карте,
запись в журнал (включая долг `native_rider_intent`) и коммит `docs(plan): launch part 3a is in the code`.

## Ответы на ревью

**Ревью всей работы (`2026-11-11-plan-review`), доля части 3a:**

| Пункт | Где |
|---|---|
| I4 — порядок «предел → срез» | Решения 6–7, задача 1; признак из той же копии намерений (задача 3) |
| I12 — восстановление по частям | Задача 2: кодек реакции, продолжение бит в бит, атомарный отказ. Задача 3: намерения целиком (восприятие + реакция + политика) — кодек → восстановление → продолжение посреди решения и посреди возврата, атомарный отказ |
| I13 — проверки таблицы реакции | Задача 2: домены, сетка тиков с верхней границей, граница наклона, запрет заднего тормоза. Те же проверки у документа запуска (конструктор `RiderIntent`) |
| I14 — события реакции | Решение 8: канал на каждом интервале, проверка при прореживании 8 |

**Ревью этого плана (`2026-11-11-plan3a-review`):**

| Пункт | Ответ |
|---|---|
| C1 — ключ не в том списке, значение не копировалось | Решение 10, задача 3: ключ только в списке конфигурации, копирование в `RuntimeConfig`, состояние — только внутри `rider_intent` |
| I1 — цикл заголовков | `intent_signals.hpp`; `perception.hpp` больше не включает `intent.hpp` |
| I2 — тесты не исполнимы | Общий помощник `drive_reference::pedaling_config`; сравнение через `canonical_json` и биты; полная опись вызовов `resolve`/конструктора, включая `emplace` и пробную рекурсию; регистрации тестов; новые тесты в `main` |
| I3 — восстановление в обход выключателей | `restore` проверяет домен фазы, согласованность фазы, счётчиков и признака и выходы относительно выключателей; тесты отказа |
| I4 — переполнение и обход проверок документом запуска | Тики ≤ 2^31; счётчик разгрузки насыщается; `validate_reaction_context` вызывают и `post_init`, и конструктор `RiderIntent` |
| I5 — частичное продвижение и восстановление | Выборка проверяется до любого изменения; восстановление через кандидата; тест атомарности и тест продолжения намерений целиком |
| I6 — пробный шаг и флаг | `resolve_full` возвращает флаг вместе с выходом той же копии; тест «пробный = настоящий» на начале и конце действия |
| I7 — сумма двух скоростей наклона | Добавка входит в цель программы позы, общий наклон подчиняется её скорости и пределам; тест ограничения скорости |
| I8 — канал пропадал при прореживании | Канал в каналах каждого интервала (как `rider_senses`); тест с прореживанием 8 |
| I9 — опись | Таблица документов и читателей; накопители `raw.diagnostics` не трогаются (канал не в подробных); каталог 511; двоичные входы `native_rider_intent` — долг |
| I10 — заглушки | Кодеки, схема, тесты профиля, восстановление, документ запуска, канал и команды задачи 4 приведены полностью |
| M1 — указатели | Обновлены (`intent.cpp:406-424`, `initial_state.cpp:80`, `equilibrium_cache.cpp:168`, `step.cpp:1264,1328`, `config_types.hpp`, `config.hpp:57-87`) |
| M2 — время выборки | Решение 4: лишний физический шаг возраста выборки записан, вторая задержка не вводится |
| M3 — доказательство уже, чем «бит в бит» | Доказательство по частям; тест-оракул против голой политики по всем полям с пробами |

## Что остаётся части 3b

План старта (`rider/launch_plan.{hpp,cpp}`), поля таблицы удержания, отпускания тормоза, предела нажима,
переключения и конца старта, встраивание в `RiderIntent` с выключателем, протокол просьбы и конца среза, передача
обычной езде с посевом политики (добавка реакции в цели программы сохраняется), события плана в записи, подбор
чисел — C2, C3, I10, I11, I14 для плана.
