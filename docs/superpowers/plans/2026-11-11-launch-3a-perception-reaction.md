# План старта, часть 3a: ощущения в шаге и реакция на подъём колеса — план реализации

> **Для исполнителей-агентов:** обязательный навык — `superpowers:subagent-driven-development` (рекомендуется) или
> `superpowers:executing-plans`. Шаги отмечаются флажками (`- [ ]`). Исполнитель — Devin (`devin-orchestration`);
> финальное ревью ветки делает оркестратор.

**Цель.** Подготовить всё, на что опирается план старта (часть 3b), кроме самого плана. Поведение поездок не
меняется:
- райдер получает свои ощущения с задержкой реакции прямо в шаге: адаптер `rider::Perception` из части 1 живёт в
  `RiderIntent`;
- реакция на подъём переднего колеса — отдельный узел на всю поездку («Решено — не сделано», пункт 2). В
  профилях оба её действия выключены, а признак и фазы пишутся в запись;
- на педалях есть порядок «предел райдера → срез переключения» для случая, когда нажим ограничивает сам
  райдер. Обычный путь не меняется.

**Архитектура.** `RiderIntent::resolve` получает ощущения шага (`RiderSenses`). На каждом тике намерений
`Perception` выдаёт задержанное восприятие (`Percepts`), а `FrontLiftReaction` по нему решает, ослабить ли нажим и
наклониться ли вперёд. Реакция меняет только выход `resolve` (предел нажима и наклон), а не состояние обычной
политики. Пока оба действия выключены, выход `resolve` бит в бит прежний. Состояния восприятия и реакции входят в
состояние намерений (`state.rider_intent`).

**Стек.** C++23, MuJoCo, CTest (`bike_add_reference_test`), таблицы полей `FieldDecl`, строгие кодеки, каталог
записи `native/src/recording/catalog_data.inc`.

**Спецификация.** `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md`, разделы «Ощущения райдера»,
«Реакция на подъём переднего колеса», «Таблица опыта» (строка реакции), «Кто чем управляет». Ревью всей работы —
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan-review/review.md`: пункты I4 (порядок на педалях), I12
(восстановление по частям для восприятия и реакции), I13 (проверки таблицы реакции), I14 (события реакции в
записи). Пункты C2, C3, I10, I11 и события плана — в части 3b. Части 1 и 2 — `2026-11-11-launch-1-channels.md`,
`2026-11-11-launch-2-preparation.md`.

## Решения плана

1. **Решение пользователя 2026-11-11.** Мягкий старт уходит в части 4, в той же задаче, где включается план
   старта. Части 3a и 3b строят узлы за выключателями, которые в профилях выключены.
2. **Реакция — раздел профиля `[front_lift_reaction]`** (тип `rider::FrontLiftReactionConfig`, все поля
   обязательны, значений по умолчанию в коде нет). Поля — по таблице опыта спецификации:
   - порог разгрузки рук `hand_unload_n` и время выдержки `unload_dwell_s`;
   - порог скорости подъёма `pitch_rate_up_rad_s` и порог накопленного наклона `pitch_rad`;
   - время решимости `decision_s`;
   - три выключателя `ease_enabled`, `lean_enabled`, `rear_brake_tap_enabled`; последний обязан быть `false`, пока
     действие не сделано (решение пользователя 4);
   - «ослабить»: `ease_fraction` (доля, на которую снижается предел нажима) и `ease_rate_per_s`;
   - «наклониться»: `lean_add_rad` и `lean_rate_rad_s`;
   - `clear_s` — сколько признак должен отсутствовать, чтобы реакция закончилась.
3. **Времена реакции считаются в тиках намерений.** `unload_dwell_s`, `decision_s` и `clear_s` — целые кратные
   `seated_climb.period_s` (правило сетки, как у задержки реакции, `intent.cpp:384-393`). Проверка в корневом
   `post_init`.
4. **Признак и фазы — на задержанном восприятии.** Восприятие уже отстаёт на `reaction_delay_s`. Признак:
   - руки разгружены (`hand_press_n < hand_unload_n`) дольше `unload_dwell_s`;
   - и при этом скорость подъёма выше порога или накопленный наклон выше порога.

   Фазы:
   - `idle` → `deciding` в тик, когда признак появился;
   - `deciding` → `acting` через `decision_s`, если признак держится всё это время; если пропал — назад в `idle`;
   - `acting` → `releasing`, когда признака нет дольше `clear_s`;
   - `releasing` → `acting`, если признак вернулся; `releasing` → `idle`, когда множитель нажима вернулся к 1, а
     добавка наклона к 0.

   Пока восприятия ещё нет (первые `reaction_delay_s`), реакция стоит в `idle`.
5. **Действия меняют только выход `resolve`.**
   - Предел нажима умножается на множитель реакции, если его не задал вызывающий.
   - К наклону позы прибавляется добавка реакции, ограниченная сверху `lean_limit_rad` (или
     `max_forward_lean_rad`), если позу не задал вызывающий.
   - Множитель 1 и добавка 0 не применяются вовсе. Поэтому при выключенных действиях выход бит в бит прежний, в
     том числе знак нуля.
6. **Порядок на педалях (I4).** Когда нажим ограничивает сам райдер (`RiderIntent::effort_limited()`: сейчас это
   множитель реакции < 1, в части 3b добавится план), цель строится как `min(цель с прибавкой mash, предел) ×
   множитель среза`, затем прежний темп. Обычный путь сохраняет нынешний порядок `(цель × срез)`, потом
   `min(предел)` (`pedaling.cpp:68-74`).
7. **Запись.** Для каждого интервала пишется диагностика `rider_reaction`: `phase`, `predicate`, `effort_factor`,
   `lean_add_rad`, по образцу `rider_intent` (`step.cpp:2131-2135`, `accounting.cpp:224-230`, каталог). Моменты
   появления признака и начала действия видны как смена `phase` и `predicate` от интервала к интервалу. Канал
   нужен будущему контроллеру тяги для замера окна.
8. **Ключи повторного использования.** Реакция и восприятие не меняют успокоенную позу. Поэтому
   `front_lift_reaction` убирается:
   - из контракта переноса (`initial_state.cpp:77`, рядом с `research`, `monitors`, `rider_launch`);
   - из обеих ветвей ключа кэша — так ключ не зависит от параметров езды, как уже сделано для полей мягкого
     старта (`equilibrium_cache.cpp:173`).

## Общие ограничения

Как в части 2 (`docs/superpowers/plans/2026-11-11-launch-2-preparation.md`, раздел «Общие ограничения»), с этими
уточнениями:
- Успокоенная поза, модель и начальное состояние бит в бит прежние (`ab_settle.sh`).
- Поездки без включённой реакции бит в бит прежние по построению: при выключенных действиях выход `resolve` тот
  же, а порядок на педалях тот же. Отчёт называет доказательство именно так.
- Перезапись эталонов — правило, которое пользователь подтвердил для части 2: только выходы зелёных тестов, только
  новые ключи, проверка `check_rerecord.py`; изменение прежнего значения (хэши включительно) — стоп. Выходы
  красных тестов не трогаются. Входные документы (`*.toml`, `state.json`, `config.json`) переводятся на новую
  форму везде.
- Известные красные — 22 обычных и 10 медленных. Причины сравниваются строго (`causes.py`) с базой
  `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/final/reds_output.log` (итог части 2).
- Без отладчиков, с ограничением времени у длинных команд, без `git stash`, без push, `.claude/` и `.agents/` не
  трогать. Последняя строка коммита ровно `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Нельзя переставлять или ослаблять проверки в красном тесте ради прежней причины падения (урок части 2). Новая
  причина допустима, только если её вызвала эта задача: она приводится и объясняется.

## Проверка задачи

После коммита задачи:

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
BASELOG=/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/baseline-before-task<N>.log \
  bash /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/check_task.sh <база задачи> \
  /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/verify-task<N> ab
```

Оркестратор кладёт `baseline-before-task<N>.log` перед запуском задачи: это `reds_output.log` проверки
предыдущей задачи; для задачи 1 — итог части 2. Конец вывода — `check_task exit 0`. Иначе каждое `STOP`
приводится и объясняется. Исправления — отдельным коммитом, затем проверка повторяется на итоговой вершине.

## Бюджет тестов

| Что | Время |
|---|---|
| целевые тесты задачи | 3–8 мин |
| `check_task.sh` с `ab` | около 32 мин |
| задача 4: итоговая проверка | около 25 мин |

Итого около 2 ч 10 мин машинного времени.

## Опись мест, которые меняются

- **Полные документы физики** получают `[front_lift_reaction]` — те же места, что получили `[rider_launch]` в
  части 2:
  - 16 файлов `*.toml` (`git grep -l '^\[monitors\]' -- '*.toml'`);
  - программные конфигурации из описи части 2: `default_physics()`, умолчания `bike-research`, три конфигурации
    стендов `validation/drivetrain.cpp`/`rider.cpp`, две в `validation/scenarios.cpp`, тестовые структуры и
    встроенные TOML-строки. Проверка — число `[front_lift_reaction]` равно числу `[monitors]` в каждом файле
    тестов, по образцу команды шага 4 задачи 1 части 2;
  - место `test_pneumatic_tire_config.cpp`, которое подставляет недостающие разделы (отклонение 2 задачи 1 части 2).
- **Значения в профилях** — предварительные, реакция выключена:
  - `ease_enabled = false`, `lean_enabled = false`, `rear_brake_tap_enabled = false`;
  - `hand_unload_n = 20.0`, `unload_dwell_s = 0.05`, `pitch_rate_up_rad_s = 0.5`, `pitch_rad = 0.17`;
  - `decision_s = 0.1`, `ease_fraction = 0.7`, `ease_rate_per_s = 5.0`;
  - `lean_add_rad = 0.1`, `lean_rate_rad_s = 0.5`, `clear_s = 0.3`.

  Числа подбирает часть 3b вместе с планом старта.
- **Документы запуска** (`config.json` в `tests/fixtures/bootstrap/**` и других захватах) получают ключ
  `rider_reaction`. **Состояния** (`state.json`) получают `rider_intent.perception` и `rider_intent.reaction`. Это
  новые ключи — перезапись по правилу; образец — `c5ceece3` (ключи `senses` и `surface_labels` части 1).
- **Конструкторы `RiderIntent`** — `step.cpp:1024` и тестовые: `test_launch_preparation.cpp:289`,
  `test_seated_posture_program.cpp:87-89,136…`, `test_native_rider_intent.cpp`, `test_state_codecs.cpp`.
  Найти все: `git grep -n 'RiderIntent(' -- native`.

---

### Задача 1. Порядок «предел райдера → срез» на педалях

**Файлы:**
- Изменить: `native/src/drivetrain/pedaling.hpp:83-86`, `native/src/drivetrain/pedaling.cpp:68-74`,
  `native/src/writers/drivetrain.hpp:119-134,302-306`, `native/src/writers/drivetrain.cpp:476-488,751-755`,
  `native/src/runtime/step.cpp:1331-1337` (передача признака; в этой задаче всегда `false`).
- Тест: `native/tests/reference/test_pedaling_cut_order.cpp` (создаётся; регистрация в `Phase3Ports.cmake` рядом
  с `launch_preparation`, `LABELS launch drive`, `LIBRARIES bike_app`).

**Интерфейсы:**
- Создаёт: последний параметр `bool cut_after_ceiling = false` у `PedalingPolicy::update`; поле
  `bool cut_after_ceiling = false` в `DrivetrainWriter::PrepareInputs`; последний параметр
  `bool cut_after_ceiling = false` у `DrivetrainWriter::prepare`. Значение по умолчанию сохраняет всех нынешних
  вызывающих; это параметр кода, а не профиля.
- Задача 3 передаёт `rider_intent_.effort_limited()`.

- [ ] **Шаг 1. Тест (падает):**

```cpp
// Launch part 3a — the shift cut after a rider-limited effort ceiling
// (review I4): the ordinary path keeps (target x cut) then min(ceiling);
// a rider-limited effort takes min(target, ceiling) first and cuts after,
// so a small ceiling cannot erase the cut.
#include "support/fixture_test.hpp"
#include "drivetrain/pedaling.hpp"

#include <optional>

namespace {

// A crank at rest with mash torque 60 N·m below 30 rpm and no slew: the
// low-cadence mash raises a 10 N·m effort toward 60 before the ceiling.
drivetrain::PedalingPolicy mash_policy() {
    drivetrain::PedalingConfig config;
    config.mash_torque_nm = 60.;
    config.mash_cadence_rpm = 30.;
    config.effort_slew_nm_s = 0.;
    return drivetrain::PedalingPolicy(config);
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

// Without a cut both orders agree: the flag changes nothing at factor 1.
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

  Имена `PedalingConfig`, его полей и конструктора `PedalingPolicy` взять из `native/src/drivetrain/pedaling.hpp`
  (поля `mash_torque_nm`, `mash_cadence_rpm`, `effort_slew_nm_s` читаются в `pedaling.cpp:70-75`). Если остальные
  поля конфигурации без значений по умолчанию не проходят проверку конструктора, заполнить их по образцу ближайшего
  теста педалирования (`git grep -ln 'PedalingPolicy(' -- native/tests`) и указать это в отчёте.
- [ ] **Шаг 2.** `mise run test 'pedaling_cut_order'` — сборка падает (нет параметра).
- [ ] **Шаг 3. Код.** `pedaling.cpp:68-74` заменить на

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

  Параметр протянуть: `PedalingPolicy::update(..., double torque_factor = 1., bool cut_after_ceiling = false)`;
  `PrepareInputs::cut_after_ceiling`; `prepare(..., RiderShift rider_shift = {}, bool cut_after_ceiling = false)` →
  `.cut_after_ceiling = cut_after_ceiling` в `stage_prepare`; вызов `pedaling.update(..., in.ceiling,
  torque_factor, in.cut_after_ceiling)`. В `step.cpp` вызов `stepper_->drive().prepare(...)` получает последним
  аргументом `false` с комментарием «launch part 3a: task 3 passes rider_intent_.effort_limited()».
- [ ] **Шаг 4.** `mise run test 'pedaling_cut_order|native_drivetrain|native_drive_policies|seated_pedaling_cycle|realistic_pedelec_acceptance'`
  — новые зелёные, прежние как в базе.
- [ ] **Шаг 5. Коммит и проверка.** `feat(drivetrain): cut a rider-limited effort after its ceiling`; затем
  `check_task.sh` по разделу «Проверка задачи», база `dc685386`, с `ab`.

### Задача 2. Таблица реакции и узел реакции

**Файлы:**
- Создать: `native/src/rider/front_lift_reaction.hpp`, `native/src/rider/front_lift_reaction.cpp`,
  `native/src/rider/reaction_codec.hpp`, `native/src/rider/reaction_codec.cpp`,
  `native/tests/reference/test_front_lift_reaction.cpp`
- Изменить:
  - `CMakeLists.txt:426-438`: `front_lift_reaction.cpp` в `bike_configuration`, рядом с `launch_config.cpp`;
    `reaction_codec.cpp` в список `bike_core`, рядом с `senses_codec.cpp` (`:142`);
  - `native/src/configuration/model_config.hpp` (псевдоним, член `front_lift_reaction` после `rider_launch`, таблица
    полей, корневой `FieldDecl`, объявление `post_init`);
  - `native/src/configuration/load.cpp` (`BIKE_T13_RECORD`);
  - `native/src/configuration/validate.cpp` (`detail::post_init`, проверки между записями, `default_physics()`);
  - `native/src/initialization/initial_state.cpp:77`, `native/src/initialization/equilibrium_cache.cpp:172-173`;
  - все полные документы физики из описи;
  - `native/tests/reference/Phase3Ports.cmake`.
- Тест: `test_front_lift_reaction.cpp`, `test_welded_profile_config.cpp`

**Интерфейсы** (`native/src/rider/front_lift_reaction.hpp`):

```cpp
// rider/front_lift_reaction.hpp — the rider's reaction to a rising front
// wheel (launch plan, part 3a; «Решено — не сделано», item 2). One reaction
// for the whole ride: on the delayed percepts, unloaded hands that last
// longer than a dwell, together with a fast pitch-up or a held pitch, make
// the predicate; after a decision time the rider eases the effort ceiling
// and/or leans forward, each behind its own switch, and recovers when the
// predicate has been absent for clear_s. Times count intention ticks.
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
// front_lift_reaction field. The period grid and the posture bound are
// checked in configuration post_init, where those records are in scope.
void validate_reaction_table(const FrontLiftReactionConfig &table);

enum class ReactionPhase : std::uint8_t { idle, deciding, acting, releasing };

struct ReactionCommand {
    ReactionPhase phase = ReactionPhase::idle;
    bool predicate = false;
    double effort_factor = 1.;   // multiplies the effort ceiling
    double lean_add_rad = 0.;    // added to the torso lean
};

class FrontLiftReaction {
public:
    struct State {
        ReactionPhase phase = ReactionPhase::idle;
        bool predicate = false;
        std::int64_t unload_ticks = 0;
        std::int64_t decision_ticks = 0;
        std::int64_t clear_ticks = 0;
        double effort_factor = 1.;
        double lean_add_rad = 0.;
    };
    // period_s is the intention period: the three times must be whole
    // multiples of it (the same rule post_init enforces on the profile).
    FrontLiftReaction(const FrontLiftReactionConfig &config, double period_s);
    void reset();
    // One intention tick. Percepts are absent until the reaction delay
    // has passed: the reaction then stays idle.
    ReactionCommand update(const std::optional<Percepts> &percepts);
    [[nodiscard]] ReactionCommand command() const;
    [[nodiscard]] State state() const;
    // Validates the whole state (phase/counter/value domains), then
    // commits; a rejected state leaves the reaction unchanged.
    void restore(const State &state);

private:
    FrontLiftReactionConfig config_;
    double period_s_;
    std::int64_t dwell_ticks_, decision_ticks_, clear_ticks_;
    State s_;
};

[[nodiscard]] const char *reaction_phase_name(ReactionPhase phase) noexcept;

} // namespace rider
```

`reaction_codec.hpp`:

```cpp
// The reaction's restorable document: {"phase", "predicate",
// "unload_ticks", "decision_ticks", "clear_ticks", "effort_factor",
// "lean_add_rad"} — exact keys, phase one of idle/deciding/acting/releasing,
// nonnegative int64 counters, finite reals. Restore-time invariants belong
// to FrontLiftReaction::restore.
[[nodiscard]] bike::io::Value encode_reaction_state(const FrontLiftReaction::State &state);
[[nodiscard]] FrontLiftReaction::State decode_reaction_state(const bike::io::Value &v, std::string_view path);
// The runtime config document of the reaction: the [front_lift_reaction]
// keys, exact, validated by validate_reaction_table.
[[nodiscard]] bike::io::Value encode_reaction_config(const FrontLiftReactionConfig &config);
[[nodiscard]] FrontLiftReactionConfig decode_reaction_config(const bike::io::Value &v, std::string_view path);
```

- Задача 3 строит `FrontLiftReaction` внутри `RiderIntent`, кладёт `State` в состояние намерений, а
  конфигурацию — в документ запуска `rider_reaction`.

- [ ] **Шаг 1. Тесты (падают).** `test_front_lift_reaction.cpp`:

```cpp
// Launch part 3a — the front-lift reaction node: its timing on the delayed
// percepts, the two independent actions, recovery, the codec round trip
// with bitwise continuation, atomic restore and the table rules.
#include "support/fixture_test.hpp"
#include "rider/front_lift_reaction.hpp"
#include "rider/reaction_codec.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
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

// Unloaded hands must last longer than the dwell (4 ticks) before the
// predicate rises; the action starts after the decision time (8 ticks);
// the first tick of acting already moves both outputs one rate step.
void test_predicate_dwell_and_decision_timing() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    const auto lifting = percept(5., 1., 0.);
    std::vector<rider::ReactionCommand> out;
    for (int tick = 0; tick < 20; ++tick) out.push_back(reaction.update(lifting));
    for (int tick = 0; tick < 4; ++tick)
        bt::require(!out[static_cast<std::size_t>(tick)].predicate, "no predicate within the dwell");
    bt::require(out[4].predicate && out[4].phase == rider::ReactionPhase::deciding,
                "the predicate rises when the unload outlasts the dwell");
    for (int tick = 4; tick < 12; ++tick)
        bt::require(out[static_cast<std::size_t>(tick)].phase == rider::ReactionPhase::deciding &&
                        out[static_cast<std::size_t>(tick)].effort_factor == 1.,
                    "the rider is still deciding");
    bt::require(out[12].phase == rider::ReactionPhase::acting, "the action starts after the decision time");
    bt::require(std::abs(out[12].effort_factor - (1. - 5. * kPeriod)) < 1e-15 &&
                    std::abs(out[12].lean_add_rad - .5 * kPeriod) < 1e-15,
                "acting moves both outputs one rate step");
}

// A held-high wheel triggers through the accumulated pitch even when the
// pitch rate is already small; ordinary light hands on acceleration (no
// pitch) never trigger; no percepts keep the reaction idle.
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

// Each action has its own switch: ease only lowers the factor, lean only
// adds lean; with both off the phases still run and both outputs stay put.
void test_each_action_has_its_own_switch() {
    for (const auto [ease, lean] : {std::pair{true, false}, std::pair{false, true}, std::pair{false, false}}) {
        auto t = table();
        t.ease_enabled = ease;
        t.lean_enabled = lean;
        rider::FrontLiftReaction reaction(t, kPeriod);
        rider::ReactionCommand last;
        for (int tick = 0; tick < 200; ++tick) last = reaction.update(percept(5., 1., 0.));
        bt::require(last.phase == rider::ReactionPhase::acting, "acting with any switch setting");
        bt::require((last.effort_factor < 1.) == ease, "ease follows its switch");
        bt::require((last.lean_add_rad > 0.) == lean, "lean follows its switch");
        if (ease) bt::require(std::abs(last.effort_factor - .3) < 1e-12, "the ease settles at 1 - ease_fraction");
        if (lean) bt::require(last.lean_add_rad == .1, "the lean settles at lean_add_rad");
    }
}

// The reaction ends when the predicate has been absent longer than clear_s,
// then both outputs return at their rates and the phase becomes idle.
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

// Serialize → read → continue: a reaction restored mid-decision and
// mid-release continues bitwise like the original; malformed documents and
// unreachable states are rejected without changing the target.
void test_codec_round_trip_and_continuation() {
    for (const int split : {7, 230}) {
        rider::FrontLiftReaction original(table(), kPeriod);
        for (int tick = 0; tick < split; ++tick)
            (void)original.update(tick < 200 ? percept(5., 1., 0.) : percept(80., 0., 0.));
        const auto doc = rider::encode_reaction_state(original.state());
        rider::FrontLiftReaction copy(table(), kPeriod);
        copy.restore(rider::decode_reaction_state(doc, "reaction"));
        for (int tick = split; tick < split + 300; ++tick) {
            const auto p = tick < 200 ? percept(5., 1., 0.) : percept(80., 0., 0.);
            const auto a = original.update(p);
            const auto b = copy.update(p);
            bt::require(a.phase == b.phase && a.predicate == b.predicate &&
                            a.effort_factor == b.effort_factor && a.lean_add_rad == b.lean_add_rad,
                        "the restored reaction continues bitwise");
        }
    }
    rider::FrontLiftReaction target(table(), kPeriod);
    (void)target.update(percept(5., 1., 0.));
    const auto before = rider::encode_reaction_state(target.state());
    auto bad = target.state();
    bad.effort_factor = 1.5;
    rejects([&] { target.restore(bad); }, "effort_factor", "a factor above one");
    bad = target.state();
    bad.decision_ticks = -1;
    rejects([&] { target.restore(bad); }, "decision_ticks", "a negative counter");
    bt::require(rider::encode_reaction_state(target.state()) == before, "a rejected restore changes nothing");
    rejects([&] { (void)rider::decode_reaction_state(bike::io::Value{bike::io::Value::Object{}}, "reaction"); },
            "reaction", "an empty document");
}

// The table's own rules, each with the valid control passing.
void test_table_rules() {
    rider::validate_reaction_table(table());
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
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.pitch_rad = std::numeric_limits<double>::quiet_NaN(); })); },
            "pitch_rad", "a NaN pitch threshold");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.clear_s = -kPeriod; })); },
            "clear_s", "a negative clear time");
    rejects([&] { (void)rider::FrontLiftReaction(with([](auto &t) { t.decision_s = 1.5 * kPeriod; }), kPeriod); },
            "decision_s", "a decision time off the tick grid");
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
        test_table_rules();
    });
}
```

  Сравнение документов `==` — если у `bike::io::Value` нет `operator==`, сравнивать
  `bike::io::canonical_json(...)` (`native/src/io/json.hpp:53`).

  В `test_welded_profile_config.cpp` — по образцу тестов части 2:
  - профиль несёт выключенную реакцию с числами из описи;
  - каждый из 13 ключей и сам раздел обязательны (удаление строки → отказ с именем ключа);
  - `cfg::validate` отклоняет `unload_dwell_s`, `decision_s` или `clear_s` не на сетке `seated_climb.period_s` и
    `lean_add_rad` больше `seated_climb.max_forward_lean_rad`; правильный контроль проходит.
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

std::int64_t ticks(double seconds, double period_s, std::string_view field) {
    const double n = std::rint(seconds / period_s);
    if (n < 0. || std::abs(n * period_s - seconds) > 1e-9)
        reject(field, "must be a whole multiple of the intention period");
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

FrontLiftReaction::FrontLiftReaction(const FrontLiftReactionConfig &config, double period_s)
    : config_(config), period_s_(period_s), dwell_ticks_(0), decision_ticks_(0), clear_ticks_(0) {
    validate_reaction_table(config_);
    if (!std::isfinite(period_s_) || period_s_ <= 0.)
        throw std::invalid_argument("front-lift reaction needs a positive intention period");
    dwell_ticks_ = ticks(config_.unload_dwell_s, period_s_, "unload_dwell_s");
    decision_ticks_ = ticks(config_.decision_s, period_s_, "decision_s");
    clear_ticks_ = ticks(config_.clear_s, period_s_, "clear_s");
}

void FrontLiftReaction::reset() { s_ = {}; }

ReactionCommand FrontLiftReaction::update(const std::optional<Percepts> &percepts) {
    State n = s_;
    if (!percepts) {
        n.unload_ticks = 0;
        n.predicate = false;
    } else {
        const double hands = percepts->senses.hand_press_n;
        if (!std::isfinite(hands) || !std::isfinite(percepts->pitch_rate_up_rad_s) ||
            !std::isfinite(percepts->pitch_rad))
            throw std::invalid_argument("front-lift reaction percepts must be finite");
        n.unload_ticks = hands < config_.hand_unload_n ? n.unload_ticks + 1 : 0;
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
    if (s.unload_ticks < 0) throw std::invalid_argument("front-lift reaction unload_ticks must be nonnegative");
    if (s.decision_ticks < 0 || s.decision_ticks > decision_ticks_)
        throw std::invalid_argument("front-lift reaction decision_ticks outside the decision window");
    if (s.clear_ticks < 0 || s.clear_ticks > clear_ticks_)
        throw std::invalid_argument("front-lift reaction clear_ticks outside the clear window");
    if (!std::isfinite(s.effort_factor) || s.effort_factor < 1. - config_.ease_fraction || s.effort_factor > 1.)
        throw std::invalid_argument("front-lift reaction effort_factor outside [1 - ease_fraction, 1]");
    if (!std::isfinite(s.lean_add_rad) || s.lean_add_rad < 0. || s.lean_add_rad > config_.lean_add_rad)
        throw std::invalid_argument("front-lift reaction lean_add_rad outside [0, lean_add_rad]");
    if (s.phase == ReactionPhase::idle && (s.effort_factor != 1. || s.lean_add_rad != 0.))
        throw std::invalid_argument("front-lift reaction idle with an action in force");
    s_ = s;
}

const char *reaction_phase_name(ReactionPhase phase) noexcept {
    switch (phase) {
    case ReactionPhase::idle: return "idle";
    case ReactionPhase::deciding: return "deciding";
    case ReactionPhase::acting: return "acting";
    case ReactionPhase::releasing: return "releasing";
    }
    return "idle";
}

} // namespace rider
```

  - `reaction_codec.cpp` — по образцу `encode_perception_state`/`decode_perception_state` в `senses_codec.cpp`:
    точный набор ключей (`ncd::exact`), фаза — одна из четырёх строк, счётчики — целые `int64` ≥ 0, вещественные —
    конечные. Конфигурация: 13 ключей с типами (bool для трёх выключателей), после декодирования —
    `validate_reaction_table`.
  - Запись конфигурации — по образцу `rider_launch` части 2:
    - `using FrontLiftReactionConfig = ::rider::FrontLiftReactionConfig;`;
    - член `front_lift_reaction` после `rider_launch`;
    - 13 `FieldDecl`: `Real` или `Boolean`, `required=true`;
    - `kFrontLiftReactionConfigSchema` (путь `bike_sim.physics.front_lift_reaction.FrontLiftReactionConfig`,
      `declared_default=nullptr`);
    - корневой `FieldDecl` `front_lift_reaction`, `required=true`;
    - `BIKE_T13_RECORD`;
    - `detail::post_init(const FrontLiftReactionConfig&)` вызывает `::rider::validate_reaction_table`.
  - Корневой `post_init`, после проверок `rider_launch`:

```cpp
    detail::post_init(config.front_lift_reaction);
    // The reaction counts intention ticks (launch part 3a): its three times
    // sit on the seated_climb period grid, and its forward lean stays inside
    // the seated posture bound.
    for (const auto &[name, value] : {
             std::pair{"front_lift_reaction.unload_dwell_s", config.front_lift_reaction.unload_dwell_s},
             {"front_lift_reaction.decision_s", config.front_lift_reaction.decision_s},
             {"front_lift_reaction.clear_s", config.front_lift_reaction.clear_s}}) {
        const double ticks = std::rint(value / config.seated_climb.period_s);
        if (ticks < 0. || std::abs(ticks * config.seated_climb.period_s - value) > 1e-9)
            fail(std::string(name) + " must be an integer multiple of seated_climb.period_s");
    }
    if (config.front_lift_reaction.lean_add_rad > config.seated_climb.max_forward_lean_rad)
        fail("front_lift_reaction.lean_add_rad: beyond the seated forward lean bound");
```

  - `default_physics()` и все программные конфигурации — выключенная реакция с числами из описи:

```cpp
    config.front_lift_reaction = {.hand_unload_n = 20., .unload_dwell_s = .05, .pitch_rate_up_rad_s = .5,
                                  .pitch_rad = .17, .decision_s = .1, .ease_enabled = false,
                                  .lean_enabled = false, .rear_brake_tap_enabled = false,
                                  .ease_fraction = .7, .ease_rate_per_s = 5., .lean_add_rad = .1,
                                  .lean_rate_rad_s = .5, .clear_s = .3};
```

  - 16 файлов физики — скрипт по образцу шага 3 задачи 1 части 2. Блок вставляется перед `[monitors]`:

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

  - `initial_state.cpp:77`: убрать и `front_lift_reaction` в той же строке, что `research`/`monitors`/`rider_launch`;
    `equilibrium_cache.cpp:173`: в начале цикла по `fields` — `std::erase_if(fields, [](const auto& f){ return
    f.first == "front_lift_reaction"; });` для обеих ветвей, с комментарием «the reaction steers the ride, never
    the rest pose — like the soft-start fields».
- [ ] **Шаг 4.** Шаблон тестов задачи 1 части 2 (шаг 4) плюс `front_lift_reaction`; проверка числа разделов
  `[front_lift_reaction]` = `[monitors]` во встроенных документах печатает пустой список.
- [ ] **Шаг 5. Перезаписи** — по правилу; список или «нет».
- [ ] **Шаг 6. Коммит и проверка.** `feat(rider): the front-lift reaction table and node`; `check_task.sh`, база —
  коммит задачи 1, с `ab`.

### Задача 3. Восприятие и реакция в намерениях райдера

**Файлы:**
- Изменить:
  - `native/src/rider/perception.hpp`, `native/src/rider/perception.cpp` (`percepts()`);
  - `native/src/rider/intent.hpp`, `native/src/rider/intent.cpp` (конструктор, `resolve`, `reset`, `State`,
    `effort_limited`, `reaction`);
  - `native/src/rider/state_codec.cpp:2278-2400` (ключи `perception` и `reaction` состояния намерений);
  - `native/src/runtime/step.hpp:50-66` (`StepConfig::reaction`), `native/src/runtime/step.cpp` (конструктор
    `rider_intent_` `:1024`, вызов `resolve` `:1262`, `prepare` `:1331`, диагностика `:2131`);
  - `native/src/runtime/project_config.cpp:505` (ключ `rider_reaction`), `native/src/runtime/runtime.cpp:350`
    (декодирование), `native/src/runtime/bootstrap.cpp:253,361` и `RuntimeConfig` (новый член `rider_reaction`);
  - `native/src/runtime/accounting.cpp:224-230` (канал `rider_reaction`);
  - `native/resources/recording/catalog-v1.json`, `native/src/recording/catalog_data.inc` (по образцу `rider_intent`);
  - все вызовы конструктора `RiderIntent` (опись);
  - документы запуска и состояний в тестах (перезапись по правилу).
- Тест: `native/tests/reference/test_front_lift_reaction.cpp` (интеграция в намерения), `test_launch_preparation.cpp`
  (только новый конструктор), ездовой тест канала в `test_rider_senses.cpp`.

**Интерфейсы:**

```cpp
// perception.hpp
// The percepts the last update() returned, rebuilt from the state: the
// delayed sample and the integrated pitch. nullopt before the first one.
[[nodiscard]] std::optional<Percepts> Perception::percepts() const;

// intent.hpp
RiderIntent(IntentConfig config, FrontLiftReactionConfig reaction, double dt_s);
struct RiderIntent::State {
    std::int64_t last_tick_step = -1;
    Intent intent;
    SeatedClimbPolicy::State policy;
    SeatedPostureProgram::State program;
    std::optional<Perception::State> perception;        // engaged while the intent is enabled
    std::optional<FrontLiftReaction::State> reaction;   // engaged while the intent is enabled
};
// resolve() gains the interval's senses, right after `signals`:
IntentControl resolve(IntentControl control, const IntentSignals &signals,
                      const RiderSenses &senses, std::int64_t step, double road_grade,
                      std::optional<double> preview_grade = std::nullopt,
                      std::optional<double> lean_limit_rad = std::nullopt,
                      bool active = true, bool advance = true,
                      std::optional<double> min_lean_rad = std::nullopt);
// The reaction's command of the last tick (idle and inert before one).
[[nodiscard]] ReactionCommand reaction() const;
// True when the rider limits the effort (the reaction eases it); the step
// passes it to the pedaling as cut_after_ceiling. Part 3b adds the plan.
[[nodiscard]] bool effort_limited() const;
```

- [ ] **Шаг 1. Тесты (падают).** В `test_front_lift_reaction.cpp` (добавить `#include "rider/intent.hpp"`,
  `#include "configuration/model_config.hpp"`):

```cpp
// The intent's reaction rides on its own delayed percepts: with both
// actions on, unloaded hands and a fast pitch-up make the resolver ease the
// effort and lean forward exactly reaction_delay + dwell + decision ticks
// after the first such sample; with both actions off the resolver output is
// bitwise the plain policy's, at every tick.
void test_intent_applies_the_reaction_after_delay_dwell_and_decision() {
    auto config = bike::configuration::default_physics().seated_climb;
    config.enabled = true;
    auto on = table();
    rider::RiderIntent reacting(config, on, kPeriod);
    auto off = on;
    off.ease_enabled = false;
    off.lean_enabled = false;
    rider::RiderIntent inert(config, off, kPeriod);
    rider::RiderIntent plain(config, off, kPeriod);
    rider::IntentSignals signals;
    signals.pitch_rate_up_rad_s = 1.;
    rider::RiderSenses senses;
    senses.hand_press_n = 5.;
    const std::int64_t delay = std::llround(config.reaction_delay_s / kPeriod);
    const std::int64_t onset = delay + 4 + 8;   // first percept + dwell(4) + decision(8)
    for (std::int64_t step = 0; step <= onset + 5; ++step) {
        const auto a = reacting.resolve(rider::IntentControl{}, signals, senses, step, 0., 0.);
        const auto b = inert.resolve(rider::IntentControl{}, signals, senses, step, 0., 0.);
        const auto c = plain.resolve(rider::IntentControl{}, signals, rider::RiderSenses{}, step, 0., 0.);
        bt::require(b.human_torque_nm == c.human_torque_nm &&
                        b.posture->torso_lean_rad == c.posture->torso_lean_rad,
                    "switched-off actions leave the resolver output bitwise unchanged");
        if (step < onset)
            bt::require(a.human_torque_nm == b.human_torque_nm && !reacting.effort_limited(),
                        "no action before delay + dwell + decision");
        if (step == onset)
            bt::require(*a.human_torque_nm < *b.human_torque_nm &&
                            a.posture->torso_lean_rad > b.posture->torso_lean_rad && reacting.effort_limited(),
                        "the action starts on the computed tick");
    }
}
```

  Если наклон политики ограничен `lean_limit` так, что добавка не видна, тест передаёт `lean_limit_rad =
  config.max_forward_lean_rad` явно (это аргумент `resolve`).

  В `test_rider_senses.cpp` (ездовой тест канала уже есть с части 1) добавить:

```cpp
// The front-lift reaction is recorded every interval: a short welded ride
// carries the rider_reaction channel, inert with the shipped switched-off
// table (any phase may show, the outputs never move).
[[gnu::noinline]] void test_reaction_channel_is_recorded() {
    auto env = launch_ride::environment(launch_ride::track("[[0.0, 0.0], [60.0, 0.0]]", 60.), 1.);
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
    bt::require(rows > 0, "the ride recorded intervals");
}
```

  Вызов — в `main` этого файла. Если канал `rider_reaction` в выборке интервала лежит не на верхнем уровне
  `sample->channels`, а внутри подробных каналов (как `rider_intent` в `accounting.cpp:224-230`), чтение идёт оттуда
  же; путь — в отчёт.
- [ ] **Шаг 2.** `mise run test 'front_lift_reaction|rider_senses'` — сборка падает.
- [ ] **Шаг 3. Код.**
  - `Perception::percepts()`: если `s_.delayed` пуст — `nullopt`, иначе тот же `Percepts`, что строит `update`
    (`perception.cpp:104-110`), с `pitch_rad = s_.pitch_rad`. `update` возвращает `percepts()` после своего
    изменения состояния, чтобы формула была одна.
  - `RiderIntent`:
    - члены `std::optional<Perception> perception_` и `std::optional<FrontLiftReaction> reaction_`, построенные в
      конструкторе, только если `config_.enabled` (у выключенных намерений период — заглушка);
    - `reset()` сбрасывает оба;
    - `state()` и `restore()` переносят оба;
    - `restore` отклоняет состояние, у которого присутствие `perception`/`reaction` не совпадает с
      `config_.enabled`, и ничего не меняет при отказе.
  - `resolve()` на тике, сразу после `intent_ = policy_.update(...)`:

```cpp
        // Launch part 3a: one acquisition per intention tick — the senses
        // of this interval join the signals in the delayed perception, and
        // the front-lift reaction decides on what the rider perceives now.
        if (perception_ && reaction_)
            (void)reaction_->update(perception_->update(SenseSample{.signals = signals, .senses = senses}));
```

    и в выдаче после `if (!control.rider_enabled) return control;`:

```cpp
    const ReactionCommand reacting = reaction();
    if (!control.posture.has_value()) {
        control.posture = intent_.posture;
        // A forward lean the rider adds while reacting, capped by the
        // posture's forward limit; zero adds nothing (bitwise unchanged).
        if (reacting.lean_add_rad != 0.) {
            control.posture->torso_lean_rad = std::min(
                control.posture->torso_lean_rad + reacting.lean_add_rad,
                lean_limit_rad.value_or(config_.max_forward_lean_rad));
            validate_posture(*control.posture);
        }
    }
    if (!control.human_torque_nm.has_value()) {
        control.human_torque_nm = intent_.effort_ceiling_nm;
        // The rider eases the effort ceiling while reacting; a factor of
        // one is not applied at all (bitwise unchanged).
        if (reacting.effort_factor != 1.)
            control.human_torque_nm = intent_.effort_ceiling_nm * reacting.effort_factor;
    }
    return control;
```

    `reaction()` возвращает `reaction_ ? reaction_->command() : ReactionCommand{}`; `effort_limited()` —
    `reaction().effort_factor < 1.`.
  - Кодек состояния намерений: ключи `perception` (через `encode_perception_state`/`decode_perception_state`) и
    `reaction` (через `encode_reaction_state`/`decode_reaction_state`), `null`, если намерения выключены; точный
    набор ключей.
  - Документ запуска:
    - `project_config.cpp` — `{"rider_reaction", rider::encode_reaction_config(config.front_lift_reaction)}`;
    - `RuntimeConfig::rider_reaction`;
    - `bootstrap.cpp` — ключ в обоих списках `required`;
    - `runtime.cpp` — `out.reaction = rider::decode_reaction_config(config.rider_reaction, "config.rider_reaction")`;
    - `StepConfig::reaction`;
    - `step.cpp:1024` — `rider_intent_(config_.intent, config_.reaction, config_.timestep_s)`.
  - `step.cpp` `resolve` (`:1262`): передать `rider_senses_` после `intent_signals_`. Вызов
    `stepper_->drive().prepare(...)`: последний аргумент — `rider_intent_.effort_limited()` вместо `false` из
    задачи 1.
  - Диагностика (`step.cpp:2131`): после `rider_intent` добавить `"rider_reaction"` — объект из четырёх полей
    (`phase` строкой через `reaction_phase_name`, `predicate`, `effort_factor`, `lean_add_rad`) или `null`, если
    намерения выключены. В `accounting.cpp` — `set(channels, "rider_reaction", required(raw.diagnostics,
    "rider_reaction"))` рядом с `rider_intent`. Каталог — по образцу записей `rider_intent`
    (`catalog_data.inc:5231,7323,7470`) в обоих файлах каталога.
  - Конструкторы `RiderIntent` в тестах: выключенная реакция с числами из описи (в тестах — как константа файла).
- [ ] **Шаг 4.** `mise run test 'front_lift_reaction|rider_senses|rider_perception|launch_preparation|seated_posture_program|seated_pedaling_cycle|realistic_pedelec_acceptance|io_state_codecs|runtime_bootstrap|native_runtime'`
  — новые зелёные, прежние как в базе (`runtime_bootstrap` и `native_runtime_*` красные: их причины сравнивает
  `check_task.sh`).
- [ ] **Шаг 5. Перезаписи** документов запуска и состояний — по правилу, с выводом `check_rerecord.py`.
- [ ] **Шаг 6. Коммит и проверка.** `feat(rider): perception and the front-lift reaction in the intent`;
  `check_task.sh`, база — коммит задачи 2, с `ab`.

### Задача 4. Итоговая проверка части 3a (оркестратор)

Команды раздела «Задача 4» части 2 в каталоге `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/final`; база причин —
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/final/reds_output.log`. Отчёт — с той же честной формулировкой
доказательства. Строка состояния в дорожной карте и запись в журнал; коммит `docs(plan): launch part 3a is in the code`.

## Ответы на ревью

| Пункт | Где |
|---|---|
| I4 — порядок «предел → срез» | Решение 6, задача 1; включается признаком `effort_limited()` в задаче 3 |
| I12 — восстановление по частям | Задача 2: кодек реакции, продолжение бит в бит после восстановления посреди решения и посреди возврата, атомарный отказ; восприятие уже проверено частью 1 и теперь лежит в состоянии намерений |
| I13 — проверки таблицы реакции | Задача 2: домены полей, сетка тиков, граница наклона, запрет заднего тормоза; у каждой правильный контроль |
| I14 — события реакции в записи | Решение 7, задача 3: канал `rider_reaction` в каждом интервале |

## Что остаётся части 3b

План старта (`rider/launch_plan.{hpp,cpp}`), поля таблицы удержания, отпускания тормоза, предела нажима,
переключения и конца старта, встраивание в `RiderIntent` с выключателем, протокол просьбы и конца среза,
передача обычной езде с посевом политики, события плана в записи, подбор чисел (C2, C3, I10, I11, I14 для плана).
