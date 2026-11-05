# План: реакции райдера на начало подъёма переднего колеса

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Цель.** Добавить три реакции райдера на начало подъёма переднего колеса. Каждая включается и выключается своим
ключом в `[seated_climb]`:
- **а)** ослабить или прекратить кручение;
- **б)** наклон вперёд: это существующая поправка наклона по нагрузке на переднее колесо, ей нужен отдельный
  выключатель;
- **в)** короткое торможение задним тормозом.

**Устройство.** Все три реакции живут в намерении райдера: `rider::SeatedClimbPolicy` и `rider::SeatedPostureProgram`
в `native/src/rider/intent.cpp`. Райдер видит нагрузку на переднее колесо с уже существующей задержкой реакции
`reaction_delay_s` (по умолчанию 0,15 с).

Признак «колесо начало подниматься» означает, что доля веса на переднем колесе не больше `lift_share_threshold` дольше
`lift_detect_s`. Отдельного признака отрыва в коде нет (проверено 2026-11-05).

Новые ключи сделаны по образцу `352f51e0`:
- значение по умолчанию не пересекает проекцию;
- старые записи пересчитываются с выброшенными ключами;
- контракт начального состояния их не видит.

При ключах по умолчанию поведение и все эталонные записи не меняются.

**Инструменты.** C++23, MuJoCo 3.12, CMake/CTest (`bike_add_reference_test`, `bt::` в
`native/tests/support/fixture_test.hpp`), `mise run test`, `bash tools/run_tests.sh`.

**Источник требований.**
- `docs/agents/simulation-decisions.md`, «Решено — не сделано», пункт 2. Это решение пользователя 2026-11-05: «все
  это должно также включаться/выключаться в настройках».
- Решение 2026-10-03 (сессия e7211d1f): задержка реакции около 0,2 с; педали в ноль примерно за 0,15 с, если подъём
  длится дольше 0,1 с; торможение задним при тангаже больше 25° и росте.
- Прочитай `docs/agents/native-engineering.md` перед задачей 1.

## Общие ограничения

- **Значения по умолчанию.** Все три реакции по умолчанию выключены, кроме (б). Наклон уже работает сегодня, поэтому
  `lean_trim_enabled = true`.
- **Эталонные записи.** Ни одна запись под `tests/fixtures/golden/**` не меняется. Если узел с эталонной записью
  падает, остановись и доложи (условие О1). Вручную поддерживаемые фикстуры конфигурации
  `tests/fixtures/config/**` обновлять можно, как это сделал `352f51e0`.
- **Профили.** `examples/research/viewer_physics_*.toml` в этом плане не меняются. Включать ли реакции в профилях,
  решает пользователь после задачи 5.
- **Каталог записи не меняется.** Новых столбцов нет, `native/tests/recording/test_catalog.cpp:43` (число 487)
  остаётся.
- **Провод управления не меняется.** Новое поле в `rider::IntentControl` и в ABI policy
  (`native/include/bike/policy_v1.h`) не добавляется. Торможение передаётся в шаг через `rider_intent_.intent()`.
- **Реальное время.** В `SeatedClimbPolicy::update` нет выделений памяти. Новое состояние — `std::optional<double>`.
- **Запреты** из `docs/agents/simulation-decisions.md`, пункты 36–41, действуют. Реакции — это поведение райдера по
  его ощущениям: нагрузка на колесо, наклон тела. Скрытых сил, приложенных к раме, нет.
- **Коммиты** в стиле `git log`: `feat(rider): …`, `test(rider): …`. В конце каждого сообщения:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Коммитить только по просьбе пользователя или по
  принятому ходу исполнения плана.
- **Сборка.** Сейчас нет ни одной сборки `build/native-release`, есть только `build/gui`. Пользователь 2026-11-05
  сказал пока не собирать. Перед задачей 1 спроси разрешение на `cmake --workflow --preset release` (полная сборка,
  около 10 ГБ).
- **Бюджет тестов** по `<git-common-dir>/bike-gate/timings.tsv`, последняя строка `5461865e native passed 598 s`.
  - **Внутри задачи.** `mise run test '<шаблон>'`, около 1–2 мин, каждая задача.
  - **Конец задач 1–4.** `bash tools/check_change.sh <base>`, не больше 5 мин, итого около 20 мин.
  - **Задача 5.** Три прогона по 30 с, около 5 мин.
  - **Итоговое доказательство.** Это физический путь, поэтому уровень 3: `bash tools/run_tests.sh full`, измерено
    1413 с на `dbc6ab7e`, около 25 мин.
  - **Итого** около 55 мин плюс сборка.
  - Никогда не запускать две проверки одновременно в одном рабочем дереве.

## Решения по умолчанию (подтвердить у пользователя до задачи 1)

- **Р1. Порог подъёма.** Колесо «начало подниматься», если доля веса на переднем колесе ≤ 0,05 дольше 0,1 с
  (`lift_share_threshold = 0.05`, `lift_detect_s = 0.1`). Доля «нет данных» (оба колеса без нагрузки,
  `step.cpp:2577-2579`) порогом не считается и сбрасывает счётчик.
- **Р2. Ослабление.**
  - `lift_effort_factor = 0.0` значит «прекратить», значение в (0, 1) — «ослабить».
  - Усилие уходит к цели за `lift_ease_time_s = 0.15` с: скорость изменения не меньше
    `max_crank_torque_nm / lift_ease_time_s`.
  - Обратно усилие возвращается с обычной скоростью `torque_slew_nm_s`.
- **Р3. Наклон.** Выключатель (б) отключает только медленную поправку наклона (`trim_rad`). Наклон по уклону дороги
  остаётся. Новый быстрый наклон «до предела» не добавляется.
- **Р4. Торможение.**
  - Условие: колесо поднимается (Р1), относительный тангаж (оценка наклона райдера минус угол дороги) ≥ 25°
    (`lift_brake_pitch_rad = 0.4363`), и тангаж растёт.
  - Тогда задний тормоз силой `lift_brake_demand = 0.3` на `lift_brake_duration_s = 0.15` с.
  - Повтор возможен не раньше, чем через ещё одну длительность.
  - Торможение не переводит педали в накат: тормоз добавляется после фиксации признака `braking`
    (`native/src/runtime/step.cpp:1554`).
- **Р5. Порядок работ.** Задача 4 (торможение) — TODO пользователя от 2026-11-05. Выполнять её только после
  отдельного «да».

## Условия остановки

- **О1.** После задачи 1 падает узел с эталонной записью под `tests/fixtures/golden/**`. Остановись, не правь
  запись, доложи.
- **О2.** В задаче 3 при включённом ослаблении в прогоне `rough_uphill_savage` колесо не опускается, или
  остановок становится больше. Доложи числа и ничего не подстраивай.
- **О3.** В задаче 4 торможение отрывает заднее колесо от дороги или блокирует его (скольжение больше 0,5 м/с).
  Остановись и доложи.

## Карта файлов

| Файл | Что меняется |
|---|---|
| `native/src/rider/intent.hpp` | 9 полей `IntentConfig`; состояние `lift_low_s`, `brake_left_s`; `Intent::rear_brake_demand` |
| `native/src/rider/intent.cpp` | признак подъёма, ослабление, выключатель поправки, торможение |
| `native/src/configuration/model_config.hpp:145-170` | записи `FieldDecl` для 9 ключей |
| `native/src/configuration/validate.cpp:805-847` | проверки диапазонов |
| `native/src/rider/state_codec.cpp:2143-2200` (настройки), `:1030-1052` и `:2340-2355` (состояние) | разбор и запись необязательных ключей |
| `native/src/runtime/project_config.cpp:492` | выбросить ключи со значением по умолчанию из `rider_intent` |
| `native/src/replay/rebuild.cpp` (рядом с `drop_if_default_reposition`) | старые записи без ключей |
| `native/src/initialization/initial_state.cpp:65-75` | ключи не входят в контракт |
| `native/src/runtime/step.cpp:1554-1560` | добавить задний тормоз от намерения |
| `native/tests/reference/test_seated_posture_program.cpp`, `test_seated_climb_surge.cpp`, `native/tests/config/test_schema.cpp` | тесты |
| `tests/fixtures/config/schema.json`, `tests/fixtures/config/resolved/*.json` | фикстуры конфигурации |

---

### Задача 1: Девять ключей в `[seated_climb]` без поведения

**Файлы:**
- Изменить: `native/src/rider/intent.hpp:27-47`, `native/src/configuration/model_config.hpp:145-170`,
  `native/src/configuration/validate.cpp:805-847`, `native/src/rider/state_codec.cpp:2135-2205`,
  `native/src/runtime/project_config.cpp:492`, `native/src/replay/rebuild.cpp`,
  `native/src/initialization/initial_state.cpp:65-75`
- Тест: `native/tests/config/test_schema.cpp`
- Фикстуры: `tests/fixtures/config/schema.json`, `tests/fixtures/config/resolved/{welded,pneumatic,distributed}.json`
  и другие `*.resolved.json`, которые затронул `352f51e0` (`git show --stat 352f51e0`)

**Интерфейсы.**
- Даёт поля `rider::IntentConfig` для задач 2–4:
  - `bool lean_trim_enabled = true;`
  - `bool lift_ease_enabled = false;`
  - `double lift_share_threshold = .05;`
  - `double lift_detect_s = .1;`
  - `double lift_effort_factor = 0.;`
  - `double lift_ease_time_s = .15;`
  - `bool lift_brake_enabled = false;`
  - `double lift_brake_pitch_rad = .4363323129985824;` — это 25°;
  - `double lift_brake_demand = .3;`
  - `double lift_brake_duration_s = .15;`
- Девять ключей вместе с `lean_trim_enabled` — это 10 полей. Ниже «9 ключей реакции» значит все поля, кроме
  `lean_trim_enabled`.

- [ ] **Шаг 1: Написать падающий тест разбора**

Добавь в `native/tests/config/test_schema.cpp` рядом с тестами `352f51e0` (найди их: `git show 352f51e0 --
native/tests/config/test_schema.cpp`):
```cpp
// Lift-reaction knobs (2026-11-05): they parse from [seated_climb], hold the
// declared defaults when absent, and reach rider::IntentConfig unchanged.
void test_seated_climb_lift_knobs_parse_and_default() {
    const auto defaults = rider::IntentConfig{};
    bt::require(defaults.lean_trim_enabled, "trim stays on by default");
    bt::require(!defaults.lift_ease_enabled && !defaults.lift_brake_enabled,
                "lift reactions are off by default");
    auto doc = load_example("examples/research/viewer_physics_welded.toml");  // the helper the 352f51e0 tests use
    set_toml(doc, "seated_climb.lift_ease_enabled", true);
    set_toml(doc, "seated_climb.lift_effort_factor", .4);
    set_toml(doc, "seated_climb.lean_trim_enabled", false);
    const auto config = resolve_physics(doc);  // same helper as the reposition test
    bt::require(config.seated_climb.lift_ease_enabled, "ease flag reaches the config");
    bt::require(config.seated_climb.lift_effort_factor == .4, "ease factor reaches the config");
    bt::require(!config.seated_climb.lean_trim_enabled, "trim flag reaches the config");
}
```
Имена `load_example`, `set_toml` и `resolve_physics` замени на те, что используют тесты `352f51e0` в этом файле.
Зарегистрируй функцию в `bt::run` того же файла.

- [ ] **Шаг 2: Убедиться, что тест падает**

Выполни: `mise run test 'config\.schema|test_schema'` (точное имя — по
`ctest --test-dir build/native-release -N | grep -i schema`).
Ожидается: ошибка компиляции «no member named 'lift_ease_enabled'».

- [ ] **Шаг 3: Добавить поля в `IntentConfig`**

В `native/src/rider/intent.hpp` после `double trim_dead_time_s = .2;` добавь:
```cpp
    // Lift reactions (simulation-decisions.md, «Решено — не сделано» 2;
    // user 2026-11-05). Each has its own switch; defaults keep today's
    // behaviour: the slow lean trim on, the two new reactions off.
    bool lean_trim_enabled = true;
    bool lift_ease_enabled = false;
    double lift_share_threshold = .05;
    double lift_detect_s = .1;
    double lift_effort_factor = 0.;
    double lift_ease_time_s = .15;
    bool lift_brake_enabled = false;
    double lift_brake_pitch_rad = .4363323129985824;  // 25 deg
    double lift_brake_demand = .3;
    double lift_brake_duration_s = .15;
```

- [ ] **Шаг 4: Добавить `FieldDecl`**

В `native/src/configuration/model_config.hpp` после `SC_REAL(trim_dead_time_s),` добавь:
```cpp
    {.name="lean_trim_enabled", .type="<class 'bool'>", .kind=FieldKind::Boolean,
     .offset=offsetof(SeatedClimbConfig, lean_trim_enabled), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="lift_ease_enabled", .type="<class 'bool'>", .kind=FieldKind::Boolean,
     .offset=offsetof(SeatedClimbConfig, lift_ease_enabled), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    SC_REAL(lift_share_threshold),
    SC_REAL(lift_detect_s),
    SC_REAL(lift_effort_factor),
    SC_REAL(lift_ease_time_s),
    {.name="lift_brake_enabled", .type="<class 'bool'>", .kind=FieldKind::Boolean,
     .offset=offsetof(SeatedClimbConfig, lift_brake_enabled), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    SC_REAL(lift_brake_pitch_rad),
    SC_REAL(lift_brake_demand),
    SC_REAL(lift_brake_duration_s),
```

- [ ] **Шаг 5: Проверки диапазонов**

В `post_init(const SeatedClimbConfig &)` (`native/src/configuration/validate.cpp`) перед последним `if` добавь:
```cpp
    scalar(config.lift_share_threshold, "lift_share_threshold", 0.);
    scalar(config.lift_detect_s, "lift_detect_s", 0.);
    scalar(config.lift_effort_factor, "lift_effort_factor", 0.);
    scalar(config.lift_ease_time_s, "lift_ease_time_s", std::nullopt, true);
    scalar(config.lift_brake_pitch_rad, "lift_brake_pitch_rad", std::nullopt, true);
    scalar(config.lift_brake_demand, "lift_brake_demand", std::nullopt, true);
    scalar(config.lift_brake_duration_s, "lift_brake_duration_s", std::nullopt, true);
    if (config.lift_share_threshold > 1. || config.lift_effort_factor > 1. ||
        config.lift_brake_demand > 1.)
        fail("lift reaction share, factor and brake demand must not exceed one");
    if (config.lift_brake_pitch_rad >= .5 * std::numbers::pi)
        fail("lift brake pitch must stay below a quarter turn");
```
Если `<numbers>` в файле нет, добавь `#include <numbers>`.

- [ ] **Шаг 6: Разбор на проводе — ключи необязательные**

В `decode_intent_config` (`native/src/rider/state_codec.cpp:2139-2148`) замени вызов `ncd::exact(d, keys(...))` на
форму с необязательным списком (`exact(d, required, optional)`, `native/src/writer_config.hpp:220-223`):
```cpp
    ncd::exact(d,
                  keys("enabled", "period_s", "reaction_delay_s",
                       "target_crank_power_w", "max_crank_torque_nm",
                       "torque_slew_nm_s", "lean_gain",
                       "max_forward_lean_rad", "max_backward_lean_rad",
                       "lean_rate_rad_s", "orientation_tau_s",
                       "surge_power_w", "surge_grade", "surge_budget_s",
                       "surge_recovery_rate", "front_load_share_target",
                       "lean_trim_gain_rad_s", "lean_trim_limit_rad",
                       "trim_dead_time_s"),
                  keys("lean_trim_enabled", "lift_ease_enabled",
                       "lift_share_threshold", "lift_detect_s",
                       "lift_effort_factor", "lift_ease_time_s",
                       "lift_brake_enabled", "lift_brake_pitch_rad",
                       "lift_brake_demand", "lift_brake_duration_s"));
```
После разбора границ (после цикла по `bounds`) добавь разбор с прежними значениями при отсутствии ключа:
```cpp
    // Lift-reaction knobs (2026-11-05): the projection sends only values that
    // differ from the declared defaults; an absent key decodes to that default.
    const IntentConfig defaults{};
    config.lean_trim_enabled = d.contains("lean_trim_enabled") ? d["lean_trim_enabled"].boolean(path) : defaults.lean_trim_enabled;
    config.lift_ease_enabled = d.contains("lift_ease_enabled") && d["lift_ease_enabled"].boolean(path);
    config.lift_brake_enabled = d.contains("lift_brake_enabled") && d["lift_brake_enabled"].boolean(path);
    const auto real_or = [&](std::string_view key, double fallback) {
        return d.contains(key) ? d[key].finite_real(path) : fallback;
    };
    config.lift_share_threshold = real_or("lift_share_threshold", defaults.lift_share_threshold);
    config.lift_detect_s = real_or("lift_detect_s", defaults.lift_detect_s);
    config.lift_effort_factor = real_or("lift_effort_factor", defaults.lift_effort_factor);
    config.lift_ease_time_s = real_or("lift_ease_time_s", defaults.lift_ease_time_s);
    config.lift_brake_pitch_rad = real_or("lift_brake_pitch_rad", defaults.lift_brake_pitch_rad);
    config.lift_brake_demand = real_or("lift_brake_demand", defaults.lift_brake_demand);
    config.lift_brake_duration_s = real_or("lift_brake_duration_s", defaults.lift_brake_duration_s);
```
Затем проверь диапазоны так же, как `post_init`: перенеси те же условия. Для ошибок используй `invalid(path + "." +
key, "invalid value or out of range")`, как цикл выше. Если у `Dict` нет `contains`, возьми тот же способ, что в
`writer_config.hpp:979` (`p.contains("rollback_release_mps")`).

- [ ] **Шаг 7: Проекция — выбросить значения по умолчанию**

В `native/src/runtime/project_config.cpp` строку `{"rider_intent", field(physics_document, "seated_climb")},` замени
на `{"rider_intent", project_rider_intent(config, physics_document)},`. Выше функции, где собирается этот словарь,
добавь:
```cpp
// Lift-reaction knobs (2026-11-05) are default-off mechanism fields: only a
// value that differs from the declared default crosses, so the projected
// rider_intent keeps its historical 19-key shape (byte-identical default
// output, the 352f51e0 pattern).
Value project_rider_intent(const cfg::SimulationPhysicsConfig& config, const Value& document) {
    Value out = field(document, "seated_climb");
    const auto& c = config.seated_climb;
    const rider::IntentConfig defaults{};
    static const std::initializer_list<std::pair<std::string_view, bool rider::IntentConfig::*>> kBools{
        {"lean_trim_enabled", &rider::IntentConfig::lean_trim_enabled},
        {"lift_ease_enabled", &rider::IntentConfig::lift_ease_enabled},
        {"lift_brake_enabled", &rider::IntentConfig::lift_brake_enabled}};
    static const std::initializer_list<std::pair<std::string_view, double rider::IntentConfig::*>> kReals{
        {"lift_share_threshold", &rider::IntentConfig::lift_share_threshold},
        {"lift_detect_s", &rider::IntentConfig::lift_detect_s},
        {"lift_effort_factor", &rider::IntentConfig::lift_effort_factor},
        {"lift_ease_time_s", &rider::IntentConfig::lift_ease_time_s},
        {"lift_brake_pitch_rad", &rider::IntentConfig::lift_brake_pitch_rad},
        {"lift_brake_demand", &rider::IntentConfig::lift_brake_demand},
        {"lift_brake_duration_s", &rider::IntentConfig::lift_brake_duration_s}};
    std::erase_if(std::get<Object>(out.data), [&](const auto& entry) {
        for (const auto& [name, member] : kBools) if (entry.first == name) return c.*member == defaults.*member;
        for (const auto& [name, member] : kReals) if (entry.first == name) return c.*member == defaults.*member;
        return false;
    });
    return out;
}
```
Имя поля `config.seated_climb` проверь по `native/src/configuration/model_config.hpp:213-216`. Если оно другое,
подставь фактическое.

- [ ] **Шаг 8: Пересчёт старых записей и контракт начального состояния**

- В `native/src/replay/rebuild.cpp` рядом с `drop_if_default_reposition` добавь `drop_if_default_lift(Value&
  seated_climb)`. Это та же схема: `at_default` для двух видов значения и таблица из шага 7. В
  `verify_configuration` (рядом с блоком reposition) вызывай её, когда в записанном `physics.seated_climb` нет ключа
  `lift_ease_enabled`.
- В `native/src/initialization/initial_state.cpp`, рядом с блоком reposition (строки 70-75), выброси те же 10 ключей
  из `config["seated_climb"]`. Комментарий: «lift-reaction knobs steer the rider intent, never the compiled model or
  the rest pose (2026-11-05)».

- [ ] **Шаг 9: Обновить фикстуры конфигурации**

Посмотри, как `352f51e0` менял `tests/fixtures/config/schema.json` и `resolved/*.json` (`git show 352f51e0 --
tests/fixtures/config/`). Добавь 10 ключей тем же способом. Если у теста схемы есть режим перезаписи, найди его через
`rg -n 'RECORD|record' native/tests/config/test_schema.cpp`. Если режима нет, правь вручную.

- [ ] **Шаг 10: Убедиться, что тест проходит, и проверить неизменность по умолчанию**

Выполни: `mise run test 'schema|resolution|native_rider_intent|seated_|replay|initial_state'`
Ожидается: всё зелёное, ни один файл под `tests/fixtures/golden/**` не изменён (`git status --short
tests/fixtures/golden` пусто). Иначе условие О1.

- [ ] **Шаг 11: Проверка этапа и коммит**

Выполни до коммита: `bash tools/check_change.sh <коммит перед задачей 1>`. Запиши его хэш (`git rev-parse HEAD`) до
начала задачи.
Ожидается: выход 0.
```bash
git add native/src/rider/intent.hpp native/src/configuration/model_config.hpp native/src/configuration/validate.cpp \
  native/src/rider/state_codec.cpp native/src/runtime/project_config.cpp native/src/replay/rebuild.cpp \
  native/src/initialization/initial_state.cpp native/tests/config/test_schema.cpp tests/fixtures/config
git commit -m "feat(rider): add the lift-reaction knobs to seated_climb (default off)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 2: Выключатель медленной поправки наклона

**Файлы:**
- Изменить: `native/src/rider/intent.cpp:131-139` (`SeatedPostureProgram::update`)
- Тест: `native/tests/reference/test_seated_posture_program.cpp`

**Интерфейсы.** Использует `IntentConfig::lean_trim_enabled` из задачи 1.

- [ ] **Шаг 1: Написать падающий тест**

Добавь рядом с `test_trim_is_delayed_and_increases_forward_lean_for_light_front` (строки 487-510):
```cpp
// With lean_trim_enabled = false the light front wheel no longer adds forward
// lean: the lean stays at the grade baseline (user switch, 2026-11-05).
void test_trim_switch_off_keeps_grade_lean() {
    rider::IntentConfig config;
    config.enabled = true;
    config.lean_rate_rad_s = 10.;
    config.trim_dead_time_s = .2;
    config.lean_trim_enabled = false;
    rider::SeatedPostureProgram program(config);
    const double base = std::atan(.15);
    double lean = 0.;
    for (int i = 0; i < 300; ++i)
        lean = program.update(i * .01, .15, .01, .20);
    bt::require(approx(lean, base), "the disabled trim leaves the grade baseline");
    bt::require(program.state().trim_rad == 0., "the disabled trim never integrates");
}
```
Зарегистрируй функцию в `bt::run`.

- [ ] **Шаг 2: Убедиться, что тест падает**

Выполни: `mise run test 'reference\.seated_posture_program'`
Ожидается: FAIL «the disabled trim leaves the grade baseline».

- [ ] **Шаг 3: Реализация**

В `native/src/rider/intent.cpp` в `SeatedPostureProgram::update` замени условие
`if (delayed_load_share_.has_value()) {` в блоке поправки на:
```cpp
    if (config_.lean_trim_enabled && delayed_load_share_.has_value()) {
```
Очередь `load_samples_` продолжает заполняться: состояние и его запись не меняются.

- [ ] **Шаг 4: Убедиться, что тест проходит**

Выполни: `mise run test 'reference\.(seated_posture_program|native_rider_intent|seated_climb_surge)'`
Ожидается: PASS.

- [ ] **Шаг 5: Проверка этапа и коммит**

`bash tools/check_change.sh <коммит задачи 1>` должен дать выход 0. Затем:
```bash
git add native/src/rider/intent.cpp native/tests/reference/test_seated_posture_program.cpp
git commit -m "feat(rider): switch the front-load lean trim off on request

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 3: Ослабить или прекратить кручение при подъёме колеса

**Файлы:**
- Изменить: `native/src/rider/intent.hpp:120-160` (`SeatedClimbPolicy::State`, член `lift_low_s_`),
  `native/src/rider/intent.cpp` (`SeatedClimbPolicy::reset`, `update`, `state`, `restore`),
  `native/src/rider/state_codec.cpp:1030-1052` (разбор) и `:2340-2355` (запись)
- Тест: `native/tests/reference/test_seated_climb_surge.cpp`

**Интерфейсы.**
- Даёт член `std::optional<double> lift_low_s_` и поле `State::lift_low_s`. Значение есть, только если включена
  реакция (а) или (в), иначе `nullopt`.
- Даёт `[[nodiscard]] bool SeatedClimbPolicy::front_lifting() const`: истина, когда счётчик ≥ `lift_detect_s`.
  Задача 4 его использует.

- [ ] **Шаг 1: Написать падающие тесты**

Добавь в `native/tests/reference/test_seated_climb_surge.cpp`:
```cpp
// Feed a steady climb, then a light front wheel; with the ease reaction on the
// effort falls to factor*target within lift_ease_time_s after detection, and
// comes back at the normal slew once the front reloads (user 2026-11-05).
rider::IntentSignals climb_signals(double share) {
    rider::IntentSignals s;
    s.crank_rate_rad_s = 7.;   // ~67 rpm, inside the torque-capped band
    s.specific_force_body_mps2 = {0., 0., 9.81};
    s.front_load_share = share;
    return s;
}
void test_lift_ease_stops_effort_and_recovers() {
    rider::IntentConfig config;
    config.enabled = true;
    config.reaction_delay_s = 0.;
    config.lift_ease_enabled = true;
    rider::SeatedClimbPolicy policy(config);
    double effort = 0.;
    for (int i = 0; i < 100; ++i) effort = policy.update(climb_signals(.3), .01, .1).effort_ceiling_nm;
    bt::require(effort > 20., "steady climb builds effort");
    for (int i = 0; i < 9; ++i) effort = policy.update(climb_signals(.02), .01, .1).effort_ceiling_nm;
    bt::require(effort > 20., "no reaction before lift_detect_s");
    for (int i = 0; i < 16; ++i) effort = policy.update(climb_signals(.02), .01, .1).effort_ceiling_nm;
    bt::require(effort <= 1e-9, "factor 0 stops the effort within the ease time");
    for (int i = 0; i < 100; ++i) effort = policy.update(climb_signals(.3), .01, .1).effort_ceiling_nm;
    bt::require(effort > 20., "the effort returns once the front reloads");
}
// Default config: the light front changes nothing and the state carries no
// lift counter (byte-identical default output).
void test_lift_ease_off_by_default() {
    rider::IntentConfig config;
    config.enabled = true;
    config.reaction_delay_s = 0.;
    rider::SeatedClimbPolicy on(config), off(config);
    for (int i = 0; i < 200; ++i) {
        (void)on.update(climb_signals(.3), .01, .1);
        (void)off.update(climb_signals(i < 100 ? .3 : .02), .01, .1);
    }
    bt::require(on.state().effort_nm == off.state().effort_nm, "default ignores the light front");
    bt::require(!off.state().lift_low_s.has_value(), "default state has no lift counter");
}
```
Зарегистрируй обе функции в `bt::run`. Если `power_target_w` при 250 Вт и 7 рад/с даёт меньше 20 Н·м, подбери
`crank_rate_rad_s` так, чтобы `crank_effort_ceiling` (`native/src/rider/spindle_math.cpp:53-63`) был не меньше 30 Н·м.
Запиши в комментарии, откуда число.

- [ ] **Шаг 2: Убедиться, что тесты падают**

Выполни: `mise run test 'reference\.seated_climb_surge'`
Ожидается: ошибка компиляции «no member named 'lift_low_s'».

- [ ] **Шаг 3: Состояние**

- В `intent.hpp`, в `SeatedClimbPolicy::State`, после `surge_budget_s_left` добавь
  `std::optional<double> lift_low_s;   // engaged only when a lift reaction is on`.
- В приватные члены добавь `std::optional<double> lift_low_s_;`.
- Объяви `[[nodiscard]] bool front_lifting() const { return lift_low_s_.has_value() && *lift_low_s_ + 1e-12 >=
  config_.lift_detect_s; }`.
- В `reset()` добавь
  `lift_low_s_ = config_.lift_ease_enabled || config_.lift_brake_enabled ? std::optional<double>{0.} : std::nullopt;`.
- В `state()` и `restore()` добавь поле `lift_low_s`.

- [ ] **Шаг 4: Поведение**

В `SeatedClimbPolicy::update` внутри `if (delayed_.has_value())` замени расчёт `target_effort` и шаг `effort_nm_`
на:
```cpp
        double target_effort = spindle::crank_effort_ceiling(
            power_target, config_.max_crank_torque_nm,
            delayed.crank_rate_rad_s);
        double slew = config_.torque_slew_nm_s;
        if (lift_low_s_.has_value()) {
            // The rider feels the light front through the bars and saddle
            // after reaction_delay_s; "no data" (both wheels unloaded) is not
            // a lift and resets the counter.
            const auto share = delayed.front_load_share;
            const bool light = share.has_value() && *share <= config_.lift_share_threshold;
            lift_low_s_ = light ? *lift_low_s_ + dt : 0.;
        }
        if (config_.lift_ease_enabled && front_lifting()) {
            target_effort *= config_.lift_effort_factor;
            slew = std::max(slew, config_.max_crank_torque_nm / config_.lift_ease_time_s);
        }
        effort_nm_ += std::max(-slew * dt, std::min(slew * dt, target_effort - effort_nm_));
```
Обратный рост идёт с той же `slew`, пока реакция активна. Когда колесо загрузилось, `front_lifting()` ложно, и
`slew` равна `torque_slew_nm_s`, как требует Р2.

- [ ] **Шаг 5: Запись и разбор состояния**

- В `native/src/rider/state_codec.cpp` при записи (около строки 2353, после `surge_budget_s_left`) добавь:
```cpp
        if (state.policy.lift_low_s.has_value())
            policy.emplace_back("lift_low_s", real_value(*state.policy.lift_low_s));
```
- При разборе (строки 1031-1034) замени `ncd::exact(policy, keys(...))` на форму с необязательным `keys("lift_low_s")`.
  После `surge_budget_s_left` добавь:
```cpp
        if (policy.contains("lift_low_s"))
            out.lift_low_s = policy["lift_low_s"].finite_real(policy_path);
```

- [ ] **Шаг 6: Убедиться, что тесты проходят**

Выполни: `mise run test 'reference\.(seated_climb_surge|native_rider_intent|seated_posture_program|seated_pedaling_cycle)'`
Ожидается: PASS, `git status --short tests/fixtures/golden` пусто.

- [ ] **Шаг 7: Проверка этапа и коммит**

`bash tools/check_change.sh <коммит задачи 2>` должен дать выход 0. Затем:
```bash
git add native/src/rider/intent.hpp native/src/rider/intent.cpp native/src/rider/state_codec.cpp \
  native/tests/reference/test_seated_climb_surge.cpp
git commit -m "feat(rider): ease or stop pedalling when the front wheel starts to lift

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 4: Короткое торможение задним тормозом (только после «да» пользователя, Р5)

**Файлы:**
- Изменить: `native/src/rider/intent.hpp` (`Intent`, `SeatedClimbPolicy::State`), `native/src/rider/intent.cpp`,
  `native/src/rider/state_codec.cpp`, `native/src/runtime/step.cpp:1554-1560`
- Тест: `native/tests/reference/test_seated_climb_surge.cpp`

**Интерфейсы.**
- Использует `front_lifting()` и `lift_low_s_` из задачи 3.
- Даёт `std::optional<double> Intent::rear_brake_demand` (по умолчанию `nullopt`) и `std::optional<double>
  State::brake_left_s`.

- [ ] **Шаг 1: Написать падающий тест**

```cpp
// Front light + body pitched >= 25 deg above the road and still rising: a
// 0.15 s rear tap at 0.3, then a pause of one more duration (user TODO,
// 2026-11-05; thresholds from the 2026-10-03 decision).
void test_lift_brake_taps_once_then_pauses() {
    rider::IntentConfig config;
    config.enabled = true;
    config.reaction_delay_s = 0.;
    config.orientation_tau_s = 1e9;   // keep the inclination from the gyro only
    config.lift_brake_enabled = true;
    rider::SeatedClimbPolicy policy(config);
    auto s = climb_signals(.02);
    s.pitch_rate_up_rad_s = 2.;       // 0.02 rad per 10 ms tick
    std::vector<double> demand;
    for (int i = 0; i < 80; ++i)
        demand.push_back(policy.update(s, .01, 0.).rear_brake_demand.value_or(0.));
    const auto first = std::ranges::find_if(demand, [](double d) { return d > 0.; });
    bt::require(first != demand.end(), "the tap fires once pitch passes 25 deg");
    const auto start = static_cast<std::size_t>(first - demand.begin());
    bt::require(start >= 22, "not before inclination reaches 0.436 rad at 0.02 rad/tick");
    for (std::size_t i = start; i < start + 15; ++i) bt::require(demand[i] == .3, "tap holds 0.3 for 0.15 s");
    for (std::size_t i = start + 15; i < start + 30 && i < demand.size(); ++i)
        bt::require(demand[i] == 0., "pause of one more duration");
}
```

- [ ] **Шаг 2: Убедиться, что тест падает**

Выполни: `mise run test 'reference\.seated_climb_surge'`
Ожидается: ошибка компиляции «no member named 'rear_brake_demand'».

- [ ] **Шаг 3: Реализация в намерении**

- В `Intent` добавь `std::optional<double> rear_brake_demand;`.
- В состояние добавь `std::optional<double> brake_left_s;` и член `brake_left_s_`. В `reset()`:
  `brake_left_s_ = config_.lift_brake_enabled ? std::optional<double>{-config_.lift_brake_duration_s} : std::nullopt;`.
  Знак значения задаёт фазу: положительное — идёт торможение, отрицательное — пауза, ноль — готов.
- В `update`, внутри `if (delayed_.has_value())`, сразу после строки с `effort_nm_ +=` из задачи 3. Счётчик
  `lift_low_s_` к этому месту уже обновлён на текущем такте:
```cpp
        if (brake_left_s_.has_value()) {
            double &left = *brake_left_s_;
            if (left > 1e-12) left = std::max(0., left - dt) == 0. ? -config_.lift_brake_duration_s : left - dt;
            else if (left < -1e-12) left = std::min(0., left + dt);
            else if (front_lifting() &&
                     inclination_rad_ - std::atan(road_grade) >= config_.lift_brake_pitch_rad &&
                     delayed.pitch_rate_up_rad_s > 0.)
                left = config_.lift_brake_duration_s;
        }
```
- В возвращаемый `Intent` добавь
  `.rear_brake_demand = brake_left_s_ && *brake_left_s_ > 1e-12 ? std::optional<double>{config_.lift_brake_demand} : std::nullopt`.
- Запись и разбор `brake_left_s` сделай как у `lift_low_s` в задаче 3, шаг 5.

- [ ] **Шаг 4: Тормоз в шаге**

В `native/src/runtime/step.cpp` после блока `if (hold > 0.) { … }` (строки 1557-1560) добавь:
```cpp
        // Rider lift reaction (seated_climb.lift_brake_*): a short rear tap
        // from the last intent tick, added after `braking` is captured so it
        // does not force the pedals into coasting (2026-11-05).
        if (const auto tap = rider_intent_.intent().rear_brake_demand; tap.has_value())
            rear = std::max(rear, *tap);
```

- [ ] **Шаг 5: Убедиться, что тесты проходят, затем проверка этапа и коммит**

Выполни: `mise run test 'reference\.(seated_climb_surge|native_rider_intent)|runtime_encoding'`
Ожидается: PASS, эталонные записи не изменены. Затем `bash tools/check_change.sh <коммит задачи 3>` с выходом 0.
```bash
git add native/src/rider/intent.hpp native/src/rider/intent.cpp native/src/rider/state_codec.cpp \
  native/src/runtime/step.cpp native/tests/reference/test_seated_climb_surge.cpp
git commit -m "feat(rider): tap the rear brake when the front lifts and the body pitches up

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Задача 5: Прогоны на трассе и решение по профилям

**Файлы:**
- Создать: `docs/superpowers/audits/2026-11-05-front-lift-reactions-rides.md`

- [ ] **Шаг 1: Три прогона по 30 с, по одному, не параллельно**

Делай временные копии профиля в `/tmp`, сам профиль не меняй:
```bash
B=build/native-release/bin/bike-ride
cp examples/research/viewer_physics_welded.toml /tmp/lift_off.toml
sed 's/^\[seated_climb\]$/[seated_climb]\nlift_ease_enabled = true/' /tmp/lift_off.toml > /tmp/lift_ease.toml
sed 's/^\[seated_climb\]$/[seated_climb]\nlift_ease_enabled = true\nlift_brake_enabled = true/' /tmp/lift_off.toml > /tmp/lift_brake.toml
for v in off ease brake; do
  $B --backend native --physics-config /tmp/lift_$v.toml --track examples/research/rough_uphill_savage.toml \
     --headless --duration 30 --no-plots --check-policy diagnostic --out /tmp/lift_rides/$v; echo "$v rc=$?"; done
```
Ключи `--out` и `--check-policy` сверь с `docs/RIDE.md`. Если `--out` называется иначе, подставь фактическое имя.

- [ ] **Шаг 2: Свести числа в отчёт**

По каждому варианту запиши:
- пройденную дистанцию;
- число и длительность интервалов, где доля нагрузки переднего колеса ≤ 0,05;
- наибольший относительный тангаж;
- число событий `balance_lost`;
- первое нарушение и его время.

Столбцы найди в записи прогона: `rg -l 'front' /tmp/lift_rides/off` и каталог `native/resources/recording/catalog-v1.json`.
Если реакция ухудшает езду, это условие О2.

- [ ] **Шаг 3: Показать отчёт пользователю**

Задай вопрос: «Включать ли (а) и (в) в `viewer_physics_*.toml` и с какими значениями?» В этом плане профили не
меняются.

- [ ] **Шаг 4: Итоговое доказательство**

Выполни: `bash tools/run_tests.sh full`. Это один полный прогон, ничего больше параллельно не запускать.
Ожидается: выход 0. Строка в `timings.tsv` — доказательство для слияния.

- [ ] **Шаг 5: Обновить файл решений**

В `docs/agents/simulation-decisions.md`, «Решено — не сделано», пункт 2: отметь, что сделано (ключи, коммиты),
перенеси сделанное в «Действует» с датой и ссылкой на отчёт. Решение пользователя по профилям запиши отдельной
строкой.
