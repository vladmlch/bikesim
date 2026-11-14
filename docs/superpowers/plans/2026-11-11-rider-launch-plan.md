# План старта райдера — план реализации

> **Статус (2026-11-11):** это дорожная карта, а не исполнимый план. Ревью codex
> (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan-review/review.md`) признало её неготовой. По решению пользователя
> работа идёт серией исполнимых планов: часть 1 — основа и каналы (`2026-11-11-launch-1-channels.md`, заменяет
> задачи 1–5 ниже; **выполнена 2026-11-11, коммиты `c5ceece3`…`5e0fe3a0`**, итоговая проверка — задача 7 части 1);
> часть 2 — таблица опыта и подготовка (`2026-11-11-launch-2-preparation.md`, заменяет задачи 6–7 ниже; **выполнена
> 2026-11-11, коммиты `a9a0b689`…`79013274`**, итоговая проверка — задача 4 части 2); часть 3a — восприятие в шаге, реакция на подъём колеса, порядок среза
> (`2026-11-11-launch-3a-perception-reaction.md`, заменяет задачу 8 ниже; **выполнена 2026-11-12, коммиты
> `58b1ed70`…`88bcc6ad`**, итоговая проверка — задача 4 части 3a); часть 3b — узел плана старта, его место в намерениях, шаге, документах
> и записи (`2026-11-12-launch-3b-launch-plan.md`, заменяет задачи 9–11 ниже, мягкий старт остаётся до части 4; **выполнена 2026-11-12, коммиты
> `94f1cd8b`…`37735f58`**, итоговая проверка — задача 5 части 3b; план выключен во всех профилях, калибровка
> `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/calibration/report.md` ждёт решения пользователя); часть 4 пишется
> после этого решения. Ошибки этой редакции, которые
> поправлены в части 1: мотор при удержании выключается путём торможения, а не `motor_limit_nm = 0`; знак силы
> рук `dot(сумма force_on_rider_n, ось вверх рамы)`; тормоз и просьба о переключении — собственные команды
> райдера, а не поля `RideControl`.
>
> **Часть 4 выполнена (2026-11-14).** Часть 4 — включение (`2026-11-12-launch-4-activation.md`, заменяет задачи
> 12–15 ниже) — и часть 4б — удержание шатуна, гашение ноги, старт с −0,8 рад
> (`2026-11-13-launch-4b-crank-hold-damping.md`); коммиты `a7af8c37`…`25bcec65`, вместе с перезаписью
> эталонов. Правило старта записано в `docs/agents/simulation-decisions.md`, «Действует», пункт 42.

> **Для исполнителей-агентов:** обязательный навык — `superpowers:subagent-driven-development` (рекомендуется) или
> `superpowers:executing-plans`. Шаги отмечаются флажками (`- [ ]`). Исполнитель по умолчанию — Devin
> (`devin-orchestration`); финальное ревью всей ветки делает оркестратор сам.

**Цель.** Райдер трогается с места как живой человек:
- сам выбирает звезду и угол педали;
- держит тормоз, пока не набрал нажим;
- сам переключается на разгоне;
- поздно, по-человечески реагирует на подъём переднего колеса.

Решения райдер принимает по таблице опыта трёх типов: осторожный, обычный, резкий.

**Архитектура.** Три новых узла в `native/src/rider/`:
- адаптер ощущений `perception`;
- реакция на подъём колеса `front_lift_reaction` — одна на всю поездку;
- план старта `launch_plan`.

Узлы встают в `RiderIntent::resolve` рядом с `SeatedClimbPolicy`. Наружу план выдаёт уже существующие или
новые поля `IntentControl`: предел нажима, позу, тормоз райдера, ограничение мотора и просьбу о переключении.
Физика, мотор и механика переключения не меняются. Новые только каналы: тормоз райдера, просьба о
переключении и ощущения.

**Стек.** C++23, MuJoCo, CMake/CTest (`bike_add_reference_test`), конфигурация TOML со строгой схемой,
JSON/бинарные эталоны.

**Спецификация.** `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md` (редакция 2 и пороги от
2026-11-11). План читается вместе с ней.

## Общие ограничения

- Лимиты, пороги, полосы, фильтры, задержки, силы, `strength`, `pedal_torque_ripple`, запас 25 Н и допуски
  тестов не меняются. Новые пороги — только утверждённые в спецификации.
- У полей профиля нет значений по умолчанию в коде; читателей старых форм нет; профиль с удалённым полем не
  загружается и называет поле.
- У каждого значения один источник.
- Мотор не меняется; запрет 41 остаётся; пока зажат тормоз, мотор выключен.
- Райдер всегда сидит; стопа только давит; демпфирования в шарнирах нет.
- Райдер не читает точную нагрузку шин, трение, центр масс и характеристику мотора.
- Шаг физики 1,25 мс; такт намерений `seated_climb.period_s`; задержка реакции `reaction_delay_s` = 0,15 с.
- Известные красные тесты: 22 обычных (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-10-delays-t5b-check/reds.txt`)
  и 10 медленных (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-09-reds-final/slow_reds.txt`). После каждой задачи —
  не больше и те же.
- Эталонные записи, которые краснеют из-за изменения поведения, перезаписываются только с согласия
  пользователя. Изменение формы файлов (новые обязательные ключи) вносится в тестовые файлы в той же задаче,
  как в `5542fcfd`.
- Коммит: автор — git-пользователь репозитория. Последняя строка сообщения ровно
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Без строк про Devin. Без push. Без `git stash`.
- Не трогать `.claude/` и `.agents/`; не добиваться чистого `git status`.
- Утверждённые пороги (спецификация, «Проверки»):
  - отрыв колеса — нагрузка передней шины ниже 50 Н дольше 50 мс;
  - короткий отрыв — до 0,3 с;
  - буксовка — тяга / нормальная сила задней шины больше 0,9 μ дольше 50 мс;
  - откат — до 10 мм;
  - окно для контроллера — от 0,4 с;
  - каденс ног — до 110 об/мин.

## Структура файлов

| Файл | Ответственность |
|---|---|
| `native/src/rider/perception.{hpp,cpp}` (новый) | Ощущения райдера из сигналов модели, задержка и накопленный наклон; никаких шин и трения |
| `native/src/rider/front_lift_reaction.{hpp,cpp}` (новый) | Признак подъёма колеса и действия «ослабить» и «наклониться»; вся поездка |
| `native/src/rider/launch_plan.{hpp,cpp}` (новый) | Фазы старта, команды плана, подготовка (`prepare_launch`) |
| `native/src/rider/launch_config.hpp` (новый) | `LaunchConfig`, `FrontLiftReactionConfig`, `SurfaceLabel` и их проверка |
| `native/src/rider/intent.{hpp,cpp}` | `IntentSignals`/`IntentControl` расширяются; `RiderIntent::resolve` собирает план, реакцию и политику по таблице владения |
| `native/src/rider/state_codec.cpp` | Кодеки новых полей сигналов, управления, состояния и конфигурации |
| `native/src/runtime/step.{hpp,cpp}` | Сбор ощущений (руки, скорость, звезда, покрытие); слияние тормоза райдера; защёлка тормоза в состоянии |
| `native/src/runtime/static_brake.{hpp,cpp}` | Состояние защёлки (`State`) для продолжения |
| `native/src/drivetrain/shifting.{hpp,cpp}`, `native/src/config_types.hpp` | Просьба райдера и ответ на неё |
| `native/src/writers/drivetrain.{hpp,cpp}` | Передача просьбы и признака владения в автомат |
| `native/src/configuration/*` | Раздел `[rider_launch]` и `[front_lift_reaction]`; удаление авторских `drive.crank_phase_rad` и `drive.gearing.rear_teeth` |
| `native/src/initialization/{setup_owner,settle_contacts,finalize}.cpp` | Подготовка до сборки модели, наклон в успокоении, посев наклона после сброса |
| `native/src/runtime/project_config.cpp`, `native/src/runtime/config.hpp` | Карта меток покрытия для ощущений |
| `native/tests/reference/test_rider_perception.cpp`, `test_front_lift_reaction.cpp`, `test_launch_plan.cpp`, `test_rider_launch_rides.cpp` (новые) | Тесты узлов и поездок |
| `examples/research/viewer_physics_welded.toml` и другие профили, `examples/research/rider_type_{careful,aggressive}.toml` (новые) | Таблицы опыта |

## Бюджет тестов

Замеры из `<git-common-dir>/bike-gate/timings.tsv`:

| Прогон | Время |
|---|---|
| `full` (2026-11-05) | 671–1506 с |
| `native` | около 600 с |
| `check_change` | до 5 мин |
| `seated_pedaling_cycle` | около 18 с |
| `native_rider_allocation` | около 117 с |

План:
- в каждой задаче — только целевые `mise run test '<шаблон>'`, 1–3 мин;
- в конце каждой задачи — один `bash tools/check_change.sh <база>`, около 5 мин; когда он останавливается на
  известных красных, — отдельные проходы анализаторов по изменённым файлам, около 15 мин;
- в задачах 13–14 — медленные `seated_pedaling_cycle`, `realistic_pedelec_acceptance` и новые поездки;
- в конце — один `bash tools/run_tests.sh full`, около 25 мин.

Итого около 15 задач × 20 мин + 25 мин ≈ 5,5 ч машинного времени.

Отдельные проходы анализаторов:
`build/native-release/bin/bike-native-checks --kind {cppcheck,frontends,tidy} --build build/native-release --only-sources <список>`.
Список изменённых файлов строится так же, как в `tools/check_change.sh`.

---

## Этап 1. Основа

### Задача 1. Базовые числа и опись эталонов

Код не меняется. Результат — файл `docs/superpowers/plans/2026-11-11-rider-launch-plan-baseline.md` и остановка
для пользователя.

**Файлы:**
- Создать: `docs/superpowers/plans/2026-11-11-rider-launch-plan-baseline.md`

- [ ] **Шаг 1. Время разгона на ровном.** Из `.scratch/devin/2026-11-11-launch-baseline/report.md`, таблица
  «Grade × variant», строка «0 % norm»: до 8 / 15 / 20 км/ч — 1,01 / 2,13 / 3,25 с от начала поездки.
  - Это замер на `5542fcfd`. Мягкий старт на ровном не включается, поэтому поведение на ровном совпадает с
    `f616f525`: CSV `norm_g00` и `mash_g00` совпадают бит в бит.
  - Записать в файл базовое значение: **время до 20 км/ч на ровном = 3,25 с от начала поездки**. Указать
    постановку: `examples/research/viewer_physics_welded.toml`, трасса из `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch-baseline/`,
    автоматическое управление.
  - Время от отпускания тормоза сейчас то же самое, потому что тормоз отпускается в 0.
- [ ] **Шаг 2. Опись файлов, которые сменят форму.** Выполнить и сохранить вывод в файл:

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
git grep -l -E '"(pitch_rate_up_rad_s|human_crank_torque_nm|front_load_share)"' -- tests native/tests > /tmp/inv-signals.txt
git grep -l -E 'crank_phase_rad|rear_teeth' -- tests examples native/tests > /tmp/inv-gear.txt
git grep -l -E '"(cooldown_s|cut_remaining_s)"' -- tests native/tests > /tmp/inv-shifter.txt
git grep -l -E 'launch_effort_ramp_s' -- tests examples native/tests > /tmp/inv-gentle.txt
git grep -l -E 'front_brake_demand' -- tests native/tests native/src/recording > /tmp/inv-brake.txt
```

  Двоичные эталоны (`*.bin`) с этими ключами внутри найти так же, как в `5542fcfd`: по файлам-описаниям
  `*.json` рядом с `.bin`. Сгруппировать: схема и конфигурация; состояние (сигналы, политика); телеметрия
  (столбцы привода); траектории (эталоны поездок, которые сдвинутся только на этапе 5).
- [ ] **Шаг 3. Остановка.** Показать пользователю базовое время и опись. Без его «да» этап 2 не начинать.
- [ ] **Шаг 4. Коммит.**

```bash
git add docs/superpowers/plans/2026-11-11-rider-launch-plan-baseline.md
git commit -m "docs(plan): launch plan baseline time and golden inventory

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Этап 2. Каналы (поведение поездок не меняется)

### Задача 2. Ощущения райдера: новые сигналы и адаптер

**Файлы:**
- Создать: `native/src/rider/launch_config.hpp` (только `SurfaceLabel` в этой задаче), `native/src/rider/perception.hpp`, `native/src/rider/perception.cpp`
- Изменить: `native/src/rider/intent.hpp:60-66` (`IntentSignals`), `native/src/runtime/step.cpp:2616-2642` (`update_intent_signals`), `native/src/runtime/step.cpp:1793-1798` (передать `in.settle`), `native/src/runtime/config.hpp:57-82` и `native/src/runtime/step.hpp:47-62` (метки покрытия), `native/src/runtime/project_config.cpp:205-212` (проекция меток), `native/src/rider/state_codec.cpp` (кодек сигналов, `decode_signals` и кодер), CMake-список исходников `bike_app` (где перечислен `rider/intent.cpp`)
- Тест: создать `native/tests/reference/test_rider_perception.cpp`; зарегистрировать в `native/tests/reference/Phase3Ports.cmake` рядом с `seated_climb_surge`

**Интерфейсы:**
- Производит:

```cpp
// launch_config.hpp
namespace rider {
enum class SurfaceLabel : std::uint8_t { asphalt, hardpack, loose, wet, scree };
[[nodiscard]] std::optional<SurfaceLabel> surface_label(std::string_view name) noexcept;
[[nodiscard]] std::string_view surface_label_name(SurfaceLabel label) noexcept;
}
// intent.hpp — IntentSignals gains (all required on the wire):
//   double hand_press_n;        // hands pressing the bar along frame up, N (sum of both grips)
//   double speed_mps;           // bike speed along the track
//   int rear_teeth;             // engaged cog
//   SurfaceLabel surface_front; // label under the front contact
//   SurfaceLabel surface_ahead; // label at the preview point
// perception.hpp
namespace rider {
struct Percepts {
    double grade_ahead = 0.;
    SurfaceLabel surface_front = SurfaceLabel::hardpack;
    SurfaceLabel surface_ahead = SurfaceLabel::hardpack;
    double speed_mps = 0.;
    int rear_teeth = 0;
    double crank_rpm = 0.;
    double human_crank_torque_nm = 0.;
    double pitch_rate_up_rad_s = 0.;
    double pitch_rad = 0.;      // integrated from the delayed rate
    double hand_press_n = 0.;
};
class Perception {
public:
    struct State {
        std::deque<std::pair<double, IntentSignals>> samples;
        std::optional<IntentSignals> delayed;
        double time_s = 0.;
        double pitch_rad = 0.;
        double grade_ahead = 0.;
    };
    explicit Perception(double reaction_delay_s);
    void reset();
    // dt_s = intention period; grade_ahead is the preview grade the rider sees.
    [[nodiscard]] std::optional<Percepts> update(const IntentSignals &raw,
                                                 double grade_ahead, double dt_s);
    [[nodiscard]] State state() const;
    void restore(const State &state);
private:
    double delay_s_;
    State s_;
};
}
```

- [ ] **Шаг 1. Тест (падает).** `native/tests/reference/test_rider_perception.cpp`:

```cpp
// The rider perceives with the reaction delay and integrates pitch from
// the delayed vestibular rate; the adapter has no tyre-load or friction
// input at all (spec «Ощущения райдера»).
#include "support/fixture_test.hpp"
#include "rider/perception.hpp"

namespace {
rider::IntentSignals raw(double t) {
    rider::IntentSignals s;
    s.pitch_rate_up_rad_s = .5;
    s.crank_rate_rad_s = 2. * std::numbers::pi;  // 60 rpm
    s.human_crank_torque_nm = 10. * t;
    s.hand_press_n = 120.;
    s.speed_mps = 1.;
    s.rear_teeth = 51;
    s.surface_front = rider::SurfaceLabel::asphalt;
    s.surface_ahead = rider::SurfaceLabel::loose;
    return s;
}
// Nothing is perceived before reaction_delay_s has elapsed; afterwards the
// percept is the sample from exactly one delay earlier.
void test_percepts_arrive_after_the_reaction_delay() {
    rider::Perception p(.15);
    for (int i = 0; i < 15; ++i)
        bt::require(!p.update(raw(i * .01), .1, .01).has_value(),
                    "no percept inside the reaction delay");
    const auto first = p.update(raw(.15), .1, .01);
    bt::require(first.has_value(), "the first percept arrives at the delay");
    bt::require(std::abs(first->human_crank_torque_nm - 0.) < 1e-12,
                "the percept is the sample one delay earlier");
    bt::require(std::abs(first->crank_rpm - 60.) < 1e-9, "crank rate in rpm");
    bt::require(first->surface_ahead == rider::SurfaceLabel::loose,
                "surface ahead is a label");
}
// pitch_rad integrates the delayed upward rate over the intention period.
void test_pitch_integrates_the_delayed_rate() {
    rider::Perception p(.15);
    std::optional<rider::Percepts> last;
    for (int i = 0; i < 35; ++i) last = p.update(raw(i * .01), 0., .01);
    // 20 delayed ticks of .5 rad/s * .01 s = .1 rad
    bt::require(last.has_value() && std::abs(last->pitch_rad - .1) < 1e-9,
                "pitch integrates the delayed rate");
}
// A mid-run snapshot restores bitwise.
void test_state_round_trip_is_bitwise() {
    rider::Perception a(.15);
    for (int i = 0; i < 20; ++i) (void)a.update(raw(i * .01), .05, .01);
    rider::Perception b(.15);
    b.restore(a.state());
    for (int i = 20; i < 40; ++i) {
        const auto x = a.update(raw(i * .01), .05, .01);
        const auto y = b.update(raw(i * .01), .05, .01);
        bt::require(x.has_value() == y.has_value() &&
                        (!x || (x->pitch_rad == y->pitch_rad &&
                                x->human_crank_torque_nm == y->human_crank_torque_nm)),
                    "restored perception continues bitwise");
    }
}
}  // namespace

int main() {
    return bt::run_all({test_percepts_arrive_after_the_reaction_delay,
                        test_pitch_integrates_the_delayed_rate,
                        test_state_round_trip_is_bitwise});
}
```

  Если в `support/fixture_test.hpp` нет `bt::run_all`, использовать тот же `main`, что в
  `test_seated_climb_surge.cpp` (вызовы подряд в `try`). Регистрация:

```cmake
bike_add_reference_test(rider_perception ENGINE
  SOURCES test_rider_perception.cpp LIBRARIES bike_app bike_validation
  LABELS t52 rider)
```

- [ ] **Шаг 2. Запуск.** `mise run test 'rider_perception'`. Ожидается: не собирается, нет `rider/perception.hpp`.
- [ ] **Шаг 3. Код адаптера.** `native/src/rider/perception.cpp`:

```cpp
#include "perception.hpp"
#include <numbers>
#include <stdexcept>

namespace rider {
Perception::Perception(double reaction_delay_s) : delay_s_(reaction_delay_s) {
    if (!std::isfinite(delay_s_) || delay_s_ < 0.)
        throw std::invalid_argument("perception reaction delay must be finite and nonnegative");
    reset();
}
void Perception::reset() { s_ = {}; }
std::optional<Percepts> Perception::update(const IntentSignals &raw,
                                           double grade_ahead, double dt_s) {
    if (!(dt_s > 0.)) throw std::invalid_argument("perception interval must be positive");
    s_.samples.emplace_back(s_.time_s, raw);
    const double ready = s_.time_s - delay_s_;
    while (!s_.samples.empty() && s_.samples.front().first <= ready + 1e-12) {
        s_.delayed = s_.samples.front().second;
        s_.samples.pop_front();
    }
    s_.time_s += dt_s;
    s_.grade_ahead = grade_ahead;
    if (!s_.delayed) return std::nullopt;
    const IntentSignals &d = *s_.delayed;
    s_.pitch_rad += d.pitch_rate_up_rad_s * dt_s;
    return Percepts{.grade_ahead = grade_ahead,
                    .surface_front = d.surface_front,
                    .surface_ahead = d.surface_ahead,
                    .speed_mps = d.speed_mps,
                    .rear_teeth = d.rear_teeth,
                    .crank_rpm = d.crank_rate_rad_s * 60. / (2. * std::numbers::pi),
                    .human_crank_torque_nm = d.human_crank_torque_nm,
                    .pitch_rate_up_rad_s = d.pitch_rate_up_rad_s,
                    .pitch_rad = s_.pitch_rad,
                    .hand_press_n = d.hand_press_n};
}
Perception::State Perception::state() const { return s_; }
void Perception::restore(const State &state) { s_ = state; }
}  // namespace rider
```

  Уклон впереди не задерживается: это зрение. Его уже задерживает просмотр дороги, как сейчас
  (`road_grade_preview`).
- [ ] **Шаг 4. Сбор сигналов в шаге.** Изменить `update_intent_signals`:
  - сигнатура: `void update_intent_signals(const WireObject &sensors_and_tires, const writers::RiderSettleOutcome *settle)`;
  - вызов на `step.cpp:1797` передаёт `&in.settle`;
  - сила рук:
    - сложить `force_on_rider_n` записей `grip_left` и `grip_right` из `settle->entries`;
    - спроецировать на ось «вверх» рамы со знаком минус: руки давят на руль, руль толкает руки вверх, это
      положительное давление;
    - ось рамы взять как у `saddle_support_n` (`native/src/writers/rider_contacts.hpp:211-216`).
  - скорость — `qvel` по `root_x_dofadr_`;
  - звезда — `stepper_->drive()` и состояние автомата: `ShiftingSnapshot::rear_teeth`;
  - метки покрытия:
    - из новой проекции `StepConfig::surface_labels` (по умолчанию и участки `[start_m, end_m)`) в точке
      переднего колеса `wheel_x_m_[0]` и в точке просмотра `wheel_x_m_[0] + config_.road_lookahead_m`;
    - проекция строится в `project_config.cpp` рядом с `surface_map`, но кладёт только имена меток, без `mu`;
  - `front_load_share` остаётся только в сигналах для обычной политики. Её перевод — вопрос пользователю,
    см. конец плана.
- [ ] **Шаг 5. Кодек сигналов.** В `state_codec.cpp`:
  - `decode_signals` требует новые ключи (`ncd::exact`);
  - кодер их пишет;
  - метка покрытия — строка из `surface_label_name`, неизвестная строка — ошибка.

  Все файлы из `/tmp/inv-signals.txt` получают новые ключи вставкой текста, как в `5542fcfd`:
  `hand_press_n: 0.0`, `speed_mps: 0.0`, `rear_teeth: 51`, `surface_front: "hardpack"`,
  `surface_ahead: "hardpack"`. Значения — нейтральные: в старых записях ощущений не было. Двоичные эталоны
  правятся тем же способом, что в `5542fcfd`: меняются входы, выходы не трогаются. Если выход сравнивается
  побайтно с кодером, тест входит в известные красные — проверить, что он уже там.
- [ ] **Шаг 6. Запуск.** `mise run test 'rider_perception|seated_climb_surge|io_state_codecs|native_rider_intent'`.
  Ожидается: `rider_perception` зелёный, остальные не хуже базы.
- [ ] **Шаг 7. Проверка этапа.** `bash tools/check_change.sh <HEAD задачи 1>`, отдельные проходы анализаторов.
  Красных — те же 22.
- [ ] **Шаг 8. Коммит.** `feat(rider): perception adapter and human-sensed intent signals`.

### Задача 3. Тормоз райдера и нажим при зажатом тормозе

**Файлы:**
- Изменить: `native/src/rider/intent.hpp:77-84` (`IntentControl`), `native/src/runtime/step.cpp:1259-1272` (`apply_forces_body`: слияние после флага `braking`), `native/src/runtime/step.cpp:1806-1812` (телеметрия), каталог телеметрии `native/src/recording/catalog_data.inc` (новые столбцы рядом с `front_brake_demand`), кодек управления в `native/src/runtime/samples.cpp`/`state_codec.cpp` (где пишется `human_torque_nm`, `step.cpp:286`), `native/src/runtime/programs.cpp:59-72` (декодер управления исследований), `native/src/replay/session.cpp` (сравнение применённых команд)
- Тест: дополнить `native/tests/reference/test_realistic_pedelec_acceptance.cpp` (рядом с `test_fifteen_percent_standstill_launch_rides_without_loop_out`)

**Интерфейсы:**
- Производит:

```cpp
// intent.hpp — IntentControl gains:
std::optional<double> rider_front_brake;  // rider's own brake demand, [0, 1]
std::optional<double> rider_rear_brake;
```

Правила:
- итоговый тормоз = наибольший из трёх: внешней команды, удержания от отката и тормоза райдера;
- флаг `braking` для педалирования и мотора считается только по внешней команде: так сейчас, `step.cpp:1262`;
- мотор во время удержания выключает план через уже существующее `motor_limit_nm = 0`:
  `AssistController` при пределе 0 держит своё состояние на 0 (`native/src/drivetrain/motor.cpp:97-108`), так
  что после отпускания мотор растёт с обычным запаздыванием.

- [ ] **Шаг 1. Тест (падает).**

```cpp
// The launch hold: with the rider's own brake applied and no caller brake,
// the legs keep their commanded effort (pedaling is not cut), the motor is
// held off by motor_limit_nm = 0, the bike does not move, and the rider's
// brake reaches the physics (recorded separately from the caller's).
void test_rider_brake_holds_the_bike_while_the_legs_preload() {
    auto env = standstill_env("tests/fixtures/tracks/climb_15pct_standstill.toml");
    runtime::RideControl control{};
    control.human_torque_nm = 30.;
    control.motor_limit_nm = 0.;
    control.rider_front_brake = 1.;
    control.rider_rear_brake = 1.;
    for (int k = 0; k < 400; ++k)  // 0.5 s
        (void)env.runtime->advance(env.runtime->snapshot().step + 1, control, 0., 0., std::nullopt);
    const auto &row = *intervals_of(*env.runtime).back();
    const auto &drive = sw::object(sw::required(row.channels, "drive"));
    bt::require(sw::number(sw::required(drive, "human_sensor_nm")) > 20.,
                "legs press against the held crank");
    bt::require(sw::number(sw::required(drive, "motor_torque_nm")) == 0.,
                "motor stays off while the rider brakes");
    bt::require(sw::number(sw::required(drive, "rider_front_brake_demand")) == 1.,
                "the rider's brake is recorded");
    bt::require(std::abs(row.qpos[0] - env.start_x_m) < .01, "the bike holds within 10 mm");
}
```

  `standstill_env` — новый помощник в этом файле по образцу окружения теста
  `test_fifteen_percent_standstill_launch_rides_without_loop_out`. Названия столбцов `human_sensor_nm` и
  `motor_torque_nm` сверить с каталогом телеметрии: в тесте `seated_pedaling_cycle` используется
  `human_sensor_nm`.
- [ ] **Шаг 2. Запуск.** `mise run test 'realistic_pedelec_acceptance'`. Ожидается: не собирается, нет
  `rider_front_brake`.
- [ ] **Шаг 3. Код.**
  - Поля в `IntentControl`.
  - В `apply_forces_body` сразу после строки `const bool braking = ...` (`step.cpp:1261-1262`):

```cpp
// The rider's own launch brake (spec «Тормоз от райдера»): it brakes the
// wheels but is not a caller braking event — pedaling is not cut, and the
// launch plan keeps the motor off through motor_limit_nm.
front = std::max(front, control.rider_front_brake.value_or(0.));
rear = std::max(rear, control.rider_rear_brake.value_or(0.));
```

  - В телеметрию привода (`step.cpp:1810-1812`) добавить `rider_front_brake_demand` и `rider_rear_brake_demand`.
    Итоговые значения уже пишутся как `front_brake_demand` и `rear_brake_demand` после слияния; проверить, что
    они берутся после новой строки.
  - Кодеки управления, исследований и воспроизведения принимают новые поля как обязательные, если они есть в
    форме: по образцу `motor_limit_nm`, необязательного по смыслу и `nullopt` на проводе. Тестовые файлы
    из `/tmp/inv-brake.txt` получают новые столбцы.
- [ ] **Шаг 4. Тест обычного торможения.** Внешний тормоз 1,0 при нажиме 30 Н·м — нажим обнуляется, как сейчас.
  Тест рядом, с комментарием.
- [ ] **Шаг 5. Запуск.** `mise run test 'realistic_pedelec_acceptance|pedelec_replay_contract|native_runtime'`.
- [ ] **Шаг 6. Проверка этапа и коммит.** `check_change.sh <HEAD задачи 2>`, проходы анализаторов;
  `feat(runtime): the rider's own brake channel, separate from caller braking`.

### Задача 4. Просьба о переключении

**Файлы:**
- Изменить: `native/src/rider/intent.hpp` (`IntentControl`), `native/src/writers/drivetrain.hpp:36-39` (`RideControl`), `native/src/writers/drivetrain.cpp:667-683` (вызов автомата), `native/src/drivetrain/shifting.hpp:122-124`, `native/src/drivetrain/shifting.cpp:21-104`, `native/src/config_types.hpp:69-74` (`ShiftingSnapshot`), кодек снимка автомата (`native/src/drivetrain/state_codec.cpp`), `step.cpp:1283-1287` (заполнение `drivetrain::RideControl`)
- Тест: `native/tests/reference/test_cadence_shifter.cpp`

**Интерфейсы:**
- Производит:

```cpp
// intent.hpp
enum class ShiftRequest : std::uint8_t { up, down };  // up = one cog smaller (harder)
// IntentControl gains:
std::optional<ShiftRequest> shift_request;
bool rider_owns_shifting = false;
// drivetrain::RideControl gains the same two fields (drivetrain::ShiftDirection up/down).
// config_types.hpp — ShiftingSnapshot gains:
std::string rider_response = "none";  // "none" | "accepted" | "deferred" | "rejected"
// shifting.hpp — new overload:
[[nodiscard]] bool update(double cadence_rpm, double required_cadence_rpm, double dt,
                          bool pedaling, bool braking, bool rear_in_contact,
                          std::optional<double> rear_slip_mps,
                          bool rider_owns, std::optional<ShiftDirection> rider_request);
```

Поведение при `rider_owns = true`:
- выбор звезды по полосе каденса (`shifting.cpp:57-85`) не выполняется;
- сглаживание, пауза между переключениями и срез идут как всегда (`shifting.cpp:32-42`);
- просьба во время паузы или среза — ответ `deferred`, переключения нет;
- на краю кассеты — `rejected`;
- запрет по проскальзыванию и по каденсу посадки (`shifting.cpp:90-94`) с прежними числами — `rejected`;
- иначе соседняя звезда, `accepted`, запуск паузы и среза как у автомата (`shifting.cpp:96-103`);
- без просьбы — `none`.

При `rider_owns = false` всё как сейчас, а `rider_response = "none"`.

- [ ] **Шаг 1. Тесты (падают)** в `test_cadence_shifter.cpp`:

```cpp
// Rider-owned shifting: the band never selects a cog; an up request moves
// exactly one cog smaller with the usual cooldown and cut.
void test_rider_up_request_moves_one_cog_with_cooldown_and_cut() {
    auto s = shifter();  // 28T, band [75,110]
    bt::require(!s.update(130., 130., .01, true, false, true, std::nullopt, true, std::nullopt),
                "no band shift while the rider owns shifting");
    bt::require(s.update(90., 90., .01, true, false, true, std::nullopt, true,
                         drivetrain::ShiftDirection::up),
                "the up request shifts");
    bt::require(s.state().rear_teeth == 24 && s.state().rider_response == "accepted" &&
                    s.state().cut_remaining_s > 0. && s.state().cooldown_s > 0.,
                "one cog smaller, cut and cooldown armed");
}
// A request inside the cooldown is deferred, not dropped or executed.
void test_rider_request_inside_cooldown_is_deferred() {
    auto s = shifter();
    (void)s.update(90., 90., .01, true, false, true, std::nullopt, true, drivetrain::ShiftDirection::up);
    bt::require(!s.update(90., 90., .01, true, false, true, std::nullopt, true,
                          drivetrain::ShiftDirection::up),
                "no second shift inside the cooldown");
    bt::require(s.state().rider_response == "deferred", "deferred response");
}
// The cassette end rejects the request.
void test_rider_request_past_the_cassette_end_is_rejected() {
    auto s = shifter_at(51);  // new helper: same config, rear_teeth 51
    bt::require(!s.update(60., 60., .01, true, false, true, std::nullopt, true,
                          drivetrain::ShiftDirection::down),
                "no cog larger than 51");
    bt::require(s.state().rider_response == "rejected", "rejected response");
}
```

  `shifter_at(int)` добавить в `drive_reference_support.hpp` рядом с `shifter()`.
- [ ] **Шаг 2. Запуск.** `mise run test 'cadence_shifter'`. Ожидается: не собирается.
- [ ] **Шаг 3. Код.** Старую перегрузку `update(...)` оставить: она вызывает новую с `rider_owns = false`.
  Ветка владения райдера:

```cpp
if (rider_owns) {
    if (!rider_request) { next.rider_response = "none"; return finish(false); }
    if (!config_.enabled || braking || !contact || next.cooldown_s > 1e-12 ||
        next.cut_remaining_s > 1e-12) { next.rider_response = "deferred"; return finish(false); }
    const auto it = std::ranges::find(config_.cassette, next.rear_teeth);
    const bool up = *rider_request == ShiftDirection::up;
    if ((up && it == config_.cassette.begin()) || (!up && std::next(it) == config_.cassette.end())) {
        next.rider_response = "rejected"; return finish(false);
    }
    const int selected = up ? *std::prev(it) : *std::next(it);
    if (slip && slip_mode_ == SlipMode::magnitude) slip = std::abs(*slip);
    const double landing = engaged(next.required_ema) * selected / next.rear_teeth;
    if ((up && slip && *slip > config_.upshift_slip_limit_mps) ||
        (up && landing < config_.target_cadence_min_rpm) ||
        (!up && landing > config_.target_cadence_max_rpm)) {
        next.rider_response = "rejected"; return finish(false);
    }
    next.from_teeth = next.rear_teeth; next.rear_teeth = selected; next.required_ema = landing;
    next.direction = shift_direction_name(*rider_request); ++next.shift_count;
    next.cooldown_s = config_.shift_cooldown_s; next.cut_remaining_s = config_.shift_cut_duration_s;
    next.rider_response = "accepted";
    return finish(true);
}
```

  Эта ветка стоит после обновления сглаживания и часов (`shifting.cpp:32-42`) и до проверки
  `!config_.enabled || ...` на строке 43. Писатель привода передаёт `in.control.rider_owns_shifting` и
  `in.control.shift_request` (`drivetrain.cpp:681-683`). Снимок `rider_response` входит в кодек;
  тестовые файлы из `/tmp/inv-shifter.txt` получают `"rider_response": "none"`.
- [ ] **Шаг 4. Запуск.** `mise run test 'cadence_shifter|realistic_pedelec_acceptance|native_drivetrain'`.
- [ ] **Шаг 5. Проверка этапа и коммит.** `feat(drivetrain): rider shift requests with accepted/deferred/rejected`.

### Задача 5. Защёлка тормоза в состоянии шага

Решение пользователя 2026-11-11 (вопрос 1, вариант «а»): сохранение и восстановление проверяется по частям —
состояние каждого нового узла и защёлка тормоза по кругу «записал → прочитал → продолжил». Полного продолжения
поездки посреди эпизода (MuJoCo со всеми писателями) в этой работе нет; оно — отдельная задача, если
понадобится.

**Файлы:**
- Изменить: `native/src/runtime/static_brake.hpp:64-92`, `native/src/runtime/static_brake.cpp`, `native/src/runtime/step.hpp:176` (`StepState`), `native/src/runtime/step.cpp:2720-2775`, кодек `StepState` (захват начального состояния `native/src/initialization/capture.cpp:208-213`)
- Тест: `native/tests/reference/test_rider_launch_rides.cpp` (создаётся здесь, пополняется в задаче 13; регистрация в `Phase3Ports.cmake`, метка `slow` добавляется в задаче 13)

**Интерфейсы:**

```cpp
// static_brake.hpp
struct State { std::optional<double> latch_front, latch_rear; };
[[nodiscard]] State state() const;
void restore(const State &state);
// step.hpp — StepState gains:
StaticBrake::State brake_latch;
```

`PhysicalStep::state()` пишет `brake_latch = brake_.state()`. `PhysicalStep::restore()` для нового эпизода не
меняется: защёлка по-прежнему сбрасывается (`step.cpp:2756-2758`). Записанное значение служит для записи и
проверки и для будущего полного продолжения.

- [ ] **Шаг 1. Тест (падает)** в `test_rider_launch_rides.cpp`:

```cpp
// The static brake latch is part of the step state: while the rider holds
// the brakes on 15 % the rear latch is captured, and the StepState codec
// round-trips it exactly (spec «Сохранение и восстановление», user ruling
// 2026-11-11: component-level round trips).
void test_brake_latch_is_captured_and_round_trips() {
    auto env = standstill_env("tests/fixtures/tracks/climb_15pct_standstill.toml");
    runtime::RideControl hold{};
    hold.human_torque_nm = 0.;
    hold.motor_limit_nm = 0.;
    hold.rider_front_brake = 1.;
    hold.rider_rear_brake = 1.;
    for (int k = 0; k < 200; ++k)  // 0.25 s
        (void)env.runtime->advance(env.runtime->snapshot().step + 1, hold, 0., 0., std::nullopt);
    runtime::StepState saved;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &, const runtime::PhysicalStep &p) { saved = p.state(); });
    bt::require(saved.brake_latch.latch_rear.has_value(), "the rear bite is latched while holding");
    const auto wire = runtime::encode_step_state(saved);
    const auto back = runtime::decode_step_state(wire);
    bt::require(back.brake_latch.latch_front == saved.brake_latch.latch_front &&
                    back.brake_latch.latch_rear == saved.brake_latch.latch_rear,
                "the latch round-trips through the state codec bitwise");
}
```

  `standstill_env` — помощник из задачи 3; перенести его в общий заголовок
  `native/tests/reference/launch_ride_support.hpp`, чтобы его видели оба файла. Имена `encode_step_state`
  и `decode_step_state` сверить с кодеком `StepState`, которым пользуется захват начального состояния
  (`capture.cpp:208-213`), и взять настоящие.
- [ ] **Шаг 2.** `mise run test 'rider_launch_rides'` — падает: нет `brake_latch`.
- [ ] **Шаг 3. Код** по интерфейсу. Кодек `StepState` пишет и читает `brake_latch` как обязательное поле;
  тестовые файлы начального состояния получают `"brake_latch": {"front": null, "rear": null}` вставкой текста.
- [ ] **Шаг 4.** `mise run test 'rider_launch_rides|io_state_codecs|native_runtime'`.
- [ ] **Шаг 5. Проверка этапа и коммит.** `feat(runtime): the static brake latch is part of the step state`.

---

## Этап 3. Таблица опыта и подготовка (поведение поездок не меняется)

### Задача 6. Схема таблицы опыта и реакции

**Файлы:**
- Изменить: `native/src/rider/launch_config.hpp` (полностью), `native/src/configuration/model_config.hpp` (новые разделы рядом с `SeatedClimbConfig`, `:39-41`, `:171`), `native/src/configuration/schema.cpp`, `native/src/configuration/validate.cpp` (`post_init` и `default_physics()`, `:1317-1325`), `native/src/rider/state_codec.cpp` (декодеры конфигурации по образцу `decode_intent_config`, `:2138-2200`), `native/src/runtime/config.hpp` (документы `rider_launch` и `front_lift_reaction`), `native/src/runtime/project_config.cpp` (проекция), все профили `examples/research/*.toml` и тестовые копии профилей (список — задача 1)
- Тест: `native/tests/reference/test_welded_profile_config.cpp`, `native/tests/config/test_schema.cpp`

**Интерфейсы** (`launch_config.hpp`):

```cpp
namespace rider {
struct GradeTableD { std::vector<double> grade; std::vector<double> value; };  // piecewise linear, grades ascending
struct LaunchConfig {
    // stepwise start cog: cog[i] applies for grade in [breaks[i-1], breaks[i]); size cog = breaks + 1
    std::vector<double> start_cog_grade_breaks;
    std::vector<int> start_cog_teeth;
    double start_pedal_angle_rad;
    GradeTableD start_lean_rad;
    GradeTableD release_torque_nm;
    double brake_release_s;
    double max_hold_s;
    double effort_rate_nm_s;
    std::array<double, 5> effort_cap_nm;   // indexed by SurfaceLabel
    double upshift_crank_rpm;
    double end_speed_mps;
    double end_crank_rpm;
    double end_hold_s;
    double max_launch_s;
    double handover_s;
    double standstill_speed_mps;
    double standstill_crank_rad_s;
};
struct FrontLiftReactionConfig {
    double hand_unload_n;          // hands feel unloaded below this press
    double unload_dwell_s;
    double pitch_rate_up_rad_s;
    double pitch_rad;
    double decision_s;             // added to reaction_delay_s
    bool ease_enabled;
    bool lean_enabled;
    bool rear_brake_tap_enabled;   // must be false: not implemented in this work
    double ease_fraction;          // keep (1 - ease_fraction) of the effort ceiling
    double ease_rate_per_s;
    double lean_add_rad;
    double lean_rate_rad_s;
    double clear_s;                // predicate absent this long ends the reaction
};
void validate(const LaunchConfig &c, const std::vector<int> &cassette);
void validate(const FrontLiftReactionConfig &c);
}
```

Проверки `validate`:
- все числа конечные; уклоны по возрастанию; размер `start_cog_teeth` = размер `breaks` + 1;
- каждая звезда есть в кассете;
- доли в [0, 1]; времена ≥ 0, а `brake_release_s`, `max_hold_s`, `max_launch_s` и `handover_s` > 0;
- `rear_brake_tap_enabled = true` отклоняется с сообщением «not implemented».

Разделы профиля: `[rider_launch]`, `[rider_launch.start_lean_rad]`, `[rider_launch.release_torque_nm]`,
`[rider_launch.effort_cap_nm]` (ключи — пять меток), `[front_lift_reaction]`.

Значения «как сегодня» для всех профилей и тестовых файлов: на этапе 3 план ещё не работает, поэтому
значения влияют только на подготовку (задача 7):
- `start_cog_grade_breaks = []`, `start_cog_teeth = [51]`;
- `start_pedal_angle_rad = 0.0` — нынешнее значение по умолчанию `drive.crank_phase_rad`;
- `start_lean_rad = { grade = [0.0], value = [0.0] }`;
- остальные поля — заметные предварительные значения с комментарием «preliminary, calibrated in task 11»:
  `release_torque_nm = { grade = [0.0, 0.2], value = [10.0, 40.0] }`, `brake_release_s = 0.3`,
  `max_hold_s = 2.0`, `effort_rate_nm_s = 40.0`, `effort_cap_nm` = 60 для `asphalt`, `hardpack` и `wet`, 40 для
  `loose` и `scree`, `upshift_crank_rpm = 90.0`, `end_speed_mps = 4.0`, `end_crank_rpm = 80.0`, `end_hold_s = 0.5`,
  `max_launch_s = 8.0`, `handover_s = 1.0`, `standstill_speed_mps = 0.1`,
  `standstill_crank_rad_s = 1.0` (как `launch_standstill_crank_rate_rad_s` мягкого старта);
- реакция: `hand_unload_n = 20.0`, `unload_dwell_s = 0.1`, `pitch_rate_up_rad_s = 0.5`, `pitch_rad = 0.17`,
  `decision_s = 0.1`, выключатели `true`/`true`/`false`, `ease_fraction = 0.7`, `ease_rate_per_s = 5.0`,
  `lean_add_rad = 0.1`, `lean_rate_rad_s = 0.5`, `clear_s = 0.3`.

Все предварительные числа подбирает задача 11, и показываются они пользователю.

- [ ] **Шаг 1. Тесты (падают).** В `test_welded_profile_config.cpp`, по образцу проверки обязательности полей
  мягкого старта (помощник изменения документа уже есть):

```cpp
// Every [rider_launch] and [front_lift_reaction] key is required: omitting
// any one fails to load and the error names the key.
void test_launch_and_reaction_keys_are_required() {
    for (const char *key : {"start_cog_teeth", "start_pedal_angle_rad", "brake_release_s",
                            "effort_rate_nm_s", "upshift_crank_rpm", "handover_s"})
        require_rejects_without(kWeldedProfile, "rider_launch", key);
    for (const char *key : {"hand_unload_n", "decision_s", "ease_enabled", "rear_brake_tap_enabled"})
        require_rejects_without(kWeldedProfile, "front_lift_reaction", key);
}
// A start cog that is not on the cassette is rejected.
void test_start_cog_must_be_on_the_cassette() {
    require_rejects_with(kWeldedProfile, "rider_launch.start_cog_teeth = [50]", "cassette");
}
// The unimplemented rear-brake tap cannot be switched on.
void test_rear_brake_tap_is_not_implemented() {
    require_rejects_with(kWeldedProfile, "front_lift_reaction.rear_brake_tap_enabled = true",
                         "not implemented");
}
```

  Имена помощников `require_rejects_without` и `require_rejects_with` взять из файла. Если их нет, написать по
  образцу помощника изменения документа из задачи мягкого старта (`e8d082c3`).
- [ ] **Шаг 2.** `mise run test 'welded_profile_config'` — падает.
- [ ] **Шаг 3. Код.** Разделы, схема, декодеры, проверки, `default_physics()` (те же значения явно), проекция в
  `RuntimeConfig` и `StepConfig`. Значения во всех профилях и тестовых файлах — по описи задачи 1, вставкой
  текста.
- [ ] **Шаг 4.** `mise run test 'welded_profile_config|config_schema|config_resolution|io_state_codecs'`. Не хуже
  базы.
- [ ] **Шаг 5. Проверка этапа и коммит.** `feat(config): rider launch experience table and front-lift reaction schema`.

### Задача 7. Подготовка: звезда и угол педали становятся вычисленными

**Файлы:**
- Создать: функция `prepare_launch` в `native/src/rider/launch_plan.{hpp,cpp}` (в этой задаче в файле только она)
- Изменить: `native/src/configuration/schema.cpp` и `physical_config.hpp:290-295` (поля `drive.crank_phase_rad` и `drive.gearing.rear_teeth` больше не авторские: схема профиля их отклоняет, структура их сохраняет), `native/src/initialization/setup_owner.cpp:162-178` (до `compile_mjcf`), `native/src/cli/configuration.cpp:89,221` (флаги задают вычисленное значение явно), `native/src/initialization/settle_contacts.cpp:10-14` (поза успокоения), `native/src/initialization/finalize.cpp:70` (посев наклона), `native/src/rider/intent.{hpp,cpp}` (`RiderIntent::seed_lean(double)`)
- Тест: `native/tests/reference/test_launch_plan.cpp` (создаётся), `native/tests/reference/test_equilibrium_state_file.cpp`

**Интерфейсы:**

```cpp
namespace rider {
struct LaunchPreparation { int rear_teeth; double crank_phase_rad; double lean_rad; };
[[nodiscard]] LaunchPreparation prepare_launch(const LaunchConfig &config, double grade_at_start);
void RiderIntent::seed_lean(double lean_rad);  // sets policy and program lean after reset
}
```

- [ ] **Шаг 1. Тест (падает)** в `test_launch_plan.cpp`:

```cpp
// Preparation is a pure function of the table and the grade at the start
// point: stepwise cog, fixed pedal angle, interpolated lean.
void test_preparation_picks_cog_angle_and_lean_from_the_table() {
    rider::LaunchConfig c = normal_table();   // helper in this file: the stage-3 profile values
    c.start_cog_grade_breaks = {.03, .08};
    c.start_cog_teeth = {24, 33, 51};
    c.start_lean_rad = {{0., .2}, {0., .2}};
    const auto flat = rider::prepare_launch(c, 0.);
    const auto mid = rider::prepare_launch(c, .05);
    const auto steep = rider::prepare_launch(c, .15);
    bt::require(flat.rear_teeth == 24 && mid.rear_teeth == 33 && steep.rear_teeth == 51,
                "stepwise start cog by grade");
    bt::require(std::abs(mid.lean_rad - .05) < 1e-12, "lean interpolates linearly");
    bt::require(flat.crank_phase_rad == c.start_pedal_angle_rad, "pedal angle from the table");
}
```

  И в `test_welded_profile_config.cpp`: профиль с `drive.crank_phase_rad` или `drive.gearing.rear_teeth`
  отклоняется и называет поле.
- [ ] **Шаг 2.** `mise run test 'launch_plan|welded_profile_config'` — падает.
- [ ] **Шаг 3. Код.**
  - `prepare_launch`: ступенька по `std::upper_bound` над `breaks`; кусочно-линейная интерполяция с
    обрезанием по краям.
  - В `make_setup` после `request.track.validate()` и до `compile_mjcf`:
    - `grade = request.track.grade_profile ? request.track.grade_profile->slope(request.start_x_m) : 0.`;
    - `prep = rider::prepare_launch(physics.rider_launch, grade)`;
    - если CLI не задала значение явно, `physics.drive.gearing.rear_teeth = prep.rear_teeth` и
      `physics.drive.crank_phase_rad = prep.crank_phase_rad`;
    - результат подготовки положить в `request` для успокоения и посева.
  - Ключ кэша равновесия уже включает всю физическую конфигурацию (`equilibrium_cache.cpp:163-170`), а значит и
    вычисленные поля. Исключение полей мягкого старта (`:173`) не расширять.
  - `apply_initial_forces` передаёт `RideControl{.posture = RiderPosture{.torso_lean_rad = prep.lean_rad, .use_saddle = true}}`.
  - После `rider_intent_.reset()` в `finalize.cpp:70` вызвать `physical.rider_intent_.seed_lean(prep.lean_rad)`.
  - Флаги CLI `crank_phase` и `rear_teeth` пишут в вычисленные поля и отмечают это в записи поездки
    (`provenance`).
- [ ] **Шаг 4. Поведение не меняется.** Значения этапа 3 дают 51 и 0,0 рад, как сейчас.
  `mise run test 'seated_pedaling_cycle|realistic_pedelec_acceptance|equilibrium_state_file|launch_plan'`:
  поездки совпадают с базой, эталоны траекторий зелёные.
- [ ] **Шаг 5. Проверка этапа и коммит.** `feat(setup): derive the start cog and pedal angle from the launch table`.

---

## Этап 4. План и реакция

### Задача 8. Реакция на подъём переднего колеса

**Файлы:**
- Создать: `native/src/rider/front_lift_reaction.{hpp,cpp}`
- Тест: `native/tests/reference/test_front_lift_reaction.cpp` (+ регистрация)

**Интерфейсы:**

```cpp
namespace rider {
struct ReactionCommand { double effort_factor = 1.; double lean_add_rad = 0.; bool active = false; };
class FrontLiftReaction {
public:
    struct State {
        double unload_s = 0.;      // time the hands have felt unloaded
        double since_trigger_s = -1.;
        double clear_for_s = 0.;
        double effort_factor = 1.;
        double lean_add_rad = 0.;
        bool triggered = false;
    };
    explicit FrontLiftReaction(FrontLiftReactionConfig config);
    void reset();
    [[nodiscard]] ReactionCommand update(const Percepts &p, double dt_s);
    [[nodiscard]] State state() const;
    void restore(const State &s);
};
}
```

Порядок признака, по спецификации:
1. `p.hand_press_n < hand_unload_n` копится в `unload_s`, иначе `unload_s` = 0;
2. сработало, если `unload_s >= unload_dwell_s` и при этом `p.pitch_rate_up_rad_s >= pitch_rate_up_rad_s` или
   `p.pitch_rad >= pitch_rad`;
3. после срабатывания часы `since_trigger_s` идут; действия начинаются при `since_trigger_s >= decision_s`.
   Задержка `reaction_delay_s` уже внутри ощущений;
4. действия: `effort_factor` движется к `1 - ease_fraction` со скоростью `ease_rate_per_s` (если
   `ease_enabled`), `lean_add_rad` — к `lean_add_rad` со скоростью `lean_rate_rad_s` (если `lean_enabled`);
5. окончание: признака нет `clear_s` подряд — `triggered = false`, действия возвращаются с теми же скоростями.

- [ ] **Шаг 1. Тесты (падают):**

```cpp
// A short bar unloading without pitch-up never triggers (variant E's failure mode).
void test_short_unloading_without_pitch_does_not_trigger();
// Sustained unloading with pitch-up triggers; the action starts decision_s later.
void test_sustained_lift_with_pitch_rate_triggers_after_decision_time();
// A wheel held high with a small pitch rate still triggers through pitch_rad.
void test_high_pitch_low_rate_triggers();
// Each action follows its own switch.
void test_switches_disable_ease_and_lean_independently();
// The reaction clears after clear_s without the predicate and recovers at the rates.
void test_reaction_clears_and_recovers();
// A mid-reaction snapshot restores bitwise.
void test_reaction_state_round_trip_is_bitwise();
```

  Каждый тест строит последовательность `Percepts` с шагом 0,01 с. Пример первого:

```cpp
void test_short_unloading_without_pitch_does_not_trigger() {
    rider::FrontLiftReaction r(reaction_table());
    rider::Percepts p = calm();               // hand_press_n 120, pitch 0
    for (int i = 0; i < 30; ++i) {
        p.hand_press_n = i < 20 ? 5. : 120.;  // 0.2 s unloaded
        const auto c = r.update(p, .01);
        bt::require(!c.active && c.effort_factor == 1., "no reaction without pitch-up");
    }
}
```

- [ ] **Шаг 2.** `mise run test 'front_lift_reaction'` — падает.
- [ ] **Шаг 3. Код** по порядку выше. Все величины — из `FrontLiftReactionConfig`, без констант в коде.
- [ ] **Шаг 4.** Тест зелёный.
- [ ] **Шаг 5. Проверка этапа и коммит.** `feat(rider): whole-ride front-lift reaction with switches`.

### Задача 9. Узел плана старта

**Файлы:**
- Изменить: `native/src/rider/launch_plan.{hpp,cpp}` (класс `LaunchPlan`)
- Тест: `native/tests/reference/test_launch_plan.cpp`

**Интерфейсы:**

```cpp
namespace rider {
enum class LaunchPhase : std::uint8_t { idle, hold, release, accelerate, handover, done };
struct LaunchCommand {
    std::optional<double> effort_ceiling_nm;
    std::optional<double> lean_rad;
    double brake = 0.;                  // both wheels, [0,1]
    bool motor_off = false;             // -> motor_limit_nm = 0
    std::optional<ShiftRequest> shift_request;
    bool owns_shifting = false;
    LaunchPhase phase = LaunchPhase::idle;
    bool preload_missed = false;        // event: brake released by max_hold_s
};
class LaunchPlan {
public:
    struct State {
        LaunchPhase phase = LaunchPhase::idle;
        double phase_s = 0.;
        double launch_s = 0.;
        double effort_nm = 0.;
        double brake = 0.;
        double end_ok_s = 0.;
        double blend = 0.;              // 0 = plan, 1 = ordinary policy
        double lean_rad = 0.;
        std::optional<ShiftRequest> pending;
        bool preload_missed = false;
    };
    explicit LaunchPlan(LaunchConfig config);
    void reset();
    // ordinary = what SeatedClimbPolicy wants this tick (for the handover blend).
    [[nodiscard]] LaunchCommand update(const Percepts &p, const Intent &ordinary,
                                       bool external_effort, double dt_s);
    [[nodiscard]] State state() const;
    void restore(const State &s);
};
}
```

Переходы фаз:
- `idle → hold`: скорость ≤ `standstill_speed_mps` и каденс шатуна ≤ `standstill_crank_rad_s` (в об/мин —
  пересчитать), без внешнего нажима. При остановке по ходу то же условие; передача и шатун остаются, наклон
  берётся из `start_lean_rad(grade_ahead)`.
- `hold`: `brake = 1`, `motor_off = true`, `effort_nm += effort_rate_nm_s·dt` до предела покрытия
  `effort_cap_nm[surface_front]`.
  - Переход в `release`, когда `p.human_crank_torque_nm ≥ release_torque_nm(grade_ahead)`.
  - Или когда `phase_s ≥ max_hold_s`: тогда ещё `preload_missed = true`.
- `release`: `brake` линейно уходит к 0 за `brake_release_s`, `motor_off` — пока `brake > 0`, нажим растёт
  дальше. Потом `accelerate`.
- `accelerate`:
  - нажим растёт до предела покрытия под передним колесом;
  - каденс ≥ `upshift_crank_rpm` — `pending = up`, просьба держится, пока звезда в ощущениях не сменилась.
    Ответ «отклонено» план не видит, это ощущение звезды; просьба снимается при передаче управления;
  - `end_ok_s` копится, пока скорость ≥ `end_speed_mps` и каденс ≥ `end_crank_rpm`;
  - когда `end_ok_s ≥ end_hold_s` или `launch_s ≥ max_launch_s` — `handover`.
- `handover`:
  - сначала ждёт конца среза: звезда та же два тика подряд и просьбы нет;
  - затем `blend` растёт от 0 до 1 за `handover_s`;
  - команда — смесь `(1 - blend)·план + blend·ordinary` для нажима и наклона;
  - при `blend = 1` — `done`, `owns_shifting = false`.
- `done → hold`: снова стоим (остановка по ходу).
- `external_effort = true` (внешний нажим или поза) — план не отпускает тормоз по нажиму, не переключает и
  отдаёт `effort_ceiling_nm = nullopt`. Тормоз остаётся у внешней стороны.

- [ ] **Шаг 1. Тесты (падают)** на искусственных `Percepts`:

```cpp
// Hold: brake 1, motor off, effort ramps at effort_rate_nm_s.
void test_hold_brakes_keeps_motor_off_and_ramps_effort();
// Release by the felt crank torque, not by the requested effort.
void test_release_starts_when_the_felt_torque_reaches_the_table_value();
// The hold times out with a preload_missed event when the torque never comes.
void test_unreachable_preload_releases_by_timeout_with_an_event();
// Upshift request at upshift_crank_rpm, cleared when the felt cog changes.
void test_upshift_request_at_the_table_cadence();
// Handover waits for the shift cut, then blends continuously to the ordinary intent.
void test_handover_is_continuous_and_waits_for_the_cut();
// A stop after handover re-arms the plan without changing cog or pedal.
void test_restart_after_a_stop_rearms_the_plan();
// External effort keeps the plan from releasing brakes or shifting.
void test_external_effort_owns_the_launch();
// Mid-hold and mid-handover snapshots restore bitwise.
void test_plan_state_round_trip_is_bitwise();
```

  Пример:

```cpp
void test_release_starts_when_the_felt_torque_reaches_the_table_value() {
    auto c = normal_table();
    c.release_torque_nm = {{0.}, {25.}};
    rider::LaunchPlan plan(c);
    rider::Percepts p = standstill();         // speed 0, crank 0, hardpack
    p.human_crank_torque_nm = 24.9;
    auto cmd = plan.update(p, rider::Intent{}, false, .01);
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.brake == 1. && cmd.motor_off,
                "below the release torque the plan holds");
    p.human_crank_torque_nm = 25.;
    cmd = plan.update(p, rider::Intent{}, false, .01);
    bt::require(cmd.phase == rider::LaunchPhase::release, "the felt torque starts the release");
}
```

- [ ] **Шаг 2.** `mise run test 'launch_plan'` — падает.
- [ ] **Шаг 3. Код** по переходам выше.
- [ ] **Шаг 4.** Зелёный.
- [ ] **Шаг 5. Проверка этапа и коммит.** `feat(rider): launch plan phases`.

### Задача 10. Встраивание в `RiderIntent`; мягкий старт уходит в план

**Файлы:**
- Изменить: `native/src/rider/intent.{hpp,cpp}` (`RiderIntent` получает `Perception`, `LaunchPlan`, `FrontLiftReaction`; `resolve()` на строках `434-453`), `SeatedClimbPolicy::update` (`intent.cpp:304-336`: код мягкого старта удаляется; новый параметр `surge_allowed`), `native/src/rider/state_codec.cpp` (состояние `RiderIntent`: `perception`, `launch`, `reaction`), `native/src/configuration/*` (три поля мягкого старта удаляются из `[seated_climb]`, `equilibrium_cache.cpp:173` — убрать их из исключения), все профили и тестовые файлы (`/tmp/inv-gentle.txt`), `native/src/runtime/step.cpp:1237-1257` (`automatic_effort` передаётся в `resolve`)
- Тест: `native/tests/reference/test_seated_climb_surge.cpp` (тесты мягкого старта переносятся в `test_launch_plan.cpp` на `LaunchPlan`), `test_realistic_pedelec_acceptance.cpp` (тест 15 % остаётся и должен быть зелёным)

**Правила встраивания** (таблица «Кто чем управляет» в спецификации). На такте намерений:
1. `percepts = perception_.update(signals, preview, period)`;
2. `ordinary = policy_.update(signals, period, ..., surge_allowed = !launch_.active())` — бюджет рывка во время
   старта не тратится;
3. `launch = launch_.update(*percepts, ordinary, external_effort, period)`;
4. `reaction = reaction_.update(*percepts, period)`;
5. итог:
   - `effort = launch.effort_ceiling_nm.value_or(ordinary.effort_ceiling_nm) * reaction.effort_factor`;
   - `lean = launch.lean_rad.value_or(ordinary lean) + reaction.lean_add_rad`, обрезанный пределами позы, как
     сейчас (`lean_limit_rad`, `min_lean_rad`);
   - тормоз райдера = `launch.brake`;
   - `motor_limit_nm = min(caller, 0)`, если `launch.motor_off`;
   - `shift_request`, `rider_owns_shifting`.

Внешние поля главнее: если вызывающая сторона задала `human_torque_nm` или `posture`, их не трогаем.

Мягкий старт удаляется. Его поведение воспроизводит план:
- подготовка выбирает звезду по уклону;
- нажим растёт с `effort_rate_nm_s` = 60 / 1,5 = 40 Н·м/с — это правило «темп = предел / время» из
  спецификации;
- `standstill_crank_rad_s` = 1,0.

Признак «уклон от 3 %» уходит совсем: план работает при любом старте с места, в том числе на ровном.

- [ ] **Шаг 1. Тест (падает).** В `test_realistic_pedelec_acceptance.cpp`:

```cpp
// The launch plan holds the brakes until the felt preload, then releases:
// the bike never rolls back more than 10 mm on 15 % (spec threshold).
void test_fifteen_percent_standstill_launch_does_not_roll_back() {
    auto env = standstill_env("tests/fixtures/tracks/climb_15pct_standstill.toml");
    double min_dx = 0.;
    ride_for(env, 3., [&](const auto &row) { min_dx = std::min(min_dx, row.qpos[0] - env.start_x_m); });
    bt::require(min_dx >= -.010, "rollback stays within 10 mm");
}
```

- [ ] **Шаг 2.** `mise run test 'realistic_pedelec_acceptance'` — новый тест падает: сейчас откат 58 мм.
- [ ] **Шаг 3. Код** по правилам выше.
- [ ] **Шаг 4. Запуск.** `mise run test 'launch_plan|front_lift_reaction|rider_perception|seated_climb_surge|realistic_pedelec_acceptance|seated_pedaling_cycle'`.
  - 15 % без опрокидывания и без отката — зелёные.
  - Сидячий тест и приёмка педелека: числа старта сдвинутся. Отметить, какие проверки красные и почему.
    **Допуски не трогать.** Красные проверки — материал для задачи 11. Задача не завершается, пока они не
    объяснены. Эталоны не перезаписываются.
- [ ] **Шаг 5. Проверка этапа и коммит.** `feat(rider): launch plan and front-lift reaction in the intent resolver`.
  Если сидячий тест или приёмка красные, коммит делается только после показа пользователю и его решения:
  промежуточный красный этап недопустим.

### Задача 11. Калибровка таблиц (замер, без коммита кода)

Замер для обычного, осторожного и резкого райдера. Делается во временной копии, по образцу
`.scratch/devin/2026-11-11-launch-baseline/`; результаты пишутся на SSD.

**Выход:** таблица чисел для трёх типов и отчёт по критериям — для каждого уклона 0, 5, 10, 15, 20 % и для
старта после остановки:
- отрыв колеса (< 50 Н, > 50 мс) — да/нет и длительность;
- буксовка (> 0,9 μ, > 50 мс);
- откат;
- каденс ног (сглаживание отсечки педалирования, `pedaling.cpp:50-56`), а также пики сырых каденсов шатуна и
  вала мотора;
- время до 20 км/ч от отпускания тормоза и от начала поездки — сравнить с 3,25 с;
- окно для контроллера: от начала отрыва до начала действия реакции или до опрокидывания;
- опора седла и стоп, силы рук и мощности на каждом шаге.

Числа сверяются с проверенными данными о стартах живых велосипедистов
(`.scratch/devin/2026-11-11-launch-literature/report.md` и его сверка). Пометки: «измерено», «по статье»,
«подобрано».

- [ ] **Шаг 1.** Прогоны и отчёт.
- [ ] **Шаг 2. Остановка.** Числа и отчёт — пользователю. Без его «да» этап 5 не начинать.

---

## Этап 5. Включение

### Задача 12. Таблицы трёх типов в профилях

**Файлы:**
- Изменить: `[rider_launch]` и `[front_lift_reaction]` в `examples/research/viewer_physics_welded.toml` и в остальных исследовательских профилях (обычный райдер — утверждённые числа); `default_physics()`
- Создать: `examples/research/rider_type_careful.toml`, `examples/research/rider_type_aggressive.toml` — копии рабочего профиля, отличаются только этими двумя разделами; каждое отличие прокомментировано
- Тест: `test_welded_profile_config.cpp` — значения трёх профилей

- [ ] **Шаг 1.** Тест на значения (падает), числа — из утверждённой таблицы задачи 11.
- [ ] **Шаг 2.** Значения в профилях.
- [ ] **Шаг 3.** `mise run test 'welded_profile_config'`.
- [ ] **Шаг 4. Коммит.** `feat(profiles): calibrated launch tables for the careful, normal and aggressive riders`.

### Задача 13. Тесты поездок с утверждёнными порогами

**Файлы:**
- Изменить: `native/tests/reference/test_rider_launch_rides.cpp` (поездки), регистрация с меткой `slow`
- Изменить: `native/tests/reference/test_realistic_pedelec_acceptance.cpp:405-425` (проверки выбора звезды автоматом на старте переносятся на участок после передачи управления; для старта — проверки просьб райдера)

Общий помощник `launch_metrics(env, seconds)` возвращает:
- отрывы колеса (< 50 Н дольше 50 мс): начало и длительность;
- буксовку (`tangential / normal > 0.9 * mu` дольше 50 мс; μ берётся из метки трассы через `terrain::get_surface`
  — это диагностика, не ощущение);
- наименьшее смещение (откат);
- пик каденса ног;
- время до 20 км/ч;
- момент начала реакции;
- исход поездки.

Тесты:

```cpp
// Normal rider, 0/5/10/15 %: no lift, no wheelspin, rollback <= 10 mm,
// leg cadence <= 110 rpm; flat time to 20 km/h <= 3.25 s (stage-1 baseline).
void test_normal_rider_launches_cleanly_up_to_fifteen_percent();
// Normal rider, 20 %: no loop-out, any lift <= 0.3 s.
void test_normal_rider_twenty_percent_lift_is_short();
// Normal rider restarts after a stop on 5 and 10 %: same criteria as a first start.
void test_normal_rider_restart_after_a_stop();
// Aggressive rider, 15 and 20 %: the front lifts and the controller window
// (lift start -> earlier of reaction start or crash) is >= 0.4 s, with the
// model valid through the window.
void test_aggressive_rider_leaves_a_controller_window();
// Careful rider: clean start on 0-20 %.
void test_careful_rider_starts_clean();
// Held-out cases not used in calibration: 8 and 12 %, wet and loose labels.
void test_held_out_grades_and_surfaces();
// Every launch keeps the per-step seated limits: saddle/foot support >= -6 N,
// rising-leg press, hand forces, joint power, energy balance.
void test_launch_keeps_the_seated_limits();
```

  Пример полного теста:

```cpp
void test_normal_rider_twenty_percent_lift_is_short() {
    auto env = profile_env("examples/research/viewer_physics_welded.toml",
                           "tests/fixtures/tracks/climb_20pct_standstill.toml");
    const auto m = launch_metrics(env, 10.);
    bt::require(!m.crashed, "no loop-out on 20 %");
    for (const auto &lift : m.lifts)
        bt::require(lift.duration_s <= .3, "every lift on 20 % lasts at most 0.3 s");
}
```

  Нужные трассы с постоянным уклоном добавить в `tests/fixtures/tracks/` по образцу
  `climb_15pct_standstill.toml`. **Заменено (часть 4):** трассы заданы в коде тестов
  (`launch_ride::track`, `native/tests/reference/launch_ride_metrics.hpp`); новые трассы запрещены пунктом 4
  «Трассы» документа решений.

- [ ] **Шаг 1.** Тесты. Если пороги не выполнены, — назад к задаче 11, с пользователем. Пороги и допуски не
  меняются.
- [ ] **Шаг 2.** `mise run test 'rider_launch_rides|realistic_pedelec_acceptance|seated_pedaling_cycle'`.
- [ ] **Шаг 3. Эталоны.** Список покрасневших эталонов траекторий с причиной — пользователю. Перезапись — только
  после «да», отдельным коммитом `test(golden): re-record launch-affected trajectories`.
- [ ] **Шаг 4. Проверка этапа и коммит.** `test(rider): launch rides against the approved thresholds`.

### Задача 14. Итоговая проверка

- [ ] **Шаг 1.** Изредка — `rough_uphill_extreme` как проверка на регрессию
  (`bike-measure-realtime --out <SSD> --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_extreme.toml --duration 60 --warmup 0 --runs 1`,
  `.runs[0].prefix`). Записать в отчёт.
- [ ] **Шаг 2.** `bash tools/run_tests.sh full`. Ожидается: красных не больше известных, медленных — тоже.

---

## Этап 6. Документ решений

### Задача 15. Записи в `docs/agents/simulation-decisions.md`

- [ ] «Действует»:
  - правило старта (план, таблица опыта, типы райдеров, утверждённые пороги);
  - реакция на подъём колеса;
  - звезда и угол педали — вычисленные;
  - пункт 42 (мягкий старт) заменяется ссылкой на план.
- [ ] «Решено — не сделано», пункт 2: сделаны «ослабить» и «наклониться»; «короткое торможение задним» осталось.
- [ ] «Пробовали — не сработало»: вариант E и всё отклонённое по пути.
- [ ] Коммит `docs: record the rider launch plan`.

---

## Решения пользователя по вопросам плана (2026-11-11)

1. **Сохранение и восстановление** — по частям: состояние новых узлов и защёлка тормоза по кругу. Полное
   продолжение поездки посреди эпизода — отдельная задача, если понадобится (задача 5).
2. **Поправка наклона по точной доле нагрузки переднего колеса** (`intent.cpp:136-158`) — позже, отдельной
   задачей. В этой работе обычная езда её сохраняет.
3. **Поля мягкого старта** уходят на этапе 4 (задача 10), вместе с появлением плана старта.
4. **Выключатель «короткое торможение задним»** остаётся в таблице; `true` отклоняется, пока действие не сделано.
