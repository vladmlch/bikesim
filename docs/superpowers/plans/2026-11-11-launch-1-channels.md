# План старта, часть 1: основа и каналы — план реализации

> **Для исполнителей-агентов:** обязательный навык — `superpowers:subagent-driven-development` (рекомендуется) или
> `superpowers:executing-plans`. Шаги отмечаются флажками (`- [ ]`). Исполнитель — Devin (`devin-orchestration`);
> финальное ревью ветки делает оркестратор.

**Цель.** Подготовить каналы, через которые план старта (часть 2) будет видеть и действовать. Поведение поездок
при этом не меняется:
- ощущения райдера: сила рук на руле, скорость, звезда, метки покрытия; адаптер с задержкой;
- собственный тормоз райдера, при котором мотор честно выключен, а ноги могут давить;
- просьба райдера о переключении с ответом и прежними запретами;
- защёлка тормоза в состоянии шага.

**Архитектура.** Тормоз и просьба о переключении — **собственные команды райдера** (`rider::RiderCommands`).
Слой намерений выдаёт их изнутри шага, а не вызывающая сторона через `RideControl`. Поэтому формы команд
управления, исследований и воспроизведения не меняются, и снаружи нельзя «нажать тормоз за райдера». В части 1
команды всегда пустые: их задаёт только тестовый доступ `RuntimeRigAccess`. Наполняет их план старта в части 2.
Ощущения собираются в шаге в `rider::RiderSenses` рядом с нынешними `IntentSignals`, попадают в запись поездки и
в состояние шага.

**Стек.** C++23, MuJoCo, CTest (`bike_add_reference_test`), строгие кодеки `ncd::exact`, каталог телеметрии
`native/src/recording/catalog_data.inc`.

**Спецификация.** `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md`. Общий план работ
`docs/superpowers/plans/2026-11-11-rider-launch-plan.md` (дорожная карта; задачи 1–5 оттуда заменены этим
планом). Ревью, на которое отвечает этот план: `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan-review/review.md`.

## Общие ограничения

- Лимиты, пороги, полосы, фильтры, задержки, силы, `strength`, `pedal_torque_ripple`, запас 25 Н и допуски
  тестов не меняются.
- У полей профиля нет значений по умолчанию в коде; читателей старых форм нет.
- Мотор не меняется: его закон и `AssistController` те же. При тормозе он выключается через существующий путь
  торможения (`native/src/drivetrain/motor.cpp:59-61`: сброс состояния и 0).
- Райдер не читает точную нагрузку шин, трение, центр масс и характеристику мотора.
- В части 1 поездки, приёмка и эталоны траекторий не меняются бит в бит. Меняется только форма записей: новые
  ключи. Тестовые файлы дополняются вставкой текста, как в `5542fcfd`; выходы эталонов не перезаписываются.
- Известные красные тесты: 22 обычных (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-10-delays-t5b-check/reds.txt`) и
  10 медленных (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-09-reds-final/slow_reds.txt`). После каждой задачи список
  красных и причины их падения те же. Новый ключ в форме не считается «уже красным»: тест, который из-за него
  упал бы, правится в той же задаче.
- Коммит: автор — git-пользователь; последняя строка ровно `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`;
  без строк про Devin; без push; без `git stash`; не трогать `.claude/` и `.agents/`.

## Проверка задачи (одинакова для всех)

1. Целевые тесты задачи: `mise run test '<шаблон>'`.
2. `bash tools/check_change.sh <база задачи>`. Когда он останавливается на известных красных (`set -e` после
   `ctest`), — отдельные проходы анализаторов тем же списком файлов:

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
base=<база задачи>; list=$(mktemp)
{ git diff --name-only "$(git merge-base HEAD "$base")"; git ls-files --others --exclude-standard; } | sort -u |
  while IFS= read -r p; do case "$p" in native/*.cpp|native/*.hpp|native/*.h|native/*.cc) [[ -f "$p" ]] && echo "$PWD/$p";; esac; done > "$list"
for kind in cppcheck frontends tidy; do
  build/native-release/bin/bike-native-checks --kind "$kind" --build build/native-release --only-sources "$list"
done
```

3. Сравнить имена красных тестов с базовым списком (`comm -3`) и причины их падения с прошлым прогоном.

Итоговая проверка всей части — задача 7.

## Бюджет тестов

| Что | Время |
|---|---|
| целевые тесты одной задачи | 1–5 мин |
| `check_change` | около 5 мин |
| отдельные проходы анализаторов | около 15 мин на задачу |
| задача 7: полный прогон с этапами отдельно | около 30 мин |

Итого шесть задач по 25 мин плюс 30 мин ≈ 3 ч машинного времени.
Последняя запись `timings.tsv` по этой копии (`29dd2d1c`, `native`, остановлена на `ctest`) прохождением не
считается.

---

### Задача 1. Базовое время разгона на `f616f525` и опись

Код не меняется.

**Файлы:**
- Создать: `docs/superpowers/plans/2026-11-11-launch-baseline.md`

- [ ] **Шаг 1. Копия на `f616f525`.**

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean
git worktree add --detach /Volumes/SSD_1TB_M2/work/wt/launch-base f616f525
cd /Volumes/SSD_1TB_M2/work/wt/launch-base && bash tools/link_deps.sh && cmake --preset release && cmake --build build/native-release -j 12
```

- [ ] **Шаг 2. Замер.** Пробник и трасса — из `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch-baseline/`:
  `probe.cpp`, `physics_normal.toml`, `track_flat00_start.toml`. Собрать пробник против объектов этой копии тем же
  способом, что описан в `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.scratch/devin/2026-11-11-launch-baseline/report.md`
  (раздел «Commands run»). Прогнать ровный старт на 20 с.
  - Время до 8, 15 и 20 км/ч — от начала поездки, по скорости `qvel[root_x]`, первый шаг физики, на котором
    скорость ≥ порога.
  - Также: пик каденса шатуна, шаги и время отрыва переднего колеса (< 50 Н дольше 50 мс).
- [ ] **Шаг 3. Запись.** В файл:
  - коммит копии;
  - пути и SHA-256 `probe.cpp`, профиля и трассы;
  - формула замера;
  - числа с тремя знаками;
  - сравнение с `5542fcfd` (там 1,01 / 2,13 / 3,25 с). Если числа расходятся, причина записывается.
- [ ] **Шаг 4. Опись файлов, которые сменят форму в части 1:**

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
git grep -l -E '"signals"|"pitch_rate_up_rad_s"' -- tests native/tests > /tmp/inv-senses.txt   # StepState / bootstrap
git grep -l -E '"(cooldown_s|cut_remaining_s)"' -- tests native/tests > /tmp/inv-shifter.txt   # ShiftingSnapshot
git grep -l -E '"rollback_brake_demand"' -- tests native/tests > /tmp/inv-brake.txt              # drive telemetry
git grep -l -E '"runtime"' -- tests/fixtures/bootstrap > /tmp/inv-runtime.txt                   # StepState runtime section
```

  Для двоичных эталонов `*.bin` — те же ключи в файлах-описаниях `*.json` рядом. Список записать в файл,
  сгруппированный по виду записи.
- [ ] **Шаг 5. Остановка.** Числа и опись — пользователю. Без «да» задачу 2 не начинать.
- [ ] **Шаг 6. Коммит** `docs(plan): launch baseline at f616f525 and part-1 inventory`. Копию `launch-base` удалить
  (`git worktree remove`).

### Задача 2. Ощущения в шаге: `RiderSenses`

**Файлы:**
- Создать: `native/src/rider/senses.hpp` (типы), `native/src/rider/senses_codec.{hpp,cpp}` (кодек)
- Изменить: `native/src/runtime/step.hpp:176-195` (`StepState::senses`), `step.hpp:47-62` (`StepConfig::surface_labels`), `step.hpp:422` (новый метод `update_rider_senses`), `native/src/runtime/step.cpp:1793-1798` (вызов), `step.cpp:2720-2775` (`state`/`restore`), `native/src/runtime/config.hpp:57-82` (`RuntimeConfig::surface_labels`), `native/src/runtime/project_config.cpp:205-212` (проекция меток), `native/src/runtime/bootstrap.cpp:245-300,359-382` (чтение), `native/src/runtime/runtime.cpp:327-345` (`step_config`), `native/src/initialization/capture.cpp:208-216` (запись), `native/src/validation/runtime_access.{hpp,cpp}` (`fold_rider_senses`), каталог `native/src/recording/catalog_data.inc` (канал `rider_senses`, все три копии рядом с `front_brake_demand`), `CMakeLists.txt:142` (исходники `bike_app`)
- Тест: создать `native/tests/reference/test_rider_senses.cpp`, регистрация в `native/tests/reference/Phase3Ports.cmake` рядом с `seated_climb_surge`

**Интерфейсы** (`senses.hpp`):

```cpp
#pragma once
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace rider {
enum class SurfaceLabel : std::uint8_t { asphalt, hardpack, loose, wet, scree };
[[nodiscard]] std::optional<SurfaceLabel> surface_label(std::string_view name) noexcept;
[[nodiscard]] std::string_view surface_label_name(SurfaceLabel label) noexcept;

// What a human rider senses this interval (spec «Ощущения райдера»).
// hand_press_n: the component, along the bicycle frame's up axis, of the
// summed grip reaction ON the rider — positive when the bar pushes the hands
// up, i.e. the hands press down on the bar (rider_contacts.hpp:211-216
// convention). No tyre load, friction or COM enters here.
struct RiderSenses {
    double hand_press_n = 0.;
    double speed_mps = 0.;
    int rear_teeth = 0;
    SurfaceLabel surface_front = SurfaceLabel::hardpack;
    SurfaceLabel surface_ahead = SurfaceLabel::hardpack;
    double grade_ahead = 0.;
};

// Label-only projection of the track surface map: default label plus
// [start_m, end_m) sections (terrain/surface.cpp:55-58 semantics).
struct SurfaceLabelSection { double start_m = 0., end_m = 0.; SurfaceLabel label{}; };
struct SurfaceLabels {
    SurfaceLabel surface = SurfaceLabel::hardpack;
    std::vector<SurfaceLabelSection> sections;
    [[nodiscard]] SurfaceLabel at(double x_m) const;
};
}
```

Кодек (`senses_codec.hpp`):

```cpp
namespace rider {
[[nodiscard]] bike::io::Value encode_rider_senses(const RiderSenses &s);
[[nodiscard]] RiderSenses decode_rider_senses(const bike::io::Value &v, std::string_view path);
[[nodiscard]] bike::io::Value encode_surface_labels(const SurfaceLabels &s);
[[nodiscard]] SurfaceLabels decode_surface_labels(const bike::io::Value &v, std::string_view path);
}
```

Ключи `RiderSenses`: `hand_press_n`, `speed_mps`, `rear_teeth`, `surface_front`, `surface_ahead`, `grade_ahead`, все
обязательные (`ncd::exact`). Метки — строки из `surface_label_name`; неизвестная строка — ошибка с путём. Числа
конечные, `rear_teeth` ≥ 3.

`PhysicalStep::update_rider_senses(const WireObject &sensors_and_tires, const writers::RiderSettleOutcome &settle,
const mjModel *m, const mjData *d)`:
- `hand_press_n`: сложить `force_on_rider_n` записей `grip_left` и `grip_right` из `settle.entries` (они есть
  при `grip_attachment = "connect"`, `native/src/writers/rider_contacts.cpp:1022-1033`) и умножить скалярно на
  ось «вверх» рамы `{xmat[frame][2], [5], [8]}` (как `rider_contacts.cpp:951-952,1000-1004`). Если записей рук нет,
  значение 0.
- `speed_mps = qvel[root_x_dofadr_]`.
- `rear_teeth` — включённая звезда: `stepper_->drive()`, состояние автомата `rear_teeth`. Имя аксессора взять у
  писателя привода.
- `surface_front = config_.surface_labels.at(wheel_x_m_[0])`, `surface_ahead = ...at(wheel_x_m_[0] + config_.road_lookahead_m)`.
- `grade_ahead = road_grade_preview(road)` — то же значение, что получает слой намерений (`step.cpp:1252-1256`).

Вызов — рядом с `update_intent_signals` (`step.cpp:1793-1798`): `in.settle` уже готов (`step.cpp:1728`).
Результат хранится в `rider_senses_`, пишется в канал телеметрии `rider_senses` (все поля) и в
`StepState::senses`. В части 1 никто его не читает: поведение не меняется.

- [ ] **Шаг 1. Тесты (падают).** `native/tests/reference/test_rider_senses.cpp`:

```cpp
// RiderSenses folds what a human feels from the interval's own data: the
// hand press sign follows the rider-contacts convention (bar pushing the
// hands up is positive), labels come from the label map, and nothing from
// the tyre contacts enters (spec «Ощущения райдера»).
#include "support/fixture_test.hpp"
#include "rider/senses.hpp"
#include "rider/senses_codec.hpp"

namespace {
// Section lookup is [start_m, end_m): the end point belongs to the default.
void test_surface_labels_use_half_open_sections() {
    rider::SurfaceLabels map{.surface = rider::SurfaceLabel::asphalt,
                             .sections = {{.start_m = 10., .end_m = 20., .label = rider::SurfaceLabel::loose}}};
    bt::require(map.at(9.999) == rider::SurfaceLabel::asphalt, "before the section");
    bt::require(map.at(10.) == rider::SurfaceLabel::loose, "the start belongs to the section");
    bt::require(map.at(20.) == rider::SurfaceLabel::asphalt, "the end belongs to the default");
}
// Every one of the five labels round-trips by name; an unknown name fails.
void test_all_five_labels_round_trip_and_unknown_fails() {
    for (const auto l : {rider::SurfaceLabel::asphalt, rider::SurfaceLabel::hardpack, rider::SurfaceLabel::loose,
                         rider::SurfaceLabel::wet, rider::SurfaceLabel::scree})
        bt::require(rider::surface_label(rider::surface_label_name(l)) == l, "label round trip");
    bt::require(!rider::surface_label("ice").has_value(), "unknown label is rejected");
}
// The senses codec is exact: a missing key fails and a full record round-trips bitwise.
void test_senses_codec_is_exact_and_bitwise() {
    rider::RiderSenses s{.hand_press_n = 123.5, .speed_mps = .25, .rear_teeth = 51,
                         .surface_front = rider::SurfaceLabel::wet, .surface_ahead = rider::SurfaceLabel::scree,
                         .grade_ahead = .15};
    const auto back = rider::decode_rider_senses(rider::encode_rider_senses(s), "senses");
    bt::require(back.hand_press_n == s.hand_press_n && back.speed_mps == s.speed_mps &&
                    back.rear_teeth == 51 && back.surface_front == s.surface_front &&
                    back.surface_ahead == s.surface_ahead && back.grade_ahead == s.grade_ahead,
                "senses round-trip bitwise");
    auto v = rider::encode_rider_senses(s);
    auto &o = std::get<bike::io::Value::Object>(v.data);
    std::erase_if(o, [](const auto &kv) { return kv.first == "grade_ahead"; });
    bool threw = false;
    try { (void)rider::decode_rider_senses(v, "senses"); } catch (const std::invalid_argument &) { threw = true; }
    bt::require(threw, "a missing key is rejected");
}
}  // namespace

int main() {
    return bt::run([] {
        test_surface_labels_use_half_open_sections();
        test_all_five_labels_round_trip_and_unknown_fails();
        test_senses_codec_is_exact_and_bitwise();
    });
}
```

  И интеграционный тест в том же файле: поездка на ровной трассе с `surface_sections` (`asphalt` по умолчанию и
  `loose` на 1–3 м) по образцу окружения `environment(...)` из `test_realistic_pedelec_acceptance.cpp:84-150`
  (скопировать функцию в `native/tests/reference/launch_ride_support.hpp` и пользоваться ею в обоих файлах).
  Проверить:
  - `rider_senses.hand_press_n > 0` в покое, после успокоения;
  - `surface_front` меняется на `loose` на шаге, где `x` переднего колеса ≥ 1,0, и обратно при ≥ 3,0;
  - `surface_ahead` меняется раньше на `road_lookahead_m`;
  - траектория (`qpos` на каждом шаге) совпадает с прогоном без новых полей. Сравнивать с записью, сделанной на
    базе задачи тем же тестом. Так проверяется, что поведение не изменилось.
  Общий заголовок тестов `native/tests/reference/launch_ride_support.hpp` (используют задачи 2, 4, 6):

```cpp
#pragma once
// Shared launch-test environment: the welded research profile on a given
// track, record_decimation 1, initial speed 0 — the realistic_pedelec
// acceptance environment (test_realistic_pedelec_acceptance.cpp:84-150),
// moved here so the channel tests reuse it verbatim.
#include "support/fixture_test.hpp"
#include "configuration/load.hpp"
#include "model/rider_geometry.hpp"
#include "research/environment.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"

namespace launch_support {
namespace sw = runtime::sample_wire;
[[nodiscard]] std::filesystem::path repo_file(std::string_view relative);   // as in the pedelec test
struct Env {
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t control_steps = 0, max_steps = 0;
    int root_x_dof = -1, root_x_qpos = -1;
};
[[nodiscard]] Env environment(bike::terrain::TrackSpec track, double duration_s);  // body: pedelec env, same fields
[[nodiscard]] inline const runtime::WireObject &section(const runtime::WireObject &o, std::string_view key) {
    return sw::object(sw::required(o, key));
}
[[nodiscard]] inline double number(const runtime::WireObject &o, std::string_view key) {
    return sw::number(sw::required(o, key));
}
[[nodiscard]] inline bool boolean(const runtime::WireObject &o, std::string_view key) {
    return sw::boolean(sw::required(o, key));
}
[[nodiscard]] inline double root_x(const Env &env) {
    return env.runtime->snapshot().qpos[static_cast<std::size_t>(env.root_x_qpos)];
}
}  // namespace launch_support
```

  Тела `repo_file` и `environment` переносятся из `test_realistic_pedelec_acceptance.cpp:63-66,84-150` без
  изменений, только без полей кассеты. Сам тест педелека переходит на этот заголовок: его поведение не меняется.
  Имена `sw::boolean`, `snapshot().qpos` сверить с `runtime/samples.hpp` и `runtime/runtime.hpp`; если
  называются иначе, взять настоящие.
- [ ] **Шаг 2.** `mise run test 'rider_senses'` — не собирается.
- [ ] **Шаг 3. Код** по интерфейсам выше:
  - `SurfaceLabels::at` повторяет `SurfaceMap::at` (`terrain/surface.cpp:55-58`);
  - проекция в `project_config.cpp` рядом с `surface_map` строит `SurfaceLabels` из `track.surface` и
    `track.surface_sections` всегда, при любом `surface_mode`;
  - `RuntimeConfig::surface_labels` — документ (`bike::io::Value`, как `rider_intent`); `decode_runtime_config`
    принимает ключ `surface_labels` (`bootstrap.cpp:245-254`), `step_config()` его декодирует
    (`runtime.cpp:327-345`);
  - `StepState::senses` пишет `capture.cpp` (ключ `senses` рядом с `signals`), а читает `bootstrap.cpp:359,382`.
- [ ] **Шаг 4. Тестовые файлы** из `/tmp/inv-senses.txt` и `/tmp/inv-runtime.txt` получают ключи `senses` (нули,
  `rear_teeth` — звезда профиля, `surface_*` — метка трассы файла) и `surface_labels` (метка трассы, участков
  нет). Двоичные эталоны — как в `5542fcfd`: меняются входы, выходы не трогаются.
- [ ] **Шаг 5.** `mise run test 'rider_senses|io_state_codecs|native_runtime|realistic_pedelec_acceptance|seated_climb_surge'`.
- [ ] **Шаг 6.** Проверка задачи (база — HEAD задачи 1). Коммит `feat(runtime): human-sensed rider senses channel`.

### Задача 3. Адаптер ощущений с задержкой

**Файлы:**
- Создать: `native/src/rider/perception.{hpp,cpp}`
- Изменить: `native/src/rider/senses_codec.{hpp,cpp}` (кодек состояния адаптера), `CMakeLists.txt:142`
- Тест: создать `native/tests/reference/test_rider_perception.cpp` и зарегистрировать его

**Интерфейсы:**

```cpp
namespace rider {
// One acquisition the rider perceives later: the intent signals plus the
// senses of the same interval, time-aligned (grade included).
struct SenseSample { IntentSignals signals; RiderSenses senses; };
struct Percepts {
    RiderSenses senses;           // delayed, including grade_ahead
    double crank_rpm = 0.;        // |crank_rate_rad_s| * 60 / 2pi, delayed
    double human_crank_torque_nm = 0.;
    double pitch_rate_up_rad_s = 0.;
    double pitch_rad = 0.;        // integrated from the delayed rate since reset
};
class Perception {
public:
    struct State {
        double time_s = 0.;
        std::deque<std::pair<double, SenseSample>> samples;
        std::optional<SenseSample> delayed;
        double pitch_rad = 0.;
    };
    // reaction_delay_s must be a nonnegative multiple of period_s (intent.cpp:382-389 rule).
    Perception(double reaction_delay_s, double period_s);
    void reset();
    // Called once per intention period. Returns nullopt until the first
    // sample is reaction_delay_s old.
    [[nodiscard]] std::optional<Percepts> update(const SenseSample &sample);
    [[nodiscard]] State state() const;
    void restore(const State &state);   // validates first, commits atomically
private:
    double delay_s_, period_s_;
    State s_;
};
[[nodiscard]] bike::io::Value encode_perception_state(const Perception::State &s);
[[nodiscard]] Perception::State decode_perception_state(const bike::io::Value &v, std::string_view path);
}
```

- [ ] **Шаг 1. Тесты (падают).** `native/tests/reference/test_rider_perception.cpp`:

```cpp
// The rider perceives every sensation with the same reaction delay — grade
// included — and integrates pitch from the delayed vestibular rate. The
// state survives serialize -> read -> continue bitwise (user ruling
// 2026-11-11: component-level restore).
#include "support/fixture_test.hpp"
#include "rider/perception.hpp"
#include <cmath>
#include <numbers>

namespace {
rider::SenseSample sample(int k) {
    rider::SenseSample s;
    s.signals.pitch_rate_up_rad_s = .5;
    s.signals.crank_rate_rad_s = 2. * std::numbers::pi;   // 60 rpm
    s.signals.human_crank_torque_nm = static_cast<double>(k);
    s.senses = {.hand_press_n = 100. + k, .speed_mps = .01 * k, .rear_teeth = 51,
                .surface_front = rider::SurfaceLabel::asphalt, .surface_ahead = rider::SurfaceLabel::loose,
                .grade_ahead = .001 * k};
    return s;
}
// Nothing arrives inside the delay; afterwards every field, grade included,
// is the sample exactly one delay earlier.
void test_every_sensation_is_delayed_together() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 15; ++k)
        bt::require(!p.update(sample(k)).has_value(), "nothing inside the reaction delay");
    const auto first = p.update(sample(15));
    bt::require(first.has_value(), "the first percept arrives at the delay");
    bt::require(first->human_crank_torque_nm == 0. && first->senses.hand_press_n == 100. &&
                    first->senses.grade_ahead == 0.,
                "torque, hands and grade are the sample from one delay earlier");
    bt::require(std::abs(first->crank_rpm - 60.) < 1e-9, "crank rate in rpm");
}
// pitch_rad integrates only delayed rates: 21 delayed ticks of .5 rad/s.
void test_pitch_integrates_the_delayed_rate() {
    rider::Perception p(.15, .01);
    std::optional<rider::Percepts> last;
    for (int k = 0; k <= 35; ++k) last = p.update(sample(k));
    bt::require(last.has_value() && std::abs(last->pitch_rad - 21 * .5 * .01) < 1e-12,
                "pitch integrates the delayed rate");
}
// Serialize mid-delay, read back into a fresh adapter, continue: identical.
void test_state_serialize_read_continue_is_bitwise() {
    rider::Perception a(.15, .01);
    for (int k = 0; k < 20; ++k) (void)a.update(sample(k));
    rider::Perception b(.15, .01);
    b.restore(rider::decode_perception_state(rider::encode_perception_state(a.state()), "perception"));
    for (int k = 20; k < 60; ++k) {
        const auto x = a.update(sample(k));
        const auto y = b.update(sample(k));
        bt::require(x.has_value() == y.has_value(), "same availability");
        if (x)
            bt::require(x->pitch_rad == y->pitch_rad && x->senses.grade_ahead == y->senses.grade_ahead &&
                            x->human_crank_torque_nm == y->human_crank_torque_nm &&
                            x->senses.hand_press_n == y->senses.hand_press_n,
                        "restored adapter continues bitwise");
    }
}
// A delay that is not on the intention grid is rejected at construction.
void test_off_grid_delay_is_rejected() {
    bool threw = false;
    try { rider::Perception p(.155, .01); } catch (const std::invalid_argument &) { threw = true; }
    bt::require(threw, "off-grid delay rejected");
}
// A malformed restored state (a sample timed after the clock) is rejected
// and leaves the adapter unchanged.
void test_malformed_state_is_rejected_atomically() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 5; ++k) (void)p.update(sample(k));
    auto bad = p.state();
    bad.samples.emplace_back(bad.time_s + 1., sample(99));
    const auto before = rider::encode_perception_state(p.state());
    bool threw = false;
    try { p.restore(bad); } catch (const std::invalid_argument &) { threw = true; }
    bt::require(threw && rider::encode_perception_state(p.state()) == before, "rejected atomically");
}
}  // namespace

int main() {
    return bt::run([] {
        test_every_sensation_is_delayed_together();
        test_pitch_integrates_the_delayed_rate();
        test_state_serialize_read_continue_is_bitwise();
        test_off_grid_delay_is_rejected();
        test_malformed_state_is_rejected_atomically();
    });
}
```

  Если у `bike::io::Value` нет `operator==`, сравнивать `bike::io::canonical_json(...)`.
- [ ] **Шаг 2.** `mise run test 'rider_perception'` — не собирается.
- [ ] **Шаг 3. Код.**

```cpp
#include "perception.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace rider {
Perception::Perception(double reaction_delay_s, double period_s)
    : delay_s_(reaction_delay_s), period_s_(period_s) {
    if (!std::isfinite(period_s_) || period_s_ <= 0.)
        throw std::invalid_argument("perception period must be finite and positive");
    const double ticks = std::rint(delay_s_ / period_s_);
    if (!std::isfinite(delay_s_) || delay_s_ < 0. || std::abs(ticks * period_s_ - delay_s_) > 1e-9)
        throw std::invalid_argument("perception reaction delay must be a nonnegative multiple of the period");
    reset();
}
void Perception::reset() { s_ = {}; }
std::optional<Percepts> Perception::update(const SenseSample &sample) {
    s_.samples.emplace_back(s_.time_s, sample);
    const double ready = s_.time_s - delay_s_;
    while (!s_.samples.empty() && s_.samples.front().first <= ready + 1e-12) {
        s_.delayed = s_.samples.front().second;
        s_.samples.pop_front();
    }
    s_.time_s += period_s_;
    if (!s_.delayed) return std::nullopt;
    const SenseSample &d = *s_.delayed;
    s_.pitch_rad += d.signals.pitch_rate_up_rad_s * period_s_;
    return Percepts{.senses = d.senses,
                    .crank_rpm = std::abs(d.signals.crank_rate_rad_s) * 60. / (2. * std::numbers::pi),
                    .human_crank_torque_nm = d.signals.human_crank_torque_nm,
                    .pitch_rate_up_rad_s = d.signals.pitch_rate_up_rad_s,
                    .pitch_rad = s_.pitch_rad};
}
Perception::State Perception::state() const { return s_; }
void Perception::restore(const State &state) {
    if (!std::isfinite(state.time_s) || state.time_s < 0. || !std::isfinite(state.pitch_rad))
        throw std::invalid_argument("perception state clocks must be finite");
    for (const auto &[t, sample] : state.samples)
        if (!std::isfinite(t) || t > state.time_s + 1e-12)
            throw std::invalid_argument("perception sample is timed after the clock");
    s_ = state;
}
}  // namespace rider
```

  Кодек состояния: ключи `time_s`, `samples` (массив пар `[t, {signals, senses}]`), `delayed` (или `null`),
  `pitch_rad`. Сигналы — через существующие `encode_intent_signals`/`decode_signals`
  (`native/src/rider/state_codec.cpp:963-969,2296-2310`), ощущения — через кодек задачи 2.
- [ ] **Шаг 4.** `mise run test 'rider_perception'` — зелёный.
- [ ] **Шаг 5.** Проверка задачи. Коммит `feat(rider): perception adapter with one reaction delay`.

### Задача 4. Тормоз райдера: мотор выключен, ноги давят

**Файлы:**
- Изменить: `native/src/rider/intent.hpp` (новый `RiderCommands`; `RiderIntent::commands()`), `native/src/runtime/step.hpp` (член `rider_commands_override_` только для тестового доступа; метод `rider_commands()`), `native/src/runtime/step.cpp:1259-1272` (слияние в `apply_forces_body`), `step.cpp:1365-1380` (`TickInputs.braking` = торможение мотора), `step.cpp:1602-1621` и `1800-1815` (`advance_body`: итоговый тормоз для `solve_tail`, телеметрии и равновесия), каталог телеметрии (столбцы `rider_front_brake_demand`, `rider_rear_brake_demand`, `motor_braking`), `native/src/runtime/encode.cpp:1425-1432,1738-1748` (мост применённых команд: новые столбцы привода), `native/src/validation/runtime_access.{hpp,cpp}` (`set_rider_commands`)
- Тест: `native/tests/reference/test_rider_brake_channel.cpp` (новый), `native/tests/reference/test_assist_controller.cpp` (без изменений — только запуск)

**Интерфейсы:**

```cpp
// intent.hpp
namespace rider {
enum class ShiftRequest : std::uint8_t { up, down };   // up = one cog smaller (harder)
// The rider's own commands — produced inside the step by the intent layer,
// never by the caller (spec «Кто чем управляет»). Part 1: always empty
// unless a test sets them through RuntimeRigAccess.
struct RiderCommands {
    double front_brake = 0.;              // [0, 1]
    double rear_brake = 0.;               // [0, 1]
    std::optional<ShiftRequest> shift_request;
    bool owns_shifting = false;
};
void validate(const RiderCommands &c);    // finite, brakes in [0, 1]
}
// RiderIntent: [[nodiscard]] const RiderCommands &commands() const;  // part 1: default-constructed
// runtime_access.hpp:
//   static void set_rider_commands(runtime::NativeRideRuntime&, std::optional<::rider::RiderCommands>);
```

**Правила** (ревью C1 и I1):
- Тормоз райдера `rf, rr` = `rider_commands_override_` в тестах, иначе `rider_intent_.commands()`.
- В `apply_forces_body`, после расчёта `braking` по внешней команде (`step.cpp:1261-1262`), идут три шага:

```cpp
const rider::RiderCommands &rider_cmd = rider_commands();
const bool rider_braking = rider_cmd.front_brake > 0. || rider_cmd.rear_brake > 0.;
// The motor brakes on the caller's braking (as before) and on the rider's
// own brake; the legs keep the caller-only predicate, so a rider's launch
// hold lets them press (spec «Нажим при зажатом тормозе»). The rollback
// hold still does not cut the motor — unchanged since `braking` is
// computed before the rollback merge (step.cpp:1602).
const bool motor_braking = braking || rider_braking;
front = std::max(front, rider_cmd.front_brake);
rear = std::max(rear, rider_cmd.rear_brake);
```

  Дальше `brake_.apply(m, front, rear)` и `hold_qfrc(...)` берут уже итоговые значения.
  `stepper_->drive().prepare(..., braking, ...)` (ноги) остаётся с `braking`, а
  `TickInputs{.braking = motor_braking}` (мотор и `AssistController::step`,
  `native/src/writers/drivetrain.cpp:839,847`) получает новый признак. Так мотор при тормозе райдера уходит в
  существующий путь торможения (`motor.cpp:59-61`): состояние и задержка сбрасываются, момент 0.
- В `advance_body` после `apply_forces(...)` (`step.cpp:1621`) повторить слияние с той же `rider_commands()`:
  `front = max(front, rf)`, `rear = max(rear, rr)`. Эти значения уходят в `solve_tail` (`step.cpp:1668-1685`), в
  телеметрию `front_brake_demand`/`rear_brake_demand` (`step.cpp:1810-1811`) и в `balance_channel`
  (`step.cpp:1972`). Добавить столбцы `rider_front_brake_demand`, `rider_rear_brake_demand` и `motor_braking`.
- Внешний `RideControl` не меняется. Пробный путь (`probe_step_inputs`) берёт ту же `rider_commands()`.
- **Четыре признака из одного результата** (ревью части 1, P6; решение пользователя 2026-11-11). В
  `apply_forces_body` собрать одну структуру и передавать её дальше. Не вычислять признаки заново в разных местах:

```cpp
// One resolved braking result per interval (spec «Нажим при зажатом тормозе»).
struct BrakeResolution {
    double caller_front = 0., caller_rear = 0., rollback = 0., rider_front = 0., rider_rear = 0.;
    double final_front = 0., final_rear = 0.;   // max of the three sources per wheel
    bool legs_coast = false;      // caller braking only (as before): the legs coast
    bool motor_braking = false;   // caller braking OR any rider brake > 0 (rollback alone: as before, no)
    bool shift_blocked = false;   // caller braking OR any rider brake > 0: no shift while a brake is held
    bool riding = false;          // balance monitor: as before but on final demands AND no rider brake > 0
};
```

  - Пустые команды райдера дают ровно прежние значения: `legs_coast = motor_braking = shift_blocked = braking`,
    `riding` = прежнее выражение `balance_channel` (`step.cpp:2569-2579`). Тест сравнивает это поле за полем
    на всех шагах поездки.
  - `shift_blocked` идёт в автомат вместо `in.braking` (`writers/drivetrain.cpp:683`). Значит, при зажатом
    тормозе райдера просьба о переключении получает «отложено».
  - `riding` идёт в `balance_monitor_.update(...)`: часы равновесия не стартуют, пока тормоз райдера хоть
    немного зажат.
  - Тесты: частичный тормоз райдера (0,5); наложение внешнего тормоза и тормоза райдера; удержание от отката
    вместе с тормозом райдера; тормоз райдера вместе с просьбой о переключении (ответ «отложено»).
- **Запись команд райдера** (ревью части 1, P4; решение пользователя 2026-11-11). Каждый интервал пишет в канал
  `rider_commands`:
  - `front_brake`, `rear_brake`, `shift_request` (`none|up|down`), `owns_shifting`;
  - источник (`intent` или `test_override`);
  - признаки `BrakeResolution`.

  Время интервала уже есть в записи. Так контроллер тяги и разбор поездки видят, когда райдер начал действовать.
  - При воспроизведении команды райдера пересчитываются из состояния, а не подставляются из записи.
    Проверка воспроизведения сравнивает канал `rider_commands` записи с пересчитанным, рядом с тем местом, где
    сравниваются применённые команды (`native/src/replay/session.cpp:105-119,203-208`).
  - В режиме исследований команды райдера не идут через очередь задержки мотора
    (`native/src/runtime/research.cpp:316-332`): они рождаются внутри шага. Внешние команды и ограничения мотора
    сохраняют свою очередь и приоритет. Тест: те же входы в обычном режиме и в режиме исследований дают один
    и тот же канал `rider_commands` по шагам.

- Удержание от отката мотор по-прежнему не выключает: `braking` считается до слияния (`step.cpp:1602`). В
  отчёте это записывается как известное: «так было и раньше». Менять — только решением пользователя.

- [ ] **Шаг 1. Тесты (падают).** `native/tests/reference/test_rider_brake_channel.cpp` (окружение из
  `launch_ride_support.hpp`, задача 2):

```cpp
// The rider's own brake: the legs press while the wheels are held, the
// motor goes through its braking path (state and run-on reset, zero torque),
// the recorded final demands equal what the wheels got, and ordinary caller
// braking still cuts the legs. Caller RideControl is unchanged.
#include "launch_ride_support.hpp"

namespace {
void advance_s(launch_support::Env &env, double seconds, const runtime::RideControl &c,
               double front = 0., double rear = 0.) {
    const auto steps = static_cast<std::int64_t>(std::llround(seconds / .00125));
    const auto target = env.runtime->snapshot().step + steps;
    while (env.runtime->snapshot().step < target)
        (void)env.runtime->advance(env.runtime->snapshot().step + env.control_steps, c, front, rear, std::nullopt);
}
// Hold: legs press, motor zero, bike within 10 mm, final demand recorded = 1.
void test_rider_brake_holds_while_the_legs_press_and_the_motor_is_off() {
    auto env = launch_support::environment(
        terrain::load_track(launch_support::repo_file("tests/fixtures/tracks/climb_15pct_standstill.toml")), 3.);
    bike::validation::RuntimeRigAccess::set_rider_commands(*env.runtime, rider::RiderCommands{.front_brake = 1., .rear_brake = 1.});
    runtime::RideControl c{};
    c.human_torque_nm = 30.;
    const double x0 = launch_support::root_x(env);
    advance_s(env, .5, c);
    const auto &row = *env.runtime->recorded_intervals().back();
    const auto &drive = launch_support::section(row.channels, "drive");
    bt::require(launch_support::number(drive, "human_sensor_nm") > 20., "legs press against the held crank");
    bt::require(launch_support::number(drive, "motor_torque_nm") == 0., "motor stays off");
    bt::require(launch_support::boolean(drive, "motor_braking"), "the motor sees braking");
    bt::require(launch_support::number(drive, "front_brake_demand") == 1. &&
                    launch_support::number(drive, "rider_front_brake_demand") == 1.,
                "the recorded final demand equals the applied rider brake");
    bt::require(std::abs(launch_support::root_x(env) - x0) <= .010, "the bike holds within 10 mm");
}
// After the rider brake is released the motor starts from zero through its
// usual lag — no torque stored during the hold appears at once.
void test_motor_starts_from_zero_after_the_rider_brake() {
    auto env = launch_support::environment(
        terrain::load_track(launch_support::repo_file("tests/fixtures/tracks/climb_15pct_standstill.toml")), 3.);
    bike::validation::RuntimeRigAccess::set_rider_commands(*env.runtime, rider::RiderCommands{.front_brake = 1., .rear_brake = 1.});
    runtime::RideControl c{};
    c.human_torque_nm = 30.;
    advance_s(env, .5, c);
    bike::validation::RuntimeRigAccess::set_rider_commands(*env.runtime, std::nullopt);
    advance_s(env, .00125, c);
    const auto &drive = launch_support::section(env.runtime->recorded_intervals().back()->channels, "drive");
    bt::require(launch_support::number(drive, "motor_torque_nm") < 5., "first released tick: no stored assist burst");
}
// Ordinary caller braking still cuts the legs (unchanged behaviour).
void test_caller_braking_still_cuts_the_legs() {
    auto env = launch_support::environment(
        terrain::load_track(launch_support::repo_file("tests/fixtures/tracks/climb_15pct_standstill.toml")), 3.);
    runtime::RideControl c{};
    c.human_torque_nm = 30.;
    advance_s(env, .5, c, 1., 1.);
    const auto &drive = launch_support::section(env.runtime->recorded_intervals().back()->channels, "drive");
    bt::require(launch_support::number(drive, "human_sensor_nm") < 1., "caller braking coasts the legs");
}
// An out-of-range rider brake is rejected before it reaches the physics.
void test_rider_brake_out_of_range_is_rejected() {
    bool threw = false;
    try { rider::validate(rider::RiderCommands{.front_brake = 1.5}); } catch (const std::invalid_argument &) { threw = true; }
    bt::require(threw, "brake above 1 rejected");
}
}  // namespace

int main() {
    return bt::run([] {
        test_rider_brake_holds_while_the_legs_press_and_the_motor_is_off();
        test_motor_starts_from_zero_after_the_rider_brake();
        test_caller_braking_still_cuts_the_legs();
        test_rider_brake_out_of_range_is_rejected();
    });
}
```

  Помощники `launch_support::environment`, `repo_file`, `section`, `number`, `boolean`, `root_x` — в
  `launch_ride_support.hpp`. `environment` — копия `test_realistic_pedelec_acceptance.cpp:84-150` с теми же
  полями `PedelecEnv`. `root_x(env)` читает `snapshot().qpos[env.root_x_qpos]`. Имена столбцов `human_sensor_nm`
  и `motor_torque_nm` сверить с каталогом телеметрии; если там другие, взять настоящие и поправить тест.
- [ ] **Шаг 2.** `mise run test 'rider_brake_channel'` — не собирается.
- [ ] **Шаг 3. Код** по правилам выше.
- [ ] **Шаг 4. Поведение не меняется.**
  `mise run test 'assist_controller|realistic_pedelec_acceptance|seated_pedaling_cycle|native_runtime|pedelec_replay_contract'`
  — те же результаты, что на базе: команды райдера пустые, а `motor_braking` при них = `braking`.
- [ ] **Шаг 5. Тестовые файлы** из `/tmp/inv-brake.txt` получают три новых столбца (нули и `false`).
- [ ] **Шаг 6.** Проверка задачи. Коммит `feat(runtime): rider-owned brake channel with motor braking`.

### Задача 5. Просьба райдера о переключении

**Файлы:**
- Изменить: `native/src/drivetrain/shifting.hpp:122-124`, `native/src/drivetrain/shifting.cpp:21-104`, `native/src/config_types.hpp:69-74` (`ShiftingSnapshot`), кодек снимка автомата (`native/src/drivetrain/state_codec.cpp`), `native/src/writers/drivetrain.hpp:119-124` (`PrepareInputs`), `native/src/writers/drivetrain.cpp:667-683`, `native/src/runtime/step.cpp:1283-1295` (передача команд райдера), каталог телеметрии (`drive.shift_rider_response`)
- Тест: `native/tests/reference/test_cadence_shifter.cpp`

**Интерфейсы:**

```cpp
// config_types.hpp — ShiftingSnapshot gains (required on the wire):
std::string rider_response = "none";   // "none" | "accepted" | "deferred" | "rejected"
// shifting.hpp
struct RiderShift { bool owns = false; std::optional<ShiftDirection> request; };
[[nodiscard]] bool update(double cadence_rpm, double required_cadence_rpm, double dt,
                          bool pedaling, bool braking, bool rear_in_contact,
                          std::optional<double> rear_slip_mps, RiderShift rider);
// writers/drivetrain.hpp — PrepareInputs gains: drivetrain::RiderShift rider_shift;
```

Старая перегрузка `update(...)` остаётся и вызывает новую с `RiderShift{}`.

**Логика** (ревью I3): общая проверка допуска и общий «коммит» переключения для обеих веток.

```cpp
bool CadenceShifter::update(double cadence, double required, double dt, bool pedaling, bool braking,
                            bool contact, std::optional<double> slip, RiderShift rider) {
    finite(cadence, "shift cadence");
    finite(required, "wheel-required shift cadence");
    positive(dt, "shift interval");
    finite_optional(slip, "rear tire slip");
    auto next = state_;
    next.rider_response = "none";
    const auto finish = [&](bool shifted) { state_ = std::move(next); return shifted; };
    // smoothing and clocks exactly as now (shifting.cpp:32-42)
    /* ...existing EMA and clock update lines, unchanged... */
    const bool admitted = config_.enabled && pedaling && !braking && contact &&
                          next.cooldown_s <= 1e-12 && std::min(cadence, required) >= 0.;
    if (rider.owns) {
        if (!rider.request) return finish(false);
        if (!admitted || next.cut_remaining_s > 1e-12) { next.rider_response = "deferred"; return finish(false); }
        const auto it = std::ranges::find(config_.cassette, next.rear_teeth);
        if (it == config_.cassette.end())
            throw std::logic_error("CadenceShifter: engaged cog is not on the cassette");
        const bool up = *rider.request == ShiftDirection::up;
        if ((up && it == config_.cassette.begin()) || (!up && std::next(it) == config_.cassette.end())) {
            next.rider_response = "rejected"; return finish(false);
        }
        const int selected = up ? *std::prev(it) : *std::next(it);
        if (!commit_checked(next, selected, *rider.request, slip)) {   // slip + landing vetoes, same numbers
            next.rider_response = "rejected"; return finish(false);
        }
        next.rider_response = "accepted";
        return finish(true);
    }
    if (!admitted) return finish(false);
    /* ...existing band selection (shifting.cpp:45-86), then: */
    return finish(commit_checked(next, selected, direction, slip));
}
```

`commit_checked(ShiftingSnapshot &next, int selected, ShiftDirection direction, std::optional<double> slip)` —
новый закрытый метод. Он переносит без изменений строки `shifting.cpp:90-103`: режим проскальзывания, запрет
по проскальзыванию на переключение вверх, запрет по каденсу посадки, проверку переполнения `shift_count` и
запись нового состояния. Возвращает `false`, если сработал запрет. Строка с `/* ...existing... */` в
исполнении заменяется настоящими строками `shifting.cpp:32-42` и `45-86`, перенесёнными как есть.

Писатель привода: `shifting_scratch_.update(..., in.slip, in.rider_shift)` (`drivetrain.cpp:681-683`). Шаг
заполняет `rider_shift` из `rider_commands()`: `owns_shifting`, `shift_request` — `up`/`down` в `ShiftDirection`.
В телеметрию привода добавить `shift_rider_response` (строка).

- [ ] **Шаг 1. Тесты (падают)** в `test_cadence_shifter.cpp`:

```cpp
// Rider-owned: the band never selects; an up request moves one cog with the
// usual cooldown and cut.
void test_rider_up_request_moves_one_cog_with_cooldown_and_cut() {
    auto s = shifter();   // 28T
    const drivetrain::RiderShift owns{.owns = true};
    bt::require(!s.update(130., 130., .01, true, false, true, std::nullopt, owns), "no band shift while the rider owns");
    bt::require(s.update(90., 90., .01, true, false, true, std::nullopt,
                         {.owns = true, .request = drivetrain::ShiftDirection::up}),
                "the up request shifts");
    bt::require(s.state().rear_teeth == 24 && s.state().rider_response == "accepted" &&
                    s.state().cut_remaining_s > 0. && s.state().cooldown_s > 0.,
                "one cog smaller, cut and cooldown armed");
}
// Inside the cooldown or cut the request is deferred.
void test_rider_request_inside_cooldown_is_deferred() {
    auto s = shifter();
    (void)s.update(90., 90., .01, true, false, true, std::nullopt, {.owns = true, .request = drivetrain::ShiftDirection::up});
    bt::require(!s.update(90., 90., .01, true, false, true, std::nullopt,
                          {.owns = true, .request = drivetrain::ShiftDirection::up}),
                "no second shift inside the cooldown");
    bt::require(s.state().rider_response == "deferred", "deferred");
}
// The cassette end rejects.
void test_rider_request_past_the_cassette_end_is_rejected() {
    auto s = shifter(51);
    bt::require(!s.update(60., 60., .01, true, false, true, std::nullopt,
                          {.owns = true, .request = drivetrain::ShiftDirection::down}),
                "no cog larger than 51");
    bt::require(s.state().rider_response == "rejected" && s.state().rear_teeth == 51, "rejected, gear kept");
}
// The existing vetoes stay: no pedaling, braking, no contact and reverse cadence defer.
void test_rider_request_keeps_the_admission_vetoes() {
    const drivetrain::RiderShift up{.owns = true, .request = drivetrain::ShiftDirection::up};
    {
        auto s = shifter();
        bt::require(!s.update(90., 90., .01, false, false, true, std::nullopt, up), "not pedaling");
    }
    {
        auto s = shifter();
        bt::require(!s.update(90., 90., .01, true, true, true, std::nullopt, up), "braking");
    }
    {
        auto s = shifter();
        bt::require(!s.update(90., 90., .01, true, false, false, std::nullopt, up), "no contact");
    }
    {
        auto s = shifter();
        bt::require(!s.update(-10., 90., .01, true, false, true, std::nullopt, up), "reverse cadence");
        bt::require(s.state().rider_response == "deferred" && s.state().rear_teeth == 28, "gear kept");
    }
}
// Slip and landing vetoes reject with the existing numbers.
void test_rider_request_keeps_the_slip_and_landing_vetoes() {
    {
        auto s = shifter();
        bt::require(!s.update(90., 90., .01, true, false, true, 1.0,
                              {.owns = true, .request = drivetrain::ShiftDirection::up}),
                    "slip above .5 m/s vetoes an upshift");
        bt::require(s.state().rider_response == "rejected", "slip rejection");
    }
    {
        auto s = shifter();
        // landing 70 * 24/28 = 60 < 75: the upshift would land below the band
        bt::require(!s.update(70., 70., .01, true, false, true, std::nullopt,
                              {.owns = true, .request = drivetrain::ShiftDirection::up}),
                    "landing below the band vetoes");
        bt::require(s.state().rider_response == "rejected", "landing rejection");
    }
}
// Not owned: behaviour and response are exactly as before.
void test_automatic_path_reports_no_rider_response() {
    auto s = shifter();
    bt::require(s.update(115., 115., .01), "automatic upshift as before");
    bt::require(s.state().rider_response == "none", "no rider response on the automatic path");
}
```

  И добавить вызовы в `main()` (`test_cadence_shifter.cpp:133`).
- [ ] **Шаг 2.** `mise run test 'cadence_shifter'` — не собирается.
- [ ] **Шаг 3. Код** по логике выше; кодек снимка; телеметрия; тестовые файлы из `/tmp/inv-shifter.txt`
  получают `"rider_response": "none"`.
- [ ] **Шаг 4.** `mise run test 'cadence_shifter|realistic_pedelec_acceptance|native_drivetrain|seated_pedaling_cycle'`
  — поведение автомата прежнее.
- [ ] **Шаг 5.** Проверка задачи. Коммит `feat(drivetrain): rider shift requests sharing the automatic vetoes`.

### Задача 6. Защёлка тормоза в состоянии шага

**Файлы:**
- Изменить: `native/src/runtime/static_brake.hpp:64-92`, `native/src/runtime/static_brake.cpp`, `native/src/runtime/step.hpp:176-195`, `native/src/runtime/step.cpp:2720-2775`, `native/src/initialization/capture.cpp:87-122,208-216` (запись `runtime.brake_latch`), `native/src/runtime/bootstrap.cpp:375-390` (чтение в `exact`-набор раздела `runtime`), `native/src/runtime/runtime.cpp:245-261` (`decode_step_state`)
- Тест: `native/tests/reference/test_static_brake_latch.cpp` (новый)

**Интерфейсы:**

```cpp
// static_brake.hpp (class StaticBrake)
struct State { std::optional<double> latch_front, latch_rear; };
[[nodiscard]] State state() const;   // {latch_front_, latch_rear_}
void restore(const State &s);        // finite or empty; commits both or throws
// step.hpp — StepState gains: StaticBrake::State brake_latch;
```

- `PhysicalStep::state()` пишет `brake_latch = brake_.state()`.
- `PhysicalStep::restore()` для нового эпизода по-прежнему вызывает `brake_.reset()` (`step.cpp:2756-2758`);
  `brake_latch` из состояния там не применяется.
- Запись — ключ `brake_latch` в разделе `runtime` захвата: `{"front": число|null, "rear": число|null}`.

- [ ] **Шаг 1. Тест (падает).** `native/tests/reference/test_static_brake_latch.cpp`:

```cpp
// The static brake latch round-trips and continues: a StaticBrake restored
// from another brake's state produces the identical held force on the next
// call (component-level continuation, user ruling 2026-11-11).
#include "launch_ride_support.hpp"
#include "runtime/static_brake.hpp"

namespace {
void test_latch_state_restores_and_continues_bitwise() {
    auto env = launch_support::environment(
        terrain::load_track(launch_support::repo_file("tests/fixtures/tracks/climb_15pct_standstill.toml")), 2.);
    bike::validation::RuntimeRigAccess::set_rider_commands(*env.runtime, rider::RiderCommands{.front_brake = 1., .rear_brake = 1.});
    runtime::RideControl c{};
    for (int k = 0; k < 40; ++k)
        (void)env.runtime->advance(env.runtime->snapshot().step + env.control_steps, c, 0., 0., std::nullopt);
    bike::validation::RuntimeRigAccess::inspect(*env.runtime, [&](const Stepper &stepper, const runtime::PhysicalStep &p) {
        const runtime::StaticBrake &held = launch_support::static_brake(p);
        const auto state = held.state();
        bt::require(state.latch_rear.has_value(), "the rear bite is latched while holding");
        runtime::StaticBrake copy(held);          // same model indices and spec
        copy.reset();
        copy.restore(state);
        const auto a = held.hold_qfrc(stepper.model(), stepper.data(), 1., 1., false);
        const auto b = copy.hold_qfrc(stepper.model(), stepper.data(), 1., 1., false);
        bt::require(a == b, "restored latch gives the identical held force");
    });
}
// A non-finite latch is rejected and the brake keeps its state.
void test_non_finite_latch_is_rejected_atomically() {
    runtime::StaticBrake brake = launch_support::make_static_brake();
    const auto before = brake.state();
    bool threw = false;
    try { brake.restore({.latch_front = std::numeric_limits<double>::quiet_NaN()}); } catch (const std::invalid_argument &) { threw = true; }
    const auto after = brake.state();
    bt::require(threw && after.latch_front == before.latch_front && after.latch_rear == before.latch_rear, "rejected atomically");
}
}  // namespace

int main() {
    return bt::run([] {
        test_latch_state_restores_and_continues_bitwise();
        test_non_finite_latch_is_rejected_atomically();
    });
}
```

  В `launch_ride_support.hpp` добавить:
  - `static_brake(const runtime::PhysicalStep&)` — через новый `RuntimeRigAccess::static_brake(...)`;
    `brake_` у `PhysicalStep` закрытый, у класса уже есть `friend` для доступа;
  - `make_static_brake()` — конструктор `StaticBrake` с теми же аргументами, что в `step.cpp`. Найти по
    `brake_(` в списке инициализации `PhysicalStep`.
- [ ] **Шаг 2.** `mise run test 'static_brake_latch'` — не собирается.
- [ ] **Шаг 3. Код** по интерфейсу; кодек; тестовые файлы начального состояния (`/tmp/inv-runtime.txt`) получают
  `"brake_latch": {"front": null, "rear": null}`.
- [ ] **Шаг 4.** `mise run test 'static_brake_latch|io_state_codecs|native_runtime|equilibrium_state_file'`.
- [ ] **Шаг 5.** Проверка задачи. Коммит `feat(runtime): the static brake latch is part of the step state`.

### Задача 7. Итоговая проверка части 1

- [ ] **Шаг 1.** `bash tools/run_tests.sh full`. Записать код выхода и стадию, на которой он остановился. Если
  он остановился на известных красных обычных тестах, стадии после неё не считаются пройденными.
- [ ] **Шаг 2. Каждая стадия отдельно**, с записью кода выхода, длительности и имён красных:
  - `bash tools/run_native_checks.sh` (все проходы анализаторов и проверка структурных контрактов);
  - `ctest --test-dir build/native-release -L slow --no-tests=error`;
  - `ctest --test-dir build/native-release -L package --no-tests=error`.
- [ ] **Шаг 3. Сравнение.** Имена красных — с базовыми списками (`comm -3`); причины падения — с прошлым
  прогоном (первая строка ошибки). Новых нет. Изменённая причина у старого красного теста — находка: её
  разбирают.
- [ ] **Шаг 4.** Отчёт оркестратору: таблица стадий, времена, списки. Записать время в `timings.tsv`, если
  скрипт этого не сделал. Неполный прогон называется неполным.

---

## Обязательные исправления по ревью части 1

Ревью: `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan1-review/review.md`. P4 и P6 вписаны в задачу 4. Остальные
замечания исполнитель обязан учесть в той задаче, к которой они относятся: сверить с кодом и исправить. В
отчёте задачи — по строке на каждое: что сделано или почему не применимо.

| Замечание | Задача | Суть |
|---|---|---|
| P1 | 1 | Профиль для `f616f525` — копия без трёх полей, которых тогда не было. Доказать, что общие значения совпадают. Пробник собирать против этой копии. Проверять поля ошибок пробника. Полные 20 с |
| P2 | 2 | Начальное значение `RiderSenses` до первого шага; оба звена записи и чтения состояния |
| P3 | 2, 4 | Три разные таблицы каталога; выгрузка записи применённых команд целиком |
| P5 | 1, 2 | Опись включает конфигурации среды; правило «меняются входы, выходы не трогаются» проверяемо |
| P7 | 2 | Общий заголовок тестов собирается и сохраняет поля, нужные тесту педелека |
| P8 | 4 | Тесты действительно доказывают сброс мотора и первый тик после отпускания: смотреть состояние `AssistController`, а не только момент |
| P9 | 6 | Тест защёлки собирается, проходит через кодек и проверяет продолжение с движением |
| P10 | 3 | Проверка восстанавливаемого состояния адаптера: порядок и содержимое очереди, отказ без частичных изменений |
| P11 | 5 | Передача просьбы в рабочем коде и её проверка; продолжение автомата посреди среза |
| P12 | 2 | Доказательство «траектория не изменилась» и проверки ощущений — код, а не описание |
| P13 | 7 | Команда анализаторов в итоговой проверке с обязательным каталогом сборки; обработка раннего отказа |
| P14 | все | Места, описанные словами, заменяются кодом при исполнении; перенос строк автомата — дословно |
| M-P1 | 1 | Время холодной сборки копии `f616f525` учесть в бюджете |
| M-P2 | 2, 3 | Ощущения одного интервала берутся в одной точке времени: начало или конец интервала, одинаково для всех |

## Что остаётся частям 2–4

По ревью `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan-review/review.md`:

**Часть 2 — таблица опыта и подготовка** (I6, I7, I9, I13):
- автор пишет только таблицу, а значения нижнего уровня вычисляются (`fields.hpp`, `load.cpp`, `to_value`,
  проекция писателя);
- 34/24 у встроенного профиля;
- флаги командной строки;
- путь результата подготовки через `SetupRequest` → `SetupContext` → успокоение → сброс;
- проверки таблиц.

**Часть 3 — план старта и реакция** (C2, C3, I4, I10, I11, I12, I14):
- первые 0,15 с до первого ощущения — тормоз и поза держатся по подготовке;
- протокол просьбы и признак конца среза — на границе автомата;
- порядок «предел плана → реакция → срез → темп» на границе педалирования;
- права на нажим и позу раздельно;
- посев обычной политики и темп позы при передаче управления;
- события в записи для окна контроллера.

**Часть 4 — включение и проверки поездок** (I15, I16, I18):
- включение, утверждённые числа, перенос проверок приёмки и согласованные эталоны — одной зелёной задачей;
- помощник замеров с точными сигналами;
- итоговая проверка по образцу задачи 7.
