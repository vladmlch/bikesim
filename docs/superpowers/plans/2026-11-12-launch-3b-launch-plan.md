# План старта, часть 3b: узел плана старта, встраивание и калибровка — план реализации

> **Для исполнителя:** задачи выполняются по одной, в порядке номеров (`superpowers:subagent-driven-development`).
> Шаги помечены `- [ ]`. Финальное ревью ветки делает оркестратор.

Редакция 2 (2026-11-12, после ревью qoder `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/plan-review-1.md`:
«готово с правками», 9 мелких замечаний исправлены, M8 — пояснение в решении 5). Факты кода: `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/facts.md` (снят на `794cc9b4`).

**Цель.** Построить узел плана старта (`rider::LaunchPlan`) и встроить его в намерения райдера за выключателем
`rider_launch.plan_enabled`, который во всех профилях выключен. Поведение поездок не меняется. Затем замерить
план со включённым выключателем для трёх типов райдеров и показать числа пользователю (без коммита кода).

**Устройство.** План — чистый узел на задержанных ощущениях (`Percepts`) и ответе автомата переключений. Он
живёт в ядре `RiderIntent::Core` рядом с восприятием и реакцией и проходит ту же транзакцию тика (решение 11
части 3a). Тормоз и просьба о переключении уходят через уже проложенный канал `RiderCommands` (часть 1). Нажим
плана заменяет предел обычной политики, а наклон плана — цель программы позы; обе замены плавно сходят на нет
при передаче управления.

**Стек.** C++23, CMake/CTest, `bike_test::run`, кодеки `bike::io::Value`.

**Спецификация.** `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md` (разделы «Фазы», «Кто чем
управляет», «Таблица опыта», «Устройство в коде», «Проверки»). Дорожная карта:
`docs/superpowers/plans/2026-11-11-rider-launch-plan.md`, задачи 9–11. Замечания ревью дорожной карты
(`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan-review/review.md`): здесь закрываются C2, C3, I10, I11 и I14 для
плана. I15 и I16 (поездки с порогами, порядок включения) — часть 4.

## Решения плана

1. **Выключатель.** В `[rider_launch]` появляется `plan_enabled`. Во всех профилях и тестовых документах он
   `false`. При `false` узел всегда в `idle`, команда нейтральная (тормоз 0, вес 0, просьбы нет, владения
   переключениями нет), и выход намерений побайтно равен выходу части 3a: тест-оракул части 3a
   (`test_switched_off_resolver_equals_the_bare_policy`, побайтно против голой политики) получает выключенную
   таблицу плана и продолжает это доказывать. Мягкий старт остаётся в политике до части 4 (решение пользователя 1
   части 3a).
2. **Поля таблицы** (все обязательные, значений по умолчанию в коде нет; к пяти полям части 2 добавляются 19):

   | Поле | Смысл | Предварительное значение |
   |---|---|---|
   | `plan_enabled` | выключатель плана | `false` |
   | `standstill_speed_mps`, `standstill_crank_rpm` | «стою»: модуль скорости и каденс шатуна не выше | 0,1; 5 |
   | `effort_rate_nm_s` | темп нарастания нажима | 40 (= 60 / 1,5, правило «темп = предел / время» спецификации) |
   | `effort_cap_asphalt_nm` … `effort_cap_scree_nm` | предел нажима по покрытию под передним колесом, пять полей | 60 каждое |
   | `release_torque_grades`, `release_torque_nm` | нажим для отпускания тормоза по уклону, кусочно-линейно, узлы по возрастанию | `[0.0, 0.2]`, `[0.0, 30.0]` |
   | `brake_release_s` | время отпускания тормоза | 0,15 |
   | `max_hold_s` | наибольшее время удержания (потом «нажим не набран») | 1,5 |
   | `upshift_crank_rpm` | каденс шатуна для просьбы «звезду меньше» | 82 |
   | `end_speed_mps`, `end_crank_rpm`, `end_hold_s` | конец старта: скорость и каденс держатся дольше | 3,0; 60; 0,3 |
   | `max_launch_s` | наибольшее время старта от начала отпускания | 6,0 |
   | `handover_s` | время схождения к обычной политике | 0,9 |

   Предварительные числа помечаются в профилях как «до калибровки» и в части 3b ни на что не влияют. Все пять
   времён — целые кратные периодов всех профилей и тестовых документов (0,00125; 0,0005; 0,005; 0,01; 0,03):
   0,15 / 1,5 / 0,3 / 6,0 / 0,9.
3. **Времена — в тиках намерений**, как у реакции: `brake_release_s`, `max_hold_s`, `end_hold_s`, `max_launch_s`,
   `handover_s` — целые кратные `seated_climb.period_s`, не больше 2^31 тиков. Проверку делает
   `validate_launch_plan_context(table, period_s)`: корневой `post_init` профиля — всегда; конструктор
   `RiderIntent` — при включённых намерениях (он строится из документа запуска, `post_init` не проходит).
4. **Фазы** (`idle, hold, release, accelerate, handover, blend, done`):
   - `idle`/`done` → `hold`, когда есть восприятие и «стою»: `|speed_mps| ≤ standstill_speed_mps` и
     `crank_rpm ≤ standstill_crank_rpm` (модули: велосипед, быстро катящийся назад, план не включает — I10). Наклон
     плана — `prepare_launch(table, grade_ahead).lean_rad` по ощущаемому уклону. Звезду и угол педали по ходу
     поменять нельзя (спецификация).
   - `hold`: тормоз 1. Нажим растёт на `effort_rate_nm_s · period` за тик до предела покрытия под передним
     колесом. Часы и нажим стоят, пока нет восприятия или тормозит вызывающая сторона (решение 6). Переход в
     `release`, когда ощущаемый нажим ноги `human_crank_torque_nm ≥ release_torque(grade_ahead)`, либо по
     `max_hold_s` — тогда ещё `preload_missed`.
   - `release`: тормоз линейно уходит от 1 к 0 за `brake_release_s` тиков (0 тиков — сразу), нажим растёт. При
     тормозе 0 — `accelerate`. Мотор выключен, пока тормоз райдера больше нуля: это делает уже проложенный
     `motor_braking` (часть 1), отдельной команды мотора нет.
   - `accelerate`: нажим растёт до предела покрытия под передним колесом (предел может упасть сразу, когда
     колесо въехало на другое покрытие); переключения — решение 5; `end_ok_ticks` считает подряд идущие
     восприятия со `speed_mps ≥ end_speed_mps` и `crank_rpm ≥ end_crank_rpm`. Переход в `handover`, когда
     `end_ok_ticks > end_hold_s / period` или прошло `max_launch_s` от начала отпускания.
   - `handover` — ждёт конца среза. Просьба снимается в тик входа. Владение переключениями остаётся у плана,
     пока автомат сообщает идущий срез (`cut_active`). Тик, в котором среза нет, — переход в `blend`.
   - `blend`: владение переключениями у автомата; вес плана `1 − blend_ticks / handover_ticks` падает до 0; при 0 —
     `done`.
   - Неподходящий тик (решение 6) из любой фазы — сразу `idle` с нейтральной командой.
5. **Протокол просьбы о переключении (C3).** Автомат (часть 1) отвечает на каждом физическом шаге: `accepted`,
   `deferred` (ждёт паузы или конца среза), `rejected` (край кассеты, проскальзывание, каденс посадки), и
   отложенную просьбу не хранит. План держит просьбу сам и повторяет её каждый тик.
   - Вход плана — `ShiftFeedback {response, shift_count, cut_active}` из состояния автомата после прошлого шага:
     последний ответ, число совершённых переключений и идёт ли срез (`cut_remaining_s > 1e-12`, тот же порог, что
     у допуска автомата; на идеальной втулке смена передаточного числа кончается вместе со срезом, на
     геометрической — сразу). Это механическая обратная связь рычага и цепи, а не задержанное ощущение.
   - Просьба выдаётся в `accelerate`, когда её нет, срез не идёт, `crank_rpm ≥ upshift_crank_rpm`, а ощущаемая
     звезда не равна заблокированной. Вместе с ней запоминаются `request_count = shift_count` и
     `request_teeth = ощущаемая звезда`.
   - Просьба снимается: принята — `shift_count > request_count` (счёт, а не метка: при периоде намерений длиннее
     шага метка последнего шага может уже быть `deferred` из-за паузы после принятия); отклонена — ответ
     `rejected`. В обоих случаях `blocked_teeth = request_teeth`: новая просьба — только когда ощущаемая звезда
     сменилась. Поэтому задержанный каденс до переключения не вызывает второго переключения, а отказ на краю
     кассеты не повторяется.
   - Второе принятие одной просьбы между двумя тиками невозможно и при нулевой паузе автомата: пока идёт срез,
     автомат отвечает `deferred` (`shifting.cpp:62-90`), а срез длиннее периода намерений во всех профилях.
   - `deferred` и `none` просьбу не снимают. При входе в `handover` просьба снимается всегда; владение уходит
     автомату только при `cut_active = false` — круговое ожидание из C3 невозможно: срез конечен.
6. **Кто может запустить план и кто им владеет (I10).** Тик подходящий, если намерения активны (`active`), райдер
   включён (`control.rider_enabled`), режим привода силовой (`articulated_effort` или `crank_effort`) и
   вызывающая сторона не задала нажим (`control.human_torque_nm` пуст). Иначе план сразу уходит в `idle`:
   тормоз 0, просьбы и владения нет. После снятия внешнего нажима план снова включается по «стою».
   - Внешняя поза (`control.posture`) план не отменяет: выход `output()` просто не подставляет позу намерений.
   - Внешний тормоз (`front > 0 || rear > 0` у вызывающей стороны) в `hold` останавливает часы и держит нажим 0:
     райдер ждёт, пока его держат. Итоговый тормоз — наибольший из трёх, как сейчас (часть 1), поэтому план не
     может отпустить то, что держит другой.
7. **Старт эпизода до первого восприятия (C2).** Если эпизод начинается стоя (`initial_speed_mps == 0`) в
   силовом режиме привода, `finalize` после `seed_lean` вызывает `RiderIntent::arm_standstill_start(lean)`: план
   сразу в `hold` с тормозом 1 и наклоном подготовки, и команды райдера уже содержат тормоз 1 до первого тика
   (торможение шага разрешается до тика намерений). Пока восприятия нет (первые `reaction_delay_s`), план держит
   тормоз и не давит: решение давить приходит с первым ощущением. Старт с ходу (`initial_speed_mps > 0`) план не
   включает. Пустое восприятие никогда не разыменовывается.
8. **Передача обычной езде (I11).**
   - **Нажим.** Пока вес плана `w > 0`, предел намерений — `w = 1`: нажим плана; `0 < w < 1`:
     `w · план + (1 − w) · политика`. Политика всё это время обновляется как обычно, но бюджет рывка не тратит
     (`spend_surge = false`; запас не тратится и не восстанавливается). При `w = 0` предел — политики, побайтно.
   - **Наклон.** План не задаёт угол напрямую. Он передаёт программе позы цель `LeanOverride {target_rad,
     weight}`: основа цели — `w = 1`: цель плана; `0 < w < 1`: `w · план + (1 − w) · (lean_gain · atan(grade))`.
     Импульсы корпуса и добавка реакции прибавляются, как сейчас; поправка по нагрузке переднего колеса при
     `w = 1` не накапливается и не применяется, при `0 < w < 1` накапливается и входит с долей `1 − w`. Дальше
     прежние ограничения программы: темп наклона, передний и задний пределы, сгиб локтя. Поэтому любой переход,
     включая повторный старт, не быстрее `lean_rate_rad_s` (I11). Без переопределения арифметика побайтно прежняя.
   - Реакция на подъём колеса продолжается через передачу без перезапуска: её множитель применяется к итоговому
     пределу, а добавка — к цели программы (часть 3a).
   - Признак `effort_limited` (порядок «предел → срез» на педалях) истинен и тогда, когда план владеет нажимом
     (`w > 0`).
9. **События в записи (I14).** Канал `rider_launch` на каждом интервале (как `rider_reaction`): `phase`, `brake`,
   `effort_nm`, `weight`, `lean_target_rad`, `preload_missed`. Нейтральный при выключенном плане и выключенных
   намерениях. Отпускание тормоза, «нажим не набран», передача управления видны как смена значений между
   интервалами; просьба и ответ автомата уже пишутся (`channel_rider_commands.shift_request`,
   `channel_drive.shift_rider_response`). В каталоге — таблица `channel_rider_launch` (2 ключа + 6 полей), счёт
   описателей 512 → 520.
10. **Документы.**
    - Документ запуска: ключ верхнего уровня `rider_launch` (вся таблица, 24 ключа) рядом с `rider_reaction`, только
      в списке ключей конфигурации. `project_config.cpp` кодирует его из профиля.
    - Состояние: внутри `rider_intent` ключ `launch` (состояние узла, 13 ключей). Есть ровно при включённых
      намерениях, как `perception` и `reaction`.
11. **Восстановление.** `LaunchPlan::restore` проверяет необходимые условия, как в части 3a, а не точную
    достижимость: отвергает состояния, которые `update()` оставить не может, по признакам (фаза,
    счётчики, тормоз по фазе и часам, вес, просьба только в `accelerate`, сброшенные поля в `idle`/`done`).
    `RiderIntent::restore` добавляет совместное правило: в `hold` `phase_ticks` не больше числа выданных
    восприятий, а `launch_ticks` не больше его во всех фазах (часы плана идут только после первого восприятия).
    Команды райдера после восстановления выводятся из состояния узла, отдельно не сохраняются.
12. **Перенос тестовых документов** — точный, скриптами (как в части 3a), задача 3:
    - 22 документа конфигурации: `rider_launch` сразу после `rider_reaction`, таблица «как сегодня» этого документа:
      `start_cog_teeth = [drive.gearing.rear_teeth]`, `start_cog_grade_breaks = []`,
      `start_pedal_angle_rad = drive.crank_phase_rad`, наклон `[0.0]`/`[0.0]`, поля плана — предварительные;
    - 21 документ состояния и два `rider_intent*.json`: `rider_intent.launch` — начальное состояние;
    - входы `input.restore.*` теста `reference.native_rider_intent` (оба платформенных набора) — то же начальное
      состояние. Тест остаётся красным с прежней первой причиной (адаптер).

## Общие ограничения

Как в части 3a (`docs/superpowers/plans/2026-11-11-launch-3a-perception-reaction.md`, «Общие ограничения»), с
этими уточнениями:
- **Доказательство неизменности — по частям:** успокоенная поза и модель — `ab_settle.sh`; выход намерений и
  команды при выключенном плане — тест-оракул задачи 3 (побайтно против пути части 3a); арифметика программы позы
  и политики без переопределения — тест задачи 2; поездки — неизменный код езды и прежние тесты поездок.
- **Перезапись эталонов** — правило частей 2 и 3a: только выходы зелёных тестов, только новые ключи, проверка
  `check_rerecord.py --ref <коммит до задачи>`; изменение прежнего значения (хэши включительно) — стоп; выходы
  красных тестов не трогаются; входные документы переводятся везде.
- **Красные тесты.** 22 обычных и 10 медленных. Нельзя переставлять или ослаблять проверки в красном тесте. Список
  принятых изменений причин — `EXPECT` (`causes.py`).
  - Задача 3 добавляет 4 числовых поля в каждый интервал: `reference.native_runtime_rollout` меняется с
    `actual 625` на `actual 629`. Список — `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/expect-task3.tsv`.
  - Другое изменение причины — стоп с точной парой строк.
- **Пороги, пределы, допуски не меняются.** Предварительные числа таблицы — новые поля за выключенным
  выключателем. Калибровочные числа — только после «да» пользователя (часть 4).
- Без отладчиков, с ограничением времени, без `git stash`, без push, `.claude/` и `.agents/` не трогать. Последняя
  строка коммита ровно `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Проверка задачи

После коммита задачи:

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
BASELOG=/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/baseline-before-task<N>.log \
  EXPECT=<список задачи или пусто> \
  bash /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/check_task.sh <база задачи> \
  /Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/verify-task<N> ab
```

Оркестратор кладёт `baseline-before-task<N>.log` (`reds_output.log` предыдущей проверки; для задачи 1 —
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/final/reds_output.log`) и пишет в бриф базу задачи. Конец
вывода — `check_task exit 0`.

## Бюджет тестов

| Что | Время |
|---|---|
| целевые тесты задачи | 3–8 мин |
| `check_task.sh` с `ab` (сборка инкрементальная) | 10–30 мин |
| задача 4 (калибровка): около 40 поездок по 20 с | около 30 мин |
| задача 5: итоговая проверка | около 40 мин |

Итого около 3 ч машинного времени без повторов.

## Опись

- Таблица `[rider_launch]` (задача 1): 16 файлов `*.toml` (`git grep -l '^\[rider_launch\]' -- '*.toml'`); встроенные
  TOML-строки тестов (`git grep -l '\[rider_launch\]' -- native/tests`); программные конфигурации — `git grep -n
  'rider_launch *=' -- native` и `git grep -n 'LaunchConfig{' -- native` (у каждого места появляются 19 полей);
  `test_schema.cpp` (`rider_launch field count` 5 → 24).
- Конструкторы `RiderIntent` (задача 3): `step.cpp:1024`, `test_native_rider_intent.cpp`,
  `test_seated_posture_program.cpp`, `test_front_lift_reaction.cpp`, `test_launch_preparation.cpp`. Тестам нужна
  выключенная таблица `kSwitchedOffLaunch` (код в задаче 3).
- Документы (задача 3) — решение 12; скрипты `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/tools/` готовит
  оркестратор до задачи 3 по образцу `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/tools/`.

---

### Задача 1. Поля плана в таблице `[rider_launch]`

**Файлы:**
- Изменить: `native/src/rider/launch_config.hpp`, `native/src/rider/launch_config.cpp`,
  `native/src/configuration/model_config.hpp` (`kRiderLaunchConfigFields`), `native/src/configuration/validate.cpp`
  (`post_init`), все документы и программные конфигурации из описи, `native/tests/config/test_schema.cpp`.
- Тест: `native/tests/reference/test_launch_preparation.cpp` (правила таблицы), `test_welded_profile_config.cpp`.

**Интерфейсы** (`launch_config.hpp`):

```cpp
struct LaunchConfig {
    // ... the five part-2 fields, unchanged ...
    // Launch part 3b — the launch plan (rider/launch_plan.hpp). Off in
    // every profile until part 4 switches it on.
    bool plan_enabled = false;
    double standstill_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double standstill_crank_rpm = std::numeric_limits<double>::quiet_NaN();
    double effort_rate_nm_s = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_asphalt_nm = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_hardpack_nm = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_loose_nm = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_wet_nm = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_scree_nm = std::numeric_limits<double>::quiet_NaN();
    // Felt crank torque that releases the brakes, piecewise linear over
    // strictly ascending grade knots, held at the end values outside them.
    std::vector<double> release_torque_grades;
    std::vector<double> release_torque_nm;
    double brake_release_s = std::numeric_limits<double>::quiet_NaN();
    double max_hold_s = std::numeric_limits<double>::quiet_NaN();
    double upshift_crank_rpm = std::numeric_limits<double>::quiet_NaN();
    double end_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double end_crank_rpm = std::numeric_limits<double>::quiet_NaN();
    double end_hold_s = std::numeric_limits<double>::quiet_NaN();
    double max_launch_s = std::numeric_limits<double>::quiet_NaN();
    double handover_s = std::numeric_limits<double>::quiet_NaN();
};

// The release torque the table gives for one grade (same interpolation as
// the start lean). Validates the table first.
[[nodiscard]] double release_torque_nm(const LaunchConfig &table, double grade);

// The plan times on the intention period grid, at most 2^31 ticks each.
// The profile post_init (always) and RiderIntent (enabled intent) call it.
void validate_launch_plan_context(const LaunchConfig &table, double period_s);
```

- [ ] **Шаг 1. Тест (падает).** В `test_launch_preparation.cpp` — правила новых полей:

```cpp
// Launch part 3b: every plan field has a domain, the release table needs one
// torque per ascending knot, and the plan times sit on the intention grid.
void test_plan_fields_name_the_field() {
    const auto with = [](auto edit) { auto t = ramp_table(); edit(t); return t; };
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.standstill_speed_mps = -.1; })); },
            "standstill_speed_mps", "a negative standstill speed");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.effort_rate_nm_s = 0.; })); },
            "effort_rate_nm_s", "a zero effort rate");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.effort_cap_wet_nm = 0.; })); },
            "effort_cap_wet_nm", "a zero wet cap");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.release_torque_nm = {5.}; })); },
            "release_torque_nm", "one torque for two knots");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.release_torque_grades = {.2, .1}; })); },
            "release_torque_grades", "descending knots");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.release_torque_nm = {-1., 30.}; })); },
            "release_torque_nm", "a negative release torque");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.max_hold_s = 0.; })); },
            "max_hold_s", "a zero hold limit");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.upshift_crank_rpm = 0.; })); },
            "upshift_crank_rpm", "a zero upshift cadence");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.max_launch_s = 0.; })); },
            "max_launch_s", "a zero launch limit");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.end_hold_s = std::numeric_limits<double>::quiet_NaN(); })); },
            "end_hold_s", "a NaN end hold");
    rejects([&] { rider::validate_launch_plan_context(with([](auto &t) { t.brake_release_s = .0011; }), .00125); },
            "brake_release_s", "a release time off the grid");
    rejects([&] { rider::validate_launch_plan_context(with([](auto &t) { t.max_launch_s = 0x1p40 * .00125; }),
                                                      .00125); },
            "max_launch_s", "a launch limit beyond 2^31 ticks");
    rider::validate_launch_plan_context(ramp_table(), .00125);   // control
    bt::require(rider::release_torque_nm(ramp_table(), -.1) == 0. &&
                    rider::release_torque_nm(ramp_table(), .1) == 15. &&
                    rider::release_torque_nm(ramp_table(), .5) == 30.,
                "the release torque interpolates and holds its end values");
}
```

  `ramp_table()` (`test_launch_preparation.cpp:58`) получает 19 полей предварительными числами решения 2
  (`plan_enabled = false`). `rejects` — помощник файла (`:44-54`).

  В `test_welded_profile_config.cpp` (ревью 1, M7):
  - `test_launch_table_keys_are_required` (`:397-399`): список строк получает 19 новых строк блока (по одной на
    ключ, точный текст из блока шага 3), чтобы отсутствие каждого ключа отвергалось с его именем;
  - удаление всего раздела (`:404-412`): конечный маркер `"start_lean_rad = [0.0]\n"` меняется на
    `"handover_s = 0.9\n"` — последнюю строку нового блока;
  - `test_load_resolves_a_flat_start` (`:518-534`) меняет отдельные ключи профиля и не трогает новые; проверить,
    что он зелёный, и не менять его.

- [ ] **Шаг 2.** `mise run test 'launch_preparation'` — сборка падает (нет полей).
- [ ] **Шаг 3. Код.**
  - `launch_config.cpp`, в `validate_launch_table` после проверок части 2:

```cpp
    const auto nonnegative = [](double v, std::string_view field) {
        if (!(std::isfinite(v) && v >= 0.)) reject(field, "must be finite and nonnegative");
    };
    const auto positive = [](double v, std::string_view field) {
        if (!(std::isfinite(v) && v > 0.)) reject(field, "must be finite and positive");
    };
    nonnegative(table.standstill_speed_mps, "standstill_speed_mps");
    nonnegative(table.standstill_crank_rpm, "standstill_crank_rpm");
    positive(table.effort_rate_nm_s, "effort_rate_nm_s");
    positive(table.effort_cap_asphalt_nm, "effort_cap_asphalt_nm");
    positive(table.effort_cap_hardpack_nm, "effort_cap_hardpack_nm");
    positive(table.effort_cap_loose_nm, "effort_cap_loose_nm");
    positive(table.effort_cap_wet_nm, "effort_cap_wet_nm");
    positive(table.effort_cap_scree_nm, "effort_cap_scree_nm");
    if (table.release_torque_grades.empty()) reject("release_torque_grades", "needs at least one knot");
    if (table.release_torque_nm.size() != table.release_torque_grades.size())
        reject("release_torque_nm", "needs one torque per release_torque_grades knot");
    strictly_ascending(table.release_torque_grades, "release_torque_grades");
    for (const double torque : table.release_torque_nm) nonnegative(torque, "release_torque_nm");
    nonnegative(table.brake_release_s, "brake_release_s");
    positive(table.max_hold_s, "max_hold_s");
    positive(table.upshift_crank_rpm, "upshift_crank_rpm");
    nonnegative(table.end_speed_mps, "end_speed_mps");
    nonnegative(table.end_crank_rpm, "end_crank_rpm");
    nonnegative(table.end_hold_s, "end_hold_s");
    positive(table.max_launch_s, "max_launch_s");
    nonnegative(table.handover_s, "handover_s");
```

  и новые функции (интерполяция — тот же порядок операций, что у наклона в `prepare_launch`):

```cpp
double release_torque_nm(const LaunchConfig &table, double grade) {
    validate_launch_table(table);
    if (!std::isfinite(grade)) throw std::invalid_argument("launch release grade must be finite");
    const auto &knots = table.release_torque_grades;
    const auto &torques = table.release_torque_nm;
    if (grade >= knots.back()) return torques.back();
    if (grade <= knots.front()) return torques.front();
    const auto upper = static_cast<std::size_t>(
        std::distance(knots.begin(), std::ranges::upper_bound(knots, grade)));
    const double t = (grade - knots[upper - 1]) / (knots[upper] - knots[upper - 1]);
    return torques[upper - 1] + t * (torques[upper] - torques[upper - 1]);
}

void validate_launch_plan_context(const LaunchConfig &table, double period_s) {
    validate_launch_table(table);
    if (!(std::isfinite(period_s) && period_s > 0.))
        throw std::invalid_argument("launch plan needs a positive intention period");
    const auto on_grid = [period_s](double seconds, std::string_view field) {
        const double ticks = std::rint(seconds / period_s);
        if (std::abs(ticks * period_s - seconds) > 1e-9) reject(field, "must be a whole number of intention periods");
        if (ticks > 0x1p31) reject(field, "must be at most 2^31 intention periods");
    };
    on_grid(table.brake_release_s, "brake_release_s");
    on_grid(table.max_hold_s, "max_hold_s");
    on_grid(table.end_hold_s, "end_hold_s");
    on_grid(table.max_launch_s, "max_launch_s");
    on_grid(table.handover_s, "handover_s");
}
```

  - `model_config.hpp`, `kRiderLaunchConfigFields` — 19 описателей после `start_lean_rad`, в порядке структуры, по
    образцу соседних: `plan_enabled` — `FieldKind::Boolean`, `.type="<class 'bool'>"`; вещественные —
    `FieldKind::Real`, `.type="<class 'float'>"`; `release_torque_grades`, `release_torque_nm` —
    `FieldKind::RealTuple`, `.type="tuple[float, ...]"`; все `.required=true`.
  - `validate.cpp`, корневой `post_init` после проверок `rider_launch`:

```cpp
    // Launch part 3b: the plan counts intention ticks — the same context
    // check RiderIntent runs on runtime documents.
    ::rider::validate_launch_plan_context(config.rider_launch, config.seated_climb.period_s);
```

  - Документы — блок после `start_lean_rad = [...]` во всех 16 файлах и встроенных строках (одинаковый текст):

```toml
# Launch plan (part 3b): off until part 4 switches it on; preliminary
# numbers, calibrated before activation.
plan_enabled = false
standstill_speed_mps = 0.1
standstill_crank_rpm = 5.0
effort_rate_nm_s = 40.0
effort_cap_asphalt_nm = 60.0
effort_cap_hardpack_nm = 60.0
effort_cap_loose_nm = 60.0
effort_cap_wet_nm = 60.0
effort_cap_scree_nm = 60.0
release_torque_grades = [0.0, 0.2]
release_torque_nm = [0.0, 30.0]
brake_release_s = 0.15
max_hold_s = 1.5
upshift_crank_rpm = 82.0
end_speed_mps = 3.0
end_crank_rpm = 60.0
end_hold_s = 0.3
max_launch_s = 6.0
handover_s = 0.9
```

    Встроенные строки получают тот же набор ключей через `\n`. Программные конфигурации — те же числа полями
    структуры. Проверка полноты — пустой вывод:

```bash
for f in $(git grep -l '\[rider_launch\]' -- native/tests examples tests); do
  r=$(grep -o '\[rider_launch\]' "$f" | wc -l); p=$(grep -o 'plan_enabled' "$f" | wc -l)
  [ "$r" -eq "$p" ] || echo "$f rider_launch=$r plan_enabled=$p"
done
```

  - `test_schema.cpp`: `fields_named(fields[11], "rider_launch").size() == 24` с комментарием «5 part-2 fields +
    19 launch plan fields».
- [ ] **Шаг 4.** `perl -e 'alarm 2400; exec @ARGV' -- mise run test 'launch_preparation|welded_profile_config|front_lift_reaction|seated_pedaling_cycle|realistic_pedelec_acceptance|equilibrium_state_file|initialization_setup|initialization_transfer|cli_capture|assist_controller|pneumatic_tire_config|articulated_config|validation|replay_session'`
  — новые зелёные, прежние как в базе.
- [ ] **Шаг 5. Перезаписи** — по правилу; список или «нет».
- [ ] **Шаг 6. Коммит и проверка.** `feat(rider): the launch plan fields of the rider launch table`; `check_task.sh`,
  `EXPECT` пусто, с `ab`.

### Задача 2. Узел плана старта и крючки программы позы и политики

**Файлы:**
- Создать: `native/src/rider/launch_plan.hpp`, `native/src/rider/launch_plan.cpp`,
  `native/tests/reference/test_launch_plan.cpp`.
- Изменить: `CMakeLists.txt` (`launch_plan.cpp` в `bike_configuration` после `front_lift_reaction.cpp`),
  `native/src/rider/senses_codec.{hpp,cpp}` (кодеки состояния и таблицы), `native/src/rider/intent.{hpp,cpp}`
  (`LeanOverride`, `SeatedPostureProgram::update`, `SeatedClimbPolicy::update`, `power_target_w`),
  `native/tests/reference/Phase3Ports.cmake`.
- Тест: `test_launch_plan.cpp`, `test_seated_posture_program.cpp` (крючки побайтно).

**Интерфейсы** (`launch_plan.hpp`):

```cpp
// rider/launch_plan.hpp — the rider's standstill launch (launch plan,
// part 3b): hold the brakes and press, release by the felt crank torque,
// accelerate with the rider's own upshift requests, then hand effort,
// lean and shifting back to the ordinary policy. Runs on the delayed
// percepts and the shifter's mechanical answer; times count intention
// ticks. Off (always idle) unless rider_launch.plan_enabled.
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

#include "launch_config.hpp"
#include "perception.hpp"

namespace rider {

enum class LaunchPhase : std::uint8_t { idle, hold, release, accelerate, handover, blend, done };
[[nodiscard]] const char *launch_phase_name(LaunchPhase phase);   // throws on a value outside the enum
[[nodiscard]] LaunchPhase launch_phase(std::string_view name);    // throws on an unknown name

enum class ShiftResponse : std::uint8_t { none, accepted, deferred, rejected };
// The shifter's rider_response label ("none", "accepted", "deferred",
// "rejected"); throws on anything else.
[[nodiscard]] ShiftResponse shift_response(std::string_view label);

// The shifter state after the previous physics step: the mechanical answer
// of lever and chain, not a delayed percept.
struct ShiftFeedback {
    ShiftResponse response = ShiftResponse::none;
    std::int64_t shift_count = 0;   // committed shifts so far
    bool cut_active = false;        // cut_remaining_s > 1e-12 (the ideal hub sweeps with it)
};

struct LaunchInputs {
    std::optional<Percepts> percepts;   // absent until the reaction delay has passed
    ShiftFeedback shift;
    bool eligible = false;         // active intent, rider enabled, effort drive mode, no caller effort
    bool external_brake = false;   // the caller brakes
};

struct LaunchCommand {
    LaunchPhase phase = LaunchPhase::idle;
    double brake = 0.;             // both wheels, [0, 1]
    double effort_nm = 0.;         // the plan's effort ceiling
    double weight = 0.;            // the plan's share of effort and lean target
    double lean_target_rad = 0.;
    bool shift_up = false;         // the pending upshift request
    bool owns_shifting = false;
    bool preload_missed = false;   // the hold ran out before the felt torque came
};

class LaunchPlan {
public:
    struct State {
        LaunchPhase phase = LaunchPhase::idle;
        std::int64_t phase_ticks = 0;    // hold: percept ticks without a caller brake; release: ticks
        std::int64_t launch_ticks = 0;   // ticks since the release began
        std::int64_t end_ok_ticks = 0;   // consecutive percepts at the end speed and cadence
        std::int64_t blend_ticks = 0;
        double effort_nm = 0.;
        double brake = 0.;
        double lean_target_rad = 0.;
        bool shift_up = false;
        std::int64_t request_count = 0;  // shift_count when the pending request was issued
        int request_teeth = 0;           // felt cog when the pending request was issued
        int blocked_teeth = 0;           // felt cog of the last answered request; 0 = none
        bool preload_missed = false;
        bool operator==(const State &) const = default;
    };
    LaunchPlan(const LaunchConfig &config, double period_s);
    void reset();
    // A standing episode start: hold with the prepared lean before any
    // percept. No-op (and false) when the plan is switched off.
    bool arm_start(double lean_rad);
    // Rejects nonfinite percept fields and a negative shift count without
    // changing anything.
    static void validate(const LaunchInputs &inputs);
    LaunchCommand update(const LaunchInputs &inputs);
    [[nodiscard]] LaunchCommand command() const;
    [[nodiscard]] State state() const;
    // Rejects every state update() cannot leave behind (decision 11); a
    // rejected state leaves the plan unchanged.
    void restore(const State &state);

private:
    [[nodiscard]] double cap(SurfaceLabel surface) const;
    [[nodiscard]] double ramp(double effort, const Percepts &percepts) const;
    [[nodiscard]] bool standstill(const Percepts &percepts) const;
    void arm(State &n, double lean_rad) const;
    void to_release(State &n) const;
    void shifting(State &n, const Percepts &percepts, const ShiftFeedback &shift) const;

    LaunchConfig config_;
    double period_s_;
    std::int64_t release_ticks_ = 0, max_hold_ticks_ = 0, end_hold_ticks_ = 0, max_launch_ticks_ = 0,
                 handover_ticks_ = 0;
    State s_;
};

} // namespace rider
```

Крючки (`intent.hpp`):

```cpp
// Launch part 3b, decision 8: the launch plan's lean target and its share.
// The program keeps its rate and bounds for every target it is given.
struct LeanOverride {
    double target_rad = 0.;
    double weight = 1.;   // (0, 1]
};

// SeatedPostureProgram::update gains a last parameter
//     std::optional<LeanOverride> launch = std::nullopt
// SeatedClimbPolicy::update gains two last parameters
//     bool spend_surge = true, std::optional<LeanOverride> launch = std::nullopt
// SeatedClimbPolicy::power_target_w gains a last parameter
//     bool spend_surge = true
```

Кодеки (`senses_codec.hpp`):

```cpp
// The plan's restorable document: {"phase", "phase_ticks", "launch_ticks",
// "end_ok_ticks", "blend_ticks", "effort_nm", "brake", "lean_target_rad",
// "shift_up", "request_count", "request_teeth", "blocked_teeth",
// "preload_missed"} — exact keys, the phase by name.
[[nodiscard]] bike::io::Value encode_launch_state(const LaunchPlan::State &state);
[[nodiscard]] LaunchPlan::State decode_launch_state(const bike::io::Value &v, std::string_view path);
// The runtime document of [rider_launch]: the 24 keys, exact, then
// validate_launch_table.
[[nodiscard]] bike::io::Value encode_launch_config(const LaunchConfig &config);
[[nodiscard]] LaunchConfig decode_launch_config(const bike::io::Value &v, std::string_view path);
```


- [ ] **Шаг 1. Тесты (падают)** — `test_launch_plan.cpp`:

```cpp
// Launch part 3b — the launch plan node on synthetic percepts: hold and
// press, release by the felt torque, the hold timeout, the caller brake
// freeze, the upshift protocol against the shifter's answers, the handover
// that waits for the cut and blends, restart after a stop, eligibility,
// the switch, the codec round trip with bitwise continuation and the
// rejected restores.
#include "support/fixture_test.hpp"
#include "io/json.hpp"
#include "rider/launch_plan.hpp"
#include "rider/senses_codec.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {

constexpr double kPeriod = .01;

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

// Effort 100 N·m/s (1 N·m per tick), caps 60 except loose 20; release at
// 10 N·m on the flat, 30 N·m at 20 %; release over 4 ticks; hold limit 50
// ticks; upshift at 80 rpm; end 3 m/s and 60 rpm for more than 5 ticks;
// launch limit 300 ticks; handover over 10 ticks.
rider::LaunchConfig table() {
    rider::LaunchConfig t;
    t.start_cog_grade_breaks = {};
    t.start_cog_teeth = {51};
    t.start_pedal_angle_rad = 1.2;
    t.start_lean_grades = {0., .2};
    t.start_lean_rad = {0., .2};
    t.plan_enabled = true;
    t.standstill_speed_mps = .1;
    t.standstill_crank_rpm = 5.;
    t.effort_rate_nm_s = 100.;
    t.effort_cap_asphalt_nm = 60.;
    t.effort_cap_hardpack_nm = 60.;
    t.effort_cap_loose_nm = 20.;
    t.effort_cap_wet_nm = 60.;
    t.effort_cap_scree_nm = 60.;
    t.release_torque_grades = {0., .2};
    t.release_torque_nm = {10., 30.};
    t.brake_release_s = 4 * kPeriod;
    t.max_hold_s = 50 * kPeriod;
    t.upshift_crank_rpm = 80.;
    t.end_speed_mps = 3.;
    t.end_crank_rpm = 60.;
    t.end_hold_s = 5 * kPeriod;
    t.max_launch_s = 300 * kPeriod;
    t.handover_s = 10 * kPeriod;
    return t;
}

rider::Percepts standing(double grade = 0.) {
    rider::Percepts p;
    p.senses.speed_mps = 0.;
    p.senses.rear_teeth = 51;
    p.senses.surface_front = rider::SurfaceLabel::hardpack;
    p.senses.surface_ahead = rider::SurfaceLabel::hardpack;
    p.senses.grade_ahead = grade;
    return p;
}

rider::LaunchInputs in(std::optional<rider::Percepts> p, rider::ShiftFeedback shift = {}) {
    return {.percepts = std::move(p), .shift = shift, .eligible = true, .external_brake = false};
}

std::string text(const rider::LaunchPlan &plan) {
    return bike::io::canonical_json(rider::encode_launch_state(plan.state()));
}

// Drives a plan from a standstill into accelerate: one standing percept
// arms the hold, the felt torque reaches the release value, the brake
// leaves over four ticks.
rider::LaunchPlan accelerating() {
    rider::LaunchPlan plan(table(), kPeriod);
    auto p = standing();
    (void)plan.update(in(p));
    p.human_crank_torque_nm = 10.;
    (void)plan.update(in(p));
    for (int i = 0; i < 4; ++i) (void)plan.update(in(p));
    bt::require(plan.command().phase == rider::LaunchPhase::accelerate, "the helper reaches accelerate");
    return plan;
}

// Hold: brake 1, the plan owns effort and shifting, effort rises one tick
// at a time to the cap of the surface under the front wheel.
void test_hold_brakes_and_ramps_to_the_surface_cap() {
    rider::LaunchPlan plan(table(), kPeriod);
    auto cmd = plan.update(in(standing(.1)));
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.brake == 1. && cmd.weight == 1. &&
                    cmd.owns_shifting && !cmd.shift_up && cmd.effort_nm == 0.,
                "a standing percept arms the hold");
    bt::require(std::abs(cmd.lean_target_rad - .1) < 1e-15, "the hold leans by the felt grade");
    auto p = standing(.1);
    for (int i = 0; i < 5; ++i) cmd = plan.update(in(p));
    bt::require(std::abs(cmd.effort_nm - 5.) < 1e-12, "effort rises by rate * period per tick");
    p.senses.surface_front = rider::SurfaceLabel::loose;
    for (int i = 0; i < 30; ++i) cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 20. && cmd.phase == rider::LaunchPhase::hold,
                "the loose cap binds while the felt torque stays below the release value");
}

// Before the first percept the plan holds without pressing and its clock
// stands; the caller's brake freezes the hold the same way.
void test_no_percept_and_caller_brake_freeze_the_hold() {
    rider::LaunchPlan plan(table(), kPeriod);
    bt::require(plan.arm_start(.05), "a switched-on plan arms at the start");
    for (int i = 0; i < 100; ++i) {
        const auto cmd = plan.update(in(std::nullopt));
        bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.brake == 1. && cmd.effort_nm == 0. &&
                        cmd.lean_target_rad == .05,
                    "without percepts the rider holds and waits");
    }
    bt::require(plan.state().phase_ticks == 0, "the hold clock does not run without percepts");
    auto braked = in(standing());
    braked.external_brake = true;
    for (int i = 0; i < 100; ++i) (void)plan.update(braked);
    bt::require(plan.state().phase_ticks == 0 && plan.command().effort_nm == 0. && !plan.command().preload_missed,
                "the caller brake freezes the hold");
}

// The release starts on the felt crank torque, not on the plan's own
// effort, and the brake leaves linearly; the timeout releases with an event.
void test_release_by_felt_torque_and_by_timeout() {
    rider::LaunchPlan plan(table(), kPeriod);
    auto p = standing();
    for (int i = 0; i < 20; ++i) (void)plan.update(in(p));
    bt::require(plan.command().phase == rider::LaunchPhase::hold && plan.command().effort_nm >= 19.,
                "a large plan effort alone does not release");
    p.human_crank_torque_nm = 10.;
    auto cmd = plan.update(in(p));
    bt::require(cmd.phase == rider::LaunchPhase::release && cmd.brake == 1. && !cmd.preload_missed,
                "the felt torque starts the release");
    double previous = 1.;
    for (int i = 1; i <= 4; ++i) {
        cmd = plan.update(in(p));
        bt::require(cmd.brake < previous && std::abs(cmd.brake - (1. - i / 4.)) < 1e-15,
                    "the brake leaves linearly over brake_release_s");
        previous = cmd.brake;
    }
    bt::require(cmd.phase == rider::LaunchPhase::accelerate && cmd.brake == 0., "accelerate at brake 0");
    rider::LaunchPlan weak(table(), kPeriod);
    for (int i = 0; i <= 50; ++i) cmd = weak.update(in(standing(.2)));
    bt::require(cmd.phase == rider::LaunchPhase::release && cmd.preload_missed,
                "the hold limit releases with preload_missed");
}

// Upshift protocol: request at the table cadence, kept while deferred,
// retired by the shift count, then blocked until the felt cog changes;
// a rejection retires and blocks at that cog; no request during a cut.
void test_upshift_request_follows_the_shifter() {
    auto plan = accelerating();
    auto p = standing();
    p.senses.speed_mps = 1.;
    p.crank_rpm = 85.;
    rider::ShiftFeedback fb{.response = rider::ShiftResponse::none, .shift_count = 3, .cut_active = false};
    auto cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up && cmd.owns_shifting, "the table cadence raises a request");
    fb.response = rider::ShiftResponse::deferred;
    cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up, "a deferred request is kept");
    fb = {.response = rider::ShiftResponse::deferred, .shift_count = 4, .cut_active = true};
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "a committed shift retires the request even under a deferred label");
    fb.cut_active = false;
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "the delayed cadence of the old cog asks nothing until the felt cog changes");
    p.senses.rear_teeth = 45;
    cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up, "the new felt cog unblocks the next request");
    fb.response = rider::ShiftResponse::rejected;
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "a rejection retires the request");
    fb.response = rider::ShiftResponse::none;
    for (int i = 0; i < 5; ++i) cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "a rejected cog is not asked again");
    p.senses.rear_teeth = 39;
    fb.cut_active = true;
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "no request while a cut runs");
}

// Handover: the request is retired at entry, shifting stays with the plan
// until the cut ends, then the weight falls linearly to 0 and the plan is done.
void test_handover_waits_for_the_cut_then_blends() {
    auto plan = accelerating();
    auto p = standing();
    p.senses.speed_mps = 3.5;
    p.crank_rpm = 85.;
    rider::ShiftFeedback fb{.response = rider::ShiftResponse::none, .shift_count = 0, .cut_active = false};
    auto cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up, "a request is pending before the end criteria hold");
    fb = {.response = rider::ShiftResponse::accepted, .shift_count = 1, .cut_active = true};
    for (int i = 0; i < 6; ++i) cmd = plan.update(in(p, fb));
    bt::require(cmd.phase == rider::LaunchPhase::handover && !cmd.shift_up && cmd.owns_shifting &&
                    cmd.weight == 1.,
                "the end criteria start the handover, which keeps shifting during the cut");
    fb.cut_active = false;
    cmd = plan.update(in(p, fb));
    bt::require(cmd.phase == rider::LaunchPhase::blend && !cmd.owns_shifting && cmd.weight == 1.,
                "the end of the cut passes shifting to the automaton");
    double previous = 1.;
    for (int i = 1; i <= 10; ++i) {
        cmd = plan.update(in(p, fb));
        bt::require(cmd.weight < previous && std::abs(cmd.weight - (1. - i / 10.)) < 1e-15,
                    "the weight falls linearly over handover_s");
        previous = cmd.weight;
    }
    bt::require(cmd.phase == rider::LaunchPhase::done && cmd.weight == 0. && cmd.brake == 0., "done");
}

// A stop after the handover re-arms the plan; a bike rolling backward
// fast does not count as standing.
void test_restart_after_a_stop_and_backward_roll() {
    auto plan = accelerating();
    auto p = standing();
    p.senses.speed_mps = 3.5;
    p.crank_rpm = 70.;
    for (int i = 0; i < 30; ++i) (void)plan.update(in(p));
    bt::require(plan.command().phase == rider::LaunchPhase::done, "the plan finishes");
    auto rolling = standing();
    rolling.senses.speed_mps = -.5;
    bt::require(plan.update(in(rolling)).phase == rider::LaunchPhase::done, "rolling back is not standing");
    const auto cmd = plan.update(in(standing(.2)));
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.brake == 1. && std::abs(cmd.lean_target_rad - .2) < 1e-15,
                "a stop re-arms with the lean of the felt grade");
}

// An ineligible tick cancels the plan from any phase; a switched-off plan
// never leaves idle and does not arm at the start.
void test_ineligible_ticks_cancel_and_the_switch_keeps_idle() {
    auto plan = accelerating();
    auto off = in(standing());
    off.eligible = false;
    auto cmd = plan.update(off);
    bt::require(cmd.phase == rider::LaunchPhase::idle && cmd.brake == 0. && cmd.weight == 0. && !cmd.owns_shifting,
                "an ineligible tick cancels to idle");
    bt::require(plan.state() == rider::LaunchPlan::State{}, "the cancelled state is the initial one");
    auto t = table();
    t.plan_enabled = false;
    rider::LaunchPlan disabled(t, kPeriod);
    bt::require(!disabled.arm_start(.1), "a switched-off plan does not arm");
    for (int i = 0; i < 20; ++i) cmd = disabled.update(in(standing()));
    bt::require(cmd.phase == rider::LaunchPhase::idle && cmd.brake == 0. && cmd.weight == 0.,
                "a switched-off plan stays idle");
}

// The state document round-trips mid-hold and mid-blend, and a restored
// copy continues bitwise; rejected states leave the plan unchanged.
void test_codec_round_trip_continuation_and_rejected_restores() {
    rider::LaunchPlan hold(table(), kPeriod);
    for (int i = 0; i < 7; ++i) (void)hold.update(in(standing()));
    auto blend = accelerating();
    auto p = standing();
    p.senses.speed_mps = 3.5;
    p.crank_rpm = 70.;
    for (int i = 0; i < 10; ++i) (void)blend.update(in(p));
    bt::require(blend.command().phase == rider::LaunchPhase::blend, "the second plan is blending");
    for (rider::LaunchPlan *source : {&hold, &blend}) {
        const auto doc = rider::encode_launch_state(source->state());
        rider::LaunchPlan copy(table(), kPeriod);
        copy.restore(rider::decode_launch_state(doc, "launch"));
        bt::require(text(copy) == text(*source), "the document restores the state");
        for (int i = 0; i < 5; ++i) {
            const auto a = source->update(in(p));
            const auto b = copy.update(in(p));
            bt::require(a.phase == b.phase && a.weight == b.weight && a.effort_nm == b.effort_nm,
                        "the restored plan continues bitwise");
        }
    }
    rider::LaunchPlan fresh(table(), kPeriod);
    const std::string before = text(fresh);
    auto bad = hold.state();
    bad.brake = .5;
    rejects([&] { fresh.restore(bad); }, "brake", "a half brake while holding");
    bad = hold.state();
    bad.phase_ticks = 50;
    rejects([&] { fresh.restore(bad); }, "phase_ticks", "a hold past its limit");
    bad = hold.state();
    bad.shift_up = true;
    rejects([&] { fresh.restore(bad); }, "shift_up", "a request outside accelerate");
    bad = rider::LaunchPlan::State{};
    bad.effort_nm = 1.;
    rejects([&] { fresh.restore(bad); }, "idle", "an idle plan that is not reset");
    bt::require(text(fresh) == before, "rejected restores change nothing");
    // A complete document with an unknown phase: the exact-key check passes,
    // so the phase name is what fails (review 1, M6).
    auto fields = bike::io::as_object(rider::encode_launch_state(hold.state()), "launch");
    for (auto &[key, value] : fields)
        if (key == "phase") value = bike::io::Value{std::string("soaring")};
    const bike::io::Value doc{std::move(fields)};
    rejects([&] { (void)rider::decode_launch_state(doc, "launch"); }, "unknown launch phase", "an unknown phase name");
}

// Nonfinite percepts and a negative shift count are rejected before any change.
void test_inputs_are_validated() {
    rider::LaunchPlan plan(table(), kPeriod);
    (void)plan.update(in(standing()));
    const std::string before = text(plan);
    auto p = standing();
    p.human_crank_torque_nm = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { (void)plan.update(in(p)); }, "launch plan percepts", "a NaN felt torque");
    rider::ShiftFeedback fb{.response = rider::ShiftResponse::none, .shift_count = -1, .cut_active = false};
    rejects([&] { (void)plan.update(in(standing(), fb)); }, "shift count", "a negative shift count");
    bt::require(text(plan) == before, "rejected inputs change nothing");
    rejects([&] { (void)rider::shift_response("maybe"); }, "rider_response", "an unknown shifter label");
}

} // namespace

int main() {
    return bt::run([] {
        test_hold_brakes_and_ramps_to_the_surface_cap();
        test_no_percept_and_caller_brake_freeze_the_hold();
        test_release_by_felt_torque_and_by_timeout();
        test_upshift_request_follows_the_shifter();
        test_handover_waits_for_the_cut_then_blends();
        test_restart_after_a_stop_and_backward_roll();
        test_ineligible_ticks_cancel_and_the_switch_keeps_idle();
        test_codec_round_trip_continuation_and_rejected_restores();
        test_inputs_are_validated();
    });
}
```

  Расчёт сроков в тестах: `accelerating()` — тик 1 арм, тик 2 нажим 10 Н·м → `release` (тормоз 1), тики 3–6
  тормоз 0,75/0,5/0,25/0 → `accelerate`. В `test_handover_…`: `end_ok_ticks` растёт с первого тика со скоростью 3,5;
  `> 5` — на шестом тике, поэтому после шести тиков фаза `handover`. В `test_restart_…` 30 тиков хватает на
  `handover` (6 тиков), `blend` (1 + 10 тиков) и `done`. Если `parse_json` в `io/json.hpp` называется иначе —
  взять функцию разбора, которую используют соседние тесты кодеков.

  В `test_seated_posture_program.cpp` — крючки:

```cpp
// Launch part 3b: without an override the program's arithmetic is bitwise
// the part-3a one; an override at weight 1 replaces the grade target and
// freezes the load trim; at a partial weight the targets mix; every
// override still obeys the lean rate; an invalid override is rejected.
void test_lean_override_hook() {
    rider::SeatedPostureProgram plain(enabled()), hooked(enabled()), owned(enabled()), mixed(enabled());
    for (int i = 0; i < 400; ++i) {
        const double t = i * .01;
        const double a = plain.update(t, .15, .01, .2);
        const double b = hooked.update(t, .15, .01, .2, std::nullopt, std::nullopt, std::nullopt, 0., std::nullopt);
        bt::require(std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b),
                    "an absent override is bitwise the old program");
        const double before = owned.lean_rad();
        (void)owned.update(t, .15, .01, .2, std::nullopt, std::nullopt, std::nullopt, 0.,
                           rider::LeanOverride{.target_rad = .3, .weight = 1.});
        bt::require(std::abs(owned.lean_rad() - before) <= enabled().lean_rate_rad_s * .01 + 1e-15,
                    "the override obeys the lean rate");
        (void)mixed.update(t, .15, .01, .2, std::nullopt, std::nullopt, std::nullopt, 0.,
                           rider::LeanOverride{.target_rad = .3, .weight = .5});
    }
    bt::require(std::abs(owned.lean_rad() - .3) < 1e-9 && owned.state().trim_rad == 0.,
                "weight 1 holds the plan target and freezes the trim");
    bt::require(mixed.lean_rad() < owned.lean_rad() && mixed.lean_rad() > plain.lean_rad() - 1.,
                "a partial weight mixes the targets");
    rejects([&] { (void)owned.update(4., .15, .01, .2, std::nullopt, std::nullopt, std::nullopt, 0.,
                                     rider::LeanOverride{.target_rad = .3, .weight = 0.}); },
            "launch override", "a zero weight");
    rejects([&] { (void)owned.update(4., .15, .01, .2, std::nullopt, std::nullopt, std::nullopt, 0.,
                                     rider::LeanOverride{.target_rad = std::numeric_limits<double>::infinity(),
                                                         .weight = 1.}); },
            "launch override", "an infinite target");
}

// Launch part 3b: spend_surge = false neither spends nor recovers the surge
// budget and returns the sustained power.
void test_surge_is_not_spent_on_request() {
    rider::SeatedClimbPolicy policy(enabled());
    const double before = policy.state().surge_budget_s_left;
    const double power = policy.power_target_w(.3, .01, false);
    bt::require(power == enabled().target_crank_power_w && policy.state().surge_budget_s_left == before,
                "a held surge budget stays where it was");
}
```

  В `test_seated_posture_program.cpp` помощника `rejects` нет: добавить его в безымянное пространство файла
  дословной копией `test_launch_preparation.cpp:44-54` (ревью 1, M2). `enabled()` — помощник файла с включёнными намерениями (`test_seated_posture_program.cpp:67-81`); если его
  период не 0,01 — подставить его период вместо `.01`. Если `lean_rate_rad_s` названо иначе — поле темпа наклона
  `IntentConfig`. Проверка «смесь меньше плана и больше обычной цели» записана так, чтобы не зависеть от знака
  обычной цели; при неудаче исполнитель приводит числа, а не ослабляет проверку.

- [ ] **Шаг 2.** `mise run test 'launch_plan|seated_posture_program'` — сборка падает.
- [ ] **Шаг 3. Код.**
  - `launch_plan.cpp`:

```cpp
// rider/launch_plan.cpp — see launch_plan.hpp and plan 3b decisions 4-7, 11.
#include "launch_plan.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace rider {
namespace {

[[noreturn]] void bad_state(std::string_view field, std::string_view why) {
    throw std::invalid_argument("launch plan state " + std::string(field) + ": " + std::string(why));
}

std::int64_t ticks(double seconds, double period_s) {
    return static_cast<std::int64_t>(std::llround(seconds / period_s));
}

} // namespace

const char *launch_phase_name(LaunchPhase phase) {
    switch (phase) {
    case LaunchPhase::idle: return "idle";
    case LaunchPhase::hold: return "hold";
    case LaunchPhase::release: return "release";
    case LaunchPhase::accelerate: return "accelerate";
    case LaunchPhase::handover: return "handover";
    case LaunchPhase::blend: return "blend";
    case LaunchPhase::done: return "done";
    }
    throw std::invalid_argument("launch phase outside the enum");
}

LaunchPhase launch_phase(std::string_view name) {
    for (const LaunchPhase phase : {LaunchPhase::idle, LaunchPhase::hold, LaunchPhase::release,
                                    LaunchPhase::accelerate, LaunchPhase::handover, LaunchPhase::blend,
                                    LaunchPhase::done})
        if (name == launch_phase_name(phase)) return phase;
    throw std::invalid_argument("unknown launch phase " + std::string(name));
}

ShiftResponse shift_response(std::string_view label) {
    if (label == "none") return ShiftResponse::none;
    if (label == "accepted") return ShiftResponse::accepted;
    if (label == "deferred") return ShiftResponse::deferred;
    if (label == "rejected") return ShiftResponse::rejected;
    throw std::invalid_argument("unknown shifter rider_response " + std::string(label));
}

LaunchPlan::LaunchPlan(const LaunchConfig &config, double period_s) : config_(config), period_s_(period_s) {
    validate_launch_plan_context(config_, period_s_);
    release_ticks_ = ticks(config_.brake_release_s, period_s_);
    max_hold_ticks_ = ticks(config_.max_hold_s, period_s_);
    end_hold_ticks_ = ticks(config_.end_hold_s, period_s_);
    max_launch_ticks_ = ticks(config_.max_launch_s, period_s_);
    handover_ticks_ = ticks(config_.handover_s, period_s_);
}

void LaunchPlan::reset() { s_ = State{}; }

bool LaunchPlan::arm_start(double lean_rad) {
    if (!config_.plan_enabled) return false;
    if (!std::isfinite(lean_rad)) throw std::invalid_argument("launch plan start lean must be finite");
    State n;
    arm(n, lean_rad);
    s_ = n;
    return true;
}

void LaunchPlan::validate(const LaunchInputs &in) {
    if (in.percepts) {
        const Percepts &p = *in.percepts;
        if (!std::isfinite(p.senses.speed_mps) || !std::isfinite(p.senses.grade_ahead) ||
            !std::isfinite(p.crank_rpm) || !std::isfinite(p.human_crank_torque_nm))
            throw std::invalid_argument("launch plan percepts must be finite");
    }
    if (in.shift.shift_count < 0) throw std::invalid_argument("launch plan shift count must be nonnegative");
}

double LaunchPlan::cap(SurfaceLabel surface) const {
    switch (surface) {
    case SurfaceLabel::asphalt: return config_.effort_cap_asphalt_nm;
    case SurfaceLabel::hardpack: return config_.effort_cap_hardpack_nm;
    case SurfaceLabel::loose: return config_.effort_cap_loose_nm;
    case SurfaceLabel::wet: return config_.effort_cap_wet_nm;
    case SurfaceLabel::scree: return config_.effort_cap_scree_nm;
    }
    throw std::invalid_argument("launch plan surface outside the enum");
}

double LaunchPlan::ramp(double effort, const Percepts &p) const {
    return std::min(effort + config_.effort_rate_nm_s * period_s_, cap(p.senses.surface_front));
}

bool LaunchPlan::standstill(const Percepts &p) const {
    return std::abs(p.senses.speed_mps) <= config_.standstill_speed_mps &&
           p.crank_rpm <= config_.standstill_crank_rpm;
}

void LaunchPlan::arm(State &n, double lean_rad) const {
    n = State{};
    n.phase = LaunchPhase::hold;
    n.brake = 1.;
    n.lean_target_rad = lean_rad;
}

void LaunchPlan::to_release(State &n) const {
    n.phase = LaunchPhase::release;
    n.phase_ticks = 0;
    if (release_ticks_ == 0) {
        n.brake = 0.;
        n.phase = LaunchPhase::accelerate;
    }
}

void LaunchPlan::shifting(State &n, const Percepts &p, const ShiftFeedback &fb) const {
    if (n.shift_up && (fb.shift_count > n.request_count || fb.response == ShiftResponse::rejected)) {
        n.shift_up = false;
        n.blocked_teeth = n.request_teeth;
    }
    if (!n.shift_up && !fb.cut_active && p.crank_rpm >= config_.upshift_crank_rpm &&
        p.senses.rear_teeth != n.blocked_teeth) {
        n.shift_up = true;
        n.request_count = fb.shift_count;
        n.request_teeth = p.senses.rear_teeth;
    }
}

LaunchCommand LaunchPlan::update(const LaunchInputs &in) {
    validate(in);
    if (!config_.plan_enabled) return command();
    if (!in.eligible) {
        s_ = State{};
        return command();
    }
    State n = s_;
    const std::optional<Percepts> &p = in.percepts;
    switch (n.phase) {
    case LaunchPhase::idle:
    case LaunchPhase::done:
        if (p && standstill(*p)) arm(n, prepare_launch(config_, p->senses.grade_ahead).lean_rad);
        break;
    case LaunchPhase::hold:
        if (!p || in.external_brake) break;   // the rider waits: no press, no clock
        ++n.phase_ticks;
        n.effort_nm = ramp(n.effort_nm, *p);
        if (p->human_crank_torque_nm >= release_torque_nm(config_, p->senses.grade_ahead)) {
            to_release(n);
        } else if (n.phase_ticks >= max_hold_ticks_) {
            n.preload_missed = true;
            to_release(n);
        }
        break;
    case LaunchPhase::release:
        ++n.phase_ticks;
        ++n.launch_ticks;
        if (p) n.effort_nm = ramp(n.effort_nm, *p);
        n.brake = std::max(0., 1. - static_cast<double>(n.phase_ticks) / static_cast<double>(release_ticks_));
        if (n.brake == 0.) {
            n.phase = LaunchPhase::accelerate;
            n.phase_ticks = 0;
        }
        break;
    case LaunchPhase::accelerate:
        ++n.launch_ticks;
        if (p) {
            n.effort_nm = ramp(n.effort_nm, *p);
            shifting(n, *p, in.shift);
            n.end_ok_ticks = p->senses.speed_mps >= config_.end_speed_mps && p->crank_rpm >= config_.end_crank_rpm
                                 ? std::min(n.end_ok_ticks + 1, end_hold_ticks_ + 1)
                                 : 0;
        }
        if (n.end_ok_ticks > end_hold_ticks_ || n.launch_ticks >= max_launch_ticks_) {
            n.phase = LaunchPhase::handover;
            n.shift_up = false;   // decision 5: the request is retired at the handover
        }
        break;
    case LaunchPhase::handover:
        if (p) n.effort_nm = ramp(n.effort_nm, *p);
        if (!in.shift.cut_active) {
            n.phase = LaunchPhase::blend;
            n.blend_ticks = 0;
            if (handover_ticks_ == 0) n = State{.phase = LaunchPhase::done};
        }
        break;
    case LaunchPhase::blend:
        if (p) n.effort_nm = ramp(n.effort_nm, *p);
        ++n.blend_ticks;
        if (n.blend_ticks >= handover_ticks_) n = State{.phase = LaunchPhase::done};
        break;
    }
    s_ = n;
    return command();
}

LaunchCommand LaunchPlan::command() const {
    LaunchCommand c{.phase = s_.phase, .brake = s_.brake, .effort_nm = s_.effort_nm,
                    .lean_target_rad = s_.lean_target_rad, .shift_up = s_.shift_up,
                    .preload_missed = s_.preload_missed};
    switch (s_.phase) {
    case LaunchPhase::idle:
    case LaunchPhase::done:
        c.weight = 0.;
        break;
    case LaunchPhase::blend:
        c.weight = 1. - static_cast<double>(s_.blend_ticks) / static_cast<double>(handover_ticks_);
        break;
    default:
        c.weight = 1.;
        c.owns_shifting = true;
        break;
    }
    return c;
}

LaunchPlan::State LaunchPlan::state() const { return s_; }

void LaunchPlan::restore(const State &s) {
    const auto max_cap = std::max({config_.effort_cap_asphalt_nm, config_.effort_cap_hardpack_nm,
                                   config_.effort_cap_loose_nm, config_.effort_cap_wet_nm,
                                   config_.effort_cap_scree_nm});
    if (s.phase_ticks < 0 || s.launch_ticks < 0 || s.end_ok_ticks < 0 || s.blend_ticks < 0 ||
        s.request_count < 0)
        bad_state("phase_ticks", "negative counter");
    if (!std::isfinite(s.effort_nm) || s.effort_nm < 0. || s.effort_nm > max_cap)
        bad_state("effort_nm", "outside [0, the largest cap]");
    if (!std::isfinite(s.lean_target_rad)) bad_state("lean_target_rad", "must be finite");
    if (s.request_teeth < 0 || s.blocked_teeth < 0) bad_state("request_teeth", "negative cog");
    if (s.shift_up && s.phase != LaunchPhase::accelerate) bad_state("shift_up", "a request outside accelerate");
    if (s.shift_up && s.request_teeth < 3) bad_state("request_teeth", "a request without its felt cog");
    switch (s.phase) {
    case LaunchPhase::idle:
    case LaunchPhase::done:
        if (s != State{.phase = s.phase}) bad_state(launch_phase_name(s.phase), "an idle or done plan is reset");
        break;
    case LaunchPhase::hold:
        if (s.brake != 1.) bad_state("brake", "the hold brakes fully");
        if (s.phase_ticks >= max_hold_ticks_) bad_state("phase_ticks", "the hold is past its limit");
        if (s.launch_ticks != 0 || s.end_ok_ticks != 0 || s.blend_ticks != 0 || s.preload_missed ||
            s.request_count != 0 || s.request_teeth != 0 || s.blocked_teeth != 0)
            bad_state("hold", "a hold with launch counters, a request memory or an event");
        break;
    case LaunchPhase::release:
        if (release_ticks_ == 0 || s.phase_ticks >= release_ticks_ || s.launch_ticks != s.phase_ticks)
            bad_state("phase_ticks", "outside the release window");
        if (s.brake != std::max(0., 1. - static_cast<double>(s.phase_ticks) / static_cast<double>(release_ticks_)))
            bad_state("brake", "off the release ramp");
        if (s.end_ok_ticks != 0 || s.blend_ticks != 0) bad_state("release", "end or blend counters");
        break;
    case LaunchPhase::accelerate:
    case LaunchPhase::handover:
    case LaunchPhase::blend:
        if (s.brake != 0.) bad_state("brake", "released after the release");
        if (s.launch_ticks < release_ticks_) bad_state("launch_ticks", "shorter than the release");
        if (s.phase == LaunchPhase::accelerate &&
            (s.end_ok_ticks > end_hold_ticks_ || s.launch_ticks >= max_launch_ticks_))
            bad_state("end_ok_ticks", "accelerate past its end");
        if (s.phase == LaunchPhase::blend && (handover_ticks_ == 0 || s.blend_ticks >= handover_ticks_))
            bad_state("blend_ticks", "outside the handover window");
        if (s.phase != LaunchPhase::blend && s.blend_ticks != 0) bad_state("blend_ticks", "outside blend");
        break;
    default:
        bad_state("phase", "outside the enum");
    }
    s_ = s;
}

} // namespace rider
```

  - Кодеки в `senses_codec.cpp` рядом с кодеками реакции, теми же помощниками (`mapping`, `ncd::exact`, `keys`,
    `real_value`, `integer_value`, `finite_real`, `integer`, `boolean`, `sequence`):

```cpp
bio::Value encode_launch_state(const LaunchPlan::State &s) {
    bio::Value::Object out;
    out.emplace_back("phase", bio::Value{std::string(launch_phase_name(s.phase))});
    out.emplace_back("phase_ticks", integer_value(s.phase_ticks));
    out.emplace_back("launch_ticks", integer_value(s.launch_ticks));
    out.emplace_back("end_ok_ticks", integer_value(s.end_ok_ticks));
    out.emplace_back("blend_ticks", integer_value(s.blend_ticks));
    out.emplace_back("effort_nm", real_value(s.effort_nm));
    out.emplace_back("brake", real_value(s.brake));
    out.emplace_back("lean_target_rad", real_value(s.lean_target_rad));
    out.emplace_back("shift_up", bio::Value{s.shift_up});
    out.emplace_back("request_count", integer_value(s.request_count));
    out.emplace_back("request_teeth", integer_value(s.request_teeth));
    out.emplace_back("blocked_teeth", integer_value(s.blocked_teeth));
    out.emplace_back("preload_missed", bio::Value{s.preload_missed});
    return bio::Value{std::move(out)};
}

LaunchPlan::State decode_launch_state(const bio::Value &value, std::string_view path) {
    const Dict d = mapping(VNode{&value}, path);
    ncd::exact(d, keys("phase", "phase_ticks", "launch_ticks", "end_ok_ticks", "blend_ticks", "effort_nm",
                       "brake", "lean_target_rad", "shift_up", "request_count", "request_teeth",
                       "blocked_teeth", "preload_missed"));
    LaunchPlan::State s;
    try {
        s.phase = launch_phase(d["phase"].string(path));
    } catch (const std::invalid_argument &) {
        invalid(path, "unknown launch phase");
    }
    s.phase_ticks = d["phase_ticks"].integer(path);
    s.launch_ticks = d["launch_ticks"].integer(path);
    s.end_ok_ticks = d["end_ok_ticks"].integer(path);
    s.blend_ticks = d["blend_ticks"].integer(path);
    s.effort_nm = d["effort_nm"].finite_real(path);
    s.brake = d["brake"].finite_real(path);
    s.lean_target_rad = d["lean_target_rad"].finite_real(path);
    s.shift_up = d["shift_up"].boolean(path);
    s.request_count = d["request_count"].integer(path);
    s.request_teeth = d["request_teeth"].integer32(path);
    s.blocked_teeth = d["blocked_teeth"].integer32(path);
    s.preload_missed = d["preload_missed"].boolean(path);
    return s;
}
```

    Таблица — `encode_launch_config`/`decode_launch_config`: 24 ключа в порядке структуры; массивы — как массивы
    `real_value`/`integer_value` (`start_cog_teeth` — целые); `decode` — `ncd::exact` на 24 ключа, затем
    `validate_launch_table`. Если в файле у строки другое имя помощника (`string`), взять то, которым
    `decode_reaction_state` читает `phase`. `integer32` — помощник файла (`senses_codec.cpp:146-152`), он отвергает значения вне `int`.
  - `intent.hpp/.cpp` — крючки:
    - `SeatedPostureProgram::update(..., double extra_target_rad = 0., std::optional<LeanOverride> launch = std::nullopt)`.
      В начале к прежним проверкам:

```cpp
    if (launch.has_value() &&
        !(std::isfinite(launch->target_rad) && launch->weight > 0. && launch->weight <= 1.))
        throw std::invalid_argument("posture launch override must be finite with a weight in (0, 1]");
```

      и вместо строк `grade_and_pulse` … `target` (`intent.cpp:182-201`):

```cpp
    // Launch part 3b, decision 8: the plan's lean target replaces (weight 1)
    // or mixes with (0 < weight < 1) the grade target; without an override
    // every expression below is the part-3a one.
    const bool plan_owns = launch.has_value() && launch->weight == 1.;
    double base = config_.lean_gain * std::atan(road_grade);
    if (launch.has_value())
        base = plan_owns ? launch->target_rad : launch->weight * launch->target_rad + (1. - launch->weight) * base;
    const double grade_and_pulse = base + extra;
    if (delayed_load_share_.has_value() && !plan_owns) {
        // ... the trim update, unchanged ...
    }
    const double trim = plan_owns ? 0. : launch.has_value() ? (1. - launch->weight) * trim_rad_ : trim_rad_;
    const double target = std::max(backward_limit, std::min(forward_limit, grade_and_pulse + trim));
```

    - `SeatedClimbPolicy::power_target_w(double preview_grade, double dt_s, bool spend_surge = true)`: первой
      строкой после проверок входа `if (!spend_surge) return config_.target_crank_power_w;` (бюджет не тратится и
      не восстанавливается).
    - `SeatedClimbPolicy::update(..., double extra_lean_rad = 0., bool spend_surge = true, std::optional<LeanOverride> launch = std::nullopt)`:
      `power_target_w(preview, dt, spend_surge)`; `program_.update(..., extra_lean_rad, launch)`.
  - `Phase3Ports.cmake` — после `front_lift_reaction`:

```cmake
# Launch part 3b — the launch plan node on synthetic percepts.
bike_add_reference_test(launch_plan ENGINE
  SOURCES test_launch_plan.cpp LIBRARIES bike_app
  LABELS launch rider)
```

    (Если у соседних тестов есть объявленная длительность — та же форма, около 1 с.)
- [ ] **Шаг 4.** `perl -e 'alarm 2400; exec @ARGV' -- mise run test 'launch_plan|seated_posture_program|front_lift_reaction|seated_climb_surge|launch_preparation|seated_pedaling_cycle|realistic_pedelec_acceptance|native_rider_intent'`
  — новые зелёные, прежние как в базе (`native_rider_intent` красный с прежней причиной).
- [ ] **Шаг 5. Перезаписи** — по правилу; ожидается «нет».
- [ ] **Шаг 6. Коммит и проверка.** `feat(rider): the launch plan node and its posture and surge hooks`;
  `check_task.sh`, `EXPECT` пусто, с `ab`.

### Задача 3. План в намерениях райдера, шаг, документы и запись

**Файлы:**
- Изменить:
  - `native/src/rider/intent.{hpp,cpp}` — `Core::launch`, `Core::commands`, конструктор, `reset`,
    `arm_standstill_start`, `tick`, `output`, `resolve_full`, `state`/`restore`, `launch()`;
  - `native/src/rider/state_codec.cpp` — ключ `launch` в `rider_intent` (декодирование `:2279-2300`, кодирование
    `:2336-2414`);
  - `native/src/runtime/config.hpp` (`RuntimeConfig::rider_launch`), `native/src/runtime/step.hpp`
    (`StepConfig::launch`), `native/src/runtime/bootstrap.cpp:253,302`, `native/src/runtime/project_config.cpp:506`,
    `native/src/runtime/runtime.cpp:352-353`;
  - `native/src/runtime/step.cpp` — конструктор (`:1024`), вызов `resolve_full` (`:1264-1269`), канал
    `rider_launch` после `rider_reaction` (`:2055-2067`);
  - `native/src/initialization/finalize.cpp:72` — `arm_standstill_start`;
  - каталог: `native/resources/recording/catalog-v1.json`, `native/src/recording/catalog_data.inc`,
    `tests/fixtures/recording/schema.json`, `native/tests/recording/test_catalog.cpp` (512 → 520),
    `native/tests/reference/test_recording_catalog.cpp` (набор таблиц `:110-114` и тест канала);
  - конструкторы `RiderIntent` в тестах из описи; документы решения 12 (скриптами).
- Создать: `native/tests/reference/test_launch_intent.cpp`.
- Тест: `test_launch_intent.cpp` (намерения), `test_rider_senses.cpp` (поездка с включённым планом),
  `test_recording_catalog.cpp` (канал).

**Интерфейсы** (`intent.hpp`):

```cpp
// Launch part 3b: what the step tells the resolver about the launch.
struct LaunchContext {
    ShiftFeedback shift;          // the shifter after the previous physics step
    bool effort_mode = false;     // drive_mode is articulated_effort or crank_effort
    bool external_brake = false;  // the caller brakes this step
};

// RiderIntent(IntentConfig config, FrontLiftReactionConfig reaction, LaunchConfig launch, double dt_s);
// [[nodiscard]] IntentResolution resolve_full(..., std::optional<double> min_lean_rad = std::nullopt,
//                                             const LaunchContext &launch = {});
// (resolve() gains the same last parameter and forwards it.)
// bool arm_standstill_start(double lean_rad);   // true when the plan armed
// [[nodiscard]] LaunchCommand launch() const;    // neutral when the intent is off
// const RiderCommands &commands() const noexcept { return core_->commands; }
// State gains: std::optional<LaunchPlan::State> launch;   // engaged exactly when the intent is enabled
```

- [ ] **Шаг 1. Тесты (падают)** — `test_launch_intent.cpp`:

```cpp
// Launch part 3b — the launch plan inside RiderIntent: the switched-off
// plan leaves the part-3a path bitwise, a standing start holds the brakes
// from step 0 through the first percept, the plan's effort and lean go
// through the intent with the reaction on top, the handover is continuous
// and within the lean rate, surge is not spent while the plan owns,
// ownership follows the caller, and the joint state restores.
#include "support/fixture_test.hpp"
#include "io/json.hpp"
#include "rider/front_lift_reaction.hpp"
#include "rider/intent.hpp"
#include "rider/launch_plan.hpp"
#include "rider/state_codec.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {

constexpr double kDt = .00125;

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

// The working profile's intent (period = dt, delay 120 ticks) and a
// switched-off reaction; copy the field values from enabled_intent() in
// test_front_lift_reaction.cpp.
rider::IntentConfig intent_config();
rider::FrontLiftReactionConfig switched_off_reaction();
// The preliminary launch table of decision 2 with plan_enabled as given.
rider::LaunchConfig launch_table(bool enabled);

rider::IntentControl automatic() { return rider::IntentControl{.rider_enabled = true}; }

struct Tick {
    rider::IntentResolution resolution;
    rider::RiderCommands commands;
};

// One physics step: the bike stands, the legs press `torque` against the brake.
Tick step(rider::RiderIntent &intent, std::int64_t k, double torque, const rider::LaunchContext &ctx,
          rider::IntentControl control = automatic(), double speed = 0., double crank_rate = 0.) {
    rider::IntentSignals signals{};
    signals.human_crank_torque_nm = torque;
    signals.crank_rate_rad_s = crank_rate;
    signals.specific_force_body_mps2 = {0., 0., 9.81};
    rider::RiderSenses senses{};
    senses.speed_mps = speed;
    senses.rear_teeth = 51;
    senses.surface_front = rider::SurfaceLabel::hardpack;
    senses.surface_ahead = rider::SurfaceLabel::hardpack;
    senses.grade_ahead = .15;
    auto r = intent.resolve_full(control, signals, senses, k, .15, .15, std::nullopt, true, true, std::nullopt, ctx);
    return {r, intent.commands()};
}

const rider::LaunchContext kEffort{.shift = {}, .effort_mode = true, .external_brake = false};

std::string encoded(const rider::RiderIntent &intent) {
    return bike::io::canonical_json(rider::encode_intent_state(intent.state()));
}

// A switched-off plan: commands stay the empty set and the resolution is
// bitwise the one of an intent whose plan cannot arm (part-3a path).
[[gnu::noinline]] void test_switched_off_plan_is_the_part_3a_path() {
    auto off = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(false), kDt);
    auto ref = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(false), kDt);
    bt::require(!off->arm_standstill_start(.1), "a switched-off plan does not arm");
    for (std::int64_t k = 0; k < 600; ++k) {
        const auto a = step(*off, k, 20., kEffort);
        const auto b = step(*ref, k, 20., rider::LaunchContext{});
        bt::require(a.commands.front_brake == 0. && a.commands.rear_brake == 0. && !a.commands.owns_shifting &&
                        !a.commands.shift_request,
                    "a switched-off plan commands nothing");
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        bt::require(std::bit_cast<std::uint64_t>(*a.resolution.control.human_torque_nm) ==
                            std::bit_cast<std::uint64_t>(*b.resolution.control.human_torque_nm) &&
                        a.resolution.effort_limited == b.resolution.effort_limited,
                    "the effort is the part-3a one");
        bt::require(off->launch().phase == rider::LaunchPhase::idle, "the plan stays idle");
    }
}

// A standing start: brake 1 from step 0 (before the first tick resolves)
// through the first percept, no press before the first percept, then the
// press ramps; the posture holds the prepared lean.
[[gnu::noinline]] void test_standing_start_holds_from_step_zero() {
    auto intent = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    intent->seed_lean(.08);
    bt::require(intent->arm_standstill_start(.08), "a switched-on plan arms at a standing start");
    bt::require(intent->commands().front_brake == 1. && intent->commands().rear_brake == 1. &&
                    intent->commands().owns_shifting,
                "the brakes are held before the first tick");
    const std::int64_t delay = std::llround(intent_config().reaction_delay_s / intent_config().period_s);
    for (std::int64_t k = 0; k <= delay; ++k) {
        const auto t = step(*intent, k, 0., kEffort);
        bt::require(t.commands.front_brake == 1. && t.commands.rear_brake == 1., "held through the first percept");
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        bt::require(k == delay || *t.resolution.control.human_torque_nm == 0., "no press before the first percept");
        bt::require(t.resolution.effort_limited, "the plan owns the effort");
    }
    double effort = 0.;
    for (std::int64_t k = delay + 1; k < delay + 200; ++k) {
        const auto t = step(*intent, k, 0., kEffort);
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        const double now = *t.resolution.control.human_torque_nm;
        bt::require(now >= effort && t.commands.front_brake == 1., "the press ramps while holding");
        effort = now;
    }
    bt::require(effort > 0., "the press has started");
}

// The handover is continuous: effort and lean change per tick no more
// than the ramp and the lean rate allow; surge is not spent while the plan
// owns; at done the effort is the ordinary policy's.
[[gnu::noinline]] void test_handover_is_continuous_and_surge_is_held() {
    auto intent = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    bt::require(intent->arm_standstill_start(0.), "armed");
    const double budget = intent->policy().state().surge_budget_s_left;
    double effort = 0., lean = 0.;
    bool done = false;
    for (std::int64_t k = 0; k < 20000 && !done; ++k) {
        // The legs press 40 N·m (above the 15 % release value), then the bike
        // rolls at 4 m/s and 70 rpm so the end criteria hold.
        const bool moving = k > 2000;
        const auto t = step(*intent, k, 40., kEffort, automatic(), moving ? 4. : 0., moving ? 7.33 : 0.);
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        const double now = *t.resolution.control.human_torque_nm;
        const double now_lean = t.resolution.control.posture->torso_lean_rad;
        if (k > 0) {
            bt::require(std::abs(now_lean - lean) <= intent_config().lean_rate_rad_s * kDt + 1e-12,
                        "the lean never moves faster than the lean rate");
            bt::require(std::abs(now - effort) <= 60. * 1.001, "the effort has no jump beyond one cap");
        }
        effort = now;
        lean = now_lean;
        if (intent->launch().weight > 0.)
            bt::require(intent->policy().state().surge_budget_s_left == budget, "surge is held while the plan owns");
        done = intent->launch().phase == rider::LaunchPhase::done;
    }
    bt::require(done, "the launch reaches done");
}

// Ownership: a caller effort cancels the plan (brakes 0, no shifting); a
// posture-only override does not; rider disabled cancels.
[[gnu::noinline]] void test_ownership_follows_the_caller() {
    auto intent = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    bt::require(intent->arm_standstill_start(0.), "armed");
    auto posture_only = automatic();
    posture_only.posture = rider::RiderPosture{};
    posture_only.posture->use_saddle = true;
    auto t = step(*intent, 0, 0., kEffort, posture_only);
    bt::require(t.commands.front_brake == 1. && intent->launch().phase == rider::LaunchPhase::hold,
                "a posture-only override keeps the plan");
    auto effort = automatic();
    effort.human_torque_nm = 10.;
    t = step(*intent, 1, 0., kEffort, effort);
    bt::require(t.commands.front_brake == 0. && !t.commands.owns_shifting &&
                    intent->launch().phase == rider::LaunchPhase::idle,
                "a caller effort cancels the plan");
    bt::require(intent->arm_standstill_start(0.), "re-armed");
    auto disabled = automatic();
    disabled.rider_enabled = false;
    t = step(*intent, 2, 0., kEffort, disabled);
    bt::require(t.commands.front_brake == 0. && intent->launch().phase == rider::LaunchPhase::idle,
                "a disabled rider cancels the plan");
    bt::require(intent->arm_standstill_start(0.), "re-armed again");
    t = step(*intent, 3, 0., rider::LaunchContext{.shift = {}, .effort_mode = false, .external_brake = false});
    bt::require(t.commands.front_brake == 0., "a non-effort drive mode cancels the plan");
}

// State: the document carries the plan, a restored copy continues bitwise
// (commands included), a probe changes nothing, and the joint rule
// rejects a plan clock ahead of the delivered percepts.
[[gnu::noinline]] void test_state_round_trip_probe_and_joint_rule() {
    auto a = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    bt::require(a->arm_standstill_start(0.), "armed");
    for (std::int64_t k = 0; k < 300; ++k) (void)step(*a, k, 5., kEffort);
    const std::string before = encoded(*a);
    rider::IntentSignals signals{};
    signals.specific_force_body_mps2 = {0., 0., 9.81};
    rider::RiderSenses senses{};
    senses.rear_teeth = 51;
    senses.surface_front = rider::SurfaceLabel::hardpack;
    senses.surface_ahead = rider::SurfaceLabel::hardpack;
    (void)a->resolve_full(automatic(), signals, senses, 300, .15, .15, std::nullopt, true, false, std::nullopt, kEffort);
    bt::require(encoded(*a) == before, "a probe changes no resolver state");
    auto b = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    b->restore(rider::decode_intent_state(rider::encode_intent_state(a->state()), "intent"));
    bt::require(encoded(*b) == encoded(*a) && b->commands().front_brake == a->commands().front_brake,
                "the document restores the plan and its commands");
    for (std::int64_t k = 300; k < 600; ++k) {
        const auto x = step(*a, k, 15., kEffort);
        const auto y = step(*b, k, 15., kEffort);
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        bt::require(std::bit_cast<std::uint64_t>(*x.resolution.control.human_torque_nm) ==
                            std::bit_cast<std::uint64_t>(*y.resolution.control.human_torque_nm) &&
                        x.commands.front_brake == y.commands.front_brake,
                    "the restored intent continues bitwise");
    }
    auto fresh = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    const std::string clean = encoded(*fresh);
    auto bad = std::make_unique<rider::RiderIntent::State>(a->state());
    bad->last_tick_step = -1;
    bad->perception = fresh->state().perception;
    bad->reaction = fresh->state().reaction;
    rejects([&] { fresh->restore(*bad); }, "launch", "a plan clock ahead of the delivered percepts");
    *bad = a->state();
    bad->launch.reset();
    rejects([&] { fresh->restore(*bad); }, "launch", "an enabled intent without plan state");
    bt::require(encoded(*fresh) == clean, "rejected restores change nothing");
}

} // namespace

int main() {
    return bt::run([] {
        test_switched_off_plan_is_the_part_3a_path();
        test_standing_start_holds_from_step_zero();
        test_handover_is_continuous_and_surge_is_held();
        test_ownership_follows_the_caller();
        test_state_round_trip_probe_and_joint_rule();
    });
}
```

  Три помощника (`intent_config`, `switched_off_reaction`, `launch_table`) исполнитель пишет полностью: значения
  `intent_config()` — те же, что у `enabled_intent()` в `test_front_lift_reaction.cpp`, но с `period_s = kDt`
  (если там другой период — взять его и `kDt` равным ему); `switched_off_reaction()` — как
  `kSwitchedOffReaction` из `test_native_rider_intent.cpp:42` с временами на сетке; `launch_table(enabled)` —
  предварительные числа решения 2. В `test_handover_…` каденс 7,33 рад/с — это 70 об/мин; если обычная политика
  при таких входах отвергает позу — привести сообщение, не ослабляя проверки.

  В `test_rider_senses.cpp` — поездка с включённым планом (то, что поездка показывает, печатается, а не
  требуется, кроме перечисленного):

```cpp
// Launch part 3b: the rider_launch channel on a standing 15 % start with
// the plan switched on (test-only): the rows start in hold with the
// brakes on, the phases follow the plan's order, the brake only falls,
// and the ride leaves the hold. Rollback, the release time and whether the
// handover is reached are printed for the calibration, not required.
[[gnu::noinline]] void test_launch_channel_on_a_standing_climb() {
    const auto edit = [](auto &physics) { physics.rider_launch.plan_enabled = true; };
    auto env = launch_ride::environment(launch_ride::track("[[0.0, 0.15], [60.0, 0.15]]", 60.), 8., edit, 1,
                                        runtime::ResearchCaptureMode::full);
    // ... read the rider_launch rows per interval exactly as reaction_rows() reads rider_reaction ...
}
```

  Тело теста: строки канала `rider_launch` по образцу `reaction_rows()` того же файла (`phase`, `brake`);
  требования — первая строка `hold` с `brake == 1`; переходы только из множества
  `{idle→idle, idle→hold, hold→hold, hold→release, hold→accelerate, release→release, release→accelerate,
  accelerate→accelerate, accelerate→handover, accelerate→blend, handover→handover, handover→blend, blend→blend,
  blend→done, done→done, done→hold}` (строки — на конце интервала, поэтому допускаются переходы через одну фазу,
  которые помещаются в интервал); `brake` не растёт, пока фаза не вернулась в `hold`; за 8 с фаза хотя бы раз не
  `hold`. Печать: наименьшее смещение назад, время начала `release`, достигнута ли `done`.

- [ ] **Шаг 2.** `mise run test 'launch_intent|rider_senses|recording_catalog'` — сборка падает.
- [ ] **Шаг 3. Код.**
  - `intent.hpp/.cpp`:
    - `Core`: `std::optional<LaunchPlan> launch;` и `RiderCommands commands;` (член `commands_` удаляется, копия и
      присваивание `RiderIntent` больше его не копируют).
    - Конструктор: при включённых намерениях после реакции
      `validate_launch_plan_context(launch, config_.period_s); core_->launch.emplace(launch, config_.period_s);`.
    - `reset()`: `if (core_->launch) core_->launch->reset(); core_->commands = {};`.
    - Новые функции:

```cpp
namespace {
RiderCommands commands_of(const LaunchCommand &c) {
    return {.front_brake = c.brake, .rear_brake = c.brake,
            .shift_request = c.shift_up ? std::optional{ShiftRequest::up} : std::nullopt,
            .owns_shifting = c.owns_shifting};
}
} // namespace

bool RiderIntent::arm_standstill_start(double lean_rad) {
    if (!core_->launch || !core_->launch->arm_start(lean_rad)) return false;
    core_->commands = commands_of(core_->launch->command());
    return true;
}

LaunchCommand RiderIntent::launch() const {
    return core_->launch ? core_->launch->command() : LaunchCommand{};
}
```

    - `tick` получает `const IntentControl &control` и `const LaunchContext &ctx` и после восприятия:

```cpp
    const SenseSample sample{.signals = signals, .senses = senses};
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) engaged exactly when the intent is enabled
    const std::optional<Percepts> percepts = core.perception->update(sample);
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) same engaged pair
    const double lean_add = core.reaction->update(percepts).lean_add_rad;
    // Launch part 3b, decisions 6-8: the plan runs on the same percepts; its
    // weight hands effort and lean target to the ordinary policy, which
    // keeps updating but spends no surge while the plan owns.
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) engaged exactly when the intent is enabled
    const LaunchCommand plan = core.launch->update(
        {.percepts = percepts, .shift = ctx.shift,
         .eligible = ctx.effort_mode && control.rider_enabled && !control.human_torque_nm.has_value(),
         .external_brake = ctx.external_brake});
    const bool owns = plan.weight > 0.;
    const std::optional<LeanOverride> lean =
        owns ? std::optional{LeanOverride{.target_rad = plan.lean_target_rad, .weight = plan.weight}} : std::nullopt;
    Intent intent = core.policy.update(signals, config_.period_s, road_grade, preview_grade, lean_limit_rad,
                                       min_lean_rad, lean_add, /*spend_surge=*/!owns, lean);
    if (owns)
        intent.effort_ceiling_nm = plan.weight == 1.
                                       ? plan.effort_nm
                                       : plan.weight * plan.effort_nm + (1. - plan.weight) * intent.effort_ceiling_nm;
    core.intent = intent;
    core.commands = commands_of(plan);
    core.last_tick_step = step;
```

    - `output`: внутри ветки `if (!control.human_torque_nm.has_value())` после множителя реакции
      `if (core.launch && core.launch->command().weight > 0.) limited = true;`.
    - `resolve_full(..., const LaunchContext &launch = {})` передаёт `control` и `launch` в оба вызова `tick`
      (пробный и настоящий).
    - `state()`: `.launch = core_->launch ? std::optional{core_->launch->state()} : std::nullopt`.
    - `restore()`: проверка присутствия `launch` (как у `reaction`, сообщение «rider intent launch state presence
      must match the enabled intent»); совместное правило (решение 11), после правила `percepts_seen`:

```cpp
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the presence check above engaged the plan
        const auto &plan = *state.launch;
        if (plan.launch_ticks > delivered ||
            (plan.phase == LaunchPhase::hold && plan.phase_ticks > delivered))
            throw std::invalid_argument("rider intent launch clock must not run ahead of the delivered percepts");
```

      затем `staged_->launch->restore(*state.launch)` вместе с остальными и
      `staged_->commands = commands_of(staged_->launch->command());` до обмена. При выключенных намерениях
      `staged_->commands = {}`.
  - `state_codec.cpp`: ключ `launch` — `decode_launch_state`/`encode_launch_state`, `null` при выключенных
    намерениях, как `reaction`; `ncd::exact` получает `"launch"`.
  - Документ запуска: `RuntimeConfig::rider_launch` (`bike::io::Value`, рядом с `rider_reaction`);
    `bootstrap.cpp` — `"rider_launch"` в списке ключей конфигурации после `"rider_reaction"` и
    `out.rider_launch = owned_mapping(root, "rider_launch");`; `project_config.cpp` —
    `{"rider_launch", rider::encode_launch_config(config.rider_launch)}` после `rider_reaction`;
    `runtime.cpp` — `out.launch = rider::decode_launch_config(config.rider_launch, "config.rider_launch");`;
    `StepConfig::launch` (`rider::LaunchConfig`).
  - `step.cpp`:
    - конструктор: `rider_intent_(config_.intent, config_.reaction, config_.launch, config_.timestep_s)`;
    - вызов `resolve_full` последним аргументом — контекст:

```cpp
            const auto &shifting = stepper_->drive().state().shifting;
            const rider::LaunchContext launch_context{
                .shift = {.response = rider::shift_response(shifting.rider_response),
                          .shift_count = shifting.shift_count,
                          .cut_active = shifting.cut_remaining_s > 1e-12},
                .effort_mode = drive_mode() == "articulated_effort" || drive_mode() == "crank_effort",
                .external_brake = caller_braking};
```

      где `caller_braking` — предикат тормоза вызывающей стороны этого шага: `resolved->caller_front > 0. ||
      resolved->caller_rear > 0.`, когда разрешение тормозов передано, иначе `front > 0. || rear > 0.` (имена — из
      `apply_forces_body`, `step.cpp:1283-1296`). Если `shift_count` в снимке не `std::int64_t` — привести.
    - канал после `rider_reaction`:

```cpp
        {
            // Launch part 3b — the launch plan on every interval, neutral
            // (idle, 0, 0, 0, 0, false) when the plan or the intent is off.
            const rider::LaunchCommand launch = rider_intent_.launch();
            WireObject fields;
            fields.emplace_back("phase", Wire(std::string(rider::launch_phase_name(launch.phase))));
            fields.emplace_back("brake", Wire(launch.brake));
            fields.emplace_back("effort_nm", Wire(launch.effort_nm));
            fields.emplace_back("weight", Wire(launch.weight));
            fields.emplace_back("lean_target_rad", Wire(launch.lean_target_rad));
            fields.emplace_back("preload_missed", Wire(launch.preload_missed));
            channels.emplace_back("rider_launch", Wire(std::move(fields)));
        }
```

  - `finalize.cpp` после `seed_lean`:

```cpp
    // Launch part 3b, decision 7: a standing start in an effort drive mode
    // holds the brakes from step 0; a moving start leaves the plan idle.
    if(context.physics.initial_speed_mps==0.&&(context.physics.drive_mode=="articulated_effort"||context.physics.drive_mode=="crank_effort"))
        (void)physical.rider_intent_.arm_standstill_start(context.preparation.lean_rad);
```

    (Если `drive_mode` в `SimulationPhysicsConfig` — `std::optional<std::string>`, сравнивать значение.)
  - Каталог: таблица `channel_rider_launch` по образцу `channel_rider_reaction` (`catalog-v1.json:8181-8300`):
    `claims: ["rider_launch"]`, два ключа — дословные копии ключей `channel_rider_reaction`, поля `phase`
    (строка), `brake`, `effort_nm` (Н·м), `weight`, `lean_target_rad` (рад), `preload_missed` (логическое), с
    описаниями. Та же таблица — в `catalog_data.inc` (вручную, как в части 3a: сырая строка делится на части не
    длиннее 64 КиБ, после правки собранный текст проверяется разбором JSON и сравнением с `catalog-v1.json` —
    раздел каталога задачи 3 плана 3a), в
    `tests/fixtures/recording/schema.json` и в ожидаемом наборе `test_recording_catalog.cpp:110-114`. Счёт
    `test_catalog.cpp:76` — 520, с комментарием «520 added channel_rider_launch (2 identity keys + 6 leaves)». Тест
    круга канала — по образцу `test_rider_reaction_channel_round_trips` (`:654-672`).
  - Тесты-конструкторы из описи получают выключенную таблицу:

```cpp
// Launch part 3b: the switched-off launch table (plan_enabled = false);
// its times sit on every test grid.
const rider::LaunchConfig kSwitchedOffLaunch = [] {
    rider::LaunchConfig t;
    t.start_cog_teeth = {51};
    t.start_pedal_angle_rad = 0.;
    t.start_lean_grades = {0.};
    t.start_lean_rad = {0.};
    t.plan_enabled = false;
    t.standstill_speed_mps = .1;
    t.standstill_crank_rpm = 5.;
    t.effort_rate_nm_s = 40.;
    t.effort_cap_asphalt_nm = t.effort_cap_hardpack_nm = t.effort_cap_loose_nm = t.effort_cap_wet_nm =
        t.effort_cap_scree_nm = 60.;
    t.release_torque_grades = {0., .2};
    t.release_torque_nm = {0., 30.};
    t.brake_release_s = .15;
    t.max_hold_s = 1.5;
    t.upshift_crank_rpm = 82.;
    t.end_speed_mps = 3.;
    t.end_crank_rpm = 60.;
    t.end_hold_s = .3;
    t.max_launch_s = 6.;
    t.handover_s = .9;
    return t;
}();
```

  - `Phase3Ports.cmake`: `launch_intent` по образцу `front_lift_reaction` (`LIBRARIES bike_app`, `LABELS launch rider`).
  - Документы — команды раздела «Опись» из `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/tools/README.md`
    (оркестратор кладёт их до задачи; ожидаемый вывод — в README). Скрипты не менять; при отказе — стоп с выводом.
- [ ] **Шаг 4. Тесты и скорость.**
  - `perl -e 'alarm 2400; exec @ARGV' -- mise run test 'launch_intent|launch_plan|front_lift_reaction|rider_senses|rider_perception|launch_preparation|seated_posture_program|seated_pedaling_cycle|realistic_pedelec_acceptance|io_state_codecs|recording_catalog|runtime_bootstrap|research_bootstrap|native_rider_intent'`
    — новые зелёные, прежние как в базе.
  - Скорость — как в части 3a (задача 3, шаг 4): две поездки по очереди, база — сборка `lane-flip`, вершина; в
    отчёт обе строки `real` и отношение.
- [ ] **Шаг 5. Перезаписи** — по правилу, с базой задачи 2.
- [ ] **Шаг 6. Коммит и проверка.** `feat(rider): the launch plan in the intent, the step and the recording`;
  `check_task.sh`, база — коммит задачи 2,
  `EXPECT=/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/expect-task3.tsv`, с `ab`.

### Задача 4. Калибровка (замер, без коммита кода)

Делается во временной копии на SSD по образцу `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch-baseline/`
(`probe.cpp`, `launch_metrics.py`, треки `track_*_start.toml`): копия профиля `viewer_physics_welded.toml` с
`plan_enabled = true` и таблицей типа райдера. Репозиторий не меняется.

**Таблицы трёх типов** оркестратор кладёт до задачи в
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/calibration/tables.toml`: поля `[rider_launch]` и
`[front_lift_reaction]` для осторожного, обычного и резкого, с пометкой каждого числа «по статье» (подтверждено
обеими сверками цитат), «по расчёту отчёта» или «подобрано». Пределы и пороги, которые сейчас задал пользователь
(предел нажима 60 Н·м, силы рук, седло), не меняются.

**Прогоны:** каждый тип × уклоны 0, 5, 10, 15, 20 % × старт с места, плюс старт после остановки на 5 % и 15 %,
плюс проверка на чужих случаях (8 и 12 %, `wet` и `loose`), по 20 с.

**Выход** — отчёт `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/calibration/report.md`, по каждому прогону:
- отрыв колеса (нагрузка ниже 50 Н дольше 50 мс) — да/нет, начало, длительность;
- буксовка (касательная / нормальная сила больше 0,9 μ дольше 50 мс; μ — материал заднего контакта);
- откат (наименьшее смещение назад до начала движения вперёд);
- каденс ног (сглаживание отсечки, `pedaling.cpp:50-56`) и пики сырого каденса шатуна и вала мотора;
- время до 20 км/ч от отпускания тормоза и от начала поездки — против 3,25 с базы;
- фазы плана: начало `release`, «нажим не набран», переключения (просьба, ответ), начало `blend` и `done`;
- окно для контроллера: от начала отрыва до начала действия реакции или опрокидывания;
- опора седла и стоп, силы рук, мощности суставов на каждом шаге — нарушения;
- для решения пользователя о признаке разгрузки рук: доля времени с `hand_press_n < hand_unload_n` без отрыва
  колеса и наименьшая сила рук на обычном подъёме.

- [ ] **Шаг 1.** Прогоны и отчёт.
- [ ] **Шаг 2. Остановка.** Числа и отчёт — пользователю. Без его «да» часть 4 не начинается.

### Задача 5. Итоговая проверка части 3b (оркестратор)

Как задача 4 части 3a (`2026-11-11-launch-3a-perception-reaction.md`), с папкой
`/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/final`, базой сравнения причин — итог части 3a
(`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch3a/final/reds_output.log`) со списком `expect-task3.tsv`, и
второй проверкой — против проверки задачи 3 без списка. Ожидание: `build 0`, `checks 0`, `ordinary 8` (22),
`slow 8` (10), `package 0`, `causes 0`, `causes-task3 0`, `ab 0`. Затем строка состояния в дорожной карте, запись в
журнал и коммит `docs(plan): launch part 3b is in the code`. В журнал — замер скорости задачи 3 и итоги калибровки.

## Ответы на ревью дорожной карты

| Пункт | Ответ |
|---|---|
| C2 — пустое восприятие в начале эпизода | Решение 7: план вооружается в `finalize` до первого тика, тормоз 1 с шага 0, пустое восприятие не разыменовывается; тест `test_standing_start_holds_from_step_zero` |
| C3 — конец среза и снятие просьбы | Решение 5: обратная связь автомата `{ответ, счёт, срез}`, принятие по счёту, отказ по метке, блок до смены звезды, снятие при входе в `handover`, передача владения только без среза; тесты `test_upshift_request_follows_the_shifter`, `test_handover_waits_for_the_cut_then_blends` |
| I10 — кто может запустить и кто владеет | Решение 6: модули скорости и каденса, режим привода, включённый райдер, внешний нажим отменяет, внешняя поза не отменяет, внешний тормоз замораживает удержание; тесты `test_ownership_follows_the_caller`, `test_restart_after_a_stop_and_backward_roll` |
| I11 — посев политики и темп наклона | Решение 8: вес плана вместо посева; наклон — цель программы позы с прежним темпом и пределами; рывок не тратится; тесты `test_lean_override_hook`, `test_handover_is_continuous_and_surge_is_held` |
| I14 — события плана | Решение 9: канал `rider_launch` на каждом интервале; просьба и ответ уже пишутся |
| I15, I16 | Часть 4 (поездки с порогами, включение одной задачей с утверждёнными числами) |
