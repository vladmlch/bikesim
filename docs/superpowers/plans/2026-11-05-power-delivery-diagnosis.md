# План: почему велосипед не использует всю мощность, и тянет ли нога педаль вверх (диагностика)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Цель.** Найти, где теряется мощность между желанием райдера и шатуном, и проверить правило «нога, идущая вверх, не
тянет педаль вверх». Итог — отчёт с причинами, подтверждёнными опытом. Модель в этом плане не исправляется, план
исправления пишется после отчёта.

**Устройство.** Здесь только прогоны и чтение записей. Код и профили не меняются. Варианты задаются временными
копиями профиля в `/tmp`, в каждой меняется одна строка. Записи читаются через `bike-recording sql`. Так устроена
диагностика в `docs/superpowers/audits/2026-11-05-welded-savage-chatter.md`, раздел «Как повторить».

**Инструменты.** `build/native-release/bin/bike-ride`, `build/native-release/bin/bike-recording`
(`sql`, `fields`, `export`). Python в проекте больше нет, поэтому расчёты делаются в SQL (duckdb внутри
`bike-recording`).

**Источник требований.** `docs/agents/simulation-decisions.md`:
- «Решено — не сделано», пункты 7 и 8;
- «Действует», пункты 9 и 35: решение пользователя 2026-11-05, «нужно исследование причин, а не искусственные
  ограничения».

## Общие ограничения

- **Ничего не исправлять.** Не меняй код, `examples/research/*.toml`, тесты и эталонные записи. Предел, который
  кажется виноватым, не поднимай в рабочем профиле. Его меняют только во временной копии, чтобы проверить догадку
  (`simulation-decisions.md`, пункт 34).
- **Прогоны по одному.** Параллельный запуск 2026-10-03 дал расхождение на 1e-9 м и другой исход. Не запускай
  прогоны вместе с проверками (`run_tests.sh`, `check_change.sh`).
- **Две проверки на вывод.** Каждый вывод о причине держится на двух независимых проверках
  (`docs/agents/native-engineering.md`, «Establish the contract» п. 2):
  - опыт, в котором убран предполагаемый ограничитель;
  - чтение кода, показывающее, почему он ограничивает.
  Одна только догадка выводом не считается.
- **Хаос.** Система хаотична, надёжны только большие качественные различия (аудит 2026-11-05, строки 56-59).
  Сравнивай средние за окно, а не отдельные пики.
- **Сборка.** Сейчас нет `build/native-release`, есть только `build/gui`. Пользователь 2026-11-05 сказал пока не
  собирать. Перед задачей 1 спроси разрешение на `cmake --workflow --preset release` (около 10 ГБ).
- **Бюджет.** Проверок не будет: код не меняется. Будет одна сборка и около 12 прогонов по 8–30 с модельного
  времени, всего около 15 мин.
- **Язык отчёта** — русский, без жаргона. У каждого числа единицы и источник: прогон и запрос.

## Известно до начала (2026-11-05, чтение кода; опытом не проверено)

- **Тест-потолок.** `test_strict_flat_cycle_delivers_and_stays_within_budgets`
  (`native/tests/reference/test_seated_pedaling_cycle.cpp:349-399`) задаёт желание 20/40/60 Н·м напрямую, мимо
  намерения райдера. Устоявшийся момент на шатуне получился 17,8/23,1/25,7 Н·м (`docs/TESTING.md:1513-1521`). В
  документе причина названа так: «25 Nm weld-servo hold limit».
- **Кандидаты в ограничители по пути «желание → сустав»:**
  - `weld_servo_limit_nm = 25`: `native/src/configuration/physical_config.hpp:418`, используется в
    `native/src/rider/spindle_controller.cpp:1170-1181`;
  - ограничение удержания задачей (`spindle_controller.cpp:1189-1191`);
  - кривые силы суставов (`spindle_controller.cpp:931-946`, `spindle_math.cpp:435-453`);
  - обнуление момента при |ω| ≥ 20 рад/с (`spindle_controller.cpp:970-972`);
  - бюджет мощности 250 Вт на сустав и 450 Вт всего (`spindle_controller.cpp:979`, `spindle_math.cpp:247`);
  - задержка активации 25 мс (`spindle_math.cpp:283-305`);
  - конечная проверка и ограничение мощности (`spindle_controller.cpp:1030-1034`);
  - накат выше 120 об/мин (`drivetrain/pedaling.cpp:49-52`);
  - сброс до 30 % при переключении (`writers/drivetrain.cpp:825`).
- **Желание в настоящей езде.** Когда желание задаёт намерение райдера, оно равно
  min(60, 250 Вт / max(ω, 20 об/мин)) (`native/src/rider/spindle_math.cpp:53-63`). При 90 об/мин это около
  26,5 Н·м. Поэтому в езде «26» может оказаться не потерей, а самим желанием. Это и нужно разделить.
- **Знак силы стопы.** У стопы `normal_n` — это сила на райдера вдоль оси стопы. Отрицательное значение —
  растяжение, то есть стопа тянет педаль (`native/src/writers/rider_contacts.cpp:974, 1090-1104`). Растяжение
  разрешено до 1050 Н (`examples/research/viewer_physics_welded.toml:145`).

## Файлы

- Создать: `docs/superpowers/audits/2026-11-05-power-delivery.md` — отчёт.
- Временные копии профилей и трасс: `/tmp/pd/`. Записи прогонов: `/tmp/pd/runs/`. В репозиторий они не попадают.

---

### Задача 1: Подготовка и словарь столбцов

**Файлы:**
- Создать: `/tmp/pd/` (временные копии)

- [ ] **Шаг 1: Сборка (только после разрешения пользователя)**

Выполни: `cmake --workflow --preset release`
Ожидается: появились `build/native-release/bin/bike-ride` и `bike-recording`.

- [ ] **Шаг 2: Прогон-образец, чтобы узнать имена столбцов**

```bash
mkdir -p /tmp/pd/runs
build/native-release/bin/bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml \
  --track examples/research/rough_uphill_savage.toml --headless --duration 8 --no-plots \
  --check-policy diagnostic --record --out /tmp/pd/runs/base8; echo rc=$?
R=$(ls -d /tmp/pd/runs/base8/*Z-* | head -1); echo "$R"
build/native-release/bin/bike-recording fields "$R" --pattern 'human_|motor_torque|crank|cadence|attachment|muscle|energy|gravitational|kinetic|component_work|positive_power|shortfall|budget|strength' > /tmp/pd/fields.txt
wc -l /tmp/pd/fields.txt
```
Ожидается: `rc=0` или `rc=2` (диагностический режим может завершиться с «модель недействительна»; это сведение, а
не ошибка). Файл `fields.txt` не пустой.

- [ ] **Шаг 3: Записать словарь в отчёт**

Создай `docs/superpowers/audits/2026-11-05-power-delivery.md` с разделом «Столбцы». Для каждого понятия ниже укажи
таблицу и точное имя из `fields.txt`:

| понятие | ожидаемое имя |
|---|---|
| желание момента на шатуне | `channel_control.human_torque_nm` |
| команда в политику педалирования | `channel_drive.human_command_nm` |
| момент на оси шатуна (датчик) | `channel_drive.human_sensor_nm` |
| момент через крепления педалей | `channel_rider.weld_crank_torque_nm` |
| недобор задачи шатуна | `channel_rider.rider_allocation.crank_task_shortfall_nm` |
| мощность райдера | `channel_rider.rider_positive_power_w` |
| момент мотора | `channel_drive.motor_torque_nm` |
| скорость шатуна, каденс | `channel_drive.crank_rad_s`, `cadence_rpm` |
| фаза шатуна | `channel_drive.crank_phase_rad` |
| сила стоп | `channel_rider.attachment_samples.foot_front.normal_n`, `…foot_rear.normal_n` |
| скорость велосипеда | `channel_mass.com_velocity_mps` или скорость корня по оси x |
| работа сил | `channel_energy.muscle_positive_j`, `motor_signed_j`, `component_work_j.road_rolling`, `component_work_j.aerodynamic`, `loss_j`, `channel_mass.gravitational_energy_j`, `kinetic_energy_j` |
| нарушения и пределы | `channel_rider.rider_strength_violations`, `rider_effort_budget_exceeded`, `rider_joint_power_violations`, `rider_joint_speed_violations` |

Если имя другое, запиши фактическое. Если столбца нет, напиши «нет» и чем его заменить. Уклон не записывается
(`grade_pct` есть только в предпросмотре), поэтому бери его из `grade_profile` трассы.

### Задача 2: Опыт с постоянным желанием на ровном — где теряется момент

Этот опыт повторяет тест-потолок, но через `bike-ride`.

**Файлы:**
- Создать: `/tmp/pd/flat.toml` (трасса), `/tmp/pd/const_{20,40,60}.toml` (профили)

- [ ] **Шаг 1: Ровная трасса и профили с постоянным желанием**

- Ровную трассу на 200 м сделай по образцу `cycle_env`
  (`native/tests/reference/test_seated_pedaling_cycle.cpp:183-186`). Возьми копию `examples/research/rough_uphill.toml`
  и замени профиль уклона и препятствия на ровный участок 200 м: одна точка уклона 0, без блоков препятствий.
- Профили: копии `viewer_physics_welded.toml`, в каждой `[seated_climb] enabled = false`, мотор выключен (найди ключ
  режима мотора в `[drive.assist]` или ключ `motor` в `[drive]`; в тесте мотор 0) и `[drive] human_torque_nm =
  <20|40|60>`.
- Ключ `human_torque_nm` проверь: `rg -n 'human_torque_nm' native/src/configuration/physical_config.hpp`.
- Каждая копия должна отличаться от оригинала только этими строками: `diff examples/research/viewer_physics_welded.toml
  /tmp/pd/const_60.toml`.

- [ ] **Шаг 2: Три прогона по 8 с**

```bash
for n in 20 40 60; do build/native-release/bin/bike-ride --backend native --physics-config /tmp/pd/const_$n.toml \
  --track /tmp/pd/flat.toml --headless --duration 8 --no-plots --check-policy diagnostic --record \
  --out /tmp/pd/runs/const_$n; echo "$n rc=$?"; done
```

- [ ] **Шаг 3: Средние после 2 с**

Для каждого прогона выполни запрос. Имена таблиц и столбцов возьми из словаря задачи 1:
```bash
R=$(ls -d /tmp/pd/runs/const_60/*Z-* | head -1)
build/native-release/bin/bike-recording sql "$R" --json --query "
  select avg(human_torque_nm) as wish, avg(human_command_nm) as command, avg(human_sensor_nm) as sensor,
         avg(weld_crank_torque_nm) as weld, avg(cadence_rpm) as rpm
  from channel_drive d join channel_rider r using (step) join channel_control c using (step)
  where d.time_s > 2"
```
Если ключ соединения называется не `step` (или время не `time_s`), посмотри имена через `bike-recording sql "$R"
--query "describe channel_drive"` и подставь.
Ожидается: при 60 Н·м `sensor` около 25–26 Н·м. Так воспроизводится `TESTING.md:1519`.

- [ ] **Шаг 4: Какой ограничитель срабатывает**

Для прогона на 60 Н·м посчитай долю интервалов после 2 с, в которых:
- недобор задачи шатуна > 1 Н·м;
- сработало ограничение силы суставов, бюджет мощности или скорость сустава (столбцы нарушений и
  `rider_effort_budget_exceeded`);
- каденс выше 120 об/мин (накат).
Запиши таблицу «ограничитель → доля времени» в отчёт.

### Задача 3: Убрать по одному ограничителю (опыты)

**Файлы:**
- Создать: `/tmp/pd/const_60_<вариант>.toml`

- [ ] **Шаг 1: Варианты, в каждом одна изменённая строка относительно `const_60.toml`**

| вариант | изменение | проверяемая догадка |
|---|---|---|
| `servo` | `weld_servo_limit_nm = 60` (ключ найди: `rg -n 'weld_servo_limit' native/src/configuration`) | потолок задаёт удержание сварки 25 Н·м |
| `power` | `active_positive_power_limit_w = 900` и предел на сустав 500 Вт (ключ: `rg -n 'joint_power_limit' native/src/configuration`) | потолок задаёт бюджет мощности |
| `act` | `activation_tau_s = 0.0` | потолок задаёт задержка активации |
| `ripple` | `pedal_torque_ripple = 0.0` | потеря в мёртвых точках |

Если ключа нет в TOML-схеме (загрузчик откажет: «unknown key»), этот вариант не делай. Запиши в отчёт, что опыт без
правки кода невозможен, и отметь это как вопрос пользователю.

- [ ] **Шаг 2: Прогоны по одному, по 8 с, на `/tmp/pd/flat.toml`**

Команда та же, что в задаче 2, шаг 2.

- [ ] **Шаг 3: Таблица результатов**

Таблица: вариант → средний момент на шатуне после 2 с → изменение относительно `const_60`. Догадка подтверждена,
если момент заметно вырос (больше чем на 5 Н·м) и чтение кода объясняет почему: укажи строку. Если ни один вариант не
поднимает момент, запиши это. Тогда следующий шаг — ограничение в самой задаче (`spindle_controller.cpp:1189-1191`)
или кривые силы. Это вопрос к пользователю.

### Задача 4: Баланс мощности в настоящей езде

**Файлы:**
- Изменить: `docs/superpowers/audits/2026-11-05-power-delivery.md`

- [ ] **Шаг 1: Прогоны с рабочим профилем**

Сделай два прогона:
- `viewer_physics_welded.toml` на `rough_uphill_savage.toml`, 30 с;
- тот же профиль на `/tmp/pd/flat.toml`, 30 с.
Флаги те же, что в задаче 1, шаг 2.

- [ ] **Шаг 2: Разделить «желание мало» и «доставка мала»**

По окнам в 1 с запиши:
- желание (`human_torque_nm`), его предел по мощности 250 Вт / ω;
- момент на шатуне;
- момент мотора;
- каденс, скорость.
Вывод по каждому окну: если желание примерно равно 250 Вт / ω, а момент на шатуне примерно равен желанию, потерь
доставки нет и «26 Н·м» — это само желание. Если момент на шатуне заметно меньше желания, это потеря доставки.
Тогда ссылайся на задачу 3.

- [ ] **Шаг 3: Баланс мощности на ровном подъёме**

Для участков с постоянным уклоном (уклоны из `grade_profile` трассы) посчитай за окно:
- **вход:** приращение `muscle_positive_j` и `motor_signed_j` за окно, делённое на длительность окна;
- **выход:**
  - m·g·v·sin(θ), где m — полная масса из записи или из профиля: райдер 80 кг плюс велосипед;
  - качение — приращение `component_work_j.road_rolling`;
  - воздух — приращение `component_work_j.aerodynamic`;
  - потери — приращение `loss_j`;
  - изменение кинетической энергии.
Запиши невязку «вход − выход». Затем посчитай, какая скорость получилась бы, если бы весь вход уходил на подъём,
качение и воздух. Сравни её с фактической. Это проверка из `simulation-decisions.md`, пункт 35.

### Задача 5: Тянет ли нога педаль вверх

**Файлы:**
- Изменить: `docs/superpowers/audits/2026-11-05-power-delivery.md`

- [ ] **Шаг 1: Сила стопы по фазе шатуна**

Используй записи задач 2 (`const_60`) и 4 (savage, ровная). Для каждой стопы раздели интервалы на «нога идёт вверх»
и «нога идёт вниз» по фазе шатуна. Ноль фазы соответствует горизонтальному шатуну (`c129db9a`), передняя педаль
впереди. Для передней педали подъём — фаза в (π, 2π) при вращении вперёд, для задней — сдвиг на π. Проверь это по
положению педалей: если в записи есть координаты стоп, бери их.

Посчитай для ноги, идущей вверх:
- долю интервалов с `normal_n < 0` (растяжение, нога тянет педаль);
- наибольшее растяжение;
- среднее давление.

- [ ] **Шаг 2: Сравнить с правилом**

Правило (`simulation-decisions.md`, пункт 9): давление ноги, идущей вверх, от 0 до веса ноги. Вес ноги посчитай по
массам тел бедра и голени в модели (`bike-recording sql "$R" --query` по таблице масс, если есть; иначе по
`native/src/rider/` и MJCF, `rg -n 'thigh|shank' native/src/model/*.cpp`). Запиши, сколько времени и с какой силой
правило нарушается.

- [ ] **Шаг 3: Откуда растяжение**

Сопоставь моменты растяжения с тремя событиями:
- мёртвыми точками;
- резким торможением шатуна (накат, переключение);
- ударами о препятствия.
Проверь, держит ли цель `return_foot_preload_n = 40` ногу в сжатии и где перестаёт. Опыт: копия профиля с
`return_foot_preload_n = 0`, savage, 30 с — меняется ли доля растяжения.

### Задача 6: Отчёт и вопрос пользователю

**Файлы:**
- Изменить: `docs/superpowers/audits/2026-11-05-power-delivery.md`
- Изменить: `docs/agents/simulation-decisions.md` (после решения пользователя)

- [ ] **Шаг 1: Свести выводы**

Начало отчёта:
- три-пять предложений: где теряется мощность, подтверждено ли это опытом и кодом, нарушается ли правило ноги;
- затем таблицы задач 2–5.
Каждый вывод помечен одной из трёх меток: «подтверждено опытом и кодом», «только опыт» или «только догадка».

- [ ] **Шаг 2: Показать пользователю**

Дай путь к отчёту, вывод в трёх строках и варианты следующего шага. Например:
- «убрать удержание 25 Н·м из пути педалирования»;
- «уточнить кривые силы»;
- «запретить растяжение ноги, идущей вверх».
Решает пользователь. План исправления пишется после его решения.

- [ ] **Шаг 3: Обновить файл решений**

В `docs/agents/simulation-decisions.md`, «Решено — не сделано», пункты 7 и 8, допиши: «Диагностика YYYY-MM-DD,
[отчёт](../superpowers/audits/2026-11-05-power-delivery.md): <вывод>. Решение пользователя: <...>.»

- [ ] **Шаг 4: Коммит отчёта — только по просьбе пользователя**

```bash
git add docs/superpowers/audits/2026-11-05-power-delivery.md docs/agents/simulation-decisions.md
git commit -m "docs(audit): trace the crank delivery gap and the rising-leg pull

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
