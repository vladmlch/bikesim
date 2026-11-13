# План старта, часть 4б: шатун на угле таблицы, гашение ноги, жёсткость позы в профиле — план реализации

> **Для исполнителя:** задачи выполняются по одной, в порядке номеров (`superpowers:subagent-driven-development`).
> Шаги помечены `- [ ]`. Финальное ревью ветки делает оркестратор.

Редакция 4 (2026-11-13). Учтены ревью qoder Ultimate `plan-review-2.md` (C1, I1–I6, мелкие 1–9),
`plan-review-3.md` (C1, I1–I3, мелкие 1–7) и `plan-review-4.md` («готов с правками»: I1–I2, мелкие 1–5) в `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/`. По разбору
pneumatic гашение удержания 200 → 88. Решения пользователя к ревью 3: удержание только при полном райдере
(решение 2), провал каденса 3,30 об/мин принят (решение 6). Часть 4 (`docs/superpowers/plans/2026-11-12-launch-4-activation.md`) остановлена после
задачи 4 (`1b75ae48`). Эта часть вставляется перед её задачей 5. После неё часть 4 продолжается с задачи 5.

Основания:
- замеры на копиях кода — `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-rising-leg-qoder/report.md`, разделы
  Follow-up 5 (гашение), 8–9 (удержание шатуна и угол), 10 (таблицы под новый угол);
- факты кода — `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/facts.md` (снят на `1b75ae48`, разделы A–H).

**Цель.** Поза покоя считается для угла педалей из таблицы старта, как требует спецификация. Колебание ноги
18–24 Гц гасит активный момент контроллера. Жёсткость и гашение позы ног задаются профилем. Старт идёт с угла
−0,8 рад и с таблицами, подобранными под этот угол.

**Устройство.**
- Удержание: при поиске равновесия на шатун действует упругий момент к углу таблицы. Он идёт через канал
  внешней обобщённой силы шага, только на этапе подготовки. В поездке его нет.
- Гашение: к запросу момента каждого сустава ноги добавляется `−c·(q̇ − q̇_kin)`. Здесь `q̇_kin` — скорость
  сустава, которую задают шатун и таз через сварную цепь, тем же разбором, что и прямая подача. Момент проходит
  все пределы силы и мощности, как любой мышечный момент.
- Главная проверка: поездки ветки побайтно равны поездкам копий qoder с той же арифметикой (решение 7).

**Стек.** C++23, CMake/CTest, `bike_test::run`, кодеки `bike::io::Value`.

**Спецификация.** `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md:38` — «поза равновесия считается
для этого угла».

## Решения пользователя (2026-11-13)

Журнал: `/Volumes/SSD_1TB_M2/work/mujoco_clean-scratch/devin/2026-11-05-physics-queue/ledger.md`.

- У1. `joint_kd_nms_rad` остаётся 15, `joint_kp_nm_rad` — 600. Перебор kd только переносил провалы между
  стартами (Follow-up 3).
- У2. Колебание ноги гасит **активный** момент в контуре контроллера, c = 35 Н·м·с/рад (вариант A, Follow-up 5).
  Это не демпфирование шарнира: решение 26 и «Отвергнуто» 1–2 документа решений не меняются.
- У3. Поза равновесия считается для угла таблицы: при поиске шатун удерживается на угле, до поездки удержание
  снимается (Follow-up 8).
- У4. Угол старта −0,8 рад у всех трёх типов райдера. Более крутые углы — в очередь. Гипотеза, не проверенная
  при гашении 88: «стена» за −0,9 (Follow-up 9) была раскачкой удержания, а не досягаемостью ноги. Запас ноги до
  педали на −0,8 — 117 мм (разбор pneumatic, решение 1).
- У5. Таблицы старта подбираются под −0,8 рад с удержанием и гашением; числа утверждает пользователь
  (Follow-up 10, решение 6 ниже).
- У6. Жёсткость и гашение позы ног переходят из кода в профили, числа не меняются.
- У7. Остановка `rough_uphill_extreme` на 13,1 с — в очереди (решение от 2026-11-13).

## Решения плана

1. **Поля удержания.** В корне физики, после `equilibrium_refine_period_s`:
   - `equilibrium_crank_hold_kp_nm_rad = 20000.0`;
   - `equilibrium_crank_hold_kd_nms_rad = 88.0`.

   Оба обязательны и больше нуля. Жёсткость — из замера (Follow-up 8): шатун отходит от угла на 0,05–0,09° под
   нагрузкой 14–30 Н·м. Гашение 88 (решение пользователя 2026-11-13) — критическое для пружины 20000 при
   инерции разомкнутого шатуна 0,096 кг·м²: 2·√(20000·0,096) = 87,6.

   Почему не прежние 200. Момент удержания идёт через внешнюю силу шага, то есть явно, мимо `implicitfast`. Шаг
   1,25 мс с таким моментом устойчив только при `kd·h/I < 2 − kp·h²/(2I)`, то есть при kd < 141. При 200 и
   разомкнутом шатуне отклонение каждый шаг умножается на −1,81 (замер −1,80…−1,815). Отсюда расходимость
   pneumatic на ровном и уход шатуна до −1,08 рад на 5 %. Разбор:
   `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-pneumatic-reach/report.md`. Запас ноги до педали на −0,8 — 117 мм у
   всех трёх профилей. Входят в ключ кэша равновесия и в контракт начального состояния:
   их нет в списках стирания `equilibrium_cache.cpp:170` и `initial_state.cpp:54`, туда их не добавлять.
2. **Где действует удержание.** Только когда ноги приварены к педалям (решение пользователя 2026-11-13):
   `context.physical.rider_present() && context.physics.articulated.pedal_attachment == "spindle"`. Так устроены все
   исследовательские профили.
   - Без приваренных ног на шатуне один его узел с педалями, около 0,013 кг·м². Это сосредоточенный райдер, нет
     райдера, сочленённый райдер на плоских педалях (`flat` — значение по умолчанию, `physical_config.hpp:446`,
     стопы давят силами накладок, связи нет). Явное удержание 20000 / 88 такой шатун раскачивает: устойчиво только
     при инерции больше ~0,063 кг·м² (ревью 3 C1, ревью 4 I1).
   - `weld` не замерен, поэтому тоже без удержания.
   - Там, где удержания нет, шатун ведёт себя как сегодня. Для этих настроек требование спецификации `:38` не
     выполняется; это записывается в документ решений как ограничение.
   - Сосредоточенный райдер шатун не нагружает. Шатун уравновешен, а свободный шарнир держит угол шатуна в мире,
     пока рама наклоняется. Поэтому тест задачи 3 проверяет `|уход| ≤ |наклон рамы| + 1e-3` рад (ревью 4 I2).
   - Почему нет проверки `a < 2 − b/2` по `M[crank,crank]`: диагональ матрицы масс не содержит ног, связанных со
     стопами приваркой, и у spindle тоже даёт ~0,013. Устойчивость при приваренных ногах подтверждают замер
     (собственное число −1,80 при 200 совпало с расчётом при 0,096) и 96 поездок Follow-up 1 при 88.
   Внутри подготовки удержание действует в `apply_initial_forces` (`settle_contacts.cpp`) — значит, во всех вызовах:
   - расслабление;
   - уточнение;
   - проверка файла состояния;
   - восстановление переноса;
   - `finalize`;
   - прямые вызовы в `test_sag.cpp:102, :194` (spindle, удержание действует; тест `initialization_sag`).

   Это так же, как держится наклон корпуса. Шаги поездки внешней силы не передают: накопитель сил очищается на
   каждом вызове (`step.cpp:1376`), поэтому в поездке шатун свободен. Угол — `drive.crank_phase_rad`, который
   подготовка берёт из `rider_launch.start_pedal_angle_rad`.
3. **Поле гашения** — `[articulated].leg_active_damping_nms_rad`, обязательное, не меньше нуля. Имя
   `joint_passive_damping_nms_rad` занято снятым ключом, который обязан отвергаться
   (`test_native_spindle_controller.cpp:1180-1188`). Ключ входит в провод конфигурации контроллера:
   `kConfigRequiredKeys` 67 → 68.
4. **Жёсткость позы.** `joint_kp_nm_rad`, `joint_kd_nms_rad` — обязательные (`REQ_REAL`). Значение в структуре — `-1.`, как у
   `balance_dwell_s`. Проверка: больше нуля. Во всех документах 600 и 15.
5. **Документы.** Входные документы переводятся везде, красные тесты включительно. Списки файлов — в
   `launch4b/inventory/` (снято на `1b75ae48`, см. «Опись»). Записанные выходы (`tests/fixtures/golden/**` кроме
   восьми входных `physics.toml`, `tests/fixtures/ft2/**`, бинарные `.bin`) не трогаются: `launch4b/inventory/excluded-recorded.txt`.
   Исторические записи `tests/fixtures/replay/*/summary.json` переводятся, как и раньше при смене полей. Пример —
   `29dd2d1c`: из `historical_v2/summary.json` убраны `coasting_brake_*`. Читателя старых форм нет. Строки `facts.md` C.3/G.2
   («входы должны читаться без новых полей») этим решением отменены. Тест
   `reference.replay_recording_v3` красный с `1b75ae48`, его записи перезаписывает часть 4, задача 7. Значения:
   - удержание 20000 / 88;
   - гашение 35;
   - жёсткость и гашение позы 600 / 15.

   Записанные выходы красных тестов не перезаписываются: это делает часть 4, задача 7, после «да» пользователя.
6. **Таблицы старта** — набор `c3` Follow-up 10 (`.../cal/candidates/c3/`). У всех трёх типов
   `start_pedal_angle_rad = -0.8`. Остальные числа таблиц части 4 (решение 5) не меняются, кроме пяти клеток:

   | профиль | поле | было | стало | поездка-причина |
   |---|---|---|---|---|
   | обычный (`viewer_physics_{welded,distributed,pneumatic}.toml`) | `effort_cap_wet_nm` | 45.0 | 13.0 | `rv25_n_wet10` (скольжение) |
   | обычный | `effort_cap_grade_nm`, узел 0,05 (2-й) | 18.0 | 15.0 | `rv25_n_g05` (подъём колеса) |
   | осторожный (`rider_type_careful.toml`) | `effort_cap_wet_nm` | 35.0 | 11.0 | `fck_c_wet11/14/17` (скольжение) |
   | осторожный | `effort_cap_grade_nm`, узел 0,15 (6-й) | 15.0 | 13.0 | `fck_c_g17`, `fck_c_loose17` (подъём) |
   | осторожный | `effort_cap_grade_nm`, узел 0,2 (7-й) | 18.0 | 14.0 | то же |

   Итог: `effort_cap_grade_nm` обычного `[38.5, 15.0, 14.5, 13.2, 14.0, 15.5, 19.5, 18.5]`, осторожного
   `[26.0, 14.0, 11.5, 13.0, 14.0, 13.0, 14.0]`. Агрессивный — только угол.

   Замер набора `c3`:
   - 27/27 поездок без остановки и 45/45 промежуточных уклонов П13 проходят; 6 поездок с остановкой — нет (П9);
   - все 78 встают в позу; при удержании 20000 / 88 момент не больше 21,7 Н·м, уход не больше 1,1e-3 рад;
   - наименьшие силы: опора седла 19,4 Н, нажим стопы 12,3 Н;
   - худший каденс в окне мёртвой точки 3,30 об/мин (`rv25_a_g20`) против 3,96 при гашении 200. Это провал
     колебания каденса после отпускания тормоза, а не застревание в мёртвой точке. Условие «не хуже 3,96» было в
     задании на подбор, в решении 10 части 4 его нет. Пользователь принял 3,30 (2026-11-13), таблицы не меняются.

   Окно мокрого предела обычного узкое: 12 < c < 14 Н·м (Follow-up 10, оговорка 1).
7. **Побайтная сверка с копиями qoder.** Копии собраны на `1b75ae48` с той же арифметикой: постоянные вместо
   полей, те же выражения. Код задач 2–4 повторяет выражения копий дословно, иначе сверка не сойдётся. Опорные
   поездки (не менять, только читать):
   - задача 2 (гашение, без удержания, углы профилей) — `launch4b/parity-ref/v2/runs/`;
   - задача 3 — побайтной сверки нет: опорные поездки при угле 1,12 сняты с прежним гашением удержания 200. Её
     заменяет сверка задачи 4, где код всех трёх задач работает вместе;
   - задача 4 (новые таблицы, −0,8, удержание 20000 / 88) — поездки Follow-up 1 разбора pneumatic в
     `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-pneumatic-reach/f1/runs/kd88/m/sweep/`:
     - `c3/runs/` — 78 поездок набора `c3`;
     - `c3wet/runs/` — мокрые 0 и 5 % у трёх типов;
     - `distributed/runs/`, `pneumatic/runs/` — по шесть поездок.

     Итоги и копии профилей — `.../pneumatic-reach/f1/f1_table_kd88_{c3,extra}.csv` и
     `.../cal/extra/candidates/`. Главные файлы поездок (`<имя>.csv`) там сжаты gzip. Их md5 до сжатия —
     `.../f1/c3_gz_record.tsv` и `.../f1/md5_kd88_extra_runs.txt`. В `c3/runs/` сжат только `<имя>.csv`, в
     `c3wet/`, `distributed/`, `pneumatic/` сжаты все три файла. Правило сверки: файл-образец с `.gz` —
     `gzip -dc <образец>.gz | cmp - <новый>`, без `.gz` — `cmp <образец> <новый>`.
     - Восемь поездок `c3` писались с паузами по нехватке места: `rv25_n_g09`, `rv25_n_loose09`, `rv25_n_wet09`,
       `rv25_n_g14`, `rv25_n_loose11`, `rv25_n_g11`, `rv25_n_wet11`, `rv25_n_loose07`. Если не совпали только они —
       переснять их образцы бинарником `.../pneumatic-reach/f1/kd88/bin/kdprobe` без пауз и сравнить снова.
     - Поездки и треки задачи 4: 78 — `build_rides("full")` из `.../cal/run_cal.py` (треки списка — из
       `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/gradecap/tracks/`, промежуточные — `.../cal/tracks/`);
       18 — `.../cal/extra/rides.json`, треки `.../cal/extra/tracks/`. Запуск — копия
       `.../pneumatic-reach/f1/run_rides.py` и `.../cal/run_cal.py` в `launch4b/parity/` с бинарником ветки и
       профилями репозитория вместо копий: `c3`, `c3wet` — `welded` → `viewer_physics_welded.toml`, `careful` →
       `rider_type_careful.toml`, `aggressive` → `rider_type_aggressive.toml`; `distributed` →
       `viewer_physics_distributed.toml`; `pneumatic` → `viewer_physics_pneumatic.toml`.

   Драйвер ветки тоже запускается с отдельным пустым каталогом кэша на поездку, поэтому `equilibrium_cache_enabled`
   на поездку не влияет.

   Каждый опорный набор снят с отдельным пустым кэшем равновесия на каждую поездку. Исходные поездки V2
   (`.../damp/m/v2/runs/`) брали позу из общего кэша, где лежали позы без гашения. Поэтому опорный набор задачи 2
   переснят с тем же бинарником V2 и пустым кэшем на поездку: `launch4b/parity-ref/v2/runs/`, скрипт
   `launch4b/parity-ref/run_v2_fresh.sh`. Профили-копии этого набора — `.../kd/profiles/{welded,careful,aggressive}_kd15.toml`,
   у каждого типа свой угол (1,12 / 0,9 / 1,24).

   Допустимые отличия профиля-копии от профиля репозитория (`diff` до сверки, иначе стоп):
   - все задачи: ключи задач 1–N (`joint_kp_nm_rad`, `joint_kd_nms_rad`, `leg_active_damping_nms_rad`,
     `equilibrium_crank_hold_*`) — их нет в копиях, или они записаны иначе (`joint_kd_nms_rad = 15`);
   - `equilibrium_cache_enabled` (`false` в копиях `hold/` и `cal/`);
   - задача 3: `start_pedal_angle_rad` у осторожного и агрессивного (1,12 в копиях, 0,9 / 1,24 в репозитории) — эти
     поездки не сравниваются.

   Здесь `...` = `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-rising-leg-qoder`. Драйвер — `.../kd/kdprobe.cpp`, сборка
   по `.../kd/build_kdprobe.sh` (копия в `launch4b/parity/`), список поездок —
   `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/parity/manifest.txt`, запуск — как в `.../hold/run_sweep.sh`
   (кэш равновесия в отдельном каталоге на поездку). Сравниваются все три файла поездки
   (`.csv`, `_meta.csv`, `_excess.csv`) через `cmp`.
8. **Покрасневшие тесты.** Задачи 2–4 меняют позу покоя и моменты ног всех сочленённых поездок, поэтому зелёные
   сегодня тесты-читатели эталонов могут покраснеть. Каждый новый красный тест — строка в
   `launch4b/golden-list-task<N>.md`: тест, файл, первая разная строка, причина («удержание», «гашение»,
   «таблицы»). Ничего не перезаписывать. Провал проверки поведения (не сравнения с эталоном) — стоп с числами.

## Общие ограничения

- **Пороги, пределы, допуски тестов, фильтры, задержки, силы, `strength`, `pedal_torque_ripple`, запас 25 Н**
  не меняются. Числа профилей — только из решений 1, 4, 5, 6.
- Значений по умолчанию в коде для полей профиля нет. Читателей старых форм нет. Профиль без поля не загружается и
  называет поле.
- Пассивного демпфирования шарниров нет ни в каком виде (AGENTS.md «Демпфирование»). Гашение У2 — момент
  контроллера в `requested`, проходит `limit_torques`.
- Стопа только давит на педаль, седло только подпирает (AGENTS.md «Райдер»). Гашение не должно заставить стопу
  тянуть педаль или седло тянуть райдера: в задачах 2 и 4 наименьшие нажим стопы и опора седла по каждой поездке —
  в отчёт, отрицательное значение вне порога решения 10 части 4 — стоп.
- Мотор не меняется. КПД 100 %.
- Красные тесты на `1b75ae48`: 23 обычных и 11 медленных (`facts.md` G.1). В красном тесте нельзя переставлять
  или ослаблять проверки.
- Без отладчиков, с ограничением времени (`perl -e 'alarm 2400; exec @ARGV' --`), без `git stash`, без push.
  `.claude/` и `.agents/` не трогать, чистого `git status` не добиваться.
- Последняя строка коммита ровно `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`, без строк про Devin и
  «Generated with».
- Временные файлы и прогоны — в `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/` (дальше `launch4b/`). Каталоги
  `.../2026-11-13-rising-leg-qoder/**`, `.../2026-11-12-launch3b/**`, `.../2026-11-12-launch4/**` только читаются.
- Сборка драйвера собирает список объектов через `find`, не `ninja -t`.
- Параметры Bosch в интернете не искать.

## Проверка задачи

После коммита задачи:

```bash
cd /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05
BASELOG=/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/baseline-before-task<N>.log \
  EXPECT=<список задачи или пусто> \
  bash /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch2/check_task.sh <база задачи> \
  /Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/verify-task<N> [ab]
```

- `ab` — только в задаче 1: числа не меняются, поза покоя та же.
- Оркестратор кладёт `baseline-before-task<N>.log`; для задачи 1 — итог проверки задачи 4 части 4.
- Конец вывода — `check_task exit 0`. В задачах 2–4 допустим стоп «new red tests» или «causes differ» только по
  тестам из `golden-list-task<N>.md` (решение 8). Исполнитель тогда останавливается с этим списком, ничего не
  перезаписывая.

## Бюджет тестов

| Что | Время |
|---|---|
| целевые тесты задачи | 3–8 мин |
| `check_task.sh` (инкрементальная сборка) | 10–30 мин на задачу, 4 раза |
| сборка драйвера и 33 поездки по 20 с (задачи 2, 3) | около 10 мин, 2 раза |
| задача 4: 78 поездок по 20 с и 18 поездок решения 6 (8 потоков) | около 20 мин |
| задача 4: семь опорных тестов, `rough_uphill_extreme`, `savage` | около 15 мин |
| задача 5: список эталонов (`ctest -R` по читателям) | около 15 мин |

Итого около 4 ч машинного времени без повторов. Полный прогон (`run_tests.sh full`) — в части 4, задача 9.

## Опись

Списки сняты на `1b75ae48` и лежат в `/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-launch4b/inventory/`. В начале задачи
исполнитель повторяет каждую команду и сверяет вывод со списком. Расхождение — стоп.

- **Программные конфигурации.**
  - С `[articulated]`: `git grep -n 'articulated\.balance_dwell_s *=\|config\.balance_dwell_s *=' -- native`
    (13 мест: `validate.cpp` `default_physics`, `cli/configuration.cpp`, `validation/{rider,drivetrain,scenarios}.cpp`,
    опорные помощники тестов, `test_schema.cpp:246`, `test_articulated_config.cpp`).
  - С корнем физики: `git grep -n 'rider_launch *= *{\|rider_launch={' -- native` (семь мест из `facts.md` A.6
    плюс помощники тестов).
- **TOML-файлы** — `toml-root.txt` и `toml-articulated.txt`, один и тот же набор из 18 файлов:
  `git grep -l 'balance_dwell_s *=' -- '*.toml'`. Восемь из них — `tests/fixtures/golden/**/physics.toml`. Это
  запросы, которые тесты загружают, поэтому они переводятся как входы.
- **Встроенные TOML-строки тестов** — `test-strings-root.txt` (`git grep -ln 'grounded_hold_s' -- native/tests`, 11
  файлов) и `test-strings-articulated.txt` (`git grep -ln 'balance_dwell_s' -- native/tests`, 13 файлов). Ключ
  добавляется в каждую строку, которую тест загружает как профиль.
- **JSON-входы.**
  - Провод контроллера — `json-wire.txt`, 28 файлов: `git grep -l '"joint_kd_nms_rad"' -- tests tools`, кроме
    `tests/fixtures/golden/**`, `tests/fixtures/ft2/**` и `.bin`.
  - Корень физики — `json-root.txt`, 9 файлов: `git grep -l '"equilibrium_refine_period_s"' -- tests tools` с теми же
    исключениями.
  - `tests/fixtures/config/schema.json` есть в обоих списках — записи полей по образцу `balance_dwell_s`
    (обязательное: `"required": true`, `"default": null`).
- **Не трогать** — `excluded-recorded.txt`, 22 записанных выхода (`golden/**`, `ft2/**`, `.bin`). Их тесты красные
  или краснеют (решение 8), перезапись — в части 4, задача 7.

- **Инструмент перевода.** Скопировать `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/tools/` в
  `launch4b/tools/` и править копию. JSON читается с порядком ключей; новый ключ ставится в упорядоченное место,
  если ключи объекта упорядочены, иначе — сразу после соседнего ключа из шага задачи. Отступы и запись чисел
  остаются как в файле (`%.1f` для целых значений: `35.0`, `600.0`). `verify_migration.py` (копия) проверяет, что
  старые значения не изменились.

---

### Задача 1. Жёсткость и гашение позы ног — обязательные поля профиля

**Файлы:**
- Изменить: `native/src/configuration/physical_config.hpp:399-400, :868-869`, `native/src/configuration/validate.cpp`
  (`:745-761` список положительных, `default_physics`), программные конфигурации описи, TOML-тексты описи,
  `tests/fixtures/config/schema.json`
- Тест: `native/tests/reference/test_articulated_config.cpp`, `native/tests/config/test_resolution.cpp:1287-1292`

**Interfaces:**
- Produces: `ArticulatedConfig::joint_kp_nm_rad`, `joint_kd_nms_rad` — обязательные, `-1.` в структуре.

Поведение поездок не меняется: значения во всех документах те же 600 и 15.

- [ ] **Шаг 1. Тест (падает).** В `test_articulated_config.cpp`:
  - во всех местах `config.balance_dwell_s = .5; config.balance_grace_s = 3.;` дописать
    `config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.;`;
  - добавить тест и вызвать его в `main` последним:

```cpp
// Launch part 4b: the posture PD gains come from the profile — an unset
// gain is not a silent 600 / 15, it fails validation and names the field.
void test_posture_gains_are_required() {
    for (const auto field : {&ArticulatedConfig::joint_kp_nm_rad, &ArticulatedConfig::joint_kd_nms_rad}) {
        ArticulatedConfig config;
        config.balance_dwell_s = .5; config.balance_grace_s = 3.;
        config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.;
        config.*field = -1.;
        bool rejected = false;
        try {
            post_init(config);
        } catch (const std::invalid_argument &error) {
            const std::string what = error.what();
            rejected = what.find(field == &ArticulatedConfig::joint_kp_nm_rad ? "joint_kp_nm_rad"
                                                                               : "joint_kd_nms_rad") !=
                       std::string::npos;
        }
        bt::require(rejected, "an unset posture gain fails validation and names the field");
    }
}
```

  - в `test_resolution.cpp:1287-1292` в список пропущенных ключей добавить
    `{"ArticulatedConfig", "joint_kp_nm_rad"}, {"ArticulatedConfig", "joint_kd_nms_rad"}`, а в строку `full` этого
    теста после `balance_grace_s = 3.0\n` — `joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\n`.

  Запустить `mise run test 'articulated_config'`. Ожидание: FAIL — сегодня у полей есть значения 600 и 15, и
  `post_init` пропускает `-1.`.

- [ ] **Шаг 2. Код.**
  - `physical_config.hpp:399-400`:

```cpp
    // Posture PD gains of the rider controller (spindle_controller.cpp
    // :1405-1420). Required from the profile, no in-code value (launch part
    // 4b, user ruling 2026-11-13); the profiles keep 600 / 15.
    double joint_kp_nm_rad = -1.;
    double joint_kd_nms_rad = -1.;
```

  - `:868-869`: `REAL` → `REQ_REAL` для обоих полей.
  - `validate.cpp`, список положительных `:745-761`: добавить
    `{"joint_kp_nm_rad", &ArticulatedConfig::joint_kp_nm_rad}, {"joint_kd_nms_rad", &ArticulatedConfig::joint_kd_nms_rad},`.
  - `default_physics` после `config.articulated.balance_grace_s = 3.;`:
    `config.articulated.joint_kp_nm_rad = 600.; config.articulated.joint_kd_nms_rad = 15.;`.
  - Остальные программные конфигурации описи — те же две строки рядом с `balance_grace_s`.

- [ ] **Шаг 3. Документы.**
  - TOML-тексты описи с `balance_dwell_s =`: после строки `balance_grace_s = ...` вставить
    `joint_kp_nm_rad = 600.0` и `joint_kd_nms_rad = 15.0`.
  - В JSON-документах ключи уже есть (600 и 15), они не меняются.
  - В `schema.json` у двух записей: `"required": true`, `"default": null`.

- [ ] **Шаг 4. Целевые тесты.**
  `mise run test 'articulated_config|config_schema|config_resolution|welded_profile_config|state_codecs|launch_preparation'`.
  - `articulated_config`, `welded_profile_config`, `state_codecs`, `launch_preparation` — PASS.
  - `config_schema` и `config_resolution` красные с `1b75ae48`: причина в выводе та же, что в базовом журнале,
    новых несовпадений нет.

- [ ] **Шаг 5. Коммит.** `refactor(config): the posture PD gains come from the profile`.

- [ ] **Шаг 6. Проверка задачи** с `ab`.

---

### Задача 2. Активное гашение ноги

**Файлы:**
- Изменить: `native/src/configuration/physical_config.hpp` (поле после `joint_kd_nms_rad`, запись схемы),
  `native/src/configuration/validate.cpp`, `native/src/rider/spindle_controller.hpp:70-71`,
  `native/src/rider/state_codec.cpp:728-759, :816-820`, `native/src/rider/spindle_controller.cpp:1598, :1716-1719`,
  программные конфигурации, TOML-тексты и JSON-документы описи
- Тест: `native/tests/io/test_state_codecs.cpp:314-322`, `native/tests/reference/test_articulated_config.cpp`
- Создать (вне репозитория): `launch4b/parity/`

**Interfaces:**
- Consumes: задача 1.
- Produces: `ArticulatedConfig::leg_active_damping_nms_rad`, `rider::SpindleConfig::leg_active_damping_nms_rad`,
  ключ провода `leg_active_damping_nms_rad`.

- [ ] **Шаг 1. Тесты (падают).**
  - `test_articulated_config.cpp`: везде, где задача 1 дописала `joint_kd_nms_rad = 15.;`, дописать
    `config.leg_active_damping_nms_rad = 35.;`. Добавить тест и вызвать его в `main`:

```cpp
// Launch part 4b: the active leg damping comes from the profile and is
// never negative — a negative gain would pump the 18-24 Hz leg mode.
void test_leg_damping_is_required_and_nonnegative() {
    ArticulatedConfig config;
    config.balance_dwell_s = .5; config.balance_grace_s = 3.;
    config.joint_kp_nm_rad = 600.; config.joint_kd_nms_rad = 15.;
    config.leg_active_damping_nms_rad = 0.;
    post_init(config);   // zero is a valid gain
    config.leg_active_damping_nms_rad = -1.;
    bool rejected = false;
    try {
        post_init(config);
    } catch (const std::invalid_argument &error) {
        rejected = std::string(error.what()).find("leg_active_damping_nms_rad") != std::string::npos;
    }
    bt::require(rejected, "a negative leg damping fails validation and names the field");
}
```

  - `test_state_codecs.cpp`, в `checks_spindle()` после первой проверки:

```cpp
    // Launch part 4b: the active leg damping is a required wire key and
    // never negative.
    bt::require(config.leg_active_damping_nms_rad == 35., "the active leg damping decodes");
    for (const bool drop : {true, false}) {
        bt::invalid([drop] {
            bio::Value doc = parse_fixture("config/rider_controller.json");
            auto &members = std::get<bio::Value::Object>(doc.data);
            const auto it = std::ranges::find_if(
                members, [](const auto &member) { return member.first == "leg_active_damping_nms_rad"; });
            bt::require(it != members.end(), "the fixture carries the leg damping");
            if (drop)
                members.erase(it);
            else
                it->second = bio::Value{bio::Number{"-1.0"}};
            (void)rider::decode_spindle_config(doc);
        });
    }
```

  Комментарий «The 70-key controller config» в `:315` исправить на «The 68-key controller config».

- [ ] **Шаг 2. Конфигурация и провод.**
  - `physical_config.hpp`, после `joint_kd_nms_rad`:

```cpp
    // Active damping of the 18-24 Hz leg-chain mode (launch part 4b, user
    // ruling 2026-11-13): a controller torque -c*(qdot - qdot_kinematic) on
    // every leg joint, added to the requested torque and limited like every
    // muscle torque — not joint damping. Required from the profile.
    double leg_active_damping_nms_rad = -1.;
```

  - Запись схемы `REQ_REAL(ArticulatedConfig, leg_active_damping_nms_rad),` после записи `joint_kd_nms_rad`.
  - `validate.cpp`, после цикла `:745-761`:
    `scalar(config.leg_active_damping_nms_rad, "leg_active_damping_nms_rad", 0.);`.
  - `default_physics` и программные конфигурации описи: `...leg_active_damping_nms_rad = 35.;` рядом с
    `joint_kd_nms_rad`.
  - `spindle_controller.hpp`, `SpindleConfig` после `joint_kd_nms_rad`: `double leg_active_damping_nms_rad = 0.;`.
    `SpindleConfig` — внутренняя структура провода, её заполняет только кодек. Ноль в ней — как у соседних
    `joint_kp_nm_rad`, `joint_kd_nms_rad` (`:70-71`), это не значение профиля по умолчанию.
  - `state_codec.cpp`:
    - в `kConfigRequiredKeys` добавить `"leg_active_damping_nms_rad"` после `"joint_strength_path"`;
    - в `decode_actuation_scalars` после `joint_kd_nms_rad`:

```cpp
        config.leg_active_damping_nms_rad = d["leg_active_damping_nms_rad"].finite_real(path);
        if (config.leg_active_damping_nms_rad < 0.)
            invalid(std::string(path) + ".leg_active_damping_nms_rad", "must be nonnegative");
```

  Если подпись `invalid` в этом месте файла другая — взять ту, что в той же функции рядом, и записать в отчёт.

- [ ] **Шаг 3. Момент гашения.** В `spindle_controller.cpp` — дословно вариант V2
  (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-13-rising-leg-qoder/damp/diffs/v2.diff`). Единственное отличие:
  `kDampC` заменён на `config_.leg_active_damping_nms_rad`, а строка `constexpr double kDampC = 35.0;` убрана.
  - После `const double incoming_coast_hold_nm = coast_hold_nm_;` (`:1598`):

```cpp
        // Launch part 4b: the velocity-level kinematic reference of the leg
        // joints — the velocity analog of compute_press_state's weld closure
        // (qdot = J*rate - gamma*hip_vel - pelvis_rel_pitch_rate). The
        // damping torque -c*(qvel_leg - qdot_leg_kin) is a muscle torque
        // added to requested (non-priority) below, so it passes the
        // strength, power and priority budget like every other leg torque.
        // It damps the leg relative to the motion the crank and pelvis
        // impose, never the crank rotation itself.
        std::array<Vec2, 2> qdot_leg_kin{};
        {
            const auto rd = [&](int dof) {
                return dof >= 0 ? qvel[static_cast<std::size_t>(dof)] : 0.;
            };
            const std::array<double, 3> f_vel{rd(frame_root_x_dof_),
                                              rd(frame_root_z_dof_),
                                              rd(frame_pitch_dof_)};
            const std::array<double, 3> p_vel{rd(rider_root_x_dof_),
                                              rd(rider_root_z_dof_),
                                              rd(rider_pitch_dof_)};
            const double v_crank_x =
                f_vel[0] + (crank_world[2] - frame_pitch_anchor[2]) * f_vel[2];
            const double v_crank_z =
                f_vel[1] - (crank_world[0] - frame_pitch_anchor[0]) * f_vel[2];
            const double pelvis_rel_pitch_rate = p_vel[2] - f_vel[2];
            for (const std::size_t si : {0U, 1U}) {
                const double v_hip_x =
                    p_vel[0] +
                    (hip_world[si][2] - rider_pitch_anchor[2]) * p_vel[2];
                const double v_hip_z =
                    p_vel[1] -
                    (hip_world[si][0] - rider_pitch_anchor[0]) * p_vel[2];
                const Vec3 v_rel_world{
                    v_hip_x - v_crank_x - delta_hip[si][2] * f_vel[2], 0.,
                    v_hip_z - v_crank_z + delta_hip[si][0] * f_vel[2]};
                const Vec2 hip_vel_xz = mat_t_vec_xz(frame_r, v_rel_world);
                qdot_leg_kin[si][0] =
                    jacobians[si][0] * rate - gammas_x[si][0] * hip_vel_xz[0] -
                    gammas[si][0] * hip_vel_xz[1] - pelvis_rel_pitch_rate;
                qdot_leg_kin[si][1] =
                    jacobians[si][1] * rate - gammas_x[si][1] * hip_vel_xz[0] -
                    gammas[si][1] * hip_vel_xz[1];
            }
        }
```

  - В `:1716-1719`:

```cpp
                for (std::size_t i = 0; i < 2; ++i) {
                    priority_requested[slots[i]] = press_torques[i];
                    const double damp_nm =
                        -config_.leg_active_damping_nms_rad *
                        (qvel[leg_dofs[s][i]] - qdot_leg_kin[s][i]);
                    requested[slots[i]] =
                        task_torques[i] + press_torques[i] + damp_nm;
                    requested_filled[slots[i]] = 1;
                }
```

  Проверить по `facts.md` E.3, что все имена (`qvel`, `leg_dofs`, `frame_root_x_dof_` и другие) в этой области видны с
  тем же смыслом. Расхождение — стоп.

- [ ] **Шаг 4. Документы.**
  - TOML-тексты описи с `balance_dwell_s =`: после `joint_kd_nms_rad = 15.0` — `leg_active_damping_nms_rad = 35.0`.
  - JSON-документы провода: ключ `"leg_active_damping_nms_rad": 35.0` в каждый объект с `"joint_kd_nms_rad"`, место —
    по правилу инструмента описи (после `"joint_strength_path"`, если ключи упорядочены).
  - `schema.json`: запись поля по образцу `balance_dwell_s`.
  - Отчёт `launch4b/tools/task2-migration.txt`: число файлов и объектов, вывод `verify_migration.py`.

- [ ] **Шаг 5. Побайтная сверка** (решение 7).
  - Скопировать `.../kd/kdprobe.cpp` и `.../kd/build_kdprobe.sh` в `launch4b/parity/`. В копии скрипта пути
    драйвера — на `launch4b/parity/`, сборка — `build/native-release` ветки.
  - Собрать, прогнать 33 поездки списка с профилями репозитория:
    - обычный — `examples/research/viewer_physics_welded.toml`;
    - осторожный — `examples/research/rider_type_careful.toml`;
    - агрессивный — `examples/research/rider_type_aggressive.toml`;
    - трассы — из `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch3b/gradecap/tracks/`, остальные столбцы списка — как в
      `.../hold/run_sweep.sh`.
  - Сравнить с `launch4b/parity-ref/v2/runs/` через `cmp`. Сначала `diff` профилей-копий
    `.../kd/profiles/{welded,careful,aggressive}_kd15.toml` с профилями репозитория: допустимы только отличия из
    решения 7, иначе — стоп с разницей.
  - Всё совпадает — шаг пройден. Иначе стоп: поездка, файл, первая разная строка.

- [ ] **Шаг 6. Целевые тесты.**
  `mise run test 'articulated_config|state_codecs|welded_profile_config|seated_pedaling_cycle|realistic_pedelec_acceptance|seated_climb_surge|rider_senses|launch_intent|front_lift_reaction'`.
  - `seated_pedaling_cycle` (красный с части 4, задачи 4) — PASS по всем пунктам. Записать в отчёт худший
    «rising front leg press after BDC» по трём поездкам (Follow-up 5: +2,34 Н).
  - Остальные — PASS. Провал — стоп с числами, ожидания не трогать.

- [ ] **Шаг 7. Список эталонов** (решение 8): `launch4b/golden-list-task2.md`.

- [ ] **Шаг 8. Коммит.** `feat(rider): active damping of the leg-chain mode`.

- [ ] **Шаг 9. Проверка задачи** (без `ab`). Стопы «new red tests» и «causes differ» сверить со списком шага 7.

---

### Задача 3. Шатун на угле таблицы при поиске равновесия

**Файлы:**
- Изменить: `native/src/configuration/model_config.hpp` (поля после `equilibrium_refine_period_s`, записи схемы
  после `:458`), `native/src/configuration/validate.cpp` (после `:1267`, `default_physics`),
  `native/src/cli/configuration.cpp:217-218`, `native/src/initialization/settle_contacts.cpp`, программные
  конфигурации, TOML-тексты и JSON-документы описи
- Тест: `native/tests/reference/test_launch_preparation.cpp`, `native/tests/config/test_schema.cpp:739-768`

**Interfaces:**
- Consumes: задачи 1–2.
- Produces: `SimulationPhysicsConfig::equilibrium_crank_hold_kp_nm_rad`, `equilibrium_crank_hold_kd_nms_rad`.

- [ ] **Шаг 1. Тесты (падают).** В `test_launch_preparation.cpp` три теста. Определения — после `settle_once`
  (`:421-433`), их нужно вызвать в `main` после `test_setup_prepares_cog_and_angle_for_the_start_grade()`. Порядок:
  - сначала добавить только первый тест и запустить `mise run test 'launch_preparation'`. Ожидание: FAIL «the settle
    holds the crank» с числом ухода, которое записать в отчёт (Follow-up 8: от 1,12 шатун уходит к 0,596);
  - затем второй и третий тест. Они ссылаются на новые поля, поэтому сборка до шага 2 не проходит — это их провал.

```cpp
// Launch part 4b: the settle holds the crank at the table's pedal angle
// (spec 2026-11-11-rider-launch-plan-design.md:38). A free crank drifted
// to about 34 degrees from every forward seed; held, the settled crank
// stays within 5 mrad of the seed — a hold torque under 100 N·m at the
// 20000 N·m/rad hold stiffness — for a forward and a backward start.
[[gnu::noinline]] void test_settle_holds_the_crank_at_the_table_angle() {
    for (const double angle : {1.12, -.8}) {
        auto request = settle_request(.04);
        stepwise_cog(*request);
        request->model.physics->rider_launch.start_pedal_angle_rad = angle; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
        auto owner = init::make_setup(*request);
        const auto result = init::settle(*owner);
        bt::require(result.initial_crank_phase_rad == angle, "the settle starts from the table angle");
        bt::require(std::abs(result.settled_crank_phase_rad - angle) < 5e-3,
                    "the settle holds the crank at the table angle: " +
                        std::to_string(result.settled_crank_phase_rad) + " for " + std::to_string(angle));
    }
}

// The crank hold shapes the rest pose, so its gains are part of every
// reuse key: an explicit state file settled with another hold stiffness or
// hold damping is refused, and so is a transfer into such a start.
[[gnu::noinline]] void test_crank_hold_binds_cache_and_contract() {
    const bt::TemporaryDirectory cache("launch-hold-cache");
    auto request = settle_request(0.);
    request->model.physics->equilibrium_cache_enabled = true; // NOLINT(bugprone-unchecked-optional-access) set by settle_request
    request->cache_directory = cache.path();
    const auto miss = settle_once(*request);
    const auto entries = cache_entries(cache.path());
    bt::require(!miss.result.cache_hit && entries.size() == 1, "a cache miss settles one entry");
    auto pinned = std::make_unique<init::SetupRequest>(*request);
    pinned->model.physics->equilibrium_cache_enabled = false; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    pinned->model.physics->equilibrium_state_file = entries.front().string(); // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    pinned->model.physics->equilibrium_crank_hold_kp_nm_rad = 10000.; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    rejects([&] { (void)settle_once(*pinned); }, "fingerprint", "a state file settled with another hold stiffness");
    pinned->model.physics->equilibrium_crank_hold_kp_nm_rad = 20000.; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    pinned->model.physics->equilibrium_crank_hold_kd_nms_rad = 60.; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    rejects([&] { (void)settle_once(*pinned); }, "fingerprint", "a state file settled with another hold damping");
    auto plain = std::make_unique<init::SetupRequest>(*request);
    plain->model.physics->equilibrium_cache_enabled = false; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    auto source = init::make_setup(*plain);
    (void)init::settle(*source);
    auto other = std::make_unique<init::SetupRequest>(*plain);
    other->initial_state = source->capture_initial_state();
    other->model.physics->equilibrium_crank_hold_kp_nm_rad = 10000.; // NOLINT(bugprone-unchecked-optional-access) copied request carries physics
    auto refused = init::make_setup(*other);
    rejects([&] { (void)init::settle(*refused); }, "contract differs", "a transfer into a start with another hold stiffness");
}

// Every setup settles with the crank at the table angle (plan review I3).
// With the feet welded to the pedals (spindle: the distributed and
// pneumatic profiles, a rolling-start request) the settle holds it. A lumped
// rider loads no pedal and is not held — the explicit hold would ring its
// light crank — so its free, balanced crank keeps its world angle while the
// frame pitches: the frame-relative drift stays within the settled pitch.
// An articulated rider on flat pedals is not held either (the same light
// crank); it must still settle.
[[gnu::noinline]] void test_every_setup_keeps_the_crank_at_the_table_angle() {
    const auto distributed = launch_ride::repo_file("examples/research/viewer_physics_distributed.toml").string();
    const auto pneumatic = launch_ride::repo_file("examples/research/viewer_physics_pneumatic.toml").string();
    const auto welded = launch_ride::repo_file("examples/research/viewer_physics_welded.toml").string();
    const std::vector<std::vector<std::string_view>> held{
        {"--physics-config", distributed, "--track", "flat", "--initial-speed", "0"},
        {"--physics-config", pneumatic, "--track", "flat", "--initial-speed", "0"},
        {"--physics-config", welded, "--track", "flat", "--initial-speed", "15"},
    };
    const auto settle_cli = [](const std::vector<std::string_view> &args) {
        auto ride = bike::cli::resolve_ride(bike::cli::parse_arguments("bike-ride", args));
        ride->setup.model.physics->equilibrium_cache_enabled = false; // NOLINT(bugprone-unchecked-optional-access) resolve_ride sets physics
        auto owner = init::make_setup(ride->setup);
        return init::settle(*owner);
    };
    for (const auto &args : held) {
        const auto result = settle_cli(args);
        bt::require(std::abs(result.settled_crank_phase_rad - result.initial_crank_phase_rad) < 5e-3,
                    "a welded-feet setup holds the crank at the table angle: " + std::string(args[1]) + " settled " +
                        std::to_string(result.settled_crank_phase_rad) + " from " +
                        std::to_string(result.initial_crank_phase_rad));
    }
    const auto lumped = settle_cli({"--physics", "physical", "--rider", "lumped", "--track", "flat"});
    bt::require(std::abs(lumped.settled_crank_phase_rad - lumped.initial_crank_phase_rad) <=
                    std::abs(lumped.pitch_rad) + 1e-3,
                "a lumped rider's free crank keeps its world angle: drift " +
                    std::to_string(lumped.settled_crank_phase_rad - lumped.initial_crank_phase_rad) + ", pitch " +
                    std::to_string(lumped.pitch_rad));
    const auto flat = settle_cli({"--physics", "physical", "--drive", "articulated_effort", "--track", "flat"});
    bt::require(std::isfinite(flat.settled_crank_phase_rad) && std::isfinite(flat.pitch_rad),
                "an articulated rider on flat pedals settles without the hold");
}
```

  `launch_ride::repo_file` — из `launch_ride_support.hpp` (уже подключён, `:34, :43`; так же вызывается в `:309`).
  Уход шатуна у сосредоточенного и у плоских педалей — в отчёт. Плоские педали: если CLI с этими аргументами даёт
  не `pedal_attachment = "flat"`, найти аргументы, которые его дают, и записать в отчёт. Если имя аргумента CLI другое
  (`--physics-config`, `--initial-speed` — `cli/argument_contracts.inc:503, :515`), взять настоящее и записать в
  отчёт. Если у CLI-поездки нет физики в `setup.model.physics`, взять поле, которое меняет тест
  `test_lumped_rider_ignores_the_prepared_lean` (`:519-521`). `--initial-speed` — в км/ч. Скорость задаётся после
  поиска равновесия (`finalize.cpp:101`), поэтому третий случай проверяет только поиск позы для запроса со
  стартом на ходу, а не саму поездку.

  - В `test_schema.cpp:739-768` массив `kTopOrder` 24 → 26: после `"equilibrium_refine_period_s"` —
    `"equilibrium_crank_hold_kp_nm_rad"`, `"equilibrium_crank_hold_kd_nms_rad"`; проверка `fields.size() == 26`.
    Помощник `:193` — `c.equilibrium_crank_hold_kp_nm_rad = 20000.0; c.equilibrium_crank_hold_kd_nms_rad = 88.0;`.

  Запустить `mise run test 'launch_preparation'`. Ожидание: сборка не проходит (полей нет) — это и есть провал.

- [ ] **Шаг 2. Поля.**
  - `model_config.hpp`, после `equilibrium_refine_period_s`:

```cpp
    // Launch part 4b (user ruling 2026-11-13): the settle holds crank_spin at
    // the prepared pedal angle with this PD torque
    // (initialization/settle_contacts.cpp). It shapes the rest pose, so both
    // gains bind the equilibrium key and the initial-state contract.
    // The torque enters as an external force, so the step integrates it
    // explicitly: the settle is stable only while kd*h/I < 2 - kp*h*h/(2*I).
    // With the feet welded to the pedals (spindle, the only setups the hold acts in) the
    // decoupled crank inertia I is about 0.096 kg·m²: kd 200 at h = 1.25 ms
    // diverged, 88 is the critical damping of kp 20000.
    // Required from the profile.
    double equilibrium_crank_hold_kp_nm_rad = -1.;
    double equilibrium_crank_hold_kd_nms_rad = -1.;
```

  - Две записи схемы после записи `equilibrium_refine_period_s`, по её образцу, с `.required=true`.
  - `validate.cpp` после проверки `equilibrium_refine_period_s`:

```cpp
    scalar(config.equilibrium_crank_hold_kp_nm_rad,
           "equilibrium_crank_hold_kp_nm_rad", std::nullopt, true);
    scalar(config.equilibrium_crank_hold_kd_nms_rad,
           "equilibrium_crank_hold_kd_nms_rad", std::nullopt, true);
```

  - `default_physics`, `cli/configuration.cpp:217-218` и программные конфигурации описи:
    `...equilibrium_crank_hold_kp_nm_rad = 20000.; ...equilibrium_crank_hold_kd_nms_rad = 88.;`.

- [ ] **Шаг 3. Удержание.** Строки `[F8-hold]` из `equilibrium.cpp` копии в код не переносятся. Отчёты задач 3–4
  берут уход шатуна из `settled_crank_phase_rad − initial_crank_phase_rad` (результат равновесия, метаданные
  подготовки), а момент удержания — как `kp · |уход|`.

  `settle_contacts.cpp` — копия qoder с одним намеренным отличием: признак решения 2 (у копии удержание без
  условия). При приваренных ногах байты те же. Сама копия —
  (`.../hold/src_mod/initialization/settle_contacts.cpp`) без строки печати в `equilibrium.cpp`. Постоянные
  заменены полями профиля:

```cpp
namespace {
// Launch part 4b (user ruling 2026-11-13): the settle holds crank_spin at
// the prepared pedal angle, the way it holds the prepared lean — spec
// 2026-11-11-rider-launch-plan-design.md:38. A stiff PD restoring torque
// through the step's external generalized-force channel: the constraint
// solver balances it like any applied torque, so the crank reaches
// qacc ~ 0 at the held pose (a kinematic clamp leaves the reaction on the
// pedal hinges and does not converge — experiment Follow-up 8). Setup
// only: require_initializing guards the caller, and the ride's steps pass
// no external force, so nothing holds the crank in the ride.
double crank_hold_torque(SetupContext const& context) {
    const auto crank=joint_address(context,"crank_spin");
    const mjModel* m=context.stepper.model();const mjData* d=context.stepper.data();
    const auto qpos=std::views::counted(d->qpos,m->nq);
    const auto qvel=std::views::counted(d->qvel,m->nv);
    const double seed=context.physics.drive.crank_phase_rad;
    return -context.physics.equilibrium_crank_hold_kp_nm_rad*(qpos[static_cast<std::size_t>(crank.qpos)]-seed)
           -context.physics.equilibrium_crank_hold_kd_nms_rad*qvel[static_cast<std::size_t>(crank.dof)];
}
} // namespace
```

  В `apply_initial_forces` вместо `std::nullopt`:

```cpp
    // Launch part 4b: with the feet welded to the pedals (spindle) the same
    // settle also holds the crank at the table angle. Otherwise (no rider, a
    // lumped rider, flat pedals; weld is unmeasured) the crank is only its
    // own light assembly, about 0.013 kg·m², and the explicit hold would
    // ring it — it settles free, as before the hold.
    std::optional<std::vector<double>> external;
    if(context.physical.rider_present()&&context.physics.articulated.pedal_attachment=="spindle"){
        const mjModel* m=context.stepper.model();
        std::vector<double> hold(static_cast<std::size_t>(m->nv),0.);
        const auto crank=joint_address(context,"crank_spin");
        hold[static_cast<std::size_t>(crank.dof)]=crank_hold_torque(context);
        external=std::move(hold);
    }
    (void)context.physical.apply_forces(false,advance,
        context.physics.initial_front_brake,context.physics.initial_rear_brake,
        external,control);
```

  Добавить `#include <optional>`, `#include <ranges>`, `#include <utility>` и `#include <vector>`, если их нет. Комментарий над `apply_initial_forces`
  (`:12-15`) дополнить одной строкой: «and the prepared pedal angle (launch part 4b)».

  Что знать о краях (в отчёт, без правок):
  - `finalize.cpp:85` зовёт `apply_initial_forces` до `initial_speed` (`:101`). Поэтому состояние шин и контактов в
    момент t = 0 снято под удержанием, а первый шаг поездки снимает момент до ~22 Н·м. Записать в отчёт скачок
    скорости шатуна за первые 10 шагов поездки обычного райдера на g00.
  - `initial_state.cpp:196-197` при восстановлении переноса зовёт `apply_initial_forces` с перенесённой `qvel`. Для
    переноса на ходу в `initial_state_target_residual_qacc` входит `−kd·qvel[crank]`. Это только запись невязки,
    проверки по ней нет.
  - Поля удержания остаются в контракте начального состояния (`initial_state.cpp:54` их не стирает), хотя
    `equilibrium_refine_*` там стираются. Причина: поля расписания не меняют позу, а удержание меняет — поза
    отходит на `T/kp`. Одна строка об этом — в комментарий у полей в `model_config.hpp`.

- [ ] **Шаг 4. Документы.**
  - TOML-тексты описи для корня физики: после `equilibrium_refine_period_s = ...` (в строках без него — после
    последнего ключа корня перед первым разделом) вставить `equilibrium_crank_hold_kp_nm_rad = 20000.0` и
    `equilibrium_crank_hold_kd_nms_rad = 88.0`.
  - JSON-документы с `"equilibrium_refine_period_s"`: оба ключа по правилу инструмента.
  - `schema.json`: две записи.
  - `replay/*/summary.json` — входы: переводятся так же.
  - Записанные начальные состояния (`facts.md` B.4: 10 JSON, `savage_pneumatic.npz`) не трогать. Их контракт и
    ключ теперь не совпадают, тесты-читатели уже красные; перезапись — часть 4, задача 7.

- [ ] **Шаг 5. Поездки без сверки** (решение 7): драйвер задачи 2 пересобрать, прогнать 33 поездки. Исход
  каждой поездки, `|settled − initial|` и `kp·|settled − initial|` — в отчёт. Поездка, которая не встала в позу, или
  уход ≥ 5e-3 рад — стоп.

- [ ] **Шаг 6. Целевые тесты.**
  `mise run test 'launch_preparation|config_schema|setup|initialization_sag|welded_profile_config|seated_pedaling_cycle|realistic_pedelec_acceptance|seated_climb_surge|rider_senses|launch_intent|front_lift_reaction'`.
  - `launch_preparation` — PASS, включая три новых теста. `settled − initial` каждого случая — в отчёт.
  - Тест `test_setup_prepares_cog_and_angle_for_the_start_grade` проверяет, что углы 0,6 и 0 дают разный шатун; он
    должен остаться зелёным.
  - Остальные зелёные — PASS. `config_schema` — причина та же, что в базе. Провал проверки поведения — стоп.

- [ ] **Шаг 7. Список эталонов** (решение 8): `launch4b/golden-list-task3.md`.

- [ ] **Шаг 8. Коммит.** `feat(initialization): the settle holds the crank at the table angle`.

- [ ] **Шаг 9. Проверка задачи** (без `ab`) — как в задаче 2.

---

### Задача 4. Угол −0,8 рад и таблицы старта под него

**Файлы:**
- Изменить: `examples/research/viewer_physics_{welded,distributed,pneumatic}.toml`,
  `examples/research/rider_type_{careful,aggressive}.toml` — разделы `[rider_launch]`
- Изменить: `native/tests/reference/test_welded_profile_config.cpp` (`:391-392`, `:443-444`, `:495-496` и числа
  таблиц решения 6, `:765`, `:901-902`, `:909-912`, `:943`)

- [ ] **Шаг 1. Тест профилей (падает).** В `test_welded_profile_config.cpp`:
  - ожидаемые таблицы трёх типов (`:380-540`) — числа решения 6;
  - строка `"start_pedal_angle_rad = 1.12\n"` (`:765`) → `"start_pedal_angle_rad = -0.8\n"`;
  - `:901-902`, `:943`: `drive.crank_phase_rad == -.8`;
  - `:909-910`: замена `-0.8` → `0.5`; проверка `:912` остаётся `== .5`.

  Запустить `mise run test 'welded_profile_config'`. Ожидание: FAIL на угле и числах таблиц.

- [ ] **Шаг 2. Профили.** В пяти файлах разделы `[rider_launch]` — числа решения 6. Над изменёнными строками —
  комментарий «launch part 4b — the -0.8 rad start (Follow-up 10, user ruling 2026-11-13)». У осторожного и
  агрессивного над каждым отличием от обычного остаётся комментарий с числом обычного.

- [ ] **Шаг 3. Побайтная сверка** с итоговыми поездками Follow-up 1 разбора pneumatic (решение 7): все 96 поездок
  (78 набора `c3` и 18 решения 6: мокрые 0 и 5 % у трёх типов, по шесть на distributed и pneumatic), все три
  файла. Профили distributed и pneumatic — файлы репозитория, их копии — `.../cal/extra/candidates/`.
  Перед этим `diff` профилей-копий `.../cal/candidates/c3/{welded,careful,aggressive}.toml` и
  `.../cal/extra/candidates/{c3wet,distributed,pneumatic}/*.toml` с профилями
  репозитория: допустимы только отличия из решения 7. Иначе — стоп с разницей.

- [ ] **Шаг 4. Строгий итог поездок.** По 96 поездкам шага 3 — правило решения 10 части 4 и П10–П13 (вывод
  `cal_metrics.py` с `--window-handover --rb-from-release`, как в части 4):
  - все 27 поездок без остановки, 45 промежуточных и 18 дополнительных — PASS. Исключение — g00 на distributed:
    там 20 км/ч приходят за 7,3 с при пороге 3,25 с, и на `1b75ae48` это было так же (7,43 с). Причина — потери
    шин distributed, около 600 Вт; задача в очереди «Distributed tyre»;
  - 6 поездок с остановкой — только в отчёт (очередь П9);
  - у каждой поездки `|settled − initial| < 5e-3` рад: удерживающий момент меньше 100 Н·м;
  - худший запас в мёртвой точке и `t20` обычного райдера на g00 — в отчёт.

  Провал любой другой поездки — стоп с поездкой и метрикой.

- [ ] **Шаг 5. Опорные тесты.**
  `mise run test 'welded_profile_config|seated_pedaling_cycle|realistic_pedelec_acceptance|seated_climb_surge|rider_senses|launch_intent|front_lift_reaction|launch_preparation'` — все PASS. Строку `launch ride:` из `rider_senses` — в отчёт.

- [ ] **Шаг 6. Тяжёлые трассы.** `build/native-release/bin/bike-measure-realtime` на `rough_uphill_extreme` и
  `savage`: время до остановки, причина, первое нарушение, RTF. Сравнить с `1b75ae48` (13,10 с и 6,36 с) и с
  Follow-up 10. Это сведения, а не критерий (У7): в отчёт.

- [ ] **Шаг 7. Список эталонов** (решение 8): `launch4b/golden-list-task4.md`.

- [ ] **Шаг 8. Коммит.** `feat(profiles): the -0.8 rad start and its launch tables`.

- [ ] **Шаг 9. Проверка задачи** (без `ab`) — как в задаче 2.

---

### Задача 5. Документ решений и передача части 4

**Файлы:**
- Изменить: `docs/agents/simulation-decisions.md`

- [ ] **Шаг 1. Записи «Действует».** Места — `facts.md` H.2:
  - «Начальное состояние» (`:258`), рядом с п. 33:
    - поза покоя считается для угла таблицы, шатун удерживается при поиске, в поездке свободен;
    - поля удержания и их числа;
    - удержание входит в ключ кэша и в контракт;
    - почему гашение удержания 88, а не 200: граница устойчивости явного момента (решение 1);
    - удержание только при приваренных ногах (`spindle`). У сосредоточенного райдера, без райдера, на плоских
      педалях и на `weld` шатун при поиске свободен, как раньше. Для этих настроек требование спецификации `:38`
      пока не выполнено (решение 2);
    - ссылка на «Не решено» п. 6.
  - «Райдер» (`:39`):
    - активное гашение ноги — момент контроллера `−c·(q̇ − q̇_kin)`, c = 35, проходит пределы;
    - явно: это не демпфирование шарнира из п. 26 и не «Отвергнуто» 1–2;
    - при накате действует так же: гасит движение ноги относительно цепи, а не вращение шатуна (п. 9).
  - Жёсткость и гашение позы ног 600 / 15 — из профиля, числа не менялись («Пробовали» п. 4).
  - Угол старта −0,8 рад у трёх типов и таблицы — решение 6 со ссылкой на Follow-up 10.
- [ ] **Шаг 2. «Пробовали — не сработало».**
  - Прямая подача решённых ускорений ног (Follow-up 4): алгебраическая положительная петля, нажим 354–452 Н.
  - Низкочастотный фильтр момента задачи шатуна (Follow-up 5, вариант B).
  - Исправленное деление нажима между суставами (Follow-up 7): разрыв 175 Н·м при 102,3°, седло на растяжение.
  - Гашение удержания 200 (Follow-up 8–11): раскачка разомкнутого шатуна, ложная «стена» за −0,9 рад.
- [ ] **Шаг 3. Коммит.** `docs(agents): the held crank, the active leg damping and the -0.8 rad start`.

После этой задачи оркестратор:
- правит план части 4: решение 5 → таблицы 4б, список эталонов задачи 7 дополняется списками 4б;
- продолжает часть 4 с задачи 5.

---

## Самопроверка плана

- Спецификация `:38` (поза для угла таблицы) — задача 3. Решения пользователя: У1, У6 — задача 1; У2 — задача 2;
  У3 — задача 3; У4, У5 — задача 4; У7 — задача 4, шаг 6, и часть 4.
- Имена: `leg_active_damping_nms_rad`, `equilibrium_crank_hold_kp_nm_rad`, `equilibrium_crank_hold_kd_nms_rad` —
  одинаково в коде, схеме, документах и тестах.
- Числа таблиц решения 6 и удержания решения 1 утверждает пользователь до ревью плана.
