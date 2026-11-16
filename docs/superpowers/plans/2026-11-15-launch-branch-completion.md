# Завершение ветки старта райдера — план

> **Статус: план полностью выполнен (2026-11-15).** Кодовая база на `bc247283` прошла полный прогон `bash tools/run_tests.sh full` (`2261 s`, exit `0`: `194/194` обычных тестов, все 9 проверок без находок при штатном `misc-no-recursion=15`, `49/49` `slow`, `1/1` `package_smoke`), экстремальный прогон `rough_uphill_extreme`, 6 информационных стартов (`distributed` / `pneumatic`) и локально влита в `main` через `--ff-only` (без push). Последующие правки документации проверены по уровню 1 (`check_change.sh main`).

> **Для исполнителей-агентов:** обязательный навык — `superpowers:subagent-driven-development`. Шаги отмечаются
> флажками (`- [ ]`). Оркестратор — Claude (эта сессия). Исполнитель каждой задачи указан в её заголовке и в
> таблице «Исполнители». Финальное ревью ветки делает оркестратор сам, без агентов.

**Цель.** Довести ветку `physics-2026-11-05` до полностью зелёной полной проверки, провести финальное ревью и
слить её в `main` (локально, без push). Остальное — очередь с отдельными планами.

**Устройство.** Пять задач по порядку: класс C (две задачи), ревью ветки, итоговая проверка (шаг 9 плана части 4),
слияние. Полная проверка `run_tests.sh full` запускается один раз — в задаче 4, на коде после правок ревью.

**Технологии.** C++23, CMake/CTest, MuJoCo; проверки — `tools/check_change.sh`, `tools/run_tests.sh`.

**Источники.**
- план части 4: `docs/superpowers/plans/2026-11-12-launch-4-activation.md` (задача 9 — итоговая проверка);
- отчёт по классу C: `/Volumes/SSD_1TB_M2/work/mujoco_clean-scratch/devin/2026-11-14-class-c-fixtures/report.md`
  (готовые и проверенные правки — раздел «Stale test-code inventory»);
- журнал: `/Volumes/SSD_1TB_M2/work/mujoco_clean-scratch/devin/2026-11-05-physics-queue/ledger.md`.

## Глобальные ограничения

- Рабочее дерево: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/physics-2026-11-05`, ветка
  `physics-2026-11-05`, сборка `build/native-release` (не создавать новых каталогов сборки).
- Не менять пределы, пороги, силы и допуски; не ослаблять и не переставлять проверки.
- `native/src` не менять, кроме случаев, прямо названных в задаче.
- Никаких push, `git stash`, `ninja -t`, отладчиков, интерактивных команд. Долгие команды — с ограничением
  времени: `perl -e 'alarm 2400; exec @ARGV' --`.
- Не трогать `.claude/` и `.agents/`; не добиваться чистого `git status`.
- Последняя строка каждого коммита ровно `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`, других строк
  об авторстве нет. В индекс — только свои файлы, явными путями.
- Логи и отчёты — на `/Volumes/SSD_1TB_M2/work/tmp/2026-11-15-branch-completion/`. Свободного места на SSD не меньше
  50 ГБ.
- Документы на русском, код и комментарии в коде — на английском.

## Исполнители

| Работа | Оркестратор и модель | Почему |
|---|---|---|
| Механические правки по готовому списку (задача 1) | `devin-orchestration`, `swe-2-flash-max` | правки уже проверены на копии |
| Правки тестов с решением (задача 2), исправления после ревью (задача 3, шаг 3), информационные поездки (задача 4, шаг 2) | `devin-orchestration`, `swe-2-max` | основной по таблице codex alibaba `qwen4-max-preview` дважды зациклился 2026-11-14 |
| Финальное ревью ветки (задача 3) | оркестратор сам | правило: финальное ревью — моё |
| Проверки `check_change.sh`, `run_tests.sh full`, поездка `extreme` (задачи 1–4) | оркестратор, фоновый Bash | длинные команды без агента |
| Запасной исполнитель кода | `qoder-orchestration`, `Ultimate` на основном аккаунте | если Devin упал дважды |

Запуск Devin: `bash ~/.claude/skills/devin-orchestration/scripts/devin-run.sh <run-dir> --workdir <рабочее дерево> --model <модель>`,
сторож — `devin-watch.py <run-dir>`. Если `~/Desktop/Projects_my` снова недоступна (`Operation not permitted`),
то запасная копия: `.scratch/devin-skill-copy/.claude/skills/devin-orchestration/` (прежний вариант без ACP).
Бриф — шаблон `task.md` того же навыка. Папки запусков — `.scratch/devin/2026-11-15-<slug>/`.

## Участие пользователя

Одобрение этого плана заменяет отдельные «да» на всё, что в нём расписано, включая локальное слияние в `main` в
задаче 5. Оркестратор решает сам:
- находки ревью уровней Critical и Important исправляются сразу (задача 3), Minor — в очередь;
- эталоны linux-aarch64 не перезаписываются (Linux — только по явной просьбе), пункт уходит в очередь;
- находки `savage_pneumatic.npz` и `model_valid` в `one_second_pair` уходят в очередь.

Оркестратор останавливается и спрашивает пользователя только в таких случаях:
- исправление требует менять предел, порог, силу, допуск или решение из `docs/agents/simulation-decisions.md`;
- красный тест, разницу которого нельзя привязать к намеренному коммиту;
- `run_tests.sh full` красный по причине, которую нельзя исправить без такого изменения;
- исполнитель дважды упал на одной задаче, и запасной тоже.

## Бюджет тестов

Замеры из `<git-common-dir>/bike-gate/timings.tsv`: `full` ≈ 1500 с (из них slow ≈ 800 с), `native` ≈ 600 с,
`check_change.sh` — до 5 мин. Тесты поездок старта — около 200 с каждый.

| Задача | Прогон | Время |
|---|---|---|
| 1, 2 | целевые тесты своих файлов | по 1–2 мин |
| 2 | `check_change.sh be045bda` + шесть тестов, которые должны оставаться зелёными | ≈ 15 мин |
| 3 | `check_change.sh` после исправлений | ≈ 5 мин |
| 4 | поездка `extreme` 60 с, информационные поездки, `run_tests.sh full` | ≈ 40 мин |

Итого около 65 минут тестов, полная проверка — один раз.

---

### Задача 1. Устаревшие числа в трёх тестах класса C

**Исполнитель:** `devin-orchestration`, `swe-2-flash-max`.

**Файлы:**
- Изменить: `native/tests/config/test_schema.cpp:806,808`
- Изменить: `native/tests/config/test_resolution.cpp:452-454, 1339-1341, 1384-1397, 1413-1417`
- Изменить: `native/tests/model/test_mjcf.cpp:163, 185, ~305, 381`

Фикстуры этих тестов уже пересозданы коммитами `251ae25c`, `1751066e`. Каждая правка ниже проверена Devin на копии
теста: исправленный `test_schema` проходит 2024 проверки, `test_resolution` — 569, `test_mjcf` — 5827.

- [x] **Шаг 1.** Убедиться, что три теста красные именно на этих строках:
  `perl -e 'alarm 600; exec @ARGV' -- ctest --test-dir build/native-release -R '^(config_schema|config_resolution|model_mjcf)$' --output-on-failure`.
  Ожидается: три FAIL, сообщения называют строки из списка. Если сообщение другое — стоп, отчёт с текстом.
- [x] **Шаг 2.** `test_schema.cpp`. Число полей считать из схемы нельзя: тест проверяет, что схема не изменилась
  молча. Поэтому меняются только числа:

  ```cpp
  // L806: 29dd2d1c removed coasting_brake_d_nms_rad and coasting_brake_limit_nm, bb5894d7 added leg_active_damping_nms_rad.
  fields_named(fields[9], "articulated").size() == 65
  // L808: 9e9361ae removed launch_effort_ramp_s, launch_min_grade, launch_standstill_crank_rate_rad_s.
  fields_named(fields[10], "seated_climb").size() == 19
  ```
  Остальной текст строк не трогать, комментарии добавить одной строкой над каждой проверкой.
- [x] **Шаг 3.** `test_resolution.cpp`:
  - L452–454: ожидаемое сообщение →
    `missing 3 required positional arguments: 'rider_launch', 'front_lift_reaction', and 'monitors'`
    (`e3e537ce` сделал `[front_lift_reaction]` обязательным);
  - L1339–1341 и L1413–1417: значение переопределения `timestep_s` `0.0005` → `0.00025`. Смысл стадии («явный шаг
    побеждает, все периоды остаются целыми») сохраняется: период намерений 5 шагов, такт 20, удержание 20,
    выпадение 50. Причина — правило целых тактов `c7f0c811`;
  - L1384–1397: в тот же объект переопределения добавить `{"drive", {"motor_clutch": false}}`. Стадия ставит
    `drive_mode = "coast"`, а муфта мотора (`cfbf847e`) требует режим с усилием.
- [x] **Шаг 4.** `test_mjcf.cpp`. Числа берутся из той же модели, что и в фикстуре, но тест проверяет топологию
  модели, поэтому остаются числами:
  - L163: `model->nq == 27 && model->nv == 27`;
  - L185: `welded->nq == 29`;
  - L381: `second->nq == 27`;
  - стадия `rotor_body_welds_to_the_drive_axis` (~L305): перед сборкой модели добавить
    `rigid->physics->drive.motor_clutch = false;` — стадия проверяет жёсткий привод (ротор → шатун), а профиль с
    `cfbf847e` собирает модель с муфтой.

  Над каждой правкой — одна строка комментария с коммитом `cfbf847e` (шарнир `drive_shaft_spin`).
- [x] **Шаг 5.** Собрать и перезапустить:
  `perl -e 'alarm 1200; exec @ARGV' -- mise run test '^(config_schema|config_resolution|model_mjcf)$'`.
  Ожидается: 3 из 3 PASS.
- [x] **Шаг 6. Коммит.** Три файла явными путями:
  `test(config): update the schema, resolution and MJCF expectations to the current model`.
  В теле — по строке на правку с коммитом-причиной.

**Готово, когда** три теста зелёные и коммит сделан.

---

### Задача 2. Старая запись в `replay_recording_v3`

**Исполнитель:** `devin-orchestration`, `swe-2-max`.

**Файлы:**
- Изменить: `native/tests/reference/test_replay_recording_v3.cpp:131-191` (функции `removed_battery_gate`,
  `removed_rider_fields_gate`, вызовы в `main`)
- Фикстура `tests/fixtures/replay/legacy_v2/` остаётся как вход отказа (контрольная сумма уже обновлена `0be18825`)

Решение пользователя 2026-11-14: читателя старых форматов не делать, проверку пересборки старой записи убрать,
проверять понятный отказ. Сейчас пересборка `legacy_v2` падает на `unknown PhysicalDriveConfig parameter(s):
bearing_c_nms_rad, chain_c_ns_m, chain_k_n_m, freehub_c_nms_rad, freehub_k_nm_rad` (поля удалены `c222a6aa`,
`be37a151`), а без них упала бы на отсутствии `rider_launch` и `front_lift_reaction`.

- [x] **Шаг 1.** Узнать тип исключения: в `/Volumes/SSD_1TB_M2/work/tmp/2026-11-15-branch-completion/` собрать
  маленький зонд или прочитать `native/src/replay/rebuild.cpp` (`request_from_input`) и
  `native/src/configuration/*` — какой класс бросает «unknown PhysicalDriveConfig parameter(s)». Записать в отчёт
  файл и строку.
- [x] **Шаг 2.** Заменить обе функции одной:

  ```cpp
  // Recordings made before the lossless drivetrain (c222a6aa, be37a151) and
  // the launch sections (a9a0b689, e3e537ce) are not rebuilt: there is no
  // legacy reader (user ruling 2026-11-14). The rebuild must refuse them by
  // naming the removed field instead of replaying a different bike.
  void legacy_recording_refused() {
      const auto input = rr::replay::load_legacy_input(bt::fixture("replay/legacy_v2"));
      const rr::replay::FixedAssets assets(input);
      std::string message;
      try {
          (void)rr::replay::request_from_input(input, assets);
      } catch (const std::exception& failure) {
          message = failure.what();
      }
      bt::require(message.find("bearing_c_nms_rad") != std::string::npos,
                  "the legacy rebuild is refused by naming a removed drive field (got: " + message + ")");
  }
  ```
  Если шаг 1 показал конкретный класс исключения, ловить его вместо `std::exception`. В `main` вызвать
  `legacy_recording_refused()` вместо двух прежних функций. Остальные шесть проверок теста не трогать.
- [x] **Шаг 3.** Если после правки в `native/src/replay/rebuild.cpp` или `native/src/configuration/legacy_*.hpp`
  остались функции, которые больше не вызывает ни один тест и ни один путь кода (`drop_removed_battery`,
  `inert_rider`), — не удалять их здесь, а записать в отчёт со строками: это пункт очереди «убрать читатели старых
  форматов».
- [x] **Шаг 4.** Перезапустить:
  `perl -e 'alarm 1200; exec @ARGV' -- mise run test '^reference\.replay_recording_v3$'`. Ожидается PASS.
- [x] **Шаг 5. Коммит.** `test(replay): refuse the pre-lossless legacy recording instead of rebuilding it`; в теле —
  решение пользователя и коммиты `c222a6aa`, `be37a151`, `a9a0b689`, `e3e537ce`.
- [x] **Шаг 6. Проверка этапа.** Из рабочего дерева: `bash tools/check_change.sh be045bda` — ожидается выход 0, ни
  одного красного теста. Затем шесть тестов, которые должны оставаться зелёными:
  `perl -e 'alarm 2400; exec @ARGV' -- ctest --test-dir build/native-release -R '^(reference\.rider_launch_rides_(normal|careful|aggressive)|seated_pedaling_cycle|seated_posture_program|native_recording_retention)$'`.
  Обновить `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/reds-after-rerecord-b.txt`: красных нет.

**Готово, когда** `check_change.sh` выходит с 0, шесть тестов зелёные, коммит сделан.

Оркестратор после задач 1–2 сам смотрит дифф кода тестов (`git diff be045bda..HEAD -- native/tests`).

---

### Задача 3. Финальное ревью ветки

**Исполнитель:** оркестратор сам (шаги 1–2, 4), исправления — `devin-orchestration`, `swe-2-max` (шаг 3).

Объём: `git log 98996e58..HEAD` (основа `main`), около 215 коммитов: части 1–4 и 4б, перезаписи эталонов, правка
`research.cpp`, помощник записи начальных состояний.

- [x] **Шаг 1. Пакет.** Сгруппировать коммиты по частям плана (журнал называет диапазоны). Код, взятый из плана
  дословно, сверить механически: блок кода плана против файла. Свои коммиты оркестратора (документы, планы)
  пропустить. Коммиты только с фикстурами — проверить выборочно: структура эталонов (метки, виды, допуски) не
  менялась, кроме удалённых или добавленных каналов с коммитом-причиной.
- [x] **Шаг 2. Чтение.** Для кода в `native/src` и тестов: корректность, соответствие спецификации
  `docs/superpowers/specs/2026-11-11-rider-launch-plan-design.md` и документу решений, отсутствие значений по
  умолчанию в коде для полей профиля, отсутствие ослабленных проверок, мёртвый код. Разобрать отложенные
  замечания Minor из журнала. Вердикт и список находок (файл:строка, уровень) — в журнал.
- [x] **Шаг 3. Исправления.** Все находки Critical и Important — одним запуском Devin `swe-2-max`, бриф со списком
  находок через `KEY=@файл`. Каждое исправление поведения — с тестом (сначала красный). Minor — в раздел очереди
  журнала.
- [x] **Шаг 4. Проверка исправлений.** Оркестратор читает дифф исправлений сам; затем
  `bash tools/check_change.sh <коммит до исправлений>` — выход 0.

**Готово, когда** вердикт записан в журнал, исправления приняты и `check_change.sh` зелёный.

---

### Задача 4. Итоговая проверка (шаг 9 плана части 4)

**Исполнитель:** оркестратор (шаги 1, 3, 4), `devin-orchestration`, `swe-2-max` (шаг 2).

- [x] **Шаг 1. `rough_uphill_extreme`.**
  `perl -e 'alarm 1800; exec @ARGV' -- build/native-release/bin/bike-measure-realtime --out /Volumes/SSD_1TB_M2/work/tmp/2026-11-15-branch-completion/extreme --physics-config examples/research/viewer_physics_welded.toml --track examples/research/rough_uphill_extreme.toml --duration 60 --warmup 0 --runs 1`.
  Сравнить с основой `/Volumes/SSD_1TB_M2/work/tmp/2026-11-12-launch4/base-extreme/` (`22dc1b0d`): `.runs[0].prefix`,
  время счёта, число входов реакции в фазу `acting`. Ухудшение префикса — не стоп, а строка отчёта с причиной.
- [x] **Шаг 2. Информационные поездки** на `distributed` и `pneumatic`: обычный райдер, 0, 10 и 15 % на асфальте.
  Метрики — `launch_ride_metrics` (`native/tests/reference/launch_ride_metrics.hpp`) через маленький исполняемый
  файл в `/Volumes/SSD_1TB_M2/work/tmp/2026-11-15-branch-completion/info-rides/`, не в репозитории. Это
  информация, порогов нет. Таблица — в отчёт Devin.
- [x] **Шаг 3. Полная проверка.** `bash tools/run_tests.sh full` в фоне. Ожидается: выход 0, ни одного красного
  теста, медленные и пакетные метки прошли. Красный тест — исправить по правилам задачи 3, шаг 3, и повторить
  только полную проверку. Решение, которое требует менять порог, — стоп и пользователь.
- [x] **Шаг 4. Отчёт** пользователю: итог, время полной проверки, три таблицы тестов поездок (печать
  `reference.rider_launch_rides_*`), сравнение `extreme` с основой, таблица шага 2.

**Готово, когда** полная проверка вышла с 0 на HEAD и отчёт отправлен.

---

### Задача 5. Слияние и уборка

**Исполнитель:** оркестратор.

- [x] **Шаг 1.** Убедиться, что `main` не ушёл вперёд: `git -C <главная копия> log --oneline -1 main` равно
  `98996e58` или является предком ветки. Если ушёл — стоп и пользователь.
- [x] **Шаг 2.** Слить fast-forward: `git -C <главная копия> merge --ff-only physics-2026-11-05`. Без push.
  Полная проверка задачи 4 на том же HEAD засчитывается как проверка уровня 2 перед слиянием.
- [x] **Шаг 3.** Дорожная карта `docs/superpowers/plans/2026-11-11-rider-launch-plan.md` и этот план: отметить
  выполненное. Журнал: итог. Память: обновить запись о состоянии `main`.
- [x] **Шаг 4.** Рабочее дерево не удалять (удаление — только по слову пользователя); сообщить, что его можно
  убрать.

**Готово, когда** `main` указывает на HEAD ветки, записи обновлены.

---

## Очередь после слияния

Каждый пункт — отдельный план; оркестратор берёт их по порядку и спрашивает пользователя только о решениях из
списка «Участие пользователя». Порядок — по правилу «обычные поездки в первую очередь».

1. **Переключение вниз перед остановкой и рестарты** (П9, «Решено — не сделано», пункт 16). **Выполнено на ветке `queue-2026-11-15`** (`16b933a9`, `4491661a`, `1e238fa4`, `9cbc13ff`, `b3b9fefa`, `e0f999f4`; план: [stationary launch gear and restarts](2026-11-15-stationary-launch-gear-and-restarts.md)): перезапуск 3 с форы `BalanceMonitor`, выбор стартовой передачи на месте в `LaunchPhase::hold`, приёмочный тест рестарта на 5 % (`33T`) и 10 % (`39T`), `check_change.sh main` (`194/194`, `397 с`) и `rider_launch_rides_*` (`3/3`, `361.95 с`) пройдены.
2. **Предел нажима в обычной езде на подъёмах** (П8, П13, пункт 17). **Выполнено на ветке `queue-2026-11-15`** (`c752736f`, `67ca4b30`, `ceb4e816`, `272f49b2`, `732a3887`, `eff7bb45`; план: [seated climb effort cap](2026-11-15-seated-climb-effort-cap.md)): обязательные поля `climb_effort_cap_grades`, `climb_effort_cap_grade_nm`, `climb_effort_rise_speed_mps`, `climb_effort_full_speed_mps` в `[seated_climb]`, калибровка `N4_lin_sc35` / `C1_lin_nosurf` / `A1_lin_sc35` и узлы `0.22`/`0.25` в `[rider_launch]` для `normal` и `careful`; `check_change.sh main` (`194/194`, `321 с`, `0` находок `cppcheck`/`frontends`/`clang-tidy`) и целевой набор `slow` (`9/9`, `136.60 с`) пройдены на `eff7bb45`.
3. **Короткое торможение задним** в реакции на подъём колеса («Решено — не сделано», пункт 2 в)). **Закрыто без включения на ветке `queue-2026-11-15`** (`rear_brake_tap_enabled = false`, эмпирически отвергнуто по замерам `q2-probe/report.md`, план: [seated climb effort cap](2026-11-15-seated-climb-effort-cap.md)): с Вариантом 2 реакция `ease + lean` уже убирает отрывы после передачи управления и падения на `0–25 %`, тогда как `rear_brake` через §16 (`motor_braking = true`) мгновенно сбрасывает момент мотора (`~67.5 → 0 Н·м`), вызывая удары переднего колеса 994–1112 Н, срывы `grip.pull` и потерю опоры седла до 6.1 Н.
4. **Старт с угла педали круче −0,8 рад** (стена −0,9 рад, вероятно, была неустойчивостью удержания).
5. **Потери шин на `distributed`**: ровный старт до 20 км/ч за 7,3 с. **Выполнено на ветке `queue-2026-11-15`**
   (`5f23a658`):
   вязкий радиальный член `damping_ns_m·δ̇` (~616 Вт на разгоне, нарушение запрета 39) заменён
   гистерезисом `η·kδ·tanh(δ̇/ε)` в `native/src/tyre/station.cpp`, `native/src/numerics/density_fit.cpp` и
   `native/src/gpu/tire_kernel.hpp`; упругая энергия ½Σkᵢxᵢ² осталась независимой от скорости. В
   `HingeDensity` добавлены обязательные `loss_factor` (0,05) и `hysteresis_rate_eps_mps` (0,01 м/с);
   `damping_ns_m` оставлен ключом совместимости и отклоняется при любом ненулевом значении.
   Двойной учёт качения снят `[resistance] crr = 0.0` в `viewer_physics_distributed.toml`. Замер
   `probe_q4_q5` (0 % асфальт, 20 с): 7,32 с → 3,73 с до 20 км/ч, `step_tire_radial_loss_j` 558 Дж.
   Переписаны эталоны `native_distributed_tire`, `distributed_tire_reference`, `native_numeric_completion`,
   `native_pneumatic_tire` (initial_state), `native_research_environment`, `native_research_recording`,
   фикстуры `replay/{legacy_v1,legacy_v2,historical_v2}`, `ft2/*/assets/resolved_config.json` (+`assets.json`,
   `provenance.json`) и порог `sweep_world_differences` (дельта вилки 50 → 10 Н после снятия вязкости).
6. **Восстановление шины `pneumatic`**: `savage_pneumatic.npz` даёт невязку загрузки 0,083 при пороге 0,05 —
   материал и гистерезис шины после `b6c42f71` не сохраняются. **Выполнено на ветке `queue-2026-11-15`**
   (`d5abc831`, правки ревизии `7f193e55`): `SetupOwner::settle` сохраняет `settled_state_file` до
   `finalize_initial_state` — файл несёт материал каркаса, на котором измерялась невязка, а не состояние
   после опережающего `compute_into`; `capture_initial_state()` отдаёт финализированный контекст, чтобы
   перенос сохранял `qvel` старта с хода (`initial_speed_mps > 0`), и стражи завершения восстановлены.
   Нормализация до той формы, что ставит `install_material` (`d->time=0`, рестарт часов привода и
   контактов райдера), сохраняет побайтовую идентичность подачи из кэша. `savage_pneumatic.npz` пересобран
   `bike-settle` на текущем исходном отпечатке (steps=3536, residual=0,0492147); регрессии в
   `reference.equilibrium_state_file`: `test_savage_pneumatic_fixture_restores` со стражем тождества
   сборки, свежий круг settle→capture→restore и страж `qvel` переноса; `check_change.sh 85c0c18b` —
   194/194, cppcheck/frontends/tidy чисто.
7. **`model_valid` в `one_second_pair`** после правил тяги опоры (`9525ce1f`, `abaff36d`). **Выполнено на ветке
   `queue-2026-11-15`** (`51702136`; решение пользователя — вариант A разбора
   `/Volumes/SSD_1TB_M2/work/tmp/2026-11-15-post-merge-queue/q6-q7-q11-report.md`): ворота
   `one_second_pair_*` (1,0 с на `research_flat`, `articulated_effort`) получают явный
   `human_torque_nm = 0` через `motor_control(0., 0.)` на каждой границе — ноги лежат на педалях
   вместо мэша 60 Н·м с места; обе вороты дополнены проверками `model_valid && rider_support_ok`
   по итогам всех 800 шагов; эталоны darwin-arm64 перезаписаны (`one_second_pair_*.{bin,json}`,
   все 101 вхождение `model_valid` = `true`). Без правки `motor_control(0.)` с `nullopt` с места
   гнал мощность бедра >375 Вт (t = 0,31875 с) и тянул седло до −516,9 Н. linux-близнецы — в
   пункт 9.
8. **Читатели старых форматов** в `native/src/replay/` (`drop_removed_battery`, `inert_rider`) — убрать, если
   задача 2 покажет, что они мертвы. **Выполнено на ветке `queue-2026-11-15`** (`3d52cd06`, `b99382d3`):
   проверка показала, что мёртв только `drop_removed_battery` — любая запись с `drive.battery` старше
   удаления модели батареи (`54ee44f8`, 03.11) и потому несёт и более поздние удалённые поля привода
   (`bearing_c_nms_rad`, `be37a151`/`ae4b1671`, 05.11), так что `construct_config` отказывает её в любом
   случае; стрип убран вместе с осиротевшим `battery_inert`. `inert_rider` жив: оракул пишет все 15 полей
   `RiderSpecs` в `resolved_config.rider` каждого эталона `research_*`, а `verify_configuration` сравнивает
   запись с текущей пяти полевой схемой — без нормализации `reference.native_research_recording` красный.
   Оставлен как проверка инертности удалённых полей и сведение сериализаций, а не читатель отказанного
   формата.
9. **Эталоны linux-aarch64** — перезапись в контейнере (`tools/linux/container.sh`) по просьбе пользователя.
   От задачи 5 (гистерезис `distributed`, `5f23a658`) подлежат перезаписи девять файлов на шести узлах
   (darwin-близнецы перезаписаны, linux остались вязкими; все пути под `tests/fixtures/golden/linux-aarch64/`):
   `distributed_tire_reference/scalar_station_formulas_equal_the_vectorised_ones_bitwise.bin`
   (`reference.distributed_tire_reference`); `native_distributed_tire/distributed_qfrc_bitwise.bin` и
   `native_distributed_tire/env_request/initial_state.json` (`reference.native_distributed_tire`);
   `control_clock/env_request/initial_state.json` (`reference.control_clock`);
   `pneumatic_tire/env_fast/initial_state.json` (`reference.native_pneumatic_tire`);
   `native_research_environment/reset_seed_generation_snapshot_and_facade_ownership.{bin,json}`
   (`reference.native_research_environment`);
   `native_research_recording/shared_export_native_rows_and_complete_provenance.{bin,json}`
   (`reference.native_research_recording`). От задачи 7 (`51702136`) на том же узле
   `reference.native_research_environment` ещё четыре файла:
   `native_research_environment/one_second_pair_with_one_e_minus_nine_gate.{bin,json}` и
   `native_research_environment/one_second_pair_complete_episode_gate.{bin,json}` — смена
   сценария ворот (`human_torque_nm = 0` вместо `None`) меняет всю траекторию. Плюс узел
   `reference.native_numeric_completion`:
   `native_numeric_completion/tire_channel_parity_across_backends.bin`, где входной seed уже пропатчен
   в `5f23a658`, а 143 выхода `distributed.*` остаются вязкими.
10. **Неровные трассы**: остановка `rough_uphill_extreme` на 13,1 с, «стиральная доска» на savage (позже, решение
    пользователя 2026-11-10).
11. **Отскок передней вилки при резкой остановке на подъёме** («Решено — не сделано», пункт 19, `1303e344`):
    разобраться, почему при 100 % торможении на подъёме 10 % вилка после остановки отстреливает и разгружает
    переднее колесо ниже 50 Н на 53–75 мс, и всё ли верно с демпфированием отбоя и клевком.
