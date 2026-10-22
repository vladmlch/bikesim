# Spatial 3D Native — CLI, viewer, отчеты и доставка Implementation Plan

> **Для исполнителей:** использовать superpowers:subagent-driven-development (рекомендуется) или superpowers:executing-plans для последовательного выполнения задач; отмечать шаги через `- [ ]`. При отсутствии этих skills сохраняется цикл с конкретной регрессией и независимым ревью.

**Goal:** Сделать пространственный режим доступным в действующих native executables, отображении, отчётах и устанавливаемом продукте macOS/Linux.

**Architecture:** Используется существующий граф самостоятельного приложения. CLI создаёт один `SetupRequest`/research runtime; viewer получает принятый `RenderFrame` и владеет отдельной кинематической копией. Нативные графики и пакетные процессы используют versioned recording catalog, без возвращения Python в runtime или инструменты.

**Tech Stack:** C++23, корневой CMake, существующий CLI parser, отрисовка GLFW/MuJoCo, нативные SVG/HTML/PNG отчёты, DuckDB reader, нативный запуск процессов, `bt`/CTest.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`, `2026-10-22-spatial-3d-native-master.md`, `2026-10-22-spatial-3d-native-contracts.md`; контракты observation/recording/replay фазы05. Историческая спецификация связывает новый план с исходными визуальными требованиями.

## Общие ограничения

- Задачи planned; приложения/сборки/тесты при написании плана не запускались.
- Существующие `bike-ride`, `bike-research`, `bike-replay`, `bike-recording`, `bike-research-sweep` расширяются, не дублируются.
- Planar defaults, keys и жизненный цикл записи сохраняются. C/1/2 меняют камеру, R означает reset; старый текст «R export» недействителен.
- Рендер не вызывает `mj_step`, полный `mj_forward`, force writers, измерение сенсоров или policy; он читает принятое состояние и выполняет разрешённые кинематические обновления.
- GUI остаётся в главном потоке, headless не требует display или инициализации GLFW. Поддерживаются macOS/Linux; Windows не добавляется.
- Каждый новый канал имеет объявленные units/frame/phase/availability. Отсутствующее измерение не заменяется нулём ради красивого графика.
- Имена библиотек или входных файлов не являются физической identity; воспроизводимость опирается на hashes сохранённого содержимого.
- Конечный продукт — корневой bindings-OFF граф. Переходный planar Python oracle принадлежит миграции, не новому пространственному фронтенду.

## Что уже существует в снимке

`native/cmake/RecordingApplication.cmake:178–197` создаёт восемь CLI executable из одного main; analyst и research-sweep созданы ниже. `viewer/render_owner.cpp:68–109` уже проверяет model hash, поколения и конечность integration state, затем выполняет только кинематические функции. Эту границу нужно сохранить.

Нынешний `Camera::update(mjvCamera&,double x,double z)` ставит y=0 (`viewer/camera.cpp:17–21`). Название «3D Isometric Chase» относится к ракурсу, не означает spatial tracking. `runtime/presentation.cpp:125–130` берет planar scalar root indices. Native plots в `viz/ride.cpp` часто требуют `x_m`; в spatial такой channel должен иметь ясный world-coordinate смысл, а progress/path length — отдельные поля.

## Карта исполнения

| Задача | Зависимости | Результат | Spatial milestone / upstream prerequisite |
|---|---|---|---|
| S45 | S19, S36, S39 | Пространственные args→native request | M5; upstream G2/G3/G5 |
| S46 | S36, S40, S45 | Пространственная поза в камере/HUD/рендере | M5; upstream G5-gui |
| S47 | S40–S43, S45 | Нативные отчёты и воспроизводимый batch | M5; upstream G4-recording |
| S48 | S12, S43, S45–S47, владелец C++/Linux packaging | Установленный продукт без Python | M5; upstream G5/G6 |
| S49 | S44–S48 | Документы, примеры, покрытие возможностей | Передача M5 в S54/M6 |

Все tests: `native/tests/spatial/<slug>.cpp`, CTest `spatial.<slug>`, target `bike_spatial_<slug>`. GUI/process scripts сохраняют эти имена CTest; target зависит от фактических executable, не от пустой подстановки.

G0–G6 обозначают baseline producer evidence C++-миграции по master; эта фаза не переопределяет их spatial-тестами. Выход S45–S49 — milestone M5 и данные для M6.

---

## S45 — Нативный CLI и единая цепочка разрешения конфигурации

**Files:** Modify `native/src/cli/{argument_contracts.inc,arguments.cpp,configuration.cpp,application.hpp,ride.cpp,research.cpp,replay.cpp,export.cpp}`, `native/src/research/configuration.cpp`; Create `native/tests/spatial/cli.cpp`; обновить help и документированный список опций в одном интеграционном изменении.

**Dependencies:** S19 topology, S36 startup, S39 schema dispatch; upstream G2/G3/G5 и spatial M3/M4 fixtures. **Consumes:** S02 `bike::spatial::Config`, native `terrain::Grid3d` request и действующий `bike::cli::Arguments`. **Produces:** `--spatial planar|3d`, `--terrain3d PATH`, native resolved request; `--threads N` добавляется только S52 и до того неизвестный flag отвергается, а не принимается впустую.

- [ ] Расширить канонический `argument_contracts.inc`, без отдельных локальных parsers. Проверить допустимый/недопустимый enum, повтор flag, пропущенное значение, NaN и неизвестный аргумент. Отсутствующий `--spatial` не меняет результат TOML; пустая конфигурация остаётся planar.
- [ ] Определить приоритет: конфигурация → явный CLI override. Конфликт явно выбранного planar с terrain3d отклоняется. `--track` legacy profile и `--terrain3d` не задаются вместе; extrusion передаётся отдельным явным request S02, без угадывания по файлу.
- [ ] Передать spatial config в один существующий `SetupRequest`/`ResearchRequest`. Неподдержанные physics/tyre/root combinations отклонять до загрузки ресурсов, окна или создания output; ошибка spatial не включает planar fallback.
- [ ] В research выбрать observation V2/ABI2 по resolved mode. Headless/viewer создают одинаковые исходные native model/state; отличаются только темп и отображение.
- [ ] Сохранить явное включение записи и смысл reset/close; `--out` задаёт только назначение. Replay выбирает inspect/checked существующим явным интерфейсом, версию контейнера определяет reader.
- [ ] Проверить одинаковые effective config hashes для эквивалентных TOML+CLI и одинаковое число физических шагов при неограничивающих wall limits; неверный input не инициализирует filesystem/output или GUI.
- [ ] Выполнить адресную native-проверку, ревью help snapshot и commit `feat(spatial): native CLI mode resolution`.

```cpp
const std::array<std::string_view,2> args{"--spatial","sideways"};
bt::invalid([&] { (void)bike::cli::parse_arguments("bike-ride", args); });
const std::array<std::string_view,2> good{"--spatial","3d"};
const auto parsed = bike::cli::parse_arguments("bike-ride", good);
bt::require(parsed.text("spatial") == "3d", "spatial option uses canonical parser");
```

**Acceptance:** новые args завершаются нативными контрактами S02; прежний разбор вызовов и planar wire неизменны. **Failure/rollback:** ошибка валидации не генерирует terrain, не запускает процесс, не открывает окно и не создаёт output. Принятая, но игнорируемая опция не считается поддержанной.

## S46 — Пространственная поза, следящая камера и читаемый HUD

**Files:** Modify `native/src/runtime/presentation.{hpp,cpp}`, `native/src/viewer/{render_owner.hpp,render_owner.cpp,camera.cpp,hud.cpp,input.cpp,ride.cpp,research.cpp,replay.cpp}`; Create `native/tests/spatial/viewer_pose.cpp`, `native/tests/spatial/viewer_smoke.cmake`.

**Dependencies:** free-root startup S36, spatial truth S40, CLI S45; upstream G5-gui для реального viewer. **Consumes:** `bike::spatial::BodyPose{position_world_m,quat_world_from_body_wxyz,linear_velocity_world_mps,angular_velocity_world_rad_s}` и принятый RenderFrame. **Produces:** пространственное расширение отображения и `Camera::update_spatial(mjvCamera&,const std::array<double,3>& position_world_m,std::optional<double> heading_rad)`; planar `update(x,z)` сохраняется.

- [ ] Добавить position xyz, orientation/heading и диагностику roll/steer/contact в owned PresentationState. Значения вычисляются по topology после commit, не угадываются через `qpos[2]`.
- [ ] Реализовать цель chase `[x+0.20*cos(heading), y+0.20*sin(heading), z+0.30]`; азимут следует непрерывно развёрнутому heading со смещением 135°. Side mode сохраняет выбранную точку, не возвращает велосипед к y=0.
- [ ] Ввести сглаживание только отображения со сбросом при новой generation; quaternion интерполируется по кратчайшей дуге и нормируется только в render copy. Физические bytes не меняются ради камеры. При `attitude_valid=false` HUD показывает N/A, optional heading пустой. Последний azimuth/preset камеры остаётся её view state, не измерением heading.
- [ ] Сохранить model-hash/finite/generation checks `RenderOwner::apply`; неверный spatial vector отклоняется до изменения состояния. Draw/update не вызывают force/sensor/step; renderer остаётся кинематической копией.
- [ ] HUD показывает roll, угол/момент руля, lateral slip, контакт/validity и startup hold до снятия. Цвета и символы не скрывают невалидный запуск; units/phase описаны в help и документации.
- [ ] Проверить y≠0, переход heading через ±π, capsize, reset и неверную model identity. На двух GUI hosts выполнить поворот/крен с terrain и сравнить simulation rows с headless-заездом.
- [ ] Отдельно выполнить проверку данных и GUI smoke; commit `feat(spatial): native pose-aware viewer`.

```cpp
bike::viewer::Camera tracker;
mjvCamera camera{};
tracker.set_mode(bike::viewer::CameraMode::chase);
tracker.update_spatial(camera, {2., 3., 4.}, 0.);
bt::require(camera.lookat[1] == 3., "chase follows lateral world position");
bt::require(camera.lookat[2] == 4.30, "chase offset is presentation-only");
```

**Acceptance:** отображение следует действительной 3D-позе и не меняет физику. **Failure/rollback:** ошибочный frame сохраняет последнее валидное отображение; графическая ошибка закрывает render resources до context. Проверка данных не заменяет реальный GUI на macOS/Linux.

## S47 — Нативные пространственные отчёты, графики и пакетные процессы

**Files:** Create `native/src/viz/spatial.{hpp,cpp}`; Modify `native/src/viz/{plot.hpp,ride.cpp}`, `native/src/recording/{analysis.cpp,report.cpp,query.cpp}`, `native/src/batch/{plan.cpp,summary.cpp,sweep.cpp}`, `native/src/cli/{export.cpp,batch.cpp}`; Test `native/tests/spatial/reports_batch.cpp`.

**Dependencies:** truth S40, catalog S41, recording/replay S42/S43, вызовы S45. **Consumes:** schema2 columns с объявленными validity/frame; существующие `bike::viz::Channels`, `Report`, `Plot` и native process batch. **Produces:** `bike::viz::Report spatial_attitude_plot(const Channels&)`, пространственные отчёты и детерминированные batch job identity/summary.

- [ ] В `spatial_attitude_plot` требовать `time_s`, `spatial.attitude_valid`, `spatial.roll_rad`, `spatial.steer_angle_rad`, равные длины и конечные timestamps; отсутствующий обязательный channel даёт `std::invalid_argument`. Query adapter переводит BOOLEAN mask в 0/1 существующих Channels. Невалидные Euler slots пропускаются, серии разбиваются на валидные участки с явным N/A; линии через gap и прошлые углы запрещены. Steer доступен независимо.
- [ ] Построить roll/steer/time, lateral-force/slip/load, пути колёс x/y, оси IMU и энергию/работу руля. Легенда указывает frame; world_x, progress и lateral displacement остаются разными каналами.
- [ ] Использовать нативный SVG/HTML/PNG backend и установленные resources. Автономный отчёт не содержит CDN; графики читают принятую запись, не создают runtime.
- [ ] Включить Grid3d/config/калибровку шины/sensor schema/seed/threads в batch job key и provenance результата. Существующий process pool параллелит независимые эпизоды; это отдельный параметр от пула внутри шага S52.
- [ ] Проверить, что число работников 1/2/4 меняет только порядок завершения и реальное время. Summary сортируется по стабильной job identity, physics trace каждой работы прежний. Неполная, crashed или invalid работа не включается в успешные показатели скорости.
- [ ] Не заполнять нулями отсутствующие поля старой записи: planar reports прежние, spatial-only report на schema1 даёт capability diagnostic. CSV/JSON сохраняют masks, shapes и manifest единиц.
- [ ] Выполнить адресную проверку, визуальное ревью автономного artifact и commit `feat(spatial): native reports and batch metadata`.

```cpp
bt::invalid([] { (void)bike::viz::spatial_attitude_plot({}); });
bike::viz::Channels bad{{"time_s",std::vector<double>{0.,1.}},
    {"spatial.attitude_valid",std::vector<double>{1.,1.}},
    {"spatial.roll_rad",std::vector<double>{0.}},
    {"spatial.steer_angle_rad",std::vector<double>{0.,0.}}};
bt::invalid([&] { (void)bike::viz::spatial_attitude_plot(bad); });
```

**Acceptance:** графики показывают действительные пространственные данные; одна job identity даёт тот же эпизод независимо от расписания процессов. **Failure/rollback:** ошибка экспорта не уничтожает запись; неуспешный дочерний процесс batch сохраняет диагностику и класс завершения, остальные независимые работы продолжаются по существующему lifecycle.

## S48 — Установка и переносимость без Python

**Files:** Modify `CMakeLists.txt`, `native/cmake/{RecordingApplication.cmake,RealtimeMeasurement.cmake}`, `native/tests/install/install_smoke.cmake`, `native/tests/package/package_smoke.cmake`, `docs/development/{linux.md,macos.md,troubleshooting.md}`; Create `native/tests/spatial/installed_product.cmake`, `examples/spatial/cross_slope.toml`, `examples/spatial/cross_slope.terrain3d`; добавить assets примеров/fixtures в список установки. Маленький бинарный пример создаётся native generator из принятого аналитического случая S12, с manifest/digest.

**Dependencies:** S12, S43, S45–S47 и принятые dependency/install/loader gates C++/Linux completion; upstream G1/G4/G5/G6. **Consumes:** корневой граф executable и закреплённые native providers. **Produces:** установленные пространственные возможности на квалифицированных macOS-arm64/Linux-aarch64, headless и GUI в выбранных конфигурациях.

- [ ] Зарегистрировать новые TUs в корневом графе, source checks и перечне установки; встроенный catalog2, `policy_v2.h`, примеры и terrain specifications доступны без source tree. Не вводить абсолютные пути разработчика или автозагрузку при запуске runtime.
- [ ] Выполнить headless Release install с bindings OFF; поиск зависимостей следует владельцу миграции, spatial не создаёт альтернативный поиск MuJoCo/OpenBLAS/DuckDB.
- [ ] Из перемещённого prefix вне repository выполнить короткие валидные spatial ride/research/record-inspect/checked replay команды. Source checkout и Python/uv недоступны runtime; зависимости PATH от личных scripts исключены.
- [ ] Headless smoke без display подтверждает отсутствие инициализации GLFW; GUI package отдельно выполняет реальный viewer smoke на поддержанном host. Разбор данных не считается проверкой отрисовки.
- [ ] Проверить resources/hashes и identity загруженных зависимостей; Linux rpaths и macOS load names/signatures наследуют квалифицированную инфраструктуру. Частичная установка targets не объявляется полным продуктом.
- [ ] Повторить минимальный planar вызов и schema1 reader на установленном продукте; несовместимая recording identity даёт диагностированный отказ, inspection сохраняется.
- [ ] Записать команды/build identity каждого host и проверенные capabilities; commit `build(spatial): install native spatial product`.

В process test использовать CMake `execute_process` с argv, а не склеенную shell-строку. Обязательные проверки:

```cmake
if(NOT DEFINED SPATIAL_INSTALL_PREFIX)
  message(FATAL_ERROR "SPATIAL_INSTALL_PREFIX is required")
endif()
if(NOT EXISTS "${SPATIAL_INSTALL_PREFIX}/bin/bike-ride")
  message(FATAL_ERROR "installed bike-ride is missing")
endif()
```

**Acceptance:** установленные binaries работают вне checkout без Python на обеих квалифицированных платформах. **Failure/rollback:** отсутствующая зависимость/resource блокирует платформу; version lock или numerical flags не меняются ради успешного smoke без ревью миграции. Готовность пакета и квалификация платформы отмечаются отдельно.

## S49 — Пользовательские сценарии, покрытие и передача

**Files:** Create `docs/spatial-3d.md`, `docs/superpowers/audits/2026-10-22-spatial-3d-application-evidence.md`, `native/tests/spatial/documented_examples.cpp`; Modify `docs/{RIDE.md,TESTING.md,perf.md}`, `CONTEXT.md`, spatial example inventory и S44 registry. Проверка документов не добавляет новый production API.

**Dependencies:** S44–S48; финальные числа производительности появятся после дополнения отчёта S50 в S54. **Consumes:** реализованные CLI/options, принятые configuration files, проверенные observations/records и платформенные доказательства. **Produces:** исполнимые пользовательские сценарии, достоверная таблица возможностей и связь task→requirement→gate, без объявления ещё не измеренного RTF.

- [ ] Документировать минимальные сценарии: прежний planar, constrained test, прямой free-root заезд, поперечный уклон, noise2 research, запись, inspect, checked replay, native report и process sweep. Команды ссылаются на реально установленный example, без выдуманных flags.
- [ ] Для примера указать physics mode, root, tyre tier, terrain/seed, dt, пределы баланса, sensor schema, явное включение записи и ожидаемый terminal classification. Один rollout не обещает устойчивость на любой скорости/уклоне.
- [ ] Обновить клавиши и термины камеры по S46; объяснить inspection/checked replay и диагностическую скорость/квалифицированную физику. Основной ride HUD не перегружать ненужными provenance деталями.
- [ ] В native test читать example argv из manifest S44, использовать действительные `bike::cli::parse_arguments` и `resolve_ride`/`resolve_research`; сверять ожидаемые mode/schema/root и положительный duration с resolved request. Получение конфигурации не вызывает ride command, GUI или stepping; исполнение покрывает S48.
- [ ] Сверить примеры с CLI parser, установленными resources и registry: документация сама по себе не делает capability поддержанной. Указать ожидающие migration dependencies и владельцев их доказательств.
- [ ] Передать S54 точные SHA/dirty payload, identities сборки/runtime каждой платформы, адресные и финальные логи, hashes валидных физических fixtures, незакрытые проверки скорости. Исторические планы сохраняются, новый master указан основным.
- [ ] Commit `docs(spatial): native workflows and capability coverage` после ревью команд и таблиц.

```cpp
const std::array<std::string_view,4> example{
    "--spatial","3d","--terrain3d","examples/spatial/cross_slope.terrain3d"};
const auto arguments = bike::cli::parse_arguments("bike-ride", example);
bt::require(arguments.text("spatial") == "3d", "example selects spatial mode");
bt::require(arguments.text("terrain3d") == example[3], "example asset path preserved");
```

**Acceptance:** каждая документированная команда соответствует parser и доступному artifact; неподтверждённая capability остаётся pending. **Failure/rollback:** ошибочный example исправляется с регрессией, не удаляется из перечня; незавершённые S50/S54 не скрываются словом «готово».

Путь в примере кода создаёт и устанавливает S48 на основе квалифицированного случая S12; parser assertion не подтверждает наличие файла. Отдельная проверка S04/S44 открывает файл и проверяет digest, а S48 использует установленный asset вне checkout.

## Матрица доказательств пользовательских сценариев

| Сценарий | Зафиксированный вход | Проверяемый результат | Owner |
|---|---|---|---|
| Совместимость planar | Прежний default example / schema1 | Прежние resolved mode, channels и reader | S45/S48 |
| Constrained control | Явный constrained topology request | 3D-пути данных доступны, root действительно ограничен | S45/S49 |
| Прямой free-root заезд | Принятый fast fixture, seed, duration | Валидный заезд без scored hold и с достоверной причиной завершения | S36/S48 |
| Поперечный уклон / поворот | Установленные grid и steering profile | Камера следует боковому движению, contact axes согласованы | S46/S48 |
| Исследование сенсоров | V2 mount/tape/ABI2 plugin | 11 каналов, masks, timestamps и последовательность управления | S39/S45 |
| Запись + inspect | Явно включённая запись / schema2 | Переносимые assets и просмотр без исходных input paths | S42/S48 |
| Checked replay | Подходящий contract4 / точная identity | Полное сравнение либо объяснённая несовместимость | S43/S48 |
| Автономный отчёт | Spatial container вне source tree | SVG/HTML с пространственными осями и обозначенными пропусками | S47/S48 |
| Process sweep | Неизменяемый job manifest | Стабильные job keys, сортированный summary, сохранённые ошибки | S47 |

Таблица содержит фактическую команду, выходной код, artifact hash и platform identity. Наличие строки не является pass. Сценарии с одинаковыми входами используют уже полученные достаточные доказательства; полный набор не повторяется ради количества запусков.

## Контроль границ фронтенда

| Вход | Разрешённое действие | Запрещённое изменение |
|---|---|---|
| Перемещение/переключение камеры | Камера и render replica | qpos/qvel/tyre state живого runtime |
| Обновление HUD | Чтение принятых полей | Повторный расчёт сил/сенсоров |
| Пауза/темп | Отсрочка следующего advance | Пропуск physics ticks или RNG draws |
| Reset | Явный lifecycle эпизода, новая generation | Скрытая смена container/schema |
| Остановка записи/экспорт | Финальная публикация сохранённых данных | Повторный rollout ради заполнения записи |
| Просмотр старой записи | Воспроизведение записанного состояния | «Verified» без checked replay |

## Будущие команды адресной проверки

```bash
cmake --build build/native-release --target bike_spatial_cli bike_spatial_reports_batch
ctest --test-dir build/native-release -R '^spatial\.(cli|reports_batch)$' --no-tests=error --output-on-failure
cmake --build build/gui --target bike-ride bike-research bike-replay
ctest --test-dir build/gui -R '^spatial\.viewer_smoke$' --no-tests=error --output-on-failure
```

GUI-тест требует настоящего display на квалифицированном host; отсутствие display блокирует среду, не считается успешным skipped test. В текущем задании команды не выполнялись.

## Передача на квалификацию

S45–S49 не вводят новые уравнения tyre/balance и не ослабляют их проверки. Они передают S54 установленный продукт с достоверной таблицей возможностей и конфигурациями для дополнения раннего отчёта S50. Измеренная скорость и принятая физика должны относиться к одной code/config identity; screenshots этого сами по себе не доказывают.
