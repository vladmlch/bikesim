# Spatial 3D Native — 06. CLI, viewer, отчёты и доставка (W-фаза: W45–W49)

> **Для исполнителей:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development (рекомендуется) или superpowers:executing-plans. Все задачи — **one-shot write** против frozen headers W00 (`00-foundation.md` §W00.10: `viz/spatial.hpp`, `viewer/camera.hpp (+update_spatial)`). Запрещены configure/build/compile/ctest/запуск/install/GUI. Проверка и acceptance — `09-verification.md` V08.

**Goal:** Написать spatial-расширение существующих native executables, viewer pose/camera/HUD, отчёты/batch, install-листы и документацию — целиком, без запуска.

**Architecture:** Существующий граф приложения: CLI создаёт один `SetupRequest`/research runtime; viewer получает принятый `RenderFrame` и владеет кинематической копией. Native SVG/HTML/PNG отчёты читают versioned recording catalog.

**Tech Stack:** C++23, root CMake (`native/cmake/RecordingApplication.cmake:178–197` создаёт восемь CLI executable из одного main), существующий CLI parser (`bike::cli::parse_arguments`, `argument_contracts.inc`), GLFW/MuJoCo render, DuckDB reader, process batch.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`; contracts C1, C2, C8; фаза 05 для схем.

## Общие ограничения

- `bike-ride`, `bike-research`, `bike-replay`, `bike-recording`, `bike-research-sweep` расширяются, не дублируются.
- Planar defaults/keys/lifecycle сохраняются. C/1/2 — камера, R — reset.
- Рендер не вызывает `mj_step`, полный `mj_forward`, writers, sensors или policy: `viewer/render_owner.cpp:68–109` проверяет model hash/generation/finite и выполняет только кинематику — границу сохранить.
- GUI в главном потоке; headless без display/GLFW. macOS/Linux.
- Каждый новый канал имеет units/frame/phase/availability; отсутствующее измерение не заменяется нулём.
- Текущий `Camera::update(mjvCamera&,double x,double z)` ставит y=0 (`viewer/camera.cpp:17–21`); `runtime/presentation.cpp:125–130` берёт planar scalar indices — заменяется в W46.

## Карта исполнения

| Задача | Dependencies (W) | Результат | V |
|---|---|---|---|
| W45 | W19, W36, W39 | Spatial args → native request | V08 |
| W46 | W36, W40, W45 | Pose в камере/HUD/рендере | V08 (GUI smoke на display) |
| W47 | W40–W43, W45 | Native отчёты и batch metadata | V08 |
| W48 | W12, W43, W45–W47 | Install-листы, examples, process smoke scripts | V08 |
| W49 | W44–W48 | Документы, examples manifest, coverage test | V08 |

Тесты: `native/tests/spatial/<slug>.cpp`, `bike_add_spatial_test(<slug> SOURCES … LIBRARIES bike_cli …)`.

---

## W45 — Native CLI и единая цепочка разрешения конфигурации (← S45)

**Dependencies:** W19, W36, W39. **Create:** `native/tests/spatial/cli.cpp`. **Modify:** `native/src/cli/{argument_contracts.inc,arguments.cpp,configuration.cpp,application.hpp,ride.cpp,research.cpp,replay.cpp,export.cpp}`, `native/src/research/configuration.cpp`; help-текст и документированный список опций в том же изменении.

- [ ] `argument_contracts.inc`: `--spatial <planar|3d>` (enum, одно значение, повтор → ошибка), `--terrain3d <PATH>` (путь к `.terrain3d` или genspec `.toml`; `TerrainRequestKind` по расширению `.terrain3d`→`grid_asset`, `.toml`→`genspec`); `--threads` **не** добавляется (W52) — неизвестный flag отвергается существующим parser.
- [ ] `configuration.cpp`: приоритет конфигурация → явный CLI override; `--spatial planar` + `--terrain3d` → `std::invalid_argument("--terrain3d requires --spatial 3d")`; `--track` (legacy profile) + `--terrain3d` вместе → ошибка; extrusion выбирается только явным `[spatial_3d.terrain] kind="profile_extrusion"` в TOML, не угадыванием по файлу; отсутствие `--spatial` не меняет результат TOML; пустая конфигурация остаётся planar.
- [ ] `ride.cpp/research.cpp`: spatial config → единый `SetupRequest`/`ResearchRequest` (поля W05/W11/W31); `check_spatial_capability` до загрузки assets/окна/output; rejection без planar fallback. Research: `sensor schema = mode==spatial3d ? 2 : 1`, ABI соответственно (W39). Headless/viewer — одинаковые model/state; различие только pacing/display.
- [ ] `replay.cpp`: inspect/checked через существующий явный интерфейс; версия контейнера определяется reader (W42/W43). `export.cpp`: spatial channels доступны через `spatial.*`. `--out` — только назначение.
- [ ] `cli.cpp` тест: `{"--spatial","sideways"}` → `bt::invalid`; `{"--spatial","3d"}` → `parsed.text("spatial")=="3d"`; повтор flag, пропущенное значение, NaN в числовом аргументе, неизвестный `--threads` → `bt::invalid`; эквивалентные TOML+CLI → одинаковый `effective_config_sha256`; неверный input не создаёт файлов (проверка temp dir пуст) и не инициализирует GUI (флаг в application).
- [ ] CMake: `bike_add_spatial_test(cli SOURCES cli.cpp LIBRARIES bike_cli bike_configuration bike_spatial)`.
- [ ] Коммит `feat(spatial): W45 native CLI mode resolution`, `Verification: pending V08`.

```cpp
const std::array<std::string_view,2> args{"--spatial","sideways"};
bt::invalid([&] { (void)bike::cli::parse_arguments("bike-ride", args); });
const std::array<std::string_view,2> good{"--spatial","3d"};
const auto parsed = bike::cli::parse_arguments("bike-ride", good);
bt::require(parsed.text("spatial") == "3d", "spatial option uses canonical parser");
```

## W46 — Пространственная поза, следящая камера и HUD (← S46)

**Dependencies:** W36, W40, W45. **Create:** `native/tests/spatial/viewer_pose.cpp`, `native/tests/spatial/viewer_smoke.cmake` (GUI smoke script, label `gui`: запускает `bike-ride --spatial 3d --terrain3d <example> --duration 2` с display; `if(NOT DEFINED ENV{DISPLAY} AND NOT APPLE) message(FATAL_ERROR "viewer smoke requires a real display")`). **Modify:** `native/src/runtime/presentation.{hpp,cpp}`, `native/src/viewer/{render_owner.hpp,render_owner.cpp,camera.cpp,hud.cpp,input.cpp,ride.cpp,research.cpp,replay.cpp}`.

**Interfaces:** `Camera::update_spatial(mjvCamera&, const std::array<double,3>&, std::optional<double> heading_rad)` (W00.10); `PresentationState` + spatial поля.

- [ ] `presentation.hpp`: `struct SpatialPresentation { std::array<double,3> position_world_m; std::array<double,4> quat_wxyz; std::optional<double> heading_rad; std::optional<double> roll_rad; double steer_angle_rad, steer_torque_nm; std::array<double,2> lateral_slip_mps; std::array<bool,2> contact_valid; bool attitude_valid; bool hold_active; std::uint64_t generation; }` как `std::optional` поле `PresentationState`; заполняется из `SpatialTruth` (W40) после commit, не из `qpos[2]`.
- [ ] `camera.cpp`: `update_spatial`: chase target `lookat = {x+0.20*cos(h), y+0.20*sin(h), z+0.30}` при heading, иначе `{x,y,z+0.30}`; azimuth = непрерывно развёрнутый heading (unwrap через накопленный `prev_heading_`) + 135°; side mode сохраняет выбранную точку (не возвращает к y=0); planar `update(x,z)` без изменений.
- [ ] `render_owner.cpp`: display-only smoothing (экспоненциальное `α=0.2`) со сбросом при новой generation; quaternion slerp по кратчайшей дуге и нормировка только в render copy; `apply` сохраняет model-hash/finite/generation checks; невалидный spatial vector (NaN) → reject до изменения состояния.
- [ ] `hud.cpp`: строки `roll`, `steer angle/torque`, `lateral slip F/R`, `contact F/R (valid/—)`, `HOLD` до снятия; при `attitude_valid=false` → `N/A`; единицы в тексте.
- [ ] `input.cpp`: клавиши без изменений (C/1/2 камера, R reset).
- [ ] `viewer_pose.cpp`: `tracker.set_mode(chase); tracker.update_spatial(camera,{2,3,4},0.)` → `camera.lookat[1]==3.`, `camera.lookat[2]==4.30` (точное сравнение допустимо: `4.+.30` одинаково вычисляется; при сомнении `<=1e-12`); heading через ±π не даёт скачка azimuth > 1e-9; `attitude_valid=false` → heading `nullopt`; reset → generation увеличена, smoothing сброшен; неверная model identity → `RenderOwner::apply` reject и прежняя копия.
- [ ] CMake: `bike_add_spatial_test(viewer_pose SOURCES viewer_pose.cpp LIBRARIES bike_viewer bike_runtime bike_spatial)`; `add_test(NAME spatial.viewer_smoke COMMAND ${CMAKE_COMMAND} -DBIKE_RIDE=$<TARGET_FILE:bike-ride> -DEXAMPLE=… -P viewer_smoke.cmake)` с `LABELS "spatial;gui"`.
- [ ] Коммит `feat(spatial): W46 native pose-aware viewer`, `Verification: pending V08`.

```cpp
bike::viewer::Camera tracker; mjvCamera camera{};
tracker.set_mode(bike::viewer::CameraMode::chase);
tracker.update_spatial(camera, {2., 3., 4.}, 0.);
bt::require(camera.lookat[1] == 3., "chase follows lateral world position");
bt::require(std::abs(camera.lookat[2] - 4.30) <= 1e-12, "chase offset is presentation-only");
```

## W47 — Native пространственные отчёты, графики и batch (← S47)

**Dependencies:** W40–W43, W45. **Create:** `native/src/viz/spatial.cpp`, `native/tests/spatial/reports_batch.cpp`. **Modify:** `native/src/viz/{plot.hpp,ride.cpp}`, `native/src/recording/{analysis.cpp,report.cpp,query.cpp}`, `native/src/batch/{plan.cpp,summary.cpp,sweep.cpp}`, `native/src/cli/{export.cpp,batch.cpp}`.

**Interfaces:** W00.10 `spatial_attitude_plot`, `spatial_contact_plot`, `spatial_wheel_path_plot`, `spatial_imu_plot`, `spatial_energy_plot`.

- [ ] `spatial_attitude_plot(channels)`: обязательные `time_s`, `spatial.attitude_valid`, `spatial.roll_rad`, `spatial.steer_angle_rad`; равные длины; finite timestamps; отсутствие → `std::invalid_argument`; invalid slots (`attitude_valid==0`) разбивают серии на сегменты с подписью `N/A`; линии через gap запрещены; steer рисуется независимо. Остальные plots: contact (`lateral_slip`, `normal_load`, `contact_valid` F/R), wheel path (x/y обоих колёс — из `root_position` + геометрия; legend `world frame`), imu (6 осей V2, `valid_mask` маска), energy (`steer_work_j`, tyre loss, hold work).
- [ ] `query.cpp`: BOOLEAN → 0/1 в `Channels`; `spatial.*` → `channel_spatial`; schema1 recording при запросе spatial → `capability diagnostic` (`std::invalid_argument("recording schema 1 has no spatial channels")`), не нули.
- [ ] `report.cpp`: SVG/HTML/PNG через существующий backend; без CDN; читает принятую запись, не создаёт runtime.
- [ ] `batch/plan.cpp`: job key = sha256(config + terrain sha + tyre calibration sha + sensor schema + seed + threads); `summary.cpp` сортировка по job key; invalid/crashed jobs исключены из speed stats; `sweep.cpp` — process pool как есть.
- [ ] `export.cpp`: CSV/JSON сохраняют masks/shapes и manifest единиц.
- [ ] `reports_batch.cpp`: `spatial_attitude_plot({})` → `bt::invalid`; несовпадающие длины → `bt::invalid`; валидные channels с gap → `Report` содержит два сегмента и `N/A` маркер (проверка по структуре `Report`, не по пикселям); planar report на schema1 fixture прежний (byte compare с существующим fixture); spatial report на schema1 → `bt::invalid`; batch: один manifest при workers 1/2/4 → одинаковые job keys и summary (порядок завершения разный, summary сортирован); crashed child → запись с классом завершения, остальные jobs завершены.
- [ ] CMake: `bike_add_spatial_test(reports_batch SOURCES reports_batch.cpp LIBRARIES bike_viz bike_recording bike_batch bike_cli)`.
- [ ] Коммит `feat(spatial): W47 native reports and batch metadata`, `Verification: pending V08`.

```cpp
bt::invalid([] { (void)bike::viz::spatial_attitude_plot({}); });
bike::viz::Channels bad{{"time_s",std::vector<double>{0.,1.}}, {"spatial.attitude_valid",std::vector<double>{1.,1.}},
    {"spatial.roll_rad",std::vector<double>{0.}}, {"spatial.steer_angle_rad",std::vector<double>{0.,0.}}};
bt::invalid([&] { (void)bike::viz::spatial_attitude_plot(bad); });
```

## W48 — Install-листы и process smoke без Python (← S48)

**Dependencies:** W12, W43, W45–W47. **Create:** `native/tests/spatial/installed_product.cmake`, `examples/spatial/cross_slope.toml` (genspec: constant bank 5°, flat lead-in/out, 60 m), `examples/spatial/cross_slope.terrain3d.manifest.json` (`sha256:null`, `status:"pending_verification"` — бинарный asset генерируется в V08 из genspec native generator и коммитится после review). **Modify:** `CMakeLists.txt`, `native/cmake/{RecordingApplication.cmake,RealtimeMeasurement.cmake}`, `native/tests/install/install_smoke.cmake`, `native/tests/package/package_smoke.cmake`, `docs/development/{linux.md,macos.md,troubleshooting.md}`.

- [ ] Root CMake: новые TUs (`bike_spatial`, `bike_pneumatic3d`, terrain3d, viz/spatial, replay/spatial_recipe, golden/spatial_registry, runtime/spatial_*) в install-листе библиотек; `install(FILES native/resources/recording/catalog-v2.json …)`, `install(FILES native/include/bike/policy_v2.h …)`, `install(DIRECTORY examples/spatial …)`; без абсолютных developer paths; без автозагрузки при старте runtime.
- [ ] `RecordingApplication.cmake`: executables получают `bike_spatial`/`bike_pneumatic3d` в link; поиск зависимостей — через существующий owner (не альтернативный find MuJoCo/OpenBLAS/DuckDB).
- [ ] `installed_product.cmake`: `SPATIAL_INSTALL_PREFIX` обязателен; `bin/bike-ride` существует; `execute_process` с argv (не shell строка) из перемещённого prefix вне repo: `bike-ride --spatial 3d --terrain3d <prefix>/share/.../cross_slope.toml --duration 2 --headless`, `bike-research --spatial 3d …`, `bike-recording inspect <out>`, `bike-replay --checked <out>`; `PATH` без uv/python (`set(ENV{PATH} "/usr/bin:/bin")`), `set(ENV{PYTHONPATH} "")`; каждый `RESULT_VARIABLE` == 0; planar: `bike-ride --duration 1 --headless` и schema1 reader на existing fixture; несовместимая identity → ненулевой exit с диагностикой, inspection — 0.
- [ ] `install_smoke.cmake/package_smoke.cmake`: вызов `installed_product.cmake` с теми же переменными; GUI package — отдельный `viewer_smoke` (W46) на поддержанном host.
- [ ] Docs: Linux rpaths / macOS load names наследуют существующую инфраструктуру; раздел «spatial product» со статусом `pending V08`.
- [ ] CMake: `add_test(NAME spatial.installed_product COMMAND ${CMAKE_COMMAND} -DSPATIAL_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -P installed_product.cmake)` с `LABELS "spatial;package"`.
- [ ] Коммит `build(spatial): W48 install native spatial product`, `Verification: pending V08`.

```cmake
if(NOT DEFINED SPATIAL_INSTALL_PREFIX)
  message(FATAL_ERROR "SPATIAL_INSTALL_PREFIX is required")
endif()
if(NOT EXISTS "${SPATIAL_INSTALL_PREFIX}/bin/bike-ride")
  message(FATAL_ERROR "installed bike-ride is missing")
endif()
```

## W49 — Пользовательские сценарии, покрытие и документация (← S49)

**Dependencies:** W44–W48. **Create:** `docs/spatial-3d.md`, `docs/superpowers/audits/2026-10-22-spatial-3d-application-evidence.md` (таблица сценариев с колонками `command, exit, artifact hash, platform` — все `pending V08`), `native/tests/spatial/documented_examples.cpp`, `examples/spatial/examples_manifest.json` (argv каждого документированного примера + ожидаемые `mode/schema/root/duration>0`). **Modify:** `docs/{RIDE.md,TESTING.md,perf.md}`, `CONTEXT.md`, manifest W44 (ссылки на examples).

- [ ] `docs/spatial-3d.md`: сценарии — planar как прежде; constrained test; прямой free-root заезд; cross-slope; noise2 research; запись; inspect; checked replay; native report; process sweep. Для каждого: physics mode, root, tyre tier, terrain/seed, dt, пределы баланса, sensor schema, явное включение записи, ожидаемая terminal classification; команды — только реально существующие flags (W45) и установленные examples (W48). Клавиши камеры по W46; inspection vs checked replay; диагностическая скорость vs квалифицированная физика. Статус capability — таблица `implemented-in-source / verification pending`.
- [ ] `documented_examples.cpp`: читает `examples_manifest.json`, для каждого примера `bike::cli::parse_arguments(program, argv)` → `resolve_ride`/`resolve_research` → `bt::require` `mode/schema/root` и `duration>0`; никакого ride/GUI/stepping; `arguments.text("spatial")=="3d"`, `arguments.text("terrain3d")==argv[3]`.
- [ ] `TESTING.md`: labels `spatial`, `slow`, `gui`, `package`, `perf`; default preset исключает slow/audit/package — явно перечислить обязательные spatial gates вне default.
- [ ] `perf.md`: раздел spatial-fast-v1 — цели 2.0/1.0, «измерений нет до V09».
- [ ] CMake: `bike_add_spatial_test(documented_examples SOURCES documented_examples.cpp LIBRARIES bike_cli bike_configuration)`.
- [ ] Коммит `docs(spatial): W49 native workflows and capability coverage`, `Verification: pending V08`.

```cpp
const std::array<std::string_view,4> example{"--spatial","3d","--terrain3d","examples/spatial/cross_slope.terrain3d"};
const auto arguments = bike::cli::parse_arguments("bike-ride", example);
bt::require(arguments.text("spatial") == "3d", "example selects spatial mode");
bt::require(arguments.text("terrain3d") == example[3], "example asset path preserved");
```

## Матрица доказательств (заполняется V08)

| Сценарий | Вход | Проверяемый результат | Owner |
|---|---|---|---|
| Совместимость planar | default example / schema1 | прежние resolved mode, channels, reader | W45/W48 |
| Constrained control | явный constrained request | 3D данные доступны, root ограничен | W45/W49 |
| Прямой free-root заезд | fast fixture, seed, duration | valid ride без scored hold, достоверная причина завершения | W36/W48 |
| Поперечный уклон | installed grid + steering profile | камера следует Y, contact axes согласованы | W46/W48 |
| Исследование сенсоров | V2 mount/tape/ABI2 plugin | 11 каналов, masks, timestamps | W39/W45 |
| Запись + inspect | явно включённая запись / schema2 | переносимые assets, просмотр без input paths | W42/W48 |
| Checked replay | contract4 / точная identity | полное сравнение либо объяснённая несовместимость | W43/W48 |
| Автономный отчёт | spatial container вне source tree | SVG/HTML с пропусками N/A | W47/W48 |
| Process sweep | неизменяемый job manifest | стабильные job keys, сортированный summary | W47 |

## Контроль границ фронтенда

| Вход | Разрешено | Запрещено |
|---|---|---|
| Камера | камера и render replica | qpos/qvel/tyre state |
| HUD | чтение принятых полей | пересчёт сил/сенсоров |
| Пауза/темп | отсрочка advance | пропуск ticks/RNG |
| Reset | явный lifecycle, новая generation | скрытая смена container/schema |
| Остановка записи/экспорт | финальная публикация | повторный rollout |
| Просмотр старой записи | воспроизведение | «verified» без checked replay |

## Передача

W45–W49 не вводят новые уравнения и не ослабляют проверки. V08 получает install/GUI/process evidence; V10 — таблицу возможностей для M6.
