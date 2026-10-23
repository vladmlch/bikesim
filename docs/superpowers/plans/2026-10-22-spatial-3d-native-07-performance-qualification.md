# Spatial 3D Native — 07. Производительность, детерминизм и квалификация (W-фаза: W50–W54)

> **Для исполнителей:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development (рекомендуется) или superpowers:executing-plans. Все задачи — **one-shot write** tooling/validators/pool-кода против frozen headers W00 (`00-foundation.md` §W00.9 `spatial_performance.hpp`, `spatial_worker_pool.hpp`; W00.11 `qualification_registry.hpp`). Запрещены configure/build/compile/ctest/запуск/измерения/санитайзеры. Измерения, решения go/no-go и итоговая квалификация — `09-verification.md` V09 (S50–S53) и V10 (S54).

**Goal:** Написать measurement driver/report/profile, детерминированный контракт пула и его реализацию, probe engine pool, qualification validator и скелеты evidence — целиком, без запуска.

**Architecture:** Один детерминированный C++-шаг и один владелец `mjData`. Пул вычисляет независимые элементы шины в заранее назначенных ячейках; суммирование и commit — владелец в неизменном порядке. Процессы batch, потоки шины и внутренний пул MuJoCo — три независимых параметра.

**Tech Stack:** C++23; существующий `bike-measure-realtime` (`native/cmake/RealtimeMeasurement.cmake`, `runtime/realtime_measurement.cpp:35–112`, `realtime_report.cpp:92–108`, `realtime_driver.cpp:37–56,139–157`); монотонные часы; `bt`.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`; contracts C8; приёмка шины W28/V06, физики W36/V06, fixtures W44/V07, приложения W49/V08.

## Общие ограничения

- Цель уровня `fast` (проверяется V09/V10): медианный headless RTF ≥2.0; с окном достигнутый средний RTF ≥1.0 при темпе 1×. Числа ниже — критерии, не измерения.
- Один поток — default; N=1 не создаёт рабочие потоки. Запрещены float atomics, непостоянный порядок результатов, RNG по расписанию, дерево суммирования от числа потоков.
- `BikeCompilerPolicy.cmake:343–349` документирует аварийное завершение TSan на arm64-Darwin; `NATIVE_TSAN` опции нет — W52 добавляет её только для Linux preset.
- `--benchmark` может измерять невалидный префикс, но не закрывает приёмку.
- W51/W52/W53 пишут код **полностью** (pool, probe) — решение `not_selected` принимает V09 по измерениям; неиспользованный код при `not_selected` удаляется в V09 отдельным коммитом (не остаётся dead code в продукте).

---

## W50 — Measurement driver, профиль и отчёт (← S50)

**Dependencies:** W28, W36 (fixture/demand trace), W19. **Create:** `native/src/runtime/spatial_performance.cpp`, `native/tests/spatial/performance_report.cpp`, `docs/superpowers/audits/2026-10-22-spatial-3d-performance.md` (скелет: profile identity, таблица runs, regions — все `pending V09`). **Modify:** `native/src/runtime/{realtime_measurement.cpp,realtime_report.cpp,realtime_options.cpp,realtime_provenance.cpp,realtime_driver.cpp}`, `native/src/viewer/pacer.{hpp,cpp}`, `docs/perf.md`.

**Interfaces:** W00.9 `rtf_threshold_met`, `SpatialPerformanceProfile`, `RegionTimings`.

- [ ] `rtf_threshold_met(sim, wall, target)`: finite, `sim>=0`, `wall>0`, `target>0` иначе `std::invalid_argument`; `return sim/wall >= target`.
- [ ] `realtime_driver.cpp`: холодный конструктор `NativeDriver(std::unique_ptr<bike::initialization::SetupRequest>, Options, Clock)` — готовит `NativeRideRuntime` напрямую, без CLI/research session; spatial ветка: завершение по W33 outcomes/duration/steps, limits progress/corridor/domain, `Terrain3dKind`; без planar `finish_x` и `terrain_vertices.npy`. Публичный measurement CLI с `--spatial/--terrain3d` (W45) — тот же driver.
- [ ] `realtime_report.cpp`: поля `profile` (`SpatialPerformanceProfile`), `regions` (`RegionTimings` с `*_pending` флагами), `runs[]{rtf, terminal_reason, steps, valid, warmup:bool}`, `median_rtf`, `target_met:bool|null`; прежний planar смысл сохранён; требование ≥1 warmup и ≥3 measured runs одной identity; non-strict run помечается `diagnostic`.
- [ ] `realtime_measurement.cpp`: regions измеряются монотонными часами вокруг terrain/rays, tyre elements, engine step, rider/steer, accounting без изменения арифметики; `sensors/recording/ui` → `pending=true` если отсутствуют в fixture; полное время включает финальный flush; подготовка/загрузка/warmup/экспорт — отдельно; overhead инструментирования — отдельное поле.
- [ ] `realtime_options.cpp`: `--profile spatial-fast-v1`, duration ≥30 s по умолчанию для spatial; `pacer.cpp`: учёт долга/опозданий/frame rate в отчёт (используется V10 для GUI measurement).
- [ ] `performance_report.cpp`: `rtf_threshold_met(30,15,2)==true`, `(30,15.1,2)==false`, `(30,0,2)` → `bt::invalid`; отчёт с управляемыми часами: нулевое/NaN время → reject; раннее падение → `valid=false` и исключение из median; несовместимая identity двух runs → reject; единственный run → `target_met=null`; неверная маркировка задержки порции (batch latency ≠ p95 шага) → reject.
- [ ] CMake: `bike_add_spatial_test(performance_report SOURCES performance_report.cpp LIBRARIES bike_runtime bike_spatial)`.
- [ ] Коммит `perf(spatial): W50 serial measurement driver and report`, `Verification: pending V09`.

```cpp
bt::require(runtime::spatial::rtf_threshold_met(30., 15., 2.), "headless target inclusive");
bt::require(!runtime::spatial::rtf_threshold_met(30., 15.1, 2.), "no threshold rounding");
bt::invalid([] { (void)runtime::spatial::rtf_threshold_met(30., 0., 2.); });
```

## W51 — Детерминированный контракт пула: ranges и ownership (← S51)

**Dependencies:** W50, W27. **Create:** `native/src/runtime/spatial_worker_pool_ranges.cpp` (только `partition_ranges`), `native/tests/spatial/worker_ranges.cpp`, раздел «Decision» в performance audit (`decision: pending V09`, критерии go/no-go записаны заранее).

**Interfaces:** W00.9 `IndexRange`, `partition_ranges`.

- [ ] `partition_ranges(count, workers)`: `workers==0` → `std::invalid_argument`; `begin = (count*i)/N`, `end = (count*(i+1))/N` через `std::uint64_t` с проверкой `count <= UINT64_MAX/N` (иначе — деление с остатком: `q=count/N, r=count%N`, первые `r` диапазонов длиной `q+1`); `N>count` → пустые диапазоны допустимы; каждый элемент ровно один раз.
- [ ] Канонический индекс элемента `(wheel, ring, longitudinal, lateral)` → линейный `((wheel*n_theta_fan)+it)*n_phi+ip` — документируется комментарием в `spatial_worker_pool.hpp` (без изменения header: комментарий добавляется в `.cpp`), порядок суммирования — по линейному индексу.
- [ ] Audit «Decision»: критерии — пул выбирается только если измеренное ускорение полного заезда после dispatch/barrier сохраняется и решает недостачу RTF либо даёт ≥20% запаса; иначе `not_selected`.
- [ ] `worker_ranges.cpp`: `partition_ranges(7,3)` → 3 диапазона, `front().begin==0`, `back().end==7`, стыки без overlap/hole; `(7,0)` → `bt::invalid`; `(0,4)` → четыре пустых; `(5,8)` → пять единичных и три пустых; `(UINT64_MAX/2, 3)` без overflow.
- [ ] CMake: `bike_add_spatial_test(worker_ranges SOURCES worker_ranges.cpp LIBRARIES bike_runtime)`.
- [ ] Коммит `design(spatial): W51 deterministic element work ownership`, `Verification: pending V09`.

```cpp
const auto ranges = runtime::spatial::partition_ranges(7, 3);
bt::require(ranges.size() == 3 && ranges.front().begin == 0 && ranges.back().end == 7, "ranges cover the complete element set");
for (std::size_t i=1; i<ranges.size(); ++i) bt::require(ranges[i-1].end == ranges[i].begin, "no overlap or hole");
bt::invalid([] { (void)runtime::spatial::partition_ranges(7, 0); });
```

## W52 — Реализация пула и тест инвариантности (← S52)

**Dependencies:** W51, W25–W28, W42. **Create:** `native/src/runtime/spatial_worker_pool.cpp`, `native/tests/spatial/worker_pool.cpp`, `native/tests/spatial/thread_invariance.cpp` (label `slow`). **Modify:** `native/src/pneumatic3d/model.cpp` (точка разбиения: `query_geometry`+`update_carcass` по ranges через `ElementWork`; `assign_patches`/brush/суммирование — владелец), `native/src/cli/{argument_contracts.inc,configuration.cpp}` (`--threads N`, default 1, upper bound 64, provenance), native check context/policy (`BIKE_NONBLOCKING` аннотация только после V09 RTSan — здесь не ставить), `CMakePresets.json` + `native/cmake/BikeCompilerPolicy.cmake` (опция `NATIVE_TSAN` для Linux preset `linux-tsan`, на Darwin — `message(FATAL_ERROR "TSan runtime unsupported on arm64-Darwin")`).

**Interfaces:** W00.9 `WorkerPool`, `ElementWork`.

- [ ] `WorkerPool::Impl`: `threads−1` постоянных `std::thread`, `std::mutex`+`std::condition_variable`, generation counter (`std::uint64_t`), `ready` подтверждение каждого worker при старте (join уже созданных при ошибке construction); `run(ranges, work, ctx)`: range 0 выполняет caller, остальные — workers по generation; ожидание всех; statuses собираются в заранее выделенный `std::vector<std::uint8_t>`; `stop()` — идемпотентный join вне hot path; одновременные `run/stop` → `std::logic_error` до публикации generation. `threads==0` → `std::invalid_argument`; `worker_count() = threads−1`.
- [ ] `model.cpp`: `ElementWork` thunk над `(TorusGrid3d, WheelKinematics3d, view, elements span, carcass spec/state, normals span)` — только запись в свои ячейки; владелец после `run` проверяет statuses, затем `assign_patches`/brush/сумма в линейном порядке; ошибка любого элемента → `committed_` неизменен.
- [ ] `--threads`: положительное представимое, `<=64`; в provenance `tyre_threads`; `hardware_concurrency` не используется для default.
- [ ] `worker_pool.cpp`: `WorkerPool serial(1)` → `worker_count()==0`; `WorkerPool(0)` → `bt::invalid`; `serial.stop(); serial.stop();` идемпотентно; повторные generations (1000 run) со счётчиком; пустые диапазоны; `run` с `work`, пишущим в ячейки — сумма равна serial; concurrent `run` из второго потока → `std::logic_error` (проверяется `try/catch`, не `bt::invalid`); негативный race control — намеренно общая ячейка в test-only work даёт различающийся результат (документирует, что тест способен поймать гонку; ожидается нестабильный результат → `bt::require(detected_race_or_equal)` только как smoke, основная проверка в TSan V09).
- [ ] `thread_invariance.cpp`: N∈{1,2,4,8} на flat extrusion, rough/cross-slope genspec, braking+steering, несколько patches, detailed tier → побайтно равны `complete_state_bytes`, `qfrc`, sensors, contact keys, последующие checkpoints; исключены только wall time и `tyre_threads` metadata.
- [ ] CMake: `bike_add_spatial_test(worker_pool SOURCES worker_pool.cpp LIBRARIES bike_runtime bike_pneumatic3d)`, `bike_add_spatial_test(thread_invariance SOURCES thread_invariance.cpp LIBRARIES bike_runtime bike_pneumatic3d bike_initialization bike_spatial LABELS slow)`.
- [ ] Коммит `perf(spatial): W52 deterministic tyre worker pool`, `Verification: pending V09`.

```cpp
runtime::spatial::WorkerPool serial(1);
bt::require(serial.worker_count() == 0, "serial mode creates no background workers");
bt::invalid([] { runtime::spatial::WorkerPool invalid(0); });
serial.stop(); serial.stop();
```

## W53 — Probe внутреннего пула MuJoCo (← S53)

**Dependencies:** W50, W52. **Create:** `native/tests/spatial/engine_pool_probe.cpp` (label `perf`), раздел «Engine pool» в performance audit (`decision: pending V09`). **Modify:** нет production файлов (hook в engine ownership/factory добавляется только при положительном решении V09).

- [ ] `engine_pool_probe.cpp`: читать поставляемый `mujoco/mujoco.h` locked версии и использовать реально объявленный API (`mju_threadPoolCreate`/`mju_bindThreadPool` или иной — W-исполнитель открывает header и записывает точные declarations и `mjVERSION_HEADER` в комментарий теста; при отсутствии API — тест `bt::require(false, "engine thread pool API not present in locked MuJoCo <version>")` как явный negative). Helper `bool qualifies_engine_pool(double baseline_wall_s, double candidate_wall_s, bool same_bits)` локально в тесте: finite, `>0`, `return same_bits && candidate <= 0.90*baseline`; `(10,8,false)==false`, `(10,9,true)==true`, `(10,0,true)` → `bt::invalid`.
- [ ] Probe (выполняется V09): одна полная модельная нагрузка; сочетания tyre1/engine1, tyreN/engine1, tyre1/engineN, tyreN/engineN; побайтное сравнение состояния при разных engine threads; печать JSON.
- [ ] CMake: `bike_add_spatial_test(engine_pool_probe SOURCES engine_pool_probe.cpp LIBRARIES bike_runtime bike_engine LABELS perf)`.
- [ ] Коммит `perf(spatial): W53 engine pool probe`, `Verification: pending V09`.

```cpp
bt::require(!qualifies_engine_pool(10., 8., false), "speed cannot override byte mismatch");
bt::require(qualifies_engine_pool(10., 9., true), "ten percent full-run gain qualifies");
bt::invalid([] { (void)qualifies_engine_pool(10., 0., true); });
```

## W54 — Qualification validator и evidence skeleton (← S54)

**Dependencies:** W44, W49, W50. **Create:** `docs/superpowers/audits/2026-10-22-spatial-3d-qualification.md` (скелет: identity кандидата `null`, таблица milestones M0–M6 и gates G0–G6 со статусом `pending`), `native/tests/spatial/support/qualification_registry.cpp`, `native/tests/spatial/qualification_registry.cpp`, `migration/spatial3d/qualification.json` (`qualification_schema=1`, `status:"pending"`, `milestones[M0..M6]{status:"pending",evidence:[]}`, `upstream_gates[G0..G6]{status:"pending"}`, `identity:null`). **Modify:** `docs/{RIDE.md,TESTING.md,perf.md}` — ссылки на qualification doc.

**Interfaces:** W00.11 `spatial_tests::validate_qualification`.

- [ ] `validate_qualification(value)`: root `qualification_schema==1`; `status ∈ {pending,passed,failed,blocked}`; `milestones` непуст и содержит ровно M0–M6; `upstream_gates` ровно G0–G6; `status=="passed"` требует `identity` (source_sha, dependency hashes, compiler, provider, config) non-null и у каждой milestone `status=="passed"` с непустым `evidence[]` (пути/hashes); иначе `std::invalid_argument` с путём.
- [ ] `qualification_registry.cpp`: `{"qualification_schema":1,"status":"passed","milestones":[],"upstream_gates":[]}` → `bt::invalid`; валидный `pending` документ (из `migration/spatial3d/qualification.json`) проходит; `passed` без identity → reject; `passed` с milestone `pending` → reject; дубликат milestone → reject.
- [ ] CMake: `bike_add_spatial_test(qualification_registry SOURCES qualification_registry.cpp LIBRARIES bike_io)`; `qualification_registry.cpp` support → `bike_spatial_support`.
- [ ] Коммит `test(spatial): W54 qualification validator and evidence skeleton`, `Verification: pending V10`.

```cpp
const auto false_pass = bike::io::parse_json(R"({"qualification_schema":1,"status":"passed","milestones":[],"upstream_gates":[]})");
bt::invalid([&] { spatial_tests::validate_qualification(false_pass); });
```

## Точные режимы сравнения (используются V09/V10)

| Проверка | Фиксировано | Сравнивается | Не доказывает |
|---|---|---|---|
| Однопоточный повтор | сборка, провайдер, config, seed, assets | побитно: state, forces, sensors, control | физической точности |
| Потоки N1/2/4/8 | всё кроме tyre workers | те же bytes + полный checkpoint | cross-platform |
| macOS и Linux | одна физическая спецификация | эталоны каждой платформы + физические допуски | общего побитового потока |
| RTF | полный валидный заезд | sim time / wall | корректности |
| Темп 1× | целевая частота, число шагов | достигнутый RTF, долг, опоздания, frame rate | предельной скорости |
| Checked replay | совместимая identity | два правила comparator + полное состояние | независимой валидации |

## Передача

W50–W54 передают V09 инструменты измерения и пул, V10 — validator и скелеты evidence. Числа (2.0/1.0 RTF, 10% engine gain, N-invariance) получают статус только после измерений в V09/V10.
