# W50–W55: исходники и передача на V09/V10/V11

Дата: 2026-10-24. Все шесть W-задач этой поставки имеют статус
**`source_written`**, проверка исполнения — **`pending_verification`**.

По прямому указанию заказчика не выполнялись configure, build, компиляция,
syntax-only, CTest, запуск проектных скриптов, приложений, симуляций, GUI,
benchmark, sanitizer, install или установка GPU SDK. Написанные тесты не
объявляются пройденными. Checkbox-приёмка планов остаётся открытой.

## Исходный snapshot и патч

| Поле | Значение |
|---|---|
| Входной архив | `repomix-output.xml(20261024-051341).zip` |
| SHA-256 архива | `d3ee1137abc327ad44f1df88e3b734a59988026ee1873440a2913bc2e23c57ae` |
| SHA-256 XML | `b1c3353d767d03c4f284496a1ae60f43ace40b3e465a774ce2e6196c5e4e56b8` |
| File blocks | 2555 безопасных уникальных путей |
| Исходная Git history | В Repomix отсутствует |
| Основа diff | Присланный snapshot, уже содержащий W00–W49 |
| Формат результата | Git unified patch `bikesim-w50-w55.patch` |

Extractor удаляет завершающий перевод строки. Для diff terminal LF
восстанавливается одинаково в pristine base и итоговых непустых текстовых
файлах, как в предыдущей поставке. Synthetic Git index используется только
для изготовления патча; он не объявляет upstream revision заказчика.

Применение из корня соответствующего локального репозитория:

```sh
git apply --check /path/to/bikesim-w50-w55.patch
git apply /path/to/bikesim-w50-w55.patch
```

Это файловые операции; они не собирают и не запускают проект. Если локальные
файлы изменены после snapshot, требуется обычное согласование контекста,
без принудительного затирания пользовательских изменений.

## Написанные части

| W | Исходная реализация | Написанные проверки / следующий gate |
|---|---|---|
| W50 | Native cold driver, spatial-fast-v1, строгий отчёт, owner-local region scopes и GUI pacing report | `spatial.performance_report`, `spatial.pacer_report`; V09/V10 |
| W51 | Детерминированные непрерывные ranges с overflow-safe fallback и заранее заданные критерии выбора | `spatial.worker_ranges`; V09 |
| W52 | Постоянный pool N−1, раздельные geometry/normal slots, thread policy/provenance/replay, Linux TSan lane | `spatial.worker_pool`, `spatial.thread_invariance`; V09 |
| W53 | Test-only MuJoCo pool probe: четыре tyre/engine сочетания, полная нагрузка и state comparisons | `spatial.engine_pool_probe` (`perf;slow`); V09 |
| W54 | Exact qualification validator, M0–M6/G0–G6 registry и pending evidence | `spatial.qualification_registry`; V10 |
| W55 | Изолированный GPU decision helper, standalone bt test и memo | `tools/spatial_gpu`, `spatial.gpu_decision`; optional V11 |

Общие CMake owners приведены к фактическому дереву. В нём нет target
`bike_runtime` из sketches плана: измерение принадлежит существующему
`bike_realtime_measurement`, kernels — `bike_pneumatic3d`, startup/runtime —
существующей композиции `bike_startup_runtime`. Независимый static target
`bike_spatial_workers` исключает цикл runtime↔tyre. Его Threads dependency и
install owner добавлены в общий `NativeStartup.cmake`. Тесты используют реальные
dependencies; standalone GPU target не включён в продуктовый граф.

## W50: смысл измерения

Холодный driver принимает принадлежащий ему `SetupRequest`, options и clock,
подготавливает native startup и `NativeRideRuntime` напрямую. CLI отдельно
разрешает конфигурацию через существующую projection. Spatial путь завершается
по W33 outcomes, domain/corridor и точному числу шагов; planar finish и
`terrain_vertices.npy` относятся только к прежней planar ветке.

Профиль по умолчанию использует free root, articulated rider, fast tyre,
flat genspec с W36 surface coefficients, 3 m/s, seed 73421 и W36 F demand trace.
Состав профиля, фактические hashes/config/provider и число потоков входят
в отчёт. Имя профиля не подменяет физическую квалификацию его входов.

Полный wall interval включает final flush. Setup/export отдельны, warmup
исключён из median. Раннее падение, неполный prefix, invalid model, changed
identity и non-strict `--benchmark` остаются diagnostic. Нужны ≥1 warmup и
≥3 полных измеренных заезда одной identity, длительностью ≥30 s. Длительность
проверяется через полные ticks × dt и согласованность actual clock; RTF
считается по наблюдаемому времени без округления в пользу порога 2.0.

Таймеры не являются causal/checkpoint state. Вложенные scopes считают
exclusive owner time; worker threads не разделяют floating-point counters.
Dispatch/barrier входит в terrain/tyre wall time. Отсутствующие research sensor
pipeline, recording и UI остаются pending. Clock-read estimate явно отделён
от неизвестного полного observer overhead; `--no-region-timing` даёт отдельный
control, не закрывающий instrumented target. Chunk latency не маркируется как
per-step p95.

GUI выводит debt, frame intervals/FPS, achieved RTF и время seal/flush,
publication и close. В зачёт входят фактически завершённые положительные
draw/swap calls. Pause, scale/reset, hidden/empty framebuffer, interruption и
non-strict policy исключают 1× claim. Terminal idle исключён. Полное время
publication известно лишь после сохранения recording, поэтому итоговый report
публикуется в stdout и не переписывает сохранённый episode summary. V10 должен
подтвердить реальный display, matching profile и обе платформы.

## W51–W53: ownership, состояние и выбор пула

Pool создаёт N−1 фоновых workers, caller исполняет range 0; есть readiness,
generation и completion barriers. N=1 не создаёт фоновые потоки. Одновременные
run/stop отвергаются до публикации поколения, stop идемпотентен. Все ranges и
completion/status buffers подготавливаются заранее.

Параллелятся независимые geometry и normal element slots. Общие pressure,
материальная история, energies, patches, brush, суммы и commit остаются
у владельца в исходном порядке. Ошибка на втором колесе не публикует staging
первого. Продуктовая thread policy по умолчанию 1, диапазон CLI 1–64, batch
processes и engine threads остаются независимыми параметрами.

N=1/2/4/8 tests охватывают движущийся articulated rider с braking/steering,
rough/bank genspec, material multipatch, fast/detailed tiers, force/sensor/contact
bytes и последовательные checkpoints. Research case сохраняет настоящие
sensor/actuator queues, RNG cursor и histories. Перед нормализацией одного
derived runtime-config digest helper доказывает, что полные конфигурации
отличаются только `tyre_threads`; causal payload не вырезается.

Linux TSan имеет отдельный preset, compile/link capability, runtime identity
и согласование native/Python context readers. Darwin TSan явно unsupported.
Intentional race control запускается отдельно от обычного CTest и требует
реальной TSan data-race диагностики вместе с exit 66. В этой поставке он не
запускался; `BIKE_NONBLOCKING` на основании source review не добавляется.

W53 использует реально прочитанный locked header MuJoCo 3.12.0:
`mju_threadpool(mjData*, int)`, `mjVERSION_HEADER=3012000`. Точные SHA и upstream
source references записаны в probe. Четыре сочетания tyre1/engine1,
tyre4/engine1, tyre1/engine4, tyre4/engine4 используют один serial-settled
bootstrap. Cold state comparisons отделены от scored timing. Продуктового
engine hook нет. Критерии выбора записаны в
[performance audit](2026-10-22-spatial-3d-performance.md) до измерения.

## W54/W55: evidence и границы GPU

Qualification validator требует ровно M0–M6 и G0–G6, уникальные IDs, известные
statuses, корректные identity и evidence paths/SHA. Любая passed-строка требует
identity и evidence; общий passed требует всех строк passed. Структурная
проверка не подтверждает существование/достоверность evidence. Persisted
`qualification.json` содержит только pending и `identity:null`.

GPU helper фиксирует H1: ≥2× секции и ≥10% полного заезда при сохранённой
точности/детерминизме; H2: ≥20× throughput при той же физике; H3: ≥3× полного
offline task с exact results. Standalone CMake переиспользует bt и его cold
JSON/hash closure, проверяет уже полученные pinned RapidJSON headers и не
приобретает SDK. Root option только указывает на отдельный каталог. Ни backend,
ни GPU host, ни ускорение не объявлены выбранными или измеренными.

## C10 и оставшиеся ограничения

Все решения отражены в
[`contract-deviations.md`](../../../migration/spatial3d/contract-deviations.md).
Явное отклонение от W00 declarations: для реального владения persistent pool
`Pneumatic3dModel` получил cold setter/count accessor, destructor и private
owner; writer — forwarding setter. Frozen WorkerPool/RegionTimings и
geometry/carcass signatures не менялись. Это требует C10/V00 review и не
объявляется заранее одобренным контрактом.

Прежний blocker extrusion surface resolver сохранён: модель не может получить
material registry через frozen extrusion view. Соответствующий N-invariance
case написан без skip и должен явно отказать до upstream исправления. Исторические
Savage domain/calibration и независимые physical V-gates также не считаются
закрытыми. Входной архив не содержит runtime binaries или platform goldens;
наличие исходников не заменяет эти данные.

V09 отвечает за build/link, controlled-clock tests, полноценный валидный
serial baseline, N-invariance, sanitizer controls, overhead и решения выбора
пулов. V10 отвечает за итоговую qualification, recording, реальный GUI и две
платформы. Optional V11 отвечает за standalone build и измеренные GPU решения.
Ни одного измеренного RTF, passing test, qualified platform или approved pool
этот source handoff не утверждает.
