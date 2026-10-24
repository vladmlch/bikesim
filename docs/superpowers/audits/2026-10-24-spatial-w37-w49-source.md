# W37–W49: исходники, патч и передача на V07/V08

Дата: 2026-10-24. Все тринадцать W-задач этой поставки имеют статус
**`source_written`**, проверка исполнения — **`pending_verification`**.

По прямому указанию заказчика не выполнялись configure, build, компиляция,
syntax-only, CTest, запуск приложений, GUI, install, симуляций, benchmarks
или проектных генераторов. Написанные тесты не объявляются пройденными.
Acceptance-checkboxes исходных планов остаются открытыми; V07/V08 и
предшествующие V-гейты не закрываются регистрацией исходников в CMake.

## Исходный snapshot и применение

| Поле | Значение |
|---|---|
| Архив | `repomix-output.xml(20261024-043811).zip` |
| SHA-256 архива | `e17e4c119abd26dc661d903739d2abf45be1e556be8db8d8e9cc45a0a93cade5` |
| SHA-256 XML | `e2cad908de260ad92a0241a2fd0892ac2b1d2336010ead4c88b1c7885108c578` |
| File blocks | 2506 безопасных уникальных путей |
| Исходная Git history | В Repomix отсутствует |
| Основа diff | Извлечённый snapshot, уже содержащий W00–W36 |
| Формат | Git unified patch с путями `a/…`, `b/…` |

Extractor удаляет завершающий перевод строки. Для упаковки патча terminal LF
восстанавливается одинаково в pristine base и окончательном snapshot у непустых
текстовых файлов. Это не массовое переформатирование содержимого. Локальная synthetic
Git-база нужна только для diff и не представляет upstream SHA заказчика.

Патч применяется из корня соответствующего репозитория:

```sh
git apply --check /path/to/bikesim-w37-w49.patch
git apply /path/to/bikesim-w37-w49.patch
```

Проверка и применение diff не собирают и не запускают проект. Если локальное
дерево изменилось после snapshot, конфликты контекста требуют обычного
согласования; патч не предписывает принудительно затирать локальную работу.

## W37–W44: observation, research, recording, checkpoint и replay

| Задача | Написанная реализация | Написанная проверка |
|---|---|---|
| W37 | Bind IMU site/sensor IDs, type/dimension/body checks, Rz·Ry·Rx misalignment, interval accelerometer/gyro cache | `spatial.sensor_mount` |
| W38 | 11 каналов V2, validity mask, preallocated delay ring, bias/noise/dropout, строгий atomic import, отдельные RNG streams | `spatial.sensor_pipeline` |
| W39 | Observation variant, native dispatch ABI1/ABI2, plugin validation/RAII/exception boundary, V2 built-ins, paced research | `spatial.policy_v2` и настоящий test MODULE |
| W40 | Canonical spatial truth, accepted-prefix outcome/slip/contact metrics, geometry/energy projections | `spatial.truth_metrics` |
| W41 | Catalog2, независимая container2 ветка, sensor2 SQL, quaternion/free-root coordinates, точное множество таблиц и каналов | `spatial.recording_catalog` |
| W42 | Полный cold checkpoint, staged plant/research swap, запись input assets и initial/final checkpoints, публикация | `spatial.checkpoint`, `spatial.recording_assets` |
| W43 | Recipe4, переносимые assets, identity checks, replay через тот же runtime, field-path comparisons полного состояния | `spatial.checked_replay` |
| W44 | Структурный manifest validator, reviewed freeze/compare, platform-specific reference identity и declared tolerances | `spatial.golden_registry` |

Planar `SensorObservation` и `policy_v1.h` сохраняют исходную форму. В spatial
observation первые шесть значений — acceleration XYZ и gyro XYZ, затем
front/rear/crank encoders, motor/human torque. Политика получает только эту
наблюдаемую часть, без указателей на terrain, model или privileged truth.
V1-only plugin не превращается в ABI2 посредством неявной проекции.

Версии разделены: runtime2/writer3/bootstrap2, sensor2, ABI2, catalog2,
container2, replay4, tyre-state1 и checkpoint1. Неизвестные версии, неверная
форма, несовместимые hashes или model-bound dimensions отвергаются явно.
Legacy V1 export отказывает spatial входу до создания результата; обычный
container2 export сохраняет V2 arrays и validity flags.

### Полнота и атомарность checkpoint

`bootstrap2` сохраняет integration state, mutable model arrays, tyre material,
drive, rider/contact/intent/controller, hold, brake, steer и outcome.
Его `runtime.continuation` дополняет существующий t=0 inventory состоянием
незавершённого accounting period, pending/retained publication references,
measurement caches, interval IMU/truth, held rider kinematic preview,
acceleration inputs и опубликованным presentation cache. Один sample payload
имеет одного владельца; pending/retained/latest ссылаются на его interval ID.
History, model status, reference monitor, generation и energy не дублируются
в accounting extension: они принадлежат прежнему runtime envelope.

Research сохраняет задержанные команды, активное control window, ack,
acquisition/delivery/RNG cursors, sensor queue, observation/transition histories,
recorder references и метрики. Restore сверяет фактическую research configuration
и noise tape, строит временный plant и research episode, затем меняет владельцев.
Ошибка последнего поля не должна изменять live runtime. Reset после restore
сохраняет исходный смысл: первоначальное t=0, новая generation, без повторного hold.

Тест continuation содержит явный малый manufactured lateral prestrain и
проверяет до сравнения наличие lateral bristles, активного brake anchor,
sensor FIFO, actuator FIFO и control window. После restore сравниваются
500 физических интервалов, control/measurement rows и полный checkpoint.
Это тест причинного состояния, не измеренный физический эталон.

Cold checkpoint поддерживает synchronous accounting. Активный async worker
не сериализуется и отвергается, вместо неявного drain/mutation. Policy state
помечается `resumable:false`: checked replay использует записанные команды;
продолжение произвольного живого policy plugin этой реализацией не обещается.

Полный checkpoint разрешён только на принятой, доступной engine boundary.
Если engine poisoned после частичного неуспешного шага, evidence сохраняет
доступный принятый префикс с явным `checkpoint-unavailable`, а не вымышленным
causal state. Такая запись предназначена для inspection; checked replay
должен отказать до создания runtime. Обычные принятые terminal outcomes
сохраняют полноценный final checkpoint.

### Replay и goldens

Recipe4 фиксирует model/config/terrain/calibration/execution identity и
initial/final checkpoints. Terrain и необходимые профили переносятся в assets.
Replay сохраняет режим полной записи, необходимый для сравнения всего
checkpoint inventory. Inspection не требует совпадения execution identity;
checked replay при несовпадении возвращает `incompatible-execution`.

Числа evidence сравниваются по C7: `abs(a-e) <= max(1e-9,
1e-9*max(abs(a),abs(e)))`; для state действует `1e-9 + 1e-9*abs(e)`.
Флаги, строки, версии и целые counters сравниваются точно, без преобразования
больших целых в double. Ошибка сохраняет первый field path и шаг.

`tests/fixtures/spatial3d/manifest.json` содержит pending entries для двух
платформ. Неизвестные measured hashes остаются `null`. Наличие записи в
registry не делает её qualified. Freeze требует явного `--accept-reviewed`;
within-platform comparison сохраняет точные bytes, cross-platform comparison
разрешает только явно объявленные в manifest поля и допуски.

## W45–W49: приложение и доставка

| Задача | Написанная реализация | Написанная проверка |
|---|---|---|
| W45 | Общий resolver CLI/config, spatial/root/tier/terrain/grid/steer/IMU options, explicit recording и physical duration outcome | `spatial.cli` |
| W46 | Accepted endpoint pose, quaternion interpolation, heading tracking, spatial camera/HUD, owned render copy | `spatial.viewer_pose`, `spatial.viewer_smoke` |
| W47 | Spatial plot/report/query/export, реальные axle trajectories, energy channels, resolved batch identity и deterministic summary ordering | `spatial.reports_batch` |
| W48 | Native link/install lists, catalog2/policy2/examples, relocated installed-product и package controls | `spatial.installed_product`, расширенные install/package smoke |
| W49 | Bounded examples, machine-readable workflow manifest, usage/development/troubleshooting/performance documentation | `spatial.documented_examples` |

`--out` задаёт destination, но не включает recording. Terrain/options
проверяются общим cold resolver до setup и открытия viewer. Spatial error
не вызывает fallback в planar mode. Существующий CLI configuration TU
принадлежит одному общему static-library target для CLI и batch без дублирования
компиляции и зависимости batch от всего application frontend.

Canonical `channel_spatial` остаётся точным исходным C7 набором. Для W47
добавлены явные вспомогательные schema2 таблицы `channel_spatial_energy` и
`channel_spatial_geometry`: cumulative hold work/tyre loss и реальные world
positions осей. `channel_spatial_outcome` сохраняет принятый outcome latch;
`channel_sensors` schema2 дополнен двумя mounted IMU vectors. Потери tyre
берутся из material counters, а не из общих
energy losses; колесная траектория не выводится из постоянной геометрии рамы.
Конкретное расширение отражено в C10 ledger.

Viewer читает опубликованное состояние. Получение кадра не выполняет force
probe или engine forward. Camera/render interpolation не изменяет replayable
physics. Пустые/invalid chart segments отображаются как отсутствие данных.
Batch identity зависит от разрешённых физических inputs; сборка summary
упорядочивает job results независимо от порядка окончания процессов.

GUI smoke регистрируется только для `BIKE_BUILD_GUI`, с labels
`spatial;gui;slow`, и требует display. Install/package проверки используют
временный каталог и установленное дерево; специальный native test helper
готовит schema1 и checksum-valid incompatible-execution controls.
Написание этих сценариев не означает фактическую установку или GUI acceptance.

## Границы результата и следующая проверка

Полный список source решений и несовпадений с frozen sketches находится в
[`contract-deviations.md`](../../../migration/spatial3d/contract-deviations.md).
Существенные пункты: ошибочный `noexcept` у проверяемого IMU reader;
targetless stage signature, которому нужны target-bound allocations;
nonzero-time bootstrap extension; auxiliary report channels; diagnostic-only
запись при недоступном checkpoint после engine failure.

Прежние ограничения W20–W36 не объявляются устранёнными: измеренная lateral
calibration, general material transport, extrusion surface ownership и
strict Savage acceptance требуют своей C10/V работы. Spatial capability
не получает `supported=true` на основании этой исходной поставки.

V07 должен выполнить build/link, V1 regression, IMU semantics, real plugin,
continuation/atomic rejection, container/asset, corruption/replay и platform
golden qualification. V08 должен выполнить CLI/example validation, reports,
process-count invariance, relocated install/package и GUI acceptance на
реальном display. Ни один exit code или измеренный результат этих проверок
не был получен при подготовке патча.

Schema1 regression в `spatial.reports_batch` требует исходный бинарный fixture
`tests/fixtures/migration/recording/good/run.duckdb`. Repomix не содержит этот
бинарный файл: перед V08 его нужно получить из штатного набора fixtures.
Тест сообщает об отсутствии данных, без skip или автоматически созданного эталона.
