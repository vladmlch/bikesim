# W00–W05: исходники и передача на V00/V01

Дата: 2026-10-23. Статус шести задач: **`source_written`**.
Статус исполнения и квалификации: **`pending_verification`**.

Основание: план `2026-10-22-spatial-3d-native-00-foundation.md`, C0–C10,
master plan и прямое указание заказчика не компилировать и не запускать проект.
Configure, build, compile, syntax-only, CTest, приложения, benchmark и генерация
fixtures прогоном не выполнялись. Коммиты, ветки и состояние локального Git
заказчика этой поставкой не изменялись. Плановые acceptance-checkboxes не закрыты.

## Состав поставки

| Задача | Написано | Что принимает V |
|---|---|---|
| W00 | Public headers пространственной геометрии, terrain, шин, bootstrap, rider, sensors, checkpoint, policy V2, replay, recording, viewer и test support; необходимые POD/private storage и константы. | Самодостаточность headers, ABI/layout, ownership и согласование C10. Реализация будущих подсистем остаётся у соответствующих W-задач. |
| W01 | `upstream-gates.json`, manifest planar capture и побитовый comparator в `test_baseline.cpp`. | Реальные независимые captures на обеих платформах, заполнение identity и hashes. |
| W02 | Девять записей C1–C9 в `contracts.json`, отдельные версии схем, proposed ADR 0004, дополнение `interface-contracts.md`. | Сверка producer/consumer, версии, точные ключи и свободный номер ADR. |
| W03 | Реестр физических источников, данные Whipple/Meijaard и опубликованные измерения lateral tyre characteristics, provenance и тесты данных. | Независимая сверка транскрипции, исходные payload/hash/license там, где они ещё pending. |
| W04 | Пять CTest targets, bounded fixture loader, контроль нулевого числа проверок, registry/reference/config/baseline tests. | Сборка и выполнение в рамках V00/V01. |
| W05 | Typed spatial configuration, строгий parser/encoder, TOML round trip и overrides, capability gate, runtime2/writer3 authoring boundaries, 11 TOML fixtures и конфигурационные тесты. | Проверка отказов, round trip, сохранения planar wire и capability dispatch. |

`migration/spatial3d/task-index.json` фиксирует `source_written` только для
W00–W05. S00, последующие W и V не объявлены выполненными этой поставкой.

## Поведение W05

Отсутствующий `spatial` и явный `spatial="planar"` нормализуются в прежнюю
конфигурацию без extension. Старый FieldDecl registry и его planar key set
сохранены. Вызовы старого TOML writer используют ту же проекцию для planar;
численное и побитовое равенство результатов должно быть подтверждено в V.

Для `spatial="3d"` создаётся принадлежащий конфигурации объект `Config`.
Поддержаны root `free`/`constrained`, настройки steer, bootstrap, IMU, terrain
request, fast/detailed tyre tier и identity lateral profile. Неизвестные ключи
в extension, неверные enum/domain/version, конфликтующие aliases, нечисловые
и не конечные значения отклоняются с путём поля. Для constrained root нужен
controller `off`; физический 3D input требует backend `pneumatic_3d`.

Source TOML и resolved wire имеют разные определённые формы. Resolved spatial
wire содержит точные восемь корневых ключей C7.0 и все вложенные поля, включая
явные null для отсутствующих grid overrides. Public TOML опускает optional
null и производный terrain corridor, чтобы частичный override корневой
ширины не конфликтовал с унаследованной копией. Все семантические поля входят
в canonical SHA-256 fingerprint.

`max_asset_bytes` хранится в `uint64_t`, но допустим только в диапазоне
`[1, INT64_MAX]`: значение должно переживать TOML integer round trip.
Владеющие строки проверяются на встроенный NUL и при прямой работе с typed
объектом. Пути terrain разрешаются относительно объявляющего TOML через
общий `resolve_config_paths`, без чтения или генерации самого terrain asset.

Overrides сохраняют прежнюю семантику null-as-absent. Вложенные tyre aliases
переопределяют соответствующие унаследованные canonical fields; противоречия
в самом override document остаются ошибкой. Строковый lateral alias задаёт
id; объект `{id, sha256}` задаёт оба поля. Переход `3d -> planar` удаляет
унаследованные spatial sections и требует явного допустимого legacy backend.
Spatial sections, явно переданные вместе с planar, остаются ошибкой.

Capability для каждой 3D-конфигурации остаётся закрытым:
`implemented=false`, `qualification_pending=true`, причина начинается с
`spatial runtime not connected`. Действующие runtime и writer entry points
проверяют этот запрет до planar model lookup. Старые rider/drive ограничения
сохраняются и не обходятся spatial extension.

`project_spatial_runtime_envelope` формирует runtime_schema 2 и writer schema 3
из явно переданных разрешённых полей. Он проверяет корневые ключи, периоды,
drive, monitors, spatial identity и структурную форму geometry. Это authoring
boundary: пространственные pose/rider codecs, повторная привязка имён к модели
и bootstrap schema 2 относятся к W17. Числовые joint indices здесь не являются
переносимой topology identity. Schema 3 writer после строгой проверки envelope
явно отказывается исполняться до W25; pneumatic3d interior ещё не реализован.
Действующий planar `RuntimeConfig` сохраняет runtime schema 1.

## Тесты, подготовленные для исполнения в V

Все пять targets используют существующий `bt` harness. Fixture loader
ограничивает размер, отклоняет выход за fixture root и неполные чтения.
`spatial.harness` содержит отдельный subprocess control, который должен
доказать отказ при нулевом числе checks; в этой поставке он не запускался.

`spatial.config` проверяет enum/defaults, все поля typed config через wire и
public TOML, canonical fingerprint, partial overrides, legacy null semantics,
asset-path anchoring, отрицательные fixtures и domain diagnostics. Отдельно
проверяются старый frozen schema и existing physics TOML examples, exact
runtime/writer version dispatch и отсутствие обхода capability gate.

Planar manifest содержит **null** там, где нужны измерения, файлы и runtime
identity. `spatial.baseline` намеренно завершается ошибкой при пустом manifest,
отсутствующих captures или нулевых payload. Выдуманных трасс и успешных
execution evidence нет. Comparator требует два независимых captures с
совпадающей execution/provider identity и побитовым равенством четырёх
стадий (`t0`, steps, reset, probe); сравнение проводится отдельно по платформам.

Этот comparator фиксирует повторяемость выбранного S00 artifact. Он сам по
себе не доказывает регрессионное равенство разных сборок до/после миграции:
для такого утверждения V должен определить подходящую identity/qualification
процедуру. Сравнение одного файла с собой, повторное использование hardlink
и неполная identity отвергаются.

## Физические источники

`docs/reference/spatial3d-provenance.md` и два reference JSON содержат ссылки,
DOI, единицы, местоположение таблиц и правила преобразования данных.
Whipple defaults привязаны к Meijaard et al. (2007). Tyre rows взяты из
опубликованных измерений городских/грузовых велосипедных шин и помечены
`cross_check_only`; они не квалифицируют MTB profile. Синтетические hashes
в parser tests явно обозначены и не служат evidence профилей.

Транскрипция имеет статус `data_entered_pending_review`. Получение raw Zenodo
payload и его SHA-256 не подменено ссылкой на запись; соответствующие поля
остаются pending/null. Источники не объявлены проверенными прогоном модели.

## Контракты, требующие решения V00/C10

Полный список находится в `migration/spatial3d/contract-deviations.md`.
В частности, prescribed W00 declarations пока оставляют три вопроса:

1. У pneumatic evaluate/try_compute_into нет явного physical interval identity
   для различения повторного advance и следующего идентичного входа.
2. Brush API не определяет передачу immutable surface table для разрешения
   `surface_id` в коэффициенты локального трения.
3. Fast brush API не передаёт все camber/axle/spin inputs, необходимые
   соответствующему будущему закону сил.

В W00 сохранены предусмотренные планом declarations. Эти вопросы должны
получить согласованное C10-решение до квалификации W23–W25. Ни физическая
реализация, ни гарантия duplicate-advance здесь не заявлены.

## База патча и применение

Патч подготовлен относительно предоставленного снимка:

- Архив: `repomix-output.xml(20261023-190035).zip`.
- SHA-256 ZIP: `13fb7b184463d7d48b33bea689d08b5a51b2f69ccaec7096855ecb7e8da3a97b`.
- SHA-256 XML: `525e4d1a7551997c5b6740da680b879472ad3e754f938492d3906f3c02cf5cca`.
- Извлечено 2302 файла. Небезопасных и повторяющихся путей при извлечении нет.
- S00 `base_sha` из переданного `baseline.json`:
  `fe4feaa560df68bce0e15b5b0d8d76eced29a062`.

Git object database и локальные worktrees заказчика в repomix отсутствуют.
Указанный S00 SHA — сохранённая provenance из входных документов, не новый
результат `git rev-parse` в этой сессии. Исторический source_export внутри
`baseline.json` оставлен неизменным; текущий export записан отдельно в
`task-index.json::latest_source_delivery` и здесь.

Repomix убрал завершающий LF у большинства текстовых файлов. Для diff-base
восстановлен завершающий LF; шесть доступных dependency files при этом
совпали с SHA-256 из S00. `uv.lock` отсутствует в export. Патч не содержит
массового изменения завершающих переводов строк в нетронутых файлах.

В исходном S00 описан незавершённый lane/b merge и port window между
W00–W04 и W05. Этот Git merge здесь не выполнялся. Если локальная ветка уже
получила upstream port, общие configuration/runtime/CMake файлы необходимо
согласовать с ним при применении; patch matching не доказывает правильный
merge-order или наличие upstream qualification evidence.

Из корня соответствующего локального checkout:

```sh
git apply --check /absolute/path/bikesim-w00-w05.patch
git apply /absolute/path/bikesim-w00-w05.patch
```

Проверка упаковки ограничена текстовым применением patch к восстановленной
копии export и сравнением файлов с подготовленным деревом. Она не является
configure, компиляцией, тестом или выполнением проекта. Реальные acceptance
остаются у V00/V01 после разрешения заказчиком исполнения.
