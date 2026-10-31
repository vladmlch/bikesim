# C++ Naming and Logging Unification — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Устранить подтверждённую неоднозначность имён и унифицировать диагностические события first-party C++, не изменяя физику, realtime-гарантии и существующие внешние форматы.

**Architecture:** Два независимых направления после фиксации контрактов: семантические переименования небольшими вертикальными срезами и единая политика диагностики на нерилтаймовых границах приложений. Существующие модели владения, namespaces, численные алгоритмы и форматы результатов остаются на месте; новый универсальный logging framework не является целью.

**Tech Stack:** C++23, CMake, существующий C++ contract harness `bt`, CTest, clang-tidy; действующий transition gate через `uv` и `tools/run_tests.sh`. Никаких новых сторонних зависимостей.

**Spec:** Запрос пользователя в этой сессии от 2026-10-30: «One concept → one name. One name → one meaning»; «The same kind of event should be logged in the same way, regardless of where it occurs». Все 12 требований к naming и 14 требований к logging входят в область плана. Отдельный согласованный design/spec-файл не создавался.

## Global Constraints

- В этой сессии: «ничего не компилируй и не запускай». Только чтение исходников и составление этого плана; команды ниже — исключительно для будущей реализации после отдельного разрешения.
- Исходный commit прочитан из git ref: `7020a89b4ffa855f432e4b89a67826ed5ef79b48`, ветка `impl/python-free-cpp`. Чистота working tree не установлена: `git status`, фильтры Git, сборки, тесты и анализаторы не запускались. Ссылки на строки относятся к прочитанным файлам, а не гарантированно к чистому HEAD.
- Работать в текущем worktree; не переходить в основной checkout и не перезаписывать чужие изменения. Перед будущей реализацией установить состояние дерева и согласовать пересекающиеся изменения.
- Source-supported означает подтверждение исходниками, но не runtime-воспроизведение. Никакого заявления о green gate или измеренной производительности этот документ не делает.
- Сохранять valid-input binary64 operation order, deliberate saturation, exception safety, poison-контракт и ownership по `docs/agents/native-engineering.md:27–43`.
- Не менять публичные JSON/TOML/NPZ/DuckDB keys, CLI flags, ABI v1, binding names, значения golden fixtures и вычислительный порядок под видом косметики. C++ member rename и rename сериализованного поля — разные изменения.
- Existing namespaces `runtime`, `rider`, `drivetrain`, `biketyre` сохраняются: это явный контракт `migration/interface-contracts.md:22`, а не повод для тотальной миграции в `bike::*`.
- Не редактировать сторонние/сгенерированные деревья `.deps`, `build`, `native/build*`, package snapshots, численные reference checkouts и исторические migration evidence ради единообразия.
- Не добавлять строки, форматирование, файловый ввод-вывод, locks или allocation в realtime-ядра и engine callbacks. Существующий транспорт ошибок и момент их доставки сохраняются.
- Не вводить logging threads, глобальный mutable logger, сетевой сборщик, dependency на spdlog/fmt, новые CLI flags или tracing/session infrastructure без отдельной необходимости.
- Python-команды при будущей проверке — только через `uv`; не создавать Python tooling для нового постоянного C++ naming/logging gate.
- Существующий `native/.clang-tidy:6–16` сознательно исключает стилевой шум. Не включать `readability-*` оптом и не ослаблять defect-checkers.
- По текущему `tools/run_tests.sh:114–156` **и `native`, и `full` включают native preflight**. Для physics/realtime среза достаточно одного финального `full`; для остальных C++/tooling срезов — `native`. Не дублировать оба дорогих прогона механически.
- Поддерживаемые квалификационные lanes: macOS arm64 и Linux aarch64. Точные build/toolchain настройки брать из текущих `docs/development/macos.md`, `docs/development/linux.md`, а не из персональных абсолютных путей.
- Не создавать commits и не начинать реализацию автоматически: это отдельные действия вне текущего запроса.

---

## 1. Подход и границы

**Рекомендуется семантическая унификация по вертикальным срезам.** Один срез меняет declaration → implementation → consumers → adapters → tests, сохраняя внешнее представление. Так review показывает смысл конкретного переименования, а не тысячи механических замен.

Альтернативы:

1. Только документировать стиль — почти нет риска, но уже существующие неоднозначности останутся.
2. Массово переименовать всё и внедрить полноценный logger — большой diff, риск сломать схемы и realtime; не рекомендуется.
3. Выбранный вариант: правила, несколько подтверждённых исправлений, контрактные тесты, затем узкий контроль повторного появления долга.

Включены first-party `native/src`, `native/include`, `native/tests`, `tools/proto_native_bench`. Python и migration-документы читаются как потребители/контракты; изменения Python допустимы лишь в реально затронутом binding adapter/test, не как отдельная Python-cleanup кампания.

Полный реестр всех символов не следует подменять выборочными примерами этого аудита. Будущая приёмка требует покрытия всех first-party каталогов и явной классификации исключений, а не утверждения «все имена исправлены» по нескольким файлам.

## 2. Предлагаемый naming-контракт

Это целевой проектный словарь, а не утверждение, что каждое перечисленное правило уже нарушено.

| Категория | Правило |
|---|---|
| Casing | Типы — `PascalCase`; namespaces, functions, variables, parameters, members — `snake_case`. Сохранять существующий локальный стиль constants и private-member suffix; не смешивать semantic cleanup с переписыванием всех констант. |
| Терминология | Для каждого спорного термина фиксировать понятие, каноническое имя, границу и разрешённое внешнее написание. Не считать разные понятия синонимами только из-за похожих имён. |
| `get` / accessor | Получение уже имеющегося значения без I/O и скрытого вычислительного lifecycle. Идиоматичные `size()`, `state()`, `config()` не превращать в `get_*` автоматически. |
| `find` / `require` | Поиск, при котором отсутствие допустимо, — `find_*` с явным missing-result. Обязательное наличие с исключением — `require_*`. Не менять поведение поиска для соответствия новому слову. |
| `load` / `read` / `parse` / `decode` | `read_*` — чтение bytes/text; `parse_*` — синтаксический разбор; `decode_*` — wire-to-domain; `load_*` — получение готового объекта с I/O. Это разные операции, не синонимы для глобальной замены. |
| `create` / `make` / `build` | `make_*` — создание in-memory value/owner; `create_*` — создание внешнего ресурса; `build_*` — сборка составного model/artifact. Не добавлять `Factory` ради суффикса. |
| `calculate` / `compute` | Каноническое `compute_*` для общей вычислительной операции; доменные `solve_*`, `integrate_*`, `evaluate_*` сохранять, если они обозначают разные действия. |
| Mutation | `set_*` — присвоить; `update_*` — изменить из новых входов; `apply_*` — применить действие/эффект. Запрос не должен незаметно менять состояние. |
| Opposites | `open/close`, `start/stop`, `register/unregister`, `attach/detach`; `reset` — начальное состояние, `clear` — пустое содержимое. STL `erase/remove`, C++ `delete`, filesystem `remove`, ABI `create/destroy` не переименовывать. |
| Classes | `Config` — параметры поведения; `Options` — настройки одной операции; `State` — изменяемое состояние; `Snapshot` — зафиксированное наблюдение; `Result` — результат операции; `Owner` — реальное владение ресурсом. `Writer`, `Reader`, `Controller`, `Policy`, `Solver` оставлять за соответствующими доменными ролями. |
| Booleans | Predicates — `is_*`, `has_*`, `can_*`, `should_*` по смыслу. Командное `use_*` и config `*_enabled` допустимы и не равны наблюдаемому состоянию. Предпочитать положительную полярность; внешние флаги не переименовывать. |
| Collections | Plural для коллекции и singular для элемента; `*_by_id`, `*_by_name` только если это действительно lookup по указанному ключу. Не кодировать тип контейнера в каждом имени. |
| Abbreviations | `config`, `id`, `index`, `count`, `min`, `max`, `position`, `velocity`; `pos/vel/cfg/idx` не переносить в новый domain API. `qpos`, `qvel`, `mjModel`, BLAS/LAPACK, NumPy parity names и ясные локальные математические `i/j/x` — контекстные исключения. |
| Identity | `*_id` — identity в своей области; `*_index` — позиция; `*_name` — читаемое имя; `*_key` — lookup/serialized key; `*_path` — filesystem path; `*_ref` только для реальной ссылки. MuJoCo object ID не превращать в независимый UUID. |
| Units | Границы API и диагностик: `_s`, `_ms`, `_ns`, `_m`, `_rad`, `_rad_s`, `_m_s`, `_nm`, `_bytes`, `_count`. Различать simulation time, elapsed wall duration и absolute timestamp; единица не заменяет тип часов. `kph` и существующие schema spellings сохраняются на внешней границе. |
| Vague names | Убирать `data/info/value/result/process/handle` только там, где локальный тип/контекст не объясняет назначение. `io::Value`, generic container `value`, `mjData`, opaque ABI `void*` и короткий локальный `result` сами по себе не дефекты. |

**Правило выбора канона:** сначала точность доменного смысла и действующий contract, затем преобладающее имя. Частота слова сама по себе не делает его правильным. Не переименовывать `step` в `advance`, `episode` в `run` или `finalize` в `close`, не доказав равенство семантики.

## 3. Предлагаемый logging-контракт

### 3.1 Не смешивать каналы

- **Product output:** JSON, таблицы, отчёты, help и прогресс, являющиеся контрактом CLI. Нельзя считать любое использование `stdout` логом и перенаправлять его в `stderr`.
- **Operational diagnostics:** предупреждения и ошибки владельца операции; обычный канал — `stderr`. Один event имеет один семантический владелец.
- **Recorded evidence:** recording events, simulation telemetry, provenance, benchmark/check reports. Их schemas и timestamps принадлежат соответствующим продуктовым контрактам, а не общему logger.
- **Realtime state/error transport:** фиксированные флаги, коды и существующее отложенное сообщение. Это не место для форматирования operational log.

### 3.2 Уровни и lifecycle

| Level | Значение | Правило |
|---|---|---|
| `TRACE` | Пошаговые подробности алгоритма вне realtime | По умолчанию выключен; не включать ради «полного покрытия логами». |
| `DEBUG` | Диагностика выбранных ветвей, параметров и начала операции | По умолчанию выключен; дорогие поля готовятся только после проверки уровня. |
| `INFO` | Значимый завершённый пользовательский этап, штатная отмена | Не дублировать уже выдаваемый обязательный CLI result. |
| `WARN` | Операция продолжается с явно описанным ограничением или восстановлением | Не использовать для ошибки, после которой операция окончательно завершилась неуспешно. |
| `ERROR` | Операция не выполнена; отказ сообщается вызывающей стороне | Ожидаемый invalid input также может завершить CLI с `ERROR`, но без stack dump; неожиданная ошибка отличается кодом и cause. |

Lifecycle: `started` → ровно одно из `completed`, `failed`, `cancelled`. Эту последовательность применять только к операциям, для которых lifecycle logging действительно нужен; не добавлять по четыре сообщения на каждый getter, step, sample или writer. `cancelled` не означает успех и не меняет существующий exit code. Completion фиксируется после завершения требуемой операции, а не после её запуска.

### 3.3 Стабильные события и поля

- Event name: `<component>.<operation>.<outcome>` в нижнем регистре; одно имя — одна семантика. Не включать user input, файловый путь, индекс или текст exception в имя события.
- Message: короткая английская фраза, объясняющая событие; значения — отдельные типизированные поля, а не разбор prose регулярным выражением. Существующие externally asserted тексты сначала сохраняются контрактными тестами.
- Общие поля: `event`, `level`, `component`, `operation`; context только при наличии реального источника: `command`, `run_id`, `episode_id`, `case_index`, `worker_index`.
- `*_id` нельзя фабриковать ради одинакового набора полей. Не добавлять `request_id/session_id` в CLI/симуляцию, если таких сущностей нет. Индекс в плане и identity исполнения — разные поля.
- `duration_ms` — неотрицательная elapsed duration по monotonic clock; `simulation_time_s` — время модели; `size_bytes` — беззнаковое число байт. В уже существующих measurement/recording schemas поля не переименовываются этим правилом.
- Codes и identifiers не сериализуются то строкой, то числом под одним именем. Отсутствующее необязательное поле отсутствует; не подставлять `0`, пустую строку или выдуманное `unknown`, если это легальное значение.
- Если меняется значение события или размерность поля, вводить новое имя/версию контракта, а не тихо переиспользовать старое.
- Context передаётся явно на orchestration boundary; не использовать глобальное состояние для эпизода и не переносить C++ logging context через policy C ABI.

### 3.4 Exceptions, безопасность и стоимость

- Throwing layer добавляет необходимый предметный контекст, но не пишет тот же сбой в log. Окончательно обрабатывающий слой пишет одно terminal-событие.
- Primary operation failure и cleanup/finalize failure — разные события. Cleanup не заменяет первичную причину и не выдаёт ложное completion.
- Не добавлять stack trace для обычной валидации или ожидаемой отмены. Для неожиданной ошибки сохранять имеющиеся type/cause сведения; не обещать stack capture без реального поддержанного механизма.
- Не логировать raw argv/environment/config/plugin config, credentials, tokens, полные пользовательские документы и SQL result rows. Применять allowlist полей до форматирования; это профилактическое правило, не заявление об обнаруженной утечке.
- Динамические строки экранировать как данные, чтобы перевод строки/control bytes не создавали поддельные события. Форматная строка всегда константна; исключения и пути не являются format strings.
- Для выключенных уровней не строить message/serialized fields. Параметризованный вызов или явный `enabled(level)` guard, а не заранее собранная конкатенация.
- Не писать синхронные operational logs на каждый physics tick, callback, worker poll или sample. Сначала использовать существующий счётчик/сводку; новый sampling/rate-limit subsystem в эту работу не входит.
- Гарантия «ровно один раз» относится к владению событием в коде, а не к доставке на диск: отказ самого output sink нельзя превращать в рекурсивное логирование или изменение результата физической операции.

## 4. Общий порядок будущей проверки

**Ни одна из следующих команд в этой сессии не выполняется.** Даже `--collect-only` у `run_tests.sh native` проходит через configure/build preflight; это не разрешённая сейчас read-only проверка.

1. После отдельного разрешения на реализацию проверить дерево, выбранный build и наличие mandatory tools. Не маскировать неизвестную исходную ошибку, не менять текущие goldens для получения green.
2. Внутренний цикл semantic rename: сначала characterization test текущего поведения; затем compile-contract на новое имя, который должен перестать собираться по ожидаемой причине; затем declaration/definition/call-site rename; затем focused tests. Не придумывать runtime red для чисто лексического изменения.
3. Внутренний цикл diagnostic change: тест на один конкретный event/stream/exit/escaping контракт; убедиться, что red относится к нему; минимальное изменение; focused tests и valid-input control.
4. Вызывать отдельный CTest executable через его реальную регистрацию с `--no-tests=error --output-on-failure`. Состав проверок задаётся задачами ниже; missing binary не является разрешением пропустить тест.
5. Для анализа изменённого C++ использовать существующий `check_tidy`; для изменения tooling — соответствующие contract/negative controls. Не запускать все sweeps на каждом rename.
6. После стабилизации поставляемого среза — один полный covering gate. Чистый non-RT C++/tooling срез:

   ```bash
   bash tools/run_tests.sh native
   ```

   Срез, затрагивающий physics/realtime:

   ```bash
   bash tools/run_tests.sh full
   ```

   Сейчас `full` уже включает native preflight. Для нескольких задач, поставляемых одним срезом, focused проверки выполняются после каждой задачи, дорогой gate — один раз на окончательном коде среза. При последующих изменениях соответствующий gate требуется повторить.

7. Не ограничиваться legacy extension graph: новые общие `.cpp` и тесты регистрируются также в standalone graph, `build/native-release`; существующие `native_sources.json`/check-context механизмы должны видеть их как first-party. Root-only produced tooling проверяется дополнительно своими зарегистрированными тестами.
8. Для затронутого warm realtime ядра — существующие RTSan controls и warmed performance checks по `docs/TESTING.md` и platform guide; не вводить численный budget без исходного измерения. Для cold-path formatter отсутствие измерения latency не объявляется realtime-гарантией.
9. Final evidence содержит commit, фактический diff, artifact path, toolchain/platform, команды, counts, исключения и exit codes. Документированная baseline failure отмечается отдельно, не называется новым дефектом и не превращается в «gate прошёл».

**Обязательные инварианты при приёмке:** те же численные bits и порядок операций; те же внешние keys/schema/ABI; те же stdout payloads и exit codes; корректный порядок primary/cleanup failures; отсутствие новых hot-path allocations/I/O; отсутствие незаявленных изменений поведения.

### Тестовая дисциплина

Использовать `bt::require`, а не `assert`, который может исчезнуть в Release. Harness находится в `native/tests/support/migration_test.hpp:24–68`; `bt::invalid` принимает только `std::invalid_argument` или производный тип. Для другого контракта писать явную проверку нужного исключения, а не перехватывать любое исключение как успешный тест.

Численная проверка должна сохранять signed zero и NaN payload там, где они входят в контракт; существующий `bt::golden_f64` сравнивает binary64 representations. Не заменять её epsilon-сравнением из-за переименования. Текстовые checks различают обязательный product output и допускаемые изменения диагностического оформления.

Новые cold-path тесты регистрировать в общем `native/tests/migration/CMakeLists.txt`, который уже включён обоими build graphs. Не присваивать новому debt-тесту занятый migration task number. Например, новый специализированный тест получает имя `native_diagnostics_tests`, а не произвольное `migration_T64`.

```cmake
add_executable(native_diagnostics_tests ../test_diagnostics.cpp)
bike_startup_target_policy(native_diagnostics_tests "${_migration_repo_root}/native")
target_link_libraries(native_diagnostics_tests PRIVATE bike_migration_test bike_io)
if(BUILD_TESTING)
  add_test(NAME native_diagnostics_tests COMMAND native_diagnostics_tests)
endif()
```

Этот пример регистрации относится только к тесту общей cold-path диагностики, описанному в задачах ниже, и не требует MuJoCo/runtime зависимости.

## 5. Карта файлов и статические основания

| Файл/область | Ответственность в плане |
|---|---|
| `docs/agents/native-engineering.md:14–69` | Обязательные invariants и место для коротких согласованных naming/diagnostic правил при будущей реализации. |
| `migration/interface-contracts.md:22,50,155–232,363–367` | Namespace, wire/JSON, policy ABI и error contracts; не трактовать все исторические proposed snippets как текущую реализацию. |
| `native/include/bike/policy_v1.h:22–75` | Действительный C ABI: layout, callbacks `create/reset/act/destroy`, owned error buffer; не переименовывать и не расширять этим планом. |
| `native/.clang-tidy:6–16,42–44` | Существующая политика анализа; точечная защита naming не должна включать весь style tier. |
| `native/tests/support/migration_test.hpp:24–68` | Общие runtime assertions, typed exception checks и битовые goldens. |
| `native/tests/migration/CMakeLists.txt:754–807` | Общая регистрация CLI/recording/viewer/batch тестов T38–T51 и аргументов binaries. |
| `native/cmake/RecordingApplication.cmake:150–226` | CLI, batch, recording и research-sweep targets; один object target для общего `main.cpp`. |
| `CMakeLists.txt:294–311`, `native/CMakeLists.txt:242–267` | Оба определения `bike_io`/`bike_platform`; новый общий `.cpp` нельзя зарегистрировать только в одном graph. |
| `native/cmake/Checks.cmake:29–52,128–143` | Produced C++ check tools и `migration_T56`; инструменты сами входят в first-party проверки. |
| `tools/run_tests.sh:114–158` | Действительное выполнение preflight и выбор gate; в этой сессии файл только прочитан. |

Не вся разница в написании является долгом: vendor API, C ABI, wire keys, schema-defined identifiers, historical oracle spellings и разные категории времени требуют различий. Каждый rename должен назвать, какое именно понятие перестало быть неоднозначным.

### Подтверждённые точки logging-долга

Все выводы здесь — **source-supported**, без выполнения.

| Приоритет | Свидетельство | Что исправлять |
|---|---|---|
| P1 | `native/src/cli/ride.cpp:200–227`: вторичные preview/publication ошибки заменяются на «also failed» без причины; первая ошибка seal при уже имеющемся primary может исчезнуть после успешного retry. | Сохранение primary exception и operation-labelled secondary causes. Возможное маскирование при `close()` — риск для теста, не воспроизведённый дефект. |
| P1 | `native/src/cli/ride.cpp:175–194,201–226`: warning drain есть в headless loop, отсутствует в viewer advance и явном final flush. `native/src/viewer/ride.cpp:151–158` показывает отдельный telemetry-gated first-failure display. | Один владелец operational warning, независимый от HUD. Не утверждать потерю всех evidence: runtime snapshots/recording сохраняют failure. |
| P1 | `native/src/batch/sweep.cpp:60–62`: `.log` получает `result.out + result.err`; `native/src/batch/summary.cpp:11–15`: отсутствующий и неразбираемый tail становятся одинаковым null. | Сохранить принадлежность потоку; различать missing/malformed/valid summary в диагностике, не превращать exit 0 в failure молча. |
| P2 | `native/src/cli/application.cpp:24–31`, `native/src/cli/research.cpp:191–208`: разное оформление аналогичных human errors. `native/src/tools/validate_main.cpp:105–110` имеет intentional JSON stderr. | Общий formatter human diagnostics; JSON stderr, stdout results и exit mapping сохраняются. |
| P2 | `native/src/tools/check_frontends.cpp:320–336,455`: version-failure record намеренно теряется ради Python parity; `414–418,492–497`: alpha diagnostics печатаются сразу и ещё раз при mixed failure. | Отдельное изменение reporting contract, не механическая стилизация; сохранять tool-health/finding-status и report-only alpha. |
| P2 | `native/src/batch/summary.cpp:18–33`, `native/src/research/lifecycle.cpp:96–108`: raw argv/exception text входят в результаты/evidence. | Разграничить human diagnostics и identity-bearing artifacts; allowlist новых полей, escaping и ограничение раскрываемого текста. Утечка реальных credentials не установлена. |

Готовый механизм для сохранения причин — `research::FailureReport`, `native/src/research/lifecycle.hpp:18–24`, `lifecycle.cpp:33–42`. Это лучше повторного изобретения глобального exception logger. Runtime warning transport уже существует в `native/src/runtime/runtime.cpp:889–921`; engine callback transport не заменяется.

## 6. Задачи реализации

Задачи ниже выполняются только после отдельного согласования реализации. `C0` обязателен первым; naming-задачи и logging-задачи далее можно вести независимо, но изменения `batch/plan.hpp` последовательно интегрировать, а не редактировать конкурентно. Каждая задача заканчивается собственным focused review/test gate; полный covering gate — по §4.

### Task C0 — Зафиксировать словарь и границы совместимости

**Files:** modify `docs/agents/native-engineering.md`, `native/tests/migration/test_t47.cpp`, `native/tests/migration/test_t46.cpp`, `native/tests/migration/test_t51.cpp`; уточнять `migration/interface-contracts.md` только для действительно меняющегося contract. Текущий план служит исходным согласуемым словарём.

**Interfaces:** consumes существующие CLI/ABI/wire contracts; produces точный реестр `concept → C++ symbol → external spelling → owner → test → rename/exempt` и согласованный §2–3 контракт.

- [x] Добавить в engineering rules короткие нормы из §2–3, без копирования полного плана и без изменения полномочий агента/permission settings. → `docs/agents/native-engineering.md` «Naming and diagnostic consistency».
- [x] Для каждого подтверждённого rename из naming-задач ниже записать точные producer/consumer paths. Для оставшихся first-party каталогов отметить `reviewed/no change` либо конкретный последующий срез; generated/vendor trees исключить по происхождению, не по случайному имени. → `docs/superpowers/audits/2026-10-30-naming-logging-registry.md`.
- [x] В T46/T47/T51 расширить characterization-проверки существующих CLI: success output, invalid-input output, runtime failure, cancellation, machine-readable stderr. Проверять stdout и stderr отдельно и точный exit code; не считать все nonzero одинаковыми.
- [x] Зафиксировать external-sensitive различия: batch `name/index/out/exit/wall_s`, sweep `returncode`, measurement/recording clocks, policy C ABI и явно экспортируемые binding names. Их внутренние понятия могут быть едиными без смены внешних строк.
- [x] Будущая проверка: `ctest --test-dir build/native-release -R '^migration_T(46|47|51)$' --no-tests=error --output-on-failure`. До rename characterization должен проходить; неисправный baseline сначала объяснить. Baseline и extended checks — green (3/3).

**Done:** на каждый запланированный rename есть test/exemption; нельзя автоматически «исправить» `tyre`, `received_wall_s`, `id` или JSON key только потому, что другой слой использует иное написание.

### Task L1 — Ввести небольшой formatter human diagnostics и применить к CLI

**Files:** create `native/src/io/diagnostics.hpp`, `native/src/io/diagnostics.cpp`, `native/tests/test_diagnostics.cpp`; modify `CMakeLists.txt` и `native/CMakeLists.txt` в `bike_io`, `native/tests/migration/CMakeLists.txt`; modify `native/src/cli/application.cpp`, `native/src/cli/research.cpp`. Остальные emitters переходят на formatter в своих задачах/проверенных boundary-срезах, не через replace-all.

**Interfaces:** consumes `std::ostream`, уже существующие stream boundaries; produces следующие cold-path объявления. Это новые API, не заявления об их наличии сегодня.

```cpp
namespace bike::io {
enum class Severity { trace, debug, info, warning, error };
enum class DiagnosticKey {
    command, operation, error_code, field_path,
    episode_id, generation, job_id, job_index, duration_ms
};
using DiagnosticValue = std::variant<bool, std::int64_t, std::uint64_t,
                                     double, std::string_view>;
struct DiagnosticField {
    DiagnosticKey key;
    DiagnosticValue value;
};
bool is_enabled(Severity severity, Severity minimum) noexcept;
bool write_diagnostic(std::ostream& output, Severity severity,
                      std::string_view event, std::string_view message,
                      std::span<const DiagnosticField> fields = {}) noexcept;
}
```

- [x] Создать тест renderer до реализации. Нужные includes: `io/diagnostics.hpp`, `support/migration_test.hpp`, `<array>`, `<sstream>`, `<string_view>`. Контрольная часть внутри `bt::run`:

  ```cpp
  std::ostringstream output;
  const std::array fields{bike::io::DiagnosticField{
      bike::io::DiagnosticKey::episode_id, std::int64_t{2}}};
  bike::io::write_diagnostic(output, bike::io::Severity::warning,
      "physical.reference.warning", "invalid reference", fields);
  bt::require(output.str() ==
      "WARN physical.reference.warning: invalid reference episode_id=2\n",
      "canonical event, severity, field type and one line");
  ```

- [x] Добавить проверки escaping `\n`, `\r`, tab, quotes и control bytes; один логический event создаёт одну строку. Текст ограничить 4096 входными bytes с явной отметкой truncation; не разрывать валидный UTF-8 code point. Цифры форматировать locale-independent, строки полей — в кавычках с escaping, bool — `true/false`.
- [x] Проверить все пять level spellings и threshold ordering; дорогой supplier вызывается только внутри `if (is_enabled(...))`. Например счётчик supplier остаётся 0 при `debug/info`. Никакого глобального level state и новых user flags.
- [x] Реализовать formatter без хранения callbacks, background threads, locks и глобального mutable state. `true` означает успешную запись в stream; stream failbit/exception или ошибка форматирования дают `false` без повторного log и без изменения primary exception. Весь throwing formatting/write находится внутри catch boundary; именно это обосновывает `noexcept`. Event names — compile-time литералы в местах вызова, не user strings. Допустимые `DiagnosticKey` образуют allowlist; типы полей: IDs/index/generation — integer, `duration_ms` — double, command/operation/error_code/field_path/job_id — string.
- [x] Зарегистрировать реализацию в обоих `bike_io`, тест — по §4. Только затем провести red→green теста; initial red обязан быть missing API/реализация или конкретное несовпадение output, не ошибка dependencies.
- [x] Мигрировать human error/warning emitters dispatcher/research на format `LEVEL event: message key=value`. Это **намеренное изменение human diagnostic оформления**; заранее защищённые exception type, field-path fragments, stdout и exit codes сохранить. JSON stderr `bike-validate` и machine error envelopes не пропускать через human formatter.
- [x] На границе недоверенного plugin/SQL/config текста не передавать в message весь raw payload: использовать фиксированное описание, `error_code` и безопасный field identifier. В тесте synthetic plugin error с `token=secret-sentinel` не должен появиться в human output; bad-column diagnostic по-прежнему идентифицирует колонку. Original exception/evidence не переписывать.
- [x] Проверить один human error и один WARN во всех переводимых командах; отсутствие дополнительной записи в normal success; newline-injection input; stream failure не вызывает повторного логирования. Будущий focused gate: `native_diagnostics_tests`, `migration_T47`; selected `check_tidy`.

**Done:** одинаковое оформление новых human events, typed context, safe formatting и одна запись на событие. Не заявлять весь проект migrated, пока не закрыта матрица emitter owners из L5/G1.

### Task L2 — Сохранить primary и secondary ошибки physical finalization

**Files:** modify `native/src/cli/ride.cpp:172–227`, `native/src/cli/physical_capture.hpp`, `physical_capture.cpp`, `native/tests/migration/test_t47.cpp`, `native/tests/integration/test_async_close.cpp`, `native/tests/reference/test_research_recording_lifecycle.cpp`. `physical_capture` уже входит в `bike_cli`; новая библиотека/production environment fault switches не нужны.

**Interfaces:** consumes неизменённые `research::FailureReport::add(operation, exception_ptr) noexcept`, `rethrow_if_failed()` и L1 formatter; produces сохранённое первое исключение и вторичные причины с различимыми operations `seal`, `preview_close`, `recording_publish`, `plot_publish`, `runtime_close`. Для проверки реального finalization order выделить из `ride.cpp` в существующий `physical_capture.cpp` только последовательность вызовов/сбора ошибок:

```cpp
namespace bike::cli {
struct PhysicalFinalizationActions {
    std::function<void()> seal;
    std::function<void()> preview_close;
    std::function<void()> recording_publish;
    std::function<void()> plot_publish;
    std::function<void()> runtime_close;
};
research::FailureReport finalize_physical_run(
    const PhysicalFinalizationActions& actions,
    std::exception_ptr primary = {});
}
```

`seal` и `runtime_close` обязательны у production caller; пустые optional actions обозначают отсутствие preview/publication/plots. Условия `capture.enabled() && sealed` и `publication` остаются внутри соответствующих захваченных actions. Helper не логирует и не меняет physics; порядок seal (максимум одна повторная попытка), preview, recording, plots, close сохранён. В header явно включить `<functional>`, `<exception>` и owning `research/lifecycle.hpp`.

- [x] Добавить fail/control cases в T47 для этого seam: primary simulation failure + preview failure; primary + first seal failure + successful seal retry; primary + publication failure; один cleanup failure без primary; successful run. Проверять сохранённый exception_ptr, обе secondary причины и вызов close даже при предыдущих ошибках. Исходные primary type/message и число diagnostic events отдельно проверяются на CLI boundary.
- [x] Проверить базовый preservation contract непосредственно через существующий `FailureReport`:

  ```cpp
  bike::research::FailureReport failures;
  const auto primary = std::make_exception_ptr(std::invalid_argument("primary field"));
  failures.add("simulation", primary);
  failures.add("preview_close", std::make_exception_ptr(std::runtime_error("disk full")));
  bt::require(failures.primary == primary, "primary identity is preserved");
  bt::require(failures.secondary == std::vector<std::string>{"preview_close: disk full"},
              "secondary keeps operation and cause");
  bt::invalid([&] { failures.rethrow_if_failed(); });
  ```

- [x] Проверить выделенный seam до подключения production closures:

  ```cpp
  int seals = 0;
  bool closed = false;
  bike::cli::PhysicalFinalizationActions actions;
  actions.seal = [&] { if (++seals == 1) throw std::runtime_error("seal first"); };
  actions.preview_close = [] { throw std::runtime_error("preview full"); };
  actions.runtime_close = [&] { closed = true; };
  const auto primary = std::make_exception_ptr(std::invalid_argument("simulation"));
  const auto report = bike::cli::finalize_physical_run(actions, primary);
  bt::require(report.primary == primary && seals == 2 && closed,
              "retry only seal and always close without replacing primary");
  bt::require(report.secondary == std::vector<std::string>{
      "seal: seal first", "preview_close: preview full"},
      "successful seal retry does not discard the first failed attempt");
  ```

- [x] Заменить локальный finalization tail вызовом `finalize_physical_run(actions, failure)` и использовать возвращённый report, не перенося весь research lifecycle. В каждом catch сохранять cause до следующего cleanup. Повторяется только допустимый seal над committed prefix, никогда physics advance.
- [x] Обернуть explicit `runtime->close()` в тот же порядок сбора ошибок; primary не заменяется ошибкой закрытия. Не добавлять throwing logging в destructor.
- [x] Secondary events выводит physical command owner; primary он rethrows, а dispatcher выводит один раз. Не печатать primary здесь и повторно наверху. Synthetic non-standard exception не должен исчезать или превращаться в success.
- [x] Провести существующие `reference.async_close`, `reference.native_research_recording_lifecycle` и physical CLI controls, затем covering `full` для поставляемого physics/lifecycle среза.

**Done:** generic «also failed» заменены полезными причинами; invariants и exit mapping остаются прежними. Не утверждать, что masking через close был исходным воспроизведённым багом.

### Task L3 — Единый владелец pending warnings независимо от viewer HUD

**Files:** modify `native/src/cli/ride.cpp:157–203`, `native/src/cli/research.cpp:68–110` и finalization warning delivery в том же файле; tests `native/tests/reference/test_native_runtime_accounting.cpp`, `native/tests/migration/test_t48.cpp`, `native/tests/reference/test_research_recording_lifecycle.cpp`. `native/src/runtime/runtime.cpp:889–921` и `native/src/viewer/ride.cpp:151–158` — проверяемые контракты, не обязательные цели редактирования.

**Interfaces:** consumes `take_pending_warning() -> optional<string>`; produces `physical.reference.warning` / `research.reference.warning` у command owner, без новой runtime queue.

- [x] Создать controls: warning после advance; warning только при trailing flush/seal; reset outgoing generation; viewer telemetry off; повторный drain без новых warnings. Проверять diagnostics отдельно от HUD/recorded failure.
- [x] В physical CLI завести одну local cold-path функцию drain→L1 formatter и вызывать её после завершённого advance в headless и viewer closures, после seal/flush, перед сменой episode при reset и перед close. Обработать также failure exit из advance/seal: первичная ошибка сохраняется по L2.
- [x] Drain ограничен семантикой существующего источника: последовательно забрать уже pending warnings до empty, не добавляя physics advance. Не приписывать outgoing warning новому episode/generation: context захватывается до reset.
- [x] Для research завершить аналогичное покрытие stop/drain/finalization; существующие after-advance warning callbacks не дублировать.
- [x] `[FIRST FAILURE]` в HUD считать presentation evidence, а не вторым operational event. Не переносить весь HUD на stderr и не добавлять вторую дедупликацию по тексту там, где pending-warning source уже once-only.
- [x] Future focused tests: `reference.native_runtime_accounting`, `reference.native_research_recording_lifecycle`, `migration_T48`. Для viewer paths при последующей реализации потребуется настоящий GUI smoke с telemetry on/off, не только fake window; в этой сессии viewer не запускать.

**Done:** warning доставляется при headless/viewer/finalization с одинаковой operational семантикой; отключение HUD не отключает warning; numerical bits и момент accounting commit прежние.

### Task L4 — Разделить worker streams и статус извлечения summary

**Files:** modify `native/src/batch/plan.hpp:22–28,42`, `native/src/batch/summary.cpp:11–35`, `native/src/batch/process_pool.cpp`, `native/src/batch/sweep.cpp:60–70`, `native/src/cli/batch.cpp`, `native/tests/migration/test_t51.cpp`, `native/tests/reference/test_research_sweep.cpp`, `native/tests/integration/test_batch.cpp`; читать `native/src/batch/log.hpp` для existing exclusive/no-follow writes.

**Interfaces:** после naming-задачи N1 использовать её `stdout_text/stderr_text/wall_duration_s`. Новое внутреннее API:

```cpp
namespace bike::batch {
enum class SummaryStatus { missing, malformed, valid };
struct ParsedSummary {
    SummaryStatus status = SummaryStatus::missing;
    io::Value value;
};
[[nodiscard]] ParsedSummary parse_stdout_summary(std::string_view stdout_text);
}
```

- [x] Добавить tests parser до замены `tail_json`:

  ```cpp
  bt::require(bike::batch::parse_stdout_summary("progress\n").status ==
      bike::batch::SummaryStatus::missing, "no summary candidate");
  bt::require(bike::batch::parse_stdout_summary("progress\n{broken}\n").status ==
      bike::batch::SummaryStatus::malformed, "candidate was rejected");
  const auto parsed = bike::batch::parse_stdout_summary("progress\n{\"outcome\":\"ok\"}\n");
  bt::require(parsed.status == bike::batch::SummaryStatus::valid &&
      bike::io::canonical_json(parsed.value) == "{\"outcome\":\"ok\"}",
      "terminal summary preserves exact parsed value");
  ```

- [x] Сохранить текущую terminal-tail selection: candidate — строка, начинающаяся `{`, и остаток потока. Valid JSON + посторонний trailing text — malformed; не искать более ранний «удобный» JSON. Exit 0 без summary не становится ошибкой только из-за отсутствия JSON.
- [x] Перенести parse status во внутренний Result отдельным полем `summary_status`; `summary` остаётся `io::Value`, чтобы `result_record` сохранял внешние keys. Для malformed ожидаемого research summary — одно warning событие `batch.summary.malformed`, не повтор raw stdout. Missing необязательного summary не вызывает WARN storm.
- [x] Для sweep заменить combined `.log` на `<job_id>.stdout.log` и `<job_id>.stderr.log`, используя существующий exclusive/no-follow `write_log`. Это явная миграция artifact layout: обновить реальных читателей и тесты в том же срезе; не обещать восстановление временного interleaving из двух готовых строк и не заводить вторую копию combined log «на всякий случай».
- [x] Проверить streams без newline (`stdout="OUT"`, `stderr="ERR"`): два raw artifact содержат ровно соответствующие bytes. Дополнительно проверить Unicode stderr tail, empty streams, spawn failure, serial/parallel order, cancellation. Сохранить `sweep.json` keys `returncode/wall_s` и batch `exit/name/index/out`.
- [x] Future gate: `migration_T51`, `reference.native_research_sweep`, `reference.batch`; root-only reference tests проверять в standalone graph, не считать их автоматически выполненными legacy gate.

**Done:** поток и parse status больше не теряются, а бизнес-результат процесса не переопределяется диагностическим formatter.

### Task L5 — Исправить ownership диагностик native-check tools

**Files:** modify `native/src/tools/check_frontends.cpp:320–336,414–418,455,492–526`, `native/tests/migration/migration_T56_check.cmake`; читать `native/src/tools/check_reports.cpp`, `native/src/tools/checks_internal.hpp`, `tests/reference/test_native_check_tools.py:488–580` как reporting contract/reference. Для оставшихся executable emitters: `native/src/cli/recording.cpp`, `native/src/cli/replay.cpp`, `native/src/tools/validate_main.cpp` — прежде всего compatibility controls, не безусловная миграция JSON output.

**Interfaces:** consumes текущие summary `records`, `tool_health`, `finding_status`, `status`; produces сохранённый `pass="version"` record и single-owner console report, без смены meaning существующих полей.

- [x] Создать fixture failed version probe + clean analysis. Final JSON должен содержать один version record, `tool_health="failed"`, `finding_status="none"`, nonzero exit. Raw stdout/stderr log paths остаются доступными.
- [x] Fixture alpha-warning-only сохраняет report-only и pass; fixture alpha-warning + отдельный stable failure печатает уникальный marker alpha diagnostic ровно один раз. Проверять health отдельно от findings.
- [x] Добавлять version record в итоговый локальный `records`, а не во временный `summary.records`, перезаписываемый позже. Удалить комментарий о намеренной потере **только после** согласования этой явной divergence от historical Python reporting.
- [x] Выбрать один момент console reporting: после формирования всех records. Не выводить alpha finding одновременно в loop и финальном failure listing; alpha-only run всё равно выводит report-only findings. Не менять suppression accounting и условия failure.
- [x] В матрице executable outputs зафиксировать exception types, stable error fragments, stdout/result schemas, JSON stderr и exit codes. Переводить только human prose на L1; все schema-bearing outputs оставлять у их владельцев. Native check JSON и raw compiler diagnostics — evidence, не произвольные human строки для redact/переформатирования.
- [x] Future gate: standalone `migration_T56`, существующие negative controls и `native`. Не считать Python tests самостоятельным доказательством работы C++ tool. `migration_T56` passed (1375.9 s).

**Done:** причина tool-health failure присутствует в summary, alpha finding не дублируется; неизменны report-only policy и процессные exit codes. Если parity-policy запрещает исправление historical drop, этот конкретный subtask остаётся явно deferred, а не помечается выполненным.

### Подтверждённые точки naming-долга

| Приоритет | Точное изменение | Основание / исключение |
|---|---|---|
| P1 | `BrakeWriter::accumulate_validated`, `RiderForcesWriter::accumulate_validated` → `compute_validated_into` | `writers/brake.cpp:198–204` присваивает оба элемента; `writers/rider_forces.cpp:125–160` очищает buffer и присваивает forces. Это overwrite, не accumulate. Tyre kernels не переименовываются автоматически. |
| P1 | `io::read_numpy_bytes` → `read_bounded_file_bytes` | `io/npy.cpp:143–149` — bounded raw read; `cli/batch.cpp:25` читает TOML, `golden/episode.cpp:379` — MJB. Это не NumPy decoding. |
| P1 | `AdvancementGuard` → `RuntimeOperationGuard`, `advancing_` → `busy_` | `runtime/runtime.cpp:72–88,808–890` исключает также snapshot/render/image/state calls. Atomic behavior и error text остаются прежними. |
| P1 | `ResearchCommand/ResearchWindow.front/rear` → `front_brake_demand/rear_brake_demand`; `seen_demand` → `observed_demand_nm`; `ResearchEpisode.commands` → `pending_delayed_controls` | `runtime/research.hpp:43–82`, `research.cpp:150–174,316–332`: brake fractions, наблюдаемый torque demand и очередь отложенных controls, не история всех команд. |
| P2 | `batch::performance_cores()` → `default_worker_count()` | `batch/process_pool.cpp:18–28`: Apple performance cores, Linux affinity CPUs или hardware concurrency. Обещание конкретной аппаратной категории неверно вне Apple. |
| P2 | `ProcessResult/Result.out/err` → `stdout_text/stderr_text`; `batch::Result.wall_s` → `wall_duration_s` | `platform/process.hpp:29–33`, `batch/plan.hpp:22–28`, `batch/process_pool.cpp:59–66`: одинаковые streams и measured monotonic duration; не filesystem `out`. Wire keys остаются прежними. |
| P2 | `simplified_` / `simplified()` → `uses_ideal_transmission_` / `uses_ideal_transmission()` | `writers/drivetrain.cpp:82`; допустимые модели в `config_validation.hpp:192–193` — `elastic_chain`, `ideal_mid_drive`, `geometric_ideal_mid_drive`. Flag — семья модели, не сложность реализации. Полярность не инвертируется. |
| P2 | `TireSnapshot.wheel_axis_m` → `reference_position_world_m` | `runtime/native_contacts.cpp:104–117` пишет position из `geom_xpos` для front/rear/**handlebar**. Поэтому предложенное при первичном аудите `hub_position_world_m` отвергнуто для общего snapshot. Реальный pneumatic hub сохраняет `hub_position_world_m`; wire `wheel_axis_m` не меняется. |

### Task N1 — Уточнить subprocess/batch имена без смены схем

**Files:** modify `native/src/platform/process.hpp`, `process.cpp`; `native/src/batch/plan.hpp`, `process_pool.cpp`, `summary.cpp`, `sweep.cpp`; `native/src/cli/batch.cpp`; consumers `native/src/tools/check_contracts.cpp`, `check_reports.cpp`, `python_audit_main.cpp`, `native/src/runtime/realtime_provenance.cpp`; tests `native/tests/migration/test_t06.cpp`, `test_t36.cpp`, `test_t46.cpp`, `test_t47.cpp`, `test_t51.cpp`, `native/tests/integration/test_batch.cpp`, `test_replay_playback.cpp`, и reference consumers перечисленные ниже.

Reference consumer set для type-aware проверки: `test_research_sweep.cpp`, `test_research_recording_lifecycle.cpp`, `test_ride_backend_cli.cpp`, `test_recording_analyst_cli.cpp`, `test_pedelec_replay_contract.cpp`, `test_native_state_bundle.cpp`, `test_native_engine_errors.cpp`, `test_native_benchmark.cpp`, `test_realtime_gate.cpp` в `native/tests/reference`. Совпадения `.out` в чужих типах, например `writers/rider_contacts.cpp`, не являются разрешением их заменить.

**Interfaces:** `platform::ProcessResult { int exit_code; string stdout_text; string stderr_text; }`; batch Result использует те же stream names и `double wall_duration_s`; `unsigned batch::default_worker_count()` с неизменённой платформенной политикой.

- [x] В T06/T51 и integration batch зафиксировать exact stdout/stderr bytes, serial/parallel order, cancellation и bypass default-discovery при explicit `--jobs`.
- [x] Выполнить type-aware rename declarations, definitions и всех действительных consumers. Не заменять JSON keys, локальные `out` других типов и paths. `Job.id/index` сохранить и документировать область identity вместо изобретения UUID или глобального run ID.
- [x] Изменить worker discovery symbol, не меняя Apple sysctl key, Linux affinity, fallback и bound `>=1`. Не объединять отдельную sweep default policy с batch policy в naming-срезе.
- [x] Сохранить serializer mapping буквально: `"wall_s"` ← `wall_duration_s`, `"out"` ← filesystem path, `"name"` ← Job.id, `"exit"` ← exit_code; sweep `"returncode"` остаётся `returncode`.
- [x] Future focused gate: `migration_T06`, `migration_T51`, `reference.batch`, `reference.native_research_sweep`; compiler reference search не оставляет старые members у двух переименованных типов.

**Done:** потоки невозможно спутать с output directory; имя worker default честно описывает платформы; внешние serialized результаты совпадают. L4 начинать только после интеграции N1.

### Task N2 — Переименовать bounded reader, не перемещая IO-модули

**Files:** modify `native/src/io/npy.hpp:34`, `npy.cpp:143–149`; production consumers `native/src/{cli/physical_capture.cpp,cli/batch.cpp,cli/ride.cpp,replay/legacy.cpp,research/configuration.cpp,research/metadata.cpp,initialization/request.cpp,initialization/state_file.cpp,golden/episode.cpp,validation/common.hpp,runtime/realtime_provenance.cpp,runtime/realtime_command.cpp,recording/analysis_detail.hpp}`.

**Tests:** `native/tests/reference/{replay_reference_support.hpp,tyre_reference_support.hpp,test_native_pneumatic_wire.cpp,test_native_pneumatic_tire.cpp,test_golden_episode_regressions.cpp,test_golden_episode_codec.cpp,test_measure_realtime.cpp}`; existing T06 filesystem safety controls. Этот список — 22 найденных files вместе с declaration/definition и consumers, не число отдельных нарушений.

**Interfaces:**

```cpp
namespace bike::io {
[[nodiscard]] std::vector<std::byte>
read_bounded_file_bytes(const std::filesystem::path& file);
}
```

- [x] Сохранить существующий plain-text контроль `test_nonempty_output_is_rejected_before_setup` в `test_measure_realtime.cpp:64–74` и bounded/no-follow/truncation contracts. Для нового публичного имени добавить compile-contract:

  ```cpp
  using FileReader = std::vector<std::byte> (*)(const std::filesystem::path&);
  static_assert(std::is_same_v<decltype(&bike::io::read_bounded_file_bytes), FileReader>);
  ```

  static_assert добавлен в `test_measure_realtime.cpp`; до rename — red `no member named 'read_bounded_file_bytes'`, после — green.
- [x] Провести declaration/definition/call-site rename. Не менять тело read loop, error types/text, `kMaxNumpyBytes` = 128 MiB, EINTR handling, extra-byte growth check и module placement/linkage. `parse_npy` / `encode_npy` остаются NumPy-specific.
- [x] Удалить старое имя полностью, без forwarding alias. Новое имя не означает unbounded/общесистемный reader; лимит остаётся в API contract. `grep read_numpy_bytes native/` → 0.
- [x] Future focused gate: `reference.measure_realtime`, `reference.golden_episode_codec`, `migration_T51` плюс затронутые pneumatic/replay reference targets; broad native build ловит остальные consumers. Cold-path rename сам по себе не требует изменения physics. 10/12 pass; `reference.golden_episode_tool` и `reference.native_pneumatic_tire` — pre-existing red на WIP (`native/build/Testing/Temporary/LastTestsFailed.log`, 2026-10-29 21:07).

**Done:** имя отражает фактическую операцию; bytes и исключения прежние; не появилась новая dependency edge `platform → numpy` или перенос большого IO-модуля.

### Task N3 — Исправить имена overwrite kernels

**Files:** modify `native/src/writers/brake.hpp:71`, `brake.cpp`, `rider_forces.hpp:89`, `rider_forces.cpp`; tests `native/tests/reference/test_native_brake_resistance.cpp`, `test_native_rider_forces.cpp`.

**Interfaces:** private helper `compute_validated_into` сохраняет каждую существующую signature/qualifier; публичные `compute_into`, `try_compute_into`, `torques`, `qfrc`, `apply` не меняются.

- [x] В existing tests заполнить caller output ненулевыми sentinel values перед двумя одинаковыми `compute_into`. Оба результата должны побитово совпасть с существующим golden, а не зависеть от предыдущего содержимого buffer.
- [x] Убедиться, что тест проверяет именно overwrite, включая нулевые untouched DOFs rider-forces и порядок front-before-rear brake. Никаких epsilon и изменения oracle values.
- [x] Переименовать два private helpers и их реальные вызовы. Сохранить `fill(out,0.0)`, все `=` и порядок вычислений. Устаревший комментарий про accumulate убрать, а не добавлять объяснение очевидного имени.
- [x] Не пытаться сделать функцию суммирующей, чтобы оправдать старое имя. Не расширять rename на `tire/distributed_tire/pneumatic_tire::accumulate_validated`: у них отдельно проверяется семантика.
- [x] Future focused gate: `reference.native_brake_resistance`, `reference.native_rider_forces`; финальный `full` для physics среза. Private-only rename не требует искусственного failing API test, его безопасность дают characterization и bitwise tests.

**Done:** операция названа по реальной семантике; приватность, ABI, порядок arithmetic и output buffers не изменились.

### Task N4 — Уточнить runtime exclusion и research control vocabulary

Это два маленьких независимых review-среза; не смешивать их с L2/L3 delivery changes.

**Files (N4a):** `native/src/runtime/runtime.hpp:232`, `runtime.cpp:69–88` и все guarded calls; test `native/tests/reference/test_native_runtime_commands.cpp:100–145`.

**Interfaces (N4a):** private `mutable std::atomic<bool> busy_{false}` и local RAII `RuntimeOperationGuard`; public methods неизменны.

- [x] Characterization: `test_concurrent_advance_and_reset_rejected` проверяет также snapshot/probe/close; добавить недостающий control только если он реально отсутствует, не переписывать synchronization harness.
- [x] Переименовать class, ctor/dtor/deleted specials, field и все acquisitions; memory ordering, guard extent и exception type не менять. Сохранить externally asserted `"advance already in progress on this runtime"`.
- [x] Заменить неверный scope-комментарий одной WHY-строкой про отказ до наблюдения частично изменённого состояния. Future gate: `reference.native_runtime_commands`, `check_tidy`, final `full`.

**Files (N4b):** `native/src/runtime/research.hpp:43–82`, `research.cpp`, `research_binding.cpp:269–273`; tests `native/tests/reference/test_native_research_environment.cpp`.

**Interfaces (N4b):**

```cpp
struct ResearchWindow {
    RideControl control;
    double front_brake_demand = 0., rear_brake_demand = 0.;
    std::int64_t start_step = 0, target_step = 0;
    std::optional<double> observed_demand_nm;
};
```

У `ResearchCommand` те же два brake member names; у `ResearchEpisode` очередь имеет тип `std::deque<std::pair<std::int64_t, RideControl>> pending_delayed_controls`.

- [x] Зафиксировать tests `test_invalid_boundary_calls_do_not_change_state` и `test_zero_budget_retains_single_window_and_no_tail_flush`: rejected calls не меняют state, zero budget не создаёт дополнительное window/flush.
- [x] Переименовать fields и одноимённые domain parameters. Не менять queue ordering, `(start + delay_steps)`, структуру pair, applied/requested histories, scale brake fractions или torque units.
- [x] Binding keyword strings `front_brake_demand/rear_brake_demand` уже каноничны — оставить. Requested/applied wire encoding сохраняется byte/key-wise, не унифицируется без доказательства равной семантики.
- [x] Future gate: `reference.native_research_environment`, native binding controls, final `full`.

**Done:** имена раскрывают reentrancy domain, единицу demand и назначение очереди; синхронизация и policy delay не изменены.

### Task N5 — Уточнить transmission predicate и contact position

Это два отдельных P2-среза после N3/N4, не условие завершения L1–L5.

**Files (N5a):** `native/src/writers/drivetrain.hpp:374,405`, `drivetrain.cpp`; test `native/tests/reference/test_native_drivetrain.cpp`; читать `native/src/config_validation.hpp:192–196` и `writer_config.hpp:1080–1082` для разрешённого domain.

- [x] Truth table: `elastic_chain → false`, `ideal_mid_drive → true`, `geometric_ideal_mid_drive → true`. Проверить оба existing force staging/solved interval tests.
- [x] `simplified_` → `uses_ideal_transmission_`, accessor → `uses_ideal_transmission()`. Сохранить initializer `transmission_model != "elastic_chain"` и полярность каждого branch. Не менять transmission-model strings или computational family.
- [x] Future gate: `reference.native_drivetrain`; final `full`.

**Files (N5b):** `native/src/writers/tire_common.hpp:51`, `tire.cpp`, `distributed_tire.cpp`, `pneumatic_tire.cpp`, `native/src/runtime/native_contacts.cpp`, `step.cpp`, `native/src/legacy/controls.cpp`, `native/src/validation/runtime_access.cpp`; test `native/tests/reference/test_native_encode_columns.cpp:1913` и native contact controls.

**Interface (N5b):** `TireSnapshot::reference_position_world_m` — `std::array<double,3>`; serialization:

```cpp
wire_set(side, "wheel_axis_m", wire_vec3(s.reference_position_world_m));
```

- [x] Проверить позицию front/rear и handlebar относительно `geom_xpos`, а не только unit direction и не только tyre hub. Существующая `wheel_axis_moment_nm` — moment, это другое поле, не substring rename.
- [x] Переименовать только C++ member и обращения; не менять `pneumatic::...hub_position_world_m`, который действительно обозначает hub, или локальные математические delta/axis в векторном расчёте.
- [x] В `step.cpp` оставить wire key `wheel_axis_m`; не трогать `recording/catalog_data.inc`, `research_metrics.cpp` string lookups и golden schema. Проверить `test_struct_list_patches_channel_tires` на неизменённый encoded key/value.
- [x] Future gate: `reference.native_encode_columns`, `reference.native_tire`, `reference.native_distributed_tire`, `reference.native_pneumatic_tire`, `native_contract_tests`; final `full`.

**Done:** bool и spatial field точно обозначают модель и координату; нет невольного rename чужого contact object в wheel hub.

### Task G1 — Завершить покрытие и поставить защиту от возврата долга

**Files:** agreed rules в `docs/agents/native-engineering.md`, тесты задач выше; `native/.clang-tidy` по умолчанию не меняется. Новую CI pipeline или собственный regex-C++ parser не создавать.

- [x] Пройти first-party source inventory целиком и для каждого naming/logging пункта §7 подтвердить `fixed`, `already consistent`, `contract exemption` или отдельную незавершённую задачу с причиной. Не удалять неудобные файлы из области учёта. `docs/superpowers/audits/2026-10-30-naming-logging-registry.md` — все items классифицированы + diagnostic-ownership accounting.
- [x] Через semantic references подтвердить отсутствие старых private/internal symbols из N1–N5. Остатки старых имён в wire keys, public diagnostic fragments и historical evidence явно ожидаемы. Regex count не доказывает корректность rename. Old symbols (`read_numpy_bytes`, writers-`accumulate_validated`, `AdvancementGuard`/`advancing_`, `ResearchCommand.front/rear`, `seen_demand`, `ResearchEpisode.commands`, `simplified_`, `TireSnapshot::wheel_axis_m`) — zero hits вне wire-key/contract spellings и local test structs.
- [x] В review checklist закрепить семантику verbs, flags, collections, units/clocks, exact event names/field types, владельца terminal error и запрещённые hot-path sinks. Автоматически проверяются formatter output, schema stability, public signatures и regression cases; доменную синонимию не притворяться решённой одним style linter. Rules — `docs/agents/native-engineering.md` (C0 section); automated — `native_diagnostics_tests` (formatter), contract tests (schema/signatures), T46/T47/T51 (characterization).
- [x] Если нужен AST naming checker, вынести узкий `readability-identifier-naming` rollout в отдельный согласованный срез после baseline: не включать его здесь вслепую и не создавать массовые suppressions. Это не блокирует исправление конкретного долга. Не включён — отложено per spec; `.clang-tidy` defaults не менялись.
- [x] Провести fresh covering gates по §4 на финальных artifact обеих поддерживаемых платформ; root `reference.*` и tooling tests запускать явно. Не подменять их legacy CTest. Все 9 sweeps green in-gate (frontends/tidy 336 TUs/analyzer/cppcheck/odr/scripts/headers 9565 TUs/diagnostic-controls 6/6/context), contracts passed; CTest 61/63 — `migration_T09`/`migration_T10` red = точное pre-session множество (`native/build/Testing/Temporary/LastTestsFailed.log`, 2026-10-29 21:07): physics-oracle drift на WIP, identifier-only rename затронуть не может. Pytest `test_native_*.py` + `test_golden_episode*.py`: 3144 passed / 41 failed / 1 skipped — все 41 failures присутствуют в Oct-29 `.pytest_cache` baseline, zero new failures. `run_tests.sh native` останавливается на ctest (exit 8) из-за этих documented reds — per §final-evidence они отмечены baseline-ом, не новыми дефектами. Linux aarch64 — deferred per plan.
- [x] Отдельно проверить неизменённые help/result output, batch/sweep final JSON, JSON stderr validator, replay mismatch report, cancellation exit codes и отсутствие duplicate primary errors. Help text — `cli/arguments.cpp` unmodified, `bike-ride --help` identical; batch/sweep JSON — T51 + `native_research_sweep` green; stderr JSON validator — T46/T47 green; cancellation — `platform::run` stop-token check в T06 green; no duplicate primary — `finalize_physical_run` seam + T47/L2 checks green; replay mismatch — `test_native_replay_*` + `test_native_platform_replay` green в pytest.
- [ ] Закрывать checkbox только по фактическому evidence; после завершения согласовать интеграцию. Этот план не разрешает автоматический commit/push.

**Done:** принятые срезы завершены, каждый exception имеет основание, будущий review способен отличить смысловое нарушение от intentional contract spelling.

## 7. Покрытие требований пользователя

| Требование | Где закрывается |
|---|---|
| Naming: synonyms | §2, C0, N2/N3; разные операции read/parse/load не сливаются. |
| Naming: domain terminology | C0, N4/N5; transmission, delayed control, reference position имеют точный смысл. |
| Naming: method verbs | §2, N2/N3/N4, G1; имя отражает side effects и missing-result contract. |
| Naming: opposite operations | §2, C0/G1; start/stop, open/close; existing ABI/STL pairs — исключения. |
| Naming: class roles/suffixes | §2, C0/G1; массовое переименование классов не обосновано аудитом. |
| Naming: variables/parameters | N1/N4/N5. |
| Naming: booleans, positive/negative meaning | §2, N4a/N5a; конфигурационное use/enabled не смешивается с predicate. |
| Naming: collections | N4b `pending_delayed_controls`; §2 и G1 для остальных контейнеров. |
| Naming: abbreviations/acronyms | §2, N1/N4; vendor/численные сокращения сохраняются. |
| Naming: id/key/name/reference/index | C0/N1/N5b; область identity фиксируется, а не переименовывается наугад. |
| Naming: explicit units | N1/N4b/N5b; logging §3.3; clocks — отдельные понятия. |
| Naming: ambiguous names | Все N-задачи; generic Value и математические locals не считаются дефектом по одному слову. |
| Logging: levels | §3.2, L1/L5. |
| Logging: message wording | §3.3, L1/L2/L3/L5, G1. |
| Logging: event naming | §3.3, L1/L3/L4; стабильные event literals вместо динамических user strings. |
| Logging: structured fields и field types | §3.3, L1/L4; existing report schemas остаются отдельными. |
| Logging: context propagation | §3.3, L1/L3/L4; existing episode/job identifiers без вымышленных session IDs. |
| Logging: lifecycle start/end/failure/cancellation | §3.2, L2/L3/L4; не логировать каждую внутреннюю функцию. |
| Logging: exceptions/causes/expected failures | §3.4, L1/L2/L5; stack traces не добавляются без поддержанного механизма. |
| Logging: duplicates | L2/L3/L5. |
| Logging: performance measurements/units | §3.3–3.4, N1, L4; report units/schema сохраняются. |
| Logging: noise/high-frequency events | §3.2–3.4, L3/L5, G1; no hot-loop stream logging. |
| Logging: sensitive data | §3.4, L1/L4; raw evidence не редактируется как human message. |
| Logging: parameterized/lazy formatting | L1 typed fields + explicit level guard. |
| Logging: comparable granularity | L2/L3/L4: phase/cause/context вместо generic «also failed». |
| Logging: stable semantics | C0, §3.3, L4/L5 compatibility controls, G1. |

## 8. Рекомендуемая очередность и критерий согласования

1. **C0 → N2 → N3 → N4a:** самые ясные смысловые исправления; каждый самостоятельный rename-only срез.
2. **N1 → L1 → L2 → L3 → L4:** единые stream names и renderer, затем сохранение причин, доставка warnings и batch evidence. L2/L3 интегрировать последовательно из-за общего `cli/ride.cpp`.
3. **N4b → N5a/N5b:** более широкий доменный rename, отдельно от logging behavior.
4. **L5 → G1:** явное согласование reporting divergence и итоговая проверка покрытия.

Нужны отдельные review gates для **naming-only**, **diagnostic behavior** и **artifact/report contract changes**. L4 меняет layout sweep log artifacts; L5 намеренно исправляет historical report behavior; L1 меняет human diagnostic formatting. Эти изменения нельзя выдавать за чистую косметику. При реализации можно согласовать и поставить только первый rename-only срез, не принимая весь logging rollout одновременно.

**Статус этого документа:** план подготовлен по статическому чтению. Никакая задача реализации, сборка, runtime reproduction, GUI smoke или тестовый gate здесь не выполнены.
