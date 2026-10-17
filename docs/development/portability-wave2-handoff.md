# Portability: вторая волна — T19 и T24

Дата: 2026-10-17.

**Статус: реализовано в исходниках; исполнение и платформенная приёмка не проверены.**
Вторая волна трактуется по первой таблице пользователя: T19 — `mjpython` и
Linux viewers; T24 — манифесты offline bundles. Сборки, тесты, приложение,
изменённые утилиты проекта и установка зависимостей не запускались. Импорты
модулей проекта и синтаксическая компиляция также не выполнялись.

## Основание и границы

Работа выполнена над `repomix-output.xml` из вложения
`repomix-output.xml(10).zip`. Извлечены 1602 файла исходного снимка, сохранена
неизменённая распаковка для сравнения. Git-история во вложении отсутствует.

В снимке нет исходных portability briefs, `task-index.json` и подтверждения
завершения T18/T23. Существующие `migration/cards/T19.md` и `T24.md` относятся
к другой программе — C++ migration, random streams и equilibrium/state.
Эти карточки, их зависимости и ledger не изменялись. Соответствие полной
исходной portability-спецификации требует сверки после получения её файлов.

В наличии были `tools/install_offline.py` и `tools/check_offline_sources.py`.
Отсутствовали `packaging/`, `tools/make_offline_bundle.sh` и
`tools/install_offline.sh`. Producer и packaging-контракт в этой поставке
созданы заново. Это не восстановление отсутствовавших untracked-файлов.
Новый install shell wrapper не создавался: установка остаётся границей T25.

## T19: подготовка и жизненный цикл viewer

Добавлен общий модуль `src/bike_sim/viewer_runtime.py`. Сам модуль использует
только стандартную библиотеку до запроса viewer. Это не квалифицирует полную
headless import isolation пакета: её исходная задача T18 отсутствует.

### macOS

Подготовка проверяет настоящий UI bridge уже загруженного `mujoco.viewer`.
Имя исполняемого файла и унаследованная переменная окружения не считаются
доказательством запуска через `mjpython`.

Launcher ищется сначала в scripts-каталоге выбранного Python и рядом с его
исполняемым файлом, затем в PATH. Python-скрипт `mjpython` запускается именно
выбранным интерпретатором. Сохраняются исходные аргументы, `-m`, флаги Python,
аргументы с пробелами, cwd и окружение, включая `BIKE_NATIVE_BUILD_PATH`.
Флаги интерпретатора применяются и к промежуточному Python launcher, чтобы
его поиск MuJoCo не менял поведение `-I`/`-S`.

PID-маркер останавливает повторный неудачный relaunch и не выдаёт дочерний
процесс за уже подготовленный GUI-процесс. Ошибки поиска и запуска launcher
возвращаются с диагностикой. Прежний
`bike_sim.sim.playground.ensure_macos_mjpython()` сохранён как делегат.

### Linux

Подготовка Linux не ищет и не запускает `mjpython`. Проверяются desktop
переменные `DISPLAY`/`WAYLAND_DISPLAY` и основной Python-поток. Инициализация
GLFW выполняется синхронно, чтобы её ошибка доходила до владельца сессии.
Наличие `WAYLAND_DISPLAY` разрешает попытку запуска, но не доказывает поддержку
native Wayland установленным GLFW; диагностика указывает на возможную
необходимость XWayland.

Успех `glfw.init()` не гарантирует успех последующего асинхронного создания
окна, контекста и рендеринга в MuJoCo. Для этих путей остаётся desktop-приёмка;
отсутствие зависания во всех GUI-сценариях не заявляется.

### Точки входа и владение ресурсами

| Маршрут | Граница подготовки |
| --- | --- |
| `bike-playground`, `bike-sim --playground` | До создания симуляции и выходных файлов интерактивного запуска |
| Графический `bike-ride`, включая research | До native preflight и создания driver/session/workers |
| `bike-replay --viewer` | До создания checked replay session |
| `bike-replay --view` | До загрузки траектории для показа |
| Низкоуровневый viewer с готовым driver/session | Только проверка; замена процесса запрещена |

Headless ride, preview, help/listing, replay без viewer и `--view --dry-run`
сохраняют неграфические ветки. Проверка некорректного playback FPS остаётся
раньше подготовки GUI, сохраняя порядок пользовательских ошибок.

Все семь мест passive-launch используют общий контекст окна. Вторичная ошибка
закрытия окна не подменяет исходную ошибку цикла или `KeyboardInterrupt`.
Аналогично защищено финальное закрытие physical driver/replay session;
вторичная ошибка прикрепляется как exception note. Ошибка обычного закрытия
без предыдущего сбоя продолжает быть ошибкой.

Q/Escape в playback запрашивает остановку цикла. Окно закрывает владеющий
контекст, в том числе если callback пришёл ещё во время запуска. Публикация
preview CSV и финализация записи сохраняют прежнее поведение и относятся
к дальнейшей работе T21. Полная recording lifecycle-приёмка не выполнена.

Подробный контракт и первичные ссылки: [viewer-portability.md](viewer-portability.md).

## T24: manifest, producer и проверка содержимого

Добавлены `tools/offline_bundle.py`, shell-вход `tools/make_offline_bundle.sh`,
JSON Schema, каталог targets и [описание packaging-контракта](../../packaging/README.md).

Producer принимает явные локальные исходники и wheelhouse. Он не собирает
wheel, не разрешает зависимости, не скачивает файлы, не импортирует проект
и не вызывает установщик. Выходной ZIP сохраняет текущую структуру
`project/`, `wheelhouse/`, `requirements.txt` и добавляет `bundle-manifest.json`.
Сам producer в этой сессии не запускался; release offline bundles не создавались.

| Часть manifest v1 | Что записывается и проверяется |
| --- | --- |
| `target` / `producer` | Целевая ОС, архитектура, CPython и OS baseline отдельно от платформы сборщика |
| `project` | Имя, версия, `requires_python` из исходного `pyproject.toml`, source digest, `authority: source`, пересборка native на целевой системе |
| `files` | Точный состав файлов, относительные пути, размеры и SHA-256 |
| `wheels` | Имена, версии, Python/ABI/platform tags, hashes и согласованность filename с `METADATA`/`WHEEL` |
| `requirements` | Исходные runtime/build declarations, pins предустановленных пакетов, честный статус непроверенной closure |
| `qualification` | Установка, native build, GUI и binary compatibility всегда `not_run` |

Targets: Ubuntu 24.04 x86_64 / CPython 3.13; macOS arm64 и x86_64 / CPython
3.13 либо 3.14. Это варианты метаданных, а не квалифицированные платформы.
macOS deployment minimum пока `unspecified`. Сборщик на другой архитектуре
может описать локальные wheels, но это не квалифицирует целевую архитектуру.

Для `full` список предустановленных Python-пакетов должен быть пустым. Для
`slim` он содержит явные `name==version`; дубли и пересечение с локальными
wheels отвергаются. Полнота supplied wheels и совместимость внешних пакетов
с полной dependency closure остаются T25. `full` не означает поставку ОС,
интерпретатора, компилятора или SDK.

### Исходники остаются авторитетным содержимым

Единственный `bike_sim` wheel должен совпадать с исходниками по версии,
Python-файлам, ресурсам пакета, console/gui/plugin entry points и
`Requires-Python`. Чужие payload-пути, в том числе `.data/purelib`, отвергаются:
они могли бы перезаписать пакет при установке после успешного сравнения
основных файлов. В project wheel не допускаются native/build-артефакты.

Проверка `requires-python` поддерживает явные границы `>=major.minor` и
`<major.minor`, включая эквивалентную запись с `.0`. Patch-specific и другие
общие формы спецификаторов явно отклоняются; собственный PEP 440 resolver
не реализован. Семантическая сверка `Requires-Dist` отмечена
`wheel_dependency_metadata: not_verified`.

Producer исключает venv, build outputs, caches, бинарные библиотеки/объекты
и локальное скрытое состояние. Сохраняются нужные source-конфиги, включая
`.clangd`, `.clang-tidy`, `.gitattributes`, `.github` и `.python-version`, если
они имеются во входных файлах. Абсолютные/traversal-пути, symlinks,
необычные типы файлов, дубли и case/Unicode-коллизии отвергаются.

Состав и hashes сверяются с фактическими байтами; лишние файлы тоже являются
ошибкой. Выход обязан находиться вне исходного дерева, существующий архив
не перезаписывается. Порядок ZIP и timestamp нормализованы. Побайтовая
воспроизводимость самого producer пока подтверждена только конструкцией кода
и подготовленным тестом, без запуска.

Совпадение hashes не является подписью издателя. Отбрасывание явно неверных
wheel tags не доказывает glibc/CPU/SDK/shared-library compatibility.

### Обратная совместимость и граница T25

`check_offline_sources.py` сохраняет прежние `--source`, `--wheel` и результат
`missing_source` / `missing_wheel` / `different`. Добавлен режим `--bundle`.
Прежнее сравнение Python-файлов не приобретает новую зависимость от producer.

`install_offline.py` получил только предварительную проверку присутствующего
manifest и заявленных preinstalled pins. Прежний no-manifest путь сохранён.
Новый manifest допускается этим installer только для его существующего slim
Ubuntu 24.04 x86_64 / CPython 3.13 контракта; full/macOS отвергаются с указанием
на T25. Проверка declared target не является проверкой реального дистрибутива.
Source-first установка и кроссплатформенная установка не объявлены готовыми.

Прямые script-входы checker/installer подавляют создание bytecode до импорта
соседнего validator, чтобы не добавить `__pycache__` внутрь проверяемого
bundle. Для импортируемого/module-вызова из такого каталога документация
требует `-B` либо `PYTHONDONTWRITEBYTECODE=1` заранее. Inventory-проверка
предназначена для неизменённого каталога до установки: созданная внутри
venv или install report меняет его состав.

## Регрессионные исходники и ревью

Подготовлены или адаптированы шесть test-файлов:

- `tests/test_viewer_runtime.py`;
- `tests/reference/test_viewer_cli_portability.py`;
- `tests/reference/test_frontend_windows.py`;
- `tests/reference/test_runtime_single_path.py`;
- `tests/reference/test_ride_backend_cli.py`;
- `tests/test_offline_bundle.py`.

Они покрывают launcher/argv/env, GUI/non-GUI маршруты, ошибки закрытия,
ранний quit callback, manifests, пути, source authority, targets и
самозагрязнение bundle bytecode-файлами. В коде тестов есть будущие fake- и
subprocess-сценарии; ни один из них не запускался. Последовательность
«сначала failing test, затем green test» не заявляется.

T19 и T24 реализованы разными владельцами; исходники и регрессии проверялись
отдельным reviewer. Дополнительно проверена граница checker/installer.
Устранены найденные при чтении дефекты сохранения interpreter flags,
закрытия playback, wheel payload/entry points и bytecode внутри inventory.
Результат такого ревью — оценка исходников, не заменяющая тестовый прогон.

## Что остаётся открытым

| Проверка | Статус |
| --- | --- |
| Исходные portability briefs и evidence T18/T23 | Отсутствуют в приложенном снимке |
| Focused Python regression suite | Не запускалась по указанию пользователя |
| Синтаксическая компиляция / native build / sanitizers | Не запускались по указанию пользователя |
| macOS desktop GUI acceptance | Не выполнялась |
| Linux desktop GUI acceptance | Не выполнялась |
| Ubuntu 24.04 x86_64 execution qualification | Не выполнялась |
| Настоящие full/slim bundles и offline install | Не выполнялись; установка относится к T25 |
| Dependency closure, `Requires-Dist`, binary compatibility | Не квалифицированы |
| Recording lifecycle, performance, CI/release acceptance | За пределами этой волны |

Исторические physics/baseline/RTSan результаты из пользовательского ревью
не перепроверялись и не закрывались. T27/T29/T30 и release gate не помечены
пройденными. Machine-readable состояние находится в
[portability-wave2-status.json](portability-wave2-status.json).

## Поставка и восстановление Repomix

Поставка содержит 29 изменённых/новых файлов: 16 существовавших изменены,
13 добавлены. Подготовлены patch, ZIP только этих файлов и ZIP обновлённого
исходного снимка с корнем `bikesim/` — всего 1615 файлов. Последний — архив исходников из Repomix;
он не содержит отсутствовавших во вложении бинарных wheels, зависимостей или
истории Git и не является release offline bundle.

Repomix/unmixer не сохраняет исходные terminal newlines буквально. Для
подготовки patch использованы исторические SHA-256 из имеющегося
`migration/source-manifest.json`: где добавление terminal LF восстанавливает
hash, окончание доказано; в остальных случаях один terminal LF восстановлен
по соглашению. Для всех 16 существовавших файлов, затронутых patch, baseline
точно восстановлен по историческому SHA-256 добавлением одного LF. Во всём
исходном снимке hash совпал у 1063 файлов; у остальных 539 terminal LF
восстановлен по соглашению. Исходный XML также позволяет сохранить имевшиеся CRLF в трёх
неизменённых CSV. Исторический manifest не обновлялся под новые исходники и
не выдается за provenance этой поставки.

Проверка применения patch выполнена только как файловая операция в
отдельной копии восстановленного baseline: `git apply --check` и применение
завершились успешно. Полученные байты совпали с архивом изменений; состав
обоих ZIP совпал с ожидаемыми путями. Такая проверка
не запускает проект и не утверждает, что patch применится без конфликта к
другому commit или к checkout с локальными изменениями.

Команды для применения пользователем к подходящему checkout:

```bash
git apply --check /path/to/bikesim-portability-wave2.patch
git apply /path/to/bikesim-portability-wave2.patch
```

Вся заявленная завершённость относится к исходникам этой поставки.
Execution verification остаётся открытой до отдельного разрешённого прогона.
