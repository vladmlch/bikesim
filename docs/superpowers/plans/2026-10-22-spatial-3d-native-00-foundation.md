# Пространственная C++-симуляция — основание и контракты Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Если этих skills нет в среде исполнителя, следовать тому же порядку: одна задача, независимый review поведения, review реализации, evidence, merge. Отсутствие skill не разрешает пропускать gates.

**Goal:** Создать проверяемую базу для параллельной 3D-разработки поверх незавершённой `impl/python-free-cpp`, сохранив planar-контракты и не создавая новых обязательных Python-компонентов.

**Architecture:** Сначала зафиксировать происхождение checkout и реальные upstream-зависимости; затем согласовать пространственные типы, схемы, источники физических эталонов и C++-тестовый контур. Новая конфигурация проходит строгую проверку; capability появляется только после подключения работающего пути, а квалификация остаётся отдельным статусом.

**Tech Stack:** C++23, CMake ≥3.27, CTest, существующий `bt` harness, root build с `BIKE_BUILD_PYTHON_BINDINGS=OFF`; зафиксированные зависимости и numerics providers проекта; macOS arm64 и Linux aarch64.

**Spec:** `docs/superpowers/specs/2026-10-22-spatial-3d-native-design.md`; master `docs/superpowers/plans/2026-10-22-spatial-3d-native-master.md`; единый реестр контрактов из master. Историческая спецификация `2026-10-14-spatial-3d-design.md` используется через таблицу исправлений, а не как второй конкурирующий источник решений.

## Global Constraints

- Это план. В подготовившей его сессии проект, тесты, CMake, compiler probes и контейнеры не запускались; все команды ниже предназначены для отдельной разрешённой реализации.
- Оставить `spatial="planar"` режимом по умолчанию; сохранить старые механические результаты, legacy wire/key sets, replay identity и platform-specific golden data.
- Новые 3D генераторы, конфигурация, физика, исследования, viewer и проверки — C++; существующие временные Python gates удаляет владелец миграции по ADR-0003, не исполнитель 3D-задачи.
- Новые пути в `Create` — предлагаемые результаты выполнения задачи; их наличие в этом плане не означает наличия реализации в приложенном архиве.
- Не объявлять untracked spatial ADR принятым. Номера ADR 0002 и 0003 заняты; 0004 используется только после проверки actual checkout.
- Shared headers, CMake, schemas и capability registry меняет назначенный интегратор. У каждого worker отдельные checkout/build/container identities.
- Bitwise comparison относится к одному execution artifact/provider. Для разных платформ использовать отдельные эталоны и явно объявленные физические допуски.
- Missing fixture, ноль проверок, неустановленная identity или незапущенный gate — блокировка соответствующей приёмки; не xfail/skip и не зелёный stub.

## Файловая структура и владельцы

| Путь | Назначение | Владелец |
|---|---|---|
| `migration/spatial3d/baseline.json` | Выбранный committed base, источник экспорта, WIP dispositions, evidence identities | Интегратор |
| `migration/spatial3d/upstream-gates.json` | G0–G6 с точными SHA, командами и статусами | Интегратор + PFC owner |
| `migration/spatial3d/contracts.json` | Типы/схемы/владельцы и потребители контрактов | Интегратор |
| `migration/spatial3d/references.json` | Источники, таблицы, единицы, uncertainty и payload hashes | Physics reviewer |
| `tests/fixtures/spatial3d/` | Малые data-only эталоны, аналитические случаи и manifests | Владельцы подсистем |
| `native/tests/spatial/` | Новые C++-контрактные, физические и интеграционные тесты | Владельцы подсистем |
| `native/src/spatial/{types,config}.hpp` | Общие пространственные типы и конфигурационный контракт | Интегратор |

## S00 — Зафиксировать базу и сохранить необходимые незакоммиченные работы

**Dependencies:** нет; gate `G0-source`. **Create:** `migration/spatial3d/baseline.json`, `docs/superpowers/audits/2026-10-22-spatial-3d-base.md`. **Modify:** только запись актуальной точки входа в master. **Test/review:** read-only reconciliation настоящего Git checkout; проверка структуры manifest в S04. Эта документальная задача не требует запуска симуляции.

**Consumes:** текущая PFC history и пользовательские рабочие деревья; исторический аудит 22 октября. **Produces:** один неизменяемый `base_sha`, перечень необходимых WIP-патчей с собственными hashes/owners и решение по каждому; имя отдельной пространственной integration branch. `9c08ff1` из аудита — кандидат для сравнения, не автоматически выбранная база.

- [ ] На настоящем checkout прочитать `git --no-optional-locks rev-parse --verify impl/python-free-cpp^{commit}`, `git --no-optional-locks worktree list --porcelain` и `git --no-optional-locks status --porcelain=v1 --untracked-files=all`. Отдельно учесть ignored dependencies/fixture payloads: отсутствие пути в status не означает отсутствия нужного файла.
- [ ] Сверить PFC, lane/b и task/t20 по конкретным необходимым файлам. Сохранить исходное содержимое tracked и untracked работ до любого переноса. Не применять массовое копирование дерева; для каждой правки записать `integrate`, `already_equivalent` или `retain_separate` и основание.
- [ ] Выбрать чистый committed checkpoint PFC. Создать `impl/spatial-3d-native` от этого SHA; рабочие каталоги будущих lanes расположить рядом, без новых вложенных worktree. Не удалять существующие каталоги/ветки.
- [ ] В manifest записать full SHA, parent integration branch, actual tree status, timestamp, source export SHA-256, exact dependency/fixture identities и ссылки на сохранённые WIP-патчи. Для неполученных binary assets установить блокировку конкретного gate.
- [ ] Проверить отсутствие вымышленных ancestry claims: пять JSON приложений старого аудита в данном repomix отсутствуют; сам текстовый экспорт не позволяет воспроизвести все Git-сравнения.
- [ ] Провести независимый review manifest и commit только документации основания. Независимые leaf-задачи могут готовиться при блокировке binary acquisition, но не получают runtime-статус.

Проверка manifest в S04 извлекает `base_sha` строкой из JSON и проверяет реальную форму identity:

```cpp
bt::require(base_sha.size() == 40, "base requires full Git SHA-1");
bt::require(base_sha.find_first_not_of("0123456789abcdef") == std::string::npos,
            "base SHA contains only lowercase hex");
bt::require(!dirty_files_lost, "every required WIP file has a preserved disposition");
```

**Acceptance:** другой исполнитель может восстановить выбранную базу и понять происхождение каждого необходимого дополнения. **Rollback:** снять только новый documentation checkpoint/новую незапущенную пространственную ветку; сохранённые пользовательские работы и старые refs не трогать.

## S01 — Зафиксировать planar-регрессии и реальные PFC gates

**Dependencies:** S00; `G0-source`. **Create:** `migration/spatial3d/upstream-gates.json`, `tests/fixtures/spatial3d/planar-baseline/manifest.json` после получения реальных payloads. **Modify:** таблица зависимостей master. **Test:** будущий `native/tests/spatial/test_baseline.cpp`; существующие affected migration/reference CTest и planar rollout gates по ADR-0003.

**Consumes:** выбранный `base_sha`, текущие `docs/releases/python-free-qualification.md`, `migration/cpp-linux-completion-evidence.md`, фактические test-port rows. **Produces:** именованные gates и набор неизменяемых planar fixtures с platform/provider/build identity. Старые количества `2105/2381 planned` и `13 source-implemented` не копировать как новый пересчёт.

- [ ] Для `G0-source` зафиксировать checkout/assets; для `G1-core` — root bindings-OFF core/writer/state/test baseline; для `G2-startup` — native config→model→terrain→equilibrium→bootstrap→reset. Назначить каждому failing requirement владельца PFC, конкретный failing case и следующий шаг.
- [ ] Аналогично описать `G3-research` для sensors/policies/pending windows, `G4-recording` для catalog/seal/publication/replay, `G5-gui` для native CLI/render replica/GUI и `G6-qualification` для обеих платформ, install/offline и no-Python release. G6 не является prerequisite для начала leaf 3D работ.
- [ ] По выбранному source state и разрешению на выполнение получить короткие planar traces: flat, `rough_uphill_savage.toml`, `alpine_rocky.toml`; отдельно config canonical bytes, model identity, t=0 material state, несколько committed steps, reset и probe. Существующие неисправности обозначить baseline failures; не перезаписывать ожидаемые значения новым кодом.
- [ ] Поставить macOS/Accelerate и Linux/scipy-openblas данные в разные namespaces. Записать compiler/dependency/numerics hashes, command, exit, source SHA, payload length/digest и сравниваемые channels.
- [ ] Разделить `source_present`, `baseline_verified`, `spatial_qualified` и `merged`. Наличие существующего C++ файла не закрывает gate; исторический успешный probe не квалифицирует сегодняшнее дерево.
- [ ] Review соответствия актуальному ADR-0003 и сохранить evidence. Перед shared merge повторить affected checks на итоговом merge commit, не на source tip отдельного worker.

В тесте сравниваются два запуска одной identity; `reference_state` и `candidate_state` — массивы integration/material state, загруженные из manifests:

```cpp
bt::require(reference_provider == candidate_provider, "bitwise comparison needs one provider");
bt::require(reference_state.size() == candidate_state.size(), "state width changed");
for (std::size_t i = 0; i < reference_state.size(); ++i)
    bt::require(std::bit_cast<std::uint64_t>(reference_state[i]) ==
                std::bit_cast<std::uint64_t>(candidate_state[i]), "planar state bit changed");
```

**Acceptance:** для каждой 3D-задачи известно, какой producer gate действительно нужен; baseline failures не замаскированы. **Rollback:** отвергнуть непригодный baseline и выбрать другой reviewed checkpoint; исходные capture payloads оставить неизменными для диагностики.

## S02 — Зафиксировать контракты, ADR и владельцев общих файлов

**Dependencies:** S00, S01; `G0-source`. **Create:** `migration/spatial3d/contracts.json`, `docs/adr/0004-spatial-3d-native-correctness.md` после проверки свободного номера. **Modify:** актуальная spec/master; `migration/interface-contracts.md` — ссылки на пространственное расширение. **Test/review:** cross-document type/schema/dependency review; позднее S04 проверяет machine-readable registry.

**Consumes:** существующие `bike::model::BuildRequest`, `bike::initialization::SetupRequest`, `runtime::RuntimeBootstrap`, native configuration/projector и state codecs. **Produces:** единый контракт пространства `bike::spatial::{Mode,RootKind,Config,Topology,RootBinding,BodyPose}`, `bike::terrain::{Grid3d,GridSample3d,RayHit3d}` и `bike::pneumatic3d`; существующие namespaces `runtime`/`nativecfg` сохраняются.

- [ ] Зафиксировать enum vocabulary `Mode::{planar,spatial3d}`, `RootKind::{planar3,constrained4,free6}`. `constrained4` означает x/y/z/pitch; roll/yaw отсутствуют кинематически. Free joint имеет независимые qpos/dof spans; нельзя вычислять их из общего joint ordinal.
- [ ] Внести ownership/signature/validity/lifetime в registry для config decode, terrain sampling/raycast, tyre evaluation/state, topology binding, bootstrap capture, observation, catalog и replay. Все coordinate frames и reference points указать рядом с единицами.
- [ ] Зафиксировать dispatch из canonical contracts C7: existing planar runtime1/bootstrap1/writer1 или2 остаются; spatial получает runtime envelope2, bootstrap2 и writer schema3 с независимыми constants и отдельными exact-key readers. `spatial_contract=1`, tyre/checkpoint, observation, recording и terrain versions самостоятельны; совпадение цифр не связывает их lifecycle.
- [ ] В ADR закрепить: новая физика проверяется unit/invariant/independent-reference/convergence gates; старые planar comparisons переходят по ADR-0003. Убрать требование Python twin, xfail stubs и автоматическую смену `proposed` на `accepted` по старому документу.
- [ ] Назначить одного интегратора shared CMake/config/topology/schema files. Изменение любого контракта сначала отражается в registry и затронутых task contracts, затем реализуется совместно с consumers; worker не придумывает альтернативный тип самостоятельно.
- [ ] Согласовать design decision с текущим запросом пользователя и сохранить статус документа честно: новый план предлагает конкретное решение, но не выдумывает историческое одобрение отсутствующего ADR.

Пример compile-time контракта, который S05/S13 воплощают в source, а S04 проверяет без fake implementations:

```cpp
enum class Mode : std::uint8_t { planar, spatial3d };
enum class RootKind : std::uint8_t { planar3, constrained4, free6 };
static_assert(RootKind::planar3 != RootKind::free6);
// runtime assertions remain real in Release, unlike assert():
bt::require(registry_has_one_owner_per_contract, "contract ownership is ambiguous");
bt::require(schema_names_are_distinct, "independent wire schemas were conflated");
```

**Acceptance:** leaf workers получают один набор терминов и версий; нет dangling contract type или consumer без владельца. **Rollback:** вернуть конкретное изменение registry вместе с его consumers; ADR-0002/0003 и PFC deletion rules не менять.

## S03 — Реестр независимых физических эталонов и происхождения данных

**Dependencies:** S02; `G0-source`. **Create:** `migration/spatial3d/references.json`, `tests/fixtures/spatial3d/references/meijaard2007.json`, `tests/fixtures/spatial3d/references/tyre-lateral.json`, `docs/reference/spatial3d-provenance.md`. **Modify:** только ссылки на существующий `docs/reference/mtb-lateral-digitisation.md`. **Test:** `native/tests/spatial/test_references.cpp` в S04.

**Consumes:** первичные публикации, доступные авторские данные, existing tyre/terrain research evidence. **Produces:** machine-readable параметры с единицами, frame/sign convention, областью применимости и источником каждого значения; независимый analytical benchmark имеет отдельный model profile.

- [ ] Для Meijaard 2007 записать DOI `10.1098/rspa.2007.1857`, точные tables/pages/equations и версии доступных авторских материалов. Перепроверить названия и число параметров по первичному источнику; не переносить ошибочное «27 параметров»/названия таблиц из старого плана автоматически.
- [ ] Зафиксировать benchmark mass/inertia/geometry и linear-system reference отдельно от compliant MTB model. Риг имеет rigid rider/frame, заданные wheel inertias, нулевой controller и явно описанный contact idealization. Проверка одного эталона не валидирует suspension/pneumatic model целиком.
- [ ] Для tyre данных сохранить tyre dimensions, pressure, load, speed, поверхности, координаты slip/camber, radians/degrees, fit convention, uncertainty и оригинальные units. Urban/cargo data использовать как отдельный cross-check, не как доказательство калибровки low-pressure aggressive MTB.
- [ ] Существующие digitisation outputs читать вместе с source provenance; допуски derive из измерений/оцифровки и model domain. Не вводить универсальный потолок `Fy <= Fz*tan(camber)` или произвольный множитель, который заменяет отсутствующий эксперимент.
- [ ] Проверить payload byte hashes, лицензии и ограничения распространения. Если источник/данные отсутствуют, записать blocked зависимые research gates; ни вымышленных чисел, ни нулевых «эталонов», ни passing skip.
- [ ] Physics reviewer независимо проверяет единицы, знаки и хотя бы одну вручную воспроизводимую reference calculation; commit данных отделить от реализации закона шины.

В C++ fixture test `degrees`/`radians` и `normal_load_n`/`cornering_stiffness_n_per_rad` читаются из одной подписанной measurement row:

```cpp
bt::require(std::isfinite(normal_load_n) && normal_load_n > 0., "invalid reference load");
bt::require(std::abs(radians - degrees * std::numbers::pi / 180.) < 1e-14,
            "reference angle conversion is inconsistent");
bt::require(cornering_stiffness_n_per_rad > 0., "reference sign convention not normalized");
bt::require(!source_doi.empty() && !source_location.empty(), "unattributed reference value");
```

**Acceptance:** независимый reviewer восстанавливает каждое число и понимает пределы его использования; blocking gaps видны. **Rollback:** спорный source row получает disputed/blocked status; реализация не перенастраивается под неподтверждённую цифру.

## S04 — C++-тестовый контур и fail-closed evidence checks

**Dependencies:** S01–S03; `G1-core` для выполнения. **Create:** `native/tests/spatial/CMakeLists.txt`, `native/tests/spatial/fixture_paths.hpp.in`, `native/tests/spatial/test_baseline.cpp`, `native/tests/spatial/test_references.cpp`, `native/tests/spatial/test_harness.cpp`. **Modify:** root `CMakeLists.txt`; shared registration до T63 согласовать с `native/CMakeLists.txt`. **Test:** `spatial.harness`, `spatial.baseline`, `spatial.references`; targets `bike_spatial_harness`, `bike_spatial_baseline`, `bike_spatial_references`.

**Consumes:** существующий `native/tests/support/migration_test.hpp` и target `bike_migration_test`, `bike_io`, absolute build-owned fixture root. **Produces:** один явный CTest registration helper и локальные пространственные fixtures; не новый testing framework.

- [ ] Добавить subdirectory только при BUILD_TESTING. Helper принимает slug, source list и libraries, создаёт `bike_spatial_<slug>`, применяет действующую `bike_apply_native_policy`/engine policy и регистрирует `spatial.<slug>`; пустой source list — configure error.
- [ ] Переиспользовать `bt::run`/`bt::require` и C++ JSON/hash IO. `bt::fixture()` сейчас привязан к `tests/fixtures/migration`; не менять его root глобально. Для spatial fixtures сгенерировать private `BIKE_SPATIAL_FIXTURE_ROOT` из source tree и отдельный bounded/path-contained reader.
- [ ] Реализовать assertions из S00–S03 на настоящих manifest fields: отсутствующий файл, invalid SHA, duplicate key, malformed array shape, traversal и mismatched identity должны завершать тест ошибкой с конкретным путём.
- [ ] Добавить успешный control на минимальном data-only fixture и негативный child-process control на `bt::run([]{})`: ноль require-проверок обязан вернуть ненулевой exit. Ни тестируемое приложение, ни production library не получают test-only global hooks.
- [ ] Во время разрешённой реализации запустить targeted tests с `--no-tests=error`; сохранить stdout/exit, exact binary, source/provider identity. CTest labels `slow`, `audit`, `package` не должны незаметно исключить обязательные spatial gates из их явной команды.
- [ ] Проверить root bindings-OFF closure на обеих required платформах; до T63 общий код продолжает компилироваться в transition graph. Не создавать nanobind bindings для 3D только ради тестирования.

Минимальный реальный test body, сохраняющий проверки в Release:

```cpp
#include "support/migration_test.hpp"
#include <bit>
#include <cstdint>
int main() {
    return bt::run([] {
        bt::require(std::bit_cast<std::uint64_t>(0.) !=
                    std::bit_cast<std::uint64_t>(-0.), "bitwise test distinguishes signed zero");
        bt::invalid([] { throw std::invalid_argument("fixture.path: rejected"); });
    });
}
```

**Acceptance:** реально выполнены положительный и отрицательный harness controls; missing reference blocks, zero-test run fails. **Rollback:** удалить только spatial registration; существующий harness и planar reference registration оставить.

## S05 — Native spatial config и отказ для неподключённых capabilities

**Dependencies:** S02, S04; `G1-core`. **Create:** `native/src/spatial/types.hpp`, `native/src/spatial/config.hpp`, `native/src/spatial/config.cpp`, `native/tests/spatial/test_config.cpp`; fixture TOML под `tests/fixtures/spatial3d/config/`. **Modify:** `native/src/configuration/{model_config.hpp,load.cpp,schema.cpp,validate.cpp}`, `native/src/runtime/{capabilities.hpp,capabilities.cpp,project_config.cpp}`, `native/src/writer_config.cpp`, shared CMake. **Test:** `spatial.config` / `bike_spatial_config`.

**Consumes:** frozen S02 schema contract, existing strict TOML→owned config→wire pipeline. **Produces:** `bike::spatial::Config`, `Mode`, `RootKind`; полный parsed spatial request без обещания, что physics/runtime уже доступны. Публичный `implemented_capabilities` по source отличается от `qualification_pending` и release evidence.

- [ ] Сначала добавить негативные fixtures: spatial table без `spatial="3d"`; 3D с planar tyre backend; planar с `pneumatic_3d`; constrained4 со steering controller; NaN/Inf/неположительные grid steps; unsupported schema; unknown ключ; недопустимые rider/drive combinations. Ошибка называет полное config field path до создания mjModel.
- [ ] Воплотить перечисленные в canonical contracts типы и defaults. Принятый enum spelling один для parser, runtime, saved config и CLI; не вводить независимые boolean switches у consumers.
- [ ] Обеспечить два projection path по C7: отсутствие spatial даёт прежние canonical bytes и runtime1/bootstrap1/writer1 или2; spatial request выводит runtime2/writer3, bootstrap2 формируется S17 и производителями state. Не пропускать новый документ через старый exact-key decoder. Проверить каждый поддерживаемый legacy example, не только один TOML.
- [ ] Конфигурационный parser может принять syntactically valid будущий spatial request, но startup capability checker до producer gates обязан выдать точное «не подключён spatial …». Не возвращать implemented=true по факту существования enum/пустой функции.
- [ ] После каждого producer task разрешать лишь конкретную capability row с source paths и meaningful tests. Например, constrained root без подключённого tyre writer не становится запускаемым режимом.
- [ ] Проверить field ownership и отсутствие Python imports/bindings в новых headers; выполнить focused tests и required affected baseline checks на разрешённом этапе, записать evidence и commit.

Для двух реально загруженных fixtures тест проверяет existing report semantics и legacy roundtrip; имена local values задаются fixture setup:

```cpp
bt::require(!spatial_report.implemented, "config parsing must not advertise missing runtime");
bt::require(spatial_report.qualification_pending, "source availability is not qualification");
bt::require(!spatial_report.rejections.empty(), "unavailable capability needs a concrete reason");
bt::require(legacy_wire_before == legacy_wire_after, "planar canonical configuration changed");
```

**Acceptance:** старые inputs сохраняют контракты; новые inputs валидируются и честно отклоняются до подключения реализации; нет принятой комбинации с отсутствующим owner. **Rollback:** вернуть новую spatial registry row/decoder вместе, сохранив old parser; новые serialized schema IDs не переиспользовать для другого смысла.

## Передача следующему исполнителю

S06–S12 получают terrain/config/reference contracts и `G1-core`; S13 получает topology vocabulary и unchanged planar fixtures. Все checkbox статусы этого документа остаются открытыми до фактического выполнения. Hand-off содержит source SHA, API version, изменённые пути, команды/exit, две платформенные identity или конкретный blocker и решение reviewer.
