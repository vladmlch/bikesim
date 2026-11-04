# Быстрая и полезная проверка — план работ

> **Для исполнителя-агента:** ОБЯЗАТЕЛЬНЫЙ НАВЫК: superpowers:subagent-driven-development (рекомендуется) или superpowers:executing-plans — задачи выполняются по одной. Шаги отмечены `- [ ]`; отмечай `- [x]` только после проверки шага.

**Цель:** полная проверка (`tools/run_tests.sh native`) укладывается в 30 минут на свободной машине, проверка одного изменения — в 5 минут, а на текущем коде полная проверка может закончиться успехом.

**Устройство:** чиним два теста, которые падают без всяких изменений. Переставляем стадии проверки от дешёвых к дорогим и даём продолжать с упавшей стадии. Убираем экспериментальную половину анализатора. Заставляем `bike-native-checks` проверять файлы одновременно, отдавая каждой проверке долю ядер по числу уже идущих. Убираем повторные компиляции заголовков. Добавляем команду `tools/check_change.sh` для проверки одного изменения.

**Средства:** C++23 (`native/src/tools/*.cpp`, программа `bike-native-checks`), bash (`tools/*.sh`), сценарии CMake `-P` в качестве тестов (`native/tests/tooling/*.cmake`), CTest, Ninja.

**Описание решений:** [docs/superpowers/specs/2026-11-03-verification-speed-design.md](../specs/2026-11-03-verification-speed-design.md) — прочитай целиком перед началом.

## Общие ограничения

- Работать в рабочей копии `.worktrees/main3`, ветка `main3`. Сборка — `build/native-release` (заготовка настроек `release`).
- Перед любым изменением в `native/` или `tools/` прочитай `docs/agents/native-engineering.md`.
- Linux вне рамок работы: не запускать `tools/linux/container.sh` и не трогать `tests/fixtures/*/linux-aarch64/` без явной просьбы владельца.
- Набор проверок в полной проверке не сокращается. Исключения: экспериментальная половина анализатора (задача 5) и повторные компиляции заголовков в одинаковых условиях (задача 9).
- Проверка по-прежнему падает при отсутствии нужного инструмента. Исключение — `ccache`: он только ускоряет сборку и ничего не проверяет.
- Отчёты о проверке остаются однозначными: итоги каждого вида пишутся в прежнем порядке файлов, независимо от числа одновременных процессов.
- Промежуточный прогон (с `--from`) нигде не выдаётся за итоговую проверку: в выводе и в журнале он помечен как частичный.
- Сообщения программ, комментарии в коде и сообщения коммитов — на английском, как во всём репозитории. Сообщения коммитов — в виде `type(scope): summary`.
- Две полные проверки в одной рабочей копии одновременно не запускать.
- Во время задач гоняй только нужное: отдельный тест через `ctest -R '^имя$'` и отдельный вид проверки через `build/native-release/bin/bike-native-checks --kind <вид> --build build/native-release`. Полная проверка `bash tools/run_tests.sh native` запускается **один раз**, в задаче 13.

## Карта файлов

| Файл | Что меняется |
|---|---|
| `native/tests/reference/test_native_numeric_portability.cpp` | ключи эталонов в новом формате (задача 1) |
| `native/tests/reference/ft2_reference_support.hpp` | перенос записанных при съёмке путей на текущую копию (задача 2) |
| `tools/run_native_checks.sh` | порядок видов, `--from`, время каждого вида (задача 3) |
| `tools/run_tests.sh` | обычные тесты первыми, `--from`, таблица времени, журнал, доля ядер (задачи 4, 8) |
| `native/tests/tooling/tooling_gate.cmake` (новый) | быстрые тесты сценариев проверки на заглушках (задачи 3, 4, 8, 12) |
| `native/tests/tooling/tooling_platform.cmake` | новая строка порядка видов (задача 3) |
| `native/src/tools/check_frontends.cpp` | одна половина анализатора, одновременный запуск, без повторов заголовков, выбор файлов (задачи 5, 7, 9, 12) |
| `native/src/tools/check_pool.cpp` (новый), `checks_internal.hpp` | ограниченный набор потоков-исполнителей (задача 7) |
| `native/src/tools/checks_main.cpp` | ключ `--only-sources` (задача 12) |
| `native/tests/tooling/fixture_support.cmake` (новый) | общие помощники учебных сборок, вынесенные из `tooling_checks.cmake` (задача 7) |
| `native/tests/tooling/tooling_sweep_engine.cmake` (новый) | быстрые тесты `bike-native-checks` на учебных сборках (задачи 5, 7, 9, 12) |
| `native/tests/tooling/tooling_checks.cmake` | вынос помощников, замена случаев про экспериментальную половину (задачи 5, 7) |
| `native/cmake/Checks.cmake` | регистрация новых тестов и `check_pool.cpp` (задачи 3, 7) |
| `CMakeLists.txt` | `ccache`, если установлен (задача 10) |
| `tools/check_change.sh` (новый) | проверка одного изменения (задача 12) |
| `AGENTS.md`, `docs/TESTING.md`, `docs/agents/native-engineering.md` | три уровня, настоящее время, новые ключи (задача 13) |

---

### Задача 0: Исходное состояние

Прежде чем что-то менять, зафиксируй, что сейчас зелёное, а что красное. Иначе потом не отличить «сломал я» от «так было».

**Файлы:**
- Создать: `docs/superpowers/audits/2026-11-03-verification-speed-baseline.md`

- [x] **Шаг 1: Запиши состояние рабочей копии**

```bash
cd .worktrees/main3
git rev-parse HEAD
git status --short
```

В отчёт: хеш и список неучтённых файлов. Ожидаемо неучтённые: `.agents/`, `.devin/`, `.junie/`, план `2026-11-03-crank-reposition-native.md`, этот план и описание решений. Не трогай их.

- [x] **Шаг 2: Собери и прогони обычные тесты**

```bash
cmake --preset release
cmake --build --preset release
ctest --test-dir build/native-release -LE 'slow|package' -j8 --output-on-failure 2>&1 | tail -15
```

Ожидаемо: `2 tests failed out of 183` — `reference.native_research_recording` и `reference.native_numeric_portability`. Если красных больше, добавь их в отчёт и сообщи владельцу до перехода к задаче 1.

- [x] **Шаг 3: Прогони тесты сценариев проверки**

`tooling_checks` ни разу не запускался на `main3` (в `CTestCostData.txt` у него 0 запусков). Его будем менять, поэтому нужно знать исходное состояние.

```bash
ctest --test-dir build/native-release -R '^(tooling_checks|tooling_platform|tooling_toolchain|tooling_install)$' --output-on-failure 2>&1 | tail -40
```

`tooling_checks` проходит настоящие виды проверки по всему коду и может идти 20+ минут. Запиши в отчёт итог по каждому тесту и **дословно** все строки неудач. Красные случаи, которые были до тебя, ты не чинишь, а только отмечаешь. Сравнивать с ними будешь в задачах 3, 5, 7.

- [x] **Шаг 4: Закоммить отчёт**

```bash
git add docs/superpowers/audits/2026-11-03-verification-speed-baseline.md docs/superpowers/specs/2026-11-03-verification-speed-design.md docs/superpowers/plans/2026-11-03-verification-speed.md
git commit -m "docs(verification): record baseline before the speed-up work

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 1: Ключи эталонов в `native_numeric_portability`

**Причина (проверена):** коммит `92fcc0f7` перевёл файлы эталонов с имён старых Python-тестов (`test_<случай>[<параметр>]`) на новый вид `<набор>/<случай>/<параметр>`. Например, файл теперь лежит в `tests/fixtures/golden/darwin-arm64/native_numeric_portability/tanh_and_hypot_strided_oracle/-1.bin`. Но помощник `node()` в тесте по-прежнему собирает старый ключ `native_numeric_portability/test_tanh_and_hypot_strided_oracle[-1]`. Проверка ключа в `native/tests/support/golden.cpp:94-98` отвергает `[` и `]` с сообщением `pytest node syntax is not a native key`.

**Файлы:**
- Изменить: `native/tests/reference/test_native_numeric_portability.cpp:43-51` и 13 мест вызова `node("test_…"`

- [x] **Шаг 1: Убедись, что тест красный по названной причине**

```bash
ctest --test-dir build/native-release -R '^reference.native_numeric_portability$' --output-on-failure 2>&1 | grep -E 'FAIL|Passed|Failed'
```

Ожидаемо: `FAIL golden node: pytest node syntax is not a native key`.

- [x] **Шаг 2: Перепиши `node()`**

Замени тело помощника (строки 43–51) на:

```cpp
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (name, param) order mirrors the oracle id
[[nodiscard]] std::string node(std::string_view name, std::string_view param) {
    // Native golden key <suite>/<case>/<param> (rekeyed in 92fcc0f7): no
    // pytest "test_" prefix, the parameter is its own sanitized component.
    std::string out{kFile};
    out += name;
    out += '/';
    out += bt::oracle::key(param);
    return out;
}
```

- [x] **Шаг 3: Убери приставку `test_` в местах вызова**

```bash
sed -i '' 's/node("test_/node("/g' native/tests/reference/test_native_numeric_portability.cpp
git grep -n 'node("' -- native/tests/reference/test_native_numeric_portability.cpp
```

Ожидаемо: 13 вызовов, ни один не начинается с `test_`. Каждое имя должно совпадать с каталогом в `tests/fixtures/golden/darwin-arm64/native_numeric_portability/`. Проверка:

```bash
for n in $(grep -oE 'node\("[a-z_0-9]+"' native/tests/reference/test_native_numeric_portability.cpp | sed 's/node("//; s/"//' | sort -u); do
  test -d "tests/fixtures/golden/darwin-arm64/native_numeric_portability/$n" || echo "MISSING $n"
done
```

Ожидаемо: пустой вывод.

- [x] **Шаг 4: Собери и прогони тест**

```bash
cmake --build --preset release --target reference.native_numeric_portability
ctest --test-dir build/native-release -R '^reference.native_numeric_portability$' --output-on-failure 2>&1 | tail -3
```

Ожидаемо: `100% tests passed`. Если упадёт на другом ключе — значит, ещё одно место строит ключ вручную. Найди его через `git grep -n '\[' -- native/tests/reference/test_native_numeric_portability.cpp` и исправь так же.

- [x] **Шаг 5: Проверь, что проходы анализа не возражают**

```bash
build/native-release/bin/bike-native-checks --kind tidy --build build/native-release 2>&1 | tail -3
```

Если к этому моменту задача 7 ещё не сделана, проход идёт около 33 минут. Тогда вместо него запусти `clang-tidy` на одном файле:

```bash
"$(brew --prefix llvm)/bin/clang-tidy" -p build/native-release native/tests/reference/test_native_numeric_portability.cpp 2>&1 | grep -E 'warning|error' | head
```

Ожидаемо: нет новых предупреждений.

- [x] **Шаг 6: Коммит**

```bash
git add native/tests/reference/test_native_numeric_portability.cpp
git commit -m "fix(test): build native golden keys in numeric portability

The rekey in 92fcc0f7 moved the fixtures to <suite>/<case>/<param>, but
node() still built pytest ids (test_<case>[<param>]), which node_path()
rejects. The test had been red since then.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 2: Пути к удалённой рабочей копии в учебных данных `research`

**Причина (проверена):** в учебных данных `tests/fixtures/ft2/darwin-arm64/research_flat/{config.json,provenance.json,assets/resolved_config.json}` записаны абсолютные пути `…/.worktrees/pfc-lane-f/examples/research/rider_joint_envelope_anatomical.json` и `…/rider_strength_reference.json`. Рабочая копия `pfc-lane-f` давно удалена. В конце прогона `identity_receipt()` (`native/src/research/metadata.cpp:203-238`) заново считает хеш файлов по этим путям, получает ошибку `platform::open_regular: … No such file or directory` и записывает `identity_check_error`, а `execution_changed_during_run` остаётся пустым. Тест `native_research_recording` требует `false` и падает: `an unmodified run keeps its startup execution identity`.

Содержимое файлов в текущей копии совпадает с записанным при съёмке. Хеши `examples/research/rider_joint_envelope_anatomical.json` = `4e9cacbc…` и `rider_strength_reference.json` = `09213126…` равны хешам в `research_flat/assets.json`. Значит, переносить нужно только начало пути.

Тот же путь записан и в эталонах: `tests/fixtures/golden/darwin-arm64/native_research_recording/shared_export_native_rows_and_complete_provenance.bin` и `native_research_environment/reset_seed_generation_snapshot_and_facade_ownership.bin`, под ключами `.resolved_config.physics.articulated.joint_envelope_path` и `…joint_strength_path`. Поэтому начало пути переносим с обеих сторон: и в загружаемых данных, и в записанных эталонах при сравнении. Сами эталоны не перезаписываем.

**Файлы:**
- Изменить: `native/tests/reference/ft2_reference_support.hpp` (новый помощник рядом с `dir()`; `research_bootstrap()` около строки 120; `consume_tree()` около строки 515)

- [x] **Шаг 1: Убедись, что тест красный по названной причине**

```bash
ctest --test-dir build/native-release -R '^reference.native_research_recording$' --output-on-failure 2>&1 | grep FAIL
jq '.research | {execution_changed_during_run, identity_check_error}' "$(cd "$TMPDIR" && pwd)/ft2_dbg_recording/summary.json"
```

Ожидаемо: `FAIL an unmodified run keeps its startup execution identity`, а в `identity_check_error` — путь с `pfc-lane-f`.

- [x] **Шаг 2: Добавь помощник переноса пути**

Сразу после функции `dir()` (около строки 78):

```cpp
// The research fixtures were captured in another worktree and record its
// absolute examples/research/*.json paths. The end-of-run identity receipt
// re-hashes those paths, so a capture-time path that no longer exists turns
// the receipt into identity_check_error. Rebase the capture-time prefix onto
// this checkout; the file bytes stay pinned by the assets.json sha256 set.
// Paths without the examples/research/ anchor are returned unchanged.
[[nodiscard]] inline std::string rebase_capture_path(std::string_view recorded) {
    constexpr std::string_view kAnchor = "/examples/research/";
    const auto at = recorded.rfind(kAnchor);
    if (at == std::string_view::npos) return std::string(recorded);
    // fixture("ft2") is <repo>/tests/fixtures/ft2.
    const fs::path repo =
        bt::fixture("ft2").parent_path().parent_path().parent_path();
    return (repo / "examples" / "research" /
            std::string(recorded.substr(at + kAnchor.size())))
        .string();
}
```

- [x] **Шаг 3: Перенеси пути в `research_bootstrap()`**

В `research_bootstrap()` сразу после строки `auto restored = *metadata;` вставь:

```cpp
        // identity_receipt() re-hashes these two paths at the end of a run.
        {
            auto resolved = runtime_reference::member(restored, "resolved_config");
            auto physics = runtime_reference::member(resolved, "physics");
            auto articulated = runtime_reference::member(physics, "articulated");
            for (const auto *key : {"joint_envelope_path", "joint_strength_path"}) {
                const auto *path = bio::find_member(
                    bio::as_object(articulated, "articulated"), key);
                if (path != nullptr &&
                    std::holds_alternative<std::string>(path->data))
                    runtime_reference::set_member(
                        articulated, key,
                        bio::Value{rebase_capture_path(
                            std::get<std::string>(path->data))});
            }
            runtime_reference::set_member(physics, "articulated", std::move(articulated));
            runtime_reference::set_member(resolved, "physics", std::move(physics));
            runtime_reference::set_member(restored, "resolved_config", std::move(resolved));
        }
```

Хеш настроек (`configuration_sha256`) от этого не меняется: при его подсчёте пути заменяются хешами файлов. Так устроено в этом же файле на строках 156–176 и в `metadata.cpp`.

- [x] **Шаг 4: Перенеси пути в записанных эталонах при сравнении**

В `consume_tree()`, в цикле `for (const auto &[path, leaf] : *recorded_map)`, замени строку `kept.emplace_back(path, leaf);` на:

```cpp
        // Recorded capture-time paths rebase exactly like the bootstrap's
        // (rebase_capture_path) so both sides name this checkout's files.
        bio::Value normalized = leaf;
        if ((leaf_name == "joint_envelope_path" ||
             leaf_name == "joint_strength_path") &&
            std::holds_alternative<std::string>(leaf.data))
            normalized = bio::Value{
                rebase_capture_path(std::get<std::string>(leaf.data))};
        kept.emplace_back(path, std::move(normalized));
```

- [x] **Шаг 5: Собери и прогони все тесты, которые пользуются этим файлом**

```bash
cmake --build --preset release
ctest --test-dir build/native-release -R 'native_research_(capture|environment|recording)$|native_runtime_(commands|rollout|step)$|native_viewer_snapshot$' --output-on-failure 2>&1 | tail -12
```

Ожидаемо: все 7 зелёные.

Если какой-то тест упадёт на сравнении через `consume_canonical()` и в различии видны `joint_*_path`, значит путь попадает ещё и в побайтовое сравнение документа. Тогда в `consume_canonical()` перед вызовом `node.bytes(...)` верни записанное начало пути в тексте `canonical`: заменой строки, где ищется результат `rebase_capture_path(x)`, а подставляется исходный `x`. Для этого заведи в `research_bootstrap()` статический список пар «исходный путь → перенесённый» (`inline std::vector<std::pair<std::string,std::string>> &rebased_paths()`), пополняй его на шаге 3 и обходи в `consume_canonical()`. После этого прогони снова.

- [x] **Шаг 6: Убедись, что все обычные тесты зелёные**

```bash
ctest --test-dir build/native-release -LE 'slow|package' -j8 --output-on-failure 2>&1 | tail -4
```

Ожидаемо: `100% tests passed, 0 tests failed out of 183`. Это первая зелёная отметка обычных тестов на `main3`.

- [x] **Шаг 7: Коммит**

```bash
git add native/tests/reference/ft2_reference_support.hpp
git commit -m "fix(test): rebase capture-time research asset paths

The ft2 research fixtures record absolute examples/research paths from
the deleted pfc-lane-f worktree. identity_receipt() re-hashes them at the
end of a run, so the receipt reported identity_check_error instead of
'unchanged'. Rebase the prefix onto this checkout on both the bootstrap
and the recorded golden side; the bytes are pinned by assets.json.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 3: Порядок видов проверки, продолжение и время в `run_native_checks.sh`

Сейчас порядок такой: `frontends tidy analyzer cppcheck odr scripts headers diagnostic-controls context`, затем `contracts`. Новый порядок — от дешёвого к дорогому по замерам 2026-11-03: `scripts cppcheck context diagnostic-controls contracts frontends headers odr tidy analyzer`.

`contracts` не читает итоги других видов: в `check_contracts.cpp` файл `native_check_summaries` упоминается только при записи собственного итога, строки 436 и 1446. Поэтому его можно ставить раньше. Перед правкой проверь это ещё раз той же командой `git grep`.

**Файлы:**
- Изменить: `tools/run_native_checks.sh` (целиком)
- Изменить: `native/tests/tooling/tooling_platform.cmake:1499-1502` (строка порядка)
- Создать: `native/tests/tooling/tooling_gate.cmake`
- Изменить: `native/cmake/Checks.cmake` (регистрация `tooling_gate`)

**Что получают следующие задачи:**
- `tools/run_native_checks.sh [--from KIND] <build_dir>`.
- Если задана переменная `BIKE_GATE_TIMINGS_FILE`, на каждый вид в этот файл дописывается строка `<вид>\t<секунды>\t<код выхода>`.
- В выводе есть строка `native launcher: PARTIAL run: …`, если был `--from`.

- [x] **Шаг 1: Напиши тест на заглушке**

Создай `native/tests/tooling/tooling_gate.cmake`:

```cmake
# tooling_gate — fast contract for the gate launchers (tools/run_tests.sh,
# tools/run_native_checks.sh) driven against stub cmake/ctest/checks
# executables. Runs no compiler and no real sweep, so it stays in the
# ordinary (non-slow) CTest set.
#
# Required -D inputs: REPO_ROOT, SCRATCH_DIR.
foreach(_required REPO_ROOT SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_gate requires -D${_required}")
  endif()
endforeach()
file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")
set(_failures "")

# gate_run(<name> EXPECT_EXIT <rc|FAIL> [EXPECT_OUTPUT <needle>...]
#          [ENV <k=v>...] COMMAND <argv...>)
function(gate_run name)
  cmake_parse_arguments(G "" "EXPECT_EXIT" "EXPECT_OUTPUT;ENV;COMMAND" ${ARGN})
  execute_process(COMMAND "${CMAKE_COMMAND}" -E env ${G_ENV} ${G_COMMAND}
    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
  set(_all "${_out}\n${_err}")
  set(${name}_output "${_all}" PARENT_SCOPE)
  if(G_EXPECT_EXIT STREQUAL "FAIL")
    if(_rc EQUAL 0)
      set(_failures "${_failures}\n  ${name}: expected failure, exited 0" PARENT_SCOPE)
    endif()
  elseif(NOT _rc EQUAL ${G_EXPECT_EXIT})
    set(_failures "${_failures}\n  ${name}: expected exit ${G_EXPECT_EXIT}, got ${_rc}:\n${_all}" PARENT_SCOPE)
  endif()
  foreach(_needle IN LISTS G_EXPECT_OUTPUT)
    string(FIND "${_all}" "${_needle}" _pos)
    if(_pos EQUAL -1)
      set(_failures "${_failures}\n  ${name}: output missing '${_needle}':\n${_all}" PARENT_SCOPE)
    endif()
  endforeach()
endfunction()

# expect_file_lines(<name> <file> <expected;lines>) — exact line list.
function(expect_file_lines name file)
  set(_got "")
  if(EXISTS "${file}")
    file(STRINGS "${file}" _got)
  endif()
  if(NOT "${_got}" STREQUAL "${ARGN}")
    set(_failures "${_failures}\n  ${name}: ${file} is '${_got}', expected '${ARGN}'" PARENT_SCOPE)
  endif()
endfunction()

# A stub bike-native-checks: appends its --kind to calls.log and exits with
# the code named by STUB_FAIL_<kind> (dashes become underscores), else 0.
set(_stub_dir "${SCRATCH_DIR}/stubs")
file(MAKE_DIRECTORY "${_stub_dir}")
# Shell ${...} inside these CMake strings is escaped as \${...} so CMake does
# not expand it at write time (same convention as b29_mock_tool).
file(WRITE "${_stub_dir}/bike-native-checks"
  "#!/usr/bin/env bash\n"
  "kind=''\n"
  "while (($#)); do [[ $1 == --kind ]] && kind=$2; shift; done\n"
  "printf 'checks %s jobs=%s\\n' \"$kind\" \"\${BIKE_CHECK_JOBS:-unset}\" >> \"\${STUB_LOG}\"\n"
  "var=\"STUB_FAIL_\${kind//-/_}\"\n"
  "exit \"\${!var:-0}\"\n")
file(CHMOD "${_stub_dir}/bike-native-checks"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

set(_build "${SCRATCH_DIR}/build")
file(MAKE_DIRECTORY "${_build}")
set(_log "${SCRATCH_DIR}/calls.log")
set(_order scripts cppcheck context diagnostic-controls contracts
  frontends headers odr tidy analyzer)
set(_expected "")
foreach(_k IN LISTS _order)
  list(APPEND _expected "checks ${_k} jobs=unset")
endforeach()
set(_checks_env "BIKE_NATIVE_CHECKS=${_stub_dir}/bike-native-checks"
  "STUB_LOG=${_log}" --unset=BIKE_CHECK_JOBS --unset=BIKE_GATE_TIMINGS_FILE)

# --- run_native_checks.sh: cheapest kinds first ---------------------------
file(REMOVE "${_log}")
gate_run(checks_order EXPECT_EXIT 0 ENV ${_checks_env}
  EXPECT_OUTPUT "all produced sweeps passed"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" "${_build}")
expect_file_lines(checks_order "${_log}" ${_expected})

# --- --from skips every earlier kind and marks the run partial -----------
file(REMOVE "${_log}")
gate_run(checks_from EXPECT_EXIT 0 ENV ${_checks_env}
  EXPECT_OUTPUT "PARTIAL run"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" --from odr "${_build}")
expect_file_lines(checks_from "${_log}"
  "checks odr jobs=unset" "checks tidy jobs=unset" "checks analyzer jobs=unset")

gate_run(checks_from_unknown EXPECT_EXIT FAIL ENV ${_checks_env}
  EXPECT_OUTPUT "unknown --from kind: bogus"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" --from bogus "${_build}")

# --- a failing kind stops the run with its exit code ----------------------
file(REMOVE "${_log}")
gate_run(checks_stop EXPECT_EXIT 3 ENV ${_checks_env} "STUB_FAIL_cppcheck=3"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" "${_build}")
expect_file_lines(checks_stop "${_log}"
  "checks scripts jobs=unset" "checks cppcheck jobs=unset")

# --- per-kind timings ------------------------------------------------------
set(_timings "${SCRATCH_DIR}/timings.tsv")
file(REMOVE "${_timings}")
gate_run(checks_timings EXPECT_EXIT 0
  ENV ${_checks_env} "BIKE_GATE_TIMINGS_FILE=${_timings}"
  COMMAND bash "${REPO_ROOT}/tools/run_native_checks.sh" --from tidy "${_build}")
file(STRINGS "${_timings}" _timing_lines)
list(LENGTH _timing_lines _timing_count)
if(NOT _timing_count EQUAL 2)
  set(_failures "${_failures}\n  checks_timings: expected 2 lines, got '${_timing_lines}'")
endif()
foreach(_line IN LISTS _timing_lines)
  if(NOT _line MATCHES "^(tidy|analyzer)\t[0-9]+\t0$")
    set(_failures "${_failures}\n  checks_timings: malformed line '${_line}'")
  endif()
endforeach()

# (run_tests.sh cases are appended by later tasks above this line.)

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_gate contract failed:${_failures}")
endif()
message(STATUS "tooling_gate contract passed")
```

В `native/cmake/Checks.cmake` перед блоком `# --- tooling_checks:` добавь:

```cmake
# --- tooling_gate: launcher contract on stub executables ---------------------
# Fast (no compiler, no real sweep): stays in the ordinary CTest set so the
# gate launchers are covered by every quick run.
if(BUILD_TESTING)
  add_test(NAME tooling_gate
    COMMAND "${CMAKE_COMMAND}"
      -D "REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_gate"
      -P "${_checks_native_dir}/tests/tooling/tooling_gate.cmake")
  set_tests_properties(tooling_gate PROPERTIES LABELS "tooling" TIMEOUT 120)
endif()
```

- [x] **Шаг 2: Убедись, что тест красный**

```bash
cmake --preset release >/dev/null
ctest --test-dir build/native-release -R '^tooling_gate$' --output-on-failure 2>&1 | tail -20
```

Ожидаемо: FAIL. В `checks_order` порядок другой, `checks_from` и `checks_timings` не находят `--from`.

- [x] **Шаг 3: Перепиши `tools/run_native_checks.sh`**

```bash
#!/usr/bin/env bash
# tools/run_native_checks.sh [--from KIND] <build_dir> — run the produced
# native check sweeps through bike-native-checks, cheapest first, so a
# failure surfaces before the expensive kinds run (measured 2026-11-03:
# cppcheck ~1 min, frontends ~9, headers ~15, odr ~19, tidy ~33,
# analyzer the longest):
#   scripts cppcheck context diagnostic-controls contracts frontends
#   headers odr tidy analyzer
#
# --from KIND skips every kind before KIND. A --from run is an
# intermediate re-check after a fix and says so (PARTIAL); the final
# verification always runs every kind.
#
# BIKE_GATE_TIMINGS_FILE, when set, receives one
# "<kind>\t<seconds>\t<exit code>" line per executed kind.
#
# Each produced kind fails closed on missing tools, malformed contexts,
# stale manifests, or blocking findings. Tool overrides are picked up from
# the emitted context (configure-time CLANG_TIDY / ANALYZER_CLANG /
# CPPCHECK / GXX environment).
set -euo pipefail

# shellcheck source=/dev/null
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)/native_common.sh"

kinds=(scripts cppcheck context diagnostic-controls contracts frontends
    headers odr tidy analyzer)

from=""
if [[ "${1:-}" == "--from" ]]; then
    [[ $# -ge 2 ]] || bike_die "--from needs a kind"
    from="$2"
    shift 2
    known=0
    for kind in "${kinds[@]}"; do
        [[ "${kind}" == "${from}" ]] && known=1
    done
    ((known)) || bike_die "unknown --from kind: ${from} (choose from: ${kinds[*]})"
fi

build_dir="${1:-}"
if [[ -z "${build_dir}" ]]; then
    bike_die "usage: tools/run_native_checks.sh [--from KIND] <build_dir>"
fi
if [[ ! -d "${build_dir}" ]]; then
    bike_die "build directory does not exist: ${build_dir}"
fi
build_dir="$(cd -- "${build_dir}" >/dev/null 2>&1 && pwd -P)"
repo_root="$(bike_repo_root)"
checks="$(bike_checks_binary "${build_dir}")"

skipping=0
[[ -n "${from}" ]] && skipping=1
for kind in "${kinds[@]}"; do
    if ((skipping)); then
        if [[ "${kind}" != "${from}" ]]; then
            bike_log "sweep ${kind} skipped (--from ${from})"
            continue
        fi
        skipping=0
    fi
    bike_log "sweep ${kind}"
    args=(--kind "${kind}" --build "${build_dir}")
    [[ "${kind}" == contracts ]] && args+=(--repo-root "${repo_root}")
    started=${SECONDS}
    rc=0
    "${checks}" "${args[@]}" || rc=$?
    elapsed=$((SECONDS - started))
    bike_log "sweep ${kind} finished in ${elapsed}s (exit ${rc})"
    if [[ -n "${BIKE_GATE_TIMINGS_FILE:-}" ]]; then
        printf '%s\t%s\t%s\n' "${kind}" "${elapsed}" "${rc}" \
            >> "${BIKE_GATE_TIMINGS_FILE}"
    fi
    ((rc == 0)) || exit "${rc}"
done
if [[ -n "${from}" ]]; then
    bike_log "PARTIAL run: kinds before ${from} were skipped; not a final verification"
fi
bike_log "all produced sweeps passed: ${build_dir}"
```

- [x] **Шаг 4: Обнови строку порядка в `tooling_platform.cmake`**

Замени два `_expect_needle` на строках 1500–1502:

```cmake
_expect_needle(${_c} "${_checks_sh}"
  "kinds=(scripts cppcheck context diagnostic-controls contracts frontends")
_expect_needle(${_c} "${_checks_sh}" "headers odr tidy analyzer)")
```

Поправь и комментарий над ними: «порядок — от дешёвого к дорогому; набор не меняется».

- [x] **Шаг 5: Прогони тесты**

```bash
cmake --preset release >/dev/null
ctest --test-dir build/native-release -R '^(tooling_gate|tooling_platform)$' --output-on-failure 2>&1 | tail -6
build/native-release/bin/bike-native-checks --kind scripts --build build/native-release | tail -2
```

Ожидаемо: оба теста зелёные; `scripts` (shellcheck) зелёный. Неудачи `tooling_platform`, записанные в задаче 0, могут остаться, но новых быть не должно.

- [x] **Шаг 6: Коммит**

```bash
git add tools/run_native_checks.sh native/tests/tooling/tooling_gate.cmake native/tests/tooling/tooling_platform.cmake native/cmake/Checks.cmake
git commit -m "feat(gate): order sweeps cheapest first, add --from and timings

A late failure used to cost the full serial chain: on 2026-11-03 a
cppcheck failure surfaced after ~1h45 of already-green sweeps. Same
sweep set, new order; --from resumes an intermediate re-check and marks
it PARTIAL.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 4: `run_tests.sh` — обычные тесты первыми, `--from`, таблица времени и журнал

**Файлы:**
- Изменить: `tools/run_tests.sh`
- Изменить: `native/tests/tooling/tooling_gate.cmake` (случаи для `run_tests.sh`)

**Что получают следующие задачи:**
- `bash tools/run_tests.sh <quick|native|full> [preset] [--from STAGE]`. Стадии после сборки: `ctest`, затем виды из задачи 3, затем `slow` и `package`; последние две только для `full`.
- Каталог состояния `BIKE_GATE_STATE_DIR`. По умолчанию это `<git common dir>/bike-gate`, а вне git — `<repo>/build/bike-gate`.
- Журнал: `$BIKE_GATE_STATE_DIR/timings.tsv`, одна строка на прогон.

- [x] **Шаг 1: Допиши тесты в `tooling_gate.cmake`**

Вставь перед строкой `# (run_tests.sh cases are appended …)`:

```cmake
# --- run_tests.sh against a scratch checkout with stub cmake/ctest --------
# The launcher resolves REPO_ROOT from its own location, so copy the three
# launcher files into a scratch tree and put stub cmake/ctest first on PATH.
set(_repo "${SCRATCH_DIR}/repo")
file(MAKE_DIRECTORY "${_repo}/tools" "${_repo}/build/native-release")
file(COPY "${REPO_ROOT}/tools/run_tests.sh" "${REPO_ROOT}/tools/run_native_checks.sh"
          "${REPO_ROOT}/tools/native_common.sh" DESTINATION "${_repo}/tools")
foreach(_tool cmake ctest)
  file(WRITE "${_stub_dir}/${_tool}"
    "#!/usr/bin/env bash\n"
    "printf '${_tool} %s\\n' \"$*\" >> \"\${STUB_LOG}\"\n"
    "exit 0\n")
  file(CHMOD "${_stub_dir}/${_tool}" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
endforeach()
set(_state "${SCRATCH_DIR}/state")
set(_gate_env ${_checks_env} "PATH=${_stub_dir}:$ENV{PATH}"
  "BIKE_GATE_STATE_DIR=${_state}" "BIKE_CPU_COUNT=8")

# native: configure, build, ordinary CTest, then every kind in order.
file(REMOVE "${_log}")
file(REMOVE_RECURSE "${_state}")
gate_run(gate_native EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "stage" "total"
  COMMAND bash "${_repo}/tools/run_tests.sh" native)
file(STRINGS "${_log}" _calls)
list(GET _calls 0 _c0)
list(GET _calls 1 _c1)
list(GET _calls 2 _c2)
list(GET _calls 3 _c3)
if(NOT _c0 MATCHES "^cmake --preset release$"
   OR NOT _c1 MATCHES "^cmake --build --preset release --parallel 8$"
   OR NOT _c2 MATCHES "^ctest --test-dir .* -LE slow\\|package "
   OR NOT _c3 MATCHES "^checks scripts ")
  set(_failures "${_failures}\n  gate_native: unexpected call order '${_calls}'")
endif()
file(STRINGS "${_state}/timings.tsv" _journal)
list(LENGTH _journal _journal_count)
if(NOT _journal_count EQUAL 1 OR NOT _journal MATCHES "\tnative\trelease\t-\t8\tpassed\t")
  set(_failures "${_failures}\n  gate_native: journal '${_journal}'")
endif()

# --from tidy: configure and build still run, ordinary CTest is skipped.
file(REMOVE "${_log}")
gate_run(gate_from EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "PARTIAL"
  COMMAND bash "${_repo}/tools/run_tests.sh" native release --from tidy)
file(STRINGS "${_log}" _calls)
string(FIND "${_calls}" "ctest " _ctest_pos)
string(FIND "${_calls}" "checks cppcheck" _cpp_pos)
string(FIND "${_calls}" "checks tidy" _tidy_pos)
if(NOT _ctest_pos EQUAL -1 OR NOT _cpp_pos EQUAL -1 OR _tidy_pos EQUAL -1)
  set(_failures "${_failures}\n  gate_from: unexpected calls '${_calls}'")
endif()
file(STRINGS "${_state}/timings.tsv" _journal)
list(GET _journal -1 _last)
if(NOT _last MATCHES "\tnative\trelease\ttidy\t8\tpassed\t")
  set(_failures "${_failures}\n  gate_from: journal line '${_last}'")
endif()

# --from is meaningless for quick and must name a known stage.
gate_run(gate_from_quick EXPECT_EXIT 2 ENV ${_gate_env}
  COMMAND bash "${_repo}/tools/run_tests.sh" quick release --from tidy)
gate_run(gate_from_bogus EXPECT_EXIT 2 ENV ${_gate_env}
  COMMAND bash "${_repo}/tools/run_tests.sh" native release --from bogus)
gate_run(gate_from_slow_native EXPECT_EXIT 2 ENV ${_gate_env}
  COMMAND bash "${_repo}/tools/run_tests.sh" native release --from slow)

# A failure is journaled as failed and keeps the exit code.
gate_run(gate_failed EXPECT_EXIT 5 ENV ${_gate_env} "STUB_FAIL_odr=5"
  COMMAND bash "${_repo}/tools/run_tests.sh" native)
file(STRINGS "${_state}/timings.tsv" _journal)
list(GET _journal -1 _last)
if(NOT _last MATCHES "\tfailed\t")
  set(_failures "${_failures}\n  gate_failed: journal line '${_last}'")
endif()
```

- [x] **Шаг 2: Убедись, что новые случаи красные**

```bash
ctest --test-dir build/native-release -R '^tooling_gate$' --output-on-failure 2>&1 | grep -E 'gate_' | head
```

Ожидаемо: `gate_native` (нет `--parallel 8`, нет журнала), `gate_from*`, `gate_failed`.

- [x] **Шаг 3: Перепиши разбор ключей и стадии в `run_tests.sh`**

Сохрани верхний комментарий и `usage()`. Дополни комментарий абзацем о стадиях, `--from`, журнале. В `usage()` добавь строки:

```
  --from STAGE  native/full only: an intermediate re-check after a fix.
                Configure and build always run; earlier stages are skipped
                and the run is marked PARTIAL. Stages: ctest scripts cppcheck
                context diagnostic-controls contracts frontends headers odr
                tidy analyzer, plus slow package for full.
```

Замени блок от `profile="${1:-quick}"` до конца файла на:

```bash
profile="${1:-quick}"
preset="release"
from=""
if (($# > 0)); then shift; fi
if (($# > 0)) && [[ "$1" != --* ]]; then
  preset="$1"
  shift
fi
while (($# > 0)); do
  case "$1" in
    --from)
      (($# >= 2)) || { printf -- '--from needs a stage\n' >&2; usage >&2; exit 2; }
      from="$2"
      shift 2
      ;;
    *)
      printf 'Unexpected argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done
case "$profile" in
  quick|native|full) ;;
  -h|--help) usage; exit 0 ;;
  *) printf 'Unknown test profile: %s\n' "$profile" >&2; usage >&2; exit 2 ;;
esac
case "$preset" in
  release|debug|gui|asan|rtsan|coverage) ;;
  *) printf 'Unknown preset: %s\n' "$preset" >&2; usage >&2; exit 2 ;;
esac

# Stages after configure+build, in execution order. The sweep kinds are
# owned by tools/run_native_checks.sh; keep this list in the same order.
sweep_kinds=(scripts cppcheck context diagnostic-controls contracts
  frontends headers odr tidy analyzer)
stages=(ctest "${sweep_kinds[@]}")
[[ "$profile" == full ]] && stages+=(slow package)
if [[ -n "$from" ]]; then
  if [[ "$profile" == quick ]]; then
    printf -- '--from applies to native and full only\n' >&2
    exit 2
  fi
  known=0
  for stage_name in "${stages[@]}"; do
    [[ "$stage_name" == "$from" ]] && known=1
  done
  if ((!known)); then
    printf 'Unknown --from stage for %s: %s (choose from: %s)\n' \
      "$profile" "$from" "${stages[*]}" >&2
    exit 2
  fi
fi
# stage_index <name> -> position in stages (or 999 when absent)
stage_index() {
  local i
  for i in "${!stages[@]}"; do
    [[ "${stages[$i]}" == "$1" ]] && { printf '%s\n' "$i"; return; }
  done
  printf '999\n'
}
from_index=0
[[ -n "$from" ]] && from_index="$(stage_index "$from")"
wanted() { (( $(stage_index "$1") >= from_index )); }

build_dir="$REPO_ROOT/build/$preset"
if [[ "$preset" == release ]]; then
  build_dir="$REPO_ROOT/build/native-release"
fi

# Shared gate state (journal; active-gate registry in later tasks): the git
# common dir is shared by every worktree of this repository.
if [[ -z "${BIKE_GATE_STATE_DIR:-}" ]]; then
  if common_dir="$(git -C "$REPO_ROOT" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)"; then
    BIKE_GATE_STATE_DIR="$common_dir/bike-gate"
  else
    BIKE_GATE_STATE_DIR="$REPO_ROOT/build/bike-gate"
  fi
fi
mkdir -p "$BIKE_GATE_STATE_DIR"
cpu_count="${BIKE_CPU_COUNT:-$(getconf _NPROCESSORS_ONLN)}"
jobs="$cpu_count"

timings_file="$(mktemp "${TMPDIR:-/tmp}/bike-gate-timings.XXXXXX")"
export BIKE_GATE_TIMINGS_FILE="$timings_file"
gate_started=$SECONDS

# stage <name> <command...> — run one stage and record its duration.
stage() {
  local name="$1" started=$SECONDS rc=0
  shift
  "$@" || rc=$?
  printf '%s\t%s\t%s\n' "$name" "$((SECONDS - started))" "$rc" >> "$timings_file"
  return "$rc"
}

finish() {
  local rc=$?
  local total=$((SECONDS - gate_started)) status=passed summary=""
  ((rc == 0)) || status=failed
  printf '\n%-22s %8s %6s\n' stage seconds exit
  while IFS=$'\t' read -r name seconds stage_rc; do
    printf '%-22s %8s %6s\n' "$name" "$seconds" "$stage_rc"
    summary+="${summary:+,}${name}=${seconds}"
  done < "$timings_file"
  printf '%-22s %8s %6s\n' total "$total" "$rc"
  if [[ -n "$from" ]]; then
    printf 'PARTIAL run (--from %s): not a final verification\n' "$from"
  fi
  local head
  head="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || printf 'nogit')"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$REPO_ROOT" "$head" "$profile" \
    "$preset" "${from:--}" "$jobs" "$status" "$total" "$summary" \
    >> "$BIKE_GATE_STATE_DIR/timings.tsv"
  rm -f "$timings_file"
  exit "$rc"
}
trap finish EXIT

# Darwin allocator debugging wraps only the ordinary `ctest -LE
# 'slow|package'` invocation — wherever a profile runs it — in a
# subshell. Never exported on Linux, and never around `ctest -L slow`,
# which contains realtime measurements (Task 3.10).
run_ordinary_leg() {
  if [[ "$(uname -s)" == Darwin ]]; then
    (
      export MallocScribble=1 MallocPreScribble=1 MallocGuardEdges=1
      "$@"
    )
  else
    "$@"
  fi
}

run_native_gate() {
  stage configure cmake --preset "$preset"
  stage build cmake --build --preset "$preset" --parallel "$jobs"
  printf 'selected native build: %s\n' "$build_dir"
  export NATIVE_TEST_BUILD_PATH="$build_dir"
  # The ordinary tests run first: ~1 min, and they catch what would
  # otherwise surface only after every sweep (2026-11-03: a 2h21m gate
  # failed on two tests that fail in under 5 s).
  if wanted ctest; then
    local ctest_jobs=$((jobs < 8 ? jobs : 8))
    stage ctest run_ordinary_leg ctest --test-dir "$build_dir" -LE 'slow|package' \
      -j"$ctest_jobs" --no-tests=error --output-on-failure \
      --output-junit "$build_dir/Testing/junit.xml"
  fi
  local first_kind=""
  for kind in "${sweep_kinds[@]}"; do
    if wanted "$kind"; then first_kind="$kind"; break; fi
  done
  if [[ -n "$first_kind" ]]; then
    local from_args=()
    [[ "$first_kind" != "${sweep_kinds[0]}" ]] && from_args=(--from "$first_kind")
    bash "$REPO_ROOT/tools/run_native_checks.sh" "${from_args[@]}" "$build_dir"
  fi
}

case "$profile" in
  quick)
    # The workflow's test step is the ordinary `ctest -LE 'slow|package'`
    # leg, so the same Darwin allocator debugging wraps the whole invocation;
    # the configure/build steps ignore the variables.
    stage workflow run_ordinary_leg cmake --workflow --preset "$preset"
    ;;
  native)
    run_native_gate
    ;;
  full)
    run_native_gate
    for label in slow package; do
      if wanted "$label"; then
        stage "$label" ctest --test-dir "$build_dir" -L "$label" --no-tests=error \
          --output-on-failure
      fi
    done
    ;;
esac
```

Важно: `run_native_checks.sh` сам дописывает строки времени по каждому виду в тот же `BIKE_GATE_TIMINGS_FILE`, поэтому в таблице они идут вперемешку со стадиями `run_tests.sh` и в правильном порядке.

`--from slow` для `native` отвергается проверкой «известная стадия»: `slow` есть в списке стадий только для `full`.

- [x] **Шаг 4: Прогони тесты и shellcheck**

```bash
ctest --test-dir build/native-release -R '^(tooling_gate|tooling_platform)$' --output-on-failure 2>&1 | tail -6
build/native-release/bin/bike-native-checks --kind scripts --build build/native-release | tail -2
bash tools/run_tests.sh bogus; echo "rc=$?"
bash tools/run_tests.sh native release --from bogus; echo "rc=$?"
```

Ожидаемо: оба теста зелёные; `scripts` зелёный; обе последние команды печатают `rc=2`.

- [x] **Шаг 5: Коммит**

```bash
git add tools/run_tests.sh native/tests/tooling/tooling_gate.cmake
git commit -m "feat(gate): ordinary tests first, --from, stage timings journal

The 1-minute ordinary CTest leg now precedes the sweeps. Every run prints
a per-stage table and appends one line to <git-common-dir>/bike-gate/
timings.tsv so regressions in gate cost are visible.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 5: Убрать экспериментальную половину анализатора

Экспериментальная половина (`-analyzer-checker=alpha`) за прогон выдаёт ~4 650 предупреждений в 453 из 511 файлов, проверку никогда не останавливает и стоит ~34 минуты (замер 2026-11-03). Остаётся только основная половина (`optin` + `cplusplus.Move`), и её находки по-прежнему останавливают проверку.

**Файлы:**
- Изменить: `native/src/tools/check_frontends.cpp`: `tool_argv()` (строки 260–278), функция `report_alpha_findings` (348–370), цикл `run_sweep` (574–660), переменная `report_only_findings` (532, 643, 687), комментарий в шапке (строки 4–5)
- Изменить: `native/tests/tooling/tooling_checks.cmake`: случаи `b29_alpha_report` и `b29_alpha_crash` (строки 1404–1430)

- [x] **Шаг 1: Замени случаи про экспериментальную половину тестом «одна половина»**

В `tooling_checks.cmake` замени блок от `# alpha findings are report-only; …` до `expect_json(b29_alpha_crash …)` включительно на:

```cmake
# The analyzer runs one stable pass per TU; the experimental (alpha)
# checkers were removed from every gate level on 2026-11-03 (thousands of
# never-blocking, unread findings for ~34 min per gate). A tool that would
# crash on -analyzer-checker=alpha must therefore never be asked to.
set(_fx_alpha "${SCRATCH_DIR}/b29-alpha")
file(MAKE_DIRECTORY "${_fx_alpha}/build")
file(COPY "${_fx}/build/native_check_context.json"
          "${_fx}/build/native_sources.json"
          "${_fx}/build/compile_commands.json"
     DESTINATION "${_fx_alpha}/build")
set(_alpha_summary "${_fx_alpha}/build/native_check_summaries/analyzer.json")
b29_mock_tool("${_fx_alpha}/alpha-crash-tool" ALPHA_RC 139
  ALPHA_TEXT "segmentation fault")
run_case(b29_analyzer_single_pass EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "ANALYZER_CLANG=${_fx_alpha}/alpha-crash-tool"
        "${CHECKS_BINARY}" --kind analyzer --build "${_fx_alpha}/build")
expect_json(b29_analyzer_single_pass "${_alpha_summary}" "ok" tool_health)
expect_json(b29_analyzer_single_pass "${_alpha_summary}" "none" finding_status)
expect_json_length(b29_analyzer_single_pass "${_alpha_summary}" 1 records)
expect_json(b29_analyzer_single_pass "${_alpha_summary}" "stable" records 0 pass)
```

- [x] **Шаг 2: Убедись, что случай красный**

`tooling_checks` долгий (задача 0, шаг 3). Чтобы проверить один случай, повтори его команду руками:

```bash
S=build/native-release/tooling_checks/b29-alpha
ls "$S/build" || echo "сначала один раз прогони tooling_checks (задача 0)"
ANALYZER_CLANG="$PWD/$S/alpha-crash-tool" build/native-release/bin/bike-native-checks --kind analyzer --build "$S/build"; echo "rc=$?"
```

Если каталога нет, создай заглушку по образцу `b29_mock_tool` руками (sh-сценарий: `--version` → строка; `*-analyzer-checker=alpha*` → код 139; иначе 0). Ожидаемо сейчас: `rc=1` (экспериментальный проход падает).

- [x] **Шаг 3: Оставь одну половину в `tool_argv()`**

Для `analyzer` замени разветвление `if (pass_name == "stable") {…} else {…}` на безусловное добавление основных настроек:

```cpp
        std::vector<std::string> extra = {"-Xanalyzer", "-analyzer-output=text",
                                          "-Xanalyzer", "-analyzer-checker=optin",
                                          "-Xanalyzer",
                                          "-analyzer-checker=cplusplus.Move"};
```

Параметр `pass_name` у `tool_argv()` теперь не нужен: удали его из объявления, определения и единственного вызова.

- [x] **Шаг 4: Упрости цикл в `run_sweep()`**

- Убери вектор `passes` и внутренний цикл `for (const std::string &pass_name: passes)`. Тело выполняется один раз с `pass_name` = `"stable"`; имя журнала по-прежнему `<index>-<stem>-stable`.
- Убери `stable_failed`, ветку `if (kind == "analyzer" && pass_name == "alpha")`, переменную `report_only_findings` и функцию `report_alpha_findings`.
- `finding_status` теперь `blocking ? "blocking" : "none"`.
- В шапке файла (строки 4–5) убери упоминание «stable/alpha pass split and the report-only alpha contract»; напиши «single stable analyzer pass (alpha checkers removed 2026-11-03)».

После правки проверь, что упоминаний не осталось:

```bash
git grep -n -i 'alpha' -- native/src/tools
```

Ожидаемо: только `check_contracts.cpp:265` — это комментарий о `cplusplus.Move`; уточни его, если он описывает удалённую половину.

- [x] **Шаг 5: Собери и проверь**

```bash
cmake --build --preset release --target bike-native-checks
ANALYZER_CLANG="$PWD/$S/alpha-crash-tool" build/native-release/bin/bike-native-checks --kind analyzer --build "$S/build"; echo "rc=$?"
jq -r '.records | length, .[0].pass' "$S/build/native_check_summaries/analyzer.json"
```

Ожидаемо: `rc=0`, затем `1` и `stable`.

Потом прогони настоящий анализатор на всём коде. Это длинный проход, около 27 минут, пока нет задачи 7:

```bash
time build/native-release/bin/bike-native-checks --kind analyzer --build build/native-release | tail -3
```

Ожидаемо: `tool health: ok`, `findings: none`, время примерно вдвое меньше прежних 60 минут. Запиши время в отчёт задачи 0 (раздел «после задачи 5»).

- [x] **Шаг 6: Коммит**

```bash
git add native/src/tools/check_frontends.cpp native/tests/tooling/tooling_checks.cmake
git commit -m "refactor(checks): drop the experimental analyzer pass

The alpha pass printed ~4650 report-only findings in 453 of 511 TUs on
every run, never blocked, and cost ~34 min per gate. The stable pass
(optin + cplusplus.Move) still blocks on findings.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 6: Перенос помощников учебных сборок в общий модуль

Тесты для задач 7, 9 и 12 должны быть быстрыми. `tooling_checks` проходит настоящие виды проверки по всему коду и идёт 20+ минут, поэтому новые случаи кладём в отдельный быстрый сценарий `tooling_sweep_engine.cmake`. Ему нужны те же помощники, что уже есть в `tooling_checks.cmake`. Переносим их без изменений.

**Файлы:**
- Создать: `native/tests/tooling/fixture_support.cmake`
- Изменить: `native/tests/tooling/tooling_checks.cmake`

- [x] **Шаг 1: Перенеси функции проверки итогов**

Перенеси **дословно**, вместе с комментариями над ними, функции из `tooling_checks.cmake`: `run_case`, `expect_json`, `expect_json_item_names`, `expect_json_contains`, `expect_json_length`, `expect_json_member_set`, `expect_json_equal`, `expect_json_nonzero`, `expect_json_type`, `expect_json_substring`. Это все функции `expect_json*` в файле; найди их через `grep -n '^function' native/tests/tooling/tooling_checks.cmake`.

- [x] **Шаг 2: Перенеси сведения b29 и помощники учебной сборки**

Перенеси **дословно** блок от комментария `# Facts mirrored from the produced context so fixture entries validate the` до конца определения `set(_b29_clean_env …)` включительно. Это вычисление `_b29_*`, функции `b29_fixture_context`, `b29_ccdb_entry`, `b29_ccdb_args`, `b29_mock_tool`. Этому блоку нужна переменная `_context_json`. Добавь в начало модуля:

```cmake
# fixture_support.cmake — shared helpers for the tooling_* cmake -P tests:
# JSON expectation functions and the b29 fixture-build helpers. Requires
# BUILD_DIR (a configured standalone build) and SCRATCH_DIR; callers own
# _failures and the final FATAL_ERROR report.
include_guard(GLOBAL)
set(_fs_ctx "${BUILD_DIR}/native_check_context.json")
if(NOT EXISTS "${_fs_ctx}")
  message(FATAL_ERROR "fixture_support requires the produced build to emit ${_fs_ctx}")
endif()
file(READ "${_fs_ctx}" _context_json)
```

- [x] **Шаг 3: Подключи модуль в `tooling_checks.cmake`**

На месте перенесённых функций (сразу после `set(_failures "")`) поставь:

```cmake
include("${CMAKE_CURRENT_LIST_DIR}/fixture_support.cmake")
```

Строку `file(READ "${_ctx}" _context_json)` около строки 347 оставь: она перечитывает тот же файл, результат тот же.

- [x] **Шаг 4: Проверь, что `tooling_checks` ведёт себя как в задаче 0**

```bash
cmake --preset release >/dev/null
ctest --test-dir build/native-release -R '^tooling_checks$' --output-on-failure 2>&1 | tail -30
```

Ожидаемо: тот же список неудач, что записан в задаче 0, плюс зелёный `b29_analyzer_single_pass`. Новых неудач быть не должно. Этот прогон долгий; он нужен, потому что перенос затрагивает каждый случай.

- [x] **Шаг 5: Коммит**

```bash
git add native/tests/tooling/fixture_support.cmake native/tests/tooling/tooling_checks.cmake
git commit -m "test(tooling): move fixture helpers into fixture_support.cmake

Verbatim move so fast tooling tests can share the b29 fixture builders
without running tooling_checks' real-tree sweeps.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 7: Одновременная проверка файлов в `bike-native-checks`

Каждый вид сейчас запускает инструмент на 511 файлах строго по очереди (`run_sweep`, `run_header_sweep`). Запуск дочерних процессов уже готов к одновременной работе: `platform::run` в `native/src/platform/process.cpp:191-199` защищает создание каналов общей блокировкой, а имена журналов и временные каталоги у каждого файла свои (по номеру).

**Файлы:**
- Создать: `native/src/tools/check_pool.cpp`
- Изменить: `native/src/tools/checks_internal.hpp` (объявления)
- Изменить: `native/cmake/Checks.cmake` (`check_pool.cpp` в `add_executable(bike-native-checks …)`, регистрация `tooling_sweep_engine`)
- Изменить: `native/src/tools/check_frontends.cpp` (`run_sweep`, `run_header_sweep`)
- Создать: `native/tests/tooling/tooling_sweep_engine.cmake`

**Что получают следующие задачи:**
- `std::size_t check_jobs(const std::map<std::string, std::string> &environment);` — значение `BIKE_CHECK_JOBS` или число ядер.
- `void for_each_index(std::size_t count, std::size_t workers, const std::function<void(std::size_t)> &task);`
- В итоге каждого вида появляется поле `"jobs"`.

- [x] **Шаг 1: Напиши быстрый тест**

Создай `native/tests/tooling/tooling_sweep_engine.cmake`:

```cmake
# tooling_sweep_engine — fast fixture contract for bike-native-checks'
# per-file engine: bounded parallelism, deterministic record order,
# BIKE_CHECK_JOBS validation. Mock tools only; no real analysis.
#
# Required -D inputs: CHECKS_BINARY, BUILD_DIR, SCRATCH_DIR.
foreach(_required CHECKS_BINARY BUILD_DIR SCRATCH_DIR)
  if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
    message(FATAL_ERROR "tooling_sweep_engine requires -D${_required}")
  endif()
endforeach()
file(REMOVE_RECURSE "${SCRATCH_DIR}")
file(MAKE_DIRECTORY "${SCRATCH_DIR}")
set(_failures "")
include("${CMAKE_CURRENT_LIST_DIR}/fixture_support.cmake")

# An eight-TU fixture build under one target.
set(_fx "${SCRATCH_DIR}/engine")
file(MAKE_DIRECTORY "${_fx}/native/src" "${_fx}/build")
file(WRITE "${_fx}/suppressions.json" "{\"schema_version\":1,\"suppressions\":[]}\n")
b29_fixture_context("${_fx}/build" "${_fx}/native"
  "{\"bike_native\":${_b29_tc}}" "${_fx}/suppressions.json")
set(_manifest_sources "")
set(_ccdb "")
foreach(_i RANGE 1 8)
  file(WRITE "${_fx}/native/src/tu${_i}.cpp" "int tu${_i}() { return ${_i}; }\n")
  if(NOT _manifest_sources STREQUAL "")
    string(APPEND _manifest_sources ",")
    string(APPEND _ccdb ",")
  endif()
  string(APPEND _manifest_sources
    "{\"path\":\"${_fx}/native/src/tu${_i}.cpp\",\"target\":\"bike_native\",\"context\":\"bike_native:fixture\"}")
  b29_ccdb_entry(_entry "${_fx}" "${_fx}/native/src/tu${_i}.cpp"
    "CMakeFiles/bike_native.dir/tu${_i}.cpp.o")
  string(APPEND _ccdb "${_entry}")
endforeach()
file(WRITE "${_fx}/build/native_sources.json"
  "{\"schema_version\":1,\"first_party_roots\":[\"${_fx}/native\"],\"sources\":[${_manifest_sources}]}\n")
file(WRITE "${_fx}/build/compile_commands.json" "[${_ccdb}]\n")
set(_summary "${_fx}/build/native_check_summaries/tidy.json")

# A mock tool that takes one second per TU.
file(WRITE "${_fx}/slow-tool"
  "#!/bin/sh\n"
  "if [ \"\${1:-}\" = \"--version\" ]; then echo 'mock analysis tool 1'; exit 0; fi\n"
  "sleep 1\nexit 0\n")
file(CHMOD "${_fx}/slow-tool" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)

# Eight TUs on eight workers finish in well under the serial 8 s.
string(TIMESTAMP _t0 "%s")
run_case(engine_parallel EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=8"
        "CLANG_TIDY=${_fx}/slow-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
string(TIMESTAMP _t1 "%s")
math(EXPR _elapsed "${_t1} - ${_t0}")
if(_elapsed GREATER 5)
  set(_failures "${_failures}\n  engine_parallel: 8 x 1 s TUs took ${_elapsed} s on 8 workers")
endif()
expect_json(engine_parallel "${_summary}" 8 jobs)
expect_json_length(engine_parallel "${_summary}" 8 records)
# Records stay in manifest (source path) order regardless of finish order.
foreach(_i RANGE 0 7)
  math(EXPR _n "${_i} + 1")
  expect_json(engine_parallel "${_summary}" "${_fx}/native/src/tu${_n}.cpp"
    records ${_i} source)
endforeach()

# One worker is the old serial behavior.
run_case(engine_serial EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=1"
        "CLANG_TIDY=${_fx}/slow-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
expect_json(engine_serial "${_summary}" 1 jobs)

# A malformed job count fails closed instead of silently serializing.
run_case(engine_jobs_invalid EXPECT_EXIT FAIL
  EXPECT_STDERR "BIKE_CHECK_JOBS must be a positive integer"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=zero"
        "CLANG_TIDY=${_fx}/slow-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")

# A blocking finding in one TU still fails the whole sweep in parallel.
file(WRITE "${_fx}/warn-one-tool"
  "#!/bin/sh\n"
  "if [ \"\${1:-}\" = \"--version\" ]; then echo 'mock analysis tool 1'; exit 0; fi\n"
  "case \"$*\" in *tu5.cpp*) echo 'tu5.cpp:1:1: warning: planted finding' >&2 ;; esac\n"
  "exit 0\n")
file(CHMOD "${_fx}/warn-one-tool" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE)
run_case(engine_finding EXPECT_EXIT FAIL
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=8"
        "CLANG_TIDY=${_fx}/warn-one-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build")
expect_json(engine_finding "${_summary}" "blocking" finding_status)

# (later tasks append cases above this line)

if(NOT _failures STREQUAL "")
  message(FATAL_ERROR "tooling_sweep_engine contract failed:${_failures}")
endif()
message(STATUS "tooling_sweep_engine contract passed")
```

В `Checks.cmake` рядом с `tooling_gate` добавь:

```cmake
if(BUILD_TESTING)
  add_test(NAME tooling_sweep_engine
    COMMAND "${CMAKE_COMMAND}"
      -D "CHECKS_BINARY=$<TARGET_FILE:bike-native-checks>"
      -D "BUILD_DIR=${CMAKE_BINARY_DIR}"
      -D "SCRATCH_DIR=${CMAKE_BINARY_DIR}/tooling_sweep_engine"
      -P "${_checks_native_dir}/tests/tooling/tooling_sweep_engine.cmake")
  set_tests_properties(tooling_sweep_engine PROPERTIES LABELS "tooling" TIMEOUT 300)
endif()
```

Если запуск поддельного инструмента не проходит проверку контекста (например, `tidy` требует ещё полей в учебной сборке), сравни свою учебную сборку со сборкой `b29-entries` в `tooling_checks.cmake` (строки около 1036–1056) и добавь недостающее. Менять проверку контекста нельзя.

- [x] **Шаг 2: Убедись, что тест красный**

```bash
cmake --preset release >/dev/null
cmake --build --preset release --target bike-native-checks
ctest --test-dir build/native-release -R '^tooling_sweep_engine$' --output-on-failure 2>&1 | tail -10
```

Ожидаемо: `engine_parallel` (≈8 с; нет поля `jobs`), `engine_jobs_invalid` (`zero` не отвергается).

- [x] **Шаг 3: Добавь набор исполнителей**

В `checks_internal.hpp` перед разделом `// ---- sweeps` добавь:

```cpp
// ---- worker pool -------------------------------------------------------------

// Concurrent child processes a sweep may run: BIKE_CHECK_JOBS when set (a
// positive decimal integer; anything else is a CheckError so a typo never
// silently serializes or overloads the gate), otherwise the online CPU
// count. tools/run_tests.sh exports a share when other gates are running.
[[nodiscard]] std::size_t check_jobs(
    const std::map<std::string, std::string> &environment);

// Runs task(0) … task(count - 1) on at most `workers` threads, each index
// exactly once. Tasks write into per-index slots; callers merge in index
// order afterwards so summaries stay deterministic. After the first
// exception no new index starts; it is rethrown once all threads joined.
void for_each_index(std::size_t count, std::size_t workers,
                    const std::function<void(std::size_t)> &task);
```

Добавь `#include <functional>` в шапку, если его там нет.

Создай `native/src/tools/check_pool.cpp`:

```cpp
// check_pool.cpp — bounded worker pool for the per-file sweeps.
//
// Child processes go through platform::run, which serializes pipe creation
// (process.cpp) so concurrent spawns never leak sibling pipe ends; per-file
// logs and scratch paths are keyed by the entry index.
#include "checks_internal.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <vector>

namespace bike::checks {

std::size_t check_jobs(const std::map<std::string, std::string> &environment) {
    if (const auto found = environment.find("BIKE_CHECK_JOBS");
        found != environment.end()) {
        const std::optional<std::int64_t> parsed = parse_int64(found->second);
        if (!parsed.has_value() || *parsed < 1)
            throw CheckError("BIKE_CHECK_JOBS must be a positive integer, got '" +
                             found->second + "'");
        return static_cast<std::size_t>(*parsed);
    }
    const long online = ::sysconf(_SC_NPROCESSORS_ONLN);
    return online > 0 ? static_cast<std::size_t>(online) : std::size_t{1};
}

void for_each_index(std::size_t count, std::size_t workers,
                    const std::function<void(std::size_t)> &task) {
    if (count == 0) return;
    const std::size_t threads = std::clamp<std::size_t>(workers, 1, count);
    std::atomic<std::size_t> next{0};
    std::exception_ptr first_error;
    std::mutex error_mutex;
    const auto drain = [&] {
        for (std::size_t index = next.fetch_add(1); index < count;
             index = next.fetch_add(1)) {
            try {
                task(index);
            } catch (...) {
                const std::lock_guard lock(error_mutex);
                if (!first_error) first_error = std::current_exception();
                next.store(count); // hand out no further indexes
            }
        }
    };
    {
        std::vector<std::jthread> pool;
        pool.reserve(threads - 1);
        for (std::size_t worker = 1; worker < threads; ++worker)
            pool.emplace_back(drain);
        drain();
    } // every jthread joins here
    if (first_error) std::rethrow_exception(first_error);
}

} // namespace bike::checks
```

Добавь `"${_checks_tools_dir}/check_pool.cpp"` в список `add_executable(bike-native-checks …)` в `Checks.cmake`. Файл первичный, поэтому попадёт в `native_sources.json`, и его проверят все виды.

- [x] **Шаг 4: Перестрой цикл `run_sweep()` на исполнителей**

Ниже — цикл после задачи 5 (одна половина). Замени лямбду `[&]() { for (std::size_t index = 0; …) {…} }();` на:

```cpp
    // Per-TU outcome slots: workers fill them in any order, the merge below
    // walks them in entry order so records and counts stay deterministic.
    struct TuOutcome {
        Value record;
        std::vector<std::string> suppressed_checkers;
        bool health_failed = false;
        bool blocking = false;
    };
    const std::size_t jobs = check_jobs(environ_copy());
    set_member(summary, "jobs", vint(static_cast<std::int64_t>(jobs)));
    std::vector<TuOutcome> outcomes(entries.size());
    for_each_index(entries.size(), jobs, [&](std::size_t index) {
        const CompileEntry &entry = entries[index];
        TuOutcome &out = outcomes[index];
        const std::vector<std::string> argv =
            tool_argv(kind, tool, entry, context, build_dir, run_id, index,
                      frontend_flags);
        std::string name = padded_index(index);
        name += '-';
        name += entry.source.stem().string();
        name += "-stable";
        Value record = run_process(argv, entry.directory, log_directory, name);
        set_member(record, "pass", vstr("stable"));
        set_member(record, "source", vstr(entry.source.string()));
        set_member(record, "target",
                   entry.target.has_value() ? vstr(*entry.target) : vnull());
        std::vector<std::string> unsuppressed;
        if (kind == "analyzer" || kind == "tidy") {
            const auto filtered = apply_suppressions(
                record_diagnostics(record), tool_version, suppressions);
            unsuppressed = filtered.remaining;
            Array items;
            for (const auto &line: unsuppressed) items.push_back(vstr(line));
            set_member(record, "unsuppressed_diagnostics", varr(std::move(items)));
            set_member(record, "suppressed_diagnostics", varr(filtered.matched));
            for (const Value &item: filtered.matched)
                out.suppressed_checkers.push_back(
                    std::get<std::string>(member(item, "checker")->data));
        } else {
            unsuppressed = record_diagnostics(record);
        }
        out.health_failed = !record_health_ok(record);
        out.blocking = has_warning(unsuppressed) || has_error(unsuppressed);
        out.record = std::move(record);
    });
    for (TuOutcome &out: outcomes) {
        for (const std::string &checker: out.suppressed_checkers)
            count_suppression(summary, checker);
        tool_health_failed = tool_health_failed || out.health_failed;
        blocking_findings = blocking_findings || out.blocking;
        records.push_back(std::move(out.record));
    }
```

Подсчёт подавлений, который раньше был встроен в цикл, вынеси без изменений в функцию анонимного пространства имён над `run_sweep`:

```cpp
// Increments summary.suppression_counts[checker] (insertion-ordered object).
void count_suppression(Value &summary, const std::string &checker) {
    Value *counts = member_mut(summary, "suppression_counts");
    auto &counts_object = std::get<Object>(counts->data);
    for (auto &[key, value]: counts_object) {
        if (key == checker) {
            const std::int64_t count =
                parse_int64(std::get<io::Number>(value.data).lexeme).value_or(0);
            value = vint(count + 1);
            return;
        }
    }
    counts_object.emplace_back(checker, vint(1));
}
```

Проверь, что `tool_argv()` для каждого вида пишет только в пути со своим `index`: временная база данных компиляции `entry-<index>` для `tidy`/`cppcheck`, объект `<index>.o` для `odr`. Если какой-то вид пишет общий файл, его надо сделать своим для каждого номера.

- [x] **Шаг 5: То же для `run_header_sweep()`**

Сначала собери список заданий: пары (заголовок, условие компиляции), как в нынешнем двойном цикле, с номером. Затем выполни их через `for_each_index` в слоты `std::vector<Value>` и слей по порядку. Временный каталог `header-<index>-<run_id>` и имя журнала уже уникальны по номеру. `jobs` возьми так же: `check_jobs(environ_copy())`, и запиши в итог.

- [x] **Шаг 6: Прогони тесты**

```bash
cmake --preset release >/dev/null
cmake --build --preset release --target bike-native-checks
ctest --test-dir build/native-release -R '^(tooling_sweep_engine|tooling_gate)$' --output-on-failure 2>&1 | tail -6
```

Ожидаемо: оба зелёные.

- [x] **Шаг 7: Замерь настоящие виды**

```bash
for k in cppcheck frontends headers odr tidy analyzer; do
  /usr/bin/time -p build/native-release/bin/bike-native-checks --kind $k --build build/native-release 2>&1 | grep -E 'tool health|findings|^real'
done
```

Ожидаемо: везде `tool health: ok` и `findings: none`. Запиши время по каждому виду в отчёт. Сумма должна быть порядка 15–25 минут против прежних ~137. Если машина начинает задыхаться (своп, `vm_stat` показывает много pageouts), попробуй `BIKE_CHECK_JOBS=8`, запиши оба замера и сообщи владельцу. Значение по умолчанию без согласия не меняй.

- [x] **Шаг 8: Долгий тест `tooling_checks`**

```bash
ctest --test-dir build/native-release -R '^tooling_checks$' --output-on-failure 2>&1 | tail -30
```

Ожидаемо: список неудач как в задаче 6, шаг 4. Теперь он идёт быстрее, потому что настоящие виды внутри стали одновременными.

- [x] **Шаг 9: Коммит**

```bash
git add native/src/tools/check_pool.cpp native/src/tools/checks_internal.hpp native/src/tools/check_frontends.cpp native/cmake/Checks.cmake native/tests/tooling/tooling_sweep_engine.cmake
git commit -m "perf(checks): run per-file sweep processes on a bounded pool

Every per-file kind ran its 511 tool invocations serially on a 16-core
host (~137 min for the sweep phase). Workers default to the online CPU
count, BIKE_CHECK_JOBS overrides it, and records merge in entry order so
summaries stay deterministic.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 8: Доля ядер при одновременных проверках

Решение: очереди нет. Каждая проверка смотрит, сколько других уже идёт, и берёт свою долю. После задачи 7 каждая проверка занимает все ядра, поэтому без доли две одновременные проверки мешали бы друг другу сильнее, чем раньше.

**Файлы:**
- Изменить: `tools/run_tests.sh`
- Изменить: `native/tests/tooling/tooling_gate.cmake`

- [x] **Шаг 1: Допиши тесты**

Перед строкой `# (run_tests.sh cases are appended …)` в `tooling_gate.cmake`:

```cmake
# --- another live gate halves the share; a stale entry is ignored ---------
execute_process(COMMAND bash -c "sleep 60 >/dev/null 2>&1 & echo $!"
  OUTPUT_VARIABLE _live_pid OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND bash -c "sleep 0 & p=$!; wait $p; echo $p"
  OUTPUT_VARIABLE _dead_pid OUTPUT_STRIP_TRAILING_WHITESPACE)
file(MAKE_DIRECTORY "${_state}/active")
file(WRITE "${_state}/active/${_live_pid}" "${_live_pid}\t0\t/elsewhere\tnative\n")
file(WRITE "${_state}/active/${_dead_pid}" "${_dead_pid}\t0\t/stale\tnative\n")
file(REMOVE "${_log}")
gate_run(gate_share EXPECT_EXIT 0 ENV ${_gate_env}
  EXPECT_OUTPUT "1 other native gate(s) running" "/elsewhere"
  COMMAND bash "${_repo}/tools/run_tests.sh" native)
execute_process(COMMAND kill "${_live_pid}")
file(STRINGS "${_log}" _calls)
string(FIND "${_calls}" "--parallel 4" _par)
string(FIND "${_calls}" "checks tidy jobs=4" _tidy_jobs)
if(_par EQUAL -1 OR _tidy_jobs EQUAL -1)
  set(_failures "${_failures}\n  gate_share: expected 4 of 8 cores, calls '${_calls}'")
endif()
if(EXISTS "${_state}/active/${_dead_pid}")
  set(_failures "${_failures}\n  gate_share: stale registry entry was not removed")
endif()
file(GLOB _left "${_state}/active/*")
list(LENGTH _left _left_count)
if(NOT _left_count EQUAL 1)
  set(_failures "${_failures}\n  gate_share: own registry entry left behind: '${_left}'")
endif()
# An explicit BIKE_CHECK_JOBS wins over the computed share.
file(REMOVE "${_log}")
gate_run(gate_jobs_override EXPECT_EXIT 0 ENV ${_gate_env} "BIKE_CHECK_JOBS=3"
  COMMAND bash "${_repo}/tools/run_tests.sh" native release --from analyzer)
file(STRINGS "${_log}" _calls)
string(FIND "${_calls}" "checks analyzer jobs=3" _override)
if(_override EQUAL -1)
  set(_failures "${_failures}\n  gate_jobs_override: calls '${_calls}'")
endif()
```

После `kill` в каталоге остаётся только запись живого (уже убитого) соседа. Своя запись удаляется при выходе. Поэтому ожидается одна запись — запись соседа. Чужие записи проверка удаляет только тогда, когда процесс точно мёртв, и только при следующем запуске.

- [x] **Шаг 2: Убедись, что тест красный**

```bash
ctest --test-dir build/native-release -R '^tooling_gate$' --output-on-failure 2>&1 | grep gate_share
```

- [x] **Шаг 3: Реализуй учёт в `run_tests.sh`**

Замени строку `jobs="$cpu_count"` на:

```bash
# Active-gate registry: one file per running native/full gate, named by its
# pid. No queue — each gate takes an equal share of the cores, so parallel
# gates slow each other down evenly instead of oversubscribing the machine.
# Entries of dead processes are removed; live ones are reported.
active_dir="$BIKE_GATE_STATE_DIR/active"
mkdir -p "$active_dir"
others=0
for entry in "$active_dir"/*; do
  [[ -e "$entry" ]] || continue
  pid="${entry##*/}"
  if [[ "$pid" == "$$" ]]; then continue; fi
  if kill -0 "$pid" 2>/dev/null; then
    others=$((others + 1))
    IFS=$'\t' read -r _ started where kind_of < "$entry" || true
    printf 'note: another %s gate is running: pid %s in %s (since %s)\n' \
      "${kind_of:-native}" "$pid" "${where:-?}" \
      "$(date -r "${started:-0}" '+%H:%M' 2>/dev/null || printf '?')" >&2
  else
    rm -f "$entry"
  fi
done
if [[ "$profile" != quick ]]; then
  printf '%s\t%s\t%s\t%s\n' "$$" "$(date +%s)" "$REPO_ROOT" "$profile" \
    > "$active_dir/$$"
fi
if ((others > 0)); then
  printf 'note: %s other native gate(s) running; using %s of %s cores\n' \
    "$others" "$((cpu_count / (others + 1) > 0 ? cpu_count / (others + 1) : 1))" \
    "$cpu_count" >&2
fi
jobs=$((cpu_count / (others + 1)))
((jobs >= 1)) || jobs=1
if [[ -n "${BIKE_CHECK_JOBS:-}" ]]; then
  jobs="$BIKE_CHECK_JOBS"
fi
export BIKE_CHECK_JOBS="$jobs"
```

В `finish()` первой строкой после `local rc=$?` добавь `rm -f "$active_dir/$$"`.

Доля считается один раз, при запуске проверки. Если сосед закончит раньше, оставшееся время проверка идёт на своей доле. Это осознанное упрощение. Запиши его в комментарий.

- [x] **Шаг 4: Прогони тесты**

```bash
ctest --test-dir build/native-release -R '^tooling_gate$' --output-on-failure 2>&1 | tail -4
build/native-release/bin/bike-native-checks --kind scripts --build build/native-release | tail -2
```

Ожидаемо: зелёные.

- [x] **Шаг 5: Коммит**

```bash
git add tools/run_tests.sh native/tests/tooling/tooling_gate.cmake
git commit -m "feat(gate): share cores between concurrent gates

Gates register in <git-common-dir>/bike-gate/active; each takes
cpu/(others+1) workers and reports who else is running. No queue:
nobody waits, nobody starves a neighbour. BIKE_CHECK_JOBS still wins.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 9: Не повторять одинаковые компиляции заголовков

Сейчас один заголовок компилируется отдельно для каждой цели, которая его подключает: 2380 компиляций на 260 заголовков. Совпадающих по аргументам и рабочему каталогу из них 751 (замер по `native_check_summaries/headers.json` 2026-11-03). Одинаковые аргументы и каталог дают одинаковый результат, поэтому компилируем один раз, а покрытыми отмечаем все цели.

**Файлы:**
- Изменить: `native/src/tools/check_frontends.cpp` (`run_header_sweep`)
- Изменить: `native/tests/tooling/tooling_sweep_engine.cmake`

**Что получают следующие задачи:** в итоге `headers.json` появляются поля `header_context_count` (все пары «заголовок — цель», как раньше `header_translation_unit_count`), `header_duplicate_contexts` (массив `{source, target, covered_by}`). Поле `header_translation_unit_count` теперь — число реальных компиляций.

- [x] **Шаг 1: Напиши тест**

Сценарий учебной сборки для заголовков — `b29_header_fixture` в `tooling_checks.cmake` (строка около 1543). Посмотри, как он строит сборку: заголовок под корнем, две цели. Сделай такую же сборку в `tooling_sweep_engine.cmake`: один заголовок `native/src/shared.hpp`, две цели `bike_a` и `bike_b` с **одинаковыми** условиями компиляции, по одному `.cpp` в каждой, оба подключают `shared.hpp`. Поддельный компилятор через `GXX` пишет строку в журнал вызовов при каждом вызове с `header_probe.cpp`. Проверки:

```cmake
expect_json(headers_dedup "${_hsummary}" 2 header_context_count)
expect_json(headers_dedup "${_hsummary}" 1 header_translation_unit_count)
expect_json_length(headers_dedup "${_hsummary}" 1 header_duplicate_contexts)
file(STRINGS "${_hfx}/probe-calls.log" _probe_calls)
list(LENGTH _probe_calls _probe_count)
if(NOT _probe_count EQUAL 1)
  set(_failures "${_failures}\n  headers_dedup: ${_probe_count} probe compiles, expected 1")
endif()
```

Добавь второй случай: у `bike_b` добавлено определение `-DVARIANT=1`. Тогда ожидаются 2 компиляции и пустой `header_duplicate_contexts`.

- [x] **Шаг 2: Убедись, что тест красный**

```bash
ctest --test-dir build/native-release -R '^tooling_sweep_engine$' --output-on-failure 2>&1 | grep headers_
```

- [x] **Шаг 3: Реализуй**

При сборке списка заданий (задача 7, шаг 5) для каждой пары считай ключ:

```cpp
// Identical argv (with a fixed probe path) in the same directory is the
// same compile: run it once, report every covered target.
std::string context_key(const CompileEntry &entry) {
    std::string key = entry.directory.string();
    for (const std::string &argument:
         replace_compile_source(entry, fs::path("/bike-header-probe.cpp"), true)) {
        key += '\x1f';
        key += argument;
    }
    return key;
}
```

Заведи `std::map<std::pair<std::string, std::string>, std::string> seen;` — ключ (заголовок, `context_key`), значение — первая цель. Если ключ уже есть, задание не создавай, а добавь `{source, target, covered_by}` в `header_duplicate_contexts`. `header_context_count` — прежнее число пар, `header_translation_unit_count` — число созданных заданий. Обнови строку вывода: `native headers selection: N headers, M header translation units (K contexts, D duplicates skipped)`.

- [x] **Шаг 4: Прогони тесты и настоящий вид**

```bash
cmake --build --preset release --target bike-native-checks
ctest --test-dir build/native-release -R '^tooling_sweep_engine$' --output-on-failure 2>&1 | tail -3
/usr/bin/time -p build/native-release/bin/bike-native-checks --kind headers --build build/native-release 2>&1 | grep -E 'selection|tool health|findings|^real'
```

Ожидаемо: тест зелёный; около 751 компиляции из около 2380; `ok`, `none`.

- [x] **Шаг 5: Коммит**

```bash
git add native/src/tools/check_frontends.cpp native/tests/tooling/tooling_sweep_engine.cmake
git commit -m "perf(checks): compile each distinct header context once

2380 header probes covered only 751 distinct (argv, directory) contexts.
Duplicates are reported as covered_by instead of recompiled.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 10: Кэш компилятора (`ccache`)

**Причина (проверена 2026-11-03):** `ccache` 4.14.2 установлен (`/opt/homebrew/bin/ccache`). Пробная настройка `cmake --preset release -B /tmp/bike-ccache-probe -DCMAKE_CXX_COMPILER_LAUNCHER=$(command -v ccache)` показала:
- CMake 4.4.3 **не пишет** обёртку компилятора в `compile_commands.json`: первая команда начинается с `/usr/bin/c++`;
- `native/cmake/BikeCheckContext.cmake:519` при этом записывает `compiler.launcher = ["/opt/homebrew/bin/ccache"]`;
- `native/src/tools/check_context.cpp:801-812` требует, чтобы каждая команда в базе начиналась с `compiler.launcher`, и падает: `compilation database launcher differs from configured C++ compiler in entry 0`.

Значит, поле `compiler.launcher` в описании сборки утверждает то, чего в базе компиляции никогда не бывает.

**Решение владельца (Q14, вариант а):** `compiler.launcher` описывает то, что действительно стоит перед компилятором в `compile_commands.json`, то есть всегда `[]`. Обёртка, через которую идёт сборка, записывается отдельным полем `compiler.build_launcher` — только для сведения. Проверка компилятора, системных файлов и флагов не меняется. Код проверки на C++ тоже не меняется: при пустом `launcher` она уже сравнивает первый аргумент с компилятором. Через `compiler_launcher()` работают ещё `tool_argv()` (`check_frontends.cpp:222`) и проверка второго компилятора (`check_context.cpp:1533`); для них пустой список — верное описание базы компиляции.

**Файлы:**
- Изменить: `native/cmake/BikeCheckContext.cmake` (строки 519 и 583)
- Изменить: `CMakeLists.txt` (сразу после `project(bike_sim …)`, строка 24)
- Изменить: `native/tests/tooling/tooling_sweep_engine.cmake`

- [x] **Шаг 1: Установка `ccache`** — сделано владельцем.

- [x] **Шаг 2: Напиши тест на описание сборки**

В `tooling_sweep_engine.cmake` перед строкой `# (later tasks append cases above this line)`:

```cmake
# The context's compiler.launcher names what compile_commands.json actually
# prefixes to the compiler. CMake never writes CMAKE_CXX_COMPILER_LAUNCHER
# there, so it is always empty; the build-time wrapper (ccache) is recorded
# separately as compiler.build_launcher, for provenance only.
expect_json_length(context_launcher "${BUILD_DIR}/native_check_context.json" 0
  compiler launcher)
string(JSON _build_launcher_type ERROR_VARIABLE _bl_err
  TYPE "${_context_json}" compiler build_launcher)
if(_bl_err OR NOT _build_launcher_type STREQUAL "ARRAY")
  set(_failures "${_failures}\n  context_launcher: compiler.build_launcher must be an array")
endif()
```

Затем настрой сборку с обёрткой и прогони тест:

```bash
cmake --preset release -DCMAKE_CXX_COMPILER_LAUNCHER="$(command -v ccache)" >/dev/null
ctest --test-dir build/native-release -R '^tooling_sweep_engine$' --output-on-failure 2>&1 | grep context_launcher
```

Ожидаемо: FAIL. В `launcher` записан `ccache`, а `build_launcher` нет.

- [x] **Шаг 3: Исправь запись в `BikeCheckContext.cmake`**

Замени строку 519:

```cmake
  # compiler.launcher describes the argv prefix compile_commands.json carries
  # before the compiler — CMake never writes CMAKE_CXX_COMPILER_LAUNCHER there
  # (verified with CMake 4.4.3 + Ninja), so it is empty. The wrapper the build
  # really runs through (e.g. ccache) is provenance only: build_launcher.
  _native_json_array(_native_compiler_launcher_json)
  _native_json_array(_native_compiler_build_launcher_json ${CMAKE_CXX_COMPILER_LAUNCHER})
```

Проверь, что `_native_json_array` без значений даёт `[]`: посмотри её определение через `git grep -n 'function(_native_json_array' native/cmake`. Если пустой список она обрабатывает иначе, напиши `set(_native_compiler_launcher_json "[]")`.

В строке 583 после `\"launcher\":${_native_compiler_launcher_json}` добавь `,\"build_launcher\":${_native_compiler_build_launcher_json}`.

- [x] **Шаг 4: Включи `ccache` по умолчанию, если он установлен**

В `CMakeLists.txt` сразу после `project(bike_sim …)`:

```cmake
# A compiler cache only speeds up rebuilds (shared across every worktree);
# it never changes what is compiled, so a missing ccache is not an error.
# It is recorded as compiler.build_launcher in native_check_context.json.
option(BIKE_COMPILER_CACHE "Use ccache as the C++ compiler launcher when installed" ON)
if(BIKE_COMPILER_CACHE AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
  find_program(BIKE_CCACHE_PROGRAM ccache)
  if(BIKE_CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${BIKE_CCACHE_PROGRAM}")
  endif()
endif()
```

Проба системных файлов Apple (`BikeAppleSDK.cmake:75-92`) тоже пойдёт через `ccache`. На её результат это не влияет.

- [x] **Шаг 5: Проверь настройку, сборку, базу и виды проверки**

Сначала убери ключ `-DCMAKE_CXX_COMPILER_LAUNCHER`, оставшийся с шага 2 в кэше, чтобы проверить именно включение по умолчанию:

```bash
cmake --preset release -U CMAKE_CXX_COMPILER_LAUNCHER >/dev/null
jq -c '.compiler | {launcher, build_launcher}' build/native-release/native_check_context.json
cmake --build --preset release
build/native-release/bin/bike-native-checks --kind cppcheck --build build/native-release | tail -2
build/native-release/bin/bike-native-checks --kind frontends --build build/native-release | tail -2
ctest --test-dir build/native-release -R '^(tooling_sweep_engine|tooling_toolchain|tooling_install|tooling_gate)$' --output-on-failure 2>&1 | tail -6
```

Ожидаемо:
- `{"launcher":[],"build_launcher":["/opt/homebrew/bin/ccache"]}`;
- оба вида `tool health: ok`, `findings: none`;
- тесты — как в задаче 0, без новых неудач.

Если `frontends` или другой вид отвергнет базу по иной причине, не ослабляй проверку: запиши сообщение и остановись.

- [x] **Шаг 6: Убедись, что кэш работает**

```bash
ccache -z
rm -rf /tmp/bike-ccache-second
cmake --preset release -B /tmp/bike-ccache-second >/dev/null
cmake --build /tmp/bike-ccache-second
ccache -s | grep -Ei 'hits|misses'
rm -rf /tmp/bike-ccache-second
```

Ожидаемо: почти всё — попадания (`Hits` близко к числу файлов). Запиши время второй сборки в отчёт.

- [x] **Шаг 7: Коммит**

```bash
git add native/cmake/BikeCheckContext.cmake CMakeLists.txt native/tests/tooling/tooling_sweep_engine.cmake
git commit -m "build: use ccache when installed; record it as build_launcher

compiler.launcher claimed a compile_commands.json prefix CMake never
writes (verified with CMake 4.4.3 + Ninja), so any launcher made every
sweep reject the database. launcher now describes the database ([]);
the wrapper is provenance in build_launcher. ccache is optional.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 11: Замер — находит ли отдельный анализатор что-то, чего не находит `clang-tidy`

`clang-tidy` уже включает `clang-analyzer-*`, `clang-analyzer-optin.core.*`, `…optin.cplusplus.*`, `…optin.taint.*` (`native/.clang-tidy`). Отдельный проход `analyzer` после задачи 5 включает основные проверки, `optin` целиком и `cplusplus.Move`. Задача — замер и отчёт, **без удаления** прохода.

**Файлы:**
- Создать: `docs/superpowers/audits/2026-11-03-analyzer-vs-tidy.md`

- [x] **Шаг 1: Полные прогоны обоих видов на одном коде**

```bash
build/native-release/bin/bike-native-checks --kind tidy --build build/native-release | tail -2
build/native-release/bin/bike-native-checks --kind analyzer --build build/native-release | tail -2
```

- [x] **Шаг 2: Сравни находки по месту и виду**

Учитывай **все** находки, включая подавленные (`diagnostics`, а не `unsuppressed_diagnostics`): сравнивается способность находить, а не нынешние подавления.

```bash
S=build/native-release/native_check_summaries
norm='[.records[].diagnostics[] | capture("^(?<f>[^:]+):(?<l>[0-9]+):[0-9]+: warning: .*\\[(?<c>[^\\]]+)\\]$")? | "\(.f):\(.l) \(.c | sub("^clang-analyzer-"; ""))"] | unique[]'
jq -r "$norm" $S/analyzer.json > /tmp/an.txt
jq -r "$norm" $S/tidy.json | grep -E ' (core|cplusplus|deadcode|optin|security|unix|nullability|osx|valist|webkit|fuchsia)\.' > /tmp/tidy-an.txt
comm -23 /tmp/an.txt /tmp/tidy-an.txt > /tmp/only-analyzer.txt
comm -13 /tmp/an.txt /tmp/tidy-an.txt > /tmp/only-tidy.txt
wc -l /tmp/an.txt /tmp/tidy-an.txt /tmp/only-analyzer.txt /tmp/only-tidy.txt
```

- [x] **Шаг 3: Сравни включённые наборы проверок**

```bash
LLVM="$(brew --prefix llvm)/bin"
"$LLVM/clang-tidy" -p build/native-release --list-checks native/src/platform/process.cpp | grep -o 'clang-analyzer-[^ ]*' | sed 's/^clang-analyzer-//' | sort > /tmp/tidy-checkers.txt
"$LLVM/clang" -cc1 -analyzer-checker-help 2>/dev/null | awk '/^  [a-z]/{print $1}' | grep -E '^(core|cplusplus|deadcode|optin|security|unix|nullability|osx|valist)\.' | sort > /tmp/all-checkers.txt
comm -13 /tmp/tidy-checkers.txt /tmp/all-checkers.txt | grep -E '^(optin\.|cplusplus\.Move)' > /tmp/analyzer-only-checkers.txt
cat /tmp/analyzer-only-checkers.txt
```

Это проверки, которые включены в отдельном проходе, но не в `clang-tidy`.

- [x] **Шаг 4: Напиши отчёт**

В отчёте:
- время обоих видов;
- число находок по каждой стороне;
- полный список `only-analyzer.txt` с разбором каждой находки (настоящая или ложная, есть ли подавление);
- список `analyzer-only-checkers.txt`.

В конце отчёта — таблица вариантов для владельца:

| Вариант | Что сделать | Цена |
|---|---|---|
| А | оставить отдельный проход | ~X мин на каждую полную проверку |
| Б | убрать проход, добавить `analyzer-only-checkers` в `native/.clang-tidy` | правка `check_contracts.cpp` (контроль `cplusplus.Move`), документации и тестов |
| В | … | … |

Решение не принимай.

- [x] **Шаг 5: Коммит**

```bash
git add docs/superpowers/audits/2026-11-03-analyzer-vs-tidy.md
git commit -m "docs(audit): compare standalone analyzer with clang-tidy findings

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 12: Проверка одного изменения — `tools/check_change.sh`

Уровень 1, до 5 минут:
1. Добавочная сборка.
2. **Все** обычные тесты (`ctest --preset release`: без `slow|audit|package`, `-j8`). Это осознанно шире решения «тесты по затронутым частям»: все 183 обычных теста идут около минуты (замер 2026-11-03: `Total Test time (real) = 55.56 sec`), так что выбирать нечего, а пропустить нужный тест невозможно.
3. `cppcheck`, `frontends`, `tidy` только для изменённых файлов `.cpp` и файлов, подключающих изменённые заголовки.

**Файлы:**
- Изменить: `native/src/tools/checks_main.cpp` (ключ `--only-sources FILE`)
- Изменить: `native/src/tools/checks_internal.hpp` (`run_sweep` получает выбор)
- Изменить: `native/src/tools/check_frontends.cpp` (отбор в `run_sweep`; `entries_reaching`)
- Создать: `tools/check_change.sh`
- Изменить: `native/tests/tooling/tooling_sweep_engine.cmake`, `native/tests/tooling/tooling_gate.cmake`

**Что получают следующие задачи:**
- `bike-native-checks --kind {cppcheck|frontends|tidy} --build B --only-sources LIST`. `LIST` — файл, по одному абсолютному или относительному к рабочему каталогу пути в строке; это `.cpp` или первичные заголовки.
- Итог пишется в `native_check_summaries/partial-<вид>.json`, в нём `"selection": "partial"`. Полный итог `<вид>.json` не затирается.

- [x] **Шаг 1: Тесты на `--only-sources`**

В `tooling_sweep_engine.cmake` используй сборку из задачи 7. Добавь в неё заголовок `native/src/common.hpp` (`#pragma once` и `inline int common() { return 0; }`) и перепиши `tu2.cpp` и `tu7.cpp` так, чтобы первой строкой в них было `#include "common.hpp"`. Остальные файлы его не подключают.

```cmake
set(_partial "${_fx}/build/native_check_summaries/partial-tidy.json")
file(WRITE "${_fx}/only.txt" "${_fx}/native/src/tu3.cpp\n${_fx}/native/src/common.hpp\n")
run_case(only_sources EXPECT_EXIT 0
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env} "BIKE_CHECK_JOBS=4"
        "CLANG_TIDY=${_fx}/slow-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build" --only-sources "${_fx}/only.txt")
expect_json(only_sources "${_partial}" "partial" selection)
expect_json_length(only_sources "${_partial}" 3 records)
expect_json(only_sources "${_partial}" "${_fx}/native/src/tu2.cpp" records 0 source)
expect_json(only_sources "${_partial}" "${_fx}/native/src/tu3.cpp" records 1 source)
expect_json(only_sources "${_partial}" "${_fx}/native/src/tu7.cpp" records 2 source)

# Nothing selected is an error, never a green zero-unit sweep.
file(WRITE "${_fx}/none.txt" "${_fx}/README.md\n")
run_case(only_sources_empty EXPECT_EXIT FAIL
  EXPECT_STDERR "selects no translation unit"
  COMMAND "${CMAKE_COMMAND}" -E env ${_b29_clean_env}
        "CLANG_TIDY=${_fx}/slow-tool"
        "${CHECKS_BINARY}" --kind tidy --build "${_fx}/build" --only-sources "${_fx}/none.txt")

# Only the per-file kinds of level 1 accept a selection.
run_case(only_sources_kind EXPECT_EXIT 2
  EXPECT_STDERR "--only-sources requires --kind cppcheck, frontends or tidy"
  COMMAND "${CHECKS_BINARY}" --kind odr --build "${_fx}/build" --only-sources "${_fx}/only.txt")
```

Красный прогон: `ctest -R '^tooling_sweep_engine$'`. Ожидаемо: `unrecognized arguments: --only-sources`.

- [x] **Шаг 2: Ключ в `checks_main.cpp`**

По образцу `--repo-root`: разбор `--only-sources` и `--only-sources=`. После проверки вида:

```cpp
    if (!only_sources.empty() && kind != "cppcheck" && kind != "frontends" &&
        kind != "tidy")
        return argparse_error(prog, "--only-sources requires --kind cppcheck, frontends or tidy");
```

Добавь ключ в `usage_line()` и в справку. Последний вызов замени на:

```cpp
        return bike::checks::run_sweep(
            kind, build_path,
            only_sources.empty() ? std::nullopt
                                 : std::optional<bike::checks::fs::path>(only_sources));
```

В `checks_internal.hpp`:

```cpp
// only_sources: a newline-separated file of .cpp sources and/or first-party
// headers; the sweep runs on the listed TUs plus every TU whose include
// walk reaches a listed header, and writes partial-<kind>.json with
// "selection":"partial". An empty selection is an error.
[[nodiscard]] int run_sweep(const std::string &kind, const fs::path &build,
                            const std::optional<fs::path> &only_sources = std::nullopt);
```

- [x] **Шаг 3: Отбор в `run_sweep()`**

Над `run_sweep` объяви:

```cpp
namespace {
std::set<std::size_t> entries_reaching(const std::vector<CompileEntry> &entries,
                                       const std::vector<fs::path> &header_roots,
                                       const std::set<std::string> &headers);
} // namespace
```

Определи её внутри анонимного пространства имён, сразу после `reaching_targets()`:

```cpp
// Indexes of entries whose include walk reaches any of `headers` — the
// walk reaching_targets() does, kept per entry instead of per target.
std::set<std::size_t> entries_reaching(const std::vector<CompileEntry> &entries,
                                       const std::vector<fs::path> &header_roots,
                                       const std::set<std::string> &headers) {
    std::map<std::string, std::vector<IncludeDirective>> cache;
    std::set<std::size_t> selected;
    for (std::size_t index = 0; index < entries.size(); ++index) {
        const CompileEntry &entry = entries[index];
        const std::vector<fs::path> dirs = entry_include_dirs(entry);
        std::set<std::string> visited;
        std::vector<fs::path> frontier{entry.source};
        bool reached = false;
        while (!frontier.empty() && !reached) {
            const fs::path file = frontier.back();
            frontier.pop_back();
            if (!visited.insert(file.string()).second) continue;
            for (const IncludeDirective &directive:
                 scan_include_directives(file, cache)) {
                const std::optional<fs::path> resolved =
                    resolve_include(directive, file.parent_path(), dirs);
                if (!resolved.has_value() || !is_under(*resolved, header_roots))
                    continue;
                if (headers.contains(resolved->string())) {
                    reached = true;
                    break;
                }
                frontier.push_back(*resolved);
            }
        }
        if (reached) selected.insert(index);
    }
    return selected;
}
```

В `run_sweep()`, сразу после `entries = load_entries(…)` внутри `setup_ok`:

```cpp
            if (only_sources.has_value()) {
                std::set<std::string> listed;
                std::ifstream stream(*only_sources);
                if (!stream)
                    throw CheckError("cannot read --only-sources file: " +
                                     only_sources->string());
                // Deleted files (a change can remove a source) select nothing.
                for (std::string line; std::getline(stream, line);)
                    if (!line.empty() && fs::exists(fs::path(line), ec))
                        listed.insert(canonical_path(fs::path(line)).string());
                std::set<std::size_t> keep = entries_reaching(
                    entries, standalone_header_roots(manifest.first_party_roots),
                    listed);
                for (std::size_t index = 0; index < entries.size(); ++index)
                    if (listed.contains(entries[index].source.string()))
                        keep.insert(index);
                if (keep.empty())
                    throw CheckError("--only-sources selects no translation unit");
                std::vector<CompileEntry> selected;
                for (const std::size_t index: keep)
                    selected.push_back(entries[index]);
                entries = std::move(selected);
                set_member(summary, "selection", vstr("partial"));
            }
```

Порядок `keep` — по возрастанию номера, то есть исходный порядок. Все вызовы `write_summary(build_dir, kind, summary)` внутри `run_sweep` замени на `write_summary(build_dir, summary_kind, summary)`, где в начале функции:

```cpp
    const std::string summary_kind =
        only_sources.has_value() ? "partial-" + kind : kind;
```

Строку вывода с путём итога поправь так же. `#include <fstream>` добавь, если его нет.

- [x] **Шаг 4: Тесты `bike-native-checks` зелёные**

```bash
cmake --build --preset release --target bike-native-checks
ctest --test-dir build/native-release -R '^tooling_sweep_engine$' --output-on-failure 2>&1 | tail -3
```

- [x] **Шаг 5: Тест на `check_change.sh`**

В `tooling_gate.cmake`, рядом с остальными случаями `run_tests.sh`: скопируй `tools/check_change.sh` в `${_repo}/tools`. Сделай `${_repo}` репозиторием git: `git init -b main`, `git -c user.name=t -c user.email=t@t commit --allow-empty -m base`. Затем вторая фиксация добавляет `native/src/x.cpp` и `docs/a.md`. Все команды git выполняй через `execute_process(… WORKING_DIRECTORY "${_repo}")`. Заглушка `bike-native-checks` должна дописывать в журнал ещё и содержимое файла после `--only-sources`. Для этого в заглушке рядом с `--kind` разбери `--only-sources` и выведи `cat` списка.

Ожидаемо в журнале по порядку:
1. `cmake --build --preset release`
2. `ctest --preset release`
3. `checks cppcheck`, затем строка `…/native/src/x.cpp`
4. то же для `frontends` и `tidy`

`docs/a.md` в списке нет. Отдельный случай: изменён только `docs/a.md` — тогда `checks` не вызывается вовсе, а в выводе `no C++ source or header changed`.

- [x] **Шаг 6: Напиши `tools/check_change.sh`**

```bash
#!/usr/bin/env bash
# tools/check_change.sh [base] — level 1: verify one change in minutes.
#   1. incremental build of the release preset;
#   2. every ordinary test (ctest --preset release: no slow|audit|package;
#      ~1 min with -j8 — cheaper to run all than to guess which);
#   3. cppcheck, frontends and clang-tidy on the changed translation units
#      plus every unit that includes a changed first-party header.
# Changed = committed since merge-base(HEAD, base) + uncommitted + untracked.
# Default base: main. Not a substitute for `tools/run_tests.sh native`
# before a merge (level 2).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
base="${1:-main}"
build_dir="$REPO_ROOT/build/native-release"

merge_base="$(git merge-base HEAD "$base")"
list="$(mktemp "${TMPDIR:-/tmp}/bike-change.XXXXXX")"
trap 'rm -f "$list"' EXIT
{
  git diff --name-only "$merge_base"
  git ls-files --others --exclude-standard
} | sort -u | while IFS= read -r path; do
  case "$path" in
    native/*.cpp|native/*.hpp|native/*.h|native/*.cc)
      [[ -f "$path" ]] && printf '%s/%s\n' "$REPO_ROOT" "$path"
      ;;
  esac
done > "$list"

selected="$(wc -l < "$list" | tr -d ' ')"
printf 'check_change: %s changed C++ file(s) since %s\n' "$selected" "$base"
if ((selected > 100)); then
  # On a long-lived branch merge-base(HEAD, main) can be far back; level 1
  # is meant for the change at hand.
  printf 'check_change: note: many files selected; pass a closer base (e.g. the commit before your task)\n'
fi

started=$SECONDS
cmake --build --preset release
ctest --preset release
if [[ ! -s "$list" ]]; then
  printf 'check_change: no C++ source or header changed since %s; analysis skipped\n' "$base"
else
  checks="$build_dir/bin/bike-native-checks"
  [[ -n "${BIKE_NATIVE_CHECKS:-}" ]] && checks="$BIKE_NATIVE_CHECKS"
  for kind in cppcheck frontends tidy; do
    "$checks" --kind "$kind" --build "$build_dir" --only-sources "$list"
  done
fi
printf 'check_change: passed in %ss (level 1; run tools/run_tests.sh native once before merging)\n' \
  "$((SECONDS - started))"
```

- [x] **Шаг 7: Тесты и настоящий прогон**

```bash
ctest --test-dir build/native-release -R '^(tooling_gate|tooling_sweep_engine)$' --output-on-failure 2>&1 | tail -4
build/native-release/bin/bike-native-checks --kind scripts --build build/native-release | tail -2
time bash tools/check_change.sh main3~3
```

Ожидаемо: тесты зелёные; `scripts` зелёный; настоящий прогон укладывается в 5 минут. Запиши время в отчёт. Если не укладывается, запиши, какая часть сколько заняла.

- [x] **Шаг 8: Коммит**

```bash
git add native/src/tools/checks_main.cpp native/src/tools/checks_internal.hpp native/src/tools/check_frontends.cpp tools/check_change.sh native/tests/tooling/tooling_sweep_engine.cmake native/tests/tooling/tooling_gate.cmake
git commit -m "feat(checks): level-1 change check with --only-sources

tools/check_change.sh builds, runs every ordinary test (~1 min) and runs
cppcheck/frontends/tidy only on changed TUs and the TUs that include a
changed header. Partial summaries never overwrite full ones.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 13: Документация и итоговая полная проверка

**Файлы:**
- Изменить: `AGENTS.md` (раздел `## Verification` и `### Verification speed`)
- Изменить: `docs/TESTING.md` (строки около 155–163 и 250–258 про экспериментальную половину; описание `run_tests.sh`)
- Изменить: `docs/agents/native-engineering.md` (строка «experimental findings remain report-only…»)

- [x] **Шаг 1: `AGENTS.md`**

Замени `### Verification speed` целиком. Числа возьми из своих замеров (задачи 7–9 и шаг 3 ниже), не из этого плана:

```markdown
### Verification levels

Three levels; pick the cheapest that covers the moment.

1. **Change** — `bash tools/check_change.sh [base]` after every edit
   (≤5 min): incremental build, every ordinary test, cppcheck/frontends/
   clang-tidy on the changed TUs and the TUs including a changed header.
2. **Full** — `bash tools/run_tests.sh native` **once per merge into the
   main branch**, not per task or per parallel lane (measured <N> min on an
   idle machine; see `<git-common-dir>/bike-gate/timings.tsv`). After a
   fix, `--from <stage>` re-checks from the failed stage; that run is
   PARTIAL and never the final verification. The completion criterion is
   one complete run that exits 0.
3. **Extended** — `bash tools/run_tests.sh full`: daily or before a large
   merge (adds the slow label set, ~46 min serial).

Linux runs only on explicit request (`tools/linux/container.sh`).
Concurrent gates do not queue; each takes cpu/(running gates) workers and
prints who else is running. `BIKE_CHECK_JOBS` overrides the share.
There is no baseline of tolerated failures: a red test is fixed, or
disabled with a reason and a tracking task.
```

В разделе `## Verification` убери предложение `Alpha analyzer findings print but never block.`

- [x] **Шаг 2: `docs/TESTING.md` и `native-engineering.md`**

Найди упоминания:

```bash
git grep -n -i 'alpha\|15–30\|report-only' -- AGENTS.md docs/TESTING.md docs/agents/native-engineering.md
```

- В каждом месте про экспериментальную половину напиши, что анализатор идёт одной основной половиной (экспериментальная убрана 2026-11-03, причина — тысячи непрочитанных находок, ~34 мин).
- В `native-engineering.md` фразу «experimental findings remain report-only while execution health is visible» замени на «sweeps report tool execution health separately from findings».
- В `TESTING.md` в описании `run_tests.sh` опиши `--from`, журнал времени, новый порядок, `BIKE_CHECK_JOBS`, `tools/check_change.sh`.

- [x] **Шаг 3: Итоговая полная проверка — один раз**

Убедись, что других полных проверок в этой рабочей копии нет (`ls "$(git rev-parse --git-common-dir)/bike-gate/active"`).

```bash
bash tools/run_tests.sh native 2>&1 | tee /tmp/verification-speed-final.log | tail -25
```

Ожидаемо: код выхода 0, таблица стадий, общее время. Цель — не больше 30 минут на свободной машине. Если больше — запиши, какие стадии заняли больше всего, в отчёт; это вход для следующего шага работ, а не повод урезать проверку.

Если упала стадия, исправь причину. Повтори с `--from <стадия>`, затем ещё раз полностью, без `--from`. Итоговым считается только полный прогон.

- [x] **Шаг 4: Отчёт**

Допиши в `docs/superpowers/audits/2026-11-03-verification-speed-baseline.md` раздел «После»:
- таблица «стадия — было — стало»;
- время уровня 1 из задачи 12;
- строка журнала `timings.tsv` итогового прогона;
- что отложено и почему (задача 10, вариант Б; решение по задаче 11; распараллеливание `slow`).

- [x] **Шаг 5: Коммит**

```bash
git add AGENTS.md docs/TESTING.md docs/agents/native-engineering.md docs/superpowers/audits/2026-11-03-verification-speed-baseline.md
git commit -m "docs(verification): three levels, measured gate cost, --from

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## Проверка плана по описанию решений

| Решение | Задача |
|---|---|
| 1. до 5 / до 30 мин | 7, 9, 12, итог — 13 |
| 2. три уровня; Linux отдельно | 12 (уровень 1), 13 (описание уровней), ограничения |
| 3. два красных теста | 1, 2 |
| 4. без очереди, доля ядер, предупреждение | 8 |
| 5. от дешёвого к дорогому, тесты первыми | 3, 4 |
| 6. продолжение с упавшей стадии | 3, 4 |
| 7. без экспериментальной половины | 5 |
| 8. замер анализатора против `clang-tidy` | 11 |
| 9. время стадий и журнал | 3, 4 |
| 10. порядок работ | порядок задач |
| повторы заголовков, кэш | 9, 10 |
