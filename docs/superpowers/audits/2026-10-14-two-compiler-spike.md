# Спайк WP-V5: двухкомпиляторный golden (D14)

Измерительный спайк из `docs/superpowers/plans/2026-10-14-round-2-integration-master.md` §WP-V5.
Цель — решить спор D14: нужен ли vendored pinned-math (корректно-округлённые реализации
`pow`/`exp`/`tanh` из CORE-MATH/fdlibm, не зависящие от системной libm) **срочно для существующих
формул** или **только для новых формул с `pow`** (бисекция `z = (f/(w·k_eq))^{1/n}` в WP-C2).
Исходное опасение D14: `std::pow`/`std::exp`/`std::tanh` могут различаться между libm/toolchain'ами
на ~0.5–1.73 ulp и сломать побитовый контракт C++↔Python.

**Ничего не чинилось.** В репозитории менялся только этот файл; вся диагностика —
untracked-артефакты внутри `native/build-gcc/` и `/tmp/`.

## Термины (при первом употреблении)

- **ulp** (unit in the last place) — вес младшего бита мантиссы binary64-значения; расхождение
  в 1 ulp — минимально возможная разница между двумя соседними представимыми числами.
- **libm** — системная математическая библиотека (`pow`, `exp`, `tanh`, `hypot`, …). На macOS это
  часть libSystem.dylib; на Linux — часть glibc (`libm.so`). Реализации между ОС и даже версиями
  могут отличаться на доли ulp.
- **Golden-тест** — тест, сравнивающий выход не с приближённым диапазоном, а с эталоном
  бит-в-бит: либо с зафиксированным артефактом (golden episode `.npz`), либо с live-вычислением
  Python-референса (`tests/reference/test_native_*.py` пинят pure-Python ядра, см.
  `_pin_oracle_kernels`, — чтобы Numba-кодеген, который может дрейфовать ~1 ulp, не мешал
  сравнению).
- **Побитовый контракт** — нативное C++-ядро обязано давать те же биты binary64, что и
  Python-референс-арифметика, на любом проверенном входе.

## 1. Сетап

| Что | Значение |
|---|---|
| Коммит | `6e2dbf3` (ветка `feat/r2-v5`, ворк-дерево `r2-v5`) |
| Платформа | macOS 27.x arm64, Xcode 26.5 (SDK `MacOSX.sdk` из `Xcode.app`) |
| clang | Apple clang 21.0.0 (`clang-2100.1.1.101`), `/usr/bin/c++`, стандартная библиотека libc++ |
| gcc | Homebrew `gcc-16`/`g++-16` 16.2.0, `/opt/homebrew/bin/g++-16`, стандартная библиотека **libstdc++** |
| Build type | пустой `CMAKE_BUILD_TYPE` в обеих сборках → **-O0 у обоих** (дефолт проекта) |
| Общие числовые флаги | `-ffp-contract=off` (глобально в `native/CMakeLists.txt`), отсутствие `-ffast-math`-семейства |
| Python | 3.14.3; NumPy 2.5.2; MuJoCo 3.12.0; nanobind 3.1.0; CMake 4.4.3; uv 0.12.9 |

Важно для интерпретации: обе сборки — на одной ОС и линкуются с **одной и той же libm**
(libSystem.dylib). Спайк измеряет различия **codegen'а компиляторов при общей libm** —
не различия между libm разных ОС (см. «Открытые вопросы»).

## 2. Сборки

### clang (существующий путь, без отклонений)

```bash
cmake -S native -B native/build
uv run cmake --build native/build -j4
# → native/build/bike_native.cpython-314-darwin.so
```

`CMakeCache`: `CMAKE_CXX_COMPILER=/usr/bin/c++`, `CMAKE_CXX_FLAGS=` (пусто),
`CMAKE_OSX_SYSROOT=<Xcode MacOSX.sdk>`.

### gcc-16 (отдельный каталог, только env/кэш-переменные + враппер в build-дереве)

```bash
CC=/opt/homebrew/bin/gcc-16 CXX=/opt/homebrew/bin/g++-16 \
cmake -S native -B native/build-gcc \
  -DCMAKE_OSX_SYSROOT=/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk \
  -DCMAKE_CXX_FLAGS="-isysroot /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk"
# после первого failed-probe: CMAKE_CXX_COMPILER заменён на враппер
# native/build-gcc/bin/g++-16-filtered (untracked, живёт только в build-дереве)
uv run cmake --build native/build-gcc -j4
# → native/build-gcc/bike_native.cpython-314-darwin.so
```

Два препятствия и их разрешение (CMakeLists не тронут, правки только в кэш-переменных и
untracked-враппере):

1. **Линковка падает с `ld: tapi error: malformed file`** —
   `/Library/Developer/CommandLineTools/SDKs/MacOSX27.0.sdk/usr/lib/libSystem.B.tbd:4:20: error:
   unknown architecture arm64e.x1-macos`. Причина: brew GCC собран с built-in sysroot
   `--with-sysroot=/Library/Developer/CommandLineTools/SDKs/MacOSX27.sdk`, а активный
   Xcode-линкер (ld/TAPI 26.5) не парсит формат `.tbd` из более нового CLT 27.
   Workaround: форсировать Xcode SDK через `CMAKE_OSX_SYSROOT` + `-isysroot` в
   `CMAKE_CXX_FLAGS` (прямой `g++-16 -isysroot <Xcode SDK>` линкует успешно; штатный
   platform-probe CMake ошибочно считает, что GCC не поддерживает `-isysroot`, потому что
   `g++-16 -v --help` падает, передав `--help` линкеру).
2. **g++-16 отвергает ~70 из 122 уникальных warning-флагов** проекта
   (Apple-clang-only: `-Wunneeded-member-function`, `-Wnonportable-system-include-path`,
   `-Wlarge-by-value-copy`, `-Wclass-varargs`, …). Плюс `-Wmissing-variable-declarations`
   — C-only диагностика, в C++-режиме идёт ошибкой. Workaround: python-враппер
   `native/build-gcc/bin/g++-16-filtered` вырезает 71 reject-флаг из argv и добавляет
   `-Wno-error=frame-larger-than=` / `-Wno-error=stack-usage=` (две диагностики GCC,
   чьи бюджеты откалиброваны под clang — понижены с error до warning, всё ещё печатаются).
   Числозначащие флаги не фильтровались: `-ffp-contract=off` доходит до компилятора как есть.

Оговорка по эквивалентности сборок: warning-наборы после фильтрации не идентичны — для целей
спайка это приемлемо (измеряется числовой выход codegen'а, а не parity диагностик), но
честно зафиксировано здесь.

## 3. Что прогонялось

- **CTest** — все 6 native-тестов в обеих сборках (`ctest --test-dir <build> --output-on-failure`):
  `native_contract_tests`, `native_distance_tests`, `native_station_tests`,
  `native_bench_bare`, `native_bench_glue`, `native_bench_heavy`.
- **pytest** — вся коллекция `tests/reference/test_native_*.py`: коллекция проверена на обеих
  сборках `pytest --collect-only`, узлов ровно **2950** в каждой (diff пустой). Прогон разбит
  на пачки; каждый падавший узел зафиксирован ниже.
- **SHA-256 зонды** (`/tmp/spike_hash.py`, не коммитится): побайтовый sha256 выходов
  debug-энтрипойнтов расширения и реальной телеметрии — см. таблицу §5.
- `test_golden_episode*.py` — Python-only (не грузят `bike_native`); побитовое сравнение
  эпизода покрывают зонды `golden_stepper40` и `rollout41_telemetry`.

## 4. Результаты тестов

### CTest

| Test | clang | gcc-16 |
|---|---|---|
| native_contract_tests | PASS | **FAIL (3 кейса)** |
| native_distance_tests | PASS | PASS |
| native_station_tests | PASS | PASS |
| native_bench_bare | PASS | PASS |
| native_bench_glue | PASS | PASS |
| native_bench_heavy | PASS | PASS |

Итог: clang **6/6**, gcc **5/6**. Падающие кейсы в `native_contract_tests`:

- `warm_core_zero_allocation` — «the warm core tick allocated or released storage»;
- `rtsan_invalid_status_control` — `drive/prime: allocation in the marked status path`;
- `warm_allocation_matrix` — `drive/elastic_chain/core_advance allocated in the warm core` (1 alloc, 1 freed).

Это контракты **нулевой аллокации** в тёплом пути (счётчик перехватывает `new`/`delete`).
Под libstdc++ тёплый путь делает единичную аллокацию — характерное отличие реализаций
`std::vector`/`std::string` (SSO-пороги, рост, exception-runtime), **нечисловое**, но означает,
что GCC-сборка не является drop-in заменой даже при идеальной числовой parity.

### pytest (`test_native_*`, 2950 узлов на сборку)

| Пачка | clang | gcc-16 |
|---|---|---|
| Подмножество pneumatic/tire/golden (1097 узлов) | 1097 passed | 1096 passed, **1 failed** |
| Остальной набор (1853 узла, включая loader+check_tools) | 1853 passed | 1850 passed, **3 failed** |
| **Итого** | **2950 / 0 fail** | **2946 / 4 fail** |

Падающие узлы на GCC:

1. `test_native_pneumatic_carcass.py::test_carcass_advance_unloaded_validation` —
   **числовое расхождение на NaN-входе** (единственное, см. §6).
2–4. `test_native_loader.py` — `test_child_confirms_selected_asan_runtime_is_loaded[False]`,
   `[True]` и `test_child_rejects_wrong_preload_when_selected_runtime_is_missing`:
   хелпер `_selected_asan_runtime()` просит компилятор выбранной сборки
   `-print-file-name=libclang_rt.asan_osx_dynamic.dylib`. У GCC нет clang-рантайма
   compiler-rt → `is_file()` ложь. **Toolchain-предположение харнесса, не числовое**
   (на clang 27/27 loader-тестов зелёные).

Остальные 2946 узлов на GCC прошли побитово с Python-оракулом — включая весь
pneumatic/tire/drivetrain/resistance/rider/research/runtime-onset, golden replay и
runtime rollout. Предупреждения (145) идентичны составу у clang (baseline
`energy.constraint_work` violation на этой платформе + DLASCL-шум BLAS — обе сборки одинаково).

## 5. Побитовое сравнение (sha256)

| Зонд | clang sha256 | gcc sha256 | совпало? |
|---|---|---|---|
| `relax_grid` — `np_tanh`+`exp`-heavy сетка `_debug_pneumatic_relax` | `3b84595f…6046c3d2` | `3b84595f…6046c3d2` | **да** |
| `carcass_unloaded_finite` — `advance_unloaded`, конечный вход | `ef52f0be…3a85c8bc` | `ef52f0be…3a85c8bc` | **да** |
| `carcass_unloaded_nan` — `advance_unloaded`, `dt=NaN` | `f7e7d273…e846f21` | `d3480726…3d81d3f` | **НЕТ** |
| `golden_stepper40` — 40 шагов golden-эпизода через `Stepper` (qfrc/susp/resistance/brake/rider) | `142de281…99c485` | `142de281…99c485` | **да** |
| `rollout41_telemetry` — 41 шаг runtime-роллаута, вся телеметрия | `59f36aa7…3dedd2` | `59f36aa7…3dedd2` | **да** |

Все конечные выходы — бит-в-бит одинаковые на обоих компиляторах. Единственное
расхождение — на входе с `NaN` (невалидный `dt_s`).

## 6. Локализация единственного расхождения

Тест `test_carcass_advance_unloaded_validation` вызывает `advance_unloaded(dt=NaN, …)`
и побитово сверяет состояние — это осознанная часть контракта: в тесте прямой комментарий
«dt=nan passes the oracle's positivity check (nan <= 0 is False) and propagates nan through
the advect — **keep that quirk**» (`test_native_pneumatic_carcass.py:715-716`). NaN-пayload
здесь не побочный артефакт, а зафиксированное требование. В
`native/src/pneumatic/carcass.cpp`:

```cpp
193:  const double decay = std::exp(-dt_s / relaxation_s);   // exp(-NaN/C) → NaN 0xfff8…
195:      maxwell_force_n_[i] = decay * advect_scratch_[i];   // NaN * NaN
```

Оба компилятора получают от `std::exp` **одинаковый** NaN `0xfff8…` (sign bit + qNaN payload),
то есть libm здесь не расходится. Дальше дизассемблер:

- clang (`-O0`): `fmul d0, d0, d1` — первый операнд `d0` = `decay`;
- gcc (`-O0`): `fmul d15, d30, d31` — первый операнд `d30` = `advect_scratch_[i]`,
  `d31` = `decay`.

AArch64 `fmul` при NaN-операндах возвращает NaN **первого исходного операнда** (сохраняя его
sign/payload). Умножение коммутативно для конечных значений, и GCC легально поменял
операнды местами → в GCC-сборке наружу выходит quiet-NaN `0x7ff8…` из `advect_scratch_`,
в clang-сборке — signed-NaN `0xfff8…` из `decay`. Это и есть весь различие sha256.

**Классификация:** расхождение **не в libm и не в точности формул** — это codegen-уровневая
разница порядка операндов коммутативной инструкции, видимая только потому, что контракт
считает NaN побитово (sign/payload), а не по IEEE-семантике «NaN == NaN».

Ожидание из WP-V5 («расходится `exp`/`tanh` в Стрибеке и гистерезисе») — **не подтвердилось в
числовой части**: `exp`/`tanh`/`hypot`/`pow`-пути дают идентичные биты на конечных входах
(в т.ч. Stribeck-ветка пневматической шины — `np_tanh` это вендоренный
`native/src/numeric_numpy.hpp`, не libm). Расхождение нашлось рядом — в той же
Maxwell-гистерезисной ветке, но по другому механизму (operand order), который pinned-math
**не лечит**: вендоренный `exp` вернул бы тот же NaN, а место перестановки — `fmul`
на выходе, вне libm.

## 7. Прочие межкомпиляторные различия (зафиксированы, нечисловые)

- brew GCC собран под CLT27-sysroot — ломает линковку с Xcode-26.5 `ld` (см. §2); штатно
  `cmake -B build-gcc` **не конфигурируется** без isysroot-workaround.
- `native_check_context.json`/loader-харнесс предполагает clang-family компилятор
  (compiler-rt пути, `-print-file-name` для `libclang_rt.asan_osx_dynamic.dylib`).
- Тёплые пути аллоцируют под libstdc++ (§4) — провал трёх CTest-контрактов.
- GCC печатает psabi-заметки про C++17 ABI для `std::pair` в параметрах — информационные.

## 8. Вердикт для WP-C1

**«pinned math нужен только для новых `pow`-формул».**

Обоснование:

- На запиненной платформе (macOS arm64, libSystem) clang и gcc дают **бит-в-бит одинаковые
  конечные выходы** по всему покрытию: ~2950 побитовых тестов против Python-оракула +
  sha256 телеметрии/силового replay совпали. Гипотеза D14 «libm-функции уже расходятся»
  на этой паре toolchain'ов **не подтвердилась** — обе сборки вызывают один и тот же libm.
- Вендорить pinned-реализации в **существующие** формулы было бы не только бесполезно, но и
  вредно: Python-оракул считает хостовой libm, и подмена `std::exp` на CORE-MATH в C++
  **разошла бы** биты там, где реализации отличаются — это прямо предупреждает WP-C1
  («не подключать к существующим формулам без решения V5»).
- Для **новых** `pow`-формул (бисекция `z=(f/(w·k_eq))^{1/n}` и т.п.) аргумент остаётся:
  pinned-реализация + таблица эталонных значений страхует от будущей cross-libm
  переносимости (другая ОС/другая libm на CI), где расхождение 0.5–1.73 ulp реально.
- Найденное расхождение — вне зоны ответственности pinned-math (см. §6): если побитовый
  контракт покрывает NaN-payload, нужна **отдельная мера** — либо канонизация NaN на
  выходе, либо ordering-дискиплина а-ля `pyfloat::pow`/`numeric::sincos` (volatile-барьеры),
  либо явное исключение NaN-входов из побитового scope. Вендорный `exp` это не чинит.

Оговорки к вердикту:

1. Спайк измерил **одну пару компиляторов на одной ОС с общей libm**. Он НЕ покрывает
   главный риск D14 — разные libm на разных ОС (macOS libSystem vs Linux glibc). Если
   побитовая parity когда-либо потребуется cross-OS, вопрос открывается заново — но тогда
   pinned-math нужен будет и на Python-стороне, что меняет масштаб решения.
2. Сборки прогонялись при `-O0` (дефолт проекта). Release-оптимизация может выявить другие
   codegen-уровневые FP-различия (уже задокументированный пример: `__sincos_stret`-слияние
   на clang — `numeric_sincos.hpp` защищается volatile-аргументами).
3. GCC-сборка не является зелёным native-профилем по гейту WP-V5 («native-профиль обеих
   сборок зелёный **или честно описанная причина**» — причины выше: 1 числовой NaN-кейс,
   3 toolchain loader-кейса, 3 CTest allocation-контракта).

## 9. Воспроизведение

```bash
# clang
cmake -S native -B native/build && uv run cmake --build native/build -j4
ctest --test-dir native/build --output-on-failure
PYTHONPATH=tests/reference NATIVE_TEST_BUILD_PATH=$PWD/native/build \
  uv run --frozen --group native python -m pytest tests/reference/test_native_*.py -q

# gcc (после враппера §2)
CC=gcc-16 CXX=$PWD/native/build-gcc/bin/g++-16-filtered \
cmake -S native -B native/build-gcc \
  -DCMAKE_OSX_SYSROOT=<Xcode MacOSX.sdk> -DCMAKE_CXX_FLAGS="-isysroot <Xcode MacOSX.sdk>"
uv run cmake --build native/build-gcc -j4
ctest --test-dir native/build-gcc --output-on-failure
NATIVE_TEST_BUILD_PATH=$PWD/native/build-gcc <тот же pytest>
```

Артефакты: `native/build/`, `native/build-gcc/` (untracked), `/tmp/spike_hash.py`,
`/tmp/collect_{clang,gcc}.txt`.

## 10. Открытые вопросы

- **NaN-payload/sign де-факто в области побитового контракта** — тест явно помечает
  propagation как «keep that quirk». Значит GCC-сборка нарушает существующее
  требование, а решение о судьбе этого требования отдельное от C1: либо легализовать
  и добавить ordering-дискиплину/канонизацию NaN, либо осознанно ослабить тест до
  IEEE-семантики на невалидных входах (тогда GCC-сборка становится эквивалентной
  по всем числовым путям).
- Считать ли GCC **поддерживаемым компилятором** или только диагностическим? От этого
  зависит, чинить ли: allocation-контракты под libstdc++, loader-харнесс под отсутствие
  compiler-rt, brew-sysroot workaround и ~71-флаговую фильтрацию в CMakeLists
  (например, через `CHECK_CXX_COMPILER_FLAG`).
- Нужен ли **cross-OS/cross-libm** прогон (Linux+glibc) перед финальным закрытием D14 —
  единственная конфигурация, где гипотеза «libm расходится» действительно проверяется.
- Нужен ли Release(-O2)-прогон той же пары для оценки codegen-рисков под оптимизацией
  (векторизация редукций, реассоциация) — сейчас доказана только -O0-parity.