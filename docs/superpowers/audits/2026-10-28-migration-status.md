# Статус миграции Python → C++ — 2026-10-28

Срез: integration `impl/python-free-cpp` @ `7020a89b` (2026-10-28 21:14).
Все цифры получены из `migration/test-port-map.json`,
`migration/python-removal.json`, `git log` и
`migration/cpp-linux-completion-evidence.md` на этом коммите.

## 1. Где мы сейчас

### Ledger тестов (`test-port-map.json`, 2 394 записи)

| Срез | port | golden | drop | planned | закрыто |
|---|---:|---:|---:|---:|---:|
| `fe4feaa` (10-23 17:32, старт) | 201 | 60 | 36 | 2 084 | 12.5 % |
| `40b93541` (10-28 04:45) | 894 | 351 | 113 | 1 035 | 56.8 % |
| `7020a89b` (10-28 21:14) | 1 272 | 356 | 173 | **593** | **75.2 %** |

За ~5,2 суток слито 40 merge'ей и закрыто ~1 490 записей (≈290 в сутки).

### Ledger удаления Python (`python-removal.json`, 535 файлов)

| Статус | Файлов | Строк Python |
|---|---:|---:|
| `replacement_landed` | 133 | 37 945 |
| `planned` | **402** | **100 125** |

**Важно:** 75 % по тестам не значит 75 % миграции. По удалению Python
закрыто ~25 % файлов и ~27 % строк. 286 из 402 оставшихся файлов — это
production-модули `src/bike_sim/**`, а не тесты.

## 2. Что ещё осталось

### 2.1 Сейчас в работе (3 агента)

| Агент | Батч | Объём | Состояние |
|---|---|---|---|
| `ebb7bef4` | 3.35 platform files (lane/d) | ~105 записей / 9 файлов | 10 коммитов на lane, отчёт написан; ревью ещё не было |
| `ce078910` | 3.36c (lane/a) | `test_qualification_report.py` 43 + `test_native_toolchain.py` 29 | портирование |
| `286343be` | 3.36d (lane/e) | `test_native_runtime.py` 39 + `test_offline_bundle.py` 35 | портирование |

По каждому дальше: ревью → фикс-раунд → merge.

### 2.2 Тестовые записи, которые ещё никому не назначены (~300)

Остаток 3.36 portability, примерно 235 записей:

| Файл | planned |
|---|---:|
| `test_ci_lanes.py` | 34 |
| `test_release_gate.py` | 34 |
| `test_install_modes.py` | 28 |
| `test_native_dependencies.py` | 28 |
| `test_offline_install.py` | 28 |
| `test_dev_commands.py` | 18 |
| `test_offline_tools.py` | 14 |
| `test_native_coverage_tools.py` | 12 |
| `test_viewer_launch_platform.py` | 11 |
| `test_worker_platform.py` | 11 |
| `test_development_docs.py` | 9 |
| `test_baseline.py` / `test_headless_imports.py` / `test_dependency_policy.py` | 4 / 3 / 1 |

Многие из этих файлов (`ci_lanes`, `release_gate`, `offline_*`,
`install_modes`, `dev_commands`, `coverage_tools`) проверяют Python-инструменты,
которые план заменяет в задачах 3.41–3.45 (CMakePresets, CPack, CI). Часть
записей станет `drop` с указанием нативной замены, а не `port`. Это нужно
решать по каждой функции.

Остаток 3.29b — 56 записей: `test_native_check_tools.py` 51 и
`test_native_contract_checks.py` 5.

### 2.3 Записи, оставленные `planned` на честном красном (~37)

Это не дыры в работе: каждая такая запись блокируется дефектом с
задокументированной сигнатурой.

- Welded-baseline drift / live-oracle parity:
  - `runtime_step` 6, `runtime_commands` 4, `runtime_rollout` 4;
  - 11 записей research (environment 10, capture 1, recording 1);
  - `closed_form_rider` / `coupled_rider_task` (`flat_launch`, `tissue_damping`).
- `realistic_pedelec_acceptance` 4 — ждут решения по Task 3.09.
- `native_pneumatic_tire` 2, `native_viewer_snapshot` 2 — известное baseline-семейство.
- `test_preserved_scientific_pins_and_python_floor` — новый реальный конфликт:
  в `pyproject.toml` стоит `>=3.13,<3.14`, а тест ожидает `>=3.13`. Нужно
  решение: менять pin или assertion.

Закрыть их можно только физическими или контрактными фиксами (как F1–F5),
а не портированием тестов.

### 2.4 Задачи плана после тестовых батчей (`docs/superpowers/plans/2026-10-20-cpp-linux-completion.md`)

| Блок | Содержание | Статус (насколько проверено) |
|---|---|---|
| CHECKPOINT 3-C…3-F | L2 после 3.17 / 3.23 / 3.29b / 3.36 | **Не выполнены как checkpoint'ы.** Merge'и шли со scoped-верификацией |
| 3.37–3.40 | Порт `src/bike_sim/validation/**` (tyre calibration, rigs, plant/rider rigs, benchmarks/`bike-validate`) | `native/src/validation/` уже есть; полноту против плана в этой сессии не проверял |
| 3.41–3.45 | CMakePresets, sanitizer/RTSan, coverage, CPack, CI | `CMakePresets.json` (45 presets) и `packaging/` уже в дереве (коммит `ef66e96a` PHASE 3); закрытие задач не аудировано |
| CHECKPOINT 3-G, 3-H | L2 | не выполнены |
| 4.01 | Golden completeness + сверка ledger'ов | не начато |
| 4.02 | `run_tests.sh` только на native graph | не начато |
| 4.03 | Release-квалификация T61 (обе платформы) | не начато |
| 4.04 | No-Python enforcement T62 | не начато |
| 4.05 | Удаление Python-исходников (T63 ч.1) | **блокер: решение OD-1** по `tools/research/**/*.py` (22 файла planned) — нужен ответ пользователя |
| 4.06 | Удаление binding graph (T63 ч.2) | не начато |
| 4.07 | Документация (T60) | не начато |
| 4.08 | Final gate + `final-closure.json` | не начато |

### 2.5 Долги по верификации (накопились)

1. **На integration ни разу не прогонялся `full` gate** с тех пор, как
   слиты 3.23 (`step.cpp` linkage), 3.34 (`step.cpp`/`step.hpp`,
   `cli/batch.cpp`) и десятки reference-тестов. Каждый merge проверялся
   только своими target'ами на обеих платформах. По правилам AGENTS.md для
   physics-path изменений нужен `full`. Это первый пункт перед Phase 4.
2. Ledger: на landed-записях других батчей ещё ~400 T01-placeholder'ов в
   `assertions_capture`, и 1 045 записей с пустым `collected_node_ids`
   (большинство — `planned`). Задача 4.01 это проверит и заблокирует.
3. Политика `snapshot_*` в `python-removal.json` расходится между батчами:
   3.27 обновлял поля, остальные замораживали.

## 3. На что уходит время

### 3.1 Время самих прогонов (wall-clock, из evidence и отчётов агентов)

| Прогон | Длительность | Где встречается |
|---|---|---|
| `bash tools/run_tests.sh native`, darwin | **~4 ч** | 3.31, 3.29b, 3.23 lane-gate'ы |
| ↳ pytest-хвост внутри gate | 1 ч 47 мин | evidence:8907 |
| ↳ legacy headers sweep внутри хвоста | ~85 мин | evidence:8911 |
| ↳ headers sweep | 9 198–9 322 probe-TU | evidence:8893 |
| Oracle pytest для 3.29b | 5 524 с (~1,5 ч) | task-C329b |
| `migration_T56` cmake-fixture | ~20–21 мин на платформу (1 213 / 1 274 / 1 291 с) | task-C329b |
| `reference.coupled_rider_task` | 25–36 мин (`TIMEOUT 5400`) | evidence:7018, 9202 |
| `migration_T53_portability` | 4,5–6 мин darwin, ~1–2 мин linux | M38 |

Один честный lane-gate `native` занимает полдня. Поэтому lane'ы
квалифицируются scoped-CTest'ом на двух платформах, а полный gate
откладывается, и из этого вырос долг 2.5.1.

### 3.2 Накладные расходы процесса (основной расход)

- **Один батч = 4–5 последовательных запусков агентов:** порт → ревью →
  фикс-раунд → (ре-ревью) → merge. Только порт идёт параллельно, остальное
  сериализовано. 40 merge'ей за ~124 ч дают в среднем **~3 ч на один
  merge** при 3 слотах.
- **Лимит 3 агента:** ревью и merge'и занимают те же слоты, что и
  портирование. В типичный момент 1–2 слота заняты ревью или merge, а
  портирует только 1–2 агента.
- **Сериализация merge'ей:** каждый merge пересчитывает key-exact union
  по 2 394-записному (~42 тыс. строк) JSON. Lane'ы отстают от tip на 1–6
  merge'ей, отсюда конфликты в `Phase3Ports.cmake` / `Checks.cmake` (EOF-append).

### 3.3 Переделки (дефекты, найденные ревью)

Почти каждый батч прошёл хотя бы один фикс-раунд. Повторяющиеся классы:

| Класс дефекта | Повторов | Пример |
|---|---:|---|
| Дрейф сериализатора ledger'ов (indent=1, `ensure_ascii=False`, потеря trailing LF) | **7** | 3.15, 3.17, 3.32, 3.34, 3.36-t1 |
| Неверные счётчики узлов/checks в отчёте | почти каждый | 3.26, 3.27, 3.30, 3.32, 3.22b |
| T01-placeholder'ы на landed-записях, нет `golden_platforms` | 3+ | 3.15, 3.33a |
| Неверный формат `native_test_id` | 1 (65 записей) | 3.31 |
| Ослабленные assertion'ы / пустые маркеры | 4+ | 3.26, 3.31, 3.36-t1 |
| Нарушение lane-дисциплины (коммит прямо в integration) | 1 | 3.34 |
| Ложное срабатывание ревью (HIGH опровергнут прогоном) | 1 | 3.36b |

### 3.4 Сбои агентов и окружения

- **~7 агентов умерли от connection error** (C14 ×2, F5, C17, F5-fix,
  3.33b и др.). Каждый раз нужна проверка lane и рестарт.
- **Linux-квалификация каждый раз переоткрывается заново:**
  - `/src` в контейнере — не git, а tar-снимок, и он устаревает;
  - путаница `native/build` (binding graph, без `reference.*`) и
    root-graph `build/native-release` / `/build/root-*`;
  - в 3.34 агент принял «DNS не резолвится» за недоступность контейнера.
- **Объём батчей недооценён планом:** 3.36 оказался 558 записями вместо
  ~24; 3.22b уже был слит раньше; в 3.21b порты уже существовали.

## 4. Рекомендации для ускорения

1. **Проверка ledger'ов перед коммитом.** Скрипт: каноничный JSON
   (indent=2, ascii, LF), уникальность ключей, headers = count, нет
   T01-placeholder'ов на landed, `golden_platforms` у golden. Убирает
   классы дефектов 1, 3 и 4, то есть большую часть фикс-раундов.
2. **Lane-playbook в AGENTS.md.** Какой build graph использовать для
   reference/CTest, как синхронизировать контейнер, формат
   `native_test_id`, где брать счётчики checks (`ctest -V`, `PASS N
   checks`). Убирает переоткрытие окружения и арифметику в отчётах.
3. **Для мелких или чисто ledger-батчей** (3.21b, 3.22b) объединять ревью
   и merge в одного агента. Это экономит один цикл.
4. **Один `full` gate на integration сейчас**, до Phase 4. Вероятно
   несколько часов, но это снимает накопленный риск.
5. **Принять решения пользователя, которые блокируют дальнейшую работу:**
   OD-1 (`tools/research/**/*.py`), конфликт `requires-python`,
   pedelec/Task 3.09.
6. **Остаток 3.36 разбирать по принципу «заменено задачами 3.41–3.45 → drop
   с указанием нативной замены»**, а не портировать Python-инструменты
   один к одному.
