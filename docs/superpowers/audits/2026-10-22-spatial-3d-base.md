# S00 — Аудит базы spatial-3d-native

Дата выполнения: 2026-10-23 (UTC 18:41). Исполнитель: интегратор (основной агент). Статус: **выполнено** — база зафиксирована, WIP сохранён, ветка создана.

## База

| Поле | Значение |
|---|---|
| `base_sha` | `fe4feaa560df68bce0e15b5b0d8d76eced29a062` (`impl/python-free-cpp`, "PHASE 3 2") |
| Интеграционная ветка | `impl/spatial-3d-native`, worktree `.worktrees/spatial-3d` |
| Документы плана | перенесены с `3d-new`: `3a8622f` (=118ea34) + `7f46ad7` (=4c442ce), docs-only |
| Push-состояние | `impl/python-free-cpp` на 8 коммитов впереди origin — база локальная |

`impl/spatial-3d-native` до S00 не существовала; `spatial/*` веток нет.

## Ключевая находка: открытый merge в PFC

Worktree `python-free-cpp` находится в состоянии **«All conflicts fixed but you are still merging»**: merge `lane/b` (`MERGE_HEAD=879c1195`, `AUTO_MERGE=e2c062f6`, no-ff) — все конфликты разрешены и staged, 158 файлов (+17174/−277), unstaged-изменений нет, коммит не сделан. Среди staged — golden-фикстуры обеих платформ и bootstrap-фикстуры.

**Решение S00:** база = последний committed tip `fe4feaa5` (merge не входит). Merge записан в `baseline.json → wip_patches` с disposition `integrate`: владелец PFC-линии завершает и проверяет merge; spatial портирует его по §5.4 в окне «после W00–W04, до правок W05/W14 общих файлов» (`builder.cpp`, `linkage.cpp`, `viz/plot.cpp`, `viewer/ride.cpp`, `cli/configuration.cpp`, `writer_config.hpp`, `golden/capture.cpp`, `io/value.hpp`, `equilibrium_cache.cpp`, `state_codec.cpp` — пересечение со списками Modify). Резолюции не наши и не проверены — в основание не берём.

## Ancestry (проверено `merge-base --is-ancestor`)

`lane/a` (152aa0de), `lane/c` (a4d4cfd1), `task/t20` (186856e9), `feat/drop-oracle-parity` (64c0a500), `free-cpp-cloud` (9c940c96) — все предки базы → `already_equivalent`. `wip/merge-lmp` (a5a33f1) — без worktree → `retain_separate`.

## WIP и артефакты

Полный список с SHA-256 — в `migration/spatial3d/baseline.json → wip_patches`; патчи лежат в `migration/spatial3d/wip/`. Главное:

- `lane-b-merge-staged.patch` — staged diff открытого merge, `fb3c5a8e…`.
- `t20-tracked.patch` + `t20-untracked-payload.tar` (66 файлов, `4ed21f61…`) — весь ценный контент par-t20: terrain-исходники, `test_t20.cpp`, 54 fixture-файла, producer-скрипты. Потребителей в PFC нет → provenance, `retain_separate`.
- `repomix-output.xml.zip` в PFC-worktree = `5034ba14…` — совпадает с хэшем архива из source-audit; копия в root-worktree — другой экспорт (`fb6514ed…`).
- Пять JSON-приложений `2026-10-22-branches-worktrees/*` существуют untracked в root-worktree, но **не входят в repomix-экспорт** — утверждение «фантомные приложения» подтверждено.
- `par-t20` как ветка полностью предок базы; ценность только untracked-payload.
- Prunable worktrees: `bikesim-wt-14`, `check_92b124e` (пути удалены); не трогали.
- Коммит `4c442ce` называется «W13 topology descriptor…», но содержит только 11 markdown-файлов — кода W13 нет; статус W13 остаётся `planned`.

## Dependency identities

SHA-256 lock-файлов на `base_sha`: `uv.lock`, `pyproject.toml`, `native/dependencies.lock.json`, `native/numerics.lock.json`, `native/cmake/BikeDependencyLock.cmake`, `tools/linux/{acquire,extract}_deps.sh` — значения в `baseline.json`.

## Missing binary assets

Активы, которых нет на базе и которые блокируют V-задачи: planar-baseline traces (V01), вся `tests/fixtures/spatial3d/` (V01–V06), `terrain3d_manifest` hashes (V02), `cross_slope.terrain3d` (V08), platform spatial goldens (V07, V10), внешние данные Meijaard (V01 через W03). Детали — `baseline.json → missing_binary_assets`.

## Команды (аудитируемо)

`git status --porcelain`, `diff --cached > patch | shasum -a 256`, `ls-files -u`, `cat-file -p`, `merge-base --is-ancestor`, `git show <base>:<lockfile> | shasum -a 256`, `tar -cf … -T untracked.txt`, `git worktree add`, `git cherry-pick 118ea34 4c442ce` (чисто). Сборки и тесты не запускались — контракт S00.

## Следующее

1. Приём отчёта index-fixer (W/V index v2 + stale refs) → cherry-pick/merge в эту ветку.
2. W00 — frozen headers — на `impl/spatial-3d-native` (worktree `.worktrees/spatial-3d` или отдельная `spatial/w00-headers`).
3. Окно порта lane/b merge — после W00–W04.
