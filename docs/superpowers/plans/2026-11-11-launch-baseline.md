# Базовый разгон на `f616f525` и опись форм части 1

План: `2026-11-11-launch-1-channels.md`, задача 1. Исправления ревью (`/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-plan1-review/review.md`): **P1** — совместимый профиль, пробник против объектов `launch-base`, контроль полей отказа, полные 20 с; **P5** — опись расширена на runtime-конфиги, различие «входы меняются — выходы нет» сделано проверяемым; **M-P1** — учтена холодная сборка копии.

Код не менялся. Единственное изменение в репозитории — этот файл.

## Закреплённые входы

- Коммит копии: `f616f5255a0933927b58c04e381d2acaf099c6d6` — «docs: queue the pneumatic ledger defects and the saddle limit behind ordinary rides». Отсоединённая копия `/Volumes/SSD_1TB_M2/work/wt/launch-base`, создана `git worktree add --detach` из основного чекаута; tracked-файлы в ней не менялись (`git status` чист перед удалением).
- Артефакты замера: `/Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch-base/` (ниже — `$A`), сохраняются после удаления копии.
- Зависимости копии (`native/dependencies.lock.json` @ `f616f525`): MuJoCo 3.12.0, DuckDB 1.5.6, toml++ 3.4.0, RapidJSON `24b5e7a`, GLFW 3.4, Cairo 1.18.4, Eigen 3.4.0, NumPy OpenBLAS 2.5.2.
- Трасса и профиль — как в замере на `5542fcfd` (`…/2026-11-11-launch-baseline/`): ровный старт `track_flat00_start.toml`, welded-профиль райдера.

### SHA-256 входов и выходов

| Файл | SHA-256 |
|---|---|
| `…/2026-11-11-launch-baseline/probe.cpp` (скомпилированный исходник; копия сохранена в `$A/probe.cpp`) | `13e4a64a5cd46cdc8f8d643369356442175c614d8957974b7ee1689719ac6094` |
| `$A/physics_normal_f616f525.toml` (профиль замера) | `881adf5e2b11ef9613699ef08de7e5855abbdc1c3b2ab7d081bb1dc91fd5ccfa` |
| `$A/physics_normal_full.toml` (полный профиль для контрольного резолва на `5542fcfd`) | `d8594252a5c34d9c80bbb7398457f2b2920e58fca4732e7d0245dffbc653b9f3` |
| `$A/track_flat00_start.toml` | `399c971effa654b5ab1e577caf645f5ef23a09b23cbba9aa042ca05eecc289c6` |
| `$A/rider_joint_envelope_anatomical.json` | `865658885b220eb6e3b60542c7f8332b2bee27cef3a021b45db7e24798ea0db8` |
| `$A/rider_strength_reference.json` | `071e8e8bc19cd23c229bfcad98c62fe03e56bbde374f9dcab61a9cc8c5a7d760` |
| `$A/build_probe.sh` | `739401d1780e08541084edb4cd494c9066bbc76dac2550a0c45b499feb5b09ad` |
| `$A/probe` (arm64) | `a2f5a63ca220cc92de4db0d485d28be991d23bbc88e758c446135f73c20fae1f` |
| `$A/dump_resolved.cpp` | `8bb7137ae4869d23f20162575859151b94c6c6ee71a3ea54c253431769b485fa` |
| `$A/dump_resolved` (собран от `f616f525`) | `aad8c232ffa8e3df7fbebecf7986ba7fe9c4f71c5399ba5ee1c4245b6aeca47a` |
| `$A/dump_resolved_5542fcfd` (собран от `5542fcfd`) | `328a026bb46a5e40537be6dcdaca81ea4b1df7c528b27528d899ad04091e0ff7` |
| `$A/runs/base_f616f525_flat00.csv` (16000 строк, 861 колонка) | `0e05fb683290cf4b6321b9453ff740e661a29c6fd3c8f1de258f3fa18724a872` |
| `$A/runs/base_f616f525_flat00_pos.csv` | `900f72705e0b4647cd2c3881d8220ac8bea04148be5efd6a2cd7b3df0cc8b956` |

## Сборка копии (M-P1)

```bash
git worktree add --detach /Volumes/SSD_1TB_M2/work/wt/launch-base f616f525   # из главного чекаута
cd /Volumes/SSD_1TB_M2/work/wt/launch-base
bash tools/link_deps.sh
perl -e 'alarm 2400; exec @ARGV' -- cmake --preset release                  # ≈ 12,2 с
perl -e 'alarm 2400; exec @ARGV' -- cmake --build build/native-release -j 12
```

Холодная сборка (новый `build/native-release`, полные ~10 ГБ): **92,5 с** стенки — 755 целей, по `.ninja_log` первый старт 0,012 с, последний конец 92,482 с, сумма компиляций ≈ 1103 с на 12 потоках. Плюс настройка ≈ 12 с и сборка пробника/дампера ~1 мин — итого порядка 2 мин на машину с уже выкачанными `.deps`. Эту строку в бюджете части учитывать отдельно от «6 × 25 мин + 30 мин».

## Совместимый профиль (P1)

Исходный `physics_normal.toml` = `examples/research/viewer_physics_welded.toml` @ `bb9bddee` байт-в-байт. На `f616f525` строгий ридер отвергает его целиком:

```
./dump_resolved --physics physics_normal_full.toml --track track_flat00_start.toml
→ std::invalid_argument: unknown SeatedClimbConfig parameter(s):
  launch_effort_ramp_s, launch_min_grade, launch_standstill_crank_rate_rad_s
```

Из профиля удалён блок комментария и **ровно три** неизвестных ключа: `launch_effort_ramp_s`, `launch_min_grade`, `launch_standstill_crank_rate_rad_s`. Результат `physics_normal_f616f525.toml` байт-в-байт равен собственному `examples/research/viewer_physics_welded.toml` коммита `f616f525` (тот же SHA `881adf5e…`).

Доказательство тождественности общих значений — два независимых диффа:

1. **TOML лист-в-лист** (`$A/profile_diff.json`): 116 общих листов, **0 различий значений**, удалены ровно 3 ключа `seated_climb.launch_*`.
2. **Резолвнутый `RuntimeConfig`** (`$A/resolved_f616f525.txt` vs `resolved_5542fcfd.txt`, `resolved_diff.json`): `dump_resolved` собран от каждого коммита тем же скриптом и прогнан на профилях из одного каталога `$A`, поэтому `joint_envelope_path`/`joint_strength_path` резолвятся в одинаковые строки (резолюция от директории профиля, `weakly_canonical`; JSON-ассеты в `$A` — настоящие копии, не симлинки в lane-flip). Итог: **507 общих листов, 0 различий значений**; единственная разница — 3 ключа `rider_intent.launch_*`, которые существуют только в схеме `5542fcfd`. Внутри `rider_intent` совпадают все 19 общих листов, включая `lean_gain`, `max_*_lean_rad`, `surge_*`, `target_crank_power_w`; в `writer_config`, `rider_controller`, `geometry.pose`, `monitors.*` и скалярах различий нет.

## Сборка пробника

Старый `build_probe.sh` из `…/2026-11-11-launch-baseline/` жёстко привязан к `/Volumes/SSD_1TB_M2/work/wt/lane-flip` — не использован. Вместо него параметризованный `$A/build_probe.sh` (тот же способ, что в `.scratch/devin/2026-11-11-launch-baseline/report.md`, раздел «Commands run»):

```bash
cd $A
WT=/Volumes/SSD_1TB_M2/work/wt/launch-base bash build_probe.sh \
  /Volumes/SSD_1TB_M2/work/tmp/2026-11-11-launch-baseline/probe.cpp probe
WT=/Volumes/SSD_1TB_M2/work/wt/launch-base bash build_probe.sh dump_resolved.cpp dump_resolved
WT=/Volumes/SSD_1TB_M2/work/wt/lane-flip   bash build_probe.sh dump_resolved.cpp dump_resolved_5542fcfd
```

Скрипт берёт дефайны/флаги реальных TU (`BIKE_*_CONTRACTED`, `-ffp-contract=off`, `_LIBCPP_HARDENING_MODE_EXTENSIVE`), include-ы, объектники `bike_core` и все `libbike_*.a` из `build/native-release` заданной копии и dylib-ы из её `.deps`. Контроль привязки `probe` к `launch-base`: `otool -l` показывает rpath `/Volumes/SSD_1TB_M2/work/wt/launch-base/.deps/{duckdb-1.5.6,mujoco-3.12.0}/lib` и абсолютный `libmujoco.3.12.0.dylib` из `launch-base`.

## Замер и контроль полей отказа

```bash
cd $A && perl -e 'alarm 2400; exec @ARGV' -- ./probe \
  --track track_flat00_start.toml --physics physics_normal_f616f525.toml \
  --duration 20 \
  --out runs/base_f616f525_flat00.csv --pos runs/base_f616f525_flat00_pos.csv \
  2> logs/base_f616f525_flat00.log
```

Вывод (`$A/logs/base_f616f525_flat00.log`):

```
probe: period=8 steps, max_steps=16000, control 0.010 s
probe: outcome=none step=16000 t=20.0000 failure=none abort=none
```

Контроль полей отказа (P1): `outcome=none`, `first_failure` пуст (`failure=none`), `abort=none` — исключение не поймано; горизонт полный — `step=16000`, `t=20.0000` (16000 строк CSV, последняя `time_s` = 19,99875 с). Код возврата 0 при живом прогоне, а не при ошибке.

## Формула замера

- t→V км/ч: `time_s` первой строки физики, где `speed_mps` ≥ V/3,6 (`speed_mps` = `qvel[root_x]`, `samples.cpp`). Шаг физики 1,25 мс.
- Пик каденса: `max(drive.cadence_rpm)` за 20 с.
- Отрыв переднего колеса: серии строк `tires.front.normal_load_n < 50` Н длительностью > 50 мс.

## Результат

| Метрика | `f616f525` (замер) | `5542fcfd` (план) |
|---|---|---|
| t→8 км/ч | **1,005 с** (шаг 804) | 1,01 с |
| t→15 км/ч | **2,132 с** (шаг 1706) | 2,13 с |
| t→20 км/ч | **3,254 с** (шаг 2603) | 3,25 с |
| Пик каденса шатуна | **156,0 rpm** при t = 1,125 с | — |
| Отрыв переднего колеса | **1 эпизод**, 0,255–0,989 с (734 мс, 588 шагов), min N_f = 0,0 Н | — |
| Конец прогона | v = 27,64 км/ч, x = 134,97 м, t = 20,0000 с, min pitch = −10,7° | — |

**Сравнение с `5542fcfd`:** CSV замера побайтово совпал с прежним `runs/norm_g00.csv` (SHA-256 `0e05fb68…` у обоих файлов) — числа идентичны не до округления, а бит-в-бит. Расхождения нет, причину фиксировать не требуется: единственное различие резолвнутых конфигов — три ключа `rider_intent.launch_*`, которых в схеме `f616f525` нет и которые на ровном старте без новых полей не влияют на физику.

## Опись файлов, меняющих форму в части 1 (шаг 4 + P5)

Список сгруппирован по виду записи; грепы велись в worktree `physics-2026-11-05` (`bb9bddee`), полные списки — в `$A/inv-*.txt`. Метки: **in** — входной документ строгого ридера (правится вставкой новых ключей, существующие значения не трогаются); **out** — записанный эталон/результат (не переснимается; новая форма — только миграция структуры с сохранением всех численных байт, смещений и дайджестов); **code** — C++-тест, встраивающий или читающий документы. Проверка «входы меняются — выходы нет»: дифф каждого `in` показывает только вставки ключей; каждый `out` сравнивается с прежними байтами пословно.

### A. `signals` → `senses` в `StepState`/bootstrap (`inv-senses.txt`, 52)

- **code (4):** `native/tests/reference/test_native_research_sensors.cpp`, `test_native_rider_information_boundary.cpp`, `test_native_rider_intent.cpp`, `native/tests/research/test_bootstrap.cpp`
- **in — bootstrap state (14):** `tests/fixtures/bootstrap/{distributed_rocky, native_start_welded, welded, welded_pinned}/state.json`, `welded/reset/{1,2}/state.json`, `faults/{digest,dims,malformed_drive,missing_key,nonfinite,schema,unknown_key,width}/state.json`
- **in — state-документы:** `tests/fixtures/state/signals.json`, `signals.canonical.json`; `tests/fixtures/ft2/{darwin-arm64,linux-aarch64}/{research_flat,research_savage,ride}/state.json` (6); `tests/fixtures/research/delayed_bootstrap/state.json`
- **out:** `tests/fixtures/ft2/{darwin-arm64,linux-aarch64}/{research_flat,research_savage}/{provenance,research}.json` (8); `research/delayed_bootstrap/{research,transitions}.json`; `tests/fixtures/replay/historical_v2/{episode_metrics,summary}.json`, `transitions.jsonl`; `replay/legacy_v{1,2}/{observations,transitions}.jsonl`; эталоны `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/native_encode_columns/{derived_events_real_episode_bitwise,real_episode_all_tables_bitwise,whole_episode_single_pass}.bin` и `native_numeric_completion/tire_channel_parity_across_backends.bin` (8) — те же ключи в соседних `.json`-описаниях

### B. `ShiftingSnapshot` (`cooldown_s`/`cut_remaining_s` → ответ райдера) (`inv-shifter.txt`, 83)

- **code (3):** `native/tests/io/test_state_codecs.cpp`, `native/tests/reference/test_native_drive_policies.cpp`, `test_native_state_restore.cpp`
- **in — state:** те же 21 `state.json` (bootstrap 14 + ft2 6 + delayed_bootstrap 1); `tests/fixtures/state/drive{,_minimal}.{json,canonical.json}` (4); `tests/fixtures/state/invalid/drive_{angle_width,label,nonfinite,pending_dt,schema,telemetry_range,unknown_key}.json` (7)
- **out — эталоны .bin ×2 платформы (48):** `native_drive_policies/` — `assist/{custom,eco,emtb,tour,turbo}`, `config_and_python_object_deletion`, `disabled_shifter_initial_teeth_outside_cassette`, `pedaling_parameter_branches/{filtered,mash,raw}`, `shifting/{ema,raw}-{magnitude,signed}`; `native_drivetrain/injected_valid_state_and_replay/{elastic_chain,geometric_ideal_mid_drive,ideal_mid_drive}`, `random_multiturn_staging/{3-geometric_ideal_mid_drive,3-ideal_mid_drive,6-elastic_chain}`; `native_encode_columns/*` (3), `native_numeric_completion/*` (1)

### C. Drive-телеметрия `rollback_brake_demand` (`inv-brake.txt`, 4)

- **code (2):** `native/tests/reference/recording_test_support.hpp`, `test_native_encode_columns.cpp`
- **in (1):** `tests/fixtures/bootstrap/welded/steps_control.json`
- **out (1):** `tests/fixtures/replay/legacy_v2/intervals.jsonl`

### D. Секция `runtime` в `StepState` → `brake_latch` (`inv-runtime.txt`, 14)

- **in (14):** те же bootstrap `state.json` из группы A.

### E. Runtime-конверты `config.json`/`state.json` (`runtime_schema`, `rider_intent` → `surface_labels`) (`inv-runtime-configs.txt`, 70; расширение P5)

- **in — bootstrap:** `config.json` во всех 15 bootstrap-местах (distributed_rocky, faults×9 включая `control_period`, native_start_welded, welded, welded/reset/{1,2}, welded_pinned), их `state.json` (14), `welded/steps_control.json`
- **in — ft2/исследование:** `tests/fixtures/ft2/{darwin-arm64,linux-aarch64}/{research_flat,research_savage,ride}/config.json` (6), `research_flat|research_savage/assets/recipe.json` (4), `research/delayed_bootstrap/{config,state}.json`
- **out:** ft2 `provenance.json` (4) и `state.json` ft2 также несут конверт (см. A); `tests/fixtures/platform-recordings/darwin-arm64.json`; `tests/fixtures/recording/catalog_welded.json`; `tests/fixtures/replay/{historical_v2,legacy_v1}/summary.json`, `legacy_v2/{intervals.jsonl,replay.json,summary.json}`; те же 8 эталонов `native_encode_columns`/`native_numeric_completion` `.bin`

### F. Существующие `surface_sections` — сохранить при миграции (`inv-surface-sections.txt`, 8)

- **in — трассы:** `tests/fixtures/tracks/preset_alpine_{rocky,scree}.{toml,json}` (4)
- **out — эталоны:** `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/native_pneumatic_model/model_surface_sections_bitwise/tier{0,1}.json` (4)
- Плюс `tests/fixtures/bootstrap/distributed_rocky/config.json` из группы E — hardpack по умолчанию + 3 scree-секции (единственный конверт с непустым `surface_map.sections`); у остальных конвертов секций нет.

### Сводка

- Входные документы с новыми ключами: bootstrap — 15 `config.json` + 14 `state.json` + `welded/steps_control.json`; ft2 — 6 `config.json` + 6 `state.json` + 4 `assets/recipe.json`; `research/delayed_bootstrap` — `config.json` + `state.json`; state-фикстуры 13 (`signals*`, `drive*`, `invalid/drive_*`); track-пресеты 4.
- Выходы, чьи байты не переснимаются: 56 `.bin` эталонов (8 из групп A/E + 48 из B), replay `*.jsonl` и `summary.json`/`replay.json`, ft2 `provenance.json`/`research.json`, `recording/catalog_welded.json`, `platform-recordings/darwin-arm64.json`, `historical_v2/episode_metrics.json`, pneumatic tier-эталоны (4).
- C++-тесты со встроенными документами: 9 файлов в группах A–C (4 + 3 + 2).

## Что дальше

Числа и опись — пользователю. Задача 2 (`RiderSenses` в шаге) не начиналась.
