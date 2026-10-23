# Spatial 3D Native — 09. Фаза V: verify & fix (V00–V11)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Это единственный документ пакета, задачи которого **собирают, запускают, измеряют и правят код**. Каждая V-задача: build → targeted CTest → анализ → fix в owner-модуле → re-run → acceptance → evidence → commit. Контрактные изменения — через C10 и `migration/spatial3d/contract-deviations.md`. Ничего из этого документа не выполнено при планировании.

**Goal:** Довести one-shot исходники W00–W55 до компилируемого, прошедшего все acceptance-критерии и квалифицированного состояния на macOS arm64 и Linux aarch64.

**Architecture:** V-фаза идёт по модулям в порядке зависимостей (V00 → V01 → V02/V03/V04 → V05 → V06 → V07/V09 → V08 → V10; V11 optional). Acceptance-числа прежних S-задач собраны здесь в таблицах; W-тесты уже содержат их как `bt::require`, поэтому «зелёный targeted CTest» и есть acceptance, если тест не был ослаблен.

**Tech Stack:** `cmake --preset release` (`build/native-release`, bindings OFF), `ctest --test-dir build/native-release --no-tests=error -L spatial`, статические sweeps `cmake --build build/native-release --target check_<name>`, `bash tools/run_tests.sh native|full`, presets `asan`, `rtsan`, `gui`, Linux `linux-tsan` (W52).

**Spec:** master; contracts C0–C10; phase docs 00–08 (W-секции); `docs/TESTING.md`.

## Global Constraints

- Правило фикса: любой дефект исправляется в owner-модуле (production или тест) с указанием W-задачи-источника в commit (`fix(terrain): V02 W08 bisection misses tangency root`). Ослабление допуска, fixture или статуса ради зелёного результата запрещено; изменение критерия — только через C10 с до/после evidence.
- Frozen header меняется только через C10: запись в `contract-deviations.md` → правка header → правка всех consumers в одном коммите → повтор затронутых V.
- Missing fixture / `null` identity / нулевое число проверок / skip — failure. `pending_verification` поля заполняются **результатами выполнения**, не копированием.
- Планар-регрессии: после каждого V, трогающего shared файлы, запускаются существующие planar CTest/golden gates затронутого consumer; изменение planar bytes — блокер.
- Evidence record (master §9) обязателен для каждого V: SHA, build identity, команды, exit, counts, hashes, reviewer.
- Heavy (`slow|gui|package|perf`) — на checkpoint V-задачи, не после каждого фикса.

## Общие команды

```bash
cmake --preset release -DBIKE_BUILD_PYTHON_BINDINGS=OFF
cmake --build --preset release --target <targets>
ctest --test-dir build/native-release --no-tests=error -R '^spatial\.<slug>$' --output-on-failure
ctest --test-dir build/native-release --no-tests=error -L spatial -LE 'slow|gui|package|perf'
cmake --build build/native-release --target check_<name>      # один sweep
bash tools/run_tests.sh native                                # финальный gate V-задачи, трогающей native
bash tools/run_tests.sh full                                  # V06, V10
```

## Статусы

`blocked_on_W -> in_progress -> verified -> merged`; `blocked(<producer>)` с причиной; `not_selected` — только V09 (S51–S53) и V11.

---

## V00 — Compile closure и статические sweeps

**Dependencies:** все W00–W54 в статусе `handed_to_V` (W55 не нужен). **Upstream:** G0-source, G1-core. **Deliverable:** все `bike_spatial_*` targets и production библиотеки (`bike_spatial`, `bike_pneumatic3d`, terrain3d, runtime/spatial_*, viz/spatial, replay/spatial_recipe, golden/spatial_registry) собираются в `release` и `debug`; все sweeps зелёные на новых sources.

- [ ] Проверить `migration/spatial3d/baseline.json` (`base_sha` 40 hex) и что checkout == `base_sha` + W-коммиты.
- [ ] Разобрать `contract-deviations.md`: каждая запись → C10 решение (header change + consumers) **до** компиляции соответствующего модуля.
- [ ] `cmake --preset release`; собрать по модулям в порядке: `bike_spatial` → terrain → `bike_pneumatic3d` → initialization/runtime → research/recording/replay → cli/viewer/viz → tests. Фиксы компиляции — коммит на модуль; header-правки только через C10.
- [ ] Собрать `debug`; затем все `check_<name>` sweeps; alpha analyzer findings не блокируют, остальные — блокируют.
- [ ] Linux aarch64: повторить configure/build (bindings OFF); platform-specific ошибки — отдельные коммиты.
- [ ] Evidence: список targets, compiler/stdlib/MuJoCo/provider identity обеих платформ, лог sweeps.

**Acceptance:** `cmake --build --preset release` и `debug` exit 0 на обеих платформах; все sweeps зелёные; `contract-deviations.md` не содержит открытых строк. Семантика тестов здесь не проверяется.

## V01 — Foundation: harness, registries, planar baseline, config (← S00–S05)

**Dependencies:** V00. **Upstream:** G0, G1.

| Тест | Acceptance |
|---|---|
| `spatial.harness` | положительный control pass; child `--zero-checks` exit ≠ 0 |
| `spatial.contracts_registry` | один owner/contract; distinct schema versions по C7; `base_sha` форма |
| `spatial.references` | все reference rows валидны; `WhippleParameters{}` == JSON (1e-12); Meijaard Table 2 присутствует |
| `spatial.baseline` | две platform identity заполнены; bitwise planar traces (flat, rough_uphill_savage, alpine_rocky) совпадают между двумя запусками одной identity |
| `spatial.config` | все negative fixtures → field path; valid → `!implemented && qualification_pending`; `legacy_wire_before == legacy_wire_after`; каждый `examples/**/*.toml` без `spatial` парсится как прежде |

- [ ] Запустить `harness`, `contracts_registry`, `references`, `config`; фиксы.
- [ ] Захватить planar traces на `base_sha` обеими платформами (`macos-arm64-accelerate`, `linux-aarch64-openblas`): config canonical bytes, model identity, t=0 material state, несколько committed steps, reset, probe; записать payload paths/sha256/provider/compiler/command/exit в `planar-baseline/manifest.json`; `spatial.baseline` зелёный.
- [ ] Заполнить `upstream-gates.json` статусами G0/G1 по реальному producer evidence (PFC owner, failing case, next step для красных).
- [ ] Проверить номер ADR-0004 по checkout; при коллизии — переименовать файл и ссылки.
- [ ] `bash tools/run_tests.sh native` зелёный. Commit `test(spatial): V01 foundation verified`.

**Acceptance:** M0 закрыт: таблица выше зелёная на обеих платформах; planar gates без изменений.

## V02 — Terrain (← S06–S10, S12)

**Dependencies:** V01. **Upstream:** G1.

| Тест | Acceptance (из S06–S12) |
|---|---|
| `spatial.terrain_codec` | round-trip побайтно; все corrupt cases отклонены с путём; transpose detector |
| `spatial.terrain_sampling` | плоскость: `|Δheight| ≤ 1e-12`, `|norm(normal)−1| ≤ 1e-12`, компоненты normal ≤ 1e-12; continuity на швах ≤ 1e-9; `nextafter` снаружи → outside; polynomial error убывает при h/2, h/4 |
| `spatial.terrain_raycast` | plane/bump hits `|Δt| ≤ 1e-6`; tangent → ok; near-miss (≥2e-8) → miss; two-root same-sign → earliest root; budget → `budget_exhausted` не miss |
| `spatial.terrain_generator` | same spec/seed → same bytes; feature location/depth; bank symmetry; lead-in/out flat; range/length rejects |
| `spatial.terrain_extrusion` | nodes + 1001 points bit-equal legacy height; `normal.y == 0`; `dir.y==0` ray path bit-equal legacy; world-distance oblique ≤ 1e-9; fingerprint различает knots |
| `spatial.terrain_acceptance` | все fixtures no NaN/OOB; `budget_exhausted_count == 0` |
| `spatial.terrain_budget_300m` (slow) | `nx==60001 && ny==301`; elevation bytes 144 482 408; peak RSS генератора ≤ 512 MiB; owner ≤ 256 MiB; seeds 0–4 within-platform hashes равны |

- [ ] Targeted запуск каждого теста; фиксы в owner `.cpp` (W06–W10, W12); при неустранимом расхождении с header — C10.
- [ ] `terrain_budget_300m` на обеих платформах; записать `examples/research/terrain3d_manifest.json` (sha256 per platform, RSS, cells/subdivisions stats); cross-platform различие помечать `strict|numerical` по G1 numerics contract — не скрывать.
- [ ] f32 study (optional, не блокирует): height ≤ 1e-5 m, normal angle ≤ 1e-4 rad на registered fixtures; если выполняется — отдельный раздел audit.
- [ ] Заполнить `docs/superpowers/audits/2026-10-22-spatial-3d-native-terrain.md`; обновить статус в `docs/RIDE.md`.
- [ ] `bash tools/run_tests.sh native` зелёный. Commit `test(terrain): V02 terrain contracts and budgets verified`.

**Acceptance:** все строки таблицы зелёные с evidence; M1 (terrain часть). Не доказывает lateral grip/balance.

## V03 — Topology, model, force mapping, rider, runtime state (← S13–S17)

**Dependencies:** V01. **Upstream:** G1, G2 (model compile).

| Тест | Acceptance |
|---|---|
| `spatial.topology`, `spatial.frames` | engine-array адреса для трёх fixtures + mixed; негативы → reject; q/−q инвариантность; rotate/inverse 1e-14; `body_pose` angular velocity 1e-12 |
| `spatial.model` | `planar_xml_before == planar_xml_after`; free: `root_free` FREE, `steer_joint` hinge, `actuator_trnid == steer`; масса/COM/inertia free≡constrained 1e-9; rejection combos |
| `spatial.force_mapping` | virtual work `≤ 1e-10·max(1,|p|)`; rigid rotation `|ΔL| < 1e-12`; internal pair net wrench ≤ 1e-10 |
| `spatial.drive` | spin/reverse/coast/brake; heading-90° local torque 1e-12; planar `qfrc` golden неизменен |
| `spatial.rider`, `spatial.grip` | mass budget 1e-9; planar bytes; state codec roundtrip; steer ±5…±60°: gaps ≤ 1 mm, rank == expected, mobility > 0, power residual ≤ 1e-8 |
| `spatial.runtime_state` | codecs roundtrip/reject; planar lifecycle: model_sha256/integration/material bytes после probe; `reset == fresh`; rejected restore не мутирует |

- [ ] Targeted запуск; фиксы (W13–W17).
- [ ] `grip`: если ±60° недостижимы — **не** clamp; записать достижимый диапазон, открыть C10-запись по профилю плеча (`spatial-shoulder.json`) и повторить; supported range в capability row.
- [ ] Planar regressions: существующие model/writers/runtime CTest и planar goldens — без изменений bytes.
- [ ] `bash tools/run_tests.sh native` зелёный. Commit `test(spatial): V03 topology tranche verified`.

**Acceptance:** M1 (topology часть); заявленный steering envelope подтверждён reachability/rank/load; planar bytes неизменны.

## V04 — Tyre kernels, writer, brake, state (← S20–S27)

**Dependencies:** V01, V02 (fixtures), V03 (writer bind). **Upstream:** G1.

| Тест | Acceptance |
|---|---|
| `spatial.tyre_geometry` | ортонормированность ≤ 1e-12; `|A−4π²Rr| ≤ 1e-8`, `|V−2π²Rr²| ≤ 1e-8`; fan analytic 1e-10; plane distance ≤ 1e-6; Z-rotation invariance 1e-12 |
| `spatial.tyre_carcass` | synthetic cell 10 N/0.005 J (1e-10/1e-12); force–potential ≤ 1e-5; refinement ≤ 2%; closed cycle energy ≤ 1e-9 J; probe bytes неизменны; flat/no-camber vs planar law ≤ 2% на регистре нагрузок/давлений |
| `spatial.tyre_patches` | identity-only reindex — exact sums; split/merge wrench error ≤ 1e-10; overflow transactional; reversed enumeration oracle |
| `spatial.tyre_brush_fast` | ellipse ≤ 1+1e-12; loss ≥ −1e-12; mirror ≤ 1e-10; energy closure ≤ 1e-6·scale + 1e-9 J; μ=0/Fz=0 → 0 |
| `spatial.tyre_brush_detailed` | rotation energy 1e-12, force 1e-9; refinement halves leading error; fast-vs-detailed plateau/relaxation difference записана (target ≤ 10% после согласованной калибровки — measured row) |
| `spatial.tyre_writer` | virtual work ≤ 1e-8 (scaled 1e-9); probe observational; duplicate advance → status; wrong mask → reject; warm allocations == 0; planar goldens неизменны |
| `spatial.brake_static_3d` | 5°/μ=.8 drift ≤ 1 mm/10 s; `tan(slope)>μ` скользит; `|τ_f+τ_h| ≤ ceiling·demand+1e-12`; Z-rotation only world repr; restart latch; energy ≤ 0.1%+1e-6 J |
| `spatial.tyre_state` | `encode(decode(b))==b`; 1000-step continuation bit-equal; corrupt/tier/terrain/truncated → reject, writer неизменен |

- [ ] Targeted запуск; фиксы (W20–W27). Планар `pneumatic` goldens и `writer_types` wire — неизменны.
- [ ] Если `≤2%` planar compatibility не достигается — задача открыта; изменение требования только reviewed C10 с объяснением физики.
- [ ] `asan` и `rtsan` presets на `tyre_writer`, `tyre_state`, `brake_static_3d`: ноль findings; warm allocation hook == 0.
- [ ] `bash tools/run_tests.sh native` зелёный. Commit `test(tyre): V04 spatial tyre kernels, writer and state verified`.

**Acceptance:** M1 (tyre часть); telemetry подтверждает новый writer в constrained path; cross-platform exact equality не обещается.

## V05 — Constrained integration (← S11, S18, S19)

**Dependencies:** V02, V03, V04. **Upstream:** G1, G2-startup (fresh cold start).

| Тест | Acceptance |
|---|---|
| `spatial.terrain_startup` | asset ≡ genspec fingerprint/samples; `render.sha == terrain.sha`; render resolution не меняет samples/physics hash; hfield все строки; planar MJCF bytes; masks |
| `spatial.planar_bridge` (2 s) / `_long` (10 s, slow) | flat/rough_uphill_savage/alpine_rocky: load RMS ≤ 2% weight; distance ≤ max(0.01 m, 1%); torque impulse ≤ max(0.01 N·m·s, 1%); transitions ≤ 1 step; persistence ≤ 1 window; `outcome_planar == outcome_spatial`; static flat `|Fy| ≤ 1e-9`; integrated probe/reset/restore с real writer; fresh constrained settle |
| `spatial.topology_negatives` | каждая adversarial строка → reject с field path и без side effects; rigid-motion covariance 1e-9; три entrypoints одинаково; `!implemented` для unsupported |

- [ ] Запустить `terrain_startup`; затем `planar_bridge` short; при failure локализовать geometry/law/state init/solver sensitivity — **не** ослаблять пороги; fix в owner (W18 fixture, W25 writer, W10 extrusion…) или reviewed пересмотр model spec.
- [ ] `planar_bridge_long` на обеих платформах; сохранить per-channel worst cases/timing data в `audits/2026-10-22-spatial-3d-bridge.md`; обе execution identities.
- [ ] Strict old planar goldens независимо — зелёные.
- [ ] `topology_negatives` на итоговом integration SHA; `audits/2026-10-22-spatial-3d-topology.md`; `upstream-gates.json` G1/G2 статусы по evidence.
- [ ] Открыть capability row `constrained4 + pneumatic_3d` → `implemented=true, qualification_pending=false` (только после всех строк выше). Free остаётся pending.
- [ ] `bash tools/run_tests.sh native` зелёный. Commit `test(spatial): V05 constrained bridge and negatives verified`.

**Acceptance:** M2 закрыт: native prepared/fresh constrained ride и numeric planar bridge на трёх tracks, обе платформы.

## V06 — Free root, physics benchmarks, tyre qualification, target rides (← S28–S36)

**Dependencies:** V05. **Upstream:** G2-startup.

| Тест | Acceptance |
|---|---|
| `spatial.equilibrium` | unit quaternion 1e-12; `max_lin ≤ 0.05 m/s²`, `max_ang ≤ 0.05 rad/s²`; weight residual ≤ 0.5% mg; moments ≤ 1 N·m; q/−q равные невязки; budget-fail не seal; probes не меняют material bytes |
| `spatial.bootstrap_hold` | scored `hold_qfrc == 0` exact; `hold_work` constant после armed; release `|F|<1 N, |M|<1 N·m`; bad bank → fail с причиной; restore scored не активирует hold |
| `spatial.startup_paths` | `fresh == cached` physics bytes; `imported.continuation_digest == fresh` после 1000 шагов; первая stage совпадает; matrix tiers×roots×rider; planar import → reject; corrupt cache → причина |
| `spatial.steer_controller` | delay == 20 ticks @200 Hz/0.1 s; `|τ| ≤ cap+1e-12`; restore continuation bit-equal; 3 m/s: `|roll|<0.5°` к 3 s, `|y|<0.3 m`, saturation ≤ 0.2 s; 5/8 m/s стабильны |
| `spatial.outcomes` | boundary/priority детерминированы; latched tick стабилен; terrain_invalid ≠ airborne; planar outcome bytes |
| `spatial.meijaard` (slow) | eigenvalues v∈{2,4,6,8,10}: `|Δλ| ≤ max(3%, 0.05 s⁻¹)`; `|λ|<0.05`: abs ≤ 5e-4, h-refinement ≤ 1e-4; weave 4.292±0.2, capsize 6.024±0.2 (локализация .01 m/s); KKT residual ≤ 1e-10; rollout знаки Re(λ) |
| `spatial.cornering` (slow) | ideal: lean vs `atan(v²/gR)` ≤ 1°, `ΣF_lat` ≤ 2%; MTB: `ΣF_ext = m a` ≤ 1e-3 mg, angular momentum ≤ 1%; radius 5%, speed 2%, hold work 0; steady criteria достигнуты |
| `spatial.mirror` (slow) | 2 s smooth: `|Δpos| ≤ 1e-7 m`, `|Δang| ≤ 1e-7 rad`; kernels exact signed bytes |
| `spatial.timestep` (slow) | fast {.00125,.000625,.0003125}, detailed {.0005,.00025,.000125}: events ≤ 1 coarse step; mean/peak load ≤ 2/5%; orientation ≤ 0.5°; energy ≤ 0.5%; `valid == requested` |
| `spatial.tyre_qualification` (slow) | `invalid_cases == 0`; conservation/passivity/mirror/yaw/μ→0/boundary/step-halving/refinement/split-merge/start-stop-reverse; fit/withheld RMS targets (Fz-deflection ≤ 5%, Fx/Fy ≤ 10% peak, Mz ≤ 15%, relaxation ≤ 15%) **только** при `source_measurements > 0`; иначе профиль остаётся `provisional` |
| `spatial.target_rides` (slow) | F: 20 s без capsize/corridor, `|y| ≤ .5`, hold 0; B: `|y| ≤ .75`, знаки, out-of-μ физично; R: полная дистанция, rim penetration 0, energy ≤ 0.5%; G: ellipse ≤ 1, hold на 5°, slip на недопустимом, restart; C: restore bit-equal; `profile_digest == manifest` |

- [ ] Порядок: `equilibrium` → `bootstrap_hold` → `startup_paths` → `steer_controller` → `outcomes` → `meijaard` → `tyre_qualification` → `cornering`/`mirror`/`timestep` → `target_rides`. Фиксы в owner (W28–W36). Gains steer profile могут корректироваться по измерениям **с provenance** (`provisional → calibrated-on-fixture`), но не для сокрытия неверных инерций/контактов (Meijaard независим от контроллера).
- [ ] Выбрать production dt per tier из `timestep` результатов; записать в `docs/research/spatial-step-convergence.md` с manifest; дефолт в config только после этого.
- [ ] Tyre calibration: при отсутствии measured MTB data профиль остаётся `provisional` → итог `exploratory spatial profile` (явно в audit). Данные TU Delft/Zenodo — cross-check only.
- [ ] Заполнить `docs/research/{spatial-meijaard-qualification,spatial-tyre-qualification,spatial-integrated-rides}.md` измеренными значениями; `capabilities.cpp`: `ride:spatial_free` → `qualification_pending=false` только по полному path evidence.
- [ ] `bash tools/run_tests.sh full` зелёный (physics path). Commit `test(spatial): V06 free-root physics and target rides verified`.

**Acceptance:** M3 закрыт: free-root cold start без hold, steering/grip, Meijaard, dt convergence, target matrix на обеих платформах. Калибровка — по фактическому статусу профиля.

## V07 — Research, recording, replay, goldens (← S37–S44)

**Dependencies:** V06. **Upstream:** G3-research, G4-recording.

| Тест | Acceptance |
|---|---|
| `spatial.sensor_mount` | identity/90°; bad matrix reject; offset accel 1e-9; tilted gravity 1e-12; V1 pitch == −V2 gyro_y bitwise при равном site/stage; presentation reads не меняют state; planar sample bytes |
| `spatial.sensor_pipeline` | V1 seed0 bytes прежние; V2 11 каналов; reset одна попытка; latency/age/dropout семантика; tape exhaustion → invalid; import_state continuation bit-equal; codec rejects |
| `spatial.policy_v2` | ABI1 intact (`BIKE_SENSOR_VALUE_COUNT==9`); ABI2 11 values; negative ABI controls; RAII close; no-window vs paced sequences равны |
| `spatial.truth_metrics` | `integrate_abs_slip` cases; инварианты сдвига/поворота/симметрии; planar metric keys/bytes; terminal reason/tick инвариантен к decimation/polling/capture |
| `spatial.recording_catalog` | free 7/6 rows; real freejoint без planar aliases; rejects; `planar1` сериализация bit-equal fixture; одна `channel_spatial` row/interval; embedded == file (hash) |
| `spatial.checkpoint`, `spatial.recording_assets` | continuous vs restore full semantic state + 500 rows bit-equal; rejected import не трогает cursor; corrupt/incompatible → reject без мутации; 6 assets с hashes; schema1 reader прежний; retry без stepping |
| `spatial.checked_replay` | `compare_evidence` controls; full spatial episode reproduces; каждое повреждение → ожидаемый `first_path`; recipe3 неизменна; mismatch не продвигает state |
| `spatial.golden_registry` | manifest validator rejects; классы incompatibility/regression/physical_invalid различимы |

- [ ] Targeted запуск; фиксы (W37–W44). `upstream-gates.json` G3/G4 по producer evidence.
- [ ] Golden freeze (ADR-0003): на обеих платформах `bike-recording golden freeze --accept-reviewed` **после** зелёных physical rigs и source review; записать platform trees `tests/fixtures/golden/<platform>/spatial/`; повторные запуски и разные threads (если V09 выбрал пул) — within-build bitwise; cross-platform — по полям с физическим обоснованием. Первый trace не становится эталоном автоматически — reviewer ≠ автор.
- [ ] Planar live-oracle файлы остаются до capture обеих платформ по ADR-0003.
- [ ] Обновить `tests/fixtures/spatial3d/manifest.json` (hashes, platform statuses), `docs/TESTING.md` coverage table.
- [ ] `bash tools/run_tests.sh native` зелёный. Commit `test(spatial): V07 research, recording, replay and goldens verified`.

**Acceptance:** M4 закрыт: sensor2/ABI2/FIFO/state, catalog2/container2, checked recipe4, platform goldens.

## V08 — Application, install, GUI (← S45–S49)

**Dependencies:** V07. **Upstream:** G5-gui, G6 (install/offline часть).

| Тест | Acceptance |
|---|---|
| `spatial.cli` | parser rejects/accepts; `effective_config_sha256` equality; invalid input без files/GUI |
| `spatial.viewer_pose` | chase lookat y/z; heading unwrap; N/A при invalid attitude; reset generation; identity reject |
| `spatial.viewer_smoke` (gui) | реальный display на macOS и Linux host: 2 s spatial ride с terrain, поворот/крен; simulation rows == headless run того же request |
| `spatial.reports_batch` | plot rejects; gap → segments/N/A; planar report bytes; schema1 spatial → diagnostic; workers 1/2/4 → same keys/summary; crashed child handled |
| `spatial.installed_product` (package) | из перемещённого prefix без Python/uv в PATH: ride/research/record-inspect/checked-replay exit 0 на обеих платформах; planar + schema1 reader; incompatible identity → диагностика, inspection 0 |
| `spatial.documented_examples` | каждый пример manifest парсится и резолвится с ожидаемыми mode/schema/root, duration > 0 |

- [ ] Targeted `cli`, `viewer_pose`, `reports_batch`, `documented_examples`; фиксы (W45–W49).
- [ ] Сгенерировать `examples/spatial/cross_slope.terrain3d` native generator из genspec; записать sha256 в manifest; коммит asset после review.
- [ ] Headless Release install (bindings OFF) на обеих платформах → `installed_product`; проверить resources/hashes, rpaths/load names; частичная установка не считается продуктом.
- [ ] GUI: preset `gui`, реальный display на обоих hosts → `viewer_smoke`; сравнить simulation rows с headless; визуальное ревью HUD/камеры (N/A, HOLD, следование Y). Xvfb — только smoke, не quality.
- [ ] Заполнить `audits/2026-10-22-spatial-3d-application-evidence.md` (command/exit/hash/platform для каждой строки матрицы W49); обновить `docs/spatial-3d.md` статусы.
- [ ] `bash tools/run_tests.sh native` зелёный. Commit `test(spatial): V08 application, install and GUI verified`.

**Acceptance:** M5 закрыт: native CLI/viewer/batch/plots, historical reader, install без Python на обеих платформах; GUI на реальном display.

## V09 — Performance: serial baseline, pool decision, invariance, engine probe (← S50–S53)

**Dependencies:** V06 (fixture/target rides); V07 для recording-режимов. **Upstream:** G1.

| Тест / измерение | Acceptance |
|---|---|
| `spatial.performance_report` | `rtf_threshold_met` cases; report rejects (NaN, early fall, identity mismatch, single run, batch latency label) |
| S50 serial baseline | ≥1 warmup + ≥3 measured runs ≥30 s одной identity на объявленном hardware обеих платформ; regions timings; median headless RTF записан (цель ≥ 2.0 — `target_met` true/false честно); sensors/recording/ui regions либо измерены, либо `pending` для V10 |
| `spatial.worker_ranges` | partition cases, overflow-safe |
| S51 decision | по измерениям S50: `selected` если пул решает недостачу RTF либо даёт ≥20% запаса после dispatch/barrier; иначе `not_selected` с причиной — записано в performance audit до S52 измерений |
| `spatial.worker_pool` | serial N=1 без workers; (0) reject; idempotent stop; generations; concurrent run → logic_error |
| `spatial.thread_invariance` (slow) | N∈{1,2,4,8}: complete state/qfrc/sensors/contact keys/checkpoints bit-equal на flat extrusion, rough/cross-slope, braking+steering, multi-patch, detailed |
| Sanitizers | `asan`, `rtsan` presets на pool tests — ноль findings; Linux `linux-tsan` на `worker_pool` с отрицательным race control (race обнаружена) и положительным (чисто); Darwin TSan — `unsupported`, не `passed` |
| S52 benefit | полная нагрузка S50 при N>1 vs N=1: пул остаётся только при измеренном ускорении + инвариантности; иначе удаление `--threads`/pool кода отдельным коммитом |
| `spatial.engine_pool_probe` (perf) | API locked MuJoCo присутствует/отсутствует — явно; при наличии: tyre1/engine1, tyreN/engine1, tyre1/engineN, tyreN/engineN; `qualifies_engine_pool`: ≥10% full-run gain **и** same bits; иначе no-go/`not_selected`, hook не добавляется |

- [ ] `performance_report`, затем S50 измерения (`bike-measure-realtime --profile spatial-fast-v1` через W50 driver на W36 fixture) на обеих платформах; заполнить `audits/2026-10-22-spatial-3d-performance.md` (hardware, identity, runs, regions, median RTF, target status).
- [ ] S51 decision записать **до** S52 измерений.
- [ ] Если selected: `worker_pool`, `thread_invariance`, sanitizers, S52 benefit; `BIKE_NONBLOCKING` аннотация только после RTSan evidence. Если `not_selected`: удалить pool/`--threads` код коммитом `chore(spatial): V09 remove unselected worker pool`, оставить `worker_ranges`? — нет: удаляется всё неиспользуемое, включая `partition_ranges`, с записью в audit.
- [ ] S53 probe; решение; при no-go — записка в audit, production без изменений.
- [ ] `docs/perf.md` обновить измеренными числами и caveats. Commit `perf(spatial): V09 measured baseline and parallelism decision`.

**Acceptance:** достоверный serial baseline с evidence (даже при RTF < 2.0); решения S51–S53 задокументированы; при выбранном пуле — инвариантность и санитайзеры зелёные. Достижение RTF-целей проверяет V10.

## V10 — Final qualification (← S54) → M6

**Dependencies:** V00–V09 `verified`; V09 решения; upstream G0–G6 в части продукта. **Deliverable:** `migration/spatial3d/qualification.json` со `status:"passed"` (или `failed/blocked` честно) и `audits/2026-10-22-spatial-3d-qualification.md`.

- [ ] Зафиксировать identity кандидата после обновлений PFC: source SHA, dirty payload разобран, dependency/compiler/provider/config hashes; старые логи другой identity — исторические.
- [ ] `spatial.qualification_registry` зелёный на документе `pending`; затем выполнять gates и переводить статусы.
- [ ] Регрессии: `bash tools/run_tests.sh full` на стабилизированном кандидате обеих платформ; отдельно `ctest -L spatial -LE ''` включая `slow|gui|package|perf` entries перечисленно; ноль skips/missing fixtures.
- [ ] Физические доказательства по порядку: terrain/raycast (V02) → tyre load/slip/camber/energy/limits (V04/V06) → free root/steer/rider reaction (V03/V06) → equilibrium/hold release (V06) → Meijaard применимость (V06) → convergence (V06) → planar preservation (все). Отрисовка не заменяет.
- [ ] Пользовательская цепочка одной версии на installed product: config→terrain→startup→ride/research→record schema2→inspect→checked recipe4→offline report→batch; planar controls schema1/ABI1.
- [ ] Детерминизм полного состояния (повтор одной сборки, threads если selected) и поддержанные санитайзеры.
- [ ] Performance итогового продукта (дополнение S50): CLI, запись on/off, headless (median RTF ≥ 2.0), реальный viewer с моделью/terrain при 1× (средний достигнутый RTF ≥ 1.0, долг/опоздания, frame rate) — обе платформы; диагностические невалидные префиксы исключены.
- [ ] macOS-arm64 и Linux-aarch64: отдельные эталоны и identities; cross-platform bitwise не заявлять; непроверенный host → `pending`.
- [ ] Заполнить `qualification.json`: milestones M0–M6 со статусами/evidence, gates G0–G6; `validate_qualification` зелёный; `status:"passed"` только если все обязательные pass. Caveat ledger: `tyre_calibration` статус, unsupported ranges, optional `not_selected`.
- [ ] Обновить master §12 статусы, `docs/{RIDE,TESTING,perf}.md`. Commit `docs(spatial): V10 final spatial qualification`.

**Acceptance:** M6: все обязательные проверки актуальны и пройдены на финальной интеграционной базе обеих платформ; S51–S53 `verified` или `not_selected` при выполненных serial целях; каждый объявленный профиль имеет evidence по физике/reproducibility/IO/платформам/скорости. Если RTF или realism не подтверждены — остаётся открытым критерием профиля, без маскировки.

## V11 — GPU feasibility memo (optional, ← S55)

**Dependencies:** V09 (измеренный CPU: serial и пул/not_selected). **Not required for V10.**

- [ ] `tools/spatial_gpu` standalone configure/build; `spatial.gpu_decision` зелёный.
- [ ] Нет GPU host → `unsupported-host`/`pending-measurement` в записке; завершить.
- [ ] При host: зафиксировать SDK/device identity, изучить официальную документацию API (version/date/URL); бюджет ≤ 2 инженерных дня; прогрев, ≥200 повторов микротеста, сравнение полной нагрузки с лучшим квалифицированным CPU; распределения, не лучший запуск; H1/H2/H3 по helper без изменения критериев после результата.
- [ ] Решение каждой гипотезы; при `go-for-separate-design` — оценка стоимости и отдельная спецификация; CPU default; удалить случайные hooks вне `tools/spatial_gpu`; свободный ADR номер только при принятом решении.
- [ ] Commit `docs(spatial): V11 GPU evidence and bounded decision`.

**Acceptance:** записка разделяет измеренное/предполагаемое/неподдержанное с воспроизводимыми входами. Не влияет на завершение 3D.

---

## Сводная матрица S → W → V

| S | W | V | Milestone |
|---|---|---|---|
| S00 | S00 (pre-W) | V00/V01 | M0 |
| S01–S05 | W01–W05 (+W00) | V01 | M0 |
| S06–S10, S12 | W06–W10, W12 | V02 | M1 |
| S11 | W11 | V05 | M2 |
| S13–S17 | W13–W17 | V03 | M1 |
| S18, S19 | W18, W19 | V05 | M2 |
| S20–S27 | W20–W27 | V04 | M1 |
| S28 | W28 | V06 | M3 |
| S29–S36 | W29–W36 | V06 | M3 |
| S37–S44 | W37–W44 | V07 | M4 |
| S45–S49 | W45–W49 | V08 | M5 |
| S50–S53 | W50–W53 | V09 | — |
| S54 | W54 | V10 | M6 |
| S55 | W55 | V11 | optional |

## Что записывать в evidence каждой V

`task_id`, baseline/implementation/integration SHA; dirty status; test/executable hashes; compiler/stdlib/MuJoCo/provider/lock identities; platform/CPU/thread count; physics dt/grid/tier; config/terrain/coefficient/noise hashes; exact command/exit; check counts; log/artifact hashes; comparison rule; observed residuals/RTF; reviewer; limitations; список fix-коммитов с W-источником.
