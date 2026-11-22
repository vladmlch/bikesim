# Единое завершение и интеграция работ по плоской (CPU) симуляции велосипеда и райдера

**Дата:** 2026-11-21  
**Рабочее дерево:** `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/surface-stage-a-2026-11-18` (ветка `feat/surface-stage-a-2026-11-18` поверх `main = 0502660d`)  
**Статус:** Утверждённая спецификация дизайна (Волны 1–6)

---

## 1. Контекст, цель и жёсткие границы области

Цель работы — довести до полного завершения все незакрытые направления плоской (2D, CPU FP64) симуляции в репозитории и связанных рабочих папках, прогнать единую перезапись эталонов (`goldens` и `poses`) и полный квалификационный гейт `bash tools/run_tests.sh full`, после чего влить результат fast-forward-слиянием в `main` и отправить в `origin/main`.

### Исключённые параллельные стримы (не трогаются)
1. **GPU / FP32 / Metal:** ветки `feat/gpu-shared-*`, `feat/sp1-*`, `feat/sp2-other-force-laws`, `feat/sp2-golden-bundle-f32`, `feat/sp3-*`, `research/fp32-physics`, `research/colab-gpu` и файлы `native/src/gpu/`, `tools/colab_gpu/`.
2. **Пространственная 3D-модель:** ветки `3d-new`, `spatial-3d`, `wip/3d-new-battery-removal`.
3. **Ранее закрытые и отвергнутые эксперименты (`docs/agents/simulation-decisions.md`):**
   - `clutch-flip-wip`, `wip/leg-t4-code` — уже реализованы в `main` («Действует», п. 9 и п. 43);
   - `wip/pedaling-rhythm`, `exp/leg-jitter-2026-11-08` — закрыты в «Пробовали — не сработало» (п. 2 и п. 10) и не переносятся.
4. **Linux-контейнер (`tools/linux/container.sh`):** отключён сторожевым блоком `PLAN-CHANGE GUARD` (`darwin-arm64` wave); квалификация выполняется нативно на macOS (`darwin-arm64`), команды обновления `linux-aarch64` эталонов фиксируются в документации.

---

## 2. Архитектура конвейера и управление тестовым бюджетом

Все изменения выполняются последовательно в одном подготовленном рабочем дереве `.worktrees/surface-stage-a-2026-11-18` (с проверенным бэкапом `/Volumes/SSD_1TB_M2/work/mujoco-stage-a-backup-2026-11-19/run-20261119T115938Z` и активной macOS `arm64` сборкой `build/native-release`), без переписывания коммитов `37946646..a0bb59e1`.

Чтобы не перезаписывать эталоны и не запускать 35-минутный `full`-гейт после каждой физической задачи по отдельности, конвейер разбит на **6 волн**:
- внутри волн 1–5 гоняются быстрые целевые тесты (`ctest -R`) и инкрементальные проверки статического анализа (`cppcheck`, `frontends`, `clang-tidy`) на изменённых единицах трансляции;
- в Волне 6 выполняется **ровно одна** синхронная перезапись `poses/*.npz` и `tests/fixtures/golden/darwin-arm64/**`, за которой следует **один** полный квалификационный прогон `bash tools/run_tests.sh full` на чистом `HEAD`.

Расчётный тестовый бюджет (по `.git/bike-gate/timings.tsv`):
- целевые прогоны в волнах 1–5: ~8–10 запусков (суммарно ~8–10 мин);
- точечные прогоны приёмочных поездок перед записью эталонов (`rider_launch_rides_*`, `realistic_pedelec_acceptance`, `crank_reposition`, `test_pneumatic_energy_balance`): ~15–20 мин;
- промежуточные `check_change.sh` на границах стадий: 2 запуска (~6 мин);
- финальный `bash tools/run_tests.sh full` (`native` + `slow` + `package`): 1 запуск (~35–40 мин);
- **итого:** ~65–75 мин машинного времени проверок.

---

## 3. Спецификация по волнам

### Волна 1 — Завершение Surface Stage A и закрытие двойного учёта `crr` у `pneumatic`

Опирается на утверждённую спецификацию [`2026-11-18-surface-stage-a-design.md`](2026-11-18-surface-stage-a-design.md) и продолжение `01-mujoco-surface-stage-a-completion.md`:

1. **`DistributedTireWriter` и 5 миноров задач 1–2 (уже в рабочем дереве):**
   - сохранить двухпроходный `accumulate_validated`, масштабирование $\sqrt{r_k}$, насыщение сцепления под нагрузкой колеса, мировой момент `m_roll`, защиту от переполнения аккумулятора `crr_delta` (проверка конечности выходных обобщённых сил и агрегатов до коммита состояния) и overflow-безопасную асимптотику `load_sensitivity_factor` при `u > 1e150`;
   - зафиксировать чистым коммитом без перезаписи `savage_pneumatic.npz` до Волны 6.
2. **Энергетическое замыкание `Pneumatic` (`native/src/pneumatic/{carcass,model}.*`, `native/src/writers/pneumatic_tire.*`):**
   - добавить квадратуру площади хорды воздуха `casing_chord_integral_m2` и работу давления $W(d)$, полную упругую энергию луча `ray_elastic_energy_j` и энергию ветви Максвелла `ray_maxwell_energy_j` (с корректным предельным переходом при `carcass_stiffness_n_mm2 == 0` на насыщении без деления `0/0`);
   - устранить двойной учёт гистерезиса при ударе в обод (оставить один канал диссипации обода) и публиковать `carcass_dissipated_power_w >= 0`.
3. **Гетерогенная поверхность по лучам (`native/src/pneumatic/brush.*`, `pneumatic_tire.*`):**
   - реализовать `DiscretisedBrush::step_heterogeneous` без динамических аллокаций (`BIKE_NONBLOCKING`), по-лучевой расчёт покрытия и мировой момент сопротивления качению `m_roll`.
4. **Снятие двойного учёта `crr` для `pneumatic` («Не решено», п. 2 `simulation-decisions.md`):**
   - выставить `[resistance] crr = 0.0` в `examples/research/viewer_physics_pneumatic.toml` и `tests/fixtures/config/viewer_physics_pneumatic.toml` (с синхронным обновлением `tests/fixtures/config/resolved/pneumatic.json`).
5. **Тестирование (`native/tests/reference/test_pneumatic_energy_balance.cpp`):**
   - квадратура хорды воздуха, удержание Максвелла против независимой убыли запасённой энергии (`< 1e-14` Дж пассивности), одиночный гистерезис обода, совпадение `stored_energy` со стендом `bt::pneumatic::Rig`, проверка отсутствия паразитной реакции на раму через сравнение `qacc` с `crr_delta` и без него, гладкое пересечение границ при `blend_m = 0.0` и `0.4` с полным проходом пятна контакта через границу, и строгий 60-м заезд по смешанному покрытию (`model_valid`, 0 нарушений пассивности, `|residual_j| / energy_scale_j <= 0.05`).

---

### Волна 2 — Диагностика, перепроверка запретов и фиксация артефактов Implicit-FSM аудита

1. **Артефакты Implicit-FSM аудита:**
   - перенести итоговые верифицированные материалы аудита ревизии `0502660d` (`fsm-map.md`, `R0.md`, `R1.md`, `R2.md`, `Rfinal.md`, `rfinal_data.json`, `journal.md`) из `Projects_downloaded/mujoco-implicit-fsm-audit-swe2/` в `docs/superpowers/audits/2026-11-21-implicit-fsm/`.
2. **П.6 (`simulation-decisions.md`): перепроверка запретов 36–41:**
   - выполнить аудит актуального C++ кода по всем шести запретам: отсутствие скрытых стабилизирующих моментов (36), нулевой `armature` на шарнирах (37), отсутствие формул Pacejka (38), отсутствие вязкого демпфирования во всех трёх моделях шин `compliant_2d`, `distributed_2d_reference`, `pneumatic` (39), недеформируемость дороги в 2D-модели (40), отсутствие автоматики мотора против вилли/пробуксовки в основном рантайме (41);
   - обновить запись п. 6 и раздел «Запреты» в `docs/agents/simulation-decisions.md` со ссылками на код и тесты.
3. **П.19 (`simulation-decisions.md`): разбор отскока вилки при резкой остановке на подъёме 10 %:**
   - воспроизвести сценарий торможения 100 % со скорости ~24 км/ч до полной остановки на уклоне 10 %, снять временны́е ряды хода и скорости вилки, сил воздушной пружины, демпфера сжатия/отбоя (`fork_rebound = 9`), продольного клевка рамы и нормальной реакции переднего колеса;
   - оформить отчёт `docs/research/2026-11-21-fork-rebound-stop-diagnosis.md` и обновить п. 19 в `docs/agents/simulation-decisions.md`. Если диагностика выявит ошибку знака/масштаба в самом коде демпфера отбоя, исправление выносится на отдельное подтверждение пользователя перед Волной 4; если поведение является прямым следствием возврата запасённой энергии клевка при штатной кривой отбоя — константы демпфера без запроса не меняются.

---

### Волна 3 — Критичные физические и граничные дефекты Implicit-FSM аудита + П.12 (обязательные поля профиля)

1. **`P01b` — Запрет `reposition_on_stall` без обгонной муфты мотора (`C1`):**
   - в `bike::configuration::validate` (`native/src/configuration/validate.cpp:1330`) и на границе декодирования `decode_drive_policies` (`native/src/writer_config.hpp`) требовать `drive.motor_clutch == true` всякий раз, когда `drive.pedaling.reposition_on_stall == true` (в дополнение к `pedal_attachment == "spindle"`), полностью закрывая возможность автоматического прокручивания жёстко связанного ротора назад.
2. **`P04b` — Единый предикат активности сброса момента при переключении:**
   - выделить единую inline-функцию `shift_cut_active(double cut_remaining_s)` в `native/src/drivetrain/shifting.hpp` и использовать её во всех точках чтения (`writers/drivetrain.{hpp,cpp}`, `runtime/step.cpp`, `rider/launch_plan.{hpp,cpp}`), устранив расщепление полуинтервала `(0.0, 1e-12]`.
3. **`P07a` и `P07b` — Наблюдательная чистота `probe_step_inputs` и паритет тормоза отката (`C4`, `C5`):**
   - **`P07a`:** в не-продвигающем пути пробы (`PhysicalStep::probe_step_inputs` / `apply_forces` при `advance == false`) сохранять исходные значения `model->dof_frictionloss` на двух колёсных степенях свободы и флаг свежести кэша позиций/скоростей `Stepper` до вызова `brake_.apply(...)` и безусловно восстанавливать их при выходе (RAII-сторож), чтобы проба не мутировала буферы `mjModel`;
   - **`P07b`:** реализовать константный метод `peek_rollback_brake_demand(double speed_mps, const RideControl &control) const` без изменения `rollback_hold_` и подавать его результат в разрешение тормозов при `resolved == nullptr`, чтобы `probe_step_inputs` при взведённом удержании на склоне возвращал ровно те же тормозные усилия и границы, что и `advance_physics`.
4. **`C2` и `C3` — Реляционные инварианты восстановления состояния привода (`set_state` и `decode_policies`):**
   - в `PedalingPolicy::set_state` (`native/src/drivetrain/pedaling.cpp`) и декодере снимков политики проверять:
     - `!r.stall_elapsed_s.has_value() => r.crank_rad == 0.0 && r.wheel_rad == 0.0`;
     - `r.goal_rad.has_value() <=> (r.reason == 1 || r.reason == 2)`;
     - `r.goal_rad.has_value() => (r.cooldown_s == 0.0 && s.effort == 0.0)`;
     - `(s.coasting || (s.reposition && s.reposition->goal_rad.has_value())) => s.effort == 0.0`;
   - в `AssistController::set_state` (`native/src/drivetrain/motor.cpp`) и wire-декодере снимка ассиста проверять:
     - `release_hold_left_s > 0.0 => (hold_power_w > 0.0 && hold_torque_nm > 0.0)`;
     - `hold_power_w > 0.0 <=> hold_torque_nm > 0.0`.
5. **П.12 (`simulation-decisions.md` и очередь `2026-11-16-reverify-fixes.md`): перенос оставшихся дефолтов в обязательные поля профиля:**
   - перевести оставшиеся необязательные поля `SeatedClimbConfig` в `native/src/configuration/model_config.hpp` (`enabled`, `target_crank_power_w`, `lean_gain`, `max_backward_lean_rad`, `surge_power_w`, `surge_grade`, `surge_recovery_rate`, `front_load_share_target`, `lean_trim_gain_rad_s`, `lean_trim_limit_rad`) в обязательные ключи схемы (`required = true`, с инициализацией недопустимыми сторожевыми значениями в `IntentConfig` / проверкой в `post_init`);
   - проверить остальные секции `SimulationPhysicsConfig` на оставшиеся скрытые кодовые дефолты таймингов/пределов и синхронно прописать явные значения во всех физических `.toml` профилях (`examples/research/*.toml`, `tests/fixtures/**/*.toml`) и конструкторах `default_physics()`.

---

### Волна 4 — Глобальные физические изменения: П.11 (тормоз на месте) и П.9 (просадка 30/30 % по умолчанию)

1. **П.11 — Суммарный момент тормоза на месте не выше `brake_ceiling_nm × нажим` (`native/src/runtime/static_brake.{hpp,cpp}`, `step.cpp`):**
   - при расчёте `StaticBrake`:
     - упруго-демпфирующий момент защёлки `tau_hold[i]` на колесе `i` ограничивается по модулю величиной `cap_hold = std::min(hold_.torque_nm, ceiling_nm_) * u`;
     - граница кулоновского трения решателя `model->dof_frictionloss[dof_i]` устанавливается равной остатку тормозного бюджета `std::max(0.0, ceiling_nm_ * u - std::abs(tau_hold[i]))` (порядок вызовов в `step.cpp`: сначала вычисляется `brake_hold = brake_.hold_qfrc(...)`, затем `brake_.apply(m, ..., brake_hold)` выставляет остаточные `dof_frictionloss` и при их изменении вызывает `stepper_->note_posvel_stale()`);
     - в `ProbeResult` поля `front_brake_bound_nm` / `rear_brake_bound_nm` считывают выставленные на время пробы остаточные `dof_frictionloss[dof_i]` до их восстановления RAII-сторожем `P07a`, а момент защёлки читается из `probe.components["brake_hold"]`, так что сумма `brake_bound_nm + |brake_hold[dof_i]| == ceiling_nm_ * u`.
   - Физические свойства схемы:
     - на любом шаге `|tau_hold[i] + tau_solver_friction[i]| <= |tau_hold[i]| + (ceiling_nm_ * u - |tau_hold[i]|) = ceiling_nm_ * u` (при `ceiling_nm_ = 200` Н·м и `u = 1` — не более `200` Н·м вместо прежних `400` Н·м);
     - на малых и средних уклонах защёлка полностью подавляет численную ползучесть колеса;
     - при превышении внешним моментом `200 * u` Н·м колесо трогается и переходит в режим чистого скольжения (`tau_hold = 0`, `dof_frictionloss = 200 * u`) непрерывно, без скачка удерживающего момента.

2. **П.9 — Просадка подвески 30/30 % по умолчанию (`native/src/geometry/specs.hpp`, `native/src/configuration/suspension_spec.hpp`, `model/builder.cpp`, `initialization/request.cpp`, `cli/configuration.cpp`):**
   - установить базовые константы по умолчанию в `BikeSpecs` и `SuspensionInputs` равными откалиброванным значениям основного профиля `welded` (`compliant_2d`, отчёт `docs/research/2026-11-17-default-sag-calibration.md`):
     - `fork_initial_psi = 97.0` (вместо `85.2`),
     - `shock_stiffness = 81550.0` Н/м (вместо `114600.0`);
   - в точках сборки модели с физическим профилем (`model/builder.cpp`, `initialization/request.cpp`, `cli/configuration.cpp`), когда `BikeSpecs` несёт базовые значения по умолчанию (не переопределённые пользователем явно), синхронизировать пару `(fork_initial_psi, shock_stiffness)` под активный бэкенд шин так же, как уже синхронизируются радиусы колёс для `pneumatic`:
     - `distributed_2d_reference` → `(90.8 psi, 84372.0 N/m)`;
     - `pneumatic` → `(113.8 psi, 73557.0 N/m)`;
     - `compliant_2d` → `(97.0 psi, 81550.0 N/m)`;
   - обновить модульные проверки дефолтов (`test_horst_link.cpp`, `test_inertia.cpp`, `resolved/*.json`) и статус п. 9 в `docs/agents/simulation-decisions.md` («Выполнено»);
   - перед переходом к перезаписи эталонов прогнать приёмочные тесты старта и езды (`reference.rider_launch_rides_{careful,normal,aggressive}`, `reference.realistic_pedelec_acceptance`, `reference.crank_reposition`, 60-м смешанный заезд `pneumatic`) и убедиться, что ни один приёмочный порог не нарушен. Если какой-либо приёмочный сценарий покраснеет из-за новой позы 30/30 % или потолка тормоза 200 Н·м — остановить конвейер и показать замеры пользователю (согласно п. 34 `simulation-decisions.md`).

---

### Волна 5 — Перенос Ghost Replay (`BGTRAJ01`) без GPU-зависимостей

- Перенести из ветки `feat/sp2-ghost-replay` (коммиты `a9e8bc1f` и `a886f91b`) исключительно C++-подсистему воспроизведения внешних траекторий `BGTRAJ01` и режима призрака во вьюере:
  - `native/cmake/RecordingApplication.cmake`,
  - `native/src/replay/{trajectory,golden_overlay}.{hpp,cpp}`, `native/src/replay/playback.{hpp,cpp}`,
  - `native/src/cli/{argument_contracts.inc,replay.cpp}`,
  - `native/src/viewer/{decorations.hpp,render_owner.hpp,render_owner.cpp,ride.cpp,session.hpp}`,
  - C++-тесты `native/tests/replay/{CMakeLists.txt,test_trajectory.cpp,test_golden_overlay.cpp}` и `native/tests/viewer/test_render.cpp`;
- не включать любые файлы `tools/colab_gpu/**` и не подтягивать коммиты `SP1`.

---

### Волна 6 — Единая перезапись эталонов, финальный гейт `full`, слияние в `main` и `git push`

1. **Пересчёт зафиксированных поз и `darwin-arm64` эталонов:**
   - пересчитать `tests/fixtures/config/poses/savage_pneumatic.npz` (и любые другие pinned-позы, затронутые дефолтами 30/30 % и тормозом) штатным `bike-settle`;
   - перегенерировать изменившиеся эталонные записи `tests/fixtures/golden/darwin-arm64/**` и эпизодные/конфигурационные фикстуры, проверяя дифф каждой группы (отсутствие посторонних мутаций схемы/метаданных).
2. **Финальная квалификация (Level 3 — `Extended`):**
   - убедиться, что `git status --porcelain` в `.worktrees/surface-stage-a-2026-11-18` полностью пуст;
   - запустить `bash tools/run_tests.sh full` (полная сборка, все статические проверки `native`, обычные тесты CTest, `slow`-набор и `package`-гейт) и добиться кода возврата `0` без пропусков и подавлений вне правил;
   - убедиться, что `git status --porcelain` остался пустым после прогона.
3. **Интеграция в `main` и отправка в `origin`:**
   - в корневом чекауте `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean` проверить:
     1) текущая ветка — `main` на `0502660d` (строгий предок проверенного `$VERIFIED_SHA`);
     2) пересечение множества путей `git diff --name-only main..$VERIFIED_SHA` с модифицированными и неотслеживаемыми путями корневого чекаута равно пустому множеству (без `git stash` и без удаления неотслеживаемых файлов корня);
   - выполнить `git -C /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean merge --ff-only "$VERIFIED_SHA"`;
   - выполнить `git -C /Users/vladislav.molchanov/Desktop/tmp/mujoco_clean push origin main` и проверить совпадение `main == origin/main == $VERIFIED_SHA`.
