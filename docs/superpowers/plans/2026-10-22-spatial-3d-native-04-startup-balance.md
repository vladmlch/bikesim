# Spatial 3D Native — 04. Startup и баланс (W-фаза: W29–W36)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Все задачи — **one-shot write** против frozen headers W00 (`00-foundation.md` §W00.4, W00.8, W00.11 `startup_fixture.hpp`, `meijaard_*.hpp`, `benchmark_linalg.hpp`). Запрещены configure/build/compile/ctest/запуск/измерения. Проверка и acceptance — `09-verification.md` V06 (S29–S36, S28).

**Goal:** Написать free-root equilibrium в tangent coordinates, bootstrap hold lifecycle, полный native setup, steer controller, outcome classifier, Meijaard benchmark, cornering/mirror/dt suites и target-ride suite — целиком, без запуска.

**Architecture:** Расширяется существующий `bike::initialization` (`setup_owner.hpp`: `SetupRequest`, `make_setup`, `SetupOwner::seal_bootstrap()`; `equilibrium.hpp`: `settle`, `settle_context`, `initialize_pose`, `refine_equilibrium`). В снимке `refine.cpp:24–43` требует `nq==nv` — W17 уже заменил guard; W29 добавляет spatial ветку. Эталон Whipple отделён от MTB.

**Tech Stack:** C++23, native setup/cache, MuJoCo (`mj_integratePos`, `mj_differentiatePos`), `bike::numerics::bounded_least_squares`/`dense`, control clock, `bt` harness.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md` §§5, 8–11; contracts C5, C7.0.

## Global Constraints

- Planar dispatch и byte contracts сохраняются. `nq≠nv`; free/ball joints по типу.
- Bootstrap phase — явное состояние; после `scored` hold force/work точно отсутствуют; steer actuator — не внешняя опора.
- Числа acceptance — в V06; W пишет их в тесты как есть.
- Порядок общих файлов: `accounting.*` → W30 после W26; `capture.cpp` → W30, W31 после W27; `bootstrap_state.cpp` active-state validation → W30, W32, W33 последовательно.

## Общий формат тестов

`native/tests/spatial/<slug>.cpp`; `support/startup_fixture.hpp` (`RigModel`, `constrained_rig`, `free_rig`, `complete_state_bytes`); регистрация `bike_add_spatial_test(<slug> SOURCES … LIBRARIES bike_initialization bike_runtime bike_spatial bike_pneumatic3d bike_model)`.

---

## W29 — Native equilibrium в tangent coordinates (← S29)

**Dependencies:** W13–W17, W25–W27. **Create:** `native/src/initialization/spatial_equilibrium.cpp`, `native/tests/spatial/equilibrium.cpp`. **Modify:** `native/src/initialization/{context.hpp,equilibrium.hpp,equilibrium.cpp,initial_pose.cpp,refine.cpp,settle_contacts.cpp}`, `native/tests/spatial/support/startup_fixture.cpp` (полная версия: `constrained_rig(cfg)`, `free_rig(cfg, articulated)` строят модель через `BuildRequest` W14 + `Terrain3dSource` плоскости `plane_grid(0,0,0,…)`; `complete_state_bytes` — конкатенация `mjSTATE_INTEGRATION` + все spatial owners через codecs W17/W27).

**Interfaces:** W00.8 `SpatialEquilibriumReport`, `spatial_refine`, `spatial_initial_pose`; `EquilibriumResult` + `std::optional<SpatialEquilibriumReport> spatial`.

- [ ] `equilibrium.hpp`: добавить `std::optional<SpatialEquilibriumReport> spatial;` в `EquilibriumResult`; `settle_context` при `Topology.root.kind==free6` вызывает `spatial_initial_pose` → `spatial_refine`, иначе прежний путь (constrained4 через W17 адресацию).
- [ ] `spatial_initial_pose`: hubs по линии контактов — `z_hub = terrain.sample(x,y).height + outer_radius − sag`, heading через quaternion `Rz(heading)`, pitch из разности высот hubs; тела/суставы райдера сохраняют parent-relative transforms; невалидная геометрия (`outside`, `invalid_input`) → `std::invalid_argument("spatial_initial_pose: terrain sample at wheel <k> is <status>")`.
- [ ] `spatial_refine(ctx, max_eval, tol)`: вектор неизвестных — tangent `dq` размера `nv` по маске разрешённых DOF (free root кроме world x/yaw: маска из `BootstrapConfig`; scalar passive joints; материал/контакт — явно выбранные), каждый кандидат `q_candidate = q0; mj_integratePos(model, q_candidate, dq, 1.0)`; разности через `mj_differentiatePos`; перед оценкой кандидата восстановить drivetrain/tyre/rider/brake/steer states (из захваченного bootstrap snapshot) и engine stage; оценка — probe (`advance=false`); невязка — `qacc` после `mj_forward` + writers probe, нормы раздельно: `max_linear_accel_mps2` по translational DOF, `max_angular_accel_radps2` по rotational; `bounded_least_squares` существующего provider по Jacobian конечных разностей (`h=1e-6` в tangent space); остановка при `max(lin,ang)<=tol` или `max_eval`; возвращает достигнутый max accel; отчёт заполняет `weight_residual_fraction = |Σ Fz − m g|/(m g)`, moment residuals, contacts valid/total, `worst_dofs` (топ-3 по |qacc|).
- [ ] `equilibrium.cpp`: scalar tolerance `.05` сохраняется; spatial report добавляет раздельные единицы.
- [ ] `equilibrium.cpp` тест: constrained/free, rider/no-rider, mixed ball fixture → `|quaternion_norm(qpos,root)−1|<=1e-12`, `result.spatial->max_linear_accel_mps2<=0.05 && max_angular_accel_radps2<=0.05`, `weight_residual_fraction<=0.005`, `|pitch/roll moment|<=1 N·m`; q↔−q начальные quaternions дают равные невязки (1e-12); finite-difference perturbation одного DOF даёт предсказуемое изменение `qacc`; `max_evaluations=1` → не сходится, `errors` непусто, `worst_dofs` непусто, owner не seal (`bt::invalid` на `seal_bootstrap`); порядок probes не меняет `complete_state_bytes` (до/после refine только `qpos` отличается, material bytes равны).
- [ ] Коммит `feat(startup): W29 spatial equilibrium on the configuration manifold`, `Verification: pending V06`.

```cpp
bt::require(std::abs(quaternion_norm(qpos, root)-1.0) <= 1e-12, "refinement preserves a unit root quaternion");
bt::require(result.spatial->max_linear_accel_mps2 <= 0.05 && result.spatial->max_angular_accel_radps2 <= 0.05, "spatial residuals use separate physical units");
```

## W30 — Bootstrap hold, отпускание и старт (← S30)

**Dependencies:** W29, W26. **Create:** `native/src/initialization/bootstrap_hold.cpp`, `native/tests/spatial/bootstrap_hold.cpp`. **Modify:** `native/src/initialization/{context.hpp,equilibrium.cpp,finalize.cpp,capture.cpp}`, `native/src/runtime/accounting.{hpp,cpp}` (ledger `hold_work_j`, `hold_active` sample flag), `native/src/spatial/bootstrap_state.cpp` (active-state validation `HoldState`: `active==true` только при `phase∈{settling,release_check}`; `phase>=armed` требует нулевые last wrench).

**Interfaces:** W00.8 `BootstrapHold`.

- [ ] `try_compute_into(model,data,phase,qfrc)`: `phase>=armed` → ничего не прикладывать, `state_.active=false`, return `ok` (точный ноль, `work_j` не меняется); иначе: `BodyPose` корня; позиционная ошибка — только `y` (lateral), `z`/pitch/sag свободны; угловая ошибка — log-map `Δq = q_ref ⊗ q*` → вектор, из него roll и yaw компоненты (pitch обнулить); сила `F = −k_pos·e_y − c·v_y` по world Y; момент `M = −k_rot·e_rot − c_rot·ω_rot` для roll/yaw; `mj_jac` корня → `qfrc += Jpᵀ F + Jrᵀ M`; `stored_energy = ½k e_y² + ½k_rot|e_rot|²`; `dt` в сигнатуру не входит, поэтому `work_j` интегрирует `accounting.cpp` (W30 правка: `hold_work_j += (F·v + M·ω)·dt` по last wrench из `state()` на каждом accepted шаге); `state_` обновляет last wrench, `active=true`.
- [ ] Автомат `settling → release_check → armed → scored` в `finalize.cpp`/`SetupOwner`: `settling` — существующий dissipative цикл со сбросом скорости; `release_check` — `release_check(report, force_limit, moment_limit)`: `|F|<1 N && |M|<1 N·m && report valid && contacts valid`; `armed` — hold нулевой, scored accounting не начат; `scored` — старт учёта. Timeout `settle_time_s` → `std::invalid_argument("bootstrap: hold not releasable: residual force …")`, не quiet success.
- [ ] Старт в движении: root velocity `v·heading`, wheel spin `ω = v/r_eff` каждому колесу, drivetrain согласован (crank по передаче); initial energy присутствует в state; без толчка/авторазгона.
- [ ] `capture.cpp`: hold state в bootstrap2 `spatial.hold`; restore scored → `phase=scored`, hold не активируется.
- [ ] `bootstrap_hold.cpp` тест: rotated startup (heading 90°) — hold работает в world Y корня, не в body; bad bank (10° при нулевом балансе) → release fails с причиной; nonzero hold at release (искусственный offset) → reject; zero-speed → scored без hold, `std::ranges::all_of(scored.hold_qfrc, ==0.0)`, `scored.hold_work_j == armed.hold_work_j`; moving start — wheel spin соответствует скорости (1e-12); restore scored → `state().active==false`, `try_compute_into` даёт нули.
- [ ] Коммит `feat(startup): W30 explicit spatial hold and release lifecycle`, `Verification: pending V06`.

```cpp
bt::require(std::ranges::all_of(scored.hold_qfrc, [](double x){ return x == 0.0; }), "scored state has no bootstrap force");
bt::require(scored.hold_work_j == armed.hold_work_j, "hold work cannot grow after release");
```

## W31 — Native setup: fresh/cache/import и полнота bootstrap (← S31)

**Dependencies:** W11, W27, W29, W30. **Create:** `native/tests/spatial/startup_paths.cpp`. **Modify:** `native/src/initialization/{setup_owner.hpp,setup_owner.cpp,request.cpp,equilibrium_cache.cpp,artifact_identity.cpp,state_file.cpp,capture.cpp,research_startup.cpp}`, `native/src/runtime/{bootstrap.hpp,bootstrap.cpp,bootstrap_io.cpp}`, `native/src/runtime/capabilities.cpp` (row `startup:spatial_free` → `implemented=true, qualification_pending=true`).

- [ ] `request.cpp`/`read_setup_request`: поля `spatial` (W05), `terrain3d` (W11), относительные пути assets через существующий resolver; fresh-start override не открывает игнорируемый initial-state файл, но проверяет config/terrain identity.
- [ ] `equilibrium_cache.cpp`/`artifact_identity.cpp`: cache key = sha256(canonical resolved config + `spatial_config_fingerprint_hex` + tyre profile digest + `topology.fingerprint_hex` + `terrain.fingerprint_hex` + initial pose/velocity + solver/integrator/dt + rider attachments + bootstrap policy + imu mount + steer profile sha). Runtime seed и generator provenance — отдельные поля metadata.
- [ ] `capture.cpp`: полный inventory C7.1 — integration vector, mutable frictionloss/site inputs, tyre (W27), brake (W26), rider, drive, hold/steer/outcome (W17 codecs; до W32 — `canonical_inactive_steer()` с `controller=off` — настоящий typed state, не null), clocks/phase/energy; `runtime/step.cpp:2745` не стирает spatial latch.
- [ ] `state_file.cpp`/`bootstrap_io.cpp`: identity импортируемого state (schema/tier/model dims/digests) до мутации; planar↔spatial import → `std::invalid_argument("state import: planar bootstrap1 cannot seed spatial runtime")`; повреждённый cache → пометить с причиной и fresh-start policy; mismatch явного initial-state → ошибка, не fallback.
- [ ] `startup_paths.cpp`: fresh settle, cache hit (второй `make_setup` того же request) и explicit captured import → `fresh.physics_state_bytes == cached.physics_state_bytes` (normalized: исключены только поля provenance из явного списка), `imported.continuation_digest == fresh.continuation_digest` после 1000 шагов на том же binary; первая observation/force stage совпадает (сравнить `qfrc` первого шага побитно); матрица: tiers fast/detailed × roots constrained4/free6 × rider rigid/articulated; негативы: planar import в spatial, corrupt cache (bit flip) → причина, explicit state mismatch → `bt::invalid`.
- [ ] Коммит `feat(startup): W31 complete spatial fresh and cached bootstrap`, `Verification: pending V06`.

```cpp
bt::require(fresh.physics_state_bytes == cached.physics_state_bytes, "cache restores the same complete physics state");
bt::require(imported.continuation_digest == fresh.continuation_digest, "explicit native import preserves continuation");
```

## W32 — Внутренний steer controller, ticks, delay и restore (← S32)

**Dependencies:** W14–W19, W25, W30, W31. **Create:** `native/src/rider/steer_balance.cpp`, `native/tests/spatial/steer_controller.cpp`, `native/tests/spatial/data/steer_profiles.json`. **Modify:** `native/src/runtime/{step.hpp,step.cpp,control.cpp,accounting.cpp}`, `configuration/{physical_config.hpp,validate.cpp}` (`steer.profile` path + digest), `native/src/spatial/bootstrap_state.cpp` (active-state validation `SteerState`: `controller!=off` требует `fifo_count<=delay_fifo_nm.size()`, finite values, `profile_sha256` непуст).

**Interfaces:** W00.8 `SteerInput`, `SteerProfile`, `SteerBalance`.

- [ ] `steer_profiles.json`: профиль `roll-heading-v1`: `speed_nodes_mps=[1,3,5,8]`, `gains` 4×6 стартовые (из cold-линеаризации — здесь записываются как `provisional`, числа: при 3 m/s `K=[-40,-8,-6,-2,-1,-0.3]` в N·m на rad/rad·s⁻¹/rad/m/rad/rad·s⁻¹; при 1/5/8 m/s масштаб `×1.6/×0.8/×0.6`), `reference_torque_nm=0`, `cap_nm=40`, `delay_s=0.1`, `sha256_hex:null`, `provenance_status:"provisional"`.
- [ ] `SteerBalance` ctor: `delay_ticks = llround(delay_s/period_s)`; если `|delay_ticks*period − delay| > 1e-9` → `resolved_delay_s = delay_ticks*period` и запись в state metadata; `delay_fifo_nm.assign(delay_ticks+1, 0.)`; `state_.controller=roll_heading`.
- [ ] `tick(input)`: `!enabled` → 0, FIFO продвигается нулём; интерполяция `K(v)` линейно по `speed_nodes` (clamp на краях; `|v|<speed_nodes.front()` → gains первого узла, флаг `low_speed` в state не нужен — поведение задано явно); `x=[roll, roll_rate, heading_error, lateral_error, steer_angle, steer_rate]`; `τ_raw = −K·x + reference`; `τ_cmd = clamp(τ_raw, ±cap)`, `saturated_ticks++` при clamp; push `τ_cmd` в FIFO, pop oldest → `last_applied_nm`; `work_j += last_applied * steer_rate * period` (по реально приложенному); `next_tick++`. Знак момента — по тесту оси W14 (положительный `steer_torque` → положительный `steer_joint` qacc).
- [ ] `step.cpp/control.cpp`: вызов `tick` ровно на integer control tick; `ctrl[actuator]=last_applied`; `accounting.cpp` — `steer_work_j` lane.
- [ ] `restore(state)`: validate (schema, `fifo_count<=size`, finite, `profile_sha256==profile_.sha256_hex`) → copy; `reset()`: FIFO обнуляется, counters/work/next_tick = 0, `controller` и профиль прежние (не `canonical_inactive_steer()`).
- [ ] `steer_controller.cpp`: unit — ноль при нулевом x; знак; задержка: `first_nonzero_tick == input_tick + 20` при 200 Hz/0.1 s; `|applied|<=cap+1e-12`; disable/reset/restore roundtrip (state codec W17); динамика на `free_rig`: v=3/5/8 m/s, roll ±3°, heading ±2° → при 3 m/s `|roll|<0.5°` к 3 s, `|y|<0.3 m`, непрерывное насыщение `<=0.2 s` (40 ticks); continuation после restore mid-episode равна непрерывной (побайтно `SteerState`).
- [ ] Коммит `feat(rider): W32 stateful spatial steering balance on native control ticks`, `Verification: pending V06`.

```cpp
bt::require(first_nonzero_tick == input_tick + 20, "100 ms reaction delay is exactly twenty 200 Hz ticks");
bt::require(std::abs(applied_torque_nm) <= cap_nm+1e-12, "steering cannot exceed the configured rider strength");
```

## W33 — Терминальные исходы, приоритет и достоверные метрики (← S33)

**Dependencies:** W19, W25, W30–W32. **Create:** `native/src/spatial/outcomes.cpp`, `native/tests/spatial/outcomes.cpp`. **Modify:** `native/src/runtime/{status.cpp,research.cpp,research_metrics.hpp,research_metrics.cpp,samples.cpp}`, `native/src/research/{metrics.cpp,evidence.cpp,quality.cpp,exit_codes.cpp}`, `native/src/spatial/bootstrap_state.cpp` (active-state validation `OutcomeState`: `latched.terminal` требует `tick>=0` и `reason!=none`).

**Interfaces:** W00.4 `SpatialOutcomeInput`, `evaluate_outcome`, `Outcome`, `OutcomeState`, `SpatialReason`.

- [ ] `evaluate_outcome(in, state)`: если `state.latched.terminal` → вернуть `state.latched` без изменений (повторный poll не сдвигает tick/counters). Приоритет в одном tick: (1) `!engine_finite || !contact_query_valid` → `numerical_failure`/`terrain_invalid` (`valid=false`); (2) `!inside_terrain_domain` → `terrain_outside` (`valid=false`); (3) `body_ground_contact` или (`attitude.valid && |roll|>=capsize_roll_rad`) → `capsize`; (4) `|lateral_position| + bike_half_width > corridor_half_width` → `off_corridor`; (5) `speed < balance_speed_floor` накопительно `balance_dwell_ticks++`, при `>= balance_dwell_limit_ticks` → `balance_lost`; иначе dwell сбрасывается; (6) `requested_completion` → `requested_completion`. Срабатывание — `latched = {reason, terminal=true, valid, tick, time_s, position, diagnostic}`. `attitude.valid==false` сам по себе не capsize.
- [ ] Wire labels (`to_string` W17): `crash:capsize`, `crash:off_corridor`, `balance_lost`, `invalid:terrain`, `invalid:outside`, `invalid:numerical`, `completed`; существующий planar namespace не переписывается.
- [ ] `research_metrics`/`metrics.cpp`: раздельно `requested_duration_s`, `valid_duration_s`, `completed_distance_m`, `terminal_reason`; invalid не считается успехом в performance/accuracy; planar outcome bytes неизменны (новые ключи только в spatial schema).
- [ ] `outcomes.cpp` тест: boundary fixtures — точное пересечение corridor с учётом `bike_half_width`; roll ровно `capsize_roll_rad` → capsize; budget exhaustion (`contact_query_valid=false`) → `terrain_invalid`, не airborne; приоритет при одновременных условиях; `second_poll.tick == first_poll.tick`; dwell reset при восстановлении скорости; детерминизм двух прогонов одинаковых входов.
- [ ] Коммит `feat(research): W33 spatial outcome classifier`, `Verification: pending V06`.

```cpp
bt::require(outcome.reason == SpatialReason::terrain_invalid, "query exhaustion cannot masquerade as airborne success");
bt::require(second_poll.tick == first_poll.tick, "terminal crossing is latched at the first accepted tick");
```

## W34 — Benchmark M: Meijaard/Whipple (← S34)

**Dependencies:** W13–W15, W19, W29–W31. **Create:** `native/src/model/benchmark_whipple.cpp`, `native/tests/spatial/meijaard.cpp` (label `slow`), `native/tests/spatial/support/{meijaard_constraints.cpp,meijaard_reference.cpp,benchmark_linalg.cpp}`, `examples/benchmarks/meijaard2007.toml`, `docs/research/spatial-meijaard-qualification.md` (методика; результаты `pending V06`). **Modify:** `model/{builder.hpp,builder.cpp}` (benchmark dispatch `BuildRequest::benchmark_whipple` optional).

**Interfaces:** W00.8 `WhippleParameters`, `make_whipple_request`; W00.11 `RollingConstraints`, `knife_edge_constraints`, `WhippleMatrices`, `reference_matrices`, `quadratic_eigenvalues`, `Kkt`, `solve_kkt`.

- [ ] `make_whipple_request`: четыре тела — rear frame+rider (B), front assembly (H), rear wheel (R), front wheel (F); z-down → z-up: `z_up = −z_down`, `I_xz` меняет знак; steer axis по `lambda`, trail `c`, wheelbase `w`; без suspension/tyre compliance/drivetrain/grip/steer damping; knife-edge колёса — тонкие cylinders (визуально), контакты — только через test constraints.
- [ ] `knife_edge_constraints(model,data,topology)`: для каждого колеса — contact point `P = hub − R·n_up`, где `n_up` — проекция world Z на wheel plane; constraints: нормальная, продольная и боковая скорость точки `P` равны нулю → 3 строки на колесо, `A` (6×nv) из `mj_jac` + поправка на вращение; `Adot_v` конечными разностями по `h=1e-7`.
- [ ] `solve_kkt`: `[M,−Aᵀ;A,0][v̇;λ]=[−c;−Ȧv]` через существующий dense provider (LU с pivoting), `relative_residual`, `rank`.
- [ ] `reference_matrices`: Appendix A формулы (M, C1, K0, K2 через `mT, xT, zT, ITxx, ITxz, ITzz, mA, …, μ, SR, SF, ST, SA, …`); `quadratic_eigenvalues`: корни `det(λ²M + λ v C1 + g K0 + v² K2)` — полином 4-й степени, test-only companion-matrix QR или Durand–Kerner на `std::complex<long double>`.
- [ ] `meijaard.cpp`: `WhippleParameters{}` vs `meijaard2007.json` (1e-12); native: steady motion при v∈{2,4,6,8,10}, tangent chart (исключены x/y/yaw/phase), возмущения roll/steer/rates через manifold, проекция допустимых скоростей, reduced 4×4 Jacobian разностями `h, h/2, h/4` (Richardson); собственные значения native vs reference: `|Δλ| <= max(0.03|λ|, 0.05)`; для `|λ|<0.05` дополнительно `|Δλ|<=5e-4` и изменение при h-refinement `<=1e-4`; weave boundary `|v_w − 4.29238253634111|<=0.2`, capsize `|v_c − 6.02426201538837|<=0.2`, корни локализованы бисекцией до `.01 m/s` с проверкой знаков; `kkt.relative_residual <= 1e-10`; малый rollout (0.5 s) подтверждает знак Re(λ) для weave при v=2 (рост) и v=6 (затухание); reference check Table 2 (`2.68234517512745±1.68066296590675i` при v=2; `−1.52644486584142±5.87673060598709i`, `−0.00406690076970` при v=6) с 1e-9.
- [ ] Коммит `test(spatial): W34 native Whipple dynamics vs Meijaard`, `Verification: pending V06`.

```cpp
bt::require(std::abs(native.weave_boundary_mps-4.29238253634111) <= 0.2, "native constrained model matches the weave boundary");
bt::require(kkt.relative_residual <= 1e-10, "benchmark contacts enforce the ideal rolling equations");
```

## W35 — Cornering, mirror invariance и timestep suites (← S35)

**Dependencies:** W28, W31–W34. **Create:** `native/tests/spatial/cornering.cpp`, `native/tests/spatial/mirror.cpp`, `native/tests/spatial/timestep.cpp` (все `slow`), `examples/benchmarks/spatial_cornering.toml` (flat arena 120×120 m, genspec без features), `docs/research/spatial-step-convergence.md` (методика; выбранный dt `pending V06`). **Modify:** `startup_fixture.cpp` (+`arena_rig(radius_m, speed_mps, articulated)`: free rig на плоской арене с начальным steady-cornering состоянием; +`mirror_rig(spec)`), spatial CMake, root CMake.

- [ ] `cornering.cpp`: радиусы 10/20/40 m × v=3/5/8 m/s; комбинации вне `v²/(gR) <= 0.8 μ` исключаются заранее с печатью причины (не `bt::require`); steady reference (тонкие контакты, rigid rider): `|lean_COM − atan(v²/(gR))| <= 1°`, `|ΣF_lat − m v²/R| <= 0.02 m v²/R`; R/v из COM trajectory (окружность МНК); полная MTB: `|ΣF_ext − m a_COM| <= 1e-3 m g`, баланс момента импульса с gyro колёс (`<=1%`); steady criteria: изменчивость yaw rate/roll/load `<1%` за окно 2 s, затем фиксированный интервал 5 s; `mean radius` в 5%, speed 2%, valid contacts, `hold_work_j==0`; недостигнутое steady → тест печатает `inconclusive` и `bt::require(false, …)` с причиной.
- [ ] `mirror.cpp`: `h(x,−y)`, lateral IC, асимметричные features/materials, steer command и оси датчика отражены согласованно → `|Δposition|<=1e-7 m`, `|Δangles|<=1e-7 rad` на гладком 2 s fixture; isolated symmetric kernels (`update_fast_brush`, `query_geometry`) — exact signed bytes (`std::bit_cast`).
- [ ] `timestep.cpp`: fast `dt∈{.00125,.000625,.0003125}`, detailed `{.0005,.00025,.000125}`; одинаковые control times/seeds; 10 s valid episodes; сравнение: события `<=` одного грубого шага, mean/peak load `<=2%/5%`, ориентация `<=0.5°`, energy residual `<=0.5%`; `report.valid_duration_s == report.requested_duration_s`; результаты печатаются JSON для manifest `docs/research/spatial-step-convergence.md`.
- [ ] Коммит `test(spatial): W35 turning, symmetry and timestep suites`, `Verification: pending V06`.

```cpp
bt::require(std::abs(mean_lateral_force_n-mass*speed*speed/radius) <= 0.02*mass*speed*speed/radius, "steady ideal fixture balances lateral acceleration");
bt::require(report.valid_duration_s == report.requested_duration_s, "dt comparison uses complete valid episodes");
```

## W36 — Target rides suite и M3 evidence skeleton (← S36)

**Dependencies:** W06–W35. **Create:** `native/tests/spatial/target_rides.cpp` (`slow`), `examples/research/{spatial_flat.toml,spatial_cross_slope.toml,spatial_rough_uphill.toml,spatial_split_grip.toml}`, `docs/research/spatial-integrated-rides.md` (матрица сценариев F/B/R/G/C, поля evidence `pending V06`). **Modify:** `native/src/runtime/capabilities.cpp` — row `ride:spatial_free` остаётся `implemented=true, qualification_pending=true` (снятие pending — только V06), root/spatial CMake.

- [ ] Низкоуровневый runner в тесте: `NativeRideRuntime` + `PhysicalStep` с заранее заданным demand trace из TOML (`[demand_trace]` массив `(t_s, torque_fraction, brake_fraction)`); без observation V2/research session.
- [ ] Сценарий F: flat, v=3/5/8, roll ±3°, rider on, оба tiers → полный 20 s без capsize/corridor loss, `|y|<=.5 m`, `bootstrap_hold_scored_ticks==0`, `invalid_ticks==0`; контроллер off — отдельный диагностический прогон с печатью времени падения (без `bt::require` на время).
- [ ] Сценарий B: cross-slope ±5/±10° с плавным bank ramp → знаки lateral load/steer, `|y|<=.75 m`, возврат на плоскость; режимы вне μ-domain завершаются физически (`outcome.valid==true`, reason ≠ invalid).
- [ ] Сценарий R: 100 m rough/uphill (`rough_uphill_savage_3d_genspec` seed фиксирован), demand program, baseline rider mass → `completed_distance_m >= requested_distance_m` для qualification profile; импульсы нагрузки, suspension travel, torque caps, rim penetration `==0`, energy ledger closure `<=0.5%`.
- [ ] Сценарий G: split-grip braking в повороте до остановки → общий friction budget (ellipse ≤1 во всех patches), удержание на 5° slope, скольжение на недопустимом, плавное отпускание/повторный старт. Сценарий C: capture/restore при повороте+тормозе+patch split → продолжение на том же binary побайтно равно непрерывному.
- [ ] `ride.profile_digest == manifest.profile_digest` для каждого сценария; результаты печатаются JSON (RTF здесь не измеряется).
- [ ] Коммит `test(spatial): W36 native end-to-end target ride suite`, `Verification: pending V06`.

```cpp
bt::require(ride.invalid_ticks == 0 && ride.bootstrap_hold_scored_ticks == 0, "target ride remains physical throughout the scored episode");
bt::require(ride.completed_distance_m >= ride.requested_distance_m, "rough-uphill acceptance requires the full requested ride");
bt::require(ride.profile_digest == manifest.profile_digest, "qualification cannot drift to a different tyre/controller profile");
```

## Передача

W31 передаёт startup/полное состояние W42/W43; W32 — steering telemetry/state; W33 — семантику завершения W40/W41/W44. Все численные критерии (0.05 m/s², 1 N/1 N·m release, 20 ticks, Meijaard 3%/0.2 m/s, cornering 1°/2%, dt 2%/5%/0.5°) проверяются только в V06; gains профиля `provisional` до V06.
