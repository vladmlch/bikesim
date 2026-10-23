# Пространственная C++-симуляция — 02. Топология, модель и runtime (W-фаза: W13–W19)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Все задачи — **one-shot write** против frozen headers W00 (`00-foundation.md` §W00.1–W00.4, W00.11). Запрещены configure/build/compile/ctest/запуск. Проверка и acceptance — `09-verification.md` V03 (S13–S17) и V05 (S18, S19).

**Goal:** Провести пространственные root/steering/rider DOF через существующие C++ model, force mapping, startup, runtime и state boundaries, сохранив прежний planar путь — всё написано за один проход.

**Architecture:** `bike::spatial::Topology` разрешается из реального `mjModel` на cold boundary; warm code использует готовые addresses/frame transforms. Один `generate_mjcf()` dispatch, один stepping path. Declarations — W00; здесь `.cpp`, тесты, fixtures, CMake.

**Tech Stack:** C++23, MuJoCo locked версии, native configuration/model/initialization/runtime, `bt` harness, `spatial_tests::startup_fixture`.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md` §§5, 8.1; contracts C2, C5, C7.0.

## Global Constraints

- `RootKind::constrained4` = x/y/z/pitch, без roll/yaw и steering; `free6` = free joint и steering. `planar3` остаётся отдельным byte-preserving путём.
- `nq`, `nv`, joint ID, qpos address и dof address — разные величины; free/ball joints обрабатываются по типу.
- Все векторы имеют frame/reference point. Angular velocity через engine accessor/Jacobian, а не копия free `qvel`.
- Минимальное дополнительное движение плеча разрешено spec §8.1 только для хвата при рулении; lateral hip/torso balance controller вне объёма.
- Порядок записи общих файлов — master §5.3: `runtime/step.*` → W15, W17; `physical_topology.cpp` → W14, W15; `builder.*` → W14, затем W11, затем W34 (W11 зависит от W14 и читает его результат).

## Общий формат тестов

`native/tests/spatial/<slug>.cpp` c `bt::run`; MuJoCo-fixtures строятся из MJCF-строк внутри `support/startup_fixture.cpp` (W29 пишет полный файл; W13 создаёт его начальную версию с `mixed_joint_rig()` и малыми rigs — см. ниже), либо через существующий `native/tests/support/model_fixture.hpp`.

---

## W13 — Descriptor топологии и преобразования координат (← S13)

**Dependencies:** W00, W05. **Create:** `native/src/spatial/topology.cpp`, `native/src/spatial/frames.cpp`, `native/tests/spatial/test_topology.cpp`, `native/tests/spatial/test_frames.cpp`, `native/tests/spatial/support/startup_fixture.cpp` (начальная версия: `mixed_joint_rig()`, вспомогательные MJCF-строки `kScalarRootXml`, `kConstrained4Xml`, `kFree6Xml`, `kMixedBallXml`; `constrained_rig/free_rig` дописывает W29 — здесь объявленные в W00.11 функции реализуются как построение MuJoCo модели из этих строк с пустой `Config` по умолчанию). **Modify:** `native/src/model_topology.hpp` (комментарий-ссылка на spatial descriptor; `topology::scalar_joint` guard не удалять), spatial CMake.

**Interfaces:** W00.3 целиком.

- [ ] `bind_topology(model, cfg)`: по `cfg.root` — `planar3`: joints `root_x,root_z,root_pitch` (существующие имена); `constrained4`: `root_x,root_y,root_z,root_pitch` — проверить `jnt_type==mjJNT_SLIDE/HINGE`, оси `{1,0,0},{0,1,0},{0,0,1},{0,1,0}`, общий body; `free6`: joint `root_free` типа `mjJNT_FREE`, `free_qpos_adr=jnt_qposadr`, `free_dof_adr=jnt_dofadr`, проверка `+7<=nq`, `+6<=nv`. Wheels: bodies `front_wheel/rear_wheel` (существующие имена из builder), hub = parent body, spin joint hinge с осью hub-Y. Steer: `steer_joint`/`steer_torque` при наличии, иначе `-1` (для planar3/constrained4 обязателен `-1`). Rider roots: все joints с префиксом `rider_` типа FREE/BALL/SLIDE верхнего уровня (по `body_parentid`). `imu_site_id` по имени `imu_site_spatial` или `-1`. Ошибки — `std::invalid_argument("topology.root_free: expected mjJNT_FREE, got …")`. `model_sha256_hex` — существующий model digest helper; `fingerprint_hex = sha256(canonical{kind, joint names, qpos/dof adr, nq, nv, model digest})`.
- [ ] `body_pose(model,data,root)`: `position=xpos[body]`, `quat=xquat[body]` (world_from_body wxyz), `linear_velocity` — `mj_objectVelocity(model,data,mjOBJ_BODY,body,res,0)` (world frame, body origin), `angular_velocity` — первые три компоненты того же результата (MuJoCo порядок: rot, lin — учесть!). Не читать free `qvel` напрямую.
- [ ] `frames.cpp`: `rotate_body_to_world(q,v) = q v q*`; `rotate_world_to_body` — conjugate; `quat_multiply`, `quat_conjugate`, `quat_normalized`; `geodesic_angle_rad(a,b) = 2*atan2(|vec(d)|, |w(d)|)` при `d=a*conj(b)` — инвариант к знаку; `euler_zyx(q, margin)`: ZYX из матрицы, `valid=false` при `|sin(pitch)| > 1 - margin`; `point_velocity_world = v + ω×(p − o)`; `axis_rate = dot(ω_child − ω_parent, axis)`.
- [ ] `test_topology.cpp`: три fixture + mixed (ball joint и переставленные scalar joints) → адреса из engine arrays (`bt::require(model.jnt_qposadr[root]+7<=model.nq)` и т.д.); negative: wrong type/axis, missing name, actuator на другом joint, модель с теми же `nq/nv`, но другим порядком joints → `bt::invalid` и различные `fingerprint_hex`.
- [ ] `test_frames.cpp`: identity, yaw ±90°, combined roll/yaw: `rotate_body_to_world` vs матрица (1e-14); `rotation_q == rotation_minus_q` (`geodesic_angle_rad(q,-q)==0`, векторы совпадают 1e-14); `rotate_world_to_body(rotate_body_to_world(v))==v` (1e-14); `euler_zyx` в singularity → `valid=false`; `point_velocity_world` на вращающемся теле vs аналитика; `body_pose` на free fixture после `mj_forward` с заданной `qvel` — angular velocity равна заданной (1e-12).
- [ ] CMake: `bike_add_spatial_test(topology SOURCES test_topology.cpp LIBRARIES bike_spatial bike_model)`, `bike_add_spatial_test(frames SOURCES test_frames.cpp LIBRARIES bike_spatial)`; `topology.cpp`, `frames.cpp` → `bike_spatial`.
- [ ] Коммит `feat(spatial): W13 topology descriptor and frame transforms`, `Verification: pending V03`.

```cpp
bt::require(model.nq >= 7 && model.nv >= 6, "free-root fixture has full state");
bt::require(model.jnt_type[root_joint] == mjJNT_FREE, "root must be a free joint");
bt::require(model.jnt_qposadr[root_joint] + 7 <= model.nq, "free qpos span escapes model");
bt::require(model.jnt_dofadr[root_joint] + 6 <= model.nv, "free dof span escapes model");
bt::require(rotation_q == rotation_minus_q, "quaternion sign changed physical rotation");
```

## W14 — Constrained/free builders и физический steering joint (← S14)

**Dependencies:** W13. **Create:** `native/src/model/spatial_topology.cpp`, `native/tests/spatial/test_model.cpp`, fixtures `tests/fixtures/spatial3d/model/planar_mjcf_expected.xml` (копия текущего planar output — получить из существующего planar golden fixture, если он есть в `tests/fixtures/migration`; иначе записать путь к нему в manifest с `pending_verification`). **Modify:** `native/src/model/{builder.hpp,builder.cpp,builder_internal.hpp,physical_topology.cpp,environment.cpp,linkage.cpp,sensors.cpp,terrain.cpp}`, `native/cmake/NativeStartup.cmake` (добавить `spatial_topology.cpp` в source list).

**Interfaces:** `BuildRequest` + `std::optional<bike::spatial::Config> spatial` (additive); `generate_mjcf()` dispatch.

- [ ] `builder.hpp`: добавить `std::optional<bike::spatial::Config> spatial;` в `BuildRequest`; при `nullopt` или `mode==planar` — прежние ветки и validation order без изменений.
- [ ] `spatial_topology.cpp`: `emit_constrained_root(xml, …)` — scalar joints `root_x(slide,1 0 0)`, `root_y(slide,0 1 0)`, `root_z(slide,0 0 1)`, `root_pitch(hinge,0 1 0)` в этом порядке; `emit_free_root` — `<freejoint name="root_free"/>` без world constraints; `emit_steer(xml, cfg.steer)` — hinge `steer_joint` по оси head tube (вектор из существующей геометрии вилки), `damping=cfg.steer.damping_nms_rad`, `range=±cfg.steer.range_deg`, actuator `<motor name="steer_torque" joint="steer_joint" ctrlrange="-cap cap"/>`; `emit_imu_site(xml, cfg.imu)` — `<site name="imu_site_spatial" pos=… quat=…/>` на frame body + `<accelerometer name="imu_acc_spatial" site=…/>`, `<gyro name="imu_gyro_spatial" site=…/>`; старые `site_BB`/frame IMU не трогать.
- [ ] `physical_topology.cpp:22–30`: условие удаления `steer_joint` — только `planar3/constrained4`; `free6` сохраняет joint с реальными inertia/mass front assembly.
- [ ] Collision masks: wheel geoms `contype=2 conaffinity=0` при `pneumatic_3d` (без hfield), frame/rider crash geoms `contype=1 conaffinity=1` с hfield; константы объявить в `builder_internal.hpp`.
- [ ] Free frame fixture сначала без articulated rider: `BuildRequest` с `rider=rigid_benchmark` (существующий lumped вариант) — не новая production capability.
- [ ] `test_model.cpp`: planar request → `planar_xml_before == planar_xml_after` (сравнение с fixture); constrained4 → четыре joints в порядке, `steer_joint` отсутствует; free6 → `mj_name2id(root_free)>=0`, `jnt_type==mjJNT_FREE`, `steer_joint` hinge, `actuator_trnid[2*actuator]==steer`, `ctrlrange` = ±cap; total mass/COM/inertia free vs constrained совпадают (1e-9); gravity/timestep прежние; rejection: `free6` + planar tyre backend → `bt::invalid`; `constrained4` + controller `roll_heading` → `bt::invalid`; masks — `geom_contype` колёс не пересекаются с hfield `conaffinity`.
- [ ] CMake: `bike_add_spatial_test(model SOURCES test_model.cpp LIBRARIES bike_model bike_spatial)`.
- [ ] Коммит `feat(model): W14 constrained/free spatial builders and steering joint`, `Verification: pending V03`.

```cpp
const int root = mj_name2id(&model, mjOBJ_JOINT, "root_free");
const int steer = mj_name2id(&model, mjOBJ_JOINT, "steer_joint");
const int actuator = mj_name2id(&model, mjOBJ_ACTUATOR, "steer_torque");
bt::require(root >= 0 && steer >= 0 && actuator >= 0, "spatial topology incomplete");
bt::require(model.actuator_trnid[2 * actuator] == steer, "steering actuator drives wrong joint");
bt::require(planar_xml_before == planar_xml_after, "planar builder serialization changed");
```

## W15 — Drivetrain, сопротивление и учёт мощности при пространственном root (← S15)

**Dependencies:** W13, W14. **Create:** `native/tests/spatial/test_force_mapping.cpp`, `native/tests/spatial/test_drive.cpp`. **Modify:** `native/src/writers/{drivetrain.cpp,drivetrain.hpp,resistance.cpp,resistance.hpp,suspension.cpp,suspension.hpp,brake.cpp,brake.hpp}`, `native/src/drivetrain/{chain.cpp,chain.hpp,transmission.cpp,transmission.hpp}`, `native/src/model/{physical_topology.cpp,drivetrain.cpp}`, `native/src/runtime/{step.cpp,accounting.cpp}`, `native/src/{config_types.hpp,config_validation.hpp}`.

- [ ] Заменить `nq==nv` guard в затронутых writers на проверку каждого реально используемого scalar joint span через `Topology` (`ScalarJointBinding.qpos_adr/dof_adr`); planar3 путь получает те же адреса из своих трёх joints — численный порядок planar не меняется.
- [ ] Wheel-relative spin: `(ω_wheel − ω_hub)·axle_world` через `axis_rate` (W00.3); bearing/brake/motor torque прикладываются к scalar spin joints (`qfrc[dof_adr] += τ`); contact wrench — `mj_jac(model,data,jacp,jacr,point,body)` → `qfrc += Jpᵀ F + Jrᵀ M`.
- [ ] Geometric transmission: path-length Jacobian по engine velocity coordinates — `dL/dt = Σ (∂L/∂p_i)·J_i v`, где точки `p_i` — реальные site positions; rigid translation/rotation всей установки даёт `|ΔL|<1e-12` (внутренняя геометрия). Связанные force/reaction пары суммируются в нулевую net world wrench.
- [ ] Resistance: wind vector в world, body offset через `rotate_body_to_world`; aerodynamic resistance остаётся выключенным как сейчас; rolling loss — один owner (`tyre`), в resistance для `pneumatic_3d` вклад `0` с явным комментарием-ссылкой на C4.
- [ ] Accounting: spring energy только для scalar spring joints (список из Topology/suspension), quaternion components исключены; прежняя measurement cadence.
- [ ] Spatial drive variants: `ideal`, `geometric`, `elastic` — реализованы; любой другой режим при `mode==spatial3d` → capability rejection с row `drive:<mode>` (запись в `capabilities.cpp` таблице; W05 создала структуру).
- [ ] `test_force_mapping.cpp`: yaw-rotated и free-root с roll fixtures: virtual work `|p_generalized − p_world| <= 1e-10*max(1,|p_world|)` на случайных `qvel`; `|rigid_rotation_chain_length_delta_m| < 1e-12`; internal drive pair → net wrench `<=1e-10`.
- [ ] `test_drive.cpp`: positive/negative spin, reverse, coast, braking; одинаковый local torque при повороте heading на 90° (1e-12); planar regression: `qfrc` planar fixture побитно равен значениям до правки (сохранить snapshot в тесте через существующий planar golden path, если доступен; иначе сравнить две сборки одной функции — старый код оставлен под `#if 0`? Нет: сравнение с `tests/fixtures/migration` planar golden через `bt::golden_f64`).
- [ ] CMake: `bike_add_spatial_test(force_mapping SOURCES test_force_mapping.cpp LIBRARIES bike_writers bike_spatial bike_model)`, `bike_add_spatial_test(drive SOURCES test_drive.cpp LIBRARIES bike_writers bike_spatial bike_model)`.
- [ ] Коммит `feat(writers): W15 spatial force mapping and drivetrain addressing`, `Verification: pending V03`.

```cpp
double p_generalized = std::inner_product(qforce.begin(), qforce.end(), qvel.begin(), 0.);
double p_world = std::inner_product(force.begin(), force.end(), point_velocity.begin(), 0.)
               + std::inner_product(couple.begin(), couple.end(), angular_velocity.begin(), 0.);
bt::require(std::abs(p_generalized - p_world) <= 1e-10 * std::max(1., std::abs(p_world)), "wrench mapping violates virtual work");
bt::require(std::abs(rigid_rotation_chain_length_delta_m) < 1e-12, "global rotation changed internal transmission geometry");
```

## W16 — Пространственный rider, полный Jacobian и хват (← S16)

**Dependencies:** W13–W15. **Create:** `native/src/rider/spatial_support.cpp`, `native/tests/spatial/test_rider.cpp`, `native/tests/spatial/test_grip.cpp`, `examples/research/profiles/spatial-shoulder.json` (`provenance_status:"provisional"`, `swivel_range_rad=0.6`, `swivel_stiffness_nm_per_rad=30`, `swivel_damping_nms_per_rad=3`, `swivel_strength_nm=25`, `sha256_hex:null`). **Modify:** `native/src/model/{articulated_rider.cpp,rider_geometry.cpp}`, `native/src/rider/{support_geometry.cpp,attachment_wrench.cpp,spindle_controller.cpp,state_codec.cpp}`, `native/src/writers/rider_contacts.cpp`, `native/src/configuration/{rider_spec.hpp,physical_config.hpp,validate.cpp}`.

**Interfaces:** W00.8 `ShoulderProfile`, `read_shoulder_profile`, `SpatialSupportFrame`, `support_frame`, `GripRigResult`.

- [ ] `articulated_rider.cpp`: для spatial root pelvis — root topology по `Topology.rider_roots` (free joint pelvis); каждое плечо получает дополнительный hinge `shoulder_swivel_{l,r}` вокруг body-Z с `range=±profile.swivel_range_rad`, stiffness/damping из профиля, сохранив существующие pitch/elbow; planar builder неизменен.
- [ ] Grip: `<equality type="connect" …/>` (positional, без weld orientation) между hand site и grip site; стойкий `solref/solimp` — малый bounded compliance.
- [ ] `support_geometry.cpp`/`attachment_wrench.cpp`: normals/tangents/sole goals и projection через `support_frame(body_pose, normal_body)` (W00.8) — повёрнутый body frame; полные `mj_jac` 3×nv/6×nv для mapping; planar reductions под прежней веткой.
- [ ] Reaction closure: grip/pedal/saddle reaction действует на bike и rider через equality constraints; steering command учитывается один раз — модельные hand efforts не добавляют torque на `steer_joint` поверх actuator (в `spindle_controller.cpp` явный флаг `steer_torque_owner=actuator`).
- [ ] `state_codec.cpp`: новые lanes `shoulder_swivel_{l,r}.{q,qd}` и controller state в существующем contract (versioned extension key `rider.spatial=1`).
- [ ] `physical_config.hpp/validate.cpp`: поле `shoulder_profile` (path) + digest.
- [ ] `test_grip.cpp`: reachability rigs для steer ±5/±15/±30/±60°, crank phases 0/90/180/270°, torso lean 0/±10°, suspension 0/50/100% → для каждого solve (существующий IK/`mj_forward` + equality) измерить `left_gap_m/right_gap_m <= 0.001`, `grip_jacobian_rank == expected_independent_constraints` (6 для двух connect), `steering_mobility > 0` (ненулевой minimal singular value проекции steer DOF на null-space constraints), `|internal_power_residual_w| <= 1e-8*max(1,|drive_power_w|)`.
- [ ] `test_rider.cpp`: mass/inertia budget (сумма сегментов ±1e-9 от planar), planar bytes прежние, roundtrip state codec с новыми lanes, unsupported reachability → `GripRigResult` с gap > 1 mm помечает диапазон `not_qualified` (тест печатает таблицу; `bt::require` только для заявленных ±60° по профилю).
- [ ] CMake: `bike_add_spatial_test(rider SOURCES test_rider.cpp LIBRARIES bike_rider bike_model bike_spatial)`, `bike_add_spatial_test(grip SOURCES test_grip.cpp LIBRARIES bike_rider bike_model bike_spatial)`.
- [ ] Коммит `feat(rider): W16 spatial rider support, shoulder swivel and grip rig`, `Verification: pending V03`.

```cpp
bt::require(left_gap_m <= 0.001 && right_gap_m <= 0.001, "hand leaves its steering grip");
bt::require(grip_jacobian_rank == expected_independent_constraints, "grip constraints are redundant or under-resolved");
bt::require(steering_mobility > 0., "rider constraints locked steering");
```

## W17 — Runtime, bootstrap и state adapters (← S17)

**Dependencies:** W13–W16. **Create:** `native/src/spatial/bootstrap_state.cpp`, `native/tests/spatial/test_runtime_state.cpp`. **Modify:** `native/src/runtime/{config.hpp,bootstrap.hpp,bootstrap.cpp,bootstrap_io.cpp,runtime.hpp,runtime.cpp,project_config.cpp,step.hpp,step.cpp,samples.hpp,samples.cpp,wire.cpp}`, `native/src/initialization/{setup_owner.cpp,capture.cpp,equilibrium.cpp,refine.cpp,finalize.cpp,initial_state.cpp,state_file.cpp}`, `native/src/stepper.{hpp,cpp}`.

**Interfaces:** W00.4 codecs; C7.0 runtime2/bootstrap2/writer3.

- [ ] `bootstrap_state.cpp`: `canonical_inactive_*`, `encode_*/decode_*` — exact keys: hold `{schema,active,reference_position_world_m[3],reference_quat_wxyz[4],last_force_world_n[3],last_couple_world_nm[3],stored_energy_j,work_j,phase,valid_mask}`; steer `{schema,controller,next_tick,delay_fifo_nm[],fifo_head,fifo_count,profile_sha256,last_command_nm,last_applied_nm,saturated_ticks,work_j,valid_mask}`; outcome `{schema,latched{reason,terminal,valid,tick,time_s,position_world_m[3],diagnostic},capsize_dwell_ticks,corridor_dwell_ticks,balance_dwell_ticks,valid_mask}`; brake (W00.4) `{schema,wheel[2]{engaged,latch_angle_rad,unwrapped_angle_rad,stored_energy_j,previous_demand},interval_index,valid_mask}`; decode проверяет finite, unit quaternion (1e-9), `fifo_count<=size`, unknown key → `std::invalid_argument` с путём; `null` вместо раздела → reject.
- [ ] `step.cpp`: заменить `root_x/root_pitch` lookups на `Topology` dispatch: speed = проекция `linear_velocity_world` на declared longitudinal axis (`rotate_body_to_world(q,{1,0,0})` с обнулённой z, нормированный), lateral displacement — `position.y` в corridor frame; planar3 ветка сохраняет прежние адреса/арифметику. Старые world-Y формулы `step.cpp:222–234` остаются только в planar ветке.
- [ ] Snapshot/`RuntimeSnapshot`: массивы по реальным `nq/nv`; model identity по digest; descriptor пересоздаётся из загруженной модели.
- [ ] `bootstrap.cpp/bootstrap_io.cpp`: dispatch по integer version: `(1,1|2,1)` planar как раньше; `(2,3,2)` spatial: root keys bootstrap2 ровно `schema,model_digest,model_dims,integration_state,model_mutable,tire,drive,rider_contacts,rider_controller,rider_intent,signals,runtime,spatial`; `spatial` ровно `spatial_contract,root_kind,topology_fingerprint,terrain_sha256,calibration_sha256,bootstrap_phase,hold,brake,steer,outcome`; `tire` для backend `pneumatic_3d` ровно `pneumatic3d_state,identity,encoding,payload` — decode payload делегируется `pneumatic3d::decode_tyre_state` (W27 пишет; здесь вызов). Любая несовместимая тройка → reject до мутации.
- [ ] `project_config.cpp`: runtime2 — обязательный `spatial` section (`spatial_config_to_wire`); writer3 — `spatial{spatial_contract,root_kind,topology_fingerprint,terrain_sha256,numeric_profile}`.
- [ ] `capture.cpp/finalize.cpp/initial_state.cpp/state_file.cpp`: протянуть новые owners (hold/brake/steer/outcome/tyre) через capture/reset/rollback/probe; `probe` не трогает их state; sealing отвергает incomplete producer state (`std::invalid_argument("bootstrap.spatial.tire: producer state missing")`).
- [ ] `equilibrium.cpp/refine.cpp`: scalar settle/refine адресует constrained4 через descriptor (включая `root_y`); `refine.cpp:24–43` условие `nq==nv` заменяется на проверку, что все optimised DOF — scalar joints descriptor; free-root ветка → `spatial_refine` (W29).
- [ ] `test_runtime_state.cpp`: codecs roundtrip для canonical inactive и произвольных заполненных состояний (exact equality полей); reject на unknown key, non-unit quaternion, `null` раздел, `fifo_count>size`; planar lifecycle control: construct/probe/advance/snapshot/reset/close на planar fixture — `before.model_sha256==after.model_sha256`, `integration_bytes` и `material_bytes` неизменны после probe, `reset_state_bytes==fresh_state_bytes`, rejected restore (`wrong_model`) оставляет `before_restore_state_bytes` неизменным; spatial cold validation: bootstrap2 с перепутанной тройкой версий → `bt::invalid`; mixed dims → reject.
- [ ] CMake: `bike_add_spatial_test(runtime_state SOURCES test_runtime_state.cpp LIBRARIES bike_runtime bike_spatial bike_initialization)`.
- [ ] Коммит `feat(runtime): W17 topology-aware runtime, bootstrap2 and spatial state codecs`, `Verification: pending V03`.

```cpp
bt::require(before.model_sha256 == after.model_sha256, "probe replaced model identity");
bt::require(before.integration_bytes == after.integration_bytes, "probe advanced integration state");
bt::require(reset_state_bytes == fresh_state_bytes, "reset omits causal spatial state");
bt::require(rejected_restore_state_bytes == before_restore_state_bytes, "invalid restore partially mutated runtime");
```

## W18 — Constrained bridge test suite (← S18)

**Dependencies:** W10, W17, W25, W27 (читать writer/codec API; тест пишется против W00 headers). **Create:** `native/tests/spatial/test_planar_bridge.cpp`, `native/tests/spatial/test_planar_bridge_long.cpp` (label `slow`), `tests/fixtures/spatial3d/bridge/manifest.json` (paired fixtures: `flat`, `rough_uphill_savage.toml`, `alpine_rocky.toml`; поля `weight_n:null`, результаты `null`, `status:"pending_verification"`), `docs/superpowers/audits/2026-10-22-spatial-3d-bridge.md` (скелет таблицы per-channel worst cases).

- [ ] Paired fixture builder в тесте: planar pneumatic request и constrained4 spatial `pneumatic_3d` на одной продольной геометрии (`ProfileExtrusion3d` того же track), одинаковые initial physical state, tyre longitudinal law params, dt/control cadence, seed, actuator program (детерминированный demand trace из manifest). Lateral state не NaN; writer настоящий (`Pneumatic3dTireWriter`).
- [ ] Статика/low-speed flat rig: нулевая lateral асимметрия (`|Fy_total|<=1e-9`), сумма нагрузок `== weight_n` (1e-6 от веса), совпадение знаков.
- [ ] Для каждого track (2 s short / 10 s long): собрать retained samples, вычислить `normal_load_rms_error_n`, `distance_error_m`, `torque_impulse_error_nms`, `contact_transition_max_step_delta`, persistence metrics; `bt::require` с порогами из V05-таблицы: load RMS `<= 0.02*weight_n`; distance `<= max(0.01, 0.01*|planar distance|)`; torque impulse `<= max(0.01, 0.01*|planar impulse|)`; transitions `<= 1` step; persistence `<=` одного configured window; `outcome_planar == outcome_spatial`.
- [ ] Интегрированные probe/reset/restore assertions W17 с настоящим writer/codec, включая fresh constrained settle (`settle_context` на constrained4).
- [ ] Тест печатает per-channel worst cases JSON для V05 evidence.
- [ ] CMake: `bike_add_spatial_test(planar_bridge SOURCES test_planar_bridge.cpp LIBRARIES bike_runtime bike_spatial bike_pneumatic3d bike_initialization)`, `bike_add_spatial_test(planar_bridge_long SOURCES test_planar_bridge_long.cpp LIBRARIES … LABELS slow)`.
- [ ] Коммит `test(spatial): W18 constrained planar bridge suite`, `Verification: pending V05`.

## W19 — Интегрированные отрицательные проверки топологии (← S19)

**Dependencies:** W13–W18. **Create:** `native/tests/spatial/test_topology_negatives.cpp`, `docs/superpowers/audits/2026-10-22-spatial-3d-topology.md` (скелет). **Modify:** `migration/spatial3d/upstream-gates.json` (consumers V03/V05 для G1/G2), capability matrix в `capabilities.cpp` (только rows для rejections, без включения).

- [ ] Adversarial matrix в тесте: free root + planar tyre; constrained root + steer controller; wrong root joint kind/axis; duplicate names; actuator bound elsewhere; invalid quaternion в bootstrap2; stale `terrain_sha256`; несовместимые shoulder state/layout; модель с теми же dims, но другим body/joint order. Для каждого: `bt::invalid` + проверка field path в `what()` + отсутствие side effects (`snapshot_before == snapshot_after` байты complete-state encoder, `publication_count_before == publication_count_after`).
- [ ] Rigid-motion covariance: поворот track/model/velocity/forces вокруг world-Z на 90° сохраняет body-frame speed, drive power и load response (1e-9); support normals rotate вместе с geometry; reverse motion; near-singular orientation → `attitude.valid=false` без NaN.
- [ ] Три entrypoints: direct C++ request, native request/TOML decode (W05), saved bootstrap restore — одинаковый reject; replay construction seam как unit на synthetic request (без recording).
- [ ] Release reporting: `!unsupported_report.implemented && !rejections.empty()`; constrained bridge не даёт free-root `implemented`; rigid-rider success не квалифицирует articulated.
- [ ] CMake: `bike_add_spatial_test(topology_negatives SOURCES test_topology_negatives.cpp LIBRARIES bike_runtime bike_spatial bike_model bike_initialization)`.
- [ ] Коммит `test(spatial): W19 integrated topology negatives`, `Verification: pending V05`.

```cpp
bt::invalid([&] { (void)runtime::compose_bootstrap(config, wrong_model, state); });
bt::require(snapshot_before == snapshot_after, "rejected model changed the live owner");
bt::require(!unsupported_report.implemented && !unsupported_report.rejections.empty(), "unsupported topology escaped the capability boundary");
```

## Передача

W29–W36 используют `Topology`, `RootBinding`, bootstrap2 codecs и настоящий tyre writer; W37–W49 — state/model identity. Acceptance (virtual work 1e-10, grip 1 mm, bridge 2%/1%) проверяется только в V03/V05.
