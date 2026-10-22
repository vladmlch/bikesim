# Пространственная C++-симуляция — топология, модель и runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. При отсутствии этих skills сохранять тот же task-by-task протокол с отдельными behavior/native reviews, фактическими gates и evidence.

**Goal:** Провести пространственные root/steering/rider degrees of freedom через существующие C++ model, force mapping, startup, runtime и state boundaries, сохранив прежний planar путь.

**Architecture:** Одна модель и один authoritative stepping path получают проверенный `bike::spatial::Topology`. Он разрешается из реального `mjModel` на cold boundary; warm code использует готовые addresses/frame transforms. Сначала строятся и проверяются constrained/free модели; полноценный constrained run допускается после подключения настоящей 3D-шины, без временного planar stub под именем 3D.

**Tech Stack:** C++23, CMake ≥3.27, MuJoCo текущей locked версии, native configuration/model/initialization/runtime, CTest и `bt` harness; macOS arm64 и Linux aarch64.

**Spec:** `docs/superpowers/specs/2026-10-22-spatial-3d-native-design.md`, особенно §§5 и 8.1; master и `2026-10-22-spatial-3d-native-contracts.md`. Старый P1 используется как inventory planar assumptions, а не как источник Python/stub архитектуры.

## Global Constraints

- В этой подготовительной сессии ничего из проекта не запускается и не компилируется. Команды проверки ниже выполняются только при отдельной разрешённой реализации.
- `RootKind::constrained4` = x/y/z/pitch, без roll/yaw и steering; `free6` = free joint и steering. Старый `planar3` остаётся отдельным byte-preserving путём.
- `nq`, `nv`, joint ID, qpos address и dof address — разные величины. Free joint и ball joint обрабатываются по типу, не по числу предполагаемых roots.
- В C++ остаётся один source of truth stepping/writers. Ни viewer, ни research не интегрируют модель отдельно.
- New source paths — предлагаемые файлы. Existing paths проверены по приложенному текстовому снимку; перед редактированием получить actual checkout base S00.
- Hot interfaces получают prevalidated model/data ownership и scratch; никаких lookup/allocation/filesystem операций внутри realtime участка. `noexcept` применять только после доказательства nonthrowing behavior.
- Все векторы имеют frame/reference point. Angular velocity через engine accessor/Jacobian, а не произвольные три значения free `qvel`.
- Минимальное дополнительное пространственное движение плеча разрешено spec §8.1 исключительно для хвата при рулении; lateral hip/torso balance controller остаётся вне объёма.

## S13 — Descriptor топологии и преобразования координат

**Dependencies:** S02, S04, S05; `G1-core`. **Create:** `native/src/spatial/{topology.hpp,topology.cpp,frames.hpp,frames.cpp}`, `native/tests/spatial/test_topology.cpp`, `native/tests/spatial/test_frames.cpp`. **Modify:** `native/src/spatial/types.hpp`, `native/src/model_topology.hpp`, spatial CMake registration. **Test:** `spatial.topology`, `spatial.frames`; targets `bike_spatial_topology`, `bike_spatial_frames`.

**Consumes:** `Config`, real `mjModel`, named frame/rider/steering joints. **Produces:** `Topology`, `RootBinding`, `BodyPose`; contracts:

```cpp
namespace bike::spatial {
[[nodiscard]] Topology bind_topology(const mjModel&, const Config&);
[[nodiscard]] BodyPose body_pose(
    const mjModel&, const mjData&, const RootBinding&) noexcept;
}
```

`BodyPose` содержит `position_world_m`, `quat_world_from_body_wxyz`, `linear_velocity_world_mps`, `angular_velocity_world_rad_s`; body-frame величины вычисляются явным inverse rotation. Binding проверяет реальные spans и ownership; warm accessor предполагает валидную пару того же owner.

- [ ] Создать три небольших MuJoCo fixture: scalar planar root, constrained4 и free6; дополнительный mixed fixture с ball joint и переставленными scalar joints. Негативные варианты: неверный тип/ось, отсутствующее имя, actuator на другом joint, одинаковые dims при другой модели.
- [ ] На construction разрешать joint/body IDs и отдельные qpos/dof ranges; validate span bounds, axes и ожидаемые ancestor relationships. Не удалять старый `topology::scalar_joint` guard без замены адресного доказательства.
- [ ] Реализовать world pose из model/data frames, body-origin velocity с правильным reference point, quaternion sign-insensitive rotation comparison. Euler angles использовать для derived outputs; singularity возвращает явную validity, а controller использует определённые vectors/quaternions.
- [ ] Проверить identity, yaw ±90°, combined roll/yaw и quaternion q/−q. Раздельно тестировать вращение body vector в world и обратное; free-root qvel не копировать как готовый IMU gyro.
- [ ] Подготовить таблицу всех root/qpos/qvel assumptions в `runtime/step`, `drivetrain`, `rider`, initialization и accounting; каждому назначить S14–S17 owner.
- [ ] Выполнить targeted проверки и старые planar descriptor controls; сохранить compiler/provider identity и review численной неизменности planar fast path.

Для free fixture фактические addresses берутся из engine arrays:

```cpp
bt::require(model.nq >= 7 && model.nv >= 6, "free-root fixture has full state");
bt::require(model.jnt_type[root_joint] == mjJNT_FREE, "root must be a free joint");
bt::require(model.jnt_qposadr[root_joint] + 7 <= model.nq, "free qpos span escapes model");
bt::require(model.jnt_dofadr[root_joint] + 6 <= model.nv, "free dof span escapes model");
bt::require(rotation_q == rotation_minus_q, "quaternion sign changed physical rotation");
```

**Acceptance:** descriptor rejects wrong topology before warm stepping; tests cover mixed nq/nv and signs/frames. **Rollback:** оставить старый descriptor route и отключить новый consumer; не менять serialized root meanings.

## S14 — Constrained/free builders и физический steering joint

**Dependencies:** S13; `G2-startup` для model/compile controls; terrain integration следует S10. **Create:** `native/src/model/spatial_topology.cpp`, `native/tests/spatial/test_model.cpp`. **Modify:** `native/src/model/{builder.hpp,builder.cpp,builder_internal.hpp,physical_topology.cpp,environment.cpp,linkage.cpp,sensors.cpp,terrain.cpp}`, `native/cmake/NativeStartup.cmake`. **Test:** `spatial.model` / `bike_spatial_model`.

**Consumes:** existing `bike::model::BuildRequest`, `Config`, `Topology` и owned geometry/mass inputs. **Produces:** native MJCF для трёх root kinds и steering actuator, model-bound descriptor; это модельный milestone, не разрешение выполнить неподключённую шину.

- [ ] Зафиксировать прежние generated planar XML/model structure fixtures. В `BuildRequest` добавить owned spatial selection согласно canonical contracts; единственный `generate_mjcf()` dispatch сохраняет прежние ветки и validation order для old input.
- [ ] Constrained builder создаёт scalar roots x/y/z/pitch в определённом порядке. Free builder создаёт named `root_free`; не добавлять скрытые world constraints для roll/yaw. Состав rider root изменяет S16, поэтому free frame fixture сначала использует отдельный rigid benchmark fixture без articulated rider; это не новая production capability для ранее запрещённого physical+lumped режима.
- [ ] Изменить unconditional удаление `steer_joint` в `physical_topology.cpp:22–30`: planar/constrained удаляют его как раньше; free сохраняет проверенную ось head tube, реальные inertia/mass, damping, диапазон и bounded torque actuator. Не прикладывать torque напрямую к world yaw координате.
- [ ] Wheel hubs/fork/frame остаются правильными родителями bodies; collision masks исключают double tyre force, но сохраняют предусмотренные frame/rider crash collisions. Смена backend должна менять collision participation согласованно с factory.
- [ ] Старые `site_BB`/frame IMU sensors не менять. Spatial IMU mount/site/sensors создаются отдельно и имеют именованные IDs для следующих задач; пустые observation поля не выдавать.
- [ ] Проверить actuator transmission, total mass/COM/inertia, gravity and timestep, root state dimensions и rejection incompatible input. Проверка actual MuJoCo compile относится к будущей разрешённой реализации, не к текущему статическому плану.

Проверка модели с free root использует реальные IDs:

```cpp
const int root = mj_name2id(&model, mjOBJ_JOINT, "root_free");
const int steer = mj_name2id(&model, mjOBJ_JOINT, "steer_joint");
const int actuator = mj_name2id(&model, mjOBJ_ACTUATOR, "steer_torque");
bt::require(root >= 0 && steer >= 0 && actuator >= 0, "spatial topology incomplete");
bt::require(model.jnt_type[root] == mjJNT_FREE, "wrong root type");
bt::require(model.actuator_trnid[2 * actuator] == steer, "steering actuator drives wrong joint");
bt::require(planar_xml_before == planar_xml_after, "planar builder serialization changed");
```

**Acceptance:** compiled geometry structurally valid, steering joint реально существует, planar model bytes неизменны. **Rollback:** вернуть model dispatch целиком; capability остаётся unavailable, а новые runtime schemas не интерпретируются старым builder.

## S15 — Drivetrain, сопротивление и учёт мощности при пространственном root

**Dependencies:** S13, S14; `G1-core`, `G2-startup`. **Create:** `native/tests/spatial/test_force_mapping.cpp`, `native/tests/spatial/test_drive.cpp`. **Modify:** `native/src/writers/{drivetrain.cpp,drivetrain.hpp,resistance.cpp,resistance.hpp,suspension.cpp,suspension.hpp,brake.cpp,brake.hpp}`, `native/src/drivetrain/{chain.cpp,chain.hpp,transmission.cpp,transmission.hpp}`, `native/src/model/{physical_topology.cpp,drivetrain.cpp}`, `native/src/runtime/{step.cpp,accounting.cpp}`, `native/src/{config_types.hpp,config_validation.hpp}`. **Test:** `spatial.force_mapping`, `spatial.drive`.

**Consumes:** bound Topology, scalar spindle/suspension addresses, wheel/hub world frames. **Produces:** корректные generalized forces и одинаковый power balance при повороте всей установки; явная support matrix transmission variants.

- [ ] Добавить yaw-rotated fixture и free-root fixture с nonzero roll. Проверить positive/negative wheel spin, reverse motion, coast, braking и одинаковый локальный torque при изменении world heading. Старое `nq==nv` заменить доказательством каждого реально используемого scalar joint span, не простым снятием guard.
- [ ] Сохранить wheel-relative spin как `(omega_wheel−omega_hub)·axle_world`. Bearing/brake/motor torque действует на соответствующие scalar joints; contact wrench переводится через полный point/rotation Jacobian с правильной точкой приложения.
- [ ] Для geometric transmission убрать обход всех qpos как scalar joints. Геометрический path-length Jacobian вычислять по engine velocity coordinates; free-body rigid translation/rotation всей установки не меняет длину цепи. Связанные force/reaction пары должны давать нулевую net world wrench для внутреннего привода.
- [ ] Сохранить существующие planar transmission laws и численный порядок. Все требуемые spatial drive variants либо реализованы и проверены, либо отклоняются конкретной capability row; нельзя silently заменить geometric/elastic drive на ideal или пометить S15 полным при необработанном required режиме.
- [ ] В сопротивлении world wind/body offset сделать пространственно корректными. Не включать ранее отключённое aerodynamic сопротивление. Rolling loss закрепить за одним owner — tyre или resistance — чтобы энергия не списывалась дважды.
- [ ] Spring energy суммируется только для scalar spring joints; quaternion components не являются углами пружин. Сверить работы actuator/tyre/internal reaction, не меняя старую measurement cadence.

В virtual-work fixture `qforce`, `qvel`, `force`, `point_velocity`, `couple`, `angular_velocity` получены из одной позиции/скорости:

```cpp
double p_generalized = std::inner_product(qforce.begin(), qforce.end(), qvel.begin(), 0.);
double p_world = std::inner_product(force.begin(), force.end(), point_velocity.begin(), 0.)
               + std::inner_product(couple.begin(), couple.end(), angular_velocity.begin(), 0.);
bt::require(std::abs(p_generalized - p_world) <= 1e-10 * std::max(1., std::abs(p_world)),
            "wrench mapping violates virtual work");
bt::require(std::abs(rigid_rotation_chain_length_delta_m) < 1e-12,
            "global rotation changed internal transmission geometry");
```

**Acceptance:** virtual work, internal reaction, rotated drive and planar regression controls pass. **Rollback:** disable spatial transmission row; revert mapping+accounting together, never conceal failure by zeroing energy channels.

## S16 — Пространственный rider, полный Jacobian и достижимый хват

**Dependencies:** S13–S15; `G2-startup`. **Create:** `native/src/rider/spatial_support.hpp`, `native/src/rider/spatial_support.cpp`, `native/tests/spatial/test_rider.cpp`, `native/tests/spatial/test_grip.cpp`; `examples/research/profiles/spatial-shoulder.json` с явно provisional параметрами. **Modify:** `native/src/model/{articulated_rider.cpp,rider_geometry.cpp}`, `native/src/rider/{support_geometry.cpp,attachment_wrench.cpp,spindle_controller.cpp,state_codec.cpp}`, `native/src/writers/rider_contacts.cpp`, `native/src/configuration/{rider_spec.hpp,physical_config.hpp,validate.cpp}`. **Test:** `spatial.rider`, `spatial.grip`.

**Consumes:** existing segment masses, sagittal joints, saddle/pedal/grip attachments; Topology and model-bound full Jacobians. **Produces:** rider follows spatial frame motion physically, сохраняя sagittal effort; минимальный дополнительный spatial shoulder swivel обеспечивает хват руля. Его torque/range/strength parameters имеют отдельную версию и provenance, не вымышленные измерения.

- [ ] Сначала добавить reachability rigs для steering ±5°, ±15°, ±30°, ±60°, нескольких crank phases, допустимого torso lean и suspension positions. Измерять hand-to-grip distance, Jacobian rank, constraint residual и дополнительный steering lock.
- [ ] Сохранить old planar builder. Для spatial root pelvis использовать предусмотренную root topology и реальные body frames; убрать world-planar ancestor assumption только в новом dispatch. Дополнить каждое плечо минимумом lateral swivel DOF, необходимым для 3D hand position, сохранив existing pitch/elbow joints.
- [ ] Использовать grip как positional connect без weld orientation. Новый shoulder axis/range/strength profile валидируется и записывается в config/model identity. Малый bounded compliance может регуляризовать контакт, но не заменяет геометрически недостижимую руку или незаметно обрезанный steer range.
- [ ] Перевести support normals/tangents/sole goals и attachment-force projection в фактический rotated body frame. Полные 3×nv/6×nv Jacobians использовать для velocity/force mapping; старые planar reductions остаются неизменными.
- [ ] Замкнуть реакцию grip/pedal/saddle на bike и rider. Steering command учитывается один раз: отдельный actuator и modelled hand efforts не должны удваивать один и тот же commanded torque. Не добавлять world anchoring, lateral torso controller или скрытый yaw servo.
- [ ] Сериализовать новые shoulder/controller state и validate restore через существующий state codec contract. Проверить mass/inertia budget, power, reachable limits и planar bytes; unsupported reachability остаётся feature-not-qualified с конкретным диапазоном.

В проверке каждого solved rig pose величины вычисляются из actual site positions и проверенного Jacobian:

```cpp
bt::require(left_gap_m <= 0.001 && right_gap_m <= 0.001, "hand leaves its steering grip");
bt::require(grip_jacobian_rank == expected_independent_constraints,
            "grip constraints are redundant or under-resolved");
bt::require(steering_mobility > 0., "rider constraints locked steering");
bt::require(std::abs(internal_power_residual_w) <= 1e-8 * std::max(1., std::abs(drive_power_w)),
            "rider attachment introduced external work");
```

**Acceptance:** полный заявленный steering envelope проходит reachability/rank/load checks; если ±60° недостижимы, задача не закрывается за счёт скрытого clamp. **Rollback:** выключить пространственный articulated capability целиком; rigid-rider diagnostic model остаётся доступным только в своём declared scope.

## S17 — Runtime, bootstrap и state adapters без scalar-root предположений

**Dependencies:** S13–S16; `G1-core`, `G2-startup`. **Create:** `native/src/spatial/bootstrap_state.hpp`, `native/src/spatial/bootstrap_state.cpp`, `native/tests/spatial/test_runtime_state.cpp`. **Modify:** `native/src/runtime/{config.hpp,bootstrap.hpp,bootstrap.cpp,bootstrap_io.cpp,runtime.hpp,runtime.cpp,project_config.cpp,step.hpp,step.cpp,samples.hpp,samples.cpp,wire.cpp}`, `native/src/initialization/{setup_owner.cpp,capture.cpp,equilibrium.cpp,refine.cpp,finalize.cpp,initial_state.cpp,state_file.cpp}`, `native/src/stepper.{hpp,cpp}`. **Test:** `spatial.runtime_state`.

**Consumes:** owned BuildRequest/SetupRequest, Topology, RuntimeBootstrap; C7 versions runtime2/bootstrap2/writer3. **Produces:** topology-aware adapters одного lifecycle `construct/probe/advance/snapshot/reset/close` и cold codec/identity boundaries. Список причинного состояния полный, но сам spatial writer создаёт S25, его state codec — S27. Адаптация scalar settle к constrained4 входит сюда; free-root equilibrium — S29–S31.

- [ ] В PhysicalStep заменить root_x/root_pitch lookups пространственным descriptor dispatch. Speed для rider/rollback/finish semantics определить как проекцию на declared longitudinal axis, lateral displacement — world/corridor frame; не использовать world vx при yaw 90°.
- [ ] Все массивы адресовать по реальным model nq/nv; snapshot хранит полный integration state. Model identity проверять digest/owner, а не совпадением размеров. Descriptor пересоздаётся из загруженной модели, numeric IDs не становятся переносимой wire identity.
- [ ] Реализовать runtime2/bootstrap2 branches C7 и writer3 dispatch. В `spatial/bootstrap_state` создать declarations/strict codecs hold/steer/outcome по S02/C7: canonical inactive — полноценный typed state, не null. Active behavior подключают S30/S32/S33; до него capability отвергает active request. Tyre decoder производит S27. Проверить schema/dimensions/digests/finite/quaternion/layout до mutation; old runtime1/bootstrap1/writer1 или2 неизменны.
- [ ] Протянуть новые state owners через capture/reset/rollback/probe. `probe` не меняет lateral brush, patch IDs, steering delay history, pending sampling/noise queues или накопленные loss; incomplete producer state запрещает sealing.
- [ ] Адаптировать scalar settle/refine к constrained4 через descriptor, включая root_y. Ранние tests проверяют адресацию/identity/cold validation на model fixtures и реальный unchanged planar lifecycle. Они не создают доступный 3D runtime с отсутствующей шиной. Fresh constrained settle выполняется в S18 после S25/S27.
- [ ] Подготовить integrated spatial reset/probe/restore cases как downstream требования S27/S18; до их появления не регистрировать passing stub или failing незавершённый target. Review complete state inventory вместе с recording/replay owner; сейчас закрыть cold failures и прежние planar reset/close controls.

Следующие assertions применяются к реальному planar control сейчас и к spatial continuation в S27/S18 после появления writer/codec. `before`/`after` — test records из model image и complete-state encoder; это не новые production поля `RuntimeSnapshot`:

```cpp
bt::require(before.model_sha256 == after.model_sha256, "probe replaced model identity");
bt::require(before.integration_bytes == after.integration_bytes, "probe advanced integration state");
bt::require(before.material_bytes == after.material_bytes, "probe advanced material or controller state");
bt::require(reset_state_bytes == fresh_state_bytes, "reset omits causal spatial state");
bt::require(rejected_restore_state_bytes == before_restore_state_bytes,
            "invalid restore partially mutated runtime");
```

**Acceptance:** cold topology/identity/schema tests и реальные planar lifecycle controls проходят; inventory однозначно назначает owners. Spatial continuation закрывается S27/S18, без обратной зависимости S17 от будущего writer. **Rollback:** reject new spatial schema and revert consumers together; не читать новые payloads как старые и не обновлять planar goldens.

## S18 — Constrained bridge с настоящей 3D-шиной

**Dependencies:** S10, S17, S25, S27; `G1-core`, `G2-startup`. **Create:** `native/tests/spatial/test_planar_bridge.cpp`, `tests/fixtures/spatial3d/bridge/manifest.json`, `docs/superpowers/audits/2026-10-22-spatial-3d-bridge.md`. **Modify:** spatial CTest registration, exact capability row для constrained mode. **Test:** `spatial.planar_bridge` (короткие 2 s cases), `spatial.planar_bridge_long` (10 s, label slow).

**Consumes:** byte-preserving planar runtime, native laterally-flat extrusion, integrated `bike::pneumatic3d` writer и его state; один идентичный command sequence. **Produces:** measured bridge evidence на flat, `rough_uphill_savage.toml`, `alpine_rocky.toml` с matched masses/geometry/input clocks.

- [ ] Настроить paired fixtures: обычный planar pneumatic и constrained4 spatial pneumatic_3d на одной продольной геометрии. Выравнять complete initial physical state, longitudinal tyre parameter law, time step/control cadence, seed и actuator program. Не подменять 3D writer planar stub и не заполнять lateral state NaN.
- [ ] Сначала проверить static/low-speed flat rig: нулевую боковую асимметрию, совпадение физических единиц/знаков, корректную сумму нагрузок. Exact pure-law zero tests отделить от finite-tolerance MuJoCo full-system result: другой nv/contact path может изменить numerical order.
- [ ] На каждом track сравнить outcome, validity category, per-wheel load RMS, travelled distance, integral motor torque, contact transition timing и wheelie persistence. Заранее назначенные проектные пороги: load RMS error ≤2% total weight; distance error ≤max(0.01 m,1% |planar distance|); motor torque impulse error ≤max(0.01 N·m·s,1% |planar torque impulse|); transitions ≤1 physics step; persistence metrics ≤одного configured persistence window. Absolute floors устраняют деление на малый/нулевой denominator, не получаются подгонкой после измерения.
- [ ] Выполнить integrated spatial construct/probe/reset/restore assertions S17 с actual writer/codec S25/S27, включая fresh constrained settle. Независимо выполнить strict old planar goldens: bridge tolerance их не заменяет. Чистые math/state degeneracy cases сохраняют exact bits при одинаковом algorithm/order.
- [ ] При failure локализовать geometry, law, state initialisation и solver sensitivity. Если физические постановки различаются, исправить fixture или явно пересмотреть model spec с reviewer; не ослаблять пороги до полученного результата.
- [ ] После разрешённого выполнения сохранить per-channel worst cases, timing/loss plots data и обе execution identities; открыть только verified constrained capability. Free balance остаётся отдельным downstream milestone.

`weight_n` относится ко всему сопоставленному bike+rider, сравниваемые summary values рассчитаны C++ test из retained samples:

```cpp
bt::require(weight_n > 0., "bridge needs a physical mass reference");
bt::require(normal_load_rms_error_n <= 0.02 * weight_n, "longitudinal load bridge failed");
bt::require(contact_transition_max_step_delta <= 1, "contact event timing drifted");
bt::require(outcome_planar == outcome_spatial, "constrained bridge changed terminal outcome");
```

**Acceptance:** все три tracks и unchanged planar gates pass с сохранённым evidence. **Rollback:** снять constrained availability и вернуть интеграционный patch; оставить диагностические comparison data, без изменения baseline эталонов.

## S19 — Интегрированные отрицательные проверки топологии и feature boundary

**Dependencies:** S16, S18; `G1-core`, `G2-startup`. **Create:** `native/tests/spatial/test_topology_negatives.cpp`, `docs/superpowers/audits/2026-10-22-spatial-3d-topology.md`. **Modify:** spatial test registration, `migration/spatial3d/upstream-gates.json`, capability matrix. **Test:** `spatial.topology_negatives`; affected planar/config/model/startup/state suites.

**Consumes:** merged S13–S18 code на одном SHA, model/config/writer/state identities и реальные failed-input fixtures. **Produces:** закрытая topology tranche без скрытого fallback, aliasing и неподдерживаемых combinations; точный список оставшихся free startup/balance/recording gates.

- [ ] Собрать adversarial matrix: free root + planar tyre; constrained root + steer controller; wrong root joint kind/axis; duplicate names; actuator bound elsewhere; invalid quaternion; stale grid digest; несовместимые rider shoulder state/layout; модель с теми же dims, но другим body/joint order.
- [ ] Для каждого отказа проверить конкретный field path и отсутствие side effects: untouched engine owner/live state, model coefficients, material history и output publication. Allocation/engine-fatal probes изолировать subprocess controls; normal invalid input остаётся обычным validation failure.
- [ ] Добавить rigid-motion covariance cases: одинаковый world-Z поворот track/model/velocity/forces сохраняет body-frame speed, drive power и load response; spatial support normals rotate вместе с geometry. Проверить reverse motion и near-singular orientation с explicit validity.
- [ ] Проверить direct C++ request, native request/TOML decode и saved bootstrap restore; CLI command wiring остаётся S45. Вызываемый из replay construction seam проверить как unit на synthetic native request без нового recipe4/recording. Нельзя обойти capability rejection через alternate construction entrypoint; полный public checked replay4 повторяет эти negatives в S43, не создавая обратную зависимость S19→S43.
- [ ] Отдельно проверить release reporting: `implemented` описывает connected code, `qualification_pending` требует evidence. Constrained bridge не даёт automatic free-root qualification, а rigid-rider success не квалифицирует articulated rider.
- [ ] На итоговом integration SHA выполнить focused negative controls и required affected platform/planar gates; оформить список exact commands/exit/binary/provider, known baseline failures и reviewer decision. Не объявлять завершение по одной зелёной worker-ветке до merge.

Реальный reject-path test передаёт неверный bootstrap в staged decoder и сравнивает до/после; local inputs подготовлены из valid control. `snapshot_before`/`snapshot_after` — byte strings complete-state encoder, не попытка вызвать отсутствующий `RuntimeSnapshot::operator==`:

```cpp
bt::invalid([&] { (void)runtime::compose_bootstrap(config, wrong_model, state); });
bt::require(snapshot_before == snapshot_after, "rejected model changed the live owner");
bt::require(publication_count_before == publication_count_after,
            "failed construction published an episode");
bt::require(!unsupported_report.implemented && !unsupported_report.rejections.empty(),
            "unsupported topology escaped the capability boundary");
```

**Acceptance:** каждая заявленная topology имеет positive control и причинный negative test, полная owner/state boundary подтверждена. **Rollback:** отозвать только затронутые capability rows и focused changes; не отключать проверки, не ослаблять physics gates, не удалять доказательства неуспеха.

## Передача следующему исполнителю

S29–S36 используют готовую модель, RootBinding и настоящий tyre writer для free startup, steering и физических benchmarks. S37–S49 используют state/model identity для IMU/research/recording/replay. Наличие этих seams не заменяет их независимые acceptance gates. Все task statuses в данном документе остаются planned до исполнения с разрешённой проверкой.
