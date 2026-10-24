# Spatial 3D Native — 05. Наблюдения, research, запись и replay (W-фаза: W37–W44)

> **Source delivery 2026-10-24:** W37–W44 — `source_written`, V07 —
> `pending_verification`. [Отчёт по патчу](../audits/2026-10-24-spatial-w37-w49-source.md)
> фиксирует реализацию и C10-расхождения. Configure/build/tests/apps не запускались;
> следующие acceptance-checkboxes сохранены открытыми.

> **Для исполнителей:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development (рекомендуется) или superpowers:executing-plans. Все задачи — **one-shot write** против frozen headers W00 (`00-foundation.md` §W00.9, W00.10; блок «Контракты V2» ниже — дословный источник для `runtime/spatial_sensor_pipeline.hpp`). Запрещены configure/build/compile/ctest/запуск/freeze goldens. Проверка и acceptance — `09-verification.md` V07.

**Goal:** Написать пространственные наблюдения V2, native research dispatch/ABI2, truth/metrics, catalog2/container2, полный checkpoint, checked replay4 и golden registry — целиком, без запуска.

**Architecture:** Расширяются существующие `runtime`, `research`, `recording`, `replay`; второй симулятор не создаётся. Planar V1 сохраняет wire-контракт; spatial — V2. Физический шаг остаётся владельцем сил и сенсорной истины.

**Tech Stack:** C++23; MuJoCo/DuckDB из lock; `bt::run/require/invalid`; `bike::io` JSON/codecs.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`; contracts C6, C7, C7.0–C7.2, C8, C9; ADR-0003.

## Общие ограничения

- Не менять `runtime::SensorObservation` V1 (`runtime/sensors.hpp:20–32`, девять чисел + dropout), `BikeObservationV1`, порядок V1 RNG draws, planar V1 сериализацию.
- Контейнер записи, physical samples, observation, policy ABI, checkpoint и replay recipe — независимые версии (таблица C7).
- Запись включается явно, держится в RAM, публикуется при завершении; `--out` не включает запись.
- Неизвестная версия, несовместимая топология, повреждённый asset → ошибка до изменения живого состояния.
- `recording/catalog_model.cpp:27` вызывает overload с навязанными `root_x/root_pitch` (`coordinates.cpp:70–72`) — W41 обязан обойти это через `CatalogVersion::spatial2`.
- `database_validate.cpp:114–150` требует точного множества таблиц/`field_catalog` → container2 отдельная ветка.

## Контракты V2 (источник для W00 `runtime/spatial_sensor_pipeline.hpp`)

```cpp
namespace runtime::spatial {
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>; // row-major sensor <- nominal frame
struct ImuTruth { double time_s = 0.; Vector3 specific_force_sensor_mps2{}; Vector3 angular_rate_sensor_rad_s{}; };
Vector3 rotate_vector(const Matrix3& sensor_from_nominal, const Vector3& value);
struct SensorObservationV2 {
    double time_s = 0., source_time_s = 0.; bool valid = true; std::uint16_t valid_mask = 0x7ffU; std::array<double, 11> values{};
    void validate() const; WireObject as_wire() const; static SensorObservationV2 from_wire(const WireObject&);
};
struct NoiseTapeV2 { std::vector<std::array<double, 11>> noise; std::vector<double> dropout_uniform; void validate() const; };
struct SensorConfigV2 {
    double latency_s = .01, maximum_age_s = .1, dropout_probability = 0.; double sample_period_s = .01, maximum_read_period_s = .02;
    Vector3 acceleration_bias{}, gyro_bias{}; Vector3 acceleration_std{.05, .05, .05}, gyro_std{.002, .002, .002};
    double encoder_std = .01, torque_std = .05; bool imu_enabled = true; void validate() const;
};
struct SensorStateV2 {
    std::deque<SensorObservationV2> queue; std::optional<double> last_time, delivery_time; std::optional<SensorObservationV2> startup;
    std::size_t cursor = 0, samples_attempted = 0, samples_dropped = 0;
};
class SensorPipelineV2 {
public:
    SensorPipelineV2(SensorConfigV2, NoiseTapeV2);
    void reset(const SensorObservationV2&); void import_state(SensorStateV2); void push(const SensorObservationV2&);
    SensorObservationV2 read(double time_s); SensorStateV2 state() const; WireObject state_wire() const;
private:
    SensorConfigV2 config_; NoiseTapeV2 tape_; std::vector<SensorObservationV2> queue_storage_; std::size_t queue_head_ = 0, queue_count_ = 0;
    std::optional<double> last_time_, delivery_time_; std::optional<SensorObservationV2> startup_; std::size_t cursor_ = 0, samples_attempted_ = 0, samples_dropped_ = 0;
};
}
```

`WireObject` — существующий `runtime::WireObject` из `samples.hpp`. Индексы `[acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,front,rear,crank,motor,human]`; wire names `specific_force_sensor_mps2[3]`, `angular_rate_sensor_rad_s[3]`, `front_wheel_rad_s, rear_wheel_rad_s, crank_rad_s, motor_torque_nm, human_torque_nm`, `sensor_schema_version=2`, `valid_mask` (биты вне `0x7ff` → reject; `valid=false` ⇒ mask 0). Ёмкость ring = `ceil((latency_s+maximum_read_period_s)/sample_period_s)+3`.

## Точные изменения SQL для catalog2/container2 (источник для W41/W42)

| Таблица | Изменения |
|---|---|
| `observations` | keys `episode_id BIGINT, observation_id BIGINT, time_s DOUBLE, source_time_s DOUBLE, valid BOOLEAN`; скаляры `front_wheel_rad_s, rear_wheel_rad_s, crank_rad_s, motor_torque_nm, human_torque_nm DOUBLE`; **+** `sensor_schema_version BIGINT`, `valid_mask BIGINT`; V1 `specific_force_body_mps2`/`pitch_rate_up_rad_s` заменены на `specific_force_sensor_mps2 DOUBLE[]`, `angular_rate_sensor_rad_s DOUBLE[]` (длина 3) |
| `channel_spatial` (new) | keys `episode_id BIGINT, interval_id BIGINT`; `root_position_world_m DOUBLE[]`(3), `root_quat_world_from_body_wxyz DOUBLE[]`(4), `root_linear_velocity_world_mps DOUBLE[]`(3), `root_angular_velocity_world_rad_s DOUBLE[]`(3); `attitude_valid BOOLEAN`; `roll_rad, pitch_rad, heading_rad` nullable DOUBLE; `steer_angle_rad, steer_rate_rad_s, steer_torque_nm, steer_work_j, lateral_displacement_m DOUBLE`; `front_contact_valid, rear_contact_valid BOOLEAN`; `front_lateral_slip_mps, rear_lateral_slip_mps, front_normal_load_n, rear_normal_load_n` nullable DOUBLE; `front_support_normal_world, rear_support_normal_world` nullable DOUBLE[](3). Ровно одна строка на физический интервал |
| `assets` | DDL неизменен; именованные `spatial/terrain_source.json`, `spatial/terrain.terrain3d`, `spatial/config.json`, `spatial/calibration.json`, `spatial/checkpoint_initial.json`, `spatial/checkpoint_final.json` |
| `coordinate_catalog`, `alias_catalog` | DDL неизменен; free root → 7 qpos / 6 dof rows; без scalar `root_pitch` alias |
| `recording_meta` | `schema_version=2`, `replay_contract_version=4` для spatial |

Слой запросов отображает `spatial.*` → `channel_spatial`.

## Карта файлов

| Задача | Новые модули | Тест (slug) |
|---|---|---|
| W37 | `runtime/spatial_sensors.cpp` | `sensor_mount` |
| W38 | `runtime/spatial_sensor_pipeline.cpp` | `sensor_pipeline` |
| W39 | `research/policy_v2_host.cpp` | `policy_v2` + plugin `plugins/policy_v2_control.cpp` |
| W40 | `runtime/spatial_truth.cpp` | `truth_metrics` |
| W41 | `resources/recording/catalog-v2.json`, `recording/catalog_v2_data.inc` | `recording_catalog` |
| W42 | `runtime/spatial_checkpoint.cpp` | `checkpoint`, `recording_assets` |
| W43 | `replay/spatial_recipe.cpp` | `checked_replay` |
| W44 | `golden/spatial_registry.cpp` | `golden_registry` |

---

## W37 — Native IMU mount и interval truth (← S37)

**Dependencies:** W13–W17, W14 (`imu_site_spatial`). **Create:** `native/src/runtime/spatial_sensors.cpp`, `native/tests/spatial/sensor_mount.cpp`. **Modify:** `native/src/model/sensors.cpp` (sensors уже создаёт W14; здесь — чтение IDs), `native/src/runtime/step.{hpp,cpp}` (snapshot gyro/accel после constraint solve до endpoint forward, `step.cpp:2153–2155` семантика), `native/src/runtime/presentation.cpp` (чтение не пересчитывает).

**Interfaces:** W00.9 `ImuMountBinding`, `bind_imu_mount`, `read_imu_truth`, `is_rotation_matrix`, `rotate_vector`.

- [ ] `bind_imu_mount`: site `imu_site_spatial`, sensors `imu_acc_spatial`/`imu_gyro_spatial` → `sensor_adr`, проверка `sensor_type==mjSENS_ACCELEROMETER/GYRO`, `sensor_dim==3`, `site_bodyid` принадлежит frame body; `sensor_from_nominal` из `cfg.imu.axis_misalignment_rad` (Rz·Ry·Rx), `is_rotation_matrix` (ортонормальность 1e-9, det +1) иначе throw.
- [ ] `read_imu_truth(model,data,binding,time)`: читает `sensordata[acc_adr..+3]`, `[gyro_adr..+3]` (MuJoCo даёт их в site frame), применяет `rotate_vector(sensor_from_nominal, ·)`; `time_s=time`; `time<0||NaN` → `std::invalid_argument`.
- [ ] `step.cpp`: после `mj_step1`/constraint solve (где уже есть `sensordata` для текущего интервала) и до endpoint forward — копия truth в `PhysicalStep` поле `imu_truth_` для spatial; стадия та же, что у существующих planar sensors; steering/tyre force impulse входит в acceleration-input snapshot `refresh_forward`.
- [ ] `presentation.cpp`: чтение `imu_truth_`/pose многократно без `mj_forward`.
- [ ] `sensor_mount.cpp`: identity/90° mount, неправильная матрица → `bt::invalid`; `rotate_vector(identity,v)==v`; смещённый mount на вращающемся теле → tangential/centripetal (`α×r + ω×(ω×r)`) с 1e-9; наклонённый неподвижный mount → повернутый gravity (1e-12); V1 pitch rate = −V2 gyro_y при одинаковом site/stage (побитно, fixture с mount на `site_BB`); повторное чтение presentation не меняет `complete_state_bytes`; planar sample bytes прежние.
- [ ] CMake: `bike_add_spatial_test(sensor_mount SOURCES sensor_mount.cpp LIBRARIES bike_runtime bike_spatial bike_model)`.
- [ ] Коммит `feat(spatial): W37 native sensor mount and interval truth`, `Verification: pending V07`.

```cpp
const runtime::spatial::Matrix3 identity{1.,0.,0., 0.,1.,0., 0.,0.,1.};
const runtime::spatial::Vector3 v{2.,-3.,9.81};
bt::require(runtime::spatial::rotate_vector(identity, v) == v, "identity mount preserves the declared sensor vector");
```

## W38 — Observation V2, noise11 и состояние задержки/dropout (← S38)

**Dependencies:** W37. **Create:** `native/src/runtime/spatial_sensor_pipeline.cpp`, `native/tests/spatial/sensor_pipeline.cpp`, `tests/fixtures/spatial3d/sensor/{v1_seed0_expected.json,v2_seed0_expected.json}` (V1 — копия существующего planar fixture values; V2 — `expected:null, status:"pending_verification"`). **Modify:** `native/src/research/rider_random.cpp` (+`make_sensor_tape_v2`), `native/src/research/environment.hpp`, `native/src/research/configuration.cpp` (`[sensors_v2]` parse), `native/src/runtime/research_io.{hpp,cpp}` (V2 codec).

**Interfaces:** блок V2; `bike::research::make_sensor_tape_v2(const SensorConfigV2&, std::string_view seed_decimal, std::int64_t count) -> NoiseTapeV2`.

- [ ] `make_sensor_tape_v2`: существующие `SeedSequence/NumpyRng`; stream `"sensor_v2/noise"` — `count×11` стандартных нормальных; stream `"sensor_v2/dropout"` — `count` uniform; V1 seed0 draws не меняются (отдельные streams); алгоритм/ревизия `"sensor-v2-rng/1"` записывается в metadata.
- [ ] `SensorConfigV2::validate`: finite, `dropout∈[0,1]`, `latency,maximum_age>=0`, `sample_period,maximum_read_period>0`, представимость ёмкости. `NoiseTapeV2::validate`: `noise.size()==dropout_uniform.size()`, finite.
- [ ] `SensorPipelineV2`: ctor — `queue_storage_.resize(capacity)`; `reset(obs)` — ровно одна стартовая попытка (cursor advance 1), `startup_=obs` (или `valid=false, mask=0` при dropout/stale); `push(truth_obs)` — `source_time` монотонно, иначе invalid run; `values += bias + std*noise[cursor]`, dropout по `dropout_uniform[cursor] < p`, `cursor++`; `imu_enabled=false` → values[0..6)=0, mask &= ~0x3f; ring overflow → run invalid (`std::runtime_error` на cold границе; внутри tick — флаг); `read(t)` — доставка sample с `source_time + latency <= t`, `t − source_time > maximum_age` → `valid=false, mask=0`; `state()` — snapshot в `SensorStateV2` (deque) вне tick; `import_state` — validate (cursor<=tape, queue sizes, монотонные времена, уникальные timestamps) → copy атомарно.
- [ ] `as_wire/from_wire`: exact keys из таблицы выше; ширина 10/12, unknown key, mask>0x7ff, `source_time>time` → `std::invalid_argument`.
- [ ] `research_io.cpp`: V2 state codec `sensor_v2{schema,config,queue[],last_time,delivery_time,startup,cursor,samples_attempted,samples_dropped}`.
- [ ] `sensor_pipeline.cpp`: V1 seed0 fixture bytes прежние; V2 tape 11 каналов (`tape.noise.front().size()==11`), `dropout_uniform.clear()` → `bt::invalid`; reset одна попытка; delay доставки по `latency`; stale → `valid=false, mask=0`; disabled IMU биты 0–5 очищены, остальные измерены; tape exhaustion → invalid run с причиной; dropout потребляет один cursor; `import_state` с непустой FIFO и задержкой → продолжение равно непрерывному (побайтно `state_wire()`); `state()` не продвигает cursor; codec reject cases.
- [ ] CMake: `bike_add_spatial_test(sensor_pipeline SOURCES sensor_pipeline.cpp LIBRARIES bike_runtime bike_research)`.
- [ ] Коммит `feat(spatial): W38 eleven-channel sensor pipeline`, `Verification: pending V07`.

```cpp
runtime::spatial::NoiseTapeV2 tape;
tape.noise.push_back(std::array<double, 11>{}); tape.dropout_uniform.push_back(0.); tape.validate();
bt::require(tape.noise.front().size() == 11, "V2 has eleven measured channels");
tape.dropout_uniform.clear(); bt::invalid([&] { tape.validate(); });
```

## W39 — Native research dispatch и policy ABI V2 (← S39)

**Dependencies:** W33, W38. **Create:** `native/src/research/policy_v2_host.cpp`, `native/tests/spatial/policy_v2.cpp`, `native/tests/spatial/plugins/policy_v2_control.cpp` (shared library plugin: экспорт `bike_policy_v2`, act = константный demand, проверяет `struct_size`/schema/mask, возвращает error на NaN). **Modify:** `native/src/research/policy.{hpp,cpp}`, `plugin_policy.cpp`, `builtin_policies.cpp`, `policy_session.cpp`, `native/src/runtime/research.{hpp,cpp}`, `research_io.{hpp,cpp}`.

**Interfaces:** W00.9 `policy_v2.h`; `using ResearchObservation = std::variant<SensorObservation, spatial::SensorObservationV2>;` в `runtime/research.hpp`.

- [ ] `research.hpp`: alias `ResearchObservation`; поля transition/history/current observation переведены на alias; V1 codec сохраняет bytes (discriminator `sensor_schema_version` 1/2 — для V1 ключ отсутствует как раньше).
- [ ] `policy.hpp/cpp`: `enum class PolicyAbi { v1, v2 }`; `plugin_policy.cpp` — `dlsym("bike_policy_v2")` при spatial; проверка `struct_size==sizeof(BikePolicyV2)`, `abi_version==2`; отсутствие символа при V2 → `std::invalid_argument("plugin: unsupported sensor schema 2 (bike_policy_v2 missing)")`; V1-only plugin в spatial не проецируется.
- [ ] `builtin_policies.cpp`: встроенные policies получают `act_v2(const BikeObservationV2&)` (motor demand из тех же encoder/torque каналов, индексы 6–10).
- [ ] `policy_session.cpp`: typed `std::visit` на границе pipeline→policy; `NativeResearchRuntime` одна; schema фиксируется при создании session.
- [ ] Plugin видит только observation и demand (нет terrain/force/truth/model/checkpoint): проверить, что `BikeObservationV2` не содержит указателей.
- [ ] `policy_v2.cpp`: `sizeof(BikeObservationV2)`/`std::size(values)==11`; `BIKE_SENSOR_VALUE_COUNT==9U` (V1 intact); negative ABI controls: неверный `struct_size`, `abi_version`, NaN demand → error, exception внутри plugin → захвачен на границе; закрытие модуля через RAII при ошибке создания session; command/applied/observation последовательности research без окна и с pacing при одинаковом seed равны (данные из существующего research runner на `free_rig` W29; wall limit неограничивающий).
- [ ] CMake: `bike_add_spatial_test(policy_v2 SOURCES policy_v2.cpp LIBRARIES bike_research bike_runtime)`; `add_library(bike_spatial_policy_v2_control MODULE plugins/policy_v2_control.cpp)` с зависимостью теста.
- [ ] Коммит `feat(spatial): W39 versioned native policy observations`, `Verification: pending V07`.

```cpp
BikeObservationV2 observation{}; observation.struct_size = sizeof(BikeObservationV2); observation.sensor_schema_version = BIKE_SENSOR_V2_SCHEMA_VERSION;
bt::require(std::size(observation.values) == 11, "ABI2 carries all six IMU axes");
bt::require(BIKE_SENSOR_VALUE_COUNT == 9U, "including ABI2 leaves ABI1 intact");
```

## W40 — Spatial truth, публикация исходов и метрики (← S40)

**Dependencies:** W19, W33, W39. **Create:** `native/src/runtime/spatial_truth.cpp`, `native/tests/spatial/truth_metrics.cpp`. **Modify:** `native/src/runtime/research_metrics.{hpp,cpp}`, `native/src/research/metrics.cpp`, `quality.cpp`, `evidence.cpp`, `native/src/runtime/research.cpp`.

**Interfaces:** W00.9 `integrate_abs_slip`, `SpatialTruth`, `spatial_truth_to_wire`.

- [ ] `integrate_abs_slip(acc, slip, dt)`: finite, `acc>=0`, `dt>0` иначе `std::invalid_argument`; `return acc + std::abs(slip)*dt`.
- [ ] `research_truth` (существующая): убрать `qpos[root_pitch]`, 1D road pitch, root_x speed из spatial ветки; pose/velocity из `body_pose`, `attitude=euler_zyx(q)`, при `!valid` углы отсутствуют (`NULL` в wire), quaternion остаётся; `lateral_displacement_m` в corridor frame; `steer_*` из joint; contacts: `contact_valid[w]` по writer wrench validity, `normal_load_n` = проекция на local terrain normal, `lateral_slip_mps` из `PatchWrench` load-weighted, `slip_path_m` через `integrate_abs_slip`; `front_unloaded = load<0.05 mg_front`, `front_lift = !contact_valid[0]`, `flight = !contact_valid[0] && !contact_valid[1]`; catch-plane не считается опорой.
- [ ] Публикация outcome: читать `OutcomeState.latched` W33 без повторного dwell/priority; metric keys `spatial.terminal_reason`, `spatial.terminal_tick`.
- [ ] `spatial_truth_to_wire`: exact keys по таблице `channel_spatial` (имена столбцов = wire keys).
- [ ] `truth_metrics.cpp`: `integrate_abs_slip(0,−2,.25)==.5`, `dt=0` → `bt::invalid`; синтетические состояния после сдвига/поворота — инварианты; левая/правая симметрия (знак slip/roll); cross-slope и crest; интегралы при разных dt разбиениях (сумма одинакова 1e-12); planar metric dictionaries keys/bytes прежние; terminal reason/tick одинаковы при decimation 1/5/10, capture off/full, разном research polling.
- [ ] CMake: `bike_add_spatial_test(truth_metrics SOURCES truth_metrics.cpp LIBRARIES bike_runtime bike_research bike_spatial)`.
- [ ] Коммит `feat(spatial): W40 native truth and event metrics`, `Verification: pending V07`.

```cpp
bt::require(runtime::spatial::integrate_abs_slip(0., -2., .25) == .5, "slip distance integrates absolute lateral velocity");
bt::invalid([] { (void)runtime::spatial::integrate_abs_slip(0., 1., 0.); });
```

## W41 — Spatial catalog, координаты и source channels (← S41)

**Dependencies:** W37–W40. **Create:** `native/resources/recording/catalog-v2.json`, `native/src/recording/catalog_v2_data.inc` (embedded copy; генерируется вручную в W: `R"json(...)json"` литерал, идентичный файлу; V07 проверяет hash равенство), `native/tests/spatial/recording_catalog.cpp`. **Modify:** `native/src/recording/catalog.{hpp (только реализация объявленных W00 overloads — без новых declarations),cpp}`, `catalog_model.cpp`, `coordinates.cpp`, `schema.cpp`, `encoding.cpp`, `plan.cpp`, `physical_source.cpp`, `research_source.cpp`.

- [ ] `catalog-v2.json`: все таблицы schema1 + изменения таблицы выше; каждое поле `{name,type,unit,frame,phase,source,classification:"measurement"|"truth",nullable,presence}`; `field_catalog` содержит native source path.
- [ ] `build_catalog(model, spatial2)`: читает `jnt_qposadr/jnt_dofadr/jnt_type`; free → 7 qpos / 6 dof rows с именами `root_free.q{w,x,y,z}`, `root_free.{x,y,z}`, `root_free.d{x,y,z,wx,wy,wz}`; без `root_x/root_pitch` aliases; derived `spatial.roll_rad` и т.д. — именованные каналы `channel_spatial`.
- [ ] `schema.cpp/encoding.cpp/plan.cpp`: dispatch по `CatalogVersion` во все DDL/encode/validator paths; старые overloads = `planar1`.
- [ ] `physical_source.cpp/research_source.cpp`: источник `spatial.*` → `channel_spatial` row на interval (из `SpatialTruth` W40), `observations` V2 поля.
- [ ] `recording_catalog.cpp`: `build_catalog(joints{root_free}, {})` → `coordinates().qpos.size()==7`, `dof.size()==6`; настоящий compiled freejoint (`free_rig`) `build_catalog(model, spatial2)` без planar aliases; unknown channel, duplicate key, несовместимая ширина, null vs absent → `bt::invalid`; `schema_catalog(planar1)` сериализация побайтно равна существующему fixture; один spatial interval → ровно одна `channel_spatial` row; embedded `.inc` == файл (сравнение строк).
- [ ] CMake: `bike_add_spatial_test(recording_catalog SOURCES recording_catalog.cpp LIBRARIES bike_recording bike_spatial bike_model)`.
- [ ] Коммит `feat(spatial): W41 versioned native recording catalog`, `Verification: pending V07`.

```cpp
const std::array<bike::recording::JointCoords,1> joints{{ {.name="root_free",.joint_type="free",.qposadr=0,.dofadr=0} }};
const auto control = bike::recording::build_catalog(joints, std::span<const bike::recording::ScalarAlias>{});
bt::require(control.coordinates().qpos.size() == 7, "free positions are not DOFs");
bt::require(control.coordinates().dof.size() == 6, "free velocity width");
```

## W42 — Полное состояние, schema2 container и вложенные assets (← S42)

**Dependencies:** W27, W31, W32, W38, W41. **Create:** `native/src/runtime/spatial_checkpoint.cpp`, `native/tests/spatial/checkpoint.cpp`, `native/tests/spatial/recording_assets.cpp`. **Modify:** `native/src/runtime/bootstrap.{hpp,cpp}`, `runtime.cpp`, `research.cpp`, `native/src/research/metadata.cpp`, `native/src/recording/{contracts.hpp,assets.cpp,database.cpp,database_validate.cpp,reader.cpp,research_publication.cpp,publish.cpp}`.

**Interfaces:** W00.9 `SpatialCheckpoint`, `encode_checkpoint/decode_checkpoint`, `stage_checkpoint/commit_checkpoint`.

- [ ] `SpatialCheckpoint` payload: `bootstrap2` (owner engine/model/tyre/brake/steer/rider/hold/outcome), `research{clock,pending_commands[],actuator_delay,window,ack,metrics_cursors}`, `sensor_v2` (W38 state codec), `policy{resumable:bool,state|null}`, `identity{model_digest,config_sha256,terrain_sha256,calibration_sha256,solver,provider,hold_removed:true}`; ничего не дублируется наверх; reserve capacity/scratch/pointers не входят.
- [ ] `stage_checkpoint`: decode → sizes/hashes против `RuntimeIdentity` → выделить все owners в stage объекте → проверить последнее поле; любая ошибка → throw до commit, runtime неизменён. `commit_checkpoint`: перенос owners в runtime (swap фиксированного размера), stage cache invalidated по existing freshness protocol; scored checkpoint не активирует hold.
- [ ] `assets.cpp`: `spatial/terrain_source.json{kind,sha256,corridor,source_track_toml|null}`, `spatial/terrain.terrain3d` (bytes, только grid), `spatial/config.json`, `spatial/calibration.json`, `spatial/checkpoint_initial.json`, `spatial/checkpoint_final.json`; hashes в `assets.hash`.
- [ ] `database_validate.cpp`: ветка container2 с точным множеством таблиц schema2; schema1 ветка без изменений; версия run фиксируется при создании; `publish.cpp` — финальная exclusive публикация с прежним retry.
- [ ] `checkpoint.cpp`: continuous vs restore при непустых sensor/actuator FIFO, активном brake anchor, ненулевых lateral bristles → весь semantic state и 500 следующих measurement/control rows побайтно равны; `invalid.cursor+=1` → `bt::invalid`, `pipeline.state().cursor==before.cursor`; corrupt terrain sha / несовместимая topology / переставленные/пропущенные поля → reject без изменения `complete_state_bytes`.
- [ ] `recording_assets.cpp`: container2 содержит шесть assets с совпадающими hashes; schema1 reader читает старый fixture как прежде; прерванная публикация → retry без повторного stepping (счётчик шагов неизменен).
- [ ] CMake: `bike_add_spatial_test(checkpoint SOURCES checkpoint.cpp LIBRARIES bike_runtime bike_recording bike_research bike_spatial)`, `bike_add_spatial_test(recording_assets SOURCES recording_assets.cpp LIBRARIES bike_recording bike_runtime)`.
- [ ] Коммит `feat(spatial): W42 complete state and recording assets`, `Verification: pending V07`.

```cpp
runtime::spatial::NoiseTapeV2 tape; tape.noise.resize(2); tape.dropout_uniform.assign(2, .5);
runtime::spatial::SensorPipelineV2 pipeline({}, std::move(tape)); pipeline.reset(runtime::spatial::SensorObservationV2{});
const auto before = pipeline.state(); auto invalid = before; invalid.cursor += 1;
bt::invalid([&] { pipeline.import_state(invalid); });
bt::require(pipeline.state().cursor == before.cursor, "rejected checkpoint does not consume sensor state");
```

## W43 — Checked replay spatial4 (← S43)

**Dependencies:** W42. **Create:** `native/src/replay/spatial_recipe.cpp`, `native/tests/spatial/checked_replay.cpp`. **Modify:** `native/src/replay/{input.cpp,input.hpp,rebuild.cpp,session.cpp}`, `native/src/recording/reader.cpp`, `database_validate.cpp`.

**Interfaces:** W00.10 `SpatialRecipe4`, `read_spatial_recipe/write_spatial_recipe`.

- [ ] `input.cpp`: принимать recipe3 planar и recipe4 spatial; `FixedAssets` path safety; recipe4 выбирает записанный `terrain_source.json` kind → grid bytes (`read_terrain3d`) или extrusion (`ProfileExtrusion3d::create` из сохранённого `track.toml`+corridor), проверяет sha256; config из `spatial/config.json`.
- [ ] `rebuild.cpp:115–123,169–173`: spatial ветка — проверка grid dims/axes/materials, topology fingerprint, state shape; неизвестная версия → reject до runtime.
- [ ] `session.cpp`: comparator прежний — evidence `|a−e|<=max(1e-9,1e-9*max(|a|,|e|))`, state `|a−e|<=1e-9+1e-9|e|`; сравнение полного semantic state по объявленной политике полей (`compare_evidence` на `bike::io::Value`), strings/flags/ints/schema exact; результат содержит `first_path`, `first_step`, schema/version/provenance; execution identity mismatch → `incompatible-execution`, inspection разрешён.
- [ ] `checked_replay.cpp`: `compare_evidence(V{true},V{true},"flag").matched && !compare_evidence(V{false},V{true},"flag").matched`, `first_path=="flag"`; полный spatial episode (из `free_rig` + recording W42 в temp dir) воспроизводится; детерминированные повреждения — одна команда, ось gyro, material bristle, terrain cell, calibration revision → каждое даёт ожидаемый `first_path`; recipe3 fixture неизменна; reset/early stop → новая presentation generation; mismatch не продвигает физическое состояние.
- [ ] CMake: `bike_add_spatial_test(checked_replay SOURCES checked_replay.cpp LIBRARIES bike_replay bike_recording bike_runtime bike_spatial)`.
- [ ] Коммит `feat(spatial): W43 checked native replay contract four`, `Verification: pending V07`.

```cpp
using V = bike::io::Value;
const auto same = bike::replay::compare_evidence(V{true}, V{true}, "flag");
const auto changed = bike::replay::compare_evidence(V{false}, V{true}, "flag");
bt::require(same.matched && !changed.matched, "replay checks nonnumeric evidence");
bt::require(changed.first_path == "flag", "first mismatch preserves field identity");
```

## W44 — Golden registry и manifest validator (← S44)

**Dependencies:** W37–W43. **Create:** `tests/fixtures/spatial3d/manifest.json` (`spatial_manifest_version=1`, `entries[]` для каждого spatial теста с `classification`, `platforms:{macos-arm64:{status:"pending_verification",hashes:null},linux-aarch64:{...}}`), `native/src/golden/spatial_registry.cpp`, `native/tests/spatial/golden_registry.cpp`. **Modify:** `native/src/golden/{episode.cpp,capture.cpp}`, `native/src/cli/recording_golden.cpp` (`bike-recording golden freeze/compare` spatial ветка), `docs/TESTING.md` (раздел spatial registry, labels slow/gui/perf).

**Interfaces:** W00.10 `validate_spatial_manifest`.

- [ ] `validate_spatial_manifest`: root `spatial_manifest_version==1`; `entries` непуст; каждая entry: `test, source_task (W-id), classification ∈ {analytic,physical_reference,deterministic,golden,compatibility}`, `config_sha256,model_sha256,terrain_sha256,calibration_sha256` (null допустим только при `status=="pending_verification"`), `provider`, `schema_versions{sensor,catalog,container,replay,tyre_state,checkpoint}`, `expected_channels[]`; дубли `test` → reject; пустой `executable` → reject.
- [ ] `recording_golden.cpp`: `freeze` пишет platform tree `tests/fixtures/golden/<platform>/spatial/...` только при явном флаге `--accept-reviewed`; `compare` — within-platform bitwise, cross-platform по полям с допусками из manifest.
- [ ] `golden_registry.cpp`: `{"spatial_manifest_version":999}` → `bt::invalid`; `{"spatial_manifest_version":1,"entries":[]}` → `bt::invalid`; подмена hash, missing artifact, неверная schema, несовместимая калибровка, stale source identity → различимые ошибки (`incompatibility` vs `regression` vs `physical_invalid`); валидный manifest с `pending_verification` проходит валидатор структуры, но `compare` возвращает `pending`.
- [ ] CMake: `bike_add_spatial_test(golden_registry SOURCES golden_registry.cpp LIBRARIES bike_golden bike_io)`.
- [ ] Коммит `test(spatial): W44 native validation and golden registry`, `Verification: pending V07`.

```cpp
const auto future = bike::io::parse_json(R"({"spatial_manifest_version":999})");
bt::invalid([&] { bike::golden::validate_spatial_manifest(future); });
const auto empty = bike::io::parse_json(R"({"spatial_manifest_version":1,"entries":[]})");
bt::invalid([&] { bike::golden::validate_spatial_manifest(empty); });
```

## Передача

W37–W40 передают W45/W46 семантику observable/schema и presentation inputs; W41–W43 — W47 read APIs, W48 устанавливаемые assets; W44 — V07/V10 классификацию и правила происхождения. Golden freeze на двух платформах, V1-bytes регрессии и checked replay acceptance выполняются только в V07.
