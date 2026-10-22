# Пространственная C++-симуляция — Interface Contracts

Дата: 22 октября 2026. Статус: **предлагаемые точные контракты будущей реализации, freeze в S02**. Этот документ не утверждает, что перечисленные новые C++ types/headers уже существуют. Existing seams обозначены отдельно.

Связанные документы: [master](2026-10-22-spatial-3d-native-master.md), [спецификация](../specs/2026-10-22-spatial-3d-native-design.md), phase plans S00–S55. Контрактный change log фиксирует owner, producer, consumers, schema impact и необходимые regression gates.

## C0. Владение и публикация

**Existing producers:** `bike::model::BuildRequest`, `bike::initialization::SetupRequest`, `SetupOwner`, `runtime::RuntimeBootstrap`, `NativeRideRuntime`.

Из cold configuration создаются owned immutable config/asset descriptors, затем owned model/setup и runtime. Никакие `mjModel*`, `mjData*`, spans в пользовательские buffers или caller-owned JSON не переживают construction без явно указанного owner.

`Grid3d` неизменяем после успешного construction. Допускается `std::shared_ptr<const Grid3d>` между runtime и render projection, но change of pressure/material/config выполняется как проверенная операция владельца и создаёт новую identity где это влияет на физику. В рабочем шаге не меняется refcount только ради запроса одного элемента.

State import: parse/validate/stage → commit. Любая ошибка до commit оставляет runtime пригодным и неизменным. Неудача самого engine после начала необратимой операции следует существующему poison contract, а не имитирует successful rollback.

**Operation classes:**

| Класс | Допустимые операции |
|---|---|
| Cold | Files/JSON/TOML, allocation, validation, exceptions, binding IDs, capacity calculation, model compile |
| Warm physics | Только проверенные buffers/IDs, bounded arithmetic, no allocation/IO; error status для численных failures |
| Commit | Публикация целого согласованного шага/состояния, integer counters, energy, outputs |
| Observation | Чтение опубликованного состояния; никакого продвижения bristles/controller/noise |
| Cold checkpoint/export | Owned копия всех required sections; allocation допустима вне warm region |

Не каждая существующая функция runtime уже realtime-safe. S-задачи доказывают это для новых warm paths и нужных consumers, а не объявляют свойство по одному `noexcept`.

## C1. Config и capability

**Producer:** S05. **Consumers:** S06–S55. **New paths:** `native/src/spatial/types.hpp`, `native/src/spatial/config.{hpp,cpp}`.

```cpp
namespace bike::spatial {
enum class Mode : std::uint8_t { planar, spatial3d };
enum class RootKind : std::uint8_t { planar3, constrained4, free6 };
}
```

`Config` является owned typed record; записи для steer/bootstrap/IMU/terrain limits/lateral profile также owned. Точная декларация configuration fields следует S05 и этому домену; `SimulationPhysicsConfig` хранит её как extension. Не создавать второй parser, игнорирующий существующие exact-key checks.

| Public key | Domain / default | Правило |
|---|---|---|
| `spatial` | `planar` / `3d`; absent=`planar` | Unknown string rejected |
| `spatial_3d.root` | `constrained` / `free`; 3D default=`free` | Typed default for absent spatial остаётся `planar3` |
| `corridor_half_width_m` | finite >0; 1.5 | Согласуется с track physical domain |
| `capsize_roll_rad` | finite, `0 < value < pi/2`; default pi/3 | Только free-root outcome policy |
| `steer.range_deg` | finite, `0 < value < 90`; target60 | Qualified range не шире прошедшего grip/geometry rig |
| `steer.damping_nms_rad` | finite >=0; 0.5 | Actuator/control damping не дублирует passive damping |
| `steer.torque_cap_nm` | finite >0 или `strength` | `strength` требует явной записи нужного joint/profile |
| `steer.reaction_delay_s` | finite >=0 | FIFO capacity/tick rounding фиксируются в resolved config |
| `steer.controller` | `off` / `roll_heading` | Off допустим для физических диагностик; не обещает устойчивость |
| Gains/speed schedule | finite; speeds strictly increasing | Defaults только из versioned reviewed profile, не скрытые числа старого плана |
| `bootstrap.settle_time_s` | finite >0; 80 | Стоп по budget и residual, без quiet success на timeout |
| `imu.mount_pos_m` | `site_BB` или 3 finite coordinates | Строка разрешается в body-local pose один раз |
| `imu.mount_rpy_rad` | три finite radians | Конвенция Rz(yaw) Ry(pitch) Rx(roll) |
| `imu.axis_misalignment_rad` | три finite radians | Отдельное sensor-frame отклонение; не второй поворот body |
| `tires.pneumatic_3d.tier` | `fast` / `detailed` | Resolve dimensions once, validate product/capacity |
| `lateral_coefficients` | существующий ID с версией/hash | Область применимости и status обязательны |
| Physical timestep | finite >0 | Поддержанные tier/dt pairs определяются convergence gate |

Поля, перечисленные как defaults, являются **проектными стартовыми значениями**. Они не свидетельствуют о прошедшей динамической квалификации. В spatial checkpoint записывается полностью resolved configuration, включая все значения профиля и algorithm revisions.

Planar compatibility: `spatial` отсутствует → прежние canonical config bytes, model settings, schema1 keys и numerical behavior. Extension serializers не добавляют spatial defaults в старое representation. Explicit planar input может нормализоваться к той же старой canonical форме.

Capability registry содержит mode/root/tyre/rider/transmission/sensor-schema combination; unknown/unsupported комбинация отклоняется до `make_setup`. У experimental/provisional physics profile есть явный статус. Приобретение capability привязано к итоговому integration task и evidence, а не к boolean `supports_3d` в config.

## C2. Topology и frame

**Producer:** S13/S14. **Consumers:** S15–S54. **New paths:** `native/src/spatial/topology.{hpp,cpp}`, `frames.{hpp,cpp}`.

```cpp
namespace bike::spatial {
struct BodyPose {
    std::array<double,3> position_world_m{};
    std::array<double,4> quat_world_from_body_wxyz{1.,0.,0.,0.};
    std::array<double,3> linear_velocity_world_mps{};
    std::array<double,3> angular_velocity_world_rad_s{};
};
Topology bind_topology(const mjModel&, const Config&);
BodyPose body_pose(const mjModel&, const mjData&, const RootBinding&) noexcept;
}
```

`RootBinding` хранит body ID, RootKind, freejoint ID/addresses для free6 или четыре пары qpos/dof addresses scalar joints для constrained4. `planar3` использует существующий путь с тремя scalar joints. Неиспользуемые IDs = -1 и никогда не разыменовываются. Не хранить borrowed spans на stack временного bind. `Topology` дополнительно описывает rider root(s), wheel/hub/spin joints, steering joint/actuator и нужные scalar internal joints; количество выводится из модели.

Preconditions `body_pose`: model/data принадлежат одному context; frame kinematics/velocity stage свежая; RootBinding создан именно для model identity. Cold binding проверяет name/type/body ownership/axes и отсутствие дублирующих aliases. Dimension equality не доказывает model identity.

Quaternion описывает вращение body→world. `v_world = R_world_from_body * v_body`; обратный поворот — transpose unit rotation. Не нормализовать raw engine qpos на каждом чтении ради красивой telemetry: valid state проверяется на import; derived quaternion copy может нормализоваться только в documented numerical profile.

Ориентационная difference для новых физических сравнений использует геодезический angle quaternion, инвариантный к `q`/`-q`. Raw state deterministic tests сохраняют bytes и sign; не переписывать quaternion sign в raw checkpoint. ZYX angles предназначены для отображения; validity flag ограждает singular configurations.

```text
v_point_world = v_origin_world + omega_world cross (point_world - origin_world)
axis_rate = dot(omega_child_world - omega_parent_world, axis_world)
```

Тесты включают arbitrary yaw+roll+pitch, freejoint с ненулевым base address, ball joint и модели с разным количеством rider roots. Проверка только `nq=nv+2` запрещена.

## C3. Native terrain API и бинарный asset

**Producer:** S06–S12. **Consumers:** S20–S54.

```cpp
namespace bike::terrain {
enum class QueryStatus3d : std::uint8_t {
    ok, miss, outside, invalid_input, budget_exhausted, origin_below_surface
};
struct GridSample3d {
    QueryStatus3d status = QueryStatus3d::invalid_input;
    double height_m = 0.;
    std::array<double,3> normal_world{};
    std::uint32_t surface_id = 0;
};
struct RayHit3d {
    QueryStatus3d status = QueryStatus3d::invalid_input;
    double distance_m = 0.;
    std::array<double,3> point_world_m{}, normal_world{};
    std::uint32_t surface_id = 0;
};
class Grid3d {
public:
    GridSample3d sample(double x_m, double y_m) const noexcept;
    RayHit3d ray_cast(std::array<double,3> origin_m,
                     std::array<double,3> unit_direction,
                     double max_distance_m) const noexcept;
};
std::expected<Grid3d,std::string> read_terrain3d(std::span<const std::byte>);
std::vector<std::byte> write_terrain3d(const Grid3d&);
}
```

В коде class содержит owned elevations/materials/surfaces/metadata и checked accessors размеров; это не opaque пустая реализация. Полные private storage и constructors реализуются S06. `terrain::Point` остаётся прежним двумерным типом. В 3D APIs используются именованные трёхмерные arrays, не изменение global alias.

`sample` возвращает `ok/outside/invalid_input`. `ray_cast`: ближайший допустимый hit при `ok`; `miss` — отсутствие пересечения в проверенном интервале; `outside` — луч не пересекает domain; `origin_below_surface` — начало уже под поверхностью; `budget_exhausted` — результат не установлен. Нормаль при `ok` единичная, z>0. Потребитель читает payload только при соответствующем status; no hit не определяется по нулевой силе или NaN.

Направление единичное в пределах выбранного numerical tolerance, max distance конечна и неотрицательна. Cold ray geometry готовится нормированной; hot query дополнительно безопасно отвергает invalid values. Число итераций и scratch capacity ограничены. Тесты касаний/нескольких roots/границ обязательны.

### C3.1 Начальный формат BKT3/v1

Ни один прежний spatial asset не считается поставленным форматом: старый plan описывал его намерение. В S06 реализуется **впервые** следующий канонический layout. Все integers и IEEE-754 scalars little-endian; byte offsets относятся к началу файла.

| Offset | Тип | Смысл |
|---|---|---|
| 0 | 4 bytes | ASCII `BKT3` |
| 4 | u32 | format_version=1 |
| 8 | u32 | header_bytes=112 |
| 12 | u32 | dtype=0 f64; dtype=1 f32 |
| 16 | u64 | nx |
| 24 | u64 | ny |
| 32 | f64 | x0_m |
| 40 | f64 | y0_m |
| 48 | f64 | dx_m |
| 56 | f64 | dy_m |
| 64 | u64 | surface_count |
| 72 | u64 | height_bytes |
| 80 | u64 | material_bytes |
| 88 | u64 | surface_json_bytes |
| 96 | u64 | metadata_json_bytes |
| 104 | u64 | reserved=0 |

Sections following header, in order: elevation array `[ny][nx]`; material IDs `u32[(ny-1)*(nx-1)]`; UTF-8 canonical surface JSON; UTF-8 canonical metadata JSON. No trailing bytes. `height_bytes = nx*ny*(dtype==0?8:4)` и `material_bytes=4*(nx-1)*(ny-1)` проверяются с защитой overflow до вычисления total length.

`nx,ny >= 4` для bicubic Grid3d. Для точной profile extrusion этот формат не используется как замена профильного закона; это отдельный query kind. Surface JSON — array записей с уникальным `id` и существующими surface properties `name,mu_peak,mu_slide,slip_stiffness_per_load,stribeck_speed_mps`. `description` optional. Нельзя добавить выдуманный существующий `tread_loss_crr` в старый SurfaceSpec; rolling-loss parameters принадлежат выбранному tyre profile/extension с явной версией.

Metadata содержит `generator_version`, исходный resolved spec, seed decimal, units=`m`, height datum и происхождение. Canonical JSON использует существующий `bike::io` serializer с заданным порядком ключей. В loader есть явный resource limit, записанный в resolved setup. Принимаемая величина limit может изменяться конфигурацией cold setup; это не влияет на физику успешно загруженного того же asset.

SHA-256 считается от всего файла и хранится внешним asset manifest/episode/cache. Недопустимо считать digest от имени файла или только от высот. Generator v1 пишет dtype0; dtype1 reader не означает разрешение использовать f32 без отдельного physics error study.

### C3.2 Extrusion и render

`ProfileExtrusion3d` — immutable adapter к принятому 1D query. Height/longitudinal contact delegated; surface normal имеет y=0. `y` только проверяет domain. Поведение на edges/vertices сохраняется от старого профиля. Он нужен для строгого сравнения shared path, а не для заявления, что bicubic grid побитово воспроизводит кусочно-линейный профиль.

Общие типы cold owner и warm query contract создаются S10, до S20; их подключение к runtime ownership выполняет S11. Автономные S20 rigs не ждут этого подключения. Здесь `ProfileExtrusion3d` владеет исходными profile/surface данными, а `Terrain3dSource` — одним из двух source kinds; вариант не копирует оба массива:

```cpp
namespace bike::terrain {
enum class Terrain3dKind : std::uint8_t { grid, profile_extrusion };
class ProfileExtrusion3d {
public:
    GridSample3d sample(double x_m, double y_m) const noexcept;
    RayHit3d ray_cast(std::array<double,3> origin_m,
                     std::array<double,3> unit_direction,
                     double max_distance_m) const noexcept;
};
struct Terrain3dQueryView {
    Terrain3dKind kind;
    const void* context;
    GridSample3d (*sample_fn)(const void*,double,double) noexcept;
    RayHit3d (*ray_fn)(const void*,std::array<double,3>,
                      std::array<double,3>,double) noexcept;
    GridSample3d sample(double x_m,double y_m) const noexcept;
    RayHit3d ray_cast(std::array<double,3> origin_m,
                     std::array<double,3> unit_direction,
                     double max_distance_m) const noexcept;
};
class Terrain3dSource {
public:
    explicit Terrain3dSource(Grid3d);
    explicit Terrain3dSource(ProfileExtrusion3d);
    Terrain3dKind kind() const noexcept;
    Terrain3dQueryView query_view() const noexcept;
private:
    std::variant<Grid3d,ProfileExtrusion3d> source_;
};
}
```

Полные storage/constructors вводятся S06/S10, а не заменяются пустыми классами из иллюстрации. Query view создаётся только от fully validated, уже окончательно размещённого immutable owner; `context` и function pointers всегда non-null. После выдачи view owner не перемещается и живёт дольше всех writers. `SetupOwner`/bootstrap удерживает shared ownership на cold boundary, а inner ray loop использует bound view без allocation, lookup или операций shared_ptr refcount. `query_geometry(...,const Terrain3dSource&,...)` из S20 берёт этот view один раз вне element loop; замена scene требует rebuild/rebind, не мутацию grid на лету.

Общий spatial ray в extrusion сохраняет world-distance `t`. Сначала XY slab clipping задаёт допустимый интервал, затем проверяется исходный кусочно-линейный профиль. Если прежний solver требует unit direction в XZ, при `s=hypot(dir.x,dir.z)>0` используется `dir_xz/s`, а его результат переводится `t_world=t_xz/s` с соответствующим origin/interval offset. Чисто боковой луч (`s=0`) рассматривается отдельно: нет деления на ноль, учитываются совпадение с поверхностью при t=0 и границы коридора. На точном planar bridge при `dir.y==0` сохраняется прежний delegated путь и его arithmetic order; дополнительная нормировка ради общего случая не разрушает bitwise контроль.

`outside` означает отсутствие пересечения query interval с XY domain, `miss` — проверенный участок внутри domain без hit. Для origin вне domain статус `origin_below_surface` относится к началу проверяемого отрезка после clipping; для origin внутри — к исходной точке. Правило одинаково для Grid3d и extrusion. S10 fixtures обязаны покрыть tilted/world-Z-rotated rays, outside-entry, pure lateral, конечную ширину, точный planar маршрут и world-distance conversion. Renderer и replay сохраняют `Terrain3dKind`; resampling не меняет canonical physics source.

Render hfield — производная визуализация. Его resolution, datum/origin, orientation и extrema проверяются; physics всегда читает canonical query. Collider road/wheel для custom tyre не должен одновременно добавлять MuJoCo tyre friction/contact. Crash/catch contacts описываются отдельными collision masks и event policy.

## C4. Шина и сила

**Producer:** S20–S28. **Consumers:** topology integration, initialization, samples/state, research, replay/performance.

Namespace kernels — `bike::pneumatic3d`. Writer adapter — `native/src/writers/pneumatic_3d_tire.{hpp,cpp}`, интегрируется в существующий writer lifecycle, не вводит второй stepper.

Обязательные semantics `PatchWrench`:

| Поле | Значение |
|---|---|
| point | World point, относительно которой задан couple |
| force | Полная world сила patch, N |
| couple | Собственный момент **вокруг point**, world N·m; cross(point-origin,F) в него не включён |
| normal_load | Неотрицательная проекция нормальной реакции, N |
| longitudinal/lateral | Компоненты в declared right-handed patch frame |
| slip/camber | Rad/rad/signed dimensions по S20/S23, без скрытой смены degrees |
| surface_id / valid / identity | Материал, активность и стабильный material association |

Force map добавляет `Jp^T F + Jr^T couple` ровно один раз. Реакция ground не добавляется второй раз через default engine tyre collider. Момент на wheel axle вычисляется отдельной projection (C2), а не world-Y component.

Friction ellipse в простом isotropic case: `Fx^2 + Fy^2 <= (mu*Fn)^2`; anisotropic version задаёт обе оси и provenance. Normal + tangential components выводятся в согласованном contact frame. Для `Fn=0` контактные tangential force/couple равны нулю. Brush/carcass может иметь internal relaxed state, но оно не даёт внешней силы в воздухе.

Writers имеют два режима: **evaluate/probe**, читающий committed material state и не продвигающий его; **advance/commit**, выполняющий один physical interval. Ни equilibrium residual evaluation, ни sensor refresh, ни render не должны повторно интегрировать material state.

### C4.1 Numerical profile и state

Geometry tier dimensions: стартовые candidates fast=`64×7`, detailed=`256×15`; final поддержанные параметры подтверждаются S28/S35. Изменение resolution не меняет силу пропорционально числу лучей. Детализация сама по себе не является численным эталоном.

State version `pneumatic3d_state=1` содержит dimensions, profile identity, material arrays, initialized masks, Maxwell/hysteresis/bristle/relaxation state, patch membership/history, advection mapping, rim state, relevant clocks и энергетические accumulators. Неактивные slots finite/canonical. Capacity выбирается до runtime; overflow возвращает invalid diagnostic и не пропускает patch.

`probe(probe(state))` не меняет state; `restore(capture(state)); advance(input)` совпадает с непрерванным `advance(input)` в одной numerical identity. Отдельные тесты проверяют seam wrap, reverse, zero speed, split/merge и air/contact transitions.

## C5. Bootstrap, равновесие и управление

**Producer:** S29–S36 совместно с S17/S27. **Existing paths:** `initialization/equilibrium.*`, `refine.cpp`, `initial_pose.cpp`, `setup_owner.*`, cache/state codecs, `runtime/control` и interval clocks.

Lifecycle: `settling -> release_check -> armed -> scored`. `hold_active` может быть true только в первых двух стадиях. Перед scored очищаются bootstrap-only commands/force buffers; material state и физические преднатяги сохраняются, а не стираются ради прохождения проверки.

Hold energy имеет отдельный bootstrap ledger. При scored `hold_active=false`, hold force/couple точные нули и cumulative scored hold work=0. На `release_check` проверяется residual и величина удерживающего wrench. Длительное удержание не заменяет равновесие; timeout явно неуспешен.

Free-root refinement оптимизирует tangent increments и использует корректную engine quaternion integration/difference. Первоначальный pose и residual имеют одинаковый transform convention; passive scalar joints адресуются независимо от root dimensions. Cache invalidation включает resolved topology/tyre/terrain/solver/mount/steer/numerical versions. `cache_hit` не обход проверки state compatibility.

Steer controller использует discrete tick и bounded FIFO. State содержит current/previous error/filter values, delayed commands, schedule selection, last applied torque, enable/rider state и clock phase. Он является внутренней моделью райдера; его privileged inputs не появляются в policy observation случайно через общий WireObject.

Внутренний момент на steer joint имеет equal-and-opposite reaction через model dynamics. Корректность проверяется virtual work и total system momentum в isolated rig. Не моделировать его произвольной world torque на раму.

## C6. Sensors и ABI

**Producer:** S37–S40. **Canonical declaration source:** блок V2 в phase05. Этот раздел определяет semantic contract и те имена/размеры, которые должны совпадать у всех consumers.

`runtime::spatial::ImuTruth`:

- `time_s` — source physics sample time;
- `specific_force_sensor_mps2[3]`;
- `angular_rate_sensor_rad_s[3]`.

`runtime::spatial::SensorObservationV2`:

- `time_s` — delivery/read time;
- `source_time_s` — captured measurement time;
- `valid` — fresh/non-dropped usable delivery по pipeline rules;
- `valid_mask` — 11 field bits;
- `values[11] = accel XYZ, gyro XYZ, front/rear/crank rad/s, motor/human N·m`;
- wire `sensor_schema_version=2`.

Disabled IMU: первые шесть значений canonical zero, первые шесть valid bits очищены; остальные измерения сохраняются. Policy дополнительно проверяет нужный ей field mask. Отсутствие канала не превращается в физическое измерение нуля. Expired/dropped delivery не помечается valid даже если числа случайно finite.

NoiseTapeV2: vector of array11 + отдельный dropout uniform vector. Оба принадлежат setup/runtime; count/seed/source identity фиксируются. Probe/read никогда не потребляет дополнительный RNG draw. Попытка sample имеет один детерминированный cursor advance даже при dropout; exact ordering записан в V2 algorithm version.

Hot queue — preallocated ring. Размер рассчитывается по latency/sample period/max interval reads с ограничениями config и запасом startup slots. Cold `SensorStateV2` может содержать owned deque/vector для импорта/экспорта; `state()` возвращает snapshot по значению вне realtime области. Нельзя обновлять deque snapshot при каждом tick и заявлять no allocation.

Site pose задаёт frame датчика; misalignment applied до bias/noise. Source stage закреплён S37; valid comparison с прежними `frame_*` допускается только при равных mounting и sampling stage. У mounted sensor с offset учитываются `alpha cross r` и `omega cross (omega cross r)`.

ABI V1: `native/include/bike/policy_v1.h`, unchanged. ABI V2: новый `native/include/bike/policy_v2.h`, exported `bike_policy_v2()`; отдельные version/struct_size/value_count/schema/required-field checks. Prefix-safe layout не выводится из похожих имён struct. Никаких C++ exceptions через C ABI. Поддержка V1 planar и V2 spatial тестируется разными plugin fixtures.

## C7. Независимые версии и полное состояние

**Producer:** S17/S27/S38/S41–S43. **Consumers:** initial state/cache/research/recording/replay/goldens.

| Пространство версий | Старый planar | Новый spatial | Причина изменения |
|---|---|---|---|
| Topology/config extension | Old key set | `spatial_contract=1` | Новые mode/root/frame/config поля |
| Runtime config envelope | `runtime_schema=1` | `runtime_schema=2` | Новый spatial dispatch и geometry/config contracts |
| Writer config | `schema=1` или `schema=2` | `schema=3` | Новый backend, terrain identity и пространственные writers |
| Bootstrap state envelope | `state.schema=1` | `state.schema=2` | Typed tyre state и дополнительные causal owners |
| Terrain binary | Existing planar assets | BKT3/v1 | Первый spatial binary asset |
| Tyre state | Existing planar row | `pneumatic3d_state=1` | Многомерная material history |
| Spatial semantic checkpoint | Не требуется новый wrapper | `spatial_checkpoint=1` | Совокупность spatial owners |
| Sensor observation | 1 / 9 values | 2 / 11 values | Три gyro axes вместо одного rate |
| Policy ABI | 1 | 2 | Собственная C ABI раскладка и schema agreement |
| Recording catalog | 1 | 2 | Новые typed fields/coordinates и alias rules |
| Recording container | 1 | 2 | Нынешние validators требуют exact table/field set |
| Research replay contract | 3 | 4 | Новые assets/state/schema reconstruction |

Все числа в spatial column — **выбранные будущие версии**, не обнаруженная текущая реализация. Container2 выбрана по реальному изменению SQL/catalog contract, независимо от совпадения её номера с observation2. Нельзя просто заменить глобальную константу `schema_version=1` на 2: historical readers и planar writer сохраняют V1 dispatch.

Run при создании фиксирует container/profile family. Смешивание planar и spatial episodes внутри одного нового container не реализуется в v1 пространственного проекта. Смена family создаёт новый run. Reader V1/V2 имеет explicit exact schema для каждой версии, reject unknown mandatory fields/versions, no silent coercion.

### C7.0 Точный dispatch runtime, writer и bootstrap

В снимке `native/src/runtime/bootstrap.cpp:249–258,342–349` проверяет exact root keys и использует одну `kRuntimeSchema=1` для двух разных документов; `native/src/writer_config.hpp:1135–1138` принимает только writer schema1/2. Поэтому добавление spatial полей под прежним номером недопустимо. S05/S17 вводят отдельные константы `kSpatialRuntimeSchema=2`, `kSpatialBootstrapSchema=2`, `kSpatialWriterSchema=3`. Существующая `kRuntimeSchema=1` и planar emitters не меняют значение и байты. Одинаковые числа новых runtime/bootstrap не связывают их дальнейшую эволюцию.

| Документ | Выбранный spatial exact-key контракт | Owner |
|---|---|---|
| Runtime config2 | Прежние обязательные root keys `runtime_schema,timestep_s,control_period_s,control_period_steps,physics_mode,drive_mode,strict,measurement_decimation,record_retention,writer_config,rider_controller,rider_intent,monitors,geometry` плюс обязательный `spatial`; прежний optional `async_telemetry` сохраняется | S05/S17 |
| `config.spatial` | `spatial_contract,mode,root,terrain,steer,bootstrap,imu,profile`; `spatial_contract=1`, `mode="3d"`, остальные разделы — fully resolved typed configuration из C1, с exact nested keys из registry S02 | S05 |
| Writer config3 | Обязательные `schema,spatial`; прежние optional writer sections сохраняют имена `drive,cruise,suspension,brake,resistance,tire,rider_forces,rider_contacts`. `spatial` содержит `spatial_contract,root_kind,topology_fingerprint,terrain_sha256,numeric_profile`. Spatial runtime дополнительно требует присутствия всех sections, которые нужны выбранной capability | S05/S17/S25 |
| Writer `tire` в schema3 | Обязательный `backend="pneumatic_3d"` и typed `pneumatic_3d` section. Dimensions/tier/carcass/brush/profile фиксируются S20–S24; формат определяется своей веткой parser, не старым `tire.pneumatic` reader. Old backend под schema3 в первой версии запрещён | S25 |
| Bootstrap2 root | Ровно `schema,model_digest,model_dims,integration_state,model_mutable,tire,drive,rider_contacts,rider_controller,rider_intent,signals,runtime,spatial` | S17/S27/S31 |
| Bootstrap2 `tire` | Ровно `pneumatic3d_state,identity,encoding,payload`; version1, identity по S27, `encoding="base64"`, canonical padded base64 без whitespace для bytes `encode_tyre_state`. Binary payload также проверяет свой header/layout и размеры; JSON не превращает uint64 IDs в неограниченно точный double | S27 |
| Bootstrap2 `spatial` | Ровно `spatial_contract,root_kind,topology_fingerprint,terrain_sha256,calibration_sha256,bootstrap_phase,hold,brake,steer,outcome`; каждый state section имеет typed/versioned schema и validity mask согласно producer. Root kind и identities совпадают с config/model | S17/S26/S30–S33 |
| Semantic checkpoint1 | Один bootstrap2 как владелец engine/tyre/brake/rider/steer/outcome плюс research/control/sensor/policy continuation sections из C7.1. Один state owner не сериализуется двумя конкурирующими независимыми copies | S42 |

`config.geometry.pose`, `state.runtime`, rider и остальные вложенные sections, которые содержат planar assumptions, получают schema-specific spatial decoder. Это не разрешение добавить произвольные keys: S02 фиксирует nested key sets по именованным producers до их реализации; S17/S27/S31/S42 проверяют точный consumer contract. Numeric joint/body IDs не сохраняются как переносимая topology identity: при загрузке bindings разрешаются из модели заново.

Public decoder сначала выбирает ветку по integer version, затем валидирует полный документ, identities и согласованность тройки runtime/writer/bootstrap. Для первой spatial версии разрешена только тройка `(2,3,2)`; planar остаётся `(1,1|2,1)`. Перемешанные combinations, неизвестные версии, новые keys под старым номером и повреждённый payload получают явный отказ до изменения live owner. Старый validation order внутри planar ветки сохраняется; spatial schema selection не переписывает исторические записи.

Чтобы обязательные state keys не создавали обратную зависимость S17/S18→S30/S32/S33, **declarations и строгие codecs** `hold/steer/outcome` принадлежат S17 по реестру S02 и размещаются в `native/src/spatial/bootstrap_state.{hpp,cpp}`. Поведение этих owners подключается позднее отдельными задачами; owner-specific headers включают общие типы, не объявляют вторые версии. Canonical inactive state: hold отключён, last wrench/storage/work нулевые; steer имеет `controller="off"`, пустую FIFO, нулевые command/work/counters; outcome unlatched, без reason/terminal tick, с нулевыми dwell counters. Schema version и validity у каждого присутствуют даже в этом состоянии. `null` вместо всего обязательного раздела не принимается.

Ранние constrained rigs и diagnostic free startup с реально отключёнными owners могут seal именно такие состояния; feature matrix не разрешает включить ещё отсутствующий hold/controller/classifier. Это не замена отсутствующего spatial writer или материала, не успешный scored free ride и не production fallback. S30 добавляет active hold lifecycle; S32 — active steer state; S33 — actual outcome/dwell logic. После подключения каждого owner повторяются complete-state tests, а S36 требует уже весь активный causal inventory. Полный active-steering probe/reset/restore проверяется после S32 и повторяется в S36/S42/S43. В review должна быть трассировка одного состояния от producer до bootstrap/cache/recording/replay, включая проверку отсутствия потерянных или дублированных owners.

### C7.1 Inventory checkpoint

| Section | Обязательное содержимое | Owner |
|---|---|---|
| Identity | model digest/dims, config/terrain/profile/solver/provider identities | Runtime/setup |
| Engine | `mjSTATE_INTEGRATION` exact width/flags выбранной engine version; необходимые mutable model inputs | Runtime |
| Tyre | Material/patch/advection/rim state, masks, capacities/version | Tyre |
| Brake | Wheel-angle latches, hysteresis phase, hold state и энергия | Runtime/brake |
| Rider | Attachments/intent/muscle/allocation/filter state | Rider |
| Steering | Controller history, cap/saturation, delay FIFO, tick phase | Steer owner |
| Control | Pending commands, actuator delay, driver clocks и counters | Research/runtime |
| Sensor | Queue/startup sample, source/delivery times, cursor/drop counts, valid masks | V2 sensor pipeline |
| Episode | Step/generation/outcomes, energy/statistics and committed publication cursors | Runtime/research |
| Policy | Serializable builtin/plugin state либо explicit non-resumable limitation | Policy host |

Если plugin не предоставляет checkpoint state, checked replay может использовать запись применённых команд при заданной semantics; resumable policy continuation не обещается. Эти режимы имеют разные capabilities и evidence. Нельзя назвать воспроизводимым policy resume только потому, что deterministic plant replay успешен.

State codec отделяет **возврат к исходному bootstrap** (`reset`) от **продолжения checkpoint** (`restore`). Restore не сбрасывает brake/tyre/controller latch. Все arbitrary qpos size assumptions и scalar alias shortcuts проверяются отрицательными fixtures.

### C7.2 Replay comparator

Существующий replay3:

```text
evidence: |actual-expected| <= max(1e-9, 1e-9*max(|actual|,|expected|))
state:    |actual-expected| <= 1e-9 + 1e-9*|expected|
```

Строки, flags, step/counter integers и schema equality — exact. Nonfinite evidence не проходит finite checked replay. Replay4 явно перечисляет сравниваемые sections и использует те же численные правила, пока изменение comparator не обосновано отдельным contract change. Physical angle equivalence применяется для physics validation отдельно, не для переписывания raw engine checkpoint.

Клиент может читать и отображать чужой compatible recording, даже когда execution identity запрещает checked replay. UI/CLI различают «прочитано», «проиграно» и «проверено повторным расчётом».

## C8. Метрики, hashes и потоки

Для новых channel names обязательно указаны units, frame, sampling phase, measurement/truth classification и nullable/validity policy. `qpos` и `qvel` сохраняют actual joint-specific dimensions. Derived pitch не является alias на quaternion[1/2/3]; catalog free-root не требует `root_x/root_pitch`.

Fingerprint физики включает canonical config, physical terrain, tyre coefficients и numerical profile. Рендер-настройки камеры не меняют physical fingerprint. Полный presentation/replay asset identity может включать visuals отдельно. Mount входит в sensor/replay identity, а physics-only equilibrium cache может иметь отдельную checked subidentity: нельзя случайно reuse state с другой моделью/site layout.

Golden identity хранит платформу, архитектуру, compiler/stdlib/provider/engine/flags, dt/tier/thread policy и hashes inputs. Cross-platform fixtures не сравниваются bitwise друг с другом; frozen oracle tests каждой платформы работают по своей записи.

Deterministic parallel tyre pipeline:

1. Caller публикует immutable frame/query/input generation.
2. Workers пишут только свои заранее выделенные element slots.
3. Caller ждёт завершения всех ranges по generation и проверяет statuses.
4. Patch association/reductions выполняются по canonical element/material order, не по completion order.
5. Единственный owner добавляет force в engine и коммитит material state.

Ни per-thread floating accumulators, размер которых зависит от N, ни shared float atomics не соответствуют bitwise thread invariant. Warm worker callbacks не вызывают `mj_jac` на одном mutable engine context. Engine-side parallelism — отдельное исследование S53 с отдельной numerical identity при изменении order.

Close/reset/failure имеют bounded lifecycle protocol: generation counters, no stale task touching destroyed buffers, join вне realtime tick, fallback serial и тест отрицательного worker failure. Спиннинг не является автоматическим доказательством realtime safety или превосходства над serial.

## C9. Контракты тестового контура

Registrar создаётся **S04** в `native/tests/spatial/CMakeLists.txt` и включается root integrator. Функция `bike_add_spatial_test(slug, source...)` создаёт target `bike_spatial_<slug>`, CTest `spatial.<slug>`, linkage к нужному production owner и existing `bt` harness. Labels: `spatial`, subsystem, `unit|integration|slow|audit|package|gui` согласно фактической проверке.

Новый fixture root `tests/fixtures/spatial3d` не меняет macro старых migration tests. Fixture helper проверяет containment/digest/bytes/schema. Missing binary asset даёт failing acquisition diagnostic, не regenerate-on-demand expected.

Примеры C++ в tasks задают **обязательные assertions и public signatures**, а не претендуют быть полностью готовыми translation units. Обозначенный в тексте fixture — вход конкретного теста, который task создаёт/загружает. Перед передачей в compilation TU содержит реальные includes, `bt::run`, fixture construction и ровно определённые API; неопределённые helper calls не принимаются как реализация.

Минимальный task report: что проверяется, counterexample который проваливает assertion, valid control, команда/exit, executed checks, fixture identity. Формат L0 source-review отчёта содержит `execution: not_run` и не выдаёт псевдопроверку за CTest.

## C10. Изменение контракта

Изменение возможно, если обнаружена конкретная ошибка или измерения опровергают проектную гипотезу. Запись содержит old/new contract, причину, затронутые task IDs, сравнение поведения и migration/readback strategy. До merge обновляются все consumers и task-index dependencies. Нельзя тихо ослабить tolerance, ограничить steer range, выбросить observations или уменьшить tyre quality ради зелёного gate.

| Дата | Решение | Основание | Tasks |
|---|---|---|---|
| 2026-10-22 | Native-only spatial architecture | Существующая PFC цель и native producers | Все |
| 2026-10-22 | Independent container2/catalog2/replay4 | Exact SQL/catalog validation старого reader | S41–S43 |
| 2026-10-22 | Minimal shoulder swivel, no lateral torso balance | Reachability прежних body-Y arms не гарантирует steer | S14/S16/S32 |
| 2026-10-22 | Separate root/query/sensor frames and versions | Freejoint addressing, IMU offset, wire compatibility | S13/S37–S43 |
| 2026-10-22 | BKT3/v1 first actual asset contract | Старый формат был только планом | S06–S12 |
| 2026-10-22 | Serial default, measured optional pool | Runtime/performance ещё не измерены | S50–S54 |

Это решения нового плана, а не фиктивная запись исторического пользовательского approval и не доказательство прохождения tests.
