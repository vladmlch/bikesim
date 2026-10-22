# Spatial 3D Native — наблюдения, research, запись и replay Implementation Plan

> **Для исполнителей:** использовать superpowers:subagent-driven-development (рекомендуется) или superpowers:executing-plans для последовательного выполнения задач; отмечать шаги через `- [ ]`. Если этих skills нет, сохранить цикл «конкретная регрессия → реализация → независимое ревью → доказательство gate». Это не меняет интерфейсы и критерии.

**Goal:** Довести пространственные наблюдения, нативный исследовательский контур, полную запись состояния и checked replay до одного воспроизводимого C++-пути без нового Python-кода.

**Architecture:** Расширяются существующие `runtime`, `research`, `recording` и `replay`; второй симулятор не создаётся. Planar V1 сохраняет свой wire-контракт; пространственные наблюдения используют V2 и совместимый по версии нативный интерфейс policy. Физический шаг остаётся единственным владельцем вычисления сил и сенсорной истины.

**Tech Stack:** C++23; существующий dependency lock MuJoCo/DuckDB; корневой CMake; `bt::run`/`bt::require`; CTest; нативные JSON/codecs и инструменты записи.

**Spec:** `../specs/2026-10-22-spatial-3d-native-design.md`, `2026-10-22-spatial-3d-native-master.md`, `2026-10-22-spatial-3d-native-contracts.md`; интерфейсы замораживаются в S02. Историческая спецификация `../specs/2026-10-14-spatial-3d-design.md` сохраняется как источник исходных требований. ADR `../../adr/0003-native-authoritative-python-removal.md` управляет переходом planar oracle-gates.

## Общие ограничения

- Все задачи ниже **planned**; текст и примеры кода не являются результатом компиляции или запуска. При подготовке плана сборки, тесты и приложения не запускались.
- Реализовывать C++-направление; новые Python-наблюдения, генераторы шумовых последовательностей, метрики и дубли фронтенда не добавлять.
- Не менять `runtime::SensorObservation` V1, `BikeObservationV1`, порядок V1 RNG draws и planar-сериализацию V1 ради spatial.
- Статусы `source-ready`, `compiled`, `verified`, `qualified` различаются. Непройденный gate нельзя заменить наличием исходника.
- Контейнер записи, физические samples, observation, policy ABI, checkpoint и replay recipe имеют независимые версии. Номер V2 наблюдения сам по себе ничего не решает о контейнере.
- Сохраняются явное включение записи, удержание в RAM и публикация только при завершении. `--out` не включает запись автоматически.
- Сенсорная истина принадлежит физическому шагу; viewer, анализатор и отображение replay не измеряют силы или сенсоры повторно.
- Неизвестная будущая версия, несовместимая топология и повреждённый asset дают ясную ошибку до изменения живого состояния.
- Все проверки выполняются позднее, когда разрешено исполнение. Общие headers, CMake и registries изменяет один integration owner.
- G0–G6 в master — **upstream gates PFC**, а M0–M6 — spatial milestones. Эта фаза потребляет G3-research/G4-recording и производит spatial M4; пространственные тесты не переименовывают и не закрывают за владельца PFC его migration gate.

## Изменения основания, которые нельзя пропустить

Текущая V1 уже нативная: `runtime/sensors.hpp:20–32` содержит девять чисел и отдельный dropout-массив. ABI1 закреплён в `native/include/bike/policy_v1.h`. Простое расширение одного `std::array` нарушит потребителей в `runtime/research.hpp`, policy plug-ins, recorder и replay. Поэтому S38–S43 — последовательные задачи совместимости, а не независимые правки форматтера.

Общий построитель координат знает размерности free/ball, но `recording/catalog_model.cpp:27` вызывает overload, который навязывает `root_x` и `root_pitch` (`coordinates.cpp:70–72`). Это отдельное препятствие записи freejoint. Нынешние camera/render данные также не доказывают готовность 3D-схемы.

## Версии и сохранение совместимости

| Ось | Planar сохраняется | Spatial вводится | Причина изменения |
|---|---|---|---|
| Sensor observation | V1, девять чисел | V2, одиннадцать чисел | Полный gyro xyz |
| Noise tape | Девять шумовых значений и отдельный dropout | Одиннадцать шумовых значений и отдельный dropout | Разные математические каналы |
| Policy ABI | `bike_policy_v1` | `bike_policy_v2` | Другая раскладка observation |
| Runtime configuration envelope | runtime config1 | runtime config2 | Пространственный dispatch config/topology/writer по C7.0 |
| Bootstrap state envelope | bootstrap1 | bootstrap2 | Пространственные владельцы и состояние engine при construction/restore |
| Writer configuration | writer config1 или config2 | writer config3 | Identity конфигурации/состояния Pneumatic3d без изменения старого codec |
| Физические samples | Существующая схема | Расширение S02 | Новые поля с собственными units/frame/phase |
| Recording catalog | schema1 | schema2 | Меняются проверяемые SQL поля и таблицы |
| Recording container | schema1 | schema2 | Нынешний validator требует точного множества catalog/table |
| Research replay recipe | contract3 | contract4 | Grid3d assets и полное пространственное состояние |
| Полный semantic checkpoint | Существующий planar codec | `spatial_checkpoint=1` | Новые владельцы материала, контроллера и сенсоров |

Выбор container2 обусловлен `database_validate.cpp:114–150`: нынешняя проверка требует точного множества таблиц и точного `field_catalog`. Скрыто менять schema1 добавлением столбцов нельзя. Reader/writer поддерживают обе схемы; planar по умолчанию продолжает писать schema1. Одновременный mixed planar/spatial run не вводится: версия контейнера фиксируется при создании run, смена spatial mode требует нового run. Inspection старых файлов сохраняется; checked replay подчиняется execution identity.

### Точные изменения SQL для catalog2/container2

Это выбранный **будущий** SQL-контракт S41/S42. DDL строится существующим кодом по descriptors; отдельная ручная копия schema не создаётся. `field_catalog` для каждого нового поля содержит действительный native source path, unit, frame, phase и классификацию measurement/truth. У неизменных таблиц сохраняются прежние поля; значение `recording_meta.schema_version` меняется на 2, а `replay_contract_version` пространственных эпизодов — на 4.

| Таблица | Конкретные изменения | Область / причина |
|---|---|---|
| `observations` | Сохранить keys `episode_id BIGINT`, `observation_id BIGINT`, `time_s DOUBLE`, `source_time_s DOUBLE`, `valid BOOLEAN` и скалярные fields `front_wheel_rad_s`, `rear_wheel_rad_s`, `crank_rad_s`, `motor_torque_nm`, `human_torque_nm` типа DOUBLE | Identity и смысл control period неизменны |
| `observations` | Добавить `sensor_schema_version BIGINT`, `valid_mask BIGINT`; заменить V1 `specific_force_body_mps2` / `pitch_rate_up_rad_s` на `specific_force_sensor_mps2 DOUBLE[]` / `angular_rate_sensor_rad_s DOUBLE[]` | Schema строго 2; mask 0…2047; длина каждого вектора 3; IMU в действительной системе датчика. Невалидная доставка имеет mask0 |
| `channel_spatial` | Keys `episode_id BIGINT`, `interval_id BIGINT`; `root_position_world_m DOUBLE[]`, `root_quat_world_from_body_wxyz DOUBLE[]`, `root_linear_velocity_world_mps DOUBLE[]`, `root_angular_velocity_world_rad_s DOUBLE[]` | Длины 3/4/3/3; истина конца интервала; quaternion world-from-body wxyz, без переписывания raw sign |
| `channel_spatial` | `attitude_valid BOOLEAN`; `roll_rad`, `pitch_rad`, `heading_rad` — nullable DOUBLE; `steer_angle_rad`, `steer_rate_rad_s`, `steer_torque_nm`, `steer_work_j`, `lateral_displacement_m` — DOUBLE | Исходные величины — pose vectors. Достоверность углов по C2: singularity → false/NULL, quaternion/vector truth остаётся конечной. Steer измеряется по joint независимо от Euler chart. Работа накопительная, lateral position относительно объявленного corridor reference |
| `channel_spatial` | `front_contact_valid`, `rear_contact_valid` — BOOLEAN; `front_lateral_slip_mps`, `rear_lateral_slip_mps`, `front_normal_load_n`, `rear_normal_load_n` — nullable DOUBLE; `front_support_normal_world`, `rear_support_normal_world` — nullable DOUBLE[] | Нет опоры на рабочую дорогу → validity false и NULL для неопределённых контактных величин; длина normals 3. Catch-plane не создаёт измерение рабочей опоры |
| `assets` | DDL неизменен; добавить именованные assets `spatial/terrain_source.json`, `spatial/terrain.terrain3d`, `spatial/config.json`, `spatial/calibration.json`, `spatial/checkpoint_initial.json`, `spatial/checkpoint_final.json` | Существующие media_type/hash/content хранят реальные bytes. Grid требует `.terrain3d`; точная profile extrusion сохраняет исходный `track.toml`, source kind и corridor metadata. Version/header каждого asset проверяется отдельно |
| `coordinate_catalog`, `alias_catalog` | DDL неизменен; free root даёт 7 qpos и 6 dof rows; aliases не притворяются scalar root_pitch | Действительные `jnt_qposadr/jnt_dofadr`; вычисленный угол не alias компоненты quaternion |

`channel_spatial` содержит ровно одну строку на физический интервал, поэтому существующий telemetry join сохраняет число строк. Детальные patch/material arrays расширяют versioned tyre channel descriptor по S27; дополнительными строками `channel_spatial` они не становятся. Слой запросов отображает source root `spatial.*` на этот table descriptor, чтобы S47 использовал имена вроде `spatial.roll_rad` без отдельного SQL. V1 reader не получает V2 columns под прежним schema marker.

## Общая карта файлов и владения

| Задача | Главный owner | Новые модули | Основные существующие потребители |
|---|---|---|---|
| S37 | runtime/model | `runtime/spatial_sensors.{hpp,cpp}` | `model/sensors.cpp`, `runtime/step.{hpp,cpp}` |
| S38 | research | `runtime/spatial_sensor_pipeline.{hpp,cpp}` | `research/rider_random.cpp`, `runtime/research_io.cpp` |
| S39 | policy | `include/bike/policy_v2.h` | `research/policy*`, `runtime/research*` |
| S40 | research physics | `runtime/spatial_truth.{hpp,cpp}` | `research_metrics*`, `research/metrics.cpp` |
| S41 | recording | `catalog-v2.json`, `catalog_v2_data.inc` | catalog/coordinates/encoding/schema |
| S42 | state/recording | `runtime/spatial_checkpoint.{hpp,cpp}` | bootstrap, research metadata, assets, publication |
| S43 | replay | `replay/spatial_recipe.{hpp,cpp}` | input/rebuild/session/reader |
| S44 | verification | spatial manifest fixtures | native golden freeze/compare and registries |

Все test-файлы размещаются в `native/tests/spatial/`. Имена ниже означают CTest `spatial.<slug>` и target `bike_spatial_<slug>`, зарегистрированные общим helper S04. Используется существующий `native/tests/support/migration_test.hpp`; `bt::run` отвергает нулевое число проверок. Не вводить Catch2/GTest macros, которых нет в проекте.

## Контракты V2, фиксируемые совместно с S02

Новые объявления находятся в namespace `runtime::spatial`; глобальный namespace `runtime` существующих владельцев сохраняется.

```cpp
namespace runtime::spatial {
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>; // row-major sensor <- nominal frame
struct ImuTruth {
    double time_s = 0.;
    Vector3 specific_force_sensor_mps2{};
    Vector3 angular_rate_sensor_rad_s{};
};
Vector3 rotate_vector(const Matrix3& sensor_from_nominal, const Vector3& value);
struct SensorObservationV2 {
    double time_s = 0., source_time_s = 0.;
    bool valid = true;
    std::uint16_t valid_mask = 0x7ffU;
    std::array<double, 11> values{};
    void validate() const;
    WireObject as_wire() const;
    static SensorObservationV2 from_wire(const WireObject&);
};
struct NoiseTapeV2 {
    std::vector<std::array<double, 11>> noise;
    std::vector<double> dropout_uniform;
    void validate() const;
};
struct SensorConfigV2 {
    double latency_s = .01, maximum_age_s = .1, dropout_probability = 0.;
    double sample_period_s = .01, maximum_read_period_s = .02;
    Vector3 acceleration_bias{}, gyro_bias{};
    Vector3 acceleration_std{.05, .05, .05}, gyro_std{.002, .002, .002};
    double encoder_std = .01, torque_std = .05;
    bool imu_enabled = true;
    void validate() const;
};
struct SensorStateV2 {
    std::deque<SensorObservationV2> queue;
    std::optional<double> last_time, delivery_time;
    std::optional<SensorObservationV2> startup;
    std::size_t cursor = 0, samples_attempted = 0, samples_dropped = 0;
};
class SensorPipelineV2 {
public:
    SensorPipelineV2(SensorConfigV2, NoiseTapeV2);
    void reset(const SensorObservationV2&);
    void import_state(SensorStateV2);
    void push(const SensorObservationV2&);
    SensorObservationV2 read(double time_s);
    SensorStateV2 state() const; // cold snapshot only
    WireObject state_wire() const;
private:
    SensorConfigV2 config_;
    NoiseTapeV2 tape_;
    std::vector<SensorObservationV2> queue_storage_; // sized at construction
    std::size_t queue_head_ = 0, queue_count_ = 0;
    std::optional<double> last_time_, delivery_time_;
    std::optional<SensorObservationV2> startup_;
    std::size_t cursor_ = 0, samples_attempted_ = 0, samples_dropped_ = 0;
};
}
```

В объявлениях выше `WireObject` означает существующий `runtime::WireObject` из `samples.hpp`. Обязательные includes: `<array>`, `<deque>`, `<optional>`, `<vector>`, `<cstddef>`, `<cstdint>` и `samples.hpp`. Времена конечны и неотрицательны; `time_s` — время доставки, `source_time_s` — физического измерения, не позже доставки. V2 wire names — `specific_force_sensor_mps2` и `angular_rate_sensor_rad_s`, по три числа; остальные пять скалярных имён сохраняют смысл энкодеров wheel/crank и измеренных моментов. Индексы: `[acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,front,rear,crank,motor,human]`. `sensor_schema_version=2` и `valid_mask` обязательны; биты вне `0x7ff` отвергаются, `valid=false` означает mask0. Отключённая IMU очищает bits0–5, сохраняя другие каналы; `valid=true` с оставшимися bits допустим, policy проверяет собственную required mask. Метаданные frame хранят nominal site pose, misalignment transform и calibration revision; `*_body` не обозначает неподтверждённый вектор системы датчика.

`SensorStateV2::queue` — представление checkpoint вне hot path. `push/read` используют заранее выделенный ring; `state()` строит снимок только вне tick. `sample_period_s` и `maximum_read_period_s` конечны и строго положительны; runtime разрешает их из действительных sample/control clocks до construction, проверяет расписание и сохраняет resolved values в metadata. Ёмкость = `ceil((latency_s+maximum_read_period_s)/sample_period_s)+3`, с проверкой представимости и ресурсного предела. Переполнение, слишком частый sample или нарушение обещанного интервала чтения делают запуск явно невалидным; история не удаляется молча. Import/export снимков и replay checkpoints выделяют память вне hot path; скалярные поля сериализуются независимо от внутреннего storage.

---

## S37 — Нативные монтаж IMU, система координат и свежая стадия измерения

**Dependencies:** S02; topology/frames S13–S19; S29–S36 для интеграционных fixtures. Независимые проверки вращений и области значений можно подготовить до G2-startup.

**Files:** Create `native/src/runtime/spatial_sensors.{hpp,cpp}`; Modify `native/src/model/sensors.cpp`, `native/src/runtime/step.{hpp,cpp}`, `native/src/runtime/presentation.cpp`; Test `native/tests/spatial/sensor_mount.cpp` (`spatial.sensor_mount`).

**Consumes:** `bike::spatial::Topology`, `BodyPose`, зафиксированный mount из `Config`, принятый физический шаг и действительные sensor addresses. **Produces:** `runtime::spatial::ImuTruth`, `rotate_vector(Matrix3,Vector3)`, metadata site/frame/calibration; accelerometer и gyro одной объявленной стадии измерения.

- [ ] Добавить регрессии для единичного и 90° поворота mount, неправильной матрицы; проверять ортонормальность и determinant +1 при construction. Отрицательные/NaN времена отклонять до чтения состояния.
- [ ] Создать отдельный именованный sensor site по `Config`; разрешить sensor IDs→`sensor_adr` один раз, проверить type, width=3 и принадлежность body. Старый planar `site_BB` не менять.
- [ ] Снять gyro и accelerometer после constraint solve до endpoint forward, сохраняя семантику интервала `step.cpp:2153–2155`. Не смешивать qvel нового endpoint с прежним sensordata.
- [ ] Включить новый torque/hold/force путь в действующий контракт свежести `refresh_forward`/acceleration-input snapshot. Импульс steering/tyre force должен отличать свежий sample от искусственно устаревшего.
- [ ] Для смещённого mount проверить касательное и центростремительное ускорения; для наклонённого неподвижного mount — повёрнутый gravity-specific-force. Побитовое V1-pitch = minus-V2-gyro-y требовать только при одинаковых site/frame/stage.
- [ ] Проверить, что многократное чтение render/presentation не меняет физическое состояние и не пересчитывает силы/сенсоры; закрепить unit/frame/timestamp в каталоге S41.
- [ ] Выполнить адресную проверку и ревью свежести/исходников; commit `feat(spatial): native sensor mount and interval truth`.

```cpp
const runtime::spatial::Matrix3 identity{1.,0.,0., 0.,1.,0., 0.,0.,1.};
const runtime::spatial::Vector3 v{2.,-3.,9.81};
bt::require(runtime::spatial::rotate_vector(identity, v) == v,
            "identity mount preserves the declared sensor vector");
```

**Acceptance:** все шесть компонентов истины имеют одну подтверждённую стадию и систему координат; bytes прежнего planar sample сохраняются. **Failure/rollback:** неправильные site/type/matrix останавливают construction; устаревшее ускорение нельзя маскировать дополнительным viewer `mj_forward`. Одного теста неподвижности недостаточно.

## S38 — Observation V2, noise11 и состояние задержки/dropout

**Dependencies:** S02, S37; upstream G3-research подтверждает native clock/noise/policy host. Независимые unit-тесты V2 можно подготовить до G3; интеграция ожидает доказательств готовности производителя. S38 даёт часть spatial M4.

**Files:** Create `native/src/runtime/spatial_sensor_pipeline.{hpp,cpp}`; Modify `native/src/research/rider_random.cpp`, `native/src/research/environment.hpp`, `native/src/research/configuration.cpp`, `native/src/runtime/research_io.{hpp,cpp}`; Test `native/tests/spatial/sensor_pipeline.cpp`.

**Consumes:** `ImuTruth`, измерения wheel/crank/torque, `SensorConfigV2`, отдельная seed identity. **Produces:** объявленные `SensorObservationV2`, `NoiseTapeV2`, `SensorStateV2`, `SensorPipelineV2`; native `make_sensor_tape_v2(const SensorConfigV2&, std::string_view seed_decimal, std::int64_t count)` в `bike::research`, возвращающий `runtime::spatial::NoiseTapeV2`.

- [ ] Зафиксировать V1 seed0 fixture без изменения девяти позиций RNG draws. Создать V2 fixture из одиннадцати значений и отдельный dropout-vector; записать алгоритм и ревизию V2 RNG в metadata.
- [ ] Реализовать bias/std каждой оси, проверку конечности, probability `[0,1]`, неотрицательные latency/maximum_age/count и представимость размеров tape. RNG использует существующие native `SeedSequence`/`NumpyRng`; `std::random_device` не добавляется.
- [ ] Реализовать reset ровно с одной стартовой попыткой, монотонное время измерений, задержанную доставку и проверку maximum age. Отключённая IMU зануляет первые шесть каналов и очищает bits0–5; остальные измеряются. Устаревший/dropout стартовый sample имеет `valid=false, valid_mask=0`; bytes отдельных valid channels не исчезают из-за общего timeout без явного правила.
- [ ] Создать V2 codec с точным набором keys; неизвестные поля/schema, ширина десять/двенадцать, несогласованные счётчики, старт из будущего и одинаковые timestamps очереди отклоняются атомарно.
- [ ] Проверить, что исчерпание noise делает запуск невалидным с объяснимой причиной; dropout одинаково потребляет noise/cursor независимо от числа доставленных samples. Отдельный dropout stream не зависит от количества нормальных IMU draws.
- [ ] Сравнить продолжение после import_state с непустой FIFO и ненулевой задержкой с непрерванным запуском; проверка состояния не продвигает noise cursor.
- [ ] Подключить schema dispatch после S39; выполнить адресную native-проверку и commit `feat(spatial): eleven-channel sensor pipeline`.

```cpp
runtime::spatial::NoiseTapeV2 tape;
tape.noise.push_back(std::array<double, 11>{});
tape.dropout_uniform.push_back(0.);
tape.validate();
bt::require(tape.noise.front().size() == 11, "V2 has eleven measured channels");
tape.dropout_uniform.clear();
bt::invalid([&] { tape.validate(); });
```

**Acceptance:** V1 bytes неизменны; повтор V2 seed/state даёт одинаковые измерения и счётчики доставки. **Failure/rollback:** ошибка row/state оставляет прежнюю pipeline пригодной; новая схема не ужимается молча до девяти значений для старой policy. Выделение памяти проверяется отдельно от валидности wire.

## S39 — Нативный research dispatch и policy ABI V2

**Dependencies:** S38, upstream G3-research; существующий T35 policy ABI owner. Приемка S39 входит в spatial M4, не заменяет baseline PFC gate. Перед shared-header правками integration owner подтверждает один change-set.

**Files:** Create `native/include/bike/policy_v2.h`; Modify `native/src/research/policy.{hpp,cpp}`, `plugin_policy.cpp`, `builtin_policies.cpp`, `policy_session.cpp`, `native/src/runtime/research.{hpp,cpp}`, `research_io.{hpp,cpp}`; Test `native/tests/spatial/policy_v2.cpp`, `native/tests/spatial/plugins/policy_v2_control.cpp`.

**Consumes:** V1 или V2 observation, выбранные mode/schema и versioned plugin capabilities. **Produces:** управляемый schema dispatch существующей research session, `bike_policy_v2()` и `BikeObservationV2`; смысл управления мотором сохраняется. Балансировка рулём остаётся у внутреннего rider/plant owner, а не становится новой anti-wheelie policy.

- [ ] В ABI2 определить собственные `BIKE_POLICY_V2_ABI_VERSION=2U`, `BIKE_SENSOR_V2_SCHEMA_VERSION=2U`, `BIKE_SENSOR_V2_VALUE_COUNT=11U`; `BikeObservationV2` имеет struct_size, valid, time/source_time, values[11], schema и valid_mask. Не менять V1 macros/struct offsets.
- [ ] Определить `BikePolicyV2` с create/reset/act/destroy, size/version/capabilities и новой точкой экспорта; существующие скалярные `BikeDemandV1`, `BikeControlV1`, `BikeErrorV1` остаются неизменными C-совместимыми payloads с явным include `policy_v1.h`.
- [ ] Добавить `using ResearchObservation = std::variant<SensorObservation, spatial::SensorObservationV2>` в header владельца research. Заменить поля transition/history/current observation на alias; V1 codec сохраняет bytes, V2 выбирается явным discriminator.
- [ ] На границе применить typed visitation к sensor pipeline и policy interface; физическая `NativeResearchRuntime` остаётся одна. Встроенные native policies получают явную V2-поддержку; V1-only plugin в spatial завершается `unsupported sensor schema` без обрезки или неявной проекции.
- [ ] Проверить error/exception/NaN, несовпадение capability/size и безопасное закрытие модуля. Plugin видит только sensor observation и demand, без terrain, force, truth, model pointer или checkpoint.
- [ ] Сравнить command/applied/observation последовательности research без окна и с ограничением темпа при одинаковом seed и неограничивающем wall limit; replay получает тот же выбор schema.
- [ ] Выполнить отрицательные ABI controls, адресные проверки и ревью обратной совместимости; commit `feat(spatial): versioned native policy observations`.

```cpp
BikeObservationV2 observation{};
observation.struct_size = sizeof(BikeObservationV2);
observation.sensor_schema_version = BIKE_SENSOR_V2_SCHEMA_VERSION;
bt::require(std::size(observation.values) == 11, "ABI2 carries all six IMU axes");
bt::require(BIKE_SENSOR_VALUE_COUNT == 9U, "including ABI2 leaves ABI1 intact");
```

**Acceptance:** ABI1 planar fixtures прежние, V2 plugin получает ровно 11 каналов, запись явно сообщает schema. **Failure/rollback:** ошибка выбора plugin возникает до первого control window; незавершённое создание session закрывает plugin/runtime через существующий RAII. Schema не переключается посреди эпизода.

## S40 — Spatial truth, публикация исходов и метрики

**Dependencies:** S19, S28, S33, S36, S39; контракты transform/terrain/contact закреплены S02. S33 — единственный производитель terminal priority/dwell/latch.

**Files:** Create `native/src/runtime/spatial_truth.{hpp,cpp}`; Modify `native/src/runtime/research_metrics.{hpp,cpp}`, `native/src/research/metrics.cpp`, `quality.cpp`, `evidence.cpp`, `native/src/runtime/research.cpp`; Test `native/tests/spatial/truth_metrics.cpp`.

**Consumes:** `bike::spatial::BodyPose`, действительные нормали/нагрузки/зазоры контактов колёс, `terrain::Grid3d`, диагностика steering/tyre и S33 `OutcomeState`/зафиксированный `spatial::Outcome` с исходным accepted tick. **Produces:** пространственные truth fields, metric keys и публикация уже принятого terminal decision; policy observations не расширяются истиной. Второй classifier/timer не создаётся.

- [ ] Добавить чистую функцию `double integrate_abs_slip(double accumulated_m,double slip_mps,double dt_s)` в `runtime::spatial`: конечные входы, accumulated≥0, dt>0; результат `accumulated_m+std::abs(slip_mps)*dt_s`. Для остальных интегралов зафиксировать точные величину и единицу.
- [ ] Убрать пространственные предположения `qpos[root_pitch]`, 1D road pitch и root_x speed из `research_truth`; получать pose/velocity из topology и локальной системы опоры, сохраняя planar-функцию.
- [ ] Определить roll/yaw/heading с validity C2, боковое смещение, знаковую боковую скорость скольжения, путь скольжения каждого колеса и момент/работу руления. При singularity сохранять quaternion/vector truth, ставить `attitude_valid=false`, углы absent/NULL; прошлый угол не выдаётся за новый. Нормальная нагрузка определяется по местной terrain normal; catch-plane не считается рабочей опорой.
- [ ] Публиковать зафиксированные reason/tick S33 без повторного dwell, приоритетов или reset terminal latch. Различать front-unloaded, front-lift и flight как truth metrics; итоговый capsize/off-corridor принадлежит S33. Низкая скорость сама по себе не означает capsize и не оправдывает внешнюю стабилизацию.
- [ ] Проверить синтетические состояния после сдвига/поворота, левую/правую симметрию, поперечный уклон и crest, разные разбиения dt для интегралов. Старые planar metric dictionaries сохраняют keys/bytes; новые поля принадлежат только spatial schema.
- [ ] Проверить независимость метрик и terminal tick от record decimation, обновления UI, research polling и capture off/full при одинаковой частоте физики. S33 принимает решение на accepted physics tick; research читает latch. Метрика с входом каждого шага вычисляется до прореживания telemetry.
- [ ] Выполнить адресную проверку, ревью source/model и commit `feat(spatial): native truth and event metrics`.

```cpp
bt::require(runtime::spatial::integrate_abs_slip(0., -2., .25) == .5,
            "slip distance integrates absolute lateral velocity");
bt::invalid([] { (void)runtime::spatial::integrate_abs_slip(0., 1., 0.); });
```

**Acceptance:** единицы/frames определены, S33 outcomes публикуются однозначно; terminal reason/tick совпадают при любом polling/decimation, front-lift отличается от общей потери контакта. **Failure/rollback:** невалидная истина передаётся существующему S33 invalidity path; неизвестная terrain normal не заменяется global up, метрика — нулём, а terminal decision — локальной research эвристикой.

## S41 — Spatial catalog, координаты и source channels

**Dependencies:** S37–S40, S19 topology и upstream G4-recording. S41/S42/S43 производят recording/replay часть spatial M4. Можно раньше проектировать schema descriptors, но не помечать их produced до upstream producer evidence.

**Files:** Create `native/resources/recording/catalog-v2.json`, `native/src/recording/catalog_v2_data.inc`; Modify `native/src/recording/catalog.{hpp,cpp}`, `catalog_model.cpp`, `coordinates.cpp`, `schema.cpp`, `encoding.cpp`, `plan.cpp`, `physical_source.cpp`, `research_source.cpp`; Test `native/tests/spatial/recording_catalog.cpp`.

**Consumes:** sensor2, пространственная truth/contact/tyre telemetry, топология free7/6. **Produces:** schema2 descriptor и его встроенная копия, aliases с учётом топологии, явный выбор schema для всех table/encoding lookup.

- [ ] Добавить `enum class CatalogVersion { planar1=1, spatial2=2 };` и перегрузки `build_catalog(const mjModel&,CatalogVersion)`, `schema_catalog(CatalogVersion)`, `all_tables(CatalogVersion)`. Старые overloads продолжают означать planar1; schema version становится свойством catalog/reader/run, не глобальной мутабельной переменной.
- [ ] Model overload читает `jnt_qposadr/jnt_dofadr`; размерность free qpos 7, dof 6, quaternion unit1. В spatial не создавать scalar aliases `root_x/root_pitch`. Вычисленные pitch/roll/yaw — именованные каналы с определённым преобразованием.
- [ ] Для каждого поля закрепить unit/frame/phase/source/optional/presence; SQL float columns не скрывают различие знака и модуля. Отдельно описать sensor schema, mount metadata и contact validity.
- [ ] Протянуть выбранную версию во все encode/DDL/validator paths. Синхронизировать встроенный asset и установленный JSON нативным сравнением hash; Python для генерации не использовать.
- [ ] Проверить общий overload и настоящий compiled freejoint `build_catalog(mjModel&,spatial2)`: второй обнаруживает ошибку неявных planar aliases. Отдельно проверить неизвестный channel, duplicate key, несовместимую ширину и различие null/absent.
- [ ] Сравнить planar1 descriptor/aliases/сериализацию строк с прежними fixtures; один spatial interval даёт ровно одну строку channel root, patch arrays не размножают базовый join.
- [ ] Выполнить адресный CTest и ревью владения schema; commit `feat(spatial): versioned native recording catalog`.

```cpp
const std::array<bike::recording::JointCoords,1> joints{{
    {.name="root_free",.joint_type="free",.qposadr=0,.dofadr=0}
}};
const auto control = bike::recording::build_catalog(
    joints, std::span<const bike::recording::ScalarAlias>{});
bt::require(control.coordinates().qpos.size() == 7, "free positions are not DOFs");
bt::require(control.coordinates().dof.size() == 6, "free velocity width");
```

**Acceptance:** полный каталог реальной модели работает, descriptor согласован с writer, schema1 неизменна. **Failure/rollback:** неподдержанная schema отклоняется до создания выходного файла; validator не ослабляется ради принятия произвольных новых полей.

## S42 — Полное состояние, schema2 container и вложенные assets

**Dependencies:** S27 tyre state/codec, S36 startup/steering state, S38 sensor state, S41 catalog; upstream G4-recording и existing capture/publication owner. S28 physical rigs квалифицируют state producer и обязательны для итогового M4 evidence, но не являются producer API.

**Files:** Create `native/src/runtime/spatial_checkpoint.{hpp,cpp}`; Modify `native/src/runtime/bootstrap.{hpp,cpp}`, `runtime.cpp`, `research.cpp`, `native/src/research/metadata.cpp`, `native/src/recording/{contracts.hpp,assets.cpp,database.cpp,database_validate.cpp,reader.cpp,research_publication.cpp,publish.cpp}`; Test `native/tests/spatial/checkpoint.cpp`, `native/tests/spatial/recording_assets.cpp`.

**Consumes:** C7.0 runtime config2/bootstrap2/writer config3, owned research/sensor/policy state, точный источник terrain и Config identity. **Produces:** `spatial_checkpoint=1`, schema2 container, replay contract4 recipe; bytes источника, разрешённая калибровка и начальный/конечный semantic checkpoint вложены вместе с hashes. Checkpoint оборачивает bootstrap2 и research/sensor/policy extensions; tyre/brake/steer сохраняют по одному bootstrap owner.

- [ ] Принять точный payload C7.0 и назначить каждому полю одного owner: bootstrap2 содержит engine/model и tyre/brake/steer/rider/release/filter state; wrapper добавляет research clock/FIFO/window/ack/metrics и sensor/policy state. Не копировать tyre/brake/steer повторно наверх. Логические размеры/version входят, reserve capacity/scratch/pointers — нет. Для policy сохраняется сериализуемое состояние либо явно запрещается resume; checked plant replay по командам не доказывает продолжение произвольного plugin.
- [ ] Реализовать restore через stage-validate-commit: размеры/hashes проверяются перед изменением engine, память выделяется до commit; ошибка последнего поля не меняет ни один owner. Scored checkpoint не активирует bootstrap hold.
- [ ] Сохранить source kind: grid — точные bytes/axes/bounds/sample convention; profile_extrusion — исходные knots/material profile без пересэмплирования и corridor. Config filenames — происхождение, не разрешение читать их во время replay. Hash включает canonical input, resolved calibration/revision и marker снятого world hold.
- [ ] Ввести отдельную container2 ветку manifest/database validator, сохранив schema1 reader/writer. Версия run фиксирована; spatial reset не меняет schema. Публикация только финальная и exclusive, с прежним retry protocol.
- [ ] Сравнить непрерванный запуск с restore при непустых sensor/actuator FIFO, активном braking anchor и ненулевых lateral bristles. Сравнить весь semantic state и последующие measurement/control rows.
- [ ] Проверить повреждённый terrain, несовместимую topology, переставленные/пропущенные checkpoint fields, прерванную публикацию и retry без повторного stepping. Адрес живого owner не сериализуется как identity.
- [ ] Выполнить адресные проверки, независимое ревью транзакционности и commit `feat(spatial): complete state and recording assets`.

```cpp
runtime::spatial::NoiseTapeV2 tape;
tape.noise.resize(2); tape.dropout_uniform.assign(2, .5);
runtime::spatial::SensorPipelineV2 pipeline({}, std::move(tape));
pipeline.reset(runtime::spatial::SensorObservationV2{});
const auto before = pipeline.state();
auto invalid = before; invalid.cursor += 1;
bt::invalid([&] { pipeline.import_state(invalid); });
bt::require(pipeline.state().cursor == before.cursor,
            "rejected checkpoint does not consume sensor state");
```

Этот короткий control не заменяет проверки продолжения выше. **Acceptance:** полный restore даёт прежнее продолжение; перенесённая запись сохраняет нужные assets. **Failure/rollback:** неуспешный restore оставляет прежнее пригодное состояние; неуспешная публикация сохраняет данные и возможность retry. Один engine `mjSTATE_INTEGRATION` не называется полным checkpoint.

## S43 — Checked replay spatial4 без ослабления comparator

**Dependencies:** S42, upstream G3-research/G4-recording и existing checked-replay reconstruction owner миграции. S43 закрывает replay часть spatial M4; S44 после этой задачи закрепляет fixtures.

**Files:** Create `native/src/replay/spatial_recipe.{hpp,cpp}`; Modify `native/src/replay/{input.cpp,input.hpp,rebuild.cpp,session.cpp}`, `native/src/recording/reader.cpp`, `database_validate.cpp`; Test `native/tests/spatial/checked_replay.cpp`.

**Consumes:** container2/catalog2/recipe4, вложенные assets, полный checkpoint и исходная execution identity. **Produces:** та же `ReplaySession` с явным recipe dispatch; результат содержит schema/version/provenance и path/step первого несовпадения. Inspection остаётся отдельным режимом.

- [ ] Принимать только известные recipe3 planar и recipe4 spatial. Сохранить безопасность путей `FixedAssets`, запрет внешних caches/input paths; spatial выбирает записанный grid/profile_extrusion, проверяет bytes/hash и восстанавливает ту же native model/config без пересэмплирования.
- [ ] Заменить пространственную ветку старых 1D-проверок (`rebuild.cpp:115–123,169–173`); проверять grid dimensions/axes/material config и topology/state shape. Неизвестная версия отклоняется до создания runtime.
- [ ] Сохранить численные правила: recursive evidence `abs(a-e)<=max(1e-9,1e-9*max(abs(a),abs(e)))`; integration vector `abs(a-e)<=1e-9+1e-9*abs(e)`. Сравнить полное semantic state с диагностикой по объявленной политике полей; общий epsilon не увеличивать.
- [ ] Восстановить начальный/конечный checkpoints, observation V2, requested/applied commands, transitions, завершение и метрики. Сравнить все конечные поля, не только `qpos`; одинаковая отрисовка quaternion q/−q не разрешает пропустить различие raw replay.
- [ ] Требовать прежнюю source/numeric-provider/dependency identity по existing checked replay. Изменённый профиль машины сообщает `incompatible-execution`; просмотр разрешён без объявления симуляции verified.
- [ ] Создать детерминированные повреждения одной команды, оси gyro, material bristle, ячейки terrain и calibration revision. Проверить reset, уступку исполнения, ранний stop и сохранение последней проверенной presentation generation при первом mismatch.
- [ ] Сохранить адресные доказательства; commit `feat(spatial): checked native replay contract four`.

```cpp
using V = bike::io::Value;
const auto same = bike::replay::compare_evidence(V{true}, V{true}, "flag");
const auto changed = bike::replay::compare_evidence(V{false}, V{true}, "flag");
bt::require(same.matched && !changed.matched, "replay checks nonnumeric evidence");
bt::require(changed.first_path == "flag", "first mismatch preserves field identity");
```

**Acceptance:** полный подходящий spatial episode воспроизводится; каждое повреждение даёт ожидаемую диагностику, recipe3 неизменна. **Failure/rollback:** mismatch не продвигает скрытое физическое состояние через отрисовку; restart получает новую presentation generation. Replay с допуском не доказывает побитовый детерминизм или физическую точность.

## S44 — Нативные fixtures, собственные эталоны и реестр проверок

**Dependencies:** S37–S43, S28/S36 physical gates; G1-core, G2-startup, G3-research, G4-recording.

**Files:** Create `tests/fixtures/spatial3d/manifest.json`, `native/src/golden/spatial_registry.{hpp,cpp}`, `native/tests/spatial/golden_registry.cpp`; Modify `native/src/golden/{episode.cpp,capture.cpp}`, `native/src/cli/recording_golden.cpp`, `native/tests/spatial/CMakeLists.txt`, `docs/TESTING.md`; artifacts каждой платформы добавлять только после квалифицированного запуска.

**Consumes:** нативная семантика sensor/replay, происхождение fixtures и независимые физические эталоны. **Produces:** машинно читаемый реестр task→requirement→test→platform→result и проверенные эталоны каждой платформы; классы сравнений отделены от физической приёмки.

- [ ] Определить `void bike::golden::validate_spatial_manifest(const bike::io::Value&)` в новом header. Entries содержат test name, исходную задачу, classification (`analytic`, `physical_reference`, `deterministic`, `golden`, `compatibility`), config/model/terrain/calibration hashes, provider identity, schema versions и ожидаемые channels; root `spatial_manifest_version=1`. Missing fixture, пустой executable и нулевой выбор CTest дают ошибку.
- [ ] Расширить native `bike-recording golden freeze/compare` на spatial recording/state, без нового `golden_manifest.py`. Freeze выполняется после физических стендов и ревью исходников; первый полученный trace не становится эталоном автоматически.
- [ ] Сохранить planar live oracle для каждого test file до появления consumed-value capture обеих платформ macOS-arm64/Linux-aarch64 и C++ port, как требует ADR0003. Нативные 3D-тесты не получают Python-дубликатов или parity с непроверенной подстановкой.
- [ ] Записывать отдельные platform trees; повторные запуски и разные потоки сравнивать внутри одной сборки. Межплатформенные допуски задаются по полям с физическим обоснованием; общего побитового обещания для разных providers нет.
- [ ] Проверить подмену hash, missing artifact, неверную schema, несовместимую калибровку и устаревшую source identity; явно различать incompatibility, регрессию и физическую невалидность. Обновление self-golden сопровождается причиной и сравнением, не массовой перегенерацией.
- [ ] Зарегистрировать targets в существующем source/sweep coverage; пометить долгие, GUI и performance тесты. Стандартный preset исключает slow/audit/package, поэтому реестр перечисляет их отдельно; default CTest не считается полной проверкой.
- [ ] Подготовить проверенную таблицу покрытия и commit `test(spatial): native validation and golden registry`.

```cpp
const auto future = bike::io::parse_json(R"({"spatial_manifest_version":999})");
bt::invalid([&] { bike::golden::validate_spatial_manifest(future); });
const auto empty = bike::io::parse_json(R"({"spatial_manifest_version":1,"entries":[]})");
bt::invalid([&] { bike::golden::validate_spatial_manifest(empty); });
```

**Acceptance:** каждая заявленная возможность имеет непустую native-проверку и provenance; неизмеренные платформы остаются pending. **Failure/rollback:** неуспешный physical/reference test блокирует freeze fixture; прежний принятый golden сохраняется. Планирование source-ready не превращается в квалифицированное доказательство отметкой checkbox.

## Команды адресной проверки при реализации

Это команды будущего исполнения, не журнал этой задачи. Targets создаются S04/S37–S44.

```bash
cmake --build build/native-release --target bike_spatial_sensor_mount
ctest --test-dir build/native-release -R '^spatial\.sensor_mount$' --no-tests=error --output-on-failure
cmake --build build/native-release --target bike_spatial_checked_replay
ctest --test-dir build/native-release -R '^spatial\.checked_replay$' --no-tests=error --output-on-failure
```

## Передача следующим исполнителям

- S37–S40 передают S45/S46 семантику native observable/schema и принятые presentation inputs.
- S41–S43 передают S47 записи/read APIs, S48 устанавливаемые assets и S54 полную проверку replay.
- S44 передаёт S50/S54 classification и правила происхождения; скорость он не подтверждает автоматически.
- Фаза05 закрывается по spatial M4 evidence на согласованном executable поверх upstream G3/G4; нерешённый gate владельца миграции остаётся явной зависимостью.
