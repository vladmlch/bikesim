// research_io.cpp — read_research_bootstrap(): the filesystem face of the
// owned research bootstrap (task T10).
//
// Layout: <directory>/{config.json, state.json, model.mjb, research.json}.
// The ride trio decodes through read_bootstrap()'s own ordinal order;
// research.json carries {"research", "noise", "dropout_uniform",
// "sensor_state"} under the capture codec's externalized tags — resolved
// through resolve_bootstrap_document() so $array payloads verify their
// declared dtype/shape/order/sha256 against the capture-root-relative
// path convention before decode.
//
// decode_research_config/decode_noise_tape/decode_sensor_state are the
// vread:: twins of the binding's parse_config/parse_tape/
// parse_sensor_state: identical field order, identical acceptance set,
// identical "path: reason" diagnostics. The only acceptance difference a
// document can express is the tape rank gate — the binding read ndarray
// .ndim; an externalized $array resolves to a nested Value tree, so a
// rank violation surfaces at the row-width/finite-real read in the same
// rejection class (a (0, N) empty tape cannot spell a nested tree, which
// is exactly the ambiguity the ndim read guarded).
#include "research_io.hpp"

#include "bootstrap_io.hpp"
#include "value_readers.hpp"
#include "../io/json.hpp"
#include "../rider/state_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace runtime {
namespace {

namespace bio = bike::io;
namespace fs = std::filesystem;
using namespace sample_wire;

[[noreturn]] void io_invalid(std::string_view field, std::string_view reason) {
    throw std::invalid_argument(std::string("read_research_bootstrap: ") +
                                std::string(field) + ": " +
                                std::string(reason));
}

[[nodiscard]] std::vector<std::byte> read_file(const fs::path &path,
                                              std::string_view field) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) io_invalid(field, "cannot open file");
    const std::streampos end = file.tellg();
    if (end < 0) io_invalid(field, "cannot size file");
    std::vector<char> raw(static_cast<std::size_t>(end));
    file.seekg(0);
    if (!raw.empty() &&
        !file.read(raw.data(), static_cast<std::streamsize>(raw.size())))
        io_invalid(field, "cannot read file");
    std::vector<std::byte> bytes(raw.size());
    std::ranges::copy(std::as_bytes(std::span{raw}), bytes.begin());
    return bytes;
}

[[nodiscard]] std::string text(const std::vector<std::byte> &bytes) {
    std::string out;
    out.reserve(bytes.size());
    for (const std::byte unit : bytes)
        out.push_back(std::to_integer<char>(unit));
    return out;
}

// ---- the sensor envelope -------------------------------------------------

SensorConfig decode_sensors(const bio::Value &document) {
    const vread::Dict d{.value = &vread::mapping(vread::Node{&document},
                                                "sensors"),
                        .path = "sensors"};
    vread::exact(d, vread::keys("latency_s", "acceleration_std_mps2",
        "gyro_std_rad_s", "encoder_std_rad_s", "torque_std_nm",
        "imu_enabled", "sample_period_s", "acceleration_bias_mps2",
        "gyro_bias_rad_s", "dropout_probability", "maximum_age_s"));
    SensorConfig c;
    c.latency_s =
        vread::finite_real(d["latency_s"], d.child("latency_s"));
    c.acceleration_bias = vread::fixed<3>(d["acceleration_bias_mps2"],
                                          d.child("acceleration_bias_mps2"));
    c.gyro_bias =
        vread::finite_real(d["gyro_bias_rad_s"], d.child("gyro_bias_rad_s"));
    c.dropout_probability = vread::finite_real(
        d["dropout_probability"], d.child("dropout_probability"));
    c.maximum_age_s =
        vread::finite_real(d["maximum_age_s"], d.child("maximum_age_s"));
    c.imu_enabled = vread::boolean(d["imu_enabled"], d.child("imu_enabled"));
    for (const char *key : {"acceleration_std_mps2", "gyro_std_rad_s",
                            "encoder_std_rad_s", "torque_std_nm"})
        if (vread::finite_real(d[key], d.child(key)) < 0.)
            vread::invalid(d.child(key), "must be nonnegative");
    if (vread::finite_real(d["sample_period_s"],
                           d.child("sample_period_s")) <= 0.)
        vread::invalid(d.child("sample_period_s"), "must be positive");
    c.validate();
    return c;
}

} // namespace

namespace {
void validate_sensor_state_v2_shape(const spatial::SensorStateV2& s) {
    if(s.cursor!=s.samples_attempted||s.samples_dropped>s.cursor||s.queue.size()>s.cursor||
        std::cmp_greater(s.cursor,std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument("sensor_v2.state: inconsistent or unrepresentable counters");
    if(s.cursor==0) {
        if(s.last_time||s.delivery_time||s.startup||!s.queue.empty())
            throw std::invalid_argument("sensor_v2.state: uninitialized state contains samples/clocks");
        return;
    }
    if(!s.last_time||!s.startup||s.queue.empty())
        throw std::invalid_argument("sensor_v2.state: initialized state lacks startup/latest attempt");
    if(!std::isfinite(*s.last_time)||*s.last_time<0.||
        (s.delivery_time&&(!std::isfinite(*s.delivery_time)||*s.delivery_time<0.)))
        throw std::invalid_argument("sensor_v2.state: clock must be finite and nonnegative");
    s.startup->validate();
    if(s.startup->source_time_s>*s.last_time||
        (s.delivery_time&&*s.delivery_time<s.startup->source_time_s))
        throw std::invalid_argument("sensor_v2.state: startup source exceeds acquisition/delivery clock");
    std::optional<double> previous;
    for(const auto& sample:s.queue) {
        sample.validate();
        if((previous&&sample.source_time_s<=*previous)||sample.source_time_s<s.startup->source_time_s||sample.source_time_s>*s.last_time)
            throw std::invalid_argument("sensor_v2.state.queue: source times must be unique within acquired prefix");
        previous=sample.source_time_s;
    }
    if(!previous||*previous!=*s.last_time)
        throw std::invalid_argument("sensor_v2.state.queue: latest attempt differs from acquisition clock");
}
}

WireObject sensor_config_v2_to_wire(const spatial::SensorConfigV2& c) {
    c.validate();
    return {{"latency_s",c.latency_s},{"maximum_age_s",c.maximum_age_s},{"dropout_probability",c.dropout_probability},
        {"sample_period_s",c.sample_period_s},{"maximum_read_period_s",c.maximum_read_period_s},
        {"acceleration_bias",wire_array(c.acceleration_bias)},{"gyro_bias",wire_array(c.gyro_bias)},
        {"acceleration_std",wire_array(c.acceleration_std)},{"gyro_std",wire_array(c.gyro_std)},
        {"encoder_std",c.encoder_std},{"torque_std",c.torque_std},{"imu_enabled",c.imu_enabled}};
}
spatial::SensorConfigV2 decode_sensor_config_v2(const bio::Value& document) {
    const auto wire=vread::wire_from_value(document,"sensor_v2.config");const auto& d=object(wire);
    constexpr std::array<std::string_view,12> keys{"latency_s","maximum_age_s","dropout_probability","sample_period_s","maximum_read_period_s",
        "acceleration_bias","gyro_bias","acceleration_std","gyro_std","encoder_std","torque_std","imu_enabled"};
    exact(d,keys,"sensor_v2.config");spatial::SensorConfigV2 c;
    const auto n=[&](std::string_view key){return number(required(d,key));};
    c.latency_s=n("latency_s");c.maximum_age_s=n("maximum_age_s");c.dropout_probability=n("dropout_probability");
    c.sample_period_s=n("sample_period_s");c.maximum_read_period_s=n("maximum_read_period_s");
    const auto vector=[&](std::string_view key,auto& target){const auto& a=array(required(d,key));
        if(a.size()!=target.size())throw std::invalid_argument("sensor_v2.config: vector width must be three");
        for(std::size_t i=0;i<a.size();++i)target[i]=number(a[i]);};
    vector("acceleration_bias",c.acceleration_bias);vector("gyro_bias",c.gyro_bias);
    vector("acceleration_std",c.acceleration_std);vector("gyro_std",c.gyro_std);
    c.encoder_std=n("encoder_std");c.torque_std=n("torque_std");c.imu_enabled=boolean(required(d,"imu_enabled"));c.validate();return c;
}
WireObject encode_sensor_state_v2(const spatial::SensorStateV2& s) {
    validate_sensor_state_v2_shape(s);
    WireArray queue;queue.reserve(s.queue.size());for(const auto& value:s.queue)queue.emplace_back(value.as_wire());
    const auto optional=[](std::optional<double>x){return x?Wire(*x):Wire(nullptr);};
    return {{"schema",std::int64_t{2}},{"queue",std::move(queue)},{"last_time",optional(s.last_time)},{"delivery_time",optional(s.delivery_time)},
        {"startup",s.startup?Wire(s.startup->as_wire()):Wire(nullptr)},{"cursor",static_cast<std::int64_t>(s.cursor)},
        {"samples_attempted",static_cast<std::int64_t>(s.samples_attempted)},{"samples_dropped",static_cast<std::int64_t>(s.samples_dropped)}};
}
spatial::SensorStateV2 decode_sensor_state_v2(const bio::Value& document) {
    const auto wire=vread::wire_from_value(document,"sensor_v2.state");const auto& d=object(wire);
    constexpr std::array<std::string_view,8> keys{"schema","queue","last_time","delivery_time","startup","cursor","samples_attempted","samples_dropped"};
    exact(d,keys,"sensor_v2.state");
    if(integer(required(d,"schema"))!=2)throw std::invalid_argument("sensor_v2.state: unsupported schema");
    spatial::SensorStateV2 s;
    const auto optional=[&](std::string_view key){const auto& value=required(d,key);return is_none(value)?std::nullopt:std::optional(number(value));};
    s.last_time=optional("last_time");s.delivery_time=optional("delivery_time");
    const auto count=[&](std::string_view key){const auto value=integer(required(d,key));
        if(value<0||std::cmp_greater(value,std::numeric_limits<std::size_t>::max()))throw std::invalid_argument("sensor_v2.state: invalid counter");
        return static_cast<std::size_t>(value);};
    s.cursor=count("cursor");s.samples_attempted=count("samples_attempted");s.samples_dropped=count("samples_dropped");
    if(!is_none(required(d,"startup")))s.startup=spatial::SensorObservationV2::from_wire(object(required(d,"startup")));
    for(const auto& value:array(required(d,"queue")))s.queue.push_back(spatial::SensorObservationV2::from_wire(object(value)));
    validate_sensor_state_v2_shape(s);
    return s;
}
spatial::NoiseTapeV2 decode_noise_tape_v2(const bio::Value& noise,const bio::Value& dropout) {
    spatial::NoiseTapeV2 tape;
    for(const auto& value:bio::as_array(noise,"noise_v2.noise")){
        const auto& row=bio::as_array(value,"noise_v2.row");if(row.size()!=11)throw std::invalid_argument("noise_v2.row: expected eleven channels");
        std::array<double,11> parsed{};for(std::size_t i=0;i<11;++i)parsed[i]=bio::as_double(row[i],"noise_v2.channel");tape.noise.push_back(parsed);}
    for(const auto& value:bio::as_array(dropout,"noise_v2.dropout_uniform"))tape.dropout_uniform.push_back(bio::as_double(value,"noise_v2.dropout_uniform"));
    tape.validate();return tape;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (noise, dropout_uniform) mirrors the capture document's pair order at the single call site
NoiseTape decode_noise_tape(const bio::Value &noise,
                            const bio::Value &dropout_uniform) {
    NoiseTape result;
    const auto &rows =
        vread::sequence(vread::Node{&noise}, "noise_tape.noise");
    result.noise.reserve(rows.size());
    for (const bio::Value &row : rows) {
        // vread::vector keeps the same sequence/finite checks as
        // vread::fixed without instantiating a 72-byte pass-by-value
        // return for N=9 (same bound as the binding's parse_tape).
        const auto values =
            vread::vector(vread::Node{&row}, "noise_tape.noise");
        if (values.size() != 9)
            vread::invalid("noise_tape.noise", "incorrect sequence width");
        std::array<double, 9> parsed{};
        std::copy_n(values.begin(), parsed.size(), parsed.begin());
        result.noise.push_back(parsed);
    }
    result.dropout_uniform = vread::vector(vread::Node{&dropout_uniform},
                                           "noise_tape.dropout_uniform");
    result.validate();
    return result;
}

SensorState decode_sensor_state(const bio::Value &document) {
    const vread::Dict d{.value = &vread::mapping(vread::Node{&document},
                                                "sensor_state"),
                        .path = "sensor_state"};
    vread::exact(d, vread::keys("queue", "last_time", "delivery_time",
                                "startup", "cursor", "samples_attempted",
                                "samples_dropped"));
    SensorState s;
    s.last_time = vread::optional_real(d["last_time"], d.child("last_time"));
    s.delivery_time =
        vread::optional_real(d["delivery_time"], d.child("delivery_time"));
    const auto count = [&d](const char *key) {
        const auto v = vread::integer(d[key], d.child(key));
        if (v < 0 ||
            std::cmp_greater(v, std::numeric_limits<std::size_t>::max()))
            vread::invalid(d.child(key),
                           "must be a representable nonnegative count");
        return static_cast<std::size_t>(v);
    };
    s.cursor = count("cursor");
    s.samples_attempted = count("samples_attempted");
    s.samples_dropped = count("samples_dropped");
    const vread::Node startup = d["startup"];
    if (!vread::is_none(startup))
        s.startup = SensorObservation::from_wire(object(
            vread::wire_from_value(*startup.value, d.child("startup"))));
    for (const bio::Value &item :
         vread::sequence(d["queue"], d.child("queue")))
        s.queue.push_back(SensorObservation::from_wire(object(
            vread::wire_from_value(item, d.child("queue")))));
    return s;
}

ResearchConfig decode_research_config(const bio::Value &document) {
    if(const auto* schema=bio::find_member(bio::as_object(document,"research"),"sensor_schema_version")) {
        if(bio::as_int64(*schema,"research.sensor_schema_version")!=2)
            throw std::invalid_argument("research: unsupported sensor schema");
        const vread::Dict d{.value=&vread::mapping(vread::Node{&document},"research"),.path="research"};
        vread::exact(d,vread::keys("sensor_schema_version","timestep_s","control_steps","delay_steps","max_steps","sensor_steps","record_decimation",
            "stop_on_model_violation","maximum_energy_residual_ratio","wheelie_persistence_s","track_length_m","start_position_m","sensors_v2",
            "startup_control","rider_program","demand_program","capture_mode","seed_decimal","metadata","raw_initial_v2"));
        ResearchConfig c;
        c.timestep_s=vread::finite_real(d["timestep_s"],d.child("timestep_s"));c.control_steps=vread::integer(d["control_steps"],d.child("control_steps"));
        c.delay_steps=vread::integer(d["delay_steps"],d.child("delay_steps"));c.max_steps=vread::integer(d["max_steps"],d.child("max_steps"));
        c.sensor_steps=vread::integer(d["sensor_steps"],d.child("sensor_steps"));c.record_decimation=vread::integer32(d["record_decimation"],d.child("record_decimation"));
        c.stop_on_model_violation=vread::boolean(d["stop_on_model_violation"],d.child("stop_on_model_violation"));
        c.maximum_energy_residual_ratio=vread::finite_real(d["maximum_energy_residual_ratio"],d.child("maximum_energy_residual_ratio"));
        c.wheelie_persistence_s=vread::finite_real(d["wheelie_persistence_s"],d.child("wheelie_persistence_s"));
        c.track_length_m=vread::finite_real(d["track_length_m"],d.child("track_length_m"));c.start_position_m=vread::finite_real(d["start_position_m"],d.child("start_position_m"));
        c.sensors_v2=decode_sensor_config_v2(*vread::field(d,"sensors_v2").value);
        if(c.timestep_s<=0.||std::abs(c.sensors_v2->sample_period_s/c.timestep_s-static_cast<double>(c.sensor_steps))>1e-8||
            static_cast<double>(c.control_steps)*c.timestep_s>c.sensors_v2->maximum_read_period_s)
            throw std::invalid_argument("research.sensors_v2: periods do not match the integer control clock");
        c.startup_control=rider::decode_intent_control(*vread::field(d,"startup_control").value,d.child("startup_control"));
        c.programs=ResearchPrograms::from_wire(vread::wire_from_value(*vread::field(d,"rider_program").value,d.child("rider_program")),
            vread::wire_from_value(*vread::field(d,"demand_program").value,d.child("demand_program")));
        const auto& mode=vread::string(d["capture_mode"],d.child("capture_mode"));
        if(mode=="off")c.capture_mode=ResearchCaptureMode::off;else if(mode=="full")c.capture_mode=ResearchCaptureMode::full;
        else if(mode=="legacy")c.capture_mode=ResearchCaptureMode::legacy;else if(mode=="verify")c.capture_mode=ResearchCaptureMode::verify;
        else throw std::invalid_argument("research.capture_mode: unsupported mode");
        c.seed_decimal=vread::string(d["seed_decimal"],d.child("seed_decimal"));c.metadata=*vread::field(d,"metadata").value;
        const auto& raw=*vread::field(d,"raw_initial_v2").value;
        if(!bio::is_null(raw))c.raw_initial_v2=spatial::SensorObservationV2::from_wire(object(vread::wire_from_value(raw,"research.raw_initial_v2")));
        return c;
    }
    const vread::Dict d{.value = &vread::mapping(vread::Node{&document},
                                                "research"),
                        .path = "research"};
    vread::exact(d, vread::keys("timestep_s", "control_steps",
        "delay_steps", "max_steps", "sensor_steps", "record_decimation",
        "stop_on_model_violation", "maximum_energy_residual_ratio",
        "wheelie_persistence_s", "track_length_m", "start_position_m",
        "sensors", "geometry", "startup_control", "rider_program",
        "demand_program"),
        vread::keys("capture_mode"));
    ResearchConfig c;
    if (d.contains("capture_mode")) {
        const auto &mode = vread::string(d["capture_mode"],
                                         d.child("capture_mode"));
        if (mode == "full") c.capture_mode = ResearchCaptureMode::full;
        else if (mode == "off") c.capture_mode = ResearchCaptureMode::off;
        else if (mode == "verify")
            c.capture_mode = ResearchCaptureMode::verify;
        else if (mode != "legacy")
            vread::invalid(d.child("capture_mode"),
                           "expected off, full, legacy or verify");
    }
    c.timestep_s = vread::finite_real(d["timestep_s"], d.child("timestep_s"));
    c.control_steps =
        vread::integer(d["control_steps"], d.child("control_steps"));
    c.delay_steps = vread::integer(d["delay_steps"], d.child("delay_steps"));
    c.max_steps = vread::integer(d["max_steps"], d.child("max_steps"));
    c.sensor_steps =
        vread::integer(d["sensor_steps"], d.child("sensor_steps"));
    c.record_decimation = vread::integer32(d["record_decimation"],
                                           d.child("record_decimation"));
    c.stop_on_model_violation = vread::boolean(
        d["stop_on_model_violation"], d.child("stop_on_model_violation"));
    c.maximum_energy_residual_ratio = vread::finite_real(
        d["maximum_energy_residual_ratio"],
        d.child("maximum_energy_residual_ratio"));
    c.wheelie_persistence_s = vread::finite_real(
        d["wheelie_persistence_s"], d.child("wheelie_persistence_s"));
    c.track_length_m =
        vread::finite_real(d["track_length_m"], d.child("track_length_m"));
    c.start_position_m = vread::finite_real(d["start_position_m"],
                                            d.child("start_position_m"));
    const vread::Node sensors_doc = vread::field(d, "sensors");
    c.sensors = decode_sensors(*sensors_doc.value);
    const auto &sensors =
        vread::mapping(sensors_doc, d.child("sensors"));
    const double sample_period = vread::finite_real(
        vread::Node{bio::find_member(sensors, "sample_period_s")},
        "sensors.sample_period_s");
    if (c.timestep_s <= 0. ||
        std::abs(sample_period / c.timestep_s -
                 static_cast<double>(c.sensor_steps)) > 1e-8)
        vread::invalid("research.sensor_steps",
                       "must match sensors.sample_period_s / timestep_s");
    c.startup_control = rider::decode_intent_control(
        *vread::field(d, "startup_control").value,
        d.child("startup_control"));
    c.programs = ResearchPrograms::from_wire(
        vread::wire_from_value(*vread::field(d, "rider_program").value,
                             d.child("rider_program")),
        vread::wire_from_value(*vread::field(d, "demand_program").value,
                             d.child("demand_program")));
    const auto g = vread::section(d, "geometry");
    vread::exact(g, vread::keys("vertices", "front_radius", "rear_radius",
                                "root_x_qpos", "root_x_dof",
                                "root_pitch_qpos", "root_pitch_dof"));
    for (const bio::Value &row :
         vread::sequence(vread::field(g, "vertices"), g.child("vertices"))) {
        const auto v =
            vread::fixed<2>(vread::Node{&row}, g.child("vertices"));
        c.geometry.x.push_back(v[0]);
        c.geometry.z.push_back(v[1]);
    }
    c.geometry.front_radius = vread::finite_real(
        vread::field(g, "front_radius"), g.child("front_radius"));
    c.geometry.rear_radius = vread::finite_real(
        vread::field(g, "rear_radius"), g.child("rear_radius"));
    const auto address = [&g](const char *key) {
        const auto v = vread::integer(vread::field(g, key), g.child(key));
        if (v < 0 ||
            std::cmp_greater(v, std::numeric_limits<std::size_t>::max()))
            vread::invalid(g.child(key), "invalid model address");
        return static_cast<std::size_t>(v);
    };
    c.geometry.root_x_qpos = address("root_x_qpos");
    c.geometry.root_x_dof = address("root_x_dof");
    c.geometry.root_pitch_qpos = address("root_pitch_qpos");
    c.geometry.root_pitch_dof = address("root_pitch_dof");
    return c;
}

ResearchBootstrap read_research_bootstrap(const fs::path &directory) {
    std::error_code ec;
    if (!fs::is_directory(directory, ec))
        io_invalid(directory.string(), "not a research bootstrap directory");
    const bio::Value document = resolve_bootstrap_document(
        bio::parse_json(
            text(read_file(directory / "research.json", "research.json"))),
        directory);
    const vread::Dict root{.value = &vread::mapping(vread::Node{&document},
                                                   "research_bootstrap"),
                           .path = "research_bootstrap"};
    vread::exact(root, vread::keys("research", "noise", "dropout_uniform",
                                   "sensor_state"));
    ResearchBootstrap out;
    // The adapter ctor's ordinal decode order: research config, noise
    // tape, sensor state, then the ride composition.
    out.config =
        decode_research_config(*vread::field(root, "research").value);
    if(out.config.sensors_v2) {
        out.tape_v2=decode_noise_tape_v2(*vread::field(root,"noise").value,*vread::field(root,"dropout_uniform").value);
        out.startup_v2=decode_sensor_state_v2(*vread::field(root,"sensor_state").value);
    } else {
        out.tape = decode_noise_tape(*vread::field(root, "noise").value,*vread::field(root, "dropout_uniform").value);
        out.startup = decode_sensor_state(*vread::field(root, "sensor_state").value);
    }
    out.ride = read_bootstrap(directory);
    return out;
}

std::unique_ptr<NativeResearchRuntime>
make_research(ResearchBootstrap bootstrap) {
    auto ride = std::make_unique<NativeRideRuntime>(std::move(bootstrap.ride));
    if(bootstrap.config.sensors_v2) {
        if(!bootstrap.tape_v2||!bootstrap.startup_v2)throw std::invalid_argument("research bootstrap: missing V2 tape/state");
        return std::make_unique<NativeResearchRuntime>(std::move(ride),std::move(bootstrap.config),std::move(*bootstrap.tape_v2),std::move(*bootstrap.startup_v2));
    }
    return std::make_unique<NativeResearchRuntime>(
        std::move(ride), std::move(bootstrap.config),
        std::move(bootstrap.tape), std::move(bootstrap.startup));
}

std::unique_ptr<NativeResearchRuntime>
make_research(std::unique_ptr<ResearchBootstrap> bootstrap) {
    auto ride = std::make_unique<NativeRideRuntime>(std::move(bootstrap->ride));
    if(bootstrap->config.sensors_v2) {
        if(!bootstrap->tape_v2||!bootstrap->startup_v2)throw std::invalid_argument("research bootstrap: missing V2 tape/state");
        return std::make_unique<NativeResearchRuntime>(std::move(ride),std::move(bootstrap->config),std::move(*bootstrap->tape_v2),std::move(*bootstrap->startup_v2));
    }
    return std::make_unique<NativeResearchRuntime>(
        std::move(ride), std::move(bootstrap->config),
        std::move(bootstrap->tape), std::move(bootstrap->startup));
}

} // namespace runtime
