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
    out.tape =
        decode_noise_tape(*vread::field(root, "noise").value,
                          *vread::field(root, "dropout_uniform").value);
    out.startup =
        decode_sensor_state(*vread::field(root, "sensor_state").value);
    out.ride = read_bootstrap(directory);
    return out;
}

std::unique_ptr<NativeResearchRuntime>
make_research(ResearchBootstrap bootstrap) {
    auto ride = std::make_unique<NativeRideRuntime>(std::move(bootstrap.ride));
    return std::make_unique<NativeResearchRuntime>(
        std::move(ride), std::move(bootstrap.config),
        std::move(bootstrap.tape), std::move(bootstrap.startup));
}

std::unique_ptr<NativeResearchRuntime>
make_research(std::unique_ptr<ResearchBootstrap> bootstrap) {
    auto ride = std::make_unique<NativeRideRuntime>(std::move(bootstrap->ride));
    return std::make_unique<NativeResearchRuntime>(
        std::move(ride), std::move(bootstrap->config),
        std::move(bootstrap->tape), std::move(bootstrap->startup));
}

} // namespace runtime
