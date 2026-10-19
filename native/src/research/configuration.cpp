#include "environment.hpp"
#include "../diag.hpp"
#include "../configuration/load.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../runtime/setup_values.hpp"
#include "../terrain/trackfile.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <ranges>
#include <stdexcept>

namespace bike::research {
namespace {
using namespace runtime::setupvalue;

void nonnegative(double value, std::string_view name, bool positive = false) {
    if (!std::isfinite(value) || (positive ? value <= 0. : value < 0.))
        throw std::invalid_argument(std::string(name) + (positive ? " must be positive and finite" : " must be nonnegative and finite"));
}
void keys(const Value& value, std::initializer_list<std::string_view> allowed, std::string_view path) {
    for (const auto& [name, ignored] : io::as_object(value, path)) {
        (void)ignored;
        if (std::ranges::find(allowed, name) == allowed.end())
            throw std::invalid_argument(std::string(path) + ": unknown field '" + name + "'");
    }
}
double number(const Value& value, std::string_view key, double fallback) {
    if (const auto* item = find(value, key)) return io::as_double(*item, key);
    return fallback;
}
bool boolean(const Value& value, std::string_view key, bool fallback) {
    if (const auto* item = find(value, key)) return io::as_bool(*item, key);
    return fallback;
}
std::filesystem::path path(const Value& value, std::string_view key, const std::filesystem::path& base) {
    const auto name = io::as_string(field(value, key), key);
    if (name.empty() || name.find('\0') != std::string::npos)
        throw std::invalid_argument(std::string(key) + " must be a nonempty path");
    return std::filesystem::weakly_canonical(base / std::filesystem::path(name));
}
ExperimentConfig experiment_from(const Value& value, ExperimentConfig out) {
    keys(value, {"control_period_s", "actuator_delay_s", "duration_s", "seed", "seed_decimal", "record_decimation",
        "wheelie_persistence_s", "maximum_energy_residual_ratio", "stop_on_model_violation", "async_telemetry"}, "experiment");
    out.control_period_s = number(value, "control_period_s", out.control_period_s);
    out.actuator_delay_s = number(value, "actuator_delay_s", out.actuator_delay_s);
    out.duration_s = number(value, "duration_s", out.duration_s);
    out.wheelie_persistence_s = number(value, "wheelie_persistence_s", out.wheelie_persistence_s);
    out.maximum_energy_residual_ratio = number(value, "maximum_energy_residual_ratio", out.maximum_energy_residual_ratio);
    out.stop_on_model_violation = boolean(value, "stop_on_model_violation", true);
    out.async_telemetry = boolean(value, "async_telemetry", false);
    if (const auto* seed = find(value, "seed")) {
        const auto* integer_value = std::get_if<io::Number>(&seed->data);
        if (integer_value == nullptr) throw std::invalid_argument("experiment seed must be a nonnegative integer");
        out.seed_decimal = validate_seed(integer_value->lexeme);
    }
    if (const auto* seed = find(value, "seed_decimal")) {
        const auto chosen = validate_seed(io::as_string(*seed, "experiment.seed_decimal"));
        if (find(value, "seed") && chosen != out.seed_decimal)
            throw std::invalid_argument("experiment seed and seed_decimal disagree");
        out.seed_decimal = chosen;
    }
    if (const auto* decimation = find(value, "record_decimation")) {
        const auto count = io::as_int64(*decimation, "experiment.record_decimation");
        if (count < 1 || count > std::numeric_limits<int>::max())
            throw std::invalid_argument("record_decimation must be a representable positive integer");
        out.record_decimation = static_cast<int>(count);
    }
    out.validate();
    return out;
}
SensorConfig sensors_from(const Value& value, SensorConfig out) {
    keys(value, {"latency_s", "acceleration_std_mps2", "gyro_std_rad_s", "encoder_std_rad_s", "torque_std_nm",
        "imu_enabled", "sample_period_s", "acceleration_bias_mps2", "gyro_bias_rad_s", "dropout_probability", "maximum_age_s"}, "sensors");
    out.delivery.latency_s = number(value, "latency_s", out.delivery.latency_s);
    out.delivery.maximum_age_s = number(value, "maximum_age_s", out.delivery.maximum_age_s);
    out.delivery.dropout_probability = number(value, "dropout_probability", 0.);
    out.delivery.gyro_bias = number(value, "gyro_bias_rad_s", 0.);
    out.delivery.imu_enabled = boolean(value, "imu_enabled", true);
    out.noise.acceleration_std_mps2 = number(value, "acceleration_std_mps2", out.noise.acceleration_std_mps2);
    out.noise.gyro_std_rad_s = number(value, "gyro_std_rad_s", out.noise.gyro_std_rad_s);
    out.noise.encoder_std_rad_s = number(value, "encoder_std_rad_s", out.noise.encoder_std_rad_s);
    out.noise.torque_std_nm = number(value, "torque_std_nm", out.noise.torque_std_nm);
    out.sample_period_s = number(value, "sample_period_s", out.sample_period_s);
    if (const auto* bias = find(value, "acceleration_bias_mps2")) {
        const auto& values = io::as_array(*bias, "sensors.acceleration_bias_mps2");
        if (values.size() != 3) throw std::invalid_argument("acceleration bias must have width 3");
        for (std::size_t i = 0; i < 3; ++i)
            out.delivery.acceleration_bias[i] = io::as_double(values[i], "sensors.acceleration_bias_mps2");
    }
    out.validate();
    return out;
}
random::RiderRandomSpec random_from(const Value& value) {
    keys(value, {"mass_kg", "height_m", "torso_lean_rad", "pelvis_pitch_rad", "human_torque_nm"}, "rider_random");
    random::RiderRandomSpec out;
    const auto range = [&](std::string_view name, std::array<double, 2>& target) {
        if (const auto* item = find(value, name)) {
            const auto& values = io::as_array(*item, name);
            if (values.size() != 2) throw std::invalid_argument(std::string(name) + " must be a (low, high) pair");
            target = {io::as_double(values[0], name), io::as_double(values[1], name)};
        }
    };
    range("mass_kg", out.mass_kg); range("height_m", out.height_m);
    range("torso_lean_rad", out.torso_lean_rad); range("pelvis_pitch_rad", out.pelvis_pitch_rad);
    range("human_torque_nm", out.human_torque_nm);
    out.validate();
    return out;
}
} // namespace

std::string validate_seed(std::string_view seed) {
    if (seed.empty() || !std::ranges::all_of(seed, [](char c) { return c >= '0' && c <= '9'; }))
        throw std::invalid_argument("seed must be a nonnegative decimal integer");
    const auto first = seed.find_first_not_of('0');
    return first == std::string_view::npos ? "0" : std::string(seed.substr(first));
}
void ExperimentConfig::validate() const {
    nonnegative(control_period_s, "control_period_s", true);
    nonnegative(duration_s, "duration_s", true);
    nonnegative(maximum_energy_residual_ratio, "maximum_energy_residual_ratio", true);
    nonnegative(actuator_delay_s, "actuator_delay_s");
    nonnegative(wheelie_persistence_s, "wheelie_persistence_s");
    (void)validate_seed(seed_decimal);
    if (record_decimation < 1) throw std::invalid_argument("record_decimation must be a positive integer");
}
void SensorConfig::validate() const {
    delivery.validate(); noise.validate();
    nonnegative(sample_period_s, "sensor period", true);
}
SensorConfig SensorConfig::ideal() {
    SensorConfig out;
    out.delivery.latency_s = 0.;
    out.noise = {.acceleration_std_mps2 = 0., .gyro_std_rad_s = 0., .encoder_std_rad_s = 0., .torque_std_nm = 0.};
    return out;
}
std::int64_t integer_steps(double seconds, double dt, std::string_view name) {
    nonnegative(seconds, name); nonnegative(dt, "physics timestep", true);
    const double ratio = seconds / dt;
    constexpr double limit = 4503599627370496.;
    if (!std::isfinite(ratio) || ratio > limit)
        throw std::invalid_argument(std::string(name) + " exceeds the exact step range");
    // Within 1e-8 of an integer the tie rule cannot affect acceptance.
    const double rounded = std::round(ratio);
    if (std::abs(ratio - rounded) > 1e-8)
        throw std::invalid_argument(std::string(name) + " must be an integer multiple of the physics timestep");
    return static_cast<std::int64_t>(rounded);
}
terrain::HeightFieldSpec research_field(const terrain::TrackSpec& track, double resolution) {
    nonnegative(resolution, "road resolution", true);
    auto field = terrain::HeightFieldSpec::for_track(track);
    const double intervals = field.track_length_m() / resolution;
    const double count = std::round(intervals);
    if (!std::isfinite(intervals) || count < 2. || count > 1000000. || std::abs(intervals - count) > 1e-7)
        throw std::invalid_argument("road resolution must divide the field length into 2..1000000 intervals");
    field.ncol = static_cast<int>(count) + 1;
    field.validate();
    return field;
}
terrain::TrackSpec resolve_research_track(const std::optional<std::filesystem::path>& track_file,
    std::string_view scenario, const std::optional<std::filesystem::path>& generator_file, std::string_view seed_decimal) {
    if (track_file) return terrain::load_track(*track_file); // authored seed and geometry win
    const auto seed = validate_seed(seed_decimal);
    if (generator_file && scenario != "generated" && scenario != "alpine")
        throw std::invalid_argument("generator_spec_file requires scenario generated or alpine");
    if (scenario == "generated") {
        const auto spec = generator_file ? terrain::TerrainGenSpec::from_dict(configuration::detail::parse_toml_file(*generator_file))
            : terrain::TerrainGenSpec{};
        return terrain::generate_track(spec, seed, "generated_" + seed);
    }
    if (scenario == "alpine") {
        const auto spec = generator_file ? terrain::AlpineGenSpec::from_dict(configuration::detail::parse_toml_file(*generator_file))
            : terrain::AlpineGenSpec{};
        return terrain::generate_alpine_track(spec, seed, "alpine_" + seed);
    }
    return terrain::build_research_track(scenario, seed);
}

configuration::SimulationPhysicsConfig resolve_research_physics(
    const configuration::SimulationPhysicsConfig& defaults,
    const std::optional<std::filesystem::path>& file, const io::Value& explicit_overrides) {
    auto overrides = file ? configuration::detail::parse_toml_file(*file) : Value{Object{}};
    if (file) configuration::detail::resolve_config_paths(overrides, std::filesystem::absolute(*file).parent_path());
    auto base = configuration::detail::plain_document(configuration::to_value(defaults));
    // A material table selects its complete type. Carrying the old TireSpec
    // fields into a TabulatedTireSpec would reject a valid source document.
    if (file)
        if (const auto* tires = find(overrides, "tires")) {
            auto base_tires = field(base, "tires");
            for (const auto side : {"front", "rear"})
                if (const auto* wheel = find(*tires, side); wheel && find(*wheel, "material")) {
                    auto base_wheel = field(base_tires, side);
                    auto& members = std::get<Object>(base_wheel.data);
                    std::erase_if(members, [](const auto& member) { return member.first == "material"; });
                    set(base_tires, side, std::move(base_wheel));
                }
            set(base, "tires", std::move(base_tires));
        }
    overrides = configuration::detail::merge_values(overrides, explicit_overrides);
    std::string transmission = defaults.drive.transmission_model;
    if (const auto* drive = find(overrides, "drive"))
        if (const auto* model = find(*drive, "transmission_model"))
            transmission = io::as_string(*model, "drive.transmission_model");
    if (!find(overrides, "timestep_s") && transmission != defaults.drive.transmission_model) {
        if (transmission == "ideal_mid_drive" || transmission == "geometric_ideal_mid_drive") set(overrides, "timestep_s", real(.0005));
        else if (transmission == "elastic_chain") set(overrides, "timestep_s", real(.000125));
    }
    if (!find(overrides, "closure_time_constant_s"))
        if (const auto* dt = find(overrides, "timestep_s"))
            if (io::as_double(*dt, "timestep_s") * 2. > defaults.closure_time_constant_s)
                set(overrides, "closure_time_constant_s", real(.0025));
    return configuration::detail::construct_config(configuration::detail::merge_values(base, overrides));
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
ResearchRequest read_research_request(const std::filesystem::path& file) {
    try {
        const auto document = configuration::detail::parse_toml_file(file);
        keys(document, {"request_schema", "physics_file", "track_file", "scenario", "generator_spec_file", "model_mode", "rider_variant", "speed_kph",
            "seed_decimal", "use_equilibrium_cache", "initial_state_file", "overrides", "experiment", "sensors",
            "control_period_s", "actuator_delay_s", "sensor_delay_s", "duration_s", "road_resolution_m",
            "capture_mode", "demand_file", "demand_nm", "rider_program_file", "demand_program", "rider_program", "rider_random",
            "rider_seed_decimal", "check_policy"}, "research request");
        if (io::as_int64(field(document, "request_schema"), "request_schema") != 1)
            throw std::invalid_argument("unsupported request_schema");
        ResearchRequest out;
        // File-driven research requests mirror the CLI defaults. The typed
        // ExperimentConfig/SensorConfig retain their dataclass defaults.
        out.experiment.record_decimation = 80;
        out.sensors.sample_period_s = .005;
        const auto base = std::filesystem::absolute(file).parent_path();
        out.setup.origin = std::filesystem::absolute(file).string();
        out.setup.asset_root = base;
        if (const auto* value = find(document, "experiment")) out.experiment = experiment_from(*value, out.experiment);
        if (const auto* value = find(document, "sensors")) out.sensors = sensors_from(*value, out.sensors);
        if (const auto* seed = find(document, "seed_decimal")) {
            const auto value = validate_seed(io::as_string(*seed, "seed_decimal"));
            if (const auto* experiment = find(document, "experiment"))
                if ((find(*experiment, "seed") || find(*experiment, "seed_decimal")) && value != out.experiment.seed_decimal)
                    throw std::invalid_argument("setup and experiment seeds disagree");
            out.experiment.seed_decimal = value;
        }
        out.experiment.control_period_s = number(document, "control_period_s", out.experiment.control_period_s);
        out.experiment.actuator_delay_s = number(document, "actuator_delay_s", out.experiment.actuator_delay_s);
        out.experiment.duration_s = number(document, "duration_s", out.experiment.duration_s);
        out.sensors.delivery.latency_s = number(document, "sensor_delay_s", out.sensors.delivery.latency_s);
        out.setup.seed_decimal = out.experiment.seed_decimal;
        const auto track_file = find(document, "track_file") ? std::optional(path(document, "track_file", base)) : std::nullopt;
        const auto generator_file = find(document, "generator_spec_file") ? std::optional(path(document, "generator_spec_file", base)) : std::nullopt;
        out.setup.track = resolve_research_track(track_file, find(document, "scenario")
            ? io::as_string(field(document, "scenario"), "scenario") : "rough_uphill", generator_file, out.setup.seed_decimal);
        out.road_resolution_m = number(document, "road_resolution_m", .005);
        out.setup.model.field = research_field(out.setup.track, out.road_resolution_m);
        if (const auto* mode = find(document, "model_mode")) {
            const auto& name = io::as_string(*mode, "model_mode");
            if (name != "research" && name != "ride") throw std::invalid_argument("research model_mode must be research or ride");
        }
        out.setup.model.mode = "ride";
        out.setup.model.rider = model::rider_from_args(find(document, "rider_variant")
            ? io::as_string(field(document, "rider_variant"), "rider_variant") : "articulated_planar");
        auto physics = configuration::load_physics(path(document, "physics_file", base));
        if (const auto* value = find(document, "overrides")) {
            auto owned = *value;
            configuration::detail::resolve_config_paths(owned, base);
            physics = resolve_research_physics(physics, std::nullopt, owned);
        }
        if (find(document, "speed_kph")) physics.initial_speed_mps = number(document, "speed_kph", 0.) / 3.6;
        physics.equilibrium_cache_enabled = boolean(document, "use_equilibrium_cache", false);
        if (find(document, "initial_state_file")) {
            const auto state_file = path(document, "initial_state_file", base);
            const auto bytes = io::read_numpy_bytes(state_file);
            std::string source;
            source.reserve(bytes.size());
            for (auto const byte : bytes) source.push_back(static_cast<char>(std::to_integer<unsigned char>(byte)));
            const auto first = source.find_first_not_of(" \t\r\n");
            bool transfer = false;
            if (first != std::string::npos && source[first] == '{')
                transfer = find(io::parse_json(source), "contract_sha256") != nullptr;
            if (transfer) {
                out.setup.initial_state.emplace(std::move(source));
                physics.equilibrium_state_file.reset();
            } else physics.equilibrium_state_file = state_file.string();
        }
        configuration::post_init(physics);
        out.setup.model.physics = std::move(physics);
        if (const auto* mode = find(document, "capture_mode")) {
            const auto& value = io::as_string(*mode, "capture_mode");
            if (value == "off") out.capture_mode = runtime::ResearchCaptureMode::off;
            else if (value == "full") out.capture_mode = runtime::ResearchCaptureMode::full;
            else if (value == "verify") out.capture_mode = runtime::ResearchCaptureMode::verify;
            else if (value != "legacy") throw std::invalid_argument("capture_mode must be off, full, legacy or verify");
        }
        if (const auto* policy = find(document, "check_policy")) {
            const auto& value = io::as_string(*policy, "check_policy");
            if (value != "strict" && value != "diagnostic") throw std::invalid_argument("check_policy must be strict or diagnostic");
            out.experiment.stop_on_model_violation = value == "strict";
        }
        if (find(document, "rider_program_file") && find(document, "rider_program"))
            throw std::invalid_argument("choose one rider program source");
        if (find(document, "demand_file") && find(document, "demand_program"))
            throw std::invalid_argument("choose one demand program source");
        if (find(document, "demand_nm") && (find(document, "demand_file") || find(document, "demand_program")))
            throw std::invalid_argument("choose one constant or scripted demand source");
        const auto program_document = [&](std::string_view filename, std::string_view inline_key) {
            if (find(document, filename)) return configuration::detail::parse_toml_file(path(document, filename, base));
            if (const auto* value = find(document, inline_key)) return *value;
            return Value{};
        };
        out.programs = programs_from_documents(program_document("rider_program_file", "rider_program"),
                                               program_document("demand_file", "demand_program"));
        if (const auto* demand = find(document, "demand_nm"))
            out.programs.demand_frames = constant_demand(io::as_double(*demand, "demand_nm")).demand_frames;
        if (const auto* value = find(document, "rider_random")) out.rider_random = random_from(*value);
        if (const auto* value = find(document, "rider_seed_decimal"))
            out.rider_seed_decimal = validate_seed(io::as_string(*value, "rider_seed_decimal"));
        out.experiment.validate(); out.sensors.validate();
        return out;
    } catch (const std::invalid_argument& error) {
        throw std::invalid_argument(file.string() + ": " + error.what());
    }
}
NATIVE_DIAG_POP

io::Value experiment_value(const ExperimentConfig& c) {
    return Value{Object{{"control_period_s", real(c.control_period_s)}, {"actuator_delay_s", real(c.actuator_delay_s)},
        {"duration_s", real(c.duration_s)}, {"seed", Value{io::Number{validate_seed(c.seed_decimal)}}},
        {"record_decimation", integer(c.record_decimation)}, {"wheelie_persistence_s", real(c.wheelie_persistence_s)},
        {"maximum_energy_residual_ratio", real(c.maximum_energy_residual_ratio)},
        {"stop_on_model_violation", Value{c.stop_on_model_violation}}, {"async_telemetry", Value{c.async_telemetry}}}};
}
io::Value sensor_config_value(const SensorConfig& c) {
    return Value{Object{{"latency_s", real(c.delivery.latency_s)}, {"acceleration_std_mps2", real(c.noise.acceleration_std_mps2)},
        {"gyro_std_rad_s", real(c.noise.gyro_std_rad_s)}, {"encoder_std_rad_s", real(c.noise.encoder_std_rad_s)},
        {"torque_std_nm", real(c.noise.torque_std_nm)}, {"imu_enabled", Value{c.delivery.imu_enabled}},
        {"sample_period_s", real(c.sample_period_s)}, {"acceleration_bias_mps2", reals(c.delivery.acceleration_bias)},
        {"gyro_bias_rad_s", real(c.delivery.gyro_bias)}, {"dropout_probability", real(c.delivery.dropout_probability)},
        {"maximum_age_s", real(c.delivery.maximum_age_s)}}};
}
} // namespace bike::research
