#include "application.hpp"
#include "../diag.hpp"
#include "../configuration/load.hpp"
#include "../model/rider_geometry.hpp"
#include "../runtime/setup_values.hpp"
#include "../terrain/trackfile.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>

namespace bike::cli {
namespace {
using namespace runtime::setupvalue;
std::optional<std::filesystem::path> file(const Arguments& args, std::string_view key) {
    const auto value = args.optional_text(key);
    return value ? std::optional(std::filesystem::path(*value)) : std::nullopt;
}
void positive(double x, std::string_view name) {
    if (!std::isfinite(x) || x <= 0.) throw ArgumentError(std::string(name) + " must be finite and positive");
}
int bounded_integer(const Arguments& args, std::string_view key, int fallback) {
    const auto value = args.integer(key, fallback);
    if (value < 1 || value > std::numeric_limits<int>::max()) throw ArgumentError(std::string(key) + " must be a positive representable integer");
    return static_cast<int>(value);
}
void add_number(Value& object, const Arguments& args, std::string_view option, std::string_view field_name) {
    if (args.has(option)) set(object, field_name, real(args.real(option)));
}
void add_text(Value& object, const Arguments& args, std::string_view option, std::string_view field_name) {
    if (args.has(option)) set(object, field_name, text(args.text(option)));
}
} // namespace

terrain::TrackSpec resolve_track(std::string_view name, std::optional<std::string> seed,
    std::optional<double> length) {
    if (seed) *seed = research::validate_seed(*seed);
    const auto names = terrain::available_presets();
    const bool preset = std::ranges::find(names, name) != names.end();
    const auto path = std::filesystem::path(name);
    if (!preset && (path.extension() == ".toml" || std::filesystem::is_regular_file(path)))
        return terrain::load_track(path, std::move(seed), length);
    if (name == "road_smooth" || name == "road_worn" || name == "road_broken") {
        auto spec = name == "road_smooth" ? terrain::road_smooth_spec() : name == "road_worn" ? terrain::road_worn_spec() : terrain::road_broken_spec();
        if (seed) spec.seed = *seed;
        return terrain::build_road(std::string(name), spec, length.value_or(terrain::DEFAULT_ROAD_LENGTH_M), {}, terrain::get_preset(name).description);
    }
    if (name == "alpine_scree" || name == "alpine_rocky") {
        if (length) throw ArgumentError("--length does not apply to '" + std::string(name) + "'");
        return name == "alpine_scree" ? terrain::alpine_scree(seed.value_or("41")) : terrain::alpine_rocky(seed.value_or("0"));
    }
    if (seed || length) throw ArgumentError("--seed/--length require a generated preset or track file");
    return terrain::get_preset(name);
}

namespace {
// The physics config, the setup request and the ride return each carry
// several KB; heap-owned helpers keep resolve_ride's frame inside the strict
// per-function bound.
// load_physics and resolve_physics return the multi-KB physics config by
// value; isolating their temporaries keeps ride_physics' own frame small.
[[gnu::noinline]] std::unique_ptr<configuration::SimulationPhysicsConfig> load_ride_physics(const Arguments& args) {
    return std::make_unique<configuration::SimulationPhysicsConfig>(
        args.has("physics_config") ? configuration::load_physics(args.text("physics_config")) : configuration::SimulationPhysicsConfig{});
}
[[gnu::noinline]] void resolve_physics_into(configuration::SimulationPhysicsConfig& physics, const Value& overrides) {
    physics = configuration::resolve_physics(overrides, physics);
}
std::unique_ptr<configuration::SimulationPhysicsConfig> ride_physics(Arguments& args) {
    Value overrides{Object{}}, drive{Object{}}, assist{Object{}};
    add_text(overrides, args, "physics", "physics_mode");
    add_text(overrides, args, "drive", "drive_mode");
    add_number(overrides, args, "timestep", "timestep_s");
    if (args.has("initial_speed")) set(overrides, "initial_speed_mps", real(args.real("initial_speed") / 3.6));
    add_number(drive, args, "human_torque", "human_torque_nm");
    add_number(assist, args, "assist_gain", "gain");
    if (args.has("crank_phase")) set(drive, "crank_phase_rad", real(args.real("crank_phase") * std::numbers::pi / 180.));
    if (!io::as_object(assist, "assist").empty()) set(drive, "assist", assist);
    if (!io::as_object(drive, "drive").empty()) set(overrides, "drive", drive);
    auto physics = load_ride_physics(args);
    resolve_physics_into(*physics, overrides);
    if (physics->physics_mode == "physical" && args.has("assist")) {
        set(assist, "mode", text(args.text("assist"))); set(drive, "assist", assist); set(overrides, "drive", drive);
        resolve_physics_into(*physics, overrides);
    }
    args.set("physics", physics->physics_mode); args.set("drive", physics->drive_mode.value_or("coast"));
    return physics;
}
std::unique_ptr<initialization::SetupRequest> ride_setup_request(Arguments const& args, const configuration::SimulationPhysicsConfig& physics,
    bool physical, bool experiment) {
    auto owner = std::make_unique<initialization::SetupRequest>();
    auto& request = *owner;
    request.origin = args.command; request.asset_root = std::filesystem::current_path();
    auto seed = args.optional_text("seed");
    // In research a frozen authored file retains its own terrain seed.
    if (experiment && std::filesystem::path(args.text("track")).extension() == ".toml") seed.reset();
    request.track = resolve_track(args.text("track", "road_worn"), seed, args.optional_real("length"));
    request.seed_decimal = research::validate_seed(args.text("seed", "0"));
    request.model.mode = "ride"; request.model.field = terrain::HeightFieldSpec::for_track(request.track);
    request.model.physics = physics;
    auto rider = model::rider_from_args(args.flag("no_rider") ? "none" : args.text("rider", "seated"), args.real("rider_mass", 80.), args.real("rider_height", 1.8), args.optional_real("rider_inseam"));
    request.model.rider = std::move(rider);
    request.model.include_rider = request.model.rider->present();
    request.model.crank_joint = physical;
    request.runtime.speed_kph = args.real("speed", 25.);
    request.runtime.strict = args.text("resolved_check_policy") == "strict";
    request.runtime.measurement_decimation = bounded_integer(args, "decimate", experiment ? 80 : 1);
    request.runtime.record_retention = args.flag("record") ? runtime::RetentionMode::full : runtime::RetentionMode::off;
    request.runtime.async_telemetry = args.flag("async_telemetry");
    return owner;
}
} // namespace

std::unique_ptr<ResolvedRide> resolve_ride(Arguments args, bool settling) {
    const bool experiment = args.flag("research");
    for (const auto name : {"policy", "demand", "rider_program", "control_period", "sensor_period", "sensor_delay", "actuator_delay", "road_resolution"})
        if (!experiment && args.present(name)) throw ArgumentError("policy, sensor and experiment options require --research");
    if (!args.has("decimate")) args.set("decimate", experiment ? "80" : "1");
    if (experiment && !args.has("seed")) args.set("seed", "0");
    const auto physics = ride_physics(args);
    const bool physical = physics->physics_mode == "physical";
    if (args.has("time_scale") && (args.flag("headless") || !physical || args.flag("preview")))
        throw ArgumentError("--time-scale requires an interactive physical or research run");
    if (args.flag("close_on_finish")) {
        if (settling || args.flag("headless") || !physical || args.flag("preview"))
            throw ArgumentError("--close-on-finish requires an interactive physical or research run");
        if (!args.has("duration")) throw ArgumentError("--close-on-finish requires --duration");
    }
    if (!args.has("time_scale")) args.set("time_scale", "1");
    if (args.present("backend") && !physical) throw ArgumentError("--backend native requires physical or research mode");
    if (experiment && (!physical || (args.text("drive") != "crank_effort" && args.text("drive") != "articulated_effort")))
        throw ArgumentError("--research requires physical crank_effort or articulated_effort");
    if (experiment && (args.has("sag") || args.flag("preview"))) throw ArgumentError("--research cannot combine with --sag or --preview");
    if (args.flag("record") && (!physical || args.flag("preview"))) throw ArgumentError("--record requires a physical run and cannot combine with --preview");
    if (args.has("check_policy") && (!physical || args.flag("preview"))) throw ArgumentError("--check-policy requires a physical run");
    if (args.has("duration")) positive(args.real("duration"), "--duration");
    if (physical) {
        if (args.has("speed") && args.text("drive") != "ideal_speed_control") throw ArgumentError("--speed is an ideal controller target; use --initial-speed for coast/effort");
        if ((args.has("drive_mode") && args.text("drive_mode") != "motor") || args.flag("visual_pedalling")) throw ArgumentError("legacy drive/visual flags cannot select physical drivetrain settings");
        if ((args.has("tyre_model") && args.text("tyre_model") != "sphere") || args.has("surface") || args.has("tyre_tier") || args.has("tyre_pressure")) throw ArgumentError("physical tire backend and friction must be selected in --physics-config");
        if (args.has("gearing") || args.has("ripple_depth")) throw ArgumentError("physical gearing and torque_ripple are configured in TOML");
        if (args.has("legs")) throw ArgumentError("use --rider articulated_planar instead of legacy --legs");
        if (args.text("drive") == "ideal_speed_control" && !args.has("speed")) args.set("speed", "25");
        if (!args.has("rider") && !args.flag("no_rider")) args.set("rider", args.text("drive") == "articulated_effort" ? "articulated_planar" : "lumped");
        if (args.text("drive") == "articulated_effort" && (args.flag("no_rider") || args.text("rider") != "articulated_planar")) throw ArgumentError("articulated_effort requires --rider articulated_planar");
        if (!settling && args.flag("headless") && args.text("drive") == "coast" && physics->initial_speed_mps == 0. && !args.has("duration")) throw ArgumentError("a stationary physical coast run needs --duration or nonzero --initial-speed");
        args.set("resolved_check_policy", args.text("check_policy", args.flag("headless") ? "strict" : "diagnostic"));
    } else {
        if (settling) throw ArgumentError("bike-settle requires physics_mode=physical");
        if (args.text("assist") == "emtb") throw ArgumentError("--assist emtb requires physical mode");
        for (const auto key : {"initial_speed", "human_torque", "assist_gain", "duration"})
            if (args.has(key)) throw ArgumentError(std::string(key) + " requires --physics physical");
        const bool climb = args.text("track") == "climb_steps";
        if (!args.has("speed")) args.set("speed", climb ? "16" : "25");
        if (!args.has("tyre_model")) args.set("tyre_model", climb ? "pneumatic" : "sphere");
        if (!args.has("drive_mode")) args.set("drive_mode", climb ? "pedelec" : "motor");
        if (!args.has("assist")) args.set("assist", climb ? "turbo" : "tour");
    }
    if (!args.has("crank_phase")) args.set("crank_phase", "0");
    if (!args.has("tyre_tier")) args.set("tyre_tier", "fast");
    if (!args.has("tyre_pressure")) args.set("tyre_pressure", "1.5/1.7");
    // Build the setup before moving args — member-order evaluation would
    // otherwise hand ride_setup_request a moved-from Arguments.
    auto resolved = std::make_unique<ResolvedRide>();
    resolved->setup = std::move(*ride_setup_request(args, *physics, physical, experiment));
    resolved->arguments = std::move(args);
    return resolved;
}

void validate_research_arguments(const Arguments& args) {
    if (args.flag("diagnostic_model_limits") && args.text("check_policy") == "strict") throw ArgumentError("--diagnostic-model-limits conflicts with --check-policy strict");
    if (args.has("demand") && args.has("demand_file")) throw ArgumentError("choose one of --demand and --demand-file");
    if (args.has("rider_program") && (args.flag("rider_random") || args.text("rider") != "articulated_planar" || args.text("posture") != "neutral" || args.present("human_torque"))) throw ArgumentError("--rider-program owns articulated posture and effort; drop conflicting rider inputs");
    if (args.flag("rider_random") && (args.text("rider") != "articulated_planar" || args.text("posture") != "neutral" || args.real("human_torque") != 0.)) throw ArgumentError("--rider-random owns rider type, posture and effort");
    if (args.text("posture") != "neutral" && args.text("rider") != "articulated_planar") throw ArgumentError("dynamic posture requires --rider articulated_planar");
    (void)research::validate_seed(args.text("seed", "0"));
    if (args.has("rider_seed")) (void)research::validate_seed(args.text("rider_seed"));
    for (const auto key : {"duration", "control_period", "sensor_period", "road_resolution", "energy_tolerance"}) positive(args.real(key), key);
    (void)bounded_integer(args, "record_decimation", 80);
    runtime::RideControl control;
    if (!args.flag("assist")) control.motor_torque_nm = args.real("motor_torque", 80.);
    control.motor_limit_nm = args.optional_real("motor_limit"); control.human_torque_nm = args.real("human_torque");
    for (const auto value : {control.motor_torque_nm, control.motor_limit_nm, control.human_torque_nm})
        if (value && (!std::isfinite(*value) || *value < 0.)) throw ArgumentError("control torque must be finite and nonnegative");
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::unique_ptr<research::ResearchRequest> resolve_research(const Arguments& args) {
    validate_research_arguments(args);
    auto owner = std::make_unique<research::ResearchRequest>();
    auto& request = *owner;
    request.setup.origin = "bike-research"; request.setup.asset_root = std::filesystem::current_path();
    request.setup.track = research::resolve_research_track(file(args, "track_file"), args.text("scenario"), file(args, "gen_spec"), args.text("seed"));
    request.setup.model.mode = "ride"; request.setup.model.crank_joint = true;
    request.setup.model.rider = model::rider_from_args(args.text("rider"), args.real("rider_mass"), args.real("rider_height"));
    request.setup.model.include_rider = true;
    const auto defaults = std::make_unique<configuration::SimulationPhysicsConfig>();
    defaults->physics_mode = "physical"; defaults->drive_mode = args.text("rider") == "articulated_planar" ? "articulated_effort" : "crank_effort";
    defaults->timestep_s = .0005; defaults->initial_speed_mps = 2.;
    defaults->initial_front_brake = 1.; defaults->initial_rear_brake = 1.;
    defaults->tires.backend = "compliant_2d"; defaults->tires.surface_mode = "track";
    defaults->tires.front.mu = 1.1; defaults->tires.rear.mu = 1.1;
    defaults->drive.transmission_model = "ideal_mid_drive";
    defaults->drive.human_torque_nm = 0.; defaults->drive.gearing.front_teeth = 34; defaults->drive.gearing.rear_teeth = 51;
    defaults->drive.assist.max_torque = 80.; defaults->drive.assist.max_power = 500.;
    defaults->equilibrium_refine_after_s = args.text("rider") == "articulated_planar" ? 3. : .1;
    defaults->equilibrium_refine_period_s = args.text("rider") == "articulated_planar" ? 3. : .5;
    Value overrides{Object{}}, drive{Object{}}, assist{Object{}}, gearing{Object{}};
    if (args.present("dt")) set(overrides, "timestep_s", real(args.real("dt")));
    if (args.present("initial_speed")) set(overrides, "initial_speed_mps", real(args.real("initial_speed")));
    if (args.present("initial_brake_demand")) { set(overrides, "initial_front_brake", real(args.real("initial_brake_demand"))); set(overrides, "initial_rear_brake", real(args.real("initial_brake_demand"))); }
    if (args.present("transmission")) set(drive, "transmission_model", text(args.text("transmission")));
    if (args.present("human_torque")) set(drive, "human_torque_nm", real(args.real("human_torque")));
    if (args.present("motor_max_torque")) set(assist, "max_torque", real(args.real("motor_max_torque")));
    if (args.present("motor_max_power")) set(assist, "max_power", real(args.real("motor_max_power")));
    if (args.present("front_teeth")) set(gearing, "front_teeth", integer(args.integer("front_teeth")));
    if (args.present("rear_teeth")) set(gearing, "rear_teeth", integer(args.integer("rear_teeth")));
    if (!io::as_object(assist, "assist").empty()) set(drive, "assist", std::move(assist));
    if (!io::as_object(gearing, "gearing").empty()) set(drive, "gearing", std::move(gearing));
    add_number(drive, args, "chain_stiffness", "chain_k_n_m");
    add_number(drive, args, "freehub_stiffness", "freehub_k_nm_rad");
    if (!io::as_object(drive, "drive").empty()) set(overrides, "drive", std::move(drive));
    request.setup.model.physics = research::resolve_research_physics(*defaults, file(args, "physics_config"), overrides);
    if (request.setup.model.physics->drive.transmission_model != "elastic_chain" && (args.has("chain_stiffness") || args.has("freehub_stiffness"))) throw ArgumentError("--chain-stiffness/--freehub-stiffness apply only to elastic_chain");
    request.experiment.control_period_s = args.real("control_period"); request.experiment.actuator_delay_s = args.real("actuator_delay");
    request.experiment.duration_s = args.real("duration"); request.experiment.seed_decimal = research::validate_seed(args.text("seed"));
    request.experiment.record_decimation = bounded_integer(args, "record_decimation", 80);
    request.experiment.maximum_energy_residual_ratio = args.real("energy_tolerance");
    request.experiment.stop_on_model_violation = args.text("check_policy", args.flag("diagnostic_model_limits") ? "diagnostic" : "strict") == "strict";
    request.experiment.async_telemetry = args.flag("fast_telemetry");
    request.sensors = args.flag("ideal_sensors") ? research::SensorConfig::ideal() : research::SensorConfig{};
    if (!args.flag("ideal_sensors")) request.sensors.delivery.latency_s = args.real("sensor_delay");
    request.sensors.sample_period_s = args.real("sensor_period");
    request.programs = research::load_programs(file(args, "rider_program"), file(args, "demand_file"));
    if (args.has("demand")) request.programs.demand_frames = research::constant_demand(args.real("demand")).demand_frames;
    if (args.flag("rider_random")) { request.rider_random = random::RiderRandomSpec{}; request.rider_seed_decimal = args.text("rider_seed", request.experiment.seed_decimal); }
    request.road_resolution_m = args.real("road_resolution");
    request.capture_mode = args.flag("record") ? runtime::ResearchCaptureMode::full : runtime::ResearchCaptureMode::off;
    request.experiment.validate(); request.sensors.validate(); request.programs.validate();
    return owner;
}
NATIVE_DIAG_POP
} // namespace bike::cli
