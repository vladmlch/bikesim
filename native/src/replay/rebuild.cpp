#include "rebuild.hpp"
#include "session.hpp"
#include "input_detail.hpp"
#include "../configuration/load.hpp"
#include "../engine_identity.hpp"
#include "../platform/files.hpp"
#include "../platform/provenance.hpp"
#include "../runtime/provenance.hpp"
#include "../terrain/trackfile.hpp"
#include "../terrain/asset3d.hpp"
#include "../terrain/spec_io.hpp"
#include "../runtime/research_io.hpp"
#include "../runtime/spatial_checkpoint.hpp"
#include "../rider/allocation.hpp"
#include <algorithm>
#include <cstdlib>
#include <cerrno>
#include <memory>
#include <system_error>
#include <unistd.h>

namespace bike::replay {
namespace {
using namespace detail;
template<class Owner> void reals(Owner& out, const Value& source,
    std::initializer_list<std::pair<std::string_view, double Owner::*>> members) {
    for (const auto& [name, member] : members) if (const auto* value = find(source, name)) out.*member = io::as_double(*value, name);
}
template<class Owner> void integers(Owner& out, const Value& source,
    std::initializer_list<std::pair<std::string_view, int Owner::*>> members) {
    for (const auto& [name, member] : members) if (const auto* value = find(source, name)) {
        const auto integer_value = count(*value);
        if (integer_value < std::numeric_limits<int>::min() || integer_value > std::numeric_limits<int>::max())
            throw recording::RecordingError(std::string(name) + " is outside the native integer domain");
        out.*member = static_cast<int>(integer_value);
    }
}
geometry::BikeSpecs bike_specs(const Value& value) {
    geometry::BikeSpecs out;
#define BIKE_REAL(name) {#name, &geometry::BikeSpecs::name}
    reals(out, value, {BIKE_REAL(reach), BIKE_REAL(stack), BIKE_REAL(head_angle_deg), BIKE_REAL(effective_seat_angle_deg), BIKE_REAL(bb_drop),
        BIKE_REAL(wheelbase), BIKE_REAL(headtube_length), BIKE_REAL(front_wheel_radius), BIKE_REAL(rear_wheel_radius), BIKE_REAL(wheel_radius),
        BIKE_REAL(crank_length), BIKE_REAL(fork_travel), BIKE_REAL(fork_offset), BIKE_REAL(rear_wheel_travel), BIKE_REAL(shock_eye_to_eye),
        BIKE_REAL(shock_stroke), BIKE_REAL(fork_stiffness), BIKE_REAL(fork_damping), BIKE_REAL(shock_stiffness), BIKE_REAL(shock_damping), BIKE_REAL(fork_initial_psi)});
#undef BIKE_REAL
#define BIKE_INT(name) {#name, &geometry::BikeSpecs::name}
    integers(out, value, {BIKE_INT(fork_hsc), BIKE_INT(fork_lsc), BIKE_INT(fork_rebound), BIKE_INT(fork_air_tokens),
        BIKE_INT(shock_hsc), BIKE_INT(shock_lsc), BIKE_INT(shock_rebound), BIKE_INT(shock_hbo)});
#undef BIKE_INT
    if (const auto* lockout = find(value, "shock_lockout")) out.shock_lockout = io::as_bool(*lockout, "shock_lockout");
    (void)geometry::validate_geometry(out); return out;
}
model::BikeMassSpecs mass_specs(const Value& value) {
    model::BikeMassSpecs out;
#define BIKE_MASS(name) {#name, &model::BikeMassSpecs::name}
    reals(out, value, {BIKE_MASS(motor_mass), BIKE_MASS(battery_mass), BIKE_MASS(frame_structure_mass), BIKE_MASS(saddle_post_mass),
        BIKE_MASS(crank_pedals_mass), BIKE_MASS(steer_assembly_mass), BIKE_MASS(stanchions_mass), BIKE_MASS(fork_lowers_mass),
        BIKE_MASS(chainstay_mass), BIKE_MASS(seatstay_mass), BIKE_MASS(rocker_mass), BIKE_MASS(shock_yoke_mass), BIKE_MASS(shock_damper_mass),
        BIKE_MASS(front_wheel_mass), BIKE_MASS(rear_wheel_mass), BIKE_MASS(wheel_rim_tire_fraction), BIKE_MASS(wheel_hub_core_fraction)});
#undef BIKE_MASS
    return out;
}
configuration::RiderSpecs rider_specs(const Value& value) {
    configuration::RiderSpecs out;
    if (const auto* variant = find(value, "variant")) out.variant = string(*variant);
    if (const auto* legs = find(value, "legs")) out.legs = string(*legs);
    if (const auto* inseam = find(value, "inseam_m"); inseam && !io::is_null(*inseam)) out.inseam_m = io::as_double(*inseam, "inseam_m");
#define BIKE_RIDER(name) {#name, &configuration::RiderSpecs::name}
    reals(out, value, {BIKE_RIDER(mass_kg), BIKE_RIDER(height_m), BIKE_RIDER(helmet_mass_kg), BIKE_RIDER(saddle_share),
        BIKE_RIDER(pedal_share), BIKE_RIDER(bar_share), BIKE_RIDER(torso_resonance_hz), BIKE_RIDER(leg_resonance_hz),
        BIKE_RIDER(arm_resonance_hz), BIKE_RIDER(damping_ratio), BIKE_RIDER(saddle_interface_k_n_m), BIKE_RIDER(saddle_interface_c_ns_m)});
#undef BIKE_RIDER
    configuration::detail::post_init(out); return out;
}
Value configuration_hashable(Value resolved) {
    auto physics = field(resolved, "physics"), articulated = field(physics, "articulated");
    for (auto key : {"joint_envelope", "joint_strength"}) set(articulated, std::string(key) + "_path", maybe(resolved, std::string(key) + "_sha256"));
    if(find(articulated,"shoulder_profile"))set(articulated,"shoulder_profile",maybe(resolved,"shoulder_profile_asset_sha256"));
    set(physics, "articulated", std::move(articulated)); set(resolved, "physics", std::move(physics)); return resolved;
}
} // namespace
FixedAssets::FixedAssets(const ReplayInput& input) {
    auto pattern = (std::filesystem::temp_directory_path() / "bike-replay-input-XXXXXX").string();
    std::vector<char> name(pattern.begin(), pattern.end()); name.push_back('\0');
    char* made = ::mkdtemp(name.data());
    if (!made) throw std::system_error(errno, std::generic_category(), "create replay input directory");
    path_ = made;
    try {
        if(input.spatial_recipe) {
            std::filesystem::create_directory(path_/"spatial");
            if(const auto it=input.assets.find("spatial/terrain.terrain3d");it!=input.assets.end())
                platform::write_exclusive(path_/"spatial/terrain.terrain3d",it->second);
            if(const auto it=input.assets.find("spatial/shoulder_profile.json");it!=input.assets.end())
                platform::write_exclusive(path_/"spatial/shoulder_profile.json",it->second);
        }
        for (auto file : {"track.toml", "rider_joint_envelope.json", "rider_joint_strength.json"})
            if (const auto it = input.assets.find(file); it != input.assets.end()) platform::write_exclusive(path_ / file, it->second);
    } catch (...) { std::error_code ignored; std::filesystem::remove_all(path_, ignored); throw; }
}
FixedAssets::~FixedAssets() { std::error_code ignored; std::filesystem::remove_all(path_, ignored); }

std::unique_ptr<research::ResearchRequest> request_from_input(const ReplayInput& input, const FixedAssets& assets) {
    const auto& saved = input.summary;
    const auto& resolved = field(saved, "resolved_config"), experiment = field(field(saved, "research"), "config");
    auto physics = field(resolved, "physics"); auto articulated = field(physics, "articulated");
    for (auto [key, name] : {std::pair{"joint_envelope", "rider_joint_envelope.json"}, std::pair{"joint_strength", "rider_joint_strength.json"}}) {
        const auto* declared = find(articulated, std::string(key) + "_path");
        const bool bundled = input.assets.contains(name);
        if (declared && !io::is_null(*declared) && !bundled) throw IncompatibleExecution(std::string("legacy recording lacks bundled ") + key);
        set(articulated, std::string(key) + "_path", bundled ? text((assets.path() / name).string()) : Value{});
    }
    if(input.spatial_recipe) {
        const auto* declared=find(articulated,"shoulder_profile");
        const bool bundled=input.assets.contains("spatial/shoulder_profile.json");
        if((declared&&!io::is_null(*declared))!=bundled)throw IncompatibleExecution("spatial shoulder profile lacks its immutable bundle");
        if(bundled)set(articulated,"shoulder_profile",text((assets.path()/"spatial/shoulder_profile.json").string()));
    }
    set(physics, "articulated", std::move(articulated));
    if (const auto* initial_file = find(physics, "equilibrium_state_file"); initial_file && !io::is_null(*initial_file)) {
        if (!input.spatial_recipe)
            throw IncompatibleExecution("recording startup depends on an unbundled initial-state file; recorded paths are never followed");
        // Contract 4 owns the complete initial checkpoint. The recorded path
        // is provenance only and must never be opened during reconstruction.
        set(physics, "equilibrium_state_file", Value{});
    }
    // Fresh deterministic setup: neither a recorded external cache pathname
    // nor a user's current equilibrium cache may replace the replay recipe.
    set(physics, "equilibrium_cache_enabled", Value{false});
    auto owner = std::make_unique<research::ResearchRequest>();
    auto& request = *owner;
    request.capture_mode = input.spatial_recipe ? runtime::ResearchCaptureMode::full : runtime::ResearchCaptureMode::verify;
    request.setup.model.physics = configuration::detail::construct_config(physics);
    request.setup.model.bike = bike_specs(field(resolved, "geometry_and_suspension"));
    request.setup.model.masses = mass_specs(field(resolved, "mass_budget"));
    request.setup.model.rider = rider_specs(field(resolved, "rider"));
    request.setup.model.mode = "ride"; request.setup.model.crank_joint = true;
    request.setup.asset_root = assets.path(); request.setup.origin = "validated replay recipe";
    request.setup.track = terrain::track_from_dict(terrain::parse_track_toml(asset_text(input, "track.toml"), "recording/track.toml"));
    if(input.spatial_recipe) {
        if(const auto* threads=find(field(saved,"execution"),"tyre_threads")) {
            const auto value=count(*threads);
            if(value<1 || value>64)throw IncompatibleExecution("execution.tyre_threads: expected [1,64]");
            request.setup.runtime.tyre_threads=static_cast<unsigned>(value);
        }
        const auto& recipe=*input.spatial_recipe;
        const auto source=io::parse_recording_json(asset_text(input,"spatial/terrain_source.json"));
        if(recipe.terrain_kind==terrain::Terrain3dKind::grid) {
            auto grid=terrain::read_terrain3d(input.assets.at("spatial/terrain.terrain3d"));
            if(!grid)throw IncompatibleExecution("spatial terrain: "+grid.error());
            request.setup.model.terrain3d=std::make_shared<const terrain::Terrain3dSource>(std::move(*grid));
        } else {
            auto extrusion=terrain::detail::load_profile_extrusion3d(assets.path()/"track.toml",io::as_double(field(source,"corridor"),"corridor"));
            request.setup.model.terrain3d=std::make_shared<const terrain::Terrain3dSource>(std::move(extrusion));
        }
        if(request.setup.model.terrain3d->fingerprint_hex()!=recipe.terrain_sha256_hex)
            throw IncompatibleExecution("spatial terrain rebuilt source identity differs");
        request.setup.start_x_m=io::as_double(field(recipe.config_spatial,"start_x_m"),"start_x_m");
    } else {
    request.setup.start_x_m = io::as_double(field(input.manifest, "start_x_m"), "start_x_m");
    auto& f = request.setup.model.field; const auto& field_value = field(input.manifest, "heightfield");
    integers(f, field_value, {{"nrow", &terrain::HeightFieldSpec::nrow}, {"ncol", &terrain::HeightFieldSpec::ncol}});
    reals(f, field_value, {{"radius_x_m", &terrain::HeightFieldSpec::radius_x_m}, {"radius_y_m", &terrain::HeightFieldSpec::radius_y_m},
        {"elevation_m", &terrain::HeightFieldSpec::elevation_m}, {"base_m", &terrain::HeightFieldSpec::base_m}, {"datum_z_m", &terrain::HeightFieldSpec::datum_z_m}});
    f.validate(); request.road_resolution_m = f.resolution_m();
    if (!(research::research_field(request.setup.track, request.road_resolution_m) == f))
        throw IncompatibleExecution("recorded heightfield differs from the native research field contract");
    }
    auto& e = request.experiment;
    reals(e, experiment, {{"control_period_s", &research::ExperimentConfig::control_period_s}, {"actuator_delay_s", &research::ExperimentConfig::actuator_delay_s},
        {"duration_s", &research::ExperimentConfig::duration_s}, {"wheelie_persistence_s", &research::ExperimentConfig::wheelie_persistence_s},
        {"maximum_energy_residual_ratio", &research::ExperimentConfig::maximum_energy_residual_ratio}});
    integers(e, experiment, {{"record_decimation", &research::ExperimentConfig::record_decimation}});
    if (const auto* stop = find(experiment, "stop_on_model_violation")) e.stop_on_model_violation = io::as_bool(*stop, "stop_on_model_violation");
    if (const auto* async = find(experiment, "async_telemetry")) e.async_telemetry = io::as_bool(*async, "async_telemetry");
    const auto& research_data = field(saved, "research");
    const auto& seed = field(research_data, "actual_sensor_seed");
    if (const auto* number = std::get_if<io::Number>(&seed.data)) e.seed_decimal = research::validate_seed(number->lexeme);
    else throw recording::RecordingError("actual_sensor_seed must be a nonnegative integer");
    request.setup.seed_decimal = e.seed_decimal;
    const auto& sensors = field(research_data, "sensor_config");
    if(input.spatial_recipe)request.sensors_v2=runtime::decode_sensor_config_v2(sensors);
    else {
    auto& s = request.sensors;
    reals(s, sensors, {{"sample_period_s", &research::SensorConfig::sample_period_s}});
    reals(s.delivery, sensors, {{"latency_s", &runtime::SensorConfig::latency_s}, {"maximum_age_s", &runtime::SensorConfig::maximum_age_s},
        {"dropout_probability", &runtime::SensorConfig::dropout_probability}, {"gyro_bias_rad_s", &runtime::SensorConfig::gyro_bias}});
    reals(s.noise, sensors, {{"acceleration_std_mps2", &random::SensorNoiseConfig::acceleration_std_mps2},
        {"gyro_std_rad_s", &random::SensorNoiseConfig::gyro_std_rad_s}, {"encoder_std_rad_s", &random::SensorNoiseConfig::encoder_std_rad_s},
        {"torque_std_nm", &random::SensorNoiseConfig::torque_std_nm}});
    if (const auto* enabled = find(sensors, "imu_enabled")) s.delivery.imu_enabled = io::as_bool(*enabled, "imu_enabled");
    if (const auto* bias = find(sensors, "acceleration_bias_mps2")) {
        const auto vector = double_vector(*bias, "acceleration bias");
        if (vector.size() != 3) throw recording::RecordingError("acceleration bias must have width 3");
        std::ranges::copy(vector, s.delivery.acceleration_bias.begin());
    }
    }
    const auto program = maybe(research_data, "rider_program");
    if (!io::is_null(program) && (!find(research_data, "rider_command_recording") || string(field(research_data, "rider_command_recording")) != "program_inputs"))
        throw IncompatibleExecution("native replay requires the recorded rider program input contract");
    request.programs = research::programs_from_documents(program, maybe(research_data, "demand_program"));
    request.experiment.validate();
    if(input.spatial_recipe)request.sensors_v2.validate();else request.sensors.validate();
    return owner;
}

void verify_configuration(const ReplayInput& input, const research::PreparedEnvironment& prepared) {
    const auto& expected = field(input.summary, "resolved_config");
    auto reconstructed = field(prepared.metadata, "resolved_config");
    auto physics = field(reconstructed, "physics");
    set(physics, "equilibrium_cache_enabled", field(field(expected, "physics"), "equilibrium_cache_enabled"));
    if (input.spatial_recipe)
        set(physics, "equilibrium_state_file", field(field(expected, "physics"), "equilibrium_state_file"));
    set(reconstructed, "physics", std::move(physics));
    const auto actual_hashable = configuration_hashable(reconstructed), expected_hashable = configuration_hashable(expected);
    const auto actual_json = io::canonical_json(actual_hashable);
    const auto expected_json = io::canonical_json(expected_hashable);
    if (actual_json != expected_json || io::sha256(std::as_bytes(std::span(expected_json.data(), expected_json.size()))) != string(field(input.summary, "configuration_sha256")))
        throw IncompatibleExecution("reconstructed configuration differs from the recorded recipe");
    if (string(field(prepared.metadata, "terrain_sha256")) != string(field(input.summary, "terrain_sha256")))
        throw IncompatibleExecution("reconstructed terrain differs from the recorded compiled road");
    if(input.spatial_recipe) {
        const auto& bootstrap=prepared.bootstrap.ride;
        const auto expected_checkpoint=runtime::spatial::decode_checkpoint(input.spatial_recipe->checkpoint_initial);
        if(!bootstrap.state.spatial || !bootstrap.config.terrain3d ||
            bootstrap.config.terrain3d->fingerprint_hex()!=input.spatial_recipe->terrain_sha256_hex ||
            bootstrap.state.spatial->topology_fingerprint!=runtime::decode_spatial_bootstrap(field(expected_checkpoint.bootstrap2,"spatial")).topology_fingerprint ||
            bootstrap.model.sha256!=string(field(expected_checkpoint.identity,"model_digest")))
            throw IncompatibleExecution("spatial model/topology/terrain/state identity differs");
        const Value expected_calibration{Object{{"calibration_sha256",text(bootstrap.state.spatial->calibration_sha256)},
            {"tire",field(bootstrap.config.writer_config,"tire")},{"imu",field(spatial::spatial_config_to_wire(*bootstrap.config.spatial),"imu")}}};
        if(!equal(expected_calibration,io::parse_recording_json(asset_text(input,"spatial/calibration.json"))))
            throw IncompatibleExecution("spatial calibration coefficients/revision differ from reconstructed configuration");
        return;
    }
    const auto& geometry = prepared.bootstrap.config.geometry;
    if (geometry.x.size() * 2 != input.terrain_vertices.size()) throw IncompatibleExecution("compiled terrain width differs");
    for (std::size_t i = 0; i < geometry.x.size(); ++i)
        if (geometry.x[i] != input.terrain_vertices[2 * i] || geometry.z[i] != input.terrain_vertices[2 * i + 1])
            throw IncompatibleExecution("compiled terrain differs at vertex " + std::to_string(i));
}
void check_execution(const ReplayInput& input) {
    if (input.replay_contract_version == 1 || !find(input.manifest, "execution") ||
        !find(field(input.manifest, "execution"), "execution_identity"))
        throw IncompatibleExecution("incompatible-execution: historical Python/native-binding arithmetic profile has no certified native-executable compatibility fixture");
    const auto& execution = field(input.manifest, "execution"); validate_execution(execution);
    const auto current = runtime::runtime_identity(); const auto provider = engine::identity();
    for (const auto& [key, actual] : {std::pair{"native_source_sha256", current.native_source_sha256},
        std::pair{"execution_identity", platform::execution_identity()}, std::pair{"mujoco_library_sha256", provider.library_sha256},
        std::pair{"numeric_provider", provider.numeric_provider}})
        if (string(field(execution, key)) != actual) throw IncompatibleExecution(std::string("incompatible-execution: ") + key + " differs from this runtime");
    if (string(field(input.summary, "model_source_sha256")) != current.native_source_sha256)
        throw IncompatibleExecution("incompatible-execution: recorded plant source differs");
    if (const auto* solver = find(execution, "allocation_solver"))
        if (string(*solver) != bike::rider::allocation_numeric_identity || string(field(execution, "allocation_solver_qualification")) != "qualified")
            throw IncompatibleExecution("incompatible-execution: recorded allocation solver is not qualified");
    for (auto key : {"source_changed_during_run", "execution_changed_during_run"})
        if (const auto* value = find(field(input.summary, "research"), key); value && !io::is_null(*value) && io::as_bool(*value, key))
            throw IncompatibleExecution("source or binary identity changed during recording");
}
} // namespace bike::replay
