#include "environment.hpp"
#include "evidence.hpp"
#include "../geometry/specs.hpp"
#include "../initialization/launch_preparation.hpp"
#include "../terrain/extrusion3d.hpp"
#include "../terrain/generator3d.hpp"
#include "../terrain/source3d.hpp"
#include <memory>
#include <stdexcept>

namespace bike::research {
namespace {
// Callback signature is fixed by Compliant3dTerrainCallbacks::sample_fn.
terrain::TerrainSample3d sample_terrain3d_cb(
    const void* c, double x, double y) noexcept { // NOLINT(bugprone-easily-swappable-parameters)
    return static_cast<const terrain::Terrain3dSource*>(c)->sample(x, y);
}
terrain::TorusClosestPoint3d closest_terrain3d_cb(
    const void* c, const terrain::Vec3d& center, double search_radius_m) noexcept {
    return static_cast<const terrain::Terrain3dSource*>(c)->closest_point(center, search_radius_m);
}
Compliant3dTerrainCallbacks terrain3d_callbacks(const terrain::Terrain3dSource& src) noexcept {
    return Compliant3dTerrainCallbacks{
        .ctx = &src,
        .sample_fn = &sample_terrain3d_cb,
        .closest_point_fn = &closest_terrain3d_cb,
    };
}
std::unique_ptr<initialization::SetupOwner> make_settled_research_owner(const initialization::SetupRequest& setup) {
    auto copy = std::make_unique<initialization::SetupRequest>(setup);
    const auto options = copy->equilibrium;
    auto owner = initialization::make_setup(std::move(*copy));
    (void)initialization::settle(*owner, options);
    return owner;
}
[[gnu::noinline]] void prepare_environment_into(ResearchRequest& request, PreparedEnvironment& out) {
    request.experiment.validate(); request.sensors.validate(); request.programs.validate();
    if (!request.setup.model.physics) throw std::invalid_argument("research requires physical configuration");
    // Launch part 2: the research request itself carries the prepared
    // start, so its recorded physics and assets name it.
    (void)initialization::prepare_request(request.setup);
    auto& physics = *request.setup.model.physics;
    configuration::post_init(physics);
    if (physics.physics_mode != "physical" || (physics.drive_mode != "crank_effort" && physics.drive_mode != "articulated_effort"))
        throw std::invalid_argument("research requires a physical effort drive, not a speed controller");
    const bool is_spatial = request.setup.spatial.has_value();
    if (physics.pitch_assist.value_or(false) || physics.tires.surface_mode != "track" ||
        (!is_spatial &&
         ((physics.tires.backend != "compliant_2d" && physics.tires.backend != "distributed_2d_reference" && physics.tires.backend != "pneumatic") ||
          (physics.tires.backend == "pneumatic" && physics.tires.pneumatic.surface))))
        throw std::invalid_argument("research requires no pitch assist and track-material compliant_2d, distributed_2d_reference or pneumatic tires");
    sample_rider(request);
    request.setup.seed_decimal = validate_seed(request.experiment.seed_decimal);
    request.setup.research_startup = true;
    if (!is_spatial)
        request.setup.model.field = research_field(request.setup.track, request.road_resolution_m);
    request.setup.runtime.strict = request.experiment.stop_on_model_violation;
    request.setup.runtime.async_telemetry = request.experiment.async_telemetry;
    request.setup.runtime.measurement_decimation = request.experiment.record_decimation;
    request.setup.runtime.record_retention = request.capture_mode == runtime::ResearchCaptureMode::full ? runtime::RetentionMode::full :
        request.capture_mode == runtime::ResearchCaptureMode::legacy ? runtime::RetentionMode::legacy : runtime::RetentionMode::off;
    auto& config = out.bootstrap.config;
    config.timestep_s = physics.timestep_s;
    config.control_steps = integer_steps(request.experiment.control_period_s, config.timestep_s, "control period");
    config.delay_steps = integer_steps(request.experiment.actuator_delay_s, config.timestep_s, "actuator delay");
    config.max_steps = integer_steps(request.experiment.duration_s, config.timestep_s, "duration");
    config.sensor_steps = integer_steps(request.sensors.sample_period_s, config.timestep_s, "sensor period");
    if (config.control_steps < 1 || config.max_steps < 1 || config.sensor_steps < 1)
        throw std::invalid_argument("control period, sensor period and duration must span at least one physics step");
    config.track_length_m = request.setup.track.length_m;
    config.record_decimation = request.experiment.record_decimation;
    config.capture_mode = request.capture_mode;
    config.stop_on_model_violation = request.experiment.stop_on_model_violation;
    config.maximum_energy_residual_ratio = request.experiment.maximum_energy_residual_ratio;
    config.wheelie_persistence_s = request.experiment.wheelie_persistence_s;
    config.sensors = request.sensors.delivery;
    config.programs = request.programs;
    config.startup_control.motor_torque_nm = 0.;
    if (!physics.seated_climb.enabled) config.startup_control.human_torque_nm = 0.;
    config.seed_decimal = request.setup.seed_decimal;
    config.noise_std = {request.sensors.noise.acceleration_std_mps2, request.sensors.noise.gyro_std_rad_s,
        request.sensors.noise.encoder_std_rad_s, request.sensors.noise.torque_std_nm};
    // All declarative checks and large tape allocation precede engine setup.
    out.bootstrap.tape = make_sensor_tape(request.sensors, config.seed_decimal, 1 + config.max_steps / config.sensor_steps, is_spatial);
    auto owner = make_settled_research_owner(request.setup);
    const auto startup = std::make_unique<initialization::ResearchStartup>(owner->research_initial_state());
    config.geometry = startup->geometry;
    if (is_spatial && config.geometry.spatial != nullptr) {
        const double ground_z_m = geometry::compute_ground_z(request.setup.model.bike) / 1000.;
        std::shared_ptr<const terrain::Terrain3dSource> terrain_src;
        if (request.setup.terrain_overlay.has_value()) {
            terrain::Terrain3dOverlaySpec overlay = *request.setup.terrain_overlay;
            if (overlay.datum_z_offset_m == 0.) overlay.datum_z_offset_m = ground_z_m;
            terrain_src = std::make_shared<terrain::Terrain3dSource>(
                terrain::build_overlay_grid3d(request.setup.track, overlay));
        } else {
            terrain_src = std::make_shared<terrain::Terrain3dSource>(
                terrain::ExtrudedTrack3d(request.setup.track, request.setup.spatial->corridor_half_width_m, ground_z_m));
        }
        auto sp_copy = *config.geometry.spatial;
        sp_copy.terrain = terrain3d_callbacks(*terrain_src);
        sp_copy.terrain_keepalive = std::move(terrain_src);
        config.geometry.spatial = std::make_shared<const runtime::SpatialTruthGeometry>(std::move(sp_copy));
    }
    config.start_position_m = startup->position_m;
    config.raw_initial = startup->observation;
    out.bootstrap.startup = startup_sensor_state(request.sensors, out.bootstrap.tape, startup->observation);
    out.bootstrap.ride = owner->seal_bootstrap();
    out.metadata = configuration_metadata(request, *startup, out.bootstrap.ride, owner->metadata());
    config.metadata = out.metadata;
    if (request.capture_mode == runtime::ResearchCaptureMode::full || request.capture_mode == runtime::ResearchCaptureMode::legacy)
        config.assets = snapshot_research_assets(request, *startup, out.metadata);
}
[[gnu::noinline]] std::unique_ptr<ResearchRequest> load_research_request_ptr(const std::filesystem::path& file) {
    return std::make_unique<ResearchRequest>(read_research_request(file));
}
} // namespace

PreparedEnvironment prepare_environment(ResearchRequest request) {
    PreparedEnvironment out;
    prepare_environment_into(request, out);
    return out;
}
[[gnu::noinline]] std::unique_ptr<PreparedEnvironment> prepare_environment(std::unique_ptr<ResearchRequest> request) {
    auto prepared = std::make_unique<PreparedEnvironment>();
    prepare_environment_into(*request, *prepared);
    return prepared;
}
[[gnu::noinline]] std::unique_ptr<runtime::NativeResearchRuntime> make_environment(ResearchRequest request) {
    auto prepared = std::make_unique<PreparedEnvironment>();
    prepare_environment_into(request, *prepared);
    return runtime::make_research(std::move(prepared->bootstrap));
}
std::unique_ptr<runtime::NativeResearchRuntime> make_environment(const std::filesystem::path& file) {
    auto req = load_research_request_ptr(file);
    auto prepared = std::make_unique<PreparedEnvironment>();
    prepare_environment_into(*req, *prepared);
    return runtime::make_research(std::move(prepared->bootstrap));
}
} // namespace bike::research
