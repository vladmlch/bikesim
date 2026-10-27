#include "environment.hpp"
#include "evidence.hpp"
#include <memory>
#include <stdexcept>

namespace bike::research {
namespace {
std::unique_ptr<initialization::SetupOwner> make_settled_research_owner(const initialization::SetupRequest& setup) {
    auto copy = std::make_unique<initialization::SetupRequest>(setup);
    const auto options = copy->equilibrium;
    auto owner = initialization::make_setup(std::move(*copy));
    (void)initialization::settle(*owner, options);
    return owner;
}
// seal_bootstrap() returns ~6.5 KiB by value; the prvalue temp lives here
// so prepare_environment_into stays under the 8 KiB frame budget.
void seal_bootstrap_into(runtime::RuntimeBootstrap& dst, initialization::SetupOwner& owner) {
    dst = owner.seal_bootstrap();
}
[[gnu::noinline]] void prepare_environment_into(ResearchRequest& request, PreparedEnvironment& out) {
    request.experiment.validate(); request.sensors.validate(); request.programs.validate();
    if (!request.setup.model.physics) throw std::invalid_argument("research requires physical configuration");
    auto& physics = *request.setup.model.physics;
    configuration::post_init(physics);
    const bool spatial=physics.spatial&&physics.spatial->mode==bike::spatial::Mode::spatial3d;
    if(spatial)request.sensors_v2.validate();
    if (physics.physics_mode != "physical" || (physics.drive_mode != "crank_effort" && physics.drive_mode != "articulated_effort"))
        throw std::invalid_argument("research requires a physical effort drive, not a speed controller");
    if (physics.pitch_assist.value_or(false) || (!spatial && (physics.tires.surface_mode != "track" ||
        (physics.tires.backend != "compliant_2d" && physics.tires.backend != "distributed_2d_reference" && physics.tires.backend != "pneumatic") ||
        (physics.tires.backend == "pneumatic" && physics.tires.pneumatic.surface))))
        throw std::invalid_argument("research requires no pitch assist and track-material compliant_2d, distributed_2d_reference or pneumatic tires");
    sample_rider(request);
    request.setup.seed_decimal = validate_seed(request.experiment.seed_decimal);
    request.setup.research_startup = true;
    if(!spatial)request.setup.model.field = research_field(request.setup.track, request.road_resolution_m);
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
    config.sensor_steps = integer_steps(spatial?request.sensors_v2.sample_period_s:request.sensors.sample_period_s, config.timestep_s, "sensor period");
    if(spatial) {
        if(request.experiment.control_period_s>request.sensors_v2.maximum_read_period_s)
            throw std::invalid_argument("sensors_v2.maximum_read_period_s: shorter than research control period");
        config.sensors_v2=request.sensors_v2;
    }
    if (config.control_steps < 1 || config.max_steps < 1 || config.sensor_steps < 1)
        throw std::invalid_argument("control period, sensor period and duration must span at least one physics step");
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
    if(spatial)out.bootstrap.tape_v2=make_sensor_tape_v2(request.sensors_v2,config.seed_decimal,1+config.max_steps/config.sensor_steps);
    else out.bootstrap.tape = make_sensor_tape(request.sensors, config.seed_decimal, 1 + config.max_steps / config.sensor_steps);
    auto owner = make_settled_research_owner(request.setup);
    const auto startup = std::make_unique<initialization::ResearchStartup>(owner->research_initial_state());
    config.geometry = startup->geometry;
    config.start_position_m = startup->position_m;
    if(spatial) {
        if(!startup->observation_v2)throw std::invalid_argument("research startup: missing mounted V2 observation");
        config.raw_initial_v2=startup->observation_v2;
        runtime::spatial::SensorPipelineV2 pipeline(request.sensors_v2,*out.bootstrap.tape_v2);
        pipeline.reset(*startup->observation_v2);(void)pipeline.read(0.);out.bootstrap.startup_v2=pipeline.state();
    } else {
        config.raw_initial = startup->observation;
        out.bootstrap.startup = startup_sensor_state(request.sensors, out.bootstrap.tape, startup->observation);
    }
    seal_bootstrap_into(out.bootstrap.ride, *owner);
    out.metadata = configuration_metadata(request, *startup, out.bootstrap.ride, owner->metadata());
    config.metadata = out.metadata;
    // The spatial extent identity is the loaded physical terrain owner;
    // the request track is a name-only label with no geometry under spatial.
    // configuration_metadata already proved spatial implies a terrain owner.
    config.track_length_m = spatial ? out.bootstrap.ride.config.terrain3d->x_max_m()
        : request.setup.track.length_m;
    if (request.capture_mode == runtime::ResearchCaptureMode::full || request.capture_mode == runtime::ResearchCaptureMode::legacy)
        config.assets = snapshot_research_assets(request, *startup, out.metadata, out.bootstrap.ride);
}
} // namespace

[[gnu::noinline]] std::unique_ptr<ResearchRequest> load_research_request_ptr(const std::filesystem::path& file) {
    return std::make_unique<ResearchRequest>(read_research_request(file));
}
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
    return make_environment(std::make_unique<ResearchRequest>(std::move(request)));
}
std::unique_ptr<runtime::NativeResearchRuntime> make_environment(std::unique_ptr<ResearchRequest> request) {
    auto prepared = std::make_unique<PreparedEnvironment>();
    prepare_environment_into(*request, *prepared);
    return runtime::make_research(std::make_unique<runtime::ResearchBootstrap>(std::move(prepared->bootstrap)));
}
std::unique_ptr<runtime::NativeResearchRuntime> make_environment(const std::filesystem::path& file) {
    return make_environment(load_research_request_ptr(file));
}
} // namespace bike::research
