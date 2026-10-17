#include "environment.hpp"
#include "evidence.hpp"
#include <stdexcept>

namespace bike::research {
PreparedEnvironment prepare_environment(ResearchRequest request) {
    request.experiment.validate(); request.sensors.validate(); request.programs.validate();
    if (!request.setup.model.physics) throw std::invalid_argument("research requires physical configuration");
    auto& physics = *request.setup.model.physics;
    configuration::post_init(physics);
    if (physics.physics_mode != "physical" || (physics.drive_mode != "crank_effort" && physics.drive_mode != "articulated_effort"))
        throw std::invalid_argument("research requires a physical effort drive, not a speed controller");
    if (physics.pitch_assist.value_or(false) || physics.tires.surface_mode != "track" ||
        (physics.tires.backend != "compliant_2d" && physics.tires.backend != "distributed_2d_reference" && physics.tires.backend != "pneumatic") ||
        (physics.tires.backend == "pneumatic" && physics.tires.pneumatic.surface))
        throw std::invalid_argument("research requires no pitch assist and track-material compliant_2d, distributed_2d_reference or pneumatic tires");
    sample_rider(request);
    request.setup.seed_decimal = validate_seed(request.experiment.seed_decimal);
    request.setup.research_startup = true;
    request.setup.model.field = research_field(request.setup.track, request.road_resolution_m);
    request.setup.runtime.strict = request.experiment.stop_on_model_violation;
    request.setup.runtime.measurement_decimation = request.experiment.record_decimation;
    request.setup.runtime.record_retention = request.capture_mode == runtime::ResearchCaptureMode::full ? runtime::RetentionMode::full :
        request.capture_mode == runtime::ResearchCaptureMode::legacy ? runtime::RetentionMode::legacy : runtime::RetentionMode::off;
    runtime::ResearchConfig config;
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
    auto tape = make_sensor_tape(request.sensors, config.seed_decimal, 1 + config.max_steps / config.sensor_steps);
    auto owner = initialization::make_setup(request.setup);
    (void)initialization::settle(*owner, request.setup.equilibrium);
    const auto startup = owner->research_initial_state();
    config.geometry = startup.geometry;
    config.start_position_m = startup.position_m;
    config.raw_initial = startup.observation;
    auto state = startup_sensor_state(request.sensors, tape, startup.observation);
    auto ride = owner->seal_bootstrap();
    auto metadata = configuration_metadata(request, startup, ride, owner->metadata());
    config.metadata = metadata;
    if (request.capture_mode == runtime::ResearchCaptureMode::full || request.capture_mode == runtime::ResearchCaptureMode::legacy)
        config.assets = snapshot_research_assets(request, startup, metadata);
    return {{std::move(ride), std::move(config), std::move(tape), std::move(state)}, std::move(metadata)};
}
std::unique_ptr<runtime::NativeResearchRuntime> make_environment(ResearchRequest request) {
    auto prepared = prepare_environment(std::move(request));
    return runtime::make_research(std::move(prepared.bootstrap));
}
std::unique_ptr<runtime::NativeResearchRuntime> make_environment(const std::filesystem::path& file) {
    return make_environment(read_research_request(file));
}
} // namespace bike::research
