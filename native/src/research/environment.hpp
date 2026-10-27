#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include "../initialization/setup_owner.hpp"
#include "../random/numpy_compat.hpp"
#include "../runtime/research_io.hpp"

namespace bike::research {

struct ExperimentConfig {
    double control_period_s = .01, actuator_delay_s = .005, duration_s = 3.;
    std::string seed_decimal = "0";
    int record_decimation = 20;
    double wheelie_persistence_s = .02, maximum_energy_residual_ratio = .05;
    bool stop_on_model_violation = true, async_telemetry = false;
    void validate() const;
};

struct SensorConfig {
    runtime::SensorConfig delivery;
    random::SensorNoiseConfig noise;
    double sample_period_s = .001;
    void validate() const;
    [[nodiscard]] static SensorConfig ideal();
};

struct ResearchRequest {
    initialization::SetupRequest setup;
    ExperimentConfig experiment;
    SensorConfig sensors;
    runtime::spatial::SensorConfigV2 sensors_v2;
    runtime::ResearchPrograms programs;
    std::optional<random::RiderRandomSpec> rider_random;
    std::optional<std::string> rider_seed_decimal;
    double road_resolution_m = .005;
    runtime::ResearchCaptureMode capture_mode = runtime::ResearchCaptureMode::legacy;
};

struct PreparedEnvironment {
    runtime::ResearchBootstrap bootstrap;
    io::Value metadata;
};

struct RiderSignals {
    double pitch_rate_up_rad_s = 0.;
    std::array<double, 3> specific_force_body_mps2{};
    double saddle_load_n = 0., bar_load_n = 0., pedal_load_n = 0.;
};
class RiderBehavior {
public:
    virtual ~RiderBehavior();
    RiderBehavior() = default;
    RiderBehavior(const RiderBehavior&) = delete;
    RiderBehavior& operator=(const RiderBehavior&) = delete;
    RiderBehavior(RiderBehavior&&) = delete;
    RiderBehavior& operator=(RiderBehavior&&) = delete;
    virtual void reset(std::string_view seed_decimal) = 0;
    [[nodiscard]] virtual std::optional<::rider::RiderPosture> act(double time_s, const RiderSignals&) = 0;
};
[[nodiscard]] RiderSignals signals_from_sample(const runtime::SamplePtr&);

[[nodiscard]] std::string validate_seed(std::string_view seed_decimal);
[[nodiscard]] std::int64_t integer_steps(double seconds, double dt_s, std::string_view name);
[[nodiscard]] terrain::HeightFieldSpec research_field(const terrain::TrackSpec&, double resolution_m = .005);
[[nodiscard]] terrain::TrackSpec resolve_research_track(const std::optional<std::filesystem::path>& track_file,
    std::string_view scenario, const std::optional<std::filesystem::path>& generator_spec_file, std::string_view seed_decimal);
[[nodiscard]] ResearchRequest read_research_request(const std::filesystem::path&);
// Heap-returning twin for callers whose frame cannot host the request prvalue.
[[nodiscard]] std::unique_ptr<ResearchRequest> load_research_request_ptr(const std::filesystem::path&);
[[nodiscard]] PreparedEnvironment prepare_environment(ResearchRequest);
// Heap-returning twin for callers whose frame cannot host the request and
// prepared values at once; the request itself stays caller-owned on the heap.
[[nodiscard]] std::unique_ptr<PreparedEnvironment> prepare_environment(std::unique_ptr<ResearchRequest>);
[[nodiscard]] std::unique_ptr<runtime::NativeResearchRuntime> make_environment(ResearchRequest);
[[nodiscard]] std::unique_ptr<runtime::NativeResearchRuntime> make_environment(std::unique_ptr<ResearchRequest>);
[[nodiscard]] std::unique_ptr<runtime::NativeResearchRuntime> make_environment(const std::filesystem::path&);

[[nodiscard]] runtime::ResearchPrograms programs_from_documents(
    const io::Value& rider_program, const io::Value& demand_program);
[[nodiscard]] runtime::ResearchPrograms load_programs(
    const std::optional<std::filesystem::path>& rider_program,
    const std::optional<std::filesystem::path>& demand_program);
[[nodiscard]] io::Value rider_program_value(const runtime::ResearchPrograms&);
[[nodiscard]] io::Value demand_program_value(const runtime::ResearchPrograms&);
[[nodiscard]] runtime::ResearchPrograms constant_demand(double torque_nm);
void sample_rider(ResearchRequest&);
[[nodiscard]] runtime::NoiseTape make_sensor_tape(const SensorConfig&, std::string_view seed_decimal,
                                                std::int64_t count);
[[nodiscard]] runtime::spatial::NoiseTapeV2 make_sensor_tape_v2(const runtime::spatial::SensorConfigV2&,
    std::string_view seed_decimal, std::int64_t count);
[[nodiscard]] runtime::SensorState startup_sensor_state(const SensorConfig&,
    const runtime::NoiseTape&, const runtime::SensorObservation& raw_initial);
[[nodiscard]] io::Value experiment_value(const ExperimentConfig&);
[[nodiscard]] io::Value sensor_config_value(const SensorConfig&);

// CLI precedence is resolved before physical setup, using the source's
// transmission-specific dt and closure-time adjustment rules.
[[nodiscard]] configuration::SimulationPhysicsConfig resolve_research_physics(
    const configuration::SimulationPhysicsConfig& defaults,
    const std::optional<std::filesystem::path>& physics_file,
    const io::Value& explicit_overrides);

} // namespace bike::research
