// Authored static regressions for T34. No build or execution was performed
// during the no-run implementation session; oracle qualification is separate.
#include "support/migration_test.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "io/json.hpp"
#include "runtime/setup_values.hpp"
#include "terrain/trackfile.hpp"
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

namespace {
using namespace bike::research;
void same(std::span<const double> a, std::span<const double> b, std::string_view message) {
    bt::require(a.size() == b.size(), message);
    for (std::size_t i = 0; i < a.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(a[i]) == std::bit_cast<std::uint64_t>(b[i]), message);
}
void clocks_and_programs() {
    bt::require(integer_steps(.01, .00125, "control") == 8, "integral control clock");
    bt::require(integer_steps(0., .00125, "delay") == 0, "zero delay");
    bt::invalid([] { (void)integer_steps(.001, .00125, "sensor"); });
    bt::invalid([] { (void)integer_steps(-1., .00125, "duration"); });
    bt::invalid([] { (void)integer_steps(1., 0., "period"); });
    ExperimentConfig asynchronous; asynchronous.async_telemetry = true;
    bt::invalid([&] { asynchronous.validate(); });
    ResearchRequest unavailable; unavailable.experiment.async_telemetry = true;
    // There is deliberately no physics/model/track to construct. The async
    // capability check must reject before touching a setup owner or cache.
    bool async_before_setup = false;
    try { (void)prepare_environment(unavailable); }
    catch (const std::invalid_argument& error) {
        async_before_setup = std::string_view(error.what()).find("native asynchronous publication (T42)") != std::string_view::npos;
    }
    bt::require(async_before_setup && !unavailable.setup.model.physics,
        "unsupported asynchronous publication rejects before model creation");
    bt::invalid([] { (void)integer_steps(std::numeric_limits<double>::infinity(), .00125, "period"); });
    const auto rider_document = bike::io::parse_json(R"({"reaction_delay_s":0.25,"keyframes":[
      {"time_s":0.0,"posture":{"torso_lean_rad":0.0,"pelvis_pitch_rad":0.0,"pelvis_offset_m":[0.0,0.0],"use_saddle":true},"human_torque_nm":10.0},
      {"time_s":1.0,"posture":{"torso_lean_rad":0.4,"pelvis_pitch_rad":0.2,"pelvis_offset_m":[0.02,-0.02],"use_saddle":false},"human_torque_nm":30.0}]})");
    const auto demand = bike::io::parse_json(R"({"keyframes":[{"time_s":0.0,"torque_nm":10.0},{"time_s":1.0,"torque_nm":30.0}]})");
    const auto programs = programs_from_documents(rider_document, demand);
    const auto before = programs.apply({}, 0.);
    const auto middle = programs.apply({}, .75);
    const auto knot = programs.apply({}, 1.25);
    bt::require(before.human_torque_nm == 10. && before.posture->use_saddle, "delay holds first keyframe");
    bt::require(middle.human_torque_nm == 20. && middle.posture->torso_lean_rad == .2,
        "quintic midpoint realizes posture and effort together");
    bt::require(middle.posture->use_saddle && !knot.posture->use_saddle, "discrete saddle request changes only at knot");
    bt::require(programs.demand_at(-1.) == 10. && programs.demand_at(.5) == 20. && programs.demand_at(2.) == 30.,
        "demand clamps time and retains final keyframe");
    const auto roundtrip = programs_from_documents(rider_program_value(programs), demand_program_value(programs));
    bt::require(runtime::controls_equal(programs.apply({}, .75), roundtrip.apply({}, .75)), "program metadata preserves controls");
    runtime::RideControl conflict; conflict.human_torque_nm = 0.;
    bt::invalid([&] { programs.validate_control(conflict); });
    conflict = {}; conflict.posture = ::rider::RiderPosture{};
    bt::invalid([&] { programs.validate_control(conflict); });
    for (const auto json : {
        R"({"keyframes":[]})",
        R"({"keyframes":[{"time_s":1.0}]})",
        R"({"keyframes":[{"time_s":0.0},{"time_s":0.0}]})",
        R"({"keyframes":[{"time_s":0.0,"human_torque_nm":0.0},{"time_s":1.0}]})",
        R"({"keyframes":[{"time_s":0.0,"posture":{"unknown":1.0}}]})"})
        bt::invalid([&] { (void)programs_from_documents(bike::io::parse_json(json), {}); });
    bt::invalid([] { (void)constant_demand(-1.); });
    bt::require(constant_demand(0.).demand_at(999.) == 0., "explicit zero demand remains present");
}
void seeded_inputs() {
    const std::string seed = "184467440737095516160012345";
    bt::require(validate_seed("000" + seed) == seed && validate_seed("000") == "0", "seed has arbitrary decimal precision");
    bt::invalid([] { (void)validate_seed("-1"); });
    bt::invalid([] { (void)validate_seed("1.0"); });
    SensorConfig config;
    const auto first = make_sensor_tape(config, seed, 8), repeat = make_sensor_tape(config, seed, 8);
    const auto other = make_sensor_tape(config, seed + "1", 8);
    for (std::size_t i = 0; i < first.noise.size(); ++i) same(first.noise[i], repeat.noise[i], "same seed has identical noise bits");
    same(first.dropout_uniform, repeat.dropout_uniform, "same seed has identical dropout bits");
    bt::require(first.noise != other.noise && first.dropout_uniform != other.dropout_uniform, "distinct seed moves both child streams");
    config.delivery.imu_enabled = false;
    const auto disabled = make_sensor_tape(config, seed, 8);
    bt::require(first.noise == disabled.noise, "disabled IMU still consumes its prescribed random draws");
    auto ideal = SensorConfig::ideal();
    auto tape = make_sensor_tape(ideal, seed, 4);
    runtime::SensorObservation raw;
    raw.values = {1., 2., 3., 4., 5., 6., 7., 8., 9.};
    const auto startup = startup_sensor_state(ideal, tape, raw);
    bt::require(startup.cursor == 1 && startup.samples_attempted == 1 && startup.samples_dropped == 0,
        "startup consumes one acquisition");
    runtime::SensorPipeline pipeline(ideal.delivery, tape);
    pipeline.import_state(startup);
    same(pipeline.read(0.).values, raw.values, "ideal zero-latency startup delivers raw sensor values");
    same(pipeline.read(0.).values, raw.values, "repeated delivery does not draw noise");
    bt::require(pipeline.state().cursor == 1, "delivery leaves RNG cursor unchanged");
    ideal.delivery.latency_s = .01;
    const auto delayed = startup_sensor_state(ideal, tape, raw);
    runtime::SensorPipeline latency(ideal.delivery, tape); latency.import_state(delayed);
    bt::require(!latency.read(0.).valid && latency.read(.01).valid, "startup respects nonzero latency");
    ideal.delivery.dropout_probability = 1.;
    const auto dropped = startup_sensor_state(ideal, tape, raw);
    bt::require(dropped.samples_attempted == 1 && dropped.samples_dropped == 1 && dropped.queue.empty(),
        "dropout never suppresses the acquisition draw");
    ResearchRequest a, b, c;
    a.rider_random = bike::random::RiderRandomSpec{};
    a.experiment.seed_decimal = seed;
    b = a; c = a; c.rider_seed_decimal = "9";
    sample_rider(a); sample_rider(b); sample_rider(c);
    bt::require(a.setup.model.rider->mass_kg == b.setup.model.rider->mass_kg &&
        bike::io::canonical_json(rider_program_value(a.programs)) == bike::io::canonical_json(rider_program_value(b.programs)),
        "rider and effort program reproduce with the same seed");
    bt::require(a.setup.model.rider->mass_kg != c.setup.model.rider->mass_kg, "independent rider seed changes the sampled rider");
    auto invalid = bike::random::RiderRandomSpec{}; invalid.mass_kg = {90., 80.};
    bt::invalid([&] { invalid.validate(); });
}
void native_construction() {
    auto request = read_research_request(bt::fixture("requests/research_delay.toml"));
    bt::require(request.sensors.sample_period_s == .005 && request.experiment.record_decimation == 80,
        "request assembly preserves source CLI defaults distinct from dataclass defaults");
    auto invalid = request; invalid.sensors.sample_period_s = .001;
    bt::invalid([&] { (void)prepare_environment(invalid); });
    invalid = request; invalid.setup.model.physics->drive_mode = "coast";
    bt::invalid([&] { (void)prepare_environment(invalid); });
    const auto field = research_field(request.setup.track, .005);
    bt::require(field.ncol > 2, "native terrain uses the requested road resolution");
    bt::invalid([&] { (void)research_field(request.setup.track, .0031); });
    const auto track = resolve_research_track(bt::fixture("tracks/flat.toml"), "unknown", std::nullopt, "999");
    bt::require(track.name == request.setup.track.name, "authored track takes precedence over scenario generation");
    request.capture_mode = runtime::ResearchCaptureMode::full;
    request.experiment.stop_on_model_violation = false;
    auto prepared = prepare_environment(request);
    bt::require(prepared.bootstrap.startup.cursor == 1 && prepared.bootstrap.config.raw_initial.has_value(),
        "native preparation owns one captured startup and its reset source");
    bt::require(prepared.bootstrap.config.assets && prepared.bootstrap.config.assets->size() >= 4,
        "recording captures immutable recipe bytes during native setup");
    auto environment = runtime::make_research(std::move(prepared.bootstrap));
    bt::require(environment->snapshot().step == 0 && environment->snapshot().time_s == 0., "source-free startup boundary");
    bt::require(environment->observations().size() == 1, "startup observation is retained once");
    const auto original = environment->snapshot().integration_state;
    environment->reset();
    same(original, environment->snapshot().integration_state, "native reset preserves exact initial integration state");
    environment->close();
    request.capture_mode = runtime::ResearchCaptureMode::off;
    request.experiment.actuator_delay_s = 0.; request.sensors.delivery.latency_s = 0.;
    auto zero = make_environment(request);
    bt::require(zero->configuration().delay_steps == 0 && !zero->configuration().assets,
        "zero delay and no-record mode are native setup choices");
    zero->close();
}
} // namespace
int main() { return bt::run([] { clocks_and_programs(); seeded_inputs(); native_construction(); }); }
