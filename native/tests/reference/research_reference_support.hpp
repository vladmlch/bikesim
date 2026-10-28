// Completion 3.33a — shared fixtures for the native-research batch.
//
// environment_factory's experiment knobs (control/actuator/sensor periods,
// decimation, seed, capture mode, strict flag) are mapped onto the fast
// flat-track physical recipe established by replay_reference_support's
// roundtrip_request(): dt .0005 physical + crank_effort + lumped rider +
// compliant_2d + equilibrium cache off. The welded articulated physics of
// viewer_physics_welded.toml is only the parity subject for the live-oracle
// legs, which stay planned; every contract ported in this batch is
// recipe-agnostic (step counting, reason strings, cursor semantics,
// validation domains, export refusal). Substitution documented in
// task-C333a-report.md.
#pragma once
#include "support/migration_test.hpp"
#include "configuration/model_config.hpp"
#include "configuration/rider_spec.hpp"
#include "random/numpy_compat.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "research/policy.hpp"
#include "research/policy_session.hpp"
#include "runtime/research.hpp"
#include "runtime/samples.hpp"
#include "runtime/sensors.hpp"
#include "terrain/profile.hpp"
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>
#include <unistd.h>

namespace research_reference {
namespace research = bike::research;
namespace sw = runtime::sample_wire;
using sw::array;
using sw::boolean;
using sw::find;
using sw::integer;
using sw::is_none;
using sw::number;
using sw::object;
using sw::required;
using sw::string;

// ---------------------------------------------------------------------------
// environment_factory knobs mapped onto the established fast recipe.
// ---------------------------------------------------------------------------
struct Knobs {
    double duration_s = .04;
    // Python defaults at dt .00125: control 8 / delay 4 / sensor 4 steps.
    // At dt .0005 the proportional defaults are control 4 / delay 2 / sensor 2;
    // every ported contract needs only delay < control and step counts > 1.
    double control_period_s = .002;
    double actuator_delay_s = .001;
    double sensor_period_s = .001;
    int decimation = 1;
    std::string seed = "19";
    runtime::ResearchCaptureMode capture = runtime::ResearchCaptureMode::legacy;
    bool strict = false;
    double energy_tolerance = .05;
};

[[nodiscard]] inline std::unique_ptr<research::ResearchRequest> request(const Knobs &k = {}) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &r = *owner;
    bike::configuration::SimulationPhysicsConfig physics;
    physics.physics_mode = "physical";
    physics.drive_mode = "crank_effort";
    physics.timestep_s = .0005;
    physics.initial_speed_mps = .1;
    physics.equilibrium_cache_enabled = false;
    physics.tires.backend = "compliant_2d";
    physics.tires.surface_mode = "track";
    bike::configuration::RiderSpecs rider;
    rider.variant = "lumped";
    r.setup.model.physics = physics;
    r.setup.model.rider = rider;
    r.setup.model.include_rider = true;
    r.setup.model.mode = "ride";
    r.setup.model.crank_joint = true;
    r.setup.track = bike::terrain::flat(12.);
    r.experiment.duration_s = k.duration_s;
    r.experiment.control_period_s = k.control_period_s;
    r.experiment.actuator_delay_s = k.actuator_delay_s;
    r.experiment.record_decimation = k.decimation;
    r.experiment.seed_decimal = k.seed;
    r.experiment.stop_on_model_violation = k.strict;
    r.experiment.maximum_energy_residual_ratio = k.energy_tolerance;
    r.sensors = research::SensorConfig::ideal();
    r.sensors.sample_period_s = k.sensor_period_s;
    r.road_resolution_m = .05;
    r.capture_mode = k.capture;
    return owner;
}

[[nodiscard]] inline std::unique_ptr<runtime::NativeResearchRuntime>
environment(const Knobs &k = {}) {
    return research::make_environment(*request(k));
}

// env.step(): one external control window — begin, advance to the completed
// transition, acknowledge. Mirrors NativeResearch.step in the Python adapter.
[[nodiscard]] inline runtime::ResearchTransition
step(runtime::NativeResearchRuntime &env, const runtime::RideControl &control,
     double front = 0., double rear = 0.) {
    env.begin_control(control, front, rear);
    std::optional<runtime::ResearchTransition> result;
    for (std::size_t guard = 0; guard < 4096 && !result; ++guard) {
        result = env.advance_control(std::nullopt, std::nullopt);
        bt::require(guard + 1 < 4096, "unbounded native control completes a physical window");
    }
    env.acknowledge_control();
    return *result;
}

// env.reset(seed=...): the reset_now() reseed path — rebuild the noise tape,
// delivered startup state, provenance metadata and assets under the chosen
// decimal seed, then hand the whole staged reset to the runtime. The staged
// pieces are returned so a test can substitute provenance before committing
// (the changed-execution port injects its mismatch on the stored-identity
// side — see test_native_research_recording.cpp).
struct StagedReset {
    runtime::NoiseTape tape;
    runtime::SensorState state;
    std::string seed;
    bike::io::Value metadata;
    runtime::ResearchAssets assets;
};
[[nodiscard]] inline StagedReset staged_reset(runtime::NativeResearchRuntime &env,
                                              std::string_view seed) {
    const auto config = env.configuration();
    const auto chosen = research::validate_seed(seed);
    bt::require(config.raw_initial.has_value(),
                "reseed requires a captured raw startup row");
    research::SensorConfig sensors;
    sensors.delivery = config.sensors;
    sensors.sample_period_s =
        static_cast<double>(config.sensor_steps) * config.timestep_s;
    sensors.noise = {.acceleration_std_mps2 = config.noise_std[0],
                     .gyro_std_rad_s = config.noise_std[1],
                     .encoder_std_rad_s = config.noise_std[2],
                     .torque_std_nm = config.noise_std[3]};
    StagedReset out{};
    out.tape = research::make_sensor_tape(
        sensors, chosen, 1 + config.max_steps / config.sensor_steps);
    out.seed = std::string(chosen);
    out.state = research::startup_sensor_state(sensors, out.tape, *config.raw_initial);
    out.metadata = research::reseed_metadata(config.metadata, chosen);
    out.assets = research::reseed_assets(config.assets, out.metadata);
    return out;
}
inline void reseed(runtime::NativeResearchRuntime &env, std::string_view seed) {
    auto staged = staged_reset(env, seed);
    env.reset(std::move(staged.tape), std::move(staged.state),
              std::move(staged.seed), std::move(staged.metadata),
              std::move(staged.assets));
}

// Status-object readers — the NativeResearchRuntime::status() wire is a
// WireObject; these name the two projections every port below needs.
[[nodiscard]] inline bool flag(const runtime::WireObject &status,
                               std::string_view name) {
    return sw::boolean(sw::required(status, name));
}
[[nodiscard]] inline std::optional<std::string> text(
    const runtime::WireObject &status, std::string_view name) {
    const auto &value = sw::required(status, name);
    if (sw::is_none(value)) return std::nullopt;
    return sw::string(value);
}

// ---------------------------------------------------------------------------
// Sensor-side helpers — the same raw shape the conftest raw_sensor fixture
// produces (the nine-column delivered observation).
// ---------------------------------------------------------------------------
[[nodiscard]] inline runtime::SensorObservation raw_sensor(double time_s = 0.) {
    runtime::SensorObservation result;
    result.time_s = time_s;
    result.source_time_s = time_s;
    result.valid = true;
    result.values = {.25, -1.5, 9.81, .03, 2., 3., 4., 5., 6.};
    return result;
}

// NativeSensorPipeline(config, tape) — the binding's two-argument form; the
// tape is built by the same build_noise_tape(config, seed, count) the Python
// fixture calls, then member-wise converted to the runtime tape shape.
[[nodiscard]] inline runtime::SensorPipeline pipeline(
    const runtime::SensorConfig &config,
    const bike::random::SensorNoiseConfig &noise,
    std::uint64_t seed, std::int64_t count) {
    const auto tape = bike::random::build_noise_tape(noise, seed, count);
    return runtime::SensorPipeline(config, {.noise = tape.noise,
        .dropout_uniform = tape.dropout_uniform});
}
// The default noise scales SensorConfig() declares; ideal() zeroes them.
[[nodiscard]] inline bike::random::SensorNoiseConfig default_noise() {
    return {};
}
[[nodiscard]] inline bike::random::SensorNoiseConfig ideal_noise() {
    return {.acceleration_std_mps2 = 0., .gyro_std_rad_s = 0.,
            .encoder_std_rad_s = 0., .torque_std_nm = 0.};
}

// The canonical flat row a golden site records: obs = 12 values.
inline void flatten_observation(const runtime::SensorObservation &obs,
                                std::vector<double> &out) {
    out.push_back(obs.time_s);
    out.push_back(obs.source_time_s);
    out.push_back(obs.valid ? 1. : 0.);
    out.insert(out.end(), obs.values.begin(), obs.values.end());
}

[[nodiscard]] inline std::vector<double> flatten_observation(
    const runtime::SensorObservation &obs) {
    std::vector<double> out;
    flatten_observation(obs, out);
    return out;
}

// The canonical flat row for a SensorPipeline::state_wire(): presence flags +
// scalar fields, then the 12-wide startup row (if any), the queue length and
// every queued observation row — identical layout to the Python capture
// site's flattening of state_dict().
inline void flatten_state(const runtime::WireObject &state,
                          std::vector<double> &out) {
    const auto optional_real = [&state, &out](std::string_view name) {
        const auto &value = required(state, name);
        if (is_none(value)) {
            out.push_back(0.);
            out.push_back(0.);
            return;
        }
        out.push_back(1.);
        out.push_back(number(value));
    };
    optional_real("last_time");
    optional_real("delivery_time");
    out.push_back(static_cast<double>(integer(required(state, "cursor"))));
    out.push_back(static_cast<double>(integer(required(state, "samples_attempted"))));
    out.push_back(static_cast<double>(integer(required(state, "samples_dropped"))));
    const auto &startup = required(state, "startup");
    out.push_back(is_none(startup) ? 0. : 1.);
    if (!is_none(startup))
        flatten_observation(runtime::SensorObservation::from_wire(object(startup)), out);
    const auto &queue = array(required(state, "queue"));
    out.push_back(static_cast<double>(queue.size()));
    for (const auto &row : queue)
        flatten_observation(runtime::SensorObservation::from_wire(object(row)), out);
}

[[nodiscard]] inline std::vector<double> flatten_state(const runtime::WireObject &state) {
    std::vector<double> out;
    flatten_state(state, out);
    return out;
}

[[nodiscard]] inline bool same_state(const runtime::WireObject &a,
                                     const runtime::WireObject &b) {
    return flatten_state(a) == flatten_state(b);
}

// Structural equality over the plain-data wire tree — used for
// "unchanged since" assertions where the Python source compares state_dict
// snapshots.
[[nodiscard]] inline bool wire_equal(const runtime::Wire &a,
                                     const runtime::Wire &b) {
    if (a.value.index() != b.value.index()) return false;
    if (const auto *x = std::get_if<runtime::WireArray>(&a.value)) {
        const auto &y = std::get<runtime::WireArray>(b.value);
        return x->size() == y.size() &&
               std::equal(x->begin(), x->end(), y.begin(),
                          [](const runtime::Wire &p, const runtime::Wire &q) {
                              return wire_equal(p, q); });
    }
    if (const auto *x = std::get_if<runtime::WireObject>(&a.value)) {
        const auto &y = std::get<runtime::WireObject>(b.value);
        if (x->size() != y.size()) return false;
        for (std::size_t i = 0; i < x->size(); ++i)
            if ((*x)[i].first != y[i].first ||
                !wire_equal((*x)[i].second, y[i].second))
                return false;
        return true;
    }
    if (const auto *x = std::get_if<double>(&a.value))
        return *x == std::get<double>(b.value);
    if (const auto *x = std::get_if<bool>(&a.value))
        return *x == std::get<bool>(b.value);
    if (const auto *x = std::get_if<std::int64_t>(&a.value))
        return *x == std::get<std::int64_t>(b.value);
    if (const auto *x = std::get_if<std::string>(&a.value))
        return *x == std::get<std::string>(b.value);
    return true; // both null
}

// RideControl(2.) — the Python single-positional form is motor_torque_nm.
[[nodiscard]] inline runtime::RideControl motor_control(double torque) {
    runtime::RideControl control;
    control.motor_torque_nm = torque;
    return control;
}

// ---------------------------------------------------------------------------
// CountingPolicy — the test_research_frontend/policy CountingPolicy double:
// records reset seeds and act() calls, returns one configurable command.
// ---------------------------------------------------------------------------
// Not final: the policy-failure and rider-smuggle ports subclass it.
class CountingPolicy : public research::Policy {
public:
    CountingPolicy() = default;
    ~CountingPolicy() override;
    CountingPolicy(const CountingPolicy &) = delete;
    CountingPolicy &operator=(const CountingPolicy &) = delete;
    CountingPolicy(CountingPolicy &&) = delete;
    CountingPolicy &operator=(CountingPolicy &&) = delete;
    void reset(std::string_view seed_decimal) override {
        seeds.emplace_back(seed_decimal);
    }
    [[nodiscard]] runtime::RideControl act(const runtime::SensorObservation &observation,
                                           std::optional<double> demand_nm) override {
        calls.emplace_back(observation, demand_nm);
        return command;
    }
    [[nodiscard]] bike::io::Value metadata() const override {
        if (meta) return *meta;
        return research::Policy::metadata();
    }
    runtime::RideControl command{.motor_torque_nm = 0.,
                                 .motor_limit_nm = std::nullopt,
                                 .human_torque_nm = std::nullopt,
                                 .crank_target_rate_rad_s = std::nullopt,
                                 .posture = std::nullopt,
                                 .rider_enabled = true};
    std::optional<bike::io::Value> meta;
    std::vector<std::string> seeds;
    std::vector<std::pair<runtime::SensorObservation, std::optional<double>>> calls;
};

// Message-fragment contract twins of bt::invalid. The Python sources pin
// pytest.raises(..., match=...) fragments; each helper asserts both the
// native exception class AND the exact fragment, so a paraphrased message
// fails the port.
template <class F>
[[nodiscard]] inline std::string thrown_message(F &&action) {
    try {
        std::forward<F>(action)();
    } catch (const std::exception &failure) {
        return failure.what();
    }
    bt::require(false, "expected an exception, got a clean return");
    return {};
}

template <class F>
void invalid_with(F &&action, std::string_view fragment) {
    bool rejected = false;
    std::string message;
    try {
        std::forward<F>(action)();
    } catch (const std::invalid_argument &failure) {
        rejected = true;
        message = failure.what();
    }
    bt::require(rejected, "expected std::invalid_argument (ValueError domain)");
    bt::require(message.find(fragment) != std::string::npos,
                "expected message fragment '" + std::string(fragment) +
                    "' in '" + message + "'");
}

// runtime_error contract: used where the Python source asserts
// pytest.raises(RuntimeError). Only std::runtime_error satisfies it; an
// std::invalid_argument propagates as a test failure instead of falsely
// matching the adapter's exception-class contract.
template <class F>
void throws_runtime(F &&action, std::string_view fragment = {}) {
    bool rejected = false;
    std::string message;
    try {
        std::forward<F>(action)();
    } catch (const std::runtime_error &failure) {
        if (dynamic_cast<const std::invalid_argument *>(&failure)) throw;
        rejected = true;
        message = failure.what();
    }
    bt::require(rejected, "expected std::runtime_error, got success or another type");
    if (!fragment.empty())
        bt::require(message.find(fragment) != std::string::npos,
                    "expected message fragment '" + std::string(fragment) +
                        "' in '" + message + "'");
}

// Temporary per-test directory (replay_reference_support's Directory twin).
struct Directory {
    std::filesystem::path path;
    Directory() {
        auto pattern = (std::filesystem::temp_directory_path() / "bike-research-reference-XXXXXX").string();
        std::vector<char> bytes(pattern.begin(), pattern.end());
        bytes.push_back('\0');
        const auto *made = ::mkdtemp(bytes.data());
        if (!made) throw std::system_error(errno, std::generic_category(), "test temporary directory");
        path = made;
    }
    ~Directory() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
    Directory(const Directory &) = delete;
    Directory &operator=(const Directory &) = delete;
    Directory(Directory &&) = delete;
    Directory &operator=(Directory &&) = delete;
};

} // namespace research_reference
