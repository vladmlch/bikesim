// test_t10.cpp — owned research bootstrap + runtime boundary regressions
// (task T10): runtime::read_research_bootstrap / decode_* and
// make_research exercised in the Python-free graph. This target's compile
// context carries no Python/nanobind include, so compiling
// runtime/research.hpp + runtime/research_io.hpp IS the include-closure
// check — binding headers are deliberately absent.
//
// The oracle halves (tests/fixtures/migration/research/delayed_bootstrap):
//   * env.json — the capture metadata (control/delay/sensor steps,
//     sensor envelope, --diagnostic-model-limits contract).
//   * research.json (+ research.arrays/) — the frozen native bootstrap:
//     the create_native_research() input staging verbatim under the
//     capture codec's externalized $array/$tuple tags.
//   * transitions.json — the Python-oracle env's 3 control windows of
//     RideControl(motor=10Nm) plus the startup + windowed observations.
//     The oracle ran production numba kernels, so transition values
//     compare by the assert_tree_close tolerance (1e-9); native-vs-native
//     reset replay compares bitwise (the same binary, the same state).
//   * config.json / state.json / model.mjb — the T09 ride bootstrap the
//     research runtime composes.
#include "support/migration_test.hpp"
#include "runtime/research.hpp"
#include "runtime/research_io.hpp"
#include "runtime/bootstrap_io.hpp"
#include "diag.hpp"
#include "engaged.hpp"
#include "io/json.hpp"
#include "io/value.hpp"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace {

namespace bio = bike::io;
namespace fs = std::filesystem;

[[nodiscard]] fs::path dir() {
    return bt::fixture("research/delayed_bootstrap");
}

[[nodiscard]] bio::Value parse_doc(const fs::path &path) {
    return bio::parse_json(bt::text(path));
}

[[nodiscard]] const bio::Value &member(const bio::Value &value,
                                       std::string_view key) {
    const auto *const object = std::get_if<bio::Value::Object>(&value.data);
    if (object == nullptr)
        throw std::invalid_argument("document member access on a non-object");
    const bio::Value *const found = bio::find_member(*object, key);
    if (found == nullptr)
        throw std::invalid_argument("document member missing: " +
                                    std::string(key));
    return *found;
}

[[nodiscard]] std::int64_t as_int(const bio::Value &value) {
    const auto *const number = std::get_if<bio::Number>(&value.data);
    if (number == nullptr ||
        !bio::is_integer_lexeme(number->lexeme))
        throw std::invalid_argument("expected an integer document member");
    std::int64_t out = 0;
    const char *const last = std::to_address(number->lexeme.cend());
    const auto parsed =
        std::from_chars(number->lexeme.data(), last, out, 10);
    if (parsed.ec != std::errc() || parsed.ptr != last)
        throw std::invalid_argument("integer document member out of range");
    return out;
}

[[nodiscard]] double as_real(const bio::Value &value) {
    const auto *const number = std::get_if<bio::Number>(&value.data);
    if (number == nullptr)
        throw std::invalid_argument("expected a real document member");
    bool ok = false;
    const double out = bio::detail::float_from_lexeme(number->lexeme, ok);
    if (!ok) throw std::invalid_argument("real document member malformed");
    return out;
}

[[nodiscard]] bool as_bool(const bio::Value &value) {
    const auto *const result = std::get_if<bool>(&value.data);
    if (result == nullptr)
        throw std::invalid_argument("expected a bool document member");
    return *result;
}

[[nodiscard]] const bio::Value::Array &as_array(const bio::Value &value) {
    const auto *const array = std::get_if<bio::Value::Array>(&value.data);
    if (array == nullptr)
        throw std::invalid_argument("expected an array document member");
    return *array;
}

[[nodiscard]] bool doc_is_null(const bio::Value &value) {
    return std::holds_alternative<std::monostate>(value.data);
}

[[nodiscard]] const std::string &as_string(const bio::Value &value) {
    const auto *const text = std::get_if<std::string>(&value.data);
    if (text == nullptr)
        throw std::invalid_argument("expected a string document member");
    return *text;
}

// assert_tree_close's leaf rule (atol=rtol=1e-9) — the transitions oracle
// ran production numba kernels, which may sit ~1 ulp off the pure-Python
// arithmetic the native port is bitwise-locked to.
[[nodiscard]] bool close(double actual, double expected) {
    return std::isfinite(actual) && std::isfinite(expected) &&
           std::abs(actual - expected) <= 1e-9 + 1e-9 * std::abs(expected);
}

void require_close(double actual, double expected, std::string_view what) {
    bt::require(close(actual, expected), what);
}

void require_bitwise(double actual, double expected, std::string_view what) {
    bt::require(std::bit_cast<std::uint64_t>(actual) ==
                    std::bit_cast<std::uint64_t>(expected),
                what);
}

// The std::runtime_error fail-fast contract (closed/double-ack cases) —
// bt::invalid pins std::invalid_argument only.
template <class F> void require_throws(F &&action, std::string_view what) {
    bool rejected = false;
    try {
        std::forward<F>(action)();
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    bt::require(rejected, what);
}

// The resolved (externalized tags already lowered) oracle documents.
[[nodiscard]] bio::Value resolved_doc(std::string_view name) {
    return runtime::resolve_bootstrap_document(parse_doc(dir() / name),
                                               dir());
}

// The 9-field SensorObservation <-> fixture dict comparison by wire name.
void require_observation(const runtime::SensorObservation &actual,
                         const bio::Value &expected, std::string_view what) {
    require_close(actual.time_s, as_real(member(expected, "time_s")), what);
    require_close(actual.source_time_s,
                  as_real(member(expected, "source_time_s")), what);
    bt::require(actual.valid == as_bool(member(expected, "valid")), what);
    const auto &accel =
        as_array(member(expected, "specific_force_body_mps2"));
    static constexpr std::array<std::string_view, 6> names{
        "pitch_rate_up_rad_s", "front_wheel_rad_s", "rear_wheel_rad_s",
        "crank_rad_s", "motor_torque_nm", "human_torque_nm"};
    bt::require(accel.size() == 3, what);
    for (std::size_t i = 0; i < 3; ++i)
        require_close(actual.values[i], as_real(accel.at(i)), what);
    for (std::size_t i = 0; i < names.size(); ++i)
        require_close(actual.values[i + 3],
                      as_real(member(expected, names[i])), what);
}

void require_transition(const runtime::ResearchTransition &actual,
                        const bio::Value &expected, std::string_view what) {
    bt::require(std::holds_alternative<runtime::SensorObservation>(actual.observation),
                "planar transition retains SensorObservation V1");
    require_observation(std::get<runtime::SensorObservation>(actual.observation), member(expected, "observation"),
                        what);
    bt::require(actual.physics_steps ==
                    as_int(member(expected, "physics_steps")),
                what);
    bt::require(actual.contact_state ==
                    as_string(member(expected, "contact_state")),
                what);
    bt::require(actual.terminated == as_bool(member(expected, "terminated")),
                what);
    bt::require(actual.truncated == as_bool(member(expected, "truncated")),
                what);
    bt::require(actual.model_valid == as_bool(member(expected, "model_valid")),
                what);
    bt::require(actual.numerically_valid ==
                    as_bool(member(expected, "numerically_valid")),
                what);
    bt::require(doc_is_null(member(expected, "reason")) ==
                    !actual.reason.has_value(),
                what);
    bt::require(doc_is_null(member(expected, "truth")) ==
                    !actual.truth.has_value(),
                what);
    if (actual.truth.has_value()) {
        const bio::Value &truth = member(expected, "truth");
        const auto &t = *actual.truth;
        const std::array<std::pair<double, std::string_view>, 16> fields{{
            {t.time_s, "time_s"},
            {t.front_load_n, "front_load_n"},
            {t.rear_load_n, "rear_load_n"},
            {t.front_clearance_m, "front_clearance_m"},
            {t.rear_clearance_m, "rear_clearance_m"},
            {t.relative_pitch_rad, "relative_pitch_rad"},
            {t.position_m, "position_m"},
            {t.speed_mps, "speed_mps"},
            {t.pitch_up_rad, "pitch_up_rad"},
            {t.pitch_rate_up_rad_s, "pitch_rate_up_rad_s"},
            {t.axle_pitch_rad, "axle_pitch_rad"},
            {t.road_pitch_rad, "road_pitch_rad"},
            {t.front_slip_mps, "front_slip_mps"},
            {t.rear_slip_mps, "rear_slip_mps"},
            {t.com_x_m, "com_x_m"},
            {t.com_z_m, "com_z_m"}}};
        for (const auto &[actual_value, key] : fields)
            require_close(actual_value,
                          as_real(member(truth, key)), what);
    }
}

// One complete external control window — the facade's step() triple. The
// pending-warning drain sits at the same position the adapter's held-GIL
// emission boundary does (after the completed advance, before ack).
[[nodiscard]] runtime::ResearchTransition
step(runtime::NativeResearchRuntime &run, const runtime::RideControl &control) {
    run.begin_control(control, 0., 0.);
    auto result = run.advance_control(std::nullopt, std::nullopt);
    bt::require(result.has_value(), "control window completes");
    static_cast<void>(run.take_pending_warning());
    run.acknowledge_control();
    return std::move(engaged(result));
}

// ---- the staged decode -----------------------------------------------------

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_read_bootstrap() {
    const auto boot = std::make_unique<runtime::ResearchBootstrap>(
        runtime::read_research_bootstrap(dir()));
    const bio::Value env = parse_doc(dir() / "env.json");

    // The ride half decodes through the T09 contract — model image owned,
    // config envelope intact.
    bt::require(!boot->ride.model.mjb.empty() &&
                    boot->ride.model.sha256.size() == 64,
                "ride bootstrap owns the model image");
    bt::require(boot->ride.state.step == 0 &&
                    boot->ride.config.timestep_s == 0.00125 &&
                    !boot->ride.config.strict,
                "ride config envelope decodes the welded pins");

    // The research envelope decodes the env.json metadata pins.
    const runtime::ResearchConfig &c = boot->config;
    bt::require(c.control_steps == as_int(member(env, "control_steps")) &&
                    c.delay_steps == as_int(member(env, "delay_steps")) &&
                    c.sensor_steps == as_int(member(env, "sensor_steps")),
                "research deadlines decode env.json");
    bt::require(c.max_steps == 80 && c.record_decimation == 80 &&
                    !c.stop_on_model_violation &&
                    c.capture_mode == runtime::ResearchCaptureMode::legacy,
                "research capture/limits decode env.json");
    const bio::Value &config = member(env, "config");
    require_close(c.timestep_s, 0.00125, "research timestep decodes");
    require_close(c.maximum_energy_residual_ratio,
                  as_real(member(config, "maximum_energy_residual_ratio")),
                  "energy residual policy decodes");
    require_close(c.wheelie_persistence_s,
                  as_real(member(config, "wheelie_persistence_s")),
                  "wheelie persistence decodes");
    const bio::Value &sensors = member(env, "sensors");
    require_close(c.sensors.latency_s,
                  as_real(member(sensors, "latency_s")), "latency decodes");
    require_close(c.sensors.maximum_age_s,
                  as_real(member(sensors, "maximum_age_s")),
                  "maximum age decodes");
    bt::require(c.sensors.imu_enabled ==
                    as_bool(member(sensors, "imu_enabled")),
                "imu enablement decodes");
    bt::require(c.startup_control.motor_torque_nm.has_value() &&
                    *c.startup_control.motor_torque_nm == 0. &&
                    c.startup_control.rider_enabled,
                "startup control decodes the applied t=0 command");
    bt::require(c.geometry.x.size() == 24001 &&
                    c.geometry.z.size() == 24001 &&
                    c.geometry.root_x_qpos == 0 &&
                    c.geometry.root_pitch_dof == 2,
                "truth geometry decodes the terrain/addresses");
    bt::require(c.programs.rider_frames.empty() &&
                    c.programs.demand_frames.empty(),
                "null programs decode to empty programs");

    // The setup-generated noise tape owns 1 + floor(80/4) rows.
    bt::require(boot->tape.noise.size() == 21 &&
                    boot->tape.dropout_uniform.size() == 21,
                "noise tape owns the setup row count");
    // The startup sensor state arrives imported, not replayed: the Python
    // setup already pushed and delivered row zero — cursor 1, one
    // attempted sample, the delivered invalid startup sample and one
    // queued noisy raw at t=0.
    const runtime::SensorState &s = boot->startup;
    bt::require(s.cursor == 1 && s.samples_attempted == 1 &&
                    s.samples_dropped == 0 && s.queue.size() == 1,
                "startup sensor state owns the delivered row boundary");
    bt::require(s.last_time.has_value() && *s.last_time == 0. &&
                    s.delivery_time.has_value() && *s.delivery_time == 0. &&
                    s.startup.has_value() && !s.startup->valid &&
                    s.startup->time_s == 0.,
                "startup sensor state pins the t=0 delivery");
    // The queued raw is exactly the first transition's observation (the
    // latency delivery the oracle recorded at t=0.01) — apart from
    // time_s, which read() restamps to the delivery clock.
    const bio::Value oracle = resolved_doc("transitions.json");
    const auto queued = std::make_unique<runtime::SensorObservation>(
        s.queue.front());
    queued->time_s = 0.01;
    require_observation(
        *queued,
        member(as_array(member(oracle, "transitions")).front(),
               "observation"),
        "queued startup sample equals the oracle's first delivery");
}
NATIVE_DIAG_POP

// ---- construction, lifecycle and the oracle transitions ---------------------

// By-value bootstrap returns materialize on this frame even boxed
// afterwards — the budget waiver mirrors the decode frames in runtime.cpp.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_construct_and_transitions() {
    const auto run = runtime::make_research(
        runtime::read_research_bootstrap(dir()));

    const auto at_zero =
        std::make_unique<const runtime::RuntimeSnapshot>(run->snapshot());
    bt::require(at_zero->step == 0 && at_zero->time_s == 0. &&
                    at_zero->generation == 1,
                "research starts at t=0");

    // The imported startup boundary: the t=0 delivery was consumed by the
    // Python setup — the native pipeline must not observe a second row.
    {
        using namespace runtime::sample_wire;
        const auto status = run->status();
        bt::require(number(required(status, "noise_cursor")) == 1 &&
                        number(required(status, "samples_attempted")) == 1 &&
                        number(required(status, "step")) == 0,
                    "startup state imported without double-consuming");
    }

    const bio::Value oracle = resolved_doc("transitions.json");
    const auto &oracle_transitions =
        as_array(member(oracle, "transitions"));
    const auto &oracle_observations =
        as_array(member(oracle, "observations"));
    bt::require(oracle_transitions.size() == 3 &&
                    oracle_observations.size() == 4,
                "oracle fixture carries 3 windows / 4 observations");

    // The --diagnostic-model-limits capture runs non-strict: the first
    // advance surfaces exactly one pending reference rejection — the
    // RuntimeWarning the adapter emits at the held-GIL boundary — and it
    // drains once.
    runtime::RideControl command;
    command.motor_torque_nm = 10.;
    run->begin_control(command, 0., 0.);
    auto first_result = run->advance_control(std::nullopt, std::nullopt);
    bt::require(first_result.has_value(), "first window completes");
    const auto warning = run->take_pending_warning();
    bt::require(warning.has_value() && !warning->empty(),
                "pending reference rejection drains through the seam");
    bt::require(!run->take_pending_warning().has_value(),
                "the rejection drains exactly once");
    run->acknowledge_control();
    require_transition(engaged(first_result), oracle_transitions.front(),
                       "first transition matches the Python oracle");
    for (std::size_t i = 1; i < oracle_transitions.size(); ++i) {
        const auto transition = step(*run, command);
        require_transition(transition, oracle_transitions.at(i),
                           "transition matches the Python oracle");
    }
    // The recorded history matches the oracle's observation sequence.
    const auto history = run->observations();
    bt::require(history.size() == oracle_observations.size(),
                "observation history length matches the oracle");
    for (std::size_t i = 0;
         i < oracle_observations.size() && i < history.size(); ++i)
        require_observation(
            runtime::SensorObservation::from_wire(
                runtime::sample_wire::object(history[i])),
            oracle_observations.at(i),
            "recorded observation matches the Python oracle");
    // The recorded transition prefix carries the completed windows — the
    // same wire rows the Python facade boxes into env.trace.
    const auto transitions = run->transitions();
    bt::require(transitions.size() == oracle_transitions.size(),
                "recorded transitions length matches the oracle");
    for (std::size_t i = 0;
         i < oracle_transitions.size() && i < transitions.size(); ++i) {
        const auto &row = runtime::sample_wire::object(transitions[i]);
        require_observation(
            runtime::SensorObservation::from_wire(runtime::sample_wire::object(
                runtime::sample_wire::required(row, "observation"))),
            member(oracle_transitions.at(i), "observation"),
            "recorded transition matches the Python oracle");
        bt::require(
            runtime::sample_wire::number(
                runtime::sample_wire::required(row, "physics_steps")) ==
                static_cast<double>(as_int(
                    member(oracle_transitions.at(i), "physics_steps"))),
            "recorded transition physics step count matches the oracle");
    }
    const auto snap = run->snapshot();
    bt::require(snap.step == 24 && close(snap.time_s, 0.03),
                "three windows commit 24 physics steps");
    run->close();
    bt::require(run->closed(), "closed research owner");
    require_throws([&] { static_cast<void>(run->snapshot()); },
                   "closed runtime fails fast");
}
NATIVE_DIAG_POP

// ---- pacing yields at the observation boundary ------------------------------

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_yield_and_pacing() {
    const auto run = runtime::make_research(
        runtime::read_research_bootstrap(dir()));
    runtime::RideControl command;
    command.motor_torque_nm = 10.;
    run->begin_control(command, 0., 0.);
    // The control window (8 steps) spans the first sensor acquisition
    // boundary at step 4. A zero wall budget yields before any committed
    // step — no flush, delivery or transition.
    bt::require(!run->advance_control(std::optional<double>(0.),
                                      std::nullopt)
                     .has_value(),
                "zero wall budget yields before the first interval");
    // A pacing yield at the sensor boundary leaves the window latched.
    bt::require(!run->advance_control(std::nullopt,
                                      std::optional<std::int64_t>(4))
                     .has_value(),
                "pacing yield at the observation boundary");
    const auto mid = run->snapshot();
    bt::require(mid.step == 4,
                "pacing yield stops at the requested step");
    // Resume across the boundary: the same window completes and the
    // delivered transition covers all 8 physics steps.
    auto result = run->advance_control(std::nullopt, std::nullopt);
    bt::require(result.has_value() && result->physics_steps == 8,
                "the latched window completes across the yield");
    static_cast<void>(run->take_pending_warning());
    run->acknowledge_control();
    // The ack drains the staged transition exactly once — a second ack
    // fails fast, it never double-commits.
    require_throws([&] { run->acknowledge_control(); },
                   "double acknowledgement fails fast");
    run->close();
}
NATIVE_DIAG_POP

// ---- stop drains only an already accepted window ----------------------------

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_stop_semantics() {
    const auto run = runtime::make_research(
        runtime::read_research_bootstrap(dir()));
    runtime::RideControl command;
    command.motor_torque_nm = 10.;
    run->begin_control(command, 0., 0.);
    run->stop();
    // The already accepted window still drains: the delivered transition
    // completes all 8 physics steps; the stop lands at the ack boundary —
    // the same status patch the Python facade publishes.
    auto result = run->advance_control(std::nullopt, std::nullopt);
    bt::require(result.has_value() && result->physics_steps == 8,
                "stop drains the accepted window, it does not reject it");
    static_cast<void>(run->take_pending_warning());
    {
        using namespace runtime::sample_wire;
        const auto status = run->status();
        bt::require(boolean(required(status, "stop_requested")),
                    "stop request stays pending until acknowledgement");
    }
    run->acknowledge_control();
    {
        using namespace runtime::sample_wire;
        const auto status = run->status();
        const auto &reason = required(status, "reason");
        bt::require(boolean(required(status, "truncated")) &&
                        std::holds_alternative<std::string>(reason.value) &&
                        std::get<std::string>(reason.value) ==
                            "operator_stop" &&
                        !boolean(required(status, "stop_requested")),
                    "ack applies the operator stop exactly once");
    }
    // An idle episode truncates immediately instead.
    const auto idle = runtime::make_research(
        runtime::read_research_bootstrap(dir()));
    idle->stop();
    {
        using namespace runtime::sample_wire;
        const auto status = idle->status();
        const auto &reason = required(status, "reason");
        bt::require(boolean(required(status, "truncated")) &&
                        std::holds_alternative<std::string>(reason.value) &&
                        std::get<std::string>(reason.value) ==
                            "operator_stop",
                    "idle stop truncates in place");
    }
    idle->close();
    run->close();
}
NATIVE_DIAG_POP

// ---- reset restores the startup boundary without double-consuming -----------

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_reset_semantics() {
    // Two owned decodes: the runtime consumes one; its untouched twin is
    // the reset tape/startup — reset() must not re-observe the bootstrap.
    const auto bootstrap = std::make_unique<runtime::ResearchBootstrap>(
        runtime::read_research_bootstrap(dir()));
    const auto run = runtime::make_research(
        runtime::read_research_bootstrap(dir()));
    runtime::RideControl command;
    command.motor_torque_nm = 10.;
    const auto first = step(*run, command);
    run->reset(runtime::NoiseTape{bootstrap->tape},
               runtime::SensorState{bootstrap->startup});
    {
        using namespace runtime::sample_wire;
        const auto status = run->status();
        // The imported cursor/attempts return to the startup row — the
        // reset imports state, it does not replay push() a second time.
        bt::require(number(required(status, "noise_cursor")) == 1 &&
                        number(required(status, "samples_attempted")) == 1 &&
                        number(required(status, "step")) == 0 &&
                        !boolean(required(status, "terminated")) &&
                        !boolean(required(status, "truncated")),
                    "reset restores the startup sensor boundary");
    }
    const auto replay = step(*run, command);
    // Native-vs-native replay is bitwise: same binary, same staged state.
    bt::require(replay.physics_steps == first.physics_steps,
                "reset replay commits the same window");
    bt::require(std::holds_alternative<runtime::SensorObservation>(replay.observation) &&
                    std::holds_alternative<runtime::SensorObservation>(first.observation),
                "planar reset retains the V1 sensor schema");
    const auto& replay_observation = std::get<runtime::SensorObservation>(replay.observation);
    const auto& first_observation = std::get<runtime::SensorObservation>(first.observation);
    for (std::size_t i = 0; i < replay_observation.values.size(); ++i)
        require_bitwise(replay_observation.values[i],
                        first_observation.values[i],
                        "reset replay observation is bitwise-identical");
    require_bitwise(replay_observation.time_s, first_observation.time_s,
                    "reset replay observation clock is bitwise-identical");
    require_bitwise(replay_observation.source_time_s,
                    first_observation.source_time_s,
                    "reset replay source clock is bitwise-identical");
    run->close();
}
NATIVE_DIAG_POP

// ---- an engine failure fabricates no successful transition ------------------

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_engine_failure() {
    const auto run = runtime::make_research(
        runtime::read_research_bootstrap(dir()));
    runtime::RideControl command;
    command.motor_torque_nm = 10.;
    run->test_fail_at_step(2);
    run->begin_control(command, 0., 0.);
    bool threw = false;
    try {
        static_cast<void>(run->advance_control(std::nullopt, std::nullopt));
    } catch (const std::runtime_error &) {
        threw = true;
    }
    bt::require(threw, "engine failure propagates out of advance_control");
    {
        using namespace runtime::sample_wire;
        const auto status = run->status();
        const auto &reason = required(status, "reason");
        bt::require(boolean(required(status, "terminated")) &&
                        std::holds_alternative<std::string>(reason.value) &&
                        std::get<std::string>(reason.value) ==
                            "simulation_error" &&
                        !runtime::sample_wire::is_none(required(status, "error")) &&
                        !boolean(required(status, "delivery_pending")) &&
                        !boolean(required(status, "active")),
                    "engine failure records the failure evidence");
        bt::require(number(required(status, "completed_count")) == 0,
                    "no completed-control prefix is fabricated");
    }
    bt::require(run->transitions().empty(),
                "no successful transition is fabricated");
    run->close();
}
NATIVE_DIAG_POP

// ---- acquisition faults ------------------------------------------------------

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_acquisition_faults() {
    bt::invalid([] {
        static_cast<void>(runtime::read_research_bootstrap(
            dir() / "does_not_exist"));
    });
    bt::invalid([] {
        // tests/fixtures/migration/research exists but carries no
        // research.json — the read fails at its first document, never
        // guesses defaults.
        static_cast<void>(runtime::read_research_bootstrap(
            bt::fixture("research")));
    });
    // A truncated tape composes against the valid ride bootstrap and the
    // ctor's row-count gate rejects at the same contract position the
    // binding's parse_tape + ctor checks covered.
    auto bootstrap = runtime::read_research_bootstrap(dir());
    bootstrap.tape.noise.pop_back();
    bootstrap.tape.dropout_uniform.pop_back();
    bt::invalid([&] {
        static_cast<void>(runtime::make_research(std::move(bootstrap)));
    });
    // The startup-state import contract: a default SensorState (cursor 0)
    // fails the delivered-startup-row check the episode ctor runs.
    auto next = runtime::read_research_bootstrap(dir());
    next.startup = runtime::SensorState{};
    bt::invalid([&] {
        static_cast<void>(runtime::make_research(std::move(next)));
    });
}
NATIVE_DIAG_POP

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(checks_read_bootstrap);
        failures += bt::run(checks_construct_and_transitions);
        failures += bt::run(checks_yield_and_pacing);
        failures += bt::run(checks_stop_semantics);
        failures += bt::run(checks_reset_semantics);
        failures += bt::run(checks_engine_failure);
        failures += bt::run(checks_acquisition_faults);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
