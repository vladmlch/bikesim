#include "policy_session.hpp"
#include <cstdint>
#include "evidence.hpp"
#include "../io/json.hpp"
#include "../platform/provenance.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <exception>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace bike::research {
namespace {
using namespace runtime::setupvalue;
bool flag(const runtime::WireObject& status, std::string_view name) {
    return runtime::sample_wire::boolean(runtime::sample_wire::required(status, name));
}
void brakes(double front, double rear) {
    for (const double value : {front, rear})
        if (!std::isfinite(value) || value < 0. || value > 1.) throw std::invalid_argument("brake demand must lie in [0, 1]");
}
bool done_status(const runtime::WireObject& status) { return flag(status, "terminated") || flag(status, "truncated"); }
} // namespace

struct PolicySession::Impl {
    // NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
    enum class Action : std::uint8_t { stop, pause, resume, brakes, reset };
    struct Queued { Action action; double front = 0., rear = 0.; std::optional<std::string> seed; };
    struct Command { runtime::RideControl control; double front = 0., rear = 0.; };
    std::unique_ptr<runtime::NativeResearchRuntime> environment;
    std::unique_ptr<Policy> policy;
    std::unique_ptr<RiderBehavior> rider_behavior;
    runtime::ResearchConfig config;
    std::optional<Command> staged;
    std::optional<runtime::ResearchTransition> completed;
    bool completed_acknowledged = false;
    std::deque<Queued> queued;
    bool paused = false, closed = false, intervention = false;
    double front = 0., rear = 0.;
    std::vector<OperatorEvent> events;
    Array brake_events;
    std::vector<std::string> cleanup_notes;
    io::Value policy_metadata, launch_metadata{Object{}};
    std::function<void()> before_reset, after_reset;

    [[nodiscard]] runtime::WireObject status() const {
        if (closed) throw std::runtime_error("policy session is closed");
        return environment->status();
    }
    [[nodiscard]] bool pending() const {
        if (completed) return true;
        const auto s = status();
        return flag(s, "active") || flag(s, "delivery_pending");
    }
    void note_cleanup(std::string_view operation) noexcept {
        try {
            // note_cleanup is invoked only from catch handlers during exception
            // unwinding, so the bare `throw;` legally rethrows the in-flight
            // exception for inspection.
            // cppcheck-suppress rethrowNoCurrentException
            try { throw; }
            catch (const std::exception& error) { cleanup_notes.push_back(std::string(operation) + ": " + error.what()); }
            catch (...) { cleanup_notes.push_back(std::string(operation) + ": non-standard exception"); }
        // NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
        } catch (...) { /* No allocation failure may replace the primary error. */ }
    }
    void mark_policy_failure(std::string_view message) noexcept {
        try { environment->fail_policy(std::string(message)); }
        catch (...) { note_cleanup("recording the policy failure failed"); }
    }
    void start_policy() {
        try {
            policy->reset(config.seed_decimal);
            if (rider_behavior) rider_behavior->reset(config.seed_decimal);
            policy_metadata = policy->metadata();
        } catch (const std::exception& failure) {
            mark_policy_failure(failure.what());
            throw;
        } catch (...) {
            mark_policy_failure("policy reset raised a non-standard exception");
            throw;
        }
    }
    void record(std::string kind, const Queued& request, bool queued_value, bool applied) {
        if (config.capture_mode != runtime::ResearchCaptureMode::full && config.capture_mode != runtime::ResearchCaptureMode::legacy) return;
        const auto s = status();
        const bool terminal = done_status(s);
        std::string_view action;
        Value value;
        switch (request.action) {
            case Action::stop: action = "stop"; break;
            case Action::pause: action = "pause"; value = Value{true}; break;
            case Action::resume: action = "pause"; value = Value{false}; break;
            case Action::brakes: action = "brakes"; value = Value{Array{real(request.front), real(request.rear)}}; break;
            case Action::reset:
                action = "reset";
                if (request.seed) value = Value{io::Number{*request.seed}};
                break;
        }
        Value values{Object{{"action", text(action)}, {"value", std::move(value)},
            {"queued", Value{queued_value}}, {"terminal", Value{terminal}}}};
        events.push_back(OperatorEvent{.event_id=static_cast<std::int64_t>(events.size()),
            .processed_step=runtime::sample_wire::integer(runtime::sample_wire::required(s, "step")),
            .time_s=runtime::sample_wire::number(runtime::sample_wire::required(s, "time_s")),
            .received_wall_s=platform::monotonic_seconds(), .kind=std::move(kind), .queued=queued_value, .terminal=terminal, .applied=applied, .values=std::move(values)});
    }
    void reset_now(const std::optional<std::string>& seed) {
        const auto chosen = seed ? validate_seed(*seed) : config.seed_decimal;
        // Stage the complete deterministic tape and delivery state first.
        std::optional<runtime::ResearchNoiseTape> tape;
        std::optional<runtime::ResearchSensorState> state;
        if(config.raw_initial_v2&&config.sensors_v2) {
            auto generated=make_sensor_tape_v2(*config.sensors_v2,chosen,1+config.max_steps/config.sensor_steps);
            runtime::spatial::SensorPipelineV2 pipeline(*config.sensors_v2,generated);
            pipeline.reset(*config.raw_initial_v2);(void)pipeline.read(0.);state=pipeline.state();tape=std::move(generated);
        } else if (config.raw_initial) {
            SensorConfig sensors;
            sensors.delivery = config.sensors;
            sensors.sample_period_s = static_cast<double>(config.sensor_steps) * config.timestep_s;
            sensors.noise = {.acceleration_std_mps2=config.noise_std[0], .gyro_std_rad_s=config.noise_std[1], .encoder_std_rad_s=config.noise_std[2], .torque_std_nm=config.noise_std[3]};
            tape = make_sensor_tape(sensors, chosen, 1 + config.max_steps / config.sensor_steps);
            state = startup_sensor_state(sensors, std::get<runtime::NoiseTape>(*tape), *config.raw_initial);
        } else if (chosen != config.seed_decimal) {
            throw std::invalid_argument("a captured bootstrap without raw startup sensors cannot be reseeded; construct a native environment");
        }
        auto next_metadata = reseed_metadata(config.metadata, chosen);
        auto next_assets = reseed_assets(config.assets, next_metadata);
        if (before_reset) before_reset();
        if (tape) environment->reset(std::move(*tape), std::move(*state), chosen, next_metadata, next_assets); // NOLINT(bugprone-unchecked-optional-access) a captured state accompanies every tape
        else environment->reset();
        config.seed_decimal = chosen;
        config.metadata = std::move(next_metadata);
        config.assets = std::move(next_assets);
        events.clear(); brake_events.clear(); paused = false; front = rear = 0.; intervention = false;
        staged.reset(); launch_metadata = Value{Object{}};
        if (after_reset) after_reset();
        start_policy();
    }
    void apply(const Queued& request) {
        const bool terminal_before = done_status(status());
        switch (request.action) {
            case Action::stop: staged.reset(); environment->stop(); break;
            case Action::pause: paused = true; break;
            case Action::resume: paused = false; break;
            case Action::brakes: front = request.front; rear = request.rear; break;
            case Action::reset: reset_now(request.seed); break;
        }
        record(terminal_before ? "operator_after_terminal" : "operator_applied", request, false, !terminal_before);
    }
    void request(Queued request) {
        const bool wait = pending();
        record("operator_requested", request, wait, false);
        if (wait) queued.push_back(std::move(request));
        else apply(request);
    }
    void apply_queued() {
        while (!queued.empty()) {
            apply(queued.front());
            queued.pop_front();
        }
    }
};

PolicySession::PolicySession(std::unique_ptr<runtime::NativeResearchRuntime>&& environment,
                            std::unique_ptr<Policy> policy, std::unique_ptr<RiderBehavior> behavior)
    : impl_(std::make_unique<Impl>()) {
    if (!environment || !policy) throw std::invalid_argument("policy session requires owned environment and policy");
    impl_->config = environment->configuration();
    if (behavior && !impl_->config.programs.rider_frames.empty())
        throw std::invalid_argument("rider behavior and a posture-owning program are exclusive");
    impl_->environment = std::move(environment); impl_->policy = std::move(policy); impl_->rider_behavior = std::move(behavior);
    try { impl_->start_policy(); }
    catch (...) {
        environment = std::move(impl_->environment);
        throw;
    }
}
PolicySession::~PolicySession() = default;
bool PolicySession::pending() const { return impl_->pending(); }
bool PolicySession::paused() const { (void)impl_->status(); return impl_->paused; }
bool PolicySession::done() const { return done_status(impl_->status()); }
runtime::NativeResearchRuntime& PolicySession::environment() { return *impl_->environment; }
const runtime::NativeResearchRuntime& PolicySession::environment() const { return *impl_->environment; }

void PolicySession::begin_advance(std::optional<double> front, std::optional<double> rear) {
    auto& s = *impl_;
    if (s.pending()) throw std::runtime_error("an external policy interval is already active");
    const auto status = s.status();
    if (s.paused || done_status(status)) throw std::runtime_error("cannot start a policy interval while paused or ended");
    const double f = front.value_or(s.front), r = rear.value_or(s.rear);
    brakes(f, r);
    if (s.staged) {
        if (s.staged->front != f || s.staged->rear != r)
            throw std::runtime_error("retry a staged policy command with its original brakes");
    } else {
        try {
            const auto observation = runtime::research_observation_from_wire(runtime::sample_wire::object(
                runtime::sample_wire::required(status, "observation")));
            const auto& requested = runtime::sample_wire::required(status, "demand_nm");
            const auto demand = runtime::sample_wire::is_none(requested) ? std::nullopt
                : std::optional(runtime::sample_wire::number(requested));
            auto command = std::visit([&](const auto& measured){
                if constexpr(std::is_same_v<std::decay_t<decltype(measured)>,runtime::SensorObservation>)return s.policy->act(measured,demand);
                else return s.policy->act_v2(measured,demand);},observation);
            validate_motor_policy_control(command);
            if (s.rider_behavior) command.posture = s.rider_behavior->act(std::visit([](const auto& measured){return measured.time_s;},observation),
                signals_from_sample(s.environment->snapshot().latest_sample));
            s.config.programs.validate_control(command);
            runtime::control_validate_for(command, true);
            s.staged = Impl::Command{.control=command, .front=f, .rear=r};
        } catch (const std::exception& error) { s.mark_policy_failure(error.what()); throw; }
        catch (...) { s.mark_policy_failure("policy act raised a non-standard exception"); throw; }
    }
    std::optional<Value> brake_event;
    if ((f != 0. || r != 0.) && (s.config.capture_mode == runtime::ResearchCaptureMode::legacy ||
                               s.config.capture_mode == runtime::ResearchCaptureMode::full)) {
        brake_event = Value{Object{{"time_s", real(runtime::sample_wire::number(runtime::sample_wire::required(status, "time_s")))},
            {"front_brake_demand", real(f)}, {"rear_brake_demand", real(r)}}};
        s.brake_events.reserve(s.brake_events.size() + 1);
    }
    // Allocation failure here keeps the already computed command. Retrying
    // begin_advance only retries native acceptance; it never calls act again.
    s.environment->begin_control(s.staged->control, s.staged->front, s.staged->rear);
    s.staged.reset();
    if (brake_event) s.brake_events.push_back(std::move(*brake_event));
    if (f != 0. || r != 0.) s.intervention = true;
}

std::optional<runtime::ResearchTransition> PolicySession::advance_pending(
    std::optional<double> budget, std::optional<std::int64_t> target) {
    auto& s = *impl_;
    if (!s.completed) {
        if (!s.pending()) throw std::runtime_error("begin_advance is required before advance_pending");
        auto result = s.environment->advance_control(budget, target);
        if (!result) return std::nullopt;
        s.completed = std::move(result);
    }
    if (!s.completed_acknowledged) {
        s.environment->acknowledge_control();
        s.completed_acknowledged = true;
    }
    s.apply_queued();
    auto result = std::move(s.completed);
    s.completed.reset();
    s.completed_acknowledged = false;
    return result;
}
runtime::ResearchTransition PolicySession::advance(std::optional<double> front, std::optional<double> rear) {
    begin_advance(front, rear);
    auto result = advance_pending();
    while (!result) result = advance_pending();
    return std::move(*result);
}
void PolicySession::run_until(std::optional<double> wall_limit) {
    if (wall_limit && (!std::isfinite(*wall_limit) || *wall_limit < 0.))
        throw std::invalid_argument("wall limit must be finite and nonnegative");
    const auto start = platform::monotonic_seconds();
    while (!done() && !paused()) {
        if (wall_limit && platform::monotonic_seconds() - start >= *wall_limit) { request_stop(); break; }
        if (!pending()) begin_advance();
        (void)advance_pending(.008);
    }
    while (pending()) (void)advance_pending(.008);
}
void PolicySession::request_stop() { impl_->request({.action=Impl::Action::stop, .front=0., .rear=0., .seed={}}); }
void PolicySession::pause() { impl_->request({.action=Impl::Action::pause, .front=0., .rear=0., .seed={}}); }
void PolicySession::resume() { impl_->request({.action=Impl::Action::resume, .front=0., .rear=0., .seed={}}); }
void PolicySession::set_brakes(double front, double rear) { brakes(front, rear); impl_->request({.action=Impl::Action::brakes, .front=front, .rear=rear, .seed={}}); }
void PolicySession::reset(std::optional<std::string> seed) {
    if (seed) *seed = validate_seed(*seed);
    if (!impl_->pending()) impl_->queued.clear();
    impl_->request({.action=Impl::Action::reset, .front=0., .rear=0., .seed=std::move(seed)});
}
io::Value PolicySession::run_metadata() const {
    Value out = impl_->launch_metadata;
    set(out, "policy", impl_->policy_metadata);
    set(out, "operator_intervention", Value{impl_->intervention});
    // The historical summary records accepted brake interventions here;
    // lifecycle requests/applies are a separate ordered publication journal.
    set(out, "operator_events", Value{impl_->brake_events});
    Array notes;
    for (const auto& note : impl_->cleanup_notes) notes.push_back(text(note));
    set(out, "cleanup_notes", Value{std::move(notes)});
    return out;
}
runtime::OwnedResearchEpisode PolicySession::seal_episode() {
    auto& s = *impl_;
    auto metadata = run_metadata();
    try {
        // The recursive policy metadata also covers a plugin wrapped by a
        // builtin traction controller. It contains no mutable policy state.
        const auto current = s.policy->metadata();
        set(metadata, "policy_source_changed_during_run", Value{io::canonical_json(s.policy_metadata) != io::canonical_json(current)});
    } catch (const std::exception& error) {
        set(metadata, "policy_source_changed_during_run", Value{true});
        set(metadata, "policy_identity_check_error", text(error.what()));
    }
    set(metadata, "identity_checks", identity_receipt(s.config.metadata));
    auto episode = s.config.capture_mode == runtime::ResearchCaptureMode::full
        ? s.environment->seal_episode() : s.environment->evidence_snapshot();
    episode.run_metadata = std::move(metadata);
    return episode;
}
const std::vector<OperatorEvent>& PolicySession::operator_events() const { return impl_->events; }
const std::vector<std::string>& PolicySession::cleanup_notes() const noexcept { return impl_->cleanup_notes; }
void PolicySession::set_reset_hooks(std::function<void()> before, std::function<void()> after) {
    impl_->before_reset = std::move(before); impl_->after_reset = std::move(after);
}
void PolicySession::close() {
    auto& s = *impl_;
    if (s.closed) return;
    if (s.pending()) throw std::runtime_error("finish the accepted policy interval before close");
    std::exception_ptr failure;
    try { s.policy->close(); } catch (...) { failure = std::current_exception(); }
    try { s.environment->close(); }
    catch (...) {
        if (!failure) failure = std::current_exception();
        else s.note_cleanup("environment close also failed");
    }
    s.closed = true;
    if (failure) std::rethrow_exception(failure);
}
} // namespace bike::research
