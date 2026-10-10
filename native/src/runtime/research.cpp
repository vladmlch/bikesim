#include "research.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include "../stepper.hpp"
#include "../model_access.hpp"

namespace runtime {
namespace {
using namespace sample_wire;
class ResearchGuard {
public:
    explicit ResearchGuard(std::atomic<bool> &busy) : busy_(busy) {
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true, std::memory_order_acquire))
            throw std::runtime_error("native research operation already in progress");
    }
    ~ResearchGuard() { busy_.store(false, std::memory_order_release); }
    ResearchGuard(const ResearchGuard &) = delete;
    ResearchGuard &operator=(const ResearchGuard &) = delete;
    ResearchGuard(ResearchGuard &&) = delete;
    ResearchGuard &operator=(ResearchGuard &&) = delete;
private:
    std::atomic<bool> &busy_;
};
Wire opt(std::optional<double> v) { return v ? Wire(*v) : Wire(nullptr); }
Wire opt(const std::optional<std::string> &v) { return v ? Wire(*v) : Wire(nullptr); }
const WireObject &section(const WireObject &v, std::string_view key) { return object(required(v, key)); }
double field(const WireObject &v, std::string_view key) { return number(required(v, key)); }
void validate_brake(double value) {
    if (!std::isfinite(value) || value < 0. || value > 1.)
        throw std::invalid_argument("brake demand must lie in [0, 1]");
}
// Steps must stay below 2^53 so every committed index is exactly real-representable.
constexpr std::int64_t kMaxExactStep = static_cast<std::int64_t>(std::uint64_t{1} << 52U);
}
WireObject ResearchCommand::as_wire(bool requested) const {
    WireObject result{{"time_s", time_s}, {"step", step}, {"control", research_control_wire(control)}};
    if (requested) {
        result.emplace_back("front_brake_demand", front);
        result.emplace_back("rear_brake_demand", rear);
    }
    return result;
}
WireObject ResearchTransition::as_wire() const {
    return {{"observation", observation.as_wire()}, {"truth", truth ? Wire(truth->as_wire()) : Wire(nullptr)},
        {"contact_state", contact_state}, {"terminated", terminated}, {"truncated", truncated},
        {"reason", opt(reason)}, {"physics_steps", physics_steps}, {"numerically_valid", numerically_valid},
        {"demand_nm", opt(demand_nm)}, {"model_valid", model_valid}};
}
ResearchEpisode::ResearchEpisode(const ResearchConfig &c, NoiseTape tape, SensorState startup)
    : sensors(c.sensors, std::move(tape)), tracker(c.wheelie_persistence_s),
      recorder(c.record_decimation, ColumnLayout{.root_x_qpos = c.geometry.root_x_qpos,
          .root_x_dof = c.geometry.root_x_dof, .root_pitch_qpos = c.geometry.root_pitch_qpos}),
      next_sensor_step(c.sensor_steps), applied(c.startup_control), motor_applied(c.startup_control) {
    // Python setup already pushed and delivered row zero. Never noise it twice.
    if (startup.cursor != 1 || startup.samples_attempted != 1 || !startup.last_time || *startup.last_time != 0. ||
        !startup.delivery_time || *startup.delivery_time != 0. || !startup.startup || startup.startup->time_s != 0.)
        throw std::invalid_argument("research bootstrap requires the delivered startup row at t=0");
    sensors.import_state(std::move(startup));
    observation = sensors.read(0.);
    observations.push_back(observation);
    applied_records.push_back(ResearchCommand{.time_s = 0., .step = 0, .control = applied,
        .front = 0., .rear = 0.});
    demand_nm = c.programs.demand_at(0.);
    if (demand_nm) demand_integral_nms = 0.;
}
NativeResearchRuntime::NativeResearchRuntime(std::unique_ptr<NativeRideRuntime> runtime,
    ResearchConfig config, NoiseTape tape, SensorState startup)
    : runtime_(std::move(runtime)), config_(std::move(config)) {
    if (!runtime_ || runtime_->closed() || runtime_->committed_step_ != 0 || runtime_->committed_time_s_ != 0.)
        throw std::invalid_argument("native research requires a fresh owned physical runtime");
    const auto &c = config_;
    if (!std::isfinite(c.timestep_s) || c.timestep_s <= 0. || c.timestep_s != runtime_->config_.timestep_s ||
        c.control_steps < 1 || c.sensor_steps < 1 || c.max_steps < 1 || c.delay_steps < 0 ||
        c.max_steps > kMaxExactStep || c.sensor_steps > kMaxExactStep ||
        c.delay_steps > std::numeric_limits<std::int64_t>::max() - c.max_steps)
        throw std::invalid_argument("invalid integer research deadlines or physics timestep");
    if (!std::isfinite(c.track_length_m) || c.track_length_m <= 0. || !std::isfinite(c.start_position_m) ||
        !std::isfinite(c.maximum_energy_residual_ratio) || c.maximum_energy_residual_ratio <= 0.)
        throw std::invalid_argument("invalid research geometry/energy limits");
    if (c.record_decimation != runtime_->config_.record_decimation ||
        c.stop_on_model_violation != runtime_->config_.strict)
        throw std::invalid_argument("research recording/strictness must match its physical runtime");
    c.geometry.validate(static_cast<std::size_t>(runtime_->stepper_->model()->nq),
                        static_cast<std::size_t>(runtime_->stepper_->model()->nv));
    if (c.geometry.root_x_qpos != runtime_->accounting_config_.columns.root_x_qpos ||
        c.geometry.root_x_dof != runtime_->accounting_config_.columns.root_x_dof ||
        c.geometry.root_pitch_qpos != runtime_->accounting_config_.columns.root_pitch_qpos ||
        c.start_position_m != runtime_->committed_position_m_)
        throw std::invalid_argument("research truth projection disagrees with physical bootstrap");
    const auto vertices = runtime_->physical_->terrain_vertices();
    if (vertices.size() != c.geometry.x.size())
        throw std::invalid_argument("research terrain width differs from physical bootstrap");
    for (std::size_t i = 0; i < vertices.size(); ++i)
        if (std::get<0>(vertices[i]) != c.geometry.x[i] || std::get<1>(vertices[i]) != c.geometry.z[i])
            throw std::invalid_argument("research terrain differs from physical bootstrap");
    const auto *tire = runtime_->stepper_->tire();
    const auto *model = runtime_->stepper_->model();
    const int pitch_joint = mj_name2id(model, mjOBJ_JOINT, "root_pitch");
    const auto dofs = model_access::readonly_buffer(model->jnt_dofadr, model->njnt,
                                                    "research pitch joint dof addresses");
    if (!tire || tire->radii()[0] != c.geometry.front_radius || tire->radii()[1] != c.geometry.rear_radius ||
        pitch_joint < 0 || std::cmp_not_equal(dofs[static_cast<std::size_t>(pitch_joint)], c.geometry.root_pitch_dof))
        throw std::invalid_argument("research radii/pitch address differ from physical bootstrap");
    c.programs.validate();
    control_validate_for(c.startup_control, runtime_->physical_->rider_present());
    const auto required_rows = static_cast<std::size_t>(1 + c.max_steps / c.sensor_steps);
    if (tape.noise.size() != required_rows)
        throw std::invalid_argument("research tape must contain 1 + floor(max_steps / sensor_steps) rows");
    episode_ = std::make_unique<ResearchEpisode>(c, std::move(tape), std::move(startup));
}
NativeResearchRuntime::~NativeResearchRuntime() = default;
void NativeResearchRuntime::require_open() const {
    if (closed_.load(std::memory_order_acquire)) throw std::runtime_error("NativeResearchRuntime is closed");
}
bool NativeResearchRuntime::model_valid() const {
    return boolean(required(runtime_->accounting_->state().model_status.as_wire(), "model_valid"));
}
void NativeResearchRuntime::begin_control(const RideControl &control, double front, double rear) {
    const ResearchGuard guard(busy_);
    require_open();
    auto &e = *episode_;
    if (e.window || e.delivery_pending) throw std::runtime_error("an external research window is already active");
    if (e.done()) throw std::runtime_error("episode has ended; reset before stepping again");
    validate_brake(front);
    validate_brake(rear);
    config_.programs.validate_control(control);
    control_validate_for(control, runtime_->physical_->rider_present());
    const auto start = runtime_->committed_step_;
    if (start >= config_.max_steps) throw std::runtime_error("research duration already reached");
    const auto count = std::min(config_.control_steps, config_.max_steps - start);
    const ResearchWindow window{.control = control, .front = front, .rear = rear,
        .start_step = start, .target_step = start + count, .seen_demand = e.demand_nm};
    const ResearchCommand request{.time_s = runtime_->committed_time_s_, .step = start,
        .control = control, .front = front, .rear = rear};
    e.requested.reserve(e.requested.size() + 1);
    e.commands.emplace_back(start + config_.delay_steps, control);
    e.requested.push_back(request);
    e.window = window;
}
void NativeResearchRuntime::consume_sample(const SamplePtr &sample, bool stop_at_outcome) {
    auto &e = *episode_;
    const auto &s = *sample;
    const double ratio = s.time_s / config_.timestep_s;
    if (!std::isfinite(ratio) || ratio < 0. || ratio > static_cast<double>(config_.max_steps))
        throw std::runtime_error("invalid sensor acquisition time");
    const auto source_step = static_cast<std::int64_t>(std::nearbyint(ratio));
    if (source_step >= e.next_sensor_step) {
        if (source_step != e.next_sensor_step)
            throw std::runtime_error("sensor acquisition skipped a physical sample");
        e.sensors.push(raw_sensor_observation(s));
        e.next_sensor_step += config_.sensor_steps;
    }
    e.truth = research_truth(s, config_.geometry);
    const auto &suspension = section(s.channels, "suspension");
    e.max_shock_stroke_m = std::max(e.max_shock_stroke_m, std::abs(field(suspension, "shock_stroke_m")));
    e.max_fork_travel_m = std::max(e.max_fork_travel_m, std::abs(field(suspension, "fork_travel_m")));
    const auto &drive = section(s.channels, "drive");
    const auto *motor = find(drive, "motor_torque_nm");
    const double delivered = motor ? number(*motor) : 0.;
    if (e.demand_integral_nms) {
        const auto demand = config_.programs.demand_at(s.time_s);
        // Integral tracking is created only when a demand program exists.
        if (!demand) throw std::logic_error("demand integration requires a demand program");
        *e.demand_integral_nms += *demand * s.dt_s();
    }
    const auto &requested = required(section(s.channels, "control"), "motor_torque_nm");
    const auto applied = is_none(requested) ? std::nullopt : std::optional(number(requested));
    if (applied) e.torque_requested_nms += *applied * s.dt_s();
    e.torque_delivered_nms += delivered * s.dt_s();
    e.tracker.update(*e.truth, s.dt_s(), delivered, applied);
    e.recorder.append(sample);
    const auto quality = research_quality(section(s.channels, "energy"), config_.maximum_energy_residual_ratio);
    e.max_energy_residual_ratio = std::max(e.max_energy_residual_ratio, quality.residual_ratio);
    if (!quality.acceptable) {
        e.numerically_valid = false;
        e.truncated = true;
        e.reason = "numerical_quality";
        if (stop_at_outcome) return;
    }
    if (config_.stop_on_model_violation && !boolean(required(section(s.channels, "model_status"), "model_valid"))) {
        e.truncated = true;
        e.reason = "model_violation";
        if (stop_at_outcome) return;
    }
    const auto &crash = runtime_->committed_crash_;
    if (crash && crash->time_s <= s.end_time_s) {
        e.terminated = true;
        e.reason = crash->cause == "pitch_over"
            ? (crash->pitch_rad < 0. ? "crash:loop_out" : "crash:endo") : "crash:" + crash->cause;
        if (stop_at_outcome) return;
    }
    if (e.truth->position_m >= config_.track_length_m) {
        e.terminated = true;
        e.reason = "finish";
    }
}
void NativeResearchRuntime::consume_samples(bool stop_at_outcome) {
    auto &e = *episode_;
    try {
        for (const auto &sample : runtime_->accounting_->pending_samples()) {
            if (sample->interval_id <= e.sample_cursor) continue;
            // Claim before side effects, as the Python invalid-run contract does.
            e.sample_cursor = sample->interval_id;
            consume_sample(sample, stop_at_outcome);
            if (stop_at_outcome && e.done()) break;
        }
    } catch (...) {
        runtime_->accounting_->acknowledge_through(e.sample_cursor);
        throw;
    }
    runtime_->accounting_->acknowledge_through(e.sample_cursor);
}
void NativeResearchRuntime::finish_window() {
    auto &e = *episode_;
    if (!e.window) throw std::logic_error("no research window to finish");
    if (!e.done() && runtime_->committed_step_ >= config_.max_steps) {
        e.truncated = true;
        e.reason = "duration";
    }
    e.observations.reserve(e.observations.size() + 1);
    e.transitions.reserve(e.transitions.size() + 1);
    const auto demand = config_.programs.demand_at(runtime_->committed_time_s_);
    const auto observation = e.sensors.read(runtime_->committed_time_s_);
    ResearchTransition result{.observation = observation, .truth = e.truth,
        .contact_state = e.tracker.state(), .terminated = e.terminated, .truncated = e.truncated,
        .numerically_valid = e.numerically_valid, .model_valid = model_valid(), .reason = e.reason,
        .physics_steps = runtime_->committed_step_ - e.window->start_step,
        .demand_nm = e.window->seen_demand};
    e.transitions.push_back(std::move(result));
    e.observations.push_back(observation);
    e.observation = observation;
    e.demand_nm = demand;
    e.delivery_pending = true;
}
void NativeResearchRuntime::handle_failure(const std::exception &failure) {
    auto &e = *episode_;
    e.terminated = true;
    const bool reference = dynamic_cast<const InvalidReferenceRun *>(&failure) != nullptr;
    const std::string reason = reference ? "invalid_controller" : "simulation_error";
    e.reason = reason;
    e.error = std::string(reference ? "InvalidReferenceRun: " : "RuntimeError: ") + failure.what();
    // Cleanup may fail independently; retain the original exception and all
    // successfully published evidence rather than manufacturing a transition.
    try { if (runtime_->accounting_->has_raws()) runtime_->flush_accounting(); }
    catch (const std::exception &tail) { e.cleanup_notes.push_back(std::string("trailing interval check: ") + tail.what()); }
    try { consume_samples(false); }
    catch (const std::exception &tail) { e.cleanup_notes.push_back(std::string("trailing sample consumption: ") + tail.what()); }
    e.reason = reason;
    e.window.reset();
    e.delivery_pending = false;
}
std::optional<ResearchTransition> NativeResearchRuntime::advance_control(
    std::optional<double> budget, std::optional<std::int64_t> target_step) {
    const ResearchGuard guard(busy_);
    require_open();
    if (budget && (!std::isfinite(*budget) || *budget < 0.))
        throw std::invalid_argument("wall_budget_s must be finite and nonnegative");
    if (target_step && *target_step < runtime_->committed_step_)
        throw std::invalid_argument("target_step is behind the committed step");
    auto &e = *episode_;
    if (e.delivery_pending) return e.transitions.back();
    if (!e.window) throw std::runtime_error("begin_control must precede advance_control");
    const auto start = std::chrono::steady_clock::now();
    try {
        runtime_->stepper_->refresh_time_callback_policy();
        while (runtime_->committed_step_ < e.window->target_step && !e.done()) {
            if (target_step && runtime_->committed_step_ >= *target_step)
                return std::nullopt; // Pacing yield; the external command stays latched.
            if (budget && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= *budget)
                return std::nullopt; // No flush, delivery, request or transition on a yield.
            while (!e.commands.empty() && e.commands.front().first <= runtime_->committed_step_) {
                e.motor_applied = e.commands.front().second;
                e.commands.pop_front();
            }
            const auto rider = config_.programs.apply(e.window->control, runtime_->committed_time_s_);
            RideControl effective = e.motor_applied;
            effective.human_torque_nm = rider.human_torque_nm;
            effective.crank_target_rate_rad_s = rider.crank_target_rate_rad_s;
            effective.posture = rider.posture;
            effective.rider_enabled = rider.rider_enabled;
            if (!controls_equal(effective, e.applied)) {
                e.applied_records.push_back(ResearchCommand{.time_s = runtime_->committed_time_s_,
                    .step = runtime_->committed_step_, .control = effective, .front = 0., .rear = 0.});
                e.applied = effective;
            }
            runtime_->advance_interval(e.applied, e.window->front, e.window->rear);
            consume_samples();
        }
        if (runtime_->accounting_->has_raws()) {
            runtime_->flush_accounting();
            consume_samples();
        }
        finish_window();
    } catch (const std::exception &failure) {
        // Best-effort cleanup must not mask even an allocation exception.
        try { handle_failure(failure); }
        catch (...) {
            e.terminated = true;
            e.window.reset();
            e.delivery_pending = false;
        }
        throw;
    }
    return e.transitions.back();
}
void NativeResearchRuntime::acknowledge_control() {
    const ResearchGuard guard(busy_);
    require_open();
    auto &e = *episode_;
    if (!e.delivery_pending) throw std::runtime_error("no completed research transition to acknowledge");
    e.delivery_pending = false;
    e.window.reset();
    if (e.stop_requested && !e.done()) { e.truncated = true; e.reason = "operator_stop"; }
    e.stop_requested = false;
}
RuntimeSnapshot NativeResearchRuntime::snapshot() const {
    const ResearchGuard guard(busy_);
    require_open();
    return runtime_->snapshot();
}
WireObject NativeResearchRuntime::status() const {
    const ResearchGuard guard(busy_);
    require_open();
    const auto &e = *episode_;
    const auto &balance = runtime_->committed_balance_;
    const auto &crash = runtime_->committed_crash_;
    WireObject result{{"step", runtime_->committed_step_}, {"time_s", runtime_->committed_time_s_},
        {"position_m", runtime_->committed_position_m_}, {"battery_energy_j", runtime_->committed_battery_energy_j_},
        {"observation", e.observation.as_wire()}, {"demand_nm", opt(e.demand_nm)},
        {"truth", e.truth ? Wire(e.truth->as_wire()) : Wire(nullptr)}, {"contact_state", e.tracker.state()},
        {"terminated", e.terminated}, {"truncated", e.truncated}, {"reason", opt(e.reason)}, {"error", opt(e.error)},
        {"numerically_valid", e.numerically_valid}, {"model_valid", model_valid()},
        {"max_energy_residual_ratio", e.max_energy_residual_ratio}, {"torque_delivered_nms", e.torque_delivered_nms},
        {"torque_requested_nms", e.torque_requested_nms}, {"demand_integral_nms", opt(e.demand_integral_nms)},
        {"max_shock_stroke_m", e.max_shock_stroke_m}, {"max_fork_travel_m", e.max_fork_travel_m},
        {"metrics", e.tracker.metrics()}, {"accounting", runtime_->accounting_->state_wire()},
        {"active", e.window.has_value()}, {"stop_requested", e.stop_requested}, {"delivery_pending", e.delivery_pending},
        {"samples_attempted", static_cast<std::int64_t>(e.sensors.state().samples_attempted)},
        {"samples_dropped", static_cast<std::int64_t>(e.sensors.state().samples_dropped)},
        {"noise_cursor", static_cast<std::int64_t>(e.sensors.state().cursor)},
        {"recorded_rows", static_cast<std::int64_t>(e.recorder.rows())},
        {"cleanup_notes", strings(e.cleanup_notes)}};
    result.emplace_back("balance_event", balance ? Wire(WireObject{{"time_s", balance->time_s},
        {"position_m", balance->position_m}, {"speed_mps", balance->speed_mps}}) : Wire(nullptr));
    result.emplace_back("crash", crash ? Wire(WireObject{{"time_s", crash->time_s}, {"position_m", crash->position_m},
        {"pitch_rad", crash->pitch_rad}, {"cause", crash->cause}}) : Wire(nullptr));
    return result;
}
WireArray NativeResearchRuntime::commands_requested() const {
    const ResearchGuard guard(busy_); require_open();
    WireArray out; out.reserve(episode_->requested.size());
    for (const auto &c : episode_->requested) out.emplace_back(c.as_wire(true));
    return out;
}
WireArray NativeResearchRuntime::commands_applied() const {
    const ResearchGuard guard(busy_); require_open();
    WireArray out; out.reserve(episode_->applied_records.size());
    for (const auto &c : episode_->applied_records) out.emplace_back(c.as_wire(false));
    return out;
}
WireArray NativeResearchRuntime::observations() const {
    const ResearchGuard guard(busy_); require_open();
    WireArray out; out.reserve(episode_->observations.size());
    for (const auto &v : episode_->observations) out.emplace_back(v.as_wire());
    return out;
}
WireArray NativeResearchRuntime::transitions() const {
    const ResearchGuard guard(busy_); require_open();
    WireArray out; out.reserve(episode_->transitions.size());
    for (const auto &v : episode_->transitions) out.emplace_back(v.as_wire());
    return out;
}
SampleColumns NativeResearchRuntime::recorded_columns() const {
    const ResearchGuard guard(busy_); require_open(); return episode_->recorder.columns();
}
WireArray NativeResearchRuntime::recorded_intervals() const {
    const ResearchGuard guard(busy_); require_open(); return episode_->recorder.intervals();
}
void NativeResearchRuntime::reset(NoiseTape tape, SensorState startup) {
    const ResearchGuard guard(busy_); require_open();
    if (episode_->window || episode_->delivery_pending)
        throw std::runtime_error("finish the active external interval before reset");
    if (tape.noise.size() != static_cast<std::size_t>(1 + config_.max_steps / config_.sensor_steps))
        throw std::invalid_argument("reset noise tape has the wrong number of rows");
    auto next = std::make_unique<ResearchEpisode>(config_, std::move(tape), std::move(startup));
    // Physical reset owns its generation and poisoned-context recovery.
    runtime_->reset();
    episode_ = std::move(next);
}
void NativeResearchRuntime::stop() {
    const ResearchGuard guard(busy_); require_open();
    auto &e = *episode_;
    if (e.done()) return;
    if (e.window) e.stop_requested = true;
    else { e.truncated = true; e.reason = "operator_stop"; }
}
void NativeResearchRuntime::fail_policy(std::string message) {
    const ResearchGuard guard(busy_); require_open();
    if (episode_->window) throw std::runtime_error("cannot fail a policy inside an active native interval");
    if (!episode_->done()) {
        episode_->terminated = true;
        episode_->reason = "policy_error";
        episode_->error = std::move(message);
    }
}
void NativeResearchRuntime::set_error_text(std::string message) {
    const ResearchGuard guard(busy_); require_open();
    if (!episode_->terminated || episode_->delivery_pending)
        throw std::runtime_error("error text requires a failed research episode");
    episode_->error = std::move(message);
}
void NativeResearchRuntime::close() {
    const ResearchGuard guard(busy_);
    if (closed_.load(std::memory_order_acquire)) return;
    runtime_->close();
    closed_.store(true, std::memory_order_release);
}
void NativeResearchRuntime::test_fail_at_step(std::int64_t step) {
    const ResearchGuard guard(busy_); require_open();
    if (episode_->window) throw std::runtime_error("arm failure before beginning a research interval");
    runtime_->test_fail_at_step(step);
}
void NativeResearchRuntime::emit_warning() {
    const ResearchGuard guard(busy_); require_open(); runtime_->emit_reference_warning();
}
} // namespace runtime
