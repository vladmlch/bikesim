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
bool keeps_history(ResearchCaptureMode mode) noexcept {
    return mode == ResearchCaptureMode::full || mode == ResearchCaptureMode::legacy;
}
template<class T> void reserve_next(std::vector<T> &values) {
    if (values.size() == values.max_size()) throw std::length_error("research history row limit reached");
    const auto required = values.size() + 1;
    if (values.capacity() >= required) return;
    const auto grown = values.capacity() > values.max_size() / 2 ? required : values.capacity() * 2;
    values.reserve(std::max(required, grown));
}
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
    // Setup already pushed and delivered row zero. Never noise it twice.
    if (startup.cursor != 1 || startup.samples_attempted != 1 || !startup.last_time || *startup.last_time != 0. ||
        !startup.delivery_time || *startup.delivery_time != 0. || !startup.startup || startup.startup->time_s != 0.)
        throw std::invalid_argument("research bootstrap requires the delivered startup row at t=0");
    sensors.import_state(std::move(startup));
    observation = sensors.read(0.);
    if (keeps_history(c.capture_mode)) observations.push_back(observation);
    if (c.capture_mode != ResearchCaptureMode::off)
        applied_records.push_back(ResearchCommand{.time_s = 0., .step = 0, .control = applied,
            .front = 0., .rear = 0.});
    demand_nm = c.programs.demand_at(0.);
    if (demand_nm) demand_integral_nms = 0.;
}
NativeResearchRuntime::NativeResearchRuntime(std::unique_ptr<NativeRideRuntime> runtime,
    ResearchConfig config, NoiseTape tape, SensorState startup)
    : runtime_(std::move(runtime)), config_(std::move(config)), reset_tape_(tape), reset_startup_(startup) {
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
    if (c.record_decimation != runtime_->config_.measurement_decimation ||
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
    const auto *model = runtime_->stepper_->model();
    const int pitch_joint = mj_name2id(model, mjOBJ_JOINT, "root_pitch");
    const auto dofs = model_access::readonly_buffer(model->jnt_dofadr, model->njnt,
                                                    "research pitch joint dof addresses");
    // Backend-agnostic presence + radii check: the writer view forwards
    // radii() for whichever tyre backend the config selected.
    const bool tire_mismatch = !runtime_->stepper_->has_tire() ||
        runtime_->stepper_->tire_writer().radii()[0] != c.geometry.front_radius ||
        runtime_->stepper_->tire_writer().radii()[1] != c.geometry.rear_radius;
    if (tire_mismatch || pitch_joint < 0 ||
        std::cmp_not_equal(dofs[static_cast<std::size_t>(pitch_joint)], c.geometry.root_pitch_dof))
        throw std::invalid_argument("research radii/pitch address differ from physical bootstrap");
    c.programs.validate();
    control_validate_for(c.startup_control, runtime_->physical_->rider_present(), runtime_->physical_->drive_mode());
    if (c.seed_decimal.empty() || !std::ranges::all_of(c.seed_decimal,
            [](char ch) { return ch >= '0' && ch <= '9'; }))
        throw std::invalid_argument("research seed must be a nonnegative decimal integer");
    for (const double scale : c.noise_std)
        if (!std::isfinite(scale) || scale < 0.)
            throw std::invalid_argument("sensor noise scales must be finite and nonnegative");
    if (c.raw_initial) {
        c.raw_initial->validate();
        if (c.raw_initial->time_s != 0. || c.raw_initial->source_time_s != 0.)
            throw std::invalid_argument("research raw startup observation must belong to time zero");
    }
    const auto required_rows = static_cast<std::size_t>(1 + c.max_steps / c.sensor_steps);
    if (tape.noise.size() != required_rows)
        throw std::invalid_argument("research tape must contain 1 + floor(max_steps / sensor_steps) rows");
    initial_integration_state_ = runtime_->snapshot().integration_state;
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
    control_validate_for(control, runtime_->physical_->rider_present(), runtime_->physical_->drive_mode());
    const auto start = runtime_->committed_step_;
    if (start >= config_.max_steps) throw std::runtime_error("research duration already reached");
    const auto count = std::min(config_.control_steps, config_.max_steps - start);
    const ResearchWindow window{.control = control, .front = front, .rear = rear,
        .start_step = start, .target_step = start + count, .seen_demand = e.demand_nm};
    const ResearchCommand request{.time_s = runtime_->committed_time_s_, .step = start,
        .control = control, .front = front, .rear = rear};
    if (keeps_history(config_.capture_mode)) reserve_next(e.requested);
    // RideControl contains value-only fields. Queue allocation precedes the
    // publication of the accepted request and the active window.
    e.commands.emplace_back(start + config_.delay_steps, control);
    if (keeps_history(config_.capture_mode)) e.requested.push_back(request);
    e.last_requested = request;
    ++e.requested_count;
    e.window = window;
}
void NativeResearchRuntime::consume_sample(const SamplePtr &sample, bool stop_at_outcome) {
    auto &e = *episode_;
    const auto &s = *sample;
    if(find(s.channels,"spatial")) {
        if(!e.spatial_metrics)e.spatial_metrics=SpatialEpisodeMetrics{
            .requested_duration_s=static_cast<double>(config_.max_steps)*config_.timestep_s,
            .start_position_m=config_.start_position_m};
        const auto* recorded=find(s.channels,"spatial_outcome");
        if(!recorded)throw std::invalid_argument("research.spatial: sample lacks its accepted outcome record");
        const auto& record=object(*recorded);
        bike::spatial::Outcome outcome;
        outcome.terminal=boolean(required(record,"terminal"));outcome.valid=boolean(required(record,"valid"));
        outcome.tick=integer(required(record,"tick"));outcome.time_s=number(required(record,"time_s"));
        const auto label=string(required(record,"reason"));bool recognized=false;
        for(auto reason:{bike::spatial::SpatialReason::none,bike::spatial::SpatialReason::capsize,
            bike::spatial::SpatialReason::off_corridor,bike::spatial::SpatialReason::balance_lost,
            bike::spatial::SpatialReason::terrain_invalid,bike::spatial::SpatialReason::terrain_outside,
            bike::spatial::SpatialReason::numerical_failure,bike::spatial::SpatialReason::requested_completion})
            if(label==bike::spatial::to_string(reason)){outcome.reason=reason;recognized=true;break;}
        if(!recognized)throw std::invalid_argument("research.spatial: unknown accepted outcome reason");
        update_spatial_metrics(*e.spatial_metrics,s.interval_id,s.end_time_s,s.dt_s(),
            number(required(record,"accepted_position_m")),outcome);
        if(outcome.terminal && outcome.time_s<=s.end_time_s) {
            e.numerically_valid=e.numerically_valid && outcome.valid;
            e.terminated=outcome.valid;e.truncated=!outcome.valid;
            e.reason=label;
            // Even cleanup consumption cannot replace a spatial latch with a
            // later planar monitor, model-quality label or finish poll.
            return;
        }
    }
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
    if (config_.capture_mode == ResearchCaptureMode::legacy) e.recorder.append(sample);
    const auto quality = research_quality(section(s.channels, "energy"), config_.maximum_energy_residual_ratio);
    e.max_energy_residual_ratio = std::max(e.max_energy_residual_ratio, quality.residual_ratio);
    if (!quality.acceptable) {
        e.numerically_valid = false;
        if(e.spatial_metrics)e.spatial_metrics->valid=false;
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
    if (keeps_history(config_.capture_mode)) {
        reserve_next(e.observations);
        reserve_next(e.transitions);
    }
    const auto demand = config_.programs.demand_at(runtime_->committed_time_s_);
    const auto observation = e.sensors.read(runtime_->committed_time_s_);
    ResearchTransition result{.observation = observation, .truth = e.truth,
        .contact_state = e.tracker.state(), .terminated = e.terminated, .truncated = e.truncated,
        .numerically_valid = e.numerically_valid, .model_valid = model_valid(), .reason = e.reason,
        .physics_steps = runtime_->committed_step_ - e.window->start_step,
        .demand_nm = e.window->seen_demand};
    // Stage the deliverable before publishing the completed control prefix.
    // Python boxing may fail and retry without advancing any of the clocks.
    auto pending = result;
    if (keeps_history(config_.capture_mode)) {
        e.transitions.push_back(std::move(result));
        e.observations.push_back(observation);
    }
    e.pending_transition = std::move(pending);
    ++e.completed_count;
    e.observation = observation;
    e.demand_nm = demand;
    e.delivery_pending = true;
}
void NativeResearchRuntime::handle_failure(const std::exception &failure) {
    auto &e = *episode_;
    e.terminated = true;
    const bool reference = dynamic_cast<const InvalidReferenceRun *>(&failure) != nullptr;
    std::string reason = reference ? "invalid_controller" : "simulation_error";
    std::optional<bike::spatial::SpatialReason> spatial_failure;
    if(runtime_->config_.spatial) {
        if(!e.spatial_metrics)e.spatial_metrics=SpatialEpisodeMetrics{
            .requested_duration_s=static_cast<double>(config_.max_steps)*config_.timestep_s,
            .start_position_m=config_.start_position_m};
        e.numerically_valid=false;e.terminated=false;e.truncated=true;
        const auto& state=runtime_->physical_->spatial_state();
        spatial_failure=state && state->outcome.latched.terminal?state->outcome.latched.reason:
            bike::spatial::SpatialReason::numerical_failure;
        // Keep the failure local until all already accepted pending samples
        // have extended the valid prefix. A premature terminal latch would
        // make their metrics updates observational no-ops.
    }
    e.reason = reason;
    e.error = std::string(reference ? "InvalidReferenceRun: " : "RuntimeError: ") + failure.what();
    // Cleanup may fail independently; retain the original exception and all
    // successfully published evidence rather than manufacturing a transition.
    try { if (runtime_->accounting_->has_raws()) runtime_->flush_accounting(); }
    catch (const std::exception &tail) { e.cleanup_notes.push_back(std::string("trailing interval check: ") + tail.what()); }
    try { consume_samples(false); }
    catch (const std::exception &tail) { e.cleanup_notes.push_back(std::string("trailing sample consumption: ") + tail.what()); }
    if(e.spatial_metrics) {
        if(e.spatial_metrics->terminal_reason==bike::spatial::SpatialReason::none && spatial_failure)
            e.spatial_metrics->terminal_reason=*spatial_failure;
        reason=std::string(bike::spatial::to_string(e.spatial_metrics->terminal_reason));
        e.spatial_metrics->valid=false;e.numerically_valid=false;e.terminated=false;e.truncated=true;
    }
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
    if (e.delivery_pending) return e.pending_transition;
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
                if (config_.capture_mode != ResearchCaptureMode::off)
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
    return e.pending_transition;
}
void NativeResearchRuntime::acknowledge_control() {
    const ResearchGuard guard(busy_);
    require_open();
    auto &e = *episode_;
    if (!e.delivery_pending) throw std::runtime_error("no completed research transition to acknowledge");
    e.pending_transition.reset();
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
RenderFrame NativeResearchRuntime::render_frame() const {
    const ResearchGuard guard(busy_);
    require_open();
    auto frame = runtime_->render_frame();
    if (episode_->reason) frame.state.outcome = episode_->reason;
    return frame;
}
ModelImage NativeResearchRuntime::model_image() const {
    const ResearchGuard guard(busy_);
    require_open();
    return runtime_->model_image();
}
WireObject NativeResearchRuntime::status() const {
    const ResearchGuard guard(busy_);
    require_open();
    return status_unlocked();
}
WireObject NativeResearchRuntime::status_unlocked() const {
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
        {"recorded_rows", static_cast<std::int64_t>(config_.capture_mode == ResearchCaptureMode::full
            ? runtime_->accounting_->recorded_samples().size() : e.recorder.rows())},
        {"requested_count", e.requested_count}, {"completed_count", e.completed_count},
        {"applied_base", e.applied_base},
        {"cleanup_notes", strings(e.cleanup_notes)}};
    result.emplace_back("balance_event", balance ? Wire(WireObject{{"time_s", balance->time_s},
        {"position_m", balance->position_m}, {"speed_mps", balance->speed_mps}}) : Wire(nullptr));
    result.emplace_back("crash", crash ? Wire(WireObject{{"time_s", crash->time_s}, {"position_m", crash->position_m},
        {"pitch_rad", crash->pitch_rad}, {"cause", crash->cause}}) : Wire(nullptr));
    if(e.spatial_metrics)result.emplace_back("spatial",Wire(e.spatial_metrics->as_wire()));
    return result;
}
ResearchConfig NativeResearchRuntime::configuration() const {
    const ResearchGuard guard(busy_); require_open();
    return config_;
}
OwnedResearchEpisode NativeResearchRuntime::evidence_unlocked(OwnedNativeEpisode physical) const {
    const auto& e = *episode_;
    return OwnedResearchEpisode{.physical = std::move(physical),
        .requested = e.requested, .applied = e.applied_records,
        .observations = e.observations, .transitions = e.transitions,
        .evaluation = status_unlocked(), .final_snapshot = runtime_->snapshot(),
        .configuration = config_, .initial_integration_state = initial_integration_state_,
        .run_metadata = bike::io::Value{bike::io::Value::Object{}}};
}
OwnedResearchEpisode NativeResearchRuntime::evidence_snapshot() const {
    const ResearchGuard guard(busy_); require_open();
    OwnedNativeEpisode physical;
    physical.generation = runtime_->committed_generation_;
    physical.columns = runtime_->accounting_config_.columns;
    if (config_.capture_mode == ResearchCaptureMode::full)
        physical = runtime_->accounting_->seal_episode();
    else if (config_.capture_mode == ResearchCaptureMode::legacy)
        physical = episode_->recorder.seal();
    return evidence_unlocked(std::move(physical));
}
std::size_t OwnedResearchEpisode::row_count(std::string_view stream) const {
    if (stream == "commands_requested") return requested.size();
    if (stream == "commands_applied") return applied.size();
    if (stream == "observations") return observations.size();
    if (stream == "transitions") return transitions.size();
    throw std::invalid_argument("unknown research stream");
}
WireArray OwnedResearchEpisode::batch(std::string_view stream, std::int64_t start,
                                      std::int64_t limit) const {
    const auto count = row_count(stream);
    if (start < 0 || limit <= 0 || std::cmp_greater(start, count))
        throw std::invalid_argument("invalid research batch range");
    const auto first = static_cast<std::size_t>(start);
    const auto size = std::min(count - first, static_cast<std::size_t>(limit));
    WireArray rows;
    rows.reserve(size);
    for (std::size_t index = first; index < first + size; ++index) {
        if (stream == "commands_requested") rows.emplace_back(requested[index].as_wire(true));
        else if (stream == "commands_applied") rows.emplace_back(applied[index].as_wire(false));
        else if (stream == "observations") rows.emplace_back(observations[index].as_wire());
        else rows.emplace_back(transitions[index].as_wire());
    }
    return rows;
}
OwnedResearchEpisode NativeResearchRuntime::seal_episode() {
    const ResearchGuard guard(busy_); require_open();
    if (config_.capture_mode != ResearchCaptureMode::full)
        throw std::runtime_error("sealing research evidence requires full capture");
    // Sealing never integrates or completes an external window. Frontends
    // drain a requested stop at the existing control boundary first.
    if (runtime_->accounting_->has_raws()) runtime_->flush_accounting();
    return evidence_unlocked(runtime_->accounting_->seal_episode());
}
OwnedResearchEpisode NativeResearchRuntime::legacy_snapshot() const {
    const ResearchGuard guard(busy_); require_open();
    if (!keeps_history(config_.capture_mode))
        throw std::runtime_error("legacy export requires legacy or full capture");
    const auto &e = *episode_;
    if (e.window || e.delivery_pending)
        throw std::runtime_error("finish the external control interval before exporting");
    return evidence_unlocked(config_.capture_mode == ResearchCaptureMode::full
        ? runtime_->accounting_->seal_episode() : e.recorder.seal());
}
Wire NativeResearchRuntime::last_requested() const {
    const ResearchGuard guard(busy_); require_open();
    return episode_->last_requested ? Wire(episode_->last_requested->as_wire(true)) : Wire(nullptr);
}
WireObject NativeResearchRuntime::prepare_applied_since(std::int64_t cursor) const {
    const ResearchGuard guard(busy_); require_open();
    const auto &e = *episode_;
    const auto end = e.applied_base + static_cast<std::int64_t>(e.applied_records.size());
    if (cursor < e.applied_base || cursor > end)
        throw std::invalid_argument("applied cursor is stale or beyond the committed prefix");
    WireArray rows;
    rows.reserve(static_cast<std::size_t>(end - cursor));
    for (auto index = cursor; index < end; ++index)
        rows.emplace_back(e.applied_records[static_cast<std::size_t>(index - e.applied_base)].as_wire(false));
    return {{"rows", std::move(rows)}, {"next_cursor", end}};
}
void NativeResearchRuntime::acknowledge_applied(std::int64_t cursor) {
    const ResearchGuard guard(busy_); require_open();
    auto &e = *episode_;
    const auto end = e.applied_base + static_cast<std::int64_t>(e.applied_records.size());
    if (cursor < e.applied_base || cursor > end)
        throw std::invalid_argument("applied acknowledgement is stale or beyond the committed prefix");
    if (config_.capture_mode == ResearchCaptureMode::verify) {
        // cursor and applied_base are std::int64_t — identical to the
        // container's difference_type on the supported LP64 toolchains, so
        // an explicit cast is redundant (GCC -Wuseless-cast fires on it).
        const auto count = cursor - e.applied_base;
        e.applied_records.erase(e.applied_records.begin(), e.applied_records.begin() + count);
        e.applied_base = cursor;
    }
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
    const ResearchGuard guard(busy_); require_open();
    if (config_.capture_mode == ResearchCaptureMode::full)
        return make_columns(runtime_->accounting_->recorded_samples(), runtime_->accounting_config_.columns);
    return episode_->recorder.columns();
}
WireArray NativeResearchRuntime::recorded_intervals() const {
    const ResearchGuard guard(busy_); require_open();
    if (config_.capture_mode == ResearchCaptureMode::full) {
        WireArray rows;
        for (const auto &sample : runtime_->accounting_->recorded_samples()) rows.emplace_back(sample->as_wire());
        return rows;
    }
    return episode_->recorder.intervals();
}
void NativeResearchRuntime::reset(NoiseTape tape, SensorState startup,
                                  std::optional<std::string> seed_decimal,
                                  std::optional<bike::io::Value> metadata, ResearchAssets assets) {
    const ResearchGuard guard(busy_); require_open();
    reset_unlocked(std::move(tape), std::move(startup), std::move(seed_decimal), std::move(metadata), std::move(assets));
}
void NativeResearchRuntime::reset_unlocked(NoiseTape tape, SensorState startup,
    std::optional<std::string> seed_decimal, std::optional<bike::io::Value> metadata, ResearchAssets assets) {
    if (episode_->window || episode_->delivery_pending)
        throw std::runtime_error("finish the active external interval before reset");
    if (tape.noise.size() != static_cast<std::size_t>(1 + config_.max_steps / config_.sensor_steps))
        throw std::invalid_argument("reset noise tape has the wrong number of rows");
    if (seed_decimal && (seed_decimal->empty() || !std::ranges::all_of(*seed_decimal,
            [](char c) { return c >= '0' && c <= '9'; })))
        throw std::invalid_argument("reset seed must be a nonnegative decimal integer");
    auto next_tape = tape;
    auto next_startup = startup;
    auto next_config = config_;
    if (seed_decimal) next_config.seed_decimal = std::move(*seed_decimal);
    if (metadata) next_config.metadata = std::move(*metadata);
    if (assets) next_config.assets = std::move(assets);
    auto next = std::make_unique<ResearchEpisode>(next_config, std::move(tape), std::move(startup));
    // Physical reset owns its generation and poisoned-context recovery.
    runtime_->reset();
    episode_ = std::move(next);
    reset_tape_ = std::move(next_tape);
    reset_startup_ = std::move(next_startup);
    config_ = std::move(next_config);
}
void NativeResearchRuntime::reset() {
    const ResearchGuard guard(busy_); require_open();
    reset_unlocked(reset_tape_, reset_startup_, std::nullopt, std::nullopt, {});
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
std::optional<std::string> NativeResearchRuntime::take_pending_warning() {
    // The pending non-strict rejection is pure runtime state; the
    // RuntimeWarning emission stays the adapter's job at the held-GIL
    // boundary — identical delegation to NativeRideRuntime.
    const ResearchGuard guard(busy_);
    require_open();
    return runtime_->take_pending_warning();
}
} // namespace runtime
