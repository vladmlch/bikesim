#include "accounting.hpp"
#include "async_close.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace runtime {
namespace {
using namespace sample_wire;

void finite(double value, std::string_view path, bool nonnegative = false) {
    if (!std::isfinite(value) || (nonnegative && value < 0.))
        throw std::invalid_argument(std::string(path) + ": invalid finite value");
}

std::int64_t next_interval(const WorkHistory &history) {
    if (!history.last_id.has_value()) return 0;
    if (*history.last_id == std::numeric_limits<std::int64_t>::max())
        throw std::overflow_error("accounting interval ID overflow");
    return *history.last_id + 1;
}

WireObject named_reals(const rider::NamedEntries<double> &values) {
    WireObject result;
    for (const auto &[name, value] : values) result.emplace_back(name, Wire(value));
    return result;
}

WireObject attachment_wire(const AttachmentSamples &samples) {
    WireObject result;
    for (const auto &[name, s] : samples)
        result.emplace_back(name, Wire(WireObject{
            {"kind", Wire(s.kind)}, {"normal_n", Wire(s.normal_n)},
            {"tangent_n", Wire(s.tangent_n)}, {"moment_nm", Wire(s.moment_nm)},
            {"gap_m", Wire(s.gap_m)}, {"pull_n", Wire(s.pull_n)},
            {"half_patch_m", Wire(s.half_patch_m)}}));
    return result;
}

WireObject terms_wire(const rider::NamedEntries<rider::JointTerms> &terms) {
    WireObject result;
    for (const auto &[name, t] : terms) {
        WireObject fields;
        if (t.requested_present)
            fields.emplace_back("requested_nm", Wire(t.requested_nm));
        fields.insert(fields.end(), {{"command_nm", Wire(t.command_nm)},
                          {"posture_nm", Wire(t.posture_nm)},
                          {"pedaling_nm", Wire(t.pedaling_nm)},
                          {"saturated", Wire(t.saturated)}});
        const auto optional = [&](const char *key, const std::optional<double> &value) {
            if (value.has_value()) fields.emplace_back(key, Wire(*value));
        };
        optional("support_nm", t.support_nm);
        optional("tracking_nm", t.tracking_nm);
        optional("hip_posture_nm", t.hip_posture_nm);
        optional("active_request_nm", t.active_request_nm);
        optional("active_delivered_nm", t.active_delivered_nm);
        optional("passive_damping_nm", t.passive_damping_nm);
        optional("solved_force_nm", t.solved_force_nm);
        optional("solved_active_nm", t.solved_active_nm);
        optional("solved_passive_nm", t.solved_passive_nm);
        result.emplace_back(name, Wire(std::move(fields)));
    }
    return result;
}

StepWork raw_work(const RawStep &raw, double dt_s) {
    std::vector<double> muscle, constraint;
    muscle.reserve(raw.components.size());
    constraint.reserve(raw.numerical_constraint_power_w.size());
    double motor = 0.;
    for (const auto &[name, force] : raw.components) {
        if (name == "human_crank" || name.starts_with("act_rider_"))
            muscle.push_back(dot(force, raw.qvel));
        if (name == "mid_drive") motor = dot(force, raw.qvel);
    }
    for (const auto &[name, power] : raw.numerical_constraint_power_w) {
        (void)name;
        constraint.push_back(power);
    }
    return step_work(muscle, motor, constraint, dt_s);
}

std::vector<std::string> attachment_errors(const RawStep &raw,
                                           const AttachmentSamples &samples) {
    auto errors = raw.attachment_errors;
    for (const auto &[name, item] : raw.attachment_raw) {
        (void)item;
        if (std::ranges::none_of(samples, [&](const auto &sample) { return sample.first == name; }))
            errors.push_back(name + ":unobservable_attachment_wrench");
    }
    return errors;
}

std::vector<std::string> interval_violations(const RawStep &raw,
                                             const AttachmentSamples &samples,
                                             const AttachmentBudget &budget,
                                             const EffortObservation &effort) {
    auto errors = attachment_errors(raw, samples);
    for (const auto &[name, item] : raw.attachment_raw) {
        (void)item;
        const auto found = std::ranges::find_if(samples, [&](const auto &sample) {
            return sample.first == name;
        });
        if (found != samples.end())
            for (const auto &error : attachment_violations(found->second, budget)) {
                errors.emplace_back(name);
                errors.back() += '.';
                errors.back() += error;
            }
    }
    errors.insert(errors.end(), effort.violations.begin(), effort.violations.end());
    return errors;
}

// CPython sum of a float generator uses compensation. Source names form a
// fixed set; its iteration order has no physical meaning, unlike force folds.
double external_work(const RawStep &raw, double dt_s) {
    double sum = 0., correction = 0.;
    for (const auto name : {"external", "rear_drive", "road_rolling", "aerodynamic", "native_contact"}) {
        const auto found = std::ranges::find_if(raw.components, [&](const auto &entry) {
            return entry.first == name;
        });
        if (found == raw.components.end()) continue;
        const double value = dot(found->second, raw.qvel) * dt_s;
        const double next = sum + value;
        correction += std::abs(sum) >= std::abs(value)
                          ? (sum - next) + value : (value - next) + sum;
        sum = next;
    }
    return sum + correction;
}

// `total_j`/`elastic` are the row's EFFECTIVE energy: the raw's own
// measurement on measured intervals, the period's closing forward
// otherwise (physical_runtime.py:783-789 — the oracle's `total, elastic`
// after the unmeasured-raw substitution). The ledger mirrors the
// published values; residual_j subtracts the row's own accumulators.
// `source` names the interval the published measurement came from — the
// row itself when `measured`, the closing/previous measurement otherwise
// (nullopt only before any measurement has ever committed).
WireObject account_energy(AccountingState &state, const AccountingConfig &config,
                           const RawStep &raw, const StepWork &work,
                           double total_j, const WireObject &elastic,
                           bool measured,
                           const std::optional<MeasurementSource> &source) {
    auto &e = state.energy;
    if (!e.initial_energy_j.has_value() || !e.initial_battery_j.has_value())
        throw std::invalid_argument("accounting requires initialized energy baselines");
    e.loss_j += raw.loss_step_j;
    e.muscle_signed_j += work.muscle_signed_j;
    e.muscle_positive_j += work.muscle_positive_j;
    e.motor_signed_j += work.motor_signed_j;
    e.motor_positive_j += work.motor_positive_j;
    e.constraint_absolute_j += work.constraint_absolute_j;
    e.solver_work_j += work.constraint_signed_j;
    e.active_work_j += work.muscle_signed_j + work.motor_signed_j;
    e.external_work_j += external_work(raw, config.timestep_s);
    e.electrical_work_j += number(raw.electrical_power_w) * config.timestep_s;
    e.mechanical_energy_j = total_j;
    e.elastic_energy_j = elastic;
    e.residual_j = total_j - *e.initial_energy_j - e.active_work_j -
                   e.external_work_j + e.loss_j;
    const double source_positive = e.muscle_positive_j + e.motor_positive_j;
    const bool work_ok = constraint_work_ok(e.constraint_absolute_j, source_positive);
    WireObject result{
        {"mechanical_energy_j", Wire(total_j)},
        {"elastic_energy_j", Wire(elastic)},
        {"active_work_j", Wire(e.active_work_j)}, {"external_work_j", Wire(e.external_work_j)},
        {"loss_j", Wire(e.loss_j)}, {"loss_step_j", Wire(raw.loss_step_j)},
        {"solver_constraint_work_j", Wire(e.solver_work_j)}, {"energy_scale_j", Wire(e.energy_scale_j)},
        {"muscle_signed_j", Wire(e.muscle_signed_j)}, {"muscle_positive_j", Wire(e.muscle_positive_j)},
        {"motor_signed_j", Wire(e.motor_signed_j)}, {"motor_positive_j", Wire(e.motor_positive_j)},
        {"source_positive_work_j", Wire(source_positive)}, {"constraint_signed_j", Wire(e.solver_work_j)},
        {"constraint_absolute_j", Wire(e.constraint_absolute_j)},
        {"constraint_work_ratio", source_positive > 0. ? Wire(e.constraint_absolute_j / source_positive) : Wire(nullptr)},
        {"constraint_work_ok", Wire(work_ok)}, {"residual_j", Wire(e.residual_j)},
        {"electrical_work_j", Wire(e.electrical_work_j)},
        {"electrical_residual_j", Wire(config.battery_enabled
            ? *e.initial_battery_j - raw.battery_energy_j - e.electrical_work_j : 0.)},
        {"measured", Wire(measured)},
        {"source_interval_id", source.has_value() ? Wire(source->interval_id)
                                                  : Wire(nullptr)},
        {"source_time_s", source.has_value() ? Wire(source->time_s)
                                             : Wire(nullptr)}};
    for (const auto &[key, value] : result)
        if (std::holds_alternative<double>(value.value)) finite(number(value), key);
    return result;
}

// `endpoint_mass` is the row's EFFECTIVE endpoint datum — the measured
// raw's own dict, or the forwarded last-measured one for an unmeasured
// close-out row (physical_runtime.py:824-831). By value so each
// published sample owns one deep copy, like the oracle's dict();
// raw.diagnostics is never mutated (restore returns captures untouched).
void add_detail(WireObject &channels, const RawStep &raw, Wire endpoint_mass,
                const AttachmentSamples &attachments, const EffortObservation &effort,
                const WorkHistory &history, double dt_s) {
    set(channels, "rider", required(raw.diagnostics, "rider"));
    set(channels, "rider_welds", required(raw.diagnostics, "rider_welds"));
    set(channels, "endpoint_mass", std::move(endpoint_mass));
    for (const auto key : {"rider_ik_saturation", "rider_support_targets"})
        set(channels, key, required(raw.diagnostics, key));
    const Wire &intent = required(raw.diagnostics, "rider_intent");
    WireObject intent_fields;
    if (!is_none(intent)) {
        intent_fields = object(intent);
        set(intent_fields, "inclination_rad", required(raw.diagnostics, "inclination_rad"));
    }
    set(channels, "rider_intent", Wire(std::move(intent_fields)));
    WireObject allocation;
    for (const auto &[key, value] : object(required(raw.diagnostics, "rider_allocation")))
        if (!key.starts_with("solution_")) allocation.emplace_back(key, value);
    set(channels, "rider_allocation", Wire(std::move(allocation)));
    set(channels, "attachment_samples", Wire(attachment_wire(attachments)));
    WireObject component_work;
    for (const auto &[name, force] : raw.components) {
        const Wire *previous = find(history.work_j, name);
        component_work.emplace_back(name, Wire((previous == nullptr ? 0. : number(*previous)) +
                                                dot(force, raw.qvel) * dt_s));
    }
    set(channels, "component_work_j", Wire(std::move(component_work)));
    set(channels, "rider_control", Wire(terms_wire(effort.terms)));
}

} // namespace

WireObject StepWork::as_wire() const {
    return {{"muscle_signed_j", Wire(muscle_signed_j)}, {"muscle_positive_j", Wire(muscle_positive_j)},
            {"motor_signed_j", Wire(motor_signed_j)}, {"motor_positive_j", Wire(motor_positive_j)},
            {"constraint_signed_j", Wire(constraint_signed_j)},
            {"constraint_absolute_j", Wire(constraint_absolute_j)}};
}

StepWork step_work(std::span<const double> muscle_power_w, double motor_power_w,
                    std::span<const double> constraint_power_w, double dt_s) {
    finite(motor_power_w, "motor power");
    finite(dt_s, "timestep");
    if (dt_s <= 0.) throw std::invalid_argument("invalid work timestep");
    std::vector<double> positive, absolute;
    positive.reserve(muscle_power_w.size());
    absolute.reserve(constraint_power_w.size());
    for (const double value : muscle_power_w) {
        finite(value, "muscle power");
        positive.push_back(std::max(value, 0.));
    }
    for (const double value : constraint_power_w) {
        finite(value, "constraint power");
        absolute.push_back(std::abs(value));
    }
    return {.muscle_signed_j = numpy_sum(muscle_power_w) * dt_s,
            .muscle_positive_j = numpy_sum(positive) * dt_s,
            .motor_signed_j = motor_power_w * dt_s,
            .motor_positive_j = std::max(0., motor_power_w) * dt_s,
            .constraint_signed_j = numpy_sum(constraint_power_w) * dt_s,
            .constraint_absolute_j = numpy_sum(absolute) * dt_s};
}

bool constraint_work_ok(double absolute_j, double source_positive_j, double roundoff_j) {
    finite(absolute_j, "absolute constraint work", true);
    finite(source_positive_j, "positive source work", true);
    finite(roundoff_j, "roundoff work budget", true);
    return absolute_j <= std::max(roundoff_j, .01 * source_positive_j);
}

namespace {
template<class Controller>
EffortObservation observe_effort_impl(const RawStep &raw,
                                  const Controller &controller,
                                  double dt_s) {
    finite(dt_s, "effort timestep");
    if (dt_s <= 0.) throw std::invalid_argument("effort timestep must be positive");
    EffortObservation result;
    result.channels = raw.effort_base;
    if (!raw.effort_state.has_value() || !raw.rider_control_terms.has_value())
        throw std::invalid_argument("rider interval is missing held effort evidence");
    result.diagnostics = *raw.effort_state;
    result.terms = *raw.rider_control_terms;
    auto &d = result.diagnostics;
    const auto &config = controller.config();
    rider::NamedEntries<double> delivered, joint_positive;
    std::vector<double> positive, passive;
    std::vector<std::string> power_errors, speed_errors;
    delivered.reserve(controller.joints().size());
    joint_positive.reserve(controller.joints().size());
    positive.reserve(controller.joints().size());
    passive.reserve(controller.joints().size());
    for (const auto &joint : controller.joints()) {
        const auto dof = static_cast<std::size_t>(joint.dof_adr);
        const double active = raw.actuator_force.at(static_cast<std::size_t>(joint.actuator_id));
        const double speed = raw.qvel.at(dof);
        const double damping = raw.qfrc_passive.at(dof);
        const double power = std::max(active * speed, 0.);
        delivered.emplace_back(joint.name, active);
        joint_positive.emplace_back(joint.name, power);
        positive.push_back(power);
        passive.push_back(damping * speed);
        if (power > config.joint_power_limit_w + 1e-9) power_errors.push_back(joint.name);
        if (std::abs(speed) > config.joint_speed_limit_rad_s + 1e-9) speed_errors.push_back(joint.name);
        const auto terms = std::ranges::find_if(result.terms, [&](const auto &entry) {
            return entry.first == joint.name;
        });
        if (terms == result.terms.end()) throw std::invalid_argument("rider interval is missing joint terms");
        terms->second.solved_force_nm = active;
        terms->second.solved_active_nm = active;
        terms->second.solved_passive_nm = damping;
    }
    const auto strength_errors = controller.strength_violations(delivered, raw.qpos, raw.qvel);
    d.active_delivered_nm = delivered;
    d.joint_positive_power_w = joint_positive;
    d.joint_power_violations = power_errors;
    d.joint_speed_violations = speed_errors;
    d.positive_power_w = numpy_sum(positive);
    d.passive_power_w = numpy_sum(passive);
    d.positive_work_step_j = d.positive_power_w * dt_s;
    d.passive_work_step_j = d.passive_power_w * dt_s;
    d.strength_violations = strength_errors;
    d.budget_exceeded = config.active_positive_power_limit_w.has_value() &&
                       d.positive_power_w > *config.active_positive_power_limit_w + 1e-9;
    d.observation = "solved_actuator_force_at_incoming_interval";
    d.budget_present = true;
    d.observation_present = true;
    set(result.channels, "rider_active_delivered_nm", Wire(named_reals(delivered)));
    set(result.channels, "rider_joint_positive_power_w", Wire(named_reals(joint_positive)));
    set(result.channels, "rider_joint_power_violations", strings(power_errors));
    set(result.channels, "rider_joint_speed_violations", strings(speed_errors));
    set(result.channels, "rider_positive_power_w", Wire(d.positive_power_w));
    set(result.channels, "rider_passive_power_w", Wire(d.passive_power_w));
    set(result.channels, "rider_positive_work_step_j", Wire(*d.positive_work_step_j));
    set(result.channels, "rider_passive_work_step_j", Wire(*d.passive_work_step_j));
    set(result.channels, "rider_strength_violations", strings(strength_errors));
    set(result.channels, "rider_effort_budget_exceeded", Wire(d.budget_exceeded));
    set(result.channels, "rider_effort_observation", Wire(d.observation));
    for (const auto &name : strength_errors) result.violations.push_back("rider_strength." + name);
    for (const auto &name : power_errors) result.violations.push_back("rider_power." + name);
    for (const auto &name : speed_errors) result.violations.push_back("rider_speed." + name);
    if (d.budget_exceeded) result.violations.emplace_back("rider_power.positive");
    if (raw.invalid_controller) result.violations.emplace_back("rider_controller.infeasible");
    return result;
}
} // namespace

EffortObservation observe_effort(const RawStep& raw,
    const rider::SpindleController& controller, double dt_s) {
    return observe_effort_impl(raw, controller, dt_s);
}

EffortContext::EffortContext(const rider::SpindleController& controller,
    std::size_t nq, std::size_t nv)
    : config_(controller.config()), joints_(controller.joints()), nq_(nq), nv_(nv) {
    for (const auto& joint : joints_)
        if (joint.qpos_adr < 0 || joint.dof_adr < 0 || joint.actuator_id < 0 ||
            std::cmp_greater_equal(joint.qpos_adr, nq_) ||
            std::cmp_greater_equal(joint.dof_adr, nv_))
            throw std::invalid_argument("async effort context has invalid joint addresses");
}

std::vector<std::string> EffortContext::strength_violations(
    const rider::NamedEntries<double>& active, std::span<const double> qpos,
    std::span<const double> qvel) const {
    if (!config_.has_strength) return {};
    if (qpos.size() != nq_ || qvel.size() != nv_)
        throw std::invalid_argument("strength state width mismatch");
    std::vector<std::string> result;
    for (const auto& joint : joints_) {
        const auto found = std::ranges::find_if(active, [&](const auto& item) { return item.first == joint.name; });
        const double torque = found == active.end() ? 0. : found->second;
        // Same statements/order as SpindleController's three pure strength
        // methods; only model dimensions have become owned numeric values.
        const auto coordinate = config_.strength_coordinates.find(joint.name);
        if (coordinate == config_.strength_coordinates.end()) throw std::out_of_range(joint.name);
        const double anatomical = spindle::anatomical_angle(coordinate->second,
            qpos[static_cast<std::size_t>(joint.qpos_adr)]);
        double capacity = std::numeric_limits<double>::infinity();
        if (torque != 0.) {
            const int direction = torque > 0. ? 1 : -1;
            const auto strength = config_.strength.find(joint.name);
            if (strength == config_.strength.end()) throw std::out_of_range(joint.name);
            const auto& curve = direction > 0 ? strength->second.positive : strength->second.negative;
            const double angle = std::min(std::max(anatomical, curve.angles_rad.front()), curve.angles_rad.back());
            capacity = spindle::directional_capacity(curve, angle,
                qvel[static_cast<std::size_t>(joint.dof_adr)], direction);
        }
        if (std::abs(torque) > capacity * (1. + 1e-6) + 1e-9) result.push_back(joint.name);
    }
    return result;
}

PeriodAccounting::PeriodAccounting(AccountingConfig config, AccountingState initial,
                                     int generation, rider::SpindleController *controller,
                                     writers::RiderContactWriter *contacts)
    : config_(config), state_(std::move(initial)), generation_(generation),
      buffer_(config_.period_steps, next_interval(state_.history), state_.history.last_end),
      workspace_(static_cast<rider::lapack_int>(config_.max_dofs), 6, 1),
      controller_(controller), contacts_(contacts) {
    finite(config_.timestep_s, "accounting timestep");
    if (config_.timestep_s <= 0. || config_.measurement_decimation < 1 || generation_ < 1)
        throw std::invalid_argument("invalid accounting configuration");
    if (!state_.energy.initial_energy_j.has_value() || !state_.energy.initial_battery_j.has_value())
        throw std::invalid_argument("accounting bootstrap requires initialized energy baselines");
    if (state_.energy_channels.empty())
        state_.energy_channels = {{"mechanical_energy_j", Wire(state_.energy.mechanical_energy_j)},
            {"elastic_energy_j", Wire(state_.energy.elastic_energy_j)},
            {"residual_j", Wire(state_.energy.residual_j)},
            {"energy_scale_j", Wire(state_.energy.energy_scale_j)}};
    if (config_.async_telemetry) {
        if (controller_) effort_context_.emplace(*controller_, config_.max_qpos, config_.max_dofs);
        async_ = std::make_unique<AsyncCloseRunner>(config_, state_, generation_, effort_context_);
    }
}

PeriodAccounting::PeriodAccounting(AccountingConfig config, AccountingState state,
    int generation, std::optional<EffortContext> effort)
    : PeriodAccounting(config, std::move(state), generation, nullptr, nullptr) {
    effort_context_ = std::move(effort);
}
PeriodAccounting::~PeriodAccounting() = default;

void PeriodAccounting::push(RawStep raw) { buffer_.push(std::move(raw)); }

bool PeriodAccounting::has_raws() const {
    return !buffer_.empty() || detached_at_ < detached_.size() ||
        (async_ && async_->pending() != 0);
}

void PeriodAccounting::apply_diagnostics(AccountingDiagnostics diagnostics) noexcept {
    if (contacts_) contacts_->publish_accounted_attachments(
        std::move(diagnostics.contacts), std::move(diagnostics.attachment_errors));
    if (controller_) controller_->publish_accounted_effort(
        std::move(diagnostics.effort), std::move(diagnostics.terms));
}

AccountingPublication PeriodAccounting::take_publication() {
    if (pending_.empty()) throw std::logic_error("closed period produced no samples");
    AccountingPublication result;
    result.state = state_; // Stage the only throwing copy before moving evidence.
    result.samples = std::move(pending_);
    result.diagnostics = std::move(last_diagnostics_);
    // A warning is transported once. The main owner merges its unconsumed
    // warning with later publications, independently of worker scheduling.
    state_.monitor.warning_pending = false;
    return result;
}

void PeriodAccounting::apply_publication(const AccountingPublication& publication) {
    if (publication.samples.empty() ||
        publication.samples.front()->interval_id != next_interval(state_.history) ||
        publication.state.history.last_id != publication.samples.back()->interval_id)
        throw std::invalid_argument("async publication is not the next accounting prefix");
    AccountingState staged = publication.state;
    staged.monitor.warning_pending = staged.monitor.warning_pending || state_.monitor.warning_pending;
    AccountingDiagnostics diagnostics = publication.diagnostics;
    std::vector<SamplePtr> retained;
    retained.reserve(config_.retention == RetentionMode::off ? 0 : publication.samples.size());
    for (const auto& sample : publication.samples)
        if (config_.retention == RetentionMode::full ||
            (config_.retention == RetentionMode::legacy &&
             sample->interval_id % config_.measurement_decimation == 0))
            retained.push_back(sample);
    pending_.reserve(pending_.size() + publication.samples.size());
    retained_.reserve(retained_.size() + retained.size());
    // Everything below is nonthrowing: acknowledge only after this commit.
    state_ = std::move(staged);
    latest_ = publication.samples.back();
    pending_.insert(pending_.end(), publication.samples.begin(), publication.samples.end());
    retained_.insert(retained_.end(), retained.begin(), retained.end());
    apply_diagnostics(std::move(diagnostics));
}

void PeriodAccounting::detach_async() {
    if (async_) {
        if (detached_at_ != detached_.size()) throw std::logic_error("two detached accounting queues");
        detached_ = async_->detach_pending();
        detached_at_ = 0;
        async_.reset();
        config_.async_telemetry = false;
    }
    while (detached_at_ < detached_.size()) {
        AccountingConfig config = config_;
        config.async_telemetry = false;
        config.retention = RetentionMode::off;
        PeriodAccounting evaluator(config, state_, generation_, effort_context_);
        for (const auto& raw : *detached_[detached_at_]) evaluator.push(raw);
        std::optional<std::string> strict_error;
        try { evaluator.flush_local(); }
        catch (const InvalidReferenceRun& error) { strict_error = error.what(); }
        auto const result = evaluator.take_publication();
        apply_publication(result);
        detached_[detached_at_++].reset();
        if (strict_error) throw InvalidReferenceRun(*strict_error);
    }
    detached_.clear();
    detached_at_ = 0;
}

void PeriodAccounting::collect(bool block) {
    if (detached_at_ < detached_.size()) detach_async();
    if (!async_) return;
    try {
        while (const auto result = async_->peek(block)) {
            apply_publication(result->publication);
            async_->acknowledge(result->sequence);
            if (result->publication.strict_error)
                throw InvalidReferenceRun(*result->publication.strict_error);
        }
    } catch (const AsyncWorkerLost&) {
        detach_async();
    }
}

void PeriodAccounting::submit_period() {
    if (buffer_.empty()) return;
    if (!async_) { flush_local(); return; }
    // Bound the off-path transport independently of recording retention. A
    // slow worker cannot turn an unrecorded ride into growing raw history.
    if (async_->pending() >= 2) collect(true);
    if (!async_) { flush_local(); return; }
    auto originals = std::make_shared<const std::vector<RawStep>>(
        buffer_.raws().begin(), buffer_.raws().end());
    try { async_->submit(std::move(originals)); }
    catch (const AsyncWorkerLost&) {
        // A normal worker publication error remains an error; only lost
        // transport falls back to synchronous replay.
        collect(false);
        if (async_) detach_async();
        flush_local();
        return;
    }
    buffer_.clear(); // Originals are owned by the acknowledged queue now.
}

void PeriodAccounting::close_period() {
    if (buffer_.empty()) return;
    if (!buffer_.full()) throw std::invalid_argument("automatic close requires a complete period");
    submit_period();
}

void PeriodAccounting::flush() {
    if (detached_at_ < detached_.size()) detach_async();
    submit_period();
    collect(true);
}

void PeriodAccounting::flush_local() {
    if (buffer_.empty()) return;
    // No public state changes until all rows, validation and allocations succeed.
    AccountingState staged = state_;
    staged.period_violations.clear();
    // Publication donates each row's captures to its sample instead of
    // deep-copying them. Until the commit below, every donation can be put
    // back: `channels` holds the in-flight row's channel object while
    // `channels_donor` names it, and published rows keep theirs in the
    // not-yet-visible sample (snapshots in a parallel non-const slot, since
    // samples share them as shared_ptr<const>). The period therefore stays
    // retryable exactly as before. Restored channels may carry merged-in keys
    // that a copy would not have produced, but every such key is written again
    // by the deterministic puts below — overwritten in place with the
    // identical value at the identical position — so a retried period
    // publishes byte-identical samples.
    const auto raws = buffer_.mutable_raws();
    // Oracle _close_period (physical_runtime.py:741-757): the
    // energy/momentum solve runs on measured (raw.full) intervals only.
    // A raw that measured its state reports its own energy; an
    // unmeasured raw carries the closing measurement of its period. A
    // crash or flush can drain a partial period holding no measured
    // interval at all — then the ledger keeps its previous datum instead
    // of fabricating one. The scan is read-only: raws stay intact for the
    // retry path, and staged.last_endpoint_mass commits only with the
    // rest of the staged state.
    std::optional<double> closing_total;
    std::optional<WireObject> closing_elastic;
    std::optional<MeasurementSource> closing_source;
    std::optional<WireObject> endpoint_mass = staged.last_endpoint_mass;
    for (const RawStep &raw : std::ranges::reverse_view(raws)) {
        if (!raw.mechanical_energy_j.has_value()) continue;
        closing_total = raw.mechanical_energy_j;
        closing_elastic = raw.elastic_energy_j;
        closing_source = MeasurementSource{.interval_id = raw.interval_id,
                                           .time_s = raw.end_time_s};
        const Wire &captured = required(raw.diagnostics, "endpoint_mass");
        endpoint_mass = is_none(captured)
                            ? std::nullopt
                            : std::optional<WireObject>(object(captured));
        break;
    }
    if (!closing_total.has_value()) {
        closing_total = staged.energy.mechanical_energy_j;
        closing_elastic = staged.energy.elastic_energy_j;
    }
    staged.last_endpoint_mass = endpoint_mass;
    // Provenance for every forwarded row in this period: the period's
    // closing measured interval when one exists, else the previous
    // period's closing measurement (nullopt until the first measurement
    // ever commits — the catalog encodes that as explicit absence).
    const std::optional<MeasurementSource> forwarding_source =
        closing_source.has_value() ? closing_source
                                   : staged.last_measurement_source;
    if (closing_source.has_value())
        staged.last_measurement_source = *closing_source;
    struct PublishedRow {
        std::shared_ptr<PhysicalSampleData> sample;
        std::shared_ptr<ConstraintSnapshot> snapshot;  // restore donor
    };
    std::vector<PublishedRow> published;
    published.reserve(raws.size());
    WireObject channels;
    RawStep *channels_donor = nullptr;
    AttachmentSamples final_attachments;
    std::vector<std::string> final_attachment_errors;
    EffortObservation final_effort;
    std::vector<SamplePtr> selected;
    std::vector<std::pair<std::string, writers::RiderAttachmentEntry>> contact_samples;
    std::optional<ReferenceFailure> strict_failure;
    const auto restore = [&]() noexcept {
        if (channels_donor != nullptr)
            channels_donor->channels = std::move(channels);
        for (std::size_t i = 0; i < published.size(); ++i) {
            RawStep &raw = raws[i];
            PhysicalSampleData &sample = *published[i].sample;
            if (!sample.channels.empty())
                raw.channels = std::move(sample.channels);
            if (published[i].snapshot != nullptr)
                raw.constraint_snapshot = std::move(*published[i].snapshot);
            raw.qpos = std::move(sample.qpos);
            raw.qvel = std::move(sample.qvel);
            raw.components = std::move(sample.forces);
        }
    };
    try {
        for (std::size_t index = 0; index < raws.size(); ++index) {
            RawStep &raw = raws[index];
            const auto &constraints = raw.constraint_snapshot;
            if (std::abs(constraints.interval_start_s - raw.time_s) > 1e-12 ||
                std::abs(constraints.interval_end_s - raw.end_time_s) > 1e-12 ||
                !std::ranges::equal(constraints.qvel_start, raw.qvel))
                throw std::invalid_argument("constraint snapshot does not match the physical interval");
            auto attachments = evaluate_attachments(raw, workspace_);
            auto effort = effort_context_ ? observe_effort_impl(raw, *effort_context_, config_.timestep_s)
                : controller_ ? observe_effort(raw, *controller_, config_.timestep_s) : EffortObservation{};
            auto violations = interval_violations(raw, attachments, config_.attachment_budget, effort);
            // Oracle (physical_runtime.py:783-789): a measured raw
            // reports its own energy; an unmeasured one publishes the
            // period's closing total/elastic — dict(closing_elastic)
            // semantics are automatic here since account_energy emits a
            // fresh WireObject per row.
            const bool measured = raw.mechanical_energy_j.has_value();
            const double total_j = measured ? *raw.mechanical_energy_j
                                            : *closing_total;
            const WireObject &elastic =
                measured ? engaged(raw.elastic_energy_j)
                         : engaged(closing_elastic);
            // A measured row cites itself; a forwarded row cites the
            // interval its measurement came from (forwarding_source —
            // nullopt only while no measurement has ever committed).
            const std::optional<MeasurementSource> source =
                measured ? std::optional<MeasurementSource>(
                               MeasurementSource{.interval_id = raw.interval_id,
                                                 .time_s = raw.end_time_s})
                         : forwarding_source;
            staged.energy_channels = account_energy(staged, config_, raw, raw_work(raw, config_.timestep_s),
                                                    total_j, elastic, measured, source);
            if (index + 1 == raws.size() &&
                !boolean(required(staged.energy_channels, "constraint_work_ok")))
                violations.emplace_back("energy.constraint_work");
            channels = std::move(raw.channels);
            channels_donor = &raw;
            merge(channels, std::move(effort.channels));
            // The committed energy channels are the final row's object;
            // earlier rows may donate theirs to the published channel.
            set(channels, "energy",
                index + 1 == raws.size() ? Wire(staged.energy_channels)
                                         : Wire(std::move(staged.energy_channels)));
            set(channels, "attachment_violations", strings(violations));
            if (raw.full || index + 1 == raws.size()) {
                // endpoint_mass: the raw's own measured dict, else the
                // forwarded last-measured datum (physical_runtime.py
                // :824-831). The by-value lane keeps the measured path at
                // exactly one deep copy, matching the old set() call.
                const Wire &captured = required(raw.diagnostics, "endpoint_mass");
                add_detail(channels, raw,
                           is_none(captured)
                               ? (staged.last_endpoint_mass.has_value()
                                      ? Wire(*staged.last_endpoint_mass)
                                      : Wire(nullptr))
                               : captured,
                           attachments, effort, staged.history, config_.timestep_s);
            }
            // Energy quality judges a measurement; unmeasured intervals
            // carry a forwarded closing total whose hybrid residual must
            // not feed the (sticky) numerically_valid verdict — observe
            // the channels WITHOUT the energy lane, like the oracle's
            // masked dict.
            staged.model_status.observe(raw.interval_id, raw.time_s, channels,
                                        /*include_energy=*/measured);
            set(channels, "model_status", Wire(staged.model_status.as_wire()));
            published.push_back({.sample = std::make_shared<PhysicalSampleData>(),
                                 .snapshot = nullptr});
            PublishedRow &row = published.back();
            PhysicalSampleData &out = *row.sample;
            out.interval_id = raw.interval_id;
            out.time_s = raw.time_s;
            out.end_time_s = raw.end_time_s;
            out.qpos = std::move(raw.qpos);
            out.qvel = std::move(raw.qvel);
            out.forces = std::move(raw.components);
            row.snapshot =
                std::make_shared<ConstraintSnapshot>(std::move(raw.constraint_snapshot));
            out.constraints = row.snapshot;
            out.powers_w.reserve(out.forces.size());
            for (const auto &[name, force] : out.forces)
                out.powers_w.emplace_back(name, Wire(dot(force, out.qvel)));
            out.channels = std::move(channels);
            channels_donor = nullptr;
            staged.history.add(out);
            if (!violations.empty()) {
                ReferenceFailure failure{.time_s = raw.end_time_s, .violations = std::move(violations)};
                staged.monitor.observe(failure);
                staged.period_violations.push_back(std::move(failure));
            }
            if (index + 1 == raws.size()) {
                final_attachment_errors = attachment_errors(raw, attachments);
                final_attachments = std::move(attachments);
                final_effort = std::move(effort);
            }
        }
        // Retention is independent of the measurement cadence: `full`
        // keeps every committed interval, `legacy` the historical
        // interval_id % measurement_decimation projection, `off` none.
        // The retained index never waits on pending acknowledgement.
        selected.reserve(config_.retention == RetentionMode::off
                             ? std::size_t{0}
                             : published.size());
        for (const auto &row : published) {
            const bool keep =
                config_.retention == RetentionMode::full ||
                (config_.retention == RetentionMode::legacy &&
                 row.sample->interval_id % config_.measurement_decimation == 0);
            if (keep) selected.push_back(row.sample);
        }
        contact_samples.reserve(final_attachments.size());
        for (auto &[name, sample] : final_attachments)
            contact_samples.emplace_back(std::move(name), std::move(sample));
        // Copying/reserving can throw; the raw period and all old evidence still exist.
        pending_.reserve(pending_.size() + published.size());
        retained_.reserve(retained_.size() + selected.size());
        strict_failure = staged.monitor.strict && !staged.period_violations.empty()
                             ? std::optional<ReferenceFailure>(staged.period_violations.front())
                             : std::nullopt;
    } catch (...) {
        restore();
        throw;
    }
    // Commit uses only moves into reserved storage and nonthrowing diagnostics.
    state_ = std::move(staged);
    latest_ = published.back().sample;
    for (auto &row : published) pending_.push_back(std::move(row.sample));
    retained_.insert(retained_.end(), std::make_move_iterator(selected.begin()),
                     std::make_move_iterator(selected.end()));
    last_diagnostics_ = {.contacts = std::move(contact_samples),
        .attachment_errors = std::move(final_attachment_errors),
        .effort = std::move(final_effort.diagnostics), .terms = std::move(final_effort.terms)};
    if (controller_ || contacts_) apply_diagnostics(std::move(last_diagnostics_));
    buffer_.clear();
    // In strict mode the entire period and its original first-failure time
    // are already available to snapshot()/prepare_samples().
    if (strict_failure.has_value()) throw InvalidReferenceRun(strict_failure->message());
}

RuntimeSampleBatch PeriodAccounting::prepare_samples() const {
    return {.generation = generation_, .samples = pending_,
            .columns = make_columns(pending_, config_.columns)};
}

void PeriodAccounting::acknowledge_samples(const RuntimeSampleBatch &batch) {
    if (batch.generation != generation_ || batch.samples.size() > pending_.size())
        throw std::invalid_argument("sample acknowledgement belongs to a different generation or prefix");
    for (std::size_t i = 0; i < batch.samples.size(); ++i)
        if (batch.samples[i] != pending_[i])
            throw std::invalid_argument("sample acknowledgement is stale or not the pending prefix");
    pending_.erase(pending_.begin(), pending_.begin() +
                    static_cast<std::vector<SamplePtr>::difference_type>(batch.samples.size()));
}

WireObject PeriodAccounting::state_wire() const {
    WireArray violations;
    violations.reserve(state_.period_violations.size());
    for (const auto &failure : state_.period_violations) violations.push_back(failure.as_wire());
    std::size_t buffered = buffer_.raws().size();
    if (async_) buffered += async_->pending_intervals();
    for (std::size_t i = detached_at_; i < detached_.size(); ++i) buffered += detached_[i]->size();
    return {{"history", Wire(state_.history.as_wire())},
            {"energy", Wire(state_.energy_channels)},
            {"model_status", Wire(state_.model_status.as_wire())},
            {"monitor", Wire(state_.monitor.as_wire())},
            {"period_violations", Wire(std::move(violations))},
            {"pending_intervals", Wire(static_cast<std::int64_t>(pending_.size()))},
            {"buffered_intervals", Wire(static_cast<std::int64_t>(buffered))}};
}

SampleColumns PeriodAccounting::recorded_columns() const {
    // The retained index is authoritative; the flat projection is
    // materialized on demand rather than stored as per-period blocks.
    return make_columns(retained_, config_.columns);
}

OwnedNativeEpisode PeriodAccounting::seal_episode() const {
    return OwnedNativeEpisode::seal(generation_, retained_, config_.columns);
}

} // namespace runtime

namespace runtime {
void PeriodAccounting::acknowledge_through(std::int64_t interval_id) {
    const auto end = std::ranges::find_if(pending_,
        [interval_id](const SamplePtr &sample) { return sample->interval_id > interval_id; });
    pending_.erase(pending_.begin(), end);
}
} // namespace runtime
