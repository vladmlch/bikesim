#include "accounting.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
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
        WireObject fields{{"requested_nm", Wire(t.requested_nm)},
                          {"command_nm", Wire(t.command_nm)},
                          {"posture_nm", Wire(t.posture_nm)},
                          {"pedaling_nm", Wire(t.pedaling_nm)},
                          {"saturated", Wire(t.saturated)}};
        const auto optional = [&](const char *key, const std::optional<double> &value) {
            if (value.has_value()) fields.emplace_back(key, Wire(*value));
        };
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
            for (const auto &error : attachment_violations(found->second, budget))
                errors.push_back(name + '.' + error);
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

WireObject account_energy(AccountingState &state, const AccountingConfig &config,
                           const RawStep &raw, const StepWork &work) {
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
    e.mechanical_energy_j = raw.mechanical_energy_j;
    e.elastic_energy_j = raw.elastic_energy_j;
    e.residual_j = raw.mechanical_energy_j - *e.initial_energy_j - e.active_work_j -
                   e.external_work_j + e.loss_j;
    const double source_positive = e.muscle_positive_j + e.motor_positive_j;
    const bool work_ok = constraint_work_ok(e.constraint_absolute_j, source_positive);
    WireObject result{
        {"mechanical_energy_j", Wire(raw.mechanical_energy_j)},
        {"elastic_energy_j", Wire(raw.elastic_energy_j)},
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
            ? *e.initial_battery_j - raw.battery_energy_j - e.electrical_work_j : 0.)}};
    for (const auto &[key, value] : result)
        if (std::holds_alternative<double>(value.value)) finite(number(value), key);
    return result;
}

void add_detail(WireObject &channels, const RawStep &raw,
                const AttachmentSamples &attachments, const EffortObservation &effort,
                const WorkHistory &history, double dt_s) {
    for (const auto key : {"rider", "rider_welds", "endpoint_mass", "rider_ik_saturation", "rider_support_targets"})
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

EffortObservation observe_effort(const RawStep &raw,
                                  const rider::SpindleController &controller,
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

PeriodAccounting::PeriodAccounting(AccountingConfig config, AccountingState initial,
                                     int generation, rider::SpindleController *controller,
                                     writers::RiderContactWriter *contacts)
    : config_(std::move(config)), state_(std::move(initial)), generation_(generation),
      buffer_(config_.period_steps, next_interval(state_.history), state_.history.last_end),
      workspace_(static_cast<rider::lapack_int>(config_.max_dofs), 6, 1),
      controller_(controller), contacts_(contacts) {
    finite(config_.timestep_s, "accounting timestep");
    if (config_.timestep_s <= 0. || config_.record_decimation < 1 || generation_ < 1)
        throw std::invalid_argument("invalid accounting configuration");
    if (!state_.energy.initial_energy_j.has_value() || !state_.energy.initial_battery_j.has_value())
        throw std::invalid_argument("accounting bootstrap requires initialized energy baselines");
    if (state_.energy_channels.empty())
        state_.energy_channels = {{"mechanical_energy_j", Wire(state_.energy.mechanical_energy_j)},
            {"elastic_energy_j", Wire(state_.energy.elastic_energy_j)},
            {"residual_j", Wire(state_.energy.residual_j)},
            {"energy_scale_j", Wire(state_.energy.energy_scale_j)}};
}

void PeriodAccounting::push(RawStep raw) { buffer_.push(std::move(raw)); }

void PeriodAccounting::flush() {
    if (buffer_.empty()) return;
    // No public state changes until all rows, validation and allocations succeed.
    AccountingState staged = state_;
    staged.period_violations.clear();
    std::vector<SamplePtr> published;
    published.reserve(buffer_.raws().size());
    AttachmentSamples final_attachments;
    std::vector<std::string> final_attachment_errors;
    EffortObservation final_effort;
    const auto raws = buffer_.raws();
    for (std::size_t index = 0; index < raws.size(); ++index) {
        const RawStep &raw = raws[index];
        const auto &constraints = raw.constraint_snapshot;
        if (std::abs(constraints.interval_start_s - raw.time_s) > 1e-12 ||
            std::abs(constraints.interval_end_s - raw.end_time_s) > 1e-12 ||
            !std::ranges::equal(constraints.qvel_start, raw.qvel))
            throw std::invalid_argument("constraint snapshot does not match the physical interval");
        auto attachments = evaluate_attachments(raw, workspace_);
        auto effort = controller_ == nullptr ? EffortObservation{} :
                                               observe_effort(raw, *controller_, config_.timestep_s);
        auto violations = interval_violations(raw, attachments, config_.attachment_budget, effort);
        staged.energy_channels = account_energy(staged, config_, raw, raw_work(raw, config_.timestep_s));
        if (index + 1 == raws.size() && !boolean(required(staged.energy_channels, "constraint_work_ok")))
            violations.emplace_back("energy.constraint_work");
        WireObject channels = raw.channels;
        merge(channels, effort.channels);
        set(channels, "energy", Wire(staged.energy_channels));
        set(channels, "attachment_violations", strings(violations));
        if (raw.full || index + 1 == raws.size())
            add_detail(channels, raw, attachments, effort, staged.history, config_.timestep_s);
        staged.model_status.observe(raw.interval_id, raw.time_s, channels);
        set(channels, "model_status", Wire(staged.model_status.as_wire()));
        auto sample = std::make_shared<PhysicalSampleData>();
        sample->interval_id = raw.interval_id;
        sample->time_s = raw.time_s;
        sample->end_time_s = raw.end_time_s;
        sample->qpos = raw.qpos;
        sample->qvel = raw.qvel;
        sample->forces = raw.components;
        sample->constraints = std::make_shared<ConstraintSnapshot>(constraints);
        for (const auto &[name, force] : raw.components)
            sample->powers_w.emplace_back(name, Wire(dot(force, raw.qvel)));
        sample->channels = std::move(channels);
        staged.history.add(*sample);
        published.push_back(std::move(sample));
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
    std::vector<SamplePtr> selected;
    for (const auto &sample : published)
        if (sample->interval_id % config_.record_decimation == 0) selected.push_back(sample);
    RecordedBlock block{.rows = selected.size(), .columns = make_columns(selected, config_.columns)};
    std::vector<std::pair<std::string, writers::RiderAttachmentEntry>> contact_samples;
    for (auto &[name, sample] : final_attachments)
        contact_samples.emplace_back(std::move(name), std::move(sample));
    // Copying/reserving can throw; the raw period and all old evidence still exist.
    pending_.reserve(pending_.size() + published.size());
    recorded_.reserve(recorded_.size() + (block.rows == 0 ? 0U : 1U));
    const auto strict_failure = staged.monitor.strict && !staged.period_violations.empty()
                                    ? std::optional<ReferenceFailure>(staged.period_violations.front())
                                    : std::nullopt;
    // Commit uses only moves into reserved storage and nonthrowing diagnostics.
    state_ = std::move(staged);
    latest_ = published.back();
    pending_.insert(pending_.end(), std::make_move_iterator(published.begin()),
                     std::make_move_iterator(published.end()));
    if (block.rows != 0) recorded_.push_back(std::move(block));
    if (contacts_ != nullptr)
        contacts_->publish_accounted_attachments(std::move(contact_samples),
                                                 std::move(final_attachment_errors));
    if (controller_ != nullptr)
        controller_->publish_accounted_effort(std::move(final_effort.diagnostics),
                                               std::move(final_effort.terms));
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
    for (const auto &failure : state_.period_violations) violations.push_back(failure.as_wire());
    return {{"history", Wire(state_.history.as_wire())},
            {"energy", Wire(state_.energy_channels)},
            {"model_status", Wire(state_.model_status.as_wire())},
            {"monitor", Wire(state_.monitor.as_wire())},
            {"period_violations", Wire(std::move(violations))},
            {"pending_intervals", Wire(static_cast<std::int64_t>(pending_.size()))},
            {"buffered_intervals", Wire(static_cast<std::int64_t>(buffer_.raws().size()))}};
}

SampleColumns PeriodAccounting::recorded_columns() const { return join_columns(recorded_); }

} // namespace runtime
