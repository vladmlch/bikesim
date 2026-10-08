// rider/intent.cpp — implementation of the seated-climb intent stack.
// Every function mirrors the Python statement it names; comments cite the
// source file. Ordering and clamp argument order are part of the FP
// contract — std::min/std::max match Python's min/max NaN semantics when
// the arguments keep the same order.
#include "intent.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "../numeric_norm.hpp"
#include "../validation.hpp"
#include "spindle_math.hpp"

namespace rider {
namespace {

// Python round() is ties-to-even (std::nearbyint in the default rounding
// mode); round(huge finite) returns a big int rather than raising, so the
// projection clamps to int64 instead — a disabled resolver never reads
// period_steps_ and an enabled one fails the |n*dt - period| check either
// way, like the oracle.
[[nodiscard]] std::int64_t python_round_steps(double ratio) {
    if (!std::isfinite(ratio))
        throw std::invalid_argument(
            "rider intention period must be an integer multiple of timestep");
    const double rounded = std::nearbyint(ratio);
    if (rounded >=
        static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        return std::numeric_limits<std::int64_t>::max();
    return static_cast<std::int64_t>(rounded);
}

// RiderPosture.__post_init__ — the scalar()/array() finiteness gates and
// bounds the oracle enforces every time a posture object is constructed.
void validate_posture(const RiderPosture &posture) {
    if (!std::isfinite(posture.torso_lean_rad))
        throw std::invalid_argument("invalid torso_lean_rad");
    if (std::abs(posture.torso_lean_rad) > .8)
        throw std::invalid_argument(
            "torso_lean_rad must lie in [-0.8, 0.8]");
    if (!std::isfinite(posture.pelvis_pitch_rad))
        throw std::invalid_argument("invalid pelvis_pitch_rad");
    if (std::abs(posture.pelvis_pitch_rad) > .5)
        throw std::invalid_argument(
            "pelvis_pitch_rad must lie in [-0.5, 0.5]");
    if (posture.pelvis_offset_m.has_value()) {
        const auto &[x, z] = *posture.pelvis_offset_m;
        if (!std::isfinite(x) || !std::isfinite(z))
            throw std::invalid_argument("invalid shape or non-finite hip offset");
        if (std::abs(x) > .25 || z < -.15 || z > .25)
            throw std::invalid_argument(
                "hip offset exceeds the posture command envelope");
    }
}

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
double body_pulse(double time_s, double start_s, double duration_s,
                  double amplitude_rad) {
    if (!(std::isfinite(time_s) && std::isfinite(start_s) &&
          std::isfinite(duration_s) && std::isfinite(amplitude_rad)))
        throw std::invalid_argument("nonfinite posture program");
    if (duration_s <= 0.)
        throw std::invalid_argument("pulse duration must be positive");
    const double phase = (time_s - start_s) / duration_s;
    if (!(0. < phase && phase < 1.))
        return 0.;
    return amplitude_rad * 64. * std::pow(phase, 3.) *
           std::pow(1. - phase, 3.);
}

// ---- SeatedPostureProgram (rider_program.py) ------------------------------

SeatedPostureProgram::SeatedPostureProgram(const IntentConfig &config)
    : config_(config) {}

void SeatedPostureProgram::reset() {
    lean_rad_ = 0.;
    trim_rad_ = 0.;
    load_samples_.clear();
    delayed_load_share_.reset();
    pulses_.clear();
}

void SeatedPostureProgram::schedule_pulse(double start_s, double duration_s,
                                          double amplitude_rad) {
    if (!std::isfinite(start_s))
        throw std::invalid_argument("nonfinite pulse start");
    if (!std::isfinite(duration_s))
        throw std::invalid_argument("nonfinite pulse duration");
    if (!std::isfinite(amplitude_rad))
        throw std::invalid_argument("nonfinite pulse amplitude");
    if (duration_s <= 0.)
        throw std::invalid_argument("pulse duration must be positive");
    pulses_.push_back({start_s, duration_s, amplitude_rad});
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
double SeatedPostureProgram::update(
    double time_s, double road_grade, double dt_s,
    std::optional<double> front_load_share,
    std::optional<double> lean_limit_rad,
    std::optional<double> dead_time_s) {
    if (!(std::isfinite(time_s) && std::isfinite(road_grade)))
        throw std::invalid_argument("nonfinite posture program input");
    if (!(std::isfinite(dt_s) && dt_s > 0.))
        throw std::invalid_argument(
            "posture program interval must be positive");
    const double delay = dead_time_s.value_or(config_.trim_dead_time_s);
    if (!std::isfinite(delay) || delay < 0.)
        throw std::invalid_argument(
            "posture load delay must be finite and nonnegative");
    if (front_load_share.has_value() &&
        !(std::isfinite(*front_load_share) && *front_load_share >= 0. &&
          *front_load_share <= 1.))
        throw std::invalid_argument(
            "front load share must be between zero and one");
    if (lean_limit_rad.has_value() &&
        !(std::isfinite(*lean_limit_rad) && *lean_limit_rad >= 0.))
        throw std::invalid_argument(
            "lean limit must be finite and nonnegative");
    load_samples_.emplace_back(time_s, front_load_share);
    while (!load_samples_.empty() &&
           load_samples_.front().first <= time_s - delay + 1e-12) {
        delayed_load_share_ = load_samples_.front().second;
        load_samples_.pop_front();
    }
    if (delayed_load_share_.has_value()) {
        trim_rad_ += config_.lean_trim_gain_rad_s *
                     (config_.front_load_share_target -
                      *delayed_load_share_) *
                     dt_s;
        trim_rad_ = std::max(-config_.lean_trim_limit_rad,
                             std::min(config_.lean_trim_limit_rad,
                                      trim_rad_));
    }
    const double forward_limit =
        lean_limit_rad.value_or(config_.max_forward_lean_rad);
    double extra = 0.;
    for (const auto &[start, duration, amplitude] : pulses_)
        extra += body_pulse(time_s, start, duration, amplitude);
    const double target = std::max(
        -config_.max_backward_lean_rad,
        std::min(forward_limit,
                 config_.lean_gain * std::atan(road_grade) + trim_rad_ +
                     extra));
    lean_rad_ += std::max(-config_.lean_rate_rad_s * dt_s,
                          std::min(config_.lean_rate_rad_s * dt_s,
                                   target - lean_rad_));
    return lean_rad_;
}

SeatedPostureProgram::State SeatedPostureProgram::state() const {
    return {.lean_rad = lean_rad_,
            .trim_rad = trim_rad_,
            .load_samples = load_samples_,
            .delayed_load_share = delayed_load_share_,
            .pulses = pulses_};
}

void SeatedPostureProgram::restore(const State &state) {
    lean_rad_ = state.lean_rad;
    trim_rad_ = state.trim_rad;
    load_samples_ = state.load_samples;
    delayed_load_share_ = state.delayed_load_share;
    pulses_ = state.pulses;
}

// ---- SeatedClimbPolicy (seated_climb.py) ----------------------------------

SeatedClimbPolicy::SeatedClimbPolicy(const IntentConfig &config)
    : config_(config), program_(config) {
    reset();
}

void SeatedClimbPolicy::reset() {
    time_s_ = 0.;
    inclination_rad_ = 0.;
    lean_rad_ = 0.;
    effort_nm_ = 0.;
    samples_.clear();
    delayed_.reset();
    surge_budget_s_left_ = config_.surge_budget_s;
    program_.reset();
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
double SeatedClimbPolicy::power_target_w(double preview_grade, double dt_s) {
    const double grade =
        validation::finite(preview_grade, "surge preview grade");
    const double dt = validation::positive(dt_s, "surge interval");
    if (grade >= config_.surge_grade) {
        if (surge_budget_s_left_ + 1e-12 >= dt) {
            surge_budget_s_left_ =
                std::max(0., surge_budget_s_left_ - dt);
            return config_.surge_power_w;
        }
    } else {
        surge_budget_s_left_ =
            std::min(config_.surge_budget_s,
                     surge_budget_s_left_ +
                         config_.surge_recovery_rate * dt);
    }
    return config_.target_crank_power_w;
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters) fixed positional Python signature
Intent SeatedClimbPolicy::update(const IntentSignals &signals, double dt_s,
                                 double road_grade,
                                 std::optional<double> preview_grade,
                                 std::optional<double> lean_limit_rad) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    const double dt = validation::positive(dt_s, "rider intention interval");
    validation::finite(road_grade, "posture road grade");
    const double preview =
        preview_grade.has_value()
            ? validation::finite(*preview_grade, "posture preview grade")
            : road_grade;
    if (!config_.enabled)
        return {};
    const double power_target = power_target_w(preview, dt);
    samples_.emplace_back(time_s_, signals);
    const double ready_time = time_s_ - config_.reaction_delay_s;
    while (!samples_.empty() &&
           samples_.front().first <= ready_time + 1e-12) {
        delayed_ = samples_.front().second;
        samples_.pop_front();
    }
    if (delayed_.has_value()) {
        const IntentSignals &delayed = *delayed_;
        inclination_rad_ += delayed.pitch_rate_up_rad_s * dt;
        const double forward = delayed.specific_force_body_mps2[0];
        const double lateral = delayed.specific_force_body_mps2[1];
        const double upward = delayed.specific_force_body_mps2[2];
        const std::array<double, 3> force{forward, lateral, upward};
        const double norm = numeric::python_vector_norm(force);
        if (.8 * 9.81 <= norm && norm <= 1.2 * 9.81) {
            const double estimate = std::atan2(forward, upward);
            const double difference =
                spindle::wrap_angle(estimate - inclination_rad_);
            inclination_rad_ +=
                (1. - std::exp(-dt / config_.orientation_tau_s)) *
                difference;
        }
        inclination_rad_ = spindle::wrap_angle(inclination_rad_);
        const double target_effort = spindle::crank_effort_ceiling(
            power_target, config_.max_crank_torque_nm,
            delayed.crank_rate_rad_s);
        effort_nm_ += std::max(-config_.torque_slew_nm_s * dt,
                               std::min(config_.torque_slew_nm_s * dt,
                                        target_effort - effort_nm_));
    }
    lean_rad_ = program_.update(time_s_, preview, dt,
                                signals.front_load_share, lean_limit_rad);
    time_s_ += dt;
    RiderPosture posture;
    posture.torso_lean_rad = lean_rad_;
    posture.use_saddle = true;
    validate_posture(posture);
    return {.posture = posture, .effort_ceiling_nm = effort_nm_};
}

SeatedClimbPolicy::State SeatedClimbPolicy::state() const {
    return {.time_s = time_s_,
            .inclination_rad = inclination_rad_,
            .lean_rad = lean_rad_,
            .effort_nm = effort_nm_,
            .samples = samples_,
            .delayed = delayed_,
            .surge_budget_s_left = surge_budget_s_left_};
}

void SeatedClimbPolicy::restore(const State &state) {
    time_s_ = state.time_s;
    inclination_rad_ = state.inclination_rad;
    lean_rad_ = state.lean_rad;
    effort_nm_ = state.effort_nm;
    samples_ = state.samples;
    delayed_ = state.delayed;
    surge_budget_s_left_ = state.surge_budget_s_left;
}

// ---- RiderIntentResolver (rider_intent.py) --------------------------------

RiderIntent::RiderIntent(IntentConfig config, double dt_s)
    : config_(config),
      dt_s_(validation::positive(dt_s, "rider physics timestep")),
      period_steps_(python_round_steps(config_.period_s / dt_s_)),
      policy_(config_) {
    if (config_.enabled &&
        (period_steps_ < 1 ||
         std::abs(static_cast<double>(period_steps_) * dt_s_ -
                  config_.period_s) > 1e-9))
        throw std::invalid_argument(
            "rider intention period must be an integer multiple of timestep");
    reset();
}

void RiderIntent::reset() {
    policy_.reset();
    last_tick_step_ = -1;
    intent_ = {};
}

void RiderIntent::schedule_pulse(double start_s, double duration_s,
                                 double amplitude_rad) {
    policy_.program().schedule_pulse(start_s, duration_s, amplitude_rad);
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters,misc-no-recursion) fixed Python
// call signature; the probe path recurses once by value like
// copy.deepcopy(self).resolve(...).
IntentControl
RiderIntent::resolve(IntentControl control, const IntentSignals &signals,
                     std::int64_t step, double road_grade,
                     std::optional<double> preview_grade,
                     std::optional<double> lean_limit_rad, bool active,
                     bool advance) {
    // The wire decode already guaranteed a well-formed control dict — the
    // oracle's isinstance check maps to that boundary.
    if (!config_.enabled || !active)
        return control;
    if (step < 0)
        throw std::invalid_argument(
            "rider clock requires a nonnegative physics step");
    if (step < last_tick_step_)
        throw std::invalid_argument(
            "reset rider intention before rewinding physics");
    if (!advance) {
        RiderIntent probe(*this);
        return probe.resolve(control, signals, step, road_grade,
                             preview_grade, lean_limit_rad, true, true);
    }
    if (step % period_steps_ == 0 && step != last_tick_step_) {
        // Python's expected = last+period uses unbounded ints; the
        // difference form cannot overflow since 0 <= last_tick <= step.
        const bool on_schedule =
            last_tick_step_ < 0 ? step == 0
                                : step - last_tick_step_ == period_steps_;
        if (!on_schedule)
            throw std::invalid_argument(
                "rider intention clock skipped an acquisition");
        intent_ = policy_.update(signals, config_.period_s, road_grade,
                                 preview_grade, lean_limit_rad);
        last_tick_step_ = step;
    }
    if (!control.rider_enabled)
        return control;
    if (!control.posture.has_value())
        control.posture = intent_.posture;
    if (!control.human_torque_nm.has_value())
        control.human_torque_nm = intent_.effort_ceiling_nm;
    return control;
}
// NOLINTEND(bugprone-easily-swappable-parameters,misc-no-recursion)

RiderIntent::State RiderIntent::state() const {
    return {.last_tick_step = last_tick_step_,
            .intent = intent_,
            .policy = policy_.state(),
            .program = policy_.program().state()};
}

void RiderIntent::restore(const State &state) {
    policy_.restore(state.policy);
    policy_.program().restore(state.program);
    last_tick_step_ = state.last_tick_step;
    intent_ = state.intent;
}

} // namespace rider
