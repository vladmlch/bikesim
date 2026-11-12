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

} // namespace

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

// Launch part 1, task 4 — the rider command set's input contract. Brake
// demands are fractions of full travel, so the [0, 1] envelope is checked
// here exactly like the posture bounds above. Task 5 adds the ownership
// invariant: a shift request without owns_shifting would be silently
// dropped on the automatic path, so it is rejected at the boundary.
void validate(const RiderCommands &commands) {
    if (!std::isfinite(commands.front_brake) ||
        commands.front_brake < 0. || commands.front_brake > 1.)
        throw std::invalid_argument(
            "rider front_brake demand must lie in [0, 1]");
    if (!std::isfinite(commands.rear_brake) ||
        commands.rear_brake < 0. || commands.rear_brake > 1.)
        throw std::invalid_argument(
            "rider rear_brake demand must lie in [0, 1]");
    if (commands.shift_request && !commands.owns_shifting)
        throw std::invalid_argument(
            "rider shift request requires owns_shifting");
}

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
    : config_(config) {
    if (!std::isfinite(config_.min_elbow_flexion_deg) ||
        config_.min_elbow_flexion_deg <= 0. ||
        config_.min_elbow_flexion_deg > 90.) {
        throw std::invalid_argument(
            "min_elbow_flexion_deg must be finite and lie in (0, 90]");
    }
}

void SeatedPostureProgram::reset() {
    lean_rad_ = 0.;
    trim_rad_ = 0.;
    load_samples_.clear();
    delayed_load_share_.reset();
    pulses_.clear();
}

void SeatedPostureProgram::seed_lean(double lean_rad) {
    if (!std::isfinite(lean_rad) || lean_rad > config_.max_forward_lean_rad ||
        lean_rad < -config_.max_backward_lean_rad)
        throw std::invalid_argument("seeded lean must be finite and inside the seated posture bounds");
    lean_rad_ = lean_rad;
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

// NOLINTBEGIN(bugprone-easily-swappable-parameters) fixed positional Python signature
double SeatedPostureProgram::update(
    double time_s, double road_grade, double dt_s,
    std::optional<double> front_load_share,
    std::optional<double> lean_limit_rad,
    std::optional<double> dead_time_s,
    std::optional<double> min_lean_rad,
    double extra_target_rad) {
// NOLINTEND(bugprone-easily-swappable-parameters)
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
    if (min_lean_rad.has_value() && !std::isfinite(*min_lean_rad))
        throw std::invalid_argument("minimum lean limit must be finite");
    if (!(std::isfinite(extra_target_rad) && extra_target_rad >= 0.))
        throw std::invalid_argument("posture extra target must be finite and nonnegative");
    load_samples_.emplace_back(time_s, front_load_share);
    while (!load_samples_.empty() &&
           load_samples_.front().first <= time_s - delay + 1e-12) {
        delayed_load_share_ = load_samples_.front().second;
        load_samples_.pop_front();
    }
    const double forward_limit =
        lean_limit_rad.value_or(config_.max_forward_lean_rad);
    const double backward_limit =
        min_lean_rad.has_value()
            ? std::min(forward_limit,
                       std::max(-config_.max_backward_lean_rad, *min_lean_rad))
            : -config_.max_backward_lean_rad;
    double extra = 0.;
    for (const auto &[start, duration, amplitude] : pulses_)
        extra += body_pulse(time_s, start, duration, amplitude);
    // Launch part 3a: the reaction's forward lean joins the target and
    // obeys the program's rate and bounds; zero leaves the arithmetic
    // unchanged.
    if (extra_target_rad != 0.) extra += extra_target_rad;
    const double grade_and_pulse =
        config_.lean_gain * std::atan(road_grade) + extra;
    if (delayed_load_share_.has_value()) {
        trim_rad_ += config_.lean_trim_gain_rad_s *
                     (config_.front_load_share_target -
                      *delayed_load_share_) *
                     dt_s;
        const double min_trim =
            min_lean_rad.has_value()
                ? std::max(-config_.lean_trim_limit_rad,
                           backward_limit - grade_and_pulse)
                : -config_.lean_trim_limit_rad;
        trim_rad_ = std::max(min_trim,
                             std::min(config_.lean_trim_limit_rad,
                                      trim_rad_));
    }
    const double target = std::max(
        backward_limit,
        std::min(forward_limit, grade_and_pulse + trim_rad_));
    lean_rad_ += std::max(-config_.lean_rate_rad_s * dt_s,
                          std::min(config_.lean_rate_rad_s * dt_s,
                                   target - lean_rad_));
    if (min_lean_rad.has_value() && lean_rad_ < backward_limit)
        lean_rad_ = backward_limit;
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
    if (!std::isfinite(config_.min_elbow_flexion_deg) ||
        config_.min_elbow_flexion_deg <= 0. ||
        config_.min_elbow_flexion_deg > 90.) {
        throw std::invalid_argument(
            "min_elbow_flexion_deg must be finite and lie in (0, 90]");
    }
    // Required launch fields carry the -1 sentinel without a profile load:
    // reject them here the same bounds the wire decode enforces, so an
    // unset config fails loudly instead of arming on every grade.
    if (!std::isfinite(config_.launch_effort_ramp_s) ||
        config_.launch_effort_ramp_s <= 0.) {
        throw std::invalid_argument(
            "launch_effort_ramp_s must be finite and positive");
    }
    if (!std::isfinite(config_.launch_min_grade) ||
        config_.launch_min_grade < 0.) {
        throw std::invalid_argument(
            "launch_min_grade must be finite and nonnegative");
    }
    if (!std::isfinite(config_.launch_standstill_crank_rate_rad_s) ||
        config_.launch_standstill_crank_rate_rad_s < 0.) {
        throw std::invalid_argument(
            "launch_standstill_crank_rate_rad_s must be finite and "
            "nonnegative");
    }
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
    launch_ramp_s_ = -1.;
    launch_decided_ = false;
    program_.reset();
}

void SeatedClimbPolicy::seed_lean(double lean_rad) {
    program_.seed_lean(lean_rad);
    lean_rad_ = lean_rad;
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
                                 std::optional<double> lean_limit_rad,
                                 std::optional<double> min_lean_rad,
                                 double extra_lean_rad) {
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
        effort_nm_ = target_effort;
        // Uphill standstill launch (2026-11-10 experiment, user decision):
        // a real rider does not mash full effort at once on a grade. The
        // decision is taken once, at the first tick the delayed signal
        // exists (the rider's reaction delay): preview grade at or above
        // launch_min_grade while the delayed crank rate is still at
        // standstill arms the ramp; while armed its clock advances by dt.
        if (!launch_decided_) {
            launch_decided_ = true;
            if (preview >= config_.launch_min_grade &&
                delayed.crank_rate_rad_s <=
                    config_.launch_standstill_crank_rate_rad_s)
                launch_ramp_s_ = 0.;
        }
        if (launch_ramp_s_ >= 0.) launch_ramp_s_ += dt;
    }
    lean_rad_ = program_.update(time_s_, preview, dt,
                                signals.front_load_share, lean_limit_rad,
                                std::nullopt, min_lean_rad, extra_lean_rad);
    time_s_ += dt;
    RiderPosture posture;
    posture.torso_lean_rad = lean_rad_;
    posture.use_saddle = true;
    validate_posture(posture);
    // Armed launch: the effort ceiling rises linearly from 0 to full over
    // launch_effort_ramp_s; unarmed (sentinel -1) the factor is 1 exactly,
    // so flat and rolling starts keep the pre-mechanism values bitwise.
    const double launch_factor =
        launch_ramp_s_ < 0.
            ? 1.
            : std::min(1., launch_ramp_s_ / config_.launch_effort_ramp_s);
    return {.posture = posture,
            .effort_ceiling_nm = effort_nm_ * launch_factor};
}

SeatedClimbPolicy::State SeatedClimbPolicy::state() const {
    return {.time_s = time_s_,
            .inclination_rad = inclination_rad_,
            .lean_rad = lean_rad_,
            .effort_nm = effort_nm_,
            .samples = samples_,
            .delayed = delayed_,
            .surge_budget_s_left = surge_budget_s_left_,
            .launch_ramp_s = launch_ramp_s_,
            .launch_decided = launch_decided_};
}

void SeatedClimbPolicy::restore(const State &state) {
    time_s_ = state.time_s;
    inclination_rad_ = state.inclination_rad;
    lean_rad_ = state.lean_rad;
    effort_nm_ = state.effort_nm;
    samples_ = state.samples;
    delayed_ = state.delayed;
    surge_budget_s_left_ = state.surge_budget_s_left;
    launch_ramp_s_ = state.launch_ramp_s;
    launch_decided_ = state.launch_decided;
}

// ---- RiderIntentResolver (rider_intent.py) --------------------------------

RiderIntent::RiderIntent(IntentConfig config, FrontLiftReactionConfig reaction, double dt_s)
    : config_(config),
      dt_s_(validation::positive(dt_s, "rider physics timestep")),
      period_steps_(python_round_steps(config_.period_s / dt_s_)),
      core_(config_),
      staged_(config_) {
    if (!std::isfinite(config_.min_elbow_flexion_deg) ||
        config_.min_elbow_flexion_deg <= 0. ||
        config_.min_elbow_flexion_deg > 90.) {
        throw std::invalid_argument(
            "min_elbow_flexion_deg must be finite and lie in (0, 90]");
    }
    if (config_.enabled) {
        if (period_steps_ < 1 ||
            std::abs(static_cast<double>(period_steps_) * dt_s_ -
                     config_.period_s) > 1e-9)
            throw std::invalid_argument(
                "rider intention period must be an integer multiple of "
                "timestep");
        const double delay_ticks =
            std::rint(config_.reaction_delay_s / config_.period_s);
        if (delay_ticks < 0. ||
            std::abs(delay_ticks * config_.period_s -
                     config_.reaction_delay_s) > 1e-9)
            throw std::invalid_argument(
                "rider intention reaction delay must be an integer multiple "
                "of the intention period");
        if (!(config_.orientation_tau_s >= 2. * config_.period_s))
            throw std::invalid_argument(
                "rider intention orientation smoothing must span at least "
                "two intention periods");
        // Launch part 3a: the runtime document never passes post_init, so
        // the reaction's period and posture context is checked here too.
        validate_reaction_context(reaction, config_.period_s, config_.max_forward_lean_rad);
        core_.perception.emplace(config_.reaction_delay_s, config_.period_s);
        core_.reaction.emplace(reaction, config_.period_s);
    }
    staged_ = core_;
    reset();
}

void RiderIntent::reset() {
    core_.policy.reset();
    if (core_.perception) core_.perception->reset();
    if (core_.reaction) core_.reaction->reset();
    core_.last_tick_step = -1;
    core_.intent = {};
}

void RiderIntent::seed_lean(double lean_rad) {
    core_.policy.seed_lean(lean_rad);
    core_.intent.posture.torso_lean_rad = lean_rad;
}

void RiderIntent::schedule_pulse(double start_s, double duration_s,
                                 double amplitude_rad) {
    core_.policy.program().schedule_pulse(start_s, duration_s, amplitude_rad);
}

void RiderIntent::tick(Core &core, const IntentSignals &signals,
                       const RiderSenses &senses, std::int64_t step,
                       double road_grade,
                       std::optional<double> preview_grade,
                       std::optional<double> lean_limit_rad,
                       std::optional<double> min_lean_rad) const {
    // Python's expected = last+period uses unbounded ints; the difference
    // form cannot overflow since 0 <= last_tick <= step.
    const bool on_schedule =
        core.last_tick_step < 0 ? step == 0 : step - core.last_tick_step == period_steps_;
    if (!on_schedule)
        throw std::invalid_argument("rider intention clock skipped an acquisition");
    // Launch part 3a: one acquisition per intention tick — the delayed
    // percepts move the front-lift reaction, whose lean add joins the
    // policy's posture target. Perception and reaction are engaged: a
    // disabled intent never reaches a tick.
    const SenseSample sample{.signals = signals, .senses = senses};
    const double lean_add = core.reaction->update(core.perception->update(sample)).lean_add_rad;
    core.intent = core.policy.update(signals, config_.period_s, road_grade, preview_grade, lean_limit_rad,
                                     min_lean_rad, lean_add);
    core.last_tick_step = step;
}

IntentResolution RiderIntent::output(const Core &core, IntentControl control) {
    if (!control.rider_enabled)
        return {.control = control, .effort_limited = false};
    if (!control.posture.has_value())
        control.posture = core.intent.posture;
    bool limited = false;
    if (!control.human_torque_nm.has_value()) {
        control.human_torque_nm = core.intent.effort_ceiling_nm;
        // The rider eases the ceiling while reacting; a factor of one is
        // not applied at all (bitwise unchanged).
        const double factor = core.reaction ? core.reaction->command().effort_factor : 1.;
        if (factor != 1.) {
            control.human_torque_nm = core.intent.effort_ceiling_nm * factor;
            limited = true;
        }
    }
    return {.control = control, .effort_limited = limited};
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters) fixed Python call signature
IntentResolution RiderIntent::resolve_full(IntentControl control, const IntentSignals &signals,
                                           const RiderSenses &senses, std::int64_t step, double road_grade,
                                           std::optional<double> preview_grade,
                                           std::optional<double> lean_limit_rad, bool active, bool advance,
                                           std::optional<double> min_lean_rad) {
    // The wire decode already guaranteed a well-formed control dict — the
    // oracle's isinstance check maps to that boundary.
    if (!config_.enabled || !active)
        return {.control = control, .effort_limited = false};
    if (step < 0)
        throw std::invalid_argument("rider clock requires a nonnegative physics step");
    if (step < core_.last_tick_step)
        throw std::invalid_argument("reset rider intention before rewinding physics");
    const bool due = step % period_steps_ == 0 && step != core_.last_tick_step;
    if (!advance) {
        // The probe runs the tick on a local copy, like
        // copy.deepcopy(self).resolve(...): core_ and staged_ stay as they are.
        if (!due) return output(core_, control);
        Core probe = core_;
        tick(probe, signals, senses, step, road_grade, preview_grade, lean_limit_rad, min_lean_rad);
        return output(probe, control);
    }
    if (due) {
        // Decision 11: stage, then commit by a nonthrowing swap — a rejected
        // input, a rejected posture or a failed allocation leaves core_ as
        // it was, and a retry is exactly one tick.
        staged_ = core_;
        tick(staged_, signals, senses, step, road_grade, preview_grade, lean_limit_rad, min_lean_rad);
        std::swap(core_, staged_);
    }
    return output(core_, control);
}
// NOLINTEND(bugprone-easily-swappable-parameters)

IntentControl RiderIntent::resolve(IntentControl control, const IntentSignals &signals,
                                   const RiderSenses &senses, std::int64_t step, double road_grade,
                                   std::optional<double> preview_grade,
                                   std::optional<double> lean_limit_rad, bool active, bool advance,
                                   std::optional<double> min_lean_rad) {
    return resolve_full(control, signals, senses, step, road_grade, preview_grade, lean_limit_rad,
                        active, advance, min_lean_rad).control;
}

RiderIntent::State RiderIntent::state() const {
    return {.last_tick_step = core_.last_tick_step,
            .intent = core_.intent,
            .policy = core_.policy.state(),
            .program = core_.policy.program().state(),
            .perception = core_.perception ? std::optional{core_.perception->state()} : std::nullopt,
            .reaction = core_.reaction ? std::optional{core_.reaction->state()} : std::nullopt};
}

void RiderIntent::restore(const State &state) {
    if (state.perception.has_value() != core_.perception.has_value())
        throw std::invalid_argument("rider intent perception state presence must match the enabled intent");
    if (state.reaction.has_value() != core_.reaction.has_value())
        throw std::invalid_argument("rider intent reaction state presence must match the enabled intent");
    if (core_.perception) {
        // One acquisition per intention tick, the first at step 0, so the
        // perception clock is the tick count; the reaction has seen exactly
        // the percepts delivered so far (saturating at its cap).
        if (state.last_tick_step < -1 ||
            (state.last_tick_step >= 0 && (state.last_tick_step % period_steps_ != 0 ||
                                           state.last_tick_step / period_steps_ ==
                                               std::numeric_limits<std::int64_t>::max())))
            throw std::invalid_argument("rider intent last_tick_step is off the intention grid");
        const std::int64_t ticks = state.last_tick_step < 0 ? 0 : state.last_tick_step / period_steps_ + 1;
        if (state.perception->tick != ticks)
            throw std::invalid_argument("rider intent perception tick must equal the intention tick count");
        const std::int64_t delivered = std::max<std::int64_t>(0, ticks - core_.perception->delay_ticks());
        if (state.reaction->percepts_seen != std::min(delivered, core_.reaction->percepts_cap()))
            throw std::invalid_argument("rider intent reaction percepts_seen must equal the delivered percepts");
    }
    Core next = core_;   // staged: every component restores into the copy first
    next.policy.restore(state.policy);
    next.policy.program().restore(state.program);
    if (next.perception) next.perception->restore(*state.perception);
    if (next.reaction) next.reaction->restore(*state.reaction);
    next.last_tick_step = state.last_tick_step;
    next.intent = state.intent;
    std::swap(core_, next);
}

ReactionCommand RiderIntent::reaction() const {
    return core_.reaction ? core_.reaction->command() : ReactionCommand{};
}

} // namespace rider
