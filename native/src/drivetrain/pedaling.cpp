#include "pedaling.hpp"
#include "../engaged.hpp"
#include <algorithm>
#include <numbers>

namespace drivetrain {
    double human_crank_torque(double mean_nm, double phase_rad, double ripple) {
        finite(mean_nm, "mean human torque");
        finite(phase_rad, "crank phase");
        nonnegative(ripple, "pedal ripple");
        if (ripple >= 1.) throw std::invalid_argument("pedal ripple");
        return validation::derived(mean_nm * (1. - ripple * std::cos(validation::derived(2. * phase_rad, "human_crank_torque.phase"))), "human_crank_torque.torque");
    }

    void PedalingPolicy::set_state(const PedalingSnapshot &s) {
        finite_optional(s.target_phase_rad, "target_phase_rad");
        finite_optional(s.cadence_ema, "_cadence_ema");
        finite(s.target_rate_rad_s, "target_rate_rad_s");
        nonnegative(s.deceleration_rad_s2, "deceleration_rad_s2");
        nonnegative(s.effort, "_effort");
        if (s.coasting && !s.target_phase_rad) throw std::invalid_argument("coasting target_phase_rad");
        state_ = s;
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror pedaling.py
    PedalingState PedalingPolicy::update(double phase, double rate, double required, double effort,
                                         double dt, bool enabled, bool braking) {
        finite(phase, "crank phase");
        finite(rate, "crank rate");
        finite(required, "required cadence");
        nonnegative(effort, "rider effort");
        positive(dt, "pedaling interval");
        const double wheel = validation::derived(required * 2. * std::numbers::pi / 60., "PedalingPolicy.wheel_rate");
        required = std::max(required, 0.);
        if (!enabled) {
            reset();
            return {
                .mode = "disabled", .reason = "disabled", .effort_nm = 0., .required_cadence_rpm = required,
                .target_phase_rad = std::nullopt, .target_rate_rad_s = 0.
            };
        }
        const double actual = validation::derived(std::abs(rate) * 60. / (2. * std::numbers::pi), "PedalingPolicy.cadence");
        const double cadence = std::max(actual, required);
        if (config_.coast_cadence_tau_s > 0. && state_.cadence_ema)
            *state_.cadence_ema = validation::derived(*state_.cadence_ema + std::min(1., dt / config_.coast_cadence_tau_s) * validation::derived(actual - *state_.cadence_ema, "PedalingPolicy.cadence_delta"), "PedalingPolicy.cadence_ema");
        else state_.cadence_ema = actual;
        const double threshold = state_.coasting ? config_.resume_below_rpm : config_.coast_above_rpm;
        const bool excessive = config_.enabled && *state_.cadence_ema >= threshold;
        const std::string reason = braking ? "braking" : effort == 0. ? "no_effort" : excessive ? "cadence" : "";
        if (reason.empty()) {
            const double previous = state_.effort;
            const auto ema = state_.cadence_ema;
            reset();
            state_.effort = previous;
            state_.cadence_ema = ema;
            double target = effort;
            if (config_.mash_torque_nm > target && cadence < config_.mash_cadence_rpm)
                target = effort + (config_.mash_torque_nm - effort) * (1. - cadence / config_.mash_cadence_rpm);
            const double slew = config_.effort_slew_nm_s;
            if (slew <= 0.) effort = target;
            else {
                const double delta = validation::derived(slew * dt, "PedalingPolicy.slew_delta");
                effort = std::max(validation::derived(previous - delta, "PedalingPolicy.slew_lower"), std::min(validation::derived(previous + delta, "PedalingPolicy.slew_upper"), target));
            }
            state_.effort = effort;
            return {
                .mode = "pedaling", .reason = "", .effort_nm = effort, .required_cadence_rpm = required,
                .target_phase_rad = std::nullopt, .target_rate_rad_s = std::max(wheel, 0.)
            };
        }
        if (!state_.coasting) {
            const double deceleration = validation::derived(std::abs(rate) / config_.stop_time_s, "PedalingPolicy.deceleration");
            state_.target_phase_rad = phase;
            state_.target_rate_rad_s = rate;
            state_.deceleration_rad_s2 = deceleration;
        }
        state_.coasting = true;
        const double previous = state_.target_rate_rad_s;
        const double next = std::max(0., validation::derived(std::abs(previous) - validation::derived(state_.deceleration_rad_s2 * dt, "PedalingPolicy.rate_delta"), "PedalingPolicy.next_speed"));
        state_.target_rate_rad_s = std::copysign(next, previous);
        engaged(state_.target_phase_rad) = validation::derived(engaged(state_.target_phase_rad) + .5 * validation::derived(previous + state_.target_rate_rad_s, "PedalingPolicy.rate_sum") * dt, "PedalingPolicy.target_phase_rad");
        state_.effort = 0.;
        return {
            .mode = "coasting", .reason = reason, .effort_nm = 0., .required_cadence_rpm = required,
            .target_phase_rad = state_.target_phase_rad, .target_rate_rad_s = state_.target_rate_rad_s
        };
    }
}
