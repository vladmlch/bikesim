#include "pedaling.hpp"
#include "stall_detector.hpp"
#include "../engaged.hpp"
#include "../numeric_sincos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <utility>

namespace drivetrain {
    double human_crank_torque(double mean_nm, double phase_rad, double ripple) {
        finite(mean_nm, "mean human torque");
        finite(phase_rad, "crank phase");
        nonnegative(ripple, "pedal ripple");
        if (ripple >= 1.) throw std::invalid_argument("pedal ripple");
        return validation::derived(mean_nm * (1. - ripple * numeric::math_cos(validation::derived(2. * phase_rad, "human_crank_torque.phase"))), "human_crank_torque.torque");
    }

    void PedalingPolicy::set_state(PedalingSnapshot s) {
        finite_optional(s.target_phase_rad, "target_phase_rad");
        finite_optional(s.cadence_ema, "_cadence_ema");
        finite(s.target_rate_rad_s, "target_rate_rad_s");
        nonnegative(s.deceleration_rad_s2, "deceleration_rad_s2");
        nonnegative(s.effort, "_effort");
        if (s.coasting && !s.target_phase_rad) throw std::invalid_argument("coasting target_phase_rad");
        // The engaged manoeuvre sub-object gates at the same domain the
        // wire codec admits — set_state is reachable without the codec
        // (restore/commit), so it cannot delegate the checks.
        if (s.reposition) {
            const RepositionSnapshot &r = *s.reposition;
            finite(r.crank_rad, "_reposition.crank_rad");
            finite(r.wheel_rad, "_reposition.wheel_rad");
            finite_optional(r.stall_elapsed_s, "_reposition.stall_elapsed_s");
            if (r.stall_elapsed_s)
                nonnegative(*r.stall_elapsed_s, "_reposition.stall_elapsed_s");
            finite_optional(r.goal_rad, "_reposition.goal_rad");
            nonnegative(r.elapsed_s, "_reposition.elapsed_s");
            nonnegative(r.cooldown_s, "_reposition.cooldown_s");
            if (r.reason > 2)
                throw std::invalid_argument("_reposition.reason");
        }
        state_ = s;
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror pedaling.py
    PedalingState PedalingPolicy::update(double phase, double rate, double required, double effort,
                                         double dt, bool enabled, bool braking, bool reposition_request,
                                         std::optional<double> shaft_rad_s,
                                         std::optional<double> ceiling_nm,
                                         double torque_factor, bool cut_after_ceiling) {
    // NOLINTEND(bugprone-easily-swappable-parameters)
        finite(phase, "crank phase");
        finite(rate, "crank rate");
        finite(required, "required cadence");
        finite_optional(shaft_rad_s, "shaft rate");
        if (ceiling_nm)
            nonnegative(*ceiling_nm, "automatic rider effort ceiling");
        nonnegative(torque_factor, "shift torque factor");
        nonnegative(effort, "rider effort");
        positive(dt, "pedaling interval");
        // Signed wheel rate in crank-equivalent units: rollback is not progress.
        const double wheel_rad_s = required * 2. * std::numbers::pi / 60.;
        if (!enabled) {
            reset();
            // A request held across a disable/enable toggle stays consumed:
            // only a fresh falling/rising pair earns a manoeuvre (pedaling.py
            // sets _reposition_armed = not reposition after reset()).
            if (reposition_request) {
                state_.reposition.emplace();
                state_.reposition->armed = false;
            }
            return {
                .mode = PedalMode::disabled, .reason = CoastReason::disabled, .effort_nm = 0.,
                .required_cadence_rpm = std::max(required, 0.),
                .target_phase_rad = std::nullopt, .target_rate_rad_s = 0.
            };
        }
        // Crank reposition manoeuvre (c809f12a^ pedaling.py:116-163): a
        // rising request edge or the stall reflex backpedals to the nearest
        // lower power phase, reporting coasting with a reposition reason.
        const bool tracked = config_.reposition_on_stall || reposition_request ||
                             (state_.reposition && (state_.reposition->goal_rad || state_.reposition->cooldown_s > 0.));
        if (tracked && !state_.reposition) state_.reposition.emplace();
        if (state_.reposition) {
            auto& r = *state_.reposition;
            r.cooldown_s = std::max(0., r.cooldown_s - dt);
            if (braking) { r.goal_rad.reset(); r.reason = 0; r.cooldown_s = config_.reposition_cooldown_s; }
            bool request_edge = false;
            if (reposition_request) { request_edge = r.armed; r.armed = false; } else r.armed = true;
            const bool loaded = config_.reposition_on_stall && !braking && !r.goal_rad && r.cooldown_s <= 0. &&
                                effort >= config_.reposition_min_effort_nm;
            StallDetectorState detector{.crank_rad = r.crank_rad, .wheel_rad = r.wheel_rad, .elapsed_s = r.stall_elapsed_s};
            const bool reflex_edge = stall_update(detector,
                {.window_s = config_.reposition_stall_window_s,
                 .crank_progress_rad = config_.reposition_stall_progress_rad,
                 .wheel_progress_rad = config_.reposition_stall_progress_rad},
                {.crank_rad_s = rate, .wheel_rad_s = wheel_rad_s}, dt, loaded);
            r.crank_rad = detector.crank_rad; r.wheel_rad = detector.wheel_rad; r.stall_elapsed_s = detector.elapsed_s;
            for (const auto& [edge, why] : {std::pair{request_edge, std::uint8_t{1}}, std::pair{reflex_edge, std::uint8_t{2}}}) {
                if (braking || !edge || r.goal_rad || r.cooldown_s > 0.) continue;
                const double rel = phase - kPowerPhaseOffsetRad;
                double turn = std::fmod(rel, std::numbers::pi);
                if (turn < 0.) turn += std::numbers::pi;
                if (std::min(turn, std::numbers::pi - turn) <= config_.reposition_noop_rad) continue;
                r.goal_rad = phase - turn; r.reason = why; r.elapsed_s = 0.;
                state_.effort = 0.; state_.coasting = false;
                break;
            }
            if (r.goal_rad) {
                const double remaining = phase - *r.goal_rad;
                r.elapsed_s += dt;
                if (remaining <= config_.reposition_phase_tolerance_rad || r.elapsed_s > config_.reposition_timeout_s) {
                    r.goal_rad.reset(); r.reason = 0; r.cooldown_s = config_.reposition_cooldown_s;
                } else {
                    constexpr double kApproachTauS = .05;
                    const double target_rate = -std::min(config_.reposition_back_rate_rad_s, remaining / kApproachTauS);
                    return {.mode = PedalMode::coasting,
                            .reason = r.reason == 1 ? CoastReason::reposition_requested : CoastReason::reposition_stall,
                            .effort_nm = 0., .required_cadence_rpm = std::max(required, 0.),
                            .target_phase_rad = phase + target_rate * dt, .target_rate_rad_s = target_rate};
                }
            }
            if (!config_.reposition_on_stall && !r.goal_rad && r.cooldown_s <= 0. && r.armed) state_.reposition.reset();
        }
        // `required` stayed signed through the manoeuvre block so the stall
        // detector and the reposition return see the raw wheel-implied rate;
        // the pedaling domain below clamps it like pedaling.py did at entry.
        required = std::max(required, 0.);
        const double actual = validation::derived(std::abs(rate) * 60. / (2. * std::numbers::pi), "PedalingPolicy.cadence");
        const double cadence = std::max(actual, required);
        if (config_.coast_cadence_tau_s > 0. && state_.cadence_ema)
            engaged(state_.cadence_ema) = validation::derived(engaged(state_.cadence_ema) + std::min(1., dt / config_.coast_cadence_tau_s) * validation::derived(actual - engaged(state_.cadence_ema), "PedalingPolicy.cadence_delta"), "PedalingPolicy.cadence_ema");
        else state_.cadence_ema = actual;
        const double threshold = state_.coasting ? config_.resume_below_rpm : config_.coast_above_rpm;
        const bool excessive = config_.enabled && engaged(state_.cadence_ema) >= threshold;
        const CoastReason reason =
                braking    ? CoastReason::braking :
                effort == 0. ? CoastReason::no_effort :
                excessive  ? CoastReason::cadence :
                             CoastReason::none;
        if (reason == CoastReason::none) {
            const double previous = state_.effort;
            const auto ema = state_.cadence_ema;
            const auto reposition = state_.reposition;
            reset();
            state_.effort = previous;
            state_.cadence_ema = ema;
            state_.reposition = reposition;
            double target = effort;
            if (config_.mash_torque_nm > target && cadence < config_.mash_cadence_rpm)
                target = effort + (config_.mash_torque_nm - effort) * (1. - cadence / config_.mash_cadence_rpm);
            if (cut_after_ceiling) {
                // Launch part 3a (review I4): a rider-limited effort is
                // capped first and the shift cut applies once to the cap.
                if (ceiling_nm)
                    target = std::min(target, *ceiling_nm);
                target = validation::derived(target * torque_factor, "PedalingPolicy.target_effort");
            } else {
                target = validation::derived(target * torque_factor, "PedalingPolicy.target_effort");
                if (ceiling_nm)
                    target = std::min(target, *ceiling_nm);
            }
            const double slew = config_.effort_slew_nm_s;
            if (slew <= 0.) effort = target;
            else {
                const double delta = validation::derived(slew * dt, "PedalingPolicy.slew_delta");
                effort = std::max(validation::derived(previous - delta, "PedalingPolicy.slew_lower"), std::min(validation::derived(previous + delta, "PedalingPolicy.slew_upper"), target));
            }
            state_.effort = effort;
            // pedaling.py:110-120 — the limb-velocity wish is the lesser of
            // the wheel-implied cadence and the rider's own perceived
            // cadence: a wheel-slip burst inflates the freehub boundary for
            // tens of milliseconds and the legs are dragged by the crank
            // through the welds, never willed past the rider's cadence
            // sense, so the wish does not chase wheelspin. When the
            // wheel-implied rate falls (the engaged sweep decelerates the
            // crank) the wish tracks instantly, since the drivetrain
            // decelerates the legs whether they intend it or not.
            double wish = validation::derived(
                std::max(std::min(required, engaged(state_.cadence_ema)) * 2. * std::numbers::pi / 60., 0.),
                "PedalingPolicy.target_rate");
            // The wish is a ceiling the rider spins the crank up to, not a
            // target the crank is braked down to. While the one-way motor
            // clutch is open the shaft outruns the crank; a wish below the
            // shaft would hold the crank back and keep the legs unloaded,
            // so it floors at the shaft and the crank comes up until the
            // clutch re-engages. A rigid link reads shaft == crank and the
            // floor never engages; only ordinary pedaling gets it — the
            // coasting and crank-reposition targets keep their own rates.
            const double shaft = shaft_rad_s.value_or(rate);
            if (shaft > rate)
                wish = std::max(wish, shaft);
            return {
                .mode = PedalMode::pedaling, .reason = CoastReason::none, .effort_nm = effort, .required_cadence_rpm = required,
                .target_phase_rad = std::nullopt, .target_rate_rad_s = wish
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
            .mode = PedalMode::coasting, .reason = reason, .effort_nm = 0., .required_cadence_rpm = required,
            .target_phase_rad = state_.target_phase_rad, .target_rate_rad_s = state_.target_rate_rad_s
        };
    }
}
