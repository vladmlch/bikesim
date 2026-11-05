#include "motor.hpp"

#include "../numeric_numpy.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace drivetrain {
    namespace {
        double interp(double x, const std::vector<std::array<double, 2> > &rows) {
            if (x < rows.front()[0]) return rows.front()[1];
            if (x > rows.back()[0]) return rows.back()[1];
            const auto right = std::ranges::lower_bound(rows, x, {},
                                                        [](const auto &row) { return row[0]; });
            if (right == rows.end()) return rows.back()[1];
            if ((*right)[0] == x) return (*right)[1];
            const auto &left = *(right - 1);
            const double slope = ((*right)[1] - left[1]) / ((*right)[0] - left[0]);
            // np.interp's interior eval + right-endpoint retry under the
            // wheel's measured lowering (BIKE_NUMPY_INTERP_CONTRACTED —
            // contracted on the macOS wheel, separate on Linux).
            return numeric::np_interp_segment(slope, x, left[0], left[1],
                                              (*right)[0], (*right)[1]);
        }
    }

    void AssistController::set_state(AssistSnapshot s) {
        nonnegative(s.torque, "torque");
        nonnegative(s.last_gain, "last_gain");
        nonnegative(s.hold_power_w, "hold_power_w");
        nonnegative(s.release_hold_left_s, "release_hold_left_s");
        state_ = s;
    }

    std::pair<double, double> AssistController::ceiling(double rpm, double speed) const {
        finite(rpm, "shaft rpm");
        finite(speed, "speed");
        const double omega = validation::derived(rpm * 2. * std::numbers::pi / 60., "AssistController.shaft_speed");
        double limit = config_.max_torque;
        if (config_.torque_curve) limit = std::min(limit, interp(rpm, *config_.torque_curve));
        if (omega > 0.) limit = std::min(limit, config_.max_power / omega);
        const double taper = std::max(
            0., std::min(1., (config_.cutoff_mps - std::abs(speed)) / config_.taper_width_mps));
        return {limit * taper, taper};
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    double AssistController::step(double human, double rpm, double speed, bool braking, double dt,
                                  std::optional<double> request, std::optional<double> shaft) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        finite(human, "human torque");
        finite(rpm, "cadence");
        finite(speed, "road speed");
        positive(dt, "assist timestep");
        if (request) nonnegative(*request, "motor setpoint");
        finite_optional(shaft, "motor shaft rpm");
        if (braking) {
            reset();
            return 0.;
        }
        const double sensed = human > config_.engage_torque_nm ? human : 0.;
        double gain = config_.gain;
        if (config_.profile) {
            // The ctor resolved the closed ProfileMode — dispatch on it
            // rather than the wire label. A profile-less controller keeps
            // its open custom label and the configured scalar gain.
            gain = profile_gain(engaged(config_.profile), engaged(profile_mode_), sensed);
        }
        const auto [limit, taper] = ceiling(shaft.value_or(rpm), speed);
        const double target = std::min(validation::derived(gain * sensed * taper, "AssistController.target"), limit);
        const double omega = validation::derived(shaft.value_or(rpm) * 2. * std::numbers::pi / 60., "AssistController.shaft_speed");
        // P1 motor run-on: a drop of the rider-proportional target below the
        // delivered torque arms a hold on the delivered POWER (torque x shaft
        // speed). The window counts down from release_hold_s and is not
        // re-armed until the target rises again — hold_power_w > 0 with an
        // expired window is the spent marker. A full stop of pedalling is
        // the same case; braking resets everything before this point.
        if (target < state_.torque) {
            if (state_.hold_power_w <= 0. && omega > 0.) {
                state_.hold_power_w = validation::derived(state_.torque * omega, "AssistController.hold_power_w");
                state_.release_hold_left_s = config_.release_hold_s;
            }
        } else {
            state_.hold_power_w = 0.;
            state_.release_hold_left_s = 0.;
        }
        double candidate = validation::derived(state_.torque - std::expm1(-dt / config_.tau) * (target - state_.torque), "AssistController.filtered_torque");
        const double delta = validation::derived(config_.slew * dt, "AssistController.slew_delta");
        const double lower = validation::derived(state_.torque - delta, "AssistController.slew_lower");
        const double upper = validation::derived(state_.torque + delta, "AssistController.slew_upper");
        candidate = std::max(lower, std::min(upper, candidate));
        const double cap = request.value_or(std::numeric_limits<double>::infinity());
        double held = 0.;
        if (state_.release_hold_left_s > 0.) {
            state_.release_hold_left_s = std::max(0., state_.release_hold_left_s - dt);
            held = omega > 0. ? std::min(state_.hold_power_w / omega, limit) : limit;
        }
        const double torque = validation::derived(std::max(0., std::min({std::max(candidate, held), limit, cap})), "delivered assist torque");
        state_.last_gain = gain;
        state_.torque = torque;
        state_.pedaling = torque > 0.;
        return torque;
    }
}
