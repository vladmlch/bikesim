#include "motor.hpp"
#include "../writers/pyfloat.hpp"
#include <algorithm>
#include <limits>
#include <numbers>
namespace drivetrain {
namespace {
double interp(double x, const std::vector<std::array<double, 2>>& rows) {
    if (x < rows.front()[0]) return rows.front()[1];
    if (x > rows.back()[0]) return rows.back()[1];
    const auto right = std::lower_bound(rows.begin(), rows.end(), x,
        [](const auto& row, double value) { return row[0] < value; });
    if (right == rows.end()) return rows.back()[1];
    if ((*right)[0] == x) return (*right)[1];
    const auto& left = *(right - 1);
    const double slope = ((*right)[1] - left[1]) / ((*right)[0] - left[0]);
    // This NumPy wheel contracts interpolation multiply/add on arm64.
    double result = std::fma(slope, x - left[0], left[1]);
    // NumPy interp retries from the right endpoint after overflow cancellation.
    if (std::isnan(result)) {
        result = std::fma(slope, x - (*right)[0], (*right)[1]);
        if (std::isnan(result) && left[1] == (*right)[1]) result = left[1];
    }
    return result;
}
}
void AssistController::set_state(const AssistSnapshot& s) {
    nonnegative(s.torque, "torque"); nonnegative(s.last_gain, "last_gain"); state_ = s;
}
std::pair<double, double> AssistController::ceiling(double rpm, double speed) const {
    finite(rpm, "shaft rpm"); finite(speed, "speed");
    const double omega = rpm * 2. * std::numbers::pi / 60.;
    double limit = config_.max_torque;
    if (config_.torque_curve) limit = std::min(limit, interp(rpm, *config_.torque_curve));
    if (omega > 0.) limit = std::min(limit, config_.max_power / omega);
    const double taper = std::max(0., std::min(1., (config_.cutoff_mps - std::abs(speed)) / config_.taper_width_mps));
    return {limit * taper, taper};
}
double AssistController::step(double human, double rpm, double speed, bool braking, double dt,
                               std::optional<double> request, std::optional<double> shaft) {
    finite(human, "human torque"); finite(rpm, "cadence"); finite(speed, "road speed");
    positive(dt, "assist timestep");
    if (request) nonnegative(*request, "motor setpoint");
    finite_optional(shaft, "motor shaft rpm");
    if (braking) { reset(); return 0.; }
    const double sensed = human > config_.engage_torque_nm ? human : 0.;
    double gain = config_.gain;
    if (config_.profile) {
        const auto& p = *config_.profile;
        if (config_.mode == "eco") gain = p.eco;
        else if (config_.mode == "tour") gain = p.tour;
        else if (config_.mode == "turbo") gain = p.turbo;
        else {
            const double fraction = std::min(1., std::max(0., sensed) / p.emtb_full_gain_at_nm);
            gain = p.emtb_low + (p.emtb_high - p.emtb_low) * fraction;
        }
    }
    state_.last_gain = gain;
    const auto [limit, taper] = ceiling(shaft.value_or(rpm), speed);
    const double target = std::min(gain * sensed * taper, limit);
    double candidate = state_.torque - std::expm1(-dt / config_.tau) * (target - state_.torque);
    candidate = std::max(state_.torque - config_.slew * dt, std::min(state_.torque + config_.slew * dt, candidate));
    const double crank = finite(rpm * 2. * std::numbers::pi / 60., "crank rate");
    const double cap = crank <= config_.gate_min_crank_rad_s || sensed <= 0. ? 0. : request.value_or(std::numeric_limits<double>::infinity());
    const double torque = finite(std::max(0., std::min({candidate, limit, cap})), "delivered assist torque");
    state_.torque = torque; state_.pedaling = torque > 0.;
    return torque;
}
void Battery::set_state(const BatterySnapshot& s) {
    nonnegative(s.initial_energy_j, "initial_energy_j"); nonnegative(s.energy_j, "energy_j");
    nonnegative(s.drawn_energy_j, "drawn_energy_j"); state_ = s;
}
double Battery::draw(double power, double dt) {
    nonnegative(power, "battery power"); positive(dt, "battery timestep");
    const double requested = nonnegative(power * dt, "requested battery energy");
    const double delivered = std::min(state_.energy_j, requested);
    state_.energy_j = std::max(0., state_.energy_j - delivered);
    state_.drawn_energy_j += delivered;
    return delivered / dt;
}
double motor_electrical_power(double torque, double omega, double a, double b, double idle, bool enabled) {
    finite(torque, "shaft torque"); finite(omega, "shaft speed");
    nonnegative(a, "copper coefficient"); nonnegative(b, "speed loss coefficient"); nonnegative(idle, "idle loss");
    if (!enabled) return 0.;
    return finite(std::max(torque * omega, 0.) + a * pyfloat::pow(torque, 2.) + b * pyfloat::pow(omega, 2.) + idle, "electrical power");
}
double limit_torque_by_energy(double request, double omega, double a, double b, double idle, double budget) {
    nonnegative(request, "requested motor torque"); finite(omega, "shaft speed");
    nonnegative(a, "copper coefficient"); nonnegative(b, "speed loss coefficient");
    nonnegative(idle, "idle loss"); nonnegative(budget, "power budget");
    const double overhead = finite(b * pyfloat::pow(omega, 2.) + idle, "motor overhead");
    const double available = budget - overhead;
    if (available <= 0.) return 0.;
    const double w = std::max(omega, 0.);
    double cap;
    if (a > 0.) {
        const double discriminant = finite(w * w + 4. * a * available, "torque budget discriminant");
        cap = 2. * available / (w + std::sqrt(discriminant));
    } else if (w > 0.) cap = available / w;
    else cap = request;
    return std::min(request, cap);
}
}
