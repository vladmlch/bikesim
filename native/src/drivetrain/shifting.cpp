#include "shifting.hpp"
#include <algorithm>
#include <limits>
namespace drivetrain {
void CadenceShifter::reset() {
    state_ = {}; state_.rear_teeth = gearing_.rear_teeth; state_.from_teeth = state_.rear_teeth;
}
void CadenceShifter::set_state(const ShiftingSnapshot& s) {
    if (s.rear_teeth < 3 || s.from_teeth < 3 || s.shift_count < 0) throw std::invalid_argument("shifter teeth/count");
    if (s.direction != "none" && s.direction != "up" && s.direction != "down") throw std::invalid_argument("direction");
    nonnegative(s.cooldown_s, "cooldown_s"); nonnegative(s.cut_remaining_s, "cut_remaining_s");
    finite_optional(s.cadence_ema, "cadence_ema"); finite_optional(s.required_ema, "required_ema");
    if (s.cadence_ema.has_value() != s.required_ema.has_value()) throw std::invalid_argument("shifter EMAs");
    state_ = s;
}
bool CadenceShifter::update(double cadence, double required, double dt, bool pedaling,
                            bool braking, bool contact, std::optional<double> slip) {
    finite(cadence, "shift cadence"); finite(required, "wheel-required shift cadence");
    positive(dt, "shift interval"); finite_optional(slip, "rear tire slip");
    auto next = state_;
    const auto finish = [&](bool shifted) { state_ = std::move(next); return shifted; };
    const double tau = config_.cadence_smoothing_tau_s;
    const double alpha = tau <= 0. ? 1. : std::min(1., dt / tau);
    if (!next.cadence_ema) { next.cadence_ema = cadence; next.required_ema = required; }
    else { *next.cadence_ema += alpha * (cadence - *next.cadence_ema);
           *next.required_ema += alpha * (required - *next.required_ema); }
    next.cooldown_s = std::max(0., next.cooldown_s - dt);
    next.cut_remaining_s = std::max(0., next.cut_remaining_s - dt);
    if (!config_.enabled || !pedaling || braking || !contact || next.cooldown_s > 1e-12 || std::min(cadence, required) < 0.) return finish(false);
    int selected = next.rear_teeth;
    std::string direction;
    if (*next.cadence_ema > config_.target_cadence_max_rpm) {
        for (int teeth : config_.cassette) if (teeth < next.rear_teeth && (selected == next.rear_teeth || teeth > selected)) selected = teeth;
        direction = "up";
    } else if (*next.cadence_ema < config_.target_cadence_min_rpm) {
        for (int teeth : config_.cassette) if (teeth > next.rear_teeth && (selected == next.rear_teeth || teeth < selected)) selected = teeth;
        direction = "down";
    } else return finish(false);
    if (selected == next.rear_teeth) return finish(false);
    if (slip && config_.upshift_slip_mode == "magnitude") slip = std::abs(*slip);
    if (direction == "up" && slip && *slip > config_.upshift_slip_limit_mps) return finish(false);
    const double landing = *next.required_ema * selected / next.rear_teeth;
    if ((direction == "up" && landing < config_.target_cadence_min_rpm) || (direction == "down" && landing > config_.target_cadence_max_rpm)) return finish(false);
    if (next.shift_count == std::numeric_limits<int>::max()) throw std::overflow_error("shift_count");
    next.from_teeth = next.rear_teeth; next.rear_teeth = selected;
    next.required_ema = landing; next.direction = direction;
    ++next.shift_count;
    next.cooldown_s = config_.shift_cooldown_s; next.cut_remaining_s = config_.shift_cut_duration_s;
    return finish(true);
}
}
