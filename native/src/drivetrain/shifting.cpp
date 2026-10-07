#include "shifting.hpp"
#include "../engaged.hpp"
#include <algorithm>
#include <limits>

namespace drivetrain {
    void CadenceShifter::set_state(ShiftingSnapshot s) {
        if (s.rear_teeth < 3 || s.from_teeth < 3 || s.shift_count < 0) throw std::invalid_argument(
            "shifter teeth/count");
        if (!shift_direction(s.direction)) throw std::invalid_argument("direction");
        nonnegative(s.cooldown_s, "cooldown_s");
        nonnegative(s.cut_remaining_s, "cut_remaining_s");
        finite_optional(s.cadence_ema, "cadence_ema");
        finite_optional(s.required_ema, "required_ema");
        if (s.cadence_ema.has_value() != s.required_ema.has_value()) throw std::invalid_argument("shifter EMAs");
        state_ = std::move(s);
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror shifting.py
    bool CadenceShifter::update(double cadence, double required, double dt, bool pedaling,
                                bool braking, bool contact, std::optional<double> slip) {
        finite(cadence, "shift cadence");
        finite(required, "wheel-required shift cadence");
        positive(dt, "shift interval");
        finite_optional(slip, "rear tire slip");
        auto next = state_;
        const auto finish = [&](bool shifted) {
            state_ = std::move(next);
            return shifted;
        };
        const double tau = config_.cadence_smoothing_tau_s;
        const double alpha = tau <= 0. ? 1. : std::min(1., dt / tau);
        if (!next.cadence_ema) {
            next.cadence_ema = cadence;
            next.required_ema = required;
        } else {
            engaged(next.cadence_ema) = validation::derived(engaged(next.cadence_ema) + alpha * validation::derived(cadence - engaged(next.cadence_ema), "CadenceShifter.cadence_delta"), "CadenceShifter.cadence_ema");
            engaged(next.required_ema) = validation::derived(engaged(next.required_ema) + alpha * validation::derived(required - engaged(next.required_ema), "CadenceShifter.required_delta"), "CadenceShifter.required_ema");
        }
        next.cooldown_s = std::max(0., next.cooldown_s - dt);
        next.cut_remaining_s = std::max(0., next.cut_remaining_s - dt);
        if (!config_.enabled || !pedaling || braking || !contact || next.cooldown_s > 1e-12 ||
            std::min(cadence, required) < 0.) return finish(false);
        int selected = next.rear_teeth;
        ShiftDirection direction = ShiftDirection::none;
        if (engaged(next.cadence_ema) > config_.target_cadence_max_rpm) {
            for (int const teeth: config_.cassette) if (
                teeth < next.rear_teeth && (selected == next.rear_teeth || teeth > selected)) selected = teeth;
            direction = ShiftDirection::up;
        } else if (engaged(next.cadence_ema) < config_.target_cadence_min_rpm) {
            for (int const teeth: config_.cassette) if (
                teeth > next.rear_teeth && (selected == next.rear_teeth || teeth < selected)) selected = teeth;
            direction = ShiftDirection::down;
        } else return finish(false);
        if (selected == next.rear_teeth) return finish(false);
        // The slip gate is mode-dependent: "magnitude" compares unsigned
        // slip, "legacy_signed" keeps the signed value — resolved once at
        // construction into slip_mode_.
        if (slip && slip_mode_ == SlipMode::magnitude) slip = std::abs(*slip);
        if (direction == ShiftDirection::up && slip && *slip > config_.upshift_slip_limit_mps) return finish(false);
        const double landing = validation::derived(engaged(next.required_ema) * selected / next.rear_teeth, "CadenceShifter.landing_cadence");
        if ((direction == ShiftDirection::up && landing < config_.target_cadence_min_rpm) || (
                direction == ShiftDirection::down && landing > config_.target_cadence_max_rpm)) return finish(false);
        if (next.shift_count == std::numeric_limits<int>::max()) throw std::overflow_error("shift_count");
        next.from_teeth = next.rear_teeth;
        next.rear_teeth = selected;
        next.required_ema = landing;
        next.direction = shift_direction_name(direction);
        ++next.shift_count;
        next.cooldown_s = config_.shift_cooldown_s;
        next.cut_remaining_s = config_.shift_cut_duration_s;
        return finish(true);
    }
}
