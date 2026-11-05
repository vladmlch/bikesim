#pragma once

#include "motor.hpp"
#include "pedaling.hpp"
#include "shifting.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace drivetrain {
// The three-policy snapshot previously owned only by the Python adapter.
// It intentionally has no model, transmission or borrowed Python state.
struct PolicySnapshot {
    PedalingSnapshot pedaling;
    ShiftingSnapshot shifting;
    AssistSnapshot assist;
};

class PolicySet {
public:
    explicit PolicySet(DrivePolicyConfig source)
        : config(std::move(source)), pedaling(config.pedaling),
          shifting(config.gearing, config.shifting), assist(config.assist) {
        validate(config);
    }

    void reset() {
        pedaling.reset(); shifting.reset(); assist.reset();
    }
    [[nodiscard]] PolicySnapshot snapshot() const {
        return {.pedaling = pedaling.state(), .shifting = shifting.state(), .assist = assist.state()};
    }
    void restore(const PolicySnapshot& source) {
        auto next = *this;
        next.pedaling.set_state(source.pedaling);
        next.shifting.set_state(source.shifting);
        next.assist.set_state(source.assist);
        next.validate_state();
        *this = std::move(next);
    }

    // Shared with the binding's strict scalar reader: parse/validate detached
    // members in its original order, then run these same relational gates
    // immediately before publication. No floating arithmetic changes here.
    void validate_state() const {
        const auto& shift = shifting.state();
        const bool shifted = shift.shift_count > 0;
        if ((shift.direction != "none") != shifted ||
            (!shifted && (shift.cooldown_s != 0. || shift.cut_remaining_s != 0.)))
            throw std::invalid_argument("shift count/direction mismatch");
        const auto& cassette = config.shifting.cassette;
        if (config.shifting.enabled &&
            (!std::ranges::contains(cassette, shift.rear_teeth) ||
             !std::ranges::contains(cassette, shift.from_teeth)))
            throw std::invalid_argument("shifter teeth outside the cassette");
    }

    // Same typed ownership as the historical adapter. Calls keep using the
    // existing policy methods; this owner adds no second policy algorithm.
    DrivePolicyConfig config;
    PedalingPolicy pedaling;
    CadenceShifter shifting;
    AssistController assist;
};
} // namespace drivetrain
