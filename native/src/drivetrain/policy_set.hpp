#pragma once

#include "freehub.hpp"
#include "motor.hpp"
#include "pedaling.hpp"
#include "shifting.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace drivetrain {
// The five-policy snapshot previously owned only by the Python adapter.
// It intentionally has no model, transmission or borrowed Python state.
struct PolicySnapshot {
    PedalingSnapshot pedaling;
    ShiftingSnapshot shifting;
    AssistSnapshot assist;
    BatterySnapshot battery;
    FreehubSnapshot hub;
};

class PolicySet {
public:
    explicit PolicySet(DrivePolicyConfig source)
        : config(std::move(source)), pedaling(config.pedaling),
          shifting(config.gearing, config.shifting), assist(config.assist),
          battery(config.battery.energy_j),
          hub(config.hub_stiffness_nm_rad, config.hub_damping_nm_s) {
        validate(config);
    }

    void reset() {
        pedaling.reset(); shifting.reset(); assist.reset(); battery.reset(); hub.reset();
    }
    [[nodiscard]] PolicySnapshot snapshot() const {
        return {.pedaling=pedaling.state(), .shifting=shifting.state(), .assist=assist.state(), .battery=battery.state(), .hub=hub.state()};
    }
    void restore(const PolicySnapshot& source) {
        auto next = *this;
        next.pedaling.set_state(source.pedaling);
        next.shifting.set_state(source.shifting);
        next.assist.set_state(source.assist);
        next.battery.set_state(source.battery);
        next.hub.set_state(source.hub);
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
        const auto& stored = battery.state();
        if (stored.initial_energy_j != config.battery.energy_j ||
            stored.energy_j > stored.initial_energy_j || stored.drawn_energy_j > stored.initial_energy_j)
            throw std::invalid_argument("battery energy outside the configured store");
    }

    // Same typed ownership as the historical adapter. Calls keep using the
    // existing policy methods; this owner adds no second policy algorithm.
    DrivePolicyConfig config;
    PedalingPolicy pedaling;
    CadenceShifter shifting;
    AssistController assist;
    Battery battery;
    Freehub hub;
};
} // namespace drivetrain
