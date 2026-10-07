#pragma once
#include "policy_config.hpp"
#include <utility>

namespace drivetrain {
    class CadenceShifter {
    public:
        CadenceShifter(GearingConfig gearing, ShiftingConfig config) : gearing_(gearing), config_(std::move(config)) {
            validate(gearing_);
            validate(config_);
            if (config_.enabled && std::ranges::find(config_.cassette, gearing_.rear_teeth) == config_.cassette.end())
                throw std::invalid_argument("CadenceShifter.rear_teeth");
            reset();
        }

        void reset() {
            state_ = {};
            state_.rear_teeth = gearing_.rear_teeth;
            state_.from_teeth = state_.rear_teeth;
        }

        bool update(double cadence_rpm, double required_cadence_rpm, double dt,
                    bool pedaling = true, bool braking = false, bool rear_in_contact = true,
                    std::optional<double> rear_slip_mps = std::nullopt);

        double gear_ratio() const { return static_cast<double>(gearing_.front_teeth) / state_.rear_teeth; }
        double torque_factor() const { return state_.cut_remaining_s > 0. ? config_.torque_factor : 1.; }
        const ShiftingSnapshot &state() const { return state_; }

        // By value so transaction commits can move the candidate into
        // place — validation runs on the argument, then the publish is a
        // memory-only move (the direction string never re-allocates).
        void set_state(ShiftingSnapshot state);

    private:
        GearingConfig gearing_;
        ShiftingConfig config_;
        ShiftingSnapshot state_;
    };
}
