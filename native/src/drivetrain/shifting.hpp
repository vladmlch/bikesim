#pragma once
#include "policy_config.hpp"
#include <utility>
namespace drivetrain {
class CadenceShifter {
public:
    CadenceShifter(GearingConfig gearing, ShiftingConfig config) : gearing_(gearing), config_(std::move(config)) { reset(); }
    void reset();
    bool update(double cadence_rpm, double required_cadence_rpm, double dt,
                bool pedaling = true, bool braking = false, bool rear_in_contact = true,
                std::optional<double> rear_slip_mps = std::nullopt);
    double gear_ratio() const { return static_cast<double>(gearing_.front_teeth) / state_.rear_teeth; }
    double torque_factor() const { return state_.cut_remaining_s > 0. ? config_.torque_factor : 1.; }
    const ShiftingSnapshot& state() const { return state_; }
    void set_state(const ShiftingSnapshot& state);
private:
    GearingConfig gearing_;
    ShiftingConfig config_;
    ShiftingSnapshot state_;
};
}
