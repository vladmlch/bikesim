#pragma once
#include "policy_config.hpp"
#include <utility>

namespace drivetrain {
    class AssistController {
    public:
        explicit AssistController(AssistConfig config) : config_(std::move(config)) {
            validate(config_);
        }

        void reset() { state_ = {}; }

        std::pair<double, double> ceiling(double shaft_rpm, double speed_mps) const;

        double step(double human_nm, double cadence_rpm, double speed_mps, bool braking, double dt,
                    std::optional<double> torque_request_nm = std::nullopt,
                    std::optional<double> shaft_rpm = std::nullopt);

        const AssistSnapshot &state() const { return state_; }

        void set_state(const AssistSnapshot &state);

    private:
        AssistConfig config_;
        AssistSnapshot state_;
    };

    class Battery {
    public:
        explicit Battery(double energy_j) : state_{
            .initial_energy_j = nonnegative(energy_j, "battery energy"), .energy_j = energy_j, .drawn_energy_j = 0.
        } {
        }

        void reset() {
            state_.energy_j = state_.initial_energy_j;
            state_.drawn_energy_j = 0.;
        }

        double draw(double requested_power_w, double dt);

        const BatterySnapshot &state() const { return state_; }

        void set_state(const BatterySnapshot &state);

    private:
        BatterySnapshot state_;
    };

    double motor_electrical_power(double torque, double omega, double a, double b, double idle, bool enabled);

    double limit_torque_by_energy(double request, double omega, double a, double b, double idle, double budget_w);
}
