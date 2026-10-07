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

        // By value so transaction commits can move the candidate into
        // place — validation runs on the argument, then the publish is a
        // memory-only move.
        void set_state(AssistSnapshot state);

        // noexcept publication for staged candidates that were validated
        // while staging (PreparedSettlement::commit is noexcept — no
        // validation may run there).
        void publish(AssistSnapshot state) noexcept { state_ = state; }

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

        // Pure candidate form of draw(): identical arithmetic, but the debit
        // lands on a detached snapshot so a settlement can stage the draw
        // before any live state moves.
        [[nodiscard]] std::pair<double, BatterySnapshot>
        debit(double requested_power_w, double dt) const;

        const BatterySnapshot &state() const { return state_; }

        // By value so transaction commits can move the candidate into
        // place — validation runs on the argument, then the publish is a
        // memory-only move.
        void set_state(BatterySnapshot state);

        // noexcept publication for staged candidates that were validated
        // while staging (PreparedSettlement::commit is noexcept — no
        // validation may run there).
        void publish(BatterySnapshot state) noexcept { state_ = state; }

    private:
        BatterySnapshot state_;
    };

    double motor_electrical_power(double torque, double omega, double a, double b, double idle, bool enabled);

    double limit_torque_by_energy(double request, double omega, double a, double b, double idle, double budget_w);
}
