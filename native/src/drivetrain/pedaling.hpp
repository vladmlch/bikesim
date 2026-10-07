#pragma once
#include "policy_config.hpp"

namespace drivetrain {
    struct PedalingState {
        std::string mode, reason;
        double effort_nm{}, required_cadence_rpm{};
        std::optional<double> target_phase_rad;
        double target_rate_rad_s{};
    };

    class PedalingPolicy {
    public:
        explicit PedalingPolicy(PedalingConfig config) : config_(config) {
            validate(config_);
        }

        void reset() { state_ = {}; }

        PedalingState update(double phase_rad, double rate_rad_s, double required_cadence_rpm,
                             double effort_nm, double dt, bool enabled = true, bool braking = false);

        const PedalingSnapshot &state() const { return state_; }

        // By value so transaction commits can move the candidate into
        // place — validation runs on the argument, then the publish is a
        // memory-only move.
        void set_state(PedalingSnapshot state);

    private:
        PedalingConfig config_;
        PedalingSnapshot state_;
    };

    double human_crank_torque(double mean_nm, double phase_rad, double ripple = .35);
}
