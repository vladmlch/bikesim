#pragma once
#include "policy_config.hpp"
namespace drivetrain {
class Freehub {
public:
    Freehub(double stiffness_nm_rad, double damping_nms_rad)
        : k_(positive(stiffness_nm_rad, "freehub stiffness")), c_(nonnegative(damping_nms_rad, "freehub damping")) {}
    void reset() { state_ = {}; }
    double update(double phi_c, double phi_w, double omega_c, double omega_w);
    const FreehubSnapshot& state() const { return state_; }
    void set_state(const FreehubSnapshot& state);
private:
    double k_, c_;
    FreehubSnapshot state_;
};
}
