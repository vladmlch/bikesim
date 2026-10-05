// Stateful scalar port of ride/cruise.py; no actuator or mjData writes.
#pragma once

#include <mujoco/mujoco.h>
#include <optional>

#include "../config.hpp"

struct CruiseState {
    double target_speed_mps;
    double integral_mps_s = 0.0;
    double torque_nm = 0.0;
    bool engaged = false;
    double gain_scale = 1.0;
};

class CruiseWriter {
public:
    CruiseWriter(const mjModel* model, nativecfg::CruiseConfig config);
    [[nodiscard]] double compute(const mjData* data, bool rear_in_contact,
                                 bool traction_limited,
                                 std::optional<bool> controller_grounded);
    void reset();
    void set_target_speed(double value_kmh);
    [[nodiscard]] double set_assist_compensation(double support_factor);
    [[nodiscard]] CruiseState state() const { return state_; }
    void set_state(const CruiseState& state);

private:
    void validate_scale(double scale) const;
    nativecfg::CruiseConfig cfg_;
    mjtSize nv_;
    int root_dof_;
    double timestep_;
    CruiseState state_;
};
