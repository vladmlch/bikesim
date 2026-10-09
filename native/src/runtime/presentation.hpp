// Owning presentation cache. Numerical lanes are captured at committed steps;
// Python dictionaries are built only when a frontend requests a frame.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "samples.hpp"
#include "../writers/writer_types.hpp"

namespace runtime {
struct PresentationSupport {
    bool in_platform = false;
    double normal_load_n = 0., gap_m = 0.;
};
struct PresentationState {
    double time_s = 0., position_m = 0., speed_mps = 0., pitch_rad = 0., z_m = 0.;
    int root_x_dofadr = 0;
    double fork_travel_mm = 0., fork_velocity_mps = 0., fork_force_n = 0.;
    double shock_stroke_mm = 0., shock_velocity_mps = 0., shock_force_n = 0.;
    double crank_phase_rad = 0., front_load_n = 0., rear_load_n = 0.;
    double battery_energy_j = 0.;
    std::optional<double> balance_lost_at_m;
    drivetrain::DriveTelemetry drive;
    bool assist_pedaling = false, rider_present = false, tire_present = false;
    bool grip_enabled = false, grip_reachable = false;
    double grip_gap_m = 0.;
    std::array<PresentationSupport, 3> supports{};
    bool stance_front = false, stance_rear = false;
    std::string ik_saturated, joints_saturated;
    double root_pitch_deg = 0., pelvis_pitch_deg = 0., torso_pitch_deg = 0.;
    std::vector<std::pair<std::string, double>> joints;
    std::vector<int> joint_qpos_addresses;
    std::array<double, 2> tire_slip{}, tire_mu{};
    // Array extents are initialized once from the owned model.
    std::array<std::vector<double>, 6> model_fields;
    [[nodiscard]] WireObject as_wire() const;
};
} // namespace runtime
