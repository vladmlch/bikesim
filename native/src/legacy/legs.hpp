#pragma once
#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>
#include "../model/rider_geometry.hpp"
#include "../rider/spindle_math.hpp"
#include <mujoco/mujoco.h>

namespace bike::legacy {
// T32 reuses the already source-equivalent two-link virtual-work math.
using spindle::LegLoopGeometry;
using spindle::leg_loop_geometry;
using spindle::leg_joint_q;
using spindle::leg_loop_jacobian;
using spindle::spindle_xz;
using spindle::planar_angle;

class LegDrive {
public:
    LegDrive(const mjModel&, const std::optional<model::SeatedPose>&,
             double crank_length_m, double ripple_depth);
    [[nodiscard]] bool active() const noexcept { return !legs_.empty(); }
    [[nodiscard]] std::size_t crank_qposadr() const noexcept { return crank_q_; }
    void initialize(mjData&, double phase_rad);
    void compute_into(const mjData&, double rider_torque_nm, std::span<double> force);
    [[nodiscard]] std::array<double,2> pedal_forces_n() const noexcept { return pedal_force_; }
private:
    struct Leg {
        model::LegChain chain;
        std::array<std::size_t,3> q{}, v{};
        std::size_t pedal_q=0, pedal_v=0;
        std::optional<model::Vec3> previous;
    };
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    const mjModel& model_;
    double crank_length_, ripple_depth_;
    std::size_t pelvis_q_ = 0, crank_q_ = 0;
    std::vector<Leg> legs_;
    std::array<double,2> pedal_force_{};
};
} // namespace bike::legacy
