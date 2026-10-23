#pragma once

#include "types.hpp"

namespace bike::spatial {

struct EulerZyx {
    double roll_rad = 0.;
    double pitch_rad = 0.;
    double yaw_rad = 0.;
    bool valid = false;
};
Vec3 rotate_body_to_world(const Quat&, const Vec3&) noexcept;
Vec3 rotate_world_to_body(const Quat&, const Vec3&) noexcept;
Quat quat_multiply(const Quat&, const Quat&) noexcept;
Quat quat_conjugate(const Quat&) noexcept;
Quat quat_normalized(const Quat&) noexcept;
double geodesic_angle_rad(const Quat&, const Quat&) noexcept;
EulerZyx euler_zyx(const Quat&, double singularity_margin_rad = 1e-6) noexcept;
Vec3 point_velocity_world(const BodyPose&, const Vec3& point_world_m) noexcept;
double axis_rate(const Vec3& omega_child_world, const Vec3& omega_parent_world,
                 const Vec3& axis_world) noexcept;

} // namespace bike::spatial
