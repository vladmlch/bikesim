#pragma once

#include <array>
#include <numbers>

namespace bike::spatial {

using Vec3 = std::array<double, 3>;
// MuJoCo convention: (w, x, y, z), rotating body-frame vectors to world-frame.
using Quat = std::array<double, 4>;

inline constexpr double kDefaultGimbalMarginRad = std::numbers::pi / 18.; // 10 deg -> |pitch| < 80 deg

struct EulerZyx {
    double roll_rad = 0.;
    double pitch_rad = 0.;
    double yaw_rad = 0.;
    bool valid = false;
};

struct EulerRatesZyx {
    double roll_rate_rad_s = 0.;
    double pitch_rate_rad_s = 0.;
    double yaw_rate_rad_s = 0.;
    bool valid = false;
};

struct BodyPose {
    Vec3 position_world_m{};
    Quat quat_world_from_body_wxyz{1., 0., 0., 0.};
    Vec3 linear_velocity_world_mps{};
    Vec3 angular_velocity_world_rad_s{};
};

[[nodiscard]] Quat quat_multiply(const Quat& a, const Quat& b) noexcept;
[[nodiscard]] Quat quat_conjugate(const Quat& q) noexcept;
[[nodiscard]] Quat quat_normalized(const Quat& q) noexcept;
[[nodiscard]] Quat quat_from_euler_zyx(double roll_rad, double pitch_rad, double yaw_rad) noexcept;

[[nodiscard]] Vec3 rotate_body_to_world(const Quat& q_world_from_body, const Vec3& v_body) noexcept;
[[nodiscard]] Vec3 rotate_world_to_body(const Quat& q_world_from_body, const Vec3& v_world) noexcept;

[[nodiscard]] double geodesic_angle_rad(const Quat& a, const Quat& b) noexcept;
[[nodiscard]] EulerZyx euler_zyx(const Quat& q_world_from_body,
                                 double gimbal_margin_rad = kDefaultGimbalMarginRad) noexcept;
[[nodiscard]] EulerRatesZyx body_gyro_to_euler_zyx_rates(
    const EulerZyx& attitude, const Vec3& omega_body_rad_s) noexcept;

// Unit horizontal heading vector (hx, hy, 0) of body +X projected onto world XY.
// Returns valid=false when body +X is within gimbal_margin_rad of world +-Z.
struct HorizontalHeading {
    double hx = 1.;
    double hy = 0.;
    double heading_rad = 0.;
    bool valid = false;
};
[[nodiscard]] HorizontalHeading horizontal_heading_unit(
    const Quat& q_world_from_body,
    double gimbal_margin_rad = kDefaultGimbalMarginRad) noexcept;

[[nodiscard]] Vec3 point_velocity_world(const BodyPose& pose, const Vec3& point_world_m) noexcept;

} // namespace bike::spatial
