#include "spatial/frames.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace bike::spatial {
namespace {
[[nodiscard]] bool all_finite(const auto& arr) noexcept {
    return std::ranges::all_of(arr, [](double x) { return std::isfinite(x); });
}
} // namespace

Quat quat_multiply(const Quat& a, const Quat& b) noexcept {
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
}

Quat quat_conjugate(const Quat& q) noexcept {
    return {q[0], -q[1], -q[2], -q[3]};
}

Quat quat_normalized(const Quat& q) noexcept {
    const double scale = std::max({std::abs(q[0]), std::abs(q[1]), std::abs(q[2]), std::abs(q[3])});
    if (!(scale > 0.) || !std::isfinite(scale)) {
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        return {invalid, invalid, invalid, invalid};
    }
    Quat r{q[0] / scale, q[1] / scale, q[2] / scale, q[3] / scale};
    const double norm = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
    for (double& c : r) {
        c /= norm;
    }
    return r;
}

Quat quat_from_euler_zyx(double roll_rad, double pitch_rad, double yaw_rad) noexcept {
    if (!std::isfinite(roll_rad) || !std::isfinite(pitch_rad) || !std::isfinite(yaw_rad)) {
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        return {invalid, invalid, invalid, invalid};
    }
    const Quat qx{std::cos(0.5 * roll_rad), std::sin(0.5 * roll_rad), 0., 0.};
    const Quat qy{std::cos(0.5 * pitch_rad), 0., std::sin(0.5 * pitch_rad), 0.};
    const Quat qz{std::cos(0.5 * yaw_rad), 0., 0., std::sin(0.5 * yaw_rad)};
    return quat_normalized(quat_multiply(quat_multiply(qz, qy), qx));
}

Vec3 rotate_body_to_world(const Quat& q, const Vec3& v) noexcept {
    const Vec3 t{
        2. * (q[2] * v[2] - q[3] * v[1]),
        2. * (q[3] * v[0] - q[1] * v[2]),
        2. * (q[1] * v[1] - q[2] * v[0]),
    };
    return {
        v[0] + q[0] * t[0] + q[2] * t[2] - q[3] * t[1],
        v[1] + q[0] * t[1] + q[3] * t[0] - q[1] * t[2],
        v[2] + q[0] * t[2] + q[1] * t[1] - q[2] * t[0],
    };
}

Vec3 rotate_world_to_body(const Quat& q, const Vec3& v) noexcept {
    return rotate_body_to_world(quat_conjugate(q), v);
}

double geodesic_angle_rad(const Quat& a, const Quat& b) noexcept {
    const Quat na = quat_normalized(a);
    const Quat nb = quat_normalized(b);
    if (!std::isfinite(na[0]) || !std::isfinite(nb[0])) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Quat d = quat_multiply(na, quat_conjugate(nb));
    return 2. * std::atan2(std::hypot(d[1], d[2], d[3]), std::abs(d[0]));
}

EulerZyx euler_zyx(const Quat& q, double gimbal_margin_rad) noexcept {
    EulerZyx out{};
    if (!all_finite(q) || !std::isfinite(gimbal_margin_rad) || gimbal_margin_rad <= 0. ||
        gimbal_margin_rad >= 0.5 * std::numbers::pi) {
        return out;
    }
    const Quat n = quat_normalized(q);
    if (!std::isfinite(n[0])) {
        return out;
    }
    const double sine = std::clamp(2. * (n[0] * n[2] - n[3] * n[1]), -1., 1.);
    out.pitch_rad = std::asin(sine);
    out.roll_rad = std::atan2(2. * (n[0] * n[1] + n[2] * n[3]),
                              1. - 2. * (n[1] * n[1] + n[2] * n[2]));
    out.yaw_rad = std::atan2(2. * (n[0] * n[3] + n[1] * n[2]),
                             1. - 2. * (n[2] * n[2] + n[3] * n[3]));
    out.valid = std::abs(out.pitch_rad) < (0.5 * std::numbers::pi - gimbal_margin_rad);
    return out;
}

EulerRatesZyx body_gyro_to_euler_zyx_rates(const EulerZyx& attitude,
                                           const Vec3& omega_body_rad_s) noexcept {
    EulerRatesZyx out{};
    if (!attitude.valid || !all_finite(omega_body_rad_s)) {
        return out;
    }
    const double cp = std::cos(attitude.pitch_rad);
    if (!(cp > 0.)) {
        return out;
    }
    const double sp = std::sin(attitude.pitch_rad);
    const double sr = std::sin(attitude.roll_rad);
    const double cr = std::cos(attitude.roll_rad);
    const double q_sin_r_cos = sr * omega_body_rad_s[1] + cr * omega_body_rad_s[2];
    out.roll_rate_rad_s = omega_body_rad_s[0] + (sp / cp) * q_sin_r_cos;
    out.pitch_rate_rad_s = cr * omega_body_rad_s[1] - sr * omega_body_rad_s[2];
    out.yaw_rate_rad_s = q_sin_r_cos / cp;
    if (!std::isfinite(out.roll_rate_rad_s) || !std::isfinite(out.pitch_rate_rad_s) ||
        !std::isfinite(out.yaw_rate_rad_s)) {
        return {};
    }
    out.valid = true;
    return out;
}

HorizontalHeading horizontal_heading_unit(const Quat& q, double gimbal_margin_rad) noexcept {
    HorizontalHeading out{};
    if (!all_finite(q) || !std::isfinite(gimbal_margin_rad) || gimbal_margin_rad <= 0. ||
        gimbal_margin_rad >= 0.5 * std::numbers::pi) {
        return out;
    }
    const Quat n = quat_normalized(q);
    if (!std::isfinite(n[0])) {
        return out;
    }
    const Vec3 forward = rotate_body_to_world(n, {1., 0., 0.});
    const double horiz = std::hypot(forward[0], forward[1]);
    if (!(horiz > std::sin(gimbal_margin_rad)) || !std::isfinite(horiz)) {
        return out;
    }
    out.hx = forward[0] / horiz;
    out.hy = forward[1] / horiz;
    out.heading_rad = std::atan2(out.hy, out.hx);
    out.valid = true;
    return out;
}

Vec3 point_velocity_world(const BodyPose& pose, const Vec3& point_world_m) noexcept {
    const Vec3 r{
        point_world_m[0] - pose.position_world_m[0],
        point_world_m[1] - pose.position_world_m[1],
        point_world_m[2] - pose.position_world_m[2],
    };
    const Vec3& w = pose.angular_velocity_world_rad_s;
    const Vec3& v = pose.linear_velocity_world_mps;
    return {
        v[0] + w[1] * r[2] - w[2] * r[1],
        v[1] + w[2] * r[0] - w[0] * r[2],
        v[2] + w[0] * r[1] - w[1] * r[0],
    };
}

} // namespace bike::spatial
