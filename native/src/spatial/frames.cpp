#include "frames.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace bike::spatial {
Quat quat_multiply(const Quat& a, const Quat& b) noexcept {
    return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
Quat quat_conjugate(const Quat& q) noexcept { return {q[0],-q[1],-q[2],-q[3]}; }
Quat quat_normalized(const Quat& q) noexcept {
    const double scale=std::max({std::abs(q[0]),std::abs(q[1]),std::abs(q[2]),std::abs(q[3])});
    if (!(scale>0.) || !std::isfinite(scale)) {
        const auto invalid=std::numeric_limits<double>::quiet_NaN();
        return {invalid,invalid,invalid,invalid};
    }
    Quat r{q[0]/scale,q[1]/scale,q[2]/scale,q[3]/scale};
    const auto n=std::sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]+r[3]*r[3]);
    for (auto& component:r) component/=n;
    return r;
}
Vec3 rotate_body_to_world(const Quat& q, const Vec3& v) noexcept {
    // The contract supplies a unit quaternion. Avoid rewriting raw engine
    // state, and use the same expression for q and -q.
    const Vec3 t{2.*(q[2]*v[2]-q[3]*v[1]),
                 2.*(q[3]*v[0]-q[1]*v[2]),
                 2.*(q[1]*v[1]-q[2]*v[0])};
    return {v[0]+q[0]*t[0]+q[2]*t[2]-q[3]*t[1],
            v[1]+q[0]*t[1]+q[3]*t[0]-q[1]*t[2],
            v[2]+q[0]*t[2]+q[1]*t[1]-q[2]*t[0]};
}
Vec3 rotate_world_to_body(const Quat& q, const Vec3& v) noexcept {
    return rotate_body_to_world(quat_conjugate(q),v);
}
double geodesic_angle_rad(const Quat& a, const Quat& b) noexcept {
    const auto d=quat_multiply(quat_normalized(a),quat_conjugate(quat_normalized(b)));
    return 2.*std::atan2(std::hypot(d[1],d[2],d[3]),std::abs(d[0]));
}
EulerZyx euler_zyx(const Quat& q, double margin) noexcept {
    EulerZyx result;
    const auto n=quat_normalized(q);
    if (!std::isfinite(n[0]) || !std::isfinite(margin) || margin<0. || margin>=std::numbers::pi/2.) return result;
    const double sine=std::clamp(2.*(n[0]*n[2]-n[3]*n[1]),-1.,1.);
    result.pitch_rad=std::asin(sine);
    result.roll_rad=std::atan2(2.*(n[0]*n[1]+n[2]*n[3]),1.-2.*(n[1]*n[1]+n[2]*n[2]));
    result.yaw_rad=std::atan2(2.*(n[0]*n[3]+n[1]*n[2]),1.-2.*(n[2]*n[2]+n[3]*n[3]));
    result.valid=std::abs(result.pitch_rad)<std::numbers::pi/2.-margin;
    return result;
}
Vec3 point_velocity_world(const BodyPose& pose, const Vec3& point) noexcept {
    const Vec3 r{point[0]-pose.position_world_m[0],point[1]-pose.position_world_m[1],point[2]-pose.position_world_m[2]};
    const auto& w=pose.angular_velocity_world_rad_s;
    const auto& v=pose.linear_velocity_world_mps;
    return {v[0]+w[1]*r[2]-w[2]*r[1],v[1]+w[2]*r[0]-w[0]*r[2],v[2]+w[0]*r[1]-w[1]*r[0]};
}
double axis_rate(const Vec3& child, const Vec3& parent, const Vec3& axis) noexcept {
    return (child[0]-parent[0])*axis[0]+(child[1]-parent[1])*axis[1]+(child[2]-parent[2])*axis[2];
}
} // namespace bike::spatial
