#include "spatial_support.hpp"

#include "../diag.hpp"
#include "../spatial/frames.hpp"

#include <cmath>

namespace bike::rider {
namespace {
using spatial::Vec3;
double dot(const Vec3& a, const Vec3& b) noexcept {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0]};
}
bool normalize(Vec3& v) noexcept {
    const double length = std::hypot(v[0], v[1], v[2]);
    if (!std::isfinite(length) || length <= 1e-14) return false;
    for (double& x : v) x /= length;
    return true;
}
}

// W00 fixes this complete frame as a returned value. Suppress only the
// return-size diagnostic here; by-value parameters remain checked elsewhere.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
SpatialSupportFrame support_frame(const spatial::BodyPose& pose,
                                  const Vec3& normal_body) noexcept {
    SpatialSupportFrame out{};
    Vec3 normal = normal_body;
    if (!normalize(normal)) return out;
    const auto& q = pose.quat_world_from_body_wxyz;
    double qnorm = 0.;
    for (const double x : q) {
        if (!std::isfinite(x)) return out;
        qnorm += x*x;
    }
    if (!std::isfinite(qnorm) || std::abs(qnorm-1.) > 1e-8) return out;
    // The tolerance admits ordinary pose roundoff, but the resulting basis
    // must still be orthonormal: attachment projection and its checkpoint
    // codec consume unit axes. Normalize only this local rotation, leaving
    // the caller's integration quaternion untouched.
    const auto rotation = spatial::quat_normalized(q);
    // Project body +X into the support plane. Near its singular direction
    // body +Y is the deterministic fallback. Both rotate with the support.
    Vec3 tangent{1.-normal[0]*normal[0], -normal[0]*normal[1], -normal[0]*normal[2]};
    if (!normalize(tangent)) {
        tangent = {-normal[1]*normal[0], 1.-normal[1]*normal[1], -normal[1]*normal[2]};
        if (!normalize(tangent)) return out;
    }
    Vec3 second = cross(normal, tangent);
    if (!normalize(second) || std::abs(dot(normal, tangent)) > 1e-12) return out;
    out.normal_world = spatial::rotate_body_to_world(rotation, normal);
    out.tangent_x_world = spatial::rotate_body_to_world(rotation, tangent);
    out.tangent_y_world = spatial::rotate_body_to_world(rotation, second);
    out.valid = true;
    return out;
}
NATIVE_DIAG_POP
} // namespace bike::rider
