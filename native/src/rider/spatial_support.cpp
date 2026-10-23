#include "spatial_support.hpp"

#include "../diag.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../spatial/frames.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <span>
#include <stdexcept>

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

ShoulderProfile read_shoulder_profile(const io::Value& value) {
    const auto& object = io::as_object(value, "shoulder_profile");
    constexpr std::array<std::string_view, 7> names{
        "schema", "swivel_range_rad", "swivel_stiffness_nm_per_rad",
        "swivel_damping_nms_per_rad", "swivel_strength_nm",
        "provenance_status", "sha256_hex"};
    if (object.size() != names.size())
        throw std::invalid_argument("shoulder_profile: exact schema-1 fields required");
    for (const auto& [name, member] : object) {
        (void)member;
        if (std::ranges::find(names, name) == names.end())
            throw std::invalid_argument("shoulder_profile." + name + ": unknown field");
    }
    const auto field = [&](std::string_view name) -> const io::Value& {
        const auto* member = io::find_member(object, name);
        if (member == nullptr)
            throw std::invalid_argument("shoulder_profile." + std::string(name) + ": required");
        return *member;
    };
    if (io::as_int64(field("schema"), "shoulder_profile.schema") != 1)
        throw std::invalid_argument("shoulder_profile.schema: expected 1");
    ShoulderProfile profile;
    profile.swivel_range_rad = io::as_double(field("swivel_range_rad"), "shoulder_profile.swivel_range_rad");
    profile.swivel_stiffness_nm_per_rad = io::as_double(field("swivel_stiffness_nm_per_rad"), "shoulder_profile.swivel_stiffness_nm_per_rad");
    profile.swivel_damping_nms_per_rad = io::as_double(field("swivel_damping_nms_per_rad"), "shoulder_profile.swivel_damping_nms_per_rad");
    profile.swivel_strength_nm = io::as_double(field("swivel_strength_nm"), "shoulder_profile.swivel_strength_nm");
    profile.provenance_status = io::as_string(field("provenance_status"), "shoulder_profile.provenance_status");
    if (!(profile.swivel_range_rad > 0. && profile.swivel_range_rad < std::numbers::pi / 2.))
        throw std::invalid_argument("shoulder_profile.swivel_range_rad: expected (0, pi/2)");
    if (profile.swivel_stiffness_nm_per_rad < 0. || profile.swivel_damping_nms_per_rad < 0. ||
        !(profile.swivel_strength_nm > 0.))
        throw std::invalid_argument("shoulder_profile: nonnegative stiffness/damping and positive strength required");
    if (profile.provenance_status != "provisional" && profile.provenance_status != "qualified")
        throw std::invalid_argument("shoulder_profile.provenance_status: expected provisional or qualified");
    // The content digest excludes only its own slot. It identifies the exact
    // canonical source (including number spelling), not a physiological claim.
    io::Value::Object payload;
    for (const auto& [name, member] : object)
        if (name != "sha256_hex") payload.emplace_back(name, member);
    const std::string canonical = io::canonical_json(io::Value{std::move(payload)});
    profile.sha256_hex = io::sha256(std::as_bytes(std::span{canonical.data(), canonical.size()}));
    if (!io::is_null(field("sha256_hex")) &&
        io::as_string(field("sha256_hex"), "shoulder_profile.sha256_hex") != profile.sha256_hex)
        throw std::invalid_argument("shoulder_profile.sha256_hex: content digest mismatch");
    if (profile.provenance_status == "qualified" && io::is_null(field("sha256_hex")))
        throw std::invalid_argument("shoulder_profile.sha256_hex: qualified profile requires a digest");
    return profile;
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
    for (double x : q) {
        if (!std::isfinite(x)) return out;
        qnorm += x*x;
    }
    if (!std::isfinite(qnorm) || std::abs(qnorm-1.) > 1e-8) return out;
    // Project body +X into the support plane. Near its singular direction
    // body +Y is the deterministic fallback. Both rotate with the support.
    Vec3 tangent{1.-normal[0]*normal[0], -normal[0]*normal[1], -normal[0]*normal[2]};
    if (!normalize(tangent)) {
        tangent = {-normal[1]*normal[0], 1.-normal[1]*normal[1], -normal[1]*normal[2]};
        if (!normalize(tangent)) return out;
    }
    Vec3 second = cross(normal, tangent);
    if (!normalize(second) || std::abs(dot(normal, tangent)) > 1e-12) return out;
    out.normal_world = spatial::rotate_body_to_world(q, normal);
    out.tangent_x_world = spatial::rotate_body_to_world(q, tangent);
    out.tangent_y_world = spatial::rotate_body_to_world(q, second);
    out.valid = true;
    return out;
}
NATIVE_DIAG_POP
} // namespace bike::rider
