// shoulder_profile.cpp — the cold io face of rider/spatial_support.cpp:
// schema-1 shoulder_profile document validation plus its content digest.
// The T56 hot-kernel clause bars io/* include edges under
// native/src/rider/, so the io:: consumer lives here while support_frame()
// stays in the kernel TU.

#include "../rider/spatial_support.hpp"

#include "../io/hash.hpp"
#include "../io/json.hpp"

#include <algorithm>
#include <array>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace bike::rider {
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
} // namespace bike::rider
