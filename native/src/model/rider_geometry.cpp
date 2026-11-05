// model/rider_geometry.cpp — T15 rider boundary implementations.
//
// numpy-provider notes kept inline at each site. Scalar `x ** y` goes
// through pyfloat::float_pow (CPython float_pow's complete ladder). Every
// sin/cos call takes the numeric::np_* or math_* family of its Python
// mirror (np.* or math.*), so the optimizer cannot merge a same-angle
// pair into a __sincos_stret the oracle never calls.
#include "rider_geometry.hpp"

#include "../diag.hpp"
#include "../io/value.hpp" // io::detail Unicode-16 printability helpers
#include "../numeric_sincos.hpp"
#include "../numeric_sum.hpp"
#include "../writers/pyfloat.hpp"
#include "checks.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// The pose/segment records are returned by value deliberately (the
// dict/tuple results the oracle returns); -Wlarge-by-value-copy exists
// to flag by-value *parameters* and clang gives no param/return split —
// same convention as kinematics/solver.cpp.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")

namespace {

using bike::model::Mat3;
using bike::model::SeatedPose;
using bike::model::Vec3;
namespace gd = bike::geometry::detail;
using bike::model::checks::scalar;

[[nodiscard]] inline double py_pow(double base, double exponent) {
    return pyfloat::float_pow(base, exponent);
}

// Elementwise Vec3 ops — the ndarray expressions' lane order.
[[nodiscard]] inline Vec3 v_add(const Vec3 &a, const Vec3 &b) noexcept {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
[[nodiscard]] inline Vec3 v_sub(const Vec3 &a, const Vec3 &b) noexcept {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
[[nodiscard]] inline Vec3 v_scale(double s, const Vec3 &v) noexcept {
    return {s * v[0], s * v[1], s * v[2]};
}
[[nodiscard]] inline Vec3 v_div(const Vec3 &v, double s) noexcept {
    return {v[0] / s, v[1] / s, v[2] / s};
}

// repr() of a Python str for f-string {name!r} interpolation and
// KeyError payloads: CPython prefers single quotes but switches to
// double quotes when the string contains "'" and no '"'.
[[nodiscard]] std::string py_str_repr(std::string_view s) {
    const bool has_single = s.find('\'') != std::string_view::npos;
    const bool has_double = s.find('"') != std::string_view::npos;
    const char quote = (has_single && !has_double) ? '"' : '\'';
    std::string out(1, quote);
    static constexpr std::string_view kHex = "0123456789abcdef";
    for (std::size_t i = 0; i < s.size();) {
        const auto c = static_cast<unsigned char>(s[i]);
        if (c == static_cast<unsigned char>(quote) || c == '\\') {
            out.push_back('\\');
            out.push_back(static_cast<char>(c));
            ++i;
            continue;
        }
        if (c >= 0x80) {
            // CPython 3.13 printability: Unicode-16-only codepoints are
            // unassigned there, so repr escapes them.
            std::size_t pos = i;
            const char32_t cp = bike::io::detail::utf8_decode_one(s, pos);
            if (cp != bike::io::detail::kInvalidCodepoint) {
                if (bike::io::detail::unicode16_only_printable(cp))
                    bike::io::detail::repr_escape_nonprintable(out, cp);
                else
                    out.append(s, i, pos - i);
                i = pos;
            } else {
                out.push_back(s[i]); // malformed UTF-8: no CPython analogue
                ++i;
            }
            continue;
        }
        switch (c) {
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20 || c == 0x7f) {
                    out += "\\x";
                    out.push_back(kHex[c >> 4U]);
                    out.push_back(kHex[c & 0xfU]);
                } else {
                    out.push_back(static_cast<char>(c));
                }
        }
        ++i;
    }
    out.push_back(quote);
    return out;
}

// ---------------------------------------------------------------------
// The dict tables — insertion order kept as declared.
struct NameValue {
    std::string_view name;
    double value;
};
constexpr std::array kDeLevaMassFractions{
    NameValue{.name = "head", .value = 0.0694},
    NameValue{.name = "trunk_upper", .value = 0.1596},
    NameValue{.name = "trunk_middle", .value = 0.1633},
    NameValue{.name = "trunk_lower", .value = 0.1117},
    NameValue{.name = "upper_arm", .value = 0.0271},
    NameValue{.name = "forearm", .value = 0.0162},
    NameValue{.name = "hand", .value = 0.0061},
    NameValue{.name = "thigh", .value = 0.1416},
    NameValue{.name = "shank", .value = 0.0433},
    NameValue{.name = "foot", .value = 0.0137},
};
[[nodiscard]] double dict_lookup(std::span<const NameValue> table,
                                 std::string_view name) {
    for (const NameValue &row: table)
        if (row.name == name) return row.value;
    throw std::out_of_range(py_str_repr(name)); // KeyError payload
}

} // namespace

namespace bike::model {

// ---------------------------------------------------------------------
// Module tables
// ---------------------------------------------------------------------
double de_leva_mass_fraction(std::string_view name) {
    return dict_lookup(std::span<const NameValue>{kDeLevaMassFractions},
                       name);
}
bool has_de_leva_mass_fraction(std::string_view name) noexcept {
    for (const NameValue &entry: kDeLevaMassFractions)
        if (entry.name == name) return true;
    return false;
}
Vec3 lumped_com_m(std::string_view name) {
    // LUMPED_COM_M — the three standing-pose capsule centres.
    if (name == "rider_torso") return {0.160, 0.0, 0.650};
    if (name == "rider_legs") return {0.050, 0.0, 0.240};
    if (name == "rider_arms") return {0.335, 0.0, 0.720};
    throw std::out_of_range(py_str_repr(name));
}

// ---------------------------------------------------------------------
// Module-level helpers
// ---------------------------------------------------------------------
Vec3 two_link_ik(const Vec3 &a, const Vec3 &b, double len_a,
                 double len_b, std::string_view prefer) {
    const Vec3 d_vec = v_sub(b, a);
    const double d = std::hypot(d_vec[0], d_vec[2]);
    if (d > len_a + len_b + 1e-12)
        throw std::invalid_argument(
                "endpoints " + gd::py_fixed(d, 3) +
                " m apart exceed the chain's " +
                gd::py_fixed(len_a + len_b, 3) + " m reach");
    if (d < std::fabs(len_a - len_b) - 1e-12)
        throw std::invalid_argument(
                "endpoints " + gd::py_fixed(d, 3) +
                " m apart are closer than the chain's " +
                gd::py_fixed(std::fabs(len_a - len_b), 3) + " m minimum");
    const double along =
            (len_a * len_a - len_b * len_b + d * d) / (2.0 * d);
    // sqrt(max(0.0, len_a*len_a - along*along)) — Python max(0.0, x)
    // returns x only when x > 0.0.
    const double off_arg = len_a * len_a - along * along;
    const double off = std::sqrt(off_arg > 0.0 ? off_arg : 0.0);
    const Vec3 u{d_vec[0] / d, 0.0, d_vec[2] / d};
    const Vec3 n{-u[2], 0.0, u[0]};
    const Vec3 foot = v_add(a, v_scale(along, u));
    const Vec3 c1 = v_add(foot, v_scale(off, n));
    const Vec3 c2 = v_sub(foot, v_scale(off, n));
    // axis = {"+x": (0, 1.0), "-x": (0, -1.0), "+z": (2, 1.0),
    //         "-z": (2, -1.0)}[prefer]
    std::size_t axis_i = 0;
    double axis_s = 1.0;
    if (prefer == "+x") { axis_i = 0; axis_s = 1.0; }
    else if (prefer == "-x") { axis_i = 0; axis_s = -1.0; }
    else if (prefer == "+z") { axis_i = 2; axis_s = 1.0; }
    else if (prefer == "-z") { axis_i = 2; axis_s = -1.0; }
    else throw std::out_of_range(py_str_repr(prefer)); // dict KeyError
    return axis_s * c1[axis_i] >= axis_s * c2[axis_i] ? c1 : c2;
}

double flexion_deg(const Vec3 &proximal, const Vec3 &joint,
                   const Vec3 &distal) {
    const Vec3 v1 = v_sub(proximal, joint);
    const Vec3 v2 = v_sub(distal, joint);
    const double cosang = gd::np_dot3(v1, v2) /
                          (gd::np_norm3(v1) * gd::np_norm3(v2));
    // max(-1.0, min(1.0, cosang)) — Python builtin argument order.
    const double lo = cosang < 1.0 ? cosang : 1.0;
    const double clamped = lo > -1.0 ? lo : -1.0;
    return 180.0 - gd::np_degrees(std::acos(clamped));
}

// ---------------------------------------------------------------------
// solve_seated_pose
// ---------------------------------------------------------------------
SeatedPose solve_seated_pose(const BikeSpecs &specs,
                             const RiderSpecs &rider) {
    const geometry::FixedFramePoints fixed =
            geometry::get_fixed_frame_points(specs);
    const Vec3 p9{fixed.P9[0] / 1000.0, fixed.P9[1] / 1000.0,
                  fixed.P9[2] / 1000.0};

    const geometry::SaddleGeometry saddle = geometry::saddle_geometry(
            p9, geometry::saddle_top_z_for_height(
                        p9, rider.saddle_height_m()));
    if (!(geometry::MIN_EXPOSED_SEATPOST_M <= saddle.exposed_post_m &&
          saddle.exposed_post_m <= geometry::MAX_EXPOSED_SEATPOST_M))
        throw std::invalid_argument(
                "a " + gd::py_fixed(rider.height_m, 2) +
                " m rider with a " + gd::py_fixed(rider.inseam(), 3) +
                " m inseam needs a " +
                gd::py_fixed(rider.saddle_height_m(), 3) +
                " m saddle height, which exposes " +
                gd::py_fixed(saddle.exposed_post_m * 1000.0, 0) +
                " mm of seatpost; the frame accepts " +
                gd::py_fixed(geometry::MIN_EXPOSED_SEATPOST_M * 1000.0,
                             0) +
                "-" +
                gd::py_fixed(geometry::MAX_EXPOSED_SEATPOST_M * 1000.0,
                             0) +
                " mm. Adjust --rider-height or --rider-inseam.");

    const Vec3 hip =
            v_add(saddle.top_center, {0.0, 0.0, HIP_ABOVE_SADDLE_M});
    const Vec3 grip = geometry::handlebar_grip_point(specs);
    const double crank_m = specs.crank_length / 1000.0;
    const auto [pedal_front, pedal_rear] =
            geometry::pedal_points(crank_m);
    const Vec3 ankle_offset{0.0, 0.0, ANKLE_ABOVE_PEDAL_M};
    const Vec3 ankle_front = v_add(pedal_front, ankle_offset);
    const Vec3 ankle_rear = v_add(pedal_rear, ankle_offset);

    const double l_thigh = rider.segment_length("thigh");
    const double l_shank = rider.segment_length("shank");
    const double l_trunk = rider.segment_length("trunk");
    const double l_ua = rider.segment_length("upper_arm");
    const double l_fa = rider.segment_length("forearm");
    const double l_hand = rider.segment_length("hand");

    Vec3 knee_front, knee_rear;
    try {
        knee_front =
                two_link_ik(hip, ankle_front, l_thigh, l_shank, "+x");
        knee_rear = two_link_ik(hip, ankle_rear, l_thigh, l_shank, "+x");
    } catch (const std::invalid_argument &exc) {
        throw std::invalid_argument(
                std::string("the rider's legs do not reach the pedals "
                            "from the saddle: ") +
                exc.what());
    }

    const Vec3 ankle_bdc =
            v_add(Vec3{0.0, 0.0, -crank_m}, ankle_offset);
    Vec3 knee_bdc;
    try {
        knee_bdc =
                two_link_ik(hip, ankle_bdc, l_thigh, l_shank, "+x");
    } catch (const std::invalid_argument &exc) {
        throw std::invalid_argument(
                std::string("the leg does not reach the pedal at bottom "
                            "dead centre (") +
                exc.what() + "); the saddle at " +
                gd::py_fixed(rider.saddle_height_m(), 3) +
                " m is too high for a " +
                gd::py_fixed(rider.height_m, 2) + " m rider");
    }
    const double knee_flexion_bdc =
            flexion_deg(hip, knee_bdc, ankle_bdc);
    if (!(KNEE_FLEXION_BDC_MIN_DEG <= knee_flexion_bdc &&
          knee_flexion_bdc <= KNEE_FLEXION_BDC_MAX_DEG))
        throw std::invalid_argument(
                "knee flexion at bottom dead centre is " +
                gd::py_fixed(knee_flexion_bdc, 1) +
                " degrees; the fit accepts " +
                gd::py_fixed(KNEE_FLEXION_BDC_MIN_DEG, 0) + "-" +
                gd::py_fixed(KNEE_FLEXION_BDC_MAX_DEG, 0) +
                ". Saddle " +
                gd::py_fixed(rider.saddle_height_m(), 3) + " m from a " +
                gd::py_fixed(rider.inseam(), 3) + " m inseam against " +
                gd::py_fixed(specs.crank_length, 0) +
                " mm cranks; adjust --rider-height, --rider-inseam or "
                "the crank length.");

    const double elbow_rad = gd::np_radians(ELBOW_FLEXION_DEG);
    const double arm_chord =
            std::sqrt(l_ua * l_ua + l_fa * l_fa +
                      2.0 * l_ua * l_fa * numeric::math_cos(elbow_rad));
    const double arm_reach =
            arm_chord + GRIP_FROM_WRIST_FRACTION * l_hand;
    Vec3 shoulder;
    try {
        shoulder = two_link_ik(hip, grip, l_trunk, arm_reach, "+z");
    } catch (const std::invalid_argument &exc) {
        throw std::invalid_argument(
                std::string("the rider cannot reach the handlebar from "
                            "the saddle: ") +
                exc.what());
    }
    const Vec3 to_grip_raw = v_sub(grip, shoulder);
    const Vec3 to_grip =
            v_div(to_grip_raw, gd::np_norm3(to_grip_raw));
    const Vec3 wrist = v_sub(
            grip, v_scale(GRIP_FROM_WRIST_FRACTION * l_hand, to_grip));
    const Vec3 elbow = two_link_ik(shoulder, wrist, l_ua, l_fa, "-z");

    const Vec3 torso_axis_raw = v_sub(shoulder, hip);
    const Vec3 torso_axis =
            v_div(torso_axis_raw, gd::np_norm3(torso_axis_raw));
    const double lo =
            torso_axis[2] < 1.0 ? torso_axis[2] : 1.0;
    const double torso_lean = gd::np_degrees(
            std::acos(lo > -1.0 ? lo : -1.0));
    const Vec3 head_center =
            v_add(shoulder,
                  v_scale(NECK_LENGTH_M + HEAD_RADIUS_M, torso_axis));

    return SeatedPose{
            .saddle = saddle,
            .hip = hip,
            .shoulder = shoulder,
            .elbow = elbow,
            .wrist = wrist,
            .grip = grip,
            .head_center = head_center,
            .knee_front = knee_front,
            .knee_rear = knee_rear,
            .ankle_front = ankle_front,
            .ankle_rear = ankle_rear,
            .pedal_front = pedal_front,
            .pedal_rear = pedal_rear,
            .torso_lean_deg = torso_lean,
            .knee_flexion_front_deg =
                    flexion_deg(hip, knee_front, ankle_front),
            .knee_flexion_rear_deg =
                    flexion_deg(hip, knee_rear, ankle_rear),
            .knee_flexion_bdc_deg = knee_flexion_bdc,
    };
}

// RiderSpecs.seated_pose member ports. The seated variant gate is gone
// with the variant: both remaining callers (geometry_pose and
// compute_rider_centers_of_mass) pass their own record and the pose
// solver consumes it as geometry input.
SeatedPose seated_pose(const RiderSpecs &rider, const BikeSpecs &specs) {
    return solve_seated_pose(specs, rider);
}
SeatedPose seated_pose(const RiderSpecs &rider) {
    return seated_pose(rider, BikeSpecs{});
}

// ---------------------------------------------------------------------
// RiderSpecs.compute_rider_centers_of_mass
// ---------------------------------------------------------------------
std::vector<RiderCenter>
compute_rider_centers_of_mass(const RiderSpecs &rider,
                              const BikeSpecs &specs) {
    if (rider.variant == "none") return {};
    if (rider.variant == "lumped")
        return {
            RiderCenter{.name = "rider_torso",
                        .pos_m = lumped_com_m("rider_torso"),
                        .mass_kg = rider.torso_helmet_mass()},
            RiderCenter{.name = "rider_legs",
                        .pos_m = lumped_com_m("rider_legs"),
                        .mass_kg = rider.legs_mass()},
            RiderCenter{.name = "rider_arms",
                        .pos_m = lumped_com_m("rider_arms"),
                        .mass_kg = rider.arms_mass()},
        };
    if (rider.variant == "articulated_planar") {
        const ArticulatedPose pose = geometry_pose(rider, specs);
        const SegmentMasses masses = split_paired_arm_masses(
                segment_masses(rider.mass_kg, rider.helmet_mass_kg));
        // Ordered centers dict, then a rider_-prefixed emit pass.
        std::vector<std::pair<std::string, Vec3>> centers;
        centers.emplace_back("pelvis",
                             v_add(pose.hip, {0.0, 0.0, 0.06}));
        centers.emplace_back("torso",
                             v_div(v_add(pose.hip, pose.shoulder), 2.0));
        centers.emplace_back("head", pose.head_center);
        const std::array<std::pair<std::string_view, double>, 2> arms{
            std::pair{"left", -1.0}, std::pair{"right", 1.0}};
        const Vec3 upper_arm_mid =
                v_div(v_add(pose.shoulder, pose.elbow), 2.0);
        const Vec3 forearm_mid =
                v_div(v_add(pose.elbow, pose.grip), 2.0);
        for (const auto &[side, sign]: arms) {
            const std::array<std::pair<std::string_view, Vec3>, 2>
                    parts{std::pair{"upper_arm", upper_arm_mid},
                          std::pair{"forearm", forearm_mid}};
            for (const auto &[part, point]: parts) {
                Vec3 center = point;
                center[1] = sign * ARM_LATERAL_OFFSET_M;
                centers.emplace_back(std::string(part) + "_" +
                                             std::string(side),
                                     center);
            }
        }
        const std::array<std::pair<std::string_view, double>, 2> sides{
            std::pair{"front", -1.0}, std::pair{"rear", 1.0}};
        for (const auto &[side, sign]: sides) {
            const Vec3 &knee =
                    side == "front" ? pose.knee_front : pose.knee_rear;
            const Vec3 &ankle =
                    side == "front" ? pose.ankle_front : pose.ankle_rear;
            const Vec3 &pedal =
                    side == "front" ? pose.pedal_front : pose.pedal_rear;
            const std::array<std::pair<std::string_view, Vec3>, 3> parts{
                std::pair{"thigh", v_div(v_add(pose.hip, knee), 2.0)},
                std::pair{"shank", v_div(v_add(knee, ankle), 2.0)},
                std::pair{"foot", v_div(v_add(ankle, pedal), 2.0)}};
            for (const auto &[part, point]: parts) {
                Vec3 center = point;
                center[1] = sign * geometry::PEDAL_LATERAL_OFFSET_M;
                centers.emplace_back(std::string(part) + "_" +
                                             std::string(side),
                                     center);
            }
        }
        std::vector<RiderCenter> out;
        out.reserve(centers.size());
        for (const auto &[key, center]: centers)
            out.push_back(RiderCenter{
                    .name = "rider_" + key, .pos_m = center,
                    .mass_kg = [&] {
                        for (const auto &[name, value]: masses)
                            if (name == key) return value;
                        throw std::out_of_range(
                                py_str_repr(key));
                    }()});
        return out;
    }
    throw std::invalid_argument(
            "unknown rider variant " + py_str_repr(rider.variant) +
            "; available: none, lumped, articulated_planar");
}
std::vector<RiderCenter>
compute_rider_centers_of_mass(const RiderSpecs &rider) {
    return compute_rider_centers_of_mass(rider, BikeSpecs{});
}

// ---------------------------------------------------------------------
// rider_from_args / resolve_rider
// ---------------------------------------------------------------------
RiderSpecs rider_from_args(std::string_view variant,
                           // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
                           double mass_kg, double height_m,
                           std::optional<double> inseam_m) {
    RiderSpecs specs;
    specs.variant = std::string(variant);
    specs.mass_kg = mass_kg;
    specs.height_m = height_m;
    specs.inseam_m = inseam_m;
    configuration::detail::post_init(specs); // the dataclass __post_init__
    return specs;
}

RiderSpecs resolve_rider(const RiderArg &rider,
                         std::optional<bool> include_rider,
                         std::string_view default_variant) {
    if (const auto *specs = std::get_if<RiderSpecs>(&rider))
        return *specs;
    if (const auto *name = std::get_if<std::string>(&rider)) {
        RiderSpecs specs;
        specs.variant = *name;
        configuration::detail::post_init(specs);
        return specs;
    }
    // monostate == None.
    if (include_rider.has_value() && !*include_rider) {
        RiderSpecs specs;
        specs.variant = "none";
        configuration::detail::post_init(specs);
        return specs;
    }
    RiderSpecs specs;
    specs.variant = std::string(default_variant);
    configuration::detail::post_init(specs);
    return specs;
}

// ---------------------------------------------------------------------
// physics/rider_segments.py
// ---------------------------------------------------------------------
SegmentMasses segment_masses(double total_kg, double helmet_kg) {
    const double total = scalar(total_kg, "rider mass", std::nullopt, true);
    const double helmet = scalar(helmet_kg, "helmet mass", 0.0);
    if (helmet >= total)
        throw std::invalid_argument("helmet must be lighter than the rider");
    const double body = total - helmet;
    const auto f = [](std::string_view name) {
        return de_leva_mass_fraction(name);
    };
    SegmentMasses result{
        {"pelvis", f("trunk_lower") * body},
        {"torso", (f("trunk_upper") + f("trunk_middle")) * body},
        {"head", f("head") * body + helmet},
        {"upper_arm_pair", 2.0 * f("upper_arm") * body},
        {"forearm_pair", 2.0 * (f("forearm") + f("hand")) * body},
    };
    for (const std::string_view side: {"front", "rear"})
        for (const std::string_view segment: {"thigh", "shank", "foot"})
            result.emplace_back(std::string(segment) + "_" +
                                        std::string(side),
                                f(segment) * body);
    // sum over float products (f(segment) * body) — float items only,
    // so the compensated float branch is the oracle's whole path.
    numeric::PythonSum total_check;
    for (const auto &[name, value]: result) total_check.add(value);
    if (!checks::np_isclose_scalar(total_check.total(), total, 0.0,
                                   1e-10))
        throw std::runtime_error(
                "anatomical mass budget does not close"); // ArithmeticError
    return result;
}

SegmentMasses split_paired_arm_masses(SegmentMasses masses) {
    // dict(masses) then pop each *_pair and append its halves at tail.
    SegmentMasses result;
    result.reserve(masses.size() + 2);
    double upper_pair = 0.0, forearm_pair = 0.0;
    bool have_upper = false, have_forearm = false;
    for (auto &[name, value]: masses) {
        if (name == "upper_arm_pair") {
            upper_pair = value;
            have_upper = true;
        } else if (name == "forearm_pair") {
            forearm_pair = value;
            have_forearm = true;
        } else {
            result.emplace_back(name, value);
        }
    }
    // pop() order: upper_arm first, then forearm — each appends its two
    // halves at the dict tail.
    if (have_upper) {
        result.emplace_back("upper_arm_left", upper_pair / 2.0);
        result.emplace_back("upper_arm_right", upper_pair / 2.0);
    }
    if (have_forearm) {
        result.emplace_back("forearm_left", forearm_pair / 2.0);
        result.emplace_back("forearm_right", forearm_pair / 2.0);
    }
    return result;
}

Mat3 segment_inertia(double mass_kg, const Vec3 &axis_vector_m,
                     double radius_m) {
    const double mass = scalar(mass_kg, "segment mass", std::nullopt, true);
    // checks.array(axis, (3,)) — finite gate first.
    for (const double v: axis_vector_m)
        if (!std::isfinite(v))
            throw std::invalid_argument(
                    "invalid shape or non-finite segment axis");
    const double radius =
            scalar(radius_m, "segment radius", std::nullopt, true);
    const double length = gd::np_norm3(axis_vector_m);
    if (length <= 0)
        throw std::invalid_argument("segment length must be positive");
    const Vec3 unit = v_div(axis_vector_m, length);
    const double parallel = .5 * mass * py_pow(radius, 2.0);
    const double transverse =
            mass * (3.0 * py_pow(radius, 2.0) + py_pow(length, 2.0)) / 12.0;
    // transverse*np.eye(3) + (parallel-transverse)*np.outer(unit, unit)
    // — the multiply ufunc applies to the outer-product lane value, so
    // unit[i]*unit[j] rounds before (parallel-transverse) is applied.
    Mat3 out{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            out[i * 3 + j] =
                    transverse * (i == j ? 1.0 : 0.0) +
                    (parallel - transverse) * (unit[i] * unit[j]);
    return out;
}

void validate_articulated_pose(const ArticulatedPose &pose) {
    // __post_init__: every non-saddle field must be a finite 3-vector.
    const std::array<std::pair<std::string_view, const Vec3 *>, 11>
            fields{{
                {"hip", &pose.hip},
                {"shoulder", &pose.shoulder},
                {"elbow", &pose.elbow},
                {"grip", &pose.grip},
                {"head_center", &pose.head_center},
                {"knee_front", &pose.knee_front},
                {"knee_rear", &pose.knee_rear},
                {"ankle_front", &pose.ankle_front},
                {"ankle_rear", &pose.ankle_rear},
                {"pedal_front", &pose.pedal_front},
                {"pedal_rear", &pose.pedal_rear},
            }};
    for (const auto &[name, point]: fields)
        for (const double v: *point)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                        "invalid shape or non-finite " + std::string(name));
}

ArticulatedPose geometry_pose(const RiderSpecs &rider,
                              const BikeSpecs &specs) {
    // dataclasses.replace(rider, legs='rigid', saddle_share=.55,
    // pedal_share=.33, bar_share=.12) — a fresh validated record. The
    // Python original also forced variant='seated' to pass the pose
    // gate; the gate is gone with the variant, and the seated pose
    // solver now takes this record purely as geometry input.
    RiderSpecs neutral = rider;
    neutral.legs = "rigid";
    neutral.saddle_share = .55;
    neutral.pedal_share = .33;
    neutral.bar_share = .12;
    configuration::detail::post_init(neutral);
    const SeatedPose pose = seated_pose(neutral, specs);
    ArticulatedPose out{
            .saddle = pose.saddle,
            .hip = pose.hip,
            .shoulder = pose.shoulder,
            .elbow = pose.elbow,
            .grip = pose.grip,
            .head_center = pose.head_center,
            .knee_front = pose.knee_front,
            .knee_rear = pose.knee_rear,
            .ankle_front = pose.ankle_front,
            .ankle_rear = pose.ankle_rear,
            .pedal_front = pose.pedal_front,
            .pedal_rear = pose.pedal_rear,
    };
    validate_articulated_pose(out);
    return out;
}

NATIVE_DIAG_POP

} // namespace bike::model
