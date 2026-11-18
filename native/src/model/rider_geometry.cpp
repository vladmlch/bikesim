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

#include <algorithm>
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
    Vec3 ankle_rear{};

    const double l_thigh = rider.segment_length("thigh");
    const double l_shank = rider.segment_length("shank");
    const double l_trunk = rider.segment_length("trunk");
    const double l_ua = rider.segment_length("upper_arm");
    const double l_fa = rider.segment_length("forearm");
    const double l_hand = rider.segment_length("hand");

    Vec3 knee_front, knee_rear;
    double l2_front = 0.0;
    double s_par = 0.0;
    double s_perp = 0.0;
    try {
        knee_front =
                two_link_ik(hip, ankle_front, l_thigh, l_shank, "+x");
        const Vec3 front_leg_vec = v_sub(pedal_front, knee_front);
        l2_front = gd::np_norm3(front_leg_vec);
        knee_rear =
                two_link_ik(hip, pedal_rear, l_thigh, l2_front, "+x");
        const Vec3 rear_leg_vec = v_sub(pedal_rear, knee_rear);
        const Vec3 u_front = v_div(front_leg_vec, l2_front);
        const Vec3 u_rear = v_div(rear_leg_vec, l2_front);
        const Vec3 shank_front = v_sub(ankle_front, knee_front);
        s_par =
                shank_front[0] * u_front[0] + shank_front[2] * u_front[2];
        s_perp =
                -shank_front[0] * u_front[2] + shank_front[2] * u_front[0];
        ankle_rear = Vec3{
                knee_rear[0] + s_par * u_rear[0] - s_perp * u_rear[2],
                0.0,
                knee_rear[2] + s_par * u_rear[2] + s_perp * u_rear[0]};
    } catch (const std::invalid_argument &exc) {
        throw std::invalid_argument(
                std::string("the rider's legs do not reach the pedals "
                            "from the saddle: ") +
                exc.what());
    }

    const Vec3 pedal_bdc{0.0, 0.0, -crank_m};
    Vec3 knee_bdc;
    try {
        knee_bdc =
                two_link_ik(hip, pedal_bdc, l_thigh, l2_front, "+x");
    } catch (const std::invalid_argument &exc) {
        throw std::invalid_argument(
                std::string("the leg does not reach the pedal at bottom "
                            "dead centre (") +
                exc.what() + "); the saddle at " +
                gd::py_fixed(rider.saddle_height_m(), 3) +
                " m is too high for a " +
                gd::py_fixed(rider.height_m, 2) + " m rider");
    }
    const Vec3 bdc_leg_vec = v_sub(pedal_bdc, knee_bdc);
    const Vec3 u_bdc = v_div(bdc_leg_vec, l2_front);
    const Vec3 ankle_bdc{
            knee_bdc[0] + s_par * u_bdc[0] - s_perp * u_bdc[2],
            0.0,
            knee_bdc[2] + s_par * u_bdc[2] + s_perp * u_bdc[0]};
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

SeatedPose3d solve_seated_pose_3d(const BikeSpecs &specs,
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
    const Vec3 grip_center = geometry::handlebar_grip_point(specs);
    const double crank_m = specs.crank_length / 1000.0;
    const auto [pedal_front, pedal_rear] =
            geometry::pedal_points(crank_m);
    const Vec3 ankle_offset{0.0, 0.0, ANKLE_ABOVE_PEDAL_M};
    const Vec3 ankle_front = v_add(pedal_front, ankle_offset);
    Vec3 ankle_rear{};

    const double l_thigh = rider.segment_length("thigh");
    const double l_shank = rider.segment_length("shank");
    const double l_trunk = rider.segment_length("trunk");
    const double l_ua = rider.segment_length("upper_arm");
    const double l_fa = rider.segment_length("forearm");
    const double l_hand = rider.segment_length("hand");
    const double l_fore_grip = l_fa + GRIP_FROM_WRIST_FRACTION * l_hand;

    Vec3 knee_front, knee_rear;
    double l2_front = 0.0;
    double s_par = 0.0;
    double s_perp = 0.0;
    try {
        knee_front =
                two_link_ik(hip, ankle_front, l_thigh, l_shank, "+x");
        const Vec3 front_leg_vec = v_sub(pedal_front, knee_front);
        l2_front = gd::np_norm3(front_leg_vec);
        knee_rear =
                two_link_ik(hip, pedal_rear, l_thigh, l2_front, "+x");
        const Vec3 rear_leg_vec = v_sub(pedal_rear, knee_rear);
        const Vec3 u_front = v_div(front_leg_vec, l2_front);
        const Vec3 u_rear = v_div(rear_leg_vec, l2_front);
        const Vec3 shank_front = v_sub(ankle_front, knee_front);
        s_par =
                shank_front[0] * u_front[0] + shank_front[2] * u_front[2];
        s_perp =
                -shank_front[0] * u_front[2] + shank_front[2] * u_front[0];
        ankle_rear = Vec3{
                knee_rear[0] + s_par * u_rear[0] - s_perp * u_rear[2],
                0.0,
                knee_rear[2] + s_par * u_rear[2] + s_perp * u_rear[0]};
    } catch (const std::invalid_argument &exc) {
        throw std::invalid_argument(
                std::string("the rider's legs do not reach the pedals "
                            "from the saddle: ") +
                exc.what());
    }

    const Vec3 pedal_bdc{0.0, 0.0, -crank_m};
    Vec3 knee_bdc;
    try {
        knee_bdc =
                two_link_ik(hip, pedal_bdc, l_thigh, l2_front, "+x");
    } catch (const std::invalid_argument &exc) {
        throw std::invalid_argument(
                std::string("the leg does not reach the pedal at bottom "
                            "dead centre (") +
                exc.what() + "); the saddle at " +
                gd::py_fixed(rider.saddle_height_m(), 3) +
                " m is too high for a " +
                gd::py_fixed(rider.height_m, 2) + " m rider");
    }
    const Vec3 bdc_leg_vec = v_sub(pedal_bdc, knee_bdc);
    const Vec3 u_bdc = v_div(bdc_leg_vec, l2_front);
    const Vec3 ankle_bdc{
            knee_bdc[0] + s_par * u_bdc[0] - s_perp * u_bdc[2],
            0.0,
            knee_bdc[2] + s_par * u_bdc[2] + s_perp * u_bdc[0]};
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

    const double b_sh =
            SHOULDER_HALF_WIDTH_REF_M *
            (rider.height_m / SHOULDER_WIDTH_REF_STATURE_M);
    const double dy_arm = GRIP_LATERAL_OFFSET_3D_M - b_sh;
    const double elbow_rad = gd::np_radians(ELBOW_FLEXION_3D_DEG);
    const double d_3d = std::sqrt(
            l_ua * l_ua + l_fore_grip * l_fore_grip +
            2.0 * l_ua * l_fore_grip * numeric::math_cos(elbow_rad));
    if (!(d_3d > std::abs(dy_arm) + 0.05))
        throw std::invalid_argument("unreachable 3D cockpit reach");
    const double d_xz = std::sqrt(d_3d * d_3d - dy_arm * dy_arm);

    Vec3 shoulder_center;
    try {
        shoulder_center = two_link_ik(hip, grip_center, l_trunk, d_xz, "+z");
    } catch (const std::invalid_argument &exc) {
        throw std::invalid_argument(
                std::string("the rider cannot reach the handlebar from "
                            "the saddle: ") +
                exc.what());
    }

    const Vec3 torso_axis_raw = v_sub(shoulder_center, hip);
    const Vec3 torso_axis =
            v_div(torso_axis_raw, gd::np_norm3(torso_axis_raw));
    const double lo =
            torso_axis[2] < 1.0 ? torso_axis[2] : 1.0;
    const double torso_lean = gd::np_degrees(
            std::acos(lo > -1.0 ? lo : -1.0));
    const Vec3 head_center =
            v_add(shoulder_center,
                  v_scale(NECK_LENGTH_M + HEAD_RADIUS_M, torso_axis));

    const auto solve_side = [&](double side_sign) {
        const Vec3 sh{shoulder_center[0], side_sign * b_sh,
                      shoulder_center[2]};
        const Vec3 gr{grip_center[0], side_sign * GRIP_LATERAL_OFFSET_3D_M,
                      grip_center[2]};
        const Vec3 dp = v_sub(gr, sh);
        const double psi0 = std::atan2(dp[1], dp[0]);
        const double rho0 = std::hypot(dp[0], dp[1]);
        const Vec3 e_rho{numeric::math_cos(psi0), numeric::math_sin(psi0), 0.0};
        const double gamma0 = std::atan2(-dp[2], rho0);
        const double cos_alpha0 = std::clamp(
                (l_ua * l_ua + d_3d * d_3d - l_fore_grip * l_fore_grip) /
                        (2.0 * l_ua * d_3d),
                -1.0, 1.0);
        const double alpha0 = std::acos(cos_alpha0);
        const double beta_ua0 = gamma0 + alpha0;
        const Vec3 el = v_add(
                sh,
                v_scale(l_ua,
                        v_sub(v_scale(numeric::math_cos(beta_ua0), e_rho),
                              v_scale(numeric::math_sin(beta_ua0),
                                      Vec3{0.0, 0.0, 1.0}))));
        const Vec3 to_fore_raw = v_sub(gr, el);
        const Vec3 to_fore = v_div(to_fore_raw, gd::np_norm3(to_fore_raw));
        const Vec3 wr = v_sub(
                gr, v_scale(GRIP_FROM_WRIST_FRACTION * l_hand, to_fore));
        return std::tuple{sh, el, wr, gr, psi0, beta_ua0};
    };

    const auto [sh_l, el_l, wr_l, gr_l, psi0_l, beta_ua0_l] =
            solve_side(+1.0);
    const auto [sh_r, el_r, wr_r, gr_r, psi0_r, beta_ua0_r] =
            solve_side(-1.0);

    const SeatedPose sagittal{
            .saddle = saddle,
            .hip = hip,
            .shoulder = shoulder_center,
            .elbow = Vec3{el_l[0], 0.0, el_l[2]},
            .wrist = Vec3{wr_l[0], 0.0, wr_l[2]},
            .grip = grip_center,
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

    return SeatedPose3d{
            .sagittal = sagittal,
            .shoulder_left = sh_l,
            .shoulder_right = sh_r,
            .elbow_left = el_l,
            .elbow_right = el_r,
            .wrist_left = wr_l,
            .wrist_right = wr_r,
            .grip_left = gr_l,
            .grip_right = gr_r,
            .b_shoulder_m = b_sh,
            .psi0_left_rad = psi0_l,
            .psi0_right_rad = psi0_r,
            .beta_ua0_rad = beta_ua0_l,
            .elbow_flexion0_rad = elbow_rad,
            .upper_arm_m = l_ua,
            .forearm_grip_m = l_fore_grip,
    };
}

SeatedPose3d seated_pose_3d(const RiderSpecs &rider, const BikeSpecs &specs) {
    return solve_seated_pose_3d(specs, rider);
}
SeatedPose3d seated_pose_3d(const RiderSpecs &rider) {
    return seated_pose_3d(rider, BikeSpecs{});
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
    if (rider.variant == "articulated_3d") {
        const ArticulatedPose3d pose = geometry_pose_3d(rider, specs);
        const SegmentMasses masses = split_paired_arm_masses(
                segment_masses(rider.mass_kg, rider.helmet_mass_kg));
        std::vector<std::pair<std::string, Vec3>> centers;
        centers.emplace_back("pelvis", v_add(pose.hip, {0.0, 0.0, 0.06}));
        centers.emplace_back("torso",
                             v_div(v_add(pose.hip, pose.shoulder_center), 2.0));
        centers.emplace_back("head", pose.head_center);
        centers.emplace_back(
                "upper_arm_left",
                v_div(v_add(pose.shoulder_left, pose.elbow_left), 2.0));
        centers.emplace_back(
                "forearm_left",
                v_div(v_add(pose.elbow_left, pose.grip_left), 2.0));
        centers.emplace_back(
                "upper_arm_right",
                v_div(v_add(pose.shoulder_right, pose.elbow_right), 2.0));
        centers.emplace_back(
                "forearm_right",
                v_div(v_add(pose.elbow_right, pose.grip_right), 2.0));
        const std::array<std::pair<std::string_view, double>, 2> sides{
            std::pair{"front", -1.0}, std::pair{"rear", 1.0}};
        for (const auto &[side, sign] : sides) {
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
            for (const auto &[part, point] : parts) {
                Vec3 center = point;
                center[1] = sign * geometry::PEDAL_LATERAL_OFFSET_M;
                centers.emplace_back(
                        std::string(part) + "_" + std::string(side), center);
            }
        }
        std::vector<RiderCenter> out;
        out.reserve(centers.size());
        for (const auto &[key, center] : centers)
            out.push_back(RiderCenter{
                .name = "rider_" + key,
                .pos_m = center,
                .mass_kg = [&] {
                    for (const auto &[name, value] : masses)
                        if (name == key) return value;
                    throw std::out_of_range(py_str_repr(key));
                }()});
        return out;
    }
    throw std::invalid_argument(
            "unknown rider variant " + py_str_repr(rider.variant) +
            "; available: none, lumped, articulated_planar, articulated_3d");
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
    // A fresh validated record, as the Python original's
    // dataclasses.replace(...) produced; the seated pose solver takes it
    // purely as geometry input.
    RiderSpecs checked = rider;
    configuration::detail::post_init(checked);
    const SeatedPose pose = seated_pose(checked, specs);
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

void validate_articulated_pose_3d(const ArticulatedPose3d &pose) {
    const std::array<std::pair<std::string_view, const Vec3 *>, 15> fields{{
        {"hip", &pose.hip},
        {"shoulder_center", &pose.shoulder_center},
        {"head_center", &pose.head_center},
        {"shoulder_left", &pose.shoulder_left},
        {"shoulder_right", &pose.shoulder_right},
        {"elbow_left", &pose.elbow_left},
        {"elbow_right", &pose.elbow_right},
        {"grip_left", &pose.grip_left},
        {"grip_right", &pose.grip_right},
        {"knee_front", &pose.knee_front},
        {"knee_rear", &pose.knee_rear},
        {"ankle_front", &pose.ankle_front},
        {"ankle_rear", &pose.ankle_rear},
        {"pedal_front", &pose.pedal_front},
        {"pedal_rear", &pose.pedal_rear},
    }};
    for (const auto &[name, point] : fields)
        for (const double v : *point)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                        "invalid shape or non-finite " + std::string(name));
}

ArticulatedPose to_articulated_pose(const ArticulatedPose3d &pose) {
    return ArticulatedPose{
        .saddle = pose.saddle,
        .hip = pose.hip,
        .shoulder = pose.shoulder_center,
        .elbow = Vec3{pose.elbow_left[0], 0.0, pose.elbow_left[2]},
        .grip = Vec3{pose.grip_left[0], 0.0, pose.grip_left[2]},
        .head_center = pose.head_center,
        .knee_front = pose.knee_front,
        .knee_rear = pose.knee_rear,
        .ankle_front = pose.ankle_front,
        .ankle_rear = pose.ankle_rear,
        .pedal_front = pose.pedal_front,
        .pedal_rear = pose.pedal_rear,
    };
}

ArticulatedPose3d geometry_pose_3d(const RiderSpecs &rider,
                                   const BikeSpecs &specs) {
    RiderSpecs checked = rider;
    configuration::detail::post_init(checked);
    const SeatedPose3d pose = seated_pose_3d(checked, specs);
    ArticulatedPose3d out{
        .saddle = pose.sagittal.saddle,
        .hip = pose.sagittal.hip,
        .shoulder_center = pose.sagittal.shoulder,
        .head_center = pose.sagittal.head_center,
        .shoulder_left = pose.shoulder_left,
        .shoulder_right = pose.shoulder_right,
        .elbow_left = pose.elbow_left,
        .elbow_right = pose.elbow_right,
        .grip_left = pose.grip_left,
        .grip_right = pose.grip_right,
        .knee_front = pose.sagittal.knee_front,
        .knee_rear = pose.sagittal.knee_rear,
        .ankle_front = pose.sagittal.ankle_front,
        .ankle_rear = pose.sagittal.ankle_rear,
        .pedal_front = pose.sagittal.pedal_front,
        .pedal_rear = pose.sagittal.pedal_rear,
        .b_shoulder_m = pose.b_shoulder_m,
        .psi0_left_rad = pose.psi0_left_rad,
        .psi0_right_rad = pose.psi0_right_rad,
        .beta_ua0_rad = pose.beta_ua0_rad,
        .elbow_flexion0_rad = pose.elbow_flexion0_rad,
        .upper_arm_m = pose.upper_arm_m,
        .forearm_grip_m = pose.forearm_grip_m,
    };
    validate_articulated_pose_3d(out);
    return out;
}

namespace {
[[nodiscard]] double rot_y_angle(const Vec3 &u, const Vec3 &v) noexcept {
    return std::atan2(u[2] * v[0] - u[0] * v[2],
                      u[0] * v[0] + u[2] * v[2]);
}
[[nodiscard]] double rot_m_angle(const Vec3 &u, const Vec3 &v,
                                 double psi0_rad) noexcept {
    const double c = numeric::math_cos(psi0_rad);
    const double s = numeric::math_sin(psi0_rad);
    const double u_rho = u[0] * c + u[1] * s;
    const double v_rho = v[0] * c + v[1] * s;
    return std::atan2(u[2] * v_rho - u_rho * v[2],
                      u_rho * v_rho + u[2] * v[2]);
}
} // namespace

double pose_neutral_anatomical_rad(const ArticulatedPose &pose,
                                   std::string_view joint_name) {
    if (joint_name == "rider_torso_hinge")
        return +rot_y_angle({0.0, 0.0, 1.0}, v_sub(pose.shoulder, pose.hip));
    if (joint_name == "rider_shoulder_left" ||
        joint_name == "rider_shoulder_right" ||
        joint_name == "rider_shoulder")
        return -rot_y_angle(v_sub(pose.hip, pose.shoulder),
                            v_sub(pose.elbow, pose.shoulder));
    if (joint_name == "rider_elbow_left" ||
        joint_name == "rider_elbow_right" ||
        joint_name == "rider_elbow")
        return -rot_y_angle(v_sub(pose.elbow, pose.shoulder),
                            v_sub(pose.grip, pose.elbow));
    if (joint_name == "rider_hip_front")
        return -rot_y_angle({0.0, 0.0, -1.0},
                            v_sub(pose.knee_front, pose.hip));
    if (joint_name == "rider_knee_front")
        return +rot_y_angle(v_sub(pose.knee_front, pose.hip),
                            v_sub(pose.ankle_front, pose.knee_front));
    if (joint_name == "rider_ankle_front")
        return +rot_y_angle(v_sub(pose.knee_front, pose.ankle_front),
                            v_sub(pose.pedal_front, pose.ankle_front));
    if (joint_name == "rider_hip_rear")
        return -rot_y_angle({0.0, 0.0, -1.0},
                            v_sub(pose.knee_rear, pose.hip));
    if (joint_name == "rider_knee_rear")
        return +rot_y_angle(v_sub(pose.knee_rear, pose.hip),
                            v_sub(pose.ankle_rear, pose.knee_rear));
    if (joint_name == "rider_ankle_rear")
        return +rot_y_angle(v_sub(pose.knee_rear, pose.ankle_rear),
                            v_sub(pose.pedal_rear, pose.ankle_rear));
    throw std::invalid_argument("unknown rider joint: " +
                                std::string(joint_name));
}

double pose_neutral_anatomical_rad(const SeatedPose &pose,
                                   std::string_view joint_name) {
    const ArticulatedPose articulated{
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
    return pose_neutral_anatomical_rad(articulated, joint_name);
}

configuration::JointEnvelopeMap
with_pose_neutral_angles(configuration::JointEnvelopeMap envelopes,
                         const ArticulatedPose &pose) {
    for (auto &[name, env] : envelopes) {
        env.neutral_anatomical_rad = pose_neutral_anatomical_rad(pose, name);
        configuration::detail::post_init(env);
        if (!(env.minimum_anatomical_rad <= env.neutral_anatomical_rad &&
              env.neutral_anatomical_rad <= env.maximum_anatomical_rad))
            throw std::invalid_argument(
                name + ": neutral angle outside the declared range");
    }
    return envelopes;
}

double pose_neutral_anatomical_rad(const ArticulatedPose3d &pose,
                                   std::string_view joint_name) {
    if (joint_name == "rider_torso_hinge")
        return +rot_y_angle({0.0, 0.0, 1.0},
                            v_sub(pose.shoulder_center, pose.hip));
    if (joint_name == "rider_shoulder_abduction_left" ||
        joint_name == "rider_shoulder_abduction_right" ||
        joint_name == "rider_hip_abduction_front" ||
        joint_name == "rider_hip_abduction_rear" ||
        joint_name == "rider_saddle_pitch")
        return 0.0;
    if (joint_name == "rider_shoulder_left")
        return -rot_m_angle(v_sub(pose.hip, pose.shoulder_center),
                            v_sub(pose.elbow_left, pose.shoulder_left),
                            pose.psi0_left_rad);
    if (joint_name == "rider_shoulder_right")
        return -rot_m_angle(v_sub(pose.hip, pose.shoulder_center),
                            v_sub(pose.elbow_right, pose.shoulder_right),
                            pose.psi0_right_rad);
    if (joint_name == "rider_elbow_left")
        return -rot_m_angle(v_sub(pose.elbow_left, pose.shoulder_left),
                            v_sub(pose.grip_left, pose.elbow_left),
                            pose.psi0_left_rad);
    if (joint_name == "rider_elbow_right")
        return -rot_m_angle(v_sub(pose.elbow_right, pose.shoulder_right),
                            v_sub(pose.grip_right, pose.elbow_right),
                            pose.psi0_right_rad);
    if (joint_name == "rider_hip_front")
        return -rot_y_angle({0.0, 0.0, -1.0},
                            v_sub(pose.knee_front, pose.hip));
    if (joint_name == "rider_knee_front")
        return +rot_y_angle(v_sub(pose.knee_front, pose.hip),
                            v_sub(pose.ankle_front, pose.knee_front));
    if (joint_name == "rider_ankle_front")
        return +rot_y_angle(v_sub(pose.knee_front, pose.ankle_front),
                            v_sub(pose.pedal_front, pose.ankle_front));
    if (joint_name == "rider_hip_rear")
        return -rot_y_angle({0.0, 0.0, -1.0},
                            v_sub(pose.knee_rear, pose.hip));
    if (joint_name == "rider_knee_rear")
        return +rot_y_angle(v_sub(pose.knee_rear, pose.hip),
                            v_sub(pose.ankle_rear, pose.knee_rear));
    if (joint_name == "rider_ankle_rear")
        return +rot_y_angle(v_sub(pose.knee_rear, pose.ankle_rear),
                            v_sub(pose.pedal_rear, pose.ankle_rear));
    throw std::invalid_argument("unknown rider joint: " +
                                std::string(joint_name));
}

configuration::JointEnvelopeMap
with_pose_neutral_angles(configuration::JointEnvelopeMap envelopes,
                         const ArticulatedPose3d &pose) {
    for (auto &[name, env] : envelopes) {
        env.neutral_anatomical_rad = pose_neutral_anatomical_rad(pose, name);
        configuration::detail::post_init(env);
        if (!(env.minimum_anatomical_rad <= env.neutral_anatomical_rad &&
              env.neutral_anatomical_rad <= env.maximum_anatomical_rad))
            throw std::invalid_argument(
                name + ": neutral angle outside the declared range");
    }
    return envelopes;
}

NATIVE_DIAG_POP

} // namespace bike::model
