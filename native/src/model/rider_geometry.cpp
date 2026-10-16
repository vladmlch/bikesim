// model/rider_geometry.cpp — T15 rider boundary implementations.
//
// numpy-provider notes kept inline at each site. Scalar `x ** y` goes
// through pyfloat::float_pow (CPython float_pow's complete ladder); the
// complex ops of saddle_path_apparent_mass reproduce NumPy's ufunc
// kernels bitwise: multiply = {fma(ar,br,-(ai*bi)), fma(ar,bi,ai*br)}
// (the ARM64 NEON simd_cmul_f64 lane form the oracle's contiguous
// complex128 loop reaches), divide = the scalar Smith-scaled
// CDOUBLE_divide body, absolute = npy_cabs == hypot(re, im). Same-angle
// sin/cos pairs route through numeric::sin/cos so the optimizer cannot
// merge them into a __sincos_stret the oracle never calls.
#include "rider_geometry.hpp"

#include "../diag.hpp"
#include "../numeric_sincos.hpp"
#include "../numeric_sum.hpp"
#include "../writers/pyfloat.hpp"
#include "checks.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
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

using bike::model::LegChain;
using bike::model::Mat3;
using bike::model::RiderBody;
using bike::model::RiderGeom;
using bike::model::SeatedPose;
using bike::model::Vec3;
namespace gd = bike::geometry::detail;
using bike::model::checks::boolean;
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
    for (const char raw_c: s) {
        const auto c = static_cast<unsigned char>(raw_c);
        if (c == static_cast<unsigned char>(quote) || c == '\\') {
            out.push_back('\\');
            out.push_back(static_cast<char>(c));
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
    }
    out.push_back(quote);
    return out;
}

// ---------------------------------------------------------------------
// Complex ufunc kernels — see the header's provider note. A Cpx value is
// the (re, im) pair NumPy stores for one complex128 lane.
struct Cpx {
    double re, im;
};
[[nodiscard]] inline Cpx cadd(Cpx a, Cpx b) noexcept {
    return Cpx{.re = a.re + b.re, .im = a.im + b.im};
}
[[nodiscard]] inline Cpx csub(Cpx a, Cpx b) noexcept {
    return Cpx{.re = a.re - b.re, .im = a.im - b.im};
}
// CDOUBLE multiply — simd_cmul_f64: the (ai*bi)/(ai*br) products are
// rounded first, then fused into fma(ar,br,-p)/fma(ar,bi,q).
[[nodiscard]] inline Cpx cmul(Cpx a, Cpx b) noexcept {
    return Cpx{.re = std::fma(a.re, b.re, -(a.im * b.im)),
               .im = std::fma(a.re, b.im, a.im * b.re)};
}
// CDOUBLE_divide — loops.c.src's scalar Smith-scaled body (the divide
// ufunc has no complex SIMD dispatch). The oracle's build contracts
// every `x ± y*rat` term to an AArch64 fma, so each fused form is
// spelled out here — the build uses -ffp-contract=off, meaning plain
// expressions would round the product first and drift ~1 ulp.
[[nodiscard]] inline Cpx cdiv(Cpx a, Cpx b) noexcept {
    const double abs_br = std::fabs(b.re), abs_bi = std::fabs(b.im);
    if (abs_br >= abs_bi) {
        if (abs_br == 0.0 && abs_bi == 0.0)
            return Cpx{.re = a.re / abs_br, .im = a.im / abs_bi};
        const double rat = b.im / b.re;
        const double scl =
                1.0 / std::fma(b.im, rat, b.re); // br + bi*rat
        return Cpx{.re = std::fma(a.im, rat, a.re) * scl, // ar + ai*rat
                   .im = std::fma(-a.re, rat, a.im) * scl}; // ai - ar*rat
    }
    const double rat = b.re / b.im;
    const double scl =
            1.0 / std::fma(b.re, rat, b.im); // bi + br*rat
    return Cpx{.re = std::fma(a.re, rat, a.im) * scl, // ar*rat + ai
               .im = std::fma(a.im, rat, -a.re) * scl}; // ai*rat - ar
}
// CDOUBLE_absolute — simd_cabsolute_f64's scaled-hypot lane form
// (loops_unary_complex.dispatch.c.src), not libm hypot: the
// inf/nan normalisation runs first, then larger·sqrt(ratio² + 1) with
// the fma product — this differs from hypot by ~1 ulp on unlucky
// inputs, so the oracle's bitwise value requires this exact form.
// npyv_max/fmin on AArch64 map to fmax/fmin (NaN-returning-the-numeric
// operand); the normalisation above already spread NaN into both
// lanes, so fmax/fmin give the same result.
[[nodiscard]] inline double cabs(Cpx z) noexcept {
    double re = std::fabs(z.re), im = std::fabs(z.im);
    constexpr double inf = std::numeric_limits<double>::infinity();
    if (re == inf) im = inf;
    if (im == inf) re = inf;
    if (std::isnan(re)) im = std::numeric_limits<double>::quiet_NaN();
    if (std::isnan(im)) re = std::numeric_limits<double>::quiet_NaN();
    const double larger = std::fmax(re, im);
    const double smaller = std::fmin(re, im);
    const double ratio =
            (larger != 0.0 && smaller != inf) ? smaller / larger : 0.0;
    return std::sqrt(std::fma(ratio, ratio, 1.0)) * larger;
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
constexpr std::array kSegmentRadii{
    NameValue{.name = "torso", .value = 0.110},
    NameValue{.name = "pelvis", .value = 0.085},
    NameValue{.name = "thigh", .value = 0.070},
    NameValue{.name = "shank", .value = 0.055},
    NameValue{.name = "foot", .value = 0.035},
    NameValue{.name = "upper_arm", .value = 0.045},
    NameValue{.name = "forearm", .value = 0.040},
};

[[nodiscard]] double dict_lookup(std::span<const NameValue> table,
                                 std::string_view name) {
    for (const NameValue &row: table)
        if (row.name == name) return row.value;
    throw std::out_of_range(py_str_repr(name)); // KeyError payload
}

// masses dict lookups used inside solve_seated_pose — linear scans over
// the nine-key ordered table.
[[nodiscard]] double mass_of(
        const std::vector<std::pair<std::string, double>> &masses,
        std::string_view key) {
    for (const auto &[name, value]: masses)
        if (name == key) return value;
    throw std::out_of_range(py_str_repr(key));
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
double segment_radius_m(std::string_view name) {
    return dict_lookup(std::span<const NameValue>{kSegmentRadii}, name);
}
bool has_segment_radius(std::string_view name) noexcept {
    for (const NameValue &entry: kSegmentRadii)
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
// RiderBody
// ---------------------------------------------------------------------
void validate_rider_body(const RiderBody &body) {
    scalar(body.stiffness_n_m, "RiderBody.stiffness_n_m", 0.0);
    scalar(body.damping_ns_m, "RiderBody.damping_ns_m", 0.0);
    scalar(body.supported_mass_kg, "RiderBody.supported_mass_kg", 0.0);
    boolean(body.unilateral, "RiderBody.unilateral");
}

double RiderBody::mass() const {
    // sum(g.mass for g in geoms) — builtin float accumulation.
    numeric::PythonSum total;
    for (const RiderGeom &g: geoms) total.add(g.mass);
    return total.total();
}

Vec3 RiderBody::center_of_mass() const {
    // sum(g.mass * g.center) has the default start=0: the first item
    // becomes int-0 + ndarray -> x + 0.0 per lane, then sequential adds.
    const double total = mass();
    if (geoms.empty())
        // int 0 / float -> ZeroDivisionError("float division by zero").
        throw std::runtime_error("float division by zero");
    Vec3 acc{0.0, 0.0, 0.0};
    bool first = true;
    for (const RiderGeom &g: geoms) {
        const Vec3 c = g.center();
        for (std::size_t i = 0; i < 3; ++i) {
            const double item = g.mass * c[i];
            acc[i] = first ? item + 0.0 : acc[i] + item;
        }
        first = false;
    }
    return v_div(acc, total);
}

double RiderBody::preload_n() const {
    scalar(supported_mass_kg, "RiderBody.supported_mass_kg", 0.0);
    const double preload = supported_mass_kg * configuration::kGravityMps2;
    checks::derived(preload, "RiderBody.preload_n");
    return preload;
}

double RiderBody::preload_deflection_m() const {
    const double stiffness = stiffness_n_m;
    scalar(stiffness, "RiderBody.stiffness_n_m", 0.0);
    const double preload = preload_n();
    if (stiffness == 0.) {
        if (preload != 0.)
            throw std::invalid_argument(
                    "RiderBody.stiffness_n_m: zero stiffness cannot "
                    "carry preload");
        return 0.;
    }
    const double deflection = preload / stiffness;
    checks::derived(deflection, "RiderBody.preload_deflection_m");
    return deflection;
}

// ---------------------------------------------------------------------
// SeatedPose
// ---------------------------------------------------------------------
const RiderBody &SeatedPose::body(std::string_view name) const {
    for (const RiderBody &b: bodies)
        if (b.name == name) return b;
    // Python raises KeyError(f"no rider body named {name!r}"): the
    // payload callers read through str(exc) is repr() of the formatted
    // message (double-quoted because it embeds a single-quoted name).
    throw std::out_of_range(
            py_str_repr("no rider body named " + py_str_repr(name)));
}

double SeatedPose::total_mass_kg() const {
    // float(sum(body masses) + sum(chain segment sums)) — two builtin
    // float accumulations combined with a plain add.
    numeric::PythonSum body_total;
    for (const RiderBody &b: bodies) body_total.add(b.mass());
    numeric::PythonSum chain_total;
    for (const LegChain &c: leg_chains)
        chain_total.add(c.thigh_mass_kg + c.shank_mass_kg +
                        c.foot_mass_kg);
    return body_total.total() + chain_total.total();
}

std::vector<std::pair<std::string, double>>
SeatedPose::interface_loads_n() const {
    // loads = {"saddle": 0.0, "pedals": 0.0, "bar": 0.0} — dict order.
    // The local names shadow the pose record's point members — suffix
    // them like the oracle's dict keys.
    double saddle_n = 0.0, pedals_n = 0.0, bar_n = 0.0;
    for (const RiderBody &b: bodies) {
        if (b.parent != "frame") continue;
        if (b.interface == "saddle") saddle_n += b.preload_n();
        else if (b.interface == "pedals") pedals_n += b.preload_n();
        else if (b.interface == "bar") bar_n += b.preload_n();
        else
            throw std::out_of_range(py_str_repr(b.interface)); // dict miss
    }
    return {{"saddle", saddle_n}, {"pedals", pedals_n}, {"bar", bar_n}};
}

std::vector<std::pair<std::string, double>>
SeatedPose::interface_shares() const {
    const auto loads = interface_loads_n();
    const double weight = total_mass_kg() * configuration::kGravityMps2;
    std::vector<std::pair<std::string, double>> out;
    out.reserve(loads.size());
    for (const auto &[name, v]: loads) {
        if (weight == 0.0)
            throw std::runtime_error("float division by zero");
        out.emplace_back(name, v / weight);
    }
    return out;
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

std::pair<double, double> spring_params(
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double mass_kg, double resonance_hz, double damping_ratio) {
    const double k = mass_kg *
                     py_pow(2.0 * M_PI * resonance_hz, 2.0);
    const double c = 2.0 * damping_ratio * std::sqrt(k * mass_kg);
    return {k, c};
}

std::vector<std::pair<std::string, double>>
seated_path_masses(const RiderSpecs &rider) {
    const auto &f = [](std::string_view name) {
        return de_leva_mass_fraction(name);
    };
    const double body = rider.mass_kg - rider.helmet_mass_kg;
    const double head = f("head") * body + rider.helmet_mass_kg;
    const double trunk_upper_mid =
            (f("trunk_upper") + f("trunk_middle")) * body;
    // The oracle computes trunk_lower but never reads it (dead in the
    // source) — keep the identical evaluation.
    [[maybe_unused]] const double trunk_lower = f("trunk_lower") * body;
    const double arms =
            2.0 * (f("upper_arm") + f("forearm") + f("hand")) * body;
    const double thighs = 2.0 * f("thigh") * body;
    const double shanks_feet = 2.0 * (f("shank") + f("foot")) * body;

    const double bar_path = rider.bar_share * rider.mass_kg;
    const double pedal_path = rider.pedal_share * rider.mass_kg;
    const double saddle_path = rider.saddle_share * rider.mass_kg;

    const double trunk_to_arms = bar_path - arms;
    const double thigh_to_legs = pedal_path - shanks_feet;
    if (trunk_to_arms < 0.0)
        throw std::invalid_argument(
                "bar share " + gd::py_fixed(rider.bar_share, 2) +
                " puts " + gd::py_fixed(bar_path, 1) +
                " kg on the bar, less than the arms' " +
                gd::py_fixed(arms, 1) + " kg");
    if (thigh_to_legs < 0.0 || thigh_to_legs > thighs)
        throw std::invalid_argument(
                "pedal share " + gd::py_fixed(rider.pedal_share, 2) +
                " needs " + gd::py_fixed(thigh_to_legs, 1) +
                " kg of thigh on the pedals; the shanks and feet weigh " +
                gd::py_fixed(shanks_feet, 1) + " kg and the thighs " +
                gd::py_fixed(thighs, 1) + " kg");
    const double torso = trunk_upper_mid + head - trunk_to_arms;
    const double pelvis = saddle_path - torso;
    if (pelvis <= 0.0)
        throw std::invalid_argument(
                "saddle share " + gd::py_fixed(rider.saddle_share, 2) +
                " leaves " + gd::py_fixed(pelvis, 1) +
                " kg for the pelvis");
    return {
        {"torso", torso},
        {"pelvis", pelvis},
        {"arms", bar_path},
        {"leg", pedal_path / 2.0},
        {"head", head},
        {"trunk_to_arms", trunk_to_arms},
        {"thigh_to_legs", thigh_to_legs / 2.0},
        {"shank_foot", shanks_feet / 2.0},
        {"upper_arm_fraction",
         f("upper_arm") / (f("upper_arm") + f("forearm") + f("hand"))},
    };
}

double pitch_xz(const Vec3 &vec) {
    return std::atan2(-vec[2], vec[0]);
}

Vec3 pedal_spindle_pos(double phase_rad, double lateral_y_m,
                       double crank_len_m) {
    // cos/sin share the phase operand — keep two separate libm calls.
    return {crank_len_m * numeric::cos(phase_rad), lateral_y_m,
            -crank_len_m * numeric::sin(phase_rad)};
}

std::pair<Vec3, Vec3> solve_leg_joints(
        const LegChain &chain,
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double phase_rad, double pelvis_z_m) {
    Vec3 ankle = pedal_spindle_pos(phase_rad, chain.lateral_y_m,
                                   chain.crank_len_m);
    ankle[2] += ANKLE_ABOVE_PEDAL_M;
    const Vec3 hip = v_add(chain.hip, {0.0, 0.0, pelvis_z_m});
    const Vec3 knee = two_link_ik(hip, ankle, chain.thigh_len_m,
                                  chain.shank_len_m, "+x");
    return {knee, ankle};
}

Vec3 solve_leg_qpos(const LegChain &chain, double phase_rad,
                  double pelvis_z_m) {
    const auto [knee, ankle] =
            solve_leg_joints(chain, phase_rad, pelvis_z_m);
    const Vec3 hip = v_add(chain.hip, {0.0, 0.0, pelvis_z_m});
    const Vec3 pedal = pedal_spindle_pos(phase_rad, chain.lateral_y_m,
                                         chain.crank_len_m);
    const double pitch_t = pitch_xz(v_sub(knee, hip));
    const double pitch_s = pitch_xz(v_sub(ankle, knee));
    const double pitch_f = pitch_xz(v_sub(pedal, ankle));
    Vec3 q{};
    q[0] = pitch_t - chain.pitch0_thigh_rad;
    q[1] = (pitch_s - chain.pitch0_shank_rad) - q[0];
    q[2] = (pitch_f - chain.pitch0_foot_rad) - (q[0] + q[1]);
    return q;
}

LegFkResult fk_leg(const LegChain &chain, const Vec3 &qpos,
                   double pelvis_z_m) {
    const Vec3 hip = v_add(chain.hip, {0.0, 0.0, pelvis_z_m});
    const double p_t = chain.pitch0_thigh_rad + qpos[0];
    const Vec3 knee =
            v_add(hip, {chain.thigh_len_m * numeric::cos(p_t), 0.0,
                        chain.thigh_len_m * -numeric::sin(p_t)});
    const double p_s =
            chain.pitch0_shank_rad + qpos[0] + qpos[1];
    const Vec3 ankle =
            v_add(knee, {chain.shank_len_m * numeric::cos(p_s), 0.0,
                         chain.shank_len_m * -numeric::sin(p_s)});
    // qpos.sum() — the 3-element pairwise lane (-0.0 seed + fold).
    const double p_f =
            chain.pitch0_foot_rad +
            numeric::numpy_pairwise_sum(
                    std::span<const double>(qpos.data(), qpos.size()));
    const double foot_len = gd::np_norm3(v_sub(chain.pedal, chain.ankle));
    const Vec3 pedal_pt =
            v_add(ankle, {foot_len * numeric::cos(p_f), 0.0,
                          foot_len * -numeric::sin(p_f)});
    return LegFkResult{.knee = knee, .ankle = ankle,
                       .pedal_pt = pedal_pt};
}

Mat32 leg_jacobian(const LegChain &chain, const Vec3 &qpos,
                   double pelvis_z_m) {
    const Vec3 hip = v_add(chain.hip, {0.0, 0.0, pelvis_z_m});
    const LegFkResult fk = fk_leg(chain, qpos, pelvis_z_m);
    const std::array<Vec3, 3> joints{hip, fk.knee, fk.ankle};
    Mat32 J{};
    for (std::size_t i = 0; i < 3; ++i) {
        const Vec3 r = v_sub(fk.pedal_pt, joints[i]);
        J[i] = {r[2], -r[0]};
    }
    return J;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
double crank_torque_share(double phase_rad, double depth) {
    // max(0.0, cos(phase)) — Python max returns cos only when cos > 0.
    const double c = std::cos(phase_rad);
    return 0.5 * (1.0 - depth) +
           depth * (M_PI / 2.0) * (c > 0.0 ? c : 0.0);
}

// ---------------------------------------------------------------------
// solve_seated_pose
// ---------------------------------------------------------------------
namespace {

[[nodiscard]] RiderGeom make_geom(std::string name, std::string kind,
                                  Vec3 p1, Vec3 p2, double radius,
                                  double mass) {
    return RiderGeom{.name = std::move(name), .kind = std::move(kind),
                     .p1 = p1, .p2 = p2, .radius = radius, .mass = mass};
}

} // namespace

// solve_seated_pose's frame is ~8.6 KB — the oracle's pose dict maps to
// dozens of point/segment locals that must stay named for the verbatim
// expression order; the 8 KB guard keeps covering the tighter code
// elsewhere (same convention as the binding TUs).
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
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
                      2.0 * l_ua * l_fa * std::cos(elbow_rad));
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

    const auto masses = seated_path_masses(rider);
    const double shank_foot_fraction =
            de_leva_mass_fraction("shank") + de_leva_mass_fraction("foot");
    const double shank_mass = mass_of(masses, "shank_foot") *
                              de_leva_mass_fraction("shank") /
                              shank_foot_fraction;
    const double foot_mass = mass_of(masses, "shank_foot") *
                             de_leva_mass_fraction("foot") /
                             shank_foot_fraction;
    const auto [k_torso, c_torso] =
            spring_params(mass_of(masses, "torso"),
                          rider.torso_resonance_hz, rider.damping_ratio);
    const auto [k_arms, c_arms] =
            spring_params(mass_of(masses, "arms"), rider.arm_resonance_hz,
                          rider.damping_ratio);
    const auto [k_leg, c_leg] =
            spring_params(mass_of(masses, "leg"), rider.leg_resonance_hz,
                          rider.damping_ratio);
    const double upper_arm_mass =
            mass_of(masses, "upper_arm_fraction") *
            mass_of(masses, "arms");

    const Vec3 pelvis_center = v_add(hip, {-0.010, 0.0, 0.040});
    RiderBody pelvis{
            .name = "rider_pelvis",
            .parent = "frame",
            .attach = hip,
            .joint = "rider_pelvis_z",
            .geoms = {make_geom("geom_rider_pelvis", "capsule",
                                v_add(pelvis_center, {-0.060, 0.0, 0.0}),
                                v_add(pelvis_center, {0.060, 0.0, 0.0}),
                                segment_radius_m("pelvis"),
                                mass_of(masses, "pelvis"))},
            .stiffness_n_m = rider.saddle_interface_k_n_m,
            .damping_ns_m = rider.saddle_interface_c_ns_m,
            .supported_mass_kg =
                    mass_of(masses, "pelvis") + mass_of(masses, "torso"),
            .unilateral = true,
            .interface = "saddle",
    };
    validate_rider_body(pelvis);
    RiderBody torso{
            .name = "rider_torso",
            .parent = "rider_pelvis",
            .attach = hip,
            .joint = "rider_torso_z",
            .geoms = {make_geom("geom_rider_torso", "capsule",
                                v_add(hip, v_scale(0.10, torso_axis)),
                                shoulder, segment_radius_m("torso"),
                                mass_of(masses, "torso") -
                                        mass_of(masses, "head")),
                      make_geom("geom_rider_head", "sphere", head_center,
                                head_center, HEAD_RADIUS_M,
                                mass_of(masses, "head"))},
            .stiffness_n_m = k_torso,
            .damping_ns_m = c_torso,
            .supported_mass_kg = mass_of(masses, "torso"),
            .unilateral = false,
            .interface = "saddle",
    };
    validate_rider_body(torso);
    RiderBody arms{
            .name = "rider_arms",
            .parent = "frame",
            .attach = grip,
            .joint = "rider_arms_z",
            .geoms = {make_geom("geom_rider_upper_arm", "capsule",
                                shoulder, elbow,
                                segment_radius_m("upper_arm"),
                                upper_arm_mass),
                      make_geom("geom_rider_forearm", "capsule", elbow,
                                grip, segment_radius_m("forearm"),
                                mass_of(masses, "arms") - upper_arm_mass)},
            .stiffness_n_m = k_arms,
            .damping_ns_m = c_arms,
            .supported_mass_kg = mass_of(masses, "arms"),
            .unilateral = false,
            .interface = "bar",
    };
    validate_rider_body(arms);

    const auto leg = [&](const std::string &side, const Vec3 &knee,
                         const Vec3 &ankle, const Vec3 &pedal) {
        RiderBody body{
                .name = "rider_leg_" + side,
                .parent = "frame",
                .attach = pedal,
                .joint = "rider_leg_" + side + "_z",
                .geoms = {make_geom("geom_rider_thigh_" + side, "capsule",
                                    hip, knee, segment_radius_m("thigh"),
                                    mass_of(masses, "thigh_to_legs")),
                          make_geom("geom_rider_shank_" + side, "capsule",
                                    knee, ankle, segment_radius_m("shank"),
                                    mass_of(masses, "shank_foot")),
                          make_geom("geom_rider_foot_" + side, "capsule",
                                    ankle, pedal, segment_radius_m("foot"),
                                    0.0)},
                .stiffness_n_m = k_leg,
                .damping_ns_m = c_leg,
                .supported_mass_kg = mass_of(masses, "leg"),
                .unilateral = true,
                .interface = "pedals",
        };
        validate_rider_body(body);
        return body;
    };
    const auto leg_chain = [&](std::string side, const Vec3 &knee,
                               const Vec3 &ankle, const Vec3 &pedal,
                               double lateral_y_m,
                               double crank_len_m) {
        return LegChain{
                .side = std::move(side),
                .lateral_y_m = lateral_y_m,
                .crank_len_m = crank_len_m,
                .hip = hip,
                .knee = knee,
                .ankle = ankle,
                .pedal = pedal,
                .thigh_len_m = l_thigh,
                .shank_len_m = l_shank,
                .thigh_mass_kg = mass_of(masses, "thigh_to_legs"),
                .shank_mass_kg = shank_mass,
                .foot_mass_kg = foot_mass,
                .pitch0_thigh_rad = pitch_xz(v_sub(knee, hip)),
                .pitch0_shank_rad = pitch_xz(v_sub(ankle, knee)),
                .pitch0_foot_rad = pitch_xz(v_sub(pedal, ankle)),
        };
    };

    std::vector<RiderBody> bodies;
    std::vector<LegChain> leg_chains;
    if (rider.legs == "articulated") {
        bodies = {std::move(pelvis), std::move(torso), std::move(arms)};
        leg_chains = {
            leg_chain("front", knee_front, ankle_front, pedal_front,
                      -geometry::PEDAL_LATERAL_OFFSET_M, crank_m),
            leg_chain("rear", knee_rear, ankle_rear, pedal_rear,
                      geometry::PEDAL_LATERAL_OFFSET_M, -crank_m),
        };
    } else {
        bodies = {std::move(pelvis), std::move(torso), std::move(arms),
                  leg("front", knee_front, ankle_front, pedal_front),
                  leg("rear", knee_rear, ankle_rear, pedal_rear)};
    }
    std::vector<std::pair<std::string, double>> path_masses;
    for (const char *key: {"torso", "pelvis", "arms", "leg"})
        path_masses.emplace_back(key, mass_of(masses, key));
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
            .bodies = std::move(bodies),
            .path_masses_kg = std::move(path_masses),
            .leg_chains = std::move(leg_chains),
    };
}
NATIVE_DIAG_POP

// RiderSpecs.seated_pose member ports.
SeatedPose seated_pose(const RiderSpecs &rider, const BikeSpecs &specs) {
    if (rider.variant != "seated")
        throw std::invalid_argument(
                "seated_pose is only defined for the seated variant, "
                "not " +
                py_str_repr(rider.variant));
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
    const SeatedPose pose = seated_pose(rider, specs);
    std::vector<RiderCenter> components;
    components.reserve(pose.bodies.size() + pose.leg_chains.size() * 3);
    for (const RiderBody &b: pose.bodies)
        components.push_back(RiderCenter{.name = b.name,
                                         .pos_m = b.center_of_mass(),
                                         .mass_kg = b.mass()});
    for (const LegChain &chain: pose.leg_chains) {
        const Vec3 offset{0.0, chain.lateral_y_m, 0.0};
        components.push_back(RiderCenter{
                .name = "rider_thigh_" + chain.side,
                .pos_m = v_add(v_scale(0.5, v_add(chain.hip, chain.knee)),
                               offset),
                .mass_kg = chain.thigh_mass_kg});
        components.push_back(RiderCenter{
                .name = "rider_shank_" + chain.side,
                .pos_m = v_add(v_scale(0.5, v_add(chain.knee, chain.ankle)),
                               offset),
                .mass_kg = chain.shank_mass_kg});
        components.push_back(RiderCenter{
                .name = "rider_foot_" + chain.side,
                .pos_m = v_add(v_scale(0.5, v_add(chain.ankle, chain.pedal)),
                               offset),
                .mass_kg = chain.foot_mass_kg});
    }
    return components;
}
std::vector<RiderCenter>
compute_rider_centers_of_mass(const RiderSpecs &rider) {
    return compute_rider_centers_of_mass(rider, BikeSpecs{});
}

// ---------------------------------------------------------------------
// saddle_path_apparent_mass
// ---------------------------------------------------------------------
std::vector<double>
saddle_path_apparent_mass(const SeatedPose &pose,
                          std::span<const double> frequencies_hz) {
    const RiderBody &pelvis = pose.body("rider_pelvis");
    const RiderBody &torso = pose.body("rider_torso");
    const double m1 = pelvis.mass(), k1 = pelvis.stiffness_n_m,
                 c1 = pelvis.damping_ns_m;
    const double m2 = torso.mass(), k2 = torso.stiffness_n_m,
                 c2 = torso.damping_ns_m;
    std::vector<double> out;
    out.reserve(frequencies_hz.size());
    for (const double f: frequencies_hz) {
        // s = 2j * np.pi * f — CPython complex*float then the complex
        // multiply ufunc per element.
        const Cpx s = cmul(Cpx{.re = 0.0, .im = 2.0 * M_PI},
                           Cpx{.re = f, .im = 0.0});
        const Cpx z1 = cadd(Cpx{.re = k1, .im = 0.0},
                            cmul(Cpx{.re = c1, .im = 0.0}, s));
        const Cpx z2 = cadd(Cpx{.re = k2, .im = 0.0},
                            cmul(Cpx{.re = c2, .im = 0.0}, s));
        // denominator = m1*s*s + z1 + z2 - z2*z2/(m2*s*s + z2)
        const Cpx m1ss = cmul(cmul(Cpx{.re = m1, .im = 0.0}, s), s);
        const Cpx m2ss = cmul(cmul(Cpx{.re = m2, .im = 0.0}, s), s);
        const Cpx frac = cdiv(cmul(z2, z2), cadd(m2ss, z2));
        const Cpx denominator =
                csub(cadd(cadd(m1ss, z1), z2), frac);
        const Cpx x1 = cdiv(z1, denominator);
        const Cpx seat_force =
                cmul(z1, csub(Cpx{.re = 1.0, .im = 0.0}, x1));
        const double abs_s = cabs(s);
        // np.abs(seat_force) / np.abs(s)**2 / (m1+m2)
        out.push_back(cabs(seat_force) / (abs_s * abs_s) / (m1 + m2));
    }
    return out;
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
    // dataclasses.replace(rider, variant='seated', legs='rigid',
    // saddle_share=.55, pedal_share=.33, bar_share=.12) — a fresh
    // validated record.
    RiderSpecs neutral = rider;
    neutral.variant = "seated";
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
