// model/mass.cpp — T15 mass/inertia boundary implementations.
//
// numpy-provider notes kept inline at each site: `x ** y` between Python
// scalars routes through pyfloat::float_pow (CPython's float_pow ladder);
// np.linalg.norm/np.dot use geometry::detail's Accelerate ILP64 ddot;
// builtin sum() uses numeric::PythonSum for float items and the generic
// sequential fold for ndarray items; ndarray.sum()/np.mean use
// numeric::numpy_pairwise_sum; format(x, ".17g") uses detail::py_g;
// np.linalg.eigvalsh reaches Accelerate's dsyevd$NEWLAPACK$ILP64 — the
// same LAPACK entry numpy's eigvalsh gufunc calls through the same ABI.
#include "mass.hpp"

#include "../diag.hpp"
#include "../io/value.hpp" // detail::float_from_lexeme (header-only)
#include "../lapack_abi.hpp"
#include "../numeric_sum.hpp"
#include "../writers/pyfloat.hpp"
#include "checks.hpp"
#include "rider_geometry.hpp" // compute_rider_centers_of_mass

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

// The model records are returned by value deliberately (the dict/tuple
// results the oracle returns); -Wlarge-by-value-copy exists to flag
// by-value *parameters* and clang gives no param/return split — same
// convention as writer_config.hpp / kinematics/solver.cpp.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")

namespace {

using bike::model::ComponentCenter;
using bike::model::Mat3;
using bike::model::Vec3;
using bike::model::checks::math_isclose;
using bike::model::checks::np_isclose_scalar;
namespace gd = bike::geometry::detail;

// Python `x ** y` between scalars — CPython float_pow's complete ladder.
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
[[nodiscard]] inline bool vec_finite(const Vec3 &v) noexcept {
    return std::isfinite(v[0]) && std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}
[[nodiscard]] inline Vec3 mm_to_m(const Vec3 &v) noexcept {
    return {v[0] / 1000.0, v[1] / 1000.0, v[2] / 1000.0};
}

// ---------------------------------------------------------------------
// Python float(str) — the CPython float_from_string_inner domain:
// ASCII-whitespace strip, optional sign, 'inf'/'infinity'/'nan' tokens
// (case-insensitive) or the decimal grammar with single underscores
// between digits. ValueError spells
// "could not convert string to float: 'repr'".
[[nodiscard]] std::string py_str_repr(std::string_view s) {
    // repr() for the token domain the attribute parser accepts:
    // single-quoted, backslash/quote escapes; control bytes render
    // through \xNN, and non-ASCII codepoints take CPython 3.13's
    // printability — Unicode-16-only codepoints are unassigned there,
    // so repr escapes them (io::detail::kUnicode16OnlyPrintable).
    std::string out = "'";
    static constexpr std::string_view kHex = "0123456789abcdef";
    for (std::size_t i = 0; i < s.size();) {
        const auto c = static_cast<unsigned char>(s[i]);
        if (c >= 0x80) {
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
            case '\\': out += "\\\\"; break;
            case '\'': out += "\\'"; break;
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
    out.push_back('\'');
    return out;
}

[[noreturn]] void fail_float_parse(std::string_view raw) {
    throw std::invalid_argument("could not convert string to float: " +
                                py_str_repr(raw));
}

[[nodiscard]] bool py_space(char c) noexcept {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' ||
           c == '\f';
}

// Consume digit-or-underscore runs: underscores only between digits.
// Returns the digit count appended to `clean`.
[[nodiscard]] std::size_t
consume_digits(std::string_view s, std::size_t &pos, std::string &clean) {
    std::size_t digits = 0;
    bool prev_digit = false;
    while (pos < s.size()) {
        const char c = s[pos];
        if (c >= '0' && c <= '9') {
            clean.push_back(c);
            prev_digit = true;
            ++digits;
        } else if (c == '_' && prev_digit && pos + 1 < s.size() &&
                   s[pos + 1] >= '0' && s[pos + 1] <= '9') {
            prev_digit = false; // swallowed; a digit follows
        } else {
            break;
        }
        ++pos;
    }
    return digits;
}

[[nodiscard]] double parse_py_float(std::string_view raw) {
    std::size_t lo = 0, hi = raw.size();
    while (lo < hi && py_space(raw[lo])) ++lo;
    while (hi > lo && py_space(raw[hi - 1])) --hi;
    const std::string_view s = raw.substr(lo, hi - lo);
    if (s.empty()) fail_float_parse(raw);

    std::size_t pos = 0;
    bool negative = false;
    if (s[pos] == '+' || s[pos] == '-') {
        negative = s[pos] == '-';
        ++pos;
        if (pos == s.size()) fail_float_parse(raw);
    }
    const std::string_view rest = s.substr(pos);
    std::string lower(rest);
    for (char &c: lower)
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    if (lower == "inf" || lower == "infinity")
        return negative ? -std::numeric_limits<double>::infinity()
                        : std::numeric_limits<double>::infinity();
    if (lower == "nan")
        return std::copysign(std::numeric_limits<double>::quiet_NaN(),
                             negative ? -1.0 : 1.0);

    // digits [. digits] [e[+-]digits] — at least one mantissa digit.
    std::string clean;
    if (negative) clean.push_back('-');
    std::size_t mantissa_digits = consume_digits(s, pos, clean);
    if (pos < s.size() && s[pos] == '.') {
        clean.push_back('.');
        ++pos;
        mantissa_digits += consume_digits(s, pos, clean);
    }
    if (mantissa_digits == 0) fail_float_parse(raw);
    if (pos < s.size() && (s[pos] == 'e' || s[pos] == 'E')) {
        clean.push_back('e');
        ++pos;
        if (pos < s.size() && (s[pos] == '+' || s[pos] == '-'))
            clean.push_back(s[pos++]);
        if (consume_digits(s, pos, clean) == 0) fail_float_parse(raw);
    }
    if (pos != s.size()) fail_float_parse(raw);
    // from_chars instead of strtod (-Wunsafe-buffer-usage-in-libc-call)
    // through the shared float_from_lexeme — identical grammar for this
    // domain, and its range errors resolve to +/-inf or signed zero the
    // way CPython float() overflow/underflow does.
    bool consumed = false;
    const double out =
            bike::io::detail::float_from_lexeme(clean, consumed);
    if (!consumed) fail_float_parse(raw);
    return out;
}

// ---------------------------------------------------------------------
// np.linalg.eigvalsh on a 3x3 symmetric matrix — numeric_backend::dsyevd,
// called exactly like numpy's eigvalsh gufunc through its ILP64 provider
// (JOBZ='N', UPLO='L', an LWORK=-1 workspace query first, then the real
// call with LWORK == the queried optimum). numpy copies the C-order
// input verbatim into the Fortran buffer, so LAPACK sees A^T —
// identical for the symmetric inputs this boundary admits.
[[nodiscard]] std::array<double, 3> eigvalsh3(const Mat3 &tensor) {
    std::array<double, 9> a = tensor; // Fortran-order copy == A^T == A.
    std::array<double, 3> w{};
    const numeric_backend::integer n = 3, lda = 3;
    numeric_backend::integer info = 0;
    const char jobz = 'N', uplo = 'L';
    numeric_backend::integer lwork = -1, liwork = -1;
    double work_query = 0.0;
    numeric_backend::integer iwork_query = 0;
    numeric_backend::dsyevd(&jobz, &uplo, &n, a.data(), &lda, w.data(), &work_query,
                            &lwork, &iwork_query, &liwork, &info);
    if (info != 0)
        throw std::runtime_error("dsyevd workspace query failed");
    lwork = static_cast<numeric_backend::integer>(work_query);
    liwork = iwork_query;
    std::vector<double> work(static_cast<std::size_t>(lwork));
    std::vector<numeric_backend::integer> iwork(static_cast<std::size_t>(liwork));
    numeric_backend::dsyevd(&jobz, &uplo, &n, a.data(), &lda, w.data(), work.data(),
                            &lwork, iwork.data(), &liwork, &info);
    if (info != 0) throw std::runtime_error("dsyevd failed to converge");
    return w;
}

[[nodiscard]] Mat3 diag3(double a, double b, double c) noexcept {
    return {a, 0., 0., 0., b, 0., 0., 0., c};
}

// BikeMassSpecs.component_masses — the dataclass-fields walk: every
// *_mass field in declared order with the suffix stripped.
struct ComponentField {
    std::string_view id;
    double bike::model::BikeMassSpecs::*member;
};
constexpr std::array kComponentFields{
    ComponentField{.id = "motor",
                   .member = &bike::model::BikeMassSpecs::motor_mass},
    ComponentField{.id = "battery",
                   .member = &bike::model::BikeMassSpecs::battery_mass},
    ComponentField{
            .id = "frame_structure",
            .member = &bike::model::BikeMassSpecs::frame_structure_mass},
    ComponentField{.id = "saddle_post",
                   .member = &bike::model::BikeMassSpecs::saddle_post_mass},
    ComponentField{
            .id = "crank_pedals",
            .member = &bike::model::BikeMassSpecs::crank_pedals_mass},
    ComponentField{
            .id = "steer_assembly",
            .member = &bike::model::BikeMassSpecs::steer_assembly_mass},
    ComponentField{.id = "stanchions",
                   .member = &bike::model::BikeMassSpecs::stanchions_mass},
    ComponentField{
            .id = "fork_lowers",
            .member = &bike::model::BikeMassSpecs::fork_lowers_mass},
    ComponentField{.id = "chainstay",
                   .member = &bike::model::BikeMassSpecs::chainstay_mass},
    ComponentField{.id = "seatstay",
                   .member = &bike::model::BikeMassSpecs::seatstay_mass},
    ComponentField{.id = "rocker",
                   .member = &bike::model::BikeMassSpecs::rocker_mass},
    ComponentField{.id = "shock_yoke",
                   .member = &bike::model::BikeMassSpecs::shock_yoke_mass},
    ComponentField{
            .id = "shock_damper",
            .member = &bike::model::BikeMassSpecs::shock_damper_mass},
    ComponentField{.id = "front_wheel",
                   .member = &bike::model::BikeMassSpecs::front_wheel_mass},
    ComponentField{.id = "rear_wheel",
                   .member = &bike::model::BikeMassSpecs::rear_wheel_mass},
};

} // namespace

namespace bike::model {

// ---------------------------------------------------------------------
// physics/inertia.py
// ---------------------------------------------------------------------
Mat3 ring_inertia(double mass_kg, double inner_m, double outer_m,
                  double width_m) {
    if (!(std::isfinite(mass_kg) && std::isfinite(inner_m) &&
          std::isfinite(outer_m) && std::isfinite(width_m)))
        throw std::invalid_argument("non-finite inertia input");
    if (mass_kg <= 0 || inner_m < 0 || outer_m <= inner_m || width_m <= 0)
        throw std::invalid_argument("invalid annular cylinder");
    // radial = inner_m**2 + outer_m**2 (scalar ** → float_pow).
    const double radial = py_pow(inner_m, 2.0) + py_pow(outer_m, 2.0);
    const double transverse =
            mass_kg * (3.0 * radial + py_pow(width_m, 2.0)) / 12.0;
    return diag3(transverse, mass_kg * radial / 2.0, transverse);
}

Mat3 parallel_axis(double mass_kg, const Vec3 &offset_m) {
    const Vec3 &r = offset_m;
    if (!vec_finite(r) || !std::isfinite(mass_kg) || mass_kg < 0)
        throw std::invalid_argument("invalid parallel-axis input");
    // mass * (float(r @ r) * I - outer(r, r)) — np.dot → ILP64 ddot.
    const double rr = gd::np_dot3(r, r);
    Mat3 out{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            out[i * 3 + j] =
                    mass_kg * ((i == j ? rr : 0.0) - r[i] * r[j]);
    return out;
}

WheelInertia wheel_body_inertia(
        double mass_kg, std::span<const WheelComponent> components) {
    if (!std::isfinite(mass_kg) || mass_kg <= 0 || components.empty())
        throw std::invalid_argument("invalid wheel mass or empty profile");
    for (const WheelComponent &component: components)
        if (!(std::isfinite(component.mass_fraction) &&
              component.mass_fraction > 0))
            throw std::invalid_argument("invalid wheel mass fractions");
    { // math.isclose(sum(fractions), 1.0, rel_tol=0, abs_tol=1e-12)
        // Every item is a WheelComponent.mass_fraction float — the
        // compensated float branch is the oracle's whole path.
        numeric::PythonSum fractions;
        for (const WheelComponent &component: components)
            fractions.add(component.mass_fraction);
        if (!math_isclose(fractions.total(), 1.0, 0.0, 1e-12))
            throw std::invalid_argument(
                    "wheel mass fractions must sum to one");
    }
    for (const WheelComponent &component: components)
        if (!vec_finite(component.offset_m))
            throw std::invalid_argument("invalid wheel component offset");
    // sum(component.mass_fraction * center, np.zeros(3)) — the generic
    // sequential fold: acc = acc + (frac * center) per component.
    Vec3 com{0.0, 0.0, 0.0};
    for (const WheelComponent &component: components)
        for (std::size_t i = 0; i < 3; ++i)
            com[i] = com[i] + component.mass_fraction * component.offset_m[i];
    Mat3 tensor{}; // np.zeros((3, 3)); in-place += elementwise.
    for (const WheelComponent &component: components) {
        const double component_mass = mass_kg * component.mass_fraction;
        const Mat3 ring =
                ring_inertia(component_mass, component.inner_radius_m,
                             component.outer_radius_m, component.width_m);
        for (std::size_t i = 0; i < 9; ++i) tensor[i] += ring[i];
        const Vec3 shifted = v_sub(component.offset_m, com);
        const Mat3 pa = parallel_axis(component_mass, shifted);
        for (std::size_t i = 0; i < 9; ++i) tensor[i] += pa[i];
    }
    return {.com = com, .tensor = tensor};
}

void add_body_inertial(XmlBodyElement &body, double mass_kg,
                       const Vec3 &com_m, const Mat3 &tensor) {
    if (!std::isfinite(mass_kg) || mass_kg <= 0)
        throw std::invalid_argument(
                "invalid body inertial dimensions or mass");
    if (!vec_finite(com_m))
        throw std::invalid_argument("non-finite body inertial");
    for (const double v: tensor)
        if (!std::isfinite(v))
            throw std::invalid_argument("non-finite body inertial");
    // np.allclose(tensor, tensor.T, rtol=0, atol=1e-12).
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            if (!np_isclose_scalar(tensor[i * 3 + j], tensor[j * 3 + i],
                                   0.0, 1e-12))
                throw std::invalid_argument(
                        "inertia tensor must be symmetric");
    const std::array<double, 3> eig = eigvalsh3(tensor);
    if (eig[0] <= 0 || eig[2] > eig[0] + eig[1] + 1e-12)
        throw std::invalid_argument(
                "inertia violates positivity or triangle inequality");
    if (body.inertial.has_value())
        throw std::invalid_argument(
                "body already has an explicit inertial");
    const std::array<double, 6> values{tensor[0], tensor[4], tensor[8],
                                       tensor[1], tensor[2], tensor[5]};
    std::string pos, fullinertia;
    for (std::size_t i = 0; i < 3; ++i) {
        if (i) pos.push_back(' ');
        pos += gd::py_g(com_m[i], 17);
    }
    for (std::size_t i = 0; i < 6; ++i) {
        if (i) fullinertia.push_back(' ');
        fullinertia += gd::py_g(values[i], 17);
    }
    body.inertial = XmlInertialAttributes{.mass = gd::py_g(mass_kg, 17),
                                          .pos = std::move(pos),
                                          .fullinertia =
                                                  std::move(fullinertia)};
}

// ---------------------------------------------------------------------
// physics/component_masses.py
// ---------------------------------------------------------------------
void assign_component_mass(std::span<XmlGeomElement> geoms,
                           double total_kg) {
    if (!std::isfinite(total_kg) || total_kg <= 0)
        throw std::invalid_argument(
                "component mass must be finite and positive");
    // weights = [float(geom.get("mass", "0")) for geom in geoms]
    std::vector<double> weights;
    weights.reserve(geoms.size());
    for (const XmlGeomElement &geom: geoms)
        weights.push_back(
                parse_py_float(geom.mass.value_or(std::string("0"))));
    if (weights.empty() ||
        std::ranges::any_of(weights, [](double w) {
            return !std::isfinite(w) || w < 0;
        }))
        throw std::invalid_argument("component has no valid mass weights");
    // sum(weights) — builtin float accumulation; the Python list holds
    // only floats, so the compensated float branch is exact.
    numeric::PythonSum denom;
    for (const double w: weights) denom.add(w);
    const double denominator = denom.total();
    if (!std::isfinite(denominator) || denominator <= 0)
        throw std::invalid_argument("component has no valid mass weights");
    for (std::size_t i = 0; i < geoms.size(); ++i)
        geoms[i].mass =
                gd::py_g(total_kg * weights[i] / denominator, 17);
}

void register_component(ComponentRegistry &registry,
                        const std::string &component_id,
                        std::span<const XmlGeomElement> parts) {
    for (const auto &[id, entries]: registry)
        if (id == component_id)
            throw std::invalid_argument(
                    "component " + py_str_repr(component_id) +
                    " was registered twice");
    std::vector<const XmlGeomElement *> positive;
    positive.reserve(parts.size());
    for (const XmlGeomElement &part: parts)
        if (parse_py_float(part.mass.value_or(std::string("0"))) > 0)
            positive.push_back(&part);
    if (positive.empty())
        throw std::invalid_argument("component " +
                                    py_str_repr(component_id) +
                                    " has no mass-bearing parts");
    registry.emplace_back(component_id, std::move(positive));
}

// ---------------------------------------------------------------------
// physics/mass.py — BikeMassSpecs.
// ---------------------------------------------------------------------
std::vector<std::pair<std::string, double>>
BikeMassSpecs::component_masses() const {
    std::vector<std::pair<std::string, double>> out;
    out.reserve(kComponentFields.size());
    for (const ComponentField &field: kComponentFields)
        out.emplace_back(std::string(field.id), this->*field.member);
    return out;
}

std::vector<std::pair<std::string, BikeMassSpecs::ComponentProvenance>>
BikeMassSpecs::component_provenance() const {
    // {component_id: {"mass": "synthetic", "distribution": "synthetic"}}
    // — iterated over component_masses's field order.
    std::vector<std::pair<std::string, ComponentProvenance>> out;
    out.reserve(kComponentFields.size());
    for (const ComponentField &field: kComponentFields)
        out.emplace_back(std::string(field.id),
                         ComponentProvenance{.mass = "synthetic",
                                             .distribution = "synthetic"});
    return out;
}

double BikeMassSpecs::total_bike_mass() const {
    // sum(component_masses.values()) — builtin float accumulation; the
    // dict values are float fields, so the compensated branch is exact.
    numeric::PythonSum total;
    for (const ComponentField &field: kComponentFields)
        total.add(this->*field.member);
    return total.total();
}

double BikeMassSpecs::compute_wheel_rotational_inertia(
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
        double wheel_mass, double outer_radius, double hub_radius) const {
    const double m_rim = wheel_rim_tire_fraction * wheel_mass;
    const double m_hub = wheel_hub_core_fraction * wheel_mass;
    // m_rim * (outer**2) + (0.5 * m_hub) * (hub**2) — scalar float_pow.
    const double i_yy = m_rim * py_pow(outer_radius, 2.0) +
                        0.5 * m_hub * py_pow(hub_radius, 2.0);
    return i_yy;
}

// ---------------------------------------------------------------------
// physics/mass.py — component centers and the static system CG.
// ---------------------------------------------------------------------
std::optional<double>
saddle_top_z_for_rider(const geometry::BikeSpecs &specs,
                       const configuration::RiderSpecs *rider_specs) {
    // Python: rider_specs is not None and rider_specs.variant == "seated".
    if (rider_specs != nullptr && rider_specs->variant == "seated")
        return seated_pose(*rider_specs, specs).saddle.top_z_m;
    return std::nullopt;
}

ComponentCenters compute_component_centers_of_mass(
        const geometry::BikeSpecs &specs, const BikeMassSpecs &mass_specs,
        const kinematics::HorstLinkageSolver &solver,
        const configuration::RiderSpecs *rider_specs) {
    const kinematics::KinematicState st0 =
            solver.solve_state_from_wheel_travel(0.0);
    const geometry::FixedFramePoints fixed =
            geometry::get_fixed_frame_points(specs);

    const Vec3 p_bb{0.0, 0.0, 0.0};
    const Vec3 p_fa = mm_to_m(geometry::compute_front_axle(specs));
    const Vec3 p_ra = mm_to_m(geometry::compute_rear_axle(specs));

    const Vec3 p0 = mm_to_m(fixed.P0);
    const Vec3 p2 = mm_to_m(st0.P2);
    const Vec3 p3 = mm_to_m(st0.P3);
    const Vec3 p4 = mm_to_m(st0.P4);
    const Vec3 p5 = mm_to_m(fixed.P5);
    const Vec3 p6 = mm_to_m(st0.P6);
    const Vec3 p7 = mm_to_m(fixed.P7);
    const Vec3 p9 = mm_to_m(fixed.P9);
    const Vec3 p10 = mm_to_m(fixed.P10);
    const Vec3 p11 = mm_to_m(fixed.P11);
    const Vec3 ht_bot = mm_to_m(fixed.P_HT_bot);

    const Vec3 pos_motor = v_add(p_bb, {0.030, 0.0, 0.0275});
    const Vec3 pos_battery{0.172834, 0.0, 0.226886};
    const Vec3 pos_frame{0.180925, 0.0, 0.335899};
    // The post and saddle follow the rider's cockpit placement.
    const geometry::SaddleGeometry saddle = geometry::saddle_geometry(
            p9, saddle_top_z_for_rider(specs, rider_specs));
    const Vec3 pos_saddle = geometry::saddle_post_center_of_mass(
            mass_specs.saddle_post_mass, p10, p9, saddle);
    const Vec3 pos_cranks = p_bb; // p_bb.copy()
    const Vec3 pos_steer = v_add(p11, {-0.015, 0.0, 0.025});
    // 0.5 * (ht_bot + 0.5 * (ht_bot + p_fa))
    const Vec3 pos_stanchions =
            v_scale(0.5, v_add(ht_bot, v_scale(0.5, v_add(ht_bot, p_fa))));
    // 0.5 * (0.5 * (ht_bot + p_fa) + p_fa)
    const Vec3 pos_lowers = v_scale(
            0.5, v_add(v_scale(0.5, v_add(ht_bot, p_fa)), p_fa));
    const Vec3 pos_front_wheel = p_fa;
    // 0.55 * p0 + 0.45 * p2
    const Vec3 pos_chainstay =
            v_add(v_scale(0.55, p0), v_scale(0.45, p2));
    // 0.50 * p_ra + 0.30 * p2 + 0.20 * p3 — left-associative.
    const Vec3 pos_seatstay =
            v_add(v_add(v_scale(0.50, p_ra), v_scale(0.30, p2)),
                  v_scale(0.20, p3));
    const Vec3 pos_rocker = v_add(p5, {-0.008, 0.0, 0.018});
    const Vec3 pos_yoke = v_scale(0.5, v_add(p4, p6));
    const Vec3 pos_damper = v_scale(0.5, v_add(p7, p6));
    const Vec3 pos_rear_wheel = p_ra;

    // Dict insertion order — NOT the dataclass field order.
    ComponentCenters components{
            ComponentCenter{.name = "motor", .pos_m = pos_motor,
                            .mass_kg = mass_specs.motor_mass},
            ComponentCenter{.name = "battery", .pos_m = pos_battery,
                            .mass_kg = mass_specs.battery_mass},
            ComponentCenter{.name = "frame_structure", .pos_m = pos_frame,
                            .mass_kg = mass_specs.frame_structure_mass},
            ComponentCenter{.name = "saddle_post", .pos_m = pos_saddle,
                            .mass_kg = mass_specs.saddle_post_mass},
            ComponentCenter{.name = "crank_pedals", .pos_m = pos_cranks,
                            .mass_kg = mass_specs.crank_pedals_mass},
            ComponentCenter{.name = "steer_assembly", .pos_m = pos_steer,
                            .mass_kg = mass_specs.steer_assembly_mass},
            ComponentCenter{.name = "stanchions", .pos_m = pos_stanchions,
                            .mass_kg = mass_specs.stanchions_mass},
            ComponentCenter{.name = "fork_lowers", .pos_m = pos_lowers,
                            .mass_kg = mass_specs.fork_lowers_mass},
            ComponentCenter{.name = "front_wheel", .pos_m = pos_front_wheel,
                            .mass_kg = mass_specs.front_wheel_mass},
            ComponentCenter{.name = "chainstay", .pos_m = pos_chainstay,
                            .mass_kg = mass_specs.chainstay_mass},
            ComponentCenter{.name = "seatstay", .pos_m = pos_seatstay,
                            .mass_kg = mass_specs.seatstay_mass},
            ComponentCenter{.name = "rocker", .pos_m = pos_rocker,
                            .mass_kg = mass_specs.rocker_mass},
            ComponentCenter{.name = "shock_yoke", .pos_m = pos_yoke,
                            .mass_kg = mass_specs.shock_yoke_mass},
            ComponentCenter{.name = "shock_damper", .pos_m = pos_damper,
                            .mass_kg = mass_specs.shock_damper_mass},
            ComponentCenter{.name = "rear_wheel", .pos_m = pos_rear_wheel,
                            .mass_kg = mass_specs.rear_wheel_mass},
    };

    if (rider_specs != nullptr)
        // components.update(rider.compute_rider_centers_of_mass(specs))
        for (const RiderCenter &center:
             compute_rider_centers_of_mass(*rider_specs, specs))
            components.push_back(
                    ComponentCenter{.name = center.name,
                                    .pos_m = center.pos_m,
                                    .mass_kg = center.mass_kg});
    return components;
}

ComponentCenters compute_component_centers_of_mass(
        const geometry::BikeSpecs &specs, const BikeMassSpecs &mass_specs,
        const configuration::RiderSpecs *rider_specs) {
    const kinematics::HorstLinkageSolver solver(specs);
    return compute_component_centers_of_mass(specs, mass_specs, solver,
                                             rider_specs);
}

ComponentCenters
compute_component_centers_of_mass(const geometry::BikeSpecs &specs) {
    const BikeMassSpecs mass_specs{};
    return compute_component_centers_of_mass(specs, mass_specs, nullptr);
}

StaticSystemCg
compute_static_system_cg(const geometry::BikeSpecs &specs,
                         const BikeMassSpecs &mass_specs,
                         const configuration::RiderSpecs *rider_specs,
                         const kinematics::HorstLinkageSolver *solver) {
    const ComponentCenters components =
            solver != nullptr
                    ? compute_component_centers_of_mass(
                              specs, mass_specs, *solver, rider_specs)
                    : compute_component_centers_of_mass(
                              specs, mass_specs, rider_specs);

    // total_mass += mass and weighted_pos += mass * pos — plain
    // sequential folds over the dict order, NOT compensated.
    double total_mass = 0.0;
    Vec3 weighted_pos{0.0, 0.0, 0.0};
    for (const ComponentCenter &component: components) {
        total_mass += component.mass_kg;
        for (std::size_t i = 0; i < 3; ++i)
            weighted_pos[i] += component.mass_kg * component.pos_m[i];
    }
    const Vec3 cg_pos = v_div(weighted_pos, total_mass);

    const Vec3 p_fa = mm_to_m(geometry::compute_front_axle(specs));
    const Vec3 p_ra = mm_to_m(geometry::compute_rear_axle(specs));
    const double wheelbase = p_fa[0] - p_ra[0];

    const double g = 9.81;
    const double total_weight_n = total_mass * g;
    const double dist_to_front = p_fa[0] - cg_pos[0];
    const double dist_to_rear = cg_pos[0] - p_ra[0];

    const double front_load_n =
            total_weight_n * (dist_to_rear / wheelbase);
    const double rear_load_n =
            total_weight_n * (dist_to_front / wheelbase);

    const double front_pct = (front_load_n / total_weight_n) * 100.0;
    const double rear_pct = (rear_load_n / total_weight_n) * 100.0;

    const double bike_mass = mass_specs.total_bike_mass();
    const double rider_mass =
            rider_specs != nullptr ? rider_specs->total_rider_mass() : 0.0;

    return {.total_mass_kg = total_mass,
            .bike_mass_kg = bike_mass,
            .rider_mass_kg = rider_mass,
            .cg_pos_m = cg_pos,
            .wheelbase_m = wheelbase,
            .front_axle_x = p_fa[0],
            .rear_axle_x = p_ra[0],
            .front_load_n = front_load_n,
            .rear_load_n = rear_load_n,
            .front_load_pct = front_pct,
            .rear_load_pct = rear_pct};
}

// ---------------------------------------------------------------------
// sim/ride/mass_properties.py
// ---------------------------------------------------------------------
Vec3 compiled_center_of_mass(const mjModel_ &model,
                             const mjData_ &data) {
    // masses = np.asarray(model.body_mass); total = float(masses.sum())
    // views::counted instead of the (ptr, size) span ctor: clang's
    // -Wunsafe-buffer-usage-in-container rejects the raw two-argument
    // form — same pattern as runtime/step.cpp's buffer views.
    const std::size_t nbody = static_cast<std::size_t>(model.nbody);
    const std::span<const mjtNum> masses{
            std::views::counted(model.body_mass, model.nbody)};
    const double total = numeric::numpy_pairwise_sum(masses);
    if (!std::isfinite(total) || total <= 0)
        throw std::invalid_argument(
                "model has no positive physical mass");
    // np.sum(masses[:, None] * data.xipos, axis=0): the reduce axis is
    // outer/strided, so numpy accumulates per lane sequentially —
    // out[j] += masses[i] * xipos[i][j] — NOT the pairwise path.
    const std::span<const mjtNum> xipos{
            std::views::counted(data.xipos, model.nbody * 3)};
    Vec3 out{0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < nbody; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            out[j] += masses[i] * xipos[i * 3 + j];
    out = v_div(out, total);
    return out;
}

std::pair<double, double>
static_contact_loads(const mjModel_ &model, const mjData_ &data,
                     // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
                     int front_geom_id, int rear_geom_id) {
    // contact_x = {front: [...], rear: [...]} — append per contact.
    const std::span<const mjContact> contacts{
            std::views::counted(data.contact, data.ncon)};
    std::vector<double> front_x, rear_x;
    for (const mjContact &contact: contacts) {
        // geom[]/pos[] are fixed C arrays — the static-extent span
        // keeps the bound in the type (raw subscript is flagged).
        const std::span<const int, 2> geoms{contact.geom};
        const std::span<const mjtNum, 3> pos{contact.pos};
        for (const int geom_id: geoms) {
            if (geom_id == front_geom_id)
                front_x.push_back(pos[0]);
            else if (geom_id == rear_geom_id)
                rear_x.push_back(pos[0]);
        }
    }
    if (front_x.empty() || rear_x.empty())
        throw std::invalid_argument(
                "both wheel contact points are required for static load");
    // np.mean(values) == pairwise_sum(values) / len(values).
    const double fx = numeric::numpy_pairwise_sum(front_x) /
                      static_cast<double>(front_x.size());
    const double rx = numeric::numpy_pairwise_sum(rear_x) /
                      static_cast<double>(rear_x.size());
    const double wheelbase = fx - rx;
    if (wheelbase <= 0)
        throw std::invalid_argument(
                "front contact must be ahead of rear contact");
    const double com_x = compiled_center_of_mass(model, data)[0];
    const std::span<const mjtNum> masses{
            std::views::counted(model.body_mass, model.nbody)};
    const std::span<const mjtNum, 3> gravity{model.opt.gravity};
    const double total_weight = numeric::numpy_pairwise_sum(masses) *
                                std::fabs(gravity[2]);
    // total_weight * (com_x - rear_x) / wheelbase — left-associative.
    const double front_load =
            total_weight * (com_x - rx) / wheelbase;
    return {front_load, total_weight - front_load};
}

NATIVE_DIAG_POP

} // namespace bike::model
