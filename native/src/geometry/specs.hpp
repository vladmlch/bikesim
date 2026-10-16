// geometry/specs.hpp — the T14 geometry boundary: specs.py's three
// dataclass records, hardpoints.py's hardpoint/trail/frame-angle
// calculators, validation.py's domain validator and cockpit.py's
// saddle/seatpost/grip/pedal placement, all in the BB-origin MuJoCo
// frame (+X forward, +Y left, +Z up).
//
// Bitwise contract: every expression is lowered to the same rounded
// binary64 operations the Python oracle performs — elementwise NumPy
// array ops become per-component ops in source order, np.linalg.norm /
// np.dot dispatch to the same Accelerate CBLAS entry points numpy's
// ILP64 build calls (verified bitwise: 0 mismatches on 600,000 random
// norms), np.radians/np.degrees multiply by NumPy's NPY_PI_180 /
// NPY_180_PI constants, np.clip scalar-bounds runs the clip.cpp
// const-minmax loop, and builtin max keeps Python's first-tie-wins
// order. Geometry units stay in millimetres; cockpit.py works in
// metres — the /1000 conversions are explicit call sites, mirroring
// the source.
#pragma once

#include <array>
#include <charconv>
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../numeric_numpy.hpp" // numeric::np_maximum — np.max reduce

namespace bike::geometry {

// [X, Y, Z] millimetre/metre vectors. Python code treats these as
// 1-D ndarrays; the fixed-size record is the exact same domain.
using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;

// ---------------------------------------------------------------------------
// specs.py — the three dataclass records (declared field order, defaults
// verbatim). BikeSpecs.geometry/.suspension rebuild the nested records
// field by field exactly like the @property implementations.
// ---------------------------------------------------------------------------
struct FrameGeometrySpecs {
    double reach = 480.0;
    double stack = 646.0;
    double head_angle_deg = 64.0;
    double effective_seat_angle_deg = 77.0;
    double bb_drop = 22.5;
    double wheelbase = 1280.55;
    double headtube_length = 120.0;
    double front_wheel_radius = 372.0;
    double rear_wheel_radius = 352.0;
    double wheel_radius = 372.0; // legacy alias (front 29")
};

struct SuspensionHardwareSpecs {
    double fork_travel = 180.0;
    double fork_offset = 44.0;
    double rear_wheel_travel = 180.0;
    double shock_eye_to_eye = 205.0;
    double shock_stroke = 65.0;
    double fork_stiffness = 6000.0;
    double fork_damping = 450.0;
    double shock_stiffness = 114600.0;
    double shock_damping = 1200.0;
};

struct BikeSpecs {
    // Frame geometry (mm / deg).
    double reach = 480.0;
    double stack = 646.0;
    double head_angle_deg = 64.0;
    double effective_seat_angle_deg = 77.0;
    double bb_drop = 22.5;
    double wheelbase = 1280.55;
    double headtube_length = 120.0;
    double front_wheel_radius = 372.0;
    double rear_wheel_radius = 352.0;
    double wheel_radius = 372.0;

    // Drivetrain.
    double crank_length = 165.0; // mm

    // Suspension hardware.
    double fork_travel = 180.0;
    double fork_offset = 44.0;
    double rear_wheel_travel = 180.0;
    double shock_eye_to_eye = 205.0;
    double shock_stroke = 65.0;
    double fork_stiffness = 6000.0;
    double fork_damping = 450.0;
    double shock_stiffness = 114600.0;
    double shock_damping = 1200.0;

    // Fork damper + air spring settings.
    int fork_hsc = 2;
    int fork_lsc = 7;
    int fork_rebound = 9;
    double fork_initial_psi = 85.2;
    int fork_air_tokens = 2;

    // Rear shock damper settings.
    int shock_hsc = 2;
    int shock_lsc = 7;
    int shock_rebound = 7;
    int shock_hbo = 2;
    bool shock_lockout = false;

    // BikeSpecs.geometry / .suspension properties.
    [[nodiscard]] FrameGeometrySpecs geometry() const;
    [[nodiscard]] SuspensionHardwareSpecs suspension() const;
};

// ---------------------------------------------------------------------------
// hardpoints.py — all coordinates mm.
// ---------------------------------------------------------------------------

// compute_headtube_points -> (P11_top, P_HT_bot).
struct HeadtubePoints {
    Vec3 top;
    Vec3 bottom;
};
[[nodiscard]] HeadtubePoints compute_headtube_points(const BikeSpecs &specs);
[[nodiscard]] Vec3 compute_front_axle(const BikeSpecs &specs);
[[nodiscard]] double compute_ground_z(const BikeSpecs &specs);
[[nodiscard]] Vec3 compute_rear_axle(const BikeSpecs &specs);
// compute_rear_axle(specs, front_axle=<given>) — the optional-arg branch.
[[nodiscard]] Vec3 compute_rear_axle(const BikeSpecs &specs,
                                     const Vec3 &front_axle);

// compute_trail result dict, source key order.
struct TrailResult {
    double ground_trail;
    double mechanical_trail;
    double axle_to_crown;
    double steer_ground_intersection_x;
    double ground_z;
};
[[nodiscard]] TrailResult compute_trail(const BikeSpecs &specs);

// get_fixed_frame_points result dict. Member names keep the public
// export names verbatim; ordered_names()/at() give the dict's
// insertion order (P0, P5, P7, P8, P9, P10, P11, P_HT_bot, BB).
struct FixedFramePoints {
    Vec3 P0;
    Vec3 P5;
    Vec3 P7;
    Vec3 P8;
    Vec3 P9;
    Vec3 P10;
    Vec3 P11;
    Vec3 P_HT_bot;
    Vec3 BB;

    static constexpr std::size_t kCount = 9;
    [[nodiscard]] const Vec3 &at(std::size_t index) const;
    [[nodiscard]] std::array<std::string_view, kCount> names() const;
    [[nodiscard]] const Vec3 &operator[](std::string_view name) const;
};
[[nodiscard]] FixedFramePoints get_fixed_frame_points(const BikeSpecs &specs);

// compute_frame_angles result dict, source key order.
struct FrameAngles {
    double angle_x_tube1_deg;
    double angle_x_tube2_deg;
    double angle_seat_tube_x_deg;
    double effective_seat_tube_angle_deg;
    double angle_1_deg;
    double head_angle_deg;
};
[[nodiscard]] FrameAngles compute_frame_angles(const BikeSpecs &specs);

// ---------------------------------------------------------------------------
// validation.py — validate_geometry. Throws std::invalid_argument with
// the Python ValueError message verbatim on unphysical geometry.
// ---------------------------------------------------------------------------
[[nodiscard]] bool validate_geometry(const BikeSpecs &specs);

// ---------------------------------------------------------------------------
// cockpit.py — metre-domain constants, records and placement functions.
// Constant names keep the module's __all__ spelling verbatim.
// ---------------------------------------------------------------------------

// SEATPOST_AXIS: the normalized (-63, 0, 194) mm photo axis — computed at
// init via the np.linalg.norm sqrt(ddot) path, elementwise division.
extern const Vec3 SEATPOST_AXIS;

inline constexpr double PHOTO_SADDLE_TOP_Z_M = 0.690;
inline constexpr double SADDLE_HALF_THICKNESS_M = 0.015;
inline constexpr double SADDLE_SETBACK_FROM_POST_TOP_M = 0.024;
inline constexpr double SADDLE_HALF_LENGTH_M = 0.13;
inline constexpr double SADDLE_HALF_WIDTH_M = 0.065;
inline constexpr double SEATPOST_MASS_KG = 0.35;
inline constexpr double SADDLE_MIN_MASS_KG = 0.20;
inline constexpr double MIN_EXPOSED_SEATPOST_M = 0.050;
inline constexpr double MAX_EXPOSED_SEATPOST_M = 0.250;
inline constexpr Vec3 STEM_TOP_OFFSET_M{0.04, 0.0, 0.02};
inline constexpr double CRANK_ARM_LATERAL_OFFSET_M = 0.075;
inline constexpr double PEDAL_LATERAL_OFFSET_M = 0.115;
inline constexpr Vec3 PEDAL_HALF_SIZE_M{0.050, 0.040, 0.008};

// SaddleGeometry dataclass (field order verbatim; ndarray members are
// fixed Vec3).
struct SaddleGeometry {
    Vec3 seatpost_top;
    Vec3 saddle_pos;
    Vec3 top_center;
    double top_z_m;
    double height_m;
    double exposed_post_m;
};

[[nodiscard]] Vec3 seatpost_top_for_saddle_z(const Vec3 &p9_m,
                                             double saddle_top_z_m);
// saddle_geometry(p9_m, saddle_top_z_m=None) — nullopt mirrors the
// Python None default (top_z = PHOTO_SADDLE_TOP_Z_M).
[[nodiscard]] SaddleGeometry
saddle_geometry(const Vec3 &p9_m,
                std::optional<double> saddle_top_z_m = std::nullopt);
// Throws std::invalid_argument (the Python ValueError) when the height
// is unreachable — the discriminant check is verbatim.
[[nodiscard]] double saddle_top_z_for_height(const Vec3 &p9_m,
                                             double saddle_height_m);

// saddle_post_masses -> (lower post kg, upper post kg, saddle kg).
struct SaddlePostMasses {
    double lower_post;
    double upper_post;
    double saddle;
};
[[nodiscard]] SaddlePostMasses
saddle_post_masses(double saddle_post_mass_kg, const Vec3 &p10_m,
                   const Vec3 &p9_m, const SaddleGeometry &geometry);
[[nodiscard]] Vec3 saddle_post_center_of_mass(double saddle_post_mass_kg,
                                              const Vec3 &p10_m,
                                              const Vec3 &p9_m,
                                              const SaddleGeometry &geometry);
[[nodiscard]] Vec3 handlebar_grip_point(const BikeSpecs &specs);
// -> (front pedal, rear pedal).
[[nodiscard]] std::pair<Vec3, Vec3> pedal_points(double crank_length_m);

// ---------------------------------------------------------------------------
// detail — the shared binary64 building blocks both geometry and
// kinematics TUs port against. These are NOT approximations of the
// oracle: they invoke the same provider entry points numpy 2.5.2's
// ILP64 build resolves on this platform, in the same operation order.
// ---------------------------------------------------------------------------
namespace detail {

    // _cblas_ddot$NEWLAPACK$ILP64 — the Accelerate CBLAS entry point
    // NumPy's x.dot(x) / np.dot dispatch to on this platform (the
    // oracle _multiarray_umath imports the ILP64 alias). __LAPACK_int
    // is `long` under ACCELERATE_LAPACK_ILP64 — same width convention
    // as rider/least_squares.cpp's dgelsd_ilp64 declaration.
    extern "C" double cblas_ddot_ilp64(long n, const double *x, long incx,
                                       const double *y, long incy)
        __asm__("_cblas_ddot$NEWLAPACK$ILP64");

    // np.dot(a, b) on contiguous 1-D doubles — BLAS ddot.
    [[nodiscard]] inline double np_dot(std::span<const double> a,
                                       std::span<const double> b) noexcept {
        return cblas_ddot_ilp64(static_cast<long>(a.size()), a.data(), 1,
                                b.data(), 1);
    }

    // np.linalg.norm(v) on a real 1-D vector: sqrt(x.dot(x)) —
    // numpy/linalg/_linalg.py:2767 + umath sqrt.
    [[nodiscard]] inline double np_norm(std::span<const double> v) noexcept {
        return std::sqrt(cblas_ddot_ilp64(static_cast<long>(v.size()),
                                          v.data(), 1, v.data(), 1));
    }
    [[nodiscard]] inline double np_norm2(const Vec2 &v) noexcept {
        return np_norm(std::span<const double>(v.data(), v.size()));
    }
    [[nodiscard]] inline double np_norm3(const Vec3 &v) noexcept {
        return np_norm(std::span<const double>(v.data(), v.size()));
    }
    [[nodiscard]] inline double np_dot2(const Vec2 &a,
                                        const Vec2 &b) noexcept {
        return np_dot(std::span<const double>(a.data(), a.size()),
                      std::span<const double>(b.data(), b.size()));
    }
    [[nodiscard]] inline double np_dot3(const Vec3 &a,
                                        const Vec3 &b) noexcept {
        return np_dot(std::span<const double>(a.data(), a.size()),
                      std::span<const double>(b.data(), b.size()));
    }

    // np.clip(x, lo, hi) with scalar bounds — clip.cpp's
    // _npy_clip_const_minmax_ floating-point fast path (clip.cpp:189-212):
    // `if (x < min) x = min; if (x > max) x = max;` — NaN input passes
    // through; a NaN bound returns isnan(lo) ? lo : hi for every lane.
    // (Distinct from numeric_numpy.hpp's np_clip, the array-bound loop.)
    [[nodiscard]] inline double np_clip_scalar(double x, double lo,
                                               double hi) noexcept {
        if (std::isnan(lo) || std::isnan(hi))
            return std::isnan(lo) ? lo : hi;
        if (x < lo) x = lo;
        if (x > hi) x = hi;
        return x;
    }

    // np.radians/np.degrees — multiplication by NPY_PI_180 / NPY_180_PI
    // (numpy/_core/src/npymath/npy_math_internal.h.src constants,
    // correctly rounded binary64 values).
    [[nodiscard]] inline double np_radians(double deg) noexcept {
        return deg * 0.017453292519943295;
    }
    [[nodiscard]] inline double np_degrees(double rad) noexcept {
        return rad * 57.29577951308232;
    }

    // np.linspace(start, stop, num): step = (stop - start)/(num-1),
    // y[i] = start + i*step for i < num-1, y[num-1] = stop pinned.
    // Verified bitwise against np.linspace outputs.
    [[nodiscard]] inline std::vector<double>
    np_linspace(double start, double stop, std::size_t num) {
        std::vector<double> y(num);
        if (num == 0) return y;
        if (num == 1) {
            y[0] = start;
            return y;
        }
        const double step = (stop - start) / static_cast<double>(num - 1);
        for (std::size_t i = 0; i + 1 < num; ++i)
            y[i] = static_cast<double>(i) * step + start;
        y[num - 1] = stop;
        return y;
    }

    // np.interp(x, xp, fp) — arr_interp (compiled_base.c:670-739) for a
    // scalar query with vector xp/fp (lenxp > lenx, so no slope
    // precompute): NaN x returns x; x outside [xp[0], xp[-1]] returns
    // the endpoint; exact knot hits return fp[j]; otherwise
    // fma(slope, x - xp[j], fp[j]) — the oracle's mul+add lowers to one
    // fused fmadd under its build, so std::fma is the bitwise form —
    // with the NaN retry from the j+1 knot and the flat fp tie-break.
    [[nodiscard]] inline double np_interp(double x,
                                          std::span<const double> xp,
                                          std::span<const double> fp) {
        const std::size_t n = xp.size();
        if (n == 0)
            throw std::invalid_argument(
                    "array of sample points is empty");
        // compiled_base.c:671-682 dedicated lenxp == 1 loop — it runs
        // BEFORE the NaN check, so a NaN query also yields fp[0].
        if (n == 1) return fp[0];
        if (std::isnan(x)) return x;
        if (x > xp[n - 1]) return fp[n - 1]; // j == lenxp -> rval
        if (x < xp[0]) return fp[0];         // j == -1    -> lval
        // Largest j in [0, n-1] with xp[j] <= x (the oracle's bisection
        // over `key >= arr[imid]`; duplicate knots resolve to the last).
        std::size_t lo = 0, hi = n;
        while (lo < hi) {
            const std::size_t mid = lo + ((hi - lo) >> 1U);
            if (x >= xp[mid]) lo = mid + 1; else hi = mid;
        }
        const std::size_t j = lo - 1;
        if (j == n - 1) return fp[n - 1];
        if (xp[j] == x) return fp[j];
        const double slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
        double out = std::fma(slope, x - xp[j], fp[j]);
        if (std::isnan(out)) {
            out = std::fma(slope, x - xp[j + 1], fp[j + 1]);
            if (std::isnan(out) && fp[j] == fp[j + 1]) out = fp[j];
        }
        return out;
    }

    // np.max(values) — pairwise np.maximum reduce via the shared
    // NEON-fmaxnm twin (numeric_numpy.hpp). Requires nonempty input
    // (np.max on an empty array raises ValueError; call sites supply
    // nonempty arrays by construction).
    [[nodiscard]] inline double np_max(std::span<const double> values) {
        double acc = values[0];
        for (const double v: values.subspan(1))
            acc = numeric::np_maximum(acc, v);
        return acc;
    }

    // Python builtin round(x, 6): CPython's _Py_dg_dtoa mode-3 decimal
    // round + strtod — equal to correctly-rounded fixed-6 formatting
    // followed by correctly-rounded parsing (verified: 0 mismatches on
    // 500,013 values vs round(x,6) — decimal ties cannot occur because
    // (2k+1)/2e6 retains a factor 5^6 and is never a dyadic binary64).
    [[nodiscard]] inline double py_round6(double x) {
        if (!std::isfinite(x)) return x;
        std::array<char, 64> buf{};
        const auto out =
                std::to_chars(buf.data(), std::to_address(buf.end()), x,
                              std::chars_format::fixed, 6);
        double result = 0.0;
        const auto res = std::from_chars(buf.data(), out.ptr, result,
                                         std::chars_format::fixed);
        // to_chars's fixed-6 output is always a complete float token.
        (void)res;
        return result;
    }

    // Python repr(float) / str(float) — CPython PyOS_double_to_string
    // 'r' mode: shortest round-trip digits, fixed notation for decimal
    // exponent in [-4, 16), scientific 'e±NN' (>= 2 exponent digits)
    // otherwise. Used verbatim in the ported ValueError message text.
    [[nodiscard]] inline std::string py_repr(double x) {
        if (std::isnan(x)) return "nan";
        if (std::isinf(x)) return x < 0.0 ? "-inf" : "inf";
        if (x == 0.0) return std::signbit(x) ? "-0.0" : "0.0";
        std::array<char, 64> buf{};
        const auto sci = std::to_chars(buf.data(), std::to_address(buf.end()),
                                       x, std::chars_format::scientific);
        const std::string s(buf.data(), sci.ptr);
        const std::size_t epos = s.find('e');
        std::string mant = s.substr(0, epos);
        const int exp = std::stoi(s.substr(epos + 1));
        const bool neg = mant.front() == '-';
        if (neg) mant.erase(0, 1);
        // digits = mantissa without the decimal point.
        std::string digits;
        for (const char c: mant)
            if (c != '.') digits.push_back(c);
        std::string out;
        if (neg) out.push_back('-');
        if (exp >= -4 && exp < 16) {
            // Fixed notation.
            const int ip = exp + 1; // digits left of the point
            if (ip <= 0) {
                out += "0.";
                out.append(static_cast<std::size_t>(-ip), '0');
                out += digits;
            } else {
                out += digits.substr(0, static_cast<std::size_t>(ip));
                if (static_cast<std::size_t>(ip) < digits.size()) {
                    out.push_back('.');
                    out += digits.substr(static_cast<std::size_t>(ip));
                } else {
                    out.append(static_cast<std::size_t>(ip) - digits.size(),
                               '0');
                    out += ".0";
                }
            }
        } else {
            // Scientific: one digit, optional fraction, e±NN.
            out.push_back(digits[0]);
            if (digits.size() > 1) {
                out.push_back('.');
                out += digits.substr(1);
            }
            out.push_back('e');
            out.push_back(exp < 0 ? '-' : '+');
            const std::string es = std::to_string(exp < 0 ? -exp : exp);
            if (es.size() < 2) out.push_back('0');
            out += es;
        }
        return out;
    }

    // Python '{:g}' / '{:.Ng}' format — std::to_chars general-with-
    // precision is specified to produce the same digit sequence as
    // printf '%.*g' (correctly rounded P significant digits, trailing
    // zeros stripped, %e only when the exponent rule selects it),
    // which is CPython's 'g' rendering; NaN -> "nan", inf -> "inf",
    // -0.0 -> "-0".
    [[nodiscard]] inline std::string py_g(double x, int precision = 6) {
        if (std::isnan(x)) return "nan";
        if (std::isinf(x)) return x < 0.0 ? "-inf" : "inf";
        std::array<char, 128> buf{};
        const auto out =
                std::to_chars(buf.data(), std::to_address(buf.end()), x,
                              std::chars_format::general, precision);
        return {buf.data(), out.ptr};
    }

    // Python '{:.Nf}' fixed format — correctly rounded decimal digits
    // (dyadic inputs cannot land exactly on a decimal tie, so the
    // correctly rounded rendering is unique).
    [[nodiscard]] inline std::string py_fixed(double x, int precision) {
        if (std::isnan(x)) return "nan";
        if (std::isinf(x)) return x < 0.0 ? "-inf" : "inf";
        std::array<char, 256> buf{};
        const auto out =
                std::to_chars(buf.data(), std::to_address(buf.end()), x,
                              std::chars_format::fixed, precision);
        return {buf.data(), out.ptr};
    }

} // namespace detail

} // namespace bike::geometry
