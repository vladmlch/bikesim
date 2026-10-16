// geometry/cockpit.cpp — cockpit.py's saddle/seatpost/grip/pedal
// placement in metres. Every expression keeps the oracle's op order:
// SEATPOST_AXIS is the (-63, 0, 194) mm photo vector divided
// elementwise by its np.linalg.norm (ILP64 ddot -> sqrt), the saddle
// height quadratic is evaluated exactly as written, and the mass split
// keeps the builtin round(x, 6) / max first-tie-wins semantics.
#include "specs.hpp"

#include "../diag.hpp"

#include <stdexcept>
#include <string>

namespace bike::geometry {

namespace {
    // _SEATPOST_AXIS_RAW = [-146.0 - -83.0, 0.0, 624.0 - 430.0] — the
    // subtractions are evaluated once, exactly like the ndarray
    // literal at module init.
    [[nodiscard]] Vec3 seatpost_axis_raw() noexcept {
        return {-146.0 - -83.0, 0.0, 624.0 - 430.0};
    }

    // SEATPOST_AXIS = _SEATPOST_AXIS_RAW / float(np.linalg.norm(raw)):
    // one elementwise scalar division per component, one rounded op
    // each. The init is noexcept — np_norm3 cannot throw — so the
    // exported namespace-scope constant's initialization is
    // non-throwing (cert-err58-cpp) just like the module-level ndarray.
    [[nodiscard]] Vec3 seatpost_axis_init() noexcept {
        const Vec3 raw = seatpost_axis_raw();
        const double n = detail::np_norm3(raw);
        return {raw[0] / n, raw[1] / n, raw[2] / n};
    }
} // namespace

// Public export — cockpit.py's module-level SEATPOST_AXIS ndarray.
const Vec3 SEATPOST_AXIS = seatpost_axis_init();

Vec3 seatpost_top_for_saddle_z(const Vec3 &p9_m,
                               double saddle_top_z_m) {
    const double saddle_underside_z =
            saddle_top_z_m - 2.0 * SADDLE_HALF_THICKNESS_M;
    // p9_m + SEATPOST_AXIS * ((underside - p9_m[2]) / SEATPOST_AXIS[2]):
    // scalar*array product first, then the elementwise add.
    const double t =
            (saddle_underside_z - p9_m[2]) / SEATPOST_AXIS[2];
    return {p9_m[0] + SEATPOST_AXIS[0] * t,
            p9_m[1] + SEATPOST_AXIS[1] * t,
            p9_m[2] + SEATPOST_AXIS[2] * t};
}

// Returned by value deliberately (96-byte record mirrors the source's
// dict); -Wlarge-by-value-copy cannot split by-value parameters (its
// useful half) from return values — same suppression precedent as
// writers/tire.cpp's make_patch.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
SaddleGeometry saddle_geometry(
        const Vec3 &p9_m, std::optional<double> saddle_top_z_m) {
    const double top_z = saddle_top_z_m.has_value()
                                 ? *saddle_top_z_m
                                 : PHOTO_SADDLE_TOP_Z_M;
    const Vec3 seatpost_top = seatpost_top_for_saddle_z(p9_m, top_z);
    const Vec3 saddle_pos{seatpost_top[0] - SADDLE_SETBACK_FROM_POST_TOP_M,
                          0.0,
                          seatpost_top[2] + SADDLE_HALF_THICKNESS_M};
    const Vec3 top_center{saddle_pos[0] + 0.0,
                          saddle_pos[1] + 0.0,
                          saddle_pos[2] + SADDLE_HALF_THICKNESS_M};
    return SaddleGeometry{
            .seatpost_top = seatpost_top,
            .saddle_pos = saddle_pos,
            .top_center = top_center,
            .top_z_m = top_z,
            .height_m = detail::np_norm3(top_center),
            .exposed_post_m = detail::np_norm3(Vec3{
                    seatpost_top[0] - p9_m[0],
                    seatpost_top[1] - p9_m[1],
                    seatpost_top[2] - p9_m[2]})};
}
NATIVE_DIAG_POP

double saddle_top_z_for_height(const Vec3 &p9_m,
                               double saddle_height_m) {
    const double s = SEATPOST_AXIS[0] / SEATPOST_AXIS[2];
    const double x0 =
            p9_m[0] - SADDLE_SETBACK_FROM_POST_TOP_M -
            s * (2.0 * SADDLE_HALF_THICKNESS_M + p9_m[2]);
    const double a = 1.0 + s * s;
    const double b = 2.0 * x0 * s;
    const double c = x0 * x0 - saddle_height_m * saddle_height_m;
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0)
        throw std::invalid_argument(
                "no saddle position on the seatpost axis is " +
                detail::py_fixed(saddle_height_m, 3) +
                " m from the BB");
    return (-b + std::sqrt(disc)) / (2.0 * a);
}

SaddlePostMasses saddle_post_masses(double saddle_post_mass_kg,
                                    const Vec3 &p10_m, const Vec3 &p9_m,
                                    const SaddleGeometry &geometry) {
    const double lower_len = detail::np_norm3(
            Vec3{p9_m[0] - p10_m[0], p9_m[1] - p10_m[1],
                 p9_m[2] - p10_m[2]});
    const double upper_len = geometry.exposed_post_m;
    const double lower = detail::py_round6(
            SEATPOST_MASS_KG * lower_len / (lower_len + upper_len));
    const double upper = SEATPOST_MASS_KG - lower;
    // builtin max(SADDLE_MIN_MASS_KG, m): the first operand keeps ties —
    // `m > min ? m : min` — NaN m resolves to the first (min) operand
    // because NaN > x is false.
    const double m = saddle_post_mass_kg - SEATPOST_MASS_KG;
    const double saddle =
            m > SADDLE_MIN_MASS_KG ? m : SADDLE_MIN_MASS_KG;
    return SaddlePostMasses{.lower_post = lower,
                            .upper_post = upper,
                            .saddle = saddle};
}

Vec3 saddle_post_center_of_mass(double saddle_post_mass_kg,
                                const Vec3 &p10_m, const Vec3 &p9_m,
                                const SaddleGeometry &geometry) {
    const SaddlePostMasses masses =
            saddle_post_masses(saddle_post_mass_kg, p10_m, p9_m, geometry);
    const double lower = masses.lower_post;
    const double upper = masses.upper_post;
    const double saddle = masses.saddle;
    // centre = lower*0.5*(p10+p9) + upper*0.5*(p9+seatpost_top)
    //          + saddle*saddle_pos — elementwise, (A + B) + C order.
    Vec3 centre{};
    for (std::size_t i = 0; i < 3; ++i) {
        const double t1 =
                (lower * 0.5) * (p10_m[i] + p9_m[i]);
        const double t2 =
                (upper * 0.5) * (p9_m[i] + geometry.seatpost_top[i]);
        const double t3 = saddle * geometry.saddle_pos[i];
        centre[i] = (t1 + t2) + t3;
    }
    const double total = (lower + upper) + saddle;
    return {centre[0] / total, centre[1] / total, centre[2] / total};
}

Vec3 handlebar_grip_point(const BikeSpecs &specs) {
    const FixedFramePoints fixed = get_fixed_frame_points(specs);
    const Vec3 p11_m{fixed.P11[0] / 1000.0, fixed.P11[1] / 1000.0,
                     fixed.P11[2] / 1000.0};
    return {p11_m[0] + STEM_TOP_OFFSET_M[0],
            p11_m[1] + STEM_TOP_OFFSET_M[1],
            p11_m[2] + STEM_TOP_OFFSET_M[2]};
}

std::pair<Vec3, Vec3> pedal_points(double crank_length_m) {
    const Vec3 front{crank_length_m, 0.0, 0.0};
    const Vec3 rear{-crank_length_m, 0.0, 0.0};
    return {front, rear};
}

} // namespace bike::geometry
