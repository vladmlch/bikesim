#include "terrain/interpolation3d.hpp"

#include <algorithm>
#include <cmath>

namespace bike::terrain {
namespace {

struct SlopeWithDeriv {
    double slope = 0.;
    double dslope_dx = 0.;
};

// Steffen (1990), "A simple method for monotonic interpolation in one dimension",
// Astron. Astrophys. 239, 443-450: C^1 monotone slope at interior node 1 from (y0, y1, y2)
// on uniform spacing h. Zero slope at extrema and adjacent to flat plateaus.
[[nodiscard]] double steffen_slope(double y0, double y1, double y2, double inv_h) noexcept {
    const double s0 = (y1 - y0) * inv_h;
    const double s1 = (y2 - y1) * inv_h;
    if (s0 * s1 <= 0.) {
        return 0.;
    }
    const double p = 0.5 * (s0 + s1);
    const double bound = 2. * std::min(std::abs(s0), std::abs(s1));
    return std::copysign(std::min(std::abs(p), bound), p);
}

// Same Steffen (1990) slope limiter together with its exact derivative with respect to x
// when the input values (y0, y1, y2) vary smoothly with x at rates (dy0, dy1, dy2).
// NOLINTBEGIN(bugprone-easily-swappable-parameters) (y0..y2, dy0..dy2, inv_h) internal math helper
[[nodiscard]] SlopeWithDeriv steffen_slope_and_deriv(
    double y0, double y1, double y2,
    double dy0, double dy1, double dy2,
    double inv_h) noexcept {
// NOLINTEND(bugprone-easily-swappable-parameters)
    const double s0 = (y1 - y0) * inv_h;
    const double s1 = (y2 - y1) * inv_h;
    const double ds0 = (dy1 - dy0) * inv_h;
    const double ds1 = (dy2 - dy1) * inv_h;

    if (s0 == 0. && s1 == 0.) {
        if (ds0 * ds1 > 0.) {
            const double dp = 0.5 * (ds0 + ds1);
            const double dbound = 2. * std::min(std::abs(ds0), std::abs(ds1));
            return {.slope = 0., .dslope_dx = std::copysign(std::min(std::abs(dp), dbound), dp)};
        }
        return {.slope = 0., .dslope_dx = 0.};
    }

    if (s0 * s1 <= 0.) {
        return {.slope = 0., .dslope_dx = 0.};
    }

    const double p = 0.5 * (s0 + s1);
    const double abs_s0 = std::abs(s0);
    const double abs_s1 = std::abs(s1);
    const double abs_p = std::abs(p);
    const double min_abs = std::min(abs_s0, abs_s1);
    if (abs_p <= 2. * min_abs) {
        return {.slope = p, .dslope_dx = 0.5 * (ds0 + ds1)};
    }
    if (abs_s0 <= abs_s1) {
        return {.slope = 2. * s0, .dslope_dx = 2. * ds0};
    }
    return {.slope = 2. * s1, .dslope_dx = 2. * ds1};
}

struct Hermite1d {
    double val = 0.;
    double deriv = 0.;
};

// NOLINTBEGIN(bugprone-easily-swappable-parameters) (f0, f1, d0_scaled, d1_scaled, t) Hermite basis helper
[[nodiscard]] Hermite1d eval_cubic_hermite(double f0, double f1, double d0_scaled,
                                           double d1_scaled, double t) noexcept {
// NOLINTEND(bugprone-easily-swappable-parameters)
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2. * t3 - 3. * t2 + 1.;
    const double h10 = t3 - 2. * t2 + t;
    const double h01 = -2. * t3 + 3. * t2;
    const double h11 = t3 - t2;

    const double dh00 = 6. * (t2 - t);
    const double dh10 = 3. * t2 - 4. * t + 1.;
    const double dh01 = 6. * (t - t2);
    const double dh11 = 3. * t2 - 2. * t;

    return Hermite1d{
        .val = h00 * f0 + h10 * d0_scaled + h01 * f1 + h11 * d1_scaled,
        .deriv = dh00 * f0 + dh10 * d0_scaled + dh01 * f1 + dh11 * d1_scaled,
    };
}

} // namespace

// NOLINTBEGIN(bugprone-easily-swappable-parameters) (dx_m, dy_m, u, v) matches public evaluate_steffen_bicubic contract
BicubicEval3d evaluate_steffen_bicubic(
    const std::array<std::array<double, 4>, 4>& stencil_xy,
    double dx_m, double dy_m, double u, double v) noexcept {
// NOLINTEND(bugprone-easily-swappable-parameters)
    const double inv_dx = 1. / dx_m;
    const double inv_dy = 1. / dy_m;

    // Step 1: Along X for each of the 4 Y-columns (iy = 0..3), evaluate height and dz/dx at u.
    std::array<double, 4> col_val{};
    std::array<double, 4> col_dzdx{};
    for (std::size_t iy = 0; iy < 4; ++iy) {
        const double sx1 = steffen_slope(stencil_xy[0][iy], stencil_xy[1][iy], stencil_xy[2][iy], inv_dx);
        const double sx2 = steffen_slope(stencil_xy[1][iy], stencil_xy[2][iy], stencil_xy[3][iy], inv_dx);
        const Hermite1d hx = eval_cubic_hermite(stencil_xy[1][iy], stencil_xy[2][iy], sx1 * dx_m, sx2 * dx_m, u);
        col_val[iy] = hx.val;
        col_dzdx[iy] = hx.deriv * inv_dx;
    }

    // Step 2: Monotone interpolation along Y of col_val(v) -> height, exact dz/dx, and dz/dy.
    const SlopeWithDeriv sy1 = steffen_slope_and_deriv(
        col_val[0], col_val[1], col_val[2],
        col_dzdx[0], col_dzdx[1], col_dzdx[2], inv_dy);
    const SlopeWithDeriv sy2 = steffen_slope_and_deriv(
        col_val[1], col_val[2], col_val[3],
        col_dzdx[1], col_dzdx[2], col_dzdx[3], inv_dy);

    const Hermite1d hy = eval_cubic_hermite(
        col_val[1], col_val[2], sy1.slope * dy_m, sy2.slope * dy_m, v);
    const Hermite1d hy_dx = eval_cubic_hermite(
        col_dzdx[1], col_dzdx[2], sy1.dslope_dx * dy_m, sy2.dslope_dx * dy_m, v);

    return BicubicEval3d{
        .height_m = hy.val,
        .dz_dx = hy_dx.val,
        .dz_dy = hy.deriv * inv_dy,
    };
}

} // namespace bike::terrain
