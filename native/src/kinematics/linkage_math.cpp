// kinematics/linkage_math.cpp — statement-for-statement ports of
// linkage_math.py's brentq_pure and circle_circle_intersection_2d.
// Every arithmetic expression keeps the Python evaluation order —
// `2.0 * m * q * (q - r)` stays ((2.0*m)*q)*(q-r), `b += d` stays one
// rounded add, and the min() call keeps the first-tie-wins builtin
// semantics. Exceptions are std::invalid_argument carrying the Python
// ValueError message verbatim.
#include "linkage_math.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace bike::kinematics {

    double brentq(const std::function<double(double)> &f, double a,
                  double b, double xtol,
                  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (rtol, maxiter) keep the source's parameter order
                  [[maybe_unused]] double rtol, int maxiter) {
        double fa = f(a);
        double fb = f(b);
        if (fa * fb > 0.0)
            throw std::invalid_argument(
                    "Root is not bracketed: f(" +
                    geometry::detail::py_repr(a) + ")=" +
                    geometry::detail::py_g(fa) + ", f(" +
                    geometry::detail::py_repr(b) + ")=" +
                    geometry::detail::py_g(fb));
        if (fa == 0.0) return a;
        if (fb == 0.0) return b;

        double c = a;
        double fc = fa;
        double d = b - a;
        double e = d; // d = e = b - a (chained assignment, same value)

        for (int iter = 0; iter < maxiter; ++iter) {
            if ((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0)) {
                c = a;
                fc = fa;
                d = e = b - a;
            }
            if (std::fabs(fc) < std::fabs(fb)) {
                const double ta = b, tb = c, tc = b;
                a = ta; b = tb; c = tc;
                const double tfa = fb, tfb = fc, tfc = fb;
                fa = tfa; fb = tfb; fc = tfc;
            }

            const double tol =
                    2.0 * 1e-16 * std::fabs(b) + 0.5 * xtol;
            const double m = 0.5 * (c - b);

            if (std::fabs(m) <= tol || fb == 0.0) return b;

            if (std::fabs(e) >= tol && std::fabs(fa) > std::fabs(fb)) {
                const double s = fb / fa;
                // Both branches assign p and q before the sign flip
                // reads them; the zero-init only satisfies the
                // init-variables lint (the source declares then assigns).
                double p{0.0}, q{0.0};
                if (a == c) {
                    // Linear interpolation (Secant method)
                    p = 2.0 * m * s;
                    q = 1.0 - s;
                } else {
                    // Inverse quadratic interpolation
                    q = fa / fc;
                    const double r = fb / fc;
                    p = s * (2.0 * m * q * (q - r) -
                             (b - a) * (r - 1.0));
                    q = (q - 1.0) * (r - 1.0) * (s - 1.0);
                }
                if (p > 0.0) {
                    q = -q;
                } else {
                    p = -p;
                }
                // min(3.0*m*q - abs(tol*q), abs(e*q)) — builtin min:
                // first-tie-wins `bm < am ? bm : am`.
                const double am = 3.0 * m * q - std::fabs(tol * q);
                const double bm = std::fabs(e * q);
                const double min_v = bm < am ? bm : am;
                if (2.0 * p < min_v) {
                    e = d;
                    d = p / q;
                } else {
                    d = e = m;
                }
            } else {
                d = e = m;
            }

            a = b;
            fa = fb;
            if (std::fabs(d) > tol) {
                b = b + d; // b += d
            } else if (m > 0.0) {
                b = b + tol; // b += tol
            } else {
                b = b - tol; // b -= tol
            }
            fb = f(b);
        }

        return b;
    }

    std::pair<geometry::Vec2, geometry::Vec2>
    circle_circle_intersection_2d(std::span<const double> c1, double r1,
                                  std::span<const double> c2,
                                  double r2) {
        using geometry::Vec2;
        // [x, z] extraction: index 2 for 3-element centers else 1.
        if (c1.size() < 2 || c2.size() < 2)
            throw std::out_of_range(
                    "circle center needs at least 2 elements");
        const Vec2 p1{c1[0], c1.size() == 3 ? c1[2] : c1[1]};
        const Vec2 p2{c2[0], c2.size() == 3 ? c2[2] : c2[1]};

        const Vec2 d_vec{p2[0] - p1[0], p2[1] - p1[1]};
        const double d = geometry::detail::np_norm2(d_vec);

        if (d < 1e-12)
            throw std::invalid_argument(
                    "Circles are concentric; infinite or zero "
                    "intersections.");
        const double r_sum = r1 + r2;
        const double r_diff = std::fabs(r1 - r2);
        if (d > r_sum + 1e-9 || d < r_diff - 1e-9)
            throw std::invalid_argument(
                    "Circles do not intersect: distance " +
                    geometry::detail::py_fixed(d, 4) + " outside [" +
                    geometry::detail::py_fixed(r_diff, 4) + ", " +
                    geometry::detail::py_fixed(r_sum, 4) + "]");

        // Clamp d for numerical stability if tangent.
        const double d_clamped =
                geometry::detail::np_clip_scalar(d, r_diff, r_sum);
        const double a = (r1 * r1 - r2 * r2 + d_clamped * d_clamped) /
                         (2.0 * d_clamped);
        const double h_sq = r1 * r1 - a * a;
        // max(0.0, h_sq): builtin first-tie-wins — h_sq > 0 ? h_sq : 0.
        const double h = std::sqrt(h_sq > 0.0 ? h_sq : 0.0);

        const Vec2 u{d_vec[0] / d, d_vec[1] / d};
        const Vec2 n{-u[1], u[0]}; // 90-degree CCW normal

        // p_mid = p1 + a*u ; sol1 = p_mid + h*n ; sol2 = p_mid - h*n —
        // elementwise scalar*vector then add/sub, source order.
        const Vec2 p_mid{p1[0] + a * u[0], p1[1] + a * u[1]};
        const Vec2 sol1{p_mid[0] + h * n[0], p_mid[1] + h * n[1]};
        const Vec2 sol2{p_mid[0] - h * n[0], p_mid[1] - h * n[1]};

        return {sol1, sol2};
    }

} // namespace bike::kinematics
