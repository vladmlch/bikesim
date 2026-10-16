// kinematics/linkage_math.hpp — linkage_math.py's two mathematical
// primitives: brentq_pure (the pure-Python Brent scalar root solver,
// ported statement for statement — every comparison, sign swap and
// convergence expression is the source's own arithmetic, including the
// rtol parameter the source accepts but never consults) and
// circle_circle_intersection_2d (closed-form (X,Z)-plane circle
// intersection with the source's concentric/non-intersection guards
// and message text).
//
// NumPy lowering inside: np.linalg.norm -> detail::np_norm (ILP64
// ddot + sqrt), np.clip scalar-bounds -> detail::np_clip_scalar, and
// the `max(0.0, h_sq)` builtin -> the `h_sq > 0 ? h_sq : 0` first-
// operand tie. Elementwise array arithmetic maps to component ops in
// source order; errors raise std::invalid_argument carrying the Python
// ValueError message verbatim.
#pragma once

#include "../geometry/specs.hpp"

#include <functional>
#include <span>
#include <utility>

namespace bike::kinematics {

    // brentq_pure(f, a, b, xtol=1e-12, rtol=1e-12, maxiter=100) — the
    // source's full keyword surface as trailing defaults, a strict
    // superset of the declared `brentq(f,a,b,xtol)` produced API.
    // `rtol` mirrors the source signature — the Python body never
    // consults it (the convergence expression is xtol-only by source
    // construction), so it is accepted and ignored verbatim.
    // Throws std::invalid_argument with the source's
    // "Root is not bracketed: f({a})={fa:g}, f({b})={fb:g}" text when
    // fa * fb > 0; returns a/b early on exact endpoint roots and the
    // last iterate after `maxiter` iterations otherwise.
    [[nodiscard]] double brentq(const std::function<double(double)> &f,
                                double a, double b,
                                double xtol = 1e-12,
                                [[maybe_unused]] double rtol = 1e-12,
                                int maxiter = 100);

    // circle_circle_intersection_2d(c1, r1, c2, r2) -> (sol1, sol2).
    // Centers are taken as [x, y, z] when they hold 3 elements, else
    // [x, z] (the source's `c[2] if len(c) == 3 else c[1]` indexing);
    // any length >= 2 follows the same rule, matching Python indexing.
    // Throws std::invalid_argument with the source's messages for the
    // concentric (d < 1e-12) and non-intersecting branches.
    [[nodiscard]] std::pair<geometry::Vec2, geometry::Vec2>
    circle_circle_intersection_2d(std::span<const double> c1, double r1,
                                  std::span<const double> c2, double r2);

    // Vec2/Vec3 conveniences — identical calls through the span core.
    [[nodiscard]] inline std::pair<geometry::Vec2, geometry::Vec2>
    circle_circle_intersection_2d(const geometry::Vec2 &c1, double r1,
                                  const geometry::Vec2 &c2, double r2) {
        return circle_circle_intersection_2d(
                std::span<const double>(c1.data(), c1.size()), r1,
                std::span<const double>(c2.data(), c2.size()), r2);
    }
    [[nodiscard]] inline std::pair<geometry::Vec2, geometry::Vec2>
    circle_circle_intersection_2d(const geometry::Vec3 &c1, double r1,
                                  const geometry::Vec3 &c2, double r2) {
        return circle_circle_intersection_2d(
                std::span<const double>(c1.data(), c1.size()), r1,
                std::span<const double>(c2.data(), c2.size()), r2);
    }

} // namespace bike::kinematics
