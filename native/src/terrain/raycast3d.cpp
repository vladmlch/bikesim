#include "raycast3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace bike::terrain {
namespace {

using Real = long double;
using Bernstein = std::array<Real, 7>;

Real choose(unsigned n, unsigned k) noexcept {
    Real result = 1.;
    for (unsigned i = 1; i <= k; ++i)
        result = result * static_cast<Real>(n + 1 - i) / static_cast<Real>(i);
    return result;
}

Real value(const Poly6& p, Real x) noexcept {
    Real result = p.a[6];
    for (std::size_t i = 6; i != 0; --i) result = result * x + p.a[i - 1];
    return result;
}

Real derivative(const Poly6& p, Real x) noexcept {
    Real result = 6. * p.a[6];
    for (std::size_t i = 5; i != 0; --i)
        result = result * x + static_cast<Real>(i) * p.a[i];
    return result;
}

Real second_derivative(const Poly6& p, Real x) noexcept {
    Real result = 30. * p.a[6];
    for (std::size_t i = 5; i > 1; --i)
        result = result * x + static_cast<Real>(i * (i - 1)) * p.a[i];
    return result;
}

bool opposite(Real a, Real b) noexcept { return (a < 0. && b > 0.) || (a > 0. && b < 0.); }

Bernstein to_bernstein(const Poly6& p) noexcept {
    Bernstein result{};
    for (unsigned k = 0; k <= 6; ++k)
        for (unsigned j = 0; j <= k; ++j)
            result[k] += static_cast<Real>(p.a[j]) * choose(k, j) / choose(6, j);
    return result;
}

void bisect(const Bernstein& input, Bernstein& left, Bernstein& right) noexcept {
    auto work = input;
    left[0] = work[0];
    right[6] = work[6];
    for (std::size_t depth = 1; depth <= 6; ++depth) {
        for (std::size_t i = 0; i <= 6 - depth; ++i) work[i] = (work[i] + work[i + 1]) / 2.;
        left[depth] = work[0];
        right[6 - depth] = work[6 - depth];
    }
}

unsigned derivative_variations(const Bernstein& b) noexcept {
    unsigned changes = 0;
    int last = 0;
    for (std::size_t i = 0; i < 6; ++i) {
        const Real d = 6. * (b[i + 1] - b[i]);
        const int sign = d > 0. ? 1 : d < 0. ? -1 : 0;
        if (sign != 0) {
            if (last != 0 && sign != last) ++changes;
            last = sign;
        }
    }
    return changes;
}

struct Interval {
    Real lo = 0., hi = 1.;
    Bernstein b{};
};

bool valid_budget(const RayBudget& budget) noexcept {
    return budget.stack_capacity > 0 && budget.stack_capacity <= 128 &&
        std::isfinite(budget.root_interval_m) && budget.root_interval_m > 0. &&
        std::isfinite(budget.residual_m) && budget.residual_m > 0.;
}

bool clip_axis(double origin, double direction, double lo, double hi,
               double& enter, double& exit) noexcept {
    if (direction == 0.) return origin >= lo && origin <= hi;
    double a = (lo - origin) / direction, b = (hi - origin) / direction;
    if (a > b) std::swap(a, b);
    enter = std::max(enter, a);
    exit = std::min(exit, b);
    return enter <= exit;
}

bool inside(const Grid3d& grid, const std::array<double, 3>& point) noexcept {
    const auto& d = grid.descriptor();
    return point[0] >= d.x0_m && point[0] <= grid.x_max_m() &&
        point[1] >= d.y0_m && point[1] <= grid.y_max_m();
}

std::array<double, 3> point_at(const Grid3d& grid, const std::array<double, 3>& origin,
                             const std::array<double, 3>& direction, double t) noexcept {
    // This helper is used only in the already clipped interval. Clamp restores
    // the exact closed XY boundary after roundoff in slab t*direction.
    const auto& d = grid.descriptor();
    return {std::clamp(origin[0] + t * direction[0], d.x0_m, grid.x_max_m()),
            std::clamp(origin[1] + t * direction[1], d.y0_m, grid.y_max_m()),
            origin[2] + t * direction[2]};
}

RayHit3d hit_at(const Grid3d& grid, const std::array<double, 3>& origin,
               const std::array<double, 3>& direction, double t) noexcept {
    const auto point = point_at(grid, origin, direction, t);
    const auto sample = grid.sample(point[0], point[1]);
    if (sample.status != QueryStatus3d::ok || !std::isfinite(point[2])) return {};
    return {.status = QueryStatus3d::ok, .distance_m = t, .point_world_m = point,
            .normal_world = sample.normal_world, .surface_id = sample.surface_id};
}

std::int64_t initial_cell(double position, double low, double spacing,
                          std::uint64_t nodes, double direction) noexcept {
    const double coordinate = (position - low) / spacing;
    const double rounded = std::floor(coordinate);
    auto cell = static_cast<std::int64_t>(rounded);
    if (direction < 0. && coordinate == rounded) --cell;
    return std::clamp(cell, std::int64_t{0}, static_cast<std::int64_t>(nodes - 2));
}

} // namespace

Poly6 line_minus_surface(const BicubicPatch& patch, std::array<double, 2> uv0,
                         std::array<double, 2> duv, double z0, double dz) noexcept {
    std::array<std::array<double, 4>, 4> u{}, v{};
    u[0][0] = v[0][0] = 1.;
    for (std::size_t n = 1; n < 4; ++n) {
        for (std::size_t k = 0; k <= n; ++k) {
            u[n][k] = (k < n ? uv0[0] * u[n - 1][k] : 0.) + (k > 0 ? duv[0] * u[n - 1][k - 1] : 0.);
            v[n][k] = (k < n ? uv0[1] * v[n - 1][k] : 0.) + (k > 0 ? duv[1] * v[n - 1][k - 1] : 0.);
        }
    }
    Poly6 result;
    result.a[0] = z0;
    result.a[1] = dz;
    for (std::size_t j = 0; j < 4; ++j)
        for (std::size_t i = 0; i < 4; ++i)
            for (std::size_t a = 0; a <= i; ++a)
                for (std::size_t b = 0; b <= j; ++b)
                    result.a[a + b] -= patch.c[j * 4 + i] * u[i][a] * v[j][b];
    return result;
}

RootSearchResult earliest_root_bernstein(const Poly6& polynomial, const RayBudget& budget) noexcept {
    if (!valid_budget(budget) ||
        !std::all_of(polynomial.a.begin(), polynomial.a.end(), [](double x) { return std::isfinite(x); }))
        return {.status = QueryStatus3d::invalid_input};
    std::array<Interval, 128> stack{};
    std::size_t size = 1;
    stack[0].b = to_bernstein(polynomial);
    std::uint32_t divisions = 0;
    const Real tolerance = std::min<Real>(1., budget.root_interval_m);
    Real coefficient_scale = 1.;
    for (const double a : polynomial.a) coefficient_scale = std::max(coefficient_scale, std::abs(static_cast<Real>(a)));
    // Poly6 was assembled in binary64. Keep a conservative arithmetic envelope
    // around zero so a rounded analytic tangent is not proved away. This is
    // much smaller than the physical residual; e.g. a 2e-8 m clearance is not
    // accepted as a tangent merely because the query's interval is small.
    const Real roundoff = 64. * std::numeric_limits<double>::epsilon() * coefficient_scale;
    const auto exhausted = [&]() { return RootSearchResult{.status = QueryStatus3d::budget_exhausted, .s = 0., .subdivisions = divisions}; };
    const auto root = [&](Real s) { return RootSearchResult{.status = QueryStatus3d::ok, .s = static_cast<double>(s), .subdivisions = divisions}; };
    while (size != 0) {
        const auto interval = stack[--size];
        const Real fa = value(polynomial, interval.lo), fb = value(polynomial, interval.hi);
        if (fa == 0.) return root(interval.lo);
        const auto [minimum, maximum] = std::minmax_element(interval.b.begin(), interval.b.end());
        if (*minimum > roundoff || *maximum < -roundoff) continue; // Convex hull proves no root.
        if (interval.hi - interval.lo <= tolerance) {
            if (opposite(fa, fb) || fb == 0.) {
                Real candidate = fb == 0. ? interval.hi :
                    interval.lo + (interval.hi - interval.lo) * fa / (fa - fb);
                for (unsigned iteration = 0; iteration < 3; ++iteration) {
                    const Real d = derivative(polynomial, candidate);
                    if (d == 0.) break;
                    const Real next = candidate - value(polynomial, candidate) / d;
                    if (next < interval.lo || next > interval.hi) break;
                    candidate = next;
                }
                if (std::abs(value(polynomial, candidate)) <= budget.residual_m) return root(candidate);
            } else {
                // Same-sign endpoint values alone do not exclude a tangent or
                // two hits. Isolate a stationary point of the derivative, with
                // a Bernstein sign-variation proof when it is the sole root.
                Real lo = interval.lo, hi = interval.hi;
                Real da = derivative(polynomial, lo), db = derivative(polynomial, hi);
                const unsigned variations = derivative_variations(interval.b);
                if (da == 0. || db == 0. || (variations == 1 && opposite(da, db))) {
                    Real stationary = da == 0. ? lo : hi;
                    if (da != 0. && db != 0.) {
                        while (hi - lo > tolerance / 8.) {
                            if (divisions == budget.max_subdivisions) return exhausted();
                            const Real mid = (lo + hi) / 2.;
                            if (mid == lo || mid == hi) break;
                            ++divisions;
                            const Real dm = derivative(polynomial, mid);
                            if (dm == 0.) { lo = hi = mid; break; }
                            if (opposite(da, dm)) { hi = mid; db = dm; }
                            else { lo = mid; da = dm; }
                        }
                        stationary = (lo + hi) / 2.;
                        for (unsigned iteration = 0; iteration < 3; ++iteration) {
                            const Real second = second_derivative(polynomial, stationary);
                            if (second == 0.) break;
                            const Real next = stationary - derivative(polynomial, stationary) / second;
                            if (next < lo || next > hi) break;
                            stationary = next;
                        }
                    }
                    if (std::abs(value(polynomial, stationary)) <= budget.residual_m) return root(stationary);
                    // One stationary point, same-sign endpoints and a same-sign
                    // extremum prove a miss on this interval.
                    const Real fs = value(polynomial, stationary);
                    if (variations <= 1 && !opposite(fa, fs) && !opposite(fb, fs)) continue;
                } else if (variations == 0) {
                    continue; // Monotone and same-sign endpoints.
                }
            }
        }
        if (divisions == budget.max_subdivisions || size + 2 > budget.stack_capacity) return exhausted();
        const Real mid = (interval.lo + interval.hi) / 2.;
        if (mid == interval.lo || mid == interval.hi) return exhausted();
        Interval left{.lo = interval.lo, .hi = mid}, right{.lo = mid, .hi = interval.hi};
        bisect(interval.b, left.b, right.b);
        ++divisions;
        stack[size++] = right;
        stack[size++] = left;
    }
    return {.status = QueryStatus3d::miss, .s = 0., .subdivisions = divisions};
}

RayHit3d ray_cast_grid(const Grid3d& grid, std::array<double, 3> origin,
                      std::array<double, 3> direction, double max_distance) noexcept {
    if (!std::isfinite(max_distance) || max_distance < 0. || !valid_budget(grid.budget()) ||
        !std::all_of(origin.begin(), origin.end(), [](double x) { return std::isfinite(x); }) ||
        !std::all_of(direction.begin(), direction.end(), [](double x) { return std::isfinite(x); }) ||
        std::abs(std::hypot(direction[0], direction[1], direction[2]) - 1.) > 1e-9)
        return {.status = QueryStatus3d::invalid_input};
    const auto& d = grid.descriptor();
    if (inside(grid, origin)) {
        const auto sample = grid.sample(origin[0], origin[1]);
        if (sample.status != QueryStatus3d::ok) return {};
        if (origin[2] < sample.height_m) return {.status = QueryStatus3d::origin_below_surface};
    }
    if (std::abs(direction[0]) < 1e-15 && std::abs(direction[1]) < 1e-15) {
        if (!inside(grid, origin)) return {.status = QueryStatus3d::outside};
        const auto sample = grid.sample(origin[0], origin[1]);
        if (sample.status != QueryStatus3d::ok) return {};
        if (direction[2] >= 0.) return {.status = QueryStatus3d::miss};
        const double t = (origin[2] - sample.height_m) / -direction[2];
        if (t > max_distance) return {.status = QueryStatus3d::miss};
        return hit_at(grid, origin, direction, t);
    }
    double enter = 0., exit = max_distance;
    if (!clip_axis(origin[0], direction[0], d.x0_m, grid.x_max_m(), enter, exit) ||
        !clip_axis(origin[1], direction[1], d.y0_m, grid.y_max_m(), enter, exit))
        return {.status = QueryStatus3d::outside};
    const auto initial = point_at(grid, origin, direction, enter);
    const auto initial_sample = grid.sample(initial[0], initial[1]);
    if (initial_sample.status != QueryStatus3d::ok || !std::isfinite(initial[2])) return {};
    if (initial[2] < initial_sample.height_m) return {.status = QueryStatus3d::origin_below_surface};
    if (initial[2] == initial_sample.height_m) return hit_at(grid, origin, direction, enter);
    if (enter == exit) return {.status = QueryStatus3d::miss};
    auto ix = initial_cell(initial[0], d.x0_m, d.dx_m, grid.nx(), direction[0]);
    auto iy = initial_cell(initial[1], d.y0_m, d.dy_m, grid.ny(), direction[1]);
    const int step_x = direction[0] > 0. ? 1 : direction[0] < 0. ? -1 : 0;
    const int step_y = direction[1] > 0. ? 1 : direction[1] < 0. ? -1 : 0;
    double t = enter;
    std::uint32_t cells = 0;
    while (t < exit) {
        if (cells == grid.budget().max_cells) return {.status = QueryStatus3d::budget_exhausted};
        ++cells;
        if (ix < 0 || iy < 0 || static_cast<std::uint64_t>(ix) >= grid.nx() - 1 ||
            static_cast<std::uint64_t>(iy) >= grid.ny() - 1) return {.status = QueryStatus3d::miss};
        const double bx = d.x0_m + static_cast<double>(ix + (step_x > 0 ? 1 : 0)) * d.dx_m;
        const double by = d.y0_m + static_cast<double>(iy + (step_y > 0 ? 1 : 0)) * d.dy_m;
        const double tx = step_x == 0 ? std::numeric_limits<double>::infinity() : (bx - origin[0]) / direction[0];
        const double ty = step_y == 0 ? std::numeric_limits<double>::infinity() : (by - origin[1]) / direction[1];
        // A clipped point can round onto the previous boundary. Crossing both
        // axes together handles corners without zero-length repeated cells.
        if (tx <= t || ty <= t) {
            if (tx <= t) ix += step_x;
            if (ty <= t) iy += step_y;
            continue;
        }
        const double next = std::min({tx, ty, exit});
        const double length = next - t;
        const auto point = point_at(grid, origin, direction, t);
        const auto patch = bicubic_patch(grid, static_cast<std::uint64_t>(ix), static_cast<std::uint64_t>(iy));
        const std::array<double, 2> uv0{
            (point[0] - (d.x0_m + static_cast<double>(ix) * d.dx_m)) / d.dx_m,
            (point[1] - (d.y0_m + static_cast<double>(iy) * d.dy_m)) / d.dy_m};
        const auto polynomial = line_minus_surface(patch, uv0,
            {direction[0] * length / d.dx_m, direction[1] * length / d.dy_m},
            point[2], direction[2] * length);
        auto local_budget = grid.budget();
        local_budget.root_interval_m = std::min(1., grid.budget().root_interval_m / length);
        const auto found = earliest_root_bernstein(polynomial, local_budget);
        if (found.status == QueryStatus3d::ok) {
            const double distance = t + found.s * length;
            auto result = hit_at(grid, origin, direction, distance);
            if (result.status != QueryStatus3d::ok) return result;
            const auto surface = grid.sample(result.point_world_m[0], result.point_world_m[1]);
            if (std::abs(result.point_world_m[2] - surface.height_m) > grid.budget().residual_m)
                return {.status = QueryStatus3d::budget_exhausted};
            return result;
        }
        if (found.status != QueryStatus3d::miss) return {.status = found.status};
        t = next;
        if (tx == next) ix += step_x;
        if (ty == next) iy += step_y;
    }
    return {.status = QueryStatus3d::miss};
}

} // namespace bike::terrain
