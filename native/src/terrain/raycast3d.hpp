#pragma once

#include "terrain/grid3d.hpp"
#include "terrain/interpolation3d.hpp"

#include <array>
#include <cstdint>

namespace bike::terrain {

struct Poly6 {
    std::array<double, 7> a{}; // a[k]*s^k, s in [0,1] within one cell.
};

Poly6 line_minus_surface(const BicubicPatch&, std::array<double, 2> uv0,
                         std::array<double, 2> duv, double z0,
                         double dz) noexcept;

struct RootSearchResult {
    QueryStatus3d status = QueryStatus3d::miss;
    double s = 0.;
    std::uint32_t subdivisions = 0;
};

// Deterministic left-first bisection; tangencies use stationary points.
// Exhaustion is explicit and must never be reported as a proven miss.
RootSearchResult earliest_root_bernstein(const Poly6&, const RayBudget&) noexcept;
RayHit3d ray_cast_grid(const Grid3d&, std::array<double, 3> origin_m,
                      std::array<double, 3> unit_direction,
                      double max_distance_m) noexcept;

} // namespace bike::terrain
