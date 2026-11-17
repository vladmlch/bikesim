#pragma once

#include "rider/senses.hpp"

#include <array>
#include <cstdint>

namespace bike::terrain {

using Vec3d = std::array<double, 3>;

enum class QueryStatus3d : std::uint8_t {
    ok,
    outside,
    invalid_input,
};

struct TerrainBounds3d {
    double x_min_m = 0.;
    double x_max_m = 0.;
    double y_min_m = 0.;
    double y_max_m = 0.;
    double z_min_m = 0.;
    double z_max_m = 0.;
};

struct TerrainSample3d {
    QueryStatus3d status = QueryStatus3d::invalid_input;
    double height_m = 0.;
    Vec3d normal_world{0., 0., 1.};
    ::rider::SurfaceLabel surface = ::rider::SurfaceLabel::hardpack;
};

// Closest point on z = h(x, y) within [x - window, x + window] x [y - window, y + window]
// to a torus tube-section centre `center_world_m` (3D generalization of SignedProfile).
// `signed_distance_m` > 0 when center is above the surface, < 0 when below;
// torus indentation is `r_tube - signed_distance_m`.
struct TorusClosestPoint3d {
    QueryStatus3d status = QueryStatus3d::invalid_input;
    double signed_distance_m = 0.;
    Vec3d surface_point_world_m{};
    Vec3d normal_world{0., 0., 1.};
    ::rider::SurfaceLabel surface = ::rider::SurfaceLabel::hardpack;
};

} // namespace bike::terrain
