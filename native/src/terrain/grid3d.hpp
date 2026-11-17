#pragma once

#include "terrain/terrain3d_types.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace bike::terrain {

struct Grid3dSpec {
    std::size_t nx = 0;
    std::size_t ny = 0;
    double x0_m = 0.;
    double y0_m = 0.;
    double dx_m = 0.;
    double dy_m = 0.;
};

class Grid3d {
public:
    // `elevations_xy` has length nx * ny in transverse-slice order: index = ix * ny + iy.
    // `cell_surfaces_xy` has length (nx - 1) * (ny - 1) in order: cell = ix * (ny - 1) + iy.
    Grid3d(Grid3dSpec spec, std::vector<double> elevations_xy,
           std::vector<rider::SurfaceLabel> cell_surfaces_xy);

    [[nodiscard]] const Grid3dSpec& spec() const noexcept { return spec_; }
    [[nodiscard]] const TerrainBounds3d& bounds() const noexcept { return bounds_; }
    [[nodiscard]] const std::string& sha256_hex() const noexcept { return sha256_hex_; }
    [[nodiscard]] std::span<const double> elevations() const noexcept { return elevations_xy_; }

    [[nodiscard]] double node_z(std::size_t ix, std::size_t iy) const noexcept {
        return elevations_xy_[ix * spec_.ny + iy];
    }

    [[nodiscard]] TerrainSample3d sample(double x_m, double y_m) const noexcept;
    [[nodiscard]] TorusClosestPoint3d closest_point(const Vec3d& center_world_m,
                                                    double search_radius_m) const noexcept;

private:
    struct CellZRange {
        double z_min = 0.;
        double z_max = 0.;
    };

    Grid3dSpec spec_{};
    std::vector<double> elevations_xy_;
    std::vector<rider::SurfaceLabel> cell_surfaces_xy_;
    std::vector<CellZRange> cell_z_ranges_;
    TerrainBounds3d bounds_{};
    std::string sha256_hex_{};
};

} // namespace bike::terrain
