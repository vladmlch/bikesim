#pragma once

#include "io/value.hpp"
#include "terrain/grid3d.hpp"

#include <array>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

namespace bike::terrain {

class ProfileExtrusion3d {
public:
    static std::expected<ProfileExtrusion3d, std::string> create(
        std::vector<double> knots_x_m, std::vector<double> knots_z_m,
        std::vector<std::uint32_t> segment_surface_ids,
        std::vector<SurfaceSpec3d>, double corridor_half_width_m,
        bike::io::Value source_track_toml);
    GridSample3d sample(double x_m, double y_m) const noexcept;
    RayHit3d ray_cast(std::array<double, 3> origin_m,
                     std::array<double, 3> unit_direction,
                     double max_distance_m) const noexcept;
    double corridor_half_width_m() const noexcept;
    std::span<const double> knots_x_m() const noexcept;
    std::span<const double> knots_z_m() const noexcept;
    // Hash includes knots, surfaces, corridor and source bytes.
    const std::string& fingerprint_hex() const noexcept;

private:
    std::vector<double> kx_, kz_;
    std::vector<std::uint32_t> seg_surface_;
    std::vector<SurfaceSpec3d> surfaces_;
    double half_width_ = 0.;
    bike::io::Value source_;
    std::string fingerprint_hex_;
};

} // namespace bike::terrain
