#pragma once

#include "terrain/source3d.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace bike::terrain {

struct RenderResolution {
    std::uint32_t nrow = 0, ncol = 0;
};

struct RenderGrid3d {
    std::uint32_t nrow = 0, ncol = 0;
    std::vector<float> normalized_height;
    double x0_m = 0., y0_m = 0., x1_m = 0., y1_m = 0.;
    double z_min_m = 0., z_max_m = 0.;
    std::string physical_asset_sha256_hex;
    RenderResolution resolution;
};

// Visual area downsample only: physics retains the canonical source.
// Constant-height grids produce normalized zero and z_min == z_max.
RenderGrid3d make_render_grid(const Terrain3dSource&, RenderResolution);

} // namespace bike::terrain
