#pragma once

#include "terrain/grid3d.hpp"
#include "terrain/spec3d.hpp"

namespace bike::terrain {

// 6u^5 - 15u^4 + 10u^3, clamped to [0,1].
double quintic_smoothstep(double u) noexcept;
// Cold generation in strips with halo; invalid input throws invalid_argument.
Grid3d generate_grid3d(const Track3dSpec&);

} // namespace bike::terrain
