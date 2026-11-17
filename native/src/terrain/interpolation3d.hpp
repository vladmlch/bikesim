#pragma once

#include <array>

namespace bike::terrain {

struct BicubicEval3d {
    double height_m = 0.;
    double dz_dx = 0.;
    double dz_dy = 0.;
};

// Evaluates a C^1 monotone bicubic Hermite patch over normalized cell coordinates
// u, v in [0, 1] from a 4x4 stencil z[ix_local][iy_local] (ix_local, iy_local in 0..3,
// cell spans stencil indices [1..2] x [1..2]) using Steffen (1990) slope limiting.
// Guarantees zero Gibbs overshoot on steps/plateaus and exact reproduction of tilted planes.
[[nodiscard]] BicubicEval3d evaluate_steffen_bicubic(
    const std::array<std::array<double, 4>, 4>& stencil_xy,
    double dx_m, double dy_m, double u, double v) noexcept;

} // namespace bike::terrain
