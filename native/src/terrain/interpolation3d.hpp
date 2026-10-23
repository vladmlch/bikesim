#pragma once

#include "terrain/grid3d.hpp"

#include <array>
#include <cstdint>

namespace bike::terrain {

struct Catmull {
    static double value(double p0, double p1, double p2, double p3,
                        double t) noexcept;
    static double derivative(double p0, double p1, double p2, double p3,
                             double t) noexcept;
};

struct BicubicPatch {
    std::array<double, 16> c{}; // c[j*4+i] * u^i * v^j.
};

// Linear ghost samples: p[-1]=2p[0]-p[1], p[n]=2p[n-1]-p[n-2].
BicubicPatch bicubic_patch(const Grid3d&, std::uint64_t ix,
                           std::uint64_t iy) noexcept;

struct SurfaceEval {
    double z = 0., dz_du = 0., dz_dv = 0.;
};

SurfaceEval evaluate(const BicubicPatch&, double u, double v) noexcept;

} // namespace bike::terrain
