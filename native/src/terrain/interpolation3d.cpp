#include "interpolation3d.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace bike::terrain {
namespace {

std::array<double, 4> coefficients(double p0, double p1, double p2, double p3) noexcept {
    return {p1, (p2 - p0) / 2., p0 - 2.5 * p1 + 2. * p2 - .5 * p3,
            (p3 - p0) / 2. + 1.5 * (p1 - p2)};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
double row_value(const Grid3d& grid, std::int64_t x, std::uint64_t y) noexcept {
    const auto values = grid.elevation();
    const auto row = static_cast<std::size_t>(y * grid.nx());
    if (x < 0) return 2. * values[row] - values[row + 1];
    if (static_cast<std::uint64_t>(x) >= grid.nx())
        return 2. * values[row + static_cast<std::size_t>(grid.nx() - 1)] -
               values[row + static_cast<std::size_t>(grid.nx() - 2)];
    return values[row + static_cast<std::size_t>(x)];
}

double ghost_value(const Grid3d& grid, std::int64_t x, std::int64_t y) noexcept {
    if (y < 0) return 2. * row_value(grid, x, 0) - row_value(grid, x, 1);
    if (static_cast<std::uint64_t>(y) >= grid.ny())
        return 2. * row_value(grid, x, grid.ny() - 1) - row_value(grid, x, grid.ny() - 2);
    return row_value(grid, x, static_cast<std::uint64_t>(y));
}

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
double Catmull::value(double p0, double p1, double p2, double p3, double t) noexcept {
    const auto a = coefficients(p0, p1, p2, p3);
    return ((a[3] * t + a[2]) * t + a[1]) * t + a[0];
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
double Catmull::derivative(double p0, double p1, double p2, double p3, double t) noexcept {
    const auto a = coefficients(p0, p1, p2, p3);
    return (3. * a[3] * t + 2. * a[2]) * t + a[1];
}

BicubicPatch bicubic_patch(const Grid3d& grid, std::uint64_t ix, std::uint64_t iy) noexcept {
    std::array<std::array<double, 4>, 4> rows{};
    for (std::size_t j = 0; j < 4; ++j) {
        const auto y = static_cast<std::int64_t>(iy) + static_cast<std::int64_t>(j) - 1;
        const auto x = static_cast<std::int64_t>(ix);
        rows[j] = coefficients(ghost_value(grid, x - 1, y), ghost_value(grid, x, y),
                               ghost_value(grid, x + 1, y), ghost_value(grid, x + 2, y));
    }
    BicubicPatch result;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto column = coefficients(rows[0][i], rows[1][i], rows[2][i], rows[3][i]);
        for (std::size_t j = 0; j < 4; ++j) result.c[j * 4 + i] = column[j];
    }
    return result;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (u, v) mirrors the reference patch-coordinates order
SurfaceEval evaluate(const BicubicPatch& patch, double u, double v) noexcept {
    std::array<double, 4> row{}, du{};
    for (std::size_t j = 0; j < 4; ++j) {
        // Indexed (not pointer-offset) access keeps -Wunsafe-buffer-usage quiet.
        const double a0 = patch.c[j * 4], a1 = patch.c[j * 4 + 1],
                     a2 = patch.c[j * 4 + 2], a3 = patch.c[j * 4 + 3];
        row[j] = ((a3 * u + a2) * u + a1) * u + a0;
        du[j] = (3. * a3 * u + 2. * a2) * u + a1;
    }
    return {.z = ((row[3] * v + row[2]) * v + row[1]) * v + row[0],
            .dz_du = ((du[3] * v + du[2]) * v + du[1]) * v + du[0],
            .dz_dv = (3. * row[3] * v + 2. * row[2]) * v + row[1]};
}

} // namespace bike::terrain
