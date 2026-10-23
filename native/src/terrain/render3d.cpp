#include "terrain/render3d.hpp"
#include "terrain/interpolation3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::terrain {
namespace {
std::array<double, 4> integral_powers(double a, double b) noexcept {
    std::array<double, 4> out{};
    double pa = a, pb = b;
    for (std::size_t i = 0; i < 4; ++i) {
        out[i] = (pb - pa) / static_cast<double>(i + 1);
        pa *= a; pb *= b;
    }
    return out;
}
double area_mean(const Grid3d& grid, double xa, double xb, double ya, double yb) noexcept {
    const auto& d = grid.descriptor();
    const double ua = (xa - d.x0_m) / d.dx_m, ub = (xb - d.x0_m) / d.dx_m;
    const double va = (ya - d.y0_m) / d.dy_m, vb = (yb - d.y0_m) / d.dy_m;
    const auto ix0 = std::min(static_cast<std::uint64_t>(std::max(0., std::floor(ua))), grid.nx() - 2);
    const auto iy0 = std::min(static_cast<std::uint64_t>(std::max(0., std::floor(va))), grid.ny() - 2);
    const auto ix1 = std::min(static_cast<std::uint64_t>(std::max(0., std::floor(ub))), grid.nx() - 2);
    const auto iy1 = std::min(static_cast<std::uint64_t>(std::max(0., std::floor(vb))), grid.ny() - 2);
    double total = 0.;
    for (auto iy = iy0; iy <= iy1; ++iy) for (auto ix = ix0; ix <= ix1; ++ix) {
        const auto u = integral_powers(std::max(0., ua - static_cast<double>(ix)), std::min(1., ub - static_cast<double>(ix)));
        const auto v = integral_powers(std::max(0., va - static_cast<double>(iy)), std::min(1., vb - static_cast<double>(iy)));
        const auto patch = bicubic_patch(grid, ix, iy);
        double cell = 0.;
        for (std::size_t j = 0; j < 4; ++j) for (std::size_t i = 0; i < 4; ++i) cell += patch.c[j * 4 + i] * u[i] * v[j];
        total += cell;
    }
    return total / ((ub - ua) * (vb - va));
}
}

RenderGrid3d make_render_grid(const Terrain3dSource& source, RenderResolution resolution) {
    if (resolution.nrow < 2 || resolution.ncol < 2)
        throw std::invalid_argument("terrain3d.render.resolution: require nrow,ncol >= 2");
    const auto rows = static_cast<std::size_t>(resolution.nrow), columns = static_cast<std::size_t>(resolution.ncol);
    if (rows > std::numeric_limits<std::size_t>::max() / columns || rows * columns > (512ull << 20) / sizeof(float))
        throw std::invalid_argument("terrain3d.render.resolution: raster exceeds resource limit");
    RenderGrid3d out;
    out.nrow = resolution.nrow; out.ncol = resolution.ncol; out.resolution = resolution;
    out.x0_m = source.x_min_m(); out.x1_m = source.x_max_m();
    out.y0_m = source.grid() ? source.grid()->descriptor().y0_m : -source.half_width_m();
    out.y1_m = source.grid() ? source.grid()->y_max_m() : source.half_width_m();
    out.physical_asset_sha256_hex = source.fingerprint_hex();
    out.normalized_height.resize(rows * columns);
    std::vector<double> heights(rows * columns);
    if (const auto* grid = source.grid()) {
        const auto physical = grid->elevation();
        if (std::ranges::all_of(physical, [&](double value) { return value == physical.front(); })) {
            out.z_min_m = physical.front(); out.z_max_m = physical.front();
            std::ranges::fill(out.normalized_height, 0.f);
            return out;
        }
        const double dx = (out.x1_m - out.x0_m) / static_cast<double>(columns - 1);
        const double dy = (out.y1_m - out.y0_m) / static_cast<double>(rows - 1);
        for (std::size_t row = 0; row < rows; ++row) for (std::size_t column = 0; column < columns; ++column) {
            const double x = out.x0_m + static_cast<double>(column) * dx, y = out.y0_m + static_cast<double>(row) * dy;
            heights[row * columns + column] = area_mean(*grid,
                column == 0 ? out.x0_m : x - .5 * dx, column + 1 == columns ? out.x1_m : x + .5 * dx,
                row == 0 ? out.y0_m : y - .5 * dy, row + 1 == rows ? out.y1_m : y + .5 * dy);
        }
    } else {
        for (std::size_t column = 0; column < columns; ++column) {
            const double x = column + 1 == columns ? out.x1_m : out.x0_m +
                static_cast<double>(column) * (out.x1_m - out.x0_m) / static_cast<double>(columns - 1);
            const auto sampled = source.query_view().sample(x, 0.);
            if (sampled.status != QueryStatus3d::ok) throw std::invalid_argument("terrain3d.render: extrusion sample outside source");
            for (std::size_t row = 0; row < rows; ++row) heights[row * columns + column] = sampled.height_m;
        }
    }
    const auto [minimum, maximum] = std::minmax_element(heights.begin(), heights.end());
    out.z_min_m = *minimum; out.z_max_m = *maximum;
    const double span = out.z_max_m - out.z_min_m;
    for (std::size_t i = 0; i < heights.size(); ++i) {
        if (!std::isfinite(heights[i]) || !std::isfinite(span)) throw std::invalid_argument("terrain3d.render: non-finite downsample");
        out.normalized_height[i] = span == 0. ? 0.f : static_cast<float>((heights[i] - out.z_min_m) / span);
    }
    return out;
}
} // namespace bike::terrain
