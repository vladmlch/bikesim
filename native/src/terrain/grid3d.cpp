#include "terrain/grid3d.hpp"

#include "io/hash.hpp"
#include "terrain/interpolation3d.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bike::terrain {
namespace {

void hash_u64(bike::io::Sha256& h, std::uint64_t v) noexcept {
    std::array<std::byte, 8> le{};
    for (std::size_t i = 0; i < 8; ++i) {
        le[i] = static_cast<std::byte>((v >> (8U * i)) & 0xffU);
    }
    h.update(le);
}

void hash_f64(bike::io::Sha256& h, double v) noexcept {
    hash_u64(h, std::bit_cast<std::uint64_t>(v == 0. ? 0. : v));
}

std::string hex_digest(const std::array<std::uint8_t, 32>& digest) {
    static constexpr std::array<char, 16> kHex{
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
    };
    std::string out(64, '0');
    for (std::size_t i = 0; i < 32; ++i) {
        const auto byte = static_cast<unsigned>(digest[i]);
        out[2 * i] = kHex[(byte >> 4U) & 0x0fU];
        out[2 * i + 1] = kHex[byte & 0x0fU];
    }
    return out;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (idx, count) internal stencil index helper
[[nodiscard]] std::size_t clamped_index(std::ptrdiff_t idx, std::size_t count) noexcept {
    if (idx <= 0) {
        return 0;
    }
    const auto u = static_cast<std::size_t>(idx);
    return u >= count ? count - 1 : u;
}

void extract_stencil(const Grid3d& grid, std::size_t ix, std::size_t iy,
                     std::array<std::array<double, 4>, 4>& out) noexcept {
    const auto ix_base = static_cast<std::ptrdiff_t>(ix) - 1;
    const auto iy_base = static_cast<std::ptrdiff_t>(iy) - 1;
    const auto nx = grid.spec().nx;
    const auto ny = grid.spec().ny;
    for (std::size_t rx = 0; rx < 4; ++rx) {
        const std::size_t sx = clamped_index(ix_base + static_cast<std::ptrdiff_t>(rx), nx);
        for (std::size_t ry = 0; ry < 4; ++ry) {
            const std::size_t sy = clamped_index(iy_base + static_cast<std::ptrdiff_t>(ry), ny);
            out[rx][ry] = grid.node_z(sx, sy);
        }
    }
}

[[nodiscard]] Vec3d normal_from_slopes(double dz_dx, double dz_dy) noexcept {
    const double inv_len = 1. / std::sqrt(1. + dz_dx * dz_dx + dz_dy * dz_dy);
    return {-dz_dx * inv_len, -dz_dy * inv_len, inv_len};
}

} // namespace

Grid3d::Grid3d(Grid3dSpec spec, std::vector<double> elevations_xy,
               std::vector<::rider::SurfaceLabel> cell_surfaces_xy)
    : spec_(spec),
      elevations_xy_(std::move(elevations_xy)),
      cell_surfaces_xy_(std::move(cell_surfaces_xy)) {
    if (spec_.nx < 4 || spec_.ny < 4 || spec_.nx > 500000 || spec_.ny > 10000) {
        throw std::invalid_argument("Grid3d: nx and ny must be >= 4 and within bounded limits");
    }
    if (!std::isfinite(spec_.x0_m) || !std::isfinite(spec_.y0_m) ||
        !std::isfinite(spec_.dx_m) || !std::isfinite(spec_.dy_m) ||
        !(spec_.dx_m > 0.) || !(spec_.dy_m > 0.)) {
        throw std::invalid_argument("Grid3d: finite origin and positive grid spacings required");
    }
    // Verify strictly increasing binary64 node coordinates on both axes.
    for (std::size_t ix = 1; ix < spec_.nx; ++ix) {
        const double prev = spec_.x0_m + static_cast<double>(ix - 1) * spec_.dx_m;
        const double curr = spec_.x0_m + static_cast<double>(ix) * spec_.dx_m;
        if (!(curr > prev) || !std::isfinite(curr)) {
            throw std::invalid_argument("Grid3d: collapsed or nonfinite X coordinate progression");
        }
    }
    for (std::size_t iy = 1; iy < spec_.ny; ++iy) {
        const double prev = spec_.y0_m + static_cast<double>(iy - 1) * spec_.dy_m;
        const double curr = spec_.y0_m + static_cast<double>(iy) * spec_.dy_m;
        if (!(curr > prev) || !std::isfinite(curr)) {
            throw std::invalid_argument("Grid3d: collapsed or nonfinite Y coordinate progression");
        }
    }

    const std::size_t expected_nodes = spec_.nx * spec_.ny;
    const std::size_t expected_cells = (spec_.nx - 1) * (spec_.ny - 1);
    if (elevations_xy_.size() != expected_nodes || cell_surfaces_xy_.size() != expected_cells) {
        throw std::invalid_argument("Grid3d: elevation or surface array size mismatch");
    }

    double z_min = elevations_xy_.front();
    double z_max = elevations_xy_.front();
    for (const double z : elevations_xy_) {
        if (!std::isfinite(z)) {
            throw std::invalid_argument("Grid3d: nonfinite elevation value");
        }
        z_min = std::min(z_min, z);
        z_max = std::max(z_max, z);
    }

    // Because Steffen interpolation is strictly monotone within cell corner bounds,
    // the exact cell Z-AABB equals min/max of its 4 corners!
    cell_z_ranges_.resize(expected_cells);
    for (std::size_t ix = 0; ix + 1 < spec_.nx; ++ix) {
        for (std::size_t iy = 0; iy + 1 < spec_.ny; ++iy) {
            const double z00 = node_z(ix, iy);
            const double z10 = node_z(ix + 1, iy);
            const double z01 = node_z(ix, iy + 1);
            const double z11 = node_z(ix + 1, iy + 1);
            cell_z_ranges_[ix * (spec_.ny - 1) + iy] = CellZRange{
                .z_min = std::min({z00, z10, z01, z11}),
                .z_max = std::max({z00, z10, z01, z11}),
            };
        }
    }

    bounds_ = TerrainBounds3d{
        .x_min_m = spec_.x0_m,
        .x_max_m = spec_.x0_m + static_cast<double>(spec_.nx - 1) * spec_.dx_m,
        .y_min_m = spec_.y0_m,
        .y_max_m = spec_.y0_m + static_cast<double>(spec_.ny - 1) * spec_.dy_m,
        .z_min_m = z_min,
        .z_max_m = z_max,
    };

    // Streaming SHA-256 without copying a second multi-MB buffer.
    bike::io::Sha256 hasher;
    hash_u64(hasher, 2U);
    hash_u64(hasher, spec_.nx);
    hash_u64(hasher, spec_.ny);
    hash_f64(hasher, spec_.x0_m);
    hash_f64(hasher, spec_.y0_m);
    hash_f64(hasher, spec_.dx_m);
    hash_f64(hasher, spec_.dy_m);
    for (const double z : elevations_xy_) {
        hash_f64(hasher, z);
    }
    for (const ::rider::SurfaceLabel s : cell_surfaces_xy_) {
        const std::array<std::byte, 1> b{static_cast<std::byte>(s)};
        hasher.update(b);
    }
    sha256_hex_ = hex_digest(hasher.finish());
}

TerrainSample3d Grid3d::sample(double x_m, double y_m) const noexcept {
    if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
        return {.status = QueryStatus3d::invalid_input};
    }
    if (x_m < bounds_.x_min_m || x_m > bounds_.x_max_m ||
        y_m < bounds_.y_min_m || y_m > bounds_.y_max_m) {
        return {.status = QueryStatus3d::outside};
    }

    const double fx = (x_m - spec_.x0_m) / spec_.dx_m;
    const double fy = (y_m - spec_.y0_m) / spec_.dy_m;
    const std::size_t ix = std::min(static_cast<std::size_t>(std::max(0., std::floor(fx))), spec_.nx - 2);
    const std::size_t iy = std::min(static_cast<std::size_t>(std::max(0., std::floor(fy))), spec_.ny - 2);
    const double u = std::clamp(fx - static_cast<double>(ix), 0., 1.);
    const double v = std::clamp(fy - static_cast<double>(iy), 0., 1.);

    std::array<std::array<double, 4>, 4> stencil{};
    extract_stencil(*this, ix, iy, stencil);
    const BicubicEval3d ev = evaluate_steffen_bicubic(stencil, spec_.dx_m, spec_.dy_m, u, v);
    return TerrainSample3d{
        .status = QueryStatus3d::ok,
        .height_m = ev.height_m,
        .normal_world = normal_from_slopes(ev.dz_dx, ev.dz_dy),
        .surface = cell_surfaces_xy_[ix * (spec_.ny - 1) + iy],
    };
}

TorusClosestPoint3d Grid3d::closest_point(const Vec3d& center_world_m,
                                          double search_radius_m) const noexcept {
    if (!std::isfinite(center_world_m[0]) || !std::isfinite(center_world_m[1]) ||
        !std::isfinite(center_world_m[2]) || !std::isfinite(search_radius_m) ||
        !(search_radius_m > 0.)) {
        return {.status = QueryStatus3d::invalid_input};
    }
    const double cx = center_world_m[0];
    const double cy = center_world_m[1];
    const double cz = center_world_m[2];
    if (cx < bounds_.x_min_m || cx > bounds_.x_max_m ||
        cy < bounds_.y_min_m || cy > bounds_.y_max_m) {
        return {.status = QueryStatus3d::outside};
    }

    // Seed with vertical sample directly below/above (cx, cy).
    const TerrainSample3d vertical = sample(cx, cy);
    if (vertical.status != QueryStatus3d::ok) {
        return {.status = vertical.status};
    }
    double best_d2 = (cz - vertical.height_m) * (cz - vertical.height_m);
    double best_x = cx;
    double best_y = cy;
    double best_z = vertical.height_m;
    Vec3d best_surf_normal = vertical.normal_world;
    ::rider::SurfaceLabel best_surface = vertical.surface;

    // Scan candidate cells inside [cx - R, cx + R] x [cy - R, cy + R] with exact Z-AABB pruning.
    const double x_lo = std::max(bounds_.x_min_m, cx - search_radius_m);
    const double x_hi = std::min(bounds_.x_max_m, cx + search_radius_m);
    const double y_lo = std::max(bounds_.y_min_m, cy - search_radius_m);
    const double y_hi = std::min(bounds_.y_max_m, cy + search_radius_m);

    const std::size_t ix0 = std::min(
        static_cast<std::size_t>(std::max(0., std::floor((x_lo - spec_.x0_m) / spec_.dx_m))),
        spec_.nx - 2);
    const std::size_t ix1 = std::min(
        static_cast<std::size_t>(std::max(0., std::floor((x_hi - spec_.x0_m) / spec_.dx_m))),
        spec_.nx - 2);
    const std::size_t iy0 = std::min(
        static_cast<std::size_t>(std::max(0., std::floor((y_lo - spec_.y0_m) / spec_.dy_m))),
        spec_.ny - 2);
    const std::size_t iy1 = std::min(
        static_cast<std::size_t>(std::max(0., std::floor((y_hi - spec_.y0_m) / spec_.dy_m))),
        spec_.ny - 2);

    const double inv_dx = 1. / spec_.dx_m;
    const double inv_dy = 1. / spec_.dy_m;

    for (std::size_t ix = ix0; ix <= ix1; ++ix) {
        const double cell_x0 = spec_.x0_m + static_cast<double>(ix) * spec_.dx_m;
        const double cell_x1 = cell_x0 + spec_.dx_m;
        const double dx_box = cx < cell_x0 ? (cell_x0 - cx) : (cx > cell_x1 ? (cx - cell_x1) : 0.);
        const double u_clamp = std::clamp((cx - cell_x0) * inv_dx, 0., 1.);

        for (std::size_t iy = iy0; iy <= iy1; ++iy) {
            const double cell_y0 = spec_.y0_m + static_cast<double>(iy) * spec_.dy_m;
            const double cell_y1 = cell_y0 + spec_.dy_m;
            const double dy_box = cy < cell_y0 ? (cell_y0 - cy) : (cy > cell_y1 ? (cy - cell_y1) : 0.);
            const std::size_t cell_idx = ix * (spec_.ny - 1) + iy;
            const CellZRange zr = cell_z_ranges_[cell_idx];
            const double dz_box = cz < zr.z_min ? (zr.z_min - cz) : (cz > zr.z_max ? (cz - zr.z_max) : 0.);
            if (dx_box * dx_box + dy_box * dy_box + dz_box * dz_box >= best_d2) {
                continue;
            }

            const double v_clamp = std::clamp((cy - cell_y0) * inv_dy, 0., 1.);
            const ::rider::SurfaceLabel cell_surf = cell_surfaces_xy_[cell_idx];

            // Fast path for flat cells: closest point is strictly (u_clamp, v_clamp).
            if (zr.z_max == zr.z_min) {
                const double d2 = dx_box * dx_box + dy_box * dy_box + (cz - zr.z_min) * (cz - zr.z_min);
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_x = cell_x0 + u_clamp * spec_.dx_m;
                    best_y = cell_y0 + v_clamp * spec_.dy_m;
                    best_z = zr.z_min;
                    best_surf_normal = {0., 0., 1.};
                    best_surface = cell_surf;
                }
                continue;
            }

            std::array<std::array<double, 4>, 4> stencil{};
            extract_stencil(*this, ix, iy, stencil);

            // Multi-seed initialization avoids getting trapped at zero-slope corners of
            // steep S-curve Steffen cells (e.g. 100 mm SquareEdge).
            std::array<std::pair<double, double>, 6> seeds{{
                {u_clamp, v_clamp},
                {0.5, 0.5},
                {0.25, v_clamp},
                {0.75, v_clamp},
                {u_clamp, 0.25},
                {u_clamp, 0.75},
            }};
            const std::size_t n_seeds = (zr.z_max - zr.z_min > 1e-4) ? 6U : 2U;

            double seed_u = u_clamp;
            double seed_v = v_clamp;
            double seed_d2 = std::numeric_limits<double>::infinity();
            for (std::size_t s = 0; s < n_seeds; ++s) {
                const double su = seeds[s].first;
                const double sv = seeds[s].second;
                const BicubicEval3d ev = evaluate_steffen_bicubic(stencil, spec_.dx_m, spec_.dy_m, su, sv);
                const double px = cell_x0 + su * spec_.dx_m;
                const double py = cell_y0 + sv * spec_.dy_m;
                const double rz = ev.height_m - cz;
                const double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy) + rz * rz;
                if (d2 < seed_d2) {
                    seed_d2 = d2;
                    seed_u = su;
                    seed_v = sv;
                }
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_x = px;
                    best_y = py;
                    best_z = ev.height_m;
                    best_surf_normal = normal_from_slopes(ev.dz_dx, ev.dz_dy);
                    best_surface = cell_surf;
                }
            }

            // Refine closest point on this cell via projected Gauss-Newton with damping.
            double u = seed_u;
            double v = seed_v;
            for (int iter = 0; iter < 6; ++iter) {
                const BicubicEval3d ev = evaluate_steffen_bicubic(stencil, spec_.dx_m, spec_.dy_m, u, v);
                const double px = cell_x0 + u * spec_.dx_m;
                const double py = cell_y0 + v * spec_.dy_m;
                const double rz = ev.height_m - cz;
                const double d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy) + rz * rz;
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_x = px;
                    best_y = py;
                    best_z = ev.height_m;
                    best_surf_normal = normal_from_slopes(ev.dz_dx, ev.dz_dy);
                    best_surface = cell_surf;
                }
                // Gradient of 0.5 * ((x-cx)^2 + (y-cy)^2 + (h(x,y)-cz)^2).
                const double gx = (px - cx) + rz * ev.dz_dx;
                const double gy = (py - cy) + rz * ev.dz_dy;
                const double hxx = 1. + ev.dz_dx * ev.dz_dx;
                const double hyy = 1. + ev.dz_dy * ev.dz_dy;
                const double hxy = ev.dz_dx * ev.dz_dy;
                const double det = hxx * hyy - hxy * hxy;
                if (!(det > 1e-12)) {
                    break;
                }
                const double step_x = (hyy * gx - hxy * gy) / det;
                const double step_y = (hxx * gy - hxy * gx) / det;
                const double next_u = std::clamp(u - step_x * inv_dx, 0., 1.);
                const double next_v = std::clamp(v - step_y * inv_dy, 0., 1.);
                if (std::abs(next_u - u) <= 1e-13 && std::abs(next_v - v) <= 1e-13) {
                    break;
                }
                u = next_u;
                v = next_v;
            }
            const BicubicEval3d ev_end = evaluate_steffen_bicubic(stencil, spec_.dx_m, spec_.dy_m, u, v);
            const double px_end = cell_x0 + u * spec_.dx_m;
            const double py_end = cell_y0 + v * spec_.dy_m;
            const double d2_end = (px_end - cx) * (px_end - cx) +
                                  (py_end - cy) * (py_end - cy) +
                                  (ev_end.height_m - cz) * (ev_end.height_m - cz);
            if (d2_end < best_d2) {
                best_d2 = d2_end;
                best_x = px_end;
                best_y = py_end;
                best_z = ev_end.height_m;
                best_surf_normal = normal_from_slopes(ev_end.dz_dx, ev_end.dz_dy);
                best_surface = cell_surf;
            }
        }
    }

    const Vec3d q_surf{best_x, best_y, best_z};
    const Vec3d diff{cx - q_surf[0], cy - q_surf[1], cz - q_surf[2]};
    const double dist = std::sqrt(best_d2);
    const double sign = cz >= vertical.height_m ? 1. : -1.;
    Vec3d normal = best_surf_normal;
    if (dist > 1e-12) {
        normal = {sign * diff[0] / dist, sign * diff[1] / dist, sign * diff[2] / dist};
        if (normal[2] < 0.) {
            normal = best_surf_normal;
        }
    }
    return TorusClosestPoint3d{
        .status = QueryStatus3d::ok,
        .signed_distance_m = sign * dist,
        .surface_point_world_m = q_surf,
        .normal_world = normal,
        .surface = best_surface,
    };
}

} // namespace bike::terrain
