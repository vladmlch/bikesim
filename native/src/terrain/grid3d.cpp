#include "grid3d.hpp"

#include "asset3d.hpp"
#include "interpolation3d.hpp"
#include "raycast3d.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace bike::terrain {
namespace {

std::expected<Grid3d, std::string> invalid(std::string path, std::string reason) {
    return std::unexpected(std::move(path) + ": " + std::move(reason));
}

bool checked_product(std::uint64_t a, std::uint64_t b, std::uint64_t& result) noexcept {
    if (b != 0 && a > std::numeric_limits<std::uint64_t>::max() / b) return false;
    result = a * b;
    return true;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
bool distinct_nodes(double origin, double spacing, std::uint64_t nodes) noexcept {
    // A positive spacing and a finite, nonempty total extent do not imply
    // representable cells. At a large world offset, adjacent authored nodes
    // can round to the same binary64 coordinate while the final node moves.
    // Check the exact coordinate expression used by sampling and DDA.
    double previous = origin;
    for (std::uint64_t i = 1; i < nodes; ++i) {
        const double current = origin + static_cast<double>(i) * spacing;
        if (!(current > previous)) return false;
        previous = current;
    }
    return true;
}

} // namespace

std::expected<Grid3d, std::string> Grid3d::create(
    Grid3dDescriptor desc, std::vector<double> elevation,
    std::vector<std::uint32_t> materials, std::vector<SurfaceSpec3d> surfaces,
    Grid3dMetadata metadata, RayBudget budget) {
    if (desc.nx < 4) return invalid("terrain3d.header.nx", "must be >= 4");
    if (desc.ny < 4) return invalid("terrain3d.header.ny", "must be >= 4");
    if (desc.dtype > 1) return invalid("terrain3d.header.dtype", "expected 0 or 1");
    if (!std::isfinite(desc.x0_m)) return invalid("terrain3d.header.x0_m", "non-finite");
    if (!std::isfinite(desc.y0_m)) return invalid("terrain3d.header.y0_m", "non-finite");
    if (!std::isfinite(desc.dx_m) || desc.dx_m <= 0.)
        return invalid("terrain3d.header.dx_m", "must be finite and > 0");
    if (!std::isfinite(desc.dy_m) || desc.dy_m <= 0.)
        return invalid("terrain3d.header.dy_m", "must be finite and > 0");
    std::uint64_t nodes = 0, cells = 0;
    if (!checked_product(desc.nx, desc.ny, nodes) ||
        nodes > std::numeric_limits<std::size_t>::max() / sizeof(double) ||
        !checked_product(desc.nx - 1, desc.ny - 1, cells) ||
        cells > std::numeric_limits<std::size_t>::max() / sizeof(std::uint32_t))
        return invalid("terrain3d.header", "dimension/byte count overflow");
    if (nodes != elevation.size()) return invalid("terrain3d.elevation", "node count mismatch");
    if (cells != materials.size()) return invalid("terrain3d.material", "cell count mismatch");
    const double x1 = desc.x0_m + static_cast<double>(desc.nx - 1) * desc.dx_m;
    const double y1 = desc.y0_m + static_cast<double>(desc.ny - 1) * desc.dy_m;
    if (!std::isfinite(x1) || !(x1 > desc.x0_m))
        return invalid("terrain3d.header.x0_m", "unrepresentable x domain");
    if (!std::isfinite(y1) || !(y1 > desc.y0_m))
        return invalid("terrain3d.header.y0_m", "unrepresentable y domain");
    if (!distinct_nodes(desc.x0_m, desc.dx_m, desc.nx))
        return invalid("terrain3d.header.dx_m", "adjacent x nodes have identical world coordinates");
    if (!distinct_nodes(desc.y0_m, desc.dy_m, desc.ny))
        return invalid("terrain3d.header.dy_m", "adjacent y nodes have identical world coordinates");
    if (budget.max_cells == 0) return invalid("terrain3d.budget.max_cells", "must be positive");
    if (budget.stack_capacity == 0 || budget.stack_capacity > 128)
        return invalid("terrain3d.budget.stack_capacity", "must be in [1,128]");
    if (!std::isfinite(budget.root_interval_m) || budget.root_interval_m <= 0.)
        return invalid("terrain3d.budget.root_interval_m", "must be finite and > 0");
    if (!std::isfinite(budget.residual_m) || budget.residual_m <= 0.)
        return invalid("terrain3d.budget.residual_m", "must be finite and > 0");
    for (std::size_t i = 0; i < elevation.size(); ++i) {
        if (!std::isfinite(elevation[i]))
            return invalid("terrain3d.elevation[" + std::to_string(i) + "]", "non-finite");
        if (desc.dtype == 1 &&
            (std::abs(elevation[i]) > static_cast<double>(std::numeric_limits<float>::max()) ||
             static_cast<double>(static_cast<float>(elevation[i])) != elevation[i]))
            return invalid("terrain3d.elevation[" + std::to_string(i) + "]",
                           "dtype=f32 requires exactly representable finite values");
    }
    if (surfaces.empty()) return invalid("terrain3d.surfaces", "must not be empty");
    if (surfaces.size() >= std::numeric_limits<std::uint32_t>::max())
        return invalid("terrain3d.surfaces", "too many surfaces");
    // Sorted indices keep arbitrary u32 IDs bounded by surface count. A dense
    // max(id)+1 table would allow a one-record asset to demand sixteen GiB.
    std::vector<std::uint32_t> indices(surfaces.size());
    std::ranges::iota(indices, std::uint32_t{0});
    std::ranges::sort(indices, [&surfaces](auto a, auto b) {
        return surfaces[a].id < surfaces[b].id;
    });
    for (std::size_t i = 0; i < surfaces.size(); ++i) {
        const auto& s = surfaces[i];
        const std::string path = "terrain3d.surfaces[" + std::to_string(i) + "]";
        if (i != 0 && surfaces[indices[i - 1]].id == surfaces[indices[i]].id)
            return invalid("terrain3d.surfaces[" + std::to_string(indices[i]) + "].id", "duplicate ID");
        if (s.name.find('\0') != std::string::npos)
            return invalid(path + ".name", "embedded NUL");
        if (!std::isfinite(s.mu_peak) || s.mu_peak < 0.)
            return invalid(path + ".mu_peak", "must be finite and >= 0");
        if (!std::isfinite(s.mu_slide) || s.mu_slide < 0.)
            return invalid(path + ".mu_slide", "must be finite and >= 0");
        if (!std::isfinite(s.slip_stiffness_per_load) || s.slip_stiffness_per_load < 0.)
            return invalid(path + ".slip_stiffness_per_load", "must be finite and >= 0");
        if (!std::isfinite(s.stribeck_speed_mps) || s.stribeck_speed_mps <= 0.)
            return invalid(path + ".stribeck_speed_mps", "must be finite and > 0");
    }
    for (std::size_t i = 0; i < materials.size(); ++i) {
        const auto it = std::ranges::lower_bound(indices, materials[i],
            [&surfaces](auto index, auto id) { return surfaces[index].id < id; });
        if (it == indices.end() || surfaces[*it].id != materials[i])
            return invalid("terrain3d.material[" + std::to_string(i) + "]", "unknown surface ID");
    }
    if (metadata.units != "m") return invalid("terrain3d.metadata.units", "expected m");
    if (!std::isfinite(metadata.height_datum_m))
        return invalid("terrain3d.metadata.height_datum_m", "non-finite");
    if (!metadata.seed_decimal.empty() &&
        !std::ranges::all_of(metadata.seed_decimal,
                             [](char c) { return c >= '0' && c <= '9'; }))
        return invalid("terrain3d.metadata.seed_decimal", "expected unsigned decimal digits");
    Grid3d result;
    result.desc_ = desc;
    result.elevation_ = std::move(elevation);
    result.materials_ = std::move(materials);
    result.surfaces_ = std::move(surfaces);
    result.surface_index_by_id_ = std::move(indices);
    result.meta_ = std::move(metadata);
    result.budget_ = budget;
    try {
        // write_terrain3d reads only the validated fields, never sha256_hex().
        result.sha256_hex_ = bike::io::sha256(write_terrain3d(result));
    } catch (const std::invalid_argument& error) {
        return invalid("terrain3d.metadata_or_surfaces", error.what());
    }
    return result;
}

std::uint64_t Grid3d::nx() const noexcept { return desc_.nx; }
std::uint64_t Grid3d::ny() const noexcept { return desc_.ny; }
const Grid3dDescriptor& Grid3d::descriptor() const noexcept { return desc_; }
std::span<const double> Grid3d::elevation() const noexcept { return elevation_; }
std::span<const std::uint32_t> Grid3d::materials() const noexcept { return materials_; }
std::span<const SurfaceSpec3d> Grid3d::surfaces() const noexcept { return surfaces_; }
const Grid3dMetadata& Grid3d::metadata() const noexcept { return meta_; }
const RayBudget& Grid3d::budget() const noexcept { return budget_; }
double Grid3d::x_max_m() const noexcept {
    return desc_.x0_m + static_cast<double>(desc_.nx - 1) * desc_.dx_m;
}
double Grid3d::y_max_m() const noexcept {
    return desc_.y0_m + static_cast<double>(desc_.ny - 1) * desc_.dy_m;
}
const std::string& Grid3d::sha256_hex() const noexcept { return sha256_hex_; }

GridSample3d Grid3d::sample(double x, double y) const noexcept {
    if (!std::isfinite(x) || !std::isfinite(y)) return {};
    if (x < desc_.x0_m || x > x_max_m() || y < desc_.y0_m || y > y_max_m())
        return {.status = QueryStatus3d::outside};
    // Boundary checks precede coordinate rounding: nextafter outside is outside.
    const double sx = (x - desc_.x0_m) / desc_.dx_m;
    const double sy = (y - desc_.y0_m) / desc_.dy_m;
    const auto ix = std::min(static_cast<std::uint64_t>(std::floor(sx)), nx() - 2);
    const auto iy = std::min(static_cast<std::uint64_t>(std::floor(sy)), ny() - 2);
    const double u = x == x_max_m() ? 1. : sx - static_cast<double>(ix);
    const double v = y == y_max_m() ? 1. : sy - static_cast<double>(iy);
    const auto value = evaluate(bicubic_patch(*this, ix, iy), u, v);
    const double gx = value.dz_du / desc_.dx_m, gy = value.dz_dv / desc_.dy_m;
    const double length = std::hypot(gx, gy, 1.);
    if (!std::isfinite(value.z) || !std::isfinite(length)) return {};
    return {.status = QueryStatus3d::ok, .height_m = value.z,
            .normal_world = {-gx / length, -gy / length, 1. / length},
            .surface_id = materials_[static_cast<std::size_t>(iy * (nx() - 1) + ix)]};
}

RayHit3d Grid3d::ray_cast(std::array<double, 3> origin,
                         std::array<double, 3> direction, double distance) const noexcept {
    return ray_cast_grid(*this, origin, direction, distance);
}

} // namespace bike::terrain
