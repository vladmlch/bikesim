#include "terrain/generator3d.hpp"
#include "terrain/detail.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"
#include "platform/provenance.hpp"
#include "random/numpy_compat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>
#include <string_view>

namespace bike::terrain {
namespace {
constexpr std::uint64_t kMaxGeneratedNodes = 100'000'000;

std::size_t nodes(double extent, double spacing, std::string_view path) {
    const double count = std::floor(extent / spacing) + 1.;
    if (!std::isfinite(count) || count < 4. || count > static_cast<double>(kMaxGeneratedNodes))
        throw std::invalid_argument(std::string(path) + ": node count outside [4,100000000]");
    return static_cast<std::size_t>(count);
}
std::size_t product(std::size_t a, std::size_t b, std::string_view path) {
    if (a != 0 && b > std::numeric_limits<std::size_t>::max() / a)
        throw std::invalid_argument(std::string(path) + ": size overflow");
    return a * b;
}
std::vector<double> gaussian(double ell, double spacing) {
    const double radius_value = std::ceil(4. * ell / spacing);
    if (!std::isfinite(radius_value) || radius_value > 1'000'000.)
        throw std::invalid_argument("terrain3d.roughness.ell_m: convolution halo exceeds resource bound");
    const auto radius = static_cast<std::int64_t>(radius_value);
    std::vector<double> weights(static_cast<std::size_t>(2 * radius + 1));
    double sum = 0.;
    for (std::int64_t k = -radius; k <= radius; ++k) {
        const double ratio = static_cast<double>(k) * spacing / ell;
        const double weight = std::exp(-.5 * ratio * ratio);
        weights[static_cast<std::size_t>(k + radius)] = weight;
        sum += weight;
    }
    for (auto& weight : weights) weight /= sum;
    return weights;
}
std::size_t mirror(std::int64_t index, std::size_t count) noexcept {
    const auto last = static_cast<std::int64_t>(count - 1);
    const auto period = 2 * last;
    auto wrapped = index % period;
    if (wrapped < 0) wrapped += period;
    return static_cast<std::size_t>(wrapped <= last ? wrapped : period - wrapped);
}
double y_node(std::size_t index, std::size_t count, double half_width, double dy) noexcept {
    // A symmetric declared corridor gets exact opposite arithmetic at paired
    // nodes. This keeps authored mirror fixtures bitwise symmetric.
    if (static_cast<double>(count - 1) * dy == 2. * half_width && index > (count - 1) / 2)
        return -(-half_width + static_cast<double>(count - 1 - index) * dy);
    return -half_width + static_cast<double>(index) * dy;
}
double window(double x, double length, const Track3dSpec& spec) noexcept {
    if (x <= spec.lead_in_m || x >= length - spec.lead_out_m) return 0.;
    const double transition = std::min(1., .5 * (length - spec.lead_in_m - spec.lead_out_m));
    return quintic_smoothstep((x - spec.lead_in_m) / transition) *
           quintic_smoothstep((length - spec.lead_out_m - x) / transition);
}
double feature_height(const Feature3dSpec& feature, double x, double y) noexcept {
    const double c = std::cos(feature.yaw_rad), s = std::sin(feature.yaw_rad);
    const double dx = x - feature.x_m, dy = y - feature.y_m;
    const double u = (c * dx + s * dy) / feature.radius_x_m;
    const double v = (-s * dx + c * dy) / feature.radius_y_m;
    const double radius2 = u * u + v * v;
    if (radius2 >= 1.) return 0.;
    const double edge = 1. - radius2;
    const double magnitude = feature.amplitude_m * std::exp(-radius2) * edge * edge;
    return feature.kind == "rut" ? -magnitude : magnitude;
}
}

double quintic_smoothstep(double u) noexcept {
    if (u <= 0.) return 0.;
    if (u >= 1.) return 1.;
    return u * u * u * (u * (6. * u - 15.) + 10.);
}

Grid3d generate_grid3d(const Track3dSpec& spec) {
    // The writer validates the complete typed domain, including direct C++
    // callers. It is also the canonical resolved specification in the asset.
    auto resolved_spec = write_track3d_spec(spec);
    const auto track = track_from_dict(spec.base_track_spec);
    const std::size_t nx = nodes(track.length_m, spec.dx_m, "terrain3d.dx_m");
    const std::size_t ny = nodes(2. * spec.half_width_m, spec.dy_m, "terrain3d.dy_m");
    const std::size_t count = product(nx, ny, "terrain3d.elevation");
    if (count > kMaxGeneratedNodes) throw std::invalid_argument("terrain3d.elevation: generator node limit exceeded");
    (void)product(count, sizeof(double), "terrain3d.elevation.bytes");
    const auto cell_count = product(nx - 1, ny - 1, "terrain3d.materials");
    (void)product(cell_count, sizeof(std::uint32_t), "terrain3d.materials.bytes");

    std::vector<double> xs(nx);
    for (std::size_t ix = 0; ix < nx; ++ix) xs[ix] = static_cast<double>(ix) * spec.dx_m;
    // This is the existing longitudinal producer, not a reimplementation.
    const auto base_height = build_profile(track, xs);
    std::vector<double> elevation(count, 0.);
    if (spec.roughness.sigma_m > 0.) {
        const auto wx = gaussian(spec.roughness.ell_m, spec.dx_m);
        const auto wy = gaussian(spec.roughness.ell_m, spec.dy_m);
        const auto rx = static_cast<std::int64_t>((wx.size() - 1) / 2);
        const auto ry = static_cast<std::int64_t>((wy.size() - 1) / 2);
        // A separate stable stream namespace, encoded as little-endian ASCII
        // words in the existing SeedSequence spawn-key domain.
        constexpr auto stream = [] {
            constexpr std::string_view label = "terrain3d/roughness";
            std::array<std::uint32_t, 5> key{};
            for (std::size_t i = 0; i < label.size(); ++i)
                key[i / 4] |= static_cast<std::uint32_t>(static_cast<unsigned char>(label[i])) << (8 * (i % 4));
            return key;
        }();
        bike::random::NumpyRng rng(spec.seed_decimal, stream);
        std::vector<double> scratch(count);
        rng.normal01(std::span<double>(elevation));
        for (std::size_t iy = 0; iy < ny; ++iy) {
            const auto row = iy * nx;
            for (std::size_t ix = 0; ix < nx; ++ix) {
                double sum = 0.;
                for (std::int64_t k = -rx; k <= rx; ++k)
                    sum += wx[static_cast<std::size_t>(k + rx)] * elevation[row + mirror(static_cast<std::int64_t>(ix) + k, nx)];
                scratch[row + ix] = sum;
            }
        }
        // Y strips read an explicit mirrored halo from the X-filtered buffer;
        // no third full-size array or per-strip allocation is permitted.
        constexpr std::size_t strip_rows = 16;
        for (std::size_t strip = 0; strip < ny; strip += strip_rows) {
            const auto stop = std::min(ny, strip + strip_rows);
            for (std::size_t iy = strip; iy < stop; ++iy) for (std::size_t ix = 0; ix < nx; ++ix) {
                double sum = 0.;
                for (std::int64_t k = -ry; k <= ry; ++k)
                    sum += wy[static_cast<std::size_t>(k + ry)] * scratch[mirror(static_cast<std::int64_t>(iy) + k, ny) * nx + ix];
                elevation[iy * nx + ix] = spec.roughness.sigma_m * sum;
            }
        }
    } // Release scratch before allocating materials and the canonical asset.
    for (std::size_t iy = 0; iy < ny; ++iy) {
        const double y = y_node(iy, ny, spec.half_width_m, spec.dy_m);
        for (std::size_t ix = 0; ix < nx; ++ix) {
            const double x = xs[ix];
            double bank_deg = 0.;
            for (const auto& bank : spec.banks) {
                const double fraction = quintic_smoothstep((x - bank.start_x_m) / bank.ramp_m);
                bank_deg += (bank.angle_deg - bank_deg) * fraction;
            }
            double lateral = elevation[iy * nx + ix] + y * std::tan(bank_deg * std::numbers::pi / 180.);
            for (const auto& feature : spec.features) lateral += feature_height(feature, x, y);
            const double fade = window(x, track.length_m, spec);
            // Preserve the exact base bits in the flat lateral lead zones.
            elevation[iy * nx + ix] = fade == 0. ? base_height[ix] : base_height[ix] + fade * lateral;
            if (!std::isfinite(elevation[iy * nx + ix]))
                throw std::invalid_argument("terrain3d.elevation[" + std::to_string(iy * nx + ix) + "]: non-finite generated height");
        }
    }
    const double slope_limit = std::tan(std::numbers::pi / 3.);
    for (std::size_t iy = 0; iy < ny; ++iy) for (std::size_t ix = 0; ix < nx; ++ix) {
        const auto left = ix == nx - 1 ? ix - 1 : ix;
        const auto down = iy == ny - 1 ? iy - 1 : iy;
        const double sx = (elevation[iy * nx + left + 1] - elevation[iy * nx + left]) / spec.dx_m;
        const double sy = (elevation[(down + 1) * nx + ix] - elevation[down * nx + ix]) / spec.dy_m;
        if (!std::isfinite(sx) || !std::isfinite(sy) || std::hypot(sx, sy) > slope_limit) {
            std::string path = "terrain3d.base_track";
            const double y = y_node(iy, ny, spec.half_width_m, spec.dy_m);
            for (std::size_t i = 0; i < spec.features.size(); ++i)
                if (feature_height(spec.features[i], xs[ix], y) != 0.) path = "terrain3d.features[" + std::to_string(i) + "]";
            throw std::invalid_argument(path + ": resulting slope exceeds limit at node " + std::to_string(iy * nx + ix));
        }
    }
    std::vector<std::uint32_t> materials(cell_count, spec.surfaces.front().id);
    for (std::size_t iy = 0; iy + 1 < ny; ++iy) for (std::size_t ix = 0; ix + 1 < nx; ++ix) {
        const double x = xs[ix], y = y_node(iy, ny, spec.half_width_m, spec.dy_m);
        for (const auto& zone : spec.zones)
            if (zone.x0_m <= x && x < zone.x1_m && zone.y0_m <= y && y < zone.y1_m)
                materials[iy * (nx - 1) + ix] = zone.surface_id;
    }
    const auto build = bike::platform::build_identity();
    Grid3dMetadata metadata;
    metadata.generator_version = spec.generator_version;
    metadata.resolved_spec = std::move(resolved_spec);
    metadata.seed_decimal = detail::seed_plus(spec.seed_decimal, 0);
    metadata.units = "m";
    metadata.height_datum_m = 0.;
    metadata.provenance = {bike::io::Value::Object{
        {"platform", bike::io::Value{build.platform}}, {"compiler", bike::io::Value{build.compiler}},
        {"dependency_lock_sha256", bike::io::Value{build.dependency_lock_sha256}},
        {"execution_identity", bike::io::Value{bike::platform::execution_identity()}},
        {"roughness_stream", bike::io::Value{std::string("terrain3d/roughness")}}}};
    auto grid = Grid3d::create({static_cast<std::uint64_t>(nx), static_cast<std::uint64_t>(ny),
        0., -spec.half_width_m, spec.dx_m, spec.dy_m, 0}, std::move(elevation), std::move(materials), spec.surfaces, std::move(metadata));
    if (!grid) throw std::invalid_argument(grid.error());
    return std::move(*grid);
}
} // namespace bike::terrain
