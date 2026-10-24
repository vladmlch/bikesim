#include "terrain/generator3d.hpp"
#include "terrain/detail.hpp"
#include "terrain/interpolation3d.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"
#include "platform/provenance.hpp"
#include "random/numpy_compat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <memory>
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
    // Catmull interpolation reads one node beyond each end of a cell. A cell
    // containing a non-grid-aligned lead boundary can therefore read as far
    // as two spacings into the active region. Keep that halo flat as well,
    // otherwise lateral relief leaks back into the declared lead zones even
    // though every node inside them equals the base profile.
    const double start = spec.lead_in_m + 2. * spec.dx_m;
    const double stop = length - spec.lead_out_m - 2. * spec.dx_m;
    if (x <= start || x >= stop) return 0.;
    const double transition = std::min(1., .5 * (stop - start));
    return quintic_smoothstep((x - start) / transition) *
           quintic_smoothstep((stop - x) / transition);
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

using Gradient = std::array<double, 2>;
using GradientNet = std::array<Gradient, 16>;

Gradient middle(const Gradient& a, const Gradient& b) noexcept {
    return {.5 * a[0] + .5 * b[0], .5 * a[1] + .5 * b[1]};
}

// Express BOTH world-gradient components in the same bicubic Bernstein basis.
// They are the derivatives of the exact coefficients used by Grid3d::sample,
// not independently estimated slopes. The common weights are nonnegative and
// sum to one, so every gradient lies in the convex hull of this control net.
GradientNet gradient_net(const BicubicPatch& patch, double dx, double dy) noexcept {
    constexpr std::array<std::array<double, 4>, 4> power_to_bernstein{{
        {1., 0., 0., 0.}, {1., 1. / 3., 0., 0.},
        {1., 2. / 3., 1. / 3., 0.}, {1., 1., 1., 1.}}};
    GradientNet out{};
    for (std::size_t j = 0; j < 4; ++j) for (std::size_t i = 0; i < 4; ++i) {
        auto& gradient = out[j * 4 + i];
        for (std::size_t b = 0; b <= j; ++b) for (std::size_t a = 0; a <= i; ++a) {
            const double weight = power_to_bernstein[i][a] * power_to_bernstein[j][b];
            if (a < 3) gradient[0] += weight * static_cast<double>(a + 1) * patch.c[b * 4 + a + 1] / dx;
            if (b < 3) gradient[1] += weight * static_cast<double>(b + 1) * patch.c[(b + 1) * 4 + a] / dy;
        }
    }
    return out;
}

void split_gradient(const GradientNet& input, bool along_x, GradientNet& left, GradientNet& right) noexcept {
    for (std::size_t line = 0; line < 4; ++line) {
        const auto index = [&](std::size_t i) { return along_x ? line * 4 + i : i * 4 + line; };
        std::array<Gradient, 4> work;
        for (std::size_t i = 0; i < 4; ++i) work[i] = input[index(i)];
        left[index(0)] = work[0]; right[index(3)] = work[3];
        for (std::size_t level = 1; level < 4; ++level) {
            for (std::size_t i = 0; i < 4 - level; ++i) work[i] = middle(work[i], work[i + 1]);
            left[index(level)] = work[0]; right[index(3 - level)] = work[3 - level];
        }
    }
}

Gradient centre_gradient(const GradientNet& net) noexcept {
    std::array<Gradient, 4> rows;
    for (std::size_t j = 0; j < 4; ++j) {
        std::array<Gradient, 4> work;
        for (std::size_t i = 0; i < 4; ++i) work[i] = net[j * 4 + i];
        for (std::size_t n = 3; n != 0; --n)
            for (std::size_t i = 0; i < n; ++i) work[i] = middle(work[i], work[i + 1]);
        rows[j] = work[0];
    }
    for (std::size_t n = 3; n != 0; --n)
        for (std::size_t i = 0; i < n; ++i) rows[i] = middle(rows[i], rows[i + 1]);
    return rows[0];
}

enum class SlopeBound { valid, exceeds, exhausted };

constexpr unsigned kSlopeMaxDepth = 8;
struct SlopeInterval { GradientNet net; unsigned depth; };
// A depth-first four-way split has at most 1+3*depth pending cells.
using SlopeStack = std::array<SlopeInterval, 1 + 3 * kSlopeMaxDepth>;

SlopeBound bound_slope(const BicubicPatch& patch, double dx, double dy, double limit,
                      SlopeStack& pending) noexcept {
    constexpr unsigned max_subdivisions = 256;
    std::size_t count = 1;
    pending[0] = {gradient_net(patch, dx, dy), 0};
    unsigned subdivisions = 0;
    // Conversion/subdivision roundoff only; this is not a physical slope margin.
    const double bounded_limit = limit + 128. * std::numeric_limits<double>::epsilon() * std::max(1., limit);
    const auto magnitude = [](const Gradient& value) { return std::hypot(value[0], value[1]); };
    while (count != 0) {
        const auto current = pending[--count];
        double maximum = 0.;
        for (const auto& value : current.net) {
            const double size = magnitude(value);
            if (!std::isfinite(size)) return SlopeBound::exceeds;
            maximum = std::max(maximum, size);
        }
        if (maximum <= bounded_limit) continue;
        for (const auto corner : {0U, 3U, 12U, 15U})
            if (magnitude(current.net[corner]) > bounded_limit) return SlopeBound::exceeds;
        if (magnitude(centre_gradient(current.net)) > bounded_limit) return SlopeBound::exceeds;
        if (current.depth == kSlopeMaxDepth || subdivisions == max_subdivisions || count + 4 > pending.size())
            return SlopeBound::exhausted;
        ++subdivisions;
        GradientNet left, right, low_left, high_left, low_right, high_right;
        split_gradient(current.net, true, left, right);
        split_gradient(left, false, low_left, high_left);
        split_gradient(right, false, low_right, high_right);
        const unsigned next_depth = current.depth + 1;
        pending[count++] = {high_right, next_depth};
        pending[count++] = {high_left, next_depth};
        pending[count++] = {low_right, next_depth};
        pending[count++] = {low_left, next_depth};
    }
    return SlopeBound::valid;
}

void validate_bicubic_slopes(const Grid3d& grid, double limit) {
    const auto& d = grid.descriptor();
    // Cold validation owns one bounded scratch allocation for the entire grid;
    // the interval stack alone exceeds 6 KiB. Together with subdivision
    // temporaries it would exceed the per-function stack budget.
    const auto pending = std::make_unique<SlopeStack>();
    for (std::uint64_t iy = 0; iy + 1 < grid.ny(); ++iy) for (std::uint64_t ix = 0; ix + 1 < grid.nx(); ++ix) {
        const auto status = bound_slope(bicubic_patch(grid, ix, iy), d.dx_m, d.dy_m, limit, *pending);
        if (status == SlopeBound::valid) continue;
        const std::string path = "terrain3d.surface.cell[" + std::to_string(iy) + "][" + std::to_string(ix) + "]";
        throw std::invalid_argument(path + (status == SlopeBound::exceeds
            ? ": bicubic gradient is non-finite or exceeds slope limit"
            : ": bicubic slope bound exceeds validation budget"));
    }
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
    validate_bicubic_slopes(*grid, slope_limit);
    return std::move(*grid);
}
} // namespace bike::terrain
