#include "terrain/extrusion3d.hpp"
#include "terrain/detail.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include "tyre/profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::terrain {
namespace {
std::size_t segment(std::span<const double> x, double query) noexcept {
    const auto upper = std::upper_bound(x.begin(), x.end(), query);
    if (upper == x.begin()) return 0;
    return std::min(static_cast<std::size_t>(upper - x.begin() - 1), x.size() - 2);
}
bool clip_axis(double origin, double direction, double lo, double hi,
               double& enter, double& leave) noexcept {
    if (direction == 0.) return lo <= origin && origin <= hi;
    double a = (lo - origin) / direction, b = (hi - origin) / direction;
    if (a > b) std::swap(a, b);
    enter = std::max(enter, a); leave = std::min(leave, b);
    return enter <= leave;
}
double cross2(double ax, double az, double bx, double bz) noexcept {
    // Identical operation order to pneumatic/geometry.cpp::_segment_distance.
    return ax * bz - az * bx;
}
}

std::expected<ProfileExtrusion3d, std::string> ProfileExtrusion3d::create(
    std::vector<double> kx, std::vector<double> kz, std::vector<std::uint32_t> segment_ids,
    std::vector<SurfaceSpec3d> surfaces, double half_width, bike::io::Value source) {
    if (kx.size() < 2 || kx.size() != kz.size())
        return std::unexpected("terrain3d.extrusion.knots: need matching arrays with at least two knots");
    if (segment_ids.size() != kx.size() - 1)
        return std::unexpected("terrain3d.extrusion.segment_surface_ids: count mismatch");
    if (!std::isfinite(half_width) || half_width <= 0.)
        return std::unexpected("terrain3d.extrusion.corridor_half_width_m: expected finite positive width");
    if (surfaces.empty()) return std::unexpected("terrain3d.extrusion.surfaces: required");
    for (std::size_t i = 0; i < kx.size(); ++i) {
        const auto path = "terrain3d.extrusion.knots[" + std::to_string(i) + "]";
        if (!std::isfinite(kx[i]) || !std::isfinite(kz[i])) return std::unexpected(path + ": non-finite coordinate");
        if (i != 0 && (!(kx[i] > kx[i - 1]) || !std::isfinite(kx[i] - kx[i - 1]) ||
            !std::isfinite((kz[i] - kz[i - 1]) / (kx[i] - kx[i - 1]))))
            return std::unexpected(path + ": require increasing finite spans and finite slopes");
    }
    for (std::size_t i = 0; i < surfaces.size(); ++i) {
        const auto& surface = surfaces[i];
        const auto path = "terrain3d.extrusion.surfaces[" + std::to_string(i) + "]";
        if (surface.name.empty() || surface.name.find('\0') != std::string::npos)
            return std::unexpected(path + ".name: invalid name");
        if (!std::isfinite(surface.mu_peak) || surface.mu_peak < 0. || !std::isfinite(surface.mu_slide) || surface.mu_slide < 0. ||
            !std::isfinite(surface.slip_stiffness_per_load) || surface.slip_stiffness_per_load < 0. ||
            !std::isfinite(surface.stribeck_speed_mps) || surface.stribeck_speed_mps <= 0.)
            return std::unexpected(path + ": invalid friction coefficients");
        for (std::size_t j = 0; j < i; ++j) if (surfaces[j].id == surface.id)
            return std::unexpected(path + ".id: duplicate surface ID");
    }
    for (std::size_t i = 0; i < segment_ids.size(); ++i)
        if (std::ranges::none_of(surfaces, [&](const auto& surface) { return surface.id == segment_ids[i]; }))
            return std::unexpected("terrain3d.extrusion.segment_surface_ids[" + std::to_string(i) + "]: unknown surface ID");
    using bike::io::Value;
    Value::Array x_values, z_values, ids, table;
    x_values.reserve(kx.size()); z_values.reserve(kz.size()); ids.reserve(segment_ids.size());
    for (const double value : kx) x_values.push_back(detail::number(value));
    for (const double value : kz) z_values.push_back(detail::number(value));
    for (const auto value : segment_ids) ids.push_back(detail::integer(value));
    for (const auto& surface : surfaces) table.push_back({Value::Object{
        {"id", detail::integer(surface.id)}, {"name", Value{surface.name}}, {"mu_peak", detail::number(surface.mu_peak)},
        {"mu_slide", detail::number(surface.mu_slide)}, {"slip_stiffness_per_load", detail::number(surface.slip_stiffness_per_load)},
        {"stribeck_speed_mps", detail::number(surface.stribeck_speed_mps)}, {"description", Value{surface.description}}}});
    std::string fingerprint;
    try {
        const auto canonical = bike::io::canonical_json({Value::Object{
            {"kind", Value{std::string("profile_extrusion/1")}}, {"knots_x_m", Value{std::move(x_values)}},
            {"knots_z_m", Value{std::move(z_values)}}, {"segment_surface_ids", Value{std::move(ids)}},
            {"surfaces", Value{std::move(table)}}, {"corridor_half_width_m", detail::number(half_width)}, {"source", source}}});
        fingerprint = bike::io::sha256(std::as_bytes(std::span(canonical.data(), canonical.size())));
    } catch (const std::invalid_argument& error) {
        return std::unexpected("terrain3d.extrusion.source: " + std::string(error.what()));
    }
    ProfileExtrusion3d out;
    out.kx_ = std::move(kx); out.kz_ = std::move(kz); out.seg_surface_ = std::move(segment_ids);
    out.surfaces_ = std::move(surfaces); out.half_width_ = half_width; out.source_ = std::move(source);
    out.fingerprint_hex_ = std::move(fingerprint);
    return out;
}

GridSample3d ProfileExtrusion3d::sample(double x, double y) const noexcept {
    if (!std::isfinite(x) || !std::isfinite(y)) return {};
    if (x < kx_.front() || x > kx_.back() || std::abs(y) > half_width_)
        return {.status = QueryStatus3d::outside};
    const auto i = segment(kx_, x);
    const double slope = (kz_[i + 1] - kz_[i]) / (kx_[i + 1] - kx_[i]);
    const double norm = std::hypot(slope, 1.);
    // Both planar SignedProfile and RoadProfile use this same np.interp
    // primitive (including exact knots and the required explicit FMA).
    return {QueryStatus3d::ok, biketyre::np_interp(x, kx_, kz_), {-slope / norm, 0., 1. / norm}, seg_surface_[i]};
}

RayHit3d ProfileExtrusion3d::ray_cast(std::array<double, 3> origin,
    std::array<double, 3> direction, double maximum) const noexcept {
    for (const double value : origin) if (!std::isfinite(value)) return {};
    for (const double value : direction) if (!std::isfinite(value)) return {};
    if (!std::isfinite(maximum) || maximum < 0. ||
        std::abs(std::hypot(direction[0], direction[1], direction[2]) - 1.) > 1e-9) return {};
    double enter = 0., leave = maximum;
    if (!clip_axis(origin[0], direction[0], kx_.front(), kx_.back(), enter, leave) ||
        !clip_axis(origin[1], direction[1], -half_width_, half_width_, enter, leave))
        return {.status = QueryStatus3d::outside};
    const std::array<double, 3> start{
        std::clamp(origin[0] + enter * direction[0], kx_.front(), kx_.back()),
        std::clamp(origin[1] + enter * direction[1], -half_width_, half_width_),
        origin[2] + enter * direction[2]};
    const auto at_start = sample(start[0], start[1]);
    if (start[2] < at_start.height_m) return {.status = QueryStatus3d::origin_below_surface};
    const auto hit_at = [&](double distance) -> RayHit3d {
        const std::array<double, 3> point{origin[0] + distance * direction[0], origin[1] + distance * direction[1], origin[2] + distance * direction[2]};
        const auto sampled = sample(std::clamp(point[0], kx_.front(), kx_.back()), std::clamp(point[1], -half_width_, half_width_));
        return {QueryStatus3d::ok, distance, point, sampled.normal_world, sampled.surface_id};
    };
    if (start[2] == at_start.height_m) return hit_at(enter);
    const double projected_norm = direction[1] == 0. ? 1. : std::hypot(direction[0], direction[2]);
    if (projected_norm == 0.) {
        if (std::abs(start[2] - at_start.height_m) <= 1e-8) return hit_at(enter);
        return {.status = QueryStatus3d::miss};
    }
    // The frozen API permits nonuniform knots and arbitrary rays, whereas
    // the legacy entry point owns a uniform RoadProfile/RayRing. Retain the
    // literal legacy scalar segment arithmetic here; shared primitive
    // extraction is recorded for V00/C10. No dy==0 renormalization occurs.
    const double ux = direction[0] / projected_norm, uz = direction[2] / projected_norm;
    const double end_x = std::clamp(origin[0] + leave * direction[0], kx_.front(), kx_.back());
    const auto first = segment(kx_, std::min(start[0], end_x));
    const auto last = segment(kx_, std::max(start[0], end_x));
    double best = std::numeric_limits<double>::infinity();
    const double available = (leave - enter) * projected_norm;
    for (std::size_t i = first; i <= last; ++i) {
        const double dx = kx_[i + 1] - kx_[i], dz = kz_[i + 1] - kz_[i];
        const double denominator = cross2(ux, uz, dx, dz);
        // Preserve the existing planar primitive's cutoff only on its exact
        // dy==0 bridge. In general spatial queries the determinant also scales
        // with segment length; a fixed cutoff discards real shallow crossings.
        if (direction[1] == 0. ? std::abs(denominator) <= 1e-15 : denominator == 0.) continue;
        const double wx = kx_[i] - start[0], wz = kz_[i] - start[2];
        const double distance = cross2(wx, wz, dx, dz) / denominator;
        const double along = cross2(wx, wz, ux, uz) / denominator;
        if (distance > 0. && distance <= available && along >= 0. && along <= 1. && distance < best) best = distance;
    }
    if (!std::isfinite(best)) return {.status = QueryStatus3d::miss};
    return hit_at(enter + best / projected_norm);
}

double ProfileExtrusion3d::corridor_half_width_m() const noexcept { return half_width_; }
std::span<const double> ProfileExtrusion3d::knots_x_m() const noexcept { return kx_; }
std::span<const double> ProfileExtrusion3d::knots_z_m() const noexcept { return kz_; }
std::span<const SurfaceSpec3d> ProfileExtrusion3d::surfaces() const noexcept { return surfaces_; }
const std::string& ProfileExtrusion3d::fingerprint_hex() const noexcept { return fingerprint_hex_; }
} // namespace bike::terrain
