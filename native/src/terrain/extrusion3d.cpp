#include "terrain/extrusion3d.hpp"

#include "io/hash.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace bike::terrain {
namespace {

void hash_f64(bike::io::Sha256& h, double v) noexcept {
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(v == 0. ? 0. : v);
    std::array<std::byte, 8> le{};
    for (std::size_t i = 0; i < 8; ++i) {
        le[i] = static_cast<std::byte>((bits >> (8U * i)) & 0xffU);
    }
    h.update(le);
}

void hash_u8(bike::io::Sha256& h, std::uint8_t v) noexcept {
    const std::array<std::byte, 1> b{static_cast<std::byte>(v)};
    h.update(b);
}

std::string hex_digest(const std::array<std::uint8_t, 32>& digest) {
    static constexpr std::array<char, 16> kHex{
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
    };
    std::string out(64, '0');
    for (std::size_t i = 0; i < 32; ++i) {
        const auto byte = static_cast<unsigned>(digest[i]);
        out[2U * i] = kHex[(byte >> 4U) & 0x0fU];
        out[2U * i + 1U] = kHex[byte & 0x0fU];
    }
    return out;
}

std::vector<Point> build_shifted_vertices(const TrackSpec& track, double datum_z_offset_m,
                                          double resolution_m) {
    track.validate();
    if (!std::isfinite(datum_z_offset_m) || !std::isfinite(resolution_m) || !(resolution_m > 0.)) {
        throw std::invalid_argument("ExtrudedTrack3d: finite datum and positive resolution required");
    }
    std::vector<Point> pts = profile(track, resolution_m);
    for (Point& p : pts) {
        p[1] += datum_z_offset_m;
    }
    return pts;
}

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (half_width_m, datum_z_offset_m, resolution_m) matches public constructor contract
ExtrudedTrack3d::ExtrudedTrack3d(const TrackSpec& track, double half_width_m,
                                 double datum_z_offset_m, double resolution_m)
    : signed_profile_(build_shifted_vertices(track, datum_z_offset_m, resolution_m)) {
    if (!std::isfinite(half_width_m) || !(half_width_m > 0.)) {
        throw std::invalid_argument("ExtrudedTrack3d: positive finite half_width_m required");
    }
    const auto& verts = signed_profile_.vertices();
    xs_.resize(verts.size());
    zs_.resize(verts.size());
    double z_min = verts.front()[1];
    double z_max = verts.front()[1];
    for (std::size_t i = 0; i < verts.size(); ++i) {
        xs_[i] = verts[i][0];
        zs_[i] = verts[i][1];
        z_min = std::min(z_min, zs_[i]);
        z_max = std::max(z_max, zs_[i]);
    }

    const auto def_lbl = ::rider::surface_label(track.surface);
    if (!def_lbl.has_value()) {
        throw std::invalid_argument("ExtrudedTrack3d: unknown default surface '" + track.surface + "'");
    }
    default_label_ = *def_lbl;

    const SurfaceMap smap = track.surface_map();
    sections_.reserve(smap.sections().size());
    for (const SurfaceSection& sec : smap.sections()) {
        const auto lbl = ::rider::surface_label(sec.surface);
        if (!lbl.has_value()) {
            throw std::invalid_argument("ExtrudedTrack3d: unknown section surface '" + sec.surface + "'");
        }
        sections_.push_back(LabelSpan{
            .start_m = sec.start_m,
            .end_m = sec.end_m,
            .label = *lbl,
        });
    }

    bounds_ = TerrainBounds3d{
        .x_min_m = xs_.front(),
        .x_max_m = xs_.back(),
        .y_min_m = -half_width_m,
        .y_max_m = half_width_m,
        .z_min_m = z_min,
        .z_max_m = z_max,
    };

    bike::io::Sha256 hasher;
    hash_u8(hasher, 1U);
    hash_f64(hasher, half_width_m);
    hash_u8(hasher, static_cast<std::uint8_t>(default_label_));
    for (std::size_t i = 0; i < xs_.size(); ++i) {
        hash_f64(hasher, xs_[i]);
        hash_f64(hasher, zs_[i]);
    }
    for (const LabelSpan& sec : sections_) {
        hash_f64(hasher, sec.start_m);
        hash_f64(hasher, sec.end_m);
        hash_u8(hasher, static_cast<std::uint8_t>(sec.label));
    }
    sha256_hex_ = hex_digest(hasher.finish());
}

::rider::SurfaceLabel ExtrudedTrack3d::label_at(double x_m) const noexcept {
    ::rider::SurfaceLabel out = default_label_;
    for (const LabelSpan& sec : sections_) {
        if (x_m >= sec.start_m && x_m < sec.end_m) {
            out = sec.label;
        }
    }
    return out;
}

TerrainSample3d ExtrudedTrack3d::sample(double x_m, double y_m) const noexcept {
    if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
        return {.status = QueryStatus3d::invalid_input};
    }
    if (x_m < bounds_.x_min_m || x_m > bounds_.x_max_m ||
        y_m < bounds_.y_min_m || y_m > bounds_.y_max_m) {
        return {.status = QueryStatus3d::outside};
    }
    try {
        const double h = biketyre::np_interp(x_m, xs_, zs_);
        const auto upper = std::ranges::upper_bound(xs_, x_m);
        std::size_t seg = 0;
        if (upper == xs_.begin()) {
            seg = 0;
        } else if (upper == xs_.end()) {
            seg = xs_.size() - 2U;
        } else {
            seg = static_cast<std::size_t>(upper - xs_.begin()) - 1U;
        }
        const double dx = xs_[seg + 1U] - xs_[seg];
        const double dz = zs_[seg + 1U] - zs_[seg];
        const double len = std::hypot(dx, dz);
        const Vec3d normal = len > 0. ? Vec3d{-dz / len, 0., dx / len} : Vec3d{0., 0., 1.};
        return TerrainSample3d{
            .status = QueryStatus3d::ok,
            .height_m = h,
            .normal_world = normal,
            .surface = label_at(x_m),
        };
    } catch (...) {
        return {.status = QueryStatus3d::invalid_input};
    }
}

TorusClosestPoint3d ExtrudedTrack3d::closest_point(const Vec3d& center_world_m,
                                                   double search_radius_m) const noexcept {
    if (!std::isfinite(center_world_m[0]) || !std::isfinite(center_world_m[1]) ||
        !std::isfinite(center_world_m[2]) || !std::isfinite(search_radius_m) ||
        !(search_radius_m > 0.)) {
        return {.status = QueryStatus3d::invalid_input};
    }
    if (center_world_m[0] < bounds_.x_min_m || center_world_m[0] > bounds_.x_max_m ||
        center_world_m[1] < bounds_.y_min_m || center_world_m[1] > bounds_.y_max_m) {
        return {.status = QueryStatus3d::outside};
    }
    try {
        const std::array<Point, 1> query_xz{Point{center_world_m[0], center_world_m[2]}};
        const ProfileDistance res = signed_profile_.query(query_xz);
        const double px = res.point[0][0];
        return TorusClosestPoint3d{
            .status = QueryStatus3d::ok,
            .signed_distance_m = res.distance[0],
            .surface_point_world_m = {px, center_world_m[1], res.point[0][1]},
            .normal_world = {res.normal[0][0], 0., res.normal[0][1]},
            .surface = label_at(px),
        };
    } catch (...) {
        return {.status = QueryStatus3d::invalid_input};
    }
}

} // namespace bike::terrain
