#include "terrain/source3d.hpp"

#include <utility>

namespace bike::terrain {
namespace {
template<class Source> GridSample3d sample_thunk(const void* context, double x, double y) noexcept {
    return static_cast<const Source*>(context)->sample(x, y);
}
template<class Source> RayHit3d ray_thunk(const void* context, std::array<double, 3> origin,
    std::array<double, 3> direction, double maximum) noexcept {
    return static_cast<const Source*>(context)->ray_cast(origin, direction, maximum);
}
}
GridSample3d Terrain3dQueryView::sample(double x, double y) const noexcept {
    return context && sample_fn ? sample_fn(context, x, y) : GridSample3d{};
}
RayHit3d Terrain3dQueryView::ray_cast(std::array<double, 3> origin, std::array<double, 3> direction, double maximum) const noexcept {
    return context && ray_fn ? ray_fn(context, origin, direction, maximum) : RayHit3d{};
}
const SurfaceSpec3d* Terrain3dQueryView::surface(std::uint32_t id) const noexcept {
    for (const auto& material : surface_table)
        if (material.id == id) return &material;
    return nullptr;
}
Terrain3dSource::Terrain3dSource(Grid3d source) : source_(std::move(source)) {}
Terrain3dSource::Terrain3dSource(ProfileExtrusion3d source) : source_(std::move(source)) {}
Terrain3dKind Terrain3dSource::kind() const noexcept {
    return grid() ? Terrain3dKind::grid : Terrain3dKind::profile_extrusion;
}
Terrain3dQueryView Terrain3dSource::query_view() const noexcept {
    if (const auto* owner = grid())
        return {.kind = Terrain3dKind::grid, .context = owner, .sample_fn = sample_thunk<Grid3d>,
                .ray_fn = ray_thunk<Grid3d>, .surface_table = owner->surfaces()};
    const auto* owner = extrusion();
    return {.kind = Terrain3dKind::profile_extrusion, .context = owner,
            .sample_fn = sample_thunk<ProfileExtrusion3d>, .ray_fn = ray_thunk<ProfileExtrusion3d>,
            .surface_table = owner->surfaces()};
}
const Grid3d* Terrain3dSource::grid() const noexcept { return std::get_if<Grid3d>(&source_); }
const ProfileExtrusion3d* Terrain3dSource::extrusion() const noexcept { return std::get_if<ProfileExtrusion3d>(&source_); }
const std::string& Terrain3dSource::fingerprint_hex() const noexcept {
    if (const auto* owner = grid()) return owner->sha256_hex();
    return extrusion()->fingerprint_hex();
}
double Terrain3dSource::x_min_m() const noexcept {
    if (const auto* owner = grid()) return owner->descriptor().x0_m;
    return extrusion()->knots_x_m().front();
}
double Terrain3dSource::x_max_m() const noexcept {
    if (const auto* owner = grid()) return owner->x_max_m();
    return extrusion()->knots_x_m().back();
}
double Terrain3dSource::half_width_m() const noexcept {
    if (const auto* owner = grid()) return .5 * (owner->y_max_m() - owner->descriptor().y0_m);
    return extrusion()->corridor_half_width_m();
}
} // namespace bike::terrain
