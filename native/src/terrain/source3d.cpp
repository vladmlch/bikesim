#include "terrain/source3d.hpp"

namespace bike::terrain {

const TerrainBounds3d& Terrain3dSource::bounds() const noexcept {
    if (const auto* ext = std::get_if<ExtrudedTrack3d>(&storage_)) {
        return ext->bounds();
    }
    if (const auto* grid = std::get_if<Grid3d>(&storage_)) {
        return grid->bounds();
    }
    static const TerrainBounds3d kEmpty{};
    return kEmpty;
}

const std::string& Terrain3dSource::sha256_hex() const noexcept {
    if (const auto* ext = std::get_if<ExtrudedTrack3d>(&storage_)) {
        return ext->sha256_hex();
    }
    if (const auto* grid = std::get_if<Grid3d>(&storage_)) {
        return grid->sha256_hex();
    }
    static const std::string kEmpty{};
    return kEmpty;
}

TerrainSample3d Terrain3dSource::sample(double x_m, double y_m) const noexcept {
    if (const auto* ext = std::get_if<ExtrudedTrack3d>(&storage_)) {
        return ext->sample(x_m, y_m);
    }
    if (const auto* grid = std::get_if<Grid3d>(&storage_)) {
        return grid->sample(x_m, y_m);
    }
    return {};
}

TorusClosestPoint3d Terrain3dSource::closest_point(const Vec3d& center_world_m,
                                                   double search_radius_m) const noexcept {
    if (const auto* ext = std::get_if<ExtrudedTrack3d>(&storage_)) {
        return ext->closest_point(center_world_m, search_radius_m);
    }
    if (const auto* grid = std::get_if<Grid3d>(&storage_)) {
        return grid->closest_point(center_world_m, search_radius_m);
    }
    return {};
}

TerrainBounds3d Terrain3dQueryView::bounds() const noexcept {
    return owner_ ? owner_->bounds() : TerrainBounds3d{};
}

std::string_view Terrain3dQueryView::sha256_hex() const noexcept {
    return owner_ ? std::string_view(owner_->sha256_hex()) : std::string_view{};
}

TerrainSample3d Terrain3dQueryView::sample(double x_m, double y_m) const noexcept {
    return owner_ ? owner_->sample(x_m, y_m) : TerrainSample3d{};
}

TorusClosestPoint3d Terrain3dQueryView::closest_point(const Vec3d& center_world_m,
                                                      double search_radius_m) const noexcept {
    return owner_ ? owner_->closest_point(center_world_m, search_radius_m) : TorusClosestPoint3d{};
}

} // namespace bike::terrain
