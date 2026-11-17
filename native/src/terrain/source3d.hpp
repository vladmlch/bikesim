#pragma once

#include "terrain/extrusion3d.hpp"
#include "terrain/grid3d.hpp"

#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace bike::terrain {

class Terrain3dSource;

class Terrain3dQueryView {
public:
    Terrain3dQueryView() noexcept = default;
    explicit Terrain3dQueryView(const Terrain3dSource* owner) noexcept : owner_(owner) {}

    [[nodiscard]] bool valid() const noexcept { return owner_ != nullptr; }
    [[nodiscard]] TerrainBounds3d bounds() const noexcept;
    [[nodiscard]] std::string_view sha256_hex() const noexcept;
    [[nodiscard]] TerrainSample3d sample(double x_m, double y_m) const noexcept;
    [[nodiscard]] TorusClosestPoint3d closest_point(const Vec3d& center_world_m,
                                                    double search_radius_m) const noexcept;

private:
    const Terrain3dSource* owner_ = nullptr;
};

class Terrain3dSource {
public:
    using Storage = std::variant<ExtrudedTrack3d, Grid3d>;

    explicit Terrain3dSource(ExtrudedTrack3d extruded) : storage_(std::move(extruded)) {}
    explicit Terrain3dSource(Grid3d grid) : storage_(std::move(grid)) {}

    Terrain3dSource(const Terrain3dSource&) = delete;
    Terrain3dSource& operator=(const Terrain3dSource&) = delete;
    Terrain3dSource(Terrain3dSource&&) = delete;
    Terrain3dSource& operator=(Terrain3dSource&&) = delete;
    ~Terrain3dSource() = default;

    [[nodiscard]] Terrain3dQueryView view() const noexcept { return Terrain3dQueryView(this); }
    [[nodiscard]] const TerrainBounds3d& bounds() const noexcept;
    [[nodiscard]] const std::string& sha256_hex() const noexcept;
    [[nodiscard]] TerrainSample3d sample(double x_m, double y_m) const noexcept;
    [[nodiscard]] TorusClosestPoint3d closest_point(const Vec3d& center_world_m,
                                                    double search_radius_m) const noexcept;

private:
    Storage storage_;
};

} // namespace bike::terrain
