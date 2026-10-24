#pragma once

#include "terrain/extrusion3d.hpp"
#include "terrain/grid3d.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <variant>

namespace bike::terrain {

enum class Terrain3dKind : std::uint8_t { grid, profile_extrusion };

// Borrowed warm-path view of a validated, immovable owner. The owner must
// outlive every view; context and function pointers are non-null after binding.
struct Terrain3dQueryView {
    Terrain3dKind kind = Terrain3dKind::grid;
    const void* context = nullptr;
    GridSample3d (*sample_fn)(const void*, double, double) noexcept = nullptr;
    RayHit3d (*ray_fn)(const void*, std::array<double, 3>,
                      std::array<double, 3>, double) noexcept = nullptr;
    // Borrowed from the same immutable owner as context. Kept trailing so
    // geometry-only aggregate views may omit materials; tyre owners require
    // a table and reject every unresolved sample/ray surface ID.
    std::span<const SurfaceSpec3d> surface_table{};
    GridSample3d sample(double x_m, double y_m) const noexcept;
    RayHit3d ray_cast(std::array<double, 3> origin_m,
                     std::array<double, 3> unit_direction,
                     double max_distance_m) const noexcept;
    const SurfaceSpec3d* surface(std::uint32_t id) const noexcept;
};

class Terrain3dSource {
public:
    explicit Terrain3dSource(Grid3d);
    explicit Terrain3dSource(ProfileExtrusion3d);
    Terrain3dSource(Terrain3dSource&&) = delete;
    Terrain3dSource& operator=(Terrain3dSource&&) = delete;
    Terrain3dKind kind() const noexcept;
    Terrain3dQueryView query_view() const noexcept;
    const Grid3d* grid() const noexcept;
    const ProfileExtrusion3d* extrusion() const noexcept;
    const std::string& fingerprint_hex() const noexcept;
    double x_min_m() const noexcept;
    double x_max_m() const noexcept;
    double half_width_m() const noexcept;

private:
    std::variant<Grid3d, ProfileExtrusion3d> source_;
};

} // namespace bike::terrain
