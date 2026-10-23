#pragma once

#include "io/value.hpp"

#include <array>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

namespace bike::terrain {

enum class QueryStatus3d : std::uint8_t {
    ok, miss, outside, invalid_input, budget_exhausted, origin_below_surface
};

// Payloads are meaningful only when status == ok. Positions and distances
// are in the world frame, in metres; successful normals are unit and z > 0.
struct GridSample3d {
    QueryStatus3d status = QueryStatus3d::invalid_input;
    double height_m = 0.;
    std::array<double, 3> normal_world{};
    std::uint32_t surface_id = 0;
};

struct RayHit3d {
    QueryStatus3d status = QueryStatus3d::invalid_input;
    double distance_m = 0.;
    std::array<double, 3> point_world_m{}, normal_world{};
    std::uint32_t surface_id = 0;
};

struct SurfaceSpec3d {
    std::uint32_t id = 0;
    std::string name;
    double mu_peak = 0., mu_slide = 0., slip_stiffness_per_load = 0.;
    double stribeck_speed_mps = 0.;
    std::string description;
};

struct Grid3dDescriptor {
    std::uint64_t nx = 0, ny = 0;
    double x0_m = 0., y0_m = 0., dx_m = 0., dy_m = 0.;
    std::uint32_t dtype = 0; // 0 = f64; 1 = f32 in the canonical asset.
};

struct Grid3dMetadata {
    std::string generator_version;
    bike::io::Value resolved_spec;
    std::string seed_decimal;
    std::string units = "m";
    double height_datum_m = 0.;
    bike::io::Value provenance;
};

struct RayBudget {
    std::uint32_t max_cells = 4096;
    std::uint32_t max_subdivisions = 64;
    std::uint32_t stack_capacity = 128;
    double root_interval_m = 1e-6;
    double residual_m = 1e-8;
};

class Grid3d {
public:
    // Cold construction validates dimensions, finite data, materials and limits.
    static std::expected<Grid3d, std::string> create(
        Grid3dDescriptor, std::vector<double> elevation_row_major_iy_nx_plus_ix,
        std::vector<std::uint32_t> material_ids_cells,
        std::vector<SurfaceSpec3d>, Grid3dMetadata, RayBudget = {});

    std::uint64_t nx() const noexcept;
    std::uint64_t ny() const noexcept;
    const Grid3dDescriptor& descriptor() const noexcept;
    std::span<const double> elevation() const noexcept;
    std::span<const std::uint32_t> materials() const noexcept;
    std::span<const SurfaceSpec3d> surfaces() const noexcept;
    const Grid3dMetadata& metadata() const noexcept;
    const RayBudget& budget() const noexcept;
    double x_max_m() const noexcept;
    double y_max_m() const noexcept;
    GridSample3d sample(double x_m, double y_m) const noexcept;
    RayHit3d ray_cast(std::array<double, 3> origin_m,
                     std::array<double, 3> unit_direction,
                     double max_distance_m) const noexcept;
    // SHA-256 of the complete canonical write_terrain3d byte sequence.
    const std::string& sha256_hex() const noexcept;

private:
    Grid3dDescriptor desc_;
    std::vector<double> elevation_;
    std::vector<std::uint32_t> materials_;
    std::vector<SurfaceSpec3d> surfaces_;
    std::vector<std::uint32_t> surface_index_by_id_; // Missing ID: 0xFFFFFFFF.
    Grid3dMetadata meta_;
    RayBudget budget_;
    std::string sha256_hex_;
};

} // namespace bike::terrain
