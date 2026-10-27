#pragma once

#include "spatial/types.hpp"
#include "terrain/source3d.hpp"
#include "writers/writer_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

namespace bike::pneumatic3d {

using ::CoreStatus;

enum class ReasonCode3d : std::uint8_t {
    none, terrain_query_failed, terrain_budget_exhausted, origin_below_surface,
    nonfinite_geometry, degenerate_down_projection, patch_overflow,
    invalid_tier, invalid_spec, state_identity_mismatch, duplicate_advance,
    writer_not_bound, engine_contact_enabled
};

struct TorusDims {
    std::uint32_t n_theta_fan = 64, n_phi = 7, n_theta_material = 256;
};

TorusDims tier_dims(bike::spatial::TyreTier) noexcept;

struct TorusGrid3d {
    TorusDims dims;
    double R_m = 0., r_m = 0.;
    double theta_min_rad = -75. * std::numbers::pi / 180.;
    double theta_max_rad = 75. * std::numbers::pi / 180.;
    double phi_min_rad = -70. * std::numbers::pi / 180.;
    double phi_max_rad = 70. * std::numbers::pi / 180.;
    std::vector<double> theta_fan, phi, w_theta, w_phi;
    // it*n_phi+ip in the fan frame. Area is the exact cell integral
    // r*dtheta*(R*dphi + r*(sin(phi_hi)-sin(phi_lo))), not the midpoint product.
    std::vector<std::array<double, 3>> local_position_fan;
    std::vector<double> geometric_area_m2;
    std::size_t element_count() const noexcept;
    std::size_t material_cell_count() const noexcept;
};

// Cold construction: r = casing_width/2, R = outer_radius-r.
TorusGrid3d make_torus_grid(TorusDims, double outer_radius_m,
                           double casing_width_m);

struct WheelKinematics3d {
    bike::spatial::BodyPose hub;
    bike::spatial::Vec3 axle_world{0., 1., 0.};
    double relative_spin_rad_s = 0.;
    double wheel_phase_rad = 0.;
};

struct FanFrame {
    bike::spatial::Vec3 axle, down, forward;
    bool valid = false;
};

// Down is world -Z projected into the wheel plane; forward = down x axle.
FanFrame fan_frame(const WheelKinematics3d&) noexcept;

struct ContactElement3d {
    std::uint32_t material_cell = 0;
    bike::terrain::QueryStatus3d ray_status =
        bike::terrain::QueryStatus3d::invalid_input;
    bike::spatial::Vec3 point_world_m{}, normal_world{};
    bike::spatial::Vec3 longitudinal_world{}, lateral_world{};
    double geometric_area_m2 = 0., projected_area_m2 = 0., penetration_m = 0.;
    std::uint32_t surface_id = 0;
    bool valid = false;
    ReasonCode3d reason = ReasonCode3d::none;
};

CoreStatus query_geometry(const TorusGrid3d&, const WheelKinematics3d&,
                          const bike::terrain::Terrain3dQueryView&,
                          std::span<ContactElement3d>) noexcept;

} // namespace bike::pneumatic3d
