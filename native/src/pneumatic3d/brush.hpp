#pragma once

#include "pneumatic3d/patches.hpp"
#include "pneumatic3d/state.hpp"
#include "spatial/types.hpp"
#include "writers/writer_types.hpp"

#include <cstdint>
#include <span>
#include <string>

namespace bike::pneumatic3d {

struct BrushSpec3d {
    double kx_n_per_m3 = 0., ky_n_per_m3 = 0.;
    double relaxation_x_m = 0., relaxation_y_m = 0.;
    double mu_x_scale = 1., mu_y_scale = 1.;
    double camber_coefficient_per_rad = 0.;
    double pneumatic_trail_m = 0.;
    double r_eff_m = 0.;
    std::string provenance_status;
};

// Force and intrinsic couple are world vectors about point_world_m. The
// lever-arm cross product is not included in couple_world_nm (contract C4).
struct PatchWrench {
    bike::spatial::Vec3 point_world_m{}, force_world_n{}, couple_world_nm{};
    double normal_load_n = 0., fx_n = 0., fy_n = 0., mz_nm = 0.;
    double slip_ratio = 0., slip_angle_rad = 0., camber_rad = 0.;
    std::uint32_t surface_id = 0;
    std::uint64_t patch_id = 0;
    bool valid = false;
    double loss_j = 0.;
};

// For count patches, the velocity span has count+1 entries: material-point
// velocities at patch centroids (including spin), followed by hub velocity.
// Include every slot with nonzero displacement history, even when its patch
// has become unloaded. Spare state capacity may only have zero ux/uy: this
// standalone interface has no old patch areas with which to release omitted
// spring energy. An incomplete history span returns invalid_input unchanged.
CoreStatus update_fast_brush(
    const BrushSpec3d&, std::span<const Patch3d>,
    std::span<const bike::spatial::Vec3> point_velocity_world_mps,
    double dt_s, bool advance, BrushState3d&, std::span<PatchWrench>) noexcept;

} // namespace bike::pneumatic3d
