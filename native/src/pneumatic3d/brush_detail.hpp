#pragma once

// Private seam for the owning model: the frozen public kernels do not carry
// material coefficients or the wheel axle/angular velocity. Resolve those
// from the terrain and kinematics once, outside the arithmetic below.
#include "pneumatic3d/bristles.hpp"

namespace bike::pneumatic3d::detail {
struct FastBrushResult {
    CoreStatus status=CoreStatus::ok;
    PatchWrench wrench;
    double ux_m=0., uy_m=0., stored_j=0., sliding_j=0., convective_j=0.;
};
FastBrushResult evaluate_fast_patch(const BrushSpec3d&, const Patch3d&,
    const bike::spatial::Vec3& point_velocity, const bike::spatial::Vec3& hub_velocity,
    const bike::spatial::Vec3& axle_world, double spin_rad_s, double camber_rad,
    double mu_x, double mu_y, double old_ux, double old_uy, double dt_s,
    const bike::spatial::Vec3& angular_velocity_world = {}) noexcept;
FastBrushResult evaluate_bristle(const BrushSpec3d&, const ContactElement3d&,
    const NormalElement3d&, const WheelKinematics3d&, double mu_x, double mu_y,
    double old_ux, double old_uy, double dt_s) noexcept;
CoreStatus update_detailed_resolved(const BrushSpec3d&,
    std::span<const ContactElement3d>,std::span<const NormalElement3d>,
    const WheelKinematics3d&,double dt_s,bool advance,BrushState3d&,
    std::span<PatchWrench>,const bike::terrain::Terrain3dQueryView&) noexcept;
} // namespace bike::pneumatic3d::detail
