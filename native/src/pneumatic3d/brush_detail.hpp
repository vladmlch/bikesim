#pragma once

// Private seam for the owning model: the frozen public kernels do not carry
// material coefficients or the wheel axle/angular velocity. Resolve those
// from the terrain and kinematics once, outside the arithmetic below.
#include "pneumatic3d/bristles.hpp"

#include <cmath>

namespace bike::pneumatic3d::detail {
// Single home of the Stribeck friction law shared by every resolved tyre
// kernel. A nonpositive transition speed is the zero-width limit: peak at
// exactly zero slip speed, sliding value otherwise. A NaN transition speed
// deliberately falls through to the exponential form so the NaN reaches the
// caller's finite check instead of being laundered into a finite coefficient.
inline double stribeck_mu(const bike::terrain::SurfaceSpec3d& material,
    double slip_speed_mps) noexcept {
    const double speed = std::abs(slip_speed_mps);
    if (material.stribeck_speed_mps <= 0.)
        return speed == 0. ? material.mu_peak : material.mu_slide;
    const double ratio = speed / material.stribeck_speed_mps;
    return material.mu_slide + (material.mu_peak - material.mu_slide) * std::exp(-ratio * ratio);
}
struct FastBrushResult {
    CoreStatus status=CoreStatus::ok;
    PatchWrench wrench;
    double ux_m=0., uy_m=0., stored_j=0., sliding_j=0., convective_j=0.;
};
FastBrushResult evaluate_fast_patch(const BrushSpec3d&, const Patch3d&,
    const bike::spatial::Vec3& point_velocity, const bike::spatial::Vec3& hub_velocity,
    // Spin is the absolute wheel angular velocity projected onto its axle,
    // conjugate to the wheel-body couple applied by the writer.
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
