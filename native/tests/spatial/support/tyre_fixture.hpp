#pragma once

#include "../../../src/pneumatic3d/model.hpp"
#include "../../../src/spatial/types.hpp"

#include <array>

namespace spatial_tests {

// Synthetic fixture coefficients must never be presented as measured MTB data.
bike::pneumatic3d::Pneumatic3dSpec synthetic_spec(bike::spatial::TyreTier);
bike::pneumatic3d::WheelKinematics3d wheel_at(const bike::spatial::Vec3& hub,
    double yaw_rad, double camber_rad, double speed_mps, double spin_rad_s);
double full_torus_area(const bike::pneumatic3d::TorusGrid3d&);
double full_torus_volume(const bike::pneumatic3d::TorusGrid3d&);
double potential_energy_fd(const bike::pneumatic3d::Pneumatic3dModel&,
    const std::array<bike::pneumatic3d::WheelKinematics3d, 2>&,
    const bike::spatial::Vec3& displacement, double h);

} // namespace spatial_tests
