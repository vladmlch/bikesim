#pragma once

#include "spatial_sensor_pipeline.hpp"
#include "../io/value.hpp"
#include "../spatial/frames.hpp"
#include "../spatial/types.hpp"

#include <array>

namespace runtime::spatial {

// Rejects nonfinite values, negative accumulated distance and negative dt.
double integrate_abs_slip(double accumulated_m, double slip_mps, double dt_s);

struct SpatialTruth {
    bike::spatial::BodyPose root;
    bike::spatial::EulerZyx attitude;
    double lateral_displacement_m = 0.;
    double steer_angle_rad = 0., steer_rate_rad_s = 0.;
    double steer_torque_nm = 0., steer_work_j = 0.;
    std::array<bool, 2> contact_valid{};
    std::array<double, 2> lateral_slip_mps{}, normal_load_n{}, slip_path_m{};
    std::array<Vector3, 2> support_normal_world{};
    bool front_unloaded = false, front_lift = false, flight = false;
};

bike::io::Value spatial_truth_to_wire(const SpatialTruth&);

} // namespace runtime::spatial
