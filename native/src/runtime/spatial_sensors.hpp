#pragma once

#include "spatial_sensor_pipeline.hpp"
#include "../spatial/config.hpp"
#include "../spatial/topology.hpp"

#include <mujoco/mujoco.h>

#include <string>

namespace runtime::spatial {

struct ImuMountBinding {
    int site_id = -1;
    int body_id = -1;
    int accel_sensor_id = -1, gyro_sensor_id = -1;
    int accel_adr = -1, gyro_adr = -1;
    Matrix3 sensor_from_nominal{1., 0., 0., 0., 1., 0., 0., 0., 1.};
    std::string calibration_revision;
};

// Cold binding; throws std::invalid_argument before publishing an invalid mount.
ImuMountBinding bind_imu_mount(const mjModel&, const bike::spatial::Config&,
                               const bike::spatial::Topology&);
// Sample after the constraint solve, before endpoint forward recomputation.
ImuTruth read_imu_truth(const mjModel&, const mjData&, const ImuMountBinding&,
                       double time_s);
bool is_rotation_matrix(const Matrix3&, double tol = 1e-9) noexcept;

} // namespace runtime::spatial
