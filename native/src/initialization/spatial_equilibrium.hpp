#pragma once

#include "context.hpp"
#include "../spatial/topology.hpp"
#include "../terrain/source3d.hpp"
#include <string>
#include <vector>

namespace bike::initialization {

struct SpatialEquilibriumReport {
    double max_linear_accel_mps2 = 0.;
    double max_angular_accel_radps2 = 0.;
    double weight_residual_fraction = 0.;
    double pitch_moment_residual_nm = 0.;
    double roll_moment_residual_nm = 0.;
    int contacts_valid = 0;
    int contacts_total = 0;
    int evaluations = 0;
    std::vector<std::string> errors;
    std::vector<int> worst_dofs;
};
double spatial_refine(SetupContext&, int max_evaluations, double acceleration_tolerance);
void spatial_initial_pose(SetupContext&, const bike::spatial::Topology&,
                          const bike::terrain::Terrain3dQueryView&, double heading_rad,
                          double initial_speed_mps);

} // namespace bike::initialization
