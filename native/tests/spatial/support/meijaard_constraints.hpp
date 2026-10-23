#pragma once

#include "../../../src/spatial/topology.hpp"

#include <mujoco/mujoco.h>

#include <vector>

namespace spatial_tests {

struct RollingConstraints {
    std::vector<double> A;
    std::vector<double> Adot_v;
    int rows = 6;
};

RollingConstraints knife_edge_constraints(const mjModel&, const mjData&,
                                          const bike::spatial::Topology&);

} // namespace spatial_tests
