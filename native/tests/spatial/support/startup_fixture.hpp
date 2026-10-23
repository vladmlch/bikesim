#pragma once

#include "../../../src/spatial/config.hpp"
#include "../../../src/spatial/topology.hpp"

#include <mujoco/mujoco.h>

#include <cstddef>
#include <vector>

namespace spatial_tests {

struct RigModel {
    mjModel* model = nullptr;
    mjData* data = nullptr;
    bike::spatial::Topology topology;
    RigModel() = default;
    ~RigModel();
    RigModel(const RigModel&) = delete;
    RigModel& operator=(const RigModel&) = delete;
    RigModel(RigModel&&) noexcept;
    RigModel& operator=(RigModel&&) noexcept;
};

RigModel constrained_rig(const bike::spatial::Config&);
RigModel free_rig(const bike::spatial::Config&, bool articulated_rider);
RigModel mixed_joint_rig();
std::vector<std::byte> complete_state_bytes(const RigModel&);

} // namespace spatial_tests
