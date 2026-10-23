#pragma once

#include "config.hpp"
#include "types.hpp"
#include <mujoco/mujoco.h>

#include <array>
#include <string>
#include <vector>

namespace bike::spatial {

struct ScalarJointBinding { int joint_id = -1; int qpos_adr = -1; int dof_adr = -1; };
struct RootBinding {
    int body_id = -1;
    RootKind kind = RootKind::planar3;
    int free_joint_id = -1;
    int free_qpos_adr = -1;
    int free_dof_adr = -1;
    // x,y,z,pitch; planar3 leaves y at the invalid sentinel.
    std::array<ScalarJointBinding, 4> scalar{};
};
struct WheelBinding {
    int wheel_body_id = -1;
    int hub_body_id = -1;
    ScalarJointBinding spin;
    Vec3 axle_axis_hub{0., 1., 0.};
};
struct SteerBinding { ScalarJointBinding joint; int actuator_id = -1; Vec3 axis_body{}; };
struct Topology {
    RootBinding root;
    std::vector<RootBinding> rider_roots;
    std::array<WheelBinding, 2> wheels; // front, rear
    SteerBinding steer;
    int imu_site_id = -1;
    int nq = 0;
    int nv = 0;
    std::string model_sha256_hex;
    std::string fingerprint_hex;
};

[[nodiscard]] Topology bind_topology(const mjModel&, const Config&);
[[nodiscard]] std::string topology_fingerprint_hex(const Topology&);
// Model/data/binding identity and a current kinematic stage are preconditions.
[[nodiscard]] BodyPose body_pose(const mjModel&, const mjData&, const RootBinding&) noexcept;

} // namespace bike::spatial
