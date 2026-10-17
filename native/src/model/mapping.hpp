#pragma once

#include <mujoco/mujoco.h>
#include <array>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bike::model {

// physical_mapping.py: all points, forces and torques use world coordinates.
// A model/data pair belongs to its owning setup/runtime; matching dimensions
// alone do not establish that ownership.
using WorldVector = std::array<double, 3>;
struct PointJacobian {
    std::vector<double> translation; // row-major (3, nv)
    std::vector<double> rotation;    // row-major (3, nv)
};

[[nodiscard]] int required_id(const mjModel& model, mjtObj kind,
                              std::string_view name);
[[nodiscard]] std::string joint_name_at_qpos(const mjModel& model, int address);
[[nodiscard]] std::pair<int, int> joint_address(const mjModel& model,
                                              std::string_view name);
[[nodiscard]] PointJacobian point_jacobian(const mjModel& model,
    const mjData& data, int body, const WorldVector& point);
void point_jacobian_into(const mjModel& model, const mjData& data, int body,
    const WorldVector& point, std::span<double> translation);
void relative_point_jacobian(const mjModel& model, const mjData& data,
    int body_a, int body_b, const WorldVector& point,
    std::span<double> jacobian_a, std::span<double> jacobian_b);
[[nodiscard]] WorldVector point_velocity(const mjModel& model,
    const mjData& data, int body, const WorldVector& point);
[[nodiscard]] WorldVector body_angular_velocity(const mjModel& model,
    const mjData& data, int body);
[[nodiscard]] std::vector<double> map_wrench(const mjModel& model,
    mjData& data, int body, const WorldVector& point,
    const WorldVector& force, std::optional<WorldVector> torque = std::nullopt);

} // namespace bike::model
