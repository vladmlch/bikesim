#pragma once
#include "allocation.hpp"
#include <mujoco/mujoco.h>
#include <functional>

namespace bike::rider {
using ConstraintMode=std::vector<std::pair<std::size_t,double>>;
struct ConstraintResponse {
    std::vector<double> acceleration_offset;
    Matrix acceleration_matrix;
    std::vector<double> force_offset;
    Matrix force_matrix;
    Matrix domain_matrix;
    std::vector<double> domain_bound;
    ConstraintMode mode;
};
struct ConstraintSystem {
    Matrix mass, jacobian, actuation;
    std::vector<double> known_force, resistance, reference_acceleration, frictionloss;
    std::vector<int> types, ids;
};

class ConstraintDynamics {
public:
    explicit ConstraintDynamics(ConstraintSystem system);
    // Internal owner boundary: model and data must belong to the same
    // Stepper/controller. The numerical object immediately owns a snapshot;
    // no pointer is retained and solved reactions/qacc are not inputs.
    ConstraintDynamics(const mjModel& model,const mjData& data,std::span<const int> actuator_dofs);
    [[nodiscard]] ConstraintResponse linearize(std::span<const double> torque) const;
    [[nodiscard]] ConstraintResponse response_for_mode(const ConstraintMode&) const;
    [[nodiscard]] std::vector<ConstraintMode> neighboring_modes(const ConstraintMode&) const;
    Matrix jacobian, smooth_matrix, inverse_jacobian, matrix, rhs_matrix;
    std::vector<double> smooth_offset, rhs_offset, lower, upper;
    std::vector<int> types,ids;
};

[[nodiscard]] Matrix attachment_force_map(const mjModel&,const mjData&,
    const ConstraintDynamics&,int equality_id,int body,const std::array<double,3>& point);

struct ModeCandidate { Allocation allocation; std::size_t payload_index=0; };
struct ModeSearchResult {
    bool complete=false;
    std::size_t modes_tried=0;
    Allocation allocation;
    ConstraintResponse response;
    std::size_t payload_index=0;
};
using ModeSolver=std::function<ModeCandidate(const ConstraintResponse&)>;
using TorqueOf=std::function<std::vector<double>(const Allocation&)>;
[[nodiscard]] ModeSearchResult search_constraint_modes(const ConstraintDynamics&,
    std::span<const double> wish,const ModeSolver&,const TorqueOf& torque_of={},
    std::size_t max_candidates=256);
} // namespace bike::rider
