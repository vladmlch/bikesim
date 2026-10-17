#pragma once
#include "allocation.hpp"
#include "constraint_dynamics.hpp"
#include "../configuration/physical_config.hpp"

namespace bike::rider {
inline constexpr double support_force_guard_n=1e-5;
inline constexpr double support_moment_guard_nm=1e-6;
inline constexpr double grip_radius_guard_fraction=1e-6;
inline constexpr double crank_task_scale_nm=1.;

struct AllocationMemory {
    std::optional<std::array<bool,2>> branch;
    std::optional<std::vector<double>> solution;
};
struct BranchResult { Allocation allocation; std::array<bool,2> branch{true,true}; };
using BranchSolver=std::function<Allocation(const std::array<bool,2>&,const std::optional<std::vector<double>>&)>;
[[nodiscard]] BranchResult select_allocation_branch(std::span<const double> target,const BranchSolver&,AllocationMemory&);

struct AllocationAttachment {
    std::string name;
    int equality_id=-1, rider_body=-1, other_body=-1, site_id=-1, geom_id=-1;
    bool weld=false;
    std::array<double,3> anchor_local{};
};
struct ResponseRequest {
    bike::configuration::ArticulatedConfig config;
    std::vector<int> actuator_dofs,rider_dofs,bike_dofs;
    std::vector<double> torque_intent,lower_torque_nm,upper_torque_nm,activation_state;
    std::vector<AllocationAttachment> attachments;
    int pelvis_body=-1,crank_dof=-1;
    double rider_mass_kg=0.,dt_s=0.,effort_scale=1.;
    bool steady_state=false;
    std::optional<double> crank_torque_nm;
};
struct ResponseAllocation {
    Allocation allocation;
    // The source disabled-controller record has no solver coordinates.
    bool disabled=false;
    std::array<bool,2> branch{true,true};
    std::vector<double> torque,excitation,activation_state,acceleration,
        power,wrenches,moments,constraint_forces,lower_torque_nm,upper_torque_nm;
    std::vector<double> rider_acceleration,base_acceleration,qpos,qvel;
    std::optional<double> predicted_crank_torque_nm;
    std::optional<double> crank_torque_target_nm;
    std::size_t modes_tried=0,linear_mode_rejections=0;
    bool mode_search_complete=false;
    double saddle_normal_lower_bound_n=0.;
    double effort_scale=1.,time_s=0.;
};
[[nodiscard]] std::array<double,3> allocation_support_point(const mjModel&,const mjData&,const AllocationAttachment&);
[[nodiscard]] std::vector<ExtraConstraint> allocation_grip_constraints(const Matrix&,
    const std::array<double,2>& direction,bool pulling,double limit_n);
[[nodiscard]] std::pair<Matrix,std::vector<double>> support_rows(const Matrix&,
    const std::array<double,2>& normal,bool saddle,const bike::configuration::ArticulatedConfig&,
    double weight_n,double half_length_m=0.);
[[nodiscard]] ResponseAllocation allocate_response(const mjModel&,const mjData&,
    const ResponseRequest&,AllocationMemory&);
} // namespace bike::rider
