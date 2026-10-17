#include "playground.hpp"
#include "../engine_call.hpp"
#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>
namespace bike::legacy {
initialization::StaticEquilibriumResult solve_static_equilibrium(initialization::StaticEquilibriumContext& context,const initialization::StaticEquilibriumOptions& options){
    return initialization::solve_static_equilibrium(context,options);
}
void apply_stand_targets(const mjModel& m,mjData& d,const kinematics::HorstLinkageSolver& solver,int rear,int fork,double rear_mm,double fork_mm){
    if(!std::isfinite(rear_mm)||!std::isfinite(fork_mm))throw std::invalid_argument("stand travel targets must be finite");
    if(rear>=m.nu||fork>=m.nu)throw std::invalid_argument("stand actuator outside model");
    // Compute both controls before committing. The source fork limit is
    // explicitly 180 mm and is independent of custom BikeSpecs travel.
    const double fork_target=std::clamp(fork_mm/1000.,0.,.180);
    const double rear_target=rear>=0?-(solver.solve_state_from_wheel_travel(rear_mm).theta_cs-solver.theta_cs_0()):0.;
    auto controls=std::views::counted(d.ctrl,m.nu);if(fork>=0)controls[static_cast<std::size_t>(fork)]=fork_target;if(rear>=0)controls[static_cast<std::size_t>(rear)]=rear_target;
}
void reset_stand(const mjModel& m,mjData& d,const kinematics::HorstLinkageSolver& solver,int rear,int fork){
    engine::reset_data(&m,&d);apply_stand_targets(m,d,solver,rear,fork,0.,0.);engine::forward(&m,&d);
}
} // namespace bike::legacy
