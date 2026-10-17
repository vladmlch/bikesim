#include "context.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bike::initialization {
void apply_initial_forces(SetupContext& context,bool advance) {
    require_initializing(context);
    (void)context.physical.apply_forces(false,advance,
        context.physics.initial_front_brake,context.physics.initial_rear_brake,
        std::nullopt,runtime::RideControl{});
}
double acceleration_residual(SetupContext& context) {
    double result=0.;
    for(double a:context.stepper.qacc()) {
        if(!std::isfinite(a))throw std::runtime_error("non-finite physical equilibrium acceleration");
        result=std::max(result,std::abs(a));
    }
    return result;
}
void settle_contacts(SetupContext& context,const std::vector<double>& qpos,
                     const std::vector<double>& qvel) {
    require_initializing(context);
    if(context.stepper.has_rider_contacts())
        (void)context.stepper.mutate([&]{return context.stepper.rider_contacts_settle(
            writers::RiderIntervalState{qpos,qvel},false,nullptr);});
}
} // namespace bike::initialization
