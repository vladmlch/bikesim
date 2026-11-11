#include "context.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bike::initialization {
void apply_initial_forces(SetupContext& context,bool advance) {
    require_initializing(context);
    // Launch part 2: an articulated rider holds the prepared start lean
    // through the settle. A zero lean keeps the empty command the settle
    // always sent, and a lumped or absent rider has no torso to lean — the
    // step rejects any posture for it (control.cpp:40-44).
    runtime::RideControl control{};
    if(context.preparation.lean_rad!=0.&&context.physical.rider_present()){
        auto posture=::rider::RiderPosture{};
        posture.torso_lean_rad=context.preparation.lean_rad;
        control.posture=posture;
    }
    (void)context.physical.apply_forces(false,advance,
        context.physics.initial_front_brake,context.physics.initial_rear_brake,
        std::nullopt,control);
}
double acceleration_residual(SetupContext const& context) {
    double result=0.;
    for(double const a:context.stepper.qacc()) {
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
