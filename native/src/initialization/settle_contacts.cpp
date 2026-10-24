#include "context.hpp"
#include "bootstrap_hold.hpp"
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
    if(context.bootstrap_hold && context.physical.spatial_state()) {
        auto spatial=*context.physical.spatial_state();
        const auto status=context.bootstrap_hold->try_compute_into(*context.stepper.model(),
            *context.stepper.data(),spatial.bootstrap_phase,
            std::span<double>(context.stepper.data()->qfrc_applied,
                static_cast<std::size_t>(context.stepper.model()->nv)));
        if(status!=CoreStatus::ok)throw std::invalid_argument("bootstrap.hold: invalid force evaluation");
        spatial.hold=context.bootstrap_hold->state();
        context.physical.commit_spatial_state(context.physical.prepare_spatial_state(spatial));
    }
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
