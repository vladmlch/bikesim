#include "context.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bike::initialization {
namespace {
// Launch part 4b (user ruling 2026-11-13): the settle holds crank_spin at
// the prepared pedal angle, the way it holds the prepared lean — spec
// 2026-11-11-rider-launch-plan-design.md:38. A stiff PD restoring torque
// through the step's external generalized-force channel: the constraint
// solver balances it like any applied torque, so the crank reaches
// qacc ~ 0 at the held pose (a kinematic clamp leaves the reaction on the
// pedal hinges and does not converge — experiment Follow-up 8). Setup
// only: require_initializing guards the caller, and the ride's steps pass
// no external force, so nothing holds the crank in the ride.
double crank_hold_torque(SetupContext const& context) {
    const auto crank=joint_address(context,"crank_spin");
    const mjModel* m=context.stepper.model();const mjData* d=context.stepper.data();
    const auto qpos=std::views::counted(d->qpos,m->nq);
    const auto qvel=std::views::counted(d->qvel,m->nv);
    const double seed=context.physics.drive.crank_phase_rad;
    return -context.physics.equilibrium_crank_hold_kp_nm_rad*(qpos[static_cast<std::size_t>(crank.qpos)]-seed)
           -context.physics.equilibrium_crank_hold_kd_nms_rad*qvel[static_cast<std::size_t>(crank.dof)];
}
} // namespace
void apply_initial_forces(SetupContext& context,bool advance) {
    require_initializing(context);
    // Launch part 2: an articulated rider holds the prepared start lean
    // and the prepared pedal angle (launch part 4b) through the settle.
    // A zero lean keeps the empty command the settle
    // always sent, and a lumped or absent rider has no torso to lean — the
    // step rejects any posture for it (control.cpp:40-44).
    runtime::RideControl control{};
    if(context.preparation.lean_rad!=0.&&context.physical.rider_present()){
        auto posture=::rider::RiderPosture{};
        posture.torso_lean_rad=context.preparation.lean_rad;
        control.posture=posture;
    }
    // Launch part 4b: with the feet welded to the pedals (spindle) the same
    // settle also holds the crank at the table angle. Otherwise (no rider, a
    // lumped rider, flat pedals; weld is unmeasured) the crank is only its
    // own light assembly, about 0.013 kg·m², and the explicit hold would
    // ring it — it settles free, as before the hold.
    std::optional<std::vector<double>> external;
    if(context.physical.rider_present()&&context.physics.articulated.pedal_attachment=="spindle"){
        const mjModel* m=context.stepper.model();
        std::vector<double> hold(static_cast<std::size_t>(m->nv),0.);
        const auto crank=joint_address(context,"crank_spin");
        hold[static_cast<std::size_t>(crank.dof)]=crank_hold_torque(context);
        external=std::move(hold);
    }
    (void)context.physical.apply_forces(false,advance,
        context.physics.initial_front_brake,context.physics.initial_rear_brake,
        external,control);
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
