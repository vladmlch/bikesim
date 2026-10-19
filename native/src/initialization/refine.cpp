#include "context.hpp"
#include "../engine_call.hpp"
#include "../numerics/least_squares.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/rider_contacts.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <ranges>
#include <stdexcept>

namespace bike::initialization {
double refine_equilibrium(SetupContext& context,int max_evaluations,double acceleration_tolerance) {
    require_initializing(context);
    if(max_evaluations<=0||!std::isfinite(acceleration_tolerance)||acceleration_tolerance<=0.)
        throw std::invalid_argument("invalid equilibrium refinement budget or tolerance");
    auto& stepper=context.stepper;
    mjModel* m=stepper.model();mjData* d=stepper.data();
    if(m->nq!=m->nv)throw std::invalid_argument("physical equilibrium requires scalar planar coordinates");
    const auto n=static_cast<std::size_t>(m->nv);
    const std::vector<double> q0(stepper.qpos().begin(),stepper.qpos().end());
    const auto root_x=static_cast<std::size_t>(joint_address(context,"root_x").qpos);
    // NOLINTNEXTLINE(modernize-make-unique) make_unique's inlined forwarding path overflows the 8192-byte frame budget here
    const auto incoming_drive=std::unique_ptr<drivetrain::DriveSnapshot>{new drivetrain::DriveSnapshot(stepper.drive().state())};
    // NOLINTNEXTLINE(modernize-make-unique) same frame-budget constraint
    const auto incoming_step=std::unique_ptr<runtime::StepState>{new runtime::StepState(context.physical.state())};
    const auto tire_names=stepper.has_tire()?stepper.tire_state_names():std::vector<std::string>{};
    const auto tire_state=stepper.has_tire()?stepper.tire_state():std::vector<double>{};
    std::unique_ptr<writers::RiderContactsState> rider_state;
    std::optional<writers::RiderContactsProbe> rider_probe;
    if(stepper.has_rider_contacts()){rider_state=std::make_unique<writers::RiderContactsState>(stepper.rider_contacts().state());rider_probe=stepper.rider_contacts().probe();}
    std::vector<std::size_t> qids,tire_ids;
    std::vector<double> x0,scales;
    const auto dof_jnt=std::views::counted(m->dof_jntid,m->nv);
    const auto joint_type=std::views::counted(m->jnt_type,m->njnt);
    for(std::size_t i=0;i<n;++i)if(i!=root_x){
        qids.push_back(i);x0.push_back(q0[i]);
        scales.push_back(joint_type[static_cast<std::size_t>(dof_jnt[i])]==mjJNT_SLIDE?.05:.2);
    }
    if(stepper.has_tire()&&!stepper.tire_writer().is_pneumatic())
        for(std::size_t i=0;i<tire_names.size();++i)
            if(tire_names[i].ends_with(".xi")&&std::isfinite(tire_state[i])){
                tire_ids.push_back(i);x0.push_back(tire_state[i]);scales.push_back(.005);
            }
    if(rider_state){
        for(const auto& support:rider_state->supports){x0.push_back(support.xi);scales.push_back(.005);}
        for(const auto& hand:rider_state->grip_xi_local)for(std::size_t const axis:{0U,2U}){x0.push_back(hand[axis]);scales.push_back(.005);}
    }
    std::vector<double> force_scale(n),basis(n,0.),column(n);
    for(std::size_t i=0;i<n;++i){
        basis[i]=1.;stepper.mutate([&]{engine::mul_m(m,d,column.data(),basis.data());});
        force_scale[i]=std::sqrt(std::max(column[i],1e-12));basis[i]=0.;
    }
    const bool polish=acceleration_residual(context)<1.;
    std::pair<int,double> best_rank{1,std::numeric_limits<double>::infinity()};
    std::vector<double> best=x0;
    auto place=[&](std::span<const double> x){
        // Every unordered candidate starts from the incoming material and
        // freehub datum. restore() stages its model writes; the ordinary
        // shared drive.prepare then linearizes at this trial's pose. No
        // ratchet state from an earlier rejected point reaches this trial.
        auto next_rider=rider_state?std::make_unique<writers::RiderContactsState>(*rider_state):nullptr;
        std::vector<double> next_tire=tire_state;
        std::size_t offset=qids.size();
        for(std::size_t const id:tire_ids)next_tire[id]=x[offset++];
        if(next_rider){
            for(auto& support:next_rider->supports)support.xi=x[offset++];
            for(auto& hand:next_rider->grip_xi_local)for(std::size_t const axis:{0U,2U})hand[axis]=x[offset++];
        }
        context.physical.restore(*incoming_step);
        stepper.mutate([&]{stepper.drive().restore(*incoming_drive);});
        if(stepper.has_tire())stepper.set_tire_state(tire_names,next_tire);
        if(next_rider){auto probe=rider_probe;stepper.rider_contacts().assign_state(std::move(*next_rider),std::move(probe));}
        auto const q=std::views::counted(d->qpos,m->nq);
        std::ranges::copy(q0,q.begin());
        for(std::size_t i=0;i<qids.size();++i)q[qids[i]]=x[i];
        std::ranges::fill(std::views::counted(d->qvel,m->nv),0.);
        stepper.note_posvel_stale();
    };
    auto const residual=[&](std::span<const double> y){
        std::vector<double> x(x0.size());
        for(std::size_t i=0;i<x.size();++i)x[i]=x0[i]+scales[i]*y[i];
        place(x);apply_initial_forces(context,false);stepper.forward();
        std::vector<double> result(n+x.size());
        if(polish)std::ranges::copy(stepper.qacc(),result.begin());
        else {
            stepper.mutate([&]{engine::mul_m(m,d,result.data(),d->qacc);});
            for(std::size_t i=0;i<n;++i)result[i]/=force_scale[i];
        }
        for(std::size_t i=0;i<x.size();++i)result[n+i]=(polish?1e-7:1e-5)*y[i];
        const double acceleration=acceleration_residual(context);
        double squared=0.;for(double const value:result)squared+=value*value;
        const std::pair<int,double> rank{acceleration>acceleration_tolerance?1:0,squared};
        if(std::isfinite(squared)&&rank<best_rank){best_rank=rank;best=std::move(x);}
        return result;
    };
    numerics::LeastSquaresOptions options;
    options.max_evaluations=max_evaluations;options.difference=numerics::DifferenceScheme::three_point;
    options.ftol=options.xtol=options.gtol=1e-10;
    options.stop=[&](std::span<const double>){return best_rank.first==0;};
    std::exception_ptr error;
    try {
        (void)numerics::bounded_least_squares(residual,std::vector<double>(x0.size(),0.),
            std::vector<double>(x0.size(),-1.),std::vector<double>(x0.size(),1.),options);
    } catch(const engine::EngineFailure&) { throw; }
      catch(...) { error=std::current_exception(); }
    // The best physically evaluated candidate wins, including candidates
    // discovered during differences; an optimizer status is not acceptance.
    place(best);apply_initial_forces(context,false);stepper.forward();
    if(error)std::rethrow_exception(error);
    return acceleration_residual(context);
}
} // namespace bike::initialization
