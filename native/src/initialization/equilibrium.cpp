#include "context.hpp"
#include "state_file.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"
#include "../writers/suspension.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numeric>
#include <ranges>
#include <stdexcept>

namespace bike::initialization {
namespace {
std::int64_t round_even(double value){
    if(!std::isfinite(value)||value<0.||value>=static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument("equilibrium timestep/budget exceeds supported step count");
    const double base=std::floor(value),part=value-base;
    return static_cast<std::int64_t>(base+((part>.5||(part==.5&&std::fmod(base,2.)!=0.))?1.:0.));
}
std::int64_t timing_steps(double seconds,double dt,std::int64_t cycle){
    const auto count=std::max<std::int64_t>(1,round_even(seconds/(static_cast<double>(cycle)*dt)));
    if(count>std::numeric_limits<std::int64_t>::max()/cycle)throw std::invalid_argument("equilibrium refinement period exceeds step count");
    return count*cycle;
}
std::vector<std::pair<std::string,double>> worst(SetupContext const& context){
    const auto q=context.stepper.qacc();std::vector<std::size_t> order(q.size());std::ranges::iota(order,0);
    std::ranges::stable_sort(order,[&](auto a,auto b){return std::abs(q[a])>std::abs(q[b]);});
    const auto* m=context.stepper.model();const auto joints=std::views::counted(m->dof_jntid,m->nv);
    std::vector<std::pair<std::string,double>> result;
    for(std::size_t i=0;i<std::min<std::size_t>(5,order.size());++i){const auto dof=order[i];const char* name=mj_id2name(m,mjOBJ_JOINT,joints[dof]);result.emplace_back(name?name:"(unnamed)",q[dof]);}return result;
}
std::string failure_message(std::int64_t steps,double residual,const std::vector<std::pair<std::string,double>>& worst){std::string message=std::format("physical equilibrium did not converge after {} steps: residual={:.6g};",steps,residual);for(const auto& [name,a]:worst)message+=std::format(" {}={:.6g}",name,a);return message;}
}
EquilibriumFailure::EquilibriumFailure(std::int64_t count,double residual,std::vector<std::pair<std::string,double>> worst)
    :std::runtime_error(failure_message(count,residual,worst)),steps(count),residual_qacc(residual),worst_accelerations(std::move(worst)){}
EquilibriumFailure::~EquilibriumFailure() = default;
std::int64_t equilibrium_step_budget(double dt,const EquilibriumOptions& options){
    if(!std::isfinite(dt)||dt<=0.||!std::isfinite(options.tolerance)||options.tolerance<=0.)throw std::invalid_argument("equilibrium timestep and tolerance must be finite and positive");
    if(options.max_steps){if(*options.max_steps<=0)throw std::invalid_argument("equilibrium max_steps must be a positive integer");return *options.max_steps;}
    return std::max<std::int64_t>(40000,round_even(80./dt));
}
std::int64_t equilibrium_cycle_steps(double dt){if(!std::isfinite(dt)||dt<=0.)throw std::invalid_argument("equilibrium timestep must be finite and positive");return std::max<std::int64_t>(1,round_even(.02/dt));}

EquilibriumResult settle_context(SetupContext& context,const EquilibriumOptions& options){
    require_initializing(context);
    const double dt=context.stepper.model()->opt.timestep;
    const auto budget=equilibrium_step_budget(dt,options),cycle=equilibrium_cycle_steps(dt);
    const auto first_refine=timing_steps(context.physics.equilibrium_refine_after_s,dt,cycle),refine_period=timing_steps(context.physics.equilibrium_refine_period_s,dt,cycle);
    std::optional<StateFile> cached;
    const auto explicit_file=context.physics.equilibrium_state_file;
    if(explicit_file)cached=load_equilibrium_state(context,*explicit_file);
    else if(context.physics.equilibrium_cache_enabled)cached=load_equilibrium_cache(context);
    context.completed=false;context.equilibrium.reset();initialize_pose(context);
    std::int64_t steps=0;double residual=std::numeric_limits<double>::infinity();bool cache_hit=false;
    if(cached){
        try{
            restore_equilibrium_state(context,*cached);
            steps=state_array(*cached,"steps").integers().front();
            if(explicit_file){apply_initial_forces(context,false);context.stepper.forward();residual=acceleration_residual(context);if(residual>options.tolerance)throw std::invalid_argument(std::format("restored residual {:.6g} exceeds tolerance {}",residual,options.tolerance));}
            else{context.stepper.forward();residual=state_array(*cached,"residual_qacc").doubles().front();}
            cache_hit=true;
        }catch(const engine::EngineFailure&){throw;}catch(const std::bad_alloc&){throw;}
        catch(const std::exception& error){if(explicit_file)throw std::invalid_argument("equilibrium state file "+*explicit_file+": "+error.what());initialize_pose(context);steps=0;}
    }
    while(!cache_hit&&steps<budget){
        const auto count=std::min(cycle,budget-steps);
        for(std::int64_t i=0;i<count;++i){
            apply_initial_forces(context,true);
            const std::vector<double> q(context.stepper.qpos().begin(),context.stepper.qpos().end()),v(context.stepper.qvel().begin(),context.stepper.qvel().end());
            context.stepper.step();settle_contacts(context,q,v);++steps;
            for(auto const state:{context.stepper.qpos(),context.stepper.qvel()})if(!std::ranges::all_of(state,[](double value){return std::isfinite(value);}))throw std::runtime_error("non-finite physical equilibrium state");
        }
        auto const* d=context.stepper.data();std::ranges::fill(std::views::counted(d->qvel,context.stepper.model()->nv),0.);context.stepper.note_posvel_stale();
        apply_initial_forces(context,false);context.stepper.forward();residual=acceleration_residual(context);
        if(residual>options.tolerance&&(steps==first_refine||steps%refine_period==0))
            for(int attempt=0;attempt<8;++attempt){const double previous=residual;residual=refine_equilibrium(context,80,options.tolerance);if(residual<=options.tolerance||residual>=previous*(1.-1e-3))break;}
        if(residual<=options.tolerance)break;
    }
    if(!cache_hit&&residual>options.tolerance)throw EquilibriumFailure(steps,residual,worst(context));
    EquilibriumResult result;result.steps=steps;result.residual_qacc=residual;result.cache_hit=cache_hit;
    const auto q=context.stepper.qpos();result.fork_travel_mm=1000.*q[static_cast<std::size_t>(context.stepper.suspension().fork_qposadr())];result.shock_stroke_mm=1000.*q[static_cast<std::size_t>(context.stepper.suspension().shock_qposadr())];
    if(!context.rear_travel_from_shock_mm)throw std::invalid_argument("equilibrium rear travel requires the source linkage solver");
    result.rear_travel_mm=context.rear_travel_from_shock_mm(result.shock_stroke_mm);
    result.root_z_m=q[static_cast<std::size_t>(joint_address(context,"root_z").qpos)];result.pitch_rad=q[static_cast<std::size_t>(joint_address(context,"root_pitch").qpos)];result.initial_crank_phase_rad=context.physics.drive.crank_phase_rad;result.settled_crank_phase_rad=q[static_cast<std::size_t>(joint_address(context,"crank_spin").qpos)];result.state_file=explicit_file;
    if(context.physics.equilibrium_cache_enabled&&!cache_hit)save_equilibrium_cache(context,capture_state(context,steps,residual));
    context.equilibrium=result;context.completed=true;return result;
}
} // namespace bike::initialization
