#include "context.hpp"
#include "state_file.hpp"
#include "../cblas_abi.hpp"
#include "../engine_call.hpp"
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
// Kinetic damping bounds: the minimum segment keeps high-frequency ripple
// from resetting every few steps, the drop margin ignores ripple on the main
// peak, the maximum segment bounds a mode that never peaks (free rolling).
constexpr double kKineticMinSegmentS=.002,kKineticMaxSegmentS=.25,kKineticPeakDrop=.02;
// Steps between two velocity resets. "cycle" takes the fixed clock; "kinetic"
// stops once the kinetic energy has dropped clearly below its running peak.
struct Segments{
    bool kinetic=false;std::int64_t cycle=1,min_segment=1,max_segment=1;std::vector<double> momentum;
    Segments(SetupContext const& context,double dt,std::int64_t cycle_steps,bool kinetic_schedule) // NOLINT(bugprone-easily-swappable-parameters) dt [s] and cycle_steps [steps] differ in unit; the single call site names both
        :kinetic(kinetic_schedule),cycle(cycle_steps){
        if(!kinetic)return;
        min_segment=std::max<std::int64_t>(1,round_even(kKineticMinSegmentS/dt));
        max_segment=std::max(min_segment,round_even(kKineticMaxSegmentS/dt));
        const auto nv=context.stepper.model()->nv;
        if(nv<0||nv>std::numeric_limits<int>::max())throw std::invalid_argument("kinetic damping: nv exceeds the BLAS int range");
        momentum.assign(static_cast<std::size_t>(nv),0.);
    }
    template<typename Step>std::int64_t run(SetupContext& context,Step& step,std::int64_t budget){
        if(!kinetic){const auto count=std::min(cycle,budget);for(std::int64_t i=0;i<count;++i)step();return count;}
        // qM after mj_step belongs to the step's start pose: one step stale,
        // which only shifts the detected peak by one step.
        double peak=0.;std::int64_t taken=0;
        while(taken<budget&&taken<max_segment){
            step();++taken;
            const mjModel* m=context.stepper.model();const mjData* d=context.stepper.data();
            context.stepper.mutate([&]{engine::mul_m(m,d,momentum.data(),d->qvel);});
            // numpy's qvel@momentum is the provider ddot with its 0.+ seed.
            const double energy=.5*blas::ddot(static_cast<int>(momentum.size()),d->qvel,1,momentum.data(),1);
            if(energy>peak)peak=energy;
            else if(taken>=min_segment&&energy<(1.-kKineticPeakDrop)*peak)break;
        }
        return taken;
    }
};
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
    // Bind the options this solve actually runs with into the context before
    // any cache read/write: the equilibrium fingerprint pins them, so a pose
    // settled under a different contract can never be served under this key.
    context.equilibrium_options=options;
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
    const auto relax_step=[&]{
        apply_initial_forces(context,true);
        const std::vector<double> q(context.stepper.qpos().begin(),context.stepper.qpos().end()),v(context.stepper.qvel().begin(),context.stepper.qvel().end());
        context.stepper.step();settle_contacts(context,q,v);
        for(auto const state:{context.stepper.qpos(),context.stepper.qvel()})if(!std::ranges::all_of(state,[](double value){return std::isfinite(value);}))throw std::runtime_error("non-finite physical equilibrium state");
    };
    // physics.equilibrium_relaxation selects the schedule.
    const bool kinetic=context.physics.equilibrium_relaxation=="kinetic";
    Segments segments(context,dt,cycle,kinetic);
    const auto root_x=static_cast<std::size_t>(joint_address(context,"root_x").qpos);
    const double start_x=context.stepper.qpos()[root_x];
    std::int64_t next_refine=first_refine,resets=0;
    while(!cache_hit&&steps<budget){
        steps+=segments.run(context,relax_step,budget-steps);++resets;
        auto const* d=context.stepper.data();std::ranges::fill(std::views::counted(d->qvel,context.stepper.model()->nv),0.);context.stepper.note_posvel_stale();
        apply_initial_forces(context,false);context.stepper.forward();residual=acceleration_residual(context);
        // Kinetic segment lengths vary, so its refine schedule is a time
        // threshold rather than an exact step multiple.
        bool refine_due=false;
        if(kinetic){refine_due=steps>=next_refine;if(refine_due)next_refine=steps+refine_period;}
        else refine_due=steps==first_refine||steps%refine_period==0;
        if(residual>options.tolerance&&refine_due)
            for(int attempt=0;attempt<8;++attempt){const double previous=residual;residual=refine_equilibrium(context,80,options.tolerance);if(residual<=options.tolerance||residual>=previous*(1.-1e-3))break;}
        if(residual<=options.tolerance)break;
    }
    if(!cache_hit&&residual>options.tolerance)throw EquilibriumFailure(steps,residual,worst(context));
    EquilibriumResult result;result.steps=steps;result.residual_qacc=residual;result.cache_hit=cache_hit;
    const auto q=context.stepper.qpos();result.fork_travel_mm=1000.*q[static_cast<std::size_t>(context.stepper.suspension().fork_qposadr())];result.shock_stroke_mm=1000.*q[static_cast<std::size_t>(context.stepper.suspension().shock_qposadr())];
    if(!context.rear_travel_from_shock_mm)throw std::invalid_argument("equilibrium rear travel requires the source linkage solver");
    result.rear_travel_mm=context.rear_travel_from_shock_mm(result.shock_stroke_mm);
    result.root_z_m=q[static_cast<std::size_t>(joint_address(context,"root_z").qpos)];result.pitch_rad=q[static_cast<std::size_t>(joint_address(context,"root_pitch").qpos)];result.initial_crank_phase_rad=context.physics.drive.crank_phase_rad;result.settled_crank_phase_rad=q[static_cast<std::size_t>(joint_address(context,"crank_spin").qpos)];result.state_file=explicit_file;
    if(kinetic&&!cache_hit){result.relaxation="kinetic";result.relaxation_resets=resets;result.settle_x_drift_m=q[root_x]-start_x;}
    if(context.physics.equilibrium_cache_enabled&&!cache_hit)save_equilibrium_cache(context,capture_state(context,steps,residual));
    context.equilibrium=result;context.completed=true;return result;
}
} // namespace bike::initialization
