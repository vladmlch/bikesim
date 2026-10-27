#include "context.hpp"
#include "bootstrap_hold.hpp"
#include "spatial_equilibrium.hpp"
#include "state_file.hpp"
#include "../runtime/step.hpp"
#include "../runtime/accounting.hpp"
#include "../numerics/dense.hpp"
#include "../stepper.hpp"
#include "../writers/suspension.hpp"
#include "../writers/tire.hpp"
#include "../spatial/frames.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <memory>
#include <new>
#include <numeric>
#include <optional>
#include <ranges>
#include <span>
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
    // A NaN compares false both ways, which is not a strict weak ordering.
    // Rank non-finite accelerations as the worst offenders (same rule as the
    // spatial report's worst_dofs) so the sort is well defined and they lead.
    const auto severity=[&](std::size_t dof){const double a=std::abs(q[dof]);return std::isfinite(a)?a:std::numeric_limits<double>::infinity();};
    std::ranges::stable_sort(order,[&](auto a,auto b){return severity(a)>severity(b);});
    const auto* m=context.stepper.model();const auto joints=std::views::counted(m->dof_jntid,m->nv);
    std::vector<std::pair<std::string,double>> result;
    for(std::size_t i=0;i<std::min<std::size_t>(5,order.size());++i){const auto dof=order[i];const char* name=mj_id2name(m,mjOBJ_JOINT,joints[dof]);result.emplace_back(name?name:"(unnamed)",q[dof]);}return result;
}
std::string failure_message(std::int64_t steps,double residual,const std::vector<std::pair<std::string,double>>& worst){std::string message=std::format("physical equilibrium did not converge after {} steps: residual={:.6g};",steps,residual);for(const auto& [name,a]:worst)message+=std::format(" {}={:.6g}",name,a);return message;}

// _released_probe — the acceleration a wheel takes from rest under the
// released-brake field: non-advancing, zeroed velocity, active=True so the
// rider path evaluates the same field the first live interval applies.
std::array<double,2> released_probe(SetupContext& context,
    const std::array<int,2>& wheel_dofs) {
    const auto* d=context.stepper.data();
    std::ranges::fill(std::views::counted(d->qvel,context.stepper.model()->nv),0.);
    context.stepper.note_posvel_stale();
    (void)context.physical.apply_forces(true,false,0.,0.,std::nullopt,runtime::RideControl{});
    context.stepper.forward();
    const auto q=context.stepper.qacc();
    return {q[static_cast<std::size_t>(wheel_dofs[0])],q[static_cast<std::size_t>(wheel_dofs[1])]};
}
// np.max(np.abs(data.qacc)) — a NaN lane propagates instead of throwing,
// matching the oracle's probe (which degrades to the kept-held-pose path).
double released_residual(SetupContext const& context) {
    double result=0.;
    for(double const a:context.stepper.qacc()) {
        const double m=std::abs(a);
        if(std::isnan(m))return std::numeric_limits<double>::quiet_NaN();
        result=std::max(result,m);
    }
    return result;
}
// _solve_brush_shear — Newton-adjust the tangential brush deflection to the
// released wheel statics. The wheel free body is essentially decoupled in
// the two spin DOFs, so a secant solve on the released accelerations reaches
// the same state an infinitely patient tread creep would.
void solve_brush_shear(SetupContext& context,const std::array<int,2>& wheel_dofs) {
    if(!context.stepper.has_tire() || !context.stepper.tire_writer().is_compliant())return;
    auto& tire=context.stepper.tire_writer().compliant_writer();
    const auto set_xi=[&](double front,double rear){tire.set_brush_xi(0,front);tire.set_brush_xi(1,rear);};
    std::array<double,2> xi{tire.brush_xi(0),tire.brush_xi(1)};
    auto residual=released_probe(context,wheel_dofs);
    constexpr double atol=5e-3,eps=2e-5;
    for(int round=0;round<4;++round) {
        const double m0=std::abs(residual[0]),m1=std::abs(residual[1]);
        const double margin=(std::isnan(m0)||std::isnan(m1))?std::numeric_limits<double>::quiet_NaN():std::max(m0,m1);
        if(margin<=atol)break;
        std::array<std::array<double,2>,2> jacobian{};
        for(std::size_t j=0;j<2;++j) {
            auto trial=xi;trial[j]+=eps;
            set_xi(trial[0],trial[1]);
            const auto probe=released_probe(context,wheel_dofs);
            jacobian[0][j]=(probe[0]-residual[0])/eps;
            jacobian[1][j]=(probe[1]-residual[1])/eps;
        }
        set_xi(xi[0],xi[1]);
        // np.linalg.det reads the partial-pivot LU as sign·exp(Σ log|u_ii|)
        // (slogdet): a zero pivot is det 0 and the oracle breaks; a NaN lane
        // makes det NaN — the <1e-9 gate is then false and flow continues
        // into the guarded solve, exactly like numpy.
        double a00=jacobian[0][0],a01=jacobian[0][1],a10=jacobian[1][0],a11=jacobian[1][1];
        double det_sign=1.;
        if(std::abs(a10)>std::abs(a00)){std::swap(a00,a10);std::swap(a01,a11);det_sign=-1.;}
        double det=0.;
        if(a00!=0.){
            const double u11=a11-(a10/a00)*a01;
            det = u11==0. ? 0. : det_sign*std::exp(std::log(std::abs(a00))+std::log(std::abs(u11)));
        }
        if(std::abs(det)<1e-9)break;
        const std::array<double,2> neg{-residual[0],-residual[1]};
        std::array<double,2> delta{};
        try {
            const auto solved=numerics::dense::solve(
                numerics::dense::Matrix{.rows=2,.cols=2,.values={jacobian[0][0],jacobian[0][1],jacobian[1][0],jacobian[1][1]}},
                std::span<const double>{neg});
            delta={solved[0],solved[1]};
        } catch(const std::runtime_error&) {
            // det≠0 means LAPACK cannot raise the singular error — a throw
            // here is a nonfinite in/out, which numpy returns instead of
            // raising; the oracle's isfinite(xi) check then leaves the
            // shear unchanged.
            return;
        }
        xi[0]+=delta[0];xi[1]+=delta[1];
        if(!std::isfinite(xi[0])||!std::isfinite(xi[1]))return;
        set_xi(xi[0],xi[1]);
        residual=released_probe(context,wheel_dofs);
    }
}
// _relax_released — damped relaxation under the released-brake demand:
// alternating shear solve and damped cycles let the chassis, weld
// deflections and the coast servo follow the wheel statics.
std::pair<double,std::int64_t> relax_released(SetupContext& context,
    std::int64_t max_steps,std::int64_t cycle,double tolerance) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors _relax_released
    const std::array<int,2> wheel_dofs{
        joint_address(context,"front_wheel_spin").dof,
        joint_address(context,"rear_wheel_spin").dof};
    const auto probe=[&]{
        (void)released_probe(context,wheel_dofs);
        return released_residual(context);};
    std::int64_t steps=0;
    double residual=std::numeric_limits<double>::infinity();
    for(int round=0;round<3;++round) {
        if(steps>=max_steps)break;
        solve_brush_shear(context,wheel_dofs);
        while(steps<max_steps) {
            const auto count=std::min(cycle,max_steps-steps);
            for(std::int64_t i=0;i<count;++i) {
                (void)context.physical.apply_forces(true,true,0.,0.,std::nullopt,runtime::RideControl{});
                const std::vector<double> q(context.stepper.qpos().begin(),context.stepper.qpos().end()),
                    v(context.stepper.qvel().begin(),context.stepper.qvel().end());
                context.stepper.step();settle_contacts(context,q,v);
                // The live interval closes with settle_actuation: the
                // bookkeeping reads the just-solved tendon/actuator state
                // and clears the pending interval the next advancing apply
                // validates against.
                context.stepper.mutate([&]{(void)context.stepper.drive().settle();});
                ++steps;
                for(auto const state:{context.stepper.qpos(),context.stepper.qvel()})
                    if(!std::ranges::all_of(state,[](double value){return std::isfinite(value);}))
                        throw std::runtime_error("non-finite physical equilibrium state");
            }
            auto const* d=context.stepper.data();
            std::ranges::fill(std::views::counted(d->qvel,context.stepper.model()->nv),0.);
            context.stepper.note_posvel_stale();
            residual=probe();
            if(residual<=tolerance)break;
        }
        if(residual<=tolerance)break;
    }
    return {residual,steps};
}
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
    auto budget=equilibrium_step_budget(dt,options);
    const auto cycle=equilibrium_cycle_steps(dt);
    if(context.physics.spatial && context.topology && context.topology->root.kind==spatial::RootKind::free6)
        budget=std::min(budget,std::max<std::int64_t>(1,round_even(context.physics.spatial->bootstrap.settle_time_s/dt)));
    const auto first_refine=timing_steps(context.physics.equilibrium_refine_after_s,dt,cycle),refine_period=timing_steps(context.physics.equilibrium_refine_period_s,dt,cycle);
    std::optional<StateFile> cached;
    const auto explicit_file=context.physics.equilibrium_state_file;
    if(explicit_file)cached=load_equilibrium_state(context,*explicit_file);
    else if(context.physics.equilibrium_cache_enabled)cached=load_equilibrium_cache(context);
    context.completed=false;context.equilibrium.reset();context.spatial_report.reset();initialize_pose(context);
    std::int64_t steps=0;double residual=std::numeric_limits<double>::infinity();bool cache_hit=false;
    if(cached){
        // A decoded cache still needs a physical probe. Preserve the fresh
        // material, model rows and causal owners so a rejected probe can take
        // exactly the fresh path. The private snapshot's scalar metadata is
        // unused: it is a rollback inventory, not an equilibrium result.
        std::optional<StateFile> fresh_material;
        std::unique_ptr<runtime::StepState> fresh_step;
        if(context.topology && !explicit_file) {
            fresh_material=capture_state(context,1,0.);
            // NOLINTNEXTLINE(modernize-make-unique,cppcoreguidelines-owning-memory) elide the large causal snapshot into heap storage
            fresh_step.reset(new runtime::StepState(context.physical.state()));
        }
        try{
            restore_equilibrium_state(context,*cached);
            steps=state_array(*cached,"steps").integers().front();
            if(context.topology) {
                residual=spatial_refine(context,1,options.tolerance);
                if(residual>options.tolerance || !context.spatial_report->errors.empty())
                    throw std::invalid_argument("restored spatial equilibrium failed its physical residual/contact check");
                if(context.bootstrap_hold && !context.bootstrap_hold->release_check(*context.spatial_report,
                    context.physics.spatial->bootstrap.release_force_limit_n, // NOLINT(bugprone-unchecked-optional-access) topology engagement implies physics.spatial by setup contract
                    context.physics.spatial->bootstrap.release_moment_limit_nm, // NOLINT(bugprone-unchecked-optional-access) topology engagement implies physics.spatial by setup contract
                    context.equilibrium_tolerance))
                    throw std::invalid_argument("restored spatial equilibrium requires bootstrap support");
            }
            else if(explicit_file){apply_initial_forces(context,false);context.stepper.forward();residual=acceleration_residual(context);if(residual>options.tolerance)throw std::invalid_argument(std::format("restored residual {:.6g} exceeds tolerance {}",residual,options.tolerance));}
            else{context.stepper.forward();residual=state_array(*cached,"residual_qacc").doubles().front();}
            cache_hit=true;
        }catch(const engine::EngineFailure&){throw;}catch(const std::bad_alloc&){throw;}
        catch(const std::exception& error){
            if(explicit_file)throw std::invalid_argument("equilibrium state file "+*explicit_file+": "+error.what());
            context.cache_rejection_reason=error.what();
            if(fresh_material) {
                restore_equilibrium_state(context,*fresh_material);
                context.physical.restore(*fresh_step);
            }
            context.spatial_report.reset();initialize_pose(context);
            steps=0;residual=std::numeric_limits<double>::infinity();
        }
    }
    // The free-root candidate is evaluated on its tangent space before the
    // dissipative fallback. Its material probes never become accepted steps.
    if(!cache_hit && context.topology && context.topology->root.kind==spatial::RootKind::free6) {
        residual=spatial_refine(context,80,options.tolerance);
        if(residual<=options.tolerance)steps=1;
    }
    // The latched elastic bite is a ride-time latch, not an equilibrium
    // force: its engage/release hysteresis ratchets the wheels while the
    // damped cycles run and the standstill residual never lands. Solve the
    // pose on the friction bound alone (static_braking.py::hold_suspended).
    auto const hold_suspension=context.physical.brake().hold_suspend();
    while(!cache_hit&&residual>options.tolerance&&steps<budget){
        const auto count=std::min(cycle,budget-steps);
        for(std::int64_t i=0;i<count;++i){
            apply_initial_forces(context,true);
            const std::vector<double> q(context.stepper.qpos().begin(),context.stepper.qpos().end()),v(context.stepper.qvel().begin(),context.stepper.qvel().end());
            const auto hold_pose=context.bootstrap_hold?std::optional{spatial::body_pose(*context.stepper.model(),*context.stepper.data(),context.topology->root)}:std::nullopt; // NOLINT(bugprone-unchecked-optional-access) bootstrap_hold implies a spatial topology by setup contract
            context.stepper.step();settle_contacts(context,q,v);++steps;
            if(hold_pose) {
                auto hold=context.bootstrap_hold->state();
                if(!runtime::account_bootstrap_hold(hold,*hold_pose,hold.phase,dt))
                    throw std::invalid_argument("bootstrap.hold.work_j: invalid accepted power");
                if(context.bootstrap_hold->restore(hold)!=CoreStatus::ok)throw std::invalid_argument("bootstrap.hold.work_j: nonfinite accepted work");
            }
            for(auto const state:{context.stepper.qpos(),context.stepper.qvel()})if(!std::ranges::all_of(state,[](double value){return std::isfinite(value);}))throw std::runtime_error("non-finite physical equilibrium state");
        }
        auto const* d=context.stepper.data();std::ranges::fill(std::views::counted(d->qvel,context.stepper.model()->nv),0.);context.stepper.note_posvel_stale();
        apply_initial_forces(context,false);context.stepper.forward();residual=acceleration_residual(context);
        if(residual>options.tolerance&&(steps==first_refine||steps%refine_period==0))
            for(int attempt=0;attempt<8;++attempt){const double previous=residual;residual=refine_equilibrium(context,80,options.tolerance);if(residual<=options.tolerance||residual>=previous*(1.-1e-3))break;}
        if(residual<=options.tolerance)break;
    }
    if(context.topology && (!cache_hit || !context.spatial_report)) {
        // Refresh separate physical-unit residuals for either root family and
        // for cache hits. A one-evaluation pass still probes the actual pose.
        residual=spatial_refine(context,cache_hit?1:80,options.tolerance);
    }
    // Cache hits were judged where they were restored: spatial and explicit-file
    // hits are re-probed and thrown inside the cache try above. A planar cache
    // hit keeps its stored residual (oracle physical_equilibrium.py parity);
    // only a freshly solved pose is judged here.
    if(!cache_hit && residual>options.tolerance) {
        if(context.bootstrap_hold && steps>=budget) {
            const auto& h=context.bootstrap_hold->state();
            throw std::invalid_argument(std::format("bootstrap: hold not releasable: residual force {} N, moment {} N m, acceleration {} after {} steps",
                std::hypot(h.last_force_world_n[0],h.last_force_world_n[1],h.last_force_world_n[2]),
                std::hypot(h.last_couple_world_nm[0],h.last_couple_world_nm[1],h.last_couple_world_nm[2]),residual,steps));
        }
        throw EquilibriumFailure(steps,residual,worst(context));
    }
    // Release consistency (physical_equilibrium.py:142-176). The braked
    // relaxation converges with the wheel brakes holding whatever tangential
    // load the pose left behind; the episode starts unbraked, so that store
    // releases across the first intervals as constraint work the energy
    // ledger rightfully rejects. Re-settle the pose under the released
    // demand: wheel brushes adjust to wheel statics first, then damped
    // cycles let the chassis and weld deflections follow. When no released
    // rest exists the held result is kept unchanged.
    if(!cache_hit && residual<=options.tolerance && steps<budget && !context.topology
        && (context.physics.initial_front_brake>0. || context.physics.initial_rear_brake>0.)) {
        const std::vector<double> held_pose(context.stepper.qpos().begin(),context.stepper.qpos().end());
        const auto held_state=capture_state(context,steps,residual);
        const auto held_steps=steps;
        const double release_tolerance=std::min(options.tolerance,.01);
        const auto released_budget=std::min(budget-steps,round_even(15./dt));
        const auto [released,used]=relax_released(context,released_budget,cycle,release_tolerance);
        steps+=used;
        if(released<=release_tolerance)residual=released;
        else {
            // No released-brake rest exists under this track/config: keep
            // the brake-assisted equilibrium rather than the drifted state.
            restore_equilibrium_material(context,held_state);
            auto const* d=context.stepper.data();
            std::ranges::copy(held_pose,std::views::counted(d->qpos,context.stepper.model()->nq).begin());
            std::ranges::fill(std::views::counted(d->qvel,context.stepper.model()->nv),0.);
            context.stepper.note_posvel_stale();
            context.physical.brake().reset();
            apply_initial_forces(context,false);
            context.stepper.forward();
            // The oracle records the residual (np.max|qacc|) without a
            // finite guard — a NaN lane lands in the result, not a throw.
            residual=released_residual(context);
            steps=held_steps;
        }
    }
    if(context.topology && context.spatial_report && !context.spatial_report->errors.empty() &&
       (!context.bootstrap_hold || context.physical.spatial_state()->bootstrap_phase>=spatial::BootstrapPhase::armed)) // NOLINT(bugprone-unchecked-optional-access) topology engagement implies an engaged spatial state by setup contract
        throw std::invalid_argument("spatial equilibrium: "+context.spatial_report->errors.front());
    EquilibriumResult result;result.steps=steps;result.residual_qacc=residual;result.cache_hit=cache_hit;
    result.spatial=context.spatial_report;
    const auto q=context.stepper.qpos();result.fork_travel_mm=1000.*q[static_cast<std::size_t>(context.stepper.suspension().fork_qposadr())];result.shock_stroke_mm=1000.*q[static_cast<std::size_t>(context.stepper.suspension().shock_qposadr())];
    if(!context.rear_travel_from_shock_mm)throw std::invalid_argument("equilibrium rear travel requires the source linkage solver");
    result.rear_travel_mm=context.rear_travel_from_shock_mm(result.shock_stroke_mm);
    if(context.topology) {
        const auto pose=spatial::body_pose(*context.stepper.model(),*context.stepper.data(),context.topology->root);
        result.root_z_m=pose.position_world_m[2];
        result.pitch_rad=spatial::euler_zyx(pose.quat_world_from_body_wxyz).pitch_rad;
    } else {
        result.root_z_m=q[static_cast<std::size_t>(joint_address(context,"root_z").qpos)];
        result.pitch_rad=q[static_cast<std::size_t>(joint_address(context,"root_pitch").qpos)];
    }
    result.initial_crank_phase_rad=context.physics.drive.crank_phase_rad;result.settled_crank_phase_rad=q[static_cast<std::size_t>(joint_address(context,"crank_spin").qpos)];result.state_file=explicit_file;
    const bool releasable=!context.bootstrap_hold || (context.spatial_report &&
        context.bootstrap_hold->release_check(*context.spatial_report,
            context.physics.spatial->bootstrap.release_force_limit_n, // NOLINT(bugprone-unchecked-optional-access) bootstrap_hold implies physics.spatial by setup contract
            context.physics.spatial->bootstrap.release_moment_limit_nm, // NOLINT(bugprone-unchecked-optional-access) bootstrap_hold implies physics.spatial by setup contract
            context.equilibrium_tolerance));
    if(context.physics.equilibrium_cache_enabled&&!cache_hit&&releasable)
        save_equilibrium_cache(context,capture_state(context,steps,residual));
    context.equilibrium=result;context.completed=true;return result;
}
} // namespace bike::initialization
