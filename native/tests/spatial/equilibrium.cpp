#include "support/migration_test.hpp"
#include "support/equilibrium_cases.hpp"
#include "support/ride_qualification.hpp"
#include "initialization/bootstrap_hold.hpp"
#include "initialization/spatial_equilibrium.hpp"
#include "pneumatic3d/state.hpp"
#include "writers/pneumatic_3d_tire.hpp"
#include "writers/spatial_tire_adapter.hpp"
#include "model_access.hpp"
#include "model/mapping.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
void failed_probe_preserves_state() {
    using namespace bike;
    spatial_tests::EquilibriumArena arena(spatial::RootKind::free6,spatial::TyreTier::fast,false);
    const auto* m=arena.stepper->model();auto* d=arena.stepper->data();
    const auto& root=arena.context->topology->root;
    const auto q=static_cast<std::size_t>(root.free_qpos_adr+3);
    const auto coordinates=model_access::mutable_buffer(d->qpos,m->nq);
    coordinates[q]=std::sqrt(.5);coordinates[q+1]=std::sqrt(.5);coordinates[q+2]=coordinates[q+3]=0.;
    model_access::mutable_buffer(d->qvel,m->nv)[static_cast<std::size_t>(root.free_dof_adr)]=.75;
    arena.stepper->note_posvel_stale();arena.stepper->forward();
    std::vector<double> before(static_cast<std::size_t>(engine::state_size(m,mjSTATE_INTEGRATION)));
    engine::get_state(m,d,before.data(),mjSTATE_INTEGRATION);
    const auto material=arena.stepper->spatial_tire_writer().state_bytes();
    const auto causal=io::canonical_json(runtime::encode_spatial_bootstrap(*arena.physical->spatial_state()));
    const auto hold=io::canonical_json(spatial::encode_hold(arena.context->bootstrap_hold->state()));
    bool rejected=false;
    try {(void)initialization::spatial_refine(*arena.context,1,.05);}
    catch(const pneumatic3d::SpatialTireEvaluationError&) {rejected=true;}
    bt::require(rejected,"vertical wheel axle rejects an observational contact probe");
    std::vector<double> after(before.size());engine::get_state(m,d,after.data(),mjSTATE_INTEGRATION);
    bt::require(std::ranges::equal(std::as_bytes(std::span{before}),std::as_bytes(std::span{after})) &&
        material==arena.stepper->spatial_tire_writer().state_bytes() &&
        causal==io::canonical_json(runtime::encode_spatial_bootstrap(*arena.physical->spatial_state())) &&
        hold==io::canonical_json(spatial::encode_hold(arena.context->bootstrap_hold->state())),
        "a failed one-point probe preserves the original velocity, forces, material and causal hold state");
}
void released_phase_still_requires_support() {
    using namespace bike;
    for(auto phase:{spatial::BootstrapPhase::armed,spatial::BootstrapPhase::scored}) {
        spatial_tests::EquilibriumArena arena(spatial::RootKind::free6,spatial::TyreTier::fast,false);
        (void)initialization::settle_context(*arena.context);
        bt::require(arena.context->spatial_report->errors.empty(),"release regression starts with a valid physical equilibrium");
        auto causal=*arena.physical->spatial_state();
        causal.bootstrap_phase=phase;causal.hold.phase=phase;causal.hold.active=false;
        causal.hold.last_force_world_n={};causal.hold.last_couple_world_nm={};causal.hold.stored_energy_j=0.;
        arena.physical->commit_spatial_state(arena.physical->prepare_spatial_state(causal));
        bt::require(arena.context->bootstrap_hold->restore(causal.hold)==CoreStatus::ok,"released hold state is valid before the physical probe");
        const auto* m=arena.stepper->model();auto* d=arena.stepper->data();
        model_access::mutable_buffer(d->qpos,m->nq)[static_cast<std::size_t>(arena.context->topology->root.free_qpos_adr+2)]+=.25;
        arena.stepper->note_posvel_stale();arena.stepper->forward();
        std::vector<double> before(static_cast<std::size_t>(engine::state_size(m,mjSTATE_INTEGRATION)));
        engine::get_state(m,d,before.data(),mjSTATE_INTEGRATION);
        const auto material=arena.stepper->spatial_tire_writer().state_bytes();
        const auto state=io::canonical_json(runtime::encode_spatial_bootstrap(*arena.physical->spatial_state()));
        bool rejected=false;
        try {initialization::finalize_initial_state(*arena.context,arena.config);}
        catch(const std::invalid_argument& error) {
            rejected=std::string_view(error.what())=="spatial equilibrium: missing or invalid wheel contacts";
        }
        bt::require(rejected,"an imported released phase cannot bypass an actual missing-support report");
        std::vector<double> after(before.size());engine::get_state(m,d,after.data(),mjSTATE_INTEGRATION);
        bt::require(std::ranges::equal(std::as_bytes(std::span{before}),std::as_bytes(std::span{after})) &&
            material==arena.stepper->spatial_tire_writer().state_bytes() &&
            state==io::canonical_json(runtime::encode_spatial_bootstrap(*arena.physical->spatial_state())),
            "failed release validation leaves integration, material and causal owners unchanged");
    }
}
void exhausted_budget_never_seals() {
    using namespace bike;
    // A one-step budget with a tolerance below floating-point noise cannot
    // converge, so setup must fail loudly (not release an unsettled pose) and
    // the unconverged owner must refuse to seal.
    auto request=spatial_tests::make_startup_request(spatial::RootKind::free6,spatial::TyreTier::fast,false,0.);
    auto options=request->equilibrium;options.max_steps=1;options.tolerance=1e-12;
    auto owner=initialization::make_setup(std::move(*request));
    bool failed=false;
    try {(void)initialization::settle(*owner,options);}
    // Only the W30 release failure counts; any other exception (terrain, engine,
    // solver dimension) propagates to bt::run as a test failure.
    catch(const std::invalid_argument& error) {
        failed=std::string_view(error.what()).starts_with("bootstrap: hold not releasable");
    }
    bt::require(failed,"an exhausted equilibrium budget is the W30 release failure, not a quiet success");
    bt::invalid([&]{(void)owner->seal_bootstrap();});
}
void local_rolling_start() {
    using namespace bike;
    namespace ride=spatial_tests::ride;
    // The small smooth relief keeps a static start inside the original
    // residual limits while separating the two actual wheel-contact slopes.
    const auto terrain=ride::surface_grid([](double x,double){return .0001*std::sin(2.*x);},
        [](double,double){return 1u;},ride::dry_and_wet(),200.,6.,.05,.5,"different local wheel slopes");
    for(auto tier:{spatial::TyreTier::fast,spatial::TyreTier::detailed}) {
        spatial_tests::EquilibriumArena arena(spatial::RootKind::free6,tier,false,terrain,3.);
        (void)initialization::settle_context(*arena.context);
        initialization::finalize_initial_state(*arena.context,arena.config);
        const auto* m=arena.stepper->model();const auto* d=arena.stepper->data();
        const auto& topology=*arena.context->topology;
        const auto pose=spatial::body_pose(*m,*d,topology.root);
        const auto root_sample=terrain->query_view().sample(pose.position_world_m[0],pose.position_world_m[1]);
        bt::require(root_sample.status==terrain::QueryStatus3d::ok,"rolling regression samples the root terrain");
        const auto rotations=model_access::readonly_buffer(d->xquat,4*m->nbody);
        const auto positions=model_access::readonly_buffer(d->xpos,3*m->nbody);
        const auto& snapshots=arena.stepper->tire_writer().snapshot_storage();
        double distinct_slope=0.;
        for(std::size_t side=0;side<2;++side) {
            const auto& wheel=topology.wheels[side];const auto& snap=snapshots[side];
            spatial::Vec3 normal{};
            for(const auto& patch:snap.patches)if(patch.working_surface && patch.normal_load_n>0.)
                for(std::size_t k=0;k<3;++k)normal[k]+=patch.normal_load_n*patch.normal[k];
            const double length=std::hypot(normal[0],normal[1],normal[2]);
            bt::require(length>0. && snap.effective_radius_m>0.,"moving wheel has loaded physical support");
            for(auto& x:normal)x/=length;
            distinct_slope=std::max(distinct_slope,std::abs(normal[0]-root_sample.normal_world[0]));
            const auto q=4*static_cast<std::size_t>(wheel.hub_body_id);
            const auto axle=spatial::rotate_body_to_world({rotations[q],rotations[q+1],rotations[q+2],rotations[q+3]},wheel.axle_axis_hub);
            const auto tangent=ride::scale(ride::cross(axle,normal),1./ride::norm(ride::cross(axle,normal)));
            const auto p=3*static_cast<std::size_t>(wheel.wheel_body_id);
            const auto velocity=model::point_velocity(*m,*d,wheel.wheel_body_id,{positions[p],positions[p+1],positions[p+2]});
            const auto omega=model::body_angular_velocity(*m,*d,wheel.hub_body_id);
            const double expected=ride::dot(velocity,tangent)/snap.effective_radius_m-ride::dot(omega,axle);
            bt::require(std::abs(arena.stepper->qvel()[static_cast<std::size_t>(wheel.spin.dof_adr)]-expected)<=1e-12,
                "moving wheel speed uses its local contact tangent and non-spin hub motion");
        }
        bt::require(distinct_slope>1e-5,"rolling regression distinguishes wheel contact normals from the root normal");
    }
}
} // namespace

int main(){return bt::run([] {
    using namespace bike;
    for(auto tier:{spatial::TyreTier::fast,spatial::TyreTier::detailed}) {
        spatial_tests::EquilibriumArena arena(spatial::RootKind::free6,tier,false);
        initialization::apply_initial_forces(*arena.context,false);arena.stepper->forward();
        bool has_inactive_slot=false;
        for(std::size_t side=0;side<2;++side) {
            const auto wrenches=arena.stepper->spatial_tire_writer().wrenches(side);
            bt::require(std::ranges::any_of(wrenches,[](const auto& p){return p.valid && p.normal_load_n>0.;}),
                "the geometric seed supports each wheel");
            has_inactive_slot=has_inactive_slot || std::ranges::any_of(wrenches,[](const auto& p){return !p.valid;});
        }
        if(tier==spatial::TyreTier::detailed)
            bt::require(has_inactive_slot,"a loaded detailed fan retains ordinary airborne slots");
        (void)initialization::spatial_refine(*arena.context,1,.05);
        bt::require(arena.context->spatial_report->contacts_valid==2 && arena.context->spatial_report->contacts_total==2,
            "equilibrium requires two supported wheels without counting inactive fan slots as invalid contacts");
    }
    for(auto root:{spatial::RootKind::constrained4,spatial::RootKind::free6})
        for(bool rider:{false,true}) {
            spatial_tests::EquilibriumArena arena(root,spatial::TyreTier::fast,rider);
            const auto material=arena.stepper->spatial_tire_writer().state_bytes();
            const auto qpos=model_access::mutable_buffer(arena.stepper->data()->qpos,arena.stepper->model()->nq);
            const std::vector<double> seed(qpos.begin(),qpos.end());
            const auto& binding=arena.context->topology->root;
            const auto height=static_cast<std::size_t>(root==spatial::RootKind::free6?binding.free_qpos_adr+2:binding.scalar[2].qpos_adr);
            qpos[height]+=.02;arena.stepper->note_posvel_stale();
            std::vector<double> before(static_cast<std::size_t>(engine::state_size(arena.stepper->model(),mjSTATE_INTEGRATION)));
            engine::get_state(arena.stepper->model(),arena.stepper->data(),before.data(),mjSTATE_INTEGRATION);
            (void)initialization::spatial_refine(*arena.context,1,.05);
            bt::require(arena.context->spatial_report!=nullptr,"one-evaluation probe emits a physical-unit report");
            bt::require(arena.context->spatial_report->contacts_valid<arena.context->spatial_report->contacts_total,
                "lifting the whole rig leaves required wheel support missing");
            bt::require((arena.context->spatial_report->max_linear_accel_mps2>.05 || arena.context->spatial_report->max_angular_accel_radps2>.05) &&
                !arena.context->spatial_report->errors.empty() && !arena.context->spatial_report->worst_dofs.empty(),
                "one evaluation of a lifted rig cannot claim convergence and identifies its worst DOFs");
            std::vector<double> after(before.size());
            engine::get_state(arena.stepper->model(),arena.stepper->data(),after.data(),mjSTATE_INTEGRATION);
            bt::require(std::ranges::equal(std::as_bytes(std::span{before}),std::as_bytes(std::span{after})),
                "one-point reporting preserves the complete integration inventory bitwise");
            bt::require(material==arena.stepper->spatial_tire_writer().state_bytes(),"probe evaluation preserves committed material bytes");
            std::ranges::copy(seed,qpos.begin());arena.stepper->note_posvel_stale();
            for(int attempt=0;attempt<8;++attempt)
                if(initialization::spatial_refine(*arena.context,80,.05)<=.05)break;
            const auto& report=*arena.context->spatial_report;
            bt::require(report.max_linear_accel_mps2<=.05 && report.max_angular_accel_radps2<=.05,
                "equilibrium reports translation and rotation in separate physical units");
            bt::require(report.weight_residual_fraction<=.005,"settled contact forces match total weight");
            bt::require(std::abs(report.pitch_moment_residual_nm)<=1. && std::abs(report.roll_moment_residual_nm)<=1.,
                "settled roll and pitch external moments are bounded");
            bt::require(material==arena.stepper->spatial_tire_writer().state_bytes(),"optimizer order never commits tyre history");
            if(root==spatial::RootKind::free6) {
                const auto qa=static_cast<std::size_t>(arena.context->topology->root.free_qpos_adr+3);
                double norm2=0.;for(std::size_t k=0;k<4;++k)norm2+=qpos[qa+k]*qpos[qa+k];
                bt::require(std::abs(std::sqrt(norm2)-1.)<=1e-12,"manifold integration keeps a unit quaternion");
                const double residual_before=initialization::spatial_refine(*arena.context,1,.05);
                for(std::size_t k=0;k<4;++k)qpos[qa+k]=-qpos[qa+k];
                arena.stepper->note_posvel_stale();
                const double residual_after=initialization::spatial_refine(*arena.context,1,.05);
                bt::require(std::abs(residual_after-residual_before)<=1e-12,"q and minus q give the same equilibrium residual");
            }
        }
    auto mixed=spatial_tests::mixed_joint_rig();
    const auto addresses=model_access::readonly_buffer(mixed.model->jnt_dofadr,mixed.model->njnt);
    const auto qaddresses=model_access::readonly_buffer(mixed.model->jnt_qposadr,mixed.model->njnt);
    const auto mixed_qpos=model_access::mutable_buffer(mixed.data->qpos,mixed.model->nq);
    std::vector<double> displacement(static_cast<std::size_t>(mixed.model->nv),0.);
    const int joint=mj_name2id(mixed.model,mjOBJ_JOINT,"unrelated_ball");
    bt::require(joint>=0,"mixed fixture contains a ball joint before root addresses");
    displacement[static_cast<std::size_t>(addresses[static_cast<std::size_t>(joint)])]=.01;
    engine::integrate_pos(mixed.model,mixed.data->qpos,displacement.data(),1.);
    const auto q=static_cast<std::size_t>(qaddresses[static_cast<std::size_t>(joint)]);
    double norm=0.;for(std::size_t k=0;k<4;++k)norm+=mixed_qpos[q+k]*mixed_qpos[q+k];
    bt::require(std::abs(norm-1.)<=1e-12,"ball joint uses three tangent and four position coordinates");
    const int slide=mj_name2id(mixed.model,mjOBJ_JOINT,"prefix_slide");
    bt::require(slide>=0,"mixed fixture supplies an independent unit-mass translation");
    const auto slide_index=static_cast<std::size_t>(slide);
    const auto slide_dof=static_cast<std::size_t>(addresses[slide_index]);
    auto stiffness=model_access::mutable_buffer(mixed.model->jnt_stiffness,mixed.model->njnt);
    stiffness[slide_index]=200.;
    const std::vector<double> base(mixed_qpos.begin(),mixed_qpos.end());
    const auto acceleration=model_access::readonly_buffer(mixed.data->qacc,mixed.model->nv);
    const auto evaluate=[&](double h) {
        std::ranges::copy(base,mixed_qpos.begin());std::ranges::fill(displacement,0.);displacement[slide_dof]=h;
        engine::integrate_pos(mixed.model,mixed.data->qpos,displacement.data(),1.);engine::forward(mixed.model,mixed.data);
        return acceleration[slide_dof];
    };
    for(double h:{1e-6,5e-7})bt::require(std::abs((evaluate(h)-evaluate(-h))/(2.*h)+200.)<=1e-8,
        "a central tangent perturbation reproduces the independent spring acceleration derivative -k/m");
    auto request=spatial_tests::make_startup_request(spatial::RootKind::free6,spatial::TyreTier::fast,false,0.);
    auto owner=initialization::make_setup(std::move(*request));
    bt::invalid([&]{(void)owner->seal_bootstrap();});
    failed_probe_preserves_state();
    exhausted_budget_never_seals();
    released_phase_still_requires_support();
    local_rolling_start();
});}
