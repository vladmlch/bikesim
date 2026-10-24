#include "support/migration_test.hpp"
#include "support/equilibrium_cases.hpp"
#include "initialization/bootstrap_hold.hpp"
#include "initialization/spatial_equilibrium.hpp"
#include "pneumatic3d/state.hpp"
#include "writers/pneumatic_3d_tire.hpp"
#include "model_access.hpp"

#include <cmath>
#include <ranges>

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
});}
