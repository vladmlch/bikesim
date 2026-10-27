#include "support/migration_test.hpp"
#include "support/startup_fixture.hpp"
#include "support/startup_cases.hpp"
#include "initialization/bootstrap_hold.hpp"
#include "runtime/accounting.hpp"
#include "model_access.hpp"
#include "spatial/frames.hpp"
#include "engine_call.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

int main(){return bt::run([] {
    using namespace bike;
    spatial::Config cfg;cfg.mode=spatial::Mode::spatial3d;cfg.root=spatial::RootKind::free6;
    auto rig=spatial_tests::free_rig(cfg,false);
    const auto q=static_cast<std::size_t>(rig.topology.root.free_qpos_adr);
    const auto qpos=model_access::mutable_buffer(rig.data->qpos,rig.model->nq);
    qpos[q+3]=std::sqrt(.5);qpos[q+4]=0.;qpos[q+5]=0.;qpos[q+6]=std::sqrt(.5);
    engine::forward(rig.model,rig.data);
    initialization::BootstrapHold hold(cfg.bootstrap,rig.topology.root,*rig.model);
    std::vector<double> force(static_cast<std::size_t>(rig.model->nv),0.);
    bt::require(hold.try_compute_into(*rig.model,*rig.data,spatial::BootstrapPhase::settling,force)==CoreStatus::ok,"hold initializes its yawed reference");
    qpos[q+1]+=.01;engine::forward(rig.model,rig.data);
    std::ranges::fill(force,0.);
    bt::require(hold.try_compute_into(*rig.model,*rig.data,spatial::BootstrapPhase::release_check,force)==CoreStatus::ok,"world lateral displacement is supported");
    bt::require(hold.state().last_force_world_n[0]==0. && hold.state().last_force_world_n[1]<0. &&
        hold.state().last_force_world_n[2]==0.,"yaw ninety degrees never rotates the world-Y hold into a body axis");
    initialization::SpatialEquilibriumReport report;report.contacts_valid=report.contacts_total=2;
    bt::require(!hold.release_check(report,1.,1.),"a nonzero residual hold cannot be released");
    auto energy=hold.state();spatial::BodyPose moving;moving.linear_velocity_world_mps[1]=2.;
    const double expected=.01*energy.last_force_world_n[1]*2.;
    bt::require(runtime::account_bootstrap_hold(energy,moving,spatial::BootstrapPhase::release_check,.01) &&
        energy.work_j==expected,"accepted bootstrap support work is the world wrench dotted with its body twist");
    bt::require(runtime::account_bootstrap_hold(energy,moving,spatial::BootstrapPhase::armed,.01) &&
        energy.work_j==expected,"armed accounting does not integrate a stale bootstrap wrench");
    qpos[q+1]-=.01;engine::forward(rig.model,rig.data);
    std::ranges::fill(force,0.);(void)hold.try_compute_into(*rig.model,*rig.data,spatial::BootstrapPhase::release_check,force);
    bt::require(hold.release_check(report,1.,1.),"balanced geometry with valid contacts can be released");
    auto bad=report;bad.errors.emplace_back("bank needs external support");
    bt::require(!hold.release_check(bad,1.,1.),"banked or invalid residual reports cannot silently release");
    const double work=hold.state().work_j;
    std::ranges::fill(force,0.);
    bt::require(hold.try_compute_into(*rig.model,*rig.data,spatial::BootstrapPhase::armed,force)==CoreStatus::ok,"hold enters armed state");
    bt::require(std::ranges::all_of(force,[](double x){return x==0.;}),"armed state contributes exact zero force");
    bt::require(hold.try_compute_into(*rig.model,*rig.data,spatial::BootstrapPhase::scored,force)==CoreStatus::ok,"scored state remains unsupported");
    bt::require(!hold.state().active && hold.state().work_j==work &&
        std::ranges::all_of(force,[](double x){return x==0.;}),"scored hold force and work increment are exactly zero");
    initialization::BootstrapHold restored(cfg.bootstrap,rig.topology.root,*rig.model);
    bt::require(restored.restore(spatial::decode_hold(spatial::encode_hold(hold.state())))==CoreStatus::ok,"scored state restores without reactivation");
    auto invalid=hold.state();invalid.active=true;
    bt::require(restored.restore(invalid)==CoreStatus::invalid_input,"active scored restore rejected");
    const auto request=spatial_tests::make_startup_request(spatial::RootKind::free6,spatial::TyreTier::fast,false,0.);
    spatial_tests::OwnedStartup stopped(*request);
    bike::validation::RuntimeRigAccess::inspect(*stopped.runtime,[&](const Stepper& stepper,const runtime::PhysicalStep& physical) {
        const auto& causal=*physical.spatial_state();
        bt::require(causal.bootstrap_phase==spatial::BootstrapPhase::scored && !causal.hold.active,
            "a zero-speed fresh startup reaches scored t=0 with no active support");
        initialization::BootstrapHold scored(request->model.physics->spatial->bootstrap,physical.topology()->root,*stepper.model());
        bt::require(scored.restore(causal.hold)==CoreStatus::ok,"zero-speed scored state restores into its physical owner");
        std::vector<double> zero(static_cast<std::size_t>(stepper.model()->nv),0.);
        bt::require(scored.try_compute_into(*stepper.model(),*stepper.data(),spatial::BootstrapPhase::scored,zero)==CoreStatus::ok &&
            std::ranges::all_of(zero,[](double x){return x==0.;}) && scored.state().work_j==causal.hold.work_j,
            "restored scored support produces exact zero generalized force and no work increment");
    });
    const auto bank=std::make_shared<const terrain::Terrain3dSource>(spatial_tests::plane_grid(
        0.,std::tan(10.*std::numbers::pi/180.),0.,401,25,.5,.5));
    auto bank_request=spatial_tests::make_startup_request(spatial::RootKind::free6,spatial::TyreTier::fast,false,0.,bank);
    bank_request->model.physics->spatial->steer.controller=spatial::SteerController::off;
    bank_request->model.spatial=bank_request->model.physics->spatial;
    bool bank_rejected=false;
    try {spatial_tests::OwnedStartup banked(std::move(*bank_request));}
    catch(const std::invalid_argument& error) {bank_rejected=std::string_view(error.what()).starts_with("bootstrap: hold not releasable: residual force");}
    bt::require(bank_rejected,"a ten-degree bank at zero speed cannot silently release required external balance support");
});}
