// T30 attachment topology, support wrench and energy contracts.
// Test bodies contain future engine calls; none were invoked while editing.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "rider/support_dispatch.hpp"
#include "rider/legacy_diagnostics.hpp"
#include "rider/state_codec.hpp"
#include "rider/contact_math.hpp"
#include "writers/rider_contacts.hpp"
#include "runtime/capabilities.hpp"
#include "runtime/project_config.hpp"
#include "configuration/model_config.hpp"
#include "model/builder.hpp"
#include "model/rider_geometry.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"
#include <array>
#include <cmath>
#include <memory>

namespace {
namespace r=bike::rider;
namespace cfg=bike::configuration;
namespace mdl=bike::model;
cfg::SimulationPhysicsConfig physical() {
    auto c=cfg::default_physics();c.physics_mode="physical";c.drive_mode="articulated_effort";c.pitch_assist=false;cfg::post_init(c);return c;
}
::rider::RiderContactsConfig contacts(const cfg::ArticulatedConfig& a) {
    ::rider::RiderContactsConfig c;c.arm_reach_m=.7;c.saddle_patch_half_length_m=a.saddle_patch_half_length_m;c.pedal_patch_half_length_m=a.pedal_patch_half_length_m;c.support_pad_radius_m=a.support_pad_radius_m;c.support_k_n_m=a.support_k_n_m;c.support_c_ns_m=a.support_c_ns_m;c.pedal_c_ns_m=a.pedal_c_ns_m;c.support_tangent_k_n_m=a.support_tangent_k_n_m;c.support_mu=a.support_mu;c.support_length_m=a.support_length_m;c.grip_k_n_m=a.grip_k_n_m;c.grip_c_ns_m=a.grip_c_ns_m;c.grip_release_distance_m=a.grip_release_distance_m;c.grip_pair_force_limit_n=a.grip_pair_force_limit_n;c.grip_capture_distance_m=a.grip_capture_distance_m;c.grip_capture_speed_mps=a.grip_capture_speed_mps;c.pedal_attachment=*::rider::pedal_attachment(a.pedal_attachment);c.saddle_attachment=*::rider::saddle_attachment(a.saddle_attachment);c.grip_attachment=*::rider::grip_attachment(a.grip_attachment);return c;
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

[[gnu::noinline]] void support_wrench() {
    const std::array<double,4> x{0.,-.2,.2,.6};const std::array<bool,4> enabled{true,true,true,true};
    const auto loads=r::gravity_support_targets(800.,.1,x,0.,.33,.12,enabled);
    double total=0.,moment=0.,crank=0.;for(std::size_t i=0;i<4;++i){total+=loads.forces[i];moment+=(x[i]-.1)*loads.forces[i];if(i==1||i==2)crank+=x[i]*loads.forces[i];}
    bt::require(loads.diagnostics.feasible&&std::abs(total-800.)<1e-8&&std::abs(moment)<1e-8&&std::abs(crank)<1e-8,"gravity support conserves weight and both declared moments");
    const auto detached=r::gravity_support_targets(800.,.1,x,0.,.33,.12,{false,false,false,false});
    bt::require(!detached.diagnostics.feasible&&detached.diagnostics.vertical_force_error_n==-800.,"missing contacts expose unsupported weight");
    const std::array<std::array<double,3>,4> points{{{0.,0.,.6},{-.2,0.,0.},{.2,0.,0.},{.6,0.,.8}}};
    const auto targets=r::pedaling_support_targets(800.,{.1,0.,1.},points,0.,.33,.12,enabled,{{{10.,0.,-100.},{-5.,0.,-30.}}},0.,{20.,0.,0.});
    bt::require(targets.diagnostics.feasible&&std::abs(targets.diagnostics.total_requested_force[0]-20.)<1e-8&&std::abs(targets.diagnostics.total_requested_force[2]-800.)<1e-8,"pedaling reaction is part of rider support, not extra weight");
    const ::rider::Mat3 rotation{1.,0.,0.,0.,1.,0.,0.,0.,1.};
    const auto top=::rider::box_pad_contact({0.,0.,.015},.01,{0.,0.,0.},rotation,{.05,.1,.01});
    const auto below=::rider::box_pad_contact({0.,0.,-.015},.01,{0.,0.,0.},rotation,{.05,.1,.01});
    bt::require(top.normal[2]==1.&&below.normal[2]==-1.&&top.gap_m<0.&&below.gap_m<0.,"finite pedal has both broad faces");
    const auto release=::rider::release_if_overloaded({100.,0.,0.},3.,50.);
    bt::require(release.overloaded&&release.force==::rider::Vec3{}&&release.loss_j==3.,"overload release accounts stored spring energy exactly once");
}
[[gnu::noinline]] void topology_and_controller() {
    mdl::BuildRequest request;request.mode="ride";request.rider=cfg::RiderSpecs{};request.rider->variant="articulated_planar";request.physics=physical();
    for(const auto*pedal:{"flat","weld","spindle"})for(const auto*saddle:{"flat","weld","pin"})for(const auto*grip:{"spring","connect"}) {
        auto&a=request.physics->articulated;a.pedal_attachment=pedal;a.saddle_attachment=saddle;a.grip_attachment=grip;
        const auto report=runtime::implemented_capabilities(*request.physics,*request.rider);
        bt::require(report.implemented&&report.qualification_pending&&!report.supported,"implemented topology remains distinct from qualified evidence");
        auto model=mdl::compile_mjcf(mdl::generate_mjcf(request));std::unique_ptr<mjData,decltype(&mj_deleteData)> data(engine::make_data(model.get()),mj_deleteData);engine::forward(model.get(),data.get());
        auto writer=r::make_support_writer(*model,contacts(a));writer->reset(data.get());
        const auto dispatch=r::support_dispatch(contacts(a));bt::require(dispatch.closed_form_spindle==(a.pedal_attachment=="spindle"),"source spindle selection depends on pedal topology");
        const auto pose=mdl::geometry_pose(*request.rider,request.bike);
        auto controller=std::make_unique<::rider::SpindleController>(model.get(),::rider::decode_spindle_pose(runtime::project_articulated_pose(pose),"pose"),::rider::decode_spindle_config(runtime::project_rider_controller(*request.physics,*model)),request.bike.crank_length/1000.);
        bt::require(controller->spindle()==dispatch.closed_form_spindle&&controller->welded()==(a.pedal_attachment=="weld"),"controller and force writer agree on attachments");
        const auto state=controller->state();controller->restore(::rider::decode_spindle_state(::rider::encode_spindle_state(state),"rider"));
        if(a.grip_attachment=="spring")bt::require(!r::set_spring_grip_enabled(*writer,false)&&!writer->state().enabled[writers::kGripIndex],"spring grip releases through owner");
        else bt::require(r::set_spring_grip_enabled(*writer,false)&&writer->state().enabled[writers::kGripIndex],"connect grip cannot release a compiled equality");
        const auto settled=r::settle_supports(*writer,*data);(void)settled;
        bt::require(std::isfinite(writer->stored_energy(data.get())),"settle and energy use the same support owner");
    }
    auto wrong=*request.rider;wrong.variant="lumped";
    bt::require(!runtime::implemented_capabilities(*request.physics,wrong).implemented,"nonflat attachment model requires articulated rider");
    auto assisted=*request.physics;assisted.pitch_assist=true;bt::invalid([&]{(void)runtime::implemented_capabilities(assisted,*request.rider);});
}
[[gnu::noinline]] void diagnostics_roundtrip() {
    r::LegacySupportDiagnostics d;d.target.vertical_forces_n={400.,150.,150.,100.};d.target.feasible=true;d.stance={true,false};d.posture.pelvis_offset_m=std::array<double,2>{.01,.02};d.target_phase_rad=0.;d.phase_error_rad=.1;d.feet[0].weld=true;d.feet[1].recovery=true;d.feet[1].recovery_stage="escape";
    const auto encoded=r::encode_support_diagnostics(d);const auto restored=r::decode_support_diagnostics(encoded,"support");
    bt::require(bike::io::canonical_json(r::encode_support_diagnostics(restored))==bike::io::canonical_json(encoded),"force targets, optional phase and foot recovery roundtrip");
}
}
int main(){return bt::run([]{support_wrench();topology_and_controller();diagnostics_roundtrip();});}
NATIVE_DIAG_POP
