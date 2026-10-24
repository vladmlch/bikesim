// No window is created by this regression. GUI smoke is a separate V08 gate.
#include "support/migration_test.hpp"
#include "viewer/render_owner.hpp"
#include "initialization/capture.hpp"
#include "model/builder.hpp"
#include "engine_call.hpp"
#include "runtime/presentation_checkpoint.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>

namespace {
void camera_contract() {
    bike::viewer::Camera tracker; mjvCamera camera{};
    tracker.set_mode(bike::viewer::CameraMode::chase);
    tracker.update_spatial(camera,{2.,3.,4.},0.);
    bt::require(camera.lookat[1]==3.&&std::abs(camera.lookat[2]-4.30)<=1e-12,"chase follows all three world coordinates");
    tracker.update_spatial(camera,{2.,3.,4.},std::numbers::pi);
    const double before=camera.azimuth;
    tracker.update_spatial(camera,{2.,3.,4.},-std::numbers::pi);
    bt::require(std::abs(camera.azimuth-before)<=1e-9,"equivalent headings at the branch cut do not jump");
    tracker.set_mode(bike::viewer::CameraMode::side);
    tracker.update_spatial(camera,{2.,3.,4.},std::nullopt);
    bt::require(camera.lookat[1]==3.,"side view keeps the lateral position");
    tracker.reset_preset();tracker.update_spatial(camera,{10.,8.,7.},std::nullopt);
    bt::require(std::abs(camera.lookat[2]-7.25)<=1e-12,"reset clears the old camera smoothing target");
    const auto x=camera.lookat[0],y=camera.lookat[1],z=camera.lookat[2];
    bt::invalid([&]{tracker.update_spatial(camera,{0.,std::numeric_limits<double>::quiet_NaN(),0.},{});});
    bt::require(camera.lookat[0]==x&&camera.lookat[1]==y&&camera.lookat[2]==z,"invalid camera input leaves prior state intact");
}
void replica_contract() {
    const auto model=bike::model::compile_mjcf(R"(<mujoco><worldbody><body name="frame" pos="0 0 .5"><freejoint name="root_free"/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>)");
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data(engine::make_data(model.get()),mj_deleteData);
    const auto image=bike::initialization::capture_model_image(*model);
    runtime::PresentationState presentation;presentation.z_m=.5;presentation.root_x_dofadr=-1;
    presentation.spatial.emplace();presentation.spatial->position_world_m={0.,0.,.5};
    presentation.spatial->attitude_valid=false;
    bt::require(!presentation.spatial->heading_rad&&!presentation.spatial->roll_rad,"invalid Euler attitude has no heading/roll sentinel");
    auto frame=bike::viewer::capture_frame(*model,*data,0,0,image.sha256,presentation);
    frame.presentation.fork_force_n=-0.;frame.presentation.shock_force_n=123.456;
    frame.presentation.tire_slip={.25,-.75};frame.presentation.tire_mu={.6,.8};
    frame.presentation.drive.set(drivetrain::TelemetryField::cadence_rpm,91.25);
    frame.presentation.drive.set_none(drivetrain::TelemetryField::crank_target_phase_rad);
    const auto encoded=runtime::encode_presentation_checkpoint(frame.presentation);
    const auto restored=runtime::decode_presentation_checkpoint(encoded,frame.presentation);
    bt::require(bike::io::canonical_json(runtime::encode_presentation_checkpoint(restored))==bike::io::canonical_json(encoded),"complete presentation cache round-trips exact force/tire/drive presence and signed-zero values without recomputation");
    auto malformed=encoded;
    runtime::setupvalue::set(malformed,"root_x_dofadr",runtime::setupvalue::integer(99));
    bt::invalid([&]{(void)runtime::decode_presentation_checkpoint(malformed,frame.presentation);});
    malformed=encoded;auto fields=runtime::setupvalue::field(malformed,"model_fields");
    runtime::setupvalue::set(fields,"site_pos",runtime::setupvalue::reals(std::array{1.}));
    runtime::setupvalue::set(malformed,"model_fields",fields);
    bt::invalid([&]{(void)runtime::decode_presentation_checkpoint(malformed,frame.presentation);});
    bike::viewer::RenderOwner owner(image);owner.apply(frame);
    const auto accepted=owner.integration_state();
    auto wrong=frame;wrong.model_sha256[0]=wrong.model_sha256[0]=='0'?'1':'0';
    bt::invalid([&]{owner.apply(wrong);});
    wrong=frame;wrong.presentation.spatial->position_world_m[1]=std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&]{owner.apply(wrong);});
    wrong=frame;wrong.presentation.spatial->generation=1;
    bt::invalid([&]{owner.apply(wrong);});
    bt::require(owner.integration_state()==accepted,"spatial validation rejects before the replica changes");
    frame.presentation.spatial->quat_wxyz={-2.,0.,0.,0.};
    owner.apply(frame);
    bt::require(owner.integration_state()==accepted&&frame.presentation.spatial->quat_wxyz[0]==-2.,"render normalization never alters accepted integration state or caller pose");
    frame.state.generation=1;frame.presentation.spatial->generation=1;
    owner.apply(frame);
    bt::require(owner.integration_state()==accepted,"new generation permits an unsmoothed reset to the accepted state");
}
}
int main(){return bt::run([]{camera_contract();replica_contract();});}
