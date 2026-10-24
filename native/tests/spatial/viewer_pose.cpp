// No window is created by this regression. GUI smoke is a separate V08 gate.
#include "support/migration_test.hpp"
#include "viewer/render_owner.hpp"
#include "initialization/capture.hpp"
#include "model/builder.hpp"
#include "engine_call.hpp"
#include "replay/playback.hpp"
#include "runtime/presentation_checkpoint.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <utility>

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
void articulated_replica_contract() {
    const auto model=bike::model::compile_mjcf(R"(<mujoco><worldbody>
      <body name="frame" pos="0 0 .5"><freejoint name="root_free"/><geom type="sphere" size=".1" mass="1"/></body>
      <body name="rider_pelvis" pos="1 0 1.5"><freejoint name="rider_root_free"/><geom type="sphere" size=".1" mass="1"/></body>
      </worldbody></mujoco>)");
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data(engine::make_data(model.get()),mj_deleteData);
    const auto image=bike::initialization::capture_model_image(*model);
    runtime::PresentationState presentation;presentation.z_m=.5;presentation.root_x_dofadr=-1;
    presentation.spatial.emplace();presentation.spatial->position_world_m={0.,0.,.5};
    bike::viewer::RenderOwner owner(image);
    owner.apply(bike::viewer::capture_frame(*model,*data,0,0,image.sha256,presentation));
    const int root=model->jnt_qposadr[mj_name2id(model.get(),mjOBJ_JOINT,"root_free")];
    const int rider=model->jnt_qposadr[mj_name2id(model.get(),mjOBJ_JOINT,"rider_root_free")];
    const double half=std::sqrt(.5);
    const std::array root_pose{5.,2.,.5,half,0.,0.,half};
    const std::array rider_pose{5.,3.,1.5,half,0.,0.,half};
    for(std::size_t i=0;i<7;++i) {
        data->qpos[root+static_cast<int>(i)]=root_pose[i];
        data->qpos[rider+static_cast<int>(i)]=rider_pose[i];
    }
    data->time=.01;
    presentation.spatial->position_world_m={5.,2.,.5};
    presentation.spatial->quat_wxyz={half,0.,0.,half};
    auto next=bike::viewer::capture_frame(*model,*data,0,1,image.sha256,presentation);
    const auto accepted=next.state.integration_state;
    owner.apply(next);
    const auto shown=owner.rendered_qpos();
    const auto r=static_cast<std::size_t>(root),p=static_cast<std::size_t>(rider);
    const double yaw=std::numbers::pi/10.; // alpha=.2 of the accepted quarter turn
    bt::require(std::abs(shown[r]-1.)<1e-12&&std::abs(shown[r+1]-.4)<1e-12,
        "the render bike keeps its existing positional smoothing");
    bt::require(std::abs(shown[p]-shown[r]-std::cos(yaw))<1e-12&&
        std::abs(shown[p+1]-shown[r+1]-std::sin(yaw))<1e-12&&
        std::abs(shown[p+2]-shown[r+2]-1.)<1e-12,
        "the independent pelvis preserves its accepted bike-relative offset through translation and rotation");
    for(std::size_t i=0;i<4;++i)bt::require(std::abs(shown[r+3+i]-shown[p+3+i])<1e-12,
        "equal accepted bike/rider orientations remain equal in the smoothed copy");
    bt::require(owner.integration_state()==accepted&&next.state.integration_state==accepted,
        "coherent display smoothing never rewrites either accepted state owner");
    next.state.generation=1;next.presentation.spatial->generation=1;owner.apply(next);
    const auto reset=owner.rendered_qpos();
    for(std::size_t i=0;i<3;++i)bt::require(std::abs(reset[r+i]-root_pose[i])<1e-12&&
        std::abs(reset[p+i]-rider_pose[i])<1e-12,
        "generation reset restores the accepted bike and pelvis without the old display offset");
}
bike::replay::PlaybackData free_playback() {
    const auto model=bike::model::compile_mjcf(R"(<mujoco><worldbody><body name="frame"><freejoint name="root_free"/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>)");
    bike::replay::PlaybackData saved;
    saved.model=bike::initialization::capture_model_image(*model);
    saved.times_s={0.,1.};saved.ends_s={1.,2.};saved.duration_s=2.;
    const double q=std::sqrt(.5);
    saved.qpos={{0.,0.,.5,q,0.,0.,q},{4.,2.,1.5,-q,0.,0.,-q}};
    saved.qvel={{0.,2.,0.,0.,0.,0.},{0.,4.,0.,0.,0.,0.}};
    saved.quaternion_starts={3};
    saved.spatial.resize(2);
    for(std::size_t i=0;i<saved.spatial.size();++i) {
        auto& pose=saved.spatial[i];
        pose.position_world_m={saved.qpos[i][0],saved.qpos[i][1],saved.qpos[i][2]};
        pose.quat_wxyz={q,0.,0.,q};pose.attitude_valid=true;
        pose.heading_rad=std::numbers::pi/2.;pose.roll_rad=0.;
        pose.contact_valid={true,false};pose.lateral_slip_mps={.25,0.};
        pose.steer_angle_rad=.125;pose.steer_torque_nm=2.;
    }
    return saved;
}
void playback_free_contract() {
    bike::replay::PlaybackSession playback(free_playback());
    const auto raw=playback.data().qpos;
    const auto expected=bike::replay::pose_at(playback.data(),.5);
    const auto frame=playback.frame(.5,3);
    const auto& view=frame.presentation;const auto& pose=view.spatial.value();
    bt::require(pose.position_world_m==std::array{expected[0],expected[1],expected[2]}&&
        pose.position_world_m==std::array{2.,1.,1.},"inspection root pose follows the interpolated saved skeleton rather than its left keyframe");
    bt::require(std::abs(pose.quat_wxyz[0]-expected[3])<=1e-12&&
        std::abs(pose.quat_wxyz[3]-expected[6])<=1e-12,"antipodal saved quaternions retain the interpolated rotation");
    bt::require(std::abs(view.speed_mps-4.)<=1e-12&&view.root_x_dofadr==-1&&
        std::abs(view.pitch_rad)<=1e-12,"free-root HUD projects saved velocity onto the displayed horizontal heading");
    bt::require(pose.generation==3&&pose.contact_valid[0]&&!pose.contact_valid[1]&&
        pose.lateral_slip_mps[0]==.25&&pose.steer_angle_rad==.125&&pose.steer_torque_nm==2.,
        "inspection keeps recorded contact/control evidence and the current generation");
    bike::viewer::RenderOwner owner(playback.model_image());owner.apply(frame);
    bt::require(owner.integration_state()==frame.state.integration_state&&playback.data().qpos==raw,
        "inspection and render interpolation do not change saved integration coordinates");
}
void playback_singular_contract() {
    auto saved=free_playback();const double q=std::sqrt(.5);
    for(auto& row:saved.qpos){row[3]=q;row[4]=0.;row[5]=q;row[6]=0.;}
    bike::replay::PlaybackSession playback(std::move(saved));
    const auto frame=playback.frame(.5);
    const auto& pose=frame.presentation.spatial.value();
    bt::require(!pose.attitude_valid&&!pose.heading_rad&&!pose.roll_rad&&
        std::abs(frame.presentation.pitch_rad-std::numbers::pi/2.)<=1e-12,
        "interpolated vertical heading clears saved Euler validity instead of reusing a stale valid keyframe");
    bt::require(frame.presentation.speed_mps==0.,"undefined horizontal heading keeps the existing finite display-speed sentinel");
}
void playback_constrained_contract() {
    const auto model=bike::model::compile_mjcf(R"(<mujoco><worldbody>
      <body name="other"><joint name="other_slide" type="slide"/><geom type="sphere" size=".1" mass="1"/></body>
      <body name="frame" pos="0 0 .5"><joint name="root_x" type="slide" axis="1 0 0"/>
        <joint name="root_y" type="slide" axis="0 1 0"/><joint name="root_z" type="slide" axis="0 0 1"/>
        <joint name="root_pitch" type="hinge" axis="0 1 0"/><geom type="sphere" size=".1" mass="1"/></body>
      </worldbody></mujoco>)");
    bike::replay::PlaybackData saved;saved.model=bike::initialization::capture_model_image(*model);
    saved.times_s={0.,1.};saved.ends_s={1.,2.};saved.duration_s=2.;
    saved.qpos={{0.,0.,0.,0.,.2},{0.,4.,2.,1.,.2}};
    saved.qvel={{0.,2.,1.,0.,0.},{0.,4.,2.,0.,0.}};
    saved.spatial.resize(2);
    bike::replay::PlaybackSession playback(std::move(saved));
    const auto frame=playback.frame(.5);
    const auto& view=frame.presentation;const auto& pose=view.spatial.value();
    bt::require(pose.position_world_m==std::array{2.,1.,1.}&&std::abs(view.pitch_rad-.2)<=1e-12,
        "constrained-root inspection includes native joint addresses and the compiled body's world offset");
    bt::require(std::abs(view.speed_mps-4.)<=1e-12&&view.root_x_dofadr==-1&&pose.attitude_valid,
        "constrained-root inspection exposes the same heading-relative HUD contract");
}
}
int main(){return bt::run([]{camera_contract();replica_contract();articulated_replica_contract();playback_free_contract();playback_singular_contract();playback_constrained_contract();});}
