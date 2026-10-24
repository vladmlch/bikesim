#include "support/rider_fixture.hpp"
#include "configuration/load.hpp"
#include "io/json.hpp"
#include "model/mapping.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/spatial_support.hpp"
#include "rider/support_geometry.hpp"
#include "spatial/frames.hpp"
#include "spatial/topology.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>

namespace {
namespace io=bike::io;
namespace sp=bike::spatial;
io::Value& member(io::Value& value,std::string_view key) {
    for(auto& [name,item]:std::get<io::Value::Object>(value.data)) if(name==key) return item;
    throw std::invalid_argument("missing test member");
}
void near3(const sp::Vec3& a,const sp::Vec3& b,double epsilon,const char* message) {
    for(std::size_t i=0;i<3;++i) bt::require(std::abs(a[i]-b[i])<=epsilon,message);
}
void profile_and_frames() {
    bike::configuration::ArticulatedConfig config;
    const auto document=bike::model::shoulder_profile_document(config);
    const auto profile=bike::rider::read_shoulder_profile(document);
    bt::require(profile.schema==1 && profile.swivel_range_rad==.6 && profile.swivel_strength_nm==25.,"authored provisional shoulder profile");
    bt::require(profile.provenance_status=="provisional" && profile.sha256_hex.size()==64,"content identity does not promote qualification");
    auto wrong=document;member(wrong,"swivel_range_rad")=io::Value{io::Number{"-0.1"}};
    bt::invalid([&]{(void)bike::rider::read_shoulder_profile(wrong);});
    wrong=document;member(wrong,"sha256_hex")=io::Value{std::string(64,'0')};
    bt::invalid([&]{(void)bike::rider::read_shoulder_profile(wrong);});
    wrong=document;std::get<io::Value::Object>(wrong.data).emplace_back("unreviewed",io::Value{true});
    bt::invalid([&]{(void)bike::rider::read_shoulder_profile(wrong);});

    const sp::Quat yaw{std::cos(.71/2.),0.,0.,std::sin(.71/2.)};
    const sp::Quat roll{std::cos(.43/2.),std::sin(.43/2.),0.,0.};
    sp::BodyPose pose;pose.position_world_m={3.,-2.,1.};
    pose.quat_world_from_body_wxyz=sp::quat_multiply(yaw,roll);
    const auto frame=bike::rider::support_frame(pose,{0.,0.,1.});
    bt::require(frame.valid,"finite unit body frame accepted");
    near3(frame.normal_world,sp::rotate_body_to_world(pose.quat_world_from_body_wxyz,{0.,0.,1.}),1e-14,"support normal rotates with body");
    near3(frame.tangent_x_world,sp::rotate_body_to_world(pose.quat_world_from_body_wxyz,{1.,0.,0.}),1e-14,"first tangent rotates with body");
    near3(frame.tangent_y_world,sp::rotate_body_to_world(pose.quat_world_from_body_wxyz,{0.,1.,0.}),1e-14,"second tangent closes right-handed frame");
    bt::require(bike::rider::support_frame(pose,{1.,0.,0.}).valid,"normal parallel to body X uses deterministic Y fallback");
    bt::require(!bike::rider::support_frame(pose,{0.,0.,0.}).valid,"zero normal rejected");
    pose.quat_world_from_body_wxyz={0.,0.,0.,0.};
    bt::require(!bike::rider::support_frame(pose,{0.,0.,1.}).valid,"zero quaternion rejected");
    pose.quat_world_from_body_wxyz=sp::quat_multiply(yaw,roll);
    const sp::Vec3 half{.1,.06,.01};
    const auto local_goal=::rider::project_spatial_sole_goal({},half,.02,.02,.003,.01);
    const auto rotated_goal=::rider::project_spatial_sole_goal(pose,half,.02,.02,.003,.01);
    auto expected=sp::rotate_body_to_world(pose.quat_world_from_body_wxyz,local_goal.goal);
    for(std::size_t i=0;i<3;++i) expected[i]+=pose.position_world_m[i];
    near3(rotated_goal.goal,expected,1e-13,"sole goal covaries under common yaw/roll and translation");
    const auto clipped=::rider::project_spatial_sole_goal(pose,half,.02,.02,.003,.2);
    bt::require(clipped.diagnostic.saturated && clipped.diagnostic.applied_shear_m==.08,"unreachable sole footprint is reported");
    const auto local_contact=::rider::spatial_box_pad_contact({.12,.08,.03},.01,{},half);
    auto center=sp::rotate_body_to_world(pose.quat_world_from_body_wxyz,{.12,.08,.03});
    for(std::size_t i=0;i<3;++i) center[i]+=pose.position_world_m[i];
    const auto rotated_contact=::rider::spatial_box_pad_contact(center,.01,pose,half);
    bt::require(std::abs(local_contact.gap_m-rotated_contact.gap_m)<1e-13,"full XYZ corner distance is covariant");
    const std::array<double,6> wrench{0.,30.,40.,0.,0.,0.};
    const auto sample=::rider::decompose_spatial_wrench(wrench,bike::rider::support_frame({}, {0.,0.,1.}),"foot",false,.02,0.,std::nullopt);
    bt::require(sample.normal_n==40. && sample.tangent_n==30.,"lateral shear is retained in spatial attachment budget");
}
void model_and_mass(spatial_tests::RiderRig& source) {
    auto planar=std::make_unique<bike::model::BuildRequest>(*source.request);
    planar->spatial.reset();planar->terrain3d.reset();planar->physics->spatial.reset();planar->physics->tires.backend="pneumatic";
    const auto old_bytes=bike::model::generate_mjcf(*planar);
    planar->spatial=sp::Config{};
    bt::require(old_bytes==bike::model::generate_mjcf(*planar),"planar model bytes unchanged by explicit planar extension");
    bt::ModelFixture old(old_bytes);
    const auto& current=*source.rig.model;
    double old_mass=0.,new_mass=0.;
    for(int id=1;id<current.nbody;++id) {
        const char* name=mj_id2name(&current,mjOBJ_BODY,id);
        if(name==nullptr || !std::string_view(name).starts_with("rider_")) continue;
        const auto other=static_cast<std::size_t>(bike::model::required_id(*old.model,mjOBJ_BODY,name));
        const auto here=static_cast<std::size_t>(id);
        const auto a=model_access::readonly_buffer(current.body_mass,current.nbody);
        const auto b=model_access::readonly_buffer(old.model->body_mass,old.model->nbody);
        old_mass+=b[other];new_mass+=a[here];
        bt::require(std::abs(a[here]-b[other])<=1e-9,"shoulder swivel preserves segment mass");
        const auto ia=model_access::readonly_buffer(current.body_inertia,3*current.nbody);
        const auto ib=model_access::readonly_buffer(old.model->body_inertia,3*old.model->nbody);
        const auto pa=model_access::readonly_buffer(current.body_ipos,3*current.nbody);
        const auto pb=model_access::readonly_buffer(old.model->body_ipos,3*old.model->nbody);
        for(std::size_t i=0;i<3;++i) {
            bt::require(std::abs(ia[3*here+i]-ib[3*other+i])<=1e-9,"shoulder swivel preserves principal segment inertia");
            bt::require(std::abs(pa[3*here+i]-pb[3*other+i])<=1e-9,"segment centre of mass is preserved");
        }
    }
    bt::require(std::abs(old_mass-new_mass)<=1e-9,"spatial rider mass budget equals planar");
    const auto topology=sp::bind_topology(current,*source.request->spatial);
    bt::require(topology.rider_roots.size()==1 && topology.rider_roots.front().kind==sp::RootKind::free6,"pelvis is an independent free root");
    bt::require(current.nq>current.nv,"free roots split position and velocity widths");
    for(const char* name:{"shoulder_swivel_l","shoulder_swivel_r"}) {
        const auto [qa,va]=bike::model::joint_address(current,name);
        bt::require(qa!=va && qa<current.nq && va<current.nv,"swivel addresses use independent nq/nv lanes");
    }
    for(const char* name:{"connect_grip_left","connect_grip_right"}) {
        const auto eq=static_cast<std::size_t>(bike::model::required_id(current,mjOBJ_EQUALITY,name));
        bt::require(model_access::readonly_buffer(current.eq_type,current.neq)[eq]==mjEQ_CONNECT &&
            model_access::readonly_buffer(current.eq_objtype,current.neq)[eq]==mjOBJ_SITE,"grips constrain position between sites, without orientation weld");
    }
}
void rounded_box_contact() {
    sp::BodyPose pose;pose.position_world_m={3.,-2.,1.};
    const double component=std::sqrt((1.+5e-9)/2.);
    pose.quat_world_from_body_wxyz={component,component,0.,0.};
    const auto raw=pose.quat_world_from_body_wxyz;
    // Analytic +90 degree X rotation: (x,y,z) -> (x,-z,y).
    // Local centre (0.03,0.02,0.04) is 0.03 m above the box top.
    const auto rounded=::rider::spatial_box_pad_contact({3.03,-2.04,1.02},.01,pose,{.1,.06,.01});
    near3(rounded.point_m,{3.03,-2.01,1.02},1e-13,
        "accepted quaternion roundoff preserves the physical contact point");
    near3(rounded.normal,{0.,-1.,0.},1e-14,
        "distance and normal use the same normalized support rotation");
    bt::require(std::abs(rounded.gap_m-.02)<1e-13,
        "accepted quaternion roundoff does not create artificial contact penetration");
    bt::require(pose.quat_world_from_body_wxyz==raw,"contact geometry preserves raw integration quaternion");
    pose.quat_world_from_body_wxyz={std::sqrt(.5),std::sqrt(.5),0.,0.};
    const auto unit=::rider::spatial_box_pad_contact({3.03,-2.04,1.02},.01,pose,{.1,.06,.01});
    near3(rounded.point_m,unit.point_m,1e-13,"rounded and unit rotations share the same geometry");
    bt::require(std::abs(rounded.gap_m-unit.gap_m)<1e-13,"rounded and unit rotations share the same gap");
}
void motor_semantics(spatial_tests::RiderRig& source) {
    auto& model=*source.rig.model;
    const auto motor=static_cast<std::size_t>(bike::model::required_id(model,mjOBJ_ACTUATOR,"act_shoulder_swivel_l"));
    auto dynamics=model_access::mutable_buffer(model.actuator_dyntype,model.nu);
    auto gains=model_access::mutable_buffer(model.actuator_gaintype,model.nu);
    auto biases=model_access::mutable_buffer(model.actuator_biastype,model.nu);
    auto parameters=model_access::mutable_buffer(model.actuator_gainprm,mjNGAIN*model.nu);
    const auto pose=::rider::decode_spatial_spindle_pose(spatial_tests::rider_pose_document(*source.request),"pose");
    const auto construct=[&] {
        const auto controller=std::make_unique<::rider::SpindleController>(
            &model,pose,source.controller->config(),source.request->bike.crank_length/1000.);
        bt::require(controller!=nullptr,"valid spatial motor owns a controller");
    };
    const auto reject=[&] {
        bt::invalid([&] {
            try { construct(); }
            catch(const std::invalid_argument& error) {
                bt::require(std::string_view(error.what()).starts_with("rider.spatial.shoulder_swivel_l:"),
                    "non-motor rejection occurs at the spatial actuator contract");
                throw;
            }
        });
    };
    const int saved_dynamics=dynamics[motor],saved_gain=gains[motor],saved_bias=biases[motor];
    const double saved_parameter=parameters[mjNGAIN*motor];
    parameters[mjNGAIN*motor]=2.;reject();parameters[mjNGAIN*motor]=saved_parameter;
    gains[motor]=mjGAIN_AFFINE;reject();gains[motor]=saved_gain;
    biases[motor]=mjBIAS_AFFINE;reject();biases[motor]=saved_bias;
    dynamics[motor]=mjDYN_FILTER;reject();dynamics[motor]=saved_dynamics;
    construct(); // the original pure unit motor remains admissible after every rejection
}
void attachment_frame_codec() {
    ::rider::AttachmentRaw raw;
    raw.rider_jac={.rows=6,.columns=3,.values=std::vector<double>(18,0.)};
    for(std::size_t i=0;i<3;++i) raw.rider_jac.values[3*i+i]=1.;
    raw.bike_jac=raw.rider_jac;
    raw.rider_qfrc={1.,30.,40.};raw.bike_qfrc={-1.,-30.,-40.};
    raw.observable=true;raw.normal={0.,0.,1.};raw.kind="foot";
    raw.half_patch_m=.02;raw.spatial_frame=bike::rider::support_frame({}, {0.,0.,1.});
    const auto encoded=::rider::encode_attachment_entry(writers::RiderAttachmentEntry{raw});
    const auto decoded=::rider::decode_attachment_entry(encoded,"state.attachment");
    bt::require(io::canonical_json(::rider::encode_attachment_entry(decoded))==io::canonical_json(encoded),
        "spatial support basis and full six-row Jacobians survive attachment checkpoint");
    // A quaternion inside the producer's normalization tolerance must
    // produce the unit basis required by its own checkpoint reader.
    sp::BodyPose rounded_pose;
    const double component=std::sqrt((1.+5e-9)/2.);
    rounded_pose.quat_world_from_body_wxyz={component,component,0.,0.};
    const auto quaternion_before=rounded_pose.quat_world_from_body_wxyz;
    raw.spatial_frame=bike::rider::support_frame(rounded_pose,{0.,0.,1.});
    bt::require(raw.spatial_frame->valid,"accepted pose roundoff produces a valid support frame");
    near3(raw.spatial_frame->normal_world,{0.,-1.,0.},1e-14,
        "accepted quaternion roundoff does not scale the normal");
    bt::require(rounded_pose.quat_world_from_body_wxyz==quaternion_before,
        "support projection preserves the raw integration quaternion");
    const auto rounded=::rider::encode_attachment_entry(writers::RiderAttachmentEntry{raw});
    bt::require(io::canonical_json(::rider::encode_attachment_entry(
        ::rider::decode_attachment_entry(rounded,"state.attachment")))==io::canonical_json(rounded),
        "every accepted rounded support frame survives its checkpoint codec");
    auto invalid=encoded;
    auto& basis=member(invalid,"spatial_frame");
    member(basis,"tangent_y_world")=member(basis,"tangent_x_world");
    bt::invalid([&]{(void)::rider::decode_attachment_entry(invalid,"state.attachment");});
    invalid=encoded;
    std::get<io::Value::Array>(member(invalid,"rider_jac").data).pop_back();
    bt::invalid([&]{(void)::rider::decode_attachment_entry(invalid,"state.attachment");});
    raw.spatial_frame.reset();
    const auto planar=::rider::encode_attachment_entry(writers::RiderAttachmentEntry{raw});
    bt::require(io::find_member(io::as_object(planar,"attachment"),"spatial_frame")==nullptr,
        "legacy attachment wire carries no spatial extension");
}
void state_and_ownership(spatial_tests::RiderRig& source) {
    auto& model=*source.rig.model;auto* data=source.rig.data.get();
    source.scalar("shoulder_swivel_l",.2);source.scalar("shoulder_swivel_r",-.1);
    const auto left=bike::model::joint_address(model,"shoulder_swivel_l");
    const auto right=bike::model::joint_address(model,"shoulder_swivel_r");
    auto velocity=model_access::mutable_buffer(data->qvel,model.nv);
    velocity[static_cast<std::size_t>(left.second)]=.35;velocity[static_cast<std::size_t>(right.second)]=-.25;
    const auto state=source.controller->state(data);
    const auto encoded=::rider::encode_spindle_state(state);
    auto decoded=::rider::decode_spindle_state(encoded,"state.rider_controller");
    source.controller->validate_spatial_lanes(decoded,model_access::readonly_buffer(data->qpos,model.nq),velocity);
    bt::require(io::canonical_json(::rider::encode_spindle_state(decoded))==io::canonical_json(encoded),"spatial controller and shoulder q/qd roundtrip exactly");
    auto wrong=decoded;wrong.spatial->arm_branch*=-1;
    const auto before=io::canonical_json(::rider::encode_spindle_state(source.controller->state(data)));
    bt::invalid([&]{source.controller->restore(wrong);});
    bt::require(before==io::canonical_json(::rider::encode_spindle_state(source.controller->state(data))),"arm branch flip rejection is atomic");
    wrong=decoded;wrong.spatial->shoulder_profile_sha256=std::string(64,'0');
    bt::invalid([&]{source.controller->restore(wrong);});
    wrong=decoded;wrong.spatial->shoulders[0].q+=.01;
    bt::invalid([&]{source.controller->validate_spatial_lanes(wrong,model_access::readonly_buffer(data->qpos,model.nq),velocity);});
    wrong=decoded;wrong.spatial.reset();bt::invalid([&]{source.controller->restore(wrong);});
    auto bad_wire=encoded;std::get<io::Value::Array>(member(bad_wire,"active_state").data).pop_back();
    bt::invalid([&]{(void)::rider::decode_spindle_state(bad_wire,"state.rider_controller");});
    ::rider::SpindleController::State planar;
    planar.active_state.assign(9,0.);planar.pedal_recovery={{"front",{}},{"rear",{}}};
    const auto planar_wire=::rider::encode_spindle_state(planar);
    bt::require(io::find_member(io::as_object(planar_wire,"planar state"),"rider.spatial")==nullptr,"legacy state emits no extension key");
    bt::require(io::canonical_json(::rider::encode_spindle_state(::rider::decode_spindle_state(planar_wire,"state")))==io::canonical_json(planar_wire),"planar state bytes retain the exact old key set");

    const int steer=bike::model::required_id(model,mjOBJ_ACTUATOR,"steer_torque");
    auto ctrl=model_access::mutable_buffer(data->ctrl,model.nu);ctrl[static_cast<std::size_t>(steer)]=8.;
    ::rider::RiderCommand command;command.enabled=false;
    auto effort=source.controller->compute(data,command);
    source.controller->write(data,effort);
    bt::require(ctrl[static_cast<std::size_t>(steer)]==8.,"rider has no second steering torque owner");
    const std::vector<double> old_ctrl(ctrl.begin(),ctrl.end());
    effort.back().second=std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&]{source.controller->write(data,effort);});
    bt::require(std::ranges::equal(ctrl,old_ctrl),"invalid spatial command rejects before any actuator write");

    auto parents=model_access::mutable_buffer(model.body_parentid,model.nbody);
    const auto pelvis=static_cast<std::size_t>(bike::model::required_id(model,mjOBJ_BODY,"rider_pelvis"));
    const int previous=parents[pelvis];parents[pelvis]=static_cast<int>(pelvis);
    bt::invalid([&]{::rider::SpindleController invalid_model(&model,
        ::rider::decode_spatial_spindle_pose(spatial_tests::rider_pose_document(*source.request),"pose"),
        source.controller->config(),source.request->bike.crank_length/1000.);});
    parents[pelvis]=previous;
}
}
int main() {return bt::run([] {
    profile_and_frames();rounded_box_contact();attachment_frame_codec();auto rig=std::make_unique<spatial_tests::RiderRig>();
    model_and_mass(*rig);motor_semantics(*rig);state_and_ownership(*rig);
});}
