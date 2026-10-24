#include "startup_fixture.hpp"
#include "model_request.hpp"
#include "terrain_fixture.hpp"
#include "fixture_paths.hpp"

#include "../../support/migration_test.hpp"
#include "../../../src/engine_call.hpp"
#include "../../../src/model/builder.hpp"

#include <cstring>
#include <filesystem>
#include <span>
#include <string>
#include <utility>

namespace spatial_tests {
namespace {
constexpr std::string_view kScalarRootXml=R"(<joint name="root_x" type="slide" axis="1 0 0"/><joint name="root_z" type="slide" axis="0 0 1"/><joint name="root_pitch" type="hinge" axis="0 1 0"/>)";
constexpr std::string_view kConstrained4Xml=R"(<joint name="root_x" type="slide" axis="1 0 0"/><joint name="root_y" type="slide" axis="0 1 0"/><joint name="root_z" type="slide" axis="0 0 1"/><joint name="root_pitch" type="hinge" axis="0 1 0"/>)";
constexpr std::string_view kFree6Xml=R"(<freejoint name="root_free"/>)";
constexpr std::string_view kMixedBallXml=R"(<body name="prefix_scalar" pos="0 0 3"><joint name="prefix_slide" type="slide" axis="0 1 0"/><geom size=".05" mass="1"/></body><body name="prefix_ball" pos="0 0 4"><joint name="unrelated_ball" type="ball"/><geom size=".05" mass="1"/></body>)";
std::string fixture_xml(bike::spatial::RootKind kind,bool mixed,bool rider) {
    std::string xml=R"(<mujoco><compiler angle="radian"/><option gravity="0 0 -9.81" timestep=".0005"/><default><geom contype="0" conaffinity="0"/></default><worldbody>)";
    if(mixed) xml+=kMixedBallXml;
    xml+=R"(<body name="frame" pos="0 0 1">)";
    xml+=kind==bike::spatial::RootKind::free6?kFree6Xml:kind==bike::spatial::RootKind::constrained4?kConstrained4Xml:kScalarRootXml;
    xml+=R"(<geom name="frame_mass" type="box" pos=".1 .02 .03" size=".4 .1 .1" mass="15"/><site name="imu_site_spatial" pos=".1 0 .1"/><body name="steer" pos=".6 0 0">)";
    if(kind==bike::spatial::RootKind::free6) xml+=R"(<joint name="steer_joint" axis="-.3 0 .9539392014169457"/>)";
    xml+=R"(<geom size=".06" mass="2"/><body name="front_wheel"><joint name="front_wheel_spin" axis="0 1 0"/><geom name="geom_front_contact" size=".3" mass="2"/></body></body><body name="rear_hub" pos="-.6 0 0"><geom size=".06" mass="1"/><body name="rear_wheel"><joint name="rear_wheel_spin" axis="0 1 0"/><geom name="geom_rear_contact" size=".3" mass="2"/></body></body><body name="crank"><joint name="crank_spin" axis="0 1 0"/><geom size=".05" mass="1"/><body name="pedal_front" pos=".17 -.1 0"><joint name="pedal_front_spin" axis="0 1 0"/><geom size=".03" mass=".2"/></body><body name="pedal_rear" pos="-.17 .1 0"><joint name="pedal_rear_spin" axis="0 1 0"/><geom size=".03" mass=".2"/></body></body></body>)";
    if(rider) xml+=R"(<body name="rider_pelvis" pos="0 0 1.7"><freejoint name="rider_root_free"/><geom size=".1" mass="10"/></body>)";
    xml+=R"(</worldbody><actuator><motor name="front_brake" joint="front_wheel_spin"/><motor name="rear_brake" joint="rear_wheel_spin"/>)";
    if(kind==bike::spatial::RootKind::free6) xml+=R"(<motor name="steer_torque" joint="steer_joint" ctrllimited="true" ctrlrange="-25 25"/>)";
    xml+="</actuator></mujoco>";
    return xml;
}
RigModel make(bike::spatial::Config cfg,bool mixed,bool rider) {
    RigModel result;
    result.model=bike::model::compile_mjcf(fixture_xml(cfg.root,mixed,rider)).release();
    result.data=engine::make_data(result.model);
    bt::require(result.data!=nullptr,"fixture owns engine state");
    engine::forward(result.model,result.data);
    result.topology=bike::spatial::bind_topology(*result.model,cfg);
    return result;
}
RigModel production_rig(bike::spatial::Config cfg,bool rider) {
    auto request=model_request(cfg.root);
    cfg.mode=bike::spatial::Mode::spatial3d;
    request->spatial=cfg;request->physics->spatial=cfg;
    request->terrain3d=std::make_shared<const bike::terrain::Terrain3dSource>(plane_grid(0.,0.,0.,65,33,.25,.25));
    request->rider.emplace();request->rider->variant=rider?"articulated_planar":"none";
    if(rider) {
        request->physics->drive_mode="articulated_effort";
        request->physics->articulated.pedal_attachment="spindle";
        request->physics->articulated.saddle_attachment="pin";
        request->physics->articulated.grip_attachment="connect";
        request->physics->articulated.shoulder_profile=(std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/
            "examples/research/profiles/spatial-shoulder.json").string();
    }
    RigModel result;
    result.model=bike::model::compile_mjcf(bike::model::generate_mjcf(*request)).release();
    result.data=engine::make_data(result.model);
    bt::require(result.data!=nullptr,"production fixture owns engine state");
    bike::model::populate_render_heightfield(*result.model,bike::terrain::make_render_grid(*request->terrain3d,
        {static_cast<std::uint32_t>(request->field.nrow),static_cast<std::uint32_t>(request->field.ncol)}));
    engine::forward(result.model,result.data);
    result.topology=bike::spatial::bind_topology(*result.model,cfg);
    return result;
}
} // namespace
RigModel::~RigModel() { if(data) mj_deleteData(data);if(model) mj_deleteModel(model); }
RigModel::RigModel(RigModel&& other) noexcept
    :model(std::exchange(other.model,nullptr)),data(std::exchange(other.data,nullptr)),topology(std::move(other.topology)) {}
RigModel& RigModel::operator=(RigModel&& other) noexcept {
    if(this!=&other) {
        if(data) mj_deleteData(data);if(model) mj_deleteModel(model);
        model=std::exchange(other.model,nullptr);data=std::exchange(other.data,nullptr);topology=std::move(other.topology);
    }
    return *this;
}
RigModel constrained_rig(const bike::spatial::Config& config) {
    auto cfg=config;
    if(cfg.root!=bike::spatial::RootKind::planar3) cfg.root=bike::spatial::RootKind::constrained4;
    cfg.steer.controller=bike::spatial::SteerController::off;
    // The explicit planar legacy fixture remains independent of the new
    // production spatial builder; all spatial roots use the real model.
    return cfg.root==bike::spatial::RootKind::planar3?make(cfg,false,false):production_rig(cfg,false);
}
RigModel free_rig(const bike::spatial::Config& config,bool articulated_rider) {
    auto cfg=config;cfg.root=bike::spatial::RootKind::free6;return production_rig(cfg,articulated_rider);
}
RigModel mixed_joint_rig() {
    bike::spatial::Config cfg;cfg.mode=bike::spatial::Mode::spatial3d;cfg.root=bike::spatial::RootKind::free6;
    return make(cfg,true,true);
}
std::vector<std::byte> complete_state_bytes(const RigModel& rig) {
    struct SizeCall {const mjModel* model;mjtSize bytes=0;} size{rig.model};
    engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw) noexcept {
        auto& call=*static_cast<SizeCall*>(raw);
        call.bytes=mj_sizeModel(call.model);
    },&size,error)) engine::throw_failure(error);
    const auto count=engine::state_size(rig.model,mjSTATE_INTEGRATION);
    bt::require(count>=0 && size.bytes>0 && std::cmp_less_equal(size.bytes,512U*1024U*1024U),"complete fixture state is bounded");
    std::vector<mjtNum> state(static_cast<std::size_t>(count));
    engine::get_state(rig.model,rig.data,state.data(),mjSTATE_INTEGRATION);
    const int model_bytes=static_cast<int>(size.bytes);
    std::vector<std::byte> bytes(static_cast<std::size_t>(model_bytes)+state.size()*sizeof(mjtNum));
    struct SaveCall {const mjModel* model;void* bytes;int count;} save{rig.model,bytes.data(),model_bytes};
    if(!engine::invoke([](void* raw) noexcept {
        const auto& call=*static_cast<SaveCall*>(raw);mj_saveModel(call.model,nullptr,call.bytes,call.count);
    },&save,error)) engine::throw_failure(error);
    std::memcpy(std::span(bytes).subspan(static_cast<std::size_t>(model_bytes)).data(),state.data(),state.size()*sizeof(mjtNum));
    return bytes;
}
} // namespace spatial_tests
