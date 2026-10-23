#pragma once

#include "model_request.hpp"
#include "support/model_fixture.hpp"
#include "configuration/model_config.hpp"
#include "io/float_format.hpp"
#include "model/mapping.hpp"
#include "rider/state_codec.hpp"

#include <algorithm>
#include <memory>
#include <string>

namespace spatial_tests {
inline std::unique_ptr<bike::model::BuildRequest> rider_request() {
    auto request=model_request(bike::spatial::RootKind::free6);
    request->rider=bike::configuration::RiderSpecs{};
    request->rider->variant="articulated_planar";
    request->physics->drive_mode="articulated_effort";
    request->physics->articulated.pedal_attachment="spindle";
    request->physics->articulated.saddle_attachment="pin";
    request->physics->articulated.grip_attachment="connect";
    request->physics->spatial=request->spatial;
    return request;
}

inline bike::io::Value rider_controller_document(const bike::model::BuildRequest& request) {
    namespace io=bike::io;
    const auto document=bike::configuration::to_value(*request.physics);
    const auto* articulated=io::find_member(io::as_object(document,"physics"),"articulated");
    bt::require(articulated!=nullptr,"typed articulated configuration is projected");
    auto fields=io::as_object(*articulated,"articulated");
    std::erase_if(fields,[](const auto& member) {
        return member.first=="weld_servo_gain" || member.first=="weld_servo_limit_nm" ||
            member.first=="leg_track_damping_nms_rad" || member.first=="shoulder_profile" ||
            member.first=="shoulder_profile_sha256";
    });
    fields.emplace_back("strength",io::Value{});
    fields.emplace_back("strength_coordinates",io::Value{io::Value::Object{}});
    fields.emplace_back("envelopes",io::Value{io::Value::Object{}});
    if(request.spatial && request.spatial->mode==bike::spatial::Mode::spatial3d) {
        fields.emplace_back("rider.spatial",io::Value{io::Number{"1"}});
        fields.emplace_back("shoulder_profile",bike::model::shoulder_profile_document(request.physics->articulated));
        fields.emplace_back("steer_torque_owner",io::Value{std::string("actuator")});
    }
    return io::Value{std::move(fields)};
}
inline bike::io::Value rider_pose_document(const bike::model::BuildRequest& request) {
    namespace io=bike::io;
    const auto pose=bike::model::geometry_pose(*request.rider,request.bike);
    const auto point=[](const auto& p) {
        io::Value::Array out;
        for(double x:p) out.emplace_back(io::Value{io::Number{io::repr_binary64(x)}});
        return io::Value{std::move(out)};
    };
    return io::Value{io::Value::Object{
        {"saddle",point(pose.saddle)}, {"hip",point(pose.hip)},
        {"shoulder",point(pose.shoulder)}, {"elbow",point(pose.elbow)},
        {"grip",point(pose.grip)}, {"head_center",point(pose.head_center)},
        {"knee_front",point(pose.knee_front)}, {"knee_rear",point(pose.knee_rear)},
        {"ankle_front",point(pose.ankle_front)}, {"ankle_rear",point(pose.ankle_rear)},
        {"pedal_front",point(pose.pedal_front)}, {"pedal_rear",point(pose.pedal_rear)}}};
}

struct RiderRig {
    std::unique_ptr<bike::model::BuildRequest> request;
    bt::ModelFixture rig;
    std::unique_ptr<::rider::SpindleController> controller;
    RiderRig() : request(rider_request()), rig(bike::model::generate_mjcf(*request)) {
        controller=std::make_unique<::rider::SpindleController>(rig.model.get(),
            ::rider::decode_spatial_spindle_pose(rider_pose_document(*request),"config.geometry.pose"),
            ::rider::decode_spatial_spindle_config(rider_controller_document(*request)),request->bike.crank_length/1000.);
        engine::forward(rig.model.get(),rig.data.get());
    }
    void scalar(std::string_view name,double value) {
        const auto [qa,va]=bike::model::joint_address(*rig.model,name);
        (void)va;
        model_access::mutable_buffer(rig.data->qpos,rig.model->nq)[static_cast<std::size_t>(qa)]=value;
    }
};
} // namespace spatial_tests
