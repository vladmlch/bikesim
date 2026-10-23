#include "builder_internal.hpp"
#include "../spatial/frames.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace bike::model::detail {
namespace {
Vec3 parse_position(const std::string& text) {
    std::istringstream input(text);Vec3 result{};
    if(!(input>>result[0]>>result[1]>>result[2])) throw std::invalid_argument("spatial_3d.imu.mount_pos: malformed site position");
    return result;
}
std::string quaternion(const bike::spatial::Quat& q) {
    return g(q[0])+" "+g(q[1])+" "+g(q[2])+" "+g(q[3]);
}
void emit_root(Element& frame,bike::spatial::RootKind kind) {
    for(auto* value:frame.direct("joint")) if(value->get("name").starts_with("root_")) (void)frame.remove(value);
    if(kind==bike::spatial::RootKind::constrained4) {
        joint(frame,"root_x","slide",{{"axis","1 0 0"}});
        joint(frame,"root_y","slide",{{"axis","0 1 0"}});
        joint(frame,"root_z","slide",{{"axis","0 0 1"}});
        joint(frame,"root_pitch","hinge",{{"axis","0 1 0"}});
    } else if(kind==bike::spatial::RootKind::free6) frame.add("freejoint",{{"name","root_free"}});
    else throw std::invalid_argument("spatial_3d.root: spatial model requires constrained4 or free6");
}
void emit_imu(Element& root,Element& frame,const bike::spatial::ImuMountConfig& cfg) {
    Vec3 position{};
    if(const auto* named=std::get_if<std::string>(&cfg.mount_pos)) {
        auto* mount=frame.find("site",*named);
        const auto sites=frame.direct("site");
        if(!mount || std::ranges::find(sites,mount)==sites.end())
            throw std::invalid_argument("spatial_3d.imu.mount_pos: site must belong directly to frame");
        position=parse_position(mount->get("pos","0 0 0"));
    } else position=std::get<Vec3>(cfg.mount_pos);
    const auto& r=cfg.mount_rpy_rad;
    const bike::spatial::Quat qx{std::cos(r[0]/2.),std::sin(r[0]/2.),0.,0.};
    const bike::spatial::Quat qy{std::cos(r[1]/2.),0.,std::sin(r[1]/2.),0.};
    const bike::spatial::Quat qz{std::cos(r[2]/2.),0.,0.,std::sin(r[2]/2.)};
    const auto q=bike::spatial::quat_multiply(bike::spatial::quat_multiply(qz,qy),qx);
    frame.add("site",{{"name","imu_site_spatial"},{"pos",vec17(position)},{"quat",quaternion(q)},{"size","0.006"}});
    auto& sensors=block(root,"sensor");
    sensors.add("accelerometer",{{"name","imu_acc_spatial"},{"site","imu_site_spatial"}});
    sensors.add("gyro",{{"name","imu_gyro_spatial"},{"site","imu_site_spatial"}});
}
} // namespace

void finish_spatial_topology(Element& root,const BuildContext& c) {
    if(!c.spatial() || !c.physical()) throw std::invalid_argument("spatial_3d: physical spatial build required");
    const auto& cfg=*c.request.spatial;
    if(c.physics->tires.backend!="pneumatic_3d") throw std::invalid_argument("tires.backend: spatial model requires pneumatic_3d");
    if(cfg.root==bike::spatial::RootKind::constrained4 && cfg.steer.controller!=bike::spatial::SteerController::off)
        throw std::invalid_argument("spatial_3d.steer.controller: constrained root requires off");
    auto& frame=required(root,"body","frame");emit_root(frame,cfg.root);
    if(cfg.root==bike::spatial::RootKind::free6) {
        auto& steer=required(root,"joint","steer_joint");
        steer.set("stiffness","0");steer.set("damping",g(cfg.steer.damping_nms_rad));
        steer.set("limited","true");steer.set("range",g(-cfg.steer.range_deg)+" "+g(cfg.steer.range_deg));
        // "strength" resolves through the same content-checked shoulder
        // profile as the rider. Its provisional status never qualifies a
        // runtime capability, and a caller's explicit cap does not read it.
        const double cap=cfg.steer.torque_cap_nm?*cfg.steer.torque_cap_nm:
            resolve_shoulder_profile(c.physics->articulated).swivel_strength_nm;
        if(!std::isfinite(cap) || cap<=0.) throw std::invalid_argument("spatial_3d.steer.torque_cap_nm: positive finite cap required");
        block(root,"actuator").add("motor",{{"name","steer_torque"},{"joint","steer_joint"},{"gear","1"},
            {"ctrllimited","true"},{"ctrlrange",g(-cap)+" "+g(cap)}});
    }
    emit_imu(root,frame,cfg.imu);
    // Chain geometry binds the authored centres rather than assuming that
    // every future sprocket keeps its body origin at the axle.
    for(const char* name:{"crank","rear_wheel","cassette","drive_shaft","motor_rotor"})
        if(auto* sprocket=root.find("body",name))
            site(*sprocket,"spatial_chain_center_"+std::string(name),"0 0 0","0.001");
    // Engine owns frame/rider crashes. The physical tyre owns wheel forces;
    // no engine hfield/contact force may duplicate that interaction (C4).
    for(const char* name:{"front_wheel","rear_wheel"}) for(auto* geom:required(root,"body",name).descendants("geom")) {
        geom->set("contype",std::to_string(kSpatialTyreMask));geom->set("conaffinity","0");
    }
    for(const char* name:{"frame","steer","fork_lower","chainstay","seatstay","rocker","shock_yoke","shock_shaft","shock_body"})
        for(auto* geom:required(root,"body",name).direct("geom")) {
            if(geom->get("contype","1")=="0") continue;
            geom->set("contype",std::to_string(kSpatialCrashMask));geom->set("conaffinity",std::to_string(kSpatialCrashMask));
        }
    for(const char* name:{"terrain","catch_plane"}) if(auto* geom=root.find("geom",name)) {
        geom->set("contype",std::to_string(kSpatialCrashMask));geom->set("conaffinity",std::to_string(kSpatialCrashMask));
    }
}
} // namespace bike::model::detail
