#include "topology.hpp"

#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../io/float_format.hpp"
#include "../model_access.hpp"
#include "../model_topology.hpp"
#include "frames.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace bike::spatial {
namespace {
using Value=bike::io::Value;
Value integer(int value) { return Value{bike::io::Number{std::to_string(value)}}; }
[[noreturn]] void invalid(std::string_view field, std::string_view message) {
    throw std::invalid_argument("topology."+std::string(field)+": "+std::string(message));
}
int named(const mjModel& m, mjtObj type, const char* name) {
    const int id=mj_name2id(&m,type,name);
    if (id<0) invalid(name,"missing name");
    return id;
}
void span(int address,int width,mjtSize count,std::string_view name) {
    if (address<0 || width<0 || count<width || address>count-width) invalid(name,"address span escapes model");
}
ScalarJointBinding scalar(const mjModel& m, const char* name,mjtJoint type,const Vec3& axis) {
    const int id=named(m,mjOBJ_JOINT,name);
    const auto index=static_cast<std::size_t>(id);
    const auto types=model_access::readonly_buffer(m.jnt_type,m.njnt);
    const auto axes=model_access::readonly_buffer(m.jnt_axis,3*m.njnt);
    const auto qa=model_access::readonly_buffer(m.jnt_qposadr,m.njnt);
    const auto va=model_access::readonly_buffer(m.jnt_dofadr,m.njnt);
    if (types[index]!=type) invalid(name,"unexpected joint kind");
    for (std::size_t k=0;k<3;++k)
        if (!std::isfinite(axes[3*index+k]) ||
            std::abs(axes[3*index+k]-axis[k])>1e-12) invalid(name,"unexpected joint axis");
    span(qa[index],1,m.nq,name);span(va[index],1,m.nv,name);
    return {.joint_id=id,.qpos_adr=qa[index],.dof_adr=va[index]};
}
RootBinding free_root(const mjModel& m,const char* name) {
    const int id=named(m,mjOBJ_JOINT,name);
    const auto index=static_cast<std::size_t>(id);
    const auto types=model_access::readonly_buffer(m.jnt_type,m.njnt);
    const auto qa=model_access::readonly_buffer(m.jnt_qposadr,m.njnt);
    const auto va=model_access::readonly_buffer(m.jnt_dofadr,m.njnt);
    const auto bodies=model_access::readonly_buffer(m.jnt_bodyid,m.njnt);
    const auto parents=model_access::readonly_buffer(m.body_parentid,m.nbody);
    if (types[index]!=mjJNT_FREE) invalid(name,"expected mjJNT_FREE");
    span(qa[index],7,m.nq,name);span(va[index],6,m.nv,name);
    const int body=bodies[index];
    if (body<=0 || body>=m.nbody || parents[static_cast<std::size_t>(body)]!=0) invalid(name,"free root must belong to a top-level body");
    return {.body_id=body,.kind=RootKind::free6,.free_joint_id=id,
            .free_qpos_adr=qa[index],.free_dof_adr=va[index]};
}
// Same compiled MJB identity as initialization::capture_model_image, kept
// here to avoid a spatial -> initialization -> spatial library cycle.
std::string model_digest(const mjModel& m) {
    struct Size { const mjModel* model=nullptr; mjtSize bytes=0; } size{.model=&m};
    engine::ErrorBuffer error;
    if (!engine::invoke([](void* p) noexcept { auto& c=*static_cast<Size*>(p);c.bytes=mj_sizeModel(c.model); },&size,error)) engine::throw_failure(error);
    if (size.bytes<=0 || std::cmp_greater(size.bytes,512U*1024U*1024U)) invalid("model","compiled image exceeds 512 MiB");
    std::vector<std::byte> bytes(static_cast<std::size_t>(size.bytes));
    struct Save { const mjModel* model=nullptr; void* buffer=nullptr; int bytes=0; } save{.model=&m,.buffer=bytes.data(),.bytes=static_cast<int>(size.bytes)};
    if (!engine::invoke([](void* p) noexcept { const auto& c=*static_cast<Save*>(p);mj_saveModel(c.model,nullptr,c.buffer,c.bytes); },&save,error)) engine::throw_failure(error);
    return bike::io::sha256(bytes);
}
Value scalar_value(const ScalarJointBinding& b) {
    return Value{Value::Object{{"joint",integer(b.joint_id)},{"qpos",integer(b.qpos_adr)},{"dof",integer(b.dof_adr)}}};
}
Value vector_value(const Vec3& value) {
    Value::Array result;
    for(const double component:value) result.push_back(Value{bike::io::Number{bike::io::repr_binary64(component)}});
    return Value{std::move(result)};
}
Value root_value(const RootBinding& root) {
    Value::Array scalars;
    for (const auto& b:root.scalar) scalars.push_back(scalar_value(b));
    return Value{Value::Object{{"body",integer(root.body_id)},{"kind",Value{std::string(to_string(root.kind))}},
        {"free_joint",integer(root.free_joint_id)},{"free_qpos",integer(root.free_qpos_adr)},
        {"free_dof",integer(root.free_dof_adr)},{"scalar",Value{std::move(scalars)}}}};
}
} // namespace

std::string model_fingerprint_hex(const mjModel& model) {
    return model_digest(model);
}

std::string topology_fingerprint_hex(const Topology& t) {
    Value::Array riders,wheels;
    for (const auto& r:t.rider_roots) riders.push_back(root_value(r));
    for (const auto& w:t.wheels) wheels.push_back(Value{Value::Object{{"body",integer(w.wheel_body_id)},
        {"hub",integer(w.hub_body_id)},{"spin",scalar_value(w.spin)},{"axle_axis_hub",vector_value(w.axle_axis_hub)}}});
    const auto document=Value{Value::Object{{"contract",integer(static_cast<int>(kSpatialContract))},
        {"root",root_value(t.root)},{"rider_roots",Value{std::move(riders)}},
        {"wheels",Value{std::move(wheels)}},{"steer",scalar_value(t.steer.joint)},
        {"steer_actuator",integer(t.steer.actuator_id)},{"steer_axis_body",vector_value(t.steer.axis_body)},
        {"imu_site",integer(t.imu_site_id)},
        {"nq",integer(t.nq)},{"nv",integer(t.nv)},{"model_digest",Value{t.model_sha256_hex}}}};
    const auto json=bike::io::canonical_json(document);
    return bike::io::sha256(std::as_bytes(std::span(json.data(),json.size())));
}

Topology bind_topology(const mjModel& m,const Config& cfg) {
    if (m.nq<0 || m.nv<0 || std::cmp_greater(m.nq,std::numeric_limits<int>::max()) ||
        std::cmp_greater(m.nv,std::numeric_limits<int>::max())) invalid("model_dims","unsupported dimensions");
    for(const auto count:{m.nbody,m.njnt,m.nu,m.nactuator,m.nout,m.nsite})
        if(count<0 || std::cmp_greater(count,std::numeric_limits<int>::max()/9)) invalid("model_dims","unsupported topology dimensions");
    // MuJoCo 3.12 keeps nu (controls), nactuator and nout (force outputs)
    // distinct, and indexes ctrl-sized and gear-sized arrays through the
    // per-actuator ctrladr/outadr. Every array below is read by actuator id,
    // so only the one-control, one-output layout is accepted; anything else
    // is rejected before the first actuator array is read.
    if(m.nu!=m.nactuator || m.nout!=m.nactuator)
        invalid("model_dims","actuators must map exactly one control and one force output each");
    {
        const auto control_address=model_access::readonly_buffer(m.actuator_ctrladr,m.nactuator);
        const auto output_address=model_access::readonly_buffer(m.actuator_outadr,m.nactuator);
        for(std::size_t i=0;i<control_address.size();++i)
            if(std::cmp_not_equal(control_address[i],i) || std::cmp_not_equal(output_address[i],i))
                invalid("model_dims","actuator control/output address must equal the actuator id");
    }
    const auto jtype=model_access::readonly_buffer(m.jnt_type,m.njnt);
    const auto jbody=model_access::readonly_buffer(m.jnt_bodyid,m.njnt);
    const auto parents=model_access::readonly_buffer(m.body_parentid,m.nbody);
    const auto counts=model_access::readonly_buffer(m.body_jntnum,m.nbody);
    const auto qa=model_access::readonly_buffer(m.jnt_qposadr,m.njnt);
    const auto va=model_access::readonly_buffer(m.jnt_dofadr,m.njnt);
    const auto axes=model_access::readonly_buffer(m.jnt_axis,3*m.njnt);
    // Even directly constructed/mutated engine models must not exploit an
    // ambiguous name lookup. Compiled MJCF normally enforces this itself.
    for (const auto kind:{mjOBJ_BODY,mjOBJ_JOINT,mjOBJ_ACTUATOR,mjOBJ_SITE}) {
        const auto count=kind==mjOBJ_BODY?m.nbody:kind==mjOBJ_JOINT?m.njnt:kind==mjOBJ_ACTUATOR?m.nactuator:m.nsite;
        std::set<std::string> names;
        for (int id=0;id<count;++id) {
            if (const char* name=mj_id2name(&m,kind,id); name && *name && !names.insert(name).second)
                invalid(name,"duplicate name");
        }
    }
    Topology out;out.nq=static_cast<int>(m.nq);out.nv=static_cast<int>(m.nv);
    out.root.kind=cfg.root;
    if (cfg.root==RootKind::free6) out.root=free_root(m,"root_free");
    else if (cfg.root==RootKind::planar3 || cfg.root==RootKind::constrained4) {
        out.root.scalar[0]=scalar(m,"root_x",mjJNT_SLIDE,{1.,0.,0.});
        if (cfg.root==RootKind::constrained4) out.root.scalar[1]=scalar(m,"root_y",mjJNT_SLIDE,{0.,1.,0.});
        out.root.scalar[2]=scalar(m,"root_z",mjJNT_SLIDE,{0.,0.,1.});
        out.root.scalar[3]=scalar(m,"root_pitch",mjJNT_HINGE,{0.,1.,0.});
        out.root.body_id=jbody[static_cast<std::size_t>(out.root.scalar[0].joint_id)];
        for (const auto& binding:out.root.scalar) if (binding.joint_id>=0 && jbody[static_cast<std::size_t>(binding.joint_id)]!=out.root.body_id)
            invalid("root","scalar coordinates must share one body");
    } else invalid("root_kind","unknown root kind");
    if (out.root.body_id!=named(m,mjOBJ_BODY,"frame")) invalid("root","coordinates must belong to frame");
    const auto root_body=static_cast<std::size_t>(out.root.body_id);
    if(parents[root_body]!=0) invalid("root","frame must be a top-level body");
    const int expected_root_count=cfg.root==RootKind::free6?1:cfg.root==RootKind::planar3?3:4;
    if(counts[root_body]!=expected_root_count) invalid("root","unexpected root joint inventory");
    std::set<int> root_positions,root_velocities;
    for(const auto& binding:out.root.scalar) if(binding.joint_id>=0)
        if(!root_positions.insert(binding.qpos_adr).second || !root_velocities.insert(binding.dof_adr).second)
            invalid("root","overlapping scalar coordinate spans");
    for (std::size_t i=0;i<2;++i) {
        auto& wheel=out.wheels[i];
        wheel.wheel_body_id=named(m,mjOBJ_BODY,i==0?"front_wheel":"rear_wheel");
        wheel.hub_body_id=parents[static_cast<std::size_t>(wheel.wheel_body_id)];
        if (wheel.hub_body_id<=0 || wheel.hub_body_id>=m.nbody) invalid("wheel.hub","wheel requires a moving hub parent");
        int ancestor=wheel.hub_body_id;
        for(mjtSize steps=0;ancestor!=out.root.body_id && ancestor>0 && steps<m.nbody;++steps) {
            model_access::require_id(ancestor,m.nbody,"topology.wheel.ancestor");
            ancestor=parents[static_cast<std::size_t>(ancestor)];
        }
        if(ancestor!=out.root.body_id) invalid("wheel.hub","wheel must belong to the bicycle root");
        wheel.spin=scalar(m,i==0?"front_wheel_spin":"rear_wheel_spin",mjJNT_HINGE,{0.,1.,0.});
        if (jbody[static_cast<std::size_t>(wheel.spin.joint_id)]!=wheel.wheel_body_id) invalid("wheel.spin","joint belongs to another body");
        if (counts[static_cast<std::size_t>(wheel.wheel_body_id)]!=1)
            invalid("wheel.spin","wheel body must have exactly its spin joint");
        const auto quaternions=model_access::readonly_buffer(m.body_quat,4*m.nbody);
        const auto q=quaternions.subspan(4*static_cast<std::size_t>(wheel.wheel_body_id),4);
        const Quat rotation{q[0],q[1],q[2],q[3]};
        const double norm=std::hypot(std::hypot(q[0],q[1]),std::hypot(q[2],q[3]));
        if(!std::isfinite(norm) || std::abs(norm-1.)>1e-12) invalid("wheel.quat","expected unit local orientation");
        wheel.axle_axis_hub=rotate_body_to_world(rotation,{0.,1.,0.});
    }
    const int steer=mj_name2id(&m,mjOBJ_JOINT,"steer_joint"),actuator=mj_name2id(&m,mjOBJ_ACTUATOR,"steer_torque");
    if (cfg.root!=RootKind::free6) {
        if (steer>=0 || actuator>=0) invalid("steer_joint","steering requires a free root");
    } else {
        if (steer<0) invalid("steer_joint","missing name");
        const auto sj=static_cast<std::size_t>(steer);
        if (jtype[sj]!=mjJNT_HINGE) invalid("steer_joint","expected hinge");
        span(qa[sj],1,m.nq,"steer_joint");span(va[sj],1,m.nv,"steer_joint");
        out.steer.joint={.joint_id=steer,.qpos_adr=qa[sj],.dof_adr=va[sj]};
        for (std::size_t k=0;k<3;++k) out.steer.axis_body[k]=axes[3*sj+k];
        const auto length=std::hypot(out.steer.axis_body[0],out.steer.axis_body[1],out.steer.axis_body[2]);
        if (!std::isfinite(length) || std::abs(length-1.)>1e-12) invalid("steer_joint.axis","expected unit axis");
        const int steer_body=jbody[sj];
        if (steer_body<=0 || steer_body>=m.nbody || parents[static_cast<std::size_t>(steer_body)]!=out.root.body_id)
            invalid("steer_joint","must attach front assembly to frame");
        if (counts[static_cast<std::size_t>(steer_body)]!=1)
            invalid("steer_joint","front assembly must have exactly its steering joint");
        for (std::size_t i=0;i<out.wheels.size();++i) {
            int ancestor=out.wheels[i].hub_body_id;
            for (mjtSize steps=0;ancestor!=steer_body && ancestor>0 && steps<m.nbody;++steps) {
                model_access::require_id(ancestor,m.nbody,"topology.steer_joint.ancestor");
                ancestor=parents[static_cast<std::size_t>(ancestor)];
            }
            if ((ancestor==steer_body)!=(i==0))
                invalid("steer_joint", "must steer the front wheel assembly only");
        }
        if (actuator<0) invalid("steer_torque","missing actuator");
        const auto actuator_types=model_access::readonly_buffer(m.actuator_trntype,m.nu);
        const auto targets=model_access::readonly_buffer(m.actuator_trnid,2*m.nu);
        const auto ai=static_cast<std::size_t>(actuator);
        if (actuator_types[ai]!=mjTRN_JOINT || targets[2*ai]!=steer)
            invalid("steer_torque","actuator targets the wrong joint");
        const auto gear=model_access::readonly_buffer(m.actuator_gear,6*m.nu).subspan(6*ai,6);
        const auto dynamics=model_access::readonly_buffer(m.actuator_dyntype,m.nu);
        const auto gains=model_access::readonly_buffer(m.actuator_gaintype,m.nu);
        const auto biases=model_access::readonly_buffer(m.actuator_biastype,m.nu);
        const auto gain_parameters=model_access::readonly_buffer(m.actuator_gainprm,mjNGAIN*m.nu);
        // Commands and accounting are in Nm: a position servo, nonunit gain,
        // or activation filter cannot masquerade as the named torque motor.
        if(gear[0]!=1. || !std::ranges::all_of(gear.subspan(1),[](double x){return x==0.;}) ||
           dynamics[ai]!=mjDYN_NONE || gains[ai]!=mjGAIN_FIXED || biases[ai]!=mjBIAS_NONE ||
           gain_parameters[mjNGAIN*ai]!=1.)
            invalid("steer_torque","expected a pure unit-gain torque motor");
        // Damping and delay change the delivered Nm or its timing, so a
        // damped or delayed motor is not a pure torque motor either.
        const auto damping=model_access::readonly_buffer(m.actuator_damping,m.nactuator);
        const auto damping_poly=model_access::readonly_buffer(m.actuator_dampingpoly,mjNPOLY*m.nactuator)
            .subspan(mjNPOLY*ai,mjNPOLY);
        const auto delay=model_access::readonly_buffer(m.actuator_delay,m.nactuator);
        if(damping[ai]!=0. || delay[ai]!=0. ||
           !std::ranges::all_of(damping_poly,[](double x){return x==0.;}))
            invalid("steer_torque","torque motor must have no damping or delay");
        const auto cap=cfg.steer.torque_cap_nm;
        if(cap && (!std::isfinite(*cap) || *cap<=0.))
            invalid("steer_torque","configured torque cap must be finite and positive");
        const auto check_range=[&](bool limited,std::span<const mjtNum> range) {
            if(!limited)return;
            if(!std::isfinite(range[0]) || !std::isfinite(range[1]) || range[0]>=range[1])
                invalid("steer_torque","enabled actuator limits require finite ordered bounds");
            if(cap && (range[0]>-*cap || range[1]<*cap))
                invalid("steer_torque","actuator limits clip the configured torque cap");
        };
        check_range(model_access::readonly_buffer(m.actuator_ctrllimited,m.nu)[ai]!=0,
            model_access::readonly_buffer(m.actuator_ctrlrange,2*m.nu).subspan(2*ai,2));
        check_range(model_access::readonly_buffer(m.actuator_forcelimited,m.nu)[ai]!=0,
            model_access::readonly_buffer(m.actuator_forcerange,2*m.nu).subspan(2*ai,2));
        out.steer.actuator_id=actuator;
    }
    std::set<int> rider_bodies;
    for (int id=0;id<m.njnt;++id) {
        const char* raw=mj_id2name(&m,mjOBJ_JOINT,id);
        if (!raw || !std::string_view(raw).starts_with("rider_")) continue;
        const auto ji=static_cast<std::size_t>(id);
        const int body=jbody[ji];
        model_access::require_id(body,m.nbody,"topology.rider.body");
        if (parents[static_cast<std::size_t>(body)]!=0 || rider_bodies.contains(body)) continue;
        if (jtype[ji]==mjJNT_FREE) out.rider_roots.push_back(free_root(m,raw));
        else if (jtype[ji]==mjJNT_BALL) {
            // W00 RootBinding has no BALL kind/span. Never mislabel a four-
            // component orientation as a seven-component FREE state.
            invalid(raw,"BALL rider root needs the C10 RootBinding extension");
        } else if (jtype[ji]==mjJNT_SLIDE) {
            RootBinding root;root.body_id=body;
            root.scalar[0]=scalar(m,"rider_root_x",mjJNT_SLIDE,{1.,0.,0.});
            root.scalar[2]=scalar(m,"rider_root_z",mjJNT_SLIDE,{0.,0.,1.});
            root.scalar[3]=scalar(m,"rider_root_pitch",mjJNT_HINGE,{0.,1.,0.});
            for (const auto& b:root.scalar) if (b.joint_id>=0 && jbody[static_cast<std::size_t>(b.joint_id)]!=body) invalid("rider_roots","scalar coordinates must share one body");
            if(counts[static_cast<std::size_t>(body)]!=3)
                invalid("rider_roots","unexpected planar rider root joint inventory");
            out.rider_roots.push_back(root);
        } else continue;
        // A leading scalar pitch hinge is not itself a root descriptor. Do
        // not mark its body consumed until the complete root has been bound.
        rider_bodies.insert(body);
    }
    out.imu_site_id=mj_name2id(&m,mjOBJ_SITE,"imu_site_spatial");
    const auto site_bodies=model_access::readonly_buffer(m.site_bodyid,m.nsite);
    if (out.imu_site_id>=0 && site_bodies[static_cast<std::size_t>(out.imu_site_id)]!=out.root.body_id) invalid("imu_site_spatial","must belong to frame");
    out.model_sha256_hex=model_fingerprint_hex(m);out.fingerprint_hex=topology_fingerprint_hex(out);
    return out;
}

BodyPose body_pose(const mjModel& model,const mjData& data,const RootBinding& root) noexcept {
    BodyPose pose;
    // The frozen API is noexcept: validated model/binding identity and current position and
    // velocity stages (xpos/xquat, subtree_com, cvel) are explicit caller preconditions.
    const std::span<const mjtNum> positions=std::views::counted(data.xpos,3*model.nbody);
    const std::span<const mjtNum> quaternions=std::views::counted(data.xquat,4*model.nbody);
    const auto body=static_cast<std::size_t>(root.body_id);
    for (std::size_t k=0;k<3;++k) pose.position_world_m[k]=positions[3*body+k];
    for (std::size_t k=0;k<4;++k) pose.quat_world_from_body_wxyz[k]=quaternions[4*body+k];
    // XBODY is the regular body frame at xpos; BODY is the inertial frame
    // at xipos. The returned velocity must share this pose's origin.
    std::array<mjtNum,6> velocity{};mj_objectVelocity(&model,&data,mjOBJ_XBODY,root.body_id,velocity.data(),0);
    for (std::size_t k=0;k<3;++k) {
        pose.angular_velocity_world_rad_s[k]=velocity[k];
        pose.linear_velocity_world_mps[k]=velocity[3+k];
    }
    return pose;
}
} // namespace bike::spatial
