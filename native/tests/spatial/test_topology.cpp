#include "support/migration_test.hpp"
#include "support/startup_fixture.hpp"
#include "spatial/topology.hpp"
#include "model/builder.hpp"
#include "model_access.hpp"

#include <string>
#include <string_view>
#include <stdexcept>
#include <limits>

namespace {
void scalar_rider_order() {
    using namespace bike::spatial;
    constexpr auto pitch=R"(<joint name="rider_root_pitch" type="hinge" axis="0 1 0"/>)";
    constexpr auto translations=R"(<joint name="rider_root_x" type="slide" axis="1 0 0"/><joint name="rider_root_z" type="slide" axis="0 0 1"/>)";
    for(const bool pitch_first:{false,true}) {
        std::string xml=R"(<mujoco><default><geom contype="0" conaffinity="0"/></default><worldbody><body name="frame"><joint name="root_x" type="slide" axis="1 0 0"/><joint name="root_z" type="slide" axis="0 0 1"/><joint name="root_pitch" axis="0 1 0"/><geom size=".1" mass="10"/><body name="front_wheel" pos=".6 0 0"><joint name="front_wheel_spin" axis="0 1 0"/><geom size=".3" mass="2"/></body><body name="rear_wheel" pos="-.6 0 0"><joint name="rear_wheel_spin" axis="0 1 0"/><geom size=".3" mass="2"/></body></body><body name="rider_pelvis" pos="0 0 1">)";
        xml+=pitch_first?pitch:translations;
        xml+=pitch_first?translations:pitch;
        xml+=R"(<geom size=".1" mass="10"/></body></worldbody></mujoco>)";
        const auto model=bike::model::compile_mjcf(xml);
        Config config;config.root=RootKind::planar3;
        const auto topology=bind_topology(*model,config);
        bt::require(topology.rider_roots.size()==1,
            "planar rider root is bound regardless of the first joint kind");
        const auto& root=topology.rider_roots.front();
        bt::require(root.kind==RootKind::planar3 &&
            root.body_id==mj_name2id(model.get(),mjOBJ_BODY,"rider_pelvis"),
            "rider descriptor identifies its physical root body");
        const auto qpos=model_access::readonly_buffer(model->jnt_qposadr,model->njnt);
        const auto dofs=model_access::readonly_buffer(model->jnt_dofadr,model->njnt);
        for(const auto& scalar:root.scalar) if(scalar.joint_id>=0)
            bt::require(scalar.qpos_adr==qpos[static_cast<std::size_t>(scalar.joint_id)] &&
                scalar.dof_adr==dofs[static_cast<std::size_t>(scalar.joint_id)],
                "rider root addresses follow the actual scalar joint order");
    }
}
void steering_motor(spatial_tests::RigModel& rig,const bike::spatial::Config& config) {
    const auto motor=static_cast<std::size_t>(rig.topology.steer.actuator_id);
    auto& model=*rig.model;
    auto gear=model_access::mutable_buffer(model.actuator_gear,6*model.nu);
    auto dynamics=model_access::mutable_buffer(model.actuator_dyntype,model.nu);
    auto gains=model_access::mutable_buffer(model.actuator_gaintype,model.nu);
    auto biases=model_access::mutable_buffer(model.actuator_biastype,model.nu);
    auto parameters=model_access::mutable_buffer(model.actuator_gainprm,mjNGAIN*model.nu);
    const auto reject=[&] {
        bt::invalid([&] {
            try { (void)bike::spatial::bind_topology(model,config); }
            catch(const std::invalid_argument& error) {
                bt::require(std::string_view(error.what()).starts_with("topology.steer_torque:"),
                    "non-torque steering motor is rejected at cold topology binding");
                throw;
            }
        });
    };
    const double saved_gear=gear[6*motor],saved_extra=gear[6*motor+1],saved_gain=parameters[mjNGAIN*motor];
    const int saved_dynamics=dynamics[motor],saved_gain_kind=gains[motor],saved_bias=biases[motor];
    gear[6*motor]=2.;reject();gear[6*motor]=saved_gear;
    gear[6*motor+1]=1.;reject();gear[6*motor+1]=saved_extra;
    parameters[mjNGAIN*motor]=2.;reject();parameters[mjNGAIN*motor]=saved_gain;
    gains[motor]=mjGAIN_AFFINE;reject();gains[motor]=saved_gain_kind;
    biases[motor]=mjBIAS_AFFINE;reject();biases[motor]=saved_bias;
    dynamics[motor]=mjDYN_FILTER;reject();dynamics[motor]=saved_dynamics;
    auto control_limited=model_access::mutable_buffer(model.actuator_ctrllimited,model.nu);
    auto force_limited=model_access::mutable_buffer(model.actuator_forcelimited,model.nu);
    auto control_ranges=model_access::mutable_buffer(model.actuator_ctrlrange,2*model.nu);
    auto force_ranges=model_access::mutable_buffer(model.actuator_forcerange,2*model.nu);
    const auto saved_control_limited=control_limited[motor],saved_force_limited=force_limited[motor];
    const std::array saved_control{control_ranges[2*motor],control_ranges[2*motor+1]};
    const std::array saved_force{force_ranges[2*motor],force_ranges[2*motor+1]};
    auto capped=config;capped.steer.torque_cap_nm=10.;
    control_limited[motor]=1;control_ranges[2*motor]=-10.;control_ranges[2*motor+1]=10.;
    force_limited[motor]=1;force_ranges[2*motor]=-10.;force_ranges[2*motor+1]=10.;
    (void)bike::spatial::bind_topology(model,capped);
    control_ranges[2*motor+1]=9.;
    bt::invalid([&]{(void)bike::spatial::bind_topology(model,capped);});
    control_ranges[2*motor+1]=10.;force_ranges[2*motor]=-9.;
    bt::invalid([&]{(void)bike::spatial::bind_topology(model,capped);});
    force_ranges[2*motor]=-10.;control_ranges[2*motor]=std::numeric_limits<double>::quiet_NaN();reject();
    control_ranges[2*motor]=10.;reject();
    control_limited[motor]=saved_control_limited;force_limited[motor]=saved_force_limited;
    for(std::size_t i=0;i<2;++i) {
        control_ranges[2*motor+i]=saved_control[i];force_ranges[2*motor+i]=saved_force[i];
    }
    bt::require(bike::spatial::bind_topology(model,config).fingerprint_hex==rig.topology.fingerprint_hex,
        "valid torque motor remains admissible and retains identity after rejection controls");
}
} // namespace

int main() { return bt::run([] {
    using namespace bike::spatial;
    scalar_rider_order();
    for(const auto kind:{RootKind::planar3,RootKind::constrained4,RootKind::free6}) {
        Config cfg;cfg.root=kind;
        auto rig=kind==RootKind::free6?spatial_tests::free_rig(cfg,true):spatial_tests::constrained_rig(cfg);
        const auto& t=rig.topology;const auto& m=*rig.model;
        const auto types=model_access::readonly_buffer(m.jnt_type,m.njnt);
        const auto qa=model_access::readonly_buffer(m.jnt_qposadr,m.njnt);
        const auto va=model_access::readonly_buffer(m.jnt_dofadr,m.njnt);
        bt::require(t.root.kind==kind,"root kind preserved");
        bt::require(t.nq==m.nq && t.nv==m.nv,"independent engine dimensions");
        if(kind==RootKind::free6) {
            bt::require(types[static_cast<std::size_t>(t.root.free_joint_id)]==mjJNT_FREE,"root must be a free joint");
            bt::require(t.root.free_qpos_adr+7<=m.nq && t.root.free_dof_adr+6<=m.nv,"free spans stay in model");
            bt::require(t.rider_roots.size()==1,"independent rider free root bound");
        } else {
            for(const auto& joint:t.root.scalar) if(joint.joint_id>=0)
                bt::require(joint.qpos_adr==qa[static_cast<std::size_t>(joint.joint_id)] && joint.dof_adr==va[static_cast<std::size_t>(joint.joint_id)],"scalar qpos and dof independently bound");
            bt::require(t.steer.actuator_id==-1,"constrained topology has no steering");
        }
        bt::require(t.fingerprint_hex.size()==64 && t.model_sha256_hex.size()==64,"content identities are complete");
    }
    auto mixed=spatial_tests::mixed_joint_rig();
    bt::require(mixed.topology.root.free_qpos_adr>0 && mixed.topology.root.free_dof_adr>0,"free root is not first");
    bt::require(mixed.topology.root.free_qpos_adr!=mixed.topology.root.free_dof_adr,"preceding ball changes qpos offset");
    Config cfg;cfg.root=RootKind::free6;
    auto plain=spatial_tests::free_rig(cfg,true);
    steering_motor(plain,cfg);
    bt::require(mixed.topology.fingerprint_hex!=plain.topology.fingerprint_hex,"different joint layout changes identity");
    const auto joint=plain.topology.wheels[0].spin.joint_id;
    auto axes=model_access::mutable_buffer(plain.model->jnt_axis,3*plain.model->njnt);
    axes[3*static_cast<std::size_t>(joint)]=1.;
    bt::invalid([&]{(void)bind_topology(*plain.model,cfg);});
    axes[3*static_cast<std::size_t>(joint)]=0.;
    auto parents=model_access::mutable_buffer(plain.model->body_parentid,plain.model->nbody);
    const auto front=static_cast<std::size_t>(plain.topology.wheels[0].wheel_body_id);
    const int front_parent=parents[front];
    parents[front]=plain.topology.root.body_id;
    bt::invalid([&]{(void)bind_topology(*plain.model,cfg);});
    parents[front]=front_parent;
    const auto rear=static_cast<std::size_t>(plain.topology.wheels[1].wheel_body_id);
    const int rear_parent=parents[rear];
    parents[rear]=front_parent;
    bt::invalid([&]{(void)bind_topology(*plain.model,cfg);});
    parents[rear]=rear_parent;
    const auto valid=bind_topology(*plain.model,cfg);
    bt::require(valid.fingerprint_hex==plain.topology.fingerprint_hex,"restored valid steering hierarchy preserves identity");
    model_access::mutable_buffer(plain.model->actuator_trnid,2*plain.model->nu)[2*static_cast<std::size_t>(plain.topology.steer.actuator_id)]=joint;
    bt::invalid([&]{(void)bind_topology(*plain.model,cfg);});
}); }
