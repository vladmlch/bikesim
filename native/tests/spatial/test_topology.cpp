#include "support/migration_test.hpp"
#include "support/startup_fixture.hpp"
#include "spatial/topology.hpp"
#include "model_access.hpp"

int main() { return bt::run([] {
    using namespace bike::spatial;
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
