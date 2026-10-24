#include "writers/pneumatic_3d_tire.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include "spatial/frames.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace bike::pneumatic3d {
namespace {
using spatial::Vec3;
Vec3 cross(const Vec3& a,const Vec3& b) noexcept {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
double dot(const Vec3& a,const Vec3& b) noexcept{return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
}
Pneumatic3dTireWriter::Pneumatic3dTireWriter(const mjModel& m,const spatial::Topology& topology,
    Pneumatic3dSpec spec,const terrain::Terrain3dSource& terrain,TyreStateIdentity3d identity)
    :model_(&m),topology_(topology),kernels_(std::move(spec),terrain.query_view(),identity),
     jacp_(3*static_cast<std::size_t>(m.nv)),jacr_(3*static_cast<std::size_t>(m.nv)) {
    if(topology.nq!=m.nq || topology.nv!=m.nv || topology.fingerprint_hex!=identity.topology_fingerprint_hex ||
       spatial::topology_fingerprint_hex(topology)!=identity.topology_fingerprint_hex ||
       spatial::model_fingerprint_hex(m)!=topology.model_sha256_hex ||
       terrain.fingerprint_hex()!=identity.terrain_sha256_hex)
        throw std::invalid_argument("pneumatic3d.writer.identity: model/topology/terrain mismatch");
    const auto geom_body=model_access::readonly_buffer(m.geom_bodyid,m.ngeom,"geom_bodyid");
    const auto geom_type=model_access::readonly_buffer(m.geom_type,m.ngeom,"geom_type");
    const auto contype=model_access::readonly_buffer(m.geom_contype,m.ngeom,"geom_contype");
    const auto conaffinity=model_access::readonly_buffer(m.geom_conaffinity,m.ngeom,"geom_conaffinity");
    for(const auto& wheel:topology.wheels) {
        if(wheel.wheel_body_id<0 || wheel.wheel_body_id>=m.nbody || wheel.hub_body_id<0 ||
           wheel.hub_body_id>=m.nbody || wheel.spin.qpos_adr<0 || wheel.spin.qpos_adr>=m.nq ||
           wheel.spin.dof_adr<0 || wheel.spin.dof_adr>=m.nv)
            throw std::invalid_argument("pneumatic3d.writer.topology: invalid wheel binding");
        for(std::size_t g=0;g<geom_body.size();++g)if(geom_body[g]==wheel.wheel_body_id)
            for(std::size_t h=0;h<geom_type.size();++h)if(geom_type[h]==mjGEOM_HFIELD || geom_type[h]==mjGEOM_PLANE)
                if((contype[g]&conaffinity[h]) || (contype[h]&conaffinity[g]))
                    throw std::invalid_argument("pneumatic3d.writer: engine_contact_enabled");
    }
    point_velocity_.resize(2*(static_cast<std::size_t>(identity.patch_capacity)+1));
}
CoreStatus Pneumatic3dTireWriter::try_compute_into(const mjData* d,double dt,bool advance,
    std::span<double> qfrc) noexcept {
    if(!d || !model_ || qfrc.size()!=static_cast<std::size_t>(model_->nv) || !std::isfinite(dt)||dt<0. ||
       !std::isfinite(d->time))return CoreStatus::invalid_input;
    if(advance && dt>0.) {
        if(state().accepted_interval==std::numeric_limits<std::int64_t>::max())return CoreStatus::invalid_input;
        // W00 exposes no writable clock in this writer. The owned stepper
        // uses a fixed dt and an integer time origin; pin that contract rather
        // than accepting a duplicate advancement or inventing hidden state.
        const auto slot=d->time/dt;
        if(!std::isfinite(slot)||slot<0.||slot>=static_cast<double>(std::numeric_limits<std::int64_t>::max()))
            return CoreStatus::invalid_input;
        const auto interval=static_cast<std::int64_t>(std::llround(slot));
        if(std::abs(slot-static_cast<double>(interval))>1e-6 || interval!=state().accepted_interval+1)
            return CoreStatus::invalid_input;
    }
    std::array<WheelKinematics3d,2> kin{};
    struct KinematicsCall {
        const mjModel* m;const mjData* d;const spatial::Topology* topology;
        std::array<WheelKinematics3d,2>* output;
    } call{model_,d,&topology_,&kin};
    engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw) noexcept {
        auto& c=*static_cast<KinematicsCall*>(raw);
        // Model/data pairing and immutable engine extents belong to the
        // owning stepper. These bounded views preserve the hot noexcept path.
        const std::span<const mjtNum> xpos=std::views::counted(
            c.d->xpos,static_cast<std::ptrdiff_t>(3*c.m->nbody));
        const std::span<const mjtNum> xquat=std::views::counted(
            c.d->xquat,static_cast<std::ptrdiff_t>(4*c.m->nbody));
        const std::span<const mjtNum> xipos=std::views::counted(
            c.d->xipos,static_cast<std::ptrdiff_t>(3*c.m->nbody));
        const std::span<const mjtNum> qpos=std::views::counted(
            c.d->qpos,static_cast<std::ptrdiff_t>(c.m->nq));
        for(std::size_t w=0;w<2;++w) {
            const auto& b=c.topology->wheels[w];auto& k=(*c.output)[w];
            const auto wheel_body=static_cast<std::size_t>(b.wheel_body_id);
            const auto hub_body=static_cast<std::size_t>(b.hub_body_id);
            for(std::size_t a=0;a<3;++a)k.hub.position_world_m[a]=xpos[3*wheel_body+a];
            for(std::size_t a=0;a<4;++a)k.hub.quat_world_from_body_wxyz[a]=xquat[4*hub_body+a];
            std::array<mjtNum,6> hub_v{},wheel_v{};
            mj_objectVelocity(c.m,c.d,mjOBJ_BODY,b.hub_body_id,hub_v.data(),0);
            mj_objectVelocity(c.m,c.d,mjOBJ_BODY,b.wheel_body_id,wheel_v.data(),0);
            Vec3 child{},parent{};
            for(std::size_t a=0;a<3;++a) {
                child[a]=wheel_v[a];parent[a]=hub_v[a];
                k.hub.angular_velocity_world_rad_s[a]=hub_v[a];
                // Object velocity is about the body COM. Shift to wheel hub.
                k.hub.linear_velocity_world_mps[a]=wheel_v[3+a];
            }
            Vec3 lever{};for(std::size_t a=0;a<3;++a)lever[a]=k.hub.position_world_m[a]-xipos[3*wheel_body+a];
            const auto correction=cross(child,lever);
            for(std::size_t a=0;a<3;++a)k.hub.linear_velocity_world_mps[a]+=correction[a];
            k.axle_world=spatial::rotate_body_to_world(k.hub.quat_world_from_body_wxyz,b.axle_axis_hub);
            k.relative_spin_rad_s=spatial::axis_rate(child,parent,k.axle_world);
            k.wheel_phase_rad=qpos[static_cast<std::size_t>(b.spin.qpos_adr)];
        }
    },&call,error))return CoreStatus::engine_failure;
    auto status=kernels_.evaluate(kin,{},dt,false);
    if(status!=CoreStatus::ok)return status;
    std::array<double,2> moments{};
    struct ForceCall {
        const mjModel* m;const mjData* d;const spatial::Topology* topology;
        const Pneumatic3dModel* kernels;const std::array<WheelKinematics3d,2>* kin;
        std::span<double> jp;std::span<double> jr;std::span<double> output;std::array<double,2>* moments;
        bool finite=true;
    } force{model_,d,&topology_,&kernels_,&kin,jacp_,jacr_,qfrc,&moments};
    if(!engine::invoke([](void* raw) noexcept {
        auto& c=*static_cast<ForceCall*>(raw);
        for(std::size_t w=0;w<2;++w)for(const auto& patch:c.kernels->wrenches(w))if(patch.valid) {
            mj_jac(c.m,c.d,c.jp.data(),c.jr.data(),patch.point_world_m.data(),c.topology->wheels[w].wheel_body_id);
            const auto nv=c.output.size();
            for(std::size_t j=0;j<nv;++j) {
                double value=0.;for(std::size_t a=0;a<3;++a)
                    value+=c.jp[a*nv+j]*patch.force_world_n[a]+c.jr[a*nv+j]*patch.couple_world_nm[a];
                c.output[j]+=value;
                c.finite=c.finite&&std::isfinite(value)&&std::isfinite(c.output[j]);
            }
            Vec3 lever{};for(std::size_t a=0;a<3;++a)lever[a]=patch.point_world_m[a]-(*c.kin)[w].hub.position_world_m[a];
            auto moment=cross(lever,patch.force_world_n);for(std::size_t a=0;a<3;++a)moment[a]+=patch.couple_world_nm[a];
            (*c.moments)[w]+=dot((*c.kin)[w].axle_world,moment);
            c.finite=c.finite&&std::isfinite((*c.moments)[w]);
        }
    },&force,error))return CoreStatus::engine_failure;
    if(!force.finite)return CoreStatus::invalid_geometry;
    // Publish material only after every engine query and both force mappings
    // succeeded. The caller discards qfrc if any status is non-ok.
    if(advance && dt>0.) {
        status=kernels_.evaluate(kin,{},dt,true);
        if(status!=CoreStatus::ok)return status;
    }
    axle_moment_=moments;return CoreStatus::ok;
}
const TyreState3d& Pneumatic3dTireWriter::state() const noexcept{return kernels_.committed_state();}
std::vector<std::byte> Pneumatic3dTireWriter::state_bytes() const{return encode_tyre_state(state());}
CoreStatus Pneumatic3dTireWriter::restore(const TyreState3d& s) noexcept {
    const auto result=kernels_.restore(s);if(result==CoreStatus::ok)axle_moment_.fill(0.);return result;
}
std::span<const PatchWrench> Pneumatic3dTireWriter::wrenches(std::size_t w) const noexcept{return kernels_.wrenches(w);}
double Pneumatic3dTireWriter::axle_moment_nm(std::size_t w) const noexcept{return w<2?axle_moment_[w]:0.;}
ReasonCode3d Pneumatic3dTireWriter::last_reason() const noexcept{return kernels_.last_reason();}
const Pneumatic3dModel& Pneumatic3dTireWriter::model() const noexcept{return kernels_;}
void Pneumatic3dTireWriter::set_worker_threads(unsigned count){kernels_.set_worker_threads(count);}
} // namespace bike::pneumatic3d
