// W25 source-authored force/ownership gates. Execution is deferred to V04.
#include "support/migration_test.hpp"
#include "support/startup_fixture.hpp"
#include "support/terrain_fixture.hpp"
#include "support/tyre_fixture.hpp"
#include "allocation_faults.hpp"
#include "writers/pneumatic_3d_tire.hpp"
#include "writers/spatial_tire_adapter.hpp"
#include "spatial/frames.hpp"
#include "runtime/runtime.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include "diag.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <new>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <vector>

namespace {
namespace st=spatial_tests;
namespace sp=bike::spatial;
namespace pn=bike::pneumatic3d;
using Vec=sp::Vec3;
using Writer=pn::Pneumatic3dTireWriter;
void allocation_probe_control() {
    constexpr auto alignment=std::align_val_t{2*__STDCPP_DEFAULT_NEW_ALIGNMENT__};
    bool rejected=false;
    try {
        void* pointer=::operator new(std::numeric_limits<std::size_t>::max(),alignment);
        ::operator delete(pointer,alignment);
    } catch(const std::bad_alloc&) {rejected=true;}
    bt::require(rejected,"allocation probe cannot wrap a huge aligned request into a small successful buffer");
    void* huge=::operator new(std::numeric_limits<std::size_t>::max(),alignment,std::nothrow);
    const bool null_result=huge==nullptr;
    ::operator delete(huge,alignment);
    bt::require(null_result,
                "nothrow allocation probe preserves rejection of unrepresentable aligned size");
    void* pointer=::operator new(1,alignment);
    bt::require(pointer!=nullptr,"ordinary aligned allocation remains valid after an oversized request");
    ::operator delete(pointer,alignment);
}
double dot(const Vec& a,const Vec& b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
Vec cross(const Vec& a,const Vec& b) {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
pn::TyreStateIdentity3d identity(const pn::Pneumatic3dSpec& spec,const sp::Topology& topology,
                                const bike::terrain::Terrain3dSource& terrain) {
    pn::TyreStateIdentity3d id;
    id.tier=spec.tier;id.dims=pn::tier_dims(spec.tier);
    id.patch_capacity=id.dims.n_theta_fan*id.dims.n_phi;
    id.topology_fingerprint_hex=topology.fingerprint_hex;
    id.config_fingerprint_hex=std::string(64,'b');
    id.terrain_sha256_hex=terrain.fingerprint_hex();
    id.calibration_sha256_hex=std::string(64,'c');
    return id;
}
pn::Pneumatic3dSpec specification(const st::RigModel& rig,sp::TyreTier tier) {
    auto spec=st::synthetic_spec(tier);
    const auto geom_size=model_access::readonly_buffer(rig.model->geom_size,3*rig.model->ngeom,"geom_size");
    for(std::size_t w=0;w<2;++w) {
        const int geom=mj_name2id(rig.model,mjOBJ_GEOM,w==0?"geom_front_contact":"geom_rear_contact");
        bt::require(geom>=0,"real wheel geometry supplies fixture radius");
        spec.outer_radius_m[w]=geom_size[3*static_cast<std::size_t>(geom)];
        spec.brush[w].r_eff_m=spec.outer_radius_m[w];
        const double r=.5*spec.casing_width_m[w],R=spec.outer_radius_m[w]-r;
        spec.carcass[w].initial_volume_m3=2.*std::numbers::pi*std::numbers::pi*R*r*r;
    }
    return spec;
}
void position(st::RigModel& rig,const pn::Pneumatic3dSpec& spec,double yaw,double roll,double steer) {
    const auto qpos0=model_access::readonly_buffer(rig.model->qpos0,rig.model->nq,"qpos0");
    const auto qpos=model_access::mutable_buffer(rig.data->qpos,rig.model->nq,"qpos");
    std::ranges::copy(qpos0,qpos.begin());
    const auto q=static_cast<std::size_t>(rig.topology.root.free_qpos_adr);
    const Vec original{qpos0[q],qpos0[q+1],qpos0[q+2]};
    // Analytic Rz(yaw)*Rx(roll), independent of the tyre fan-frame builder.
    const sp::Quat rotation{std::cos(yaw/2.)*std::cos(roll/2.),
        std::cos(yaw/2.)*std::sin(roll/2.),std::sin(yaw/2.)*std::sin(roll/2.),
        std::sin(yaw/2.)*std::cos(roll/2.)};
    const auto move_root=[&](const sp::RootBinding& root) {
        bt::require(root.kind==sp::RootKind::free6,"writer fixture roots use the manifold free chart");
        const auto a=static_cast<std::size_t>(root.free_qpos_adr);
        const Vec relative{qpos0[a]-original[0],qpos0[a+1]-original[1],qpos0[a+2]-original[2]};
        const auto offset=sp::rotate_body_to_world(rotation,relative);
        qpos[a]=5.+offset[0];qpos[a+1]=offset[1];qpos[a+2]=1.+offset[2];
        const sp::Quat initial{qpos0[a+3],qpos0[a+4],qpos0[a+5],qpos0[a+6]};
        const auto oriented=sp::quat_multiply(rotation,initial);
        for(std::size_t j=0;j<4;++j)qpos[a+3+j]=oriented[j];
    };
    move_root(rig.topology.root);for(const auto& root:rig.topology.rider_roots)move_root(root);
    qpos[static_cast<std::size_t>(rig.topology.steer.joint.qpos_adr)]=steer;
    engine::forward(rig.model,rig.data);
    // Move the assembled bike and independent rider together until at least
    // one actual torus has 8 mm indentation; no contact-force solve is used.
    double lowest=std::numeric_limits<double>::infinity();
    const auto xaxis=model_access::readonly_buffer(rig.data->xaxis,3*rig.model->njnt,"xaxis");
    const auto xpos=model_access::readonly_buffer(rig.data->xpos,3*rig.model->nbody,"xpos");
    for(std::size_t w=0;w<2;++w) {
        const auto& b=rig.topology.wheels[w];
        const auto axis=xaxis.subspan(3*static_cast<std::size_t>(b.spin.joint_id),3);
        const double r=.5*spec.casing_width_m[w];
        const double extent=(spec.outer_radius_m[w]-r)*std::sqrt(std::max(0.,1.-axis[2]*axis[2]))+r;
        lowest=std::min(lowest,xpos[3*static_cast<std::size_t>(b.wheel_body_id)+2]-extent);
    }
    const double shift=lowest+.008;
    qpos[q+2]-=shift;
    for(const auto& root:rig.topology.rider_roots)qpos[static_cast<std::size_t>(root.free_qpos_adr)+2]-=shift;
    rig.data->time=0.;engine::forward(rig.model,rig.data);
}
struct WorkEvidence {std::size_t contacts=0;double couple_magnitude=0.;};
WorkEvidence virtual_work(const st::RigModel& rig,const Writer& writer,
                          std::span<const double> qfrc,std::span<const double> seed) {
    const auto nv=static_cast<std::size_t>(rig.model->nv);
    std::vector<double> expected(seed.begin(),seed.end()),jp(3*nv),jr(3*nv);
    const auto xpos=model_access::readonly_buffer(rig.data->xpos,3*rig.model->nbody,"xpos");
    const auto xaxis=model_access::readonly_buffer(rig.data->xaxis,3*rig.model->njnt,"xaxis");
    const auto qvel=model_access::readonly_buffer(rig.data->qvel,rig.model->nv,"qvel");
    long double world_power=0.L,generalized_power=0.L;WorkEvidence evidence;
    for(std::size_t w=0;w<2;++w) {
        const auto& binding=rig.topology.wheels[w];double expected_axle_moment=0.;
        std::array<double,6> velocity{};
        struct VelocityCall {const mjModel* m;const mjData* d;int body;double* result;}
            velocity_call{rig.model,rig.data,binding.wheel_body_id,velocity.data()};
        engine::ErrorBuffer error;
        if(!engine::invoke([](void* raw) noexcept {
            const auto& c=*static_cast<VelocityCall*>(raw);
            // XBODY refers to xpos, so this independent power calculation
            // never reuses the writer's BODY/COM shift implementation.
            mj_objectVelocity(c.m,c.d,mjOBJ_XBODY,c.body,c.result,0);
        },&velocity_call,error))engine::throw_failure(error);
        const Vec angular{velocity[0],velocity[1],velocity[2]};
        const auto body=static_cast<std::size_t>(binding.wheel_body_id);
        const auto joint=static_cast<std::size_t>(binding.spin.joint_id);
        const Vec origin{xpos[3*body],xpos[3*body+1],xpos[3*body+2]};
        const Vec axle{xaxis[3*joint],xaxis[3*joint+1],xaxis[3*joint+2]};
        for(const auto& p:writer.wrenches(w))if(p.valid) {
            ++evidence.contacts;
            evidence.couple_magnitude+=std::hypot(p.couple_world_nm[0],p.couple_world_nm[1],p.couple_world_nm[2]);
            const Vec lever{p.point_world_m[0]-origin[0],p.point_world_m[1]-origin[1],p.point_world_m[2]-origin[2]};
            const auto rotational=cross(angular,lever);
            const Vec point_velocity{velocity[3]+rotational[0],velocity[4]+rotational[1],velocity[5]+rotational[2]};
            world_power+=static_cast<long double>(dot(p.force_world_n,point_velocity))+
                         static_cast<long double>(dot(p.couple_world_nm,angular));
            auto moment=cross(lever,p.force_world_n);
            for(std::size_t j=0;j<3;++j)moment[j]+=p.couple_world_nm[j];
            expected_axle_moment+=dot(axle,moment);
            struct JacobianCall {const mjModel* m;const mjData* d;double* jp;double* jr;const double* point;int body;}
                call{rig.model,rig.data,jp.data(),jr.data(),p.point_world_m.data(),binding.wheel_body_id};
            if(!engine::invoke([](void* raw) noexcept {
                const auto& c=*static_cast<JacobianCall*>(raw);mj_jac(c.m,c.d,c.jp,c.jr,c.point,c.body);
            },&call,error))engine::throw_failure(error);
            for(std::size_t j=0;j<nv;++j)for(std::size_t axis=0;axis<3;++axis)
                expected[j]+=jp[axis*nv+j]*p.force_world_n[axis]+jr[axis*nv+j]*p.couple_world_nm[axis];
        }
        bt::require(std::abs(expected_axle_moment-writer.axle_moment_nm(w))<=1e-9,
            "wheel-axis moment contains force lever and intrinsic couple exactly once");
    }
    for(std::size_t j=0;j<nv;++j) {
        generalized_power+=static_cast<long double>(qfrc[j]-seed[j])*static_cast<long double>(qvel[j]);
        bt::require(std::isfinite(qfrc[j])&&std::abs(qfrc[j]-expected[j])<=1e-9,
            "writer adds every translation and rotation Jacobian row to caller forces");
    }
    bt::require(evidence.contacts>0,"virtual work exercises loaded contact, never an empty-force identity");
    bt::require(std::abs(generalized_power-world_power)<=1e-8L,
        "generalized force preserves all-axis world virtual work");
    for(const auto& root:rig.topology.rider_roots)for(std::size_t j=0;j<6;++j) {
        const auto a=static_cast<std::size_t>(root.free_dof_adr)+j;
        bt::require(qfrc[a]==seed[a],"tyre writer does not apply a second contact wrench to independent rider roots");
    }
    return evidence;
}
void restore_rejections(Writer& writer) {
    const auto good=writer.state();const auto before=writer.state_bytes();
    const auto reject=[&](const pn::TyreState3d& wrong) {
        bt::require(writer.restore(wrong)!=CoreStatus::ok,"invalid restore is rejected");
        bt::require(writer.state_bytes()==before,"rejected restore preserves complete material bytes");
    };
    auto wrong=good;wrong.identity.terrain_sha256_hex[0]=wrong.identity.terrain_sha256_hex[0]=='a'?'b':'a';reject(wrong);
    wrong=good;wrong.carcass[1].deflection_m.pop_back();reject(wrong);
    wrong=good;wrong.patches[0].element_to_patch_slot[0]=wrong.identity.patch_capacity;reject(wrong);
    wrong=good;wrong.brush[0].elastic_energy_j=std::numeric_limits<double>::quiet_NaN();reject(wrong);
    bt::require(writer.restore(good)==CoreStatus::ok&&writer.state_bytes()==before,"valid complete restore is exact");
}
void projection_failure(st::RigModel& rig,Writer& writer,
                        const bike::terrain::Terrain3dSource& terrain,double dt) {
    namespace tr=bike::terrain;
    struct Fault {tr::Terrain3dQueryView source;bool reject=false;} fault{terrain.query_view()};
    auto view=fault.source;view.context=&fault;
    view.sample_fn=[](const void* raw,double x,double y) noexcept -> tr::GridSample3d {
        const auto& state=*static_cast<const Fault*>(raw);
        if(state.reject)return {.status=tr::QueryStatus3d::invalid_input};
        return state.source.sample(x,y);
    };
    view.ray_fn=[](const void* raw,Vec origin,Vec direction,double distance) noexcept {
        return static_cast<const Fault*>(raw)->source.ray_cast(origin,direction,distance);
    };
    SpatialTireAdapter adapter(*rig.model,rig.topology,writer,view);
    (void)adapter.probe_qfrc(rig.data,dt);
    const auto before=adapter.probe_snapshots();const auto material=writer.state_bytes();
    std::size_t contacts=0;for(const auto& wheel:before)if(wheel)contacts+=wheel->patches.size();
    bt::require(contacts>0,"projection fault control has previously published contact snapshots");
    fault.reject=true;bool rejected=false;
    try {(void)adapter.probe_qfrc(rig.data,dt);}
    catch(const pn::SpatialTireEvaluationError&){rejected=true;}
    bt::require(rejected&&writer.state_bytes()==material,"failed projection cannot advance material state");
    const auto after=adapter.probe_snapshots();
    for(std::size_t w=0;w<2;++w) {
        bt::require(before[w].has_value()&&after[w].has_value(),"previous complete probe pair remains published");
        const auto& a=*before[w];const auto& b=*after[w];
        bt::require(a.time_s==b.time_s&&a.interval_id==b.interval_id&&a.backend==b.backend&&
            a.geometric_contact==b.geometric_contact&&a.wheel_axis_m==b.wheel_axis_m&&
            a.effective_radius_m==b.effective_radius_m&&a.patches.size()==b.patches.size(),
            "failed replacement preserves all previously published snapshot fields");
        for(std::size_t i=0;i<a.patches.size();++i) {
            const auto& x=a.patches[i];const auto& y=b.patches[i];
            bt::require(x.point_m==y.point_m&&x.normal==y.normal&&x.normal_load_n==y.normal_load_n&&
                x.tangent_force_n==y.tangent_force_n&&x.slip_mps==y.slip_mps&&
                x.working_surface==y.working_surface&&x.native_world_force_n==y.native_world_force_n&&
                x.couple_world_nm==y.couple_world_nm,"failed projection preserves every published contact");
        }
    }
    fault.reject=false;(void)adapter.probe_qfrc(rig.data,dt);
    bt::require(writer.state_bytes()==material,"valid projection retry remains observational");
}
void slot_clock_contract(st::RigModel& rig,Writer& writer,std::span<double> out,double dt) {
    // Precondition: interval 0 is accepted. MuJoCo's `t += dt` drifts off exact
    // slot multiples on long rides, so a sub-slot offset is a legitimate step,
    // while a skipped slot (>= one whole slot away) must still be refused.
    const auto before=writer.state_bytes();
    const std::vector<double> untouched(out.begin(),out.end());
    rig.data->time=2.*dt;
    bt::require(writer.try_compute_into(rig.data,dt,true,out)!=CoreStatus::ok,"skipped time slot is rejected");
    bt::require(writer.state_bytes()==before&&std::equal(out.begin(),out.end(),untouched.begin()),
        "skipped slot changes neither state nor additive output");
    rig.data->time=(1.+1e-6)*dt;
    bt::require(writer.try_compute_into(rig.data,dt,true,out)==CoreStatus::ok,
        "integrator-clock drift of 1e-6 slot is still the next accepted interval");
    bt::require(writer.state().accepted_interval==1,"drifted step advances exactly one interval");
    rig.data->time=0.;
}
void warm_window(st::RigModel& rig,Writer& writer,std::span<double> out,double dt) {
    // Warm error frames and engine Jacobian scratch before counting. This
    // hook measures executable-owned new/delete, not hidden vendor malloc.
    std::fill(out.begin(),out.end(),0.);
    bt::require(writer.try_compute_into(rig.data,dt,false,out)==CoreStatus::ok,"warmup probe succeeds");
    std::array<CoreStatus,8> statuses{};
    allocation_counter::reset();
    for(std::size_t i=0;i<4;++i) {
        rig.data->time=static_cast<double>(writer.state().accepted_interval+1)*dt;
        std::fill(out.begin(),out.end(),0.);
        statuses[2*i]=writer.try_compute_into(rig.data,dt,false,out);
        std::fill(out.begin(),out.end(),0.);
        statuses[2*i+1]=writer.try_compute_into(rig.data,dt,true,out);
    }
    const auto allocations_during_compute=allocation_counter::count();
    const auto frees_during_compute=allocation_counter::freed();
    for(auto status:statuses)bt::require(status==CoreStatus::ok,"warm probe/accepted evaluations succeed");
    bt::require(allocations_during_compute==0&&frees_during_compute==0,
        "warm spatial tyre computation performs no owned allocation or deallocation");
}
void exercise(st::RigModel& rig,sp::TyreTier tier,const bike::terrain::Terrain3dSource& terrain) {
    const auto spec=specification(rig,tier);
    auto writer=std::make_unique<Writer>(*rig.model,rig.topology,spec,terrain,identity(spec,rig.topology,terrain));
    // Dimensions and a self-consistent topology hash do not bind another
    // compiled model with the same joint layout and different physical data.
    const auto root_body=static_cast<std::size_t>(rig.topology.root.body_id);
    const auto old_mass=rig.model->body_mass[root_body];
    rig.model->body_mass[root_body]=old_mass+1.;
    bt::invalid([&] {
        (void)std::make_unique<Writer>(*rig.model,rig.topology,spec,terrain,
                                      identity(spec,rig.topology,terrain));
    });
    rig.model->body_mass[root_body]=old_mass;
    const auto initial=writer->state();const auto nv=static_cast<std::size_t>(rig.model->nv);
    std::vector<double> force(nv),seed(nv);
    const auto qvel=model_access::mutable_buffer(rig.data->qvel,rig.model->nv,"qvel");
    for(std::size_t j=0;j<nv;++j)seed[j]=.125*std::cos(static_cast<double>(j)+.3);
    std::mt19937_64 random(0x57250031ULL);
    std::uniform_real_distribution<double> velocity(-2.,2.),yaw(-std::numbers::pi,std::numbers::pi),roll(-.22,.22),steer(-.25,.25);
    constexpr double dt=.0005;double couples=0.;
    for(int trial=0;trial<8;++trial) {
        bt::require(writer->restore(initial)==CoreStatus::ok,"independent trial restores the canonical initial state");
        for(auto& value:qvel)value=velocity(random);
        position(rig,spec,trial==0?std::numbers::pi/2.:yaw(random),roll(random),steer(random));
        const auto before=writer->state_bytes();const auto engine_before=st::complete_state_bytes(rig);
        force=seed;
        bt::require(writer->try_compute_into(rig.data,dt,false,force)==CoreStatus::ok,"spatial force probe succeeds");
        couples+=virtual_work(rig,*writer,force,seed).couple_magnitude;
        bt::require(writer->state_bytes()==before,"force probe is observational for every material byte");
        bt::require(st::complete_state_bytes(rig)==engine_before,"writer probe preserves engine integration and model bytes");
        if(trial==0 && tier==sp::TyreTier::detailed)projection_failure(rig,*writer,terrain,dt);
        force=seed;
        bt::require(writer->try_compute_into(rig.data,dt,true,force)==CoreStatus::ok,"first interval is accepted once");
        (void)virtual_work(rig,*writer,force,seed);
        bt::require(writer->state().accepted_interval==0,"both wheel histories share one accepted generation");
        const auto accepted=writer->state_bytes();const auto accepted_force=force;
        bt::require(writer->try_compute_into(rig.data,dt,true,force)!=CoreStatus::ok,"duplicate accepted interval is rejected");
        bt::require(writer->state_bytes()==accepted&&force==accepted_force,"duplicate interval changes neither state nor additive output");
        restore_rejections(*writer);
        if(trial==0)slot_clock_contract(rig,*writer,force,dt);
        if(trial==7)warm_window(rig,*writer,force,dt);
    }
    bt::require(couples>1e-10,"nonzero intrinsic couples exercise rotational Jacobian mapping");
}
void collision_masks(st::RigModel& rig,sp::TyreTier tier,const bike::terrain::Terrain3dSource& terrain,const sp::Config& cfg) {
    const auto geom_type=model_access::readonly_buffer(rig.model->geom_type,rig.model->ngeom,"geom_type");
    const auto geom_body=model_access::readonly_buffer(rig.model->geom_bodyid,rig.model->ngeom,"geom_bodyid");
    const auto contype=model_access::mutable_buffer(rig.model->geom_contype,rig.model->ngeom,"geom_contype");
    const auto conaffinity=model_access::mutable_buffer(rig.model->geom_conaffinity,rig.model->ngeom,"geom_conaffinity");
    auto ground=geom_type.size();
    for(std::size_t g=0;g<geom_type.size();++g)if(geom_type[g]==mjGEOM_HFIELD || geom_type[g]==mjGEOM_PLANE){ground=g;break;}
    bt::require(ground<geom_type.size(),"production rider model contains the real crash/render ground");
    for(const auto& wheel:rig.topology.wheels)for(std::size_t g=0;g<geom_body.size();++g)
        if(geom_body[g]==wheel.wheel_body_id)
            bt::require((contype[g]&conaffinity[ground])==0&&
                (conaffinity[g]&contype[ground])==0,
                "production wheel has one physical contact owner");
    const auto spec=specification(rig,tier);
    for(const char* name:{"geom_front_contact","geom_rear_contact"}) {
        const int wheel_id=mj_name2id(rig.model,mjOBJ_GEOM,name);
        bt::require(wheel_id>=0,"production contact geometry is bound");
        const auto wheel=static_cast<std::size_t>(wheel_id);
        const int wt=contype[wheel],wa=conaffinity[wheel];
        const int gt=contype[ground],ga=conaffinity[ground];
        for(bool reciprocal:{false,true}) {
            contype[wheel]=reciprocal?0:1;conaffinity[wheel]=reciprocal?1:0;
            contype[ground]=reciprocal?1:0;conaffinity[ground]=reciprocal?0:1;
            // Rebind identity after the deliberate model mutation: the
            // rejection must be the collision mask, not a stale model hash.
            const auto topology=sp::bind_topology(*rig.model,cfg);bool named=false;
            bt::invalid([&] {
                try {(void)std::make_unique<Writer>(*rig.model,topology,spec,terrain,identity(spec,topology,terrain));}
                catch(const std::invalid_argument& error){named=std::string(error.what()).find("engine_contact_enabled")!=std::string::npos;throw;}
            });
            bt::require(named,"constructor identifies the competing engine contact producer");
        }
        contype[wheel]=wt;conaffinity[wheel]=wa;
        contype[ground]=gt;conaffinity[ground]=ga;
    }
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void planar_golden() {
    // Existing immutable capture; absent binary fixture is an acquisition
    // failure. No replacement baseline is generated by this source test.
    auto ride=std::make_unique<runtime::NativeRideRuntime>(runtime::read_bootstrap(bt::fixture("bootstrap/welded")));
    bt::golden_f64("bootstrap/welded/integration.f64",ride->snapshot().integration_state);
    runtime::RideControl control;control.motor_torque_nm=25.;control.human_torque_nm=10.;
    (void)ride->probe_step_inputs(control,.25,.5);
    bt::golden_f64("bootstrap/welded/integration.f64",ride->snapshot().integration_state);
    (void)ride->advance(4,control,0.,0.,std::nullopt);ride->reset();
    bt::golden_f64("bootstrap/welded/integration.f64",ride->snapshot().integration_state);
}
NATIVE_DIAG_POP
} // namespace

int main(){return bt::run([] {
    allocation_probe_control();
    bike::terrain::Terrain3dSource terrain(st::plane_grid(0.,0.,0.,129,65,.125,.125));
    for(auto tier:{sp::TyreTier::fast,sp::TyreTier::detailed}) {
        auto mixed=st::mixed_joint_rig();
        bt::require(mixed.model->nq!=mixed.model->nv&&mixed.topology.root.free_qpos_adr!=mixed.topology.root.free_dof_adr,
            "mixed FREE/BALL/scalar fixture detects qpos-to-dof indexing shortcuts");
        exercise(mixed,tier,terrain);
        sp::Config cfg;cfg.mode=sp::Mode::spatial3d;cfg.root=sp::RootKind::free6;cfg.steer.controller=sp::SteerController::off;
        auto rider=st::free_rig(cfg,true);
        bt::require(!rider.topology.rider_roots.empty(),"virtual work includes the production articulated rider inventory");
        exercise(rider,tier,terrain);collision_masks(rider,tier,terrain,cfg);
    }
    planar_golden();
});}
