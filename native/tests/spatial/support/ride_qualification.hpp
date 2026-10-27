#pragma once

#include "startup_cases.hpp"
#include "spatial_fixture.hpp"
#include "pneumatic3d/kernel_math.hpp"
#include "writers/pneumatic_3d_tire.hpp"
#include "engine_call.hpp"
#include "stepper.hpp"
#include "model_access.hpp"
#include "spatial/frames.hpp"
#include "io/hash.hpp"
#include "configuration/load.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/trackfile.hpp"
#include "terrain/spec_io.hpp"
#include "runtime/spatial_tire_factory.hpp"
#include "runtime/samples.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace bike::validation {
// Test-only owner access. It is deliberately not the W42 persisted checkpoint
// API. No incomplete engine-vector snapshot can enter this restore boundary.
class SpatialRideAccess {
public:
    struct Checkpoint {
        std::string model_sha256;
        std::shared_ptr<mjData> engine_data;
        std::vector<double> integration,frictionloss,site_pos,tendon_stiffness,tendon_damping;
        std::vector<double> tendon_lengthspring,tendon_range,wrap_prm;
        drivetrain::DriveSnapshot drive;
        std::optional<std::pair<writers::RiderContactsState,std::optional<writers::RiderContactsProbe>>> rider;
        pneumatic3d::TyreState3d tyre;
        runtime::StepState step;
        runtime::AccountingState accounting;
        std::optional<runtime::PeriodBuffer> buffer;
        runtime::AccountingDiagnostics diagnostics;
        std::vector<runtime::SamplePtr> pending,retained;
        runtime::SamplePtr latest;
        runtime::NativeRideRuntime::BoundaryState boundary;
        runtime::PresentationState presentation;
        runtime::WheelSnapshots wheel_snapshots;
        std::optional<runtime::RiderKinematicState> rider_kinematics;
        std::optional<runtime::ForceSample> force_sample;
        runtime::ConstraintSnapshot constraints;
        runtime::SpatialBrakeResult brake_result;
        std::vector<double> brake_force,acc_inputs;
        bool acc_inputs_valid=false;
        std::optional<double> finish_position;
        std::optional<std::string> warning;
        int accounting_generation=0;
    };
    // Cold test snapshots copy the complete engine/owner inventory. They are
    // heap-owned; the large StepState return is initialized directly on heap.
    static std::unique_ptr<Checkpoint> capture(runtime::NativeRideRuntime& r) {
        r.require_open();r.require_boundary_ready();
        if(r.advancing_.load() || r.accounting_->asynchronous() || !r.accounting_->detached_.empty())
            throw std::invalid_argument("spatial qualification capture requires a synchronous committed boundary");
        auto checkpoint=std::make_unique<Checkpoint>();
        auto& c=*checkpoint;
        auto& s=*r.stepper_;auto* m=s.model();auto* d=s.data();
        c.engine_data=std::shared_ptr<mjData>(engine::make_data(m),mj_deleteData);
        if(!c.engine_data)throw std::runtime_error("qualification checkpoint engine allocation");
        copy_engine(m,c.engine_data.get(),d);
        c.model_sha256=r.model_image_.sha256;
        c.integration.resize(static_cast<std::size_t>(engine::state_size(m,mjSTATE_INTEGRATION)));
        engine::get_state(m,d,c.integration.data(),mjSTATE_INTEGRATION);
        const auto copy_row=[](std::vector<double>& out,const double* data,mjtSize count) {
            const auto row=model_access::readonly_buffer(data,count);out.assign(row.begin(),row.end());
        };
        copy_row(c.frictionloss,m->dof_frictionloss,m->nv);
        copy_row(c.site_pos,m->site_pos,3*m->nsite);
        copy_row(c.tendon_stiffness,m->tendon_stiffness,m->ntendon);
        copy_row(c.tendon_damping,m->tendon_damping,m->ntendon);
        copy_row(c.tendon_lengthspring,m->tendon_lengthspring,2*m->ntendon);
        copy_row(c.tendon_range,m->tendon_range,2*m->ntendon);
        copy_row(c.wrap_prm,m->wrap_prm,m->nwrap);
        c.drive=s.drive().state();c.tyre=s.spatial_tire_writer().state();
        if(s.has_rider_contacts())c.rider.emplace(s.rider_contacts().state(),s.rider_contacts().probe());
        const std::unique_ptr<runtime::StepState> step{new runtime::StepState(r.physical_->state())};
        c.step=std::move(*step);c.wheel_snapshots=r.physical_->snapshots_;
        c.rider_kinematics=r.physical_->rider_state_;c.force_sample=r.physical_->last_force_sample_;
        c.constraints=r.physical_->last_constraint_snapshot_;
        c.brake_result=r.physical_->spatial_brake_last_;c.brake_force=r.physical_->spatial_brake_force_;
        c.acc_inputs=r.physical_->acc_inputs_;c.acc_inputs_valid=r.physical_->acc_inputs_valid_;
        const auto& a=*r.accounting_;
        c.accounting=a.state_;c.buffer=a.buffer_;c.diagnostics=a.last_diagnostics_;
        c.pending=a.pending_;c.retained=a.retained_;c.latest=a.latest_;
        c.boundary=r.capture_boundary();c.presentation=r.committed_presentation_;
        c.finish_position=r.finish_position_m_;c.warning=r.pending_warning_;c.accounting_generation=a.generation_;
        return checkpoint;
    }
    static void restore(runtime::NativeRideRuntime& r,const Checkpoint& c) {
        r.require_open();r.require_boundary_ready();
        auto& s=*r.stepper_;auto* m=s.model();auto* d=s.data();
        if(r.advancing_.load() || r.accounting_->asynchronous() ||
           c.model_sha256!=r.model_image_.sha256 || !c.buffer || !c.engine_data ||
           c.integration.size()!=static_cast<std::size_t>(engine::state_size(m,mjSTATE_INTEGRATION)) ||
           c.frictionloss.size()!=static_cast<std::size_t>(m->nv) ||
           c.site_pos.size()!=static_cast<std::size_t>(3*m->nsite) ||
           c.tendon_stiffness.size()!=static_cast<std::size_t>(m->ntendon) ||
           c.tendon_damping.size()!=static_cast<std::size_t>(m->ntendon) ||
           c.tendon_lengthspring.size()!=static_cast<std::size_t>(2*m->ntendon) ||
           c.tendon_range.size()!=static_cast<std::size_t>(2*m->ntendon) ||
           c.wrap_prm.size()!=static_cast<std::size_t>(m->nwrap) || !c.step.spatial ||
           c.rider.has_value()!=s.has_rider_contacts())
            throw std::invalid_argument("spatial qualification checkpoint identity or inventory mismatch");
        // Validate the owned material envelope before changing live owners.
        const auto encoded=pneumatic3d::encode_tyre_state(c.tyre);
        const auto decoded=pneumatic3d::decode_tyre_state(encoded,s.spatial_tire_writer().state().identity);
        if(!decoded)throw std::invalid_argument(decoded.error());
        s.mutate([&] {
            const auto copy_row=[](std::span<const double> source,double* data,mjtSize count) {
                auto row=model_access::mutable_buffer(data,count);
                if(source.size()!=row.size())throw std::invalid_argument("qualification model row shape mismatch");
                std::ranges::copy(source,row.begin());
            };
            copy_row(c.frictionloss,m->dof_frictionloss,m->nv);
            copy_row(c.site_pos,m->site_pos,3*m->nsite);
            copy_row(c.tendon_stiffness,m->tendon_stiffness,m->ntendon);
            copy_row(c.tendon_damping,m->tendon_damping,m->ntendon);
            copy_row(c.tendon_lengthspring,m->tendon_lengthspring,2*m->ntendon);
            copy_row(c.tendon_range,m->tendon_range,2*m->ntendon);
            copy_row(c.wrap_prm,m->wrap_prm,m->nwrap);
            engine::set_const(m,d);
            s.drive().restore(c.drive);
            engine::set_state(m,d,c.integration.data(),mjSTATE_INTEGRATION);
            s.restore_spatial_tire(*decoded);
            if(c.rider) {
                auto state=c.rider->first;auto probe=c.rider->second;
                s.rider_contacts().assign_state(std::move(state),std::move(probe));
            }
            r.physical_->restore(c.step);r.physical_->snapshots_=c.wheel_snapshots;
            r.physical_->rider_state_=c.rider_kinematics;r.physical_->last_force_sample_=c.force_sample;
            r.physical_->last_constraint_snapshot_=c.constraints;
            r.physical_->spatial_brake_last_=c.brake_result;r.physical_->spatial_brake_force_=c.brake_force;
            // mj_copyData retains derived acceleration, solver workspace and
            // every warmstart as well as mjSTATE_INTEGRATION. Re-forwarding
            // here would alter the continuation being tested.
            copy_engine(m,d,c.engine_data.get());s.note_posvel_fresh();
            r.physical_->acc_inputs_=c.acc_inputs;r.physical_->acc_inputs_valid_=c.acc_inputs_valid;
        });
        auto& a=*r.accounting_;
        a.state_=c.accounting;a.buffer_=*c.buffer;a.last_diagnostics_=c.diagnostics;
        a.pending_=c.pending;a.retained_=c.retained;a.latest_=c.latest;
        a.detached_.clear();a.detached_at_=0;a.generation_=c.accounting_generation;
        r.next_integration_state_=c.integration;r.next_presentation_=c.presentation;
        auto boundary=c.boundary;r.commit_boundary(std::move(boundary));
        r.finish_position_m_=c.finish_position;r.pending_warning_=c.warning;
    }
    static void copy_engine(const mjModel* m,mjData* output,const mjData* input) {
        struct Call {const mjModel* m;mjData* out;const mjData* in;} call{m,output,input};engine::ErrorBuffer error;
        if(!engine::invoke([](void* raw) noexcept {const auto& c=*static_cast<Call*>(raw);mj_copyData(c.out,c.m,c.in);},&call,error))
            engine::throw_failure(error);
    }
    static void perturb(runtime::NativeRideRuntime& r,spatial::Vec3 translation,
                        spatial::Vec3 rotation,double steer_angle=0.,double yaw_rate=0.) {
        if(r.committed_step_!=0 || r.stepper_->time()!=0.)
            throw std::invalid_argument("spatial qualification initial conditions require t=0");
        auto& s=*r.stepper_;auto& p=*r.physical_;auto* m=s.model();auto* d=s.data();
        if(!p.topology() || p.topology()->root.kind!=spatial::RootKind::free6)
            throw std::invalid_argument("spatial qualification requires a free root");
        const auto& topology=*p.topology();
        const auto address=static_cast<std::size_t>(topology.root.free_dof_adr);
        const auto root_q=static_cast<std::size_t>(topology.root.free_qpos_adr);
        const auto joint_type=model_access::readonly_buffer(m->jnt_type,m->njnt);
        const auto qpos_address=model_access::readonly_buffer(m->jnt_qposadr,m->njnt);
        const auto dof_address=model_access::readonly_buffer(m->jnt_dofadr,m->njnt);
        auto qpos=model_access::mutable_buffer(d->qpos,m->nq);
        auto qvel=model_access::mutable_buffer(d->qvel,m->nv);
        const auto pivot=spatial::body_pose(*m,*d,topology.root).position_world_m;
        const auto rotation_norm=std::sqrt(rotation[0]*rotation[0]+rotation[1]*rotation[1]+rotation[2]*rotation[2]);
        const double sinc=rotation_norm>0.?std::sin(.5*rotation_norm)/rotation_norm:.5;
        const spatial::Quat turn{std::cos(.5*rotation_norm),sinc*rotation[0],sinc*rotation[1],sinc*rotation[2]};
        s.mutate([&] {
            // Move all independent free bodies (including the rider) by the
            // same rigid transform; preserve their attachment geometry.
            for(std::size_t j=0;j<joint_type.size();++j)if(joint_type[j]==mjJNT_FREE) {
                const auto q=static_cast<std::size_t>(qpos_address[j]);
                const auto v=static_cast<std::size_t>(dof_address[j]);
                spatial::Vec3 relative{},linear{};
                for(std::size_t k=0;k<3;++k){relative[k]=qpos[q+k]-pivot[k];linear[k]=qvel[v+k];}
                const auto moved=spatial::rotate_body_to_world(turn,relative);
                const auto velocity=spatial::rotate_body_to_world(turn,linear);
                const spatial::Quat old{qpos[q+3],qpos[q+4],qpos[q+5],qpos[q+6]};
                const auto quat=spatial::quat_multiply(turn,old);
                const spatial::Vec3 extra_velocity{-yaw_rate*moved[1],yaw_rate*moved[0],0.};
                const auto local_angular=spatial::rotate_world_to_body(quat,{0.,0.,yaw_rate});
                for(std::size_t k=0;k<3;++k){qpos[q+k]=pivot[k]+translation[k]+moved[k];qvel[v+k]=velocity[k]+extra_velocity[k];qvel[v+3+k]=local_angular[k];}
                for(std::size_t k=0;k<4;++k)qpos[q+3+k]=quat[k];
            }
            if(topology.steer.joint.qpos_adr>=0)qpos[static_cast<std::size_t>(topology.steer.joint.qpos_adr)]=steer_angle;
            const spatial::Quat root_quat{qpos[root_q+3],qpos[root_q+4],qpos[root_q+5],qpos[root_q+6]};
            const auto local_rate=spatial::rotate_world_to_body(root_quat,{0.,0.,yaw_rate});
            for(std::size_t k=0;k<3;++k)qvel[address+3+k]=local_rate[k];
            s.note_posvel_stale();s.forward();
            auto [mass,elastic,total]=p.energy_state();
            double storage=0.;for(const auto& [name,value]:elastic){(void)name;storage+=runtime::sample_wire::number(value);}
            p.energy_.initial_energy_j=total;p.energy_.energy_scale_j=std::max(1.,mass.kinetic_energy_j+storage);
            engine::get_state(m,d,r.bootstrap_.integration_state.data(),mjSTATE_INTEGRATION);
            const std::unique_ptr<runtime::StepState> initial{new runtime::StepState(p.state())};
            r.bootstrap_step_state_=std::move(*initial);
        });
        r.initialize_accounting();
    }
    static Stepper& stepper(runtime::NativeRideRuntime& r) { return *r.stepper_; }
    static runtime::PhysicalStep& physical(runtime::NativeRideRuntime& r) { return *r.physical_; }
    static const runtime::SpatialBootstrapState& causal(runtime::NativeRideRuntime& r) {return *r.physical_->spatial_state_;}
    static const terrain::SurfaceSpec3d& surface(runtime::NativeRideRuntime& r,std::uint32_t id) {
        const auto* grid=r.config_.terrain3d?r.config_.terrain3d->grid():nullptr;
        if(grid)for(const auto& surface:grid->surfaces())if(surface.id==id)return surface;
        throw std::invalid_argument("spatial qualification: material coefficient unavailable");
    }
    static void reference_tick(runtime::NativeRideRuntime& r,double roll_reference,
                               double heading_reference,double lateral_error,double steer_reference=0.) {
        auto& p=*r.physical_;
        if(!p.steer_balance_ || !p.control_clock_.is_tick(p.step_))return;
        const auto pose=spatial::body_pose(*r.stepper_->model(),*r.stepper_->data(),p.topology_->root);
        const auto angles=spatial::euler_zyx(pose.quat_world_from_body_wxyz);
        const auto omega=spatial::rotate_world_to_body(pose.quat_world_from_body_wxyz,pose.angular_velocity_world_rad_s);
        const double roll_rate=angles.valid?omega[0]+std::tan(angles.pitch_rad)*(
            std::sin(angles.roll_rad)*omega[1]+std::cos(angles.roll_rad)*omega[2]):0.;
        const auto& j=p.topology_->steer.joint;const auto* d=r.stepper_->data();
        const auto qpos=model_access::readonly_buffer(d->qpos,r.stepper_->model()->nq);
        const auto qvel=model_access::readonly_buffer(d->qvel,r.stepper_->model()->nv);
        const auto tick=p.step_/p.control_clock_.steps_per_period;
        const double old_work=p.steer_balance_->state().work_j;
        (void)p.steer_balance_->tick({.control_tick=tick,.roll_rad=angles.roll_rad-roll_reference,
            .roll_rate_rad_s=roll_rate,
            .heading_error_rad=std::remainder(angles.yaw_rad-heading_reference,2.*std::numbers::pi),
            .lateral_error_m=lateral_error,.steer_angle_rad=qpos[static_cast<std::size_t>(j.qpos_adr)]-steer_reference,
            .steer_rate_rad_s=qvel[static_cast<std::size_t>(j.dof_adr)],.speed_signed_mps=p.longitudinal_speed_mps(),.enabled=angles.valid});
        auto state=p.steer_balance_->state();state.work_j=old_work;
        if(p.steer_balance_->restore(state)!=CoreStatus::ok)throw std::logic_error("test reference controller state rejected");
        p.spatial_state_->steer=state;
        // apply_forces sees this same integer tick and cannot consume the FIFO
        // twice. PhysicalStep integrates actual solved steering work per dt.
    }
    static double energy_error(runtime::NativeRideRuntime& r) {
        const auto& e=r.accounting_->state().energy;
        return std::abs(e.residual_j)/std::max(1.,e.energy_scale_j);
    }
    static terrain::Terrain3dQueryView terrain_view(runtime::NativeRideRuntime& r) {return r.config_.terrain3d->query_view();}
    static double brake_budget_error(runtime::NativeRideRuntime& r) {
        const auto& p=*r.physical_;const auto& s=*r.stepper_;
        const auto solved=p.brake_.solved_components(s.model(),s.data());
        const std::array<const std::vector<double>*,2> components{&solved.first,&solved.second};
        double excess=0.;
        for(std::size_t w=0;w<2;++w) {
            const auto dof=p.topology_->wheels[w].spin.dof_adr;
            const double total=(*components[w])[static_cast<std::size_t>(dof)]+p.spatial_brake_last_.hold_torque_nm[w];
            excess=std::max(excess,std::abs(total)-p.brake_.ceiling_nm()*p.spatial_state_->brake.wheel[w].previous_demand);
        }
        return excess;
    }
};
} // namespace bike::validation

namespace spatial_tests::ride {
using bike::pneumatic3d::detail::add;
using bike::pneumatic3d::detail::sub;
using bike::pneumatic3d::detail::scale;
using bike::pneumatic3d::detail::cross;
using bike::pneumatic3d::detail::dot;
using bike::pneumatic3d::detail::norm;
using bike::spatial::Vec3;
using bike::validation::SpatialRideAccess;
inline constexpr double gravity=9.81;
struct Demand { double time_s=0.,torque_fraction=0.,brake_fraction=0.; };
inline std::filesystem::path example(std::string_view relative) {
    return std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"examples"/relative;
}
inline std::shared_ptr<const bike::terrain::Terrain3dSource> surface_grid(
        const std::function<double(double,double)>& elevation,
        const std::function<std::uint32_t(double,double)>& material,
        std::vector<bike::terrain::SurfaceSpec3d> surfaces,
        double length=200.,double half_width=6.,double dx=.25,double dy=.25,
        std::string name="analytic qualification terrain") {
    using namespace bike::terrain;
    const auto nx=static_cast<std::uint64_t>(std::llround(length/dx))+1;
    const auto ny=static_cast<std::uint64_t>(std::llround(2.*half_width/dy))+1;
    std::vector<double> z(static_cast<std::size_t>(nx*ny));
    std::vector<std::uint32_t> ids(static_cast<std::size_t>((nx-1)*(ny-1)));
    for(std::uint64_t y=0;y<ny;++y)for(std::uint64_t x=0;x<nx;++x)
        z[static_cast<std::size_t>(y*nx+x)]=elevation(static_cast<double>(x)*dx,-half_width+static_cast<double>(y)*dy);
    for(std::uint64_t y=0;y+1<ny;++y)for(std::uint64_t x=0;x+1<nx;++x)
        ids[static_cast<std::size_t>(y*(nx-1)+x)]=material((static_cast<double>(x)+.5)*dx,-half_width+(static_cast<double>(y)+.5)*dy);
    auto grid=Grid3d::create({.nx=nx,.ny=ny,.x0_m=0.,.y0_m=-half_width,.dx_m=dx,.dy_m=dy,.dtype=0},
        std::move(z),std::move(ids),std::move(surfaces),
        {.generator_version="qualification-analytic/v1",.resolved_spec=bike::io::Value{name},
         .seed_decimal="73421",.units="m",.height_datum_m=0.,
         .provenance=bike::io::Value{std::string("independent analytic fixture; no captured trajectory")}});
    if(!grid)throw std::invalid_argument(grid.error());
    return std::make_shared<const Terrain3dSource>(std::move(*grid));
}
inline std::vector<bike::terrain::SurfaceSpec3d> dry_and_wet(double dry=.8,double wet=.3) {
    return {{.id=1,.name="dry",.mu_peak=dry,.mu_slide=.75*dry,.slip_stiffness_per_load=20.,.stribeck_speed_mps=.1,.description="synthetic"},
            {.id=2,.name="wet",.mu_peak=wet,.mu_slide=.75*wet,.slip_stiffness_per_load=15.,.stribeck_speed_mps=.1,.description="synthetic"}};
}
inline std::shared_ptr<const bike::terrain::Terrain3dSource> mirror_terrain(const bike::terrain::Terrain3dSource& source) {
    using namespace bike::terrain;
    const auto* original=source.grid();if(!original)throw std::invalid_argument("mirror fixture requires an owned grid");
    auto desc=original->descriptor();desc.y0_m=-original->y_max_m();
    std::vector<double> z(original->elevation().size());std::vector<std::uint32_t> ids(original->materials().size());
    for(std::uint64_t y=0;y<desc.ny;++y)for(std::uint64_t x=0;x<desc.nx;++x)
        z[static_cast<std::size_t>(y*desc.nx+x)]=original->elevation()[static_cast<std::size_t>((desc.ny-1-y)*desc.nx+x)];
    for(std::uint64_t y=0;y+1<desc.ny;++y)for(std::uint64_t x=0;x+1<desc.nx;++x)
        ids[static_cast<std::size_t>(y*(desc.nx-1)+x)]=original->materials()[static_cast<std::size_t>((desc.ny-2-y)*(desc.nx-1)+x)];
    auto metadata=original->metadata();metadata.provenance=bike::io::Value{std::string("y-reflection of ")+original->sha256_hex()};
    auto grid=Grid3d::create(desc,std::move(z),std::move(ids),
        std::vector<SurfaceSpec3d>(original->surfaces().begin(),original->surfaces().end()),std::move(metadata),original->budget());
    if(!grid)throw std::invalid_argument(grid.error());return std::make_shared<const Terrain3dSource>(std::move(*grid));
}
struct MirrorSpec {
    bike::spatial::TyreTier tier=bike::spatial::TyreTier::fast;
    std::shared_ptr<const bike::terrain::Terrain3dSource> terrain;
    double speed=3.,lateral=.02,roll=.01,heading=.02,steer=.01;
    bike::spatial::Vec3 sensor_position{.1,.03,.1},sensor_rpy{.1,.02,.03};
    bool reflected=false;
};
struct ProfilePin {std::string tyre,steer;};
inline ProfilePin pin_profile(bike::initialization::SetupRequest&,const std::filesystem::path&);
inline std::unique_ptr<OwnedStartup> mirror_rig(MirrorSpec s) {
    const double sign=s.reflected?-1.:1.;
    auto request=make_startup_request(bike::spatial::RootKind::free6,s.tier,false,s.speed,
        s.reflected?mirror_terrain(*s.terrain):s.terrain);
    auto& cfg=*request->model.physics->spatial;
    cfg.imu.mount_pos=bike::spatial::Vec3{s.sensor_position[0],sign*s.sensor_position[1],s.sensor_position[2]};
    cfg.imu.mount_rpy_rad={sign*s.sensor_rpy[0],s.sensor_rpy[1],sign*s.sensor_rpy[2]};
    cfg.corridor_half_width_m=5.;request->model.spatial=cfg;
    (void)pin_profile(*request,example("research/spatial_flat.toml"));
    auto rig=std::make_unique<OwnedStartup>(std::move(*request));
    SpatialRideAccess::perturb(*rig->runtime,{0.,sign*s.lateral,0.},{sign*s.roll,0.,sign*s.heading},sign*s.steer);
    return rig;
}
inline std::unique_ptr<OwnedStartup> arena_rig(double radius,double speed,bool articulated,
                                             bike::spatial::TyreTier tier=bike::spatial::TyreTier::fast) {
    auto terrain=surface_grid([](double,double){return 0.;},[](double,double){return 1u;},dry_and_wet(),120.,60.,.5,.5,"120m flat arena");
    auto request=make_startup_request(bike::spatial::RootKind::free6,tier,articulated,speed,std::move(terrain));
    request->start_x_m=60.;request->track.length_m=120.;
    request->model.physics->spatial->corridor_half_width_m=60.;request->model.spatial=request->model.physics->spatial;
    (void)pin_profile(*request,example("benchmarks/spatial_cornering.toml"));
    auto rig=std::make_unique<OwnedStartup>(std::move(*request));
    SpatialRideAccess::perturb(*rig->runtime,{0.,-radius,0.},{-std::atan(speed*speed/(gravity*radius)),0.,0.},
        std::atan(1.2/radius),speed/radius);
    return rig;
}
inline double mean(std::span<const double> values) {
    if(values.empty())throw std::invalid_argument("empty statistic");
    return std::accumulate(values.begin(),values.end(),0.)/static_cast<double>(values.size());
}
inline double coefficient_of_variation(std::span<const double> values) {
    const double average=mean(values);double variance=0.;for(double v:values)variance+=(v-average)*(v-average);
    return std::sqrt(variance/static_cast<double>(values.size()))/std::max(1e-12,std::abs(average));
}
inline ProfilePin pin_profile(bike::initialization::SetupRequest& request,const std::filesystem::path& manifest) {
    const auto doc=bike::configuration::detail::parse_toml_file(manifest);
    const auto& root=bike::io::as_object(doc,"qualification");
    const auto* profile=bike::io::find_member(root,"profile");
    if(!profile)throw std::invalid_argument("qualification profile pin missing");
    const auto& fields=bike::io::as_object(*profile,"profile");
    const auto string=[&](const char* key){const auto* v=bike::io::find_member(fields,key);
        if(!v)throw std::invalid_argument(std::string("profile.")+key+" missing");return bike::io::as_string(*v,key);};
    auto& physics=*request.model.physics;
    physics.spatial->tyre.lateral_coefficients_id=string("tyre_id");
    physics.spatial->steer.profile_id=string("steer_id");
    const auto coefficients=bike::configuration::spatial_tyre_coefficient_profile(physics.spatial->tyre.lateral_coefficients_id);
    const auto real=[&](const char* key){const auto* v=bike::io::find_member(fields,key);
        if(!v)throw std::invalid_argument(std::string("profile.")+key+" missing");return bike::io::as_double(*v,key);};
    bt::require(coefficients.kx_n_per_m3==real("kx_n_per_m3") && coefficients.ky_n_per_m3==real("ky_n_per_m3") &&
        coefficients.relaxation_x_m==real("relaxation_x_m") && coefficients.relaxation_y_m==real("relaxation_y_m") &&
        coefficients.camber_coefficient_per_rad==real("camber_coefficient_per_rad") && coefficients.pneumatic_trail_m==real("pneumatic_trail_m"),
        "authored qualification coefficients equal the pinned source recipe");
    bt::require(physics.tires.pneumatic.front.outer_radius_mm==372. && physics.tires.pneumatic.rear.outer_radius_mm==352. &&
        physics.tires.pneumatic.front.pressure_pa()==150000. && physics.tires.pneumatic.rear.pressure_pa()==170000.,
        "qualification geometry and pressure retain their declared profile");
    const auto data_root=std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"native/tests/spatial/data";
    const auto pinned=spatial_tests::json(data_root/"ride_profile_pin.json");
    const auto& object=bike::io::as_object(pinned,"source_pin");
    const auto* wire=bike::io::find_member(object,"pneumatic_3d");
    const auto* rates=bike::io::find_member(object,"maxwell_rate_stiffening");
    if(!wire || !rates)throw std::invalid_argument("source coefficient pin is incomplete");
    auto expected=*wire;
    runtime::setupvalue::set(expected,"tier",bike::io::Value{std::string(bike::spatial::to_string(physics.spatial->tyre.tier))});
    const auto mutable_field=[](bike::io::Value& value,std::string_view key)->bike::io::Value& {
        for(auto& [name,item]:std::get<bike::io::Value::Object>(value.data))if(name==key)return item;
        throw std::invalid_argument("source coefficient pin field missing");
    };
    auto& carcass=std::get<bike::io::Value::Array>(mutable_field(expected,"carcass").data);
    const auto& outer=bike::io::as_array(mutable_field(expected,"outer_radius_m"),"outer_radius_m");
    const auto& width=bike::io::as_array(mutable_field(expected,"casing_width_m"),"casing_width_m");
    const auto& rate=bike::io::as_array(*rates,"maxwell_rate_stiffening");
    if(carcass.size()!=2 || outer.size()!=2 || width.size()!=2 || rate.size()!=2)
        throw std::invalid_argument("source coefficient pin requires front/rear pairs");
    for(std::size_t w=0;w<2;++w) {
        const double r=.5*bike::io::as_double(width[w],"casing_width_m");
        const double R=bike::io::as_double(outer[w],"outer_radius_m")-r;
        runtime::setupvalue::set(carcass[w],"initial_volume_m3",runtime::setupvalue::real(2.*std::numbers::pi*std::numbers::pi*R*r*r));
        runtime::setupvalue::set(carcass[w],"maxwell_k_n_per_m3",runtime::setupvalue::real(bike::io::as_double(rate[w],"rate_stiffening")*1e6));
    }
    // Numeric content, rather than optional trailing zeroes in the JSON
    // file, defines the source pin. Keep integer fields (capacity) integers.
    std::function<void(bike::io::Value&)> normalize=[&](bike::io::Value& value) {
        if(auto* number=std::get_if<bike::io::Number>(&value.data)) {
            if(number->lexeme.find_first_of(".eE")!=std::string::npos)
                value=runtime::setupvalue::real(bike::io::as_double(value,"source_pin.number"));
        } else if(auto* array=std::get_if<bike::io::Value::Array>(&value.data))for(auto& item:*array)normalize(item);
        else if(auto* fields0=std::get_if<bike::io::Value::Object>(&value.data))for(auto& [key,item]:*fields0){(void)key;normalize(item);}
    };
    normalize(expected);
    const auto expected_json=bike::io::canonical_json(expected);
    bt::require(bike::io::canonical_json(runtime::project_spatial_tire_config(physics))==expected_json,
        "resolved tyre content equals the independently authored full coefficient pin");
    const auto expected_tyre_digest=bike::io::sha256(std::as_bytes(std::span{expected_json}));
    const auto expected_steer=bike::rider::read_steer_profile(spatial_tests::json(data_root/"steer_profiles.json"));
    runtime::resolve_spatial_runtime_profiles(physics);request.model.spatial=physics.spatial;
    // Content identities are computed from the pinned cold recipe. They are
    // distinct from the unfilled experimental/qualification hashes.
    bt::require(physics.spatial->tyre.lateral_coefficients_sha256==expected_tyre_digest &&
        physics.spatial->steer.profile_sha256==expected_steer.sha256_hex,"resolved profile identities equal independent source pins");
    return {expected_tyre_digest,expected_steer.sha256_hex};
}
inline std::vector<Demand> demand_trace(const std::filesystem::path& file) {
    const auto doc=bike::configuration::detail::parse_toml_file(file);
    const auto* table=bike::io::find_member(bike::io::as_object(doc,"qualification"),"demand_trace");
    if(!table)throw std::invalid_argument("qualification file lacks demand_trace");
    const auto* points=bike::io::find_member(bike::io::as_object(*table,"demand_trace"),"points");
    if(!points)throw std::invalid_argument("demand_trace.points missing");
    std::vector<Demand> out;
    for(const auto& row:bike::io::as_array(*points,"demand_trace.points")) {
        const auto& values=bike::io::as_array(row,"demand_trace.points[]");
        if(values.size()!=3)throw std::invalid_argument("demand trace row must be [t_s,torque_fraction,brake_fraction]");
        Demand d{bike::io::as_double(values[0],"t_s"),bike::io::as_double(values[1],"torque_fraction"),bike::io::as_double(values[2],"brake_fraction")};
        if(!std::isfinite(d.time_s) || d.time_s<0. || !std::isfinite(d.torque_fraction) ||
           d.torque_fraction<0. || d.torque_fraction>1. || !std::isfinite(d.brake_fraction) ||
           d.brake_fraction<0. || d.brake_fraction>1. || (!out.empty() && d.time_s<=out.back().time_s))
            throw std::invalid_argument("invalid qualification demand trace");
        out.push_back(d);
    }
    if(out.empty() || out.front().time_s!=0.)throw std::invalid_argument("demand trace must start at zero");
    return out;
}
struct Point {
    double time=0.,mass=0.,roll=0.,yaw=0.,speed=0.,yaw_rate=0.;
    Vec3 position{},com{},velocity{},momentum{},force{},moment{},tyre_force{},accel_sensor{},gyro_sensor{};
    bike::spatial::Quat quaternion{1.,0.,0.,0.};
    double load=0.,lateral_load=0.,max_rim=0.,ellipse=0.,steer=0.,steer_work=0.,hold_work=0.;
    double energy_error=0.,suspension_travel=0.,brake_torque=0.;
    double brake_budget_error=0.,motor_torque=0.;
    std::int64_t steer_tick=0,saturated_ticks=0;std::size_t patches=0;
    bool valid=true,hold=false;bike::spatial::SpatialReason reason=bike::spatial::SpatialReason::none;
};
inline Point observe(runtime::NativeRideRuntime& rt) {
    Point p;auto& stepper=SpatialRideAccess::stepper(rt);auto& physical=SpatialRideAccess::physical(rt);
    auto* m=stepper.model();auto* d=stepper.data();
    const auto& topology=*physical.topology();
    const auto root=bike::spatial::body_pose(*m,*d,topology.root);
    const auto attitude=bike::spatial::euler_zyx(root.quat_world_from_body_wxyz);
    p.time=d->time;p.position=root.position_world_m;p.roll=attitude.roll_rad;p.yaw=attitude.yaw_rad;
    p.quaternion=root.quat_world_from_body_wxyz;
    p.yaw_rate=root.angular_velocity_world_rad_s[2];p.speed=physical.longitudinal_speed_mps();
    engine::subtree_vel(m,d);
    const auto subtree_mass=model_access::readonly_buffer(m->body_subtreemass,m->nbody);
    const auto subtree_com=model_access::readonly_buffer(d->subtree_com,3*m->nbody);
    const auto subtree_velocity=model_access::readonly_buffer(d->subtree_linvel,3*m->nbody);
    const auto subtree_momentum=model_access::readonly_buffer(d->subtree_angmom,3*m->nbody);
    const auto body_com=model_access::readonly_buffer(d->xipos,3*m->nbody);
    const auto gravity_vector=std::span<const double,3>{m->opt.gravity};
    p.mass=subtree_mass[0];
    for(std::size_t k=0;k<3;++k) { p.com[k]=subtree_com[k];p.velocity[k]=subtree_velocity[k];p.momentum[k]=subtree_momentum[k];p.force[k]=p.mass*gravity_vector[k]; }
    auto& tire=stepper.spatial_tire_writer();
    // A zero-interval observational force query gives endpoint kinematics.
    // It cannot advance material, controller, brake or accounting owners.
    std::vector<double> force_probe(static_cast<std::size_t>(m->nv),0.);
    if(tire.try_compute_into(d,0.,false,force_probe)!=CoreStatus::ok)p.valid=false;
    p.valid=p.valid&&tire.last_reason()==bike::pneumatic3d::ReasonCode3d::none;
    for(std::size_t wheel=0;wheel<2;++wheel) {
        std::array<double,6> wheel_velocity{};
        struct Call {const mjModel* m;const mjData* d;int body;double* result;} call{m,d,topology.wheels[wheel].wheel_body_id,wheel_velocity.data()};
        engine::ErrorBuffer error;
        if(!engine::invoke([](void* raw) noexcept {const auto& c=*static_cast<Call*>(raw);mj_objectVelocity(c.m,c.d,mjOBJ_BODY,c.body,c.result,0);},&call,error))
            engine::throw_failure(error);
        const Vec3 angular{wheel_velocity[0],wheel_velocity[1],wheel_velocity[2]};
        const Vec3 linear{wheel_velocity[3],wheel_velocity[4],wheel_velocity[5]};
        const auto wheel_com_base=3*static_cast<std::size_t>(call.body);
        Vec3 wheel_com{};for(std::size_t k=0;k<3;++k)wheel_com[k]=body_com[wheel_com_base+k];
        p.patches+=tire.model().patches(wheel).size();
        for(const auto& w:tire.wrenches(wheel))if(w.valid) {
            p.force=add(p.force,w.force_world_n);
            p.tyre_force=add(p.tyre_force,w.force_world_n);
            p.moment=add(p.moment,add(cross(sub(w.point_world_m,p.com),w.force_world_n),w.couple_world_nm));
            p.load+=w.normal_load_n;
            p.lateral_load+=w.fy_n;
            const auto& spec=tire.model().spec().brush[wheel];
            Vec3 point=w.point_world_m,normal{};
            const auto patch=std::ranges::find_if(tire.model().patches(wheel),[&](const auto& q){return q.id==w.patch_id;});
            if(tire.model().spec().tier==bike::spatial::TyreTier::fast) {
                if(patch==tire.model().patches(wheel).end())throw std::logic_error("wrench has no geometric patch");
                point=patch->centroid_world_m;normal=patch->normal_world;
            } else {
                const auto sample=SpatialRideAccess::terrain_view(rt).sample(point[0],point[1]);
                if(sample.status!=bike::terrain::QueryStatus3d::ok)throw std::logic_error("wrench surface unavailable");
                normal=sample.normal_world;
            }
            const auto velocity=add(linear,cross(angular,sub(point,wheel_com)));
            const double slip=norm(sub(velocity,scale(normal,dot(velocity,normal))));
            const auto& material=SpatialRideAccess::surface(rt,w.surface_id);
            const double mu=material.stribeck_speed_mps>0.?material.mu_slide+(material.mu_peak-material.mu_slide)*
                std::exp(-std::pow(slip/material.stribeck_speed_mps,2)):
                (slip==0.?material.mu_peak:material.mu_slide);
            const double cx=mu*spec.mu_x_scale*w.normal_load_n,cy=mu*spec.mu_y_scale*w.normal_load_n;
            if(cx>0.&&cy>0.)p.ellipse=std::max(p.ellipse,(w.fx_n/cx)*(w.fx_n/cx)+(w.fy_n/cy)*(w.fy_n/cy));
            else if(w.fx_n!=0. || w.fy_n!=0.)p.ellipse=std::numeric_limits<double>::infinity();
        }
        const auto& c=tire.state().carcass[wheel];
        for(double d0:c.deflection_m)p.max_rim=std::max(p.max_rim,d0-tire.model().spec().carcass[wheel].rim_clearance_m);
    }
    const auto& s=SpatialRideAccess::causal(rt);
    p.hold=s.hold.active;p.hold_work=s.hold.work_j;
    p.steer=s.steer.last_applied_nm;p.steer_work=s.steer.work_j;
    p.steer_tick=s.steer.next_tick;p.saturated_ticks=s.steer.saturated_ticks;
    p.reason=s.outcome.latched.reason;p.valid=p.valid&&s.outcome.latched.valid;
    const auto sensor_dimensions=model_access::readonly_buffer(m->sensor_dim,m->nsensor);
    const auto sensor_types=model_access::readonly_buffer(m->sensor_type,m->nsensor);
    const auto sensor_addresses=model_access::readonly_buffer(m->sensor_adr,m->nsensor);
    const auto sensor_data=model_access::readonly_buffer(d->sensordata,m->nsensordata);
    for(std::size_t sensor=0;sensor<sensor_dimensions.size();++sensor) {
        if(sensor_dimensions[sensor]!=3)continue;
        const auto value=sensor_data.subspan(static_cast<std::size_t>(sensor_addresses[sensor]),3);
        if(sensor_types[sensor]==mjSENS_ACCELEROMETER)std::ranges::copy(value,p.accel_sensor.begin());
        if(sensor_types[sensor]==mjSENS_GYRO)std::ranges::copy(value,p.gyro_sensor.begin());
    }
    const auto joint_type=model_access::readonly_buffer(m->jnt_type,m->njnt);
    const auto qpos_address=model_access::readonly_buffer(m->jnt_qposadr,m->njnt);
    const auto qpos=model_access::readonly_buffer(d->qpos,m->nq);
    const auto reference_qpos=model_access::readonly_buffer(m->qpos0,m->nq);
    const auto constraint_force=model_access::readonly_buffer(d->qfrc_constraint,m->nv);
    for(std::size_t j=0;j<joint_type.size();++j)if(joint_type[j]==mjJNT_SLIDE) {
        const auto address=static_cast<std::size_t>(qpos_address[j]);
        p.suspension_travel=std::max(p.suspension_travel,std::abs(qpos[address]-reference_qpos[address]));
    }
    for(const auto& wheel:topology.wheels)p.brake_torque=std::max(p.brake_torque,std::abs(constraint_force[static_cast<std::size_t>(wheel.spin.dof_adr)]));
    p.brake_budget_error=SpatialRideAccess::brake_budget_error(rt);
    const auto& drive=stepper.drive().diagnostics(false);
    if(drive.present("motor_torque_nm"))p.motor_torque=std::get<double>(drive.at("motor_torque_nm"));
    else if(p.time>0.)throw std::logic_error("qualification motor torque lane missing");
    p.energy_error=SpatialRideAccess::energy_error(rt);
    return p;
}
struct Report {
    struct Event {std::string kind;double time=0.;};
    double requested_duration=0.,valid_duration=0.,requested_distance=0.,completed_distance=0.;
    std::vector<Point> points;std::uint64_t invalid_ticks=0,hold_ticks=0;
    std::vector<Event> events;
    double mean_load=0.,peak_load=0.;std::uint64_t samples=0;
    // Qualification limits are reduced over every accepted physics interval;
    // points may be decimated for trajectories and must not hide excursions.
    double max_abs_steer=0.,max_abs_motor_torque=0.,max_brake_budget_error=0.;
    double max_ellipse=0.,max_rim=0.,max_abs_hold_work=0.;
    std::optional<std::string> terminal_reason;
    std::string profile_digest,steer_digest;
};
inline void record_limits(Report& report,const Point& point) {
    const auto maximum=[](double previous,double value) {
        return std::isfinite(value)?std::max(previous,value):std::numeric_limits<double>::infinity();
    };
    report.max_abs_steer=maximum(report.max_abs_steer,std::abs(point.steer));
    report.max_abs_motor_torque=maximum(report.max_abs_motor_torque,std::abs(point.motor_torque));
    report.max_brake_budget_error=maximum(report.max_brake_budget_error,point.brake_budget_error);
    report.max_ellipse=maximum(report.max_ellipse,point.ellipse);
    report.max_rim=maximum(report.max_rim,point.max_rim);
    report.max_abs_hold_work=maximum(report.max_abs_hold_work,std::abs(point.hold_work));
}
using BeforeStep=std::function<void(runtime::NativeRideRuntime&,double)>;
inline Report run(OwnedStartup& fixture,double duration,std::span<const Demand> trace,
                  double requested_distance=0.,std::int64_t decimation=1,BeforeStep before={},bool close_period=true) {
    auto& rt=*fixture.runtime;
    if(decimation<1 || duration<=0. || requested_distance<0.)throw std::invalid_argument("invalid qualification interval");
    const double dt=fixture.bootstrap->config.timestep_s;
    Report report;report.requested_duration=duration;report.requested_distance=requested_distance;
    report.profile_digest=fixture.bootstrap->config.spatial->tyre.lateral_coefficients_sha256;
    report.steer_digest=fixture.bootstrap->config.spatial->steer.profile_sha256;
    const auto start=rt.snapshot();const double start_x=observe(rt).position[0];
    auto preceding=observe(rt);
    const auto steps=static_cast<std::int64_t>(std::llround(duration/dt));
    // The last step reports valid_duration=duration, so a non-whole-step duration would count unsimulated time as valid.
    if(std::abs(static_cast<double>(steps)*dt-duration)>1e-9*std::max(1.,duration))
        throw std::invalid_argument("qualification duration must be an integer number of physics steps");
    std::size_t at=0;
    for(std::int64_t i=1;i<=steps;++i) {
        const double time=start.time_s+static_cast<double>(i-1)*dt;
        while(at+1<trace.size() && trace[at+1].time_s<=time)++at;
        const Demand demand=trace.empty()?Demand{}:trace[at];
        runtime::RideControl control;
        control.motor_torque_nm=85.*demand.torque_fraction;control.motor_limit_nm=85.;control.human_torque_nm=0.;
        if(before)before(rt,time);
        const auto result=rt.advance(start.step+i,control,demand.brake_fraction,demand.brake_fraction,std::nullopt);
        const auto point=observe(rt);
        record_limits(report,point);
        if(!point.valid)++report.invalid_ticks;if(point.hold)++report.hold_ticks;
        if(point.patches!=preceding.patches)report.events.push_back({"patches:"+std::to_string(point.patches),point.time});
        if((point.load>1e-6)!=(preceding.load>1e-6))report.events.push_back({point.load>1e-6?"landing":"airborne",point.time});
        if((point.max_rim>0.)!=(preceding.max_rim>0.))report.events.push_back({point.max_rim>0.?"rim_on":"rim_off",point.time});
        report.mean_load+=point.load;report.peak_load=std::max(report.peak_load,point.load);++report.samples;
        // The last step closes the requested interval exactly: i*dt for i==steps can round a
        // few ulp below `duration`, which would make valid_duration==requested_duration fail spuriously.
        if(point.valid && report.invalid_ticks==0)report.valid_duration=i==steps?duration:std::min(duration,static_cast<double>(i)*dt);
        report.completed_distance=point.position[0]-start_x;
        const bool distance_complete=requested_distance>0. && report.completed_distance>=requested_distance;
        if(i%decimation==0 || result.outcome || i==steps || distance_complete)report.points.push_back(point);
        if(result.outcome) { report.terminal_reason=result.outcome;report.events.push_back({*result.outcome,point.time});break; }
        if(distance_complete)break;
        preceding=point;
    }
    if(report.samples)report.mean_load/=static_cast<double>(report.samples);
    if(close_period)rt.flush();if(!report.points.empty())report.points.back().energy_error=SpatialRideAccess::energy_error(rt);return report;
}
inline double circle_radius(std::span<const Point> points) {
    if(points.size()<3)throw std::invalid_argument("circle fit needs three samples");
    std::array<std::array<double,4>,3> a{};
    const double x0=points.front().com[0],y0=points.front().com[1];
    for(const auto& p:points) {
        const double x=p.com[0]-x0,y=p.com[1]-y0;const std::array v{x,y,1.};
        const double rhs=-(x*x+y*y);
        for(unsigned i=0;i<3;++i){for(unsigned j=0;j<3;++j)a[i][j]+=v[i]*v[j];a[i][3]+=v[i]*rhs;}
    }
    for(unsigned k=0;k<3;++k) {
        unsigned pivot=k;for(unsigned i=k+1;i<3;++i)if(std::abs(a[i][k])>std::abs(a[pivot][k]))pivot=i;
        if(std::abs(a[pivot][k])<1e-12)throw std::invalid_argument("circle fit is rank deficient");
        for(unsigned j=k;j<4;++j)std::swap(a[k][j],a[pivot][j]);
        const double d=a[k][k];for(unsigned j=k;j<4;++j)a[k][j]/=d;
        for(unsigned i=0;i<3;++i)if(i!=k){const double f=a[i][k];for(unsigned j=k;j<4;++j)a[i][j]-=f*a[k][j];}
    }
    const double radius2=.25*(a[0][3]*a[0][3]+a[1][3]*a[1][3])-a[2][3];
    if(!(radius2>0.))throw std::invalid_argument("circle fit has nonpositive radius");
    return std::sqrt(radius2);
}
inline void print_report(std::string_view suite,const Report& r) {
    std::cout << "{\"suite\":\"" << suite << "\",\"requested_duration_s\":" << r.requested_duration
              << ",\"valid_duration_s\":" << r.valid_duration << ",\"completed_distance_m\":" << r.completed_distance
              << ",\"invalid_ticks\":" << r.invalid_ticks << ",\"bootstrap_hold_scored_ticks\":" << r.hold_ticks
              << ",\"profile_digest\":\"" << r.profile_digest << "\",\"steer_digest\":\"" << r.steer_digest
              << "\",\"mean_load_n\":" << r.mean_load << ",\"peak_load_n\":" << r.peak_load << "}\n";
}
} // namespace spatial_tests::ride
