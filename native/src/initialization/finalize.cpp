#include "context.hpp"
#include "state_file.hpp"
#include "../model/mapping.hpp"
#include "../numeric_sum.hpp"
#include "../runtime/config.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/pneumatic_3d_tire.hpp"
#include "../spatial/frames.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <ranges>
#include <stdexcept>

namespace bike::initialization {
namespace {
void spatial_initial_velocity(SetupContext& context,const runtime::WheelSnapshots& snapshots) {
    const auto& topology=*context.topology;
    const auto* m=context.stepper.model();auto* d=context.stepper.data();
    const auto pose=spatial::body_pose(*m,*d,topology.root);
    const auto sample=context.terrain3d->query_view().sample(pose.position_world_m[0],pose.position_world_m[1]);
    if(sample.status!=terrain::QueryStatus3d::ok)
        throw std::invalid_argument("initial_speed.spatial.terrain: root outside physical domain");
    auto direction=spatial::rotate_body_to_world(pose.quat_world_from_body_wxyz,{1.,0.,0.});
    const auto& normal=sample.normal_world;
    double normal_component=0.;for(std::size_t i=0;i<3;++i)normal_component+=direction[i]*normal[i];
    double length2=0.;for(std::size_t i=0;i<3;++i){direction[i]-=normal_component*normal[i];length2+=direction[i]*direction[i];}
    if(!(length2>1e-24))throw std::invalid_argument("initial_speed.spatial.heading: singular terrain projection");
    for(double& component:direction)component/=std::sqrt(length2);
    const auto place=[&](const spatial::RootBinding& root) {
        if(root.kind==spatial::RootKind::free6 && root.free_joint_id>=0 && m->jnt_type[root.free_joint_id]==mjJNT_FREE) {
            // Free translational velocities are world coordinates; angular
            // velocities remain untouched and are observed through accessors.
            for(int i=0;i<3;++i)d->qvel[root.free_dof_adr+i]=context.physics.initial_speed_mps*direction[static_cast<std::size_t>(i)];
        } else for(std::size_t i=0;i<3;++i)if(root.scalar[i].dof_adr>=0)
            d->qvel[root.scalar[i].dof_adr]=context.physics.initial_speed_mps*direction[i];
    };
    place(topology.root);for(const auto& root:topology.rider_roots)place(root);
    context.stepper.note_posvel_stale();context.stepper.forward();
    for(std::size_t side=0;side<2;++side) {
        if(!snapshots[side])throw std::invalid_argument("initial_speed.spatial.tire: snapshot missing");
        const auto& wheel=topology.wheels[side];
        const auto q=static_cast<std::size_t>(wheel.hub_body_id)*4;
        const auto axle=spatial::rotate_body_to_world({d->xquat[q],d->xquat[q+1],d->xquat[q+2],d->xquat[q+3]},wheel.axle_axis_hub);
        spatial::Vec3 tangent{axle[1]*normal[2]-axle[2]*normal[1],
            axle[2]*normal[0]-axle[0]*normal[2],axle[0]*normal[1]-axle[1]*normal[0]};
        const double norm=std::sqrt(tangent[0]*tangent[0]+tangent[1]*tangent[1]+tangent[2]*tangent[2]);
        if(!(norm>1e-12))throw std::invalid_argument("initial_speed.spatial.tire: axle parallel to surface normal");
        for(double& component:tangent)component/=norm;
        const auto center=static_cast<std::size_t>(wheel.wheel_body_id)*3;
        const auto v=model::point_velocity(*m,*d,wheel.wheel_body_id,{d->xpos[center],d->xpos[center+1],d->xpos[center+2]});
        const auto hub_omega=model::body_angular_velocity(*m,*d,wheel.hub_body_id);
        const int geom=side==0?context.front_geom_id:context.rear_geom_id;
        const double radius=snapshots[side]->effective_radius_m>0.?snapshots[side]->effective_radius_m:m->geom_size[3*geom];
        double roll=0.,hub_rate=0.;for(std::size_t i=0;i<3;++i){roll+=v[i]*tangent[i];hub_rate+=hub_omega[i]*axle[i];}
        d->qvel[wheel.spin.dof_adr]=roll/radius-hub_rate;
    }
    const auto& drive=context.physics.drive;
    const double ratio=static_cast<double>(drive.gearing.front_teeth)/static_cast<double>(drive.gearing.rear_teeth);
    const double rate=d->qvel[topology.wheels[1].spin.dof_adr]/ratio;
    bool pedaling=(context.physics.drive_mode=="crank_effort" || context.physics.drive_mode=="articulated_effort")&&drive.human_torque_nm>0.;
    if(drive.pedaling.enabled && std::abs(rate)*60./(2.*std::numbers::pi)>=drive.pedaling.coast_above_rpm)pedaling=false;
    if(pedaling) {
        d->qvel[joint_address(context,"crank_spin").dof]=rate;
        for(std::string_view name:{"drive_shaft_spin","rotor_spin","cassette_spin"})
            if(mj_name2id(m,mjOBJ_JOINT,std::string(name).c_str())>=0)
                d->qvel[joint_address(context,name).dof]=name=="cassette_spin"?d->qvel[topology.wheels[1].spin.dof_adr]:rate;
        const auto axle=spatial::rotate_body_to_world(pose.quat_world_from_body_wxyz,{0.,1.,0.});
        const auto omega=model::body_angular_velocity(*m,*d,topology.root.body_id);
        const double frame_rate=omega[0]*axle[0]+omega[1]*axle[1]+omega[2]*axle[2];
        d->qvel[joint_address(context,"pedal_front_spin").dof]=-rate-frame_rate;
        d->qvel[joint_address(context,"pedal_rear_spin").dof]=-rate-frame_rate;
        if(context.physical.rider_present())context.stepper.mutate([&]{context.physical.rider_control().initialize_velocity(d);});
    }
    context.stepper.note_posvel_stale();context.stepper.forward();
}
void initial_speed(SetupContext& context,const runtime::WheelSnapshots& snapshots){
    if(context.topology){spatial_initial_velocity(context,snapshots);return;}
    mjModel const* m=context.stepper.model();mjData* d=context.stepper.data();auto const velocity=std::views::counted(d->qvel,m->nv);
    std::array<double,3> tangent{};
    for(const auto& snapshot:snapshots)if(snapshot)for(const auto& p:snapshot->patches)if(p.working_surface){tangent[0]+=p.normal_load_n*p.normal[2];tangent[2]-=p.normal_load_n*p.normal[0];}
    const double length=std::sqrt(tangent[0]*tangent[0]+tangent[2]*tangent[2]);
    if(length>0.){tangent[0]/=length;tangent[2]/=length;}else tangent={1.,0.,0.};
    for(std::string_view const root:{"root","rider_root"}){
        if(root=="rider_root"&&!context.physical.rider_present())continue;
        velocity[static_cast<std::size_t>(joint_address(context,std::string(root)+"_x").dof)]=context.physics.initial_speed_mps*tangent[0];
        velocity[static_cast<std::size_t>(joint_address(context,std::string(root)+"_z").dof)]=context.physics.initial_speed_mps*tangent[2];
    }
    context.stepper.note_posvel_stale();context.stepper.forward();
    const auto bodies=std::views::counted(m->geom_bodyid,m->ngeom);
    const auto centers=std::views::counted(d->geom_xpos,3*m->ngeom);
    const auto sizes=std::views::counted(m->geom_size,3*m->ngeom);
    for(std::size_t side=0;side<2;++side){
        const auto& snap=snapshots[side];if(!snap)throw std::invalid_argument("initial speed requires both physical tire snapshots");
        const auto geom=static_cast<std::size_t>(side==0?context.front_geom_id:context.rear_geom_id);
        const int body=bodies[geom];
        const std::array<double,3> point{centers[3*geom],centers[3*geom+1],centers[3*geom+2]};
        const std::array<double,3> direction=snap->patches.empty()?std::array<double,3>{1.,0.,0.}:std::array<double,3>{snap->patches.front().normal[2],0.,-snap->patches.front().normal[0]};
        const double radius=snap->effective_radius_m>0.?snap->effective_radius_m:sizes[3*geom];
        const auto v=model::point_velocity(*m,*d,body,point);const auto omega=model::body_angular_velocity(*m,*d,body);
        const auto dof=static_cast<std::size_t>(joint_address(context,side==0?"front_wheel_spin":"rear_wheel_spin").dof);
        const double roll=v[0]*direction[0]+v[1]*direction[1]+v[2]*direction[2],parent_omega=omega[1]-velocity[dof];
        velocity[dof]=roll/radius-parent_omega;
    }
    const auto& drive=context.physics.drive;
    const double ratio=static_cast<double>(drive.gearing.front_teeth)/static_cast<double>(drive.gearing.rear_teeth);
    const double rate=velocity[static_cast<std::size_t>(joint_address(context,"rear_wheel_spin").dof)]/ratio;
    bool pedaling=(context.physics.drive_mode=="crank_effort"||context.physics.drive_mode=="articulated_effort")&&drive.human_torque_nm>0.;
    if(drive.pedaling.enabled&&std::abs(rate)*60./(2.*std::numbers::pi)>=drive.pedaling.coast_above_rpm)pedaling=false;
    if(pedaling){
        velocity[static_cast<std::size_t>(joint_address(context,"crank_spin").dof)]=rate;
        for(std::string_view const name:{"drive_shaft_spin","rotor_spin","cassette_spin"})if(mj_name2id(m,mjOBJ_JOINT,std::string(name).c_str())>=0)
            velocity[static_cast<std::size_t>(joint_address(context,name).dof)]=name=="cassette_spin"?velocity[static_cast<std::size_t>(joint_address(context,"rear_wheel_spin").dof)]:rate;
        const double frame_rate=model::body_angular_velocity(*m,*d,context.frame_body_id)[1];
        velocity[static_cast<std::size_t>(joint_address(context,"pedal_front_spin").dof)]=-rate-frame_rate;
        velocity[static_cast<std::size_t>(joint_address(context,"pedal_rear_spin").dof)]=-rate-frame_rate;
        if(context.physical.rider_present())context.stepper.mutate([&]{context.physical.rider_control().initialize_velocity(d);});
    }
    context.stepper.note_posvel_stale();context.stepper.forward();
}
}
void finalize_initial_state(SetupContext& context,const runtime::RuntimeConfig& config){
    require_initializing(context);
    if(context.topology && (!context.stepper.has_spatial_tire() || !context.physical.spatial_state()))
        throw std::invalid_argument("bootstrap.spatial.tire: producer state missing");
    if(!context.equilibrium)throw std::invalid_argument("cannot finalize an incomplete equilibrium");
    auto& physical=context.physical;auto& stepper=context.stepper;mjData* d=stepper.data();mjModel* m=stepper.model();
    d->time=0.;physical.step_=0;physical.generation_=1;physical.initializing_=false;
    physical.control_clock_.reset();physical.contact_query_.restore({});physical.probe_query_.restore({});
    for(auto& filter:physical.filters_)filter.restore({});
    physical.balance_monitor_.restore({});physical.crash_detector_.set_event(std::nullopt);
    physical.rider_intent_.reset();physical.intent_signals_={};physical.applied_control_={};physical.held_rider_terms_.reset();
    physical.energy_={};physical.rollback_hold_=false;physical.research_accounting_valid_=false;physical.last_force_sample_.reset();physical.last_constraint_snapshot_={};physical.accumulator_.clear();
    physical.measurement_decimation_=config.measurement_decimation;
    if(stepper.has_cruise())stepper.cruise_reset();
    stepper.drive().restart_clock();
    if(context.topology) {
        auto material=stepper.spatial_tire_writer().state();
        material.accepted_interval=-1;
        for(auto& carcass:material.carcass)carcass.dissipated_j=0.;
        for(auto& brush:material.brush){brush.sliding_loss_j=0.;brush.convective_release_j=0.;}
        stepper.restore_spatial_tire(material);
    }
    if(stepper.has_tire())stepper.tire_writer().restart_clock();
    if(stepper.has_rider_contacts())stepper.rider_contacts_restart_clock();
    std::ranges::fill(std::views::counted(d->qfrc_applied,m->nv),0.);std::ranges::fill(std::views::counted(d->ctrl,m->nu),0.);
    stepper.note_posvel_stale();stepper.forward();apply_initial_forces(context,false);stepper.forward();
    if(stepper.has_tire()){
        std::vector<double> forces(static_cast<std::size_t>(m->nv));
        stepper.mutate([&]{stepper.tire_writer().compute_into(d,m->opt.timestep,forces);});
        stepper.tire_writer().clear_settled_loss();stepper.tire_writer().restart_clock();
    }
    if(stepper.has_rider_contacts())stepper.mutate([&]{stepper.rider_contacts_initialize_settled_state();});
    physical.contacts_=physical.refresh_contacts(false,std::nullopt,false);
    context.equilibrium->total_vertical_force_n=0.;
    for(std::size_t side=0;side<2;++side){
        const auto& snap=physical.snapshots_[side];if(!snap)throw std::invalid_argument("settled model lacks a wheel contact snapshot");
        numeric::PythonSum normal,vertical;for(const auto& p:snap->patches){normal.add(p.normal_load_n);vertical.add(p.native_world_force_n ? (p.native_world_force_n.value())[2] : p.normal_load_n*p.normal[2]-p.tangent_force_n*p.normal[0]);}// every addend is a float patch field or float product — exact under CPython's float branch
        if(side==0)context.equilibrium->static_front_load_n=normal.total();
        else context.equilibrium->static_rear_load_n=normal.total();
        context.equilibrium->total_vertical_force_n+=vertical.total();
    }
    if(physical.rider_present()){
        auto controller=physical.rider_control().state();
        if(controller.allocation_diagnostics.legacy && !controller.allocation_diagnostics.legacy->disabled) {
            // The allocator records pre-clamp latent activation separately
            // from excitation and delivered torque. Preserve that state at
            // the equilibrium-to-ride transition together with its warm start.
            controller.active_state=controller.allocation_diagnostics.legacy->activation_state;
        } else {
            controller.active_state.clear();
            for(const auto& joint:physical.rider_control().joints()){
                const auto it=std::ranges::find(controller.effort_diagnostics.active_request_nm,joint.name,&std::pair<std::string,double>::first);
                controller.active_state.push_back(it==controller.effort_diagnostics.active_request_nm.end()?0.:it->second);
            }
        }
        controller.activation_time_s.reset();physical.rider_control().restore(controller);
    }
    initial_speed(context,physical.snapshots_);stepper.forward();
    if(context.initial_state)context.initial_state->restore(context);
    if(stepper.has_rider_contacts())stepper.mutate([&]{(void)stepper.rider_contacts_settle(std::nullopt,false,nullptr);});
    stepper.mutate([&]{
        auto [mass,elastic,total]=physical.energy_state();numeric::PythonSum sum;
        // sum(elastic.values()): stored_terms entries are Python floats —
        // the compensated float branch is the oracle's whole path.
        for(const auto& [name,value]:elastic){(void)name;sum.add(std::get<double>(value.value));}
        physical.energy_.initial_energy_j=total;physical.energy_.energy_scale_j=std::max(1.,mass.kinetic_energy_j+sum.total());
        physical.energy_.initial_battery_j=stepper.drive().state().battery.energy_j;physical.energy_.mechanical_energy_j=total;physical.energy_.elastic_energy_j=std::move(elastic);physical.energy_.residual_j=0.;
        physical.update_compiled_com_marker();
        if(context.physics.seated_climb.enabled){auto sensors=physical.sensor_channels(stepper.qvel(),stepper.drive().diagnostics(true).to_map());auto tires=physical.tire_channels(physical.snapshots_,stepper.qvel());sensors.emplace_back("tires",runtime::Wire{std::move(tires)});physical.update_intent_signals(sensors);}
    });
    physical.research_accounting_valid_=true;context.completed=true;
}
} // namespace bike::initialization
