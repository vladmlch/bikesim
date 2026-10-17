#include "context.hpp"
#include "state_file.hpp"
#include "../model/mapping.hpp"
#include "../numeric_sum.hpp"
#include "../runtime/config.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"
#include "../writers/drivetrain.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <ranges>
#include <stdexcept>

namespace bike::initialization {
namespace {
void initial_speed(SetupContext& context,const runtime::WheelSnapshots& snapshots){
    mjModel* m=context.stepper.model();mjData* d=context.stepper.data();auto velocity=std::views::counted(d->qvel,m->nv);
    std::array<double,3> tangent{};
    for(const auto& snapshot:snapshots)if(snapshot)for(const auto& p:snapshot->patches)if(p.working_surface){tangent[0]+=p.normal_load_n*p.normal[2];tangent[2]-=p.normal_load_n*p.normal[0];}
    const double length=std::sqrt(tangent[0]*tangent[0]+tangent[2]*tangent[2]);
    if(length>0.){tangent[0]/=length;tangent[2]/=length;}else tangent={1.,0.,0.};
    for(std::string_view root:{"root","rider_root"}){
        if(root=="rider_root"&&!context.physical.rider_present())continue;
        velocity[static_cast<std::size_t>(joint_address(context,std::string(root)+"_x").dof)]=context.physics.initial_speed_mps*tangent[0];
        velocity[static_cast<std::size_t>(joint_address(context,std::string(root)+"_z").dof)]=context.physics.initial_speed_mps*tangent[2];
    }
    context.stepper.note_posvel_stale();context.stepper.forward();
    const auto bodies=std::views::counted(m->geom_bodyid,m->ngeom);
    const auto centers=std::views::counted(d->geom_xpos,3*m->ngeom);
    const auto sizes=std::views::counted(m->geom_size,3*m->ngeom);
    for(std::size_t side=0;side<2;++side){
        if(!snapshots[side])throw std::invalid_argument("initial speed requires both physical tire snapshots");
        const auto& snap=*snapshots[side];
        const auto geom=static_cast<std::size_t>(side==0?context.front_geom_id:context.rear_geom_id);
        const int body=bodies[geom];
        const std::array<double,3> point{centers[3*geom],centers[3*geom+1],centers[3*geom+2]};
        const std::array<double,3> direction=snap.patches.empty()?std::array<double,3>{1.,0.,0.}:std::array<double,3>{snap.patches.front().normal[2],0.,-snap.patches.front().normal[0]};
        const double radius=snap.effective_radius_m>0.?snap.effective_radius_m:sizes[3*geom];
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
        for(std::string_view name:{"drive_shaft_spin","rotor_spin","cassette_spin"})if(mj_name2id(m,mjOBJ_JOINT,std::string(name).c_str())>=0)
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
        if(!physical.snapshots_[side])throw std::invalid_argument("settled model lacks a wheel contact snapshot");
        numeric::PythonSum normal,vertical;for(const auto& p:physical.snapshots_[side]->patches){normal.add(p.normal_load_n);vertical.add(p.native_world_force_n ? (*p.native_world_force_n)[2] : p.normal_load_n*p.normal[2]-p.tangent_force_n*p.normal[0]);}
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
        for(const auto& [name,value]:elastic){(void)name;sum.add(std::get<double>(value.value));}
        physical.energy_.initial_energy_j=total;physical.energy_.energy_scale_j=std::max(1.,mass.kinetic_energy_j+sum.total());
        physical.energy_.initial_battery_j=stepper.drive().state().battery.energy_j;physical.energy_.mechanical_energy_j=total;physical.energy_.elastic_energy_j=std::move(elastic);physical.energy_.residual_j=0.;
        physical.update_compiled_com_marker();
        if(context.physics.seated_climb.enabled){auto sensors=physical.sensor_channels(stepper.qvel(),stepper.drive().diagnostics(true).to_map());auto tires=physical.tire_channels(physical.snapshots_,stepper.qvel());sensors.emplace_back("tires",runtime::Wire{std::move(tires)});physical.update_intent_signals(sensors);}
    });
    physical.research_accounting_valid_=true;context.completed=true;
}
} // namespace bike::initialization
