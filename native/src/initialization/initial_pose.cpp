#include "context.hpp"
#include "../engine_call.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"
#include "../tyre/profile.hpp"
#include "../writers/drivetrain.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <ranges>
#include <stdexcept>

namespace bike::initialization {

void require_initializing(const SetupContext& context) {
    if (!context.initializing || context.physical.step() != 0)
        throw std::invalid_argument("equilibrium operations are confined to initialization");
}
JointAddress joint_address(const SetupContext& context, std::string_view name) {
    const mjModel* m = context.stepper.model();
    const int id = mj_name2id(m, mjOBJ_JOINT, std::string(name).c_str());
    if (id < 0)
        throw std::invalid_argument("model has no joint '" + std::string(name) + "'; equilibrium requires a planar root");
    const auto types=std::views::counted(m->jnt_type,m->njnt);
    const auto index=static_cast<std::size_t>(id);
    if(types[index]!=mjJNT_SLIDE && types[index]!=mjJNT_HINGE)
        throw std::invalid_argument("equilibrium requires scalar planar joint '"+std::string(name)+"'");
    return {.joint=id,.qpos=std::views::counted(m->jnt_qposadr,m->njnt)[index],
               .dof=std::views::counted(m->jnt_dofadr,m->njnt)[index]};
}

void initialize_pose(SetupContext& context) {
    require_initializing(context);
    auto& stepper=context.stepper;
    mjModel* m=stepper.model();mjData* d=stepper.data();
    const auto vertices=context.physical.terrain_vertices();
    if(vertices.size()<2 || !std::isfinite(context.start_x_m) || !std::isfinite(context.ground_z_m))
        throw std::invalid_argument("initial pose requires finite source geometry and a compiled road");
    for(std::size_t i=0;i<vertices.size();++i)
        if(!std::isfinite(vertices[i][0])||!std::isfinite(vertices[i][1])||(i>0&&vertices[i][0]<=vertices[i-1][0]))
            throw std::invalid_argument("initial pose road vertices must be finite and strictly increasing");
    for(int const id:{context.front_geom_id,context.rear_geom_id})
        if(id<0||id>=m->ngeom)throw std::invalid_argument("initial pose wheel geometry id is invalid");
    const JointAddress x=joint_address(context,"root_x"),z=joint_address(context,"root_z"),pitch=joint_address(context,"root_pitch");
    const JointAddress crank=joint_address(context,"crank_spin"),front=joint_address(context,"pedal_front_spin"),rear=joint_address(context,"pedal_rear_spin");
    auto locate=[&](double position){
        const auto it=std::ranges::lower_bound(vertices,position,{},[](const auto& a){return a[0];});
        const auto distance=std::ranges::distance(vertices.begin(),it);
        return static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(distance-1,0,static_cast<std::ptrdiff_t>(vertices.size())-2));
    };
    auto const height=[&](double position){
        if(position<=vertices.front()[0])return vertices.front()[1];
        if(position>=vertices.back()[0])return vertices.back()[1];
        const auto at=locate(position);
        return biketyre::np_interp(position,std::array{vertices[at][0],vertices[at+1][0]},std::array{vertices[at][1],vertices[at+1][1]});
    };
    const auto i=locate(context.start_x_m);
    const double slope=(vertices[i+1][1]-vertices[i][1])/(vertices[i+1][0]-vertices[i][0]);
    auto stage=std::make_unique<runtime::StepState>(context.physical.state());
    stage->initializing=true;stage->contact_query={};stage->probe_query={};
    context.physical.restore(*stage);
    stepper.mutate([&]{engine::reset_data(m,d);});
    auto const q=std::views::counted(d->qpos,m->nq);
    q[static_cast<std::size_t>(x.qpos)]=context.start_x_m;
    q[static_cast<std::size_t>(z.qpos)]=height(context.start_x_m)-context.ground_z_m+.005;
    q[static_cast<std::size_t>(pitch.qpos)]=-std::atan(slope);
    stepper.note_posvel_stale();stepper.forward();
    double clearance=0.;
    const auto centers=std::views::counted(d->geom_xpos,3*m->ngeom);
    const auto sizes=std::views::counted(m->geom_size,3*m->ngeom);
    for(int const id:{context.front_geom_id,context.rear_geom_id}) {
        const auto offset=3*static_cast<std::size_t>(id);
        clearance=std::max(clearance,height(centers[offset])+sizes[offset]*std::sqrt(1.+slope*slope)-centers[offset+2]+.005);
    }
    q[static_cast<std::size_t>(z.qpos)]+=clearance;
    const double phase=context.physics.drive.crank_phase_rad;
    q[static_cast<std::size_t>(crank.qpos)]=phase;
    q[static_cast<std::size_t>(front.qpos)]=-phase;
    q[static_cast<std::size_t>(rear.qpos)]=-phase;
    if(context.physical.rider_present()) {
        if(!context.rider_pose||context.frame_body_id<0||context.frame_body_id>=m->nbody)
            throw std::invalid_argument("initial rider pose and frame identity are required");
        stepper.note_posvel_stale();stepper.forward();
        const auto xp=std::views::counted(d->xpos,3*m->nbody);
        const auto xm=std::views::counted(d->xmat,9*m->nbody);
        const auto& hip=context.rider_pose->hip;
        const auto frame=static_cast<std::size_t>(context.frame_body_id);
        std::array<double,3> wanted{};
        for(std::size_t axis=0;axis<3;++axis){wanted[axis]=xp[3*frame+axis];for(std::size_t a=0;a<3;++a)wanted[axis]+=xm[9*frame+3*axis+a]*hip[a];}
        q[static_cast<std::size_t>(joint_address(context,"rider_root_x").qpos)]=wanted[0]-hip[0];
        q[static_cast<std::size_t>(joint_address(context,"rider_root_z").qpos)]=wanted[2]-hip[2];
        q[static_cast<std::size_t>(joint_address(context,"rider_root_pitch").qpos)]=q[static_cast<std::size_t>(pitch.qpos)];
    }
    stepper.note_posvel_stale();stepper.forward();
    stepper.mutate([&]{
        if(context.physical.rider_present()) {
            auto controller=context.physical.rider_control().state();controller.enabled=true;
            context.physical.rider_control().restore(controller);
            context.physical.rider_control().initialize(d);
            if(stepper.has_rider_contacts())stepper.rider_contacts_reset();
        }
        stepper.drive().reset();
        if(stepper.has_tire())stepper.tire_writer().reset();
    });
    stepper.note_posvel_stale();
}
} // namespace bike::initialization
