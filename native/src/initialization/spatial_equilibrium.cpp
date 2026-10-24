#include "spatial_equilibrium.hpp"

#include "bootstrap_hold.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../numerics/least_squares.hpp"
#include "../runtime/step.hpp"
#include "../spatial/frames.hpp"
#include "../stepper.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/pneumatic_3d_tire.hpp"
#include "../writers/rider_contacts.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <format>
#include <limits>
#include <memory>
#include <numeric>
#include <ranges>
#include <span>
#include <stdexcept>
#include <utility>

namespace bike::initialization {
namespace {
spatial::Vec3 difference(const spatial::Vec3& a,const spatial::Vec3& b) {
    return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};
}
std::string_view query_name(terrain::QueryStatus3d s) {
    switch(s) {
    case terrain::QueryStatus3d::ok:return "ok";
    case terrain::QueryStatus3d::miss:return "miss";
    case terrain::QueryStatus3d::outside:return "outside";
    case terrain::QueryStatus3d::invalid_input:return "invalid_input";
    case terrain::QueryStatus3d::budget_exhausted:return "budget_exhausted";
    case terrain::QueryStatus3d::origin_below_surface:return "origin_below_surface";
    }
    return "invalid_input";
}
void move_root(std::span<double> qpos,const spatial::RootBinding& root,const spatial::BodyPose& old,
               const spatial::Vec3& position,const spatial::Quat& rotation,double pitch) {
    if(root.kind==spatial::RootKind::free6) {
        const auto start=static_cast<std::size_t>(root.free_qpos_adr);
        for(std::size_t k=0;k<3;++k)qpos[start+k]=position[k];
        for(std::size_t k=0;k<4;++k)qpos[start+3+k]=rotation[k];
    } else {
        for(std::size_t k=0;k<3;++k)if(root.scalar[k].qpos_adr>=0)
            qpos[static_cast<std::size_t>(root.scalar[k].qpos_adr)]+=position[k]-old.position_world_m[k];
        if(root.scalar[3].qpos_adr>=0)qpos[static_cast<std::size_t>(root.scalar[3].qpos_adr)]=pitch;
    }
}
bool translation(const mjModel& m,int dof) {
    const auto joints=model_access::readonly_buffer(m.dof_jntid,m.nv);
    const auto types=model_access::readonly_buffer(m.jnt_type,m.njnt);
    const auto addresses=model_access::readonly_buffer(m.jnt_dofadr,m.njnt);
    const auto joint=static_cast<std::size_t>(joints[static_cast<std::size_t>(dof)]);
    const int type=types[joint];
    return type==mjJNT_SLIDE || (type==mjJNT_FREE && dof-addresses[joint]<3);
}
SpatialEquilibriumReport report(SetupContext& context,int evaluations,double tolerance) {
    const auto& m=*context.stepper.model();const auto& d=*context.stepper.data();
    SpatialEquilibriumReport out;out.evaluations=evaluations;
    const auto accelerations=context.stepper.qacc();
    std::vector<int> order(static_cast<std::size_t>(m.nv));std::iota(order.begin(),order.end(),0);
    for(int i=0;i<m.nv;++i) {
        const double a=std::abs(accelerations[static_cast<std::size_t>(i)]);
        auto& maximum=translation(m,i)?out.max_linear_accel_mps2:out.max_angular_accel_radps2;
        if(!std::isfinite(a)) {out.errors.emplace_back("nonfinite acceleration");maximum=std::numeric_limits<double>::infinity();continue;}
        maximum=std::max(maximum,a);
    }
    std::stable_sort(order.begin(),order.end(),[&](int a,int b){return std::abs(accelerations[static_cast<std::size_t>(a)])>std::abs(accelerations[static_cast<std::size_t>(b)]);});
    for(std::size_t i=0;i<std::min<std::size_t>(3,order.size());++i)out.worst_dofs.push_back(order[i]);
    const auto masses=model_access::readonly_buffer(m.body_mass,m.nbody);
    double mass=0.;for(std::size_t b=1;b<masses.size();++b)mass+=masses[b];
    double vertical=0.;spatial::Vec3 moment{};
    const auto centers=model_access::readonly_buffer(d.subtree_com,3*m.nbody);
    const spatial::Vec3 com{centers[0],centers[1],centers[2]};
    if(context.stepper.has_spatial_tire()) {
        const auto& writer=context.stepper.spatial_tire_writer();
        for(std::size_t side=0;side<2;++side) {
            for(const auto& p:writer.wrenches(side)) {
                ++out.contacts_total;
                if(!p.valid)continue;
                ++out.contacts_valid;
                vertical+=p.force_world_n[2];
                const auto arm=difference(p.point_world_m,com);
                moment[0]+=arm[1]*p.force_world_n[2]-arm[2]*p.force_world_n[1]+p.couple_world_nm[0];
                moment[1]+=arm[2]*p.force_world_n[0]-arm[0]*p.force_world_n[2]+p.couple_world_nm[1];
                moment[2]+=arm[0]*p.force_world_n[1]-arm[1]*p.force_world_n[0]+p.couple_world_nm[2];
            }
        }
    }
    const double weight=mass*std::abs(m.opt.gravity[2]);
    out.weight_residual_fraction=weight>0.?std::abs(vertical-weight)/weight:0.;
    out.roll_moment_residual_nm=moment[0];out.pitch_moment_residual_nm=moment[1];
    if(out.contacts_total==0 || out.contacts_valid!=out.contacts_total)out.errors.emplace_back("missing or invalid wheel contacts");
    if(out.max_linear_accel_mps2>tolerance || out.max_angular_accel_radps2>tolerance)
        out.errors.emplace_back("acceleration tolerance not reached");
    if(out.weight_residual_fraction>.005)out.errors.emplace_back("vertical load differs from weight by more than 0.5 percent");
    if(std::abs(moment[0])>1. || std::abs(moment[1])>1.)out.errors.emplace_back("roll/pitch external moment exceeds 1 N m");
    return out;
}
} // namespace

void spatial_initial_pose(SetupContext& context,const spatial::Topology& topology,
        const terrain::Terrain3dQueryView& view,double heading,double initial_speed) {
    require_initializing(context);
    if(!std::isfinite(heading) || !std::isfinite(initial_speed) || !std::isfinite(context.start_x_m))
        throw std::invalid_argument("spatial_initial_pose: finite heading, speed and position required");
    auto& stepper=context.stepper;auto& m=*stepper.model();auto& d=*stepper.data();
    const auto positions=model_access::readonly_buffer(d.xpos,3*m.nbody);
    const auto sizes=model_access::readonly_buffer(m.geom_size,3*m.ngeom);
    const auto qpos=model_access::mutable_buffer(d.qpos,m.nq);
    const auto joint_types=model_access::readonly_buffer(m.jnt_type,m.njnt);
    const auto joint_qpos=model_access::readonly_buffer(m.jnt_qposadr,m.njnt);
    // NOLINTNEXTLINE(modernize-make-unique) direct construction elides the large owner return into heap storage
    std::unique_ptr<runtime::StepState> stage{new runtime::StepState(context.physical.state())};
    stage->initializing=true;stage->contact_query={};stage->probe_query={};
    if(stage->spatial) {
        stage->spatial->bootstrap_phase=spatial::BootstrapPhase::settling;
        stage->spatial->hold=spatial::canonical_inactive_hold();
        stage->spatial->brake=spatial::canonical_released_brake();
        auto& steer=stage->spatial->steer;
        steer.next_tick=0;steer.fifo_head=0;steer.fifo_count=0;
        std::ranges::fill(steer.delay_fifo_nm,0.);
        steer.last_command_nm=steer.last_applied_nm=steer.work_j=0.;
        steer.saturated_ticks=0;steer.valid_mask=0;
        stage->spatial->outcome=spatial::canonical_unlatched_outcome();
    }
    context.physical.restore(*stage);
    stepper.mutate([&]{engine::reset_data(&m,&d);});
    stepper.note_posvel_stale();stepper.forward();
    const auto original=spatial::body_pose(m,d,topology.root);
    std::vector<spatial::BodyPose> riders;
    for(const auto& root:topology.rider_roots)riders.push_back(spatial::body_pose(m,d,root));
    const spatial::Quat yaw{std::cos(heading/2.),0.,0.,std::sin(heading/2.)};
    std::array<double,2> terrain_height{},radius{},forward_offset{};
    for(std::size_t side=0;side<2;++side) {
        const auto body=static_cast<std::size_t>(topology.wheels[side].wheel_body_id);
        const spatial::Vec3 hub{positions[3*body],positions[3*body+1],positions[3*body+2]};
        const auto local=spatial::rotate_world_to_body(original.quat_world_from_body_wxyz,difference(hub,original.position_world_m));
        const auto offset=spatial::rotate_body_to_world(yaw,local);
        const auto sample=view.sample(context.start_x_m+offset[0],offset[1]);
        if(sample.status!=terrain::QueryStatus3d::ok)
            throw std::invalid_argument(std::format("spatial_initial_pose: terrain sample at wheel {} is {}",side,query_name(sample.status)));
        terrain_height[side]=sample.height_m;
        const int geom=side==0?context.front_geom_id:context.rear_geom_id;
        if(geom<0 || geom>=m.ngeom)throw std::invalid_argument("spatial_initial_pose: wheel geometry missing");
        radius[side]=sizes[3*static_cast<std::size_t>(geom)];forward_offset[side]=local[0];
    }
    const double wheelbase=forward_offset[0]-forward_offset[1];
    if(!(wheelbase>0.))throw std::invalid_argument("spatial_initial_pose: nonpositive wheelbase");
    const double pitch=-std::atan2(terrain_height[0]+radius[0]-terrain_height[1]-radius[1],wheelbase);
    const auto attitude=spatial::quat_normalized(spatial::quat_multiply(yaw,{std::cos(pitch/2.),0.,std::sin(pitch/2.),0.}));
    const auto delta=spatial::quat_multiply(attitude,spatial::quat_conjugate(original.quat_world_from_body_wxyz));
    double root_height=0.;
    for(std::size_t side=0;side<2;++side) {
        const auto body=static_cast<std::size_t>(topology.wheels[side].wheel_body_id);
        const auto offset=spatial::rotate_body_to_world(delta,difference(
            {positions[3*body],positions[3*body+1],positions[3*body+2]},original.position_world_m));
        // A geometric seed only; the tangent solver determines physical sag.
        root_height+=.5*(terrain_height[side]+radius[side]-.003-offset[2]);
    }
    const spatial::Vec3 location{context.start_x_m,0.,root_height};
    move_root(qpos,topology.root,original,location,attitude,pitch);
    for(std::size_t i=0;i<riders.size();++i) {
        auto position=spatial::rotate_body_to_world(delta,difference(riders[i].position_world_m,original.position_world_m));
        for(std::size_t axis=0;axis<3;++axis)position[axis]+=location[axis];
        move_root(qpos,topology.rider_roots[i],riders[i],position,
            spatial::quat_normalized(spatial::quat_multiply(delta,riders[i].quat_world_from_body_wxyz)),pitch);
    }
    const double phase=context.physics.drive.crank_phase_rad;
    for(const auto& [name,value]:std::array<std::pair<const char*,double>,3>{{{"crank_spin",phase},{"pedal_front_spin",-phase},{"pedal_rear_spin",-phase}}}) {
        const int joint=mj_name2id(&m,mjOBJ_JOINT,name);
        if(joint>=0 && joint_types[static_cast<std::size_t>(joint)]==mjJNT_HINGE)
            qpos[static_cast<std::size_t>(joint_qpos[static_cast<std::size_t>(joint)])]=value;
    }
    stepper.note_posvel_stale();stepper.forward();
    stepper.mutate([&]{
        stepper.drive().reset();
        if(stepper.has_tire())stepper.tire_writer().reset();
        if(context.physical.rider_present()) {
            auto controller=context.physical.rider_control().state();controller.enabled=true;
            context.physical.rider_control().restore(controller);context.physical.rider_control().initialize(&d);
            if(stepper.has_rider_contacts())stepper.rider_contacts_reset();
        }
    });
    if(context.physics.spatial && topology.root.kind==spatial::RootKind::free6) {
        context.bootstrap_hold=std::make_shared<BootstrapHold>(context.physics.spatial->bootstrap,topology.root,m);
        auto hold=spatial::canonical_inactive_hold();hold.reference_position_world_m=location;
        hold.reference_quat_wxyz=attitude;hold.valid_mask=1;
        if(context.bootstrap_hold->restore(hold)!=CoreStatus::ok)throw std::invalid_argument("bootstrap.hold: invalid initial reference");
    }
    // Equilibrium probes are at rest. finalize_initial_state establishes the
    // requested moving start after release, including wheel/drive rates.
    (void)initial_speed;
    stepper.note_posvel_stale();stepper.forward();
}

double spatial_refine(SetupContext& context,int max_evaluations,double tolerance) {
    require_initializing(context);
    if(!context.topology || max_evaluations<=0 || !std::isfinite(tolerance) || tolerance<=0.)
        throw std::invalid_argument("spatial_refine: topology, positive budget and tolerance required");
    auto& stepper=context.stepper;auto& m=*stepper.model();auto& d=*stepper.data();
    const auto qpos=model_access::mutable_buffer(d.qpos,m.nq);
    const auto qvel=model_access::mutable_buffer(d.qvel,m.nv);
    const auto nv=static_cast<std::size_t>(m.nv);
    const std::vector<double> q0(stepper.qpos().begin(),stepper.qpos().end());
    std::vector<double> integration(static_cast<std::size_t>(engine::state_size(&m,mjSTATE_INTEGRATION)));
    engine::get_state(&m,&d,integration.data(),mjSTATE_INTEGRATION);
    // NOLINTNEXTLINE(modernize-make-unique) preserve the native stack-frame budget for complete causal snapshots
    const std::unique_ptr<runtime::StepState> incoming_step{new runtime::StepState(context.physical.state())};
    const auto drive=std::make_unique<drivetrain::DriveSnapshot>(stepper.drive().state());
    const auto tire=stepper.has_spatial_tire()?std::make_unique<pneumatic3d::TyreState3d>(stepper.spatial_tire_writer().state()):nullptr;
    std::unique_ptr<writers::RiderContactsState> rider;
    std::optional<writers::RiderContactsProbe> rider_probe;
    if(stepper.has_rider_contacts()){rider=std::make_unique<writers::RiderContactsState>(stepper.rider_contacts().state());rider_probe=stepper.rider_contacts().probe();}
    const auto hold=context.bootstrap_hold?std::optional{context.bootstrap_hold->state()}:std::nullopt;
    struct ModelRow{std::span<double> target;std::vector<double> bytes;};std::vector<ModelRow> model_rows;
    const auto save_row=[&](double* target,mjtSize width){const auto values=model_access::mutable_buffer(target,width);
        model_rows.push_back({values,std::vector<double>(values.begin(),values.end())});};
    save_row(m.dof_frictionloss,m.nv);save_row(m.site_pos,3*m.nsite);
    save_row(m.tendon_stiffness,m.ntendon);save_row(m.tendon_damping,m.ntendon);
    save_row(m.tendon_lengthspring,2*m.ntendon);
    save_row(m.tendon_range,2*m.ntendon);save_row(m.wrap_prm,m.nwrap);
    std::vector<int> dofs;
    const auto& root=context.topology->root;
    for(int i=0;i<m.nv;++i) {
        // W00 BootstrapConfig has no DOF mask: fix the world-x station and
        // yaw gauge, allowing height, lateral position, roll, pitch and all
        // passive scalar/ball DOFs to seek equilibrium on their manifold.
        if(root.kind==spatial::RootKind::free6 && (i==root.free_dof_adr || i==root.free_dof_adr+5))continue;
        if(root.kind!=spatial::RootKind::free6 && i==root.scalar[0].dof_adr)continue;
        dofs.push_back(i);
    }
    std::vector<double> best(dofs.size(),0.),dq(nv),difference_q(nv);
    double best_residual=std::numeric_limits<double>::infinity();int evaluations=0;
    const auto restore_material=[&] {
        engine::set_state(&m,&d,integration.data(),mjSTATE_INTEGRATION);
        context.physical.restore(*incoming_step);
        stepper.mutate([&]{stepper.drive().restore(*drive);});
        if(tire)stepper.restore_spatial_tire(*tire);
        if(rider){auto copy=*rider;auto probe=rider_probe;stepper.rider_contacts().assign_state(std::move(copy),std::move(probe));}
        if(hold && context.bootstrap_hold->restore(*hold)!=CoreStatus::ok)
            throw std::invalid_argument("spatial_refine: hold snapshot failed restore");
        for(const auto& row:model_rows)std::ranges::copy(row.bytes,row.target.begin());
    };
    const auto place=[&](std::span<const double> x) {
        restore_material();std::ranges::copy(q0,qpos.begin());std::ranges::fill(dq,0.);
        for(std::size_t i=0;i<x.size();++i) {
            const int dof=dofs[i];
            if(root.kind==spatial::RootKind::free6 && dof>=root.free_dof_adr+3 && dof<root.free_dof_adr+5) {
                // MuJoCo stores free angular tangent velocities in body
                // coordinates. Rotate the allowed WORLD roll/pitch basis so
                // fixing world yaw remains correct at a pitched/yawed start.
                spatial::Vec3 axis{};axis[static_cast<std::size_t>(dof-root.free_dof_adr-3)]=1.;
                const auto qa=static_cast<std::size_t>(root.free_qpos_adr+3);
                const auto local=spatial::rotate_world_to_body({q0[qa],q0[qa+1],q0[qa+2],q0[qa+3]},axis);
                for(std::size_t k=0;k<3;++k)dq[static_cast<std::size_t>(root.free_dof_adr+3)+k]+=x[i]*local[k];
            } else dq[static_cast<std::size_t>(dof)]=x[i];
        }
        stepper.mutate([&]{engine::integrate_pos(&m,d.qpos,dq.data(),1.);});
        struct Difference{const mjModel* model;double* dq;const double* a;const double* b;} call{&m,difference_q.data(),q0.data(),d.qpos};
        engine::ErrorBuffer error;
        if(!engine::invoke([](void* raw) noexcept {const auto& c=*static_cast<Difference*>(raw);mj_differentiatePos(c.model,c.dq,1.,c.a,c.b);},&call,error))engine::throw_failure(error);
        if(!std::ranges::all_of(difference_q,[](double x){return std::isfinite(x);}))
            throw std::invalid_argument("spatial_refine: nonfinite tangent displacement");
        std::ranges::fill(qvel,0.);stepper.note_posvel_stale();
    };
    const auto residual=[&](std::span<const double> x) {
        place(x);apply_initial_forces(context,false);stepper.forward();++evaluations;
        std::vector<double> r(stepper.qacc().begin(),stepper.qacc().end());
        const double max=acceleration_residual(context);
        if(max<best_residual){best_residual=max;best.assign(x.begin(),x.end());}
        return r;
    };
    numerics::LeastSquaresOptions options;options.max_evaluations=max_evaluations;
    options.difference=numerics::DifferenceScheme::three_point;options.relative_difference_step=1e-6;
    options.ftol=options.xtol=options.gtol=1e-10;
    options.stop=[&](std::span<const double>){return best_residual<=tolerance;};
    std::vector<double> lower(dofs.size()),upper(dofs.size());
    for(std::size_t i=0;i<dofs.size();++i){const double limit=translation(m,dofs[i])?.10:.35;lower[i]=-limit;upper[i]=limit;}
    std::exception_ptr failure;
    try {
        if(max_evaluations==1)(void)residual(std::vector<double>(dofs.size(),0.));
        else (void)numerics::bounded_least_squares(residual,std::vector<double>(dofs.size(),0.),lower,upper,options);
    }catch(const engine::EngineFailure&){throw;}catch(...){failure=std::current_exception();}
    place(best);apply_initial_forces(context,false);stepper.forward();
    context.spatial_report=std::make_shared<SpatialEquilibriumReport>(report(context,evaluations,tolerance));
    // A one-point report is observational. In particular a cached pose must
    // retain its warm start and complete integration bytes before the same
    // finalize sequence used by a fresh solve.
    if(max_evaluations==1) {
        restore_material();stepper.note_posvel_stale();stepper.forward();
        engine::set_state(&m,&d,integration.data(),mjSTATE_INTEGRATION);
        stepper.note_posvel_fresh();
    }
    if(failure)std::rethrow_exception(failure);
    return std::max(context.spatial_report->max_linear_accel_mps2,context.spatial_report->max_angular_accel_radps2);
}
} // namespace bike::initialization
