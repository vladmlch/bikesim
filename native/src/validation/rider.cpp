#include "mechanics.hpp"
#include "../diag.hpp"
#include "../rider/state_codec.hpp"
#include "../writers/rider_contacts.hpp"
#include <numbers>

namespace bike::validation {
using mechanics::metrics;
using mechanics::bounds;
namespace {
std::array<double,3> cross(const std::array<double,3>& a,const std::array<double,3>& b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
[[gnu::noinline]] std::unique_ptr<::rider::SpindleController> controller(mechanics::BikeStand& bike) {
    const auto pose=::rider::decode_spindle_pose(runtime::project_articulated_pose(model::geometry_pose(*bike.request.rider,bike.request.bike)),"validation.pose"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
    return std::make_unique<::rider::SpindleController>(bike.model(),pose,::rider::decode_spindle_config(runtime::project_rider_controller(*bike.request.physics,*bike.model())),bike.request.bike.crank_length/1000.); // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
}
// The frame limit keeps every multi-kilobyte fixture owner on the heap;
// construction order below is unchanged from the stack-resident original.
// Config, BuildRequest and BikeStand temporaries each get their own frame;
// noinline keeps the same split under optimization.
[[gnu::noinline]] configuration::SimulationPhysicsConfig momentum_config(double dt,std::string_view transmission){
    configuration::SimulationPhysicsConfig c;c.physics_mode="physical";c.drive_mode="articulated_effort";c.timestep_s=dt;c.closure_time_constant_s=.0025;
    c.drive.transmission_model=transmission;c.drive.human_torque_nm=0.;c.drive.gearing={.front_teeth=34,.rear_teeth=51};c.drive.assist.gain=2.;c.drive.assist.tau=.03;return c;
}
[[gnu::noinline]] model::BuildRequest momentum_model(double dt,std::string_view transmission){
    return mechanics::request(momentum_config(dt,transmission),true);
}
[[gnu::noinline]] std::unique_ptr<mechanics::BikeStand> momentum_bike(double dt,std::string_view transmission){
    return std::make_unique<mechanics::BikeStand>(momentum_model(dt,transmission));
}
[[gnu::noinline]] configuration::SimulationPhysicsConfig airborne_config(double dt){
    configuration::SimulationPhysicsConfig c;c.physics_mode="physical";c.drive_mode="articulated_effort";c.timestep_s=dt;return c;
}
[[gnu::noinline]] model::BuildRequest airborne_model(double dt){
    return mechanics::request(airborne_config(dt),true);
}
[[gnu::noinline]] std::unique_ptr<mechanics::BikeStand> airborne_bike(double dt){
    return std::make_unique<mechanics::BikeStand>(airborne_model(dt));
}
}

Momentum system_momentum(const mjModel* m,const mjData* d) {
    if(!m||!d)throw std::invalid_argument("momentum requires model and data");mechanics::finite_state(m,d);
    const std::span<const mjtNum> body_mass=model_access::readonly_buffer(m->body_mass,m->nbody,"body_mass"),body_inertia=model_access::readonly_buffer(m->body_inertia,3*m->nbody,"body_inertia"),xipos=model_access::readonly_buffer(d->xipos,3*m->nbody,"xipos"),ximat=model_access::readonly_buffer(d->ximat,9*m->nbody,"ximat"),qvel=model_access::readonly_buffer(d->qvel,m->nv,"qvel");
    std::vector<int> bodies;std::vector<double> masses;
    for(std::size_t b=0;b<body_mass.size();++b)if(body_mass[b]>0.){bodies.push_back(static_cast<int>(b));masses.push_back(body_mass[b]);}
    const double total=numeric::numpy_sum(masses);if(!std::isfinite(total)||total<=0.)throw std::invalid_argument("system must contain positive mass");Momentum out;
    std::array<double,3> origin{};std::ranges::copy(xipos.subspan(3*static_cast<std::size_t>(bodies.front()),3),origin.begin());
    // NumPy's strided axis=0 reduction accumulates each XYZ lane in body
    // order. A contiguous flatten-and-sum would change its numerical tree.
    for(std::size_t i=0;i<bodies.size();++i)for(std::size_t axis=0;axis<3;++axis)out.com[axis]+=masses[i]*(xipos[3*static_cast<std::size_t>(bodies[i])+axis]-origin[axis]);
    for(std::size_t axis=0;axis<3;++axis)out.com[axis]=origin[axis]+out.com[axis]/total;
    const auto nv=static_cast<std::size_t>(m->nv);std::vector<double> jp(3*nv),jr(3*nv);
    for(int const body:bodies){mj_jacBodyCom(m,d,jp.data(),jr.data(),body);const auto velocity=mechanics::matvec(jp,qvel),omega=mechanics::matvec(jr,qvel);std::array<double,3> p{},r{};
        const auto b=static_cast<std::size_t>(body);
        for(std::size_t a=0;a<3;++a){p[a]=body_mass[b]*velocity[a];r[a]=xipos[3*b+a]-out.com[a];}
        std::array<double,9> diagonal{},product{},inertia{};for(std::size_t a=0;a<3;++a)diagonal[4*a]=body_inertia[3*b+a];const auto rotation=ximat.subspan(9*b,9);
        blas::dgemm(blas::Order::row_major,blas::Transpose::no,blas::Transpose::no,3,3,3,1.,rotation.data(),3,diagonal.data(),3,0.,product.data(),3);
        blas::dgemm(blas::Order::row_major,blas::Transpose::no,blas::Transpose::yes,3,3,3,1.,product.data(),3,rotation.data(),3,0.,inertia.data(),3);
        const auto spin=mechanics::matvec(inertia,omega),orbital=cross(r,p);for(std::size_t a=0;a<3;++a){out.linear[a]+=p[a];out.angular[a]+=spin[a]+orbital[a];}}
    return out;
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
RigResult momentum_rig(double dt,bool active,std::string_view transmission,std::optional<bool> rider_active) {
    const bool enabled=rider_active.value_or(active);const auto bike=momentum_bike(dt,transmission);
    auto* m=bike->model();auto* d=bike->data();auto rider=controller(*bike);rider->initialize(d);
    const std::span<mjtNum> qpos=model_access::mutable_buffer(d->qpos,m->nq,"qpos"),qvel=model_access::mutable_buffer(d->qvel,m->nv,"qvel"),qfrc_applied=model_access::mutable_buffer(d->qfrc_applied,m->nv,"qfrc_applied");
    const std::span<const mjtNum> qfrc_actuator=model_access::readonly_buffer(d->qfrc_actuator,m->nv,"qfrc_actuator"),actuator_force=model_access::readonly_buffer(d->actuator_force,m->nu,"actuator_force"),body_mass=model_access::readonly_buffer(m->body_mass,m->nbody,"body_mass");const auto gravity=std::span{m->opt.gravity};
    for(const char* name:{"root_z","rider_root_z"})qpos[static_cast<std::size_t>(mechanics::qpos(m,name))]+=30.;qpos[static_cast<std::size_t>(mechanics::qpos(m,"rider_torso_hinge"))]+=.01;
    for(const char* name:{"front_wheel_spin","rear_wheel_spin"})qvel[static_cast<std::size_t>(mechanics::dof(m,name))]=10.;qvel[static_cast<std::size_t>(mechanics::dof(m,"crank_spin"))]=4.;engine::forward(m,d);
    if(!bike->writers.rider_contacts||!bike->writers.drive||!bike->writers.suspension)throw std::logic_error("articulated fixture writers were not projected");
    const auto contacts=std::make_unique<writers::RiderContactWriter>(m,*bike->writers.rider_contacts);contacts->reset(d);const auto drive=std::make_unique<drivetrain::DrivetrainWriter>(m,d,*bike->writers.drive);drive->reset();SuspensionWriter const suspension(m,*bike->writers.suspension);
    const auto initial=system_momentum(m,d);constexpr double duration=.1;const auto count=steps(duration,dt),nv=static_cast<std::size_t>(m->nv);std::vector<double> contact_force(nv),velocity(nv);std::array<double,3> external{};
    double max_p=0.,max_l=0.,max_ly=0.,max_plane=0.,motor_work=0.,joint_work=0.,power_error=0.,peak=0.,root_actuation=0.,defect=0.;std::int64_t ground_contacts=0;
    const double mass=numeric::numpy_sum(body_mass);
    for(std::size_t i=0;i<count;++i){const double torque=active&&i<count/2?20.:0.;engine::forward(m,d);contacts->compute_qfrc_into(d,dt,true,false,contact_force);
        const auto& state=contacts->state();const auto& support=state.diagnostics.supports;
        const std::array available{state.enabled[1]&&support[1].in_platform,state.enabled[2]&&support[2].in_platform,state.enabled[3],support[0].in_platform};
        const auto commands=rider->compute(d,::rider::RiderCommand{.mean_crank_torque_nm=0.,.enabled=enabled},true,dt,false,available);rider->write(d,commands); // NOLINT(clang-diagnostic-missing-designated-field-initializers) unlisted fields keep their declared defaults.
        const double sensor=torque>0.?10.:0.;const drivetrain::RideControl control{.motor_torque_nm=torque,.human_torque_nm=sensor}; // NOLINT(clang-diagnostic-missing-designated-field-initializers) unlisted fields keep their declared defaults.
        const auto components=drive->components(control,dt,0.,false,true,true,sensor,{},true,{});auto force=mechanics::sum_components(components,nv);const auto spring=mechanics::sum_components(suspension.components(d),nv);
        for(std::size_t q=0;q<nv;++q)qfrc_applied[q]=force[q]+contact_force[q]+spring[q];std::ranges::copy(qvel,velocity.begin());engine::step(m,d);(void)drive->settle();
        for(const char* name:{"root_x","root_z","root_pitch","rider_root_x","rider_root_z","rider_root_pitch"})root_actuation=std::max(root_actuation,std::abs(qfrc_actuator[static_cast<std::size_t>(mechanics::dof(m,name))]));
        const double actual=mechanics::telemetry(*drive,"motor_torque_nm"),shaft=velocity[static_cast<std::size_t>(mechanics::dof(m,"crank_spin"))];power_error=std::max(power_error,std::abs(mechanics::telemetry(*drive,"motor_shaft_power_w")-actual*shaft));motor_work+=actual*shaft*dt;peak=std::max(peak,actual);
        numeric::PythonSum joint_power;for(const auto& j:rider->joints())joint_power.add(std::max(0.,actuator_force[static_cast<std::size_t>(j.actuator_id)]*velocity[static_cast<std::size_t>(j.dof_adr)]));joint_work+=joint_power.total()*dt;defect=std::max(defect,std::abs(mechanics::telemetry(*drive,"transmission_constraint_defect_m")));
        engine::forward(m,d);for(std::size_t a=0;a<3;++a)external[a]+=mass*gravity[a]*dt;const auto current=system_momentum(m,d);std::array<double,3> residual{},angular{};for(std::size_t a=0;a<3;++a){residual[a]=current.linear[a]-initial.linear[a]-external[a];angular[a]=current.angular[a]-initial.angular[a];}
        max_p=std::max(max_p,vector_norm(residual));max_l=std::max(max_l,vector_norm(angular));max_ly=std::max(max_ly,std::abs(angular[1]));const std::array plane{angular[0],angular[2]};max_plane=std::max(max_plane,vector_norm(plane));
        const std::span<const mjContact> contact_list=model_access::readonly_buffer(d->contact,d->ncon,"contact");
        for(const auto & contact : contact_list){bool ground=false;for(int const g:contact.geom){const char* name=mj_id2name(m,mjOBJ_GEOM,g);if(name&&(std::string_view(name)=="terrain"||std::string_view(name)=="catch_plane"))ground=true;}ground_contacts+=ground?1:0;}}
    const double impulse=std::max(1.,mass*vector_norm(gravity)*duration),angular_scale=std::max({1.,vector_norm(initial.angular),impulse*1.}),inf=std::numeric_limits<double>::infinity();
    return {.metrics=metrics({{"impulse_residual_ns",max_p},{"impulse_residual_ratio",max_p/impulse},{"angular_momentum_residual_kg_m2_s",max_l},{"angular_residual_ratio",max_ly/angular_scale},
        {"pitch_angular_momentum_residual_kg_m2_s",max_ly},{"planar_roll_yaw_constraint_impulse_required_kg_m2_s",max_plane},{"impulse_floor_ns",1.},{"angular_scale_length_m",1.},{"impulse_scale_ns",impulse},
        {"motor_shaft_work_j",motor_work},{"joint_positive_work_j",joint_work},{"peak_motor_torque_nm",peak},{"shaft_power_identity_error_w",power_error},{"root_actuator_force_n",root_actuation},
        {"ground_contact_count",static_cast<double>(ground_contacts)},{"constraint_defect_m",defect},{"duration_s",duration},{"mass_kg",mass},{"synthetic_sensor_torque_nm",active?10.:0.},{"synthetic_rider_intent_nm",active?10.:0.}}),
        .bounds=bounds({{"impulse_residual_ratio",0.,.001},{"angular_residual_ratio",0.,.001},{"shaft_power_identity_error_w",0.,1e-10},{"root_actuator_force_n",0.,0.},{"ground_contact_count",0.,0.},
            {"peak_motor_torque_nm",active?1e-8:0.,active?20.:0.},{"motor_shaft_work_j",active?1e-8:0.,active?inf:0.}})};
}
NATIVE_DIAG_POP

RigResult airborne_internal_actuation(double dt) {
    const auto bike=airborne_bike(dt);auto const* m=bike->model();auto* d=bike->data();auto rider=controller(*bike);rider->initialize(d);
    const std::span<mjtNum> qpos=model_access::mutable_buffer(d->qpos,m->nq,"qpos"),qvel=model_access::mutable_buffer(d->qvel,m->nv,"qvel");
    const std::span<const mjtNum> actuator_force=model_access::readonly_buffer(d->actuator_force,m->nu,"actuator_force"),subtree_angmom=model_access::readonly_buffer(d->subtree_angmom,3*m->nbody,"subtree_angmom");
    for(const char* name:{"root_z","rider_root_z"})qpos[static_cast<std::size_t>(mechanics::qpos(m,name))]+=30.;qpos[static_cast<std::size_t>(mechanics::qpos(m,"rider_torso_hinge"))]+=.03;for(const char* name:{"front_wheel_spin","rear_wheel_spin"})qvel[static_cast<std::size_t>(mechanics::dof(m,name))]=10.;engine::forward(m,d);engine::subtree_vel(m,d);
    const double initial=subtree_angmom[1],scale=std::max(1.,std::abs(initial));const std::vector<double> q0(qpos.begin(),qpos.end());std::vector<double> velocity(static_cast<std::size_t>(m->nv));double maximum=0.,work=0.,absolute_work=0.,peak=0.;
    for(std::size_t i=0,n=steps(2.,dt);i<n;++i){engine::forward(m,d);const auto commands=rider->compute(d,::rider::RiderCommand{.mean_crank_torque_nm=20.},true,{},false,std::array<bool,4>{});rider->write(d,commands);std::ranges::copy(qvel,velocity.begin());engine::step(m,d);mechanics::finite_state(m,d); // NOLINT(clang-diagnostic-missing-designated-field-initializers) unlisted fields keep their declared defaults.
        numeric::PythonSum power;for(const auto& j:rider->joints()){power.add(actuator_force[static_cast<std::size_t>(j.actuator_id)]*velocity[static_cast<std::size_t>(j.dof_adr)]);peak=std::max(peak,std::abs(actuator_force[static_cast<std::size_t>(j.actuator_id)]));}work+=power.total()*dt;absolute_work+=std::abs(power.total())*dt;engine::forward(m,d);engine::subtree_vel(m,d);maximum=std::max(maximum,std::abs(subtree_angmom[1]-initial));}
    double motion=0.;for(const auto& j:rider->joints())motion=std::max(motion,std::abs(qpos[static_cast<std::size_t>(j.qpos_adr)]-q0[static_cast<std::size_t>(j.qpos_adr)]));const double inf=std::numeric_limits<double>::infinity();
    return {.metrics=metrics({{"relative_Ly_drift",maximum/scale},{"joint_motion_rad",motion},{"actual_joint_work_j",work},{"absolute_joint_work_j",absolute_work},{"peak_joint_torque_nm",peak}}),
        .bounds=bounds({{"relative_Ly_drift",0.,.001},{"joint_motion_rad",.01,inf},{"absolute_joint_work_j",.01,inf},{"peak_joint_torque_nm",0.,bike->request.physics->articulated.joint_limit_nm+1e-9}})}; // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
}
} // namespace bike::validation
