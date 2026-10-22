#include "mechanics.hpp"
#include "acceptance.hpp"
#include "evidence.hpp"
#include "../drivetrain/chain.hpp"
#include "../geometry/specs.hpp"
#include "../numerics/least_squares.hpp"
#include "../model/suspension_setup.hpp"
#include "../model_access.hpp"
#include "../numeric_sincos.hpp"
#include <numbers>

namespace bike::validation {
using mechanics::metrics;
using mechanics::bounds;
namespace {
constexpr std::array<std::string_view,3> modes{"ideal_mid_drive","geometric_ideal_mid_drive","elastic_chain"};
constexpr std::array strokes{.005,.015,.030};
// The 8 KiB frame budget keeps the SimulationPhysicsConfig, BuildRequest
// and BikeStand temporaries each in their own noinline frame; the inlined
// member-init lambda folded all three into the fixture owner's caller.
// Validation order and config values are unchanged.
[[gnu::noinline]] configuration::SimulationPhysicsConfig fixture_config(double dt,std::string_view mode,double stroke){
    positive(dt,"fixture step");if(std::ranges::find(modes,mode)==modes.end()||!std::isfinite(stroke)||stroke<0.||stroke>.04)throw std::invalid_argument("unknown transmission or suspension fixture position");
    configuration::SimulationPhysicsConfig c;c.physics_mode="physical";c.drive_mode="crank_effort";c.timestep_s=dt;c.closure_time_constant_s=.0025;
    c.drive.transmission_model=mode;c.drive.human_torque_nm=0.;c.drive.gearing={.front_teeth=34,.rear_teeth=51};return c;
}
[[gnu::noinline]] model::BuildRequest fixture_model(double dt,std::string_view mode,double stroke){
    return mechanics::request(fixture_config(dt,mode,stroke));
}
[[gnu::noinline]] mechanics::BikeStand fixture_bike(double dt,std::string_view mode,double stroke){
    return mechanics::BikeStand(fixture_model(dt,mode,stroke));
}
struct Fixture {
    mechanics::BikeStand bike;
    std::unique_ptr<drivetrain::DrivetrainWriter> drive;
    std::unique_ptr<SuspensionWriter> suspension;
    double closure=0.;
    Fixture(double dt,std::string_view mode,double stroke):bike(fixture_bike(dt,mode,stroke)) {
        auto* m=bike.model();auto* d=bike.data();std::ranges::fill(model_access::mutable_buffer(m->geom_contype,m->ngeom,"geom_contype"),0);std::ranges::fill(model_access::mutable_buffer(m->geom_conaffinity,m->ngeom,"geom_conaffinity"),0);
        const std::span<mjtNum> qpos=model_access::mutable_buffer(d->qpos,m->nq,"qpos");const std::span<const mjtNum> site_xpos=model_access::readonly_buffer(d->site_xpos,3*m->nsite,"site_xpos");
        const std::array indices{mechanics::qpos(m,"main_pivot"),mechanics::qpos(m,"horst_pivot"),mechanics::qpos(m,"rocker_frame_pivot"),mechanics::qpos(m,"yoke_pivot")};
        const std::array<std::array<int,2>,2> pairs{{{object_id(m,mjOBJ_SITE,"site_P3_ss"),object_id(m,mjOBJ_SITE,"site_P3_rocker")},{object_id(m,mjOBJ_SITE,"site_P7_shock"),object_id(m,mjOBJ_SITE,"site_P7")}}};
        qpos[static_cast<std::size_t>(mechanics::qpos(m,"fork_travel"))]=.03;
        auto const residual=[&](std::span<const double> q){for(std::size_t i=0;i<indices.size();++i)qpos[static_cast<std::size_t>(indices[i])]=q[i];engine::kinematics(m,d);std::vector<double> out;for(auto [a,b]:pairs){const auto ea=static_cast<std::size_t>(a),eb=static_cast<std::size_t>(b);out.push_back(site_xpos[3*ea]-site_xpos[3*eb]);out.push_back(site_xpos[3*ea+2]-site_xpos[3*eb+2]);}return out;};
        numerics::LeastSquaresOptions options;options.jacobian_scaling=false;options.ftol=options.xtol=options.gtol=1e-13;options.max_evaluations=120;
        const std::array lower{-1.5,-1.5,-1.5,-1.5},upper{1.5,1.5,1.5,1.5};
        for(double const position:geometry::detail::np_linspace(0.,stroke,7)) {
            qpos[static_cast<std::size_t>(mechanics::qpos(m,"shock_stroke"))]=position;std::array<double,4> q{};for(std::size_t i=0;i<q.size();++i)q[i]=qpos[static_cast<std::size_t>(indices[i])];
            const auto solution=numerics::bounded_least_squares(residual,q,lower,upper,options);const auto final=residual(solution.x);for(double const x:final)closure=std::max(closure,std::abs(x));
        }
        // Only the final continuation position is the source closure gate.
        std::array<double,4> q{};for(std::size_t i=0;i<q.size();++i)q[i]=qpos[static_cast<std::size_t>(indices[i])];closure=0.;for(double const x:residual(q))closure=std::max(closure,std::abs(x));
        if(closure>1e-7)throw std::runtime_error("initial closed linkage did not converge");engine::forward(m,d);
        if(!bike.writers.drive||!bike.writers.suspension)throw std::logic_error("physical fixture writers were not projected");
        suspension=std::make_unique<SuspensionWriter>(m,*bike.writers.suspension);drive=std::make_unique<drivetrain::DrivetrainWriter>(m,d,*bike.writers.drive);drive->reset();
    }
};
// The frame limit keeps the BikeStand out of caller frames; noinline
// helpers hold the same split under optimization. Gearing values reach
// the caller through request.physics, identical to the pre-refactor
// stack-resident default configuration.
[[gnu::noinline]] configuration::SimulationPhysicsConfig chain_config(){
    configuration::SimulationPhysicsConfig c;c.physics_mode="physical";return c;
}
[[gnu::noinline]] model::BuildRequest chain_model(){
    return mechanics::request(chain_config());
}
[[gnu::noinline]] std::unique_ptr<mechanics::BikeStand> chain_bike(){
    return std::make_unique<mechanics::BikeStand>(chain_model());
}
}

RigResult drive_suspension_rig(double dt,std::string_view mode,double torque,double stroke,double duration) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
    if(!std::isfinite(torque)||torque<0.||torque>40.)throw std::invalid_argument("fixture torque range is 0..40 Nm");
    const auto fixture=std::make_unique<Fixture>(dt,mode,stroke);auto const* m=fixture->bike.model();auto* d=fixture->bike.data();auto& drive=*fixture->drive;auto const& suspension=*fixture->suspension;
    const auto count=steps(duration,dt),nv=static_cast<std::size_t>(m->nv);const int frame=object_id(m,mjOBJ_BODY,"frame"),rear=object_id(m,mjOBJ_BODY,"rear_wheel");
    const std::array bodies{object_id(m,mjOBJ_BODY,"front_wheel"),rear};std::array<double,2> radii{},initial_bottoms{};
    const std::span<mjtNum> ctrl=model_access::mutable_buffer(d->ctrl,m->nu,"ctrl"),qfrc_applied=model_access::mutable_buffer(d->qfrc_applied,m->nv,"qfrc_applied");
    const std::span<const mjtNum> qpos=model_access::readonly_buffer(d->qpos,m->nq,"qpos"),qvel=model_access::readonly_buffer(d->qvel,m->nv,"qvel"),xpos=model_access::readonly_buffer(d->xpos,3*m->nbody,"xpos"),xmat=model_access::readonly_buffer(d->xmat,9*m->nbody,"xmat"),xipos=model_access::readonly_buffer(d->xipos,3*m->nbody,"xipos");
    const std::span<const mjtNum> geom_size=model_access::readonly_buffer(m->geom_size,3*m->ngeom,"geom_size"),body_mass=model_access::readonly_buffer(m->body_mass,m->nbody,"body_mass"),actuator_force=model_access::readonly_buffer(d->actuator_force,m->nu,"actuator_force"),qfrc_actuator=model_access::readonly_buffer(d->qfrc_actuator,m->nv,"qfrc_actuator");
    for(std::size_t s=0;s<2;++s){const int geom=object_id(m,mjOBJ_GEOM,s==0?"geom_front_contact":"geom_rear_contact");radii[s]=geom_size[3*static_cast<std::size_t>(geom)];initial_bottoms[s]=xpos[3*static_cast<std::size_t>(bodies[s])+2]-radii[s];}
    constexpr double frame_load=80.*9.81;const double mass=numeric::numpy_sum(body_mass),total_weight=mass*9.81+frame_load;
    const std::array preload{total_weight*.41,total_weight*.59};
    auto const rear_z=[&]{std::array<double,3> p{},axis{};const auto eframe=static_cast<std::size_t>(frame),erear=static_cast<std::size_t>(rear);for(std::size_t k=0;k<3;++k){p[k]=xpos[3*erear+k]-xpos[3*eframe+k];axis[k]=xmat[9*eframe+3*k+2];}return vector_dot(p,axis);};
    const double initial_rear=rear_z(),initial_pitch=qpos[static_cast<std::size_t>(mechanics::qpos(m,"root_pitch"))];std::vector<double> jac(3*nv),rot(3*nv);
    std::array<std::vector<double>,5> rows;double defect=0.,power_error=0.,root_actuation=0.;mjtSize contacts=0;numeric::PythonSum work;
    for(std::size_t i=0;i<count;++i){engine::forward(m,d);
        auto force=mechanics::sum_components(drive.components({},dt,0.,false,false,true,0.,{},true,{}),nv);
        const auto suspension_components=suspension.components(d);const auto spring=mechanics::sum_components(suspension_components,nv);
        for(std::size_t q=0;q<nv;++q)force[q]+=spring[q];
        const int aid=object_id(m,mjOBJ_ACTUATOR,"mid_drive");ctrl[static_cast<std::size_t>(aid)]=torque;std::array<double,2> loads{};
        for(std::size_t s=0;s<2;++s){std::array<double,3> p{};std::ranges::copy(xpos.subspan(3*static_cast<std::size_t>(bodies[s]),3),p.begin());p[2]-=radii[s];mj_jac(m,d,jac.data(),rot.data(),p.data(),bodies[s]);const auto velocity=mechanics::matvec(jac,qvel);
            const double normal=std::max(0.,preload[s]-130000.*(p[2]-initial_bottoms[s])-800.*velocity[2]),tangent=std::clamp(-2000.*velocity[0],-.8*normal,.8*normal);
            mechanics::add_transpose(force,jac,{tangent,0.,normal});loads[s]=normal;}
        mj_jac(m,d,jac.data(),rot.data(),xipos.subspan(3*static_cast<std::size_t>(frame),3).data(),frame);mechanics::add_transpose(force,jac,{0.,0.,-frame_load});std::ranges::copy(force,qfrc_applied.begin());
        const double omega=qvel[static_cast<std::size_t>(mechanics::dof(m,"crank_spin"))],travel=rear_z()-initial_rear;
        // This report currently reconstructs the writer's discarded scalar
        // from components. Subtracting/readding HBO can differ in the last
        // bits from Python shock_total; qualification must compare the
        // original scalar before calling this source port equivalent.
        const int sd=suspension.shock_dofadr();double shock=0.;
        const auto component=[&](std::string_view key){for(const auto& [name,v]:suspension_components)if(name==key)return -v[static_cast<std::size_t>(sd)];return 0.;};
        shock=component("shock_coil")+component("shock_bumper")+(component("shock_damper")+component("shock_hbo"))+component("shock_top_out")+component("shock_upper_stop");
        engine::step(m,d);(void)drive.settle();const double actual=actuator_force[static_cast<std::size_t>(aid)],power=actual*omega;power_error=std::max(power_error,std::abs(actual-torque));
        if(drive.transmission_storage())defect=std::max(defect,std::abs(mechanics::telemetry(drive,"transmission_constraint_defect_m")));
        for(const char* name:{"root_x","root_z","root_pitch"})root_actuation=std::max(root_actuation,std::abs(qfrc_actuator[static_cast<std::size_t>(mechanics::dof(m,name))]));contacts=std::max(contacts,static_cast<mjtSize>(d->ncon));
        rows[0].push_back(shock);rows[1].push_back(loads[0]);rows[2].push_back(travel);rows[3].push_back(power);rows[4].push_back(actual);work.add(power*dt);mechanics::finite_state(m,d);
    }
    return {.metrics=metrics({{"mean_shock_force_n",mean(rows[0])},{"mean_front_normal_load_n",mean(rows[1])},{"mean_rear_travel_delta_m",mean(rows[2])},{"final_rear_travel_delta_m",rows[2].back()},
        {"mean_shaft_power_w",mean(rows[3])},{"shaft_work_j",work.total()},{"mean_delivered_torque_nm",mean(rows[4])},{"delivered_torque_error_nm",power_error},{"constraint_defect_m",defect},
        {"initial_closure_error_m",fixture->closure},{"initial_shock_stroke_m",stroke},{"duration_s",duration},{"external_frame_load_n",frame_load},{"compiled_mass_kg",mass},
        {"root_actuator_force",root_actuation},{"native_contact_count",static_cast<double>(contacts)},{"frame_pitch_initial_rad",initial_pitch}}),
        .bounds=bounds({{"delivered_torque_error_nm",0.,1e-10},{"initial_closure_error_m",0.,1e-7},{"root_actuator_force",0.,0.},{"native_contact_count",0.,0.}})};
}

Value drive_suspension_comparison(const Value& config,const std::filesystem::path& root) {
    const auto dt= numbers(optional(config,"dt_values",Value{Array{value::real(.0003125),value::real(.00015625)}}),"dt_values");if(dt.empty())throw std::invalid_argument("positive steps required");for(double const x:dt)positive(x,"dt");
    Value report{Object{{"schema_version",value::integer(1)},{"environment",environment_contract(root)},{"source_sha256",value::text(source_fingerprint(root/"native/src"))},
        {"fixture",value::text("free fork/linkage, vertical 784.8 N frame dead-load, compliant viscous rollers; initial rest, zero stored chain/freehub energy")},
        {"force_comparison_gate",value::real(.05)},{"force_floor_n",value::real(10.)},{"series",Value{Array{}}},{"comparisons",Value{Array{}}},{"passed",Value{false}}}};
    Array series,comparisons;std::map<std::tuple<double,double,double,std::string_view>,Value> lookup;
    const auto checkpoint=[&]{if(const auto* path=find(config,"output"))write_report(root/io::as_string(*path,"output"),report);};
    for(double const step:dt)for(double const stroke:strokes)for(double const torque:{0.,20.,40.})for(auto const mode:modes){Value item{Object{{"dt_s",value::real(step)},{"mode",value::text(mode)},{"imposed_torque_nm",value::real(torque)},{"initial_shock_stroke_m",value::real(stroke)}}};auto& fields=std::get<Object>(item.data);
        try{auto result=drive_suspension_rig(step,mode,torque,stroke);value::set(fields,"passed",Value{checks_passed(check_bounds(result.metrics,result.bounds))});value::set(fields,"metrics",std::move(result.metrics));}
        catch(const std::exception& e){value::set(fields,"passed",Value{false});value::set(fields,"error",value::text(e.what()));value::set(fields,"metrics",Value{});}
        lookup.emplace(std::tuple{step,stroke,torque,mode},item);series.push_back(std::move(item));value::set(std::get<Object>(report.data),"series",Value{series});checkpoint();}
    const double finest=*std::ranges::min_element(dt);bool proposed=false,passed=true;
    for(double const stroke:strokes)for(double const torque:{0.,20.,40.})for(auto const mode:{modes[0],modes[1]}){const auto& a=lookup.at({finest,stroke,torque,mode});const auto& b=lookup.at({finest,stroke,torque,modes[2]});Value result{Object{{"dt_s",value::real(finest)},{"mode",value::text(mode)},{"initial_shock_stroke_m",value::real(stroke)},{"torque_nm",value::real(torque)}}};auto& fields=std::get<Object>(result.data);bool ok=false;
        const auto& ma=get(a,"metrics");const auto& mb=get(b,"metrics");if(!io::is_null(ma)&&!io::is_null(mb)){Object errors;double maximum=0.;for(const char* name:{"mean_shock_force_n","mean_front_normal_load_n"}){const double error=relative_change(real(ma,name),real(mb,name),10.);maximum=std::max(maximum,error);errors.emplace_back(name,value::real(error));}ok=flag(a,"passed")&&flag(b,"passed")&&maximum<=.05;
            value::set(fields,"relative_force_errors",Value{std::move(errors)});value::set(fields,"rear_travel_difference_m",value::real(real(ma,"mean_rear_travel_delta_m")-real(mb,"mean_rear_travel_delta_m")));value::set(fields,"shaft_power_difference_w",value::real(real(ma,"mean_shaft_power_w")-real(mb,"mean_shaft_power_w")));}
        else value::set(fields,"reason",value::text("missing fixture result"));value::set(fields,"passed",Value{ok});if(mode=="geometric_ideal_mid_drive"&&torque>0.){proposed=true;passed=passed&&ok;}comparisons.push_back(std::move(result));}
    for(const auto& row:series)passed=passed&&flag(row,"passed");passed=passed&&proposed;auto& out=std::get<Object>(report.data);value::set(out,"comparisons",Value{std::move(comparisons)});value::set(out,"passed",Value{passed});value::set(out,"status",value::text(passed?"synthetic_local_fixture_pass":"experimental_gate_failed"));checkpoint();return report;
}

Value compiled_chain_motion_rig() {
    const auto bike=chain_bike();auto* m=bike->model();auto* d=bike->data();engine::forward(m,d);
    const int front=object_id(m,mjOBJ_BODY,"crank"),rear=object_id(m,mjOBJ_BODY,"cassette"),frame=object_id(m,mjOBJ_BODY,"frame"),pivot=mechanics::qpos(m,"main_pivot"),cassette=mechanics::qpos(m,"cassette_spin");
    const auto& gearing=bike->request.physics->drive.gearing;const drivetrain::GearingConfig gear{.front_teeth=gearing.front_teeth,.rear_teeth=gearing.rear_teeth,.chain_pitch_m=gearing.chain_pitch_m};drivetrain::GeometryWorkspace analytic(m->nv),numeric(m->nv);const double reference=analytic.evaluate(m,d,gear,front,rear,frame,{}, {},false);const drivetrain::Vec2 original_angles{analytic.theta_f,analytic.theta_r};const double psi=analytic.psi,phase=analytic.theta_r; // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
    const std::span<mjtNum> qpos=model_access::mutable_buffer(d->qpos,m->nq,"qpos");
    const std::vector<double> base(qpos.begin(),qpos.end());std::unique_ptr<mjData,DataDeleter> scratch(engine::make_data(m));double error=0.,coupling=0.;
    for(double const displacement:geometry::detail::np_linspace(-.05,.05,9)){std::ranges::copy(base,qpos.begin());qpos[static_cast<std::size_t>(pivot)]+=displacement;engine::forward(m,d);qpos[static_cast<std::size_t>(cassette)]+=phase-drivetrain::unwrap(analytic.angle(m,d,rear,false),phase);engine::forward(m,d);
        const std::vector<double> saved(qpos.begin(),qpos.end());(void)analytic.evaluate(m,d,gear,front,rear,frame,original_angles,psi,false);const auto jac=analytic.jacobian;const drivetrain::Vec2 angles{analytic.theta_f,analytic.theta_r};
        const auto numerical=drivetrain::chain_jacobian(saved,[&](std::span<const double> q){std::ranges::copy(q,model_access::mutable_buffer(scratch->qpos,m->nq,"qpos").begin());engine::kinematics(m,scratch.get());return numeric.evaluate(m,scratch.get(),gear,front,rear,frame,angles,psi,false)-reference;});
        std::vector<double> difference(jac.size());for(std::size_t i=0;i<jac.size();++i)difference[i]=jac[i]-numerical[i];error=std::max(error,vector_norm(difference)/std::max(vector_norm(numerical),1e-12));coupling=std::max(coupling,std::abs(jac[static_cast<std::size_t>(mechanics::dof(m,"main_pivot"))]));
        if(!std::ranges::equal(saved,qpos))throw std::logic_error("chain oracle changed live generalized coordinates");}
    return metrics({{"compiled_jacobian_relative_error",error},{"linkage_force_per_tension_m",coupling}});
}

RigResult suspension_cycle(double dt) {
    positive(dt,"damper cycle step");model::SuperDeluxeDamper damper;double minimum=0.,work=0.;const auto count=static_cast<std::size_t>(std::ceil(1./dt));
    for(std::size_t i=0;i<count;++i){const double t=static_cast<double>(i)*dt,x=.0325+.03*numeric::np_sin(2.*std::numbers::pi*t),v=.03*2.*std::numbers::pi*numeric::np_cos(2.*std::numbers::pi*t);const double power=damper.compute_damping_force(v,x*1000.)*v;minimum=std::min(minimum,power);work+=power*dt;}
    damper.lockout_firm=true;const double jump=std::abs(damper.compute_damping_force(.03+1e-8,20.)-damper.compute_damping_force(.03-1e-8,20.)),hbo=damper.compute_damping_components(.5,60.).hbo_n;const double inf=std::numeric_limits<double>::infinity();
    return {.metrics=metrics({{"minimum_dissipative_power_w",minimum},{"cycle_loss_j",work},{"firm_knee_jump_n",jump},{"firm_hbo_n",hbo}}),.bounds=bounds({{"minimum_dissipative_power_w",-1e-9,inf},{"cycle_loss_j",0.,inf},{"firm_knee_jump_n",0.,.001},{"firm_hbo_n",1.,inf}})};
}
} // namespace bike::validation
