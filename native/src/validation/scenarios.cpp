#include "scenarios.hpp"
#include "../diag.hpp"
#include "mechanics.hpp"
#include "acceptance.hpp"
#include "evidence.hpp"
#include "../configuration/load.hpp"
#include "../research/evidence.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/value_readers.hpp"
#include "../terrain/trackfile.hpp"
#include "../initialization/sag_fit.hpp"
#include "../model_access.hpp"
#include <chrono>
#include <functional>
#include <numbers>

namespace bike::validation {
using mechanics::metrics;
using mechanics::bounds;
namespace {
// The -O0 frame budget keeps the equilibrium result, metadata and sealed
// bootstrap temporaries in these helpers instead of the constructor frame.
void settle_into(initialization::SetupOwner& owner,const initialization::SetupRequest& request,initialization::EquilibriumResult& equilibrium){
    equilibrium=initialization::settle(owner,request.equilibrium);
}
void seal_runtime_into(initialization::SetupOwner& owner,Value& metadata,std::unique_ptr<runtime::NativeRideRuntime>& runtime){
    auto bootstrap=owner.seal_bootstrap();metadata=owner.metadata();runtime=std::make_unique<runtime::NativeRideRuntime>(std::move(bootstrap));
}
void seal_into(initialization::SetupRequest& request,initialization::EquilibriumResult& equilibrium,Value& metadata,std::unique_ptr<runtime::NativeRideRuntime>& runtime){
    auto owner=initialization::make_setup(request);settle_into(*owner,request,equilibrium);seal_runtime_into(*owner,metadata,runtime);
}
configuration::SimulationPhysicsConfig profile_config(const std::filesystem::path& physics,std::optional<double> dt,const Value& overrides){
    auto config=configuration::load_physics(physics);if(dt){positive(*dt,"timestep");config=configuration::resolve_physics(Value{Object{{"timestep_s",value::real(*dt)}}},std::move(config));}config=configuration::resolve_physics(overrides,std::move(config));return config;
}
model::BuildRequest profile_model(const std::filesystem::path& physics,std::optional<double> dt,const Value& overrides){
    return mechanics::request(profile_config(physics,dt,overrides),true);
}
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
Simulation::Simulation(initialization::SetupRequest input):request(std::move(input)) {
    request.runtime.record_retention=runtime::RetentionMode::full;request.runtime.measurement_decimation=1;request.runtime.async_telemetry=false;
    seal_into(request,equilibrium,metadata,runtime);
}
NATIVE_DIAG_POP
initialization::PhysicalInitialState Simulation::initial_state(std::optional<int> stations){return RuntimeRigAccess::initial_state(*runtime,request,equilibrium,metadata,stations);}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::unique_ptr<initialization::SetupRequest> profile_request(const std::filesystem::path& physics,const std::filesystem::path& track,std::optional<double> dt,const Value& overrides) {
    auto request=std::make_unique<initialization::SetupRequest>();request->model=profile_model(physics,dt,overrides);request->track=terrain::load_track(track);request->model.field=terrain::HeightFieldSpec::for_track(request->track);
    request->asset_root=physics.parent_path();request->origin="validation:"+physics.string();return request;
}
NATIVE_DIAG_POP
runtime::RideControl resume_schedule(double time){nonnegative(time,"schedule time");return runtime::RideControl{.motor_torque_nm=0.,.human_torque_nm=time<2.||time>=4.?20.:0.};}
runtime::RideControl automatic_schedule(double time){nonnegative(time,"schedule time");return {};}
runtime::RideControl open_loop_schedule(double time){nonnegative(time,"schedule time");throw std::invalid_argument(std::string(motor_only_unsupported));}
Value diagnostic_row(const runtime::PhysicalSampleData& sample) {
    const auto c=research::wire_value(runtime::Wire{sample.channels});Object row{{"interval_id",value::integer(sample.interval_id)},{"time_s",value::real(sample.time_s)},{"end_time_s",value::real(sample.end_time_s)}};
    for(const auto& [target,source]:std::array<std::pair<const char*,const char*>,7>{{{"drive","drive"},{"rider","rider"},{"support","rider_support_targets"},{"joint_terms","rider_control"},{"ik_saturation","rider_ik_saturation"},{"tires","tires"},{"energy","energy"}}})row.emplace_back(target,get(c,source));
    for(const char* name:{"control","model_status","rider_intent","suspension","mass"})row.emplace_back(name,optional(c,name,empty_object()));
    for(const auto& [key,x]:io::as_object(c,"channels"))if(key.starts_with("rider_active")||key=="rider_positive_power_w"||key=="rider_activation_saturated"||key=="rider_passive_power_w"||key=="rider_effort_budget_exceeded")row.emplace_back(key,x);return Value{std::move(row)};
}
Value interval_row(const runtime::PhysicalSampleData& sample,const runtime::TruthGeometry& geometry,runtime::WheelieTracker& tracker) {
    const auto c=research::wire_value(runtime::Wire{sample.channels});const auto truth=runtime::research_truth(sample,geometry);tracker.update(truth,sample.dt_s());const auto quality=research::energy_quality(runtime::sample_wire::object(runtime::sample_wire::required(sample.channels,"energy")));
    auto row=metrics({{"time_s",sample.time_s},{"end_time_s",sample.end_time_s},{"position_m",truth.position_m},{"speed_mps",truth.speed_mps},{"pitch_rad",truth.pitch_up_rad},{"pitch_rate_rad_s",truth.pitch_rate_up_rad_s},
        {"com_x_m",truth.com_x_m},{"com_z_m",truth.com_z_m},{"fork_travel_m",real(get(c,"suspension"),"fork_travel_m")},{"shock_stroke_m",real(get(c,"suspension"),"shock_stroke_m")},
        {"human_torque_nm",real(get(c,"drive"),"human_sensor_nm",0.)},{"motor_torque_nm",real(get(c,"drive"),"motor_torque_nm",0.)},{"crank_rad_s",real(get(c,"drive"),"crank_rad_s",0.)},
        {"drive_shaft_rad_s",real(get(c,"drive"),"drive_shaft_rad_s",real(get(c,"drive"),"crank_rad_s",0.))},{"rear_slip_mps",truth.rear_slip_mps},{"joint_positive_power_w",real(c,"rider_positive_power_w",0.)},{"energy_residual_ratio",quality.residual_ratio},
        {"front_load_n",truth.front_load_n},{"rear_load_n",truth.rear_load_n},{"front_vertical_force_n",real(get(get(c,"tires"),"front"),"vertical_force_n")},{"rear_vertical_force_n",real(get(get(c,"tires"),"rear"),"vertical_force_n")},{"front_clearance_m",truth.front_clearance_m},{"rear_clearance_m",truth.rear_clearance_m}});
    auto status=get(c,"model_status");value::set(std::get<Object>(status.data),"numerically_valid",Value{flag(status,"numerically_valid")&&quality.acceptable});auto& fields=std::get<Object>(row.data);fields.emplace_back("contact_state",value::text(tracker.state()));fields.emplace_back("control",get(c,"control"));fields.emplace_back("model_status",std::move(status));fields.emplace_back("energy",get(c,"energy"));return row;
}
namespace {
using RowSink=std::function<void(const runtime::SamplePtr&)>;
using BoundarySink=std::function<void(const runtime::RenderFrame&)>;
void drain(runtime::NativeRideRuntime& ride,const RowSink& sink){const auto batch=ride.prepare_samples();for(const auto& row:batch.samples)sink(row);ride.acknowledge_samples(batch);}
void replay(Simulation& sim,const std::function<runtime::RideControl(double)>& schedule,double duration,const RowSink& sink,double front=0.,double rear=0.,const BoundarySink& boundary={},bool require_complete=true) {
    const double dt=sim.request.model.physics->timestep_s;const auto count=steps(duration,dt,1e-9);
    const auto initial=sim.runtime->snapshot();IntervalCoverage coverage(initial.step,initial.time_s);
    const auto collected=[&](const runtime::SamplePtr& sample){coverage.observe(*sample);sink(sample);};
    try {for(std::size_t i=0;i<count;++i){const auto current=sim.runtime->snapshot();const auto result=sim.runtime->advance(current.step+1,schedule(current.time_s),front,rear,{});
            // Live properties in ride_cases.py observe the committed endpoint
            // after each sim.step(), before any later publication batch. A
            // sample's suspension channel instead describes incoming qpos.
            if(boundary&&result.step==current.step+1)boundary(sim.runtime->render_frame());
            drain(*sim.runtime,collected);if(result.outcome)break;}
        sim.runtime->flush();drain(*sim.runtime,collected);
        const auto final=sim.runtime->snapshot();coverage.finish(final.step,final.time_s);
        if(require_complete&&(static_cast<std::size_t>(final.step-initial.step)!=count||final.outcome))
            throw std::runtime_error("fixed-duration validation case ended before an outcome-free full horizon");
    }
    catch(...){try{sim.runtime->flush();drain(*sim.runtime,collected);}catch(...){ }throw;}
}
Value model_status(Simulation& sim){return research::wire_value(runtime::Wire{sim.runtime->snapshot().model_status});}
double position(Simulation& sim){return sim.runtime->render_frame().presentation.position_m;}
void merge(Value& out,const Value& in){for(const auto& [key,x]:io::as_object(in,"object"))value::set(std::get<Object>(out.data),key,x);}
configuration::SimulationPhysicsConfig simple_config(double dt,std::string_view drive,double human,double mu,bool brakes,std::string_view backend,double speed){
    configuration::SimulationPhysicsConfig c;c.physics_mode="physical";c.drive_mode=drive;c.timestep_s=dt;c.tires.backend=backend;c.tires.front.mu=mu;c.tires.rear.mu=mu;c.initial_speed_mps=speed;c.initial_front_brake=brakes?1.:0.;c.initial_rear_brake=brakes?1.:0.;c.drive.human_torque_nm=human;c.drive.assist.gain=0.;c.resistance.crr=0.;c.resistance.cda_m2=0.;return c;
}
model::BuildRequest simple_model(double dt,std::string_view rider,double speed,std::string_view drive,double human,double mu,bool brakes,std::string_view backend){
    return mechanics::request(simple_config(dt,drive,human,mu,brakes,backend,speed),rider=="articulated_planar");
}
// Request builders return heap owners so the multi-kilobyte SetupRequest and
// BuildRequest temporaries stay out of the per-case handler frames.
std::unique_ptr<initialization::SetupRequest> simple_request(double dt,std::string_view rider="lumped",double speed=0.,std::string_view drive="coast",double human=0.,double mu=.8,bool brakes=false,std::string_view backend="compliant_2d") {
    auto out=std::make_unique<initialization::SetupRequest>();out->model=simple_model(dt,rider,speed,drive,human,mu,brakes,backend);out->model.rider->variant=rider;out->track=terrain::get_preset("flat");return out;
}
double expected_mass(const Simulation& sim){return sim.request.model.masses.total_bike_mass()+sim.request.model.rider->total_rider_mass();}
}
Value run_replay(const Value& config,const std::filesystem::path& root) {
    const auto mode=text(config,"mode","human-only");const std::array<std::string_view,7> valid{"human-only","motor40","assist","shifting","rollback","open-loop","automatic"};if(std::ranges::find(valid,mode)==valid.end())throw std::invalid_argument("unknown isolated replay factor");
    const auto physics=root/text(config,"physics_profile","examples/research/rider_resume_physics.toml"),track=root/text(config,"track","examples/research/rider_resume_flat.toml");const double duration=real(config,"duration_s",8.);Value overrides=empty_object();
    if(mode=="shifting")overrides=Value{Object{{"drive",Value{Object{{"shifting",Value{Object{{"enabled",Value{true}}}}}}}}}};
    if(mode=="rollback")overrides=Value{Object{{"drive",Value{Object{{"pedaling",Value{Object{{"rollback_brake",Value{true}}}}}}}}}};
    if(const auto* tau=find(config,"activation_tau_s"))if(!io::is_null(*tau))value::set(std::get<Object>(overrides.data),"articulated",Value{Object{{"activation_tau_s",*tau},{"active_positive_power_limit_w",value::real(600.)}}});
    Value report{Object{{"schema_version",value::integer(1)},{"mode",value::text(mode)},{"physics_path",value::text(physics.string())},{"track_path",value::text(track.string())},{"duration_requested_s",value::real(duration)},{"source_sha256",value::text(source_fingerprint(root/"native/src"))},{"environment",environment_contract(root)},{"calibration_status",value::text("parameterized_unvalidated")}}};Array rows;std::unique_ptr<Simulation> sim;
    try{if(mode=="open-loop")throw std::invalid_argument(std::string(motor_only_unsupported));std::optional<double> dt;if(find(config,"dt_s"))dt=real(config,"dt_s");sim=std::make_unique<Simulation>(std::move(*profile_request(physics,track,dt,overrides)));merge(report,sim->metadata);
        replay(*sim,[&](double t){if(mode=="automatic")return automatic_schedule(t);auto control=resume_schedule(t);control.motor_torque_nm=mode=="motor40"?std::optional(40.):mode=="assist"?std::nullopt:std::optional(0.);return control;},duration,[&](const auto& sample){rows.push_back(diagnostic_row(*sample));},0.,0.,{},false);
        value::set(std::get<Object>(report.data),"evidence",mode=="automatic"?coasting_evidence(Value{rows}):resume_evidence(Value{rows},std::max(8.,duration)));const auto final=sim->runtime->snapshot();value::set(std::get<Object>(report.data),"crash",final.outcome?value::text(*final.outcome):Value{});value::set(std::get<Object>(report.data),"completed_requested_duration",Value{std::abs(final.time_s-duration)<1e-8});}
    catch(const std::exception& e){auto& out=std::get<Object>(report.data);value::set(out,"error",value::text(e.what()));value::set(out,"diagnosis",value::text(mode=="open-loop"?"unsupported_experiment":"mixed_or_unresolved"));value::set(out,"completed_requested_duration",Value{false});}
    auto& out=std::get<Object>(report.data);value::set(out,"rows",Value{rows});try{value::set(out,"support_evidence",support_evidence(Value{rows},duration));}catch(const std::exception& e){value::set(out,"support_evidence",Value{Object{{"interval_count",value::integer(static_cast<std::int64_t>(rows.size()))},{"complete",Value{false}},{"valid_for_learning",Value{false}},{"evaluation_error",value::text(e.what())}}});value::set(out,"completed_requested_duration",Value{false});}
    if(sim){value::set(out,"end_time_s",value::real(sim->runtime->snapshot().time_s));value::set(out,"end_position_m",value::real(position(*sim)));value::set(out,"model_status",model_status(*sim));}if(const auto* file=find(config,"output"))write_report(root/io::as_string(*file,"output"),report);return report;
}
Value rider_factor_matrix(const Value& config,const std::filesystem::path& root) {
    const auto out=root/text(config,"output","results/rider_factors");std::filesystem::create_directories(out);struct Variant{const char* name;const char* mode;const char* track;std::optional<double> tau;};
    const std::array<Variant,9> variants{{{"human_flat","human-only","rider_resume_flat.toml",{}},{"human_incline","human-only","rider_resume_incline.toml",{}},{"motor_40","motor40","rider_resume_flat.toml",{}},{"assist","assist","rider_resume_flat.toml",{}},{"shifting","shifting","rider_resume_flat.toml",{}},{"rollback","rollback","rider_resume_incline.toml",{}},{"active_tau0","human-only","rider_resume_flat.toml",0.},{"active_tau005","human-only","rider_resume_flat.toml",.05},{"motor_open_loop","open-loop","rider_resume_flat.toml",{}}}};
    Value summary{Object{
        {"schema_version",value::integer(1)}, {"duration_s",value::real(10.)},
        {"resume_window_s",pair(4.,10.)}, {"synthetic",Value{true}},
        {"anti_wheelie_policy_used",Value{false}}, {"variants",Value{Array{}}}
    }};
    Array entries;bool resumed=true,resolved=true;
    for(const auto& variant:variants){const std::string file=std::string(variant.name)+".json.gz";Value request{Object{{"mode",value::text(variant.mode)},{"duration_s",value::real(10.)},{"track",value::text((root/"examples/research"/variant.track).string())},{"physics_profile",optional(config,"physics_profile",value::text((root/"examples/research"/(std::string_view(variant.mode)=="open-loop"?"plant_reference_open_loop.toml":"rider_resume_physics.toml")).string()))},{"output",value::text((out/file).string())}}};if(variant.tau)value::set(std::get<Object>(request.data),"activation_tau_s",value::real(*variant.tau));auto result=run_replay(request,root);auto& fields=std::get<Object>(result.data);std::erase_if(fields,[](const auto& field){return field.first=="rows";});fields.emplace_back("name",value::text(variant.name));fields.emplace_back("file",value::text(file));
        if(std::string_view(variant.name)=="human_flat"||std::string_view(variant.name)=="human_incline"){const auto diagnosis=text(optional(result,"evidence",empty_object()),"diagnosis");const std::array<std::string_view,5> known{"resumed","support_unavailable","actuator_saturated","assist_gated","mechanically_stalled"};resumed=resumed&&diagnosis=="resumed";resolved=resolved&&std::ranges::find(known,diagnosis)!=known.end()&&flag(result,"completed_requested_duration");}entries.push_back(std::move(result));value::set(std::get<Object>(summary.data),"variants",Value{entries});write_report(out/"report.json",summary);}
    auto& fields=std::get<Object>(summary.data);fields.emplace_back("baseline_resumed",Value{resumed});fields.emplace_back("causal_replay_resolved",Value{resolved});write_report(out/"report.json",summary);return summary;
}
Value automatic_coast_case(double dt,double duration,const std::filesystem::path& root) {
    auto report=run_replay(Value{Object{{"mode",value::text("automatic")},{"dt_s",value::real(dt)},{"duration_s",value::real(duration)},{"physics_profile",value::text("examples/research/viewer_physics_fast.toml")},{"track",value::text("examples/research/rough_uphill_extreme.toml")}}},root);
    if(find(report,"error"))return report;const auto rows=get(report,"rows");Value result=coasting_evidence(rows);merge(result,Value{Object{{"metadata",report},{"rows",rows},{"duration_s",get(report,"end_time_s")},{"model_valid",Value{flag(get(report,"model_status"),"model_valid")}}}});bool numeric_valid=!io::as_array(rows,"rows").empty();for(const auto& row:io::as_array(rows,"rows"))numeric_valid=numeric_valid&&research::energy_quality(runtime::sample_wire::object(vread::wire_from_value(get(row,"energy"),"energy"))).acceptable;value::set(std::get<Object>(result.data),"numerically_valid",Value{numeric_valid});return result;
}

namespace {
// One handler per registered case keeps each -O0 frame under the
// -Wframe-larger-than budget; Simulation and SetupRequest stay on the heap.
RigResult ride_flat_static(std::string_view name,double dt){
    const auto sim=std::make_unique<Simulation>(std::move(*simple_request(dt,"lumped",0.,"coast",0.,.8,false,name=="flat_static_native"?"native_reference":"compliant_2d")));
    return {RuntimeRigAccess::static_metrics(*sim->runtime,sim->metadata,expected_mass(*sim)),bounds({{"mass_error_kg",0.,1e-8},{"force_relative_error",0.,.005},{"moment_relative_error",0.,.005},{"residual_qacc",0.,.05}})};
}
configuration::SimulationPhysicsConfig sag_config(double dt){
    configuration::SimulationPhysicsConfig physics;physics.physics_mode="physical";physics.timestep_s=dt;physics.tires.backend="compliant_2d";return physics;
}
model::BuildRequest sag_model(double dt){
    auto out=mechanics::request(sag_config(dt));out.rider->variant="lumped";return out;
}
std::unique_ptr<initialization::SetupRequest> sag_request(double dt){
    auto request=std::make_unique<initialization::SetupRequest>();
    request->model=sag_model(dt);request->track=terrain::get_preset("flat");return request;
}
std::array<double,2> fit_requested_sag(const initialization::SetupRequest& request,const std::array<double,2>& target,const std::array<double,2>& initial){
    return initialization::fit_sag(initialization::build_equilibrium_evaluator(request),target,initial,{10.,1000.},{400.,250000.});
}
RigResult ride_requested_sag(std::string_view,double dt){
    const auto request=sag_request(dt);
    auto& specs=request->model.bike;const std::array target{specs.fork_travel*.3,specs.rear_wheel_travel*.3},initial{specs.fork_initial_psi,specs.shock_stiffness};
    const auto fitted=fit_requested_sag(*request,target,initial);
    specs.fork_initial_psi=fitted[0];specs.shock_stiffness=fitted[1];const auto sim=std::make_unique<Simulation>(std::move(*request));
    auto result=RuntimeRigAccess::static_metrics(*sim->runtime,sim->metadata,expected_mass(*sim));
    merge(result,metrics({{"fork_error_mm",std::abs(sim->equilibrium.fork_travel_mm-target[0])},{"rear_error_mm",std::abs(sim->equilibrium.rear_travel_mm-target[1])},{"fork_pressure_psi",fitted[0]},{"coil_rate_n_m",fitted[1]}}));
    return {std::move(result),bounds({{"fork_error_mm",0.,.5},{"rear_error_mm",0.,.5},{"force_relative_error",0.,.005},{"moment_relative_error",0.,.005}})};
}
RigResult ride_brake_hold(std::string_view,double dt){
    const double inf=std::numeric_limits<double>::infinity();
    const auto request=simple_request(dt,"lumped",0.,"coast",0.,.8,true);request->track={.name="incline_10_deg",.length_m=8.,.grade_profile=terrain::GradeProfile{{{0.,std::tan(10.*std::numbers::pi/180.)},{8.,std::tan(10.*std::numbers::pi/180.)}}}};request->model.field={.ncol=1601,.radius_x_m=4.,.elevation_m=2.,.datum_z_m=.1};const auto sim=std::make_unique<Simulation>(std::move(*request));const double start=position(*sim);replay(*sim,automatic_schedule,5.,[](const auto&){},1.,1.);const auto frame=sim->runtime->render_frame();const auto account=research::wire_value(runtime::Wire{sim->runtime->accounting_state()});const auto work=optional(get(account,"history"),"work_j",empty_object());return {metrics({{"displacement_m",std::abs(frame.presentation.position_m-start)},{"speed_mps",std::abs(frame.presentation.speed_mps)},{"motor_torque_nm",std::get<double>(frame.presentation.drive.at("motor_torque_nm"))},{"front_brake_work_j",real(work,"front_static_brake",0.)},{"rear_brake_work_j",real(work,"rear_static_brake",0.)}}),bounds({{"displacement_m",0.,.001},{"motor_torque_nm",0.,0.},{"front_brake_work_j",-inf,1e-8},{"rear_brake_work_j",-inf,1e-8}})};
}
RigResult ride_drive_low_mu(std::string_view,double dt){
    const double inf=std::numeric_limits<double>::infinity();
    const auto sim=std::make_unique<Simulation>(std::move(*simple_request(dt,"lumped",0.,"crank_effort",80.,.1)));double slip=0.,excess=0.,human=0.,load=0.,impulse=0.;replay(*sim,automatic_schedule,.5,[&](const auto& s){const auto c=research::wire_value(runtime::Wire{s->channels});const auto& rear=get(get(c,"tires"),"rear");slip=std::max(slip,std::abs(real(rear,"slip_mps")));excess=std::max(excess,std::abs(real(rear,"tangent_force_n"))-.1*real(rear,"normal_load_n"));human=std::max(human,real(get(c,"drive"),"human_torque_nm"));load+=real(rear,"normal_load_n")*dt/.5;impulse+=real(rear,"tangent_force_n")*dt;});return {metrics({{"max_slip_mps",slip},{"friction_excess_n",excess},{"human_peak_nm",human},{"mean_rear_load_n",load},{"traction_impulse_ns",impulse},{"speed_mps",sim->runtime->render_frame().presentation.speed_mps}}),bounds({{"max_slip_mps",.1,inf},{"friction_excess_n",0.,1e-6},{"human_peak_nm",50.,inf}})};
}
RigResult ride_airborne(std::string_view name,double dt){
    const auto sim=std::make_unique<Simulation>(std::move(*simple_request(dt,"articulated_planar")));const bool passive=name=="airborne_passive";const auto initial=RuntimeRigAccess::rebase_airborne(*sim->runtime,30.,0.,true,true,passive,false);const auto angular=numbers(get(initial,"angular_momentum_kg_m2_s"),"angular momentum");
    if(passive){double maximum=0.;Value last;replay(*sim,automatic_schedule,2.,[&](const auto& sample){const auto c=research::wire_value(runtime::Wire{sample->channels});last=get(c,"endpoint_mass");const auto a=numbers(get(last,"angular_momentum_kg_m2_s"),"angular momentum");maximum=std::max(maximum,std::abs(a[1]-angular[1]));});const auto initial_com=numbers(get(initial,"com_m"),"com"),velocity=numbers(get(initial,"com_velocity_mps"),"velocity"),end=numbers(get(last,"com_m"),"end com");std::array<double,3> error{};RuntimeRigAccess::inspect(*sim->runtime,[&](const Stepper& stepper,const auto&){const auto gravity=std::span{stepper.model()->opt.gravity};for(std::size_t a=0;a<3;++a)error[a]=end[a]-(initial_com[a]+2.*velocity[a]+.5*gravity[a]*4.);});const double scale=std::max(std::abs(angular[1]),1.);return {metrics({{"relative_Ly_drift",maximum/scale},{"Ly_scale_kg_m2_s",scale},{"ballistic_position_error_m",vector_norm(error)},{"root_pitch_rad",sim->runtime->render_frame().presentation.pitch_rad}}),bounds({{"relative_Ly_drift",0.,.001},{"ballistic_position_error_m",0.,std::max(.012,10.*dt)}})};}
    std::vector<double> incoming,force;RuntimeRigAccess::inspect(*sim->runtime,[&](const Stepper& stepper,const auto&){auto* m=stepper.model();auto* d=stepper.data();const auto nv=static_cast<std::size_t>(m->nv);incoming.resize(nv);force.resize(nv);std::vector<double> jp(3*nv),jr(3*nv);const auto body_mass=model_access::readonly_buffer(m->body_mass,m->nbody,"body_mass");for(mjtSize b=0;b<m->nbody;++b){const char* body=mj_id2name(m,mjOBJ_BODY,static_cast<int>(b));if(body&&std::string_view(body).starts_with("rider_"))continue;mj_jacBodyCom(m,d,jp.data(),jr.data(),static_cast<int>(b));for(std::size_t q=0;q<nv;++q)incoming[q]+=body_mass[static_cast<std::size_t>(b)]*jp[q];}force[static_cast<std::size_t>(mechanics::dof(m,"root_x"))]=100.;});RuntimeRigAccess::external_interval(*sim->runtime,force);double rider_p=0.,bike_p=0.,contact_force=0.,mass_error=0.,impulse_error=0.;RuntimeRigAccess::inspect(*sim->runtime,[&](const Stepper& stepper,const auto&){const auto* m=stepper.model();const auto* d=stepper.data();const auto nv=static_cast<std::size_t>(m->nv);const auto body_mass=model_access::readonly_buffer(m->body_mass,m->nbody,"body_mass"),qvel=model_access::readonly_buffer(d->qvel,m->nv,"qvel");std::vector<double> jp(3*nv),jr(3*nv);for(mjtSize b=0;b<m->nbody;++b){mj_jacBodyCom(m,d,jp.data(),jr.data(),static_cast<int>(b));const double momentum=body_mass[static_cast<std::size_t>(b)]*mechanics::matvec(jp,qvel)[0];const char* body=mj_id2name(m,mjOBJ_BODY,static_cast<int>(b));if(body&&std::string_view(body).starts_with("rider_"))rider_p+=momentum;else bike_p+=momentum;}numeric::PythonSum contact;const auto& diag=stepper.rider_contacts().state().diagnostics;for(const auto& support:diag.supports)contact.add(vector_norm(support.force_on_rider_n));for(const auto& grip:diag.grip_sides)contact.add(vector_norm(grip.force_on_rider_n));contact_force=contact.total();mass_error=std::abs(numeric::numpy_sum(body_mass)-real(initial,"mass_kg"));impulse_error=std::abs(vector_dot(incoming,qvel)-100.*dt);});return {metrics({{"rider_horizontal_momentum_kg_mps",std::abs(rider_p)},{"contact_force_n",contact_force},{"mass_change_kg",mass_error},{"bike_impulse_error_ns",impulse_error},{"endpoint_bike_momentum_error_ns",std::abs(bike_p-100.*dt)},{"bike_root_speed_mps",sim->runtime->render_frame().presentation.speed_mps}}),bounds({{"rider_horizontal_momentum_kg_mps",0.,1e-9},{"contact_force_n",0.,1e-9},{"mass_change_kg",0.,1e-10},{"bike_impulse_error_ns",0.,1e-5}})};
}
RigResult ride_terrain_resolution(std::string_view,double dt){
    std::array<Value,2> results;
    for(std::size_t i=0;i<2;++i){
        const auto request=simple_request(dt,"lumped",3.);
        request->track={.name="spatial_refinement_rig",.length_m=20.,.obstacles={{terrain::Washboard{3.,.002,2.,6}}}};
        request->model.field.ncol=i==0?1001:2001;request->model.field.radius_x_m=10.;const auto sim=std::make_unique<Simulation>(std::move(*request));
        std::vector<double> loads,travels;
        replay(*sim,automatic_schedule,2.,[&](const auto& sample){
            const auto c=research::wire_value(runtime::Wire{sample->channels});numeric::PythonSum sum;
            for(const char* side:{"front","rear"})sum.add(real(get(get(c,"tires"),side),"normal_load_n"));
            loads.push_back(sum.total());
        },0.,0.,[&](const auto& endpoint){travels.push_back(endpoint.presentation.fork_travel_mm/1000.);});
        std::vector<double> squared;for(double x:loads)squared.push_back(x*x);
        results[i]=metrics({{"mean_load_n",mean(loads)},{"rms_load_n",std::sqrt(mean(squared))},
            {"normal_impulse_ns",numeric::numpy_sum(loads)*dt},{"mean_fork_travel_m",mean(travels)}});
    }
    Value result=metrics({{"coarse_dx_m",.02},{"fine_dx_m",.01}}),criteria=empty_object();
    for(const auto& [key,x]:io::as_object(results[1],"fine metrics")){
        value::set(std::get<Object>(result.data),key+"_relative_change",value::real(relative_change(real(results[0],key),io::as_double(x,key),1e-4)));
        value::set(std::get<Object>(result.data),key+"_fine",x);value::set(std::get<Object>(criteria.data),key+"_relative_change",pair(0.,.02));
    }
    return {std::move(result),std::move(criteria)};
}
RigResult ride_coast(std::string_view name,double dt){
    const double inf=std::numeric_limits<double>::infinity();
    const double duration=name=="single_edge"?2.:5.;const auto request=simple_request(dt,"lumped",3.);
    if(name=="single_edge")request->track={.name="single_edge_rig",.length_m=20.,.obstacles={{terrain::SquareEdge{3.5,.04,.5}}}};
    else if(name=="road_worn")request->track=terrain::get_preset("road_worn");
    else request->track={.name="smooth_coast_rig",.length_m=20.,.obstacles={{terrain::Washboard{3.,.002,2.,6}}}};
    const auto sim=std::make_unique<Simulation>(std::move(*request));const double start=position(*sim);std::vector<double> loads,travels;
    std::array<std::vector<double>,2> wheel;double maximum=0.,impulse=0.,residual=0.;std::size_t multi=0;
    replay(*sim,automatic_schedule,duration,[&](const auto& s){
        const auto c=research::wire_value(runtime::Wire{s->channels});const auto& energy=get(c,"energy");
        residual=std::abs(real(energy,"residual_j"))/real(energy,"energy_scale_j");maximum=std::max(maximum,residual);
        numeric::PythonSum sum;bool multiple=false;
        for(std::size_t side=0;side<2;++side){
            const auto& tire=get(get(c,"tires"),side==0?"front":"rear");const double load=real(tire,"normal_load_n");
            sum.add(load);wheel[side].push_back(load);multiple=multiple||flag(tire,"multi_support");
        }
        loads.push_back(sum.total());impulse+=sum.total()*dt;multi+=multiple?1:0;
    },0.,0.,[&](const auto& endpoint){travels.push_back(endpoint.presentation.fork_travel_mm/1000.);});
    const auto rms=[](const auto& data){std::vector<double> squares;for(double x:data)squares.push_back(x*x);return std::sqrt(mean(squares));};
    const auto account=research::wire_value(runtime::Wire{sim->runtime->accounting_state()});const auto& airtime=get(get(account,"history"),"airtime_s");
    Value result=metrics({{"mean_load_n",mean(loads)},{"rms_load_n",rms(loads)},{"normal_impulse_ns",impulse},
        {"mean_fork_travel_m",mean(travels)},{"distance_m",position(*sim)-start},{"energy_residual_relative",residual},
        {"max_energy_residual_relative",maximum},{"multi_support_duration_s",static_cast<double>(multi)*dt},
        {"front_airtime_s",real(get(airtime,"front"),"0.0")},{"rear_airtime_s",real(get(airtime,"rear"),"0.0")}});
    for(std::size_t side=0;side<2;++side){const std::string key=side==0?"front":"rear";
        merge(result,metrics({{key+"_mean_load_n",mean(wheel[side])},{key+"_rms_load_n",rms(wheel[side])},
            {key+"_normal_impulse_ns",numeric::numpy_sum(wheel[side])*dt}}));}
    const auto geometry=RuntimeRigAccess::truth_geometry(*sim->runtime);double low=inf,high=-inf;
    for(std::size_t i=0;i<geometry.x.size();++i)if(geometry.x[i]>=start&&geometry.x[i]<=position(*sim)){
        low=std::min(low,geometry.z[i]);high=std::max(high,geometry.z[i]);}
    value::set(std::get<Object>(result.data),"visited_profile_span_m",value::real(std::isfinite(low)?high-low:0.));
    auto criteria=bounds({{"distance_m",0.,inf},{"mean_load_n",0.,inf}});
    if(name=="road_worn")value::set(std::get<Object>(criteria.data),"visited_profile_span_m",pair(1e-5,inf));
    if(name=="smooth_coast")value::set(std::get<Object>(criteria.data),"energy_residual_relative",pair(0.,.01));
    return {std::move(result),std::move(criteria)};
}
struct RideCaseHandler{std::string_view name;RigResult (*run)(std::string_view,double);};
constexpr auto ride_handlers=std::to_array<RideCaseHandler>({
    {"flat_static",ride_flat_static},{"flat_static_native",ride_flat_static},
    {"requested_sag",ride_requested_sag},{"brake_hold",ride_brake_hold},{"drive_low_mu",ride_drive_low_mu},
    {"airborne_passive",ride_airborne},{"released_rider",ride_airborne},{"terrain_resolution",ride_terrain_resolution},
    {"smooth_coast",ride_coast},{"single_edge",ride_coast},{"road_worn",ride_coast}});
}
RigResult ride_case(std::string_view name,double dt) {
    if(const auto found=std::ranges::find_if(ride_handlers,[&](const RideCaseHandler& handler){return handler.name==name;});found!=ride_handlers.end())return found->run(name,dt);
    throw std::invalid_argument("unknown compiled ride case");
}
} // namespace bike::validation
