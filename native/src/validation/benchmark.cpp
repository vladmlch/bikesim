// benchmarks.py:1-240, antiwheelie_bench.py:1-133,
// climbing_benchmark.py:1-166. Cold orchestration over the actual native model.
#include "scenarios.hpp"
#include "../diag.hpp"
#include "acceptance.hpp"
#include "evidence.hpp"
#include "mechanics.hpp"
#include "tire.hpp"
#include "../configuration/load.hpp"
#include "../engine_identity.hpp"
#include "../platform/provenance.hpp"
#include "../research/policy_session.hpp"
#include "../research/evidence.hpp"
#include "../research/lifecycle.hpp"
#include "../recording/research_publication.hpp"
#include "../cli/physical_capture.hpp"
#include <chrono>
#include <format>
#include <map>
#include <sys/utsname.h>
#include <thread>

namespace bike::validation {
namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point start){return std::chrono::duration<double>(Clock::now()-start).count();}
void set(Value& out,std::string key,Value item){value::set(std::get<Object>(out.data),std::move(key),std::move(item));}
void merge(Value& out,const Value& from){for(const auto& [key,item]:io::as_object(from,"report"))set(out,key,item);}
constexpr auto names=std::to_array<std::string_view>({
    "contact_event","terrain_resolution","requested_sag","wheel_inertia",
    "radial_100","radial_300","radial_600","radial_1000","radial_native_reference",
    "flat_static","flat_static_native","incline_30_deg","brake_hold","drive_low_mu",
    "airborne_internal_actuation","airborne_passive","released_rider","suspension_cycle",
    "chain_locked_geometry","chain_suspension_motion","smooth_coast","single_edge","road_worn",
    "distributed_flat_600","distributed_step","distributed_incline","rigid_load_transfer",
    "whole_system_passive","whole_system_active","motor_only_airborne","actual_shaft_ratio","actual_freehub_overrun",
    "drive_suspension_ideal_mid_drive_0","drive_suspension_ideal_mid_drive_20","drive_suspension_ideal_mid_drive_40",
    "drive_suspension_geometric_ideal_mid_drive_0","drive_suspension_geometric_ideal_mid_drive_20","drive_suspension_geometric_ideal_mid_drive_40",
    "drive_suspension_elastic_chain_0","drive_suspension_elastic_chain_20","drive_suspension_elastic_chain_40"});
Value versions(){const auto b=platform::build_identity();const auto e=engine::identity();return Value{Object{
    {"compiler",value::text(b.compiler)},{"mujoco",value::text(e.mujoco_version)},
    {"numeric_provider",value::text(e.numeric_provider)},{"execution_identity",value::text(platform::execution_identity())}}};}
Value bounded(std::string_view name,const RigResult& result,bool require_criteria) {
    const auto checks=check_bounds(result.metrics,result.bounds);Object criteria;
    for(const auto& [key,range]:io::as_object(result.bounds,"bounds")){const auto& limits=io::as_array(range,"range");
        const auto lo=io::tagged_double(limits[0],"minimum"),hi=io::tagged_double(limits[1],"maximum");
        criteria.emplace_back(key,Value{Object{{"actual",get(result.metrics,key)},
            {"minimum",std::isfinite(lo)?value::real(lo):Value{}},{"maximum",std::isfinite(hi)?value::real(hi):Value{}},
            {"passed",get(checks,key)}}});}
    return Value{Object{{"name",value::text(name)},{"metrics",result.metrics},{"criteria",Value{criteria}},
        {"passed",Value{(!require_criteria||!criteria.empty())&&checks_passed(checks)}}}};
}
Value execute(std::string_view name,double dt){
    const auto start=Clock::now();const bool diagnostic=name=="radial_native_reference";
    Value row{Object{{"case_id",value::text(name)},{"physics_mode",value::text("physical")},
        {"backend",value::text(name.find("native")!=std::string_view::npos?"native_reference":"compliant_2d")},
        {"dt_s",value::real(dt)},{"mandatory",Value{!diagnostic}}}};
    try{const auto result=benchmark_case(name,dt);
        for(const auto& [key,item]:io::as_object(result.metrics,"metrics"))
            if(!std::isfinite(io::tagged_double(item,key)))throw std::runtime_error("non-finite validation metric");
        auto measured=bounded(name,result,false);const bool passed=flag(measured,"passed");
        for(const char* field:{"metrics","criteria","passed"})set(row,field,get(measured,field));
        set(row,"status",value::text(diagnostic?"diagnostic":passed?"passed":"failed"));
    }catch(const std::exception& e){set(row,"metrics",empty_object());set(row,"criteria",empty_object());
        set(row,"passed",Value{false});set(row,"status",value::text("error"));set(row,"error",value::text(e.what()));}
    set(row,"wall_time_s",value::real(elapsed(start)));return row;
}
std::map<std::string,std::vector<std::pair<std::string,double>>> convergence_keys(){
    std::map<std::string,std::vector<std::pair<std::string,double>>> result{
        {"wheel_inertia",{{"omega_rad_s",1.}}},{"flat_static",{{"fork_travel_m",.001},{"rear_travel_m",.001}}},
        {"drive_low_mu",{{"mean_rear_load_n",1.},{"traction_impulse_ns",.1}}},
        {"suspension_cycle",{{"cycle_loss_j",.1}}},{"chain_locked_geometry",{{"speed_ratio",1.}}}};
    for(const char* name:{"radial_100","radial_300","radial_600","radial_1000"})result[name]={{"deflection_m",.0001}};
    for(const char* name:{"smooth_coast","single_edge","road_worn"}){auto& keys=result[name];
        keys={{"mean_load_n",1.},{"rms_load_n",1.},{"normal_impulse_ns",.1},{"mean_fork_travel_m",.001}};
        for(const char* side:{"front","rear"})for(const char* metric:{"mean_load_n","rms_load_n","normal_impulse_ns"})keys.emplace_back(std::string(side)+"_"+metric,1.);}
    return result;
}
Value actual_or_null(const Value& row,std::string_view key){return optional(get(row,"metrics"),key);}
// Extracted for the -O0 frame budget: the convergence scan, per-check try
// bodies and climbing setup/loop temporaries each live in their own frame.
Array benchmark_convergence(const std::vector<double>& dts,const std::vector<std::string>& chosen,const Array& rows,const std::map<std::pair<std::string,double>,std::size_t>& lookup){
    Array convergence;const double coarse=dts[dts.size()-2],fine=dts.back();
    for(const auto& [name,keys]:convergence_keys()){if(std::ranges::find(chosen,name)==chosen.end())continue;
        const auto& a=rows[lookup.at({name,coarse})];const auto& b=rows[lookup.at({name,fine})];
        for(const auto& [key,floor]:keys){Value change;bool passed=false;const auto x=actual_or_null(a,key),y=actual_or_null(b,key);
            if(!io::is_null(x)&&!io::is_null(y)){const double delta=relative_change(io::as_double(x,key),io::as_double(y,key),floor);change=value::real(delta);passed=delta<=.02;}
            convergence.push_back(Value{Object{{"case_id",value::text(name)},{"metric",value::text(key)},
                {"coarse_dt_s",value::real(coarse)},{"fine_dt_s",value::real(fine)},{"relative_change",change},
                {"scale_floor",value::real(floor)},{"limit",value::real(.02)},{"passed",Value{passed}}}});}}
    if(std::ranges::find(chosen,"contact_event")!=chosen.end()){const auto a=actual_or_null(rows[lookup.at({"contact_event",coarse})],"contact_time_s"),b=actual_or_null(rows[lookup.at({"contact_event",fine})],"contact_time_s");
        Value change;bool passed=false;if(!io::is_null(a)&&!io::is_null(b)){const double delta=std::abs(io::as_double(a,"time")-io::as_double(b,"time"));change=value::real(delta);passed=delta<=2.*fine+1e-12;}
        convergence.push_back(Value{Object{{"case_id",value::text("contact_event")},{"metric",value::text("contact_time_s")},{"absolute_change_s",change},{"limit_s",value::real(2.*fine)},{"passed",Value{passed}}}});}
    if(std::ranges::find(chosen,"smooth_coast")!=chosen.end()){const auto a=actual_or_null(rows[lookup.at({"smooth_coast",coarse})],"energy_residual_relative"),b=actual_or_null(rows[lookup.at({"smooth_coast",fine})],"energy_residual_relative");
        const bool passed=!io::is_null(a)&&!io::is_null(b)&&io::as_double(b,"residual")<=std::max(io::as_double(a,"residual"),1e-8);
        convergence.push_back(Value{Object{{"case_id",value::text("smooth_coast")},{"metric",value::text("energy_residual_refinement")},{"coarse_residual",a},{"fine_residual",b},{"passed",Value{passed}}}});}
    return convergence;
}
Value mechanics_check(std::string_view kind,const std::filesystem::path& out,const std::filesystem::path& profile,const std::filesystem::path& root){
    std::filesystem::path artifact;Value result,check{Object{{"name",value::text(kind)},{"passed",Value{false}},{"sha256",Value{}},{"error",Value{}}}};
    if(kind=="analytic")artifact=out/"analytic/report.json";else if(kind=="load_and_shaft")artifact=out/"load-and-shaft.json";
    else if(kind=="drive_suspension")artifact=out/"drive-suspension.json";else if(kind=="contact")artifact=out/"contact/report.json";else artifact=out/"rider/report.json";
    set(check,"artifact",value::text(artifact.string()));
    try{if(kind=="analytic")result=benchmark_suite(Value{Object{{"output",value::text((out/"analytic").string())},{"cases",value::strings(std::array<std::string,6>{
        "wheel_inertia","chain_suspension_motion","suspension_cycle","airborne_internal_actuation","incline_30_deg","brake_hold"})}}},root);
        else if(kind=="load_and_shaft"){Array cases;for(double grade:{0.,.12,.28,.35})for(const auto& [x,h]:std::array<std::pair<double,double>,2>{{{.45,.9},{.55,.8}}})
            cases.push_back(bounded(std::format("load_grade{}_com{}_{}",grade,x,h),load_transfer_rig(.00025,std::atan(grade),x,h),true));
            for(int rear:{10,24,51})for(bool overrun:{false,true})cases.push_back(bounded(std::format("shaft_34_{}_overrun{}",rear,overrun?"True":"False"),shaft_ratio_rig(.000125,34./rear,overrun),true));
            bool passed=!cases.empty();for(const auto& c:cases)passed=passed&&flag(c,"passed");result=Value{Object{{"cases",Value{std::move(cases)}},{"passed",Value{passed}}}};write_report(artifact,result);}
        else if(kind=="drive_suspension")result=drive_suspension_comparison(Value{Object{{"output",value::text(artifact.string())}}},root);
        else if(kind=="contact")result=contact_resolution(Value{Object{{"output",value::text((out/"contact").string())}}},root);
        else result=rider_factor_matrix(Value{Object{{"output",value::text((out/"rider").string())},{"physics_profile",value::text(profile.string())}}},root);
        set(check,"passed",Value{kind=="rider"?flag(result,"baseline_resumed")&&flag(result,"causal_replay_resolved"):flag(result,"passed")});set(check,"sha256",value::text(file_hash(artifact)));
    }catch(const std::exception& e){set(check,"error",value::text(e.what()));}
    return check;
}
struct ClimbingSession{std::unique_ptr<Simulation> sim;std::unique_ptr<research::PolicySession> session;std::unique_ptr<research::ResearchRun> publication;double dt=0.;};
ClimbingSession climbing_setup(std::string_view mode,double duration,const std::filesystem::path& physics,const std::filesystem::path& track,const std::filesystem::path& out,std::int64_t repetition,Value& result){
    const auto setup=profile_request(physics,track);const double dt=setup->model.physics->timestep_s;
    if(mode!="research"){ClimbingSession active;active.dt=dt;active.sim=std::make_unique<Simulation>(std::move(*setup));merge(result,active.sim->metadata);set(result,"equilibrium",get(active.sim->metadata,"equilibrium"));return active;}
    (void)steps(duration,.01,1e-9);auto request=std::make_unique<research::ResearchRequest>();request->setup=std::move(*setup);
    request->experiment.duration_s=duration;request->experiment.control_period_s=.01;request->experiment.actuator_delay_s=.005;request->experiment.record_decimation=80;
    request->sensors.sample_period_s=.005;request->capture_mode=runtime::ResearchCaptureMode::full;
    auto prepared=research::prepare_environment(std::move(request));merge(result,prepared->metadata);
    auto writer=recording::make_research_publication(recording::make_run_metadata("native","validation_climbing_benchmark",prepared->metadata),
        cli::model_catalog(prepared->bootstrap.ride.model),out/std::format("episode-{:04}",repetition));
    auto environment=runtime::make_research(std::make_unique<runtime::ResearchBootstrap>(std::move(prepared->bootstrap)));
    ClimbingSession active;active.dt=dt;active.session=std::make_unique<research::PolicySession>(std::move(environment),research::load_policy("passthrough"));
    active.publication=std::make_unique<research::ResearchRun>(*active.session,std::move(writer));return active;
}
// The snapshot/frame/transition temporaries of the control loop stay in this
// helper's frame under the -O0 -Wframe-larger-than budget.
struct ClimbingLoop{std::optional<std::pair<Clock::time_point,double>> warmed;double minimum_front=std::numeric_limits<double>::infinity();double gap=0.;};
ClimbingLoop climbing_loop(ClimbingSession& active,std::size_t required,std::size_t warmup,Value& result,std::vector<double>& timings){
    ClimbingLoop loop;
    const auto snapshot=[&]{return active.session?active.session->environment().snapshot():active.sim->runtime->snapshot();};
    const auto frame=[&]{return active.session?active.session->environment().render_frame():active.sim->runtime->render_frame();};
    while(snapshot().step<static_cast<std::int64_t>(required)){
        const auto incoming=snapshot();if(incoming.step>=static_cast<std::int64_t>(warmup)&&!loop.warmed)loop.warmed=std::pair{Clock::now(),incoming.time_s};
        const auto began=Clock::now();std::optional<runtime::ResearchTransition> transition;
        if(active.session)transition=active.session->advance();else (void)active.sim->runtime->advance(incoming.step+1,{},0.,0.,{});
        const double ns=std::chrono::duration<double,std::nano>(Clock::now()-began).count();if(incoming.step>=static_cast<std::int64_t>(warmup))timings.push_back(ns);
        const auto current=frame();loop.minimum_front=std::min(loop.minimum_front,current.presentation.front_load_n);
        if(current.presentation.rider_present)loop.gap=std::max({loop.gap,current.presentation.supports[1].gap_m,current.presentation.supports[2].gap_m});
        const auto status=research::wire_value(runtime::Wire{current.state.model_status});
        if(!flag(status,"model_valid")||false_value(optional(status,"numerically_valid"))){set(result,"reason",value::text("invalid_model_or_numerics"));break;}
        if(current.state.outcome){set(result,"reason",value::text(*current.state.outcome));break;}
        if(active.session&&active.session->done()){if(transition&&transition->reason)set(result,"reason",value::text(*transition->reason));break;}
    }
    return loop;
}
// Post-loop snapshot, timing-metric and publication temporaries also get
// their own frames; the final-state block likewise stays out of the handler.
void climbing_outcome(ClimbingSession& active,const ClimbingLoop& loop,std::size_t required,double dt,Clock::time_point loop_start,Value& result,const std::vector<double>& timings){
    const auto& warmed=loop.warmed;const auto final=active.session?active.session->environment().snapshot():active.sim->runtime->snapshot();
    set(result,"loop_wall_s",value::real(elapsed(loop_start)));set(result,"warmup_sim_s",warmed?value::real(warmed->second):Value{});
    set(result,"warmed_loop_real_time_factor",warmed?value::real((final.time_s-warmed->second)/std::max(elapsed(warmed->first),1e-12)):Value{});
    set(result,"timing",timings.empty()?Value{}:timing_metrics(timings,active.session?.01:dt));set(result,"complete",Value{final.step==static_cast<std::int64_t>(required)});
    set(result,"max_pedal_gap_m",value::real(loop.gap));set(result,"min_front_load_n",std::isfinite(loop.minimum_front)?value::real(loop.minimum_front):Value{});
    if(active.publication){const auto begin=Clock::now();const auto path=active.publication->finish();set(result,"recording_write_wall_s",value::real(elapsed(begin)));set(result,"recording",path?value::text(path->string()):Value{});}
}
void climbing_finalize(ClimbingSession& active,Value& result){
    const auto final=active.session?active.session->environment().snapshot():active.sim->runtime->snapshot();const auto presentation=active.session?active.session->environment().render_frame().presentation:active.sim->runtime->render_frame().presentation;
    set(result,"end_time_s",value::real(final.time_s));set(result,"end_position_m",value::real(presentation.position_m));set(result,"model_status",research::wire_value(runtime::Wire{final.model_status}));
    set(result,"final_state_sha256",value::text(io::sha256(std::as_bytes(std::span(final.integration_state)))));
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
Value climbing_repetition(std::string_view mode,double duration,const std::filesystem::path& physics,const std::filesystem::path& track,const std::filesystem::path& out,const std::string& source,std::int64_t repetition,const std::filesystem::path& root){
    Value result{Object{{"repetition",value::integer(repetition)},{"complete",Value{false}},{"error",Value{}}}};
    ClimbingSession active;const auto setup_start=Clock::now();std::vector<double> timings;
    try{active=climbing_setup(mode,duration,physics,track,out,repetition,result);const double dt=active.dt;const auto required=steps(duration,dt,1e-9);
        set(result,"setup_wall_s",value::real(elapsed(setup_start)));set(result,"timing_unit",value::text(active.session?"control_tick":"physics_step"));
        // physical_runtime.py:155-157 removes this path explicitly.
        if(mode=="preview")throw std::runtime_error("preview path removed");
        const auto warmup=std::min<std::size_t>(static_cast<std::size_t>(std::nearbyint(.5/dt)),required/2);const auto loop_start=Clock::now();
        const auto loop=climbing_loop(active,required,warmup,result,timings);climbing_outcome(active,loop,required,dt,loop_start,result,timings);
    }catch(const std::exception& e){set(result,"error",value::text(e.what()));if(!find(result,"setup_wall_s"))set(result,"setup_wall_s",value::real(elapsed(setup_start)));}
    if(active.session||active.sim)climbing_finalize(active,result);
    const bool unchanged=source_fingerprint(root/"native/src")==source;set(result,"source_unchanged_during_run",Value{unchanged});
    const auto status=optional(result,"model_status",empty_object());const auto rtf=optional(result,"warmed_loop_real_time_factor");
    const bool passed=flag(result,"complete")&&io::is_null(get(result,"error"))&&text(result,"reason")!="invalid_model_or_numerics"&&flag(status,"model_valid")&&
        !false_value(optional(status,"numerically_valid"))&&!io::is_null(rtf)&&io::as_double(rtf,"real time factor")>=1.&&unchanged;
    set(result,"performance_passed",Value{passed});write_report(out/std::format("run-{:04}.json",repetition),result);return result;
}
NATIVE_DIAG_POP
} // namespace

std::span<const std::string_view> benchmark_case_names(){return names;}
RigResult benchmark_case(std::string_view name,double dt){
    positive(dt,"timestep");using mechanics::bounds;const double inf=std::numeric_limits<double>::infinity();
    if(name=="wheel_inertia")return {wheel_inertia_rig(dt),bounds({{"acceleration_relative_error",0.,.001}})};
    if(name=="contact_event")return {contact_event_rig(dt),bounds({{"event_error_s",0.,2.*dt}})};
    if(name=="radial_native_reference"){Object results;for(double load:{100.,300.,600.,1000.}){results.emplace_back("deflection_at_"+std::to_string(static_cast<int>(load))+"n_m",get(radial_rig(load,dt,2.,true),"effective_deflection_m"));}return {Value{std::move(results)},empty_object()};}
    for(double load:{100.,300.,600.,1000.})if(name=="radial_"+std::to_string(static_cast<int>(load))){auto result=radial_rig(load,dt);
        set(result,"deflection_error_m",value::real(std::abs(real(result,"deflection_m")-real(result,"expected_deflection_m"))));
        return {result,bounds({{"deflection_error_m",0.,std::max(.0001,.02*real(result,"expected_deflection_m"))},{"speed_mps",-1e-5,1e-5}})};}
    if(name=="incline_30_deg"){auto result=incline_rig(dt);set(result,"normal_relative_error",value::real(relative_change(real(result,"normal_load_n"),real(result,"expected_normal_load_n"),1.)));
        set(result,"vertical_relative_error",value::real(relative_change(real(result,"vertical_with_stand_reaction_n"),real(result,"expected_weight_n"),1.)));
        return {std::move(result),bounds({{"normal_relative_error",0.,.005},{"vertical_relative_error",0.,.005}})};}
    if(name=="chain_locked_geometry"){auto result=chain_locked_rig(dt);set(result,"speed_ratio_error",value::real(relative_change(real(result,"speed_ratio"),real(result,"expected_ratio"),1.)));
        set(result,"torque_ratio_error",value::real(relative_change(real(result,"torque_ratio"),real(result,"expected_ratio"),1.)));
        return {std::move(result),bounds({{"speed_ratio_error",0.,.001},{"torque_ratio_error",0.,.001},{"power_relative_error",0.,.005}})};}
    if(name=="chain_suspension_motion"){auto result=chain_geometry_rig();merge(result,compiled_chain_motion_rig());return {std::move(result),bounds({
        {"rigid_motion_extension_m",-1e-12,1e-12},{"directional_relative_error",0.,1e-7},{"compiled_jacobian_relative_error",0.,1e-7},{"linkage_force_per_tension_m",1e-5,inf}})};}
    if(name=="suspension_cycle")return suspension_cycle(dt);
    if(name=="airborne_internal_actuation")return airborne_internal_actuation(dt);
    if(name=="distributed_flat_600")return distributed_flat_rig(dt,256,600.);
    if(name=="distributed_step")return distributed_step_rig(dt,256,.005);
    if(name=="distributed_incline")return distributed_incline_rig(dt,256);
    if(name=="rigid_load_transfer")return load_transfer_rig(dt,.1,.5,.9);
    if(name=="whole_system_passive")return momentum_rig(dt,false);
    if(name=="whole_system_active")return momentum_rig(dt,true);
    if(name=="motor_only_airborne")return momentum_rig(dt,true,"geometric_ideal_mid_drive",false);
    if(name=="actual_shaft_ratio")return shaft_ratio_rig(dt);
    if(name=="actual_freehub_overrun")return shaft_ratio_rig(dt,34./51.,true);
    for(const char* mode:{"ideal_mid_drive","geometric_ideal_mid_drive","elastic_chain"})for(double torque:{0.,20.,40.})
        if(name==std::string("drive_suspension_")+mode+"_"+std::to_string(static_cast<int>(torque)))return drive_suspension_rig(dt,mode,torque);
    if(std::ranges::find(names,name)!=names.end())return ride_case(name,dt);
    throw std::invalid_argument("unknown validation benchmark case: "+std::string(name));
}

Value benchmark_suite(const Value& config,const std::filesystem::path& root){
    const auto jobs=io::as_int64(optional(config,"jobs",value::integer(1)),"jobs");
    if(jobs<1||jobs>32)throw std::invalid_argument("jobs must be an integer between 1 and 32");
    // Do not silently turn the source's isolated multi-process request into
    // unsafe shared-engine threads or claim that it was executed in parallel.
    if(jobs!=1)throw std::invalid_argument("native benchmark worker isolation is not implemented; jobs must be 1");
    auto dts=numbers(optional(config,"dt_values_s",Value{Array{value::real(.0005),value::real(.00025),value::real(.000125)}}),"dt_values_s");
    if(dts.empty())throw std::invalid_argument("positive, finite, distinct timestep values required");
    for(double dt:dts)positive(dt,"timestep");std::ranges::sort(dts,std::greater<>{});
    if(std::ranges::adjacent_find(dts)!=dts.end())throw std::invalid_argument("positive, finite, distinct timestep values required");
    const auto chosen=strings(optional(config,"cases",value::strings(std::vector<std::string>(names.begin(),names.end()))),"cases");
    if(chosen.empty()||std::set<std::string>(chosen.begin(),chosen.end()).size()!=chosen.size())throw std::invalid_argument("unknown, duplicate or empty validation case selection");
    for(const auto& name:chosen)if(std::ranges::find(names,name)==names.end())throw std::invalid_argument("unknown, duplicate or empty validation case selection");
    const auto out=root/text(config,"output","results/validation");std::filesystem::create_directories(out);
    const auto source=source_fingerprint(root/"native/src");const auto environment=environment_contract(root);
    Value report{Object{{"schema_version",value::integer(1)},{"calibration_status",value::text("parameterized_unvalidated")},
        {"complete_suite",Value{chosen.size()==names.size()}},{"dt_values_s",value::doubles(dts)},{"versions",versions()},
        {"version_scope",value::text("native compiler, engine and numerical provider; Python package versions are not impersonated")},
        {"cases",Value{Array{}}},{"convergence",Value{Array{}}},{"passed",Value{false}},{"jobs",value::integer(jobs)},
        {"environment",environment},{"source_sha256",value::text(source)},
        {"required_case_count",value::integer(static_cast<std::int64_t>(chosen.size()*dts.size()))}}};
    Array rows,convergence;std::map<std::pair<std::string,double>,std::size_t> lookup;
    for(const auto& name:chosen)for(double dt:dts){lookup.emplace(std::pair{name,dt},rows.size());rows.push_back(execute(name,dt));set(report,"cases",Value{rows});write_report(out/"report.json",report);}
    if(dts.size()>=2)convergence=benchmark_convergence(dts,chosen,rows,lookup);
    bool passed=true;for(const auto& row:rows)if(flag(row,"mandatory"))passed=passed&&flag(row,"passed");for(const auto& row:convergence)passed=passed&&flag(row,"passed");
    const bool unchanged=source_fingerprint(root/"native/src")==source;
    const bool mechanics=passed&&flag(report,"complete_suite")&&dts==std::vector<double>{.0005,.00025,.000125}&&unchanged;
    set(report,"convergence",Value{std::move(convergence)});set(report,"passed",Value{passed});set(report,"source_unchanged_during_run",Value{unchanged});
    set(report,"mechanics_gate_passed",Value{mechanics});set(report,"release_gate_passed",Value{mechanics&&flag(environment,"locked_environment_verified")});
    write_report(out/"report.json",report);return report;
}

bool evidence_matches(const Value& record,const Value& expected){for(const char* key:{"source_sha256","configuration_sha256","versions"}){
    const auto* e=find(expected,key);const auto* r=find(record,key);if(!e||io::is_null(*e)||!r||io::canonical_json(*e)!=io::canonical_json(*r))return false;}return true;}
Value profile_identity(const std::filesystem::path& physics,const std::filesystem::path& root){
    const auto config=configuration::load_physics(physics);auto values=configuration::detail::plain_document(configuration::to_value(config));
    auto articulated=get(values,"articulated");if(const auto* envelope=find(articulated,"joint_envelope_path")){if(!io::is_null(*envelope))set(articulated,"joint_envelope_path",value::text(file_hash(io::as_string(*envelope,"joint envelope"))));}set(values,"articulated",std::move(articulated));
    return Value{Object{{"source_sha256",value::text(source_fingerprint(root/"native/src"))},{"configuration_sha256",value::text(dataset_hash(values))},
        {"configuration_scope",value::text("resolved profile inputs; individual fixtures declare their own parameters")},
        {"physics_profile",value::text(physics.string())},{"physics_profile_sha256",value::text(file_hash(physics))},
        {"versions",versions()},{"environment",environment_contract(root)}}};
}
Value mechanics_report(const Value& config,const std::filesystem::path& root){
    const auto profile=root/text(config,"physics_profile","examples/research/plant_reference_open_loop.toml");
    const auto out=root/text(config,"output","results/antiwheelie");std::filesystem::create_directories(out);
    Value report=profile_identity(profile,root);set(report,"schema_version",value::integer(1));set(report,"checks",Value{Array{}});set(report,"passed",Value{false});Array checks;
    for(const char* name:{"analytic","load_and_shaft","drive_suspension","contact","rider"}){
        checks.push_back(mechanics_check(name,out,profile,root));set(report,"checks",Value{checks});write_report(out/"mechanics.json",report);}
    const bool unchanged=text(report,"source_sha256")==source_fingerprint(root/"native/src");bool passed=checks.size()==5&&unchanged;
    for(const auto& check:checks)passed=passed&&flag(check,"passed");set(report,"source_unchanged_during_run",Value{unchanged});set(report,"passed",Value{passed});write_report(out/"mechanics.json",report);return report;
}

Value climbing_benchmark(const Value& config,const std::filesystem::path& root){
    const auto mode=text(config,"mode");if(mode!="preview"&&mode!="accounted"&&mode!="research")throw std::invalid_argument("benchmark mode must be preview, accounted or research");
    const double duration=positive(real(config,"duration_s",8.),"benchmark duration");const auto repeats=io::as_int64(optional(config,"repeats",value::integer(3)),"repetitions");
    if(repeats<1)throw std::invalid_argument("benchmark repetitions must be positive integers");
    const auto physics=root/identifier(get(config,"physics_profile"),"physics profile"),track=root/identifier(get(config,"track"),"track"),out=root/identifier(get(config,"output"),"output");
    if(std::filesystem::exists(out)&&(!std::filesystem::is_directory(out)||!std::filesystem::is_empty(out)))throw std::invalid_argument("benchmark output already exists; choose a new directory");
    std::filesystem::create_directories(out);const auto source=source_fingerprint(root/"native/src");struct utsname hardware{};const bool has_hardware=::uname(&hardware)==0;
    Value report{Object{{"schema_version",value::integer(1)},{"mode",value::text(mode)},{"duration_requested_s",value::real(duration)},
        {"physics_profile",value::text(physics.string())},{"track_path",value::text(track.string())},{"source_sha256",value::text(source)},
        {"hardware",Value{Object{{"machine",has_hardware?value::text(hardware.machine):Value{}},{"platform",has_hardware?value::text(std::string(hardware.sysname)+" "+hardware.release):Value{}},
            {"cpu_count",std::thread::hardware_concurrency()?value::integer(std::thread::hardware_concurrency()):Value{}}}}},
        {"runs",Value{Array{}}},{"passed",Value{false}}}};Array runs;
    for(std::int64_t repetition=0;repetition<repeats;++repetition){
        auto result=climbing_repetition(mode,duration,physics,track,out,source,repetition,root);
        runs.push_back(result);set(report,"runs",Value{runs});write_report(out/"report.json",report);
    }
    bool passed=!runs.empty();for(const auto& run:runs)passed=passed&&flag(run,"performance_passed");set(report,"passed",Value{passed});write_report(out/"report.json",report);return report;
}
} // namespace bike::validation
