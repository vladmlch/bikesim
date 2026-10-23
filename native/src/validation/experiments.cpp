#include "scenarios.hpp"
#include "../diag.hpp"
#include "mechanics.hpp"
#include "acceptance.hpp"
#include "evidence.hpp"
#include "../configuration/load.hpp"
#include "../research/environment.hpp"
#include "../research/evidence.hpp"
#include "../rider/state_codec.hpp"
#include "../terrain/trackfile.hpp"
#include "../numerics/density_fit.hpp"
#include <chrono>
#include <format>
#include <map>
#include <tuple>

namespace bike::validation {
namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point start){return std::chrono::duration<double>(Clock::now()-start).count();}
void set(Value& out,std::string key,Value item){value::set(std::get<Object>(out.data),std::move(key),std::move(item));}
void merge(Value& out,const Value& from){for(const auto& [key,item]:io::as_object(from,"report"))set(out,key,item);}
constexpr std::array<std::string_view,7> fidelity_cases{"static_sag","motor_ramp_grade","bump_coast","crest_coast","step_4cm","recontact","coast_resume"};
double case_duration(std::string_view name){if(name=="static_sag")return .25;if(name=="motor_ramp_grade")return 5.;if(name=="bump_coast"||name=="step_4cm")return 2.;if(name=="crest_coast")return 3.;if(name=="recontact")return 1.;if(name=="coast_resume")return 8.;throw std::invalid_argument("unknown fidelity case");}
terrain::TrackSpec case_track(std::string_view name) {
    if(std::ranges::find(fidelity_cases,name)==fidelity_cases.end())throw std::invalid_argument("unknown fidelity case");
    terrain::TrackSpec track{.name=std::string(name),.length_m=20.};
    if(name=="motor_ramp_grade")track.grade_profile=terrain::GradeProfile{{{0.,.05},{20.,.05}}};
    else if(name=="bump_coast")track.obstacles.push_back({terrain::Bump{4.,.02,.4}});
    else if(name=="crest_coast")track.obstacles.push_back({terrain::Bump{4.,.12,1.5}});
    else if(name=="step_4cm")track.obstacles.push_back({terrain::SquareEdge{4.,.04,1.}});
    return track;
}
// The -O0 frame budget keeps the override Value, resolved config and
// BuildRequest temporaries each in their own helper frame.
Value fidelity_override(std::string_view name,double dt,int n,std::string_view backend,std::string_view transmission){
    const bool moving=name=="bump_coast"||name=="crest_coast"||name=="step_4cm"||name=="recontact";
    Value tires{Object{
        {"backend",value::text(backend)},
        {"distributed",Value{Object{{"station_count",value::integer(n)}}}}
    }};
    return Value{Object{
        {"timestep_s",value::real(dt)},{"closure_time_constant_s",value::real(.0025)},
        {"initial_speed_mps",value::real(moving?2.:0.)},{"tires",std::move(tires)},
        {"drive",Value{Object{{"transmission_model",value::text(transmission)}}}},
        {"equilibrium_cache_enabled",Value{true}}
    }};
}
model::BuildRequest fidelity_model(const Value& override,const std::filesystem::path& profile){
    return mechanics::request(configuration::resolve_physics(override,configuration::load_physics(profile)),true);
}
std::unique_ptr<initialization::SetupRequest> fidelity_request(std::string_view name,double dt,double dx,int n,std::string_view backend,std::string_view transmission,const std::filesystem::path& profile,std::optional<initialization::PhysicalInitialState> initial){
    auto request=std::make_unique<initialization::SetupRequest>();
    request->model=fidelity_model(fidelity_override(name,dt,n,backend,transmission),profile);
    request->track=case_track(name);request->model.field=research::research_field(request->track,dx);request->asset_root=profile.parent_path();request->initial_state=std::move(initial);
    return request;
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::unique_ptr<Simulation> make_fidelity(std::string_view name,double dt,double dx,int n,std::string_view backend,std::string_view transmission,const std::filesystem::path& profile,std::optional<initialization::PhysicalInitialState> initial={}) {
    if(name=="motor_ramp_grade")throw std::invalid_argument(std::string(motor_only_unsupported));
    const auto request=fidelity_request(name,dt,dx,n,backend,transmission,profile,std::move(initial));
    const bool recontact=name=="recontact"&&!request->initial_state;auto sim=std::make_unique<Simulation>(std::move(*request));
    if(recontact){(void)RuntimeRigAccess::rebase_airborne(*sim->runtime,.12,-.5,false,false,false,true);auto eq=get(sim->metadata,"equilibrium");
        set(eq,"initial_condition_fixture",Value{Object{{"airborne_height_increment_m",value::real(.12)},{"vertical_velocity_mps",value::real(-.5)}}});set(sim->metadata,"equilibrium",std::move(eq));}
    return sim;
}
NATIVE_DIAG_POP
Value run_fidelity_case(Simulation& sim,std::string_view name,std::string_view shared_hash) {
    const double duration=case_duration(name),dt=sim.request.model.physics->timestep_s;const auto count=steps(duration,dt,1e-9);const auto initial=sim.initial_state();
    Value report{Object{{"case",value::text(name)},{"metadata",sim.metadata},{"equilibrium",get(sim.metadata,"equilibrium")},
        {"initial_state_sha256",value::text(shared_hash.empty()?initial.sha256():std::string(shared_hash))},{"serialized_initial_state_sha256",value::text(initial.sha256())},
        {"station_projection",sim.request.initial_state?optional(sim.request.initial_state->payload(),"station_projection"):Value{}},{"expected_duration_s",value::real(duration)},{"rows",Value{Array{}}}}};
    RuntimeRigAccess::inspect(*sim.runtime,[&](const Stepper& stepper,const auto&){set(report,"initial_qpos",value::doubles(stepper.qpos()));set(report,"initial_qvel",value::doubles(stepper.qvel()));});
    const auto geometry=RuntimeRigAccess::truth_geometry(*sim.runtime);runtime::WheelieTracker tracker;Array rows;const auto start=Clock::now();std::string termination="duration_completed";bool stop=false;
    const auto drain=[&]{const auto batch=sim.runtime->prepare_samples();for(const auto& sample:batch.samples){auto row=interval_row(*sample,geometry,tracker);
        if(!flag(get(row,"model_status"),"numerically_valid")){termination="numerical_quality";stop=true;}
        else if(!flag(get(row,"model_status"),"model_valid")){termination="model_invalid";stop=true;}rows.push_back(std::move(row));}sim.runtime->acknowledge_samples(batch);};
    try{for(std::size_t i=0;i<count&&!stop;++i){const auto current=sim.runtime->snapshot();auto control=name=="coast_resume"?resume_schedule(current.time_s):runtime::RideControl{.motor_torque_nm=0.,.human_torque_nm=0.};
        const double brake=name=="static_sag"?1.:0.;const auto result=sim.runtime->advance(current.step+1,control,brake,brake,{});drain();if(result.outcome){termination="crash";stop=true;}}sim.runtime->flush();drain();}
    catch(const std::exception& e){termination="numerical_failure";set(report,"error",value::text(e.what()));try{sim.runtime->flush();drain();}catch(const std::exception& cleanup){set(report,"cleanup_error",value::text(cleanup.what()));}}
    set(report,"termination",value::text(termination));set(report,"wall_time_s",value::real(elapsed(start)));set(report,"rows",Value{rows});
    if(!rows.empty()){auto m=metrics_from_rows(Value{rows},duration,text(report,"initial_state_sha256"));if(termination=="numerical_failure")set(m,"numerically_valid",Value{false});set(report,"metrics",std::move(m));}else set(report,"metrics",Value{});
    set(report,"wheelie_observer",research::wire_value(runtime::Wire{tracker.metrics()}));return report;
}
Value axes_summary(const Array& comparisons,bool require_nonempty=true) {
    Object result;for(const char* axis:{"time","road","stations"}){bool exists=false,passed=true;for(const auto& c:comparisons)if(flag(c,"finest_pair")&&text(c,"axis")==axis){exists=true;passed=passed&&flag(c,"passed");}
        result.emplace_back(axis,Value{passed&&(!require_nonempty||exists)});}return Value{std::move(result)};
}
// Per-case series/comparison production extracted so fidelity_sweep's frame
// stays under the -O0 -Wframe-larger-than budget.
void fidelity_case(const std::string& name,const std::vector<double>& anchor,const Value& resolution,const Value& times,const Value& roads,const Value& stations,bool include,std::string_view backend,std::string_view transmission,const std::filesystem::path& profile,const std::filesystem::path& out,Value& report,Array& series,Array& comparisons){
    std::optional<initialization::PhysicalInitialState> initial;std::map<int,initialization::PhysicalInitialState> projected;
    using Key=std::tuple<double,double,int>;std::map<Key,Value> lookup;
    try{auto sim=make_fidelity(name,anchor[0],anchor[1],static_cast<int>(anchor[2]),backend,transmission,profile);initial=sim->initial_state();write_report_text(out/(name+"_initial_state.json"),initial->payload_json());
        if(include)for(const auto& count:io::as_array(stations,"stations")){const auto n=io::as_int64(count,"stations");if(n!=static_cast<int>(anchor[2]))projected.emplace(static_cast<int>(n),sim->initial_state(static_cast<int>(n)));}}
    catch(const std::exception& e){auto errors=optional(report,"initialization_errors",empty_object());set(errors,name,value::text(e.what()));set(report,"initialization_errors",std::move(errors));}
    for(const auto& variant:io::as_array(get(resolution,"variants"),"variants")){const auto& a=io::as_array(variant,"variant");const auto axis=io::as_string(a[0],"axis");
        const double dt=io::as_double(a[1],"dt"),dx=io::as_double(a[2],"dx");const int n=static_cast<int>(io::as_int64(a[3],"station count"));const Key key{dt,dx,n};if(lookup.contains(key))continue;
        const auto file=std::format("{}_dt{:.7f}_dx{:g}_n{}.json.gz",name,dt,dx,n);
        Value item{Object{{"case",value::text(name)},{"dt_s",value::real(dt)},{"dx_m",value::real(dx)},{"station_count",value::integer(n)},{"axis",value::text(axis)},{"file",value::text(file)}}};const auto start=Clock::now();
        try{if(!initial)throw std::invalid_argument("shared candidate initialization failed");auto seed=n==static_cast<int>(anchor[2])?*initial:projected.at(n);
            auto sim=make_fidelity(name,dt,dx,n,backend,transmission,profile,std::move(seed));auto result=run_fidelity_case(*sim,name,initial->sha256());write_report(out/file,result);
            for(const char* field:{"metrics","termination","metadata","equilibrium"})set(item,field,get(result,field));set(item,"runtime_s",get(result,"wall_time_s"));}
        catch(const std::exception& e){set(item,"metrics",Value{});set(item,"termination",value::text("initialization_failure"));set(item,"error",value::text(e.what()));}
        set(item,"total_wall_time_s",value::real(elapsed(start)));lookup.emplace(key,item);series.push_back(std::move(item));set(report,"series",Value{series});write_report(out/"report.json",report);}
    std::vector<std::pair<std::string,std::vector<Key>>> groups;std::vector<Key> keys;
    for(double dt:numbers(times,"times"))keys.emplace_back(dt,anchor[1],static_cast<int>(anchor[2]));groups.emplace_back("time",std::move(keys));keys.clear();
    for(double dx:numbers(roads,"roads"))keys.emplace_back(anchor[0],dx,static_cast<int>(anchor[2]));groups.emplace_back("road",std::move(keys));
    if(include){keys.clear();for(const auto& n:io::as_array(stations,"stations"))keys.emplace_back(anchor[0],anchor[1],static_cast<int>(io::as_int64(n,"n")));groups.emplace_back("stations",std::move(keys));}
    for(const auto& [axis,entries]:groups)for(std::size_t i=0;i<entries.size();++i)for(std::size_t j=i+1;j<entries.size();++j){const auto& a=lookup.at(entries[i]);const auto& b=lookup.at(entries[j]);
        Value comparison{Object{{"case",value::text(name)},{"axis",value::text(axis)},{"coarse",get(a,"file")},{"reference",get(b,"file")},{"finest_pair",Value{i==entries.size()-2&&j==entries.size()-1}}}};
        try{if(io::is_null(get(a,"metrics"))||io::is_null(get(b,"metrics")))throw std::invalid_argument("missing/aborted series");merge(comparison,compare_metrics(get(a,"metrics"),get(b,"metrics")));}
        catch(const std::exception& e){set(comparison,"passed",Value{false});set(comparison,"failed_criteria",value::strings(std::array<std::string,1>{"incomplete_evidence"}));set(comparison,"error",value::text(e.what()));}comparisons.push_back(std::move(comparison));}
}
} // namespace

Value fidelity_sweep(const Value& config,const std::filesystem::path& root) {
    const auto jobs=io::as_int64(optional(config,"jobs",value::integer(1)),"jobs");
    if(jobs<1||jobs>16)throw std::invalid_argument("jobs must be 1..16");
    if(jobs!=1)throw std::invalid_argument("native fidelity worker isolation is not implemented; jobs must be 1");
    const auto selected=strings(optional(config,"cases",value::strings(std::vector<std::string>(fidelity_cases.begin(),fidelity_cases.end()))),"cases");
    if(selected.empty()||std::set<std::string>(selected.begin(),selected.end()).size()!=selected.size())throw std::invalid_argument("distinct known fidelity cases required");
    for(const auto& c:selected)if(std::ranges::find(fidelity_cases,c)==fidelity_cases.end())throw std::invalid_argument("distinct known fidelity cases required");
    const auto times=optional(config,"time_steps",Value{Array{value::real(.00125),value::real(.000625),value::real(.0003125)}});
    const auto roads=optional(config,"road_steps",Value{Array{value::real(.01),value::real(.005),value::real(.0025)}});
    const auto stations=optional(config,"station_counts",Value{Array{value::integer(128),value::integer(256),value::integer(512)}});
    const bool include=flag(config,"include_stations");const auto resolution=resolution_variants(times,roads,stations,include);const auto anchor=numbers(get(resolution,"anchor"),"anchor");
    const auto backend=text(config,"backend","compliant_2d"),transmission=text(config,"transmission","ideal_mid_drive");
    if(include&&backend!="distributed_2d_reference")throw std::invalid_argument("station sweep requires distributed backend");
    const auto profile=root/text(config,"physics_profile","examples/research/plant_reference_open_loop.toml"),out=root/text(config,"output","results/fidelity");std::filesystem::create_directories(out);
    Value report{Object{{"schema_version",value::integer(1)},{"backend",value::text(backend)},{"transmission",value::text(transmission)},{"physics_profile",value::text(profile.string())},
        {"closure_time_constant_s",value::real(.0025)},{"environment",environment_contract(root)},{"source_sha256",value::text(source_fingerprint(root/"native/src"))},
        {"series",Value{Array{}}},{"comparisons",Value{Array{}}},{"passed",Value{false}},
        {"candidate_resolution",Value{Object{{"dt_s",value::real(anchor[0])},{"dx_m",value::real(anchor[1])},{"station_count",value::integer(static_cast<int>(anchor[2]))}}}},
        {"time_steps",times},{"road_steps",roads},{"station_counts",stations},{"station_initialization",value::text("same qpos/qvel; explicit periodic angular material-field projection, target residual recorded")},{"jobs",value::integer(1)}}};
    Array series,comparisons;
    for(const auto& name:selected)fidelity_case(name,anchor,resolution,times,roads,stations,include,backend,transmission,profile,out,report,series,comparisons);
    bool any=false,passed=true;std::set<std::string> failures;std::map<std::string,double> maxima;
    for(const auto& c:comparisons){if(flag(c,"finest_pair")){any=true;passed=passed&&flag(c,"passed");for(const auto& key:strings(optional(c,"failed_criteria",Value{Array{}}),"failed criteria"))failures.insert(key);}
        for(const auto& [key,v]:io::as_object(optional(c,"criteria",empty_object()),"criteria"))if(const auto* actual=find(v,"actual"))if(std::holds_alternative<io::Number>(actual->data)){const double x=io::as_double(*actual,"actual");auto it=maxima.find(key);if(it==maxima.end())maxima.emplace(key,x);else it->second=std::max(it->second,x);}}
    const bool unchanged=text(report,"source_sha256")==source_fingerprint(root/"native/src");set(report,"comparisons",Value{std::move(comparisons)});
    set(report,"axes_passed",axes_summary(io::as_array(get(report,"comparisons"),"comparisons")));set(report,"passed",Value{any&&passed&&unchanged});set(report,"source_unchanged_during_run",Value{unchanged});
    set(report,"failed_criteria",value::strings(std::vector<std::string>(failures.begin(),failures.end())));Object errors;for(const auto& [key,x]:maxima)errors.emplace_back(key,value::real(x));set(report,"maximum_errors",Value{std::move(errors)});
    write_report(out/"report.json",report);return report;
}
namespace {
terrain::TrackSpec scenario_track(const Value& item,const std::filesystem::path& root) {
    const auto name=text(item,"name");if(name=="extreme_100m")return terrain::load_track(root/"examples/research/rough_uphill_extreme.toml");
    terrain::TrackSpec track{.name=name,.length_m=20.};
    if(find(item,"grade"))track.grade_profile=terrain::GradeProfile{{{0.,real(item,"grade")},{20.,real(item,"grade")}}};
    else if(name=="bump")track.obstacles={{terrain::Bump{4.,.02,.4}}};
    else if(name=="crest")track.obstacles={{terrain::Bump{4.,.12,1.5}}};
    else if(name.starts_with("step_"))track.obstacles={{terrain::SquareEdge{4.,name=="step_4cm"?.04:.06,1.}}};
    else if(name=="two_support")track.obstacles={{terrain::Bump{4.,.03,.05}},{terrain::Bump{4.26,.03,.05}}};return track;
}
runtime::RideControl scenario_control(const Value& item,double t) {
    const auto name=text(item,"name");if(name=="coast_resume")return resume_schedule(t);
    double torque=t<1.?0.:40.*std::min(1.,t-1.);if(name=="bump"||name=="crest"||name=="step_4cm"||name=="step_6cm"||name=="two_support"||name=="recontact")torque=0.;
    runtime::RideControl control{.motor_torque_nm=torque,.human_torque_nm=0.};
    if(const auto* requested=find(item,"posture")){const auto p=io::as_string(*requested,"posture");::rider::RiderPosture posture;
        if(p=="standing"){posture.torso_lean_rad=.08;posture.pelvis_offset_m=spindle::Vec2{0.,.16};posture.use_saddle=false;}
        else if(p=="forward"){posture.torso_lean_rad=.2;posture.pelvis_offset_m=spindle::Vec2{.08,.02};}
        else if(p=="rearward"){posture.torso_lean_rad=-.1;posture.pelvis_offset_m=spindle::Vec2{-.08,.02};}control.posture=posture;}return control;
}
// The -O0 frame budget keeps the resolve/request temporaries inside the
// model builder and one scenario execution per helper frame.
model::BuildRequest resolve_model(const Value& overrides,const configuration::SimulationPhysicsConfig& base){
    return mechanics::request(configuration::resolve_physics(overrides,base),true);
}
Value reference_overrides(const Value& item,const Value& config){
    const auto name=text(item,"name");const bool moving=name=="bump"||name=="crest"||name=="step_4cm"||name=="step_6cm"||name=="two_support"||name=="recontact";const double speed=moving?2.:0.;
    Value drive{Object{{"human_torque_nm",value::real(0.)},{"shifting",Value{Object{{"enabled",Value{false}}}}},{"pedaling",Value{Object{{"rollback_brake",Value{false}}}}}}};
    Value overrides{Object{{"initial_speed_mps",value::real(speed)},{"drive",std::move(drive)}}};if(const auto* dt=find(config,"dt_s"))set(overrides,"timestep_s",*dt);return overrides;
}
std::unique_ptr<initialization::SetupRequest> reference_request(const Value& item,const configuration::SimulationPhysicsConfig& base,const Value& config,const std::filesystem::path& root,const std::map<std::pair<double,double>,initialization::PhysicalInitialState>& seeds,const std::pair<double,double>& key){
    auto request=std::make_unique<initialization::SetupRequest>();
    request->model=resolve_model(reference_overrides(item,config),base);request->track=scenario_track(item,root);request->model.field=research::research_field(request->track,real(config,"dx_m",.005));
    if(auto found=seeds.find(key);found!=seeds.end())request->initial_state=found->second;
    return request;
}
void reference_run(Simulation& sim,const Value& item,Array& rows,Value& record){
    const auto geometry=RuntimeRigAccess::truth_geometry(*sim.runtime);runtime::WheelieTracker tracker;const double duration=real(item,"duration_s"),dt=sim.request.model.physics->timestep_s;
    const auto count=steps(duration,dt,1e-9),check_every=std::max<std::size_t>(1,static_cast<std::size_t>(std::nearbyint(.1/dt)));
    std::optional<double> goal;if(const auto* g=find(item,"goal_x_m"))if(!io::is_null(*g))goal=io::as_double(*g,"goal");Value outcome;std::optional<std::string> error;bool stop=false;
    auto drain=[&]{const auto batch=sim.runtime->prepare_samples();for(const auto& s:batch.samples)rows.push_back(interval_row(*s,geometry,tracker));sim.runtime->acknowledge_samples(batch);};
    try{for(std::size_t i=0;i<count&&!stop;++i){const auto state=sim.runtime->snapshot();const auto result=sim.runtime->advance(state.step+1,scenario_control(item,state.time_s),0.,0.,{});drain();
        if(!rows.empty()&&(result.outcome||!flag(get(rows.back(),"model_status"),"model_valid")||!flag(get(rows.back(),"model_status"),"numerically_valid")||i%check_every==0)){
            outcome=classify_outcome(Value{rows},goal,result.outcome.has_value());stop=text(outcome,"outcome")!="duration_limit";}}sim.runtime->flush();drain();}
    catch(const std::exception& e){error=e.what();try{sim.runtime->flush();drain();}catch(const std::exception& cleanup){set(record,"cleanup_error",value::text(cleanup.what()));}}
    if(error||io::is_null(outcome)||text(outcome,"outcome")=="duration_limit")outcome=classify_outcome(Value{rows},goal,sim.runtime->snapshot().outcome.has_value(),error,true);
    set(record,"outcome",std::move(outcome));if(!rows.empty())set(record,"metrics",metrics_from_rows(Value{rows},duration,text(record,"initial_state_sha256")));
    set(record,"observer",research::wire_value(runtime::Wire{tracker.metrics()}));
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
Value reference_scenario(const Value& item,const configuration::SimulationPhysicsConfig& base,const Value& config,const std::filesystem::path& root,const std::filesystem::path& out,std::map<std::pair<double,double>,initialization::PhysicalInitialState>& seeds){
    const auto name=text(item,"name");Value record{Object{{"scenario",item},{"rows",Value{Array{}}}}};Array rows;const auto start=Clock::now();
    try{const bool moving=name=="bump"||name=="crest"||name=="step_4cm"||name=="step_6cm"||name=="two_support"||name=="recontact";const auto key=std::pair{moving?2.:0.,real(item,"grade",0.)};
        const auto request=reference_request(item,base,config,root,seeds,key);
        const auto sim=std::make_unique<Simulation>(std::move(*request));if(!seeds.contains(key))seeds.emplace(key,sim->initial_state());
        if(name=="recontact"){(void)RuntimeRigAccess::rebase_airborne(*sim->runtime,.12,-.5,false,false,false,true);auto eq=get(sim->metadata,"equilibrium");set(eq,"airborne_initial_offset_m",value::real(.12));set(sim->metadata,"equilibrium",std::move(eq));}
        set(record,"metadata",sim->metadata);set(record,"equilibrium",get(sim->metadata,"equilibrium"));set(record,"initial_state_sha256",value::text(sim->initial_state().sha256()));
        RuntimeRigAccess::inspect(*sim->runtime,[&](const Stepper& stepper,const auto&){set(record,"initial_qpos",value::doubles(stepper.qpos()));set(record,"initial_qvel",value::doubles(stepper.qvel()));});
        reference_run(*sim,item,rows,record);}
    catch(const std::exception& e){set(record,"outcome",Value{Object{{"outcome",value::text("numerical_failure")},{"reason",value::text(std::string("initialization: ")+e.what())},
        {"time_s",value::real(0.)},{"position_m",Value{}},{"control",::rider::encode_intent_control(scenario_control(item,0.))}}});}
    set(record,"rows",Value{rows});set(record,"wall_time_s",value::real(elapsed(start)));const std::string file=name+".json.gz";write_report(out/file,record);
    std::erase_if(std::get<Object>(record.data),[](const auto& entry){return entry.first=="rows";});set(record,"file",value::text(file));set(record,"interval_count",value::integer(static_cast<std::int64_t>(rows.size())));
    return record;
}
NATIVE_DIAG_POP
Array contact_fits(const configuration::HingeDensity& density){
    const auto depths=geometry::detail::np_linspace(.001,.012,12);std::vector<double> linear;for(double x:depths)linear.push_back(130000.*x);Array fits;
    const std::array<std::pair<std::string,std::vector<double>>,2> targets{{{"synthetic-original-linear-radial-130000",linear},{"synthetic-density-identifiability",numerics::flat_load(depths,.37,density,1024).force}}};
    for(const auto& [name,loads]:targets){const auto fit=numerics::fit_density(depths,loads,.37,name);
        fits.push_back(Value{Object{{"dataset_id",value::text(fit.dataset_id)},{"calibration_status",value::text(fit.calibration_status)},{"radius_m",value::real(fit.radius_m)},
            {"maximum_relative_error",value::real(fit.maximum_relative_error)},{"rms_relative_error",value::real(fit.rms_relative_error)},{"error_gate",value::real(fit.error_gate)},
            {"station_count",value::integer(fit.station_count)},{"accepted",Value{fit.accepted}},{"target_load_n",value::doubles(fit.target_load_n)},
            {"predicted_load_n",value::doubles(fit.predicted_load_n)},{"deflection_m",value::doubles(fit.deflection_m)},
            {"density",Value{Object{{"knots_m",value::doubles(fit.density.knots_m)},{"stiffness_n_m",value::doubles(fit.density.stiffness_n_m)},
                {"damping_ns_m",value::real(fit.density.damping_ns_m)},{"provenance",value::text(fit.density.provenance)}}}}}});}
    return fits;
}
Array contact_stand_checks(){
    Array stands;
    for(int n:{128,256,512})for(double load:{100.,300.,600.,1000.}){const auto result=distributed_flat_rig(.0003125,n,load);
        stands.push_back(Value{Object{{"case",value::text("flat_load")},{"n",value::integer(n)},{"load_n",value::real(load)},{"metrics",result.metrics},{"passed",Value{checks_passed(check_bounds(result.metrics,result.bounds))}}}});}
    for(double angle:{5.,15.,30.}){const auto result=distributed_incline_rig(.0003125,256,angle);
        stands.push_back(Value{Object{{"case",value::text("incline_no_slip_fixture")},{"angle_deg",value::real(angle)},{"metrics",result.metrics},{"passed",Value{checks_passed(check_bounds(result.metrics,result.bounds))}}}});}
    return stands;
}
void contact_series(const std::filesystem::path& out,Value& report,Array& series,Array& comparisons){
    using Key=std::tuple<double,int,double>;
    const std::array<std::pair<std::string,std::array<Key,3>>,3> variants{{
        {"time",{Key{.00125,256,.005},Key{.000625,256,.005},Key{.0003125,256,.005}}},
        {"road",{Key{.0003125,256,.01},Key{.0003125,256,.005},Key{.0003125,256,.0025}}},
        {"stations",{Key{.0003125,128,.005},Key{.0003125,256,.005},Key{.0003125,512,.005}}}}};
    for(const char* name:{"step","two_support","edge_unload","recontact","bump","incline"}){std::map<Key,Value> lookup;
        for(const auto& [axis,keys]:variants){for(const auto& key:keys){if(lookup.contains(key))continue;const auto [dt,n,dx]=key;
            const auto file=std::format("{}_dt{:.7f}_dx{:g}_n{}.json.gz",name,dt,dx,n);
            Value item{Object{{"case",value::text(name)},{"dt_s",value::real(dt)},{"station_count",value::integer(n)},{"dx_m",value::real(dx)},{"file",value::text(file)}}};
            try{auto [rows,m]=distributed_road_trace(dt,n,dx,name);set(m,"contact_events",persistent_events(rows,.005,true));set(item,"metrics",std::move(m));set(item,"termination",value::text("duration_completed"));
                write_report(out/file,Value{Object{{"configuration",item},{"rows",std::move(rows)}}});}
            catch(const std::exception& e){set(item,"metrics",Value{});set(item,"termination",value::text("numerical_failure"));set(item,"error",value::text(e.what()));}
            lookup.emplace(key,item);series.push_back(std::move(item));set(report,"series",Value{series});write_report(out/"report.json",report);}
            for(auto [i,j]:std::array<std::pair<std::size_t,std::size_t>,3>{{{0,1},{1,2},{0,2}}}){const auto& a=lookup.at(keys[i]);const auto& b=lookup.at(keys[j]);
                Value comparison{Object{{"case",value::text(name)},{"axis",value::text(axis)},{"coarse",get(a,"file")},{"reference",get(b,"file")},{"finest_pair",Value{i==1&&j==2}}}};
                if(!io::is_null(get(a,"metrics"))&&!io::is_null(get(b,"metrics")))merge(comparison,contact_comparison(get(a,"metrics"),get(b,"metrics")));
                else{set(comparison,"passed",Value{false});set(comparison,"reason",value::text("failed series"));}comparisons.push_back(std::move(comparison));}}}
}
}
Value reference_matrix(const Value& config,const std::filesystem::path& root) {
    const auto profile=root/text(config,"physics_profile","examples/research/plant_reference_open_loop.toml"),out=root/text(config,"output","results/reference-matrix");
    const auto base=configuration::load_physics(profile);std::filesystem::create_directories(out);const auto all=scenario_definitions();Array items=io::as_array(all,"scenarios");
    if(const auto* choice=find(config,"names")){const auto names=strings(*choice,"names");std::set<std::string> known;for(const auto& item:items)known.insert(text(item,"name"));
        if(names.empty())throw std::invalid_argument("unknown/empty reference scenario selection");for(const auto& name:names)if(!known.contains(name))throw std::invalid_argument("unknown/empty reference scenario selection");
        std::erase_if(items,[&](const auto& item){return std::ranges::find(names,text(item,"name"))==names.end();});}
    Value summary{Object{{"schema_version",value::integer(1)},{"source_sha256",value::text(source_fingerprint(root/"native/src"))},{"environment",environment_contract(root)},
        {"physics_profile",value::text(profile.string())},{"scenarios",Value{Array{}}},{"release_status",value::text("parameterized_unvalidated")}}};
    Array records;std::map<std::pair<double,double>,initialization::PhysicalInitialState> seeds;
    for(const auto& item:items){
        auto record=reference_scenario(item,base,config,root,out,seeds);
        records.push_back(std::move(record));set(summary,"scenarios",Value{records});write_report(out/"matrix.json",summary);}
    return summary;
}

Value contact_resolution(const Value& config,const std::filesystem::path& root) {
    const auto out=root/text(config,"output","results/contact_resolution");std::filesystem::create_directories(out);const configuration::HingeDensity density;
    Value material{Object{{"knots_m",value::doubles(density.knots_m)},{"stiffness_n_m",value::doubles(density.stiffness_n_m)},{"damping_ns_m",value::real(density.damping_ns_m)},{"provenance",value::text(density.provenance)}}};
    Value report{Object{{"schema_version",value::integer(1)},{"source_sha256",value::text(source_fingerprint(root/"native/src"))},{"environment",environment_contract(root)},
        {"material",material},{"synthetic",Value{true}},{"series",Value{Array{}}},{"stand_checks",Value{Array{}}},{"comparisons",Value{Array{}}},{"passed",Value{false}}}};
    set(report,"fits",Value{contact_fits(density)});set(report,"material_selection",value::text("unchanged declared synthetic density; no automatic promotion from fit"));Array series,comparisons;
    const Array stands=contact_stand_checks();
    set(report,"stand_checks",Value{stands});contact_series(out,report,series,comparisons);
    const auto axes=axes_summary(comparisons);bool passed=checks_passed(axes);for(const auto& stand:stands)passed=passed&&flag(stand,"passed");
    set(report,"comparisons",Value{std::move(comparisons)});set(report,"axes_passed",axes);set(report,"passed",Value{passed});
    set(report,"status",value::text(passed?"numerically_verified_synthetic_stands":"experimental_gate_failed"));write_report(out/"report.json",report);return report;
}
} // namespace bike::validation
