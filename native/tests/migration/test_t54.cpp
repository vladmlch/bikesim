// Phase 3 Tasks 3.37-3.40. These source regressions were authored without
// running a compiler, test, or oracle capture. Existing fixtures stay immutable.
#include "support/migration_test.hpp"
#include "validation/acceptance.hpp"
#include "validation/cli_options.hpp"
#include "validation/evidence.hpp"
#include "validation/scenarios.hpp"
#include "validation/tire.hpp"
#include "configuration/load.hpp"
#include "runtime/value_readers.hpp"
#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <set>

namespace {
using namespace bike::validation;
namespace io=bike::io;
Value json(std::string_view input){return io::parse_json(input);}
void set(Value& target,std::string name,Value item){value::set(std::get<Object>(target.data),std::move(name),std::move(item));}
// NOLINTNEXTLINE(misc-no-recursion) recursive JSON value tree walk
void exact_value(const Value& actual,const Value& expected,const std::string& path){
    bt::require(actual.data.index()==expected.data.index(),path+": value kind");
    if(const auto* n=std::get_if<io::Number>(&expected.data)){
        const auto bits=std::bit_cast<std::uint64_t>(io::number_double(*n,path));
        bt::require(std::bit_cast<std::uint64_t>(io::as_double(actual,path))==bits,path+": binary64 bits");return;
    }
    if(const auto* entries=std::get_if<Object>(&expected.data)){
        bt::require(keys(actual)==keys(expected),path+": exact field inventory");
        for(const auto& [key,item]:*entries){std::string next(path);next+='.';next+=key;exact_value(get(actual,key),item,next);}return;
    }
    if(const auto* entries=std::get_if<Array>(&expected.data)){
        const auto& found=io::as_array(actual,path);bt::require(found.size()==entries->size(),path+": exact shape");
        for(std::size_t i=0;i<entries->size();++i)exact_value(found[i],(*entries)[i],path+"["+std::to_string(i)+"]");return;
    }
    bt::require(io::canonical_json(actual)==io::canonical_json(expected),path+": exact scalar");
}
void captured_tire_characterization(){
    bt::manifest_verify_files(bt::manifest_entry("validation/tire"));
    auto config=bike::configuration::detail::parse_toml_file(bt::fixture("validation/tire/config.toml"));
    // capture_reference.py:2572-2576 obtains the tire parameters from this
    // additional fixture. The short TOML grid alone omits mu/stiffness/length;
    // make that source dependency explicit instead of guessing its values.
    set(config,"physics_profile",value::text(bt::fixture("config/viewer_physics_welded.toml").string()));
    const auto result=run_case("tire-characterization",config);
    bt::require(result.accepted,"isolated material input/finite-result contract accepted");
    exact_value(result.evidence,io::parse_json(bt::text(bt::fixture("validation/tire/verdict.json"))),"tire-characterization");
    auto bad=config;set(bad,"duration_s",value::real(.5001));bt::invalid([&]{(void)run_case("tire-characterization",bad);});
    bad=std::move(config);set(bad,"loads_n",Value{Array{}});bt::invalid([&]{(void)run_case("tire-characterization",bad);});
}
void material_curve_contract(){
    auto record=json(R"({"dataset_id":"synthetic-radial","source_kind":"synthetic","tyre_label":"bench",
        "radius_m":0.37,"inflated_width_m":0.06,"rim_internal_width_m":0.03,"pressure_pa_gauge":150000,
        "radial_c_ns_m":100,"deflection_m":[0,0.01,0.02],"force_n":[0,1000,1000],
        "valid_load_range_n":[0,1000],"provenance":"synthetic monotone test curve"})");
    const auto calibration=calibration_from_record(record);const auto curve=calibration.curve();
    const auto [force,energy]=curve.elastic_response(.005);
    bt::require(force==500.&&energy==1.25,"linear segment integrates force into elastic joules");
    const auto [normal,stored]=curve.normal_contact(.005,-10.,100.);
    bt::require(normal==0.&&stored==energy,"unilateral damping never creates a tensile normal force or erases stored energy");
    bt::require(calibration.is_load_in_valid_range(1000.)&&!calibration.is_load_in_valid_range(1000.01),"declared calibration load range is inclusive");
    bt::invalid([&]{(void)curve.elastic_response(.0201);});
    set(record,"source_kind",value::text("validated"));bt::invalid([&]{(void)calibration_from_record(record);});
}
Value dataset(){return json(R"({"dataset_id":"rig-1","source":"synthetic:fixture","synthetic":true,
    "measured_at":"2026-09-30","units":{"force":"N","deflection":"m"},"bike_config_hash":"configuration-a",
    "sensor_uncertainty":{"force":1.0,"deflection":0.0001},"conditions":{"pressure_pa_gauge":150000.0},
    "samples":[{"experiment_id":"fit-a","force":100.0,"deflection":0.001},
               {"experiment_id":"hold-b","force":200.0,"deflection":0.002}],
    "split":{"fit":["fit-a"],"holdout":["hold-b"]}})");}
void calibration_evidence_contract(){
    const auto data=dataset();bt::invalid([&]{(void)validate_dataset(data);});
    const auto accepted=validate_dataset(data,true);bt::require(is_synthetic(accepted),"synthetic data retains its declaration");
    const auto report=calibration_report(data,json(R"({"fit-a":[100.0],"hold-b":[200.0]})"),"force",
        json(R"({"stiffness":100000.0})"),json(R"({"stiffness":[1000.0,200000.0]})"),
        json(R"({"budget_id":"registered-budget","rmse":1.0,"max_abs":1.0,"bias_abs":1.0})"),true);
    bt::require(flag(report,"holdout_passed")&&text(report,"calibration_status")=="parameterized_unvalidated",
        "a perfect synthetic holdout does not become measured calibration");
    bt::require(accepted_calibration_status(false,true,true,true)=="validated_within_declared_scope"&&
        accepted_calibration_status(true,true,true,true)=="parameterized_unvalidated","synthetic evidence cannot cross the measured-release gate");
    bt::invalid([&]{validate_split(json(R"(["same"] )"),json(R"(["same"] )"));});
    bt::invalid([&]{validate_split(json(R"([" trailing "] )"),json(R"(["holdout"] )"));});
    auto bad=data;set(bad,"measured_at",value::text("2026-02-30"));bt::invalid([&]{(void)validate_dataset(bad,true);});
    bad=data;set(bad,"sensor_uncertainty",json(R"({"force":1.0})"));bt::invalid([&]{(void)validate_dataset(bad,true);});
    bt::require(dataset_hash(json("{\"label\":\"тест\",\"value\":1.0}"))==hash_text(R"({"label":"\u0442\u0435\u0441\u0442","value":1.0})"),
        "dataset hashing preserves Python ensure_ascii and decoded float spelling");
    const std::array measured{1.,2.,3.},predicted{1.5,2.,2.5};const auto errors=holdout_metrics(measured,predicted);
    bt::require(real(errors,"bias")==0.&&real(errors,"max_abs")==.5&&real(errors,"rmse")==std::sqrt(1./6.),"holdout errors retain source reductions");
    bt::invalid([&]{(void)holdout_metrics(std::array{1.},std::array{std::numeric_limits<double>::infinity()});});
}
runtime::PhysicalSampleData sample(double start,bool valid){
    runtime::PhysicalSampleData result;result.time_s=start;result.end_time_s=start+.01;
    auto channels=json(R"({"model_status":{"model_valid":true,"numerically_valid":true},
        "tires":{"front":{"normal_load_n":100.0},"rear":{"normal_load_n":200.0}}})");
    auto status=get(channels,"model_status");set(status,"model_valid",Value{valid});set(channels,"model_status",std::move(status));
    result.channels=std::get<runtime::WireObject>(vread::wire_from_value(channels,"sample.channels").value);return result;
}
void invalid_prefix_and_axes(){
    ValidPrefix prefix;bt::require(prefix.observe(sample(0.,true)),"first fully valid physical interval enters prefix");
    bt::require(!prefix.observe(sample(.01,false))&&!prefix.observe(sample(.02,true)),"later valid rows never resume an invalid prefix");
    bt::require(prefix.seen==3&&prefix.accepted==1&&prefix.front_normal_impulse_ns==1.&&prefix.rear_normal_impulse_ns==2.,"invalid suffix contributes no impulse");
    bt::require(real(prefix.first_bad,"time_s")==.01,"first invalid interval remains the diagnostic origin");
    const auto empty=support_evidence(Value{Array{}},1.);
    bt::require(!flag(empty,"complete")&&!flag(empty,"valid_for_learning")&&io::is_null(get(empty,"model_valid")),"empty support evidence is unknown and incomplete");
    const auto axes=report_axes(json(R"({"accepted":true,"model_valid":false,"numerically_valid":true,"performance_passed":true})"));
    bt::require(false_value(get(axes,"physical_validity"))&&true_value(get(axes,"numerical_validity"))&&
        true_value(get(axes,"runtime_performance"))&&io::is_null(get(axes,"numerical_parity")),
        "fast and numerically finite results do not establish model validity or numerical parity");
    const auto unknown=report_axes(json(R"({"passed":true})"));
    bt::require(io::is_null(get(unknown,"physical_validity"))&&io::is_null(get(unknown,"runtime_performance")),"aggregate passed does not invent missing axes");
    const auto truthy=report_axes(json(R"({"model_valid":1,"numerically_valid":"true","performance_passed":1})"));
    for(const char* axis:{"physical_validity","numerical_validity","numerical_parity","runtime_performance"})
        bt::require(io::is_null(get(truthy,axis)),"only explicit boolean evidence populates an independent report axis");
}
void publication_coverage_contract(){
    auto first=sample(0.,true),second=sample(.01,true);first.interval_id=0;second.interval_id=1;
    IntervalCoverage complete(0,0.);complete.observe(first);complete.observe(second);complete.finish(2,.02);
    // These may arrive in one period batch or in two batches. Coverage is
    // defined over all committed physics intervals, independent of batching.
    IntervalCoverage duplicate(0,0.);duplicate.observe(first);
    bt::invalid([&]{duplicate.observe(first);});
    IntervalCoverage omitted(0,0.);omitted.observe(first);
    bt::invalid([&]{omitted.finish(2,.02);});
    auto skipped=second;skipped.interval_id=2;
    bt::invalid([&]{omitted.observe(skipped);});
    auto discontinuous=sample(.02,true);discontinuous.interval_id=1;
    bt::invalid([&]{omitted.observe(discontinuous);});
    IntervalCoverage empty(0,0.);bt::invalid([&]{empty.finish(1,.01);});
    bt::invalid([&]{complete.finish(2,.03);});
    IntervalCoverage resumed(1,.01);resumed.observe(second);resumed.finish(2,.02);
}
void release_and_bound_rejections(){
    auto evidence=json(R"({"model_valid":true,"numerically_valid":true,"contact_passed":true,"convergence_passed":true,
        "rider_budget_passed":true,"mass_momentum_passed":true,"transmission_parity_passed":true,"sensor_split_passed":true})");
    bt::require(flag(release_decision(evidence,false),"sil_ready")&&!flag(release_decision(evidence,true),"sil_ready"),"completion is an independent optional required gate");
    set(evidence,"contact_passed",value::integer(1));bt::require(!flag(release_decision(evidence,false),"sil_ready"),"integer truthiness cannot satisfy boolean evidence");
    bt::require(!release_gates(empty_object()).first&&!flag(release_decision(empty_object(),false),"sil_ready"),"empty reports reject both release contracts");
    bt::invalid([&]{(void)write_release_manifest("unused-unvalidated.json",empty_object(),"missing-profile.toml");});
    auto result=run_case("contact_bounds",json(R"({"metrics":{},"bounds":{}})"));bt::require(!result.accepted,"empty ad-hoc bounds cannot form accepted CLI evidence");
    result=run_case("contact_bounds",json(R"({"metrics":{"force":10.0},"bounds":{"force":[0,9],"missing":[0,1]}})"));
    bt::require(!result.accepted&&!flag(result.evidence,"force")&&!flag(result.evidence,"missing"),"out-of-range and absent metrics fail independently");
    bt::invalid([&]{(void)open_loop_schedule(0.);});bt::invalid([&]{(void)resume_schedule(-.1);});
    bt::invalid([&]{(void)automatic_schedule(std::numeric_limits<double>::quiet_NaN());});
    bt::require(!evidence_matches(empty_object(),empty_object()),"missing identities never authenticate evidence");
}
void intervals_and_dimensions(){
    bt::require(trace_integral(std::array{0.,1.,2.},std::array{0.,2.,2.})==3.,"sample trapezoid produces force-time integral");
    const auto rows=json(R"([{"time_s":0.0,"end_time_s":0.1,"force_n":10.0},{"time_s":0.1,"end_time_s":0.3,"force_n":20.0}])");
    bt::require(interval_integral(rows,"force_n")==10.*.1+20.*(.3-.1),"contiguous intervals use their actual durations");
    bt::invalid([&]{(void)trace_integral(std::array{0.,0.},std::array{1.,1.});});
    bt::invalid([&]{(void)interval_integral(json(R"([{"time_s":0,"end_time_s":0.1,"x":1},{"time_s":0.2,"end_time_s":0.3,"x":1}])"),"x");});
    const double front=quasistatic_front_load(100.,1.,.5,1.,0.);
    bt::require(front==490.5&&necessary_traction_margin(.8,.5,.2)==.2,"load balance is newtons; traction margin is dimensionless");
    bt::invalid([&]{(void)necessary_traction_margin(.8,1.01,.2);});
    const auto ledger=rider_work_ledger(json(R"({"act_rider_knee":5,"act_rider_hip":-2,"rider_passive_damping":-1,"mid_drive":7,"other":99})"));
    bt::require(real(ledger,"active_joint_work_j")==3.&&real(ledger,"net_joint_work_j")==2.&&real(ledger,"motor_mechanical_work_j")==7.,"rider and motor work remain separate physical channels");
    const auto timing=timing_metrics(std::array{1000000.,2000000.,3000000.,4000000.},.01);
    bt::require(real(timing,"p50_step_ms")==2.5&&real(timing,"real_time_factor")==4.,"latency milliseconds and simulated/wall throughput have explicit units");
    bt::invalid([&]{(void)timing_metrics(std::array{0.},.01);});
}
void resolution_and_pair_inventory(){
    const auto times=json("[0.00125,0.000625,0.0003125]"),roads=json("[0.01,0.005,0.0025]"),stations=json("[128,256,512]");
    const auto variants=resolution_variants(times,roads,stations,true);bt::require(io::as_array(get(variants,"variants"),"variants").size()==9,"all three axes retain all declared levels");
    bt::invalid([&]{(void)resolution_variants(times,roads,json("[128.0,256,512]"),true);});
    bt::require(io::is_null(select_verified_resolution(json("[]"))),"no measured candidates remains no selected resolution");
    const auto data=json(R"([{"scenario_id":"flat","seed":10,"plant_and_inputs_sha256":"p","policy_id":"passthrough","metrics":{"duration_s":1,"model_status":{"model_valid":true},"numerically_valid":true}},
        {"scenario_id":"flat","seed":2,"plant_and_inputs_sha256":"p","policy_id":"passthrough","metrics":{"duration_s":1,"model_status":{"model_valid":true},"numerically_valid":true}}])");
    const auto pairs=paired_summary(data);const auto& ordered=io::as_array(get(pairs,"pairs"),"pairs");
    bt::require(real(ordered[0],"seed")==2.&&real(ordered[1],"seed")==10.,"Python numeric seed ordering is not JSON lexicographic ordering");
    bt::require(real(pairs,"comparable_pair_count")==0.,"baseline-only groups are exclusions, not candidate comparisons");
    std::set<std::string_view> ids;for(const auto& entry:cases()){
        bt::require(!entry.id.empty()&&!entry.source.empty()&&!entry.description.empty(),"registry rows identify actual retained operations");
        bt::require(ids.insert(entry.id).second,"registry ids are unique");}
    for(const auto name:benchmark_case_names())bt::require(ids.contains(name),"every benchmark is directly discoverable");
    for(const char* name:{"tire-characterization","datasets","valid_prefix","contact_matrix","shaft_ratio","rider_factor_matrix","fidelity_sweep","climbing_benchmark","antiwheelie_bench","release_manifest"})bt::require(ids.contains(name),"validation domains have concrete public operations");
    bt::invalid([&]{(void)run_case("nonexistent",empty_object());});
    bt::invalid([&]{(void)benchmark_suite(json(R"({"jobs":2})"),".");});
}
void cli_option_contract(){
    // Exercise the frontend's actual grammar without invoking a subprocess,
    // opening case inputs, or performing any simulation as part of parsing.
    const std::array<std::pair<std::string_view,std::vector<std::string_view>>,9> modules{{
        {"rider-replay",{"--physics-config","--track","--duration","--timestep","--mode","--activation-tau","--output"}},
        {"rider-factor-matrix",{"--physics-config","--output"}},
        {"drive-suspension-rig",{"--output"}},
        {"contact-resolution",{"--output"}},
        {"fidelity-sweep",{"--output","--cases","--backend","--transmission","--stations","--jobs","--time-steps","--road-steps","--station-counts","--physics-config"}},
        {"reference-release",{"--output","--physics-config","--dt","--dx","--scenarios"}},
        {"climbing-benchmark",{"--physics-config","--track","--mode","--duration","--repeats","--output"}},
        {"antiwheelie-bench",{"--physics-config","--output","--phase"}},
        {"benchmarks",{"--output","--cases","--dt-values","--jobs"}}}};
    for(const auto& [module,flags]:modules){CliOptions options(module);for(const auto flag_name:flags)options.accept(flag_name);
        options.accept("--config");options.accept("--root");options.accept("--report");
        bt::invalid([&]{options.accept("--unrecognized");});}
    for(const auto& [module,flag_name]:std::array<std::pair<std::string_view,std::string_view>,7>{{
            {"datasets","--duration"},{"drive-suspension-rig","--dt"},{"contact-resolution","--jobs"},
            {"rider-replay","--backend"},{"climbing-benchmark","--activation-tau"},
            {"fidelity-sweep","--dt"},{"benchmarks","--transmission"}}}){
        CliOptions options(module);bt::invalid([&]{options.accept(flag_name);});
    }
    CliOptions aliases("rider-replay");aliases.accept("--dt");bt::invalid([&]{aliases.accept("--timestep");});
    CliOptions reverse_aliases("radial_rig");reverse_aliases.accept("--timestep");bt::invalid([&]{reverse_aliases.accept("--dt");});
    CliOptions duplicate("tire-characterization");duplicate.accept("--output");bt::invalid([&]{duplicate.accept("--output");});
    bt::invalid([&]{(void)CliOptions("nonexistent");});
    for(const auto name:benchmark_case_names()){CliOptions options(name);options.accept("--dt");}
}
void actual_stand_checks(){
    // Preserve the source horizons, loads and thresholds. There is no short
    // substitute trajectory and no golden update mode in this regression.
    for(const char* name:{"wheel_inertia","chain_suspension_motion","actual_shaft_ratio","actual_freehub_overrun"}){
        const auto result=benchmark_case(name,.0005);const auto checks=check_bounds(result.metrics,result.bounds);
        bt::require(!io::as_object(checks,"checks").empty()&&checks_passed(checks),std::string("source stand conservation/geometry bounds: ")+name);
    }
}
} // namespace
int main(){return bt::run([]{
    captured_tire_characterization();material_curve_contract();calibration_evidence_contract();invalid_prefix_and_axes();publication_coverage_contract();
    release_and_bound_rejections();intervals_and_dimensions();resolution_and_pair_inventory();cli_option_contract();actual_stand_checks();
});}
