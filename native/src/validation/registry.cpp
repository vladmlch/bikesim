// Explicit retained-operation registry: plan-08 T54 and source module public
// callables. Physics cases reuse the same rigs as benchmark/convergence runs.
#include "acceptance.hpp"
#include "evidence.hpp"
#include "rigs.hpp"
#include "scenarios.hpp"
#include "tire.hpp"
#include "../configuration/load.hpp"
#include "../runtime/value_readers.hpp"
#include "../terrain/types.hpp"
#include <array>

namespace bike::validation {
namespace {
constexpr auto operations=std::to_array<CaseDescription>({
    {"tire-characterization","validation/tire_characterization.py:12-42","Isolated longitudinal brush material bench",false},
    {"tire_calibration","physics/tire_calibration.py:16-84","Explicit radial curve, geometry and provenance",false},
    {"tire_curve","physics/tire_curve.py:17-92","Tabulated elastic/damped contact and stored energy",false},
    {"datasets","validation/datasets.py:41-237","Measured experiment schema and whole-experiment holdout",false},
    {"dataset_hash","validation/datasets.py:30-38","Canonical finite JSON SHA-256",false},
    {"calibration_report","validation/datasets.py:136-184","Per-experiment holdout errors and parameter/error budgets",false},
    {"holdout_metrics","validation/datasets.py:121-134","Bias, maximum absolute error and RMSE",false},
    {"experiment_acceptance","validation/experiment_acceptance.py:8-44","Disjoint experiments and explicit calibration gates",false},
    {"contact_bounds","validation/contact_acceptance.py:12-29","Finite, present metrics within declared bounds",false},
    {"contact_matrix","validation/contact_acceptance.py:32-82","Flat, step and incline station/timestep matrix",true},
    {"radial_rig","validation/rigs.py:23-36","Loaded radial spring/damper stand",true},
    {"native_radial_rig","validation/rigs.py:38-51","Diagnostic native reference radial stand",true},
    {"wheel_inertia_rig","validation/rigs.py:53-66","Applied torque versus known wheel inertia",false},
    {"incline_rig","validation/rigs.py:68-88","Gravity decomposition on a constrained incline",true},
    {"brake_stand","validation/rigs.py:91-106","Static hold then overload friction work",true},
    {"chain_locked_rig","validation/rigs.py:108-127","Locked geometry chain/freehub ratio and power",true},
    {"chain_geometry","validation/rigs.py:129-138","Rigid-motion invariance and directional derivative",false},
    {"compiled_chain_motion","validation/rigs.py:158-194","Actual linkage geometry Jacobian finite difference",true},
    {"load_transfer","validation/load_transfer.py:24-71","Free-pitch rigid stand and independent normal-load balance",true},
    {"quasistatic_front_load","validation/load_transfer.py:13-21","Explicit gravity and acceleration load transfer",false},
    {"grade_feasibility","validation/grade_feasibility.py:5-20","Necessary rear traction margin; no completion claim",false},
    {"distributed_flat","validation/contact_manifold_rigs.py:80-92","Distributed contact flat-load stand",true},
    {"distributed_step_rig","validation/contact_manifold_rigs.py:125-128","Distributed contact step road stand",true},
    {"distributed_incline_rig","validation/contact_manifold_rigs.py:131-173","No-slip incline load decomposition",true},
    {"distributed_road_trace","validation/contact_manifold_rigs.py:108-123","Actual station/contact trajectories and energy",true},
    {"contact_case_vertices","validation/contact_manifold_rigs.py:94-105","Declared analytic contact road discretization",false},
    {"shaft_ratio","validation/plant_torque_rig.py:7-40","Actual motor shaft, unilateral tendon/freehub and work",true},
    {"drive_suspension","validation/drive_suspension_rig.py:61-124","Free suspension under imposed actual shaft torque",true},
    {"drive_suspension_rig","validation/drive_suspension_rig.py:127-163","Three transmissions, shocks and torque comparison",true},
    {"momentum","validation/system_momentum.py:28-128","Independent whole-system momentum ledger",true},
    {"rider_work","validation/rider_work.py:4-10","Active/passive rider work channels",false},
    {"valid_prefix","validation/plant_prefix.py:7-43","Immutable rejection of the first invalid plant interval",false},
    {"plant_release","validation/plant_release.py:13-18","Independent fail-closed SIL evidence gates",false},
    {"trace_integral","validation/plant_release.py:21-28","Strictly increasing paired trapezoidal integral",false},
    {"interval_integral","validation/fidelity_sweep.py:49-59","Contiguous interval-weighted physical integral",false},
    {"support_evidence","validation/climbing_evidence.py:7-111","Complete-horizon support, activation and scope evidence",false},
    {"coasting_evidence","validation/rider_replay.py:43-87","Automatic coast phases and frozen pedal targets",false},
    {"resume_evidence","validation/rider_replay.py:159-216","Measured coast/resume diagnosis",false},
    {"timing_metrics","validation/climbing_benchmark.py:20-29","Completed-step latency percentiles and throughput",false},
    {"assess_run","validation/antiwheelie_reports.py:7-28","Full episode validity and explicit outcome",false},
    {"paired_summary","validation/antiwheelie_reports.py:31-70","Comparable matched-policy records and exclusions",false},
    {"classify_outcome","validation/reference_release.py:103-150","Invalid-prefix, crash, goal and persistent-condition ordering",false},
    {"resolution_variants","validation/fidelity_sweep.py:25-40","Strict ordered time, road and station axes",false},
    {"persistent_events","validation/fidelity_sweep.py:191-199","Contact transitions confirmed by dwell time",false},
    {"contact_comparison","validation/contact_resolution.py:27-36","Impulse/energy/event convergence criteria",false},
    {"fidelity_compare","validation/fidelity_sweep.py:66-103","Full physical metrics and shared-initial-state convergence",false},
    {"metrics_from_rows","validation/fidelity_sweep.py:202-232","Physical interval metrics with completeness and prefix evidence",false},
    {"select_verified_resolution","validation/reference_release.py:36-52","Only measured candidates passing every axis",false},
    {"release_gates","validation/reference_release.py:22-30","Required numerical reference gates without promotion",false},
    {"release_manifest","validation/reference_release.py:55-100","Hash-verified reference release and measured holdout evidence",false},
    {"scenarios","validation/reference_release.py:153-163","Explicit eighteen-scenario reference matrix definition",false},
    {"rider_replay","validation/rider_replay.py:227-295","One declared isolated replay factor",true},
    {"rider_factor_matrix","validation/rider_factor_matrix.py:7-40","Nine causal rider factors including unsupported control rejection",true},
    {"automatic_coast","validation/rider_replay.py:90-104","Exact fast-profile automatic coast/resume run",true},
    {"fidelity_sweep","validation/fidelity_sweep.py:273-409","Time, road and station convergence from shared initial states",true},
    {"reference_release","validation/reference_release.py:195-289","Reference scenario matrix; unvalidated report",true},
    {"contact_resolution","validation/contact_resolution.py:39-95","Independent contact resolution and density-fit stands",true},
    {"benchmarks","validation/benchmarks.py:163-240","Versioned case registry and convergence gates",true},
    {"climbing_benchmark","validation/climbing_benchmark.py:32-166","Setup, warmup, timing and validity kept separate",true},
    {"antiwheelie_bench","validation/antiwheelie_bench.py:74-133","Source-matched mechanics evidence pipeline",true},
    {"environment","validation/environment.py:31-82","Actual native build lock, engine and provider identity",false},
    {"profile_identity","validation/antiwheelie_bench.py:29-43","Resolved physical profile and source identity",false},
    {"evidence_matches","validation/antiwheelie_bench.py:23-26","All three source/configuration/version identities must exist",false},
    {"source_fingerprint","validation/environment.py:17-28","Sorted length-delimited native source SHA-256",false}});
bool boolean(const Value& input,std::string_view name,bool fallback=false){const auto* found=find(input,name);return found?io::as_bool(*found,name):fallback;}
Verdict bounded_case(RigResult result){auto checks=check_bounds(result.metrics,result.bounds);const bool nonempty=!io::as_object(result.bounds,"bounds").empty();
    return {nonempty&&checks_passed(checks),Value{Object{{"metrics",std::move(result.metrics)},{"bounds",std::move(result.bounds)},{"checks",std::move(checks)}}}};}
Verdict report_result(Value result,std::string_view key="passed"){const bool accepted=flag(result,key);return {accepted,std::move(result)};}
Verdict diagnostic(Value result){return {false,std::move(result)};}
std::optional<double> optional_real(const Value& input,std::string_view key){const auto* item=find(input,key);return item&&!io::is_null(*item)?std::optional(io::as_double(*item,key)):std::nullopt;}
std::string normalized(std::string_view input){std::string result(input);std::ranges::replace(result,'-','_');return result;}
Value prefix_report(const Value& input){ValidPrefix prefix;for(const auto& row:io::as_array(get(input,"samples"),"samples")){
    runtime::PhysicalSampleData sample;sample.time_s=real(row,"time_s");sample.end_time_s=real(row,"end_time_s");
    sample.channels=std::get<runtime::WireObject>(vread::wire_from_value(get(row,"channels"),"prefix.channels").value);(void)prefix.observe(sample);}
    return Value{Object{{"seen",value::integer(static_cast<std::int64_t>(prefix.seen))},{"accepted",value::integer(static_cast<std::int64_t>(prefix.accepted))},
        {"valid_until_s",value::real(prefix.valid_until_s)},{"first_bad",prefix.first_bad},
        {"front_normal_impulse_ns",value::real(prefix.front_normal_impulse_ns)},{"rear_normal_impulse_ns",value::real(prefix.rear_normal_impulse_ns)}}};}
Value tire_characterization(const Value& config,const std::filesystem::path& base){
    nativecfg::TireParams tire;
    if(const auto* profile=find(config,"physics_profile")){const auto c=configuration::load_physics(base/io::as_string(*profile,"physics_profile"));
        tire.tangent_k_n_m=c.tires.front.tangent_k_n_m;tire.mu=c.tires.front.mu;tire.relaxation_length_m=c.tires.front.relaxation_length_m;}
    else{const auto& t=get(config,"tire");tire.tangent_k_n_m=real(t,"tangent_k_n_m");tire.mu=real(t,"mu");tire.relaxation_length_m=real(t,"relaxation_length_m");}
    const auto surface_name=identifier(get(config,"surface"),"surface");const auto& s=terrain::get_surface(surface_name);
    const nativecfg::SurfaceSpec surface{s.name,s.mu_peak,s.mu_slide,s.slip_stiffness_per_load,s.stribeck_speed_mps};
    const auto slips=numbers(get(config,"slip_ratios"),"slip ratios"),loads=numbers(get(config,"loads_n"),"loads");
    if(slips.empty()||loads.empty())throw std::invalid_argument("tire characterization requires nonempty slip/load axes");Array rows;
    for(double slip:slips)for(double load:loads)rows.push_back(steady_brush_force(tire,surface,slip,load,real(config,"speed_mps",2.),real(config,"dt_s",.000625),real(config,"duration_s",1.)));
    return Value{Object{{"surface",value::text(surface_name)},{"cases",Value{std::move(rows)}}}};
}
// One handler per registered id keeps run_case's stack frame under the
// project limit at -O0; bodies are verbatim from the previous if-chain.
double case_dt(const Value& c){return real(c,"dt_s",.000625);}
int case_stations(const Value& c){const auto n=io::as_int64(optional(c,"station_count",value::integer(128)),"station_count");if(n<16||n>std::numeric_limits<int>::max())throw std::invalid_argument("station count outside native domain");return static_cast<int>(n);}
Verdict case_tire_characterization(const Value& c,const std::filesystem::path& base){auto report=tire_characterization(c,base);return {true,std::move(report)};}
Verdict case_tire_calibration(const Value& c,const std::filesystem::path&){const auto& record=get(c,"record");const auto r=calibration_from_record(record);return {true,Value{Object{{"record",record},{"source_kind",value::text(r.source_kind)},{"calibration_status",value::text("parameterized_unvalidated")}}}};}
Verdict case_tire_curve(const Value& c,const std::filesystem::path&){const nativecfg::TireTable table{numbers(get(c,"deflection_m"),"deflection"),numbers(get(c,"force_n"),"force")};const contactlaw::TabulatedRadial curve(table);const double damping=nonnegative(real(c,"radial_c_ns_m",0.),"radial damping");Array rows;
    for(const auto& entry:io::as_array(get(c,"samples"),"samples")){const auto elastic=curve.elastic_response(real(entry,"penetration_m"));const auto contact=curve.normal_contact(real(entry,"penetration_m"),real(entry,"rate_mps",0.),damping);
        rows.push_back(Value{Object{{"elastic_force_n",value::real(elastic.first)},{"stored_energy_j",value::real(elastic.second)},{"normal_force_n",value::real(contact.first)},{"contact_stored_energy_j",value::real(contact.second)}}});}
    if(rows.empty())throw std::invalid_argument("curve samples required");return {true,Value{std::move(rows)}};}
Verdict case_datasets(const Value& c,const std::filesystem::path&){auto payload=validate_dataset(get(c,"dataset"),boolean(c,"allow_synthetic"));return {true,std::move(payload)};}
Verdict case_dataset_hash(const Value& c,const std::filesystem::path&){return {true,value::text(dataset_hash(get(c,"dataset")))};}
Verdict case_calibration_report(const Value& c,const std::filesystem::path&){return report_result(calibration_report(get(c,"dataset"),get(c,"predictions"),text(c,"quantity"),get(c,"parameters"),get(c,"bounds"),get(c,"error_budget"),boolean(c,"allow_synthetic")),"holdout_passed");}
Verdict case_holdout_metrics(const Value& c,const std::filesystem::path&){return diagnostic(holdout_metrics(numbers(get(c,"measured"),"measured"),numbers(get(c,"predicted"),"predicted")));}
Verdict case_experiment_acceptance(const Value& c,const std::filesystem::path&){validate_split(get(c,"fit"),get(c,"holdout"));const auto status=accepted_calibration_status(io::as_bool(get(c,"synthetic"),"synthetic"),io::as_bool(get(c,"holdout_passed"),"holdout_passed"),io::as_bool(get(c,"converged"),"converged"),io::as_bool(get(c,"within_scope"),"within_scope"));
    return {status=="validated_within_declared_scope",Value{Object{{"status",value::text(status)},{"measurement_authentication",Value{false}}}}};}
Verdict case_contact_bounds(const Value& c,const std::filesystem::path&){auto checks=check_bounds(get(c,"metrics"),get(c,"bounds"));const bool accepted=!io::as_object(checks,"checks").empty()&&checks_passed(checks);return {accepted,std::move(checks)};}
Verdict case_contact_matrix(const Value& c,const std::filesystem::path&){auto rows=contact_matrix(c);bool accepted=!io::as_array(rows,"matrix").empty();for(const auto& row:io::as_array(rows,"matrix"))accepted=accepted&&flag(row,"accepted");return {accepted,std::move(rows)};}
Verdict case_radial_rig(const Value& c,const std::filesystem::path&){return diagnostic(radial_rig(real(c,"load_n"),case_dt(c),real(c,"duration_s",2.),false));}
Verdict case_native_radial_rig(const Value& c,const std::filesystem::path&){return diagnostic(radial_rig(real(c,"load_n"),case_dt(c),real(c,"duration_s",2.),true));}
Verdict case_wheel_inertia_rig(const Value& c,const std::filesystem::path&){return diagnostic(wheel_inertia_rig(case_dt(c),real(c,"duration_s",.5)));}
Verdict case_incline_rig(const Value& c,const std::filesystem::path&){return diagnostic(incline_rig(case_dt(c),real(c,"duration_s",2.),real(c,"angle_deg",30.)));}
Verdict case_brake_stand(const Value& c,const std::filesystem::path&){return diagnostic(brake_rig(case_dt(c),real(c,"duration_s",5.)));}
Verdict case_chain_locked_rig(const Value& c,const std::filesystem::path&){return diagnostic(chain_locked_rig(case_dt(c),real(c,"duration_s",1.)));}
Verdict case_chain_geometry(const Value&,const std::filesystem::path&){return diagnostic(chain_geometry_rig());}
Verdict case_compiled_chain_motion(const Value&,const std::filesystem::path&){return diagnostic(compiled_chain_motion_rig());}
Verdict case_load_transfer(const Value& c,const std::filesystem::path&){return bounded_case(load_transfer_rig(case_dt(c),real(c,"slope_rad"),real(c,"com_x_m"),real(c,"com_h_m")));}
Verdict case_quasistatic_front_load(const Value& c,const std::filesystem::path&){return diagnostic(value::real(quasistatic_front_load(real(c,"mass_kg"),real(c,"wheelbase_m"),real(c,"com_x_m"),real(c,"com_h_m"),real(c,"slope_rad"),real(c,"acceleration_mps2",0.),real(c,"gravity_mps2",9.81))));}
Verdict case_grade_feasibility(const Value& c,const std::filesystem::path&){const double margin=necessary_traction_margin(real(c,"mu"),real(c,"rear_fraction"),real(c,"grade"));return diagnostic(Value{Object{{"necessary_traction_margin",value::real(margin)},{"necessary_condition_met",Value{margin>=0.}},{"sufficient_for_completion",Value{false}}}});}
Verdict case_distributed_flat(const Value& c,const std::filesystem::path&){return bounded_case(distributed_flat_rig(case_dt(c),case_stations(c),real(c,"load_n")));}
Verdict case_distributed_step_rig(const Value& c,const std::filesystem::path&){return bounded_case(distributed_step_rig(case_dt(c),case_stations(c),real(c,"dx_m",.005)));}
Verdict case_distributed_incline_rig(const Value& c,const std::filesystem::path&){return bounded_case(distributed_incline_rig(case_dt(c),case_stations(c),real(c,"angle_deg",15.)));}
Verdict case_distributed_road_trace(const Value& c,const std::filesystem::path&){auto [rows,metrics]=distributed_road_trace(case_dt(c),case_stations(c),real(c,"dx_m",.005),text(c,"case","step"),real(c,"duration_s",.4));return diagnostic(Value{Object{{"rows",std::move(rows)},{"metrics",std::move(metrics)}}});}
Verdict case_contact_case_vertices(const Value& c,const std::filesystem::path&){Array vertices;for(const auto& vertex:contact_case_vertices(text(c,"case","step"),real(c,"dx_m",.005)))vertices.push_back(value::doubles(vertex));return {true,Value{std::move(vertices)}};}
Verdict case_shaft_ratio(const Value& c,const std::filesystem::path&){return bounded_case(shaft_ratio_rig(case_dt(c),real(c,"ratio",34./51.),boolean(c,"overrun")));}
Verdict case_drive_suspension(const Value& c,const std::filesystem::path&){return bounded_case(drive_suspension_rig(case_dt(c),text(c,"transmission","geometric_ideal_mid_drive"),real(c,"torque_nm"),real(c,"shock_stroke_m",.015),real(c,"duration_s",.05)));}
Verdict case_drive_suspension_rig(const Value& c,const std::filesystem::path& base){auto config=c;if(!find(config,"output"))value::set(std::get<Object>(config.data),"output",value::text("results/drive_suspension.json"));return report_result(drive_suspension_comparison(config,base));}
Verdict case_momentum(const Value& c,const std::filesystem::path&){std::optional<bool> rider;if(const auto* enabled=find(c,"rider_active"))rider=io::as_bool(*enabled,"rider_active");return bounded_case(momentum_rig(case_dt(c),boolean(c,"active"),text(c,"transmission","geometric_ideal_mid_drive"),rider));}
Verdict case_rider_work(const Value& c,const std::filesystem::path&){return diagnostic(rider_work_ledger(get(c,"work_j")));}
Verdict case_valid_prefix(const Value& c,const std::filesystem::path&){auto report=prefix_report(c);const bool accepted=real(report,"accepted")>0.&&real(report,"accepted")==real(report,"seen")&&io::is_null(get(report,"first_bad"));return {accepted,std::move(report)};}
Verdict case_plant_release(const Value& c,const std::filesystem::path&){return report_result(release_decision(get(c,"evidence"),io::as_bool(get(c,"require_completion"),"require_completion")),"sil_ready");}
Verdict case_trace_integral(const Value& c,const std::filesystem::path&){return diagnostic(value::real(trace_integral(numbers(get(c,"time_s"),"times"),numbers(get(c,"values"),"values"))));}
Verdict case_interval_integral(const Value& c,const std::filesystem::path&){return diagnostic(value::real(interval_integral(get(c,"rows"),text(c,"key"))));}
Verdict case_support_evidence(const Value& c,const std::filesystem::path&){return report_result(support_evidence(get(c,"rows"),optional_real(c,"required_duration_s")),"valid_for_learning");}
Verdict case_coasting_evidence(const Value& c,const std::filesystem::path&){return diagnostic(coasting_evidence(get(c,"rows")));}
Verdict case_resume_evidence(const Value& c,const std::filesystem::path&){return diagnostic(resume_evidence(get(c,"rows"),real(c,"end_s",8.)));}
Verdict case_timing_metrics(const Value& c,const std::filesystem::path&){return diagnostic(timing_metrics(numbers(get(c,"step_ns"),"step_ns"),case_dt(c)));}
Verdict case_assess_run(const Value& c,const std::filesystem::path&){return report_result(assess_run(get(c,"summary"),real(c,"required_duration_s"),real(c,"minimum_progress_m",0.)),"accepted");}
Verdict case_paired_summary(const Value& c,const std::filesystem::path&){return diagnostic(paired_summary(get(c,"records")));}
Verdict case_classify_outcome(const Value& c,const std::filesystem::path&){std::optional<std::string> error;if(const auto* v=find(c,"numerical_error"))if(!io::is_null(*v))error=io::as_string(*v,"numerical_error");return diagnostic(classify_outcome(get(c,"rows"),optional_real(c,"goal_x_m"),boolean(c,"crashed"),error,boolean(c,"duration_reached")));}
Verdict case_resolution_variants(const Value& c,const std::filesystem::path&){return {true,resolution_variants(get(c,"time_steps"),get(c,"road_steps"),get(c,"station_counts"),boolean(c,"include_stations",true))};}
Verdict case_persistent_events(const Value& c,const std::filesystem::path&){return diagnostic(persistent_events(get(c,"rows"),real(c,"persistence_s",.02),boolean(c,"normal_load_only")));}
Verdict case_contact_comparison(const Value& c,const std::filesystem::path&){return report_result(contact_comparison(get(c,"coarse"),get(c,"reference")));}
Verdict case_fidelity_compare(const Value& c,const std::filesystem::path&){return report_result(compare_metrics(get(c,"coarse"),get(c,"reference")));}
Verdict case_metrics_from_rows(const Value& c,const std::filesystem::path&){return diagnostic(metrics_from_rows(get(c,"rows"),real(c,"expected_duration_s"),text(c,"initial_state_sha256")));}
Verdict case_select_verified_resolution(const Value& c,const std::filesystem::path&){auto result=select_verified_resolution(get(c,"comparisons"));const bool accepted=!io::is_null(result);return {accepted,std::move(result)};}
Verdict case_release_gates(const Value& c,const std::filesystem::path&){const auto [accepted,missing]=release_gates(get(c,"report"));return {accepted,Value{Object{{"passed",Value{accepted}},{"missing",value::strings(missing)},{"status",value::text(release_level(get(c,"report")))}}}};}
Verdict case_release_manifest(const Value& c,const std::filesystem::path& base){return {true,write_release_manifest(base/text(c,"output"),get(c,"report"),base/text(c,"physics_profile"),optional(c,"measurement_evidence",Value{Array{}}))};}
Verdict case_scenarios(const Value&,const std::filesystem::path&){return diagnostic(scenario_definitions());}
Verdict case_rider_replay(const Value& c,const std::filesystem::path& base){return report_result(run_replay(c,base),"completed_requested_duration");}
Verdict case_rider_factor_matrix(const Value& c,const std::filesystem::path& base){auto result=rider_factor_matrix(c,base);const bool accepted=flag(result,"baseline_resumed")&&flag(result,"causal_replay_resolved");return {accepted,std::move(result)};}
Verdict case_automatic_coast(const Value& c,const std::filesystem::path& base){return diagnostic(automatic_coast_case(case_dt(c),real(c,"duration_s",5.),base));}
Verdict case_fidelity_sweep(const Value& c,const std::filesystem::path& base){return report_result(fidelity_sweep(c,base));}
Verdict case_reference_release(const Value& c,const std::filesystem::path& base){return diagnostic(reference_matrix(c,base));}
Verdict case_contact_resolution(const Value& c,const std::filesystem::path& base){return report_result(contact_resolution(c,base));}
Verdict case_benchmarks(const Value& c,const std::filesystem::path& base){return report_result(benchmark_suite(c,base));}
Verdict case_climbing_benchmark(const Value& c,const std::filesystem::path& base){return report_result(climbing_benchmark(c,base));}
Verdict case_antiwheelie_bench(const Value& c,const std::filesystem::path& base){if(text(c,"phase","mechanics")!="mechanics")throw std::invalid_argument("antiwheelie phase must be mechanics");return report_result(mechanics_report(c,base));}
Verdict case_environment(const Value&,const std::filesystem::path& base){return report_result(environment_contract(base),"locked_environment_verified");}
Verdict case_profile_identity(const Value& c,const std::filesystem::path& base){return diagnostic(profile_identity(base/text(c,"physics_profile"),base));}
Verdict case_evidence_matches(const Value& c,const std::filesystem::path&){const bool matched=evidence_matches(get(c,"record"),get(c,"expected"));return {matched,Value{Object{{"matches",Value{matched}}}}};}
Verdict case_source_fingerprint(const Value& c,const std::filesystem::path& base){return diagnostic(value::text(source_fingerprint(base/text(c,"source_root","native/src"))));}
struct CaseHandler{std::string_view id;Verdict (*run)(const Value&,const std::filesystem::path&);};
constexpr auto handlers=std::to_array<CaseHandler>({
    {"tire_characterization",case_tire_characterization},{"tire_calibration",case_tire_calibration},{"tire_curve",case_tire_curve},
    {"datasets",case_datasets},{"dataset_hash",case_dataset_hash},{"calibration_report",case_calibration_report},
    {"holdout_metrics",case_holdout_metrics},{"experiment_acceptance",case_experiment_acceptance},{"contact_bounds",case_contact_bounds},
    {"contact_matrix",case_contact_matrix},{"radial_rig",case_radial_rig},{"native_radial_rig",case_native_radial_rig},
    {"wheel_inertia_rig",case_wheel_inertia_rig},{"incline_rig",case_incline_rig},{"brake_stand",case_brake_stand},
    {"chain_locked_rig",case_chain_locked_rig},{"chain_geometry",case_chain_geometry},{"compiled_chain_motion",case_compiled_chain_motion},
    {"load_transfer",case_load_transfer},{"quasistatic_front_load",case_quasistatic_front_load},{"grade_feasibility",case_grade_feasibility},
    {"distributed_flat",case_distributed_flat},{"distributed_step_rig",case_distributed_step_rig},{"distributed_incline_rig",case_distributed_incline_rig},
    {"distributed_road_trace",case_distributed_road_trace},{"contact_case_vertices",case_contact_case_vertices},{"shaft_ratio",case_shaft_ratio},
    {"drive_suspension",case_drive_suspension},{"drive_suspension_rig",case_drive_suspension_rig},{"momentum",case_momentum},
    {"rider_work",case_rider_work},{"valid_prefix",case_valid_prefix},{"plant_release",case_plant_release},
    {"trace_integral",case_trace_integral},{"interval_integral",case_interval_integral},{"support_evidence",case_support_evidence},
    {"coasting_evidence",case_coasting_evidence},{"resume_evidence",case_resume_evidence},{"timing_metrics",case_timing_metrics},
    {"assess_run",case_assess_run},{"paired_summary",case_paired_summary},{"classify_outcome",case_classify_outcome},
    {"resolution_variants",case_resolution_variants},{"persistent_events",case_persistent_events},{"contact_comparison",case_contact_comparison},
    {"fidelity_compare",case_fidelity_compare},{"metrics_from_rows",case_metrics_from_rows},{"select_verified_resolution",case_select_verified_resolution},
    {"release_gates",case_release_gates},{"release_manifest",case_release_manifest},{"scenarios",case_scenarios},
    {"rider_replay",case_rider_replay},{"rider_factor_matrix",case_rider_factor_matrix},{"automatic_coast",case_automatic_coast},
    {"fidelity_sweep",case_fidelity_sweep},{"reference_release",case_reference_release},{"contact_resolution",case_contact_resolution},
    {"benchmarks",case_benchmarks},{"climbing_benchmark",case_climbing_benchmark},{"antiwheelie_bench",case_antiwheelie_bench},
    {"environment",case_environment},{"profile_identity",case_profile_identity},{"evidence_matches",case_evidence_matches},
    {"source_fingerprint",case_source_fingerprint}});
} // namespace

std::span<const CaseDescription> cases(){
    static const std::vector<CaseDescription> table=[] {std::vector<CaseDescription> rows(operations.begin(),operations.end());
        for(const auto id:benchmark_case_names())rows.push_back({id,"validation/benchmarks.py:89-118","Bounded source benchmark case",true});return rows;}();
    return table;
}
Value report_axes(const Value& evidence){
    auto model=optional(evidence,"model_valid");if(io::is_null(model))model=optional(optional(evidence,"model_status",empty_object()),"model_valid");
    auto numerical=optional(evidence,"numerically_valid");if(io::is_null(numerical))numerical=optional(optional(evidence,"model_status",empty_object()),"numerically_valid");
    const auto performance=optional(evidence,"performance_passed");
    return Value{Object{{"physical_validity",std::holds_alternative<bool>(model.data)?model:Value{}},
        {"numerical_validity",std::holds_alternative<bool>(numerical.data)?numerical:Value{}},
        {"numerical_parity",Value{}},{"runtime_performance",std::holds_alternative<bool>(performance.data)?performance:Value{}}}};
}
Verdict run_case(std::string_view id,const std::filesystem::path& configuration_file){
    const auto input=configuration_file.extension()==".toml"?configuration::detail::parse_toml_file(configuration_file):read_json(configuration_file);
    return run_case(id,input,configuration_file.parent_path());
}
Verdict run_case(std::string_view input_id,const Value& c,const std::filesystem::path& base){
    (void)io::as_object(c,"case configuration");const auto id=normalized(input_id);
    if(const auto found=std::ranges::find_if(handlers,[&](const CaseHandler& handler){return handler.id==id;});found!=handlers.end())return found->run(c,base);
    if(std::ranges::find(benchmark_case_names(),id)!=benchmark_case_names().end())return bounded_case(benchmark_case(id,case_dt(c)));
    throw std::invalid_argument("unknown validation case: "+std::string(input_id));
}
} // namespace bike::validation
