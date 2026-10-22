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
    {.id="tire-characterization",.source="validation/tire_characterization.py:12-42",.description="Isolated longitudinal brush material bench",.slow=false},
    {.id="tire_calibration",.source="physics/tire_calibration.py:16-84",.description="Explicit radial curve, geometry and provenance",.slow=false},
    {.id="tire_curve",.source="physics/tire_curve.py:17-92",.description="Tabulated elastic/damped contact and stored energy",.slow=false},
    {.id="datasets",.source="validation/datasets.py:41-237",.description="Measured experiment schema and whole-experiment holdout",.slow=false},
    {.id="dataset_hash",.source="validation/datasets.py:30-38",.description="Canonical finite JSON SHA-256",.slow=false},
    {.id="calibration_report",.source="validation/datasets.py:136-184",.description="Per-experiment holdout errors and parameter/error budgets",.slow=false},
    {.id="holdout_metrics",.source="validation/datasets.py:121-134",.description="Bias, maximum absolute error and RMSE",.slow=false},
    {.id="experiment_acceptance",.source="validation/experiment_acceptance.py:8-44",.description="Disjoint experiments and explicit calibration gates",.slow=false},
    {.id="contact_bounds",.source="validation/contact_acceptance.py:12-29",.description="Finite, present metrics within declared bounds",.slow=false},
    {.id="contact_matrix",.source="validation/contact_acceptance.py:32-82",.description="Flat, step and incline station/timestep matrix",.slow=true},
    {.id="radial_rig",.source="validation/rigs.py:23-36",.description="Loaded radial spring/damper stand",.slow=true},
    {.id="native_radial_rig",.source="validation/rigs.py:38-51",.description="Diagnostic native reference radial stand",.slow=true},
    {.id="wheel_inertia_rig",.source="validation/rigs.py:53-66",.description="Applied torque versus known wheel inertia",.slow=false},
    {.id="incline_rig",.source="validation/rigs.py:68-88",.description="Gravity decomposition on a constrained incline",.slow=true},
    {.id="brake_stand",.source="validation/rigs.py:91-106",.description="Static hold then overload friction work",.slow=true},
    {.id="chain_locked_rig",.source="validation/rigs.py:108-127",.description="Locked geometry chain/freehub ratio and power",.slow=true},
    {.id="chain_geometry",.source="validation/rigs.py:129-138",.description="Rigid-motion invariance and directional derivative",.slow=false},
    {.id="compiled_chain_motion",.source="validation/rigs.py:158-194",.description="Actual linkage geometry Jacobian finite difference",.slow=true},
    {.id="load_transfer",.source="validation/load_transfer.py:24-71",.description="Free-pitch rigid stand and independent normal-load balance",.slow=true},
    {.id="quasistatic_front_load",.source="validation/load_transfer.py:13-21",.description="Explicit gravity and acceleration load transfer",.slow=false},
    {.id="grade_feasibility",.source="validation/grade_feasibility.py:5-20",.description="Necessary rear traction margin; no completion claim",.slow=false},
    {.id="distributed_flat",.source="validation/contact_manifold_rigs.py:80-92",.description="Distributed contact flat-load stand",.slow=true},
    {.id="distributed_step_rig",.source="validation/contact_manifold_rigs.py:125-128",.description="Distributed contact step road stand",.slow=true},
    {.id="distributed_incline_rig",.source="validation/contact_manifold_rigs.py:131-173",.description="No-slip incline load decomposition",.slow=true},
    {.id="distributed_road_trace",.source="validation/contact_manifold_rigs.py:108-123",.description="Actual station/contact trajectories and energy",.slow=true},
    {.id="contact_case_vertices",.source="validation/contact_manifold_rigs.py:94-105",.description="Declared analytic contact road discretization",.slow=false},
    {.id="shaft_ratio",.source="validation/plant_torque_rig.py:7-40",.description="Actual motor shaft, unilateral tendon/freehub and work",.slow=true},
    {.id="drive_suspension",.source="validation/drive_suspension_rig.py:61-124",.description="Free suspension under imposed actual shaft torque",.slow=true},
    {.id="drive_suspension_rig",.source="validation/drive_suspension_rig.py:127-163",.description="Three transmissions, shocks and torque comparison",.slow=true},
    {.id="momentum",.source="validation/system_momentum.py:28-128",.description="Independent whole-system momentum ledger",.slow=true},
    {.id="rider_work",.source="validation/rider_work.py:4-10",.description="Active/passive rider work channels",.slow=false},
    {.id="valid_prefix",.source="validation/plant_prefix.py:7-43",.description="Immutable rejection of the first invalid plant interval",.slow=false},
    {.id="plant_release",.source="validation/plant_release.py:13-18",.description="Independent fail-closed SIL evidence gates",.slow=false},
    {.id="trace_integral",.source="validation/plant_release.py:21-28",.description="Strictly increasing paired trapezoidal integral",.slow=false},
    {.id="interval_integral",.source="validation/fidelity_sweep.py:49-59",.description="Contiguous interval-weighted physical integral",.slow=false},
    {.id="support_evidence",.source="validation/climbing_evidence.py:7-111",.description="Complete-horizon support, activation and scope evidence",.slow=false},
    {.id="coasting_evidence",.source="validation/rider_replay.py:43-87",.description="Automatic coast phases and frozen pedal targets",.slow=false},
    {.id="resume_evidence",.source="validation/rider_replay.py:159-216",.description="Measured coast/resume diagnosis",.slow=false},
    {.id="timing_metrics",.source="validation/climbing_benchmark.py:20-29",.description="Completed-step latency percentiles and throughput",.slow=false},
    {.id="assess_run",.source="validation/antiwheelie_reports.py:7-28",.description="Full episode validity and explicit outcome",.slow=false},
    {.id="paired_summary",.source="validation/antiwheelie_reports.py:31-70",.description="Comparable matched-policy records and exclusions",.slow=false},
    {.id="classify_outcome",.source="validation/reference_release.py:103-150",.description="Invalid-prefix, crash, goal and persistent-condition ordering",.slow=false},
    {.id="resolution_variants",.source="validation/fidelity_sweep.py:25-40",.description="Strict ordered time, road and station axes",.slow=false},
    {.id="persistent_events",.source="validation/fidelity_sweep.py:191-199",.description="Contact transitions confirmed by dwell time",.slow=false},
    {.id="contact_comparison",.source="validation/contact_resolution.py:27-36",.description="Impulse/energy/event convergence criteria",.slow=false},
    {.id="fidelity_compare",.source="validation/fidelity_sweep.py:66-103",.description="Full physical metrics and shared-initial-state convergence",.slow=false},
    {.id="metrics_from_rows",.source="validation/fidelity_sweep.py:202-232",.description="Physical interval metrics with completeness and prefix evidence",.slow=false},
    {.id="select_verified_resolution",.source="validation/reference_release.py:36-52",.description="Only measured candidates passing every axis",.slow=false},
    {.id="release_gates",.source="validation/reference_release.py:22-30",.description="Required numerical reference gates without promotion",.slow=false},
    {.id="release_manifest",.source="validation/reference_release.py:55-100",.description="Hash-verified reference release and measured holdout evidence",.slow=false},
    {.id="scenarios",.source="validation/reference_release.py:153-163",.description="Explicit eighteen-scenario reference matrix definition",.slow=false},
    {.id="rider_replay",.source="validation/rider_replay.py:227-295",.description="One declared isolated replay factor",.slow=true},
    {.id="rider_factor_matrix",.source="validation/rider_factor_matrix.py:7-40",.description="Nine causal rider factors including unsupported control rejection",.slow=true},
    {.id="automatic_coast",.source="validation/rider_replay.py:90-104",.description="Exact fast-profile automatic coast/resume run",.slow=true},
    {.id="fidelity_sweep",.source="validation/fidelity_sweep.py:273-409",.description="Time, road and station convergence from shared initial states",.slow=true},
    {.id="reference_release",.source="validation/reference_release.py:195-289",.description="Reference scenario matrix; unvalidated report",.slow=true},
    {.id="contact_resolution",.source="validation/contact_resolution.py:39-95",.description="Independent contact resolution and density-fit stands",.slow=true},
    {.id="benchmarks",.source="validation/benchmarks.py:163-240",.description="Versioned case registry and convergence gates",.slow=true},
    {.id="climbing_benchmark",.source="validation/climbing_benchmark.py:32-166",.description="Setup, warmup, timing and validity kept separate",.slow=true},
    {.id="antiwheelie_bench",.source="validation/antiwheelie_bench.py:74-133",.description="Source-matched mechanics evidence pipeline",.slow=true},
    {.id="environment",.source="validation/environment.py:31-82",.description="Actual native build lock, engine and provider identity",.slow=false},
    {.id="profile_identity",.source="validation/antiwheelie_bench.py:29-43",.description="Resolved physical profile and source identity",.slow=false},
    {.id="evidence_matches",.source="validation/antiwheelie_bench.py:23-26",.description="All three source/configuration/version identities must exist",.slow=false},
    {.id="source_fingerprint",.source="validation/environment.py:17-28",.description="Sorted length-delimited native source SHA-256",.slow=false}});
bool boolean(const Value& input,std::string_view name,bool fallback=false){const auto* found=find(input,name);return found?io::as_bool(*found,name):fallback;}
Verdict bounded_case(RigResult result){auto checks=check_bounds(result.metrics,result.bounds);const bool nonempty=!io::as_object(result.bounds,"bounds").empty();
    return {.accepted=nonempty&&checks_passed(checks),.evidence=Value{Object{{"metrics",std::move(result.metrics)},{"bounds",std::move(result.bounds)},{"checks",std::move(checks)}}}};}
Verdict report_result(Value result,std::string_view key="passed"){const bool accepted=flag(result,key);return {.accepted=accepted,.evidence=std::move(result)};}
Verdict diagnostic(Value result){return {.accepted=false,.evidence=std::move(result)};}
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
    const nativecfg::SurfaceSpec surface{.name=s.name,.mu_peak=s.mu_peak,.mu_slide=s.mu_slide,.slip_stiffness_per_load=s.slip_stiffness_per_load,.stribeck_speed_mps=s.stribeck_speed_mps};
    const auto slips=numbers(get(config,"slip_ratios"),"slip ratios"),loads=numbers(get(config,"loads_n"),"loads");
    if(slips.empty()||loads.empty())throw std::invalid_argument("tire characterization requires nonempty slip/load axes");Array rows;
    for(double const slip:slips)for(double const load:loads)rows.push_back(steady_brush_force(tire,surface,slip,load,real(config,"speed_mps",2.),real(config,"dt_s",.000625),real(config,"duration_s",1.)));
    return Value{Object{{"surface",value::text(surface_name)},{"cases",Value{std::move(rows)}}}};
}
// One handler per registered id keeps run_case's stack frame under the
// project limit at -O0; bodies are verbatim from the previous if-chain.
double case_dt(const Value& c){return real(c,"dt_s",.000625);}
int case_stations(const Value& c){const auto n=io::as_int64(optional(c,"station_count",value::integer(128)),"station_count");if(n<16||n>std::numeric_limits<int>::max())throw std::invalid_argument("station count outside native domain");return static_cast<int>(n);}
Verdict case_tire_characterization(const Value& c,const std::filesystem::path& base){auto report=tire_characterization(c,base);return {.accepted=true,.evidence=std::move(report)};}
Verdict case_tire_calibration(const Value& c,const std::filesystem::path&){const auto& record=get(c,"record");const auto r=calibration_from_record(record);return {.accepted=true,.evidence=Value{Object{{"record",record},{"source_kind",value::text(r.source_kind)},{"calibration_status",value::text("parameterized_unvalidated")}}}};}
Verdict case_tire_curve(const Value& c,const std::filesystem::path&){const nativecfg::TireTable table{.deflection_m=numbers(get(c,"deflection_m"),"deflection"),.force_n=numbers(get(c,"force_n"),"force")};const contactlaw::TabulatedRadial curve(table);const double damping=nonnegative(real(c,"radial_c_ns_m",0.),"radial damping");Array rows;
    for(const auto& entry:io::as_array(get(c,"samples"),"samples")){const auto elastic=curve.elastic_response(real(entry,"penetration_m"));const auto contact=curve.normal_contact(real(entry,"penetration_m"),real(entry,"rate_mps",0.),damping);
        rows.push_back(Value{Object{{"elastic_force_n",value::real(elastic.first)},{"stored_energy_j",value::real(elastic.second)},{"normal_force_n",value::real(contact.first)},{"contact_stored_energy_j",value::real(contact.second)}}});}
    if(rows.empty())throw std::invalid_argument("curve samples required");return {.accepted=true,.evidence=Value{std::move(rows)}};}
Verdict case_datasets(const Value& c,const std::filesystem::path&){auto payload=validate_dataset(get(c,"dataset"),boolean(c,"allow_synthetic"));return {.accepted=true,.evidence=std::move(payload)};}
Verdict case_dataset_hash(const Value& c,const std::filesystem::path&){return {.accepted=true,.evidence=value::text(dataset_hash(get(c,"dataset")))};}
Verdict case_calibration_report(const Value& c,const std::filesystem::path&){return report_result(calibration_report(get(c,"dataset"),get(c,"predictions"),text(c,"quantity"),get(c,"parameters"),get(c,"bounds"),get(c,"error_budget"),boolean(c,"allow_synthetic")),"holdout_passed");}
Verdict case_holdout_metrics(const Value& c,const std::filesystem::path&){return diagnostic(holdout_metrics(numbers(get(c,"measured"),"measured"),numbers(get(c,"predicted"),"predicted")));}
Verdict case_experiment_acceptance(const Value& c,const std::filesystem::path&){validate_split(get(c,"fit"),get(c,"holdout"));const auto status=accepted_calibration_status(io::as_bool(get(c,"synthetic"),"synthetic"),io::as_bool(get(c,"holdout_passed"),"holdout_passed"),io::as_bool(get(c,"converged"),"converged"),io::as_bool(get(c,"within_scope"),"within_scope"));
    return {.accepted=status=="validated_within_declared_scope",.evidence=Value{Object{{"status",value::text(status)},{"measurement_authentication",Value{false}}}}};}
Verdict case_contact_bounds(const Value& c,const std::filesystem::path&){auto checks=check_bounds(get(c,"metrics"),get(c,"bounds"));const bool accepted=!io::as_object(checks,"checks").empty()&&checks_passed(checks);return {.accepted=accepted,.evidence=std::move(checks)};}
Verdict case_contact_matrix(const Value& c,const std::filesystem::path&){auto rows=contact_matrix(c);bool accepted=!io::as_array(rows,"matrix").empty();for(const auto& row:io::as_array(rows,"matrix"))accepted=accepted&&flag(row,"accepted");return {.accepted=accepted,.evidence=std::move(rows)};}
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
Verdict case_contact_case_vertices(const Value& c,const std::filesystem::path&){Array vertices;for(const auto& vertex:contact_case_vertices(text(c,"case","step"),real(c,"dx_m",.005)))vertices.push_back(value::doubles(vertex));return {.accepted=true,.evidence=Value{std::move(vertices)}};}
Verdict case_shaft_ratio(const Value& c,const std::filesystem::path&){return bounded_case(shaft_ratio_rig(case_dt(c),real(c,"ratio",34./51.),boolean(c,"overrun")));}
Verdict case_drive_suspension(const Value& c,const std::filesystem::path&){return bounded_case(drive_suspension_rig(case_dt(c),text(c,"transmission","geometric_ideal_mid_drive"),real(c,"torque_nm"),real(c,"shock_stroke_m",.015),real(c,"duration_s",.05)));}
Verdict case_drive_suspension_rig(const Value& c,const std::filesystem::path& base){auto config=c;if(!find(config,"output"))value::set(std::get<Object>(config.data),"output",value::text("results/drive_suspension.json"));return report_result(drive_suspension_comparison(config,base));}
Verdict case_momentum(const Value& c,const std::filesystem::path&){std::optional<bool> rider;if(const auto* enabled=find(c,"rider_active"))rider=io::as_bool(*enabled,"rider_active");return bounded_case(momentum_rig(case_dt(c),boolean(c,"active"),text(c,"transmission","geometric_ideal_mid_drive"),rider));}
Verdict case_rider_work(const Value& c,const std::filesystem::path&){return diagnostic(rider_work_ledger(get(c,"work_j")));}
Verdict case_valid_prefix(const Value& c,const std::filesystem::path&){auto report=prefix_report(c);const bool accepted=real(report,"accepted")>0.&&real(report,"accepted")==real(report,"seen")&&io::is_null(get(report,"first_bad"));return {.accepted=accepted,.evidence=std::move(report)};}
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
Verdict case_resolution_variants(const Value& c,const std::filesystem::path&){return {.accepted=true,.evidence=resolution_variants(get(c,"time_steps"),get(c,"road_steps"),get(c,"station_counts"),boolean(c,"include_stations",true))};}
Verdict case_persistent_events(const Value& c,const std::filesystem::path&){return diagnostic(persistent_events(get(c,"rows"),real(c,"persistence_s",.02),boolean(c,"normal_load_only")));}
Verdict case_contact_comparison(const Value& c,const std::filesystem::path&){return report_result(contact_comparison(get(c,"coarse"),get(c,"reference")));}
Verdict case_fidelity_compare(const Value& c,const std::filesystem::path&){return report_result(compare_metrics(get(c,"coarse"),get(c,"reference")));}
Verdict case_metrics_from_rows(const Value& c,const std::filesystem::path&){return diagnostic(metrics_from_rows(get(c,"rows"),real(c,"expected_duration_s"),text(c,"initial_state_sha256")));}
Verdict case_select_verified_resolution(const Value& c,const std::filesystem::path&){auto result=select_verified_resolution(get(c,"comparisons"));const bool accepted=!io::is_null(result);return {.accepted=accepted,.evidence=std::move(result)};}
Verdict case_release_gates(const Value& c,const std::filesystem::path&){const auto [accepted,missing]=release_gates(get(c,"report"));return {.accepted=accepted,.evidence=Value{Object{{"passed",Value{accepted}},{"missing",value::strings(missing)},{"status",value::text(release_level(get(c,"report")))}}}};}
Verdict case_release_manifest(const Value& c,const std::filesystem::path& base){return {.accepted=true,.evidence=write_release_manifest(base/text(c,"output"),get(c,"report"),base/text(c,"physics_profile"),optional(c,"measurement_evidence",Value{Array{}}))};}
Verdict case_scenarios(const Value&,const std::filesystem::path&){return diagnostic(scenario_definitions());}
Verdict case_rider_replay(const Value& c,const std::filesystem::path& base){return report_result(run_replay(c,base),"completed_requested_duration");}
Verdict case_rider_factor_matrix(const Value& c,const std::filesystem::path& base){auto result=rider_factor_matrix(c,base);const bool accepted=flag(result,"baseline_resumed")&&flag(result,"causal_replay_resolved");return {.accepted=accepted,.evidence=std::move(result)};}
Verdict case_automatic_coast(const Value& c,const std::filesystem::path& base){return diagnostic(automatic_coast_case(case_dt(c),real(c,"duration_s",5.),base));}
Verdict case_fidelity_sweep(const Value& c,const std::filesystem::path& base){return report_result(fidelity_sweep(c,base));}
Verdict case_reference_release(const Value& c,const std::filesystem::path& base){return diagnostic(reference_matrix(c,base));}
Verdict case_contact_resolution(const Value& c,const std::filesystem::path& base){return report_result(contact_resolution(c,base));}
Verdict case_benchmarks(const Value& c,const std::filesystem::path& base){return report_result(benchmark_suite(c,base));}
Verdict case_climbing_benchmark(const Value& c,const std::filesystem::path& base){return report_result(climbing_benchmark(c,base));}
Verdict case_antiwheelie_bench(const Value& c,const std::filesystem::path& base){if(text(c,"phase","mechanics")!="mechanics")throw std::invalid_argument("antiwheelie phase must be mechanics");return report_result(mechanics_report(c,base));}
Verdict case_environment(const Value&,const std::filesystem::path& base){return report_result(environment_contract(base),"locked_environment_verified");}
Verdict case_profile_identity(const Value& c,const std::filesystem::path& base){return diagnostic(profile_identity(base/text(c,"physics_profile"),base));}
Verdict case_evidence_matches(const Value& c,const std::filesystem::path&){const bool matched=evidence_matches(get(c,"record"),get(c,"expected"));return {.accepted=matched,.evidence=Value{Object{{"matches",Value{matched}}}}};}
Verdict case_source_fingerprint(const Value& c,const std::filesystem::path& base){return diagnostic(value::text(source_fingerprint(base/text(c,"source_root","native/src"))));}
struct CaseHandler{std::string_view id;Verdict (*run)(const Value&,const std::filesystem::path&);};
constexpr auto handlers=std::to_array<CaseHandler>({
    {.id="tire_characterization",.run=case_tire_characterization},{.id="tire_calibration",.run=case_tire_calibration},{.id="tire_curve",.run=case_tire_curve},
    {.id="datasets",.run=case_datasets},{.id="dataset_hash",.run=case_dataset_hash},{.id="calibration_report",.run=case_calibration_report},
    {.id="holdout_metrics",.run=case_holdout_metrics},{.id="experiment_acceptance",.run=case_experiment_acceptance},{.id="contact_bounds",.run=case_contact_bounds},
    {.id="contact_matrix",.run=case_contact_matrix},{.id="radial_rig",.run=case_radial_rig},{.id="native_radial_rig",.run=case_native_radial_rig},
    {.id="wheel_inertia_rig",.run=case_wheel_inertia_rig},{.id="incline_rig",.run=case_incline_rig},{.id="brake_stand",.run=case_brake_stand},
    {.id="chain_locked_rig",.run=case_chain_locked_rig},{.id="chain_geometry",.run=case_chain_geometry},{.id="compiled_chain_motion",.run=case_compiled_chain_motion},
    {.id="load_transfer",.run=case_load_transfer},{.id="quasistatic_front_load",.run=case_quasistatic_front_load},{.id="grade_feasibility",.run=case_grade_feasibility},
    {.id="distributed_flat",.run=case_distributed_flat},{.id="distributed_step_rig",.run=case_distributed_step_rig},{.id="distributed_incline_rig",.run=case_distributed_incline_rig},
    {.id="distributed_road_trace",.run=case_distributed_road_trace},{.id="contact_case_vertices",.run=case_contact_case_vertices},{.id="shaft_ratio",.run=case_shaft_ratio},
    {.id="drive_suspension",.run=case_drive_suspension},{.id="drive_suspension_rig",.run=case_drive_suspension_rig},{.id="momentum",.run=case_momentum},
    {.id="rider_work",.run=case_rider_work},{.id="valid_prefix",.run=case_valid_prefix},{.id="plant_release",.run=case_plant_release},
    {.id="trace_integral",.run=case_trace_integral},{.id="interval_integral",.run=case_interval_integral},{.id="support_evidence",.run=case_support_evidence},
    {.id="coasting_evidence",.run=case_coasting_evidence},{.id="resume_evidence",.run=case_resume_evidence},{.id="timing_metrics",.run=case_timing_metrics},
    {.id="assess_run",.run=case_assess_run},{.id="paired_summary",.run=case_paired_summary},{.id="classify_outcome",.run=case_classify_outcome},
    {.id="resolution_variants",.run=case_resolution_variants},{.id="persistent_events",.run=case_persistent_events},{.id="contact_comparison",.run=case_contact_comparison},
    {.id="fidelity_compare",.run=case_fidelity_compare},{.id="metrics_from_rows",.run=case_metrics_from_rows},{.id="select_verified_resolution",.run=case_select_verified_resolution},
    {.id="release_gates",.run=case_release_gates},{.id="release_manifest",.run=case_release_manifest},{.id="scenarios",.run=case_scenarios},
    {.id="rider_replay",.run=case_rider_replay},{.id="rider_factor_matrix",.run=case_rider_factor_matrix},{.id="automatic_coast",.run=case_automatic_coast},
    {.id="fidelity_sweep",.run=case_fidelity_sweep},{.id="reference_release",.run=case_reference_release},{.id="contact_resolution",.run=case_contact_resolution},
    {.id="benchmarks",.run=case_benchmarks},{.id="climbing_benchmark",.run=case_climbing_benchmark},{.id="antiwheelie_bench",.run=case_antiwheelie_bench},
    {.id="environment",.run=case_environment},{.id="profile_identity",.run=case_profile_identity},{.id="evidence_matches",.run=case_evidence_matches},
    {.id="source_fingerprint",.run=case_source_fingerprint}});
} // namespace

std::span<const CaseDescription> cases(){
    static const std::vector<CaseDescription> table=[] {std::vector<CaseDescription> rows(operations.begin(),operations.end());
        for(const auto id:benchmark_case_names())rows.push_back({.id=id,.source="validation/benchmarks.py:89-118",.description="Bounded source benchmark case",.slow=true});return rows;}();
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
