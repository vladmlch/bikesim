#include "acceptance.hpp"
#include "tire.hpp"
#include <array>
#include <cctype>
#include <map>
#include <tuple>

namespace bike::validation {
namespace {
constexpr auto required_numerical=std::to_array<std::string_view>({"known_tests_passed","coast_resume_resolved","mechanics_passed","numerics_converged","model_scope_passed"});
Value outcome(std::string_view name,Value reason,const Value& row,bool use_end=false) {
    return Value{Object{{"outcome",value::text(name)},{"reason",std::move(reason)},
        {"time_s",optional(row,use_end?"end_time_s":"time_s",value::real(0.))},
        {"position_m",optional(row,"position_m")},{"control",optional(row,"control",empty_object())}}};
}
bool all_flags(const Value& object) {
    for(const auto& [name,item]:io::as_object(object,"checks")){(void)name;if(!true_value(item))return false;}
    return true;
}
int integer_order(std::string a,std::string b) {
    const bool na=a.starts_with('-'),nb=b.starts_with('-');
    if(na)a.erase(a.begin());if(nb)b.erase(b.begin());
    if(a=="0"&&b=="0")return 0;if(na!=nb)return na?-1:1;
    int result=a.size()<b.size()?-1:a.size()>b.size()?1:a<b?-1:a>b?1:0;
    return na?-result:result;
}
int scalar_order(const Value& a,const Value& b) {
    if(const auto* x=std::get_if<std::string>(&a.data)){
        const auto* y=std::get_if<std::string>(&b.data);if(!y)throw std::invalid_argument("pair keys contain incomparable seed types");
        return *x<*y?-1:*x>*y?1:0;
    }
    const auto* x=std::get_if<io::Number>(&a.data);const auto* y=std::get_if<io::Number>(&b.data);
    if(!x||!y)throw std::invalid_argument("pair keys require comparable string or numeric values");
    if(x->lexeme.find_first_of(".eE")==std::string::npos&&y->lexeme.find_first_of(".eE")==std::string::npos)
        return integer_order(x->lexeme,y->lexeme);
    const double left=io::as_double(a,"pair key"),right=io::as_double(b,"pair key");return left<right?-1:left>right?1:0;
}
struct PairOrder {
    bool operator()(const Value& a,const Value& b)const{
        const auto& x=io::as_array(a,"pair key");const auto& y=io::as_array(b,"pair key");
        for(std::size_t i=0;i<3;++i)if(const int order=scalar_order(x[i],y[i]);order)return order<0;return false;
    }
};
} // namespace

// contact_acceptance.py:12-29. Invalid/missing metrics reject the criterion.
Value check_bounds(const Value& metrics,const Value& bounds) {
    Object checks;
    for(const auto& [name,range]:io::as_object(bounds,"bounds")) {
        const auto& pair=io::as_array(range,name);
        if(pair.size()!=2)throw std::invalid_argument("bound must contain exactly two values");
        const auto lower=io::tagged_double(pair[0],name),upper=io::tagged_double(pair[1],name);
        if(std::isnan(lower)||std::isnan(upper)||lower>upper)throw std::invalid_argument("invalid acceptance bound");
        bool passed=false;
        if(const auto* metric=find(metrics,name)) {
            try {const auto x=io::tagged_double(*metric,name);passed=std::isfinite(x)&&lower<=x&&x<=upper;}
            catch(const std::invalid_argument&){passed=false;}
        }
        checks.emplace_back(name,Value{passed});
    }
    return Value{std::move(checks)};
}
// plant_release.py:13-18. Physical, numerical, and completion gates remain independent.
Value release_decision(const Value& evidence,bool require_completion) {
    Object checks;
    for(const auto name:{"model_valid","numerically_valid","contact_passed","convergence_passed","rider_budget_passed","mass_momentum_passed","transmission_parity_passed","sensor_split_passed"})
        checks.emplace_back(name,Value{flag(evidence,name)});
    if(require_completion)checks.emplace_back("finished",Value{flag(evidence,"finished")});
    Value c{std::move(checks)};
    return Value{Object{{"sil_ready",Value{all_flags(c)}},{"checks",std::move(c)},
        {"calibration_status",value::text("unvalidated")},{"real_world_safety_claim",Value{false}}}};
}
// antiwheelie_reports.py:7-28.
Value assess_run(const Value& summary,double required_duration_s,double minimum_progress_m) {
    positive(required_duration_s,"required duration");nonnegative(minimum_progress_m,"minimum progress");
    const double duration=nonnegative(real(summary,"duration_s",0.),"actual duration");
    const double progress=real(summary,"progress_m",0.);
    std::vector<std::string> reasons;
    if(!flag(summary,"numerically_valid"))reasons.emplace_back("numerical_invalid");
    if(!flag(optional(summary,"model_status",empty_object()),"model_valid"))reasons.emplace_back("model_invalid");
    if(duration+1e-9<required_duration_s)reasons.emplace_back("incomplete_horizon");
    if(progress<minimum_progress_m)reasons.emplace_back("insufficient_progress");
    const auto* terminal=find(summary,"outcome");
    const auto end=terminal&&!io::is_null(*terminal)?io::as_string(*terminal,"outcome"):std::string{};
    if(end.starts_with("crash:"))reasons.emplace_back("crash");
    else if(end!="duration"&&end!="finish")reasons.emplace_back("incomplete_outcome");
    if(flag(summary,"operator_intervention"))reasons.emplace_back("operator_intervention");
    return Value{Object{{"accepted",Value{reasons.empty()}},{"reasons",value::strings(reasons)}}};
}
// antiwheelie_reports.py:31-70. Group keys retain seed type and full plant identity.
Value paired_summary(const Value& records) {
    std::map<Value,std::vector<Value>,PairOrder> groups;
    for(const auto& r:io::as_array(records,"records")) {
        Value key{Array{get(r,"scenario_id"),get(r,"seed"),get(r,"plant_and_inputs_sha256")}};
        groups[std::move(key)].push_back(r);
    }
    Array pairs,exclusions;std::int64_t comparable=0;
    for(const auto& [key,group]:groups) {
        Object policies;std::set<std::string> policy_ids,reasons;std::vector<double> durations;
        for(const auto& record:group) {
            const auto id=text(record,"policy_id");
            if(!policy_ids.insert(id).second)reasons.insert("duplicate_policy");
            const auto& m=get(record,"metrics");value::set(policies,id,m);
            durations.push_back(nonnegative(real(m,"duration_s",0.),"pair duration"));
            if(!flag(m,"numerically_valid"))reasons.insert("numerical_invalid");
            if(!flag(optional(m,"model_status",empty_object()),"model_valid"))reasons.insert("model_invalid");
            if(flag(record,"operator_intervention")||flag(m,"operator_intervention"))reasons.insert("operator_intervention");
            const auto raw=optional(m,"outcome");const auto state=io::is_null(raw)?std::string{}:io::as_string(raw,"outcome");
            if(state=="policy_error"||state=="simulation_error"||state=="operator_stop")reasons.insert("incomplete_run");
        }
        if(!policy_ids.contains("passthrough")||policy_ids.size()<2)reasons.insert("missing_baseline_or_candidate");
        if(!durations.empty()&&*std::ranges::max_element(durations)-*std::ranges::min_element(durations)>1e-8)reasons.insert("different_horizons");
        const auto& parts=io::as_array(key,"pair key");
        Object entry{{"scenario_id",parts[0]},{"seed",parts[1]},{"plant_and_inputs_sha256",parts[2]},
            {"policies",Value{std::move(policies)}},{"comparable",Value{reasons.empty()}}};
        pairs.push_back(Value{entry});
        if(reasons.empty())++comparable;
        else {value::set(entry,"reasons",value::strings(std::vector<std::string>(reasons.begin(),reasons.end())));exclusions.push_back(Value{std::move(entry)});}
    }
    return Value{Object{{"schema_version",value::integer(1)},{"records",records},{"pairs",Value{std::move(pairs)}},
        {"exclusions",Value{std::move(exclusions)}},{"comparable_pair_count",value::integer(comparable)}}};
}
std::pair<bool,std::vector<std::string>> release_gates(const Value& report) {
    (void)io::as_object(report,"release report");std::vector<std::string> missing;
    for(const auto gate:required_numerical)if(!flag(report,gate))missing.emplace_back(gate);
    return {missing.empty(),std::move(missing)};
}
std::string release_level(const Value& report) {
    return release_gates(report).first?"numerically_verified_synthetic":"parameterized_unvalidated";
}
// reference_release.py:36-52. Select only exact measured candidates, without extrapolation.
Value select_verified_resolution(const Value& comparisons) {
    const Value* best=nullptr;
    for(const auto& item:io::as_array(comparisons,"comparisons")) {
        const auto axes=optional(item,"axes_passed",empty_object());
        if(!flag(item,"passed")||keys(axes)!=std::set<std::string>{"time","road","stations"}||!all_flags(axes))continue;
        const double dt=positive(real(item,"dt_s"),"dt_s"),dx=positive(real(item,"dx_m"),"dx_m");
        const auto n=io::as_int64(get(item,"station_count"),"station_count");
        if(n<16)throw std::invalid_argument("invalid measured resolution");
        if(!best||std::tuple(-dt,n,-dx)<std::tuple(-real(*best,"dt_s"),io::as_int64(get(*best,"station_count"),"station_count"),-real(*best,"dx_m")))best=&item;
    }
    return best?*best:Value{};
}
Value write_release_manifest(const std::filesystem::path& path,const Value& report,
                            const std::filesystem::path& physics_profile,const Value& measurement_evidence) {
    const auto [ok,reasons]=release_gates(report);
    if(!ok)throw std::invalid_argument("reference release blocked: "+value::join(reasons,", "));
    const auto& evidence=get(report,"evidence_files");
    for(const auto gate:required_numerical) {
        const auto& rows=io::as_array(get(evidence,gate),gate);
        if(rows.empty())throw std::invalid_argument("release needs hashed evidence for "+std::string(gate));
        for(const auto& row:rows) {
            const std::filesystem::path file=text(row,"path");
            if(!std::filesystem::is_regular_file(file)||file_hash(file)!=text(row,"sha256"))throw std::invalid_argument("release evidence file/hash mismatch");
        }
    }
    if(!std::filesystem::is_regular_file(physics_profile))throw std::invalid_argument("physics profile is missing");
    std::string status="numerically_verified_synthetic";Array calibrations;
    for(const auto& entry:io::as_array(measurement_evidence,"measurement evidence")) {
        const auto synthetic=optional(report,"synthetic");
        if(!false_value(synthetic))throw std::invalid_argument("real release must declare actual non-synthetic experiments");
        const auto payload=validate_dataset(get(entry,"dataset"));
        const auto& args=get(entry,"arguments");
        const auto result=calibration_report(payload,get(entry,"predictions"),text(args,"output"),get(args,"parameters"),get(args,"bounds"),get(args,"error_budget"));
        if(is_synthetic(payload)||!flag(result,"holdout_passed"))throw std::invalid_argument("measured holdout has not passed");
        if(text(payload,"bike_config_hash")!=text(report,"configuration_sha256"))throw std::invalid_argument("holdout measured configuration does not match the release");
        calibrations.push_back(Value{Object{{"dataset_id",get(payload,"dataset_id")},{"dataset_hash",value::text(dataset_hash(payload))},{"holdout",result}}});
        status=accepted_calibration_status(false,true,true,true);
    }
    auto stem=path.stem().string();
    std::ranges::transform(stem,stem.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    if(stem.find("validated")!=std::string::npos&&status!="validated_within_declared_scope")throw std::invalid_argument("synthetic evidence cannot produce a validated file");
    Object gates;for(const auto gate:required_numerical)gates.emplace_back(std::string(gate),get(report,gate));
    for(const auto name:{"configuration_sha256","source_sha256","versions","declared_scope"}) {
        const auto* item=find(report,name);
        if(!item||io::is_null(*item)||io::canonical_json(*item)=="{}"||io::canonical_json(*item)=="\"\"")throw std::invalid_argument("release requires configuration/source versions and explicit scope");
    }
    Value manifest{Object{{"schema_version",value::integer(1)},{"status",value::text(status)},{"gates",Value{std::move(gates)}},
        {"physics_profile",value::text(physics_profile.string())},{"physics_profile_sha256",value::text(file_hash(physics_profile))},
        {"configuration_sha256",get(report,"configuration_sha256")},{"source_sha256",get(report,"source_sha256")},
        {"versions",get(report,"versions")},{"declared_scope",get(report,"declared_scope")},{"evidence_files",evidence},
        {"calibration_datasets",Value{std::move(calibrations)}},{"scenarios",optional(report,"scenarios",empty_object())}}};
    write_report(path,manifest);return manifest;
}
// reference_release.py:103-150. Preserve precedence, dwell, and invalid-prefix rejection.
Value classify_outcome(const Value& rows,std::optional<double> goal,bool crashed,
                       std::optional<std::string> numerical_error,bool duration_reached) {
    const auto& entries=io::as_array(rows,"rows");const auto last=entries.empty()?empty_object():entries.back();
    if(numerical_error)return outcome("numerical_failure",value::text(*numerical_error),last,true);
    for(const auto& row:entries) {
        const auto status=optional(row,"model_status",empty_object());
        if(false_value(optional(status,"numerically_valid")))return outcome("numerical_failure",value::text("energy quality gate failed"),row);
        if(false_value(optional(status,"model_valid")))return outcome("model_invalid",optional(status,"first_model_violation"),row);
    }
    if(crashed)return outcome("crash",value::text("runtime crash detector"),last,true);
    if(goal)for(const auto& row:entries)if(real(row,"position_m")>=*goal)return outcome("completed",value::text("declared distance reached"),row);
    std::map<std::string,const Value*> starts;
    for(const auto& row:entries) {
        const bool loaded=real(row,"front_load_n")+real(row,"rear_load_n")>10.;
        const bool effort=real(row,"motor_torque_nm",0.)+std::max(real(row,"human_torque_nm",0.),0.)>5.;
        const bool slow=std::abs(real(row,"speed_mps"))<.1;
        const std::array conditions{
            std::pair{"physical_stall",real(row,"time_s")>=2.&&loaded&&effort&&slow&&std::abs(real(row,"crank_rad_s",0.))<.3&&std::abs(real(row,"rear_slip_mps",0.))<.2},
            std::pair{"loss_of_traction",loaded&&effort&&slow&&std::abs(real(row,"rear_slip_mps",0.))>.5},
            std::pair{"contact_loss",real(row,"front_load_n")<=5.&&real(row,"rear_load_n")<=5.}};
        Array candidates;
        for(const auto& [name,present]:conditions) {
            if(!present){starts.erase(name);continue;}
            starts.try_emplace(name,&row);const auto& first=*starts.at(name);
            const double persistence=std::string_view(name)=="contact_loss"?1.:.75;
            if(real(row,"end_time_s")-real(first,"time_s")+1e-12<persistence)continue;
            auto result=outcome(name,value::text("persistent measured condition"),first);
            auto& o=std::get<Object>(result.data);value::set(o,"confirmed_at_s",get(row,"end_time_s"));
            Object evidence;for(const auto key:{"speed_mps","crank_rad_s","rear_slip_mps","front_load_n","rear_load_n","motor_torque_nm","human_torque_nm"})evidence.emplace_back(key,optional(first,key));
            value::set(o,"evidence",Value{std::move(evidence)});candidates.push_back(std::move(result));
        }
        if(!candidates.empty())return *std::ranges::min_element(candidates,{},[](const Value& item){return real(item,"time_s");});
    }
    return outcome("duration_limit",value::text(duration_reached?"declared observation duration ended":"no confirmed terminal condition"),last,true);
}
Value scenario_definitions() {
    Array scenarios;
    const auto add=[&](std::string name,double duration,std::optional<double> goal){
        scenarios.push_back(Value{Object{{"name",value::text(name)},{"duration_s",value::real(duration)},{"goal_x_m",goal?value::real(*goal):Value{}}}});
    };
    add("flat",5.,5.);
    for(int percent:{5,12,20,28,35}) {
        const double grade=static_cast<double>(percent)/100.;add("grade_"+std::to_string(percent)+"pct",8.,5.);
        auto& entry=std::get<Object>(scenarios.back().data);value::set(entry,"grade",value::real(grade));value::set(entry,"angle_rad",value::real(std::atan(grade)));
    }
    for(const auto name:{"bump","crest","step_4cm","step_6cm","two_support"})add(name,3.,std::string_view(name)=="crest"?7.:6.);
    add("recontact",1.,{});add("coast_resume",8.,{});
    for(const auto name:{"seated","standing","forward","rearward"}) {
        add(std::string("posture_")+name,3.,{});value::set(std::get<Object>(scenarios.back().data),"posture",value::text(name));
    }
    add("extreme_100m",90.,100.);return Value{std::move(scenarios)};
}
} // namespace bike::validation
