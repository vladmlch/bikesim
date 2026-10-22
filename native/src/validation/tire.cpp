#include "tire.hpp"
#include "../contact/laws.hpp"
#include <array>
#include <cctype>
#include <chrono>
#include <tuple>

namespace bike::validation {
namespace {
void require_keys(const Value& object, const std::set<std::string>& expected, std::string_view message) {
    if (keys(object) != expected) throw std::invalid_argument(std::string(message));
}
// NOLINTNEXTLINE(misc-no-recursion) recursive JSON value tree walk
void finite_json(const Value& item) {
    if (const auto* n = std::get_if<io::Number>(&item.data)) (void)io::number_double(*n, "metadata");
    else if (const auto* a = std::get_if<Array>(&item.data)) for (const auto& v : *a) finite_json(v);
    else if (const auto* o = std::get_if<Object>(&item.data)) for (const auto& [k,v] : *o) { (void)k; finite_json(v); }
    else if (std::holds_alternative<io::Tagged>(item.data) || std::holds_alternative<io::Bytes>(item.data))
        throw std::invalid_argument("metadata must be finite JSON data");
}
Value units(std::initializer_list<std::pair<std::string,std::string>> entries) {
    Object result;
    for (const auto& [k,v] : entries) result.emplace_back(k, value::text(v));
    return Value{std::move(result)};
}
std::vector<std::pair<std::string, Value>> rig_units() {
    return {
        {"tire", units({{"force","N"},{"deflection","m"}})},
        {"damper", units({{"force","N"},{"velocity","m/s"},{"stroke","m"}})},
        {"motor", units({{"torque","N*m"},{"angular_speed","rad/s"},{"electrical_power","W"}})},
        {"axle_loads", units({{"front_load_n","N"},{"rear_load_n","N"},{"slope_rad","rad"}})},
        {"rider_pose", units({{"pelvis_x_m","m"},{"pelvis_z_m","m"},{"torso_pitch_rad","rad"},{"com_x_m","m"},{"com_z_m","m"}})},
        {"suspension_kinematics", units({{"rear_travel_m","m"},{"shock_stroke_m","m"}})},
        {"full_bike_run", units({{"time_s","s"},{"pitch_rad","rad"},{"pitch_rate_rad_s","rad/s"},{"speed_mps","m/s"},
             {"fork_travel_m","m"},{"shock_stroke_m","m"},{"motor_torque_nm","N*m"},{"crank_rad_s","rad/s"}})}
    };
}
std::set<std::string> experiment_ids(const Value& sequence, bool canonical) {
    const auto list = strings(sequence,"experiment identifiers");
    std::set<std::string> result;
    for (const auto& id : list) {
        if (trim(id).empty() || (canonical && trim(id) != id))
            throw std::invalid_argument("experiment identifiers must be nonempty canonical strings");
        if (!result.insert(id).second) throw std::invalid_argument("duplicate experiment identifier");
    }
    return result;
}
void bicycle_metadata(const Value& p, std::string_view kind) {
    validate_split(get(get(p,"split"),"fit"),get(get(p,"split"),"holdout"));
    for (const auto key : {"rider_config_hash","sensor_calibration","synchronisation_method","quantity_provenance","experiment_metadata"})
        if (!find(p,key)) throw std::invalid_argument("missing bicycle/rider measurement metadata");
    identifier(get(p,"rider_config_hash"),"rider configuration hash");
    identifier(get(p,"synchronisation_method"),"synchronisation method");
    const auto& u=get(p,"units"); const auto& provenance=get(p,"quantity_provenance");
    if (keys(u)!=keys(provenance)) throw std::invalid_argument("quantity provenance required for every channel");
    const auto& calibration=get(p,"sensor_calibration");
    (void)io::as_object(calibration,"sensor calibration");
    for (const auto& [channel,entry] : io::as_object(provenance,"quantity provenance")) {
        const auto type=text(entry,"kind");
        if (type!="sensor" && type!="estimate") throw std::invalid_argument("channel must be explicitly sensor or estimate");
        identifier(get(entry,"method"),"channel method");
        if (type=="sensor") identifier(get(calibration,channel),"sensor calibration identifier");
        else nonnegative(real(entry,"estimate_uncertainty"),"estimate uncertainty");
    }
    auto ids=experiment_ids(get(get(p,"split"),"fit"),true);
    const auto other=experiment_ids(get(get(p,"split"),"holdout"),true); ids.insert(other.begin(),other.end());
    const auto& experiments=get(p,"experiment_metadata");
    if (keys(experiments)!=ids) throw std::invalid_argument("metadata must cover whole experiments exactly");
    for (const auto& [id, conditions] : io::as_object(experiments,"experiment metadata")) {
        for (const auto key : {"bike_mass_kg","rider_mass_kg","front_pressure_pa_gauge","rear_pressure_pa_gauge","temperature_c","front_teeth","rear_teeth","bike_config_hash","rider_config_hash"})
            if (!find(conditions,key)) throw std::invalid_argument("incomplete experiment mass/pressure/temperature/gear metadata");
        for (const auto name : {"bike_mass_kg","rider_mass_kg","front_pressure_pa_gauge","rear_pressure_pa_gauge"}) {
            const auto x=nonnegative(real(conditions,name),name);
            if (std::string_view(name)!="rider_mass_kg" && x==0.) throw std::invalid_argument("positive mass and pressure required");
        }
        if (real(conditions,"temperature_c")<=-273.15) throw std::invalid_argument("temperature is below absolute zero");
        for (const auto name : {"front_teeth","rear_teeth"})
            if (io::as_int64(get(conditions,name),name)<=0) throw std::invalid_argument("gear tooth counts must be positive integers");
        for (const auto name : {"bike_config_hash","rider_config_hash"}) {
            identifier(get(conditions,name),name);
            if (text(conditions,name)!=text(p,name)) throw std::invalid_argument("mixed configuration hashes require separate datasets");
        }
        if (kind=="full_bike_run") {
            std::optional<double> last;
            for (const auto& row : io::as_array(get(p,"samples"),"samples")) if (text(row,"experiment_id")==id) {
                const auto time=real(row,"time_s");
                if (last && time<=*last) throw std::invalid_argument("full-bike times must increase within each experiment");
                last=time;
            }
        }
    }
}
} // namespace

// physics/tire_calibration.py:16-84, tire_curve.py:17-92. The production
// TabulatedRadial owns the identical slopes/integrals; no second material law.
contactlaw::TabulatedRadial TireCalibration::curve() const { return contactlaw::TabulatedRadial(*material.table); } // NOLINT(bugprone-unchecked-optional-access) material.table is always populated by calibration_from_record
bool TireCalibration::is_load_in_valid_range(double load_n) const {
    scalar(load_n,"load_n"); return material.valid_load_range_n[0]<=load_n && load_n<=material.valid_load_range_n[1];
}
TireCalibration calibration_from_record(const Value& record) {
    require_keys(record,{"dataset_id","source_kind","tyre_label","radius_m","inflated_width_m","rim_internal_width_m",
        "pressure_pa_gauge","radial_c_ns_m","deflection_m","force_n","valid_load_range_n","provenance"},
        "calibration fields must match the version-1 schema exactly");
    TireCalibration r;
    r.material.table=nativecfg::TireTable{.deflection_m=numbers(get(record,"deflection_m"),"deflection"),.force_n=numbers(get(record,"force_n"),"elastic force")};
    (void)r.curve();
    r.material.radial_c_ns_m=nonnegative(real(record,"radial_c_ns_m"),"radial_c_ns_m");
    r.material.pressure_pa_gauge=nonnegative(real(record,"pressure_pa_gauge"),"pressure_pa_gauge");
    r.material.provenance=identifier(get(record,"provenance"),"tire curve provenance");
    const auto limits=numbers(get(record,"valid_load_range_n"),"valid load");
    if (limits.size()!=2 || !(0.<=limits[0] && limits[0]<limits[1] && limits[1]<=r.material.table->force_n.back()))
        throw std::invalid_argument("valid load range must fit within the supplied force table");
    r.material.valid_load_range_n={limits[0],limits[1]};
    r.dataset_id=identifier(get(record,"dataset_id"),"dataset_id");
    r.tyre_label=identifier(get(record,"tyre_label"),"tyre_label");
    r.source_kind=text(record,"source_kind");
    if (r.source_kind!="synthetic" && r.source_kind!="measured") throw std::invalid_argument("source_kind must describe input data provenance");
    r.radius_m=positive(real(record,"radius_m"),"radius_m");
    r.inflated_width_m=positive(real(record,"inflated_width_m"),"inflated_width_m");
    r.rim_internal_width_m=positive(real(record,"rim_internal_width_m"),"rim_internal_width_m");
    return r;
}
TireCalibration load_calibration(const std::filesystem::path& path) { return calibration_from_record(read_json(path)); }

Value steady_brush_force(const nativecfg::TireParams& tire,const nativecfg::SurfaceSpec& surface,
                        double slip_ratio,double load_n,double speed_mps,double dt_s,double duration_s) {
    scalar(slip_ratio,"slip ratio");positive(load_n,"normal load");positive(speed_mps,"rolling speed");
    const auto count=steps(duration_s,dt_s);
    positive(tire.tangent_k_n_m,"tangent stiffness");positive(tire.relaxation_length_m,"relaxation length");
    nonnegative(tire.mu,"tire friction");positive(surface.mu_peak,"peak friction");positive(surface.mu_slide,"sliding friction");
    positive(surface.slip_stiffness_per_load,"surface slip stiffness");
    if (surface.mu_slide>surface.mu_peak) throw std::invalid_argument("sliding friction exceeds peak");
    positive(surface.stribeck_speed_mps,"Stribeck speed");
    const double u=-slip_ratio*speed_mps;
    const double ratio=std::abs(u)/surface.stribeck_speed_mps;
    const double mu=std::min(tire.mu,surface.mu_slide+(surface.mu_peak-surface.mu_slide)*std::exp(-ratio));
    double xi=0.,loss=0.,force=0.;
    for (std::size_t i=0;i<count;++i) {
        const auto [next,f,step_loss]=contactlaw::brush_step(xi,u,speed_mps,load_n,tire.tangent_k_n_m,mu,tire.relaxation_length_m,dt_s);
        xi=next;force=f;loss+=step_loss;
    }
    for (double const x : {xi,force,loss}) if (!std::isfinite(x)) throw std::invalid_argument("nonfinite tire characterization");
    return Value{Object{{"slip_ratio",value::real(slip_ratio)},{"slip_mps",value::real(u)},{"force_n",value::real(force)},
        {"mu_used",value::real(mu)},{"normal_load_n",value::real(load_n)},{"loss_j",value::real(loss)},
        {"stored_energy_j",value::real(.5*tire.tangent_k_n_m*xi*xi)},
        {"configured_ckappa_n",value::real(tire.tangent_k_n_m*tire.relaxation_length_m)},
        {"surface_ckappa_per_load_metadata",value::real(surface.slip_stiffness_per_load)}}};
}

void validate_split(const Value& fit,const Value& holdout) {
    const auto a=experiment_ids(fit,true),b=experiment_ids(holdout,true);
    if (a.empty()||b.empty()) throw std::invalid_argument("fit and holdout experiments are both required as lists");
    for (const auto& id:a) if (b.contains(id)) throw std::invalid_argument("fit/holdout leakage at experiment level");
}
bool is_synthetic(const Value& payload) {
    bool synthetic=false;
    if (const auto* item=find(payload,"synthetic")) synthetic=io::as_bool(*item,"synthetic declaration");
    auto source=trim(text(payload,"source"));
    std::ranges::transform(source,source.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    return synthetic || source.starts_with("synthetic") || source.starts_with("simulation:") || source.starts_with("fixture:");
}
std::string_view accepted_calibration_status(bool synthetic,bool holdout,bool converged,bool scope) {
    return synthetic||!holdout||!converged||!scope ? "parameterized_unvalidated" : "validated_within_declared_scope";
}
std::string dataset_hash(const Value& payload) {
    finite_json(payload);
    // datasets.py uses json.dumps(..., ensure_ascii=True), including Python
    // float repr after JSON decoding. The recording codec deliberately uses
    // UTF-8; reproduce this dataset-specific hashing boundary explicitly.
    // NOLINTNEXTLINE(misc-no-recursion) recursive JSON value normalizer
    const auto normalize=[](const auto& self,const Value& input)->Value {
        if(const auto* n=std::get_if<io::Number>(&input.data)) {
            if(n->lexeme.find_first_of(".eE")!=std::string::npos)return value::real(io::as_double(input,"dataset number"));
            return n->lexeme=="-0"?value::integer(0):input;
        }
        if(const auto* a=std::get_if<Array>(&input.data)){Array out;for(const auto& x:*a)out.push_back(self(self,x));return Value{std::move(out)};}
        if(const auto* o=std::get_if<Object>(&input.data)){Object out;for(const auto& [k,x]:*o)out.emplace_back(k,self(self,x));return Value{std::move(out)};}
        return input;
    };
    const auto utf8=io::canonical_json(normalize(normalize,payload));std::string ascii;
    const auto escape=[&](std::uint32_t cp){constexpr std::string_view hex{"0123456789abcdef"};ascii+="\\u";for(unsigned const shift:{12U,8U,4U,0U})ascii+=hex[(cp>>shift)&15U];};
    for(std::size_t i=0;i<utf8.size();) {
        const auto first=static_cast<unsigned char>(utf8[i++]);if(first<128){ascii+=static_cast<char>(first);continue;}
        const unsigned width=first<224?2U:first<240?3U:4U;std::uint32_t cp=first&((1U<<(7U-width))-1U);
        for(unsigned j=1;j<width;++j)cp=(cp<<6U)|(static_cast<unsigned char>(utf8.at(i++))&63U);
        if(cp<=65535U)escape(cp);else{cp-=65536U;escape(55296U+(cp>>10U));escape(56320U+(cp&1023U));}
    }
    return hash_text(ascii);
}
Value validate_dataset(const Value& p,bool allow_synthetic) {
    for (const auto name:{"dataset_id","source","measured_at","units","bike_config_hash","sensor_uncertainty","conditions","samples","split"})
        if (!find(p,name)) throw std::invalid_argument("missing dataset metadata");
    const auto& split=get(p,"split");require_keys(split,{"fit","holdout"},"invalid experiment split");
    const auto fit=experiment_ids(get(split,"fit"),false),holdout=experiment_ids(get(split,"holdout"),false);
    if (fit.empty()||holdout.empty()) throw std::invalid_argument("fit and holdout experiments are both required");
    for (const auto& id:fit) if (holdout.contains(id)) throw std::invalid_argument("fit/holdout experiment overlap");
    for (const auto name:{"dataset_id","source","measured_at","bike_config_hash"}) identifier(get(p,name),name);
    auto stamp=text(p,"measured_at");const bool datetime=stamp.find('T')!=std::string::npos;
    if(datetime)for(std::size_t i=0;(i=stamp.find('Z',i))!=std::string::npos;i+=6)stamp.replace(i,1,"+00:00");
    try{(void)io::recording_json(Value{io::Tagged{.kind=datetime?"datetime":"date",.fields={{"value",stamp}}}});}
    catch(const std::invalid_argument&){throw std::invalid_argument("measured_at must be an ISO date or datetime");}
    const bool synthetic=is_synthetic(p);
    if (synthetic&&!allow_synthetic) throw std::invalid_argument("synthetic data cannot validate a measured model");
    const auto& u=get(p,"units");std::string kind;
    for (const auto& [candidate,expected]:rig_units()) if (io::canonical_json(u)==io::canonical_json(expected)) kind=candidate;
    if (kind.empty()) throw std::invalid_argument("unsupported or incomplete rig units; no implicit unit conversion");
    if (find(p,"kind") && text(p,"kind")!=kind) throw std::invalid_argument("rig kind conflicts with its units");
    const auto& uncertainty=get(p,"sensor_uncertainty");
    if (keys(u)!=keys(uncertainty)) throw std::invalid_argument("sensor uncertainty required for every measured quantity");
    for (const auto& [name,x]:io::as_object(uncertainty,"sensor uncertainty")) nonnegative(io::as_double(x,name),"sensor uncertainty "+name);
    const auto& conditions=get(p,"conditions");
    if (io::as_object(conditions,"conditions").empty()) throw std::invalid_argument("experiment conditions are required");
    if ((kind=="tire"&&!find(conditions,"pressure_pa_gauge")) || (!synthetic&&!find(conditions,"temperature_c")) || (!synthetic&&kind=="motor"&&!find(conditions,"voltage_v")))
        throw std::invalid_argument("incomplete experiment conditions");
    if (find(conditions,"temperature_c")&&real(conditions,"temperature_c")<=-273.15) throw std::invalid_argument("temperature is below absolute zero");
    for (const auto name:{"pressure_pa_gauge","voltage_v"}) if(find(conditions,name)) nonnegative(real(conditions,name),name);
    const auto& samples=io::as_array(get(p,"samples"),"samples");
    if(samples.empty()) throw std::invalid_argument("nonempty samples required");
    auto expected=keys(u);expected.insert("experiment_id");std::set<std::string> ids;
    for(const auto& row:samples) {
        if(keys(row)!=expected) throw std::invalid_argument("sample columns must match the declared units exactly");
        ids.insert(identifier(get(row,"experiment_id"),"experiment_id"));
        for(const auto& [name,unit]:io::as_object(u,"units")) {(void)unit;scalar(real(row,name),"measurement "+name);}
    }
    if(kind=="axle_loads"||kind=="rider_pose"||kind=="suspension_kinematics"||kind=="full_bike_run") bicycle_metadata(p,kind);
    auto all=fit;all.insert(holdout.begin(),holdout.end());
    if(all!=ids) throw std::invalid_argument("every experiment must have exactly one split");
    finite_json(p);return p;
}
Value holdout_metrics(std::span<const double> measured,std::span<const double> predicted) {
    if(measured.empty()||measured.size()!=predicted.size()) throw std::invalid_argument("invalid holdout vectors");
    std::vector<double> errors,squares;double peak=0.;
    for(std::size_t i=0;i<measured.size();++i) {
        scalar(measured[i],"measured holdout");scalar(predicted[i],"predicted holdout");
        const double error=predicted[i]-measured[i];errors.push_back(error);squares.push_back(error*error);peak=std::max(peak,std::abs(error));
    }
    const double rmse=std::sqrt(mean(squares));
    if(!std::isfinite(rmse)) throw std::invalid_argument("holdout error overflow");
    return Value{Object{{"rmse",value::real(rmse)},{"max_abs",value::real(peak)},{"bias",value::real(mean(errors))}}};
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
Value calibration_report(const Value& payload,const Value& predictions,std::string_view output,
                         const Value& parameters,const Value& bounds,const Value& budget,bool allow_synthetic) {
    const auto data=validate_dataset(payload,allow_synthetic);
    if(!find(get(data,"units"),output)) throw std::invalid_argument("output quantity lacks units");
    if(keys(parameters).empty()||keys(parameters)!=keys(bounds)) throw std::invalid_argument("every parameter needs declared bounds");
    for(const auto& [name,p]:io::as_object(parameters,"parameters")) {
        const double x=io::as_double(p,name);const auto limits=numbers(get(bounds,name),name);
        if(limits.size()!=2||limits[0]>=limits[1]||x<limits[0]||x>limits[1]) throw std::invalid_argument("parameter lies outside its declared bound");
    }
    require_keys(budget,{"budget_id","rmse","max_abs","bias_abs"},"predeclared budget ID and all three error limits are required");
    identifier(get(budget,"budget_id"),"error budget");
    for(const auto name:{"rmse","max_abs","bias_abs"}) nonnegative(real(budget,name),name);
    const auto fit=experiment_ids(get(get(data,"split"),"fit"),false), holdout=experiment_ids(get(get(data,"split"),"holdout"),false);
    auto ids=fit;ids.insert(holdout.begin(),holdout.end());
    if(keys(predictions)!=ids) throw std::invalid_argument("predictions must cover every fit/holdout experiment exactly");
    Object experiments;bool passed=true;
    for(const auto& id:ids) {
        std::vector<double> measured;
        for(const auto& row:io::as_array(get(data,"samples"),"samples")) if(text(row,"experiment_id")==id) measured.push_back(real(row,output));
        const auto m=holdout_metrics(measured,numbers(get(predictions,id),id));
        const bool ok=real(m,"rmse")<=real(budget,"rmse")&&real(m,"max_abs")<=real(budget,"max_abs")&&std::abs(real(m,"bias"))<=real(budget,"bias_abs");
        if(holdout.contains(id)&&!ok)passed=false;
        experiments.emplace_back(id,Value{Object{{"split",value::text(fit.contains(id)?"fit":"holdout")},{"metrics",m},{"passed",Value{ok}},{"sample_count",value::integer(static_cast<std::int64_t>(measured.size()))}}});
    }
    return Value{Object{{"dataset_id",get(data,"dataset_id")},{"dataset_hash",value::text(dataset_hash(data))},{"output",value::text(output)},
        {"unit",get(get(data,"units"),output)},{"source",get(data,"source")},{"parameters",parameters},{"bounds",bounds},
        {"sensor_uncertainty",get(data,"sensor_uncertainty")},{"conditions",get(data,"conditions")},{"error_budget",budget},
        {"error_budget_hash",value::text(dataset_hash(budget))},{"experiments",Value{std::move(experiments)}},{"holdout_passed",Value{passed}},
        {"calibration_status",value::text(passed&&!is_synthetic(data)?"measured_holdout_passed":"parameterized_unvalidated")}}};
}
} // namespace bike::validation
