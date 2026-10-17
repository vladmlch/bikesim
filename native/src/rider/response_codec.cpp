#include "response_codec.hpp"
#include "legacy_diagnostics.hpp"
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace bike::rider {
namespace {
using io::Value;
Value real(double x){if(!std::isfinite(x))throw std::invalid_argument("allocation diagnostics must be finite");std::array<char,32>b{};auto r=std::to_chars(b.data(),std::to_address(b.end()),x);if(r.ec!=std::errc())throw std::runtime_error("allocation float encode");std::string s(b.data(),r.ptr);if(s.find_first_of(".eE")==std::string::npos)s+=".0";return Value{io::Number{std::move(s)}};}
Value values(std::span<const double> v){Value::Array a;for(double x:v)a.push_back(real(x));return Value{std::move(a)};}
Value optional(std::optional<double> v){return v?real(*v):Value{};}
Value residual_value(double v){return std::isinf(v)&&v>0.?Value{io::Tagged{"float64",{{"value","+inf"}}}}:real(v);}
const Value& member(const Value::Object& o,std::string_view key,std::string_view path){const auto*v=io::find_member(o,key);if(!v)throw std::invalid_argument(std::string(path)+"."+std::string(key)+": missing required key");return *v;}
double number(const Value& v,std::string_view path){return io::as_double(v,path);}
std::size_t count(const Value& v,std::string_view path){const auto n=io::as_int64(v,path);if(n<0||std::cmp_greater(n,std::numeric_limits<std::size_t>::max()))throw std::invalid_argument(std::string(path)+": invalid count");return static_cast<std::size_t>(n);}
bool boolean(const Value& v,std::string_view path){const auto* b=std::get_if<bool>(&v.data);if(!b)throw std::invalid_argument(std::string(path)+": expected bool");return *b;}
std::vector<double> vector(const Value& v,std::string_view path){const auto*a=std::get_if<Value::Array>(&v.data);if(!a)throw std::invalid_argument(std::string(path)+": expected ordered sequence");std::vector<double> out;for(const auto&x:*a)out.push_back(number(x,path));return out;}
}
ResponseAllocation decode_response_allocation(const Value& value,std::string_view path) {
    const auto* object=std::get_if<Value::Object>(&value.data);if(!object)throw std::invalid_argument(std::string(path)+": expected dict");const auto&o=*object;
    static const std::set<std::string_view> allowed{
        "invalid_controller","feasible","violation","grip_branch","effort_scale","crank_torque_target_nm","solution_crank_torque_nm","saddle_normal_lower_bound_n",
        "solution_tau","solution_qddot","solution_base_qacc","solution_power","solution_wrenches","solution_moments","solution_constraint_forces","solution_excitation_nm","solution_activation_state_nm","solution_active_lower_nm","solution_active_upper_nm",
        "dynamics_model","constraint_modes_tried","constraint_mode_search_complete","linear_mode_rejections","grip_radius_guard_fraction","allocation_force_guard_n","allocation_moment_guard_nm","solution_qpos","solution_qvel","solution_time_s","numeric_identity"};
    for(const auto&[key,item]:o){(void)item;if(!allowed.contains(key))throw std::invalid_argument(std::string(path)+"."+key+": unknown key");}
    const auto get=[&](std::string_view key)->const Value&{return member(o,key,path);};
    const auto maybe=[&](std::string_view key)->std::optional<double>{const auto*v=io::find_member(o,key);return !v||io::is_null(*v)?std::nullopt:std::optional<double>(number(*v,path));};
    ResponseAllocation out;out.allocation.feasible=boolean(get("feasible"),path);out.allocation.violation=io::tagged_double(get("violation"),path);
    if(std::isnan(out.allocation.violation)||out.allocation.violation<0.||out.allocation.feasible!=(out.allocation.violation<=1e-7))throw std::invalid_argument(std::string(path)+": inconsistent allocation feasibility");
    if(const auto*v=io::find_member(o,"invalid_controller");v&&boolean(*v,path)==out.allocation.feasible)throw std::invalid_argument(std::string(path)+": inconsistent invalid_controller");
    if(o.size()==3&&io::find_member(o,"invalid_controller")) {
        if(!out.allocation.feasible||out.allocation.violation!=0.)throw std::invalid_argument(std::string(path)+": disabled controller must be feasible with zero residual");
        out.disabled=true;return out;
    }
    const auto*b=std::get_if<Value::Array>(&get("grip_branch").data);if(!b||b->size()!=2)throw std::invalid_argument(std::string(path)+": grip branch width");out.branch={boolean((*b)[0],path),boolean((*b)[1],path)};
    out.effort_scale=number(get("effort_scale"),path);out.saddle_normal_lower_bound_n=number(get("saddle_normal_lower_bound_n"),path);out.predicted_crank_torque_nm=maybe("solution_crank_torque_nm");out.crank_torque_target_nm=maybe("crank_torque_target_nm");
    out.torque=vector(get("solution_tau"),path);out.rider_acceleration=vector(get("solution_qddot"),path);out.base_acceleration=vector(get("solution_base_qacc"),path);out.power=vector(get("solution_power"),path);out.wrenches=vector(get("solution_wrenches"),path);out.excitation=vector(get("solution_excitation_nm"),path);out.activation_state=vector(get("solution_activation_state_nm"),path);out.lower_torque_nm=vector(get("solution_active_lower_nm"),path);out.upper_torque_nm=vector(get("solution_active_upper_nm"),path);
    for(const auto*v:{&out.power,&out.excitation,&out.activation_state,&out.lower_torque_nm,&out.upper_torque_nm})if(v->size()!=out.torque.size())throw std::invalid_argument(std::string(path)+": actuator vector width mismatch");
    for(std::size_t i=0;i<out.torque.size();++i)if(out.lower_torque_nm[i]>out.upper_torque_nm[i])throw std::invalid_argument(std::string(path)+": empty actuator bounds");
    if(const auto*v=io::find_member(o,"solution_moments"))out.moments=vector(*v,path);
    if(const auto*v=io::find_member(o,"solution_constraint_forces"))out.constraint_forces=vector(*v,path);
    if(const auto*v=io::find_member(o,"constraint_modes_tried"))out.modes_tried=count(*v,path);
    if(const auto*v=io::find_member(o,"linear_mode_rejections"))out.linear_mode_rejections=count(*v,path);
    if(const auto*v=io::find_member(o,"constraint_mode_search_complete"))out.mode_search_complete=boolean(*v,path);
    if(const auto*v=io::find_member(o,"solution_qpos"))out.qpos=vector(*v,path);
    if(const auto*v=io::find_member(o,"solution_qvel"))out.qvel=vector(*v,path);
    if(const auto t=maybe("solution_time_s"))out.time_s=*t;
    if(const auto*v=io::find_member(o,"numeric_identity"))if(const auto*s=std::get_if<std::string>(&v->data);!s||*s!=allocation_numeric_identity)throw std::invalid_argument(std::string(path)+": unsupported allocation numeric identity");
    return out;
}
Value encode_response_allocation(const ResponseAllocation& a) {
    if(a.disabled) {
        if(!a.allocation.feasible||a.allocation.violation!=0.)throw std::invalid_argument("disabled controller must be feasible with zero residual");
        return Value{Value::Object{{"feasible",Value{true}},{"violation",real(0.)},{"invalid_controller",Value{false}}}};
    }
    Value::Object out{{"invalid_controller",Value{!a.allocation.feasible}}, {"feasible",Value{a.allocation.feasible}}, {"violation",residual_value(a.allocation.violation)},
        {"grip_branch",Value{Value::Array{Value{a.branch[0]},Value{a.branch[1]}}}}, {"effort_scale",real(a.effort_scale)},
        {"crank_torque_target_nm",optional(a.crank_torque_target_nm)}, {"solution_crank_torque_nm",optional(a.predicted_crank_torque_nm)},
        {"saddle_normal_lower_bound_n",real(a.saddle_normal_lower_bound_n)}, {"solution_tau",values(a.torque)}, {"solution_qddot",values(a.rider_acceleration)},
        {"solution_base_qacc",values(a.base_acceleration)}, {"solution_power",values(a.power)}, {"solution_wrenches",values(a.wrenches)},
        {"solution_excitation_nm",values(a.excitation)}, {"solution_activation_state_nm",values(a.activation_state)}, {"solution_active_lower_nm",values(a.lower_torque_nm)}, {"solution_active_upper_nm",values(a.upper_torque_nm)}};
    if(a.modes_tried!=0){out.emplace_back("solution_moments",values(a.moments));out.emplace_back("solution_constraint_forces",values(a.constraint_forces));out.emplace_back("dynamics_model",Value{std::string("current_soft_constraint_response")});out.emplace_back("constraint_modes_tried",Value{io::Number{std::to_string(a.modes_tried)}});out.emplace_back("constraint_mode_search_complete",Value{a.mode_search_complete});out.emplace_back("linear_mode_rejections",Value{io::Number{std::to_string(a.linear_mode_rejections)}});out.emplace_back("grip_radius_guard_fraction",real(grip_radius_guard_fraction));out.emplace_back("allocation_force_guard_n",real(support_force_guard_n));out.emplace_back("allocation_moment_guard_nm",real(support_moment_guard_nm));}
    out.emplace_back("solution_qpos",values(a.qpos));out.emplace_back("solution_qvel",values(a.qvel));out.emplace_back("solution_time_s",real(a.time_s));out.emplace_back("numeric_identity",Value{std::string(allocation_numeric_identity)});return Value{std::move(out)};
}

Value encode_support_diagnostics(const LegacySupportDiagnostics& s) {
    const auto&t=s.target;Value::Object loads,stance,pedals,feet;
    constexpr std::array<const char*,4> names{"saddle","front","rear","grip"};
    for(std::size_t i=0;i<4;++i)loads.emplace_back(names[i],real(t.vertical_forces_n[i]));
    for(std::size_t i=0;i<2;++i) {
        const std::string side=i?"rear":"front";const auto&f=s.feet[i];stance.emplace_back(side,Value{s.stance[i]});pedals.emplace_back(side,values(s.pedal_forces[i]));
        Value::Object geometry;Value::Array reasons;
        if(f.geometry){const auto&g=*f.geometry;geometry={{"requested_compression_m",real(g.requested_compression_m)},{"applied_compression_m",real(g.applied_compression_m)},{"requested_shear_m",real(g.requested_shear_m)},{"applied_shear_m",real(g.applied_shear_m)},{"saturated",Value{g.saturated}}};for(const auto&name:g.limiting_reasons)reasons.push_back(Value{name});}
        else geometry.emplace_back("saturated",Value{false});
        geometry.emplace_back("limiting_reasons",Value{std::move(reasons)});
        if(f.weld)geometry.emplace_back("weld",Value{true});if(f.recovery)geometry.emplace_back("recovery",Value{true});
        feet.emplace_back(side,Value{Value::Object{{"actual_sole_position_m",values(f.actual)},{"target_sole_position_m",values(f.target)},{"ik_saturated",Value{f.ik_saturated}},{"recovery_stage",Value{f.recovery_stage}},{"goal_geometry",Value{std::move(geometry)}}}});
    }
    Value::Object posture{{"torso_lean_rad",real(s.posture.torso_lean_rad)},{"pelvis_pitch_rad",real(s.posture.pelvis_pitch_rad)},{"pelvis_offset_m",s.posture.pelvis_offset_m?values(*s.posture.pelvis_offset_m):Value{}},{"use_saddle",Value{s.posture.use_saddle}}};
    Value::Object tracking{{"actual_phase_rad",real(s.actual_phase_rad)},{"target_phase_rad",optional(s.target_phase_rad)},{"phase_error_rad",optional(s.phase_error_rad)},{"actual_rate_rad_s",real(s.actual_rate_rad_s)},{"target_rate_rad_s",real(s.target_rate_rad_s)}};
    return Value{Value::Object{{"requested_pitch_moment_nm",real(t.requested_pitch_moment_nm)},{"vertical_force_error_n",real(t.vertical_force_error_n)},{"gravity_moment_error_nm",real(t.gravity_moment_error_nm)},{"coasting_crank_torque_nm",real(t.coasting_crank_torque_nm)},{"feasible",Value{t.feasible}},{"requested_balance_force_on_rider_n",values(t.requested_balance_force)},{"horizontal_force_error_n",real(t.horizontal_force_error_n)},{"requested_vertical_forces_n",Value{std::move(loads)}},{"total_requested_force_on_rider_n",values(t.total_requested_force)},{"total_requested_pitch_moment_nm",real(t.total_requested_pitch_moment_nm)},{"stance",Value{std::move(stance)}},{"feasible_pedal_force_on_bike_n",Value{std::move(pedals)}},{"posture",Value{std::move(posture)}},{"coasting",Value{s.coasting}},{"coasting_requested_crank_torque_nm",real(s.coasting_requested_crank_torque_nm)},{"crank_tracking",Value{std::move(tracking)}},{"feet",Value{std::move(feet)}}}};
}

LegacySupportDiagnostics decode_support_diagnostics(const Value& value,std::string_view path) {
    const auto&o=io::as_object(value,path);
    static const std::set<std::string_view> allowed{"requested_pitch_moment_nm","vertical_force_error_n","gravity_moment_error_nm","coasting_crank_torque_nm","feasible","requested_balance_force_on_rider_n","horizontal_force_error_n","requested_vertical_forces_n","total_requested_force_on_rider_n","total_requested_pitch_moment_nm","stance","feasible_pedal_force_on_bike_n","posture","coasting","coasting_requested_crank_torque_nm","crank_tracking","feet"};
    for(const auto&[key,item]:o){(void)item;if(!allowed.contains(key))throw std::invalid_argument(std::string(path)+"."+key+": unknown key");}
    const auto get=[&](std::string_view key)->const Value&{return member(o,key,path);};
    const auto fixed3=[&](const Value&v){const auto x=vector(v,path);if(x.size()!=3)throw std::invalid_argument(std::string(path)+": expected 3-vector");return std::array<double,3>{x[0],x[1],x[2]};};
    LegacySupportDiagnostics out;auto&t=out.target;
    t.requested_pitch_moment_nm=number(get("requested_pitch_moment_nm"),path);t.vertical_force_error_n=number(get("vertical_force_error_n"),path);t.gravity_moment_error_nm=number(get("gravity_moment_error_nm"),path);t.coasting_crank_torque_nm=number(get("coasting_crank_torque_nm"),path);t.feasible=boolean(get("feasible"),path);t.requested_balance_force=fixed3(get("requested_balance_force_on_rider_n"));t.horizontal_force_error_n=number(get("horizontal_force_error_n"),path);t.total_requested_force=fixed3(get("total_requested_force_on_rider_n"));t.total_requested_pitch_moment_nm=number(get("total_requested_pitch_moment_nm"),path);
    const auto& loads=io::as_object(get("requested_vertical_forces_n"),path);constexpr std::array<const char*,4> names{"saddle","front","rear","grip"};if(loads.size()!=4)throw std::invalid_argument(std::string(path)+": support load keys");for(std::size_t i=0;i<4;++i)t.vertical_forces_n[i]=number(member(loads,names[i],path),path);
    const auto&stance=io::as_object(get("stance"),path);const auto&pedals=io::as_object(get("feasible_pedal_force_on_bike_n"),path);const auto&feet=io::as_object(get("feet"),path);if(stance.size()!=2||pedals.size()!=2||feet.size()!=2)throw std::invalid_argument(std::string(path)+": two feet required");
    for(std::size_t i=0;i<2;++i) {
        const auto side=i?"rear":"front";out.stance[i]=boolean(member(stance,side,path),path);out.pedal_forces[i]=fixed3(member(pedals,side,path));const auto&f=io::as_object(member(feet,side,path),path);if(f.size()!=5)throw std::invalid_argument(std::string(path)+": foot target keys");auto&target=out.feet[i];
        target.actual=fixed3(member(f,"actual_sole_position_m",path));target.target=fixed3(member(f,"target_sole_position_m",path));target.ik_saturated=boolean(member(f,"ik_saturated",path),path);target.recovery_stage=io::as_string(member(f,"recovery_stage",path),path);
        if(target.recovery_stage!="none"&&target.recovery_stage!="release"&&target.recovery_stage!="escape"&&target.recovery_stage!="raise"&&target.recovery_stage!="return")throw std::invalid_argument(std::string(path)+": unknown recovery stage");
        const auto&g=io::as_object(member(f,"goal_geometry",path),path);const bool saturated=boolean(member(g,"saturated",path),path);const auto&reasons=io::as_array(member(g,"limiting_reasons",path),path);
        if(io::find_member(g,"requested_compression_m")){::rider::SoleGoalDiagnostic geometry{};geometry.requested_compression_m=number(member(g,"requested_compression_m",path),path);geometry.applied_compression_m=number(member(g,"applied_compression_m",path),path);geometry.requested_shear_m=number(member(g,"requested_shear_m",path),path);geometry.applied_shear_m=number(member(g,"applied_shear_m",path),path);geometry.saturated=saturated;for(const auto&r:reasons)geometry.limiting_reasons.push_back(io::as_string(r,path));target.geometry=std::move(geometry);}
        else if(saturated||!reasons.empty())throw std::invalid_argument(std::string(path)+": unmodeled sole geometry diagnostics");
        if(const auto*v=io::find_member(g,"weld"))target.weld=boolean(*v,path);if(const auto*v=io::find_member(g,"recovery"))target.recovery=boolean(*v,path);
        const std::set<std::string_view> geometry_keys{"requested_compression_m","applied_compression_m","requested_shear_m","applied_shear_m","saturated","limiting_reasons","weld","recovery"};for(const auto&[key,v]:g){(void)v;if(!geometry_keys.contains(key))throw std::invalid_argument(std::string(path)+": unknown sole geometry field");}
    }
    const auto&p=io::as_object(get("posture"),path);if(p.size()!=4)throw std::invalid_argument(std::string(path)+": posture keys");out.posture.torso_lean_rad=number(member(p,"torso_lean_rad",path),path);out.posture.pelvis_pitch_rad=number(member(p,"pelvis_pitch_rad",path),path);out.posture.use_saddle=boolean(member(p,"use_saddle",path),path);const auto&offset=member(p,"pelvis_offset_m",path);if(!io::is_null(offset)){const auto x=vector(offset,path);if(x.size()!=2)throw std::invalid_argument(std::string(path)+": pelvis offset width");out.posture.pelvis_offset_m=std::array<double,2>{x[0],x[1]};}
    const auto&c=io::as_object(get("crank_tracking"),path);if(c.size()!=5)throw std::invalid_argument(std::string(path)+": crank tracking keys");out.actual_phase_rad=number(member(c,"actual_phase_rad",path),path);out.actual_rate_rad_s=number(member(c,"actual_rate_rad_s",path),path);out.target_rate_rad_s=number(member(c,"target_rate_rad_s",path),path);const auto&target=member(c,"target_phase_rad",path),&error=member(c,"phase_error_rad",path);if(!io::is_null(target))out.target_phase_rad=number(target,path);if(!io::is_null(error))out.phase_error_rad=number(error,path);
    out.coasting=boolean(get("coasting"),path);out.coasting_requested_crank_torque_nm=number(get("coasting_requested_crank_torque_nm"),path);return out;
}
} // namespace bike::rider
