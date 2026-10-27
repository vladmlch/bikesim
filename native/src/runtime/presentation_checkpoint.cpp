#include "presentation_checkpoint.hpp"
#include "checkpoint_io.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <type_traits>

namespace runtime {
namespace {
using namespace checkpoint_io;
constexpr auto scalar_fields=std::to_array<std::pair<std::string_view,double PresentationState::*>>({
    {"time_s",&PresentationState::time_s},{"position_m",&PresentationState::position_m},
    {"speed_mps",&PresentationState::speed_mps},{"pitch_rad",&PresentationState::pitch_rad},
    {"z_m",&PresentationState::z_m},{"fork_travel_mm",&PresentationState::fork_travel_mm},
    {"fork_velocity_mps",&PresentationState::fork_velocity_mps},{"fork_force_n",&PresentationState::fork_force_n},
    {"shock_stroke_mm",&PresentationState::shock_stroke_mm},{"shock_velocity_mps",&PresentationState::shock_velocity_mps},
    {"shock_force_n",&PresentationState::shock_force_n},{"crank_phase_rad",&PresentationState::crank_phase_rad},
    {"front_load_n",&PresentationState::front_load_n},{"rear_load_n",&PresentationState::rear_load_n},
    {"battery_energy_j",&PresentationState::battery_energy_j},{"grip_gap_m",&PresentationState::grip_gap_m},
    {"root_pitch_deg",&PresentationState::root_pitch_deg},{"pelvis_pitch_deg",&PresentationState::pelvis_pitch_deg},
    {"torso_pitch_deg",&PresentationState::torso_pitch_deg}});
constexpr auto flag_fields=std::to_array<std::pair<std::string_view,bool PresentationState::*>>({
    {"assist_pedaling",&PresentationState::assist_pedaling},{"rider_present",&PresentationState::rider_present},
    {"tire_present",&PresentationState::tire_present},{"grip_enabled",&PresentationState::grip_enabled},
    {"grip_reachable",&PresentationState::grip_reachable},{"stance_front",&PresentationState::stance_front},
    {"stance_rear",&PresentationState::stance_rear}});
constexpr std::array<std::string_view,6> model_names{"dof_frictionloss","site_pos","tendon_stiffness",
    "tendon_damping","tendon_lengthspring","tendon_range"};

double finite_number(const Value& value,std::string_view path) {
    const auto result=bike::io::as_double(value,path);
    if(!std::isfinite(result))vread::invalid(path,"expected finite binary64");
    return result;
}
template<class Members> void exact_members(const Value& value,const Members& members,std::string_view path) {
    const auto& object=bike::io::as_object(value,path);std::set<std::string_view> names;
    if(object.size()!=members.size())vread::invalid(path,"wrong field set");
    for(const auto& [name,unused]:object) {
        (void)unused;
        if(!names.insert(name).second||!std::ranges::any_of(members,[&](const auto& item){return item.first==name;}))
            vread::invalid(path,"duplicate or unknown field");
    }
}
template<std::size_t N> std::array<double,N> fixed(const Value& value,std::string_view path) {
    const auto& values=bike::io::as_array(value,path);
    if(values.size()!=N)vread::invalid(path,"wrong shape");
    std::array<double,N> out{};
    for(std::size_t i=0;i<N;++i)out[i]=finite_number(values[i],path);
    return out;
}
std::optional<double> finite_optional(const Value& value,std::string_view path) {
    return bike::io::is_null(value)?std::nullopt:std::optional<double>{finite_number(value,path)};
}
std::string bounded_string(const Value& value,std::size_t maximum,std::string_view path) {
    const auto& source=bike::io::as_string(value,path);
    if(source.size()>maximum||source.find('\0')!=std::string::npos)vread::invalid(path,"string exceeds bound or contains NUL");
    return source;
}
Value drive_value(const drivetrain::DriveTelemetry& telemetry) {
    Object values;
    telemetry.for_each([&](drivetrain::TelemetryField key,const drivetrain::TelemetryScalar& source) {
        Value item;
        std::visit([&](const auto& current) {
            using T=std::decay_t<decltype(current)>;
            if constexpr(std::is_same_v<T,std::monostate>)item=Value{};
            else if constexpr(std::is_same_v<T,double>)item=real(current);
            else if constexpr(std::is_same_v<T,std::int32_t>)item=integer(current);
            else if constexpr(std::is_same_v<T,std::uint8_t>)item=text(drivetrain::telemetry_label_name(key,current));
            else if constexpr(std::is_same_v<T,bool>)item=Value{current};
            else item=text(current);
        },source);
        values.emplace_back(drivetrain::telemetry_spec(key).name,std::move(item));
    });
    return Value{std::move(values)};
}
drivetrain::DriveTelemetry drive_read(const Value& value,const drivetrain::DriveTelemetry& layout) {
    auto out=layout;out.clear();std::set<std::string> seen;
    for(const auto& [name,item]:bike::io::as_object(value,"presentation.drive")) {
        const auto key=drivetrain::telemetry_field(name);
        if(!seen.insert(name).second||!key)vread::invalid("presentation.drive","duplicate or unknown field");
        const auto& spec=drivetrain::telemetry_spec(*key);
        const auto path="presentation.drive."+name;
        switch(spec.kind) {
        case drivetrain::TelemetryKind::flag:out.set(*key,bike::io::as_bool(item,path));break;
        case drivetrain::TelemetryKind::integer: {
            const auto number=bike::io::as_int64(item,path);
            if(number<spec.integer_min||number>std::numeric_limits<std::int32_t>::max())vread::invalid(path,"integer outside telemetry range");
            out.set(*key,static_cast<std::int32_t>(number));break;
        }
        case drivetrain::TelemetryKind::label: {
            const auto label=drivetrain::telemetry_label(*key,bike::io::as_string(item,path));
            if(!label)vread::invalid(path,"unknown telemetry label");
            out.set_label(*key,*label);break;
        }
        case drivetrain::TelemetryKind::open:out.set_open(*key,bounded_string(item,4096,path));break;
        case drivetrain::TelemetryKind::optional:
            if(bike::io::is_null(item)){out.set_none(*key);break;}
            [[fallthrough]];
        case drivetrain::TelemetryKind::real: {
            const auto number=finite_number(item,path);
            if(name=="gear_ratio"&&number<=0.)vread::invalid(path,"gear ratio must be positive");
            out.set(*key,number);break;
        }
        }
    }
    return out;
}
Value spatial_value(const std::optional<SpatialPresentation>& pose) {
    if(!pose)return {};
    if(pose->generation>static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
        vread::invalid("presentation.spatial.generation","outside runtime generation domain");
    return Value{Object{{"position_world_m",reals(pose->position_world_m)},{"quat_wxyz",reals(pose->quat_wxyz)},
        {"heading_rad",optional(pose->heading_rad)},{"roll_rad",optional(pose->roll_rad)},
        {"steer_angle_rad",real(pose->steer_angle_rad)},{"steer_torque_nm",real(pose->steer_torque_nm)},
        {"lateral_slip_mps",reals(pose->lateral_slip_mps)},
        {"contact_valid",Value{Array{Value{pose->contact_valid[0]},Value{pose->contact_valid[1]}}}},
        {"attitude_valid",Value{pose->attitude_valid}},{"hold_active",Value{pose->hold_active}},
        {"generation",integer(static_cast<std::int64_t>(pose->generation))}}};
}
std::optional<SpatialPresentation> spatial_read(const Value& value,bool expected) {
    if(bike::io::is_null(value)) {
        if(expected)vread::invalid("presentation.spatial","spatial layout requires a saved pose");
        return {};
    }
    if(!expected)vread::invalid("presentation.spatial","planar layout has no spatial pose");
    exact(value,{"position_world_m","quat_wxyz","heading_rad","roll_rad","steer_angle_rad","steer_torque_nm",
        "lateral_slip_mps","contact_valid","attitude_valid","hold_active","generation"},"presentation.spatial");
    SpatialPresentation out;
    out.position_world_m=fixed<3>(field(value,"position_world_m"),"presentation.spatial.position_world_m");
    out.quat_wxyz=fixed<4>(field(value,"quat_wxyz"),"presentation.spatial.quat_wxyz");
    double norm=0.;for(const auto x:out.quat_wxyz)norm+=x*x;
    if(!std::isfinite(norm)||std::abs(norm-1.)>1e-6)vread::invalid("presentation.spatial.quat_wxyz","expected unit quaternion; decoder never normalizes");
    out.heading_rad=finite_optional(field(value,"heading_rad"),"presentation.spatial.heading_rad");
    out.roll_rad=finite_optional(field(value,"roll_rad"),"presentation.spatial.roll_rad");
    out.steer_angle_rad=finite_number(field(value,"steer_angle_rad"),"presentation.spatial.steer_angle_rad");
    out.steer_torque_nm=finite_number(field(value,"steer_torque_nm"),"presentation.spatial.steer_torque_nm");
    out.lateral_slip_mps=fixed<2>(field(value,"lateral_slip_mps"),"presentation.spatial.lateral_slip_mps");
    const auto& valid=bike::io::as_array(field(value,"contact_valid"),"presentation.spatial.contact_valid");
    if(valid.size()!=2)vread::invalid("presentation.spatial.contact_valid","expected front/rear flags");
    for(std::size_t i=0;i<2;++i)out.contact_valid[i]=bike::io::as_bool(valid[i],"presentation.spatial.contact_valid");
    out.attitude_valid=boolean(field(value,"attitude_valid"));out.hold_active=boolean(field(value,"hold_active"));
    if(out.attitude_valid!=out.heading_rad.has_value()||out.attitude_valid!=out.roll_rad.has_value())
        vread::invalid("presentation.spatial.attitude_valid","angle availability differs");
    const auto generation=count(field(value,"generation"));
    if(generation<0||generation>std::numeric_limits<int>::max())vread::invalid("presentation.spatial.generation","outside runtime generation domain");
    out.generation=static_cast<std::uint64_t>(generation);return out;
}
}

bike::io::Value encode_presentation_checkpoint(const PresentationState& state) {
    Object scalar,flags,model;Array supports,joints,addresses;
    for(const auto& [name,member]:scalar_fields)scalar.emplace_back(name,real(state.*member));
    for(const auto& [name,member]:flag_fields){const bool flag=state.*member;flags.emplace_back(name,Value{flag});}
    for(const auto& support:state.supports)supports.push_back(Value{Object{{"in_platform",Value{support.in_platform}},
        {"normal_load_n",real(support.normal_load_n)},{"gap_m",real(support.gap_m)}}});
    for(const auto& [name,angle]:state.joints)joints.push_back(Value{Object{{"name",text(name)},{"angle_rad",real(angle)}}});
    for(const auto address:state.joint_qpos_addresses)addresses.push_back(integer(address));
    for(std::size_t i=0;i<model_names.size();++i)model.emplace_back(model_names[i],reals(state.model_fields[i]));
    return Value{Object{{"schema",integer(1)},{"scalars",Value{std::move(scalar)}},{"flags",Value{std::move(flags)}},
        {"root_x_dofadr",integer(state.root_x_dofadr)},{"balance_lost_at_m",optional(state.balance_lost_at_m)},
        {"drive",drive_value(state.drive)},{"supports",Value{std::move(supports)}},
        {"ik_saturated",text(state.ik_saturated)},{"joints_saturated",text(state.joints_saturated)},
        {"joints",Value{std::move(joints)}},{"joint_qpos_addresses",Value{std::move(addresses)}},
        {"tire_slip",reals(state.tire_slip)},{"tire_mu",reals(state.tire_mu)},
        {"model_fields",Value{std::move(model)}},{"spatial",spatial_value(state.spatial)}}};
}

PresentationState decode_presentation_checkpoint(const bike::io::Value& value,const PresentationState& layout) {
    exact(value,{"schema","scalars","flags","root_x_dofadr","balance_lost_at_m","drive","supports","ik_saturated",
        "joints_saturated","joints","joint_qpos_addresses","tire_slip","tire_mu","model_fields","spatial"},"presentation");
    if(count(field(value,"schema"))!=1)vread::invalid("presentation.schema","unsupported version");
    auto out=layout;
    const auto& scalar=field(value,"scalars");exact_members(scalar,scalar_fields,"presentation.scalars");
    for(const auto& [name,member]:scalar_fields)out.*member=finite_number(field(scalar,name),"presentation.scalars."+std::string(name));
    if(out.time_s<0.)vread::invalid("presentation.time_s","must be nonnegative");
    const auto& flags=field(value,"flags");exact_members(flags,flag_fields,"presentation.flags");
    for(const auto& [name,member]:flag_fields)out.*member=bike::io::as_bool(field(flags,name),"presentation.flags."+std::string(name));
    if(out.rider_present!=layout.rider_present||out.tire_present!=layout.tire_present)
        vread::invalid("presentation.flags","model-bound rider/tyre availability differs");
    if(count(field(value,"root_x_dofadr"))!=layout.root_x_dofadr)vread::invalid("presentation.root_x_dofadr","model-bound address differs");
    out.balance_lost_at_m=finite_optional(field(value,"balance_lost_at_m"),"presentation.balance_lost_at_m");
    out.drive=drive_read(field(value,"drive"),layout.drive);
    const auto& supports=bike::io::as_array(field(value,"supports"),"presentation.supports");
    if(supports.size()!=out.supports.size())vread::invalid("presentation.supports","wrong model-bound width");
    for(std::size_t i=0;i<supports.size();++i) {
        const auto& entry=supports[i];exact(entry,{"in_platform","normal_load_n","gap_m"},"presentation.supports");
        out.supports[i]={.in_platform=boolean(field(entry,"in_platform")),
            .normal_load_n=finite_number(field(entry,"normal_load_n"),"presentation.support.normal_load_n"),
            .gap_m=finite_number(field(entry,"gap_m"),"presentation.support.gap_m")};
    }
    std::size_t saturated_bound=0;for(const auto& [name,unused]:layout.joints){(void)unused;saturated_bound+=name.size()+1;}
    out.ik_saturated=bounded_string(field(value,"ik_saturated"),4096,"presentation.ik_saturated");
    out.joints_saturated=bounded_string(field(value,"joints_saturated"),saturated_bound,"presentation.joints_saturated");
    const auto& joints=bike::io::as_array(field(value,"joints"),"presentation.joints");
    const auto& addresses=bike::io::as_array(field(value,"joint_qpos_addresses"),"presentation.joint_qpos_addresses");
    if(joints.size()!=layout.joints.size()||addresses.size()!=layout.joint_qpos_addresses.size()||joints.size()!=addresses.size())
        vread::invalid("presentation.joints","wrong model-bound width");
    for(std::size_t i=0;i<joints.size();++i) {
        exact(joints[i],{"name","angle_rad"},"presentation.joints");
        if(string(field(joints[i],"name"))!=layout.joints[i].first||count(addresses[i])!=layout.joint_qpos_addresses[i])
            vread::invalid("presentation.joints","model-bound name/address differs");
        out.joints[i].second=finite_number(field(joints[i],"angle_rad"),"presentation.joints.angle_rad");
    }
    out.tire_slip=fixed<2>(field(value,"tire_slip"),"presentation.tire_slip");
    out.tire_mu=fixed<2>(field(value,"tire_mu"),"presentation.tire_mu");
    const auto& model=field(value,"model_fields");
    exact(model,{"dof_frictionloss","site_pos","tendon_stiffness","tendon_damping","tendon_lengthspring","tendon_range"},"presentation.model_fields");
    for(std::size_t i=0;i<model_names.size();++i) {
        const auto& values=bike::io::as_array(field(model,model_names[i]),"presentation.model_fields");
        if(values.size()!=layout.model_fields[i].size())vread::invalid("presentation.model_fields","wrong model-bound width");
        for(std::size_t j=0;j<values.size();++j)out.model_fields[i][j]=finite_number(values[j],"presentation.model_fields");
    }
    out.spatial=spatial_read(field(value,"spatial"),layout.spatial.has_value());
    return out;
}
} // namespace runtime
