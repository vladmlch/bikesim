#include "export.hpp"
#include "builder_internal.hpp"
#include "../io/json.hpp"
#include <charconv>
#include <fstream>
#include <memory>
#include <stdexcept>

namespace bike::model {
namespace {
using V=io::Value;
V n(double value) {return V{io::Number{geometry::detail::py_repr(value)}};}
V i(int value) {return V{io::Number{std::to_string(value)}};}
V s(std::string value) {return V{std::move(value)};}
double rounded(double value,int places) {
    const auto token=geometry::detail::py_fixed(value,places);double result=0.;
    const auto* end=std::to_address(token.end());
    const auto parse=std::from_chars(token.data(),end,result);
    if(parse.ec!=std::errc{} || parse.ptr!=end) throw std::invalid_argument("geometry export: nonfinite or out-of-range coordinate");
    return result;
}
V r(double value,int places=4) {return n(rounded(value,places));}
V points(const geometry::FixedFramePoints& fixed,const kinematics::KinematicState& state,
         const geometry::BikeSpecs& specs,double scale,int decimals) {
    const std::vector<std::pair<std::string,Vec3>> values{
        {"P0",fixed.P0},{"P1",state.P1},{"P2",state.P2},{"P3",state.P3},{"P4",state.P4},
        {"P5",fixed.P5},{"P6",state.P6},{"P7",fixed.P7},{"P8",fixed.P8},{"P9",fixed.P9},
        {"P10",fixed.P10},{"P11",fixed.P11},{"P12",state.P12},{"P_HT_bot",fixed.P_HT_bot},
        {"P_FA",geometry::compute_front_axle(specs)},{"BB",fixed.BB},
    };
    V::Object result;
    for(const auto& [name,point]:values) {
        V::Array coordinates;
        for(double v:point) coordinates.push_back(r(v*scale,decimals));
        result.emplace_back(name,V{std::move(coordinates)});
    }
    return V{std::move(result)};
}
void pretty_value(const V& value,std::string& output,std::size_t depth) {
    if(const auto* object=std::get_if<V::Object>(&value.data)) {
        if(object->empty()) {output+="{}";return;}
        output+="{\n";
        for(std::size_t index=0;index<object->size();++index) {
            output.append((depth+1)*2,' ');output+=io::canonical_json(s((*object)[index].first));output+=": ";
            pretty_value((*object)[index].second,output,depth+1);
            output+=index+1==object->size()?"\n":",\n";
        }
        output.append(depth*2,' ');output+='}';
    } else if(const auto* array=std::get_if<V::Array>(&value.data)) {
        if(array->empty()) {output+="[]";return;}
        output+="[\n";
        for(std::size_t index=0;index<array->size();++index) {
            output.append((depth+1)*2,' ');pretty_value((*array)[index],output,depth+1);
            output+=index+1==array->size()?"\n":",\n";
        }
        output.append(depth*2,' ');output+=']';
    } else output+=io::canonical_json(value);
}
void write_export(const std::filesystem::path& file,std::string_view contents) {
    if(file.empty() || file.native().find('\0')!=std::string::npos) throw std::invalid_argument("model export needs a nonempty output path without NUL");
    if(!file.parent_path().empty()) std::filesystem::create_directories(file.parent_path());
    // exporter.py explicitly opens with 'w': replacing a requested export
    // is supported, and does not adopt the recording publication semantics.
    std::ofstream output(file,std::ios::binary|std::ios::trunc);
    if(!output) throw std::runtime_error("cannot open model export: "+file.string());
    output.write(contents.data(),static_cast<std::streamsize>(contents.size()));
    output.close();
    if(!output) throw std::runtime_error("cannot write model export: "+file.string());
}
} // namespace

io::Value bike_specs_value(const geometry::BikeSpecs& x) {
    return V{V::Object{
        {"reach",n(x.reach)},{"stack",n(x.stack)},{"head_angle_deg",n(x.head_angle_deg)},
        {"effective_seat_angle_deg",n(x.effective_seat_angle_deg)},{"bb_drop",n(x.bb_drop)},
        {"wheelbase",n(x.wheelbase)},{"headtube_length",n(x.headtube_length)},
        {"front_wheel_radius",n(x.front_wheel_radius)},{"rear_wheel_radius",n(x.rear_wheel_radius)},
        {"wheel_radius",n(x.wheel_radius)},{"crank_length",n(x.crank_length)},
        {"fork_travel",n(x.fork_travel)},{"fork_offset",n(x.fork_offset)},{"rear_wheel_travel",n(x.rear_wheel_travel)},
        {"shock_eye_to_eye",n(x.shock_eye_to_eye)},{"shock_stroke",n(x.shock_stroke)},
        {"fork_stiffness",n(x.fork_stiffness)},{"fork_damping",n(x.fork_damping)},
        {"shock_stiffness",n(x.shock_stiffness)},{"shock_damping",n(x.shock_damping)},
        {"fork_hsc",i(x.fork_hsc)},{"fork_lsc",i(x.fork_lsc)},{"fork_rebound",i(x.fork_rebound)},
        {"fork_initial_psi",n(x.fork_initial_psi)},{"fork_air_tokens",i(x.fork_air_tokens)},
        {"shock_hsc",i(x.shock_hsc)},{"shock_lsc",i(x.shock_lsc)},{"shock_rebound",i(x.shock_rebound)},
        {"shock_hbo",i(x.shock_hbo)},{"shock_lockout",V{x.shock_lockout}},
    }};
}
io::Value mass_specs_value(const BikeMassSpecs& x) {
    return V{V::Object{
        {"motor_mass",n(x.motor_mass)},{"battery_mass",n(x.battery_mass)},{"frame_structure_mass",n(x.frame_structure_mass)},
        {"saddle_post_mass",n(x.saddle_post_mass)},{"crank_pedals_mass",n(x.crank_pedals_mass)},
        {"steer_assembly_mass",n(x.steer_assembly_mass)},{"stanchions_mass",n(x.stanchions_mass)},
        {"fork_lowers_mass",n(x.fork_lowers_mass)},{"chainstay_mass",n(x.chainstay_mass)},
        {"seatstay_mass",n(x.seatstay_mass)},{"rocker_mass",n(x.rocker_mass)},{"shock_yoke_mass",n(x.shock_yoke_mass)},
        {"shock_damper_mass",n(x.shock_damper_mass)},{"front_wheel_mass",n(x.front_wheel_mass)},
        {"rear_wheel_mass",n(x.rear_wheel_mass)},{"wheel_rim_tire_fraction",n(x.wheel_rim_tire_fraction)},
        {"wheel_hub_core_fraction",n(x.wheel_hub_core_fraction)},
    }};
}
io::Value geometry_value(const BuildRequest& request) {
    const auto& specs=request.bike;
    std::optional<kinematics::HorstLinkageSolver> default_solver;
    if(!request.solver) default_solver.emplace(specs);
    const auto& solver=request.solver?*request.solver:*default_solver;
    const auto fixed=geometry::get_fixed_frame_points(specs);
    const auto uncomp=solver.solve_state_from_wheel_travel(0.),comp=solver.solve_state_from_wheel_travel(specs.rear_wheel_travel);
    const auto trail=geometry::compute_trail(specs);
    const auto angles=geometry::compute_frame_angles(specs);
    const auto trajectory=solver.solve_trajectory(101,specs.rear_wheel_travel);
    const V links{V::Object{
        {"chainstay_p0_p2_mm",r(solver.l_cs())},{"seatstay_p2_p3_mm",r(solver.l_ss())},
        {"seatstay_tube_p3_p12_mm",r(solver.l_ss_tube())},{"dropout_height_p12_p1_mm",r(solver.l_dropout_height())},
        {"dropout_base_p2_p1_mm",r(solver.r_21())},{"dropout_front_p2_p12_mm",r(solver.r_2_12())},
        {"rocker_p5_p3_mm",r(solver.r_53())},{"rocker_p5_p4_mm",r(solver.r_54())},
        {"rocker_span_p3_p4_mm",r(geometry::detail::np_norm3(detail::sub(uncomp.P3,uncomp.P4)))},
        {"rear_axle_offset_p2_p1_mm",r(solver.r_21())},{"shock_yoke_p4_p6_mm",r(solver.l_yoke())},
        {"shock_eye_to_eye_uncompressed_mm",r(solver.l_shock_0())},{"shock_eye_to_eye_compressed_mm",r(comp.shock_length)},
        {"shock_stroke_mm",r(comp.shock_stroke)},{"headtube_length_mm",r(specs.headtube_length)},
    }};
    const V summary{V::Object{
        {"reach_mm",n(specs.reach)},{"stack_mm",n(specs.stack)},{"head_angle_deg",n(specs.head_angle_deg)},
        {"effective_seat_angle_deg",n(specs.effective_seat_angle_deg)},{"wheelbase_mm",n(specs.wheelbase)},
        {"bb_drop_mm",n(specs.bb_drop)},{"wheel_radius_mm",n(specs.wheel_radius)},{"fork_travel_mm",n(specs.fork_travel)},
        {"fork_offset_mm",n(specs.fork_offset)},{"ground_trail_mm",r(trail.ground_trail)},
        {"mechanical_trail_mm",r(trail.mechanical_trail)},{"axle_to_crown_mm",r(trail.axle_to_crown)},
        {"rear_wheel_travel_mm",n(specs.rear_wheel_travel)},{"angle_x_tube1_deg",r(angles.angle_x_tube1_deg,3)},
        {"angle_x_tube2_deg",r(angles.angle_x_tube2_deg,3)},{"angle_1_deg",r(angles.angle_1_deg,3)},
        {"initial_leverage_ratio",r(trajectory.initial_leverage_ratio)},{"final_leverage_ratio",r(trajectory.final_leverage_ratio)},
        {"progressivity_pct",r(trajectory.progressivity_pct,3)},
        {"initial_transmission_angle_deg",r(trajectory.transmission_angle_deg.front(),3)},
        {"final_transmission_angle_deg",r(trajectory.transmission_angle_deg.back(),3)},
        {"max_link_error_mm",n(trajectory.max_link_error)},
    }};
    // Source exporter intentionally uses DamperClickConfig() defaults for
    // display metadata, independent of BikeSpecs knob values.
    const V damper{V::Object{
        {"fork",V{V::Object{
            {"model",s("RockShox ZEB Ultimate (Charger 3 RC2 + DebonAir+)")},
            {"travel_mm",n(specs.fork_travel)},{"air_pressure_psi",n(85.2)},{"volume_tokens",i(2)},
            {"hsc_clicks",i(2)},{"hsc_range",s("0..4 clicks (0=Open, 2=Neutral, 4=Firm)")},
            {"lsc_clicks",i(7)},{"lsc_range",s("0..14 clicks (0=Open, 7=Neutral, 14=Firm)")},
            {"rebound_clicks",i(9)},{"rebound_range",s("0..17 clicks (0=Fast/Open, 9=Neutral, 17=Slow/Firm)")},
            {"hbo",s("Fixed Hydraulic Bottom-Out (active last 20 mm)")},
        }}},
        {"rear_shock",V{V::Object{
            {"model",s("RockShox Super Deluxe Ultimate (RC2T + HBO)")},{"eye_to_eye_mm",n(specs.shock_eye_to_eye)},
            {"stroke_mm",n(specs.shock_stroke)},{"hsc_clicks",i(2)},{"hsc_range",s("0..4 clicks (0=Open, 2=Neutral, 4=Firm)")},
            {"lsc_clicks",i(7)},{"lsc_range",s("0..14 clicks (0=Open, 7=Neutral, 14=Firm)")},
            {"rebound_clicks",i(7)},{"rebound_range",s("0..14 clicks (0=Fast/Open, 7=Neutral, 14=Slow/Firm)")},
            {"hbo_clicks",i(2)},{"hbo_range",s("0..4 clicks (0=Min HBO, 2=Neutral, 4=Max HBO)")},{"lockout_state",s("Open")},
        }}},
    }};
    return V{V::Object{
        {"coordinate_frame",V{V::Object{{"origin",s("Bottom Bracket (BB) [0, 0, 0]")},
            {"axes",V{V::Object{{"+X",s("Forward")},{"+Y",s("Lateral (Left)")},{"+Z",s("Upward")}}}},
            {"unit",s("mm and meters")}}}},
        {"uncompressed_points_mm",points(fixed,uncomp,specs,1.,4)},
        {"uncompressed_points_m",points(fixed,uncomp,specs,.001,6)},
        {"compressed_points_mm",points(fixed,comp,specs,1.,4)},
        {"compressed_points_m",points(fixed,comp,specs,.001,6)},
        {"link_lengths_mm",links},{"geometry_summary",summary},{"damper_specifications",damper},
    }};
}
void export_mjcf(const BuildRequest& request,const std::filesystem::path& file) {write_export(file,generate_mjcf(request));}
void export_geometry_json(const BuildRequest& request,const std::filesystem::path& file) {
    std::string json;pretty_value(geometry_value(request),json,0);write_export(file,json);
}
void export_playground_models(const BuildRequest& request,const std::filesystem::path& directory) {
    BuildRequest common;common.bike=request.bike;common.solver=request.solver;
    common.mode="stand";common.include_rider=request.include_rider;
    export_mjcf(common,directory/"bike_playground_stand.xml");
    common.mode="standard";common.include_rider=false;
    export_mjcf(common,directory/"bike_model.xml");
    common.mode="playground";common.include_rider=request.include_rider;common.debug_markers=true;
    export_mjcf(common,directory/"bike_playground.xml");
    common.mode="ride";common.debug_markers=false;common.include_rider=false;
    common.rider=configuration::RiderSpecs{};common.tyre_model=request.tyre_model;
    export_mjcf(common,directory/"bike_ride.xml");
}
void export_playground_models(const std::filesystem::path& directory) {export_playground_models(BuildRequest{},directory);}
void export_model(const BuildRequest& request,const ExportOptions& options) {
    if(options.xml_only && options.json_only) throw std::invalid_argument("xml_only and json_only are mutually exclusive");
    if(!options.json_only) export_mjcf(request,options.xml_file);
    if(!options.xml_only) export_geometry_json(request,options.json_file);
}
} // namespace bike::model
