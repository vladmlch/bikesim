#include "plot_internal.hpp"
#include <iomanip>
#include <locale>
#include <sstream>

namespace bike::viz {
namespace {
std::string fixed(double value,int places=2) {std::ostringstream out;out.imbue(std::locale::classic());out<<std::fixed<<std::setprecision(places)<<value;return out.str();}
std::string scientific(double value) {std::ostringstream out;out.imbue(std::locale::classic());out<<std::scientific<<std::setprecision(2)<<value;return out.str();}
std::string description(std::string_view name) {
    constexpr std::array<std::pair<std::string_view,std::string_view>,16> values{{
        {"P0","Main Pivot (Frame / Chainstay)"},{"P1","Rear Wheel Axle"},{"P2","Horst Pivot (Chainstay / Seatstay)"},{"P3","Seatstay-Rocker Pivot"},
        {"P4","Rocker-Yoke Pivot"},{"P5","Rocker Frame Pivot (Seat Tube)"},{"P6","Shock Lower Eyelet"},{"P7","Shock Upper Mount (Frame Tab)"},
        {"P8","Top Tube Kink / Bend"},{"P9","Seatpost Clamp / Saddle Base"},{"P10","Seat Tube Junction (Seat Cluster)"},{"P11","Headtube Top (Reach, Stack)"},
        {"P12","Seatstay Dropout Upper Kink"},{"P_HT_bot","Headtube Bottom"},{"P_FA","Front Wheel Axle"},{"BB","Bottom Bracket Origin (0,0,0)"}}};
    for(const auto& [key,value]:values)if(key==name)return std::string(value);
    return {};
}
struct Table {
    std::ostringstream out;
    Table(){out.imbue(std::locale::classic());}
    void row(std::string_view name,std::string_view value,std::string_view unit,std::string_view note) {
        out<<std::left<<std::setw(36)<<name<<" | "<<std::right<<std::setw(16)<<value<<" | "<<std::left<<std::setw(8)<<unit<<" | "<<note<<'\n';
    }
};
}
std::string geometry_table(const geometry::BikeSpecs& specs,const kinematics::HorstLinkageSolver& solver) {
    const auto zero=detail::hardpoints(specs,solver,0.),compressed=detail::hardpoints(specs,solver,180.);std::ostringstream out;out.imbue(std::locale::classic());
    out<<"UNCOMPRESSED STATE HARDPOINTS (Wheel Travel = 0.0 mm, Shock Stroke = 0.0 mm)\n";
    out<<"Point | Description | X (mm) Y (mm) Z (mm) | X (m) Y (m) Z (m)\n";
    for(const auto& [name,p]:zero)out<<std::left<<std::setw(10)<<name<<" | "<<std::setw(32)<<description(name)<<" | "<<std::right<<std::fixed<<std::setprecision(3)<<std::setw(9)<<p[0]<<' '<<std::setw(7)<<p[1]<<' '<<std::setw(9)<<p[2]<<" | "<<std::setprecision(4)<<std::setw(8)<<p[0]/1000.<<' '<<std::setw(7)<<p[1]/1000.<<' '<<std::setw(8)<<p[2]/1000.<<'\n';
    out<<"\nFULLY COMPRESSED STATE HARDPOINTS (Wheel Travel = 180.0 mm, Shock Stroke = "<<fixed(solver.solve_state_from_wheel_travel(180.).shock_stroke)<<" mm)\n";
    out<<"Point | Description | X (mm) Y (mm) Z (mm) | dX (mm) dZ (mm)\n";
    for(std::size_t i=0;i<compressed.size();++i){const auto& [name,p]=compressed[i];const auto& initial=zero[i].second;
        out<<std::left<<std::setw(10)<<name<<" | "<<std::setw(32)<<description(name)<<" | "<<std::right<<std::fixed<<std::setprecision(3)<<std::setw(9)<<p[0]<<' '<<std::setw(7)<<p[1]<<' '<<std::setw(9)<<p[2]<<" | "<<std::showpos<<std::setw(8)<<p[0]-initial[0]<<' '<<std::setw(8)<<p[2]-initial[2]<<std::noshowpos<<'\n';}
    Table table;const auto angles=geometry::compute_frame_angles(specs);const auto trail=geometry::compute_trail(specs);
    table.out<<"\nBICYCLE FRAME GEOMETRY & SUSPENSION KINEMATICS SUMMARY\n";
    table.row("Reach",fixed(specs.reach,1),"mm","BB to Headtube Top (Horizontal)");table.row("Stack",fixed(specs.stack,1),"mm","BB to Headtube Top (Vertical)");
    table.row("Head Tube Angle (HA)",fixed(specs.head_angle_deg,1),"deg","Steering Axis Inclination");table.row("Effective Seat Tube Angle",fixed(specs.effective_seat_angle_deg,1),"deg","Virtual Seat Angle");
    table.row("Actual Seat Tube Angle (STA)",fixed(angles.effective_seat_tube_angle_deg),"deg","Seat Tube Axis relative to -X");table.row("Angle 1 (Seat Tube to Tube 1)",fixed(angles.angle_1_deg),"deg","Interior angle at Seat Cluster");
    table.row("Tube 1 Inclination (X to Tube 1)",fixed(angles.angle_x_tube1_deg),"deg","P10 -> P8 relative to +X");table.row("Tube 2 Inclination (X to Tube 2)",fixed(angles.angle_x_tube2_deg),"deg","BB -> P_HT_bot relative to +X");
    table.row("Wheelbase",fixed(specs.wheelbase,1),"mm","Front Axle to Rear Axle");table.row("Bottom Bracket Drop",fixed(specs.bb_drop,1),"mm","Axle Plane to BB Height");
    table.row("Front Wheel Radius (29\")",fixed(specs.front_wheel_radius,1),"mm","Front tire radius");table.row("Rear Wheel Radius (27.5\")",fixed(specs.rear_wheel_radius,1),"mm","Rear tire radius");
    table.row("Ground Plane Z",fixed(trail.ground_z,1),"mm","Flat ground contact plane");table.row("Fork Travel",fixed(specs.fork_travel,1),"mm","Front Suspension Travel");
    table.row("Fork Offset (Rake)",fixed(specs.fork_offset,1),"mm","Perpendicular Offset");table.row("Ground Trail",fixed(trail.ground_trail),"mm","Steer Ground Contact Offset");
    table.row("Mechanical Trail",fixed(trail.mechanical_trail),"mm","Torque Arm for Self-Centering");table.row("Axle-to-Crown (A2C)",fixed(trail.axle_to_crown),"mm","Fork Length (Uncompressed)");
    return out.str()+table.out.str()+kinematics_table(solver)+suspension_table(specs);
}
std::string kinematics_table(const kinematics::HorstLinkageSolver& solver) {
    const auto trajectory=solver.solve_trajectory(101,solver.specs().rear_wheel_travel);const auto compressed=solver.solve_state_from_wheel_travel(180.);Table out;
    out.row("Rear Wheel Travel",fixed(solver.specs().rear_wheel_travel,1),"mm","Vertical Axle Travel (+Z)");
    out.row("Shock Eye-to-Eye (Uncompressed)",fixed(solver.l_shock_0()),"mm","Target: 205.0 mm (trunnion)");out.row("Shock Eye-to-Eye (Compressed)",fixed(compressed.shock_length),"mm","Target: 140.0 mm");
    out.row("Shock Stroke",fixed(compressed.shock_stroke),"mm","Target: 65.00 mm (+-0.05 mm)");out.row("Initial Leverage Ratio",fixed(trajectory.initial_leverage_ratio,3),"ratio","At 0 mm Wheel Travel");
    out.row("Final Leverage Ratio",fixed(trajectory.final_leverage_ratio,3),"ratio","At 180 mm Wheel Travel");out.row("Suspension Progressivity","+"+fixed(trajectory.progressivity_pct),"%","Progressive Leverage Curve");
    out.row("Initial Transmission Angle",fixed(trajectory.transmission_angle_deg.front()),"deg","Safe Range (50 deg - 140 deg)");out.row("Final Transmission Angle",fixed(trajectory.transmission_angle_deg.back()),"deg","Safe Range (50 deg - 140 deg)");
    out.row("Max Rigid Link Invariant Error",scientific(trajectory.max_link_error),"mm","Requirement < 1e-10 mm");return out.out.str();
}
std::string suspension_table(const geometry::BikeSpecs& specs) {
    Table out;out.row("Fork Damper Model","Charger 3 RC2","model","RockShox ZEB Ultimate (38mm)");
    out.row("Fork HSC Clicks",std::to_string(specs.fork_hsc)+" / 4","clicks","0=Open, 2=Neutral, 4=Firm (+2)");out.row("Fork LSC Clicks",std::to_string(specs.fork_lsc)+" / 14","clicks","0=Open, 7=Neutral, 14=Firm");
    out.row("Fork Rebound Clicks",std::to_string(specs.fork_rebound)+" / 17","clicks","0=Fast, 9=Neutral, 17=Slow");out.row("Fork Air Pressure & Tokens",fixed(specs.fork_initial_psi,1)+" PSI, "+std::to_string(specs.fork_air_tokens)+" tok","psi/tok","DebonAir+ Pneumatic Spring");
    out.row("Rear Shock Damper Model","Super Deluxe RC2T","model","RockShox Ultimate + HBO");out.row("Rear Shock HSC Clicks",std::to_string(specs.shock_hsc)+" / 4","clicks","0=Open, 2=Neutral, 4=Firm");
    out.row("Rear Shock LSC Clicks",std::to_string(specs.shock_lsc)+" / 14","clicks","0=Open, 7=Neutral, 14=Firm");out.row("Rear Shock Rebound Clicks",std::to_string(specs.shock_rebound)+" / 14","clicks","0=Fast, 7=Neutral, 14=Slow");
    out.row("Hydraulic Bottom-Out (HBO)",std::to_string(specs.shock_hbo)+" / 4","clicks","Active in last 13mm stroke");out.row("Shock Lockout State",specs.shock_lockout?"Firm Platform":"Open / Active","state","Threshold 2-Pos Lever");return out.out.str();
}
} // namespace bike::viz
