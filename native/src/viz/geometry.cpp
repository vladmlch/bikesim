#include "plot_internal.hpp"
#include "../model/suspension_setup.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace bike::viz {
namespace detail {
Hardpoints hardpoints(const geometry::BikeSpecs& specs,const kinematics::HorstLinkageSolver& solver,double travel) {
    const auto fixed=geometry::get_fixed_frame_points(specs);const auto state=solver.solve_state_from_wheel_travel(travel);
    return {{"P0",fixed.P0},{"P1",state.P1},{"P2",state.P2},{"P3",state.P3},{"P4",state.P4},{"P5",fixed.P5},
        {"P6",state.P6},{"P7",fixed.P7},{"P8",fixed.P8},{"P9",fixed.P9},{"P10",fixed.P10},{"P11",fixed.P11},
        {"P12",state.P12},{"P_HT_bot",fixed.P_HT_bot},{"P_FA",geometry::compute_front_axle(specs)},{"BB",fixed.BB}};
}
}
namespace {
const geometry::Vec3& point(const detail::Hardpoints& points,std::string_view name) {
    for(const auto& [label,point]:points)if(label==name)return point;
    throw std::logic_error("missing geometry hardpoint");
}
void segment(Plot& plot,const detail::Hardpoints& points,std::string name,std::initializer_list<std::string_view> labels,std::string color) {
    Series series;series.name=std::move(name);series.color=std::move(color);
    for(const auto label:labels){const auto& p=point(points,label);series.x.push_back(p[0]);series.y.push_back(p[2]);}
    plot.series.push_back(std::move(series));
}
void circle(Plot& plot,std::string name,const geometry::Vec3& center,double radius,std::string color) {
    Series series;series.name=std::move(name);series.color=std::move(color);
    for(unsigned i=0;i<=128;++i){const double angle=static_cast<double>(i)/128.*2.*std::numbers::pi;series.x.push_back(center[0]+radius*std::cos(angle));series.y.push_back(center[2]+radius*std::sin(angle));}
    plot.series.push_back(std::move(series));
}
void links(Plot& plot,const detail::Hardpoints& p,std::string_view suffix,const std::optional<std::string>& color={}) {
    segment(plot,p,"Chainstay"+std::string(suffix),{"P0","P2"},color.value_or("#27AE60"));
    segment(plot,p,"Seatstay"+std::string(suffix),{"P2","P12","P3"},color.value_or("#E67E22"));
    segment(plot,p,"Rocker"+std::string(suffix),{"P3","P5","P4","P3"},color.value_or("#C0392B"));
    segment(plot,p,"Yoke"+std::string(suffix),{"P4","P6"},color.value_or("#8E44AD"));
    segment(plot,p,"Shock"+std::string(suffix),{"P7","P6"},color.value_or("#2980B9"));
    segment(plot,p,"Dropout"+std::string(suffix),{"P1","P2","P12","P1"},color.value_or("#E67E22"));
}
Plot frame_plot(const geometry::BikeSpecs& specs,const kinematics::HorstLinkageSolver& solver) {
    const auto points=detail::hardpoints(specs,solver,0.);const double ground=geometry::compute_ground_z(specs);
    Plot plot{.title="Bicycle Geometry & Horst-Link Suspension Layout",.x_label="X (mm) [Forward ->]",.y_label="Z (mm) [Upward ^]",.series={}};
    plot.equal_axes=true;plot.x_range=std::pair{-620.,1020.};plot.y_range=std::pair{ground-90.,780.};
    plot.series.push_back({.name="Ground",.x={-620.,1020.},.y={ground,ground},.color="#BDC3C7"});
    circle(plot,"Front wheel",point(points,"P_FA"),specs.front_wheel_radius,"#95A5A6");
    segment(plot,points,"Front triangle",{"BB","P_HT_bot","P11","P8","P10","BB"},"#2C3E50");
    segment(plot,points,"Seatpost",{"P10","P9"},"#2C3E50");
    segment(plot,points,"Fork & steering",{"P11","P_HT_bot","P_FA"},"#7F8C8D");
    const auto trail=geometry::compute_trail(specs);
    plot.notes={"Reach "+detail::number(specs.reach)+" mm | Stack "+detail::number(specs.stack)+" mm | Wheelbase "+detail::number(specs.wheelbase)+" mm",
        "Ground trail "+detail::number(trail.ground_trail)+" mm | Mechanical trail "+detail::number(trail.mechanical_trail)+" mm"};
    return plot;
}
template<class F>std::vector<double> transform(const std::vector<double>& x,F f){std::vector<double> y;y.reserve(x.size());for(const double v:x)y.push_back(f(v));return y;}
}
Report geometry_plot(const geometry::BikeSpecs& specs,const kinematics::HorstLinkageSolver& solver) {
    auto plot=frame_plot(specs,solver);const auto points=detail::hardpoints(specs,solver,0.);
    circle(plot,"Rear wheel",point(points,"P1"),specs.rear_wheel_radius,"#95A5A6");links(plot,points,"");
    Series pivots{.name="Pivot markers",.x={}, .y={},.color="#fbbf24",.style=SeriesStyle::points};for(const auto& [name,p]:points){pivots.x.push_back(p[0]);pivots.y.push_back(p[2]);plot.notes.push_back(name+" ("+detail::number(p[0])+", "+detail::number(p[2])+") mm");}plot.series.push_back(std::move(pivots));
    return {.title="Bicycle Geometry & Horst-Link Suspension Layout",.panels={std::move(plot)},.columns=1,.width=3200,.height=1840};
}
Report compressed_comparison(const geometry::BikeSpecs& specs,const kinematics::HorstLinkageSolver& solver) {
    auto plot=frame_plot(specs,solver);plot.title="Suspension Kinematics Overlay: 0 mm vs 180 mm";
    const auto initial=detail::hardpoints(specs,solver,0.),final=detail::hardpoints(specs,solver,180.);
    circle(plot,"Rear wheel: 0 mm",point(initial,"P1"),specs.rear_wheel_radius,"#2980B9");
    circle(plot,"Rear wheel: 180 mm",point(final,"P1"),specs.rear_wheel_radius,"#E74C3C");
    links(plot,initial," (0 mm)","#2980B9");links(plot,final," (180 mm)","#E74C3C");
    for(auto& series:plot.series)if(series.name.find("180 mm")!=std::string::npos)series.dashed=true;
    const auto trajectory=solver.solve_trajectory(101,180.);
    for(const auto name:{"P1","P2","P3","P4","P6","P12"}){Series arc{.name=std::string(name)+" trajectory",.x={}, .y={},.color="#95A5A6"};for(const auto& p:trajectory.point_array(name)){arc.x.push_back(p[0]);arc.y.push_back(p[2]);}plot.series.push_back(std::move(arc));}
    const auto a=point(initial,"P1"),b=point(final,"P1");plot.notes={"Rear axle dX = "+detail::number(b[0]-a[0])+" mm; dZ = "+detail::number(b[2]-a[2])+" mm",
        "Shock compression = "+detail::number(solver.solve_state_from_wheel_travel(180.).shock_stroke)+" mm"};
    return {.title="Suspension Compressed Comparison",.panels={std::move(plot)},.columns=1,.width=3200,.height=1840};
}
Report leverage_plot(const kinematics::HorstLinkageSolver& solver) {
    const auto trajectory=solver.solve_trajectory(101,solver.specs().rear_wheel_travel);
    Plot leverage{.title="1. Leverage Ratio Curve",.x_label="Rear Wheel Vertical Travel (mm)",.y_label="Leverage Ratio (dTravel / dStroke)",
        .series={{.name="Instantaneous Leverage Ratio",.x=trajectory.wheel_travel,.y=trajectory.leverage_ratio,.color="#1F77B4"}}};
    leverage.x_range=std::pair{0.,180.};leverage.y_range=std::pair{2.2,3.4};
    leverage.notes={"Initial LR: "+detail::number(trajectory.initial_leverage_ratio),"Final LR: "+detail::number(trajectory.final_leverage_ratio),"Progressivity: "+detail::number(trajectory.progressivity_pct)+"%"};
    Plot stroke{.title="2. Shock Stroke vs Wheel Travel",.x_label="Rear Wheel Vertical Travel (mm)",.y_label="Shock Stroke (mm)",
        .series={{.name="Shock Compression",.x=trajectory.wheel_travel,.y=trajectory.shock_stroke,.color="#E67E22"}}};
    stroke.markers.push_back({.kind=MarkerKind::horizontal,.first=65.,.second=0.,.label="Target Stroke (65.0 mm)",.color="#C0392B"});stroke.x_range=std::pair{0.,180.};stroke.y_range=std::pair{0.,72.};
    stroke.notes={"Max Stroke: "+detail::number(trajectory.max_stroke)+" mm"};
    Plot axle{.title="3. Rear Axle Path (X vs Z Trajectory)",.x_label="Rear Axle X Coordinate (mm)",.y_label="Rear Axle Z Coordinate (mm)",
        .series={{.name="Rear Axle Trajectory",.x=trajectory.axle_path_x,.y=trajectory.axle_path_z,.color="#27AE60"},
         {.name="Uncompressed (0 mm)",.x={trajectory.axle_path_x.front()},.y={trajectory.axle_path_z.front()},.color="#2980B9",.style=SeriesStyle::points},
         {.name="Compressed (180 mm)",.x={trajectory.axle_path_x.back()},.y={trajectory.axle_path_z.back()},.color="#C0392B",.style=SeriesStyle::points}}};
    axle.notes={"dX = "+detail::number(trajectory.axle_path_x.back()-trajectory.axle_path_x.front())+" mm",
        "dZ = "+detail::number(trajectory.axle_path_z.back()-trajectory.axle_path_z.front())+" mm"};
    return {.title="Horst-Link Rear Suspension Kinematics Analysis",.panels={std::move(leverage),std::move(stroke),std::move(axle)},.columns=3,.width=3600,.height=1160};
}
Report damper_dyno() {
    const auto velocity=geometry::detail::np_linspace(-2.,2.,201);
    Report report{.title="RockShox ZEB Ultimate (Charger 3 RC2) & Super Deluxe Ultimate Hydrodynamic Damper Curves",.panels={},.columns=3,.width=3600,.height=2200};
    for(const auto title:{"RockShox ZEB Ultimate: LSC Sweep (HSC=2, Reb=9)","RockShox ZEB Ultimate: HSC Sweep (LSC=7, Reb=9)",
        "RockShox ZEB Ultimate: Rebound Sweep (HSC=2, LSC=7)","Super Deluxe Ultimate: Damping & Threshold Lockout",
        "Super Deluxe HBO: Damping vs Stroke (v = 1.5 m/s)","Factory Presets: Fork vs Rear Force Balance"})
        report.panels.push_back({.title=title,.x_label="Shaft Velocity v (m/s)",.y_label="Damping Force F (N)",.series={}});
    constexpr std::array lsc{0,3,7,11,14},reb{0,5,9,13,17};
    constexpr std::array<std::string_view,5> lsc_names{"Click 0 (Fully Open)","Click 3 (Soft)","Click 7 (Neutral)","Click 11 (Firm)","Click 14 (Max LSC)"};
    constexpr std::array<std::string_view,5> hsc_names{"HSC 0 (Open / -2)","HSC 1 (-1)","HSC 2 (Neutral / 0)","HSC 3 (+1)","HSC 4 (Firm / +2)"};
    constexpr std::array<std::string_view,5> reb_names{"Reb 0 (Fast / Open)","Reb 5 (Fast-Mid)","Reb 9 (Neutral)","Reb 13 (Slow-Mid)","Reb 17 (Slow / Closed)"};
    for(std::size_t i=0;i<5;++i){const model::Charger3Damper a(2,lsc[i],9),b(static_cast<int>(i),7,9),c(2,7,reb[i]);
        report.panels[0].series.push_back({.name=std::string(lsc_names[i]),.x=velocity,.y=transform(velocity,[&](double v){return a.compute_damping_force(v);})});
        report.panels[1].series.push_back({.name=std::string(hsc_names[i]),.x=velocity,.y=transform(velocity,[&](double v){return b.compute_damping_force(v);})});
        report.panels[2].series.push_back({.name=std::string(reb_names[i]),.x=velocity,.y=transform(velocity,[&](double v){return c.compute_damping_force(v);})});}
    const std::array<model::SuperDeluxeDamper,4> shock{model::SuperDeluxeDamper(2,7,7),model::SuperDeluxeDamper(4,14,12),model::SuperDeluxeDamper(0,3,4),model::SuperDeluxeDamper(2,7,7,2,true)};
    const std::array<std::string_view,4> shock_names{"Open: Base Tune (HSC=2, LSC=7)","Open: Park/Firm (HSC=4, LSC=14)","Open: Plush (HSC=0, LSC=3)","Firm: Threshold Lockout Platform"};
    for(std::size_t i=0;i<shock.size();++i)report.panels[3].series.push_back({.name=std::string(shock_names[i]),.x=velocity,.y=transform(velocity,[&](double v){return shock[i].compute_damping_force(v);})});
    const auto stroke=geometry::detail::np_linspace(0.,65.,201);report.panels[4].x_label="Shock Stroke (mm)";report.panels[4].y_label="Total Damper Force (N)";
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay) decay is the designed wire idiom here
    for(int i=0;i<5;++i){const model::SuperDeluxeDamper damper(2,7,7,i);report.panels[4].series.push_back({.name="HBO Click "+std::to_string(i)+(i==0?" (Min)":i==4?" (Max)":" (Mid)"),.x=stroke,.y=transform(stroke,[&](double x){return damper.compute_damping_force(1.5,x);})});}
    report.panels[4].markers.push_back({.kind=MarkerKind::vertical,.first=52.,.second=0.,.label="HBO Start (52mm / 80%)",.color="#ec4899"});
    for(const auto preset:{model::DamperPreset::base,model::DamperPreset::plush,model::DamperPreset::enduro,model::DamperPreset::park}){model::BikeSuspensionSystem system;system.apply_preset(preset);const auto full=model::damper_preset_label(preset);const auto name=std::string(full.substr(0,full.find(' ')));
        report.panels[5].series.push_back({.name="Fork: "+name,.x=velocity,.y=transform(velocity,[&](double v){return system.fork_damper.compute_damping_force(v);})});
        report.panels[5].series.push_back({.name="Rear: "+name,.x=velocity,.y=transform(velocity,[&](double v){return system.shock_damper.compute_damping_force(v);})});}
    report.panels[0].notes={"Knee v = 0.18 m/s (Shim Stack Opens)"};report.panels[1].notes={"High-Speed Blow-Off Slope (Shim Stack Stiffness)"};
    report.panels[2].notes={"Rebound Extension Force"};report.panels[3].notes={"Pedal Platform Blow-off (~480 N Threshold)"};
    for(auto& p:report.panels){p.markers.push_back({.kind=MarkerKind::horizontal,.first=0.,.second=0.,.label={},.color="#6e7681"});p.markers.push_back({.kind=MarkerKind::vertical,.first=0.,.second=0.,.label={},.color="#6e7681"});}
    return report;
}
} // namespace bike::viz
