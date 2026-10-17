#include "builder_internal.hpp"
#include "../diag.hpp"

namespace bike::model::detail {
namespace {
Element& tube(Element& parent,std::string name,std::string type,const Vec3& a,const Vec3& b,
              std::string size,std::string mass,std::string material) {
    return geom(parent,std::move(name),std::move(type),{{"fromto",fromto(a,b)},{"size",std::move(size)},
        {"mass",std::move(mass)},{"material",std::move(material)}});
}
Element& front_wheel(Element& lower,const Vec3& axle,BuildContext& c) {
    auto& wheel=body(lower,"front_wheel",axle);
    joint(wheel,"front_wheel_spin","hinge",{{"axis","0 1 0"},{"damping","0.01"}});
    geom(wheel,"geom_front_hub","cylinder",{{"fromto","0 -0.055 0 0 0.055 0"},{"size","0.018"},{"mass",c.physical()?"0":"0.35"},{"material","mat_metal"}});
    geom(wheel,"geom_front_rim","cylinder",{{"fromto","0 -0.015 0 0 0.015 0"},{"size","0.320"},{"mass",c.physical()?"0":"0.45"},{"material","mat_rim"},{"contype","0"},{"conaffinity","0"}});
    const auto radius=f(c.request.bike.front_wheel_radius/1000.);
    const auto mask=c.request.mode=="ride"?"0":"1";
    geom(wheel,"geom_front_tire","cylinder",{{"fromto","0 -0.030 0 0 0.030 0"},{"size",radius},{"mass",c.physical()?"0":"1.35"},{"material","mat_tire"},{"contype",mask},{"conaffinity",mask},{"friction","1.2 0.005 0.0001"}});
    geom(wheel,"geom_front_rotor","cylinder",{{"fromto","0 0.032 0 0 0.034 0"},{"size","0.1015"},{"mass",c.physical()?"0":"0.25"},{"material","mat_metal"},{"contype","0"},{"conaffinity","0"}});
    if(c.request.mode=="ride") {
        const auto contact=c.request.tyre_model=="pneumatic"?"0":"1";
        geom(wheel,"geom_front_contact","sphere",{{"pos","0 0 0"},{"size",radius},{"mass","0"},{"condim","3"},{"friction","1.2 0.005 0.0001"},{"solref","-130000 -800"},{"contype",contact},{"conaffinity",contact},{"rgba","0.08 0.08 0.08 0"}});
    }
    if(c.physical()) {
        const auto value=wheel_body_inertia(c.request.masses.front_wheel_mass,kFrontWheelProfile);
        inertial(wheel,c.request.masses.front_wheel_mass,value.com,value.tensor);
    }
    return wheel;
}
void shock_assembly(Element& yoke,const Vec3& p6,const Vec3& p7,BuildContext& c) {
    const double length=geometry::detail::np_norm3(p7),stroke=c.request.bike.shock_stroke/1000.;
    const auto axis=div(mul(p7,-1.),length),up=div(p7,length);
    std::string range="0 "+f(stroke);
    if(c.request.mode=="ride" && c.physical()) {
        const auto over=c.physics->end_stops.overtravel_m;
        range=f(-over)+" "+f(stroke+over);
    }
    const auto tip=mul(up,.145);
    const double can_length=(c.request.bike.shock_eye_to_eye-c.request.bike.shock_stroke)/1000.-.002;
    const auto can_end=mul(axis,can_length),cap=mul(axis,-.014);
    const Vec3 perpendicular{-axis[2],0.,axis[0]};
    const auto offset=add(mul(perpendicular,.034),{0.,.008,0.});
    const auto piggy_start=add(mul(axis,.018),offset),piggy_end=add(mul(axis,.108),offset);
    const auto bridge1=mul(axis,.030),bridge2=add(mul(axis,.030),offset);
    auto& shaft=body(yoke,"shock_shaft",p6);
    geom(shaft,"geom_shock_lower_eyelet","cylinder",{{"fromto","0 -0.016 0 0 0.016 0"},{"size","0.011"},{"mass","0.06"},{"material","mat_metal"}});
    tube(shaft,"geom_shock_shaft","cylinder",{},tip,"0.014","0.14","mat_kashima");
    geom(shaft,"geom_shock_travel_indicator","cylinder",{{"fromto",fromto(mul(up,.016),mul(up,.021))},{"size","0.0165"},{"mass","0.0"},{"material","mat_linkage"},{"contype","0"},{"conaffinity","0"}});
    auto& damper=body(shaft,"shock_body",p7);
    if(!c.physical()) damper.add("inertial",{{"pos","-0.033395 0.011400 -0.028962"},{"quat","-0.10166406 0.38183084 -0.06020627 0.91664870"},{"mass","0.400000"},{"diaginertia","0.00084836 0.00071226 0.00023249"}});
    Attributes ja{{"axis",vec(axis)},{"range",range},{"stiffness","0"},{"damping","0"},{"springref","0"}};
    if(c.request.mode=="ride") ja.emplace_back("solreflimit","0.01 1");
    joint(damper,"shock_stroke","slide",std::move(ja));
    tube(damper,"geom_shock_trunnion_boss_l","cylinder",{0.,.019,0.},{0.,.027,0.},"0.013","0.02","mat_metal");
    tube(damper,"geom_shock_trunnion_boss_r","cylinder",{0.,-.019,0.},{0.,-.027,0.},"0.013","0.02","mat_metal");
    tube(damper,"geom_shock_trunnion_overhang","cylinder",{},cap,"0.024","0.04","mat_damper_body");
    tube(damper,"geom_shock_body","cylinder",{},can_end,"0.026","0.18","mat_damper_body");
    tube(damper,"geom_shock_piggyback","cylinder",piggy_start,piggy_end,"0.018","0.10","mat_damper_body");
    tube(damper,"geom_shock_piggy_bridge","cylinder",bridge1,bridge2,"0.010","0.04","mat_damper_body");
    site(damper,"site_P7_shock","0 0 0","0.008","0.1 0.6 0.9 1.0");
    auto parts=shaft.direct("geom");const auto more=damper.direct("geom");parts.insert(parts.end(),more.begin(),more.end());
    register_component(c.mass_registry(),"shock_damper",std::move(parts));
}
} // namespace

void build_steering_and_fork(Element& frame,BuildContext& c) {
    const auto p11=div(c.fixed.P11,1000.),ht=div(c.fixed.P_HT_bot,1000.);
    const auto fa=div(geometry::compute_front_axle(c.request.bike),1000.);
    const auto relative=sub(p11,ht),axis=div(relative,geometry::detail::np_norm3(relative)),axle=sub(fa,ht);
    auto& steer=body(frame,"steer",ht);
    const auto& mode=c.request.mode;
    if(mode=="stand" || mode=="dynamic" || mode=="playground" || mode=="ride")
        joint(steer,"steer_joint","hinge",{{"axis",vec(axis)},{"range","-0.01 0.01"},{"stiffness","50000"},{"damping","1000"}});
    else joint(steer,"steer_joint","hinge",{{"axis",vec(axis)},{"range","-60 60"},{"damping","1.5"}});
    tube(steer,"geom_steerer_tube","cylinder",{},relative,"0.014","0.25","mat_metal");
    const auto stem=add(relative,{.04,0.,.02});
    tube(steer,"geom_stem","cylinder",relative,stem,"0.016","0.15","mat_handlebar");
    tube(steer,"geom_handlebar","cylinder",lateral(stem,-.40),lateral(stem,.40),"0.0155","0.40","mat_handlebar");
    if(mode=="ride") site(steer,"site_handlebar",vec(stem),"0.008","0.9 0.6 0.1 1.0");
    geom(steer,"geom_fork_crown","box",{{"pos","0 0 0"},{"size","0.038 0.085 0.018"},{"mass","0.30"},{"material","mat_fork_lower"}});
    const auto tip=mul(axis,-.320);
    const double each=c.physical()?.425:c.request.masses.stanchions_mass/2.;
    tube(steer,"geom_stanchion_l","cylinder",{0.,.065,0.},lateral(tip,.065),"0.019",f(each,3),"mat_kashima");
    tube(steer,"geom_stanchion_r","cylinder",{0.,-.065,0.},lateral(tip,-.065),"0.019",f(each,3),"mat_kashima");
    auto& lower=steer.add("body",{{"name","fork_lower"},{"pos","0 0 0"}});
    Attributes ja{{"axis",vec(axis)},{"range","0 "+f(c.request.bike.fork_travel/1000.)},{"stiffness","0"},{"damping","0"},{"springref","0"}};
    if(mode=="ride") ja.emplace_back("solreflimit","0.01 1");
    joint(lower,"fork_travel","slide",std::move(ja));
    const auto top=add(axle,mul(axis,.330));
    tube(lower,"geom_fork_lower_l","capsule",lateral(top,.065),lateral(axle,.065),"0.022","0.55","mat_fork_lower");
    tube(lower,"geom_fork_lower_r","capsule",lateral(top,-.065),lateral(axle,-.065),"0.022","0.55","mat_fork_lower");
    tube(lower,"geom_fork_arch","capsule",lateral(top,-.065),lateral(top,.065),"0.016","0.25","mat_fork_lower");
    tube(lower,"geom_front_dropout","cylinder",lateral(axle,-.065),lateral(axle,.065),"0.012","0.15","mat_metal");
    site(lower,"site_PFA",vec(axle),"0.008","0.1 0.7 0.9 1.0");
    if(c.request.debug_markers) marker(lower,"marker_pfa",axle);
    auto& wheel=front_wheel(lower,axle,c);
    if(c.physical()) {
        std::vector<Element*> assembly,stanchions;
        for(auto* value:steer.direct("geom")) (value->get("name").starts_with("geom_stanchion_")?stanchions:assembly).push_back(value);
        register_component(c.mass_registry(),"steer_assembly",std::move(assembly));
        register_component(c.mass_registry(),"stanchions",std::move(stanchions));
        register_component(c.mass_registry(),"fork_lowers",lower.direct("geom"));
        register_component(c.mass_registry(),"front_wheel",wheel.direct("inertial"));
    }
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void build_rear_linkage(Element& frame,BuildContext& c) {
    const auto p0=div(c.fixed.P0,1000.),p5=div(c.fixed.P5,1000.),p7=div(c.fixed.P7,1000.);
    const auto p1=div(c.state.P1,1000.),p2=div(c.state.P2,1000.),p3=div(c.state.P3,1000.),p4=div(c.state.P4,1000.),p6=div(c.state.P6,1000.),p12=div(c.state.P12,1000.);
    const auto d20=sub(p2,p0),d32=sub(p3,p2),d12=sub(p1,p2),d122=sub(p12,p2);
    const auto d35=sub(p3,p5),d45=sub(p4,p5),d64=sub(p6,p4),d76=sub(p7,p6);
    const bool debug=c.request.debug_markers;
    auto& cs=body(frame,"chainstay",p0);
    joint(cs,"main_pivot","hinge",{{"axis","0 1 0"},{"damping","0.5"}});
    tube(cs,"geom_cs_sleeve","cylinder",{0.,-.040,0.},{0.,.040,0.},"0.018","0.25","mat_metal");
    tube(cs,"geom_cs_left","capsule",{0.,.045,0.},lateral(d20,.074),"0.016","0.40","mat_frame");
    tube(cs,"geom_cs_right","capsule",{0.,-.045,0.},lateral(d20,-.074),"0.016","0.40","mat_frame");
    const auto cs_bridge=mul(d20,.40);
    tube(cs,"geom_cs_bridge","capsule",lateral(cs_bridge,-.055),lateral(cs_bridge,.055),"0.014","0.20","mat_frame");
    site(cs,"site_P2_cs",vec(d20),"0.008","0.1 0.9 0.1 1.0");
    if(debug) marker(cs,"marker_p2",d20);
    register_component(c.mass_registry(),"chainstay",cs.direct("geom"));
    auto& ss=body(cs,"seatstay",d20);
    joint(ss,"horst_pivot","hinge",{{"axis","0 1 0"},{"damping","0.3"}});
    tube(ss,"geom_horst_sleeve_l","cylinder",{0.,.068,0.},{0.,.080,0.},"0.013","0.075","mat_metal");
    tube(ss,"geom_horst_sleeve_r","cylinder",{0.,-.080,0.},{0.,-.068,0.},"0.013","0.075","mat_metal");
    tube(ss,"geom_ss_left","capsule",lateral(d122,.074),lateral(d32,.045),"0.015","0.25","mat_frame");
    tube(ss,"geom_ss_right","capsule",lateral(d122,-.074),lateral(d32,-.045),"0.015","0.25","mat_frame");
    for(int index=0;index<2;++index) {
        const std::string side=index==0?"_l":"_r";const double y=index==0?.074:-.074;
        tube(ss,"geom_dropout_bottom"+side,"capsule",{0.,y,0.},lateral(d12,y),"0.016","0.045","mat_frame");
        tube(ss,"geom_dropout_rear"+side,"capsule",lateral(d12,y),lateral(d122,y),"0.016","0.045","mat_frame");
        tube(ss,"geom_dropout_front"+side,"capsule",{0.,y,0.},lateral(d122,y),"0.015","0.045","mat_frame");
    }
    const auto ss_bridge=mul(d32,.70);
    tube(ss,"geom_ss_bridge","capsule",lateral(ss_bridge,-.050),lateral(ss_bridge,.050),"0.013","0.13","mat_frame");
    site(ss,"site_P3_ss",vec(d32),"0.008","0.9 0.5 0.1 1.0");
    site(ss,"site_P12_ss",vec(d122),"0.007","0.9 0.6 0.1 1.0");
    if(debug) marker(ss,"marker_p12",d122,"0.011");
    site(ss,"site_PRA",vec(d12),"0.008","0.2 0.4 0.9 1.0");
    if(debug) marker(ss,"marker_pra",d12);
    build_rear_wheel(ss,d12,c);
    register_component(c.mass_registry(),"seatstay",ss.direct("geom"));
    auto& rocker=body(frame,"rocker",p5);
    joint(rocker,"rocker_frame_pivot","hinge",{{"axis","0 1 0"},{"damping","0.3"}});
    tube(rocker,"geom_rocker_sleeve","cylinder",{0.,-.045,0.},{0.,.045,0.},"0.016","0.16","mat_metal");
    for(int arm=1;arm<=3;++arm) for(int index=0;index<2;++index) {
        const double y=index==0?.045:-.045;const std::string side=index==0?"l":"r";
        tube(rocker,"geom_roc_arm_"+side+std::to_string(arm),"capsule",arm==3?lateral(d35,y):Vec3{0.,y,0.},
            lateral(arm==1?d35:d45,y),arm==3?"0.012":"0.014","0.04","mat_linkage");
    }
    site(rocker,"site_P3_rocker",vec(d35),"0.008","0.9 0.5 0.1 1.0");
    if(debug) {marker(rocker,"marker_p3_roc_l",lateral(d35,.045),"0.011");marker(rocker,"marker_p3_roc_r",lateral(d35,-.045),"0.011");}
    site(rocker,"site_P4_rocker",vec(d45),"0.008","0.7 0.2 0.8 1.0");
    if(debug) {marker(rocker,"marker_p4_l",lateral(d45,.045));marker(rocker,"marker_p4_r",lateral(d45,-.045));}
    register_component(c.mass_registry(),"rocker",rocker.direct("geom"));
    auto& yoke=body(rocker,"shock_yoke",d45);
    joint(yoke,"yoke_pivot","hinge",{{"axis","0 1 0"},{"damping","0.2"}});
    tube(yoke,"geom_yoke_pivot_p4","cylinder",{0.,-.044,0.},{0.,.044,0.},"0.011","0.06","mat_metal");
    tube(yoke,"geom_yoke_arm_l","capsule",{0.,.040,0.},lateral(d64,.038),"0.011","0.08","mat_yoke");
    tube(yoke,"geom_yoke_arm_r","capsule",{0.,-.040,0.},lateral(d64,-.038),"0.011","0.08","mat_yoke");
    const auto bridge=mul(d64,.75);
    tube(yoke,"geom_yoke_bridge","capsule",lateral(bridge,-.038),lateral(bridge,.038),"0.009","0.04","mat_yoke");
    tube(yoke,"geom_yoke_eyelet","cylinder",lateral(d64,-.040),lateral(d64,.040),"0.011","0.04","mat_metal");
    site(yoke,"site_P6_yoke",vec(d64),"0.008","0.6 0.1 0.7 1.0");
    if(debug) {marker(yoke,"marker_p6_l",lateral(d64,.038));marker(yoke,"marker_p6_r",lateral(d64,-.038));}
    register_component(c.mass_registry(),"shock_yoke",yoke.direct("geom"));
    shock_assembly(yoke,d64,d76,c);
}
NATIVE_DIAG_POP
void build_equality_constraints(Element& root,std::string_view mode) {
    auto& equality=root.add("equality");
    equality.add("connect",{{"name","seatstay_rocker_joint"},{"site1","site_P3_ss"},{"site2","site_P3_rocker"},{"solref","0.0005 1"},{"solimp","0.99 0.999 0.0001 0.5 2"}});
    equality.add("connect",{{"name","shock_frame_joint"},{"site1","site_P7_shock"},{"site2","site_P7"},{"solref","0.0005 1"},{"solimp","0.99 0.999 0.0001 0.5 2"}});
    if(mode=="playground") equality.add("weld",{{"name","stand_clamp"},{"body1","world"},{"body2","frame"},{"active","true"}});
}
void build_contact_exclusions(Element& root) {
    auto& contact=root.add("contact");
    constexpr std::array names{"frame","steer","fork_lower","chainstay","seatstay","rocker","shock_yoke","shock_shaft","shock_body","front_wheel","rear_wheel"};
    for(std::size_t i=0;i<names.size();++i) for(std::size_t j=i+1;j<names.size();++j)
        contact.add("exclude",{{"body1",names[i]},{"body2",names[j]}});
}
} // namespace bike::model::detail
