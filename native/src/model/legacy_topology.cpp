#include "builder_internal.hpp"
#include <stdexcept>

namespace bike::model::detail {
namespace {
void build_lumped_rider(Element& frame,const configuration::RiderSpecs& rider) {
    const bool visible=rider.variant=="lumped";
    struct Part {const char* name;const char* fromto;const char* size;double mass;};
    const Part parts[]{
        {"torso","0.10 0 0.48 0.22 0 0.82","0.13",rider.torso_helmet_mass()},
        {"legs","0.0 0 0.0 0.10 0 0.48","0.075",rider.legs_mass()},
        {"arms","0.22 0 0.78 0.45 0 0.66","0.055",rider.arms_mass()},
    };
    for(const auto& part:parts) geom(frame,"geom_rider_"+std::string(part.name),"capsule",{{"fromto",part.fromto},
        {"size",visible?part.size:"0.0001"},{"mass",visible?f(part.mass,1):"0.0"},
        {"rgba",visible?"0.5 0.5 0.5 1.0":"0.5 0.5 0.5 0.0"},{"material","mat_rider"},{"contype","0"},{"conaffinity","0"}});
}
void build_seated_rider(Element& frame,const SeatedPose& pose) {
    std::map<std::string,Element*> elements{{"frame",&frame}};
    std::map<std::string,Vec3> origins{{"frame",{0.,0.,0.}}};
    for(const auto& spec:pose.bodies) {
        const auto parent=elements.find(spec.parent);
        if(parent==elements.end()) throw std::invalid_argument("rider body '"+spec.name+"' names parent '"+spec.parent+"' before it is built");
        auto& element=body(*parent->second,spec.name,sub(spec.attach,origins.at(spec.parent)));
        joint(element,spec.joint,"slide",{{"axis","0 0 1"}});
        for(const auto& shape:spec.geoms) {
            Attributes attributes;
            if(shape.kind=="sphere") attributes.emplace_back("pos",vec(sub(shape.p1,spec.attach)));
            else attributes.emplace_back("fromto",fromto(sub(shape.p1,spec.attach),sub(shape.p2,spec.attach)));
            attributes.insert(attributes.end(),{{"size",f(shape.radius,3)},{"mass",f(shape.mass,6)},
                {"rgba","0.5 0.5 0.5 1.0"},{"material","mat_rider"},{"contype","0"},{"conaffinity","0"}});
            geom(element,shape.name,shape.kind=="sphere"?"sphere":"capsule",std::move(attributes));
        }
        elements[spec.name]=&element;origins[spec.name]=spec.attach;
    }
    for(const auto* part:{"torso","pelvis"}) {
        const std::string name="rider_"+std::string(part);
        const auto& spec=pose.body(name);
        site(*elements.at(name),"site_"+name,vec(sub(spec.center_of_mass(),spec.attach)),"0.008","0.9 0.6 0.1 1.0");
    }
    for(const auto& chain:pose.leg_chains) {
        auto& thigh=body(*elements.at("rider_pelvis"),"rider_thigh_"+chain.side,{0.,chain.lateral_y_m,0.});
        joint(thigh,"rider_hip_"+chain.side,"hinge",{{"axis","0 1 0"}});
        const auto segment=[&](Element& owner,std::string name,const Vec3& vector,double radius,double mass) {
            geom(owner,std::move(name),"capsule",{{"fromto",fromto({},vector)},{"size",f(radius,3)},{"mass",f(mass,6)},
                {"rgba","0.5 0.5 0.5 1.0"},{"material","mat_rider"},{"contype","0"},{"conaffinity","0"}});
        };
        segment(thigh,"geom_rider_thigh_"+chain.side,sub(chain.knee,chain.hip),segment_radius_m("thigh"),chain.thigh_mass_kg);
        auto& shank=body(thigh,"rider_shank_"+chain.side,sub(chain.knee,chain.hip));
        joint(shank,"rider_knee_"+chain.side,"hinge",{{"axis","0 1 0"}});
        segment(shank,"geom_rider_shank_"+chain.side,sub(chain.ankle,chain.knee),segment_radius_m("shank"),chain.shank_mass_kg);
        auto& foot=body(shank,"rider_foot_"+chain.side,sub(chain.ankle,chain.knee));
        joint(foot,"rider_ankle_"+chain.side,"hinge",{{"axis","0 1 0"}});
        segment(foot,"geom_rider_foot_"+chain.side,sub(chain.pedal,chain.ankle),segment_radius_m("foot"),chain.foot_mass_kg);
        site(foot,"site_foot_"+chain.side,vec(sub(chain.pedal,chain.ankle)),"0.006");
    }
}
void front_triangle(Element& frame,const BuildContext& c) {
    const Vec3 bb{};
    const auto ht=div(c.fixed.P_HT_bot,1000.),p11=div(c.fixed.P11,1000.),p8=div(c.fixed.P8,1000.),p10=div(c.fixed.P10,1000.);
    const auto middle=div(add(bb,ht),2.),axis=sub(ht,bb);
    const double length=geometry::detail::np_norm3(axis),angle=geometry::detail::np_degrees(std::atan2(axis[2],axis[0]));
    const double sine=axis[2]/length,cosine=axis[0]/length;
    const double half=(((266.-150.)*sine/2.)+((305.-208.)*sine/2.))/2./1000.;
    const double offset=(((150.+266.)/2.*-sine+255.*cosine)+((208.+305.)/2.*-sine+323.*cosine))/2./1000.;
    const Vec3 perpendicular{-sine,0.,cosine};
    geom(frame,"geom_downtube","box",{{"pos",vec(add(middle,mul(perpendicular,offset)))},{"size",f(length/2.)+" 0.038 "+f(half)},{"euler","0 "+f(-angle)+" 0"},{"mass","0.60"},{"material","mat_frame"}});
    geom(frame,"geom_battery_pack","capsule",{{"fromto",fromto(add({.080,0.,.080},mul(perpendicular,offset)),add({.320,0.,.320},mul(perpendicular,offset)))},{"size","0.032"},{"mass",c.physical()?"4.30":f(c.request.masses.battery_mass,2)},{"material","mat_battery"}});
    geom(frame,"geom_headtube","cylinder",{{"fromto",fromto(ht,p11)},{"size","0.027"},{"mass","0.40"},{"material","mat_frame"}});
    geom(frame,"geom_toptube_bridge","capsule",{{"fromto",fromto(p11,p8)},{"size","0.024"},{"mass","0.25"},{"material","mat_frame"}});
    geom(frame,"geom_toptube_main","capsule",{{"fromto",fromto(p8,p10)},{"size","0.023"},{"mass","0.45"},{"material","mat_frame"}});
}
Vec3 seatpost_and_saddle(Element& frame,const BuildContext& c) {
    const auto p10=div(c.fixed.P10,1000.),p9=div(c.fixed.P9,1000.),p8=div(c.fixed.P8,1000.),p7=div(c.fixed.P7,1000.),p5=div(c.fixed.P5,1000.);
    const Vec3 end{-.030,0.,.258};
    geom(frame,"geom_seattube","capsule",{{"fromto",fromto(p10,end)},{"size","0.020"},{"mass","0.05"},{"material","mat_frame"}});
    geom(frame,"geom_frame_casting","box",{{"pos",f((end[0]+p5[0])/2.)+" 0 "+f((end[2]+0.)/2.)},{"size","0.075 0.036 0.130"},{"mass",f(.65-.05,2)},{"material","mat_frame"},{"contype","0"},{"conaffinity","0"}});
    std::optional<double> height;
    if(c.seated) height=c.seated->saddle.top_z_m;
    if(c.articulated) height=c.articulated->saddle.top_z_m;
    const auto saddle=geometry::saddle_geometry(p9,height);
    const auto masses=geometry::saddle_post_masses(c.physical()?.90:c.request.masses.saddle_post_mass,p10,p9,saddle);
    geom(frame,"geom_seatpost","cylinder",{{"fromto",fromto(p10,p9)},{"size","0.016"},{"mass",f(masses.lower_post)},{"material","mat_metal"}});
    geom(frame,"geom_seatpost_upper","cylinder",{{"fromto",fromto(p9,saddle.seatpost_top)},{"size","0.016"},{"mass",f(masses.upper_post)},{"material","mat_metal"}});
    geom(frame,"geom_seat_gusset","capsule",{{"fromto",fromto(p9,add(p10,mul(sub(p8,p10),.22)))},{"size","0.015"},{"mass","0.10"},{"material","mat_frame"}});
    geom(frame,"geom_frame_junction","capsule",{{"fromto",fromto(add(p10,mul(sub(p8,p10),.52)),p7)},{"size","0.042"},{"mass","0.10"},{"material","mat_frame"}});
    geom(frame,"geom_saddle","box",{{"pos",vec(saddle.saddle_pos)},
        {"size",g(geometry::SADDLE_HALF_LENGTH_M,6)+" "+g(geometry::SADDLE_HALF_WIDTH_M,6)+" "+f(geometry::SADDLE_HALF_THICKNESS_M,3)},
        {"mass",f(masses.saddle,2)},{"material","mat_saddle"}});
    return saddle.seatpost_top;
}
void mounts_and_sites(Element& frame,const BuildContext& c,const Vec3& seatpost) {
    const auto p0=div(c.fixed.P0,1000.),p5=div(c.fixed.P5,1000.),p7=div(c.fixed.P7,1000.),p8=div(c.fixed.P8,1000.),p9=div(c.fixed.P9,1000.),p10=div(c.fixed.P10,1000.),p11=div(c.fixed.P11,1000.),ht=div(c.fixed.P_HT_bot,1000.);
    const auto mount=[&](const char* name,const Vec3& point,double width,const char* size) {
        geom(frame,name,"cylinder",{{"fromto",fromto(lateral(point,-width),lateral(point,width))},{"size",size},{"mass","0.10"},{"material","mat_metal"}});
    };
    mount("geom_p0_mount",p0,.035,"0.016");mount("geom_p5_mount",p5,.048,"0.013");mount("geom_p7_mount",p7,.020,"0.012");
    site(frame,"site_BB",vec({}),"0.008","0.9 0.6 0.1 1.0");
    site(frame,"site_P0",vec(p0),"0.008","0.2 0.8 0.2 1.0");
    site(frame,"site_P5",vec(p5),"0.008","0.8 0.2 0.2 1.0");
    site(frame,"site_P7",vec(p7),"0.008","0.1 0.6 0.9 1.0");
    site(frame,"site_P8",vec(p8),"0.006","0.5 0.5 0.5 1.0");
    site(frame,"site_P9",vec(p9),"0.006","0.5 0.5 0.5 1.0");
    site(frame,"site_P10",vec(p10),"0.006","0.5 0.5 0.5 1.0");
    site(frame,"site_P11",vec(p11),"0.008","0.9 0.1 0.1 1.0");
    site(frame,"site_HT_bot",vec(ht),"0.008","0.9 0.1 0.1 1.0");
    site(frame,"site_CG",vec(c.cg),"0.012","0.95 0.20 0.85 0.9");
    site(frame,"site_top_tube_kink",vec(p8),"0.005","0.5 0.5 0.5 0.8");
    site(frame,"site_seat_collar",vec(p9),"0.005","0.5 0.5 0.5 0.8");
    site(frame,"site_seatpost_top",vec(seatpost),"0.005","0.5 0.5 0.5 0.8");
    site(frame,"site_seat_tube_junction",vec(p10),"0.005","0.5 0.5 0.5 0.8");
    site(frame,"site_headtube_top",vec(p11),"0.006","0.9 0.1 0.1 0.8");
    site(frame,"site_headtube_bottom",vec(ht),"0.006","0.9 0.1 0.1 0.8");
    if(c.request.debug_markers) {
        marker(frame,"marker_bb",{});marker(frame,"marker_p0",p0);
        marker(frame,"marker_p5_l",lateral(p5,.048));marker(frame,"marker_p5_r",lateral(p5,-.048));
        marker(frame,"marker_p7",p7);marker(frame,"marker_p8",p8,"0.011");
        marker(frame,"marker_p9",p9,"0.011");marker(frame,"marker_p10",p10,"0.011");
        marker(frame,"marker_p11",p11);marker(frame,"marker_cg",c.cg,"0.014","mat_cg_marker");
    }
}
} // namespace
Element& build_frame_body(Element& world,BuildContext& c) {
    auto& frame=world.add("body",{{"name","frame"},{"pos","0 0 0"}});
    if(c.request.mode=="ride") {
        joint(frame,"root_x","slide",{{"axis","1 0 0"}});
        joint(frame,"root_z","slide",{{"axis","0 0 1"}});
        joint(frame,"root_pitch","hinge",{{"axis","0 1 0"}});
    }
    if(c.seated) build_seated_rider(frame,*c.seated);
    else if(c.request.mode=="stand" || c.request.mode=="playground" || c.request.mode=="ride") build_lumped_rider(frame,c.rider);
    build_bb_and_motor(frame,c);
    front_triangle(frame,c);
    const auto top=seatpost_and_saddle(frame,c);
    mounts_and_sites(frame,c,top);
    if(c.physical()) {
        register_component(c.mass_registry(),"battery",{&required(frame,"geom","geom_battery_pack")});
        register_component(c.mass_registry(),"saddle_post",{&required(frame,"geom","geom_seatpost"),&required(frame,"geom","geom_seatpost_upper"),&required(frame,"geom","geom_saddle")});
        std::set<Element*> excluded;
        for(const auto& [name,parts]:c.registry) { (void)name;excluded.insert(parts.begin(),parts.end()); }
        std::vector<Element*> remaining;
        for(auto* value:frame.direct("geom")) if(!excluded.contains(value) && !value->get("name").starts_with("geom_rider_")) remaining.push_back(value);
        register_component(c.mass_registry(),"frame_structure",std::move(remaining));
    }
    return frame;
}
} // namespace bike::model::detail
