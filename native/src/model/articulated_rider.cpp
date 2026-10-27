#include "builder_internal.hpp"
#include <algorithm>
#include <numbers>
#include <stdexcept>

namespace bike::model::detail {
namespace {
void finish_spatial_rider(Element& root, const BuildContext& c) {
    const auto& cfg = c.physics->articulated;
    if (cfg.pedal_attachment != "spindle" || cfg.saddle_attachment != "pin" ||
        cfg.grip_attachment != "connect" || cfg.coupled_task_control)
        throw std::invalid_argument("rider.spatial: requires spindle pedals, pin saddle and connect grips");
    const auto profile = resolve_shoulder_profile(cfg);
    auto& pelvis = required(root, "body", "rider_pelvis");
    for (auto* value : pelvis.direct("joint"))
        if (value->get("name").starts_with("rider_root_")) (void)pelvis.remove(value);
    pelvis.add("freejoint", {{"name", "rider_root_free"}});
    auto& actuators = block(root, "actuator");
    for (int side = 0; side < 2; ++side) {
        const std::string suffix = side == 0 ? "left" : "right";
        const std::string name = side == 0 ? "shoulder_swivel_l" : "shoulder_swivel_r";
        auto& upper = required(root, "body", "rider_upper_arm_" + suffix);
        const double range_deg = profile.swivel_range_rad * 180. / std::numbers::pi;
        // Body Z swivel precedes the existing body Y pitch. Adding a joint
        // to the existing segment preserves every mass, COM and inertia.
        auto swivel = std::make_unique<Element>("joint", Attributes{
            {"name", name}, {"type", "hinge"}, {"axis", "0 0 1"},
            {"limited", "true"}, {"range", repr(-range_deg)+" "+repr(range_deg)},
            {"stiffness", repr(profile.swivel_stiffness_nm_per_rad)},
            {"damping", repr(profile.swivel_damping_nms_per_rad)}, {"springref", "0"}});
        upper.children.insert(upper.children.begin(), std::move(swivel));
        const std::string limits = repr(-profile.swivel_strength_nm)+" "+repr(profile.swivel_strength_nm);
        actuators.add("motor", {{"name", "act_"+name}, {"joint", name}, {"gear", "1"},
            {"ctrllimited", "true"}, {"ctrlrange", limits},
            {"forcelimited", "true"}, {"forcerange", limits}});
        auto& steer = required(root, "body", "steer");
        const Vec3 hand = add(c.articulated->grip, {0., side == 0 ? -ARM_LATERAL_OFFSET_M : ARM_LATERAL_OFFSET_M, 0.}); // NOLINT(bugprone-unchecked-optional-access) finish_spatial_rider only runs under the caller's context.articulated guard
        const Vec3 steer_origin = div(c.fixed.P_HT_bot, 1000.);
        site(steer, "site_grip_"+suffix, vec17(sub(hand, steer_origin)), ".004");
        auto& connect = required(root, "connect", "connect_grip_"+suffix);
        std::erase_if(connect.attributes, [](const auto& attribute) {
            return attribute.first == "body1" || attribute.first == "body2" || attribute.first == "anchor";
        });
        connect.set("site1", "site_rider_grip_"+suffix);
        connect.set("site2", "site_grip_"+suffix);
    }
    if (c.physics->timestep_s > .01)
        throw std::invalid_argument("rider.spatial: attachment compliance requires timestep <= 0.01 s");
    const double time_constant = std::clamp(c.physics->closure_time_constant_s,
        std::max(2.*c.physics->timestep_s, .001), .02);
    for (const char* name : {"connect_grip_left", "connect_grip_right", "connect_foot_front", "connect_foot_rear", "connect_saddle"}) {
        auto& connect = required(root, "connect", name);
        connect.set("solref", repr(time_constant)+" 1");
        connect.set("solimp", "0.98 0.999 0.001 0.5 2");
    }
    for (const char* name : {"geom_rider_head", "geom_rider_pelvis", "geom_rider_torso"}) {
        auto& value = required(root, "geom", name);
        value.set("contype", "1"); value.set("conaffinity", "1");
    }
}
}
void build_articulated_rider(Element& world,const ArticulatedPose& pose,const SegmentMasses& masses,
                             const configuration::JointEnvelopeMap* envelopes,bool lock_ankles) {
    std::set<std::string> expected{"pelvis","torso","head"},actual;
    for(const char* side:{"left","right"}) for(const char* part:{"upper_arm","forearm"}) expected.insert(std::string(part)+"_"+side);
    for(const char* side:{"front","rear"}) for(const char* part:{"thigh","shank","foot"}) expected.insert(std::string(part)+"_"+side);
    for(const auto& [name,mass]:masses) {(void)mass;actual.insert(name);}
    if(actual!=expected || actual.size()!=masses.size()) throw std::invalid_argument("incomplete anatomical segment mass budget");
    const auto mass_for=[&](std::string_view name) {return std::ranges::find_if(masses,[&](const auto& item){return item.first==name;})->second;};
    std::map<Element*,Vec3> origins;
    const auto segment=[&](const std::string& key,Element& parent,const Vec3& start,const Vec3& end,double radius,std::string joint_name={}) -> Element& {
        const Vec3 origin=&parent==&world?Vec3{}:origins.at(&parent);
        auto& owner=parent.add("body",{{"name","rider_"+key},{"pos",vec17(sub(start,origin))}});
        origins[&owner]=start;
        if(!joint_name.empty() && !(lock_ankles && joint_name.starts_with("rider_ankle_"))) {
            Attributes attributes{{"name",joint_name},{"type","hinge"},{"axis","0 1 0"},{"damping","0"}};
            if(envelopes) {
                const auto it=std::ranges::find_if(*envelopes,[&](const auto& item){return item.first==joint_name;});
                if(it==envelopes->end()) throw std::invalid_argument("missing joint envelope: "+joint_name);
                const auto range=configuration::joint_q_range(it->second);
                attributes.emplace_back("limited","true");
                attributes.emplace_back("range",g(geometry::detail::np_degrees(range[0]))+" "+g(geometry::detail::np_degrees(range[1])));
            }
            owner.add("joint",std::move(attributes));
        }
        const auto vector=sub(end,start);const double mass=mass_for(key);
        inertial(owner,mass,div(vector,2.),segment_inertia(mass,vector,radius));
        geom(owner,"geom_rider_"+key,"capsule",{{"fromto","0 0 0 "+vec17(vector)},{"size",repr(radius)},
            {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});
        return owner;
    };
    auto& pelvis=segment("pelvis",world,pose.hip,add(pose.hip,{0.,0.,.12}),.085);
    pelvis.add("joint",{{"name","rider_root_x"},{"type","slide"},{"axis","1 0 0"},{"damping","0"}});
    pelvis.add("joint",{{"name","rider_root_z"},{"type","slide"},{"axis","0 0 1"},{"damping","0"}});
    pelvis.add("joint",{{"name","rider_root_pitch"},{"type","hinge"},{"axis","0 1 0"},{"damping","0"}});
    site(pelvis,"site_rider_saddle","0 0 -0.060",".004");
    auto& torso=segment("torso",pelvis,pose.hip,pose.shoulder,.11,"rider_torso_hinge");
    auto& head=torso.add("body",{{"name","rider_head"},{"pos",vec17(sub(pose.head_center,pose.hip))}});
    const double head_mass=mass_for("head"),moment=.4*head_mass*.11*.11;
    inertial(head,head_mass,{},Mat3{moment,0.,0.,0.,moment,0.,0.,0.,moment});
    geom(head,"geom_rider_head","sphere",{{"size","0.11"},{"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".6 .6 .6 1"}});
    for(int index=0;index<2;++index) {
        const std::string side=index==0?"left":"right";const double sign=index==0?-1.:1.;
        const Vec3 offset{0.,sign*ARM_LATERAL_OFFSET_M,0.};
        const auto shoulder=add(pose.shoulder,offset),elbow=add(pose.elbow,offset),grip=add(pose.grip,offset);
        auto& upper=segment("upper_arm_"+side,torso,shoulder,elbow,.045,"rider_shoulder_"+side);
        auto& forearm=segment("forearm_"+side,upper,elbow,grip,.04,"rider_elbow_"+side);
        site(forearm,"site_rider_grip_"+side,vec17(sub(grip,elbow)),".004");
    }
    for(int index=0;index<2;++index) {
        const std::string side=index==0?"front":"rear";const double y=index==0?-.115:.115;
        const auto hip=lateral(pose.hip,y),knee=lateral(index==0?pose.knee_front:pose.knee_rear,y);
        const auto ankle=lateral(index==0?pose.ankle_front:pose.ankle_rear,y),pedal=lateral(index==0?pose.pedal_front:pose.pedal_rear,y);
        auto& thigh=segment("thigh_"+side,pelvis,hip,knee,.07,"rider_hip_"+side);
        auto& shank=segment("shank_"+side,thigh,knee,ankle,.055,"rider_knee_"+side);
        auto& foot=segment("foot_"+side,shank,ankle,pedal,.035,"rider_ankle_"+side);
        site(foot,"site_rider_sole_"+side,vec17(add(sub(pedal,ankle),{0.,0.,.008})),".004");
    }
}
void add_rider_actuators(Element& root,const configuration::ArticulatedConfig& cfg) {
    auto& actuators=block(root,"actuator");
    for(auto const* owner:root.descendants("body")) for(auto* value:owner->direct("joint")) {
        const auto name=value->get("name");
        if(name.starts_with("rider_") && !name.starts_with("rider_root_")) {
            value->set("damping",g(cfg.passive_damping_nms_rad()));
            actuators.add("motor",{{"name","act_"+name},{"joint",name},{"gear","1"},{"ctrllimited","false"},{"forcelimited","false"}});
        }
    }
}
void add_saddle_pin(Element& root,double solref) {
    if(!std::isfinite(solref) || solref<=0.) throw std::invalid_argument("saddle pin requires a positive time constant");
    auto* equality=root.child("equality");auto const* site_=root.find("site","site_rider_saddle");
    if(!equality || !site_) throw std::invalid_argument("saddle pin requires an equality block and saddle site");
    if(auto* weld=equality->find("weld","weld_saddle")) (void)equality->remove(weld);
    equality->add("connect",{{"name","connect_saddle"},{"body1","rider_pelvis"},{"body2","frame"},{"anchor",site_->get("pos","0 0 0")},{"solref",g(solref)+" 1"}});
}
void finish_rider_attachments(Element& root,const BuildContext& c) {
    const auto& cfg=*c.physics;
    const double solref=std::max(2.*cfg.timestep_s,cfg.closure_time_constant_s);
    auto& equality=block(root,"equality");
    for(const char* side:{"front","rear"}) {
        const std::string suffix=side;
        if(cfg.articulated.pedal_attachment=="weld") equality.add("weld",{{"name","weld_foot_"+suffix},{"body1","rider_foot_"+suffix},{"body2","pedal_"+suffix},{"solref",g(solref)+" 1"}});
        if(cfg.articulated.pedal_attachment=="spindle") equality.add("connect",{{"name","connect_foot_"+suffix},{"body1","rider_foot_"+suffix},{"body2","pedal_"+suffix},{"anchor","0 0 "+g(-ANKLE_ABOVE_PEDAL_M)},{"solref",g(solref)+" 1"}});
    }
    if(cfg.articulated.saddle_attachment=="weld") equality.add("weld",{{"name","weld_saddle"},{"body1","rider_pelvis"},{"body2","frame"},{"solref",g(solref)+" 1"}});
    else if(cfg.articulated.saddle_attachment=="pin") add_saddle_pin(root,solref);
    if(cfg.articulated.grip_attachment=="connect") for(const char* side:{"left","right"}) {
        const std::string suffix=side;
        const auto& position=required(root,"site","site_rider_grip_"+suffix);
        equality.add("connect",{{"name","connect_grip_"+suffix},{"body1","rider_forearm_"+suffix},{"body2","steer"},{"anchor",position.get("pos")},{"solref",g(solref)+" 1"}});
    }
    for(const char* name:{"geom_rider_head","geom_rider_pelvis","geom_rider_torso"}) {
        auto& value=required(root,"geom",name);value.set("contype","4");value.set("conaffinity","0");
    }
    for(const char* name:{"terrain","catch_plane"}) {
        auto& value=required(root,"geom",name);value.set("conaffinity",std::to_string((static_cast<unsigned>(std::stoi(value.get("conaffinity","1")))|4U)));
    }
    if (c.spatial()) finish_spatial_rider(root, c);
}
} // namespace bike::model::detail
