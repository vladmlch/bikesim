#include "builder_internal.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bike::model::detail {
namespace {
struct RadiiOfGyration { double sag; double trans; double lon; };

void planar_segment_inertia(Mat3& out, double mass, const Vec3& vector, RadiiOfGyration k) {
    const double length = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    if (length <= 0.) throw std::invalid_argument("segment length must be positive");
    const double ux = vector[0] / length;
    const double uz = vector[2] / length;
    const double l_sag = k.sag * length;
    const double l_trans = k.trans * length;
    const double l_long = k.lon * length;
    const double i_sag = mass * l_sag * l_sag;
    const double i_trans = mass * l_trans * l_trans;
    const double i_long = mass * l_long * l_long;
    const double ixx = i_trans * uz * uz + i_long * ux * ux;
    const double iyy = i_sag;
    const double izz = i_trans * ux * ux + i_long * uz * uz;
    const double ixz = (i_long - i_trans) * ux * uz;
    out = Mat3{ixx, 0., ixz, 0., iyy, 0., ixz, 0., izz};
}
} // namespace

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
            Attributes attributes{{"name",joint_name},{"type","hinge"},{"axis","0 1 0"}};
            if(envelopes) {
                const auto it=std::ranges::find_if(*envelopes,[&](const auto& item){return item.first==joint_name;});
                if(it==envelopes->end()) throw std::invalid_argument("missing joint envelope: "+joint_name);
                auto env=it->second;
                env.neutral_anatomical_rad=pose_neutral_anatomical_rad(pose,joint_name);
                const auto range=configuration::joint_q_range(env);
                attributes.emplace_back("limited","true");
                attributes.emplace_back("range",g(geometry::detail::np_degrees(range[0]))+" "+g(geometry::detail::np_degrees(range[1])));
            }
            owner.add("joint",std::move(attributes));
        }
        const auto vector=sub(end,start);const double mass=mass_for(key);
        Vec3 com=div(vector,2.);
        Mat3 tensor{};
        if(key.starts_with("thigh_")) {
            constexpr double kComFrac=0.4095;
            com=mul(vector,kComFrac);
            planar_segment_inertia(tensor,mass,vector,{.sag=0.329,.trans=0.329,.lon=0.149});
        } else if(key.starts_with("shank_")) {
            constexpr double kComFrac=0.4459;
            com=mul(vector,kComFrac);
            planar_segment_inertia(tensor,mass,vector,{.sag=0.251,.trans=0.246,.lon=0.102});
        } else if(key.starts_with("foot_")) {
            constexpr double kRealFootLengthM=0.2668;
            constexpr double kSagK=0.257;
            constexpr double kTransK=0.245;
            constexpr double kLongK=0.124;
            const double l_long=kLongK*kRealFootLengthM;
            const double l_sag=kSagK*kRealFootLengthM;
            const double l_trans=kTransK*kRealFootLengthM;
            const double i_long=mass*l_long*l_long;
            const double i_sag=mass*l_sag*l_sag;
            const double i_trans=mass*l_trans*l_trans;
            tensor=Mat3{i_long,0.,0.,0.,i_sag,0.,0.,0.,i_trans};
        } else {
            tensor=segment_inertia(mass,vector,radius);
        }
        inertial(owner,mass,com,tensor);
        geom(owner,"geom_rider_"+key,"capsule",{{"fromto","0 0 0 "+vec17(vector)},{"size",repr(radius)},
            {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});
        return owner;
    };
    auto& pelvis=segment("pelvis",world,pose.hip,add(pose.hip,{0.,0.,.12}),.085);
    pelvis.add("joint",{{"name","rider_root_x"},{"type","slide"},{"axis","1 0 0"}});
    pelvis.add("joint",{{"name","rider_root_z"},{"type","slide"},{"axis","0 0 1"}});
    pelvis.add("joint",{{"name","rider_root_pitch"},{"type","hinge"},{"axis","0 1 0"}});
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
void add_rider_actuators(Element& root) {
    auto& actuators=block(root,"actuator");
    for(auto const* owner:root.descendants("body")) for(auto const* value:owner->direct("joint")) {
        const auto name=value->get("name");
        if(name.starts_with("rider_") && !name.starts_with("rider_root_")) {
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
        if(cfg.articulated.pedal_attachment=="spindle") {
            const Vec3 anchor=c.articulated?sub(suffix=="front"?c.articulated->pedal_front:c.articulated->pedal_rear,
                                                suffix=="front"?c.articulated->ankle_front:c.articulated->ankle_rear)
                                           :Vec3{0.,0.,-ANKLE_ABOVE_PEDAL_M};
            equality.add("connect",{{"name","connect_foot_"+suffix},{"body1","rider_foot_"+suffix},{"body2","pedal_"+suffix},{"anchor",vec17(anchor)},{"solref",g(solref)+" 1"}});
        }
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
}
} // namespace bike::model::detail
