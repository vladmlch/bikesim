#include "articulated_rider.hpp"
#include "builder_internal.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

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
        const bool passive=name.starts_with("rider_root_") ||
                           name=="rider_saddle_pitch" ||
                           name.starts_with("rider_hip_abduction_");
        if(name.starts_with("rider_") && !passive) {
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

void build_articulated_rider_3d(Element& world,const ArticulatedPose3d& pose,const SegmentMasses& masses,
                                const configuration::JointEnvelopeMap* envelopes,bool lock_ankles) {
    std::set<std::string> expected{"pelvis","torso","head"},actual;
    for(const char* side:{"left","right"}) for(const char* part:{"upper_arm","forearm"}) expected.insert(std::string(part)+"_"+side);
    for(const char* side:{"front","rear"}) for(const char* part:{"thigh","shank","foot"}) expected.insert(std::string(part)+"_"+side);
    for(const auto& [name,mass]:masses) {(void)mass;actual.insert(name);}
    if(actual!=expected || actual.size()!=masses.size()) throw std::invalid_argument("incomplete anatomical segment mass budget");
    const auto mass_for=[&](std::string_view name) {return std::ranges::find_if(masses,[&](const auto& item){return item.first==name;})->second;};
    const auto add_joint_3d=[&](Element& owner,const std::string& joint_name,const Vec3& axis,bool apply_envelope,const char* default_range_deg=nullptr) {
        Attributes attributes{
            {"name",joint_name},
            {"type","hinge"},
            {"axis",vec17(axis)},
            {"damping","0"},
            {"stiffness","0"},
            {"frictionloss","0"},
            {"armature","0"}};
        if(apply_envelope && envelopes) {
            const auto it=std::ranges::find_if(*envelopes,[&](const auto& item){return item.first==joint_name;});
            if(it!=envelopes->end()) {
                auto env=it->second;
                env.neutral_anatomical_rad=pose_neutral_anatomical_rad(pose,joint_name);
                const auto range=configuration::joint_q_range(env);
                attributes.emplace_back("limited","true");
                attributes.emplace_back("range",g(geometry::detail::np_degrees(range[0]))+" "+g(geometry::detail::np_degrees(range[1])));
            } else if(default_range_deg!=nullptr) {
                attributes.emplace_back("limited","true");
                attributes.emplace_back("range",default_range_deg);
            } else {
                throw std::invalid_argument("missing joint envelope: "+joint_name);
            }
        } else if(default_range_deg!=nullptr) {
            attributes.emplace_back("limited","true");
            attributes.emplace_back("range",default_range_deg);
        }
        owner.add("joint",std::move(attributes));
    };
    const auto split_carrier=[&](double m0,const Vec3& c0,const Mat3& i0,double mc_sum,const Vec3& mc_pos_sum) {
        const double mp=m0-mc_sum;
        if(!(mp>0.)) throw std::invalid_argument("carrier split leaves non-positive parent mass");
        const Vec3 cp=div(sub(mul(c0,m0),mc_pos_sum),mp);
        Mat3 ip=i0;
        const double scale=mp/m0;
        for(double& entry:ip) entry*=scale;
        return std::tuple{mp,cp,ip};
    };

    // 1. Pelvis (direct child of worldbody with freejoint) + 5-DOF saddle carrier.
    auto& pelvis=world.add("body",{{"name","rider_pelvis"},{"pos",vec17(pose.hip)}});
    pelvis.add("freejoint",{{"name","rider_root_free"}});
    const Vec3 pelvis_vec{0.,0.,.12};
    const Vec3 pelvis_c0=div(pelvis_vec,2.);
    const double pelvis_m0=mass_for("pelvis");
    const Mat3 pelvis_i0=segment_inertia(pelvis_m0,pelvis_vec,.085);
    const Vec3 saddle_carrier_pos{0.,0.,-HIP_ABOVE_SADDLE_M};
    const auto [pelvis_mp,pelvis_cp,pelvis_ip]=split_carrier(
        pelvis_m0,pelvis_c0,pelvis_i0,SADDLE_CARRIER_MASS_KG,mul(saddle_carrier_pos,SADDLE_CARRIER_MASS_KG));
    inertial(pelvis,pelvis_mp,pelvis_cp,pelvis_ip);
    geom(pelvis,"geom_rider_pelvis","capsule",{{"fromto","0 0 0 "+vec17(pelvis_vec)},{"size",repr(.085)},
        {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});
    site(pelvis,"site_rider_saddle","0 0 -0.060",".004");

    auto& saddle_carrier=pelvis.add("body",{{"name","rider_saddle_carrier"},{"pos","0 0 -0.060"}});
    add_joint_3d(saddle_carrier,"rider_saddle_pitch",{0.,1.,0.},false);
    constexpr double kSaddleCarrierI=5e-5;
    inertial(saddle_carrier,SADDLE_CARRIER_MASS_KG,{},
             Mat3{kSaddleCarrierI,0.,0.,0.,kSaddleCarrierI,0.,0.,0.,kSaddleCarrierI});

    // 2. Torso + Head (split both shoulder carriers symmetrically from torso).
    auto& torso=pelvis.add("body",{{"name","rider_torso"},{"pos","0 0 0"}});
    add_joint_3d(torso,"rider_torso_hinge",{0.,1.,0.},true);
    const Vec3 torso_vec=sub(pose.shoulder_center,pose.hip);
    const Vec3 torso_c0=div(torso_vec,2.);
    const double torso_m0=mass_for("torso");
    const Mat3 torso_i0=segment_inertia(torso_m0,torso_vec,.11);
    const Vec3 sh_pos_l=sub(pose.shoulder_left,pose.hip);
    const Vec3 sh_pos_r=sub(pose.shoulder_right,pose.hip);
    const auto [torso_mp,torso_cp,torso_ip]=split_carrier(
        torso_m0,torso_c0,torso_i0,2.*SHOULDER_CARRIER_MASS_KG,
        mul(add(sh_pos_l,sh_pos_r),SHOULDER_CARRIER_MASS_KG));
    inertial(torso,torso_mp,torso_cp,torso_ip);
    geom(torso,"geom_rider_torso","capsule",{{"fromto","0 0 0 "+vec17(torso_vec)},{"size",repr(.11)},
        {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});

    auto& head=torso.add("body",{{"name","rider_head"},{"pos",vec17(sub(pose.head_center,pose.hip))}});
    const double head_mass=mass_for("head"),moment=.4*head_mass*.11*.11;
    inertial(head,head_mass,{},Mat3{moment,0.,0.,0.,moment,0.,0.,0.,moment});
    geom(head,"geom_rider_head","sphere",{{"size","0.11"},{"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".6 .6 .6 1"}});

    // 3. Upper limbs: Rz -> Rm -> Rm (+Y left, -Y right).
    constexpr double kSmallCarrierI=2e-5;
    for(int index=0;index<2;++index) {
        const std::string side=index==0?"left":"right";
        const Vec3& shoulder=index==0?pose.shoulder_left:pose.shoulder_right;
        const Vec3& elbow=index==0?pose.elbow_left:pose.elbow_right;
        const Vec3& grip=index==0?pose.grip_left:pose.grip_right;
        const double psi0=index==0?pose.psi0_left_rad:pose.psi0_right_rad;
        const Vec3 axis_m{-std::sin(psi0),std::cos(psi0),0.};

        auto& sh_carrier=torso.add("body",{{"name","rider_shoulder_carrier_"+side},{"pos",vec17(sub(shoulder,pose.hip))}});
        add_joint_3d(sh_carrier,"rider_shoulder_abduction_"+side,{0.,0.,1.},true,"-45 45");
        inertial(sh_carrier,SHOULDER_CARRIER_MASS_KG,{},
                 Mat3{kSmallCarrierI,0.,0.,0.,kSmallCarrierI,0.,0.,0.,kSmallCarrierI});

        auto& upper=sh_carrier.add("body",{{"name","rider_upper_arm_"+side},{"pos","0 0 0"}});
        add_joint_3d(upper,"rider_shoulder_"+side,axis_m,true);
        const Vec3 ua_vec=sub(elbow,shoulder);
        const double ua_mass=mass_for("upper_arm_"+side);
        inertial(upper,ua_mass,div(ua_vec,2.),segment_inertia(ua_mass,ua_vec,.045));
        geom(upper,"geom_rider_upper_arm_"+side,"capsule",{{"fromto","0 0 0 "+vec17(ua_vec)},{"size",repr(.045)},
            {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});

        auto& forearm=upper.add("body",{{"name","rider_forearm_"+side},{"pos",vec17(ua_vec)}});
        add_joint_3d(forearm,"rider_elbow_"+side,axis_m,true);
        const Vec3 fa_vec=sub(grip,elbow);
        const double fa_mass=mass_for("forearm_"+side);
        inertial(forearm,fa_mass,div(fa_vec,2.),segment_inertia(fa_mass,fa_vec,.04));
        geom(forearm,"geom_rider_forearm_"+side,"capsule",{{"fromto","0 0 0 "+vec17(fa_vec)},{"size",repr(.04)},
            {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});
        site(forearm,"site_rider_grip_"+side,vec17(fa_vec),".004");
    }

    // 4. Lower limbs: passive hip abduction Rx + sagittal hip/knee (front=-0.115, rear=+0.115).
    for(int index=0;index<2;++index) {
        const std::string side=index==0?"front":"rear";
        const double y=index==0?-.115:.115;
        const Vec3 hip_s=lateral(pose.hip,y);
        const Vec3 knee_s=lateral(index==0?pose.knee_front:pose.knee_rear,y);
        const Vec3 ankle_s=lateral(index==0?pose.ankle_front:pose.ankle_rear,y);
        const Vec3 pedal_s=lateral(index==0?pose.pedal_front:pose.pedal_rear,y);

        auto& hip_carrier=pelvis.add("body",{{"name","rider_hip_carrier_"+side},{"pos",vec17(sub(hip_s,pose.hip))}});
        add_joint_3d(hip_carrier,"rider_hip_abduction_"+side,{1.,0.,0.},false,"-25 25");
        inertial(hip_carrier,HIP_CARRIER_MASS_KG,{},
                 Mat3{kSmallCarrierI,0.,0.,0.,kSmallCarrierI,0.,0.,0.,kSmallCarrierI});

        auto& thigh=hip_carrier.add("body",{{"name","rider_thigh_"+side},{"pos","0 0 0"}});
        add_joint_3d(thigh,"rider_hip_"+side,{0.,1.,0.},true);
        const Vec3 thigh_vec=sub(knee_s,hip_s);
        const double thigh_m0=mass_for("thigh_"+side);
        constexpr double kThighComFrac=0.4095;
        const Vec3 thigh_c0=mul(thigh_vec,kThighComFrac);
        Mat3 thigh_i0{};
        planar_segment_inertia(thigh_i0,thigh_m0,thigh_vec,{.sag=0.329,.trans=0.329,.lon=0.149});
        const auto [thigh_mp,thigh_cp,thigh_ip]=split_carrier(
            thigh_m0,thigh_c0,thigh_i0,HIP_CARRIER_MASS_KG,Vec3{0.,0.,0.});
        inertial(thigh,thigh_mp,thigh_cp,thigh_ip);
        geom(thigh,"geom_rider_thigh_"+side,"capsule",{{"fromto","0 0 0 "+vec17(thigh_vec)},{"size",repr(.07)},
            {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});

        auto& shank=thigh.add("body",{{"name","rider_shank_"+side},{"pos",vec17(thigh_vec)}});
        add_joint_3d(shank,"rider_knee_"+side,{0.,1.,0.},true);
        const Vec3 shank_vec=sub(ankle_s,knee_s);
        const double shank_mass=mass_for("shank_"+side);
        constexpr double kShankComFrac=0.4459;
        Mat3 shank_i{};
        planar_segment_inertia(shank_i,shank_mass,shank_vec,{.sag=0.251,.trans=0.246,.lon=0.102});
        inertial(shank,shank_mass,mul(shank_vec,kShankComFrac),shank_i);
        geom(shank,"geom_rider_shank_"+side,"capsule",{{"fromto","0 0 0 "+vec17(shank_vec)},{"size",repr(.055)},
            {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});

        auto& foot=shank.add("body",{{"name","rider_foot_"+side},{"pos",vec17(shank_vec)}});
        if(!lock_ankles) add_joint_3d(foot,"rider_ankle_"+side,{0.,1.,0.},true);
        const Vec3 foot_vec=sub(pedal_s,ankle_s);
        const double foot_mass=mass_for("foot_"+side);
        constexpr double kRealFootLengthM=0.2668;
        constexpr double kSagK=0.257,kTransK=0.245,kLongK=0.124;
        const double l_long=kLongK*kRealFootLengthM,l_sag=kSagK*kRealFootLengthM,l_trans=kTransK*kRealFootLengthM;
        const Mat3 foot_i{foot_mass*l_long*l_long,0.,0.,0.,foot_mass*l_sag*l_sag,0.,0.,0.,foot_mass*l_trans*l_trans};
        inertial(foot,foot_mass,div(foot_vec,2.),foot_i);
        geom(foot,"geom_rider_foot_"+side,"capsule",{{"fromto","0 0 0 "+vec17(foot_vec)},{"size",repr(.035)},
            {"mass","0"},{"contype","0"},{"conaffinity","0"},{"rgba",".35 .45 .6 1"}});
        site(foot,"site_rider_sole_"+side,vec17(add(foot_vec,{0.,0.,.008})),".004");
    }
}

void finish_rider_attachments_3d(Element& root,const BuildContext& c,const ArticulatedPose3d& pose) {
    const auto& cfg=*c.physics;
    const double solref=std::max(2.*cfg.timestep_s,cfg.closure_time_constant_s);
    auto& equality=block(root,"equality");
    for(const char* side:{"front","rear"}) {
        const std::string suffix=side;
        if(cfg.articulated.pedal_attachment=="weld")
            equality.add("weld",{{"name","weld_foot_"+suffix},{"body1","rider_foot_"+suffix},{"body2","pedal_"+suffix},{"solref",g(solref)+" 1"}});
        if(cfg.articulated.pedal_attachment=="spindle") {
            const Vec3 anchor=sub(suffix=="front"?pose.pedal_front:pose.pedal_rear,
                                  suffix=="front"?pose.ankle_front:pose.ankle_rear);
            equality.add("connect",{{"name","connect_foot_"+suffix},{"body1","rider_foot_"+suffix},{"body2","pedal_"+suffix},{"anchor",vec17(anchor)},{"solref",g(solref)+" 1"}});
        }
    }
    if(cfg.articulated.saddle_attachment=="weld" || cfg.articulated.saddle_attachment=="pin")
        equality.add("weld",{{"name","weld_saddle"},{"body1","rider_saddle_carrier"},{"body2","frame"},{"solref",g(solref)+" 1"},{"solimp","0.999 0.9999 0.00001 0.5 2"}});
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
