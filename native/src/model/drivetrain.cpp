#include "builder_internal.hpp"
#include <stdexcept>

namespace bike::model::detail {
// mujoco/drivetrain.py: geometry and authored relative mass weights are kept
// separate from the physical inertials/budgets applied after tree assembly.
void build_bb_and_motor(Element& frame,BuildContext& c) {
    const auto& masses=c.request.masses;
    const double length=c.request.bike.crank_length/1000.;
    geom(frame,"geom_bb_shell","cylinder",{{"fromto","0 -0.0365 0 0 0.0365 0"},{"size","0.024"},{"mass","0.30"},{"material","mat_metal"}});
    geom(frame,"geom_motor_core","cylinder",{{"fromto","0.015 -0.055 0.020 0.045 0.055 0.035"},{"size","0.045"},{"mass",c.physical()?"2.90":f(masses.motor_mass,2)},{"material","mat_motor"}});
    double spindle_mass=masses.crank_pedals_mass-2.*.20-2.*.175;
    if(c.physical()) spindle_mass=.10;
    if(spindle_mass<=0.) throw std::invalid_argument("crank_pedals_mass "+f(masses.crank_pedals_mass,3)+" kg does not cover two 0.200 kg arms and two 0.175 kg pedals");
    auto& crank=c.crank_joint? frame.add("body",{{"name","crank"},{"pos","0 0 0"}}):frame;
    if(c.crank_joint) joint(crank,"crank_spin","hinge",{{"axis","0 1 0"},{"damping","0.030"}});
    geom(crank,"geom_crank_spindle","cylinder",{{"fromto","0 -0.075000 0 0 0.075000 0"},{"size","0.016"},{"mass",f(spindle_mass,3)},{"material","mat_metal"},{"contype","0"},{"conaffinity","0"}});
    const auto pedals=geometry::pedal_points(length);
    for(int index=0;index<2;++index) {
        const std::string side=index==0?"front":"rear";
        const auto& pedal=index==0?pedals.first:pedals.second;
        const double sign=index==0?-1.:1.;
        const double ya=sign*geometry::CRANK_ARM_LATERAL_OFFSET_M,yp=sign*geometry::PEDAL_LATERAL_OFFSET_M;
        geom(crank,"geom_crank_arm_"+side,"capsule",{{"fromto","0 "+f(ya)+" 0 "+f(pedal[0])+" "+f(ya)+" "+f(pedal[2])},
            {"size","0.012"},{"mass","0.200"},{"material","mat_metal"},{"contype","0"},{"conaffinity","0"}});
        if(c.crank_joint) {
            auto& platform=body(crank,"pedal_"+side,{pedal[0],ya,pedal[2]});
            joint(platform,"pedal_spin_"+side,"hinge",{{"axis","0 1 0"},{"damping","0.005"}});
            const auto pos="0 "+f(yp-ya)+" 0";
            geom(platform,"geom_pedal_"+side,"box",{{"pos",pos},{"size","0.050 0.040 0.008"},{"mass","0.175"},{"material","mat_metal"},{"contype","0"},{"conaffinity","0"}});
            site(platform,"site_pedal_"+side,pos,"0.006");
        } else {
            geom(crank,"geom_pedal_"+side,"box",{{"pos",vec({pedal[0],yp,pedal[2]})},{"size","0.050 0.040 0.008"},{"mass","0.175"},{"material","mat_metal"},{"contype","0"},{"conaffinity","0"}});
        }
    }
    if(c.physical()) {
        std::vector<Element*> motor,crank_parts;
        for(auto* value:frame.descendants("geom")) {
            const auto name=value->get("name");
            if(name=="geom_motor_core") motor.push_back(value);
            if(name.starts_with("geom_crank_") || name.starts_with("geom_pedal_")) crank_parts.push_back(value);
        }
        register_component(c.mass_registry(),"motor",std::move(motor));
        register_component(c.mass_registry(),"crank_pedals",std::move(crank_parts));
    }
}
void build_chain_constraint(Element& root,double ratio) {
    if(!std::isfinite(ratio) || ratio<=0.) throw std::invalid_argument("gear_ratio must be positive, got "+repr(ratio));
    block(root,"equality").add("joint",{{"name","chain_drive"},{"joint1","crank_spin"},{"joint2","rear_wheel_spin"},
        {"polycoef","0 "+f(1./ratio,9)+" 0 0 0"},{"solref","0.002 1"}});
}
Element& build_rear_wheel(Element& parent,const Vec3& axle,BuildContext& c) {
    auto& wheel=body(parent,"rear_wheel",axle);
    joint(wheel,"rear_wheel_spin","hinge",{{"axis","0 1 0"},{"damping","0.01"}});
    geom(wheel,"geom_rear_hub","cylinder",{{"fromto","0 -0.074 0 0 0.074 0"},{"size","0.020"},{"mass",c.physical()?"0":"0.30"},{"material","mat_metal"}});
    geom(wheel,"geom_rear_rim","cylinder",{{"fromto","0 -0.015 0 0 0.015 0"},{"size","0.300"},{"mass",c.physical()?"0":"0.40"},{"material","mat_rim"},{"contype","0"},{"conaffinity","0"}});
    const auto radius=f(c.request.bike.rear_wheel_radius/1000.);
    const auto mask=c.request.mode=="ride"?"0":"1";
    geom(wheel,"geom_rear_tire","cylinder",{{"fromto","0 -0.032 0 0 0.032 0"},{"size",radius},{"mass",c.physical()?"0":"1.50"},{"material","mat_tire"},{"contype",mask},{"conaffinity",mask},{"friction","1.2 0.005 0.0001"}});
    if(c.request.mode=="ride") {
        const auto contact=c.request.tyre_model=="pneumatic"?"0":"1";
        geom(wheel,"geom_rear_contact","sphere",{{"pos","0 0 0"},{"size",radius},{"mass","0"},{"condim","3"},{"friction","1.2 0.005 0.0001"},{"solref","-130000 -800"},{"contype",contact},{"conaffinity",contact},{"rgba","0.08 0.08 0.08 0"}});
    }
    geom(wheel,"geom_rear_cassette","cylinder",{{"fromto","0 -0.055 0 0 -0.035 0"},{"size","0.095"},{"mass",c.physical()?"0":"0.40"},{"material","mat_metal"},{"contype","0"},{"conaffinity","0"}});
    geom(wheel,"geom_rear_rotor","cylinder",{{"fromto","0 0.038 0 0 0.040 0"},{"size","0.1015"},{"mass",c.physical()?"0":"0.20"},{"material","mat_metal"},{"contype","0"},{"conaffinity","0"}});
    if(c.physical()) {
        const auto value=wheel_body_inertia(c.request.masses.rear_wheel_mass,kRearWheelProfile);
        inertial(wheel,c.request.masses.rear_wheel_mass,value.com,value.tensor);
        register_component(c.mass_registry(),"rear_wheel",wheel.direct("inertial"));
    }
    return wheel;
}
// actuators.py. Keep declaration order: address mapping and control schemas
// consume it even where physical topology later removes legacy actuators.
void build_actuators(Element& root,const BuildContext& c) {
    auto& actuator=root.add("actuator");const auto& mode=c.request.mode;
    const auto motor=[&](std::string name,std::string joint_name,std::string range) {
        actuator.add("motor",{{"name",std::move(name)},{"joint",std::move(joint_name)},{"gear","1"},{"ctrlrange",std::move(range)}});
    };
    if(mode=="stand" || mode=="playground") {
        actuator.add("position",{{"name","rear_travel_actuator"},{"joint","main_pivot"},{"kp","150000"},{"kv","5000"},{"ctrlrange","-0.60 0.60"}});
        actuator.add("position",{{"name","fork_travel_actuator"},{"joint","fork_travel"},{"kp","150000"},{"kv","5000"},{"ctrlrange","0 0.30"}});
        motor("front_wheel_spin","front_wheel_spin","-50 50");motor("rear_wheel_spin","rear_wheel_spin","-50 50");
    } else if(mode=="ride") {
        if(c.crank_joint) motor("crank_drive","crank_spin","-180 180");
        motor("rear_drive","rear_wheel_spin","-150 150");
        motor("front_brake","front_wheel_spin","-200 200");motor("rear_brake","rear_wheel_spin","-200 200");
    } else {
        actuator.add("position",{{"name","steer_servo"},{"joint","steer_joint"},{"kp","150"},{"kv","15"},{"ctrlrange","-45 45"}});
        motor("rear_drive_motor","rear_wheel_spin","-150 150");
        motor("front_brake","front_wheel_spin","-200 0");motor("rear_brake","rear_wheel_spin","-200 0");
    }
}
} // namespace bike::model::detail
