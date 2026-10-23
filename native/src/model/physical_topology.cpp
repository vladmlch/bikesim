#include "builder_internal.hpp"
#include "../diag.hpp"
#include <stdexcept>

namespace bike::model::detail {
namespace {
void add_tensor(Mat3& target,const Mat3& value) { for(std::size_t i=0;i<target.size();++i) target[i]+=value[i]; }
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
Mat3 diagonal(double x,double y,double z) { return {x,0.,0.,0.,y,0.,0.,0.,z}; }
NATIVE_DIAG_POP
Element& fixed_tendon(Element& root,std::string name,double closure) {
    return block(root,"tendon").add("fixed",{{"name",std::move(name)},{"limited","true"},{"range","-1e12 0"},{"margin","0"},{"solreflimit",g(closure)+" 1"}});
}
void tendon_joint(Element& tendon,std::string name,double coefficient) {
    tendon.add("joint",{{"joint",std::move(name)},{"coef",g(coefficient)}});
}
} // namespace

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void finish_physical_topology(Element& root,const BuildContext& c) {
    if(!c.physical()) throw std::invalid_argument("physical topology cannot modify the legacy model");
    const auto& cfg=*c.physics;const auto& drive=cfg.drive;
    auto& steer=required(root,"body","steer");
    const bool spatial_free=c.spatial() && c.request.spatial->root==bike::spatial::RootKind::free6;
    if(!spatial_free) for(auto* value:steer.direct("joint")) if(value->get("name")=="steer_joint") (void)steer.remove(value);
    if(auto* sensors=root.child("sensor"); sensors && !spatial_free) {
        std::vector<Element*> remove;
        for(const auto& value:sensors->children) if(value->get("joint")=="steer_joint") remove.push_back(value.get());
        for(auto* value:remove) (void)sensors->remove(value);
    }
    if(auto* equality=root.child("equality")) for(auto& constraint:equality->children) constraint->set("solref",g(cfg.closure_time_constant_s)+" 1");
    auto& crank=required(root,"body","crank");
    const double length=c.request.bike.crank_length/1000.,budget=c.request.masses.crank_pedals_mass;
    if(!std::isfinite(length) || length<=0.) throw std::invalid_argument("invalid crank length");
    if(!std::isfinite(budget) || budget<=0.) throw std::invalid_argument("invalid crank mass");
    double spindle=budget*.10/.85,shaft_mass=0.;
    const double arm=budget*.20/.85,pedal_mass=budget*.175/.85;
    if(drive.motor_clutch) {shaft_mass=spindle*.6;spindle-=shaft_mass;}
    auto tensor=ring_inertia(spindle,0.,.016,2.*.075);
    for(double const sign:{-1.,1.}) {
        add_tensor(tensor,segment_inertia(arm,{length,0.,0.},.012));
        add_tensor(tensor,parallel_axis(arm,{sign*length/2.,-sign*.075,0.}));
    }
    set_inertia(crank,spindle+2.*arm,{},tensor);
    required(root,"joint","crank_spin").set("damping","0");
    auto& frame=required(root,"body","frame");
    if(drive.motor_clutch) {
        auto& shaft=frame.add("body",{{"name","drive_shaft"},{"pos","0 0 0"}});
        joint(shaft,"drive_shaft_spin","hinge",{{"axis","0 1 0"},{"damping","0"}});
        const double radius=drive.gearing.front_radius_m();
        geom(shaft,"geom_drive_shaft_ring","cylinder",{{"fromto","0 -0.015 0 0 0.015 0"},{"size",g(radius)},{"mass","0"},{"material","mat_metal"},{"contype","0"},{"conaffinity","0"}});
        set_inertia(shaft,shaft_mass,{},ring_inertia(shaft_mass,.015,radius,.03));
    }
    const double rotor=drive.rotor_inertia_kgm2;
    if(rotor>0.) {
        auto& body_=frame.add("body",{{"name","motor_rotor"},{"pos","0 0 0"}});
        joint(body_,"rotor_spin","hinge",{{"axis","0 1 0"},{"damping","0"}});
        set_inertia(body_,.3,{},diagonal(rotor/2.,rotor,rotor/2.));
    }
    for(int index=0;index<2;++index) {
        const std::string side=index==0?"front":"rear";const double sign=index==0?-1.:1.;
        auto& pedal=required(root,"body","pedal_"+side);
        auto& hinge=required(pedal,"joint","pedal_spin_"+side);
        hinge.set("name","pedal_"+side+"_spin");hinge.set("damping","0");
        const double scale=pedal_mass/3.;
        set_inertia(pedal,pedal_mass,{0.,sign*.04,0.},diagonal(scale*(.04*.04+.008*.008),scale*(.05*.05+.008*.008),scale*(.05*.05+.04*.04)));
    }
    auto& wheel=required(root,"body","rear_wheel");
    if(drive.transmission_model=="elastic_chain") {
        Element* parent=nullptr;
        for(auto* candidate:root.descendants("body")) for(const auto& value:candidate->children) if(value.get()==&wheel) parent=candidate;
        if(!parent) throw std::invalid_argument("rear wheel has no body parent");
        auto& cassette=parent->add("body",{{"name","cassette"},{"pos",wheel.get("pos","0 0 0")}});
        cassette.add("joint",{{"name","cassette_spin"},{"type","hinge"},{"axis","0 1 0"},{"damping","0"}});
        auto& cassette_geom=required(wheel,"geom","geom_rear_cassette");
        cassette.children.push_back(wheel.remove(&cassette_geom));
        auto profile=kRearWheelProfile;profile[0].mass_fraction=.75/.9;profile[1].mass_fraction=.15/.9;
        const double wheel_mass=c.request.masses.rear_wheel_mass*.9;
        const auto wi=wheel_body_inertia(wheel_mass,profile);set_inertia(wheel,wheel_mass,wi.com,wi.tensor);
        const double mass=c.request.masses.rear_wheel_mass*.1;const auto& core=kRearWheelProfile[1];
        set_inertia(cassette,mass,core.offset_m,ring_inertia(mass,core.inner_radius_m,core.outer_radius_m,core.width_m));
    } else {
        const auto wi=wheel_body_inertia(c.request.masses.rear_wheel_mass,kRearWheelProfile);
        set_inertia(wheel,c.request.masses.rear_wheel_mass,wi.com,wi.tensor);
    }
    for(const char* side:{"front","rear"}) {
        const std::string prefix=side;
        auto& owner=required(root,"body",prefix+"_wheel");
        auto& spin=required(owner,"joint",prefix+"_wheel_spin");
        spin.set("damping","0");spin.set("frictionloss","0");spin.set("solreffriction","0.005 1");spin.set("solimpfriction","0.9999 0.9999 0.001 0.5 2");
        auto& contact=required(owner,"geom","geom_"+prefix+"_contact");
        contact.set("priority","1");
        contact.set("friction",g(prefix=="front"?cfg.tires.front.mu:cfg.tires.rear.mu)+" 0.005 0.0001");
        if(cfg.tires.backend=="compliant_2d" || cfg.tires.backend=="distributed_2d_reference" || cfg.tires.backend=="pneumatic")
            for(auto* value:owner.descendants("geom")) { value->set("contype","0");value->set("conaffinity","0"); }
    }
    for(const char* name:{"frame","steer","fork_lower","chainstay","seatstay","rocker","shock_yoke","shock_shaft","shock_body"})
        for(auto* value:required(root,"body",name).direct("geom")) {
            if(value->get("name")=="geom_handlebar" || value->get("contype","1")=="0") continue;
            value->set("contype","8");value->set("conaffinity","0");
        }
    if(auto* catch_=root.find("geom","catch_plane")) catch_->set("conaffinity",std::to_string((static_cast<unsigned>(std::stoi(catch_->get("conaffinity","1")))|8U)));
    auto& contact=block(root,"contact");
    std::set<std::pair<std::string,std::string>> excluded;
    const auto pair=[](std::string a,std::string b){ if(b<a) std::swap(a,b);return std::make_pair(std::move(a),std::move(b)); };
    for(auto const* item:contact.direct("exclude")) excluded.insert(pair(item->get("body1"),item->get("body2")));
    std::vector<std::string> names;
    for(auto const* item:root.descendants("body")) if(!item->get("name").empty()) names.push_back(item->get("name"));
    std::vector<std::string> moving{"crank","pedal_front","pedal_rear"};
    if(drive.transmission_model=="elastic_chain") moving.emplace_back("cassette");
    if(drive.motor_clutch) moving.emplace_back("drive_shaft");
    for(const auto& source:moving) for(const auto& target:names) {
        if(source==target || target.starts_with("rider_") || !excluded.insert(pair(source,target)).second) continue;
        contact.add("exclude",{{"body1",source},{"body2",target}});
    }
    auto& actuators=block(root,"actuator");
    std::vector<Element*> remove;
    for(const auto& value:actuators.children) if(value->get("name")=="crank_drive" || (value->get("name")=="rear_drive" && cfg.drive_mode!="ideal_speed_control")) remove.push_back(value.get());
    for(auto* value:remove) (void)actuators.remove(value);
    if(cfg.drive_mode=="crank_effort" || cfg.drive_mode=="articulated_effort") {
        const std::string driver=drive.motor_clutch?"drive_shaft_spin":"crank_spin";
        if(drive.transmission_model=="ideal_mid_drive") {
            auto& tendon=fixed_tendon(root,"ideal_mid_drive_freehub",cfg.closure_time_constant_s);
            tendon_joint(tendon,driver,static_cast<double>(drive.gearing.front_teeth)/static_cast<double>(drive.gearing.rear_teeth));
            tendon_joint(tendon,"rear_wheel_spin",-1.);
        }
        if(drive.motor_clutch) {
            auto& tendon=fixed_tendon(root,"crank_clutch",cfg.closure_time_constant_s);
            tendon_joint(tendon,"crank_spin",1.);tendon_joint(tendon,"drive_shaft_spin",-1.);
        }
        if(rotor>0.) {
            auto& tendon=fixed_tendon(root,"motor_freewheel",cfg.closure_time_constant_s);
            tendon_joint(tendon,"rotor_spin",1.);tendon_joint(tendon,"crank_spin",-1.);
        }
        if(drive.transmission_model=="geometric_ideal_mid_drive") finalize_geometric_transmission(root,cfg);
        actuators.add("motor",{{"name","mid_drive"},{"joint",rotor>0.?"rotor_spin":driver},{"gear","1"},{"ctrllimited","true"},{"ctrlrange","0 "+g(drive.assist.effective_max_torque())}});
    }
    if(cfg.drive_mode=="crank_effort") actuators.add("motor",{{"name","human_crank"},{"joint","crank_spin"},{"gear","1"}});
}
NATIVE_DIAG_POP
void finalize_geometric_transmission(Element& root,const configuration::SimulationPhysicsConfig& cfg) {
    if(cfg.drive.transmission_model!="geometric_ideal_mid_drive" || (cfg.drive_mode!="crank_effort" && cfg.drive_mode!="articulated_effort")) return;
    auto& tendons=block(root,"tendon");
    auto* freehub=tendons.find("fixed","geometric_mid_drive_freehub");
    if(!freehub) freehub=&fixed_tendon(root,"geometric_mid_drive_freehub",cfg.closure_time_constant_s);
    freehub->children.clear();
    const auto driver=cfg.drive.motor_clutch?"drive_shaft_spin":"crank_spin";
    for(auto const* value:block(root,"worldbody").descendants("joint")) {
        const auto kind=value->get("type","hinge"),name=value->get("name");
        if(kind!="slide" && kind!="hinge") {
            if(cfg.spatial && cfg.spatial->mode==bike::spatial::Mode::spatial3d) continue;
            throw std::invalid_argument("geometric ideal drive requires scalar planar joints");
        }
        if(name.empty()) throw std::invalid_argument("geometric tendon requires named coordinates");
        tendon_joint(*freehub,name,name==driver?cfg.drive.gearing.front_radius_m():name=="rear_wheel_spin"?-cfg.drive.gearing.rear_radius_m():0.);
    }
}
} // namespace bike::model::detail
