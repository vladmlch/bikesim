#include "benchmark_whipple.hpp"
#include "benchmark_whipple_detail.hpp"
#include "xml_writer.hpp"
#include "../configuration/fields.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

namespace bike::model {
namespace {
std::string real(double x){return configuration::detail::repr_double(x);}
std::string xyz(double x,double y,double z){return real(x)+" "+real(y)+" "+real(z);}
void validate(const WhippleParameters& p) {
    for(const double value:std::array{p.w_m,p.c_m,p.lambda_rad,p.g_mps2,p.v_mps,p.rR_m,p.mR_kg,p.IRxx,p.IRyy,
        p.xB_m,p.zB_m,p.mB_kg,p.IBxx,p.IBxz,p.IByy,p.IBzz,p.xH_m,p.zH_m,p.mH_kg,p.IHxx,p.IHxz,p.IHyy,
        p.IHzz,p.rF_m,p.mF_kg,p.IFxx,p.IFyy})
        if(!std::isfinite(value))throw std::invalid_argument("benchmark_whipple: nonfinite parameter");
    for(const double value:std::array{p.w_m,p.g_mps2,p.rR_m,p.mR_kg,p.IRxx,p.IRyy,p.mB_kg,p.IBxx,p.IByy,p.IBzz,
        p.mH_kg,p.IHxx,p.IHyy,p.IHzz,p.rF_m,p.mF_kg,p.IFxx,p.IFyy})
        if(value<=0.)throw std::invalid_argument("benchmark_whipple: nonpositive geometry, mass or inertia");
    if(p.c_m<0.||std::abs(p.lambda_rad)>=std::numbers::pi/2.||
       p.IBxx*p.IBzz<=p.IBxz*p.IBxz||p.IHxx*p.IHzz<=p.IHxz*p.IHxz)
        throw std::invalid_argument("benchmark_whipple: invalid trail, steer axis or inertia tensor");
}
void inertia(xml::Element& body,double mass,double x,double z,double xx,double yy,double zz,double xz) {
    body.add("inertial",{{"mass",real(mass)},{"pos",xyz(x,0.,z)},
        {"fullinertia",real(xx)+" "+real(yy)+" "+real(zz)+" 0 "+real(xz)+" 0"}});
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the reference API
void wheel(xml::Element& parent,const std::string& name,double x,double z,double radius,double mass,double ix,double iy) {
    auto& w=parent.add("body",{{"name",name+"_wheel"},{"pos",xyz(x,0.,z)}});
    w.add("joint",{{"name",name+"_wheel_spin"},{"type","hinge"},{"axis","0 1 0"}});
    inertia(w,mass,0.,0.,ix,iy,ix,0.);
    w.add("geom",{{"name","geom_"+name+"_contact"},{"type","cylinder"},
        {"size",real(radius)+" 0.002"},{"quat",real(std::sqrt(.5))+" "+real(std::sqrt(.5))+" 0 0"},
        {"rgba","0.15 0.15 0.18 1"}});
}
}
BuildRequest make_whipple_request(const WhippleParameters& p) {
    validate(p);BuildRequest r;r.mode="benchmark_whipple";r.include_rider=false;r.crank_joint=false;
    r.benchmark_whipple=std::make_shared<const WhippleParameters>(p);
    r.spatial.emplace();r.spatial->mode=spatial::Mode::spatial3d;r.spatial->root=spatial::RootKind::free6;
    r.spatial->steer.controller=spatial::SteerController::off;r.spatial->steer.damping_nms_rad=0.;
    return r;
}
namespace detail {
std::string generate_whipple_mjcf(const WhippleParameters& p) {
    validate(p);xml::Element root("mujoco",{{"model","Meijaard_2007_ideal_Whipple"}});
    root.add("compiler",{{"angle","radian"},{"inertiafromgeom","false"}});
    root.add("option",{{"gravity",xyz(0.,0.,-p.g_mps2)},{"timestep","0.0001"},{"integrator","Euler"}});
    auto& defaults=root.add("default");
    defaults.add("joint",{{"damping","0"},{"frictionloss","0"},{"armature","0"},{"limited","false"}});
    defaults.add("geom",{{"contype","0"},{"conaffinity","0"},{"group","1"}});
    auto& world=root.add("worldbody");
    auto& frame=world.add("body",{{"name","frame"},{"pos",xyz(0.,0.,p.rR_m)}});
    frame.add("freejoint",{{"name","root_free"}});
    // The reference z axis points down. Transform position and the off-diagonal
    // inertia Ixz, retaining all four physical rigid-body inertia tensors.
    inertia(frame,p.mB_kg,p.xB_m,-p.zB_m-p.rR_m,p.IBxx,p.IByy,p.IBzz,-p.IBxz);
    frame.add("geom",{{"name","frame_visual"},{"type","capsule"},{"size","0.025"},
        {"fromto",xyz(0.,0.,0.)+" "+xyz(p.xB_m,0.,-p.zB_m-p.rR_m)},{"rgba","0.2 0.4 0.7 1"}});
    frame.add("site",{{"name","imu_site_spatial"},{"pos",xyz(p.xB_m,0.,-p.zB_m-p.rR_m)}});
    wheel(frame,"rear",0.,0.,p.rR_m,p.mR_kg,p.IRxx,p.IRyy);
    // The steering axis intersects the ground c ahead of the front contact.
    // The hinge axis points upwards/backwards; no artificial yaw/roll support.
    auto& head=frame.add("body",{{"name","steer"},{"pos",xyz(p.w_m+p.c_m,0.,-p.rR_m)}});
    head.add("joint",{{"name","steer_joint"},{"type","hinge"},
        {"axis",xyz(-std::sin(p.lambda_rad),0.,std::cos(p.lambda_rad))}});
    inertia(head,p.mH_kg,p.xH_m-p.w_m-p.c_m,-p.zH_m,p.IHxx,p.IHyy,p.IHzz,-p.IHxz);
    head.add("geom",{{"name","front_assembly_visual"},{"type","capsule"},{"size","0.02"},
        {"fromto",xyz(-p.c_m,0.,p.rF_m)+" "+xyz(p.xH_m-p.w_m-p.c_m,0.,-p.zH_m)},
        {"rgba","0.7 0.3 0.2 1"}});
    wheel(head,"front",-p.c_m,p.rF_m,p.rF_m,p.mF_kg,p.IFxx,p.IFyy);
    root.add("actuator").add("motor",{{"name","steer_torque"},{"joint","steer_joint"},{"gear","1"}});
    // Ground contact is supplied exclusively by test-only ideal constraints.
    return xml::document(root);
}
}
} // namespace bike::model
