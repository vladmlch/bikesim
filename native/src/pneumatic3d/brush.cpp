#include "pneumatic3d/brush.hpp"
#include "pneumatic3d/brush_detail.hpp"
#include "pneumatic3d/kernel_math.hpp"
#include "spatial/frames.hpp"

#include <algorithm>
#include <cmath>

namespace bike::pneumatic3d {
namespace detail {
namespace {
FastBrushResult compute(const BrushSpec3d& spec, const Patch3d& patch,
    const Vec3& point_velocity, const Vec3& hub_velocity, const Vec3& axle,
    double spin, double camber, double mux, double muy, double oldx, double oldy,
    double dt, const Vec3& angular_velocity, bool material_resolved) noexcept {
    FastBrushResult result;
    if (!std::isfinite(dt) || dt<0. || !finite(point_velocity) || !finite(hub_velocity) ||
        !finite(axle) || !finite(angular_velocity) || !std::isfinite(spin) ||
        !std::isfinite(camber) || !std::isfinite(oldx) || !std::isfinite(oldy) ||
        !std::isfinite(mux) || mux<0. || !std::isfinite(muy) || muy<0. ||
        !std::isfinite(spec.kx_n_per_m3) || spec.kx_n_per_m3<0. ||
        !std::isfinite(spec.ky_n_per_m3) || spec.ky_n_per_m3<0. ||
        !std::isfinite(spec.relaxation_x_m) || spec.relaxation_x_m<=0. ||
        !std::isfinite(spec.relaxation_y_m) || spec.relaxation_y_m<=0. ||
        !std::isfinite(spec.r_eff_m) || spec.r_eff_m<=0. ||
        !std::isfinite(spec.camber_coefficient_per_rad) ||
        !std::isfinite(spec.pneumatic_trail_m) || spec.pneumatic_trail_m<0. ||
        !std::isfinite(patch.normal_load_n) || patch.normal_load_n<0. ||
        !std::isfinite(patch.area_m2) || patch.area_m2<0.) {
        result.status=CoreStatus::invalid_input; return result;
    }
    auto& w=result.wrench;
    w.point_world_m=patch.centroid_world_m; w.surface_id=patch.surface_id;
    w.patch_id=patch.id; w.camber_rad=camber;
    if (!patch.valid || !(patch.normal_load_n>0.) || !(patch.area_m2>0.)) return result;
    const double kx=spec.kx_n_per_m3*patch.area_m2, ky=spec.ky_n_per_m3*patch.area_m2;
    const double hub_longitudinal=dot(hub_velocity,patch.longitudinal_world);
    const double rolling=std::abs(hub_longitudinal);
    const double direction=hub_longitudinal<0. ? -1. : (hub_longitudinal>0. ? 1. : 0.);
    const Vec3 offset=scale(patch.longitudinal_world,material_resolved ? 0. : spec.pneumatic_trail_m*direction);
    w.point_world_m=add(patch.centroid_world_m,offset);
    const Vec3 velocity=add(point_velocity,cross(angular_velocity,offset));
    const double vx=dot(velocity,patch.longitudinal_world), vy=dot(velocity,patch.lateral_world);
    const double b=ky>0. ? spec.r_eff_m*spec.camber_coefficient_per_rad*std::sin(camber)/
                          (ky*spec.relaxation_y_m) : 0.;
    const double sx=vx, sy=vy+b*spin;
    const double ax=material_resolved ? 0. : rolling/spec.relaxation_x_m;
    const double ay=material_resolved ? 0. : rolling/spec.relaxation_y_m;
    // u is spring extension, so du/dt=v_rel-a*u and F=-K*u. Reversing
    // only the first sign (as the W prose does) would be an active tyre.
    const double tx=(oldx+dt*sx)/(1.+dt*ax), ty=(oldy+dt*sy)/(1.+dt*ay);
    double x=tx,y=ty;
    const double capx=mux*patch.normal_load_n, capy=muy*patch.normal_load_n;
    if (capx==0. || kx==0.) x=0.;
    if (capy==0. || ky==0.) y=0.;
    const double nx=capx>0. ? kx*x/capx : 0., ny=capy>0. ? ky*y/capy : 0.;
    const double ellipse=std::hypot(nx,ny);
    if (ellipse>1.) { x/=ellipse; y/=ellipse; }
    const double fx=-kx*x, fy=-ky*y;
    result.ux_m=x; result.uy_m=y;
    result.stored_j=.5*(kx*x*x+ky*y*y);
    // Endpoint work identity: -F.v*dt = Delta(U) + losses. This includes
    // the implicit integrator's nonnegative algorithmic loss explicitly.
    result.sliding_j=std::max(0.,kx*x*(tx-x)+ky*y*(ty-y));
    result.convective_j=std::max(0.,dt*(ax*kx*x*tx+ay*ky*y*ty)+
        .5*(kx*(x-oldx)*(x-oldx)+ky*(y-oldy)*(y-oldy)));
    const Vec3 tangent=add(scale(patch.longitudinal_world,fx),scale(patch.lateral_world,fy));
    w.force_world_n=add(patch.force_world_n,tangent);
    // Keep the distributed normal wrench at its original centroid even
    // though the tangential force is shifted by pneumatic trail.
    w.couple_world_nm=add(sub(patch.couple_world_nm,cross(offset,patch.force_world_n)),scale(axle,b*fy));
    w.normal_load_n=patch.normal_load_n; w.fx_n=fx; w.fy_n=fy;
    w.mz_nm=dot(cross(offset,tangent),patch.normal_world);
    w.slip_ratio=vx/std::max(rolling,1e-6);
    w.slip_angle_rad=std::atan2(vy,std::max(rolling,1e-6));
    w.loss_j=result.sliding_j+result.convective_j; w.valid=true;
    if (!finite(w.force_world_n) || !finite(w.couple_world_nm) ||
        !std::isfinite(w.loss_j) || !std::isfinite(result.stored_j))
        result.status=CoreStatus::invalid_geometry;
    return result;
}
}

FastBrushResult evaluate_fast_patch(const BrushSpec3d& spec, const Patch3d& patch,
    const Vec3& point_velocity, const Vec3& hub_velocity, const Vec3& axle,
    double spin,double camber,double mux,double muy,double oldx,double oldy,
    double dt,const Vec3& angular_velocity) noexcept {
    return compute(spec,patch,point_velocity,hub_velocity,axle,spin,camber,
                   mux,muy,oldx,oldy,dt,angular_velocity,false);
}

FastBrushResult evaluate_bristle(const BrushSpec3d& spec,const ContactElement3d& e,
    const NormalElement3d& n,const WheelKinematics3d& kin,double mux,double muy,
    double oldx,double oldy,double dt) noexcept {
    Patch3d patch;
    patch.id=static_cast<std::uint64_t>(e.material_cell)+1;
    patch.centroid_world_m=e.point_world_m; patch.normal_world=e.normal_world;
    patch.longitudinal_world=e.longitudinal_world; patch.lateral_world=e.lateral_world;
    patch.normal_load_n=n.normal_load_n; patch.area_m2=e.geometric_area_m2;
    patch.surface_id=e.surface_id; patch.valid=e.valid&&n.active;
    patch.force_world_n=scale(e.normal_world,n.normal_load_n);
    const Vec3 angular=add(kin.hub.angular_velocity_world_rad_s,scale(kin.axle_world,kin.relative_spin_rad_s));
    const Vec3 velocity=add(kin.hub.linear_velocity_world_mps,cross(angular,sub(e.point_world_m,kin.hub.position_world_m)));
    const double gamma=std::asin(std::clamp(dot(kin.axle_world,e.normal_world),-1.,1.));
    return compute(spec,patch,velocity,kin.hub.linear_velocity_world_mps,kin.axle_world,
                   kin.relative_spin_rad_s,gamma,mux,muy,oldx,oldy,dt,angular,true);
}
} // namespace detail

CoreStatus update_fast_brush(const BrushSpec3d& spec,std::span<const Patch3d> patches,
    std::span<const bike::spatial::Vec3> velocity,double dt,bool advance,
    BrushState3d& state,std::span<PatchWrench> out) noexcept {
    using namespace detail;
    const auto n=patches.size();
    if (out.size()!=n || velocity.size()!=n+1 || state.ux_m.size()<n ||
        state.uy_m.size()!=state.ux_m.size() || state.camber_state.size()!=state.ux_m.size())
        return CoreStatus::invalid_input;
    // Frozen standalone interface represents a unit-friction synthetic road.
    // Production calls the private per-patch seam with resolved road mu.
    for (std::size_t i=0;i<n;++i) {
        const auto& p=patches[i];
        const double gamma=state.camber_state[i];
        const Vec3 axle=add(scale(p.lateral_world,std::cos(gamma)),scale(p.normal_world,std::sin(gamma)));
        const double spin=spec.r_eff_m>0. ? dot(sub(velocity[n],velocity[i]),p.longitudinal_world)/spec.r_eff_m : 0.;
        const auto result=evaluate_fast_patch(spec,p,velocity[i],velocity[n],axle,spin,gamma,
            spec.mu_x_scale,spec.mu_y_scale,state.ux_m[i],state.uy_m[i],dt);
        if (result.status!=CoreStatus::ok) return result.status;
        out[i]=result.wrench;
    }
    if (advance && dt>0.) {
        double stored=0.,sliding=0.,convective=0.;
        for (std::size_t i=0;i<n;++i) {
            const auto& p=patches[i]; const double gamma=state.camber_state[i];
            const Vec3 axle=add(scale(p.lateral_world,std::cos(gamma)),scale(p.normal_world,std::sin(gamma)));
            const double spin=dot(sub(velocity[n],velocity[i]),p.longitudinal_world)/spec.r_eff_m;
            const auto result=evaluate_fast_patch(spec,p,velocity[i],velocity[n],axle,spin,gamma,
                spec.mu_x_scale,spec.mu_y_scale,state.ux_m[i],state.uy_m[i],dt);
            state.ux_m[i]=result.ux_m; state.uy_m[i]=result.uy_m;
            stored+=result.stored_j; sliding+=result.sliding_j; convective+=result.convective_j;
        }
        state.elastic_energy_j=stored; state.sliding_loss_j+=sliding;
        state.convective_release_j+=convective;
    }
    return CoreStatus::ok;
}
} // namespace bike::pneumatic3d
