#include "support/migration_test.hpp"
#include "support/tyre_fixture.hpp"
#include "support/tyre_test_helpers.hpp"
#include "pneumatic3d/brush_detail.hpp"

#include <array>
#include <cmath>
#include <limits>

int main() { return bt::run([] {
    using namespace bike::pneumatic3d;
    using namespace bike::pneumatic3d::detail;
    using namespace spatial_tests::tyre;
    auto spec=spatial_tests::synthetic_spec(bike::spatial::TyreTier::fast).brush[0];
    spec.pneumatic_trail_m=0.;spec.camber_coefficient_per_rad=0.;
    spec.mu_x_scale=.8;spec.mu_y_scale=.6;
    std::array<Patch3d,1> patches{patch()};
    auto s=brush_state(1);
    std::array<bike::spatial::Vec3,2> v{{{.3,.2,0.},{3.,0.,0.}}};
    std::array<PatchWrench,1> out;
    const double dt=.001;
    for(unsigned tick=0;tick<1200;++tick) {
        if(tick==300) { v[0]={0.,0.,0.};v[1]={0.,0.,0.}; }
        if(tick==500) { v[0]={-.2,-.4,0.};v[1]={-2.,0.,0.}; }
        if(tick==900) { v[0]={.1,-.2,0.};v[1]={1.,0.,0.}; }
        const double old=s.elastic_energy_j;
        bt::require(update_fast_brush(spec,patches,v,dt,true,s,out)==CoreStatus::ok,"stop/start/reverse brush interval");
        const auto& w=out[0];
        const double ellipse=std::pow(w.fx_n/(spec.mu_x_scale*w.normal_load_n),2)+
                             std::pow(w.fy_n/(spec.mu_y_scale*w.normal_load_n),2);
        bt::require(ellipse<=1.+1e-12,"combined force respects the shared friction ellipse");
        bt::require(w.loss_j>=-1e-12,"plastic and convective losses nonnegative");
        const double work=-(w.fx_n*v[0][0]+w.fy_n*v[0][1])*dt;
        bt::require(std::abs((s.elastic_energy_j-old)+w.loss_j-work)<=1e-6*std::max(1.,std::abs(work))+1e-9,"endpoint force work closes stored energy plus losses");
    }
    const auto before=bytes(s);
    bt::require(update_fast_brush(spec,patches,v,dt,false,s,out)==CoreStatus::ok && bytes(s)==before,"brush probe byte preservation");
    auto mirrored=s;mirrored.uy_m[0]=-mirrored.uy_m[0];
    const auto original=evaluate_fast_patch(spec,patches[0],{.1,.2,0.},{3.,0.,0.},{0.,1.,0.},0.,0.,.8,.6,s.ux_m[0],s.uy_m[0],dt);
    const auto mirror=evaluate_fast_patch(spec,patches[0],{.1,-.2,0.},{3.,0.,0.},{0.,1.,0.},0.,0.,.8,.6,mirrored.ux_m[0],mirrored.uy_m[0],dt);
    bt::require(std::abs(original.wrench.fx_n-mirror.wrench.fx_n)<=1e-10 && std::abs(original.wrench.fy_n+mirror.wrench.fy_n)<=1e-10,"mirror longitudinal even and lateral odd");
    spec.mu_x_scale=spec.mu_y_scale=0.;
    bt::require(update_fast_brush(spec,patches,v,dt,true,s,out)==CoreStatus::ok && out[0].fx_n==0. && out[0].fy_n==0.,"zero grip gives zero tangential load");
    patches[0].normal_load_n=0.;
    bt::require(update_fast_brush(spec,patches,v,dt,true,s,out)==CoreStatus::ok && out[0].force_world_n==bike::spatial::Vec3{},"unloaded contact has no force");
    spec=spatial_tests::synthetic_spec(bike::spatial::TyreTier::fast).brush[0];
    const auto p=patch();
    const auto cambered=evaluate_fast_patch(spec,p,{.2,.1,0.},{3.,0.,0.},{0.,std::cos(.2),std::sin(.2)},8.,.2,.8,.8,0.,0.,dt,{0.,8.,0.});
    bt::require(cambered.status==CoreStatus::ok && std::isfinite(cambered.wrench.mz_nm),"camber plus trail has finite conjugate moment");
    const auto tangential=add(scale(p.longitudinal_world,cambered.wrench.fx_n),scale(p.lateral_world,cambered.wrench.fy_n));
    bt::require(std::abs(cambered.wrench.mz_nm-dot(cross(sub(cambered.wrench.point_world_m,p.centroid_world_m),tangential),p.normal_world))<=1e-12,"trail torque counted once at its shifted force point");

    // The normalized camber coefficient must multiply load. Splitting the
    // same area and load changes quadrature, not the physical tyre response.
    spec.pneumatic_trail_m=0.;
    const double gamma=.2,spin=8.;
    const bike::spatial::Vec3 axle{0.,std::cos(gamma),std::sin(gamma)};
    const auto whole=evaluate_fast_patch(spec,p,{0.,0.,0.},{3.,0.,0.},axle,spin,gamma,10.,10.,0.,0.,dt);
    auto half=p;half.area_m2*=.5;half.normal_load_n*=.5;
    half.force_world_n=scale(half.force_world_n,.5);
    const auto divided=evaluate_fast_patch(spec,half,{0.,0.,0.},{3.,0.,0.},axle,spin,gamma,10.,10.,0.,0.,dt);
    const double expected=-dt*spec.r_eff_m*spec.camber_coefficient_per_rad*p.normal_load_n*
        std::sin(gamma)*spin/(spec.relaxation_y_m*(1.+dt*3./spec.relaxation_y_m));
    bt::require(whole.status==CoreStatus::ok && divided.status==CoreStatus::ok,"camber quadrature controls accepted");
    bt::require(std::abs(whole.wrench.fy_n-expected)<=1e-12,"camber source has load-normalized stiffness and velocity units");
    bt::require(std::abs(whole.wrench.fy_n-2.*divided.wrench.fy_n)<=1e-12 &&
                norm(sub(whole.wrench.couple_world_nm,scale(divided.wrench.couple_world_nm,2.)))<=1e-12 &&
                std::abs(whole.stored_j-2.*divided.stored_j)<=1e-12,"camber force, work-conjugate couple and storage are invariant under area/load partition");
    auto contact=element(0,.01,p.area_m2);
    NormalElement3d load;load.normal_load_n=p.normal_load_n;load.active=true;
    const auto kin=spatial_tests::wheel_at({0.,0.,spec.r_eff_m},0.,gamma,3.,spin);
    const auto full_bristle=evaluate_bristle(spec,contact,load,kin,10.,10.,0.,0.,dt);
    contact.geometric_area_m2*=.5;load.normal_load_n*=.5;
    const auto half_bristle=evaluate_bristle(spec,contact,load,kin,10.,10.,0.,0.,dt);
    bt::require(full_bristle.status==CoreStatus::ok && half_bristle.status==CoreStatus::ok &&
                std::abs(full_bristle.wrench.fy_n-2.*half_bristle.wrench.fy_n)<=1e-12 &&
                std::abs(full_bristle.stored_j-2.*half_bristle.stored_j)<=1e-12,
                "material-resolved camber uses the same subdivision-invariant load normalization");
    auto carrier=kin;carrier.relative_spin_rad_s=0.;
    carrier.hub.angular_velocity_world_rad_s={0.,2.,0.};
    carrier.hub.linear_velocity_world_mps={2.*spec.r_eff_m,0.,0.};
    const auto rotating=evaluate_bristle(spec,contact,load,carrier,10.,10.,0.,0.,dt);
    const double supplied_work=-dot(rotating.wrench.couple_world_nm,
        carrier.hub.angular_velocity_world_rad_s)*dt;
    bt::require(rotating.status==CoreStatus::ok && std::abs(rotating.wrench.fy_n)>1e-8 &&
                std::abs(rotating.stored_j+rotating.wrench.loss_j-supplied_work)<=1e-12,
                "camber storage uses absolute wheel spin conjugate to the wheel-body couple, including carrier rotation");

    auto finite_state=brush_state(1);
    const auto finite_before=bytes(finite_state);
    const std::array<Patch3d,1> loaded_patch{p};
    const std::array<bike::spatial::Vec3,2> huge_velocity{{{1e308,0.,0.},{1e308,0.,0.}}};
    bt::require(update_fast_brush(spec,loaded_patch,huge_velocity,1.,true,finite_state,out)==CoreStatus::invalid_geometry &&
                bytes(finite_state)==finite_before,
                "finite velocity cannot hide an overflowing relaxation denominator as zero force and zero loss");
    const std::array<bike::spatial::Vec3,2> control_velocity{{{.1,0.,0.},{3.,0.,0.}}};
    bt::require(update_fast_brush(spec,loaded_patch,control_velocity,dt,true,finite_state,out)==CoreStatus::ok && out[0].fx_n<0.,
                "valid retry after rejected brush arithmetic retains its friction response");
    auto invalid_tail=brush_state(2);
    invalid_tail.ux_m[1]=std::numeric_limits<double>::quiet_NaN();
    const auto tail_before=bytes(invalid_tail);
    bt::require(update_fast_brush(spec,loaded_patch,control_velocity,dt,true,invalid_tail,out)==CoreStatus::invalid_input &&
                bytes(invalid_tail)==tail_before,"unused capacity cannot hide a nonfinite material-state lane");
    invalid_tail.ux_m[1]=.001;
    invalid_tail.elastic_energy_j=.5*spec.kx_n_per_m3*p.area_m2*.001*.001;
    const auto omitted_before=bytes(invalid_tail);
    bt::require(update_fast_brush(spec,loaded_patch,control_velocity,dt,true,invalid_tail,out)==CoreStatus::invalid_input &&
                bytes(invalid_tail)==omitted_before,
                "omitting a finite loaded spring cannot discard its stored energy while retaining its displacement");
    invalid_tail.ux_m[1]=0.;
    invalid_tail.elastic_energy_j=0.;
    invalid_tail.camber_state[1]=.2;
    bt::require(update_fast_brush(spec,loaded_patch,control_velocity,dt,true,invalid_tail,out)==CoreStatus::ok,
                "zero-displacement spare capacity with finite camber remains supported");

    // Drop a loaded spring at zero speed: no external work, all old stored
    // energy must be released rather than disappearing from the ledger.
    auto unloaded=brush_state(1);unloaded.ux_m[0]=.001;
    unloaded.elastic_energy_j=.5*spec.kx_n_per_m3*p.area_m2*.001*.001;
    const double released=unloaded.elastic_energy_j;
    patches[0]=p;patches[0].normal_load_n=0.;patches[0].force_world_n={};v={};
    bt::require(update_fast_brush(spec,patches,v,dt,true,unloaded,out)==CoreStatus::ok &&
                unloaded.elastic_energy_j==0. && std::abs(unloaded.convective_release_j-released)<=1e-12 &&
                std::abs(out[0].loss_j-released)<=1e-12,"contact exit releases fast spring storage exactly once");

    auto overflowing=brush_state(1);
    overflowing.ux_m[0]=1e150;overflowing.convective_release_j=std::numeric_limits<double>::max();
    overflowing.elastic_energy_j=.5*spec.kx_n_per_m3*p.area_m2*1e150*1e150;
    const auto overflow_before=bytes(overflowing);
    bt::require(update_fast_brush(spec,patches,v,dt,true,overflowing,out)==CoreStatus::invalid_geometry &&
                bytes(overflowing)==overflow_before,"overflow of cumulative fast release rejects before material commit");
}); }
