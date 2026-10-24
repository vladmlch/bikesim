#include "support/migration_test.hpp"
#include "support/tyre_fixture.hpp"
#include "support/tyre_test_helpers.hpp"
#include "pneumatic3d/brush_detail.hpp"

#include <array>
#include <cmath>

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
}); }
