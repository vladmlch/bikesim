#include "support/migration_test.hpp"
#include "spatial/support/ride_qualification.hpp"
#include "spatial/support/tyre_fixture.hpp"
#include "spatial/support/tyre_test_helpers.hpp"

#include <bit>

namespace {
namespace ride=spatial_tests::ride;
using bike::terrain::QueryStatus3d;
bool identical(double a,double b) {return std::bit_cast<std::uint64_t>(a)==std::bit_cast<std::uint64_t>(b);}
bike::terrain::GridSample3d plane_sample(const void*,double,double) noexcept {return {QueryStatus3d::ok,0.,{0.,0.,1.},1};}
bike::terrain::RayHit3d plane_ray(const void*,ride::Vec3 origin,ride::Vec3 direction,double limit) noexcept {
    if(origin[2]<0.)return {.status=QueryStatus3d::origin_below_surface};
    if(direction[2]>=0.)return {.status=QueryStatus3d::miss};
    const double t=-origin[2]/direction[2];
    if(t<0. || t>limit)return {.status=QueryStatus3d::miss};
    return {.status=QueryStatus3d::ok,.distance_m=t,
        .point_world_m={origin[0]+t*direction[0],origin[1]+t*direction[1],0.},.normal_world={0.,0.,1.},.surface_id=1};
}
void exact_kernels() {
    using namespace bike::pneumatic3d;
    auto spec=spatial_tests::synthetic_spec(bike::spatial::TyreTier::fast);
    auto left=spatial_tests::tyre::brush_state(1),right=left;
    left.ux_m[0]=right.ux_m[0]=.0003;left.uy_m[0]=.0007;right.uy_m[0]=-.0007;
    left.camber_state[0]=.03;right.camber_state[0]=-.03;
    spec.brush[0].pneumatic_trail_m=0.;
    std::array<Patch3d,1> patches{spatial_tests::tyre::patch()};
    const std::array<ride::Vec3,2> vl{{{.07,.03,0.},{3.,0.,0.}}},vr{{{.07,-.03,0.},{3.,0.,0.}}};
    std::array<PatchWrench,1> wl{},wr{};
    bt::require(update_fast_brush(spec.brush[0],patches,vl,.001,true,left,wl)==CoreStatus::ok &&
        update_fast_brush(spec.brush[0],patches,vr,.001,true,right,wr)==CoreStatus::ok,"mirrored isolated brushes evaluate");
    bt::require(identical(wl[0].fx_n,wr[0].fx_n) && identical(wl[0].fy_n,-wr[0].fy_n) &&
        identical(left.ux_m[0],right.ux_m[0]) && identical(left.uy_m[0],-right.uy_m[0]) &&
        identical(left.elastic_energy_j,right.elastic_energy_j) && identical(wl[0].loss_j,wr[0].loss_j),
        "isolated symmetric brush has exact even/odd IEEE-754 representations");
    const auto grid=make_torus_grid(tier_dims(bike::spatial::TyreTier::fast),.35,.06);
    int context=0;const bike::terrain::Terrain3dQueryView view{bike::terrain::Terrain3dKind::grid,&context,plane_sample,plane_ray};
    const auto a=spatial_tests::wheel_at({3.,.125,.34},0.,0.,3.,3./.35);
    const auto b=spatial_tests::wheel_at({3.,-.125,.34},0.,0.,3.,3./.35);
    std::vector<ContactElement3d> ea(grid.element_count()),eb(grid.element_count());
    bt::require(query_geometry(grid,a,view,ea)==CoreStatus::ok && query_geometry(grid,b,view,eb)==CoreStatus::ok,
        "symmetric geometry uses the production query kernel and an independent plane intersection");
    std::size_t compared=0;
    for(std::size_t t=0;t<grid.dims.n_theta_fan;++t)for(std::size_t p=0;p<grid.dims.n_phi;++p) {
        const auto& l=ea[t*grid.dims.n_phi+p];const auto& r=eb[t*grid.dims.n_phi+grid.dims.n_phi-1-p];
        bt::require(l.ray_status==r.ray_status && l.valid==r.valid,"mirror preserves every geometric query status");
        if(!l.valid)continue;++compared;
        bt::require(identical(l.point_world_m[0],r.point_world_m[0]) && identical(l.point_world_m[1],-r.point_world_m[1]) &&
            identical(l.penetration_m,r.penetration_m) && identical(l.geometric_area_m2,r.geometric_area_m2) &&
            identical(l.projected_area_m2,r.projected_area_m2),"geometry mirror scalar invariants and signed lateral points are byte exact");
    }
    bt::require(compared>0,"exact geometry mirror exercised loaded contact elements");
}
void integrated_mirror() {
    auto terrain=ride::surface_grid([](double x,double y) {
        return .001*y+.003*std::exp(-std::pow((x-7.)/2.,2)-std::pow((y-.5)/.8,2));
    },[](double x,double y){return x>5. && y>-.3?2u:1u;},ride::dry_and_wet(),30.,6.,.1,.1,"smooth asymmetric mirror feature and material zones");
    ride::MirrorSpec spec;spec.terrain=terrain;
    auto a=ride::mirror_rig(spec);spec.reflected=true;auto b=ride::mirror_rig(spec);
    const std::array<ride::Demand,1> demand{{{0.,.03,0.}}};
    const auto left=ride::run(*a,2.,demand,0.,4),right=ride::run(*b,2.,demand,0.,4);
    bt::require(left.valid_duration==2. && right.valid_duration==2. && !left.terminal_reason && !right.terminal_reason,
        "mirror comparison covers two complete valid seconds");
    bt::require(left.points.size()==right.points.size() && !left.points.empty(),"mirror trajectories use identical observation clocks");
    double max_position=0.,max_angle=0.,max_sensor=0.;
    for(std::size_t i=0;i<left.points.size();++i) {
        const auto& l=left.points[i];const auto& r=right.points[i];
        for(std::size_t k=0;k<3;++k) {
            const double polar=k==1?-1.:1.,axial=k==1?1.:-1.;
            max_position=std::max(max_position,std::abs(l.position[k]-polar*r.position[k]));
            max_sensor=std::max({max_sensor,std::abs(l.accel_sensor[k]-polar*r.accel_sensor[k]),std::abs(l.gyro_sensor[k]-axial*r.gyro_sensor[k])});
        }
        const bike::spatial::Quat reflected{r.quaternion[0],-r.quaternion[1],r.quaternion[2],-r.quaternion[3]};
        max_angle=std::max(max_angle,bike::spatial::geodesic_angle_rad(l.quaternion,reflected));
        bt::require(std::abs(l.steer+r.steer)<=1e-7 && std::abs(l.steer_work-r.steer_work)<=1e-7,
            "mirror reverses steering torque and preserves its physical work");
    }
    std::cout << "{\"suite\":\"mirror\",\"duration_s\":2,\"position_error_m\":" << max_position
              << ",\"orientation_error_rad\":" << max_angle << ",\"sensor_axis_error\":" << max_sensor << "}\n";
    bt::require(max_position<=1e-7 && max_angle<=1e-7,"spatial trajectory mirror invariance is within 1e-7 m and rad");
    bt::require(max_sensor<=1e-6,"mirrored sensor mounting follows polar acceleration and axial angular velocity");
}
} // namespace
int main() {return bt::run([]{exact_kernels();integrated_mirror();});}
