#include "support/migration_test.hpp"
#include "support/tyre_fixture.hpp"
#include "support/terrain_fixture.hpp"
#include "pneumatic3d/geometry.hpp"
#include "pneumatic3d/kernel_math.hpp"
#include "terrain/source3d.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <numeric>
#include <vector>

int main() { return bt::run([] {
    using namespace bike::pneumatic3d;
    using namespace bike::pneumatic3d::detail;
    using bike::spatial::TyreTier;
    const double pi=std::numbers::pi;
    bt::require(tier_dims(TyreTier::fast).n_theta_fan==64 && tier_dims(TyreTier::detailed).n_theta_material==1024,"frozen numerical tiers");
    bt::invalid([] { (void)make_torus_grid({64,7,256},.04,.061); });
    bike::terrain::Terrain3dSource flat(spatial_tests::plane_grid(0.,0.,0.,33,33,.25,.25));
    for (const auto tier:{TyreTier::fast,TyreTier::detailed}) {
        const auto g=make_torus_grid(tier_dims(tier),.3675,.061);
        const double area=std::accumulate(g.geometric_area_m2.begin(),g.geometric_area_m2.end(),0.);
        const double exact=g.r_m*(g.theta_max_rad-g.theta_min_rad)*
            (g.R_m*(g.phi_max_rad-g.phi_min_rad)+g.r_m*(std::sin(g.phi_max_rad)-std::sin(g.phi_min_rad)));
        bt::require(std::abs(area-exact)<=1e-10,"finite-volume fan area has no midpoint mass defect");
        bt::require(std::abs(spatial_tests::full_torus_area(g)-4.*pi*pi*g.R_m*g.r_m)<=1e-8,"independent full torus area");
        bt::require(std::abs(spatial_tests::full_torus_volume(g)-2.*pi*pi*g.R_m*g.r_m*g.r_m)<=1e-8,"independent full torus volume");
        for (const double yaw:{0.,37.*pi/180.,pi/2.}) for (const double camber:{-45.,-15.,-5.,0.,5.,15.,45.}) {
            std::vector<ContactElement3d> elements(g.element_count());
            const auto kin=spatial_tests::wheel_at({3.,0.,.352},yaw,camber*pi/180.,3.,3./.3675);
            bt::require(query_geometry(g,kin,flat.query_view(),elements)==CoreStatus::ok,"flat fan query succeeds");
            std::size_t contacts=0;
            for (const auto& e:elements) if(e.valid) {
                bt::require(std::abs(dot(e.longitudinal_world,e.lateral_world))<=1e-12,"tangent axes orthogonal");
                bt::require(std::abs(norm(e.normal_world)-1.)<=1e-12 && std::abs(norm(e.longitudinal_world)-1.)<=1e-12 && std::abs(norm(e.lateral_world)-1.)<=1e-12,"contact axes unit");
                bt::require(std::abs(e.point_world_m[2])<=1e-6,"ray contact agrees with analytic plane");
                if(e.penetration_m>0.) ++contacts;
            }
            if(camber==0.) bt::require(contacts>0,"loaded upright wheel has contacts");
        }
        std::vector<ContactElement3d> a(g.element_count()),b(g.element_count());
        const auto k0=spatial_tests::wheel_at({3.,0.,.352},0.,.1,0.,0.);
        const auto k1=spatial_tests::wheel_at({3.,0.,.352},pi/2.,.1,0.,0.);
        bt::require(query_geometry(g,k0,flat.query_view(),a)==CoreStatus::ok && query_geometry(g,k1,flat.query_view(),b)==CoreStatus::ok,"yaw pair evaluated");
        for (std::size_t i=0;i<a.size();++i) {
            bt::require(a[i].valid==b[i].valid,"yaw does not change contact topology on flat terrain");
            if(a[i].valid) bt::require(std::abs(a[i].penetration_m-b[i].penetration_m)<=1e-12 && std::abs(a[i].projected_area_m2-b[i].projected_area_m2)<=1e-12,"yaw invariance of local geometry");
        }
        auto degenerate=k0; degenerate.axle_world={0.,0.,1.};
        bt::require(query_geometry(g,degenerate,flat.query_view(),a)==CoreStatus::invalid_geometry,"vertical axle has no downward fan");
        bt::require(a.front().reason==ReasonCode3d::degenerate_down_projection,"degenerate fan gives named reason");
        auto buried=k0; buried.hub.position_world_m[2]=-.5;
        bt::require(query_geometry(g,buried,flat.query_view(),a)==CoreStatus::ok,"element-level bad origins are reported");
        bool found=false; for(const auto& e:a) if(e.reason==ReasonCode3d::origin_below_surface) { found=true; bt::require(!e.valid,"buried ray not a valid contact"); }
        bt::require(found,"below-surface condition survives geometry boundary");
    }
    const auto plus=spatial_tests::wheel_at({},0.,.2,0.,0.);
    const auto minus=spatial_tests::wheel_at({},0.,-.2,0.,0.);
    bt::require(std::abs(plus.axle_world[2]+minus.axle_world[2])<=1e-12,"normal-dot-axle, not lateral-dot-axle, is odd under camber mirror");
}); }
