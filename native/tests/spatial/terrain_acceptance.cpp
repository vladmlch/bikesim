#include "support/migration_test.hpp"
#include "support/spatial_fixture.hpp"
#include "support/terrain_fixture.hpp"
#include "terrain/raycast3d.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numbers>

int main() { return bt::run([] {
    using namespace bike::terrain;
    std::uint64_t queries=0, exhausted=0, root_subdivisions=0;
    const auto exercise=[&](const Grid3d& grid) {
        const auto& descriptor=grid.descriptor();
        for (int iy=0;iy<=30;++iy) for (int ix=0;ix<=50;++ix) {
            const double x=ix==50?grid.x_max_m():descriptor.x0_m+(grid.x_max_m()-descriptor.x0_m)*static_cast<double>(ix)/50.;
            const double y=iy==30?grid.y_max_m():descriptor.y0_m+(grid.y_max_m()-descriptor.y0_m)*static_cast<double>(iy)/30.;
            const auto sample=grid.sample(x,y);
            ++queries;
            bt::require(sample.status==QueryStatus3d::ok && std::isfinite(sample.height_m),"registered sampler domain is finite and valid");
            bt::require(std::abs(std::hypot(sample.normal_world[0],sample.normal_world[1],sample.normal_world[2])-1.)<=1e-12,"registered normals are unit");
            const auto hit=grid.ray_cast({x,y,sample.height_m+1.},{0.,0.,-1.},2.);
            ++queries;
            if(hit.status==QueryStatus3d::budget_exhausted)++exhausted;
            bt::require(hit.status==QueryStatus3d::ok && std::abs(hit.distance_m-1.)<=1e-6,"registered downward ray agrees with sampling");
            for(const double component:hit.point_world_m)bt::require(std::isfinite(component),"finite hit point");
            for(const double component:hit.normal_world)bt::require(std::isfinite(component),"finite hit normal");
        }
        const std::array<std::array<double,2>,4> outside{{
            {std::nextafter(descriptor.x0_m,-std::numeric_limits<double>::infinity()),descriptor.y0_m},
            {std::nextafter(grid.x_max_m(),std::numeric_limits<double>::infinity()),descriptor.y0_m},
            {descriptor.x0_m,std::nextafter(descriptor.y0_m,-std::numeric_limits<double>::infinity())},
            {descriptor.x0_m,std::nextafter(grid.y_max_m(),std::numeric_limits<double>::infinity())}}};
        for(const auto& point:outside) {
            bt::require(grid.sample(point[0],point[1]).status==QueryStatus3d::outside,"nextafter outside is never extrapolated");
            bt::require(grid.ray_cast({point[0],point[1],10.},{0.,0.,-1.},20.).status==QueryStatus3d::outside,"outside vertical ray stays outside");
        }
    };
    exercise(spatial_tests::plane_grid(.1,-.2,.3,16,12,.25,.5));
    exercise(spatial_tests::asymmetric_4x4());
    exercise(spatial_tests::bump_grid(.08,.3,64,32,.05,.1));
    exercise(spatial_tests::plane_grid(std::tan(std::numbers::pi/3.),0.,0.,16,12,.25,.5));
    for(const Poly6 polynomial:std::array<Poly6,3>{Poly6{{.25,-1.,1.,0.,0.,0.,0.}},Poly6{{-.5,1.,0.,0.,0.,0.,0.}},Poly6{{.21,-1.,1.,0.,0.,0.,0.}}}) {
        const auto root=earliest_root_bernstein(polynomial,{});
        root_subdivisions+=root.subdivisions;
        if(root.status==QueryStatus3d::budget_exhausted)++exhausted;
        bt::require(root.status==QueryStatus3d::ok,"registered multiple/tangent polynomial fits explicit root budget");
    }
    bt::require(exhausted==0,"registered terrain fixtures must have no budget-exhausted queries");
    // RayHit3d deliberately has no visit counters in W00. A null cell count
    // is honest unavailable telemetry, not a fabricated zero.
    std::cout<<"{\"terrain_queries\":"<<queries<<",\"budget_exhausted_count\":"<<exhausted
             <<",\"direct_root_subdivisions\":"<<root_subdivisions<<",\"ray_cell_visits\":null}\n";
}); }
