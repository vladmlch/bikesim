#include "support/migration_test.hpp"
#include "support/spatial_fixture.hpp"
#include "fixture_paths.hpp"
#include "terrain/spec_io.hpp"
#include "terrain/trackfile.hpp"
#include "terrain/source3d.hpp"
#include "pneumatic/geometry.hpp"

#include <bit>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>

namespace {
using namespace bike::terrain;
bool exact(double a,double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }
ProfileExtrusion3d from_track(const TrackSpec& track) {
    const auto points = profile(track,.005);
    std::vector<double> x,z;
    for (const auto& point : points) { x.push_back(point[0]); z.push_back(point[1]); }
    auto result = ProfileExtrusion3d::create(std::move(x),std::move(z),std::vector<std::uint32_t>(points.size()-1,1),
        {{1,"fixture",.8,.6,12.,4.5,{}}},1.5,track_to_dict(track));
    bt::require(result.has_value(),"valid analytic profile must construct");
    return std::move(*result);
}
void heights(const TrackSpec& track) {
    const auto vertices = profile(track,.005);
    const SignedProfile legacy(vertices);
    const auto extrusion = from_track(track);
    for (const auto& vertex : vertices) bt::require(exact(extrusion.sample(vertex[0],.7).height_m,legacy.height(vertex[0])),"exact legacy knot height");
    for (int i = 0; i <= 1000; ++i) {
        const double x = vertices.front()[0] + (vertices.back()[0]-vertices.front()[0]) * static_cast<double>(i)/1000.;
        const auto positive = extrusion.sample(x,.7), negative = extrusion.sample(x,-.7);
        bt::require(positive.status == QueryStatus3d::ok && exact(positive.height_m,legacy.height(x)),"interior height must delegate to legacy interpolation");
        bt::require(exact(positive.height_m,negative.height_m) && positive.normal_world[1] == 0.,"extrusion is exactly independent of lateral coordinate");
    }
    bt::require(extrusion.sample(2.,std::nextafter(1.5,2.)).status == QueryStatus3d::outside,"corridor has no extrapolation");
    bt::require(extrusion.sample(-1e-9,0.).status == QueryStatus3d::outside,"longitudinal domain has no extension");
}
}

int main() { return bt::run([] {
    heights(flat(20.));
    const auto root = std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT) / "examples/research";
    heights(load_track(root / "rough_uphill_savage.toml"));
    heights(load_track(root / "alpine_rocky.toml"));
    const auto surface = from_track(flat(20.));
    const double yaw = std::numbers::pi / 6., pitch = -std::numbers::pi / 9.;
    const std::array<double,3> direction{std::cos(pitch)*std::cos(yaw),std::cos(pitch)*std::sin(yaw),std::sin(pitch)};
    const auto oblique = surface.ray_cast({2.,0.,.2},direction,2.);
    bt::require(oblique.status == QueryStatus3d::ok && std::abs(oblique.distance_m - .2/-direction[2]) <= 1e-9,"oblique world-distance law");
    bt::require(surface.ray_cast({2.,0.,0.},{0.,1.,0.},1.).distance_m == 0.,"pure lateral on-surface ray hits origin");
    bt::require(surface.ray_cast({2.,0.,1.},{0.,1.,0.},1.).status == QueryStatus3d::miss,"pure lateral ray above flat profile misses");
    bt::require(surface.ray_cast({2.,0.,-.01},{0.,1.,0.},1.).status == QueryStatus3d::origin_below_surface,"below origin distinct from miss");
    const double inv = 1./std::sqrt(2.);
    const auto entry = surface.ray_cast({-1.,0.,1.},{inv,0.,-inv},3.);
    bt::require(entry.status == QueryStatus3d::ok && std::abs(entry.distance_m-std::sqrt(2.)) <= 1e-9,"outside-entry clipping retains world distance");
    biketyre::RayRing ring(33,.6);
    std::vector<double> z(4001,0.);
    biketyre::RoadProfile road(0.,.005,std::move(z));
    const auto planar = biketyre::intersect(ring,2.,.3,.8,road);
    for (std::size_t i=0;i<ring.n();++i) {
        const auto spatial = surface.ray_cast({2.,0.,.3},{ring.ux()[i],0.,ring.uz()[i]},.8);
        bt::require(spatial.status == QueryStatus3d::ok && exact(spatial.distance_m,planar.r_m[i]),"dy=0 retains legacy ray arithmetic exactly");
    }
    const std::vector<SurfaceSpec3d> table{{1,"test",.8,.6,12.,4.5,{}}};
    auto sparse = ProfileExtrusion3d::create({0.,1.,2.},{0.,0.,0.},{1,1},table,1.5,bike::io::Value{bike::io::Value::Object{}});
    auto dense = ProfileExtrusion3d::create({0.,.5,1.,2.},{0.,0.,0.,0.},{1,1,1},table,1.5,bike::io::Value{bike::io::Value::Object{}});
    bt::require(sparse && dense && sparse->fingerprint_hex()!=dense->fingerprint_hex(),"equal samples with different original knots must have different identities");
    const Terrain3dSource owner(std::move(*dense));
    const auto query=owner.query_view();
    bt::require(owner.kind()==Terrain3dKind::profile_extrusion && query.context && query.sample_fn && query.ray_fn,"query view binds real immutable owner");
    bt::require(query.sample(1.,.2).status==QueryStatus3d::ok,"bound query dispatch");
    bt::require(!ProfileExtrusion3d::create({0.,0.},{0.,0.},{1},table,1.5,{}),"duplicate knots rejected");
    bt::require(!ProfileExtrusion3d::create({0.,1.},{0.,0.},{7},table,1.5,{}),"unresolved surface rejected");
}); }
