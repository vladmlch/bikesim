#include "support/migration_test.hpp"
#include "support/tyre_fixture.hpp"
#include "support/tyre_test_helpers.hpp"
#include "pneumatic3d/bristles.hpp"
#include "pneumatic3d/brush_detail.hpp"
#include "pneumatic3d/kernel_math.hpp"
#include "spatial/frames.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

int main() { return bt::run([] {
    using namespace bike::pneumatic3d;
    using namespace bike::pneumatic3d::detail;
    using namespace spatial_tests::tyre;
    auto spec=spatial_tests::synthetic_spec(bike::spatial::TyreTier::detailed).brush[0];
    spec.camber_coefficient_per_rad=0.;spec.mu_x_scale=spec.mu_y_scale=.8;
    std::array<ContactElement3d,4> elements;
    std::array<NormalElement3d,4> normal{};
    for(std::size_t i=0;i<4;++i) {
        elements[i]=element(static_cast<std::uint32_t>(i),.01,.0025);
        elements[i].point_world_m={.01*static_cast<double>(i),0.,0.};
        normal[i].active=true;normal[i].normal_load_n=100.;
    }
    auto kin=spatial_tests::wheel_at({0.,0.,.3675},0.,0.,.1,0.);
    auto s=brush_state(1,4);
    std::array<PatchWrench,4> out{};
    for(unsigned step=0;step<10;++step) {
        bt::require(update_detailed_brush(spec,elements,normal,kin,.001,true,s,out)==CoreStatus::ok,"material bristles advance");
        for(std::size_t i=0;i<4;++i)
            bt::require(std::abs(s.bristle_ux_m[i]-.1*.001*static_cast<double>(step+1))<=1e-12,"unsaturated conveyor displacement follows residence time");
    }
    const auto before=bytes(s);
    bt::require(update_detailed_brush(spec,elements,normal,kin,.001,false,s,out)==CoreStatus::ok && bytes(s)==before,"detailed probe does not advance history");
    auto rotated_elements=elements;auto rotated_kin=kin;
    const bike::spatial::Quat q{std::cos(std::numbers::pi/4.),0.,0.,std::sin(std::numbers::pi/4.)};
    const auto rotate=[&](const bike::spatial::Vec3& x){return bike::spatial::rotate_body_to_world(q,x);};
    for(auto& e:rotated_elements) {
        e.point_world_m=rotate(e.point_world_m);e.normal_world=rotate(e.normal_world);
        e.longitudinal_world=rotate(e.longitudinal_world);e.lateral_world=rotate(e.lateral_world);
    }
    rotated_kin.hub.position_world_m=rotate(kin.hub.position_world_m);
    rotated_kin.hub.linear_velocity_world_mps=rotate(kin.hub.linear_velocity_world_mps);
    rotated_kin.axle_world=rotate(kin.axle_world);
    auto sr=s,reference=s;
    std::array<PatchWrench,4> rotated{},ref{};
    bt::require(update_detailed_brush(spec,rotated_elements,normal,rotated_kin,.001,true,sr,rotated)==CoreStatus::ok &&
                update_detailed_brush(spec,elements,normal,kin,.001,true,reference,ref)==CoreStatus::ok,"rotated installations both advance");
    bt::require(std::abs(sr.elastic_energy_j-reference.elastic_energy_j)<=1e-12,"rigid frame rotation preserves stored energy");
    for(std::size_t i=0;i<4;++i) bt::require(norm(sub(rotated[i].force_world_n,rotate(ref[i].force_world_n)))<=1e-9,"force rotates with tangent frame");
    const double stored=s.elastic_energy_j,release=s.convective_release_j;
    for(auto& e:elements) e.valid=false;
    bt::require(update_detailed_brush(spec,elements,normal,kin,.001,true,s,out)==CoreStatus::ok,"contact exit accepted");
    bt::require(s.elastic_energy_j==0. && std::abs(s.convective_release_j-release-stored)<=1e-12,"exit releases bristle energy exactly once");
    for(auto& e:elements) e.valid=true;
    kin.hub.linear_velocity_world_mps={-.1,0.,0.};
    bt::require(update_detailed_brush(spec,elements,normal,kin,.001,true,s,out)==CoreStatus::ok && out[0].fx_n>0.,"reverse reentry force opposes slip");

    // A zero-width Stribeck transition still has a static peak at exactly
    // zero slip; the fast owner uses the same convention.
    auto grid=bike::terrain::Grid3d::create(
        {.nx=4,.ny=4,.x0_m=0.,.y0_m=0.,.dx_m=1.,.dy_m=1.,.dtype=0},
        std::vector<double>(16,0.),std::vector<std::uint32_t>(9,1),
        {{.id=1,.name="static-step",.mu_peak=1.,.mu_slide=.1,
          .slip_stiffness_per_load=20.,.stribeck_speed_mps=.1,.description="analytic control"}},
        {.generator_version="analytic/v1",.resolved_spec=bike::io::Value{std::string("flat")},
         .seed_decimal="0",.units="m",.height_datum_m=0.,.provenance=bike::io::Value{std::string("analytic")}});
    bt::require(grid.has_value(),"static friction grid constructed");
    bike::terrain::Terrain3dSource surface(std::move(*grid));
    auto view=surface.query_view();
    // Serialized terrain requires a positive transition speed. Supply the
    // exact zero-width limiting law through an explicitly synthetic table,
    // keeping the real geometry owner and its validated registry unchanged.
    const std::array<bike::terrain::SurfaceSpec3d,1> static_material{{
        {.id=1,.name="static-step",.mu_peak=1.,.mu_slide=.1,
         .slip_stiffness_per_load=20.,.stribeck_speed_mps=0.,.description="analytic limiting law"}}};
    view.surface_table=static_material;
    auto static_state=brush_state(1,4);
    kin.hub.linear_velocity_world_mps={};kin.relative_spin_rad_s=0.;
    spec.mu_x_scale=spec.mu_y_scale=1.;
    for(std::size_t i=0;i<4;++i) {
        elements[i].surface_id=1;
        const double stiffness=spec.kx_n_per_m3*elements[i].geometric_area_m2;
        static_state.bristle_ux_m[i]=.5*normal[i].normal_load_n/stiffness;
        static_state.bristle_energy_j[i]=.5*stiffness*static_state.bristle_ux_m[i]*static_state.bristle_ux_m[i];
        static_state.elastic_energy_j+=static_state.bristle_energy_j[i];
    }
    bt::require(update_detailed_resolved(spec,elements,normal,kin,.001,false,static_state,out,view)==CoreStatus::ok &&
                std::abs(out[0].fx_n+.5*normal[0].normal_load_n)<=1e-12,"zero slip preserves a preload below peak and above sliding friction");

    auto overflowing=brush_state(1,4);overflowing.convective_release_j=std::numeric_limits<double>::max();
    overflowing.bristle_energy_j[0]=1e307;overflowing.elastic_energy_j=1e307;
    for(auto& e:elements)e.valid=false;
    const auto overflow_before=bytes(overflowing);
    bt::require(update_detailed_brush(spec,elements,normal,kin,.001,true,overflowing,out)==CoreStatus::invalid_geometry &&
                bytes(overflowing)==overflow_before,"cumulative bristle release overflow rejects before clearing material history");
    // The fast/detailed measured curve comparison and subcell interpolation
    // refinement remain V04/C10 evidence; never print invented error values.
    std::cout << "{\"suite\":\"tyre_brush_detailed\",\"curve_comparison\":null,\"status\":\"pending V04 material-frame contract\"}\n";
}); }
