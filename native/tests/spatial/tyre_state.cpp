// W27 source acceptance; execution belongs to V04.
#include "support/migration_test.hpp"
#include "spatial/support/terrain_fixture.hpp"
#include "spatial/support/tyre_fixture.hpp"
#include "pneumatic3d/model.hpp"
#include "pneumatic3d/state.hpp"
#include "pneumatic3d/state_detail.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace bike::pneumatic3d;
namespace sp=bike::spatial;
TyreStateIdentity3d identity(sp::TyreTier tier,const bike::terrain::Terrain3dSource& terrain) {
    TyreStateIdentity3d id;id.tier=tier;id.dims=tier_dims(tier);
    id.patch_capacity=id.dims.n_theta_fan*id.dims.n_phi;
    id.topology_fingerprint_hex=std::string(64,'a');id.config_fingerprint_hex=std::string(64,'b');
    id.terrain_sha256_hex=terrain.fingerprint_hex();id.calibration_sha256_hex=std::string(64,'c');
    return id;
}
void put64(std::vector<std::byte>& bytes,std::size_t at,std::uint64_t x) {
    for(unsigned i=0;i<8;++i)bytes[at+i]=static_cast<std::byte>((x>>(8U*i))&255U);
}
std::array<WheelKinematics3d,2> input(const Pneumatic3dSpec& spec,std::size_t tick) {
    std::array<WheelKinematics3d,2> k;
    const double t=static_cast<double>(tick)*.0005;
    for(std::size_t w=0;w<2;++w) {
        const double speed=t<.2?2.0:t<.4?2.-8.*(t-.2):.4;
        k[w]=spatial_tests::wheel_at({4.+.1*static_cast<double>(w),0.,spec.outer_radius_m[w]-.004},
            .07*std::sin(t*3.),.03*std::cos(t),speed,(1.-.2*std::sin(t*7.))*speed/spec.outer_radius_m[w]);
        k[w].wheel_phase_rad=t*4.;
    }
    return k;
}
void codecs(sp::TyreTier tier) {
    bike::terrain::Terrain3dSource terrain(spatial_tests::plane_grid(0.,0.,0.,21,17,.5,.2));
    bt::require(terrain.query_view().surface(1)==&terrain.grid()->surfaces().front() &&
        terrain.query_view().surface(42)==nullptr,"grid query binds its validated immutable material table");
    const auto spec=spatial_tests::synthetic_spec(tier);const auto id=identity(tier,terrain);
    Pneumatic3dModel a(spec,terrain.query_view(),id);
    const auto zero=encode_tyre_state(a.committed_state());
    auto decoded=decode_tyre_state(zero,id);
    bt::require(decoded.has_value(),"cold fully sized tyre state decodes");
    bt::require(encode_tyre_state(*decoded)==zero,"canonical BKS3 roundtrip is byte exact");
    bt::require(bike::io::canonical_json(tyre_identity_to_wire(tyre_identity_from_wire(tyre_identity_to_wire(id))))==
        bike::io::canonical_json(tyre_identity_to_wire(id)),"identity wire exact-key roundtrip");
    bt::require(base64_decode_strict(base64_canonical(zero))==std::expected<std::vector<std::byte>,std::string>{zero},
        "canonical base64 retains the exact binary state");
    for(const auto bad:{"A","====","YQ= =","YQ==\n","YR==","YWJ=","YQ==AAAA","AA=A"})
        bt::require(!base64_decode_strict(bad),"malformed or noncanonical base64 must fail");
    for(std::size_t n=0;n<7;++n) {
        const auto sample=std::span<const std::byte>(zero).first(n);
        const auto roundtrip=base64_decode_strict(base64_canonical(sample));
        bt::require(roundtrip&&std::ranges::equal(*roundtrip,sample),"all padding lengths roundtrip");
    }
    auto changed=id;changed.terrain_sha256_hex=std::string(64,'d');
    bt::require(!decode_tyre_state(zero,changed),"different terrain identity is rejected before state allocation");
    changed=id;changed.tier=tier==sp::TyreTier::fast?sp::TyreTier::detailed:sp::TyreTier::fast;
    bt::require(!decode_tyre_state(zero,changed),"different tier is rejected");
    auto corrupt=zero;corrupt[4]=std::byte{2};bt::require(!decode_tyre_state(corrupt,id),"unknown version rejected");
    corrupt=zero;corrupt.pop_back();bt::require(!decode_tyre_state(corrupt,id),"truncated last section rejected");
    corrupt=zero;corrupt.push_back(std::byte{0});bt::require(!decode_tyre_state(corrupt,id),"trailing bytes rejected");
    corrupt=zero;put64(corrupt,300,std::bit_cast<std::uint64_t>(std::numeric_limits<double>::quiet_NaN()));
    bt::require(!decode_tyre_state(corrupt,id),"nonfinite carcass payload rejected");
    auto invalid=*decoded;invalid.patches[0].element_to_patch_slot[0]=id.patch_capacity;
    bt::require(a.restore(invalid)!=CoreStatus::ok,"invalid mapping cannot mutate live model");
    bt::require(encode_tyre_state(a.committed_state())==zero,"failed restore is atomic");
    invalid=*decoded;invalid.brush[0].ux_m[0]=.1;
    bt::invalid([&]{(void)encode_tyre_state(invalid);});
    invalid=*decoded;invalid.carcass[0].previous_penetration_m[0]=.01;
    bt::require(a.restore(invalid)==CoreStatus::invalid_input && encode_tyre_state(a.committed_state())==zero,
                "inactive cell cannot restore a fictitious penetration rate history");
    corrupt=zero;
    const auto material_cells=static_cast<std::size_t>(id.dims.n_theta_material)*id.dims.n_phi;
    put64(corrupt,300U+16U*material_cells,std::bit_cast<std::uint64_t>(.01));
    bt::require(!decode_tyre_state(corrupt,id),"binary codec rejects the same inactive penetration inconsistency");
    invalid=*decoded;invalid.carcass[0].active[0]=1;
    invalid.carcass[0].deflection_m[0]=.01;invalid.carcass[0].previous_penetration_m[0]=.02;
    bt::require(a.restore(invalid)==CoreStatus::invalid_input && encode_tyre_state(a.committed_state())==zero,
                "active cell must retain the producer's identical accepted penetration lanes");
    auto relaxed_memory=*decoded;
    const auto memory_grid=make_torus_grid(id.dims,spec.outer_radius_m[0],spec.casing_width_m[0]);
    relaxed_memory.carcass[0].maxwell_q_n_per_m2[0]=17.;
    relaxed_memory.carcass[0].stored_energy_j=.5*memory_grid.geometric_area_m2[0]*17.*17./spec.carcass[0].maxwell_k_n_per_m3;
    const auto memory_bytes=encode_tyre_state(relaxed_memory);
    bt::require(a.restore(relaxed_memory)==CoreStatus::ok && encode_tyre_state(a.committed_state())==memory_bytes &&
                decode_tyre_state(memory_bytes,id).has_value(),
                "inactive material may retain finite Maxwell relaxation memory with zero penetration");
    bt::require(a.restore(*decoded)==CoreStatus::ok,"valid cold control restores the original unloaded state");
    invalid=*decoded;
    invalid.carcass[0].dissipated_j=std::numeric_limits<double>::max();
    invalid.carcass[1].dissipated_j=std::numeric_limits<double>::max();
    bt::require(a.restore(invalid)==CoreStatus::invalid_input &&
                encode_tyre_state(a.committed_state())==zero,
                "finite individual wheel energies cannot restore an overflowing total");
    bt::invalid([&]{(void)encode_tyre_state(invalid);});
    auto bad_spec=spec;bad_spec.brush[0].pneumatic_trail_m=-.01;
    bt::invalid([&]{Pneumatic3dModel bad(bad_spec,terrain.query_view(),id);});
    const std::array<double,7> render_pose{0.,0.,1.,1.,0.,0.,0.};
    bt::require(!decode_tyre_state(std::as_bytes(std::span(render_pose)),id),"render-only integration pose cannot seed complete tyre state");
    for(const double phase:std::array{-2.*std::numbers::pi-.001,-2.*std::numbers::pi+.001,
                                    -.001,0.,.001,2.*std::numbers::pi-.001,2.*std::numbers::pi+.001}) {
        auto k=input(spec,0);
        for(auto& wheel:k) {
            wheel.wheel_phase_rad=phase;
            wheel.relative_spin_rad_s=std::copysign(std::abs(wheel.relative_spin_rad_s),phase);
        }
        bt::require(a.evaluate(k,{},0.,false)==CoreStatus::ok,"material-order wrap/reverse probe is valid");
        const auto forces=a.wrenches(0);
        if(tier==sp::TyreTier::detailed) {
            const auto grid=make_torus_grid(id.dims,spec.outer_radius_m[0],spec.casing_width_m[0]);
            std::vector<ContactElement3d> geometry(grid.element_count());
            bt::require(query_geometry(grid,k[0],terrain.query_view(),geometry)==CoreStatus::ok,
                        "independent geometry supplies material IDs for the order oracle");
            std::vector<std::pair<std::uint32_t,sp::Vec3>> ordered;
            for(const auto& e:geometry)if(e.valid&&e.penetration_m>0.)
                ordered.emplace_back(e.material_cell,e.point_world_m);
            std::sort(ordered.begin(),ordered.end(),[](const auto& lhs,const auto& rhs){return lhs.first<rhs.first;});
            bt::require(!ordered.empty(),"material-order regression contains loaded contacts");
            std::size_t at=0;
            for(const auto& force:forces)if(force.valid) {
                bt::require(at<ordered.size()&&force.point_world_m==ordered[at].second,
                            "detailed world-force publication follows sorted material IDs through wrap and reverse");
                ++at;
            }
            bt::require(at==ordered.size(),"material ordering neither drops nor duplicates a loaded element");
        } else {
            const auto patches=a.patches(0);
            bt::require(forces.size()==patches.size(),"fast force publication retains patch order");
            for(std::size_t i=0;i<forces.size();++i)
                bt::require(forces[i].patch_id==patches[i].id,"fast patch association is unchanged by detailed ordering");
        }
    }
    bt::require(encode_tyre_state(a.committed_state())==zero,"material-order probes preserve complete committed state");
    for(std::size_t t=0;t<37;++t)bt::require(a.evaluate(input(spec,t),{},.0005,true)==CoreStatus::ok,"prefix accepted");
    const auto checkpoint=encode_tyre_state(a.committed_state());
    const auto checkpoint_decoded=decode_tyre_state(checkpoint,id);bt::require(checkpoint_decoded.has_value(),"active state decodes");
    Pneumatic3dModel b(spec,terrain.query_view(),id);
    bt::require(b.restore(*checkpoint_decoded)==CoreStatus::ok,"active state restored");
    for(std::size_t t=37;t<1037;++t) {
        const auto k=input(spec,t);
        bt::require(a.evaluate(k,{},.0005,true)==CoreStatus::ok&&b.evaluate(k,{},.0005,true)==CoreStatus::ok,
            "same inputs accepted after continuation");
        bt::require(encode_tyre_state(a.committed_state())==encode_tyre_state(b.committed_state()),
            "1000 accepted steps retain complete same-binary material history");
    }
}
void extrusion_materials(sp::TyreTier tier) {
    namespace terrain=bike::terrain;
    // Deliberately sparse, unsorted IDs: lookup is by identity, never by
    // vector offset or by assuming every road has unit friction.
    auto source=terrain::ProfileExtrusion3d::create(
        {-10.,5.,20.},{0.,0.,0.},{7,42},
        {{.id=42,.name="high-grip",.mu_peak=.8,.mu_slide=.8,
          .slip_stiffness_per_load=20.,.stribeck_speed_mps=.1,.description="analytic"},
         {.id=7,.name="low-grip",.mu_peak=.2,.mu_slide=.2,
          .slip_stiffness_per_load=20.,.stribeck_speed_mps=.1,.description="analytic"}},
        1.5,bike::io::Value{bike::io::Value::Object{}});
    bt::require(source.has_value(),"two-material extrusion is valid");
    terrain::Terrain3dSource owner(std::move(*source));
    auto spec=spatial_tests::synthetic_spec(tier);
    for(auto& brush:spec.brush) {
        brush.camber_coefficient_per_rad=0.;brush.pneumatic_trail_m=0.;
    }
    const auto id=identity(tier,owner);
    const auto view=owner.query_view();
    auto model=std::make_unique<Pneumatic3dModel>(spec,view,id);
    std::array<WheelKinematics3d,2> kin;
    for(std::size_t w=0;w<2;++w)
        kin[w]=spatial_tests::wheel_at({w==0?2.:8.,0.,spec.outer_radius_m[w]-.004},0.,0.,100.,0.);
    // Zero spin and large prescribed slip put both synthetic brushes on
    // their analytical Coulomb cap within this single material interval.
    bt::require(model->evaluate(kin,{},.001,true)==CoreStatus::ok,
        "production fast/detailed tyre accepts a real profile extrusion");
    for(std::size_t w=0;w<2;++w) {
        const std::uint32_t material_id=w==0?7U:42U;
        const double mu=w==0?.2:.8;
        std::size_t contacts=0;
        for(const auto& wrench:model->wrenches(w)) if(wrench.valid) {
            ++contacts;
            bt::require(wrench.surface_id==material_id && wrench.normal_load_n>0.,
                "each loaded wheel retains its extrusion segment material");
            bt::require(std::abs(wrench.fx_n+mu*wrench.normal_load_n)<=
                1e-10*std::max(1.,wrench.normal_load_n) && std::abs(wrench.fy_n)<=1e-12,
                "real extrusion material sets the saturated longitudinal friction force");
        }
        bt::require(contacts>0,"material force assertion exercises loaded contacts");
    }
    auto missing=view;missing.surface_table={};
    bt::invalid([&]{Pneumatic3dModel rejected(spec,missing,id);});
    const std::array<terrain::SurfaceSpec3d,1> unrelated{{
        {.id=99,.name="unrelated",.mu_peak=1.,.mu_slide=1.,
         .slip_stiffness_per_load=20.,.stribeck_speed_mps=.1,.description="analytic"}}};
    missing.surface_table=unrelated;
    auto unresolved=std::make_unique<Pneumatic3dModel>(spec,missing,id);
    const auto before=encode_tyre_state(unresolved->committed_state());
    bt::require(unresolved->evaluate(kin,{},.001,true)==CoreStatus::invalid_geometry &&
        encode_tyre_state(unresolved->committed_state())==before,
        "unresolved extrusion surface ID rejects before any material-state commit");
}
}
int main(){return bt::run([]{
    extrusion_materials(sp::TyreTier::fast);extrusion_materials(sp::TyreTier::detailed);
    codecs(sp::TyreTier::fast);codecs(sp::TyreTier::detailed);
});}
