// W27 source acceptance; execution belongs to V04.
#include "support/migration_test.hpp"
#include "spatial/support/terrain_fixture.hpp"
#include "spatial/support/tyre_fixture.hpp"
#include "pneumatic3d/model.hpp"
#include "pneumatic3d/state.hpp"
#include "pneumatic3d/state_detail.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

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
    const std::array<double,7> render_pose{0.,0.,1.,1.,0.,0.,0.};
    bt::require(!decode_tyre_state(std::as_bytes(std::span(render_pose)),id),"render-only integration pose cannot seed complete tyre state");
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
}
int main(){return bt::run([]{codecs(sp::TyreTier::fast);codecs(sp::TyreTier::detailed);});}
