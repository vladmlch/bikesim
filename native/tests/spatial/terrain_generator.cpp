#include "support/migration_test.hpp"
#include "support/spatial_fixture.hpp"
#include "terrain/spec_io.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/asset3d.hpp"
#include "terrain/trackfile.hpp"

#include <bit>
#include <cmath>
#include <numbers>
#include <string>

namespace {
using namespace bike::terrain;
Track3dSpec input(std::string_view filename) {
    return detail::load_track3d_spec(spatial_tests::fixture("terrain/genspec/" + std::string(filename)));
}
const bike::io::Value& member(const bike::io::Value& value,std::string_view key) {
    const auto* out = bike::io::find_member(bike::io::as_object(value,key),key);
    bt::require(out != nullptr,"required generator manifest key");
    return *out;
}
void pending_hash_gate(std::string_view filename,const Grid3d& grid) {
    const auto manifest = spatial_tests::json(spatial_tests::fixture("terrain/genspec/manifest.json"));
#if defined(__APPLE__) && defined(__aarch64__)
    constexpr std::string_view platform = "macos-arm64";
#elif defined(__linux__) && defined(__aarch64__)
    constexpr std::string_view platform = "linux-aarch64";
#else
    constexpr std::string_view platform = "unsupported-platform";
#endif
    bool found = false;
    for (const auto& capture : bike::io::as_array(member(manifest,"captures"),"captures")) {
        if (bike::io::as_string(member(capture,"platform"),"platform") != platform ||
            bike::io::as_string(member(capture,"fixture"),"fixture") != filename) continue;
        found = true;
        const auto& expected = member(capture,"expected_sha256");
        bt::require(!bike::io::is_null(expected),"V02 must capture independent per-platform terrain hash; pending is not a pass");
        bt::require(bike::io::as_string(expected,"expected_sha256") == grid.sha256_hex(),"generated asset differs from qualified platform capture");
    }
    bt::require(found,"generator manifest has no entry for this platform and fixture");
}
}

int main() { return bt::run([] {
    const auto rock_spec = input("single_rock.toml");
    const auto rock = generate_grid3d(rock_spec);
    bt::require(write_terrain3d(rock) == write_terrain3d(generate_grid3d(rock_spec)),"same seed/spec must reproduce complete canonical bytes");
    bt::require(std::abs(rock.sample(8.,.5).height_m - .08) <= 1e-9,"rock centre is not at authored coordinates/amplitude");
    const auto rut = generate_grid3d(input("single_rut.toml"));
    bt::require(std::abs(rut.sample(8.,-.5).height_m + .04) <= 1e-9,"rut must be negative with authored depth");
    const auto bank = generate_grid3d(input("constant_bank.toml"));
    for (double x : {0.,2.,4.,15.,18.,20.})
        bt::require(bank.sample(x,.7).height_m == bank.sample(x,-.7).height_m,"lead zones must be flat laterally");
    bt::require(std::abs(bank.sample(8.,.7).height_m - bank.sample(8.,-.7).height_m -
        1.4 * std::tan(10. * std::numbers::pi / 180.)) <= 1e-12,"bank antisymmetry law");
    const auto mirrored = generate_grid3d(input("mirrored.toml"));
    for (std::uint64_t iy = 0; iy < rock.ny(); ++iy) for (std::uint64_t ix = 0; ix < rock.nx(); ++ix)
        bt::require(std::bit_cast<std::uint64_t>(rock.elevation()[iy * rock.nx() + ix]) ==
            std::bit_cast<std::uint64_t>(mirrored.elevation()[(rock.ny() - 1 - iy) * rock.nx() + ix]),"mirrored authored feature must mirror node bits");
    auto zoned = rock_spec;
    zoned.zones = {{7,6.,10.,-1.,1.},{1,8.,9.,-.5,.5}};
    const auto zones = generate_grid3d(zoned);
    bt::require(zones.sample(7.,0.).surface_id == 7 && zones.sample(8.,0.).surface_id == 1,"material zones are last-wins and half-open");
    auto noisy = rock_spec;
    noisy.roughness = {.003,.1}; noisy.seed_decimal = "184467440737095516170";
    const auto noise = generate_grid3d(noisy);
    bt::require(noise.sha256_hex() == generate_grid3d(noisy).sha256_hex(),"arbitrary decimal seed has a deterministic independent roughness stream");
    for (const double height : noise.elevation()) bt::require(std::isfinite(height),"generated elevation must be finite");
    bt::require(bike::io::canonical_json(write_track3d_spec(read_track3d_spec(write_track3d_spec(noisy)))) ==
        bike::io::canonical_json(write_track3d_spec(noisy)),"resolved spatial spec roundtrip");
    auto bad = rock_spec; bad.lead_in_m = 19.; bt::invalid([&] { (void)generate_grid3d(bad); });
    bad = rock_spec; bad.banks = {{4.,1.,15.0001}}; bt::invalid([&] { (void)generate_grid3d(bad); });
    bad = rock_spec; bad.roughness.ell_m = 0.; bt::invalid([&] { (void)generate_grid3d(bad); });
    bad = rock_spec; bad.features[0].kind = "unknown"; bt::invalid([&] { (void)generate_grid3d(bad); });
    bad = rock_spec; bad.features[0].amplitude_m = .151; bt::invalid([&] { (void)generate_grid3d(bad); });
    bad = rock_spec; bad.dx_m = 1e-300; bt::invalid([&] { (void)generate_grid3d(bad); });
    // Nodal differences alone miss the Catmull overshoot: 80 mm / 50 mm
    // gives a nodal slope of 1.6, but the interior cubic gradient reaches 2.
    auto edge_spec = rock_spec;
    edge_spec.features.clear();
    auto edge_track = flat(20.);
    edge_track.obstacles.push_back({.shape = SquareEdge{8., .08, .25}});
    edge_spec.base_track_spec = track_to_dict(edge_track);
    bt::invalid([&] { (void)generate_grid3d(edge_spec); });
    // This smaller edge has an actual maximum slope of 1.5. Its initial
    // Bernstein bound is looser, so subdivision must accept the valid surface.
    edge_track.obstacles.front().shape = SquareEdge{8., .06, .25};
    edge_spec.base_track_spec = track_to_dict(edge_track);
    const auto bounded_edge = generate_grid3d(edge_spec);
    bt::require(bounded_edge.sample(7.975, 0.).status == QueryStatus3d::ok,
        "bounded analytic slope validation retains a valid sharp surface");
    auto document = write_track3d_spec(rock_spec);
    auto& root = std::get<bike::io::Value::Object>(document.data);
    for (auto& [key,value] : root) if (key == "terrain3d")
        std::get<bike::io::Value::Object>(value.data).emplace_back("unexpected",bike::io::Value{true});
    bt::invalid([&] { (void)read_track3d_spec(document); });
    // Physical assertions above execute before the fail-closed V02 capture gate.
    pending_hash_gate("single_rock.toml",rock);
    pending_hash_gate("single_rut.toml",rut);
    pending_hash_gate("constant_bank.toml",bank);
    pending_hash_gate("mirrored.toml",mirrored);
}); }
