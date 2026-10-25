// test_terrain_placement_regression.cpp — C++ port of
// tests/reference/test_terrain_placement_regression.py (batch 3.13).
//
// The procedural generators stay byte-identical across the placement
// refactor: the default TerrainGenSpec sha256 and the sha256 of each
// seeded generated track / road preset dump are pinned constants.
#include "tyre_reference_support.hpp"

#include "io/hash.hpp"
#include "terrain/generators.hpp"
#include "terrain/trackfile.hpp"

#include <map>
#include <string>

namespace {

namespace terrain = bike::terrain;

[[nodiscard]] std::string digest(const terrain::TrackSpec &track) {
    const auto dump = terrain::dump_track(track);
    return bike::io::sha256(std::as_bytes(std::span{dump}));
}

// tests/reference/test_terrain_placement_regression.py::test_default_terrain_spec_hash_is_pinned
void test_default_terrain_spec_hash_is_pinned() {
    bt::require(
            terrain::TerrainGenSpec{}.sha256() ==
                    "45d11d386fb651dff55d7af3292ea3e2a972d9354e1bef677227f2399bfcf8e7",
            "default TerrainGenSpec sha256 pinned");
}

// tests/reference/test_terrain_placement_regression.py::test_generated_tracks_are_byte_identical[seed]
void test_generated_tracks_are_byte_identical() {
    const std::map<std::uint64_t, std::string> want{
            {0, "b298d8170a8391daf50b671f41b9dc7e33855354fe770d3cb894af70400f544d"},
            {1, "7ac544f21b89f27a03a176e5d017e0a0e175af8ec80119601dd05a652133848a"},
            {2, "4837293048585ed234201595833d3e8ed7851b6c6e8a384ab41745840c7f5622"},
            {3, "be8c67f11a7961b2bcb012612b6d16ed0b72394edcae09f9b12fc0e1b73857c0"}};
    for (const auto &[seed, expected] : want)
        bt::require(digest(terrain::generate_track(
                            terrain::TerrainGenSpec{}, seed)) == expected,
                    "generated track seed " + std::to_string(seed) +
                            " byte-identical");
}

// tests/reference/test_terrain_placement_regression.py::test_road_presets_are_byte_identical[name]
void test_road_presets_are_byte_identical() {
    const std::map<std::string, std::string> want{
            {"road_broken",
             "f6eb8883c2f83196328a06cb7eaf2d7eb1736f9b5de49206dc25dffcc1aa029e"},
            {"road_smooth",
             "d05d8a6b82f179bdb80f66ee5cfa71bdb8c74237135456f673e74dfc74eafb1d"},
            {"road_worn",
             "455af2ec2c2665629898e2c05b7ae29cf0b2f766a4cf2d9a1e0e77fd0017288c"}};
    for (const auto &[name, expected] : want)
        bt::require(digest(terrain::get_preset(name)) == expected,
                    "road preset " + name + " byte-identical");
}

} // namespace

int main() {
    return bt::run([] {
        test_default_terrain_spec_hash_is_pinned();
        test_generated_tracks_are_byte_identical();
        test_road_presets_are_byte_identical();
    });
}
