// test_savage_track_surface.cpp — C++ port of
// tests/reference/test_savage_track_surface.py (batch 3.13).
//
// The frozen savage climb file keeps asphalt end to end so the scenario
// isolates climb power and geometry: base surface 'asphalt', no surface
// sections, 120 m.
#include "tyre_reference_support.hpp"

#include "terrain/trackfile.hpp"

namespace {

// tests/reference/test_savage_track_surface.py::test_savage_is_asphalt_everywhere
void test_savage_is_asphalt_everywhere() {
    const auto track = bike::terrain::load_track(tyre_reference::repo_file(
            "examples/research/rough_uphill_savage.toml"));
    bt::require(track.surface == "asphalt",
                "savage base surface stays asphalt");
    bt::require(track.surface_sections.empty(),
                "savage carries no surface sections");
    bt::require(track.length_m == 120.0, "savage length stays 120 m");
}

} // namespace

int main() { return bt::run(test_savage_is_asphalt_everywhere); }
