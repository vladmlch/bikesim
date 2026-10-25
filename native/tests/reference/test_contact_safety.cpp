// Completion 3.14b (T52). Source: tests/reference/test_contact_safety.py.
// The closed-form single-contact guards — bump-length floor, bowl-depth
// ceiling, the concave-facet span window and the hazard report — replayed
// bitwise against the platform goldens. The StubGarden nodes exercise the
// raw-facet concave_span_deg(x, z, window) overload added as this batch's
// seam; the RockGarden/RoadRoughness nodes go through nodes().

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "diag.hpp"
#include "support/golden.hpp"
#include "support/migration_test.hpp"
#include "terrain/profile.hpp"
#include "terrain/types.hpp"

namespace {
using namespace bike::terrain;

constexpr std::string_view kFile =
    "tests/reference/test_contact_safety.py::";

void scalar(bt::oracle::Node &golden, double actual, std::string_view label) {
    const std::array<double, 1> value{actual};
    golden.f64(value, std::span<const std::size_t>{}, label);
}

RockGarden garden(double amp, double corr, int seed, double length = 8.,
                  double start = 0.) {
    return {.start_m = start, .section_length_m = length, .amplitude_m = amp,
            .correlation_length_m = corr, .seed = std::to_string(seed)};
}

void test_bump_length_floor_matches_spike() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const double floor_10 = min_bump_length_m(0.10);
    const double floor_05 = min_bump_length_m(0.05);
    const double floor_15 = min_bump_length_m(0.15);
    scalar(golden, floor_10, "bump floor 0.10");
    scalar(golden, floor_05, "bump floor 0.05");
    scalar(golden, floor_15, "bump floor 0.15");
    bt::require(std::abs(floor_10 - 0.9425) <= 1e-4, "floor_10 approx 0.9425");
    bt::require(floor_05 < 0.67, "floor_05 < 0.67");
    bt::require(floor_15 < 1.16, "floor_15 < 1.16");
    golden.done();
}

void test_bowl_depth_ceiling_matches_spike() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const double depth_08 = max_bowl_depth_m(0.8);
    const double depth_12 = max_bowl_depth_m(1.2);
    const double depth_15 = max_bowl_depth_m(1.5);
    scalar(golden, depth_08, "bowl depth 0.8");
    scalar(golden, depth_12, "bowl depth 1.2");
    scalar(golden, depth_15, "bowl depth 1.5");
    bt::require(std::abs(depth_08 - 0.07205) <= 1e-5, "depth_08 approx");
    bt::require(depth_08 >= 0.07 && depth_08 < 0.10, "depth_08 bounds");
    bt::require(depth_12 >= 0.15, "depth_12 bound");
    bt::require(depth_15 >= 0.20, "depth_15 bound");
    golden.done();
}

// scipy's gaussian_filter1d correlate loop is a platform oracle (~1 ulp
// between lanes): the source records the concrete (s, z) facets as
// input.* pairs before each span, so this replay feeds the recorded
// platform-faithful nodes through the raw-facet overload.
double garden_span(bt::oracle::Node &golden, const std::string &label) {
    const std::vector<double> s = golden.f64_input("input." + label + ".s");
    const std::vector<double> z = golden.f64_input("input." + label + ".z");
    return concave_span_deg(s, z);
}

void test_concave_span_flags_the_dirty_spike_seed_and_passes_clean_ones() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    // The source's comprehensions record every input pair before a single
    // span output lands: consume all 20 (s, z) pairs first.
    const double dirty = garden_span(golden, "dirty");
    const double clean = garden_span(golden, "clean");
    const std::array<const char *, 3> names{"0.03", "0.05", "0.10"};
    std::array<std::vector<double>, 3> spans;
    for (std::size_t c = 0; c < names.size(); ++c)
        for (int s = 0; s < 6; ++s)
            spans[c].push_back(
                garden_span(golden, std::string(names[c]) + " s" +
                                        std::to_string(s)));
    scalar(golden, dirty, "span dirty");
    scalar(golden, clean, "span clean");
    for (std::size_t c = 0; c < names.size(); ++c)
        for (int s = 0; s < 6; ++s)
            scalar(golden, spans[c][static_cast<std::size_t>(s)],
                   std::string("span ") + names[c] + " s" +
                       std::to_string(s));
    const auto max_of = [](const std::vector<double> &v) {
        return *std::ranges::max_element(v);
    };
    bt::require(dirty >= 20., "dirty flags");
    bt::require(clean < 20., "clean passes");
    bt::require(max_of(spans[0]) < 20., "0.03 sweep clean");
    bt::require(max_of(spans[1]) < 20., "0.05 sweep clean");
    bt::require(max_of(spans[2]) >= 20., "0.10 sweep flags");
    golden.done();
}

void test_concave_span_ignores_convex_crests() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const double span = garden_span(golden, "crest");
    scalar(golden, span, "span crest");
    bt::require(span < 20., "crest stays clean");
    golden.done();
}

// StubGarden(S, z): the raw-facet (x, z) overload.
const std::vector<double> kS{0.0, 0.2, 0.6, 1.4, 1.8, 2.4, 3.0};
const std::vector<double> kSExit{0.0, 0.6, 1.4, 2.0, 2.6, 3.0};

double deg_of_atan(double slope) { return std::atan(slope) * 180. / std::numbers::pi; }

void test_concave_span_counts_a_steep_entry_join() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const std::vector<double> z{0.0, 0.08, 0.10, 0.10, 0.10, 0.05, 0.0};
    const double span = concave_span_deg(kS, z);
    scalar(golden, span, "span entry concave");
    bt::require(std::abs(span - deg_of_atan(0.4)) <= 1e-9,
                "entry join is atan(0.4)");
    bt::require(span >= 20., "entry join flags");
    golden.done();
}

void test_concave_span_ignores_a_convex_entry_join() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const std::vector<double> z{0.0, -0.08, -0.10, -0.10, -0.10, -0.05, 0.0};
    const double span = concave_span_deg(kS, z);
    scalar(golden, span, "span entry convex");
    bt::require(std::abs(span) <= 1e-9, "convex entry stays on the floor");
    golden.done();
}

void test_concave_span_control_gentle_facets_stay_clean() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const std::vector<double> z{0.0, 0.01, 0.02, 0.02, 0.02, 0.01, 0.0};
    const double span = concave_span_deg(kS, z);
    scalar(golden, span, "span gentle");
    bt::require(span < 17., "gentle facets stay clean");
    golden.done();
}

void test_concave_span_counts_a_steep_exit_join_and_ignores_a_convex_one() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const std::vector<double> z_c{0.0, 0.02, 0.16, 0.16, 0.16, 0.0};
    const double concave = concave_span_deg(kSExit, z_c);
    scalar(golden, concave, "span exit concave");
    bt::require(std::abs(concave - deg_of_atan(0.4)) <= 1e-9,
                "exit join is atan(0.4)");
    const std::vector<double> z_v{0.0, -0.02, -0.16, -0.16, -0.16, 0.0};
    const double convex = concave_span_deg(kSExit, z_v);
    scalar(golden, convex, "span exit convex");
    bt::require(std::abs(convex) <= 1e-9, "convex exit never flags");
    golden.done();
}

void hazard_bytes(bt::oracle::Node &golden, const std::string &text,
                  std::size_t index) {
    const std::span<const std::byte> bytes =
        std::as_bytes(std::span{text});
    const std::array<std::size_t, 1> shape{text.size()};
    golden.bytes(bytes, "|u1", shape,
                 "hazard " + std::to_string(index));
}

void test_hazards_report_every_unsafe_obstacle_kind() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    TrackSpec track;
    track.name = "t";
    track.length_m = 60.;
    track.obstacles = {
        {.shape = Bump{.start_m = 5., .height_m = .10, .bump_length_m = .66}},
        {.shape = Bump{.start_m = 8., .height_m = .10, .bump_length_m = .95}},
        {.shape = BowlPothole{.start_m = 12., .depth_m = .10, .hole_length_m = .8}},
        {.shape = BowlPothole{.start_m = 16., .depth_m = .07, .hole_length_m = .8}},
        {.shape = SlopedPothole{.start_m = 20., .depth_m = .05, .hole_length_m = .8}},
        {.shape = SquareEdge{.start_m = 24., .height_m = .05, .ledge_length_m = .3}},
        {.shape = garden(0.07, 0.45, 5, 8., 30.)},
        {.shape = RoadRoughness{.start_m = 40., .section_length_m = 8.,
                                .amplitude_m = .01, .correlation_length_m = .10,
                                .seed = "0"}},
    };
    const std::vector<std::string> hazards = single_contact_hazards(track);
    for (std::size_t i = 0; i < hazards.size(); ++i)
        hazard_bytes(golden, hazards[i], i);
    std::vector<std::string> labels;
    for (const std::string &h : hazards)
        labels.push_back(h.substr(0, h.find(':')));
    const std::vector<std::string> expected{
        "Bump@5m", "BowlPothole@12m", "SlopedPothole@20m", "SquareEdge@24m",
        "RockGarden@30m"};
    bt::require(labels == expected, "hazard labels and order");
    golden.done();
}

void test_clean_track_has_no_hazards() {
    TrackSpec track;
    track.name = "t";
    track.length_m = 30.;
    track.obstacles = {
        {.shape = Bump{.start_m = 5., .height_m = .05, .bump_length_m = .7}},
        {.shape = garden(0.03, 0.25, 2, 8., 10.)},
    };
    bt::require(single_contact_hazards(track).empty(), "clean track reports");
}
} // namespace

// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        test_bump_length_floor_matches_spike();
        test_bowl_depth_ceiling_matches_spike();
        test_concave_span_flags_the_dirty_spike_seed_and_passes_clean_ones();
        test_concave_span_ignores_convex_crests();
        test_concave_span_counts_a_steep_entry_join();
        test_concave_span_ignores_a_convex_entry_join();
        test_concave_span_control_gentle_facets_stay_clean();
        test_concave_span_counts_a_steep_exit_join_and_ignores_a_convex_one();
        test_hazards_report_every_unsafe_obstacle_kind();
        test_clean_track_has_no_hazards();
    });
}
