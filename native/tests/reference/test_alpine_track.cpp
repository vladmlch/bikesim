// test_alpine_track.cpp — C++ port of tests/reference/test_alpine_track.py
// (batch 3.13).
//
// Alpine scree climb: generator, preset, frozen files and CLI routing.
// Every record is a port — the source pins deterministic structure,
// frozen bytes and rejection contracts against literal values, not
// oracle goldens.
#include "tyre_reference_support.hpp"

#include "cli/application.hpp"
#include "configuration/load.hpp"
#include "io/json.hpp"
#include "configuration/physical_config.hpp"
#include "random/numpy_compat.hpp"
#include "research/environment.hpp"
#include "terrain/detail.hpp"
#include "terrain/generators.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <sstream>
#include <unistd.h>

namespace {

using tyre_reference::rejects;
using tyre_reference::repo_file;
namespace cfg = bike::configuration;
namespace terrain = bike::terrain;
using terrain::AlpineGenSpec;

constexpr double kFrontRadiusM = .372; // wheelpath.FRONT_WHEEL_RADIUS_M
constexpr double kRearRadiusM = .352;  // wheelpath.REAR_WHEEL_RADIUS_M

[[nodiscard]] std::string read_text(const std::filesystem::path &path) {
    std::ifstream in(path);
    bt::require(in.good(), "frozen file readable: " + path.string());
    return {std::istreambuf_iterator<char>(in),
            std::istreambuf_iterator<char>()};
}

[[nodiscard]] bike::io::Value toml(std::string_view source) {
    return cfg::detail::parse_toml(source, "mem.toml");
}

[[nodiscard]] bike::io::Value object(
        std::initializer_list<
                std::pair<std::string_view, bike::io::Value>>
                members) {
    bike::io::Value::Object out;
    for (const auto &[key, value] : members)
        out.emplace_back(std::string(key), value);
    return bike::io::Value{std::move(out)};
}

[[nodiscard]] bike::io::Value number(std::string_view spelling) {
    return bike::io::Value{bike::io::Number{std::string(spelling)}};
}

[[nodiscard]] bike::io::Value list(std::initializer_list<bike::io::Value> v) {
    return bike::io::Value{bike::io::Value::Array{v}};
}

[[nodiscard]] bike::io::Value str(std::string_view v) {
    return bike::io::Value{std::string(v)};
}

// ---------------------------------------------------------------------
// Surface + spec contract
// ---------------------------------------------------------------------

// tests/reference/test_alpine_track.py::test_scree_surface_is_registered_below_loose
[[gnu::noinline]] void test_scree_surface_is_registered_below_loose() {
    const auto &scree = terrain::get_surface("scree");
    const auto &loose = terrain::get_surface("loose");
    const auto names = terrain::available_surfaces();
    bt::require(std::ranges::find(names, "scree") != names.end(),
                "'scree' in SURFACES");
    bt::require(scree.mu_slide <= scree.mu_peak, "mu_slide <= mu_peak");
    bt::require(scree.mu_peak < loose.mu_peak &&
                        scree.mu_slide < loose.mu_slide &&
                        scree.slip_stiffness_per_load <
                                loose.slip_stiffness_per_load &&
                        scree.stribeck_speed_mps == loose.stribeck_speed_mps,
                "scree sits below loose");
}

// tests/reference/test_alpine_track.py::test_default_spec_matches_the_design_numbers
[[gnu::noinline]] void test_default_spec_matches_the_design_numbers() {
    const AlpineGenSpec s;
    bt::require(s.length_m == 120. && s.lead_in_m == 10. &&
                        s.lead_out_m == 15. && s.transition_m == 2.,
                "length/lead/transition design numbers");
    bt::require(s.ramp_grade == terrain::Range{.15, .25} &&
                        s.wall_grade == terrain::Range{.26, .32} &&
                        s.n_walls == 1,
                "ramp/wall design numbers");
    bt::require(s.counter_grade == terrain::Range{-.08, -.03} &&
                        s.counter_prob == .3,
                "counter-slope design numbers");
    bt::require(s.base_surface == "hardpack" &&
                        s.n_scree == terrain::CountRange{2, 3} &&
                        s.scree_ramp_bias == .7,
                "scree design numbers");
    bt::require(s.n_sharp_edges == terrain::CountRange{0, 0} &&
                        s.single_contact_safe,
                "single-contact safe default");
    bt::require(s.min_contact_radius_m == .45 && s.max_kink_deg == 17. &&
                        s.contact_window_m == .25,
                "contact-safety design numbers");
    bt::require(std::is_same_v<decltype(terrain::ALPINE_PRESET_SEED),
                               const std::int64_t>,
                "ALPINE_PRESET_SEED is an integer");
}

// tests/reference/test_alpine_track.py::test_spec_round_trips_through_dict_and_toml_lists
[[gnu::noinline]] void test_spec_round_trips_through_dict_and_toml_lists() {
    const auto spec = AlpineGenSpec::from_dict(object(
            {{"ramp_grade", list({number("0.10"), number("0.20")})},
             {"n_boulders", list({number("1"), number("2")})}}));
    bt::require(spec.ramp_grade == terrain::Range{.10, .20} &&
                        spec.n_boulders == terrain::CountRange{1, 2},
                "lists coerce into ranges");
    const auto again = AlpineGenSpec::from_dict(
            toml("ramp_grade = [0.10, 0.20]\nn_boulders = [1, 2]\n"));
    bt::require(bike::io::canonical_json(again.to_dict()) ==
                        bike::io::canonical_json(spec.to_dict()) &&
                        again.sha256() == spec.sha256(),
                "TOML list spelling equals the dict construction");
    bt::require(bike::io::canonical_json(
                        AlpineGenSpec::from_dict(spec.to_dict())
                                .to_dict()) ==
                        bike::io::canonical_json(spec.to_dict()),
                "spec round-trips through its dict");
    bt::require(AlpineGenSpec{}.sha256() != spec.sha256(),
                "the edited spec hashes differently");
}

// tests/reference/test_alpine_track.py::test_spec_hash_is_stable_across_numeric_input_types
[[gnu::noinline]] void test_spec_hash_is_stable_across_numeric_input_types() {
    const auto plain = AlpineGenSpec{}.sha256();
    bt::require(AlpineGenSpec::from_dict(object({{"length_m", number("120")}}))
                        .sha256() ==
                        AlpineGenSpec::from_dict(
                                object({{"length_m", number("120.0")}}))
                                .sha256(),
                "int and float spellings hash alike");
    const auto mixed = AlpineGenSpec::from_dict(
            object({{"length_m", number("120")},
                    {"counter_prob", number("0.3")}}));
    bt::require(mixed.length_m == 120. && mixed.counter_prob == .3 &&
                        mixed.sha256() == plain,
                "mixed numeric types equal the plain default hash");
    // np.float32(.3) widens to a different double; it must still hash.
    AlpineGenSpec widened;
    widened.counter_prob = static_cast<double>(.3F);
    (void)widened.sha256();
}

// tests/reference/test_alpine_track.py::test_spec_rejects_invalid_values — 11 rows
[[gnu::noinline]] void test_spec_rejects_invalid_values() {
    const std::array<bike::io::Value, 11> bad{
            object({{"unknown_key", number("1")}}),
            object({{"ramp_grade", list({number("0.25"), number("0.15")})}}),
            object({{"counter_grade",
                     list({number("-0.05"), number("0.01")})}}),
            object({{"n_boulders", list({number("1.5"), number("2")})}}),
            object({{"counter_prob", number("1.5")}}),
            object({{"n_walls", number("2")}}),
            object({{"wall_grade", list({number("0.20"), number("0.30")})}}),
            object({{"base_surface", str("ice")}}),
            object({{"length_m", number("30.0")}}),
            object({{"n_sharp_edges", list({number("1"), number("1")})}}),
            object({{"single_contact_safe", str("yes")}})};
    for (const auto &doc : bad)
        rejects<std::invalid_argument>(
                [&] { (void)AlpineGenSpec::from_dict(doc); });
}

// tests/reference/test_alpine_track.py::test_sharp_edges_need_the_multi_contact_opt_in
[[gnu::noinline]] void test_sharp_edges_need_the_multi_contact_opt_in() {
    auto spec = AlpineGenSpec{};
    spec.single_contact_safe = false;
    spec.n_sharp_edges = {1, 2};
    spec.validate();
    bt::require(spec.n_sharp_edges == terrain::CountRange{1, 2},
                "opted-in sharp edges survive validation");
}

// tests/reference/test_alpine_track.py::test_spec_rejects_lengths_below_the_grid — 14 rows
[[gnu::noinline]] void test_spec_rejects_lengths_below_the_grid() {
    const auto reject = [](auto &&edit) {
        rejects<std::invalid_argument>([&] {
            auto spec = AlpineGenSpec{};
            edit(spec);
            spec.validate();
        });
    };
    reject([](auto &s) {
        s.length_m = 44.014;
        s.transition_m = 2.007;
    });
    reject([](auto &s) { s.transition_m = 0.001; });
    reject([](auto &s) { s.transition_m = 0.009; });
    reject([](auto &s) { s.ramp_len_m = {0., 0.}; });
    reject([](auto &s) { s.shelf_len_m = {0., 0.}; });
    reject([](auto &s) { s.counter_len_m = {0., 0.}; });
    reject([](auto &s) {
        s.scree_len_m = {0., 0.};
        s.n_scree = {0, 0};
    });
    reject([](auto &s) {
        s.wet_len_m = {0., 0.};
        s.n_wet = {0, 0};
    });
    reject([](auto &s) {
        s.boulder_len_m = {0., 0.};
        s.n_boulders = {0, 0};
    });
    reject([](auto &s) {
        s.rock_len_m = {0., 0.};
        s.n_rock_gardens = {0, 0};
    });
    reject([](auto &s) {
        s.water_bar_len_m = {0., 0.};
        s.n_water_bars = {0, 0};
    });
    reject([](auto &s) { s.scree_corr_m = {0., 0.}; });
    reject([](auto &s) { s.background_corr_m = {0., 0.}; });
    reject([](auto &s) { s.rock_corr_m = {0., 0.}; });
}

// tests/reference/test_alpine_track.py::test_spec_accepts_the_grid_boundary
[[gnu::noinline]] void test_spec_accepts_the_grid_boundary() {
    auto spec = AlpineGenSpec{};
    spec.transition_m = 0.01;
    spec.validate();
    bt::require(std::abs(spec.transition_m - 0.01) < 1e-15,
                "the centimetre boundary validates");
    auto ramp = AlpineGenSpec{};
    ramp.ramp_len_m = {0.01, 15.};
    ramp.validate();
    bt::require(ramp.ramp_len_m == terrain::Range{0.01, 15.},
                "ramp_len boundary validates");
}

// ---------------------------------------------------------------------
// Rhythm internals — the same private surface the oracle imports as
// _segments/_rhythm.
// ---------------------------------------------------------------------

[[nodiscard]] bike::random::NumpyRng rng(std::uint64_t seed) {
    return bike::random::NumpyRng{std::to_string(seed), {}};
}

// tests/reference/test_alpine_track.py::test_rhythm_fills_the_body_with_ramp_shelf_cycles[0..7]
[[gnu::noinline]] void test_rhythm_fills_the_body_with_ramp_shelf_cycles() {
    for (std::uint64_t seed = 0; seed < 8; ++seed) {
        const AlpineGenSpec spec;
        auto gen = rng(seed);
        const auto segments = terrain::detail::segments(spec, gen);
        const double body_end = spec.length_m - spec.lead_out_m;
        const double t = spec.transition_m;
        bt::require(!segments.empty(), "segments exist");
        bt::require(segments[0].kind == "ramp" ||
                            segments[0].kind == "wall",
                    "first body segment climbs");
        std::size_t walls = 0;
        for (const auto &seg : segments) {
            walls += seg.kind == "wall";
            bt::require(seg.kind == "ramp" || seg.kind == "wall" ||
                                seg.kind == "shelf" ||
                                seg.kind == "counter",
                        "segment kinds stay in the body vocabulary");
        }
        bt::require(walls == 1, "exactly one wall");
        for (std::size_t i = 1; i < segments.size(); ++i) {
            bt::require(segments[i - 1].start < segments[i - 1].end,
                        "segments advance");
            bt::require(std::abs(segments[i].start -
                                         (segments[i - 1].end + t)) <= 1e-9,
                        "transition gap between segments");
        }
        bt::require(std::abs(segments[0].start -
                                     (spec.lead_in_m + t)) <= 1e-9,
                    "body begins after lead-in plus a transition");
        bt::require(segments.back().end <= body_end - t + 1e-9,
                    "body ends before lead-out minus a transition");
        bt::require(body_end - t - segments.back().end <
                            spec.ramp_len_m[0] + t,
                    "no room left for another ramp");
        for (const auto &seg : segments) {
            const terrain::Range bounds =
                    seg.kind == "ramp"   ? spec.ramp_grade
                    : seg.kind == "wall" ? spec.wall_grade
                    : seg.kind == "shelf"
                            ? spec.shelf_grade
                            : spec.counter_grade;
            bt::require(bounds[0] <= seg.grade && seg.grade <= bounds[1],
                        "grade inside the kind's range");
            if (seg.kind == "ramp")
                bt::require(spec.ramp_len_m[0] <= seg.end - seg.start &&
                                    seg.end - seg.start <=
                                            spec.ramp_len_m[1] + 1e-9,
                            "ramp length inside its range");
        }
    }
}

// tests/reference/test_alpine_track.py::test_rhythm_grade_profile_is_flat_outside_the_body[0..7]
[[gnu::noinline]] void test_rhythm_grade_profile_is_flat_outside_the_body() {
    for (std::uint64_t seed = 0; seed < 8; ++seed) {
        const AlpineGenSpec spec;
        auto gen = rng(seed);
        const auto [grade, ramps] = terrain::detail::rhythm(spec, gen);
        const auto x = bike::geometry::detail::np_linspace(0., spec.length_m,
                                                         2401);
        const auto slope = grade.slope(x);
        for (std::size_t i = 0; i < x.size(); ++i) {
            if (x[i] <= spec.lead_in_m)
                bt::require(slope[i] == 0., "flat inside lead-in");
            if (x[i] >= spec.length_m - spec.lead_out_m)
                bt::require(slope[i] == 0., "flat inside lead-out");
        }
        bt::require(*std::ranges::max_element(slope) <=
                            spec.wall_grade[1] + 1e-12,
                    "slope stays under the wall ceiling");
        bt::require(*std::ranges::min_element(slope) >=
                            spec.counter_grade[0] - 1e-12,
                    "slope stays above the counter floor");
        bt::require(!ramps.empty(), "ramps exist");
        for (const auto &[s, e] : ramps)
            bt::require(spec.lead_in_m < s && s < e &&
                                e < spec.length_m - spec.lead_out_m,
                        "ramps sit inside the body");
        bt::require(grade.elevation(spec.length_m) > 8.,
                    "a climb, not a flat");
    }
}

// tests/reference/test_alpine_track.py::test_rhythm_without_walls_or_counter_slopes
[[gnu::noinline]] void test_rhythm_without_walls_or_counter_slopes() {
    auto spec = AlpineGenSpec{};
    spec.n_walls = 0;
    spec.counter_prob = 0.;
    auto gen = rng(3);
    const auto segments = terrain::detail::segments(spec, gen);
    for (const auto &seg : segments)
        bt::require(seg.kind == "ramp" || seg.kind == "shelf",
                    "without walls/counters only ramps and shelves remain");
}

// ---------------------------------------------------------------------
// Generation + structure
// ---------------------------------------------------------------------

// tests/reference/test_alpine_track.py::test_generate_rejects_bad_arguments
[[gnu::noinline]] void test_generate_rejects_bad_arguments() {
    // generate_alpine_track(object(), seed=0) — a non-spec argument is a
    // compile-time impossibility natively; the ValueError seed domain is
    // what the port pins.
    rejects<std::invalid_argument>([&] {
        (void)terrain::generate_alpine_track(AlpineGenSpec{}, "-1");
    });
    rejects<std::invalid_argument>([&] {
        (void)terrain::generate_alpine_track(AlpineGenSpec{}, "1.0");
    });
}

// tests/reference/test_alpine_track.py::test_same_seed_same_bytes_and_toml_round_trip[0..7]
[[gnu::noinline]] void test_same_seed_same_bytes_and_toml_round_trip() {
    for (std::uint64_t seed = 0; seed < 8; ++seed) {
        const AlpineGenSpec spec;
        const auto first = terrain::dump_track(
                terrain::generate_alpine_track(spec, seed));
        const auto second = terrain::dump_track(
                terrain::generate_alpine_track(spec, seed));
        bt::require(first == second, "same seed, same bytes");
        const auto doc = toml(first);
        const auto round_trip = terrain::track_to_dict(
                terrain::track_from_dict(doc));
        const auto direct = terrain::track_to_dict(
                terrain::generate_alpine_track(spec, seed));
        bt::require(bike::io::canonical_json(round_trip) ==
                            bike::io::canonical_json(direct),
                    "TOML round trip equals the regenerated track");
    }
}

// tests/reference/test_alpine_track.py::test_different_seeds_differ
[[gnu::noinline]] void test_different_seeds_differ() {
    std::vector<std::string> dumps;
    for (std::uint64_t seed = 0; seed < 8; ++seed)
        dumps.push_back(terrain::dump_track(terrain::generate_alpine_track(
                AlpineGenSpec{}, seed)));
    std::ranges::sort(dumps);
    bt::require(std::ranges::adjacent_find(dumps) == dumps.end() &&
                        dumps.size() == 8,
                "eight distinct seeds give eight distinct dumps");
}

[[nodiscard]] std::pair<double, double> body(const AlpineGenSpec &spec) {
    return {spec.lead_in_m, spec.length_m - spec.lead_out_m};
}

// tests/reference/test_alpine_track.py::test_default_track_structure[0..7]
[[gnu::noinline]] void test_default_track_structure() {
    for (std::uint64_t seed = 0; seed < 8; ++seed) {
        const AlpineGenSpec spec;
        const auto track = terrain::generate_alpine_track(spec, seed);
        track.validate();
        const auto [lo, hi] = body(spec);
        bt::require(track.length_m == 120. && track.surface == "hardpack" &&
                            track.name ==
                                    "alpine_" + std::to_string(seed),
                    "track header");
        bt::require(track.description.find("seed=" +
                                           std::to_string(seed)) !=
                                    std::string::npos &&
                            track.description.find(spec.sha256()) !=
                                    std::string::npos,
                    "description carries seed and spec hash");
        for (const auto &o : track.obstacles)
            bt::require(lo <= o.start_m() && o.end_m() <= hi + 1e-9,
                        "obstacles inside the body");
        for (const auto &s : track.surface_sections)
            bt::require(lo <= s.start_m && s.end_m <= hi + 1e-9,
                        "sections inside the body");
        const std::array<double, 6> x{0., 5., 10., 105., 112., 120.};
        for (const double slope :
             track.grade_profile->slope(std::span<const double>(x)))
            bt::require(slope == 0., "grade flat outside the body");

        std::size_t scree = 0;
        for (const auto &section : track.surface_sections) {
            if (section.surface != "scree") continue;
            ++scree;
            std::size_t matches = 0;
            double amplitude = 0.;
            for (const auto &o : track.obstacles) {
                const auto *rough =
                        std::get_if<terrain::RoadRoughness>(&o.shape);
                if (!rough) continue;
                if (std::abs(rough->start_m - section.start_m) < 1e-6 &&
                    std::abs(rough->start_m + rough->section_length_m -
                             section.end_m) < 1e-6) {
                    ++matches;
                    amplitude = rough->amplitude_m;
                }
            }
            bt::require(matches == 1, "one roughness raster per scree span");
            bt::require(spec.scree_amp_m[0] * std::pow(.8, 6) <= amplitude &&
                                amplitude <= spec.scree_amp_m[1],
                        "scree amplitude inside the redraw floor");
        }
        bt::require(scree >= 2, "at least two scree sections");
        std::size_t gardens = 0, boulders = 0, bowls = 0;
        bool square_edge = false;
        for (const auto &o : track.obstacles) {
            if (std::holds_alternative<terrain::SquareEdge>(o.shape))
                square_edge = true;
            if (std::holds_alternative<terrain::RockGarden>(o.shape))
                ++gardens;
            if (const auto *b = std::get_if<terrain::Bump>(&o.shape)) {
                ++boulders;
                bt::require(spec.boulder_height_m[0] <= b->height_m &&
                                    b->height_m <= spec.boulder_height_m[1],
                            "boulder height inside its range");
            }
            if (const auto *p =
                        std::get_if<terrain::BowlPothole>(&o.shape)) {
                ++bowls;
                bt::require(p->depth_m <= spec.water_bar_depth_m[1],
                            "water-bar depth inside its range");
            }
        }
        bt::require(1 <= gardens &&
                            gardens <= static_cast<std::size_t>(
                                    spec.n_rock_gardens[1]),
                    "rock-garden count inside its range");
        bt::require(1 <= boulders &&
                            boulders <= static_cast<std::size_t>(
                                    spec.n_boulders[1]),
                    "boulder count inside its range");
        bt::require(1 <= bowls &&
                            bowls <= static_cast<std::size_t>(
                                    spec.n_water_bars[1]),
                    "water-bar count inside its range");
        bt::require(!square_edge, "no sharp edges by default");
        bt::require(terrain::build_field_data(track).size() ==
                            static_cast<std::size_t>(
                                    terrain::HeightFieldSpec::for_track(track)
                                            .ncol) * 2,
                    "field data has two rows worth of samples");
    }
}

// tests/reference/test_alpine_track.py::test_default_track_has_no_single_contact_hazards[0..7]
[[gnu::noinline]] void test_default_track_has_no_single_contact_hazards() {
    for (std::uint64_t seed = 0; seed < 8; ++seed)
        bt::require(terrain::single_contact_hazards(
                            terrain::generate_alpine_track(AlpineGenSpec{},
                                                           seed))
                            .empty(),
                    "default generation stays single-contact clean");
}

// roll_multi_support_count — the 5 mm raster ground truth through the
// production ProfileQuery.
[[nodiscard]] std::int64_t roll_multi_support_count(
        const terrain::TrackSpec &track, double radius, double delta_m = .010,
        double step_m = .005) {
    const cfg::TireBackendConfig config;
    const auto x = terrain::detail::arange(
            0., track.length_m + 2. * radius + step_m / 2., step_m);
    const auto z = terrain::build_profile(track, x);
    std::vector<terrain::Point> vertices(x.size());
    for (std::size_t i = 0; i < x.size(); ++i)
        vertices[i] = terrain::Point{x[i], z[i]};
    const terrain::ProfileQuery query(
            std::move(vertices), config.significant_delta_m,
            config.significance_fraction, config.distinct_normal_deg);
    const auto window = static_cast<std::ptrdiff_t>(
            std::llround(radius / step_m));
    std::int64_t count = 0;
    for (std::ptrdiff_t i = window + 1;
         i < static_cast<std::ptrdiff_t>(x.size()) - window - 1; ++i) {
        const auto lo = static_cast<std::size_t>(i - window);
        const auto hi = static_cast<std::size_t>(i + window + 1);
        double best = std::numeric_limits<double>::lowest();
        for (std::size_t j = lo; j < hi; ++j) {
            const double dx = x[static_cast<std::size_t>(i)] - x[j];
            best = std::max(
                    best,
                    z[j] + std::sqrt(std::max(radius * radius - dx * dx,
                                              0.)));
        }
        const terrain::Point centre{x[static_cast<std::size_t>(i)],
                                    best - delta_m};
        count += query.contact(centre, radius).multi_support;
    }
    return count;
}

// tests/reference/test_alpine_track.py::test_default_track_rolls_clean_for_both_wheels[0..2]
[[gnu::noinline]] void test_default_track_rolls_clean_for_both_wheels() {
    for (std::uint64_t seed = 0; seed < 3; ++seed) {
        const auto track =
                terrain::generate_alpine_track(AlpineGenSpec{}, seed);
        bt::require(roll_multi_support_count(track, kFrontRadiusM) == 0,
                    "front wheel rolls clean");
        bt::require(roll_multi_support_count(track, kRearRadiusM) == 0,
                    "rear wheel rolls clean");
    }
}

// tests/reference/test_alpine_track.py::test_opting_out_of_single_contact_produces_detectable_hazards
[[gnu::noinline]] void test_opting_out_of_single_contact_produces_detectable_hazards() {
    auto spec = AlpineGenSpec{};
    spec.single_contact_safe = false;
    spec.n_sharp_edges = {2, 2};
    spec.rock_amp_m = {.10, .10};
    spec.rock_corr_m = {.15, .15};
    const auto track = terrain::generate_alpine_track(spec, 0);
    const bool any_edge = std::ranges::any_of(
            track.obstacles, [](const terrain::Obstacle &o) {
                return std::holds_alternative<terrain::SquareEdge>(o.shape);
            });
    bt::require(any_edge, "sharp edges land when opted out");
    bt::require(!terrain::single_contact_hazards(track).empty(),
                "hazards are detected");
    bt::require(roll_multi_support_count(track, kFrontRadiusM) > 0,
                "the raster roll counts multi-support windows");
}

// tests/reference/test_alpine_track.py::test_redraw_keeps_a_marginal_rock_garden_clean
[[gnu::noinline]] void test_redraw_keeps_a_marginal_rock_garden_clean() {
    auto spec = AlpineGenSpec{};
    spec.rock_amp_m = {.07, .07};
    spec.rock_corr_m = {.45, .45};
    spec.n_rock_gardens = {2, 2};
    for (std::uint64_t seed = 0; seed < 8; ++seed) {
        const auto track = terrain::generate_alpine_track(spec, seed);
        bt::require(terrain::single_contact_hazards(track).empty(),
                    "redraw keeps the garden clean");
        for (const auto &o : track.obstacles)
            if (const auto *g =
                        std::get_if<terrain::RockGarden>(&o.shape))
                bt::require(g->amplitude_m <= .07,
                            "redrawn amplitude stays under the cap");
    }
}

// tests/reference/test_alpine_track.py::test_faceted_redraw_is_capped_not_endless
[[gnu::noinline]] void test_faceted_redraw_is_capped_not_endless() {
    auto spec = AlpineGenSpec{};
    spec.max_kink_deg = 1e-4;
    spec.n_rock_gardens = {1, 1};
    for (std::uint64_t seed = 0; seed < 8; ++seed)
        rejects<std::invalid_argument>(
                [&] { (void)terrain::generate_alpine_track(spec, seed); },
                "concave-span redraw exhausted");
}

// ---------------------------------------------------------------------
// Presets + frozen files
// ---------------------------------------------------------------------

// tests/reference/test_alpine_track.py::test_alpine_scree_preset_builder
[[gnu::noinline]] void test_alpine_scree_preset_builder() {
    const auto pinned = terrain::alpine_scree();
    const auto other = terrain::alpine_scree("7");
    bt::require(pinned.name == "alpine_scree" &&
                        other.name == "alpine_scree_7",
                "preset naming");
    bt::require(terrain::dump_track(pinned) ==
                        terrain::dump_track(terrain::generate_alpine_track(
                                AlpineGenSpec{}, terrain::ALPINE_PRESET_SEED,
                                "alpine_scree")),
                "the preset is the seeded generator output");
}

// tests/reference/test_alpine_track.py::test_preset_is_registered_and_seeded
[[gnu::noinline]] void test_preset_is_registered_and_seeded() {
    const auto names = terrain::available_presets();
    bt::require(std::ranges::find(names, "alpine_scree") != names.end(),
                "alpine_scree is registered");
    // SEEDED_PRESETS['alpine_scree'] == (alpine_scree, ALPINE_PRESET_SEED):
    // the preset resolves to the same builder output at the pinned seed.
    bt::require(terrain::dump_track(terrain::get_preset("alpine_scree")) ==
                        terrain::dump_track(terrain::alpine_scree()),
                "preset binding keeps builder and seed");
    const auto track = terrain::get_preset("alpine_scree");
    bt::require(track.name == "alpine_scree" && track.length_m == 120.,
                "preset track header");
}

// tests/reference/test_alpine_track.py::test_frozen_example_equals_the_preset_dump
[[gnu::noinline]] void test_frozen_example_equals_the_preset_dump() {
    const auto path = repo_file("examples/research/alpine_scree.toml");
    bt::require(read_text(path) == terrain::dump_track(
                                           terrain::get_preset(
                                                   "alpine_scree")),
                "the frozen example equals the preset dump");
    const auto track = terrain::load_track(path);
    bt::require(track.surface == "hardpack", "frozen base surface");
    bt::require(std::ranges::any_of(
                        track.surface_sections,
                        [](const terrain::SurfaceSection &s) {
                            return s.surface == "scree";
                        }),
                "a scree section is present");
    bt::require(track.length_m == 120., "frozen length");
}

// tests/reference/test_alpine_track.py::test_rocky_genspec_is_the_multi_contact_opt_in
[[gnu::noinline]] void test_rocky_genspec_is_the_multi_contact_opt_in() {
    const auto spec = AlpineGenSpec::from_dict(cfg::detail::parse_toml_file(
            repo_file("examples/research/alpine_rocky_genspec.toml")));
    bt::require(!spec.single_contact_safe &&
                        spec.n_sharp_edges[1] >= 1 &&
                        spec.rock_amp_m[1] > AlpineGenSpec{}.rock_amp_m[1],
                "rocky genspec is the multi-contact opt-in");
    bt::require(!terrain::single_contact_hazards(
                            terrain::generate_alpine_track(spec, 0))
                            .empty(),
                "the rocky spec generates detectable hazards");
}

// ---------------------------------------------------------------------
// CLI + research routing
// ---------------------------------------------------------------------

// tests/reference/test_alpine_track.py::test_ride_resolves_the_seeded_preset
[[gnu::noinline]] void test_ride_resolves_the_seeded_preset() {
    bt::require(terrain::dump_track(bike::cli::resolve_track(
                            "alpine_scree")) ==
                        terrain::dump_track(terrain::alpine_scree()),
                "the seeded preset resolves to the pinned builder");
    bt::require(terrain::dump_track(bike::cli::resolve_track(
                            "alpine_scree", "7")) ==
                        terrain::dump_track(terrain::alpine_scree("7")),
                "the seed override reroutes the builder");
    rejects<std::invalid_argument>([&] {
        (void)bike::cli::resolve_track("alpine_scree", std::nullopt, 150.);
    }, "--length");
    // ride_cli.track_seed folds into resolve_track's seed resolution
    // natively: the seedless resolve above already proves the preset
    // seed (41) wins, and the '7' resolve proves the override wins.
}

// tests/reference/test_alpine_track.py::test_ride_lists_the_preset_as_generated
// DROPPED — the Python --list tracks row text ('generated', '120 m') is
// a CLI-format pin; the native --list_tracks emits bare preset names and
// the registration half is covered by test_preset_is_registered_and_seeded.

// tests/reference/test_alpine_track.py::test_research_alpine_scenario_builds_the_generator_track
[[gnu::noinline]] void test_research_alpine_scenario_builds_the_generator_track() {
    const auto track = bike::research::resolve_research_track(
            std::nullopt, "alpine", std::nullopt, "3");
    bt::require(track.name == "alpine_3", "research alpine names the seed");
    bt::require(terrain::dump_track(track) ==
                        terrain::dump_track(terrain::generate_alpine_track(
                                AlpineGenSpec{}, 3, "alpine_3")),
                "the research scenario builds the generator track");
    const auto spec_file =
            std::filesystem::temp_directory_path() /
            ("alpine_genspec_" + std::to_string(::getpid()) + ".toml");
    {
        std::ofstream out(spec_file);
        out << "ramp_grade = [0.10, 0.18]\nwall_grade = [0.20, 0.24]\n";
    }
    const auto custom = bike::research::resolve_research_track(
            std::nullopt, "alpine", spec_file, "3");
    const auto x = bike::geometry::detail::np_linspace(0., 120., 1201);
    const auto custom_slope = custom.grade_profile->slope(x);
    bt::require(*std::ranges::max_element(custom_slope) <= .24 + 1e-12,
                "the custom genspec caps the wall grade");
    std::filesystem::remove(spec_file);
}

// tests/reference/test_alpine_track.py::test_research_gen_spec_rejects_unknown_keys_and_wrong_scenarios
[[gnu::noinline]] void test_research_gen_spec_rejects_unknown_keys_and_wrong_scenarios() {
    const auto bad = std::filesystem::temp_directory_path() /
                     ("alpine_bad_" + std::to_string(::getpid()) + ".toml");
    const auto good = std::filesystem::temp_directory_path() /
                      ("alpine_good_" + std::to_string(::getpid()) + ".toml");
    {
        std::ofstream(bad) << "rampgrade = [0.1, 0.2]\n";
        std::ofstream(good) << "ramp_grade = [0.1, 0.2]\n";
    }
    rejects<std::invalid_argument>([&] {
        (void)bike::research::resolve_research_track(std::nullopt, "alpine",
                                                   bad, "3");
    }, "unknown alpine generator key");
    rejects<std::invalid_argument>([&] {
        (void)bike::research::resolve_research_track(
                std::nullopt, "rough_uphill", good, "3");
    }, "requires scenario generated or alpine");
    std::filesystem::remove(bad);
    std::filesystem::remove(good);
}

// tests/reference/test_alpine_track.py::test_alpine_rocky_preset_is_frozen_and_hazardous_by_design
[[gnu::noinline]] void test_alpine_rocky_preset_is_frozen_and_hazardous_by_design() {
    const auto track = terrain::get_preset("alpine_rocky");
    bt::require(track.name == "alpine_rocky", "rocky preset name");
    // SEEDED_PRESETS['alpine_rocky'] == (alpine_rocky, ROCKY_PRESET_SEED).
    bt::require(terrain::dump_track(track) ==
                        terrain::dump_track(terrain::alpine_rocky()),
                "rocky preset binding keeps builder and seed");
    bt::require(read_text(repo_file("examples/research/alpine_rocky.toml")) ==
                        terrain::dump_track(track),
                "the frozen rocky example equals the preset dump");
    bt::require(!terrain::single_contact_hazards(track).empty(),
                "rocky is hazardous by design");
}

// tests/reference/test_alpine_track.py::test_distributed_physics_profile_loads
[[gnu::noinline]] void test_distributed_physics_profile_loads() {
    const auto config = cfg::load_physics(
            repo_file("examples/research/viewer_physics_distributed.toml"));
    bt::require(config.tires.backend == "distributed_2d_reference" &&
                        config.tires.surface_mode == "track",
                "the distributed profile selects the reference backend");
    bt::require(16 <= config.tires.distributed.station_count &&
                        config.tires.distributed.station_count <= 512,
                "station count inside the supported range");
}

} // namespace

int main() {
    return bt::run([] {
        test_scree_surface_is_registered_below_loose();
        test_default_spec_matches_the_design_numbers();
        test_spec_round_trips_through_dict_and_toml_lists();
        test_spec_hash_is_stable_across_numeric_input_types();
        test_spec_rejects_invalid_values();
        test_sharp_edges_need_the_multi_contact_opt_in();
        test_spec_rejects_lengths_below_the_grid();
        test_spec_accepts_the_grid_boundary();
        test_rhythm_fills_the_body_with_ramp_shelf_cycles();
        test_rhythm_grade_profile_is_flat_outside_the_body();
        test_rhythm_without_walls_or_counter_slopes();
        test_generate_rejects_bad_arguments();
        test_same_seed_same_bytes_and_toml_round_trip();
        test_different_seeds_differ();
        test_default_track_structure();
        test_default_track_has_no_single_contact_hazards();
        test_default_track_rolls_clean_for_both_wheels();
        test_opting_out_of_single_contact_produces_detectable_hazards();
        test_redraw_keeps_a_marginal_rock_garden_clean();
        test_faceted_redraw_is_capped_not_endless();
        test_alpine_scree_preset_builder();
        test_preset_is_registered_and_seeded();
        test_frozen_example_equals_the_preset_dump();
        test_rocky_genspec_is_the_multi_contact_opt_in();
        test_ride_resolves_the_seeded_preset();
        test_research_alpine_scenario_builds_the_generator_track();
        test_research_gen_spec_rejects_unknown_keys_and_wrong_scenarios();
        test_alpine_rocky_preset_is_frozen_and_hazardous_by_design();
        test_distributed_physics_profile_loads();
    });
}
