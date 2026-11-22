// Stage A canonical SurfaceMap evaluator and wheel-load friction law contracts
// (Spec §2.4, §3.1): bitwise parity outside transition bands, C1 Hermite
// smoothstep continuity, origin-opening section rule, uniform-span fast path,
// and C1 load-sensitive Stribeck friction.
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"
#include "terrain/types.hpp"
#include "writers/tire_common.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace {

nativecfg::SurfaceSpec cfg_surface(std::string_view name) {
    const bike::terrain::SurfaceSpec &s = bike::terrain::get_surface(name);
    return {
        .name = s.name,
        .mu_peak = s.mu_peak,
        .mu_slide = s.mu_slide,
        .slip_stiffness_per_load = s.slip_stiffness_per_load,
        .stribeck_speed_mps = s.stribeck_speed_mps,
        .crr_delta = s.crr_delta,
        .load_sensitivity = s.load_sensitivity,
    };
}

bool bits_eq(double a, double b) noexcept {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

bool sample_bits_eq(const tirecommon::SurfaceSample &a,
                    const tirecommon::SurfaceSample &b) noexcept {
    return a.name == b.name &&
           bits_eq(a.mu_peak, b.mu_peak) &&
           bits_eq(a.mu_slide, b.mu_slide) &&
           bits_eq(a.slip_stiffness_per_load, b.slip_stiffness_per_load) &&
           bits_eq(a.stribeck_speed_mps, b.stribeck_speed_mps) &&
           bits_eq(a.crr_delta, b.crr_delta) &&
           bits_eq(a.load_sensitivity, b.load_sensitivity);
}

void require_c1_edge(const nativecfg::SurfaceMap &map, double edge_x_m) {
    constexpr double h = 1e-8;
    const tirecommon::SurfaceSample minus =
            tirecommon::eval_surface(map, edge_x_m - h);
    const tirecommon::SurfaceSample plus =
            tirecommon::eval_surface(map, edge_x_m + h);
    const double inv_2h = 1.0 / (2.0 * h);
    bt::require(std::abs(plus.mu_peak - minus.mu_peak) * inv_2h < 1e-6,
                "C1 zero slope at transition edge: mu_peak");
    bt::require(std::abs(plus.mu_slide - minus.mu_slide) * inv_2h < 1e-6,
                "C1 zero slope at transition edge: mu_slide");
    bt::require(std::abs(plus.slip_stiffness_per_load -
                         minus.slip_stiffness_per_load) *
                                inv_2h <
                        1e-6,
                "C1 zero slope at transition edge: slip_stiffness_per_load");
    bt::require(std::abs(plus.stribeck_speed_mps - minus.stribeck_speed_mps) *
                                inv_2h <
                        1e-6,
                "C1 zero slope at transition edge: stribeck_speed_mps");
    bt::require(std::abs(plus.crr_delta - minus.crr_delta) * inv_2h < 1e-6,
                "C1 zero slope at transition edge: crr_delta");
    bt::require(
            std::abs(plus.load_sensitivity - minus.load_sensitivity) * inv_2h <
                    1e-6,
            "C1 zero slope at transition edge: load_sensitivity");
}

void test_constants_and_sample_from_spec() {
    bt::require(bits_eq(tirecommon::kSurfaceReferenceLoadN, 500.0),
                "reference wheel normal load is 500 N");
    bt::require(bits_eq(tirecommon::kSurfaceLoadSmoothDelta, 0.1),
                "load-sensitivity turn-on smoothing delta is 0.1");
    bt::require(bits_eq(tirecommon::kReferenceSlipStiffnessPerLoad, 15.0),
                "reference slip stiffness per load is 15.0");
    bt::require(bits_eq(tirecommon::kWheelRollingTaperRadps, 0.2),
                "wheel rolling taper is 0.2 rad/s");

    const nativecfg::SurfaceSpec spec = cfg_surface("scree");
    static_assert(noexcept(tirecommon::sample_from_spec(spec)));
    const tirecommon::SurfaceSample s = tirecommon::sample_from_spec(spec);
    bt::require(s.name == "scree" && s.name.data() == spec.name.data(),
                "sample_from_spec borrows canonical SurfaceSpec::name storage");
    bt::require(bits_eq(s.mu_peak, 0.45) && bits_eq(s.mu_slide, 0.38) &&
                        bits_eq(s.slip_stiffness_per_load, 4.5) &&
                        bits_eq(s.stribeck_speed_mps, 4.5) &&
                        bits_eq(s.crr_delta, 0.0080) &&
                        bits_eq(s.load_sensitivity, 0.15),
                "sample_from_spec copies all six scalar fields bitwise");
}

void test_bitwise_identity_outside_transitions_and_same_surface() {
    const nativecfg::SurfaceSpec asphalt = cfg_surface("asphalt");
    const nativecfg::SurfaceSpec hardpack = cfg_surface("hardpack");
    const nativecfg::SurfaceSpec wet = cfg_surface("wet");
    const nativecfg::SurfaceSpec loose = cfg_surface("loose");
    const nativecfg::SurfaceSpec scree = cfg_surface("scree");

    // 1a. Zero blend_m everywhere: bitwise identical to surface_at at all x.
    const nativecfg::SurfaceMap sharp_map{
        .surface = hardpack,
        .sections = {
            {.start_m = 2.0, .end_m = 5.0, .surface = asphalt, .blend_m = 0.0},
            {.start_m = 5.0, .end_m = 8.0, .surface = scree, .blend_m = 0.0},
            {.start_m = 10.0, .end_m = 13.0, .surface = wet, .blend_m = 0.0},
        },
    };
    for (const double x :
         {-1.0, 0.0, 1.999999, 2.0, 3.5, 4.999999, 5.0, 6.2, 7.999999, 8.0,
          9.0, 10.0, 11.5, 13.0, 20.0}) {
        const nativecfg::SurfaceSpec &want_spec =
                tirecommon::surface_at(sharp_map, x);
        const tirecommon::SurfaceSample got =
                tirecommon::eval_surface(sharp_map, x);
        bt::require(sample_bits_eq(got, tirecommon::sample_from_spec(want_spec)),
                    "blend_m == 0 matches sample_from_spec(surface_at) bitwise");
        bt::require(got.name.data() == want_spec.name.data(),
                    "blend_m == 0 borrows canonical name pointer");
    }

    // 1b. Nonzero blend_m: bitwise identical outside [x_b - 0.5*b, x_b + 0.5*b]
    // and at exact window endpoints.
    const nativecfg::SurfaceMap blended_map{
        .surface = hardpack,
        .sections = {
            {.start_m = 2.0, .end_m = 6.0, .surface = asphalt, .blend_m = 0.8},
            {.start_m = 6.0, .end_m = 10.0, .surface = scree, .blend_m = 0.8},
            {.start_m = 12.0, .end_m = 16.0, .surface = loose, .blend_m = 0.6},
        },
    };
    for (const double x :
         {-0.5, 0.0, 1.5, 1.6, 2.4, 4.0, 5.6, 6.4, 8.0, 9.6, 10.4, 11.0, 11.7,
          12.3, 14.0, 15.7, 16.3, 25.0}) {
        const nativecfg::SurfaceSpec &want_spec =
                tirecommon::surface_at(blended_map, x);
        const tirecommon::SurfaceSample got =
                tirecommon::eval_surface(blended_map, x);
        bt::require(sample_bits_eq(got, tirecommon::sample_from_spec(want_spec)),
                    "outside transition windows eval_surface is bitwise pure");
    }

    // 1c. Same-surface boundary (L == R): non-dyadic fields (0.60, 0.0025,
    // 0.08) would suffer 1-ULP drift under (1 - s)*L + s*R; eval_surface must
    // short-circuit to exact identity.
    const nativecfg::SurfaceMap same_map{
        .surface = hardpack,
        .sections = {
            {.start_m = 2.0, .end_m = 5.0, .surface = hardpack, .blend_m = 1.0},
            {.start_m = 5.0, .end_m = 9.0, .surface = hardpack, .blend_m = 1.0},
        },
    };
    const tirecommon::SurfaceSample want_hp =
            tirecommon::sample_from_spec(hardpack);
    for (const double x :
         {1.55, 1.7, 1.85, 2.0, 2.13, 2.3, 4.63, 4.85, 5.0, 5.17, 5.37, 8.7,
          9.0, 9.3}) {
        const tirecommon::SurfaceSample got =
                tirecommon::eval_surface(same_map, x);
        bt::require(sample_bits_eq(got, want_hp),
                    "same-surface transition band preserves 0-ULP identity");
        bt::require(got.name.data() ==
                            tirecommon::surface_at(same_map, x).name.data(),
                    "same-surface transition band borrows surface_at name");
    }

    // 1d. Non-finite x_m rejected with std::invalid_argument.
    for (const double bad_x : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
        bt::invalid([&] { (void)tirecommon::eval_surface(blended_map, bad_x); });
    }
}

void test_c1_continuity_midpoint_and_hermite_curve() {
    const nativecfg::SurfaceSpec asphalt = cfg_surface("asphalt");
    const nativecfg::SurfaceSpec hardpack = cfg_surface("hardpack");
    const nativecfg::SurfaceSpec scree = cfg_surface("scree");

    // Covers base->sec (x_b = 2.0, b = 0.8), adjacent sec->sec across an
    // IEEE-754 exponent boundary (x_b = 8.0, b = 0.4), and sec->base
    // (x_b = 14.0, b = 0.6).
    const nativecfg::SurfaceMap map{
        .surface = hardpack,
        .sections = {
            {.start_m = 2.0, .end_m = 8.0, .surface = asphalt, .blend_m = 0.4},
            {.start_m = 8.0, .end_m = 14.0, .surface = scree, .blend_m = 0.4},
        },
    };

    // C1 slope < 1e-6 at all six edges: 1.8, 2.2, 7.8, 8.2, 13.8, 14.2.
    for (const double edge : {1.8, 2.2, 7.8, 8.2, 13.8, 14.2}) {
        require_c1_edge(map, edge);
    }

    // Exact midpoint average 0.5 * (L + R) at x_b = 2.0, 8.0, 14.0.
    {
        const tirecommon::SurfaceSample mid_8 =
                tirecommon::eval_surface(map, 8.0);
        bt::require(bits_eq(mid_8.mu_peak,
                            0.5 * (asphalt.mu_peak + scree.mu_peak)) &&
                            bits_eq(mid_8.mu_slide,
                                    0.5 * (asphalt.mu_slide + scree.mu_slide)) &&
                            bits_eq(mid_8.slip_stiffness_per_load,
                                    0.5 * (asphalt.slip_stiffness_per_load +
                                           scree.slip_stiffness_per_load)) &&
                            bits_eq(mid_8.stribeck_speed_mps,
                                    0.5 * (asphalt.stribeck_speed_mps +
                                           scree.stribeck_speed_mps)) &&
                            bits_eq(mid_8.crr_delta,
                                    0.5 * (asphalt.crr_delta +
                                           scree.crr_delta)) &&
                            bits_eq(mid_8.load_sensitivity,
                                    0.5 * (asphalt.load_sensitivity +
                                           scree.load_sensitivity)),
                    "midpoint x_b = 8.0 equals exact 0.5 * (L + R)");
        bt::require(mid_8.name == "scree" &&
                            mid_8.name == tirecommon::surface_at(map, 8.0).name,
                    "midpoint name matches surface_at(map, x_b)");
    }

    // Hand-derived quarter-point Hermite values on [1.0, 3.0] (x_b = 2.0,
    // b = 2.0): at x = 1.5, t = 0.25 -> s = 5/32 = 0.15625; at x = 2.5,
    // t = 0.75 -> s = 27/32 = 0.84375.
    const nativecfg::SurfaceMap dyadic_map{
        .surface = hardpack,
        .sections = {
            {.start_m = 2.0, .end_m = 6.0, .surface = scree, .blend_m = 2.0},
        },
    };
    const tirecommon::SurfaceSample q1 =
            tirecommon::eval_surface(dyadic_map, 1.5);
    const tirecommon::SurfaceSample q3 =
            tirecommon::eval_surface(dyadic_map, 2.5);
    bt::require(std::abs(q1.mu_peak - (0.84375 * 0.80 + 0.15625 * 0.45)) <
                        1e-15 &&
                        std::abs(q1.crr_delta -
                                 (0.84375 * 0.0025 + 0.15625 * 0.0080)) < 1e-18,
                "quarter-point Hermite blend at t = 0.25 matches 5/32 weight");
    bt::require(std::abs(q3.mu_peak - (0.15625 * 0.80 + 0.84375 * 0.45)) <
                        1e-15 &&
                        std::abs(q3.load_sensitivity -
                                 (0.15625 * 0.08 + 0.84375 * 0.15)) < 1e-16,
                "three-quarter Hermite blend at t = 0.75 matches 27/32 weight");

    // Canonical name matches surface_at across interior points of every window.
    for (const double x :
         {1.81, 1.99, 2.0, 2.01, 2.19, 7.81, 7.99, 8.0, 8.01, 8.19, 13.81,
          13.99, 14.0, 14.01, 14.19}) {
        const nativecfg::SurfaceSpec &at_spec = tirecommon::surface_at(map, x);
        const tirecommon::SurfaceSample got = tirecommon::eval_surface(map, x);
        bt::require(got.name == at_spec.name &&
                            got.name.data() == at_spec.name.data(),
                    "eval_surface.name references surface_at(map, x).name");
    }

    // Blended right window [2.5, 3.5] kissing an unblended section [3.5, 6.0):
    // at x = 3.5 the sharp section [3.5, 6.0) takes effect immediately.
    const nativecfg::SurfaceMap kissing_sharp{
        .surface = hardpack,
        .sections = {
            {.start_m = 1.0, .end_m = 3.0, .surface = asphalt, .blend_m = 1.0},
            {.start_m = 3.5, .end_m = 6.0, .surface = scree, .blend_m = 0.0},
        },
    };
    bt::require(sample_bits_eq(tirecommon::eval_surface(kissing_sharp, 3.5),
                               tirecommon::sample_from_spec(scree)),
                "unblended section starting at previous window's x_1 wins at start_m");
}

void test_origin_section_has_no_left_transition() {
    const nativecfg::SurfaceSpec asphalt = cfg_surface("asphalt");
    const nativecfg::SurfaceSpec hardpack = cfg_surface("hardpack");
    const nativecfg::SurfaceMap map{
        .surface = hardpack,
        .sections = {
            {.start_m = 0.0, .end_m = 5.0, .surface = asphalt, .blend_m = 1.0},
        },
    };
    const tirecommon::SurfaceSample want_asphalt =
            tirecommon::sample_from_spec(asphalt);
    for (const double x : {-0.5, -1e-6, 0.0, 0.2, 4.49, 4.5}) {
        const tirecommon::SurfaceSample got = tirecommon::eval_surface(map, x);
        bt::require(sample_bits_eq(got, want_asphalt),
                    "sec[0].start_m == 0 returns pure sec[0].surface on (-inf, end - 0.5*b]");
    }
    // Right boundary at x = 5.0 still blends asphalt -> hardpack on [4.5, 5.5].
    require_c1_edge(map, 4.5);
    require_c1_edge(map, 5.5);
    const tirecommon::SurfaceSample mid_5 = tirecommon::eval_surface(map, 5.0);
    bt::require(bits_eq(mid_5.mu_peak, 0.5 * (asphalt.mu_peak + hardpack.mu_peak)),
                "origin-opening section still blends at its right boundary");
}

void test_uniform_surface_for_span() {
    const nativecfg::SurfaceSpec asphalt = cfg_surface("asphalt");
    const nativecfg::SurfaceSpec hardpack = cfg_surface("hardpack");
    const nativecfg::SurfaceSpec wet = cfg_surface("wet");
    const nativecfg::SurfaceSpec loose = cfg_surface("loose");
    const nativecfg::SurfaceSpec scree = cfg_surface("scree");

    // Uniform map (no sections): every valid span returns &map.surface.
    const nativecfg::SurfaceMap uniform{.surface = asphalt, .sections = {}};
    bt::require(tirecommon::uniform_surface_for_span(uniform, -5.0, 100.0) ==
                        &uniform.surface,
                "uniform map fast-path returns &map.surface");
    bt::require(tirecommon::uniform_surface_for_span(uniform, 3.0, 3.0) ==
                        &uniform.surface,
                "degenerate single-point span is valid");

    // Validation rejections: non-finite bounds or x_min_m > x_max_m.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    bt::invalid([&] {
        (void)tirecommon::uniform_surface_for_span(uniform, nan, 1.0);
    });
    bt::invalid([&] {
        (void)tirecommon::uniform_surface_for_span(uniform, 0.0, nan);
    });
    bt::invalid([&] {
        (void)tirecommon::uniform_surface_for_span(uniform, -inf, 1.0);
    });
    bt::invalid([&] {
        (void)tirecommon::uniform_surface_for_span(uniform, 0.0, inf);
    });
    bt::invalid([&] {
        (void)tirecommon::uniform_surface_for_span(uniform, 2.0, 1.999999);
    });

    const nativecfg::SurfaceMap map{
        .surface = hardpack,
        .sections = {
            // [0, 5) with blend 1.0 -> pure (-inf, 4.5), window [4.5, 5.5]
            {.start_m = 0.0, .end_m = 5.0, .surface = asphalt, .blend_m = 1.0},
            // [5, 10) with blend 1.0 -> pure (5.5, 9.5), window [9.5, 10.5]
            {.start_m = 5.0, .end_m = 10.0, .surface = scree, .blend_m = 1.0},
            // gap (10.5, 12.7); [13, 18) with blend 0.6 -> windows [12.7, 13.3] & [17.7, 18.3]
            {.start_m = 13.0, .end_m = 18.0, .surface = wet, .blend_m = 0.6},
            // gap (18.3, 20.0); [20, 24) with blend 0.0 -> sharp [20.0, 24.0)
            {.start_m = 20.0, .end_m = 24.0, .surface = loose, .blend_m = 0.0},
        },
    };

    // Origin section pure region (-inf, 4.5):
    bt::require(tirecommon::uniform_surface_for_span(map, -0.5, 0.2) ==
                        &map.sections[0].surface,
                "origin section span across x = 0 returns &sec[0].surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 0.0, 4.4999) ==
                        &map.sections[0].surface,
                "span inside [0, 4.5) returns &sec[0].surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 0.0, 4.5) == nullptr,
                "span touching right transition window edge 4.5 returns nullptr");

    // Transition window [4.5, 5.5] and sec[1] pure interior (5.5, 9.5):
    bt::require(tirecommon::uniform_surface_for_span(map, 4.8, 5.2) == nullptr,
                "span inside transition window returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 5.5, 6.0) == nullptr,
                "span touching left transition window edge 5.5 returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 5.5001, 9.4999) ==
                        &map.sections[1].surface,
                "span strictly inside (5.5, 9.5) returns &sec[1].surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 9.0, 9.5) == nullptr,
                "span touching right transition window edge 9.5 returns nullptr");

    // Background gap (10.5, 12.7) and sec[2] interior (13.3, 17.7):
    bt::require(tirecommon::uniform_surface_for_span(map, 10.5, 11.0) == nullptr,
                "gap span touching 10.5 returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 10.5001, 12.6999) ==
                        &map.surface,
                "span inside pure background gap returns &map.surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 12.0, 12.7) == nullptr,
                "gap span touching 12.7 returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 13.3001, 17.6999) ==
                        &map.sections[2].surface,
                "span inside sec[2] pure region returns &sec[2].surface");

    // Sharp section [20.0, 24.0) with blend_m == 0.0:
    bt::require(tirecommon::uniform_surface_for_span(map, 18.3001, 19.9999) ==
                        &map.surface,
                "gap before sharp section returns &map.surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 19.9, 20.0) == nullptr,
                "span ending at sharp section start_m crosses boundary");
    bt::require(tirecommon::uniform_surface_for_span(map, 20.0, 20.0) ==
                        &map.sections[3].surface &&
                        tirecommon::uniform_surface_for_span(map, 20.0, 23.9999) ==
                                &map.sections[3].surface,
                "span inside half-open [start_m, end_m) with blend 0 returns &sec.surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 20.0, 24.0) == nullptr &&
                        tirecommon::uniform_surface_for_span(map, 23.9, 24.0) ==
                                nullptr,
                "span reaching exact half-open end_m returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 24.0, 24.0) ==
                        &map.surface &&
                        tirecommon::uniform_surface_for_span(map, 24.0, 30.0) ==
                                &map.surface,
                "tail [end_m, +inf) after sharp section returns &map.surface");

    // Map whose first section starts at start_m > 0 with blend_m > 0:
    const nativecfg::SurfaceMap delayed_first{
        .surface = hardpack,
        .sections = {
            {.start_m = 3.0, .end_m = 7.0, .surface = asphalt, .blend_m = 1.0},
        },
    };
    bt::require(tirecommon::uniform_surface_for_span(delayed_first, -1.0, 2.4999) ==
                        &delayed_first.surface,
                "pre-section region (-inf, start_m - 0.5*b) returns &map.surface");
    bt::require(tirecommon::uniform_surface_for_span(delayed_first, 0.0, 2.5) ==
                        nullptr,
                "pre-section span touching start_m - 0.5*b returns nullptr");
}

void test_load_sensitivity_factor_and_sample_mu() {
    static_assert(noexcept(tirecommon::load_sensitivity_factor(600.0, 0.12)));

    // 5a. Bitwise 1.0 for F_z <= 500.0 N or k_z == 0.0.
    for (const double fz : {0.0, 100.0, 418.0, 490.5, 500.0}) {
        bt::require(bits_eq(tirecommon::load_sensitivity_factor(fz, 0.15), 1.0) &&
                            bits_eq(tirecommon::load_sensitivity_factor(fz, 0.5),
                                    1.0),
                    "load_sensitivity_factor is bitwise 1.0 up to 500 N");
    }
    for (const double fz : {0.0, 500.0, 750.0, 2500.0, 5000.0}) {
        bt::require(bits_eq(tirecommon::load_sensitivity_factor(fz, 0.0), 1.0),
                    "load_sensitivity_factor is bitwise 1.0 when k_z == 0");
    }

    // 5b. Hand-derived analytical values above 500 N:
    // F_z = 700 N -> u = 0.4, g(0.4) = 0.16 / 0.5 = 0.32; k_z = 0.25 -> 1 / 1.08.
    // F_z = 1000 N -> u = 1.0, g(1.0) = 1.0 / 1.1 = 10/11; k_z = 0.11 -> 10/11.
    bt::require(std::abs(tirecommon::load_sensitivity_factor(700.0, 0.25) -
                         (1.0 / 1.08)) < 1e-15,
                "load_sensitivity_factor matches 1 / 1.08 at F_z = 700 N, k_z = 0.25");
    bt::require(std::abs(tirecommon::load_sensitivity_factor(1000.0, 0.11) -
                         (10.0 / 11.0)) < 1e-15,
                "load_sensitivity_factor matches 10 / 11 at F_z = 1000 N, k_z = 0.11");

    // 5c. C1 smoothness at F_z = 500.0 N: g(0) = 0 and g'(0) = 0.
    {
        constexpr double h = 1e-5;
        const double slope_phi =
                (tirecommon::load_sensitivity_factor(500.0 + h, 0.5) -
                 tirecommon::load_sensitivity_factor(500.0 - h, 0.5)) /
                (2.0 * h);
        bt::require(std::abs(slope_phi) < 1e-9,
                    "load_sensitivity_factor has zero derivative at 500 N");

        const tirecommon::SurfaceSample scree =
                tirecommon::sample_from_spec(cfg_surface("scree"));
        const double slope_mu =
                (tirecommon::sample_mu(scree, 1.2, 500.0 + h) -
                 tirecommon::sample_mu(scree, 1.2, 500.0 - h)) /
                (2.0 * h);
        bt::require(std::abs(slope_mu) < 1e-9,
                    "sample_mu has zero derivative wrt load at 500 N");
    }

    // 5d. Bitwise parity of sample_mu with surface_mu when F_z <= 500 N or
    // load_sensitivity == 0 (no multiply-by-1.0 rounding).
    for (const std::string_view name :
         {"asphalt", "hardpack", "wet", "loose", "scree"}) {
        const nativecfg::SurfaceSpec spec = cfg_surface(name);
        const tirecommon::SurfaceSample sample =
                tirecommon::sample_from_spec(spec);
        tirecommon::SurfaceSample zero_kz = sample;
        zero_kz.load_sensitivity = 0.0;

        for (const double vs : {-4.5, -0.35, -0.0, 0.0, 0.35, 1.7, 4.5, 12.0}) {
            const double want = tirecommon::surface_mu(spec, vs);
            for (const double fz : {0.0, 418.0, 490.5, 500.0}) {
                bt::require(bits_eq(tirecommon::sample_mu(sample, vs, fz), want),
                            "sample_mu matches surface_mu bitwise for F_z <= 500 N");
            }
            bt::require(bits_eq(tirecommon::sample_mu(zero_kz, vs, 1800.0), want),
                        "sample_mu matches surface_mu bitwise when k_z == 0");
        }
    }

    // 5e. Strict monotonicity of peak traction F_z * sample_mu(s, v_s, F_z)
    // across [0, 5000] N at k_z = 0.5 (worst-case legal saturation) and on
    // every calibrated surface.
    {
        tirecommon::SurfaceSample max_kz =
                tirecommon::sample_from_spec(cfg_surface("scree"));
        max_kz.load_sensitivity = 0.5;
        double prev_force = 0.0;
        double prev_mu = tirecommon::sample_mu(max_kz, 0.8, 500.0);
        for (int step = 1; step <= 500; ++step) {
            const double fz = 10.0 * static_cast<double>(step);
            const double mu = tirecommon::sample_mu(max_kz, 0.8, fz);
            const double force = fz * mu;
            bt::require(force > prev_force,
                        "F_z * sample_mu strictly increases up to 5000 N at k_z = 0.5");
            if (fz > 500.0) {
                bt::require(mu < prev_mu,
                            "sample_mu strictly decreases above 500 N when k_z > 0");
            }
            prev_force = force;
            prev_mu = mu;
        }
    }

    // 5f. Domain and overflow rejections on sample_mu.
    const tirecommon::SurfaceSample wet =
            tirecommon::sample_from_spec(cfg_surface("wet"));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double bad_vs : {nan, inf, -inf}) {
        bt::invalid([&] { (void)tirecommon::sample_mu(wet, bad_vs, 400.0); });
    }
    for (const double bad_fz : {-1e-12, -100.0, nan, inf, -inf}) {
        bt::invalid([&] { (void)tirecommon::sample_mu(wet, 0.5, bad_fz); });
    }
    {
        const tirecommon::SurfaceSample huge{
            .name = "huge",
            .mu_peak = 1e308,
            .mu_slide = -1e308,
            .slip_stiffness_per_load = 15.0,
            .stribeck_speed_mps = 4.5,
            .crr_delta = 0.0,
            .load_sensitivity = 0.0,
        };
        bool overflowed = false;
        try {
            (void)tirecommon::sample_mu(huge, 0.0, 400.0);
        } catch (const std::overflow_error &) {
            overflowed = true;
        }
        bt::require(overflowed,
                    "non-finite derived Stribeck mu raises std::overflow_error");
    }

    // 5g. Extreme finite load: u*u overflows binary64 once u exceeds
    // ~1.34e154, so the unguarded rational form collapses to zero. The
    // factor must stay strictly positive finite and track the high-load
    // asymptote 500 / (k_z * F_z) (g(u) -> u for u >> delta).
    {
        constexpr double fz = 1e305;
        constexpr double kz = 0.15;
        const double extreme = tirecommon::load_sensitivity_factor(fz, kz);
        bt::require(std::isfinite(extreme) && extreme > 0.0,
                    "load_sensitivity_factor is strictly positive finite at F_z = 1e305 N");
        const double asymptote = 500.0 / (kz * fz);
        bt::require(std::abs(extreme - asymptote) <= 1e-14 * asymptote,
                    "extreme-load factor matches 500 / (k_z * F_z) within 1e-14 relative");
        const double max_load = tirecommon::load_sensitivity_factor(
                std::numeric_limits<double>::max(), 0.5);
        bt::require(std::isfinite(max_load) && max_load > 0.0,
                    "load_sensitivity_factor is strictly positive finite at DBL_MAX, k_z = 0.5");
        // Branch-boundary control: u = 1e150 keeps the rational form, and
        // both sides of the switch agree with 1 / (1 + k_z * u) tightly.
        const double f_edge = 500.0 * (1e150 + 1.0);
        const double u_edge = f_edge / 500.0 - 1.0;
        const double g_edge = (u_edge * u_edge) / (u_edge + 0.1);
        const double want_edge = 1.0 / (1.0 + 0.15 * g_edge);
        const double got_edge = tirecommon::load_sensitivity_factor(f_edge, 0.15);
        bt::require(std::abs(got_edge - want_edge) <= 1e-15 * want_edge,
                    "u at the 1e150 branch boundary keeps the rational value");
        const double f_above = 500.0 * (2e150 + 1.0);
        const double u_above = f_above / 500.0 - 1.0;
        const double want_above = 1.0 / (1.0 + 0.15 * u_above);
        const double got_above = tirecommon::load_sensitivity_factor(f_above, 0.15);
        bt::require(std::abs(got_above - want_above) <= 1e-15 * want_above,
                    "u above the 1e150 branch boundary follows g(u) = u");
    }
}

// ---- Stage C: obstacle micro-surfaces (Spec §3) ---------------------------

[[nodiscard]] nativecfg::SurfaceSpec cfg_obstacle_surface(std::string_view name) {
    const bike::terrain::SurfaceSpec &s =
            bike::terrain::get_obstacle_surface(name);
    return {
        .name = s.name,
        .mu_peak = s.mu_peak,
        .mu_slide = s.mu_slide,
        .slip_stiffness_per_load = s.slip_stiffness_per_load,
        .stribeck_speed_mps = s.stribeck_speed_mps,
        .crr_delta = s.crr_delta,
        .load_sensitivity = s.load_sensitivity,
    };
}

void test_micro_surface_registry_and_obstacle_lookup() {
    namespace t = bike::terrain;
    // The ten-name obstacle domain: the five macro surfaces plus five local
    // micro surfaces, all sharing stribeck_speed_mps = 4.5 (Spec §3.1).
    bt::require(t::available_micro_surfaces() ==
                    std::vector<std::string>{"dry_wood", "wet_wood",
                                             "polished_rock", "wet_rock",
                                             "mud_lens"},
                "micro registry carries the exact five names in order");
    bt::require(t::available_surfaces() ==
                    std::vector<std::string>{"asphalt", "hardpack", "loose",
                                             "wet", "scree"},
                "macro registry keeps the exact five names");
    struct Row {
        const char *name;
        double mu_peak, mu_slide, c_k, crr, k_z;
    };
    const std::array<Row, 5> rows{{
        {.name = "dry_wood", .mu_peak = .45, .mu_slide = .35, .c_k = 11., .crr = .0020, .k_z = .08},
        {.name = "wet_wood", .mu_peak = .25, .mu_slide = .18, .c_k = 9., .crr = .0030, .k_z = .12},
        {.name = "polished_rock", .mu_peak = .60, .mu_slide = .45, .c_k = 14., .crr = .0010, .k_z = .06},
        {.name = "wet_rock", .mu_peak = .30, .mu_slide = .20, .c_k = 12., .crr = .0020, .k_z = .08},
        {.name = "mud_lens", .mu_peak = .28, .mu_slide = .20, .c_k = 5., .crr = .0100, .k_z = .15}}};
    for (const auto &r : rows) {
        const auto &s = t::get_obstacle_surface(r.name);
        bt::require(s.name == r.name && s.mu_peak == r.mu_peak &&
                        s.mu_slide == r.mu_slide &&
                        s.slip_stiffness_per_load == r.c_k &&
                        s.stribeck_speed_mps == 4.5 &&
                        s.crr_delta == r.crr && s.load_sensitivity == r.k_z,
                    "micro surface carries its calibrated spec values");
        s.validate();
    }
    for (const char *name :
         {"asphalt", "hardpack", "loose", "wet", "scree"}) {
        bt::require(&t::get_obstacle_surface(name) == &t::get_surface(name),
                    "obstacle lookup resolves macro names to the same registry spec");
    }
    bt::invalid([] { (void)t::get_surface("wet_wood"); });
    bt::invalid([] { (void)t::get_obstacle_surface("dry_dirt"); });
}

void test_obstacle_micro_surface_spans() {
    namespace t = bike::terrain;
    // Roots: one span per bump, width_m wide, gaps keep the base surface.
    t::Obstacle roots{.shape = t::Roots{.start_m = 10., .n_bumps = 5,
                                       .height_m = .08, .width_m = .25,
                                       .section_length_m = 7., .jitter = .35,
                                       .seed = "2029"},
                      .surface = "wet_wood", .surface_blend_m = .02};
    const auto centres =
            std::get<t::Roots>(roots.shape).bump_centres_m();
    const auto spans = roots.micro_surface_spans();
    bt::require(spans.size() == centres.size(),
                "one micro span per root bump");
    for (std::size_t i = 0; i < spans.size(); ++i) {
        bt::require(spans[i].start_m == 10. + centres[i] - .125 &&
                        spans[i].end_m == 10. + centres[i] + .125 &&
                        spans[i].blend_m == .02 &&
                        spans[i].surface == "wet_wood",
                    "root span wraps its bump centre with the obstacle blend");
        if (i != 0)
            bt::require(spans[i].start_m > spans[i - 1].end_m,
                        "inter-root gaps keep the base surface");
    }
    // Pothole family and g_out: the bottom 60 percent of the interval.
    const t::Obstacle hole{.shape = t::Pothole{.start_m = 20., .depth_m = .18,
                                        .hole_length_m = .6},
                     .surface = "mud_lens"};
    const auto pit = hole.micro_surface_spans();
    bt::require(pit.size() == 1 && pit[0].start_m == 20. + .2 * .6 &&
                    pit[0].end_m == 20. + .8 * .6 && pit[0].blend_m == 0. &&
                    pit[0].surface == "mud_lens",
                "pothole mud lens covers the bottom 60 percent");
    const t::Obstacle gout{.shape = t::GOut{.start_m = 40., .depth_m = .35,
                                     .dip_length_m = 5.},
                     .surface = "wet_rock"};
    const auto dip = gout.micro_surface_spans();
    bt::require(dip.size() == 1 && dip[0].start_m == 40. + .2 * 5. &&
                    dip[0].end_m == 40. + .8 * 5. &&
                    dip[0].surface == "wet_rock",
                "g_out mud lens covers the bottom 60 percent");
    // Every other non-zero-length obstacle overlays its whole interval.
    const t::Obstacle ledge{.shape = t::Bump{.start_m = 30., .height_m = .05,
                                      .bump_length_m = .4},
                      .surface = "polished_rock"};
    const auto shelf = ledge.micro_surface_spans();
    bt::require(shelf.size() == 1 && shelf[0].start_m == 30. &&
                    shelf[0].end_m == 30. + .4 &&
                    shelf[0].surface == "polished_rock",
                "plain obstacle spans its whole interval");
    // An unmarked obstacle generates nothing.
    bt::require(t::Obstacle{.shape = t::Bump{}}.micro_surface_spans().empty(),
                "unmarked obstacle carries no micro spans");
    // Obstacle-level domain checks.
    t::Obstacle neg = roots;
    neg.surface_blend_m = -.01;
    bt::invalid([&] { neg.validate(); });
    const t::Obstacle stray{.shape = t::Bump{.start_m = 5., .height_m = .05,
                                            .bump_length_m = .4},
                            .surface_blend_m = .1};
    bt::invalid([&] { stray.validate(); });
    t::Obstacle bogus = roots;
    bogus.surface = "dry_dirt";
    bt::invalid([&] { bogus.validate(); });
    // A macro name is a legal obstacle surface (obstacle lookup is ten-name).
    t::Obstacle macro = roots;
    macro.surface = "scree";
    macro.validate();
    bt::require(macro.micro_surface_spans().front().surface == "scree",
                "macro surface name resolves as an obstacle overlay");

    // Track-level validation over the generated spans.
    t::TrackSpec control = t::flat(60.);
    control.obstacles.push_back(roots);
    control.validate();
    // Collar wider than the span it wraps.
    t::TrackSpec fat_blend = t::flat(60.);
    t::Obstacle fat = roots;
    fat.surface_blend_m = .5;
    fat_blend.obstacles.push_back(fat);
    bt::invalid([&] { fat_blend.validate(); });
    // Expanded collars of adjacent obstacles must not overlap.
    const t::Obstacle left{.shape = t::Bump{.start_m = 30., .height_m = .05,
                                     .bump_length_m = .5},
                     .surface = "dry_wood", .surface_blend_m = .4};
    const t::Obstacle right{.shape = t::Bump{.start_m = 30.5, .height_m = .05,
                                      .bump_length_m = .5},
                      .surface = "wet_rock", .surface_blend_m = .4};
    t::TrackSpec overlap = t::flat(60.);
    overlap.obstacles = {left, right};
    bt::invalid([&] { overlap.validate(); });
    // Same adjacency with zero blends only touches — a valid control.
    t::Obstacle sharp_l = left, sharp_r = right;
    sharp_l.surface_blend_m = 0.;
    sharp_r.surface_blend_m = 0.;
    overlap.obstacles = {sharp_l, sharp_r};
    overlap.validate();
    // The expanded span must fit inside the track.
    t::TrackSpec off_track = t::flat(30.);
    const t::Obstacle edge{.shape = t::Bump{.start_m = 29.7, .height_m = .05,
                                     .bump_length_m = .3},
                     .surface = "wet_wood", .surface_blend_m = .2};
    off_track.obstacles.push_back(edge);
    bt::invalid([&] { off_track.validate(); });

    // TOML read: `surface`/`surface_blend_m` are obstacle-level keys.
    const auto parsed = t::track_from_dict(t::parse_track_toml(
            "name='micro'\nlength_m=20\n[[obstacles]]\ntype='roots'\n"
            "start_m=5\nn_bumps=3\nheight_m=0.06\nwidth_m=0.2\n"
            "section_length_m=6\njitter=0.1\nseed=9\nsurface='wet_wood'\n"
            "surface_blend_m=0.02\n"));
    bt::require(parsed.obstacles.size() == 1 &&
                    parsed.obstacles[0].surface == "wet_wood" &&
                    parsed.obstacles[0].surface_blend_m == .02,
                "track TOML carries obstacle surface keys");
    // TOML write: marked obstacles emit both keys; unmarked emit neither,
    // so existing tracks keep byte-identical serialization.
    const auto fields = t::obstacle_to_dict(roots);
    bool saw_surface = false, saw_blend = false;
    for (const auto &[key, value] :
         bike::io::as_object(fields, "obstacle")) {
        saw_surface = saw_surface || key == "surface";
        saw_blend = saw_blend || key == "surface_blend_m";
    }
    bt::require(saw_surface && saw_blend,
                "marked obstacle emits surface and surface_blend_m");
    for (const auto &[key, value] :
         bike::io::as_object(t::obstacle_to_dict(t::Obstacle{.shape = t::Bump{}}),
                             "obstacle")) {
        bt::require(key != "surface" && key != "surface_blend_m",
                    "unmarked obstacle emits no micro-surface keys");
    }
    // Full-dump round trip preserves the authored surface fields.
    const auto dumped = t::dump_track(parsed);
    bt::require(dumped.find("surface") != std::string::npos &&
                    t::dump_track(t::track_from_dict(t::parse_track_toml(dumped))) == dumped,
                "obstacle micro-surface keys round-trip through TOML");
    // An unknown obstacle surface name is rejected at the track boundary.
    bt::invalid([&] {
        (void)t::track_from_dict(t::parse_track_toml(
                "name='bad'\nlength_m=10\n[[obstacles]]\ntype='pothole'\n"
                "start_m=2\nhole_length_m=0.5\ndepth_m=0.1\n"
                "surface='dry_dirt'\n"));
    });
}

void test_eval_surface_micro_span_collars() {
    const nativecfg::SurfaceSpec hardpack = cfg_surface("hardpack");
    const nativecfg::SurfaceSpec scree = cfg_surface("scree");
    const nativecfg::SurfaceSpec wood = cfg_obstacle_surface("wet_wood");
    const nativecfg::SurfaceSpec dry_wood = cfg_obstacle_surface("dry_wood");

    // wet_wood span [4.0, 5.0] blend 0.4 on hardpack; a zero-blend dry_wood
    // span [10.5, 11.5) inside a scree section [10, 20) blend 0.4.
    const nativecfg::SurfaceMap map{
        .surface = hardpack,
        .sections = {
            {.start_m = 10.0, .end_m = 20.0, .surface = scree, .blend_m = 0.4},
        },
        .micro_spans = {
            {.start_m = 4.0, .end_m = 5.0, .blend_m = 0.4, .surface = wood},
            {.start_m = 10.5, .end_m = 11.5, .blend_m = 0.0, .surface = dry_wood},
        },
    };
    const auto wood_sample = tirecommon::sample_from_spec(wood);
    const auto dry_sample = tirecommon::sample_from_spec(dry_wood);
    const auto hardpack_sample = tirecommon::sample_from_spec(hardpack);
    const auto scree_sample = tirecommon::sample_from_spec(scree);

    // Away from every span: the hardpack base and the scree section rule.
    bt::require(sample_bits_eq(tirecommon::eval_surface(map, 3.0),
                               hardpack_sample),
                "outside micro spans the base surface evaluates pure");
    bt::require(sample_bits_eq(tirecommon::eval_surface(map, 10.3),
                               scree_sample),
                "scree section pure outside the embedded micro span");
    bt::require(sample_bits_eq(tirecommon::eval_surface(map, 12.0),
                               scree_sample),
                "scree section pure after the embedded micro span");

    // Pure micro core [s + b/2, e - b/2): the span spec bitwise, canonical
    // name borrowed from the span's own spec storage inside the map.
    for (const double x : {4.2, 4.3, 4.5, 4.7, 4.79}) {
        const auto got = tirecommon::eval_surface(map, x);
        bt::require(sample_bits_eq(got, wood_sample) &&
                        got.name.data() == map.micro_spans[0].surface.name.data(),
                    "pure micro core returns the span spec bitwise");
    }
    for (const double x : {10.5, 11.0, 11.499}) {
        const auto got = tirecommon::eval_surface(map, x);
        bt::require(sample_bits_eq(got, dry_sample) &&
                        got.name.data() == map.micro_spans[1].surface.name.data(),
                    "zero-blend micro span evaluates pure inside [s, e)");
    }
    bt::require(sample_bits_eq(tirecommon::eval_surface(map, 11.5),
                               scree_sample),
                "zero-blend span is half-open: x = e evaluates the section");

    // Left Hermite collar (3.8, 4.2) centred at start_m = 4.0.
    bt::require(sample_bits_eq(tirecommon::eval_surface(map, 3.8),
                               hardpack_sample),
                "left collar outer edge is pure base");
    {
        const auto mid = tirecommon::eval_surface(map, 4.0);
        bt::require(bits_eq(mid.mu_peak, 0.5 * (hardpack.mu_peak + wood.mu_peak)) &&
                        bits_eq(mid.crr_delta,
                                0.5 * (hardpack.crr_delta + wood.crr_delta)) &&
                        mid.name == "wet_wood",
                    "collar centre blends 50/50 and takes the micro name");
        const auto quarter = tirecommon::eval_surface(map, 3.9);
        bt::require(std::abs(quarter.mu_peak -
                             (0.84375 * hardpack.mu_peak + 0.15625 * wood.mu_peak)) < 1e-15,
                    "Hermite quarter point weights match t^2(3-2t)");
    }
    // Right Hermite collar (4.8, 5.2) centred at end_m = 5.0.
    {
        const auto mid = tirecommon::eval_surface(map, 5.0);
        bt::require(bits_eq(mid.mu_peak, 0.5 * (wood.mu_peak + hardpack.mu_peak)) &&
                        mid.name == "hardpack",
                    "right collar centre blends 50/50 and takes the base name");
        const auto quarter = tirecommon::eval_surface(map, 5.1);
        bt::require(std::abs(quarter.mu_slide -
                             (0.15625 * wood.mu_slide + 0.84375 * hardpack.mu_slide)) < 1e-15,
                    "right collar quarter point matches 27/32 base weight");
    }
    bt::require(sample_bits_eq(tirecommon::eval_surface(map, 5.2),
                               hardpack_sample),
                "right collar outer edge is pure base");
    // C1 zero-slope at all four collar edges.
    for (const double edge : {3.8, 4.2, 4.8, 5.2})
        require_c1_edge(map, edge);

    // uniform_surface_for_span: pure micro core -> the span's SurfaceSpec.
    bt::require(tirecommon::uniform_surface_for_span(map, 4.25, 4.75) ==
                    &map.micro_spans[0].surface,
                "span inside the micro core returns &micro_span.surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 10.5, 11.5) ==
                    nullptr,
                "span reaching the half-open micro end returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 10.6, 11.4) ==
                    &map.micro_spans[1].surface,
                "span inside the sharp micro interval returns &micro_span.surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 10.5, 10.5) ==
                    &map.micro_spans[1].surface,
                "point at sharp micro start uses the micro surface");
    // Collar or boundary contact -> nullptr.
    bt::require(tirecommon::uniform_surface_for_span(map, 3.95, 4.2) ==
                    nullptr,
                "span touching the left collar returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 4.95, 5.05) ==
                    nullptr,
                "span touching the right collar returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 4.0, 4.0) ==
                    nullptr,
                "degenerate span on the collar centre returns nullptr");
    // Gaps and pure macro regions keep their Stage A answers.
    bt::require(tirecommon::uniform_surface_for_span(map, 3.0, 3.5) ==
                    &map.surface,
                "gap before the micro span returns &map.surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 5.2, 6.0) ==
                    &map.surface,
                "gap after the micro span returns &map.surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 10.4, 10.6) ==
                    nullptr,
                "span reaching into the sharp micro interval returns nullptr");
    bt::require(tirecommon::uniform_surface_for_span(map, 10.25, 10.45) ==
                    &map.sections[0].surface,
                "pure scree interior beside the micro span returns &sec.surface");
    bt::require(tirecommon::uniform_surface_for_span(map, 9.9, 10.1) ==
                    nullptr,
                "section-transition window still returns nullptr");
}

} // namespace

int main() {
    return bt::run([] {
        test_constants_and_sample_from_spec();
        test_bitwise_identity_outside_transitions_and_same_surface();
        test_c1_continuity_midpoint_and_hermite_curve();
        test_origin_section_has_no_left_transition();
        test_uniform_surface_for_span();
        test_load_sensitivity_factor_and_sample_mu();
        test_micro_surface_registry_and_obstacle_lookup();
        test_obstacle_micro_surface_spans();
        test_eval_surface_micro_span_collars();
    });
}
