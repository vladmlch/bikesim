// Stage A canonical SurfaceMap evaluator and wheel-load friction law contracts
// (Spec §2.4, §3.1): bitwise parity outside transition bands, C1 Hermite
// smoothstep continuity, origin-opening section rule, uniform-span fast path,
// and C1 load-sensitive Stribeck friction.
#include "support/fixture_test.hpp"
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
    });
}
