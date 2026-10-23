// test_native_pneumatic_carcass.cpp — C++ port of
// tests/reference/test_native_pneumatic_carcass.py (batch 3.12a, T52).
//
// Replays evaluate_carcass / advance_unloaded / material rate / advect /
// np_interp_unit_grid / np_tanh / std::hypot against the frozen
// two-platform goldens — multi-step sequences included, post-call state
// pinned bitwise at every step. The math-probe rows are captured-input
// replays (the uint64-derived oracle draws cannot be regenerated through
// the signed NumpyRng API).
#include "pneumatic_reference_support.hpp"

namespace {

using namespace pneumatic_reference;
constexpr std::string_view kFile = "test_native_pneumatic_carcass.py";

// ---------------------------------------------------------------------
// Shared roads / hit drivers
// ---------------------------------------------------------------------
[[nodiscard]] biketyre::RoadProfile flat_road() {
    return {0.0, 0.01, std::vector<double>(1201, 0.0)};
}

// _bump_road — a short rounded bump plus a dip.
[[nodiscard]] biketyre::RoadProfile bump_road() {
    const auto x = arange_scaled(1201, 0.01);
    std::vector<double> z(1201);
    for (std::size_t i = 0; i < 1201; ++i) {
        const double bump = (x[i] - 5.05) / 0.16;
        const double dip = (x[i] - 4.72) / 0.10;
        const double tail = (x[i] - 5.55) / 0.09;
        z[i] = 0.028 * std::exp(-(bump * bump)) -
               0.018 * std::exp(-(dip * dip)) +
               0.012 * std::exp(-(tail * tail));
    }
    return {0.0, 0.01, std::move(z)};
}

// ---------------------------------------------------------------------
// test_carcass_progressive_load_unload_bitwise[n_rays-tyre{0,1}] — hub
// descends through a bump, holds, rises again.
// ---------------------------------------------------------------------
void test_carcass_progressive_load_unload_bitwise(std::int64_t n_rays,
                                                std::size_t tyre_index) {
    auto golden = node(kFile, "test_carcass_progressive_load_unload_bitwise",
                       std::to_string(n_rays) + "-tyre" +
                               std::to_string(tyre_index));
    const auto tyre =
            tyre_index == 0 ? bike::legacy::front_tyre()
                            : bike::legacy::rear_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    const auto road = bump_road();
    const double radius = tyre.outer_radius_mm * 0.001;
    constexpr std::array cz_path{0.46, 0.40, 0.355, 0.343, 0.339, 0.335,
                                 0.333, 0.336, 0.350, 0.385, 0.44};
    std::vector<CarcassStep> steps;
    steps.reserve(cz_path.size());
    for (const double cz : cz_path)
        steps.push_back(eval_step(
                biketyre::intersect(ring, 5.0, cz, radius, road), 0.004,
                9.5, 4.4));
    run_carcass_sequence(golden, ring, tyre, steps);
    golden.done();
}

// test_carcass_rim_strike_bitwise[n_rays] — deep deflection crosses
// rim_strike_deflection_mm: the FSM must start, track peaks and emit
// the completed RimStrikeEvent on release.
void test_carcass_rim_strike_bitwise(std::int64_t n_rays) {
    auto golden = node(kFile, "test_carcass_rim_strike_bitwise",
                       std::to_string(n_rays));
    const auto tyre = bike::legacy::front_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    const auto road = flat_road();
    const double radius = tyre.outer_radius_mm * 0.001;
    const double rim_defl = tyre.rim_strike_deflection_mm() * 0.001;
    const double cz_below = radius - rim_defl - 0.004;
    const double cz_below_deep = radius - rim_defl - 0.012;
    const std::array rows{
            std::array{5.0, 0.36, 0.002, -7.0, -3.2},
            std::array{5.0, cz_below, 0.002, -7.5, -3.2},
            std::array{5.0, cz_below_deep, 0.002, -8.0, -3.2},
            std::array{5.0, cz_below, 0.002, -8.0, -3.2},
            std::array{5.0, 0.34, 0.002, -8.0, -3.2},
            std::array{5.0, 0.36, 0.002, -7.0, -3.2},
    };
    std::vector<CarcassStep> steps;
    steps.reserve(rows.size());
    for (const auto &[cx, cz, dt, omega, speed] : rows)
        steps.push_back(eval_step(
                biketyre::intersect(ring, cx, cz, radius, road), dt,
                omega, speed));
    std::vector<std::optional<biketyre::RimStrikeEvent>> seen;
    run_carcass_sequence(
            golden, ring, tyre, steps,
            [&seen](const biketyre::CarcassResult &result,
                    const biketyre::CarcassState &,
                    const std::string &tag) {
                seen.push_back(result.rim_event);
                // The event actually engaged at some point.
                if (tag.starts_with("step 4"))
                    bt::require(std::ranges::any_of(
                                        seen,
                                        [](const auto &event) {
                                            return event.has_value();
                                        }),
                                "rim strike never engaged");
            });
    std::vector<biketyre::RimStrikeEvent> events;
    for (const auto &event : seen)
        if (event.has_value()) events.push_back(*event);
    bt::require(events.size() == 1, "exactly one completed rim event");
    const auto &event = events.front();
    bt::require(event.peak_load_n > 0.0 && event.peak_rim_force_n > 0.0,
                "rim event carries nonzero peaks");
    bt::require(event.absorbed_energy_j > 0.0 && event.speed_mps == 3.2,
                "rim event carries energy and |speed|");
    golden.done();
}

// test_carcass_airborne_landed_bitwise[n_rays] — airborne -> land ->
// airborne again.
void test_carcass_airborne_landed_bitwise(std::int64_t n_rays) {
    auto golden = node(kFile, "test_carcass_airborne_landed_bitwise",
                       std::to_string(n_rays));
    const auto tyre = bike::legacy::front_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    const auto road = flat_road();
    const double radius = tyre.outer_radius_mm * 0.001;
    const std::array czs{radius + 0.5, radius + 0.4, radius - 0.01,
                         radius - 0.02, radius + 0.5};
    std::vector<CarcassStep> steps;
    steps.reserve(czs.size());
    for (const double cz : czs)
        steps.push_back(eval_step(
                biketyre::intersect(ring, 5.0, cz, radius, road), 0.002,
                6.0, 2.0));
    run_carcass_sequence(golden, ring, tyre, steps);
    golden.done();
}

// test_carcass_advance_unloaded_sequence_bitwise[n_rays] — early
// return, the np.any gate skip and the spinning advect+decay path.
void test_carcass_advance_unloaded_sequence_bitwise(std::int64_t n_rays) {
    auto golden = node(kFile, "test_carcass_advance_unloaded_sequence_bitwise",
                       std::to_string(n_rays));
    const auto tyre = bike::legacy::front_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    const auto road = flat_road();
    const double radius = tyre.outer_radius_mm * 0.001;
    const double dtheta = ring.dtheta_rad();
    const double tau = tyre.rate_relaxation_s;
    const std::vector<CarcassStep> steps{
            // Not initialized yet — immediate return.
            advance_step(7.0, 0.002, dtheta, tau),
            // Land: initializes, maxwell stays zero -> gate skip.
            eval_step(biketyre::intersect(ring, 5.0, radius - 0.012,
                                          radius, road),
                      0.002, 6.0, 2.5),
            advance_step(7.0, 0.002, dtheta, tau),
            // Load deeper while rolling: Maxwell memory becomes nonzero.
            eval_step(biketyre::intersect(ring, 5.0, radius - 0.02,
                                          radius, road),
                      0.002, 6.0, 2.5),
            // Spinning advance: decay * advected row.
            advance_step(7.0, 0.002, dtheta, tau),
            advance_step(-4.0, 0.003, dtheta, tau),
            advance_step(0.0, 0.002, dtheta, tau),
            advance_step(7.0, 0.002, dtheta, tau),
            // Back on the ground.
            eval_step(biketyre::intersect(ring, 5.0, radius - 0.018,
                                          radius, road),
                      0.002, 6.0, 2.5),
    };
    run_carcass_sequence(golden, ring, tyre, steps);
    golden.done();
}

// ---------------------------------------------------------------------
// material_deflection_rate_mps / _advect_material unit coverage
// ---------------------------------------------------------------------
void test_carcass_material_rate_bitwise(std::int64_t n, double omega) {
    auto golden = node(kFile, "test_carcass_material_rate_bitwise",
                       std::to_string(n) + "-" + repr_binary64(omega));
    bike::random::NumpyRng rng(static_cast<std::uint64_t>(
            n * 7919 +
            static_cast<std::int64_t>(std::fabs(omega) * 1000)));
    std::vector<double> delta(static_cast<std::size_t>(n)),
            prev(static_cast<std::size_t>(n));
    rng.uniform(0.0, 0.09, delta);
    rng.uniform(0.0, 0.09, prev);
    vector(golden,
           biketyre::material_deflection_rate_mps(delta, prev, omega,
                                                  0.004, 0.117),
           "");
    golden.done();
}

void test_carcass_material_rate_edges(std::int64_t n) {
    auto golden = node(kFile, "test_carcass_material_rate_edges",
                       std::to_string(n));
    const auto size = static_cast<std::size_t>(n);
    const auto delta = np_linspace(0.0, 0.07, size);
    const auto prev = np_linspace(0.04, 0.0, size);
    for (const double omega : {12.0, -12.0})
        vector(golden,
               biketyre::material_deflection_rate_mps(delta, prev, omega,
                                                      0.002, 0.09),
               "");
    golden.done();
}

void test_carcass_advect_bitwise(double shift, std::int64_t n) {
    auto golden = node(kFile, "test_carcass_advect_bitwise",
                       repr_binary64(shift) + "-" + std::to_string(n));
    // rng.uniform(-1e3, 1e3, n): the wheel's low + range*next transform
    // contracts to fmadd on darwin but not linux — the captured row is
    // the replay source of truth.
    const auto values = golden.f64_input("input.values");
    const double dtheta = 0.11, dt = 0.004;
    const double omega = shift != 0.0 ? shift * dtheta / dt : 0.0;
    vector(golden,
           biketyre::advect_material(values, omega, dt, dtheta), "");
    golden.done();
}

void test_carcass_advect_zero_omega_copy() {
    auto golden = node(kFile, "test_carcass_advect_zero_omega_copy");
    const std::array values{1.5, -0.0, 4.25,
                            std::nextafter(0.0, 1.0)};
    vector(golden, biketyre::advect_material(values, 0.0, 0.01, 0.5),
           "");
    golden.done();
}

void test_carcass_empty_rows() {
    auto golden = node(kFile, "test_carcass_empty_rows");
    const std::span<const double> empty;
    vector(golden, biketyre::advect_material(empty, 0.0, 0.01, 0.1), "");
    vector(golden, biketyre::advect_material(empty, 3.0, 0.01, 0.1), "");
    for (const double omega : {0.0, 1.0, -2.0})
        vector(golden,
               biketyre::material_deflection_rate_mps(empty, empty, omega,
                                                      0.01, 0.1),
               "");
    // Width mismatch — the shape check precedes the empty path.
    const std::array two{1.0, 1.0};
    rejects<std::invalid_argument>([&] {
        (void)biketyre::material_deflection_rate_mps(empty, two, 0.0,
                                                   0.01, 0.1);
    });
    golden.done();
}

// ---------------------------------------------------------------------
// np.tanh / np.hypot transcendental probes — the uint64-derived rows
// are captured-input replays.
// ---------------------------------------------------------------------
void test_carcass_math_probes_bitwise() {
    auto golden = node(kFile, "test_carcass_math_probes_bitwise");
    const auto xs = golden.f64_input("input.xs");
    const auto ys = golden.f64_input("input.ys");
    std::vector<double> tanh_out(xs.size()),
            hypot_out(std::min(xs.size(), ys.size()));
    for (std::size_t i = 0; i < xs.size(); ++i)
        tanh_out[i] = numeric::np_tanh(xs[i]);
    for (std::size_t i = 0; i < hypot_out.size(); ++i)
        hypot_out[i] = std::hypot(xs[i], ys[i]);
    vector(golden, tanh_out, "");
    vector(golden, hypot_out, "");
    golden.done();
}

// ---------------------------------------------------------------------
// Nonnegative saturation, single-ray patch, boundary patches
// ---------------------------------------------------------------------
void test_carcass_saturation_and_patch_shapes_bitwise(
        std::int64_t n_rays) {
    auto golden =
            node(kFile, "test_carcass_saturation_and_patch_shapes_bitwise",
                 std::to_string(n_rays));
    const auto tyre = bike::legacy::front_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    bike::random::NumpyRng rng(static_cast<std::uint64_t>(n_rays));
    const auto n = static_cast<std::size_t>(n_rays);

    // --- step 1: deep wide load, rolling forward -------------------
    std::vector<double> delta(n, 0.0);
    const std::size_t centre = n / 2;
    std::array<double, 9> sprinkles{};
    rng.uniform(0.0, 1.0, sprinkles);
    for (std::size_t i = 0; i < 9; ++i)
        delta[centre - 4 + i] = 0.05 + 0.01 * sprinkles[i];
    std::vector<double> road_x(n);
    {
        double acc = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            acc += 0.001;
            road_x[i] = acc + 4.9;
        }
    }
    std::vector<double> road_z(n);
    rng.uniform(-0.05, -0.02, road_z);
    // rng.uniform's low + range*next transform is platform-contracted —
    // the draw keeps the stream aligned, the frozen row wins the bits.
    road_z = golden.f64_input("input.road_z");
    const auto hits1 = synth_hits(n, delta, &rng, road_x, road_z);
    biketyre::CarcassState state(n_rays);
    auto result = biketyre::evaluate_carcass(ring, hits1, tyre, state,
                                             0.002, 8.0, 3.0);
    carcass_result(golden, result, "load");
    carcass_state(golden, state, "load");

    // --- step 2: rapid partial unload — the strong negative Maxwell
    // contribution clamps total at zero on rays still loaded.
    std::vector<double> delta2(n);
    for (std::size_t i = 0; i < n; ++i) delta2[i] = delta[i] * 0.06;
    const auto hits2 = synth_hits(n, delta2, &rng, road_x, road_z);
    result = biketyre::evaluate_carcass(ring, hits2, tyre, state, 0.001,
                                        9.0, 3.1);
    carcass_result(golden, result, "unload-clamp");
    bool saw_negative = false;
    for (std::size_t i = 0; i < n; ++i) {
        const double pre = result.elastic_force_n[i] +
                                   result.maxwell_force_n[i] +
                                   result.hysteresis_force_n[i] +
                                   result.rim_force_n[i];
        if (pre < 0.0) saw_negative = true;
    }
    bt::require(saw_negative, "the saturation branch never ran");
    carcass_state(golden, state, "unload-clamp");

    // --- step 3: single-ray patch + boundary rays ------------------
    std::vector<double> delta3(n, 0.0);
    delta3[0] = 0.02;
    delta3[n - 1] = 0.015;
    delta3[n / 3] = 0.03;
    auto hits3 = synth_hits(n, delta3, &rng);
    // The drawn road_z row is wheel-contraction divergent — replay it.
    hits3.road_z_m = golden.f64_input("input.hits3 road_z_m");
    result = biketyre::evaluate_carcass(ring, hits3, tyre, state, 0.002,
                                        0.0, 0.0);
    carcass_result(golden, result, "single-ray");
    bt::require(result.contact_patches.size() == 3,
                "three disjoint patches");
    // The single-ray patch takes the one_ray_span branch.
    bt::require(result.contact_lengths_m.size() == 3,
                "three contact lengths");
    golden.done();
}

// ---------------------------------------------------------------------
// Validation order + failure atomicity
// ---------------------------------------------------------------------
void test_carcass_invalid_inputs_bitwise_atomicity(std::int64_t n_rays) {
    auto golden =
            node(kFile, "test_carcass_invalid_inputs_bitwise_atomicity",
                 std::to_string(n_rays));
    const auto tyre = bike::legacy::front_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    const auto n = static_cast<std::size_t>(n_rays);
    std::vector<double> delta(n, 0.0);
    delta[n / 2] = 0.01;
    auto hits = synth_hits(n, delta);
    // The drawn road_z row is wheel-contraction divergent — replay it.
    hits.road_z_m = golden.f64_input("input.road_z_m");
    biketyre::CarcassState state(n_rays);
    // Establish a nonzero history so the failed call has something to
    // lose.
    (void)biketyre::evaluate_carcass(ring, hits, tyre, state, 0.002,
                                     4.0, 1.0);

    // (dt_s, omega_radps, speed_mps, delta variant) rows in oracle
    // check order: dt invalid first, then non-finite omega/speed, then
    // delta shape/content.
    enum class DeltaKind { none, nan, neg, short_ };
    const struct {
        double dt, omega, speed;
        DeltaKind delta;
    } bad_cases[] = {
            {0.0, 4.0, 1.0, DeltaKind::none},
            {-0.002, 4.0, 1.0, DeltaKind::none},
            {std::numeric_limits<double>::quiet_NaN(), 4.0, 1.0,
             DeltaKind::none},
            {std::numeric_limits<double>::infinity(), 4.0, 1.0,
             DeltaKind::none},
            {0.002, std::numeric_limits<double>::quiet_NaN(), 1.0,
             DeltaKind::none},
            {0.002, std::numeric_limits<double>::infinity(), 1.0,
             DeltaKind::none},
            {0.002, 4.0, -std::numeric_limits<double>::infinity(),
             DeltaKind::none},
            {0.002, 4.0, 1.0, DeltaKind::short_},
            {0.002, 4.0, 1.0, DeltaKind::nan},
            {0.002, 4.0, 1.0, DeltaKind::neg},
    };
    for (const auto &bad : bad_cases) {
        auto o_hits = hits;
        if (bad.delta != DeltaKind::none) {
            auto row = delta;
            if (bad.delta == DeltaKind::nan)
                row[3] = std::numeric_limits<double>::quiet_NaN();
            else if (bad.delta == DeltaKind::neg)
                row[3] = -1e-9;
            else if (bad.delta == DeltaKind::short_)
                row.pop_back();
            o_hits.delta_m = std::move(row);
            o_hits.airborne = false;
        }
        rejects<std::invalid_argument>([&] {
            (void)biketyre::evaluate_carcass(ring, o_hits, tyre, state,
                                             bad.dt, bad.omega,
                                             bad.speed);
        });
        carcass_state(golden, state, "oracle atomicity");
    }

    // The state-mismatch check sits between the omega/speed and delta
    // checks: a state one ray too narrow fails even with a nan in
    // delta AND a good dt.
    auto nan_delta_hits = hits;
    nan_delta_hits.delta_m[0] =
            std::numeric_limits<double>::quiet_NaN();
    nan_delta_hits.airborne = false;
    biketyre::CarcassState narrow_state(n_rays - 1);
    rejects<std::invalid_argument>([&] {
        (void)biketyre::evaluate_carcass(ring, nan_delta_hits, tyre,
                                         narrow_state, 0.002, 4.0,
                                         1.0);
    });
    // The oracle's own wrong-ring check with an otherwise-valid call.
    rejects<std::invalid_argument>([&] {
        (void)biketyre::evaluate_carcass(ring, hits, tyre, narrow_state,
                                         0.002, 4.0, 1.0);
    });

    // Ordering: dt wins over bad omega and bad delta; omega wins over
    // bad delta.
    auto bad_hits = hits;
    bad_hits.delta_m.assign(
            n, std::numeric_limits<double>::quiet_NaN());
    bad_hits.airborne = false;
    rejects<std::invalid_argument>(
            [&] {
                (void)biketyre::evaluate_carcass(ring, bad_hits, tyre,
                                                 state, 0.0,
                                                 std::numeric_limits<
                                                         double>::quiet_NaN(),
                                                 0.0);
            },
            "dt_s");
    rejects<std::invalid_argument>(
            [&] {
                (void)biketyre::evaluate_carcass(
                        ring, bad_hits, tyre, state, 0.002,
                        std::numeric_limits<double>::infinity(), 0.0);
            },
            "finite");

    // The next valid call still matches bitwise.
    const auto result = biketyre::evaluate_carcass(ring, hits, tyre,
                                                   state, 0.002, 4.0,
                                                   1.0);
    carcass_result(golden, result, "post-failure");
    carcass_state(golden, state, "post-failure");
    golden.done();
}

// test_carcass_advance_unloaded_validation — early return before
// validation, the positivity check, and the np.any gate.
void test_carcass_advance_unloaded_validation() {
    auto golden = node(kFile, "test_carcass_advance_unloaded_validation");
    constexpr std::int64_t n_rays = 64;
    const auto tyre = bike::legacy::front_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    // Uninitialized: even invalid dt returns silently.
    biketyre::CarcassState early(n_rays);
    early.advance_unloaded(5.0, -1.0, 0.1, 0.2);
    carcass_state(golden, early, "uninitialized early return");

    // Build a state with nonzero Maxwell memory.
    const auto road = flat_road();
    const double radius = tyre.outer_radius_mm * 0.001;
    biketyre::CarcassState state(n_rays);
    auto hits = biketyre::intersect(ring, 5.0, radius - 0.02, radius,
                                    road);
    (void)biketyre::evaluate_carcass(ring, hits, tyre, state, 0.002,
                                     6.0, 2.0);
    hits = biketyre::intersect(ring, 5.0, radius - 0.012, radius, road);
    (void)biketyre::evaluate_carcass(ring, hits, tyre, state, 0.002,
                                     6.0, 2.0);
    bt::require(std::ranges::any_of(
                        state.maxwell_force_n(),
                        [](const double v) { return v != 0.0; }),
                "maxwell memory established");
    // Invalid params raise; state untouched. Rows:
    // (dt_s, dtheta_rad, relaxation_s) — omega stays 5.0.
    const std::array bad{
            std::array{0.0, ring.dtheta_rad(), 0.2},
            std::array{-1.0, ring.dtheta_rad(), 0.2},
            std::array{0.002, 0.0, 0.2},
            std::array{0.002, -0.5, 0.2},
            std::array{0.002, ring.dtheta_rad(), 0.0},
            std::array{0.002, ring.dtheta_rad(), -3.0},
    };
    for (const auto &[dt, dtheta, tau] : bad) {
        rejects<std::invalid_argument>([&] {
            state.advance_unloaded(5.0, dt, dtheta, tau);
        });
        carcass_state(golden, state, "advance atomicity");
    }
    // dt=nan passes the oracle's positivity check (nan <= 0 is False)
    // and propagates nan through the advect.
    state.advance_unloaded(5.0, std::numeric_limits<double>::quiet_NaN(),
                           ring.dtheta_rad(), 0.2);
    carcass_state(golden, state, "nan dt propagation");
    golden.done();
}

// test_carcass_state_construction_validation [port] — n < 3 is
// rejected at construction (the state-dict intake funnels into the
// same CarcassState ctor).
void test_carcass_state_construction_validation() {
    rejects<std::invalid_argument>(
            [] { biketyre::CarcassState narrow(2); },
            "at least 3 rays");
}

// test_carcass_first_step_initialized_branch — the first evaluate on a
// fresh state takes the uninitialized branch: material_rate and
// maxwell are exact +0.0 rows.
void test_carcass_first_step_initialized_branch() {
    auto golden = node(kFile, "test_carcass_first_step_initialized_branch");
    constexpr std::int64_t n_rays = 64;
    const auto tyre = bike::legacy::front_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    std::vector<double> delta(64, 0.0);
    for (std::size_t i = 30; i < 35; ++i) delta[i] = 0.02;
    auto hits = synth_hits(64, delta);
    // The drawn road_z row is wheel-contraction divergent — replay it.
    hits.road_z_m = golden.f64_input("input.road_z_m");
    biketyre::CarcassState state(n_rays);
    auto result = biketyre::evaluate_carcass(ring, hits, tyre, state,
                                             0.002, 5.0, 2.0);
    // The two +0.0 rows the uninitialized branch emits — recorded
    // first, in oracle order.
    vector(golden, result.material_rate_mps,
           "first-step material_rate_mps +0.0");
    vector(golden, result.maxwell_force_n,
           "first-step maxwell_force_n +0.0");
    bt::require(std::ranges::all_of(
                        result.material_rate_mps,
                        [](const double v) {
                            return v == 0.0 && !std::signbit(v);
                        }) &&
                        std::ranges::all_of(
                                result.maxwell_force_n,
                                [](const double v) {
                                    return v == 0.0 &&
                                           !std::signbit(v);
                                }),
                "the uninitialized branch emits +0.0 rows");
    carcass_result(golden, result, "first-step");
    carcass_state(golden, state, "first-step");
    // The second step takes the initialized branch.
    std::vector<double> delta2(64);
    for (std::size_t i = 0; i < 64; ++i) delta2[i] = delta[i] * 1.1;
    auto hits2 = synth_hits(64, delta2);
    hits2.road_z_m = golden.f64_input("input.hits2 road_z_m");
    result = biketyre::evaluate_carcass(ring, hits2, tyre, state, 0.002,
                                        5.0, 2.0);
    carcass_result(golden, result, "second-step");
    carcass_state(golden, state, "second-step");
    golden.done();
}

// test_carcass_multistep_mixed_speed_spin_bitwise[n_rays] — varying
// spin sign, speed (incl. negative for the event record's abs()), dt
// and hub path on the bump road.
void test_carcass_multistep_mixed_speed_spin_bitwise(std::int64_t n_rays) {
    auto golden =
            node(kFile, "test_carcass_multistep_mixed_speed_spin_bitwise",
                 std::to_string(n_rays));
    const auto tyre = bike::legacy::rear_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    const auto road = bump_road();
    const double radius = tyre.outer_radius_mm * 0.001;
    std::vector<CarcassStep> steps;
    steps.reserve(22);
    double cx = 4.6;
    for (std::int64_t k = 0; k < 22; ++k) {
        cx += 0.028;
        const double kd = static_cast<double>(k);
        const double cz = radius + 0.10 - std::min(kd * 0.013, 0.125);
        const double omega = k % 5 < 3 ? 8.0 : -6.5;
        const double speed =
                (3.4 + 0.1 * kd) * (k % 4 != 0 ? 1.0 : -1.0);
        const double dt = 0.002 + 0.0002 * static_cast<double>(k % 3);
        steps.push_back(eval_step(
                biketyre::intersect(ring, cx, cz, radius, road), dt,
                omega, speed));
    }
    run_carcass_sequence(golden, ring, tyre, steps);
    golden.done();
}

// test_carcass_negative_zero_delta_and_inf_hits — delta carrying -0.0
// (accepted: not < 0.0) and inf r_m — signed-zero propagation through
// np.maximum/np.where stays bitwise.
void test_carcass_negative_zero_delta_and_inf_hits() {
    auto golden =
            node(kFile, "test_carcass_negative_zero_delta_and_inf_hits");
    constexpr std::int64_t n_rays = 64;
    const auto tyre = bike::legacy::front_tyre();
    const biketyre::RayRing ring(n_rays, kHalf);
    std::vector<double> delta(64, 0.0);
    delta[10] = -0.0;
    delta[11] = 0.01;
    auto hits = synth_hits(64, delta);
    // The drawn road_z row is wheel-contraction divergent — replay it.
    hits.road_z_m = golden.f64_input("input.road_z_m");
    biketyre::CarcassState state(n_rays);
    const auto result = biketyre::evaluate_carcass(ring, hits, tyre,
                                                   state, 0.002, 3.0,
                                                   1.0);
    carcass_result(golden, result, "-0.0 delta");
    carcass_state(golden, state, "-0.0 delta");
    golden.done();
}

// ---------------------------------------------------------------------
// np.interp on the unit knot grid — np_interp_unit_grid's lenxp == 1
// branch, the general path and the empty rejection.
// ---------------------------------------------------------------------
void test_pneumatic_interp_single_knot_bitwise() {
    auto golden = node(kFile, "test_pneumatic_interp_single_knot_bitwise");
    const std::array values{7.25};
    // (x, is_np_float64) — the np.nextafter params carry the np
    // repr spelling in the label.
    const std::array<std::pair<double, bool>, 12> xs{{
            {0.0, false},
            {-0.0, false},
            {std::numeric_limits<double>::quiet_NaN(), false},
            {-std::numeric_limits<double>::quiet_NaN(), false},
            {1e-300, false},
            {-1e-300, false},
            {0.5, false},
            {-0.5, false},
            {std::numeric_limits<double>::infinity(), false},
            {-std::numeric_limits<double>::infinity(), false},
            {std::nextafter(0.0, 1.0), true},
            {std::nextafter(0.0, -1.0), true},
    }};
    for (const auto &[x, is_np] : xs)
        scalar(golden,
               numeric::np_interp_unit_grid(x, values, -9.5, 11.75),
               "len1 x=" +
                       (is_np ? np_f64(x) : repr_binary64(x)));
    golden.done();
}

void test_pneumatic_interp_multi_knot_bitwise() {
    auto golden = node(kFile, "test_pneumatic_interp_multi_knot_bitwise");
    const std::array values{2.0, -4.5, 8.25, -1.0};
    const std::array<std::pair<double, bool>, 17> xs{{
            {std::numeric_limits<double>::quiet_NaN(), false},
            {-std::numeric_limits<double>::quiet_NaN(), false},
            {0.0, false},
            {-0.0, false},
            {1.0, false},
            {2.0, false},
            {3.0, false},
            {0.25, false},
            {1.5, false},
            {2.999, false},
            {3.0000000001, false},
            {-1e-9, false},
            {4.0, false},
            {std::numeric_limits<double>::infinity(), false},
            {-std::numeric_limits<double>::infinity(), false},
            {std::nextafter(3.0, 0.0), true},
            {std::nextafter(0.0, 1.0), true},
    }};
    for (const auto &[x, is_np] : xs)
        scalar(golden,
               numeric::np_interp_unit_grid(x, values, -9.5, 11.75),
               "len4 x=" +
                       (is_np ? np_f64(x) : repr_binary64(x)));
    golden.done();
}

void test_pneumatic_interp_two_knot_and_empty() {
    auto golden = node(kFile, "test_pneumatic_interp_two_knot_and_empty");
    // Zero slope between equal knots.
    const std::array flat{1.5, 1.5};
    scalar(golden,
           numeric::np_interp_unit_grid(0.5, flat, -1.0, 1.0), "");
    // Two-element interpolation through the slope formula.
    const std::array slope{-3.0, 9.0};
    const std::array<std::pair<double, bool>, 4> xs{{
            {0.25, false},
            {0.5, false},
            {0.75, false},
            {std::nextafter(1.0, 0.0), true},
    }};
    for (const auto &[x, is_np] : xs)
        scalar(golden,
               numeric::np_interp_unit_grid(x, slope, -7.0, 7.0),
               "len2 x=" +
                       (is_np ? np_f64(x) : repr_binary64(x)));
    // Empty values raise on both sides — np.interp's ValueError.
    const std::span<const double> empty;
    rejects<std::invalid_argument>([&] {
        (void)numeric::np_interp_unit_grid(0.5, empty, 0.0, 1.0);
    });
    golden.done();
}

} // namespace

// bt::run wraps the test body in try/catch(...) — failures are
// reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        for (const std::int64_t n : kRays) {
            test_carcass_progressive_load_unload_bitwise(n, 0);
            test_carcass_progressive_load_unload_bitwise(n, 1);
            test_carcass_rim_strike_bitwise(n);
            test_carcass_airborne_landed_bitwise(n);
            test_carcass_advance_unloaded_sequence_bitwise(n);
        }
        for (const std::int64_t n : {3, 17, 64, 256})
            for (const double omega : {0.0, 3.7, -5.2, 1e-9})
                test_carcass_material_rate_bitwise(n, omega);
        for (const std::int64_t n : {3, 5, 64, 256})
            test_carcass_material_rate_edges(n);
        for (const double shift : {0.0, 0.3, -0.6, 1.0, -2.7, 3.4, -4.9,
                                   63.9, -64.1})
            for (const std::int64_t n : {3, 5, 17, 64, 256})
                test_carcass_advect_bitwise(shift, n);
        test_carcass_advect_zero_omega_copy();
        test_carcass_empty_rows();
        test_carcass_math_probes_bitwise();
        for (const std::int64_t n : kRays) {
            test_carcass_saturation_and_patch_shapes_bitwise(n);
            test_carcass_invalid_inputs_bitwise_atomicity(n);
            test_carcass_multistep_mixed_speed_spin_bitwise(n);
        }
        test_carcass_advance_unloaded_validation();
        test_carcass_state_construction_validation();
        test_carcass_first_step_initialized_branch();
        test_carcass_negative_zero_delta_and_inf_hits();
        test_pneumatic_interp_single_knot_bitwise();
        test_pneumatic_interp_multi_knot_bitwise();
        test_pneumatic_interp_two_knot_and_empty();
    });
}
