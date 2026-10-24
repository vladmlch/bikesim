// test_native_pneumatic_brush.cpp — C++ port of
// tests/reference/test_native_pneumatic_brush.py (batch 3.12a, T52).
//
// Replays relax / sigma_x / surface_mu / tread_stiffness_n_m2 /
// lumped_brush / DiscretisedBrush.step against the frozen two-platform
// goldens — result triples and post-call brush state pinned bitwise at
// every step. Rejection-class checks port the oracle's validation
// contract directly (no golden entries).
#include "pneumatic_reference_support.hpp"

namespace {

using namespace pneumatic_reference;
constexpr std::string_view kFile = "test_native_pneumatic_brush.py";

// ---------------------------------------------------------------------
// relax
// ---------------------------------------------------------------------

// test_relax_sweep_bitwise — a grid over all four return paths plus
// near-degenerate values; bitwise at every node.
void test_relax_sweep_bitwise() {
    auto golden = node(kFile, "test_relax_sweep_bitwise");
    bike::random::NumpyRng rng(0xE1A0);
    std::vector<double> kappas{0.0, -0.0, 0.15, -0.85, 2.5, -12.0};
    {
        std::array<double, 12> draws{};
        rng.uniform(-4.0, 4.0, draws);
        kappas.insert(kappas.end(), draws.begin(), draws.end());
    }
    constexpr std::array vxs{0.0, -0.0, 3.2, -7.5, 1e-9, 400.0};
    constexpr std::array vss{0.0, -0.0, 0.4, -2.2, 30.0};
    constexpr std::array sigmas{0.0, -0.0, 1e-6, 0.45, 2.0};
    constexpr std::array dts{0.0, 1e-4, 0.0005, 0.002, 0.25};
    for (const double kappa : kappas)
        for (const double vx : vxs)
            for (const double vs : vss)
                for (const double sigma : sigmas)
                    for (const double dt : dts)
                        scalar(golden,
                               biketyre::relax(kappa, vx, vs, sigma, dt),
                               "relax(" + repr_binary64(kappa) + "," +
                                       repr_binary64(vx) + "," +
                                       repr_binary64(vs) + "," +
                                       repr_binary64(sigma) + "," +
                                       repr_binary64(dt) + ")");
    golden.done();
}

// test_relax_extreme_and_transient_bitwise — overflowing products,
// exact-zero exponents and a chained transient.
void test_relax_extreme_and_transient_bitwise() {
    auto golden =
            node(kFile, "test_relax_extreme_and_transient_bitwise");
    const std::array<std::array<double, 5>, 9> cases{{
            // |v_x|*dt overflows to inf -> exponent -inf -> exp(-inf)=+0
            {0.5, 1e200, -3.0, 1e-300, 1e200},
            {-0.5, -1e200, 3.0, 1e-300, 1e200},
            // dt = 0 -> exponent -0.0 -> exp = 1.0
            {0.7, 3.0, -1.0, 0.4, 0.0},
            {0.7, -3.0, 1.0, 0.4, -0.0},
            // sigma huge -> exponent ~ -0
            {0.2, 4.0, 0.5, 1e300, 0.001},
            // parked, sigma > 0: linear integration, both vs signs
            {0.2, 0.0, 0.5, 0.4, 0.002},
            {0.2, -0.0, -0.5, 0.4, 0.002},
            // parked, sigma = 0: state holds regardless of vs
            {-0.9, 0.0, 5.0, 0.0, 0.002},
            // sigma = 0, moving: algebraic jump to -vs/|vx|
            {0.333, 2.0, 0.125, 0.0, 0.001},
    }};
    for (const auto &args : cases) {
        std::string label = "relax(";
        for (std::size_t i = 0; i < args.size(); ++i) {
            if (i != 0) label += ", ";
            label += repr_binary64(args[i]);
        }
        scalar(golden,
               biketyre::relax(args[0], args[1], args[2], args[3],
                               args[4]),
               label + ")");
    }

    // Chained transient: kappa' feeds forward like the model drives it.
    double kappa = 0.0;
    const std::array<std::array<double, 4>, 7> chain{{
            {4.0, -0.5, 0.4, 0.0005},
            {4.0, -0.9, 0.4, 0.0005},
            {4.0, 0.2, 0.4, 0.0005},
            {0.0, 0.0, 0.4, 0.0005},
            {0.0, -0.3, 0.4, 0.0005},
            {-2.0, 0.1, 0.4, 0.001},
            {6.5, 1.4, 0.0, 0.0005},
    }};
    for (const auto &[vx, vs, sigma, dt] : chain) {
        kappa = biketyre::relax(kappa, vx, vs, sigma, dt);
        scalar(golden, kappa,
               "transient (" + repr_binary64(vx) + "," +
                       repr_binary64(vs) + "," + repr_binary64(sigma) +
                       "," + repr_binary64(dt) + ")");
    }
    golden.done();
}

// test_relax_rejections[badN] [port] — finite gate first, then the
// nonnegative-length/dt gate.
void test_relax_rejections() {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double inf = std::numeric_limits<double>::infinity();
    // (kappa_prime, v_x, v_s, sigma, dt) — one bad field per row.
    const std::array<std::array<double, 5>, 13> cases{{
            {nan, 2.0, -0.3, 0.5, 0.001},
            {inf, 2.0, -0.3, 0.5, 0.001},
            {-inf, 2.0, -0.3, 0.5, 0.001},
            {0.1, nan, -0.3, 0.5, 0.001},
            {0.1, inf, -0.3, 0.5, 0.001},
            {0.1, 2.0, nan, 0.5, 0.001},
            {0.1, 2.0, -inf, 0.5, 0.001},
            {0.1, 2.0, -0.3, nan, 0.001},
            {0.1, 2.0, -0.3, inf, 0.001},
            {0.1, 2.0, -0.3, -1e-9, 0.001},
            {0.1, 2.0, -0.3, 0.5, nan},
            {0.1, 2.0, -0.3, 0.5, -inf},
            {0.1, 2.0, -0.3, 0.5, -1e-9},
    }};
    for (const auto &args : cases)
        rejects<std::invalid_argument>([&] {
            (void)biketyre::relax(args[0], args[1], args[2], args[3],
                                  args[4]);
        });
}

// test_relax_rejection_order [port] — a non-finite input beats a
// negative sigma; a negative sigma and a negative dt share the second
// message.
void test_relax_rejection_order() {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rejects<std::invalid_argument>(
            [] { (void)biketyre::relax(nan, 2.0, 0.0, -1.0, -1.0); },
            "finite");
    rejects<std::invalid_argument>(
            [] { (void)biketyre::relax(0.0, 2.0, 0.0, -1.0, 0.001); },
            "not be negative");
}

// ---------------------------------------------------------------------
// _sigma_x / _tread_stiffness_n_m2 / SurfaceSpec.mu probes
// ---------------------------------------------------------------------

// test_sigma_x_bitwise[kappa] — (value, pytest id, np-float64 flag):
// the two duplicated reprs carry pytest's _0/_1 dedup suffixes and the
// np.nextafter params the np.float64() label spelling.
void test_sigma_x_bitwise(double kappa, std::string_view id, bool is_np) {
    auto golden = node(kFile, "test_sigma_x_bitwise", id);
    scalar(golden, biketyre::sigma_x(kappa),
           "sigma_x(" +
                   (is_np ? np_f64(kappa) : repr_binary64(kappa)) + ")");
    golden.done();
}

// test_tread_stiffness_bitwise[tread-load-half-surfaceN]
void test_tread_stiffness_bitwise(std::size_t surface_index,
                                  std::pair<double, double> load_half,
                                  const std::optional<double> &tread,
                                  std::string_view tread_str,
                                  std::string_view load_str,
                                  std::string_view half_str) {
    const auto [load, half] = load_half;
    auto golden = node(
            kFile, "test_tread_stiffness_bitwise",
            std::string(tread_str) + "-" + std::string(load_str) + "-" +
                    std::string(half_str) + "-surface" +
                    std::to_string(surface_index));
    const auto spec = surface_at(surface_index);
    scalar(golden,
           biketyre::tread_stiffness_n_m2(load, half, spec, tread),
           spec.name + " " + std::string(load_str) + " " +
                   std::string(half_str) + " " + std::string(tread_str));
    golden.done();
}

// test_tread_stiffness_rejections [port]
void test_tread_stiffness_rejections() {
    const auto spec = surface("asphalt");
    for (const double tread :
         {std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(), -1.0})
        rejects<std::invalid_argument>([&] {
            (void)biketyre::tread_stiffness_n_m2(400.0, 0.05, spec,
                                                 tread);
        });
}

// test_surface_mu_bitwise[surfaceN] — the Stribeck curve plus
// saturation at large |v|.
void test_surface_mu_bitwise(std::size_t surface_index) {
    const auto spec = surface_at(surface_index);
    auto golden = node(kFile, "test_surface_mu_bitwise",
                       "surface" + std::to_string(surface_index));
    const auto first = np_linspace(0.0, 12.0, 121);
    const auto second = np_linspace(-12.0, 0.0, 60);
    std::vector<double> speeds;
    speeds.reserve(first.size() + second.size() + 6);
    speeds.insert(speeds.end(), first.begin(), first.end());
    speeds.insert(speeds.end(), second.begin(), second.end());
    for (const double v :
         {-0.0, 1e-9, 4.5, -4.5, 1e3, 1e30})
        speeds.push_back(v);
    for (const double v : speeds)
        scalar(golden, biketyre::surface_mu(spec, v),
               spec.name + ".mu(" + np_f64(v) + ")");
    golden.done();
}

// test_surface_mu_rejections [port]
void test_surface_mu_rejections() {
    const auto spec = surface("wet");
    for (const double v :
         {std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()})
        rejects<std::invalid_argument>(
                [&] { (void)biketyre::surface_mu(spec, v); });
}

// ---------------------------------------------------------------------
// lumped_brush
// ---------------------------------------------------------------------

// test_lumped_brush_sweep_bitwise[surfaceN] — slip sweep through the
// cubic region into saturation.
void test_lumped_brush_sweep_bitwise(std::size_t surface_index) {
    const auto spec = surface_at(surface_index);
    auto golden = node(kFile, "test_lumped_brush_sweep_bitwise",
                       "surface" + std::to_string(surface_index));
    auto kappas = np_linspace(-1.05, 1.05, 43);
    for (const double k :
         {-0.0, std::nextafter(-1.0, 0.0), -1.0,
          std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(), 1e6, -1e6})
        kappas.push_back(k);
    const std::array<
            std::tuple<double, double, std::optional<double>, double>,
            7>
            configs{{
                    {0.0, 0.05, std::nullopt, 0.0},
                    {1e-9, 0.03, std::nullopt, 1.0},
                    {140.0, 0.021, std::nullopt, 0.0},
                    {520.0, 0.052, std::nullopt, 3.7},
                    {520.0, 0.052, 4.2e4, 0.0},
                    {520.0, 0.052, 0.0, 8.0},
                    {7.0, 1e-6, std::nullopt, 0.25},
            }};
    for (const double kappa : kappas)
        for (const auto &[load, half, tread, slide] : configs) {
            const auto result = biketyre::lumped_brush(
                    kappa, load, half, spec, tread, slide);
            brush_result(golden, result,
                         spec.name + " k=" + np_f64(kappa) +
                                 " F=" + repr_binary64(load));
        }
    golden.done();
}

// test_lumped_brush_stribeck_speeds_bitwise — sliding_speed sweeps
// decay mu from peak towards slide.
void test_lumped_brush_stribeck_speeds_bitwise() {
    auto golden =
            node(kFile, "test_lumped_brush_stribeck_speeds_bitwise");
    const auto spec = surface("loose");
    for (const double slide :
         {0.0, -0.0, 0.2, 2.0, 4.5, 9.0, 60.0, 1e6})
        for (const double kappa :
             {0.05, -0.4, std::numeric_limits<double>::infinity()}) {
            const auto result = biketyre::lumped_brush(
                    kappa, 300.0, 0.04, spec, std::nullopt, slide);
            brush_result(golden, result,
                         "slide=" + repr_binary64(slide));
        }
    golden.done();
}

// test_lumped_brush_early_returns_and_edges — zero load, c_px == 0
// via explicit zero stiffness and huge half-length underflow,
// nan-z propagation and the z==±inf saturation boundary.
void test_lumped_brush_early_returns_and_edges() {
    auto golden =
            node(kFile, "test_lumped_brush_early_returns_and_edges");
    const auto spec = surface("asphalt");
    // load == 0 wins over an invalid half-length (no raise).
    brush_result(golden,
                 biketyre::lumped_brush(0.3, 0.0, -1.0, spec,
                                        std::nullopt, 0.0),
                 "zero load bad half");
    // Explicit zero stiffness -> (0, False, mu) even mid-range.
    brush_result(golden,
                 biketyre::lumped_brush(0.4, 400.0, 0.05, spec, 0.0,
                                        0.0),
                 "c_px==0 explicit");
    // half**2 overflows: CPython float_pow raises OverflowError at the
    // `** 2` — the port throws std::overflow_error, the same class the
    // binding maps back.
    for (const std::optional<double> tread :
         {std::optional<double>{std::nullopt},
          std::optional<double>{5.0e4}})
        rejects<std::overflow_error>([&] {
            (void)biketyre::lumped_brush(0.4, 400.0, 1e200, spec, tread,
                                         0.0);
        });
    // kappa <= -1 exact: the saturation copysign boundary.
    const auto result = biketyre::lumped_brush(-1.0, 300.0, 0.05, spec,
                                               std::nullopt, 0.0);
    brush_result(golden, result, "kappa=-1 locked");
    bt::require(result.fully_sliding, "kappa=-1 fully slides");
    golden.done();
}

// test_lumped_brush_validation_order [port] — the staged finite gate:
// kappa must be *infinite* to defer to the load/half/slide gate.
void test_lumped_brush_validation_order() {
    const auto spec = surface("hardpack");
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double inf = std::numeric_limits<double>::infinity();
    // (kappa, load, half, tread, slide) rows over base
    // (400.0, 0.05, 1.0).
    const std::array<
            std::tuple<double, double, double, std::optional<double>,
                       double>,
            13>
            cases{{
                    {nan, 400.0, 0.05, std::nullopt, 1.0},
                    {nan, inf, 0.05, std::nullopt, 1.0},
                    {inf, inf, 0.05, std::nullopt, 1.0},
                    {-inf, 400.0, nan, std::nullopt, 1.0},
                    {inf, 400.0, 0.05, std::nullopt, nan},
                    {inf, -3.0, 0.05, std::nullopt, 1.0},
                    {0.2, -3.0, 0.05, std::nullopt, 1.0},
                    {0.2, 400.0, 0.0, std::nullopt, 1.0},
                    {0.2, 400.0, -0.5, std::nullopt, 1.0},
                    {0.2, 400.0, 0.05, nan, 1.0},
                    {0.2, 400.0, 0.05, inf, 1.0},
                    {0.2, 400.0, 0.05, -2.0, 1.0},
                    {inf, 400.0, 0.05, nan, 1.0},
            }};
    for (const auto &[kappa, load, half, tread, slide] : cases)
        rejects<std::invalid_argument>([&] {
            (void)biketyre::lumped_brush(kappa, load, half, spec, tread,
                                         slide);
        });
}

// test_lumped_brush_inf_kappa_zero_load — inf kappa + zero load
// returns cleanly; loaded +/-inf saturate with their sign.
void test_lumped_brush_inf_kappa_zero_load() {
    auto golden = node(kFile, "test_lumped_brush_inf_kappa_zero_load");
    const auto spec = surface("scree");
    brush_result(golden,
                 biketyre::lumped_brush(
                         std::numeric_limits<double>::infinity(), 0.0,
                         -1.0, spec, std::nullopt, 0.0),
                 "inf kappa zero load");
    for (const double kappa :
         {std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()}) {
        const auto result = biketyre::lumped_brush(
                kappa, 250.0, 0.05, spec, std::nullopt, 0.0);
        brush_result(golden, result,
                     "kappa=" + repr_binary64(kappa));
        bt::require(result.fully_sliding,
                    "infinite kappa fully slides");
    }
    golden.done();
}

// ---------------------------------------------------------------------
// DiscretisedBrush.step — single-step coverage
// ---------------------------------------------------------------------

// test_discretised_step_sweep_bitwise[surfaceN-n]
void test_discretised_step_sweep_bitwise(std::size_t surface_index,
                                         std::int64_t n) {
    const auto spec = surface_at(surface_index);
    auto golden = node(kFile, "test_discretised_step_sweep_bitwise",
                       "surface" + std::to_string(surface_index) + "-" +
                               std::to_string(n));
    bike::random::NumpyRng rng(static_cast<std::uint64_t>(
            n * 131 +
            static_cast<std::int64_t>(spec.mu_peak * 1000)));
    biketyre::DiscretisedBrush brush(n);
    const auto width = static_cast<std::size_t>(n);
    for (std::size_t step_idx = 0; step_idx < 6; ++step_idx) {
        const std::string tag = "step " + std::to_string(step_idx);
        std::vector<double> loads(width), mask_draw(width);
        rng.uniform(0.0, 900.0, loads);
        rng.uniform(0.0, 1.0, mask_draw);
        bool any_load = false;
        for (std::size_t i = 0; i < width; ++i) {
            if (mask_draw[i] < 0.25) loads[i] = 0.0;
            if (loads[i] > 0.0) any_load = true;
        }
        if (!any_load)
            loads[static_cast<std::size_t>(
                    rng.integers(0, n))] = 10.0;
        // The nonzero-low uniform transforms are wheel-contraction
        // divergent — the draws still run to keep the stream aligned,
        // but the frozen scalars win the bits (sigma's zero-low draw is
        // stable and stays regenerated).
        (void)rng.uniform(0.004, 0.09);
        (void)rng.uniform(-9.0, 9.0);
        (void)rng.uniform(-3.0, 3.0);
        const double sigma = rng.uniform(0.0, 0.8);
        (void)rng.uniform(0.0002, 0.004);
        const auto replay =
                golden.f64_input("input." + tag + " scalars");
        const auto result =
                brush.step(loads, replay[0], replay[1], replay[2],
                           sigma, spec, replay[3]);
        brush_result(golden, result, tag);
        brush_state(golden, brush, tag);
    }
    golden.done();
}

// test_discretised_multi_step_evolution_bitwise — rolling -> harder
// slip -> stop -> parked hold -> reverse.
void test_discretised_multi_step_evolution_bitwise() {
    auto golden =
            node(kFile, "test_discretised_multi_step_evolution_bitwise");
    constexpr std::int64_t n = 12;
    const auto spec = surface("hardpack");
    bike::random::NumpyRng rng(0xD15C);
    auto base_loads = np_linspace(60.0, 340.0, 12);
    base_loads[0] = 0.0; // inactive leading bristles
    base_loads[1] = 0.0;
    base_loads[11] = 0.0; // inactive trailing bristle
    constexpr double half = 0.045;
    const std::array<std::array<double, 4>, 14> program{{
            {4.0, -0.30, 0.50, 0.0005},
            {4.0, -0.30, 0.50, 0.0005},
            {4.0, -0.85, 0.50, 0.0005},
            {4.2, -1.60, 0.50, 0.0005},
            {3.0, -2.60, 0.50, 0.0005},
            {0.0, -0.10, 0.50, 0.0005},
            {0.0, 0.00, 0.50, 0.0005},
            {0.0, 0.40, 0.50, 0.0010},
            {-2.5, 0.35, 0.50, 0.0005},
            {-2.5, 0.60, 0.50, 0.0005},
            {4.0, 4.00, 0.50, 0.0005},
            {4.0, -0.30, 0.02, 0.0005},
            {4.0, -0.30, 0.045, 0.0005},
            {4.0, -0.30, 0.50, 0.0005},
    }};
    biketyre::DiscretisedBrush brush(n);
    for (std::size_t idx = 0; idx < program.size(); ++idx) {
        const auto &[v_x, v_s, sigma, dt] = program[idx];
        std::vector<double> jitter(12);
        rng.uniform(-1.0, 1.0, jitter);
        std::vector<double> loads(12);
        for (std::size_t i = 0; i < 12; ++i)
            loads[i] = base_loads[i] * (1.0 + 0.05 * jitter[i]);
        const std::string tag = "step " + std::to_string(idx);
        const auto result = brush.step(loads, half, v_x, v_s, sigma,
                                       spec, dt);
        brush_result(golden, result, tag);
        brush_state(golden, brush, tag);
    }
    golden.done();
}

// test_discretised_advection_extremes_bitwise — shifts out of the
// window on each side, fractional bins, exact knots and the
// zero-wheel-speed copy.
void test_discretised_advection_extremes_bitwise() {
    auto golden =
            node(kFile, "test_discretised_advection_extremes_bitwise");
    constexpr std::int64_t n = 7;
    const auto spec = surface("asphalt");
    bike::random::NumpyRng rng(77);
    std::vector<double> bristles(7);
    rng.uniform(-0.02, 0.02, bristles);
    bristles[0] = -0.0; // signed-zero payload survives the copy
    // Wheel-contraction divergent — the frozen row wins the bits.
    bristles = golden.f64_input("input.bristles");
    const auto loads = np_linspace(50.0, 300.0, 7);
    constexpr double half = 0.035; // dx = 0.07/7 = 0.01
    const std::array<std::array<double, 3>, 7> cases{{
            {4.0, -0.3, 0.0025},
            {-4.0, 0.3, 0.0025},
            {20.0, -1.0, 0.01},
            {-20.0, 1.0, 0.01},
            {3.0, 3.0, 0.0025},
            {0.0, 0.0, 0.0025},
            {1e300, 0.0, 1e-2},
    }};
    for (const auto &[v_x, v_s, dt] : cases) {
        biketyre::DiscretisedBrush brush(n);
        std::ranges::copy(bristles,
                          brush.bristle_deflection_m().begin());
        brush.restore_kappa_prime(0.11);
        const auto result = brush.step(loads, half, v_x, v_s, 0.5,
                                       spec, dt);
        const std::string tag = "advect v_x=" + repr_binary64(v_x) +
                                " v_s=" + repr_binary64(v_s);
        brush_result(golden, result, tag);
        brush_state(golden, brush, tag);
    }
    golden.done();
}

// test_discretised_clip_boundary_bitwise — |trial| == limit exactly:
// the sliding mask's >= edge and both bristles saturating together.
void test_discretised_clip_boundary_bitwise() {
    auto golden =
            node(kFile, "test_discretised_clip_boundary_bitwise");
    constexpr std::int64_t n = 2;
    const std::vector<double> loads{1.0, 1.0};
    constexpr double half = 0.5;
    {
        biketyre::DiscretisedBrush brush(n);
        const std::array bristles{1.0, -1.0};
        std::ranges::copy(bristles,
                          brush.bristle_deflection_m().begin());
        const auto result = brush.step(loads, half, 0.0, 0.0, 1.0,
                                       kUnitSurface, 0.001);
        brush_result(golden, result, "clip boundary");
        bt::require(result.fully_sliding,
                    "the exact-boundary bristles must slide");
        brush_state(golden, brush, "clip boundary");
    }
    // One bristle exactly at the limit, the other just inside ->
    // partial sliding (fully_sliding False), still bitwise.
    {
        biketyre::DiscretisedBrush brush(n);
        const std::array bristles{1.0, std::nextafter(-1.0, 0.0)};
        std::ranges::copy(bristles,
                          brush.bristle_deflection_m().begin());
        const auto result = brush.step(loads, half, 0.0, 0.0, 1.0,
                                       kUnitSurface, 0.001);
        brush_result(golden, result, "partial clip boundary");
        bt::require(!result.fully_sliding,
                    "one inside bristle keeps partial sliding");
        brush_state(golden, brush, "partial clip boundary");
    }
    golden.done();
}

// test_discretised_clip_sign_and_zero_edges — clip's operand order:
// force at the -limit side and signed-zero trial/limit payloads.
void test_discretised_clip_sign_and_zero_edges() {
    auto golden =
            node(kFile, "test_discretised_clip_sign_and_zero_edges");
    constexpr std::int64_t n = 3;
    const std::vector<double> loads{1.0, 1.0, 1.0};
    biketyre::DiscretisedBrush brush(n);
    const std::array bristles{5.0, -5.0, -0.0};
    std::ranges::copy(bristles,
                      brush.bristle_deflection_m().begin());
    const auto result = brush.step(loads, 0.5, 0.0, 0.0, 1.0,
                                   kUnitSurface, 0.001);
    brush_result(golden, result, "clip signs");
    brush_state(golden, brush, "clip signs");
    // The saturated bristles rewrite to (force - damping)/k = ±1.0.
    vector(golden, brush.bristle_deflection_m(), "write-back");
    golden.done();
}

// test_discretised_zero_load_reset_bitwise — total_load <= 0 or
// half_length <= 0 resets state and returns (0, False, mu).
void test_discretised_zero_load_reset_bitwise() {
    auto golden =
            node(kFile, "test_discretised_zero_load_reset_bitwise");
    constexpr std::int64_t n = 6;
    const auto spec = surface("wet");
    const std::vector<double> quiet(6, 0.0);
    const auto loaded = np_linspace(1.0, 3.0, 6);
    const std::array<std::pair<std::span<const double>, double>, 3>
            cases{{
                    {quiet, 0.04}, // all-inactive patch
                    {loaded, 0.0}, // half == 0 -> reset path
                    {loaded, -0.0},
            }};
    for (const auto &[loads, half] : cases) {
        biketyre::DiscretisedBrush brush(n);
        // Prime the state so the reset has something to clear.
        const std::vector<double> prime(6, 10.0);
        (void)brush.step(prime, 0.04, 3.0, -0.5, 0.4, spec, 0.001);
        bt::require(std::ranges::any_of(
                            brush.bristle_deflection_m(),
                            [](const double v) { return v != 0.0; }),
                    "primed bristle state is nonzero");
        const auto result = brush.step(loads, half, 3.0, -0.5, 0.4,
                                       spec, 0.001);
        const std::string tag = "reset half=" + repr_binary64(half);
        brush_result(golden, result, tag);
        brush_state(golden, brush, tag);
        vector(golden, brush.bristle_deflection_m(),
               "reset clears bristles");
        scalar(golden, brush.kappa_prime(), "reset clears kappa");
    }
    golden.done();
}

// test_discretised_state_nan_propagation — a NaN bristle seeded into
// the state propagates through the unit-grid interp and the clip.
void test_discretised_state_nan_propagation() {
    auto golden =
            node(kFile, "test_discretised_state_nan_propagation");
    constexpr std::int64_t n = 5;
    const auto spec = surface("asphalt");
    const std::array bristles{0.01,
                              std::numeric_limits<double>::quiet_NaN(),
                              -0.02, 0.015, 0.0};
    const auto loads = np_linspace(80.0, 240.0, 5);
    biketyre::DiscretisedBrush brush(n);
    std::ranges::copy(bristles,
                      brush.bristle_deflection_m().begin());
    brush.restore_kappa_prime(0.05);
    const std::array<std::pair<double, double>, 3> kinematics{{
            {4.0, -0.3},
            {0.0, 0.0},
            {-3.0, 0.2},
    }};
    for (std::size_t idx = 0; idx < kinematics.size(); ++idx) {
        const auto &[v_x, v_s] = kinematics[idx];
        const auto result = brush.step(loads, 0.05, v_x, v_s, 0.5,
                                       spec, 0.001);
        const std::string tag =
                "nan bristle step " + std::to_string(idx);
        brush_result(golden, result, tag);
        brush_state(golden, brush, tag);
    }
    golden.done();
}

// ---------------------------------------------------------------------
// DiscretisedBrush.step — validation order and construction
// ---------------------------------------------------------------------

// test_discretised_validation_order_and_atomicity — every rejection
// class in the oracle's check order; failed calls leave state
// untouched and the follow-up valid step matches bitwise.
void test_discretised_validation_order_and_atomicity() {
    auto golden = node(
            kFile, "test_discretised_validation_order_and_atomicity");
    constexpr std::int64_t n = 8;
    const auto spec = surface("hardpack");
    const auto loads = np_linspace(40.0, 300.0, 8);
    biketyre::DiscretisedBrush brush(n);
    // Nonzero state to lose.
    (void)brush.step(loads, 0.04, 4.0, -0.3, 0.5, spec, 0.001);

    // Level 1 — loads shape.
    const auto short_loads =
            std::span<const double>(loads).subspan(0, loads.size() - 1);
    rejects<std::invalid_argument>([&] {
        (void)brush.step(short_loads, 0.04, 4.0, -0.3, 0.5, spec,
                         0.001);
    });
    brush_state(golden, brush, "shape rejection atomicity");

    // Level 2 — loads content (finite + nonnegative share one
    // message).
    for (const double bad :
         {std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity(), -1e-9,
          -std::numeric_limits<double>::infinity()}) {
        auto row = loads;
        row[3] = bad;
        rejects<std::invalid_argument>([&] {
            (void)brush.step(row, 0.04, 4.0, -0.3, 0.5, spec, 0.001);
        });
    }
    brush_state(golden, brush, "loads rejection atomicity");

    // Level 3 — scalar finite gate beats the domain gate below it.
    const std::array<std::array<double, 5>, 6> finite_cases{{
            // (half, v_x, v_s, sigma, dt)
            {std::numeric_limits<double>::quiet_NaN(), 4.0, -0.3, 0.5,
             0.001},
            {0.04, std::numeric_limits<double>::infinity(), -0.3, 0.5,
             0.001},
            {0.04, 4.0, -std::numeric_limits<double>::infinity(), 0.5,
             0.001},
            {0.04, 4.0, -0.3,
             std::numeric_limits<double>::quiet_NaN(), 0.001},
            {0.04, 4.0, -0.3, 0.5,
             std::numeric_limits<double>::infinity()},
            {std::numeric_limits<double>::quiet_NaN(), 4.0, -0.3, -1.0,
             0.0},
    }};
    for (const auto &[half, v_x, v_s, sigma, dt] : finite_cases)
        rejects<std::invalid_argument>([&] {
            (void)brush.step(loads, half, v_x, v_s, sigma, spec, dt);
        });

    // Level 4 — domain gate: half < 0, sigma < 0, dt <= 0.
    const std::array<std::array<double, 5>, 5> domain_cases{{
            {-1e-9, 4.0, -0.3, 0.5, 0.001},
            {0.04, 4.0, -0.3, -0.25, 0.001},
            {0.04, 4.0, -0.3, 0.5, 0.0},
            {0.04, 4.0, -0.3, 0.5, -0.0},
            {0.04, 4.0, -0.3, 0.5, -1.0},
    }};
    for (const auto &[half, v_x, v_s, sigma, dt] : domain_cases)
        rejects<std::invalid_argument>([&] {
            (void)brush.step(loads, half, v_x, v_s, sigma, spec, dt);
        });
    brush_state(golden, brush, "domain rejection atomicity");

    // A non-finite kappa_prime injected into the state reaches
    // relax's own finite gate — after every step-level check.
    const double saved_kappa = brush.kappa_prime();
    brush.restore_kappa_prime(
            std::numeric_limits<double>::quiet_NaN());
    rejects<std::invalid_argument>([&] {
        (void)brush.step(loads, 0.04, 4.0, -0.3, 0.5, spec, 0.001);
    });
    // The oracle restores kappa_prime after the raise; the C++ object
    // must do the same before the follow-up step so the comparison
    // replays the same state.
    brush.restore_kappa_prime(saved_kappa);

    // The follow-up valid step still matches bitwise.
    const auto result = brush.step(loads, 0.04, 4.0, -0.3, 0.5, spec,
                                   0.001);
    brush_result(golden, result, "post-failure");
    brush_state(golden, brush, "post-failure");
    golden.done();
}

// test_discretised_pow_overflow_mid_step_commit — half=1e200 raises
// OverflowError at the c_px line AFTER kappa_prime committed through
// relax() but before the bristle commit.
void test_discretised_pow_overflow_mid_step_commit() {
    auto golden = node(
            kFile, "test_discretised_pow_overflow_mid_step_commit");
    constexpr std::int64_t n = 4;
    const auto spec = surface("asphalt");
    const auto loads = np_linspace(60.0, 200.0, 4);
    biketyre::DiscretisedBrush brush(n);
    (void)brush.step(loads, 0.04, 3.0, -0.2, 0.4, spec, 0.001);
    const std::vector<double> before_bristles(
            brush.bristle_deflection_m().begin(),
            brush.bristle_deflection_m().end());
    const double before_kappa = brush.kappa_prime();
    rejects<std::overflow_error>([&] {
        (void)brush.step(loads, 1e200, 3.0, -0.2, 0.4, spec, 0.001);
    });
    // bristles byte-identical; kappa_prime advanced through relax.
    vector(golden, brush.bristle_deflection_m(), "bristles untouched");
    bt::require(brush.bristle_deflection_m().size() ==
                        before_bristles.size() &&
                    std::ranges::equal(brush.bristle_deflection_m(),
                                       before_bristles),
            "bristles byte-identical");
    bt::require(brush.kappa_prime() != before_kappa,
                "kappa_prime advanced before the pow raise");
    scalar(golden, brush.kappa_prime(),
           "kappa committed before the pow raise");
    golden.done();
}

// test_discretised_construction_rejection [port] — n_elements < 1
// raises (through the state dict intake for the native kernel, which
// funnels into the same ctor).
void test_discretised_construction_rejection() {
    rejects<std::invalid_argument>(
            [] { biketyre::DiscretisedBrush empty(0); },
            "at least 1 element");
}

// test_discretised_partial_inactive_patch_bitwise — a lopsided patch:
// only interior bristles loaded.
void test_discretised_partial_inactive_patch_bitwise() {
    auto golden =
            node(kFile, "test_discretised_partial_inactive_patch_bitwise");
    constexpr std::int64_t n = 9;
    const auto spec = surface("loose");
    std::vector<double> loads(9, 0.0);
    loads[3] = 120.0;
    loads[4] = 240.0;
    loads[5] = 90.0;
    biketyre::DiscretisedBrush brush(n);
    const std::array<std::array<double, 3>, 5> cases{{
            {5.0, -0.4, 0.0005},
            {5.0, -1.2, 0.0005},
            {5.0, -4.0, 0.0005},
            {-3.0, 0.8, 0.0005},
            {0.0, 0.0, 0.0005},
    }};
    for (std::size_t idx = 0; idx < cases.size(); ++idx) {
        const auto &[v_x, v_s, dt] = cases[idx];
        const auto result = brush.step(loads, 0.05, v_x, v_s, 0.6,
                                       spec, dt);
        const std::string tag = "partial " + std::to_string(idx);
        brush_result(golden, result, tag);
        brush_state(golden, brush, tag);
    }
    // Hard slip: every active bristle saturates -> fully_sliding.
    const auto result = brush.step(loads, 0.05, 5.0, -30.0, 0.6, spec,
                                   0.02);
    brush_result(golden, result, "partial full-slide");
    bt::require(result.fully_sliding,
                "every active bristle saturates");
    brush_state(golden, brush, "partial full-slide");
    golden.done();
}

// test_discretised_single_element_bitwise — n=1: interp's j == n-1
// last-knot branch and the single-element sliding decision.
void test_discretised_single_element_bitwise() {
    auto golden =
            node(kFile, "test_discretised_single_element_bitwise");
    const auto spec = surface("asphalt");
    biketyre::DiscretisedBrush brush(1);
    const std::array<std::array<double, 4>, 5> cases{{
            {4.0, -0.3, 0.5, 0.0005},
            {4.0, -3.0, 0.5, 0.001},
            {0.0, 0.2, 0.5, 0.0005},
            {-2.0, 0.5, 0.0, 0.0005},
            {8.0, -9.0, 0.5, 0.002},
    }};
    for (std::size_t idx = 0; idx < cases.size(); ++idx) {
        const auto &[v_x, v_s, sigma, dt] = cases[idx];
        const std::vector<double> loads{260.0};
        const auto result = brush.step(loads, 0.03, v_x, v_s, sigma,
                                       spec, dt);
        const std::string tag = "n=1 step " + std::to_string(idx);
        brush_result(golden, result, tag);
        brush_state(golden, brush, tag);
    }
    golden.done();
}

// test_discretised_zero_kappa_inf_products — damping/trial arithmetic
// where filtered_v_s and dt make the growth exact zero, keeping IEEE
// -0.0 payloads bitwise.
void test_discretised_zero_kappa_inf_products() {
    auto golden =
            node(kFile, "test_discretised_zero_kappa_inf_products");
    constexpr std::int64_t n = 4;
    const auto spec = surface("wet");
    const auto loads = np_linspace(50.0, 150.0, 4);
    biketyre::DiscretisedBrush brush(n);
    const std::array<std::array<double, 3>, 5> cases{{
            {4.0, 0.0, 0.0},
            {4.0, -0.0, 0.0},
            {0.0, -0.0, 0.5},
            {0.0, 0.0, 0.5},
            {-4.0, -0.0, 0.3},
    }};
    for (const auto &[v_x, v_s, sigma] : cases) {
        const auto result = brush.step(loads, 0.04, v_x, v_s, sigma,
                                       spec, 0.001);
        const std::string tag = "v_x=" + repr_binary64(v_x) +
                                " v_s=" + repr_binary64(v_s);
        brush_result(golden, result, tag);
        brush_state(golden, brush, tag);
    }
    golden.done();
}

} // namespace

// bt::run wraps the test body in try/catch(...) — failures are
// reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        test_relax_sweep_bitwise();
        test_relax_extreme_and_transient_bitwise();
        test_relax_rejections();
        test_relax_rejection_order();
        // (kappa, pytest id, is_np_float64) — list order with the two
        // dedup suffixes preserved.
        const std::array<std::tuple<double, std::string_view, bool>,
                         18>
                sigma_cases{{
                        {0.0, "0.0", false},
                        {-0.0, "-0.0", false},
                        {0.25, "0.25", false},
                        {-0.5, "-0.5", false},
                        {0.9999999999999999, "0.9999999999999999",
                         false},
                        {1.0, "1.0", false},
                        {5.0, "5.0", false},
                        {1e300, "1e+300", false},
                        {-0.9999999999999999,
                         "-0.9999999999999999_0", false},
                        {-1.0, "-1.0", false},
                        {-1.0000000000000002,
                         "-1.0000000000000002_0", false},
                        {-4.0, "-4.0", false},
                        {-1e300, "-1e+300", false},
                        {std::numeric_limits<double>::infinity(),
                         "inf", false},
                        {-std::numeric_limits<double>::infinity(),
                         "-inf", false},
                        {std::numeric_limits<double>::quiet_NaN(),
                         "nan", false},
                        {std::nextafter(-1.0, -2.0),
                         "-1.0000000000000002_1", true},
                        {std::nextafter(-1.0, 0.0),
                         "-0.9999999999999999_1", true},
                }};
        for (const auto &[kappa, id, is_np] : sigma_cases)
            test_sigma_x_bitwise(kappa, id, is_np);
        // tread_stiffness: treads (outermost param id first), then
        // (load, half) pairs, then surfaces.
        const std::array<std::pair<std::optional<double>,
                                   std::string_view>,
                         4>
                treads{{
                        {std::nullopt, "None"},
                        {0.0, "0.0"},
                        {3.4e4, "34000.0"},
                        {1e-12, "1e-12"},
                }};
        const std::array<std::array<std::string_view, 3>, 5>
                load_half{{
                        {"0.0", "0.05", "x"},
                        {"350.0", "0.041", "x"},
                        {"1e-09", "0.02", "x"},
                        {"800.0", "0.005", "x"},
                        {"5.0", "1e-09", "x"},
                }};
        const std::array<std::pair<double, double>, 5> load_half_v{{
                {0.0, 0.05},
                {350.0, 0.041},
                {1e-9, 0.02},
                {800.0, 0.005},
                {5.0, 1e-9},
        }};
        for (std::size_t t = 0; t < treads.size(); ++t)
            for (std::size_t lh = 0; lh < load_half.size(); ++lh)
                for (std::size_t s = 0; s < 5; ++s)
                    test_tread_stiffness_bitwise(
                            s, load_half_v[lh], treads[t].first,
                            treads[t].second, load_half[lh][0],
                            load_half[lh][1]);
        test_tread_stiffness_rejections();
        for (std::size_t s = 0; s < 5; ++s)
            test_surface_mu_bitwise(s);
        test_surface_mu_rejections();
        for (std::size_t s = 0; s < 5; ++s)
            test_lumped_brush_sweep_bitwise(s);
        test_lumped_brush_stribeck_speeds_bitwise();
        test_lumped_brush_early_returns_and_edges();
        test_lumped_brush_validation_order();
        test_lumped_brush_inf_kappa_zero_load();
        for (std::size_t s = 0; s < 5; ++s)
            for (const std::int64_t n : {1, 2, 3, 5, 9, 17, 64})
                test_discretised_step_sweep_bitwise(s, n);
        test_discretised_multi_step_evolution_bitwise();
        test_discretised_advection_extremes_bitwise();
        test_discretised_clip_boundary_bitwise();
        test_discretised_clip_sign_and_zero_edges();
        test_discretised_zero_load_reset_bitwise();
        test_discretised_state_nan_propagation();
        test_discretised_validation_order_and_atomicity();
        test_discretised_pow_overflow_mid_step_commit();
        test_discretised_construction_rejection();
        test_discretised_partial_inactive_patch_bitwise();
        test_discretised_single_element_bitwise();
        test_discretised_zero_kappa_inf_products();
    });
}
