// pneumatic_reference_support.hpp — shared plumbing for the batch-3.12a
// pneumatic kernel reference ports:
//
//   tests/reference/test_native_pneumatic_geometry.py
//   tests/reference/test_native_pneumatic_carcass.py
//   tests/reference/test_native_pneumatic_brush.py
//
// Everything here is test harness: golden-node addressing, entry consumers
// matching the capture-side `_golden.record` calls the oracle emits, the
// NumpyRng-driven input syntheses the Python corpus builds
// (np.linspace/rng.uniform/rng.integers in source order), the
// SurfaceSpec/TyreSpecs adapters and the exception-class checks. No kernel
// behavior is duplicated — every comparison replays the production
// biketyre:: implementations captured by the two-platform oracle goldens.
#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "support/golden.hpp"
#include "support/migration_test.hpp"

#include "config_types.hpp"
#include "geometry/specs.hpp"
#include "io/float_format.hpp"
#include "legacy/tyre.hpp"
#include "numeric_numpy.hpp"
#include "numeric_sincos.hpp"
#include "pneumatic/brush.hpp"
#include "pneumatic/carcass.hpp"
#include "pneumatic/geometry.hpp"
#include "random/numpy_compat.hpp"
#include "terrain/types.hpp"

namespace pneumatic_reference {

// The shared corpus constants from the three oracle files.
inline constexpr double kRad = std::numbers::pi / 180.0;
inline constexpr double kHalfDeg = 75.0;
inline constexpr double kHalf = kHalfDeg * kRad;
inline constexpr double kRadius = 0.33;
inline constexpr std::array<std::int64_t, 2> kRays{64, 256};

// ---------------------------------------------------------------------
// Golden node addressing + entry consumers
// ---------------------------------------------------------------------

// bt::oracle::Node("tests/reference/<file>::<function>[<parameters>]") —
// the exact pytest node id the capture recorded.
[[nodiscard]] inline bt::oracle::Node node(std::string_view file,
                                           std::string_view function,
                                           std::string_view parameters = {}) {
    std::string name = "tests/reference/" + std::string(file) +
                       "::" + std::string(function);
    if (!parameters.empty()) name += "[" + std::string(parameters) + "]";
    return bt::oracle::Node(name);
}

// assert_bitwise_equal(scalar, ...) — a 0-d <f8 entry.
inline void scalar(bt::oracle::Node &golden, double actual,
                   std::string_view label) {
    const std::array value{actual};
    golden.f64(value, std::span<const std::size_t>{}, label);
}

// assert_bitwise_equal(row, ...) — a 1-d <f8 entry.
inline void vector(bt::oracle::Node &golden, std::span<const double> actual,
                   std::string_view label) {
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.f64(actual, shape, label);
}

// _golden.record(np.asarray(bool), ...) — a 0-d |b1 byte entry.
inline void boolean(bt::oracle::Node &golden, bool actual,
                    std::string_view label) {
    const std::array<std::byte, 1> bytes{
            std::byte{static_cast<unsigned char>(actual ? 1 : 0)}};
    golden.bytes(bytes, "|b1", std::span<const std::size_t>{}, label);
}

// _golden.record(np.asarray(pairs, np.int32).reshape(-1, 2), ...) — a
// (K, 2) little-endian int32 entry.
inline void i4pairs(bt::oracle::Node &golden,
                    const std::vector<std::pair<int, int>> &pairs,
                    std::string_view label) {
    std::vector<std::byte> bytes;
    bytes.reserve(pairs.size() * 8);
    for (const auto &[start, stop] : pairs)
        for (const auto value : {start, stop}) {
            const auto bits = std::bit_cast<std::uint32_t>(value);
            for (unsigned shift = 0; shift < 32; shift += 8)
                bytes.push_back(
                        static_cast<std::byte>((bits >> shift) & 0xffu));
        }
    const std::array<std::size_t, 2> shape{pairs.size(), 2};
    golden.bytes(bytes, "<i4", shape, label);
}

// ---------------------------------------------------------------------
// Oracle repr spellings — the f-string labels the capture carries.
// bike::io::repr_binary64 is the established CPython repr(float) twin
// (shortest round-trip digits, fixed for -4 <= e < 16, ".0" on
// integral values, nan/inf/-inf spellings).
// ---------------------------------------------------------------------
using bike::io::repr_binary64;

// repr(np.float64(v)) under numpy 2.x — 'np.float64(<repr>)'.
[[nodiscard]] inline std::string np_f64(double value) {
    return "np.float64(" + repr_binary64(value) + ")";
}

// np.linspace is already lowered in production (geometry/specs.hpp):
// arange*step + start rows, endpoint-pinned last element.
using bike::geometry::detail::np_linspace;

// np.arange(n) * step — the corpus grid generator.
[[nodiscard]] inline std::vector<double> arange_scaled(std::size_t n,
                                                       double step) {
    std::vector<double> x(n);
    for (std::size_t i = 0; i < n; ++i) x[i] = static_cast<double>(i) * step;
    return x;
}

// ---------------------------------------------------------------------
// Surface / tyre adapters — the wire dicts the Python tests build from
// the registered registry and stock specs.
// ---------------------------------------------------------------------

[[nodiscard]] inline nativecfg::SurfaceSpec surface(std::string_view name) {
    const auto &registered = bike::terrain::get_surface(name);
    return {.name = registered.name,
            .mu_peak = registered.mu_peak,
            .mu_slide = registered.mu_slide,
            .slip_stiffness_per_load = registered.slip_stiffness_per_load,
            .stribeck_speed_mps = registered.stribeck_speed_mps};
}

// ALL_SURFACES order: ('asphalt', 'hardpack', 'loose', 'wet', 'scree').
inline constexpr std::array<std::string_view, 5> kSurfaceNames{
        "asphalt", "hardpack", "loose", "wet", "scree"};

// The unit-law surface the clip-boundary tests construct — friction
// limit == mu == 1.0 at v_s == 0 with loads = 1.0.
inline const nativecfg::SurfaceSpec kUnitSurface{
        .name = "unit",
        .mu_peak = 1.0,
        .mu_slide = 0.5,
        .slip_stiffness_per_load = 0.5,
        .stribeck_speed_mps = 1.0};

[[nodiscard]] inline nativecfg::SurfaceSpec surface_at(std::size_t index) {
    return surface(kSurfaceNames.at(index));
}

// ---------------------------------------------------------------------
// Exception-class checks — the oracle's ValueError arrives as
// std::invalid_argument, its OverflowError as std::overflow_error.
// The optional needle pins the oracle's `match=` text.
// ---------------------------------------------------------------------
template <class Exception, class F>
void rejects(F &&action, std::string_view needle = {},
             const std::source_location &loc =
                     std::source_location::current()) {
    bool thrown = false;
    try {
        std::forward<F>(action)();
    } catch (const Exception &failure) {
        thrown = true;
        bt::require(needle.empty() ||
                            std::string_view(failure.what())
                                            .find(needle) !=
                                    std::string_view::npos,
                    (std::string("exception text lacks '") +=
                     std::string(needle)) += "'");
    }
    bt::require(thrown, (std::string("expected rejection at ") +=
                         loc.file_name()) += ":" +
                                std::to_string(loc.line()));
}

// ---------------------------------------------------------------------
// _synth_hits — the carcass tests' manufactured RayHits: inf r_m,
// caller/seeded road rows, coverage_event False, airborne = all(<= 0).
// rng=None in the oracle is a fresh default_rng(0x5eed).
// ---------------------------------------------------------------------
[[nodiscard]] inline biketyre::RayHits
synth_hits(std::size_t n, std::span<const double> delta_m,
           bike::random::NumpyRng *rng = nullptr,
           std::span<const double> road_x_m = {},
           std::span<const double> road_z_m = {}) {
    std::unique_ptr<bike::random::NumpyRng> fresh;
    if (rng == nullptr) {
        fresh = std::make_unique<bike::random::NumpyRng>(0x5eed);
        rng = fresh.get();
    }
    biketyre::RayHits hits;
    hits.r_m.assign(n, std::numeric_limits<double>::infinity());
    hits.delta_m.assign(delta_m.begin(), delta_m.end());
    hits.road_x_m.resize(n);
    if (road_x_m.empty()) {
        rng->uniform(4.0, 6.0, hits.road_x_m);
        for (std::size_t i = 0; i < n; ++i)
            hits.road_x_m[i] =
                    hits.road_x_m[i] + static_cast<double>(i) * 0.001;
    } else {
        std::ranges::copy(road_x_m, hits.road_x_m.begin());
    }
    hits.road_z_m.resize(n);
    if (road_z_m.empty())
        rng->uniform(-0.4, 0.4, hits.road_z_m);
    else
        std::ranges::copy(road_z_m, hits.road_z_m.begin());
    hits.coverage_event = false;
    hits.airborne = std::ranges::all_of(
            hits.delta_m, [](const double d) { return d <= 0.0; });
    return hits;
}

// ---------------------------------------------------------------------
// Entry consumers mirroring the oracle assertion helpers, in source
// record order.
// ---------------------------------------------------------------------

// _assert_state_bitwise (carcass.py helpers): three rows, two flags,
// five rim-event slots — "<msg> state.<key>" labels.
inline void carcass_state(bt::oracle::Node &golden,
                          const biketyre::CarcassState &state,
                          const std::string &msg) {
    vector(golden, state.delta_prev_m(), msg + " state.delta_prev_m");
    vector(golden, state.elastic_prev_n(), msg + " state.elastic_prev_n");
    vector(golden, state.maxwell_force_n(), msg + " state.maxwell_force_n");
    boolean(golden, state.initialized(), msg + " state.initialized");
    boolean(golden, state.rim_strike_active(),
            msg + " state.rim_strike_active");
    scalar(golden, state.rim_event_x_m(), msg + " state.rim_event_x_m");
    scalar(golden, state.rim_event_speed_mps(),
           msg + " state.rim_event_speed_mps");
    scalar(golden, state.rim_event_peak_load_n(),
           msg + " state.rim_event_peak_load_n");
    scalar(golden, state.rim_event_peak_force_n(),
           msg + " state.rim_event_peak_force_n");
    scalar(golden, state.rim_event_energy_j(),
           msg + " state.rim_event_energy_j");
}

// _assert_result_bitwise (carcass): seven rows, the (K,2) int32 patch
// table, lengths, four scalars, the two flags, then the optional event
// payload — "<msg> result.<key>" labels.
inline void carcass_result(bt::oracle::Node &golden,
                           const biketyre::CarcassResult &result,
                           const std::string &msg) {
    vector(golden, result.pressure_force_n,
           msg + " result.pressure_force_n");
    vector(golden, result.elastic_force_n, msg + " result.elastic_force_n");
    vector(golden, result.maxwell_force_n, msg + " result.maxwell_force_n");
    vector(golden, result.hysteresis_force_n,
           msg + " result.hysteresis_force_n");
    vector(golden, result.rim_force_n, msg + " result.rim_force_n");
    vector(golden, result.total_element_force_n,
           msg + " result.total_element_force_n");
    vector(golden, result.material_rate_mps,
           msg + " result.material_rate_mps");
    i4pairs(golden, result.contact_patches, msg + " result.contact_patches");
    vector(golden, result.contact_lengths_m,
           msg + " result.contact_lengths_m");
    scalar(golden, result.force_x_n, msg + " result.force_x_n");
    scalar(golden, result.force_z_n, msg + " result.force_z_n");
    scalar(golden, result.normal_load_n, msg + " result.normal_load_n");
    scalar(golden, result.rim_energy_j, msg + " result.rim_energy_j");
    boolean(golden, result.rim_strike_active,
            msg + " result.rim_strike_active");
    boolean(golden, result.rim_event.has_value(), msg + " result.rim_event");
    if (result.rim_event.has_value()) {
        scalar(golden, result.rim_event->x_m, msg + " result.rim_event.x_m");
        scalar(golden, result.rim_event->speed_mps,
               msg + " result.rim_event.speed_mps");
        scalar(golden, result.rim_event->peak_load_n,
               msg + " result.rim_event.peak_load_n");
        scalar(golden, result.rim_event->peak_rim_force_n,
               msg + " result.rim_event.peak_rim_force_n");
        scalar(golden, result.rim_event->absorbed_energy_j,
               msg + " result.rim_event.absorbed_energy_j");
    }
}

// _assert_result_bitwise (brush): "<msg> result.{force_n,
// fully_sliding, friction_coefficient}".
inline void brush_result(bt::oracle::Node &golden,
                         const biketyre::BrushResult &result,
                         const std::string &msg) {
    scalar(golden, result.force_n, msg + " result.force_n");
    boolean(golden, result.fully_sliding, msg + " result.fully_sliding");
    scalar(golden, result.friction_coefficient,
           msg + " result.friction_coefficient");
}

// _assert_brush_state_bitwise: "<msg> state.{bristle_deflection_m,
// kappa_prime}".
inline void brush_state(bt::oracle::Node &golden,
                        const biketyre::DiscretisedBrush &brush,
                        const std::string &msg) {
    vector(golden, brush.bristle_deflection_m(),
           msg + " state.bristle_deflection_m");
    scalar(golden, brush.kappa_prime(), msg + " state.kappa_prime");
}

// _assert_patches: 'patches' — the (K,2) int32 run table.
inline void consume_patches(bt::oracle::Node &golden,
                            std::span<const double> delta_m) {
    i4pairs(golden, biketyre::patches(delta_m), "patches");
}

// ---------------------------------------------------------------------
// _run_sequence — the shared evaluate/advance driver: 'step {idx}
// (evaluate)' emits result then state entries; 'step {idx} (advance)'
// emits state entries only. `watch` is the optional
// check_oracle_state(want, oracle_state, tag) callback.
// ---------------------------------------------------------------------
struct CarcassStep {
    bool evaluate;
    biketyre::RayHits hits;
    double dt_s = 0.0, omega_radps = 0.0, speed_mps = 0.0;
    double dtheta_rad = 0.0, relaxation_s = 0.0;
};

[[nodiscard]] inline CarcassStep eval_step(biketyre::RayHits hits,
                                           double dt_s, double omega_radps,
                                           double speed_mps) {
    return {.evaluate = true,
            .hits = std::move(hits),
            .dt_s = dt_s,
            .omega_radps = omega_radps,
            .speed_mps = speed_mps};
}

[[nodiscard]] inline CarcassStep advance_step(double omega_radps,
                                              double dt_s,
                                              double dtheta_rad,
                                              double relaxation_s) {
    return {.evaluate = false,
            .hits = {},
            .dt_s = dt_s,
            .omega_radps = omega_radps,
            .dtheta_rad = dtheta_rad,
            .relaxation_s = relaxation_s};
}

inline void run_carcass_sequence(
        bt::oracle::Node &golden, const biketyre::RayRing &ring,
        const nativecfg::PneumaticTyreSpec &tyre,
        std::span<const CarcassStep> steps,
        const std::function<void(const biketyre::CarcassResult &,
                                 const biketyre::CarcassState &,
                                 const std::string &)> &watch = {}) {
    biketyre::CarcassState state(static_cast<std::int64_t>(ring.n()));
    for (std::size_t idx = 0; idx < steps.size(); ++idx) {
        const auto &step = steps[idx];
        const std::string tag = "step " + std::to_string(idx) +
                                (step.evaluate ? " (evaluate)"
                                               : " (advance)");
        if (step.evaluate) {
            const auto result = biketyre::evaluate_carcass(
                    ring, step.hits, tyre, state, step.dt_s,
                    step.omega_radps, step.speed_mps);
            carcass_result(golden, result, tag);
            carcass_state(golden, state, tag);
            if (watch) watch(result, state, tag);
        } else {
            state.advance_unloaded(step.omega_radps, step.dt_s,
                                   step.dtheta_rad, step.relaxation_s);
            carcass_state(golden, state, tag);
        }
    }
}

} // namespace pneumatic_reference
