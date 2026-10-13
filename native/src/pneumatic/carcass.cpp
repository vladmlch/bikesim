// pneumatic/carcass.cpp — implementation of pneumatic/carcass.hpp, a
// bitwise port of sim/ride/tyre/carcass.py.
//
// NumPy lowering decisions (each verified against numpy 2.5.2 on this
// platform; see the task-8 report for the evidence):
//
//   np.sum(a)                       -> numeric::numpy_pairwise_sum
//   np.dot(a, b)                    -> blas::ddot (Accelerate CBLAS;
//                                    same contract as the other ported
//                                    kernels — contiguous f64 rows)
//   np.tanh(a)                      -> numeric::np_tanh — numpy's own
//                                    lut18x16 SIMD table scalarized;
//                                    std::tanh is NOT bitwise-equal
//   np.exp(scalar)                  -> std::exp (DOUBLE_exp lowers to
//                                    npy_exp -> libm exp on aarch64;
//                                    the SIMD path is AVX512-only)
//   np.hypot(dx, dz)                -> std::hypot (npy_hypot -> libm
//                                    hypot; NOT biketyre::py_hypot —
//                                    that is CPython math.hypot's
//                                    DoubleLength kernel, which is a
//                                    different function bitwise)
//   np.maximum(a, 0.0)              -> numeric::np_maximum — NEON fmax
//                                    semantics (NaN payload passthrough,
//                                    +/-0.0 tie -> +0.0)
//   np.interp(idx, arange, v, 0, 0) -> numeric::np_interp_unit_grid —
//                                    arr_interp on integer knots, with
//                                    the left/right=0.0 sentinels
//   np.argmax(a)                    -> strict-> first-maximum scan
//   np.where(c, a, b)               -> the ternary (elementwise select;
//                                    both operand values are computed
//                                    anyway — evaluation order is free)
//   np.any(a) / np.any(a > 0.0)     -> first-true scan
//   np.abs / abs()                  -> std::fabs / std::abs
//   x ** 2 on an array              -> x * x (numpy's pow fast path for
//                                    integral exponent 2, multiplication
//                                    not npy_pow)
//   np.asarray(x, float)            -> the contiguous f64 input spans
//   np.zeros(n)                     -> the allocation's zero fill
//
// Millimetre/pascal conversions multiply by the oracle's literal
// factors: _MM_TO_M = 0.001, _MM2_TO_M2 = 1000000.0, _BAR_TO_PA =
// 100000.0 (carcass.py:17-19).
//
// Scalarization freedom: every numpy elementwise expression is
// evaluated per-element in the same order as the oracle's array op;
// whole-array temporaries become the same value streamed through one
// lane, and reductions keep their own reduction kernels.

#include "carcass.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>

#include "../cblas_abi.hpp"
#include "../numeric_numpy.hpp"
#include "../numeric_sum.hpp"

namespace biketyre {
    namespace {
        // carcass.py:17-19 — the oracle's literal conversion factors.
        inline constexpr double kBarToPa = 100000.0;
        inline constexpr double kMmToM = 0.001;
        inline constexpr double kMm2ToM2 = 1000000.0;

        // material_deflection_rate_mps body (carcass.py:151-165) shared
        // by the standalone kernel and evaluate_carcass_into — the
        // caller is responsible for the oracle's two validations.
        inline void material_rate_loop(
                std::span<const double> delta,
                std::span<const double> previous, double omega_radps,
                // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) oracle parameter order
                double dt_s, double dtheta_rad,
                std::span<double> out) noexcept {
            const std::size_t n = delta.size();
            if (n == 0)
                // numpy 2.x broadcasts a length-0 row against the
                // concatenate() length-1 tap without error: every
                // branch yields an empty result row and never reads
                // delta[0] / delta[n-1].
                return;
            if (omega_radps > 0.0) {
                // next_ray = concatenate((delta[1:], [0.0])) — the
                // out-of-range tap is a literal +0.0.
                for (std::size_t i = 0; i + 1 < n; ++i) {
                    const double gradient =
                            (delta[i + 1] - delta[i]) / dtheta_rad;
                    out[i] = (delta[i] - previous[i]) / dt_s -
                             omega_radps * gradient;
                }
                const double gradient =
                        (0.0 - delta[n - 1]) / dtheta_rad;
                out[n - 1] = (delta[n - 1] - previous[n - 1]) / dt_s -
                             omega_radps * gradient;
            } else if (omega_radps < 0.0) {
                // previous_ray = concatenate(([0.0], delta[:-1])).
                {
                    const double gradient =
                            (delta[0] - 0.0) / dtheta_rad;
                    out[0] = (delta[0] - previous[0]) / dt_s -
                             omega_radps * gradient;
                }
                for (std::size_t i = 1; i < n; ++i) {
                    const double gradient =
                            (delta[i] - delta[i - 1]) / dtheta_rad;
                    out[i] = (delta[i] - previous[i]) / dt_s -
                             omega_radps * gradient;
                }
            } else {
                // gradient = np.zeros_like(delta) — including NaN omega:
                // the oracle's `omega > 0` / `omega < 0` tests are false
                // and `nan * 0.0` propagates nan into the rate.
                for (std::size_t i = 0; i < n; ++i)
                    out[i] = (delta[i] - previous[i]) / dt_s -
                             omega_radps * 0.0;
            }
        }

        // _contact_lengths_m (carcass.py:177-192): scaled Euclidean span
        // between the first and last road point of each patch; a
        // single-ray patch keeps one finite angular cell instead.
        void contact_lengths_m_into(
                // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) oracle field order
                std::span<const double> road_x_m,
                std::span<const double> road_z_m,
                std::span<const std::pair<int, int>> contact_patches,
                // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) oracle parameter order
                double contact_length_factor, double one_ray_span_m,
                std::span<double> out) noexcept {
            for (std::size_t k = 0; k < contact_patches.size(); ++k) {
                const auto [start, stop] = contact_patches[k];
                const std::size_t a =
                        static_cast<std::size_t>(start);
                const std::size_t b =
                        static_cast<std::size_t>(stop) - 1;
                const double length_m =
                        stop - start == 1
                                ? one_ray_span_m
                                // np.hypot -> npy_hypot -> libm hypot.
                                : std::hypot(road_x_m[b] - road_x_m[a],
                                             road_z_m[b] - road_z_m[a]);
                out[k] = contact_length_factor * length_m;
            }
        }
    } // namespace

    // ---------------------------------------------------------------
    // CarcassState (carcass.py:33-98)
    // ---------------------------------------------------------------

    CarcassState::CarcassState(std::int64_t n_rays) {
        // CarcassState.__post_init__ — ValueError("carcass needs at
        // least 3 rays, got {n}").
        if (n_rays < 3)
            throw std::invalid_argument(
                    "carcass needs at least 3 rays, got " +
                    std::to_string(n_rays));
        n_rays_ = static_cast<std::size_t>(n_rays);
        delta_prev_m_.assign(n_rays_, 0.0);
        elastic_prev_n_.assign(n_rays_, 0.0);
        maxwell_force_n_.assign(n_rays_, 0.0);
        advect_scratch_.assign(n_rays_, 0.0);
    }

    void CarcassState::reset() noexcept {
        std::ranges::fill(delta_prev_m_, 0.0);
        std::ranges::fill(elastic_prev_n_, 0.0);
        std::ranges::fill(maxwell_force_n_, 0.0);
        initialized_ = false;
        clear_rim_event();
    }

    void CarcassState::advance_unloaded(double omega_radps, double dt_s,
                                        double dtheta_rad,
                                        double relaxation_s) {
        // carcass.py:83-84 — the early return precedes validation.
        if (!initialized_) return;
        if (dt_s <= 0.0 || dtheta_rad <= 0.0 || relaxation_s <= 0.0)
            throw std::invalid_argument(
                    "time, angular spacing and relaxation time must be "
                    "positive");
        // np.any(maxwell_force_n): elementwise truthiness — nan counts
        // as True (v != 0.0 keeps it), +/-0.0 do not.
        bool any = false;
        for (const double v : maxwell_force_n_) any = any || v != 0.0;
        if (!any) return;
        // The oracle computes a fresh advected array and rescales it
        // into the row — the scratch member keeps that staging
        // allocation-free.
        advect_material_into(maxwell_force_n_, omega_radps, dt_s,
                             dtheta_rad, advect_scratch_);
        const double decay = std::exp(-dt_s / relaxation_s);
        for (std::size_t i = 0; i < n_rays_; ++i)
            maxwell_force_n_[i] = decay * advect_scratch_[i];
    }

    void CarcassState::assign(const CarcassState &source) {
        if (source.n_rays_ != n_rays_)
            throw std::invalid_argument(
                    "carcass state ray count mismatch");
        std::ranges::copy(source.delta_prev_m_, delta_prev_m_.begin());
        std::ranges::copy(source.elastic_prev_n_, elastic_prev_n_.begin());
        std::ranges::copy(source.maxwell_force_n_, maxwell_force_n_.begin());
        initialized_ = source.initialized_;
        rim_strike_active_ = source.rim_strike_active_;
        rim_event_x_m_ = source.rim_event_x_m_;
        rim_event_speed_mps_ = source.rim_event_speed_mps_;
        rim_event_peak_load_n_ = source.rim_event_peak_load_n_;
        rim_event_peak_force_n_ = source.rim_event_peak_force_n_;
        rim_event_energy_j_ = source.rim_event_energy_j_;
    }

    void CarcassState::restore_fields(
            bool initialized, bool rim_strike_active,
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) oracle field order
            double rim_event_x_m, double rim_event_speed_mps,
            double rim_event_peak_load_n, double rim_event_peak_force_n,
            double rim_event_energy_j) noexcept {
        initialized_ = initialized;
        rim_strike_active_ = rim_strike_active;
        rim_event_x_m_ = rim_event_x_m;
        rim_event_speed_mps_ = rim_event_speed_mps;
        rim_event_peak_load_n_ = rim_event_peak_load_n;
        rim_event_peak_force_n_ = rim_event_peak_force_n;
        rim_event_energy_j_ = rim_event_energy_j;
    }

    void CarcassState::clear_rim_event() noexcept {
        // _clear_rim_event (carcass.py:92-98).
        rim_strike_active_ = false;
        rim_event_x_m_ = 0.0;
        rim_event_speed_mps_ = 0.0;
        rim_event_peak_load_n_ = 0.0;
        rim_event_peak_force_n_ = 0.0;
        rim_event_energy_j_ = 0.0;
    }

    // _update_rim_event (carcass.py:327-364) — the rim-strike FSM.
    // Tracks an event from the first engaged ray until the entire rim
    // patch releases; returns the completed RimStrikeEvent on release.
    std::optional<RimStrikeEvent> CarcassState::update_rim_event(
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) oracle signature
            bool active, std::span<const double> road_x_m,
            std::span<const double> overlap_m,
            std::span<const double> rim_force_n, double vertical_load_n,
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) oracle signature
            double energy_j, double speed_mps) noexcept {
        if (active) {
            if (!rim_strike_active_) {
                // np.argmax — the first index holding the maximum.
                std::size_t ray = 0;
                double best = overlap_m[0];
                for (std::size_t i = 1; i < overlap_m.size(); ++i) {
                    if (overlap_m[i] > best) {
                        best = overlap_m[i];
                        ray = i;
                    }
                }
                rim_event_x_m_ = road_x_m[ray];
                rim_event_speed_mps_ = std::fabs(speed_mps);
                rim_event_peak_load_n_ = 0.0;
                rim_event_peak_force_n_ = 0.0;
                rim_event_energy_j_ = 0.0;
            }
            rim_strike_active_ = true;
            // Python builtin max(a, b): keeps the running peak on a tie
            // and on a NaN candidate (unreachable — both operands are
            // finite here, but keep the same ordering anyway).
            rim_event_peak_load_n_ =
                    std::max(rim_event_peak_load_n_, vertical_load_n);
            rim_event_peak_force_n_ =
                    std::max(rim_event_peak_force_n_,
                             numeric::numpy_pairwise_sum(rim_force_n));
            rim_event_energy_j_ =
                    std::max(rim_event_energy_j_, energy_j);
            return std::nullopt;
        }
        if (!rim_strike_active_) return std::nullopt;
        const RimStrikeEvent completed{
                .x_m = rim_event_x_m_,
                .speed_mps = rim_event_speed_mps_,
                .peak_load_n = rim_event_peak_load_n_,
                .peak_rim_force_n = rim_event_peak_force_n_,
                .absorbed_energy_j = rim_event_energy_j_,
        };
        clear_rim_event();
        return completed;
    }

    // ---------------------------------------------------------------
    // material_deflection_rate_mps (carcass.py:127-165)
    // ---------------------------------------------------------------

    void material_deflection_rate_mps_into(
            std::span<const double> delta_m,
            std::span<const double> delta_prev_m, double omega_radps,
            double dt_s, double dtheta_rad, std::span<double> out) {
        // The oracle's 1-D shape check — span lengths stand in for
        // ndarray shapes.
        if (delta_m.size() != delta_prev_m.size() ||
            delta_m.size() != out.size())
            throw std::invalid_argument(
                    "current and previous deflections must be matching "
                    "1-D arrays");
        if (dt_s <= 0.0 || dtheta_rad <= 0.0)
            throw std::invalid_argument(
                    "time step and angular spacing must be positive");
        material_rate_loop(delta_m, delta_prev_m, omega_radps, dt_s,
                           dtheta_rad, out);
    }

    std::vector<double> material_deflection_rate_mps(
            std::span<const double> delta_m,
            std::span<const double> delta_prev_m, double omega_radps,
            double dt_s, double dtheta_rad) {
        // The oracle allocates the result array; this twin mirrors it —
        // the vector is materialized before the kernel runs.
        std::vector<double> out(delta_m.size(), 0.0);
        material_deflection_rate_mps_into(delta_m, delta_prev_m,
                                          omega_radps, dt_s, dtheta_rad,
                                          out);
        return out;
    }

    // ---------------------------------------------------------------
    // _advect_material (carcass.py:168-174)
    // ---------------------------------------------------------------

    void advect_material_into(std::span<const double> values,
                              double omega_radps, double dt_s,
                              double dtheta_rad, std::span<double> out) {
        if (values.size() != out.size())
            throw std::invalid_argument(
                    "advection output does not match the input width");
        if (omega_radps == 0.0) {
            // values.copy() — verbatim, -0.0 and NaN payloads included.
            std::ranges::copy(values, out.begin());
            return;
        }
        // source_index = arange(n) + omega*dt/dtheta — the scalar
        // ((omega*dt)/dtheta) evaluated once, like the oracle.
        const double shift = omega_radps * dt_s / dtheta_rad;
        const std::size_t n = values.size();
        for (std::size_t i = 0; i < n; ++i)
            out[i] = numeric::np_interp_unit_grid(
                    static_cast<double>(i) + shift, values, 0.0, 0.0);
    }

    std::vector<double> advect_material(std::span<const double> values,
                                        double omega_radps, double dt_s,
                                        double dtheta_rad) {
        std::vector<double> out(values.size(), 0.0);
        advect_material_into(values, omega_radps, dt_s, dtheta_rad, out);
        return out;
    }

    // ---------------------------------------------------------------
    // evaluate_carcass (carcass.py:195-324)
    // ---------------------------------------------------------------

    void evaluate_carcass_into(const RayRing &ring,
                               std::span<const double> delta_m,
                               std::span<const double> road_x_m,
                               std::span<const double> road_z_m,
                               const nativecfg::PneumaticTyreSpec &tyre,
                               CarcassState &state, double dt_s,
                               double omega_radps, double speed_mps,
                               CarcassBuffers &out) {
        const std::size_t n = ring.n();
        // carcass.py:226-234 — every check precedes any mutation; a
        // rejected call leaves the state byte-identical.
        if (dt_s <= 0.0 || !std::isfinite(dt_s))
            throw std::invalid_argument(
                    "dt_s must be positive and finite");
        if (!std::isfinite(omega_radps) || !std::isfinite(speed_mps))
            throw std::invalid_argument(
                    "wheel spin and speed must be finite");
        if (state.n_rays_ != n)
            throw std::invalid_argument(
                    "carcass state does not match the ray ring");
        if (delta_m.size() != n)
            throw std::invalid_argument(
                    "ray deflections must be finite, nonnegative and "
                    "match the ring");
        for (const double d : delta_m) {
            if (!std::isfinite(d) || d < 0.0)
                throw std::invalid_argument(
                        "ray deflections must be finite, nonnegative "
                        "and match the ring");
        }
        // The C++ boundary contract — the oracle's ndarray shapes
        // expressed as buffer sizes (the oracle reads
        // hits.road_x_m / hits.road_z_m at least n wide implicitly).
        // Still strictly before any mutation, like the oracle's checks.
        if (road_x_m.size() != n || road_z_m.size() != n ||
            out.pressure_force_n.size() != n ||
            out.elastic_force_n.size() != n ||
            out.maxwell_force_n.size() != n ||
            out.hysteresis_force_n.size() != n ||
            out.rim_force_n.size() != n ||
            out.total_element_force_n.size() != n ||
            out.material_rate_mps.size() != n ||
            out.scratch.size() < 2 * n ||
            out.contact_patches.size() < (n + 1) / 2 ||
            out.contact_lengths_m.size() < (n + 1) / 2)
            throw std::invalid_argument(
                    "carcass buffers do not match the ray ring");

        const double radius_m = tyre.outer_radius_mm * kMmToM;
        const double casing_width_m = tyre.casing_width_mm * kMmToM;
        const double casing_radius_m = 0.5 * casing_width_m;
        const double ds_m = radius_m * ring.dtheta_rad();
        const double pressure_pa = tyre.pressure_bar * kBarToPa;
        const double area_pressure = tyre.area_factor * pressure_pa;
        const double carcass_si =
                tyre.carcass_stiffness_n_mm2 * kMm2ToM2;

        // carcass.py:244-249 — the casing chord saturates at the casing
        // width; the carcass term stays progressive in delta.
        const double two_casing_radius = 2.0 * casing_radius_m;
        for (std::size_t i = 0; i < n; ++i) {
            const double d = delta_m[i];
            const double inner = numeric::np_maximum(
                    d * (two_casing_radius - d), 0.0);
            const double chord = d >= casing_radius_m
                                         ? casing_width_m
                                         : 2.0 * std::sqrt(inner);
            const double pressure_line = area_pressure * chord;
            const double carcass_line = carcass_si * d;
            out.pressure_force_n[i] = pressure_line * ds_m;
            out.elastic_force_n[i] =
                    (pressure_line + carcass_line) * ds_m;
        }

        // carcass.py:251-267 — material memory. The two advected rows
        // stage in the caller's scratch; they stay live until the
        // Maxwell update has consumed them.
        const std::span<double> advected_a = out.scratch.subspan(0, n);
        const std::span<double> advected_b = out.scratch.subspan(n, n);
        if (state.initialized_) {
            material_rate_loop(delta_m, state.delta_prev_m_,
                               omega_radps, dt_s, ring.dtheta_rad(),
                               out.material_rate_mps);
            advect_material_into(state.elastic_prev_n_, omega_radps,
                                 dt_s, ring.dtheta_rad(), advected_a);
            advect_material_into(state.maxwell_force_n_, omega_radps,
                                 dt_s, ring.dtheta_rad(), advected_b);
            const double decay =
                    std::exp(-dt_s / tyre.rate_relaxation_s);
            for (std::size_t i = 0; i < n; ++i)
                out.maxwell_force_n[i] =
                        decay * advected_b[i] +
                        tyre.rate_stiffening *
                                (out.elastic_force_n[i] -
                                 advected_a[i]);
        } else {
            std::ranges::fill(out.material_rate_mps, 0.0);
            std::ranges::fill(out.maxwell_force_n, 0.0);
        }

        // carcass.py:269-284 — rim overlap, hysteresis and the
        // nonnegative saturation. The advected scratch rows are dead
        // now; they hold the overlap and the rim-energy summand.
        const double rim_deflection_m =
                tyre.rim_strike_deflection_mm() * kMmToM;
        const double rim_si = tyre.rim_stiffness_n_mm2 * kMm2ToM2;
        const std::span<double> &rim_overlap = advected_a;
        const std::span<double> &rim_energy_term = advected_b;
        for (std::size_t i = 0; i < n; ++i) {
            const double d = delta_m[i];
            const double overlap =
                    numeric::np_maximum(d - rim_deflection_m, 0.0);
            rim_overlap[i] = overlap;
            const double rim_elastic = rim_si * overlap * ds_m;
            // np.tanh(material_rate_mps / eps): elementwise divide,
            // then the numpy LUT tanh.
            const double sign = numeric::np_tanh(
                    out.material_rate_mps[i] /
                    tyre.hysteresis_rate_eps_mps);
            const double carcass_hysteresis =
                    tyre.loss_factor * out.elastic_force_n[i] * sign;
            const double rim_hysteresis =
                    tyre.loss_factor * rim_elastic * sign;
            out.hysteresis_force_n[i] =
                    carcass_hysteresis + rim_hysteresis;
            out.rim_force_n[i] = rim_elastic + rim_hysteresis;
            const double total = numeric::np_maximum(
                    out.elastic_force_n[i] + out.maxwell_force_n[i] +
                            out.hysteresis_force_n[i] +
                            out.rim_force_n[i],
                    0.0);
            // np.where(loaded, np.maximum(total, 0.0), 0.0) — the
            // inner maximum is evaluated on the whole row by the
            // oracle; the same value flows through one lane here.
            out.total_element_force_n[i] =
                    d > 0.0 ? numeric::np_maximum(total, 0.0) : 0.0;
            // rim_overlap ** 2 — numpy's power-of-2 lowering is a
            // plain multiply.
            rim_energy_term[i] = rim_si * (overlap * overlap) * ds_m;
        }

        // carcass.py:286-291 — wheel-level reductions.
        const int n_i32 = static_cast<int>(n);
        out.force_x_n =
                -blas::ddot(n_i32, out.total_element_force_n.data(), 1,
                            ring.ux().data(), 1);
        out.force_z_n =
                -blas::ddot(n_i32, out.total_element_force_n.data(), 1,
                            ring.uz().data(), 1);
        out.normal_load_n = numeric::numpy_pairwise_sum(
                out.total_element_force_n);
        out.rim_energy_j =
                0.5 * numeric::numpy_pairwise_sum(rim_energy_term);

        // carcass.py:293-296 — contact patches and scaled lengths.
        out.patch_count =
                patches_into(delta_m, out.contact_patches);
        contact_lengths_m_into(
                road_x_m, road_z_m,
                out.contact_patches.first(out.patch_count),
                tyre.contact_length_factor, ds_m,
                out.contact_lengths_m);

        // carcass.py:297-306 — the rim FSM runs before the material
        // commit, exactly like the oracle: a completed event is
        // reported on this step's result and the event state clears
        // before the history rows update.
        bool rim_active = false;
        for (const double ov : rim_overlap)
            rim_active = rim_active || ov > 0.0;
        out.rim_strike_active = rim_active;
        out.rim_event = state.update_rim_event(
                rim_active, road_x_m, rim_overlap, out.rim_force_n,
                out.force_z_n, out.rim_energy_j, speed_mps);

        std::ranges::copy(delta_m, state.delta_prev_m_.begin());
        std::ranges::copy(out.elastic_force_n.first(n),
                          state.elastic_prev_n_.begin());
        std::ranges::copy(out.maxwell_force_n.first(n),
                          state.maxwell_force_n_.begin());
        state.initialized_ = true;
    }

    CarcassResult evaluate_carcass(const RayRing &ring,
                                   const RayHits &hits,
                                   const nativecfg::PneumaticTyreSpec &tyre,
                                   CarcassState &state, double dt_s,
                                   double omega_radps, double speed_mps) {
        const std::size_t n = ring.n();
        // All allocation precedes the kernel — an invalid-input throw
        // leaves the state untouched (and the half-built result is
        // discarded, never observed).
        CarcassResult result;
        result.pressure_force_n.assign(n, 0.0);
        result.elastic_force_n.assign(n, 0.0);
        result.maxwell_force_n.assign(n, 0.0);
        result.hysteresis_force_n.assign(n, 0.0);
        result.rim_force_n.assign(n, 0.0);
        result.total_element_force_n.assign(n, 0.0);
        result.material_rate_mps.assign(n, 0.0);
        result.contact_patches.assign((n + 1) / 2, {0, 0});
        result.contact_lengths_m.assign((n + 1) / 2, 0.0);
        std::vector<double> scratch(2 * n, 0.0);
        CarcassBuffers out{
                .pressure_force_n = result.pressure_force_n,
                .elastic_force_n = result.elastic_force_n,
                .maxwell_force_n = result.maxwell_force_n,
                .hysteresis_force_n = result.hysteresis_force_n,
                .rim_force_n = result.rim_force_n,
                .total_element_force_n = result.total_element_force_n,
                .material_rate_mps = result.material_rate_mps,
                .contact_patches = result.contact_patches,
                .contact_lengths_m = result.contact_lengths_m,
                .scratch = scratch,
                .patch_count = 0,
                .force_x_n = 0.0,
                .force_z_n = 0.0,
                .normal_load_n = 0.0,
                .rim_energy_j = 0.0,
                .rim_strike_active = false,
                .rim_event = std::nullopt,
        };
        evaluate_carcass_into(ring, hits.delta_m, hits.road_x_m,
                              hits.road_z_m, tyre, state, dt_s,
                              omega_radps, speed_mps, out);
        result.contact_patches.resize(out.patch_count);
        result.contact_lengths_m.resize(out.patch_count);
        result.force_x_n = out.force_x_n;
        result.force_z_n = out.force_z_n;
        result.normal_load_n = out.normal_load_n;
        result.rim_energy_j = out.rim_energy_j;
        result.rim_strike_active = out.rim_strike_active;
        result.rim_event = out.rim_event;
        return result;
    }

} // namespace biketyre
