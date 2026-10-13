// pneumatic/brush.hpp — the pneumatic tyre's longitudinal slip kernels:
// a port of BRISTLE_DAMPING_S, BrushResult, relax, _sigma_x,
// _tread_stiffness_n_m2, lumped_brush and DiscretisedBrush from
// sim/ride/tyre/brush.py (lines 17-275), plus the SurfaceSpec::mu
// Stribeck helper both brush paths call (terrain/surface.py:74-92,
// including its array()/derived_array() checks from
// physics/checks.py).
//
// The lumped brush is the steady parabolic-pressure solution; the
// discretised brush carries one bristle per loaded ray, advects the
// bristle memory with the wheel surface speed and clips each bristle
// at its local friction limit. Quantities at this boundary are metres,
// seconds, newtons.
//
// Divergence guide:
//  - pneumatic/brush.cpp — implementation; its header comment lists
//    the NumPy/CPython lowering decisions that keep the port
//    bitwise-identical (math.exp -> std::exp, float `** 2` ->
//    pyfloat::pow, np.clip -> numeric::np_clip, np.interp on the unit
//    grid -> numeric::np_interp_unit_grid, np.sum -> pairwise_sum).
//  - src/bike_sim/sim/ride/tyre/brush.py — the behavioral oracle.
//  - tests/reference/test_native_pneumatic_brush.py — the bitwise
//    arbiter.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "../config_types.hpp" // nativecfg::SurfaceSpec

namespace biketyre {

    // BRISTLE_DAMPING_S (brush.py:17) — the bristle dashpot relaxation
    // time in the discretised kernel's damping force.
    inline constexpr double kBristleDampingS = 0.0005;

    // BrushResult (brush.py:21-27) — tangential force, full-sliding
    // flag and the Stribeck coefficient for one contact patch.
    struct BrushResult {
        double force_n = 0.0;
        bool fully_sliding = false;
        double friction_coefficient = 0.0;
    };

    // SurfaceSpec.mu (surface.py:74-92) — mu_slide +
    // (mu_peak - mu_slide) * exp(-|v| / stribeck_speed_mps). Replicates
    // the oracle's intake/delivery checks: std::invalid_argument when
    // the sliding speed is non-finite (checks.py array()), and
    // std::overflow_error when the coefficient itself comes out
    // non-finite (checks.py derived_array()).
    [[nodiscard]] double surface_mu(const nativecfg::SurfaceSpec &surface,
                                    double sliding_speed_mps);

    // relax (brush.py:30-70) — the exact one-step update of transient
    // slip under constant velocities. Throws std::invalid_argument
    // where the oracle raises ValueError. The exponent is always <= -0.0
    // on the reachable path, so math.exp never raises OverflowError and
    // lowers to std::exp.
    [[nodiscard]] double relax(double kappa_prime, double v_x_mps,
                               double v_s_mps, double sigma_m,
                               double dt_s);

    // _sigma_x (brush.py:88-94) — the brush slip input; a locked wheel
    // is the negative fully-sliding limit. Branch order preserved:
    // kappa <= -1.0 -> -inf, then the isinf copysign, then the ratio.
    [[nodiscard]] double sigma_x(double kappa_prime) noexcept;

    // _tread_stiffness_n_m2 (brush.py:73-85) — resolves the
    // pressure-normalised brush stiffness c_px: the explicit value when
    // present (validated finite and nonnegative), otherwise
    // slip_stiffness_per_load * load / (2 * a^2). Throws
    // std::invalid_argument where the oracle raises ValueError.
    [[nodiscard]] double tread_stiffness_n_m2(
            double normal_load_n, double half_length_m,
            const nativecfg::SurfaceSpec &surface,
            std::optional<double> explicit_stiffness_n_m2);

    // lumped_brush (brush.py:97-147) — the steady parabolic-pressure
    // brush force for one patch. Validation order and the three early
    // returns match the oracle; an infinite kappa_prime is a legal
    // input (the isinf path in the finite gate).
    [[nodiscard]] BrushResult lumped_brush(
            double kappa_prime, double normal_load_n,
            double half_length_m, const nativecfg::SurfaceSpec &surface,
            std::optional<double> explicit_stiffness_n_m2,
            double sliding_speed_mps);

    // DiscretisedBrush (brush.py:150-272) — per-bristle brush state for
    // one contiguous contact patch; bristles are ordered rear to front.
    // The bristle row is sized n_elements at construction and never
    // resizes — assign() copies into the existing storage (the writer's
    // rollback/probe path in Task 11 must never reach the allocator
    // through it). step() validates and stages every write before the
    // bristle commit, matching the oracle's mutation order: the
    // zero-load early return resets state, the kappa_prime update
    // commits right after relax() returns, and the bristle row commits
    // last.
    class DiscretisedBrush {
    public:
        // DiscretisedBrush.__init__ (brush.py:160-165): at least 1
        // element; the bristle row starts zeroed, kappa_prime clears.
        explicit DiscretisedBrush(std::int64_t n_elements);

        // reset (brush.py:167-170): zeroes the bristles and the
        // transient slip.
        void reset() noexcept;

        // step (brush.py:172-272): advects and updates every bristle,
        // then sums the limited shear forces. Throws
        // std::invalid_argument where the oracle raises ValueError.
        // Allocates nothing — the two staging rows are members, so the
        // object is confined to one owning context like CarcassState.
        BrushResult step(std::span<const double> normal_loads_n,
                         double half_length_m, double v_x_mps,
                         double v_s_mps, double sigma_m,
                         const nativecfg::SurfaceSpec &surface,
                         double dt_s);

        // Explicit semantic-state copy — the Task-11 writer's failure
        // rollback/probe boundary. Copies into the existing storage
        // (never reallocates); throws std::invalid_argument on an
        // element-count mismatch instead of resizing.
        void assign(const DiscretisedBrush &source);

        // kappa_prime is a public attribute in the oracle — the debug
        // binding's state-dict intake and writer probes need the same
        // write access. Never on the warm path.
        void restore_kappa_prime(double kappa_prime) noexcept {
            kappa_prime_ = kappa_prime;
        }

        [[nodiscard]] std::size_t n_elements() const noexcept {
            return n_elements_;
        }

        // The oracle's public bristle_deflection_m row.
        [[nodiscard]] std::span<double> bristle_deflection_m() noexcept {
            return bristle_deflection_m_;
        }

        [[nodiscard]] std::span<const double> bristle_deflection_m()
                const noexcept {
            return bristle_deflection_m_;
        }

        [[nodiscard]] double kappa_prime() const noexcept {
            return kappa_prime_;
        }

    private:
        std::size_t n_elements_ = 0;
        std::vector<double> bristle_deflection_m_;
        // step() staging rows — sized n_elements at construction, never
        // escape the class (not semantic state): the advected/updated
        // deflection row (interp must finish reading the bristle row
        // before the commit writes it) and the clipped force row that
        // np.sum reduces.
        std::vector<double> updated_, force_n_;
        double kappa_prime_ = 0.0;
    };

} // namespace biketyre
