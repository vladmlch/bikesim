// pneumatic/brush.cpp — implementation of pneumatic/brush.hpp, a
// bitwise port of sim/ride/tyre/brush.py plus the SurfaceSpec::mu
// Stribeck helper from terrain/surface.py.
//
// CPython/NumPy lowering decisions (each verified against CPython
// 3.13.16 / numpy 2.5.2 on this platform):
//
//   math.isfinite / math.isinf     -> std::isfinite / std::isinf
//   abs(float) / np.abs            -> std::fabs
//   math.copysign(1.0, x)          -> std::copysign — both lower to
//                                     the same libm copysign
//   math.exp(x)                    -> std::exp — CPython math_exp wraps
//                                     libm exp and raises OverflowError
//                                     only on overflow to inf; the
//                                     relax exponent is -|v_x|*dt/sigma
//                                     with validated operands, so it is
//                                     <= -0.0 always and can never
//                                     overflow (it can be -inf through
//                                     a |v_x|*dt product overflow, in
//                                     which case exp(-inf) = +0.0)
//   np.exp(scalar) in SurfaceSpec.mu -> std::exp (DOUBLE_exp lowers to
//                                     npy_exp -> the same libm exp on
//                                     aarch64; the argument is
//                                     -|v|/stribeck <= -0.0, so the
//                                     overflow warning path is
//                                     unreachable)
//   x ** 2 on a Python float       -> py_pow2 below: CPython
//                                     float.__pow__ = libm pow(iv, 2.0)
//                                     through the opaque pyfloat::pow
//                                     call site (no x*x fold), wrapped
//                                     in float_pow's edge cases —
//                                     nan -> the input payload verbatim,
//                                     +-inf -> +inf, and an infinite
//                                     result from finite input raises
//                                     OverflowError (errno ERANGE).
//                                     Underflow to subnormal/zero does
//                                     NOT raise (verified: 1e-160**2 ->
//                                     1e-320, 1e-170**2 -> 0.0).
//   max(0.0, x)                    -> std::max(0.0, x) — builtin max
//                                     returns the first operand on a
//                                     tie, matching std::max(a,b)'s
//                                     (a<b ? b : a); on -0.0 it yields
//                                     +0.0 either way
//   np.interp(x, arange(n), v, 0,0) -> numeric::np_interp_unit_grid
//                                     (arr_interp on integer knots,
//                                     verified bitwise in Task 8)
//   np.clip(x, -l, l)              -> numeric::np_clip — the umath
//                                     DOUBLE_clip generic loop:
//                                     _NPY_MIN(_NPY_MAX(x, lo), hi) with
//                                     PyArray_MIN/MAX ternary semantics
//                                     (clip.cpp:148-153, 239-246). NOT
//                                     np_maximum/np_minimum composition:
//                                     clip(+0.0, -0.0, hi) yields -0.0
//                                     and sNaN inputs pass through
//                                     unquieted — verified bitwise
//                                     against numpy 2.5.2
//   np.where(c, a, b)              -> the ternary (elementwise select)
//   np.sum(a)                      -> numeric::numpy_sum (np.sum reduce
//                                      identity over pairwise order)
//   np.all / np.any / .any()       -> first-false / first-true scans
//   np.isfinite(a) / a < 0.0       -> std::isfinite / <
//   np.asarray(x, float)           -> the contiguous f64 input span
//   a[i] = masked write            -> per-element read-then-write; each
//                                     lane's update depends only on its
//                                     own pre-write values, so the fused
//                                     loop is the oracle's whole-row
//                                     temporaries streamed one lane
//
// Scalarization freedom: every numpy elementwise expression is
// evaluated per-element in the same order as the oracle's array op;
// scalar broadcasts (filtered_v_s * dt_s, the damping coefficient) are
// evaluated once like the oracle's scalar expressions.

#include "brush.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>

#include "../numeric_numpy.hpp"
#include "../numeric_sum.hpp"
#include "../writers/pyfloat.hpp"

namespace biketyre {
    namespace {
        // CPython float_pow(iv, 2.0) (Objects/floatobject.c): the
        // pre-pow special cases plus the post-pow ERANGE check, fixed
        // at the even integer exponent the oracle uses for
        // `half_length_m ** 2` (brush.py:85,142,253).
        //   iv NaN        -> iv, payload preserved (float_pow returns
        //                    the input NaN, not a libm-quieted one)
        //   iv +-inf      -> +inf (iw even), no raise
        //   result +-inf  -> OverflowError("math range error"), the
        //                    errno == ERANGE path — reachable: e.g.
        //                    half_length_m = 1e200 inside a valid patch
        [[nodiscard]] double py_pow2(double iv) {
            if (std::isnan(iv)) return iv;
            if (std::isinf(iv))
                return std::numeric_limits<double>::infinity();
            const double r = pyfloat::pow(iv, 2.0);
            if (std::isinf(r))
                throw std::overflow_error("math range error");
            return r;
        }
    } // namespace

    double surface_mu(const nativecfg::SurfaceSpec &surface,
                      double sliding_speed_mps) {
        // surface.py:88 — array(sliding_speed_mps, ...): the oracle
        // rejects a non-finite argument before evaluating the curve.
        if (!std::isfinite(sliding_speed_mps))
            throw std::invalid_argument(
                    "invalid shape or non-finite "
                    "SurfaceSpec.sliding_speed_mps");
        // surface.py:89-90 — np.exp lowers to npy_exp -> libm exp.
        const double decay =
                std::exp(-std::fabs(sliding_speed_mps) /
                         surface.stribeck_speed_mps);
        const double value =
                surface.mu_slide +
                (surface.mu_peak - surface.mu_slide) * decay;
        // surface.py:91 — derived_array(value, 'SurfaceSpec.mu') raises
        // OverflowError on a non-finite coefficient.
        if (!std::isfinite(value))
            throw std::overflow_error(
                    "SurfaceSpec.mu: non-finite derived array");
        return value;
    }

    double relax(double kappa_prime, double v_x_mps, double v_s_mps,
                 double sigma_m, double dt_s) {
        // brush.py:52-56 — the finite gate precedes the domain checks.
        if (!std::isfinite(kappa_prime) || !std::isfinite(v_x_mps) ||
            !std::isfinite(v_s_mps) || !std::isfinite(sigma_m) ||
            !std::isfinite(dt_s))
            throw std::invalid_argument(
                    "slip relaxation inputs must be finite");
        if (sigma_m < 0.0 || dt_s < 0.0)
            throw std::invalid_argument(
                    "relaxation length and time step must not be "
                    "negative");

        const double speed_mps = std::fabs(v_x_mps);
        if (sigma_m == 0.0) {
            if (speed_mps > 0.0)
                return -v_s_mps / speed_mps;
            // No rolling distance: the ratio has no finite algebraic
            // target; the state holds (brush.py:62-64).
            return kappa_prime;
        }
        if (speed_mps == 0.0)
            return kappa_prime - v_s_mps * dt_s / sigma_m;

        const double steady_kappa = -v_s_mps / speed_mps;
        const double exponent = -speed_mps * dt_s / sigma_m;
        return steady_kappa +
               (kappa_prime - steady_kappa) * std::exp(exponent);
    }

    double sigma_x(double kappa_prime) noexcept {
        // brush.py:90-94 — check order preserved: the locked-wheel
        // limit first, then the isinf copysign (only +inf reaches it),
        // then the ratio.
        if (kappa_prime <= -1.0)
            return -std::numeric_limits<double>::infinity();
        if (std::isinf(kappa_prime))
            return std::copysign(1.0, kappa_prime);
        return kappa_prime / (1.0 + kappa_prime);
    }

    double tread_stiffness_n_m2(
            double normal_load_n, double half_length_m,
            const nativecfg::SurfaceSpec &surface,
            std::optional<double> explicit_stiffness_n_m2) {
        // brush.py:80-85.
        if (explicit_stiffness_n_m2.has_value()) {
            const double value = *explicit_stiffness_n_m2;
            if (!std::isfinite(value) || value < 0.0)
                throw std::invalid_argument(
                        "tread stiffness must be finite and "
                        "nonnegative, got " +
                        std::to_string(value));
            return value;
        }
        // `half_length_m ** 2` is CPython float.__pow__ — the
        // overflow-raising libm pow, not a bare multiply.
        return surface.slip_stiffness_per_load * normal_load_n /
               (2.0 * py_pow2(half_length_m));
    }

    BrushResult lumped_brush(
            double kappa_prime, double normal_load_n,
            double half_length_m, const nativecfg::SurfaceSpec &surface,
            std::optional<double> explicit_stiffness_n_m2,
            double sliding_speed_mps) {
        // brush.py:122-133 — validation order preserved verbatim. An
        // infinite kappa_prime alone does not raise; it flows through
        // _sigma_x's copysign branch.
        if (!std::isfinite(kappa_prime) ||
            !std::isfinite(normal_load_n) ||
            !std::isfinite(half_length_m) ||
            !std::isfinite(sliding_speed_mps)) {
            if (!std::isinf(kappa_prime))
                throw std::invalid_argument(
                        "lumped brush inputs must be finite");
            if (!std::isfinite(normal_load_n) ||
                !std::isfinite(half_length_m) ||
                !std::isfinite(sliding_speed_mps))
                throw std::invalid_argument(
                        "load, patch length and sliding speed must be "
                        "finite");
        }
        if (normal_load_n < 0.0)
            throw std::invalid_argument(
                    "normal load must not be negative");
        if (normal_load_n == 0.0)
            return {.force_n = 0.0,
                    .fully_sliding = false,
                    .friction_coefficient =
                            surface_mu(surface, sliding_speed_mps)};
        if (half_length_m <= 0.0)
            throw std::invalid_argument(
                    "a loaded patch must have positive half-length");

        const double mu = surface_mu(surface, sliding_speed_mps);
        const double c_px =
                tread_stiffness_n_m2(normal_load_n, half_length_m,
                                     surface, explicit_stiffness_n_m2);
        if (c_px == 0.0)
            return {.force_n = 0.0,
                    .fully_sliding = false,
                    .friction_coefficient = mu};

        const double theta = 2.0 * c_px *
                             py_pow2(half_length_m) /
                             (3.0 * mu * normal_load_n);
        const double z = theta * sigma_x(kappa_prime);
        if (std::fabs(z) >= 1.0)
            return {.force_n = mu * normal_load_n *
                              std::copysign(1.0, z),
                    .fully_sliding = true,
                    .friction_coefficient = mu};
        const double force = 3.0 * mu * normal_load_n * z *
                             (1.0 - std::fabs(z) + z * z / 3.0);
        return {.force_n = force,
                .fully_sliding = false,
                .friction_coefficient = mu};
    }

    // ---------------------------------------------------------------
    // DiscretisedBrush (brush.py:150-272)
    // ---------------------------------------------------------------

    DiscretisedBrush::DiscretisedBrush(std::int64_t n_elements) {
        // brush.py:161-163 — "a discretised patch needs at least 1
        // element, got {n}".
        if (n_elements < 1)
            throw std::invalid_argument(
                    "a discretised patch needs at least 1 element, got " +
                    std::to_string(n_elements));
        n_elements_ = static_cast<std::size_t>(n_elements);
        bristle_deflection_m_.assign(n_elements_, 0.0);
        updated_.assign(n_elements_, 0.0);
        force_n_.assign(n_elements_, 0.0);
    }

    void DiscretisedBrush::reset() noexcept {
        std::ranges::fill(bristle_deflection_m_, 0.0);
        kappa_prime_ = 0.0;
    }

    void DiscretisedBrush::assign(const DiscretisedBrush &source) {
        if (source.n_elements_ != n_elements_)
            throw std::invalid_argument(
                    "discretised brush element count mismatch");
        std::ranges::copy(source.bristle_deflection_m_,
                          bristle_deflection_m_.begin());
        kappa_prime_ = source.kappa_prime_;
    }

    BrushResult DiscretisedBrush::step(
            std::span<const double> normal_loads_n,
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) oracle keyword order
            double half_length_m, double v_x_mps, double v_s_mps,
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) oracle keyword order
            double sigma_m, const nativecfg::SurfaceSpec &surface,
            double dt_s) {
        const std::size_t n = n_elements_;
        // brush.py:200-209 — every check precedes any mutation; a
        // rejected call leaves bristles and kappa_prime byte-identical.
        if (normal_loads_n.size() != n)
            throw std::invalid_argument(
                    "expected " + std::to_string(n) +
                    " normal loads, got (" +
                    std::to_string(normal_loads_n.size()) + ",)");
        for (const double load : normal_loads_n) {
            if (!std::isfinite(load) || load < 0.0)
                throw std::invalid_argument(
                        "bristle loads must be finite and nonnegative");
        }
        if (!std::isfinite(half_length_m) || !std::isfinite(v_x_mps) ||
            !std::isfinite(v_s_mps) || !std::isfinite(sigma_m) ||
            !std::isfinite(dt_s))
            throw std::invalid_argument(
                    "brush kinematics and dimensions must be finite");
        if (half_length_m < 0.0 || sigma_m < 0.0 || dt_s <= 0.0)
            throw std::invalid_argument(
                    "patch length and relaxation length must be "
                    "nonnegative; dt must be positive");

        const double total_load_n =
                numeric::numpy_sum(normal_loads_n);
        const double mu = surface_mu(surface, std::fabs(v_s_mps));
        if (total_load_n <= 0.0 || half_length_m <= 0.0) {
            // brush.py:213-215 — the zero-load path DOES mutate:
            // reset() clears bristles and transient slip.
            reset();
            return {.force_n = 0.0,
                    .fully_sliding = false,
                    .friction_coefficient = mu};
        }

        // brush.py:217-220 — the separate input filter uses
        // max(0, sigma - a); kappa_prime commits right after relax
        // returns, like the oracle's attribute assignment.
        const double input_sigma_m =
                std::max(0.0, sigma_m - half_length_m);
        kappa_prime_ =
                relax(kappa_prime_, v_x_mps, v_s_mps, input_sigma_m,
                      dt_s);

        const double dx_m =
                2.0 * half_length_m / static_cast<double>(n);
        // brush.py:225-229 — the filter supplies the slip ratio during
        // rolling; at rest the measured V_s integrates directly. The
        // v_x != 0.0 test keeps -0.0 on the else branch (-0.0 == 0.0).
        const double filtered_v_s_mps =
                v_x_mps != 0.0 ? -kappa_prime_ * std::fabs(v_x_mps)
                               : v_s_mps;
        // brush.py:233-247 — advection at the wheel's measured tread
        // speed through the unit-grid np.interp, or the verbatim copy.
        const double wheel_surface_speed_mps = v_x_mps - v_s_mps;
        const std::span<const double> bristle = bristle_deflection_m_;
        if (wheel_surface_speed_mps != 0.0) {
            // source_index = arange(n) + (wheel*dt)/dx — the scalar
            // shift is evaluated once, like the oracle's expression.
            const double shift =
                    wheel_surface_speed_mps * dt_s / dx_m;
            for (std::size_t i = 0; i < n; ++i)
                updated_[i] = numeric::np_interp_unit_grid(
                        static_cast<double>(i) + shift, bristle, 0.0,
                        0.0);
        } else {
            std::ranges::copy(bristle, updated_.begin());
        }

        // brush.py:249-251 — updated = advected - filtered*dt, then
        // where(active, updated, 0.0). The product is the oracle's
        // scalar broadcast, evaluated once.
        const double growth_m = filtered_v_s_mps * dt_s;
        for (std::size_t i = 0; i < n; ++i) {
            const double upd = updated_[i] - growth_m;
            updated_[i] = normal_loads_n[i] > 0.0 ? upd : 0.0;
        }

        // brush.py:253-259 — stiffness and the dashpot term
        // np.where(active, k*tau*(-filtered), 0.0); the damped value is
        // a scalar broadcast like the oracle's.
        const double c_px =
                surface.slip_stiffness_per_load * total_load_n /
                (2.0 * py_pow2(half_length_m));
        const double element_stiffness_n_m = c_px * dx_m;
        const double damping_active_n =
                element_stiffness_n_m * kBristleDampingS *
                (-filtered_v_s_mps);

        // brush.py:260-269 — per-element trial, friction clip, sliding
        // mask and the sliding-bristle write-back, fused lane by lane.
        // The oracle's `sliding.any()` gate on the write-back is
        // element-local: a lane rewrites updated only when its own
        // sliding bit is set, which is exactly the mask.
        bool any_active = false;
        bool all_active_sliding = true;
        for (std::size_t i = 0; i < n; ++i) {
            const bool active = normal_loads_n[i] > 0.0;
            const double damping = active ? damping_active_n : 0.0;
            const double trial =
                    element_stiffness_n_m * updated_[i] + damping;
            const double limit = mu * normal_loads_n[i];
            const double force =
                    numeric::np_clip(trial, -limit, limit);
            force_n_[i] = force;
            const bool sliding =
                    active && std::fabs(trial) >= limit;
            any_active = any_active || active;
            if (active)
                all_active_sliding = all_active_sliding && sliding;
            if (element_stiffness_n_m > 0.0 && sliding)
                updated_[i] = (force - damping) /
                              element_stiffness_n_m;
        }
        // brush.py:264 — active.any() and np.all(sliding[active]).
        const bool fully_sliding = any_active && all_active_sliding;

        // brush.py:270 — self.bristle_deflection_m[:] = updated.
        std::ranges::copy(updated_, bristle_deflection_m_.begin());
        return {.force_n = numeric::numpy_sum(force_n_),
                .fully_sliding = fully_sliding,
                .friction_coefficient = mu};
    }

} // namespace biketyre
