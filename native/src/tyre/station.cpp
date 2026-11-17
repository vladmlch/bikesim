// tyre/station.cpp — expression-for-expression port of the scalar
// station law in physics/distributed_tire.py (station_angles:99-104,
// station_density:125-139, station_response:142-148). Comments cite the
// source lines at the time of porting.
//
// NumPy lowering decisions:
//   2*np.pi*np.arange(count)/count — the Python scalar (2*pi) rounds
//     once, then one multiply and one divide per element, in that order.
//   np.full(count, 2*np.pi/count) — the scalar quotient (2*pi)/count
//     repeated verbatim.
//   max(a, 0.) / max(0., a) — std::max, whose `a < b ? b : a` is
//     CPython's `b if b > a else a` (same predicate, same signed-zero
//     result on equal inputs).
//   the knot fold `force += k*x; energy += k*x*x` — sequential `+=`
//     accumulation: pairwise_sum is the sequential fold below eight
//     terms, so the loop is bitwise the oracle's np.sum reduction.
#include "station.hpp"

#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/tyre_distributed.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <stdexcept>

namespace {
    bool fill_hinge_density_view(
        const nativecfg::HingeDensity &density,
        bike::physics_core::HingeDensityView<double> &view
    ) noexcept {
        const std::size_t knots = std::min(
            density.knots_m.size(), density.stiffness_n_m.size());
        if (knots > static_cast<std::size_t>(bike::physics_core::kMaxHingeKnots)) {
            return false;
        }
        view = {};
        view.knot_count = static_cast<bike::physics_core::i32>(knots);
        view.damping_ns_m = density.damping_ns_m;
        view.loss_factor = density.loss_factor;
        view.hysteresis_rate_eps_mps = density.hysteresis_rate_eps_mps;
        const std::span<double, bike::physics_core::kMaxHingeKnots> dst_k(
            view.knots_m);
        const std::span<double, bike::physics_core::kMaxHingeKnots> dst_s(
            view.stiffness_n_m);
        for (std::size_t i = 0; i < knots; ++i) {
            dst_k[i] = density.knots_m[i];
            dst_s[i] = density.stiffness_n_m[i];
        }
        return true;
    }
} // namespace

namespace biketyre {
    StationAngles station_angles(int count) {
        // distributed_tire.py:100-103 — ValueError domain: the bool /
        // non-int gates are the Python type contract; `int` is already
        // the C++ domain, leaving the two bounds.
        if (count < 16)
            throw std::invalid_argument(
                "at least sixteen tread stations required");
        if (count > 512)
            throw std::invalid_argument(
                "at most 512 tread stations supported");
        // `2*np.pi*np.arange(count)/count` evaluates the scalar `2*np.pi`
        // first (one rounding), multiplies elementwise, then divides
        // elementwise; `np.full(count, 2*np.pi/count)` repeats the scalar
        // quotient `(2*np.pi)/count`.
        const double two_pi = 2.0 * std::numbers::pi;
        const double c = static_cast<double>(count);
        const double weight = two_pi / c;
        StationAngles out;
        out.angles.reserve(static_cast<std::size_t>(count));
        out.weights.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            out.angles.push_back(two_pi * static_cast<double>(i) / c);
            out.weights.push_back(weight);
        }
        return out;
    }

    std::pair<double, double>
    station_density(double delta, const nativecfg::HingeDensity &density) {
        bike::physics_core::HingeDensityView<double> view{};
        if (fill_hinge_density_view(density, view)) {
            double force = 0.0;
            double energy = 0.0;
            auto err = bike::physics_core::ErrCode::none;
            const auto st = bike::physics_core::station_density<
                double, bike::physics_core::CpuExactToolkit>(
                    delta, view, force, energy, err);
            if (st != bike::physics_core::Status::ok)
                throw std::invalid_argument(
                    "contact penetration must be finite");
            return {force, energy};
        }
        // Fallback when knot count exceeds kMaxHingeKnots:
        if (!std::isfinite(delta))
            throw std::invalid_argument(
                "contact penetration must be finite");
        double force = 0.0;
        double energy = 0.0;
        const std::size_t knots = std::min(density.knots_m.size(),
                                           density.stiffness_n_m.size());
        for (std::size_t i = 0; i < knots; ++i) {
            const double x = std::max(delta - density.knots_m[i], 0.0);
            force += density.stiffness_n_m[i] * x;
            energy += density.stiffness_n_m[i] * x * x;
        }
        return {force, 0.5 * energy};
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror the Python API
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    std::pair<double, double>
    station_response(double delta, double rate, double weight,
                     const nativecfg::HingeDensity &density) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        bike::physics_core::HingeDensityView<double> view{};
        if (fill_hinge_density_view(density, view)) {
            double force = 0.0;
            double energy = 0.0;
            auto err = bike::physics_core::ErrCode::none;
            const auto st = bike::physics_core::station_response<
                double, bike::physics_core::CpuExactToolkit>(
                    delta, rate, weight, view, force, energy, err);
            if (st != bike::physics_core::Status::ok) {
                if (err == bike::physics_core::ErrCode::invalid_station_quad)
                    throw std::invalid_argument("invalid contact quadrature");
                throw std::invalid_argument(
                    "contact penetration must be finite");
            }
            return {force, energy};
        }
        if (!(std::isfinite(delta) && std::isfinite(rate) &&
              std::isfinite(weight)) ||
            weight < 0.0)
            throw std::invalid_argument("invalid contact quadrature");
        const auto [elastic, energy] = station_density(delta, density);
        const double force =
            delta > 0.0
                ? std::max(0.0, elastic +
                                density.loss_factor * elastic *
                                    std::tanh(rate / density.hysteresis_rate_eps_mps))
                : 0.0;
        return {weight * force, weight * energy};
    }
} // namespace biketyre
