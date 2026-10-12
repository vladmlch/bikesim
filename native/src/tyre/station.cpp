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

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

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
        // distributed_tire.py:131-132 — the oracle's finite-penetration
        // gate; material validity is HingeDensity's construction contract.
        if (!std::isfinite(delta))
            throw std::invalid_argument(
                "contact penetration must be finite");
        // lines 133-139 — the sequential fold over
        // zip(knots_m, stiffness_n_m): zip truncates to the shorter
        // column, mirrored by the min bound (config validation already
        // keeps the two vectors equal-length, so this is a faithful zip).
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
        // distributed_tire.py:144-145 — finite quadrature triple plus a
        // nonnegative weight, in the oracle's clause order.
        if (!(std::isfinite(delta) && std::isfinite(rate) &&
              std::isfinite(weight)) ||
            weight < 0.0)
            throw std::invalid_argument("invalid contact quadrature");
        // lines 146-148 — density first, then the gated damped force;
        // the weight multiplies force and energy alike.
        const auto [elastic, energy] = station_density(delta, density);
        const double force =
            delta > 0.0
                ? std::max(0.0, elastic + density.damping_ns_m * rate)
                : 0.0;
        return {weight * force, weight * energy};
    }
} // namespace biketyre
