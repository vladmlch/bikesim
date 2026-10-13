// pneumatic/profile.hpp — the pneumatic tyre's road profile: a port of
// RoadProfile (sim/ride/tyre/geometry.py:38-85), the uniform-grid world
// polyline the tyre's rays intersect. The native wire carries the resolved
// profile as nativecfg::PneumaticProfile {x0_m, dx_m, z_m}
// (config_types.hpp:300-303); this class wraps the same payload.
//
// Bitwise contract beyond the ordinary scalar expressions:
//   x_m / xs windows     — x0_m + dx_m * arange: one multiply then one add
//     per element under the target-wide -ffp-contract=off (an FMA would
//     round once instead of twice and land up to one ulp off). The grid is
//     materialized once at construction; the oracle recomputes the same
//     expression into per-call arrays, so the stored row is bitwise-equal.
//   np.floor / np.ceil   — std::floor/std::ceil: exactly-rounded lowering
//     ops, no libm variance.
//   int(v)               — the floor/ceil results are integral doubles;
//     casting to int64 is exact inside range. The value is first clamped
//     into [-4.5e18, 4.5e18] so the cast stays defined for absurd queries —
//     Python's unbounded ints would keep the full value, but every index
//     beyond that bound still falls outside [0, n), so the observable
//     (clamped, sliced) behavior is unchanged.
//   int(NaN) / int(±inf) — the oracle raises ValueError/OverflowError at
//     the int() cast; the port throws std::invalid_argument at the same
//     boundary (before clamping — Python never reaches the max/min).
//   np.interp scalar     — biketyre::np_interp (tyre/profile.hpp: numpy's
//     single-query fma interpolation path, end-clamped).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "../config_types.hpp" // nativecfg::PneumaticProfile
#include "../tyre/profile.hpp" // biketyre::np_interp

namespace biketyre {
    // The road as a polyline on a uniform longitudinal grid in world
    // coordinates (geometry.py:38-51): x0_m is the world X of the first
    // sample, dx_m the grid spacing, z_m the world Z of the road surface
    // at each sample. Owns its storage — same convention as SignedProfile.
    class RoadProfile {
    public:
        // RoadProfile(x0_m, dx_m, z_m) with the domain the wire parser and
        // RoadProfile.from_samples already enforce upstream: finite x0,
        // positive finite dx, and at least two finite heights (the
        // validated wire domain — config_validation.hpp:368-373).
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror geometry.py:49-51
        // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the oracle dataclass
        RoadProfile(double x0_m, double dx_m, std::vector<double> z_m)
            : x0_(x0_m), dx_(dx_m), z_(std::move(z_m)) {
            // NOLINTEND(bugprone-easily-swappable-parameters)
            if (!std::isfinite(x0_) || !std::isfinite(dx_) || !(dx_ > 0.0))
                throw std::invalid_argument(
                    "road profile needs finite x0_m and positive dx_m");
            if (z_.size() < 2)
                throw std::invalid_argument(
                    "road profile needs at least two samples");
            for (const double z: z_)
                if (!std::isfinite(z))
                    throw std::invalid_argument(
                        "road profile heights must be finite");
            x_.resize(z_.size());
            // x_m = x0_m + dx_m * np.arange(n) (geometry.py:74): one
            // multiply then one add per element — the same expression the
            // oracle's property and intersect's xs window evaluate.
            for (std::size_t i = 0; i < z_.size(); ++i)
                x_[i] = x0_ + dx_ * static_cast<double>(i);
        }

        // Wraps the resolved wire profile — the same {x0_m, dx_m, z_m}
        // payload nativecfg::PneumaticProfile carries.
        explicit RoadProfile(const nativecfg::PneumaticProfile &wire)
            : RoadProfile(wire.x0_m, wire.dx_m, wire.z_m) {}

        [[nodiscard]] double x0_m() const noexcept { return x0_; }
        [[nodiscard]] double dx_m() const noexcept { return dx_; }
        [[nodiscard]] std::size_t size() const noexcept { return z_.size(); }

        // RoadProfile.x_m (geometry.py:71-74) — the materialized grid, one
        // x0 + dx*k entry per sample.
        [[nodiscard]] std::span<const double> x_m() const noexcept {
            return x_;
        }

        [[nodiscard]] std::span<const double> z_m() const noexcept {
            return z_;
        }

        // RoadProfile.height_at (geometry.py:76-78) — np.interp's scalar
        // path over the materialized grid, end-clamped like the oracle.
        [[nodiscard]] double height_at(double x) const {
            return np_interp(x, x_, z_);
        }

        // RoadProfile.index_range (geometry.py:80-85): the [lo, hi) sample
        // indices covering [x_lo_m, x_hi_m], returned as
        // (max(0, floor((x_lo-x0)/dx)), min(n, ceil((x_hi-x0)/dx) + 1)).
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror geometry.py:80
        // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
        [[nodiscard]] std::pair<std::int64_t, std::int64_t>
        index_range(double x_lo_m, double x_hi_m) const {
            // NOLINTEND(bugprone-easily-swappable-parameters)
            const double lo_f = std::floor((x_lo_m - x0_) / dx_);
            const double hi_f = std::ceil((x_hi_m - x0_) / dx_);
            if (!std::isfinite(lo_f) || !std::isfinite(hi_f))
                throw std::invalid_argument(
                    "profile index window is not finite");
            constexpr double kIndexBound = 4.5e18; // < 2^63
            const std::int64_t lo = static_cast<std::int64_t>(
                std::clamp(lo_f, -kIndexBound, kIndexBound));
            const std::int64_t hi = static_cast<std::int64_t>(
                    std::clamp(hi_f, -kIndexBound, kIndexBound)) +
                                    1;
            const std::int64_t n = static_cast<std::int64_t>(z_.size());
            return {std::max<std::int64_t>(0, lo),
                    std::min<std::int64_t>(n, hi)};
        }

    private:
        double x0_, dx_;
        std::vector<double> z_, x_;
    };
} // namespace biketyre
