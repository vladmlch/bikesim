// pneumatic/geometry.hpp — the pneumatic tyre's ray/road geometry: a port
// of RayRing, RayHits, intersect and patches from
// sim/ride/tyre/geometry.py (lines 88-253).
//
// Each wheel carries `n` rays fixed in the world frame, spread uniformly
// over +/-half_angle about world -Z; intersect() finds the distance from
// the hub to the first visible road point along every ray, the element
// deflection max(R - r, 0), the per-ray road point, and the coverage /
// airborne flags. patches() splits the loaded rays into contiguous runs.
//
// Divergence guide:
//  - pneumatic/geometry.cpp — implementation; its header comment lists the
//    NumPy lowering decisions that keep the port bitwise-identical.
//  - src/bike_sim/sim/ride/tyre/geometry.py — the behavioral oracle.
//  - tests/reference/test_native_pneumatic_geometry.py — the bitwise
//    arbiter.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "profile.hpp"

namespace biketyre {
    // COVERAGE_SLACK_RAD (geometry.py:35) — profile samples inside the
    // tyre circle whose polar angle exceeds the coverage by more than this
    // are reported as a coverage event rather than silently ignored.
    inline constexpr double kCoverageSlackRad = 1e-9;

    // RayRing (geometry.py:88-117) — the world-fixed ray directions of one
    // tyre: angles from -Z towards +X, increasing; the ray at theta has
    // direction (sin theta, -cos theta) in world (x, z). Owns the three
    // per-ray rows, sized once at construction.
    class RayRing {
    public:
        // RayRing.__init__ (geometry.py:99-117): rejects n_rays < 3 and a
        // coverage reaching the horizontal, like the oracle's ValueError.
        RayRing(std::int64_t n_rays, double half_angle_rad);

        [[nodiscard]] std::size_t n() const noexcept { return n_; }

        [[nodiscard]] double half_angle_rad() const noexcept {
            return half_;
        }

        [[nodiscard]] double dtheta_rad() const noexcept { return dtheta_; }

        [[nodiscard]] std::span<const double> theta_rad() const noexcept {
            return theta_;
        }

        [[nodiscard]] std::span<const double> ux() const noexcept {
            return ux_;
        }

        [[nodiscard]] std::span<const double> uz() const noexcept {
            return uz_;
        }

    private:
        std::size_t n_ = 0;
        double half_ = 0.0, dtheta_ = 0.0;
        std::vector<double> theta_, ux_, uz_;
    };

    // RayHits (geometry.py:120-140) — owned per-ray row set for one
    // intersect() call: distance to the first visible road point (inf
    // where the ray meets no road within the radius), element deflection,
    // world position of each ray's road point, and the step's flags.
    struct RayHits {
        std::vector<double> r_m, delta_m, road_x_m, road_z_m;
        bool coverage_event = false;
        bool airborne = false;
    };

    // The warm-path twin of RayHits: caller-owned output rows the kernel
    // writes, each exactly ring.n() wide, plus the two flag slots. The
    // spans stay caller-owned — intersect_into allocates nothing.
    struct RayHitBuffers {
        std::span<double> r_m, delta_m, road_x_m, road_z_m;
        bool coverage_event = false;
        bool airborne = false;
    };

    // intersect (geometry.py:147-196) writing into caller storage — the
    // allocation-free path the writer's warm tick uses. Throws
    // std::invalid_argument when a buffer is not ring.n() wide, or the
    // index window is non-finite (the oracle's ValueError/OverflowError
    // boundary — see profile.hpp).
    void intersect_into(const RayRing &ring, double centre_x_m,
                        double centre_z_m, double radius_m,
                        const RoadProfile &road, RayHitBuffers &out);

    // The allocating convenience twin — identical values, owned storage.
    [[nodiscard]] RayHits intersect(const RayRing &ring, double centre_x_m,
                                    double centre_z_m, double radius_m,
                                    const RoadProfile &road);

    // patches (geometry.py:237-253): contiguous runs of loaded rays
    // (delta > 0) as (start, stop) pairs — stop exclusive — in ray order.
    [[nodiscard]] std::vector<std::pair<int, int>>
    patches(std::span<const double> delta_m);

    // The caller-buffer twin of patches(): `out` must hold at least
    // ceil(delta_m.size() / 2) pairs — the maximum disjoint-run count —
    // and the written count is returned. Throws std::invalid_argument on
    // an undersized buffer instead of writing out of bounds.
    std::size_t patches_into(std::span<const double> delta_m,
                             std::span<std::pair<int, int>> out);
} // namespace biketyre
