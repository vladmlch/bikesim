// pneumatic/geometry.cpp — expression-for-expression port of the ray/road
// geometry kernels in sim/ride/tyre/geometry.py (RayRing.__init__ :99-117,
// intersect :147-196, _segment_distance :199-205, _brute_force :208-220,
// _hits :223-234, patches :237-253). Comments cite the oracle's lines at
// the time of porting.
//
// NumPy lowering decisions (each verified bitwise by
// tests/reference/test_native_pneumatic_geometry.py):
//   np.linspace(-h, h, n)     — function_base.py: delta = stop - start
//                               (one subtract), step = delta / (num - 1),
//                               y = arange * step + start (in-place
//                               multiply then in-place add: two roundings
//                               under -ffp-contract=off), y[-1] = stop.
//   ux = sin, uz = -cos       — a same-operand sin/cos pair per angle:
//                               numeric::np_sin / numeric::np_cos keep
//                               the two standalone libm calls the oracle
//                               makes (volatile-lowered; __sincos_stret
//                               fusion can drift one ulp — see the
//                               header).
//   np.arctan2                — std::atan2: the 'd' ufunc lowers to
//                               npy_atan2 = the platform libm call on this
//                               toolchain (the SVML dispatch requires
//                               AVX512-SKX; loops_umath_fp.dispatch.c.src).
//   np.floor / np.ceil        — std::floor / std::ceil (exact).
//   np.abs                    — std::fabs.
//   np.searchsorted 'right'   — manual binary-search probe over the lazily
//                               evaluated window angles: recomputing
//                               phi[j] = atan2(px[j]-cx, cz-pz[j]) is a
//                               pure function of the profile row, so the
//                               partition point is bitwise-identical and
//                               no per-call phi buffer is needed (the warm
//                               path stays allocation-free with no caller
//                               scratch contract).
//   np.diff(phi) > 0          — streaming adjacent-difference check.
//   np.flatnonzero / np.any / np.all / np.min / np.clip / np.where —
//                               ordered scans, flags and clamps; the
//                               per-ray min over segments is order-free.
//   np.errstate               — no-op: IEEE arithmetic traps nothing here.
//   np.maximum / np.minimum   — std::max/std::min on non-NaN operands (r
//                               is only ever finite-positive or +inf — the
//                               s > 0 gate drops every NaN before _hits).
#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "../numeric_sincos.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/tyre_pneumatic_geometry.hpp"

namespace biketyre {

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror geometry.py:99
    RayRing::RayRing(std::int64_t n_rays, double half_angle_rad) {
        // geometry.py:108-117 — same checks, same order (ray count first).
        if (n_rays < 3)
            throw std::invalid_argument(
                "a ray ring needs at least 3 rays");
        if (!(0.0 < half_angle_rad &&
              half_angle_rad < 0.5 * std::numbers::pi))
            throw std::invalid_argument(
                "half angle must be inside (0, pi/2)");
        n_ = static_cast<std::size_t>(n_rays);
        half_ = half_angle_rad;
        theta_.resize(n_);
        ux_.resize(n_);
        uz_.resize(n_);
        // np.linspace(-half, half, n): delta = np.subtract(stop, start),
        // step = delta / (num - 1), y = arange * step + start.
        const double delta = half_angle_rad - (-half_angle_rad);
        const double step = delta / static_cast<double>(n_ - 1);
        for (std::size_t i = 0; i < n_; ++i)
            theta_[i] = static_cast<double>(i) * step + (-half_angle_rad);
        theta_[n_ - 1] = half_angle_rad; // linspace pins y[-1] = stop
        // :115 — dtheta is theta[1] - theta[0], not the unrounded step.
        dtheta_ = theta_[1] - theta_[0];
        for (std::size_t i = 0; i < n_; ++i) {
            // :116-117 — one operand feeding both sin and cos: the
            // volatile-lowered calls keep __sincos_stret fusion out.
            ux_[i] = numeric::np_sin(theta_[i]);
            uz_[i] = -numeric::np_cos(theta_[i]);
        }
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror geometry.py:147
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror geometry.py:147
    void intersect_into(const RayRing &ring, double centre_x_m,
                        double centre_z_m, double radius_m,
                        const RoadProfile &road, RayHitBuffers &out) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const std::size_t n = ring.n();
        if (out.r_m.size() != n || out.delta_m.size() != n ||
            out.road_x_m.size() != n || out.road_z_m.size() != n)
            throw std::invalid_argument(
                "pneumatic hit buffers must match the ray ring width");
        const std::span<const double> xs = road.x_m();
        const std::span<const double> zs = road.z_m();
        const std::span<const double> ux = ring.ux();
        const std::span<const double> uz = ring.uz();

        // :163 — index_range(centre_x - R, centre_x + R): one subtract and
        // one add at this call's entry.
        const auto [lo, hi] = road.index_range(centre_x_m - radius_m,
                                               centre_x_m + radius_m);
        bike::physics_core::u8 coverage = 0;
        bike::physics_core::u8 airborne = 0;
        auto err = bike::physics_core::ErrCode::none;
        const auto st = bike::physics_core::pneumatic_intersect_window_arrays<
            double, bike::physics_core::CpuExactToolkit>(
                static_cast<bike::physics_core::i32>(n),
                ring.half_angle_rad(),
                ring.theta_rad().data(),
                ux.data(),
                uz.data(),
                centre_x_m,
                centre_z_m,
                radius_m,
                xs.data(),
                zs.data(),
                static_cast<bike::physics_core::i32>(lo),
                static_cast<bike::physics_core::i32>(hi),
                out.r_m.data(),
                out.delta_m.data(),
                out.road_x_m.data(),
                out.road_z_m.data(),
                coverage,
                airborne,
                err);
        if (st != bike::physics_core::Status::ok)
            throw std::invalid_argument("pneumatic intersect invalid input");
        out.coverage_event = coverage != 0;
        out.airborne = airborne != 0;
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror geometry.py:147
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror geometry.py:147
    RayHits intersect(const RayRing &ring, double centre_x_m,
                      double centre_z_m, double radius_m,
                      const RoadProfile &road) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const std::size_t n = ring.n();
        RayHits hits{.r_m = std::vector<double>(n),
                     .delta_m = std::vector<double>(n),
                     .road_x_m = std::vector<double>(n),
                     .road_z_m = std::vector<double>(n)};
        RayHitBuffers out{.r_m = hits.r_m,
                          .delta_m = hits.delta_m,
                          .road_x_m = hits.road_x_m,
                          .road_z_m = hits.road_z_m};
        intersect_into(ring, centre_x_m, centre_z_m, radius_m, road, out);
        hits.coverage_event = out.coverage_event;
        hits.airborne = out.airborne;
        return hits;
    }

    std::size_t patches_into(std::span<const double> delta_m,
                             std::span<std::pair<int, int>> out) {
        // geometry.py:247-253 — loaded = flatnonzero(delta > 0); runs
        // break where diff(loaded) > 1, reported as (start, stop)
        // exclusive pairs in ray order.
        std::size_t count = 0;
        bool open = false;
        std::size_t start = 0, prev = 0;
        const auto emit = [&](std::size_t last) {
            if (count >= out.size())
                throw std::invalid_argument(
                    "pneumatic patch buffer too small");
            out[count] = {static_cast<int>(start),
                          static_cast<int>(last) + 1};
            ++count;
        };
        for (std::size_t k = 0; k < delta_m.size(); ++k) {
            if (!(delta_m[k] > 0.0))
                continue;
            if (!open) {
                start = k;
                open = true;
            } else if (k - prev > 1) {
                emit(prev);
                start = k;
            }
            prev = k;
        }
        if (open)
            emit(prev);
        return count;
    }

    std::vector<std::pair<int, int>>
    patches(std::span<const double> delta_m) {
        // The disjoint positive-run count never exceeds ceil(n / 2).
        std::vector<std::pair<int, int>> out((delta_m.size() + 1) / 2);
        out.resize(patches_into(delta_m, out));
        return out;
    }

} // namespace biketyre
