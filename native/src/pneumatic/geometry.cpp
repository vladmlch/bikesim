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

namespace {
    constexpr double kInf = std::numeric_limits<double>::infinity();

    // _cross (geometry.py:143-144): ax*bz - az*bx — one multiply, one
    // multiply, one subtract per element under -ffp-contract=off.
    constexpr double cross2(double ax, double az, double bx, double bz) {
        return ax * bz - az * bx;
    }

    // np.searchsorted(phi, v, side='right') (geometry.py:187) over the
    // lazily-evaluated window angles — the first index j in [0, count)
    // with phi[j] > v, where phi[j] = atan2(px[j] - cx, cz - pz[j]) at
    // global profile index base + j. The window is monotone-increasing
    // here — the caller only reaches this after verifying all(diff > 0) —
    // so the partition point is unique and well-defined.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror the oracle's slices
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the oracle's slices
    std::size_t phi_upper_bound(std::span<const double> xs,
                                std::span<const double> zs,
                                std::size_t base, std::size_t count,
                                double cx, double cz, double v) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        std::size_t first = 0, len = count;
        while (len > 0) {
            const std::size_t step = len / 2;
            const std::size_t mid = first + step;
            const double phi_mid =
                    std::atan2(xs[base + mid] - cx, cz - zs[base + mid]);
            if (v < phi_mid) {
                len = step;
            } else {
                first = mid + 1;
                len -= step + 1;
            }
        }
        return first;
    }
} // namespace

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

        bool coverage = false;
        bool airborne = false;

        // :163 — index_range(centre_x - R, centre_x + R): one subtract and
        // one add at this call's entry.
        const auto [lo, hi] = road.index_range(centre_x_m - radius_m,
                                               centre_x_m + radius_m);
        if (hi - lo < 2) {
            // :164-165 — narrow-window early-out: all rays inf, airborne.
            std::ranges::fill(out.r_m, kInf);
            airborne = true;
        } else {
            const std::size_t glo = static_cast<std::size_t>(lo);
            const std::size_t w = static_cast<std::size_t>(hi - lo);
            const double r2 = radius_m * radius_m;
            // :169-173 — the inside-circle mask over the window; the
            // oracle's flatnonzero reduces to the first/last inside
            // indices, which is all the window slice needs.
            std::size_t first = w, last = 0;
            for (std::size_t k = 0; k < w; ++k) {
                const std::size_t g = glo + k;
                const double dxs = xs[g] - centre_x_m;
                const double dzs = centre_z_m - zs[g];
                if (dxs * dxs + dzs * dzs < r2) {
                    if (first == w)
                        first = k;
                    last = k;
                }
            }
            if (first == w) {
                // :172-173 — no sample inside the circle: airborne.
                std::ranges::fill(out.r_m, kInf);
                airborne = true;
            } else {
                // :176-179 — the window padded one sample either side and
                // its polar angles from the hub. `a`/`b` are relative to
                // the window start `lo` exactly like the oracle's slice.
                const std::size_t a = first > 0 ? first - 1 : 0;
                const std::size_t b = std::min(last + 2, w);
                const std::size_t base = glo + a;
                const std::size_t win = b - a;
                // :181-184 — coverage: an inside sample outside the ray
                // coverage by more than the slack, or at/above the hub.
                const double cov_limit =
                        ring.half_angle_rad() + kCoverageSlackRad;
                bool monotone = true;
                double prev_phi = 0.0;
                for (std::size_t j = 0; j < win; ++j) {
                    const std::size_t g = base + j;
                    const double px = xs[g], pz = zs[g];
                    const double phi = std::atan2(px - centre_x_m,
                                                  centre_z_m - pz);
                    const double dxs = px - centre_x_m;
                    const double dzs = centre_z_m - pz;
                    if (dxs * dxs + dzs * dzs < r2) {
                        if (std::fabs(phi) > cov_limit)
                            coverage = true;
                        if (pz >= centre_z_m)
                            coverage = true;
                    }
                    // :186 — np.all(np.diff(phi) > 0.0) as a streaming
                    // adjacent-difference check over the window.
                    if (j > 0 && !(phi - prev_phi > 0.0))
                        monotone = false;
                    prev_phi = phi;
                }
                if (win >= 2 && monotone) {
                    // :187-192 — the monotonic fast path: each ray's
                    // bracketing segment comes from searchsorted 'right',
                    // then the exact line intersection distance.
                    for (std::size_t i = 0; i < n; ++i) {
                        const std::size_t ub = phi_upper_bound(
                            xs, zs, base, win, centre_x_m, centre_z_m,
                            ring.theta_rad()[i]);
                        const std::ptrdiff_t j =
                            static_cast<std::ptrdiff_t>(ub) - 1;
                        const bool valid =
                            j >= 0 &&
                            j < static_cast<std::ptrdiff_t>(win) - 1;
                        // np.clip(j, 0, px.size - 2): valid rows read the
                        // bracket, invalid rows are masked afterwards —
                        // the clipped segment still gets evaluated like
                        // the oracle's vectorized _segment_distance.
                        const std::size_t jj = static_cast<std::size_t>(
                            std::clamp(j, std::ptrdiff_t{0},
                                       static_cast<std::ptrdiff_t>(win) -
                                           2));
                        const std::size_t g = base + jj;
                        const double x1 = xs[g], z1 = zs[g];
                        const double dx = xs[g + 1] - x1;
                        const double dz = zs[g + 1] - z1;
                        const double denom = cross2(ux[i], uz[i], dx, dz);
                        const double s =
                            cross2(x1 - centre_x_m, z1 - centre_z_m, dx,
                                   dz) /
                            denom;
                        // where(|denom| > 1e-15, s, inf) composed with
                        // where(valid & s > 0, s, inf): a degenerate pair
                        // contributes inf either way.
                        out.r_m[i] =
                            valid && std::fabs(denom) > 1e-15 && s > 0.0
                                ? s
                                : kInf;
                    }
                } else {
                    // :194 → _brute_force (:208-220): the exact first
                    // crossing of every ray with every window segment.
                    const std::size_t segs = win - 1;
                    for (std::size_t i = 0; i < n; ++i) {
                        double best = kInf;
                        for (std::size_t k = 0; k < segs; ++k) {
                            const std::size_t g = base + k;
                            const double x1 = xs[g], z1 = zs[g];
                            const double dx = xs[g + 1] - x1;
                            const double dz = zs[g + 1] - z1;
                            const double denom =
                                cross2(ux[i], uz[i], dx, dz);
                            const double wx = x1 - centre_x_m;
                            const double wz = z1 - centre_z_m;
                            const double s =
                                cross2(wx, wz, dx, dz) / denom;
                            const double t =
                                cross2(wx, wz, ux[i], uz[i]) / denom;
                            // ok = |denom|>1e-15 & s>0 & 0<=t<=1; the
                            // masked s.min(axis=1) is order-free.
                            if (std::fabs(denom) > 1e-15 && s > 0.0 &&
                                t >= 0.0 && t <= 1.0 && s < best)
                                best = s;
                        }
                        out.r_m[i] = best;
                    }
                }
            }
        }
        // _hits (:223-234) — elementwise deflection and ray reach; the
        // hub + R.u road point for unloaded rays.
        for (std::size_t i = 0; i < n; ++i) {
            const double r = out.r_m[i];
            const double reach = std::min(r, radius_m);
            out.delta_m[i] = std::max(radius_m - r, 0.0);
            out.road_x_m[i] = centre_x_m + reach * ux[i];
            out.road_z_m[i] = centre_z_m + reach * uz[i];
        }
        out.coverage_event = coverage;
        out.airborne = airborne;
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
