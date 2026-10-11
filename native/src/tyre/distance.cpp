// tyre/distance.cpp — expression-for-expression port of
// terrain/profile_distance.py (SignedProfile). Comments cite the source
// lines at the time of porting.
//
// NumPy lowering decisions (each one verified bitwise by the oracle gate
// in tests/reference/test_native_distributed_tire.py):
//   np.interp scalar                  → biketyre::np_interp (fma form)
//   np.searchsorted right/left        → upper_bound/lower_bound
//   np.dot 1-D @ 1-D                  → blas::ddot
//   np.linalg.norm(v) on (2,)         → sqrt(ddot(v, v))   [axis=None path]
//   np.linalg.norm(M, axis=1) (k,2)   → sqrt(x*x + y*y) per row
//                                       [sqrt(add.reduce) path]
//   np.sum(rows, axis=1) on (k,2)     → +0.0-seeded fold `0.0 + a + b`:
//                                       add.reduce's +0.0 identity wraps
//                                       pairwise_sum's -0.0 seed, which is
//                                       equivalent even on the all--0.0
//                                       product edge case
//   np.clip(v, 0., 1.)                → clip01 (clip.cpp const-minmax path:
//                                       `x < lo ? lo : (x > hi ? hi : x)`,
//                                       which keeps an input -0.0)
//   np.min / flatnonzero / argmin-id  → ordered scans (first winner)
#include "distance.hpp"

#include "profile.hpp"
#include "../cblas_abi.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <stdexcept>
#include <utility>

namespace {
    // np.searchsorted(x, v, side='right') — first index with x[i] > v.
    int search_right(std::span<const double> x, double v) {
        return static_cast<int>(
            std::ranges::upper_bound(x, v) - x.begin());
    }

    // np.searchsorted(x, v, side='left') — first index with x[i] >= v.
    int search_left(std::span<const double> x, double v) {
        return static_cast<int>(
            std::ranges::lower_bound(x, v) - x.begin());
    }

    // np.clip(t, 0., 1.) constant-bound float path: the sequential
    // comparisons keep -0.0 and propagate NaN like the clip ufunc.
    double clip01(double t) { return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t); }
} // namespace

namespace biketyre {
    SignedProfile::SignedProfile(
        std::vector<std::array<double, 2>> vertices_xz) {
        // profile_distance.py:23-26 — the oracle's single check combines
        // shape, size, finiteness, and strict-X monotonicity; the shape
        // side is the (N,2) column split the caller performs here.
        if (vertices_xz.size() < 2)
            throw std::invalid_argument(
                "road vertices must be finite and strictly increasing in X");
        const std::size_t n = vertices_xz.size();
        x_.resize(n);
        z_.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            x_[i] = vertices_xz[i][0];
            z_[i] = vertices_xz[i][1];
            if (!std::isfinite(x_[i]) || !std::isfinite(z_[i]))
                throw std::invalid_argument(
                    "road vertices must be finite and strictly increasing "
                    "in X");
        }
        // np.any(np.diff(vertices[:, 0]) <= 0) — elementwise over the
        // strictly-monotone condition; NaN diffs are unreachable (every
        // stored coordinate is finite, so a diff is finite or +inf).
        for (std::size_t i = 0; i + 1 < n; ++i)
            if (x_[i + 1] - x_[i] <= 0.0)
                throw std::invalid_argument(
                    "road vertices must be finite and strictly increasing "
                    "in X");
        const std::size_t nseg = n - 1;
        edge_x_.resize(nseg);
        edge_z_.resize(nseg);
        length2_.resize(nseg);
        normal_x_.resize(nseg);
        normal_z_.resize(nseg);
        for (std::size_t i = 0; i < nseg; ++i) {
            const double ex = x_[i + 1] - x_[i]; // np.diff row (axis=0)
            const double ez = z_[i + 1] - z_[i];
            edge_x_[i] = ex;
            edge_z_[i] = ez;
            // length2 = np.sum(edges**2, axis=1) — seeded fold over the
            // two-element row.
            double len2 = 0.0;
            len2 += ex * ex;
            len2 += ez * ez;
            length2_[i] = len2;
            // normals = column_stack((-ez, ex)) / sqrt(length2)[:, None] —
            // numpy broadcasts the division: two independent divides by
            // the same scalar, never a reciprocal-multiply.
            const double len = std::sqrt(len2);
            normal_x_[i] = -ez / len;
            normal_z_[i] = ex / len;
        }
    }

    double SignedProfile::height(double x) const {
        // profile_distance.py:34-38 — finite scalar inside the compiled
        // X domain, then the np.interp port.
        if (!std::isfinite(x) || x < x_.front() || x > x_.back())
            throw std::invalid_argument(
                "query lies outside the compiled road domain");
        return np_interp(x, x_, z_);
    }

    SignedProfile::Candidate SignedProfile::candidate(
        int i, std::array<double, 2> p) const {
        // profile_distance.py:55-59 evaluated as scalar rows for segment
        // i: offsets = p - vertices[i], fractions = clip(sum(offsets *
        // edges, axis=1)/length2), projections = vertices + fractions
        // [:, None]*edges, vectors = p - projections, squared =
        // np.sum(vectors*vectors, axis=1).
        const std::size_t u = static_cast<std::size_t>(i);
        const double ox = p[0] - x_[u];
        const double oz = p[1] - z_[u];
        // Two-element row of offsets*edges[ids]: +0.0-seeded fold (see the
        // header for why the seed sign is +0.0, not pairwise's -0.0).
        double num = 0.0;
        num += ox * edge_x_[u];
        num += oz * edge_z_[u];
        const double fraction = clip01(num / length2_[u]);
        // vertices + fractions[:, None]*edges — one multiply then one add
        // per component (two roundings, kept un-fused).
        const std::array<double, 2> projection = {
            x_[u] + fraction * edge_x_[u], z_[u] + fraction * edge_z_[u]};
        const std::array<double, 2> vector = {p[0] - projection[0],
                                              p[1] - projection[1]};
        double squared = 0.0;
        squared += vector[0] * vector[0];
        squared += vector[1] * vector[1];
        return {.fraction = fraction,
                .projection = projection,
                .vector = vector,
                .squared = squared};
    }

    PointDistance SignedProfile::query(std::array<double, 2> p) const {
        // profile_distance.py:42-43 — the oracle checks finiteness of the
        // whole (N,2) batch before the domain gate; for the single-point
        // API both reduce to this point.
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]))
            throw std::invalid_argument(
                "distance queries must be a finite (N, 2) array");
        // line 44 — height() carries the domain check and np.interp.
        const double h = height(p[0]);
        const int nseg = static_cast<int>(x_.size()) - 1;
        // line 47 — local = clip(searchsorted(x, px, 'right') - 1,
        // 0, nseg-1); the integer clip is a plain clamp.
        const int first =
            std::clamp(search_right(x_, p[0]) - 1, 0, nseg - 1);
        const std::size_t f = static_cast<std::size_t>(first);
        const std::array<double, 2> edge = {edge_x_[f], edge_z_[f]};
        const std::array<double, 2> offset = {p[0] - x_[f], p[1] - z_[f]};
        // line 50 — np.dot on two (2,) vectors → ddot; then the scalar
        // divide and the scalar np.clip.
        const double t = clip01(
            blas::ddot(2, offset.data(), 1, edge.data(), 1) /
            length2_[f]);
        // line 51 — vertices[first] + t*edge is multiply-then-add per
        // component; np.linalg.norm on the (2,) difference → sqrt(ddot).
        const std::array<double, 2> projected = {x_[f] + t * edge[0],
                                                 z_[f] + t * edge[1]};
        const std::array<double, 2> upper_d = {p[0] - projected[0],
                                               p[1] - projected[1]};
        const double upper = std::sqrt(
            blas::ddot(2, upper_d.data(), 1, upper_d.data(), 1));
        // lines 52-54 — the horizontal window follows from the first
        // candidate's Euclidean length; ids = arange(left, hi) is the
        // nonempty ascending segment range.
        const int left = std::max(0, search_left(x_, p[0] - upper) - 1);
        const int right = std::min(nseg, search_right(x_, p[0] + upper));
        const int hi = std::max(left + 1, right);
        // line 61 — best = min(squared): a value scan over the window.
        double best = candidate(left, p).squared;
        for (int i = left + 1; i < hi; ++i)
            best = std::min(best, candidate(i, p).squared);
        // line 62 — numerical ties only, not a geometric smoothing
        // length; the bound is scalar best + 2e-15*max(1., best).
        const double bound = best + 2e-15 * std::max(1.0, best);
        // lines 63-70 — the window ascends, so the first squared <= bound
        // is the winner (ties[0]); every tie's segment normal is compared
        // against the winner's for the nonsmooth flag. The candidate row
        // is a pure function of the segment id, so recomputing it here
        // reproduces the oracle's temporaries bit-for-bit.
        int sid = -1;
        Candidate winner{};
        bool nonsmooth = false;
        for (int i = left; i < hi; ++i) {
            const Candidate row = candidate(i, p);
            if (row.squared <= bound) {
                if (sid < 0) {
                    sid = i;
                    winner = row;
                }
                const std::size_t u = static_cast<std::size_t>(i);
                const std::size_t w = static_cast<std::size_t>(sid);
                const double dnx = normal_x_[u] - normal_x_[w];
                const double dnz = normal_z_[u] - normal_z_[w];
                // norm(tie_normals - normals[sid], axis=1) > 1e-10 —
                // the (k,2) row-norm add.reduce fold.
                if (std::sqrt(dnx * dnx + dnz * dnz) > 1e-10)
                    nonsmooth = true;
            }
        }
        if (sid < 0)
            // Unreachable: the best element is always inside its own tie
            // bound; the throw pins the invariant instead of relying on
            // an out-of-contract empty-window caller.
            throw std::logic_error(
                "signed-distance window produced no closest segment");
        // lines 64-66 — length, sign (p.z >= height), distance, point,
        // segment.
        const double length = std::sqrt(winner.squared);
        const double sign = p[1] >= h ? 1.0 : -1.0;
        const std::size_t w = static_cast<std::size_t>(sid);
        PointDistance out{
            .distance = sign * length,
            // line 67 — sign*vector/length is elementwise-mul then
            // elementwise-div (two roundings per component); below the
            // 1e-12 length floor the raw segment normal is used.
            .normal = length > 1e-12
                          ? std::array<double, 2>{
                                (sign * winner.vector[0]) / length,
                                (sign * winner.vector[1]) / length}
                          : std::array<double, 2>{normal_x_[w],
                                                  normal_z_[w]},
            .point = winner.projection,
            .segment = sid,
            .nonsmooth = nonsmooth,
        };
        // lines 71-74 — a zero-length hit on a shared endpoint is
        // nonsmooth only when the adjacent segment normals really differ.
        if (length <= 1e-12 &&
            (winner.fraction <= 1e-12 ||
             winner.fraction >= 1.0 - 1e-12)) {
            const int vertex = winner.fraction <= 1e-12 ? sid : sid + 1;
            if (0 < vertex && vertex < static_cast<int>(x_.size()) - 1) {
                const std::size_t v = static_cast<std::size_t>(vertex);
                const std::array<double, 2> d = {
                    normal_x_[v - 1] - normal_x_[v],
                    normal_z_[v - 1] - normal_z_[v]};
                // 1-D np.linalg.norm here (axis=None → x.dot(x)) →
                // sqrt(ddot(d, d)), not the axis=1 fold used above.
                if (std::sqrt(blas::ddot(2, d.data(), 1, d.data(), 1)) >
                    1e-10)
                    out.nonsmooth = true;
            }
        }
        return out;
    }
} // namespace biketyre
