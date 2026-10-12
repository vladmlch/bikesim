// tyre/distance.hpp — port of terrain/profile_distance.py: the exact
// signed distance to the compiled, monotone-X, piecewise-linear road.
//
// The oracle builds (N,2) vertex/edge/normals tables; this port keeps the
// same values as column vectors — the split loses no information because
// every oracle expression reads one column or one (2,) row at a time.
//
// Bitwise contract beyond the ordinary scalar expressions:
//   np.interp(x, xs, ys) scalar   — biketyre::np_interp (tyre/profile.hpp:
//                                   numpy's single-query fma path).
//   np.searchsorted right/left    — std::ranges::upper_bound/lower_bound.
//   np.dot(a, b) 1-D @ 1-D        — blas::ddot (same numpy libcall).
//   np.linalg.norm(v) 1-D         — sqrt(blas::ddot(v, v)): the oracle's
//                                   axis=None path lowers to x.dot(x).
//   np.linalg.norm(M, axis=1)     — elementwise sqrt(x*x + y*y) per (k,2)
//                                   row: the axis path lowers to
//                                   sqrt(add.reduce(x*x)), whose +0.0
//                                   identity over pairwise_sum's -0.0
//                                   seed is `0.0 + a + b` for n = 2 —
//                                   equal to `a + b` here because the
//                                   products are squares (never -0.0).
//   np.sum(row, axis=1) on (k,2)  — the same +0.0-seeded two-term fold,
//                                   which also matches add.reduce on the
//                                   all--0.0 edge case (the identity fold
//                                   turns pairwise's -0.0 into +0.0).
//   np.clip(t, 0., 1.)            — clip01: clip.cpp's constant-minmax
//                                   floating path is `x < min ? min :
//                                   (x > max ? max : x)`, preserving -0.0.
//   np.min / np.flatnonzero       — ordered scans; min needs only the
//                                   value, flatnonzero takes the first.
#pragma once

#include <array>
#include <vector>

namespace biketyre {
    // One row of ProfileDistance (profile_distance.py:12-18): the signed
    // distance, unit normal, closest road point, winning segment id, and
    // the nonsmooth flag for a single query point.
    struct PointDistance {
        double distance;
        std::array<double, 2> normal;
        std::array<double, 2> point;
        int segment;
        bool nonsmooth;
    };

    class SignedProfile {
    public:
        // SignedProfile.__init__ (profile_distance.py:22-32): rejects
        // fewer than two vertices, non-finite entries, and X that does not
        // increase strictly — all under the oracle's single error message.
        explicit SignedProfile(
            std::vector<std::array<double, 2>> vertices_xz);

        // SignedProfile.height (profile_distance.py:34-38): the domain
        // gate, then np.interp — the scalar np_interp port.
        [[nodiscard]] double height(double x) const;

        // SignedProfile.query (profile_distance.py:40-75) for one point:
        // same per-point window logic, sign convention, and nonsmooth
        // flagging as the oracle's vectorized loop.
        [[nodiscard]] PointDistance query(std::array<double, 2> p) const;

    private:
        // One row of the oracle's offsets/fractions/projections/vectors/
        // squared temporaries (profile_distance.py:55-59) for candidate
        // segment i — a pure function of the segment id, so query()
        // recomputes rows on each scan instead of materializing the
        // oracle's (k,) arrays.
        struct Candidate {
            double fraction;
            std::array<double, 2> projection;
            std::array<double, 2> vector;
            double squared;
        };

        [[nodiscard]] Candidate candidate(int i,
                                          std::array<double, 2> p) const;

        std::vector<double> x_, z_; // vertices[:, 0] / vertices[:, 1]
        std::vector<double> edge_x_, edge_z_; // np.diff columns
        std::vector<double> length2_; // np.sum(edges**2, axis=1)
        std::vector<double> normal_x_, normal_z_; // (-ez, ex)/sqrt(length2)
    };
} // namespace biketyre
