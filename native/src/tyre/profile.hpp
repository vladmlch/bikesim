// tyre/profile.hpp — port of terrain/contact_profile.py (ProfileQuery,
// ProfileContact) plus the two scalar primitives its numeric contract
// needs: np.interp's scalar path and CPython's math.hypot.
//
// Bitwise contract beyond the ordinary scalar expressions:
//   np.interp(x, xs, ys)   — numpy's single-query path (np_interp_scalar,
//     multiarray/compiled_base.c interp): bisect, exact-hit return, then
//     slope*(xq-dx[j])+dy[j] evaluated by numpy's C code as a literal
//     fma(slope, xq - dx[j], dy[j]) — the fma is required for bitwise
//     parity (a mul+add pair lands one ULP off on ~1e3 of 24001 rows).
//   math.hypot(a, b)       — CPython Modules/mathmodule.c vector_norm:
//     scaled compensated squaring via DoubleLength fma arithmetic, NOT the
//     platform libm hypot (verified to differ on this build).
//   (c-p) @ n  1-D @ 1-D   — np.dot → Accelerate cblas_ddot; called with
//     the same entry point, not a manual multiply-add.
//   einsum('ij,ij->i')     — numpy's inner-product loop: a seeded
//     0.0 + p0 + p1 accumulation per row, which the einsum() helper keeps
//     literal so even signed-zero results match.
//   normals @ n  (k,2)@(2,)— np.matmul → cblas_dgemv(RowMajor, NoTrans).
//   np.linalg.norm(v)      — sqrt(ddot(v,v)) (same-libcall contract).
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <optional>
#include <span>
#include <vector>

namespace biketyre {
    // ProfileContact (contact_profile.py:15-29) — construction validates the
    // normal's unit length to 1e-10 (math.isclose abs_tol) and delta's
    // finiteness, like the dataclass __post_init__.
    struct ProfileContact {
        std::array<double, 2> point;
        std::array<double, 2> normal;
        double delta;
        int segment_id;
        bool multi_support;
    };

    namespace detail {
        // _candidates' row bundle (contact_profile.py:59-68).
        struct Candidates {
            std::vector<int> ids;
            std::vector<double> t;
            std::vector<std::array<double, 2> > points;
            std::vector<double> distances;
        };

        // _candidates over validated profile tables. Interval contract —
        // np.arange(lo, hi): `hi <= lo` yields the declared empty interval
        // (zero candidates), not an error and never a negative reserve.
        // A nonempty window must satisfy 0 <= lo < hi <= segment count;
        // anything else is rejected with std::invalid_argument.
        [[nodiscard]] Candidates candidates(
            std::span<const double> px, std::span<const double> pz,
            std::span<const double> seg_x, std::span<const double> seg_z,
            std::span<const double> seg_len_sq, std::array<double, 2> c,
            int lo, int hi);

        // Same interval contract and row order as candidates(), writing
        // into caller-owned storage — the destination keeps its capacity so
        // a warm-path caller never reaches the allocator.
        void candidates_into(
            std::span<const double> px, std::span<const double> pz,
            std::span<const double> seg_x, std::span<const double> seg_z,
            std::span<const double> seg_len_sq, std::array<double, 2> c,
            int lo, int hi, Candidates &out);
    } // namespace detail

    class ProfileQuery {
    public:
        // ProfileQuery.__init__ (contact_profile.py:34-57): table build plus
        // the scalar threshold validation. `px` must increase strictly.
        ProfileQuery(std::vector<double> px, std::vector<double> pz,
                     double significant_delta_m, double significance_fraction,
                     double normal_angle_deg);

        // ProfileQuery.contact (contact_profile.py:87-165).
        [[nodiscard]] ProfileContact contact(
            std::array<double, 2> center_xz, double radius,
            std::optional<int> previous_segment) const;

    private:
        // _candidates (contact_profile.py:59-68) on this profile's tables —
        // delegates to detail::candidates_into; the interval contract above.
        // contact() passes the member scratch so the warm path allocates
        // nothing (the Python original builds fresh arrays per query).
        void candidates_into(std::array<double, 2> c, int lo, int hi,
                             detail::Candidates &out) const;

        // _endpoint_keep (contact_profile.py:70-85) — written as a mask in
        // candidate order into `out`, which must already hold at least
        // cand.ids.size() entries; the write only covers that prefix.
        void endpoint_keep_into(std::array<double, 2> c,
                                const detail::Candidates &cand,
                                std::vector<bool> &out) const;

        std::vector<double> px_, pz_; // _profile_x / _profile_z
        std::vector<double> seg_x_, seg_z_; // _segments columns
        std::vector<double> seg_len_sq_; // _segment_lengths_sq
        std::vector<int> height_changes_; // _height_changes
        std::vector<double> left_x_, left_z_; // _left_directions columns
        std::vector<double> right_x_, right_z_; // _right_directions columns
        double maximum_z_; // _maximum_z
        double significant_delta_m_;
        double significance_fraction_;
        double normal_cosine_;
        // Warm-contact scratch — sized once in the ctor against the segment
        // bound (a window never exceeds the segment count), cleared and
        // refilled by every contact() call without touching the allocator.
        // `mutable` because contact() is const.
        mutable detail::Candidates cand_scratch_;
        mutable std::vector<bool> keep_scratch_;
        mutable std::vector<int> near_scratch_;
        mutable std::vector<double> normals_scratch_;
        mutable std::vector<double> dots_scratch_;
    };

    // np.interp(x, xs, ys) for a scalar query (numpy/_core/multiarray/
    // compiled_base.c: np_interp → binary_search_with_guess + the fma
    // interpolation; the constant-y degenerate case included).
    [[nodiscard]] double np_interp(double xq, std::span<const double> xs,
                                   std::span<const double> ys);

    // CPython math.hypot(a, b) — vector_norm port, exact for n=2.
    [[nodiscard]] double py_hypot(double a, double b);

    // compiled_profile_vertices (sim/ride/tire_forces.py:18-44): the exact
    // longitudinal cross-section of the named geom's compiled heightfield,
    // as world X-Z vertices. Same validation and same NumPy lowerings:
    //   np.linspace(-sx, sx, cols) — delta/step then i*step+start, last = stop
    //   raster[0]*sz               — float32 row cast to float64 then *sz
    //   local @ R.T + xpos         — dgemm(RowMajor, NoTrans, Trans) + add
    //   np.allclose checks         — elementwise |a-b| <= atol (+rtol*|b|)
    [[nodiscard]] std::vector<std::array<double, 2> >
    compiled_profile_vertices(const mjModel *model, const mjData *data,
                              const char *terrain_name);
} // namespace biketyre
