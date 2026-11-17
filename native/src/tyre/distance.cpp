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

#include "../physics_core/common.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/terrain_distance.hpp"
#include "../physics_core/tyre_types.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

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
        const bike::physics_core::SignedProfileView<double> view{
            .x = x_.data(),
            .z = z_.data(),
            .edge_x = edge_x_.data(),
            .edge_z = edge_z_.data(),
            .length2 = length2_.data(),
            .normal_x = normal_x_.data(),
            .normal_z = normal_z_.data(),
            .num_vertices = static_cast<bike::physics_core::i32>(x_.size()),
        };
        double h = 0.0;
        bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
        const bike::physics_core::Status status =
            bike::physics_core::profile_height<
                double, bike::physics_core::CpuExactToolkit>(view, x, h, err);
        if (status != bike::physics_core::Status::ok)
            throw std::invalid_argument(
                "query lies outside the compiled road domain");
        return h;
    }

    PointDistance SignedProfile::query(std::array<double, 2> p) const {
        const bike::physics_core::SignedProfileView<double> view{
            .x = x_.data(),
            .z = z_.data(),
            .edge_x = edge_x_.data(),
            .edge_z = edge_z_.data(),
            .length2 = length2_.data(),
            .normal_x = normal_x_.data(),
            .normal_z = normal_z_.data(),
            .num_vertices = static_cast<bike::physics_core::i32>(x_.size()),
        };
        bike::physics_core::PointDistanceResult<double> res{};
        bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
        const bike::physics_core::Status status =
            bike::physics_core::signed_profile_query<
                double, bike::physics_core::CpuExactToolkit>(
                view, p[0], p[1], res, err);
        if (status == bike::physics_core::Status::logic_error)
            throw std::logic_error(
                "signed-distance window produced no closest segment");
        if (status != bike::physics_core::Status::ok) {
            if (err == bike::physics_core::ErrCode::nonfinite_center)
                throw std::invalid_argument(
                    "distance queries must be a finite (N, 2) array");
            throw std::invalid_argument(
                "query lies outside the compiled road domain");
        }
        return {
            .distance = res.distance,
            .normal = {res.normal[0], res.normal[1]},
            .point = {res.point[0], res.point[1]},
            .segment = res.segment,
            .nonsmooth = res.nonsmooth != 0,
        };
    }
} // namespace biketyre
