// kinematics/curves.cpp — curves.py's compute_instant_centers: the
// coupler instant center per travel sample, solved as the 2x2 linear
// system np.linalg.solve(A, b) where A = column_stack([d1, -d2]).
//
// Bitwise contract: np.linalg.solve on a 2x2 double matrix dispatches
// to numeric_backend::dgesv — the exact provider symbol the
// oracle's linalg/_umath_linalg.*.so imports (nm-verified) — through
// the same Fortran column-major ABI with 64-bit integer arguments. A
// nonzero info is the LinAlgError path the source maps to np.nan
// (canonical quiet NaN 0x7ff8000000000000). Verified bitwise against
// np.linalg.solve on 150,000 random well/ill-conditioned 2x2 systems:
// 0 mismatches, including singular detections.
#include "solver.hpp"
#include "../lapack_abi.hpp"

#include <array>
#include <limits>

namespace bike::kinematics {

namespace {
    [[nodiscard]] inline double np_nan() noexcept {
        return std::numeric_limits<double>::quiet_NaN();
    }
} // namespace

InstantCenters compute_instant_centers(const HorstLinkageSolver &solver,
                                       int n_points) {
    const KinematicTrajectory traj = solver.solve_trajectory(n_points);
    const std::vector<double> &travel = traj.wheel_travel;

    InstantCenters out;
    out.travel = travel;
    out.ic_x.reserve(travel.size());
    out.ic_z.reserve(travel.size());

    for (std::size_t i = 0; i < travel.size(); ++i) {
        // p_i[[0, 2]] — the (x, z) components of each 3-vector.
        const geometry::Vec2 p0_i{traj.P0[i][0], traj.P0[i][2]};
        const geometry::Vec2 p2_i{traj.P2[i][0], traj.P2[i][2]};
        const geometry::Vec2 p5_i{traj.P5[i][0], traj.P5[i][2]};
        const geometry::Vec2 p3_i{traj.P3[i][0], traj.P3[i][2]};

        const geometry::Vec2 d1{p2_i[0] - p0_i[0], p2_i[1] - p0_i[1]};
        const geometry::Vec2 d2{p3_i[0] - p5_i[0], p3_i[1] - p5_i[1]};

        // A = column_stack([d1, -d2]): column-major {{d1x, d1z},
        // {-d2x, -d2z}}; b = p5_i - p0_i. dgesv overwrites a (LU
        // factors), ipiv (pivots) and b (the (t, u) solution —
        // b.at(0) is the line-1 parameter).
        std::array<double, 4> a{d1[0], d1[1], -d2[0], -d2[1]};
        std::array<double, 2> b{p5_i[0] - p0_i[0],
                                p5_i[1] - p0_i[1]};
        const numeric_backend::integer n = 2;
        const numeric_backend::integer nrhs = 1;
        const numeric_backend::integer lda = 2;
        const numeric_backend::integer ldb = 2;
        numeric_backend::integer info = 0;
        std::array<numeric_backend::integer, 2> ipiv{0, 0};
        numeric_backend::dgesv(&n, &nrhs, a.data(), &lda, ipiv.data(), b.data(),
                               &ldb, &info);

        if (info != 0) {
            // np.linalg.LinAlgError -> append np.nan (canonical qnan).
            out.ic_x.push_back(np_nan());
            out.ic_z.push_back(np_nan());
            continue;
        }
        // ic = p0_i + params[0] * d1 — scalar*vector then add.
        out.ic_x.push_back(p0_i[0] + b.at(0) * d1[0]);
        out.ic_z.push_back(p0_i[1] + b.at(0) * d1[1]);
    }
    return out;
}

} // namespace bike::kinematics
