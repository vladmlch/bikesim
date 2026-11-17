// native/src/physics_core/terrain_distance.hpp — Tri-target allocation-free
// signed road profile height and closest-point distance queries.
#pragma once

#include "common.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel header: MSL has no std::span/std::array and device buffers are address-space-qualified raw pointers. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

namespace terrain_distance_detail {

BIKE_HD int clamp_i32(int v, int lo, int hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

BIKE_HD int max_i32(int a, int b) noexcept {
    return (a < b) ? b : a;
}

BIKE_HD int min_i32(int a, int b) noexcept {
    return (b < a) ? b : a;
}

template <class Real>
BIKE_HD int search_right(BIKE_DEVICE const Real *x, int n, Real v) noexcept {
    int first = 0;
    int count = n;
    while (count > 0) {
        const int step = count / 2;
        const int mid = first + step;
        if (!(v < x[mid])) {
            first = mid + 1;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    return first;
}

template <class Real>
BIKE_HD int search_left(BIKE_DEVICE const Real *x, int n, Real v) noexcept {
    int first = 0;
    int count = n;
    while (count > 0) {
        const int step = count / 2;
        const int mid = first + step;
        if (x[mid] < v) {
            first = mid + 1;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    return first;
}

template <class Real>
struct DistanceCandidate {
    Real fraction = static_cast<Real>(0);
    Real projection[2]{};
    Real vector[2]{};
    Real squared = static_cast<Real>(0);
};

template <class Real, class TK>
BIKE_HD DistanceCandidate<Real> eval_distance_candidate(
    const SignedProfileView<Real> &sp,
    int i,
    Real px,
    Real pz
) noexcept {
    const Real ox = px - sp.x[i];
    const Real oz = pz - sp.z[i];
    Real num = static_cast<Real>(0);
    num += ox * sp.edge_x[i];
    num += oz * sp.edge_z[i];
    const Real fraction = TK::clamp(
        TK::div(num, sp.length2[i]), static_cast<Real>(0), static_cast<Real>(1));
    const Real proj0 = sp.x[i] + fraction * sp.edge_x[i];
    const Real proj1 = sp.z[i] + fraction * sp.edge_z[i];
    const Real vec0 = px - proj0;
    const Real vec1 = pz - proj1;
    Real squared = static_cast<Real>(0);
    squared += vec0 * vec0;
    squared += vec1 * vec1;
    DistanceCandidate<Real> c{};
    c.fraction = fraction;
    c.projection[0] = proj0;
    c.projection[1] = proj1;
    c.vector[0] = vec0;
    c.vector[1] = vec1;
    c.squared = squared;
    return c;
}

} // namespace terrain_distance_detail

template <class Real, class TK>
BIKE_HD Status profile_height(
    const SignedProfileView<Real> &sp,
    Real x,
    Real &out_h,
    ErrCode &err
) noexcept {
    if (sp.num_vertices < 2 || sp.x == nullptr || sp.z == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const int n = sp.num_vertices;
    if (!TK::isfinite(x) || x < sp.x[0] || x > sp.x[n - 1]) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    const Real lval = sp.z[0];
    const Real rval = sp.z[n - 1];
    int j = 0;
    if (x < sp.x[0]) {
        j = -1;
    } else if (x > sp.x[n - 1]) {
        j = n;
    } else {
        j = terrain_distance_detail::search_right<Real>(sp.x, n, x) - 1;
    }
    if (j == -1) {
        out_h = lval;
        err = ErrCode::none;
        return Status::ok;
    }
    if (j == n || j == n - 1) {
        out_h = rval;
        err = ErrCode::none;
        return Status::ok;
    }
    if (sp.x[j] == x) {
        out_h = sp.z[j];
        err = ErrCode::none;
        return Status::ok;
    }
    const Real slope = TK::div(sp.z[j + 1] - sp.z[j], sp.x[j + 1] - sp.x[j]);
    out_h = TK::interp_segment(
        slope, x, sp.x[j], sp.z[j], sp.x[j + 1], sp.z[j + 1]);
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status signed_profile_query(
    const SignedProfileView<Real> &sp,
    Real px,
    Real pz,
    PointDistanceResult<Real> &out,
    ErrCode &err
) noexcept {
    out = PointDistanceResult<Real>{};
    if (!TK::isfinite(px) || !TK::isfinite(pz)) {
        err = ErrCode::nonfinite_center;
        return Status::invalid_argument;
    }
    if (sp.num_vertices < 2 || sp.x == nullptr || sp.z == nullptr ||
        sp.edge_x == nullptr || sp.edge_z == nullptr ||
        sp.length2 == nullptr || sp.normal_x == nullptr ||
        sp.normal_z == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    Real h = static_cast<Real>(0);
    const Status hs = profile_height<Real, TK>(sp, px, h, err);
    if (hs != Status::ok) {
        return hs;
    }

    const int nseg = sp.num_vertices - 1;
    const int first = terrain_distance_detail::clamp_i32(
        terrain_distance_detail::search_right<Real>(sp.x, sp.num_vertices, px) - 1,
        0,
        nseg - 1);
    const Real edge[2] = {sp.edge_x[first], sp.edge_z[first]};
    const Real offset[2] = {px - sp.x[first], pz - sp.z[first]};
    const Real t = TK::clamp(
        TK::div(TK::dot2(offset, edge), sp.length2[first]),
        static_cast<Real>(0),
        static_cast<Real>(1));
    const Real projected[2] = {
        sp.x[first] + t * edge[0],
        sp.z[first] + t * edge[1]
    };
    const Real upper_d[2] = {
        px - projected[0],
        pz - projected[1]
    };
    const Real upper = TK::norm2_ddot(upper_d);

    const int left = terrain_distance_detail::max_i32(
        0,
        terrain_distance_detail::search_left<Real>(sp.x, sp.num_vertices, px - upper) - 1);
    const int right = terrain_distance_detail::min_i32(
        nseg,
        terrain_distance_detail::search_right<Real>(sp.x, sp.num_vertices, px + upper));
    const int hi = terrain_distance_detail::max_i32(left + 1, right);

    Real best = terrain_distance_detail::eval_distance_candidate<Real, TK>(
        sp, left, px, pz).squared;
    for (int i = left + 1; i < hi; ++i) {
        best = TK::min(
            best,
            terrain_distance_detail::eval_distance_candidate<Real, TK>(
                sp, i, px, pz).squared);
    }

    const Real max_one_best = TK::max(static_cast<Real>(1), best);
    const Real bound = best + BIKE_REAL(Real, 2e-15) * max_one_best;

    int sid = -1;
    terrain_distance_detail::DistanceCandidate<Real> winner{};
    bool nonsmooth = false;
    for (int i = left; i < hi; ++i) {
        const terrain_distance_detail::DistanceCandidate<Real> row =
            terrain_distance_detail::eval_distance_candidate<Real, TK>(sp, i, px, pz);
        if (row.squared <= bound) {
            if (sid < 0) {
                sid = i;
                winner = row;
            }
            const Real dnx = sp.normal_x[i] - sp.normal_x[sid];
            const Real dnz = sp.normal_z[i] - sp.normal_z[sid];
            if (TK::sqrt(dnx * dnx + dnz * dnz) > BIKE_REAL(Real, 1e-10)) {
                nonsmooth = true;
            }
        }
    }
    if (sid < 0) {
        err = ErrCode::logic_no_closest;
        return Status::logic_error;
    }

    const Real length = TK::sqrt(winner.squared);
    const Real sign = pz >= h ? static_cast<Real>(1) : static_cast<Real>(-1);
    out.distance = sign * length;
    if (length > BIKE_REAL(Real, 1e-12)) {
        out.normal[0] = TK::div(sign * winner.vector[0], length);
        out.normal[1] = TK::div(sign * winner.vector[1], length);
    } else {
        out.normal[0] = sp.normal_x[sid];
        out.normal[1] = sp.normal_z[sid];
    }
    out.point[0] = winner.projection[0];
    out.point[1] = winner.projection[1];
    out.segment = sid;
    out.nonsmooth = nonsmooth ? static_cast<u8>(1) : static_cast<u8>(0);

    if (length <= BIKE_REAL(Real, 1e-12) &&
        (winner.fraction <= BIKE_REAL(Real, 1e-12) ||
         winner.fraction >= static_cast<Real>(1) - BIKE_REAL(Real, 1e-12))) {
        const int vertex_id =
            winner.fraction <= BIKE_REAL(Real, 1e-12) ? sid : sid + 1;
        if (0 < vertex_id && vertex_id < sp.num_vertices - 1) {
            const Real d[2] = {
                sp.normal_x[vertex_id - 1] - sp.normal_x[vertex_id],
                sp.normal_z[vertex_id - 1] - sp.normal_z[vertex_id]
            };
            if (TK::norm2_ddot(d) > BIKE_REAL(Real, 1e-10)) {
                out.nonsmooth = static_cast<u8>(1);
            }
        }
    }

    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
