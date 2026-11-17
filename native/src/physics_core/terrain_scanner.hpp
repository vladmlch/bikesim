// native/src/physics_core/terrain_scanner.hpp — Tri-target O(1)-register
// stateless two-pass polyline contact scanner mirroring ProfileQuery::contact.
#pragma once

#include "common.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel header: MSL has no std::span/std::array and device buffers are address-space-qualified raw pointers. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

namespace terrain_scanner_detail {

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

template <class Real, class TK>
BIKE_HD Real np_interp_view(
    Real xq,
    BIKE_DEVICE const Real *xs,
    BIKE_DEVICE const Real *ys,
    int n
) noexcept {
    const Real lval = ys[0];
    const Real rval = ys[n - 1];
    int j = 0;
    if (xq < xs[0]) {
        j = -1;
    } else if (xq > xs[n - 1]) {
        j = n;
    } else {
        j = search_right<Real>(xs, n, xq) - 1;
    }
    if (j == -1) {
        return lval;
    }
    if (j == n || j == n - 1) {
        return rval;
    }
    if (xs[j] == xq) {
        return ys[j];
    }
    const Real slope = TK::div(ys[j + 1] - ys[j], xs[j + 1] - xs[j]);
    return TK::interp_segment(slope, xq, xs[j], ys[j], xs[j + 1], ys[j + 1]);
}

template <class Real>
struct CandidateEval {
    Real t = static_cast<Real>(0);
    Real ptx = static_cast<Real>(0);
    Real ptz = static_cast<Real>(0);
    Real dist = static_cast<Real>(0);
};

template <class Real, class TK>
BIKE_HD CandidateEval<Real> eval_segment(
    const PolylineView<Real> &pl,
    int seg,
    Real cx,
    Real cz
) noexcept {
    const Real ox = cx - pl.px[seg];
    const Real oz = cz - pl.pz[seg];
    Real num = static_cast<Real>(0);
    num += ox * pl.seg_x[seg];
    num += oz * pl.seg_z[seg];
    const Real t = TK::clamp(
        TK::div(num, pl.seg_len_sq[seg]), static_cast<Real>(0), static_cast<Real>(1));
    const Real ptx = pl.px[seg] + t * pl.seg_x[seg];
    const Real ptz = pl.pz[seg] + t * pl.seg_z[seg];
    const Real dx = cx - ptx;
    const Real dz = cz - ptz;
    Real dsq = static_cast<Real>(0);
    dsq += dx * dx;
    dsq += dz * dz;
    CandidateEval<Real> out{};
    out.t = t;
    out.ptx = ptx;
    out.ptz = ptz;
    out.dist = TK::sqrt(dsq);
    return out;
}

template <class Real>
BIKE_HD bool eval_endpoint_keep(
    const PolylineView<Real> &pl,
    int seg,
    Real t,
    Real cx,
    Real cz
) noexcept {
    if (t != static_cast<Real>(0) && t != static_cast<Real>(1)) {
        return true;
    }
    const int v = seg + (t == static_cast<Real>(1) ? 1 : 0);
    const Real ox = cx - pl.px[v];
    const Real oz = cz - pl.pz[v];
    bool keep = true;
    Real dl = static_cast<Real>(0);
    dl += ox * pl.left_x[v];
    dl += oz * pl.left_z[v];
    if (dl > BIKE_REAL(Real, 1e-14)) {
        keep = false;
    }
    Real dr = static_cast<Real>(0);
    dr += ox * pl.right_x[v];
    dr += oz * pl.right_z[v];
    if (dr > BIKE_REAL(Real, 1e-14)) {
        keep = false;
    }
    return keep;
}

template <class Real>
struct WindowScanSummary {
    bool nonempty = false;
    Real min_dist = static_cast<Real>(0);
    int winner_seg = -1;
    Real winner_dist = static_cast<Real>(0);
    Real winner_ptx = static_cast<Real>(0);
    Real winner_ptz = static_cast<Real>(0);
    bool has_prev = false;
    Real prev_dist = static_cast<Real>(0);
    Real prev_ptx = static_cast<Real>(0);
    Real prev_ptz = static_cast<Real>(0);
    bool prev_keep = false;
};

template <class Real, class TK>
BIKE_HD WindowScanSummary<Real> scan_window_o1(
    const PolylineView<Real> &pl,
    int lo,
    int hi,
    Real cx,
    Real cz,
    bool has_prev_seg,
    int prev_seg
) noexcept {
    WindowScanSummary<Real> s{};
    if (hi <= lo) {
        return s;
    }
    s.nonempty = true;
    for (int seg = lo; seg < hi; ++seg) {
        const CandidateEval<Real> c = eval_segment<Real, TK>(pl, seg, cx, cz);
        if (seg == lo) {
            s.min_dist = c.dist;
            s.winner_seg = seg;
            s.winner_dist = c.dist;
            s.winner_ptx = c.ptx;
            s.winner_ptz = c.ptz;
        } else {
            if (c.dist < s.min_dist) {
                s.min_dist = c.dist;
            }
            if (c.dist < s.winner_dist) {
                s.winner_seg = seg;
                s.winner_dist = c.dist;
                s.winner_ptx = c.ptx;
                s.winner_ptz = c.ptz;
            }
        }
        if (has_prev_seg && seg == prev_seg) {
            s.has_prev = true;
            s.prev_dist = c.dist;
            s.prev_ptx = c.ptx;
            s.prev_ptz = c.ptz;
            s.prev_keep = eval_endpoint_keep<Real>(pl, seg, c.t, cx, cz);
        }
    }
    return s;
}

} // namespace terrain_scanner_detail

struct NoMultiSupport {
    template <class Real, class TK>
    BIKE_HD static bool evaluate(
        const PolylineView<Real> & /*pl*/,
        int /*win_lo*/,
        int /*win_hi*/,
        Real /*cx*/,
        Real /*cz*/,
        Real /*radius*/,
        Real /*winner_ptx*/,
        Real /*winner_ptz*/,
        Real /*delta_winner*/,
        const Real /*normal_winner*/[2]
    ) noexcept {
        return false;
    }
};

struct O1CpuMultiSupport {
    template <class Real, class TK>
    BIKE_HD static bool evaluate(
        const PolylineView<Real> &pl,
        int win_lo,
        int win_hi,
        Real cx,
        Real cz,
        Real radius,
        Real winner_ptx,
        Real winner_ptz,
        Real delta_winner,
        const Real normal_winner[2]
    ) noexcept {
        const Real threshold = TK::max(
            pl.significant_delta_m,
            pl.significance_fraction * TK::max(delta_winner, static_cast<Real>(0)));
        for (int seg = win_lo; seg < win_hi; ++seg) {
            const terrain_scanner_detail::CandidateEval<Real> c =
                terrain_scanner_detail::eval_segment<Real, TK>(pl, seg, cx, cz);
            if (radius - c.dist >= threshold &&
                terrain_scanner_detail::eval_endpoint_keep<Real>(pl, seg, c.t, cx, cz)) {
                const Real n[2] = {
                    TK::div(cx - c.ptx, c.dist),
                    TK::div(cz - c.ptz, c.dist)
                };
                const Real dot_val = TK::dot2(n, normal_winner);
                const Real ex = c.ptx - winner_ptx;
                const Real ez = c.ptz - winner_ptz;
                const bool separated =
                    TK::sqrt(ex * ex + ez * ez) > BIKE_REAL(Real, 1e-10);
                if (dot_val < pl.normal_cosine && separated) {
                    return true;
                }
            }
        }
        return false;
    }
};

template <class Real, class TK, class MultiSupportPolicy = NoMultiSupport>
BIKE_HD Status scan_polyline_contact(
    const PolylineView<Real> &pl,
    Real cx,
    Real cz,
    Real radius,
    bool has_prev_seg,
    int prev_seg,
    PolylineContactResult<Real> &out,
    ErrCode &err
) noexcept {
    out = PolylineContactResult<Real>{};
    if (!TK::isfinite(cx) || !TK::isfinite(cz)) {
        err = ErrCode::nonfinite_center;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(radius) || radius <= static_cast<Real>(0)) {
        err = ErrCode::invalid_radius;
        return Status::invalid_argument;
    }
    if (pl.num_vertices < 2 || pl.px == nullptr || pl.pz == nullptr ||
        pl.seg_x == nullptr || pl.seg_z == nullptr ||
        pl.seg_len_sq == nullptr || pl.height_changes == nullptr ||
        pl.left_x == nullptr || pl.left_z == nullptr ||
        pl.right_x == nullptr || pl.right_z == nullptr) {
        err = ErrCode::degenerate_contact;
        return Status::invalid_argument;
    }
    const int count = pl.num_vertices - 1;
    if (has_prev_seg && (prev_seg < 0 || prev_seg >= count)) {
        err = ErrCode::invalid_prev_seg;
        return Status::invalid_argument;
    }
    if (!(pl.px[0] <= cx && cx <= pl.px[count])) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (cz <= terrain_scanner_detail::np_interp_view<Real, TK>(
                  cx, pl.px, pl.pz, pl.num_vertices)) {
        err = ErrCode::solid_road;
        return Status::invalid_argument;
    }

    const int lo = terrain_scanner_detail::max_i32(
        0,
        terrain_scanner_detail::search_right<Real>(pl.px, pl.num_vertices, cx - radius) - 2);
    const int hi = terrain_scanner_detail::min_i32(
        count,
        terrain_scanner_detail::search_right<Real>(pl.px, pl.num_vertices, cx + radius) + 1);
    const Real height = cz - pl.pz[lo];

    if (height < radius && pl.height_changes[hi] == pl.height_changes[lo]) {
        int segment = terrain_scanner_detail::max_i32(
            0,
            terrain_scanner_detail::search_left<Real>(pl.px, pl.num_vertices, cx) - 1);
        Real ptx = cx;
        const Real ptz = pl.pz[lo];
        Real distance = height;
        if (has_prev_seg && lo <= prev_seg && prev_seg < hi) {
            const Real previous_x =
                TK::min(TK::max(cx, pl.px[prev_seg]), pl.px[prev_seg + 1]);
            const Real previous_distance = TK::hypot2(cx - previous_x, height);
            const int vertex =
                previous_x == pl.px[prev_seg] ? prev_seg : prev_seg + 1;
            const bool endpoint =
                previous_x == pl.px[prev_seg] || previous_x == pl.px[prev_seg + 1];
            bool closer = false;
            if (endpoint) {
                const Real off[2] = {cx - pl.px[vertex], cz - pl.pz[vertex]};
                const Real left[2] = {pl.left_x[vertex], pl.left_z[vertex]};
                const Real right[2] = {pl.right_x[vertex], pl.right_z[vertex]};
                closer = TK::dot2(off, left) > BIKE_REAL(Real, 1e-14) ||
                         TK::dot2(off, right) > BIKE_REAL(Real, 1e-14);
            }
            if (!closer &&
                TK::abs(previous_distance - distance) <= BIKE_REAL(Real, 1e-12)) {
                segment = prev_seg;
                ptx = previous_x;
                distance = previous_distance;
            }
        }
        if (distance <= BIKE_REAL(Real, 1e-12)) {
            err = ErrCode::degenerate_contact;
            return Status::invalid_argument;
        }
        const Real n[2] = {TK::div(cx - ptx, distance), TK::div(cz - ptz, distance)};
        if (!TK::isfinite(ptx) || !TK::isfinite(ptz) ||
            !TK::isfinite(n[0]) || !TK::isfinite(n[1])) {
            err = ErrCode::degenerate_contact;
            return Status::invalid_argument;
        }
        const Real unit_tol = sizeof(Real) == 4
            ? BIKE_REAL(Real, 1e-5)
            : BIKE_REAL(Real, 1e-10);
        if (!(TK::abs(TK::hypot2(n[0], n[1]) - static_cast<Real>(1)) <= unit_tol)) {
            err = ErrCode::nonunit_normal;
            return Status::invalid_argument;
        }
        const Real delta = radius - distance;
        if (!TK::isfinite(delta)) {
            err = ErrCode::overflow_nonfinite;
            return Status::invalid_argument;
        }
        out.point[0] = ptx;
        out.point[1] = ptz;
        out.normal[0] = n[0];
        out.normal[1] = n[1];
        out.delta = delta;
        out.segment_id = segment;
        out.multi_support = 0;
        err = ErrCode::none;
        return Status::ok;
    }

    terrain_scanner_detail::WindowScanSummary<Real> sum =
        terrain_scanner_detail::scan_window_o1<Real, TK>(
            pl, lo, hi, cx, cz, has_prev_seg, prev_seg);
    int win_lo = lo;
    int win_hi = hi;

    if (!sum.nonempty || sum.min_dist >= radius) {
        if (sum.nonempty) {
            const Real best = sum.min_dist;
            const Real vertical_lower = TK::max(static_cast<Real>(0), cz - pl.maximum_z);
            const Real reach =
                TK::sqrt(TK::max(
                    static_cast<Real>(0),
                    best * best - vertical_lower * vertical_lower)) +
                BIKE_REAL(Real, 1e-8);
            win_lo = terrain_scanner_detail::max_i32(
                0,
                terrain_scanner_detail::search_right<Real>(
                    pl.px, pl.num_vertices, cx - reach) - 2);
            win_hi = terrain_scanner_detail::min_i32(
                count,
                terrain_scanner_detail::search_right<Real>(
                    pl.px, pl.num_vertices, cx + reach) + 1);
        } else {
            win_lo = 0;
            win_hi = count;
        }
        sum = terrain_scanner_detail::scan_window_o1<Real, TK>(
            pl, win_lo, win_hi, cx, cz, has_prev_seg, prev_seg);
    }

    if (!sum.nonempty || sum.min_dist <= BIKE_REAL(Real, 1e-12)) {
        err = ErrCode::degenerate_contact;
        return Status::invalid_argument;
    }

    int winner_seg = sum.winner_seg;
    Real winner_dist = sum.winner_dist;
    Real winner_ptx = sum.winner_ptx;
    Real winner_ptz = sum.winner_ptz;

    if (sum.has_prev && sum.prev_keep &&
        TK::abs(sum.prev_dist - winner_dist) <= BIKE_REAL(Real, 1e-12)) {
        winner_seg = prev_seg;
        winner_dist = sum.prev_dist;
        winner_ptx = sum.prev_ptx;
        winner_ptz = sum.prev_ptz;
    }

    const Real delta_winner = radius - winner_dist;
    const Real normal_winner[2] = {
        TK::div(cx - winner_ptx, winner_dist),
        TK::div(cz - winner_ptz, winner_dist)
    };

    const bool multi = MultiSupportPolicy::template evaluate<Real, TK>(
        pl, win_lo, win_hi, cx, cz, radius,
        winner_ptx, winner_ptz, delta_winner, normal_winner);

    if (!TK::isfinite(winner_ptx) || !TK::isfinite(winner_ptz) ||
        !TK::isfinite(normal_winner[0]) || !TK::isfinite(normal_winner[1])) {
        err = ErrCode::degenerate_contact;
        return Status::invalid_argument;
    }
    const Real unit_tol = sizeof(Real) == 4
        ? BIKE_REAL(Real, 1e-5)
        : BIKE_REAL(Real, 1e-10);
    if (!(TK::abs(TK::hypot2(normal_winner[0], normal_winner[1]) - static_cast<Real>(1)) <=
          unit_tol)) {
        err = ErrCode::nonunit_normal;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(delta_winner)) {
        err = ErrCode::overflow_nonfinite;
        return Status::invalid_argument;
    }

    out.point[0] = winner_ptx;
    out.point[1] = winner_ptz;
    out.normal[0] = normal_winner[0];
    out.normal[1] = normal_winner[1];
    out.delta = delta_winner;
    out.segment_id = winner_seg;
    out.multi_support = multi ? static_cast<u8>(1) : static_cast<u8>(0);
    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
