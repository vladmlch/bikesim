// tyre/profile.cpp — expression-for-expression port of contact_profile.py
// plus compiled_profile_vertices (tire_forces.py:18-44). Comments cite the
// source lines at the time of porting.
//
// NumPy/CPython lowering decisions (each verified bitwise by probe):
//   `a @ b` / `np.dot` 1-D@1-D          → blas::ddot  (same numpy libcall)
//   `normals @ n`  (k,2)@(2,)           → blas::dgemv(row_major, no)
//   `local @ R.T`  (cols,3)@(3,3).T     → blas::dgemm(row_major, no, yes)
//   `np.linalg.norm(v)` on (2,)/(3,)    → sqrt(ddot(v, v))
//   `np.linalg.norm(M, axis=1)` on (k,2)→ elementwise sqrt(x*x + y*y)
//   `np.einsum('ij,ij->i')` on (n,2)    → seeded `0.0 + a0*b0 + a1*b1`
//   `np.interp` scalar                  → np_interp() below (fma form)
//   `math.hypot`                        → py_hypot()
//                                       (numeric::python_vector_norm)
//   `np.searchsorted` right/left        → upper_bound/lower_bound
//   `np.linspace(-s, s, n)`             → i*step + (-s), last element = s
//   `np.cos/np.radians/np.exp` scalars  → std::cos / x*(pi/180) / std::exp
//   `round()`                           → std::nearbyint (banker's, default FP)
#include "profile.hpp"

#include "../cblas_abi.hpp"
#include "../model_access.hpp"
#include "../numeric_norm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
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

    // np.clip(t, 0., 1.): comparisons propagate NaN like np.maximum/np.minimum
    // do for these bounds (inputs are finite in every caller anyway).
    double clip01(double t) { return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t); }

    // np.min on a non-empty span of finite values.
    double min_elt(std::span<const double> v) {
        double m = v.front();
        for (const double x: v)
            if (x < m)
                m = x;
        return m;
    }

    // ProfileContact.__post_init__ (contact_profile.py:23-29).
    biketyre::ProfileContact make_contact(std::array<double, 2> point,
                                          std::array<double, 2> normal,
                                          double delta, int segment,
                                          bool multi) {
        for (const double v: point)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "invalid shape or non-finite contact point");
        for (const double v: normal)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "invalid shape or non-finite contact normal");
        // math.isclose(hypot(n0,n1), 1., rel_tol=0., abs_tol=1e-10)
        if (!(std::abs(biketyre::py_hypot(normal[0], normal[1]) - 1.0) <= 1e-10))
            throw std::invalid_argument("contact normal must be a unit vector");
        if (!std::isfinite(delta))
            throw std::invalid_argument("invalid shape or non-finite penetration");
        return {.point = point, .normal = normal, .delta = delta, .segment_id = segment, .multi_support = multi};
    }

} // namespace

namespace biketyre {
    double py_hypot(double a, double b) {
        // CPython math.hypot — the local vector_norm port moved to
        // numeric::python_vector_norm unchanged (same fabs scan and the
        // same DoubleLength compensated kernel, verified bitwise on the
        // math.hypot oracle corpus).
        const std::array<double, 2> values = {a, b};
        return numeric::python_vector_norm(values);
    }

    double np_interp(double xq, std::span<const double> xs,
                     std::span<const double> ys) {
        // Exported boundary: an empty table or an xs/ys width mismatch is
        // an explicit rejection (np.interp raises on an empty table too);
        // the interpolation math below is the validated-table inner loop.
        if (xs.empty() || xs.size() != ys.size())
            throw std::invalid_argument(
                "interp tables must be nonempty and equally wide");
        // numpy/_core/multiarray/compiled_base.c: np_interp with lenxp > 1,
        // needs_right = 0 (float64/float64 arrays carry no NaN mask here).
        const std::size_t n = xs.size();
        const double lval = ys.front();
        const double rval = ys.back();
        // NaN-query note: np.interp returns NaN early without searching; this
        // falls through upper_bound to j=-1 → lval. Out of contract — all
        // callers gate non-finite queries upstream (profile.cpp:~441).
        // binary_search_with_guess(len, key, arr, 1): for an in-range key this
        // is the largest j with arr[j] <= key — upper_bound(key) - 1.
        int j = 0;
        if (xq < xs.front())
            j = -1;
        else if (xq > xs.back())
            j = static_cast<int>(n);
        else
            j = static_cast<int>(
                    std::ranges::upper_bound(xs, xq) - xs.begin()) - 1;
        if (j == -1)
            return lval;
        if (std::cmp_equal(j, n))
            return rval;
        if (j == static_cast<int>(n) - 1)
            return rval;
        const std::size_t ju = static_cast<std::size_t>(j);
        if (xs[ju] == xq)
            return ys[ju];
        const double slope = (ys[ju + 1] - ys[ju]) / (xs[ju + 1] - xs[ju]);
        double result = std::fma(slope, xq - xs[ju], ys[ju]);
        if (std::isnan(result)) {
            result = std::fma(slope, xq - xs[ju + 1], ys[ju + 1]);
            if (std::isnan(result) && ys[ju] == ys[ju + 1])
                result = ys[ju];
        }
        return result;
    }

    std::vector<std::array<double, 2> >
    compiled_profile_vertices(const mjModel *model, const mjData *data,
                              const char *terrain_name) {
        // tire_forces.py:24-44. resolve_id(model, mjOBJ_GEOM, terrain_name).
        const int geom = mj_name2id(model, mjOBJ_GEOM, terrain_name);
        if (geom < 0)
            throw std::invalid_argument("model has no '" +
                                        std::string(terrain_name) + "'");
        model_access::require_id(geom, model->ngeom, "terrain geom");
        const std::size_t g = static_cast<std::size_t>(geom);
        const std::span<const int> geom_type = model_access::readonly_buffer(
            model->geom_type, model->ngeom);
        const std::span<const int> geom_dataid = model_access::readonly_buffer(
            model->geom_dataid, model->ngeom);
        if (geom_type[g] != mjGEOM_HFIELD)
            throw std::invalid_argument(
                "compliant_2d requires the compiled terrain heightfield");
        const int hfield = geom_dataid[g];
        model_access::require_id(hfield, model->nhfield,
                                 "terrain heightfield ID");
        const std::size_t hf = static_cast<std::size_t>(hfield);
        const std::span<const int> nrow = model_access::readonly_buffer(
            model->hfield_nrow, model->nhfield);
        const std::span<const int> ncol = model_access::readonly_buffer(
            model->hfield_ncol, model->nhfield);
        const std::span<const int> adr = model_access::readonly_buffer(
            model->hfield_adr, model->nhfield);
        const int rows = nrow[hf];
        const int cols = ncol[hf];
        const int start = adr[hf];
        if (cols < 2 || rows < 2 || start < 0)
            throw std::invalid_argument("invalid compiled heightfield raster");
        const std::span<const float> hfield_data =
                model_access::readonly_buffer(model->hfield_data,
                                              model->nhfielddata);
        // Positive rows/cols are cast to the element width before the
        // multiply; the storage limit is the engine's mjtSize
        // (nhfielddata), never an int intermediate product.
        const std::size_t raster_count = model_access::checked_product(
            static_cast<std::size_t>(rows), static_cast<std::size_t>(cols),
            hfield_data.size());
        const std::size_t raster_end = model_access::checked_sum(
            static_cast<std::size_t>(start), raster_count,
            hfield_data.size());
        const std::size_t raster_first = static_cast<std::size_t>(start);
        const std::span<const float> raster = hfield_data.subspan(
            raster_first, raster_end - raster_first);
        const std::size_t cs = static_cast<std::size_t>(cols);
        for (const float v: raster)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "invalid compiled heightfield raster");
        // np.allclose(raster, raster[0], rtol=0, atol=1e-10) — float32 math:
        // the diff is f32 and the Python-float atol is weak → f32 compare.
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                if (!(std::abs(raster[static_cast<std::size_t>(r) * cs +
                                      static_cast<std::size_t>(c)] -
                               raster[static_cast<std::size_t>(c)]) <= 1e-10f))
                    throw std::invalid_argument(
                        "compliant_2d does not support lateral terrain "
                        "variation");
        const std::span<const mjtNum> hsize =
                model_access::readonly_buffer(model->hfield_size,
                                              4 * model->nhfield);
        const double sx = hsize[4 * static_cast<std::size_t>(hfield)];
        const double sz = hsize[4 * static_cast<std::size_t>(hfield) + 2];
        const std::span<const mjtNum> xmat = model_access::readonly_buffer(
            data->geom_xmat,
            model_access::kRotationElements * model->ngeom);
        const std::span<const mjtNum> xpos = model_access::readonly_buffer(
            data->geom_xpos, model_access::kXYZ * model->ngeom);
        const std::size_t gs = static_cast<std::size_t>(geom);
        // Nonfinite transform entries are rejected before the tolerance
        // comparisons — NaN must fail as "invalid transform", not just fall
        // out of an allclose predicate.
        for (int k = 0; k < model_access::kRotationElements; ++k)
            if (!std::isfinite(xmat[
                    model_access::kRotationElements * gs +
                    static_cast<std::size_t>(k)]))
                throw std::invalid_argument(
                    "invalid terrain transform: non-finite entry");
        for (int k = 0; k < model_access::kXYZ; ++k)
            if (!std::isfinite(xpos[model_access::kXYZ * gs +
                                    static_cast<std::size_t>(k)]))
                throw std::invalid_argument(
                    "invalid terrain transform: non-finite entry");
        // np.allclose(R[:, 1], [0,1,0], rtol=0, atol=1e-10)
        if (!(std::abs(xmat[model_access::kRotationElements * gs + 1]) <=
                  1e-10 &&
              std::abs(xmat[model_access::kRotationElements * gs + 4] -
                       1.0) <= 1e-10 &&
              std::abs(xmat[model_access::kRotationElements * gs + 7]) <=
                  1e-10))
            throw std::invalid_argument(
                "terrain transform must preserve the planar Y axis");
        // local = column_stack((linspace(-sx, sx, cols), 0, raster[0]*sz))
        // (cols,3) XYZ rows — the 3 is the spatial width.
        const std::size_t xyz = static_cast<std::size_t>(model_access::kXYZ);
        std::vector<double> local(xyz * cs);
        const double delta = sx - (-sx); // np.subtract(stop, start)
        const double step = delta / static_cast<double>(cols - 1);
        for (std::size_t i = 0; i < cs; ++i) {
            local[xyz * i] = static_cast<double>(i) * step + (-sx);
            local[xyz * i + 1] = 0.0;
            local[xyz * i + 2] = static_cast<double>(raster[i]) * sz;
        }
        local[xyz * (cs - 1)] = sx; // linspace's y[-1] = stop
        // world = local @ R.T + xpos — dgemm NoTrans/Trans, then the broadcast
        // elementwise add (two separate numpy roundings).
        std::vector<double> world(xyz * cs);
        blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                    blas::Transpose::yes, cols, model_access::kXYZ,
                    model_access::kXYZ, 1.0, local.data(),
                    model_access::kXYZ,
                    &xmat[model_access::kRotationElements * gs],
                    model_access::kXYZ, 0.0, world.data(),
                    model_access::kXYZ);
        for (std::size_t i = 0; i < cs; ++i)
            for (int j = 0; j < model_access::kXYZ; ++j)
                world[xyz * i + static_cast<std::size_t>(j)] +=
                        xpos[model_access::kXYZ * gs +
                             static_cast<std::size_t>(j)];
        // np.allclose(world[:, 1], 0., rtol=0, atol=1e-9)
        for (std::size_t i = 0; i < cs; ++i)
            if (!(std::abs(world[xyz * i + 1]) <= 1e-9))
                throw std::invalid_argument(
                    "working profile must lie in world Y=0");
        std::vector<std::array<double, 2> > vertices;
        vertices.reserve(cs);
        for (std::size_t i = 0; i < cs; ++i)
            vertices.push_back({world[xyz * i], world[xyz * i + 2]});
        return vertices;
    }

    ProfileQuery::ProfileQuery(std::vector<double> px, std::vector<double> pz,
                               double significant_delta_m,
                               double significance_fraction,
                               double normal_angle_deg)
        : px_(std::move(px)), pz_(std::move(pz)) {
        // contact_profile.py:36-57 — shape/finiteness checks (the C++ ctor
        // takes the two columns separately, so the ndim/shape[1] part is the
        // size agreement between them and the >=2 row count).
        if (px_.size() < 2 || pz_.size() != px_.size())
            throw std::invalid_argument(
                "profile must contain at least two X-Z vertices");
        for (const double v: px_)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "invalid shape or non-finite profile vertices");
        for (const double v: pz_)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "invalid shape or non-finite profile vertices");
        const int count = static_cast<int>(px_.size()) - 1;
        seg_x_.resize(static_cast<std::size_t>(count));
        seg_z_.resize(static_cast<std::size_t>(count));
        seg_len_sq_.resize(static_cast<std::size_t>(count));
        height_changes_.resize(px_.size());
        for (int i = 0; i < count; ++i) {
            const std::size_t u = static_cast<std::size_t>(i);
            seg_x_[u] = px_[u + 1] - px_[u]; // np.diff, elementwise
            seg_z_[u] = pz_[u + 1] - pz_[u];
            // einsum('ij,ij->i', segs, segs): seeded accumulation.
            double s = 0.0;
            s += seg_x_[u] * seg_x_[u];
            s += seg_z_[u] * seg_z_[u];
            seg_len_sq_[u] = s;
        }
        for (int i = 0; i < count; ++i)
            if (!(px_[static_cast<std::size_t>(i) + 1] -
                  px_[static_cast<std::size_t>(i)] > 0.0))
                throw std::invalid_argument("profile x must increase strictly");
        height_changes_[0] = 0;
        for (int i = 0; i < count; ++i)
            height_changes_[static_cast<std::size_t>(i) + 1] =
                    height_changes_[static_cast<std::size_t>(i)] +
                    (seg_z_[static_cast<std::size_t>(i)] != 0.0 ? 1 : 0);
        // _left_directions = vstack((zeros(2), -segs));
        // _right_directions = vstack((segs, zeros(2))).
        left_x_.resize(px_.size());
        left_z_.resize(px_.size());
        right_x_.resize(px_.size());
        right_z_.resize(px_.size());
        left_x_[0] = 0.0;
        left_z_[0] = 0.0;
        for (int v = 1; v <= count; ++v) {
            const std::size_t u = static_cast<std::size_t>(v);
            left_x_[u] = -seg_x_[u - 1];
            left_z_[u] = -seg_z_[u - 1];
        }
        for (int v = 0; v < count; ++v) {
            const std::size_t u = static_cast<std::size_t>(v);
            right_x_[u] = seg_x_[u];
            right_z_[u] = seg_z_[u];
        }
        right_x_[static_cast<std::size_t>(count)] = 0.0;
        right_z_[static_cast<std::size_t>(count)] = 0.0;
        maximum_z_ = *std::ranges::max_element(pz_);
        // scalar() validation for the three thresholds (checks.py:7-20).
        if (!std::isfinite(significant_delta_m) || significant_delta_m < 0.0)
            throw std::invalid_argument("invalid significant penetration");
        if (!std::isfinite(significance_fraction) || significance_fraction < 0.0)
            throw std::invalid_argument("invalid significance fraction");
        if (!std::isfinite(normal_angle_deg) || normal_angle_deg <= 0.0)
            throw std::invalid_argument("invalid normal angle");
        if (significance_fraction > 1.0 || normal_angle_deg >= 180.0)
            throw std::invalid_argument("invalid multi-support threshold");
        significant_delta_m_ = significant_delta_m;
        significance_fraction_ = significance_fraction;
        // np.cos(np.deg2rad(angle)) — deg2rad is x*(pi/180), cos is libm cos.
        normal_cosine_ =
                std::cos(normal_angle_deg * (std::numbers::pi / 180.0));
        // Warm-contact scratch: contact() refills this storage every call
        // instead of allocating per query like the Python original. A
        // candidates window never exceeds the segment count, so px_.size()
        // bounds every row the scratch can hold.
        const std::size_t bound = px_.size();
        cand_scratch_.ids.reserve(bound);
        cand_scratch_.t.reserve(bound);
        cand_scratch_.points.reserve(bound);
        cand_scratch_.distances.reserve(bound);
        keep_scratch_.resize(bound);
        near_scratch_.reserve(bound);
        normals_scratch_.resize(2 * bound);
        dots_scratch_.resize(bound);
    }

    namespace detail {
        void candidates_into(std::span<const double> px,
                             std::span<const double> pz,
                             std::span<const double> seg_x,
                             std::span<const double> seg_z,
                             std::span<const double> seg_len_sq,
                             std::array<double, 2> c, int lo, int hi,
                             Candidates &out) {
            // Table-shape boundary first: the projection loop indexes
            // px[u]/seg_x[u]/seg_len_sq[u] for u in [lo, hi), so a
            // mismatched or empty table is an explicit rejection, not an
            // unchecked read.
            const std::size_t segs = seg_x.size();
            if (px.size() != pz.size() || px.size() != segs + 1 ||
                seg_z.size() != segs || seg_len_sq.size() != segs)
                throw std::invalid_argument(
                    "candidate tables must come from one validated profile");
            out.ids.clear();
            out.t.clear();
            out.points.clear();
            out.distances.clear();
            // Interval contract (np.arange semantics): hi <= lo is the
            // declared empty interval — zero candidates, never a negative
            // reserve. A nonempty window must lie inside the segment table.
            if (hi <= lo)
                return;
            if (lo < 0 || std::cmp_greater(hi, segs))
                throw std::invalid_argument(
                    "candidate window outside segment range");
            // _candidates (contact_profile.py:59-68): ids = arange(lo, hi),
            // project c onto every window segment, clip t into [0,1],
            // measure the gap. Inner loop stays unchecked — the ctor's
            // validated profile plus the window bounds above cover it.
            // reserve() is a no-op once the owner's capacity covers the
            // bound, so warm calls never reach the allocator.
            const std::size_t n = static_cast<std::size_t>(hi - lo);
            out.ids.reserve(n);
            out.t.reserve(n);
            out.points.reserve(n);
            out.distances.reserve(n);
            for (int i = lo; i < hi; ++i) {
                const std::size_t u = static_cast<std::size_t>(i);
                out.ids.push_back(i);
                const double ox = c[0] - px[u], oz = c[1] - pz[u];
                // einsum('ij,ij->i', c-a, d): seeded per-row accumulation.
                double num = 0.0;
                num += ox * seg_x[u];
                num += oz * seg_z[u];
                const double t = clip01(num / seg_len_sq[u]);
                out.t.push_back(t);
                // points = a + t[:, None]*d  (mul, then add — two roundings)
                const double ptx = px[u] + t * seg_x[u];
                const double ptz = pz[u] + t * seg_z[u];
                out.points.push_back({ptx, ptz});
                const double dx = c[0] - ptx, dz = c[1] - ptz;
                // sqrt(einsum('ij,ij->i', diff, diff))
                double dsq = 0.0;
                dsq += dx * dx;
                dsq += dz * dz;
                out.distances.push_back(std::sqrt(dsq));
            }
        }

        Candidates candidates(std::span<const double> px,
                              std::span<const double> pz,
                              std::span<const double> seg_x,
                              std::span<const double> seg_z,
                              std::span<const double> seg_len_sq,
                              std::array<double, 2> c, int lo, int hi) {
            Candidates out;
            candidates_into(px, pz, seg_x, seg_z, seg_len_sq, c, lo, hi,
                            out);
            return out;
        }
    } // namespace detail

    void ProfileQuery::candidates_into(std::array<double, 2> c, int lo,
                                       int hi,
                                       detail::Candidates &out) const {
        detail::candidates_into(px_, pz_, seg_x_, seg_z_, seg_len_sq_, c,
                                lo, hi, out);
    }

    void ProfileQuery::endpoint_keep_into(
        std::array<double, 2> c, const detail::Candidates &cand,
        std::vector<bool> &out) const {
        // _endpoint_keep (contact_profile.py:70-85): an endpoint projection is
        // not a separate support when a neighbour's direction of travel moves
        // closer to the wheel. Written into the caller's mask — only the
        // cand.ids.size() prefix is touched.
        for (std::size_t i = 0; i < cand.ids.size(); ++i) {
            out[i] = true;
            const double t = cand.t[i];
            if (t != 0.0 && t != 1.0)
                continue; // not an endpoint projection
            const int vertex = cand.ids[i] + (t == 1.0 ? 1 : 0);
            const std::size_t v = static_cast<std::size_t>(vertex);
            const double ox = c[0] - px_[v], oz = c[1] - pz_[v];
            // einsum('ij,ij->i', offsets, directions[vertex_ids]) per table.
            double dl = 0.0;
            dl += ox * left_x_[v];
            dl += oz * left_z_[v];
            if (dl > 1e-14)
                out[i] = false;
            double dr = 0.0;
            dr += ox * right_x_[v];
            dr += oz * right_z_[v];
            if (dr > 1e-14)
                out[i] = false;
        }
    }

    ProfileContact ProfileQuery::contact(
        std::array<double, 2> c, double radius,
        std::optional<int> previous_segment) const {
        // contact_profile.py:88-102 — argument validation in source order.
        if (!std::isfinite(c[0]) || !std::isfinite(c[1]))
            throw std::invalid_argument(
                "invalid shape or non-finite wheel center");
        if (!std::isfinite(radius) || radius <= 0.0)
            throw std::invalid_argument("invalid wheel radius");
        const int count = static_cast<int>(px_.size()) - 1;
        if (previous_segment &&
            (*previous_segment < 0 || *previous_segment >= count))
            throw std::invalid_argument("invalid previous contact segment");
        if (!(px_.front() <= c[0] && c[0] <= px_.back()))
            throw std::invalid_argument(
                "wheel center is outside profile domain");
        if (c[1] <= np_interp(c[0], px_, pz_))
            throw std::invalid_argument(
                "wheel center reached or entered solid road");
        const int lo = std::max(0, search_right(px_, c[0] - radius) - 2);
        const int hi = std::min(count, search_right(px_, c[0] + radius) + 1);
        const double height = c[1] - pz_[static_cast<std::size_t>(lo)];
        if (height < radius &&
            height_changes_[static_cast<std::size_t>(hi)] ==
            height_changes_[static_cast<std::size_t>(lo)]) {
            // Flat-window fast path (contact_profile.py:104-121).
            int segment = std::max(0, search_left(px_, c[0]) - 1);
            std::array<double, 2> point = {c[0], pz_[static_cast<std::size_t>(lo)]};
            double distance = height;
            if (previous_segment && lo <= *previous_segment &&
                *previous_segment < hi) {
                const std::size_t pv = static_cast<std::size_t>(*previous_segment);
                const double previous_x =
                        std::min(std::max(c[0], px_[pv]), px_[pv + 1]);
                const double previous_distance =
                        py_hypot(c[0] - previous_x, height);
                const int vertex = previous_x == px_[pv]
                                       ? *previous_segment
                                       : *previous_segment + 1;
                const bool endpoint = previous_x == px_[pv] ||
                                      previous_x == px_[pv + 1];
                // eligible = not endpoint or not any(offset @ dir > 1e-14)
                bool closer = false;
                if (endpoint) {
                    const std::size_t vv = static_cast<std::size_t>(vertex);
                    const std::array<double, 2> off = {
                        c[0] - px_[vv],
                        c[1] - pz_[vv]
                    };
                    const std::array<double, 2> left = {left_x_[vv], left_z_[vv]};
                    const std::array<double, 2> right = {
                        right_x_[vv],
                        right_z_[vv]
                    };
                    closer = blas::ddot(2, off.data(), 1, left.data(), 1) >
                                 1e-14 ||
                             blas::ddot(2, off.data(), 1, right.data(), 1) >
                                 1e-14;
                }
                if (!closer && std::abs(previous_distance - distance) <= 1e-12) {
                    segment = *previous_segment;
                    point[0] = previous_x;
                    distance = previous_distance;
                }
            }
            if (distance <= 1e-12)
                throw std::invalid_argument(
                    "unsupported or degenerate contact geometry");
            return make_contact(point,
                                {
                                    (c[0] - point[0]) / distance,
                                    (c[1] - point[1]) / distance
                                },
                                radius - distance, segment, false);
        }
        // candidates path (contact_profile.py:122-165) — the member scratch
        // vectors replace the Python temporaries; their construction-sized
        // capacity keeps steady-state queries off the allocator.
        candidates_into(c, lo, hi, cand_scratch_);
        if (cand_scratch_.ids.empty() ||
            min_elt(cand_scratch_.distances) >= radius) {
            int search_lo = 0, search_hi = 0;
            if (!cand_scratch_.ids.empty()) {
                const double best = min_elt(cand_scratch_.distances);
                const double vertical_lower =
                        std::max(0.0, c[1] - maximum_z_);
                const double reach = std::sqrt(std::max(
                                         0.0, best * best - vertical_lower * vertical_lower)) + 1e-8;
                search_lo = std::max(0, search_right(px_, c[0] - reach) - 2);
                search_hi =
                        std::min(count, search_right(px_, c[0] + reach) + 1);
            } else {
                search_lo = 0;
                search_hi = count;
            }
            candidates_into(c, search_lo, search_hi, cand_scratch_);
        }
        if (cand_scratch_.ids.empty() ||
            min_elt(cand_scratch_.distances) <= 1e-12)
            throw std::invalid_argument(
                "unsupported or degenerate contact geometry");
        endpoint_keep_into(c, cand_scratch_, keep_scratch_);
        // np.argmin(distances): first index of the minimum.
        std::size_t winner = 0;
        for (std::size_t i = 1; i < cand_scratch_.distances.size(); ++i)
            if (cand_scratch_.distances[i] < cand_scratch_.distances[winner])
                winner = i;
        if (previous_segment) {
            // same = flatnonzero((ids == prev) & keep); same[0] if it exists.
            for (std::size_t i = 0; i < cand_scratch_.ids.size(); ++i)
                if (cand_scratch_.ids[i] == *previous_segment &&
                    keep_scratch_[i]) {
                    if (std::abs(cand_scratch_.distances[i] -
                                 cand_scratch_.distances[winner]) <=
                        1e-12)
                        winner = i;
                    break;
                }
        }
        const double delta_winner = radius - cand_scratch_.distances[winner];
        const std::array<double, 2> normal_winner = {
            (c[0] - cand_scratch_.points[winner][0]) /
                cand_scratch_.distances[winner],
            (c[1] - cand_scratch_.points[winner][1]) /
                cand_scratch_.distances[winner]
        };
        const double threshold =
                std::max(significant_delta_m_,
                         significance_fraction_ * std::max(delta_winner, 0.0));
        near_scratch_.clear();
        for (std::size_t i = 0; i < cand_scratch_.ids.size(); ++i)
            if (radius - cand_scratch_.distances[i] >= threshold &&
                keep_scratch_[i])
                near_scratch_.push_back(static_cast<int>(i));
        bool multi = false;
        if (!near_scratch_.empty()) {
            const std::size_t near_n = near_scratch_.size();
            // normals = (c - points[near]) / distances[near, None]
            const std::span<double> normals =
                    std::span{normals_scratch_}.first(2 * near_n);
            for (std::size_t j = 0; j < near_n; ++j) {
                const std::size_t i =
                        static_cast<std::size_t>(near_scratch_[j]);
                normals[2 * j] = (c[0] - cand_scratch_.points[i][0]) /
                                     cand_scratch_.distances[i];
                normals[2 * j + 1] = (c[1] - cand_scratch_.points[i][1]) /
                                         cand_scratch_.distances[i];
            }
            // different = normals @ normal_winner < normal_cosine — a
            // (k,2)@(2,) matmul → gemv path, not a per-row manual dot.
            const std::span<double> dots =
                    std::span{dots_scratch_}.first(near_n);
            blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                        static_cast<int>(near_n), 2, 1.0,
                        normals.data(), 2, normal_winner.data(), 1, 0.0,
                        dots.data(), 1);
            for (std::size_t j = 0; j < near_n; ++j) {
                const std::size_t i =
                        static_cast<std::size_t>(near_scratch_[j]);
                const double ex = cand_scratch_.points[i][0] -
                                      cand_scratch_.points[winner][0];
                const double ez = cand_scratch_.points[i][1] -
                                      cand_scratch_.points[winner][1];
                // np.linalg.norm(..., axis=1) on (k,2): elementwise sqrt(x²+y²).
                const bool separated = std::sqrt(ex * ex + ez * ez) > 1e-10;
                if (dots[j] < normal_cosine_ && separated) {
                    multi = true;
                    break;
                }
            }
        }
        return make_contact(cand_scratch_.points[winner], normal_winner,
                            delta_winner, cand_scratch_.ids[winner], multi);
    }
} // namespace biketyre
