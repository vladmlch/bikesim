// attachment_wrench.cpp — the attachment_wrench.py port (see the header
// for the contract). Nothing in this TU touches Python: the binding in
// attachment_binding.cpp converts to/from dicts at the seam, and the
// numerics below mirror the oracle's operation order bitwise:
//
//   * mj_jac fills the (6, nv) spatial block — rows 0-2 are jacp, rows
//     3-5 are jacr — invoked directly: like mj_mulJacTVec in
//     equality_reactions.cpp it has no reachable mju_error for these
//     inputs (verified against the pinned 3.12.0 dylib: out-of-range
//     bodies zero-fill, nonfinite points propagate as NaN columns).
//     Out-of-range body ids are gated to that same zero block BEFORE
//     the call rather than relying on the engine's arena-adjacent
//     reads — the observable contract (zero jacobian -> 'no dof') is
//     identical and never undefined.
//   * Column support is np.flatnonzero(np.abs(jac).max(axis=0) > 0):
//     maximum.reduce propagates NaN, so NaN columns are excluded like
//     the oracle's.
//   * Reconstruction/matvec go through Accelerate's legacy CBLAS — the
//     entry points numpy's A.T @ x / (3,3) @ x / np.linalg.norm /
//     vector @ vector dispatch to (cblas_abi.hpp's verified mapping);
//     norms are sqrt(ddot(...)) exactly like np.linalg.norm.
//   * np.linalg.lstsq is the shared ILP64 DGELSD workspace at the
//     oracle's rcond = 1e-12; recover_wrench additionally requires full
//     column rank of the transposed matrix.
//   * np.allclose is numpy 2.5's asymmetric elementwise gate:
//     (|a-b| <= atol + rtol*|b| AND isfinite(b)) OR (a == b) — the
//     second operand is the relative reference, inf==inf passes on the
//     equality escape, NaN never passes.
//   * Python indexing on eq_type / xpos / qfrc[columns] / efc_pos[idx] /
//     wrench[i]: negatives wrap, out-of-range throws std::out_of_range,
//     which nanobind surfaces as IndexError — numpy's own exception.
#include "attachment_wrench.hpp"

#include "../cblas_abi.hpp"
#include "../model_access.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>

namespace rider {

    namespace {

        // numpy's IndexError wording ("index i is out of bounds for
        // axis 0 with size n") — the ORIGINAL index is reported, like
        // numpy, not the wrapped candidate.
        [[noreturn]] void throw_index(std::int64_t index, std::int64_t size) {
            throw std::out_of_range("index " + std::to_string(index) +
                                    " is out of bounds for axis 0 with size " +
                                    std::to_string(size));
        }

        // 1-D Python indexing semantics for engine buffers: negatives
        // wrap by the extent; an unresolvable index is IndexError.
        std::int64_t wrapped_index(std::int64_t index, std::int64_t size) {
            std::int64_t wrapped = index;
            if (wrapped < 0) wrapped += size;
            if (wrapped < 0 || wrapped >= size) throw_index(index, size);
            return wrapped;
        }

        // Scalar/vector indexing used by decompose_wrench's wrench[i].
        double at(std::span<const double> values, std::int64_t index) {
            return values[static_cast<std::size_t>(
                wrapped_index(index, static_cast<std::int64_t>(values.size())))];
        }

        bool all_finite(std::span<const double> values) {
            return std::ranges::all_of(
                values, [](double v) { return std::isfinite(v); });
        }

        // numpy 2.5's elementwise isclose:
        //   (|a-b| <= atol + rtol*|b| AND isfinite(b)) OR (a == b)
        // `b` is the asymmetric reference — rtol scales with |b| and the
        // inequality path requires b finite; the equality escape admits
        // inf==inf and signed-zero equality; NaN never passes.
        bool is_close(double a, double b, double rtol, double atol) {
            const double limit = atol + rtol * std::fabs(b);
            return (std::fabs(a - b) <= limit && std::isfinite(b)) ||
                   a == b;
        }

        // np.allclose — all(isclose(a, b)); a shape mismatch is numpy's
        // broadcast failure, which surfaces as ValueError like the
        // oracle's other measurement errors.
        bool all_close(std::span<const double> a, std::span<const double> b,
                       double rtol, double atol) {
            if (a.size() != b.size())
                throw std::invalid_argument(
                    "operands could not be broadcast together");
            for (std::size_t i = 0; i < a.size(); ++i)
                if (!is_close(a[i], b[i], rtol, atol)) return false;
            return true;
        }

        // np.linalg.norm(v) — sqrt(v @ v) through Accelerate's ddot, the
        // same entry point numpy dispatches (verified bitwise, see
        // cblas_abi.hpp's probe table).
        double norm(std::span<const double> v) {
            return std::sqrt(blas::ddot(static_cast<int>(v.size()),
                                        v.data(), 1, v.data(), 1));
        }

        // (np.abs(jac).max(axis=0))[column] — np.maximum.reduce: a NaN
        // element makes the column max NaN; comparison downstream then
        // excludes it, like the oracle.
        double column_abs_max(const DenseMatrix &jac, std::size_t column) {
            if (jac.rows == 0 || jac.columns == 0)
                throw std::invalid_argument(
                    "column_abs_max needs a nonempty matrix");
            double best = std::fabs(jac.values[column]);
            for (std::size_t i = 1; i < jac.rows; ++i) {
                const double v =
                    std::fabs(jac.values[i * jac.columns + column]);
                if (std::isnan(v) || v > best) best = v;
            }
            return best;
        }

        // np.abs(jac[row, cols]).max() — the same NaN-propagating reduce
        // over the selected columns.
        double row_abs_max_at(const DenseMatrix &jac, std::size_t row,
                              std::span<const std::int64_t> cols) {
            double best = std::fabs(
                jac.values[row * jac.columns +
                           static_cast<std::size_t>(cols.front())]);
            for (std::size_t k = 1; k < cols.size(); ++k) {
                const double v = std::fabs(
                    jac.values[row * jac.columns +
                               static_cast<std::size_t>(cols[k])]);
                if (std::isnan(v) || v > best) best = v;
            }
            return best;
        }

        // np.flatnonzero(np.abs(jac).max(axis=0) > 0) — ascending int64
        // dof columns; NaN column maxima compare False and drop out.
        std::vector<std::int64_t> support_columns(const DenseMatrix &jac) {
            std::vector<std::int64_t> cols;
            for (std::size_t j = 0; j < jac.columns; ++j)
                if (column_abs_max(jac, j) > 0.)
                    cols.push_back(static_cast<std::int64_t>(j));
            return cols;
        }

        // jac[:, cols].copy() — the (rows, k) restricted block.
        DenseMatrix gather_columns(const DenseMatrix &jac,
                                   std::span<const std::int64_t> cols) {
            DenseMatrix out{
                .rows = jac.rows, .columns = cols.size(),
                .values = std::vector<double>(jac.rows * cols.size())};
            for (std::size_t j = 0; j < cols.size(); ++j) {
                const std::size_t c = static_cast<std::size_t>(cols[j]);
                for (std::size_t i = 0; i < jac.rows; ++i)
                    out.values[i * cols.size() + j] =
                        jac.values[i * jac.columns + c];
            }
            return out;
        }

        // qfrc[cols].copy() — numpy fancy indexing: negatives wrap, an
        // unresolvable column is IndexError.
        std::vector<double>
        gather_values(std::span<const double> source,
                      std::span<const std::int64_t> cols) {
            std::vector<double> out(cols.size());
            for (std::size_t j = 0; j < cols.size(); ++j)
                out[j] = source[static_cast<std::size_t>(
                    wrapped_index(cols[j],
                                  static_cast<std::int64_t>(source.size())))];
            return out;
        }

        // np.intersect1d(cr, cb).size — inputs are ascending by
        // construction (np.flatnonzero order), so the two-pointer merge
        // finds any shared dof column.
        bool shares_support(std::span<const std::int64_t> a,
                            std::span<const std::int64_t> b) {
            std::size_t i = 0, j = 0;
            while (i < a.size() && j < b.size()) {
                if (a[i] == b[j]) return true;
                if (a[i] < b[j]) {
                    ++i;
                } else {
                    ++j;
                }
            }
            return false;
        }

        // The row-major transpose — the (columns, rows) copy behind
        // every jac.T operand staged for DGELSD.
        DenseMatrix transpose(const DenseMatrix &m) {
            DenseMatrix out{.rows = m.columns,
                            .columns = m.rows,
                            .values = std::vector<double>(m.values.size())};
            for (std::size_t i = 0; i < m.rows; ++i)
                for (std::size_t j = 0; j < m.columns; ++j)
                    out.values[j * m.rows + i] =
                        m.values[i * m.columns + j];
            return out;
        }

        // (jac.T) @ x for a (rows=6 or r)-by-(columns) row-major jac —
        // dgemv(RowMajor, Trans, rows, columns) is the same Fortran call
        // numpy's matvec makes for the F-order operand (the verified
        // "A.T @ x (F-view)" row in cblas_abi.hpp).
        std::vector<double> jac_t_vec(const DenseMatrix &jac,
                                      std::span<const double> x) {
            std::vector<double> out(jac.columns, 0.);
            blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                        static_cast<int>(jac.rows),
                        static_cast<int>(jac.columns), 1.,
                        jac.values.data(),
                        static_cast<int>(jac.columns), x.data(), 1, 0.,
                        out.data(), 1);
            return out;
        }

        // The full (6, nv) spatial jacobian at `point`, np.vstack((jp,
        // jr)) layout: mj_jac writes jacp into rows 0-2 and jacr into
        // rows 3-5 of one contiguous block. An out-of-range body maps to
        // the same all-zero block the oracle's mj_jac produces (the
        // engine zero-fills; verified on the pinned dylib) — without
        // depending on the engine's arena-adjacent id reads. Nonfinite
        // points propagate as NaN columns, exactly like the oracle.
        DenseMatrix body_jacobian(const mjModel *m, const mjData *d,
                                  int body, const Vec3 &point) {
            if (m->nv < 0)
                throw std::invalid_argument(
                    "attachment measurement needs a nonnegative model width");
            const std::size_t nv = static_cast<std::size_t>(m->nv);
            DenseMatrix jac{.rows = 6,
                            .columns = nv,
                            .values = std::vector<double>(6 * nv, 0.)};
            if (body < 0 || body >= m->nbody) return jac;
            // subspan keeps the 3*nv offset bound-checked
            // (-Wunsafe-buffer-usage); jacr still fills rows [3, 6).
            const std::span<double> flat(jac.values);
            mj_jac(m, d, jac.values.data(),
                   flat.subspan(3 * nv).data(), point.data(), body);
            return jac;
        }

        // data.xpos[body] + data.xmat[body].reshape(3,3) @ offset — the
        // compiled world anchor. `body` indexes xpos/xmat like numpy
        // (negatives wrap, OOB -> IndexError); the matvec is
        // cblas_dgemv(RowMajor, NoTrans, 3, 3) — numpy's own dispatch
        // for a C-order (3,3) @ (3,) (cblas_abi.hpp).
        // NOLINTBEGIN(bugprone-easily-swappable-parameters) the three
        // engine-buffer spans keep call sites in model-data order.
        Vec3 anchor_world(const mjModel *m, int body,
                          std::span<const double> offset,
                          std::span<const double> xpos,
                          std::span<const double> xmat) {
            // NOLINTEND(bugprone-easily-swappable-parameters)
            const std::size_t row = static_cast<std::size_t>(
                wrapped_index(body, m->nbody));
            std::array<double, 3> rotated{};
            blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3, 3,
                        1., xmat.subspan(9 * row, 9).data(), 3,
                        offset.data(), 1, 0., rotated.data(), 1);
            return {xpos[3 * row] + rotated[0],
                    xpos[3 * row + 1] + rotated[1],
                    xpos[3 * row + 2] + rotated[2]};
        }

        // The shared-point branch's (3,) gate — the oracle's point only
        // reaches mj_jac through _body_geometry, where the binding
        // enforces a 3-element coordinate.
        Vec3 shared_point(std::span<const double> point) {
            if (point.size() != 3)
                throw std::invalid_argument(
                    "attachment world point must be a 3-element array");
            return {point[0], point[1], point[2]};
        }

        // np.linalg.lstsq(jac.T, target, rcond=1e-12)[0] — jac is
        // (rows, k), the solve is (k, rows) row-major.
        std::vector<double> solve_transposed(LeastSquaresWorkspace &ws,
                                             const DenseMatrix &jac,
                                             std::span<const double> target) {
            const DenseMatrix matrix = transpose(jac);
            return ws.solve(matrix.values,
                            static_cast<lapack_int>(matrix.rows),
                            static_cast<lapack_int>(matrix.columns), target,
                            1).solution;
        }

        // v[:3] — numpy's first-three slice (shorter input gives the
        // shorter slice).
        std::span<const double> first3(std::span<const double> v) {
            return v.first(std::min<std::size_t>(3, v.size()));
        }

        // -v — elementwise unary minus (sign bit: -0.0 -> +0.0).
        std::vector<double> negated(std::span<const double> v) {
            std::vector<double> out(v.size());
            for (std::size_t i = 0; i < v.size(); ++i) out[i] = -v[i];
            return out;
        }

        // The oracle's validate block shared by attachment_sample and
        // attachment_raw_from_geometry's validate_wrench=True branch:
        // Newton third law then planarity, in that order, on the RIDER
        // wrench.
        void check_newton_and_planarity(std::span<const double> bike_wrench,
                                        std::span<const double> rider_wrench) {
            const double scale = std::max(1., norm(first3(rider_wrench)));
            if (!all_close(bike_wrench, negated(rider_wrench), 1e-4,
                           1e-4 * scale))
                throw std::invalid_argument(
                    "attachment wrenches fail Newton third law");
            const std::array<double, 3> out_of_plane{
                at(rider_wrench, 1), at(rider_wrench, 3),
                at(rider_wrench, 5)};
            if (!std::ranges::all_of(out_of_plane, [scale](double v) {
                    return std::fabs(v) <= 1e-6 * scale;
                }))
                throw std::invalid_argument(
                    "attachment wrench leaves the planar model");
        }

        // rows.get(eq_id) — the oracle dict lookup on the ordered map.
        const RowList *find_rows(const EqualityRowMap &rows, int eq_id) {
            for (const auto &[id, list] : rows)
                if (id == eq_id) return &list;
            return nullptr;
        }

        // 0. if idx is None or idx.size == 0 else
        // float(np.linalg.norm(data.efc_pos[idx][:3])) — idx comes from
        // the current arena map (or a caller's pass-through) and is
        // bounds-checked like the oracle's fancy indexing.
        double attachment_gap(const mjData *d, const RowList *idx) {
            if (idx == nullptr || idx->empty()) return 0.;
            const std::span<const double> pos =
                model_access::readonly_buffer(d->efc_pos, d->nefc,
                                              "efc_pos");
            const std::size_t count = std::min<std::size_t>(3, idx->size());
            std::array<double, 3> gathered{};
            for (std::size_t i = 0; i < count; ++i)
                gathered[i] = pos[static_cast<std::size_t>(
                    wrapped_index((*idx)[i], d->nefc))];
            return std::sqrt(blas::ddot(static_cast<int>(count),
                                        gathered.data(), 1,
                                        gathered.data(), 1));
        }

        // The rows= pass-through shared by raw/sample: nullptr derives a
        // fresh equality_rows(data) map — the oracle's
        // `if rows is None: rows = equality_rows(data)` fallback.
        const RowList *attachment_index(const mjData *d,
                                        const EqualityReactions &reactions,
                                        const EqualityRowMap *rows, int eq_id,
                                        EqualityRowMap &local) {
            if (rows == nullptr) {
                local = reactions.equality_rows(d);
                return find_rows(local, eq_id);
            }
            return find_rows(*rows, eq_id);
        }

        // -0.0-safe copy of a stored vector input (np.array(x,
        // copy=True)).
        std::vector<double> copy_span(std::span<const double> values) {
            return {values.begin(), values.end()};
        }
    } // namespace

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional
    // args mirror the oracle's _attachment_points call order.
    std::pair<Vec3, Vec3>
    attachment_points(const mjModel *m, const mjData *d, int eq_id,
                      int body_rider, int body_bike,
                      std::span<const double> point, bool rotational) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        // `not rotational` short-circuits BEFORE model.eq_type[eq_id] is
        // indexed — a rotational measurement never touches eq_type, so a
        // bogus eq_id is only rejected on the nonrotational path.
        if (!rotational) {
            const std::span<const int> types =
                model_access::readonly_buffer(m->eq_type, m->neq,
                                              "eq_type");
            const std::int64_t index = wrapped_index(eq_id, m->neq);
            if (types[static_cast<std::size_t>(index)] == mjEQ_CONNECT) {
                // A soft connect's force acts at each body's own
                // compiled anchor — evaluating both bodies at the shared
                // point would invent a moment from the anchor gap.
                const std::span<const double> eq_data =
                    model_access::readonly_buffer(
                        m->eq_data, m->neq * mjNEQDATA, "eq_data");
                const std::span<const double> xpos =
                    model_access::readonly_buffer(
                        d->xpos, 3 * m->nbody, "xpos");
                const std::span<const double> xmat =
                    model_access::readonly_buffer(
                        d->xmat, 9 * m->nbody, "xmat");
                const std::span<const double> row = eq_data.subspan(
                    static_cast<std::size_t>(index) *
                    static_cast<std::size_t>(mjNEQDATA));
                return {anchor_world(m, body_rider, row.subspan(0, 3),
                                     xpos, xmat),
                        anchor_world(m, body_bike, row.subspan(3, 3),
                                     xpos, xmat)};
            }
        }
        const Vec3 shared = shared_point(point);
        return {shared, shared};
    }

    BodyGeometry body_geometry(const mjModel *m, const mjData *d,
                               int body, const Vec3 &point) {
        const DenseMatrix full = body_jacobian(m, d, body, point);
        std::vector<std::int64_t> cols = support_columns(full);
        if (cols.empty())
            throw std::invalid_argument(
                "attachment body carries no degrees of freedom");
        // np.abs(jac[0, cols]).max() != 0 and np.abs(jac[2, cols]).max()
        // != 0 — Python's `and` evaluates both operands.
        const bool observable = row_abs_max_at(full, 0, cols) != 0. &&
                                row_abs_max_at(full, 2, cols) != 0.;
        return BodyGeometry{.jac = gather_columns(full, cols),
                            .columns = std::move(cols),
                            .observable = observable};
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional
    // args mirror the oracle's prepare_attachment_geometry call order.
    AttachmentGeometry prepare_attachment_geometry(
        const mjModel *m, const mjData *d, int eq_id, int body_rider,
        int body_bike, std::span<const double> point,
        std::span<const double> normal, std::string kind, bool rotational,
        double half_patch_m,
        const std::optional<std::vector<double>> &pull_direction) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const auto [rider_point, bike_point] =
            attachment_points(m, d, eq_id, body_rider, body_bike, point,
                              rotational);
        BodyGeometry rider = body_geometry(m, d, body_rider, rider_point);
        BodyGeometry bike = body_geometry(m, d, body_bike, bike_point);
        if (shares_support(rider.columns, bike.columns))
            throw std::invalid_argument(
                "attachment bodies share kinematic support");
        return AttachmentGeometry{
            .eq_id = eq_id,
            .rider_jac = std::move(rider.jac),
            .rider_columns = std::move(rider.columns),
            .bike_jac = std::move(bike.jac),
            .bike_columns = std::move(bike.columns),
            .observable = rider.observable && bike.observable,
            .normal = copy_span(normal),
            .kind = std::move(kind),
            .rotational = rotational,
            .half_patch_m = half_patch_m,
            .pull_direction = pull_direction};
    }

    AttachmentRaw attachment_raw_from_geometry(
        const mjData *d, const EqualityReactions &reactions,
        LeastSquaresWorkspace &workspace, std::span<double> qfrc_scratch,
        const AttachmentGeometry &geometry, bool validate_wrench,
        const EqualityRowMap *rows) {
        reactions.equality_qfrc_into(d, geometry.eq_id, qfrc_scratch);
        std::vector<double> rider_qfrc =
            gather_values(qfrc_scratch, geometry.rider_columns);
        std::vector<double> bike_qfrc =
            gather_values(qfrc_scratch, geometry.bike_columns);
        if (validate_wrench) {
            const std::vector<double> rider_wrench = solve_transposed(
                workspace, geometry.rider_jac, rider_qfrc);
            const std::vector<double> bike_wrench = solve_transposed(
                workspace, geometry.bike_jac, bike_qfrc);
            if (!all_close(jac_t_vec(geometry.rider_jac, rider_wrench),
                           rider_qfrc, 1e-8, 1e-8))
                throw std::invalid_argument(
                    "attachment wrench does not explain generalized force");
            if (!all_close(jac_t_vec(geometry.bike_jac, bike_wrench),
                           bike_qfrc, 1e-8, 1e-8))
                throw std::invalid_argument(
                    "attachment wrench does not explain generalized force");
            if (!geometry.observable)
                throw std::invalid_argument(
                    "in-plane attachment force is not observable");
            check_newton_and_planarity(bike_wrench, rider_wrench);
        }
        EqualityRowMap local_rows;
        const double gap_m = attachment_gap(
            d, attachment_index(d, reactions, rows, geometry.eq_id,
                                local_rows));
        return AttachmentRaw{
            .rider_jac = geometry.rider_jac,
            .rider_qfrc = std::move(rider_qfrc),
            .bike_jac = geometry.bike_jac,
            .bike_qfrc = std::move(bike_qfrc),
            .observable = geometry.observable,
            .normal = geometry.normal,
            .kind = geometry.kind,
            .rotational = geometry.rotational,
            .half_patch_m = geometry.half_patch_m,
            .gap_m = gap_m,
            .pull_direction = geometry.pull_direction};
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional
    // args mirror the oracle's attachment_raw call order.
    AttachmentRaw attachment_raw(
        const mjModel *m, const mjData *d,
        const EqualityReactions &reactions, std::span<double> qfrc_scratch,
        int eq_id, int body_rider, int body_bike,
        std::span<const double> point, std::span<const double> normal,
        std::string kind, bool rotational, double half_patch_m,
        const std::optional<std::vector<double>> &pull_direction,
        const EqualityRowMap *rows) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        reactions.equality_qfrc_into(d, eq_id, qfrc_scratch);
        const auto [rider_point, bike_point] =
            attachment_points(m, d, eq_id, body_rider, body_bike, point,
                              rotational);
        BodyGeometry rider = body_geometry(m, d, body_rider, rider_point);
        std::vector<double> rider_qfrc =
            gather_values(qfrc_scratch, rider.columns);
        BodyGeometry bike = body_geometry(m, d, body_bike, bike_point);
        std::vector<double> bike_qfrc =
            gather_values(qfrc_scratch, bike.columns);
        if (shares_support(rider.columns, bike.columns))
            throw std::invalid_argument(
                "attachment bodies share kinematic support");
        EqualityRowMap local_rows;
        const double gap_m = attachment_gap(
            d, attachment_index(d, reactions, rows, eq_id, local_rows));
        return AttachmentRaw{
            .rider_jac = std::move(rider.jac),
            .rider_qfrc = std::move(rider_qfrc),
            .bike_jac = std::move(bike.jac),
            .bike_qfrc = std::move(bike_qfrc),
            .observable = rider.observable && bike.observable,
            .normal = copy_span(normal),
            .kind = std::move(kind),
            .rotational = rotational,
            .half_patch_m = half_patch_m,
            .gap_m = gap_m,
            .pull_direction = pull_direction};
    }

    std::vector<double> recover_wrench(LeastSquaresWorkspace &workspace,
                                       const DenseMatrix &relative_jacobian,
                                       std::span<const double> qfrc) {
        // matrix = relative_jacobian.T — solve (columns, rows). The
        // oracle screens isfinite BEFORE lstsq (unlike the workspace's
        // pass-through contract).
        if (!all_finite(relative_jacobian.values) || !all_finite(qfrc))
            throw std::invalid_argument("nonfinite wrench input");
        const DenseMatrix matrix = transpose(relative_jacobian);
        LeastSquaresResult result =
            workspace.solve(matrix.values,
                            static_cast<lapack_int>(matrix.rows),
                            static_cast<lapack_int>(matrix.columns), qfrc,
                            1);
        // rank != matrix.shape[1] — full column rank of the transposed
        // matrix, i.e. rank == relative_jacobian.rows.
        if (std::cmp_not_equal(result.rank, matrix.columns))
            throw std::invalid_argument(
                "rank-deficient attachment Jacobian");
        // matrix @ wrench == jac.T @ wrench — the verified A.T @ x
        // dgemv call.
        if (!all_close(jac_t_vec(relative_jacobian, result.solution),
                       qfrc, 1e-8, 1e-8))
            throw std::invalid_argument(
                "attachment wrench does not explain generalized force");
        return result.solution;
    }

    DenseMatrix relative_planar_jacobian(const mjModel *m, const mjData *d,
                                         int body_a, int body_b,
                                         std::span<const double> point,
                                         bool rotational) {
        // np.asarray(point, dtype=float): shape (3,) AND all-finite.
        if (point.size() != 3 || !all_finite(point))
            throw std::invalid_argument("invalid world point");
        if (!(0 < body_a && body_a < m->nbody && 0 < body_b &&
              body_b < m->nbody) ||
            body_a == body_b)
            throw std::invalid_argument(
                "relative Jacobian requires distinct physical bodies");
        const Vec3 world{point[0], point[1], point[2]};
        const DenseMatrix a = body_jacobian(m, d, body_a, world);
        const DenseMatrix b = body_jacobian(m, d, body_b, world);
        const std::size_t nv = a.columns;
        const std::size_t rows_out = rotational ? 3 : 2;
        DenseMatrix out{.rows = rows_out,
                        .columns = nv,
                        .values = std::vector<double>(rows_out * nv)};
        // (jp_a - jp_b)[[0, 2], :] plus the world-y rotational
        // difference row (jr_a - jr_b)[[1], :] when rotational.
        for (std::size_t j = 0; j < nv; ++j) {
            out.values[j] = a.values[j] - b.values[j];
            out.values[nv + j] =
                a.values[2 * nv + j] - b.values[2 * nv + j];
            if (rotational)
                out.values[2 * nv + j] =
                    a.values[4 * nv + j] - b.values[4 * nv + j];
        }
        return out;
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional
    // args mirror the oracle's attachment_wrench call order.
    std::vector<double> attachment_wrench(
        const mjModel *m, const mjData *d,
        const EqualityReactions &reactions,
        LeastSquaresWorkspace &workspace, std::span<double> qfrc_scratch,
        int eq_id, int body_a, int body_b, std::span<const double> point,
        bool rotational) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const DenseMatrix jac =
            relative_planar_jacobian(m, d, body_a, body_b, point,
                                     rotational);
        reactions.equality_qfrc_into(d, eq_id, qfrc_scratch);
        return recover_wrench(workspace, jac, qfrc_scratch);
    }

    BodyWrench body_wrench(const mjModel *m, const mjData *d,
                           LeastSquaresWorkspace &workspace,
                           std::span<const double> qfrc_eq, int body,
                           const Vec3 &point) {
        const DenseMatrix full = body_jacobian(m, d, body, point);
        std::vector<std::int64_t> cols = support_columns(full);
        if (cols.empty())
            throw std::invalid_argument(
                "attachment body carries no degrees of freedom");
        const DenseMatrix sub = gather_columns(full, cols);
        std::vector<double> target = gather_values(qfrc_eq, cols);
        // np.linalg.lstsq(jac[:, columns].T, qfrc_eq[columns]) — the
        // (k, 6) solve; minimum-norm underdetermined answers are
        // accepted when the checks below pass, exactly like the oracle.
        std::vector<double> wrench = solve_transposed(workspace, sub, target);
        if (!all_close(jac_t_vec(sub, wrench), target, 1e-8, 1e-8))
            throw std::invalid_argument(
                "attachment wrench does not explain generalized force");
        if (row_abs_max_at(full, 0, cols) == 0. ||
            row_abs_max_at(full, 2, cols) == 0.)
            throw std::invalid_argument(
                "in-plane attachment force is not observable");
        return BodyWrench{.wrench = std::move(wrench),
                          .columns = std::move(cols)};
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional
    // args mirror the oracle's attachment_sample call order.
    AttachmentSample attachment_sample(
        const mjModel *m, const mjData *d,
        const EqualityReactions &reactions,
        LeastSquaresWorkspace &workspace, std::span<double> qfrc_scratch,
        int eq_id, int body_rider, int body_bike,
        std::span<const double> point, std::span<const double> normal,
        const std::string &kind, bool rotational, double half_patch_m,
        const std::optional<std::vector<double>> &pull_direction,
        const EqualityRowMap *rows) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        reactions.equality_qfrc_into(d, eq_id, qfrc_scratch);
        const auto [rider_point, bike_point] =
            attachment_points(m, d, eq_id, body_rider, body_bike, point,
                              rotational);
        BodyWrench rider =
            body_wrench(m, d, workspace, qfrc_scratch, body_rider,
                        rider_point);
        BodyWrench bike =
            body_wrench(m, d, workspace, qfrc_scratch, body_bike,
                        bike_point);
        if (shares_support(rider.columns, bike.columns))
            throw std::invalid_argument(
                "attachment bodies share kinematic support");
        check_newton_and_planarity(bike.wrench, rider.wrench);
        EqualityRowMap local_rows;
        const double gap_m = attachment_gap(
            d, attachment_index(d, reactions, rows, eq_id, local_rows));
        return decompose_wrench(rider.wrench, normal, kind, rotational,
                                half_patch_m, gap_m, pull_direction);
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional
    // args mirror the oracle's decompose_wrench call order.
    AttachmentSample decompose_wrench(
        std::span<const double> wrench, std::span<const double> normal,
        const std::string &kind, bool rotational, double half_patch_m,
        double gap_m,
        const std::optional<std::vector<double>> &pull_direction) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        // n = np.asarray(normal, dtype=float); shape (3,) AND finite.
        if (normal.size() != 3 || !all_finite(normal))
            throw std::invalid_argument(
                "attachment sample needs a finite 3D normal");
        std::array<double, 2> normal_xz{normal[0], normal[2]};
        const double length = norm(normal_xz);
        if (length < 1e-12)
            throw std::invalid_argument(
                "attachment normal has no planar component");
        // normal_xz/length — elementwise divide.
        normal_xz[0] /= length;
        normal_xz[1] /= length;
        const std::array<double, 2> tangent_xz{normal_xz[1], -normal_xz[0]};
        // float(wrench[0]*nxz[0] + wrench[2]*nxz[1]) — the oracle's
        // multiply-then-add order (no fused contraction).
        const double w0 = at(wrench, 0);
        const double w2 = at(wrench, 2);
        const double normal_n = w0 * normal_xz[0] + w2 * normal_xz[1];
        const double tangent_n = w0 * tangent_xz[0] + w2 * tangent_xz[1];
        const double moment_nm = rotational ? at(wrench, 4) : 0.;
        double pull_n = 0.;
        if (pull_direction.has_value()) {
            const std::vector<double> &direction = *pull_direction;
            if (direction.size() != 3 || !all_finite(direction))
                throw std::invalid_argument(
                    "pull direction must be a finite 3D vector");
            std::array<double, 2> direction_xz{direction[0], direction[2]};
            const double dlength = norm(direction_xz);
            if (dlength < 1e-12)
                throw std::invalid_argument(
                    "pull direction has no planar component");
            direction_xz[0] /= dlength;
            direction_xz[1] /= dlength;
            const std::array<double, 2> force_on_bike{-w0, -w2};
            // force_on_bike @ direction_xz — a 2-element vector dot.
            if (blas::ddot(2, force_on_bike.data(), 1,
                           direction_xz.data(), 1) > 0.)
                pull_n = norm(force_on_bike);
        }
        return AttachmentSample{.kind = kind,
                                .normal_n = normal_n,
                                .tangent_n = tangent_n,
                                .moment_nm = moment_nm,
                                .gap_m = gap_m,
                                .pull_n = pull_n,
                                .half_patch_m = half_patch_m};
    }
} // namespace rider
