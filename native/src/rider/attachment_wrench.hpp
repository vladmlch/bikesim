// attachment_wrench.hpp — the typed, Python-free port of
// bike_sim/sim/ride/attachment_wrench.py (T2b).
//
// Every function borrows the (mjModel, mjData) pair the owning Stepper
// holds — no model/data ownership, no second simulation, no Python
// objects anywhere in this TU. Prepared geometry owns the Jacobians and
// dof columns captured at the interval-start pose; raw measurements own
// the qfrc slices gathered from the CURRENT solved arena; the row map
// used for the gap is either passed through (one fresh equality_rows
// pass, like the oracle's rows= kwarg) or derived on the spot.
//
// Numerical contract — see attachment_wrench.cpp for the per-site
// evidence: np.flatnonzero/np.max column support, np.intersect1d support
// overlap, dgemv/ddot through Accelerate's legacy CBLAS (the same entry
// points numpy's matvec/matmul/norm dispatch to), the Accelerate ILP64
// DGELSD behind np.linalg.lstsq (rcond = 1e-12), and numpy 2.5's
// elementwise isclose semantics:
//   isclose(a, b) = (|a-b| <= atol + rtol*|b| AND isfinite(b)) OR a==b
// evaluated on the SECOND argument as the relative reference, so
// allclose(a, b) is asymmetric exactly like np.allclose.
#pragma once

#include <mujoco/mujoco.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "contact_math.hpp"
#include "equality_reactions.hpp"
#include "least_squares.hpp"

namespace rider {

    // Owning row-major dense block — the C++ home of np.vstack /
    // jac[:, cols].copy() / (jp_a - jp_b) slices. `values` always holds
    // rows*columns entries in row-major order (empty iff rows == 0 or
    // columns == 0), so spans never dangle.
    struct DenseMatrix {
        std::size_t rows = 0;
        std::size_t columns = 0;
        std::vector<double> values;
    };

    // attachment_wrench.AttachmentGeometry — pose-dependent inputs
    // captured before integration. The two jacobians are (6, k) blocks
    // restricted to each body's own dof support; `columns` are the
    // int64 dof indices np.flatnonzero produced.
    struct AttachmentGeometry {
        int eq_id = 0;
        DenseMatrix rider_jac;
        std::vector<std::int64_t> rider_columns;
        DenseMatrix bike_jac;
        std::vector<std::int64_t> bike_columns;
        bool observable = false;
        std::vector<double> normal;
        std::string kind;
        bool rotational = false;
        double half_patch_m = 0.;
        std::optional<std::vector<double>> pull_direction;
    };

    // attachment_wrench.AttachmentRaw — every solved measurement input,
    // copied out before an endpoint forward could overwrite the arena.
    struct AttachmentRaw {
        DenseMatrix rider_jac;
        std::vector<double> rider_qfrc;
        DenseMatrix bike_jac;
        std::vector<double> bike_qfrc;
        bool observable = false;
        std::vector<double> normal;
        std::string kind;
        bool rotational = false;
        double half_patch_m = 0.;
        double gap_m = 0.;
        std::optional<std::vector<double>> pull_direction;
    };

    // attachment_budget.AttachmentSample — the projected budget sample:
    // (kind, normal_n, tangent_n, moment_nm, gap_m, pull_n,
    // half_patch_m).
    struct AttachmentSample {
        std::string kind;
        double normal_n = 0.;
        double tangent_n = 0.;
        double moment_nm = 0.;
        double gap_m = 0.;
        double pull_n = 0.;
        double half_patch_m = 0.;
    };

    // body_wrench()'s (wrench, columns) pair.
    struct BodyWrench {
        std::vector<double> wrench;
        std::vector<std::int64_t> columns;
    };

    // ---- the oracle's module-level functions ------------------------

    // _attachment_points: per-body world anchors when the equality is a
    // nonrotational CONNECT (each body's own compiled eq_data anchor is
    // the force point — see the oracle's F x gap comment), the shared
    // `point` otherwise. `eq_id` indexes model.eq_type like numpy —
    // negatives wrap, out-of-range throws std::out_of_range (IndexError).
    [[nodiscard]] std::pair<Vec3, Vec3>
    attachment_points(const mjModel *model, const mjData *data, int eq_id,
                      int body_rider, int body_bike,
                      std::span<const double> point, bool rotational);

    // _body_geometry without its Python name: the full (6, nv) spatial
    // jacobian at `point` folded into (jp; jr) rows, the nonzero-support
    // columns (np.flatnonzero(abs(jac).max(axis=0) > 0)), and whether x
    // AND z translational rows are observable on that support. Throws
    // std::invalid_argument('attachment body carries no degrees of
    // freedom') for an empty support — the oracle's exact gate.
    struct BodyGeometry {
        DenseMatrix jac;                  // (6, k) restricted block
        std::vector<std::int64_t> columns;
        bool observable = false;
    };
    [[nodiscard]] BodyGeometry
    body_geometry(const mjModel *model, const mjData *data, int body,
                  const Vec3 &point);

    // prepare_attachment_geometry — interval-start capture. Throws the
    // oracle's 'attachment bodies share kinematic support' before
    // returning; all returned storage is owned.
    [[nodiscard]] AttachmentGeometry prepare_attachment_geometry(
        const mjModel *model, const mjData *data, int eq_id,
        int body_rider, int body_bike, std::span<const double> point,
        std::span<const double> normal, std::string kind, bool rotational,
        double half_patch_m,
        const std::optional<std::vector<double>> &pull_direction);

    // attachment_raw_from_geometry — frozen jacobians/columns + current
    // qfrc/gap. `validate_wrench` controls the spatial block EXACTLY like
    // the oracle (true: reconstruct both wrenches, check explanation,
    // observability, Newton's third law, planarity; false: defer). `rows`
    // is a pass-through of the oracle's rows= kwarg — nullptr derives a
    // fresh equality_rows(data) map. `qfrc_scratch` is caller-owned nv
    // work storage (the equality_qfrc destination); `workspace` must
    // admit (ncr, 6) and (ncb, 6) solves.
    [[nodiscard]] AttachmentRaw attachment_raw_from_geometry(
        const mjData *data, const EqualityReactions &reactions,
        LeastSquaresWorkspace &workspace, std::span<double> qfrc_scratch,
        const AttachmentGeometry &geometry, bool validate_wrench,
        const EqualityRowMap *rows = nullptr);

    // attachment_raw — the eager path: capture geometry AND read the
    // current solve in one pass. Same checks/order as the oracle.
    [[nodiscard]] AttachmentRaw attachment_raw(
        const mjModel *model, const mjData *data,
        const EqualityReactions &reactions,
        std::span<double> qfrc_scratch, int eq_id, int body_rider,
        int body_bike, std::span<const double> point,
        std::span<const double> normal, std::string kind, bool rotational,
        double half_patch_m,
        const std::optional<std::vector<double>> &pull_direction,
        const EqualityRowMap *rows = nullptr);

    // recover_wrench — solve relative_jacobian^T w = qfrc; requires full
    // column rank of the TRANSPOSED matrix (rank == jac.rows) and an
    // explained target (np.allclose rtol=atol=1e-8). `workspace` must
    // admit the (jac.columns, jac.rows) problem.
    [[nodiscard]] std::vector<double>
    recover_wrench(LeastSquaresWorkspace &workspace,
                   const DenseMatrix &relative_jacobian,
                   std::span<const double> qfrc);

    // relative_planar_jacobian — (jp_a - jp_b) rows [0, 2] plus the
    // world-y rotational difference row when `rotational`: (2, nv) or
    // (3, nv). Validates the point and the distinct-physical-bodies
    // domain exactly like the oracle.
    [[nodiscard]] DenseMatrix
    relative_planar_jacobian(const mjModel *model, const mjData *data,
                             int body_a, int body_b,
                             std::span<const double> point,
                             bool rotational);

    // attachment_wrench — the solved planar wrench applied BY body_b ON
    // body_a at `point`.
    [[nodiscard]] std::vector<double>
    attachment_wrench(const mjModel *model, const mjData *data,
                      const EqualityReactions &reactions,
                      LeastSquaresWorkspace &workspace,
                      std::span<double> qfrc_scratch, int eq_id,
                      int body_a, int body_b, std::span<const double> point,
                      bool rotational);

    // body_wrench — the spatial [Fx,Fy,Fz,Mx,My,Mz] wrench the equality
    // applies to `body` at `point`, solved on the body's own dof support
    // (minimum-norm for underdetermined supports, then the explanation
    // and x/z observability checks — in the oracle's order).
    [[nodiscard]] BodyWrench
    body_wrench(const mjModel *model, const mjData *data,
                LeastSquaresWorkspace &workspace,
                std::span<const double> qfrc_eq, int body,
                const Vec3 &point);

    // attachment_sample — the full measurement: support-disjoint body
    // wrenches, Newton's third law, planarity, gap from current efc_pos,
    // then decompose_wrench.
    [[nodiscard]] AttachmentSample
    attachment_sample(const mjModel *model, const mjData *data,
                      const EqualityReactions &reactions,
                      LeastSquaresWorkspace &workspace,
                      std::span<double> qfrc_scratch, int eq_id,
                      int body_rider, int body_bike,
                      std::span<const double> point,
                      std::span<const double> normal,
                      const std::string &kind, bool rotational,
                      double half_patch_m,
                      const std::optional<std::vector<double>>
                          &pull_direction,
                      const EqualityRowMap *rows = nullptr);

    // decompose_wrench — the support-frame projection producing an
    // AttachmentSample. `wrench` is indexed like a numpy vector
    // (out-of-range -> std::out_of_range, i.e. IndexError).
    [[nodiscard]] AttachmentSample
    decompose_wrench(std::span<const double> wrench,
                     std::span<const double> normal,
                     const std::string &kind, bool rotational,
                     double half_patch_m, double gap_m,
                     const std::optional<std::vector<double>>
                         &pull_direction);
} // namespace rider
