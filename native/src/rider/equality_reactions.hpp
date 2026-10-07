// equality_reactions.hpp — solved equality-constraint reactions read off
// the CURRENT mjData constraint arena.
//
// Ports weld_pedals.py / attachment_wrench.py: rows are re-derived from
// data.efc_type/data.efc_id on every evaluation (membership is never
// cached — contacts, limits and friction rows shift the arena layout
// every forward), grouped by efc_id with a stable ascending order, and
// only mjCNSTR_EQUALITY rows are selected. A missing equality is a "no
// current measurement" read: zero force, zero residual, zero
// generalized force — never invented.
//
// Ownership: the reader borrows the model/data pair; the Stepper's owned
// (m_, d_) is the only owner. Work storage (the multiplier scratch and
// the nv qfrc buffer) is owned HERE, sized at construction and grown
// only when the arena demands — nothing escapes as a view into engine
// memory.
#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace rider {

    // Ascending EFC arena positions of one equality's constraint rows —
    // the value side of weld_pedals.equality_rows' dict.
    using RowList = std::vector<std::int64_t>;

    // One equality_rows(data) pass: (eq_id, ascending rows) pairs in
    // ascending eq_id order — np.argsort(ids, kind='stable') + split.
    using EqualityRowMap = std::vector<std::pair<int, RowList>>;

    class EqualityReactions {
    public:
        // model is borrowed (the Stepper owns it); its nv is validated
        // once and bounds every method's output.
        explicit EqualityReactions(const mjModel *model);

        // weld_pedals.equality_rows: one fresh pass over the arena,
        // equality-type rows only, stable ascending groups by efc_id.
        [[nodiscard]] EqualityRowMap equality_rows(const mjData *data) const;

        // flatnonzero(efc_type == EQUALITY & efc_id == eq_id) — the
        // per-equality mask scan _eq_rows falls back to.
        [[nodiscard]] RowList rows_of(const mjData *data, int eq_id) const;

        // PedalWelds/SaddleWeld/GripConnect.force_on_rider_n:
        // efc_force[rows][:3] — np.zeros(3) when the equality is absent
        // and a shorter-than-3 vector when the equality has fewer rows
        // (a joint equality's single row returns one element), exactly
        // like the lam[:3] slice in Python.
        [[nodiscard]] std::vector<double>
        force_on_rider(const mjData *data, int eq_id) const;

        // *_pedals' translation_residual_m: np.linalg.norm(efc_pos
        // [rows][:3]) — 0.0 when the equality is absent.
        [[nodiscard]] double
        translation_residual(const mjData *data, int eq_id) const;

        // attachment_wrench.equality_qfrc: zeros(nv) + J^T * vec where
        // vec carries only this equality's CURRENT multipliers at their
        // CURRENT arena positions — via mj_mulJacTVec on the caller's
        // (borrowed) data. `out` is filled completely (it is zeroed
        // before the engine call, so an empty arena leaves zeros —
        // mj_mulJacTVec itself early-returns without writing).
        void equality_qfrc_into(const mjData *data, int eq_id,
                                std::span<double> out) const;

        // PedalWelds.delivered_crank_torque_nm generalized: qfrc[dof] of
        // the summed selected equality multipliers. Returns 0.0 on an
        // empty arena (the Python early-return). `dof` follows Python
        // indexing — negatives wrap; an out-of-range dof throws
        // std::out_of_range, surfacing as IndexError like numpy's.
        [[nodiscard]] double
        equalities_qfrc_at(const mjData *data, std::span<const int> eq_ids,
                           int dof) const;

        [[nodiscard]] mjtSize nv() const { return nv_; }

    private:
        const mjModel *model_;
        mjtSize nv_ = 0;
        // Per-call scratch — sized to the CURRENT nefc as needed (grow
        // only), re-derived contents on every call.
        mutable std::vector<double> multipliers_;
        mutable std::vector<double> qfrc_;
    };
} // namespace rider
