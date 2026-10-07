// equality_reactions.cpp — see the header for the ported contract.
// The EFC arena (efc_type/efc_id/efc_pos/efc_force) is read through
// model_access extent checks on every call; mj_mulJacTVec cannot raise
// an engine error (pure multiply over d->efc_JT/efc_J — no mju_error
// path in engine_core_constraint.c), so it is invoked directly here,
// like every other arithmetic-only MJAPI in this tree.
#include "equality_reactions.hpp"

#include "../cblas_abi.hpp"
#include "../model_access.hpp"
#include "../stepper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <ranges>
#include <stdexcept>

namespace rider {

    EqualityReactions::EqualityReactions(const mjModel *model)
        : model_(model) {
        if (model == nullptr)
            throw std::invalid_argument(
                "equality reactions need a live model");
        if (model->nv < 0)
            throw std::invalid_argument(
                "equality reactions need a nonnegative model width");
        nv_ = model->nv;
        qfrc_.assign(static_cast<std::size_t>(nv_), 0.);
    }

    namespace {

        // One flatnonzero-style pass: equality-type rows with their ids.
        // `data` is validated once per call through checked extents —
        // nefc is a runtime count, never a cached one.
        void select_rows(const mjData *data, int eq_id, RowList &rows) {
            const std::span<const int> types =
                model_access::readonly_buffer(data->efc_type, data->nefc,
                                              "efc_type");
            const std::span<const int> ids =
                model_access::readonly_buffer(data->efc_id, data->nefc,
                                              "efc_id");
            for (std::size_t i = 0; i < types.size(); ++i)
                if (types[i] == mjCNSTR_EQUALITY && ids[i] == eq_id)
                    rows.push_back(static_cast<std::int64_t>(i));
        }

        // zeros(nefc) + the equality's current multipliers at their
        // current positions — np.where(selected, efc_force, 0.0).
        // `matches` is the caller's membership predicate over efc_id.
        template <typename Match>
        void gather_multipliers(const mjData *data,
                                std::vector<double> &scratch,
                                Match matches) {
            const std::span<const int> types =
                model_access::readonly_buffer(data->efc_type, data->nefc,
                                              "efc_type");
            const std::span<const int> ids =
                model_access::readonly_buffer(data->efc_id, data->nefc,
                                              "efc_id");
            const std::span<const double> force =
                model_access::readonly_buffer(data->efc_force, data->nefc,
                                              "efc_force");
            if (scratch.size() < types.size()) scratch.resize(types.size());
            // Rows beyond the current nefc are never read — filling the
            // whole scratch is the same work and keeps the code flat.
            std::ranges::fill(scratch, 0.);
            for (std::size_t i = 0; i < types.size(); ++i)
                if (types[i] == mjCNSTR_EQUALITY && matches(ids[i]))
                    scratch[i] = force[i];
        }
    } // namespace

    EqualityRowMap EqualityReactions::equality_rows(const mjData *data) const {
        const std::span<const int> types =
            model_access::readonly_buffer(data->efc_type, data->nefc,
                                          "efc_type");
        const std::span<const int> ids =
            model_access::readonly_buffer(data->efc_id, data->nefc,
                                          "efc_id");
        // np.flatnonzero + stable argsort(ids) + split — the tagged
        // pairs keep each group's rows in mask order (ascending), which
        // a stable sort on the id key preserves.
        std::vector<std::pair<int, std::int64_t>> tagged;
        tagged.reserve(types.size());
        for (std::size_t i = 0; i < types.size(); ++i)
            if (types[i] == mjCNSTR_EQUALITY)
                tagged.emplace_back(ids[i], static_cast<std::int64_t>(i));
        std::ranges::stable_sort(tagged, {},
                                 &std::pair<int, std::int64_t>::first);
        EqualityRowMap out;
        for (std::size_t i = 0; i < tagged.size();) {
            const int eq_id = tagged[i].first;
            RowList rows;
            while (i < tagged.size() && tagged[i].first == eq_id) {
                rows.push_back(tagged[i].second);
                ++i;
            }
            out.emplace_back(eq_id, std::move(rows));
        }
        return out;
    }

    RowList EqualityReactions::rows_of(const mjData *data, int eq_id) const {
        RowList rows;
        select_rows(data, eq_id, rows);
        return rows;
    }

    std::vector<double>
    EqualityReactions::force_on_rider(const mjData *data, int eq_id) const {
        // lam[:3] — np.zeros(3) when the equality is absent; a shorter
        // slice (1 or 2 elements) when it has fewer rows than a weld.
        const RowList rows = rows_of(data, eq_id);
        if (rows.empty()) return {0., 0., 0.};
        const std::span<const double> force =
            model_access::readonly_buffer(data->efc_force, data->nefc,
                                          "efc_force");
        const std::size_t count = std::min<std::size_t>(3, rows.size());
        std::vector<double> out(count);
        for (std::size_t i = 0; i < count; ++i)
            out[i] = force[static_cast<std::size_t>(rows[i])];
        return out;
    }

    double
    EqualityReactions::translation_residual(const mjData *data,
                                            int eq_id) const {
        // np.linalg.norm(efc_pos[rows][:3]) — the gathered 1-3 element
        // slice's x.dot(x) runs through Accelerate's ddot on both sides.
        const RowList rows = rows_of(data, eq_id);
        if (rows.empty()) return 0.;
        const std::span<const double> pos =
            model_access::readonly_buffer(data->efc_pos, data->nefc,
                                          "efc_pos");
        const std::size_t count = std::min<std::size_t>(3, rows.size());
        std::array<double, 3> gathered{};
        for (std::size_t i = 0; i < count; ++i)
            gathered[i] = pos[static_cast<std::size_t>(rows[i])];
        return std::sqrt(blas::ddot(static_cast<int>(count),
                                    gathered.data(), 1, gathered.data(),
                                    1));
    }

    void EqualityReactions::equality_qfrc_into(const mjData *data,
                                               int eq_id,
                                               std::span<double> out) const {
        if (out.size() != static_cast<std::size_t>(nv_))
            throw std::invalid_argument(
                "equality_qfrc destination must be nv elements");
        // np.zeros(model.nv) FIRST — mj_mulJacTVec early-returns on an
        // empty arena without touching res.
        std::ranges::fill(out, 0.);
        gather_multipliers(data, multipliers_,
                           [eq_id](int id) { return id == eq_id; });
        mj_mulJacTVec(model_, data, out.data(), multipliers_.data());
    }

    double
    EqualityReactions::equalities_qfrc_at(const mjData *data,
                                          std::span<const int> eq_ids,
                                          int dof) const {
        // delivered_crank_torque_nm's early return: an empty arena is a
        // zero read, BEFORE the dof index is looked at.
        if (model_access::readonly_buffer(data->efc_type, data->nefc,
                                          "efc_type")
                .empty())
            return 0.;
        gather_multipliers(data, multipliers_, [eq_ids](int id) {
            return std::ranges::find(eq_ids, id) != eq_ids.end();
        });
        std::ranges::fill(qfrc_, 0.);
        mj_mulJacTVec(model_, data, qfrc_.data(), multipliers_.data());
        // Python's qfrc[crank_dof] indexing — negatives wrap; out of
        // range is numpy's IndexError (std::out_of_range maps there).
        std::int64_t index = dof;
        if (index < 0) index += nv_;
        if (index < 0 || index >= nv_)
            throw std::out_of_range("equalities_qfrc_at: dof out of range");
        return qfrc_[static_cast<std::size_t>(index)];
    }
} // namespace rider

// This member and rider_equalities() live in this TU (not stepper.cpp)
// because the port leaves stepper.cpp untouched — the definitions are
// ordinary out-of-line members and the Stepper's own (m_, d_) stay the
// only simulation owner.
rider::EqualityReactions &Stepper::rider_equalities() const {
    require_healthy();
    if (equality_scratch_ == nullptr)
        equality_scratch_ =
            std::make_unique<rider::EqualityReactions>(m_);
    return *equality_scratch_;
}

std::vector<double> Stepper::rider_equality_qfrc(int eq_id) const {
    const rider::EqualityReactions &reactions = rider_equalities();
    std::vector<double> out(static_cast<std::size_t>(m_->nv));
    reactions.equality_qfrc_into(d_, eq_id, out);
    return out;
}
