#include "transmission.hpp"
#include "../engaged.hpp"
#include "../engine_call.hpp"
#include "../model_topology.hpp"
#include "../numeric_sum.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

namespace drivetrain {
    namespace {
        struct CloneModelContext {
            const mjModel *source;
            mjModel *result;
        };
        void clone_model_operation(void *raw) noexcept {
            auto *context = static_cast<CloneModelContext *>(raw);
            context->result = mj_copyModel(nullptr, context->source);
        }
        struct CopyModelContext {
            mjModel *destination;
            const mjModel *source;
        };
        void copy_model_operation(void *raw) noexcept {
            const auto *context = static_cast<CopyModelContext *>(raw);
            mj_copyModel(context->destination, context->source);
        }
        // Fatal-capable whole-model copy through the E1 boundary.
        mjModel *clone_model(const mjModel *model) {
            CloneModelContext context{.source = model, .result = nullptr};
            engine::ErrorBuffer error;
            if (!engine::invoke(&clone_model_operation, &context, error))
                engine::throw_failure(error);
            if (!context.result)
                throw std::runtime_error("mj_copyModel");
            return context.result;
        }
    }

    Transmission::Transmission(mjModel *model, GearingConfig gear, bool geometric,
                               const char *tendon, const char *driver, const char *driven,
                               const char *front)
        : model_(model), gear_(gear), geometric_(geometric),
          tendon_(resolve(model, mjOBJ_TENDON, tendon)), constant_(engine::make_data(model)),
          endpoint_(geometric ? engine::make_data(model) : nullptr),
          scratch_model_(clone_model(model)),
          scratch_data_(engine::make_data(scratch_model_.get())),
          geometry_(model->nv),
          prepared_storage_{
              .phi = 0., .time = 0.,
              .jacobian = std::vector<double>(static_cast<std::size_t>(geometric ? model->nv : 0)),
              .qpos = std::vector<double>(static_cast<std::size_t>(geometric ? model->nv : 0))
          },
          force_(static_cast<std::size_t>(model->nv)),
          multipliers_(static_cast<std::size_t>(model->njmax > 0 ? model->njmax : 0)),
          displacement_(force_.size()) {
        drivetrain::validate(gear_);
        if (!constant_ || (geometric && !endpoint_) || !scratch_data_)
            throw std::runtime_error("mj_makeData");
        const int dj = resolve(model, mjOBJ_JOINT, driver),
                wj = resolve(model, mjOBJ_JOINT, driven);
        if (geometric) {
            topology::joint(model, dj, driver, mjJNT_HINGE, {0., 1., 0.});
            topology::joint(model, wj, driven, mjJNT_HINGE, {0., 1., 0.});
        } else {
            topology::scalar_joint(model, dj, driver);
            topology::scalar_joint(model, wj, driven);
        }
        driver_qpos_ =
                buffer(model->jnt_qposadr, model->njnt)[static_cast<std::size_t>(dj)];
        driver_dof_ = buffer(model->jnt_dofadr, model->njnt)[static_cast<std::size_t>(dj)];
        driven_qpos_ =
                buffer(model->jnt_qposadr, model->njnt)[static_cast<std::size_t>(wj)];
        driven_dof_ = buffer(model->jnt_dofadr, model->njnt)[static_cast<std::size_t>(wj)];
        state_.ratio = static_cast<double>(gear.front_teeth) / gear.rear_teeth;
        state_.rear_teeth = gear.rear_teeth;
        if (geometric) {
            front_ = resolve(model, mjOBJ_BODY, front);
            rear_ = resolve(model, mjOBJ_BODY, "rear_wheel");
            frame_ = resolve(model, mjOBJ_BODY, "frame");
            coefficients_.assign(force_.size(), -1);
        }
        const int start = buffer(model->tendon_adr,
                                 model->ntendon)[static_cast<std::size_t>(tendon_)],
                end = start + buffer(model->tendon_num,
                                     model->ntendon)[static_cast<std::size_t>(tendon_)];
        for (int i = start; i < end; ++i) {
            const auto at = static_cast<std::size_t>(i);
            if (buffer(model->wrap_type, model->nwrap)[at] != mjWRAP_JOINT) {
                if (geometric)
                    throw std::invalid_argument(
                        "geometric reduction must use one fixed scalar-joint tendon");
                continue;
            }
            const int joint = buffer(model->wrap_objid, model->nwrap)[at];
            if (geometric) {
                const auto dof = static_cast<std::size_t>(buffer(
                    model->jnt_dofadr, model->njnt)[static_cast<std::size_t>(joint)]);
                if (coefficients_.at(dof) >= 0)
                    throw std::invalid_argument("duplicate geometric tendon coordinate");
                coefficients_.at(dof) = i;
            } else if (joint == dj)
                coefficients_.push_back(i);
        }
        if (geometric) {
            if (model->nq != model->nv ||
                std::ranges::find(coefficients_, -1) != coefficients_.end())
                throw std::invalid_argument("geometric tendon must contain every scalar "
                    "model coordinate exactly once");
        } else if (coefficients_.size() != 1)
            throw std::invalid_argument("ideal freehub needs one driver joint coefficient");
        // The first commit's state_⇄update.state swap hands the live
        // snapshot's storage to the staged bank slot. Reserving here keeps
        // that hand-off capacity-sized so the next marked
        // make_update_into() assigns coefficients into capacity instead of
        // allocating on the cold slot (RTSan flagged the swap-poisoned
        // slot on the first warm tick).
        state_.coefficients.reserve(coefficients_.size());
    }

    double Transmission::relative(const mjData *d) const {
        return relative(d, state_.ratio);
    }

    double Transmission::relative(const mjData *d, double ratio) const {
        const auto q = buffer(d->qpos, model_->nq);
        return ratio * q[static_cast<std::size_t>(driver_qpos_)] -
               q[static_cast<std::size_t>(driven_qpos_)];
    }

    double Transmission::relative_rate(const mjData *d) const {
        const auto v = buffer(d->qvel, model_->nv);
        return state_.ratio * v[static_cast<std::size_t>(driver_dof_)] -
               v[static_cast<std::size_t>(driven_dof_)];
    }

    double Transmission::geometry(mjData *d) {
        mj_kinematics(model_, d);
        mj_comPos(model_, d);
        return geometry_.evaluate(model_, d, gear_, front_, rear_, frame_);
    }

    void Transmission::sync_scratch_data(const mjData *data) const {
        std::ranges::copy(buffer(data->qpos, model_->nq), scratch_data_->qpos);
        std::ranges::copy(buffer(data->qvel, model_->nv), scratch_data_->qvel);
        scratch_data_->time = data->time;
    }

    // Candidate geometry still refreshes kinematics on the caller's data:
    // that live-data refresh is observable behavior — the writer's angle
    // bookkeeping reads xmat after prepare() (Python relies on the same
    // side effect). Only the MODEL is write-protected during staging;
    // the gear parameter carries the candidate.
    double Transmission::candidate_geometry(mjData *data,
                                            const GearingConfig &gear) {
        mj_kinematics(model_, data);
        mj_comPos(model_, data);
        return geometry_.evaluate(model_, data, gear, front_, rear_, frame_);
    }

    void Transmission::refresh_scratch_model() {
        CopyModelContext context{.destination = scratch_model_.get(), .source = model_};
        engine::ErrorBuffer error;
        if (!engine::invoke(&copy_model_operation, &context, error))
            engine::throw_failure(error);
    }

    // Values only — every write goes into `update`; scratch/live models stay
    // untouched until evaluate_candidate()/commit().
    void Transmission::linearize_candidate(const mjData *data,
                                           TransmissionUpdate &update, double phi) {
        const auto q = buffer(data->qpos, model_->nq);
        const double upper = validation::derived(
            engaged(update.state.boundary) - phi + dot(geometry_.jacobian, q),
            "transmission.upper_bound");
        update.coefficients.assign(geometry_.jacobian.begin(), geometry_.jacobian.end());
        update.state.coefficients = update.coefficients;
        update.range[1] = upper;
        update.state.range = update.range;
        update.prepared.phi = phi;
        update.prepared.time = data->time;
        update.prepared.jacobian = geometry_.jacobian;
        update.prepared.qpos.assign(q.begin(), q.end());
        update.prepared_valid = true;
        update.state.prepared = update.prepared;
    }

    // Reseeds caller-owned candidate storage from the live state. Every
    // assignment is a copy-assign into the update's existing vectors, lanes
    // and optionals, so a bank-resident update reaches steady state after
    // one warmup — libc++/MSVC/libstdc++ assign-into-capacity never
    // reallocates when the source fits.
    // state_.prepared is disengaged by invariant, so the whole-state
    // copy-assign would DESTROY the bank slot's engaged prepared vectors
    // (two frees) only for the re-engagement below to rebuild them (two
    // news) — the same churn commit() avoids by swapping the slot back.
    // Lift the engaged slot into a keep-alive optional across the assign
    // and swap it back in, preserving the bank's storage verbatim.
    void Transmission::make_update_into(TransmissionUpdate &update) const {
        update.gear = gear_;
        std::optional<PreparedTransmission> keep;
        keep.swap(update.state.prepared);
        update.state = state_;
        update.coefficients.clear();
        const auto prm = buffer(model_->wrap_prm, model_->nwrap);
        for (int const index: coefficients_)
            update.coefficients.push_back(prm[static_cast<std::size_t>(index)]);
        update.state.coefficients = update.coefficients;
        const auto range = buffer(model_->tendon_range, 2 * model_->ntendon)
                .subspan(2 * static_cast<std::size_t>(tendon_), 2);
        update.range = {range[0], range[1]};
        update.state.range = update.range;
        if (prepared_valid_) {
            PreparedTransmission &slot = keep ? *keep : keep.emplace();
            slot = prepared_storage_;
            update.state.prepared.swap(keep);
            update.prepared = prepared_storage_;
            update.prepared_valid = true;
        } else {
            update.prepared.phi = update.prepared.time = 0.;
            update.prepared.jacobian.clear();
            update.prepared.qpos.clear();
            // A bank slot can arrive here holding stale prepared storage
            // only when prepared_valid_ dropped (a restore() that detached
            // the prepared candidate) — an out-of-claim event. The lifted
            // storage dies with `keep`; the slot stays disengaged like the
            // non-geometric banks, which never engaged it at all.
            update.state.prepared.reset();
            update.prepared_valid = false;
        }
    }

    TransmissionUpdate Transmission::make_update_from_current() const {
        TransmissionUpdate update;
        make_update_into(update);
        return update;
    }

    int Transmission::validated_rear_teeth(int front_teeth, double ratio) const {
        const double teeth = std::nearbyint(front_teeth / ratio);
        if (teeth < 3. || teeth > std::numeric_limits<int>::max() ||
            std::abs(ratio - front_teeth / teeth) >
            1e-12 * std::abs(front_teeth / teeth))
            throw std::invalid_argument(
                "geometric ratio must identify an integer rear sprocket");
        return static_cast<int>(teeth);
    }

    void Transmission::evaluate_candidate(mjData *data, TransmissionUpdate &update) {
        // Candidates validate every field the stage produced, but not the
        // ideal coefficient/ratio coupling: reset/prepare only mirror the
        // live wrap row, which an external model mutation may have moved
        // off the logical ratio — reporting it is truthful, not a contract
        // violation. stage_ratio rebuilds the pair consistently by
        // construction, and restore() still enforces the coupling on the
        // caller-supplied snapshot via validate() before staging.
        validate_state(update.state);
        const auto prm = buffer(model_->wrap_prm, model_->nwrap);
        bool changed = false;
        for (std::size_t i = 0; i < coefficients_.size(); ++i)
            if (prm[static_cast<std::size_t>(coefficients_[i])] != update.coefficients[i])
                changed = true;
        if (!changed)
            return;
        // Refresh the owned scratch from live, then rehearse the commit in
        // the same order it will run below (coefficients, set_const, range):
        // a fatal lands on the scratch model, never the committed one.
        refresh_scratch_model();
        if (data != nullptr)
            sync_scratch_data(data);
        const auto candidate = buffer(scratch_model_->wrap_prm, scratch_model_->nwrap);
        for (std::size_t i = 0; i < coefficients_.size(); ++i)
            candidate[static_cast<std::size_t>(coefficients_[i])] = update.coefficients[i];
        engine::set_const(scratch_model_.get(), scratch_data_.get());
        std::ranges::copy(update.range,
                          buffer(scratch_model_->tendon_range,
                                 2 * scratch_model_->ntendon)
                          .subspan(2 * static_cast<std::size_t>(tendon_), 2).begin());
    }

    void Transmission::stage_reset_into(mjData *data,
                                        TransmissionUpdate &update) {
        make_update_into(update);
        if (geometric_) {
            const double phi = candidate_geometry(data, update.gear);
            update.state.boundary = phi;
            update.state.diagnostics.clear();
            update.state.shift_pending = false;
            update.state.shift_parameter_work_j = update.state.shift_constraint_work_j =
                    update.state.last_tension_n = 0.;
            linearize_candidate(data, update, phi);
        } else {
            update.state.boundary = relative(data);
            update.range[1] = *update.state.boundary;
            update.state.range = update.range;
        }
        evaluate_candidate(data, update);
    }

    TransmissionUpdate Transmission::stage_reset(mjData *data) {
        TransmissionUpdate update;
        stage_reset_into(data, update);
        return update;
    }

    void Transmission::stage_prepare_into(mjData *data,
                                          TransmissionUpdate &update) {
        make_update_into(update);
        if (geometric_) {
            const double phi = candidate_geometry(data, update.gear);
            update.state.boundary =
                    update.state.boundary ? std::min(*update.state.boundary, phi) : phi;
            linearize_candidate(data, update, phi);
        } else {
            const double phi = relative(data);
            update.state.boundary =
                    update.state.boundary ? std::min(*update.state.boundary, phi) : phi;
            update.range[1] = *update.state.boundary;
            update.state.range = update.range;
        }
        evaluate_candidate(data, update);
    }

    TransmissionUpdate Transmission::stage_prepare(mjData *data) {
        TransmissionUpdate update;
        stage_prepare_into(data, update);
        return update;
    }

    TransmissionUpdate Transmission::stage_ratio(mjData *data, double ratio) {
        TransmissionUpdate update;
        make_update_into(update);
        stage_ratio_into(data, ratio, update);
        return update;
    }

    TransmissionUpdate Transmission::stage_ratio(mjData *data, double ratio,
                                                 TransmissionUpdate base) {
        stage_ratio_into(data, ratio, base);
        return base;
    }

    void Transmission::stage_ratio_into(mjData *data, double ratio,
                                        TransmissionUpdate &update) {
        positive(ratio, "gear ratio");
        if (ratio == update.state.ratio)
            return;
        // The seed candidate's gear/state supply every pre-shift read, so a
        // staged prepare composes under the ratio change exactly as the
        // sequential live order did.
        const double from = update.state.ratio;
        const GearingConfig base_gear = update.gear;
        update.state.ratio = ratio;
        if (!geometric_) {
            const double previous = relative(data, from);
            update.coefficients[0] = ratio;
            update.state.coefficients = update.coefficients;
            const double current = relative(data, ratio);
            update.state.boundary = update.state.boundary
                                        ? *update.state.boundary + current - previous
                                        : current;
            update.range[1] = *update.state.boundary;
            update.state.range = update.range;
            evaluate_candidate(data, update);
            return;
        }
        update.gear.rear_teeth = validated_rear_teeth(base_gear.front_teeth, ratio);
        update.state.rear_teeth = update.gear.rear_teeth;
        const double old_phi = candidate_geometry(data, base_gear);
        const double old_gap =
                update.state.boundary ? *update.state.boundary - old_phi : 0.;
        const double phi = candidate_geometry(data, update.gear);
        update.state.boundary = phi + old_gap;
        const double new_gap = *update.state.boundary - phi;
        update.state.shift_parameter_work_j +=
                update.state.last_tension_n * (new_gap - old_gap);
        update.state.shift_pending = true;
        linearize_candidate(data, update, phi);
        evaluate_candidate(data, update);
    }

    void Transmission::commit(TransmissionUpdate &update) {
        // Phase 1 — guarded live-model writes. An engine failure here
        // propagates before any logical publication; the owning Stepper
        // treats it as fatal.
        const auto prm = buffer(model_->wrap_prm, model_->nwrap);
        bool changed = false;
        for (std::size_t i = 0; i < coefficients_.size(); ++i) {
            auto &c = prm[static_cast<std::size_t>(coefficients_[i])];
            if (c != update.coefficients[i]) {
                c = update.coefficients[i];
                changed = true;
            }
        }
        if (changed)
            engine::set_const(model_, constant_.get());
        std::ranges::copy(update.range,
                          buffer(model_->tendon_range, 2 * model_->ntendon)
                          .subspan(2 * static_cast<std::size_t>(tendon_), 2).begin());
        // Phase 2 — memory-only logical publication. state_.prepared stays
        // disengaged; the candidate lands in the construction-owned storage
        // element-by-element so vector identity (prepared_generations) is
        // preserved — resize only repairs a width drift, never reallocates.
        // The state swap (not a move) hands the old live snapshot back into
        // the caller's storage, and the second swap returns the staged
        // prepared mirror into the update's slot — bank-resident candidates
        // keep every vector/lane allocation for the next stage.
        gear_ = update.gear;
        std::swap(state_, update.state);
        update.state.prepared.swap(state_.prepared);
        prepared_valid_ = update.prepared_valid;
        if (update.prepared_valid) {
            if (prepared_storage_.jacobian.size() != update.prepared.jacobian.size())
                prepared_storage_.jacobian.resize(update.prepared.jacobian.size());
            if (prepared_storage_.qpos.size() != update.prepared.qpos.size())
                prepared_storage_.qpos.resize(update.prepared.qpos.size());
            prepared_storage_.phi = update.prepared.phi;
            prepared_storage_.time = update.prepared.time;
            std::ranges::copy(update.prepared.jacobian, prepared_storage_.jacobian.begin());
            std::ranges::copy(update.prepared.qpos, prepared_storage_.qpos.begin());
        }
    }

    void Transmission::reset(mjData *d) {
        auto update = stage_reset(d);
        commit(update);
    }

    void Transmission::prepare(mjData *d) {
        auto update = stage_prepare(d);
        commit(update);
    }

    void Transmission::set_ratio(mjData *d, double ratio) {
        auto update = stage_ratio(d, ratio);
        commit(update);
    }

    SolvedTransmission Transmission::stage_solved(mjData *d) {
        SolvedTransmission solved;
        stage_solved_into(d, solved);
        return solved;
    }

    void Transmission::stage_solved_into(mjData *d,
                                         SolvedTransmission &solved) {
        // The candidate detaches the live snapshot up front; everything the
        // solve used to publish now lands on `solved.state`, so a throw in
        // any step below leaves state_ byte-identical. Scratch writes
        // (force_, multipliers_, endpoint_, displacement_, geometry_) are
        // not logical state and may partially complete on a throw — the
        // next staging run overwrites them wholesale. The copy-assign seeds
        // caller-owned storage — vectors and telemetry lanes reuse their
        // capacity, so a bank-resident solve allocates nothing per settle.
        solved.state = state_;
        std::ranges::fill(force_, 0.);
        // multipliers_ was sized to model_->njmax at construction; models
        // with an unlimited constraint arena (njmax <= 0) can exceed that
        // hint — assign() grows once to the observed extent, then every
        // warm solve reuses the established capacity. Steady state is
        // allocation-free; the size stays nefc for the loop below.
        multipliers_.assign(static_cast<std::size_t>(d->nefc), 0.);
        const auto type = buffer(d->efc_type, d->nefc), ids = buffer(d->efc_id, d->nefc);
        const auto ef = buffer(d->efc_force, d->nefc);
        // np.sum(efc_force[:nefc][selected]) — the masked copy keeps row
        // order, then numpy's pairwise block order reduces it; a naive
        // fold regroups the tail differently at n in [2, 8).
        tension_terms_.clear();
        bool selected = false;
        for (std::size_t i = 0; i < multipliers_.size(); ++i)
            if (type[i] == mjCNSTR_LIMIT_TENDON && ids[i] == tendon_) {
                multipliers_[i] = ef[i];
                tension_terms_.push_back(ef[i]);
                selected = true;
            }
        const double tension = numeric::numpy_pairwise_sum(tension_terms_);
        if (selected)
            mj_mulJacTVec(model_, d, force_.data(), multipliers_.data());
        solved.force = force_;
        if (!geometric_)
            return;
        if (!prepared_valid_)
            throw std::runtime_error("prepare the geometric transmission before solving");
        const auto &p = prepared_storage_;
        std::ranges::copy(buffer(d->qpos, model_->nq), endpoint_->qpos);
        const double endpoint = geometry(endpoint_.get());
        const auto q = buffer(d->qpos, model_->nq);
        for (std::size_t i = 0; i < displacement_.size(); ++i)
            displacement_[i] = q[i] - p.qpos[i];
        const double defect = endpoint - p.phi - dot(p.jacobian, displacement_),
                work = dot(force_, displacement_);
        // Every derived value is in hand: the candidate publication below
        // is detached until commit(), so the work counter, the telemetry
        // map, last tension and the consumed pending flag land together.
        if (state_.shift_pending)
            solved.state.shift_constraint_work_j += work;
        double error = 0.;
        for (std::size_t i = 0; i < force_.size(); ++i)
            error = std::max(
                error, std::abs(force_[i] - (-std::max(0., tension) * p.jacobian[i])));
        // Fixed-lane telemetry replaces the per-solve map build: clear()
        // drops the seeded lanes wholesale (the old map assignment's
        // replace semantics), then the eleven declared fields write in
        // place — no node allocations at any point.
        DriveTelemetry &diag = solved.state.diagnostics;
        diag.clear();
        diag.set(TelemetryField::transmission_phi_m, p.phi);
        diag.set(TelemetryField::transmission_boundary_m,
                 engaged(state_.boundary));
        diag.set(TelemetryField::transmission_gap_m,
                 engaged(state_.boundary) - p.phi);
        diag.set(TelemetryField::transmission_tension_n, tension);
        diag.set(TelemetryField::transmission_constraint_defect_m, defect);
        diag.set(TelemetryField::transmission_interval_work_j, work);
        diag.set(TelemetryField::transmission_reaction_error_n, error);
        diag.set(TelemetryField::shift_parameter_work_j,
                 state_.shift_parameter_work_j);
        diag.set(TelemetryField::shift_interval_constraint_work_j,
                 state_.shift_pending ? work : 0.);
        diag.set(TelemetryField::shift_constraint_work_cumulative_j,
                 solved.state.shift_constraint_work_j);
        diag.set_label(TelemetryField::transmission_reference_status,
                       /*experimental_geometric_reduction*/ 0);
        solved.state.last_tension_n = tension;
        solved.state.shift_pending = false;
    }

    void Transmission::commit(SolvedTransmission &solved) noexcept {
        // Every member of TransmissionSnapshot is noexcept-movable, so the
        // whole candidate swaps in without a single allocation — this runs
        // inside the settlement's noexcept commit. std::swap (not move)
        // additionally hands the old live snapshot back into the staged
        // slot, preserving the bank's lanes/vector capacity for the next
        // settle.
        static_assert(std::is_nothrow_move_assignable_v<TransmissionSnapshot> &&
                      std::is_nothrow_move_constructible_v<TransmissionSnapshot>);
        std::swap(state_, solved.state);
    }

    std::span<const double> Transmission::solved(mjData *d) {
        auto update = stage_solved(d);
        const std::span<const double> force = update.force;
        commit(update);
        return force;
    }

    void Transmission::state_into(TransmissionSnapshot &s) const {
        // Same bank-storage preservation as make_update_into(): the live
        // state's prepared is disengaged, so a whole-snapshot copy-assign
        // would destroy an engaged destination slot that the re-engagement
        // below immediately rebuilds — lift it out, assign, swap back.
        std::optional<PreparedTransmission> keep;
        keep.swap(s.prepared);
        s = state_;
        if (prepared_valid_) {
            PreparedTransmission &slot = keep ? *keep : keep.emplace();
            slot = prepared_storage_;
            s.prepared.swap(keep);
        } else
            s.prepared.reset();
        const auto range = buffer(model_->tendon_range, 2 * model_->ntendon)
                .subspan(2 * static_cast<std::size_t>(tendon_), 2);
        s.range = {range[0], range[1]};
        s.coefficients.clear();
        for (int const index: coefficients_)
            s.coefficients.push_back(
                buffer(model_->wrap_prm, model_->nwrap)[static_cast<std::size_t>(index)]);
    }

    TransmissionSnapshot Transmission::state() const {
        TransmissionSnapshot s;
        state_into(s);
        return s;
    }

    void Transmission::validate(const TransmissionSnapshot &s) const {
        validate_state(s);
        // An ideal transmission's single wrap coefficient IS the ratio.
        if (!geometric_ && s.coefficients[0] != s.ratio)
            throw std::invalid_argument("transmission coefficient/ratio mismatch");
    }

    void Transmission::validate_state(const TransmissionSnapshot &s) const {
        positive(s.ratio, "transmission ratio");
        if (s.rear_teeth < 3)
            throw std::invalid_argument("transmission rear teeth");
        finite_optional(s.boundary, "transmission boundary");
        finite(s.shift_parameter_work_j, "shift parameter work");
        finite(s.shift_constraint_work_j, "shift constraint work");
        finite(s.last_tension_n, "last tension");
        if (s.coefficients.size() != coefficients_.size())
            throw std::invalid_argument("transmission coefficient width");
        for (double const c: s.coefficients)
            finite(c, "transmission coefficient");
        for (double const r: s.range)
            finite(r, "transmission range");
        // Only a geometric shift sets shift_pending (linearize runs in the
        // same call), and only geometric transmissions own a prepared slot.
        if (s.shift_pending && (!geometric_ || !s.prepared))
            throw std::invalid_argument("transmission shift_pending");
        if (s.prepared) {
            const auto &p = *s.prepared;
            if (!geometric_ || !s.boundary || p.qpos.size() != force_.size() ||
                p.jacobian.size() != force_.size())
                throw std::invalid_argument("prepared transmission width");
            finite(p.phi, "prepared phi");
            finite(p.time, "prepared time");
            for (double const x: p.qpos)
                finite(x, "prepared qpos");
            for (double const x: p.jacobian)
                finite(x, "prepared Jacobian");
        }
    }

    void Transmission::restore(TransmissionSnapshot s) {
        validate(s);
        // A restore is a staged candidate like any other: validate up front,
        // let the scratch mj_setConst guard the coefficients, then commit.
        TransmissionUpdate update;
        update.gear = gear_;
        update.gear.rear_teeth = s.rear_teeth;
        update.state = s;
        update.coefficients = s.coefficients;
        update.range = s.range;
        update.prepared_valid = s.prepared.has_value();
        if (s.prepared)
            update.prepared = *s.prepared;
        evaluate_candidate(nullptr, update);
        commit(update);
    }
} // namespace drivetrain
