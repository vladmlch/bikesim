#pragma once
#include "chain.hpp"
#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <variant>

namespace drivetrain {
    using DiagnosticValue = std::variant<std::monostate, double, int, bool, std::string>;
    using Diagnostics = std::map<std::string, DiagnosticValue>;

    struct DataDeleter {
        void operator()(mjData *data) const {
            if (data)
                mj_deleteData(data);
        }
    };

    struct ModelDeleter {
        void operator()(mjModel *model) const {
            if (model)
                mj_deleteModel(model);
        }
    };

    using OwnedData = std::unique_ptr<mjData, DataDeleter>;
    using OwnedModel = std::unique_ptr<mjModel, ModelDeleter>;

    struct PreparedTransmission {
        double phi{}, time{};
        std::vector<double> jacobian, qpos;
    };

    struct TransmissionSnapshot {
        double ratio{};
        int rear_teeth{};
        std::optional<double> boundary;
        std::optional<PreparedTransmission> prepared;
        Diagnostics diagnostics;
        bool shift_pending{};
        double shift_parameter_work_j{}, shift_constraint_work_j{}, last_tension_n{};
        std::array<double, 2> range{};
        std::vector<double> coefficients;
    };

    // A fully evaluated, still-unpublished transmission update. Stage calls
    // build the candidate on owned scratch model/data — the live model is
    // never touched — and commit() performs guarded model writes before the
    // no-throw logical swap. `prepared` carries the candidate linearization;
    // `state.prepared` mirrors it while staged so validate() can check the
    // shift/prepared relational invariants, and commit() detaches it again
    // before publishing (state_.prepared stays disengaged by invariant).
    struct TransmissionUpdate {
        GearingConfig gear;
        TransmissionSnapshot state;
        PreparedTransmission prepared;
        std::vector<double> coefficients;
        std::array<double, 2> range{};
        bool prepared_valid{};
    };

    // A fully evaluated, still-unpublished constraint solve. stage_solved()
    // collects the tendon-limit reaction into the persistent force_ scratch
    // (exposed as `force`) and mutates only scratch plus the detached
    // candidate `state` — telemetry map, work counters, last tension and the
    // consumed shift_pending flag all land at commit() together, so a throw
    // anywhere in staging leaves the live snapshot byte-identical. Unlike
    // TransmissionUpdate the solve never touches model rows, so its commit
    // is a proved memory-only swap (it runs inside the settlement's
    // noexcept commit).
    struct SolvedTransmission {
        std::span<const double> force;
        TransmissionSnapshot state;
    };

    class Transmission {
    public:
        Transmission(mjModel *model, GearingConfig gear, bool geometric = false,
                     const char *tendon = "ideal_mid_drive_freehub",
                     const char *driver = "crank_spin",
                     const char *driven = "rear_wheel_spin", const char *front = "crank");

        // Owns mjData scratch and mutates the model's tendon coefficients:
        // copying or moving would leave the source half-configured, so value
        // semantics stay deleted and ownership remains
        // std::unique_ptr<Transmission> (writers/drivetrain.cpp).
        Transmission(const Transmission &) = delete;
        Transmission &operator=(const Transmission &) = delete;
        Transmission(Transmission &&) = delete;
        Transmission &operator=(Transmission &&) = delete;
        ~Transmission() = default;

        void reset(mjData *data);

        void prepare(mjData *data);

        void set_ratio(mjData *data, double ratio);

        // Stage → commit: every mutating entry point above routes through a
        // staged TransmissionUpdate evaluated on owned scratch model/data.
        // A rejected candidate throws before any live write; a committed one
        // lands atomically (guarded model writes, then memory-only swaps).
        [[nodiscard]] TransmissionUpdate stage_ratio(mjData *data, double ratio);

        // Composed staging: `base` carries an already-staged candidate (e.g.
        // the tick's staged prepare) whose gear/state/coefficients/range seed
        // the update, so the sequential prepare→set_ratio order is preserved
        // bitwise while both changes publish in a single commit.
        [[nodiscard]] TransmissionUpdate stage_ratio(mjData *data, double ratio,
                                                     TransmissionUpdate base);

        [[nodiscard]] TransmissionUpdate stage_prepare(mjData *data);

        [[nodiscard]] TransmissionUpdate stage_reset(mjData *data);

        // Publishes a staged update. Guarded live model writes run first —
        // an engine failure there propagates and poisons the owning Stepper
        // while no logical state has been published — then the no-throw
        // gear/state/prepared swap runs. The update's state is consumed.
        void commit(TransmissionUpdate &update);

        // Solve follows the same transaction shape: staging collects the
        // force into persistent scratch and mutates only the detached
        // candidate — an allocation failure or rejection publishes nothing,
        // not even the solve's own telemetry.
        [[nodiscard]] SolvedTransmission stage_solved(mjData *data);

        // Memory-only and noexcept: the whole candidate state swaps in.
        void commit(SolvedTransmission &solved) noexcept;

        // Convenience: stage + commit, returning the persistent force span.
        std::span<const double> solved(mjData *data);

        double relative_rate(const mjData *data) const;

        TransmissionSnapshot state() const;

        void validate(const TransmissionSnapshot &state) const;

        void restore(TransmissionSnapshot state);

        const Diagnostics &diagnostics() const { return state_.diagnostics; }
        int driven_dof() const { return driven_dof_; }
        // Private FFI regression diagnostics read storage identity, not
        // snapshots: each generation counter bumps when its buffer's
        // data() pointer changes. The prepared buffers are sized at
        // construction and only ever rewritten in place, so a stable
        // nonzero generation across prepares proves no reallocation —
        // no raw address ever crosses the binding.
        const PreparedTransmission *prepared_storage() const {
            if (!geometric_)
                return nullptr;
            sync_prepared_generations();
            return &prepared_storage_;
        }
        [[nodiscard]] std::pair<std::uint64_t, std::uint64_t>
        prepared_generations() const {
            sync_prepared_generations();
            return {jacobian_generation_, qpos_generation_};
        }

    private:
        mjModel *model_ = nullptr;
        GearingConfig gear_{};
        bool geometric_ = false;
        // tendon_/driver_*/driven_* resolve unconditionally in the ctor and
        // front_/rear_/frame_ only for the geometric model; absent_id is the
        // declared "not resolved" state — never the world body's 0.
        int tendon_ = absent_id, driver_qpos_ = absent_id,
                driver_dof_ = absent_id, driven_qpos_ = absent_id,
                driven_dof_ = absent_id, front_ = absent_id, rear_ = absent_id,
                frame_ = absent_id;
        std::vector<int> coefficients_;
        OwnedData constant_, endpoint_;
        // Owned staging world: scratch_model_ mirrors the live model at each
        // coefficient-changing stage (mj_copyModel destination reuse), and
        // scratch_data_ carries a synced qpos/qvel/time copy for candidate
        // geometry and the scratch mj_setConst validation. Neither ever
        // aliases live state.
        OwnedModel scratch_model_;
        OwnedData scratch_data_;
        GeometryWorkspace geometry_;
        TransmissionSnapshot state_;
        PreparedTransmission prepared_storage_;
        bool prepared_valid_{};
        std::vector<double> force_, multipliers_, displacement_;
        // Storage-identity telemetry synced on prepared_storage() access.
        mutable const double *jacobian_identity_ = nullptr;
        mutable const double *qpos_identity_ = nullptr;
        mutable std::uint64_t jacobian_generation_ = 0;
        mutable std::uint64_t qpos_generation_ = 0;

        void sync_prepared_generations() const {
            const double *jacobian = prepared_storage_.jacobian.data();
            if (jacobian != jacobian_identity_) {
                jacobian_identity_ = jacobian;
                ++jacobian_generation_;
            }
            const double *qpos = prepared_storage_.qpos.data();
            if (qpos != qpos_identity_) {
                qpos_identity_ = qpos;
                ++qpos_generation_;
            }
        }

        double relative(const mjData *data) const;

        // Same tendon coordinate at a candidate ratio (used while staging,
        // before ratio is published).
        double relative(const mjData *data, double ratio) const;

        double geometry(mjData *data);

        // Candidate helpers — geometry evaluation refreshes kinematics on the
        // caller's data (a live-data cache refresh, not a model write) and
        // evaluates under an explicit candidate gear.
        double candidate_geometry(mjData *data, const GearingConfig &gear);

        // Fills update.prepared/coefficients/range from the just-evaluated
        // geometry_ workspace and the current qpos — values only, no writes.
        void linearize_candidate(const mjData *data, TransmissionUpdate &update,
                                 double phi);

        // Last staging step: validates the candidate snapshot, then — when a
        // wrap coefficient would change — refreshes the scratch model and
        // validates the candidate mj_setConst through the E1 boundary.
        void evaluate_candidate(mjData *data, TransmissionUpdate &update);

        // validate() minus the ideal coefficient/ratio coupling — see the
        // evaluate_candidate() comment for why staged candidates are exempt.
        void validate_state(const TransmissionSnapshot &s) const;

        void sync_scratch_data(const mjData *data) const;

        void refresh_scratch_model();

        [[nodiscard]] TransmissionUpdate make_update_from_current() const;

        [[nodiscard]] int validated_rear_teeth(int front_teeth, double ratio) const;
    };
} // namespace drivetrain
