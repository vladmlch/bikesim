// stepper.hpp — owns mjModel/mjData; buffers never escape ownership.
#pragma once
#include <mujoco/mujoco.h>
#include <array>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <nanobind/nanobind.h>
#include "engine_call.hpp"
#include "rtsan.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/equality_reactions.hpp"
#include "writers/writer_types.hpp"

class SuspensionWriter;
class BrakeWriter;
class CruiseWriter;
struct CruiseState;
class ResistanceWriter;
class TireWriter;
class RiderForcesWriter;

namespace drivetrain {
    class DrivetrainWriter;
}

namespace writers {
    class RiderContactWriter;
    // T3b-2 measurement value types (writers/rider_contacts.hpp) —
    // forward-declared for the settle face's signatures.
    struct RiderPreparedAttachments;
    struct RiderAttachmentMeasurement;
    struct RiderSettleOutcome;
    using RiderIntervalState =
        std::pair<std::vector<double>, std::vector<double>>;
}

struct TireSideInput;

class Stepper {
public:
    explicit Stepper(const std::string &mjb_path);

    // Config-bridge ctor: the dict from tools.native_config.project(env)
    // (or an empty dict — every writer stays disabled). The dict is read
    // ONCE here; nothing keeps a reference into Python objects.
    Stepper(const std::string &mjb_path, const nanobind::dict &config);

    Stepper(const std::string &mjb_path, nanobind::handle config);

    ~Stepper();

    Stepper(const Stepper &) = delete;

    Stepper &operator=(const Stepper &) = delete;

    // Immobile owner: a defaulted move would copy the raw m_/d_ pointers
    // and double-free via the moved-from object's destructor.
    Stepper(Stepper &&) = delete;

    Stepper &operator=(Stepper &&) = delete;

    // Fixed-storage status paths are marked for RTSan. Public methods build
    // Python-visible exceptions only after leaving those marked regions.
    [[nodiscard]] bool try_step(engine::ErrorBuffer &error) noexcept BIKE_NONBLOCKING;
    [[nodiscard]] bool try_forward(engine::ErrorBuffer &error) noexcept BIKE_NONBLOCKING;
    void step();
    void forward();
    void reset();
    // Refresh outside a marked region after process-global callback changes.
    // A direct C++ try_* caller must invoke this before its realtime loop.
    void refresh_time_callback_policy();

    // Binding wrappers use this boundary for writer mutations that can call
    // mj_setConst indirectly. Read-only diagnostics remain available after a
    // fatal engine error.
    template <typename Action>
    decltype(auto) mutate(Action &&action) {
        require_healthy();
        require_unmarked_time_callback();
        try {
            return std::forward<Action>(action)();
        } catch (const engine::EngineFailure &) {
            poisoned_ = true;
            throw;
        }
    }
    // Internal typed access for the private per-call drivetrain oracle adapter.
    [[nodiscard]] mjModel *model() const { return m_; }
    [[nodiscard]] mjData *data() const { return d_; }

    drivetrain::DrivetrainWriter &drive() const;

    // Nonthrowing presence check for the installed drive writer — private
    // hooks use it to refuse mutating the live model behind the writer's
    // caches (drive() itself throws when the config is absent).
    [[nodiscard]] bool has_drive() const { return drive_ != nullptr; }

    void set_inputs(std::span<const double> ctrl,
                    std::span<const double> force);

    [[nodiscard]] std::span<const double> qfrc_applied() const {
        return std::views::counted(d_->qfrc_applied, m_->nv);
    }

    [[nodiscard]] std::span<const double> actuator_force() const {
        return std::views::counted(d_->actuator_force, m_->nu);
    }

    // Restore a solver-relevant snapshot (qpos/qvel/act/qacc_warmstart/time)
    // exactly as `mj_resetData` + buffer writes do in Python. Widths are
    // checked against the model BEFORE reset — a bad argument must not
    // clobber previously restored state. Inputs are snapshotted before reset,
    // so views of this Stepper (including cross-buffer aliases) are accepted.
    // Empty `act` is legal (na==0).
    void set_state(std::span<const double> qpos,
                   std::span<const double> qvel,
                   std::span<const double> act,
                   std::span<const double> warmstart,
                   double time);

    // views::counted instead of the (ptr, size) span ctor: clang's
    // -Wunsafe-buffer-usage-in-container rejects the raw two-argument form.
    // nq/nv are mjtSize (signed) — counted takes iter_difference_t (signed);
    // passing size_t here trips GCC's -Wsign-conversion, so no cast.
    [[nodiscard]] std::span<const double> qpos() const {
        return std::views::counted(d_->qpos, m_->nq);
    }

    [[nodiscard]] std::span<const double> qvel() const {
        return std::views::counted(d_->qvel, m_->nv);
    }

    [[nodiscard]] std::span<const double> qacc() const {
        return std::views::counted(d_->qacc, m_->nv);
    }

    [[nodiscard]] std::span<const double> qfrc_constraint() const {
        return std::views::counted(d_->qfrc_constraint, m_->nv);
    }

    // nefc is a RUNTIME mjData field — the span's size varies per forward.
    // (mjModel has no nefc member; do not look there.) d->efc_force lives
    // in mjData's constraint arena, whose contents rewind and whose offset
    // moves whenever the contact set changes — the binding copies this
    // into an owned snapshot rather than exposing a live view.
    [[nodiscard]] std::span<const double> efc_force() const {
        return std::views::counted(d_->efc_force, d_->nefc);
    }

    // d.ctrl is an actuator INPUT buffer (nu wide) — the brake writer's
    // output surface; kept readable so tests can verify the write path.
    [[nodiscard]] std::span<const double> ctrl() const {
        return std::views::counted(d_->ctrl, m_->nu);
    }

    [[nodiscard]] double time() const { return d_->time; }
    // compute_qfrc_components on the CURRENT mjData — insertion-ordered
    // (name, nv-vector) pairs, identical to the Python dict. Throws
    // std::logic_error when constructed without a suspension config.
    [[nodiscard]] std::vector<std::pair<std::string, std::vector<double> > >
    suspension_components() const;

    // BrakeController.compute — (front, rear) torques at the current qvel.
    [[nodiscard]] std::pair<double, double>
    brake_torques(double front_demand, double rear_demand) const;

    // compute() + the ctrl writes (ride_sim.py:530-534).
    void apply_brake(double front_demand, double rear_demand);

    // Standalone PI state survives mjData set_state; no actuator writes.
    [[nodiscard]] double cruise_compute(bool rear_in_contact,
                                        bool traction_limited, std::optional<bool> controller_grounded);

    void cruise_reset();

    void cruise_set_target_speed(double value_kmh);

    [[nodiscard]] double cruise_set_assist_compensation(double support_factor);

    [[nodiscard]] CruiseState cruise_state() const;

    void set_cruise_state(const CruiseState &state);

    // ExternalResistanceApplier.compute_components — 'road_rolling' +
    // 'aerodynamic' for the given per-side tire snapshots.
    [[nodiscard]] std::vector<std::pair<std::string, std::vector<double> > >
    resistance_components(const TireSideInput &front,
                          const TireSideInput &rear) const;

    // TireForceApplier.compute_qfrc advance=True on the CURRENT mjData —
    // restores no brush state on its own; call set_tire_state first when
    // replaying. Throws std::logic_error without a tire config.
    [[nodiscard]] std::vector<double> tire_qfrc(double dt);

    // The artifact's flattened brush-state encoding: names follow the
    // flatten_row schema ('front.tangent.0' style; leaf 'front.tangent'
    // columns also decode, all-NaN vector -> unset field). Restoring
    // clears the once-per-timestamp clock like `last_time_s = None`.
    // A segment is NaN (unset) or a finite integer in int's range; invalid
    // input leaves both brush state and clock unchanged.
    void set_tire_state(std::span<const std::string> names,
                        std::span<const double> row);

    [[nodiscard]] std::vector<double> tire_state() const;

    [[nodiscard]] const std::vector<std::string> &
    tire_state_names() const;

    // The writer itself — snapshots()/diagnostics() are read here by the
    // binding. nullptr without a tire config (callers gate on it).
    [[nodiscard]] const TireWriter *tire() const { return tire_.get(); }

    // ---- Owned-writer access for the runtime's physical step (plan A3) --
    // Mutable borrowed references into the Stepper-owned writers — the
    // runtime's step kernel drives these directly; the boxed convenience
    // surfaces above serialize into fresh vectors, which the warm step
    // cannot afford. Same throw-on-absent contract as drive(); the
    // borrowed references never outlive the Stepper and remain valid only
    // while the caller holds no other alias into the same data.
    [[nodiscard]] SuspensionWriter &suspension() const;
    [[nodiscard]] ResistanceWriter &resistance() const;
    [[nodiscard]] RiderForcesWriter &rider_forces() const;
    [[nodiscard]] TireWriter &tire_writer() const;
    [[nodiscard]] CruiseWriter &cruise() const { return require_cruise(); }
    [[nodiscard]] bool has_suspension() const noexcept {
        return suspension_ != nullptr;
    }
    [[nodiscard]] bool has_resistance() const noexcept {
        return resistance_ != nullptr;
    }
    [[nodiscard]] bool has_rider_forces() const noexcept {
        return rider_forces_ != nullptr;
    }
    [[nodiscard]] bool has_rider_contacts() const noexcept {
        return rider_contacts_ != nullptr;
    }
    [[nodiscard]] bool has_cruise() const noexcept {
        return cruise_ != nullptr;
    }
    [[nodiscard]] std::span<const double> qfrc_passive() const {
        return std::views::counted(d_->qfrc_passive, m_->nv);
    }
    // RiderForceApplier.apply on the CURRENT mjData, read back as a vector:
    // zeros(nv) + each path's force at its dofadr — the 'seated_interfaces'
    // accumulator row. Throws std::logic_error without a rider_forces
    // config.
    [[nodiscard]] std::vector<double> rider_forces_qfrc() const;

    // attachment_wrench.py equality_qfrc on the CURRENT mjData: zeros(nv),
    // then J^T times the solved multipliers of the equality rows whose
    // efc_id matches — so a missing equality is exactly zeros(nv). The
    // row membership is re-derived from the live arena on every call.
    [[nodiscard]] std::vector<double> rider_equality_qfrc(int eq_id) const;

    // Internal typed access for the binding's equality diagnostics: the
    // Stepper-owned EqualityReactions workspace (lazy — materialized on
    // first use, borrowed pointer, never a second owner). Runs
    // require_healthy() like every read path.
    [[nodiscard]] rider::EqualityReactions &rider_equalities() const;

    // T2b — attachment_wrench.py measurement faces on the owned (m_, d_)
    // pair. These are thin owners over the typed rider:: core: the
    // Stepper supplies the model/data, the EqualityReactions reader and
    // the reused DGELSD workspace; every returned block owns its
    // storage. All validation/errors match the Python oracle (see
    // rider/attachment_wrench.hpp).
    [[nodiscard]] rider::DenseMatrix rider_relative_planar_jacobian(
        int body_a, int body_b, std::span<const double> point,
        bool rotational) const;

    [[nodiscard]] rider::AttachmentGeometry rider_prepare_attachment(
        int eq_id, int body_rider, int body_bike,
        std::span<const double> point, std::span<const double> normal,
        std::string kind, bool rotational, double half_patch_m,
        const std::optional<std::vector<double>> &pull_direction) const;

    [[nodiscard]] rider::AttachmentRaw rider_attachment_raw(
        int eq_id, int body_rider, int body_bike,
        std::span<const double> point, std::span<const double> normal,
        std::string kind, bool rotational, double half_patch_m,
        const std::optional<std::vector<double>> &pull_direction) const;

    [[nodiscard]] rider::AttachmentRaw rider_attachment_raw_from_geometry(
        const rider::AttachmentGeometry &geometry,
        bool validate_wrench) const;

    [[nodiscard]] rider::AttachmentSample rider_attachment_sample(
        int eq_id, int body_rider, int body_bike,
        std::span<const double> point, std::span<const double> normal,
        const std::string &kind, bool rotational, double half_patch_m,
        const std::optional<std::vector<double>> &pull_direction) const;

    // The reused DGELSD workspace behind the attachment measurements
    // (lazy, like equality_scratch_): bound (nv, max(nv,6), 1) admits
    // the (k,6) spatial solves, the (ncr,6)/(ncb,6) validation solves
    // and the (nv, r) recover problems this face ever runs.
    [[nodiscard]] rider::LeastSquaresWorkspace &
    rider_attachment_lstsq() const;

    // ForceAccumulator.total() (force_accumulator.py:47-51): zero-init,
    // then sequential in-place adds in the GIVEN order — the P2 ordering
    // contract the native step loop replicates. Each component is checked
    // like acc.add (width nv, all-finite) before it joins the fold —
    // std::invalid_argument on violation. Available on every Stepper.
    [[nodiscard]] std::vector<double>
    total(std::span<const std::vector<double>> components) const;

    // T3b-1 — the model-owned RiderContactWriter (rider_contacts.py
    // RiderContactApplier): pad force assembly, grip capture/release,
    // diagnostics, probe publication and snapshot restore live on the
    // writer; the Stepper owns it, supplies the (m_, d_) pair and the
    // construction-sized nv output buffer. The accessor throws
    // std::logic_error naming 'rider_contacts' when the config is absent,
    // like drive()/require_cruise(). set_enabled/reset/release paths go
    // through the binding's mutate() boundary.
    [[nodiscard]] writers::RiderContactWriter &rider_contacts() const;
    void rider_contacts_reset();
    void rider_contacts_restart_clock();
    void rider_contacts_initialize_settled_state();
    void rider_contacts_release_all();
    [[nodiscard]] bool rider_contacts_set_enabled(std::string_view name,
                                                  bool enabled);
    [[nodiscard]] double rider_contacts_stored_energy() const;
    [[nodiscard]] std::vector<double>
    rider_contacts_qfrc(double dt, bool advance, bool detailed);

    // T3b-2 — the settle/sample face (rider_contacts.py:221-412):
    // prepare_attachment_raw's interval-start geometry capture,
    // settle_welds' solved-reaction latch plus its measurement half, and
    // attachment_samples' optional interval-state measurement. Settle
    // and samples write data.qpos/qvel during the swap — the binding
    // routes them through mutate() like every mutating entry point.
    [[nodiscard]] writers::RiderPreparedAttachments
    rider_contacts_prepare_attachment_raw() const;
    [[nodiscard]] writers::RiderAttachmentMeasurement
    rider_contacts_attachment_samples(
        const std::optional<writers::RiderIntervalState> &interval_state,
        bool raw);
    [[nodiscard]] writers::RiderSettleOutcome rider_contacts_settle(
        const std::optional<writers::RiderIntervalState> &interval_state,
        bool raw,
        const writers::RiderPreparedAttachments *prepared);

private:
    mjModel *m_;
    mjData *d_;
    bool poisoned_ = false;
    mjfTime allowed_time_callback_ = nullptr;
    mjfTime observed_time_callback_ = nullptr;
    bool time_callback_policy_ready_ = false;
    bool unmarked_time_callback_safe_ = true;
    void require_healthy() const;
    void require_unmarked_time_callback();
    std::unique_ptr<SuspensionWriter> suspension_;
    std::unique_ptr<BrakeWriter> brake_;
    std::unique_ptr<CruiseWriter> cruise_;

    [[nodiscard]] CruiseWriter &require_cruise() const;

    std::unique_ptr<ResistanceWriter> resistance_;
    std::unique_ptr<TireWriter> tire_;
    std::unique_ptr<RiderForcesWriter> rider_forces_;
    std::unique_ptr<drivetrain::DrivetrainWriter> drive_;
    // T3b-1 — the model-owned RiderContactWriter plus its construction-
    // sized nv qfrc buffer (resized beside the writer in the config ctor).
    std::unique_ptr<writers::RiderContactWriter> rider_contacts_;
    std::vector<double> rider_contacts_out_;

    // R1 warm-core scratch for the convenience faces above: the boxed
    // methods route through the writers' *_into APIs over these members —
    // the writer side allocates nothing and the serialized copy out is the
    // only per-call allocation (it IS the boxing surface). The view arrays
    // are sized to each writer's maximum component count (static_asserts in
    // stepper.cpp pin them to the writer constants); the nv buffers are
    // sized in the config ctor when its writer is built.
    mutable std::array<ForceComponentView, 8> suspension_views_{};
    mutable std::array<ForceComponentView, 2> resistance_views_{};
    mutable std::vector<double> rider_out_;
    std::vector<double> tire_out_;
    // Lazy per-Stepper scratch for rider_equality_qfrc — the reusable
    // nefc multiplier / nv qfrc buffers behind it. Null until the first
    // call; the arena contents are still re-derived on every call.
    mutable std::unique_ptr<rider::EqualityReactions> equality_scratch_;
    // T2b lazy scratch: the shared DGELSD workspace plus the nv qfrc
    // buffer the equality_qfrc stage writes before the dof gathers.
    // Contents are re-derived on every call like equality_scratch_.
    mutable std::unique_ptr<rider::LeastSquaresWorkspace> attachment_lstsq_;
    mutable std::vector<double> attachment_qfrc_;
    // nv-sized view over attachment_qfrc_ (assigned on first use or an
    // nv change — never a per-call reallocation).
    [[nodiscard]] std::span<double> attachment_qfrc_scratch() const;
};
