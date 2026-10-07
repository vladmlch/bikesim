#pragma once
#include "../drivetrain/freehub.hpp"
#include "../drivetrain/motor.hpp"
#include "../drivetrain/pedaling.hpp"
#include "../drivetrain/shifting.hpp"
#include "../drivetrain/transmission.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace drivetrain {
    class ArithmeticError : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;

        ~ArithmeticError() override; // key function: anchors the vtable
        ArithmeticError(const ArithmeticError &) = default;

        ArithmeticError &operator=(const ArithmeticError &) = default;

        ArithmeticError(ArithmeticError &&) = default;

        ArithmeticError &operator=(ArithmeticError &&) = default;
    };

    struct RideControl {
        std::optional<double> motor_torque_nm, motor_limit_nm, human_torque_nm;
        bool rider_enabled = true;
    };

    struct PendingActuation {
        double requested{}, omega{}, dt{};
        bool enabled{};
    };

    struct DriveSnapshot {
        PedalingSnapshot pedaling;
        ShiftingSnapshot shifting;
        AssistSnapshot assist;
        BatterySnapshot battery;
        std::optional<FreehubSnapshot> hub;
        std::optional<double> shift_time_s, last_time_s, reference, psi;
        std::optional<std::vector<double> > angles;
        Diagnostics last;
        std::optional<Diagnostics> probe_last;
        std::optional<PendingActuation> pending_actuation;
        std::optional<TransmissionSnapshot> ideal_hub, clutch, freewheel;
    };

    // Snapshot↔config relational invariants. restore() runs this before any
    // member is staged: a candidate state must agree with the selected config
    // and with itself, not merely carry field-valid values.
    void validate(const DriveSnapshot &snapshot, const DriveConfig &config,
                  const mjModel &model);

    using ForceComponents = std::vector<std::pair<std::string, std::vector<double> > >;

    // The fixed serialized force-component layout — names and order mirror
    // the Python dict's insertion order, so they are part of the wire
    // contract. The enum names each slot for the write sites, and the table
    // below binds each name to its slot instead of relying on emplace order.
    enum class ForceComponent : std::uint8_t {
        chain,
        freehub,
        drive_bearings,
        ideal_transmission
    };

    struct ForceComponentSpec {
        ForceComponent slot;
        std::string_view name;
    };

    inline constexpr std::array<ForceComponentSpec, 4> force_component_specs{{
        {.slot = ForceComponent::chain, .name = "chain"},
        {.slot = ForceComponent::freehub, .name = "freehub"},
        {.slot = ForceComponent::drive_bearings, .name = "drive_bearings"},
        {.slot = ForceComponent::ideal_transmission, .name = "ideal_transmission"},
    }};

    [[nodiscard]] constexpr std::size_t
    force_component_index(ForceComponent component) noexcept {
        return static_cast<std::size_t>(component);
    }

    // Compile-time pin: every table row must sit at its enum position, so a
    // reordered row is a build failure rather than a silent wire change.
    consteval bool force_component_table_ordered() {
        for (std::size_t i = 0; i < force_component_specs.size(); ++i)
            if (force_component_index(force_component_specs[i].slot) != i)
                return false;
        return true;
    }
    static_assert(force_component_table_ordered(),
                  "force-component rows must sit at their enum positions");

    struct PrepareInputs {
        RideControl control;
        double dt{};
        bool braking = false, active = true, advance = true, contact = true;
        std::optional<double> slip, ceiling;
    };

    struct TickInputs {
        RideControl control;
        double dt{}, speed{}, sensed{};
        bool braking = false, active = true, advance = true, contact = true;
        std::optional<PedalingState> pedaling;
        std::optional<double> slip;
    };

    // A fully evaluated, still-unpublished drivetrain tick. stage_prepare /
    // stage_components build every candidate up front — policy snapshots,
    // force rows, actuator writes, diagnostics and (through E2's
    // TransmissionUpdate) the staged model updates for hub/clutch/freewheel —
    // so a rejection anywhere publishes nothing. commit() runs the guarded
    // model writes first, then publishes the memory-only remainder.
    struct PreparedTick {
        DriveSnapshot snapshot;
        PedalingState result{};
        PedalingSnapshot pedaling;
        ShiftingSnapshot shifting;
        AssistSnapshot assist;
        BatterySnapshot battery;
        std::optional<FreehubSnapshot> hub;
        ForceComponents components;
        // Candidate actuator writes as (ctrl index, value) pairs; the live
        // data->ctrl row is touched only at commit.
        std::vector<std::pair<int, double> > ctrl;
        std::optional<TransmissionUpdate> ideal_hub, clutch, freewheel;
        // advance publishes the whole candidate; probe publishes only the
        // declared probe telemetry (components + probe_last diagnostics).
        bool advance = false, probe = false;
    };

    // A fully evaluated, still-unpublished settlement. stage_settle()
    // performs every throwing step — candidate battery debit, pending
    // validation, force solution (which drives the transmissions' own
    // solve telemetry) and diagnostics construction — so commit() is a
    // memory-only, non-throwing publication of the whole result:
    // force vector, battery candidate, telemetry and pending removal land
    // together or not at all.
    struct PreparedSettlement {
        // One declared telemetry write: a pointer to an existing live_.last
        // slot (resolved while staging — std::map element addresses are
        // stable — so commit performs no lookup and no allocation) plus the
        // value to publish into it.
        struct Write {
            DiagnosticValue *slot;
            DiagnosticValue value;
        };

        // Solved transmission force — a view over the writer's persistent
        // transmission_ scratch so the FFI binding can box it before
        // commit(), and the convenience settle() keeps returning the same
        // persistent span.
        std::span<const double> force{};
        // In-place telemetry writes, applied to slots that already exist
        // in live_.last (stage_settle proves membership first, so commit
        // never allocates a map node). Only scalar values ever land here —
        // a string-valued write would make the in-place path allocating.
        static constexpr std::size_t write_capacity = 16;
        std::array<Write, write_capacity> writes{};
        std::size_t write_count = 0;
        // Full candidate diagnostics — engaged when an in-place write
        // cannot express the publication (a key absent from live_.last —
        // e.g. after restoring a sparse `last` — or the geometric
        // transmission telemetry merge, which carries a string status).
        // All candidate work happens during staging; commit only moves.
        std::optional<Diagnostics> last;
        // Staged per-transmission solve candidates — one per transmission
        // the topology owns. The solve's own telemetry/state publication
        // rides these: commit() lands each through
        // Transmission::commit(SolvedTransmission&), a memory-only swap, so
        // a staging throw (a bad_alloc in a candidate map, an unprepared
        // geometric hub) publishes none of them.
        std::optional<SolvedTransmission> ideal_solve, clutch_solve,
                freewheel_solve;
        // Candidate store after the pending debit — identical arithmetic to
        // Battery::draw, staged detached so a failed publish never drains.
        std::optional<BatterySnapshot> battery;
        std::optional<AssistSnapshot> assist;
        // Pending actuation is consumed only when the settlement commits.
        bool clear_pending = false;
    };

    // One writer per owning Stepper context: model_/data_ are non-owning
    // pointers to the context's engine objects, and every mutable field —
    // geometry_, components_, transmission_, live_ and the policy objects —
    // is per-instance scratch fully rewritten by each call. Calls arrive
    // through the nanobind boundary while the GIL is held, so there is no
    // concurrent access; sharing one writer across contexts would require
    // separate instances or external synchronization outside realtime code.
    class DrivetrainWriter {
    public:
        DrivetrainWriter(mjModel *model, mjData *data, DriveConfig config);

        void reset();

        void restart_clock();

        [[nodiscard]] PedalingState
        prepare(const RideControl &control, double dt, bool braking,
                bool active, bool advance, bool contact,
                std::optional<double> slip, std::optional<double> ceiling);

        [[nodiscard]] const ForceComponents &
        components(const RideControl &control, double dt,
                   double speed, bool braking, bool active,
                   bool advance, double sensed,
                   const std::optional<PedalingState> &pedaling,
                   bool contact, std::optional<double> slip);

        // Stage → commit: all runtime/pending/time checks run first, every
        // policy advance is computed on detached candidates, model updates
        // stay inside staged TransmissionUpdates, and the caller may box the
        // result (FFI) before commit publishes anything.
        [[nodiscard]] PreparedTick stage_prepare(const PrepareInputs &inputs);

        [[nodiscard]] PreparedTick stage_components(const TickInputs &inputs);

        void commit(PreparedTick &tick);

        // Settlement follows the same transaction shape: staging runs the
        // transmission solve, the battery debit on a detached snapshot and
        // every diagnostics allocation, so a throw anywhere publishes
        // nothing. commit() only lands already-computed values.
        [[nodiscard]] PreparedSettlement stage_settle();

        // Memory-only and noexcept: the pending actuation, battery store
        // and telemetry all publish together.
        void commit(PreparedSettlement &settlement) noexcept;

        // Convenience settle for native callers: stage + commit, returning
        // the persistent transmission_ span it always did.
        [[nodiscard]] std::span<const double> settle();

        [[nodiscard]] const Diagnostics &diagnostics(bool probe) const noexcept;

        [[nodiscard]] Diagnostics stored_energy();

        [[nodiscard]] DriveSnapshot state() const;

        void restore(const DriveSnapshot &snapshot);

        [[nodiscard]] const Transmission *transmission_storage() const noexcept {
            return ideal_hub_.get();
        }

    private:
        mjModel *model_ = nullptr;
        mjData *data_ = nullptr;
        DriveConfig config_;
        // geometric_hub_ selects the telemetry-merge settle path (the
        // geometric transmission publishes its solve diagnostics — a map
        // carrying a string status — which cannot be written in place).
        bool simplified_ = false, effort_ = false, geometric_hub_ = false;
        GeometryWorkspace geometry_;
        // frame_/crank_/rear_ resolve unconditionally in the ctor; cassette_
        // resolves only for the elastic-chain model, so it needs a named
        // invalid sentinel — 0 would silently be the world body. Dereferences
        // go through cassette_body() below.
        int frame_ = absent_id, crank_ = absent_id, rear_ = absent_id,
                cassette_ = absent_id;

        struct Joint {
            int qpos{}, dof{};
        };

        std::map<std::string, Joint> joints_;
        std::vector<Joint> bearing_joints_;
        int human_actuator_ = absent_id, motor_actuator_ = absent_id;
        PedalingPolicy pedaling_;
        CadenceShifter shifting_;
        AssistController assist_;
        Battery battery_;
        std::optional<Freehub> hub_;
        std::unique_ptr<Transmission> ideal_hub_, clutch_, freewheel_;
        DriveSnapshot live_;
        ForceComponents components_;
        std::vector<double> transmission_;

        // cassette_ resolves only under the elastic-chain topology; the
        // absent_id sentinel makes every other mode honest, and this throws
        // like engaged() does for the same compiler-invisible invariant.
        [[nodiscard]] int cassette_body() const {
            if (cassette_ == absent_id)
                throw std::logic_error(
                    "cassette body is absent outside the elastic_chain model");
            return cassette_;
        }

        [[nodiscard]] double
        angle(int body, std::optional<double> reference = std::nullopt);

        void shifts(Diagnostics &d, const ShiftingSnapshot &state,
                    std::optional<double> shift_time_s) const;

        // Shared prepare staging: the shifter and pedaling updates run on the
        // supplied candidate policies; a staged gear change composes onto the
        // tick's already-staged ideal-hub update. Nothing live is written.
        void stage_pedal_advance(PreparedTick &tick, const PrepareInputs &inputs,
                                 PedalingPolicy &pedaling);

        // Scalar results the solve half of stage_components hands to the
        // telemetry build — kept private so the split stays an
        // implementation detail. Each staging frame stays under the
        // 8192-byte frame contract this way (the sanitizer build's red
        // zones pushed the single merged frame past it).
        struct TickMetrics {
            double extension = 0., rate = 0., tension = 0., energy = 0.,
                    torque = 0., deflection = 0., relative_rate = 0.,
                    cadence = 0., omega_crank = 0., omega = 0., human = 0.,
                    sensor = 0., assist_sensor = 0., request = 0.,
                    safety = 0., delivered = 0., electrical = 0.;
            bool enabled = false;
            PedalingState ps;
            std::vector<double> angles;
            std::optional<double> psi;
        };

        // The solve half of stage_components: pedaling advance, chain/freehub
        // and bearing force evaluation and the motor/battery solve land the
        // component forces on the candidate tick and return the scalar
        // metrics the telemetry build publishes.
        [[nodiscard]] TickMetrics
        stage_components_metrics(PreparedTick &tick, const TickInputs &in);

        // The publish half: derived-force revalidation, pending reservation,
        // control rows, the telemetry map and mirror staging all land on the
        // candidate tick from the already-computed metrics.
        void stage_components_telemetry(PreparedTick &tick,
                                        const TickInputs &in,
                                        const TickMetrics &metrics);

        // Stages the snapshot's embedded policy/transmission mirrors with
        // the same values commit() will publish, so commit() performs no
        // fresh state() copies (which allocate).
        void stage_snapshot_mirrors(PreparedTick &tick) const;

        // Re-runs each policy's set_state validation on the staged candidate
        // snapshots; staged TransmissionUpdates were already validated inside
        // stage_* by evaluate_candidate().
        void validate_tick(const PreparedTick &tick) const;

        // Same re-validation for the settlement candidates.
        void validate_settlement(const PreparedSettlement &settlement) const;

        // Post-commit snapshot view of a staged transmission update —
        // mirrors what Transmission::state() reports after commit() lands.
        [[nodiscard]] static TransmissionSnapshot
        committed_snapshot(const TransmissionUpdate &update);
    };
} // namespace drivetrain
