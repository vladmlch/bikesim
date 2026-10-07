#pragma once
#include "../drivetrain/freehub.hpp"
#include "../drivetrain/motor.hpp"
#include "../drivetrain/pedaling.hpp"
#include "../drivetrain/shifting.hpp"
#include "../drivetrain/transmission.hpp"
#include "../rtsan.hpp"
#include "writer_types.hpp"
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

    // Fixed-width angle mirror — construction-sized (simplified emits crank
    // only, elastic emits crank + cassette), so snapshot copies never
    // allocate the way the former vector did.
    struct DriveAngles {
        std::array<double, 2> values{};
        std::size_t count = 0;
    };

    struct DriveSnapshot {
        PedalingSnapshot pedaling;
        ShiftingSnapshot shifting;
        AssistSnapshot assist;
        BatterySnapshot battery;
        std::optional<FreehubSnapshot> hub;
        std::optional<double> shift_time_s, last_time_s, reference, psi;
        std::optional<DriveAngles> angles;
        // Typed telemetry lanes (R1) — the closed 67-field schema replaces
        // the per-tick map; lane presence keeps the wire's absent/explicit-
        // None distinction and for_each()/to_map() emit the map's sorted
        // order at the serialization boundary.
        DriveTelemetry last;
        std::optional<DriveTelemetry> probe_last;
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
        ForceKind kind;
    };

    inline constexpr std::array<ForceComponentSpec, 4> force_component_specs{{
        {.slot = ForceComponent::chain, .name = "chain",
         .kind = ForceKind::transmission},
        {.slot = ForceComponent::freehub, .name = "freehub",
         .kind = ForceKind::transmission},
        {.slot = ForceComponent::drive_bearings, .name = "drive_bearings",
         .kind = ForceKind::bearing},
        {.slot = ForceComponent::ideal_transmission, .name = "ideal_transmission",
         .kind = ForceKind::transmission},
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

    namespace detail {
        // One of the two fixed transaction banks (R1). Construction/reset
        // sizes every member; a stage call seeds the bank from live_ and
        // fills its rows/lanes/candidate storage in place — the bank is
        // rewritten, never rebuilt, so a warm tick allocates nothing.
        struct TickBank {
            DriveSnapshot snapshot;
            PedalingState result{};
            PedalingSnapshot pedaling;
            ShiftingSnapshot shifting;
            AssistSnapshot assist;
            BatterySnapshot battery;
            std::optional<FreehubSnapshot> hub;
            // Candidate force rows — construction-sized nv vectors indexed
            // by ForceComponent; the emitted span carries component_count
            // entries in the declared force_component order (3 physical,
            // 4 for the simplified models).
            std::array<std::vector<double>, 4> components;
            std::array<ForceComponentView, 4> component_views{};
            std::array<std::string_view, 4> component_names{};
            std::size_t component_count = 0;
            // Candidate actuator writes as (ctrl index, value) pairs; the
            // live data->ctrl row is touched only at commit.
            std::array<std::pair<int, double>, 2> ctrl{};
            std::size_t ctrl_count = 0;
            // Staged transmission updates — the optionals stay engaged once
            // first staged so their vectors/telemetry lanes keep capacity;
            // the *_staged flags track whether the CURRENT tick refreshed
            // them.
            std::optional<TransmissionUpdate> ideal_hub, clutch, freewheel;
            bool ideal_hub_staged = false, clutch_staged = false,
                    freewheel_staged = false;
            // advance publishes the whole candidate; probe publishes only
            // the declared probe telemetry (components + probe_last
            // diagnostics).
            bool advance = false, probe = false;
            // generation retires outstanding handles on the next stage or
            // commit of this bank; staged_epoch records commit_epoch_ at
            // stage time so commit rejects a seed invalidated by an
            // intervening publication.
            std::uint64_t generation = 0, staged_epoch = 0;
        };

        // Same fixed-bank shape for settlements: the write plan carries
        // (TelemetryField, scalar) pairs — lanes always exist, so unlike
        // the former map-slot plan there is no detached-map fallback —
        // plus the per-transmission staged solves and policy candidates.
        struct SettlementBank {
            struct Write {
                TelemetryField field = TelemetryField::assist_demand_gated;
                TelemetryScalar value{};
            };
            std::array<Write, telemetry_field_count> writes{};
            std::size_t write_count = 0;
            // Staged per-transmission solve candidates — the optionals stay
            // engaged once first staged so state swaps keep their storage;
            // engagement tracks the topology (staged every settle when the
            // transmission exists).
            std::optional<SolvedTransmission> ideal_solve, clutch_solve,
                    freewheel_solve;
            // Candidate store after the pending debit — identical
            // arithmetic to Battery::draw, staged detached so a failed
            // publish never drains.
            std::optional<BatterySnapshot> battery;
            std::optional<AssistSnapshot> assist;
            // Pending actuation is consumed only when the settlement
            // commits.
            bool clear_pending = false;
            std::uint64_t generation = 0, staged_epoch = 0;
        };
    } // namespace detail

    class DrivetrainWriter;

    // A fully evaluated, still-unpublished drivetrain tick — a non-owning
    // view over one of the writer's two fixed banks. stage_prepare /
    // stage_components build every candidate up front — policy snapshots,
    // force rows, actuator writes, diagnostics and (through E2's
    // TransmissionUpdate) the staged model updates for hub/clutch/freewheel —
    // so a rejection anywhere publishes nothing. commit() runs the guarded
    // model writes first, then publishes the memory-only remainder.
    //
    // The views stay valid until the writer stages this bank again or the
    // handle is consumed by commit(); committing any other tick or
    // settlement first makes the seed stale — commit() then throws
    // std::logic_error rather than publish a predated snapshot.
    class PreparedTick {
    public:
        // Public const views are the API: the handle is a read-only staged
        // snapshot, so the reference/const members are the design, not an
        // accident — the same aggregate-publicity the dataclass mirror
        // policy accepts elsewhere.
        // NOLINTBEGIN(cppcoreguidelines-avoid-const-or-ref-data-members, cppcoreguidelines-non-private-member-variables-in-classes)
        const DriveSnapshot &snapshot;
        const PedalingState &result;
        const std::span<const ForceComponentView> components;
        const std::span<const std::string_view> component_names;
        const std::span<const std::pair<int, double> > ctrl;
        const bool advance, probe;
        // NOLINTEND(cppcoreguidelines-avoid-const-or-ref-data-members, cppcoreguidelines-non-private-member-variables-in-classes)

    private:
        friend class DrivetrainWriter;
        PreparedTick(DrivetrainWriter &owner, detail::TickBank &bank) noexcept
            : snapshot(bank.snapshot), result(bank.result),
              // first() over the container-bound span: component_count/
              // ctrl_count are invariantly within the fixed array extents
              // (writers only ever assign components_.size() ≤ 4 or 0).
              components(std::span<const ForceComponentView>(
                             bank.component_views)
                             .first(bank.component_count)),
              component_names(std::span<const std::string_view>(
                                  bank.component_names)
                                  .first(bank.component_count)),
              ctrl(std::span<const std::pair<int, double> >(bank.ctrl)
                       .first(bank.ctrl_count)),
              advance(bank.advance), probe(bank.probe),
              owner_(&owner), bank_(&bank), generation_(bank.generation) {}
        DrivetrainWriter *owner_;
        detail::TickBank *bank_;
        std::uint64_t generation_;
    };

    // A fully evaluated, still-unpublished settlement — a non-owning view
    // over one of the writer's two settlement banks plus the solved force
    // span. stage_settle() performs every throwing step — candidate battery
    // debit, pending validation, force solution (which drives the
    // transmissions' own solve telemetry) and the telemetry write plan —
    // so commit() is a memory-only, non-throwing publication of the whole
    // result: force, battery candidate, telemetry and pending removal land
    // together or not at all.
    class PreparedSettlement {
    public:
        // Solved force — a view over the writer's persistent transmission_
        // scratch so the FFI binding can box it before commit(), and the
        // convenience settle() keeps returning the same persistent span.
        std::span<const double> force{};

    private:
        friend class DrivetrainWriter;
        DrivetrainWriter *owner_ = nullptr;
        detail::SettlementBank *bank_ = nullptr;
        std::uint64_t generation_ = 0;
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

        // R1 allocation-free face. components_into runs one components
        // tick through the fixed banks (same staging and arithmetic as
        // stage_components+commit) and fills `out` with views over the
        // published component rows — the caller supplies the view array.
        // try_components_into is the prevalidated warm-core entry: the
        // tick's public rejections map onto CoreStatus, and no
        // throw/string construction happens in its own path.
        void components_into(const TickInputs &inputs,
                             std::span<ForceComponentView> out);
        // Steady-tick note: a tick whose staged transmission coefficients
        // change rehearse-commits on a scratch model (evaluate_candidate's
        // refresh_scratch_model + engine::set_const path), which is outside
        // the marked zero-allocation claim — the mark covers the declared
        // warm path where the staged update carries no coefficient delta.
        [[nodiscard]] CoreStatus
        try_components_into(const TickInputs &inputs,
                            std::span<ForceComponentView> out) noexcept
                BIKE_NONBLOCKING;

        // Stage → commit: all runtime/pending/time checks run first, every
        // policy advance is computed on detached candidates, model updates
        // stay inside staged TransmissionUpdates, and the caller may box the
        // result (FFI) before commit publishes anything. The returned handle
        // borrows one of the two fixed banks — see PreparedTick for the
        // lifetime/staleness contract.
        [[nodiscard]] PreparedTick stage_prepare(const PrepareInputs &inputs);

        [[nodiscard]] PreparedTick stage_components(const TickInputs &inputs);

        void commit(const PreparedTick &tick);

        // Settlement follows the same transaction shape: staging runs the
        // transmission solve, the battery debit on a detached snapshot and
        // the telemetry write plan, so a throw anywhere publishes nothing.
        // commit() only lands already-computed values.
        [[nodiscard]] PreparedSettlement stage_settle();

        // Memory-only and noexcept: the pending actuation, battery store
        // and telemetry all publish together. Precondition: the handle came
        // from this writer's latest stage_settle — a stale or foreign handle
        // is a contract violation (checked; terminates under noexcept).
        void commit(PreparedSettlement &settlement) noexcept;

        // Convenience settle for native callers: stage + commit, returning
        // the persistent transmission_ span it always did.
        [[nodiscard]] std::span<const double> settle();

        [[nodiscard]] const DriveTelemetry &diagnostics(bool probe) const noexcept;

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
        // Detached snapshot staging for reset()/restore(): keeping the
        // candidate in a member keeps those cold paths under the 8192-byte
        // frame contract and reuses its storage across calls.
        DriveSnapshot snapshot_scratch_;
        ForceComponents components_;
        std::vector<double> transmission_;
        // The two fixed tick/settlement banks plus the detached policy
        // candidates the stage/validate calls share — all sized at
        // construction so a warm tick only rewrites existing storage.
        std::array<detail::TickBank, 2> banks_{};
        std::size_t staged_bank_ = 0;
        std::array<detail::SettlementBank, 2> settlement_banks_{};
        std::size_t staged_settlement_ = 0;
        // Bumped by every publication (tick/settlement commits, reset,
        // restore, restart_clock): a handle staged at an earlier epoch is
        // stale — its bank seed no longer matches live_.
        std::uint64_t commit_epoch_ = 0;
        PedalingPolicy pedaling_scratch_;
        CadenceShifter shifting_scratch_;
        AssistController assist_scratch_;
        Battery battery_scratch_;
        std::optional<Freehub> hub_scratch_;

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

        void shifts(DriveTelemetry &d, const ShiftingSnapshot &state,
                    std::optional<double> shift_time_s) const;

        // Stage-bank selection and staleness checks: stage_* pick the
        // inactive bank, retire its previous handle (generation bump) and
        // record the publication epoch; commit_* rejects a handle whose
        // bank or epoch has moved on.
        detail::TickBank &next_tick_bank() noexcept;
        detail::SettlementBank &next_settlement_bank() noexcept;
        detail::TickBank &checked_tick(const PreparedTick &tick);
        detail::SettlementBank &checked_settlement(PreparedSettlement &s);

        // Shared prepare staging: the shifter and pedaling updates run on
        // the supplied candidate policies; a staged gear change composes
        // onto the bank's already-staged ideal-hub update. Nothing live is
        // written.
        void stage_pedal_advance(detail::TickBank &bank,
                                 const PrepareInputs &inputs,
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
            DriveAngles angles;
            std::optional<double> psi;
        };

        // The solve half of stage_components: pedaling advance, chain/freehub
        // and bearing force evaluation and the motor/battery solve land the
        // component forces on the candidate bank and return the scalar
        // metrics the telemetry build publishes.
        [[nodiscard]] TickMetrics
        stage_components_metrics(detail::TickBank &bank, const TickInputs &in);

        // The publish half: derived-force revalidation, pending reservation,
        // control rows, the telemetry lanes and mirror staging all land on
        // the candidate bank from the already-computed metrics.
        void stage_components_telemetry(detail::TickBank &bank,
                                        const TickInputs &in,
                                        const TickMetrics &metrics);

        // Stages the snapshot's embedded policy/transmission mirrors with
        // the same values commit() will publish, so commit() performs no
        // fresh state() copies (which allocate).
        void stage_snapshot_mirrors(detail::TickBank &bank);

        // Re-runs each policy's set_state validation on the staged candidate
        // snapshots; staged TransmissionUpdates were already validated inside
        // stage_* by evaluate_candidate().
        void validate_tick(detail::TickBank &bank);

        // Same re-validation for the settlement candidates.
        void validate_settlement(const detail::SettlementBank &bank);

        // Post-commit snapshot view of a staged transmission update, written
        // into caller-owned storage — mirrors what Transmission::state()
        // reports after commit() lands. The by-value twin serves the cold
        // reset() path.
        static void committed_into(const TransmissionUpdate &update,
                                   TransmissionSnapshot &snapshot);
        static void committed_into(const TransmissionUpdate &update,
                                   std::optional<TransmissionSnapshot> &out);
        [[nodiscard]] static TransmissionSnapshot
        committed_snapshot(const TransmissionUpdate &update);
    };
} // namespace drivetrain
