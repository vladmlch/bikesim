// runtime/step.hpp — the owned physical step (plan A3).
//
// PhysicalRuntime's step half, ported statement-for-statement:
// apply_forces' staged force assembly plus _advance_physics' guarded
// solve and per-interval capture. The runtime binding owns construction
// and the decode/emit surface; this header carries only typed state —
// no nanobind handles. Operation order is the floating-point contract:
// every forward/write site matches its named Python line, and every
// ordered force map preserves dict insertion order.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "../engaged.hpp"
#include "../initialization/finalize.hpp"
#include "../initialization/research_startup.hpp"
#include "../rider/attachment_wrench.hpp"
#include "../rider/intent.hpp"
#include "../rider/spindle_controller.hpp"
#include "../writers/rider_contacts.hpp"
#include "../writers/tire.hpp"
#include "../writers/writer_types.hpp"
#include "config.hpp"
#include "control.hpp"
#include "static_brake.hpp"

class Stepper;

namespace runtime {
struct PresentationState;

// The step-owned slice of the runtime config — decoded in the binding.
// `monitors`, `intent` and rollback thresholds arrive prevalidated;
// `rider_present` is the articulated_planar gate control_validate_for
// needs (the runtime config pins physics_mode='physical' and
// drive_mode='articulated_effort' at decode).
struct StepConfig {
    double timestep_s = 0.;
    double control_period_s = 0.;
    // The energy/momentum measurement cadence (the oracle's
    // physical.record_decimation -> measurement_decimation split; retention
    // policy lives on AccountingConfig, not here).
    int measurement_decimation = 1;
    bool strict = false;
    double crank_length_m = 0.;
    double road_lookahead_m = 0.;
    bool rider_present = false;
    MonitorConfig monitors;
    rider::IntentConfig intent{};
};

// constraint_forces.py ConstraintForceSnapshot — solved-limit forces for
// the just-finished interval, measured against its incoming velocity.
struct ConstraintSnapshot {
    double interval_start_s = 0., interval_end_s = 0.;
    std::vector<double> qvel_start;
    std::vector<std::pair<std::string, std::vector<double>>> components;
};

// period_buffer.py RawStep — the per-interval capture A4's period
// accounting consumes. Ordered pair vectors preserve the dict insertion
// order of the Python counterparts; the Wire payloads carry the
// channel/diagnostic maps verbatim.
struct RawStep {
    std::int64_t interval_id = 0;
    double time_s = 0., end_time_s = 0.;
    std::vector<double> qpos, qvel;
    std::vector<std::pair<std::string, std::vector<double>>> components;
    std::vector<std::pair<std::string, rider::AttachmentRaw>>
        attachment_raw;
    std::vector<double> actuator_force, qfrc_passive;
    WireObject contact_truth;
    WireObject channels, diagnostics;
    std::vector<std::string> attachment_errors;
    std::optional<rider::NamedEntries<rider::JointTerms>>
        rider_control_terms;
    std::vector<std::pair<std::string, double>>
        numerical_constraint_power_w;
    double loss_step_j = 0.;
    // The energy/momentum solve runs on measured intervals only
    // (raw.full): disengaged optionals are the oracle's None raws, and
    // period close forwards the closing measured datum to the rest.
    std::optional<double> mechanical_energy_j;
    std::optional<WireObject> elastic_energy_j;
    double battery_energy_j = 0.;
    Wire electrical_power_w{};
    bool invalid_controller = false;
    WireObject effort_base;
    // The typed baseline accompanies the wire channels, so accounting a
    // committed prefix after an engine failure never reads a later attempt.
    std::optional<rider::EffortDiagnostics> effort_state;
    ConstraintSnapshot constraint_snapshot;
    bool full = false;
};

// telemetry_v2.py ForceSample — the interval-start force capture the
// loss ledger reads back (engine_passive_loss_power takes its qpos).
struct ForceSample {
    double time_s = 0.;
    std::vector<double> qpos, qvel;
    std::vector<std::pair<std::string, std::vector<double>>> components;
};

// The step-owned share of _runtime_state — the binding decodes/emits the
// full section; these are the fields the step mutates. PeriodAccounting
// separately owns history, model status, monitor and the energy ledger.
struct StepState {
    std::int64_t step = 0;
    int generation = 0;
    int measurement_decimation = 1;
    RideControl applied_control{};
    std::optional<rider::NamedEntries<double>> held_control;
    std::optional<rider::NamedEntries<rider::JointTerms>> held_rider_terms;
    bool rollback_hold = false;
    bool research_accounting_valid = true;
    bool initializing = false;
    std::array<GroundedFilter::State, 2> filters{};
    BalanceMonitor::State balance{};
    std::optional<CrashEvent> crash;
    RuntimeContacts contacts{};
    RuntimeContactQuery::State contact_query{}, probe_query{};
    EnergyState energy{};
    rider::IntentSignals signals{};
    std::optional<rider::SpindleController::State> rider_controller;
    rider::RiderIntent::State rider_intent{};
};

// The two-snapshot map _contacts returns ('front' then 'rear'), with
// optional slots mirroring Python's present/absent snapshot dict.
using WheelSnapshots =
    std::array<std::optional<TireSnapshot>, 2>;

// physical_energy.py mass_observations — the wire dict beside the two
// scalars the energy ledger sums with the elastic terms.
struct MassObservations {
    WireObject wire;
    double kinetic_energy_j = 0., gravitational_energy_j = 0.;
    double mass_kg = 0.;
};

class PhysicalStep {
    friend class bike::initialization::PhysicalInitialState;
    friend void bike::initialization::finalize_initial_state(
        bike::initialization::SetupContext&, const runtime::RuntimeConfig&);
    friend bike::initialization::ResearchStartup bike::initialization::prepare_research_startup(
        bike::initialization::SetupContext&);
public:
    // The caller resolves every named id BEFORE construction (the binding
    // validates the bootstrap geometry against the live model); `vertices`
    // is the compiled terrain cross-section — compiled_profile_vertices
    // output on the already-forwarded bootstrap data. The rider controller
    // emplaces in place (it is immovable by design); pass both pose and
    // config or neither — exactly like the oracle's control-is-None gate.
    PhysicalStep(Stepper &stepper, StepConfig config,
                 std::vector<std::array<double, 2>> vertices,
                 StaticBrake brake,
                 const std::optional<rider::SpindlePose> &rider_pose,
                 const std::optional<rider::SpindleConfig> &rider_config);

    // Immovable like the oracle's runtime: members own model pointers and
    // the rider controller emplaces in place.
    ~PhysicalStep() = default;
    PhysicalStep(const PhysicalStep &) = delete;
    PhysicalStep &operator=(const PhysicalStep &) = delete;
    PhysicalStep(PhysicalStep &&) = delete;
    PhysicalStep &operator=(PhysicalStep &&) = delete;

    // apply_forces(active=active, advance=advance) — the staged force
    // assembly. Returns this refresh's contacts; the snapshot pair the
    // caller's probe/channels consume publishes into the durable
    // snapshots_ member inside refresh_contacts (the oracle's
    // self.snapshots). `external` is the injected nv generalized force
    // (internal tests only — the public binding never passes one).
    [[nodiscard]] RuntimeContacts
    apply_forces(bool active, bool advance, double front, double rear,
                 const std::optional<std::vector<double>> &external,
                 const RideControl &control,
                 std::optional<bool> braking = std::nullopt);

    // _advance_physics — one committed interval. Validates every public
    // input before any mutation; engine failures poison the Stepper
    // through Stepper::mutate.
    [[nodiscard]] RawStep
    advance_physics(double front, double rear,
                    const std::optional<std::vector<double>> &external,
                    const RideControl &control);

    // The non-advancing staged-input probe (apply_forces with
    // advance=false): returns the ordered component fold, the actuator
    // input row, the two model friction bounds and the staged
    // diagnostics views. No state commits — probe publication rules of
    // each writer apply.
    struct ProbeResult {
        std::vector<std::pair<std::string, std::vector<double>>>
            components;
        std::vector<double> ctrl;
        double front_brake_bound_nm = 0., rear_brake_bound_nm = 0.;
        // drive diagnostics (probe tick) + contact probe view.
        WireObject drive;
        bool has_contact_probe = false;
        // probe_contacts['diagnostics'] — populated when
        // has_contact_probe is set.
        WireObject contact_probe;
        // Committed-state observability lanes — pure reads of what the
        // last interval left behind, surfaced so regression tests can
        // reach the sensordata rows (sensor id -> sensor_adr resolution)
        // and the rider-intent state without draining RawSteps (A4).
        WireObject sensors;
        WireObject intent_signals;
        double intent_inclination_rad = 0.;
    };
    [[nodiscard]] ProbeResult
    probe_step_inputs(const RideControl &control, double front_demand,
                      double rear_demand);

    // State round-trip: state() reads live owners; restore() stages then
    // commits every member (the binding validates the wire shape first).
    [[nodiscard]] StepState state() const;
    void restore(const StepState &state);

    // Accessors the binding/adapters read.
    [[nodiscard]] std::int64_t step() const noexcept { return step_; }
    [[nodiscard]] int generation() const noexcept { return generation_; }
    void bump_generation() noexcept { ++generation_; }
    // reset() overrides the bootstrap restore with the next epoch — the
    // oracle's generation increments monotonically across resets.
    void set_generation(int generation) noexcept { generation_ = generation; }
    [[nodiscard]] const RuntimeContacts &contacts() const noexcept {
        return contacts_;
    }
    void initialize_presentation(PresentationState &out) const;
    void capture_presentation(PresentationState &out) const;
    void capture_accounted_presentation(PresentationState &out) const;
    [[nodiscard]] const CrashDetector &crash_detector() const noexcept {
        return crash_detector_;
    }
    [[nodiscard]] const std::optional<CrashEvent> &crash() const noexcept {
        return crash_detector_.event();
    }
    [[nodiscard]] std::optional<BalanceLostEvent> balance_event() const noexcept {
        return balance_monitor_.state().event;
    }
    [[nodiscard]] std::span<const std::array<double, 2>> terrain_vertices() const noexcept {
        return vertices_;
    }
    [[nodiscard]] bool rider_present() const noexcept {
        return rider_control_.has_value();
    }
    [[nodiscard]] std::string_view drive_mode() const;
    [[nodiscard]] rider::SpindleController &rider_control() {
        return engaged(rider_control_);
    }
    [[nodiscard]] rider::RiderIntent &rider_intent() noexcept {
        return rider_intent_;
    }
    [[nodiscard]] ControlClock &control_clock() noexcept {
        return control_clock_;
    }
    [[nodiscard]] const rider::IntentSignals &intent_signals()
        const noexcept {
        return intent_signals_;
    }

private:
    // Freshness predicate for the mjSTAGE_VEL-skip fast path: drains the
    // drivetrain's live-model write bit into the Stepper's flag, then
    // answers whether d's position/velocity-stage products still match the
    // current qpos/qvel under the current pos/vel-stage model inputs.
    [[nodiscard]] bool posvel_fresh();
    // The deduplicated engine refresh: mj_forwardSkip(mjSTAGE_VEL, 0) while
    // posvel_fresh() holds, a full mj_forward otherwise — the flag is true
    // on return either way. Callers keep the oracle's call sites verbatim;
    // only the redundant position/velocity stages are elided.
    void refresh_forward(mjModel *m, mjData *d);
    // Pre-pass / control-tick freshness (perf-roadmap/17, R1): the
    // acceleration stage computed there is never read before the next
    // refresh_forward overwrites it, so only a stale pos/vel cache costs a
    // (full) forward; a fresh cache costs nothing.
    void ensure_posvel_fresh(mjModel *m, mjData *d);
    // R2: byte snapshot of every acceleration-stage input right after the
    // force-phase forward; step_skip may then omit its own forward.
    void snapshot_acc_inputs(const mjModel *m, const mjData *d);
    [[nodiscard]] bool acc_inputs_unchanged(const mjModel *m,
                                            const mjData *d) const;

    // apply_forces tail — the contacts refresh at physical_runtime.py:337
    // (update_grounded = active && advance) and :631 (final=true). The
    // tire backend always owns the wheel channels; the query's
    // handlebar load is its only live read. The wheel snapshots publish
    // into snapshots_ — a copy-assign into construction-reserved
    // capacity, not a by-value materialization.
    [[nodiscard]] RuntimeContacts
    refresh_contacts(bool final, std::optional<double> time_s,
                     bool update_grounded, std::span<const double> qvel = {});

    // The shared resistance stage — physical_runtime.py:315 reads the
    // applied backend's post-compute snapshots while :386 reads the
    // native_reference pair _contacts just published. Both flatten into
    // patch_loads_/patch_working_ before components_into so the spans
    // stay contiguous, then fold into accumulator_ in
    // compute_components' insertion order. `committed` is the oracle's
    // snapshots publication gate (tire.snapshots/probe_snapshots exist
    // only after a first committed/probe evaluation); the
    // native_reference pair is always published, so that call site
    // passes true.
    void add_resistance(const mjData *d, const TireSnapshot &front,
                        const TireSnapshot &rear, bool committed);

    // _rollback_brake_demand — the hill-hold reflex. `control` nullopt
    // means the apply_forces default (no explicit control).
    [[nodiscard]] double
    rollback_brake_demand(double speed_mps, const RideControl *control);

    // `drive_last` is the interval's single materialized
    // DriveTelemetry::to_map() — the caller owns the hoisted copy; every
    // consumer below sees the same lanes, matching the oracle's repeated
    // dict reads on an unchanged sim.drive.diagnostics().
    [[nodiscard]] WireObject
    sensor_channels(std::span<const double> qvel,
                    const drivetrain::Diagnostics &drive_last) const;
    [[nodiscard]] WireObject
    tire_channels(const WheelSnapshots &snapshots,
                  std::span<const double> qvel) const;
    [[nodiscard]] WireObject stored_terms() const;
    // mass_observations (physical_energy.py:45-67) — mj_mulM momentum
    // and subtree_vel linear/angular caches at the CURRENT data. Runs
    // inside the caller's mutate() frame.
    [[nodiscard]] MassObservations mass_observations(mjModel *m,
                                                     mjData *d);
    // energy_state — (mass channels map, elastic terms, total).
    [[nodiscard]] std::tuple<MassObservations, WireObject, double>
    energy_state();
    [[nodiscard]] double
    loss_increment(const std::vector<std::pair<
                       std::string, std::vector<double>>> &components,
                   std::span<const double> velocity, double dt,
                   const drivetrain::Diagnostics &drive_last) const;
    [[nodiscard]] WireObject
    balance_channel(double front, double rear, bool braking,
                    bool rider_enabled);
    // sim._update_compiled_com_marker — model.site_pos write + the data
    // mirror; a no-op when the marker site is absent.
    void update_compiled_com_marker();
    // update_rider_intent_signals — signals_from_channels evaluated on
    // the same sources the channel dicts carry (sensordata rows, the
    // committed drive telemetry lane, snapshots_): the sensor channel
    // dict with 'tires' merged in, as a wire object.
    void update_intent_signals(const WireObject &sensors_and_tires);

    // The mutate()'d body of apply_forces — separate frame keeps the
    // per-function stack budget sweep green on unoptimized builds.
    [[nodiscard]] RuntimeContacts
    apply_forces_body(bool active, bool advance, double front, double rear,
                      const std::optional<std::vector<double>> &external,
                      const RideControl &caller_control,
                      std::optional<bool> braking_opt);

    // The mutate()'d body of advance_physics — separate frame keeps the
    // per-function stack budget sweep green on unoptimized builds.
    [[nodiscard]] RawStep
    advance_body(double front, double rear,
                 const std::optional<std::vector<double>> &external,
                 const RideControl &control);

    // Everything after mj_step: the solved-interval captures, channel
    // dicts, diagnostics and RawStep assembly consume the pre-step
    // staging as one moved bundle (single parameter keeps the caller's
    // frame small as well). The lower half carries the products the
    // solved stage fills for publish_raw.
    struct SolvedInputs {
        // Staged by advance_body.
        double t = 0.;
        std::vector<double> q, v;
        double front = 0., rear = 0.;
        bool braking = false;
        double hold = 0.;
        const RideControl *control = nullptr;
        std::optional<writers::RiderPreparedAttachments> prepared;
        // `mass` channel wire — mass_observations on measured steps,
        // com_observations' {mass_kg, com_m} otherwise. `full` is the
        // single measurement gate (step.cpp advance_body computes it
        // once; raw.full and the energy gate share it).
        WireObject mass0;
        bool full = false;
        std::vector<std::pair<std::string, std::vector<double>>> components;
        std::array<int, 3> warning_counts{};
        // Produced by solve_tail, consumed by publish_raw.
        std::vector<std::pair<std::string, double>> constraint_powers{};
        writers::RiderSettleOutcome settle{};
        std::vector<std::pair<std::string, rider::AttachmentRaw>> raw_map{};
        std::vector<std::string> attachment_errors{};
        std::optional<std::string> contact_crash{};
        WireObject sensors{}, tires{}, drive{};
        writers::RiderContactsDiagnostics rider_diag{};
        double loss_step = 0., linkage_error = 0., shock_limit_power = 0.;
        Wire electrical_power_w{};
        std::vector<double> solved_actuator{}, solved_passive{};
    };
    // Solved-interval stage: constraint/brake/component merges through the
    // endpoint forward + finiteness check (physical_runtime.py:624-661).
    [[nodiscard]] RawStep solve_tail(SolvedInputs in);
    // Publish stage: energy observation, channel/diagnostic dicts,
    // RawStep assembly, commit (physical_runtime.py:662-700).
    [[nodiscard]] RawStep publish_raw(SolvedInputs in);

    Stepper *stepper_;
    StepConfig config_;
    std::vector<std::array<double, 2>> vertices_;
    RoadProfile road_profile_;  // validated/split once from vertices_

    // Resolved once at construction (physical_runtime.__init__ /
    // ride_sim.py accessors).
    int drive_ctrl_adr_ = -1;
    int cg_site_id_ = -1, frame_body_ = -1;
    int root_x_qposadr_ = 0, root_x_dofadr_ = 0, root_pitch_qposadr_ = 0;
    int shock_stroke_jnt_ = -1;
    std::array<int, 2> wheel_bodies_{};
    std::array<int, 2> wheel_spin_dofs_{};
    int crank_spin_qposadr_ = 0, crank_spin_dof_ = 0;
    int accel_sensor_adr_ = -1, gyro_sensor_adr_ = -1;
    CrashGeomIds crash_geoms_;

    // Controller ensemble — present iff the articulated_planar rider is
    // configured (variant gate mirrors sim.rider.variant).
    std::optional<rider::SpindleController> rider_control_;
    rider::RiderIntent rider_intent_;
    rider::IntentSignals intent_signals_{};
    StaticBrake brake_;

    ControlClock control_clock_;
    RuntimeContactQuery contact_query_, probe_query_;
    RuntimeContacts contacts_;
    std::array<GroundedFilter, 2> filters_;
    BalanceMonitor balance_monitor_;
    CrashDetector crash_detector_;
    ForceAccumulator accumulator_;

    // PhysicalRuntime instance state.
    std::int64_t step_ = 0;
    int generation_ = 0;
    int measurement_decimation_ = 1;
    RideControl applied_control_{};
    std::optional<rider::NamedEntries<rider::JointTerms>>
        held_rider_terms_;
    bool rollback_hold_ = false;
    bool research_accounting_valid_ = true;
    bool initializing_ = false;
    std::optional<RiderKinematicState> rider_state_;
    std::optional<ForceSample> last_force_sample_;
    ConstraintSnapshot last_constraint_snapshot_;
    EnergyState energy_;
    // self.snapshots — the current wheel-snapshot pair _contacts
    // publishes. Disengaged until the first publish, mirroring the
    // oracle's `self.snapshots = {}` (physical_runtime.py:110); not
    // wire-restored, and restore() re-disengages the slots the same
    // way. The publish path engages each slot once with the writer's
    // own per-wheel bound reserved, so every later copy-assign reuses
    // capacity — a bounded one-time cost per publication lifetime,
    // never a steady-state regrow.
    WheelSnapshots snapshots_{};

    // Construction-sized scratch for the hot path (R1): the view arrays
    // bound each writer's fixed component count; the nv buffers are the
    // fold/probe destinations.
    std::array<::ForceComponentView, 8> suspension_views_{};
    std::array<::ForceComponentView, 2> resistance_views_{};
    std::array<::ForceComponentView, 4> drive_views_{};
    std::vector<double> component_out_;
    std::vector<double> acc_inputs_;  // qfrc_applied|xfrc_applied|ctrl|act|time|qpos|qvel
    bool acc_inputs_valid_ = false;
    std::vector<double> mul_scratch_;
    // Per-side patch staging for TireSideInput (loads + working flags are
    // strided off the patch structs — contiguous copies keep the span
    // contract without touching the snapshots). std::vector<bool> is a
    // bitset and cannot back a std::span<const bool>, so the working
    // flags gather into persistent unique_ptr<bool[]> buffers. Both are
    // sized at construction to the tire writer's per-wheel patch bound
    // (station_count for the distributed backend, one contact patch for
    // compliant_2d); the regrowth guard in apply_forces_body stays as
    // the defensive invariant.
    std::array<std::vector<double>, 2> patch_loads_;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) vector<bool> is a bitset; the span<const bool> contract needs contiguous bool storage
    std::array<std::unique_ptr<bool[]>, 2> patch_working_{};
    std::array<std::size_t, 2> patch_working_capacity_{};
    // rider_state wheel_x_m staging ('front','rear' order).
    std::array<double, 2> wheel_x_m_{};
};

} // namespace runtime
