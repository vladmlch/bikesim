// runtime.hpp — the owned native ride runtime, pure C++ (task T09;
// migration/interface-contracts.md §1).
//
// NativeRideRuntime owns the model image + Stepper + the complete
// captured bootstrap. It aliases nothing caller-side: the configuration
// and state documents decode once into owned records and documents.
// `reset()` replays the stored bootstrap — the deterministic-restart
// surface the viewer's reset and the checked-replay path share.
// `close()` frees the engine objects explicitly; every other method then
// fails fast.
//
// Nothing in this header reaches nanobind/Python: the adapter layer in
// runtime_binding.cpp materializes the Python documents into
// bike::io::Value trees and composes the same RuntimeBootstrap.
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "../writers/rider_contacts.hpp"
#include "accounting.hpp"
#include "bootstrap.hpp"
#include "config.hpp"
#include "control.hpp"
#include "presentation.hpp"
#include "step.hpp"

class Stepper;
namespace bike::golden { class RuntimeAccess; }
namespace bike::validation { class RuntimeRigAccess; class SpatialRideAccess; }

namespace runtime {
namespace spatial { class CheckpointAccess; }

// The compiled-model artifact, owned. `mjb` is the verbatim model.mjb
// image; `sha256` is the lowercase-hex content digest the bootstrap's
// state.model_digest pins.
struct ModelImage {
    std::vector<std::byte> mjb;
    std::string sha256;
};

// One fully staged bootstrap: the model image plus the decoded config and
// state inventories the runtime consumes.
struct RuntimeBootstrap {
    ModelImage model;
    RuntimeConfig config;
    BootstrapState state;
};

// Shared construction for fresh SetupOwner and frozen-bootstrap loading.
// Stepper must outlive this owner; no bootstrap is needed for construction.
[[nodiscard]] std::unique_ptr<PhysicalStep>
make_physical_step(Stepper& stepper, const RuntimeConfig& config);

// Loads one capture_bootstrap directory (config.json, state.json,
// model.mjb) into an owned RuntimeBootstrap — externalized array/nonfinite
// tags resolve against the capture-root-relative path convention and every
// declared digest/shape verifies before the staged decode runs.
[[nodiscard]] RuntimeBootstrap
read_bootstrap(const std::filesystem::path &directory);

// The composition seam shared by read_bootstrap() and the bindings
// adapter: the config arrives already decoded (position 1 of the
// validation order — the caller controls what runs before it), the state
// as an owned Value tree, the model as owned bytes. Validation inside
// runs at the same ordinal positions the Python-shaped path ran it — the
// state.model_digest == sha256(model) comparison, then the writer-config
// decode, then the model-bound and document-level state sections.
[[nodiscard]] RuntimeBootstrap compose_bootstrap(RuntimeConfig config,
                                                 ModelImage model,
                                                 const bike::io::Value &state);

// An OWNED state view — the integration vector is copied out on every call,
// so a snapshot can never observe later steps or mutate the runtime.
struct RuntimeSnapshot {
    int generation = 0;
    std::int64_t step = 0;
    double time_s = 0.;
    std::vector<double> integration_state;
    // The latched terminal outcome (crash:<cause> family) or none.
    std::optional<std::string> outcome;
    SamplePtr latest_sample;
    WireObject view;
    WireObject model_status;
    std::optional<ReferenceFailure> first_failure;
    std::string model_sha256;
    std::optional<SpatialBootstrapState> spatial;
    // Owned committed tyre material bytes. Spatial is the W27 canonical codec;
    // planar is the legacy double row for same-process observational equality.
    std::vector<std::byte> tire_state_bytes;
};

// An owned presentation boundary. The digest identifies the compiled model,
// not merely its dimensions; no live model/data pointers cross into the UI.
struct RenderFrame {
    RuntimeSnapshot state;
    PresentationState presentation;
    std::string model_sha256;
};

// The committed prefix of one advance() call (design section 3.2).
struct AdvanceResult {
    std::int64_t step = 0;
    double time_s = 0.;
    // 'target' | 'budget' | 'outcome'
    std::string reason;
    std::optional<std::string> outcome;
};

class NativeRideRuntime {
public:
    // A fully staged bootstrap — produced by read_bootstrap() or by the
    // adapter's compose_bootstrap() on materialized documents. Construction
    // fails fast on a digest/dims/state mismatch; a throwing ctor leaks
    // nothing.
    explicit NativeRideRuntime(RuntimeBootstrap bootstrap);
    ~NativeRideRuntime();

    NativeRideRuntime(const NativeRideRuntime &) = delete;
    NativeRideRuntime &operator=(const NativeRideRuntime &) = delete;
    NativeRideRuntime(NativeRideRuntime &&) = delete;
    NativeRideRuntime &operator=(NativeRideRuntime &&) = delete;

    // Bounded advancement: committed intervals until target_step, the
    // latched crash outcome or the wall-clock budget — whichever lands
    // first. `control` is the full decoded RideControl for every interval
    // in the range (no command latching across calls); a disengaged
    // wall_budget_s runs unbounded. All public inputs validate before the
    // first mutation; a mid-range failure leaves the committed prefix
    // readable. The guard rejects concurrent advance/reset on the same
    // runtime.
    [[nodiscard]] AdvanceResult
    advance(std::int64_t target_step, const RideControl &control,
            double front_brake_demand, double rear_brake_demand,
            std::optional<double> wall_budget_s);
    // apply_forces(advance=false) surfaced as the staged-inputs probe —
    // no committed state moves.
    [[nodiscard]] PhysicalStep::ProbeResult
    probe_step_inputs(const RideControl &control, double front_brake_demand,
                      double rear_brake_demand);
    [[nodiscard]] RuntimeSnapshot snapshot() const;
    [[nodiscard]] RenderFrame render_frame() const;
    [[nodiscard]] ModelImage model_image() const;
    void flush();
    [[nodiscard]] RuntimeSampleBatch prepare_samples() const;
    void acknowledge_samples(const RuntimeSampleBatch &batch);
    [[nodiscard]] WireObject accounting_state() const;
    [[nodiscard]] SampleColumns recorded_columns() const;
    [[nodiscard]] std::span<const SamplePtr> recorded_intervals() const;
    // Seals the committed retained index into an owned immutable episode
    // (design A2). The trailing partial period is closed first, like
    // flush(); the returned value outlives reset() and close().
    [[nodiscard]] OwnedNativeEpisode seal_episode();
    void release_samples();
    // Frontend-only finish gate, checked after every committed physics step.
    // Research installs its own experiment limits and leaves this unset.
    void set_finish_position(double position_m);
    // Spatial duration limit, configured at t=0 and latched by the accepted
    // physical interval after higher-priority invalid/physical outcomes.
    void set_completion_step(std::int64_t step);
    // Diagnostic-only, reads accepted-owner availability without touching
    // poisoned engine state or manufacturing a resumable checkpoint.
    [[nodiscard]] std::optional<std::string> checkpoint_unavailable_reason() const;
    // Process-isolated regression hook; never armed by an application.
    void test_fail_at_step(std::int64_t step);
    // Replays the stored bootstrap: integration vector, model coefficients,
    // writer state and counters return to the captured t=0. Mirrors
    // PhysicalRuntime.reset() — generation increments per reset.
    void reset();
    void close();
    [[nodiscard]] bool closed() const { return closed_; }

    // The pending non-strict reference rejection (the RuntimeWarning the
    // adapter emits at the held-GIL boundary after advance/flush/
    // seal_episode/reset). Pure layer keeps the state; emission is the
    // adapter's.
    [[nodiscard]] std::optional<std::string> take_pending_warning();

private:
    friend class NativeResearchRuntime;
    friend class spatial::CheckpointAccess;
    friend class bike::golden::RuntimeAccess;
    friend class bike::validation::RuntimeRigAccess;
    // W35/W36 cold qualification fixtures; no application checkpoint API.
    friend class bike::validation::SpatialRideAccess;
    void advance_interval(const RideControl &command, double front, double rear,
                          GoldenForceObservation *golden = nullptr);
    // All owning copies are captured before accounting/publication. Together
    // with the next integration/presentation buffers, this is one boundary.
    struct BoundaryState {
        int generation = 0;
        std::int64_t step = 0;
        double time_s = 0.;
        double position_m = 0.;
        double battery_energy_j = 0.;
        std::optional<CrashEvent> crash;
        std::optional<BalanceLostEvent> balance;
        std::optional<std::string> outcome;
        std::optional<SpatialBootstrapState> spatial;
        std::vector<std::byte> tire_state_bytes;
    };
    [[nodiscard]] BoundaryState capture_boundary() const;
    void commit_boundary(BoundaryState&&) noexcept;
    void flush_accounting();
    double committed_position_m_ = 0.;
    double committed_battery_energy_j_ = 0.;
    std::optional<CrashEvent> committed_crash_;
    std::optional<BalanceLostEvent> committed_balance_;
    void require_open() const;
    void require_boundary_ready() const;
    // The ctor's decode/build/restore half — its own frame keeps the
    // stack-budget sweep green on unoptimized builds.
    struct ContinuationTag {};
    NativeRideRuntime(RuntimeBootstrap bootstrap, ContinuationTag);
    void init(bool continuation = false);
    void apply_bootstrap();
    void initialize_accounting(bool continuation = false);
    void test_failure_boundary();
    [[nodiscard]] std::optional<std::string> outcome_reason() const;

    RuntimeConfig config_;
    BootstrapState bootstrap_;
    ModelImage model_image_;
    std::unique_ptr<Stepper> stepper_;
    std::unique_ptr<PhysicalStep> physical_;
    // The decoded state.runtime (+ signals/controller/intent) inventory —
    // parsed once at construction, replayed verbatim by reset().
    StepState bootstrap_step_state_;
    // state.rider_contacts decoded against the writer topology — absent
    // when the config has no rider_contacts writer (like the oracle's
    // optional `_rider_contacts` restore).
    std::optional<
        std::pair<writers::RiderContactsState,
                  std::optional<writers::RiderContactsProbe>>>
        bootstrap_rider_contacts_;
    AccountingConfig accounting_config_;
    AccountingState bootstrap_accounting_state_;
    std::unique_ptr<PeriodAccounting> accounting_;
    // The engine may already be partially mutated when a fatal call throws.
    // Snapshots therefore read the last successful boundary, not live mjData.
    std::vector<double> committed_integration_state_;
    std::vector<double> next_integration_state_;
    PresentationState committed_presentation_, next_presentation_;
    int committed_generation_ = 0;
    std::int64_t committed_step_ = 0;
    double committed_time_s_ = 0.;
    std::optional<std::string> committed_outcome_;
    std::optional<SpatialBootstrapState> committed_spatial_;
    std::vector<std::byte> committed_tire_state_bytes_;
    // Physics/reset may complete before an owning capture fails. Keep the
    // preceding snapshot readable, but require reset before further inputs.
    bool boundary_recovery_required_ = false;
    std::optional<double> finish_position_m_;
    std::optional<std::int64_t> completion_step_;
    std::optional<std::int64_t> test_failure_step_;
    // reset() replaces the accounting owner mid-flight; a warning raised
    // by the reset-time flush moves here so take_pending_warning() still
    // surfaces it to the adapter (the old emit happened between the flush
    // and the owner swap).
    std::optional<std::string> pending_warning_;
    // Single-advancement reentrancy guard: advance(), probe_step_inputs(),
    // reset(), snapshot() and close() exclude each other — a second entry
    // fails fast instead of observing or freeing a half-advanced runtime.
    // `mutable` lets the const snapshot() take it; `closed_` is atomic so
    // the unguarded closed() property reads a defined value.
    mutable std::atomic<bool> advancing_{false};
    std::atomic<bool> closed_{false};
};

} // namespace runtime
