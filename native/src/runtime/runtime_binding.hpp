// runtime_binding.hpp — the owned native ride runtime (design doc 3.1).
//
// NativeRideRuntime owns the model artifact + Stepper + the complete
// captured bootstrap. It aliases nothing Python-side: the configuration and
// state dicts are decoded once into owned storage. `reset()` replays the
// stored bootstrap — the deterministic-restart surface the viewer's reset
// and the checked-replay path share. `close()` frees the engine objects
// explicitly; every other method then fails fast.
#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <nanobind/nanobind.h>
#include "accounting.hpp"
#include "bootstrap.hpp"
#include "config.hpp"
#include "step.hpp"

class Stepper;

namespace runtime {

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
    // model_path — the capture_bootstrap artifact (model.mjb). config/state
    // — the runtime_schema=1 envelopes beside it. Construction fails fast on
    // a digest/dims/state mismatch; a throwing ctor leaks nothing.
    NativeRideRuntime(nanobind::handle model_path, nanobind::handle config,
                      nanobind::handle state);
    ~NativeRideRuntime();

    NativeRideRuntime(const NativeRideRuntime &) = delete;
    NativeRideRuntime &operator=(const NativeRideRuntime &) = delete;
    NativeRideRuntime(NativeRideRuntime &&) = delete;
    NativeRideRuntime &operator=(NativeRideRuntime &&) = delete;

    // Bounded advancement: committed intervals until target_step, the
    // latched crash outcome or the wall-clock budget — whichever lands
    // first. `control` is the full RideControl wire dict for every
    // interval in the range (no command latching across calls);
    // wall_budget_s=None runs unbounded. All public inputs validate
    // before the first mutation; a mid-range failure leaves the
    // committed prefix readable. The GIL releases over the loop; the
    // guard rejects concurrent advance/reset on the same runtime.
    [[nodiscard]] AdvanceResult
    advance(std::int64_t target_step, nanobind::handle control,
            double front_brake_demand, double rear_brake_demand,
            nanobind::handle wall_budget_s);
    // apply_forces(advance=false) surfaced as the staged-inputs probe —
    // no committed state moves.
    [[nodiscard]] nanobind::dict
    probe_step_inputs(nanobind::handle control, double front_brake_demand,
                      double rear_brake_demand);
    [[nodiscard]] RuntimeSnapshot snapshot() const;
    void flush();
    [[nodiscard]] RuntimeSampleBatch prepare_samples() const;
    void acknowledge_samples(const RuntimeSampleBatch &batch);
    [[nodiscard]] nanobind::dict accounting_state() const;
    [[nodiscard]] nanobind::dict recorded_columns() const;
    // Process-isolated regression hook; never armed by an application.
    void test_fail_at_step(std::int64_t step);
    // Replays the stored bootstrap: integration vector, model coefficients,
    // writer state and counters return to the captured t=0. Mirrors
    // PhysicalRuntime.reset() — generation increments per reset.
    void reset();
    void close();
    [[nodiscard]] bool closed() const { return closed_; }

private:
    void require_open() const;
    // The ctor's decode/build/restore half — its own frame keeps the
    // stack-budget sweep green on unoptimized builds.
    void init(nanobind::handle model_path, nanobind::handle state);
    void apply_bootstrap();
    void initialize_accounting();
    void emit_reference_warning();
    void test_failure_boundary();
    [[nodiscard]] std::optional<std::string> outcome_reason() const;

    RuntimeConfig config_;
    BootstrapState bootstrap_;
    std::unique_ptr<Stepper> stepper_;
    std::unique_ptr<PhysicalStep> physical_;
    // The decoded state.runtime (+ signals/controller/intent) inventory —
    // parsed once at construction, replayed verbatim by reset().
    StepState bootstrap_step_state_;
    AccountingConfig accounting_config_;
    AccountingState bootstrap_accounting_state_;
    std::unique_ptr<PeriodAccounting> accounting_;
    // The engine may already be partially mutated when a fatal call throws.
    // Snapshots therefore read the last successful boundary, not live mjData.
    std::vector<double> committed_integration_state_;
    std::vector<double> next_integration_state_;
    std::int64_t committed_step_ = 0;
    double committed_time_s_ = 0.;
    std::optional<std::string> committed_outcome_;
    std::optional<std::int64_t> test_failure_step_;
    // Single-advancement reentrancy guard: advance(), probe_step_inputs(),
    // reset(), snapshot() and close() exclude each other — a second entry
    // fails fast instead of observing or freeing a half-advanced runtime.
    // `mutable` lets the const snapshot() take it; `closed_` is atomic so
    // the unguarded closed() property reads a defined value.
    mutable std::atomic<bool> advancing_{false};
    std::atomic<bool> closed_{false};
};

void bind_runtime(nanobind::module_ &module);

} // namespace runtime
