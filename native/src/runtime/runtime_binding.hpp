// runtime_binding.hpp — the owned native ride runtime (design doc 3.1).
//
// NativeRideRuntime owns the model artifact + Stepper + the complete
// captured bootstrap. It aliases nothing Python-side: the configuration and
// state dicts are decoded once into owned storage. `reset()` replays the
// stored bootstrap — the deterministic-restart surface the viewer's reset
// and the checked-replay path share. `close()` frees the engine objects
// explicitly; every other method then fails fast.
#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include <nanobind/nanobind.h>
#include "bootstrap.hpp"
#include "config.hpp"

class Stepper;

namespace runtime {

// An OWNED state view — the integration vector is copied out on every call,
// so a snapshot can never observe later steps or mutate the runtime.
struct RuntimeSnapshot {
    int generation = 0;
    std::int64_t step = 0;
    double time_s = 0.;
    std::vector<double> integration_state;
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

    [[nodiscard]] RuntimeSnapshot snapshot() const;
    // Replays the stored bootstrap: integration vector, model coefficients,
    // writer state and counters return to the captured t=0. Mirrors
    // PhysicalRuntime.reset() — generation increments per reset.
    void reset();
    void close();
    [[nodiscard]] bool closed() const { return closed_; }

private:
    void require_open() const;
    void apply_bootstrap();

    RuntimeConfig config_;
    BootstrapState bootstrap_;
    std::unique_ptr<Stepper> stepper_;
    std::int64_t step_ = 0;
    int generation_ = 0;
    bool closed_ = false;
};

void bind_runtime(const nanobind::module_ &module);

} // namespace runtime
