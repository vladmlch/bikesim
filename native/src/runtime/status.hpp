// Applicability, numerical quality and reference rejection remain independent.
#pragma once
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "samples.hpp"

namespace runtime {

struct ReferenceFailure {
    double time_s = 0.;
    std::vector<std::string> violations;
    [[nodiscard]] Wire as_wire() const;
    [[nodiscard]] std::string message() const;
};

class InvalidReferenceRun final : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
    ~InvalidReferenceRun() override;
    InvalidReferenceRun(const InvalidReferenceRun &) = default;
    InvalidReferenceRun &operator=(const InvalidReferenceRun &) = default;
    InvalidReferenceRun(InvalidReferenceRun &&) = default;
    InvalidReferenceRun &operator=(InvalidReferenceRun &&) = default;
};

struct ReferenceMonitor {
    bool strict = false;
    std::optional<ReferenceFailure> first_failure;
    bool warning_pending = false;

    void observe(const ReferenceFailure &failure);
    [[nodiscard]] std::optional<std::string> take_warning();
    [[nodiscard]] WireObject as_wire() const;
    [[nodiscard]] static ReferenceMonitor from_wire(const WireObject &value);
};

struct ModelStatus {
    std::vector<std::pair<std::string, std::int64_t>> counts;
    std::optional<WireObject> first;
    std::int64_t last_interval = -1;
    double maximum_compression_fraction = .15;
    double maximum_linkage_error_m = .002;
    Wire numerically_valid = Wire("not_evaluated");
    std::string calibration_status = "parameterized_unvalidated";

    // `include_energy=false` mirrors the oracle's masked observe on
    // unmeasured intervals ({k:v for k,v in channels if k!='energy'}):
    // the forwarded residual must not feed the sticky numerically_valid
    // verdict. Other channel checks still run — only the energy verdict
    // is suppressed.
    void observe(std::int64_t interval_id, double time_s, const WireObject &channels,
                 bool include_energy = true);
    void reset();
    [[nodiscard]] WireObject as_wire() const;
    // The persisted state.runtime.model_status document — dataclass field
    // schema (native/setup.py:_runtime_state), distinct from as_wire()'s
    // channel/summary publication view.
    [[nodiscard]] WireObject state_wire() const;
    [[nodiscard]] static ModelStatus from_wire(const WireObject &value);
};

[[nodiscard]] std::vector<std::string>
channel_violations(const WireObject &channels, double maximum_compression_fraction,
                   double maximum_linkage_error_m);
[[nodiscard]] bool energy_quality_ok(const WireObject &energy);

// Simulation decisions «Действует» 34 (user, 2026-11-09): a ride fails
// when on any single physics step the press of either foot or the
// saddle support in the rider_support history is below -6 N.
inline constexpr double kSupportPullToleranceN = 6.;
// Judges the rider_support object of accounting_state()["history"] —
// front_pedal, rear_pedal and saddle min_n — against
// kSupportPullToleranceN. A channel never observed (null min_n)
// carries no pull evidence.
[[nodiscard]] bool rider_support_ok(const WireObject &rider_support);

} // namespace runtime
