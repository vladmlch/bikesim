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

    void observe(std::int64_t interval_id, double time_s, const WireObject &channels);
    [[nodiscard]] WireObject as_wire() const;
    [[nodiscard]] static ModelStatus from_wire(const WireObject &value);
};

[[nodiscard]] std::vector<std::string>
channel_violations(const WireObject &channels, double maximum_compression_fraction,
                   double maximum_linkage_error_m);
[[nodiscard]] bool energy_quality_ok(const WireObject &energy);

} // namespace runtime
