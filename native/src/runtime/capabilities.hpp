#pragma once

#include "../configuration/model_config.hpp"
#include "../configuration/rider_spec.hpp"
#include <optional>
#include <string>
#include <vector>

namespace runtime {
// A structural dispatch report, deliberately separate from migration gates.
// Availability never implies numerical parity or a completed runtime test.
struct CapabilityReport {
    bool supported = false;
    std::vector<std::string> reasons;
    bool implemented = false;
    bool qualification_pending = true;
    std::vector<std::string> paths;
    std::vector<std::string> rejections;
    std::string allocation_numeric_identity;
    // Explicit dispatch row; absence preserves the planar report contract.
    std::optional<bike::spatial::CapabilityReport> spatial;
};
[[nodiscard]] CapabilityReport capabilities(
    const bike::configuration::SimulationPhysicsConfig&,
    const bike::configuration::RiderSpecs&);
[[nodiscard]] CapabilityReport implemented_capabilities(
    const bike::configuration::SimulationPhysicsConfig&,
    const bike::configuration::RiderSpecs&);
void validate_physical_capabilities(
    const bike::configuration::SimulationPhysicsConfig&,
    const bike::configuration::RiderSpecs&);
} // namespace runtime
