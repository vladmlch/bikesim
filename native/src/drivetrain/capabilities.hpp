#pragma once

#include "../config_types.hpp"

namespace drivetrain {

// Availability is distinct from qualification: task evidence records which
// platform/oracle gates have actually run for these existing writer branches.
struct Capability {
    bool elastic_chain = true;
    bool motor_clutch = true;
    bool reflected_rotor = true;
};
[[nodiscard]] Capability implemented_capabilities() noexcept;

struct MotorTopology {
    const char* actuator_joint;
    const char* reduction_driver;
    const char* reduction_body;
    bool crank_clutch;
    bool motor_freewheel;
};

// Shared typed selection for the actual writer, setup and topology tests.
// Validates incompatible elastic-chain/clutch/rotor/shifting combinations.
[[nodiscard]] MotorTopology motor_topology(const DriveConfig& config);

} // namespace drivetrain
