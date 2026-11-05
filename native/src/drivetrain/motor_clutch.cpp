#include "capabilities.hpp"
#include "../config_validation.hpp"

namespace drivetrain {

Capability implemented_capabilities() noexcept { return {}; }

MotorTopology motor_topology(const DriveConfig& config) {
    validate(config);
    return {
        .actuator_joint = config.motor_clutch ? "drive_shaft_spin" : "crank_spin",
        .reduction_driver = config.motor_clutch ? "drive_shaft_spin" : "crank_spin",
        .reduction_body = config.motor_clutch ? "drive_shaft" : "crank",
        .crank_clutch = config.motor_clutch
    };
}

} // namespace drivetrain
