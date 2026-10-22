#pragma once

#include "support/migration_test.hpp"
#include "configuration/load.hpp"
#include "drivetrain/motor.hpp"
#include "drivetrain/pedaling.hpp"
#include "drivetrain/shifting.hpp"
#include "model/builder.hpp"
#include "runtime/project_config.hpp"
#include "terrain/trackfile.hpp"
#include "writer_config.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <string_view>

namespace drive_reference {
namespace cfg = bike::configuration;

// The reference files use pytest.approx's default relative/absolute limits
// unless a test declares a tighter or looser bound explicitly. A source
// pytest.approx(abs=...) with no rel uses only that absolute bound; pass
// relative=0 explicitly in that case.
inline void near(double actual, double expected, std::string_view label,
                 double relative = 1e-6, double absolute = 1e-12) {
    bt::require(std::isfinite(actual) && std::isfinite(expected) &&
        std::abs(actual - expected) <= std::max(absolute, relative * std::abs(expected)), label);
}

inline constexpr double rpm_to_rad_s = 2. * std::numbers::pi / 60.;

// Obtain the low-level policy through the real profile resolver and wire
// decoder. Keeping that transformation out of the test avoids giving the
// profile-override tests their own implementation of the feature under test.
inline bike::io::Value projected_writer(const cfg::PhysicalDriveConfig& drive) {
    auto physics = std::make_unique<cfg::SimulationPhysicsConfig>();
    physics->physics_mode = "physical"; physics->drive_mode = "crank_effort"; physics->drive = drive;
    return bike::io::Value{bike::io::Value::Object{
        {"schema", bike::io::Value{bike::io::Number{"2"}}},
        {"drive", runtime::project_drive_config(*physics)}}};
}

inline bike::io::Value projected_writer(const cfg::AssistConfig& assist) {
    cfg::PhysicalDriveConfig drive;
    drive.transmission_model = "ideal_mid_drive";
    drive.assist = assist;
    return projected_writer(drive);
}

inline drivetrain::DrivePolicyConfig projected_policies(const cfg::AssistConfig& assist) {
    auto decoded = std::make_unique<nativecfg::WriterConfig>(nativecfg::decode(projected_writer(assist)));
    bt::require(decoded->drive.has_value(), "physical profile projects a drive policy");
    return decoded->drive->policies; // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
}

inline cfg::AssistConfig turbo_source() {
    cfg::AssistConfig result;
    result.profile = "bosch_cx_gen4";
    result.mode = "turbo";
    return result;
}

inline drivetrain::AssistConfig turbo_config() {
    return projected_policies(turbo_source()).assist;
}

inline double run_assist(drivetrain::AssistController& controller, double human,
    double cadence_rpm, double speed, double seconds, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
    std::optional<double> request = std::nullopt, double dt = .001) {
    double torque = 0.;
    const auto count = static_cast<int>(std::llround(seconds / dt));
    for (int step = 0; step < count; ++step)
        torque = controller.step(human, cadence_rpm, speed, false, dt, request);
    return torque;
}

inline drivetrain::CadenceShifter shifter(int rear = 28, double tau = 0.) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
    drivetrain::ShiftingConfig config{
        .enabled = true,
        .cassette = {10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51},
        .target_cadence_min_rpm = 75., .target_cadence_max_rpm = 110.,
        .shift_cooldown_s = .4, .shift_cut_duration_s = .2,
        .torque_factor = .3, .cadence_smoothing_tau_s = tau,
        .upshift_slip_limit_mps = .5, .upshift_slip_mode = "legacy_signed"};
    return drivetrain::CadenceShifter(
        {.front_teeth = 22, .rear_teeth = rear, .chain_pitch_m = .0127}, std::move(config));
}

inline drivetrain::PedalingConfig pedaling_config(double tau = 0.) {
    return {.enabled = true, .coast_above_rpm = 120., .resume_below_rpm = 105.,
        .stop_time_s = .35, .coast_cadence_tau_s = tau, .mash_cadence_rpm = 45.,
        .mash_torque_nm = 60., .effort_slew_nm_s = 0., .rollback_brake = true,
        .rollback_engage_mps = .25, .rollback_release_mps = 0., .rollback_demand = 1.};
}
} // namespace drive_reference
