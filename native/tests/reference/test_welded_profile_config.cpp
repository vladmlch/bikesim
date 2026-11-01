// Ports tests/reference/test_welded_profile_config.py — the welded preview
// profile is the realistic pedelec default. The module-scope `cfg` fixture
// becomes one load_physics() of the live example document; every member is
// asserted exactly like the oracle's attribute reads.
#include "support/migration_test.hpp"
#include "support/golden.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/physical_config.hpp"
#include <array>
#include <cmath>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace {

namespace cfg = bike::configuration;

// bt::oracle::root() is <repo>/tests/fixtures/golden — the live example
// document is the oracle's WELDED constant verbatim.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

const cfg::SimulationPhysicsConfig &profile() {
    static const cfg::SimulationPhysicsConfig config = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    return config;
}

// test_rigid_crank_topology_is_the_default
void test_rigid_crank_topology_is_the_default() {
    const auto &drive = profile().drive;
    bt::require(drive.transmission_model == "ideal_mid_drive",
                "the welded profile selects the rigid crank topology");
    bt::require(drive.motor_clutch == false,
                "no one-way motor clutch on the rigid topology");
    bt::require(drive.rotor_inertia_kgm2 == 0.,
                "no reflected motor inertia");
    bt::require(drive.human_torque_nm == 0.,
                "effort comes from seated_climb, not a crank torque");
}

// test_assist_is_bosch_turbo_with_the_declared_envelope
void test_assist_is_bosch_turbo_with_the_declared_envelope() {
    const auto &assist = profile().drive.assist;
    bt::require(assist.profile.has_value() &&
                    *assist.profile == "bosch_cx_gen4",
                "the Bosch CX Gen 4 assist profile is selected");
    bt::require(assist.mode == "turbo", "turbo mode is selected");
    const std::vector<std::array<double, 2>> expected{
        {0., 85.}, {120., 85.}, {120.1, 47.7}, {180., 0.}};
    bt::require(assist.torque_curve.has_value(),
                "the profile resolves a torque curve");
    bt::require((assist.torque_curve).value() == expected, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "the declared torque envelope is ((0,85),(120,85),"
                "(120.1,47.7),(180,0))");
    bt::require(assist.engage_torque_nm == 4.,
                "assist engages at 4 Nm");
}

// test_rider_numbers
void test_rider_numbers() {
    const auto &pedaling = profile().drive.pedaling;
    bt::require(pedaling.coast_above_rpm == 120.,
                "coast above 120 rpm");
    bt::require(pedaling.resume_below_rpm == 105.,
                "resume below 105 rpm");
    bt::require(pedaling.coast_cadence_tau_s == .35,
                "coast cadence filter tau .35 s");
    bt::require(pedaling.rollback_brake == true,
                "rollback brake is on");
    const auto &shifting = profile().drive.shifting;
    bt::require(shifting.target_cadence_min_rpm == 75. &&
                    shifting.target_cadence_max_rpm == 110.,
                "the shift window is 75-110 rpm");
    bt::require(shifting.torque_factor == .3,
                "the shift torque derate is .3");
    const auto &climb = profile().seated_climb;
    bt::require(climb.enabled && climb.target_crank_power_w == 250.,
                "seated climb is enabled at a 250 W target");
    bt::require(climb.max_crank_torque_nm == 60.,
                "the crank torque ceiling is 60 Nm");
    bt::require(climb.surge_power_w == 400.,
                "the surge power is 400 W");
    const auto &articulated = profile().articulated;
    bt::require(articulated.pedal_torque_ripple == .5,
                "the pedal torque ripple is .5");
    bt::require(articulated.return_foot_preload_n == 40.,
                "the return-foot preload is 40 N");
    bt::require(articulated.active_positive_power_limit_w.has_value() &&
                    *articulated.active_positive_power_limit_w == 450.,
                "the active positive power limit is 450 W");
    bt::require(articulated.pedal_attachment == "spindle" &&
                    articulated.saddle_attachment == "pin" &&
                    articulated.grip_attachment == "connect",
                "the pinned plant attachments are spindle/pin/connect");
    bt::require(articulated.joint_passive_damping_nms_rad.has_value() &&
                    *articulated.joint_passive_damping_nms_rad == 0.,
                "passive hinge damping is zeroed");
}

} // namespace

int main() {
    return bt::run([] {
        test_rigid_crank_topology_is_the_default();
        test_assist_is_bosch_turbo_with_the_declared_envelope();
        test_rider_numbers();
    });
}
