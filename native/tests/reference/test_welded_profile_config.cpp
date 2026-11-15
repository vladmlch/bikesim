// Ports tests/reference/test_welded_profile_config.py — the welded preview
// profile is the realistic pedelec default. The module-scope `cfg` fixture
// becomes one load_physics() of the live example document; every member is
// asserted exactly like the oracle's attribute reads.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "support/temporary_directory.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/physical_config.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
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

// test_motor_clutch_topology_is_the_default
// The motor clutch is the default topology (user, 2026-11-05): the pedals
// drive the chainring through a one-way clutch, so they turn backward
// freely and the motor may turn the chainring while they stand.
void test_motor_clutch_topology_is_the_default() {
    const auto &drive = profile().drive;
    bt::require(drive.transmission_model == "ideal_mid_drive",
                "ideal lossless drive");
    bt::require(drive.motor_clutch,
                "the welded example uses the motor clutch");
    bt::require(drive.human_torque_nm == 0.,
                "effort comes from seated_climb, not a crank torque");
}

// test_fixture_copies_carry_the_research_clutch
// The test copies under tests/fixtures/config must ride the same clutch
// topology as the research profiles they mirror — the motor clutch is the
// bike everywhere the rider pedals (user, 2026-11-10), not a research-only
// A/B arm.
[[gnu::noinline]] void test_fixture_copies_carry_the_research_clutch() {
    for (const char *name :
         {"viewer_physics_welded.toml", "viewer_physics_pneumatic.toml",
          "viewer_physics_distributed.toml"}) {
        const auto fixture = cfg::load_physics(
            bt::fixture(std::string("config/") + name));
        const auto research = cfg::load_physics(repo_file(
            std::string("examples/research/") + name));
        bt::require(research.drive.motor_clutch,
                    std::string(name) + " research profile uses the clutch");
        bt::require(fixture.drive.motor_clutch ==
                        research.drive.motor_clutch,
                    std::string(name) +
                        " fixture copy matches the research clutch");
    }
    // The variant copies mirror the welded document's clutch too.
    for (const char *name :
         {"variant_alt_support.toml", "variant_grip_weld_coercion.toml"}) {
        const auto fixture = cfg::load_physics(
            bt::fixture(std::string("config/") + name));
        bt::require(fixture.drive.motor_clutch == profile().drive.motor_clutch,
                    std::string(name) +
                        " variant copy matches the welded clutch");
    }
}

// test_rotor_inertia_is_the_audit_value
// The 0.2 kg·m² reflected rotor inertia (drive audit 2026-10-07,
// unverified) rides [drive] rotor_inertia_kgm2 through the usual schema →
// validation path; negatives reject on load.
void test_rotor_inertia_is_the_audit_value() {
    bt::require(profile().drive.rotor_inertia_kgm2 == .2,
                "the welded research profile carries the audited rotor inertia");
    const std::string source = bt::text(repo_file(
        "examples/research/viewer_physics_welded.toml"));
    const std::string key = "rotor_inertia_kgm2 = 0.2";
    const auto anchor = source.find(key);
    bt::require(anchor != std::string::npos,
                "the welded document carries the rotor key");
    std::string injected(source);
    injected.replace(anchor, key.size(), "rotor_inertia_kgm2 = -0.2");
    const bt::TemporaryDirectory dir("welded-negative-rotor");
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << injected; }
    bool threw = false;
    try {
        (void)cfg::load_physics(path);
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(
            std::string_view(failure.what()).find("rotor_inertia_kgm2") !=
                std::string_view::npos,
            "the load rejection names the rotor key");
    }
    bt::require(threw, "the schema rejects a negative rotor inertia");
}

// test_relaxation_is_kinetic_for_the_rotor_drive
// With a 0.2 kg·m² rotor the drivetrain equilibrates through a mode slower
// than cycle's fixed 20 ms reset window: on the 15 % climb diag the cycle
// settle needed 146368 steps vs the 64000-step contract (welded, measured
// 2026-11-06); kinetic's energy-adaptive window settled the same pose in
// 966. The switch is a mechanism ruling, not tuning — budget and tolerance
// are untouched.
void test_relaxation_is_kinetic_for_the_rotor_drive() {
    bt::require(profile().equilibrium_relaxation == "kinetic",
                "the rotor drivetrain settles on the kinetic schedule");
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
    bt::require(assist.release_hold_s == .3,
                "the motor runs on for .3 s after the rider eases off");
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters)
[[gnu::noinline]] void assert_injected_key_rejected(
    std::string_view dir_tag, std::string_view table,
    std::string_view injected_line, std::string_view expected_token,
    std::string_view message) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    const std::string source = bt::text(repo_file(
        "examples/research/viewer_physics_welded.toml"));
    const auto anchor = source.find(table);
    bt::require(anchor != std::string::npos,
                "the welded document keeps the target table");
    std::string injected(source);
    injected.insert(anchor + table.size(), injected_line);
    const bt::TemporaryDirectory dir(dir_tag);
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << injected; }
    bool threw = false;
    try {
        (void)cfg::load_physics(path);
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(
            std::string_view(failure.what()).find(expected_token) !=
                std::string_view::npos,
            "the load rejection names the removed key");
    }
    bt::require(threw, message);
}

// Checks that the profile loads every rider section cleanly.

// test_rider_numbers
[[gnu::noinline]] void test_rider_numbers() {
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
    bt::require(shifting.torque_factor_larger_cog == .8 &&
                    shifting.torque_factor_smaller_cog == .3,
                "the shift derate is .8 to a larger cog and .3 to a smaller");
    // torque_factor is gone: a document that still carries the removed
    // single-derate key under [drive.shifting] fails to load.
    assert_injected_key_rejected(
        "welded-legacy-shift-factor", "[drive.shifting]",
        "\ntorque_factor = 0.3", "torque_factor",
        "the schema rejects the removed torque_factor key");
    const auto &climb = profile().seated_climb;
    bt::require(climb.enabled && climb.target_crank_power_w == 250.,
                "seated climb is enabled at a 250 W target");
    bt::require(climb.max_crank_torque_nm == 60.,
                "the crank torque ceiling is 60 Nm");
    bt::require(climb.surge_power_w == 400.,
                "the surge power is 400 W");
    bt::require(climb.min_elbow_flexion_deg == 15.,
                "the torso backward-lean stop is at 15 deg elbow flexion");
    for (const char *rel : {"examples/research/viewer_physics_pneumatic.toml",
                            "examples/research/viewer_physics_distributed.toml"}) {
        const auto other = cfg::load_physics(repo_file(rel));
        bt::require(other.seated_climb.min_elbow_flexion_deg == 15.,
                    "all three profiles set min_elbow_flexion_deg = 15 deg");
    }
    // The soft-start keys are gone: any profile carrying any of them fails
    // to load and names the removed field.
    for (const auto &[dir_tag, key, line] : std::initializer_list<
             std::tuple<std::string_view, std::string_view, std::string_view>>{
             {"welded-legacy-launch-effort-ramp", "launch_effort_ramp_s",
              "\nlaunch_effort_ramp_s = 1.5"},
             {"welded-legacy-launch-min-grade", "launch_min_grade",
              "\nlaunch_min_grade = 0.03"},
             {"welded-legacy-launch-standstill-crank",
              "launch_standstill_crank_rate_rad_s",
              "\nlaunch_standstill_crank_rate_rad_s = 1.0"}}) {
        assert_injected_key_rejected(
            dir_tag, "[seated_climb]", line, key,
            "the schema rejects the removed soft-start key");
    }
    // min_elbow_flexion_deg is a required profile field with no code default:
    // omitting it from [seated_climb] fails to load, and out-of-range values
    // fail validation.
    {
        std::string missing_elbow = bt::text(repo_file(
            "examples/research/viewer_physics_welded.toml"));
        const std::string needle = "min_elbow_flexion_deg = 15.0";
        const auto pos = missing_elbow.find(needle);
        bt::require(pos != std::string::npos,
                    "viewer_physics_welded.toml declares min_elbow_flexion_deg");
        missing_elbow.erase(pos, needle.size());
        const bt::TemporaryDirectory dir("welded-missing-min-elbow");
        const auto path = dir.path() / "physics.toml";
        { std::ofstream out(path); out << missing_elbow; }
        bool threw = false;
        try {
            (void)cfg::load_physics(path);
        } catch (const std::invalid_argument &failure) {
            threw = true;
            bt::require(
                std::string_view(failure.what()).find(
                    "min_elbow_flexion_deg") != std::string_view::npos,
                "the load rejection names min_elbow_flexion_deg");
        }
        bt::require(threw,
                    "omitting min_elbow_flexion_deg from [seated_climb] fails");
    }
    for (const double bad_deg : {0., -5., 95.}) {
        auto mutated = profile();
        mutated.seated_climb.min_elbow_flexion_deg = bad_deg;
        bool threw = false;
        try {
            cfg::post_init(mutated);
        } catch (const std::invalid_argument &failure) {
            threw = true;
            bt::require(
                std::string_view(failure.what()).find(
                    "min_elbow_flexion_deg") != std::string_view::npos,
                "range rejection names min_elbow_flexion_deg");
        }
        bt::require(threw,
                    "out-of-range min_elbow_flexion_deg is rejected");
    }
    bt::require(profile().control_period_s == 0.00125,
                "the leg controller runs every 1.25 ms physics step");
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
    // Joint damping does not exist in the model: a document that still
    // carries the removed joint_passive_damping_nms_rad key fails to load.
    assert_injected_key_rejected(
        "welded-legacy-key", "[articulated]",
        "\njoint_passive_damping_nms_rad = 0.0",
        "joint_passive_damping_nms_rad",
        "the schema rejects joint_passive_damping_nms_rad");
    // The 25 ms activation filter is removed completely (2026-11-07): a
    // document that still carries activation_tau_s under [articulated] fails
    // to load.
    assert_injected_key_rejected(
        "welded-legacy-activation-tau", "[articulated]",
        "\nactivation_tau_s = 0.025", "activation_tau_s",
        "the schema rejects activation_tau_s");
}

// Runs `action`, which must throw std::invalid_argument naming `needle`.
template <typename Action>
void rejects(Action &&action, std::string_view needle, std::string_view what) {
    bool threw = false;
    try {
        std::forward<Action>(action)();
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(std::string_view(failure.what()).find(needle) != std::string_view::npos,
                    std::string(what) + " names " + std::string(needle) + " (" + failure.what() + ")");
    }
    bt::require(threw, std::string(what) + " is rejected");
}

// Loads the shipped welded profile after `edit` changed its text. The two
// rider data files the profile names travel along, so only the edit
// decides whether the load succeeds.
[[gnu::noinline]] cfg::SimulationPhysicsConfig load_edited_profile(
    const std::function<void(std::string &)> &edit) {
    std::string text = bt::text(repo_file("examples/research/viewer_physics_welded.toml"));
    edit(text);
    const bt::TemporaryDirectory dir("welded-edited-profile");
    for (const char *name : {"rider_joint_envelope_anatomical.json", "rider_strength_reference.json"})
        std::filesystem::copy_file(repo_file(std::string("examples/research/") + name), dir.path() / name);
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << text; }
    return cfg::load_physics(path);
}

// Replaces the one occurrence of `from`; a missing or repeated line fails.
void replace_once(std::string &text, std::string_view from, std::string_view to) {
    const auto at = text.find(from);
    bt::require(at != std::string::npos && text.find(from, at + 1) == std::string::npos,
                "the profile text occurs exactly once: " + std::string(from));
    text.replace(at, from.size(), to);
}

// ---- launch part 4 — the calibrated rider tables ------------------------

// The launch part 4 decision-5 table of one rider type: the calibrated
// probe profile's [rider_launch] and [front_lift_reaction] blocks plus the
// part-1/2 fields (grade cap knots, rise speed, shift retry, criterion B).
struct ClimbCapTable {
    std::vector<double> climb_effort_cap_grades;
    std::vector<double> climb_effort_cap_grade_nm;
    double climb_effort_rise_speed_mps{};
    double climb_effort_full_speed_mps{};
};

struct RiderTable {
    cfg::RiderLaunchConfig launch;
    cfg::FrontLiftReactionConfig reaction;
    ClimbCapTable climb_cap;
};

const RiderTable &careful_rider_table() {
    static const RiderTable table{
        .launch{
            .start_cog_grade_breaks = {.03, .08, .13, .18},
            .start_cog_teeth = {28, 39, 45, 39, 39},
            .start_pedal_angle_rad = -.8,
            .start_lean_grades = {0., .15},
            .start_lean_rad = {0., .35},
            .plan_enabled = true,
            .standstill_speed_mps = .1,
            .standstill_crank_rpm = 5.,
            .effort_rate_nm_s = 40.,
            .effort_cap_asphalt_nm = 45.,
            .effort_cap_hardpack_nm = 45.,
            .effort_cap_loose_nm = 35.,
            .effort_cap_wet_nm = 11.,
            .effort_cap_scree_nm = 30.,
            .effort_cap_grades = {0., .05, .08, .12, .13, .15, .2, .22, .25},
            .effort_cap_grade_nm = {26., 14., 11.5, 13., 14., 13., 14., 18., 21.},
            .effort_rise_speed_mps = 2.5,
            .release_torque_grades = {0., .05, .1, .15, .2},
            .release_torque_nm = {0., 7., 7., 7., 7.},
            .brake_release_s = .2,
            .max_hold_s = 2.,
            .upshift_crank_rpm = 72.,
            .shift_retry_s = .25,
            .end_speed_mps = 3.,
            .end_crank_rpm = 60.,
            .end_hold_s = .3,
            .max_launch_s = 6.,
            .handover_s = 1.2,
        },
        .reaction{
            .hand_window_s = .5,
            .hand_lag_s = .5,
            .hand_drop_ratio = .7,
            .pitch_rate_up_rad_s = .105,
            .pitch_rad = .06,
            .decision_s = .28,
            .ease_enabled = true,
            .lean_enabled = true,
            .rear_brake_tap_enabled = false,
            .ease_fraction = .9,
            .ease_rate_per_s = 4.,
            .lean_add_rad = .08,
            .lean_rate_rad_s = .5,
            .clear_s = .65,
        },
        .climb_cap{
            .climb_effort_cap_grades = {0., .05, .10, .15, .20, .22, .25},
            .climb_effort_cap_grade_nm = {38., 25., 22., 25., 30., 30., 32.},
            .climb_effort_rise_speed_mps = 2.5,
            .climb_effort_full_speed_mps = 4.,
        },
    };
    return table;
}

const RiderTable &normal_rider_table() {
    static const RiderTable table{
        .launch{
            .start_cog_grade_breaks = {.03, .08, .13, .18},
            .start_cog_teeth = {24, 33, 39, 33, 39},
            .start_pedal_angle_rad = -.8,
            .start_lean_grades = {0., .15},
            .start_lean_rad = {0., .35},
            .plan_enabled = true,
            .standstill_speed_mps = .1,
            .standstill_crank_rpm = 5.,
            .effort_rate_nm_s = 80.,
            .effort_cap_asphalt_nm = 45.,
            .effort_cap_hardpack_nm = 60.,
            .effort_cap_loose_nm = 45.,
            .effort_cap_wet_nm = 13.,
            .effort_cap_scree_nm = 40.,
            .effort_cap_grades = {0., .05, .08, .1, .12, .13, .15, .2, .22, .25},
            .effort_cap_grade_nm = {38.5, 15., 14.5, 13.2, 14., 15.5, 19.5, 18.5, 21., 22.5},
            .effort_rise_speed_mps = 2.5,
            .release_torque_grades = {0., .05, .1, .15, .2},
            .release_torque_nm = {0., 9., 9., 9., 8.},
            .brake_release_s = .15,
            .max_hold_s = 1.5,
            .upshift_crank_rpm = 82.,
            .shift_retry_s = .25,
            .end_speed_mps = 3.,
            .end_crank_rpm = 60.,
            .end_hold_s = .3,
            .max_launch_s = 6.,
            .handover_s = .9,
        },
        .reaction{
            .hand_window_s = .5,
            .hand_lag_s = .5,
            .hand_drop_ratio = .7,
            .pitch_rate_up_rad_s = .15,
            .pitch_rad = .09,
            .decision_s = .2,
            .ease_enabled = true,
            .lean_enabled = true,
            .rear_brake_tap_enabled = false,
            .ease_fraction = .7,
            .ease_rate_per_s = 5.,
            .lean_add_rad = .15,
            .lean_rate_rad_s = 1.,
            .clear_s = .4,
        },
        .climb_cap{
            .climb_effort_cap_grades = {0., .05, .10, .15, .20, .22, .25},
            .climb_effort_cap_grade_nm = {45., 35., 30., 30., 34., 34., 35.},
            .climb_effort_rise_speed_mps = 2.5,
            .climb_effort_full_speed_mps = 4.,
        },
    };
    return table;
}

const RiderTable &aggressive_rider_table() {
    static const RiderTable table{
        .launch{
            .start_cog_grade_breaks = {.03, .08, .13, .18},
            .start_cog_teeth = {21, 28, 33, 28, 33},
            .start_pedal_angle_rad = -.8,
            .start_lean_grades = {0., .15},
            .start_lean_rad = {0., .35},
            .plan_enabled = true,
            .standstill_speed_mps = .1,
            .standstill_crank_rpm = 5.,
            .effort_rate_nm_s = 200.,
            .effort_cap_asphalt_nm = 45.,
            .effort_cap_hardpack_nm = 60.,
            .effort_cap_loose_nm = 60.,
            .effort_cap_wet_nm = 60.,
            .effort_cap_scree_nm = 60.,
            .effort_cap_grades = {0., .05, .08, .12, .13, .15, .2},
            .effort_cap_grade_nm = {50., 32., 24., 26., 33., 39.5, 35.},
            .effort_rise_speed_mps = 2.5,
            .release_torque_grades = {0., .05, .1, .15, .2},
            .release_torque_nm = {0., 12., 12., 12., 12.},
            .brake_release_s = .1,
            .max_hold_s = 1.,
            .upshift_crank_rpm = 95.,
            .shift_retry_s = .25,
            .end_speed_mps = 3.,
            .end_crank_rpm = 60.,
            .end_hold_s = .3,
            .max_launch_s = 6.,
            .handover_s = .6,
        },
        .reaction{
            .hand_window_s = .5,
            .hand_lag_s = .5,
            .hand_drop_ratio = .7,
            .pitch_rate_up_rad_s = .21,
            .pitch_rad = .12,
            .decision_s = .1,
            .ease_enabled = true,
            .lean_enabled = true,
            .rear_brake_tap_enabled = false,
            .ease_fraction = .5,
            .ease_rate_per_s = 6.,
            .lean_add_rad = .2,
            .lean_rate_rad_s = 1.5,
            .clear_s = .25,
        },
        .climb_cap{
            .climb_effort_cap_grades = {0., .05, .10, .15, .20, .22, .25},
            .climb_effort_cap_grade_nm = {50., 35., 32., 39.5, 35., 35., 35.},
            .climb_effort_rise_speed_mps = 2.5,
            .climb_effort_full_speed_mps = 4.,
        },
    };
    return table;
}

// Field-by-field equality against a decision-5 table; a failed check names
// the file and the member.
[[gnu::noinline]] void require_launch_table(
    const cfg::RiderLaunchConfig &actual, const cfg::RiderLaunchConfig &expected,
    std::string_view who) {
    const auto field = [&](bool equal, std::string_view name) {
        bt::require(equal, std::string(who) + " [rider_launch]." +
                               std::string(name) +
                               " matches the calibrated table");
    };
    field(actual.start_cog_grade_breaks == expected.start_cog_grade_breaks,
          "start_cog_grade_breaks");
    field(actual.start_cog_teeth == expected.start_cog_teeth,
          "start_cog_teeth");
    field(actual.start_pedal_angle_rad == expected.start_pedal_angle_rad,
          "start_pedal_angle_rad");
    field(actual.start_lean_grades == expected.start_lean_grades,
          "start_lean_grades");
    field(actual.start_lean_rad == expected.start_lean_rad, "start_lean_rad");
    field(actual.plan_enabled == expected.plan_enabled, "plan_enabled");
    field(actual.standstill_speed_mps == expected.standstill_speed_mps,
          "standstill_speed_mps");
    field(actual.standstill_crank_rpm == expected.standstill_crank_rpm,
          "standstill_crank_rpm");
    field(actual.effort_rate_nm_s == expected.effort_rate_nm_s,
          "effort_rate_nm_s");
    field(actual.effort_cap_asphalt_nm == expected.effort_cap_asphalt_nm,
          "effort_cap_asphalt_nm");
    field(actual.effort_cap_hardpack_nm == expected.effort_cap_hardpack_nm,
          "effort_cap_hardpack_nm");
    field(actual.effort_cap_loose_nm == expected.effort_cap_loose_nm,
          "effort_cap_loose_nm");
    field(actual.effort_cap_wet_nm == expected.effort_cap_wet_nm,
          "effort_cap_wet_nm");
    field(actual.effort_cap_scree_nm == expected.effort_cap_scree_nm,
          "effort_cap_scree_nm");
    field(actual.effort_cap_grades == expected.effort_cap_grades,
          "effort_cap_grades");
    field(actual.effort_cap_grade_nm == expected.effort_cap_grade_nm,
          "effort_cap_grade_nm");
    field(actual.effort_rise_speed_mps == expected.effort_rise_speed_mps,
          "effort_rise_speed_mps");
    field(actual.release_torque_grades == expected.release_torque_grades,
          "release_torque_grades");
    field(actual.release_torque_nm == expected.release_torque_nm,
          "release_torque_nm");
    field(actual.brake_release_s == expected.brake_release_s,
          "brake_release_s");
    field(actual.max_hold_s == expected.max_hold_s, "max_hold_s");
    field(actual.upshift_crank_rpm == expected.upshift_crank_rpm,
          "upshift_crank_rpm");
    field(actual.shift_retry_s == expected.shift_retry_s, "shift_retry_s");
    field(actual.end_speed_mps == expected.end_speed_mps, "end_speed_mps");
    field(actual.end_crank_rpm == expected.end_crank_rpm, "end_crank_rpm");
    field(actual.end_hold_s == expected.end_hold_s, "end_hold_s");
    field(actual.max_launch_s == expected.max_launch_s, "max_launch_s");
    field(actual.handover_s == expected.handover_s, "handover_s");
}

[[gnu::noinline]] void require_reaction_table(
    const cfg::FrontLiftReactionConfig &actual,
    const cfg::FrontLiftReactionConfig &expected, std::string_view who) {
    const auto field = [&](bool equal, std::string_view name) {
        bt::require(equal, std::string(who) + " [front_lift_reaction]." +
                               std::string(name) +
                               " matches the calibrated table");
    };
    field(actual.hand_window_s == expected.hand_window_s, "hand_window_s");
    field(actual.hand_lag_s == expected.hand_lag_s, "hand_lag_s");
    field(actual.hand_drop_ratio == expected.hand_drop_ratio,
          "hand_drop_ratio");
    field(actual.pitch_rate_up_rad_s == expected.pitch_rate_up_rad_s,
          "pitch_rate_up_rad_s");
    field(actual.pitch_rad == expected.pitch_rad, "pitch_rad");
    field(actual.decision_s == expected.decision_s, "decision_s");
    field(actual.ease_enabled == expected.ease_enabled, "ease_enabled");
    field(actual.lean_enabled == expected.lean_enabled, "lean_enabled");
    field(actual.rear_brake_tap_enabled == expected.rear_brake_tap_enabled,
          "rear_brake_tap_enabled");
    field(actual.ease_fraction == expected.ease_fraction, "ease_fraction");
    field(actual.ease_rate_per_s == expected.ease_rate_per_s,
          "ease_rate_per_s");
    field(actual.lean_add_rad == expected.lean_add_rad, "lean_add_rad");
    field(actual.lean_rate_rad_s == expected.lean_rate_rad_s,
          "lean_rate_rad_s");
    field(actual.clear_s == expected.clear_s, "clear_s");
}

[[gnu::noinline]] void require_seated_climb_cap(
    const cfg::SeatedClimbConfig &actual, const ClimbCapTable &expected,
    std::string_view who) {
    const auto field = [&](bool equal, std::string_view name) {
        bt::require(equal, std::string(who) + " [seated_climb]." +
                               std::string(name) +
                               " matches the calibrated table");
    };
    field(actual.climb_effort_cap_grades == expected.climb_effort_cap_grades,
          "climb_effort_cap_grades");
    field(actual.climb_effort_cap_grade_nm == expected.climb_effort_cap_grade_nm,
          "climb_effort_cap_grade_nm");
    field(actual.climb_effort_rise_speed_mps ==
              expected.climb_effort_rise_speed_mps,
          "climb_effort_rise_speed_mps");
    field(actual.climb_effort_full_speed_mps ==
              expected.climb_effort_full_speed_mps,
          "climb_effort_full_speed_mps");
}

// The welded profile carries the calibrated normal rider's launch table
// (launch part 4, decision 5; launch part 4b, decision 6): the start cog steps
// down by grade, the pedal leads at -0.8 rad and the torso leans up to .35
// rad. The built-in default profile keeps its own 34/24 drivetrain with the
// plan off.
[[gnu::noinline]] void test_launch_table_is_the_normal_rider() {
    require_launch_table(profile().rider_launch, normal_rider_table().launch,
                         "viewer_physics_welded.toml");
    const auto defaults = cfg::default_physics();
    bt::require(!defaults.rider_launch.plan_enabled &&
                    defaults.rider_launch.start_cog_teeth ==
                        std::vector<int>{24} &&
                    defaults.drive.gearing.front_teeth == 34 &&
                    defaults.drive.gearing.rear_teeth == 24,
                "the built-in profile keeps its own 34/24 with the plan off");
}

// The three research profiles carry the normal rider; the two rider-type
// files carry their own calibrated tables — five files, five loads, every
// field of the two sections checked against decision 5 (launch part 4,
// step 1).
[[gnu::noinline]] void test_all_five_profiles_carry_the_calibrated_tables() {
    const std::array cases = std::to_array<
        std::pair<const char *, const RiderTable *>>({
        {"viewer_physics_welded.toml", &normal_rider_table()},
        {"viewer_physics_distributed.toml", &normal_rider_table()},
        {"viewer_physics_pneumatic.toml", &normal_rider_table()},
        {"rider_type_careful.toml", &careful_rider_table()},
        {"rider_type_aggressive.toml", &aggressive_rider_table()},
    });
    for (const auto &[file, expected] : cases) {
        const auto loaded = cfg::load_physics(
            repo_file(std::string("examples/research/") + file));
        require_launch_table(loaded.rider_launch, expected->launch, file);
        require_reaction_table(loaded.front_lift_reaction, expected->reaction,
                               file);
        require_seated_climb_cap(loaded.seated_climb, expected->climb_cap, file);
    }
}

bool is_climb_cap_key(std::string_view key) {
    return key == "climb_effort_cap_grades" ||
           key == "climb_effort_cap_grade_nm" ||
           key == "climb_effort_rise_speed_mps" ||
           key == "climb_effort_full_speed_mps";
}

// Profile text without comments and blank lines, dropping [rider_launch],
// [front_lift_reaction], and the four climb_effort_cap_* keys in
// [seated_climb].
[[nodiscard]] std::vector<std::string>
text_outside_rider_sections(std::string_view text) {
    std::vector<std::string> lines;
    bool in_rider_section = false;
    bool in_seated_climb = false;
    std::size_t at = 0;
    while (at <= text.size()) {
        const auto eol = text.find('\n', at);
        const auto end =
            eol == std::string_view::npos ? text.size() : eol;
        std::string line(text.substr(at, end - at));
        // The research profiles carry no '#' inside string values.
        if (const auto hash = line.find('#'); hash != std::string::npos)
            line.erase(hash);
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos) {
            if (eol == std::string_view::npos) break;
            at = eol + 1;
            continue;
        }
        line.erase(0, first);
        line.erase(line.find_last_not_of(" \t\r") + 1);
        if (line.front() == '[') {
            in_rider_section = line == "[rider_launch]" ||
                               line == "[front_lift_reaction]";
            in_seated_climb = line == "[seated_climb]";
        } else if (!in_rider_section) {
            bool skip_line = false;
            if (in_seated_climb) {
                const auto eq = line.find('=');
                if (eq != std::string::npos) {
                    auto key = std::string_view(line).substr(0, eq);
                    while (!key.empty() &&
                           (key.back() == ' ' || key.back() == '\t')) {
                        key.remove_suffix(1);
                    }
                    skip_line = is_climb_cap_key(key);
                }
            }
            if (!skip_line)
                lines.push_back(line);
        }
        if (eol == std::string_view::npos) break;
        at = eol + 1;
    }
    return lines;
}

// rider_type_{careful,aggressive}.toml are full copies of
// viewer_physics_welded.toml that differ only in [rider_launch],
// [front_lift_reaction], and the four climb_effort_cap_* keys in
// [seated_climb]: the parsed SimulationPhysicsConfig is identical once those
// and the two computed members are dropped, and the remaining text matches
// without comments and blank lines.
[[gnu::noinline]] void test_rider_type_files_differ_only_in_the_two_sections() {
    const auto canonical_outside_sections = [](
        const cfg::SimulationPhysicsConfig &config) {
        auto document = cfg::detail::plain_document(cfg::to_value(config));
        // The launch table computes these two at load; the comparison
        // leaves them out along with the sections that produce them.
        cfg::detail::erase_computed_members(document);
        auto &root = std::get<bike::io::Value::Object>(document.data);
        std::erase_if(
            root,
            [](const auto &member) {
                return member.first == "rider_launch" ||
                       member.first == "front_lift_reaction";
            });
        for (auto &member : root) {
            if (member.first == "seated_climb") {
                std::erase_if(
                    std::get<bike::io::Value::Object>(member.second.data),
                    [](const auto &field) {
                        return is_climb_cap_key(field.first);
                    });
            }
        }
        return bike::io::canonical_json(document);
    };
    const auto welded_lines = text_outside_rider_sections(
        bt::text(repo_file("examples/research/viewer_physics_welded.toml")));
    const std::string welded_canonical = canonical_outside_sections(profile());
    for (const char *name :
         {"rider_type_careful.toml", "rider_type_aggressive.toml"}) {
        const auto path =
            repo_file(std::string("examples/research/") + name);
        const auto config = cfg::load_physics(path);
        bt::require(
            canonical_outside_sections(config) == welded_canonical,
            std::string(name) + " parses to the welded profile outside "
                                "the two rider sections");
        const auto lines = text_outside_rider_sections(bt::text(path));
        std::string first_difference;
        for (std::size_t i = 0;
             i < std::max(lines.size(), welded_lines.size()); ++i) {
            const std::string_view own =
                i < lines.size() ? std::string_view(lines[i]) : "<eof>";
            const std::string_view base =
                i < welded_lines.size() ? std::string_view(welded_lines[i])
                                        : "<eof>";
            if (own != base) {
                first_difference = "line " + std::to_string(i) + ": '" +
                                   std::string(own) + "' vs '" +
                                   std::string(base) + "'";
                break;
            }
        }
        bt::require(first_difference.empty(),
                    std::string(name) +
                        " matches the welded profile outside the two "
                        "rider sections (" + first_difference + ")");
    }
}

// Every rider_launch key and the section itself are required: dropping
// one fails the load and the error names it (no code default).
[[gnu::noinline]] void test_launch_table_keys_are_required() {
    for (const char *line : {"start_cog_grade_breaks = [0.03, 0.08, 0.13, 0.18]\n",
                             "start_cog_teeth = [24, 33, 39, 33, 39]\n",
                             "start_pedal_angle_rad = -0.8\n", "start_lean_grades = [0.0, 0.15]\n",
                             "start_lean_rad = [0.0, 0.35]\n", "plan_enabled = true\n",
                             "standstill_speed_mps = 0.1\n", "standstill_crank_rpm = 5.0\n",
                             "effort_rate_nm_s = 80.0\n", "effort_cap_asphalt_nm = 45.0\n",
                             "effort_cap_hardpack_nm = 60.0\n", "effort_cap_loose_nm = 45.0\n",
                             "effort_cap_wet_nm = 13.0\n", "effort_cap_scree_nm = 40.0\n",
                             "effort_cap_grades = [0.0, 0.05, 0.08, 0.1, 0.12, 0.13, 0.15, 0.2, 0.22, 0.25]\n",
                             "effort_cap_grade_nm = [38.5, 15.0, 14.5, 13.2, 14.0, 15.5, 19.5, 18.5, 21.0, 22.5]\n",
                             "effort_rise_speed_mps = 2.5\n",
                             "release_torque_grades = [0.0, 0.05, 0.1, 0.15, 0.2]\n",
                             "release_torque_nm = [0.0, 9.0, 9.0, 9.0, 8.0]\n",
                             "brake_release_s = 0.15\n", "max_hold_s = 1.5\n", "upshift_crank_rpm = 82.0\n",
                             "shift_retry_s = 0.25\n",
                             "end_speed_mps = 3.0\n", "end_crank_rpm = 60.0\n", "end_hold_s = 0.3\n",
                             "max_launch_s = 6.0\n", "handover_s = 0.9\n"}) {
        const std::string key(std::string_view(line).substr(0, std::string_view(line).find(' ')));
        rejects([&] {
            (void)load_edited_profile([&](std::string &t) {
                const auto from = t.find("[rider_launch]\n");
                bt::require(from != std::string::npos, "the profile has the launch block");
                const std::string needle = "\n" + std::string(line);
                const auto at = t.find(needle, from);
                bt::require(at != std::string::npos && t.find(needle, at + 1) == std::string::npos,
                            "one " + key + " line inside the launch block");
                t.erase(at + 1, needle.size() - 1);
            });
        }, key, "a profile without " + key);
    }
    rejects([] {
        (void)load_edited_profile([](std::string &t) {
            const auto from = t.find("[rider_launch]\n");
            const std::string_view last = "handover_s = 0.9\n";
            const auto to = t.find(last, from);
            bt::require(from != std::string::npos && to != std::string::npos, "the profile has the launch block");
            t.erase(from, to + last.size() - from);
        });
    }, "rider_launch", "a profile without the rider_launch section");
}

// Rules that need other records: a start cog must be on the cassette while
// shifting runs (any cog when it does not), and a start lean must stay
// inside the seated posture bounds of [seated_climb].
[[gnu::noinline]] void test_launch_table_cross_record_rules() {
    auto config = profile();
    config.rider_launch.start_cog_teeth = {50, 50, 50, 50, 50};
    rejects([&] { cfg::validate(config); }, "start_cog_teeth", "a start cog off the cassette");
    config.drive.shifting.enabled = false;
    cfg::validate(config);   // control: any cog >= 3 without shifting
    config = profile();
    config.rider_launch.start_lean_rad = {config.seated_climb.max_forward_lean_rad + 1e-3,
                                        config.seated_climb.max_forward_lean_rad + 1e-3};
    rejects([&] { cfg::validate(config); }, "start_lean_rad", "a lean beyond the forward bound");
    config.rider_launch.start_lean_rad = {-config.seated_climb.max_backward_lean_rad - 1e-3,
                                        -config.seated_climb.max_backward_lean_rad - 1e-3};
    rejects([&] { cfg::validate(config); }, "start_lean_rad", "a lean beyond the backward bound");
    config.rider_launch.start_lean_rad = {config.seated_climb.max_forward_lean_rad,
                                        config.seated_climb.max_forward_lean_rad};
    cfg::validate(config);   // control: the bound itself is a valid lean
}

// The welded profile carries the calibrated normal rider's front-lift
// reaction with the ease and lean actions on (launch part 4, decision 5).
[[gnu::noinline]] void test_reaction_table_is_the_calibrated_normal_rider() {
    require_reaction_table(profile().front_lift_reaction,
                           normal_rider_table().reaction,
                           "viewer_physics_welded.toml");
}

// Every [front_lift_reaction] key and the section are required; the period
// grid and the forward bound are checked against [seated_climb].
[[gnu::noinline]] void test_reaction_keys_and_context_rules() {
    for (const char *line : {"hand_window_s = 0.5\n", "hand_lag_s = 0.5\n", "hand_drop_ratio = 0.7\n",
                             "pitch_rate_up_rad_s = 0.15\n",
                             "pitch_rad = 0.09\n", "decision_s = 0.2\n", "ease_enabled = true\n",
                             "lean_enabled = true\n", "rear_brake_tap_enabled = false\n",
                             "ease_fraction = 0.7\n", "ease_rate_per_s = 5.0\n", "lean_add_rad = 0.15\n",
                             "lean_rate_rad_s = 1.0\n", "clear_s = 0.4\n"}) {
        const std::string key(std::string_view(line).substr(0, std::string_view(line).find(' ')));
        // Remove the key inside [front_lift_reaction] only: lean_rate_rad_s
        // also exists in [seated_climb], so replace_once would refuse the
        // profile (review-2 I2). The leading newline keeps a longer key
        // ending in the same text from matching.
        rejects([&] {
            (void)load_edited_profile([&](std::string &t) {
                const auto from = t.find("[front_lift_reaction]\n");
                bt::require(from != std::string::npos, "the profile has the reaction block");
                const std::string needle = "\n" + std::string(line);
                const auto at = t.find(needle, from);
                bt::require(at != std::string::npos && t.find(needle, at + 1) == std::string::npos,
                            "one " + key + " line inside the reaction block");
                t.erase(at + 1, needle.size() - 1);
            });
        }, key, "a profile without " + key);
    }
    rejects([] {
        (void)load_edited_profile([](std::string &t) {
            const auto from = t.find("[front_lift_reaction]\n");
            const std::string_view last = "clear_s = 0.4\n";
            const auto to = t.find(last, from);
            bt::require(from != std::string::npos && to != std::string::npos, "the profile has the reaction block");
            t.erase(from, to + last.size() - from);
        });
    }, "front_lift_reaction", "a profile without [front_lift_reaction]");
    const auto config = std::make_unique<cfg::SimulationPhysicsConfig>(profile());
    config->front_lift_reaction.decision_s = .1 + .0005;
    rejects([&] { cfg::validate(*config); }, "decision_s", "a decision time off the period grid");
    *config = profile();
    config->front_lift_reaction.lean_add_rad = config->seated_climb.max_forward_lean_rad + .01;
    rejects([&] { cfg::validate(*config); }, "lean_add_rad", "a lean add beyond the forward bound");
    cfg::validate(profile());   // control
}

// Rejects one profile-text edit; noinline keeps the loaded config's return
// slot in this frame so the caller can hold further checks.
template <typename Edit>
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (needle, what) mirrors the rejects() signature below
[[gnu::noinline]] void rejects_profile_edit(Edit &&edit, std::string_view needle, std::string_view what) {
    rejects([&] { (void)load_edited_profile(std::forward<Edit>(edit)); }, needle, what);
}

// The two computed drive members are never authored: a profile that still
// names either one is rejected, and the error points at the table field
// that replaced it. An override document is held to the same rule.
[[gnu::noinline]] void test_computed_drive_members_are_not_authored() {
    rejects_profile_edit(
        [](std::string &t) { replace_once(t, "[drive.gearing]\n", "[drive.gearing]\nrear_teeth = 51\n"); },
        "start_cog_teeth", "a profile that authors drive.gearing.rear_teeth");
    rejects_profile_edit(
        [](std::string &t) { replace_once(t, "[drive]\n", "[drive]\ncrank_phase_rad = 0.0\n"); },
        "start_pedal_angle_rad", "a profile that authors drive.crank_phase_rad");
    using bike::io::Number;
    using bike::io::Value;
    const Value cog{Value::Object{{"rear_teeth", Value{Number{"39"}}}}};
    const Value gearing{Value::Object{{"gearing", cog}}};
    const Value gear{Value::Object{{"drive", gearing}}};
    rejects([&] { (void)cfg::resolve_physics(gear, profile()); },
            "start_cog_teeth", "an override that authors the rear cog");
}

// Loading resolves the computed members for a flat start from the
// document's own table: the shipped profile carries the normal rider's
// 24T cog at -0.8 rad, and a stepwise table puts grade 0 on the cog of
// its middle interval. An override of the table re-resolves them after
// the merge.
[[gnu::noinline]] void test_load_resolves_a_flat_start() {
    bt::require(profile().drive.gearing.rear_teeth == 24 &&
                    profile().drive.crank_phase_rad == -.8,
                "the shipped profile resolves to the flat-start cog and pedal angle");
    const auto stepwise = load_edited_profile([](std::string &t) {
        replace_once(t, "start_cog_grade_breaks = [0.03, 0.08, 0.13, 0.18]",
                     "start_cog_grade_breaks = [-0.01, 0.05]");
        replace_once(t, "start_cog_teeth = [24, 33, 39, 33, 39]",
                     "start_cog_teeth = [21, 39, 51]");
        replace_once(t, "start_pedal_angle_rad = -0.8",
                     "start_pedal_angle_rad = 0.5");
    });
    bt::require(stepwise.drive.gearing.rear_teeth == 39 && stepwise.drive.crank_phase_rad == .5,
                "grade 0 takes the middle cog and the table's angle");
    using bike::io::Value;
    const Value teeth{Value::Array{Value{bike::io::Number{"45"}}}};
    const Value launch{Value::Object{
        {"start_cog_teeth", teeth}, {"start_cog_grade_breaks", Value{Value::Array{}}}}};
    const Value table{Value::Object{{"rider_launch", launch}}};
    bt::require(cfg::resolve_physics(table, profile()).drive.gearing.rear_teeth == 45,
                "an overridden table re-resolves the start cog");
}

// A written profile authors only the table (round trip normalized to a
// flat start): write_physics_toml leaves the two computed members out, the
// text loads back with the same table, and the computed members come back
// as the table's flat start — for a stepwise table too.
[[gnu::noinline]] void test_written_profile_authors_only_the_table() {
    const auto stepwise = std::make_unique<cfg::SimulationPhysicsConfig>(profile());
    stepwise->rider_launch.start_cog_grade_breaks = {-.01, .05};
    stepwise->rider_launch.start_cog_teeth = {21, 39, 51};
    stepwise->drive.gearing.rear_teeth = 51;   // a prepared value for a steep start
    const auto text = cfg::write_physics_toml(*stepwise);
    bt::require(text.find("rear_teeth") == std::string::npos &&
                    text.find("crank_phase_rad") == std::string::npos,
                "the writer leaves out the computed members");
    const bt::TemporaryDirectory dir("welded-written-profile");
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << text; }
    const auto reread = cfg::load_physics(path);
    bt::require(reread.rider_launch.start_cog_teeth == stepwise->rider_launch.start_cog_teeth &&
                    reread.rider_launch.start_cog_grade_breaks == stepwise->rider_launch.start_cog_grade_breaks,
                "the written table loads back unchanged");
    bt::require(reread.drive.gearing.rear_teeth == 39 && reread.drive.crank_phase_rad == -.8,
                "the computed members come back as the flat start");
}

// A recorded (resolved) physics document keeps its prepared members: the
// reader for recordings does not re-resolve them. A document without the
// launch table — a recording from before part 2 — is rejected and named.
[[gnu::noinline]] void test_recorded_document_keeps_the_prepared_start() {
    const auto prepared = std::make_unique<cfg::SimulationPhysicsConfig>(profile());
    prepared->rider_launch.start_cog_grade_breaks = {.03};
    prepared->rider_launch.start_cog_teeth = {24, 39};
    prepared->rider_launch.start_pedal_angle_rad = .6;
    prepared->drive.gearing.rear_teeth = 39;   // what setup prepares on a 4 % start
    prepared->drive.crank_phase_rad = .6;
    const auto recorded = cfg::detail::plain_document(cfg::to_value(*prepared));
    const auto rebuilt = cfg::detail::construct_config(recorded);
    bt::require(rebuilt.drive.gearing.rear_teeth == 39 && rebuilt.drive.crank_phase_rad == .6,
                "the recorded prepared start survives the recording reader");
    auto old = recorded;
    std::erase_if(std::get<bike::io::Value::Object>(old.data),
                  [](const auto &member) { return member.first == "rider_launch"; });
    rejects([&] { (void)cfg::detail::construct_config(old); }, "rider_launch",
            "a recorded document from before the launch table");
}

// Launch part 4b: both settle crank-hold gains are required profile keys
// (omitting either fails load_physics and names the key) and strictly
// positive finite scalars (the unset -1 sentinel, 0, negative, NaN, and Inf
// all fail post_init naming the field).
[[gnu::noinline]] void test_crank_hold_gains_are_required_and_positive() {
    for (const char *line : {"equilibrium_crank_hold_kp_nm_rad = 20000.0\n",
                             "equilibrium_crank_hold_kd_nms_rad = 88.0\n"}) {
        const std::string key(std::string_view(line).substr(0, std::string_view(line).find(' ')));
        rejects([&] { (void)load_edited_profile([&](std::string &t) { replace_once(t, line, ""); }); },
                key, "a profile without " + key);
    }
    constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const auto fresh = std::make_unique<cfg::SimulationPhysicsConfig>();
    for (const auto &[field, name] : {
             std::pair{&cfg::SimulationPhysicsConfig::equilibrium_crank_hold_kp_nm_rad,
                       "equilibrium_crank_hold_kp_nm_rad"},
             std::pair{&cfg::SimulationPhysicsConfig::equilibrium_crank_hold_kd_nms_rad,
                       "equilibrium_crank_hold_kd_nms_rad"}}) {
        for (const double bad : {fresh.get()->*field, 0., -10., kNan, kInf}) {
            const auto mutated = std::make_unique<cfg::SimulationPhysicsConfig>(profile());
            mutated.get()->*field = bad;
            rejects([&] { cfg::post_init(*mutated); }, name,
                    std::string("out-of-range ") + name);
        }
    }
    const auto valid = std::make_unique<cfg::SimulationPhysicsConfig>(profile());
    cfg::post_init(*valid);
}

// Queue Item 2: the four climb_effort_cap_* keys in [seated_climb] are
// required profile keys with no code default and validated domains.
[[gnu::noinline]] void test_seated_climb_cap_keys_and_domains() {
    for (const char *line : {
             "climb_effort_cap_grades = [0.0, 0.05, 0.10, 0.15, 0.20, 0.22, 0.25]\n",
             "climb_effort_cap_grade_nm = [45.0, 35.0, 30.0, 30.0, 34.0, 34.0, 35.0]\n",
             "climb_effort_rise_speed_mps = 2.5\n",
             "climb_effort_full_speed_mps = 4.0\n"}) {
        const std::string key(std::string_view(line).substr(0, std::string_view(line).find(' ')));
        rejects([&] { (void)load_edited_profile([&](std::string &t) { replace_once(t, line, ""); }); },
                key, "a profile without " + key);
    }
    const auto check_bad = [](auto mutate, std::string_view needle, std::string_view label) {
        const auto mutated = std::make_unique<cfg::SimulationPhysicsConfig>(profile());
        mutate(*mutated);
        rejects([&] { cfg::post_init(*mutated); }, needle, label);
    };
    check_bad([](cfg::SimulationPhysicsConfig &c) {
        c.seated_climb.climb_effort_cap_grades = {0.1, 0.05};
        c.seated_climb.climb_effort_cap_grade_nm = {30., 35.};
    }, "climb_effort_cap_grades", "non-ascending climb_effort_cap_grades");
    check_bad([](cfg::SimulationPhysicsConfig &c) {
        c.seated_climb.climb_effort_cap_grade_nm[0] = 0.;
    }, "climb_effort_cap_grade_nm", "non-positive climb_effort_cap_grade_nm");
    check_bad([](cfg::SimulationPhysicsConfig &c) {
        c.seated_climb.climb_effort_rise_speed_mps = -0.1;
    }, "climb_effort_rise_speed_mps", "negative climb_effort_rise_speed_mps");
    check_bad([](cfg::SimulationPhysicsConfig &c) {
        c.seated_climb.climb_effort_full_speed_mps =
            c.seated_climb.climb_effort_rise_speed_mps - 0.1;
    }, "climb_effort_full_speed_mps", "climb_effort_full_speed_mps < rise");
}

} // namespace

int main() {
    return bt::run([] {
        test_motor_clutch_topology_is_the_default();
        test_fixture_copies_carry_the_research_clutch();
        test_rotor_inertia_is_the_audit_value();
        test_relaxation_is_kinetic_for_the_rotor_drive();
        test_assist_is_bosch_turbo_with_the_declared_envelope();
        test_rider_numbers();
        test_seated_climb_cap_keys_and_domains();
        test_launch_table_is_the_normal_rider();
        test_all_five_profiles_carry_the_calibrated_tables();
        test_rider_type_files_differ_only_in_the_two_sections();
        test_launch_table_keys_are_required();
        test_launch_table_cross_record_rules();
        test_reaction_table_is_the_calibrated_normal_rider();
        test_reaction_keys_and_context_rules();
        test_computed_drive_members_are_not_authored();
        test_load_resolves_a_flat_start();
        test_written_profile_authors_only_the_table();
        test_recorded_document_keeps_the_prepared_start();
        test_crank_hold_gains_are_required_and_positive();
    });
}
