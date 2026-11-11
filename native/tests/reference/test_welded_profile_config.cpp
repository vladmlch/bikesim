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
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
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

// The launch fields are required like min_elbow_flexion_deg — no code
// default, no legacy reader: a document that drops one fails to load and
// the rejection names the key.
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
[[gnu::noinline]] void assert_launch_field_required(
    std::string_view launch_key_line,
    std::string_view launch_key_name) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    std::string missing_launch = bt::text(repo_file(
        "examples/research/viewer_physics_welded.toml"));
    const auto pos = missing_launch.find(launch_key_line);
    bt::require(pos != std::string::npos,
                "viewer_physics_welded.toml declares the launch field");
    missing_launch.erase(pos, launch_key_line.size());
    const bt::TemporaryDirectory dir("welded-missing-launch");
    const auto path = dir.path() / "physics.toml";
    { std::ofstream out(path); out << missing_launch; }
    bool threw = false;
    try {
        (void)cfg::load_physics(path);
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(
            std::string_view(failure.what()).find(launch_key_name) !=
                std::string_view::npos,
            "the load rejection names the missing launch field");
    }
    bt::require(threw,
                "omitting a launch field from [seated_climb] fails");
}

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
    // The uphill-standstill launch ramp (2026-11-10 experiment, user
    // decision): the values decided there are the profile values.
    bt::require(climb.launch_effort_ramp_s == 1.5 &&
                    climb.launch_min_grade == .03 &&
                    climb.launch_standstill_crank_rate_rad_s == 1.,
                "the launch ramp is 1.5 s from a 3% grade at <=1 rad/s");
    for (const char *rel : {"examples/research/viewer_physics_pneumatic.toml",
                            "examples/research/viewer_physics_distributed.toml"}) {
        const auto other = cfg::load_physics(repo_file(rel));
        bt::require(other.seated_climb.min_elbow_flexion_deg == 15.,
                    "all three profiles set min_elbow_flexion_deg = 15 deg");
        bt::require(other.seated_climb.launch_effort_ramp_s == 1.5 &&
                        other.seated_climb.launch_min_grade == .03 &&
                        other.seated_climb.launch_standstill_crank_rate_rad_s ==
                            1.,
                    "all three profiles set the launch ramp fields");
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
    for (const auto &[line, name] : std::initializer_list<
             std::pair<std::string_view, std::string_view>>{
             {"launch_effort_ramp_s = 1.5", "launch_effort_ramp_s"},
             {"launch_min_grade = 0.03", "launch_min_grade"},
             {"launch_standstill_crank_rate_rad_s = 1.0",
              "launch_standstill_crank_rate_rad_s"}})
        assert_launch_field_required(line, name);
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

// The welded profile carries today's start as its launch table: the 51T
// cog at every grade, the pedal at angle 0 and no lean. The built-in
// default profile keeps its own 34/24 drivetrain.
[[gnu::noinline]] void test_launch_table_reproduces_todays_start() {
    const auto &launch = profile().rider_launch;
    bt::require(launch.start_cog_grade_breaks.empty() &&
                    launch.start_cog_teeth == std::vector<int>{51},
                "one 51T start cog");
    bt::require(launch.start_pedal_angle_rad == 0., "pedal angle 0");
    bt::require(launch.start_lean_grades == std::vector<double>{0.} &&
                    launch.start_lean_rad == std::vector<double>{0.},
                "no start lean");
    const auto defaults = cfg::default_physics();
    bt::require(defaults.rider_launch.start_cog_teeth == std::vector<int>{24} &&
                    defaults.drive.gearing.front_teeth == 34 &&
                    defaults.drive.gearing.rear_teeth == 24,
                "the built-in profile starts on 34/24");
}

// Every [rider_launch] key and the section itself are required: dropping
// one fails the load and the error names it (no code default).
[[gnu::noinline]] void test_launch_table_keys_are_required() {
    for (const char *line : {"start_cog_grade_breaks = []\n", "start_cog_teeth = [51]\n",
                             "start_pedal_angle_rad = 0.0\n", "start_lean_grades = [0.0]\n",
                             "start_lean_rad = [0.0]\n"}) {
        const std::string key(std::string_view(line).substr(0, std::string_view(line).find(' ')));
        rejects([&] { (void)load_edited_profile([&](std::string &t) { replace_once(t, line, ""); }); },
                key, "a profile without " + key);
    }
    rejects([] {
        (void)load_edited_profile([](std::string &t) {
            const auto from = t.find("[rider_launch]\n");
            const std::string_view last = "start_lean_rad = [0.0]\n";
            const auto to = t.find(last, from);
            bt::require(from != std::string::npos && to != std::string::npos, "the profile has the launch block");
            t.erase(from, to + last.size() - from);
        });
    }, "rider_launch", "a profile without [rider_launch]");
}

// Rules that need other records: a start cog must be on the cassette while
// shifting runs (any cog when it does not), and a start lean must stay
// inside the seated posture bounds of [seated_climb].
[[gnu::noinline]] void test_launch_table_cross_record_rules() {
    auto config = profile();
    config.rider_launch.start_cog_teeth = {50};
    rejects([&] { cfg::validate(config); }, "start_cog_teeth", "a start cog off the cassette");
    config.drive.shifting.enabled = false;
    cfg::validate(config);   // control: any cog >= 3 without shifting
    config = profile();
    config.rider_launch.start_lean_rad = {config.seated_climb.max_forward_lean_rad + 1e-3};
    rejects([&] { cfg::validate(config); }, "start_lean_rad", "a lean beyond the forward bound");
    config.rider_launch.start_lean_rad = {-config.seated_climb.max_backward_lean_rad - 1e-3};
    rejects([&] { cfg::validate(config); }, "start_lean_rad", "a lean beyond the backward bound");
    config.rider_launch.start_lean_rad = {config.seated_climb.max_forward_lean_rad};
    cfg::validate(config);   // control: the bound itself is a valid lean
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
        test_launch_table_reproduces_todays_start();
        test_launch_table_keys_are_required();
        test_launch_table_cross_record_rules();
    });
}
