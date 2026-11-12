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

// Every rider_launch key and the section itself are required: dropping
// one fails the load and the error names it (no code default).
[[gnu::noinline]] void test_launch_table_keys_are_required() {
    for (const char *line : {"start_cog_grade_breaks = []\n", "start_cog_teeth = [51]\n",
                             "start_pedal_angle_rad = 0.0\n", "start_lean_grades = [0.0]\n",
                             "start_lean_rad = [0.0]\n", "plan_enabled = false\n",
                             "standstill_speed_mps = 0.1\n", "standstill_crank_rpm = 5.0\n",
                             "effort_rate_nm_s = 40.0\n", "effort_cap_asphalt_nm = 60.0\n",
                             "effort_cap_hardpack_nm = 60.0\n", "effort_cap_loose_nm = 60.0\n",
                             "effort_cap_wet_nm = 60.0\n", "effort_cap_scree_nm = 60.0\n",
                             "effort_cap_grades = [0.0, 0.05, 0.08, 0.1, 0.12, 0.13, 0.15, 0.2]\n",
                             "effort_cap_grade_nm = [38.5, 18.0, 14.5, 13.2, 14.0, 15.5, 19.5, 18.5]\n",
                             "effort_rise_speed_mps = 2.5\n",
                             "release_torque_grades = [0.0, 0.2]\n", "release_torque_nm = [0.0, 30.0]\n",
                             "brake_release_s = 0.15\n", "max_hold_s = 1.5\n", "upshift_crank_rpm = 82.0\n",
                             "shift_retry_s = 0.25\n",
                             "end_speed_mps = 3.0\n", "end_crank_rpm = 60.0\n", "end_hold_s = 0.3\n",
                             "max_launch_s = 6.0\n", "handover_s = 0.9\n"}) {
        const std::string key(std::string_view(line).substr(0, std::string_view(line).find(' ')));
        rejects([&] { (void)load_edited_profile([&](std::string &t) { replace_once(t, line, ""); }); },
                key, "a profile without " + key);
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

// The welded profile carries the switched-off reaction with the
// preliminary numbers of the inventory.
[[gnu::noinline]] void test_reaction_table_is_switched_off() {
    const auto &r = profile().front_lift_reaction;
    bt::require(!r.ease_enabled && !r.lean_enabled && !r.rear_brake_tap_enabled, "both actions off");
    bt::require(r.hand_window_s == .5 && r.hand_lag_s == .5 && r.hand_drop_ratio == .7 && r.pitch_rate_up_rad_s == .5 &&
                    r.pitch_rad == .17 && r.decision_s == .1 && r.ease_fraction == .7 &&
                    r.ease_rate_per_s == 5. && r.lean_add_rad == .1 && r.lean_rate_rad_s == .5 && r.clear_s == .3,
                "the preliminary numbers");
}

// Every [front_lift_reaction] key and the section are required; the period
// grid and the forward bound are checked against [seated_climb].
[[gnu::noinline]] void test_reaction_keys_and_context_rules() {
    for (const char *line : {"hand_window_s = 0.5\n", "hand_lag_s = 0.5\n", "hand_drop_ratio = 0.7\n", "pitch_rate_up_rad_s = 0.5\n",
                             "pitch_rad = 0.17\n", "decision_s = 0.1\n", "ease_enabled = false\n",
                             "lean_enabled = false\n", "rear_brake_tap_enabled = false\n",
                             "ease_fraction = 0.7\n", "ease_rate_per_s = 5.0\n", "lean_add_rad = 0.1\n",
                             "lean_rate_rad_s = 0.5\n", "clear_s = 0.3\n"}) {
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
            const std::string_view last = "clear_s = 0.3\n";
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
// document's own table: the shipped profile carries 51T at angle 0, and a
// stepwise table puts grade 0 on the cog of its middle interval. An
// override of the table re-resolves them after the merge.
[[gnu::noinline]] void test_load_resolves_a_flat_start() {
    bt::require(profile().drive.gearing.rear_teeth == 51 && profile().drive.crank_phase_rad == 0.,
                "the shipped profile resolves to 51T at angle 0");
    const auto stepwise = load_edited_profile([](std::string &t) {
        replace_once(t, "start_cog_grade_breaks = []", "start_cog_grade_breaks = [-0.01, 0.05]");
        replace_once(t, "start_cog_teeth = [51]", "start_cog_teeth = [21, 39, 51]");
        replace_once(t, "start_pedal_angle_rad = 0.0", "start_pedal_angle_rad = 0.5");
    });
    bt::require(stepwise.drive.gearing.rear_teeth == 39 && stepwise.drive.crank_phase_rad == .5,
                "grade 0 takes the middle cog and the table's angle");
    using bike::io::Value;
    const Value teeth{Value::Array{Value{bike::io::Number{"45"}}}};
    const Value launch{Value::Object{{"start_cog_teeth", teeth}}};
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
    bt::require(reread.drive.gearing.rear_teeth == 39 && reread.drive.crank_phase_rad == 0.,
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
        test_reaction_table_is_switched_off();
        test_reaction_keys_and_context_rules();
        test_computed_drive_members_are_not_authored();
        test_load_resolves_a_flat_start();
        test_written_profile_authors_only_the_table();
        test_recorded_document_keeps_the_prepared_start();
    });
}
