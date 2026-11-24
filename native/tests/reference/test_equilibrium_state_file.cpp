// test_equilibrium_state_file.cpp — C++ port of
// tests/reference/test_equilibrium_state_file.py (T53 batch 3.27).
//
// The explicit `equilibrium_state_file` pose file alongside the derived-name
// auto-cache. The Python suite builds flat/lumped/coast RideSimulation
// rigs; the native equivalent boundary is a SetupRequest + init::settle on
// the welded_flat migration request (the shared cheap rig the T22-T25
// equilibrium ports already exercise — the settle is deterministic, so
// identical configs record identical steps).
//
// Post-fix parity note (8bb51212): the Python poisoning regression relies on
// a reset()-time re-settle publishing under a quarantined episode key. The
// native runtime has no re-settle surface at all — SetupOwner::settle is
// single-shot (a second call throws before any state work) and runtime
// reset() replays the sealed bootstrap, so no post-episode write can reach
// ANY cache name. The native contract test pins that strictly stronger
// guarantee: after settle + seal + advance + reset + advance, the cache
// directory still holds byte-identical canonical bytes and nothing else.
// There is no episode_fingerprint() to port; the episode-key mechanism is
// `python-only` in the port map.
#include <mujoco/mujoco.h>

#include "diag.hpp"

#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "initialization/setup_owner.hpp"
#include "initialization/state_file.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "io/value.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model/rider_geometry.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "terrain/trackfile.hpp"
#include "terrain/types.hpp"
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "support/temporary_directory.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {
NATIVE_DIAG_PUSH
// The settle-and-serve tests keep whole decoded StateFile fixtures (model
// arrays plus state arrays) in automatic storage; like the T22-T26 runtime
// ports the frame budget is suppressed instead of heap-routing the fixture.
// cppcheck-suppress unknownMacro // NATIVE_DIAG_* is the clang-dialect _Pragma wrapper from diag.hpp; cppcheck does not resolve it through -isystem
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

namespace init = bike::initialization;
namespace io = bike::io;
namespace cfg = bike::configuration;
namespace model = bike::model;

// The welded_flat request with the T24 hygiene resets: no transfer or pose
// input, explicit speed/cache knobs set by each test.
[[nodiscard]] init::SetupRequest
welded_request(const std::filesystem::path &cache_directory,
               bool cache_enabled) {
    auto request = init::read_setup_request(
        bt::fixture("requests/welded_flat.toml"));
    request.model.physics->equilibrium_cache_enabled = cache_enabled; // NOLINT(bugprone-unchecked-optional-access) the fixture request always decodes a physics file
    request.model.physics->equilibrium_state_file.reset(); // NOLINT(bugprone-unchecked-optional-access) same
    request.model.physics->initial_speed_mps = 0.; // NOLINT(bugprone-unchecked-optional-access) same
    request.initial_state.reset();
    request.cache_directory = cache_directory;
    return request;
}

// The pneumatic request on the Savage climb (the profile's track and
// rider), with cache and pose pins under each test's control.
[[nodiscard]] init::SetupRequest
pneumatic_savage_request(const std::filesystem::path &cache_directory) {
    auto request = init::read_setup_request(
        bt::fixture("requests/pneumatic_flat.toml"));
    request.track = bike::terrain::load_track(
        bt::fixture("tracks/rough_uphill_savage.toml"));
    request.model.field =
        bike::terrain::HeightFieldSpec::for_track(request.track);
    request.model.physics->equilibrium_cache_enabled = false; // NOLINT(bugprone-unchecked-optional-access) the fixture request always decodes a physics file
    request.model.physics->equilibrium_state_file.reset(); // NOLINT(bugprone-unchecked-optional-access) same
    request.cache_directory = cache_directory;
    return request;
}

[[nodiscard]] std::vector<std::filesystem::path>
npz_files(const std::filesystem::path &dir) {
    std::vector<std::filesystem::path> out;
    for (const auto &entry : std::filesystem::directory_iterator(dir))
        if (entry.path().extension() == ".npz") out.push_back(entry.path());
    std::ranges::sort(out);
    return out;
}

void expect_invalid(std::string_view needle,
                    const std::function<void()> &action) {
    bool rejected = false;
    try {
        action();
    } catch (const std::invalid_argument &failure) {
        rejected = true;
        bt::require(std::string_view(failure.what()).find(needle) !=
                        std::string_view::npos,
                    "rejection names '" + std::string(needle) +
                        "' (got: " + failure.what() + ")");
    }
    bt::require(rejected, "the request rejects with std::invalid_argument");
}

// Settle a cache-disabled owner and publish its pose with the native writer —
// the save_state_file twin.
[[nodiscard]] std::pair<std::int64_t, init::StateFile>
write_pose(const init::SetupRequest &request,
           const std::filesystem::path &path) {
    auto owner = init::make_setup(request);
    const auto result = init::settle(*owner);
    bt::require(!result.cache_hit && result.residual_qacc <= .05,
                "the settle that writes the pose physically converges");
    auto pose = owner->capture_state_file();
    init::write_state_file(path, pose);
    return {result.steps, std::move(pose)};
}

// ---------------------------------------------------------------------------
// tests/reference/test_equilibrium_state_file.py
// ::test_state_file_round_trip_loads_the_pose_without_settling
// ---------------------------------------------------------------------------
void test_state_file_round_trip() {
    const bt::TemporaryDirectory dir("eq_state_rt");
    const auto pose_path = dir.path() / "pose.npz";
    auto const pose_request = welded_request(dir.path() / "cache_a", false);
    const auto [steps, pose] = write_pose(pose_request, pose_path);
    const std::string before = bt::text(pose_path);

    auto load_request = welded_request(dir.path() / "cache_b", false);
    load_request.model.physics->equilibrium_state_file = // NOLINT(bugprone-unchecked-optional-access) fixture contract
        pose_path.string();
    auto loaded = init::make_setup(load_request);
    const auto eq = init::settle(*loaded);
    bt::require(eq.cache_hit, "explicit pose restores without settling");
    bt::require(eq.state_file && *eq.state_file == pose_path.string(),
                "equilibrium reports the explicit state file");
    bt::require(eq.steps == steps,
                "the recorded settle length is served");
    bt::require(eq.residual_qacc <= .05,
                "the restored pose is rechecked against measured acceleration");
    const auto reloaded = loaded->capture_state_file();
    bt::require(init::state_array(reloaded, "qpos").bytes ==
                    init::state_array(pose, "qpos").bytes,
                "the loaded qpos equals the settled pose bitwise");
    bt::require(bt::text(pose_path) == before,
                "the pose file is read-only at runtime");
}

// ---------------------------------------------------------------------------
// ::test_state_file_wins_over_the_auto_cache — precedence proven end to end:
// a poisoned canonical entry (steps sentinel) sits in the cache directory;
// if the auto-cache loader served first the sentinel steps would surface.
// ---------------------------------------------------------------------------
void test_state_file_wins_over_the_auto_cache() {
    const bt::TemporaryDirectory dir("eq_state_prec");
    const auto pose_path = dir.path() / "pose" / "pose.npz";
    const auto cache_dir = dir.path() / "cache";
    const auto [pose_steps, pose] =
        write_pose(welded_request(dir.path() / "unused", false), pose_path);

    // Populate the canonical entry with a cache-enabled settle on the same
    // configuration (cache/state-file knobs are fingerprint-excluded).
    auto const cache_request = welded_request(cache_dir, true);
    auto first = init::make_setup(cache_request);
    const auto first_result = init::settle(*first);
    bt::require(!first_result.cache_hit && first_result.steps == pose_steps,
                "the canonical settle reproduces the recorded pose");
    const auto entries = npz_files(cache_dir);
    bt::require(entries.size() == 1, "one canonical entry was published");
    const auto &canonical = entries.front();

    // Poison: same fingerprint array, sentinel steps — a served cache hit
    // would report 777777 steps.
    auto poisoned = init::read_state_file(canonical);
    for (auto &[name, array] : poisoned.arrays)
        if (name == "steps")
            array = io::integer_array(std::array<std::int64_t, 1>{777777}, {});
    init::write_state_file(canonical, poisoned);

    auto both = cache_request;
    both.model.physics->equilibrium_state_file = pose_path.string(); // NOLINT(bugprone-unchecked-optional-access) fixture contract
    auto owner = init::make_setup(both);
    const auto result = init::settle(*owner);
    bt::require(result.cache_hit && result.state_file &&
                    *result.state_file == pose_path.string(),
                "the explicit pose file is the served entry");
    bt::require(result.steps == pose_steps && result.steps != 777777,
                "the auto-cache loader never ran (sentinel steps absent)");
}

// ---------------------------------------------------------------------------
// ::test_state_file_rejects_a_different_configuration — rider mass is part of
// the scenario fingerprint, not the file.
// ---------------------------------------------------------------------------
void test_state_file_rejects_a_different_configuration() {
    const bt::TemporaryDirectory dir("eq_state_cfg");
    const auto pose_path = dir.path() / "pose.npz";
    (void)write_pose(welded_request(dir.path() / "cache", false), pose_path);
    auto changed = welded_request(dir.path() / "cache", false);
    changed.model.physics->equilibrium_state_file = pose_path.string(); // NOLINT(bugprone-unchecked-optional-access)
    bt::require(changed.model.rider.has_value(),
                "the welded request carries a rider spec");
    changed.model.rider->mass_kg = 75.; // NOLINT(bugprone-unchecked-optional-access) checked above
    auto owner = init::make_setup(changed);
    expect_invalid(pose_path.string(),
                   [&] { (void)init::settle(*owner); });
}

// ---------------------------------------------------------------------------
// ::test_state_file_rejects_a_state_payload_that_does_not_fit — drop the
// canonical tire wire so only the strict all-or-nothing material check can
// reject the file (the fingerprint member is declared verbatim, so the
// fingerprint check still passes).
// ---------------------------------------------------------------------------
void test_state_file_rejects_payload_that_does_not_fit() {
    const bt::TemporaryDirectory dir("eq_state_fit");
    const auto pose_path = dir.path() / "pose.npz";
    (void)write_pose(welded_request(dir.path() / "cache", false), pose_path);
    auto broken = init::read_state_file(pose_path);
    const auto removed = std::erase_if(
        broken.arrays,
        [](const auto &entry) { return entry.first == "state_tire_wire_names"; });
    bt::require(removed == 1, "the captured pose carries the tire wire");
    init::write_state_file(pose_path, broken);
    auto request = welded_request(dir.path() / "cache", false);
    request.model.physics->equilibrium_state_file = pose_path.string(); // NOLINT(bugprone-unchecked-optional-access)
    auto owner = init::make_setup(request);
    expect_invalid(pose_path.string(),
                   [&] { (void)init::settle(*owner); });
}

// ---------------------------------------------------------------------------
// ::test_state_file_rechecks_the_residual — a +5 cm root_z offset keeps a
// valid fingerprint and shape; only the post-restore residual gate rejects.
// ---------------------------------------------------------------------------
void test_state_file_rechecks_the_residual() {
    const bt::TemporaryDirectory dir("eq_state_res");
    const auto pose_path = dir.path() / "pose.npz";
    const auto pose_request = welded_request(dir.path() / "cache", false);
    (void)write_pose(pose_request, pose_path);
    // The pose's qpos row is laid out by the same compiled model the request
    // resolves to — reproduce that build to locate root_z's qpos address.
    auto probe_request = pose_request;
    probe_request.model.rider = model::resolve_rider(
        model::RiderArg{*probe_request.model.rider}, // NOLINT(bugprone-unchecked-optional-access) the welded fixture always declares a rider
        probe_request.model.include_rider, "lumped");
    const auto probe = model::compile_mjcf(
        model::generate_mjcf(probe_request.model), probe_request.asset_root);
    const auto [root_z, ignored] = model::joint_address(*probe, "root_z");
    (void)ignored;

    auto shifted = init::read_state_file(pose_path);
    for (auto &[name, array] : shifted.arrays) {
        if (name != "qpos") continue;
        auto values = array.doubles();
        values.at(static_cast<std::size_t>(root_z)) += .05;
        array = io::double_array(values, array.shape);
    }
    init::write_state_file(pose_path, shifted);
    auto request = welded_request(dir.path() / "cache", false);
    request.model.physics->equilibrium_state_file = pose_path.string(); // NOLINT(bugprone-unchecked-optional-access)
    auto owner = init::make_setup(request);
    expect_invalid(pose_path.string(),
                   [&] { (void)init::settle(*owner); });
}

// ---------------------------------------------------------------------------
// ::test_state_file_names_a_missing_file
// ---------------------------------------------------------------------------
void test_state_file_names_a_missing_file() {
    const bt::TemporaryDirectory dir("eq_state_miss");
    const auto missing = dir.path() / "absent.npz";
    auto request = welded_request(dir.path() / "cache", false);
    request.model.physics->equilibrium_state_file = missing.string(); // NOLINT(bugprone-unchecked-optional-access)
    auto owner = init::make_setup(request);
    expect_invalid(missing.string(),
                   [&] { (void)init::settle(*owner); });
}

// ---------------------------------------------------------------------------
// ::test_equilibrium_state_field_validation — '', '   ' are field-level
// rejections via post_init (an empty TOML string resolves to the declaring
// directory before post_init, so the blank spellings go through the typed
// member directly); 3.5 and ['x'] are wrong-kind leaves rejected at the TOML
// resolution boundary — both diagnostics name the field.
// ---------------------------------------------------------------------------
void test_equilibrium_state_field_validation() {
    const bt::TemporaryDirectory dir("eq_state_field");
    for (const std::string_view blank : {"", "   "}) {
        auto config = cfg::default_physics();
        config.equilibrium_state_file = std::string(blank);
        expect_invalid("equilibrium_state_file",
                       [&] { cfg::post_init(config); });
    }
    for (const std::string_view spelling : {"3.5", "[\"x\"]"}) {
        const auto toml = dir.path() / "physics.toml";
        std::string document = "physics_mode = \"physical\"\n"
                               "timestep_s = 0.0005\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                               "equilibrium_state_file = ";
        document.append(spelling).append(
            "\n[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
            "contact_dropout_s = 0.005\n"
            "[research]\ncontrol_period_s = 0.01\nactuator_delay_s = 0.005\n"
            "sensor_latency_s = 0.01\nsensor_maximum_age_s = 0.1\n"
            "sample_period_s = 0.005\nrecord_decimation = 80\n");
        std::ofstream file(toml, std::ios::binary | std::ios::trunc);
        bt::require(file.is_open(), "field-validation TOML write opens");
        file << document;
        file.close();
        expect_invalid("equilibrium_state_file",
                       [&] { (void)cfg::load_physics(toml); });
    }
}

// ---------------------------------------------------------------------------
// ::test_toml_state_file_resolves_relative_to_the_config
// ---------------------------------------------------------------------------
void test_toml_state_file_resolves_relative_to_the_config() {
    const bt::TemporaryDirectory dir("eq_state_toml");
    const auto toml = dir.path() / "physics.toml";
    {
        std::ofstream file(toml, std::ios::binary | std::ios::trunc);
        bt::require(file.is_open(), "relative-path TOML write opens");
        file << "physics_mode = \"physical\"\n"
                "timestep_s = 0.0005\ncontrol_period_s = 0.005\n"
                "equilibrium_state_file = \"poses/flat.npz\"\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[seated_climb]\n"
                "enabled = false\ntarget_crank_power_w = 225.0\nlean_gain = 1.0\nmax_backward_lean_rad = 0.1\n"
                "surge_power_w = 400.0\nsurge_grade = 0.2\nsurge_recovery_rate = 0.3333333333333333\n"
                "front_load_share_target = 0.3\nlean_trim_gain_rad_s = 0.3\nlean_trim_limit_rad = 0.15\n"
                "period_s = 0.00125\nreaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\norientation_tau_s = 0.5\nsurge_budget_s = 15\ntrim_dead_time_s = 0.2\nmax_forward_lean_rad = 0.55\nmax_crank_torque_nm = 60.0\nmin_elbow_flexion_deg = 15.0\nclimb_effort_cap_grades = [0.0]\nclimb_effort_cap_grade_nm = [1000.0]\nclimb_effort_rise_speed_mps = 2.5\nclimb_effort_full_speed_mps = 4.0\n"
                "[drive]\nmotor_clutch = false\n"
                "[drive.assist]\nrelease_hold_s = 0.3\n"
                "[drive.pedaling]\n"
                "reposition_on_stall = false\nreposition_min_effort_nm = 20.0\n"
                "reposition_stall_window_s = 1.0\nreposition_stall_progress_rad = 0.5\n"
                "reposition_cooldown_s = 1.0\nreposition_back_rate_rad_s = 3.0\nreposition_damping_nms_rad = 12.0\n"
                "reposition_timeout_s = 3.0\nreposition_phase_tolerance_rad = 0.05\n"
                "reposition_noop_rad = 0.12\nstop_time_s = 0.35\n"
                "[drive.shifting]\nshift_cooldown_s = 0.4\nshift_cut_duration_s = 0.2\n"
                "[articulated]\nbalance_dwell_s = 0.5\nbalance_grace_s = 3.0\njoint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.005\n"
                "[research]\ncontrol_period_s = 0.01\nactuator_delay_s = 0.005\n"
                "sensor_latency_s = 0.01\nsensor_maximum_age_s = 0.1\n"
                "sample_period_s = 0.005\nrecord_decimation = 80\n";
    }
    const auto resolved = cfg::load_physics(toml);
    bt::require(resolved.equilibrium_state_file ==
                    std::filesystem::weakly_canonical(dir.path() / "poses" /
                                                      "flat.npz")
                        .string(),
                "a relative state-file path resolves against the declaring file");
    const auto absolute = dir.path() / "elsewhere.npz";
    {
        std::ofstream file(toml, std::ios::binary | std::ios::trunc);
        bt::require(file.is_open(), "absolute-path TOML write opens");
        file << "physics_mode = \"physical\"\n"
             << "timestep_s = 0.0005\ncontrol_period_s = 0.005\n"
             << "equilibrium_state_file = \"" << absolute.string() << "\"\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
             << "[seated_climb]\n"
             << "enabled = false\ntarget_crank_power_w = 225.0\nlean_gain = 1.0\nmax_backward_lean_rad = 0.1\n"
             << "surge_power_w = 400.0\nsurge_grade = 0.2\nsurge_recovery_rate = 0.3333333333333333\n"
             << "front_load_share_target = 0.3\nlean_trim_gain_rad_s = 0.3\nlean_trim_limit_rad = 0.15\n"
             << "period_s = 0.00125\nreaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\norientation_tau_s = 0.5\nsurge_budget_s = 15\ntrim_dead_time_s = 0.2\nmax_forward_lean_rad = 0.55\nmax_crank_torque_nm = 60.0\nmin_elbow_flexion_deg = 15.0\nclimb_effort_cap_grades = [0.0]\nclimb_effort_cap_grade_nm = [1000.0]\nclimb_effort_rise_speed_mps = 2.5\nclimb_effort_full_speed_mps = 4.0\n"
             << "[drive]\nmotor_clutch = false\n"
             << "[drive.assist]\nrelease_hold_s = 0.3\n"
             << "[drive.pedaling]\n"
             << "reposition_on_stall = false\nreposition_min_effort_nm = 20.0\n"
             << "reposition_stall_window_s = 1.0\nreposition_stall_progress_rad = 0.5\n"
             << "reposition_cooldown_s = 1.0\nreposition_back_rate_rad_s = 3.0\nreposition_damping_nms_rad = 12.0\n"
             << "reposition_timeout_s = 3.0\nreposition_phase_tolerance_rad = 0.05\n"
             << "reposition_noop_rad = 0.12\nstop_time_s = 0.35\n"
             << "[drive.shifting]\nshift_cooldown_s = 0.4\nshift_cut_duration_s = 0.2\n"
             << "[articulated]\nbalance_dwell_s = 0.5\nbalance_grace_s = 3.0\njoint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n"
             << "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.005\n"
             << "[research]\ncontrol_period_s = 0.01\nactuator_delay_s = 0.005\n"
             << "sensor_latency_s = 0.01\nsensor_maximum_age_s = 0.1\n"
             << "sample_period_s = 0.005\nrecord_decimation = 80\n";
    }
    bt::require(cfg::load_physics(toml).equilibrium_state_file ==
                    absolute.string(),
                "an absolute state-file path survives verbatim");
}

// ---------------------------------------------------------------------------
// ::test_equilibrium_override_kwargs_reach_the_solve — EquilibriumOptions
// carries max_steps/tolerance into the solve and the domain guards reject
// nonfinite tolerance and nonpositive budgets. The source's
// `RideSimulation(equilibrium_max_steps=4000)` ctor rejection is a
// kwargs-require-physics contract with no native twin: `python-only`.
// ---------------------------------------------------------------------------
void test_equilibrium_override_kwargs_reach_the_solve() {
    const bt::TemporaryDirectory dir("eq_state_opts");
    auto const request = welded_request(dir.path() / "cache", false);
    {
        auto owner = init::make_setup(request);
        const auto result =
            init::settle(*owner, {.max_steps = 4000, .tolerance = .25});
        bt::require(result.residual_qacc <= .25,
                    "the tolerance override reaches the solve");
    }
    {
        auto owner = init::make_setup(request);
        expect_invalid("tolerance", [&] {
            (void)init::settle(
                *owner,
                {.max_steps = std::nullopt,
                 .tolerance =
                     std::numeric_limits<double>::quiet_NaN()});
        });
    }
    for (const std::int64_t bad_budget : {0, -5}) {
        auto owner = init::make_setup(request);
        expect_invalid("max_steps", [&] {
            (void)init::settle(*owner, {.max_steps = bad_budget,
                                        .tolerance = .05});
        });
    }
}

// ---------------------------------------------------------------------------
// ::test_dirty_resettle_writes_the_episode_key_not_the_canonical_name +
// ::test_experiment_configs_produce_different_episode_keys — the native
// equivalent of the 8bb51212 poisoning fix. Native has no episode-key
// namespace because no post-episode settle exists: a second settle on the
// same owner throws, and runtime reset() replays the sealed bootstrap
// without touching the cache directory. The pin is therefore stronger:
// after settle -> seal -> advance -> reset -> advance -> reset the cache
// directory still holds exactly the canonical bytes, and a fresh owner on
// the same directory still warm-loads the canonical entry.
// ---------------------------------------------------------------------------
void test_post_episode_lifecycle_writes_no_cache_entry() {
    const bt::TemporaryDirectory dir("eq_state_poison");
    const auto cache_dir = dir.path() / "cache";
    auto const request = welded_request(cache_dir, true);
    auto owner = init::make_setup(request);
    const auto first = init::settle(*owner);
    bt::require(!first.cache_hit,
                "the construction-time settle publishes the canonical entry");
    const auto entries = npz_files(cache_dir);
    bt::require(entries.size() == 1, "exactly one canonical entry exists");
    const auto &canonical = entries.front();
    const std::string canonical_bytes = bt::text(canonical);

    // The construction-time settle is the only settle an owner can run.
    expect_invalid("only be completed once",
                   [&] { (void)init::settle(*owner); });

    // Post-episode lifecycle: seal the t=0 bootstrap, run an episode fragment,
    // reset, run another. None of this can reach the cache directory.
    auto runtime = std::make_unique<runtime::NativeRideRuntime>(
        owner->seal_bootstrap());
    (void)runtime->advance(3, runtime::RideControl{}, 0., 0., std::nullopt);
    runtime->reset();
    (void)runtime->advance(3, runtime::RideControl{}, 0., 0., std::nullopt);
    runtime->reset();
    const auto after = npz_files(cache_dir);
    bt::require(after.size() == 1 && after.front() == canonical,
                "no quarantined or canonical rewrite follows a reset");
    bt::require(bt::text(canonical) == canonical_bytes,
                "the canonical entry is byte-identical after the lifecycle");

    // A fresh consumer still resolves the canonical key — the only name the
    // loader reads.
    auto consumer = init::make_setup(request);
    const auto second = init::settle(*consumer);
    bt::require(second.cache_hit && second.steps == first.steps,
                "a fresh consumer loads the canonical entry, never episode state");
}

// ---------------------------------------------------------------------------
// Queue item 6 follow-up: the repaired capture -> restore path proven on a
// fresh solve, with no build-bound checked-in file. Settle the pneumatic
// Savage configuration, write the served pose and restore it through the
// explicit path — the welded round trip's twin, through the carcass whose
// pre-finalize hysteresis memory the writer now preserves.
// ---------------------------------------------------------------------------
void test_pneumatic_settle_restore_round_trip() {
    const bt::TemporaryDirectory dir("eq_state_pneu");
    const auto pose_path = dir.path() / "pose.npz";
    const auto [steps, pose] =
        write_pose(pneumatic_savage_request(dir.path() / "cache_a"),
                   pose_path);
    const double certified = init::state_array(pose, "residual_qacc").doubles().front();
    std::cerr << "pneumatic savage certified residual=" << certified << '\n';

    auto load_request = pneumatic_savage_request(dir.path() / "cache_b");
    load_request.model.physics->equilibrium_state_file = // NOLINT(bugprone-unchecked-optional-access) fixture contract
        pose_path.string();
    auto loaded = init::make_setup(load_request);
    const auto eq = init::settle(*loaded);
    std::cerr << "pneumatic savage restored residual=" << eq.residual_qacc << '\n';
    bt::require(eq.cache_hit && eq.steps == steps,
                "the fresh pneumatic pose restores without settling");
    bt::require(eq.residual_qacc <= .05,
                "the restored pneumatic pose passes the residual recheck");
    bt::require(eq.residual_qacc == certified,
                "the restored pneumatic residual equals the certified residual exactly");
    const auto reloaded = loaded->capture_state_file();
    bt::require(init::state_array(reloaded, "qpos").bytes ==
                    init::state_array(pose, "qpos").bytes,
                "the restored qpos equals the settled pose bitwise");
}

// ---------------------------------------------------------------------------
// The equilibrium key binds joint file content, not the checkout: the
// pneumatic configuration staged in two directories keys one pose, and
// editing a joint file the physics config names changes the key.
// ---------------------------------------------------------------------------
// The request tree the pneumatic fixture reaches: the request, its physics
// TOML, the track, and the two joint files the physics TOML names by path.
constexpr std::array<std::string_view, 5> kPneumaticTree{
    "requests/pneumatic_flat.toml", "config/viewer_physics_pneumatic.toml",
    "config/rider_joint_envelope_anatomical.json",
    "config/rider_strength_reference.json", "tracks/flat.toml"};

void stage_pneumatic_tree(const std::filesystem::path &root) {
    for (const auto relative : kPneumaticTree) {
        const auto destination = root / std::filesystem::path(relative);
        std::filesystem::create_directories(destination.parent_path());
        std::filesystem::copy_file(bt::fixture(relative), destination);
    }
}

// Settles the request with the cache off and returns the fingerprint its
// settled pose records.
[[nodiscard]] std::string
settled_fingerprint(const std::filesystem::path &request_file) {
    auto request = init::read_setup_request(request_file);
    request.model.physics->equilibrium_cache_enabled = false; // NOLINT(bugprone-unchecked-optional-access) the request always decodes a physics file
    request.model.physics->equilibrium_state_file.reset(); // NOLINT(bugprone-unchecked-optional-access) same
    auto owner = init::make_setup(request);
    (void)init::settle(*owner);
    return init::state_array(owner->capture_state_file(), "fingerprint_sha256")
        .strings()
        .front();
}

void test_fingerprint_binds_joint_content_not_checkout() {
    const bt::TemporaryDirectory first_dir("eq_key_first");
    const bt::TemporaryDirectory second_dir("eq_key_second");
    stage_pneumatic_tree(first_dir.path());
    stage_pneumatic_tree(second_dir.path());
    const auto first_request = first_dir.path() / "requests" / "pneumatic_flat.toml";
    const auto second_request = second_dir.path() / "requests" / "pneumatic_flat.toml";
    const auto first = settled_fingerprint(first_request);
    bt::require(settled_fingerprint(second_request) == first,
                "the same physics configuration keys one pose from any checkout");
    {
        std::ofstream file(second_dir.path() / "config" / "rider_strength_reference.json",
                           std::ios::binary | std::ios::app);
        bt::require(file.is_open(), "joint file append opens");
        file << '\n';
    }
    bt::require(settled_fingerprint(second_request) != first,
                "editing a joint file the physics config names changes the pose key");
}

// ---------------------------------------------------------------------------
// An unreadable joint file the cache key hashes fails settle by field name, before any solve.
// ---------------------------------------------------------------------------
void test_unreadable_joint_file_fails_before_settle() {
    const bt::TemporaryDirectory dir("eq_joint_missing");
    const auto cache_dir = dir.path() / "cache";
    const auto missing = dir.path() / "absent_envelope.json";
    // A lumped rider on the default physical profile builds no joints, so
    // nothing else opens the envelope file; the welded profile's pinned planar
    // attachments would demand an articulated rider.
    auto request = welded_request(cache_dir, true);
    request.model.rider->variant = "lumped"; // NOLINT(bugprone-unchecked-optional-access) the welded fixture always declares a rider
    request.model.physics = cfg::default_physics(); // NOLINT(bugprone-unchecked-optional-access) the request always decodes a physics file
    request.model.physics->physics_mode = "physical"; // NOLINT(bugprone-unchecked-optional-access) set just above
    cfg::post_init(*request.model.physics); // NOLINT(bugprone-unchecked-optional-access) set just above
    request.model.physics->equilibrium_cache_enabled = true; // NOLINT(bugprone-unchecked-optional-access) set just above
    request.model.physics->articulated.joint_envelope_path = missing.string(); // NOLINT(bugprone-unchecked-optional-access) set just above
    auto owner = init::make_setup(request);
    expect_invalid("joint_envelope_path",
                   [&] { (void)init::settle(*owner); });
    bt::require(!std::filesystem::exists(cache_dir) || npz_files(cache_dir).empty(),
                "the rejected configuration writes no cache entry");
}

// ---------------------------------------------------------------------------
// Queue item 6 follow-up: capture_initial_state() serves the finalized
// context — a PhysicalInitialState is a launch transfer, so a moving
// initial_speed_mps start must carry the qvel finalize writes, not the
// rest pose's qvel=0.
// ---------------------------------------------------------------------------
void test_capture_initial_state_keeps_launch_velocity() {
    const bt::TemporaryDirectory dir("eq_state_speed");
    auto request = welded_request(dir.path() / "cache", false);
    request.model.physics->initial_speed_mps = 5.; // NOLINT(bugprone-unchecked-optional-access) fixture contract
    auto owner = init::make_setup(request);
    (void)init::settle(*owner);
    const auto captured = owner->capture_initial_state();
    const auto payload = captured.payload();
    const auto &object = io::as_object(payload, "initial state");
    const auto *qvel = io::find_member(object, "qvel");
    bt::require(qvel != nullptr, "the transfer payload carries qvel");
    bool moving = false;
    for (const auto &entry : io::as_array(*qvel, "qvel"))
        moving = moving || std::abs(io::as_double(entry, "qvel")) > 0.;
    bt::require(moving,
                "a moving start's initial-state transfer preserves qvel");
}

void test_default_sag_across_backends() {
    const bt::TemporaryDirectory dir("default_sag");
    const auto root = bt::oracle::root().parent_path().parent_path().parent_path();
    std::array<init::EquilibriumResult, 3> results;
    constexpr std::array profiles{"welded", "distributed", "pneumatic"};
    for (std::size_t i = 0; i < profiles.size(); ++i) {
        auto request = welded_request(dir.path(), false);
        request.model.physics = cfg::load_physics(
            root / "examples/research" /
            (std::string("viewer_physics_") + profiles[i] + ".toml"));
        request.model.physics->equilibrium_state_file.reset();
        request.model.physics->equilibrium_cache_enabled = false;
        request.model.physics->initial_speed_mps = 0.;
        auto owner = init::make_setup(std::move(request));
        results[i] = init::settle(*owner);
        std::cerr << profiles[i] << " default sag fork_mm="
                  << results[i].fork_travel_mm << " shock_mm="
                  << results[i].shock_stroke_mm << " residual="
                  << results[i].residual_qacc << '\n';
    }
    for (std::size_t i = 0; i < profiles.size(); ++i) {
        bt::require(std::abs(results[i].fork_travel_mm - 54.) <= .5,
                    std::string(profiles[i]) + " fork default sag");
        bt::require(std::abs(results[i].shock_stroke_mm - 19.5) <= .5,
                    std::string(profiles[i]) + " shock default sag");
    }
}

void test_sag_evaluator_explicit_default_pair() {
    const bt::TemporaryDirectory dir("sag_evaluator_pair");
    const auto root = bt::oracle::root().parent_path().parent_path().parent_path();
    for (const auto *profile : {"distributed", "pneumatic"}) {
        auto request = welded_request(dir.path(), false);
        request.model.physics = cfg::load_physics(root / "examples/research" /
            (std::string("viewer_physics_") + profile + ".toml"));
        request.model.physics->initial_speed_mps = 0.;
        const auto evaluator = init::build_equilibrium_evaluator(std::move(request));
        const auto exact = evaluator(97., 81550.);
        const auto nearby = evaluator(97.000001, 81550.);
        std::cerr << profile << " evaluator exact=" << exact[0] << "/" << exact[1]
                  << " nearby=" << nearby[0] << "/" << nearby[1] << '\n';
        bt::require(std::abs(exact[0] - nearby[0]) < .01 &&
                    std::abs(exact[1] - nearby[1]) < .01,
                    std::string(profile) + " preserves explicit default pair");
    }
}

void check() {
    test_sag_evaluator_explicit_default_pair();
    test_default_sag_across_backends();
    test_state_file_round_trip();
    test_state_file_wins_over_the_auto_cache();
    test_state_file_rejects_a_different_configuration();
    test_state_file_rejects_payload_that_does_not_fit();
    test_state_file_rechecks_the_residual();
    test_state_file_names_a_missing_file();
    test_equilibrium_state_field_validation();
    test_toml_state_file_resolves_relative_to_the_config();
    test_equilibrium_override_kwargs_reach_the_solve();
    test_post_episode_lifecycle_writes_no_cache_entry();
    test_pneumatic_settle_restore_round_trip();
    test_fingerprint_binds_joint_content_not_checkout();
    test_unreadable_joint_file_fails_before_settle();
    test_capture_initial_state_keeps_launch_velocity();
}

NATIVE_DIAG_POP
} // namespace

int main() { return bt::run(check); }
