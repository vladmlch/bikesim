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
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model/rider_geometry.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
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
    auto pose_request = welded_request(dir.path() / "cache_a", false);
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
    auto cache_request = welded_request(cache_dir, true);
    auto first = init::make_setup(cache_request);
    const auto first_result = init::settle(*first);
    bt::require(!first_result.cache_hit && first_result.steps == pose_steps,
                "the canonical settle reproduces the recorded pose");
    const auto entries = npz_files(cache_dir);
    bt::require(entries.size() == 1, "one canonical entry was published");
    const auto canonical = entries.front();

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
        config.physics_mode = "physical";
        // post_init already resolved the legacy pitch_assist default at
        // default_physics() time — clear it so the physical-mode config is
        // valid except for the state-file field under test.
        config.pitch_assist = false;
        config.equilibrium_state_file = std::string(blank);
        expect_invalid("equilibrium_state_file",
                       [&] { cfg::post_init(config); });
    }
    for (const std::string_view spelling : {"3.5", "[\"x\"]"}) {
        const auto toml = dir.path() / "physics.toml";
        std::string document = "physics_mode = \"physical\"\n"
                               "equilibrium_state_file = ";
        document.append(spelling).append("\n");
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
                "equilibrium_state_file = \"poses/flat.npz\"\n";
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
             << "equilibrium_state_file = \"" << absolute.string() << "\"\n";
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
    auto request = welded_request(dir.path() / "cache", false);
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
    auto request = welded_request(cache_dir, true);
    auto owner = init::make_setup(request);
    const auto first = init::settle(*owner);
    bt::require(!first.cache_hit,
                "the construction-time settle publishes the canonical entry");
    const auto entries = npz_files(cache_dir);
    bt::require(entries.size() == 1, "exactly one canonical entry exists");
    const auto canonical = entries.front();
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

void check() {
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
}

NATIVE_DIAG_POP
} // namespace

int main() { return bt::run(check); }
