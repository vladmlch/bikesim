// test_t13.cpp — configuration loading/resolution regression (task T13):
// the TOML/JSON -> SimulationPhysicsConfig boundary ported from
// physics/resolution.py and the plant_profile.py TOML writer.
//
// Covered seams:
//   * load_physics(file): bounded no-follow TOML parse, {cls}-table/unknown-
//     key/missing-required rejection, nested-record construction and the
//     TireSpec -> TabulatedTireSpec material promotion — verified bitwise
//     against the captured resolved/*.json and variant_*.resolved.json
//     oracle documents (machine-local path relocalization only).
//   * resolve_config_paths: joint_envelope_path / joint_strength_path /
//     equilibrium_state_file anchor at the declaring TOML's directory —
//     relative spelling in, canonical absolute out, never the process cwd;
//     absolute spellings pass through untouched.
//   * resolve_physics(overrides, base): the _merge precedence port —
//     defaults < file < explicit values, deep object merge, explicit null
//     skipped, non-mapping overrides rejected.
//   * write_physics_toml: asdict-equivalent TOML emission that reads back to
//     the identical effective configuration; exclusive (non-overwriting)
//     destination semantics via the platform boundary.
#include "support/fixture_test.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "platform/files.hpp"
#include "io/json.hpp"
#include "diag.hpp"
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace {

namespace cfg = bike::configuration;
namespace io = bike::io;
namespace pf = bike::platform;
using bike::io::Number;
using bike::io::Value;

// ---- Value helpers (same spellings the T12 regression uses) ---------------

const Value::Object& object_of(const Value& value, std::string_view field) {
    return io::as_object(value, field);
}
Value::Object& object_of(Value& value) {
    return std::get<Value::Object>(value.data);
}
const Value* member_opt(const Value& object, std::string_view key,
                        std::string_view path) {
    return io::find_member(object_of(object, path), key);
}
Value* find_member_mut(Value::Object& object, std::string_view key) {
    for (auto& member : object)
        if (member.first == key) return &member.second;
    return nullptr;
}
Value make_number(std::string_view lexeme) {
    Value v;
    v.data = Number{std::string(lexeme)};
    return v;
}
Value make_text(std::string_view text) {
    Value v;
    v.data = std::string(text);
    return v;
}
Value make_bool(bool flag) {
    Value v;
    v.data = flag;
    return v;
}
Value make_null() {
    return Value{};
}
Value make_array(std::initializer_list<Value> items) {
    Value v;
    v.data = Value::Array(items.begin(), items.end());
    return v;
}
Value make_object(std::initializer_list<Value::Object::value_type> members) {
    Value v;
    v.data = Value::Object(members.begin(), members.end());
    return v;
}

std::string canonical(const Value& value) { return io::canonical_json(value); }

// Drops a [[nodiscard]] result inside rejection probes.
template <typename T> void discard(T&&) {}

// The required [research] profile section. The values are the research-mode
// timings in effect today; staged documents carry them so each probe keeps
// the failure mode it asserts rather than tripping the missing-key check.
constexpr std::string_view kResearchSection =
    "[research]\ncontrol_period_s = 0.01\nactuator_delay_s = 0.005\n"
    "sensor_latency_s = 0.01\nsensor_maximum_age_s = 0.1\n"
    "sample_period_s = 0.005\nrecord_decimation = 80\n";

// The remaining required profile timings staged documents carry so each
// probe keeps the failure mode it asserts rather than tripping a
// missing-member check: the assist run-on, the shift cooldown/cut and the
// balance dwell/grace windows at the shipped values — whole physics ticks
// at the staged 1.25 ms step.
constexpr std::string_view kAssistSection =
    "[drive.assist]\nrelease_hold_s = 0.3\n";
constexpr std::string_view kShiftingSection =
    "[drive.shifting]\nshift_cooldown_s = 0.4\nshift_cut_duration_s = 0.2\n";
constexpr std::string_view kArticulatedSection =
    "[articulated]\nbalance_dwell_s = 0.5\nbalance_grace_s = 3.0\n"
    "joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n";
constexpr std::string_view kAuxTimings =
    "[drive.assist]\nrelease_hold_s = 0.3\n"
    "[drive.shifting]\nshift_cooldown_s = 0.4\nshift_cut_duration_s = 0.2\n"
    "[articulated]\nbalance_dwell_s = 0.5\nbalance_grace_s = 3.0\n"
    "joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n";

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (manifest name, fixture path) call sites stay ordered by usage
Value fixture_json(std::string_view manifest_name [[maybe_unused]],
                    std::string_view relative) {
    return io::parse_json(bt::text(bt::fixture(relative)));
}

// Recursive first-difference walk so a resolved-tree mismatch names the
// divergent member path instead of an opaque canonical-JSON inequality.
// NOLINTNEXTLINE(misc-no-recursion) recursive tree descent is the point of the helper
std::string first_diff(const Value& actual, const Value& expected,
                       const std::string& path) {
    if (actual.data.index() != expected.data.index())
        return path + ": kind mismatch (" +
               std::to_string(actual.data.index()) + " != " +
               std::to_string(expected.data.index()) + ")";
    if (const auto* a = std::get_if<io::Number>(&actual.data)) {
        const io::Number& b = std::get<io::Number>(expected.data);
        if (a->lexeme != b.lexeme)
            return path + ": number " + a->lexeme + " != " + b.lexeme;
        return {};
    }
    if (const auto* a = std::get_if<bool>(&actual.data)) {
        if (*a != std::get<bool>(expected.data))
            return path + ": bool " + std::string(*a ? "true" : "false") +
                   " != " +
                   std::string(std::get<bool>(expected.data) ? "true"
                                                            : "false");
        return {};
    }
    if (const auto* a = std::get_if<std::string>(&actual.data)) {
        if (*a != std::get<std::string>(expected.data))
            return path + ": string '" + *a + "' != '" +
                   std::get<std::string>(expected.data) + "'";
        return {};
    }
    if (const auto* a = std::get_if<Value::Array>(&actual.data)) {
        const Value::Array& b = std::get<Value::Array>(expected.data);
        if (a->size() != b.size())
            return path + ": array length " + std::to_string(a->size()) +
                   " != " + std::to_string(b.size());
        for (std::size_t i = 0; i < a->size(); ++i)
            if (std::string diff = first_diff(
                    a->at(i), b.at(i), path + "[" + std::to_string(i) + "]");
                !diff.empty())
                return diff;
        return {};
    }
    if (const auto* a = std::get_if<Value::Object>(&actual.data)) {
        const Value::Object& b = std::get<Value::Object>(expected.data);
        if (a->size() != b.size())
            return path + ": member count " + std::to_string(a->size()) +
                   " != " + std::to_string(b.size());
        std::vector<const Value::Object::value_type*> am, bm;
        am.reserve(a->size());
        bm.reserve(b.size());
        for (const auto& member : *a) am.push_back(&member);
        for (const auto& member : b) bm.push_back(&member);
        const auto by_key = [](const Value::Object::value_type* x,
                               const Value::Object::value_type* y) {
            return x->first < y->first;
        };
        std::ranges::sort(am, by_key);
        std::ranges::sort(bm, by_key);
        for (std::size_t i = 0; i < am.size(); ++i) {
            if (am.at(i)->first != bm.at(i)->first)
                return path + ": member '" + am.at(i)->first +
                       "' missing or reordered";
            if (std::string diff = first_diff(am.at(i)->second,
                                              bm.at(i)->second,
                                              path + "." + am.at(i)->first);
                !diff.empty())
                return diff;
        }
        return {};
    }
    if (io::canonical_json(actual) != io::canonical_json(expected))
        return path + ": leaf mismatch";
    return {};
}

// The oracle's resolved docs embed absolute paths captured in its own
// worktree; resolve_config_paths anchors them at the TOML's directory. The
// behavioral contract is profile-adjacent resolution — rewrite each captured
// leaf under the recorded fixture tree to the local fixture root, failing
// when a captured leaf escapes that layout (a fixture defect, not a
// tolerated difference).
void relocalize_resolved_paths(Value& doc) {
    constexpr std::string_view kAnchor = "tests/fixtures/migration/";
    const auto relocalize = [&](Value* leaf) {
        if (!leaf) return;
        auto* s = std::get_if<std::string>(&leaf->data);
        if (!s) return;  // explicit null stays null
        const std::size_t anchor = s->rfind(kAnchor);
        bt::require(anchor != std::string::npos,
                    "resolved doc path escapes the fixture tree: " + *s);
        const std::string relative = s->substr(anchor + kAnchor.size());
        *s = bt::fixture(relative).string();
    };
    auto* root = std::get_if<Value::Object>(&doc.data);
    bt::require(root != nullptr, "resolved doc root must be an object");
    if (Value* articulated = find_member_mut(*root, "articulated")) {
        auto* obj = std::get_if<Value::Object>(&articulated->data);
        bt::require(obj != nullptr,
                    "resolved doc .articulated must be an object");
        relocalize(find_member_mut(*obj, "joint_envelope_path"));
        relocalize(find_member_mut(*obj, "joint_strength_path"));
    }
    relocalize(find_member_mut(*root, "equilibrium_state_file"));
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (config, manifest name, fixture path) ordered by usage
void expect_resolved(const cfg::SimulationPhysicsConfig& config,
                     std::string_view manifest_name,
                     std::string_view relative) {
    Value expected = fixture_json(manifest_name, relative);
    relocalize_resolved_paths(expected);
    const Value actual = cfg::to_value(config);
    if (canonical(actual) != canonical(expected))
        bt::require(false,
                    "resolved config mismatch for " + std::string(relative) +
                        ": " + first_diff(actual, expected, "$"));
}

// The message text of a rejected load is part of the ported contract.
void invalid_with(std::string_view message,
                  const std::function<void()>& action) {
    try {
        action();
    } catch (const std::invalid_argument& error) {
        bt::require(std::string_view(error.what()) == message,
                    "rejection text mismatch — got: " +
                        std::string(error.what()));
        return;
    }
    bt::require(false,
                "expected std::invalid_argument: " + std::string(message));
}

// An exclusive temp directory owned for the test's scope.
struct TempDir {
    std::filesystem::path path;
    explicit TempDir(std::string_view label) {
        static std::atomic<int> counter{0};
        path = std::filesystem::temp_directory_path() /
               (std::string(label) + "-" +
                std::to_string(static_cast<long long>(::getpid())) + "-" +
                std::to_string(counter.fetch_add(1)));
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    TempDir(TempDir&&) = delete;
    TempDir& operator=(TempDir&&) = delete;
};

void write_text_file(const std::filesystem::path& path,
                     std::string_view content) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
    out.close();
    bt::require(out.good(), "could not stage " + path.string());
}

// ---- shipped profile loads -------------------------------------------------

// Test bodies stage whole config/Value trees and inline the typed
// construct path into one frame — the same cold-path exemption the other
// migration tests carry for -Wframe-larger-than.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[gnu::noinline]] void checks_shipped() {
    const cfg::SimulationPhysicsConfig cfg_welded =
        cfg::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
    bt::require(cfg_welded.timestep_s == 0.00125, "shipped timestep");
    bt::require(cfg_welded.physics_mode == "physical", "shipped mode");
    bt::require(cfg_welded.drive_mode.has_value() &&
                    *cfg_welded.drive_mode == "articulated_effort",
                "shipped drive mode");
    expect_resolved(cfg_welded, "config/resolved/welded.json",
                    "config/resolved/welded.json");

    const auto cfg_distributed = cfg::load_physics(
        bt::fixture("config/viewer_physics_distributed.toml"));
    bt::require(cfg_distributed.tires.backend == "distributed_2d_reference",
                "distributed backend");
    bt::require(cfg_distributed.tires.distributed.station_count == 128,
                "station count");
    expect_resolved(cfg_distributed, "config/resolved/distributed.json",
                    "config/resolved/distributed.json");

    const auto cfg_pneumatic =
        cfg::load_physics(bt::fixture("config/viewer_physics_pneumatic.toml"));
    bt::require(cfg_pneumatic.tires.backend == "pneumatic",
                "pneumatic backend");
    bt::require(cfg_pneumatic.tires.pneumatic.front.name ==
                    "Schwalbe Magic Mary 29x2.4",
                "pneumatic front tyre default retained");
    bt::require(cfg_pneumatic.equilibrium_state_file.has_value() &&
                    *cfg_pneumatic.equilibrium_state_file ==
                        bt::fixture("config/poses/savage_pneumatic.npz")
                            .string(),
                "equilibrium pose resolves profile-adjacent");
    expect_resolved(cfg_pneumatic, "config/resolved/pneumatic.json",
                    "config/resolved/pneumatic.json");
}

// ---- variant profiles ------------------------------------------------------

[[gnu::noinline]] void checks_variants() {
    for (const auto& entry : {
             std::pair<std::string_view, std::string_view>{
                 "config/variant_alt_support", "config/variant_alt_support"},
         }) {
        const std::string name(entry.first);
        const auto config = cfg::load_physics(
            bt::fixture(name + ".toml"));
        expect_resolved(config, name, name + ".resolved.json");
    }
    {
        const std::string name("config/variant_grip_weld_coercion");
        const auto config = cfg::load_physics(bt::fixture(name + ".toml"));
        bt::require(config.articulated.grip_attachment == "connect",
                    "'weld' grip coerces to 'connect'");
        expect_resolved(config, name, name + ".resolved.json");
    }
}

// ---- rejection surface ------------------------------------------------------

[[gnu::noinline]] void checks_rejection() {
    invalid_with(
        "unknown SimulationPhysicsConfig parameter(s): definitely_not_a_field",
        [] {
            discard(cfg::load_physics(bt::fixture("config/unknown_key.toml")));
        });

    const TempDir temp("bike-t13-reject");
    const auto stage = [&](std::string_view name, std::string_view toml) {
        const auto path = temp.path / std::string(name);
        write_text_file(path, toml);
        return path;
    };

    // Malformed TOML — the source's TOMLDecodeError; the native port reports
    // the parse failure with the file name attached.
    bt::invalid([&] {
        discard(cfg::load_physics(stage("bad.toml", "x = = 1\n")));
    });

    // A nested section that is not a table: '{cls} requires a TOML table'.
    invalid_with("TireBackendConfig requires a TOML table", [&] {
        discard(cfg::load_physics(stage("non_table.toml",
                                "tires = \"rubber\"\n")));
    });
    invalid_with("EndStopConfig requires a TOML table", [&] {
        discard(cfg::load_physics(stage("non_table2.toml", "end_stops = 3\n")));
    });

    // Unknown keys in a nested section name the nested class.
    invalid_with("unknown TireParameters parameter(s): bad_key", [&] {
        discard(cfg::load_physics(stage(
            "nested_unknown.toml", "[tires.front]\nbad_key = 1\n")));
    });

    // Required members of the material record are all mandatory — the
    // CPython missing-arguments text is preserved verbatim.
    invalid_with(
        "invalid TireSpec: TireSpec.__init__() missing 5 required positional "
        "arguments: 'loss_factor', 'hysteresis_rate_eps_mps', "
        "'pressure_pa_gauge', 'provenance', and 'valid_load_range_n'",
        [&] {
            discard(cfg::load_physics(stage(
                "missing_required.toml",
                "[tires.front.material]\nradial_k_n_m = 130000\n" + std::string(kResearchSection))));
        });

    // Decision 39 (2026-11-10): the compliant_2d material's viscous radial
    // damping was removed — a profile still carrying the dropped
    // radial_c_ns_m key fails to load and names it.
    invalid_with(
        "unknown TireSpec parameter(s): radial_c_ns_m",
        [&] {
            discard(cfg::load_physics(stage(
                "legacy_radial_c.toml",
                "[tires.front.material]\nradial_k_n_m = 130000\n"
                "radial_c_ns_m = 800\nloss_factor = 0.0662\n"
                "hysteresis_rate_eps_mps = 0.01\npressure_pa_gauge = 200000\n"
                "provenance = \"legacy\"\n"
                "valid_load_range_n = [0.0, 2000.0]\n" +
                std::string(kResearchSection))));
        });

    // Timing/delay fields are required profile members — a document that
    // omits one fails the missing-argument check in declaration order.
    invalid_with(
        "invalid SimulationPhysicsConfig: SimulationPhysicsConfig.__init__() "
        "missing 1 required positional argument: 'timestep_s'",
        [&] {
            discard(cfg::load_physics(stage(
                "missing_timestep.toml",
                "control_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
                "contact_dropout_s = 0.0125\n" + std::string(kResearchSection))));
        });
    invalid_with(
        "invalid SimulationPhysicsConfig: SimulationPhysicsConfig.__init__() "
        "missing 1 required positional argument: 'control_period_s'",
        [&] {
            discard(cfg::load_physics(stage(
                "missing_control_period.toml",
                "timestep_s = 0.00125\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
                "contact_dropout_s = 0.0125\n" + std::string(kResearchSection))));
        });
    invalid_with(
        "invalid SimulationPhysicsConfig: SimulationPhysicsConfig.__init__() "
        "missing 3 required positional arguments: 'rider_launch', "
        "'front_lift_reaction', and 'monitors'",
        [&] {
            discard(cfg::load_physics(stage(
                "missing_monitors.toml",
                "timestep_s = 0.00125\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n" + std::string(kResearchSection))));
        });
    invalid_with(
        "invalid MonitorsConfig: MonitorsConfig.__init__() missing 1 "
        "required positional argument: 'grounded_hold_s'",
        [&] {
            discard(cfg::load_physics(stage(
                "missing_grounded_hold.toml",
                "timestep_s = 0.00125\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ncontact_dropout_s = 0.0125\n" + std::string(kResearchSection))));
        });
    invalid_with(
        "invalid MonitorsConfig: MonitorsConfig.__init__() missing 1 "
        "required positional argument: 'contact_dropout_s'",
        [&] {
            discard(cfg::load_physics(stage(
                "missing_contact_dropout.toml",
                "timestep_s = 0.00125\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n" + std::string(kResearchSection))));
        });

    // The monitor holds count whole physics steps — a non-multiple value is
    // a rejected document, not a silently rounded one.
    invalid_with(
        "monitors.grounded_hold_s must be an integer multiple of timestep_s",
        [&] {
            discard(cfg::load_physics(stage(
                "nonmultiple_grounded.toml",
                "timestep_s = 0.00125\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.006\n"
                "contact_dropout_s = 0.0125\n"
                "[seated_climb]\nperiod_s = 0.00125\n"
                "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
                "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
                "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
                "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
                "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
                "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
                "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
                "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\n"
                "stop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
        });
    invalid_with(
        "monitors.contact_dropout_s must be an integer multiple of "
        "timestep_s",
        [&] {
            discard(cfg::load_physics(stage(
                "nonmultiple_dropout.toml",
                "timestep_s = 0.00125\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
                "contact_dropout_s = 0.006\n"
                "[seated_climb]\nperiod_s = 0.00125\n"
                "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
                "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
                "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
                "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
                "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
                "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
                "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
                "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\n"
                "stop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
        });

    // Nonfinite scalars arrive as TOML literals and are rejected by the
    // domain validation the typed decode feeds.
    bt::invalid([&] {
        discard(cfg::load_physics(stage(
            "nonfinite.toml",
            "timestep_s = inf\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
            "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
            "contact_dropout_s = 0.0125\n"
            "[seated_climb]\nenabled = false\nperiod_s = 0.00125\n"
            "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
            "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
            "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
            "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
            "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
            "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
            "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
            "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\n"
            "stop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
    });
    bt::invalid([&] {
        discard(cfg::load_physics(stage(
            "nonfinite2.toml",
            "timestep_s = nan\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
            "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
            "contact_dropout_s = 0.0125\n"
            "[seated_climb]\nenabled = false\nperiod_s = 0.00125\n"
            "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
            "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
            "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
            "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
            "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
            "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
            "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
            "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\n"
            "stop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
    });

    // Integer fields reject floating spellings and booleans.
    bt::invalid([&] {
        discard(cfg::load_physics(
            stage("int_float.toml", "[tires.distributed]\nstation_count = 128.0\n")));
    });
    bt::invalid([&] {
        discard(cfg::load_physics(
            stage("int_bool.toml", "[tires.distributed]\nstation_count = true\n")));
    });

    // Wrong scalar/array shapes for leaf members.
    bt::invalid([&] {
        discard(cfg::load_physics(stage(
            "scalar_array.toml",
            "timestep_s = [0.001]\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
            "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
            "contact_dropout_s = 0.0125\n" + std::string(kResearchSection))));
    });
    bt::invalid([&] {
        discard(cfg::load_physics(stage(
            "tuple_scalar.toml", "[resistance]\npoint_body_m = 0.5\n")));
    });

    // A legacy-mode document fails at the domain boundary — 'physical' is
    // the only accepted physics_mode.
    invalid_with("unknown physics_mode 'legacy'; legacy mode was removed, "
                 "use physical", [&] {
        discard(cfg::load_physics(stage(
            "legacy.toml",
            "physics_mode = \"legacy\"\ntimestep_s = 0.00125\n"
            "control_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
            "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
            "contact_dropout_s = 0.0125\n" + std::string(kResearchSection))));
    });

    // A missing file reports the OS failure with the staged path — a stable
    // diagnostic string (no fallback search).
    const auto absent = temp.path / "does_not_exist.toml";
    try {
        discard(cfg::load_physics(absent));
        bt::require(false, "missing file must throw");
    } catch (const std::system_error& error) {
        bt::require(std::string_view(error.what()).find("does_not_exist.toml") !=
                        std::string_view::npos,
                    "missing-file diagnostic names the path: " +
                        std::string(error.what()));
    }
}

// ---- seated-climb intent timings --------------------------------------------

// The seated-climb intent ticks on the physics clock: its period, reaction
// delay, lean rate and orientation smoothing are required profile members —
// a document that omits one fails the missing-argument check in declaration
// order, and a non-multiple delay or sub-two-tick smoothing is a rejected
// document, never a silently rounded one.
[[gnu::noinline]] void checks_seated_climb_timings() {
    const TempDir temp("bike-t13-seated");
    const auto stage = [&](std::string_view name, std::string_view toml) {
        const auto path = temp.path / std::string(name);
        write_text_file(path, toml);
        return path;
    };
    const std::string_view seated_head =
        "timestep_s = 0.00125\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
        "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.0125\n"
        "[seated_climb]\nenabled = true\n";
    const std::string_view seated_tail =
        "period_s = 0.00125\nreaction_delay_s = 0.15\n"
        "lean_rate_rad_s = 0.5\norientation_tau_s = 0.5\n"
        "surge_budget_s = 15\ntrim_dead_time_s = 0.2\n"
        "min_elbow_flexion_deg = 15\n";
    for (const auto missing : {"period_s", "reaction_delay_s",
                              "lean_rate_rad_s", "orientation_tau_s",
                              "surge_budget_s", "trim_dead_time_s"}) {
        std::string tail(seated_tail);
        const std::string needle = std::string(missing) + " =";
        const auto at = tail.find(needle);
        bt::require(at != std::string::npos, "staged key present");
        tail.erase(at, tail.find('\n', at) - at + 1);
        invalid_with(
            "invalid SeatedClimbConfig: SeatedClimbConfig.__init__() "
            "missing 1 required positional argument: '" +
                std::string(missing) + "'",
            [&] {
                discard(cfg::load_physics(stage(
                    "missing_seated_" + std::string(missing) + ".toml",
                    std::string(seated_head) + tail)));
            });
    }

    invalid_with(
        "reaction_delay_s must be an integer multiple of period_s", [&] {
            discard(cfg::load_physics(stage(
                "nonmultiple_reaction.toml",
                std::string(seated_head) +
                "period_s = 0.00125\nreaction_delay_s = 0.151\n"
                "lean_rate_rad_s = 0.5\norientation_tau_s = 0.5\n"
                "surge_budget_s = 15\ntrim_dead_time_s = 0.2\n"
                "min_elbow_flexion_deg = 15\n"
                "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
                "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
                "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
                "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
                "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\nstop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
        });
    invalid_with(
        "orientation_tau_s must span at least two period_s ticks", [&] {
            discard(cfg::load_physics(stage(
                "short_tau.toml",
                std::string(seated_head) +
                "period_s = 0.00125\nreaction_delay_s = 0.15\n"
                "lean_rate_rad_s = 0.5\norientation_tau_s = 0.002\n"
                "surge_budget_s = 15\ntrim_dead_time_s = 0.2\n"
                "min_elbow_flexion_deg = 15\n"
                "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
                "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
                "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
                "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
                "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\nstop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
        });
    invalid_with(
        "seated_climb.period_s must be an integer multiple of timestep_s",
        [&] {
            discard(cfg::load_physics(stage(
                "nonmultiple_seated_period.toml",
                std::string(seated_head) +
                "period_s = 0.002\nreaction_delay_s = 0.15\n"
                "lean_rate_rad_s = 0.5\norientation_tau_s = 0.5\n"
                "surge_budget_s = 15\ntrim_dead_time_s = 0.2\n"
                "min_elbow_flexion_deg = 15\n"
                "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
                "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
                "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
                "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
                "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\nstop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
        });
    // A disabled intent never ticks: its delay/smoothing carry no
    // whole-tick constraint — only presence and scalar bounds apply.
    {
        const auto disabled = cfg::load_physics(stage(
            "disabled_seated.toml",
            "timestep_s = 0.00125\ncontrol_period_s = 0.005\n"
            "closure_time_constant_s = 0.0025\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
            "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.0125\n"
            "[seated_climb]\nenabled = false\n"
            "period_s = 0.002\nreaction_delay_s = 0.151\n"
            "lean_rate_rad_s = 0.5\norientation_tau_s = 0.001\n"
            "surge_budget_s = 15\ntrim_dead_time_s = 0.2\n"
            "min_elbow_flexion_deg = 15\n"
            "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
            "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
            "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
            "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
            "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\nstop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection)));
        bt::require(!disabled.seated_climb.enabled &&
                        disabled.seated_climb.reaction_delay_s == 0.151,
                    "a disabled intent tolerates non-multiple timings");
    }
}

// ---- crank-reposition timings -----------------------------------------------

// The reposition manoeuvre counts its stall window, cooldown and timeout
// inside PedalingPolicy::update on the physics clock (step.cpp's
// apply_forces passes timestep_s as the tick dt): every knob is a required
// [drive.pedaling] member, and while the mechanism is engaged a timing that
// lands between ticks is a rejected document, never a silently rounded one.
[[gnu::noinline]] void checks_reposition_timings() {
    const TempDir temp("bike-t13-reposition");
    const auto stage = [&](std::string_view name, std::string_view toml) {
        const auto path = temp.path / std::string(name);
        write_text_file(path, toml);
        return path;
    };
    const std::string_view repo_head =
        "timestep_s = 0.00125\ncontrol_period_s = 0.005\n"
        "closure_time_constant_s = 0.0025\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
        "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.0125\n"
        "[seated_climb]\nenabled = false\nperiod_s = 0.00125\n"
        "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
        "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
        "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
        "[drive]\nmotor_clutch = false\n[drive.pedaling]\n";
    const std::string_view repo_tail =
        "reposition_min_effort_nm = 20.0\n"
        "reposition_stall_window_s = 1.0\n"
        "reposition_stall_progress_rad = 0.5\n"
        "reposition_cooldown_s = 1.0\n"
        "reposition_back_rate_rad_s = 3.0\n"
        "reposition_timeout_s = 3.0\n"
        "reposition_phase_tolerance_rad = 0.05\n"
        "reposition_noop_rad = 0.12\n"
        "stop_time_s = 0.35\n";
    for (const auto missing : {"reposition_on_stall", "reposition_min_effort_nm",
                              "reposition_stall_window_s",
                              "reposition_stall_progress_rad",
                              "reposition_cooldown_s",
                              "reposition_back_rate_rad_s",
                              "reposition_timeout_s",
                              "reposition_phase_tolerance_rad",
                              "reposition_noop_rad", "stop_time_s"}) {
        std::string body = "reposition_on_stall = false\n" + std::string(repo_tail);
        const std::string needle = std::string(missing) + " =";
        const auto at = body.find(needle);
        bt::require(at != std::string::npos, "staged key present");
        body.erase(at, body.find('\n', at) - at + 1);
        invalid_with(
            "invalid PedalingConfig: PedalingConfig.__init__() missing 1 "
            "required positional argument: '" + std::string(missing) + "'",
            [&] {
                discard(cfg::load_physics(stage(
                    "missing_repo_" + std::string(missing) + ".toml",
                    std::string(repo_head) + body)));
            });
    }

    // Engaged manoeuvre (spindle-weld rider): each physics-tick-counted
    // timing rejects a non-multiple of timestep_s.
    const std::string_view engaged_head =
        "timestep_s = 0.00125\ncontrol_period_s = 0.005\n"
        "closure_time_constant_s = 0.0025\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
        "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.0125\n"
        "[seated_climb]\nenabled = false\nperiod_s = 0.00125\n"
        "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
        "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
        "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
        "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = true\n"
        "reposition_min_effort_nm = 20.0\n"
        "reposition_stall_progress_rad = 0.5\n"
        "reposition_back_rate_rad_s = 3.0\n"
        "reposition_phase_tolerance_rad = 0.05\n"
        "reposition_noop_rad = 0.12\n"
        "stop_time_s = 0.35\n";
    const std::string_view engaged_tail =
        "[articulated]\npedal_attachment = \"spindle\"\n"
        "balance_dwell_s = 0.5\nbalance_grace_s = 3.0\n"
        "joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n";
    for (const auto& [name, value] : {
             std::pair{"reposition_stall_window_s", "1.0005"},
             {"reposition_cooldown_s", "1.0005"},
             {"reposition_timeout_s", "3.0005"}}) {
        std::string timings{engaged_head};
        for (const auto& [key, valid] : {
                 std::pair{"reposition_stall_window_s", "1.0"},
                 {"reposition_cooldown_s", "1.0"},
                 {"reposition_timeout_s", "3.0"}})
            timings += std::string(key) + " = " +
                (std::string_view(key) == std::string_view(name)
                     ? std::string(value)
                     : std::string(valid)) +
                "\n";
        invalid_with(
            "drive.pedaling." + std::string(name) +
                " must be an integer multiple of timestep_s",
            [&] {
                discard(cfg::load_physics(stage(
                    "nonmultiple_repo_" + std::string(name) + ".toml",
                    timings + std::string(engaged_tail) + std::string(kAssistSection) +
                        std::string(kShiftingSection) + std::string(kResearchSection))));
            });
    }

    // The tick-multiple rule is unconditional: a disengaged manoeuvre
    // still rejects non-multiple timings — the profile declares physics
    // ticks, not intent.
    invalid_with(
        "drive.pedaling.reposition_stall_window_s must be an integer "
        "multiple of timestep_s",
        [&] {
            discard(cfg::load_physics(stage(
                "off_reposition.toml",
                std::string(repo_head) + "reposition_on_stall = false\n" +
                "reposition_min_effort_nm = 20.0\n"
                "reposition_stall_window_s = 1.0005\n"
                "reposition_stall_progress_rad = 0.5\n"
                "reposition_cooldown_s = 1.0005\n"
                "reposition_back_rate_rad_s = 3.0\n"
                "reposition_timeout_s = 3.0005\n"
                "reposition_phase_tolerance_rad = 0.05\n"
                "reposition_noop_rad = 0.12\nstop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
        });

    // The back rate is a positive rate limit, never a counted tick — a
    // non-multiple value is fine, a nonpositive one is not.
    invalid_with("invalid reposition back rate", [&] {
        discard(cfg::load_physics(stage(
            "nonpositive_back_rate.toml",
            std::string(repo_head) + "reposition_on_stall = false\n" +
            "reposition_min_effort_nm = 20.0\n"
            "reposition_stall_window_s = 1.0\n"
            "reposition_stall_progress_rad = 0.5\n"
            "reposition_cooldown_s = 1.0\n"
            "reposition_back_rate_rad_s = -3.0\n"
            "reposition_timeout_s = 3.0\n"
            "reposition_phase_tolerance_rad = 0.05\n"
            "reposition_noop_rad = 0.12\nstop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
    });
    invalid_with(
        "reposition no-op band must be below a quarter crank turn", [&] {
            discard(cfg::load_physics(stage(
                "wide_noop.toml",
                std::string(repo_head) + "reposition_on_stall = false\n" +
                "reposition_min_effort_nm = 20.0\n"
                "reposition_stall_window_s = 1.0\n"
                "reposition_stall_progress_rad = 0.5\n"
                "reposition_cooldown_s = 1.0\n"
                "reposition_back_rate_rad_s = 3.0\n"
                "reposition_timeout_s = 3.0\n"
                "reposition_phase_tolerance_rad = 0.05\n"
                "reposition_noop_rad = 1.6\nstop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
        });
    invalid_with(
        "reposition stall progress must be below half a crank turn", [&] {
            discard(cfg::load_physics(stage(
                "wide_stall_progress.toml",
                std::string(repo_head) + "reposition_on_stall = false\n" +
                "reposition_min_effort_nm = 20.0\n"
                "reposition_stall_window_s = 1.0\n"
                "reposition_stall_progress_rad = 3.2\n"
                "reposition_cooldown_s = 1.0\n"
                "reposition_back_rate_rad_s = 3.0\n"
                "reposition_timeout_s = 3.0\n"
                "reposition_phase_tolerance_rad = 0.05\n"
                "reposition_noop_rad = 0.12\nstop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection))));
        });

    // Valid engaged control: spindle rider, every knob present, all counted
    // timings whole ticks — the document resolves.
    {
        const auto engaged = cfg::load_physics(stage(
            "engaged_reposition.toml",
            std::string(engaged_head) +
            "reposition_stall_window_s = 1.0\n"
            "reposition_cooldown_s = 1.0\n"
            "reposition_timeout_s = 3.0\n" + std::string(engaged_tail) + std::string(kAssistSection) + std::string(kShiftingSection) + std::string(kResearchSection)));
        bt::require(engaged.drive.pedaling.reposition_on_stall &&
                        engaged.drive.pedaling.reposition_stall_window_s == 1.0,
                    "engaged manoeuvre resolves");
    }
}

// ---- research-mode timings ----------------------------------------------------

// The research environment counts its control period, actuator delay and
// sensor sample period on the physics clock (environment.cpp's integer_steps
// calls): all three are required [research] members with the same whole-step
// rule the physics control period carries, never a silently rounded value.
// Sensor latency and maximum age bound delivery by timestamp cutoff, not by
// a counted tick — presence and scalar bounds only.
[[gnu::noinline]] void checks_research_timings() {
    const TempDir temp("bike-t13-research");
    const auto stage = [&](std::string_view name, std::string_view toml) {
        const auto path = temp.path / std::string(name);
        write_text_file(path, toml);
        return path;
    };
    const std::string_view research_head =
        "timestep_s = 0.00125\ncontrol_period_s = 0.005\n"
        "closure_time_constant_s = 0.0025\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
        "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.0125\n"
        "[seated_climb]\nperiod_s = 0.00125\n"
        "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
        "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
        "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
        "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
        "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
        "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
        "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
        "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\n"
        "stop_time_s = 0.35\n"
        "[research]\n";
    const std::string_view research_body =
        "control_period_s = 0.01\nactuator_delay_s = 0.005\n"
        "sensor_latency_s = 0.01\nsensor_maximum_age_s = 0.1\n"
        "sample_period_s = 0.005\nrecord_decimation = 80\n";

    invalid_with(
        "invalid SimulationPhysicsConfig: SimulationPhysicsConfig.__init__() "
        "missing 1 required positional argument: 'research'",
        [&] {
            discard(cfg::load_physics(stage(
                "missing_research.toml",
                "timestep_s = 0.00125\ncontrol_period_s = 0.005\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
                "contact_dropout_s = 0.0125\n")));
        });
    for (const auto missing : {"control_period_s", "actuator_delay_s",
                              "sensor_latency_s", "sensor_maximum_age_s",
                              "sample_period_s", "record_decimation"}) {
        std::string body(research_body);
        const std::string needle = std::string(missing) + " =";
        const auto at = body.find(needle);
        bt::require(at != std::string::npos, "staged key present");
        body.erase(at, body.find('\n', at) - at + 1);
        invalid_with(
            "invalid ResearchConfig: ResearchConfig.__init__() missing 1 "
            "required positional argument: '" + std::string(missing) + "'",
            [&] {
                discard(cfg::load_physics(stage(
                    "missing_research_" + std::string(missing) + ".toml",
                    std::string(research_head) + body + std::string(kAuxTimings))));
            });
    }

    // The three tick-counted timings reject a non-multiple of timestep_s.
    for (const auto& [name, value] : {
             std::pair{"control_period_s", "0.0101"},
             {"actuator_delay_s", "0.0051"},
             {"sample_period_s", "0.0051"}}) {
        std::string body(research_body);
        const std::string needle = std::string(name) + " = ";
        const auto at = body.find(needle);
        bt::require(at != std::string::npos, "staged key present");
        const auto end = body.find('\n', at);
        body.replace(at, end - at, needle + std::string(value));
        invalid_with(
            "research." + std::string(name) +
                " must be an integer multiple of timestep_s",
            [&] {
                discard(cfg::load_physics(stage(
                    "nonmultiple_research_" + std::string(name) + ".toml",
                    std::string(research_head) + body + std::string(kAuxTimings))));
            });
    }

    // Latency and maximum age are timestamp cutoffs, not counted ticks —
    // non-multiple values resolve and survive to the emitted document.
    {
        std::string body(research_body);
        for (const auto& [name, value] : {
                 std::pair{"sensor_latency_s", "0.0115"},
                 {"sensor_maximum_age_s", "0.093"}}) {
            const std::string needle = std::string(name) + " = ";
            const auto at = body.find(needle);
            bt::require(at != std::string::npos, "staged key present");
            const auto end = body.find('\n', at);
            body.replace(at, end - at, needle + std::string(value));
        }
        const auto loaded = cfg::load_physics(
            stage("research_cutoffs.toml",
                  std::string(research_head) + body + std::string(kAuxTimings)));
        const std::string flat = canonical(cfg::to_value(loaded));
        bt::require(flat.find("\"sensor_latency_s\":0.0115") !=
                        std::string::npos &&
                    flat.find("\"sensor_maximum_age_s\":0.093") !=
                        std::string::npos,
                    "timestamp cutoffs carry no whole-tick constraint");
    }
}

// ---- drive holds, balance windows and smoothing ------------------------------

// The remaining profile delays and holds count discrete loop ticks: the
// motor run-on, shift cooldown/cut, coast stop and the balance dwell/grace
// windows count physics steps (timestep_s); the seated-climb trim dead time
// and surge budget count intent ticks (seated_climb.period_s, engaged only
// while the intent runs). Every smoothing — the assist lag, the two cadence
// EMAs and the pneumatic rate relaxation — advances on the physics step and
// must span at least two ticks. Rate limits (N·m/s, rad/s) carry no rule.
[[gnu::noinline]] void checks_drive_timings() {
    const TempDir temp("bike-t13-drive");
    const auto stage = [&](std::string_view name, std::string_view toml) {
        const auto path = temp.path / std::string(name);
        write_text_file(path, toml);
        return path;
    };
    const std::string_view head =
        "timestep_s = 0.00125\ncontrol_period_s = 0.005\n"
        "closure_time_constant_s = 0.0025\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
        "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.0125\n"
        "[seated_climb]\nenabled = false\nperiod_s = 0.00125\n"
        "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
        "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
        "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
        "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
        "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
        "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
        "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
        "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\n";
    // `middle` lands inside the still-open [drive.pedaling] table until the
    // first [table] header; kResearchSection closes the document.
    const auto doc = [&](std::string_view middle) {
        return std::string(head) + std::string(middle) +
               std::string(kResearchSection);
    };

    // Physics-step-counted holds reject a non-multiple of timestep_s. Each
    // staged document carries every other required timing so the probe
    // isolates the one field under test.
    for (const auto& [snippet, field] : {
             std::pair{std::string("stop_time_s = 0.35\n") +
                           "[drive.assist]\nrelease_hold_s = 0.3005\n" +
                           std::string(kShiftingSection) + std::string(kArticulatedSection),
                       "drive.assist.release_hold_s"},
             std::pair{std::string("stop_time_s = 0.35\n") + std::string(kAssistSection) +
                           "[drive.shifting]\nshift_cooldown_s = 0.2505\n"
                           "shift_cut_duration_s = 0.2\n" +
                           std::string(kArticulatedSection),
                       "drive.shifting.shift_cooldown_s"},
             std::pair{std::string("stop_time_s = 0.35\n") + std::string(kAssistSection) +
                           "[drive.shifting]\nshift_cooldown_s = 0.4\n"
                           "shift_cut_duration_s = 0.2005\n" +
                           std::string(kArticulatedSection),
                       "drive.shifting.shift_cut_duration_s"},
             std::pair{std::string("stop_time_s = 0.3505\n") + std::string(kAuxTimings),
                       "drive.pedaling.stop_time_s"},
             std::pair{std::string("stop_time_s = 0.35\n") + std::string(kAssistSection) +
                           std::string(kShiftingSection) +
                           "[articulated]\nbalance_dwell_s = 0.5005\n"
                           "balance_grace_s = 3.0\n"
                           "joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n",
                       "articulated.balance_dwell_s"},
             std::pair{std::string("stop_time_s = 0.35\n") + std::string(kAssistSection) +
                           std::string(kShiftingSection) +
                           "[articulated]\nbalance_dwell_s = 0.5\n"
                           "balance_grace_s = 3.0005\n"
                           "joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n",
                       "articulated.balance_grace_s"}}) {
        invalid_with(
            std::string(field) +
                " must be an integer multiple of timestep_s",
            [&] {
                discard(cfg::load_physics(stage(
                    "nonmultiple_" + std::string(field) + ".toml",
                    doc(snippet))));
            });
    }

    // Physics-step smoothings reject a tau shorter than two ticks; a zero
    // tau disables the filter outright and carries no constraint.
    for (const auto& [snippet, field] : {
             std::pair{std::string("stop_time_s = 0.35\n") +
                           "[drive.assist]\ntau = 0.002\nrelease_hold_s = 0.3\n" +
                           std::string(kShiftingSection) + std::string(kArticulatedSection),
                       "drive.assist.tau"},
             std::pair{std::string("stop_time_s = 0.35\n"
                                   "coast_cadence_tau_s = 0.002\n") +
                           std::string(kAuxTimings),
                       "drive.pedaling.coast_cadence_tau_s"},
             std::pair{std::string("stop_time_s = 0.35\n") + std::string(kAssistSection) +
                           "[drive.shifting]\ncadence_smoothing_tau_s = 0.002\n"
                           "shift_cooldown_s = 0.4\nshift_cut_duration_s = 0.2\n" +
                           std::string(kArticulatedSection),
                       "drive.shifting.cadence_smoothing_tau_s"},
             std::pair{std::string("stop_time_s = 0.35\n") + std::string(kAuxTimings) +
                           "[tires]\nbackend = \"pneumatic\"\n"
                           "surface_mode = \"track\"\n"
                           "[tires.pneumatic.front]\nrate_relaxation_s = 0.002\n",
                       "tires.pneumatic.front.rate_relaxation_s"},
             std::pair{std::string("stop_time_s = 0.35\n") + std::string(kAuxTimings) +
                           "[tires]\nbackend = \"pneumatic\"\n"
                           "surface_mode = \"track\"\n"
                           "[tires.pneumatic.rear]\nrate_relaxation_s = 0.002\n",
                       "tires.pneumatic.rear.rate_relaxation_s"}}) {
        invalid_with(
            std::string(field) +
                " must span at least two timestep_s ticks",
            [&] {
                discard(cfg::load_physics(stage(
                    "short_" + std::string(field) + ".toml",
                    doc(snippet))));
            });
    }

    // The assist lag is the RESOLVED torque tau: a named motor profile
    // supplies it, so a declared tau that would fail is ignored while the
    // profile value is what the two-tick rule measures.
    {
        const auto ignored = cfg::load_physics(stage(
            "assist_profile_tau.toml",
            doc(std::string("stop_time_s = 0.35\n[drive.assist]\n"
                            "profile = \"bosch_cx_gen4\"\ntau = 0.001\n"
                            "release_hold_s = 0.3\n") +
                std::string(kShiftingSection) + std::string(kArticulatedSection))));
        bt::require(ignored.drive.assist.tau == 0.001,
                    "declared assist tau survives beside a profile");
    }
    // At dt = 0.03 s the Bosch tau of .04 s spans 1.33 ticks while a
    // declared .08 s would span 2.67 — only the resolved value is checked.
    invalid_with(
        "drive.assist.tau must span at least two timestep_s ticks", [&] {
            discard(cfg::load_physics(stage(
                "assist_resolved_tau.toml",
                "timestep_s = 0.03\ncontrol_period_s = 0.03\n"
                "closure_time_constant_s = 0.06\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
                "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.3\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.06\nhand_lag_s = 0.06\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.12\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.03\n"
                "contact_dropout_s = 0.03\n"
                "[seated_climb]\nenabled = false\nperiod_s = 0.03\n"
                "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
                "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
                "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
                "[drive.assist]\nprofile = \"bosch_cx_gen4\"\ntau = 0.08\n"
                "release_hold_s = 0.3\n"
                "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
                "reposition_min_effort_nm = 20.0\n"
                "reposition_stall_window_s = 0.99\n"
                "reposition_stall_progress_rad = 0.5\n"
                "reposition_cooldown_s = 0.99\n"
                "reposition_back_rate_rad_s = 3.0\n"
                "reposition_timeout_s = 3.0\n"
                "reposition_phase_tolerance_rad = 0.05\n"
                "reposition_noop_rad = 0.12\n"
                "stop_time_s = 0.36\ncoast_cadence_tau_s = 0.0\n"
                "[drive.shifting]\nshift_cooldown_s = 0.42\n"
                "shift_cut_duration_s = 0.21\n"
                "cadence_smoothing_tau_s = 0.35\n"
                "[articulated]\nbalance_dwell_s = 0.6\n"
                "balance_grace_s = 3.0\n"
                "joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n"
                "[research]\ncontrol_period_s = 0.03\n"
                "actuator_delay_s = 0.0\nsensor_latency_s = 0.01\n"
                "sensor_maximum_age_s = 0.1\nsample_period_s = 0.03\n"
                "record_decimation = 80\n")));
        });

    // The seated-climb trim dead time and surge budget count intent ticks:
    // engaged, each rejects a non-multiple of period_s; disengaged, the
    // dormant values carry no whole-tick constraint.
    const std::string_view climb_head =
        "timestep_s = 0.00125\ncontrol_period_s = 0.005\n"
        "closure_time_constant_s = 0.0025\n"
        "drive_mode = \"articulated_effort\"\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
        "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.0125\n"
        "[drive]\nmotor_clutch = true\n[drive.pedaling]\nreposition_on_stall = false\n"
        "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
        "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
        "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
        "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\n"
        "stop_time_s = 0.35\n"
        "[seated_climb]\nenabled = true\nperiod_s = 0.00125\n"
        "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
        "orientation_tau_s = 0.5\nmin_elbow_flexion_deg = 15\n";
    for (const auto& [key, value, peer] : {
             std::tuple{"trim_dead_time_s", "0.201", "surge_budget_s = 15.0\n"},
             {"surge_budget_s", "15.001", "trim_dead_time_s = 0.2\n"}}) {
        invalid_with(
            std::string(key) +
                " must be an integer multiple of period_s",
            [&] {
                discard(cfg::load_physics(stage(
                    "nonmultiple_" + std::string(key) + ".toml",
                    std::string(climb_head) +
                        std::string(key) + " = " + value + "\n" +
                        std::string(peer) + std::string(kAuxTimings) +
                        std::string(kResearchSection))));
            });
    }
    {
        const auto climb = cfg::load_physics(stage(
            "climb_timings.toml",
            std::string(climb_head) +
            "trim_dead_time_s = 0.4\nsurge_budget_s = 30.0\n" +
            std::string(kAuxTimings) + std::string(kResearchSection)));
        bt::require(climb.seated_climb.trim_dead_time_s == 0.4 &&
                        climb.seated_climb.surge_budget_s == 30.0,
                    "whole-tick intent timings resolve");
        const auto disabled = cfg::load_physics(stage(
            "disabled_climb_timings.toml",
            "timestep_s = 0.00125\ncontrol_period_s = 0.005\n"
            "closure_time_constant_s = 0.0025\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
            "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\n"
            "contact_dropout_s = 0.0125\n"
            "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
            "reposition_min_effort_nm = 20.0\n"
            "reposition_stall_window_s = 1.0\n"
            "reposition_stall_progress_rad = 0.5\n"
            "reposition_cooldown_s = 1.0\n"
            "reposition_back_rate_rad_s = 3.0\n"
            "reposition_timeout_s = 3.0\n"
            "reposition_phase_tolerance_rad = 0.05\n"
            "reposition_noop_rad = 0.12\n"
            "stop_time_s = 0.35\n"
            "[seated_climb]\nenabled = false\nperiod_s = 0.00125\n"
            "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
            "orientation_tau_s = 0.5\nmin_elbow_flexion_deg = 15\n"
            "trim_dead_time_s = 0.201\nsurge_budget_s = 15.001\n" +
            std::string(kAuxTimings) + std::string(kResearchSection)));
        bt::require(disabled.seated_climb.trim_dead_time_s == 0.201,
                    "a disabled intent tolerates non-multiple timings");
    }

    // Valid control: every counted timing at a whole tick and every
    // smoothing at exactly two ticks or more resolves through the loader.
    {
        const auto valid = cfg::load_physics(stage(
            "drive_timings.toml",
            doc("stop_time_s = 0.35\ncoast_cadence_tau_s = 0.0025\n"
                "[drive.assist]\nrelease_hold_s = 0.3\ntau = 0.0025\n"
                "[drive.shifting]\nshift_cooldown_s = 0.25\n"
                "shift_cut_duration_s = 0.2\n"
                "cadence_smoothing_tau_s = 0.0025\n"
                "[articulated]\nbalance_dwell_s = 0.5\n"
                "balance_grace_s = 3.0\n"
                "joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n"
                "[tires]\nbackend = \"pneumatic\"\n"
                "surface_mode = \"track\"\n"
                "[tires.pneumatic.front]\nrate_relaxation_s = 0.2\n"
                "[tires.pneumatic.rear]\nrate_relaxation_s = 0.2\n")));
        bt::require(valid.drive.assist.release_hold_s == 0.3 &&
                        valid.drive.shifting.shift_cooldown_s == 0.25 &&
                        valid.drive.pedaling.stop_time_s == 0.35 &&
                        valid.articulated.balance_dwell_s == 0.5 &&
                        valid.tires.pneumatic.front.rate_relaxation_s == 0.2,
                    "whole-tick timings and two-tick smoothings resolve");
    }
}

// ---- required hold members ---------------------------------------------------

// Every migrated hold/delay is a required profile member: a document that
// omits one fails the missing-argument check in declaration order — the
// same contract the seated-climb, pedaling and research timings carry.
// (The seated-climb pair and pedaling stop_time_s probes live in their own
// sections' missing-key loops.)
[[gnu::noinline]] void checks_hold_members() {
    const TempDir temp("bike-t13-holdmembers");
    const auto stage = [&](std::string_view name, std::string_view toml) {
        const auto path = temp.path / std::string(name);
        write_text_file(path, toml);
        return path;
    };
    const std::string full =
        "timestep_s = 0.00125\ncontrol_period_s = 0.005\n"
        "closure_time_constant_s = 0.0025\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n"
        "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\ngrounded_hold_s = 0.005\ncontact_dropout_s = 0.0125\n"
        "[seated_climb]\nenabled = false\nperiod_s = 0.00125\n"
        "reaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\n"
        "orientation_tau_s = 0.5\nsurge_budget_s = 15\n"
        "trim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15\n"
        "[drive]\nmotor_clutch = false\n[drive.pedaling]\nreposition_on_stall = false\n"
        "reposition_min_effort_nm = 20.0\nreposition_stall_window_s = 1.0\n"
        "reposition_stall_progress_rad = 0.5\nreposition_cooldown_s = 1.0\n"
        "reposition_back_rate_rad_s = 3.0\nreposition_timeout_s = 3.0\n"
        "reposition_phase_tolerance_rad = 0.05\nreposition_noop_rad = 0.12\n"
        "stop_time_s = 0.35\n" + std::string(kAuxTimings) + std::string(kResearchSection);
    for (const auto& [record, missing] : {
             std::pair{"PhysicalDriveConfig", "motor_clutch"},
             {"AssistConfig", "release_hold_s"},
             {"ShiftingConfig", "shift_cooldown_s"},
             {"ShiftingConfig", "shift_cut_duration_s"},
             {"ArticulatedConfig", "balance_dwell_s"},
             {"ArticulatedConfig", "balance_grace_s"},
             {"ArticulatedConfig", "joint_kp_nm_rad"},
             {"ArticulatedConfig", "joint_kd_nms_rad"},
             {"ArticulatedConfig", "leg_active_damping_nms_rad"}}) {
        std::string body(full);
        const std::string needle = std::string(missing) + " =";
        const auto at = body.find(needle);
        bt::require(at != std::string::npos, "staged key present");
        body.erase(at, body.find('\n', at) - at + 1);
        invalid_with(
            "invalid " + std::string(record) + ": " + std::string(record) +
                ".__init__() missing 1 required positional argument: '" +
                std::string(missing) + "'",
            [&] {
                discard(cfg::load_physics(
                    stage("missing_" + std::string(missing) + ".toml",
                          body)));
            });
    }
}

// ---- merge precedence --------------------------------------------------------

[[gnu::noinline]] void checks_precedence() {
    const auto base = cfg::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));

    // Absent explicit values never override the file's resolved values.
    {
        const auto same = cfg::resolve_physics(make_object({}), base);
        bt::require(canonical(cfg::to_value(same)) ==
                        canonical(cfg::to_value(base)),
                    "empty overrides preserve the resolved file config");
    }

    // Explicit --dt beats the file value; untouched fields keep file values.
    // The override tick keeps every counted period an integer multiple
    // (intent 5x, control 20x, grounded 20x, dropout 50x at 0.25 ms).
    {
        const auto over = cfg::resolve_physics(
            make_object({{"timestep_s", make_number("0.00025")}}), base);
        bt::require(over.timestep_s == 0.00025, "explicit --dt wins");
        bt::require(over.control_period_s == base.control_period_s,
                    "unmentioned fields keep file values");
        bt::require(over.articulated.joint_envelope_path ==
                        base.articulated.joint_envelope_path,
                    "resolved asset paths survive an unrelated override");
    }

    // Deep nested merge: one leaf overridden, every sibling preserved.
    {
        const auto over = cfg::resolve_physics(
            make_object({{"drive", make_object({{"pedaling",
                                                make_object({{"mash_torque_nm",
                                                              make_number("70")}})}})}}),
            base);
        bt::require(over.drive.pedaling.mash_torque_nm == 70.0,
                    "nested leaf overridden");
        bt::require(over.drive.pedaling.enabled ==
                        base.drive.pedaling.enabled,
                    "nested sibling preserved");
        bt::require(over.drive.gearing.front_teeth ==
                        base.drive.gearing.front_teeth,
                    "deeper sibling preserved");
    }

    // Explicit null is skipped entirely — it is 'flag absent', not 'set to
    // null' (resolution.py _merge: `if value is None: continue`).
    {
        const auto over = cfg::resolve_physics(
            make_object({{"timestep_s", make_null()},
                         {"equilibrium_state_file", make_null()}}),
            base);
        bt::require(over.timestep_s == base.timestep_s,
                    "null override is skipped");
        bt::require(over.equilibrium_state_file ==
                        base.equilibrium_state_file,
                    "null optional override is skipped");
    }

    // An engaged optional can still be set by an explicit value; the welded
    // base's seated-climb contract gates drive_mode, so the probe disables it
    // in the same override object — deep-merge plus leaf replacement.
    {
        const auto over = cfg::resolve_physics(
            make_object({{"seated_climb",
                          make_object({{"enabled", make_bool(false)}})},
                         {"drive_mode", make_text("coast")},
                         {"drive",
                          make_object({{"motor_clutch", make_bool(false)}})}}),
            base);
        bt::require(over.drive_mode.has_value() &&
                        *over.drive_mode == "coast",
                    "explicit drive_mode override");
        bt::require(!over.seated_climb.enabled,
                    "deep-merged flag override");
        bt::require(over.seated_climb.target_crank_power_w ==
                        base.seated_climb.target_crank_power_w,
                    "untouched seated_climb leaf preserved");
    }
    // The same drive_mode override WITHOUT disabling seated climb is a domain
    // error — resolve_physics constructs the merged record, so __post_init__
    // invariants apply to overrides exactly as the Python resolver applies
    // them (model_config.py: seated_climb.enabled requires
    // drive_mode='articulated_effort').
    invalid_with(
        "seated climb requires the articulated physical effort drive", [&] {
            discard(cfg::resolve_physics(
                make_object({{"drive_mode", make_text("coast")}}), base));
        });

    // Overrides arrive as bike::io::Value — a JSON document parses into the
    // same boundary the TOML tree uses, so JSON-spelled explicit values take
    // the identical merge path (the 'JSON loading' half of the task brief).
    {
        const Value json_overrides = io::parse_json(
            R"({"timestep_s": 0.00025, "drive": {"pedaling": {"mash_torque_nm": 61.5}}})");
        const auto over = cfg::resolve_physics(json_overrides, base);
        bt::require(over.timestep_s == 0.00025, "JSON timestep override");
        bt::require(over.drive.pedaling.mash_torque_nm == 61.5,
                    "JSON nested override");
        bt::require(over.drive.gearing.front_teeth ==
                        base.drive.gearing.front_teeth,
                    "JSON merge keeps untouched leaf");
    }

    // Overrides must be a mapping — resolution.py's own guard text.
    invalid_with("configuration and overrides must be mappings", [&] {
        discard(cfg::resolve_physics(make_number("3"), base));
    });
    invalid_with("configuration and overrides must be mappings", [&] {
        discard(cfg::resolve_physics(make_array({}), base));
    });

    // The two-layer mapping carries the base's RESOLVED values verbatim:
    // Python's resolve_physics_config(asdict(resolved_base), overrides)
    // throws when a resolved field violates the mode — an override setting
    // pitch_assist=true on the resolved physical base raises 'external
    // pitch assist is forbidden in physical mode'. An unresolved
    // SimulationPhysicsConfig{} base instead carries None for the derived
    // fields, so post_init resolves them (drive_mode='coast',
    // pitch_assist=False). Both behaviors locked.
    invalid_with(
        "external pitch assist is forbidden in physical mode", [] {
            discard(cfg::resolve_physics(
                make_object({{"pitch_assist", make_bool(true)}}),
                cfg::default_physics()));
        });
    {
        auto unresolved_base = cfg::SimulationPhysicsConfig{};
        unresolved_base.timestep_s = 0.0005;
        unresolved_base.control_period_s = 0.005;
        unresolved_base.monitors.grounded_hold_s = 0.005;
        unresolved_base.monitors.contact_dropout_s = 0.005;
        unresolved_base.seated_climb.period_s = .00125;
        unresolved_base.seated_climb.reaction_delay_s = .15;
        unresolved_base.seated_climb.lean_rate_rad_s = .5;
        unresolved_base.seated_climb.orientation_tau_s = .5;
        unresolved_base.seated_climb.min_elbow_flexion_deg = 15.0;
        unresolved_base.seated_climb.surge_budget_s = 15.;
        unresolved_base.seated_climb.trim_dead_time_s = .2;
        unresolved_base.drive.assist.release_hold_s = .3;
        unresolved_base.drive.pedaling.stop_time_s = .35;
        unresolved_base.drive.pedaling.reposition_min_effort_nm = 20.;
        unresolved_base.drive.pedaling.reposition_stall_window_s = 1.;
        unresolved_base.drive.pedaling.reposition_stall_progress_rad = .5;
        unresolved_base.drive.pedaling.reposition_cooldown_s = 1.;
        unresolved_base.drive.pedaling.reposition_back_rate_rad_s = 3.;
        unresolved_base.drive.pedaling.reposition_timeout_s = 3.;
        unresolved_base.drive.pedaling.reposition_phase_tolerance_rad = .05;
        unresolved_base.drive.pedaling.reposition_noop_rad = .12;
        unresolved_base.drive.shifting.shift_cooldown_s = .4;
        unresolved_base.drive.shifting.shift_cut_duration_s = .2;
        unresolved_base.articulated.balance_dwell_s = .5;
        unresolved_base.articulated.balance_grace_s = 3.;
        unresolved_base.articulated.joint_kp_nm_rad = 600.;
        unresolved_base.articulated.joint_kd_nms_rad = 15.;
        unresolved_base.articulated.leg_active_damping_nms_rad = 35.;
        unresolved_base.equilibrium_crank_hold_kp_nm_rad = 20000.;
        unresolved_base.equilibrium_crank_hold_kd_nms_rad = 88.;
        // [rider_launch]
        unresolved_base.rider_launch = {.start_cog_grade_breaks = {},
                                        .start_cog_teeth = {24},
                                        .start_pedal_angle_rad = 0.,
                                        .start_lean_grades = {0.},
                                        .start_lean_rad = {0.},
                                        .plan_enabled = false,
                                        .standstill_speed_mps = .1,
                                        .standstill_crank_rpm = 5.,
                                        .effort_rate_nm_s = 40.,
                                        .effort_cap_asphalt_nm = 60.,
                                        .effort_cap_hardpack_nm = 60.,
                                        .effort_cap_loose_nm = 60.,
                                        .effort_cap_wet_nm = 60.,
                                        .effort_cap_scree_nm = 60.,
                                        .effort_cap_grades = {0.},
                                        .effort_cap_grade_nm = {1000.},
                                        .effort_rise_speed_mps = 2.5,
                                        .release_torque_grades = {0., .2},
                                        .release_torque_nm = {0., 30.},
                                        .brake_release_s = .15,
                                        .max_hold_s = 1.5,
                                        .upshift_crank_rpm = 82.,
                                        .shift_retry_s = .25,
                                        .end_speed_mps = 3.,
                                        .end_crank_rpm = 60.,
                                        .end_hold_s = .3,
                                        .max_launch_s = 6.,
                                        .handover_s = .9};
        unresolved_base.front_lift_reaction = {.hand_window_s = .5, .hand_lag_s = .5, .hand_drop_ratio = .7,
                                               .pitch_rate_up_rad_s = .5, .pitch_rad = .17,
                                               .decision_s = .1, .ease_enabled = false,
                                               .lean_enabled = false, .rear_brake_tap_enabled = false,
                                               .ease_fraction = .7, .ease_rate_per_s = 5.,
                                               .lean_add_rad = .1, .lean_rate_rad_s = .5,
                                               .clear_s = .3};
        unresolved_base.research.control_period_s = .01;
        unresolved_base.research.actuator_delay_s = .005;
        unresolved_base.research.sensor_latency_s = .01;
        unresolved_base.research.sensor_maximum_age_s = .1;
        unresolved_base.research.sample_period_s = .005;
        unresolved_base.research.record_decimation = 80;
        const auto unresolved = cfg::resolve_physics(
            make_object({{"physics_mode", make_text("physical")}}),
            unresolved_base);
        bt::require(unresolved.drive_mode.has_value() &&
                        *unresolved.drive_mode == "coast",
                    "unresolved base resolves drive_mode");
        bt::require(unresolved.pitch_assist.has_value() &&
                        !*unresolved.pitch_assist,
                    "physical mode resolves pitch_assist false");
    }
}

// ---- asset path resolution -------------------------------------------------

[[gnu::noinline]] void checks_paths() {
    const auto config = cfg::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));
    bt::require(config.articulated.joint_envelope_path.has_value() &&
                    *config.articulated.joint_envelope_path ==
                        bt::fixture("config/rider_joint_envelope_anatomical.json")
                            .string(),
                "relative joint_envelope_path anchors at the TOML dir");
    bt::require(config.articulated.joint_strength_path.has_value() &&
                    *config.articulated.joint_strength_path ==
                        bt::fixture("config/rider_strength_reference.json")
                            .string(),
                "relative joint_strength_path anchors at the TOML dir");

    // resolve_config_paths in isolation: relative names resolve against the
    // declared directory; absolute spellings pass through byte-identical —
    // the 'same asset identity' contract for either spelling.
    {
        const auto dir = bt::fixture("config");
        const auto relative_asset = std::filesystem::weakly_canonical(
            dir / "rider_joint_envelope_anatomical.json");
        Value doc = make_object(
            {{"articulated",
              make_object({{"joint_envelope_path",
                            make_text("rider_joint_envelope_anatomical.json")}})}});
        cfg::detail::resolve_config_paths(doc, dir);
        const Value* leaf = member_opt(doc, "articulated", "doc");
        const Value* resolved =
            member_opt(*leaf, "joint_envelope_path", "doc.articulated");
        bt::require(resolved && std::get<std::string>(resolved->data) ==
                                    relative_asset.string(),
                    "relative asset resolves to canonical path");

        Value absolute = make_object(
            {{"articulated",
              make_object({{"joint_envelope_path",
                            make_text(relative_asset.string())}})}});
        cfg::detail::resolve_config_paths(
            absolute, std::filesystem::temp_directory_path());
        const Value* abs_leaf = member_opt(absolute, "articulated", "doc");
        const Value* abs_resolved = member_opt(
            *abs_leaf, "joint_envelope_path", "doc.articulated");
        bt::require(abs_resolved &&
                        std::get<std::string>(abs_resolved->data) ==
                            relative_asset.string(),
                    "absolute spelling passes through unchanged");
        bt::require(std::get<std::string>(resolved->data) ==
                        std::get<std::string>(abs_resolved->data),
                    "relative and absolute spellings name the same asset");
    }

    // Paths with spaces and nonexistent named assets keep a stable,
    // directory-anchored diagnostic spelling — resolution never checks
    // existence (the asset owner opens it later).
    {
        const TempDir temp("bike-t13-space dir");
        const auto spaced = temp.path / "assets with spaces";
        std::filesystem::create_directories(spaced);
        Value doc = make_object(
            {{"articulated",
              make_object(
                  {{"joint_envelope_path",
                    make_text("missing envelope dir/envelope.json")},
                   {"joint_strength_path", make_null()}})},
             {"equilibrium_state_file",
              make_text("missing poses/pose.npz")}});
        cfg::detail::resolve_config_paths(doc, spaced);
        const auto expect = [&](const Value& section, std::string_view key,
                                const std::filesystem::path& want) {
            const Value* leaf = member_opt(section, key, "section");
            bt::require(leaf != nullptr, "missing path member");
            bt::require(std::get<std::string>(leaf->data) == want.string(),
                        "diagnostic path is the anchored spelling: " +
                            std::get<std::string>(leaf->data));
        };
        const Value* articulated = member_opt(doc, "articulated", "doc");
        // Path.resolve() canonicalizes the existing prefix (macOS /var is a
        // symlink to /private/var) then appends the nonexistent tail — the
        // same weakly_canonical contract.
        expect(*articulated, "joint_envelope_path",
               std::filesystem::weakly_canonical(
                   spaced / "missing envelope dir" / "envelope.json"));
        expect(doc, "equilibrium_state_file",
               std::filesystem::weakly_canonical(spaced / "missing poses" /
                                                 "pose.npz"));
        // A null member is left alone — Python's `if path is not None` gate.
        const Value* strength =
            member_opt(*articulated, "joint_strength_path", "doc.articulated");
        bt::require(strength != nullptr && io::is_null(*strength),
                    "null strength path untouched");
    }

    // '..' traversal resolves lexically through the parent — Path.resolve()
    // semantics, not a traversal block.
    {
        const TempDir temp("bike-t13-dotdot");
        const auto nested = temp.path / "sub" / "dir";
        std::filesystem::create_directories(nested);
        Value doc = make_object(
            {{"equilibrium_state_file", make_text("../pose.npz")}});
        cfg::detail::resolve_config_paths(doc, nested);
        const Value* leaf = member_opt(doc, "equilibrium_state_file", "doc");
        bt::require(std::get<std::string>(leaf->data) ==
                        std::filesystem::weakly_canonical(temp.path / "sub" /
                                                          "pose.npz")
                            .string(),
                    "dotdot resolves lexically");
    }
}

// ---- TOML writer -------------------------------------------------------------

[[gnu::noinline]] void checks_writer() {
    const auto config = cfg::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));
    const std::string toml = cfg::write_physics_toml(config);
    bt::require(toml.starts_with(
                    "# Explicit candidate; not real-world safety "
                    "validation.\n"),
                "candidate banner");
    bt::require(toml.find("[tires.front]") != std::string::npos &&
                    toml.find("[drive.assist]") != std::string::npos,
                "nested tables emitted");

    // Write -> read round-trip preserves the effective configuration
    // bit-for-bit through the resolved encoding.
    const TempDir temp("bike-t13-write");
    const auto destination = temp.path / "candidate profile.toml";
    write_text_file(destination, toml);
    const auto reread = cfg::load_physics(destination);
    bt::require(canonical(cfg::to_value(reread)) ==
                    canonical(cfg::to_value(config)),
                "write/read round-trip preserves the effective config");
    bt::require(cfg::write_physics_toml(reread) == toml,
                "writer output is a fixed point");

    // The exclusive destination contract: the platform write seam refuses a
    // pre-existing name — the FileExistsError half of the source writer.
    pf::write_exclusive(temp.path / "second.toml",
                        std::as_bytes(std::span(toml)));
    bool refused = false;
    try {
        pf::write_exclusive(temp.path / "second.toml",
                            std::as_bytes(std::span(toml)));
    } catch (const std::system_error&) {
        refused = true;
    }
    bt::require(refused, "exclusive destination refuses overwrite");

    // Defaults round-trip too — a write of the declared-default config reads
    // back to the identical post_init-resolved record.
    const auto defaults = cfg::default_physics();
    const auto defaults_path = temp.path / "defaults.toml";
    write_text_file(defaults_path, cfg::write_physics_toml(defaults));
    const auto defaults_reread = cfg::load_physics(defaults_path);
    bt::require(canonical(cfg::to_value(defaults_reread)) ==
                    canonical(cfg::to_value(defaults)),
                "defaults round-trip");

    // A nonfinite member cannot serialize: json.dumps(allow_nan=False)
    // 'Out of range float values are not JSON compliant'.
    {
        auto broken = cfg::default_physics();
        broken.timestep_s = std::numeric_limits<double>::infinity();
        invalid_with(
            "Out of range float values are not JSON compliant",
            [&] { discard(cfg::write_physics_toml(broken)); });
    }
}

// ---- TOML parse details -------------------------------------------------------

[[gnu::noinline]] void checks_toml_details() {
    // Integer-vs-float spelling and heterogeneous values survive the parse;
    // leaf kind rules land in the typed decode.
    auto doc = cfg::detail::parse_toml(
        "a = 1\nb = 1.5\nc = \"text\"\nd = [1, \"two\", 3.0]\n[t]\nx = true\n",
        "mem.toml");
    bt::require(object_of(doc).size() == 5, "root members");
    const Value* t = member_opt(doc, "t", "doc");
    bt::require(t != nullptr, "nested table present");
    const Value* d = member_opt(doc, "d", "doc");
    bt::require(io::as_array(*d, "d").size() == 3, "hetero array");

    // Duplicate keys are a parse failure like tomllib's.
    bt::invalid([] {
        discard(cfg::detail::parse_toml("a = 1\na = 2\n", "mem.toml"));
    });

    // Dates carry no typed config meaning: they parse (tomllib accepts them)
    // then fail in the typed domain as a foreign kind.
    bt::invalid([&] {
        discard(cfg::detail::construct_config(cfg::detail::parse_toml(
            "timestep_s = 1979-05-27\nequilibrium_crank_hold_kp_nm_rad = 20000.0\nequilibrium_crank_hold_kd_nms_rad = 88.0\n", "mem.toml")));
    });
}
NATIVE_DIAG_POP

} // namespace

int main() {
    return bt::run([] {
        const auto cfg_welded = bike::configuration::load_physics(
            bt::fixture("config/viewer_physics_welded.toml"));
        bt::require(cfg_welded.timestep_s == 0.00125, "shipped timestep");
        bt::invalid([] {
            discard(bike::configuration::load_physics(
                bt::fixture("config/unknown_key.toml")));
        });
        checks_seated_climb_timings();
        checks_reposition_timings();
        checks_research_timings();
        checks_drive_timings();
        checks_hold_members();
        checks_shipped();
        checks_variants();
        checks_rejection();
        checks_precedence();
        checks_paths();
        checks_writer();
        checks_toml_details();
    });
}
