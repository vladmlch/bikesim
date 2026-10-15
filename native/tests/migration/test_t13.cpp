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
#include "support/migration_test.hpp"
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

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (manifest name, fixture path) call sites stay ordered by usage
Value fixture_json(std::string_view manifest_name, std::string_view relative) {
    bt::manifest_verify_files(bt::manifest_entry(manifest_name));
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
// leaf under tests/fixtures/migration/ to the local fixture root, failing
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
    bt::manifest_verify_files(
        bt::manifest_entry("config/viewer_physics_welded.toml"));
    const cfg::SimulationPhysicsConfig cfg_welded =
        cfg::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
    bt::require(cfg_welded.timestep_s == 0.00125, "shipped timestep");
    bt::require(cfg_welded.physics_mode == "physical", "shipped mode");
    bt::require(cfg_welded.drive_mode.has_value() &&
                    *cfg_welded.drive_mode == "articulated_effort",
                "shipped drive mode");
    expect_resolved(cfg_welded, "config/resolved/welded.json",
                    "config/resolved/welded.json");

    bt::manifest_verify_files(
        bt::manifest_entry("config/viewer_physics_distributed.toml"));
    const auto cfg_distributed = cfg::load_physics(
        bt::fixture("config/viewer_physics_distributed.toml"));
    bt::require(cfg_distributed.tires.backend == "distributed_2d_reference",
                "distributed backend");
    bt::require(cfg_distributed.tires.distributed.station_count == 128,
                "station count");
    expect_resolved(cfg_distributed, "config/resolved/distributed.json",
                    "config/resolved/distributed.json");

    bt::manifest_verify_files(
        bt::manifest_entry("config/viewer_physics_pneumatic.toml"));
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

    bt::manifest_verify_files(bt::manifest_entry("config/legacy.toml"));
    const auto cfg_legacy =
        cfg::load_physics(bt::fixture("config/legacy.toml"));
    bt::require(cfg_legacy.physics_mode == "legacy", "legacy mode");
    bt::require(cfg_legacy.drive_mode.has_value() &&
                    *cfg_legacy.drive_mode == "ideal_speed_control",
                "legacy drive mode resolved");
    bt::require(cfg_legacy.timestep_s == 0.0005, "declared default dt");
}

// ---- variant profiles ------------------------------------------------------

[[gnu::noinline]] void checks_variants() {
    for (const auto& entry : {
             std::pair<std::string_view, std::string_view>{
                 "config/variant_elastic_chain",
                 "config/variant_elastic_chain"},
             {"config/variant_alt_support", "config/variant_alt_support"},
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
    invalid_with("automatic physical shifting requires ideal_mid_drive",
                 [] {
                     discard(cfg::load_physics(bt::fixture(
                         "config/variant_elastic_chain_shifting_rejected.toml")));
                 });
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
        "invalid TireSpec: TireSpec.__init__() missing 4 required positional "
        "arguments: 'radial_c_ns_m', 'pressure_pa_gauge', 'provenance', and "
        "'valid_load_range_n'",
        [&] {
            discard(cfg::load_physics(stage(
                "missing_required.toml",
                "[tires.front.material]\nradial_k_n_m = 130000\n")));
        });

    // Nonfinite scalars arrive as TOML literals and are rejected by the
    // domain validation the typed decode feeds.
    bt::invalid([&] {
        discard(cfg::load_physics(stage("nonfinite.toml", "timestep_s = inf\n")));
    });
    bt::invalid([&] {
        discard(cfg::load_physics(stage("nonfinite2.toml", "timestep_s = nan\n")));
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
        discard(cfg::load_physics(stage("scalar_array.toml", "timestep_s = [0.001]\n")));
    });
    bt::invalid([&] {
        discard(cfg::load_physics(stage(
            "tuple_scalar.toml", "[resistance]\npoint_body_m = 0.5\n")));
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
    {
        const auto over = cfg::resolve_physics(
            make_object({{"timestep_s", make_number("0.001")}}), base);
        bt::require(over.timestep_s == 0.001, "explicit --dt wins");
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
                         {"drive_mode", make_text("coast")}}),
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
            R"({"timestep_s": 0.001, "drive": {"pedaling": {"mash_torque_nm": 61.5}}})");
        const auto over = cfg::resolve_physics(json_overrides, base);
        bt::require(over.timestep_s == 0.001, "JSON timestep override");
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
    // throws when a resolved field violates the new mode — verified against
    // the source: switching a resolved legacy base (pitch_assist=True) to
    // physical mode raises 'external pitch assist is forbidden in physical
    // mode'. An unresolved SimulationPhysicsConfig{} base instead carries
    // None for the derived fields, so post_init resolves them for the new
    // mode (drive_mode='coast', pitch_assist=False). Both behaviors locked.
    invalid_with(
        "external pitch assist is forbidden in physical mode", [] {
            discard(cfg::resolve_physics(
                make_object({{"physics_mode", make_text("physical")}}),
                cfg::default_physics()));
        });
    {
        const auto unresolved = cfg::resolve_physics(
            make_object({{"physics_mode", make_text("physical")}}),
            cfg::SimulationPhysicsConfig{});
        bt::require(unresolved.drive_mode.has_value() &&
                        *unresolved.drive_mode == "coast",
                    "unresolved base resolves drive_mode for the new mode");
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
            "timestep_s = 1979-05-27\n", "mem.toml")));
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
        checks_shipped();
        checks_variants();
        checks_rejection();
        checks_precedence();
        checks_paths();
        checks_writer();
        checks_toml_details();
    });
}
