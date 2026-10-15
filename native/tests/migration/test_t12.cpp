// test_t12.cpp — configuration/parameter schema migration regressions
// (task T12). Covers the owned declarations behind the T12 boundary:
// SimulationPhysicsConfig fields/defaults/__post_init__ (physics_mode versus
// drive_mode resolution and restrictions, integer-versus-bool, optional
// versus missing), the nested physical records (tires/drive/resistance/
// articulated/seated_climb incl. the 'weld'->'connect' grip alias), the
// schema() introspection document, to_value() resolved encodings, the
// native/schema.py resolved-wire validator, the envelope/strength JSON
// loaders, RiderSpecs/RiderPosture/PedalIntent, and the suspension spec
// records/presets/resolve_suspension mapping — mirroring the Python oracle
// in the listed source-authority files.
#include "support/migration_test.hpp"
#include "configuration/model_config.hpp"
#include "configuration/physical_config.hpp"
#include "configuration/rider_spec.hpp"
#include "configuration/suspension_spec.hpp"
#include "io/json.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <cstdlib>
#include <functional>
#include <limits>
#include <numbers>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace {

namespace cfg = bike::configuration;
namespace io = bike::io;
using bike::io::Number;
using bike::io::Value;

// ---- Value helpers ---------------------------------------------------------

const Value::Object& object_of(const Value& value, std::string_view field) {
    return io::as_object(value, field);
}
Value::Object& object_of(Value& value) {
    return std::get<Value::Object>(value.data);
}
const Value& member(const Value& object, std::string_view key,
                    std::string_view path) {
    const Value* found =
        io::find_member(object_of(object, path), key);
    bt::require(found != nullptr,
                "missing member " + std::string(path) + "." +
                    std::string(key));
    return *found;
}
const Value* member_opt(const Value& object, std::string_view key,
                        std::string_view path) {
    return io::find_member(object_of(object, path), key);
}
Value& member_mut(Value& object, std::string_view key,
                  std::string_view path) {
    for (auto& entry : object_of(object))
        if (entry.first == key) return entry.second;
    bt::require(false, "missing member " + std::string(path) + "." +
                           std::string(key));
    std::abort();
}
double f64_of(const Value& value, std::string_view field) {
    const std::string lexeme = io::as_number(value, field).lexeme;
    bool ok = false;
    const double result = io::detail::float_from_lexeme(lexeme, ok);
    bt::require(ok, "number lexeme did not decode: " + lexeme);
    return result;
}
bool bool_of(const Value& value, std::string_view field) {
    const auto* flag = std::get_if<bool>(&value.data);
    bt::require(flag != nullptr,
                std::string(field) + " must be a bool");
    return *flag;
}
const std::string& string_of(const Value& value, std::string_view field) {
    const auto* text = std::get_if<std::string>(&value.data);
    bt::require(text != nullptr,
                std::string(field) + " must be a string");
    return *text;
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

// A schema-valid tire.distributed subtree (density + both wheel materials).
// validate_mapping recurses into the optional section BEFORE the
// backend-presence matrix runs, so the "forbidden key" probe must carry a
// subtree that survives the recursive check.
Value valid_distributed_node() {
    const auto density = [] {
        return make_object(
            {{"knots_m",
              make_array({make_number("0.05"), make_number("0.2")})},
             {"stiffness_n_m",
              make_array({make_number("90000"), make_number("120000")})},
             {"damping_ns_m", make_number("40")},
             {"provenance", make_text("synthetic")}});
    };
    const auto material = [&density](std::string_view dataset) {
        return make_object(
            {{"density", density()},
             {"fitting_dataset_id", make_text(dataset)},
             {"valid_load_range_n",
              make_array({make_number("0"), make_number("500")})},
             {"valid_deflection_range_m",
              make_array({make_number("0"), make_number("0.1")})},
             {"calibration_status", make_text("synthetic")}});
    };
    return make_object({{"station_count", make_number("64")},
                        {"density", density()},
                        {"front", material("ds_front")},
                        {"rear", material("ds_rear")}});
}

// std::invalid_argument carrying exactly the expected Python message.
void expect_invalid(std::string_view expected,
                    const std::function<void()>& action) {
    try {
        action();
    } catch (const std::invalid_argument& error) {
        bt::require(std::string_view(error.what()) == expected,
                    "diagnostic mismatch: got '" +
                        std::string(error.what()) + "' expected '" +
                        std::string(expected) + "'");
        return;
    }
    bt::require(false, "expected std::invalid_argument: " +
                           std::string(expected));
}

std::string canonical(const Value& value) { return io::canonical_json(value); }

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (manifest name, fixture path) call sites stay ordered by usage
Value fixture_json(std::string_view manifest_name, std::string_view relative) {
    bt::manifest_verify_files(bt::manifest_entry(manifest_name));
    return io::parse_json(
        bt::text(bt::fixture(relative)));
}

// ---- shipped-profile construction ------------------------------------------

// viewer_physics_welded.toml supplied values, applied to the declared
// defaults and resolved by post_init() — the same end state the Python
// loader produces (drive_mode is supplied; pitch_assist stays disengaged so
// the physical-mode resolution below is the one under test).
cfg::SimulationPhysicsConfig welded_profile() {
    cfg::SimulationPhysicsConfig c;
    c.physics_mode = "physical";
    c.drive_mode = "articulated_effort";
    c.timestep_s = 0.00125;
    c.closure_time_constant_s = 0.0025;
    c.initial_speed_mps = 0.0;
    c.initial_front_brake = 1.0;
    c.initial_rear_brake = 1.0;
    c.equilibrium_refine_after_s = 1.0;
    c.equilibrium_refine_period_s = 3.0;
    c.equilibrium_cache_enabled = true;
    c.tires.backend = "compliant_2d";
    c.tires.surface_mode = "track";
    c.tires.front.mu = 1.1;
    c.tires.rear.mu = 1.1;
    c.drive.transmission_model = "ideal_mid_drive";
    c.drive.motor_clutch = false;
    c.drive.rotor_inertia_kgm2 = 0.0;
    auto& pedaling = c.drive.pedaling;
    pedaling.enabled = true;
    pedaling.coast_above_rpm = 120.0;
    pedaling.resume_below_rpm = 105.0;
    pedaling.stop_time_s = 0.35;
    pedaling.coast_cadence_tau_s = 0.35;
    pedaling.mash_cadence_rpm = 45.0;
    pedaling.mash_torque_nm = 60.0;
    pedaling.effort_slew_nm_s = 300.0;
    pedaling.rollback_brake = true;
    pedaling.rollback_engage_mps = 0.25;
    pedaling.rollback_release_mps = 0.0;
    pedaling.rollback_demand = 1.0;
    auto& assist = c.drive.assist;
    assist.profile = "bosch_cx_gen4";
    assist.mode = "turbo";
    assist.torque_curve = std::vector<std::array<double, 2>>{
        {0.0, 85.0}, {120.0, 85.0}, {120.1, 47.7}, {180.0, 0.0}};
    assist.engage_torque_nm = 4.0;
    c.drive.battery.enabled = false;
    c.drive.gearing.front_teeth = 22;
    c.drive.gearing.rear_teeth = 51;
    auto& shifting = c.drive.shifting;
    shifting.enabled = true;
    shifting.cassette = {10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51};
    shifting.target_cadence_min_rpm = 75.0;
    shifting.target_cadence_max_rpm = 110.0;
    shifting.shift_cooldown_s = 0.4;
    shifting.shift_cut_duration_s = 0.2;
    shifting.torque_factor = 0.3;
    shifting.cadence_smoothing_tau_s = 0.35;
    shifting.upshift_slip_limit_mps = 0.5;
    auto& articulated = c.articulated;
    articulated.joint_passive_damping_nms_rad = 0.0;
    articulated.balance_floor_kmh = 4.0;
    articulated.balance_dwell_s = 0.5;
    articulated.balance_grace_s = 3.0;
    articulated.road_lookahead_m = 2.0;
    articulated.pedal_attachment = "spindle";
    articulated.saddle_attachment = "pin";
    articulated.grip_attachment = "connect";
    // resolution.py resolve_config_paths: joint_envelope_path /
    // joint_strength_path / equilibrium_state_file resolve to absolute
    // paths anchored at the TOML's directory — the fixture tree's config/
    // dir here. bt::fixture returns the weakly_canonical absolute path,
    // matching Path.resolve().
    articulated.joint_envelope_path =
        bt::fixture("config/rider_joint_envelope_anatomical.json").string();
    articulated.joint_strength_path =
        bt::fixture("config/rider_strength_reference.json").string();
    articulated.active_positive_power_limit_w = 450.0;
    articulated.pedal_torque_ripple = 0.5;
    articulated.return_foot_preload_n = 40.0;
    auto& climb = c.seated_climb;
    climb.enabled = true;
    climb.target_crank_power_w = 250.0;
    climb.max_crank_torque_nm = 60.0;
    climb.surge_power_w = 400.0;
    climb.surge_grade = 0.20;
    climb.surge_budget_s = 15.0;
    climb.surge_recovery_rate = 0.3333333333333333;
    climb.front_load_share_target = 0.30;
    climb.lean_trim_gain_rad_s = 0.3;
    climb.lean_trim_limit_rad = 0.15;
    climb.trim_dead_time_s = 0.2;
    cfg::post_init(c);
    return c;
}

// Recursive first-difference walk so a resolved-tree mismatch names the
// divergent member path instead of an opaque canonical-JSON inequality.
// Returns "" when the trees are identical.
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
        // canonical_json sorts object keys, so the comparison walks both
        // member lists in key order — stored insertion order is not a
        // difference.
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
    // Bytes/Tagged leaves compare through their canonical encoding.
    if (io::canonical_json(actual) != io::canonical_json(expected))
        return path + ": leaf mismatch";
    return {};
}

Value* find_member_mut(Value::Object& object, std::string_view key) {
    for (auto& member : object)
        if (member.first == key) return &member.second;
    return nullptr;
}

// The oracle's resolved docs embed absolute paths captured in its own
// worktree: resolve_config_paths anchors joint_envelope_path /
// joint_strength_path / equilibrium_state_file at the TOML's directory. The
// behavioral contract is the profile-adjacent resolution, not the machine
// prefix — rewrite each captured leaf under tests/fixtures/migration/ to
// the local fixture root, failing when a captured leaf escapes that layout
// (a doc carrying a relative or outside-tree path is a fixture defect, not
// a tolerated difference).
void relocalize_resolved_paths(Value& doc) {
    constexpr std::string_view kAnchor = "tests/fixtures/migration/";
    const auto relocalize = [&](Value* leaf) {
        if (!leaf) return;
        auto* s = std::get_if<std::string>(&leaf->data);
        if (!s) return;  // explicit null stays null
        const std::size_t anchor = s->rfind(kAnchor);
        bt::require(anchor != std::string::npos,
                    "resolved doc path escapes the fixture tree: " + *s);
        const std::string relative =
            s->substr(anchor + kAnchor.size());
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

// Resolved-doc comparison: the to_value() canonical form must equal the
// oracle's resolved/<name>.json bitwise after canonicalization and the
// captured-path relocalization.
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

// ---- shipped TOML field coverage -------------------------------------------
//
// Minimal TOML key-path extractor scoped to the shipped profile grammar:
// [dotted.headers], bare or dotted 'lhs = value' assignments, '#' comments
// outside strings, inline arrays as values. Any line outside that grammar
// fails the test — the coverage claim only holds while the shipped files
// stay inside it.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (text, source-name) call sites stay ordered by usage
std::vector<std::string> toml_key_paths(std::string_view text,
                                        std::string_view source) {
    std::vector<std::string> paths;
    std::set<std::string> seen;
    std::string section;
    auto fail = [&](std::string_view detail) {
        bt::require(false, std::string(source) + ": " + std::string(detail));
    };
    const auto normalize_keys = [&](std::string_view dotted) {
        std::string result;
        std::size_t at = 0;
        for (;;) {
            const std::size_t dot = dotted.find('.', at);
            const std::string_view segment = dotted.substr(
                at, dot == std::string_view::npos
                        ? std::string_view::npos
                        : dot - at);
            // trim ASCII whitespace inside a segment (a . b == a.b)
            const std::size_t b = segment.find_first_not_of(" \t");
            const std::size_t e = segment.find_last_not_of(" \t");
            if (b == std::string_view::npos)
                fail("empty key segment in '" + std::string(dotted) + "'");
            std::string key{segment.substr(b, e - b + 1)};
            if (key.size() >= 2 && key.front() == '"' && key.back() == '"')
                key = key.substr(1, key.size() - 2);
            if (key.empty()) fail("empty quoted key");
            for (const char ch : key)
                if (ch == '"' || ch == '\'' || ch == '[' || ch == ']' ||
                    ch == '{' || ch == '}' || ch == ',')
                    fail("key outside the shipped grammar: '" + key + "'");
            if (!result.empty()) result += '.';
            result += key;
            if (dot == std::string_view::npos) break;
            at = dot + 1;
        }
        return result;
    };
    std::size_t at = 0;
    while (at <= text.size()) {
        const std::size_t nl = text.find('\n', at);
        std::string_view line =
            text.substr(at, nl == std::string_view::npos
                              ? std::string_view::npos
                              : nl - at);
        at = nl == std::string_view::npos ? text.size() + 1 : nl + 1;
        // Strip '#' comments outside quoted strings.
        bool quoted = false;
        std::size_t cut = std::string_view::npos;
        for (std::size_t i = 0; i < line.size(); ++i) {
            const char ch = line[i];
            if (ch == '"') quoted = !quoted;
            if (ch == '#' && !quoted) {
                cut = i;
                break;
            }
        }
        if (cut != std::string_view::npos) line = line.substr(0, cut);
        const std::size_t b = line.find_first_not_of(" \t\r");
        if (b == std::string_view::npos) continue;
        line = line.substr(b);
        line = line.substr(0, line.find_last_not_of(" \t\r") + 1);
        if (line.front() == '[') {
            if (line.size() < 3 || line.back() != ']' ||
                line[1] == '[' || line[line.size() - 2] == ']')
                fail("table header outside the shipped grammar: '" +
                     std::string(line) + "'");
            section = normalize_keys(line.substr(1, line.size() - 2));
            continue;
        }
        // 'lhs = value' — the '=' is outside quotes in the shipped files.
        quoted = false;
        std::size_t eq = std::string_view::npos;
        for (std::size_t i = 0; i < line.size(); ++i) {
            const char ch = line[i];
            if (ch == '"') quoted = !quoted;
            if (ch == '=' && !quoted) {
                eq = i;
                break;
            }
        }
        if (eq == std::string_view::npos)
            fail("line outside the shipped grammar: '" + std::string(line) +
                 "'");
        const std::string lhs =
            normalize_keys(line.substr(0, eq));
        std::string path = lhs;
        if (!section.empty()) {
            path = section;
            path += '.';
            path += lhs;
        }
        if (!seen.insert(path).second)
            fail("duplicate key path '" + path + "'");
        paths.push_back(path);
    }
    return paths;
}

// Walks the typed field tables (the same declaration schema() reads): each
// intermediate component must resolve to a nested record; a union arm is
// descended through whichever alternative declares the next name.
const cfg::FieldDecl* find_field(const cfg::RecordSchema& record,
                                 std::string_view name) {
    for (const auto& field : record.fields)
        if (field.name == name) return &field;
    return nullptr;
}

bool schema_path_exists(const cfg::RecordSchema& record,
                        std::string_view path) {
    const cfg::RecordSchema* scope = &record;
    std::size_t at = 0;
    for (;;) {
        const std::size_t dot = path.find('.', at);
        const std::string_view name = path.substr(
            at, dot == std::string_view::npos ? std::string_view::npos
                                              : dot - at);
        const cfg::FieldDecl* field = find_field(*scope, name);
        if (field == nullptr) return false;
        if (dot == std::string_view::npos) return true;
        at = dot + 1;
        // Intermediate component: must be a descendable record.
        if (field->kind == cfg::FieldKind::Dataclass &&
            field->nested != nullptr) {
            scope = field->nested;
            continue;
        }
        if (field->kind == cfg::FieldKind::OptionalDataclass &&
            field->nested != nullptr) {
            scope = field->nested;
            continue;
        }
        if (field->kind == cfg::FieldKind::UnionMaterial) {
            const std::size_t next_dot = path.find('.', at);
            const std::string_view next = path.substr(
                at, next_dot == std::string_view::npos
                        ? std::string_view::npos
                        : next_dot - at);
            if (field->nested != nullptr &&
                find_field(*field->nested, next) != nullptr) {
                scope = field->nested;
                continue;
            }
            if (field->nested2 != nullptr &&
                find_field(*field->nested2, next) != nullptr) {
                scope = field->nested2;
                continue;
            }
            return false;
        }
        return false;
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (manifest name, fixture path) call sites stay ordered by usage
void expect_covered(std::string_view manifest_name,
                    std::string_view relative) {
    bt::manifest_verify_files(bt::manifest_entry(manifest_name));
    const std::vector<std::string> paths = toml_key_paths(
        bt::text(bt::fixture(relative)), relative);
    bt::require(!paths.empty(), std::string(relative) + ": no keys parsed");
    for (const std::string& path : paths)
        bt::require(schema_path_exists(
                        cfg::detail::kSimulationPhysicsConfigSchema, path),
                    "profile field not covered by the schema: " +
                        std::string(relative) + " -> " + path);
}

// ---- section bodies ---------------------------------------------------------

void test_defaults() {
    const cfg::SimulationPhysicsConfig c = cfg::default_physics();
    bt::require(c.physics_mode == "legacy", "default physics_mode");
    // Legacy __post_init__ resolution.
    bt::require(c.drive_mode && *c.drive_mode == "ideal_speed_control",
                "legacy drive_mode resolution");
    bt::require(c.pitch_assist && *c.pitch_assist,
                "legacy pitch_assist resolution");
    bt::require(c.timestep_s == 0.0005, "default timestep_s");
    bt::require(c.control_period_s == 0.005, "default control_period_s");
    bt::require(c.closure_time_constant_s == 0.001, "default closure tau");
    bt::require(c.equilibrium_refine_after_s == 1.0, "refine_after default");
    bt::require(c.equilibrium_refine_period_s == 3.0, "refine_period default");
    bt::require(!c.equilibrium_cache_enabled, "cache default");
    bt::require(!c.equilibrium_state_file.has_value(), "state file default");
    bt::require(c.initial_speed_mps == 0. && c.initial_front_brake == 0. &&
                    c.initial_rear_brake == 0.,
                "initial speed/brake defaults");
    // Nested factory defaults.
    bt::require(c.end_stops.reference_force_n == 7000. &&
                    c.end_stops.reference_deflection_m == 0.01 &&
                    c.end_stops.damping_n_s_m == 500. &&
                    c.end_stops.overtravel_m == 0.01 &&
                    c.end_stops.provenance == "synthetic",
                "EndStopConfig defaults");
    bt::require(c.tires.backend == "native_reference" &&
                    c.tires.surface_mode == "configured" &&
                    c.tires.pneumatic.tier == "fast" &&
                    !c.tires.pneumatic.surface.has_value() &&
                    c.tires.front.mu == .8 &&
                    !c.tires.distributed.front_material.has_value(),
                "TireBackendConfig defaults");
    bt::require(c.drive.transmission_model == "elastic_chain" &&
                    !c.drive.motor_clutch &&
                    c.drive.rotor_inertia_kgm2 == 0. &&
                    c.drive.gearing.front_teeth == 34 &&
                    c.drive.gearing.rear_teeth == 24 &&
                    !c.drive.assist.profile.has_value() &&
                    c.drive.assist.mode == "turbo" &&
                    c.drive.assist.max_torque == 80. &&
                    c.drive.assist.max_power == 500. &&
                    c.drive.assist.tau == .05 &&
                    !c.drive.assist.torque_curve.has_value() &&
                    c.drive.battery.enabled &&
                    !c.drive.pedaling.enabled &&
                    !c.drive.shifting.enabled &&
                    c.drive.shifting.cassette.size() == 12,
                "PhysicalDriveConfig defaults");
    bt::require(c.resistance.wind_world_mps ==
                    std::array<double, 3>{0., 0., 0.} &&
                    c.resistance.point_body_m ==
                        std::array<double, 3>{0., 0., .5},
                "ResistanceConfig tuple defaults");
    bt::require(c.articulated.grip_attachment == "spring" &&
                    c.articulated.pedal_attachment == "flat" &&
                    c.articulated.saddle_attachment == "flat" &&
                    !c.articulated.joint_envelope_path.has_value() &&
                    !c.articulated.joint_strength_path.has_value() &&
                    !c.articulated.grip_pair_force_limit_n.has_value() &&
                    !c.articulated.coupled_task_control,
                "ArticulatedConfig defaults");
    bt::require(!c.seated_climb.enabled &&
                    c.seated_climb.surge_recovery_rate == 1. / 3.,
                "SeatedClimbConfig defaults");
    // Physical-mode resolution is the mirror image.
    cfg::SimulationPhysicsConfig physical;
    physical.physics_mode = "physical";
    cfg::post_init(physical);
    bt::require(physical.drive_mode && *physical.drive_mode == "coast",
                "physical drive_mode resolution");
    bt::require(physical.pitch_assist && !*physical.pitch_assist,
                "physical pitch_assist resolution");
}

void test_schema() {
    const Value document = cfg::schema();
    bt::require(string_of(member(document, "config_class", "schema"),
                         "schema.config_class") == "SimulationPhysicsConfig",
                "schema config_class");
    const Value::Array& fields = io::as_array(
        member(document, "fields", "schema"), "schema.fields");
    static constexpr std::array<std::string_view, 19> kTopOrder{
        "physics_mode",
        "drive_mode",
        "timestep_s",
        "control_period_s",
        "pitch_assist",
        "end_stops",
        "tires",
        "drive",
        "resistance",
        "articulated",
        "seated_climb",
        "initial_speed_mps",
        "initial_front_brake",
        "initial_rear_brake",
        "closure_time_constant_s",
        "equilibrium_refine_after_s",
        "equilibrium_refine_period_s",
        "equilibrium_cache_enabled",
        "equilibrium_state_file",
    };
    bt::require(fields.size() == 19, "top-level field count");
    for (std::size_t i = 0; i < fields.size(); ++i)
        bt::require(
            string_of(member(fields[i], "name", "fields"), "name") ==
                kTopOrder.at(i),
            "field order mismatch at index " + std::to_string(i));

    // The whole document — every default, factory qualname, type string and
    // nested field table — equals the oracle capture after canonicalization
    // (the fixture is pretty-printed; canonical_json is the semantic form).
    const std::string expected = canonical(
        fixture_json("config/schema.json", "config/schema.json"));
    bt::require(canonical(document) == expected,
                "schema() differs from the oracle document");

    // Structural spot checks behind the golden: nested counts and the
    // required/optional/default encodings.
    const auto fields_named = [&](const Value& node, std::string_view path) {
        std::vector<std::string> names;
        for (const Value& f :
             io::as_array(member(node, "fields", path),
                          std::string(path) + ".fields"))
            names.push_back(string_of(member(f, "name", path), "name"));
        return names;
    };
    const Value& tires = member(fields[6], "fields", "fields.tires");
    bt::require(io::as_array(tires, "fields.tires.fields").size() == 9,
                "tires field count");
    const Value& drive = member(fields[7], "fields", "fields.drive");
    bt::require(io::as_array(drive, "fields.drive.fields").size() == 17,
                "drive field count");
    bt::require(fields_named(fields[9], "articulated").size() == 59,
                "articulated field count");
    bt::require(fields_named(fields[10], "seated_climb").size() == 19,
                "seated_climb field count");

    // Optional leaves serialize explicit null defaults; required leaves
    // (TyreSpecs) also carry null defaults but no factory.
    const Value& drive_mode = fields[1];
    bt::require(io::is_null(member(drive_mode, "default", "drive_mode")) &&
                    io::is_null(member(drive_mode, "default_factory",
                                       "drive_mode")),
                "drive_mode must default to null without a factory");
    const Value& end_stops = fields[5];
    bt::require(io::is_null(member(end_stops, "default", "end_stops")) &&
                    string_of(member(end_stops, "default_factory",
                                     "end_stops"),
                              "end_stops.factory") == "EndStopConfig",
                "end_stops factory declaration");

    // material is the union arm field: default_factory _material, and a
    // Union annotation carries no nested 'fields' member at all.
    const Value& front_tp = io::as_array(tires, "tires.fields")[1];  // front
    const Value& material = io::as_array(
        member(front_tp, "fields", "tires.front"), "tires.front.fields")[0];
    bt::require(
        string_of(member(material, "name", "material"), "material.name") ==
                "material" &&
            string_of(member(material, "default_factory", "material"),
                      "material.factory") == "_material" &&
            member_opt(material, "fields", "material") == nullptr &&
            io::is_null(member(material, "default", "material")),
        "union material must declare _material without nested fields");

    // Dataclass-instance defaults (pneumatic.front/rear) encode the
    // {"$dataclass": path, "fields": {...}} marker.
    const Value& pneu_entry = io::as_array(tires, "tires.fields")[8];
    const Value& pneu_front = io::as_array(
        member(pneu_entry, "fields", "tires.pneumatic"),
        "tires.pneumatic.fields")[1];
    const Value& dclass =
        member(pneu_front, "default", "tires.pneumatic.front");
    bt::require(
        string_of(member(dclass, "$dataclass", "default"),
                  "default.$dataclass") == "bike_sim.physics.tyre.TyreSpecs",
        "pneumatic.front dataclass-instance default marker");
}

// Each domain case mutates a fresh config inside its own call frame: the
// record is multi-kilobyte and -Wframe-larger-than bounds every function at
// 8 KiB, so the case helpers own the SimulationPhysicsConfig temporaries.
void case_from_default(
    std::string_view expected,
    const std::function<void(cfg::SimulationPhysicsConfig&)>& mutate) {
    cfg::SimulationPhysicsConfig config = cfg::default_physics();
    mutate(config);
    expect_invalid(expected, [&] { cfg::validate(config); });
}
void case_from_welded(
    std::string_view expected,
    const std::function<void(cfg::SimulationPhysicsConfig&)>& mutate) {
    cfg::SimulationPhysicsConfig config = welded_profile();
    mutate(config);
    expect_invalid(expected, [&] { cfg::validate(config); });
}
// A post_init'd variant of the welded profile whose to_value() must equal
// the named resolved oracle document.
void resolved_case(
    std::string_view manifest_name, std::string_view relative,
    const std::function<void(cfg::SimulationPhysicsConfig&)>& mutate) {
    cfg::SimulationPhysicsConfig config = welded_profile();
    mutate(config);
    cfg::post_init(config);
    expect_resolved(config, manifest_name, relative);
}

void test_validation() {
    // The brief's mandatory red-line case plus exact diagnostics.
    case_from_default("timestep_s must be finite and positive",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.timestep_s = 0.0;
                      });
    case_from_default("timestep_s must be finite and positive",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.timestep_s =
                              std::numeric_limits<double>::quiet_NaN();
                      });
    case_from_default("unknown physics_mode",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.physics_mode = "bogus";
                      });
    case_from_default(
        "control_period_s must be an integer multiple of timestep_s",
        [](cfg::SimulationPhysicsConfig& c) {
            c.control_period_s = 0.0077;  // not a multiple of 0.0005
        });
    case_from_default("invalid equilibrium_refine_after_s",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.equilibrium_refine_after_s = 0.0;
                      });

    // Physics-mode versus drive-mode restrictions.
    case_from_default(
        "physical effort drive modes require physics_mode=physical",
        [](cfg::SimulationPhysicsConfig& c) { c.drive_mode = "crank_effort"; });
    case_from_default("unknown drive_mode", [](cfg::SimulationPhysicsConfig& c) {
        c.drive_mode = "bogus";
    });
    case_from_default("external pitch assist is forbidden in physical mode",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.physics_mode = "physical";
                          c.pitch_assist = true;
                      });

    // Physical-mode closure domain: 2*timestep_s floor.
    case_from_default(
        "closure_time_constant_s=0.0006 must be at least "
        "2*timestep_s=0.001; otherwise MuJoCo silently changes closure "
        "stiffness",
        [](cfg::SimulationPhysicsConfig& c) {
            c.physics_mode = "physical";
            c.closure_time_constant_s = 0.0006;
        });

    // Brakes and initial speed.
    case_from_default("initial_front_brake must lie in [0,1]",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.initial_front_brake = 1.5;
                      });
    case_from_default("invalid initial_rear_brake",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.initial_rear_brake = -0.1;
                      });
    case_from_default("invalid initial speed",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.initial_speed_mps =
                              std::numeric_limits<double>::infinity();
                      });
    case_from_default(
        "equilibrium_state_file must be a non-empty path or None",
        [](cfg::SimulationPhysicsConfig& c) {
            c.equilibrium_state_file = "   ";
        });

    // Drive/transmission rules inside PhysicalDriveConfig.__post_init__.
    case_from_welded("automatic physical shifting requires ideal_mid_drive",
                     [](cfg::SimulationPhysicsConfig& c) {
                         c.drive.transmission_model = "elastic_chain";
                     });
    case_from_welded(
        "a motor rotor requires an ideal mid-drive transmission",
        [](cfg::SimulationPhysicsConfig& c) {
            c.drive.transmission_model = "elastic_chain";
            c.drive.shifting.enabled = false;
            c.drive.rotor_inertia_kgm2 = 0.2;
        });
    case_from_welded(
        "rotor freewheel and the legacy crank clutch are exclusive",
        [](cfg::SimulationPhysicsConfig& c) {
            c.drive.rotor_inertia_kgm2 = 0.2;
            c.drive.motor_clutch = true;
        });
    case_from_welded("initial rear sprocket must belong to the cassette",
                     [](cfg::SimulationPhysicsConfig& c) {
                         c.drive.gearing.rear_teeth = 99;
                     });
    case_from_welded(
        "seated climb requires the articulated physical effort drive",
        [](cfg::SimulationPhysicsConfig& c) { c.drive_mode = "coast"; });
    // Nested PhysicalDriveConfig rules run first: the clutch needs an ideal
    // mid-drive transmission before the top-level effort-mode check is
    // reached.
    case_from_default("drive.motor_clutch requires an effort drive mode",
                      [](cfg::SimulationPhysicsConfig& c) {
                          c.drive.transmission_model = "ideal_mid_drive";
                          c.drive.motor_clutch = true;
                      });
    case_from_welded("drive.rotor_inertia_kgm2 requires an effort drive mode",
                     [](cfg::SimulationPhysicsConfig& c) {
                         c.drive_mode = "coast";
                         c.seated_climb.enabled = false;
                         c.drive.rotor_inertia_kgm2 = 0.2;
                     });

    // ArticulatedConfig domain + field-name diagnostics.
    case_from_welded(
        "arm reach fraction must lie strictly between zero and one",
        [](cfg::SimulationPhysicsConfig& c) {
            c.articulated.arm_reach_fraction = 1.0;
        });
    case_from_welded(
        "postural support fractions must leave a saddle share",
        [](cfg::SimulationPhysicsConfig& c) {
            c.articulated.pedal_support_fraction = 0.95;
            c.articulated.bar_support_fraction = 0.1;
        });
    case_from_welded("joint_envelope_path must be a nonempty path or None",
                     [](cfg::SimulationPhysicsConfig& c) {
                         c.articulated.joint_envelope_path = "  ";
                     });
    case_from_welded("joint_strength_path must be a nonempty path or None",
                     [](cfg::SimulationPhysicsConfig& c) {
                         c.articulated.joint_strength_path = "";
                     });
    case_from_welded("pedal torque ripple must be below one",
                     [](cfg::SimulationPhysicsConfig& c) {
                         c.articulated.pedal_torque_ripple = 1.0;
                     });
    case_from_welded("pedal scrape fraction must not exceed one",
                     [](cfg::SimulationPhysicsConfig& c) {
                         c.articulated.pedal_scrape_fraction = 1.5;
                     });
    case_from_welded(
        "pedal_attachment must be 'flat', 'weld' or 'spindle', got 'bolt'",
        [](cfg::SimulationPhysicsConfig& c) {
            c.articulated.pedal_attachment = "bolt";
        });
    case_from_welded(
        "saddle_attachment must be 'flat', 'weld' or 'pin', got 'spindle'",
        [](cfg::SimulationPhysicsConfig& c) {
            c.articulated.saddle_attachment = "spindle";
        });
    case_from_welded(
        "grip_attachment must be 'spring' or 'connect', got 'pin'",
        [](cfg::SimulationPhysicsConfig& c) {
            c.articulated.grip_attachment = "pin";
        });

    // Optional-valued limits stay disengaged under validate().
    {
        cfg::SimulationPhysicsConfig config = welded_profile();
        config.articulated.grip_pair_force_limit_n = std::nullopt;
        config.articulated.active_positive_power_limit_w = std::nullopt;
        config.articulated.joint_passive_damping_nms_rad = std::nullopt;
        cfg::validate(config);
    }

    // tire_curve.py wraps both _scalar calls: a non-finite bound re-raises
    // as 'valid load range requires two finite bounds', not
    // 'invalid valid load'.
    {
        cfg::TabulatedTireSpec tab{
            .deflection_m = {0., .01, .02},
            .force_n = {0., 800., 1600.},
            .radial_c_ns_m = 40.,
            .pressure_pa_gauge = 70000.,
            .provenance = "test fixture",
            .valid_load_range_n = {0., 1600.}};
        cfg::detail::post_init(tab);  // control: valid construction passes
        tab.valid_load_range_n = {std::numeric_limits<double>::infinity(),
                                  1600.};
        expect_invalid("valid load range requires two finite bounds",
                       [&] { cfg::detail::post_init(tab); });
    }
}

void check_grip_weld_alias() {
    // 'weld' normalizes to 'connect' through __post_init__.
    cfg::SimulationPhysicsConfig config = welded_profile();
    config.articulated.grip_attachment = "weld";
    cfg::post_init(config);
    bt::require(config.articulated.grip_attachment == "connect",
                "grip 'weld' alias must normalize to 'connect'");
    // 'weld' is accepted verbatim for pedal/saddle (member of their sets).
    config.articulated.pedal_attachment = "weld";
    config.articulated.saddle_attachment = "weld";
    cfg::post_init(config);
    bt::require(config.articulated.pedal_attachment == "weld" &&
                    config.articulated.saddle_attachment == "weld",
                "pedal/saddle 'weld' stay verbatim");
}
void check_validate_nonmutating() {
    // Missing drive_mode/pitch_assist resolve per mode; validate() is
    // non-mutating — the copy resolves, the caller's optionals stay empty.
    cfg::SimulationPhysicsConfig config;
    config.physics_mode = "physical";
    config.drive_mode = std::nullopt;
    config.pitch_assist = std::nullopt;
    cfg::validate(config);
    bt::require(!config.drive_mode.has_value() &&
                    !config.pitch_assist.has_value(),
                "validate() must not mutate the supplied config");
    cfg::post_init(config);
    // optional == scalar compares present-value only — no unchecked access.
    bt::require(config.drive_mode == "coast" && config.pitch_assist == false,
                "physical-mode resolution");
}
void check_cassette_sorted() {
    // Cassette normalization: ShiftingConfig.__post_init__ sorts the supplied
    // sprocket sizes.
    cfg::SimulationPhysicsConfig config = welded_profile();
    config.drive.shifting.cassette = {51, 10, 33, 24};
    config.drive.gearing.rear_teeth = 51;
    cfg::post_init(config);
    bt::require(config.drive.shifting.cassette ==
                    std::vector<int>({10, 24, 33, 51}),
                "cassette must normalize to sorted order");
}

void test_aliases_and_resolution() {
    check_grip_weld_alias();
    check_validate_nonmutating();
    check_cassette_sorted();

    // Alias/attachment/transmission variant profiles equal their resolved
    // oracle documents — the round-trip keeps optional/missing distinctions
    // and the alias normalization.
    resolved_case("config/variant_grip_weld_coercion",
                  "config/variant_grip_weld_coercion.resolved.json",
                  [](cfg::SimulationPhysicsConfig& c) {
                      c.articulated.grip_attachment = "weld";
                  });
    resolved_case("config/variant_alt_support",
                  "config/variant_alt_support.resolved.json",
                  [](cfg::SimulationPhysicsConfig& c) {
                      c.articulated.pedal_attachment = "flat";
                      c.articulated.saddle_attachment = "flat";
                      c.articulated.grip_attachment = "spring";
                  });
    resolved_case("config/variant_elastic_chain",
                  "config/variant_elastic_chain.resolved.json",
                  [](cfg::SimulationPhysicsConfig& c) {
                      c.drive.transmission_model = "elastic_chain";
                      c.drive.shifting.enabled = false;
                  });
}

void test_resolved_profiles() {
    // The oracle documents carry the profile override values — guard the
    // fixtures themselves so a stale capture cannot pass vacuously.
    const Value welded_doc =
        fixture_json("config/resolved/welded.json",
                     "config/resolved/welded.json");
    bt::require(f64_of(member(welded_doc, "timestep_s", "resolved"),
                       "timestep_s") == 0.00125 &&
                    string_of(member(welded_doc, "drive_mode", "resolved"),
                              "drive_mode") == "articulated_effort" &&
                    !bool_of(member(welded_doc, "pitch_assist", "resolved"),
                             "pitch_assist"),
                "welded oracle carries the profile timestep/effort mode");

    expect_resolved(welded_profile(), "config/resolved/welded.json",
                    "config/resolved/welded.json");
    resolved_case("config/resolved/distributed.json",
                  "config/resolved/distributed.json",
                  [](cfg::SimulationPhysicsConfig& c) {
                      c.tires.backend = "distributed_2d_reference";
                      c.tires.distributed.station_count = 128;
                  });
    resolved_case("config/resolved/pneumatic.json",
                  "config/resolved/pneumatic.json",
                  [](cfg::SimulationPhysicsConfig& c) {
                      c.tires.backend = "pneumatic";
                      c.tires.pneumatic.tier = "fast";
                      // The pneumatic TOML drops the [tires.front]/[rear]
                      // mu overrides — the parameters stay at their
                      // declared defaults.
                      c.tires.front.mu = 0.8;
                      c.tires.rear.mu = 0.8;
                      c.equilibrium_state_file =
                          bt::fixture("config/poses/savage_pneumatic.npz")
                              .string();
                  });

    // Optional-vs-missing on the encoded tree: a disengaged optional
    // serializes as an explicit null member, never as an absent member.
    const Value encoded = cfg::to_value(cfg::default_physics());
    const Value& drive_mode =
        member(encoded, "drive_mode", "config");
    bt::require(!io::is_null(drive_mode) &&
                    string_of(drive_mode, "drive_mode") ==
                        "ideal_speed_control",
                "resolved drive_mode encoding");
    const Value& assist =
        member(member(encoded, "drive", "config"), "assist", "drive");
    bt::require(io::is_null(member(assist, "profile", "assist")) &&
                    io::is_null(member(assist, "torque_curve", "assist")),
                "disengaged assist optionals must encode as explicit null");
    const Value& distributed_node =
        member(member(encoded, "tires", "config"), "distributed", "tires");
    bt::require(io::is_null(member(distributed_node, "front_material",
                                   "distributed")) &&
                    io::is_null(member(distributed_node, "rear_material",
                                       "distributed")),
                "disengaged wheel materials must encode as explicit null");
    // Tuple members carry the $tuple tag; nested records are plain objects.
    const Value& cassette =
        member(member(member(encoded, "drive", "config"), "shifting",
                      "drive.shifting"),
               "cassette", "drive.shifting");
    const Value& tuple = member(cassette, "$tuple", "cassette");
    bt::require(io::as_array(tuple, "cassette.$tuple").size() == 12,
                "cassette tuple tag");
    const Value& wind = member(
        member(encoded, "resistance", "config"), "wind_world_mps",
        "resistance");
    bt::require(io::as_array(member(wind, "$tuple", "wind"),
                             "wind.$tuple")
                        .size() == 3,
                "wind_world_mps tuple tag");
}

void test_profile_coverage() {
    // Every shipped TOML profile field resolves to exactly one declared
    // schema field (the typed FieldDecl graph — not a second list).
    for (const auto& [name, path] :
         {std::pair{"config/legacy.toml", "config/legacy.toml"},
          {"config/viewer_physics_welded.toml",
           "config/viewer_physics_welded.toml"},
          {"config/viewer_physics_distributed.toml",
           "config/viewer_physics_distributed.toml"},
          {"config/viewer_physics_pneumatic.toml",
           "config/viewer_physics_pneumatic.toml"},
          {"config/variant_alt_support", "config/variant_alt_support.toml"},
          {"config/variant_grip_weld_coercion",
           "config/variant_grip_weld_coercion.toml"},
          {"config/variant_elastic_chain",
           "config/variant_elastic_chain.toml"},
          {"config/variant_elastic_chain_shifting_rejected",
           "config/variant_elastic_chain_shifting_rejected.toml"}})
        expect_covered(name, path);

    // The rejection fixture ships a deliberately uncovered key.
    bt::manifest_verify_files(bt::manifest_entry("config/unknown_key.toml"));
    const std::vector<std::string> paths = toml_key_paths(
        bt::text(bt::fixture("config/unknown_key.toml")),
        "config/unknown_key.toml");
    bool found_uncovered = false;
    for (const std::string& path : paths)
        if (!schema_path_exists(
                cfg::detail::kSimulationPhysicsConfigSchema, path))
            found_uncovered = true;
    bt::require(found_uncovered,
                "unknown_key.toml must contain an uncovered field");
}

void test_wire_validation() {
    // The engine-side resolved wire document lives under writer_config.
    bt::manifest_verify_files(bt::manifest_entry("bootstrap/welded_pinned"));
    const Value document = io::parse_json(bt::text(
        bt::fixture("bootstrap/welded_pinned/config.json")));
    const Value& wire =
        member(document, "writer_config", "config");

    cfg::wire::validate_config(wire);
    // plain() coerces INTEGER members to int spellings post-validation.
    const Value plain_doc = cfg::wire::plain(wire);
    const Value& plain_gearing = member(
        member(plain_doc, "drive", "wire"), "gearing", "drive");
    bt::require(
        io::as_number(member(plain_gearing, "front_teeth", "gearing"),
                      "front_teeth")
                .lexeme == "22",
        "plain() must emit the integer lexeme for front_teeth");

    // Integer-versus-bool: a JSON boolean is never an integer.
    Value mutated = wire;
    member_mut(member_mut(member_mut(mutated, "drive", "wire"), "gearing",
                          "drive"),
               "front_teeth", "gearing") = make_bool(true);
    expect_invalid("config.drive.gearing.front_teeth: expected integer",
                   [&] { cfg::wire::validate_config(mutated); });
    mutated = wire;
    member_mut(member_mut(member_mut(mutated, "drive", "wire"), "gearing",
                          "drive"),
               "front_teeth", "gearing") = make_number("22.5");
    expect_invalid("config.drive.gearing.front_teeth: expected integer",
                   [&] { cfg::wire::validate_config(mutated); });

    // Scalar members are equally strict: checks.py scalar() rejects bools
    // (isinstance(value, (bool, np.bool_))) and strings (not Real) BEFORE
    // any float() coercion — JSON true / "0.0127" are not scalars.
    mutated = wire;
    member_mut(member_mut(member_mut(mutated, "drive", "wire"), "gearing",
                          "drive"),
               "chain_pitch_m", "gearing") = make_bool(true);
    expect_invalid(
        "config.drive.gearing.chain_pitch_m must be a finite real scalar",
        [&] { cfg::wire::validate_config(mutated); });
    mutated = wire;
    member_mut(member_mut(member_mut(mutated, "drive", "wire"), "gearing",
                          "drive"),
               "chain_pitch_m", "gearing") = make_text("0.0127");
    expect_invalid(
        "config.drive.gearing.chain_pitch_m must be a finite real scalar",
        [&] { cfg::wire::validate_config(mutated); });

    // Missing-versus-null on required members.
    mutated = wire;
    {
        auto& drive_obj = object_of(member_mut(mutated, "drive", "wire"));
        for (auto it = drive_obj.begin(); it != drive_obj.end(); ++it)
            if (it->first == "drive_mode") {
                drive_obj.erase(it);
                break;
            }
    }
    expect_invalid("config.drive.drive_mode: missing required key",
                   [&] { cfg::wire::validate_config(mutated); });

    // Unknown keys are named at their full path.
    mutated = wire;
    object_of(member_mut(mutated, "drive", "wire"))
        .emplace_back("bogus_member", make_number("1"));
    expect_invalid("config.drive.bogus_member: unknown key",
                   [&] { cfg::wire::validate_config(mutated); });
    mutated = wire;
    object_of(mutated).emplace_back("bogus", make_number("1"));
    expect_invalid("config.bogus: unknown key",
                   [&] { cfg::wire::validate_config(mutated); });

    // Backend presence matrix: compliant_2d forbids tire.pneumatic and
    // requires tire.<side>.material; distributed does the opposite.
    mutated = wire;
    member_mut(member_mut(mutated, "tire", "wire"), "backend", "tire") =
        make_text("pneumatic");
    expect_invalid("config.tire.pneumatic: missing required key",
                   [&] { cfg::wire::validate_config(mutated); });
    // validate_mapping recurses into the optional section BEFORE the
    // presence matrix: an empty pneumatic subtree fails its own schema
    // first.
    mutated = wire;
    {
        auto& tire_obj = object_of(member_mut(mutated, "tire", "wire"));
        Value extra;
        extra.data = Value::Object{};
        tire_obj.emplace_back("pneumatic", std::move(extra));
    }
    expect_invalid("config.tire.pneumatic.tier: missing required key",
                   [&] { cfg::wire::validate_config(mutated); });
    // With a schema-valid subtree the backend-presence rule itself fires.
    mutated = wire;
    object_of(member_mut(mutated, "tire", "wire"))
        .emplace_back("distributed", valid_distributed_node());
    expect_invalid("config.tire.distributed: unknown key",
                   [&] { cfg::wire::validate_config(mutated); });
    mutated = wire;
    member_mut(member_mut(mutated, "tire", "wire"), "backend", "tire") =
        make_text("distributed_2d_reference");
    expect_invalid("config.tire.distributed: missing required key",
                   [&] { cfg::wire::validate_config(mutated); });

    // Profile-mode policy gate on the wire.
    mutated = wire;
    member_mut(member_mut(member_mut(mutated, "drive", "wire"), "assist",
                          "drive"),
               "mode", "assist") = make_text("sport");
    expect_invalid("config.drive.assist.mode: unsupported profiled mode",
                   [&] { cfg::wire::validate_config(mutated); });
    mutated = wire;
    member_mut(member_mut(member_mut(mutated, "drive", "wire"), "shifting",
                          "drive"),
               "upshift_slip_mode", "shifting") = make_text("bogus");
    expect_invalid(
        "config.drive.shifting.upshift_slip_mode: unsupported mode",
        [&] { cfg::wire::validate_config(mutated); });
    mutated = wire;
    member_mut(member_mut(mutated, "drive", "wire"), "transmission_model",
               "drive") = make_text("gearbox");
    expect_invalid("config.drive.transmission_model: unsupported model",
                   [&] { cfg::wire::validate_config(mutated); });
    mutated = wire;
    member_mut(member_mut(mutated, "drive", "wire"), "drive_mode",
               "drive") = make_text("warp_drive");
    expect_invalid("config.drive.drive_mode: unsupported mode",
                   [&] { cfg::wire::validate_config(mutated); });

    // Rider-contacts domain gate (ArticulatedConfig mirror on the wire).
    Value contacts =
        *io::find_member(object_of(wire, "wire"), "rider_contacts");
    member_mut(contacts, "grip_attachment", "rider_contacts") =
        make_text("weld");
    expect_invalid(
        "config.rider_contacts.grip_attachment: unsupported attachment",
        [&] { cfg::wire::validate_rider_contacts(contacts); });
    member_mut(contacts, "grip_attachment", "rider_contacts") =
        make_text("connect");
    member_mut(contacts, "arm_reach_m", "rider_contacts") =
        make_number("-0.1");
    expect_invalid("invalid config.rider_contacts.arm_reach_m",
                   [&] { cfg::wire::validate_rider_contacts(contacts); });
}

void test_rider() {
    const cfg::RiderSpecs specs;
    bt::require(specs.variant == "seated" && specs.legs == "rigid" &&
                    specs.mass_kg == 80. && specs.height_m == 1.80 &&
                    !specs.inseam_m.has_value() &&
                    specs.helmet_mass_kg == 0.40 &&
                    specs.saddle_share == 0.55 && specs.pedal_share == 0.33 &&
                    specs.bar_share == 0.12,
                "RiderSpecs defaults");
    bt::require(specs.present() && specs.total_rider_mass() == 80.,
                "RiderSpecs presence/mass");
    // Derived helpers: inseam estimate, LeMond saddle height, segment scale.
    const double inseam = specs.inseam();
    bt::require(inseam == 0.47 * 1.80, "inseam estimate");
    bt::require(specs.saddle_height_m() == 0.883 * inseam,
                "saddle height");
    bt::require(specs.segment_length("thigh") ==
                    cfg::de_leva_segment_length_fraction("thigh") * 1.80,
                "segment length scale");

    cfg::RiderSpecs bad = specs;
    bad.variant = "bogus";
    expect_invalid(
        "rider variant must be one of ('none', 'lumped', 'seated', "
        "'articulated_planar'), got 'bogus'",
        [&] { cfg::detail::post_init(bad); });
    bad = specs;
    bad.legs = "bogus";
    expect_invalid("legs must be 'rigid' or 'articulated', got 'bogus'",
                   [&] { cfg::detail::post_init(bad); });
    bad = specs;
    bad.mass_kg = 0.;
    expect_invalid("rider mass must be positive, got 0.0",
                   [&] { cfg::detail::post_init(bad); });
    bad = specs;
    bad.height_m = -1.;
    expect_invalid("rider height must be positive, got -1.0",
                   [&] { cfg::detail::post_init(bad); });
    bad = specs;
    bad.inseam_m = 1.90;  // above height
    expect_invalid("inseam 1.9 m must lie between 0 and the height 1.8 m",
                   [&] { cfg::detail::post_init(bad); });
    bad = specs;
    bad.saddle_share = 0.9;  // shares no longer sum to one
    expect_invalid("saddle, pedal and bar shares must sum to 1, got 1.350000",
                   [&] { cfg::detail::post_init(bad); });
    bad = specs;
    bad.helmet_mass_kg = 81.0;
    expect_invalid(
        "helmet mass 81.0 kg must be non-negative and below the rider mass",
        [&] { cfg::detail::post_init(bad); });
    // A 'none' rider is absent: no positive-mass requirement, zero total.
    cfg::RiderSpecs absent = specs;
    absent.variant = "none";
    cfg::detail::post_init(absent);
    bt::require(!absent.present() && absent.total_rider_mass() == 0.,
                "absent rider mass");

    // RiderPosture envelope + the standing() factory.
    cfg::RiderPosture posture;
    cfg::detail::post_init(posture);
    posture.torso_lean_rad = 0.9;
    expect_invalid("torso_lean_rad must lie in [-0.8, 0.8]",
                   [&] { cfg::detail::post_init(posture); });
    posture = cfg::RiderPosture{};
    posture.pelvis_offset_m = std::array<double, 2>{0.3, 0.};
    expect_invalid("hip offset exceeds the posture command envelope",
                   [&] { cfg::detail::post_init(posture); });
    const cfg::RiderPosture standing = cfg::standing_posture();
    bt::require(standing.torso_lean_rad == 0.08 &&
                    standing.pelvis_offset_m &&
                    (*standing.pelvis_offset_m)[0] == 0. &&
                    (*standing.pelvis_offset_m)[1] == 0.16 &&
                    !standing.use_saddle,
                "standing() factory values");

    // rider_program.py pedal_intent bounds.
    const cfg::PedalIntent intent =
        cfg::pedal_intent(1.5707963267948966, 10., 0.2, 0.5);
    bt::require(intent.cadence_rad_s == 10. &&
                    intent.scrape_fraction == 0.5 &&
                    std::abs(intent.ankle_offset_rad -
                             0.2 * std::sin(1.5707963267948966)) < 1e-15,
                "pedal_intent construction");
    bt::invalid([] {
        (void) cfg::pedal_intent(
            std::numeric_limits<double>::infinity(), 10., 0.2, 0.5);
    });
    bt::invalid([] { (void) cfg::pedal_intent(0., -1., 0.2, 0.5); });
    bt::invalid([] { (void) cfg::pedal_intent(0., 10., 0.2, 1.5); });
}

void test_loaders() {
    const cfg::JointEnvelopeMap envelopes = cfg::load_joint_envelopes(
        bt::verified_fixture("config/rider_joint_envelope_anatomical.json"));
    bt::require(envelopes.size() == 11, "envelope joint count");
    const std::span<const std::string_view> joint_names{cfg::kRiderJoints};
    auto expected_name = joint_names.begin();
    for (const auto& [name, e] : envelopes) {
        bt::require(expected_name != joint_names.end() &&
                        name == *expected_name,
                    "envelope joint order at " + name);
        ++expected_name;
        bt::require(e.minimum_anatomical_rad < e.maximum_anatomical_rad &&
                        (e.direction == 1 || e.direction == -1) &&
                        !e.provenance.empty(),
                    "envelope domain at " + name);
    }
    // present={} (an engaged empty set) requests no joints; present=None
    // requests the whole topology — the two must stay distinct.
    const cfg::JointEnvelopeMap none = cfg::load_joint_envelopes(
        bt::verified_fixture("config/rider_joint_envelope_anatomical.json"),
        std::span<const std::string_view>{});
    bt::require(none.empty(), "engaged empty present set");

    // Strength profile: 11 joints x {positive:+1, negative:-1} curves; the
    // shipped sheet is provisional (verified=false) so the verified gate
    // must reject it.
    const cfg::StrengthProfile strength = cfg::load_strength_profile(
        bt::verified_fixture("config/rider_strength_reference.json"),
        cfg::kRiderJoints, /*require_verified=*/false);
    bt::require(strength.size() == 11, "strength joint count");
    for (const auto& [name, directions] : strength) {
        bt::require(directions.size() == 2 &&
                        directions.count(1) == 1 &&
                        directions.count(-1) == 1,
                    "directions for " + name);
        const cfg::TorqueCurve& curve = directions.at(1);
        bt::require(curve.angles_rad.size() == curve.torques_nm.size() &&
                        !curve.angles_rad.empty() &&
                        curve.vmax_rad_s > 0. && !curve.source.empty(),
                    "curve domain for " + name);
    }
    bt::invalid([&] {
        (void) cfg::load_strength_profile(
            bt::verified_fixture("config/rider_strength_reference.json"),
            cfg::kRiderJoints, /*require_verified=*/true);
    });
    const cfg::JointEnvelopeMap coordinates = cfg::load_strength_coordinates(
        bt::verified_fixture("config/rider_strength_reference.json"),
        cfg::kRiderJoints);
    bt::require(coordinates.size() == 11, "strength coordinate count");
}

void test_suspension() {
    // Declared defaults (air_spring.py / coil_shock.py / damper.py).
    const cfg::AirSpringSpecs air;
    bt::require(air.stanchion_inner_diam_mm == 24.0 &&
                    air.total_travel_mm == 180.0 &&
                    air.pos_chamber_length_mm == 280.0 &&
                    air.neg_chamber_length_mm == 55.0 &&
                    air.token_volume_cm3 == 8.0 && air.max_tokens == 4 &&
                    air.default_tokens == 2 && air.gamma == 1.40 &&
                    air.atm_pressure_pa == 101325.0,
                "AirSpringSpecs defaults");
    const double radius = 0.024 / 2.0;
    bt::require(air.piston_area_m2() ==
                    std::numbers::pi_v<double> * radius * radius,
                "piston area");
    bt::require(air.token_volume_m3() == 8.0e-6, "token volume");
    bt::require(air.base_pos_volume_m3() ==
                    air.piston_area_m2() * 0.280 &&
                    air.base_neg_volume_m3() ==
                        air.piston_area_m2() * 0.055,
                "chamber volumes");
    const cfg::CoilShockSpecs coil;
    bt::require(coil.rate_n_m == 114600.0 && coil.preload_mm == 0.0 &&
                    coil.stroke_mm == 65.0 && coil.bumper_length_mm == 10.0 &&
                    coil.bumper_peak_n == 7000.0 &&
                    coil.bumper_engage_mm() == 55.0,
                "CoilShockSpecs defaults");

    // Preset labels and click tables (damper.py DAMPER_PRESETS; CUSTOM is
    // never registered).
    bt::require(cfg::damper_preset_label(cfg::DamperPreset::kBase) ==
                    "Base Tune (Factory Neutral)" &&
                    cfg::damper_preset_label(cfg::DamperPreset::kPlush) ==
                        "Plush / Wet Traction" &&
                    cfg::damper_preset_label(cfg::DamperPreset::kEnduro) ==
                        "Enduro Race (Supportive)" &&
                    cfg::damper_preset_label(cfg::DamperPreset::kPark) ==
                        "Bike Park / Jumps (Firm)" &&
                    cfg::damper_preset_label(cfg::DamperPreset::kCustom) ==
                        "Custom Rider Tune",
                "preset labels");
    bt::require(!cfg::damper_preset_registered(cfg::DamperPreset::kCustom) &&
                    cfg::damper_preset_registered(cfg::DamperPreset::kBase),
                "preset registration");
    const cfg::DamperClickConfig& park =
        cfg::damper_preset_row(cfg::DamperPreset::kPark);
    bt::require(park.fork_hsc == 4 && park.fork_lsc == 13 &&
                    park.fork_rebound == 14 && park.shock_hsc == 4 &&
                    park.shock_lsc == 13 && park.shock_rebound == 12 &&
                    park.shock_hbo == 4 && !park.shock_lockout &&
                    park.preset_name == "Bike Park / Jumps (Firm)",
                "park preset clicks");
    const cfg::DamperClickConfig& plush =
        cfg::damper_preset_row(cfg::DamperPreset::kPlush);
    bt::require(plush.fork_hsc == 0 && plush.fork_lsc == 3 &&
                    plush.fork_rebound == 5 && plush.shock_hsc == 0 &&
                    plush.shock_lsc == 3 && plush.shock_rebound == 4 &&
                    plush.shock_hbo == 1,
                "plush preset clicks");

    // resolve_suspension: the BikeSpecs->ctor mapping incl. click clamps.
    const cfg::SuspensionSpec resolved =
        cfg::resolve_suspension(cfg::SuspensionInputs{});
    bt::require(resolved.clicks.preset_name ==
                    "Base Tune (Factory Neutral)" &&
                    resolved.air.total_travel_mm == 180.0 &&
                    resolved.num_tokens == 2 &&
                    resolved.gauge_pressure_psi == 85.2 &&
                    resolved.coil.rate_n_m == 114600.0 &&
                    resolved.coil.stroke_mm == 65.0 &&
                    resolved.fork_damper.total_travel_mm == 180.0 &&
                    resolved.shock_damper.total_stroke_mm == 65.0,
                "resolved suspension mapping");
    cfg::SuspensionInputs clamped;
    clamped.fork_hsc = 99;
    clamped.shock_rebound = -5;
    const cfg::SuspensionSpec clamped_spec = cfg::resolve_suspension(clamped);
    // The DamperClickConfig record carries the requested clicks verbatim;
    // BaseDamper.__init__ clamps (0..max) on the resolved damper records.
    bt::require(clamped_spec.clicks.fork_hsc == 99 &&
                    clamped_spec.clicks.shock_rebound == -5 &&
                    clamped_spec.fork_damper.core.hsc_clicks == 4 &&
                    clamped_spec.shock_damper.core.rebound_clicks == 0,
                "click clamping");
    bt::invalid([] {
        cfg::SuspensionInputs bad;
        bad.shock_stiffness = -1.0;
        (void) cfg::resolve_suspension(bad);
    });
    bt::invalid([] {
        cfg::SuspensionInputs bad;
        bad.fork_travel = 0.0;
        (void) cfg::resolve_suspension(bad);
    });
}

} // namespace

int main() {
    return bt::run([] {
        test_defaults();
        test_schema();
        test_validation();
        test_aliases_and_resolution();
        test_resolved_profiles();
        test_profile_coverage();
        test_wire_validation();
        test_rider();
        test_loaders();
        test_suspension();
    });
}
