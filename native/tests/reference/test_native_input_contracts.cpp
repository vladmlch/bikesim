// Completion 3.28. Source: tests/reference/test_native_input_contracts.py.
// Strict wire readers and explicit contracts, with valid controls. Every
// resolved()-fixture record is ported: the module-scoped Python fixture is
// project(env) over the welded_pinned bootstrap plus fixed overrides, so
// the port replays the captured wire/state instead of re-running a live
// environment. test_schema_migration_preserves_resolved_fork_force is the
// file's one golden — bitwise oracle scalars captured on darwin-arm64 and
// linux-aarch64.
#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "configuration/model_config.hpp"
#include "diag.hpp"
#include "drivetrain_reference_support.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"
#include "model/builder.hpp"
#include "runtime/value_readers.hpp"
#include "writer_config.hpp"
#include "writers/resistance.hpp"

namespace {
namespace io = bike::io;
using namespace drivetrain_reference;
using policy_reference::field;
using policy_reference::member;
using policy_reference::set;

// Recursive `dictionaries` twin: yields every object-valued location in
// insertion order (dict-valued paths only — lists recurse without yielding).
void dictionaries(const Value& value, std::vector<std::string>& path,
                  std::vector<std::vector<std::string>>& out) {
    if (const auto* object = std::get_if<Value::Object>(&value.data)) {
        out.push_back(path);
        for (const auto& [key, item] : *object) {
            path.push_back(key);
            dictionaries(item, path, out);
            path.pop_back();
        }
    } else if (const auto* items = std::get_if<Value::Array>(&value.data)) {
        for (std::size_t i = 0; i < items->size(); ++i) {
            path.push_back(std::to_string(i));
            dictionaries((*items)[i], path, out);
            path.pop_back();
        }
    }
}
Value& at(Value& value, const std::vector<std::string>& path) {
    Value* cursor = &value;
    for (const auto& key : path) {
        if (std::holds_alternative<Value::Object>(cursor->data)) {
            cursor = &member(*cursor, key);
        } else {
            cursor = &std::get<Value::Array>(cursor->data)
                          .at(static_cast<std::size_t>(std::stoull(key)));
        }
    }
    return *cursor;
}
// Literal-path overload: brace lists bind to a span directly.
Value& at(Value& value, std::initializer_list<std::string_view> path) {
    Value* cursor = &value;
    for (const auto key : path) {
        if (std::holds_alternative<Value::Object>(cursor->data)) {
            cursor = &member(*cursor, key);
        } else {
            cursor = &std::get<Value::Array>(cursor->data)
                          .at(static_cast<std::size_t>(std::stoull(std::string(key))));
        }
    }
    return *cursor;
}
std::string join(std::span<const std::string> parts) {
    std::string out;
    for (const auto& part : parts) { if (!out.empty()) out += '.'; out += part; }
    return out;
}
// Python's `at(value, path[:-1])[path[-1]] = leaf`: object parents gain or
// replace the named member; array parents assign the indexed element.
void put(Value& root, const std::vector<std::string>& path,
         const Value& leaf) {
    Value& parent =
        at(root, std::vector<std::string>(path.begin(), path.end() - 1));
    if (std::holds_alternative<Value::Object>(parent.data))
        set(parent, path.back(), leaf);
    else
        std::get<Value::Array>(parent.data)
            .at(static_cast<std::size_t>(std::stoull(path.back()))) = leaf;
}

// ---- the resolved() fixture ------------------------------------------------
// The Python module-scoped `resolved` builds _pinned_config physics (the
// welded_pinned bootstrap plant), projects it, then applies the cruise,
// assist profile/mode, rider-path and surface-section overrides below. The
// static bootstrap captures exactly that projection plus the settled state,
// so the port replays it rather than building a live environment (same seam
// as test_native_numeric_completion.cpp's resolved_wire()).
std::string pinned_path() {
    return bt::fixture("bootstrap/welded_pinned/model.mjb").string();
}
const std::vector<double>& pinned_integration() {
    static const std::vector<double> state =
        bt::read_f64(bt::fixture("bootstrap/welded_pinned/integration.f64"));
    return state;
}
// mjSTATE_INTEGRATION layout: [time, qpos(26), qvel(26), act(0),
// qacc_warmstart(26), plugin] for the nq=nv=26, na=0 plant.
std::vector<double> pinned_qpos() {
    const auto& v = pinned_integration();
    return {v.begin() + 1, v.begin() + 27};
}
int pinned_adr(const std::string& joint) {
    static const bike::model::ModelOwner model(
        engine::load_model(pinned_path().c_str()));
    return bike::model::joint_address(*model, joint).first;
}
int pinned_dof(const std::string& joint) {
    static const bike::model::ModelOwner model(
        engine::load_model(pinned_path().c_str()));
    return bike::model::joint_address(*model, joint).second;
}
Value resolved_wire() {
    auto wire = field(io::parse_json(
        bt::text(bt::fixture("bootstrap/welded_pinned/config.json"))), "writer_config");
    set(wire, "cruise", io::parse_json(
        R"({"target_speed_kmh":25.0,"kp_nm_per_mps":1.0,"ki_nm_per_mps_s":0.1,"torque_ceiling_nm":20.0})"));
    auto& assist = member(member(wire, "drive"), "assist");
    set(assist, "profile", io::parse_json(
        R"({"mode_gains":{"eco":1.0,"tour":2.0,"emtb":[1.0,3.0],"turbo":3.0},"emtb_full_gain_at_nm":20.0})"));
    set(assist, "mode", text("eco"));
    set(wire, "rider_forces", io::parse_json(
        R"({"paths":[{"name":"probe_path","joint":"root_x","stiffness_n_m":1.0,"damping_ns_m":0.0,"preload_deflection_m":0.0,"offset_m":0.0,"unilateral":false}]})"));
    auto& surface_map = member(member(wire, "tire"), "surface_map");
    auto& sections = member(surface_map, "sections");
    if (io::as_array(sections, "tire.surface_map.sections").empty()) {
        // surface=copy.deepcopy(config['tire']['surface_map']['surface'])
        const Value surface = field(surface_map, "surface");
        Value entry{Value::Object{{"start_m", real(1.)}, {"end_m", real(2.)},
                                  {"surface", surface}}};
        sections = Value{Value::Array{std::move(entry)}};
    }
    return wire;
}
std::unique_ptr<Stepper> resolved_stepper(const Value& wire) {
    return std::make_unique<Stepper>(pinned_path(), nativecfg::decode(wire));
}

// ---- golden plumbing --------------------------------------------------------
bt::oracle::Node node(std::string_view function, std::string_view id = {}) {
    auto name = "tests/reference/test_native_input_contracts.py::" +
                std::string(function);
    if (!id.empty()) name += "[" + std::string(id) + "]";
    return bt::oracle::Node(name);
}
void scalar(bt::oracle::Node& golden, double value) {
    const std::array values{value};
    golden.f64(values, std::span<const std::size_t>{}, "");
}

// Source: test_native_input_contracts.py::test_recursive_config_key_contract
[[gnu::noinline]] void test_recursive_config_key_contract() {
    // mutations unknown/nonstring/missing over every object-valued location.
    // 'nonstring' (key=7) has no wire spelling — Value::Object keys are
    // strings by construction; the nb-side "expected string" gate stays with
    // the parity suite until T63.
    const Value original = resolved_wire();
    (void)resolved_stepper(original);  // Stepper(path, original): valid control
    std::vector<std::vector<std::string>> locations;
    { std::vector<std::string> path; dictionaries(original, path, locations); }
    bt::require(!locations.empty(), "resolved wire exposes object locations");
    static const std::set<std::string> root_sections{
        "drive", "cruise", "suspension", "brake", "resistance", "tire",
        "rider_forces", "rider_contacts"};
    static const std::set<std::string> optional_nested{
        "profile", "torque_curve", "surface_map", "grip_pair_force_limit_n"};
    for (const std::string_view mutation : {"unknown", "missing"}) {
        for (const auto& location : locations) {
            Value candidate = original;
            Value& section = at(candidate, location);
            const std::string expected = "config" +
                (location.empty() ? "" : "." + join(location));
            if (mutation == "missing") {
                const auto& excluded =
                    location.empty() ? root_sections : optional_nested;
                std::optional<std::string> victim;
                for (const auto& [key, ignored] :
                     std::get<Value::Object>(section.data)) {
                    (void)ignored;
                    if (!excluded.count(key) && (!victim || key < *victim))
                        victim = key;
                }
                if (!victim) continue;  // every key is optional: Python skips
                policy_reference::erase(section, *victim);
            } else {
                set(section, "misspelled", real(1.));
            }
            error<std::invalid_argument>(
                [&] { bike::configuration::wire::validate_config(candidate); },
                expected);
            error<std::invalid_argument>(
                [&] { (void)resolved_stepper(candidate); }, expected);
        }
    }
}

// Source: test_native_input_contracts.py::test_present_empty_writer_requires_its_schema
[[gnu::noinline]] void test_present_empty_writer_requires_its_schema() {
    for (const std::string_view section :
         {"drive", "cruise", "suspension", "brake", "resistance", "tire",
          "rider_forces", "rider_contacts"}) {
        const Value config{Value::Object{
            {"schema", integer(2)},
            {std::string(section), Value{Value::Object{}}}}};
        const std::string expected = "config." + std::string(section);
        error<std::invalid_argument>(
            [&] { bike::configuration::wire::validate_config(config); },
            expected);
        error<std::invalid_argument>(
            [&] { (void)resolved_stepper(config); }, expected);
    }
}

// Source: test_native_input_contracts.py::test_schema_is_closed_integer
[[gnu::noinline]] void test_schema_is_closed_integer() {
    // bad ∈ {True, np.bool_(False), '2', 2.5, 0, 3}: np.bool_ marshals as a
    // wire bool, so the two Python bool spellings are one Value arm each of
    // true/false; '2' is a text scalar; 2.5/0/3 stay real/integer lexemes.
    for (const Value& bad : {Value{true}, Value{false}, text("2"),
                             real(2.5), integer(0), integer(3)}) {
        const Value config{Value::Object{{"schema", bad}}};
        error<std::invalid_argument>(
            [&] { (void)resolved_stepper(config); }, "config.schema");
    }
}

// Source: test_native_input_contracts.py::test_boolean_flags_are_exact
[[gnu::noinline]] void test_boolean_flags_are_exact() {
    // bad ∈ {1, 0., 'no', np.bool_(True)}: np.bool_(True) fails the binding's
    // exact-Py_True gate, but on the wire a bool IS the accepted spelling —
    // the portable matrix is {1, 0., 'no'}.
    const Value original = resolved_wire();
    for (const std::vector<std::string>& location :
         std::vector<std::vector<std::string>>{
             {"suspension", "shock_damper", "lockout_firm"},
             {"suspension", "coil", "legacy_behavior"},
             {"drive", "pedaling", "enabled"}}) {
        const std::string expected = join(location);
        for (const Value& bad : {integer(1), real(0.), text("no")}) {
            Value candidate = original;
            put(candidate, location, bad);
            error<std::invalid_argument>(
                [&] { (void)resolved_stepper(candidate); }, expected);
        }
    }
}

// Source: test_native_input_contracts.py::test_numeric_types_precede_conversion
[[gnu::noinline]] void test_numeric_types_precede_conversion() {
    // bad ∈ {True, np.bool_(True), '1', nan, inf}: the bool covers both
    // spellings; nonfinite Python floats arrive as repr() lexemes.
    const Value original = resolved_wire();
    for (const std::vector<std::string>& location :
         std::vector<std::vector<std::string>>{
             {"brake", "torque_ceiling_nm"}, {"drive", "assist", "gain"},
             {"resistance", "wind_world_mps", "0"}}) {
        const std::string expected =
            join(std::span(location).first(location.size() - 1));
        for (const Value& bad :
             {Value{true}, text("1"),
              Value{bike::io::Number{"nan"}},
              Value{bike::io::Number{"inf"}}}) {
            Value candidate = original;
            put(candidate, location, bad);
            error<std::invalid_argument>(
                [&] { (void)resolved_stepper(candidate); }, expected);
        }
    }
}

// Source: test_native_input_contracts.py::test_numpy_numeric_scalars_and_ordered_vectors
[[gnu::noinline]] void test_numpy_numeric_scalars_and_ordered_vectors() {
    // np.int64/np.int32 arrive as integer lexemes, np.float32/np.float64 as
    // real lexemes; a tuple of them is the same ordered wire array.
    Value config = resolved_wire();
    set(config, "schema", integer(2));
    set(at(config, {"drive", "gearing"}), "front_teeth", integer(34));
    set(at(config, {"drive", "assist"}), "gain", real(1.));
    set(at(config, {"resistance"}), "wind_world_mps",
        Value{Value::Array{integer(0), real(0.), real(0.)}});
    (void)resolved_stepper(config);
}

// Source: test_native_input_contracts.py::test_vectors_reject_unordered_wrong_width_and_boolean
[[gnu::noinline]] void test_vectors_reject_unordered_wrong_width_and_boolean() {
    // bad ∈ {set{0,1,2}, iter([0,0,0]), [0,True,0], [0,0]}: a set's closest
    // wire kin is an unordered mapping; a bare iterator has no wire spelling
    // (a string is the same "not an ordered sequence" failure); the bool
    // element and wrong-width arms are real wire values.
    const Value original = resolved_wire();
    for (const Value& bad : {
             Value{Value::Object{{"0", real(0.)}, {"1", real(1.)},
                                 {"2", real(2.)}}},
             text("0.0,0.0,0.0"),
             Value{Value::Array{real(0.), Value{true}, real(0.)}},
             Value{Value::Array{real(0.), real(0.)}}}) {
        Value candidate = original;
        set(at(candidate, {"resistance"}), "wind_world_mps", bad);
        error<std::invalid_argument>(
            [&] { (void)resolved_stepper(candidate); },
            "resistance.wind_world_mps");
    }
}

// Source: test_native_input_contracts.py::test_schema_migration_preserves_resolved_fork_force
// GOLDEN — schema ∈ {1,2} × (travel,legacy) ∈ {(150,T),(180,F),(150,F)}:
// the C++ ids are the pytest parameter spellings; each node consumes the
// two bitwise fork_damper scalars (compression travel-1 then travel+15 mm).
[[gnu::noinline]] void test_schema_migration_preserves_resolved_fork_force() {
    const int fork_adr = pinned_adr("fork_travel");
    const int fork_dof = pinned_dof("fork_travel");
    struct Case { const char* id; int schema; double travel; bool legacy; };
    for (const Case item : std::array{
             Case{"1-150.0-True", 1, 150., true},
             Case{"1-180.0-False", 1, 180., false},
             Case{"1-150.0-False", 1, 150., false},
             Case{"2-150.0-True", 2, 150., true},
             Case{"2-180.0-False", 2, 180., false},
             Case{"2-150.0-False", 2, 150., false}}) {
        auto golden =
            node("test_schema_migration_preserves_resolved_fork_force", item.id);
        // Charger3Damper(total_travel_mm=travel, legacy_behavior=legacy):
        // _damper_core fields plus the fork extras; schema 2 also persists
        // the resolved flag (schema 1 derives it from hbo_start_mm == 160).
        Value::Object fork{
            {"max_hsc", integer(4)}, {"max_lsc", integer(14)},
            {"max_reb", integer(17)}, {"hsc_clicks", integer(2)},
            {"lsc_clicks", integer(7)}, {"rebound_clicks", integer(9)},
            {"c_lsc_min", real(260.)}, {"c_lsc_max", real(1150.)},
            {"c_hsc_min", real(180.)}, {"c_hsc_max", real(520.)},
            {"c_reb_min", real(350.)}, {"c_reb_max", real(1550.)},
            {"v_knee_comp", real(.18)}, {"v_knee_reb", real(.22)},
            {"total_travel_mm", real(item.travel)},
            {"hbo_start_mm", real(item.legacy ? 160. : item.travel - 20.)},
            {"c_hbo_base", real(3500.)}};
        if (item.schema == 2)
            fork.emplace_back("legacy_behavior", Value{item.legacy});
        Value wire = resolved_wire();
        set(wire, "schema", integer(item.schema));
        set(member(wire, "suspension"), "fork_damper",
            Value{std::move(fork)});
        auto stepper = resolved_stepper(wire);
        for (const double compression : {item.travel - 1., item.travel + 15.}) {
            auto qpos = pinned_qpos();
            std::vector<double> qvel(qpos.size(), 0.);
            qpos[static_cast<std::size_t>(fork_adr)] = compression / 1000.;
            qvel[static_cast<std::size_t>(fork_dof)] = .7;
            stepper->set_state(qpos, qvel, {},
                               std::vector<double>(qpos.size(), 0.), 0.);
            const auto components = stepper->suspension_components();
            const auto found = std::ranges::find_if(components,
                [](const auto& row) { return row.first == "fork_damper"; });
            bt::require(found != components.end(),
                        "suspension components carry fork_damper");
            scalar(golden,
                   found->second[static_cast<std::size_t>(fork_dof)]);
        }
        golden.done();
    }
}

// Source: test_native_input_contracts.py::test_schema_two_requires_explicit_fork_flag
[[gnu::noinline]] void test_schema_two_requires_explicit_fork_flag() {
    Value config = resolved_wire();
    policy_reference::erase(
        at(config, {"suspension", "fork_damper"}), "legacy_behavior");
    error<std::invalid_argument>(
        [&] { (void)resolved_stepper(config); }, "fork_damper.legacy_behavior");
}

// Source: test_native_input_contracts.py::test_projection_emits_schema_two_and_persisted_flag
[[gnu::noinline]] void test_projection_emits_schema_two_and_persisted_flag() {
    // The welded_pinned bootstrap's writer_config is the verbatim capture of
    // project(env) for exactly this env (tools/migration/capture_reference.py).
    const Value document = io::parse_json(
        bt::text(bt::fixture("bootstrap/welded_pinned/config.json")));
    const Value& wire = field(document, "writer_config");
    const auto& lexeme =
        std::get<bike::io::Number>(field(wire, "schema").data).lexeme;
    bt::require(lexeme == "2" && bike::io::is_integer_lexeme(lexeme),
                "project(env) emits schema as the integer 2");
    const Value& flag = field(field(field(wire, "suspension"), "fork_damper"),
                              "legacy_behavior");
    bt::require(std::holds_alternative<bool>(flag.data) &&
                !std::get<bool>(flag.data),
                "projection persists fork_damper.legacy_behavior as false");
}

// Source: test_native_input_contracts.py::test_closed_discriminators_have_full_field_paths
[[gnu::noinline]] void test_closed_discriminators_have_full_field_paths() {
    const Value original = resolved_wire();
    for (const std::vector<std::string>& location :
         std::vector<std::vector<std::string>>{
             {"suspension", "physics_mode"}, {"tire", "backend"},
             {"tire", "surface_mode"}, {"drive", "drive_mode"},
             {"drive", "transmission_model"},
             {"drive", "shifting", "upshift_slip_mode"},
             {"drive", "assist", "mode"}}) {
        Value candidate = original;
        put(candidate, location, text("unsupported"));
        const std::string expected = "config." + join(location);
        error<std::invalid_argument>(
            [&] { bike::configuration::wire::validate_config(candidate); },
            expected);
        error<std::invalid_argument>(
            [&] { (void)resolved_stepper(candidate); }, expected);
    }
}

// Source: test_native_input_contracts.py::test_schema_one_rejects_schema_two_fork_metadata
[[gnu::noinline]] void test_schema_one_rejects_schema_two_fork_metadata() {
    Value candidate = resolved_wire();
    set(candidate, "schema", integer(1));
    error<std::invalid_argument>(
        [&] { bike::configuration::wire::validate_config(candidate); },
        "fork_damper.legacy_behavior");
    error<std::invalid_argument>(
        [&] { (void)resolved_stepper(candidate); },
        "fork_damper.legacy_behavior");
}

// Value twin of the binding's side_input snapshot reader (binding.cpp
// side_input): section+exact over the closed per-side schema, then each
// patches[i] row exact against the four-member patch schema — vread:: is
// wire::'s documented mechanical-substitution twin, so the same "path:
// reason" lattice is re-asserted on the owned-Value document.
void patch_side_schema(const Value& snaps, std::string_view side) {
    const vread::Dict root{
        .value = &vread::mapping(vread::Node{&snaps}, "resistance_components"),
        .path = "resistance_components"};
    const vread::Dict parsed = vread::section(root, side);
    constexpr auto required =
        vread::keys("patch_loads", "patch_working", "eff_radius");
    constexpr auto optional =
        vread::keys("time_s", "interval_id", "backend", "geometric_contact",
                    "effective_radius_m", "patches");
    vread::exact(parsed, required, optional);
    if (parsed.contains("patches")) {
        const auto& patches =
            vread::sequence(parsed["patches"], parsed.child("patches"));
        std::size_t index = 0;
        for (const Value& row : patches) {
            const std::string patch_path =
                parsed.child("patches") + "." + std::to_string(index++);
            const vread::Dict patch{
                .value = &vread::mapping(vread::Node{&row}, patch_path),
                .path = patch_path};
            vread::exact(patch, vread::keys("normal_load_n", "tangent_force_n",
                                            "slip_mps", "working_surface"));
            for (const char* key : {"normal_load_n", "tangent_force_n", "slip_mps"})
                (void)vread::finite_real(patch[key], patch.child(key));
            (void)vread::boolean(patch["working_surface"],
                                 patch.child("working_surface"));
        }
    }
}

// Source: test_native_input_contracts.py::test_resistance_snapshot_patch_schema_is_complete
[[gnu::noinline]] void test_resistance_snapshot_patch_schema_is_complete() {
    // Stepper(path, config) is the same-path valid control; the snapshots
    // dict itself is the nb-boundary input side_input reads — see the mirror
    // comment above. The 'nonstring' arm (patch[7]) has no wire spelling.
    auto stepper = resolved_stepper(resolved_wire());
    const auto side = [] {
        return Value{Value::Object{
            {"patch_loads", Value{Value::Array{real(100.)}}},
            {"patch_working", Value{Value::Array{Value{true}}}},
            {"eff_radius", real(.3)},
            {"patches", Value{Value::Array{Value{Value::Object{
                {"normal_load_n", real(100.)},
                {"tangent_force_n", real(0.)}, {"slip_mps", real(0.)},
                {"working_surface", Value{true}}}}}}}}};
    };
    const Value snapshots{Value::Object{{"front", side()}, {"rear", side()}}};
    patch_side_schema(snapshots, "front");
    patch_side_schema(snapshots, "rear");
    {   // ...and the typed call the decoded snapshot feeds still answers
        // 'road_rolling' + 'aerodynamic' — the valid control's last leg.
        const std::array loads{100.};
        const std::array working{true};
        const TireSideInput input{loads, working, .3};
        bt::require(stepper->resistance_components(input, input).size() == 2,
                    "resistance components are road_rolling + aerodynamic");
    }
    for (const std::string_view mutation : {"unknown", "missing"}) {
        Value candidate = snapshots;
        Value& patch = at(candidate, {"front", "patches", "0"});
        if (mutation == "missing") policy_reference::erase(patch, "working_surface");
        else set(patch, "misspelled", real(1.));
        error<std::invalid_argument>(
            [&] { patch_side_schema(candidate, "front"); },
            "resistance_components.front.patches.0");
    }
}

// Source: test_native_input_contracts.py::test_zero_dimensional_arrays_are_not_wire_sequences
[[gnu::noinline]] void test_zero_dimensional_arrays_are_not_wire_sequences() {
    // np.array(1.) is a 0-d array — a scalar, not a sequence; its wire value
    // is the scalar itself.
    const Value original = resolved_wire();
    for (const std::vector<std::string>& location :
         std::vector<std::vector<std::string>>{
             {"resistance", "wind_world_mps"},
             {"drive", "shifting", "cassette"},
             {"drive", "assist", "torque_curve"}}) {
        Value candidate = original;
        put(candidate, location, real(1.));
        const std::string expected = "config." + join(location);
        error<std::invalid_argument>(
            [&] { bike::configuration::wire::validate_config(candidate); },
            expected);
        error<std::invalid_argument>(
            [&] { (void)resolved_stepper(candidate); }, expected);
    }
}

// Source: test_native_input_contracts.py::test_ordered_dictionary_collections_match_python
[[gnu::noinline]] void test_ordered_dictionary_collections_match_python() {
    // OrderedRows (Sequence of dicts) and np.array(rows, dtype=object) both
    // marshal as an ordered array of mappings — the identical wire value,
    // so each container arm reasserts the same rebuilt Array.
    const Value original = resolved_wire();
    for (const std::vector<std::string>& location :
         std::vector<std::vector<std::string>>{
             {"tire", "surface_map", "sections"},
             {"rider_forces", "paths"}}) {
        for (const std::string_view container : {"OrderedRows", "np-object-array"}) {
            (void)container;
            Value candidate = original;
            const auto& rows =
                std::get<Value::Array>(at(candidate, location).data);
            at(candidate, location) = Value{Value::Array(rows.begin(), rows.end())};
            bike::configuration::wire::validate_config(candidate);
            (void)resolved_stepper(candidate);
        }
    }
}

// Source: test_native_input_contracts.py::test_mapping_is_not_a_numeric_sequence
[[gnu::noinline]] void test_mapping_is_not_a_numeric_sequence() {
    // UserDict({0: 0., 1: 100.}) — iterating it would give a valid
    // width-2 load range, so accidental mapping-as-sequence would be
    // observable; the wire kin is an ordered mapping with "0"/"1" keys.
    Value candidate = resolved_wire();
    put(candidate, {"tire", "front", "material", "valid_load_range_n"},
        Value{Value::Object{{"0", real(0.)}, {"1", real(100.)}}});
    error<std::invalid_argument>(
        [&] { bike::configuration::wire::validate_config(candidate); },
        "tire.front.material.valid_load_range_n");
    error<std::invalid_argument>(
        [&] { (void)resolved_stepper(candidate); },
        "tire.front.material.valid_load_range_n");
}

// Source: test_native_input_contracts.py::test_scalar_conversion_memory_error_keeps_its_meaning
[[gnu::noinline]] void test_scalar_conversion_memory_error_keeps_its_meaning() {
    // The injected-__float__ MemoryError is the nb adapter's conversion_error
    // passthrough (binding_readers.hpp) — a Python-side channel with no wire
    // twin. The portable contract asserted here: a scalar whose conversion
    // fails keeps its full field path instead of degrading to a generic
    // error — a malformed lexeme at drive.assist.gain names it for both
    // readers, and no decoder swallows a real bad_alloc (nothing catches it).
    Value candidate = resolved_wire();
    put(candidate, {"drive", "assist", "gain"},
        Value{bike::io::Number{"2x"}});
    error<std::invalid_argument>(
        [&] { bike::configuration::wire::validate_config(candidate); },
        "config.drive.assist.gain");
    error<std::invalid_argument>(
        [&] { (void)resolved_stepper(candidate); },
        "config.drive.assist.gain");
}

// Source: test_native_input_contracts.py::test_control_and_recursive_state_keys_are_value_errors
[[gnu::noinline]] void test_control_and_recursive_state_keys_are_value_errors() {
    // Python parametrizes key over {'misspelled', 7}; a non-string dict key
    // is a Python-marshalling concept — the Value object's keys are strings
    // by construction — so the port covers 'misspelled'. The
    // drive_components({key: 1.}) half is the binding's closed nb::dict→
    // RideControl reader (drive_binding.cpp:182, `RideControl control(
    // nb::handle)`) — that check lives at the nanobind boundary and has no
    // Value-level decoder, so it stays with the parity suite until T63.
    for (auto kind : kinds) {
        auto owner = pair(kind);
        (void)components(*owner, .001, 0.);
        const auto original = state(*owner);
        const auto frozen_state = bike::io::canonical_json(original);
        std::vector<std::vector<std::string>> locations;
        { std::vector<std::string> path; dictionaries(original, path, locations); }
        bt::require(!locations.empty(), "snapshot exposes object locations");
        for (const auto& location : locations) {
            auto candidate = original;
            set(at(candidate, location), "misspelled", real(1.));
            error<std::invalid_argument>([&] { restore(*owner, candidate); }, "state");
            bt::require(bike::io::canonical_json(state(*owner)) == frozen_state,
                        "rejected state mutation publishes nothing");
        }
    }
}

// Source: test_native_input_contracts.py::test_state_numeric_arrays_reject_boolean_before_cast
[[gnu::noinline]] void test_state_numeric_arrays_reject_boolean_before_cast() {
    auto owner = pair();
    auto candidate = state(*owner);
    auto& hub = member(candidate, "ideal_hub");
    const auto width = std::get<Value::Array>(field(hub, "coefficients").data).size();
    set(hub, "coefficients", Value{Value::Array(width, Value{true})});
    error<std::invalid_argument>([&] { restore(*owner, candidate); }, "ideal_hub.coefficients");
}

// Source: test_native_input_contracts.py::test_public_config_requires_mapping_before_model_load
[[gnu::noinline]] void test_public_config_requires_mapping_before_model_load() {
    // validate_config(bad), Stepper('missing.mjb', bad), DrivePolicies(bad)
    // all reject non-mapping configs naming 'config'; the model-load
    // ordering half is structural on the native side — Stepper takes the
    // already-decoded WriterConfig, so no file access precedes the check.
    for (const auto& bad : {Value{Value::Array{}}, Value{}, Value{integer(1)}, Value{text("config")}}) {
        error<std::invalid_argument>([&] { bike::configuration::wire::validate_config(bad); }, "config");
        error<std::invalid_argument>([&] { (void)nativecfg::decode(bad); }, "config");
        error<std::invalid_argument>([&] { (void)nativecfg::decode_policy_config(bad); }, "config");
    }
}

// Source: test_native_input_contracts.py::test_projection_canonicalizes_valid_real_integer_after_validation
[[gnu::noinline]] void test_projection_canonicalizes_valid_real_integer_after_validation() {
    // The projection emits typed wire lexemes: real fields carry a real
    // spelling ("2.0" — real_value's ".0" marker keeps whole magnitudes
    // floats) while integer fields keep integer lexemes. The Python-side
    // `type() is float/int` assertion is exactly this wire-type split.
    auto config = policy_reference::config(policy_reference::source());
    const auto& gain =
        std::get<bike::io::Number>(field(member(config, "assist"), "gain").data).lexeme;
    bt::require(gain.find_first_of(".eE") != std::string::npos,
                "assist.gain projects as a real lexeme");
    const auto& teeth =
        std::get<bike::io::Number>(field(member(config, "gearing"), "front_teeth").data).lexeme;
    bt::require(teeth.find_first_of(".eE") == std::string::npos,
                "gearing.front_teeth projects as an integer lexeme");
}

// Source: test_native_input_contracts.py::test_standalone_projection_checks_closed_discriminators
[[gnu::noinline]] void test_standalone_projection_checks_closed_discriminators() {
    // project_drive rejects a bad closed discriminator naming 'config.drive':
    // drive_mode, transmission_model and the policy modes all have closed
    // domains — and assist.mode only closes when a profile is present.
    const auto drive_path = [](std::string_view leaf) {
        return "config.drive." + std::string(leaf);
    };
    for (const auto& item : std::initializer_list<std::pair<std::string_view, std::string_view>>{
             {"drive_mode", "unsupported"}, {"transmission_model", "unsupported"},
             {"shifting.upshift_slip_mode", "unsupported"}}) {
        auto wire = writer_config(source());
        auto& drive = member(wire, "drive");
        const auto dot = item.first.find('.');
        if (dot == std::string_view::npos) set(drive, std::string(item.first), text(item.second));
        else set(member(drive, item.first.substr(0, dot)), std::string(item.first.substr(dot + 1)),
                 text(item.second));
        error<std::invalid_argument>(
            [&] { bike::configuration::wire::validate_config(wire); }, drive_path(item.first));
    }
    {   // assist.mode closes only alongside a profile — same contract as the
        // Python profile=... / mode='unsupported' case. The profile keeps
        // its full schema shape so the mode check is the reached failure.
        auto wire = writer_config(source());
        auto& assist = member(member(wire, "drive"), "assist");
        set(assist, "profile", Value{Value::Object{
            {"mode_gains", Value{Value::Object{{"eco", real(1.)}, {"tour", real(2.)},
                {"emtb", Value{Value::Array{real(1.5), real(3.)}}}, {"turbo", real(4.)}}}},
            {"emtb_full_gain_at_nm", real(50.)}}});
        set(assist, "mode", text("unsupported"));
        error<std::invalid_argument>(
            [&] { bike::configuration::wire::validate_config(wire); }, "config.drive.assist.mode");
    }
}

// Source: test_native_input_contracts.py::test_projection_does_not_turn_unordered_or_scalar_cassette_into_sequence
[[gnu::noinline]] void test_projection_does_not_turn_unordered_or_scalar_cassette_into_sequence() {
    // Python feeds set/iter/0-d ndarray; the wire-level contract is that a
    // non-sequence cassette is rejected naming the field — mapping, scalar
    // and scalar-text all violate it at the Value boundary.
    for (const auto& bad : {Value{Value::Object{{"0", real(24.)}, {"1", real(34.)}}},
                            Value{integer(24)}, Value{bike::io::Number{"24"}}}) {
        auto config = policy_reference::config(policy_reference::source());
        set(member(config, "shifting"), "cassette", bad);
        error<std::invalid_argument>([&] { (void)nativecfg::decode_policy_config(config); }, "shifting.cassette");
    }
}

// Source: test_native_input_contracts.py::test_huge_integer_real_conversion_and_large_finite_control
[[gnu::noinline]] void test_huge_integer_real_conversion_and_large_finite_control() {
    // human_crank_torque(1e300, 0., 0.) == 1e300 bitwise: a huge-but-finite
    // mean keeps its value. The 10**1000 Python-int rejection reaches the
    // same ValueError('mean') the function raises for a nonfinite mean —
    // the binding's int→double overflow surfaces as nonfinite input.
    bt::require(drivetrain::human_crank_torque(1e300, 0., 0.) == 1e300,
                "huge finite mean converts identically");
    error<std::invalid_argument>(
        [&] { (void)drivetrain::human_crank_torque(std::numeric_limits<double>::infinity(), 0.); },
        "mean");
}

// Source: test_native_input_contracts.py::test_wire_string_encoding_matches_native
[[gnu::noinline]] void test_wire_string_encoding_matches_native() {
    // 'custom' and the non-ASCII mode are accepted wire strings; the lone
    // surrogate's rejection is the Python str→UTF-8 boundary check in the
    // binding — a C++ std::string carries raw bytes, so the portable half is
    // that both encodings decode through the same validator unchanged.
    for (const std::string& mode : {std::string("custom"), std::string("自定义")}) {
        auto config = policy_reference::config(policy_reference::source());
        set(member(config, "assist"), "mode", text(mode));
        auto decoded = nativecfg::decode_policy_config(config);
        bt::require(decoded.assist.mode == mode,
                    "wire preserves a valid free-form assist mode");
    }
}
} // namespace

// Each contract body is noinline so its by-value Value snapshots live in
// the callee frame — the dispatcher stays under the 8 KiB frame policy.
int main() {
    return bt::run([] {
        test_recursive_config_key_contract();
        test_present_empty_writer_requires_its_schema();
        test_schema_is_closed_integer();
        test_boolean_flags_are_exact();
        test_numeric_types_precede_conversion();
        test_numpy_numeric_scalars_and_ordered_vectors();
        test_vectors_reject_unordered_wrong_width_and_boolean();
        test_projection_canonicalizes_valid_real_integer_after_validation();
        test_standalone_projection_checks_closed_discriminators();
        test_schema_migration_preserves_resolved_fork_force();
        test_schema_two_requires_explicit_fork_flag();
        test_projection_emits_schema_two_and_persisted_flag();
        test_control_and_recursive_state_keys_are_value_errors();
        test_state_numeric_arrays_reject_boolean_before_cast();
        test_public_config_requires_mapping_before_model_load();
        test_closed_discriminators_have_full_field_paths();
        test_schema_one_rejects_schema_two_fork_metadata();
        test_resistance_snapshot_patch_schema_is_complete();
        test_zero_dimensional_arrays_are_not_wire_sequences();
        test_projection_does_not_turn_unordered_or_scalar_cassette_into_sequence();
        test_ordered_dictionary_collections_match_python();
        test_mapping_is_not_a_numeric_sequence();
        test_scalar_conversion_memory_error_keeps_its_meaning();
        test_huge_integer_real_conversion_and_large_finite_control();
        test_wire_string_encoding_matches_native();
    });
}
