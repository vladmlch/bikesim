// Completion 3.28. Source: tests/reference/test_native_input_contracts.py.
// Strict wire readers and explicit contracts, with valid controls. Every
// ported record is self-contained (contract/invariant/analytic) — no Python
// oracle value is compared, so this file holds no golden nodes.
//
// Seventeen source records remain `planned` (not ported): their pytest
// nodes take the module-scoped `resolved` fixture, whose `_golden`
// environment build currently fails at the documented macOS/Linux
// savage-plant equilibrium defect — `RuntimeError: physical equilibrium did
// not converge after 64000 steps: residual=0.094603` (front_wheel_spin
// dominant) — the same physics-stop signature as baseline nodes
// `test_realistic_pedelec_acceptance.py::test_savage_is_ridden_to_the_end`
// and `test_realtime_gate.py::...[rough_uphill_savage.toml]` measured at
// fe4feaa (migration/baseline-refresh-2026-10-23.md). The pytest run for
// this batch confirms 59 fixture-setup errors on these nodes and no test
// failures elsewhere. Per the batch stop rule they are not ported until
// the fixture is green again:
//   test_recursive_config_key_contract
//   test_present_empty_writer_requires_its_schema
//   test_schema_is_closed_integer
//   test_boolean_flags_are_exact
//   test_numeric_types_precede_conversion
//   test_numpy_numeric_scalars_and_ordered_vectors
//   test_vectors_reject_unordered_wrong_width_and_boolean
//   test_schema_migration_preserves_resolved_fork_force (would-be golden)
//   test_schema_two_requires_explicit_fork_flag
//   test_projection_emits_schema_two_and_persisted_flag
//   test_closed_discriminators_have_full_field_paths
//   test_schema_one_rejects_schema_two_fork_metadata
//   test_resistance_snapshot_patch_schema_is_complete
//   test_zero_dimensional_arrays_are_not_wire_sequences
//   test_ordered_dictionary_collections_match_python
//   test_mapping_is_not_a_numeric_sequence
//   test_scalar_conversion_memory_error_keeps_its_meaning
// Their native-side equivalents run against a valid resolved wire document
// once the fixture heals.
#include <limits>
#include <string>
#include <vector>

#include "configuration/model_config.hpp"
#include "diag.hpp"
#include "drivetrain_reference_support.hpp"
#include "io/json.hpp"

namespace {
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
        test_control_and_recursive_state_keys_are_value_errors();
        test_state_numeric_arrays_reject_boolean_before_cast();
        test_public_config_requires_mapping_before_model_load();
        test_projection_canonicalizes_valid_real_integer_after_validation();
        test_standalone_projection_checks_closed_discriminators();
        test_projection_does_not_turn_unordered_or_scalar_cassette_into_sequence();
        test_huge_integer_real_conversion_and_large_finite_control();
        test_wire_string_encoding_matches_native();
    });
}
