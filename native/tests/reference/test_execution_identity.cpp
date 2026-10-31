// Completion 3.36 tranche B (T53). Source: tests/test_execution_identity.py.
//
// The Python suite drove bike_sim.native.execution_identity —
// validate_execution_identity, execution_identity_differences and
// require_execution_compatible over hand-built record dicts. The data-only
// native owners are bike::replay::validate_execution (replay/legacy.cpp, the
// recorded-execution schema gate every reader runs before touching files)
// and bike::replay::check_execution (replay/rebuild.cpp, the
// recorded-versus-live compatibility check at session open). The native
// executable profile carries the provenance-bound identity fields
// (native_source_sha256, execution_identity, mujoco_library_sha256,
// numeric_provider); the schema-2 profile keeps the persisted
// platform/numerics context the historical records used.
//
// Dropped nodes (python-only):
//   * test_single_field_mismatch_reports_the_dotted_key (17 params),
//     test_os_and_backend_pairing_change_reports_the_fields,
//     test_provider_hash_names_the_indexed_library_path,
//     test_library_list_length_difference_is_indexed,
//     test_legacy_vs_v2_reports_the_missing_identity_fields —
//     execution_identity_differences' recursive record-vs-record field-path
//     differ has no native subject: the only replayable profile natively is
//     the native-executable one, whose bound fields check_execution compares
//     against live provenance and names on mismatch (ported below as
//     recorded_mismatch_names_the_field). Malformed nested fields reject in
//     validate_execution — covered by nested_field_rules_fail_closed.
//   * test_validation_imports_nothing — sys.modules accounting is
//     interpreter bookkeeping; validate_execution is a pure io::Value
//     function with no import machinery behind it.
#include "replay_reference_support.hpp"
#include "engine_identity.hpp"
#include "platform/provenance.hpp"
#include "runtime/provenance.hpp"
#include <algorithm>
#include <set>

namespace {
namespace rr = replay_reference;
using rr::count;
using rr::field;
using rr::find;
using rr::integer;
using rr::set;
using rr::text;
using bike::io::Value;
const char* HEX64_A = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const char* HEX64_B = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
const char* HEX64_C = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
const char* HEX64_D = "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";
const char* HEX64_E = "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee";
const char* HEX64_F = "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff";

// Darwin+ACCELERATE v2 fixture — the Python v2_native()/v2_python() record.
// rr::identity only builds the Linux+NUMPY_OPENBLAS pairing; several nested
// rules are backend-qualified, so the ACCELERATE face needs its own fixture.
Value platform_darwin() {
    return Value{Value::Object{
        {"system", text("Darwin")}, {"machine", text("aarch64")},
        {"libc", Value{Value::Object{{"name", text("libSystem")},
                                     {"version", text("26A434")}}}},
        {"os_build", text("26A434")}, {"python_version", text("3.13.16")},
        {"python_soabi", text("cpython-313-darwin")}}};
}
Value numerics_accelerate() {
    return Value{Value::Object{
        {"backend", text("ACCELERATE")}, {"numpy_version", text("2.5.2")},
        {"blas_integer_bits", integer(32)}, {"lapack_integer_bits", integer(64)},
        {"cpu_profile", text("Apple M4 Max")},
        {"threading", Value{Value::Object{
            {"implementation", text("accelerate")}, {"num_threads", integer(4)}}}},
        {"libraries", Value{Value::Array{Value{Value::Object{
            {"role", text("blas_lapack")}, {"identity_kind", text("apple-system")},
            {"sha256", Value{}},
            {"build_id", text("macos-26A434+accelerate-"
                "a697e7b9-8ff4-356c-a507-64b38af1ccaa+sdk-26A425")}}}}}}}};
}
Value identity_darwin(std::string_view backend) {
    const bool native = backend == "native";
    Value record{Value::Object{
        {"backend", text(backend)},
        {"runtime_schema", integer(1)},
        {"python_source_sha256", text(HEX64_A)},
        {"native_source_sha256", native ? text(HEX64_B) : Value{}},
        {"extension_sha256", native ? text(HEX64_C) : Value{}},
        {"mujoco_version", text("3.12.0")},
        {"mujoco_library_sha256", native ? text(HEX64_D) : Value{}},
        {"build_context", native ? Value{Value::Object{
            {"build_type", text("Release")}, {"compiler_id", text("AppleClang")},
            {"compiler_version", text("21.0.0")},
            {"numerical_flags", text("-O3 -ffp-contract=off")}}} : Value{}}}};
    auto context = native ? field(record, "build_context")
                          : Value{Value::Object{}};
    set(context, "platform", platform_darwin());
    set(context, "numerics", numerics_accelerate());
    set(record, "build_context", std::move(context));
    set(record, "identity_schema", integer(2));
    return record;
}

// A ReplayInput carrying a valid native-executable identity bound to this
// build's live provenance — the only profile check_execution accepts.
bike::replay::ReplayInput native_executable_input() {
    const auto runtime_id = runtime::runtime_identity();
    const auto engine_id = engine::identity();
    Value execution{Value::Object{
        {"backend", text("native")},
        {"native_source_sha256", text(runtime_id.native_source_sha256)},
        {"execution_identity", text(bike::platform::execution_identity())},
        {"mujoco_library_sha256", text(engine_id.library_sha256)},
        {"numeric_provider", text(engine_id.numeric_provider)},
        {"executable_sha256", text(HEX64_E)},
        {"source_identity_scope", text("test scope only")}}};
    bike::replay::ReplayInput input;
    input.replay_contract_version = 2;
    input.manifest = Value{Value::Object{{"execution", execution}}};
    input.summary = Value{Value::Object{
        {"model_source_sha256", text(runtime_id.native_source_sha256)},
        {"research", Value{Value::Object{{"execution", execution}}}}}};
    return input;
}
// Mutate one member of manifest.execution inside a live-bound input.
bike::replay::ReplayInput mutated_input(std::string_view key, const Value& value) {
    auto input = native_executable_input();
    auto execution = field(input.manifest, "execution");
    set(execution, key, value);
    set(input.manifest, "execution", std::move(execution));
    return input;
}

// One nested mutation step inside build_context: an object key, or a step
// into element 0 of an array member (libraries[0]).
struct Step { const char* key; bool into_first_element; };
void mutate_context(Value& record, std::initializer_list<Step> steps,
                    const Value& bad) {
    auto context = field(record, "build_context");
    Value* node = &context;
    for (const auto& step : steps) {
        if (step.into_first_element) {
            auto& array = std::get<Value::Array>(node->data);
            bt::require(!array.empty(), "library path index exists in the fixture");
            node = &array.front();
            continue;
        }
        auto& object = std::get<Value::Object>(node->data);
        bool found = false;
        for (auto& [name, child] : object) {
            if (name == step.key) { node = &child; found = true; break; }
        }
        bt::require(found, "fixture carries the mutated key");
    }
    *node = bad;
    set(record, "build_context", std::move(context));
}
void erase_context_key(Value& record, std::string_view key) {
    auto context = field(record, "build_context");
    auto& object = std::get<Value::Object>(context.data);
    object.erase(std::remove_if(object.begin(), object.end(),
        [&](const auto& pair) { return pair.first == key; }), object.end());
    set(record, "build_context", std::move(context));
}
// Add an "extra" member to the named context object.
void inject_context_extra(Value& record, std::string_view scope) {
    auto context = field(record, "build_context");
    auto scoped = field(context, scope);
    set(scoped, "extra", text("untracked"));
    set(context, scope, std::move(scoped));
    set(record, "build_context", std::move(context));
}

// --- schema acceptance ----------------------------------------------------

// test_legacy_read_stays_valid: the unversioned python record validates and
// gains no identity_schema member.
void legacy_read_stays_valid() {
    const auto record = rr::identity("python");
    bike::replay::validate_execution(record);
    bt::require(!find(record, "identity_schema"),
        "legacy reads do not grow a schema marker");
}

// test_legacy_native_read_stays_valid.
void legacy_native_read_stays_valid() {
    bike::replay::validate_execution(rr::identity("native"));
}

// test_v2_records_validate: python/native records on both qualified
// pairings validate with the schema markers intact.
void v2_records_validate() {
    for (const auto& record : {rr::identity("python", true), rr::identity("native", true),
             identity_darwin("native"), identity_darwin("python")}) {
        bike::replay::validate_execution(record);
        bt::require(count(field(record, "identity_schema")) == 2 &&
            count(field(record, "runtime_schema")) == 1,
            "the versioned record keeps identity_schema=2 and runtime_schema=1");
    }
}

// test_validation_does_not_mutate_the_record: canonical bytes are identical
// before and after validation (the input is a const record natively).
void validation_does_not_mutate() {
    for (const auto& record : {rr::identity("python"), rr::identity("native"),
             rr::identity("python", true), rr::identity("native", true)}) {
        const auto before = bike::io::canonical_json(record);
        bike::replay::validate_execution(record);
        bt::require(bike::io::canonical_json(record) == before,
            "validation never mutates the record it checks");
    }
}

// --- identity_schema dispatch is exact -------------------------------------

// test_unknown_identity_schema_is_not_guessed.
void unknown_identity_schema_is_not_guessed() {
    auto bad = rr::identity("python");
    set(bad, "identity_schema", integer(999));
    rr::rejects([&] { bike::replay::validate_execution(bad); }, "identity_schema");
}

// test_noncanonical_identity_schema_fails: bool/int/str/real/null spellings
// of anything but the integer 2 all reject.
void noncanonical_identity_schema_fails() {
    const Value bad_values[] = {Value{true}, Value{false}, integer(1), integer(3),
        text("2"), rr::real(2.0), Value{}};
    for (const auto& schema : bad_values) {
        auto bad = rr::identity("python", true);
        set(bad, "identity_schema", schema);
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
}

// test_v2_rejects_missing_and_extra_top_level_keys.
void v2_rejects_missing_and_extra_top_level_keys() {
    for (const auto* backend : {"python", "native"}) {
        const auto expected = rr::identity(backend, true);
        for (const auto& key : {"backend", "runtime_schema", "python_source_sha256",
                 "native_source_sha256", "extension_sha256", "mujoco_version",
                 "mujoco_library_sha256", "build_context", "identity_schema"}) {
            auto bad = expected;
            auto& object = std::get<Value::Object>(bad.data);
            object.erase(std::remove_if(object.begin(), object.end(),
                [&](const auto& pair) { return pair.first == key; }), object.end());
            rr::rejects([&] { bike::replay::validate_execution(bad); });
        }
        auto bad = expected;
        set(bad, "extra", text("untracked"));
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
}

// test_legacy_rejects_the_v2_field_set.
void legacy_rejects_the_v2_field_set() {
    auto bad = rr::identity("python");
    set(bad, "extra", integer(1));
    rr::rejects([&] { bike::replay::validate_execution(bad); });
}

// test_native_context_requires_exactly_six_keys: every native context member
// is required and an injected key rejects.
void native_context_requires_exactly_six_keys() {
    for (const auto* key : {"build_type", "compiler_id", "compiler_version",
             "numerical_flags", "platform", "numerics"}) {
        auto bad = identity_darwin("native");
        erase_context_key(bad, key);
        rr::rejects([&] { bike::replay::validate_execution(bad); }, "build_context");
    }
    auto bad = identity_darwin("native");
    auto context = field(bad, "build_context");
    set(context, "injected", text("untracked"));
    set(bad, "build_context", std::move(context));
    rr::rejects([&] { bike::replay::validate_execution(bad); }, "build_context");
}

// test_python_v2_context_is_only_platform_and_numerics.
void python_v2_context_is_only_platform_and_numerics() {
    auto bad = identity_darwin("python");
    auto context = field(bad, "build_context");
    set(context, "build_type", text("Release"));
    set(bad, "build_context", std::move(context));
    rr::rejects([&] { bike::replay::validate_execution(bad); }, "build_context");
}

// --- field-level rules -----------------------------------------------------

// test_field_type_and_null_rules: the eight backend/field/bad-value rows.
void field_type_and_null_rules() {
    const std::pair<const char*, Value> python_rows[] = {
        {"runtime_schema", Value{true}}, {"runtime_schema", integer(2)},
        {"extension_sha256", text("0000000000000000000000000000000000000000000000000000000000000000")},
        {"native_source_sha256", text("0000000000000000000000000000000000000000000000000000000000000000")},
        {"mujoco_library_sha256", text("0000000000000000000000000000000000000000000000000000000000000000")}};
    for (const auto& [key, bad_value] : python_rows) {
        auto bad = rr::identity("python", true);
        set(bad, key, bad_value);
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
    const std::pair<const char*, Value> native_rows[] = {
        {"python_source_sha256", text("not-a-digest")},
        {"extension_sha256", Value{}},
        {"mujoco_version", text("")}};
    for (const auto& [key, bad_value] : native_rows) {
        auto bad = rr::identity("native", true);
        set(bad, key, bad_value);
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
}

// test_nested_field_rules_fail_closed: the 27 ACCELERATE-record mutations
// (extra-key rows are applied by inject_context_extra below).
void nested_field_rules_fail_closed() {
    const struct { std::initializer_list<Step> path; Value bad; } rows[] = {
        {{{"platform", false}, {"system", false}}, text("FreeBSD")},
        {{{"platform", false}, {"machine", false}}, text("")},
        {{{"platform", false}, {"machine", false}}, text("ARM 64")},
        {{{"platform", false}, {"libc", false}},
            Value{Value::Object{{"name", text("glibc")}}}},
        {{{"platform", false}, {"libc", false}},
            Value{Value::Object{{"name", text("glibc")}, {"version", text("2.39")},
                {"x", integer(1)}}}},
        {{{"platform", false}, {"libc", false}, {"name", false}}, text("")},
        {{{"platform", false}, {"os_build", false}}, text("   ")},
        {{{"platform", false}, {"python_version", false}}, text("")},
        {{{"platform", false}, {"python_soabi", false}}, Value{}},
        {{{"numerics", false}, {"backend", false}}, text("MKL")},
        {{{"numerics", false}, {"numpy_version", false}}, text("")},
        {{{"numerics", false}, {"blas_integer_bits", false}}, Value{true}},
        {{{"numerics", false}, {"blas_integer_bits", false}}, integer(64)},
        {{{"numerics", false}, {"lapack_integer_bits", false}}, integer(32)},
        {{{"numerics", false}, {"cpu_profile", false}}, text("")},
        {{{"numerics", false}, {"threading", false}},
            Value{Value::Object{{"implementation", text("accelerate")}}}},
        {{{"numerics", false}, {"threading", false}, {"num_threads", false}}, integer(0)},
        {{{"numerics", false}, {"threading", false}, {"num_threads", false}}, Value{true}},
        {{{"numerics", false}, {"libraries", false}}, Value{Value::Array{}}},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"role", false}},
            text("blas")},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"identity_kind", false}},
            text("file")},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"sha256", false}},
            text(HEX64_F)},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"build_id", false}},
            Value{}},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"build_id", false}},
            text("")},
    };
    for (const auto& row : rows) {
        auto bad = identity_darwin("native");
        mutate_context(bad, row.path, row.bad);
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
    // Injected keys violate the exact-key rule at the platform, numerics and
    // library-entry levels.
    for (const auto* scope : {"platform", "numerics"}) {
        auto bad = identity_darwin("native");
        inject_context_extra(bad, scope);
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
    {
        auto bad = identity_darwin("native");
        auto context = field(bad, "build_context");
        auto numerics = field(context, "numerics");
        auto libraries = field(numerics, "libraries");
        auto& array = std::get<Value::Array>(libraries.data);
        auto first = array.front();
        set(first, "extra", text("untracked"));
        array.front() = std::move(first);
        set(numerics, "libraries", std::move(libraries));
        set(context, "numerics", std::move(numerics));
        set(bad, "build_context", std::move(context));
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
}

// test_linux_file_identity_rules: the five NUMPY_OPENBLAS-record mutations.
void linux_file_identity_rules() {
    const struct { std::initializer_list<Step> path; Value bad; } rows[] = {
        {{{"numerics", false}, {"blas_integer_bits", false}}, integer(32)},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"identity_kind", false}},
            text("apple-system")},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"sha256", false}},
            Value{}},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"sha256", false}},
            text("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF")},
        {{{"numerics", false}, {"libraries", false}, {"", true}, {"build_id", false}},
            text("fabricated")},
    };
    for (const auto& row : rows) {
        auto bad = rr::identity("native", true);
        mutate_context(bad, row.path, row.bad);
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
}

// test_integer_abi_change_is_a_named_failure: an out-of-range integer width
// rejects naming the integer-ABI contract on both pairings.
void integer_abi_change_is_a_named_failure() {
    for (const auto* field_name : {"blas_integer_bits", "lapack_integer_bits"}) {
        for (const auto& record : {identity_darwin("native"), rr::identity("native", true)}) {
            auto bad = record;
            mutate_context(bad, {{"numerics", false}, {field_name, false}}, integer(128));
            rr::rejects([&] { bike::replay::validate_execution(bad); }, "integer ABI");
        }
    }
}

// test_os_change_is_a_named_failure: a bare system mutation leaves an
// inconsistent pairing that names platform.system.
void os_change_is_a_named_failure() {
    {
        auto bad = identity_darwin("native");
        mutate_context(bad, {{"platform", false}, {"system", false}}, text("Linux"));
        rr::rejects([&] { bike::replay::validate_execution(bad); }, "platform.system");
    }
    {
        auto bad = rr::identity("native", true);
        mutate_context(bad, {{"platform", false}, {"system", false}}, text("Darwin"));
        rr::rejects([&] { bike::replay::validate_execution(bad); }, "platform.system");
    }
}

// --- recorded-vs-live compatibility (check_execution) ----------------------

// test_mismatch_names_the_field (the native bound-field analog) and the
// indexed-sha naming semantic of test_provider_hash_names_the_indexed_
// library_path: a recorded identity field that differs from live provenance
// names the bound field in the rejection.
void recorded_mismatch_names_the_field() {
    for (const auto* key : {"mujoco_library_sha256", "execution_identity",
             "native_source_sha256"}) {
        auto input = mutated_input(key, text(HEX64_F));
        rr::incompatible([&] { bike::replay::check_execution(input); }, key);
    }
    auto input = mutated_input("numeric_provider", text("foreign provider"));
    rr::incompatible([&] { bike::replay::check_execution(input); }, "numeric_provider");
}

// test_differences_validate_both_sides: the recorded side is validated
// before comparison; the live side is compiled provenance that cannot be
// malformed. A malformed recorded object rejects before field comparison,
// and validate_execution rejects incomplete objects directly.
void recorded_side_is_validated() {
    rr::rejects([&] {
        bike::replay::validate_execution(
            Value{Value::Object{{"backend", text("python")}}}); });
    auto input = native_executable_input();
    auto execution = field(input.manifest, "execution");
    auto& object = std::get<Value::Object>(execution.data);
    object.erase(std::remove_if(object.begin(), object.end(),
        [](const auto& pair) { return pair.first == "numeric_provider"; }), object.end());
    set(input.manifest, "execution", std::move(execution));
    rr::rejects([&] { bike::replay::check_execution(input); });
}

// test_require_execution_compatible_accepts_identity: a recorded identity
// equal to live provenance passes.
void compatible_accepts_identity() {
    auto input = native_executable_input();
    bike::replay::check_execution(input);
}

// test_require_execution_compatible_raises_with_named_fields: every bound
// field mismatch names that field; a foreign recorded plant source is named
// as well.
void compatible_raises_with_named_fields() {
    recorded_mismatch_names_the_field();
    auto input = native_executable_input();
    set(input.summary, "model_source_sha256", text(HEX64_A));
    rr::incompatible([&] { bike::replay::check_execution(input); }, "plant source");
}

// test_require_execution_compatible_rejects_legacy_recorded: schema-1/2
// python and binding-native records carry no certified native-executable
// profile and fail closed as historical.
void compatible_rejects_legacy_recorded() {
    for (const auto& execution : {rr::identity("python"), rr::identity("native"),
             rr::identity("python", true), rr::identity("native", true)}) {
        bike::replay::ReplayInput input;
        input.replay_contract_version = 2;
        input.manifest = Value{Value::Object{{"execution", execution}}};
        input.summary = Value{Value::Object{{"research",
            Value{Value::Object{{"execution", execution}}}}}};
        rr::incompatible([&] { bike::replay::check_execution(input); }, "historical");
    }
    bike::replay::ReplayInput version_one;
    version_one.replay_contract_version = 1;
    rr::incompatible([&] { bike::replay::check_execution(version_one); }, "historical");
}

} // namespace
int main() {
    return bt::run([] {
        legacy_read_stays_valid();
        legacy_native_read_stays_valid();
        v2_records_validate();
        validation_does_not_mutate();
        unknown_identity_schema_is_not_guessed();
        noncanonical_identity_schema_fails();
        v2_rejects_missing_and_extra_top_level_keys();
        legacy_rejects_the_v2_field_set();
        native_context_requires_exactly_six_keys();
        python_v2_context_is_only_platform_and_numerics();
        field_type_and_null_rules();
        nested_field_rules_fail_closed();
        linux_file_identity_rules();
        integer_abi_change_is_a_named_failure();
        os_change_is_a_named_failure();
        recorded_mismatch_names_the_field();
        recorded_side_is_validated();
        compatible_accepts_identity();
        compatible_raises_with_named_fields();
        compatible_rejects_legacy_recorded();
    });
}
