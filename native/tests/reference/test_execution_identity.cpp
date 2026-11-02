// Port of tests/test_execution_identity.py — the data-only execution-identity
// contract: exact-key schema validation for the legacy 8-field record and the
// identity_schema=2 platform/numerics profile, plus the native-executable
// compatibility gate's named-field verdicts.
//
// Native mapping:
//   bike_sim.native.execution_identity.validate_execution_identity
//     -> bike::replay::validate_execution (replay/legacy.cpp; RecordingError)
//   require_execution_compatible(recorded, current)
//     -> bike::replay::check_execution over a ReplayInput carrying the live
//        native-executable profile; a foreign field throws
//        IncompatibleExecution("incompatible-execution: <field> differs").
//   execution_identity_differences(recorded, current)
//     -> no native twin by design: the native gate refuses the historical
//        platform/numerics profile wholesale ("historical Python/native-
//        binding arithmetic profile has no certified native-executable
//        compatibility fixture") and names the first mismatched digest of
//        the native-executable profile. Ports below assert those verdicts.
//
// Dropped source contracts (no native counterpart):
//   test_library_list_length_difference_is_indexed — the indexed-list diff
//   lives only in the Python diagnostic; the native-executable profile
//   carries a single mujoco_library_sha256 digest and the list-shape rules
//   are enforced by validate_execution (nested_field_rules_fail_closed).
//   test_validation_imports_nothing — a sys.modules probe; the native
//   validator is data-only compiled code with no import machinery.
#include "../support/fixture_test.hpp"
#include "engine_identity.hpp"
#include "io/json.hpp"
#include "platform/provenance.hpp"
#include "recording/analysis_detail.hpp"
#include "recording/reader.hpp"
#include "replay/input.hpp"
#include "replay/rebuild.hpp"
#include "replay/session.hpp"
#include "runtime/provenance.hpp"

#include <variant>

namespace {
namespace rec = bike::recording;
namespace replay = bike::replay;
using bike::io::Value;
using bike::io::canonical_json;
using rec::detail::field;
using rec::detail::integer;
using rec::detail::real;
using rec::detail::set;
using rec::detail::text;
using Object = Value::Object;
using Array = Value::Array;
using Path = std::vector<std::variant<std::string, std::size_t>>;

// ---- the exact Python fixtures --------------------------------------------
[[nodiscard]] Value legacy_python() {
    return Value{Object{{"backend", text("python")}, {"runtime_schema", integer(1)},
        {"python_source_sha256", text(std::string(64, 'a'))}, {"native_source_sha256", Value{}},
        {"extension_sha256", Value{}}, {"mujoco_version", text("3.12.0")},
        {"mujoco_library_sha256", Value{}}, {"build_context", Value{}}}};
}
[[nodiscard]] Value platform_darwin() {
    return Value{Object{{"system", text("Darwin")}, {"machine", text("aarch64")},
        {"libc", Value{Object{{"name", text("libSystem")}, {"version", text("26A434")}}}},
        {"os_build", text("26A434")}, {"python_version", text("3.13.16")},
        {"python_soabi", text("cpython-313-darwin")}}};
}
[[nodiscard]] Value platform_linux() {
    return Value{Object{{"system", text("Linux")}, {"machine", text("aarch64")},
        {"libc", Value{Object{{"name", text("glibc")}, {"version", text("2.39")}}}},
        {"os_build", text("7.0.14-linuxkit")}, {"python_version", text("3.13.16")},
        {"python_soabi", text("cpython-313-aarch64-linux-gnu")}}};
}
[[nodiscard]] Value accelerate_library() {
    return Value{Object{{"role", text("blas_lapack")},
        {"identity_kind", text("apple-system")}, {"sha256", Value{}},
        {"build_id", text("macos-26A434+accelerate-a697e7b9-8ff4-356c-a507-64b38af1ccaa+sdk-26A425")}}};
}
[[nodiscard]] Value openblas_library() {
    return Value{Object{{"role", text("blas_lapack")},
        {"identity_kind", text("file")}, {"sha256", text(std::string(64, '5'))},
        {"build_id", Value{}}}};
}
[[nodiscard]] Value numerics_accelerate() {
    return Value{Object{{"backend", text("ACCELERATE")}, {"numpy_version", text("2.5.2")},
        {"blas_integer_bits", integer(32)}, {"lapack_integer_bits", integer(64)},
        {"cpu_profile", text("Apple M4 Max")},
        {"threading", Value{Object{{"implementation", text("accelerate")}, {"num_threads", integer(4)}}}},
        {"libraries", Value{Array{accelerate_library()}}}}};
}
[[nodiscard]] Value numerics_openblas() {
    return Value{Object{{"backend", text("NUMPY_OPENBLAS")}, {"numpy_version", text("2.5.2")},
        {"blas_integer_bits", integer(64)}, {"lapack_integer_bits", integer(64)},
        {"cpu_profile", text("neoversen1")},
        {"threading", Value{Object{{"implementation", text("pthreads")}, {"num_threads", integer(16)}}}},
        {"libraries", Value{Array{openblas_library()}}}}};
}
[[nodiscard]] Value legacy_native() {
    auto value = legacy_python();
    set(value, "backend", text("native"));
    set(value, "native_source_sha256", text(std::string(64, 'b')));
    set(value, "extension_sha256", text(std::string(64, 'c')));
    set(value, "mujoco_library_sha256", text(std::string(64, 'd')));
    set(value, "build_context", Value{Object{{"build_type", text("Release")},
        {"compiler_id", text("AppleClang")}, {"compiler_version", text("21.0.0")},
        {"numerical_flags", text("-O3 -ffp-contract=off")}}});
    return value;
}
[[nodiscard]] Value v2_python() {
    auto value = legacy_python();
    set(value, "identity_schema", integer(2));
    set(value, "build_context",
        Value{Object{{"platform", platform_darwin()}, {"numerics", numerics_accelerate()}}});
    return value;
}
[[nodiscard]] Value v2_native() {
    auto value = legacy_native();
    set(value, "identity_schema", integer(2));
    auto context = field(value, "build_context");
    set(context, "platform", platform_darwin());
    set(context, "numerics", numerics_accelerate());
    set(value, "build_context", std::move(context));
    return value;
}
[[nodiscard]] Value v2_native_linux() {
    auto value = v2_native();
    auto context = field(value, "build_context");
    set(context, "platform", platform_linux());
    set(context, "numerics", numerics_openblas());
    set(context, "compiler_id", text("Clang"));
    set(value, "build_context", std::move(context));
    return value;
}

// ---- path mutation mirroring Python nested-subscript writes ----------------
[[nodiscard]] Value& member(Value& object, const std::string& key) {
    for (auto& [name, child] : std::get<Object>(object.data))
        if (name == key) return child;
    throw std::invalid_argument("test fixture member path absent: " + key);
}
// node[path[-1]] = bad in the Python source: an upsert on the last member,
// an in-place write on the last index.
void set_at(Value& root, const Path& path, Value bad) {
    Value* node = &root;
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
        const auto& part = path[index];
        node = std::holds_alternative<std::string>(part)
            ? &member(*node, std::get<std::string>(part))
            : &std::get<Array>(node->data).at(std::get<std::size_t>(part));
    }
    const auto& last = path.back();
    if (std::holds_alternative<std::string>(last))
        set(*node, std::get<std::string>(last), std::move(bad));
    else
        std::get<Array>(node->data).at(std::get<std::size_t>(last)) = std::move(bad);
}
void erase_member(Value& object, const std::string& key) {
    auto& members = std::get<Object>(object.data);
    for (auto it = members.begin(); it != members.end(); ++it)
        if (it->first == key) { members.erase(it); return; }
    throw std::invalid_argument("test fixture member path absent: " + key);
}

// ---- the live native-executable profile under test -------------------------
[[nodiscard]] Value live_execution() {
    const auto current = ::runtime::runtime_identity();
    const auto provider = ::engine::identity();
    return Value{Object{{"backend", text("native")},
        {"native_source_sha256", text(current.native_source_sha256)},
        {"execution_identity", text(bike::platform::execution_identity())},
        {"mujoco_library_sha256", text(provider.library_sha256)},
        {"numeric_provider", text(provider.numeric_provider)},
        {"executable_sha256", text(bike::platform::build_identity().executable_sha256)},
        {"source_identity_scope",
            text("compiled first-party source manifest; running image hashed separately")}}};
}
// A ReplayInput whose recorded execution equals this runtime: check_execution
// is the native require_execution_compatible.
[[nodiscard]] replay::ReplayInput compatible_input() {
    replay::ReplayInput input;
    input.replay_contract_version = 2;
    input.manifest = Value{Object{{"execution", live_execution()}}};
    input.summary = Value{Object{
        {"model_source_sha256", text(::runtime::runtime_identity().native_source_sha256)},
        {"research", Value{Object{}}}}};
    return input;
}
// check_execution names the first mismatching key — the Python "names the
// dotted field" contract under the native fail-fast verdict.
void gate_names(const Value& execution, std::string_view needle) {
    auto input = compatible_input();
    set(input.manifest, "execution", execution);
    bool rejected = false;
    std::string message;
    try { replay::check_execution(input); }
    catch (const replay::IncompatibleExecution& failure) {
        rejected = true; message = failure.what(); }
    bt::require(rejected, "the execution gate refuses the foreign identity");
    bt::require(message.find("incompatible-execution") != std::string::npos,
        "the verdict carries the incompatible-execution marker: " + message);
    bt::require(message.find(needle) != std::string::npos,
        "the verdict names the mismatched field: " + std::string(needle) +
        " (got: " + message + ")");
}

// ---- schema acceptance: legacy and v2 --------------------------------------
// test_legacy_read_stays_valid / test_legacy_native_read_stays_valid /
// test_v2_records_validate — all three profiles validate and the validator
// returns without rewriting the record.
void reads_stay_valid() {
    replay::validate_execution(legacy_python());
    replay::validate_execution(legacy_native());
    for (const auto make : {v2_python, v2_native, v2_native_linux})
        replay::validate_execution(make());
}
// test_validation_does_not_mutate_the_record — canonical JSON is identical
// across a validation pass for every record shape.
void validation_does_not_mutate() {
    for (const auto make : {legacy_python, legacy_native, v2_python, v2_native}) {
        const auto value = make();
        const auto before = canonical_json(value);
        replay::validate_execution(value);
        bt::require(canonical_json(value) == before,
            "validation leaves the record byte-identical");
    }
}

// ---- identity-schema dispatch is exact --------------------------------------
void schema_dispatch() {
    // test_unknown_identity_schema_is_not_guessed
    {
        auto value = legacy_python();
        set(value, "identity_schema", integer(999));
        bool rejected = false; std::string message;
        try { replay::validate_execution(value); }
        catch (const std::invalid_argument& failure) { rejected = true; message = failure.what(); }
        bt::require(rejected && message.find("identity_schema") != std::string::npos,
            "an unknown identity_schema is refused by name: " + message);
    }
    // test_noncanonical_identity_schema_fails — every noncanonical tag fails
    // closed; integer tags name identity_schema verbatim, wrong-type tags are
    // refused by the typed extractor (the Python match="identity_schema" is
    // its own validator's wording).
    const std::vector<std::pair<Value, bool>> variants{
        {Value{true}, false}, {Value{false}, false}, {integer(1), true},
        {integer(3), true}, {text("2"), false}, {real(2.0), false}, {Value{}, false}};
    for (const auto& [schema, names_field] : variants) {
        auto value = v2_python();
        set(value, "identity_schema", schema);
        bool rejected = false; std::string message;
        try { replay::validate_execution(value); }
        catch (const std::invalid_argument& failure) { rejected = true; message = failure.what(); }
        bt::require(rejected, "a noncanonical identity_schema fails closed");
        if (names_field)
            bt::require(message.find("identity_schema") != std::string::npos,
                "an out-of-range integer schema names identity_schema: " + message);
    }
}
// test_v2_rejects_missing_and_extra_top_level_keys — deleting any of the nine
// keys, or adding an untracked one, fails closed for both v2 backends.
void v2_top_level_keys() {
    for (const auto make : {v2_python, v2_native}) {
        const Value value = make();
        for (const auto& [name, child] : std::get<Object>(value.data)) {
            (void)child;
            auto bad = value;
            erase_member(bad, name);
            bt::invalid([&] { replay::validate_execution(bad); });
        }
        auto bad = value;
        set(bad, "extra", text("untracked"));
        bt::invalid([&] { replay::validate_execution(bad); });
    }
}
// test_legacy_rejects_the_v2_field_set — the legacy record carries exactly
// its eight keys.
void legacy_rejects_v2_fields() {
    auto value = legacy_python();
    set(value, "extra", integer(1));
    bt::invalid([&] { replay::validate_execution(value); });
}
// test_native_context_requires_exactly_six_keys — the identity-schema-2
// native context is build_type/compiler_id/compiler_version/numerical_flags/
// platform/numerics and nothing else; the message names build_context.
void native_context_exact_keys() {
    for (const char* key : {"build_type", "compiler_id", "compiler_version",
                           "numerical_flags", "platform", "numerics"}) {
        for (const bool remove : {true, false}) {
            auto value = v2_native();
            auto context = field(value, "build_context");
            if (remove) erase_member(context, key);
            else set(context, "injected", text("untracked"));
            set(value, "build_context", std::move(context));
            bool rejected = false; std::string message;
            try { replay::validate_execution(value); }
            catch (const std::invalid_argument& failure) { rejected = true; message = failure.what(); }
            bt::require(rejected && message.find("build_context") != std::string::npos,
                "a malformed native build_context names build_context: " + message);
        }
    }
}
// test_python_v2_context_is_only_platform_and_numerics — a compiler key under
// the python backend refuses by name.
void python_context_only_platform_numerics() {
    auto value = v2_python();
    auto context = field(value, "build_context");
    set(context, "build_type", text("Release"));
    set(value, "build_context", std::move(context));
    bool rejected = false; std::string message;
    try { replay::validate_execution(value); }
    catch (const std::invalid_argument& failure) { rejected = true; message = failure.what(); }
    bt::require(rejected && message.find("build_context") != std::string::npos,
        "a foreign key under the python context names build_context: " + message);
}

// ---- field-level rules fail closed ------------------------------------------
// test_field_type_and_null_rules — the same strictness the probe applies to
// its own record.
void field_type_and_null_rules() {
    struct Case {
      bool native{};
      const char *key{};
      Value bad;
    };
    const Case cases[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {.native = false, .key = "runtime_schema", .bad = Value{true}},
        {.native = false, .key = "runtime_schema", .bad = integer(2)},
        {.native = true,
         .key = "python_source_sha256",
         .bad = text("not-a-digest")},
        {.native = false,
         .key = "extension_sha256",
         .bad = text(std::string(64, '0'))},
        {.native = false,
         .key = "native_source_sha256",
         .bad = text(std::string(64, '0'))},
        {.native = false,
         .key = "mujoco_library_sha256",
         .bad = text(std::string(64, '0'))},
        {.native = true, .key = "extension_sha256", .bad = Value{}},
        {.native = true, .key = "mujoco_version", .bad = text("")},
    };
    for (const auto& entry : cases) {
        auto value = entry.native ? v2_native() : v2_python();
        set(value, entry.key, entry.bad);
        bt::invalid([&] { replay::validate_execution(value); });
    }
}
// One mutation case per call keeps each frame under the 8 KiB warning cap.
void expect_nested_invalid(Value (*make)(), const Path& path, const Value& bad) {
    auto value = make();
    set_at(member(value, "build_context"), path, bad);
    bt::invalid([&] { replay::validate_execution(value); });
}
// test_nested_field_rules_fail_closed — every platform/numerics mutation of
// the ACCELERATE record fails validation.
void nested_field_rules_fail_closed() {
    // emplace_back rather than a braced table: an initializer_list backing
    // array of 27 Path/Value pairs exceeds the 8 KiB frame cap.
    static const std::vector<std::pair<Path, Value>> cases = [] {
        std::vector<std::pair<Path, Value>> out;
        out.reserve(27);
        out.emplace_back(Path{{"platform", "system"}}, text("FreeBSD"));
        out.emplace_back(Path{{"platform", "machine"}}, text(""));
        out.emplace_back(Path{{"platform", "machine"}}, text("ARM 64"));
        out.emplace_back(Path{{"platform", "libc"}},
            Value{Object{{"name", text("glibc")}}});
        out.emplace_back(Path{{"platform", "libc"}},
            Value{Object{{"name", text("glibc")}, {"version", text("2.39")},
                {"x", integer(1)}}});
        out.emplace_back(Path{{"platform", "libc", "name"}}, text(""));
        out.emplace_back(Path{{"platform", "os_build"}}, text("   "));
        out.emplace_back(Path{{"platform", "python_version"}}, text(""));
        out.emplace_back(Path{{"platform", "python_soabi"}}, Value{});
        out.emplace_back(Path{{"platform", "extra"}}, text("untracked"));
        out.emplace_back(Path{{"numerics", "extra"}}, text("untracked"));
        out.emplace_back(Path{{"numerics", "backend"}}, text("MKL"));
        out.emplace_back(Path{{"numerics", "numpy_version"}}, text(""));
        out.emplace_back(Path{{"numerics", "blas_integer_bits"}}, Value{true});
        out.emplace_back(Path{{"numerics", "blas_integer_bits"}}, integer(64));
        out.emplace_back(Path{{"numerics", "lapack_integer_bits"}}, integer(32));
        out.emplace_back(Path{{"numerics", "cpu_profile"}}, text(""));
        out.emplace_back(Path{{"numerics", "threading"}},
            Value{Object{{"implementation", text("accelerate")}}});
        out.emplace_back(Path{{"numerics", "threading", "num_threads"}},
            integer(0));
        out.emplace_back(Path{{"numerics", "threading", "num_threads"}},
            Value{true});
        out.emplace_back(Path{{"numerics", "libraries"}}, Value{Array{}});
        out.emplace_back(Path{{"numerics", "libraries", std::size_t{0}, "role"}},
            text("blas"));
        out.emplace_back(
            Path{{"numerics", "libraries", std::size_t{0}, "identity_kind"}},
            text("file"));
        out.emplace_back(Path{{"numerics", "libraries", std::size_t{0}, "sha256"}},
            text(std::string(64, 'f')));
        out.emplace_back(
            Path{{"numerics", "libraries", std::size_t{0}, "build_id"}},
            Value{});
        out.emplace_back(
            Path{{"numerics", "libraries", std::size_t{0}, "build_id"}},
            text(""));
        out.emplace_back(
            Path{{"numerics", "libraries", std::size_t{0}, "extra"}},
            text("untracked"));
        return out;
    }();
    for (const auto& [path, bad] : cases) {
        expect_nested_invalid(v2_native, path, bad);
    }
}
// test_linux_file_identity_rules — the NUMPY_OPENBLAS file identity pins a
// real digest, a matching kind, and an absent build_id.
void linux_file_identity_rules() {
    const std::vector<std::pair<Path, Value>> cases{
        {Path{{"numerics", "blas_integer_bits"}}, integer(32)},
        {Path{{"numerics", "libraries", std::size_t{0}, "identity_kind"}},
            text("apple-system")},
        {Path{{"numerics", "libraries", std::size_t{0}, "sha256"}}, Value{}},
        {Path{{"numerics", "libraries", std::size_t{0}, "sha256"}},
            text("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF")},
        {Path{{"numerics", "libraries", std::size_t{0}, "build_id"}},
            text("fabricated")},
    };
    for (const auto& [path, bad] : cases) {
        expect_nested_invalid(v2_native_linux, path, bad);
    }
}

// ---- diagnostics name the exact field ---------------------------------------
// test_integer_abi_change_is_a_named_failure — integer widths are pinned per
// qualified backend; any mutation makes the record unqualified, so the
// diagnostic is the validation error itself.
void integer_abi_named_failure() {
    const std::vector<std::pair<Value (*)(), const char*>> cases{
        {v2_native, "blas_integer_bits"},
        {v2_native, "lapack_integer_bits"},
        {v2_native_linux, "blas_integer_bits"}};
    for (const auto& [make, key] : cases) {
        auto value = make();
        auto context = field(value, "build_context");
        auto numerics = field(context, "numerics");
        set(numerics, key, integer(128));
        set(context, "numerics", std::move(numerics));
        set(value, "build_context", std::move(context));
        bool rejected = false; std::string message;
        try { replay::validate_execution(value); }
        catch (const std::invalid_argument& failure) { rejected = true; message = failure.what(); }
        bt::require(rejected && message.find("integer ABI") != std::string::npos,
            "an unqualified integer width names the ABI rule: " + message);
    }
}
// test_os_change_is_a_named_failure — the qualified pairings are
// Darwin+ACCELERATE and Linux+NUMPY_OPENBLAS; a bare OS mutation leaves an
// inconsistent record whose error names platform.system.
void os_change_named_failure() {
    for (const auto make : {v2_native, v2_native_linux}) {
        auto value = make();
        auto context = field(value, "build_context");
        auto platform = field(context, "platform");
        // flip Darwin<->Linux, leaving the numerics backend unpaired
        set(platform, "system", make == v2_native ? text("Linux") : text("Darwin"));
        set(context, "platform", std::move(platform));
        set(value, "build_context", std::move(context));
        bool rejected = false; std::string message;
        try { replay::validate_execution(value); }
        catch (const std::invalid_argument& failure) { rejected = true; message = failure.what(); }
        bt::require(rejected && message.find("platform.system") != std::string::npos,
            "an unqualified OS pairing names platform.system: " + message);
    }
}

// ---- the native-executable gate (require_execution_compatible twin) ---------
// test_require_execution_compatible_accepts_identity — the live profile
// passes the gate unchanged.
void compatible_accepts_identity() {
    replay::check_execution(compatible_input());
}
// test_require_execution_compatible_raises_with_named_fields — a single
// mismatched field is named; mutating two fields reports the first in the
// gate's fixed order (native_source_sha256 → execution_identity →
// mujoco_library_sha256 → numeric_provider): fail-fast, not report-all.
void incompatible_raises_named_fields() {
    {
        auto execution = live_execution();
        set(execution, "mujoco_library_sha256", text(std::string(64, 'e')));
        set(execution, "numeric_provider", text("OTHER_PROVIDER"));
        gate_names(execution, "mujoco_library_sha256");
    }
    // test_mismatch_names_the_field — the provider hash is the named key.
    {
        auto execution = live_execution();
        set(execution, "mujoco_library_sha256", text(std::string(64, 'e')));
        gate_names(execution, "mujoco_library_sha256");
    }
    // test_provider_hash_names_the_indexed_library_path — the recorded
    // provider digest is the single compared hash under the native profile.
    {
        auto execution = live_execution();
        set(execution, "native_source_sha256", text(std::string(64, '9')));
        gate_names(execution, "native_source_sha256");
    }
}
// test_single_field_mismatch_reports_the_dotted_key — each compared field of
// the native-executable profile is named when foreign. The historical
// platform/numerics paths of the Python parameters are covered by the
// validation mutations above; the gate compares the four live digests plus
// the recorded plant source and solver qualification.
void single_field_mismatch_names_it() {
    {
        auto execution = live_execution();
        set(execution, "execution_identity", text(std::string(64, '0')));
        gate_names(execution, "execution_identity");
    }
    {
        auto execution = live_execution();
        set(execution, "numeric_provider", text("OTHER_PROVIDER"));
        gate_names(execution, "numeric_provider");
    }
    {
        // summary.model_source_sha256 is the recorded plant source digest.
        auto input = compatible_input();
        set(input.summary, "model_source_sha256", text(std::string(64, '9')));
        bool rejected = false; std::string message;
        try { replay::check_execution(input); }
        catch (const replay::IncompatibleExecution& failure) { rejected = true; message = failure.what(); }
        bt::require(rejected && message.find("plant source") != std::string::npos,
            "a foreign recorded plant source is named: " + message);
    }
    {
        // A foreign run-flagged identity receipt is named before any compare.
        auto input = compatible_input();
        auto execution = field(input.manifest, "execution");
        set(execution, "allocation_solver", text("other_solver"));
        set(execution, "allocation_solver_qualification", text("pending"));
        set(input.manifest, "execution", std::move(execution));
        bool rejected = false; std::string message;
        try { replay::check_execution(input); }
        catch (const replay::IncompatibleExecution& failure) { rejected = true; message = failure.what(); }
        bt::require(rejected && message.find("allocation solver") != std::string::npos,
            "an unqualified recorded solver is named: " + message);
    }
}
// test_require_execution_compatible_rejects_legacy_recorded /
// test_legacy_vs_v2_reports_the_missing_identity_fields — a historical
// platform/numerics record has no certified native-executable fixture; the
// gate's verdict names the missing profile wholesale.
void historical_profile_refused() {
    for (const auto make : {legacy_python, v2_python, v2_native_linux}) {
        auto input = compatible_input();
        set(input.manifest, "execution", make());
        bool rejected = false; std::string message;
        try { replay::check_execution(input); }
        catch (const replay::IncompatibleExecution& failure) { rejected = true; message = failure.what(); }
        bt::require(rejected && message.find("incompatible-execution") != std::string::npos &&
            message.find("historical") != std::string::npos,
            "a historical profile is refused by name: " + message);
    }
}
// test_differences_validate_both_sides — a malformed recorded profile still
// fails closed: a record without the native-executable axis gets the profile
// verdict; a record carrying the axis but malformed digests is refused by
// the recorded-side validate_execution before any comparison.
void gate_validates_the_recorded_side() {
    {
        auto input = compatible_input();
        set(input.manifest, "execution", Value{Object{{"backend", text("python")}}});
        bool rejected = false;
        try { replay::check_execution(input); }
        catch (const replay::IncompatibleExecution&) { rejected = true; }
        bt::require(rejected, "a partial recorded execution is refused");
    }
    {
        auto input = compatible_input();
        auto execution = live_execution();
        set(execution, "native_source_sha256", text("not-a-digest"));
        set(input.manifest, "execution", std::move(execution));
        bool rejected = false;
        try { replay::check_execution(input); }
        catch (const rec::RecordingError&) { rejected = true; }
        bt::require(rejected, "a malformed recorded execution is refused by validation");
    }
}
// test_os_and_backend_pairing_change_reports_the_fields — a consistent
// foreign-platform record stays readable; under the native gate the recorded
// axis is the single execution_identity digest, and a foreign value names it.
void foreign_pairing_stays_readable() {
    replay::validate_execution(v2_native_linux());
    auto execution = live_execution();
    set(execution, "execution_identity", text(std::string(64, '7')));
    gate_names(execution, "execution_identity");
}
} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        reads_stay_valid();
        validation_does_not_mutate();
        schema_dispatch();
        v2_top_level_keys();
        legacy_rejects_v2_fields();
        native_context_exact_keys();
        python_context_only_platform_numerics();
        field_type_and_null_rules();
        nested_field_rules_fail_closed();
        linux_file_identity_rules();
        integer_abi_named_failure();
        os_change_named_failure();
        compatible_accepts_identity();
        incompatible_raises_named_fields();
        single_field_mismatch_names_it();
        historical_profile_refused();
        gate_validates_the_recorded_side();
        foreign_pairing_stays_readable();
    });
}
