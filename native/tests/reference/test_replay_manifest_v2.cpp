// Port of tests/reference/test_replay_manifest_v2.py — the data-only
// schema-1/2 recording reader contract. Synthetic bundles only; no plant,
// model or runtime is constructed by load_legacy_input (the data-only
// reader never loads an extension — there is none to load natively).
#include "replay_reference_support.hpp"
#include "runtime/provenance.hpp"
#include <algorithm>
#include <print>
#include <set>

namespace {
namespace rr = replay_reference;
using bike::io::Value;
using rr::field;
using rr::integer;
using rr::set;
using rr::text;

// The eight legacy EXECUTION_FIELDS (artifact.EXECUTION_FIELDS).
const std::set<std::string> EXECUTION_FIELDS{"backend", "runtime_schema", // NOLINT(bugprone-throwing-static-initialization,cert-err58-cpp) static fixture table built once at startup; a throw is a fatal defect
    "python_source_sha256", "native_source_sha256", "extension_sha256",
    "mujoco_version", "mujoco_library_sha256", "build_context"};

Value manifest_of(const std::filesystem::path& path) {
    return bike::io::parse_json(rr::read_file(path / "replay.json"));
}
void write_manifest(const std::filesystem::path& path, const Value& manifest) {
    rr::write_file(path / "replay.json", bike::io::recording_json(manifest) + "\n");
}
void append(const std::filesystem::path& path, std::string_view bytes) {
    std::ofstream out(path, std::ios::binary | std::ios::app);
    bt::require(static_cast<bool>(out), "fixture file opens for append");
    out << bytes;
    out.close();
}
std::set<std::string> key_set(const Value& value) {
    std::set<std::string> keys;
    for (const auto& [name, ignored] : bike::io::as_object(value, "record")) {
        (void)ignored; keys.insert(name);
    }
    return keys;
}

// test_fixed_schema_compatibility_without_loading_extension —
// version∈{1,2}, backend∈{python,native}, versioned∈{false,true}: the
// data-only reader loads the bundle; the recorded policy reference stays a
// string (no import machinery exists natively to invoke).
void schema_compat_one(int version, std::string_view backend, bool versioned) {
  rr::Directory const tmp;
  const auto path =
      rr::synthetic_recording(tmp.path, version, backend, true, versioned);
  const auto input = bike::replay::load_legacy_input(path);
  bt::require(input.replay_contract_version == version,
              "manifest schema round-trips");
  const auto reference = rr::text_of(
      field(field(field(input.summary, "research"), "run_metadata"), "policy"),
      "reference");
  bt::require(reference.starts_with("malicious_"),
              "recorded policy references stay inert text through the "
              "data-only reader");
}
void schema_compat() {
    schema_compat_one(1, "python", false);
    schema_compat_one(2, "python", false);
    schema_compat_one(2, "native", false);
    schema_compat_one(2, "python", true);
    schema_compat_one(2, "native", true);
}

// test_every_schema2_file_is_authenticated — each authenticated file,
// including the two bundles, is digest-bound to the manifest.
void every_file_authenticated() {
    std::vector<std::string> all(rr::SCHEMA1_FILES.begin(), rr::SCHEMA1_FILES.end());
    all.insert(all.end(), rr::SCHEMA2_EXTRA.begin(), rr::SCHEMA2_EXTRA.end());
    all.emplace_back(rr::ENVELOPE_FILE);
    all.emplace_back(rr::STRENGTH_FILE);
    for (const auto& name : all) {
      rr::Directory const tmp;
      const auto path = rr::synthetic_recording(tmp.path);
      append(path / name, "tamper");
      rr::rejects([&] { (void)bike::replay::load_legacy_input(path); },
                  "checksum");
    }
}

// test_manifest_rejects_unlisted_fields_and_paths — six mutations.
void manifest_mutation(std::string_view mutation) {
  rr::Directory const tmp;
  const auto path = rr::synthetic_recording(tmp.path);
  auto manifest = manifest_of(path);
  if (mutation == "missing") {
    auto hashes = field(manifest, "file_sha256");
    auto &object = std::get<Value::Object>(hashes.data);
    object.erase(
        std::ranges::remove_if(
            object, [](const auto &pair) { return pair.first == "trace.csv"; })
            .begin(),
        object.end());
    set(manifest, "file_sha256", std::move(hashes));
  } else if (mutation == "extra") {
    auto hashes = field(manifest, "file_sha256");
    set(hashes, "other.json",
        text("000000000000000000000000000000000000000000000000000000000000000"
             "0"));
    set(manifest, "file_sha256", std::move(hashes));
  } else if (mutation == "traversal") {
    set(manifest, "joint_strength_file", text("../rider_joint_strength.json"));
  } else if (mutation == "upper_hash") {
    auto hashes = field(manifest, "file_sha256");
    set(hashes, "trace.csv",
        text("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
             "A"));
    set(manifest, "file_sha256", std::move(hashes));
  } else if (mutation == "unknown_top") {
    set(manifest, "policy_module", text("untrusted"));
  } else {
    set(manifest, "schema_version", Value{true});
  }
    write_manifest(path, manifest);
    rr::rejects([&] { (void)bike::replay::load_legacy_input(path); });
}
void manifest_mutations() {
    for (auto mutation : {"missing", "extra", "traversal", "upper_hash", "unknown_top", "bool_schema"})
        manifest_mutation(mutation);
}

// test_schema1_does_not_silently_accept_schema2_checksum_set.
void schema1_checksum_set() {
  rr::Directory const tmp;
  const auto path = rr::synthetic_recording(tmp.path, 1, "python");
  auto manifest = manifest_of(path);
  auto hashes = field(manifest, "file_sha256");
  set(hashes, "telemetry.csv",
      text("0000000000000000000000000000000000000000000000000000000000000000"));
  set(manifest, "file_sha256", std::move(hashes));
  write_manifest(path, manifest);
  rr::rejects([&] { (void)bike::replay::load_legacy_input(path); }, "file set");
}

// test_summary_identity_and_referenced_bundles_are_cross_checked.
void bundle_cross_check(std::string_view mutation) {
  rr::Directory const tmp;
  const auto path = rr::synthetic_recording(tmp.path);
  auto summary = bike::io::parse_json(rr::read_file(path / "summary.json"));
  if (mutation == "missing_bundle") {
    auto resolved = field(summary, "resolved_config");
    auto physics = field(resolved, "physics");
    auto articulated = field(physics, "articulated");
    set(articulated, "joint_strength_path", Value{});
    set(physics, "articulated", std::move(articulated));
    set(resolved, "physics", std::move(physics));
    set(summary, "resolved_config", std::move(resolved));
  } else if (mutation == "bundle_digest") {
    auto resolved = field(summary, "resolved_config");
    set(resolved, "joint_strength_sha256",
        text("000000000000000000000000000000000000000000000000000000000000000"
             "0"));
    set(summary, "resolved_config", std::move(resolved));
  } else if (mutation == "execution") {
    auto research = field(summary, "research");
    auto execution = field(research, "execution");
    set(execution, "extension_sha256",
        text("000000000000000000000000000000000000000000000000000000000000000"
             "0"));
    set(research, "execution", std::move(execution));
    set(summary, "research", std::move(research));
  } else {
    auto resolved = field(summary, "resolved_config");
    set(resolved, "physics", Value{Value::Array{}});
    set(summary, "resolved_config", std::move(resolved));
  }
    rr::write_json(path / "summary.json", summary);
    rr::reseal(path, "summary.json");
    rr::rejects([&] { (void)bike::replay::load_legacy_input(path); });
}
void bundle_cross_checks() {
    for (auto mutation : {"missing_bundle", "bundle_digest", "execution", "malformed_physics"})
        bundle_cross_check(mutation);
}

// test_symlinked_recorded_file_is_rejected — a leaf symlink fails the
// no-follow regular-file open before its bytes are ever hashed.
void symlink_rejected() {
  rr::Directory const tmp;
  const auto path = rr::synthetic_recording(tmp.path);
  const auto outside = tmp.path / "outside-trace.csv";
  rr::write_file(outside, rr::read_file(path / "trace.csv"));
  std::filesystem::remove(path / "trace.csv");
  std::filesystem::create_symlink(outside, path / "trace.csv");
  rr::rejects([&] { (void)bike::replay::load_legacy_input(path); });
}

// test_duplicate_and_nonfinite_json_fields_are_rejected — strict parser:
// duplicate keys, NaN and out-of-range literals never reach the schema layer.
void strict_json() {
  for (const char *payload :
       {R"({"schema_version":2,"schema_version":1})",
        "{\"schema_version\":NaN}", "{\"schema_version\":1e999}"}) {
    rr::Directory const tmp;
    const auto path = rr::synthetic_recording(tmp.path);
    rr::write_file(path / "replay.json", payload);
    rr::rejects([&] { (void)bike::replay::load_legacy_input(path); });
  }
}

// test_execution_exact_shape_and_nullability — the legacy 8-field record.
void execution_shape(std::string_view backend) {
    const auto expected = rr::identity(backend);
    bt::require(key_set(expected) == EXECUTION_FIELDS, "the fixture spans exactly the legacy fields");
    bike::replay::validate_execution(expected);
    for (const auto& key : EXECUTION_FIELDS) {
        auto bad = expected;
        auto& object = std::get<Value::Object>(bad.data);
        object.erase(
            std::ranges::remove_if(
                object, [&](const auto &pair) { return pair.first == key; })
                .begin(),
            object.end());
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
    const std::pair<std::string, Value> mutations[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {"runtime_schema", Value{true}}, {"backend", text("auto")},
        {"python_source_sha256", text("not a digest")}, {"mujoco_version", text("")}};
    for (const auto& [key, value] : mutations) {
        auto bad = expected; set(bad, key, value);
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
    auto bad = expected;
    if (backend == "native") {
        auto context = field(bad, "build_context");
        set(context, "injected", text("untracked"));
        set(bad, "build_context", std::move(context));
    } else {
        set(bad, "extension_sha256",
            text("0000000000000000000000000000000000000000000000000000000000000000"));
    }
    rr::rejects([&] { bike::replay::validate_execution(bad); });
}

// test_execution_v2_shape_and_nullability — identity_schema=2 keeps every
// legacy rule and adds the measured platform/numerics context.
void execution_v2(std::string_view backend) {
    const auto expected = rr::identity(backend, true);
    auto expected_keys = EXECUTION_FIELDS; expected_keys.insert("identity_schema");
    bt::require(key_set(expected) == expected_keys,
        "the versioned fixture adds exactly identity_schema");
    bike::replay::validate_execution(expected);
    for (const auto& key : expected_keys) {
        auto bad = expected;
        auto& object = std::get<Value::Object>(bad.data);
        object.erase(
            std::ranges::remove_if(
                object, [&](const auto &pair) { return pair.first == key; })
                .begin(),
            object.end());
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
    const std::pair<std::string, Value> mutations[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {"identity_schema", Value{true}}, {"identity_schema", integer(3)},
        {"runtime_schema", Value{true}}, {"backend", text("auto")},
        {"mujoco_version", text("")}};
    for (const auto& [key, value] : mutations) {
        auto bad = expected; set(bad, key, value);
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
    {
        auto bad = expected;
        auto context = field(bad, "build_context");
        auto numerics = field(context, "numerics");
        auto threading = field(numerics, "threading");
        set(threading, "num_threads", integer(0));
        set(numerics, "threading", std::move(threading));
        set(context, "numerics", std::move(numerics));
        set(bad, "build_context", std::move(context));
        rr::rejects([&] { bike::replay::validate_execution(bad); });
    }
}

// test_native_source_manifest_has_stable_paths_and_tracks_headers — the
// name=sha256 manifest algorithm on a synthetic tree, plus the compiled
// runtime identity matching a recompute over the live native/ tree.
std::string fingerprint(const std::filesystem::path& root) {
    // Same per-directory allowlists as native/CMakeLists.txt and
    // artifact.native_source_fingerprint: src tracks code files; cmake
    // tracks build scripts (a stray cmake/*.cpp is a probe, not a source).
    std::vector<std::filesystem::path> inputs{root / "CMakeLists.txt"};
    const std::pair<std::string_view, std::set<std::string>> scopes[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {"src", {".cpp", ".hpp", ".h", ".in"}}, {"cmake", {".cmake", ".in"}}};
    for (const auto& [directory, extensions] : scopes)
        for (const auto& entry : std::filesystem::recursive_directory_iterator(root / directory)) {
            if (!entry.is_regular_file()) continue;
            if (extensions.contains(entry.path().extension().string()))
                inputs.push_back(entry.path());
        }
    std::ranges::sort(inputs, [&](const auto& a, const auto& b) {
        return a.lexically_relative(root).generic_string() < b.lexically_relative(root).generic_string(); });
    std::string manifest;
    for (const auto& file : inputs)
        manifest += file.lexically_relative(root).generic_string() + "=" +
            bike::platform::file_sha256(file) + "\n";
    return bike::io::sha256(std::as_bytes(std::span(manifest.data(), manifest.size())));
}
void source_manifest() {
  rr::Directory const tmp;
  const auto root = tmp.path / "native";
  std::filesystem::create_directories(root / "src");
  std::filesystem::create_directories(root / "cmake");
  const std::pair<std::string, std::string> files[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
      {"CMakeLists.txt", "project(reference)\n"},
      {"src/core.cpp", "int value = 1;\n"},
      {"src/core.hpp", "#pragma once\n"},
      {"cmake/config.cmake", "set(VALUE 1)\n"}};
  for (const auto &[name, payload] : files)
    rr::write_file(root / name, payload);
  const auto expected = fingerprint(root);
  // Build outputs are not source inputs (the Python fixture drops a binary
  // under build/); the real manifest tree scopes to src/+cmake/+root list.
  std::filesystem::create_directories(root / "build");
  rr::write_file(root / "build/bike_native.so", "not a source input");
  bt::require(fingerprint(root) == expected,
              "build trees never enter the source manifest");
  rr::write_file(root / "src/core.hpp", "#pragma once\n// changed\n");
  bt::require(fingerprint(root) != expected,
              "header content changes are tracked");
  // The compiled build identity equals the same algorithm over the real tree.
  const auto compiled =
      std::string(runtime::runtime_identity().native_source_sha256);
  const auto recomputed = fingerprint(rr::repo_root() / "native");
  if (compiled != recomputed) {
    std::println(stderr, "compiled={} recomputed={} root={}", compiled,
                 recomputed, (rr::repo_root() / "native").string());
  }
    bt::require(compiled == recomputed,
        "the compiled native_source_sha256 is this manifest over live sources");
}
} // namespace
int main() {
    return bt::run([] {
        schema_compat();
        every_file_authenticated();
        manifest_mutations();
        schema1_checksum_set();
        bundle_cross_checks();
        symlink_rejected();
        strict_json();
        execution_shape("python");
        execution_shape("native");
        execution_v2("python");
        execution_v2("native");
        source_manifest();
    });
}
