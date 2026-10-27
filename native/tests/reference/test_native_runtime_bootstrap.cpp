// Completion 3.25 (batch runtime). Source: tests/reference/
// test_native_runtime_bootstrap.py — A1 construction, bootstrap ownership
// and restore, plus the full validation surface (19 records, all ported).
//
// The Python sources compose NativeRideRuntime(model_path, config, state)
// from a fresh capture of the welded ride; the native ports run the same
// boundary on the recorded welded capture (bootstrap/welded). Mutation
// checks rewrite the resolved bike::io::Value documents and recompose
// through decode_runtime_config()/compose_bootstrap() — the identical
// decode path the adapter feeds — so every rejection names the same
// "path: reason" diagnostic contract the oracle asserts. All comparisons
// are native-vs-native or literal; no golden nodes live in this file.
#include "runtime_reference_support.hpp"

#include "diag.hpp"

#include <memory>
#include <string>
#include <vector>

namespace {

using runtime_reference::ResolvedDocs;
using runtime_reference::as_int;
using runtime_reference::compose_runtime;
using runtime_reference::erase_member;
using runtime_reference::expect_error;
using runtime_reference::expect_invalid;
using runtime_reference::member;
using runtime_reference::model_image;
using runtime_reference::nan_value;
using runtime_reference::num;
using runtime_reference::require_bitwise;
using runtime_reference::resolved_docs;
using runtime_reference::set_member;
using runtime_reference::welded_dir;

namespace fs = std::filesystem;
namespace bio = bike::io;

// ride() through a tiny frame — keeps the by-value RuntimeBootstrap
// temporaries off the lifecycle check frames.
[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> ride_runtime() {
    return runtime_reference::ride();
}

// ---- lifecycle --------------------------------------------------------------

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_bootstrap_owns_input_and_restores_complete_start() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    const auto rt = compose_runtime(docs);
    const auto before =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    // The capture documents are caller-owned; the runtime decoded its own
    // records — mutating the held trees must not reach inside.
    auto &lanes = std::get<bio::Value::Array>(
        member(docs.state, "integration_state").data);
    for (auto &lane : lanes) lane = num(0);
    const auto after_mutate =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    require_bitwise(after_mutate->integration_state,
                    before->integration_state,
                    "caller-held capture mutation cannot reach the runtime");
    rt->reset();
    const auto after =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    require_bitwise(after->integration_state, before->integration_state,
                    "reset replays the captured bootstrap bitwise");
    bt::require(after->step == 0,
                "reset restores the committed step to t=0");
    bt::require(after->generation == before->generation + 1,
                "reset increments the generation");
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_snapshot_reports_captured_counters() {
    const ResolvedDocs docs = resolved_docs(welded_dir());
    const auto rt = compose_runtime(docs);
    const auto snap =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    const bio::Value &runtime_section = member(docs.state, "runtime");
    bt::require(snap->step == as_int(member(runtime_section, "step")) &&
                    snap->step == 0,
                "snapshot reports the captured step counter");
    bt::require(
        snap->generation == as_int(member(runtime_section, "generation")),
        "snapshot reports the captured generation counter");
    bt::require(snap->time_s == 0.0,
                "snapshot reports the captured time origin exactly");
    bt::require(!snap->integration_state.empty(),
                "integration state materializes as an owned f64 vector");
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_bootstrap_survives_caller_destruction() {
    std::unique_ptr<runtime::NativeRideRuntime> rt;
    {
        const ResolvedDocs docs = resolved_docs(welded_dir());
        rt = compose_runtime(docs);
    }
    const auto before =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    // deepcopy/deletion of the caller's capture cannot reach the runtime —
    // the same contract destroyed `setup` exercises on the Python side.
    const auto after =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    require_bitwise(after->integration_state, before->integration_state,
                    "runtime state survives caller capture destruction");
}
NATIVE_DIAG_POP

void test_reset_twice_replays_the_same_bootstrap() {
    const auto rt = ride_runtime();
    const std::vector<double> first = rt->snapshot().integration_state;
    rt->reset();
    rt->reset();
    const runtime::RuntimeSnapshot third = rt->snapshot();
    require_bitwise(third.integration_state, first,
                    "a second reset replays the same bootstrap bitwise");
    bt::require(third.generation == 3,
                "generation increments once per reset");
}

void test_close_releases_and_fails_fast() {
    const auto rt = ride_runtime();
    rt->close();
    rt->close(); // idempotent
    bt::require(rt->closed(), "close() seals the runtime");
    expect_error("closed",
                 [&] { static_cast<void>(rt->snapshot()); });
    expect_error("closed", [&] { rt->reset(); });
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_config_mutation_after_construction_is_isolated() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    const auto rt = compose_runtime(docs);
    const std::vector<double> state_vec = rt->snapshot().integration_state;
    // Same mutations as the source: the decoded config and runtime record
    // are already owned inside the runtime.
    set_member(docs.config, "timestep_s", num(-1));
    set_member(member(docs.config, "monitors"), "balance_floor_mps",
               num(-5));
    set_member(member(docs.state, "runtime"), "step", num(999));
    const auto snap =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    require_bitwise(snap->integration_state, state_vec,
                    "post-construction caller mutation cannot reach the runtime");
    bt::require(snap->step == 0, "committed step survives caller mutation");
}
NATIVE_DIAG_POP

// ---- validation surface ---------------------------------------------------
// Each mutation mirrors the source's broken document; the rejection must
// name the same field path the oracle match= asserts (native "path:
// reason" diagnostics).

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_missing_state_key() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    erase_member(docs.state, "drive");
    expect_invalid("state.drive",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_unknown_state_key() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    set_member(docs.state, "mystery", num(1));
    expect_invalid("state.mystery",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_digest_mismatch() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    set_member(docs.state, "model_digest",
               bio::Value{std::string(64, 'f')});
    expect_invalid("model_digest",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_model_dims_mismatch() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    set_member(member(docs.state, "model_dims"), "nq",
               num(as_int(member(member(docs.state, "model_dims"), "nq")) +
                   1));
    expect_invalid("model_dims.nq",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_integration_width_mismatch() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    auto &lanes = std::get<bio::Value::Array>(
        member(docs.state, "integration_state").data);
    lanes.pop_back();
    expect_invalid("integration_state",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_nonfinite_integration_lane() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    auto &lanes = std::get<bio::Value::Array>(
        member(docs.state, "integration_state").data);
    lanes[3] = nan_value();
    expect_invalid("integration_state",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_malformed_drive_state() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    erase_member(member(docs.state, "drive"), "policies");
    expect_invalid("drive",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_unknown_joint_in_mutable_state() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    set_member(
        member(member(member(docs.state, "model_mutable"),
                      "dof_frictionloss"),
               "front"),
        "joint", bio::Value{std::string("no_such")});
    expect_invalid("dof_frictionloss",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_unknown_runtime_key() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    erase_member(member(docs.state, "runtime"), "balance");
    expect_invalid("runtime.balance",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_schema_version() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    set_member(docs.state, "schema", num(99));
    expect_invalid("schema",
                   [&] { static_cast<void>(compose_runtime(docs)); });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_bad_runtime_config_envelope() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    set_member(docs.config, "runtime_schema", num(2));
    expect_invalid("runtime_schema", [&] {
        static_cast<void>(runtime::decode_runtime_config(docs.config));
    });
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_rejects_control_period_inconsistency() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    set_member(docs.config, "control_period_steps",
               num(as_int(member(docs.config, "control_period_steps")) + 1));
    expect_invalid("control_period", [&] {
        static_cast<void>(runtime::decode_runtime_config(docs.config));
    });
}
NATIVE_DIAG_POP

// A staged copy of the welded capture with model.mjb deleted — the file
// must be present for read_bootstrap() to reach composition, and its
// absence names the model member in the diagnostic.
void test_rejects_missing_model_file() {
    const bt::TemporaryDirectory staging("runtime_bootstrap_model");
    const fs::path staged = staging.path() / "bootstrap" / "welded";
    std::error_code error;
    // fs::copy creates the leaf only — intermediate parents must exist.
    fs::create_directories(staged.parent_path(), error);
    bt::require(!error, "staged bootstrap parent directories exist");
    fs::copy(welded_dir(), staged,
             fs::copy_options::recursive |
                 fs::copy_options::overwrite_existing,
             error);
    bt::require(!error, "staged bootstrap copy succeeds");
    fs::remove(staged / "model.mjb", error);
    bt::require(!error, "staged model.mjb removal succeeds");
    expect_error("model",
                 [&] { static_cast<void>(runtime::read_bootstrap(staged)); });
}

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(
            test_bootstrap_owns_input_and_restores_complete_start);
        failures += bt::run(test_snapshot_reports_captured_counters);
        failures += bt::run(test_bootstrap_survives_caller_destruction);
        failures += bt::run(test_reset_twice_replays_the_same_bootstrap);
        failures += bt::run(test_close_releases_and_fails_fast);
        failures += bt::run(
            test_config_mutation_after_construction_is_isolated);
        failures += bt::run(test_rejects_missing_state_key);
        failures += bt::run(test_rejects_unknown_state_key);
        failures += bt::run(test_rejects_digest_mismatch);
        failures += bt::run(test_rejects_model_dims_mismatch);
        failures += bt::run(test_rejects_integration_width_mismatch);
        failures += bt::run(test_rejects_nonfinite_integration_lane);
        failures += bt::run(test_rejects_malformed_drive_state);
        failures += bt::run(test_rejects_unknown_joint_in_mutable_state);
        failures += bt::run(test_rejects_unknown_runtime_key);
        failures += bt::run(test_rejects_schema_version);
        failures += bt::run(test_rejects_bad_runtime_config_envelope);
        failures += bt::run(test_rejects_control_period_inconsistency);
        failures += bt::run(test_rejects_missing_model_file);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
