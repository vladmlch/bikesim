// Completion 3.25 (batch runtime). Source: tests/reference/
// test_native_runtime_step.py — A3 owned physical step and bounded
// NativeRideRuntime.advance().
//
// Ported (all 10 records): the four native-vs-native contracts plus the
// six oracle-parity records, now gated on FT-2 goldens replayed through
// the committed welded bootstrap under
// tests/fixtures/ft2/<platform>/ride.
#include "ft2_reference_support.hpp"
#include "runtime_reference_support.hpp"

#include "diag.hpp"

#include <array>
#include <cstdio>
#include <memory>
#include <utility>
#include <string>
#include <vector>

namespace {

using ft2_reference::consume_tree;
using runtime_reference::bitwise_equal;
using runtime_reference::require_bitwise;

constexpr std::string_view kNode =
    "tests/reference/test_native_runtime_step.py::";

// The FT-2 ride capture: make_python_ride(strict=False, decimation=1)
// frozen at the oracle boundary — the identical recipe to
// runtime_reference::ride() but sourced from the platform-qualified
// capture directory that owns these goldens.
[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> ride() {
    return ft2_reference::ride();
}

// The call-chunking contract: one bounded advance(40) vs the (1,3,9,17,40)
// chunk ladder must land on the identical committed integration vector.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_physics_result_is_independent_of_call_chunks() {
    const auto one = ride();
    const auto chunks = ride();
    const runtime::RideControl command;
    const runtime::AdvanceResult whole =
        one->advance(40, command, 0., 0., std::nullopt);
    for (const std::int64_t target : {1, 3, 9, 17, 40})
        static_cast<void>(chunks->advance(target, command, 0., 0.,
                                          std::nullopt));
    bt::require(whole.step == 40 && whole.reason == "target",
                "the single advance commits to target 40");
    const auto a = std::make_unique<const runtime::RuntimeSnapshot>(
        one->snapshot());
    const auto b = std::make_unique<const runtime::RuntimeSnapshot>(
        chunks->snapshot());
    bt::require(a->step == 40 && b->step == 40,
                "both chunkings commit to step 40");
    require_bitwise(a->integration_state, b->integration_state,
                    "committed state is independent of call chunking");
}
NATIVE_DIAG_POP

// reset() replays the captured bootstrap; a re-advanced runtime tracks a
// fresh replay bitwise (assert_array_equal on the source — bitwise is the
// stronger form for native-vs-native).
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_reset_after_advance_replays_the_bootstrap() {
    const auto native = ride();
    const auto replay = ride();
    const runtime::RideControl command;
    static_cast<void>(native->advance(15, command, 0., 0., std::nullopt));
    native->reset();
    bt::require(native->snapshot().step == 0,
                "reset returns to the committed t=0");
    static_cast<void>(native->advance(7, command, 0., 0., std::nullopt));
    static_cast<void>(replay->advance(7, command, 0., 0., std::nullopt));
    require_bitwise(native->snapshot().integration_state,
                    replay->snapshot().integration_state,
                    "reset replay reproduces a fresh runtime bitwise");
}
NATIVE_DIAG_POP

// snapshot() materializes an owned copy — the held frame cannot observe
// later steps, shares no storage with a newer copy, and mutating it can
// never reach the runtime.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_snapshot_is_an_owned_copy() {
    const auto native = ride();
    const runtime::RideControl command;
    static_cast<void>(native->advance(3, command, 0., 0., std::nullopt));
    auto snap =
        std::make_unique<runtime::RuntimeSnapshot>(native->snapshot());
    static_cast<void>(native->advance(4, command, 0., 0., std::nullopt));
    const auto later =
        std::make_unique<const runtime::RuntimeSnapshot>(
            native->snapshot());
    bt::require(snap->step == 3 && later->step == 4,
                "the held snapshot keeps its committed boundary");
    bt::require(!bitwise_equal(snap->integration_state,
                               later->integration_state),
                "step-3 and step-4 vectors differ");
    bt::require(snap->integration_state.data() !=
                    later->integration_state.data(),
                "snapshots share no storage");
    // A mutated copy of the held data cannot reach the runtime.
    std::vector<double> held = snap->integration_state;
    for (double &lane : held) lane = 0.;
    require_bitwise(native->snapshot().integration_state,
                    later->integration_state,
                    "caller-held copies never mutate committed state");
    snap.reset();
}
NATIVE_DIAG_POP

// probe_step_inputs() surfaces apply_forces(advance=False)'s staged
// diagnostics: the drive-diagnostics dict and a contact-probe view that
// is either absent or a dict — plus the non-advancing contract (no
// committed state moves).
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_probe_step_inputs_returns_staged_diagnostics() {
    const auto native = ride();
    const runtime::RideControl command;
    const runtime::PhysicalStep::ProbeResult probe =
        native->probe_step_inputs(command, 0., 0.);
    // probe['drive'] is always the staged diagnostics dict; the contact
    // probe is a dict exactly when the flag marks it populated.
    bt::require(probe.has_contact_probe == !probe.contact_probe.empty(),
                "contact_probe is a dict or absent per its flag");
    static_cast<void>(probe);
    bt::require(native->snapshot().step == 0,
                "a probe is not a step: committed state does not move");
}
NATIVE_DIAG_POP

// ---- FT-2 oracle-parity ports ----------------------------------------------
// Each test replays the oracle's committed boundary out of the platform
// golden; entries stream in the Python file's capture order.

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_native_first_step_uses_settled_material_state() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_native_first_step_uses_settled_material_state");
    const auto native = ride();
    const runtime::RideControl command;
    static_cast<void>(native->advance(1, command, 0., 0., std::nullopt));
    node.f64(native->snapshot().integration_state, "settled first step");
    node.done();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_native_step_sequence_matches_python_state() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_native_step_sequence_matches_python_state");
    const auto native = ride();
    const runtime::RideControl command;
    char label[32]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    for (std::int64_t target = 1; target <= 30; ++target) {
        const runtime::AdvanceResult result =
            native->advance(target, command, 0., 0., std::nullopt);
        bt::require(result.step == target && result.reason == "target",
                    "committed interval lands on the requested target");
        std::snprintf(label, sizeof(label), "step %lld", // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                      static_cast<long long>(target));
        node.f64(native->snapshot().integration_state, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
    }
    node.done();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_native_force_fold_matches_python_recorded_inputs() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_native_force_fold_matches_python_recorded_inputs");
    const auto native = ride();
    const runtime::RideControl command;
    const std::array<std::pair<double, double>, 3> demands{
        {{0., 0.}, {.35, .2}, {1., 1.}}};
    char label[48]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    int fold = 0;
    for (const auto &[front, rear] : demands) {
        const runtime::PhysicalStep::ProbeResult probe =
            native->probe_step_inputs(command, front, rear);
        // components dict, in the oracle's compute_components order.
        runtime::WireObject components;
        for (const auto &[name, force] : probe.components) {
            runtime::WireArray row;
            for (const double value : force) row.emplace_back(value);
            components.emplace_back(name, runtime::Wire{std::move(row)});
        }
        std::snprintf(label, sizeof(label), "fold.%d.components", fold); // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
        consume_tree(node, components, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        std::snprintf(label, sizeof(label), "fold.%d.ctrl", fold); // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
        node.f64_close(probe.ctrl, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        const double bounds[2] = {probe.front_brake_bound_nm, // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
                                  probe.rear_brake_bound_nm};
        std::snprintf(label, sizeof(label), "fold.%d.brake_bounds", fold); // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
        node.f64_close(bounds, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        bt::require(native->snapshot().step == 0,
                    "a probe commits no state");
        const std::vector<double> before =
            native->snapshot().integration_state;
        native->reset();
        bt::require(bitwise_equal(native->snapshot().integration_state,
                                  before),
                    "reset replays the bootstrap bitwise");
        ++fold;
    }
    node.done();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_native_advance_tracks_python_under_load() {
    bt::oracle::Node node(
        std::string(kNode) + "test_native_advance_tracks_python_under_load");
    const auto native = ride();
    const runtime::RideControl command;
    char label[32]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    for (std::int64_t target = 1; target <= 30; ++target) {
        const double demand = (target >= 11 && target <= 20) ? .6 : 0.;
        static_cast<void>(native->advance(target, command, demand, demand,
                                        std::nullopt));
        std::snprintf(label, sizeof(label), "interval %lld", // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                      static_cast<long long>(target));
        node.f64(native->snapshot().integration_state, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
    }
    node.done();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_sensor_channels_read_sensordata_rows() {
    bt::oracle::Node node(
        std::string(kNode) + "test_sensor_channels_read_sensordata_rows");
    const auto native = ride();
    const runtime::RideControl command;
    char label[64]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    for (std::int64_t target = 1; target <= 5; ++target) {
        static_cast<void>(
            native->advance(target, command, 0., 0., std::nullopt));
        // The oracle staged apply_forces(advance=False) before reading;
        // the probe stages the same inputs on the native side.
        const runtime::PhysicalStep::ProbeResult probe =
            native->probe_step_inputs(command, 0., 0.);
        const auto gyro = runtime::sample_wire::array(
            runtime::sample_wire::required(probe.sensors,
                                           "frame_gyro_body_rad_s"));
        const auto accel = runtime::sample_wire::array(
            runtime::sample_wire::required(
                probe.sensors, "frame_specific_force_body_mps2"));
        std::vector<double> gyro_values, accel_values;
        for (const runtime::Wire &lane : gyro)
            gyro_values.push_back(runtime::sample_wire::number(lane));
        for (const runtime::Wire &lane : accel)
            accel_values.push_back(runtime::sample_wire::number(lane));
        std::snprintf(label, sizeof(label), // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                      "step %lld frame_gyro_body_rad_s",
                      static_cast<long long>(target));
        node.f64(gyro_values, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        std::snprintf(label, sizeof(label), // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                      "step %lld frame_specific_force_body_mps2",
                      static_cast<long long>(target));
        node.f64(accel_values, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
    }
    node.done();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_native_intent_lanes_match_python_past_reaction_delay() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_native_intent_lanes_match_python_past_reaction_delay");
    const auto native = ride();
    const runtime::RideControl command;
    for (std::int64_t target = 1; target <= 160; ++target)
        static_cast<void>(
            native->advance(target, command, 0., 0., std::nullopt));
    bt::require(native->snapshot().step == 160,
                "160 intervals committed");
    node.f64(native->snapshot().integration_state,
             "step 160 integration state");
    const runtime::PhysicalStep::ProbeResult probe =
        native->probe_step_inputs(command, 0., 0.);
    consume_tree(node, probe.intent_signals, "intent_signals");
    const double inclination[1] = {probe.intent_inclination_rad}; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    node.f64(inclination, "intent_inclination_rad");
    // The source file's trailing bitwise leg: the force vector lane.
    const auto &force = runtime::sample_wire::array(
        runtime::sample_wire::required(probe.intent_signals,
                                       "specific_force_body_mps2"));
    std::vector<double> force_values;
    for (const runtime::Wire &lane : force)
        force_values.push_back(runtime::sample_wire::number(lane));
    node.f64(force_values, "");
    node.done();
}
NATIVE_DIAG_POP

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(
            test_physics_result_is_independent_of_call_chunks);
        failures += bt::run(test_reset_after_advance_replays_the_bootstrap);
        failures += bt::run(test_snapshot_is_an_owned_copy);
        failures += bt::run(
            test_probe_step_inputs_returns_staged_diagnostics);
        failures += bt::run(
            test_native_first_step_uses_settled_material_state);
        failures += bt::run(test_native_step_sequence_matches_python_state);
        failures += bt::run(
            test_native_force_fold_matches_python_recorded_inputs);
        failures += bt::run(test_native_advance_tracks_python_under_load);
        failures += bt::run(test_sensor_channels_read_sensordata_rows);
        failures += bt::run(
            test_native_intent_lanes_match_python_past_reaction_delay);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
