// Completion 3.25 (batch runtime). Source: tests/reference/
// test_native_runtime_step.py — A3 owned physical step and bounded
// NativeRideRuntime.advance().
//
// Ported (4 of 10 records): every assertion below compares native state
// against native state, so no oracle data enters this file. The remaining
// six records compare NativeRideRuntime against the *live Python oracle's*
// integration vector bit-for-bit; they are pre-existing live-oracle reds
// on both platforms (A/B-verified unchanged across the F4 refresh_forward
// fix — see task-F4-report.md and task-C325-report.md) and stay `planned`
// in the port map rather than being ported past as green:
//   test_native_first_step_uses_settled_material_state
//   test_native_step_sequence_matches_python_state
//   test_native_force_fold_matches_python_recorded_inputs
//   test_native_advance_tracks_python_under_load
//   test_sensor_channels_read_sensordata_rows
//   test_native_intent_lanes_match_python_past_reaction_delay
#include "runtime_reference_support.hpp"

#include "diag.hpp"

#include <memory>
#include <vector>

namespace {

using runtime_reference::bitwise_equal;
using runtime_reference::expect_error;
using runtime_reference::require_bitwise;

[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> ride() {
    return runtime_reference::ride();
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
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
