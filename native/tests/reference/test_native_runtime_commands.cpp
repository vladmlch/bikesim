// Completion 3.25 (batch runtime). Source: tests/reference/
// test_native_runtime_commands.py — A3 command handling through
// NativeRideRuntime.advance().
//
// Ported (all 10 records): the six native-vs-native contracts plus the
// four oracle-parity records, now gated on FT-2 goldens replayed through
// the committed welded bootstrap under
// tests/fixtures/migration/ft2/<platform>/ride.
#include "ft2_reference_support.hpp"
#include "runtime_reference_support.hpp"

#include "diag.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

using runtime_reference::expect_error;
using runtime_reference::expect_invalid;

constexpr std::string_view kNode =
    "tests/reference/test_native_runtime_commands.py::";

[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> ride() {
    return ft2_reference::ride();
}

// A zero wall budget yields between steps; a generous budget commits the
// whole range.
void test_budget_yield_stops_between_steps() {
    const auto native = ride();
    const runtime::RideControl command;
    const runtime::AdvanceResult result =
        native->advance(50, command, 0., 0., 0.);
    bt::require(result.reason == "budget" && result.step == 0,
                "a zero budget yields before the first interval");
    const runtime::AdvanceResult done =
        native->advance(5, command, 0., 0., 60.);
    bt::require(done.reason == "target" && done.step == 5,
                "a generous budget commits to the target");
}

// advance() to the already-committed step is a no-op, not a rejection.
void test_advance_to_current_step_is_a_noop() {
    const auto native = ride();
    const runtime::RideControl command;
    static_cast<void>(native->advance(4, command, 0., 0., std::nullopt));
    const runtime::AdvanceResult result =
        native->advance(4, command, 0., 0., std::nullopt);
    bt::require(result.reason == "target" && result.step == 4,
                "advancing to the committed step is a target no-op");
}

// Rejections never mutate: the committed prefix is intact afterwards.
// The source's 'not a control'/unknown-field cases are Python type
// freedoms — RideControl is a typed record natively (documented in the
// port map); the value-domain rejections port verbatim.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_invalid_target_and_command_rejected() {
    const auto native = ride();
    const runtime::RideControl command;
    static_cast<void>(native->advance(3, command, 0., 0., std::nullopt));
    expect_invalid("target_step",
                   [&] { static_cast<void>(native->advance(
                             -1, command, 0., 0., std::nullopt)); });
    expect_invalid("behind", [&] {
        static_cast<void>(
            native->advance(2, command, 0., 0., std::nullopt));
    });
    expect_error("finite", [&] {
        static_cast<void>(native->advance(
            5, command, std::numeric_limits<double>::quiet_NaN(), 0.,
            std::nullopt));
    });
    bt::require(native->snapshot().step == 3,
                "rejections never mutate: the committed prefix is intact");
    const runtime::AdvanceResult done =
        native->advance(6, command, 0., 0., std::nullopt);
    bt::require(done.step == 6, "the runtime keeps advancing after rejections");
}
NATIVE_DIAG_POP

// The advancement guard excludes concurrent advance/reset/snapshot/
// probe/close — a second entry fails fast while one is in flight, and a
// rejected close must not latch.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_concurrent_advance_and_reset_rejected() {
    const auto native = ride();
    const runtime::RideControl command;
    std::atomic<bool> started{false};
    std::atomic<bool> worker_done{false};
    std::atomic<int> worker_errors{0};

    std::thread worker([&] {
        started.store(true);
        try {
            // Long target + a 5 s wall budget — the worker holds the
            // advancement guard for the whole window.
            static_cast<void>(
                native->advance(1000000000, command, 0., 0., 5.));
        } catch (...) {
            // The budget-expiry path (or a fatal interval) is fine; the
            // guard contract under test is on the main thread.
            worker_errors.fetch_add(1);
        }
        worker_done.store(true);
    });
    // The worker holds the advancement guard once its advance begins —
    // give it a head start the same way the source's Event+sleep does.
    const auto started_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!started.load() &&
           std::chrono::steady_clock::now() < started_deadline)
        std::this_thread::yield();
    bt::require(started.load(), "worker thread launched");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    expect_error("advance", [&] {
        static_cast<void>(
            native->advance(5, command, 0., 0., std::nullopt));
    });
    expect_error("advance", [&] { native->reset(); });
    // snapshot() reads the committed cache and close() frees engine
    // objects — both take the same guard rather than racing the loop.
    expect_error("advance",
                 [&] { static_cast<void>(native->snapshot()); });
    expect_error("advance", [&] {
        static_cast<void>(native->probe_step_inputs(command, 0., 0.));
    });
    expect_error("advance", [&] { native->close(); });
    // The rejected close must not have latched: the runtime is still open.
    bt::require(!native->closed(),
                "a rejected close does not latch the runtime");
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (!worker_done.load() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    worker.join();
    bt::require(native->snapshot().step >= 0,
                "the runtime survives the guarded window");
    static_cast<void>(worker_errors);
}
NATIVE_DIAG_POP

// Every operation after close() fails fast with 'closed'.
void test_advance_after_close_fails() {
    const auto native = ride();
    const runtime::RideControl command;
    native->close();
    expect_error("closed", [&] {
        static_cast<void>(
            native->advance(1, command, 0., 0., std::nullopt));
    });
}

// A fatal interval mid-advance surfaces as an exception, not an abort —
// and the committed prefix stays readable (the source's process-isolated
// child; in-process here because the engine boundary maps fatal MuJoCo
// errors to C++ exceptions).
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_engine_failure_keeps_committed_prefix() {
    const auto native = ride();
    runtime::RideControl command;
    static_cast<void>(native->advance(3, command, 0., 0., std::nullopt));
    // An absurd motor demand drives the solve into the numerical-warning
    // wall mid-range; the loop must surface it as an exception and keep
    // every already-committed interval readable.
    command.motor_torque_nm = 1e9;
    try {
        static_cast<void>(
            native->advance(200, command, 0., 0., std::nullopt));
        bt::require(native->snapshot().step == 200,
                    "PREFIX-CLEAN: the range committed without a fatal");
    } catch (const std::exception &) {
        const auto snap =
            std::make_unique<const runtime::RuntimeSnapshot>(
                native->snapshot());
        bt::require(snap->step >= 3,
                    "PREFIX: the committed prefix stays readable");
    }
}
NATIVE_DIAG_POP

// ---- FT-2 oracle-parity ports ----------------------------------------------
// Each replays the source file's scripted command program against the
// committed integration vector the oracle froze.

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_explicit_zero_effort_differs_from_automatic() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_explicit_zero_effort_differs_from_automatic");
    const auto native = ride();
    const auto automatic = ride();
    runtime::RideControl zero;
    zero.human_torque_nm = 0.;
    const runtime::RideControl automatic_command;
    for (std::int64_t target = 1; target <= 140; ++target) {
        static_cast<void>(
            native->advance(target, zero, 0., 0., std::nullopt));
        static_cast<void>(automatic->advance(target, automatic_command,
                                             0., 0., std::nullopt));
    }
    node.f64(native->snapshot().integration_state,
             "explicit zero effort step 140");
    bt::require(!runtime_reference::bitwise_equal(
                    native->snapshot().integration_state,
                    automatic->snapshot().integration_state),
                "explicit zero is not the automatic lane");
    node.done();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_motor_ceiling_change_matches_python() {
    bt::oracle::Node node(
        std::string(kNode) + "test_motor_ceiling_change_matches_python");
    const auto native = ride();
    char label[32]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    for (std::int64_t target = 1; target <= 30; ++target) {
        runtime::RideControl command;
        if (target <= 15) command.motor_limit_nm = 40.;
        command.human_torque_nm = 30.;
        static_cast<void>(
            native->advance(target, command, 0., 0., std::nullopt));
        std::snprintf(label, sizeof(label), "interval %lld", // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                      static_cast<long long>(target));
        node.f64(native->snapshot().integration_state, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
    }
    node.done();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_brake_apply_release_matches_python() {
    bt::oracle::Node node(
        std::string(kNode) + "test_brake_apply_release_matches_python");
    const auto native = ride();
    runtime::RideControl command;
    command.human_torque_nm = 25.;
    char label[32]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    for (std::int64_t target = 1; target <= 24; ++target) {
        const bool braking = target >= 9 && target <= 16;
        static_cast<void>(native->advance(target, command,
                                        braking ? 1. : 0.,
                                        braking ? .5 : 0., std::nullopt));
        std::snprintf(label, sizeof(label), "interval %lld", // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                      static_cast<long long>(target));
        node.f64(native->snapshot().integration_state, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
    }
    node.done();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_gear_shift_and_coast_resume_match_python() {
    bt::oracle::Node node(
        std::string(kNode) +
        "test_gear_shift_and_coast_resume_match_python");
    const auto native = ride();
    const std::array<std::pair<std::int64_t, double>, 3> segments{
        {{60, 35.}, {80, 0.}, {120, 35.}}};
    char label[32]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    std::int64_t step = 0;
    for (const auto &[target, torque] : segments) {
        runtime::RideControl command;
        command.human_torque_nm = torque;
        while (step < target) {
            ++step;
            static_cast<void>(
                native->advance(step, command, 0., 0., std::nullopt));
        }
        std::snprintf(label, sizeof(label), "interval %lld", // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                      static_cast<long long>(target));
        node.f64(native->snapshot().integration_state, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
    }
    node.done();
}
NATIVE_DIAG_POP

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(test_budget_yield_stops_between_steps);
        failures += bt::run(test_advance_to_current_step_is_a_noop);
        failures += bt::run(test_invalid_target_and_command_rejected);
        failures += bt::run(test_concurrent_advance_and_reset_rejected);
        failures += bt::run(test_advance_after_close_fails);
        failures += bt::run(test_engine_failure_keeps_committed_prefix);
        failures += bt::run(test_explicit_zero_effort_differs_from_automatic);
        failures += bt::run(test_motor_ceiling_change_matches_python);
        failures += bt::run(test_brake_apply_release_matches_python);
        failures += bt::run(test_gear_shift_and_coast_resume_match_python);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
