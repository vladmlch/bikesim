// Completion 3.23. Source: tests/reference/test_reference_monitor.py.
// ReferenceMonitor.accept semantics: strict raises InvalidReferenceRun on the
// first violated interval; non-strict warns once and keeps the first failure.
// The native monitor's observe() stores the first failure and sets
// warning_pending in non-strict mode; the caller raises — mirroring
// reference_monitor.py's accept()/observe split.
#include "support/fixture_test.hpp"
#include "runtime/status.hpp"
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

template <class Ex, class F>
void throws(F &&action, std::string_view needle, std::string_view message) {
    try {
        std::forward<F>(action)();
    } catch (const Ex &failure) {
        bt::require(std::string(failure.what()).find(needle) != std::string::npos,
                    std::string(message) + ": unexpected message '" +
                        failure.what() + "'");
        return;
    }
    bt::require(false, std::string(message) + ": no exception was raised");
}

// Python ReferenceMonitor.accept — observe, then strict raise / non-strict
// warning emitted by the call that stored the first failure. Native splits
// warning delivery into take_warning(); the had/pending guard keeps the
// emission tied to the observing call, not to whichever call runs next.
std::optional<std::string> accept(runtime::ReferenceMonitor &monitor, double time_s,
                                  std::vector<std::string> violations) {
  runtime::ReferenceFailure const failure{.time_s = time_s,
                                          .violations = std::move(violations)};
  const auto message = failure.message();
  const bool had_failure = monitor.first_failure.has_value();
  monitor.observe(failure);
  if (monitor.strict && !failure.violations.empty())
    throw runtime::InvalidReferenceRun(message);
  return !had_failure && monitor.warning_pending ? monitor.take_warning()
                                                 : std::nullopt;
}

void require_first(const runtime::ReferenceMonitor &monitor, double time_s,
                   const std::vector<std::string> &violations,
                   std::string_view message) {
  bt::require(monitor.first_failure.has_value(),
              std::string(message) + ": first_failure was not stored");
  bt::require(monitor.first_failure.value().time_s == time_s, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
              std::string(message) + ": first_failure time drifted");
  bt::require(monitor.first_failure.value().violations == violations, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
              std::string(message) + ": first_failure violations drifted");
}

// test_reference_monitor.py::test_research_stops_before_decimation_can_hide_bad_step
void test_research_stops_before_decimation_can_hide_bad_step() {
    runtime::ReferenceMonitor monitor{.strict = true, .first_failure = std::nullopt, .warning_pending = false};
    throws<runtime::InvalidReferenceRun>(
        [&] { accept(monitor, 0.00125, {"foot_front.normal"}); },
        "foot_front.normal", "strict accept raises on the bad interval");
    require_first(monitor, 0.00125, {"foot_front.normal"},
                  "the strict monitor keeps the first failure it raised on");
}

// test_reference_monitor.py::test_viewer_warns_but_never_restores_valid_status
void test_viewer_warns_but_never_restores_valid_status() {
    runtime::ReferenceMonitor monitor{.strict = false, .first_failure = std::nullopt, .warning_pending = false};
    const auto warning = accept(monitor, 0.01, {"grip_left.pull"});
    bt::require(warning.has_value() &&
                    warning->find("grip_left.pull") != std::string::npos,
                "the non-strict monitor emits the first-failure warning");
    bt::require(!accept(monitor, 0.02, {}).has_value(),
                "a clean interval emits no warning");
    require_first(monitor, 0.01, {"grip_left.pull"},
                  "a clean interval never restores valid status");
}

// test_reference_monitor.py::test_non_strict_warns_once_and_keeps_the_first_failure
void test_non_strict_warns_once_and_keeps_the_first_failure() {
    runtime::ReferenceMonitor monitor{.strict = false, .first_failure = std::nullopt, .warning_pending = false};
    const auto warning = accept(monitor, 0.01, {"foot_rear.friction"});
    bt::require(warning.has_value() &&
                    warning->find("foot_rear.friction") != std::string::npos,
                "the first non-strict failure warns");
    bt::require(!accept(monitor, 0.02, {"saddle.normal"}).has_value(),
                "a later violation never warns twice");
    bt::require(!accept(monitor, 0.03, {}).has_value(),
                "a clean interval emits no warning");
    require_first(monitor, 0.01, {"foot_rear.friction"},
                  "later violations never replace the first failure");
}

// test_reference_monitor.py::test_strict_raises_even_after_a_stored_failure
void test_strict_raises_even_after_a_stored_failure() {
    runtime::ReferenceMonitor monitor{.strict = true, .first_failure = std::nullopt, .warning_pending = false};
    throws<runtime::InvalidReferenceRun>(
        [&] { accept(monitor, 0.005, {"grip.pull"}); },
        "grip.pull", "strict accept raises on the first failure");
    require_first(monitor, 0.005, {"grip.pull"},
                  "the strict monitor stores the failure it raised on");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_research_stops_before_decimation_can_hide_bad_step();
        test_viewer_warns_but_never_restores_valid_status();
        test_non_strict_warns_once_and_keeps_the_first_failure();
        test_strict_raises_even_after_a_stored_failure();
    });
}
