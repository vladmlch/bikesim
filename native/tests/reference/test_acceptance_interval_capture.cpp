// Completion 3.23. Source: tests/reference/test_acceptance_interval_capture.py.
// Road acceptance must not hide an intermediate rejected physical interval:
// _interval_violations reads each committed interval's own timestamp and
// _violations_after applies the startup grace per-interval, so a clean
// period endpoint cannot mask an earlier rejection.
#include "acceptance_reference_support.hpp"
#include "support/fixture_test.hpp"
#include "runtime/samples.hpp"
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace {

runtime::SamplePtr sample(std::int64_t interval_id, double time_s, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                          std::vector<std::string> violations) {
    auto row = std::make_shared<runtime::PhysicalSampleData>();
    row->interval_id = interval_id;
    row->time_s = time_s;
    row->end_time_s = time_s;
    row->channels = {{"attachment_violations",
                      runtime::sample_wire::strings(violations)}};
    return row;
}

// test_acceptance_interval_capture.py::
// test_earlier_interval_failure_survives_clean_period_endpoint
void test_earlier_interval_failure_survives_clean_period_endpoint() {
    const std::vector<runtime::SamplePtr> samples{
        sample(0, .6, {"rider_power.rider_hip_front", "energy.constraint_work"}),
        sample(1, .60125, {})};
    const std::vector<std::vector<acceptance::IntervalViolation>> rows{
        acceptance::interval_violations(samples)};
    const auto kept = acceptance::violations_after(rows);
    bt::require(kept.size() == 1 &&
                    acceptance::same_violation(
                        kept.front(),
                        {.interval_id = 0, .time_s = .6,
                         .violations = {"rider_power.rider_hip_front",
                                        "energy.constraint_work"}}),
                "the earlier violating interval survives the clean endpoint");
}

// test_acceptance_interval_capture.py::
// test_startup_grace_uses_each_violation_interval_time
void test_startup_grace_uses_each_violation_interval_time() {
    const std::vector<std::vector<acceptance::IntervalViolation>> rows{
        {{.interval_id = 10, .time_s = .5, .violations = {"saddle.normal"}}},
        {{.interval_id = 11, .time_s = .50125,
          .violations = {"rider_strength.rider_hip_rear"}}}};
    const auto kept = acceptance::violations_after(rows);
    bt::require(kept.size() == 1 &&
                    acceptance::same_violation(
                        kept.front(),
                        {.interval_id = 11, .time_s = .50125,
                         .violations = {"rider_strength.rider_hip_rear"}}),
                "grace filtering keys on each violation's interval time");
}

} // namespace

int main() {
    return bt::run([] {
        test_earlier_interval_failure_survives_clean_period_endpoint();
        test_startup_grace_uses_each_violation_interval_time();
    });
}
