// Completion 3.23 — the per-interval violation surface shared by
// test_acceptance_interval_capture.cpp and test_realistic_pedelec_acceptance
// .cpp. Mirrors the helpers in tests/reference/test_realistic_pedelec_
// acceptance.py: _interval_violations keeps each physical interval's own
// time and channel tuple (never the period/report timestamp) and
// _violations_after applies the startup-grace cutoff to each interval's
// timestamp individually.
#pragma once

#include "runtime/samples.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace acceptance {

// (interval_id, time_s, violations) — the triple _interval_violations
// yields per violating sample.
struct IntervalViolation {
    std::int64_t interval_id = 0;
    double time_s = 0.;
    std::vector<std::string> violations;
};

// _interval_violations(samples): every committed sample carrying a
// nonempty attachment_violations channel, in commit order.
[[nodiscard]] inline std::vector<IntervalViolation>
interval_violations(std::span<const runtime::SamplePtr> samples) {
    namespace sw = runtime::sample_wire;
    std::vector<IntervalViolation> result;
    for (const auto &sample : samples) {
        // channels['attachment_violations'] — the oracle's required-key
        // indexing, not a tolerated absence.
        const auto &channel =
            sw::required(sample->channels, "attachment_violations");
        std::vector<std::string> names;
        for (const auto &name : sw::array(channel))
            names.push_back(sw::string(name));
        if (!names.empty())
            result.push_back({.interval_id = sample->interval_id,
                              .time_s = sample->time_s,
                              .violations = std::move(names)});
    }
    return result;
}

// _violations_after(rows, after_s=.5): the concatenation of each row's
// interval_violations filtered by the violation's own interval timestamp.
[[nodiscard]] inline std::vector<IntervalViolation>
violations_after(const std::vector<std::vector<IntervalViolation>> &rows,
                 double after_s = .5) {
    std::vector<IntervalViolation> result;
    for (const auto &row : rows)
        for (const auto &failure : row)
            if (failure.time_s > after_s) result.push_back(failure);
    return result;
}

inline bool same_violation(const IntervalViolation &a,
                         const IntervalViolation &b) {
    return a.interval_id == b.interval_id && a.time_s == b.time_s &&
           a.violations == b.violations;
}

} // namespace acceptance
