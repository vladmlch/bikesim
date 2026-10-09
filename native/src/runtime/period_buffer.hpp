// Raw captures survive partial calls and failed period publication.
#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>
#include "step.hpp"

namespace runtime {
class PeriodBuffer {
public:
    explicit PeriodBuffer(std::size_t capacity, std::int64_t first_interval = 0,
                          std::optional<double> previous_end = std::nullopt);
    void push(RawStep raw);
    [[nodiscard]] bool full() const noexcept { return raws_.size() == capacity_; }
    [[nodiscard]] bool empty() const noexcept { return raws_.empty(); }
    [[nodiscard]] std::span<const RawStep> raws() const noexcept { return raws_; }
    // Publication acknowledges captures only after the whole period commits.
    // The next ID/time markers deliberately survive an explicit flush.
    void clear() noexcept { raws_.clear(); }
private:
    std::size_t capacity_;
    std::int64_t next_interval_;
    std::optional<double> previous_end_;
    std::vector<RawStep> raws_;
};

struct AttachmentBudget {
    double foot_min_normal_n = 0.;
    double foot_mu = 0., saddle_mu = 0.;
    double grip_pull_n = 0., max_gap_m = 0.;
};
using AttachmentSamples = std::vector<std::pair<std::string, rider::AttachmentSample>>;
[[nodiscard]] std::vector<std::string>
attachment_violations(const rider::AttachmentSample &sample, const AttachmentBudget &budget);
[[nodiscard]] AttachmentSamples
evaluate_attachments(const RawStep &raw, rider::LeastSquaresWorkspace &workspace);
} // namespace runtime
