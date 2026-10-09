#include "period_buffer.hpp"
#include "samples.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace runtime {
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) capacity and first interval are distinct domains guarded below
PeriodBuffer::PeriodBuffer(std::size_t capacity, std::int64_t first_interval,
                           std::optional<double> previous_end)
    : capacity_(capacity), next_interval_(first_interval), previous_end_(previous_end) {
    if (capacity_ == 0 || first_interval < 0 ||
        (previous_end.has_value() && (!std::isfinite(*previous_end) || *previous_end < 0.)))
        throw std::invalid_argument("period buffer needs positive capacity and valid interval markers");
    raws_.reserve(capacity_);
}

void PeriodBuffer::push(RawStep raw) {
    if (full()) throw std::runtime_error("period buffer overflow: close the period first");
    if (raw.interval_id != next_interval_ ||
        (previous_end_.has_value() && std::abs(raw.time_s - *previous_end_) > 1e-10))
        throw std::invalid_argument("period buffer requires consecutive, continuous intervals");
    if (!std::isfinite(raw.time_s) || raw.time_s < 0. ||
        !std::isfinite(raw.end_time_s) || raw.end_time_s <= raw.time_s)
        throw std::invalid_argument("period buffer requires a finite positive interval");
    if (next_interval_ == std::numeric_limits<std::int64_t>::max())
        throw std::overflow_error("physical interval ID overflow");
    const double end = raw.end_time_s;
    raws_.push_back(std::move(raw));
    ++next_interval_;
    previous_end_ = end;
}

namespace {
std::optional<std::vector<double>> spatial_wrench(
    const rider::DenseMatrix &jac, std::span<const double> force,
    rider::LeastSquaresWorkspace &workspace) {
    if (jac.rows != 6 || jac.columns == 0 ||
        jac.columns > std::numeric_limits<std::size_t>::max() / jac.rows ||
        jac.values.size() != jac.rows * jac.columns || force.size() != jac.columns)
        throw std::invalid_argument("attachment raw: incompatible spatial wrench dimensions");
    std::vector<double> transpose(jac.values.size());
    for (std::size_t row = 0; row < jac.columns; ++row)
        for (std::size_t column = 0; column < jac.rows; ++column)
            transpose[row * jac.rows + column] = jac.values[column * jac.columns + row];
    // Spatial bodies need explanation, not full six-dimensional rank.
    // The existing native minimum-norm primitive uses the same 1e-12 cutoff.
    auto solved = workspace.solve(transpose, static_cast<rider::lapack_int>(jac.columns),
                                   6, force, 1, 1e-12);
    for (std::size_t row = 0; row < jac.columns; ++row) {
        const auto coefficients = std::span<const double>(transpose).subspan(row * 6, 6);
        const double explained = sample_wire::dot(coefficients, solved.solution);
        if (!std::isfinite(force[row]) ||
            std::abs(explained - force[row]) > 1e-8 + 1e-8 * std::abs(force[row]))
            return std::nullopt;
    }
    return std::move(solved.solution);
}
} // namespace

AttachmentSamples evaluate_attachments(const RawStep &raw,
                                         rider::LeastSquaresWorkspace &workspace) {
    AttachmentSamples samples;
    for (const auto &[name, item] : raw.attachment_raw) {
        const auto rider_wrench = spatial_wrench(item.rider_jac, item.rider_qfrc, workspace);
        const auto bike_wrench = spatial_wrench(item.bike_jac, item.bike_qfrc, workspace);
        if (!rider_wrench.has_value() || !bike_wrench.has_value() || !item.observable)
            continue;
        const auto &wr = *rider_wrench;
        const auto &wb = *bike_wrench;
        const auto force = std::span<const double>(wr).first(3);
        const double scale = std::max(1., std::sqrt(sample_wire::dot(force, force)));
        bool valid = true;
        for (std::size_t i = 0; i < wr.size(); ++i)
            valid = valid && std::abs(wb[i] + wr[i]) <= 1e-4 * scale + 1e-4 * std::abs(wr[i]);
        for (const std::size_t i : std::array<std::size_t, 3>{1, 3, 5})
            valid = valid && std::abs(wr[i]) <= 1e-6 * scale;
        if (valid)
            samples.emplace_back(name, rider::decompose_wrench(
                wr, item.normal, item.kind, item.rotational, item.half_patch_m,
                item.gap_m, item.pull_direction));
    }
    // evaluate_attachments visits sorted attachment names, while error
    // reporting below keeps the original raw-map insertion order.
    std::ranges::sort(samples, [](const auto &a, const auto &b) { return a.first < b.first; });
    return samples;
}

std::vector<std::string> attachment_violations(
    const rider::AttachmentSample &sample, const AttachmentBudget &budget) {
    if (sample.kind != "foot" && sample.kind != "saddle" && sample.kind != "grip")
        throw std::invalid_argument("unknown attachment kind");
    const std::array<double, 6> values{sample.normal_n, sample.tangent_n,
        sample.moment_nm, sample.gap_m, sample.pull_n, sample.half_patch_m};
    if (!std::ranges::all_of(values, [](double value) { return std::isfinite(value); }))
        return {"nonfinite"};
    if (std::min({sample.gap_m, sample.pull_n, sample.half_patch_m}) < 0.)
        return {"invalid_measurement"};
    std::vector<std::string> errors;
    if (sample.gap_m > budget.max_gap_m) errors.emplace_back("gap");
    if (sample.kind == "grip") {
        if (sample.pull_n > budget.grip_pull_n) errors.emplace_back("pull");
        return errors;
    }
    if ((sample.kind == "foot" && sample.normal_n < budget.foot_min_normal_n) ||
        (sample.kind == "saddle" && sample.normal_n <= 0.)) errors.emplace_back("normal");
    const double mu = sample.kind == "foot" ? budget.foot_mu : budget.saddle_mu;
    if (std::abs(sample.tangent_n) > mu * std::max(0., sample.normal_n))
        errors.emplace_back("friction");
    if (std::abs(sample.moment_nm) > sample.half_patch_m * std::max(0., sample.normal_n))
        errors.emplace_back("cop");
    return errors;
}
} // namespace runtime
