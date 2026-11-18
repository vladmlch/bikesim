#include "period_buffer.hpp"
#include "samples.hpp"
#include "../spatial/frames.hpp"
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
    // The (6, k) row-major Jacobian already carries the (k, 6)
    // column-major J.T layout DGELSD stages — byte-for-byte what an
    // explicit transpose produced. Spatial bodies need explanation,
    // not full six-dimensional rank; the existing native minimum-norm
    // primitive uses the same 1e-12 cutoff.
    auto solved = workspace.solve_column_major(
        jac.values, static_cast<rider::lapack_int>(jac.columns), 6, force,
        1, 1e-12);
    for (std::size_t row = 0; row < jac.columns; ++row) {
        // J.T's row is J's strided column — gathered contiguous so the
        // dot keeps unit-stride BLAS dispatch and summation order.
        std::array<double, 6> coefficients{};
        for (std::size_t j = 0; j < 6; ++j)
            coefficients[j] = jac.values[j * jac.columns + row];
        const double explained = sample_wire::dot(coefficients, solved.solution);
        if (!std::isfinite(force[row]) ||
            std::abs(explained - force[row]) > 1e-8 + 1e-8 * std::abs(force[row]))
            return std::nullopt;
    }
    return std::move(solved.solution);
}
} // namespace

namespace {
std::optional<rider::AttachmentSample> decompose_spatial_wrench(
    std::span<const double> wrench, std::span<const double> normal,
    const std::string &kind, bool rotational, double half_patch_m,
    double gap_m, const std::optional<std::vector<double>> &pull_direction,
    std::span<const double> qpos) {
    if (wrench.size() < 6 || normal.size() != 3 ||
        !std::ranges::all_of(normal, [](double v) { return std::isfinite(v); }))
        return std::nullopt;
    const double n_len = std::sqrt(
        normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    if (n_len < 1e-12) return std::nullopt;
    const std::array<double, 3> n_unit{
        normal[0] / n_len, normal[1] / n_len, normal[2] / n_len};
    const std::array<double, 3> f_rider{wrench[0], wrench[1], wrench[2]};
    const double normal_n =
        f_rider[0] * n_unit[0] + f_rider[1] * n_unit[1] + f_rider[2] * n_unit[2];
    const std::array<double, 3> f_tan{
        f_rider[0] - normal_n * n_unit[0],
        f_rider[1] - normal_n * n_unit[1],
        f_rider[2] - normal_n * n_unit[2]};
    const double tangent_n = std::sqrt(
        f_tan[0] * f_tan[0] + f_tan[1] * f_tan[1] + f_tan[2] * f_tan[2]);
    double moment_nm = 0.;
    if (rotational) {
        std::array<double, 3> frame_y{0., 1., 0.};
        if (qpos.size() >= 7) {
            const bike::spatial::Quat quat{qpos[3], qpos[4], qpos[5], qpos[6]};
            frame_y = bike::spatial::rotate_body_to_world(quat, {0., 1., 0.});
        }
        moment_nm =
            wrench[3] * frame_y[0] + wrench[4] * frame_y[1] + wrench[5] * frame_y[2];
    }
    double pull_n = 0.;
    if (pull_direction.has_value()) {
        const std::vector<double> &direction = *pull_direction;
        if (direction.size() != 3 ||
            !std::ranges::all_of(direction, [](double v) { return std::isfinite(v); }))
            return std::nullopt;
        const double f_norm = std::sqrt(
            f_rider[0] * f_rider[0] + f_rider[1] * f_rider[1] + f_rider[2] * f_rider[2]);
        if (kind == "grip") {
            pull_n = f_norm;
        } else {
            const double d_len = std::sqrt(
                direction[0] * direction[0] + direction[1] * direction[1] +
                direction[2] * direction[2]);
            if (d_len < 1e-12) return std::nullopt;
            const double dot_bike =
                (-f_rider[0] * direction[0] - f_rider[1] * direction[1] -
                 f_rider[2] * direction[2]) / d_len;
            if (dot_bike > 0.) pull_n = f_norm;
        }
    }
    return rider::AttachmentSample{.kind = kind,
                                   .normal_n = normal_n,
                                   .tangent_n = tangent_n,
                                   .moment_nm = moment_nm,
                                   .gap_m = gap_m,
                                   .pull_n = pull_n,
                                   .half_patch_m = half_patch_m};
}
} // namespace

AttachmentSamples evaluate_attachments(const RawStep &raw,
                                         rider::LeastSquaresWorkspace &workspace) {
    AttachmentSamples samples;
    const bool is_spatial = raw.qpos.size() != raw.qvel.size();
    for (const auto &[name, item] : raw.attachment_raw) {
        const auto rider_wrench = spatial_wrench(item.rider_jac, item.rider_qfrc, workspace);
        const auto bike_wrench = spatial_wrench(item.bike_jac, item.bike_qfrc, workspace);
        if (!rider_wrench.has_value() || !bike_wrench.has_value() || !item.observable)
            continue;
        const auto &wr = *rider_wrench;
        const auto &wb = *bike_wrench;
        const auto force = std::span<const double>(wr).first(3);
        const double scale = std::max(1., std::sqrt(sample_wire::dot(force, force)));
        if (is_spatial) {
            bool valid =
                std::ranges::all_of(wr, [](double v) { return std::isfinite(v); }) &&
                std::ranges::all_of(wb, [](double v) { return std::isfinite(v); });
            for (std::size_t i = 0; i < wr.size(); ++i)
                valid = valid &&
                        std::abs(wb[i] + wr[i]) <= 1e-3 * scale + 1e-3 * std::abs(wr[i]);
            if (valid) {
                if (auto sample = decompose_spatial_wrench(
                        wr, item.normal, item.kind, item.rotational,
                        item.half_patch_m, item.gap_m, item.pull_direction,
                        raw.qpos))
                    samples.emplace_back(name, std::move(*sample));
            }
        } else {
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
    if ((sample.kind == "foot" &&
         (sample.normal_n < budget.foot_min_normal_n ||
          (budget.foot_max_normal_n.has_value() &&
           sample.normal_n > *budget.foot_max_normal_n))) ||
        (sample.kind == "saddle" &&
         (sample.normal_n <= budget.saddle_min_normal_n ||
          (budget.saddle_max_normal_n.has_value() &&
           sample.normal_n > *budget.saddle_max_normal_n))))
        errors.emplace_back("normal");
    const double mu = sample.kind == "foot" ? budget.foot_mu : budget.saddle_mu;
    const double shear = sample.kind == "foot" ? budget.foot_shear_n : budget.saddle_shear_n;
    // Weld shear capacity plus whatever the friction cone admits while the
    // interface is compressive; a flat pad (shear 0) reduces to the cone.
    if (std::abs(sample.tangent_n) > shear + mu * std::max(0., sample.normal_n))
        errors.emplace_back("friction");
    if (std::abs(sample.moment_nm) > sample.half_patch_m * std::max(0., sample.normal_n))
        errors.emplace_back("cop");
    return errors;
}
} // namespace runtime
