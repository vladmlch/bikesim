#include "status.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string_view>

namespace runtime {
namespace {
using namespace sample_wire;

const WireObject &section_or_empty(const WireObject &value, std::string_view key) {
    static const WireObject empty;
    const Wire *found = find(value, key);
    return found == nullptr ? empty : object(*found);
}

bool flag(const WireObject &value, std::string_view key) {
    const Wire *found = find(value, key);
    return found != nullptr && boolean(*found);
}

double real_or(const WireObject &value, std::string_view key, double fallback) {
    const Wire *found = find(value, key);
    if (found == nullptr) return fallback;
    if (const auto *real = std::get_if<double>(&found->value)) return *real;
    if (const auto *whole = std::get_if<std::int64_t>(&found->value))
        return static_cast<double>(*whole);
    throw std::invalid_argument("model status: expected numeric " + std::string(key));
}

void threshold(double value, std::string_view path) {
    if (!std::isfinite(value) || value < 0.)
        throw std::invalid_argument(std::string(path) + ": must be finite and nonnegative");
}

std::vector<std::string> string_list(const Wire &value) {
    std::vector<std::string> result;
    for (const auto &item : array(value)) result.push_back(string(item));
    return result;
}

} // namespace

Wire ReferenceFailure::as_wire() const {
    return Wire(WireArray{Wire(time_s), sample_wire::strings(violations)});
}

std::string ReferenceFailure::message() const {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << "invalid reference step at " << std::fixed << std::setprecision(9)
           << time_s << ": (";
    for (std::size_t i = 0; i < violations.size(); ++i) {
        if (i != 0) stream << ", ";
        stream << '\'' << violations[i] << '\'';
    }
    if (violations.size() == 1) stream << ',';
    stream << ')';
    return stream.str();
}

InvalidReferenceRun::~InvalidReferenceRun() = default;

void ReferenceMonitor::observe(const ReferenceFailure &failure) {
    if (failure.violations.empty()) return;
    if (!std::isfinite(failure.time_s) || failure.time_s < 0.)
        throw std::invalid_argument("reference failure time must be finite and nonnegative");
    if (!first_failure.has_value()) {
        first_failure = failure;
        warning_pending = !strict;
    }
}

std::optional<std::string> ReferenceMonitor::take_warning() {
    if (!warning_pending || !first_failure.has_value()) return std::nullopt;
    std::string message = first_failure->message();
    warning_pending = false;
    return message;
}

WireObject ReferenceMonitor::as_wire() const {
    return {{"strict", Wire(strict)},
            {"first_failure", first_failure.has_value() ? first_failure->as_wire() : Wire(nullptr)}};
}

ReferenceMonitor ReferenceMonitor::from_wire(const WireObject &value) {
    constexpr std::array<std::string_view, 2> keys{"strict", "first_failure"};
    exact(value, keys, "state.runtime.monitor");
    ReferenceMonitor result;
    result.strict = boolean(required(value, "strict"));
    const Wire &first = required(value, "first_failure");
    if (!is_none(first)) {
        const auto &items = array(first);
        if (items.size() != 2)
            throw std::invalid_argument("state.runtime.monitor.first_failure: expected time and violations");
        ReferenceFailure failure{.time_s = number(items[0]), .violations = string_list(items[1])};
        threshold(failure.time_s, "state.runtime.monitor.first_failure.time_s");
        if (failure.violations.empty())
            throw std::invalid_argument("state.runtime.monitor.first_failure: empty violations");
        result.first_failure = std::move(failure);
    }
    return result;
}

std::vector<std::string> channel_violations(const WireObject &channels,
                                           double maximum_compression_fraction,
                                           double maximum_linkage_error_m) {
    threshold(maximum_compression_fraction, "maximum_compression_fraction");
    threshold(maximum_linkage_error_m, "maximum_linkage_error_m");
    std::vector<std::string> reasons;
    const auto &tires = section_or_empty(channels, "tires");
    for (const auto side : {"front", "rear"}) {
        const auto &tire = section_or_empty(tires, side);
        const std::string prefix = std::string(side) + ':';
        if (flag(tire, "multi_support") && !flag(tire, "supports_multiple_contacts"))
            reasons.push_back(prefix + "multi_support");
        if (real_or(tire, "normal_load_n", 0.) > 0. && flag(tire, "outside_material_load_range"))
            reasons.push_back(prefix + "material_load_range");
        if (flag(tire, "outside_material_deflection_range"))
            reasons.push_back(prefix + "material_deflection_range");
        const Wire *radius_value = find(tire, "unloaded_radius_m");
        const double radius = radius_value == nullptr || is_none(*radius_value)
                                  ? 0. : real_or(tire, "unloaded_radius_m", 0.);
        if (!std::isfinite(radius) || radius <= 0.)
            reasons.push_back(prefix + "missing_radius");
        else if (real_or(tire, "penetration_m", 0.) > radius * maximum_compression_fraction)
            reasons.push_back(prefix + "tire_compression");
        if (const Wire *patches = find(tire, "patches")) {
            for (const auto &item : array(*patches)) {
                const auto &patch = object(item);
                const Wire *source = find(patch, "source_geom");
                if (source != nullptr && string(*source) == "catch_plane" &&
                    real_or(patch, "normal_load_n", 0.) > 0.) {
                    reasons.push_back(prefix + "catch_plane");
                    break;
                }
            }
        }
        if (flag(tire, "outside_profile_domain")) reasons.push_back(prefix + "profile_domain");
    }
    const double error = real_or(section_or_empty(channels, "suspension"),
                                 "linkage_closure_max_m", 0.);
    if (!std::isfinite(error) || error > maximum_linkage_error_m)
        reasons.emplace_back("linkage:closure_error");
    if (const Wire *violations = find(channels, "attachment_violations")) {
        const auto extra = string_list(*violations);
        reasons.insert(reasons.end(), extra.begin(), extra.end());
    }
    return reasons;
}

bool energy_quality_ok(const WireObject &energy) {
    const auto read = [&](std::string_view key, double fallback) {
        const Wire *value = find(energy, key);
        return value == nullptr ? fallback : number(*value);
    };
    double scale = read("energy_scale_j", 1.);
    threshold(scale, "initial energy scale");
    scale += std::abs(read("external_work_j", 0.));
    scale += std::abs(find(energy, "source_positive_work_j") != nullptr
                          ? read("source_positive_work_j", 0.) : read("active_work_j", 0.));
    if (scale == 0.) throw std::invalid_argument("energy quality requires a nonzero scale");
    const double ratio = std::abs(read("residual_j", 0.)) / scale;
    const double electrical = read("electrical_residual_j", 0.);
    const double budget = std::max(1., std::abs(read("electrical_work_j", 0.)));
    return ratio <= .05 && std::abs(electrical) <= 1e-7 * budget;
}

void ModelStatus::observe(std::int64_t interval_id, double time_s,
                          const WireObject &channels) {
    if (interval_id <= last_interval || !std::isfinite(time_s))
        throw std::invalid_argument("model status requires one advancing finite interval");
    const auto reasons = channel_violations(channels, maximum_compression_fraction,
                                            maximum_linkage_error_m);
    auto next_counts = counts;
    auto next_first = first;
    Wire next_numerical = numerically_valid;
    for (const auto &reason : reasons) {
        auto found = std::ranges::find_if(next_counts, [&](const auto &entry) {
            return entry.first == reason;
        });
        if (found == next_counts.end()) next_counts.emplace_back(reason, 1);
        else {
            if (found->second == std::numeric_limits<std::int64_t>::max())
                throw std::overflow_error("model violation count overflow");
            ++found->second;
        }
    }
    if (!reasons.empty() && !next_first.has_value())
        next_first = WireObject{{"interval_id", Wire(interval_id)}, {"time_s", Wire(time_s)},
                                {"reasons", sample_wire::strings(reasons)}};
    if (const Wire *energy = find(channels, "energy")) {
        const bool was_false = std::holds_alternative<bool>(numerically_valid.value) &&
                               !boolean(numerically_valid);
        next_numerical = Wire(energy_quality_ok(object(*energy)) && !was_false);
    }
    counts = std::move(next_counts);
    first = std::move(next_first);
    numerically_valid = std::move(next_numerical);
    last_interval = interval_id;
}

WireObject ModelStatus::as_wire() const {
    WireObject counters;
    for (const auto &[name, count] : counts) counters.emplace_back(name, Wire(count));
    return {{"model_valid", Wire(counts.empty())},
            {"first_model_violation", first.has_value() ? Wire(*first) : Wire(nullptr)},
            {"model_violation_counts", Wire(std::move(counters))},
            {"numerically_valid", numerically_valid},
            {"calibration_status", Wire(calibration_status)}};
}

ModelStatus ModelStatus::from_wire(const WireObject &value) {
    constexpr std::array<std::string_view, 7> keys{
        "counts", "first", "last_interval", "maximum_compression_fraction",
        "maximum_linkage_error_m", "numerically_valid", "calibration_status"};
    exact(value, keys, "state.runtime.model_status");
    ModelStatus result;
    for (const auto &[key, item] : object(required(value, "counts"))) {
        const auto count = integer(item);
        if (count < 0) throw std::invalid_argument("state.runtime.model_status.counts must be nonnegative");
        result.counts.emplace_back(key, count);
    }
    result.last_interval = integer(required(value, "last_interval"));
    if (result.last_interval < -1)
        throw std::invalid_argument("state.runtime.model_status.last_interval must be at least -1");
    const Wire &first_value = required(value, "first");
    if (!is_none(first_value)) {
        result.first = object(first_value);
        constexpr std::array<std::string_view, 3> first_keys{"interval_id", "time_s", "reasons"};
        exact(*result.first, first_keys, "state.runtime.model_status.first");
        const auto first_id = integer(required(*result.first, "interval_id"));
        if (first_id < 0 || first_id > result.last_interval)
            throw std::invalid_argument("state.runtime.model_status.first.interval_id is inconsistent");
        threshold(number(required(*result.first, "time_s")), "state.runtime.model_status.first.time_s");
        if (string_list(required(*result.first, "reasons")).empty())
            throw std::invalid_argument("state.runtime.model_status.first.reasons must not be empty");
    }
    result.maximum_compression_fraction = number(required(value, "maximum_compression_fraction"));
    result.maximum_linkage_error_m = number(required(value, "maximum_linkage_error_m"));
    threshold(result.maximum_compression_fraction, "state.runtime.model_status.maximum_compression_fraction");
    threshold(result.maximum_linkage_error_m, "state.runtime.model_status.maximum_linkage_error_m");
    result.numerically_valid = required(value, "numerically_valid");
    if (!std::holds_alternative<bool>(result.numerically_valid.value) &&
        (!std::holds_alternative<std::string>(result.numerically_valid.value) ||
         string(result.numerically_valid) != "not_evaluated"))
        throw std::invalid_argument("state.runtime.model_status.numerically_valid: invalid state");
    result.calibration_status = string(required(value, "calibration_status"));
    return result;
}

} // namespace runtime
