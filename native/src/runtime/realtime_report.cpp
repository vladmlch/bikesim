#include "realtime_measurement.hpp"
#include "setup_values.hpp"
#include "../io/json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

namespace runtime::measurement {
namespace {
using namespace setupvalue;
namespace io = bike::io;
bool yes(const Value& value) {
    const auto* boolean = std::get_if<bool>(&value.data);
    return boolean != nullptr && *boolean;
}
Value get(const Value& object, std::string_view name) {
    const auto* value = find(object, name);
    return value != nullptr ? *value : Value{};
}
bool equal(const Value& left, const Value& right) {
    return io::canonical_json(left) == io::canonical_json(right);
}
double number(const Value& object, std::string_view name) {
    return io::as_double(field(object, name), name);
}
std::int64_t count(const Value& object, std::string_view name) {
    return io::as_int64(field(object, name), name);
}
bool isclose(double a, double b) {
    return std::isfinite(a) && std::isfinite(b) &&
        (a == b || std::abs(a - b) <= std::max(1e-9, 1e-9 * std::max(std::abs(a), std::abs(b))));
}
bool short_prefix(const Value& run) {
    const auto& prefix = field(run, "prefix");
    return count(prefix, "steps") < count(prefix, "requested_steps");
}
Value median(std::vector<double> values) {
    if (values.empty()) return {};
    std::ranges::sort(values);
    const auto index = values.size() / 2;
    return real(values.size() % 2 != 0 ? values.at(index) : (values.at(index - 1) + values.at(index)) / 2.);
}
} // namespace

bike::io::Value compare_runs(const bike::io::Value& left, const bike::io::Value& right) {
    const auto& left_key = field(left, "comparability");
    const auto& right_key = field(right, "comparability");
    std::set<std::string> keys;
    for (const auto& [name, ignored] : io::as_object(left_key, "comparability")) { (void)ignored; keys.insert(name); }
    for (const auto& [name, ignored] : io::as_object(right_key, "comparability")) { (void)ignored; keys.insert(name); }
    Array differences, classifications;
    for (const auto& name : keys) if (!equal(get(left_key, name), get(right_key, name))) differences.push_back(text(name));
    const auto& a = field(left, "prefix");
    const auto& b = field(right, "prefix");
    for (std::string_view name : {"reason", "model_valid", "numerically_valid", "steps"})
        if (!equal(field(a, name), field(b, name))) classifications.push_back(text(name));
    if (!isclose(number(a, "simulated_time_s"), number(b, "simulated_time_s")))
        classifications.push_back(text("simulated_time_s"));
    const bool comparable = differences.empty() && classifications.empty() && yes(field(left, "measured_ok")) && yes(field(right, "measured_ok"));
    const bool valid = comparable && yes(field(left, "valid_prefix")) && yes(field(right, "valid_prefix"));
    return Value{Object{{"comparable", Value{comparable}}, {"valid_prefix_comparison", Value{valid}},
        {"differing_settings", Value{std::move(differences)}}, {"differing_prefix_fields", Value{std::move(classifications)}},
        {"classification", text(valid ? "same_valid_prefix" : comparable ? "equally_classified_diagnostic_prefix" : "not_comparable")},
        {"short_prefix", Value{short_prefix(left) || short_prefix(right)}},
        {"scope", text("Performance comparability only; not a numerical parity or physical calibration check.")}}};
}

bike::io::Value build_report(const bike::io::Value::Array& warmups, const bike::io::Value::Array& runs) {
    if (runs.empty()) throw std::invalid_argument("at least one measured run is required");
    std::vector<double> factors;
    bool valid_runs = true, any_short = false, same = true, valid_warmups = true;
    for (const auto& run : runs) {
        const auto& factor = field(run, "real_time_factor");
        if (!io::is_null(factor)) factors.push_back(io::as_double(factor, "real_time_factor"));
        valid_runs = valid_runs && yes(field(run, "valid_prefix"));
        any_short = any_short || short_prefix(run);
    }
    Array pairs, warmup_checks;
    for (std::size_t i = 1; i < runs.size(); ++i) {
        auto check = compare_runs(runs.front(), runs.at(i));
        same = same && yes(field(check, "comparable"));
        pairs.push_back(std::move(check));
    }
    for (const auto& warmup : warmups) {
        auto check = compare_runs(runs.front(), warmup);
        valid_warmups = valid_warmups && yes(field(check, "valid_prefix_comparison"));
        warmup_checks.push_back(std::move(check));
    }
    const bool protocol = !warmups.empty() && runs.size() >= 3;
    const bool eligible = same && valid_runs && factors.size() == runs.size() && protocol && valid_warmups;
    const auto median_factor = median(std::move(factors));
    const auto context = get(field(runs.front(), "execution"), "build_context");
    const bool release = !io::is_null(context) && equal(get(context, "build_type"), text("Release"));
    const bool native = equal(field(runs.front(), "backend"), text("native"));
    return Value{Object{{"schema_version", integer(1)}, {"backend", field(runs.front(), "backend")},
        {"warmup_count", integer(static_cast<std::int64_t>(warmups.size()))},
        {"measurement_count", integer(static_cast<std::int64_t>(runs.size()))},
        {"warmups", Value{warmups}}, {"runs", Value{runs}}, {"median_real_time_factor", median_factor},
        {"repeated_runs_comparable", Value{same}}, {"repeated_run_comparisons", Value{std::move(pairs)}},
        {"sampling_protocol_satisfied", Value{protocol}}, {"warmup_comparisons", Value{std::move(warmup_checks)}},
        {"headless_rtf_threshold_met", eligible && release && native ? Value{io::as_double(median_factor, "median") >= 1.} : Value{}},
        {"short_prefix_review_required", Value{any_short}}, {"full_acceptance_verified", Value{false}},
        {"pending_gates", Value{Array{text("same-prefix Python/native comparison"), text("full correctness and sanitizer gates"),
            text("actual macOS viewer and 1x mean achieved RTF >= 0.95")}}},
        {"scope", text("Measured headless runtime only; warm-ups excluded from median; no GUI or calibration claim.")}}};
}

bike::io::Value steady_state_evidence(const bike::io::Value& run, double minimum_simulated_time_s) {
    if (!std::isfinite(minimum_simulated_time_s) || minimum_simulated_time_s <= 0.)
        throw std::invalid_argument("minimum_simulated_time_s must be finite and positive");
    const auto& prefix = field(run, "prefix");
    Array reasons;
    if (!yes(field(run, "measured_ok")) || !io::as_array(field(run, "errors"), "errors").empty()) reasons.push_back(text("measurement_failed"));
    if (!yes(field(run, "valid_prefix")) || !yes(field(prefix, "model_valid")) || !yes(field(prefix, "numerically_valid"))) reasons.push_back(text("invalid_prefix"));
    if (!yes(field(run, "identity_unchanged"))) reasons.push_back(text("identity_changed"));
    if (count(prefix, "requested_steps") <= 0 || count(prefix, "steps") != count(prefix, "requested_steps") || count(run, "physics_steps") != count(prefix, "steps"))
        reasons.push_back(text("incomplete_requested_prefix"));
    if (!equal(field(prefix, "reason"), text("duration_reached"))) reasons.push_back(text("non_duration_terminal"));
    // Diagnostic documents may contain the same tagged nonfinites supported
    // by the native JSON boundary; such values never support a timing claim.
    const auto finite_or_nan = [](const Value& value) {
        if (const auto* n = std::get_if<io::Number>(&value.data)) {
            bool ok = false; const double result = io::detail::float_from_lexeme(n->lexeme, ok);
            if (ok) return result;
        }
        return std::numeric_limits<double>::quiet_NaN();
    };
    const double actual = finite_or_nan(field(run, "simulated_time_s"));
    if (!std::isfinite(actual) || actual <= 0. || (actual < minimum_simulated_time_s && !isclose(actual, minimum_simulated_time_s))) reasons.push_back(text("insufficient_simulated_window"));
    const double wall = finite_or_nan(field(run, "wall_clock_s"));
    if (!std::isfinite(wall) || wall <= 0.) reasons.push_back(text("invalid_wall_clock"));
    const bool diagnostic = !reasons.empty();
    return Value{Object{{"status", text(diagnostic ? "diagnostic" : "eligible")}, {"reasons", Value{std::move(reasons)}}}};
}

bike::io::Value baseline_comparison(const bike::io::Value& current, const bike::io::Value& baseline, std::string_view path) {
    (void)io::as_object(current, "current report");
    if (!std::holds_alternative<Object>(baseline.data))
        throw std::invalid_argument("baseline report is not a JSON object: " + std::string(path));
    Object metrics, comparison;
    for (std::string_view key : {"factor", "step_ms_mean", "step_ms_p50", "step_ms_p95", "step_ms_p99", "wall_seconds", "steps", "sim_seconds"}) {
        const auto a = get(current, key), b = get(baseline, key);
        Value entry{Object{{"current", a}, {"baseline", b}}};
        if (std::holds_alternative<io::Number>(a.data) && std::holds_alternative<io::Number>(b.data)) {
            const double value = io::as_double(a, key), reference = io::as_double(b, key);
            set(entry, "delta", real(value - reference));
            if (reference != 0.) set(entry, "delta_pct", real((value - reference) / std::abs(reference) * 100.));
        }
        metrics.emplace_back(key, std::move(entry));
    }
    for (std::string_view key : {"git_sha", "track", "physics", "machine", "effective_timestep_s", "requested_duration_s", "record_decimation"}) {
        const auto a = get(current, key), b = get(baseline, key);
        comparison.emplace_back(key, Value{Object{{"current", a}, {"baseline", b}, {"match", Value{equal(a, b)}}}});
    }
    return Value{Object{{"path", text(path)}, {"metrics", Value{std::move(metrics)}}, {"comparability", Value{std::move(comparison)}}}};
}
} // namespace runtime::measurement
