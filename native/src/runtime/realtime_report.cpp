#include "realtime_measurement.hpp"
#include "setup_values.hpp"
#include "spatial_measurement.hpp"
#include "../spatial/bootstrap_state.hpp"
#include "../io/json.hpp"
#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
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
    return real(values.size() % 2 != 0 ? values.at(index) : std::midpoint(values.at(index - 1), values.at(index)));
}
std::uint64_t unsigned_value(const Value& object, std::string_view name, std::uint64_t maximum) {
    const auto& spelling = io::as_number(field(object, name), name).lexeme;
    std::uint64_t value = 0;
    // to_address(end) rather than data() + size(): -Wunsafe-buffer-usage is
    // an error under the native warning policy.
    const char* const end = std::to_address(spelling.cend());
    const auto parsed = std::from_chars(std::to_address(spelling.cbegin()), end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end || value > maximum)
        throw std::invalid_argument("spatial report." + std::string(name) + ": unsigned integer out of range");
    return value;
}
void validate_spatial_profile(const Value& profile) {
    spatial::SpatialPerformanceProfile decoded;
    for (const auto& [name, destination] : std::array{
        std::pair{"id", &decoded.id}, std::pair{"source_sha", &decoded.source_sha},
        std::pair{"config_sha256", &decoded.config_sha256}, std::pair{"calibration_sha256", &decoded.calibration_sha256},
        std::pair{"provider_identity", &decoded.provider_identity}, std::pair{"sensor_mode", &decoded.sensor_mode},
        std::pair{"recording_mode", &decoded.recording_mode}})
        *destination = io::as_string(field(profile, name), name);
    const auto root = bike::spatial::parse_root_kind(io::as_string(field(profile, "root"), "profile.root"));
    const auto tier = bike::spatial::parse_tyre_tier(io::as_string(field(profile, "tier"), "profile.tier"));
    if (!root || !tier) throw std::invalid_argument("spatial.profile: unknown root or tyre tier");
    decoded.root = *root; decoded.tier = *tier;
    decoded.grid_nx = unsigned_value(profile, "grid_nx", std::numeric_limits<std::uint64_t>::max());
    decoded.grid_ny = unsigned_value(profile, "grid_ny", std::numeric_limits<std::uint64_t>::max());
    decoded.dt_s = number(profile, "dt_s");
    decoded.processes = static_cast<unsigned>(unsigned_value(profile, "processes", std::numeric_limits<unsigned>::max()));
    decoded.tyre_threads = static_cast<unsigned>(unsigned_value(profile, "tyre_threads", std::numeric_limits<unsigned>::max()));
    decoded.engine_threads = static_cast<unsigned>(unsigned_value(profile, "engine_threads", std::numeric_limits<unsigned>::max()));
    // Loaded reports must satisfy the same identity/domain checks as the
    // live producer; equality of two incomplete profiles is not evidence.
    (void)spatial_profile_value(decoded);
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
    if (find(left, "profile") != nullptr || find(right, "profile") != nullptr) {
        // Spatial baseline comparison includes the actual build/provider
        // identity, not merely identical source/configuration fingerprints.
        for (const std::string_view name : {"backend", "profile", "execution"})
            if (!equal(get(left, name), get(right, name))) differences.push_back(text(name));
    }
    const auto& a = field(left, "prefix");
    const auto& b = field(right, "prefix");
    for (std::string_view const name : {"reason", "model_valid", "numerically_valid", "steps"})
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
    if (find(runs.front(), "profile") != nullptr) return build_spatial_report(warmups, runs);
    for (const auto& run : runs) if (find(run, "profile") != nullptr)
        throw std::invalid_argument("measurement report cannot mix spatial and planar runs");
    for (const auto& run : warmups) if (find(run, "profile") != nullptr)
        throw std::invalid_argument("measurement report cannot mix spatial and planar warmups");
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

bike::io::Value build_spatial_report(const bike::io::Value::Array& warmups,
                                    const bike::io::Value::Array& runs) {
    if (runs.empty()) throw std::invalid_argument("spatial report requires at least one measured run");
    const auto& first = runs.front();
    const auto& profile = field(first, "profile");
    (void)io::as_object(profile, "profile");
    validate_spatial_profile(profile);
    const auto& reference_key = field(first, "comparability");
    const auto& reference_execution = field(first, "execution");
    if (!equal(field(reference_key, "spatial_profile"), profile))
        throw std::invalid_argument("spatial report profile differs from its comparison identity");
    if (!equal(field(reference_key, "timestep_s"), field(profile, "dt_s")) ||
        !equal(field(reference_key, "configuration_sha256"), field(profile, "config_sha256")))
        throw std::invalid_argument("spatial report profile timestep/config differs from run identity");
    const double timestep = number(profile, "dt_s");
    if (!std::isfinite(timestep) || timestep <= 0.)
        throw std::invalid_argument("spatial report timestep must be finite and positive");
    if (!equal(field(profile, "source_sha"), field(reference_execution, "native_source_sha256")))
        throw std::invalid_argument("spatial report source differs from execution identity");
    const bool fast = equal(field(profile, "id"), text("spatial-fast-v1"));
    if (fast && (!equal(field(profile, "root"), text("free")) ||
                 !equal(field(profile, "tier"), text("fast"))))
        throw std::invalid_argument("spatial-fast-v1 report requires free root and fast tyre identity");

    constexpr std::array<std::string_view, 8> region_names{
        "terrain_rays_s", "tyre_elements_s", "engine_step_s", "rider_steer_s",
        "accounting_s", "sensors_s", "recording_s", "ui_s"};
    constexpr std::array<std::string_view, 3> pending_names{
        "sensors_pending", "recording_pending", "ui_pending"};
    std::array<double, 8> region_means{};
    std::array<bool, 3> pending{};
    std::vector<double> factors;
    Array measured_rows, warmup_rows, pairs;
    bool all_valid = true, valid_warmups = !warmups.empty(), duration_protocol = true;
    bool instrumentation_available = true, any_short = false, repeated_comparable = true;
    double overhead_mean = 0.;

    const auto inspect = [&](const Value& run, bool warmup) {
        if (!equal(field(run, "backend"), field(first, "backend")) ||
            !equal(field(run, "profile"), profile) ||
            !equal(field(run, "comparability"), reference_key) ||
            !equal(field(run, "execution"), reference_execution))
            throw std::invalid_argument("spatial report run identity mismatch (including warmups)");
        const double wall = number(run, "wall_clock_s");
        const double simulated = number(run, "simulated_time_s");
        (void)spatial::rtf_threshold_met(simulated, wall, 2.);
        const double factor = simulated / wall;
        if (!std::isfinite(factor) || !equal(field(run, "real_time_factor"), real(factor)))
            throw std::invalid_argument("spatial report real_time_factor does not match finite sim/wall clocks");
        if (!io::is_null(field(run, "per_step_latency")) ||
            !equal(field(field(run, "chunk_latency"), "units"), text("seconds per advance call")))
            throw std::invalid_argument("spatial report chunk latency cannot be labeled per-step latency");
        const auto& regions = field(run, "regions");
        std::array<double, 8> times{};
        double region_sum = 0.;
        for (std::size_t i = 0; i < region_names.size(); ++i) {
            times[i] = number(regions, region_names[i]);
            if (!std::isfinite(times[i]) || times[i] < 0.)
                throw std::invalid_argument("spatial.regions." + std::string(region_names[i]) + ": invalid time");
            region_sum += times[i];
        }
        // The owner collector pauses a parent scope while a child runs.
        // These exclusive regions are all inside the measured wall window;
        // impossible or overflowing evidence cannot support a timing claim.
        if (!std::isfinite(region_sum) || (region_sum > wall && !isclose(region_sum, wall)))
            throw std::invalid_argument("spatial report exclusive region times exceed full wall time");
        for (const auto name : pending_names)
            if (!std::holds_alternative<bool>(field(regions, name).data))
                throw std::invalid_argument("spatial.regions." + std::string(name) + ": boolean required");
        const double overhead = number(run, "instrumentation_overhead_s");
        if (!std::isfinite(overhead) || overhead < 0.)
            throw std::invalid_argument("spatial report instrumentation overhead is invalid");
        const auto clock_reads = unsigned_value(run, "instrumentation_clock_reads", std::numeric_limits<std::uint64_t>::max());
        const bool timing_observed = io::as_bool(field(run, "instrumentation_available"), "instrumentation_available");
        if (timing_observed && clock_reads == 0)
            throw std::invalid_argument("spatial report available instrumentation requires observed clock reads");
        const auto& prefix = field(run, "prefix");
        const auto requested = count(prefix, "requested_steps");
        const auto completed = count(prefix, "steps");
        const double requested_time = static_cast<double>(requested) * timestep;
        const double completed_time = static_cast<double>(completed) * timestep;
        if (!equal(field(prefix, "simulated_time_s"), field(run, "simulated_time_s")) ||
            requested != count(reference_key, "max_steps") ||
            completed != count(run, "physics_steps") || requested <= 0 ||
            completed < 0 || completed > requested || !std::isfinite(requested_time) ||
            !isclose(simulated, completed_time))
            throw std::invalid_argument("spatial report prefix clock/step counts contradict the declared run");
        const bool complete = completed == requested &&
            count(run, "physics_steps") == requested &&
            (equal(field(prefix, "reason"), text("duration_reached")) ||
             equal(field(prefix, "reason"), text(bike::spatial::to_string(bike::spatial::SpatialReason::requested_completion))));
        const bool valid = complete && yes(field(run, "measured_ok")) && yes(field(run, "valid_prefix")) &&
            yes(field(run, "identity_unchanged")) && yes(field(reference_key, "strict")) &&
            yes(field(prefix, "model_valid")) && yes(field(prefix, "numerically_valid")) &&
            io::as_array(field(run, "errors"), "errors").empty();
        Value row = run;
        set(row, "rtf", real(factor)); set(row, "terminal_reason", field(prefix, "reason"));
        set(row, "steps", field(run, "physics_steps")); set(row, "valid", Value{valid});
        set(row, "warmup", Value{warmup});
        set(row, "classification", text(valid ? "valid_episode" : "diagnostic"));
        // MuJoCo accumulates double time at each tick. Use the declared tick
        // window for the 30 s protocol after checking its agreement with the
        // actual clock; RTF itself always uses the unrounded observed time.
        const bool full_window = complete && requested_time >= 30.;
        if (warmup) {
            valid_warmups = valid_warmups && valid && full_window &&
                yes(field(compare_runs(first, run), "comparable"));
            warmup_rows.push_back(std::move(row));
        } else {
            all_valid = all_valid && valid;
            any_short = any_short || !complete;
            duration_protocol = duration_protocol && full_window;
            instrumentation_available = instrumentation_available && timing_observed;
            if (valid) {
                factors.push_back(factor);
                const double samples = static_cast<double>(factors.size());
                // Nonnegative finite observations have a finite mean even
                // when their raw sum over several repetitions overflows.
                for (std::size_t i = 0; i < times.size(); ++i)
                    region_means[i] += (times[i] - region_means[i]) / samples;
                for (std::size_t i = 0; i < pending.size(); ++i) pending[i] = pending[i] || yes(field(regions, pending_names[i]));
                overhead_mean += (overhead - overhead_mean) / samples;
            }
            measured_rows.push_back(std::move(row));
        }
    };
    for (const auto& warmup : warmups) inspect(warmup, true);
    for (const auto& run : runs) inspect(run, false);
    for (std::size_t i = 1; i < runs.size(); ++i) {
        auto comparison = compare_runs(first, runs[i]);
        repeated_comparable = repeated_comparable && yes(field(comparison, "comparable"));
        pairs.push_back(std::move(comparison));
    }
    const auto accepted = factors.size();
    const auto median_factor = median(std::move(factors));
    Object averaged_regions;
    for (std::size_t i = 0; i < region_names.size(); ++i)
        averaged_regions.emplace_back(region_names[i], accepted == 0 ? Value{} : real(region_means[i]));
    for (std::size_t i = 0; i < pending_names.size(); ++i)
        averaged_regions.emplace_back(pending_names[i], Value{accepted == 0 || pending[i]});
    const bool protocol = !warmups.empty() && runs.size() >= 3 && duration_protocol;
    const auto context = get(reference_execution, "build_context");
    const bool release = !io::is_null(context) && equal(get(context, "build_type"), text("Release"));
    const bool native = equal(field(first, "backend"), text("native"));
    const bool eligible = fast && protocol && valid_warmups && all_valid && repeated_comparable &&
        accepted == runs.size() && instrumentation_available && release && native;
    const Value target = eligible ? Value{spatial::rtf_threshold_met(io::as_double(median_factor, "median_rtf"), 1., 2.)} : Value{};
    return Value{Object{{"schema_version", integer(1)}, {"spatial_performance_schema", integer(1)},
        {"backend", field(first, "backend")}, {"profile", profile},
        {"warmup_count", integer(static_cast<std::int64_t>(warmups.size()))},
        {"measurement_count", integer(static_cast<std::int64_t>(runs.size()))},
        {"valid_measurement_count", integer(static_cast<std::int64_t>(accepted))},
        {"warmups", Value{std::move(warmup_rows)}}, {"runs", Value{std::move(measured_rows)}},
        {"median_rtf", median_factor}, {"median_real_time_factor", median_factor},
        {"target_rtf", fast ? real(2.) : Value{}}, {"target_met", target}, {"headless_rtf_threshold_met", target},
        {"regions", Value{std::move(averaged_regions)}},
        {"regions_aggregation", text("mean full-run seconds over valid measured episodes; warmups excluded")},
        {"instrumentation_overhead_s", accepted == 0 ? Value{} : real(overhead_mean)},
        {"instrumentation_overhead_kind", text("estimated clock-read cost; included in measured wall time")},
        {"instrumentation_total_overhead_s", Value{}}, {"instrumentation_total_overhead_pending", Value{true}},
        {"instrumentation_available", Value{instrumentation_available}}, {"sampling_protocol_satisfied", Value{protocol}},
        {"same_identity", Value{true}}, {"repeated_runs_comparable", Value{repeated_comparable}},
        {"repeated_run_comparisons", Value{std::move(pairs)}},
        {"short_prefix_review_required", Value{any_short}}, {"full_acceptance_verified", Value{false}},
        {"pending_gates", Value{Array{text("V09 physical fixture, invariance and sanitizer evidence"),
            text("V10 both-platform installed product, recording and real-display 1x RTF >= 1.0"),
            text("instrumented versus uninstrumented full-run overhead")}}},
        {"scope", text("Strict complete native episodes only; diagnostic and warmup runs excluded from median; no qualification assertion")}}};
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
    if (!equal(field(prefix, "reason"), text("duration_reached")) &&
        !(find(run, "profile") != nullptr && equal(field(prefix, "reason"),
            text(bike::spatial::to_string(bike::spatial::SpatialReason::requested_completion)))))
        reasons.push_back(text("non_duration_terminal"));
    if (find(run, "profile") != nullptr && !yes(field(field(run, "comparability"), "strict")))
        reasons.push_back(text("non_strict_diagnostic"));
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
