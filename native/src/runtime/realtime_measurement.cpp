#include "realtime_measurement.hpp"
#include "setup_values.hpp"
#include "status.hpp"
#include "spatial_measurement.hpp"
#include "../io/json.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <vector>

namespace runtime::measurement {
namespace {
using namespace setupvalue;
namespace io = bike::io;
double elapsed(double end, double start) {
    const double result = end - start;
    if (!std::isfinite(result) || result < 0.)
        throw std::invalid_argument("measurement clock must be finite and monotonic");
    return result;
}
bool true_value(const Value& value) {
    const auto* result = std::get_if<bool>(&value.data);
    return result != nullptr && *result;
}
Value optional_field(const Value& value, std::string_view key) {
    const auto* result = find(value, key);
    return result != nullptr ? *result : Value{};
}
void failure(Array& errors, std::string_view stage, const std::exception& error) {
    const auto* reference = dynamic_cast<const InvalidReferenceRun*>(&error);
    errors.push_back(Value{Object{{"stage", text(stage)},
        {"type", text(reference != nullptr ? "InvalidReferenceRun" : "RuntimeError")},
        {"message", text(error.what())}}});
}
Value measured(Driver& driver, double setup_start,
    const std::optional<std::filesystem::path>& destination, const Clock& clock) {
    const auto key = comparison_key(driver.settings());
    const auto initial = driver.boundary();
    const double setup_end = clock();
    Array errors, counts;
    std::vector<double> chunks;
    const double measured_start = clock();
    driver.start();
    try {
        while (!driver.reason()) {
            const auto before = driver.boundary();
            const double chunk_start = clock();
            std::exception_ptr advance_failure;
            try { driver.advance(driver.settings().max_steps, compute_slice_s); }
            catch (...) { advance_failure = std::current_exception(); }
            chunks.push_back(elapsed(clock(), chunk_start));
            counts.push_back(integer(driver.boundary().step - before.step));
            if (advance_failure) std::rethrow_exception(advance_failure);
        }
    } catch (const std::exception& error) { failure(errors, "advance", error); }
    const double flush_start = clock();
    try { driver.flush(); }
    catch (const std::exception& error) { failure(errors, "flush", error); }
    const double measured_end = clock();
    const double wall = elapsed(measured_end, measured_start);
    const auto frame = driver.boundary();
    const double simulated = frame.time_s - initial.time_s;
    const bool spatial_run = driver.settings().spatial_profile.has_value();
    if (spatial_run && (!std::isfinite(simulated) || simulated < 0. || wall <= 0.))
        throw std::invalid_argument("spatial measurement requires finite simulated time and positive elapsed wall time");
    std::string reason = driver.reason().value_or("");
    if (!errors.empty()) {
        reason = io::as_string(field(errors.front(), "type"), "error.type") == "InvalidReferenceRun"
            ? "invalid_controller" : "simulation_error";
    }
    const auto summary = driver.summary(reason);
    const auto& execution = field(driver.settings().metadata, "execution");
    bool identity_unchanged = false;
    bool identity_error = false;
    try { identity_unchanged = io::canonical_json(driver.execution_identity()) == io::canonical_json(execution); }
    catch (const std::exception& error) { failure(errors, "identity", error); identity_error = true; }
    if (!identity_unchanged && !identity_error) {
        errors.push_back(Value{Object{{"stage", text("identity")}, {"type", text("IdentityChanged")},
            {"message", text("source or artifact changed during measurement")}}});
    }
    double export_wall = 0.;
    if (destination) {
        const double export_start = clock();
        try { driver.export_run(*destination, reason); }
        catch (const std::exception& error) { failure(errors, "export", error); }
        export_wall = elapsed(clock(), export_start);
    }
    const auto& status = field(summary, "model_status");
    const auto steps = frame.step - initial.step;
    const auto requested_steps = driver.settings().max_steps - initial.step;
    const bool valid = true_value(field(status, "model_valid")) && true_value(field(status, "numerically_valid"));
    Value prefix{Object{{"steps", integer(steps)}, {"simulated_time_s", real(simulated)},
        {"requested_steps", integer(requested_steps)},
        {"fraction_of_requested_steps", requested_steps > 0 ? real(static_cast<double>(steps) / static_cast<double>(requested_steps)) : Value{}},
        {"reason", reason.empty() ? Value{} : text(reason)}, {"model_valid", field(status, "model_valid")},
        {"numerically_valid", field(status, "numerically_valid")},
        {"first_failure", optional_field(summary, "first_failure")},
        {"first_model_violation", optional_field(status, "first_model_violation")}}};
    Array limits;
    if (steps < requested_steps) limits.push_back(text("Natural/failed terminal prefix is shorter than requested; inspect before accepting throughput."));
    if (!valid) limits.push_back(text("Model/numerical validity is not established for this prefix."));
    if (!errors.empty()) limits.push_back(text("A failed run is evidence, not a successful performance measurement."));
    const bool measured_ok = errors.empty() && steps > 0 && wall > 0.;
    Value result{Object{{"backend", text(driver.settings().backend)}, {"comparability", key},
        {"execution", execution}, {"identity_unchanged", Value{identity_unchanged}},
        {"setup_wall_s", real(elapsed(setup_end, setup_start))}, {"wall_clock_s", real(wall)},
        {"final_flush_wall_s", real(elapsed(measured_end, flush_start))}, {"export_wall_s", real(export_wall)},
        {"simulated_time_s", real(simulated)}, {"physics_steps", integer(steps)},
        {"real_time_factor", wall > 0. ? real(simulated / wall) : Value{}},
        {"measured_scope", text("Native outer wall clock: all advance calls, boundary/loop overhead, and final flush")},
        {"chunk_latency", chunk_statistics(chunks)}, {"chunk_completed_steps", Value{std::move(counts)}},
        {"chunk_wall_s", reals(chunks)}, {"per_step_latency", Value{}},
        {"prefix", std::move(prefix)}, {"measured_ok", Value{measured_ok}},
        {"valid_prefix", Value{errors.empty() && steps > 0 && valid && identity_unchanged}},
        {"errors", Value{std::move(errors)}}, {"limitations", Value{std::move(limits)}}}};
    if (spatial_run) {
        const auto timings = driver.timings();
        if (!std::isfinite(timings.clock_overhead_estimate_s) || timings.clock_overhead_estimate_s < 0.)
            throw std::invalid_argument("spatial instrumentation overhead estimate must be finite and nonnegative");
        set(result, "profile", spatial_profile_value(*driver.settings().spatial_profile));
        set(result, "regions", region_timings_value(timings.regions));
        set(result, "instrumentation_available", Value{timings.available});
        set(result, "instrumentation_clock_reads", Value{io::Number{std::to_string(timings.clock_reads)}});
        set(result, "instrumentation_overhead_s", real(timings.clock_overhead_estimate_s));
        set(result, "instrumentation_overhead_kind", text("estimated clock-read cost; not subtracted from full wall time"));
        set(result, "instrumentation_total_overhead_s", Value{});
        set(result, "instrumentation_total_overhead_pending", Value{true});
        const bool complete = measured_ok && identity_unchanged && valid && driver.settings().strict &&
            steps == requested_steps && requested_steps > 0 &&
            (reason == "duration_reached" || reason == "requested_completion");
        set(result, "rtf", real(simulated / wall));
        set(result, "terminal_reason", text(reason));
        set(result, "steps", integer(steps));
        set(result, "valid", Value{complete});
        set(result, "classification", text(complete ? "valid_episode" : "diagnostic"));
        // The command/report builder assigns the actual warmup role.
        set(result, "warmup", Value{false});
    }
    return result;
}
} // namespace

Driver::~Driver() = default;
double monotonic_clock() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
bike::io::Value comparison_key(const Settings& settings) {
    using namespace setupvalue;
    const auto& metadata = settings.metadata;
    const auto& execution = field(metadata, "execution");
    // Preserve the historical key for consumers of schema 1. In a native
    // executable this is the compiled native digest, identified explicitly
    // by execution.source_identity_scope; no Python tree is inspected.
    const auto* source = find(execution, "python_source_sha256");
    if (source == nullptr) source = find(execution, "native_source_sha256");
    if (source == nullptr) throw std::invalid_argument("measurement execution has no source digest");
    Value result{Object{{"configuration_sha256", field(metadata, "configuration_sha256")},
        {"terrain_sha256", field(metadata, "terrain_sha256")}, {"python_source_sha256", *source},
        {"versions", field(metadata, "versions")}, {"timestep_s", real(settings.timestep_s)},
        {"internal_controller_period_s", field(metadata, "controller_interval_s")},
        {"external_policy_period_s", Value{}}, {"max_steps", integer(settings.max_steps)},
        {"record_decimation", integer(settings.record_decimation)}, {"strict", Value{settings.strict}},
        {"compute_slice_s", real(compute_slice_s)},
        {"controls", text("default RideControl; front/rear brake demand 0; profile-owned rider/assist")}}};
    if (settings.spatial_profile) {
        set(result, "spatial_profile", spatial_profile_value(*settings.spatial_profile));
        set(result, "terrain_kind", field(metadata, "terrain_kind"));
        set(result, "measurement_demand_sha256", optional_field(metadata, "measurement_demand_sha256"));
        set(result, "controls", optional_field(metadata, "measurement_controls"));
        set(result, "minimum_simulated_time_s", real(30.));
        set(result, "region_timing", optional_field(metadata, "region_timing"));
    }
    return result;
}
bike::io::Value chunk_statistics(std::span<const double> values) {
    using namespace setupvalue;
    std::vector<double> data(values.begin(), values.end());
    for (double const value : data) if (!std::isfinite(value) || value < 0.)
        throw std::invalid_argument("chunk latencies must be finite and nonnegative");
    std::ranges::sort(data);
    const auto quantile = [&](double fraction) -> Value {
        if (data.empty()) return {};
        const double index = static_cast<double>(data.size() - 1) * fraction;
        const auto lower = static_cast<std::size_t>(std::floor(index));
        const auto upper = static_cast<std::size_t>(std::ceil(index));
        return real(data.at(lower) + (data.at(upper) - data.at(lower)) * (index - static_cast<double>(lower)));
    };
    return Value{Object{{"count", integer(static_cast<std::int64_t>(data.size()))},
        {"units", text("seconds per advance call")}, {"min", data.empty() ? Value{} : real(data.front())},
        {"p50", quantile(.5)}, {"p95", quantile(.95)}, {"max", data.empty() ? Value{} : real(data.back())}}};
}
bike::io::Value measure_run(const Factory& factory,
    const std::optional<std::filesystem::path>& destination, const Clock& clock) {
    const double setup_start = clock();
    auto driver = factory();
    if (!driver) throw std::invalid_argument("measurement factory returned no driver");
    bike::io::Value result;
    try { result = measured(*driver, setup_start, destination, clock); }
    catch (...) { driver->close(); throw; }
    driver->close();
    return result;
}
} // namespace runtime::measurement
