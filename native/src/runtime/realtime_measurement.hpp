// Whole-boundary measurement (completion 3.04), ported from
// sim/realtime_measurement.py. The schema retains comparability.*, execution.*,
// prefix.*, chunk_latency.*, per_step_latency=null, warmups/runs, sampling and
// threshold classifications. The CLI also emits provenance.sanitizer_options.
// Boundary-call percentiles are never relabelled as per-physics-step latency.
#pragma once

#include "../io/value.hpp"
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace runtime::measurement {
inline constexpr double compute_slice_s = .008;
struct Boundary { std::int64_t step = 0; double time_s = 0.; };
struct Settings {
    std::string backend = "native";
    double timestep_s = 0.;
    std::int64_t max_steps = 0;
    int record_decimation = 80;
    bool strict = true;
    bike::io::Value metadata;
};

// A fresh owned driver is required for each warmup and measured repetition.
// This narrow interface also lets deterministic-clock tests establish exactly
// which work is inside each timing interval without running a simulation.
class Driver {
public:
    virtual ~Driver();
    [[nodiscard]] virtual const Settings& settings() const = 0;
    [[nodiscard]] virtual Boundary boundary() const = 0;
    [[nodiscard]] virtual std::optional<std::string> reason() const = 0;
    virtual void start() = 0;
    virtual void advance(std::int64_t target, double wall_budget_s) = 0;
    virtual void flush() = 0;
    [[nodiscard]] virtual bike::io::Value summary(std::string_view reason) const = 0;
    [[nodiscard]] virtual bike::io::Value execution_identity() const = 0;
    virtual void export_run(const std::filesystem::path&, std::string_view reason) = 0;
    virtual void close() = 0;
};
using Factory = std::function<std::unique_ptr<Driver>()>;
using Clock = std::function<double()>;
[[nodiscard]] double monotonic_clock();
[[nodiscard]] bike::io::Value comparison_key(const Settings&);
[[nodiscard]] bike::io::Value chunk_statistics(std::span<const double>);
[[nodiscard]] bike::io::Value measure_run(const Factory&,
    const std::optional<std::filesystem::path>& destination = {},
    const Clock& clock = monotonic_clock);
[[nodiscard]] bike::io::Value compare_runs(const bike::io::Value&, const bike::io::Value&);
[[nodiscard]] bike::io::Value build_report(const bike::io::Value::Array& warmups,
    const bike::io::Value::Array& runs);
[[nodiscard]] bike::io::Value steady_state_evidence(const bike::io::Value&,
    double minimum_simulated_time_s);
bike::io::Value steady_state_evidence(const bike::io::Value&, bool) = delete;
// Historical per-metric reports can still be inspected without inventing
// missing fields or treating a changed input/machine as comparable.
[[nodiscard]] bike::io::Value baseline_comparison(const bike::io::Value& current,
    const bike::io::Value& baseline, std::string_view path);
} // namespace runtime::measurement
