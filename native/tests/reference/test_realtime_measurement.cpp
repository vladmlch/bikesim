#include "support/migration_test.hpp"
#include "../../src/runtime/realtime_measurement.hpp"
#include "../../src/runtime/realtime_runner.hpp"
#include "../../src/runtime/setup_values.hpp"
#include "../../src/io/json.hpp"
#include "../../src/cli/application.hpp"
#include <array>
#include <cmath>
#include <memory>

namespace {
using namespace runtime::setupvalue;
namespace m = runtime::measurement;
namespace io = bike::io;
struct State { double clock = 0.; bool closed = false, exported = false; bool throw_advance = false, changed_identity = false; };
class FixtureDriver final : public m::Driver {
public:
    explicit FixtureDriver(std::shared_ptr<State> state) : state_(std::move(state)) {
        settings_.backend = "python"; settings_.timestep_s = .00125; settings_.max_steps = 4;
        settings_.metadata = Value{Object{
            {"configuration_sha256", text("cfg")},
            {"terrain_sha256", text("terrain")},
            {"controller_interval_s", real(.0025)},
            {"versions", Value{Object{{"python", text("fixture")}}}},
            {"execution", Value{Object{
                {"python_source_sha256", text("source")},
                {"build_context", Value{}}
            }}}
        }};
    }
    const m::Settings& settings() const override { return settings_; }
    m::Boundary boundary() const override { return {step_, static_cast<double>(step_) * .00125}; }
    std::optional<std::string> reason() const override { return step_ == 4 ? std::optional<std::string>("duration_reached") : std::nullopt; }
    void start() override { state_->clock += .5; }
    void advance(std::int64_t target, double budget) override {
        bt::require(target == 4 && budget == .008, "measurement uses the full target and the shared 8 ms boundary budget");
        state_->clock += 2.; step_ += 2;
        if (state_->throw_advance) throw std::runtime_error("fixture advance failure");
    }
    void flush() override { state_->clock += 1.; }
    Value summary(std::string_view) const override {
        return Value{Object{
            {"model_status", Value{Object{
                {"model_valid", Value{true}},
                {"numerically_valid", Value{true}}
            }}}
        }};
    }
    Value execution_identity() const override { auto value = field(settings_.metadata, "execution"); if (state_->changed_identity) set(value, "python_source_sha256", text("changed")); return value; }
    void export_run(const std::filesystem::path&, std::string_view) override { state_->clock += 3.; state_->exported = true; }
    void close() override { state_->closed = true; }
private:
    std::shared_ptr<State> state_; m::Settings settings_; std::int64_t step_ = 0;
};
Value sample(const std::shared_ptr<State>& state) {
    return m::measure_run([&] { state->clock += 2.; return std::make_unique<FixtureDriver>(state); },
        std::filesystem::path("not-written-by-fixture"), [&] { return state->clock; });
}
Value sample() { return sample(std::make_shared<State>()); }
double number(const Value& value, std::string_view key) { return io::as_double(field(value, key), key); }
bool boolean(const Value& value, std::string_view key) { return io::as_bool(field(value, key), key); }

// tests/reference/test_realtime_measurement.py::test_whole_loop_includes_boundary_and_final_flush_but_not_setup_export
void test_whole_loop_includes_boundary_and_final_flush_but_not_setup_export() {
    auto state = std::make_shared<State>(); const auto run = sample(state);
    bt::require(number(run, "setup_wall_s") == 2. && number(run, "wall_clock_s") == 5.5 && number(run, "final_flush_wall_s") == 1. && number(run, "export_wall_s") == 3., "setup/export are excluded and boundary/flush are included");
    bt::require(io::canonical_json(field(run, "chunk_wall_s")) == "[2.0,2.0]", "the two measured advance calls retain their actual costs");
    bt::require(io::canonical_json(field(run, "chunk_completed_steps")) == "[2,2]", "chunk step counts describe the committed prefix");
    bt::require(number(run, "real_time_factor") == .005 / 5.5 && number(field(run, "chunk_latency"), "p95") == 2., "whole-loop RTF and boundary p95 use different measured quantities");
    bt::require(io::is_null(field(run, "per_step_latency")) && boolean(run, "valid_prefix") && state->closed && state->exported, "no per-step percentile is inferred; driver is exported and closed");
}
// tests/reference/test_realtime_measurement.py::test_different_numerical_or_recording_settings_are_not_comparable
void test_different_numerical_or_recording_settings_are_not_comparable() {
    const auto run = sample();
    const std::array cases{std::pair{"timestep_s", real(.0005)}, std::pair{"configuration_sha256", text("other")},
        std::pair{"record_decimation", integer(1)}, std::pair{"terrain_sha256", text("other")}, std::pair{"strict", Value{false}}};
    for (const auto& [key, value] : cases) {
        auto changed = run; auto settings = field(changed, "comparability"); set(settings, key, value); set(changed, "comparability", std::move(settings));
        const auto check = m::compare_runs(run, changed);
        bt::require(!boolean(check, "comparable") && io::canonical_json(field(check, "differing_settings")) == "[\"" + std::string(key) + "\"]", "every changed shared numerical setting prevents comparison");
    }
}
// tests/reference/test_realtime_measurement.py::test_different_outcome_and_short_prefix_remain_visible
void test_different_outcome_and_short_prefix_remain_visible() {
    const auto run = sample(); auto other = run; auto prefix = field(other, "prefix");
    set(prefix, "steps", integer(2)); set(prefix, "reason", text("crash")); set(prefix, "simulated_time_s", real(.0025)); set(other, "prefix", prefix);
    const auto check = m::compare_runs(run, other);
    bt::require(!boolean(check, "comparable") && boolean(check, "short_prefix"), "a faster shorter trajectory is exposed as a different prefix");
    const auto aggregate = m::build_report({other}, {run, run, run});
    bt::require(number(aggregate, "warmup_count") == 1. && number(aggregate, "measurement_count") == 3. && number(aggregate, "median_real_time_factor") == number(run, "real_time_factor"), "warmups do not enter the measured median");
    bt::require(!boolean(aggregate, "full_acceptance_verified"), "a throughput aggregate never establishes full acceptance");
}
// tests/reference/test_realtime_measurement.py::test_empty_chunk_statistics_are_not_invented
void test_empty_chunk_statistics_are_not_invented() {
    bt::require(io::is_null(field(m::chunk_statistics({}), "p95")), "empty measurements have no invented percentile");
    const std::array costs{1., 3., 2.}; const auto statistics = m::chunk_statistics(costs);
    bt::require(number(statistics, "p50") == 2. && number(statistics, "p95") == 2.9, "quantiles use linear interpolation over ordered measured calls");
    const std::array negative{-1.}; bt::invalid([&] { (void)m::chunk_statistics(negative); });
}
// tests/reference/test_realtime_measurement.py::test_native_threshold_requires_warmup_and_three_valid_repetitions
void test_native_threshold_requires_warmup_and_three_valid_repetitions() {
    auto run = sample(); set(run, "backend", text("native")); set(run, "real_time_factor", real(2.));
    auto execution = field(run, "execution"); set(execution, "build_context", Value{Object{{"build_type", text("Release")}}}); set(run, "execution", execution);
    const auto insufficient = m::build_report({}, {run});
    bt::require(!boolean(insufficient, "sampling_protocol_satisfied") && io::is_null(field(insufficient, "headless_rtf_threshold_met")), "one unwarmed run cannot establish the native threshold");
    const auto enough = m::build_report({run}, {run, run, run});
    bt::require(boolean(enough, "sampling_protocol_satisfied") && boolean(enough, "headless_rtf_threshold_met") && !boolean(enough, "full_acceptance_verified"), "three valid Release repetitions with a matching warmup establish only headless throughput");
    bt::invalid([] { (void)m::build_report({}, {}); });
}
// tests/reference/test_realtime_measurement.py::test_measurement_cli_keeps_profile_dt_until_explicit_override
void test_measurement_cli_keeps_profile_dt_until_explicit_override() {
    const std::array<std::string_view, 2> defaults{"--out", "unused"};
    const auto options = m::parse_options(defaults);
    bt::require(!options.timestep_s, "profile dt is retained until an explicit override");
    const auto resolved = m::resolve_measurement_ride(options);
    bt::require(resolved->setup.model.physics.has_value() && resolved->setup.model.physics->timestep_s == .00125, "actual profile projection retains the shipped 1.25 ms dt");
    const std::array<std::string_view, 4> override{"--out", "unused", "--dt", ".0005"};
    const auto explicit_options = m::parse_options(override);
    bt::require(explicit_options.timestep_s == .0005 && m::resolve_measurement_ride(explicit_options)->setup.model.physics->timestep_s == .0005, "the explicit dt reaches actual profile resolution");
}
// tests/reference/test_realtime_measurement.py::test_benchmark_mode_relaxes_only_the_monitor_and_marks_the_report
void test_benchmark_mode_relaxes_only_the_monitor_and_marks_the_report() {
    const std::array<std::string_view, 2> arguments{"--out", "unused"};
    auto options = m::parse_options(arguments);
    const auto strict_ride = m::resolve_measurement_ride(options);
    const auto strict = m::annotate_report(Value{Object{}}, options, .00125);
    options.benchmark = true; const auto benchmark = m::annotate_report(Value{Object{}}, options, .00125);
    const auto benchmark_ride = m::resolve_measurement_ride(options);
    bt::require(strict_ride->setup.runtime.strict && !benchmark_ride->setup.runtime.strict, "runtime strictness follows benchmark mode");
    bt::require(strict_ride->arguments.values == benchmark_ride->arguments.values && strict_ride->arguments.supplied == benchmark_ride->arguments.supplied, "benchmark never changes projected ride arguments");
    bt::require(io::canonical_json(bike::configuration::to_value(*strict_ride->setup.model.physics)) == io::canonical_json(bike::configuration::to_value(*benchmark_ride->setup.model.physics)), "benchmark never changes resolved physical inputs");
    bt::require(boolean(field(strict, "invocation"), "monitor_strict") && !boolean(field(benchmark, "invocation"), "monitor_strict"), "benchmark changes the monitor stop policy");
    bt::require(!find(strict, "limitations") && io::canonical_json(field(benchmark, "limitations")).find("never a valid-ride claim") != std::string::npos, "diagnostic benchmark reports cannot be mistaken for valid rides");
}
void failure_keeps_prefix_and_closes_owner() {
    auto state = std::make_shared<State>(); state->throw_advance = true; const auto run = sample(state);
    bt::require(!boolean(run, "measured_ok") && !boolean(run, "valid_prefix") && number(run, "physics_steps") == 2. && state->closed && state->exported, "advance failure preserves and exports the completed prefix and always closes");
    state = std::make_shared<State>(); state->changed_identity = true; const auto changed = sample(state);
    bt::require(!boolean(changed, "identity_unchanged") && !boolean(changed, "valid_prefix"), "changed source/artifact identity prevents a valid prefix claim");
}
} // namespace
int main() { return bt::run([] {
    test_whole_loop_includes_boundary_and_final_flush_but_not_setup_export();
    test_different_numerical_or_recording_settings_are_not_comparable();
    test_different_outcome_and_short_prefix_remain_visible();
    test_empty_chunk_statistics_are_not_invented();
    test_native_threshold_requires_warmup_and_three_valid_repetitions();
    test_measurement_cli_keeps_profile_dt_until_explicit_override();
    test_benchmark_mode_relaxes_only_the_monitor_and_marks_the_report();
    failure_keeps_prefix_and_closes_owner();
}); }
