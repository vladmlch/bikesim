// Completion 3.26 (batch runtime). Source: tests/reference/
// test_native_runtime_performance_contract.py — C4 performance-acceptance
// contract gates (10 records, all ported): the threshold constants as gate
// behavior, failed runs measured as ineligible evidence, native/Release
// scoping of the headless gate, the pending visual-1x gate, the
// distributed-tyre case sharing the same constant, and selected-artifact
// identity binding.
//
// All checks are native-vs-native contract tests over the ported
// measurement/report boundary (runtime::measurement): deterministic fake
// drivers + clocks for measured shape, synthesized report Values for the
// aggregate classification, and runtime_identity()/native_provenance()
// for the compiled-in artifact identity — the native counterpart of
// execution_provenance('native', artifact=...). No Python tree or golden
// is consulted.
#include "support/migration_test.hpp"

#include "../../src/io/json.hpp"
#include "../../src/runtime/provenance.hpp"
#include "../../src/runtime/realtime_measurement.hpp"
#include "../../src/runtime/realtime_runner.hpp"
#include "../../src/runtime/setup_values.hpp"

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

using namespace runtime::setupvalue;
namespace m = runtime::measurement;
namespace io = bike::io;

constexpr double kUnpacedMedianRtfMin = 1.0;
constexpr double kVisual1xMeanRtfMin = 0.95;

struct State { double clock = 0.; bool closed = false; };

// _FailingDriver: setup succeeds; the first measured advance raises while
// the model status stays valid — ineligibility must come from the failure
// alone, never a model-status flag.
class FailingDriver final : public m::Driver {
public:
    explicit FailingDriver(std::shared_ptr<State> state) : state_(std::move(state)) {
        settings_.backend = "python";
        settings_.timestep_s = .00125;
        settings_.max_steps = 4;
        settings_.record_decimation = 80;
        settings_.strict = true;
        settings_.metadata = Value{Object{
            {"configuration_sha256", text("cfg")},
            {"terrain_sha256", text("terrain")},
            {"controller_interval_s", real(.0025)},
            {"versions", Value{Object{{"python", text("fixture")}}}},
            {"execution", Value{Object{
                {"python_source_sha256", text("source")},
                {"build_context", Value{}}}}}}};
    }
    const m::Settings& settings() const override { return settings_; }
    m::Boundary boundary() const override { return {.step = step_, .time_s = time_s_}; }
    std::optional<std::string> reason() const override { return reason_; }
    void start() override {}
    [[noreturn]] void advance(std::int64_t, double) override {
        throw std::runtime_error("engine exploded mid-interval");
    }
    void flush() override {}
    Value summary(std::string_view reason) const override {
      return Value{Object{
          {"outcome", Value{Object{{"reason", text(reason)}}}},
          {"model_status", Value{Object{{"model_valid", Value{true}},
                                        {"numerically_valid", Value{true}}}}}}};
    }
    Value execution_identity() const override {
        return field(settings_.metadata, "execution");
    }
    void export_run(const std::filesystem::path&, std::string_view) override {}
    void close() override { state_->closed = true; }
private:
    std::shared_ptr<State> state_;
    m::Settings settings_;
    std::int64_t step_ = 0;
    double time_s_ = 0.;
    std::optional<std::string> reason_;
};

// _MeasuredDriver: the loop completes and the metadata carries the real
// native execution identity a NativeRideDriver would.
class MeasuredDriver final : public m::Driver {
public:
    MeasuredDriver(std::shared_ptr<State> state, Value execution)
        : state_(std::move(state)) {
        settings_.backend = "native";
        settings_.timestep_s = .00125;
        settings_.max_steps = 4;
        settings_.record_decimation = 80;
        settings_.strict = true;
        settings_.metadata = Value{Object{
            {"configuration_sha256", text("cfg")},
            {"terrain_sha256", text("terrain")},
            {"controller_interval_s", real(.0025)},
            {"versions", Value{Object{{"python", text("fixture")}}}},
            {"execution", std::move(execution)}}};
    }
    const m::Settings& settings() const override { return settings_; }
    m::Boundary boundary() const override { return {.step = step_, .time_s = time_s_}; }
    std::optional<std::string> reason() const override { return reason_; }
    void start() override { state_->clock += .5; }
    void advance(std::int64_t target, double) override {
        bt::require(target == settings_.max_steps,
                    "advance is driven to the full requested target");
        state_->clock += 2.;
        step_ = settings_.max_steps;
        time_s_ = static_cast<double>(step_) * settings_.timestep_s;
        reason_ = "duration_reached";
    }
    void flush() override { state_->clock += 1.; }
    Value summary(std::string_view) const override {
        return Value{Object{
            {"model_status", Value{Object{
                {"model_valid", Value{true}},
                {"numerically_valid", Value{true}}}}}}};
    }
    Value execution_identity() const override {
        return field(settings_.metadata, "execution");
    }
    void export_run(const std::filesystem::path&, std::string_view) override {}
    void close() override { state_->closed = true; }
private:
    std::shared_ptr<State> state_;
    m::Settings settings_;
    std::int64_t step_ = 0;
    double time_s_ = 0.;
    std::optional<std::string> reason_;
};

std::pair<std::shared_ptr<State>, Value> failing_report() {
    auto state = std::make_shared<State>();
    Value report = m::measure_run(
        [&] { return std::make_unique<FailingDriver>(state); },
        std::nullopt, [&] { return state->clock; });
    return {state, std::move(report)};
}

// _native_report: a valid native Release measurement Value for
// build_report.
Value native_report(double real_time_factor) {
    return Value{Object{
        {"backend", text("native")},
        {"comparability", Value{Object{
            {"timestep_s", real(.00125)},
            {"record_decimation", integer(80)}}}},
        {"execution", Value{Object{
            {"build_context", Value{Object{
                {"build_type", text("Release")}}}}}}},
        {"measured_ok", Value{true}},
        {"valid_prefix", Value{true}},
        {"real_time_factor", real(real_time_factor)},
        {"prefix", Value{Object{
            {"steps", integer(800)},
            {"requested_steps", integer(800)},
            {"reason", text("duration")},
            {"model_valid", Value{true}},
            {"numerically_valid", Value{true}},
            {"simulated_time_s", real(1.)}}}}}};
}

// _distributed_report: the alpine_rocky / viewer_physics_distributed
// measurement identity, annotated the way tools/measure_realtime.py's
// annotate_report writes it. valid=false reproduces the benchmark-mode
// shape: the welded rider's strength envelope trips inside the first
// 35 ms, model_valid latches false on a full 30 s prefix.
Value distributed_report(double real_time_factor, bool valid) {
    Value report = native_report(real_time_factor);
    Value comparability = field(report, "comparability");
    set(comparability, "configuration_sha256",
        text("distributed-128station-alpine-rocky"));
    set(comparability, "terrain_sha256", text("alpine-rocky-hfield"));
    set(comparability, "max_steps", integer(24000));
    set(comparability, "strict", Value{valid});
    set(report, "comparability", std::move(comparability));
    set(report, "invocation", Value{Object{
        {"physics_config", text("examples/research/viewer_physics_distributed.toml")},
        {"track", text("examples/research/alpine_rocky.toml")},
        {"duration_s", real(30.)},
        {"dt_override", Value{}},
        {"effective_dt_s", real(.00125)},
        {"record_decimation", integer(80)},
        {"monitor_strict", Value{valid}},
        {"benchmark_mode", Value{!valid}}}});
    set(report, "benchmark_mode", Value{!valid});
    Value prefix = field(report, "prefix");
    set(prefix, "steps", integer(24000));
    set(prefix, "requested_steps", integer(24000));
    set(prefix, "simulated_time_s", real(30.));
    if (!valid) {
        set(report, "valid_prefix", Value{false});
        set(report, "limitations", Value{Array{text(
            "Benchmark mode: the reference monitor was non-strict; the "
            "measured prefix is a throughput sample of a ride whose "
            "validity is not established, never a valid-ride claim.")}});
        set(prefix, "model_valid", Value{false});
        set(prefix, "first_failure", Value{Object{
            {"interval_id", integer(27)},
            {"time_s", real(.03375)},
            {"reasons", Value{Array{
                text("rider_strength.rider_elbow_left"),
                text("rider_strength.rider_elbow_right")}}}}});
    }
    set(report, "prefix", std::move(prefix));
    return report;
}

// _aggregate: one warmup plus three identical-classification measurements.
Value aggregate(const Value& report) {
    return m::build_report(Value::Array{report},
                           Value::Array{report, report, report});
}

double number(const Value& v, std::string_view key) {
    return io::as_double(field(v, key), key);
}
bool boolean(const Value& v, std::string_view key) {
    return io::as_bool(field(v, key), key);
}
// f'1x mean achieved RTF >= {VISUAL_1X_MEAN_RTF_MIN}' — the pending-gate
// needle formatted from the named constant.
[[nodiscard]] std::string visual_gate_needle() {
    char buffer[32]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    std::snprintf(buffer, sizeof(buffer), "1x mean achieved RTF >= %g", // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                  kVisual1xMeanRtfMin);
    return buffer; // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
}
std::string joined_gates(const Value& v) {
    std::string out;
    for (const auto& gate : io::as_array(field(v, "pending_gates"), "pending_gates")) {
        if (!out.empty()) out += ' ';
        out += io::as_string(gate, "pending_gates[]");
    }
    return out;
}
bool contains_fold(const Value& v, std::string_view key, std::string_view needle) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    const auto& entries = io::as_array(field(v, key), key);
    for (const auto& entry : entries) {
      std::string const line = io::as_string(entry, "limitations[]");
      std::string lowered;
      lowered.reserve(line.size());
      for (char const c : line)
        lowered += static_cast<char>(std::tolower(c));
      if (lowered.find(needle) != std::string::npos)
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// test_selected_native_artifact_loads — the compiled-in identity resolves,
// doubling as the loaded-engine check the Python loader performed.
// ---------------------------------------------------------------------------
void test_selected_native_artifact_loads() {
    const runtime::RuntimeIdentity identity = runtime::runtime_identity();
    bt::require(!identity.mujoco_version.empty() &&
                    !identity.native_source_sha256.empty(),
                "the selected native artifact resolves its compiled identity");
}

// ---------------------------------------------------------------------------
// test_failed_run_is_measured_not_ok_and_short
// ---------------------------------------------------------------------------
void test_failed_run_is_measured_not_ok_and_short() {
    const auto [state, report] = failing_report();
    bt::require(state->closed, "the failed driver is still closed");
    bt::require(!boolean(report, "measured_ok") && !boolean(report, "valid_prefix"),
                "a failed run is measured as ineligible evidence");
    const auto& errors = io::as_array(field(report, "errors"), "errors");
    bt::require(!errors.empty() &&
                    io::as_string(field(errors.front(), "stage"), "errors[0].stage") ==
                        "advance",
                "the first failure is staged at advance");
    const auto& prefix = field(report, "prefix");
    bt::require(io::as_string(field(prefix, "reason"), "prefix.reason") ==
                    "simulation_error",
                "a throwing engine classifies as simulation_error");
    bt::require(io::as_int64(field(prefix, "steps"), "prefix.steps") <
                    io::as_int64(field(prefix, "requested_steps"),
                                 "prefix.requested_steps"),
                "the failed prefix is shorter than requested");
    bt::require(contains_fold(report, "limitations", "failed run is evidence"),
                "the limitations name the failed run as evidence");
}

// ---------------------------------------------------------------------------
// test_failed_runs_are_never_comparable
// ---------------------------------------------------------------------------
void test_failed_runs_are_never_comparable() {
    auto [state, report] = failing_report();
    const Value check = m::compare_runs(report, report);
    bt::require(!boolean(check, "comparable"),
                "identical failed runs refuse comparison on measured_ok alone");
    bt::require(io::as_array(field(check, "differing_settings"), "differing_settings")
                    .empty() &&
                io::as_array(field(check, "differing_prefix_fields"),
                             "differing_prefix_fields")
                    .empty(),
                "refusal comes from measured_ok, not settings or prefix");
    set(report, "backend", text("native"));
    set(report, "execution", Value{Object{
        {"build_context", Value{Object{{"build_type", text("Release")}}}}}});
    const Value aggregate_report = aggregate(report);
    bt::require(boolean(aggregate_report, "sampling_protocol_satisfied"),
                "the full warmup + 3x protocol is met");
    bt::require(io::is_null(field(aggregate_report, "headless_rtf_threshold_met")),
                "every run invalid leaves the gate undecided, never failed");
}

// ---------------------------------------------------------------------------
// test_headless_threshold_boundary_is_exactly_the_named_constant
// ---------------------------------------------------------------------------
void test_headless_threshold_boundary_is_exactly_the_named_constant() {
    const Value below = aggregate(native_report(.5));
    bt::require(std::abs(number(below, "median_real_time_factor") - .5) < 1e-12,
                "the median is the measured factor");
    bt::require(!io::is_null(field(below, "headless_rtf_threshold_met")) &&
                    !boolean(below, "headless_rtf_threshold_met"),
                "eligible and below the gate evaluates false");
    const Value at = aggregate(native_report(kUnpacedMedianRtfMin));
    bt::require(std::abs(number(at, "median_real_time_factor") - 1.) < 1e-12 &&
                    boolean(at, "headless_rtf_threshold_met"),
                "the >= boundary is inclusive at the named constant");
}

// ---------------------------------------------------------------------------
// test_threshold_gate_only_applies_to_native_release
// ---------------------------------------------------------------------------
void test_threshold_gate_only_applies_to_native_release() {
    Value python_run = native_report(2.);
    set(python_run, "backend", text("python"));
    bt::require(io::is_null(field(aggregate(python_run),
                                "headless_rtf_threshold_met")),
                "a python-backend run is undecided, never failed");
    Value debug = native_report(2.);
    set(debug, "execution", Value{Object{
        {"build_context", Value{Object{{"build_type", text("Debug")}}}}}});
    bt::require(io::is_null(field(aggregate(debug), "headless_rtf_threshold_met")),
                "a non-Release build is undecided, never failed");
}

// ---------------------------------------------------------------------------
// test_visual_1x_gate_is_emitted_as_a_pending_gate
// ---------------------------------------------------------------------------
void test_visual_1x_gate_is_emitted_as_a_pending_gate() {
    const Value report = aggregate(native_report(2.));
    bt::require(joined_gates(report).find(visual_gate_needle()) !=
                    std::string::npos,
                "the 1x visual gate is emitted with its named constant");
    bt::require(!boolean(report, "full_acceptance_verified"),
                "a headless report keeps full acceptance pending");
}

// ---------------------------------------------------------------------------
// test_distributed_case_gates_on_the_same_headless_constant
// ---------------------------------------------------------------------------
void test_distributed_case_gates_on_the_same_headless_constant() {
    const Value below = aggregate(distributed_report(.5, true));
    bt::require(!io::is_null(field(below, "headless_rtf_threshold_met")) &&
                    !boolean(below, "headless_rtf_threshold_met"),
                "distributed below the shared floor evaluates false");
    const Value at = aggregate(distributed_report(kUnpacedMedianRtfMin, true));
    bt::require(std::abs(number(at, "median_real_time_factor") -
                         kUnpacedMedianRtfMin) < 1e-12 &&
                    boolean(at, "headless_rtf_threshold_met"),
                "the distributed case meets the identical inclusive floor");
    bt::require(joined_gates(at).find(visual_gate_needle()) !=
                    std::string::npos,
                "the distributed case emits the same pending visual gate");
}

// ---------------------------------------------------------------------------
// test_distributed_benchmark_sample_reports_but_never_meets_the_gate
// ---------------------------------------------------------------------------
void test_distributed_benchmark_sample_reports_but_never_meets_the_gate() {
    const Value report = aggregate(distributed_report(2., false));
    bt::require(std::abs(number(report, "median_real_time_factor") - 2.) < 1e-12,
                "the benchmark sample still reports an honest median RTF");
    bt::require(boolean(report, "sampling_protocol_satisfied"),
                "the full sampling protocol is met");
    for (const auto& run : io::as_array(field(report, "runs"), "runs"))
        bt::require(!boolean(run, "valid_prefix"),
                    "every benchmark run keeps valid_prefix false");
    bt::require(!boolean(report, "short_prefix_review_required"),
                "full-duration runs are not a short-prefix case");
    bt::require(io::is_null(field(report, "headless_rtf_threshold_met")),
                "a latched model violation keeps the gate undecided");
    bt::require(!boolean(report, "full_acceptance_verified"),
                "full acceptance is never claimed for a benchmark sample");
}

// ---------------------------------------------------------------------------
// test_measure_run_binds_the_selected_artifact_identity — the driver's
// execution document is the compiled provenance; identity_unchanged then
// reads the real build context.
// ---------------------------------------------------------------------------
void test_measure_run_binds_the_selected_artifact_identity() {
    const Value execution = m::native_provenance();
    auto state = std::make_shared<State>();
    const Value report = m::measure_run(
        [&] { return std::make_unique<MeasuredDriver>(state, execution); },
        std::nullopt, [&] { return state->clock; });
    bt::require(boolean(report, "measured_ok") && boolean(report, "valid_prefix"),
                "the measured driver produces a valid prefix");
    bt::require(boolean(report, "identity_unchanged"),
                "the bound identity is unchanged across the measurement");
    bt::require(io::canonical_json(field(report, "execution")) ==
                    io::canonical_json(execution),
                "the report carries the driver's execution document verbatim");
    const runtime::RuntimeIdentity identity = runtime::runtime_identity();
    bt::require(io::as_string(
                    field(field(report, "execution"), "native_source_sha256"),
                    "execution.native_source_sha256") ==
                    identity.native_source_sha256,
                "the bound provenance is the selected artifact's source digest");
}

// ---------------------------------------------------------------------------
// test_threshold_gate_tracks_the_selected_artifact_build_type — the gate
// keys off the compiled build_context, not a fabricated Release flag.
// ---------------------------------------------------------------------------
void test_threshold_gate_tracks_the_selected_artifact_build_type() {
    const Value execution = m::native_provenance();
    Value report = native_report(kUnpacedMedianRtfMin);
    set(report, "execution", execution);
    const Value aggregate_report = aggregate(report);
    const bool is_release =
        io::as_string(field(field(execution, "build_context"), "build_type"),
                      "build_context.build_type") == "Release";
    if (is_release)
        bt::require(boolean(aggregate_report, "headless_rtf_threshold_met"),
                    "a Release artifact meets the gate at the constant");
    else
        bt::require(io::is_null(field(aggregate_report,
                                      "headless_rtf_threshold_met")),
                    "a non-Release artifact stays undecided");
}

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(test_selected_native_artifact_loads);
        failures += bt::run(test_failed_run_is_measured_not_ok_and_short);
        failures += bt::run(test_failed_runs_are_never_comparable);
        failures += bt::run(
            test_headless_threshold_boundary_is_exactly_the_named_constant);
        failures += bt::run(test_threshold_gate_only_applies_to_native_release);
        failures += bt::run(test_visual_1x_gate_is_emitted_as_a_pending_gate);
        failures += bt::run(
            test_distributed_case_gates_on_the_same_headless_constant);
        failures += bt::run(
            test_distributed_benchmark_sample_reports_but_never_meets_the_gate);
        failures += bt::run(test_measure_run_binds_the_selected_artifact_identity);
        failures += bt::run(
            test_threshold_gate_tracks_the_selected_artifact_build_type);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
