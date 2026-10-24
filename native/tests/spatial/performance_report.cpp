// W50: synthetic controlled-clock contracts, not measured performance evidence.
#include "support/migration_test.hpp"
#include "runtime/realtime_runner.hpp"
#include "runtime/spatial_measurement.hpp"
#include "runtime/setup_values.hpp"
#include "cli/application.hpp"
#include "io/json.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <memory>

namespace {
namespace measure = runtime::measurement;
namespace io = bike::io;
using namespace runtime::setupvalue;
struct Fixture {
    double clock = 0., advance_s = 14., flush_s = 1.;
    bool fall = false, strict = true, closed = false, exported = false;
};
class ClockDriver final : public measure::Driver {
public:
    explicit ClockDriver(std::shared_ptr<Fixture> fixture) : fixture_(std::move(fixture)) {
        settings_.timestep_s = .00125; settings_.max_steps = 24000;
        settings_.strict = fixture_->strict;
        runtime::spatial::SpatialPerformanceProfile profile;
        profile.root = bike::spatial::RootKind::free6; profile.tier = bike::spatial::TyreTier::fast;
        profile.grid_nx = 401; profile.grid_ny = 25; profile.dt_s = settings_.timestep_s;
        profile.sensor_mode = "none"; profile.recording_mode = "off";
        profile.source_sha = std::string(64, '1'); profile.config_sha256 = std::string(64, '2');
        profile.calibration_sha256 = std::string(64, '3'); profile.provider_identity = "synthetic-controlled-clock";
        settings_.spatial_profile = profile;
        settings_.metadata = Value{Object{{"configuration_sha256", text(profile.config_sha256)},
            {"terrain_sha256", text(std::string(64, '4'))}, {"controller_interval_s", real(.005)},
            {"terrain_kind", text("grid")}, {"measurement_controls", text("synthetic fixture")},
            {"versions", Value{Object{{"engine", text("synthetic")}}}},
            {"execution", Value{Object{{"native_source_sha256", text(profile.source_sha)},
                {"build_context", Value{Object{{"build_type", text("Release")}}}}}}}};
    }
    const measure::Settings& settings() const override { return settings_; }
    measure::Boundary boundary() const override { return {.step = step_, .time_s = static_cast<double>(step_) * settings_.timestep_s}; }
    std::optional<std::string> reason() const override {
        if (step_ == 0) return {};
        return fixture_->fall ? "capsize" : "requested_completion";
    }
    void start() override {}
    void advance(std::int64_t target, double budget) override {
        bt::require(target == settings_.max_steps && budget == measure::compute_slice_s,
            "driver receives a full target with an advance-call budget");
        fixture_->clock += fixture_->advance_s;
        step_ = fixture_->fall ? target / 2 : target;
    }
    void flush() override { fixture_->clock += fixture_->flush_s; }
    Value summary(std::string_view) const override {
        return Value{Object{{"model_status", Value{Object{
            {"model_valid", Value{true}}, {"numerically_valid", Value{true}}}}}};
    }
    Value execution_identity() const override { return field(settings_.metadata, "execution"); }
    measure::MeasurementTimings timings() const override {
        return {.regions = {.terrain_rays_s = .5, .tyre_elements_s = 1., .engine_step_s = 1.,
            .rider_steer_s = .25, .accounting_s = .25},
            .clock_overhead_estimate_s = .001, .clock_reads = 100, .available = true};
    }
    void export_run(const std::filesystem::path&, std::string_view) override {
        fixture_->clock += 3.; fixture_->exported = true;
    }
    void close() override { fixture_->closed = true; }
private:
    std::shared_ptr<Fixture> fixture_;
    measure::Settings settings_;
    std::int64_t step_ = 0;
};
Value sample(std::shared_ptr<Fixture> fixture = std::make_shared<Fixture>()) {
    return measure::measure_run([&] {
        fixture->clock += 2.;
        return std::make_unique<ClockDriver>(fixture);
    }, std::filesystem::path("not-written-by-synthetic-spatial-driver"), [&] { return fixture->clock; });
}
double number(const Value& object, std::string_view key) { return io::as_double(field(object, key), key); }
bool yes(const Value& object, std::string_view key) { return io::as_bool(field(object, key), key); }
void thresholds() {
    using runtime::spatial::rtf_threshold_met;
    bt::require(rtf_threshold_met(30., 15., 2.), "headless target includes exact 2x boundary");
    bt::require(!rtf_threshold_met(30., 15.1, 2.), "threshold has no favorable rounding");
    bt::require(!rtf_threshold_met(0., 1., 2.), "zero simulated time is admissible but misses target");
    bt::invalid([] { (void)rtf_threshold_met(30., 0., 2.); });
    bt::invalid([] { (void)rtf_threshold_met(-1., 1., 2.); });
    bt::invalid([] { (void)rtf_threshold_met(1., 1., 0.); });
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        bt::invalid([&] { (void)rtf_threshold_met(invalid, 1., 2.); });
        bt::invalid([&] { (void)rtf_threshold_met(1., invalid, 2.); });
        bt::invalid([&] { (void)rtf_threshold_met(1., 1., invalid); });
    }
}
void controlled_wall_and_report() {
    auto fixture = std::make_shared<Fixture>(); const auto run = sample(fixture);
    bt::require(number(run, "setup_wall_s") == 2. && number(run, "wall_clock_s") == 15. &&
        number(run, "final_flush_wall_s") == 1. && number(run, "export_wall_s") == 3.,
        "whole interval includes final flush, excludes preparation and export");
    bt::require(number(run, "rtf") == 2. && fixture->closed && fixture->exported,
        "controlled clocks establish exact report arithmetic and owner teardown");
    const auto report = measure::build_report({run}, {run, run, run});
    bt::require(number(report, "median_rtf") == 2. && yes(report, "target_met") &&
        number(report, "valid_measurement_count") == 3., "one warmup and three full strict runs permit the performance threshold");
    bt::require(!yes(report, "full_acceptance_verified") && yes(field(report, "regions"), "sensors_pending") &&
        yes(field(report, "regions"), "recording_pending") && yes(field(report, "regions"), "ui_pending"),
        "headless measurement never invents sensor delivery, recording, GUI or physical qualification");
    bt::require(yes(io::as_array(field(report, "warmups"), "warmups").front(), "warmup") &&
        !yes(io::as_array(field(report, "runs"), "runs").front(), "warmup"), "run roles are explicit");
    bt::require(io::is_null(field(measure::build_report({run}, {run}), "target_met")) &&
        io::is_null(field(measure::build_report({}, {run, run, run}), "target_met")), "insufficient repetition or missing warmup stays null");
    auto accumulated = run;
    const double accumulated_time = std::nextafter(30., 0.);
    set(accumulated, "simulated_time_s", real(accumulated_time));
    set(accumulated, "real_time_factor", real(accumulated_time / 15.));
    auto accumulated_prefix = field(accumulated, "prefix");
    set(accumulated_prefix, "simulated_time_s", real(accumulated_time));
    set(accumulated, "prefix", accumulated_prefix);
    const auto accumulated_report = measure::build_report({accumulated}, {accumulated, accumulated, accumulated});
    bt::require(yes(accumulated_report, "sampling_protocol_satisfied") &&
        !io::is_null(field(accumulated_report, "target_met")) && !yes(accumulated_report, "target_met"),
        "full 24000-tick window survives accumulated clock rounding without rounding up the RTF threshold");
    for (const double invalid : {0., std::numeric_limits<double>::quiet_NaN()}) {
        auto bad = std::make_shared<Fixture>(); bad->advance_s = invalid; bad->flush_s = 0.;
        bt::invalid([&] { (void)sample(bad); });
        bt::require(bad->closed, "clock rejection closes the owned driver");
    }
}
void diagnostic_exclusion() {
    const auto run = sample();
    auto fallen = std::make_shared<Fixture>(); fallen->fall = true; fallen->advance_s = .25; fallen->flush_s = .25;
    const auto early = sample(fallen);
    bt::require(!yes(early, "valid") && number(early, "rtf") == 30., "fast early fall is explicitly diagnostic");
    const auto report = measure::build_report({run}, {run, early, run});
    bt::require(number(report, "median_rtf") == 2. && number(report, "valid_measurement_count") == 2. &&
        io::is_null(field(report, "target_met")) && !yes(report, "repeated_runs_comparable"),
        "early failure is excluded from median and cannot satisfy full-run protocol");
    auto diagnostic = std::make_shared<Fixture>(); diagnostic->strict = false;
    const auto nonstrict = sample(diagnostic);
    const auto diagnostic_report = measure::build_report({nonstrict}, {nonstrict, nonstrict, nonstrict});
    bt::require(!yes(nonstrict, "valid") && io::is_null(field(diagnostic_report, "median_rtf")) &&
        io::is_null(field(diagnostic_report, "target_met")), "non-strict throughput never closes a valid ride gate");
    auto missing = run; set(missing, "instrumentation_available", Value{false});
    bt::require(io::is_null(field(measure::build_report({missing}, {missing, missing, missing}), "target_met")),
        "missing region instrumentation cannot close the profile protocol");
}
void malformed_or_incompatible_report() {
    const auto run = sample();
    auto other = run; auto identity = field(other, "execution");
    set(identity, "native_source_sha256", text(std::string(64, '5'))); set(other, "execution", identity);
    bt::invalid([&] { (void)measure::build_report({run}, {run, other, run}); });
    bt::invalid([&] { (void)measure::build_report({other}, {run, run, run}); });
    auto per_step = run; set(per_step, "per_step_latency", Value{Object{{"p95", real(14.)}}});
    bt::invalid([&] { (void)measure::build_report({per_step}, {per_step}); });
    auto mislabeled = run; auto latency = field(mislabeled, "chunk_latency");
    set(latency, "units", text("seconds per physics step")); set(mislabeled, "chunk_latency", latency);
    bt::invalid([&] { (void)measure::build_report({mislabeled}, {mislabeled}); });
    for (const double invalid : {0., std::numeric_limits<double>::quiet_NaN()}) {
        auto clock = run; set(clock, "wall_clock_s", real(invalid));
        bt::invalid([&] { (void)measure::build_report({clock}, {clock}); });
    }
    auto contradiction = run; auto prefix = field(contradiction, "prefix");
    set(prefix, "simulated_time_s", real(31.)); set(contradiction, "prefix", prefix);
    bt::invalid([&] { (void)measure::build_report({contradiction}, {contradiction}); });
    set(contradiction, "simulated_time_s", real(31.));
    set(contradiction, "real_time_factor", real(31. / 15.));
    bt::invalid([&] { (void)measure::build_report({contradiction}, {contradiction}); });
    auto bad_profile = run; auto profile = field(bad_profile, "profile");
    set(profile, "dt_s", real(.0005)); set(bad_profile, "profile", profile);
    auto key = field(bad_profile, "comparability"); set(key, "spatial_profile", profile); set(bad_profile, "comparability", key);
    bt::invalid([&] { (void)measure::build_report({bad_profile}, {bad_profile}); });
}
void cli_projection() {
    const std::array<std::string_view, 4> args{"--out", "not-created", "--profile", "spatial-fast-v1"};
    const auto options = measure::parse_options(args);
    bt::require(options.spatial_mode == "3d" && options.duration_s >= 30. && options.warmups == 1 &&
        options.runs == 3 && options.tyre_threads == 1 && options.demand_trace.has_value(),
        "spatial-fast-v1 declares a serial 30s workload and W36 demand trace");
    const auto resolved = measure::resolve_measurement_ride(options);
    bt::require(resolved->setup.model.spatial && resolved->setup.model.spatial->root == bike::spatial::RootKind::free6 &&
        resolved->setup.start_x_m == 3. && resolved->setup.runtime.strict,
        "measurement uses the public W45 spatial projection without planar finish inputs");
    const std::array<std::string_view, 5> no_timing{"--out", "unused", "--profile", "spatial-fast-v1", "--no-region-timing"};
    bt::require(!measure::parse_options(no_timing).region_timing,
        "separate collector-disabled runs can measure incremental full-run instrumentation overhead");
    const std::array<std::string_view, 6> duplicate{"--out", "unused", "--profile", "spatial-fast-v1", "--profile", "spatial-fast-v1"};
    bt::invalid([&] { (void)measure::parse_options(duplicate); });
    for (const auto value : {"0", "65", "-1", "999999999999999999999"}) {
        const std::array<std::string_view, 4> invalid{"--out", "unused", "--threads", value};
        bt::invalid([&] { (void)measure::parse_options(invalid); });
    }
}
} // namespace
int main() { return bt::run([] {
    thresholds(); controlled_wall_and_report(); diagnostic_exclusion();
    malformed_or_incompatible_report(); cli_projection();
}); }
