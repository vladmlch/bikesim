// Completion 3.10 / decision F3 (2026-10-24): the realtime gate re-aimed at
// the native backend. The ported contract of
// tests/reference/test_realtime_gate.py::test_realtime_factor_for_full_twenty_seconds
// is unchanged — 20 simulated seconds at dt=1.25 ms (16,000 steps, record
// decimation 80) on each of rough_uphill_savage/rough_uphill_extreme and a
// real-time factor >= 1x — but the measured subject is now the shipped
// Release `bike-measure-realtime` artifact, not the Python-oracle accounted
// path that T63 deletes. One CTest node per track supplies the artifact path
// and the track file as argv; this executable spawns the artifact through
// the bounded platform::run seam and asserts the report fields the Python
// gate asserted, plus the artifact's own Release/identity provenance.
//
// --benchmark is the Python gate's `reference_monitor.strict=False`
// equivalent: the reference monitor still runs and retains the documented
// energy.constraint_work baseline violation as prefix.first_failure instead
// of stopping physics, so a strict run can never produce the 16,000-step
// prefix the contract measures (same class as test_measure_realtime.cpp).
// The reported prefix stays a throughput sample of a ride whose validity is
// separately documented red — the gate asserts duration/steps/RTF, never a
// valid-ride claim.
#include "../support/migration_test.hpp"
#include "../support/temporary_directory.hpp"
#include "io/json.hpp"
#include "platform/argv.hpp"
#include "platform/process.hpp"
#include "runtime/setup_values.hpp"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {
namespace io = bike::io;
using namespace runtime::setupvalue;

constexpr double kDtS = .00125;
constexpr double kDurationS = 20.;
constexpr std::int64_t kSteps = 16000;

io::Value read_json(const std::filesystem::path& path) {
    return io::parse_json(bt::text(path));
}
bool is_true(const Value& object, std::string_view key) {
    const auto* flag = std::get_if<bool>(&field(object, key).data);
    return flag != nullptr && *flag;
}
// Spawns the artifact and asserts the schema-1 measurement report. The same
// body the Python gate asserted is re-aimed verbatim; only the measured
// backend changed. `native_test_id` in the port map names this function.
void test_realtime_factor_for_full_twenty_seconds(const std::filesystem::path& artifact,
    const std::filesystem::path& track, const std::filesystem::path& physics_config) {
    bt::require(std::filesystem::is_regular_file(artifact),
        "realtime artifact is an acquisition failure, not a skip: " + artifact.string());
    for (const auto& input : {track, physics_config})
        bt::require(std::filesystem::is_regular_file(input), "gate input exists: " + input.string());
    const bt::TemporaryDirectory scratch("realtime-gate");
    const auto out = scratch.path() / "measure";
    bike::platform::ProcessOptions process;
    // Four whole-boundary runs (warmup + 3 measured) each replay setup plus a
    // 16,000-step ride; the deadline is generous and wall-clock free.
    process.timeout_s = 900.;
    const auto result = bike::platform::run(
        {artifact.string(), "--backend", "native", "--physics-config", physics_config.string(),
         "--track", track.string(), "--duration", "20", "--dt", "0.00125", "--warmup", "1",
         "--runs", "3", "--record-decimation", "80", "--benchmark", "--out", out.string()},
        scratch.path(), process);
    // The artifact's own per-run lines are the gate's measured values — keep
    // them in the CTest log on either outcome.
    std::cout << result.stdout_text;
    if (!result.stderr_text.empty()) std::cerr << result.stderr_text;
    bt::require(result.exit_code == 0,
        "bike-measure-realtime reports every measured run measured_ok (exit " +
        std::to_string(result.exit_code) + ")");
    const auto report = read_json(out / "report.json");
    bt::require(io::as_int64(field(report, "schema_version"), "schema_version") == 1, "schema-1 report");
    bt::require(io::as_string(field(report, "backend"), "backend") == "native", "native backend report");
    bt::require(io::as_int64(field(report, "warmup_count"), "warmup_count") == 1 &&
        io::as_int64(field(report, "measurement_count"), "measurement_count") == 3,
        "sampling protocol: one excluded warmup plus three measured runs");
    bt::require(is_true(report, "sampling_protocol_satisfied"), "tool sampling protocol satisfied");
    const auto& invocation = field(report, "invocation");
    bt::require(io::as_double(field(invocation, "duration_s"), "duration_s") == kDurationS &&
        io::as_double(field(invocation, "effective_dt_s"), "effective_dt_s") == kDtS &&
        is_true(invocation, "benchmark_mode") && !is_true(invocation, "monitor_strict"),
        "gate invocation pins the Python contract's duration/dt in non-strict mode");
    const auto& provenance = field(report, "provenance");
    const auto build_type = io::as_string(field(field(provenance, "build_context"), "build_type"), "build_type");
    bt::require(build_type == "Release",
        "the realtime contract measures a Release artifact, got build_type=" + build_type);
    std::cout << "provenance: machine=" << io::canonical_json(field(provenance, "machine"))
        << " git_sha=" << io::canonical_json(field(provenance, "git_sha"))
        << " working_tree_dirty=" << io::canonical_json(field(provenance, "working_tree_dirty"))
        << " build_type=" << build_type
        << " compiler=" << io::canonical_json(field(field(provenance, "build_context"), "compiler_id"))
        << '\n';
    const auto& comparability = field(io::as_array(field(report, "runs"), "runs").front(), "comparability");
    bt::require(io::as_double(field(comparability, "timestep_s"), "timestep_s") == kDtS &&
        io::as_int64(field(comparability, "max_steps"), "max_steps") == kSteps &&
        io::as_int64(field(comparability, "record_decimation"), "record_decimation") == 80,
        "comparability key pins dt/steps/decimation");
    std::vector<double> factors;
    for (const auto& run : io::as_array(field(report, "runs"), "runs")) {
        bt::require(is_true(run, "measured_ok"), "measured run has no errors");
        bt::require(io::as_array(field(run, "errors"), "errors").empty(), "run errors stay empty");
        bt::require(io::as_int64(field(run, "physics_steps"), "physics_steps") == kSteps,
            "all 16,000 steps rode");
        const auto& prefix = field(run, "prefix");
        bt::require(io::as_string(field(prefix, "reason"), "reason") == "duration_reached",
            "the ride reaches duration, not a crash/finish/cap prefix");
        bt::require(io::as_int64(field(prefix, "steps"), "steps") == kSteps &&
            io::as_int64(field(prefix, "requested_steps"), "requested_steps") == kSteps,
            "complete requested prefix");
        bt::require(io::as_double(field(run, "simulated_time_s"), "simulated_time_s") >= kDurationS - kDtS,
            "at least duration-dt simulated");
        // The non-strict monitor's retained first_failure is evidence the
        // gate measures the real instrumented path, not a quieted one.
        std::cout << "run RTF=" << io::canonical_json(field(run, "real_time_factor"))
            << " wall_s=" << io::canonical_json(field(run, "wall_clock_s"))
            << " first_failure=" << io::canonical_json(field(prefix, "first_failure")) << '\n';
        factors.push_back(io::as_double(field(run, "real_time_factor"), "real_time_factor"));
    }
    const auto median = io::as_double(field(report, "median_real_time_factor"), "median_real_time_factor");
    std::cout << "median RTF=" << median << " contract>=1.0 track=" << track.filename().string() << '\n';
    bt::require(std::isfinite(median) && median >= 1.,
        "re-aimed contract: native Release median RTF >= 1x, measured " + std::to_string(median));
}
} // namespace
int main(int argc, char** argv) {
    return bt::run([&] {
        bt::require(argc == 4, "artifact path, track file and physics config are supplied by CTest");
        const auto owned = bike::platform::main_arguments(argc, argv);
        test_realtime_factor_for_full_twenty_seconds(owned[1], owned[2], owned[3]);
    });
}
