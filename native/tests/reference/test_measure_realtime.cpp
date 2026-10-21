// Completion 3.04 CLI acceptance contracts. These are source registrations,
// not a record that a measurement or the native build has passed.
#include "../support/migration_test.hpp"
#include "../support/temporary_directory.hpp"
#include "runtime/realtime_runner.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "platform/files.hpp"
#include <array>
#include <cmath>
#include <sstream>

namespace {
namespace m = runtime::measurement;
namespace io = bike::io;
using namespace runtime::setupvalue;

io::Value read_json(const std::filesystem::path& path) {
    const auto bytes = io::read_numpy_bytes(path);
    std::string text; text.reserve(bytes.size());
    for (std::byte byte : bytes) text.push_back(static_cast<char>(byte));
    return io::parse_json(text);
}
void test_short_native_run_writes_measurement_schema() {
    bt::TemporaryDirectory directory("measure-realtime");
    const auto output = (directory.path() / "run").string();
    // --benchmark (non-strict monitor) is the documented mode for a
    // completed measurement on this platform: strict headless runs abort at
    // step 4 on the recorded energy.constraint_work baseline violation
    // (docs/RIDE.md, docs/perf.md), identical on the Python oracle, so a
    // strict invocation can never produce the successful schema this test
    // verifies. The report still records monitor_strict=false,
    // benchmark_mode=true and keeps valid_prefix false — the diagnostic is
    // retained, not relabeled.
    const std::array<std::string_view, 11> args{
        "--backend", "native", "--duration", ".2", "--runs", "1", "--warmup", "0",
        "--benchmark", "--out", output};
    std::ostringstream message, error;
    const int code = m::measure_realtime_command(args, message, error);
    bt::require(code == 0, "short native measurement succeeds: " + error.str());
    const auto report = read_json(std::filesystem::path(output) / "report.json");
    bt::require(io::as_int64(field(report, "schema_version"), "schema_version") == 1, "measurement schema version");
    bt::require(io::as_int64(field(report, "measurement_count"), "count") == 1, "one measured run");
    bt::require(io::as_array(field(report, "warmups"), "warmups").empty(), "no warmup in requested short run");
    const auto& run = io::as_array(field(report, "runs"), "runs").at(0);
    for (std::string_view key : {"comparability", "execution", "prefix", "chunk_latency", "errors", "limitations", "identity_unchanged", "valid_prefix", "measured_ok", "per_step_latency"})
        bt::require(find(run, key) != nullptr, "required measurement key " + std::string(key));
    for (std::string_view key : {"setup_wall_s", "wall_clock_s", "final_flush_wall_s", "export_wall_s", "simulated_time_s"})
        bt::require(std::isfinite(io::as_double(field(run, key), key)), "finite timing field " + std::string(key));
    for (std::string_view key : {"configuration_sha256", "terrain_sha256", "python_source_sha256", "versions", "timestep_s", "internal_controller_period_s", "external_policy_period_s", "max_steps", "record_decimation", "strict", "compute_slice_s", "controls"})
        bt::require(find(field(run, "comparability"), key) != nullptr, "required comparison key " + std::string(key));
    for (std::string_view key : {"address", "realtime", "coverage"})
        (void)io::as_bool(field(field(field(report, "provenance"), "sanitizer_options"), key), key);
    bt::require(io::is_null(field(run, "per_step_latency")), "boundary timing is not relabelled per-step timing");
    bt::require(io::is_null(field(report, "headless_rtf_threshold_met")), "one short run cannot claim the performance gate");
    bt::require(!io::as_bool(field(report, "full_acceptance_verified"), "acceptance"), "short run does not qualify full acceptance");
    bt::require(io::canonical_json(run) == io::canonical_json(read_json(std::filesystem::path(output) / "run-00.json")), "run sidecar equals report run");
    for (const auto* name : {"summary.json", "terrain_vertices.npy", "telemetry.csv", "intervals.jsonl"})
        bt::require(std::filesystem::is_regular_file(std::filesystem::path(output) / "run-00" / name), "measurement export " + std::string(name));
    bt::require(bt::text(std::filesystem::path(output) / "run-00/telemetry.csv") == "schema_version,time_s\r\n",
        "retention-off export retains the Python recorder's empty CSV schema");
}
void test_nonempty_output_is_rejected_before_setup() {
    bt::TemporaryDirectory directory("measure-nonempty");
    const std::string marker = "keep me\n";
    bike::platform::write_exclusive(directory.path() / "existing.txt", std::as_bytes(std::span(marker)));
    const auto path = directory.path().string();
    const std::array<std::string_view, 4> args{"--out", path, "--physics-config", "/not/a/real/config.toml"};
    std::ostringstream message, error;
    bt::require(m::measure_realtime_command(args, message, error) == 2, "nonempty output exits 2");
    bt::require(error.str().find("new or empty") != std::string::npos, "output check precedes setup");
    bt::require(!std::filesystem::exists(directory.path() / "report.json"), "rejected output has no report");
    bt::require(io::read_numpy_bytes(directory.path() / "existing.txt").size() == marker.size(), "existing file preserved");
}
void test_invalid_runs_and_python_backend_exit_two() {
    bt::TemporaryDirectory directory("measure-invalid");
    const auto path = (directory.path() / "uncreated").string();
    for (const auto& pair : {std::pair{"--runs", "0"}, std::pair{"--backend", "python"}}) {
        const std::array<std::string_view, 4> args{"--out", path, pair.first, pair.second};
        std::ostringstream message, error;
        bt::require(m::measure_realtime_command(args, message, error) == 2, "invalid measurement arguments exit 2");
        bt::require(!std::filesystem::exists(path), "argument rejection does not create output");
    }
}
void test_native_driver_wall_cap_excludes_setup_and_prevents_more_physics() {
    const std::array<std::string_view, 4> args{"--out", "not-exported-by-wall-cap-control", "--duration", ".2"};
    const auto options = m::parse_options(args);
    double clock = 0.;
    auto driver = m::make_native_driver(options, [&] { return clock; });
    // A slow setup is outside measurement. start() must reset the origin,
    // and a zero wall-budget call remains a no-physics valid control.
    clock = 301.; driver->start();
    const auto initial = driver->boundary();
    driver->advance(driver->settings().max_steps, 0.);
    bt::require(!driver->reason() && driver->boundary().step == initial.step,
        "setup time does not consume the whole-run wall cap");
    clock += 300.;
    driver->advance(driver->settings().max_steps, m::compute_slice_s);
    bt::require(driver->reason() == "wall_clock_cap" && driver->boundary().step == initial.step,
        "the real driver stops before physics at the source wall-clock cap");
    driver->flush(); driver->close();
}
} // namespace
int main() {
    return bt::run([] {
        test_nonempty_output_is_rejected_before_setup();
        test_invalid_runs_and_python_backend_exit_two();
        test_native_driver_wall_cap_excludes_setup_and_prevents_more_physics();
        test_short_native_run_writes_measurement_schema();
    });
}
