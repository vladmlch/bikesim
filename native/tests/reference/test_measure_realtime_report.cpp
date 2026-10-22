#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "../../src/runtime/realtime_runner.hpp"
#include "../../src/runtime/setup_values.hpp"
#include "../../src/io/json.hpp"
#include "../../src/platform/files.hpp"
#include <algorithm>
#include <array>
#include <sstream>

namespace {
using namespace runtime::setupvalue;
namespace m = runtime::measurement;
namespace io = bike::io;
Value report() {
    return Value{Object{{"factor", real(2.)}, {"step_ms_mean", real(1.)}, {"step_ms_p50", real(.9)},
        {"step_ms_p95", real(1.5)}, {"step_ms_p99", real(2.)}, {"wall_seconds", real(10.)},
        {"steps", integer(100)}, {"sim_seconds", real(.05)}, {"git_sha", text("aaa")},
        {"track", text("t")}, {"physics", text("p")}, {"machine", text("m")},
        {"effective_timestep_s", real(.001)}, {"requested_duration_s", real(20.)}, {"record_decimation", integer(80)}}};
}
double number(const Value& value, std::string_view key) { return io::as_double(field(value, key), key); }
void write(const std::filesystem::path& path, std::string_view text) { bike::platform::write_exclusive(path, std::as_bytes(std::span(text))); }

// tests/reference/test_measure_realtime_report.py::test_baseline_comparison_emits_per_metric_deltas
void test_baseline_comparison_emits_per_metric_deltas() {
    auto baseline = report(); set(baseline, "factor", real(1.)); set(baseline, "step_ms_p95", real(1.)); set(baseline, "git_sha", text("bbb"));
    const auto result = m::baseline_comparison(report(), baseline, "baseline.json");
    const auto& metrics = field(result, "metrics");
    bt::require(number(field(metrics, "factor"), "delta") == 1. && number(field(metrics, "factor"), "delta_pct") == 100., "factor reports its measured absolute and percent change");
    bt::require(number(field(metrics, "step_ms_p95"), "delta") == .5 && number(field(metrics, "step_ms_p95"), "delta_pct") == 50., "p95 delta retains milliseconds units");
    bt::require(number(field(metrics, "step_ms_p50"), "delta") == 0. && number(field(metrics, "step_ms_p99"), "delta") == 0., "unchanged metrics retain explicit zero deltas");
}
// tests/reference/test_measure_realtime_report.py::test_baseline_comparison_comparability_flags
void test_baseline_comparison_comparability_flags() {
    auto baseline = report(); set(baseline, "git_sha", text("bbb")); set(baseline, "machine", text("other-box")); set(baseline, "effective_timestep_s", real(.002));
    const auto result = m::baseline_comparison(report(), baseline, "baseline.json");
    const auto& comparison = field(result, "comparability");
    for (std::string_view const name : {"git_sha", "machine", "effective_timestep_s"}) bt::require(!io::as_bool(field(field(comparison, name), "match"), name), "changed provenance/input prevents like-for-like classification");
    for (std::string_view const name : {"track", "physics", "record_decimation"}) bt::require(io::as_bool(field(field(comparison, name), "match"), name), "unchanged comparison fields remain explicit");
}
// tests/reference/test_measure_realtime_report.py::test_baseline_comparison_missing_key_stays_missing
void test_baseline_comparison_missing_key_stays_missing() {
    auto baseline = report(); auto& items = std::get<Object>(baseline.data);
    std::erase_if(items, [](const auto& item) { return item.first == "step_ms_p99"; });
    const auto result = m::baseline_comparison(report(), baseline, "baseline.json");
    const auto& entry = field(field(result, "metrics"), "step_ms_p99");
    bt::require(number(entry, "current") == 2. && io::is_null(field(entry, "baseline")) && find(entry, "delta") == nullptr, "missing historical percentile remains null with no fabricated delta");
}
// tests/reference/test_measure_realtime_report.py::test_baseline_comparison_rejects_unreadable_or_non_object
void test_baseline_comparison_rejects_unreadable_or_non_object() {
    bt::invalid([] { (void)m::baseline_comparison(report(), Value{Array{integer(1), integer(2)}}, "not-object.json"); });
    bt::TemporaryDirectory const temporary("realtime-report");
    const std::string absent = (temporary.path() / "missing.json").string();
    const std::string out = (temporary.path() / "out").string();
    const std::array<std::string_view, 4> arguments{"--out", out, "--compare", absent};
    std::ostringstream output, error;
    bt::require(m::measure_realtime_command(arguments, output, error) == 2 && !std::filesystem::exists(out), "unreadable comparison is rejected before simulation or output creation");
}
// tests/reference/test_measure_realtime_report.py::test_native_build_provenance_reads_configured_context
void test_native_build_provenance_reads_configured_context() {
    bt::TemporaryDirectory const temporary("realtime-context");
    write(temporary.path() / "native_check_context.json", R"({"compiler":{"id":"Clang","path":"/usr/bin/clang++","version":"19.0"},"sdk":{"path":"/sdk"},"libcpp_hardening":"EXTENSIVE","target_contexts":{"bike_native":{"configuration":"Release","cxx_standard":23,"cxx_extensions":false,"compile_definitions":["NDEBUG"],"compile_options":["-O2"]}}})");
    write(temporary.path() / "CMakeCache.txt", "NATIVE_SANITIZE:BOOL=ON\nNATIVE_RTSAN:BOOL=OFF\nNATIVE_COVERAGE:BOOL=OFF\nOTHER:STRING=x\n");
    const auto value = m::native_build_provenance(temporary.path());
    bt::require(io::as_string(field(field(value, "compiler"), "id"), "compiler.id") == "Clang" && io::as_string(field(value, "hardening"), "hardening") == "EXTENSIVE", "configured compiler and hardening identities are retained");
    bt::require(number(value, "cxx_standard") == 23. && !io::as_bool(field(value, "cxx_extensions"), "extensions") && io::as_string(field(value, "target_configuration"), "configuration") == "Release", "selected target context retains standard and configuration");
    bt::require(io::canonical_json(field(value, "sanitizer_options")) == R"({"address":true,"coverage":false,"realtime":false})" && io::is_null(field(value, "imported_extension")), "native report states actual lane options and never invents a loaded Python extension");
}
// tests/reference/test_measure_realtime_report.py::test_native_build_provenance_degrades_without_context
void test_native_build_provenance_degrades_without_context() {
    bt::TemporaryDirectory const temporary("realtime-no-context"); const auto value = m::native_build_provenance(temporary.path());
    bt::require(find(value, "context_error") != nullptr && find(value, "compiler") == nullptr, "missing build context is a recorded diagnostic, never invented provenance");
}
} // namespace
// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() { return bt::run([] {
    test_baseline_comparison_emits_per_metric_deltas(); test_baseline_comparison_comparability_flags();
    test_baseline_comparison_missing_key_stays_missing(); test_baseline_comparison_rejects_unreadable_or_non_object();
    test_native_build_provenance_reads_configured_context(); test_native_build_provenance_degrades_without_context();
}); }
