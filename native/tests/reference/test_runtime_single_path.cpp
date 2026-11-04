// Completion 3.34 (T53). Source: tests/reference/test_runtime_single_path.py.
//
// Portable halves only: the accounted HUD's real-time factor and
// first-failure marker, and the measurement driver's configured
// dt/decimation plus the run-level flush/measurement summary. The
// adapter-only records (python-mode preview path existence, preview
// rejection, monkeypatched viewer-window terminal tail and exceptional
// cleanup) have no native contract and are dropped in the ledger.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "support/temporary_directory.hpp"
#include "io/value.hpp"
#include "runtime/realtime_measurement.hpp"
#include "runtime/realtime_runner.hpp"
#include "runtime/samples.hpp"
#include "runtime/status.hpp"
#include "runtime/runtime.hpp"
#include "viewer/hud.hpp"
#include <filesystem>
#include <string>
#include <vector>

namespace {

namespace io = bike::io;
namespace msr = runtime::measurement;
namespace viewer = bike::viewer;

const io::Value::Object &object_of(const io::Value &v, std::string_view what) {
    const auto *o = std::get_if<io::Value::Object>(&v.data);
    bt::require(o != nullptr, std::string(what) + " is an object");
    return *o;
}
const io::Value &member(const io::Value &v, std::string_view key) {
    const auto *found = io::find_member(object_of(v, "member"), key);
    bt::require(found != nullptr, "member present: " + std::string(key));
    return *found;
}
double number(const io::Value &v) { return io::as_double(v, "number"); }
std::int64_t integer(const io::Value &v) { return io::as_int64(v, "integer"); }

std::filesystem::path repo(std::string_view rel) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::string(rel);
}

std::string hud_text(const std::vector<viewer::HudColumn> &columns) {
    std::string out;
    for (const auto &column : columns) out += column.text;
    return out;
}

// test_accounted_hud_displays_realtime_factor_without_repeated_failure —
// one accounted frame carrying a retained first model failure renders its
// real-time factor and the generic invalid marker; the failure's own text
// is not re-printed on the line (first_failure_only_once semantics live in
// the monitor — the HUD consumes the retained marker, not the message).
void test_accounted_hud_displays_realtime_factor_without_repeated_failure() {
    runtime::RenderFrame frame;
    frame.state.generation = 1;
    frame.state.step = 8;
    frame.state.time_s = .01;
    auto const sample = std::make_shared<runtime::PhysicalSampleData>();
    sample->interval_id = 1;
    sample->time_s = .01;
    sample->end_time_s = .01125;
    sample->channels = runtime::WireObject{
        {"drive", runtime::Wire{runtime::WireObject{{"crank_phase_rad", runtime::Wire{0.5}},
                                                   {"crank_speed_rps", runtime::Wire{1.0}}}}},
        {"rider", runtime::Wire{runtime::WireObject{}}},
        {"tires", runtime::Wire{runtime::WireObject{}}}};
    frame.state.latest_sample = sample;
    frame.state.first_failure = runtime::ReferenceFailure{
        .time_s = .01, .violations = {"foot_front.friction"}};
    const auto columns = viewer::physical_columns(frame, .75);
    const auto line = hud_text(columns);
    bt::require(line.find("RTF=0.75x") != std::string::npos,
                "the HUD line prints the achieved real-time factor");
    bt::require(line.find("foot_front") == std::string::npos &&
                    line.find("friction") == std::string::npos,
                "the retained first failure is not re-printed on the HUD");
    const auto legend = viewer::hud_legend(columns);
    bt::require(!legend.empty(), "the HUD legend renders for the columns");
}

// test_measurement_uses_v2_dt_configures_diagnostics_and_counts_flush —
// the native driver honors the configured timestep/decimation, reports the
// strict diagnostic posture (the Python `relaxed=True` monkeypatch has no
// native twin — natively diagnostics are always checked), and the run
// summary counts the terminal flush inside wall-clock time.
void test_measurement_uses_v2_dt_configures_diagnostics_and_counts_flush() {
    msr::Options options;
    options.physics_config =
        repo("examples/research/viewer_physics_welded.toml");
    options.track = repo("examples/research/rough_uphill_savage.toml").string();
    options.duration_s = .0015;
    options.timestep_s = .0005;
    options.record_decimation = 80;
    options.warmups = 0;
    options.runs = 1;
    // The oracle's `monitor.relaxed = True` monkeypatch maps onto the
    // benchmark posture (non-strict monitor), which this test pins below.
    options.benchmark = true;
    auto driver = msr::make_native_driver(options);
    const auto &settings = driver->settings();
    bt::require(settings.timestep_s == .0005,
                "the driver runs the configured v2 timestep");
    bt::require(settings.record_decimation == 80,
                "the driver runs the configured decimation");
    bt::require(!settings.strict,
                "benchmark mode is the native twin of the relaxed monitor");
    driver->close();
    const auto run = msr::measure_run(
        [&] { return msr::make_native_driver(options); }, std::nullopt);
    bt::require(integer(member(run, "physics_steps")) == 3,
                "the measurement commits duration/dt steps");
    bt::require(number(member(run, "simulated_time_s")) > 0.,
                "the measurement reports simulated time");
    bt::require(number(member(run, "wall_clock_s")) > 0.,
                "the measurement reports wall-clock time");
    bt::require(number(member(run, "real_time_factor")) > 0.,
                "the measurement reports a real-time factor");
    const auto flush = number(member(run, "final_flush_wall_s"));
    bt::require(flush >= 0. && flush <= number(member(run, "wall_clock_s")),
                "the terminal flush is counted inside the measured wall clock");
}

} // namespace

int main() {
    return bt::run([] {
        test_accounted_hud_displays_realtime_factor_without_repeated_failure();
        test_measurement_uses_v2_dt_configures_diagnostics_and_counts_flush();
    });
}
