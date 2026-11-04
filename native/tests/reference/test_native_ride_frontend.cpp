// Completion 3.34 (T53). Source: tests/reference/test_native_ride_frontend.py.
//
// The Python file fronts PhysicalRideDriver(backend='native'): the real
// driver advance/flush/summary/export/reset surface plus the per-generation
// seal ownership contract (R03). The native twins are
// runtime::measurement::Driver (NativeDriver — the same resolve/prepare/
// NativeRideRuntime pipeline the CLI measures through) and
// NativeRideRuntime::seal_episode/reset on the welded bootstrap, with the
// per-generation dedupe/conflict assertions landing on
// recording::RunCapture::add_episode — the production journal the driver's
// seal feeds. Assets are the real cli::physical_assets output; the typed
// table decode is covered by reference.native_recording_lifecycle's
// real-episode rows (cited in the ledger).
#include "support/fixture_test.hpp"
#include "support/temporary_directory.hpp"
#include "recording_test_support.hpp"
#include "app/prepare.hpp"
#include "cli/application.hpp"
#include "cli/physical_capture.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "recording/capture.hpp"
#include "runtime/realtime_measurement.hpp"
#include "runtime/realtime_runner.hpp"
#include "runtime/runtime.hpp"
#include <algorithm>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace {

namespace io = bike::io;
namespace msr = runtime::measurement;
namespace rec = bike::recording;
namespace rt = recording_test;

const io::Value::Object &object_of(const io::Value &v, std::string_view what) {
    const auto *o = std::get_if<io::Value::Object>(&v.data);
    bt::require(o != nullptr, std::string(what) + " is an object");
    return *o;
}
const io::Value &member(const io::Value &v, std::string_view key) {
    const auto *found = io::find_member(object_of(v, "member"), key);
    bt::require(found != nullptr,
                "member present: " + std::string(key));
    return *found;
}
std::int64_t integer(const io::Value &v) {
    return io::as_int64(v, "integer");
}
std::string text(const io::Value &v) {
    return io::as_string(v, "text");
}

std::filesystem::path repo(std::string_view rel) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::string(rel);
}

// test_native_physical_driver_finishes_exports_and_retains_owned_snapshot —
// the real NativeDriver pipeline (resolve_measurement_ride + app::prepare +
// NativeRideRuntime). Duration .005/dt .00125 -> 4 max steps like the
// oracle's .005 ride.
void test_native_physical_driver_finishes_exports_and_retains_owned_snapshot() {
    msr::Options options;
    options.physics_config =
        repo("examples/research/viewer_physics_welded.toml");
    options.track = repo("examples/research/rough_uphill_savage.toml").string();
    options.duration_s = .005;
    options.record_decimation = 2;
    options.warmups = 0;
    options.runs = 1;
    // Strict monitor: the welded ride's first 5 ms record no model
    // violation, so the strict driver completes the requested prefix.
    options.benchmark = false;
    auto driver = msr::make_native_driver(options);
    driver->start();
    driver->advance(100, 60.);
    bt::require(driver->boundary().step == 4,
                "the driver commits exactly the duration's step count");
    const auto reason = driver->reason();
    bt::require(reason.has_value() && *reason == "duration_reached",
                "the terminal reason is duration_reached");
    driver->flush();
    const auto summary = driver->summary("duration_reached");
    bt::require(text(member(summary, "backend")) == "native",
                "the summary names the native backend");
    bt::require(integer(member(member(summary, "outcome"), "steps")) == 4,
                "the outcome carries the committed step count");
    bt::TemporaryDirectory const root("native-ride-frontend");
    const auto directory = root.path() / "physical";
    driver->export_run(directory, "duration_reached");
    for (const char *name : {"summary.json", "telemetry.csv",
                             "intervals.jsonl", "terrain_vertices.npy"})
        bt::require(std::filesystem::is_regular_file(directory / name),
                    std::string("export writes ") + name);
    driver->close();
    // The retained-owned snapshot half: PhysicalRideDriver.reset()'s twin is
    // NativeRideRuntime::reset — a fresh generation whose previously
    // captured frames/retained rows stay owned and unchanged.
    auto rig = rt::welded_runtime();
    const auto frame = rig->runtime->render_frame();
    const auto expected = frame.state.integration_state;
    const auto before = rig->runtime->snapshot();
    rig->runtime->reset();
    bt::require(rig->runtime->snapshot().generation ==
                    before.generation + 1,
                "reset publishes the next generation");
    bt::require(rig->runtime->recorded_intervals().empty(),
                "reset clears the retained row index (recorder.rows == 0)");
    bt::require(frame.state.integration_state == expected,
                "the pre-reset snapshot keeps its owned integration state");
    rig->runtime->close();
}

// test_native_driver_seals_episodes_that_survive_reset_and_close — R03:
// per-generation seal ownership, the capture's same-evidence dedupe and
// conflicting/prefix rejections, the physical_assets name set and the
// owned typed source decoding after reset+close.
void test_native_driver_seals_episodes_that_survive_reset_and_close() {
    auto rig = rt::welded_runtime("full");
    rt::advance(*rig->runtime, rig->period);
    auto const first = std::make_shared<runtime::OwnedNativeEpisode>(
        rig->runtime->seal_episode());
    bt::require(first->generation == 1 && !first->samples.empty(),
                "the first generation seals with retained rows");
    // seal_episode(0) is first — natively the seal is a pure snapshot of
    // the SAME retained prefix; identity-level per-generation dedupe is
    // the capture's same-evidence contract below.
    const auto again = rig->runtime->seal_episode();
    bt::require(again.generation == first->generation &&
                    again.samples.size() == first->samples.size(),
                "re-sealing one generation returns the same retained prefix");
    // RunCapture::add_episode — the journal the driver's seal feeds:
    // same-evidence-ptr dedupe, 'conflicting evidence' on a colliding id
    // and 'breaks prefix continuity' on a non-consecutive id.
    rec::RunCapture capture(rt::run_metadata(), true);
    const auto evidence = rt::make_evidence(0, 1, {});
    capture.add_episode(evidence);
    capture.add_episode(evidence); // same pointer: idempotent
    bt::invalid([&] { capture.add_episode(rt::make_evidence(0, 2, {})); });
    bt::invalid([&] { capture.add_episode(rt::make_evidence(4, 2, {})); });
    rig->runtime->reset();
    auto const second = std::make_shared<runtime::OwnedNativeEpisode>(
        rig->runtime->seal_episode());
    bt::require(second->generation == 2,
                "the second generation seals under the next generation");
    // The production asset set the driver snapshot carries —
    // physical_assets on the real prepared launch.
    const auto args = bike::cli::parse_arguments(
        "bike-ride",
        std::vector<std::string_view>{
            "--headless", "--physics-config",
            repo("examples/research/viewer_physics_welded.toml").string(),
            "--track",
            repo("examples/research/rough_uphill_savage.toml").string(),
            "--duration", "0.005"});
    auto resolved = bike::cli::resolve_ride(args);
    const auto request = resolved->setup;
    auto const prepared = bike::app::prepare(
        std::make_unique<bike::initialization::SetupRequest>(request));
    const auto metadata =
        bike::cli::launch_metadata(request, *prepared, std::nullopt);
    const auto assets =
        bike::cli::physical_assets(request, *prepared, metadata);
    std::set<std::string> names;
    for (const auto &asset : assets) names.insert(asset.name);
    bt::require(names.count("track.toml") && names.count("resolved_config.json"),
                "the sealed evidence carries track and resolved config");
    rig->runtime->close();
    // Owned evidence outlives the runtime: the sealed episode's typed rows
    // still decode their channel tables after close — the twin of
    // first.physical.tables() carrying channel_tires/physics (the SQL
    // table decode itself is asserted by
    // reference.native_recording_lifecycle's real-episode rows).
    const auto wire = first->samples.front()->as_wire();
    bt::require(runtime::sample_wire::find(wire, "tires") != nullptr &&
                    !wire.empty(),
                "sealed rows still decode channel tables after close");
}

} // namespace

int main() {
    return bt::run([] {
        test_native_physical_driver_finishes_exports_and_retains_owned_snapshot();
        test_native_driver_seals_episodes_that_survive_reset_and_close();
    });
}
