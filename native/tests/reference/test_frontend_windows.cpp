// Completion 3.34 (T53). Source: tests/reference/test_frontend_windows.py.
//
// The Python fake-window suite drives run_viewer/run_replay_viewer through a
// monkeypatched mujoco.viewer.launch_passive. The qualified native builds
// compile BIKE_BUILD_GUI=OFF, so viewer::run can never open a GLFW window on
// this lane — the ported contracts map each fake-window assertion onto the
// production owner that carries it natively:
//   * key handling -> action_for_key + InputState::apply + speed_for_action;
//   * the >=1/60 sync bound -> Pacer's render-interval request_frame gate;
//   * seal-per-generation / publish-once -> RunCapture/EpisodeJournal (the
//     3.30 lifecycle ports, cited in the ledger);
//   * preview rows -> research::PhysicalLiveCsv generation/'reset' events;
//   * window/import failure -> the 'viewer unavailable' construction throw,
//     which precedes every frame/advance hook so caller-owned runtimes and
//     replay sessions stay open and closeable.
//
// Dropped nodes (python-only):
//   * test_physical_window_skips_ride_html_when_plotly_is_absent — the
//     absent-plotly degradation is an optional-import seam; native HTML
//     plotting is compiled in (viz::plot_physical_ride_html) and the nearest
//     opt-out is the --no-plots flag, not an import failure.
//   * test_replay_window_close_is_incomplete_not_success — the
//     close-before-done -> exit-2 path lives inside the window loop; the
//     incomplete-not-success exit mapping is already covered by
//     reference.t49's replay_exit_code table, and the report-only-on-
//     complete branch is replay_command's caller-side selection, which no
//     headless seam can reach.
#include "support/fixture_test.hpp"
#include "recording_test_support.hpp"
#include "research/preview_csv.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "viewer/input.hpp"
#include "viewer/pacer.hpp"
#include "viewer/session.hpp"
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace view = bike::viewer;
using Clock = std::chrono::steady_clock;

// test_physical_window_bounds_rendering_and_leaves_no_files_when_off — the
// fake window's key schedule {1:[296], 15:['R'], 30:[297], 60:['Q']} and the
// >=1/60 presenter.sync bound. Native owners: action_for_key's F6/F7/F8 and
// R/Q table, InputState::apply(reset), and Pacer's render gate. The
// seal/reset ordering and the no-files tail are asserted by the 3.30
// lifecycle ports (test_viewer_native_pump_seals_reset_generations,
// test_headless_off_writes_no_artifact_files).
void test_physical_window_bounds_rendering_and_leaves_no_files_when_off() {
    using view::Action;
    const auto key = [](int code) {
        return view::action_for_key(view::Mode::physical, code);
    };
    // GLFW_F6=295 slower, GLFW_F7=296 faster, GLFW_F8=297 normal — the fake
    // window's speed keys land on the same Action rows.
    bt::require(key(295) == Action::slower && key(296) == Action::faster &&
                    key(297) == Action::normal_speed,
                "F6/F7/F8 map to slower/faster/normal_speed");
    bt::require(key('R') == Action::reset && key('r') == Action::reset,
                "R maps to reset (case-folded)");
    bt::require(key('Q') == Action::close && key(256) == Action::close,
                "Q/Escape map to close");
    view::InputState input;
    input.paused = true;
    input.braking = true;
    input.apply(Action::reset);
    bt::require(!input.paused && !input.braking && input.brake_demand() == 0.,
                "reset clears pause and braking");
    bt::require(view::speed_for_action(Action::faster, 1) == 2 &&
                    view::speed_for_action(Action::faster, 8) == 8 &&
                    view::speed_for_action(Action::slower, 1) == 1 &&
                    view::speed_for_action(Action::normal_speed, 8) == 1,
                "speed scale is bounded inside [1,8] like the oracle");
    // The fake window measured sync_times spacing >= 1/60: natively the
    // presenter only re-renders when the Pacer's render interval elapses.
    view::Pacer pacer(.00125);
    const auto t0 = Clock::now();
    auto decision = pacer.poll(t0); // first poll initializes + requests a frame
    bt::require(decision.request_frame, "first poll requests the initial frame");
    decision = pacer.poll(t0 + std::chrono::milliseconds(8));
    bt::require(decision.physics_steps > 0 && !decision.request_frame,
                "inside the render interval the pacer offers physics without a new frame");
    decision = pacer.poll(t0 + std::chrono::milliseconds(17));
    bt::require(decision.request_frame,
                "the next frame is gated by the render interval, like the "
                ">=1/60 sync bound the fake window measured");
}

// test_physical_window_recording_publishes_once_and_writes_preview — the
// capture-half (episodes [0,1] generations [1,2], one publication) is covered
// by reference.native_recording_lifecycle::test_viewer_journals_two_episodes_
// and_publishes_once; the preview.csv half ports here: the generation-change
// 'reset' event and the trailing marker rows the oracle greps for. The
// 'requested_scale' column is a Python present_view adapter field — the
// native preview row carries the playback context in 'event'/'generation'.
void test_physical_window_recording_publishes_once_and_writes_preview() {
    std::ostringstream stream;
    {
        bike::research::PhysicalLiveCsv csv(stream);
        runtime::WireObject row;
        runtime::sample_wire::set(row, "time_s", runtime::Wire{0.});
        csv.write(0., 1, row);
        csv.write(.5, 1, row);
        csv.write(.5, 2, row); // the reset: new generation at the same clock
        csv.write_marker("run ended; viewer_closed; t=0.500000s");
        csv.close();
    }
    const auto text = stream.str();
    bt::require(text.find("generation") != std::string::npos &&
                    text.find("event") != std::string::npos,
                "the preview header carries generation/event columns");
    bt::require(text.find("reset") != std::string::npos,
                "a generation change emits the 'reset' event row");
    bt::require(text.find("run ended; viewer_closed") != std::string::npos,
                "the trailing marker row lands in the preview stream");
}

// test_window_construction_error_still_accounts_and_closes +
// test_viewer_import_failure_still_closes_physical_and_replay_owners — the
// fake launch_passive/import failures collapse to the single native gate:
// BIKE_BUILD_GUI=OFF makes GlfwContext's ctor throw before hooks.frame or
// any runtime contact, so the caller-owned runtime stays usable and
// closeable — the same flush+summary tail accounting the oracle's adapter
// ran before close.
void test_window_construction_error_still_accounts_and_closes() {
    bt::require(!view::gui_available(),
                "this lane compiles BIKE_BUILD_GUI=OFF");
    auto rig = recording_test::welded_runtime();
    int frame_calls = 0, advance_calls = 0;
    view::SessionHooks hooks;
    hooks.frame = [&] {
        ++frame_calls;
        return rig->runtime->render_frame();
    };
    hooks.advance = [&](std::int64_t, double) { ++advance_calls; };
    hooks.terminal = [&] { return false; };
    view::ViewerOptions options;
    options.mode = view::Mode::physical;
    options.timestep_s = .00125;
    bool thrown = false;
    try {
        (void)view::run(std::move(hooks), rig->runtime->model_image(), options);
    } catch (const std::exception &error) {
        thrown = true;
        bt::require(std::string_view(error.what()).find("viewer") !=
                        std::string_view::npos,
                    "the window failure is the compiled viewer gate");
    }
    bt::require(thrown, "viewer::run rejects when the window cannot open");
    bt::require(frame_calls == 0 && advance_calls == 0,
                "no frame/advance hook runs before the window exists");
    // The caller still owns the runtime: the accounted tail (flush, summary,
    // close) is the caller's contract, exactly like run_viewer's finally.
    rig->runtime->flush();
    const auto snapshot = rig->runtime->snapshot();
    bt::require(snapshot.step >= 0 && snapshot.generation >= 1,
                "the owned runtime stays inspectable after the viewer failure");
    rig->runtime->close();
}

void test_viewer_import_failure_still_closes_physical_and_replay_owners() {
    auto rig = recording_test::welded_runtime();
    int calls = 0;
    view::SessionHooks hooks;
    hooks.frame = [&] {
        ++calls;
        return rig->runtime->render_frame();
    };
    hooks.advance = [&](std::int64_t, double) { ++calls; };
    hooks.terminal = [&] { return false; };
    view::ViewerOptions options;
    options.timestep_s = .00125;
    int throws = 0;
    try {
        (void)view::run(std::move(hooks), rig->runtime->model_image(), options);
    } catch (const std::exception &) {
        ++throws;
    }
    view::SessionHooks replay_hooks;
    replay_hooks.frame = [&] {
        ++calls;
        return rig->runtime->render_frame();
    };
    replay_hooks.advance = [&](std::int64_t, double) { ++calls; };
    replay_hooks.terminal = [&] { return false; };
    try {
        (void)view::run_replay(std::move(replay_hooks),
                               rig->runtime->model_image(), options, true);
    } catch (const std::exception &) {
        ++throws;
    }
    bt::require(throws == 2,
                "both the physical and replay entries refuse the absent viewer");
    bt::require(calls == 0,
                "neither owner is touched by the unavailable viewer");
    // The replay-session owner is caller-owned like the runtime: closing it
    // is the caller's cleanup, never the viewer's.
    rig->runtime->close();
}

} // namespace

int main() {
    return bt::run([] {
        test_physical_window_bounds_rendering_and_leaves_no_files_when_off();
        test_physical_window_recording_publishes_once_and_writes_preview();
        test_window_construction_error_still_accounts_and_closes();
        test_viewer_import_failure_still_closes_physical_and_replay_owners();
    });
}
