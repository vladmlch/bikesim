#pragma once
#include "input.hpp"
#include "pacer.hpp"
#include "../runtime/runtime.hpp"
#include <functional>
#include <optional>
#include <string>

namespace bike::research { class PolicySession; }
namespace bike::legacy { class RideSimulation; class Playground; }
namespace bike::viewer {
struct ViewerOptions {
    Mode mode = Mode::physical;
    std::string title = "bike-sim";
    double timestep_s = .00125;
    unsigned time_scale = 1;
    double render_interval_s = 1. / 60.;
    int width = 1280, height = 800;
    bool close_on_terminal = false;
    bool hidden = false;
    std::optional<double> duration_s;
};
// All callbacks run on the viewer-owning main thread. They own the simulation
// policy. In particular reset must seal the old episode BEFORE resetting and
// stop may drain only an already-issued research interval. The viewer never
// closes/finalizes a runtime or invokes a policy itself.
struct SessionHooks {
    std::function<runtime::RenderFrame()> frame;
    std::function<void(std::int64_t target_step, double wall_budget_s)> advance;
    std::function<bool()> terminal;
    // Sampled at every UI iteration, including pause and terminal hold. A
    // signal handler only sets an atomic flag; cleanup runs on this thread.
    std::function<bool()> close_requested;
    std::function<void()> reset;
    std::function<void()> stop;
    std::function<void(double front, double rear)> brakes;
    std::function<void(bool)> pause;
    std::function<bool(int)> unhandled_key;
    std::function<bool(int key, int modifiers)> key_event;
    // Needed only when a playground operation replaces the compiled model.
    std::function<runtime::ModelImage()> model_image;
    std::function<void(Action)> operator_event;
    std::function<void(std::string_view)> console;
    std::function<std::string(const runtime::RenderFrame&,const InputState&,std::optional<double>)> hud;
    std::function<void(const runtime::RenderFrame&,std::optional<double>)> presented;
};
struct RunResult {
    bool terminal = false, closed_by_user = false;
    std::uint64_t resets = 0;
    std::optional<PacingReport> pacing;
};
[[nodiscard]] bool gui_available() noexcept;
[[nodiscard]] RunResult run(SessionHooks, const runtime::ModelImage&, const ViewerOptions& = {});
struct LifecycleHooks {
    std::function<void()> before_reset, after_reset, after_advance;
    std::function<void(Action)> operator_event;
    std::function<bool()> close_requested;
};
[[nodiscard]] RunResult run_ride(runtime::NativeRideRuntime&, const ViewerOptions&,
    runtime::RideControl control = {}, const LifecycleHooks& = {});
[[nodiscard]] RunResult run_research(research::PolicySession&, const ViewerOptions&,
    const LifecycleHooks& = {});
[[nodiscard]] RunResult run_legacy(legacy::RideSimulation&, const ViewerOptions&,
    const LifecycleHooks& = {});
[[nodiscard]] RunResult run_playground(legacy::Playground&, const ViewerOptions& = {});
[[nodiscard]] int replay_exit_code(bool completed, bool passed) noexcept;
[[nodiscard]] RunResult run_replay(SessionHooks, const runtime::ModelImage&,
    ViewerOptions, bool checked);
} // namespace bike::viewer
