#include "session.hpp"
#include "../diag.hpp"
#include "glfw_context.hpp"
#include "hud.hpp"
#include "pacer.hpp"
#include "render_owner.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace bike::viewer {
bool gui_available() noexcept {
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    return true;
#else
    return false;
#endif
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
RunResult run(SessionHooks hooks, const runtime::ModelImage& model, const ViewerOptions& options) {
    if (!hooks.frame || !hooks.advance || !hooks.terminal)
        throw std::invalid_argument("viewer session requires frame, advance and terminal callbacks");
    if (options.duration_s && (!std::isfinite(*options.duration_s) || *options.duration_s <= 0.))
        throw std::invalid_argument("viewer duration must be finite and positive");
    Pacer pacer(options.timestep_s,options.render_interval_s); pacer.set_scale(options.time_scale);
    // Resource order matters on every exit, including an apply/draw failure.
    GlfwContext window(options.title, options.width, options.height, options.hidden);
    auto renderer = std::make_unique<RenderOwner>(model);
    auto frame = hooks.frame(); renderer->apply(frame);
    InputState input; input.scale = options.time_scale;
    const auto print = [&](std::string_view line) { if (hooks.console) hooks.console(line); else std::cout << line << '\n'; };
    print(help_text(options.mode));
    using Clock = std::chrono::steady_clock;
    auto now = Clock::now(); pacer.reset(now);
    AchievedRate rate(now, frame.state.time_s);
    auto last_hud = now;
    int legend_generation = -1;
    int clock_generation = frame.state.generation;
    std::optional<std::string> reported_failure;
    RunResult result;
    bool request_frame = true, stopped = false, terminal_drawn = false;
    const auto stop = [&] { if (!stopped) { stopped = true; if (hooks.stop) hooks.stop(); } };
    while (window.open()) {
        if (hooks.close_requested && hooks.close_requested()) { result.closed_by_user = true; break; }
        for (const auto& event : window.poll()) {
            if (event.kind != WindowEvent::Kind::key) { renderer->move_camera(event.action,event.dx,event.dy); request_frame = true; continue; }
            const Action action = action_for_key(options.mode,event.key);
            if (action == Action::none) {
                const bool handled=hooks.key_event&&hooks.key_event(event.key,event.modifiers);
                if(handled)request_frame=true;
                else if(hooks.unhandled_key)request_frame=hooks.unhandled_key(event.key)||request_frame;
                continue;
            }
            if (hooks.operator_event) hooks.operator_event(action);
            if (action == Action::close) { result.closed_by_user = true; window.request_close(); break; }
            if (action == Action::reset) {
                if (!hooks.reset) continue;
                // The caller seals/drains the previous episode before reset.
                hooks.reset();
                ++result.resets; stopped = false; terminal_drawn = false;
                input.apply(action);
                if (hooks.brakes) hooks.brakes(0.,0.);
                if (hooks.pause) hooks.pause(false);
                frame = hooks.frame();
                now = Clock::now(); pacer.set_paused(false); pacer.reset(now); rate.rebase(now,frame.state.time_s);
                renderer->reset_camera(); request_frame = true;
                continue;
            }
            const auto previous_scale = input.scale;
            input.apply(action);
            if (action == Action::toggle_pause) { if(hooks.pause) hooks.pause(input.paused); pacer.set_paused(input.paused); }
            if (action == Action::toggle_brakes || action == Action::brake_less || action == Action::brake_more)
                if(hooks.brakes) hooks.brakes(input.brake_demand(),input.brake_demand());
            if (input.scale != previous_scale) pacer.set_scale(input.scale);
            if (action == Action::camera_cycle) renderer->cycle_camera();
            if (action == Action::camera_side) renderer->set_camera(CameraMode::side);
            if (action == Action::camera_chase) renderer->set_camera(CameraMode::chase);
            if (action == Action::markers) { renderer->set_markers(input.markers);if(!renderer->has_markers())print("[KEY G] No marker geoms in this model."); }
            if (action == Action::help) print(help_text(options.mode));
            if (action == Action::fixed_parameter) print("Physical parameters are fixed per run; edit the configuration and start a new run.");
            request_frame = true;
        }
        // A close event is a hard boundary: never begin another control tick.
        if (!window.open()) { result.closed_by_user = true; break; }
        if (hooks.close_requested && hooks.close_requested()) { result.closed_by_user = true; break; }
        // Operator callbacks may finish an already-issued interval or replace
        // the compiled model. Rebase BEFORE computing a new target, so an
        // old generation's step count cannot leak into the new model.
        if (request_frame) {
            auto observed = hooks.frame();
            if (observed.state.generation != frame.state.generation || observed.state.step != frame.state.step) {
                now = Clock::now(); pacer.reset(now); rate.rebase(now,observed.state.time_s);
            }
            frame = std::move(observed);
        }
        now = Clock::now();
        const bool time_complete = options.duration_s && frame.state.time_s >= *options.duration_s;
        result.terminal = hooks.terminal() || time_complete;
        const auto decision = pacer.poll(now);
        if (!result.terminal && !input.paused && decision.physics_steps != 0) {
            const auto before = frame.state;
            const auto room = std::numeric_limits<std::int64_t>::max() - before.step;
            const auto count = std::min(decision.physics_steps,static_cast<std::uint64_t>(room));
            auto target = before.step + static_cast<std::int64_t>(count);
            if (options.duration_s) {
                const double remaining = std::max(0.,*options.duration_s-before.time_s);
                const double ticks = std::ceil(remaining/options.timestep_s);
                if (ticks < static_cast<double>(count)) target = before.step + static_cast<std::int64_t>(ticks);
            }
            hooks.advance(target,kComputeSliceS);
            auto after = hooks.frame();
            const bool inspection_wrap=options.mode==Mode::inspection&&after.state.generation>before.generation;
            const bool checked_rollback=options.mode==Mode::checked_replay&&after.state.generation>before.generation&&hooks.terminal();
            if (!inspection_wrap&&!checked_rollback&&(after.state.generation != before.generation || after.state.step < before.step || after.state.step > target))
                throw std::runtime_error("viewer advance violated generation/target-step boundary");
            pacer.completed(inspection_wrap?count:checked_rollback?0:static_cast<std::uint64_t>(after.state.step-before.step));
            frame = std::move(after);
            result.terminal = hooks.terminal() || (options.duration_s && frame.state.time_s >= *options.duration_s);
        } else if (request_frame) frame = hooks.frame();
        // Model/rider changes are explicitly model-generation changes.
        // A renderer cannot accept a different digest merely because nq/nv fit.
        if (frame.model_sha256 != renderer->model_sha256()) {
            if (!hooks.model_image) throw std::runtime_error("viewer received a new model without an owned image supplier");
            auto replacement = std::make_unique<RenderOwner>(hooks.model_image());
            replacement->apply(frame); replacement->set_markers(input.markers); renderer=std::move(replacement);
        }
        if (frame.state.generation != clock_generation) {
            if(frame.state.generation < clock_generation) throw std::runtime_error("viewer generation moved backwards");
            clock_generation=frame.state.generation;
            reported_failure.reset();
            pacer.reset(Clock::now());rate.rebase(Clock::now(),frame.state.time_s);renderer->reset_camera();request_frame=true;
        }
        if(result.terminal&&!terminal_drawn){request_frame=true;terminal_drawn=true;}
        // Stored-state inspection can seek and run backwards. Its cursor is
        // not an integrated simulation clock, so it has no achieved RTF.
        const auto achieved = options.mode==Mode::inspection?std::optional<double>{}:rate.update(Clock::now(),frame.state.time_s);
        if (request_frame || decision.request_frame) {
            renderer->apply(frame);
            renderer->set_overlay(overlay_text(frame,input,achieved,options.mode));
            const auto [width,height]=window.framebuffer_size();renderer->draw(width,height);
            if(width>0&&height>0)window.swap();
            if(hooks.presented)hooks.presented(frame,achieved);
            request_frame=false;
        }
        if (input.telemetry && (frame.state.generation != legend_generation || std::chrono::duration<double>(now-last_hud).count() >= .5)) {
            if (frame.state.generation != legend_generation && (options.mode==Mode::physical||options.mode==Mode::research||options.mode==Mode::checked_replay))
                print(hud_legend(physical_columns(frame,achieved)));
            if(frame.state.first_failure) {
                const auto failure=frame.state.first_failure->message();
                if(reported_failure!=failure){print("[FIRST FAILURE] "+failure);reported_failure=failure;}
            }
            legend_generation=frame.state.generation;last_hud=now;print(hooks.hud?hooks.hud(frame,input,achieved):hud_line(frame,input,achieved,options.mode));
        }
        if (result.terminal && options.close_on_terminal) break;
        if (result.terminal || input.paused || decision.physics_steps==0) window.wait(.005);
    }
    if (result.closed_by_user) stop();
    // Stop may have finished an already-issued research window. Its capture
    // remains available to the caller's finalization; no new policy was asked.
    result.terminal = hooks.terminal() || result.terminal;
    renderer->close();
    return result;
}
NATIVE_DIAG_POP
RunResult run_ride(runtime::NativeRideRuntime& ride,const ViewerOptions& options,
    runtime::RideControl control,const LifecycleHooks& lifecycle) {
    double front=0.,rear=0.;
    SessionHooks hooks;
    hooks.frame=[&]{return ride.render_frame();};
    hooks.advance=[&](std::int64_t target,double budget){(void)ride.advance(target,control,front,rear,budget);if(lifecycle.after_advance)lifecycle.after_advance();};
    hooks.terminal=[&]{return ride.snapshot().outcome.has_value();};
    hooks.reset=[&]{if(lifecycle.before_reset)lifecycle.before_reset();ride.reset();if(lifecycle.after_reset)lifecycle.after_reset();};
    hooks.brakes=[&](double f,double r){front=f;rear=r;};
    hooks.operator_event=lifecycle.operator_event;
    hooks.close_requested=lifecycle.close_requested;
    return run(std::move(hooks),ride.model_image(),options);
}
} // namespace bike::viewer
