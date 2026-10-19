#include "application.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../platform/signals.hpp"
#include "../replay/playback.hpp"
#include "../replay/session.hpp"
#include "../viewer/session.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>

namespace bike::cli {
namespace {
int inspect(const Arguments& args, std::optional<std::int64_t> episode) {
    const double fps = args.real("fps", 60.);
    if (fps <= 0.) throw ArgumentError("--fps must be positive");
    replay::PlaybackSession playback(args.text("recording"), episode);
    platform::InterruptShield interrupt;
    if (args.flag("dry_run")) {
        const double count = std::floor(playback.duration_s() * fps) + 1.;
        if (!std::isfinite(count) || count >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
            throw ArgumentError("dry-run frame count exceeds range");
        const auto frames = static_cast<std::int64_t>(count);
        for (std::int64_t index = 0; index < frames; ++index) {
            if (interrupt.deferred()) return 130;
            (void)playback.frame(static_cast<double>(index) / fps);
        }
        std::cout << "frames=" << frames << " duration=" << std::fixed << std::setprecision(3)
                  << playback.duration_s() << "s ok\n";
        return 0;
    }
    double time = 0., speed = args.real("speed", 1.);
    std::int64_t tick = 0;
    int generation = 0;
    const bool loop = args.flag("loop");
    const auto rebase = [&] {
        if (generation == std::numeric_limits<int>::max()) throw std::overflow_error("playback generation exhausted");
        ++generation; tick = 0;
    };
    viewer::SessionHooks hooks;
    hooks.frame = [&] {
        auto frame = playback.frame(time, generation);
        // This cursor is a presentation clock, not a newly integrated state.
        // No replay certification is exposed from the inspection command.
        frame.state.step = tick; frame.state.time_s = time;
        return frame;
    };
    hooks.advance = [&](std::int64_t target, double) {
        if (target <= tick) return;
        time += static_cast<double>(target - tick) / fps * speed; tick = target;
        if (loop && playback.duration_s() > 0. && (time > playback.duration_s() || time < 0.)) {
            time = std::fmod(time, playback.duration_s());
            if (time < 0.) time += playback.duration_s();
            rebase();
        } else time = std::clamp(time, 0., playback.duration_s());
    };
    hooks.terminal = [&] { return !loop && (speed > 0. ? time >= playback.duration_s() : speed < 0. && time <= 0.); };
    hooks.close_requested = [&] { return interrupt.deferred() != 0; };
    hooks.stop = [] {};
    hooks.reset = [&] { time = 0.; rebase(); };
    hooks.key_event = [&](int key, int modifiers) {
        if (key == 262 || key == 263) {
            const double delta = (static_cast<unsigned>(modifiers) & 1U) != 0 ? .1 : 1.;
            time = std::clamp(time + (key == 262 ? delta : -delta), 0., playback.duration_s()); rebase(); return true;
        }
        if (key == 264 || key == 265) {
            const double next = key == 265 ? speed * 1.5 : speed / 1.5;
            if (std::isfinite(next)) { speed = next; rebase(); }
            return true;
        }
        return false;
    };
    viewer::ViewerOptions options;
    options.mode = viewer::Mode::inspection; options.timestep_s = 1. / fps; options.render_interval_s = 1. / fps;
    (void)viewer::run_replay(std::move(hooks), playback.model_image(), options, false);
    return interrupt.deferred() ? 130 : 0;
}
} // namespace

int replay_command(const Arguments& args) {
    if (args.has("time_scale") && !args.flag("viewer")) throw ArgumentError("--time-scale requires --viewer");
    if (args.flag("viewer") && args.flag("view")) throw ArgumentError("--viewer and --view are mutually exclusive");
    const auto episode = args.has("episode") ? std::optional(args.integer("episode")) : std::nullopt;
    try {
        if (args.flag("view")) return inspect(args, episode);
        platform::InterruptShield interrupt;
        replay::ReplaySession session(args.text("recording"), episode);
        std::exception_ptr failure;
        if (args.flag("viewer")) {
            viewer::SessionHooks hooks;
            hooks.frame = [&] { return session.render_frame(); };
            hooks.terminal = [&] { return session.done() || session.error().has_value(); };
            hooks.close_requested = [&] { return interrupt.deferred() != 0; };
            hooks.advance = [&](std::int64_t target, double budget) {
                try { (void)session.advance(budget, target); }
                catch (...) { failure = std::current_exception(); }
            };
            hooks.stop = [] {}; // Closing discards a pending replay suffix.
            hooks.reset = [&] { session.restart(); failure = {}; };
            viewer::ViewerOptions options;
            options.mode = viewer::Mode::checked_replay; options.timestep_s = session.timestep_s();
            options.time_scale = static_cast<unsigned>(args.integer("time_scale", 1));
            const auto& bytes = session.model_image();
            (void)viewer::run_replay(std::move(hooks), runtime::ModelImage{.mjb=bytes, .sha256=io::sha256(bytes)}, options, true);
        } else {
            try { while (!session.done() && !interrupt.deferred()) (void)session.advance(.008); }
            catch (...) { failure = std::current_exception(); }
        }
        const bool completed = session.done();
        const auto diagnostics = completed ? session.report() : session.diagnostics();
        const bool failed = session.error().has_value() || static_cast<bool>(failure);
        session.close();
        std::cout << io::recording_json(diagnostics) << '\n';
        if (interrupt.deferred()) return 130;
        return completed ? 0 : failed ? 1 : 2;
    } catch (const std::exception& error) {
        std::cerr << "REPLAY ERROR: " << error.what() << '\n'; return 1;
    }
}
} // namespace bike::cli
