#include "pacer.hpp"
#include "input.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::viewer {
struct Pacer::Impl {
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    explicit Impl(double timestep,double render_interval) : dt(timestep),render_interval_s(render_interval) {}
    double dt, render_interval_s, debt = 0.;
    unsigned scale = 1;
    bool paused = false, initialized = false, rebase_pending = false;
    std::uint64_t offered = 0;
    std::chrono::steady_clock::time_point wall{}, render{};
};
Pacer::Pacer(double dt,double render_interval) : impl_(std::make_unique<Impl>(dt,render_interval)) {
    if (!std::isfinite(dt) || dt <= 0.) throw std::invalid_argument("timestep_s must be finite and positive");
    if (!std::isfinite(render_interval) || render_interval <= 0.) throw std::invalid_argument("render_interval_s must be finite and positive");
}
Pacer::~Pacer() = default;
void Pacer::set_scale(unsigned scale) {
    (void)speed_for_action(Action::none, scale);
    if (impl_->scale != scale) { impl_->scale = scale; impl_->debt = 0.; impl_->offered = 0; impl_->rebase_pending = true; }
}
void Pacer::set_paused(bool paused) {
    if (impl_->paused != paused) { impl_->paused = paused; impl_->debt = 0.; impl_->offered = 0; impl_->rebase_pending = true; }
}
void Pacer::reset(std::chrono::steady_clock::time_point now) {
    impl_->wall = now; impl_->render = now; impl_->debt = 0.; impl_->offered = 0;
    impl_->initialized = true; impl_->rebase_pending = false;
}
PaceDecision Pacer::poll(std::chrono::steady_clock::time_point now) {
    auto& p = *impl_;
    if (!p.initialized) { reset(now); return {.physics_steps=0, .request_frame=true}; }
    if (now < p.wall) throw std::invalid_argument("playback monotonic clock moved backwards");
    const double elapsed = std::chrono::duration<double>(now - p.wall).count();
    p.wall = now;
    if (p.rebase_pending) { p.debt = 0.; p.rebase_pending = false; }
    else if (!p.paused) p.debt += elapsed * p.scale;
    if (!std::isfinite(p.debt)) throw std::overflow_error("playback pacing debt overflow");
    const bool render = std::chrono::duration<double>(now - p.render).count() >= p.render_interval_s;
    if (render) p.render = now;
    const double raw_steps = std::floor(p.debt / p.dt);
    // Preserve debt while bounding each cooperative offer. The runtime's 8 ms
    // wall budget can yield before the offer is finished; completed() debits
    // only actual physics steps, never the requested target or a control tick.
    constexpr std::uint64_t max_offer = 1000000;
    p.offered = p.paused || raw_steps <= 0. ? 0 :
        static_cast<std::uint64_t>(std::min(raw_steps, static_cast<double>(max_offer)));
    return {.physics_steps=p.offered, .request_frame=render};
}
void Pacer::completed(std::uint64_t actual) {
    auto& p = *impl_;
    if (actual > p.offered) throw std::invalid_argument("completed physics steps exceed pacing offer");
    p.debt = std::max(0., p.debt - static_cast<double>(actual) * p.dt);
    p.offered -= actual;
}
double Pacer::debt_s() const noexcept { return impl_->debt; }
double Pacer::debt_s_at(std::chrono::steady_clock::time_point now) const {
    const auto& p = *impl_;
    if (!p.initialized) return 0.;
    if (now < p.wall) throw std::invalid_argument("pacing debt observation clock moved backwards");
    const double elapsed = std::chrono::duration<double>(now - p.wall).count();
    const double debt = p.debt + (!p.paused && !p.rebase_pending ? elapsed * p.scale : 0.);
    if (!std::isfinite(debt)) throw std::overflow_error("pacing debt observation overflow");
    return debt;
}

PacingMeasurement::PacingMeasurement(TimePoint start, double simulation_s,
    double render_interval_s, unsigned scale, bool hidden)
    : start_(start), observed_(start), last_frame_(start), last_simulation_s_(simulation_s) {
    if (!std::isfinite(simulation_s) || !std::isfinite(render_interval_s) || render_interval_s <= 0.)
        throw std::invalid_argument("pacing measurement requires finite simulation time and positive finite render interval");
    (void)speed_for_action(Action::none, scale);
    report_.render_interval_s = render_interval_s;
    report_.initial_scale = scale;
    report_.scaled = scale != 1;
    report_.hidden = hidden;
}
void PacingMeasurement::observe(TimePoint now, double simulation_s, double debt_s) {
    if (report_.stopped) return; // terminal idle time never enters the window
    if (now < observed_ || !std::isfinite(simulation_s) || simulation_s < last_simulation_s_ ||
        !std::isfinite(debt_s) || debt_s < 0.)
        throw std::invalid_argument("pacing measurement requires monotone finite clocks and nonnegative debt");
    const double simulated = report_.simulated_s + (simulation_s - last_simulation_s_);
    const double wall = std::chrono::duration<double>(now - start_).count();
    if (!std::isfinite(simulated) || !std::isfinite(wall))
        throw std::invalid_argument("pacing measurement elapsed clocks overflow");
    report_.simulated_s = simulated;
    report_.wall_s = wall;
    report_.debt_s = debt_s;
    report_.max_debt_s = std::max(report_.max_debt_s, debt_s);
    observed_ = now;
    last_simulation_s_ = simulation_s;
}
void PacingMeasurement::mode(bool paused, unsigned scale) {
    (void)speed_for_action(Action::none, scale);
    if (report_.stopped) return;
    report_.paused = report_.paused || paused;
    report_.scaled = report_.scaled || scale != 1;
}
void PacingMeasurement::reset(TimePoint now, double simulation_s) {
    if (!std::isfinite(simulation_s) || now < observed_)
        throw std::invalid_argument("pacing reset requires a monotone wall clock and finite simulation time");
    if (report_.resets == std::numeric_limits<std::uint64_t>::max())
        throw std::overflow_error("pacing reset counter overflow");
    ++report_.resets;
    // A post-terminal reset invalidates attachment to any later episode, while
    // preserving the first window's observed timing instead of counting idle.
    if (report_.stopped) return;
    observed_ = now;
    last_simulation_s_ = simulation_s;
    report_.wall_s = std::chrono::duration<double>(now - start_).count();
}
void PacingMeasurement::presented(TimePoint render_started, TimePoint now,
    double simulation_s, double debt_s) {
    if (report_.stopped) return;
    if (render_started < last_frame_ || render_started > now)
        throw std::invalid_argument("pacing frame render timestamps are invalid");
    const double render_wall = std::chrono::duration<double>(now - render_started).count();
    const double interval = std::chrono::duration<double>(now - last_frame_).count();
    const double lateness = report_.presented_frames == 0 ? 0. : std::max(0., interval - report_.render_interval_s);
    if (report_.presented_frames == std::numeric_limits<std::uint64_t>::max() ||
        !std::isfinite(report_.render_wall_s + render_wall) ||
        !std::isfinite(report_.total_frame_lateness_s + lateness))
        throw std::overflow_error("pacing frame statistics overflow");
    observe(now, simulation_s, debt_s);
    if (report_.presented_frames == 0)
        report_.first_frame_delay_s = std::chrono::duration<double>(now - start_).count();
    report_.render_wall_s += render_wall;
    ++report_.presented_frames;
    if (lateness > 0.) ++report_.late_frames;
    report_.total_frame_lateness_s += lateness;
    report_.max_frame_lateness_s = std::max(report_.max_frame_lateness_s, lateness);
    last_frame_ = now;
}
void PacingMeasurement::unavailable_framebuffer() noexcept {
    if (!report_.stopped) report_.framebuffer_unavailable = true;
}
void PacingMeasurement::stop(TimePoint now, double simulation_s, double debt_s, bool terminal) {
    if (report_.stopped) return;
    observe(now, simulation_s, debt_s);
    report_.terminal = terminal;
    report_.stopped = true;
}
PacingReport PacingMeasurement::report() const {
    auto result = report_;
    if (result.wall_s > 0.) {
        const double rtf = result.simulated_s / result.wall_s;
        const double fps = static_cast<double>(result.presented_frames) / result.wall_s;
        if (std::isfinite(rtf)) result.mean_achieved_rtf = rtf;
        if (std::isfinite(fps)) result.frames_per_s = fps;
    }
    if (result.presented_frames > 1)
        result.mean_frame_lateness_s = result.total_frame_lateness_s /
            static_cast<double>(result.presented_frames - 1);
    result.one_x_eligible = result.stopped && result.terminal &&
        result.simulated_s > 0. && result.mean_achieved_rtf.has_value() &&
        result.presented_frames > 0 && result.frames_per_s.has_value() &&
        !result.hidden && !result.paused && !result.scaled &&
        !result.framebuffer_unavailable && result.resets == 0;
    return result;
}
AchievedRate::AchievedRate(std::chrono::steady_clock::time_point now, double simulation_s)
    : wall_(now), simulation_s_(simulation_s) { rebase(now, simulation_s); }
void AchievedRate::rebase(std::chrono::steady_clock::time_point now, double simulation_s) {
    if (!std::isfinite(simulation_s)) throw std::invalid_argument("simulation time must be finite");
    wall_ = now; simulation_s_ = simulation_s; value_.reset();
}
std::optional<double> AchievedRate::update(std::chrono::steady_clock::time_point now, double simulation_s) {
    if (!std::isfinite(simulation_s) || simulation_s < simulation_s_ || now < wall_)
        throw std::invalid_argument("achieved rate requires monotone finite clocks; rebase after reset");
    const double elapsed = std::chrono::duration<double>(now - wall_).count();
    if (elapsed >= .5) { value_ = (simulation_s - simulation_s_) / elapsed; wall_ = now; simulation_s_ = simulation_s; }
    return value_;
}
} // namespace bike::viewer
