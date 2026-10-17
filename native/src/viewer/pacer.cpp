#include "pacer.hpp"
#include "input.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::viewer {
struct Pacer::Impl {
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
    if (!p.initialized) { reset(now); return {0, true}; }
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
    return {p.offered, render};
}
void Pacer::completed(std::uint64_t actual) {
    auto& p = *impl_;
    if (actual > p.offered) throw std::invalid_argument("completed physics steps exceed pacing offer");
    p.debt = std::max(0., p.debt - static_cast<double>(actual) * p.dt);
    p.offered -= actual;
}
double Pacer::debt_s() const noexcept { return impl_->debt; }
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
