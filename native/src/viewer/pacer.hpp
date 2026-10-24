#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

namespace bike::viewer {
inline constexpr double kComputeSliceS = .008;
inline constexpr double kRenderIntervalS = 1. / 60.;
struct PaceDecision { std::uint64_t physics_steps; bool request_frame; };
class Pacer {
public:
    explicit Pacer(double dt, double render_interval_s = kRenderIntervalS);
    ~Pacer();
    Pacer(const Pacer&) = delete;
    Pacer& operator=(const Pacer&) = delete;
    Pacer(Pacer&&) = delete;
    Pacer& operator=(Pacer&&) = delete;
    void set_scale(unsigned scale);
    void set_paused(bool paused);
    [[nodiscard]] PaceDecision poll(std::chrono::steady_clock::time_point now);
    void completed(std::uint64_t actual_steps);
    void reset(std::chrono::steady_clock::time_point now);
    [[nodiscard]] double debt_s() const noexcept;
    // Observational debt at an arbitrary later timestamp. This neither offers
    // nor completes physics steps and leaves the scheduling arithmetic alone.
    [[nodiscard]] double debt_s_at(std::chrono::steady_clock::time_point now) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// One real display window, frozen after its terminal frame. These are timing
// observations only: physical validity and full V10 qualification are separate.
struct PacingReport {
    double wall_s = 0., simulated_s = 0., render_wall_s = 0.;
    double debt_s = 0., max_debt_s = 0.;
    double first_frame_delay_s = 0., max_frame_lateness_s = 0., total_frame_lateness_s = 0.;
    double render_interval_s = kRenderIntervalS;
    std::uint64_t presented_frames = 0, late_frames = 0, resets = 0;
    unsigned initial_scale = 1;
    bool stopped = false, terminal = false, hidden = false;
    bool paused = false, scaled = false, framebuffer_unavailable = false;
    bool one_x_eligible = false;
    std::optional<double> mean_achieved_rtf, frames_per_s, mean_frame_lateness_s;
};

class PacingMeasurement {
public:
    using TimePoint = std::chrono::steady_clock::time_point;
    PacingMeasurement(TimePoint start, double simulation_s,
        double render_interval_s = kRenderIntervalS, unsigned scale = 1, bool hidden = false);
    void observe(TimePoint now, double simulation_s, double debt_s);
    void mode(bool paused, unsigned scale);
    void reset(TimePoint now, double simulation_s);
    // Call only after a positive-size draw and its successful swap. The first
    // presentation delay is separate; subsequent lateness is max(0, actual
    // presentation interval - requested render interval), not physics latency.
    void presented(TimePoint render_started, TimePoint now,
        double simulation_s, double debt_s);
    void unavailable_framebuffer() noexcept;
    void stop(TimePoint now, double simulation_s, double debt_s, bool terminal);
    [[nodiscard]] PacingReport report() const;
private:
    TimePoint start_, observed_, last_frame_;
    double last_simulation_s_;
    PacingReport report_;
};

class AchievedRate {
public:
    AchievedRate(std::chrono::steady_clock::time_point now, double simulation_s);
    void rebase(std::chrono::steady_clock::time_point now, double simulation_s);
    [[nodiscard]] std::optional<double> update(std::chrono::steady_clock::time_point now, double simulation_s);
private:
    std::chrono::steady_clock::time_point wall_;
    double simulation_s_;
    std::optional<double> value_;
};
} // namespace bike::viewer
