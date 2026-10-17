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
    void set_scale(unsigned scale);
    void set_paused(bool paused);
    [[nodiscard]] PaceDecision poll(std::chrono::steady_clock::time_point now);
    void completed(std::uint64_t actual_steps);
    void reset(std::chrono::steady_clock::time_point now);
    [[nodiscard]] double debt_s() const noexcept;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
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
