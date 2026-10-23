#pragma once
#include "environment.hpp"
#include "policy.hpp"
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace bike::research {
struct OperatorEvent {
    std::int64_t event_id = 0, processed_step = 0;
    double time_s = 0., received_wall_s = 0.;
    std::string kind;
    bool queued = false, terminal = false, applied = false;
    io::Value values;
    std::int64_t episode_id = 0;
};

class PolicySession {
public:
    // Named caller ownership is restored if policy reset/metadata fails, so
    // the failed zero-step environment can still be sealed and closed.
    PolicySession(std::unique_ptr<runtime::NativeResearchRuntime>&&, std::unique_ptr<Policy>,
                  std::unique_ptr<RiderBehavior> rider_behavior = {});
    ~PolicySession();
    PolicySession(const PolicySession&) = delete;
    PolicySession& operator=(const PolicySession&) = delete;
    PolicySession(PolicySession&&) = delete;
    PolicySession& operator=(PolicySession&&) = delete;
    void begin_advance(std::optional<double> front_brake_demand = {}, std::optional<double> rear_brake_demand = {});
    [[nodiscard]] std::optional<runtime::ResearchTransition> advance_pending(
        std::optional<double> wall_budget_s = {}, std::optional<std::int64_t> target_step = {});
    [[nodiscard]] runtime::ResearchTransition advance(std::optional<double> front = {}, std::optional<double> rear = {});
    void run_until(std::optional<double> wall_limit_s = {});
    void request_stop();
    void pause();
    void resume();
    void set_brakes(double front = 0., double rear = 0.);
    void reset(std::optional<std::string> seed_decimal = {});
    [[nodiscard]] bool pending() const;
    [[nodiscard]] bool paused() const;
    [[nodiscard]] bool done() const;
    [[nodiscard]] runtime::NativeResearchRuntime& environment();
    [[nodiscard]] const runtime::NativeResearchRuntime& environment() const;
    [[nodiscard]] runtime::OwnedResearchEpisode seal_episode();
    [[nodiscard]] io::Value run_metadata() const;
    [[nodiscard]] const std::vector<OperatorEvent>& operator_events() const;
    [[nodiscard]] const std::vector<std::string>& cleanup_notes() const noexcept;
    // Recording owns these external-boundary hooks; never called in force loops.
    void set_reset_hooks(std::function<void()> before, std::function<void()> after);
    void close();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class PlaybackClock {
public:
    explicit PlaybackClock(double timestep_s);
    PlaybackClock(bool) = delete;
    void rebase(double now_s, std::int64_t step);
    void set_scale(int scale, double now_s, std::int64_t step);
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) deleted overload pins the live signature's (scale, now_s, step) order
    void set_scale(bool scale, double now_s, std::int64_t step) = delete;
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) deleted overload pins the live signature's (scale, now_s, step) order
    void set_scale(double scale, double now_s, std::int64_t step) = delete;
    void set_paused(bool paused, double now_s, std::int64_t step);
    [[nodiscard]] std::int64_t target(double now_s, std::int64_t completed_step);
    [[nodiscard]] double debt_s() const noexcept { return debt_s_; }
private:
    double timestep_s_, wall_s_ = 0., debt_s_ = 0.;
    std::int64_t step_ = 0;
    int scale_ = 1;
    bool paused_ = false, initialized_ = false;
};
} // namespace bike::research
