// rider/launch_plan.hpp — the rider's standstill launch (launch plan,
// part 3b): hold the brakes and press, release by the felt crank torque,
// accelerate with the rider's own upshift requests, then hand effort,
// lean and shifting back to the ordinary policy. Runs on the delayed
// percepts and the shifter's mechanical answer; times count intention
// ticks. Off (always idle) unless rider_launch.plan_enabled.
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

#include "launch_config.hpp"
#include "perception.hpp"

namespace rider {

enum class LaunchPhase : std::uint8_t { idle, hold, release, accelerate, handover, blend, done };
[[nodiscard]] const char *launch_phase_name(LaunchPhase phase);   // throws on a value outside the enum
[[nodiscard]] LaunchPhase launch_phase(std::string_view name);    // throws on an unknown name

enum class ShiftResponse : std::uint8_t { none, accepted, deferred, rejected };
// The shifter's rider_response label ("none", "accepted", "deferred",
// "rejected"); throws on anything else.
[[nodiscard]] ShiftResponse shift_response(std::string_view label);

// The shifter state after the previous physics step: the mechanical answer
// of lever and chain, not a delayed percept.
struct ShiftFeedback {
    ShiftResponse response = ShiftResponse::none;
    std::int64_t shift_count = 0;   // committed shifts so far
    bool cut_active = false;        // drivetrain::shift_cut_active (the ideal hub sweeps with it)
};

struct LaunchInputs {
    std::optional<Percepts> percepts;   // absent until the reaction delay has passed
    ShiftFeedback shift;
    bool eligible = false;         // active intent, rider enabled, effort drive mode, no caller effort
    bool external_brake = false;   // the caller brakes
};

struct LaunchCommand {
    LaunchPhase phase = LaunchPhase::idle;
    double brake = 0.;             // both wheels, [0, 1]
    double effort_nm = 0.;         // the plan's effort ceiling
    double weight = 0.;            // the plan's share of effort and lean target
    double lean_target_rad = 0.;
    bool shift_up = false;         // the pending upshift request
    bool owns_shifting = false;
    bool preload_missed = false;   // the hold ran out before the felt torque came
};

class LaunchPlan {
public:
    struct State {
        LaunchPhase phase = LaunchPhase::idle;
        std::int64_t phase_ticks = 0;    // hold: percept ticks without a caller brake; release: ticks
        std::int64_t launch_ticks = 0;   // ticks since the release began
        std::int64_t end_ok_ticks = 0;   // consecutive percepts at the end speed and cadence
        std::int64_t blend_ticks = 0;
        double effort_nm = 0.;
        double brake = 0.;
        double lean_target_rad = 0.;
        bool shift_up = false;
        std::int64_t request_count = 0;  // shift_count when the pending request was issued
        int request_teeth = 0;           // felt cog when the pending request was issued
        int blocked_teeth = 0;           // felt cog of the last answered request; 0 = none
        bool preload_missed = false;
        // Launch part 4: intention ticks since the felt speed first reached
        // effort_rise_speed_mps in this launch; -1 = not yet.
        std::int64_t rise_ticks = -1;
        // launch_ticks at which a rejected request's same-cog block ends;
        // -1 = the block lasts until the felt cog changes.
        std::int64_t retry_tick = -1;
        bool operator==(const State &) const = default;
    };
    LaunchPlan(LaunchConfig config, double period_s);
    void reset();
    // A standing episode start: hold with the prepared lean before any
    // percept. No-op (and false) when the plan is switched off.
    bool arm_start(double lean_rad);
    // Rejects nonfinite percept fields and a negative shift count without
    // changing anything.
    static void validate(const LaunchInputs &inputs);
    LaunchCommand update(const LaunchInputs &inputs);
    [[nodiscard]] LaunchCommand command() const;
    [[nodiscard]] State state() const;
    [[nodiscard]] int launch_teeth(double grade_ahead) const;
    // Rejects every state update() cannot leave behind (decision 11); a
    // rejected state leaves the plan unchanged.
    void restore(const State &state);

private:
    [[nodiscard]] double cap(SurfaceLabel surface) const;
    [[nodiscard]] double ramp(double effort, const Percepts &percepts, State &n) const;
    [[nodiscard]] bool standstill(const Percepts &percepts) const;
    void arm(State &n, double lean_rad) const;
    void to_release(State &n) const;
    void shifting(State &n, const Percepts &percepts, const ShiftFeedback &shift) const;

    LaunchConfig config_;
    double period_s_;
    std::int64_t release_ticks_ = 0, max_hold_ticks_ = 0, end_hold_ticks_ = 0, max_launch_ticks_ = 0,
                 handover_ticks_ = 0, retry_ticks_ = 0;
    State s_;
};

} // namespace rider
