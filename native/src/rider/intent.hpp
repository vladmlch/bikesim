// rider/intent.hpp — port of the seated-climb intent stack:
// rider_intent.py RiderIntentResolver over seated_climb.py
// SeatedClimbPolicy and rider_program.py SeatedPostureProgram, for the
// supported welded configuration.
//
// The resolver owns the integer-step acquisition clock; the policy owns
// the inclination estimate, surge budget and effort slew; the program
// owns rate-limited lean intent, the delayed front-load-share trim and
// scheduled body pulses. Statement order and lazy-validation placement
// mirror the Python sources — resolve() arguments are validated where the
// oracle validates them, not earlier. Pure C++: no nanobind or mujoco
// types enter this header; dict decoding lives in intent_wire.*.
//
// Launch part 3a: perception and the front-lift reaction ride inside the
// resolver's Core double buffer (decision 11): every tick mutates a staged
// copy and commits by a nonthrowing swap.
#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "front_lift_reaction.hpp"
#include "intent_signals.hpp"
#include "launch_plan.hpp"
#include "perception.hpp"
#include "rider_posture.hpp"

namespace rider {

// seated_climb.py SeatedClimbConfig — validated at wire decode
// (parse_intent_config) exactly like the frozen dataclass's __post_init__.
struct IntentConfig {
    bool enabled = false;
    double period_s = -1.;
    double reaction_delay_s = -1.;
    double target_crank_power_w = 225.;
    double max_crank_torque_nm = -1.;
    double lean_gain = 1.;
    double max_forward_lean_rad = -1.;
    double max_backward_lean_rad = .10;
    double lean_rate_rad_s = -1.;
    double orientation_tau_s = -1.;
    double surge_power_w = 400.;
    double surge_grade = .20;
    double surge_budget_s = -1.;
    double surge_recovery_rate = 1. / 3.;
    double front_load_share_target = .30;
    double lean_trim_gain_rad_s = .3;
    double lean_trim_limit_rad = .15;
    double trim_dead_time_s = -1.;
    double min_elbow_flexion_deg = 0.;
    // Low-speed seated climb ceiling by delayed grade_ahead, linearly blended
    // from climb_effort_cap_grade_nm at <= climb_effort_rise_speed_mps up to
    // max_crank_torque_nm at >= climb_effort_full_speed_mps.
    std::vector<double> climb_effort_cap_grades;
    std::vector<double> climb_effort_cap_grade_nm;
    double climb_effort_rise_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double climb_effort_full_speed_mps = std::numeric_limits<double>::quiet_NaN();
};

// Validates the speed+grade climb effort cap fields on IntentConfig.
void validate_climb_effort_cap(const IntentConfig &config);

// seated_climb.py SeatedClimbIntent.
struct Intent {
    RiderPosture posture;
    double effort_ceiling_nm = 0.;
};

// control.py RideControl — the full wire shape. resolve() merges posture
// and human_torque_nm only; the other command fields pass through
// untouched like dataclasses.replace.
struct IntentControl {
    std::optional<double> motor_torque_nm;
    std::optional<double> motor_limit_nm;
    std::optional<double> human_torque_nm;
    std::optional<double> crank_target_rate_rad_s;
    std::optional<RiderPosture> posture;
    bool rider_enabled = true;
};

struct IntentResolution {
    IntentControl control;
    bool effort_limited = false;   // the rider's own reduction is in force
};

// Launch part 1, task 4 — the rider-owned command set. Part 1's intent
// always resolves to the empty commands below (no brakes, no shift
// request, shifting owned by the caller's cadence shifter); the channel
// exists so runtime, recording and tests carry the real shape before a
// later part teaches the intent to fill it. ShiftRequest up moves one
// cog smaller (a harder gear), down one cog larger — the same convention
// the cadence shifter's direction labels already use.
enum class ShiftRequest : std::uint8_t { up, down };
struct RiderCommands {
    double front_brake = 0.;
    double rear_brake = 0.;
    std::optional<ShiftRequest> shift_request = std::nullopt;
    bool owns_shifting = false;
    std::optional<int> stationary_rear_teeth = std::nullopt;
};
// Brake demands are fractions in [0, 1] — validate rejects nonfinite or
// out-of-range values with std::invalid_argument, the same error every
// other input-boundary check raises.
void validate(const RiderCommands &commands);

// Launch part 3b, decision 8: the launch plan's lean target and its share.
// The program keeps its rate and bounds for every target it is given.
struct LeanOverride {
    double target_rad = 0.;
    double weight = 1.;   // (0, 1]
};

// Launch part 3b: what the step tells the resolver about the launch.
struct LaunchContext {
    ShiftFeedback shift;          // the shifter after the previous physics step
    bool effort_mode = false;     // drive_mode is articulated_effort or crank_effort
    bool external_brake = false;  // the caller brakes this step
};

// rider_program.py body_pulse — a smooth finite torso-thrust wish. Runs
// its input validation on every call exactly like the oracle.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
[[nodiscard]] double body_pulse(double time_s, double start_s,
                                double duration_s, double amplitude_rad);

// rider_program.py SeatedPostureProgram.
class SeatedPostureProgram {
public:
    // program.state_dict() — owned snapshot fields.
    struct State {
        double lean_rad = 0.;
        double trim_rad = 0.;
        // (time_s, front_load_share|None) in append order.
        std::deque<std::pair<double, std::optional<double>>> load_samples;
        std::optional<double> delayed_load_share;
        // (start_s, duration_s, amplitude_rad) in append order.
        std::vector<std::array<double, 3>> pulses;
    };

    explicit SeatedPostureProgram(IntentConfig config);

    void reset();
    void seed_lean(double lean_rad);
    void schedule_pulse(double start_s, double duration_s,
                        double amplitude_rad);
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    double update(double time_s, double road_grade, double dt_s,
                  std::optional<double> front_load_share = std::nullopt,
                  std::optional<double> lean_limit_rad = std::nullopt,
                  std::optional<double> dead_time_s = std::nullopt,
                  std::optional<double> min_lean_rad = std::nullopt,
                  double extra_target_rad = 0.,
                  std::optional<LeanOverride> launch = std::nullopt);

    [[nodiscard]] State state() const;
    void restore(const State &state);
    [[nodiscard]] double lean_rad() const { return lean_rad_; }

private:
    IntentConfig config_;
    double lean_rad_ = 0.;
    double trim_rad_ = 0.;
    std::deque<std::pair<double, std::optional<double>>> load_samples_;
    std::optional<double> delayed_load_share_;
    std::vector<std::array<double, 3>> pulses_;
};

// seated_climb.py SeatedClimbPolicy.
class SeatedClimbPolicy {
public:
    // policy.state_dict() — owned snapshot fields.
    struct State {
        double time_s = 0.;
        double inclination_rad = 0.;
        double lean_rad = 0.;
        double effort_nm = 0.;
        // (time_s, signals) in append order.
        std::deque<std::pair<double, IntentSignals>> samples;
        std::optional<IntentSignals> delayed;
        double surge_budget_s_left = 0.;
    };

    explicit SeatedClimbPolicy(const IntentConfig &config);

    void reset();
    void seed_lean(double lean_rad);
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    double power_target_w(double preview_grade, double dt_s, bool spend_surge = true);
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    [[nodiscard]] Intent update(
        const IntentSignals &signals, double dt_s, double road_grade,
        std::optional<double> preview_grade = std::nullopt,
        std::optional<double> lean_limit_rad = std::nullopt,
        std::optional<double> min_lean_rad = std::nullopt,
        double extra_lean_rad = 0., bool spend_surge = true,
        std::optional<LeanOverride> launch = std::nullopt);

    [[nodiscard]] const IntentConfig &config() const { return config_; }
    [[nodiscard]] SeatedPostureProgram &program() { return program_; }
    [[nodiscard]] const SeatedPostureProgram &program() const {
        return program_;
    }
    [[nodiscard]] State state() const;
    void restore(const State &state);
    [[nodiscard]] double inclination_rad() const { return inclination_rad_; }

private:
    IntentConfig config_;
    SeatedPostureProgram program_;
    double time_s_ = 0.;
    double inclination_rad_ = 0.;
    double lean_rad_ = 0.;
    double effort_nm_ = 0.;
    std::deque<std::pair<double, IntentSignals>> samples_;
    std::optional<IntentSignals> delayed_;
    double surge_budget_s_left_ = 0.;
};

// rider_intent.py RiderIntentResolver — the physics-clock owner.
class RiderIntent {
public:
    // resolver.state_dict() — owned snapshot fields.
    struct State {
        std::int64_t last_tick_step = -1;
        Intent intent;
        SeatedClimbPolicy::State policy;
        SeatedPostureProgram::State program;
        std::optional<Perception::State> perception;        // engaged exactly when the intent is enabled
        std::optional<FrontLiftReaction::State> reaction;   // engaged exactly when the intent is enabled
        std::optional<LaunchPlan::State> launch;            // engaged exactly when the intent is enabled
    };

    // Replaces RiderIntent(IntentConfig, double): the reaction and launch
    // tables have no in-code default.
    RiderIntent(IntentConfig config, FrontLiftReactionConfig reaction,
                const LaunchConfig &launch, double dt_s);
    RiderIntent(const RiderIntent &);
    RiderIntent &operator=(const RiderIntent &);
    RiderIntent(RiderIntent &&) noexcept;
    RiderIntent &operator=(RiderIntent &&) noexcept;
    ~RiderIntent() = default;

    void reset();
    // Starts the posture from the prepared start lean after reset(): the
    // policy, its posture program and the held intent continue from it.
    void seed_lean(double lean_rad);
    // Launch part 3b, decision 7: arms the launch plan for a standing
    // episode start — hold with the prepared lean before any percept.
    // False (and no state change) when the plan is off or the intent is
    // disabled.
    bool arm_standstill_start(double lean_rad);
    void schedule_pulse(double start_s, double duration_s,
                        double amplitude_rad);

    // resolve() — probe calls (advance=false) run on a value copy, exactly
    // like copy.deepcopy(self).resolve(...): clocks, budget and queues are
    // preserved. `step` is the physics step counter; the wire layer rejects
    // non-int values before this point (type(step) is int in the oracle).
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python call signature
    [[nodiscard]] IntentResolution
    resolve_full(IntentControl control, const IntentSignals &signals,
                 const RiderSenses &senses, std::int64_t step,
                 double road_grade,
                 std::optional<double> preview_grade = std::nullopt,
                 std::optional<double> lean_limit_rad = std::nullopt,
                 bool active = true, bool advance = true,
                 std::optional<double> min_lean_rad = std::nullopt,
                 const LaunchContext &launch = {});
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python call signature
    [[nodiscard]] IntentControl
    resolve(IntentControl control, const IntentSignals &signals,
            const RiderSenses &senses, std::int64_t step, double road_grade,
            std::optional<double> preview_grade = std::nullopt,
            std::optional<double> lean_limit_rad = std::nullopt,
            bool active = true, bool advance = true,
            std::optional<double> min_lean_rad = std::nullopt,
            const LaunchContext &launch = {});   // resolve_full(...).control

    [[nodiscard]] State state() const;
    void restore(const State &state);
    [[nodiscard]] const Intent &intent() const { return core_->intent; }
    // The reaction's current command; neutral (idle, false, 1, 0) when the
    // intent is disabled.
    [[nodiscard]] ReactionCommand reaction() const;
    // Launch part 3b: the launch plan's current command; neutral when the
    // plan or the intent is off.
    [[nodiscard]] LaunchCommand launch() const;
    // Launch part 1, tasks 4-5: the rider-owned commands the resolver
    // currently resolves to — always the empty set in part 1; part 3b
    // derives them from the launch plan's command on the resolve()
    // cadence. Task 5 already transports whatever this returns — brakes,
    // owns_shifting, shift_request — through PhysicalStep into the
    // drivetrain's cadence shifter.
    [[nodiscard]] const RiderCommands &commands() const noexcept {
        return core_->commands;
    }
    [[nodiscard]] const IntentConfig &config() const { return config_; }
    [[nodiscard]] const SeatedClimbPolicy &policy() const { return core_->policy; }
    [[nodiscard]] SeatedClimbPolicy &policy() { return core_->policy; }
    [[nodiscard]] std::int64_t period_steps() const { return period_steps_; }

private:
    // Launch part 3a, decision 11: everything a tick changes. The tick runs
    // on staged_ (copy-assigned from core_, so its queues keep their blocks
    // and a warm tick does not allocate) and commits by a nonthrowing swap.
    // SeatedClimbPolicy and SeatedPostureProgram are not transactional on
    // their own; the transaction boundary is RiderIntent.
    struct Core {
        explicit Core(const IntentConfig &config) : policy(config) {}
        SeatedClimbPolicy policy;
        std::optional<Perception> perception;
        std::optional<FrontLiftReaction> reaction;
        // Launch part 3b: the plan and the commands derived from it ride
        // the same transaction — a tick's command set commits with the
        // plan state that produced it.
        std::optional<LaunchPlan> launch;
        RiderCommands commands;
        std::int64_t last_tick_step = -1;
        Intent intent;
    };
    // The commit is a pointer swap: unconditionally noexcept on every
    // toolchain (libstdc++'s deque move constructor is not noexcept, so a
    // by-value Core swap could not guarantee the all-or-nothing commit).
    static_assert(std::is_nothrow_swappable_v<std::unique_ptr<Core>>);
    // One tick on `core` (called on *staged_ or on a probe copy).
    void tick(Core &core, const IntentControl &control,
              const IntentSignals &signals,
              const RiderSenses &senses, std::int64_t step,
              double road_grade, std::optional<double> preview_grade,
              std::optional<double> lean_limit_rad,
              std::optional<double> min_lean_rad,
              const LaunchContext &ctx) const;
    // The output merge from `core`'s intent and reaction.
    static IntentResolution output(const Core &core, IntentControl control);

    IntentConfig config_;
    double dt_s_;
    std::int64_t period_steps_;
    std::unique_ptr<Core> core_;
    std::unique_ptr<Core> staged_;
};

} // namespace rider
