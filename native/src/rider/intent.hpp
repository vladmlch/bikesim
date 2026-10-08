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
#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

#include "rider_posture.hpp"

namespace rider {

// seated_climb.py SeatedClimbConfig — validated at wire decode
// (parse_intent_config) exactly like the frozen dataclass's __post_init__.
struct IntentConfig {
    bool enabled = false;
    double period_s = .01;
    double reaction_delay_s = .15;
    double target_crank_power_w = 225.;
    double max_crank_torque_nm = 60.;
    double torque_slew_nm_s = 300.;
    double lean_gain = 1.;
    double max_forward_lean_rad = .35;
    double max_backward_lean_rad = .10;
    double lean_rate_rad_s = .5;
    double orientation_tau_s = .5;
    double surge_power_w = 400.;
    double surge_grade = .20;
    double surge_budget_s = 15.;
    double surge_recovery_rate = 1. / 3.;
    double front_load_share_target = .30;
    double lean_trim_gain_rad_s = .3;
    double lean_trim_limit_rad = .15;
    double trim_dead_time_s = .2;
};

// seated_climb.py SeatedClimbSignals — validated at wire decode like the
// frozen dataclass's __post_init__.
struct IntentSignals {
    double pitch_rate_up_rad_s = 0.;
    std::array<double, 3> specific_force_body_mps2{0., 0., 0.};
    double crank_rate_rad_s = 0.;
    double human_crank_torque_nm = 0.;
    std::optional<double> front_load_share;
};

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

    explicit SeatedPostureProgram(const IntentConfig &config);

    void reset();
    void schedule_pulse(double start_s, double duration_s,
                        double amplitude_rad);
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    double update(double time_s, double road_grade, double dt_s,
                  std::optional<double> front_load_share = std::nullopt,
                  std::optional<double> lean_limit_rad = std::nullopt,
                  std::optional<double> dead_time_s = std::nullopt);

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
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    double power_target_w(double preview_grade, double dt_s);
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed positional Python signature
    [[nodiscard]] Intent update(
        const IntentSignals &signals, double dt_s, double road_grade,
        std::optional<double> preview_grade = std::nullopt,
        std::optional<double> lean_limit_rad = std::nullopt);

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
    };

    RiderIntent(IntentConfig config, double dt_s);

    void reset();
    void schedule_pulse(double start_s, double duration_s,
                        double amplitude_rad);

    // resolve() — probe calls (advance=false) run on a value copy, exactly
    // like copy.deepcopy(self).resolve(...): clocks, budget and queues are
    // preserved. `step` is the physics step counter; the wire layer rejects
    // non-int values before this point (type(step) is int in the oracle).
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python call signature
    [[nodiscard]] IntentControl
    resolve(IntentControl control, const IntentSignals &signals,
            std::int64_t step, double road_grade,
            std::optional<double> preview_grade = std::nullopt,
            std::optional<double> lean_limit_rad = std::nullopt,
            bool active = true, bool advance = true);

    [[nodiscard]] State state() const;
    void restore(const State &state);
    [[nodiscard]] const Intent &intent() const { return intent_; }
    [[nodiscard]] const IntentConfig &config() const { return config_; }
    [[nodiscard]] const SeatedClimbPolicy &policy() const { return policy_; }
    [[nodiscard]] SeatedClimbPolicy &policy() { return policy_; }
    [[nodiscard]] std::int64_t period_steps() const { return period_steps_; }

private:
    IntentConfig config_;
    double dt_s_;
    std::int64_t period_steps_;
    SeatedClimbPolicy policy_;
    std::int64_t last_tick_step_ = -1;
    Intent intent_{};
};

} // namespace rider
