// perception.hpp — the rider's delayed, periodic senses adapter (launch
// part 1, task 3).
//
// Perception is the component-level twin of the intent policy's delayed
// signal queue (intent.cpp's samples_/delayed_ pair): the caller hands
// it one SenseSample per intention period, and it returns the sample
// exactly reaction_delay_s old — or nothing until that much time has
// passed. One reaction delay covers every sensation, grade included;
// the vestibular pitch is integrated from the delayed rate.
//
// Sample-time convention (review M-P2): every field of a SenseSample is
// taken at the SAME instant — the start of the interval it describes.
// That is the step's own convention: update_rider_senses samples every
// RiderSenses field at the interval-start state (step.cpp's fold
// comment), and the step hands it, at each intention tick, the senses of
// the interval that just ended — taken at that interval's start — so the
// percepts lag reaction_delay_s plus one physics step (launch part 3a,
// decision 4). The adapter stamps each sample with its interval-start
// tick, so the Percepts returned at interval start n carry the
// acquisition of interval start n - delay_ticks.
//
// The clock is an integer tick count (follow-up 2): a double clock
// accumulated by += drifts off the period grid and makes restore-time
// validation depend on accumulated error; integer ticks are exact.
// Where a wall time is ever needed it is derived as tick * period_s —
// closed form, never accumulated.
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

#include "intent_signals.hpp"
#include "senses.hpp"

namespace rider {

// One acquisition the rider perceives later: the intent signals plus
// the senses of the same interval, time-aligned (grade included).
struct SenseSample {
    IntentSignals signals;
    RiderSenses senses;
};

struct Percepts {
    RiderSenses senses;      // delayed, including grade_ahead
    double crank_rpm = 0.;   // |crank_rate_rad_s| * 60 / 2pi, delayed
    double human_crank_torque_nm = 0.;
    double pitch_rate_up_rad_s = 0.;
    double pitch_rad = 0.;   // integrated from the delayed rate since reset
};

class Perception {
public:
    struct State {
        std::int64_t tick = 0;   // updates since reset
        std::deque<std::pair<std::int64_t, SenseSample>> samples;
        std::optional<SenseSample> delayed;
        double pitch_rad = 0.;
        std::optional<double> initial_grade_rad{};
    };
    // reaction_delay_s must be a nonnegative multiple of period_s (the
    // intent.cpp:382-389 rule); the integer delay tick count follows.
    Perception(double reaction_delay_s, double period_s);
    void reset();
    // Called once per intention period at the interval start. Returns
    // nullopt until the first sample is delay_ticks old. A sample
    // outside the accepted domain mutates nothing.
    [[nodiscard]] std::optional<Percepts> update(const SenseSample &sample);
    [[nodiscard]] State state() const;
    // Validates the whole state in O(queue) — nonnegative tick, the
    // queue exactly the contiguous not-yet-due tail
    // tick-size..tick-1, the delayed slot consistent with the tick, all
    // sample contents inside the readers' domains — then commits
    // atomically (P10).
    void restore(const State &state);
    [[nodiscard]] std::int64_t delay_ticks() const noexcept { return delay_ticks_; }

private:
    std::int64_t delay_ticks_ = 0;
    double period_s_ = 0.;
    State s_;
};

} // namespace rider
