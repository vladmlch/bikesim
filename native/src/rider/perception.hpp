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
// comment), and update() is called once per intention period at an
// interval start. The adapter stamps each sample with its
// interval-start clock, so the Percepts returned at interval start T
// carry the acquisition of interval start T - reaction_delay_s.
//
// Part 1 carries the record only — no rider decision consumes Percepts.
#pragma once

#include <deque>
#include <optional>
#include <utility>

#include "intent.hpp"
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
        double time_s = 0.;
        std::deque<std::pair<double, SenseSample>> samples;
        std::optional<SenseSample> delayed;
        double pitch_rad = 0.;
    };
    // reaction_delay_s must be a nonnegative multiple of period_s (the
    // intent.cpp:382-389 rule).
    Perception(double reaction_delay_s, double period_s);
    void reset();
    // Called once per intention period at the interval start. Returns
    // nullopt until the first sample is reaction_delay_s old. A sample
    // outside the accepted domain mutates nothing.
    [[nodiscard]] std::optional<Percepts> update(const SenseSample &sample);
    [[nodiscard]] State state() const;
    // Validates the whole state first — the clock and every queued
    // timestamp on the period grid, the queue exactly the contiguous
    // not-yet-due tail, the delayed slot consistent with the clock, all
    // sample contents inside the readers' domains — then commits
    // atomically (P10).
    void restore(const State &state);

private:
    double delay_s_, period_s_;
    State s_;
};

} // namespace rider
