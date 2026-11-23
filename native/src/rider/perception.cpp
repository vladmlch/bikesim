// perception.cpp — the delayed, periodic senses adapter.
#include "perception.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rider {
namespace {

    // One acquisition's accepted domain: every channel a finite real,
    // the engaged cog inside the strict reader's floor, the optional
    // load share a share — mirroring what decode_intent_signals and
    // decode_rider_senses accept on the wire (P10: a poisoned sample
    // must move neither the queue nor the clock).
    void check_sample(const SenseSample &sample, std::string_view what) {
        const auto finite = [what](double value, std::string_view field) {
            if (!std::isfinite(value))
                throw std::invalid_argument(std::string(what) + " " +
                                            std::string(field) +
                                            " must be finite");
        };
        finite(sample.signals.pitch_rate_up_rad_s, "pitch_rate_up_rad_s");
        finite(sample.signals.crank_rate_rad_s, "crank_rate_rad_s");
        finite(sample.signals.human_crank_torque_nm,
               "human_crank_torque_nm");
        for (const double component : sample.signals.specific_force_body_mps2)
            finite(component, "specific_force_body_mps2");
        if (sample.signals.front_load_share.has_value()) {
            finite(*sample.signals.front_load_share, "front_load_share");
            if (*sample.signals.front_load_share < 0. ||
                *sample.signals.front_load_share > 1.)
                throw std::invalid_argument(
                    std::string(what) +
                    " front_load_share must lie in [0, 1]");
        }
        finite(sample.senses.hand_press_n, "hand_press_n");
        finite(sample.senses.speed_mps, "speed_mps");
        finite(sample.senses.grade_ahead, "grade_ahead");
        if (sample.senses.rear_teeth < 3)
            throw std::invalid_argument(
                std::string(what) + " rear_teeth below the cog domain");
        if (surface_label_name(sample.senses.surface_rear).empty())
            throw std::invalid_argument(
                std::string(what) +
                " surface_rear outside the label domain");
        if (surface_label_name(sample.senses.surface_front).empty())
            throw std::invalid_argument(
                std::string(what) +
                " surface_front outside the label domain");
        if (surface_label_name(sample.senses.surface_ahead).empty())
            throw std::invalid_argument(
                std::string(what) +
                " surface_ahead outside the label domain");
    }

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed (delay, period) order
Perception::Perception(double reaction_delay_s, double period_s)
    : period_s_(period_s) {
    if (!std::isfinite(period_s_) || period_s_ <= 0.)
        throw std::invalid_argument(
            "perception period must be finite and positive");
    const double ticks = std::rint(reaction_delay_s / period_s_);
    if (!std::isfinite(reaction_delay_s) || reaction_delay_s < 0. ||
        std::abs(ticks * period_s_ - reaction_delay_s) > 1e-9)
        throw std::invalid_argument(
            "perception reaction delay must be a nonnegative multiple of "
            "the period");
    // The double bound rounds up to 2^63, so the comparison must be
    // inclusive: every integral double below it is castable to int64.
    if (ticks >= static_cast<double>(
                     std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument(
            "perception reaction delay exceeds the tick domain");
    delay_ticks_ = static_cast<std::int64_t>(ticks);
    reset();
}

void Perception::reset() { s_ = {}; }

std::optional<Percepts> Perception::update(const SenseSample &sample) {
    if (s_.tick == std::numeric_limits<std::int64_t>::max())
        throw std::invalid_argument("perception tick counter overflowed");
    check_sample(sample, "perception sample");
    // Validate the delivery's outputs before live state moves.
    const SenseSample *delayed = s_.delayed ? &*s_.delayed : nullptr;
    if (!s_.samples.empty() &&
        s_.samples.front().first <= s_.tick - delay_ticks_)
        delayed = &s_.samples.front().second;
    if (delay_ticks_ == 0) delayed = &sample;
    double pitch = s_.pitch_rad;
    std::optional<double> initial_grade = s_.initial_grade_rad;
    std::optional<Percepts> out;
    if (delayed) {
        pitch = s_.pitch_rad +
                delayed->signals.pitch_rate_up_rad_s * period_s_;
        if (!std::isfinite(pitch))
            throw std::invalid_argument("perception pitch overflowed");
        const double trail_rad = std::atan(delayed->senses.grade_ahead);
        const double init_rad = s_.initial_grade_rad.value_or(trail_rad);
        const double pitch_rel = pitch - (trail_rad - init_rad);
        if (!std::isfinite(pitch_rel))
            throw std::invalid_argument("perception pitch overflowed");
        const double crank_rpm =
            std::abs(delayed->signals.crank_rate_rad_s) * 60. /
            (2. * std::numbers::pi);
        if (!std::isfinite(crank_rpm))
            throw std::invalid_argument("perception crank rate overflowed");
        initial_grade = init_rad;
        out = Percepts{.senses = delayed->senses,
                       .crank_rpm = crank_rpm,
                       .human_crank_torque_nm =
                           delayed->signals.human_crank_torque_nm,
                       .pitch_rate_up_rad_s =
                           delayed->signals.pitch_rate_up_rad_s,
                       .pitch_rad = pitch_rel};
    }
    s_.samples.emplace_back(s_.tick, sample);
    // A stamp k is due at the tick n with k <= n - delay_ticks — the
    // integer form of the intent queue's `<= ready + 1e-12` (exact, no
    // accumulated rounding can slip it).
    while (!s_.samples.empty() &&
           s_.samples.front().first <= s_.tick - delay_ticks_) {
        s_.delayed = s_.samples.front().second;
        s_.samples.pop_front();
    }
    ++s_.tick;
    s_.pitch_rad = pitch;
    s_.initial_grade_rad = initial_grade;
    return out;
}

Perception::State Perception::state() const { return s_; }

void Perception::restore(const State &state) {
    if (state.tick < 0)
        throw std::invalid_argument(
            "perception state tick must be nonnegative");
    // The reachable domain (P10) is now closed over the integer tick —
    // O(queue), no replay: update() leaves exactly min(tick,
    // delay_ticks) queued rows stamped with the contiguous not-yet-due
    // tail tick-size..tick-1, and engages the delayed slot iff at least
    // one delivery already happened. A state failing any of these can
    // never come out of update(), so it can never change delay
    // semantics.
    const std::int64_t queued =
        static_cast<std::int64_t>(state.samples.size());
    if (queued != std::min(state.tick, delay_ticks_) ||
        state.delayed.has_value() != (state.tick > delay_ticks_))
        throw std::invalid_argument(
            "perception queue is not the reachable undelivered tail");
    std::int64_t expected = state.tick - queued;
    for (const auto &[t, sample] : state.samples) {
        if (t != expected)
            throw std::invalid_argument(
                "perception queue times are not the contiguous tail");
        check_sample(sample, "perception queued sample");
        ++expected;
    }
    if (state.delayed.has_value())
        check_sample(*state.delayed, "perception delayed sample");
    if (!std::isfinite(state.pitch_rad))
        throw std::invalid_argument("perception pitch_rad must be finite");
    if (state.initial_grade_rad.has_value() != (state.tick > delay_ticks_))
        throw std::invalid_argument(
            "perception initial_grade_rad does not match delivery state");
    if (state.initial_grade_rad.has_value() &&
        !std::isfinite(*state.initial_grade_rad))
        throw std::invalid_argument(
            "perception initial_grade_rad must be finite");
    // The allocating deque copy goes into a candidate first: it can
    // throw without touching the live state, and the move assignment
    // below is the nonthrowing commit.
    State candidate = state;
    s_ = std::move(candidate);
}

} // namespace rider
