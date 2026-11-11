// perception.cpp — the delayed, periodic senses adapter.
#include "perception.hpp"

#include <cmath>
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
        if (surface_label_name(sample.senses.surface_front).empty())
            throw std::invalid_argument(
                std::string(what) +
                " surface_front outside the label domain");
        if (surface_label_name(sample.senses.surface_ahead).empty())
            throw std::invalid_argument(
                std::string(what) +
                " surface_ahead outside the label domain");
    }

    // acc(n): the adapter clock after n updates — n repeated
    // `+= period_s` steps from zero. The reachable timestamps are this
    // accumulated sequence, never the closed-form n * period: after a
    // long ride the IEEE drift pushes stamps off the ideal grid by more
    // than any small absolute tolerance (follow-up 1). O(n); restore is
    // a snapshot boundary, not a tick.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed (ticks, period) order
    [[nodiscard]] double accumulated_clock(long long ticks,
                                           double period_s) {
        double acc = 0.;
        for (long long i = 0; i < ticks; ++i) acc += period_s;
        return acc;
    }

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed (delay, period) order
Perception::Perception(double reaction_delay_s, double period_s)
    : delay_s_(reaction_delay_s), period_s_(period_s) {
    if (!std::isfinite(period_s_) || period_s_ <= 0.)
        throw std::invalid_argument(
            "perception period must be finite and positive");
    const double ticks = std::rint(delay_s_ / period_s_);
    if (!std::isfinite(delay_s_) || delay_s_ < 0. ||
        std::abs(ticks * period_s_ - delay_s_) > 1e-9)
        throw std::invalid_argument(
            "perception reaction delay must be a nonnegative multiple of "
            "the period");
    reset();
}

void Perception::reset() { s_ = {}; }

std::optional<Percepts> Perception::update(const SenseSample &sample) {
    check_sample(sample, "perception sample");
    s_.samples.emplace_back(s_.time_s, sample);
    const double ready = s_.time_s - delay_s_;
    while (!s_.samples.empty() &&
           s_.samples.front().first <= ready + 1e-12) {
        s_.delayed = s_.samples.front().second;
        s_.samples.pop_front();
    }
    s_.time_s += period_s_;
    if (!s_.delayed) return std::nullopt;
    const SenseSample &d = *s_.delayed;
    const double pitch =
        s_.pitch_rad + d.signals.pitch_rate_up_rad_s * period_s_;
    if (!std::isfinite(pitch))
        throw std::invalid_argument("perception pitch overflowed");
    s_.pitch_rad = pitch;
    return Percepts{.senses = d.senses,
                    .crank_rpm = std::abs(d.signals.crank_rate_rad_s) *
                                     60. / (2. * std::numbers::pi),
                    .human_crank_torque_nm =
                        d.signals.human_crank_torque_nm,
                    .pitch_rate_up_rad_s = d.signals.pitch_rate_up_rad_s,
                    .pitch_rad = s_.pitch_rad};
}

Perception::State Perception::state() const { return s_; }

void Perception::restore(const State &state) {
    if (!std::isfinite(state.time_s) || state.time_s < 0.)
        throw std::invalid_argument(
            "perception state clock must be finite and nonnegative");
    // The reachable domain (P10), stated over update()'s own arithmetic
    // rather than the ideal period grid (follow-up 1): the recorded
    // clock must be a bitwise value of the accumulated sequence
    // acc(n); the queue must be exactly the samples update() left
    // undelivered — the contiguous tail acc(delivered)..acc(clock-1) —
    // and the delayed slot is engaged iff at least one delivery
    // happened. Replaying the accumulation makes the acceptance set
    // exactly the reachable set, drift and delivery slips included;
    // anything else — reordered, duplicated, off-sequence or fabricated
    // timestamps — can never have come out of update() and rejects.
    const double ratio = state.time_s / period_s_;
    if (!std::isfinite(ratio) || std::abs(ratio) > 4e15)
        throw std::invalid_argument(
            "perception state clock outside the replayable domain");
    const long long estimate = std::llround(ratio);
    // Drift stays far below one period for any realistic ride length,
    // so the tick count is the grid estimate or an immediate neighbour.
    long long clock_ticks = -1;
    double ready_clock = 0.;   // acc(clock_ticks - 1) once found
    {
        double acc = 0., prev = 0.;
        for (long long n = 0; n <= estimate + 1 && clock_ticks < 0; ++n) {
            if (n >= estimate - 1 && acc == state.time_s) {
                clock_ticks = n;
                ready_clock = prev;
            }
            prev = acc;
            acc += period_s_;
        }
    }
    if (clock_ticks < 0)
        throw std::invalid_argument(
            "perception state clock is not an update()-reachable value");
    // Delivered prefix length: update() pops every sample whose stamp
    // is at or under `ready + 1e-12`; the last tick's clock is
    // acc(clock_ticks - 1) and a stamp delivered by then is a prefix.
    long long delivered = 0;
    if (clock_ticks > 0) {
        const double boundary = ready_clock - delay_s_ + 1e-12;
        double acc = 0.;
        for (long long j = 0; j < clock_ticks; ++j) {
            if (acc <= boundary) ++delivered;
            acc += period_s_;
        }
    }
    const long long queued = static_cast<long long>(state.samples.size());
    if (queued != clock_ticks - delivered ||
        state.delayed.has_value() != (delivered > 0))
        throw std::invalid_argument(
            "perception queue is not the reachable undelivered tail");
    {
        double stamp = accumulated_clock(delivered, period_s_);
        for (const auto &[t, sample] : state.samples) {
            if (t != stamp)
                throw std::invalid_argument(
                    "perception queue times are not the contiguous "
                    "tail");
            check_sample(sample, "perception queued sample");
            stamp += period_s_;
        }
    }
    if (state.delayed.has_value())
        check_sample(*state.delayed, "perception delayed sample");
    if (!std::isfinite(state.pitch_rad))
        throw std::invalid_argument("perception pitch_rad must be finite");
    // The allocating deque copy goes into a candidate first: it can
    // throw without touching the live state, and the move assignment
    // below is the nonthrowing commit.
    State candidate = state;
    s_ = std::move(candidate);
}

} // namespace rider
