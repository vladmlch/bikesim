// perception.cpp — the delayed, periodic senses adapter.
#include "perception.hpp"

#include <algorithm>
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

    // t must sit on this adapter's period grid within the constructor's
    // own 1e-9 tolerance; the returned tick is the grid index.
    long long grid_ticks(double t, double period_s,
                         std::string_view what) {
        const double ratio = t / period_s;
        if (!std::isfinite(ratio) || std::abs(ratio) > 4e15)
            throw std::invalid_argument(std::string(what));
        const long long ticks = std::llround(ratio);
        if (std::abs(static_cast<double>(ticks) * period_s - t) > 1e-9)
            throw std::invalid_argument(std::string(what));
        return ticks;
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
    // The reachable domain (P10): every emitted tick sits on the period
    // grid; the queue is exactly the contiguous tail of samples not yet
    // due — min(clock, delay) rows stamped clock-size..clock-1 — and the
    // delayed slot is engaged iff at least one delivery already
    // happened. A state failing any of these can never come out of
    // update(), so it can never change delay semantics.
    const long long clock_ticks =
        grid_ticks(state.time_s, period_s_,
                   "perception clock off the period grid");
    const long long delay_ticks = std::llround(delay_s_ / period_s_);
    const long long queued = static_cast<long long>(state.samples.size());
    if (queued != std::min(clock_ticks, delay_ticks))
        throw std::invalid_argument(
            "perception queue is not the reachable undelivered tail");
    if (state.delayed.has_value() != (clock_ticks > delay_ticks))
        throw std::invalid_argument(
            "perception delayed slot disagrees with the clock");
    long long expected = clock_ticks - queued;
    for (const auto &[t, sample] : state.samples) {
        if (grid_ticks(t, period_s_,
                       "perception sample time off the period grid") !=
            expected)
            throw std::invalid_argument(
                "perception queue times are not the contiguous tail");
        check_sample(sample, "perception queued sample");
        ++expected;
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
