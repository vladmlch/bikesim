#include "front_lift_reaction.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rider {
namespace {

[[noreturn]] void reject(std::string_view field, std::string_view why) {
    throw std::invalid_argument("front_lift_reaction." + std::string(field) + ": " + std::string(why));
}

void finite_at_least(double value, double minimum, std::string_view field) {
    if (!std::isfinite(value) || value < minimum) reject(field, "must be finite and not below its minimum");
}

void finite_positive(double value, std::string_view field) {
    if (!std::isfinite(value) || value <= 0.) reject(field, "must be finite and positive");
}

// A time on the intention grid, at most 2^31 ticks (exactly representable,
// far beyond any ride, and safe to count in int64).
std::int64_t ticks(double seconds, double period_s, std::string_view field) {
    const double n = std::rint(seconds / period_s);
    if (!(n >= 0. && n <= 0x1p31) || std::abs(n * period_s - seconds) > 1e-9)
        reject(field, "must be a whole multiple of the intention period, at most 2^31 ticks");
    return static_cast<std::int64_t>(n);
}

double step_toward(double value, double target, double step) {
    if (std::abs(target - value) <= step) return target;
    return value + (target > value ? step : -step);
}

} // namespace

void validate_reaction_table(const FrontLiftReactionConfig &t) {
    finite_positive(t.hand_window_s, "hand_window_s");
    finite_positive(t.hand_lag_s, "hand_lag_s");
    if (!std::isfinite(t.hand_drop_ratio) || t.hand_drop_ratio <= 0. || t.hand_drop_ratio > 1.)
        reject("hand_drop_ratio", "must lie in (0, 1]");
    finite_positive(t.pitch_rate_up_rad_s, "pitch_rate_up_rad_s");
    finite_positive(t.pitch_rad, "pitch_rad");
    finite_at_least(t.decision_s, 0., "decision_s");
    if (t.rear_brake_tap_enabled) reject("rear_brake_tap_enabled", "the rear brake tap is not implemented");
    if (!std::isfinite(t.ease_fraction) || t.ease_fraction < 0. || t.ease_fraction > 1.)
        reject("ease_fraction", "must lie in [0, 1]");
    finite_positive(t.ease_rate_per_s, "ease_rate_per_s");
    finite_at_least(t.lean_add_rad, 0., "lean_add_rad");
    finite_positive(t.lean_rate_rad_s, "lean_rate_rad_s");
    finite_at_least(t.clear_s, 0., "clear_s");
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (period_s, bound) is the seated-climb context order
void validate_reaction_context(const FrontLiftReactionConfig &t, double period_s, double max_forward_lean_rad) {
    validate_reaction_table(t);
    if (!std::isfinite(period_s) || period_s <= 0.)
        throw std::invalid_argument("front-lift reaction needs a positive intention period");
    (void)ticks(t.hand_window_s, period_s, "hand_window_s");
    (void)ticks(t.hand_lag_s, period_s, "hand_lag_s");
    (void)ticks(t.decision_s, period_s, "decision_s");
    (void)ticks(t.clear_s, period_s, "clear_s");
    if (t.lean_add_rad > max_forward_lean_rad) reject("lean_add_rad", "beyond the seated forward lean bound");
}

const char *reaction_phase_name(ReactionPhase phase) {
    switch (phase) {
    case ReactionPhase::idle: return "idle";
    case ReactionPhase::deciding: return "deciding";
    case ReactionPhase::acting: return "acting";
    case ReactionPhase::releasing: return "releasing";
    }
    throw std::invalid_argument("front-lift reaction phase outside the enum");
}

ReactionPhase reaction_phase(std::string_view name) {
    if (name == "idle") return ReactionPhase::idle;
    if (name == "deciding") return ReactionPhase::deciding;
    if (name == "acting") return ReactionPhase::acting;
    if (name == "releasing") return ReactionPhase::releasing;
    throw std::invalid_argument("front-lift reaction phase: unknown name");
}

FrontLiftReaction::FrontLiftReaction(const FrontLiftReactionConfig &config, double period_s)
    : config_(config), period_s_(period_s) {
    validate_reaction_table(config_);
    if (!std::isfinite(period_s_) || period_s_ <= 0.)
        throw std::invalid_argument("front-lift reaction needs a positive intention period");
    window_ticks_ = ticks(config_.hand_window_s, period_s_, "hand_window_s");
    lag_ticks_ = ticks(config_.hand_lag_s, period_s_, "hand_lag_s");
    decision_ticks_ = ticks(config_.decision_s, period_s_, "decision_s");
    clear_ticks_ = ticks(config_.clear_s, period_s_, "clear_s");
}

void FrontLiftReaction::reset() { s_ = {}; }

void FrontLiftReaction::validate(const std::optional<Percepts> &p) {
    if (p && (!std::isfinite(p->senses.hand_press_n) || !std::isfinite(p->crank_rpm) ||
              !std::isfinite(p->pitch_rate_up_rad_s) || !std::isfinite(p->pitch_rad)))
        throw std::invalid_argument("front-lift reaction percepts must be finite");
}

// Launch part 4, decision 4 — the calibration probe's criterion B on a
// bounded history (gradecap/_diff_reaction.txt; same arithmetic and
// summation order). The newest sample at least half a revolution back is
// inside the last window + 1 samples whenever the probe's span fits the
// window, so the bounded search finds the same span or falls back the same way.
bool FrontLiftReaction::hand_term(State &n, const Percepts &p) const {
    n.hand_n.push_back(p.senses.hand_press_n);
    n.crank_angle_total_rad += p.crank_rpm * (2. * std::numbers::pi / 60.) * period_s_;
    n.crank_angle_rad.push_back(n.crank_angle_total_rad);
    if (std::ssize(n.hand_n) > window_ticks_ + 1) {
        n.hand_n.pop_front();
        n.crank_angle_rad.pop_front();
    }
    std::int64_t span = -1;
    for (std::size_t i = n.crank_angle_rad.size(); i-- > 0;)
        if (n.crank_angle_total_rad - n.crank_angle_rad[i] >= std::numbers::pi) {
            span = static_cast<std::int64_t>(n.crank_angle_rad.size() - 1 - i);
            break;
        }
    if (span < 0 || span > window_ticks_) span = std::min<std::int64_t>(window_ticks_, std::ssize(n.hand_n));
    double sum = 0.;
    const std::int64_t count = std::ssize(n.hand_n);
    for (std::int64_t i = count - span; i < count; ++i) sum += n.hand_n[static_cast<std::size_t>(i)];
    const double average = sum / static_cast<double>(span);
    n.hand_average_n.push_back(average);
    if (std::ssize(n.hand_average_n) > lag_ticks_ + 1) n.hand_average_n.pop_front();
    if (std::ssize(n.hand_average_n) < lag_ticks_ + 1) return false;
    const double reference = n.hand_average_n.front();
    return reference > 0. && average < config_.hand_drop_ratio * reference;
}

ReactionCommand FrontLiftReaction::update(const std::optional<Percepts> &percepts) {
    validate(percepts);
    State n = s_;
    if (!percepts) {
        n.predicate = false;
    } else {
        n.percepts_seen = std::min(n.percepts_seen + 1, percepts_cap());
        // The hand term runs first on every percept so the history always advances.
        const bool hands = hand_term(n, *percepts);
        n.predicate = hands && (percepts->pitch_rate_up_rad_s > config_.pitch_rate_up_rad_s ||
                                percepts->pitch_rad > config_.pitch_rad);
    }
    switch (n.phase) {
    case ReactionPhase::idle:
        if (n.predicate) { n.phase = ReactionPhase::deciding; n.decision_ticks = 0; }
        break;
    case ReactionPhase::deciding:
        if (!n.predicate) { n.phase = ReactionPhase::idle; n.decision_ticks = 0; }
        else ++n.decision_ticks;
        break;
    case ReactionPhase::acting:
        n.clear_ticks = n.predicate ? 0 : n.clear_ticks + 1;
        if (n.clear_ticks > clear_ticks_) { n.phase = ReactionPhase::releasing; n.clear_ticks = 0; }
        break;
    case ReactionPhase::releasing:
        if (n.predicate) n.phase = ReactionPhase::acting;
        break;
    }
    if (n.phase == ReactionPhase::deciding && n.decision_ticks >= decision_ticks_) {
        n.phase = ReactionPhase::acting;
        n.decision_ticks = 0;
        n.clear_ticks = 0;
    }
    const bool acting = n.phase == ReactionPhase::acting;
    n.effort_factor = step_toward(n.effort_factor,
                                  acting && config_.ease_enabled ? 1. - config_.ease_fraction : 1.,
                                  config_.ease_rate_per_s * period_s_);
    n.lean_add_rad = step_toward(n.lean_add_rad, acting && config_.lean_enabled ? config_.lean_add_rad : 0.,
                                 config_.lean_rate_rad_s * period_s_);
    if (n.phase == ReactionPhase::releasing && n.effort_factor == 1. && n.lean_add_rad == 0.)
        n.phase = ReactionPhase::idle;
    s_ = std::move(n);
    return command();
}

std::int64_t FrontLiftReaction::percepts_cap() const noexcept {
    return lag_ticks_ + 1 + decision_ticks_ + clear_ticks_ + 1;
}

ReactionCommand FrontLiftReaction::command() const {
    return {.phase = s_.phase, .predicate = s_.predicate, .effort_factor = s_.effort_factor,
            .lean_add_rad = s_.lean_add_rad};
}

FrontLiftReaction::State FrontLiftReaction::state() const { return s_; }

// Rejects states update() cannot leave behind (review-2 I3, review-3 I5):
// a state outside that set could report events that never happened or skip
// the reference lag and the decision time. These are necessary conditions,
// not exact reachability: the output values between their bounds are not
// checked against the grid of rate steps.
void FrontLiftReaction::restore(const State &s) {
    const auto bad = [] [[noreturn]] (std::string_view field, std::string_view why) {
        throw std::invalid_argument("front-lift reaction " + std::string(field) + ": " + std::string(why));
    };
    (void)reaction_phase_name(s.phase);   // throws on a phase outside the enum
    if (s.percepts_seen < 0 || s.percepts_seen > percepts_cap()) bad("percepts_seen", "outside [0, cap]");
    if (s.hand_n.size() != s.crank_angle_rad.size()) bad("hand_n", "hand and crank histories differ in length");
    const std::int64_t hands = std::ssize(s.hand_n), averages = std::ssize(s.hand_average_n);
    const bool saturated = s.percepts_seen == percepts_cap();
    if (saturated ? hands < std::min(percepts_cap(), window_ticks_ + 1) || hands > window_ticks_ + 1
                  : hands != std::min(s.percepts_seen, window_ticks_ + 1))
        bad("hand_n", "history length off its percept count");
    if (averages != std::min(s.percepts_seen, lag_ticks_ + 1)) bad("hand_average_n", "history length off its percept count");
    for (const double v : s.hand_n) if (!std::isfinite(v)) bad("hand_n", "must be finite");
    for (const double v : s.hand_average_n) if (!std::isfinite(v)) bad("hand_average_n", "must be finite");
    for (const double v : s.crank_angle_rad) if (!std::isfinite(v)) bad("crank_angle_rad", "must be finite");
    if (!std::isfinite(s.crank_angle_total_rad) ||
        (s.crank_angle_rad.empty() ? s.crank_angle_total_rad != 0. : s.crank_angle_rad.back() != s.crank_angle_total_rad))
        bad("crank_angle_total_rad", "must equal the newest crank angle");
    // update() sets the predicate only on a full reference and a hand drop.
    if (s.predicate && (averages != lag_ticks_ + 1 || !(s.hand_average_n.front() > 0.) ||
                        !(s.hand_average_n.back() < config_.hand_drop_ratio * s.hand_average_n.front())))
        bad("predicate", "set without a hand drop against a full reference");
    if (!std::isfinite(s.effort_factor) || s.effort_factor > 1. ||
        s.effort_factor < (config_.ease_enabled ? 1. - config_.ease_fraction : 1.))
        bad("effort_factor", "outside the range its switch allows");
    if (!std::isfinite(s.lean_add_rad) || s.lean_add_rad < 0. ||
        s.lean_add_rad > (config_.lean_enabled ? config_.lean_add_rad : 0.))
        bad("lean_add_rad", "outside the range its switch allows");
    const bool neutral = s.effort_factor == 1. && s.lean_add_rad == 0.;
    // The earliest percept count of each phase: the predicate first holds
    // at lag + 1 percepts, acting starts decision ticks later, and a
    // release needs clear + 1 more percepts without the predicate.
    const std::int64_t acting_from = lag_ticks_ + 1 + decision_ticks_;
    switch (s.phase) {
    case ReactionPhase::idle:
        if (s.predicate) bad("predicate", "idle with the predicate");
        if (s.decision_ticks != 0 || s.clear_ticks != 0) bad("phase", "idle with a running counter");
        if (!neutral) bad("phase", "no action may be in force while idle");
        break;
    case ReactionPhase::deciding:
        if (!s.predicate) bad("predicate", "deciding without the predicate");
        // A zero decision time makes deciding unreachable: the bound rejects every count.
        if (s.decision_ticks < 0 || s.decision_ticks >= decision_ticks_)
            bad("decision_ticks", "outside the decision window");
        if (s.clear_ticks != 0) bad("clear_ticks", "running while deciding");
        if (!neutral) bad("phase", "no action may be in force while deciding");
        if (s.percepts_seen < lag_ticks_ + 1 + s.decision_ticks)
            bad("percepts_seen", "deciding longer than the percepts allow");
        break;
    case ReactionPhase::acting:
        if (s.decision_ticks != 0) bad("decision_ticks", "running while acting");
        if (s.clear_ticks < 0 || s.clear_ticks > clear_ticks_) bad("clear_ticks", "outside the clear window");
        // Acting counts clear ticks exactly while the predicate is absent.
        if (s.predicate != (s.clear_ticks == 0)) bad("clear_ticks", "inconsistent with the predicate");
        if (s.percepts_seen < std::min(acting_from + s.clear_ticks, percepts_cap()))
            bad("percepts_seen", "acting before the lag and decision could pass");
        // The first acting update already moves every enabled output whose
        // first step is nonzero — the same step_toward call update() makes, so
        // a subnormal rate or a fraction lost to rounding (review 4, M3) is
        // not rejected.
        if ((config_.ease_enabled && s.effort_factor == 1. &&
             step_toward(1., 1. - config_.ease_fraction, config_.ease_rate_per_s * period_s_) != 1.) ||
            (config_.lean_enabled && s.lean_add_rad == 0. &&
             step_toward(0., config_.lean_add_rad, config_.lean_rate_rad_s * period_s_) != 0.))
            bad("phase", "acting without moving an enabled output");
        break;
    case ReactionPhase::releasing:
        if (s.predicate) bad("predicate", "releasing with the predicate");
        if (s.decision_ticks != 0 || s.clear_ticks != 0) bad("phase", "releasing with a running counter");
        // A release with nothing left turns idle on the same tick.
        if (neutral) bad("phase", "releasing with nothing left to release");
        if (s.percepts_seen != percepts_cap()) bad("percepts_seen", "releasing before clear_s could pass");
        break;
    }
    s_ = s;
}

} // namespace rider
