// rider/front_lift_reaction.hpp — the rider's reaction to a rising front
// wheel (launch plan, part 3a; «Решено — не сделано», item 2). One reaction
// for the whole ride: on the delayed percepts, hands whose press averaged
// over the last half crank revolution (at most a window) is below a share
// of the same average a lag earlier — launch part 4, criterion B —
// together with a fast pitch-up or a held pitch, make the predicate; after
// a decision time the rider eases the effort ceiling and/or shifts the
// lean target forward, each behind its own switch, and recovers once the
// predicate has been absent for clear_s. Times count intention ticks.
#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string_view>

#include "perception.hpp"

namespace rider {

struct FrontLiftReactionConfig {
    // Launch part 4, criterion B: the hands are unloading while their press
    // averaged over the last half crank revolution (at most hand_window_s)
    // is below hand_drop_ratio of the same average hand_lag_s earlier and
    // that earlier average is positive.
    double hand_window_s = std::numeric_limits<double>::quiet_NaN();
    double hand_lag_s = std::numeric_limits<double>::quiet_NaN();
    double hand_drop_ratio = std::numeric_limits<double>::quiet_NaN();
    double pitch_rate_up_rad_s = std::numeric_limits<double>::quiet_NaN();
    double pitch_rad = std::numeric_limits<double>::quiet_NaN();
    double decision_s = std::numeric_limits<double>::quiet_NaN();
    bool ease_enabled = false;
    bool lean_enabled = false;
    bool rear_brake_tap_enabled = false;   // must stay false: not implemented
    double ease_fraction = std::numeric_limits<double>::quiet_NaN();
    double ease_rate_per_s = std::numeric_limits<double>::quiet_NaN();
    double lean_add_rad = std::numeric_limits<double>::quiet_NaN();
    double lean_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
    double clear_s = std::numeric_limits<double>::quiet_NaN();
};

// The table's own domain; throws std::invalid_argument naming the
// front_lift_reaction field.
void validate_reaction_table(const FrontLiftReactionConfig &table);
// The rules that need the seated-climb record: the three times on the
// intention period grid (at most 2^31 ticks) and the lean add inside the
// forward lean bound. Both the profile post_init and RiderIntent call it.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (period_s, bound) is the seated-climb context order
void validate_reaction_context(const FrontLiftReactionConfig &table, double period_s,
                               double max_forward_lean_rad);

enum class ReactionPhase : std::uint8_t { idle, deciding, acting, releasing };
[[nodiscard]] const char *reaction_phase_name(ReactionPhase phase);          // throws on a value outside the enum
[[nodiscard]] ReactionPhase reaction_phase(std::string_view name);           // throws on an unknown name

struct ReactionCommand {
    ReactionPhase phase = ReactionPhase::idle;
    bool predicate = false;
    double effort_factor = 1.;   // multiplies the effort ceiling
    double lean_add_rad = 0.;    // joins the posture program's lean target
};

class FrontLiftReaction {
public:
    struct State {
        ReactionPhase phase = ReactionPhase::idle;
        bool predicate = false;
        std::int64_t decision_ticks = 0;   // < decision ticks while deciding, else 0
        std::int64_t clear_ticks = 0;      // <= clear ticks while acting, else 0
        double effort_factor = 1.;
        double lean_add_rad = 0.;
        // Updates that carried percepts, saturating at percepts_cap(): the
        // earliest tick each phase can be reached is a count of percepts.
        std::int64_t percepts_seen = 0;
        // Criterion B history, one entry per percept: the delayed hand press
        // and the crank angle integrated from the delayed cadence (the last
        // window + 1 of each), the window averages (the last lag + 1) and
        // the integrated angle itself.
        std::deque<double> hand_n;
        std::deque<double> crank_angle_rad;
        std::deque<double> hand_average_n;
        double crank_angle_total_rad = 0.;
        bool operator==(const State &) const = default;
    };
    FrontLiftReaction(const FrontLiftReactionConfig &config, double period_s);
    // lag + 1 + decision + clear + 1 ticks: the percept count past which
    // every phase is reachable (at most about 2^33 with the 2^31 tick bound).
    [[nodiscard]] std::int64_t percepts_cap() const noexcept;
    void reset();
    // Rejects nonfinite percept fields without changing anything.
    static void validate(const std::optional<Percepts> &percepts);
    // One intention tick; percepts are absent until the reaction delay has
    // passed and the reaction then stays idle.
    ReactionCommand update(const std::optional<Percepts> &percepts);
    [[nodiscard]] ReactionCommand command() const;
    [[nodiscard]] State state() const;
    // Rejects every state that breaks a necessary condition of update():
    // phase domain, phase/counter/predicate consistency, the earliest
    // percept count of each phase, switch ranges of the outputs and
    // "acting moves every enabled output". It does not check the outputs
    // against the grid of rate steps. A rejected state leaves the reaction
    // unchanged.
    void restore(const State &state);

private:
    FrontLiftReactionConfig config_;
    double period_s_;
    std::int64_t window_ticks_ = 0, lag_ticks_ = 0, decision_ticks_ = 0, clear_ticks_ = 0;
    bool hand_term(State &n, const Percepts &p) const;
    State s_;
};

} // namespace rider
