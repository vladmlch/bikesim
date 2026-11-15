// Launch part 3a — the front-lift reaction node: its timing on the delayed
// percepts, the two independent actions, recovery, the codec round trip
// with bitwise continuation, atomic restore and the table rules. Launch
// part 4 replaces the dwell hand term with criterion B (launch-4, decision
// 4): a bounded serialized hand/crank history and the lagged window
// average as the reference.
#include "support/fixture_test.hpp"
#include "allocation_faults.hpp"
#include "configuration/model_config.hpp"
#include "io/json.hpp"
#include "rider/front_lift_reaction.hpp"
#include "rider/intent.hpp"
#include "rider/senses_codec.hpp"
#include "rider/state_codec.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <new>
#include <numbers>
#include <optional>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr double kPeriod = .00125;

template <typename Action>
void rejects(Action &&action, std::string_view needle, std::string_view what) {
    bool threw = false;
    try {
        std::forward<Action>(action)();
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(std::string_view(failure.what()).find(needle) != std::string_view::npos,
                    std::string(what) + " names " + std::string(needle) + " (" + failure.what() + ")");
    }
    bt::require(threw, std::string(what) + " is rejected");
}

// Window and lag 40 ticks, drop ratio .7, decision 8 ticks, clear 16
// ticks; both actions on.
rider::FrontLiftReactionConfig table() {
    return {.hand_window_s = 40 * kPeriod, .hand_lag_s = 40 * kPeriod, .hand_drop_ratio = .7,
            .pitch_rate_up_rad_s = .5, .pitch_rad = .17, .decision_s = 8 * kPeriod, .ease_enabled = true,
            .lean_enabled = true, .rear_brake_tap_enabled = false, .ease_fraction = .7,
            .ease_rate_per_s = 5., .lean_add_rad = .1, .lean_rate_rad_s = .5, .clear_s = 16 * kPeriod};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) mirrors the Percepts field order
rider::Percepts percept(double hand_press_n, double pitch_rate, double pitch) {
    rider::Percepts p;
    p.senses.hand_press_n = hand_press_n;
    p.pitch_rate_up_rad_s = pitch_rate;
    p.pitch_rad = pitch;
    return p;
}

// 40 loaded samples fill the window and the 41st completes the reference:
// the window average and the reference are both 80 N.
void loaded(rider::FrontLiftReaction &reaction, int ticks) {
    for (int tick = 0; tick < ticks; ++tick) (void)reaction.update(percept(80., 0., 0.));
}

// A steadily unloading grip with the pitch term on: the press falls by 1 %
// per tick. After 41 loaded samples criterion B first holds at k = 62 and
// then without a gap (the window/reference ratio settles at 0.99^40 = 0.669);
// a step to light hands would extinguish itself once the reference is light too.
rider::Percepts pulling(int k) { return percept(80. * std::pow(.99, k), 1., 0.); }

std::string text(const rider::FrontLiftReaction &r) {
    return bike::io::canonical_json(rider::encode_reaction_state(r.state()));
}

// The calibration probe's criterion B (gradecap/_diff_reaction.txt) with
// unbounded histories, kept verbatim as the oracle for the bounded state.
struct ProbeHandB {
    std::deque<double> hand{}, phase_at{}, avg{};
    double phase = 0.;
    std::int64_t fcap = 0, lag = 0;
    double thr = 0.;
    bool term(const rider::Percepts &p, double period_s) {
        hand.push_back(p.senses.hand_press_n);
        phase += p.crank_rpm * (2. * std::numbers::pi / 60.) * period_s;
        phase_at.push_back(phase);
        std::int64_t span = -1;
        for (std::size_t i = phase_at.size(); i-- > 0;)
            if (phase - phase_at[i] >= std::numbers::pi) {
                span = static_cast<std::int64_t>(phase_at.size() - 1 - i);
                break;
            }
        if (span < 0 || span > fcap) span = std::min<std::int64_t>(fcap, static_cast<std::int64_t>(hand.size()));
        double sum = 0.;
        const auto n = static_cast<std::int64_t>(hand.size());
        for (std::int64_t i = n - span; i < n; ++i) sum += hand[static_cast<std::size_t>(i)];
        const double a = sum / static_cast<double>(span);
        avg.push_back(a);
        const auto j = static_cast<std::int64_t>(avg.size()) - 1;
        if (j < lag) return false;
        const double ref = avg[static_cast<std::size_t>(j - lag)];
        return ref > 0. && a < thr * ref;
    }
};

// Criterion B on the bounded state equals the probe's unbounded replay on
// every tick: steady cadence, a stall (the fallback window), a cadence
// that leaves no sample half a revolution back inside the window, hands
// that rise out of a pull (negative and zero references) and absent
// percepts; the pitch term is always on so the predicate is the hand term.
void test_criterion_b_equals_the_probe_replay() {
    auto t = table();
    t.decision_s = 0.;
    // The profile's 0.5 s window and lag (400 ticks): at 90 rpm half a
    // revolution is about 267 ticks, inside the window; 40 rpm and a stall
    // take the fallback.
    t.hand_window_s = 400 * kPeriod;
    t.hand_lag_s = 400 * kPeriod;
    rider::FrontLiftReaction reaction(t, kPeriod);
    ProbeHandB probe{.fcap = 400, .lag = 400, .thr = .7};
    for (int tick = 0; tick < 3000; ++tick) {
        if (tick % 997 == 500) {
            (void)reaction.update(std::nullopt);   // absent percepts leave the history alone
            continue;
        }
        rider::Percepts p = percept(0., 1., 0.);
        p.crank_rpm = tick < 800 ? 90. : tick < 1200 ? 0. : tick < 1600 ? 3. : 70. + 30. * std::sin(tick * .01);
        p.senses.hand_press_n = 60. + 50. * std::sin(tick * .037) - (tick > 2200 && tick < 2400 ? 120. : 0.);
        const bool expected = probe.term(p, kPeriod);
        const auto cmd = reaction.update(p);
        bt::require(cmd.predicate == expected, "tick " + std::to_string(tick) + ": the bounded criterion B equals the probe");
    }
}

// Criterion B: after 41 loaded samples (80 N, the 40-tick reference full)
// the light-hand average falls below 0.7 of the reference on the 13th light
// sample; the action starts after the decision time (8 ticks); the first
// acting tick already moves both outputs one rate step.
void test_predicate_window_and_decision_timing() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    loaded(reaction, 41);
    const auto lifting = percept(5., 1., 0.);
    std::vector<rider::ReactionCommand> out;
    out.reserve(30);
    for (int tick = 0; tick < 30; ++tick) out.push_back(reaction.update(lifting));
    for (std::size_t tick = 0; tick < 12; ++tick)
        bt::require(!out[tick].predicate, "no predicate before the window average drops below 0.7");
    bt::require(out[12].predicate && out[12].phase == rider::ReactionPhase::deciding,
                "the predicate rises on the 13th light sample");
    for (std::size_t tick = 12; tick < 20; ++tick)
        bt::require(out[tick].phase == rider::ReactionPhase::deciding && out[tick].effort_factor == 1.,
                    "the rider is still deciding");
    bt::require(out[20].phase == rider::ReactionPhase::acting, "the action starts after the decision time");
    bt::require(std::abs(out[20].effort_factor - (1. - 5. * kPeriod)) < 1e-15 &&
                    std::abs(out[20].lean_add_rad - .5 * kPeriod) < 1e-15,
                "acting moves both outputs one rate step");
}

// A step to light hands extinguishes itself: once the reference 40 ticks
// back is light too, the ratio returns to 1 and the predicate ends (k = 79).
void test_a_hand_step_extinguishes_itself() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    loaded(reaction, 41);
    std::vector<bool> predicate;
    for (int k = 1; k <= 90; ++k) predicate.push_back(reaction.update(percept(5., 1., 0.)).predicate);
    for (int k = 1; k <= 90; ++k)
        bt::require(predicate[static_cast<std::size_t>(k - 1)] == (k >= 13 && k <= 78),
                    "light sample " + std::to_string(k) + ": the predicate holds exactly for k = 13..78");
}

// A held-high wheel triggers through the accumulated pitch with a small
// rate; light hands without pitch never trigger; unloaded hands without
// pitch never trigger either; a reference that is not positive never
// triggers; no percepts keep idle.
void test_held_pitch_triggers_and_light_hands_alone_do_not() {
    rider::FrontLiftReaction held(table(), kPeriod);
    loaded(held, 41);
    rider::ReactionCommand last;
    for (int tick = 0; tick < 13; ++tick) last = held.update(percept(5., .01, .3));
    bt::require(last.predicate, "a held pitch above the threshold makes the predicate");
    rider::FrontLiftReaction light(table(), kPeriod);
    for (int tick = 0; tick < 400; ++tick) last = light.update(percept(5., .01, .01));
    bt::require(!last.predicate && last.phase == rider::ReactionPhase::idle, "light hands alone never trigger");
    rider::FrontLiftReaction blind(table(), kPeriod);
    for (int tick = 0; tick < 40; ++tick) last = blind.update(std::nullopt);
    bt::require(last.phase == rider::ReactionPhase::idle && !last.predicate, "no percepts, no reaction");
    rider::FrontLiftReaction flat(table(), kPeriod);
    loaded(flat, 41);
    for (int tick = 0; tick < 40; ++tick) last = flat.update(percept(5., 0., 0.));
    bt::require(!last.predicate && last.phase == rider::ReactionPhase::idle, "unloaded hands without pitch never trigger");
    rider::FrontLiftReaction pull(table(), kPeriod);
    for (int tick = 0; tick < 41; ++tick) last = pull.update(percept(-10., 1., 0.));
    for (int tick = 0; tick < 40; ++tick) last = pull.update(percept(-30., 1., 0.));
    bt::require(!last.predicate && last.phase == rider::ReactionPhase::idle,
                "a non-positive reference never triggers");
}

// A predicate that vanishes during the decision returns to idle without
// any action.
void test_short_predicate_is_dropped_while_deciding() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    loaded(reaction, 41);
    rider::ReactionCommand last;
    for (int tick = 0; tick < 14; ++tick) last = reaction.update(percept(5., 1., 0.));
    bt::require(last.phase == rider::ReactionPhase::deciding, "deciding");
    last = reaction.update(percept(80., 0., 0.));
    bt::require(last.phase == rider::ReactionPhase::idle && last.effort_factor == 1. && last.lean_add_rad == 0.,
                "a short lift is dropped before any action");
}

// Each action has its own switch; with both off the phases still run and
// both outputs stay exactly neutral.
void test_each_action_has_its_own_switch() {
    for (const auto &[ease, lean] : {std::pair{true, false}, std::pair{false, true}, std::pair{false, false}}) {
        auto t = table();
        t.ease_enabled = ease;
        t.lean_enabled = lean;
        rider::FrontLiftReaction reaction(t, kPeriod);
        loaded(reaction, 41);
        rider::ReactionCommand last;
        for (int k = 0; k < 300; ++k) last = reaction.update(pulling(k));
        bt::require(last.phase == rider::ReactionPhase::acting, "acting with any switch setting");
        bt::require((last.effort_factor < 1.) == ease && (ease || last.effort_factor == 1.), "ease follows its switch");
        bt::require((last.lean_add_rad > 0.) == lean && (lean || last.lean_add_rad == 0.), "lean follows its switch");
        if (ease) bt::require(std::abs(last.effort_factor - .3) < 1e-12, "the ease settles at 1 - ease_fraction");
        if (lean) bt::require(last.lean_add_rad == .1, "the lean add settles at lean_add_rad");
    }
}

// The reaction ends when the predicate has been absent longer than
// clear_s; both outputs return at their rates and the phase becomes idle.
void test_recovery_after_clear() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    loaded(reaction, 41);
    rider::ReactionCommand last;
    for (int k = 0; k < 300; ++k) last = reaction.update(pulling(k));
    for (int tick = 0; tick < 16; ++tick) {
        last = reaction.update(percept(80., 0., 0.));
        bt::require(last.phase == rider::ReactionPhase::acting, "still acting within clear_s");
    }
    last = reaction.update(percept(80., 0., 0.));
    bt::require(last.phase == rider::ReactionPhase::releasing, "releasing after clear_s");
    for (int tick = 0; tick < 400 && last.phase != rider::ReactionPhase::idle; ++tick)
        last = reaction.update(percept(80., 0., 0.));
    bt::require(last.phase == rider::ReactionPhase::idle && last.effort_factor == 1. && last.lean_add_rad == 0.,
                "idle again with both outputs back exactly");
}

// Serialize → read → continue: restored mid-decision and mid-release, the
// reaction continues bitwise (whole state compared as canonical JSON). The
// template run of the same sequence locates the split points itself: the
// first deciding tick plus 2 and the first releasing tick plus 20.
void test_codec_round_trip_and_continuation() {
    const auto input = [](int tick) {
        return tick < 41 ? percept(80., 0., 0.) : tick < 341 ? pulling(tick - 41) : percept(80., 0., 0.);
    };
    rider::FrontLiftReaction reference(table(), kPeriod);
    std::vector<rider::ReactionPhase> phases;
    phases.reserve(700);
    int first_deciding = -1, first_releasing = -1;
    for (int tick = 0; tick < 700; ++tick) {
        const auto cmd = reference.update(input(tick));
        phases.push_back(cmd.phase);
        if (first_deciding < 0 && cmd.phase == rider::ReactionPhase::deciding) first_deciding = tick;
        if (first_releasing < 0 && cmd.phase == rider::ReactionPhase::releasing) first_releasing = tick;
    }
    bt::require(first_deciding >= 0 && first_releasing >= 0, "the template run reaches deciding and releasing");
    const int deciding_split = first_deciding + 2, releasing_split = first_releasing + 20;
    std::println("codec splits: deciding {} releasing {}", deciding_split, releasing_split);
    bt::require(phases[static_cast<std::size_t>(deciding_split)] == rider::ReactionPhase::deciding,
                "the deciding split is deciding");
    bt::require(phases[static_cast<std::size_t>(releasing_split)] == rider::ReactionPhase::releasing,
                "the releasing split is releasing");
    for (const int split : {deciding_split, releasing_split}) {
        rider::FrontLiftReaction original(table(), kPeriod);
        for (int tick = 0; tick < split; ++tick) (void)original.update(input(tick));
        rider::FrontLiftReaction copy(table(), kPeriod);
        copy.restore(rider::decode_reaction_state(rider::encode_reaction_state(original.state()), "reaction"));
        for (int tick = split; tick < split + 300; ++tick) {
            (void)original.update(input(tick));
            (void)copy.update(input(tick));
            bt::require(text(original) == text(copy), "the restored reaction continues bitwise");
        }
    }
}

// Restore rejects unreachable states atomically, including actions a
// switch forbids, a phase outside the enum, inconsistent counters and
// histories that cannot come out of update(); a reachable state from the
// same configuration is accepted.
void test_restore_rejects_unreachable_states() {
    auto off = table();
    off.ease_enabled = false;
    off.lean_enabled = false;
    rider::FrontLiftReaction target(off, kPeriod);
    loaded(target, 41);
    rider::ReactionCommand last;
    for (int tick = 0; tick < 40 && last.phase != rider::ReactionPhase::deciding; ++tick)
        last = target.update(percept(5., 1., 0.));
    bt::require(last.phase == rider::ReactionPhase::deciding, "the light-hand drop reaches deciding");
    (void)target.update(percept(5., 1., 0.));   // one more tick deciding
    const std::string before = text(target);
    auto bad = target.state();
    bad.phase = rider::ReactionPhase::acting;
    bad.decision_ticks = 0;
    bad.effort_factor = .5;
    rejects([&] { target.restore(bad); }, "effort_factor", "an ease the switch forbids");
    bad.effort_factor = 1.;
    bad.lean_add_rad = .05;
    rejects([&] { target.restore(bad); }, "lean_add_rad", "a lean the switch forbids");
    bad = target.state();
    bad.phase = static_cast<rider::ReactionPhase>(9);   // NOLINT(clang-analyzer-optin.core.EnumCastOutOfRange) the out-of-range phase is the rejection under test
    rejects([&] { target.restore(bad); }, "phase", "a phase outside the enum");
    bad = target.state();
    bad.predicate = false;
    rejects([&] { target.restore(bad); }, "predicate", "deciding without the predicate");
    bad = target.state();
    bad.hand_n.push_back(1.);
    rejects([&] { target.restore(bad); }, "hand_n", "hand and crank histories differ in length");
    // The predicate needs a full positive reference: a state from 30 loaded
    // percepts cannot carry one.
    rider::FrontLiftReaction short_run(off, kPeriod);
    for (int tick = 0; tick < 30; ++tick) (void)short_run.update(percept(80., 0., 0.));
    bad = short_run.state();
    bad.predicate = true;
    rejects([&] { short_run.restore(bad); }, "predicate", "a predicate before the reference is full");
    rider::FrontLiftReaction fresh(off, kPeriod);
    bad = fresh.state();
    bad.crank_angle_total_rad = 1.;
    rejects([&] { fresh.restore(bad); }, "crank_angle", "a crank total on an empty history");
    bt::require(text(target) == before, "rejected restores change nothing");
    target.restore(target.state());   // control: its own reachable state

    // Phase/predicate/counter combinations update() never leaves behind
    // (review-2 I3), on a reaction with both actions on.
    rider::FrontLiftReaction on(table(), kPeriod);
    loaded(on, 41);
    for (int k = 0; k < 300; ++k) (void)on.update(pulling(k));
    const auto acting = on.state();
    bt::require(acting.phase == rider::ReactionPhase::acting && acting.predicate, "acting under the predicate");
    for (int tick = 0; tick < 20; ++tick) (void)on.update(percept(80., 0., 0.));
    const auto releasing = on.state();
    bt::require(releasing.phase == rider::ReactionPhase::releasing, "releasing after clear_s");
    const std::string on_before = text(on);
    bad = acting;
    bad.clear_ticks = 3;
    rejects([&] { on.restore(bad); }, "clear_ticks", "acting that counts clear ticks under the predicate");
    bad = releasing;
    bad.predicate = true;
    rejects([&] { on.restore(bad); }, "predicate", "releasing with the predicate");
    bad = releasing;
    bad.effort_factor = 1.;
    bad.lean_add_rad = 0.;
    rejects([&] { on.restore(bad); }, "phase", "releasing with nothing left");
    bad = acting;
    bad.effort_factor = 1.;
    bad.lean_add_rad = 0.;
    rejects([&] { on.restore(bad); }, "phase", "acting that moved no enabled output");
    bad = acting;
    bad.percepts_seen = 48;   // lag + 1 + decision = 49 percepts are needed
    rejects([&] { on.restore(bad); }, "percepts_seen", "acting before the lag and decision could pass");
    bt::require(text(on) == on_before, "rejected restores change nothing");
    on.restore(acting);   // control: a reachable state of this configuration
    bt::require(on.state().phase == rider::ReactionPhase::acting, "the reachable state restores");
    rejects([&] { (void)rider::decode_reaction_state(bike::io::Value{bike::io::Value::Object{}}, "reaction"); },
            "reaction", "an empty document");
}

// The table's own rules and the period/posture context, each with the
// valid control passing.
void test_table_rules() {
    rider::validate_reaction_table(table());
    rider::validate_reaction_context(table(), kPeriod, .35);
    auto with = [](auto change) { auto t = table(); change(t); return t; };
    rider::validate_reaction_table(with([](auto &t) { t.hand_drop_ratio = 1.; }));   // a unit ratio is allowed
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.hand_window_s = 0.; })); },
            "hand_window_s", "a zero hand window");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.hand_lag_s = 0.; })); },
            "hand_lag_s", "a zero hand lag");
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.hand_window_s = .5 * kPeriod; }), kPeriod, .35); },
            "whole multiple", "a hand window below one tick");
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.hand_lag_s = .5 * kPeriod; }), kPeriod, .35); },
            "whole multiple", "a hand lag below one tick");
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.hand_window_s = 1.5 * kPeriod; }), kPeriod, .35); },
            "whole multiple", "a hand window off the tick grid");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.hand_drop_ratio = 0.; })); },
            "hand_drop_ratio", "a zero drop ratio");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.hand_drop_ratio = 1.5; })); },
            "hand_drop_ratio", "a drop ratio above one");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.rear_brake_tap_enabled = true; })); },
            "not implemented", "the rear brake tap");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.ease_fraction = 1.2; })); },
            "ease_fraction", "an ease beyond one");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.ease_rate_per_s = 0.; })); },
            "ease_rate_per_s", "a zero ease rate");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.lean_add_rad = -.1; })); },
            "lean_add_rad", "a backward lean add");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.lean_rate_rad_s = 0.; })); },
            "lean_rate_rad_s", "a zero lean rate");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.pitch_rate_up_rad_s = 0.; })); },
            "pitch_rate_up_rad_s", "a zero pitch-rate threshold");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) {
                t.pitch_rad = std::numeric_limits<double>::quiet_NaN(); })); },
            "pitch_rad", "a NaN pitch threshold");
    rejects([&] { rider::validate_reaction_table(with([](auto &t) { t.clear_s = -kPeriod; })); },
            "clear_s", "a negative clear time");
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.decision_s = 1.5 * kPeriod; }), kPeriod, .35); },
            "decision_s", "a decision time off the tick grid");
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.clear_s = 0x1p40 * kPeriod; }), kPeriod, .35); },
            "clear_s", "a clear time beyond 2^31 ticks");
    rejects([&] { rider::validate_reaction_context(table(), kPeriod, .05); },
            "lean_add_rad", "a lean add beyond the forward bound");
    // A positive-required duration must not round down to zero ticks.
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.hand_window_s = 1e-12; }), kPeriod, .35); },
            "hand_window_s", "a sub-tick hand window");
    rejects([&] { rider::validate_reaction_context(with([](auto &t) { t.hand_lag_s = 1e-12; }), kPeriod, .35); },
            "hand_lag_s", "a sub-tick hand lag");
    rejects([&] { (void)rider::FrontLiftReaction(with([](auto &t) { t.hand_window_s = 1e-12; }), kPeriod); },
            "hand_window_s", "the constructor rejects a sub-tick window");
    // Nonnegative durations keep their zero-tick semantics.
    rider::validate_reaction_context(with([](auto &t) {
        t.decision_s = 1e-12;
        t.clear_s = 1e-12;
    }), kPeriod, .35);
    const auto one_tick = with([](auto &t) {
        t.hand_window_s = kPeriod;
        t.hand_lag_s = kPeriod;
        t.decision_s = 0.;
        t.clear_s = 0.;
    });
    rider::validate_reaction_context(one_tick, kPeriod, .35);
    rider::FrontLiftReaction minimal(one_tick, kPeriod);
    (void)minimal.update(percept(100., 0., 0.));
    bt::require(minimal.update(percept(50., 1., 0.)).predicate,
                "a one-tick window and lag evaluate the hand drop");
}

// ---- RiderIntent: perception and the reaction inside the resolver -----------

rider::IntentConfig enabled_intent() {
    auto config = bike::configuration::default_physics().seated_climb;
    config.enabled = true;
    return config;
}

// The 1.25 ms timing tables of this file use table()'s short times; this
// one keeps them with both actions off.
rider::FrontLiftReactionConfig switched_off() {
    auto t = table();
    t.ease_enabled = false;
    t.lean_enabled = false;
    return t;
}

// Launch part 3b: the switched-off launch table (plan_enabled = false);
// its times sit on every test grid.
rider::LaunchConfig switched_off_launch() {
    rider::LaunchConfig t;
    t.start_cog_teeth = {51};
    t.start_pedal_angle_rad = 0.;
    t.start_lean_grades = {0.};
    t.start_lean_rad = {0.};
    t.plan_enabled = false;
    t.standstill_speed_mps = .1;
    t.standstill_crank_rpm = 5.;
    t.effort_rate_nm_s = 40.;
    t.effort_cap_asphalt_nm = t.effort_cap_hardpack_nm = t.effort_cap_loose_nm = t.effort_cap_wet_nm =
        t.effort_cap_scree_nm = 60.;
    t.effort_cap_grades = {0.};
    t.effort_cap_grade_nm = {1000.};
    t.effort_rise_speed_mps = 2.5;
    t.release_torque_grades = {0., .2};
    t.release_torque_nm = {0., 30.};
    t.brake_release_s = .15;
    t.max_hold_s = 1.5;
    t.upshift_crank_rpm = 82.;
    t.shift_retry_s = .25;
    t.end_speed_mps = 3.;
    t.end_crank_rpm = 60.;
    t.end_hold_s = .3;
    t.max_launch_s = 6.;
    t.handover_s = .9;
    return t;
}

bool same_bits(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }

bool same_optional(const std::optional<double> &a, const std::optional<double> &b) {
    return a.has_value() == b.has_value() && (!a || same_bits(*a, *b));
}

bool same_posture(const rider::RiderPosture &a, const rider::RiderPosture &b) {
    return same_bits(a.torso_lean_rad, b.torso_lean_rad) && same_bits(a.pelvis_pitch_rad, b.pelvis_pitch_rad) &&
           a.pelvis_offset_m.has_value() == b.pelvis_offset_m.has_value() &&
           (!a.pelvis_offset_m || (same_bits((*a.pelvis_offset_m)[0], (*b.pelvis_offset_m)[0]) &&
                                   same_bits((*a.pelvis_offset_m)[1], (*b.pelvis_offset_m)[1]))) &&
           a.use_saddle == b.use_saddle;
}

// Every IntentControl field, bit for bit.
bool same_control(const rider::IntentControl &a, const rider::IntentControl &b) {
    return same_optional(a.motor_torque_nm, b.motor_torque_nm) && same_optional(a.motor_limit_nm, b.motor_limit_nm) &&
           same_optional(a.human_torque_nm, b.human_torque_nm) &&
           same_optional(a.crank_target_rate_rad_s, b.crank_target_rate_rad_s) &&
           a.posture.has_value() == b.posture.has_value() && (!a.posture || same_posture(*a.posture, *b.posture)) &&
           a.rider_enabled == b.rider_enabled;
}

std::string encoded(const rider::RiderIntent &intent) {
    return bike::io::canonical_json(rider::encode_intent_state(intent.state()));
}

// The varying inputs every intent test feeds: a pitch-up pulse with light
// hands on steps 161..399, crank and load-share signals that exercise the
// policy's queues and trim, a 6 % preview.
std::pair<rider::IntentSignals, rider::RiderSenses> inputs(std::int64_t step) {
    rider::IntentSignals s;
    s.pitch_rate_up_rad_s = step > 160 && step < 400 ? 1. : 0.;
    s.specific_force_body_mps2 = {.3, 0., 9.81};
    s.crank_rate_rad_s = 6.;
    s.human_crank_torque_nm = 20.;
    s.front_load_share = .35;
    rider::RiderSenses senses;
    senses.hand_press_n = step > 160 && step < 400 ? 5. : 60.;
    senses.grade_ahead = .06;
    return {s, senses};
}

rider::IntentResolution tick(rider::RiderIntent &intent, std::int64_t step, bool advance = true) {
    const auto [signals, senses] = inputs(step);
    return intent.resolve_full(rider::IntentControl{}, signals, senses, step, .06, .06, .35, true, advance);
}

// Oracle (review M3): with both actions off the resolver output is the
// bare ordinary policy's in every IntentControl field, bit for bit, at
// every tick, across a reset with a seeded lean; probes between ticks
// change no resolver state and preview the advancing output exactly.
[[gnu::noinline]] void test_switched_off_resolver_equals_the_bare_policy() {
    const auto config = enabled_intent();
    rider::RiderIntent intent(config, switched_off(), switched_off_launch(), kPeriod);
    rider::SeatedClimbPolicy policy(config);
    for (const auto &[steps, seed] : {std::pair{600, 0.}, std::pair{300, .05}}) {
        intent.reset();
        policy.reset();
        if (seed != 0.) {
            intent.seed_lean(seed);
            policy.seed_lean(seed);
        }
        for (std::int64_t step = 0; step < steps; ++step) {
            const std::string before = encoded(intent);
            const auto probe = tick(intent, step, false);
            bt::require(encoded(intent) == before, "a probe changes no resolver state");
            const auto out = tick(intent, step);
            const auto [signals, senses] = inputs(step);
            const auto bare = policy.update(signals, config.period_s, .06, .06, .35);
            rider::IntentControl expected;
            expected.posture = bare.posture;
            expected.human_torque_nm = bare.effort_ceiling_nm;
            bt::require(same_control(out.control, expected) && !out.effort_limited,
                        "switched-off actions leave the resolver output bitwise the policy's");
            bt::require(same_control(probe.control, out.control) && probe.effort_limited == out.effort_limited,
                        "the probe previews the advancing output");
        }
    }
}

// Caller-set effort and posture pass through untouched — never eased,
// never flagged — while the reaction acts; rider-off and inactive calls
// return the control as given; a disabled intent has no perception or
// reaction state and reports the neutral command.
[[gnu::noinline]] void test_caller_overrides_and_disabled_paths() {
    const auto config = enabled_intent();
    rider::RiderIntent intent(config, table(), switched_off_launch(), kPeriod);
    rider::IntentControl given;
    given.human_torque_nm = 12.5;
    given.motor_limit_nm = 30.;
    rider::RiderPosture posture;
    posture.torso_lean_rad = .02;
    given.posture = posture;
    for (std::int64_t step = 0; step < 320; ++step) {
        const auto [signals, senses] = inputs(step);
        const auto out = intent.resolve_full(given, signals, senses, step, .06, .06, .35);
        bt::require(same_control(out.control, given) && !out.effort_limited,
                    "caller-set fields win and are never eased");
    }
    bt::require(intent.reaction().phase == rider::ReactionPhase::acting, "the reaction acts meanwhile");
    rider::IntentControl off;
    off.rider_enabled = false;
    const auto [signals, senses] = inputs(320);
    bt::require(same_control(intent.resolve_full(off, signals, senses, 320, .06, .06, .35).control, off),
                "a rider-off call returns the control as given");
    const std::string before = encoded(intent);
    bt::require(same_control(intent.resolve_full(rider::IntentControl{}, signals, senses, 321, .06, .06, .35,
                                                 false).control,
                             rider::IntentControl{}),
                "an inactive call returns the control as given");
    bt::require(encoded(intent) == before, "an inactive call advances nothing");
    auto disabled_config = config;
    disabled_config.enabled = false;
    const rider::RiderIntent disabled(disabled_config, table(), switched_off_launch(), kPeriod);
    bt::require(!disabled.state().perception && !disabled.state().reaction, "a disabled intent has no new state");
    const auto neutral = disabled.reaction();
    bt::require(neutral.phase == rider::ReactionPhase::idle && !neutral.predicate && neutral.effort_factor == 1. &&
                    neutral.lean_add_rad == 0.,
                "a disabled intent reports the neutral reaction");
}

// With both actions on: the delayed light-hand samples arrive from step
// 281 and the predicate rises on the 14th of them against the 60 N
// reference, so the ease starts exactly reaction_delay + 13 + decision
// ticks after the first lifting sample (161 + 120 + 13 + 8 = 302); the
// lean add goes through the posture program, so the total lean never
// outruns its rate; probe and advance agree on output, flag and the whole
// owner state; encode → decode → restore during warm-up (50), deciding
// (299) and releasing (400: the predicate ends at 359, releasing runs
// 375-446) continues with the same output and the same serialized state.
[[gnu::noinline]] void test_reacting_resolver_timing_slew_probe_and_restore() {
    const auto config = enabled_intent();
    // -Wframe-larger-than: the three resolvers live on the heap.
    auto reacting = std::make_unique<rider::RiderIntent>(config, table(), switched_off_launch(), kPeriod);
    auto inert = std::make_unique<rider::RiderIntent>(config, switched_off(), switched_off_launch(), kPeriod);
    const std::int64_t delay = std::llround(config.reaction_delay_s / config.period_s);
    // The predicate rises on the 14th light sample against the 60 N
    // reference (ratio .7, fallback window 40).
    const std::int64_t onset = 161 + delay + 13 + 8;
    double previous_lean = 0.;
    std::unique_ptr<rider::RiderIntent> restored;
    for (std::int64_t step = 0; step < 900; ++step) {
        const std::string before = encoded(*reacting);
        const auto probe = tick(*reacting, step, false);
        bt::require(encoded(*reacting) == before, "a probe changes no resolver state");
        const auto a = tick(*reacting, step);
        const auto b = tick(*inert, step);
        bt::require(same_control(probe.control, a.control) && probe.effort_limited == a.effort_limited,
                    "the probe previews the advancing output and its cut order");
        if (step < onset)
            // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
            bt::require(same_bits(*a.control.human_torque_nm, *b.control.human_torque_nm) && !a.effort_limited,
                        "no ease before delay + dwell + decision");
        if (step == onset)
            // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
            bt::require(*a.control.human_torque_nm < *b.control.human_torque_nm && a.effort_limited,
                        "the ease starts on the computed tick");
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the posture
        const double lean = a.control.posture->torso_lean_rad;
        bt::require(std::abs(lean - previous_lean) <= config.lean_rate_rad_s * config.period_s + 1e-15,
                    "the total lean never outruns the posture rate");
        previous_lean = lean;
        if (step == 50 || step == onset - 3 || step == 400) {
            const auto phase = reacting->reaction().phase;
            bt::require(step != onset - 3 || phase == rider::ReactionPhase::deciding, "snapshot while deciding");
            bt::require(step != 400 || phase == rider::ReactionPhase::releasing, "snapshot while releasing");
            restored = std::make_unique<rider::RiderIntent>(config, table(), switched_off_launch(), kPeriod);
            restored->restore(rider::decode_intent_state(rider::encode_intent_state(reacting->state()), "intent"));
        } else if (restored) {
            const auto r = tick(*restored, step);
            bt::require(same_control(r.control, a.control) && r.effort_limited == a.effort_limited &&
                            encoded(*restored) == encoded(*reacting),
                        "the restored resolver continues bitwise, state included");
        }
    }
    bt::require(reacting->reaction().phase == rider::ReactionPhase::idle, "the reaction ended after the pulse");
}

// Every rejected call leaves the whole resolver untouched, and the
// corrected retry of that step equals one successful tick from the same
// state (review-2 I1, review-3 I3): a NaN sense, a NaN road grade, a NaN
// lean limit, and finite limits whose posture validate_posture rejects
// after the policy has computed (lean_limit_rad = min_lean_rad = 1).
[[gnu::noinline]] void test_rejected_ticks_change_nothing() {
    const auto config = enabled_intent();
    rider::RiderIntent intent(config, table(), switched_off_launch(), kPeriod);
    for (std::int64_t step = 0; step < 300; ++step) (void)tick(intent, step);   // predicate from 294, acting from 302
    const std::string before = encoded(intent);
    rider::RiderIntent reference(intent);
    const auto good = tick(reference, 300);
    const auto [signals, senses] = inputs(300);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto bad_senses = senses;
    bad_senses.hand_press_n = nan;
    rejects([&] { (void)intent.resolve_full(rider::IntentControl{}, signals, bad_senses, 300, .06, .06, .35); },
            "hand_press_n", "a NaN hand press");
    bt::require(encoded(intent) == before, "a rejected sense advances nothing");
    rejects([&] { (void)intent.resolve_full(rider::IntentControl{}, signals, senses, 300, nan, .06, .35); },
            "grade", "a NaN road grade");
    bt::require(encoded(intent) == before, "a rejected grade advances nothing");
    rejects([&] { (void)intent.resolve_full(rider::IntentControl{}, signals, senses, 300, .06, .06, nan); },
            "lean limit", "a NaN lean limit");
    bt::require(encoded(intent) == before, "a rejected lean limit advances nothing");
    rejects([&] {
        (void)intent.resolve_full(rider::IntentControl{}, signals, senses, 300, .06, .06, 1., true, true, 1.);
    }, "torso_lean_rad", "a posture the limits force out of its envelope");
    bt::require(encoded(intent) == before, "a rejected posture advances nothing");
    const auto retry = tick(intent, 300);
    bt::require(same_control(retry.control, good.control) && retry.effort_limited == good.effort_limited &&
                    encoded(intent) == encoded(reference),
                "the corrected retry is exactly one tick");
}

// Allocation failures (native-engineering «Validate and stage»): each
// failing allocation inside a cold tick — the staged twin and the queues
// grow — leaves the resolver unchanged, and the retry after disarming
// equals one successful tick. A warm tick may allocate nothing by design.
[[gnu::noinline]] void test_allocation_failures_change_nothing() {
    const auto config = enabled_intent();
    const auto [signals, senses] = inputs(0);
    std::size_t needed = 0;
    {
        rider::RiderIntent measured(config, table(), switched_off_launch(), kPeriod);
        const allocation_faults::Guard guard;
        allocation_faults::arm(SIZE_MAX);
        (void)measured.resolve_full(rider::IntentControl{}, signals, senses, 0, .06, .06, .35);
        allocation_faults::disarm();
        needed = allocation_faults::allocated();
    }
    bt::require(needed > 0, "a cold tick allocates, so the sweep exercises failures");
    for (std::size_t k = 0; k < needed; ++k) {
        rider::RiderIntent intent(config, table(), switched_off_launch(), kPeriod);
        rider::RiderIntent reference(intent);
        const std::string before = encoded(intent);
        bool threw = false;
        {
            const allocation_faults::Guard guard;
            allocation_faults::arm(k);
            try {
                (void)intent.resolve_full(rider::IntentControl{}, signals, senses, 0, .06, .06, .35);
            } catch (const std::bad_alloc &) {
                threw = true;
            }
        }
        bt::require(threw, "the armed allocation fails");
        bt::require(encoded(intent) == before, "a failed allocation changes nothing");
        const auto retry = intent.resolve_full(rider::IntentControl{}, signals, senses, 0, .06, .06, .35);
        const auto good = reference.resolve_full(rider::IntentControl{}, signals, senses, 0, .06, .06, .35);
        bt::require(same_control(retry.control, good.control) && encoded(intent) == encoded(reference),
                    "the retry after a failed allocation is exactly one tick");
    }
}

// The posture-target add rejects NaN, infinity and a backward add before
// the program moves (review-2 I6); a valid add is accepted. The policy
// forwards the add and rejects the same values (it is not a transaction
// on its own — RiderIntent is).
[[gnu::noinline]] void test_extra_lean_domain() {
    const auto config = enabled_intent();
    const auto [signals, senses] = inputs(0);
    for (const double extra : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -.01}) {
        rider::SeatedPostureProgram program(config);
        const auto before = program.state();
        rejects([&] { (void)program.update(0., .06, config.period_s, .35, .35, std::nullopt, std::nullopt, extra); },
                "extra", "an invalid lean add on the program");
        bt::require(program.state().lean_rad == before.lean_rad &&
                        program.state().load_samples.size() == before.load_samples.size(),
                    "a rejected add moves no program state");
        rider::SeatedClimbPolicy policy(config);
        rejects([&] { (void)policy.update(signals, config.period_s, .06, .06, .35, std::nullopt, extra); },
                "extra", "an invalid lean add on the policy");
    }
    rider::SeatedClimbPolicy policy(config);
    (void)policy.update(signals, config.period_s, .06, .06, .35, std::nullopt, .05);   // control
}

// Restore rejects what breaks a necessary condition of resolve_full
// (decision 12): presence, the perception clock against the intention
// ticks (without overflow), the reaction's percept count against the
// delivered percepts, and the component checks. A reaction grafted onto
// the first delivery is rejected; that state's own reaction is accepted.
// Rejected restores change nothing; the document path rejects the same.
[[gnu::noinline]] void test_restore_checks_the_joint_state() {
    const auto config = enabled_intent();
    const std::int64_t delay = std::llround(config.reaction_delay_s / config.period_s);   // 120
    // -Wframe-larger-than: the intents and the staged states live on the heap.
    auto donor = std::make_unique<rider::RiderIntent>(config, table(), switched_off_launch(), kPeriod);
    for (std::int64_t step = 0; step < 300; ++step) (void)tick(*donor, step);
    auto first = std::make_unique<rider::RiderIntent>(config, table(), switched_off_launch(), kPeriod);
    for (std::int64_t step = 0; step <= delay; ++step) (void)tick(*first, step);   // exactly one percept delivered
    rider::RiderIntent fresh(config, table(), switched_off_launch(), kPeriod);
    const std::string before = encoded(fresh);
    auto early = std::make_unique<rider::RiderIntent::State>(fresh.state());
    early->reaction = donor->state().reaction;   // deciding, from another episode
    rejects([&] { fresh.restore(*early); }, "percepts_seen", "a reaction before any percept");
    rejects([&] { fresh.restore(rider::decode_intent_state(rider::encode_intent_state(*early), "intent")); },
            "percepts_seen", "the same state through the document");
    auto graft = std::make_unique<rider::RiderIntent::State>(first->state());
    graft->reaction = donor->state().reaction;
    rejects([&] { fresh.restore(*graft); }, "percepts_seen", "a deciding reaction on the first delivery");
    auto bad = std::make_unique<rider::RiderIntent::State>(donor->state());
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the donor intent is enabled, so the pair is engaged
    bad->perception->tick += 1;
    rejects([&] { fresh.restore(*bad); }, "perception tick", "a perception clock off the intention ticks");
    *bad = donor->state();
    bad->last_tick_step = -2;
    rejects([&] { fresh.restore(*bad); }, "intention grid", "a tick before the first one");
    *bad = donor->state();
    bad->last_tick_step = std::numeric_limits<std::int64_t>::max();   // period_steps == 1 here
    rejects([&] { fresh.restore(*bad); }, "intention grid", "a tick count that would overflow");
    rejects([&] { fresh.restore(rider::decode_intent_state(rider::encode_intent_state(*bad), "intent")); },
            "intention grid", "the overflowing tick through the document");
    *bad = donor->state();
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the donor intent is enabled, so the pair is engaged
    bad->reaction->effort_factor = .1;   // below 1 - ease_fraction
    bad->policy.time_s += 1.;            // must not leak when the restore fails
    rejects([&] { fresh.restore(*bad); }, "effort_factor", "a reaction outside its switch range");
    *bad = donor->state();
    bad->perception.reset();
    rejects([&] { fresh.restore(*bad); }, "perception", "an enabled intent without perception state");
    bt::require(encoded(fresh) == before, "rejected restores change nothing");
    fresh.restore(first->state());   // control: the first delivery with its own reaction
    bt::require(encoded(fresh) == encoded(*first), "the first-delivery state restores");
    fresh.restore(donor->state());   // control
    bt::require(encoded(fresh) == encoded(*donor), "a reachable state restores");
}

} // namespace

int main() {
    return bt::run([] {
        test_criterion_b_equals_the_probe_replay();
        test_predicate_window_and_decision_timing();
        test_a_hand_step_extinguishes_itself();
        test_held_pitch_triggers_and_light_hands_alone_do_not();
        test_short_predicate_is_dropped_while_deciding();
        test_each_action_has_its_own_switch();
        test_recovery_after_clear();
        test_codec_round_trip_and_continuation();
        test_restore_rejects_unreachable_states();
        test_table_rules();
        test_switched_off_resolver_equals_the_bare_policy();
        test_caller_overrides_and_disabled_paths();
        test_reacting_resolver_timing_slew_probe_and_restore();
        test_rejected_ticks_change_nothing();
        test_allocation_failures_change_nothing();
        test_extra_lean_domain();
        test_restore_checks_the_joint_state();
    });
}
