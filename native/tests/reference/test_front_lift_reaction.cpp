// Launch part 3a — the front-lift reaction node: its timing on the delayed
// percepts, the two independent actions, recovery, the codec round trip
// with bitwise continuation, atomic restore and the table rules.
#include "support/fixture_test.hpp"
#include "io/json.hpp"
#include "rider/front_lift_reaction.hpp"
#include "rider/senses_codec.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
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

// Dwell 4 ticks, decision 8 ticks, clear 16 ticks; both actions on.
rider::FrontLiftReactionConfig table() {
    return {.hand_unload_n = 20., .unload_dwell_s = 4 * kPeriod, .pitch_rate_up_rad_s = .5,
            .pitch_rad = .17, .decision_s = 8 * kPeriod, .ease_enabled = true, .lean_enabled = true,
            .rear_brake_tap_enabled = false, .ease_fraction = .7, .ease_rate_per_s = 5.,
            .lean_add_rad = .1, .lean_rate_rad_s = .5, .clear_s = 16 * kPeriod};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) mirrors the Percepts field order
rider::Percepts percept(double hand_press_n, double pitch_rate, double pitch) {
    rider::Percepts p;
    p.senses.hand_press_n = hand_press_n;
    p.pitch_rate_up_rad_s = pitch_rate;
    p.pitch_rad = pitch;
    return p;
}

std::string text(const rider::FrontLiftReaction &r) {
    return bike::io::canonical_json(rider::encode_reaction_state(r.state()));
}

// Unloaded hands must last longer than the dwell (4 samples) before the
// predicate rises; the action starts after the decision time (8 ticks);
// the first acting tick already moves both outputs one rate step.
void test_predicate_dwell_and_decision_timing() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    const auto lifting = percept(5., 1., 0.);
    std::vector<rider::ReactionCommand> out;
    out.reserve(20);
    for (int tick = 0; tick < 20; ++tick) out.push_back(reaction.update(lifting));
    for (std::size_t tick = 0; tick < 4; ++tick)
        bt::require(!out[tick].predicate, "no predicate within the dwell");
    bt::require(out[4].predicate && out[4].phase == rider::ReactionPhase::deciding,
                "the predicate rises when the unload outlasts the dwell");
    for (std::size_t tick = 4; tick < 12; ++tick)
        bt::require(out[tick].phase == rider::ReactionPhase::deciding && out[tick].effort_factor == 1.,
                    "the rider is still deciding");
    bt::require(out[12].phase == rider::ReactionPhase::acting, "the action starts after the decision time");
    bt::require(std::abs(out[12].effort_factor - (1. - 5. * kPeriod)) < 1e-15 &&
                    std::abs(out[12].lean_add_rad - .5 * kPeriod) < 1e-15,
                "acting moves both outputs one rate step");
}

// A held-high wheel triggers through the accumulated pitch with a small
// rate; light hands without pitch never trigger; no percepts keep idle.
void test_held_pitch_triggers_and_light_hands_alone_do_not() {
    rider::FrontLiftReaction held(table(), kPeriod);
    rider::ReactionCommand last;
    for (int tick = 0; tick < 5; ++tick) last = held.update(percept(5., .01, .3));
    bt::require(last.predicate, "a held pitch above the threshold makes the predicate");
    rider::FrontLiftReaction light(table(), kPeriod);
    for (int tick = 0; tick < 40; ++tick) last = light.update(percept(5., .01, .01));
    bt::require(!last.predicate && last.phase == rider::ReactionPhase::idle, "light hands alone never trigger");
    rider::FrontLiftReaction blind(table(), kPeriod);
    for (int tick = 0; tick < 40; ++tick) last = blind.update(std::nullopt);
    bt::require(last.phase == rider::ReactionPhase::idle && !last.predicate, "no percepts, no reaction");
}

// A predicate that vanishes during the decision returns to idle without
// any action.
void test_short_predicate_is_dropped_while_deciding() {
    rider::FrontLiftReaction reaction(table(), kPeriod);
    rider::ReactionCommand last;
    for (int tick = 0; tick < 6; ++tick) last = reaction.update(percept(5., 1., 0.));
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
        rider::ReactionCommand last;
        for (int tick = 0; tick < 200; ++tick) last = reaction.update(percept(5., 1., 0.));
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
    rider::ReactionCommand last;
    for (int tick = 0; tick < 200; ++tick) last = reaction.update(percept(5., 1., 0.));
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
// reaction continues bitwise (whole state compared as canonical JSON).
void test_codec_round_trip_and_continuation() {
    const auto input = [](int tick) { return tick < 200 ? percept(5., 1., 0.) : percept(80., 0., 0.); };
    for (const int split : {7, 230}) {
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
// switch forbids, a phase outside the enum and inconsistent counters; a
// reachable state from the same configuration is accepted.
void test_restore_rejects_unreachable_states() {
    auto off = table();
    off.ease_enabled = false;
    off.lean_enabled = false;
    rider::FrontLiftReaction target(off, kPeriod);
    for (int tick = 0; tick < 6; ++tick) (void)target.update(percept(5., 1., 0.));
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
    bad.phase = static_cast<rider::ReactionPhase>(9);
    rejects([&] { target.restore(bad); }, "phase", "a phase outside the enum");
    bad = target.state();
    bad.predicate = false;
    rejects([&] { target.restore(bad); }, "predicate", "deciding without the predicate");
    bad = target.state();
    bad.unload_ticks = std::numeric_limits<std::int64_t>::max();
    rejects([&] { target.restore(bad); }, "unload_ticks", "an unload counter past saturation");
    bt::require(text(target) == before, "rejected restores change nothing");
    target.restore(target.state());   // control: its own reachable state

    // Phase/predicate/counter combinations update() never leaves behind
    // (review-2 I3), on a reaction with both actions on.
    rider::FrontLiftReaction on(table(), kPeriod);
    for (int tick = 0; tick < 200; ++tick) (void)on.update(percept(5., 1., 0.));
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
    bad.unload_ticks = 5;
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
    bad.percepts_seen = 12;   // dwell + 1 + decision = 13 percepts are needed
    rejects([&] { on.restore(bad); }, "percepts_seen", "acting before the dwell and decision could pass");
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
}

} // namespace

int main() {
    return bt::run([] {
        test_predicate_dwell_and_decision_timing();
        test_held_pitch_triggers_and_light_hands_alone_do_not();
        test_short_predicate_is_dropped_while_deciding();
        test_each_action_has_its_own_switch();
        test_recovery_after_clear();
        test_codec_round_trip_and_continuation();
        test_restore_rejects_unreachable_states();
        test_table_rules();
    });
}
