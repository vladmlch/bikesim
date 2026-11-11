// test_rider_perception.cpp — launch part 1, task 3: the delayed,
// periodic senses adapter (Perception) and its restorable state.
//
// The rider perceives every sensation with the same reaction delay —
// grade included — and integrates pitch from the delayed vestibular
// rate. The state survives serialize -> read -> continue bitwise (user
// ruling 2026-11-11: component-level restore). Review P10: restore
// admits only reachable states — the contiguous, on-grid, not-yet-due
// tail of domain-valid samples — and rejects without touching the
// adapter; update() validates its input before the queue or clock move.

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "io/json.hpp"
#include "io/value.hpp"
#include "rider/perception.hpp"
#include "rider/senses_codec.hpp"
#include "support/fixture_test.hpp"

namespace {

namespace bio = bike::io;

rider::SenseSample sample(int k) {
    rider::SenseSample s;
    s.signals.pitch_rate_up_rad_s = .5;
    s.signals.crank_rate_rad_s = 2. * std::numbers::pi;   // 60 rpm
    s.signals.human_crank_torque_nm = static_cast<double>(k);
    s.senses = {.hand_press_n = 100. + k,
                .speed_mps = .01 * k,
                .rear_teeth = 51,
                .surface_front = rider::SurfaceLabel::asphalt,
                .surface_ahead = rider::SurfaceLabel::loose,
                .grade_ahead = .001 * k};
    return s;
}

// Every Percepts field compares — not a subset (P10).
void require_same(const rider::Percepts &a, const rider::Percepts &b,
                  std::string_view what) {
    bt::require(a.senses.hand_press_n == b.senses.hand_press_n &&
                    a.senses.speed_mps == b.senses.speed_mps &&
                    a.senses.rear_teeth == b.senses.rear_teeth &&
                    a.senses.surface_front == b.senses.surface_front &&
                    a.senses.surface_ahead == b.senses.surface_ahead &&
                    a.senses.grade_ahead == b.senses.grade_ahead &&
                    a.crank_rpm == b.crank_rpm &&
                    a.human_crank_torque_nm == b.human_crank_torque_nm &&
                    a.pitch_rate_up_rad_s == b.pitch_rate_up_rad_s &&
                    a.pitch_rad == b.pitch_rad,
                what);
}

std::string state_json(const rider::Perception::State &state) {
    return bio::canonical_json(rider::encode_perception_state(state));
}

// The mutable twin of bio::find_member for building malformed fixtures.
[[nodiscard]] bio::Value &mut_member(bio::Value::Object &object,
                                    std::string_view key) {
    for (auto &[name, value] : object)
        if (name == key) return value;
    throw std::runtime_error("mut_member: fixture key is missing");
}

[[nodiscard]] bio::Value::Object &as_object(bio::Value &v) {
    return std::get<bio::Value::Object>(v.data);
}

[[nodiscard]] bio::Value::Array &as_array(bio::Value &v) {
    return std::get<bio::Value::Array>(v.data);
}

// Nothing arrives inside the delay; afterwards every field, grade
// included, is the sample exactly one delay earlier.
void test_every_sensation_is_delayed_together() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 15; ++k)
        bt::require(!p.update(sample(k)).has_value(),
                    "nothing inside the reaction delay");
    const auto first = p.update(sample(15));
    bt::require(first.has_value(),
                "the first percept arrives at the delay");
    const auto &percept =
        (first).value(); // NOLINT(bugprone-unchecked-optional-access) engagement proven by the require above; .value() reports violations loudly
    bt::require(percept.human_crank_torque_nm == 0. &&
                    percept.senses.hand_press_n == 100. &&
                    percept.senses.grade_ahead == 0.,
                "torque, hands and grade are the sample from one delay "
                "earlier");
    bt::require(std::abs(percept.crank_rpm - 60.) < 1e-9,
                "crank rate in rpm");
}

// pitch_rad integrates only delayed rates: 21 delayed ticks of
// .5 rad/s.
void test_pitch_integrates_the_delayed_rate() {
    rider::Perception p(.15, .01);
    std::optional<rider::Percepts> last;
    for (int k = 0; k <= 35; ++k) last = p.update(sample(k));
    bt::require(last.has_value() &&
                    std::abs(last->pitch_rad - 21 * .5 * .01) < 1e-12,
                "pitch integrates the delayed rate");
}

// Serialize, read back into a fresh adapter, continue: identical — the
// genuinely pre-delivery (10 < 15 ticks) and the post-delivery (20)
// cases, comparing every returned field and the full state (P10). The
// restored adapter also replaces state it already holds.
void test_state_serialize_read_continue_is_bitwise() {
    for (const int mark : {10, 20}) {
        rider::Perception a(.15, .01);
        for (int k = 0; k < mark; ++k) (void)a.update(sample(k));
        rider::Perception b(.15, .01);
        for (int k = 0; k < 3; ++k) (void)b.update(sample(100 + k));
        b.restore(rider::decode_perception_state(
            rider::encode_perception_state(a.state()), "perception"));
        for (int k = mark; k < 60; ++k) {
            const auto x = a.update(sample(k));
            const auto y = b.update(sample(k));
            bt::require(x.has_value() == y.has_value(),
                        "same availability");
            if (x.has_value() && y.has_value())
                require_same(*x, *y,
                             "restored adapter continues bitwise");
        }
        bt::require(state_json(a.state()) == state_json(b.state()),
                    "restored state matches bitwise after continuation");
    }
}

// A delay that is not on the intention grid is rejected at
// construction.
void test_off_grid_delay_is_rejected() {
    bt::invalid([] { (void)rider::Perception(.155, .01); });
}

// A malformed restored state (a sample timed after the clock) is
// rejected and leaves the adapter unchanged.
void test_malformed_state_is_rejected_atomically() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 5; ++k) (void)p.update(sample(k));
    const std::string before = state_json(p.state());
    auto bad = p.state();
    bad.samples.emplace_back(bad.time_s + 1., sample(99));
    bt::invalid([&] { p.restore(bad); });
    bt::require(state_json(p.state()) == before, "rejected atomically");
}

// Restore rejects every unreachable queue — reordered, duplicated,
// already-due or after-the-clock sample times — without touching the
// adapter (P10).
void test_unreachable_queue_is_rejected_atomically() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 20; ++k) (void)p.update(sample(k));
    const std::string before = state_json(p.state());
    {
        auto bad = p.state();
        std::swap(bad.samples[0], bad.samples[1]);
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.samples[1].first = bad.samples[0].first;
        bt::invalid([&] { p.restore(bad); });
    }
    // A sample already due can never sit in a live queue: update()
    // would have delivered it.
    {
        auto bad = p.state();
        bad.samples.front().first = bad.time_s - .15 - .01;
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.samples.back().first = bad.time_s;
        bt::invalid([&] { p.restore(bad); });
    }
    // A dropped or inserted sample breaks the contiguous tail.
    {
        auto bad = p.state();
        bad.samples.pop_front();
        bt::invalid([&] { p.restore(bad); });
    }
    bt::require(state_json(p.state()) == before,
                "every unreachable queue rejects without touching state");
}

// Invalid sample contents — inside the queue or the delayed slot — are
// rejected the same way, along with a non-finite clock or integrator.
void test_invalid_sample_contents_are_rejected() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 20; ++k) (void)p.update(sample(k));
    const std::string before = state_json(p.state());
    {
        auto bad = p.state();
        bad.samples[3].second.senses.hand_press_n =
            std::numeric_limits<double>::quiet_NaN();
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.samples[4].second.signals.crank_rate_rad_s =
            std::numeric_limits<double>::infinity();
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.delayed.value() // NOLINT(bugprone-unchecked-optional-access) 20 ticks > 15-tick delay: engagement is the fixture contract; .value() reports violations loudly
            .senses.rear_teeth = 0;
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.delayed.value() // NOLINT(bugprone-unchecked-optional-access) engagement is the fixture contract; .value() reports violations loudly
            .signals.front_load_share = 1.5;
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.delayed.value() // NOLINT(bugprone-unchecked-optional-access) engagement is the fixture contract; .value() reports violations loudly
            .senses.surface_front = static_cast<rider::SurfaceLabel>(9);
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.delayed = std::nullopt;
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.pitch_rad = -std::numeric_limits<double>::infinity();
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.time_s = -0.005;
        bt::invalid([&] { p.restore(bad); });
    }
    bt::require(state_json(p.state()) == before,
                "invalid contents reject without touching state");
}

// A poisoned acquisition is rejected before the queue or the clock
// move at all.
void test_update_rejects_bad_input_without_mutation() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 5; ++k) (void)p.update(sample(k));
    const std::string before = state_json(p.state());
    auto bad = sample(5);
    bad.signals.pitch_rate_up_rad_s =
        std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] { (void)p.update(bad); });
    bt::require(state_json(p.state()) == before,
                "a poisoned sample mutates nothing");
}

// Zero delay delivers each acquisition in the period it was taken.
void test_zero_delay_delivers_immediately() {
    rider::Perception p(0., .01);
    const auto first = p.update(sample(0));
    bt::require(first.has_value(),
                "zero delay delivers in the same period");
    const auto &percept =
        (first).value(); // NOLINT(bugprone-unchecked-optional-access) engagement proven by the require above; .value() reports violations loudly
    bt::require(percept.senses.hand_press_n == 100. &&
                    percept.human_crank_torque_nm == 0. &&
                    std::abs(percept.pitch_rad - .005) < 1e-12,
                "the zero-delay percept is the current sample");
    const auto second = p.update(sample(1));
    bt::require(second.has_value() && second->human_crank_torque_nm == 1.,
                "the next period delivers the next sample");
    bt::require(p.state().samples.empty(), "nothing stays queued");
}

// reset() returns the adapter to its constructed state.
void test_reset_returns_to_constructed_state() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 20; ++k) (void)p.update(sample(k));
    p.reset();
    const rider::Perception fresh(.15, .01);
    bt::require(state_json(p.state()) == state_json(fresh.state()),
                "reset restores the constructed state");
}

// The state document is an exact four-key record whose rows carry the
// signals/senses readers' domains: a missing or unknown key, a
// malformed row or a bad nested member rejects; the decoded state
// round-trips bitwise.
void test_perception_state_codec_is_exact_and_bitwise() {
    rider::Perception p(.15, .01);
    for (int k = 0; k < 20; ++k) (void)p.update(sample(k));
    const bio::Value encoded = rider::encode_perception_state(p.state());
    const auto decoded =
        rider::decode_perception_state(encoded, "perception");
    bt::require(bio::canonical_json(
                    rider::encode_perception_state(decoded)) ==
                    bio::canonical_json(encoded),
                "state document round-trips bitwise");
    {
        auto missing = encoded;
        auto &members = std::get<bio::Value::Object>(missing.data);
        std::erase_if(members,
                      [](const auto &kv) { return kv.first == "pitch_rad"; });
        bt::invalid([&] {
            (void)rider::decode_perception_state(missing, "perception");
        });
    }
    {
        auto unknown = encoded;
        auto &members = std::get<bio::Value::Object>(unknown.data);
        members.emplace_back("bogus", bio::Value{});
        bt::invalid([&] {
            (void)rider::decode_perception_state(unknown, "perception");
        });
    }
    {
        auto wide = encoded;
        as_array(mut_member(as_object(wide), "samples"))
            .front()
            .data = bio::Value::Array{bio::Value{}, bio::Value{},
                                     bio::Value{}};
        bt::invalid([&] {
            (void)rider::decode_perception_state(wide, "perception");
        });
    }
    {
        auto bad_cog = encoded;
        bio::Value &row = as_array(mut_member(as_object(bad_cog), "samples"))
                              .front();
        bio::Value &record = as_array(row).back();
        mut_member(as_object(mut_member(as_object(record), "senses")),
                   "rear_teeth") = bio::Value{bio::Number{"2"}};
        bt::invalid([&] {
            (void)rider::decode_perception_state(bad_cog, "perception");
        });
    }
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_every_sensation_is_delayed_together();
        test_pitch_integrates_the_delayed_rate();
        test_state_serialize_read_continue_is_bitwise();
        test_off_grid_delay_is_rejected();
        test_malformed_state_is_rejected_atomically();
        test_unreachable_queue_is_rejected_atomically();
        test_invalid_sample_contents_are_rejected();
        test_update_rejects_bad_input_without_mutation();
        test_zero_delay_delivers_immediately();
        test_reset_returns_to_constructed_state();
        test_perception_state_codec_is_exact_and_bitwise();
    });
}
