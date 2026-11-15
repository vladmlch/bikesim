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

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <numbers>
#include <optional>
#include <print>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "configuration/load.hpp"
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

// Bitwise equality on every Percepts field — bit patterns, not ==.
void require_same_bits(const rider::Percepts &a, const rider::Percepts &b,
                       std::string_view what) {
    const auto bits = [](double v) {
        return std::bit_cast<std::uint64_t>(v);
    };
    bt::require(bits(a.senses.hand_press_n) == bits(b.senses.hand_press_n) &&
                    bits(a.senses.speed_mps) == bits(b.senses.speed_mps) &&
                    a.senses.rear_teeth == b.senses.rear_teeth &&
                    a.senses.surface_front == b.senses.surface_front &&
                    a.senses.surface_ahead == b.senses.surface_ahead &&
                    bits(a.senses.grade_ahead) ==
                        bits(b.senses.grade_ahead) &&
                    bits(a.crank_rpm) == bits(b.crank_rpm) &&
                    bits(a.human_crank_torque_nm) ==
                        bits(b.human_crank_torque_nm) &&
                    bits(a.pitch_rate_up_rad_s) ==
                        bits(b.pitch_rate_up_rad_s) &&
                    bits(a.pitch_rad) == bits(b.pitch_rad),
                what);
}

// A varying but always domain-valid acquisition; the torque carries the
// push index so delivery ticks can be matched across adapters.
rider::SenseSample ride_sample(int k) {
    rider::SenseSample s;
    s.signals.pitch_rate_up_rad_s = .02 * ((k % 11) - 5);
    s.signals.specific_force_body_mps2 = {.1 * (k % 7), 0., 9.81};
    s.signals.crank_rate_rad_s = 5. + .01 * (k % 40);
    s.signals.human_crank_torque_nm = static_cast<double>(k);
    if (k % 4 == 1) s.signals.front_load_share = .3;
    s.senses = {.hand_press_n = 30. + (k % 80),
                .speed_mps = 4. + .0001 * (k % 300),
                .rear_teeth = 11 + (k % 41),
                .surface_front =
                    static_cast<rider::SurfaceLabel>(k % 5),
                .surface_ahead =
                    static_cast<rider::SurfaceLabel>((k + 2) % 5),
                .grade_ahead = .001 * ((k % 13) - 6)};
    return s;
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
    bad.samples.emplace_back(bad.tick + 1000, sample(99));
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
        bad.samples.front().first = bad.tick - 16;
        bt::invalid([&] { p.restore(bad); });
    }
    {
        auto bad = p.state();
        bad.samples.back().first = bad.tick;
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
        bad.tick = -1;
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

// An overflowing output changes nothing and a corrected retry advances one tick.
void test_output_overflow_rejects_atomically() {
    {
        rider::Perception p(0., .01);
        rider::Perception::State seeded;
        seeded.tick = 1;
        seeded.delayed = sample(0);
        seeded.pitch_rad = std::numeric_limits<double>::max();
        p.restore(seeded);
        const std::string before = state_json(p.state());
        auto boom = sample(1);
        boom.signals.pitch_rate_up_rad_s = 1e308;
        bt::invalid([&] { (void)p.update(boom); });
        bt::require(state_json(p.state()) == before,
                    "the pitch overflow changes nothing");
        rider::Perception reference(0., .01);
        reference.restore(seeded);
        const auto retry = p.update(sample(1));
        const auto good = reference.update(sample(1));
        bt::require(retry.has_value() && good.has_value(),
                    "the corrected retry delivers");
        require_same(*retry, *good, "the retry equals one clean update");
        bt::require(state_json(p.state()) == state_json(reference.state()),
                    "the retried state equals one clean update");
    }
    {
        rider::Perception p(.01, .01);
        rider::Perception::State seeded;
        seeded.tick = 2;
        seeded.delayed = sample(0);
        seeded.pitch_rad = std::numeric_limits<double>::max();
        auto queued = sample(7);
        queued.signals.pitch_rate_up_rad_s = 1e308;
        seeded.samples.emplace_back(1, queued);
        p.restore(seeded);
        const std::string before = state_json(p.state());
        bt::invalid([&] { (void)p.update(sample(2)); });
        bt::require(state_json(p.state()) == before,
                    "a queued overflowing sample changes nothing either");
    }
    {
        rider::Perception p(0., .01);
        rider::Perception::State seeded;
        seeded.tick = 1;
        seeded.delayed = sample(0);
        p.restore(seeded);
        const std::string before = state_json(p.state());
        auto boom = sample(1);
        boom.signals.crank_rate_rad_s = std::numeric_limits<double>::max();
        bt::invalid([&] { (void)p.update(boom); });
        bt::require(state_json(p.state()) == before,
                    "a crank-rate overflow changes nothing");
        const auto retry = p.update(sample(1));
        bt::require(retry.has_value() && p.state().tick == 2,
                    "the corrected retry advances one tick");
    }
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
    // `tick` is a strict nonnegative int64: a real lexeme, a negative
    // and an out-of-range integer all reject.
    for (const char *const lexeme : {"3.5", "-1", "9223372036854775808"}) {
        auto bad_tick = encoded;
        mut_member(as_object(bad_tick), "tick") =
            bio::Value{bio::Number{lexeme}};
        bt::invalid([&] {
            (void)rider::decode_perception_state(bad_tick, "perception");
        });
    }
}

// Follow-up 2 (review of b79ea64b): with the integer tick, every state
// update() emits restores — 200000 updates at both realistic cadences,
// a twin restored every 10000 continuing bitwise for the next 50.
// Within 300 s of simulated time Perception must deliver on exactly the
// tick the intent policy's float queue would (intent.cpp:276-282,322 —
// push at the pre-increment clock, pop every stamp <= clock - delay +
// 1e-12, advance); beyond 300 s the first divergence tick is recorded
// as information only — the float queue may slip while the integer one
// stays exact, and that is the intended difference, not a failure.
void test_long_run_states_stay_restorable() {
    const auto welded = bike::configuration::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));
    const std::array<std::pair<double, double>, 2> cadences{{
        {welded.seated_climb.period_s,
         welded.seated_climb.reaction_delay_s},
        {.01, .15}}};
    for (const auto &[period, delay] : cadences) {
        // Ticks inside the 300 s parity window: update k happens at
        // interval start k*period.
        const int parity_ticks = static_cast<int>(300. / period);
        rider::Perception a(delay, period);
        double intent_clock = 0.;
        std::deque<std::pair<double, int>> intent_queue;
        int first_divergence = -1;
        const auto tick_check = [&](int k, const auto &percept) {
            intent_queue.emplace_back(intent_clock, k);
            int delivered = -1;
            while (!intent_queue.empty() &&
                   intent_queue.front().first <=
                       intent_clock - delay + 1e-12) {
                delivered = intent_queue.front().second;
                intent_queue.pop_front();
            }
            intent_clock += period;
            const bool same =
                percept.has_value() == (delivered >= 0) &&
                (!percept.has_value() ||
                 percept->human_crank_torque_nm == delivered);
            if (k <= parity_ticks)
                bt::require(same,
                            "perception delivers on the intent queue's "
                            "tick");
            else if (!same && first_divergence < 0)
                first_divergence = k;
        };
        for (int k = 0; k < 200000; ++k) {
            tick_check(k, a.update(ride_sample(k)));
            if ((k + 1) % 10000 != 0) continue;
            rider::Perception twin(delay, period);
            twin.restore(a.state());   // a state update() produced
            for (int j = k + 1; j <= k + 50; ++j) {
                const auto x = a.update(ride_sample(j));
                const auto y = twin.update(ride_sample(j));
                tick_check(j, x);
                bt::require(x.has_value() == y.has_value(),
                            "restored twin keeps the same tick");
                if (x.has_value() && y.has_value())
                    require_same_bits(*x, *y,
                                      "restored twin continues bitwise");
            }
            bt::require(state_json(a.state()) ==
                            state_json(twin.state()),
                        "restored twin state matches bitwise");
        }
        if (first_divergence >= 0)
            std::println(
                "perception/intent divergence beyond 300 s: first at "
                "update {} (period {})",
                first_divergence, period);
        else
            std::println("no perception/intent divergence observed "
                         "(period {})",
                         period);
    }
}

// A huge tick with an inconsistent queue rejects by the O(queue) size
// check — no clock replay can hang the reader (follow-up 2).
void test_huge_tick_rejects_promptly() {
    rider::Perception p(.15, .01);
    rider::Perception::State bad;
    bad.tick = 1000000000000000LL;   // 1e15 updates, empty queue
    bt::invalid([&] { p.restore(bad); });
    // A consistent-but-enormous state is still structurally reachable.
    rider::Perception::State huge;
    huge.tick = 1000000000000000LL;
    huge.delayed = sample(0);
    for (std::int64_t t = huge.tick - 15; t < huge.tick; ++t)
        huge.samples.emplace_back(t, sample(0));
    rider::Perception q(.15, .01);
    q.restore(huge);
    bt::require(q.state().tick == huge.tick,
                "a huge but reachable state restores");
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
        test_output_overflow_rejects_atomically();
        test_zero_delay_delivers_immediately();
        test_reset_returns_to_constructed_state();
        test_perception_state_codec_is_exact_and_bitwise();
        test_long_run_states_stay_restorable();
        test_huge_tick_rejects_promptly();
    });
}
