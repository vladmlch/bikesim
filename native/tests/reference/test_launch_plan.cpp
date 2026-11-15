// Launch part 3b — the launch plan node on synthetic percepts: hold and
// press, release by the felt torque, the hold timeout, the caller brake
// freeze, the upshift protocol against the shifter's answers, the handover
// that waits for the cut and blends, restart after a stop, eligibility,
// the switch, the codec round trip with bitwise continuation and the
// rejected restores.
#include "support/fixture_test.hpp"
#include "io/json.hpp"
#include "rider/launch_plan.hpp"
#include "rider/senses_codec.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {

constexpr double kPeriod = .01;

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

// Effort 100 N·m/s (1 N·m per tick), caps 60 except loose 20; release at
// 10 N·m on the flat, 30 N·m at 20 %; release over 4 ticks; hold limit 50
// ticks; upshift at 80 rpm; end 3 m/s and 60 rpm for more than 5 ticks;
// launch limit 300 ticks; handover over 10 ticks.
rider::LaunchConfig table() {
    rider::LaunchConfig t;
    t.start_cog_grade_breaks = {};
    t.start_cog_teeth = {51};
    t.start_pedal_angle_rad = 1.2;
    t.start_lean_grades = {0., .2};
    t.start_lean_rad = {0., .2};
    t.plan_enabled = true;
    t.standstill_speed_mps = .1;
    t.standstill_crank_rpm = 5.;
    t.effort_rate_nm_s = 100.;
    t.effort_cap_asphalt_nm = 60.;
    t.effort_cap_hardpack_nm = 60.;
    t.effort_cap_loose_nm = 20.;
    t.effort_cap_wet_nm = 60.;
    t.effort_cap_scree_nm = 60.;
    // Launch part 4: a grade cap that never binds below the 60 N·m surface
    // caps and a rise speed no helper reaches keep the 3b tests exact.
    t.effort_cap_grades = {0.};
    t.effort_cap_grade_nm = {1000.};
    t.effort_rise_speed_mps = 100.;
    t.release_torque_grades = {0., .2};
    t.release_torque_nm = {10., 30.};
    t.brake_release_s = 4 * kPeriod;
    t.max_hold_s = 50 * kPeriod;
    t.upshift_crank_rpm = 80.;
    t.shift_retry_s = 10 * kPeriod;
    t.end_speed_mps = 3.;
    t.end_crank_rpm = 60.;
    t.end_hold_s = 5 * kPeriod;
    t.max_launch_s = 300 * kPeriod;
    t.handover_s = 10 * kPeriod;
    return t;
}

rider::Percepts standing(double grade = 0.) {
    rider::Percepts p;
    p.senses.speed_mps = 0.;
    p.senses.rear_teeth = 51;
    p.senses.surface_front = rider::SurfaceLabel::hardpack;
    p.senses.surface_ahead = rider::SurfaceLabel::hardpack;
    p.senses.grade_ahead = grade;
    return p;
}

rider::LaunchInputs in(std::optional<rider::Percepts> p, rider::ShiftFeedback shift = {}) {
    return {.percepts = p, .shift = shift, .eligible = true, .external_brake = false};
}

std::string text(const rider::LaunchPlan &plan) {
    return bike::io::canonical_json(rider::encode_launch_state(plan.state()));
}

// Drives a plan from a standstill into accelerate: one standing percept
// arms the hold, the felt torque reaches the release value, the brake
// leaves over four ticks.
rider::LaunchPlan accelerating() {
    rider::LaunchPlan plan(table(), kPeriod);
    auto p = standing();
    (void)plan.update(in(p));
    p.human_crank_torque_nm = 10.;
    (void)plan.update(in(p));
    for (int i = 0; i < 4; ++i) (void)plan.update(in(p));
    bt::require(plan.command().phase == rider::LaunchPhase::accelerate, "the helper reaches accelerate");
    return plan;
}

// Hold: brake 1, the plan owns effort and shifting, effort rises one tick
// at a time to the cap of the surface under the front wheel.
void test_hold_brakes_and_ramps_to_the_surface_cap() {
    rider::LaunchPlan plan(table(), kPeriod);
    auto cmd = plan.update(in(standing(.1)));
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.brake == 1. && cmd.weight == 1. &&
                    cmd.owns_shifting && !cmd.shift_up && cmd.effort_nm == 0.,
                "a standing percept arms the hold");
    bt::require(std::abs(cmd.lean_target_rad - .1) < 1e-15, "the hold leans by the felt grade");
    auto p = standing(.1);
    for (int i = 0; i < 5; ++i) cmd = plan.update(in(p));
    bt::require(std::abs(cmd.effort_nm - 5.) < 1e-12, "effort rises by rate * period per tick");
    p.senses.surface_front = rider::SurfaceLabel::loose;
    for (int i = 0; i < 30; ++i) cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 20. && cmd.phase == rider::LaunchPhase::hold,
                "the loose cap binds while the felt torque stays below the release value");
}

// Before the first percept the plan holds without pressing and its clock
// stands; the caller's brake freezes the hold the same way.
void test_no_percept_and_caller_brake_freeze_the_hold() {
    rider::LaunchPlan plan(table(), kPeriod);
    bt::require(plan.arm_start(.05), "a switched-on plan arms at the start");
    for (int i = 0; i < 100; ++i) {
        const auto cmd = plan.update(in(std::nullopt));
        bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.brake == 1. && cmd.effort_nm == 0. &&
                        cmd.lean_target_rad == .05,
                    "without percepts the rider holds and waits");
    }
    bt::require(plan.state().phase_ticks == 0, "the hold clock does not run without percepts");
    auto braked = in(standing());
    braked.external_brake = true;
    for (int i = 0; i < 100; ++i) (void)plan.update(braked);
    bt::require(plan.state().phase_ticks == 0 && plan.command().effort_nm == 0. && !plan.command().preload_missed,
                "the caller brake freezes the hold");
}

// The release starts on the felt crank torque, not on the plan's own
// effort, and the brake leaves linearly; the timeout releases with an event.
void test_release_by_felt_torque_and_by_timeout() {
    rider::LaunchPlan plan(table(), kPeriod);
    auto p = standing();
    for (int i = 0; i < 20; ++i) (void)plan.update(in(p));
    bt::require(plan.command().phase == rider::LaunchPhase::hold && plan.command().effort_nm >= 19.,
                "a large plan effort alone does not release");
    p.human_crank_torque_nm = 10.;
    auto cmd = plan.update(in(p));
    bt::require(cmd.phase == rider::LaunchPhase::release && cmd.brake == 1. && !cmd.preload_missed,
                "the felt torque starts the release");
    double previous = 1.;
    for (int i = 1; i <= 4; ++i) {
        cmd = plan.update(in(p));
        bt::require(cmd.brake < previous && std::abs(cmd.brake - (1. - i / 4.)) < 1e-15,
                    "the brake leaves linearly over brake_release_s");
        previous = cmd.brake;
    }
    bt::require(cmd.phase == rider::LaunchPhase::accelerate && cmd.brake == 0., "accelerate at brake 0");
    rider::LaunchPlan weak(table(), kPeriod);
    for (int i = 0; i <= 50; ++i) cmd = weak.update(in(standing(.2)));
    bt::require(cmd.phase == rider::LaunchPhase::release && cmd.preload_missed,
                "the hold limit releases with preload_missed");
}

// Upshift protocol: request at the table cadence, kept while deferred,
// retired by the shift count, then blocked until the felt cog changes;
// a rejection retires and blocks at that cog; no request during a cut.
void test_upshift_request_follows_the_shifter() {
    auto plan = accelerating();
    auto p = standing();
    p.senses.speed_mps = 1.;
    p.crank_rpm = 85.;
    rider::ShiftFeedback fb{.response = rider::ShiftResponse::none, .shift_count = 3, .cut_active = false};
    auto cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up && cmd.owns_shifting, "the table cadence raises a request");
    fb.response = rider::ShiftResponse::deferred;
    cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up, "a deferred request is kept");
    fb = {.response = rider::ShiftResponse::deferred, .shift_count = 4, .cut_active = true};
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "a committed shift retires the request even under a deferred label");
    fb.cut_active = false;
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "the delayed cadence of the old cog asks nothing until the felt cog changes");
    p.senses.rear_teeth = 45;
    cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up, "the new felt cog unblocks the next request");
    fb.response = rider::ShiftResponse::rejected;
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "a rejection retires the request");
    fb.response = rider::ShiftResponse::none;
    for (int i = 0; i < 5; ++i) cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "a rejected cog is not asked again");
    p.senses.rear_teeth = 39;
    fb.cut_active = true;
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "no request while a cut runs");
}

// Launch part 4: the grade cap — piecewise linear over the knots, held at
// the end values, the lower of it and the surface cap binds the ramp.
void test_grade_cap_binds_below_the_surface_cap() {
    auto t = table();
    t.effort_cap_grades = {0., .1, .2};
    t.effort_cap_grade_nm = {30., 10., 20.};
    bt::require(rider::effort_grade_cap_nm(t, -.05) == 30. && rider::effort_grade_cap_nm(t, .3) == 20.,
                "the grade cap holds its end values outside the knots");
    bt::require(std::abs(rider::effort_grade_cap_nm(t, .05) - 20.) < 1e-12 &&
                    std::abs(rider::effort_grade_cap_nm(t, .15) - 15.) < 1e-12,
                "the grade cap is linear between knots, also where it rises again");
    rider::LaunchPlan plan(t, kPeriod);
    auto p = standing(.1);
    rider::LaunchCommand cmd;
    for (int i = 0; i < 40; ++i) cmd = plan.update(in(p));
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.effort_nm == 10.,
                "the 10 % grade cap binds below the 60 N·m surface cap");
    // A fresh plan: the 50-tick hold limit would end the first hold.
    rider::LaunchPlan loose(t, kPeriod);
    auto q = standing(0.);
    q.senses.surface_front = rider::SurfaceLabel::loose;
    for (int i = 0; i < 30; ++i) cmd = loose.update(in(q));
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.effort_nm == 20.,
                "the 20 N·m loose cap binds below the 30 N·m flat grade cap");
}

// Launch part 4: once the felt speed reaches the rise speed the ceiling
// rises from the grade cap at effort_rate_nm_s per second to the surface
// cap; below the rise speed it is the grade cap again while the rise
// clock keeps running; a new start resets the clock.
void test_cap_rises_after_the_rise_speed() {
    auto t = table();
    t.effort_cap_grades = {0.};
    t.effort_cap_grade_nm = {10.};
    t.effort_rise_speed_mps = 2.;
    rider::LaunchPlan plan(t, kPeriod);
    auto p = standing();
    (void)plan.update(in(p));
    p.human_crank_torque_nm = 10.;
    for (int i = 0; i < 5; ++i) (void)plan.update(in(p));
    bt::require(plan.command().phase == rider::LaunchPhase::accelerate && plan.command().effort_nm == 5.,
                "accelerate starts below the 10 N·m grade cap");
    p.senses.speed_mps = 1.;
    rider::LaunchCommand cmd;
    for (int i = 0; i < 10; ++i) cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 10. && plan.state().rise_ticks == -1, "below the rise speed the grade cap binds");
    p.senses.speed_mps = 2.;
    cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 10. && plan.state().rise_ticks == 0, "the rise starts at the grade cap");
    for (int i = 0; i < 5; ++i) cmd = plan.update(in(p));
    bt::require(std::abs(cmd.effort_nm - 15.) < 1e-12 && plan.state().rise_ticks == 5,
                "the ceiling rises one rate step per tick");
    p.senses.speed_mps = 1.;
    cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 10. && plan.state().rise_ticks == 6,
                "below the rise speed the ceiling is the grade cap again; the clock runs on");
    p.senses.speed_mps = 2.;
    for (int i = 0; i < 100; ++i) cmd = plan.update(in(p));
    bt::require(cmd.effort_nm == 60., "the rise ends at the surface cap");
}

// Launch part 4: a rejected request repeats on the same cog after
// shift_retry_s; a committed shift keeps the old cog blocked until the
// felt cog changes.
void test_rejected_request_repeats_after_the_retry_time() {
    auto plan = accelerating();
    auto p = standing();
    p.senses.speed_mps = 1.;
    p.crank_rpm = 85.;
    rider::ShiftFeedback fb{.response = rider::ShiftResponse::none, .shift_count = 3, .cut_active = false};
    auto cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up, "the table cadence raises a request");
    fb.response = rider::ShiftResponse::rejected;
    cmd = plan.update(in(p, fb));
    const std::int64_t rejected_at = plan.state().launch_ticks;
    bt::require(!cmd.shift_up && plan.state().retry_tick == rejected_at + 10,
                "a rejection retires the request and sets the retry tick");
    fb.response = rider::ShiftResponse::none;
    for (int i = 0; i < 9; ++i) {
        cmd = plan.update(in(p, fb));
        bt::require(!cmd.shift_up, "the same cog waits for the retry time");
    }
    cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up && plan.state().retry_tick == -1 && plan.state().launch_ticks == rejected_at + 10,
                "the same cog is asked again at the retry tick");
    fb.shift_count = 4;
    cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up && plan.state().retry_tick == -1, "a committed shift sets no retry");
    for (int i = 0; i < 20; ++i) cmd = plan.update(in(p, fb));
    bt::require(!cmd.shift_up, "after a committed shift the old cog stays blocked");
}

// Handover: the request is retired at entry, shifting stays with the plan
// until the cut ends, then the weight falls linearly to 0 and the plan is done.
void test_handover_waits_for_the_cut_then_blends() {
    auto plan = accelerating();
    auto p = standing();
    p.senses.speed_mps = 3.5;
    p.crank_rpm = 85.;
    rider::ShiftFeedback fb{.response = rider::ShiftResponse::none, .shift_count = 0, .cut_active = false};
    auto cmd = plan.update(in(p, fb));
    bt::require(cmd.shift_up, "a request is pending before the end criteria hold");
    fb = {.response = rider::ShiftResponse::accepted, .shift_count = 1, .cut_active = true};
    for (int i = 0; i < 6; ++i) cmd = plan.update(in(p, fb));
    bt::require(cmd.phase == rider::LaunchPhase::handover && !cmd.shift_up && cmd.owns_shifting &&
                    cmd.weight == 1.,
                "the end criteria start the handover, which keeps shifting during the cut");
    fb.cut_active = false;
    cmd = plan.update(in(p, fb));
    bt::require(cmd.phase == rider::LaunchPhase::blend && !cmd.owns_shifting && cmd.weight == 1.,
                "the end of the cut passes shifting to the automaton");
    double previous = 1.;
    for (int i = 1; i <= 10; ++i) {
        cmd = plan.update(in(p, fb));
        bt::require(cmd.weight < previous && std::abs(cmd.weight - (1. - i / 10.)) < 1e-15,
                    "the weight falls linearly over handover_s");
        previous = cmd.weight;
    }
    bt::require(cmd.phase == rider::LaunchPhase::done && cmd.weight == 0. && cmd.brake == 0., "done");
}

// A stop after the handover re-arms the plan; a bike rolling backward
// fast does not count as standing.
void test_restart_after_a_stop_and_backward_roll() {
    auto plan = accelerating();
    auto p = standing();
    p.senses.speed_mps = 3.5;
    p.crank_rpm = 70.;
    for (int i = 0; i < 30; ++i) (void)plan.update(in(p));
    bt::require(plan.command().phase == rider::LaunchPhase::done, "the plan finishes");
    auto rolling = standing();
    rolling.senses.speed_mps = -.5;
    bt::require(plan.update(in(rolling)).phase == rider::LaunchPhase::done, "rolling back is not standing");
    const auto cmd = plan.update(in(standing(.2)));
    bt::require(cmd.phase == rider::LaunchPhase::hold && cmd.brake == 1. && std::abs(cmd.lean_target_rad - .2) < 1e-15,
                "a stop re-arms with the lean of the felt grade");
}

// An ineligible tick cancels the plan from any phase; a switched-off plan
// never leaves idle and does not arm at the start.
void test_ineligible_ticks_cancel_and_the_switch_keeps_idle() {
    auto plan = accelerating();
    auto off = in(standing());
    off.eligible = false;
    auto cmd = plan.update(off);
    bt::require(cmd.phase == rider::LaunchPhase::idle && cmd.brake == 0. && cmd.weight == 0. && !cmd.owns_shifting,
                "an ineligible tick cancels to idle");
    bt::require(plan.state() == rider::LaunchPlan::State{}, "the cancelled state is the initial one");
    auto t = table();
    t.plan_enabled = false;
    rider::LaunchPlan disabled(t, kPeriod);
    bt::require(!disabled.arm_start(.1), "a switched-off plan does not arm");
    for (int i = 0; i < 20; ++i) cmd = disabled.update(in(standing()));
    bt::require(cmd.phase == rider::LaunchPhase::idle && cmd.brake == 0. && cmd.weight == 0.,
                "a switched-off plan stays idle");
}

// The state document round-trips mid-hold and mid-blend, and a restored
// copy continues bitwise; rejected states leave the plan unchanged.
void test_codec_round_trip_continuation_and_rejected_restores() {
    rider::LaunchPlan hold(table(), kPeriod);
    for (int i = 0; i < 7; ++i) (void)hold.update(in(standing()));
    auto blend = accelerating();
    auto p = standing();
    p.senses.speed_mps = 3.5;
    p.crank_rpm = 70.;
    for (int i = 0; i < 10; ++i) (void)blend.update(in(p));
    bt::require(blend.command().phase == rider::LaunchPhase::blend, "the second plan is blending");
    for (rider::LaunchPlan *source : {&hold, &blend}) {
        const auto doc = rider::encode_launch_state(source->state());
        rider::LaunchPlan copy(table(), kPeriod);
        copy.restore(rider::decode_launch_state(doc, "launch"));
        bt::require(text(copy) == text(*source), "the document restores the state");
        for (int i = 0; i < 5; ++i) {
            const auto a = source->update(in(p));
            const auto b = copy.update(in(p));
            bt::require(a.phase == b.phase && a.weight == b.weight && a.effort_nm == b.effort_nm,
                        "the restored plan continues bitwise");
        }
    }
    rider::LaunchPlan fresh(table(), kPeriod);
    const std::string before = text(fresh);
    auto bad = hold.state();
    bad.brake = .5;
    rejects([&] { fresh.restore(bad); }, "brake", "a half brake while holding");
    bad = hold.state();
    bad.phase_ticks = 50;
    rejects([&] { fresh.restore(bad); }, "phase_ticks", "a hold past its limit");
    bad = hold.state();
    bad.shift_up = true;
    rejects([&] { fresh.restore(bad); }, "shift_up", "a request outside accelerate");
    bad = rider::LaunchPlan::State{};
    bad.effort_nm = 1.;
    rejects([&] { fresh.restore(bad); }, "idle", "an idle plan that is not reset");
    bad = hold.state();
    bad.rise_ticks = -2;
    rejects([&] { fresh.restore(bad); }, "rise_ticks", "a rise clock below -1");
    bad = hold.state();
    bad.retry_tick = 5;
    rejects([&] { fresh.restore(bad); }, "retry_tick", "a retry tick in the hold");
    bt::require(text(fresh) == before, "rejected restores change nothing");
    // A complete document with an unknown phase: the exact-key check passes,
    // so the phase name is what fails (review 1, M6).
    auto fields = bike::io::as_object(rider::encode_launch_state(hold.state()), "launch");
    for (auto &[key, value] : fields)
        if (key == "phase") value = bike::io::Value{std::string("soaring")};
    const bike::io::Value doc{std::move(fields)};
    rejects([&] { (void)rider::decode_launch_state(doc, "launch"); }, "unknown launch phase", "an unknown phase name");
}

// Nonfinite percepts and a negative shift count are rejected before any change.
void test_inputs_are_validated() {
    rider::LaunchPlan plan(table(), kPeriod);
    (void)plan.update(in(standing()));
    const std::string before = text(plan);
    auto p = standing();
    p.human_crank_torque_nm = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { (void)plan.update(in(p)); }, "launch plan percepts", "a NaN felt torque");
    rider::ShiftFeedback fb{.response = rider::ShiftResponse::none, .shift_count = -1, .cut_active = false};
    rejects([&] { (void)plan.update(in(standing(), fb)); }, "shift count", "a negative shift count");
    bt::require(text(plan) == before, "rejected inputs change nothing");
    rejects([&] { (void)rider::shift_response("maybe"); }, "rider_response", "an unknown shifter label");
}

// A restored rise counter at the int64 top stays inside the declared
// domain, but the next update must refuse to count it — checked, not
// wrapping — and must leave the plan untouched. The retry deadline
// comparison in restore must not overflow on a huge launch clock either.
void test_counter_domains_are_checked() {
    rider::LaunchPlan plan(table(), kPeriod);
    for (int i = 0; i < 3; ++i) (void)plan.update(in(standing()));
    auto state = plan.state();
    state.rise_ticks = std::numeric_limits<std::int64_t>::max();
    plan.restore(state);
    const std::string top = text(plan);
    rejects([&] { (void)plan.update(in(standing())); }, "rise",
            "the rise clock cannot be counted past the int64 top");
    bt::require(text(plan) == top, "the refused tick changes nothing");
    // One tick below the top still counts.
    state.rise_ticks = std::numeric_limits<std::int64_t>::max() - 1;
    plan.restore(state);
    (void)plan.update(in(standing()));
    bt::require(plan.state().rise_ticks == std::numeric_limits<std::int64_t>::max(),
                "the tick below the top counts once");
    // A retry deadline after an enormous launch clock compares, not wraps.
    rider::LaunchPlan::State handover;
    handover.phase = rider::LaunchPhase::handover;
    handover.launch_ticks = std::numeric_limits<std::int64_t>::max();
    handover.blocked_teeth = 51;
    handover.retry_tick = 310;
    plan.restore(handover);
    bt::require(plan.state().retry_tick == 310,
                "a retry inside its deadline restores without wrapping");
    handover.retry_tick = std::numeric_limits<std::int64_t>::max();
    plan.restore(handover);
    bt::require(plan.state().retry_tick == std::numeric_limits<std::int64_t>::max(),
                "the deadline top itself restores without wrapping");
    handover.retry_tick = 5;   // below retry_ticks_
    rejects([&] { plan.restore(handover); }, "retry_tick",
            "a retry before its deadline stays rejected");
}

} // namespace

int main() {
    return bt::run([] {
        test_hold_brakes_and_ramps_to_the_surface_cap();
        test_no_percept_and_caller_brake_freeze_the_hold();
        test_release_by_felt_torque_and_by_timeout();
        test_upshift_request_follows_the_shifter();
        test_grade_cap_binds_below_the_surface_cap();
        test_cap_rises_after_the_rise_speed();
        test_rejected_request_repeats_after_the_retry_time();
        test_handover_waits_for_the_cut_then_blends();
        test_restart_after_a_stop_and_backward_roll();
        test_ineligible_ticks_cancel_and_the_switch_keeps_idle();
        test_codec_round_trip_continuation_and_rejected_restores();
        test_inputs_are_validated();
        test_counter_domains_are_checked();
    });
}
