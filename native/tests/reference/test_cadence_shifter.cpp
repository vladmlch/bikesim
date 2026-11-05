// Task 3.19: tests/reference/test_cadence_shifter.py, all nine functions.
// Expectations are one-cog steps per the Р3 ruling (plan
// 2026-11-05-clutch-pedaling-fixes, task 4): cadence above the band moves
// one cassette position harder, below the band one position easier, and
// the step repeats only after shift_cooldown_s. The multi-cog throw the
// oracle's shifting.py implemented is superseded by that ruling.
#include "drive_reference_support.hpp"

namespace {
using drive_reference::shifter;

void test_low_crank_cadence_downshifts_to_the_next_bigger_cog() {
    auto s = shifter();
    bt::require(s.update(60., 60., .01), "low crank cadence shifts");
    // One position easier: 28 -> 33 (landing 60*33/28 = 70.7 stays under
    // the band top, which is all the landing guard checks on a downshift).
    bt::require(s.state().rear_teeth == 33 && s.state().direction == "down",
                "one-cog downshift lands on 33T");
}
void test_a_wheel_implied_spike_alone_never_upshifts() {
    auto s = shifter();
    for (int i = 0; i < 50; ++i)
        bt::require(!s.update(85., 140., .01), "wheel-only cadence spike cannot trigger upshift");
    bt::require(s.state().rear_teeth == 28, "wheel spike preserves gear");
}
void test_high_crank_cadence_upshifts_to_the_next_smaller_cog() {
    auto s = shifter();
    bt::require(s.update(115., 115., .01), "high crank cadence shifts");
    // One position harder: 28 -> 24 (landing 115*24/28 = 98.6 clears the
    // band floor).
    bt::require(s.state().rear_teeth == 24 && s.state().direction == "up",
                "one-cog upshift lands on 24T");
}
void test_upshifts_stop_at_the_smallest_cog() {
    auto large = shifter(51);
    bt::require(large.update(112., 112., .01) && large.state().rear_teeth == 45,
                "51T steps one cog to 45T");
    auto small = shifter(12);
    bt::require(small.update(112., 112., .01) && small.state().rear_teeth == 10,
                "12T steps one cog to 10T");
    auto state = small.state(); state.cooldown_s = 0.; small.set_state(state);
    bt::require(!small.update(130., 130., .01), "smallest cog stops upshifts");
}
void test_cooldown_and_not_pedalling_block_shifts() {
    auto s = shifter();
    bt::require(s.update(60., 60., .01), "initial downshift");
    bt::require(!s.update(60., 60., .01), "cooldown blocks second shift");
    auto gated = shifter();
    bt::require(!gated.update(60., 60., .01, false), "coasting blocks shift");
    bt::require(!gated.update(60., 60., .01, true, true), "braking blocks shift");
    bt::require(!gated.update(60., -5., .01), "rollback blocks shift");
}
void test_still_out_of_band_repeats_one_cog_after_cooldown() {
    auto s = shifter();
    bt::require(s.update(60., 60., .01) && s.state().rear_teeth == 33,
                "first downshift steps 28 -> 33");
    // 0.4 s cooldown at dt = .01: 39 more ticks stay blocked, the 40th
    // reopens the gate and the still-low cadence steps exactly one more
    // cog — not a re-throw to the band edge.
    for (int i = 0; i < 39; ++i)
        bt::require(!s.update(60., 60., .01), "cooldown holds the repeat");
    bt::require(s.update(60., 60., .01) && s.state().rear_teeth == 39,
                "repeat steps one more cog 33 -> 39");
}
void test_wheelspin_blocks_an_upshift_but_not_a_downshift() {
    auto s = shifter();
    bt::require(!s.update(115., 115., .01, true, false, true, .8), "wheelspin blocks upshift");
    bt::require(s.update(60., 60., .01, true, false, true, .8), "wheelspin allows downshift");
    bt::require(s.state().rear_teeth == 33, "allowed downshift steps one cog");
}
void test_ema_filters_a_single_stroke_spike() {
    auto s = shifter(28, .35);
    (void)s.update(90., 90., .01);
    for (int i = 0; i < 5; ++i)
        bt::require(!s.update(130., 90., .01), "EMA filters 50ms cadence spike");
}
void test_upshift_that_lands_below_the_band_is_rejected() {
    auto s = shifter();
    // The one-cog step 28 -> 24 lands at 70*24/28 = 60 < 75: rejected.
    bt::require(!s.update(115., 70., .01), "reject below-band landing");
    bt::require(s.state().rear_teeth == 28, "rejected upshift preserves gear");
}
void test_downshift_that_lands_above_the_band_is_rejected() {
    auto s = shifter();
    // The one-cog step 28 -> 33 lands at 110*33/28 = 129.6 > 110: rejected.
    bt::require(!s.update(60., 110., .01), "reject above-band landing");
    bt::require(s.state().rear_teeth == 28, "rejected downshift preserves gear");
}
} // namespace
int main() { return bt::run([] {
    test_low_crank_cadence_downshifts_to_the_next_bigger_cog();
    test_a_wheel_implied_spike_alone_never_upshifts();
    test_high_crank_cadence_upshifts_to_the_next_smaller_cog();
    test_upshifts_stop_at_the_smallest_cog();
    test_cooldown_and_not_pedalling_block_shifts();
    test_still_out_of_band_repeats_one_cog_after_cooldown();
    test_wheelspin_blocks_an_upshift_but_not_a_downshift();
    test_ema_filters_a_single_stroke_spike();
    test_upshift_that_lands_below_the_band_is_rejected();
    test_downshift_that_lands_above_the_band_is_rejected();
}); }
