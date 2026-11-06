// Task 3.19: tests/reference/test_cadence_shifter.py, all nine functions.
// Expectations follow the 2026-11-06 ruling (superseding Р3's strict
// one-cog step): cadence above the band shifts to the NEAREST smaller cog
// whose landing cadence (required_ema * new/old teeth) lands inside
// [min, max] — usually the adjacent cog, further only as far as needed.
// When no cog lands inside the band the fallback is the last cog in the
// shift direction. Cooldown, cut, torque factor, band and the landing
// veto are unchanged; the oracle's band-edge throw stays superseded.
#include "drive_reference_support.hpp"

namespace {
using drive_reference::shifter;

void test_low_crank_cadence_downshifts_to_the_next_bigger_cog() {
    auto s = shifter();
    bt::require(s.update(60., 70., .01), "low crank cadence shifts");
    // The adjacent cog already lands in band: 70*33/28 = 82.5 in [75,110],
    // so the nearest in-band cog IS the next bigger one, 28 -> 33.
    bt::require(s.state().rear_teeth == 33 && s.state().direction == "down",
                "adjacent downshift lands on 33T");
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
    // Adjacent lands in band: 115*24/28 = 98.6 in [75,110] — 28 -> 24.
    bt::require(s.state().rear_teeth == 24 && s.state().direction == "up",
                "adjacent upshift lands on 24T");
}
void test_upshift_skips_only_the_cogs_that_cannot_reach_the_band() {
    auto s = shifter();
    // 150*24/28 = 128.6 and 150*21/28 = 112.5 stay above the band; the
    // nearest cog landing inside is 18T (96.4). 16T (85.7) and 14T (75.0)
    // land in band too but are further than needed.
    bt::require(s.update(115., 150., .01) && s.state().rear_teeth == 18 &&
                    s.state().direction == "up",
                "upshift skips 24T and 21T, lands on 18T");
}
void test_downshift_skips_only_the_cogs_that_cannot_reach_the_band() {
    auto s = shifter();
    // 50*33/28 = 58.9 and 50*39/28 = 69.6 stay below the band; the
    // nearest cog landing inside is 45T (80.4). 51T (91.1) lands in
    // band too but is further than needed.
    bt::require(s.update(60., 50., .01) && s.state().rear_teeth == 45 &&
                    s.state().direction == "down",
                "downshift skips 33T and 39T, lands on 45T");
}
void test_upshift_falls_back_to_the_last_cog_when_none_reaches_the_band() {
    auto s = shifter();
    // Even the smallest cog lands high: 320*10/28 = 114.3 > 110. The
    // fallback takes the last cog in the shift direction — 10T — which
    // the landing veto does not reject (it only guards the band floor
    // on upshifts).
    bt::require(s.update(115., 320., .01) && s.state().rear_teeth == 10 &&
                    s.state().direction == "up",
                "unreachable band falls back to 10T");
}
void test_downshift_falls_back_to_the_last_cog_when_none_reaches_the_band() {
    auto s = shifter();
    // Even the largest cog lands low: 40*51/28 = 72.9 < 75. The fallback
    // takes the last cog in the shift direction — 51T — which the veto
    // does not reject (it only guards the band ceiling on downshifts).
    bt::require(s.update(60., 40., .01) && s.state().rear_teeth == 51 &&
                    s.state().direction == "down",
                "unreachable band falls back to 51T");
}
void test_upshifts_stop_at_the_smallest_cog() {
    auto large = shifter(51);
    // 112*45/51 = 98.8 lands in band: the adjacent cog is the pick.
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
    bt::require(s.update(60., 70., .01), "initial downshift");
    bt::require(!s.update(60., 70., .01), "cooldown blocks second shift");
    auto gated = shifter();
    bt::require(!gated.update(60., 70., .01, false), "coasting blocks shift");
    bt::require(!gated.update(60., 70., .01, true, true), "braking blocks shift");
    bt::require(!gated.update(60., -5., .01), "rollback blocks shift");
}
void test_still_out_of_band_shifts_again_after_cooldown() {
    auto s = shifter();
    // 33T lands at 70.7 < 75: skipped; the nearest in-band cog is 39T
    // (60*39/28 = 83.6).
    bt::require(s.update(60., 60., .01) && s.state().rear_teeth == 39,
                "first downshift reaches 39T");
    // 0.4 s cooldown at dt = .01: 39 more ticks stay blocked, the 40th
    // reopens the gate and the still-low cadence shifts again — 45T
    // lands at 69.2 < 75 so the pick is 51T (78.5 in band).
    for (int i = 0; i < 39; ++i)
        bt::require(!s.update(60., 60., .01), "cooldown holds the repeat");
    bt::require(s.update(60., 60., .01) && s.state().rear_teeth == 51,
                "repeat reaches 51T");
}
void test_wheelspin_blocks_an_upshift_but_not_a_downshift() {
    auto s = shifter();
    bt::require(!s.update(115., 115., .01, true, false, true, .8), "wheelspin blocks upshift");
    bt::require(s.update(60., 60., .01, true, false, true, .8), "wheelspin allows downshift");
    bt::require(s.state().rear_teeth == 39, "allowed downshift reaches the band");
}
void test_ema_filters_a_single_stroke_spike() {
    auto s = shifter(28, .35);
    (void)s.update(90., 90., .01);
    for (int i = 0; i < 5; ++i)
        bt::require(!s.update(130., 90., .01), "EMA filters 50ms cadence spike");
}
void test_upshift_that_lands_below_the_band_is_rejected() {
    auto s = shifter();
    // No smaller cog can reach the band at wheel cadence 70 (24T lands at
    // 60, even the 10T fallback lands at 25): the fallback is vetoed.
    bt::require(!s.update(115., 70., .01), "reject below-band landing");
    bt::require(s.state().rear_teeth == 28, "rejected upshift preserves gear");
}
void test_downshift_that_lands_above_the_band_is_rejected() {
    auto s = shifter();
    // No larger cog can reach the band at wheel cadence 110 (33T lands at
    // 129.6, even the 51T fallback lands at 200): the fallback is vetoed.
    bt::require(!s.update(60., 110., .01), "reject above-band landing");
    bt::require(s.state().rear_teeth == 28, "rejected downshift preserves gear");
}
} // namespace
int main() { return bt::run([] {
    test_low_crank_cadence_downshifts_to_the_next_bigger_cog();
    test_a_wheel_implied_spike_alone_never_upshifts();
    test_high_crank_cadence_upshifts_to_the_next_smaller_cog();
    test_upshift_skips_only_the_cogs_that_cannot_reach_the_band();
    test_downshift_skips_only_the_cogs_that_cannot_reach_the_band();
    test_upshift_falls_back_to_the_last_cog_when_none_reaches_the_band();
    test_downshift_falls_back_to_the_last_cog_when_none_reaches_the_band();
    test_upshifts_stop_at_the_smallest_cog();
    test_cooldown_and_not_pedalling_block_shifts();
    test_still_out_of_band_shifts_again_after_cooldown();
    test_wheelspin_blocks_an_upshift_but_not_a_downshift();
    test_ema_filters_a_single_stroke_spike();
    test_upshift_that_lands_below_the_band_is_rejected();
    test_downshift_that_lands_above_the_band_is_rejected();
}); }
