// Task 3.19: tests/reference/test_cadence_shifter.py, all nine functions.
// Expectations follow the 2026-11-06 ruling (superseding Р3's strict
// one-cog step): cadence above the band shifts to the NEAREST smaller cog
// whose landing cadence (required_ema * new/old teeth) lands inside
// [min, max] — usually the adjacent cog, further only as far as needed.
// When no cog lands inside the band the fallback is the last cog in the
// shift direction. Cooldown, cut, torque factor, band and the landing
// veto are unchanged; the oracle's band-edge throw stays superseded.
// Launch part 1, task 5 adds the rider-owned request path: one admission
// check and one commit shared with the automaton, with the rider_response
// label reporting accepted/deferred/rejected.
#include "drive_reference_support.hpp"
#include "drivetrain_reference_support.hpp"

#include <limits>

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

// ---- Launch part 1, task 5: the rider-owned request --------------------
// Rider-owned: the band never selects; an up request moves one cog with
// the usual cooldown and cut.
void test_rider_up_request_moves_one_cog_with_cooldown_and_cut() {
    auto s = shifter();   // 28T
    const drivetrain::RiderShift owns{.owns = true};
    bt::require(!s.update(130., 130., .01, true, false, true, std::nullopt,
                          owns),
                "no band shift while the rider owns");
    bt::require(s.update(90., 90., .01, true, false, true, std::nullopt,
                         {.owns = true,
                          .request = drivetrain::ShiftDirection::up}),
                "the up request shifts");
    bt::require(s.state().rear_teeth == 24 &&
                    s.state().rider_response == "accepted" &&
                    s.state().cut_remaining_s > 0. &&
                    s.state().cooldown_s > 0.,
                "one cog smaller, cut and cooldown armed");
}
// Inside the cooldown or cut the request is deferred.
void test_rider_request_inside_cooldown_is_deferred() {
    auto s = shifter();
    (void)s.update(90., 90., .01, true, false, true, std::nullopt,
                   {.owns = true, .request = drivetrain::ShiftDirection::up});
    bt::require(!s.update(90., 90., .01, true, false, true, std::nullopt,
                          {.owns = true,
                           .request = drivetrain::ShiftDirection::up}),
                "no second shift inside the cooldown");
    bt::require(s.state().rider_response == "deferred" &&
                    s.state().rear_teeth == 24,
                "deferred, gear kept");
}
// The cassette end rejects.
void test_rider_request_past_the_cassette_end_is_rejected() {
    auto s = shifter(51);
    bt::require(!s.update(60., 60., .01, true, false, true, std::nullopt,
                          {.owns = true,
                           .request = drivetrain::ShiftDirection::down}),
                "no cog larger than 51");
    bt::require(s.state().rider_response == "rejected" &&
                    s.state().rear_teeth == 51,
                "rejected, gear kept");
}
// The existing vetoes stay: no pedaling, braking, no contact and reverse
// cadence defer.
void test_rider_request_keeps_the_admission_vetoes() {
    const drivetrain::RiderShift up{.owns = true,
                                    .request = drivetrain::ShiftDirection::up};
    {
        auto s = shifter();
        bt::require(!s.update(90., 90., .01, false, false, true,
                              std::nullopt, up),
                    "not pedaling defers");
        bt::require(s.state().rider_response == "deferred" &&
                        s.state().rear_teeth == 28,
                    "deferred, gear kept");
    }
    {
        auto s = shifter();
        bt::require(!s.update(90., 90., .01, true, true, true, std::nullopt,
                              up),
                    "braking defers");
        bt::require(s.state().rider_response == "deferred" &&
                        s.state().rear_teeth == 28,
                    "deferred, gear kept");
    }
    {
        auto s = shifter();
        bt::require(!s.update(90., 90., .01, true, false, false,
                              std::nullopt, up),
                    "no contact defers");
        bt::require(s.state().rider_response == "deferred" &&
                        s.state().rear_teeth == 28,
                    "deferred, gear kept");
    }
    {
        auto s = shifter();
        bt::require(!s.update(-10., 90., .01, true, false, true,
                              std::nullopt, up),
                    "reverse cadence defers");
        bt::require(s.state().rider_response == "deferred" &&
                        s.state().rear_teeth == 28,
                    "deferred, gear kept");
    }
}
// Slip and landing vetoes reject with the existing numbers.
void test_rider_request_keeps_the_slip_and_landing_vetoes() {
    {
        auto s = shifter();
        bt::require(!s.update(90., 90., .01, true, false, true, 1.0,
                              {.owns = true,
                               .request = drivetrain::ShiftDirection::up}),
                    "slip above .5 m/s vetoes an upshift");
        bt::require(s.state().rider_response == "rejected",
                    "slip rejection");
    }
    {
        auto s = shifter();
        // landing 70 * 24/28 = 60 < 75: the upshift would land below the
        // band.
        bt::require(!s.update(70., 70., .01, true, false, true,
                              std::nullopt,
                              {.owns = true,
                               .request = drivetrain::ShiftDirection::up}),
                    "landing below the band vetoes");
        bt::require(s.state().rider_response == "rejected",
                    "landing rejection");
    }
    {
        auto s = shifter();
        // Down request landing above the band: 110*33/28 = 129.6 > 110.
        bt::require(!s.update(60., 110., .01, true, false, true,
                              std::nullopt,
                              {.owns = true,
                               .request = drivetrain::ShiftDirection::down}),
                    "downshift landing above the band vetoes");
        bt::require(s.state().rider_response == "rejected" &&
                        s.state().rear_teeth == 28,
                    "downshift landing rejection keeps the gear");
    }
}
// Not owned: behaviour and response are exactly as before.
void test_automatic_path_reports_no_rider_response() {
    auto s = shifter();
    bt::require(s.update(115., 115., .01), "automatic upshift as before");
    bt::require(s.state().rider_response == "none",
                "no rider response on the automatic path");
    // The default RiderShift{} overload is the same call: the band keeps
    // committing through the cooldown cycle and every tick reports none.
    int shifts = 0;
    for (int i = 0; i < 90; ++i)
        if (s.update(115., 115., .01)) {
            ++shifts;
            bt::require(s.state().rider_response == "none",
                        "the old overload stays a pure automaton");
        }
    bt::require(shifts >= 2, "the automatic path keeps shifting");
}
// Magnitude slip mode: a signed-negative slip magnitude still vetoes the
// upshift — the mode resolution is shared with the automaton's.
void test_rider_request_magnitude_slip_mode_vetoes_negative_slip() {
    drivetrain::ShiftingConfig config{
        .enabled = true,
        .cassette = {10, 12, 14, 16, 18, 21, 24, 28, 33, 39, 45, 51},
        .target_cadence_min_rpm = 75., .target_cadence_max_rpm = 110.,
        .shift_cooldown_s = .4, .shift_cut_duration_s = .2,
        .torque_factor_larger_cog = .8, .torque_factor_smaller_cog = .3,
        .cadence_smoothing_tau_s = 0.,
        .upshift_slip_limit_mps = .5, .upshift_slip_mode = "magnitude"};
    drivetrain::CadenceShifter s(
        {.front_teeth = 22, .rear_teeth = 28, .chain_pitch_m = .0127},
        std::move(config));
    bt::require(!s.update(90., 90., .01, true, false, true, -1.0,
                          {.owns = true,
                           .request = drivetrain::ShiftDirection::up}),
                "magnitude mode turns -1 m/s into a veto");
    bt::require(s.state().rider_response == "rejected" &&
                    s.state().rear_teeth == 28,
                "magnitude slip rejection keeps the gear");
    auto legacy = shifter();   // legacy_signed keeps the signed value
    bt::require(legacy.update(90., 90., .01, true, false, true, -1.0,
                              {.owns = true,
                               .request = drivetrain::ShiftDirection::up}),
                "legacy_signed admits a negative slip");
    bt::require(legacy.state().rider_response == "accepted",
                "the control request accepts");
}
// A request inside the torque cut defers; the serialized mid-cut state
// restores and continues the cut on the next ticks.
void test_mid_cut_request_defers_and_serialized_state_continues() {
    auto s = shifter();
    bt::require(s.update(90., 90., .01, true, false, true, std::nullopt,
                         {.owns = true,
                          .request = drivetrain::ShiftDirection::up}),
                "first request accepted");
    auto restored = shifter();
    restored.set_state(s.state());
    bt::require(restored.state().cut_remaining_s > 0. &&
                    restored.state().cooldown_s > 0. &&
                    restored.state().rider_response == "accepted",
                "the mid-cut snapshot restores verbatim");
    const double cut = restored.state().cut_remaining_s;
    bt::require(!restored.update(90., 90., .01, true, false, true,
                                 std::nullopt,
                                 {.owns = true,
                                  .request = drivetrain::ShiftDirection::up}),
                "a request inside the cut defers");
    bt::require(restored.state().rider_response == "deferred" &&
                    restored.state().cut_remaining_s < cut &&
                    restored.state().cut_remaining_s > 0.,
                "the restored cut keeps running down");
}
// The shared commit still throws when the counter saturates — the rider
// branch cannot bypass the overflow guard.
void test_rider_request_keeps_the_count_overflow_guard() {
    auto s = shifter();
    auto state = s.state();
    state.shift_count = std::numeric_limits<int>::max();
    s.set_state(state);
    drivetrain_reference::error<std::overflow_error>([&] {
        (void)s.update(90., 90., .01, true, false, true, std::nullopt,
                       {.owns = true,
                        .request = drivetrain::ShiftDirection::up});
    });
}
// A restored state off the cassette is an internal-invariant failure, not
// a request rejection.
void test_rider_request_on_an_off_cassette_state_is_internal_error() {
    auto s = shifter();
    auto state = s.state();
    state.rear_teeth = 25;   // not on the cassette
    s.set_state(state);
    drivetrain_reference::error<std::logic_error>([&] {
        (void)s.update(60., 60., .01, true, false, true, std::nullopt,
                       {.owns = true,
                        .request = drivetrain::ShiftDirection::down});
    });
}
// Request/ownership consistency and the closed response domain are
// validated, not silently dropped.
void test_rider_shift_contract_validation() {
    {
        auto s = shifter();
        drivetrain_reference::error<std::invalid_argument>([&] {
            (void)s.update(90., 90., .01, true, false, true, std::nullopt,
                           {.owns = true,
                            .request = drivetrain::ShiftDirection::none});
        }, "rider shift");
        drivetrain_reference::error<std::invalid_argument>([&] {
            (void)s.update(90., 90., .01, true, false, true, std::nullopt,
                           {.owns = false,
                            .request = drivetrain::ShiftDirection::up});
        }, "rider shift");
        // Rejections are atomic: a valid control still shifts.
        bt::require(s.update(90., 90., .01, true, false, true, std::nullopt,
                             {.owns = true,
                              .request = drivetrain::ShiftDirection::up}),
                    "the valid control shifts after the rejections");
    }
    {
        auto s = shifter();
        auto state = s.state();
        state.rider_response = "bogus";
        drivetrain_reference::error<std::invalid_argument>([&] {
            s.set_state(state);
        }, "rider_response");
    }
}
// Writer level: a non-advancing prepare never consumes the request — the
// staged shifter tick is gated on advance, and the probe publishes the
// committed snapshot unchanged.
void test_probe_prepare_does_not_consume_a_rider_request() {
    auto source = drivetrain_reference::source("ideal_mid_drive");
    source.shifting.enabled = true;
    source.shifting.cadence_smoothing_tau_s = 0.;
    const auto owner = drivetrain_reference::pair(source);
    const drivetrain::RiderShift up{.owns = true,
                                    .request = drivetrain::ShiftDirection::up};
    const auto before = owner->drive().state().shifting;
    (void)owner->drive().prepare({}, .01, false, true, false, true,
                                 std::nullopt, std::nullopt, std::nullopt,
                                 up);
    const auto &probed = owner->drive().state().shifting;
    bt::require(probed.rear_teeth == before.rear_teeth &&
                    probed.shift_count == before.shift_count &&
                    probed.direction == before.direction &&
                    probed.rider_response == "none",
                "a probe tick publishes the committed shifter state");
    (void)owner->drive().prepare({}, .01, false, true, true, true,
                                 std::nullopt, std::nullopt, std::nullopt,
                                 up);
    const auto &after = owner->drive().state().shifting;
    // Every evaluated outcome — deferred at standstill as much as
    // accepted/rejected — proves the request reached the automaton.
    bt::require(after.rider_response != "none",
                "the advancing tick consumes the request");
    bt::require((after.rider_response == "accepted") ==
                    (after.rear_teeth != probed.rear_teeth),
                "accepted shifts, rejected keeps the cog");
}

// Stationary target teeth: validation, cassette rejection, multi-cog commit
// under braking/no-pedaling, idempotency across perception lag, and
// deferral during an active shift cut.
void test_stationary_target_teeth() {
    // 1) Boundary validation rejects inconsistent or too-small targets.
    {
        auto s = shifter(12);
        drivetrain_reference::error<std::invalid_argument>([&] {
            (void)s.update(0., 0., .01, false, true, true, std::nullopt,
                           {.owns = false, .stationary_target_teeth = 39});
        }, "rider shift without ownership");
        drivetrain_reference::error<std::invalid_argument>([&] {
            (void)s.update(0., 0., .01, false, true, true, std::nullopt,
                           {.owns = true,
                            .request = drivetrain::ShiftDirection::down,
                            .stationary_target_teeth = 39});
        }, "rider shift conflict");
        drivetrain_reference::error<std::invalid_argument>([&] {
            (void)s.update(0., 0., .01, false, true, true, std::nullopt,
                           {.owns = true, .stationary_target_teeth = 2});
        }, "stationary target teeth");
    }
    // 2) Unknown cog on the cassette is rejected without moving gear.
    {
        auto s = shifter(12);
        bt::require(!s.update(0., 0., .01, false, true, true, std::nullopt,
                              {.owns = true, .stationary_target_teeth = 25}),
                    "unknown target cog does not shift");
        bt::require(s.state().rider_response == "rejected" &&
                        s.state().rear_teeth == 12,
                    "unknown target cog reports rejected");
    }
    // 3) Multi-cog jump 12 -> 39 commits while braking & not pedaling,
    //    clears cut immediately, and repeats idempotently while >= target.
    {
        auto s = shifter(12);
        bt::require(s.update(60., 60., .01, false, true, true, std::nullopt,
                             {.owns = true, .stationary_target_teeth = 39}),
                    "stationary jump 12 -> 39 commits even when braking and not pedaling");
        bt::require(s.state().from_teeth == 12 &&
                        s.state().rear_teeth == 39 &&
                        s.state().direction == "down" &&
                        s.state().shift_count == 1 &&
                        s.state().cut_remaining_s == 0. &&
                        s.state().rider_response == "accepted" &&
                        std::abs(s.state().required_ema.value() - 60. * 39. / 12.) < 1e-9, // NOLINT(bugprone-unchecked-optional-access) update() initializes required_ema
                    "stationary commit updates gear, count, landing EMA and clears cut");
        // Repeat while perception lag keeps target=39: idempotent no-op.
        bt::require(!s.update(0., 0., .01, false, true, true, std::nullopt,
                              {.owns = true, .stationary_target_teeth = 39}),
                    "repeat stationary target at 39 is idempotent");
        bt::require(s.state().rider_response == "none" &&
                        s.state().shift_count == 1 &&
                        s.state().rear_teeth == 39,
                    "idempotent repeat leaves shift_count and response none");
        // Already on an even larger cog (51 >= 39) is also idempotent.
        auto bigger = shifter(51);
        bt::require(!bigger.update(0., 0., .01, false, true, true, std::nullopt,
                                   {.owns = true, .stationary_target_teeth = 39}),
                    "larger cog than target is a no-op");
        bt::require(bigger.state().rider_response == "none" &&
                        bigger.state().rear_teeth == 51,
                    "larger cog keeps 51 and reports none");
    }
    // 4) Deferred while an ordinary cut is active, then commits once clear.
    {
        auto s = shifter(12);
        bt::require(s.update(90., 90., .01, true, false, true, std::nullopt,
                             {.owns = true,
                              .request = drivetrain::ShiftDirection::down}),
                    "start ordinary downshift 12 -> 14 with 0.2 s cut");
        bt::require(s.state().rear_teeth == 14 &&
                        s.state().cut_remaining_s > .15,
                    "ordinary downshift arms cut");
        bt::require(!s.update(0., 0., .05, false, true, true, std::nullopt,
                              {.owns = true, .stationary_target_teeth = 39}),
                    "stationary target defers while cut_remaining_s > 0");
        bt::require(s.state().rider_response == "deferred" &&
                        s.state().rear_teeth == 14,
                    "deferred response keeps 14");
        bt::require(s.update(0., 0., .20, false, true, true, std::nullopt,
                             {.owns = true, .stationary_target_teeth = 39}),
                    "stationary target commits as soon as cut finishes");
        bt::require(s.state().rider_response == "accepted" &&
                        s.state().rear_teeth == 39 &&
                        s.state().cut_remaining_s == 0.,
                    "committed stationary shift clears cut and lands on 39");
    }
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
    test_rider_up_request_moves_one_cog_with_cooldown_and_cut();
    test_rider_request_inside_cooldown_is_deferred();
    test_rider_request_past_the_cassette_end_is_rejected();
    test_rider_request_keeps_the_admission_vetoes();
    test_rider_request_keeps_the_slip_and_landing_vetoes();
    test_automatic_path_reports_no_rider_response();
    test_rider_request_magnitude_slip_mode_vetoes_negative_slip();
    test_mid_cut_request_defers_and_serialized_state_continues();
    test_rider_request_keeps_the_count_overflow_guard();
    test_rider_request_on_an_off_cassette_state_is_internal_error();
    test_rider_shift_contract_validation();
    test_probe_prepare_does_not_consume_a_rider_request();
    test_stationary_target_teeth();
}); }
