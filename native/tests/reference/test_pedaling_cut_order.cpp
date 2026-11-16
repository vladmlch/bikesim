// Launch part 3a — the shift cut after a rider-limited effort ceiling
// (review I4): the ordinary path keeps (target x cut) then min(ceiling);
// a rider-limited effort takes min(target, ceiling) first and cuts after,
// so a small ceiling cannot erase the cut.
#include "support/fixture_test.hpp"
#include "drive_reference_support.hpp"
#include "drivetrain/pedaling.hpp"

#include <cmath>
#include <optional>

namespace {

// The shared reference pedaling config (mash 60 N·m below 45 rpm, no
// slew): a crank at rest raises a 10 N·m effort toward 60 before any cap.
drivetrain::PedalingPolicy mash_policy() {
    return drivetrain::PedalingPolicy(drive_reference::pedaling_config());
}

// Ordinary order: (60 x .3 = 18) then min(10) = 10 — the cut is erased.
void test_ordinary_order_is_unchanged() {
    auto policy = mash_policy();
    const auto out = policy.update(0., 0., 0., 10., .00125, true, false, false, std::nullopt, 10., .3);
    bt::require(out.effort_nm == 10., "ordinary order keeps min(target x cut, ceiling)");
}

// Rider-limited order: min(60, 10) = 10, then x .3 = 3.
void test_rider_limited_effort_cuts_after_the_ceiling() {
    auto policy = mash_policy();
    const auto out = policy.update(0., 0., 0., 10., .00125, true, false, false, std::nullopt, 10., .3, true);
    bt::require(std::abs(out.effort_nm - 3.) < 1e-12, "the cut applies after the rider's ceiling");
}

// Without a cut the flag changes nothing, bit for bit.
void test_orders_agree_without_a_cut() {
    auto a = mash_policy();
    auto b = mash_policy();
    const auto ordinary = a.update(0., 0., 0., 10., .00125, true, false, false, std::nullopt, 10., 1.);
    const auto limited = b.update(0., 0., 0., 10., .00125, true, false, false, std::nullopt, 10., 1., true);
    bt::require(ordinary.effort_nm == limited.effort_nm, "factor 1 gives the same effort in both orders");
}

} // namespace

int main() {
    return bt::run([] {
        test_ordinary_order_is_unchanged();
        test_rider_limited_effort_cuts_after_the_ceiling();
        test_orders_agree_without_a_cut();
    });
}
