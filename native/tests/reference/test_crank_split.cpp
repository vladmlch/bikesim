// Ports all nine source test functions in
// tests/reference/test_crank_split.py — directional virtual work, preload
// and positive-work muscle budgets. The Python dict returns keep their
// insertion order: leg_shares/leg_crank_targets are (front, rear) pairs and
// scale_to_power_budget's dict is the caller's parallel-array order.
#include "support/fixture_test.hpp"
#include "rider/spindle_math.hpp"

#include <cmath>
#include <numbers>
#include <vector>

namespace {

// pytest.approx defaults: rel=1e-6, abs=1e-12.
bool approx(double actual, double expected, double rtol = 1e-6,
            double atol = 1e-12) {
    return std::abs(actual - expected) <= atol + rtol * std::abs(expected);
}

[[gnu::noinline]] void test_split_delivers_requested_torque_within_capacity() {
    // crank_split/split_delivers_requested_torque_within_capacity
    const auto cap = [](int joint, double) {
        return joint == 0 ? 180. : 140.;
    };
    const auto [th, tk] =
        spindle::split_joint_torques(40., {-.6, .9}, cap);
    bt::require(approx(th * -.6 + tk * .9, 40.),
                "the split's virtual work delivers the requested torque");
}

[[gnu::noinline]] void test_split_prefers_stronger_joint() {
    // crank_split/split_prefers_stronger_joint
    const auto cap = [](int joint, double) {
        return joint == 0 ? 200. : 50.;
    };
    const auto [th, tk] =
        spindle::split_joint_torques(30., {1., 1.}, cap);
    bt::require(th > tk && tk > 0.,
                "both joints push positive with the hip ahead");
    bt::require(approx(th / tk, 16.),
                "the split ratio is the squared capacity ratio");
}

[[gnu::noinline]] void test_split_uses_directional_capacity_and_clips() {
    // crank_split/split_uses_directional_capacity_and_clips
    const auto cap = [](int, double sign) { return sign > 0. ? 100. : 50.; };
    const auto [th, tk] =
        spindle::split_joint_torques(1000., {-1., 1.}, cap);
    bt::require(th == -50. && tk == 100.,
                "each joint saturates at its own directional capacity");
}

[[gnu::noinline]] void test_singular_and_zero_capacity_deliver_zero() {
    // crank_split/singular_and_zero_capacity_deliver_zero
    const auto hundred = [](int, double) { return 100.; };
    const auto zero = [](int, double) { return 0.; };
    const auto singular =
        spindle::split_joint_torques(20., {0., 0.}, hundred);
    const auto empty = spindle::split_joint_torques(20., {1., 1.}, zero);
    bt::require(singular[0] == 0. && singular[1] == 0.,
                "a zero Jacobian has no work channel and returns zero");
    bt::require(empty[0] == 0. && empty[1] == 0.,
                "zero capacity in both directions returns zero");
}

[[gnu::noinline]] void test_shares_sum_to_one() {
    // crank_split/shares_sum_to_one
    for (const double phi : {0., .3, 1.2, std::numbers::pi / 2., 2.,
                             std::numbers::pi, 4.}) {
        const auto shares = spindle::leg_shares(phi);
        bt::require(approx(shares[0] + shares[1], 1.),
                    "the two leg shares always sum to one");
    }
}

[[gnu::noinline]] void test_vertical_handover_and_opposite_power_strokes() {
    // crank_split/vertical_handover_and_opposite_power_strokes
    const auto top = spindle::leg_shares(std::numbers::pi / 2.);
    bt::require(approx(top[0], .5) && approx(top[1], .5),
                "the vertical handover splits the stroke evenly");
    const auto front_down = spindle::leg_shares(0.);
    bt::require(front_down[0] == 1. && front_down[1] == 0.,
                "phase zero is the front power stroke");
    const auto rear_down = spindle::leg_shares(std::numbers::pi);
    bt::require(rear_down[0] == 0. && rear_down[1] == 1.,
                "phase pi is the rear power stroke");
}

[[gnu::noinline]] void test_preload_is_downward_virtual_work_in_native_plus_y() {
    // crank_split/preload_is_downward_virtual_work_in_native_plus_y
    bt::require(approx(spindle::preload_torque_nm(40., {.17, 0.}, {0., 0.}),
                       6.8),
                "preload on the +x spindle is positive crank work");
    bt::require(approx(spindle::preload_torque_nm(40., {-.17, 0.}, {0., 0.}),
                       -6.8),
                "preload on the -x spindle is negative crank work");
    const auto targets =
        spindle::leg_crank_targets(60., 0., {.17, 0.}, {-.17, 0.}, {0., 0.},
                                   40.);
    bt::require(approx(targets[0], 66.8) && approx(targets[1], -6.8),
                "the rising rear leg keeps the preload recovery torque");
}

[[gnu::noinline]] void test_handover_splits_without_preload() {
    // crank_split/handover_splits_without_preload
    const auto targets =
        spindle::leg_crank_targets(60., std::numbers::pi / 2., {0., -.17},
                                   {0., .17}, {0., 0.}, 40.);
    bt::require(targets[0] == 30. && targets[1] == 30.,
                "the even handover splits the request exactly in half");
}

[[gnu::noinline]] void test_power_budget_scales_positive_work_only() {
    // crank_split/power_budget_scales_positive_work_only
    // Dict order {'a','b','eccentric','still'} is the array order here.
    const std::vector<double> torques{100., 100., -100., 20.};
    const std::vector<double> velocities{4., 4., 4., 0.};
    const auto out =
        spindle::scale_to_power_budget(torques, velocities, 250., 450.);
    bt::require(out.size() == 4, "the budget keeps every dict entry");
    bt::require(out[0] == 56.25 && out[1] == 56.25 && out[2] == -100. &&
                    out[3] == 20.,
                "only positive-work channels scale down");
    bt::require(torques[0] == 100.,
                "the input torques are not mutated by the budget");
}
} // namespace

int main() {
    return bt::run([] {
        test_split_delivers_requested_torque_within_capacity();
        test_split_prefers_stronger_joint();
        test_split_uses_directional_capacity_and_clips();
        test_singular_and_zero_capacity_deliver_zero();
        test_shares_sum_to_one();
        test_vertical_handover_and_opposite_power_strokes();
        test_preload_is_downward_virtual_work_in_native_plus_y();
        test_handover_splits_without_preload();
        test_power_budget_scales_positive_work_only();
    });
}
