// Ports all five source test functions in
// tests/reference/test_coupled_rider_task.py — the opt-in coupled task
// control for the fully welded rider: crank torque as the allocator's task
// coordinate and tissue damping declared apart from the muscle gain.
//
// ArticulatedConfig(coupled_task_control=1) is a compile-time type
// rejection in the bool signature (the Python bool-field validator rejects
// the int); only the default-false half is a runtime contract.
#include "support/fixture_test.hpp"
#include "rider_control_reference_support.hpp"
#include "configuration/physical_config.hpp"
#include "numerics/dense.hpp"
#include "numerics/linear_feasibility.hpp"
#include "rider/response_allocation.hpp"
#include "runtime/samples.hpp"
#include "terrain/trackfile.hpp"

#include <cmath>
#include <cstdio>
#include <print>
#include <string>
#include <vector>

namespace {
namespace rr = rider_control_reference;
namespace cfg = bike::configuration;
namespace d = bike::numerics::dense;
namespace sw = ::runtime::sample_wire;
using bike::rider::Matrix;

// _environment's track document (test_coupled_rider_task.py:63-65).
bike::terrain::TrackSpec probe_track() {
    return bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = 100.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [100.0, 0.0]] }\n"));
}

// _environment's physics rewrite: weld pedal/saddle/grip (post_init
// normalizes the weld grip to the compiled connect equality), 0.5 N.m.s/rad
// tissue damping, and the opt-in coupled task control.
std::unique_ptr<rr::EnvRig> coupled_rig() {
    return rr::pinned_rig(
        [](cfg::SimulationPhysicsConfig &physics) {
            auto &a = physics.articulated;
            a.pedal_attachment = "weld";
            a.saddle_attachment = "weld";
            a.grip_attachment = "weld";
            a.joint_passive_damping_nms_rad = .5;
            a.coupled_task_control = true;
        },
        probe_track());
}

double time_s(const rr::EnvRig &rig) {
    return rig.stepper->data()->time;
}

[[gnu::noinline]] void
test_passive_damping_defaults_to_the_legacy_gain_coupling() {
    // coupled_rider_task/passive_damping_defaults_to_the_legacy_gain_coupling
    const cfg::ArticulatedConfig base;
    bt::require(base.passive_damping_nms_rad() == base.joint_kd_nms_rad,
                "nullopt keeps the dof damping == joint_kd coupling");
    cfg::ArticulatedConfig explicit_gain;
    explicit_gain.joint_passive_damping_nms_rad = .5;
    cfg::detail::post_init(explicit_gain);
    bt::require(explicit_gain.passive_damping_nms_rad() == .5,
                "an explicit damping value is its own");
    cfg::ArticulatedConfig negative;
    negative.joint_passive_damping_nms_rad = -1.;
    rr::expect_invalid([&] { cfg::detail::post_init(negative); }, "");
}

[[gnu::noinline]] void test_coupled_task_control_is_opt_in() {
    // coupled_rider_task/coupled_task_control_is_opt_in
    // ArticulatedConfig(coupled_task_control=1) is a compile-time bool
    // rejection; the oracle's runtime ValueError is the same boundary.
    const cfg::ArticulatedConfig base;
    bt::require(!base.coupled_task_control,
                "coupled task control defaults off");
}

[[gnu::noinline]] void
test_crank_task_coordinate_is_appended_and_weighted() {
    // coupled_rider_task/crank_task_coordinate_is_appended_and_weighted
    bt::require(bike::rider::crank_task_scale_nm == 1.,
                "the soft task is measured in newton metres");
    Matrix g = d::zeros(1, 2);
    g.values[0] = 1.;
    const std::vector<double> bound{1.}, lo{-1., -1.}, hi{1., 1.};
    bt::require(!bike::numerics::linear_region_infeasible(g, bound, lo, hi),
                "the box region stays feasible without the equality block");
    Matrix eq = d::zeros(1, 2);
    eq.values[0] = 1.;
    const std::vector<double> rhs{5.};
    bt::require(bike::numerics::linear_region_infeasible(g, bound, lo, hi,
                                                       eq, rhs),
                "the optional equality block joins the prefilter");
}

// test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step was
// deferred pre-F4: the native plant diverged from the oracle once the crank
// task ramped (t≈0.19 s: predicted 10.310 N·m vs welded-settle 9.689 N·m,
// oracle asserts <1e-3 relative and stays green). F4 (step.cpp:
// refresh_forward before the control-tick compute) closed the stale-efc
// sequencing gap the divergence traced to; the assert is enabled on the
// fixed physics. The pairing used here — allocation diagnostics captured
// right after the period's first interval against that interval's
// diagnostics.rider_welds.crank_torque_nm — matches bitwise for the first
// ~18 control periods.
[[gnu::noinline]] void
test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step() {
    // coupled_rider_task/planned_crank_torque_is_the_solved_weld_torque_of_the_first_step
    auto rig = coupled_rig();
    auto &physical = *rig->physical;
    {
        auto state = physical.state();
        state.measurement_decimation = 1;
        physical.restore(state);
    }
    std::size_t checked = 0;
    while (time_s(*rig) < .6) {
        // env.step(): run one control period; capture the allocation
        // diagnostics right after the first interval (the control tick
        // fires at the head of each period) so `planned` pairs with the
        // weld solve measured on that same first interval.
        const auto per = static_cast<std::size_t>(std::llround(
            bike::research::ExperimentConfig{}.control_period_s /
            physical.control_clock().timestep_s));
        std::vector<::runtime::RawStep> samples;
        samples.reserve(per);
        samples.push_back(physical.advance_physics(
            0., 0., std::nullopt, rr::idle_control()));
        const auto diag =
            physical.rider_control().state().allocation_diagnostics;
        for (std::size_t i = 1; i < per; ++i)
            samples.push_back(physical.advance_physics(
                0., 0., std::nullopt, rr::idle_control()));
        const auto *legacy = diag.legacy ? &*diag.legacy : nullptr;
        const auto *planned =
            legacy ? &legacy->predicted_crank_torque_nm : nullptr;
        if (planned == nullptr || !planned->has_value() ||
            samples.empty() || legacy->effort_scale == 0.)
            continue;
        // RawStep.diagnostics carries rider_welds; completed_samples'
        // channel copy is the accounting fold of the same object.
        const auto &welds = sw::object(
            sw::required(samples[0].diagnostics, "rider_welds"));
        const double solved = sw::number(sw::required(welds,
                                                    "crank_torque_nm"));
        if (std::abs(**planned - solved) >=
            1e-3 * std::max(1., std::abs(**planned)))
          std::println(stderr, "MISMATCH t={:f} planned={:.6f} solved={:.6f}",
                       time_s(*rig), **planned, solved);
        bt::require(std::abs(**planned - solved) <
                        1e-3 * std::max(1., std::abs(**planned)),
                    "the task coordinate predicts the measured weld torque");
        ++checked;
    }
    bt::require(checked >= 20,
                "at least twenty completed periods were compared");
}

[[gnu::noinline]] void
test_tissue_damping_does_not_eat_the_muscle_budget() {
    // coupled_rider_task/tissue_damping_does_not_eat_the_muscle_budget
    auto rig = coupled_rig();
    auto &physical = *rig->physical;
    double positive_sum = 0., passive_sum = 0.;
    std::size_t count = 0;
    while (time_s(*rig) < 1.5) {
        (void)rr::advance_env_steps(physical, 1);
        if (time_s(*rig) < .5)
            continue;
        const auto &effort =
            physical.rider_control().state().effort_diagnostics;
        positive_sum += effort.positive_power_w;
        passive_sum += -effort.passive_power_w;
        ++count;
    }
    const double positive = positive_sum / static_cast<double>(count);
    const double passive = passive_sum / static_cast<double>(count);
    bt::require(positive > 20.,
                "pedalling sustains a meaningful positive power");
    bt::require(passive < .3 * positive,
                "tissue damping stays a small fraction of muscle power");
}
} // namespace

int main() {
    return bt::run([] {
        test_passive_damping_defaults_to_the_legacy_gain_coupling();
        test_coupled_task_control_is_opt_in();
        test_crank_task_coordinate_is_appended_and_weighted();
        test_planned_crank_torque_is_the_solved_weld_torque_of_the_first_step();
        test_tissue_damping_does_not_eat_the_muscle_budget();
    });
}
