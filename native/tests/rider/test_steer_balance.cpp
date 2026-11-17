#include "rider/steer_balance.hpp"
#include "support/fixture_test.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

using bike::rider::DelayedBalanceSample;
using bike::rider::SteerBalance;
using bike::rider::SteerBalanceObservation;
using bike::rider::SteerBalanceState;
using bike::rider::SteerGainNode;
using bike::rider::SteerRiderPreset;
using bike::spatial::SpatialConfig;
using bike::spatial::SpatialStartMode;
using bike::spatial::SpatialSteerSpec;
using bike::spatial::SpatialTireBackend;
using bike::spatial::SpatialTirePreset;

constexpr double kDt = 0.00125;

[[nodiscard]] SpatialSteerSpec default_steer_spec() noexcept {
    return {
        .range_deg = 45.0,
        .control_period_s = 0.005,
        .reaction_delay_s = 0.100,
        .max_torque_nm = 35.0,
    };
}

[[nodiscard]] SpatialConfig default_spatial_config() noexcept {
    return {
        .tire_backend = SpatialTireBackend::compliant_3d,
        .tire_preset = SpatialTirePreset::mtb_2p8_1p2bar,
        .start_mode = SpatialStartMode::moving,
        .corridor_half_width_m = 1.5,
        .capsize_roll_rad = 1.0,
        .steer = default_steer_spec(),
    };
}

[[nodiscard]] bool approx(double a, double b, double tol = 1e-12) noexcept {
    return std::abs(a - b) <= tol;
}

[[nodiscard]] bool node_approx(const SteerGainNode& a,
                               const SteerGainNode& b,
                               double tol = 1e-12) noexcept {
    return approx(a.speed_mps, b.speed_mps, tol) &&
           approx(a.k_phi, b.k_phi, tol) &&
           approx(a.k_phidot, b.k_phidot, tol) &&
           approx(a.k_delta, b.k_delta, tol) &&
           approx(a.k_deltadot, b.k_deltadot, tol) &&
           approx(a.k_psi, b.k_psi, tol) &&
           approx(a.k_y, b.k_y, tol) &&
           approx(a.k_ydot, b.k_ydot, tol);
}

void test_noexcept_and_nonblocking_contracts() {
    SteerBalance ctrl(default_steer_spec(), kDt, SteerRiderPreset::lumped);
    const SteerBalanceObservation obs{};
    const DelayedBalanceSample sample{};
    static_assert(noexcept(ctrl.step(0, obs)));
    static_assert(noexcept(ctrl.update(0, obs)));
    static_assert(noexcept(ctrl.reset(sample, 0.0)));
    static_assert(noexcept(ctrl.held_torque_nm()));
    static_assert(noexcept(ctrl.period_steps()));
    static_assert(noexcept(ctrl.delay_steps()));
    static_assert(noexcept(ctrl.gains_for_speed(4.0)));
    bt::require(ctrl.period_steps() == 4, "5 ms at 1.25 ms dt is 4 steps");
    bt::require(ctrl.delay_steps() == 80, "100 ms at 1.25 ms dt is 80 steps");
}

void test_z_up_sign_conventions_and_immediate_proprioception() {
    // Use zero delay to verify pure algebraic sign conventions at step 0,
    // and separate nonzero delay to verify immediate (delta, delta_dot) vs delayed channels.
    SpatialSteerSpec zero_delay = default_steer_spec();
    zero_delay.reaction_delay_s = 0.0;

    // 1. Lean RIGHT (phi > 0) -> steer RIGHT (tau_steer < 0)
    {
        SteerBalance ctrl(zero_delay, kDt, SteerRiderPreset::lumped);
        SteerBalanceObservation obs{};
        obs.forward_speed_mps = 4.0;
        obs.roll_rad = 0.05; // +phi = leaning RIGHT
        const double tau = ctrl.step(0, obs);
        bt::require(tau < 0.0 && approx(tau, -92.0 * 0.05),
                    "phi > 0 (lean right) -> tau_steer < 0 (steer right)");
    }

    // 2. Roll rate RIGHT (phi_dot > 0) -> steer RIGHT (tau_steer < 0)
    {
        SteerBalance ctrl(zero_delay, kDt, SteerRiderPreset::lumped);
        SteerBalanceObservation obs{};
        obs.forward_speed_mps = 4.0;
        obs.roll_rate_rad_s = 0.10;
        const double tau = ctrl.step(0, obs);
        bt::require(tau < 0.0 && approx(tau, -46.0 * 0.10),
                    "phi_dot > 0 (rolling right) -> tau_steer < 0 (steer right)");
    }

    // 3. Lateral offset LEFT (y > 0) -> phi_cmd > 0 (lean right) -> initial counter-steer LEFT (tau_steer > 0)
    {
        SteerBalance ctrl(zero_delay, kDt, SteerRiderPreset::lumped);
        SteerBalanceObservation obs{};
        obs.forward_speed_mps = 4.0;
        obs.lateral_err_m = 0.35; // +Y = LEFT of centerline
        const double expected_phi_cmd = 0.042 * 0.35;
        const double expected_tau = 92.0 * expected_phi_cmd;
        const double tau = ctrl.step(0, obs);
        bt::require(tau > 0.0 && approx(tau, expected_tau),
                    "y > 0 (left of centerline) -> phi_cmd > 0 -> initial counter-steer LEFT (tau > 0)");
    }

    // 4. Heading LEFT (psi > 0) and lateral velocity LEFT (ydot > 0) -> counter-steer LEFT (tau > 0)
    {
        SteerBalance ctrl(zero_delay, kDt, SteerRiderPreset::lumped);
        SteerBalanceObservation obs{};
        obs.forward_speed_mps = 4.0;
        obs.heading_err_rad = 0.08;
        obs.lateral_rate_mps = 0.25;
        const double expected_phi_cmd = (0.20 * 0.08) + (0.032 * 0.25);
        const double expected_tau = 92.0 * expected_phi_cmd;
        const double tau = ctrl.step(0, obs);
        bt::require(tau > 0.0 && approx(tau, expected_tau),
                    "psi > 0 and ydot > 0 -> phi_cmd > 0 -> initial counter-steer LEFT (tau > 0)");
    }

    // 5. Proprioceptive (delta, delta_dot) are UN-DELAYED even with 100 ms ring buffer
    {
        SteerBalance ctrl(default_steer_spec(), kDt, SteerRiderPreset::lumped);
        SteerBalanceObservation obs{};
        obs.forward_speed_mps = 6.0;
        obs.roll_rad = 0.10;         // delayed 80 steps -> ignored at step 0
        obs.lateral_err_m = 0.30;    // delayed 80 steps -> ignored at step 0
        obs.steer_angle_rad = 0.04;  // immediate!
        obs.steer_rate_rad_s = 0.20; // immediate!
        const double tau0 = ctrl.step(0, obs);
        const double expected = -(15.0 * 0.04) - (22.0 * 0.20);
        bt::require(tau0 < 0.0 && approx(tau0, expected),
                    "(delta, delta_dot) act immediately at step 0 with zero delay");
    }
}

void test_exact_100ms_delay_step_response_and_zoh() {
    SteerBalance ctrl(default_steer_spec(), kDt, SteerRiderPreset::lumped);

    // Step 0: inject a unit roll perturbation on the delayed vestibular channel
    // while keeping immediate steer channels at 0.
    SteerBalanceObservation step0_obs{};
    step0_obs.forward_speed_mps = 4.0;
    step0_obs.roll_rad = 0.10;
    bt::require(approx(ctrl.step(0, step0_obs), 0.0),
                "step 0 must output 0 (ring initialized to settled equilibrium)");

    // Steps 1..79: return delayed channels to 0; output must remain strictly 0.0
    // across all 20 intermediate ticks (< 80 steps = 100 ms).
    SteerBalanceObservation zero_obs{};
    zero_obs.forward_speed_mps = 4.0;
    for (std::int64_t s = 1; s < 80; ++s) {
        const double tau = ctrl.step(s, zero_obs);
        bt::require(approx(tau, 0.0),
                    "delay ring must hold initial equilibrium for steps 1..79 (< 100 ms)");
    }

    // Step 80 (100.0 ms, control tick 80 % 4 == 0): exact step-0 perturbation emerges!
    const double tau80 = ctrl.step(80, zero_obs);
    bt::require(approx(tau80, -92.0 * 0.10),
                "step-0 roll perturbation must emerge at exact step 80 (100 ms)");

    // Steps 81, 82, 83: ZOH must hold tau80 EVEN IF immediate (delta, delta_dot) change!
    for (std::int64_t s = 81; s <= 83; ++s) {
        SteerBalanceObservation perturbed_steer = zero_obs;
        perturbed_steer.steer_angle_rad = 0.5;
        perturbed_steer.steer_rate_rad_s = 2.0;
        const double tau = ctrl.update(s, perturbed_steer);
        bt::require(approx(tau, tau80),
                    "ZOH must hold torque across intermediate steps 81..83 of 5 ms period");
    }

    // Step 84 (next 5 ms tick): delayed sample is step-4 (zero), immediate steer is active!
    SteerBalanceObservation tick84 = zero_obs;
    tick84.steer_angle_rad = 0.1;
    tick84.steer_rate_rad_s = 0.2;
    const double tau84 = ctrl.step(84, tick84);
    bt::require(approx(tau84, -(22.0 * 0.1) - (11.0 * 0.2)),
                "control tick 84 updates using step-4 delayed state and immediate steer");

    // Verify sub-period delay support (control_period_s = 10 ms, reaction_delay_s = 15 ms = 12 steps)
    SpatialSteerSpec sub_period = default_steer_spec();
    sub_period.control_period_s = 0.010;
    sub_period.reaction_delay_s = 0.015;
    SteerBalance sub_ctrl(sub_period, kDt, SteerRiderPreset::none);
    bt::require(sub_ctrl.period_steps() == 8 && sub_ctrl.delay_steps() == 12,
                "10 ms period = 8 steps, 15 ms delay = 12 steps");
    SteerBalanceObservation pulse_at_4{};
    pulse_at_4.forward_speed_mps = 4.0;
    pulse_at_4.roll_rad = 0.10;
    for (std::int64_t s = 0; s < 16; ++s) {
        const SteerBalanceObservation& in = (s == 4) ? pulse_at_4 : zero_obs;
        const double tau = sub_ctrl.step(s, in);
        bt::require(approx(tau, 0.0), "steps 0..15 see pre-step-4 zeros on ticks 0 and 8");
    }
    const double tau16 = sub_ctrl.step(16, zero_obs);
    bt::require(approx(tau16, -25.0 * 0.10),
                "step-4 observation arrives at step 16 (12-step delay = 15 ms)");
}

void test_saturation_and_lean_command_clamp() {
    SpatialSteerSpec zero_delay = default_steer_spec();
    zero_delay.reaction_delay_s = 0.0;
    zero_delay.max_torque_nm = 35.0;
    SteerBalance ctrl(zero_delay, kDt, SteerRiderPreset::lumped, 0.25);

    // Huge lateral error clamps phi_cmd to +0.25 rad -> tau = +92 * 0.25 = +23 N*m (< 35 N*m)
    {
        SteerBalanceObservation obs{};
        obs.forward_speed_mps = 4.0;
        obs.lateral_err_m = 100.0;
        const double tau = ctrl.step(0, obs);
        bt::require(approx(tau, 92.0 * 0.25),
                    "outer loop lean command clamps strictly to +max_lean_cmd_rad (0.25 rad)");
    }

    // Huge negative lateral error clamps phi_cmd to -0.25 rad
    {
        SteerBalanceObservation obs{};
        obs.forward_speed_mps = 4.0;
        obs.lateral_err_m = -100.0;
        const double tau = ctrl.step(0, obs);
        bt::require(approx(tau, -92.0 * 0.25),
                    "outer loop lean command clamps strictly to -max_lean_cmd_rad (-0.25 rad)");
    }

    // Torque saturation at +max_torque_nm and -max_torque_nm
    {
        SteerBalanceObservation obs{};
        obs.forward_speed_mps = 2.5;
        obs.roll_rad = 1.0; // -110 * 1.0 = -110 -> -35 N*m
        bt::require(approx(ctrl.step(0, obs), -35.0),
                    "negative steer torque clamps strictly to -max_torque_nm");

        obs.roll_rad = -1.0; // +110 -> +35 N*m
        bt::require(approx(ctrl.step(4, obs), 35.0),
                    "positive steer torque clamps strictly to +max_torque_nm");
    }
}

void test_presets_schedules_and_mass_adaptation() {
    const SpatialSteerSpec spec = default_steer_spec();
    const SteerBalance lumped_preset(spec, kDt, SteerRiderPreset::lumped);
    const SteerBalance lumped_mass(spec, kDt, 104.70);
    const SteerBalance none_preset(spec, kDt, SteerRiderPreset::none);
    const SteerBalance none_mass(spec, kDt, 24.40);

    // Check all 7 canonical nodes for lumped (Table 2.3A)
    const std::array<SteerGainNode, 7> expected_lumped = {{
        {.speed_mps = 2.5,  .k_phi = 110.0, .k_phidot = 56.0, .k_delta = 18.0, .k_deltadot = 7.5,  .k_psi = 0.14, .k_y = 0.030, .k_ydot = 0.025},
        {.speed_mps = 3.0,  .k_phi = 108.0, .k_phidot = 56.0, .k_delta = 20.0, .k_deltadot = 9.0,  .k_psi = 0.18, .k_y = 0.036, .k_ydot = 0.030},
        {.speed_mps = 4.0,  .k_phi = 92.0,  .k_phidot = 46.0, .k_delta = 22.0, .k_deltadot = 11.0, .k_psi = 0.20, .k_y = 0.042, .k_ydot = 0.032},
        {.speed_mps = 6.0,  .k_phi = 36.0,  .k_phidot = 34.0, .k_delta = 15.0, .k_deltadot = 22.0, .k_psi = 0.22, .k_y = 0.050, .k_ydot = 0.035},
        {.speed_mps = 8.0,  .k_phi = 44.0,  .k_phidot = 10.0, .k_delta = 26.0, .k_deltadot = 4.0,  .k_psi = 0.22, .k_y = 0.050, .k_ydot = 0.035},
        {.speed_mps = 10.0, .k_phi = 44.0,  .k_phidot = 7.0,  .k_delta = 18.0, .k_deltadot = 5.0,  .k_psi = 0.22, .k_y = 0.050, .k_ydot = 0.035},
        {.speed_mps = 12.0, .k_phi = 44.0,  .k_phidot = 5.0,  .k_delta = 15.0, .k_deltadot = 5.0,  .k_psi = 0.22, .k_y = 0.050, .k_ydot = 0.035},
    }};
    // Check all 7 canonical nodes for none (Table 2.3B)
    const std::array<SteerGainNode, 7> expected_none = {{
        {.speed_mps = 2.5,  .k_phi = 29.0, .k_phidot = 16.5, .k_delta = 4.2, .k_deltadot = 2.8, .k_psi = 0.14, .k_y = 0.028, .k_ydot = 0.025},
        {.speed_mps = 3.0,  .k_phi = 28.0, .k_phidot = 16.0, .k_delta = 4.8, .k_deltadot = 3.5, .k_psi = 0.18, .k_y = 0.035, .k_ydot = 0.030},
        {.speed_mps = 4.0,  .k_phi = 25.0, .k_phidot = 14.5, .k_delta = 5.8, .k_deltadot = 4.8, .k_psi = 0.20, .k_y = 0.042, .k_ydot = 0.032},
        {.speed_mps = 6.0,  .k_phi = 19.0, .k_phidot = 10.0, .k_delta = 6.5, .k_deltadot = 5.2, .k_psi = 0.22, .k_y = 0.048, .k_ydot = 0.035},
        {.speed_mps = 8.0,  .k_phi = 15.5, .k_phidot = 6.8,  .k_delta = 6.5, .k_deltadot = 4.0, .k_psi = 0.22, .k_y = 0.048, .k_ydot = 0.035},
        {.speed_mps = 10.0, .k_phi = 15.0, .k_phidot = 5.5,  .k_delta = 6.8, .k_deltadot = 2.6, .k_psi = 0.22, .k_y = 0.045, .k_ydot = 0.035},
        {.speed_mps = 12.0, .k_phi = 15.0, .k_phidot = 4.5,  .k_delta = 7.0, .k_deltadot = 1.6, .k_psi = 0.22, .k_y = 0.045, .k_ydot = 0.035},
    }};

    for (std::size_t i = 0; i < 7; ++i) {
        bt::require(node_approx(lumped_preset.schedule()[i], expected_lumped[i]),
                    "lumped preset node matches Table 2.3A");
        bt::require(node_approx(lumped_mass.schedule()[i], expected_lumped[i]),
                    "104.70 kg mass matches lumped preset");
        bt::require(node_approx(none_preset.schedule()[i], expected_none[i]),
                    "none preset node matches Table 2.3B");
        bt::require(node_approx(none_mass.schedule()[i], expected_none[i]),
                    "24.40 kg mass matches none preset");
    }

    // Speed clamping outside [2.5, 12.0] and piecewise linear midpoint at 5.0 m/s
    bt::require(node_approx(lumped_preset.gains_for_speed(0.5), expected_lumped[0]),
                "speed < 2.5 clamps to node 0");
    bt::require(node_approx(lumped_preset.gains_for_speed(20.0), expected_lumped[6]),
                "speed > 12.0 clamps to node 6");
    const SteerGainNode mid5 = lumped_preset.gains_for_speed(5.0);
    bt::require(approx(mid5.k_phi, 0.5 * (92.0 + 36.0)) &&
                    approx(mid5.k_deltadot, 0.5 * (11.0 + 22.0)),
                "linear speed interpolation at 5.0 m/s");

    // Midpoint mass adaptation between 24.40 kg and 104.70 kg
    const double mid_mass_kg = 0.5 * (24.40 + 104.70);
    const SteerBalance mid_ctrl(spec, kDt, mid_mass_kg);
    bt::require(approx(mid_ctrl.schedule()[0].k_phi, 0.5 * (29.0 + 110.0), 1e-11),
                "mid-mass interpolation blends k_phi linearly");

    // Overweight mass adaptation clamped at 1.40 * 104.70
    const SteerBalance heavy_ctrl(spec, kDt, 200.0);
    bt::require(approx(heavy_ctrl.schedule()[0].k_phi, 110.0 * 1.40, 1e-11) &&
                    approx(heavy_ctrl.schedule()[0].k_deltadot, 7.5 * std::sqrt(1.40), 1e-11) &&
                    approx(heavy_ctrl.schedule()[0].k_y, 0.030, 1e-12),
                "overweight mass scales inner/steer gains by clamped alpha and sqrt(alpha)");
}

void expect_invalid_spec(const SpatialSteerSpec& spec,
                         double timestep_s = kDt,
                         double total_mass_kg = 104.70,
                         double max_lean_cmd_rad = 0.25) {
    bt::invalid([&] {
        (void)SteerBalance(spec, timestep_s, total_mass_kg, max_lean_cmd_rad);
    });
}

void test_fail_closed_validation() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    // Non-multiple control_period_s or reaction_delay_s rejected (no silent llround!)
    {
        SpatialSteerSpec bad = default_steer_spec();
        bad.control_period_s = 0.0051; // 4.08 ticks at 1.25 ms
        expect_invalid_spec(bad);
        bad.control_period_s = 0.005 + 1e-8;
        expect_invalid_spec(bad);
        bad.control_period_s = 0.0;
        expect_invalid_spec(bad);
    }
    {
        SpatialSteerSpec bad = default_steer_spec();
        bad.reaction_delay_s = 0.103; // 82.4 ticks at 1.25 ms
        expect_invalid_spec(bad);
        bad.reaction_delay_s = 0.1000001; // > 1e-12 relative tolerance
        expect_invalid_spec(bad);
        bad.reaction_delay_s = -0.005;
        expect_invalid_spec(bad);
    }
    {
        SpatialSteerSpec bad = default_steer_spec();
        bad.max_torque_nm = 0.0;
        expect_invalid_spec(bad);
        bad.max_torque_nm = -10.0;
        expect_invalid_spec(bad);
        bad.max_torque_nm = nan;
        expect_invalid_spec(bad);
    }

    // Invalid timestep_s, total_mass_kg, max_lean_cmd_rad
    const SpatialSteerSpec ok = default_steer_spec();
    expect_invalid_spec(ok, 0.0);
    expect_invalid_spec(ok, -kDt);
    expect_invalid_spec(ok, inf);
    expect_invalid_spec(ok, kDt, 0.0);
    expect_invalid_spec(ok, kDt, nan);
    expect_invalid_spec(ok, kDt, 104.70, 0.0);
    expect_invalid_spec(ok, kDt, 104.70, -0.1);

    // SpatialConfig constructor validates corridor_half_width_m >= 0.25
    {
        SpatialConfig bad = default_spatial_config();
        bad.corridor_half_width_m = 0.249;
        bt::invalid([&] { (void)SteerBalance(bad, kDt, SteerRiderPreset::lumped); });
        bad.corridor_half_width_m = nan;
        bt::invalid([&] { (void)SteerBalance(bad, kDt, SteerRiderPreset::none); });
    }
}

void test_reset_determinism_and_state_restore() {
    SteerBalance ctrl(default_spatial_config(), kDt, SteerRiderPreset::lumped);

    const auto run_sequence = [](SteerBalance& c, std::int64_t start, std::int64_t end) {
        std::vector<double> out;
        out.reserve(static_cast<std::size_t>(end - start));
        for (std::int64_t s = start; s < end; ++s) {
            SteerBalanceObservation obs{};
            obs.forward_speed_mps = 5.5 + (0.01 * static_cast<double>(s % 20));
            obs.roll_rad = 0.03 * std::sin(0.05 * static_cast<double>(s));
            obs.roll_rate_rad_s = 0.10 * std::cos(0.05 * static_cast<double>(s));
            obs.heading_err_rad = 0.02 * std::sin(0.02 * static_cast<double>(s));
            obs.lateral_err_m = 0.15 * std::cos(0.01 * static_cast<double>(s));
            obs.lateral_rate_mps = -0.05 * std::sin(0.01 * static_cast<double>(s));
            obs.steer_angle_rad = -0.02 * std::sin(0.07 * static_cast<double>(s));
            obs.steer_rate_rad_s = 0.08 * std::cos(0.07 * static_cast<double>(s));
            out.push_back(c.step(s, obs));
        }
        return out;
    };

    const std::vector<double> first_run = run_sequence(ctrl, 0, 200);

    // Mid-run snapshot + replay from reset must produce bit-identical results
    ctrl.reset();
    bt::require(ctrl.held_torque_nm() == 0.0, "reset clears held torque");
    const std::vector<double> second_run = run_sequence(ctrl, 0, 200);
    bt::require(first_run == second_run, "reset() restores bit-identical deterministic trajectory");

    // State capture/restore at step 100 matches uninterrupted continuation 100..200 bit-for-bit
    ctrl.reset();
    (void)run_sequence(ctrl, 0, 100);
    const SteerBalanceState snap = ctrl.state();

    // Perturb controller with junk, then restore
    (void)run_sequence(ctrl, 100, 150);
    ctrl.restore(snap);
    const std::vector<double> resumed_tail = run_sequence(ctrl, 100, 200);
    for (std::size_t i = 0; i < resumed_tail.size(); ++i) {
        bt::require(resumed_tail[i] == first_run[100U + i],
                    "state()/restore() yields bit-identical continuation");
    }

    // Reject invalid restore states atomically without mutating live state
    {
        SteerBalanceState bad_size = snap;
        bad_size.ring.pop_back();
        bt::invalid([&] { ctrl.restore(bad_size); });

        SteerBalanceState bad_head = snap;
        bad_head.head = snap.ring.size();
        bt::invalid([&] { ctrl.restore(bad_head); });

        SteerBalanceState bad_sample = snap;
        bad_sample.ring[0].roll_rad = std::numeric_limits<double>::quiet_NaN();
        bt::invalid([&] { ctrl.restore(bad_sample); });

        SteerBalanceState bad_torque = snap;
        bad_torque.held_torque_nm = ctrl.max_torque_nm() + 1.0;
        bt::invalid([&] { ctrl.restore(bad_torque); });

        bt::require(ctrl.held_torque_nm() == first_run[199],
                    "failed restore() leaves live state untouched");
    }
}

} // namespace

int main() {
    return bt::run([] {
        test_noexcept_and_nonblocking_contracts();
        test_z_up_sign_conventions_and_immediate_proprioception();
        test_exact_100ms_delay_step_response_and_zoh();
        test_saturation_and_lean_command_clamp();
        test_presets_schedules_and_mass_adaptation();
        test_fail_closed_validation();
        test_reset_determinism_and_state_restore();
    });
}
