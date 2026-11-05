// Task 3.19: tests/reference/test_assist_controller.py, all 22 functions.
// Constants and tolerances below are the source's analytic assertions.
#include "drive_reference_support.hpp"
#include "support/temporary_directory.hpp"
#include "cli/application.hpp"
#include "io/json.hpp"

#include <array>
#include <fstream>
#include <limits>
#include <string>

namespace {
using drivetrain::AssistController;
using drive_reference::near;
using drive_reference::run_assist;
namespace cfg = bike::configuration;

[[gnu::noinline]] void test_profile_owns_limits_and_lag(const drivetrain::AssistConfig& config) {
    bt::require(config.max_torque == 85. && config.max_power == 600. && config.tau == .04,
                "profile owns torque, power and lag");
    near(config.cutoff_mps, 25. / 3.6, "profile cutoff");
    near(config.taper_width_mps, 2. / 3.6, "profile taper");
    bt::require(config.profile.has_value() && config.mode == "turbo", "resolved profile and mode");
    bt::require(cfg::kBoschCxGen4.name.starts_with("bosch"), "profile has Bosch source identity");
}
[[gnu::noinline]] void test_turbo_multiplies_rider_torque_through_a_40ms_first_order_lag(const drivetrain::AssistConfig& config) {
    AssistController c(config);
    near(run_assist(c, 20., 60., 3., .04), 68. * (1. - std::exp(-1.)), "40ms torque response", .03);
    near(run_assist(c, 20., 60., 3., .5), 68., "settled turbo torque", 1e-3);
    near(c.state().last_gain, 3.4, "turbo support factor");
}
[[gnu::noinline]] void test_emtb_gain_rises_with_rider_torque(const drivetrain::AssistConfig& config) {
    auto input = config; input.mode = "emtb"; AssistController c(input);
    near(run_assist(c, 10., 60., 3., .5), (1.4 + .5) * 10., "eMTB support ramp", 1e-3);
}
[[gnu::noinline]] void test_stopped_or_backward_crank_still_gets_assist_while_pressing(const drivetrain::AssistConfig& config) {
    // The crank-speed gate is gone with the motor clutch: the controller
    // cannot see crank direction — the weld owns that information — so a
    // pressed pedal is assisted even while the crank stands or moves back.
    for (double const cadence : {0., .5, -30.}) {
        AssistController c(config);
        bt::require(run_assist(c, 50., cadence, 0., .2) > 0., "crank speed no longer gates assist");
    }
}
[[gnu::noinline]] void test_torque_below_engage_threshold_is_not_pedalling(const drivetrain::AssistConfig& config) {
    auto input = config; input.engage_torque_nm = 4.; AssistController c(input);
    bt::require(run_assist(c, 4., 60., 3., .2) == 0., "engage threshold is exclusive");
    bt::require(run_assist(c, 4.01, 60., 3., .2) > 0., "torque above threshold engages");
}
// P1 (plan 2026-11-05-clutch-pedaling-fixes; release_hold_s = 0.3 s approved
// 2026-11-05): on a decrease of the rider-proportional target the motor
// holds the last delivered POWER — torque x shaft speed — for
// release_hold_s, still under the torque envelope and speed taper, then
// follows the smaller target through the tau lag.
[[gnu::noinline]] void test_easing_off_holds_the_delivered_power_for_release_hold_s(const drivetrain::AssistConfig& config) {
    bt::require(config.release_hold_s == .3, "approved run-on window reaches the controller");
    AssistController c(config);
    near(run_assist(c, 20., 60., 3., .5), 68., "settled rider-proportional delivery", 1e-3);
    for (int i = 0; i < 300; ++i)
        (void)c.step(10., 60., 3., false, .001, std::nullopt, 60.);
    near(c.state().torque, 68., "held power reads as held torque at the armed shaft speed", 1e-3);
    near(run_assist(c, 10., 60., 3., .5), 34., "the lag then follows the smaller target", 1e-2);
    // The hold is in power, not torque: armed at 60 rpm, a faster shaft
    // then draws less torque for the same watts; a slower shaft is still
    // envelope-capped.
    AssistController faster(config), slower(config);
    (void)run_assist(faster, 20., 60., 3., .5);
    (void)run_assist(slower, 20., 60., 3., .5);
    (void)faster.step(10., 60., 3., false, .001, std::nullopt, 60.);
    for (int i = 0; i < 110; ++i)
        (void)faster.step(10., 60., 3., false, .001, std::nullopt, 90.);
    near(faster.state().torque, 68. * 60. / 90., "same held power through a faster shaft", 2e-2);
    (void)slower.step(10., 60., 3., false, .001, std::nullopt, 60.);
    near(slower.step(10., 60., 3., false, .001, std::nullopt, 30.), 85.,
         "the torque envelope still caps the held power", 1e-3);
}
[[gnu::noinline]] void test_stopped_pedals_hold_the_motor_for_release_hold_s_then_release(const drivetrain::AssistConfig& config) {
    // A full stop of pedalling is the same ease-off case: with the shaft
    // still turning (the one-way clutch case) the motor runs on for
    // release_hold_s, then the lag takes the delivery to zero.
    AssistController c(config);
    (void)run_assist(c, 20., 60., 3., .5);
    for (int i = 0; i < 300; ++i)
        near(c.step(0., 0., 3., false, .001, std::nullopt, 60.), 68.,
             "run-on keeps delivering the held power", 1e-3);
    bt::require(run_assist(c, 0., 0., 3., .8) < .1, "the hold expires and the lag releases to zero");
    bt::require(c.state().release_hold_left_s == 0. && c.state().torque < .1,
                "expired hold leaves no armed state behind");
    bt::require(c.state().hold_power_w > 0.,
                "the spent marker stays until a target rise re-arms the hold");
}
[[gnu::noinline]] void test_speed_taper_and_cutoff(const drivetrain::AssistConfig& config) {
    AssistController c22(config), c24(config), c25(config), c30(config);
    near(run_assist(c22, 20., 60., 22. / 3.6, .5), 68., "below speed taper", 1e-3);
    near(run_assist(c24, 20., 60., 24. / 3.6, .5), 34., "half speed taper", 1e-2);
    bt::require(run_assist(c25, 20., 60., 25. / 3.6, .5) == 0., "cutoff boundary");
    bt::require(run_assist(c30, 20., 60., 30. / 3.6, .5) == 0., "above speed cutoff");
}
[[gnu::noinline]] void test_torque_curve_and_power_ceiling_bind_on_shaft_rpm(const drivetrain::AssistConfig& config) {
    auto input = config;
    input.torque_curve = std::vector<std::array<double, 2>>{{0., 85.}, {120., 85.}, {120.1, 47.7}, {180., 0.}};
    AssistController c60(input), c150(input), override_rpm(input);
    near(run_assist(c60, 100., 60., 3., .5), 85., "peak torque binds at 60rpm", 1e-3);
    near(run_assist(c150, 100., 150., 3., .5), 23.85, "curve binds at 150rpm", 1e-2);
    double torque = 0.;
    for (int i = 0; i < 500; ++i) torque = override_rpm.step(100., 60., 3., false, .001, std::nullopt, 150.);
    near(torque, 23.85, "shaft rpm overrides crank rpm for ceiling", 1e-2);
}
[[gnu::noinline]] void test_external_request_is_a_ceiling_not_a_throttle(const drivetrain::AssistConfig& config) {
    AssistController no_human(config), above(config), capped(config), zero(config);
    bt::require(run_assist(no_human, 0., 60., 3., .5, 100.) == 0., "cap alone cannot command assist");
    near(run_assist(above, 20., 60., 3., .5, 100.), 68., "cap above rider request", 1e-3);
    near(run_assist(capped, 20., 60., 3., .5, 30.), 30., "external cap binds", 1e-3);
    bt::require(run_assist(zero, 20., 60., 3., .5, 0.) == 0., "zero cap closes permission");
}
[[gnu::noinline]] void test_braking_resets_everything(const drivetrain::AssistConfig& config) {
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    (void)c.step(10., 60., 3., false, .001, std::nullopt, 60.);
    bt::require(c.state().release_hold_left_s > 0., "run-on armed before the brake");
    bt::require(c.step(20., 60., 3., true, .001) == 0. && c.state().torque == 0., "braking resets torque at once");
    bt::require(c.state().release_hold_left_s == 0. && c.state().hold_power_w == 0.,
                "braking clears the armed run-on");
}
[[gnu::noinline]] void test_rising_press_keeps_the_lag_even_mid_hold(const drivetrain::AssistConfig& config) {
    // Rises are unchanged: a bigger target ends the run-on early and the
    // same 40 ms lag follows it up — the hold never freezes delivery.
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    (void)c.step(10., 60., 3., false, .001, std::nullopt, 60.);
    const double first = c.step(40., 60., 3., false, .001, std::nullopt, 60.);
    bt::require(first > 68. && first < 69., "a rise resumes through the lag, not a jump");
    bt::require(c.state().release_hold_left_s == 0. && c.state().hold_power_w == 0.,
                "a rise ends the run-on early");
    near(run_assist(c, 40., 60., 3., .5), 85., "the lag settles at the envelope-limited target", 1e-2);
}
[[gnu::noinline]] void test_without_profile_the_plain_gain_is_used() {
    cfg::AssistConfig source; source.gain = 4.; source.tau = .3; source.max_torque = 85.; source.max_power = 600.;
    const auto config = drive_reference::projected_policies(source).assist;
    bt::require(!config.profile, "synthetic motor has no profile");
    AssistController c(config);
    near(run_assist(c, 10., 60., 3., 3.), 40., "synthetic scalar support factor", 1e-3);
}
[[gnu::noinline]] void test_rejects_unknown_profile_mode_and_removed_knobs() {
    auto source = drive_reference::turbo_source(); source.profile = "shimano_ep8";
    bt::invalid([&] { cfg::detail::post_init(source); });
    source = drive_reference::turbo_source(); source.mode = "sport";
    bt::invalid([&] { cfg::detail::post_init(source); });
    for (auto knob : {"stall_timeout_s", "spin_rpm", "boost_s", "stop_delay"}) {
        const auto value = bike::io::parse_json(R"({"drive":{"assist":{")" + std::string(knob) + "\":1}}}");
        bt::invalid([&] { (void)cfg::detail::construct_config(value); });
    }
}
[[gnu::noinline]] void test_assist_config_accepts_profile_and_mode() {
    auto source = drive_reference::turbo_source(); source.mode = "emtb";
    cfg::detail::post_init(source);
    bt::require(source.mode == "emtb", "public config accepts eMTB");
    source.mode = "off"; bt::invalid([&] { cfg::detail::post_init(source); });
    const auto removed = bike::io::parse_json(R"({"drive":{"assist":{"stall_timeout_s":1}}})");
    bt::invalid([&] { (void)cfg::detail::construct_config(removed); });
}
[[gnu::noinline]] void test_slow_forward_pedalling_is_assisted_without_a_stall_timer(const drivetrain::AssistConfig& config) {
    AssistController c(config);
    near(run_assist(c, 20., 1., 0., 2.), 68., "slow forward crank remains assisted", 1e-3);
}
[[gnu::noinline]] void test_cutoff_is_instantaneous_after_assist_has_settled(const drivetrain::AssistConfig& config) {
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    bt::require(c.step(20., 60., 25. / 3.6, false, .001) == 0. && c.state().torque == 0., "cutoff immediately clears torque");
}
[[gnu::noinline]] void test_profile_overrides_explicit_torque_power_tau_and_speed() {
    auto source = drive_reference::turbo_source();
    source.max_torque = 1.; source.max_power = 1.; source.tau = 1.; source.cutoff_mps = 1.;
    source.taper_width_mps = .1; source.release_hold_s = .9;
    const auto resolved = drive_reference::projected_policies(source).assist;
    bt::require(resolved.release_hold_s == .9, "the run-on window stays user-owned under a profile");
    AssistController c(resolved);
    near(run_assist(c, 20., 60., 3., .5), 68., "profile overrides declared synthetic limits", 1e-3);
}
[[gnu::noinline]] void test_controller_rejects_invalid_parameters(const drivetrain::AssistConfig& config) {
    auto input = config; input.tau = std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] { (void)AssistController(input); });
    input = config; input.slew = std::numeric_limits<double>::infinity();
    bt::invalid([&] { (void)AssistController(input); });
    input = config; input.taper_width_mps = std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] { (void)AssistController(input); });
    // The C++ field is a string; the dynamic config boundary owns the [] case.
    const auto array_mode = bike::io::parse_json(R"({"drive":{"assist":{"mode":[]}}})");
    bt::invalid([&] { (void)cfg::detail::construct_config(array_mode); });
}
[[gnu::noinline]] void test_assist_config_exposes_profile_torque_limit() {
    auto profile = drive_reference::turbo_source(); profile.max_torque = 1.;
    cfg::AssistConfig plain; plain.max_torque = 27.;
    bt::require(profile.effective_max_torque() == 85., "profile effective maximum");
    bt::require(plain.effective_max_torque() == 27., "synthetic effective maximum");
}
[[gnu::noinline]] void test_profile_controller_can_be_copied_for_read_only_force_probes(const drivetrain::AssistConfig& config) {
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    auto probe = c; (void)probe.step(0., 60., 3., false, .001);
    // The copy armed its own run-on window; the original is untouched.
    bt::require(probe.state().release_hold_left_s > 0., "probe runs its own hold down");
    near(probe.state().torque, 68., "probe delivers its own held power", 1e-3);
    bt::require(c.state().release_hold_left_s == 0., "probe cannot arm the original's hold");
    near(c.state().torque, 68., "probe cannot mutate original torque", 1e-3);
    // Python's shared immutable profile identity has become owned value data;
    // restoring permission on the copy still uses the original profile gain.
    (void)probe.step(20., 60., 3., false, .001);
    near(probe.state().last_gain, c.state().last_gain, "copied profile preserves gain");
}
[[gnu::noinline]] void test_physical_cli_assist_selects_mode_without_replacing_the_toml_profile() {
    bt::TemporaryDirectory const directory("assist-cli");
    const auto path = directory.path() / "physics.toml";
    {
        std::ofstream file(path);
        file << "physics_mode = \"physical\"\ndrive_mode = \"crank_effort\"\n"
                "[drive.assist]\nprofile = \"bosch_cx_gen4\"\nmode = \"turbo\"\n";
        bt::require(static_cast<bool>(file), "write temporary physics config");
    }
    const auto spelling = path.string();
    const std::array<std::string_view, 4> override_args{"--physics-config", spelling, "--assist", "emtb"};
    const auto overridden = bike::cli::resolve_ride(bike::cli::parse_arguments("bike-ride", override_args));
    bt::require(overridden->setup.model.physics.has_value(), "CLI resolves physical config");
    const auto& resolved = overridden->setup.model.physics->drive.assist; // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
    bt::require(resolved.profile == std::optional<std::string>("bosch_cx_gen4") && resolved.mode == "emtb",
                "CLI changes mode while preserving TOML profile");
    const std::array<std::string_view, 2> default_args{"--physics-config", spelling};
    const auto defaults = bike::cli::resolve_ride(bike::cli::parse_arguments("bike-ride", default_args));
    bt::require(defaults->setup.model.physics.has_value() && defaults->setup.model.physics->drive.assist.mode == "turbo",
                "absent CLI override preserves TOML mode");
}
} // namespace
int main() { return bt::run([] {
    const auto config = drive_reference::turbo_config();
    test_profile_owns_limits_and_lag(config);
    test_turbo_multiplies_rider_torque_through_a_40ms_first_order_lag(config);
    test_emtb_gain_rises_with_rider_torque(config);
    test_stopped_or_backward_crank_still_gets_assist_while_pressing(config);
    test_torque_below_engage_threshold_is_not_pedalling(config);
    test_easing_off_holds_the_delivered_power_for_release_hold_s(config);
    test_stopped_pedals_hold_the_motor_for_release_hold_s_then_release(config);
    test_speed_taper_and_cutoff(config);
    test_torque_curve_and_power_ceiling_bind_on_shaft_rpm(config);
    test_external_request_is_a_ceiling_not_a_throttle(config);
    test_braking_resets_everything(config);
    test_rising_press_keeps_the_lag_even_mid_hold(config);
    test_without_profile_the_plain_gain_is_used();
    test_rejects_unknown_profile_mode_and_removed_knobs();
    test_assist_config_accepts_profile_and_mode();
    test_slow_forward_pedalling_is_assisted_without_a_stall_timer(config);
    test_cutoff_is_instantaneous_after_assist_has_settled(config);
    test_profile_overrides_explicit_torque_power_tau_and_speed();
    test_controller_rejects_invalid_parameters(config);
    test_assist_config_exposes_profile_torque_limit();
    test_profile_controller_can_be_copied_for_read_only_force_probes(config);
    test_physical_cli_assist_selects_mode_without_replacing_the_toml_profile();
}); }
