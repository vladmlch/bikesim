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

void test_profile_owns_limits_and_lag(const drivetrain::AssistConfig& config) {
    bt::require(config.max_torque == 85. && config.max_power == 600. && config.tau == .04,
                "profile owns torque, power and lag");
    near(config.cutoff_mps, 25. / 3.6, "profile cutoff");
    near(config.taper_width_mps, 2. / 3.6, "profile taper");
    near(config.gate_min_crank_rad_s, 5. * std::numbers::pi / 180., "profile crank gate");
    bt::require(config.profile.has_value() && config.mode == "turbo", "resolved profile and mode");
    bt::require(cfg::kBoschCxGen4.name.starts_with("bosch"), "profile has Bosch source identity");
}
void test_turbo_multiplies_rider_torque_through_a_40ms_first_order_lag(const drivetrain::AssistConfig& config) {
    AssistController c(config);
    near(run_assist(c, 20., 60., 3., .04), 68. * (1. - std::exp(-1.)), "40ms torque response", .03);
    near(run_assist(c, 20., 60., 3., .5), 68., "settled turbo torque", 1e-3);
    near(c.state().last_gain, 3.4, "turbo support factor");
}
void test_emtb_gain_rises_with_rider_torque(const drivetrain::AssistConfig& config) {
    auto input = config; input.mode = "emtb"; AssistController c(input);
    near(run_assist(c, 10., 60., 3., .5), (1.4 + .5) * 10., "eMTB support ramp", 1e-3);
}
void test_stationary_or_backward_crank_gets_nothing_even_with_pedal_pressure(const drivetrain::AssistConfig& config) {
    AssistController c(config);
    for (double cadence : {0., .5, -30.})
        bt::require(run_assist(c, 50., cadence, 0., .2) == 0., "closed crank gate");
    AssistController above(config), below(config);
    bt::require(run_assist(above, 50., 5. / 6. + 1e-3, 0., .2) > 0., "just above crank gate");
    bt::require(run_assist(below, 50., 5. / 6. - 1e-3, 0., .2) == 0., "just below crank gate");
}
void test_torque_below_engage_threshold_is_not_pedalling(const drivetrain::AssistConfig& config) {
    auto input = config; input.engage_torque_nm = 4.; AssistController c(input);
    bt::require(run_assist(c, 4., 60., 3., .2) == 0., "engage threshold is exclusive");
    bt::require(run_assist(c, 4.01, 60., 3., .2) > 0., "torque above threshold engages");
}
void test_release_of_rider_torque_hard_zeroes_and_forgets(const drivetrain::AssistConfig& config) {
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    bt::require(c.step(0., 60., 3., false, .001) == 0., "rider release zeros output");
    bt::require(c.state().torque == 0. && !c.state().pedaling, "rider release clears torque and permission");
    bt::require(c.step(20., 60., 3., false, .001) < 5., "permission restoration restarts lag");
}
void test_positive_unloading_keeps_the_lag(const drivetrain::AssistConfig& config) {
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    const double first = c.step(10., 60., 3., false, .001);
    bt::require(first > 34. && first < 68., "positive unloading preserves filter history");
}
void test_speed_taper_and_cutoff(const drivetrain::AssistConfig& config) {
    AssistController c22(config), c24(config), c25(config), c30(config);
    near(run_assist(c22, 20., 60., 22. / 3.6, .5), 68., "below speed taper", 1e-3);
    near(run_assist(c24, 20., 60., 24. / 3.6, .5), 34., "half speed taper", 1e-2);
    bt::require(run_assist(c25, 20., 60., 25. / 3.6, .5) == 0., "cutoff boundary");
    bt::require(run_assist(c30, 20., 60., 30. / 3.6, .5) == 0., "above speed cutoff");
}
void test_torque_curve_and_power_ceiling_bind_on_shaft_rpm(const drivetrain::AssistConfig& config) {
    auto input = config;
    input.torque_curve = std::vector<std::array<double, 2>>{{0., 85.}, {120., 85.}, {120.1, 47.7}, {180., 0.}};
    AssistController c60(input), c150(input), override_rpm(input);
    near(run_assist(c60, 100., 60., 3., .5), 85., "peak torque binds at 60rpm", 1e-3);
    near(run_assist(c150, 100., 150., 3., .5), 23.85, "curve binds at 150rpm", 1e-2);
    double torque = 0.;
    for (int i = 0; i < 500; ++i) torque = override_rpm.step(100., 60., 3., false, .001, std::nullopt, 150.);
    near(torque, 23.85, "shaft rpm overrides crank rpm for ceiling", 1e-2);
}
void test_external_request_is_a_ceiling_not_a_throttle(const drivetrain::AssistConfig& config) {
    AssistController no_human(config), above(config), capped(config), zero(config);
    bt::require(run_assist(no_human, 0., 60., 3., .5, 100.) == 0., "cap alone cannot command assist");
    near(run_assist(above, 20., 60., 3., .5, 100.), 68., "cap above rider request", 1e-3);
    near(run_assist(capped, 20., 60., 3., .5, 30.), 30., "external cap binds", 1e-3);
    bt::require(run_assist(zero, 20., 60., 3., .5, 0.) == 0., "zero cap closes permission");
}
void test_braking_resets_everything(const drivetrain::AssistConfig& config) {
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    bt::require(c.step(20., 60., 3., true, .001) == 0. && c.state().torque == 0., "braking resets torque");
}
void test_without_profile_the_plain_gain_is_used() {
    cfg::AssistConfig source; source.gain = 4.; source.tau = .3; source.max_torque = 85.; source.max_power = 600.;
    const auto config = drive_reference::projected_policies(source).assist;
    bt::require(!config.profile, "synthetic motor has no profile");
    AssistController c(config);
    near(run_assist(c, 10., 60., 3., 3.), 40., "synthetic scalar support factor", 1e-3);
}
void test_rejects_unknown_profile_mode_and_removed_knobs() {
    auto source = drive_reference::turbo_source(); source.profile = "shimano_ep8";
    bt::invalid([&] { cfg::detail::post_init(source); });
    source = drive_reference::turbo_source(); source.mode = "sport";
    bt::invalid([&] { cfg::detail::post_init(source); });
    for (auto knob : {"stall_timeout_s", "spin_rpm", "boost_s", "stop_delay"}) {
        const auto value = bike::io::parse_json("{\"drive\":{\"assist\":{\"" + std::string(knob) + "\":1}}}");
        bt::invalid([&] { (void)cfg::detail::construct_config(value); });
    }
}
void test_assist_config_accepts_profile_and_mode() {
    auto source = drive_reference::turbo_source(); source.mode = "emtb";
    cfg::detail::post_init(source);
    bt::require(source.mode == "emtb", "public config accepts eMTB");
    source.mode = "off"; bt::invalid([&] { cfg::detail::post_init(source); });
    const auto removed = bike::io::parse_json(R"({"drive":{"assist":{"stall_timeout_s":1}}})");
    bt::invalid([&] { (void)cfg::detail::construct_config(removed); });
}
void test_slow_forward_pedalling_is_assisted_without_a_stall_timer(const drivetrain::AssistConfig& config) {
    AssistController c(config);
    near(run_assist(c, 20., 1., 0., 2.), 68., "slow forward crank remains assisted", 1e-3);
}
void test_crank_gate_resets_the_filter_even_with_an_external_cap(const drivetrain::AssistConfig& config) {
    for (double cadence : {0., -30., 5. / 6.}) {
        AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
        bt::require(c.step(20., cadence, 3., false, .001, 100.) == 0., "cap cannot bypass crank gate");
        bt::require(c.state().torque == 0., "crank gate clears torque state");
        bt::require(c.step(20., 60., 3., false, .001) < 5., "crank gate resets lag");
    }
}
void test_cutoff_is_instantaneous_after_assist_has_settled(const drivetrain::AssistConfig& config) {
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    bt::require(c.step(20., 60., 25. / 3.6, false, .001) == 0. && c.state().torque == 0., "cutoff immediately clears torque");
}
void test_profile_overrides_explicit_torque_power_tau_speed_and_gate() {
    auto source = drive_reference::turbo_source();
    source.max_torque = 1.; source.max_power = 1.; source.tau = 1.; source.cutoff_mps = 1.;
    source.taper_width_mps = .1; source.gate_min_crank_rad_s = 10.;
    AssistController c(drive_reference::projected_policies(source).assist);
    near(run_assist(c, 20., 60., 3., .5), 68., "profile overrides declared synthetic limits", 1e-3);
}
void test_controller_rejects_invalid_parameters(const drivetrain::AssistConfig& config) {
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
void test_assist_config_exposes_profile_torque_limit() {
    auto profile = drive_reference::turbo_source(); profile.max_torque = 1.;
    cfg::AssistConfig plain; plain.max_torque = 27.;
    bt::require(profile.effective_max_torque() == 85., "profile effective maximum");
    bt::require(plain.effective_max_torque() == 27., "synthetic effective maximum");
}
void test_profile_controller_can_be_copied_for_read_only_force_probes(const drivetrain::AssistConfig& config) {
    AssistController c(config); (void)run_assist(c, 20., 60., 3., .5);
    auto probe = c; (void)probe.step(0., 60., 3., false, .001);
    bt::require(probe.state().torque == 0., "probe may clear its own torque");
    near(c.state().torque, 68., "probe cannot mutate original torque", 1e-3);
    // Python's shared immutable profile identity has become owned value data;
    // restoring permission on the copy still uses the original profile gain.
    (void)probe.step(20., 60., 3., false, .001);
    near(probe.state().last_gain, c.state().last_gain, "copied profile preserves gain");
}
void test_physical_cli_assist_selects_mode_without_replacing_the_toml_profile() {
    bt::TemporaryDirectory directory("assist-cli");
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
    const auto& resolved = overridden->setup.model.physics->drive.assist;
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
    test_stationary_or_backward_crank_gets_nothing_even_with_pedal_pressure(config);
    test_torque_below_engage_threshold_is_not_pedalling(config);
    test_release_of_rider_torque_hard_zeroes_and_forgets(config);
    test_positive_unloading_keeps_the_lag(config);
    test_speed_taper_and_cutoff(config);
    test_torque_curve_and_power_ceiling_bind_on_shaft_rpm(config);
    test_external_request_is_a_ceiling_not_a_throttle(config);
    test_braking_resets_everything(config);
    test_without_profile_the_plain_gain_is_used();
    test_rejects_unknown_profile_mode_and_removed_knobs();
    test_assist_config_accepts_profile_and_mode();
    test_slow_forward_pedalling_is_assisted_without_a_stall_timer(config);
    test_crank_gate_resets_the_filter_even_with_an_external_cap(config);
    test_cutoff_is_instantaneous_after_assist_has_settled(config);
    test_profile_overrides_explicit_torque_power_tau_speed_and_gate();
    test_controller_rejects_invalid_parameters(config);
    test_assist_config_exposes_profile_torque_limit();
    test_profile_controller_can_be_copied_for_read_only_force_probes(config);
    test_physical_cli_assist_selects_mode_without_replacing_the_toml_profile();
}); }
