// Task 3.19: tests/reference/test_pedaling_policy.py, all nine functions.
#include "drive_reference_support.hpp"
#include "io/json.hpp"
#include "research/environment.hpp"
#include "rider/state_codec.hpp"
#include "runtime/setup_values.hpp"

#include <string>

namespace {
using drivetrain::PedalMode;
using drivetrain::PedalingPolicy;
using drive_reference::pedaling_config;
using drive_reference::rpm_to_rad_s;
namespace cfg = bike::configuration;

void test_reposition_knobs_and_mode_are_gone() {
    const auto removed = bike::io::parse_json(R"({"drive":{"pedaling":{"reposition_on_stall":true}}})");
    bt::invalid([&] { (void)cfg::detail::construct_config(removed); });
    for (const auto& field : cfg::detail::kPedalingConfigFields)
        bt::require(field.name.find("reposition") == std::string_view::npos, "pedaling schema has no reposition knob");
    bt::require(!drivetrain::pedal_mode("reposition"), "reposition is absent from native mode domain");
    // Python's removed update(reposition=...) keyword is a signature-only
    // check. There is no C++ keyword argument or equivalent state field.
}
void test_a_rocking_loaded_crank_simply_keeps_pedalling() {
    auto config = pedaling_config(); config.coast_above_rpm = 110.; config.resume_below_rpm = 90.;
    PedalingPolicy policy(config);
    for (int step = 0; step < 3000; ++step) {
        const double rate = 3. * std::sin(static_cast<double>(step) * .02);
        const auto state = policy.update(std::numbers::pi / 2., rate, 0., 40., .001);
        bt::require(state.mode == PedalMode::pedaling, "rocking crank never enters removed reposition mode");
    }
}
void test_ride_control_has_no_reposition_field() {
    auto control = rider::encode_intent_control(runtime::RideControl{});
    bt::require(bike::io::find_member(bike::io::as_object(control, "control"), "crank_reposition") == nullptr,
                "control wire has no crank_reposition field");
    runtime::setupvalue::set(control, "crank_reposition", bike::io::Value{true});
    bt::invalid([&] { (void)rider::decode_intent_control(control, "control"); });
}
void test_rider_program_rejects_the_removed_reposition_command() {
    const auto removed = bike::io::parse_json(R"({"keyframes":[{"time_s":0.0,"crank_reposition":true}]})");
    bt::invalid([&] { (void)bike::research::programs_from_documents(removed, bike::io::Value{}); });
    // The native document loader names the unknown key in its diagnostic;
    // the source's Python-only constructor message is recorded in the port
    // review rather than falsely claimed as an identical error spelling.
}
void test_coast_decision_filters_a_single_stroke_spike_with_ema() {
    PedalingPolicy policy(pedaling_config(.35));
    for (int i = 0; i < 100; ++i)
        bt::require(policy.update(0., 90. * rpm_to_rad_s, 90., 30., .001).mode == PedalMode::pedaling,
                    "initial in-band cadence pedals");
    for (int i = 0; i < 50; ++i)
        bt::require(policy.update(0., 160. * rpm_to_rad_s, 90., 30., .001).mode == PedalMode::pedaling,
                    "50ms cadence spike is filtered");
    drivetrain::PedalingState state;
    for (int i = 0; i < 1500; ++i) state = policy.update(0., 160. * rpm_to_rad_s, 160., 30., .001);
    bt::require(state.mode == PedalMode::coasting && state.reason == drivetrain::CoastReason::cadence,
                "sustained cadence enters coast");
    for (int i = 0; i < 1500; ++i) state = policy.update(0., 95. * rpm_to_rad_s, 95., 30., .001);
    bt::require(state.mode == PedalMode::pedaling, "sustained low cadence resumes pedaling");
}
void test_zero_tau_keeps_the_raw_hysteresis() {
    PedalingPolicy policy(pedaling_config());
    bt::require(policy.update(0., 90. * rpm_to_rad_s, 90., 30., .001).mode == PedalMode::pedaling,
                "zero-tau initial cadence pedals");
    bt::require(policy.update(0., 121. * rpm_to_rad_s, 90., 30., .001).mode == PedalMode::coasting,
                "zero tau coasts on the first excessive sample");
}
void test_coast_tau_must_be_nonnegative() {
    auto config = pedaling_config(); config.coast_cadence_tau_s = -.1;
    bt::invalid([&] { (void)PedalingPolicy(config); });
    cfg::PedalingConfig source; source.enabled = true; source.coast_cadence_tau_s = -.1;
    bt::invalid([&] { cfg::detail::post_init(source); });
}
void test_wheel_implied_cadence_does_not_stop_legs_below_their_limit() {
    PedalingPolicy policy(pedaling_config(.35));
    const auto state = policy.update(0., 90. * rpm_to_rad_s, 160., 30., .001);
    bt::require(state.mode == PedalMode::pedaling, "wheel-implied cadence does not stop legs");
    bt::require(state.required_cadence_rpm == 160. && state.effort_nm == 30., "wheel cadence and effort retained");
}
void test_stopped_cranks_resume_while_the_open_freehub_wheel_keeps_turning() {
    PedalingPolicy policy(pedaling_config(.35));
    drivetrain::PedalingState state;
    for (int i = 0; i < 1500; ++i) state = policy.update(0., 160. * rpm_to_rad_s, 160., 30., .001);
    bt::require(state.mode == PedalMode::coasting, "high cadence first enters coast");
    for (int i = 0; i < 350; ++i) state = policy.update(0., 0., 160., 30., .001);
    bt::require(state.mode == PedalMode::pedaling && state.required_cadence_rpm == 160.,
                "stopped crank resumes despite spinning freehub wheel");
}
} // namespace
int main() { return bt::run([] {
    test_reposition_knobs_and_mode_are_gone();
    test_a_rocking_loaded_crank_simply_keeps_pedalling();
    test_ride_control_has_no_reposition_field();
    test_rider_program_rejects_the_removed_reposition_command();
    test_coast_decision_filters_a_single_stroke_spike_with_ema();
    test_zero_tau_keeps_the_raw_hysteresis();
    test_coast_tau_must_be_nonnegative();
    test_wheel_implied_cadence_does_not_stop_legs_below_their_limit();
    test_stopped_cranks_resume_while_the_open_freehub_wheel_keeps_turning();
}); }
