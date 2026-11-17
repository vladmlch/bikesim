// Task 3.19: tests/reference/test_pedaling_policy.py, all nine functions.
#include "drive_reference_support.hpp"
#include "drivetrain/stall_detector.hpp"
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

// The reposition knobs are back (2026-11-03); the mode domain is not:
// the manoeuvre reports as coasting with a reposition reason.
void test_reposition_knobs_exist_and_mode_domain_is_unchanged() {
    const auto doc = bike::io::parse_json(R"({"drive":{"pedaling":{"reposition_on_stall":true}}})");
    bool found = false;
    for (const auto& field : cfg::detail::kPedalingConfigFields)
        found = found || field.name == std::string_view("reposition_on_stall");
    bt::require(found, "pedaling schema declares reposition_on_stall");
    bt::require(!drivetrain::pedal_mode("reposition"), "reposition is not a pedaling mode");
    // Python's removed update(reposition=...) keyword is a signature-only
    // check. There is no C++ keyword argument or equivalent state field.
    (void)doc;
}
// ProgressStallDetector ports (c809f12a^ tests/test_pedaling_stall.py).
void test_stall_detector() {
    using drivetrain::StallDetectorLimits; using drivetrain::StallDetectorState;
    constexpr double dt = .00125;
    const StallDetectorLimits limits{.window_s = 1., .crank_progress_rad = .5, .wheel_progress_rad = .5};
    {   // wheel progress alone prevents a stall
        StallDetectorState s; bool fired = false;
        for (int i = 0; i < 2400; ++i)
            fired = drivetrain::stall_update(s, limits, {.crank_rad_s = 0., .wheel_rad_s = 1.}, dt, true) || fired;
        bt::require(!fired, "a rolling wheel with a parked crank is not a stall");
    }
    {   // rollback is not progress: fires exactly once after the window
        StallDetectorState s; int fired = 0;
        for (int i = 0; i < 880; ++i)
            fired += drivetrain::stall_update(s, limits, {.crank_rad_s = -.8, .wheel_rad_s = -2.}, dt, true) ? 1 : 0;
        bt::require(fired == 1, "backward motion accumulates as no progress");
    }
    {   // dropping the load restarts the window
        StallDetectorState s;
        const drivetrain::StallDetectorRates parked{.crank_rad_s = 0., .wheel_rad_s = 0.};
        for (int i = 0; i < 720; ++i) bt::require(!drivetrain::stall_update(s, limits, parked, dt, true), "no early fire");
        (void)drivetrain::stall_update(s, limits, parked, dt, false);
        bool fired = false;
        for (int i = 0; i < 720; ++i) fired = drivetrain::stall_update(s, limits, parked, dt, true) || fired;
        bt::require(!fired, "a stall needs a full loaded window");
    }
}
constexpr double kDt = .00125;
constexpr double kDeadSpot = 81. * std::numbers::pi + .5 * std::numbers::pi;  // phase % pi == pi/2
// A valid pedaling config (the ctor's validate() gates the legacy fields)
// with the reflex on and the nine knobs at their documented defaults (D2).
drivetrain::PedalingConfig reposition_config() {
    drivetrain::PedalingConfig c = pedaling_config();
    c.reposition_on_stall = true;
    c.reposition_min_effort_nm = 20.;
    c.reposition_stall_window_s = 1.;
    c.reposition_stall_progress_rad = .5;
    c.reposition_cooldown_s = 1.;
    c.reposition_back_rate_rad_s = 3.;c.reposition_damping_nms_rad = 12.;
    c.reposition_timeout_s = 3.;
    c.reposition_phase_tolerance_rad = .05;
    c.reposition_noop_rad = .12;
    return c;
}
void test_reposition_stall_reflex() {
    using drivetrain::CoastReason;
    {   // rocking about the dead spot with no net advance fires within the window
        drivetrain::PedalingPolicy p(reposition_config());
        const double amp = .26, peak = 4.6, w = peak / amp;
        double first = -1.;
        for (int i = 0; i < 1600 && first < 0.; ++i) {
            const double t = i * kDt;
            const auto s = p.update(kDeadSpot + amp * std::sin(w * t), amp * w * std::cos(w * t), 0., 20., kDt);
            if (s.reason == CoastReason::reposition_stall) first = t;
        }
        bt::require(first >= 0. && first <= 1.05, "rocking stall triggers the reflex within the window");
    }
    {   // a frozen crank on the dead spot is a stall; the goal is the lower power phase
        drivetrain::PedalingPolicy p(reposition_config());
        std::optional<drivetrain::PedalingState> hit;
        for (int i = 0; i < 1200 && !hit; ++i) {
            const auto s = p.update(kDeadSpot, 0., 0., 20., kDt);
            if (drivetrain::is_reposition(s.reason)) hit = s;
        }
        bt::require(hit.has_value(), "static dead-spot stall triggers");
        const auto hit_state = hit.value_or(drivetrain::PedalingState{});
        bt::require(hit_state.mode == drivetrain::PedalMode::coasting && hit_state.effort_nm == 0., "manoeuvre reports coasting, zero effort");
        bt::require(hit_state.target_rate_rad_s < 0., "manoeuvre backpedals");
        bt::require(p.state().reposition && p.state().reposition->goal_rad &&
                    std::abs(*p.state().reposition->goal_rad - 81. * std::numbers::pi) < 1e-9,
                    "goal is the nearest lower power phase");
        // Revision 2026-11-05: the manoeuvre's crank-rate target stays
        // negative even when the wheel-implied cadence and the open-clutch
        // shaft floor would hold ordinary pedaling at the shaft speed.
        const auto s = p.update(kDeadSpot, -1.5, 120., 20., kDt, true, false, false, 15.);
        bt::require(drivetrain::is_reposition(s.reason) && s.target_rate_rad_s < 0.,
                    "manoeuvre target stays negative regardless of required_cadence_rpm");
    }
    {   // slow forward grind (~1 rad/s) is progress
        drivetrain::PedalingPolicy p(reposition_config());
        for (int i = 0; i < 4000; ++i) {
            const double t = i * kDt;
            bt::require(!drivetrain::is_reposition(p.update(kDeadSpot + t, 1., 9.5, 20., kDt).reason), "no reflex on a grind");
        }
    }
    {   // a crank already near a power phase is a no-op
        drivetrain::PedalingPolicy p(reposition_config());
        const auto s = p.update(81. * std::numbers::pi + .05, 0., 0., 20., kDt, true, false, true);
        bt::require(!drivetrain::is_reposition(s.reason), "request inside the no-op band does nothing");
    }
    {   // braking aborts and starts the cooldown; a held request does not retrigger
        drivetrain::PedalingPolicy p(reposition_config());
        auto s = p.update(kDeadSpot, 0., 0., 20., kDt, true, false, true);
        bt::require(s.reason == CoastReason::reposition_requested, "request starts the manoeuvre");
        s = p.update(kDeadSpot, 0., 0., 20., kDt, true, true, true);
        bt::require(!drivetrain::is_reposition(s.reason), "braking aborts");
        for (int i = 0; i < 1000; ++i)
            bt::require(!drivetrain::is_reposition(p.update(kDeadSpot, 0., 0., 20., kDt, true, false, true).reason),
                        "held request + cooldown never retrigger");
    }
}
// R4 (2026-11-03 crank-reposition plan): the manoeuvre is bounded — it
// ends on goal, timeout or abort, starts the cooldown, and normal
// pedaling resumes with the effort slewing back from zero.
void test_reposition_lifecycle() {
    using drivetrain::CoastReason;
    {   // reaching the goal ends the manoeuvre: reason and goal clear,
        // the cooldown starts, and effort slews back from zero rather
        // than stepping to the target
        auto config = reposition_config(); config.effort_slew_nm_s = 300.;
        drivetrain::PedalingPolicy p(config);
        double phase = kDeadSpot;
        auto s = p.update(phase, 0., 0., 20., kDt, true, false, true);
        bt::require(s.reason == CoastReason::reposition_requested && s.target_rate_rad_s < 0.,
                    "the request engages the manoeuvre");
        for (int i = 0; i < 4000 && drivetrain::is_reposition(s.reason); ++i) {
            phase += s.target_rate_rad_s * kDt;
            s = p.update(phase, s.target_rate_rad_s, 0., 20., kDt, true, false, false);
        }
        bt::require(!drivetrain::is_reposition(s.reason) && s.mode == PedalMode::pedaling,
                    "reaching the goal resumes normal pedaling");
        bt::require(p.state().reposition && !p.state().reposition->goal_rad &&
                    p.state().reposition->reason == 0 &&
                    p.state().reposition->cooldown_s > 0.,
                    "completion clears the goal and starts the cooldown");
        bt::require(s.effort_nm > 0. && s.effort_nm <= 300. * kDt + 1e-12,
                    "effort slews back from zero, one slew step on the resume tick");
        const double resumed = s.effort_nm;
        s = p.update(phase, s.target_rate_rad_s, 0., 20., kDt, true, false, false);
        bt::require(s.effort_nm > resumed,
                    "the effort ramp continues slewing on later ticks");
    }
    {   // a crank that never reaches the goal ends on the timeout and
        // starts the same cooldown
        drivetrain::PedalingPolicy p(reposition_config());
        auto s = p.update(kDeadSpot, 0., 0., 20., kDt, true, false, true);
        bt::require(drivetrain::is_reposition(s.reason), "the request engages");
        int ticks = 0;
        for (; ticks < 4000 && drivetrain::is_reposition(s.reason); ++ticks)
            s = p.update(kDeadSpot, 0., 0., 20., kDt, true, false, false);
        bt::require(ticks * kDt > 2.5 && ticks * kDt <= 3.5,
                    "the manoeuvre holds until the 3 s timeout");
        bt::require(s.mode == PedalMode::pedaling,
                    "timeout resumes normal pedaling");
        bt::require(p.state().reposition && p.state().reposition->cooldown_s > 0.,
                    "timeout starts the cooldown");
    }
    {   // the cooldown swallows a press; a release+press after expiry
        // re-arms a fresh manoeuvre
        drivetrain::PedalingPolicy p(reposition_config());
        double phase = kDeadSpot;
        auto s = p.update(phase, 0., 0., 20., kDt, true, false, true);
        for (int i = 0; i < 4000 && drivetrain::is_reposition(s.reason); ++i) {
            phase += s.target_rate_rad_s * kDt;
            s = p.update(phase, s.target_rate_rad_s, 0., 20., kDt, true, false, false);
        }
        bt::require(p.state().reposition && p.state().reposition->cooldown_s > 0.,
                    "completion starts the cooldown");
        s = p.update(phase, 0., 0., 20., kDt, true, false, true);
        bt::require(!drivetrain::is_reposition(s.reason),
                    "a press inside the cooldown is consumed");
        for (int i = 0; i < 1000 && p.state().reposition &&
             p.state().reposition->cooldown_s > 0.; ++i)
            s = p.update(phase, 0., 0., 20., kDt, true, false, false);
        const auto &cooled = p.state().reposition;
        bt::require(cooled && cooled->cooldown_s <= 0., "the cooldown expires");
        // The crank finished inside the no-op band of the last power
        // phase; a fresh request at the next dead spot earns a new
        // manoeuvre.
        phase = 83. * std::numbers::pi + .5 * std::numbers::pi;
        s = p.update(phase, 0., 0., 20., kDt, true, false, true);
        bt::require(s.reason == CoastReason::reposition_requested && cooled &&
                    cooled->goal_rad &&
                    std::abs(*cooled->goal_rad - 83. * std::numbers::pi) < 1e-9,
                    "a fresh press after expiry re-arms the manoeuvre");
    }
    {   // a manual request works without the stall reflex; once the
        // cooldown lapses the engaged snapshot disengages entirely
        auto config = reposition_config(); config.reposition_on_stall = false;
        drivetrain::PedalingPolicy p(config);
        double phase = kDeadSpot;
        auto s = p.update(phase, 0., 0., 20., kDt, true, false, true);
        bt::require(s.reason == CoastReason::reposition_requested,
                    "the manual request engages without the reflex");
        for (int i = 0; i < 4000 && drivetrain::is_reposition(s.reason); ++i) {
            phase += s.target_rate_rad_s * kDt;
            s = p.update(phase, s.target_rate_rad_s, 0., 20., kDt, true, false, false);
        }
        bt::require(!drivetrain::is_reposition(s.reason), "the manoeuvre completes");
        for (int i = 0; i < 1000 && p.state().reposition; ++i)
            s = p.update(phase, 0., 0., 20., kDt, true, false, false);
        bt::require(!p.state().reposition.has_value(),
                    "the reposition snapshot disengages after the cooldown");
    }
    {   // a request held across a disable/enable toggle stays consumed:
        // only a fresh release+press earns the manoeuvre
        drivetrain::PedalingPolicy p(reposition_config());
        auto s = p.update(kDeadSpot, 0., 0., 20., kDt, false, false, true);
        bt::require(s.mode == PedalMode::disabled && s.reason == CoastReason::disabled,
                    "the disabled path wins over the request");
        s = p.update(kDeadSpot, 0., 0., 20., kDt, true, false, true);
        bt::require(!drivetrain::is_reposition(s.reason),
                    "a request held through the toggle does not retrigger");
        s = p.update(kDeadSpot, 0., 0., 20., kDt, true, false, false);
        s = p.update(kDeadSpot, 0., 0., 20., kDt, true, false, true);
        bt::require(s.reason == CoastReason::reposition_requested,
                    "a fresh press after release re-arms");
    }
    {   // a negative crank phase folds through fmod into the same band;
        // the goal stays the nearest lower power phase
        drivetrain::PedalingPolicy p(reposition_config());
        const double phase = -.5 * std::numbers::pi;
        const auto s = p.update(phase, 0., 0., 20., kDt, true, false, true);
        bt::require(s.reason == CoastReason::reposition_requested && s.target_rate_rad_s < 0.,
                    "negative phase engages the backpedal");
        const auto &neg = p.state().reposition;
        bt::require(neg && neg->goal_rad &&
                    std::abs(*neg->goal_rad + std::numbers::pi) < 1e-9,
                    "the goal is the nearest lower power phase");
    }
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
    cfg::PedalingConfig source; source.enabled = true; source.coast_cadence_tau_s = -.1; source.stop_time_s = .35;
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
void test_open_clutch_never_wishes_below_the_shaft() {
    // Plan 2026-11-05-clutch-pedaling-fixes task 2 (Р2): the crank-rate
    // wish is the ceiling the rider spins up to, not a target the crank is
    // braked down to. While the one-way motor clutch is open the shaft
    // outruns the crank, and a wish below it holds the crank back and
    // starves the legs — the wish meets the shaft so the crank comes up
    // and the clutch re-engages.
    const double crank = 60. * rpm_to_rad_s, shaft = 110. * rpm_to_rad_s;
    PedalingPolicy policy(pedaling_config(.35));
    const auto open = policy.update(0., crank, 110., 30., .001, true, false, false, shaft);
    bt::require(open.mode == PedalMode::pedaling, "the lagging crank keeps pedaling");
    bt::require(open.target_rate_rad_s >= shaft,
                "the wish is not below the shaft while the clutch is open");
    // The backdriven stall signature: a crank ringing negative still gets a
    // wish at the shaft, not its own slowed EMA.
    const auto backdriven = policy.update(0., -4.5, 110., 30., .001, true, false, false, shaft);
    bt::require(backdriven.target_rate_rad_s >= shaft,
                "a backward-ringing crank still meets the shaft");
    // Rigid link (the shaft arg absent) and an engaged clutch (shaft not
    // faster than the crank) keep the min(required, ema) wish — the floor
    // never engages there.
    const auto rigid = PedalingPolicy(pedaling_config(.35)).update(0., crank, 110., 30., .001);
    drive_reference::near(rigid.target_rate_rad_s, crank, "rigid link keeps the ema wish");
    const auto engaged = PedalingPolicy(pedaling_config(.35))
                             .update(0., crank, 110., 30., .001, true, false, false, crank);
    drive_reference::near(engaged.target_rate_rad_s, crank, "engaged clutch keeps the ema wish");
    // Coasting keeps its own decaying target — the floor is pedaling-only.
    PedalingPolicy coasting(pedaling_config(.35));
    drivetrain::PedalingState state;
    for (int i = 0; i < 1500; ++i) state = coasting.update(0., 160. * rpm_to_rad_s, 160., 30., .001);
    bt::require(state.mode == PedalMode::coasting, "sustained cadence coasts");
    const auto hold = coasting.update(0., 0., 160., 30., .001, true, false, false, shaft);
    bt::require(hold.mode == PedalMode::coasting && hold.target_rate_rad_s < shaft,
                "coasting keeps its decaying target below the shaft");
}
} // namespace
int main() { return bt::run([] {
    test_reposition_knobs_exist_and_mode_domain_is_unchanged();
    test_stall_detector();
    test_reposition_stall_reflex();
    test_reposition_lifecycle();
    test_a_rocking_loaded_crank_simply_keeps_pedalling();
    test_ride_control_has_no_reposition_field();
    test_rider_program_rejects_the_removed_reposition_command();
    test_coast_decision_filters_a_single_stroke_spike_with_ema();
    test_zero_tau_keeps_the_raw_hysteresis();
    test_coast_tau_must_be_nonnegative();
    test_wheel_implied_cadence_does_not_stop_legs_below_their_limit();
    test_stopped_cranks_resume_while_the_open_freehub_wheel_keeps_turning();
    test_open_clutch_never_wishes_below_the_shaft();
}); }
