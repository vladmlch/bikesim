// Launch part 3b — the launch plan inside RiderIntent: the switched-off
// plan leaves the part-3a path bitwise, a standing start holds the brakes
// from step 0 through the first percept, the plan's effort and lean go
// through the intent with the reaction on top, the handover is continuous
// and within the lean rate, surge is not spent while the plan owns,
// ownership follows the caller, and the joint state restores.
#include "support/fixture_test.hpp"
#include "configuration/model_config.hpp"
#include "io/json.hpp"
#include "rider/front_lift_reaction.hpp"
#include "rider/intent.hpp"
#include "rider/launch_plan.hpp"
#include "rider/state_codec.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace {

constexpr double kDt = .00125;

template <typename Action>
void rejects(Action &&action, std::string_view needle, std::string_view what) {
    bool threw = false;
    try {
        std::forward<Action>(action)();
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(std::string_view(failure.what()).find(needle) != std::string_view::npos,
                    std::string(what) + " names " + std::string(needle) + " (" + failure.what() + ")");
    }
    bt::require(threw, std::string(what) + " is rejected");
}

// The working profile's intent (period = dt, delay 120 ticks) and a
// switched-off reaction; copy the field values from enabled_intent() in
// test_front_lift_reaction.cpp.
rider::IntentConfig intent_config() {
    auto config = bike::configuration::default_physics().seated_climb;
    config.enabled = true;
    return config;
}
rider::FrontLiftReactionConfig switched_off_reaction() {
    return {.hand_unload_n = 20., .unload_dwell_s = .05, .pitch_rate_up_rad_s = .5, .pitch_rad = .17,
            .decision_s = .1, .ease_enabled = false, .lean_enabled = false, .rear_brake_tap_enabled = false,
            .ease_fraction = .7, .ease_rate_per_s = 5., .lean_add_rad = .1, .lean_rate_rad_s = .5,
            .clear_s = .3};
}
// The preliminary launch table of decision 2 with plan_enabled as given.
rider::LaunchConfig launch_table(bool enabled) {
    rider::LaunchConfig t;
    t.start_cog_teeth = {51};
    t.start_pedal_angle_rad = 0.;
    t.start_lean_grades = {0.};
    t.start_lean_rad = {0.};
    t.plan_enabled = enabled;
    t.standstill_speed_mps = .1;
    t.standstill_crank_rpm = 5.;
    t.effort_rate_nm_s = 40.;
    t.effort_cap_asphalt_nm = t.effort_cap_hardpack_nm = t.effort_cap_loose_nm = t.effort_cap_wet_nm =
        t.effort_cap_scree_nm = 60.;
    t.release_torque_grades = {0., .2};
    t.release_torque_nm = {0., 30.};
    t.brake_release_s = .15;
    t.max_hold_s = 1.5;
    t.upshift_crank_rpm = 82.;
    t.end_speed_mps = 3.;
    t.end_crank_rpm = 60.;
    t.end_hold_s = .3;
    t.max_launch_s = 6.;
    t.handover_s = .9;
    return t;
}

rider::IntentControl automatic() { return {}; }

struct Tick {
    rider::IntentResolution resolution;
    rider::RiderCommands commands;
};

// One physics step: the bike stands, the legs press `torque` against the brake.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (k, torque) ordered by usage
Tick step(rider::RiderIntent &intent, std::int64_t k, double torque, const rider::LaunchContext &ctx,
          // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (speed, crank_rate) ordered by usage
          rider::IntentControl control = automatic(), double speed = 0., double crank_rate = 0.) {
    rider::IntentSignals signals{};
    signals.human_crank_torque_nm = torque;
    signals.crank_rate_rad_s = crank_rate;
    signals.specific_force_body_mps2 = {0., 0., 9.81};
    rider::RiderSenses senses{};
    senses.speed_mps = speed;
    senses.rear_teeth = 51;
    senses.surface_front = rider::SurfaceLabel::hardpack;
    senses.surface_ahead = rider::SurfaceLabel::hardpack;
    senses.grade_ahead = .15;
    const auto r =
        intent.resolve_full(control, signals, senses, k, .15, .15, std::nullopt, true, true, std::nullopt, ctx);
    return Tick{.resolution = r, .commands = intent.commands()};
}

const rider::LaunchContext kEffort{.shift = {}, .effort_mode = true, .external_brake = false};

std::string encoded(const rider::RiderIntent &intent) {
    return bike::io::canonical_json(rider::encode_intent_state(intent.state()));
}

// A switched-off plan: commands stay the empty set and the resolution is
// bitwise the one of an intent whose plan cannot arm (part-3a path).
[[gnu::noinline]] void test_switched_off_plan_is_the_part_3a_path() {
    auto off = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(false), kDt);
    auto ref = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(false), kDt);
    bt::require(!off->arm_standstill_start(.1), "a switched-off plan does not arm");
    for (std::int64_t k = 0; k < 600; ++k) {
        const auto a = step(*off, k, 20., kEffort);
        const auto b = step(*ref, k, 20., rider::LaunchContext{});
        bt::require(a.commands.front_brake == 0. && a.commands.rear_brake == 0. && !a.commands.owns_shifting &&
                        !a.commands.shift_request,
                    "a switched-off plan commands nothing");
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        const double ea = *a.resolution.control.human_torque_nm;
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        const double eb = *b.resolution.control.human_torque_nm;
        bt::require(std::bit_cast<std::uint64_t>(ea) == std::bit_cast<std::uint64_t>(eb) &&
                        a.resolution.effort_limited == b.resolution.effort_limited,
                    "the effort is the part-3a one");
        bt::require(off->launch().phase == rider::LaunchPhase::idle, "the plan stays idle");
    }
}

// A standing start: brake 1 from step 0 (before the first tick resolves)
// through the first percept, no press before the first percept, then the
// press ramps; the posture holds the prepared lean.
[[gnu::noinline]] void test_standing_start_holds_from_step_zero() {
    auto intent = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    intent->seed_lean(.08);
    bt::require(intent->arm_standstill_start(.08), "a switched-on plan arms at a standing start");
    bt::require(intent->commands().front_brake == 1. && intent->commands().rear_brake == 1. &&
                    intent->commands().owns_shifting,
                "the brakes are held before the first tick");
    const std::int64_t delay = std::llround(intent_config().reaction_delay_s / intent_config().period_s);
    for (std::int64_t k = 0; k <= delay; ++k) {
        const auto t = step(*intent, k, 0., kEffort);
        bt::require(t.commands.front_brake == 1. && t.commands.rear_brake == 1., "held through the first percept");
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        bt::require(k == delay || *t.resolution.control.human_torque_nm == 0., "no press before the first percept");
        bt::require(t.resolution.effort_limited, "the plan owns the effort");
    }
    double effort = 0.;
    for (std::int64_t k = delay + 1; k < delay + 200; ++k) {
        const auto t = step(*intent, k, 0., kEffort);
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        const double now = *t.resolution.control.human_torque_nm;
        bt::require(now >= effort && t.commands.front_brake == 1., "the press ramps while holding");
        effort = now;
    }
    bt::require(effort > 0., "the press has started");
}

// The handover is continuous: effort and lean change per tick no more
// than the ramp and the lean rate allow; surge is not spent while the plan
// owns; at done the effort is the ordinary policy's.
[[gnu::noinline]] void test_handover_is_continuous_and_surge_is_held() {
    auto intent = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    bt::require(intent->arm_standstill_start(0.), "armed");
    const double budget = intent->policy().state().surge_budget_s_left;
    double effort = 0., lean = 0.;
    bool done = false;
    for (std::int64_t k = 0; k < 20000 && !done; ++k) {
        // The legs press 40 N·m (above the 15 % release value), then the bike
        // rolls at 4 m/s and 70 rpm so the end criteria hold.
        const bool moving = k > 2000;
        const auto t = step(*intent, k, 40., kEffort, automatic(), moving ? 4. : 0., moving ? 7.33 : 0.);
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        const double now = *t.resolution.control.human_torque_nm;
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the posture
        const double now_lean = t.resolution.control.posture->torso_lean_rad;
        if (k > 0) {
            bt::require(std::abs(now_lean - lean) <= intent_config().lean_rate_rad_s * kDt + 1e-12,
                        "the lean never moves faster than the lean rate");
            bt::require(std::abs(now - effort) <= 60. * 1.001, "the effort has no jump beyond one cap");
        }
        effort = now;
        lean = now_lean;
        if (intent->launch().weight > 0.)
            bt::require(intent->policy().state().surge_budget_s_left == budget, "surge is held while the plan owns");
        done = intent->launch().phase == rider::LaunchPhase::done;
    }
    bt::require(done, "the launch reaches done");
}

// Ownership: a caller effort cancels the plan (brakes 0, no shifting); a
// posture-only override does not; rider disabled cancels.
[[gnu::noinline]] void test_ownership_follows_the_caller() {
    auto intent = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    bt::require(intent->arm_standstill_start(0.), "armed");
    auto posture_only = automatic();
    posture_only.posture = rider::RiderPosture{};
    posture_only.posture->use_saddle = true;
    auto t = step(*intent, 0, 0., kEffort, posture_only);
    bt::require(t.commands.front_brake == 1. && intent->launch().phase == rider::LaunchPhase::hold,
                "a posture-only override keeps the plan");
    auto effort = automatic();
    effort.human_torque_nm = 10.;
    t = step(*intent, 1, 0., kEffort, effort);
    bt::require(t.commands.front_brake == 0. && !t.commands.owns_shifting &&
                    intent->launch().phase == rider::LaunchPhase::idle,
                "a caller effort cancels the plan");
    bt::require(intent->arm_standstill_start(0.), "re-armed");
    auto disabled = automatic();
    disabled.rider_enabled = false;
    t = step(*intent, 2, 0., kEffort, disabled);
    bt::require(t.commands.front_brake == 0. && intent->launch().phase == rider::LaunchPhase::idle,
                "a disabled rider cancels the plan");
    bt::require(intent->arm_standstill_start(0.), "re-armed again");
    t = step(*intent, 3, 0., rider::LaunchContext{.shift = {}, .effort_mode = false, .external_brake = false});
    bt::require(t.commands.front_brake == 0., "a non-effort drive mode cancels the plan");
}

// State: the document carries the plan, a restored copy continues bitwise
// (commands included), a probe changes nothing, and the joint rule
// rejects a plan clock ahead of the delivered percepts.
[[gnu::noinline]] void test_state_round_trip_probe_and_joint_rule() {
    auto a = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    bt::require(a->arm_standstill_start(0.), "armed");
    for (std::int64_t k = 0; k < 300; ++k) (void)step(*a, k, 5., kEffort);
    const std::string before = encoded(*a);
    rider::IntentSignals signals{};
    signals.specific_force_body_mps2 = {0., 0., 9.81};
    rider::RiderSenses senses{};
    senses.rear_teeth = 51;
    senses.surface_front = rider::SurfaceLabel::hardpack;
    senses.surface_ahead = rider::SurfaceLabel::hardpack;
    (void)a->resolve_full(automatic(), signals, senses, 300, .15, .15, std::nullopt, true, false, std::nullopt, kEffort);
    bt::require(encoded(*a) == before, "a probe changes no resolver state");
    auto b = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    b->restore(rider::decode_intent_state(rider::encode_intent_state(a->state()), "intent"));
    bt::require(encoded(*b) == encoded(*a) && b->commands().front_brake == a->commands().front_brake,
                "the document restores the plan and its commands");
    for (std::int64_t k = 300; k < 600; ++k) {
        const auto x = step(*a, k, 15., kEffort);
        const auto y = step(*b, k, 15., kEffort);
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        const double ex = *x.resolution.control.human_torque_nm;
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) the enabled intent always fills the effort
        const double ey = *y.resolution.control.human_torque_nm;
        bt::require(std::bit_cast<std::uint64_t>(ex) == std::bit_cast<std::uint64_t>(ey) &&
                        x.commands.front_brake == y.commands.front_brake,
                    "the restored intent continues bitwise");
    }
    auto fresh = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), launch_table(true), kDt);
    const std::string clean = encoded(*fresh);
    auto bad = std::make_unique<rider::RiderIntent::State>(a->state());
    bad->last_tick_step = -1;
    bad->perception = fresh->state().perception;
    bad->reaction = fresh->state().reaction;
    rejects([&] { fresh->restore(*bad); }, "launch", "a plan clock ahead of the delivered percepts");
    *bad = a->state();
    bad->launch.reset();
    rejects([&] { fresh->restore(*bad); }, "launch", "an enabled intent without plan state");
    bt::require(encoded(*fresh) == clean, "rejected restores change nothing");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_switched_off_plan_is_the_part_3a_path();
        test_standing_start_holds_from_step_zero();
        test_handover_is_continuous_and_surge_is_held();
        test_ownership_follows_the_caller();
        test_state_round_trip_probe_and_joint_rule();
    });
}
