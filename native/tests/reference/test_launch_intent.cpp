// Launch part 3b — the launch plan inside RiderIntent: the switched-off
// plan leaves the part-3a path bitwise, a standing start holds the brakes
// from step 0 through the first percept, the plan's effort and lean go
// through the intent with the reaction on top, the handover is continuous
// and within the lean rate, surge is not spent while the plan owns,
// ownership follows the caller, and the joint state restores.
#include "support/fixture_test.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "io/json.hpp"
#include "rider/front_lift_reaction.hpp"
#include "rider/intent.hpp"
#include "rider/launch_plan.hpp"
#include "rider/state_codec.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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
    return {.hand_window_s = .5, .hand_lag_s = .5, .hand_drop_ratio = .7, .pitch_rate_up_rad_s = .5, .pitch_rad = .17,
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
    t.effort_cap_grades = {0.};
    t.effort_cap_grade_nm = {1000.};
    t.effort_rise_speed_mps = 2.5;
    t.release_torque_grades = {0., .2};
    t.release_torque_nm = {0., 30.};
    t.brake_release_s = .15;
    t.max_hold_s = 1.5;
    t.upshift_crank_rpm = 82.;
    t.shift_retry_s = .25;
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

// Stationary launch-gear selection in LaunchPhase::hold:
// - validate(RiderCommands) rejects invalid stationary_rear_teeth combinations;
// - cold start leaves stationary_rear_teeth unset;
// - mid-ride stop in hold with grade_ahead > 0 & rear_teeth < target sets
//   stationary_rear_teeth, state()/restore() preserves it, release clears it,
//   and grade_ahead <= 0 or rear_teeth >= target leaves it unset.
[[gnu::noinline]] void test_stationary_launch_gear_selection_in_hold() {
    // 1) RiderCommands validation.
    rider::validate(rider::RiderCommands{.owns_shifting = true, .stationary_rear_teeth = 39});
    rejects([] { rider::validate(rider::RiderCommands{.stationary_rear_teeth = 39}); },
            "owns_shifting", "stationary_rear_teeth without ownership");
    rejects([] { rider::validate(rider::RiderCommands{.owns_shifting = true, .stationary_rear_teeth = 2}); },
            "at least 3", "stationary_rear_teeth below 3");
    rejects([] {
        rider::validate(rider::RiderCommands{.shift_request = rider::ShiftRequest::up,
                                             .owns_shifting = true,
                                             .stationary_rear_teeth = 39});
    }, "conflicts", "shift_request with stationary_rear_teeth");

    // Configure launch table with grade-indexed launch cogs: <3% -> 24, <8% -> 33, >=8% -> 39.
    auto table = launch_table(true);
    table.start_cog_grade_breaks = {.03, .08};
    table.start_cog_teeth = {24, 33, 39};

    struct SensesInput {
        double speed = 0.;
        double crank_rate = 0.;
        int rear_teeth = 12;
        double grade_ahead = 0.;
        double torque = 0.;
    };
    const auto tick_senses = [](rider::RiderIntent &intent, std::int64_t k,
                                const SensesInput &in) {
        rider::IntentSignals signals{};
        signals.human_crank_torque_nm = in.torque;
        signals.crank_rate_rad_s = in.crank_rate;
        signals.specific_force_body_mps2 = {0., 0., 9.81};
        rider::RiderSenses senses{};
        senses.speed_mps = in.speed;
        senses.rear_teeth = in.rear_teeth;
        senses.surface_front = rider::SurfaceLabel::asphalt;
        senses.surface_ahead = rider::SurfaceLabel::asphalt;
        senses.grade_ahead = in.grade_ahead;
        (void)intent.resolve_full(automatic(), signals, senses, k, in.grade_ahead,
                                  in.grade_ahead, std::nullopt, true, true,
                                  std::nullopt, kEffort);
        return intent.commands();
    };

    // 2) Cold start at t=0 leaves stationary_rear_teeth == nullopt.
    auto intent = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), table, kDt);
    bt::require(intent->arm_standstill_start(0.), "armed cold start");
    bt::require(!intent->commands().stationary_rear_teeth.has_value(),
                "cold start at t=0 leaves stationary_rear_teeth unset");

    // Advance through cold start until done (cruising at 4 m/s, 70 rpm in 12T).
    const std::int64_t delay = std::llround(intent_config().reaction_delay_s / intent_config().period_s);
    std::int64_t k = 0;
    for (; k < 5000 && intent->launch().phase != rider::LaunchPhase::done; ++k) {
        const bool moving = k > delay + 50;
        (void)tick_senses(*intent, k, {.speed = moving ? 4. : 0.,
                                       .crank_rate = moving ? 7.33 : 0.,
                                       .rear_teeth = 12,
                                       .grade_ahead = .10,
                                       .torque = 40.});
    }
    bt::require(intent->launch().phase == rider::LaunchPhase::done, "cold start reaches done");

    // 3) Mid-ride stop on 10% in 12T -> hold sets stationary_rear_teeth = 39.
    bool saw_hold = false;
    for (int i = 0; i < delay + 20 && !saw_hold; ++i, ++k) {
        const auto cmds = tick_senses(*intent, k, {.rear_teeth = 12, .grade_ahead = .10});
        if (intent->launch().phase == rider::LaunchPhase::hold) {
            saw_hold = true;
            bt::require(cmds.owns_shifting && cmds.stationary_rear_teeth == std::optional{39},
                        "mid-ride hold on 10% in 12T requests stationary_rear_teeth = 39");
        }
    }
    bt::require(saw_hold, "mid-ride stop enters hold");

    // 4) state()/restore() preserves stationary_rear_teeth.
    auto restored = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), table, kDt);
    restored->restore(rider::decode_intent_state(rider::encode_intent_state(intent->state()), "intent"));
    bt::require(restored->commands().stationary_rear_teeth == std::optional{39},
                "state/restore preserves stationary_rear_teeth in hold");

    // 5) Entering release clears stationary_rear_teeth.
    bool saw_release = false;
    for (std::int64_t i = 0; i < delay + 20 && !saw_release; ++i, ++k) {
        const auto cmds =
            tick_senses(*intent, k, {.rear_teeth = 39, .grade_ahead = .10, .torque = 40.});
        if (intent->launch().phase == rider::LaunchPhase::release ||
            intent->launch().phase == rider::LaunchPhase::accelerate) {
            saw_release = true;
            bt::require(!cmds.stationary_rear_teeth.has_value(),
                        "leaving hold clears stationary_rear_teeth");
        }
    }
    bt::require(saw_release, "hold advances into release");

    // 6) grade_ahead <= 0 or rear_teeth >= target leaves stationary_rear_teeth == nullopt.
    for (const auto &[teeth, grade, label] : {
             std::tuple{12, 0., "flat grade_ahead == 0"},
             std::tuple{12, -.05, "downhill grade_ahead < 0"},
             std::tuple{39, .10, "already on target cog 39 >= 39"},
             std::tuple{45, .10, "already on lighter cog 45 >= 39"}}) {
        auto check = std::make_unique<rider::RiderIntent>(intent_config(), switched_off_reaction(), table, kDt);
        bool saw_case_hold = false;
        for (std::int64_t step_k = 0; step_k <= delay + 5 && !saw_case_hold; ++step_k) {
            const auto cmds =
                tick_senses(*check, step_k, {.rear_teeth = teeth, .grade_ahead = grade});
            if (check->launch().phase == rider::LaunchPhase::hold) {
                saw_case_hold = true;
                bt::require(!cmds.stationary_rear_teeth.has_value(),
                            std::string(label) + ": leaves stationary_rear_teeth unset");
            }
        }
        bt::require(saw_case_hold, std::string(label) + ": enters hold");
    }
}

// Queue Item 2: speed+grade seated-climb effort cap interpolation, speed ramp,
// saturation at max_crank_torque_nm, and inert 1000.0 bitwise equivalence.
struct ClimbCapQuery {
    double grade_ahead = 0.;
    double speed_mps = 0.;
    double crank_rate_rad_s = 3.;
};

[[gnu::noinline]] rider::IntentResolution resolve_climb_cap_at(
    const rider::IntentConfig &cfg, const ClimbCapQuery &q) {
    auto intent = std::make_unique<rider::RiderIntent>(
        cfg, switched_off_reaction(), launch_table(false), kDt);
    const std::int64_t delay =
        std::llround(cfg.reaction_delay_s / cfg.period_s);
    rider::IntentResolution out{};
    for (std::int64_t k = 0; k <= delay + 2; ++k) {
        rider::IntentSignals signals{};
        signals.human_crank_torque_nm = 60.;
        signals.crank_rate_rad_s = q.crank_rate_rad_s;
        signals.specific_force_body_mps2 = {0., 0., 9.81};
        rider::RiderSenses senses{};
        senses.speed_mps = q.speed_mps;
        senses.rear_teeth = 39;
        senses.surface_front = rider::SurfaceLabel::hardpack;
        senses.surface_ahead = rider::SurfaceLabel::hardpack;
        senses.grade_ahead = q.grade_ahead;
        out = intent->resolve_full(automatic(), signals, senses, k,
                                   q.grade_ahead, q.grade_ahead, std::nullopt,
                                   true, true, std::nullopt, kEffort);
    }
    return out;
}

[[gnu::noinline]] bike::io::Value to_seated_doc(const rider::IntentConfig &c) {
    auto phys = std::make_unique<bike::configuration::SimulationPhysicsConfig>(
        bike::configuration::default_physics());
    phys->seated_climb = c;
    const auto doc = bike::configuration::detail::plain_document(
        bike::configuration::to_value(*phys));
    for (const auto &[k, v] : std::get<bike::io::Value::Object>(doc.data))
        if (k == "seated_climb") return v;
    throw std::runtime_error("missing seated_climb in serialized config");
}

[[gnu::noinline]] void test_seated_climb_effort_cap() {
    auto cfg = intent_config();
    cfg.max_crank_torque_nm = 60.;
    cfg.target_crank_power_w = 250.;
    cfg.climb_effort_cap_grades = {0., .10, .20};
    cfg.climb_effort_cap_grade_nm = {45., 30., 35.};
    cfg.climb_effort_rise_speed_mps = 2.5;
    cfg.climb_effort_full_speed_mps = 4.;

    // 1) Piecewise-linear grade interpolation and endpoint clamping at v <= rise.
    // Use crank_rate = 3.0 rad/s -> power/rate = 83.3 Nm -> unconstrained 60 Nm.
    for (const auto &[grade, speed, expected_nm, label] : {
             std::tuple{-.05, 1., 45., "grade below first knot clamps"},
             std::tuple{0., 0., 45., "first grade knot"},
             std::tuple{.05, 2.5, 37.5, "midpoint interpolation at rise speed"},
             std::tuple{.10, 2.0, 30., "interior grade knot below rise"},
             std::tuple{.15, 2.5, 32.5, "second segment midpoint"},
             std::tuple{.20, 1.5, 35., "last grade knot"},
             std::tuple{.30, 1.5, 35., "grade above last knot clamps"},
             // 2) Linear speed ramp across [2.5, 4.0] toward max_crank_torque_nm (60).
             std::tuple{.10, 3.25, 45., "halfway speed ramp 30 -> 60 = 45"},
             std::tuple{.10, 3.7, 54., "80% speed ramp 30 -> 60 = 54"},
             // 3) Saturation at and above full_speed_mps.
             std::tuple{.10, 4.0, 60., "at full_speed_mps reaches max_crank_torque_nm"},
             std::tuple{.10, 6.0, 60., "above full_speed_mps saturates at max_crank_torque_nm"},
         }) {
        const auto res = resolve_climb_cap_at(
            cfg, {.grade_ahead = grade, .speed_mps = speed, .crank_rate_rad_s = 3.});
        bt::require(res.control.human_torque_nm.has_value() &&
                        std::abs(*res.control.human_torque_nm - expected_nm) < 1e-12,
                    std::string(label) + ": got " +
                        std::to_string(res.control.human_torque_nm.value_or(-1.)));
        bt::require(!res.effort_limited,
                    std::string(label) + ": climb cap leaves effort_limited false");
    }

    // Degenerate full_speed == rise_speed steps at speed > rise_speed.
    auto step_cfg = cfg;
    step_cfg.climb_effort_full_speed_mps = step_cfg.climb_effort_rise_speed_mps;
    const auto lo_step = resolve_climb_cap_at(
        step_cfg, {.grade_ahead = .10, .speed_mps = 2.5, .crank_rate_rad_s = 3.});
    const auto hi_step = resolve_climb_cap_at(
        step_cfg, {.grade_ahead = .10, .speed_mps = 2.51, .crank_rate_rad_s = 3.});
    bt::require(lo_step.control.human_torque_nm.has_value() &&
                    std::abs(lo_step.control.human_torque_nm.value() - 30.) < 1e-12,
                "degenerate rise==full holds base cap at rise speed");
    bt::require(hi_step.control.human_torque_nm.has_value() &&
                    std::abs(hi_step.control.human_torque_nm.value() - 60.) < 1e-12,
                "degenerate rise==full unlocks max_crank_torque_nm above rise speed");

    // 4) Inert [1000.0] cap leaves effort_ceiling_nm bitwise identical to unconstrained policy.
    const auto inert_a = intent_config();
    auto inert_b = inert_a;
    inert_b.climb_effort_cap_grades = {0., .25};
    inert_b.climb_effort_cap_grade_nm = {900., 1200.};
    for (const auto &[grade, speed, rate] : {
             std::tuple{.10, 1., 3.},
             std::tuple{.10, 3., 6.},
             std::tuple{.20, 4.5, 4.5},
         }) {
        const auto ra = resolve_climb_cap_at(
            inert_a, {.grade_ahead = grade, .speed_mps = speed, .crank_rate_rad_s = rate});
        const auto rb = resolve_climb_cap_at(
            inert_b, {.grade_ahead = grade, .speed_mps = speed, .crank_rate_rad_s = rate});
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) enabled intent fills human_torque_nm
        const double ea = *ra.control.human_torque_nm;
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) enabled intent fills human_torque_nm
        const double eb = *rb.control.human_torque_nm;
        bt::require(std::bit_cast<std::uint64_t>(ea) == std::bit_cast<std::uint64_t>(eb) &&
                        ra.effort_limited == rb.effort_limited,
                    "inert climb cap is bitwise identical");
    }
}

struct InvalidClimbCapSpec {
    std::string_view needle;
    std::string_view label;
};

[[gnu::noinline]] void check_invalid_climb_cap(
    const rider::IntentConfig &base,
    const std::function<void(rider::IntentConfig &)> &mutate,
    const InvalidClimbCapSpec &spec) {
    auto bad = base;
    mutate(bad);
    rejects([&] {
        (void)rider::RiderIntent(bad, switched_off_reaction(), launch_table(false), kDt);
    }, spec.needle, std::string("RiderIntent rejects ") + std::string(spec.label));
    const auto doc = to_seated_doc(bad);
    rejects([&] { (void)rider::decode_intent_config(doc, "seated_climb"); },
            spec.needle, std::string("decode_intent_config rejects ") + std::string(spec.label));
}

[[gnu::noinline]] void test_seated_climb_effort_cap_validation() {
    auto cfg = intent_config();
    cfg.climb_effort_cap_grades = {0., .10, .20};
    cfg.climb_effort_cap_grade_nm = {45., 30., 35.};
    cfg.climb_effort_rise_speed_mps = 2.5;
    cfg.climb_effort_full_speed_mps = 4.;

    // Validation errors (constructor and decode_intent_config) + round-trip.
    const auto round_tripped =
        rider::decode_intent_config(to_seated_doc(cfg), "seated_climb");
    bt::require(round_tripped.climb_effort_cap_grades == cfg.climb_effort_cap_grades &&
                    round_tripped.climb_effort_cap_grade_nm == cfg.climb_effort_cap_grade_nm &&
                    round_tripped.climb_effort_rise_speed_mps == cfg.climb_effort_rise_speed_mps &&
                    round_tripped.climb_effort_full_speed_mps == cfg.climb_effort_full_speed_mps,
                "decode_intent_config round-trips climb_effort_cap fields");

    const auto check_invalid = [&](const std::function<void(rider::IntentConfig &)> &mutate,
                                   const InvalidClimbCapSpec &spec) {
        check_invalid_climb_cap(cfg, mutate, spec);
    };

    constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    check_invalid([](auto &c) { c.climb_effort_cap_grades.clear(); c.climb_effort_cap_grade_nm.clear(); },
                  {"climb_effort_cap_grades", "empty table"});
    check_invalid([](auto &c) { c.climb_effort_cap_grade_nm.pop_back(); },
                  {"climb_effort_cap_grade_nm", "mismatched table length"});
    check_invalid([](auto &c) { c.climb_effort_cap_grades = {0., .10, .10}; },
                  {"climb_effort_cap_grades", "duplicate grade knots"});
    check_invalid([](auto &c) { c.climb_effort_cap_grades = {.20, .10, .30}; },
                  {"climb_effort_cap_grades", "descending grade knots"});
    check_invalid([&](auto &c) { c.climb_effort_cap_grades[1] = kNan; },
                  {"climb_effort_cap_grades", "NaN grade knot"});
    check_invalid([&](auto &c) { c.climb_effort_cap_grade_nm[0] = 0.; },
                  {"climb_effort_cap_grade_nm", "zero torque cap"});
    check_invalid([&](auto &c) { c.climb_effort_cap_grade_nm[0] = -10.; },
                  {"climb_effort_cap_grade_nm", "negative torque cap"});
    check_invalid([&](auto &c) { c.climb_effort_cap_grade_nm[0] = kInf; },
                  {"climb_effort_cap_grade_nm", "infinite torque cap"});
    check_invalid([&](auto &c) { c.climb_effort_rise_speed_mps = -.1; },
                  {"climb_effort_rise_speed_mps", "negative rise speed"});
    check_invalid([&](auto &c) { c.climb_effort_rise_speed_mps = kNan; },
                  {"climb_effort_rise_speed_mps", "NaN rise speed"});
    check_invalid([&](auto &c) { c.climb_effort_full_speed_mps = c.climb_effort_rise_speed_mps - .01; },
                  {"climb_effort_full_speed_mps", "full speed below rise speed"});
    check_invalid([&](auto &c) { c.climb_effort_full_speed_mps = kInf; },
                  {"climb_effort_full_speed_mps", "infinite full speed"});
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
        test_stationary_launch_gear_selection_in_hold();
        test_seated_climb_effort_cap();
        test_seated_climb_effort_cap_validation();
    });
}
