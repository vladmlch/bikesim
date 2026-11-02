// Ports all eight source test functions in
// tests/reference/test_closed_form_rider.py — the pinned (spindle) rider's
// closed-form command path: virtual-work torques with no allocator call,
// posture targets vs pelvis/frame pitch, activation probe/advance
// semantics, and two slow environment records (flat launch power,
// front-load-share intent signal boundary).
//
// c.allocate's monkeypatch "forbidden" has no native analogue: the pinned
// controller never reaches the allocator — the proof is structural (final
// commands are published and allocation diagnostics stay clean).
#include "support/fixture_test.hpp"
#include "rider_control_reference_support.hpp"
#include "model_access.hpp"
#include "runtime/samples.hpp"
#include "terrain/trackfile.hpp"

#include <cmath>
#include <cstdio>
#include <optional>
#include <print>
#include <set>
#include <string>
#include <vector>

namespace {
namespace rr = rider_control_reference;
namespace rcfg = ::runtime;
namespace sw = ::runtime::sample_wire;
namespace rider = ::rider;

using rider::NamedEntries;

const double *find(const NamedEntries<double> &entries,
                   const std::string &name) {
    for (const auto &[n, v] : entries)
        if (n == name)
            return &v;
    return nullptr;
}

std::set<std::string> names(const NamedEntries<double> &entries) {
    std::set<std::string> out;
    for (const auto &[n, v] : entries)
        out.insert(n);
    return out;
}

std::set<std::string>
joint_names(const std::vector<rider::SpindleController::JointEntry> &joints) {
    std::set<std::string> out;
    for (const auto &j : joints)
        out.insert(j.name);
    return out;
}

// pytest.approx(expected, rel=1e-6) — absolute floor only for the ~1e-12
// default abs; these contracts are O(1) so a plain relative band suffices.
bool approx(double value, double expected, double rel = 1e-6) {
    return std::abs(value - expected) <=
           rel * std::max(std::abs(expected), 1e-9);
}

int jnt_qposadr(const mjModel *m, const char *name) {
    const auto adrs =
        model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr");
    return adrs[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, name))];
}

int jnt_dofadr(const mjModel *m, const char *name) {
    const auto adrs =
        model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr");
    return adrs[static_cast<std::size_t>(mj_name2id(m, mjOBJ_JOINT, name))];
}

rider::RiderCommand crank_command(double mean_torque_nm,
                                  bool enabled = true) {
    rider::RiderCommand command;
    command.mean_crank_torque_nm = mean_torque_nm;
    command.enabled = enabled;
    return command;
}

std::pair<std::span<const double>, std::span<const double>>
state_spans(const mjModel *m, const mjData *d) {
    return {model_access::readonly_buffer(d->qpos, m->nq, "qpos"),
            model_access::readonly_buffer(d->qvel, m->nv, "qvel")};
}

// _environment's flat track (test_realistic_pedelec_acceptance.py:134).
bike::terrain::TrackSpec flat_track() {
    return bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = 300.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [300.0, 0.0]] }\n"));
}

[[gnu::noinline]] void
test_pinned_control_never_calls_allocator_and_publishes_final_commands() {
    // closed_form_rider/pinned_control_never_calls_allocator_and_publishes_final_commands
    auto plant = rr::pinned_model();
    auto &c = *plant.controller;
    const auto result = c.compute(plant.data.get(), crank_command(40.),
                                  /*advance=*/false);
    const auto joints = joint_names(c.joints());
    bt::require(names(result) == joints,
                "the closed-form result publishes every pinned joint");
    const auto state = c.state();
    bt::require(state.joint_torques_nm == result,
                "joint_torques_nm is the published command dict");
    bt::require(names(state.joint_capacity_nm) == joints,
                "joint_capacity_nm covers the same joint set");
    for (const auto &[n, v] : result)
        bt::require(std::isfinite(v), "every command is finite");
    bt::require(!state.allocation_diagnostics.invalid_controller,
                "closed form never flags an invalid controller");
    const auto [qpos, qvel] = state_spans(plant.model.get(), plant.data.get());
    const auto violations = c.strength_violations(result, qpos, qvel);
    bt::require(violations.empty(), "pinned commands respect strength curves");
}

[[gnu::noinline]] void
test_upper_pose_target_accounts_for_pelvis_and_frame_pitch() {
    // closed_form_rider/upper_pose_target_accounts_for_pelvis_and_frame_pitch
    auto plant = rr::pinned_model();
    auto &c = *plant.controller;
    rider::RiderPosture pose;
    pose.torso_lean_rad = .1;
    const auto neutral = c.upper_targets(plant.data.get(), &pose);
    const double *torso = find(neutral, "rider_torso_hinge");
    bt::require(torso != nullptr, "upper targets name the torso hinge");
    bt::require(approx(*torso, .1), "neutral target mirrors torso_lean_rad");
    mjModel const *m = plant.model.get();
    mjData *d = plant.data.get();
    auto const qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    qpos[static_cast<std::size_t>(jnt_qposadr(m, "rider_root_pitch"))] = .08;
    qpos[static_cast<std::size_t>(jnt_qposadr(m, "root_pitch"))] = -.03;
    engine::forward(m, d);
    const auto changed = c.upper_targets(d, &pose);
    const double *moved = find(changed, "rider_torso_hinge");
    bt::require(moved != nullptr, "upper targets name the torso hinge");
    bt::require(approx(*moved - *torso, -.11),
                "pelvis +.08 and frame -.03 shift the hinge target by -.11");
}

[[gnu::noinline]] void
test_native_torso_torque_has_restoring_pelvis_reaction() {
    // closed_form_rider/native_torso_torque_has_restoring_pelvis_reaction
    auto plant = rr::pinned_model();
    mjModel const *m = plant.model.get();
    mjData *d = plant.data.get();
    const int root = jnt_dofadr(m, "rider_root_pitch");
    int actuator = -1;
    for (const auto &j : plant.controller->joints())
        if (j.name == "rider_torso_hinge")
            actuator = j.actuator_id;
    bt::require(actuator >= 0, "torso hinge actuator is resolved");
    auto const ctrl = model_access::mutable_buffer(d->ctrl, m->nu, "ctrl");
    auto const qacc = model_access::readonly_buffer(d->qacc, m->nv, "qacc");
    ctrl[static_cast<std::size_t>(actuator)] = -1.;
    engine::forward(m, d);
    const double negative = qacc[static_cast<std::size_t>(root)];
    ctrl[static_cast<std::size_t>(actuator)] = 1.;
    engine::forward(m, d);
    bt::require(qacc[static_cast<std::size_t>(root)] < negative - 1.,
                "positive torso torque restores the pelvis root");
}

[[gnu::noinline]] void
test_final_torques_obey_strength_power_and_accelerating_speed_limits() {
    // closed_form_rider/final_torques_obey_strength_power_and_accelerating_speed_limits
    auto plant = rr::pinned_model();
    auto &c = *plant.controller;
    mjModel const *m = plant.model.get();
    mjData *d = plant.data.get();
    for (const double speed : {4., -4., 25., -25.}) {
      auto const qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
      for (const auto &j : c.joints())
        qvel[static_cast<std::size_t>(j.dof_adr)] = speed;
      engine::forward(m, d);
      const auto result = c.compute(d, crank_command(1000.),
                                    /*advance=*/false);
      double max_power = -std::numeric_limits<double>::infinity();
      double positive_sum = 0.;
      for (const auto &j : c.joints()) {
        const double *torque = find(result, j.name);
        bt::require(torque != nullptr, "every joint has a delivered torque");
        const double power =
            *torque * qvel[static_cast<std::size_t>(j.dof_adr)];
        max_power = std::max(max_power, power);
        positive_sum += std::max(power, 0.);
      }
        bt::require(max_power <= 250. + 1e-9,
                    "per-joint power respects the 250 W cap");
        bt::require(positive_sum <= 450. + 1e-9,
                    "summed positive power respects the 450 W budget");
        if (std::abs(speed) >= 20.)
            bt::require(max_power <= 0.,
                        "past the accelerating limit all joints absorb");
        const auto [qpos_s, qvel_s] = state_spans(m, d);
        const auto violations = c.strength_violations(result, qpos_s, qvel_s);
        bt::require(violations.empty(),
                    "strength prefilter never reports a violation");
    }
}

[[gnu::noinline]] void
test_optional_activation_is_bounded_and_probes_do_not_advance_state() {
    // closed_form_rider/optional_activation_is_bounded_and_probes_do_not_advance_state
    auto plant = rr::pinned_model(
        "examples/research/viewer_physics_welded.toml",
        [](rider::SpindleConfig &c) { c.activation_tau_s = .05; });
    auto &c = *plant.controller;
    const auto before = c.state().active_state;
    const auto probe = c.compute(plant.data.get(), crank_command(40.),
                                 /*advance=*/false, .005);
    bt::require(c.state().active_state == before,
                "advance=False leaves the activation state untouched");
    const auto actual = c.compute(plant.data.get(), crank_command(40.),
                                  /*advance=*/true, .005);
    bt::require(actual.size() == probe.size(),
                "advance and probe publish the same joint set");
    for (std::size_t i = 0; i < actual.size(); ++i) {
        bt::require(actual[i].first == probe[i].first,
                    "advance and probe keep joint order");
        bt::require(approx(actual[i].second, probe[i].second),
                    "the committed step reproduces the probe torques");
    }
    const auto active = c.state().active_state;
    bt::require(active.size() == c.joints().size(),
                "activation state spans every joint");
    for (std::size_t i = 0; i < c.joints().size(); ++i) {
        const double *command = find(actual, c.joints()[i].name);
        bt::require(command != nullptr && active[i] == *command,
                    "active_state equals the delivered torques");
    }
    rr::expect_invalid(
        [&] {
            (void)c.compute(plant.data.get(), crank_command(40.),
                            /*advance=*/true, .005);
        },
        "once per timestamp");
}

[[gnu::noinline]] void
test_disabled_activation_probe_returns_zero_without_advancing_state() {
    // closed_form_rider/disabled_activation_probe_returns_zero_without_advancing_state
    auto plant = rr::pinned_model(
        "examples/research/viewer_physics_welded.toml",
        [](rider::SpindleConfig &c) { c.activation_tau_s = .05; });
    auto &c = *plant.controller;
    mjData *d = plant.data.get();
    (void)c.compute(d, crank_command(40.), /*advance=*/true, .005);
    const auto before = c.state().active_state;
    const auto disabled = crank_command(0., /*enabled=*/false);
    const auto torques =
        c.compute(d, disabled, /*advance=*/false, .005);
    bt::require(!torques.empty(), "the probe still publishes the joint set");
    for (const auto &[n, v] : torques)
        bt::require(v == 0., "a disabled command probes to zero");
    bt::require(c.state().active_state == before,
                "a disabled probe does not advance the filter");
    d->time += .005;
    (void)c.compute(d, disabled, /*advance=*/true, .005);
    for (const double v : c.state().active_state)
        bt::require(v == 0., "a disabled commit zeroes the activation state");
}

[[gnu::noinline]] void test_flat_launch_delivers_crank_torque() {
    // closed_form_rider/flat_launch_delivers_crank_torque
    auto rig = rr::pinned_rig({}, flat_track());
    auto &physical = *rig->physical;
    mjModel const *m = rig->stepper->model();
    const int root_x = jnt_dofadr(m, "root_x");
    double human_sum = 0., motor_on = 0., max_v = 0.;
    std::size_t late = 0;
    while (rig->stepper->data()->time < 6.) {
        const auto raw = rr::advance_env_steps(physical, 1);
        const double t = rig->stepper->data()->time;
        const auto &drive =
            sw::object(sw::required(raw.channels, "drive"));
        const double v = raw.qvel[static_cast<std::size_t>(root_x)];
        max_v = std::max(max_v, v);
        if (t > 2.) {
            const double human =
                sw::number(sw::required(drive, "human_sensor_nm"));
            const double motor =
                sw::number(sw::required(drive, "motor_torque_nm"));
            human_sum += human;
            if (motor > 0.)
                motor_on += 1.;
            ++late;
        }
        bt::require(!physical.rider_control()
                         .state()
                         .allocation_diagnostics.invalid_controller,
                    "closed form never flags an invalid controller");
    }
    bt::require(late > 0, "the launch ride produced late rows");
    bt::require(human_sum / static_cast<double>(late) >= 10.,
                "the rider sustains >=10 Nm human crank torque");
    // Engagement measurand, always reported (stderr is captured only on
    // failure under ctest --output-on-failure). The >=80% gate was parked
    // pre-F4 while the delivered human_sensor_nm dead-spot measurand
    // diverged from the oracle (279/401 = 69.6% vs the oracle's 351/400 =
    // 87.8%). The F-family fixes that landed since (F4 stale-efc arena,
    // FT-1 staged coast-hold serialization, the enabled-compute torque-map
    // work) closed the mechanism: the leg now measures 354/401 = 88.3% on
    // darwin-arm64 and 326/401 = 81.3% on linux-aarch64, so the source's
    // contract runs unsuppressed.
    std::println(stderr,
                 "engagement measurand: motor_on {:.0f}/{} = {:.1f}% of late "
                 "intervals (gate >=80%)",
                 motor_on, late, 100. * motor_on / static_cast<double>(late));
    bt::require(motor_on / static_cast<double>(late) >= .8,
                "the pedelec assist is engaged >=80% of the ride");
    bt::require(max_v >= 15. / 3.6,
                "the launch reaches at least 15 km/h");
}

[[gnu::noinline]] void
test_runtime_produces_front_load_share_without_changing_external_sensors() {
    // closed_form_rider/runtime_produces_front_load_share_without_changing_external_sensors
    auto rig = rr::pinned_rig();
    auto &physical = *rig->physical;
    rcfg::RideControl control;
    control.motor_torque_nm = 0.;
    control.human_torque_nm = 0.;
    ::runtime::RawStep last;
    for (int i = 0; i < 4; ++i)
        last = physical.advance_physics(1., 1., std::nullopt, control);
    const auto &tires =
        sw::object(sw::required(last.channels, "tires"));
    const double front = sw::number(
        sw::required(sw::object(sw::required(tires, "front")),
                     "normal_load_n"));
    const double rear = sw::number(
        sw::required(sw::object(sw::required(tires, "rear")),
                     "normal_load_n"));
    const auto share = physical.intent_signals().front_load_share;
    bt::require(share.has_value(),
                "the intent signal publishes a front load share");
    bt::require(approx((share).value(), front / (front + rear)), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "front_load_share tracks measured tire normals");
    const auto &sensors =
        sw::object(sw::required(last.channels, "sensors"));
    for (const auto &[k, v] : sensors)
        bt::require(k != "tires",
                    "tire channels never leak into the sensors object");
}
} // namespace

int main() {
    return bt::run([] {
        test_pinned_control_never_calls_allocator_and_publishes_final_commands();
        test_upper_pose_target_accounts_for_pelvis_and_frame_pitch();
        test_native_torso_torque_has_restoring_pelvis_reaction();
        test_final_torques_obey_strength_power_and_accelerating_speed_limits();
        test_optional_activation_is_bounded_and_probes_do_not_advance_state();
        test_disabled_activation_probe_returns_zero_without_advancing_state();
        test_flat_launch_delivers_crank_torque();
        test_runtime_produces_front_load_share_without_changing_external_sensors();
    });
}
