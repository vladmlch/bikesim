// Task 3.20: five original static-brake functions. The first two consume
// real Python oracle captures; absent artifacts fail through oracle::Node.
#include "support/golden.hpp"
#include "support/model_fixture.hpp"
#include "runtime/static_brake.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
constexpr std::string_view xml = R"(<mujoco><worldbody>
  <body name="bike" pos="0 0 1"><freejoint/><geom type="box" size=".1 .1 .1" mass="20"/>
    <body name="front_wheel" pos=".6 0 -.6"><joint name="front_wheel_spin" type="hinge" axis="0 1 0"/><geom type="sphere" size=".2" mass="2"/></body>
    <body name="rear_wheel" pos="-.6 0 -.6"><joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/><geom type="sphere" size=".2" mass="2"/></body>
  </body></worldbody></mujoco>)";
struct Probe : bt::ModelFixture {
    runtime::StaticBrake brake;
    Probe() : ModelFixture(xml), brake(dof("front_wheel_spin"), dof("rear_wheel_spin"), 200.) {}
    std::span<double> friction() { return model_access::mutable_buffer(model->dof_frictionloss, model->nv); }
    void loads(double front, double rear) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
        auto const applied = model_access::mutable_buffer(data->qfrc_applied, model->nv);
        std::ranges::fill(applied, 0.); applied[static_cast<std::size_t>(brake.front_dof())] = front;
        applied[static_cast<std::size_t>(brake.rear_dof())] = rear;
        engine::forward(model.get(), data.get());
    }
};
void test_frictionloss_writes_match_python() {
    Probe probe; (void)probe.brake.apply(probe.model.get(), .4, .7);
    const auto written = probe.friction();
    bt::require(std::abs(written[static_cast<std::size_t>(probe.brake.front_dof())] - 80.) <= 80e-6, "front frictionloss matches original approx assertion");
    bt::require(std::abs(written[static_cast<std::size_t>(probe.brake.rear_dof())] - 140.) <= 140e-6, "rear frictionloss matches original approx assertion");
    bt::oracle::Node node("native_static_brake/frictionloss_writes_match_python");
    node.f64(written, std::array<std::size_t, 1>{written.size()}, "static brake frictionloss"); node.done();
}
void test_solved_components_match_python() {
    Probe probe; (void)probe.brake.apply(probe.model.get(), .4, .7); probe.loads(60., -45.);
    const auto solved = probe.brake.solved_components(probe.model.get(), probe.data.get());
    bt::oracle::Node node("native_static_brake/solved_components_match_python");
    node.f64_close(solved.first, std::array<std::size_t, 1>{solved.first.size()}, "front static brake solved");
    node.f64_close(solved.second, std::array<std::size_t, 1>{solved.second.size()}, "rear static brake solved"); node.done();
    bt::require(std::abs(solved.first.at(static_cast<std::size_t>(probe.brake.front_dof()))) > 1., "solved friction row actually holds the applied wheel load");
}
void test_demands_clamp_and_sides_are_independent() {
    Probe probe; (void)probe.brake.apply(probe.model.get(), 1.5, -.5);
    bt::require(probe.friction()[static_cast<std::size_t>(probe.brake.front_dof())] == 200., "front demand saturates at ceiling");
    bt::require(probe.friction()[static_cast<std::size_t>(probe.brake.rear_dof())] == 0., "negative rear demand clamps to zero");
    probe.loads(60., 0.); const auto solved = probe.brake.solved_components(probe.model.get(), probe.data.get());
    bt::require(std::ranges::all_of(solved.second, [](double value) { return value == 0.; }), "unwritten rear brake has no solved generalized force");
    bt::require(std::abs(solved.first.at(static_cast<std::size_t>(probe.brake.front_dof()))) > 1., "front brake remains loaded independently");
}
void test_brake_validation_matches_python() {
    Probe const probe; const auto front = probe.brake.front_dof(), rear = probe.brake.rear_dof();
    bt::invalid([&] { runtime::StaticBrake const invalid(front, front, 200.); });
    bt::invalid([&] { runtime::StaticBrake const invalid(-1, rear, 200.); });
    bt::invalid([&] { runtime::StaticBrake const invalid(front, rear, -1.); });
}
void test_apply_rejects_nonfinite_demands() {
    Probe probe;
    bt::invalid([&] { (void)probe.brake.apply(probe.model.get(), std::numeric_limits<double>::quiet_NaN(), 0.); });
    bt::invalid([&] { (void)probe.brake.apply(probe.model.get(), 0., std::numeric_limits<double>::infinity()); });
    bt::require(std::ranges::all_of(probe.friction(), [](double value) { return value == 0.; }), "invalid demands publish neither side");
    (void)probe.brake.apply(probe.model.get(), .25, .5);
    const auto before = std::vector<double>(probe.friction().begin(), probe.friction().end());
    bt::invalid([&] { (void)probe.brake.apply(probe.model.get(), 1., 1., std::numeric_limits<double>::quiet_NaN(), 0.); });
    bt::invalid([&] { (void)probe.brake.apply(probe.model.get(), 1., 1., 10., std::numeric_limits<double>::infinity()); });
    bt::require(std::ranges::equal(before, probe.friction()), "invalid hold torque changes neither bound");
    (void)probe.brake.apply(probe.model.get(), .5, .75, -20., 200.);
    bt::require(probe.friction()[static_cast<std::size_t>(probe.brake.front_dof())] == 80. &&
                    probe.friction()[static_cast<std::size_t>(probe.brake.rear_dof())] == 0.,
                "signed holds consume their magnitudes and residual floors at zero");
}

// --- standstill bite (brake_hold_*) -------------------------------------------
// Mirrors the added Python cases: the latched spring-damper pins the wheel
// angle at standstill, clipped by demand; release hysteresis keeps a rolling
// wheel on the friction bound alone.
struct HoldProbe : bt::ModelFixture {
    runtime::StaticBrake brake;
    HoldProbe() : ModelFixture(xml),
        brake(dof("front_wheel_spin"), dof("rear_wheel_spin"), 200.,
              runtime::BrakeHoldSpec{.torque_nm = 50.,
                                     .stiffness_nm_rad = 1000.,
                                     .damping_nms_rad = 5.,
                                     .engage_radps = .3,
                                     .release_radps = .6}) {}
    void wheel(double theta, double omega) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
        auto const qp = model_access::mutable_buffer(data->qpos, model->nq, "data.qpos");
        auto const qv = model_access::mutable_buffer(data->qvel, model->nv, "data.qvel");
        std::ranges::fill(qp, 0.); std::ranges::fill(qv, 0.);
        const auto qposadr = model_access::readonly_buffer(
            model->jnt_qposadr, model->njnt, "jnt_qposadr");
        const int joint = mj_name2id(model.get(), mjOBJ_JOINT, "front_wheel_spin");
        model_access::require_id(joint, model->njnt, "front_wheel_spin");
        qp[static_cast<std::size_t>(
            qposadr[static_cast<std::size_t>(joint)])] = theta;
        qv[static_cast<std::size_t>(brake.front_dof())] = omega;
    }
    double front_hold(double front, double rear, bool advance,
                      bool reseat = false) {
        return brake.hold_qfrc(model.get(), data.get(), front, rear, advance,
                               reseat)
            .at(static_cast<std::size_t>(brake.front_dof()));
    }
};

// Exercise both the actual MuJoCo friction row and the elastic bite on a
// wheel with unit inertia, with no other forces or constraints.
constexpr std::string_view budget_xml = R"(<mujoco>
  <option timestep=".001" gravity="0 0 0"/>
  <worldbody>
    <body><joint name="front" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size=".5" mass="10"/></body>
    <body pos="2 0 0"><joint name="rear" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size=".5" mass="10"/></body>
  </worldbody></mujoco>)";

void test_combined_budget_and_breakaway() {
    for (const double load : {150., 300.}) {
        bt::ModelFixture rig(budget_xml);
        const int front = rig.dof("front"), rear = rig.dof("rear");
        const auto fi = static_cast<std::size_t>(front);
        runtime::StaticBrake brake(front, rear, 200.,
            {.torque_nm = 200., .stiffness_nm_rad = 1000.,
             .damping_nms_rad = 0., .engage_radps = .3,
             .release_radps = .6});
        brake.restore({.latch_front = 0., .latch_rear = std::nullopt});
        const auto qpos = model_access::mutable_buffer(rig.data->qpos, rig.model->nq);
        const auto qvel = model_access::mutable_buffer(rig.data->qvel, rig.model->nv);
        const auto applied = model_access::mutable_buffer(rig.data->qfrc_applied, rig.model->nv);
        qpos[fi] = .15;
        bool saw_release = false;
        for (int step = 0; step < 1000; ++step) {
            const auto hold = brake.hold_qfrc(rig.model.get(), rig.data.get(), 1., 0., true);
            (void)brake.apply(rig.model.get(), 1., 0., hold[fi], 0.);
            if (!brake.state().latch_front.has_value()) {
                saw_release = true;
                const auto friction = model_access::readonly_buffer(rig.model->dof_frictionloss, rig.model->nv);
                bt::require(hold[fi] == 0. && friction[fi] == 200.,
                            "after release all 200 Nm return to solver friction");
            }
            applied[fi] = load + hold[fi];
            engine::forward(rig.model.get(), rig.data.get());
            const auto solved = brake.solved_components(rig.model.get(), rig.data.get());
            const double resistance = hold[fi] + solved.first[fi];
            bt::require(std::abs(resistance) <= 200. + 1e-6,
                        "combined solved friction and hold never exceed 200 Nm");
            if (load == 150.) {
                bt::require(std::abs(resistance + 150.) < 1e-6,
                            "the preloaded stationary wheel balances 150 Nm");
            }
            engine::step(rig.model.get(), rig.data.get());
        }
        if (load == 150.) {
            bt::require(std::abs(qpos[fi] - .15) < 1e-9 && std::abs(qvel[fi]) < 1e-9,
                        "sub-ceiling torque holds without creep");
        } else {
            bt::require(saw_release && qvel[fi] > .6 && qpos[fi] > .15,
                        "super-ceiling torque breaks away and accelerates");
        }
    }
}

void test_ceiling_also_controls_reseat() {
    HoldProbe probe;
    runtime::StaticBrake brake(probe.brake.front_dof(), probe.brake.rear_dof(), 200.,
        {.torque_nm = 400., .stiffness_nm_rad = 1000.,
         .damping_nms_rad = 0., .engage_radps = .3, .release_radps = .6});
    probe.wheel(0., 0.);
    (void)brake.hold_qfrc(probe.model.get(), probe.data.get(), .5, 0., true);
    probe.wheel(.15, 0.);
    const auto clipped = brake.hold_qfrc(probe.model.get(), probe.data.get(), .5, 0., false);
    bt::require(clipped[static_cast<std::size_t>(brake.front_dof())] == -100.,
                "hold torque itself cannot exceed the brake ceiling");
    const auto reseated = brake.hold_qfrc(probe.model.get(), probe.data.get(), .5, 0., true, true);
    bt::require(reseated[static_cast<std::size_t>(brake.front_dof())] == 0. &&
                    brake.state().latch_front == .15,
                "reseat uses the ceiling cap, not the larger configured hold");
}
void test_hold_bite_engagement_clip_release() {
    HoldProbe probe;
    // rolling wheel past the release speed never latches — friction bound only
    probe.wheel(1., -2.);
    bt::require(probe.front_hold(.8, .8, true) == 0., "rolling wheel sees no bite");
    // standstill inside the engage band latches the angle, zero windup
    probe.wheel(.02, 0.);
    bt::require(probe.front_hold(.8, .8, true) == 0., "fresh latch carries no spring torque");
    // wound spring below the clip: -k*(theta-latched); the literal is not
    // exactly -10 (0.03-0.02 is 0.009999999999999998 in binary64), so the
    // expectation spells the same arithmetic the writer performs.
    probe.wheel(.03, 0.);
    bt::require(probe.front_hold(.8, .8, true) == -1000. * (.03 - .02),
                "below-clip spring torque");
    // wound past the clip: demand-scaled cap 50*.8
    probe.wheel(.5, 0.);
    bt::require(probe.front_hold(.8, .8, true) == -40., "clip bounds the stored torque");
    // a probe pass reads the committed latch — release speed does not move it
    probe.wheel(.5, 1.);
    bt::require(probe.front_hold(.8, .8, false) == -40., "probe pass keeps the latch");
    // advancing past the release speed forgets the captured angle
    probe.wheel(.5, 1.);
    bt::require(probe.front_hold(.8, .8, true) == 0., "release speed forgets the latch");
    // hysteresis: inside the band but above engage stays free
    probe.wheel(.5, .4);
    bt::require(probe.front_hold(.8, .8, true) == 0., "hysteresis band holds the release");
    // re-engage at the new angle — damping lands in the same call
    probe.wheel(.5, .1);
    bt::require(probe.front_hold(.8, .8, true) == -.5, "re-bite damps within the band");
    probe.wheel(.6, .1);
    bt::require(probe.front_hold(.8, .8, true) == -40., "re-wound spring clips again");
    // lever freed: the bite releases
    probe.wheel(.6, .1);
    bt::require(probe.front_hold(0., .8, true) == 0., "freed lever releases");
    // reset forgets the captured angle; the next standstill re-bites clean
    probe.brake.reset();
    probe.wheel(.9, 0.);
    bt::require(probe.front_hold(.5, .8, true) == 0., "reset re-bites without memory");
}
void test_hold_reseat_saturated_bite() {
    // Mirrors test_hold_reseat_parity_sequence: the relaxation field's
    // re-anchor drops a saturated stale offset while the live field keeps
    // the stale latch pinned at the clip (the savage settle defect fix).
    HoldProbe probe;
    probe.wheel(.02, 0.);
    bt::require(probe.front_hold(.8, .8, true) == 0.,
                "fresh latch carries no spring torque");
    // crept past the clip wind-up: live field (reseat off) stays pinned
    probe.wheel(.5, 0.);
    bt::require(probe.front_hold(.8, .8, true) == -40.,
                "live field keeps the stale latch at the clip");
    // relaxation field re-anchors the slipped pad at the current angle
    probe.wheel(.5, 0.);
    bt::require(probe.front_hold(.8, .8, true, true) == 0.,
                "reseat drops the saturated stale offset");
    // below the clip nothing re-seats — the fresh anchor holds
    // (same binary64 spelling caveat as above: 0.51-0.5 is not exact 0.01)
    probe.wheel(.51, 0.);
    bt::require(probe.front_hold(.8, .8, true, true) == -1000. * (.51 - .5),
                "unsaturated spring keeps the anchor");
    // a probe pass never moves the latch even with reseat set
    probe.wheel(.6, 0.);
    bt::require(probe.front_hold(.8, .8, false, true) == -40.,
                "probe reads the committed latch");
    probe.wheel(.6, 0.);
    bt::require(probe.front_hold(.8, .8, true, true) == 0.,
                "advancing re-seat follows the creep");
    // inside the hysteresis band (above engage, below release) a rolling
    // wheel is not a standstill slip — the bite stays pinned at the clip
    probe.wheel(.62, .4);
    bt::require(probe.front_hold(.8, .8, true, true) ==
                    -1000. * (.62 - .6) - 5. * .4,
                "hysteresis band is not a re-seat slip");
    // above the release speed the latch forgets as before
    probe.wheel(.62, 1.);
    bt::require(probe.front_hold(.8, .8, true, true) == 0.,
                "release speed still forgets the latch");
}
void test_hold_validation_matches_python() {
    HoldProbe probe;
    bt::invalid([&] {
        runtime::StaticBrake const invalid(
            probe.brake.front_dof(), probe.brake.rear_dof(), 200.,
            runtime::BrakeHoldSpec{.torque_nm = -1.}); });
    bt::invalid([&] {
        runtime::StaticBrake const invalid(
            probe.brake.front_dof(), probe.brake.rear_dof(), 200.,
            runtime::BrakeHoldSpec{.engage_radps = .6, .release_radps = .3}); });
    // disabled-by-default: zero torque or stiffness never produces a force
    Probe plain;
    bt::require(plain.brake.hold_qfrc(plain.model.get(), plain.data.get(),
                                      1., 1., true)
                    .at(static_cast<std::size_t>(plain.brake.front_dof())) == 0.,
                "default hold spec disables the bite");
}
} // namespace
int main() { return bt::run([] {
    test_combined_budget_and_breakaway();
    test_ceiling_also_controls_reseat();
    test_brake_validation_matches_python(); test_apply_rejects_nonfinite_demands();
    test_demands_clamp_and_sides_are_independent();
    test_frictionloss_writes_match_python(); test_solved_components_match_python();
    test_hold_bite_engagement_clip_release(); test_hold_reseat_saturated_bite();
    test_hold_validation_matches_python();
}); }
