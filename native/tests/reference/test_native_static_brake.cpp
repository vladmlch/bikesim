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
    bt::oracle::Node node("tests/reference/test_native_static_brake.py::test_frictionloss_writes_match_python");
    node.f64(written, std::array<std::size_t, 1>{written.size()}, "static brake frictionloss"); node.done();
}
void test_solved_components_match_python() {
    Probe probe; (void)probe.brake.apply(probe.model.get(), .4, .7); probe.loads(60., -45.);
    const auto solved = probe.brake.solved_components(probe.model.get(), probe.data.get());
    bt::oracle::Node node("tests/reference/test_native_static_brake.py::test_solved_components_match_python");
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
    double front_hold(double front, double rear, bool advance) {
        return brake.hold_qfrc(model.get(), data.get(), front, rear, advance)
            .at(static_cast<std::size_t>(brake.front_dof()));
    }
};
void test_hold_bite_engagement_clip_release() {
    HoldProbe probe;
    // rolling wheel past the release speed never latches — friction bound only
    probe.wheel(1., -2.);
    bt::require(probe.front_hold(.8, .8, true) == 0., "rolling wheel sees no bite");
    // standstill inside the engage band latches the angle, zero windup
    probe.wheel(.02, 0.);
    bt::require(probe.front_hold(.8, .8, true) == 0., "fresh latch carries no spring torque");
    // wound spring below the clip: -k*(theta-latched)
    probe.wheel(.03, 0.);
    bt::require(probe.front_hold(.8, .8, true) == -10., "below-clip spring torque");
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
    test_brake_validation_matches_python(); test_apply_rejects_nonfinite_demands();
    test_demands_clamp_and_sides_are_independent();
    test_frictionloss_writes_match_python(); test_solved_components_match_python();
    test_hold_bite_engagement_clip_release(); test_hold_validation_matches_python();
}); }
