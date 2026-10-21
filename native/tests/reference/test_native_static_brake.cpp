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
    void loads(double front, double rear) {
        auto applied = model_access::mutable_buffer(data->qfrc_applied, model->nv);
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
    Probe probe; const auto front = probe.brake.front_dof(), rear = probe.brake.rear_dof();
    bt::invalid([&] { runtime::StaticBrake invalid(front, front, 200.); });
    bt::invalid([&] { runtime::StaticBrake invalid(-1, rear, 200.); });
    bt::invalid([&] { runtime::StaticBrake invalid(front, rear, -1.); });
}
void test_apply_rejects_nonfinite_demands() {
    Probe probe;
    bt::invalid([&] { (void)probe.brake.apply(probe.model.get(), std::numeric_limits<double>::quiet_NaN(), 0.); });
    bt::invalid([&] { (void)probe.brake.apply(probe.model.get(), 0., std::numeric_limits<double>::infinity()); });
    bt::require(std::ranges::all_of(probe.friction(), [](double value) { return value == 0.; }), "invalid demands publish neither side");
}
} // namespace
int main() { return bt::run([] {
    test_brake_validation_matches_python(); test_apply_rejects_nonfinite_demands();
    test_demands_clamp_and_sides_are_independent();
    test_frictionloss_writes_match_python(); test_solved_components_match_python();
}); }
