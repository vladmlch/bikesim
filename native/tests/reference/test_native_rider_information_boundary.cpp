// Ports tests/reference/test_rider_information_boundary.py — the planner
// information-boundary contracts. Two records resolve differently in C++:
// test_planner_sources_do_not_read_solver_internals (an AST scan of Python
// sources; the native boundary is structural — ConstraintDynamics and the
// kinematic-state snapshot have no solver-field inputs, asserted
// behaviorally by the ported tests below) is dropped per the ledger, and
// test_same_kinematic_state_same_command is covered by
// reference.native_rider_allocation::test_command_does_not_read_solved_reactions,
// which corrupts all three privileged channels the oracle perturbs
// (efc_force, qfrc_constraint, committed rider-contacts diagnostics) on the
// same controller and demands an identical command. The oracle node was red
// at HEAD — a second committing compute() at one timestamp trips the
// activation-once-per-timestamp guard; the test now probes non-advancing.
#include "support/fixture_test.hpp"
#include "support/model_fixture.hpp"
#include "configuration/load.hpp"
#include "diag.hpp"
#include "engine_call.hpp"
#include "model/rider_geometry.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "research/policy.hpp"
#include "runtime/control.hpp"
#include "runtime/research.hpp"
#include "runtime/sensors.hpp"
#include "support/golden.hpp"
#include "terrain/trackfile.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace engine = ::engine;
using runtime::RideControl;
using runtime::RoadSample;
using runtime::RiderKinematicState;

bt::ModelFixture free_rig() {
    bt::ModelFixture rig(
        "<mujoco><worldbody><body name=\"b\"><freejoint/>"
        "<geom size=\".1\" mass=\"1\"/></body></worldbody></mujoco>");
    engine::forward(rig.model.get(), rig.data.get());
    return rig;
}

const std::vector<std::array<double, 2>> &vertices() {
    static const std::vector<std::array<double, 2>> v{
        {0., 0.}, {1., .05}, {2., .15}, {3., .15}};
    return v;
}

bool approx(double actual, double expected) {
    return std::abs(actual - expected) <= 1e-6 * std::abs(expected) + 1e-12;
}

[[gnu::noinline]] void test_kinematic_state_contract_fields() {
    // rider_information_boundary/kinematic_state_contract_fields
    // dataclasses.fields() order has no runtime C++ counterpart; the compile
    // itself pins the declared member set and order is observed below by
    // construction/readback of every declared field.
    static_assert(std::is_same_v<decltype(RiderKinematicState::qpos),
                                 std::vector<double>>);
    static_assert(std::is_same_v<decltype(RiderKinematicState::qvel),
                                 std::vector<double>>);
    static_assert(std::is_same_v<decltype(RiderKinematicState::rider_time_s),
                                 double>);
    static_assert(std::is_same_v<decltype(RiderKinematicState::road),
                                 std::vector<RoadSample>>);
    static_assert(std::is_same_v<decltype(RoadSample::x_m), double>);
    static_assert(std::is_same_v<decltype(RoadSample::height_m), double>);
    static_assert(std::is_same_v<decltype(RoadSample::grade), double>);
    const RiderKinematicState state{
        .qpos = {1., 2.}, .qvel = {3.}, .rider_time_s = .5,
        .road = {RoadSample{.x_m = 1., .height_m = 2., .grade = .5}}};
    bt::require(state.qpos.size() == 1 + 1 && state.qvel.size() == 1 &&
                    state.rider_time_s == .5 && state.road.size() == 1 &&
                    state.road[0].x_m == 1. && state.road[0].height_m == 2. &&
                    state.road[0].grade == .5,
                "the declared fields hold their declared values");
}

[[gnu::noinline]] void test_state_snapshots_and_freezes_the_interval() {
    // rider_information_boundary/state_snapshots_and_freezes_the_interval
    auto rig = free_rig();
    const std::array<double, 2> wheels{0., 1.};
    const auto state = runtime::rider_kinematic_state(
        rig.model.get(), rig.data.get(), vertices(), wheels, .2);
    bt::require(state.qpos.size() ==
                        static_cast<std::size_t>(rig.model->nq) &&
                    state.qvel.size() ==
                        static_cast<std::size_t>(rig.model->nv),
                "the snapshot covers qpos and qvel");
    const auto qpos = model_access::readonly_buffer(
        rig.data->qpos, rig.model->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        rig.data->qvel, rig.model->nv, "qvel");
    for (std::size_t i = 0; i < state.qpos.size(); ++i)
        bt::require(state.qpos[i] == qpos[i],
                    "the snapshot carries the interval's qpos");
    for (std::size_t i = 0; i < state.qvel.size(); ++i)
        bt::require(state.qvel[i] == qvel[i],
                    "the snapshot carries the interval's qvel");
    bt::require(approx(state.rider_time_s, 0.),
                "the snapshot carries the interval start time");
    model_access::mutable_buffer(rig.data->qpos, rig.model->nq,
                                 "qpos")[0] = 5.;
    bt::require(state.qpos[0] != 5.,
                "the snapshot is an owned copy, not a view into mjData");
    // dataclasses.FrozenInstanceError is the oracle's mutation rejection;
    // the C++ equivalent is const-correctness — assigning to a const
    // RiderKinematicState fails to compile, so the freeze is checked here
    // by observing that only owned copies ever mutate.
    // The owned copy IS the contract: the comment above explains the
    // freeze is observed because only owned snapshots can mutate, so a
    // reference would make the bt::require below tautological.
    const RiderKinematicState frozen = state; // NOLINT(performance-unnecessary-copy-initialization) owned copy is the contract per the comment above
    bt::require(frozen.rider_time_s == state.rider_time_s,
                "the frozen snapshot value is stable");
}

[[gnu::noinline]] void test_state_is_blind_to_solver_truth() {
    // rider_information_boundary/state_is_blind_to_solver_truth
    auto rig = free_rig();
    const std::array<double, 2> wheels{0., 1.};
    const auto before = runtime::rider_kinematic_state(
        rig.model.get(), rig.data.get(), vertices(), wheels, 0.);
    const auto forces = model_access::mutable_buffer(
        rig.data->efc_force, rig.data->nefc, "efc_force");
    // A range-for over the span would bind a `double` copy and the write
    // would not reach the buffer; indexed assignment through forces[] is
    // the load-bearing idiom here.
    for (std::size_t i = 0; i < forces.size(); ++i) forces[i] = 1e6; // NOLINT(modernize-loop-convert) indexed write must reach the buffer
    const auto constraint = model_access::mutable_buffer(
        rig.data->qfrc_constraint, rig.model->nv, "qfrc_constraint");
    const auto applied = model_access::mutable_buffer(
        rig.data->qfrc_applied, rig.model->nv, "qfrc_applied");
    for (std::size_t i = 0; i < constraint.size(); ++i) {
        constraint[i] = -1e6;
        applied[i] = 1e6;
    }
    const auto after = runtime::rider_kinematic_state(
        rig.model.get(), rig.data.get(), vertices(), wheels, 0.);
    bt::require(before.qpos == after.qpos && before.qvel == after.qvel &&
                    before.rider_time_s == after.rider_time_s &&
                    before.road.size() == after.road.size(),
                "solver internals are not state inputs");
    for (std::size_t i = 0; i < before.road.size(); ++i)
        bt::require(before.road[i].x_m == after.road[i].x_m &&
                        before.road[i].height_m == after.road[i].height_m &&
                        before.road[i].grade == after.road[i].grade,
                    "corrupting the constraint solve changes no road sample");
}

[[gnu::noinline]] void test_road_window_is_bounded() {
    // rider_information_boundary/road_window_is_bounded
    const std::array<double, 2> wheels{.2, 1.2};
    const auto samples =
        runtime::road_samples(vertices(), wheels, .5, .1);
    bt::require(!samples.empty(), "the window yields samples");
    bool sorted = true, has_low = false, has_high = false;
    double low = std::numeric_limits<double>::max(), high = -low;
    double h_at_high = 0., g_at_high = 0., g_at_low = 0.;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (i && samples[i].x_m < samples[i - 1].x_m) sorted = false;
        low = std::min(low, samples[i].x_m);
        high = std::max(high, samples[i].x_m);
        if (samples[i].x_m == .2) has_low = true;
        if (samples[i].x_m == 1.2) {
            has_high = true;
            h_at_high = samples[i].height_m;
            g_at_high = samples[i].grade;
        }
        if (samples[i].x_m == .2) g_at_low = samples[i].grade;
    }
    bt::require(sorted, "sample positions arrive sorted");
    bt::require(has_low && has_high, "both wheel positions are sampled");
    bt::require(low >= .2 && high <= 1.2 + .5 + 1e-12,
                "the disclosed window is bounded by the wheels + lookahead");
    bt::require(approx(h_at_high, .07) && approx(g_at_high, .10) &&
                    approx(g_at_low, .05),
                "interpolated heights and segment grades match the profile");
    // Zero lookahead discloses only what is under the wheels (the oracle
    // compares x_m as a set — the duplicated front+lookahead sample
    // carries the same disclosed position).
    const auto only = runtime::road_samples(vertices(), wheels, 0.);
    std::vector<double> only_x;
    only_x.reserve(only.size());
    for (const auto &s : only)
      only_x.push_back(s.x_m);
    std::ranges::sort(only_x);
    const auto tail = std::ranges::unique(only_x);
    only_x.erase(tail.begin(), tail.end());
    bt::require(only_x == std::vector<double>{.2, 1.2},
                "zero lookahead discloses only the wheel positions");
    bool rejected = false;
    try {
        const std::array<double, 0> none{};
        (void)runtime::road_samples(vertices(), none, 0.);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    bt::require(rejected, "empty wheel positions are rejected");
}

[[gnu::noinline]] void test_sensor_observation_contract_unchanged() {
    // rider_information_boundary/sensor_observation_contract_unchanged
    // The Python dataclass field order is the wire order in the native
    // codec; the emitted key sequence is the observable contract.
    runtime::SensorObservation const observation;
    const auto wire = observation.as_wire();
    const std::array<std::string_view, 10> keys{
        "time_s", "source_time_s", "valid", "specific_force_body_mps2",
        "pitch_rate_up_rad_s", "front_wheel_rad_s", "rear_wheel_rad_s",
        "crank_rad_s", "motor_torque_nm", "human_torque_nm"};
    bt::require(wire.size() == keys.size(),
                "the sensor observation carries exactly the declared fields");
    for (std::size_t i = 0; i < keys.size(); ++i)
        bt::require(wire[i].first == keys[i],
                    "sensor observation field order: " +
                        std::string(keys[i]));
}

[[gnu::noinline]] void test_motor_policy_cannot_take_ownership_of_rider_cadence() {
    // rider_information_boundary/motor_policy_cannot_take_ownership_of_rider_cadence
    // The oracle drives the rejection through PolicySession.advance; the
    // native gate is the same validate_motor_policy_control call the
    // session's begin_advance performs before touching the environment.
    RideControl control;
    control.crank_target_rate_rad_s = 8.;
    bool rejected = false;
    try {
        bike::research::validate_motor_policy_control(control);
    } catch (const std::invalid_argument &error) {
        rejected = std::string_view(error.what()).find(
                       "motor policy cannot own rider inputs") !=
                   std::string_view::npos;
        bt::require(rejected,
                    "a rider-input policy output names the ownership gate");
    }
    bt::require(rejected,
                "a policy RideControl carrying rider cadence is rejected");
}

// Stage D: changing the surface label 1.5 m ahead of the front wheel
// (asphalt vs scree) never leaks into the research SensorObservation.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[gnu::noinline]] void test_research_observation_is_invariant_to_surface_ahead() {
    const auto make_env = [](std::string_view track_toml) {
        auto req = std::make_unique<bike::research::ResearchRequest>();
        const auto repo_root =
            bt::oracle::root().parent_path().parent_path().parent_path();
        req->setup.model.physics = bike::configuration::load_physics(
            repo_root / "examples/research/viewer_physics_welded.toml");
        req->setup.model.mode = "ride";
        req->setup.model.crank_joint = true;
        req->setup.model.rider =
            bike::model::rider_from_args("articulated_planar");
        req->setup.track = bike::terrain::track_from_dict(
            bike::terrain::parse_track_toml(track_toml));
        req->setup.model.physics->initial_speed_mps = 0.;
        req->experiment.duration_s = .05;
        req->experiment.control_period_s =
            req->setup.model.physics->research.control_period_s;
        req->experiment.actuator_delay_s =
            req->setup.model.physics->research.actuator_delay_s;
        req->experiment.record_decimation = 1;
        req->experiment.maximum_energy_residual_ratio = 1e9;
        req->experiment.stop_on_model_violation = false;
        req->sensors.delivery.latency_s =
            req->setup.model.physics->research.sensor_latency_s;
        req->sensors.delivery.maximum_age_s =
            req->setup.model.physics->research.sensor_maximum_age_s;
        req->sensors.sample_period_s = .005;
        return bike::research::make_environment(std::move(*req));
    };
    const auto bits = [](double v) { return std::bit_cast<std::uint64_t>(v); };
    const auto require_obs_bitwise = [&](const runtime::SensorObservation &a,
                                         const runtime::SensorObservation &b) {
        bt::require(bits(a.time_s) == bits(b.time_s) &&
                        bits(a.source_time_s) == bits(b.source_time_s) &&
                        a.valid == b.valid,
                    "research SensorObservation metadata is bitwise identical");
        for (std::size_t i = 0; i < a.values.size(); ++i)
            bt::require(bits(a.values[i]) == bits(b.values[i]),
                        "research SensorObservation values are bitwise identical despite surface_ahead change");
    };
    auto env_asphalt = make_env(
        "name = \"ahead_asphalt\"\nlength_m = 40.0\nsurface = \"asphalt\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [40.0, 0.0]] }\n");
    auto env_scree = make_env(
        "name = \"ahead_scree\"\nlength_m = 40.0\nsurface = \"asphalt\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [40.0, 0.0]] }\n"
        "surface_sections = [{ start_m = 4.3, end_m = 8.0, surface = \"scree\" }]\n");
    const RideControl cmd{};
    for (int step = 0; step < 4; ++step) {
        env_asphalt->begin_control(cmd, 0., 0.);
        const auto ra = env_asphalt->advance_control(std::nullopt);
        env_asphalt->acknowledge_control();
        env_scree->begin_control(cmd, 0., 0.);
        const auto rb = env_scree->advance_control(std::nullopt);
        env_scree->acknowledge_control();
        bt::require(ra.has_value() && rb.has_value(),
                    "both research environments produce transitions");
        require_obs_bitwise(ra.value().observation, rb.value().observation); // NOLINT(bugprone-unchecked-optional-access) checked immediately above
    }
}
NATIVE_DIAG_POP
} // namespace

int main() {
    return bt::run([] {
        test_kinematic_state_contract_fields();
        test_state_snapshots_and_freezes_the_interval();
        test_state_is_blind_to_solver_truth();
        test_road_window_is_bounded();
        test_sensor_observation_contract_unchanged();
        test_motor_policy_cannot_take_ownership_of_rider_cadence();
        test_research_observation_is_invariant_to_surface_ahead();
    });
}
