// Completion 3.22b. Source: tests/reference/test_period_buffer.py.
// The Python PeriodBuffer's plan tuple path (_raw's (jac, qfrc) pairs) has no
// native surface: evaluate_attachments consumes AttachmentRaw spatial 6 x k
// blocks only. The scalar leg keeps recover_wrench on the same dense matrix,
// which is exactly what the Python test compares the batched sample against.
#include "support/migration_test.hpp"
#include "support/golden.hpp"
#include "diag.hpp"
#include "configuration/fields.hpp"
#include "configuration/load.hpp"
#include "configuration/rider_spec.hpp"
#include "io/json.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/spindle_controller.hpp"
#include "runtime/accounting.hpp"
#include "runtime/period_buffer.hpp"
#include "runtime/recording.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "terrain/trackfile.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <ranges>
#include <set>
#include <string>
#include <vector>

namespace bt_oracle = bt::oracle;
namespace research = bike::research;
namespace sw = runtime::sample_wire;

namespace {

const runtime::AttachmentBudget BUDGET{
    .foot_min_normal_n = 20., .foot_mu = .9, .saddle_mu = .6,
    .grip_pull_n = 300., .max_gap_m = .005};

// The exception contract pytest.raises(..., match=...) expresses: type plus
// a message fragment.
template <class Ex, class F>
void throws(F &&action, std::string_view needle, std::string_view message) {
    try {
        std::forward<F>(action)();
    } catch (const Ex &failure) {
        bt::require(needle.empty() ||
                    std::string(failure.what()).find(needle) != std::string::npos,
                    std::string(message) + ": unexpected message '" +
                        failure.what() + "'");
        return;
    }
    bt::require(false, std::string(message) + ": no exception was raised");
}

runtime::RawStep raw(std::int64_t i) {
    runtime::RawStep step;
    step.interval_id = i;
    step.time_s = static_cast<double>(i) * .0005;
    step.end_time_s = static_cast<double>(i + 1) * .0005;
    return step;
}

// The spatial AttachmentRaw whose batched solve reproduces the Python test's
// planar jac row: dof0/-dof3 carry Fx=5, dof1 carries Fz=100-12i, so the
// min-norm spatial wrench is [5,0,100-12i,0,0,0] and decompose's Z-normal
// projection reports the identical normal_n the scalar recover_wrench row
// reports at index 1.
rider::AttachmentRaw spatial_foot(std::int64_t i) {
    rider::AttachmentRaw item;
    const rider::DenseMatrix jac{.rows=6, .columns=4, .values={
        1., 0., 0., -1.,
        0., 0., 0., 0.,
        0., 1., 0., 0.,
        0., 0., 0., 0.,
        0., 0., 0., 0.,
        0., 0., 0., 0.}};
    const double fz = 100. - 12. * static_cast<double>(i);
    item.rider_jac = jac;
    item.rider_qfrc = {5., fz, 0., -5.};
    item.bike_jac = jac;
    item.bike_qfrc = {-5., -fz, 0., 5.};
    item.observable = true;
    item.normal = {0., 0., 1.};
    item.kind = "foot";
    item.rotational = false;
    item.half_patch_m = .025;
    item.gap_m = 0.;
    return item;
}

std::vector<std::string> violations_of(const rider::AttachmentSample &sample) {
    return runtime::attachment_violations(sample, BUDGET);
}

// test_batched_wrench_matches_scalar_and_first_violation
void test_batched_wrench_matches_scalar_and_first_violation() {
    runtime::PeriodBuffer buffer(10);
    for (std::int64_t i = 0; i < 10; ++i) {
        auto step = raw(i);
        step.attachment_raw = {{"foot_front", spatial_foot(i)}};
        buffer.push(std::move(step));
    }
    bt::require(buffer.full(), "the buffer reports full at capacity");
    const rider::DenseMatrix relative{.rows=3, .columns=4, .values={
        1., 0., 0., -1.,
        0., 1., 0., 0.,
        0., 0., 1., 0.}};
    rider::LeastSquaresWorkspace workspace{4, 6, 1};
    std::optional<std::int64_t> first;
    for (const auto &step : buffer.raws()) {
        const auto samples = runtime::evaluate_attachments(step, workspace);
        bt::require(samples.size() == 1 && samples.front().first == "foot_front",
                    "the batched pass publishes one foot_front sample per raw");
        const auto &sample = samples.front().second;
        const auto wrench = rider::recover_wrench(workspace, relative,
                                                step.attachment_raw.front().second.rider_qfrc);
        bt::require(std::abs(sample.normal_n - wrench.at(1)) <= 1e-9,
                    "batched normal_n matches the scalar recovered wrench");
        if (!violations_of(sample).empty() && !first.has_value())
            first = step.interval_id;
    }
    bt::require(first == 7, "the first normal-force violation lands on interval 7");
    buffer.clear();
    bt::require(!buffer.full() && buffer.empty(),
                "drain leaves the buffer empty");
}

// test_missing_wrench_is_not_replaced_by_zero
void test_missing_wrench_is_not_replaced_by_zero() {
    auto step = raw(0);
    rider::AttachmentRaw item = spatial_foot(0);
    std::ranges::fill(item.rider_jac.values, 0.);
    std::ranges::fill(item.bike_jac.values, 0.);
    step.attachment_raw = {{"foot_front", std::move(item)}};
    rider::LeastSquaresWorkspace workspace{4, 6, 1};
    const auto samples = runtime::evaluate_attachments(step, workspace);
    bt::require(samples.empty(),
                "an unexplainable wrench drops the attachment — no zero fill");
    // Same gate on the bike side and on the observability flag.
    step = raw(0);
    item = spatial_foot(0);
    item.observable = false;
    step.attachment_raw = {{"foot_front", std::move(item)}};
    bt::require(runtime::evaluate_attachments(step, workspace).empty(),
                "an unobservable attachment is dropped, not zero-filled");
}

// test_buffer_overflow_and_discontinuous_interval_are_rejected
void test_buffer_overflow_and_discontinuous_interval_are_rejected() {
    runtime::PeriodBuffer buffer(1);
    buffer.push(raw(0));
    throws<std::runtime_error>([&] { buffer.push(raw(1)); }, "overflow",
        "push past capacity is the overflow RuntimeError");
    runtime::PeriodBuffer continuous(3);
    continuous.push(raw(0));
    throws<std::invalid_argument>([&] { continuous.push(raw(2)); }, "consecutive",
        "a nonconsecutive interval_id is the ValueError");
}

// test_recording_a_published_period_twice_is_idempotent
void test_recording_a_published_period_twice_is_idempotent() {
    runtime::ResearchRecorder recorder(1, {});
    const auto make = [](std::int64_t id) {
        auto sample = std::make_shared<runtime::PhysicalSampleData>();
        sample->interval_id = id;
        sample->end_time_s = .0005 * static_cast<double>(id + 1);
        return runtime::SamplePtr(std::move(sample));
    };
    recorder.append(make(0));
    recorder.append(make(1));
    // The second record() sees the same committed samples; the _last_id gate
    // keeps the second pass idempotent.
    throws<std::invalid_argument>([&] { recorder.append(make(0)); }, "",
        "an id below the acknowledged tail is rejected, never re-recorded");
    recorder.append(make(1));
    recorder.append(make(1));
    bt::require(recorder.rows() == 2,
                "re-recording the published period keeps exactly the two rows");
}

// test_actual_spatial_body_inputs_match_scalar_attachment_measurement
void test_actual_spatial_body_inputs_match_scalar_attachment_measurement() {
    constexpr std::string_view xml = R"XML(
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
</worldbody>
<equality><weld name="w" body1="rider" body2="bike"/></equality></mujoco>
)XML";
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    bt::require(mj_addBufferVFS(&vfs, "pair.xml", xml.data(),
                              static_cast<int>(xml.size())) == 0,
                "pair MJCF mounts into the VFS");
    std::array<char, 1024> error{};
    mjModel *loaded = mj_loadXML("pair.xml", &vfs, error.data(),
                                 static_cast<int>(error.size()));
    mj_deleteVFS(&vfs);
    bt::require(loaded != nullptr, "pair model compiles: " + std::string(error.data()));
    const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(loaded, &mj_deleteModel);
    const std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), &mj_deleteData);
    const int rider = mj_name2id(model.get(), mjOBJ_BODY, "rider");
    const int bike = mj_name2id(model.get(), mjOBJ_BODY, "bike");
    bt::require(rider > 0 && bike > 0, "both bodies resolve");
    rider::EqualityReactions const reactions(model.get());
    rider::LeastSquaresWorkspace workspace{12, 6, 1};
    std::vector<double> scratch(static_cast<std::size_t>(model.get()->nv));
    const std::vector<double> point{0., 0., 1.}, normal{0., 0., 1.};
    const std::optional<std::vector<double>> pull;
    std::vector<runtime::RawStep> raws;
    std::vector<rider::AttachmentSample> expected;
    for (std::int64_t i = 0; i < 10; ++i) {
        mj_forward(model.get(), data.get());
        const auto xfrc = std::views::counted(data->xfrc_applied,
            model->nbody * 6);
        std::ranges::fill(xfrc, 0.);
        xfrc[static_cast<std::size_t>(rider) * 6 + 0] = 5. + static_cast<double>(i);
        xfrc[static_cast<std::size_t>(rider) * 6 + 2] = -100. + 12. * static_cast<double>(i);
        mj_forward(model.get(), data.get());
        expected.push_back(rider::attachment_sample(
            model.get(), data.get(), reactions, workspace, scratch, 0,
            rider, bike, point, normal, "foot", true, .025, pull));
        auto item = rider::attachment_raw(
            model.get(), data.get(), reactions, scratch, 0,
            rider, bike, point, normal, "foot", true, .025, pull);
        auto step = raw(i);
        step.attachment_raw = {{"foot_front", std::move(item)}};
        raws.push_back(std::move(step));
    }
    const auto same = [](const rider::AttachmentSample &a,
                         const rider::AttachmentSample &b) {
        return a.kind == b.kind && a.normal_n == b.normal_n &&
               a.tangent_n == b.tangent_n && a.moment_nm == b.moment_nm &&
               a.gap_m == b.gap_m && a.pull_n == b.pull_n &&
               a.half_patch_m == b.half_patch_m;
    };
    for (std::size_t i = 0; i < raws.size(); ++i) {
        const auto samples = runtime::evaluate_attachments(raws[i], workspace);
        bt::require(samples.size() == 1 && samples.front().first == "foot_front",
                    "the deferred spatial measurement produces a sample");
        bt::require(same(samples.front().second, expected[i]),
                    "batched AttachmentSample equals the scalar measurement");
    }
    // A spatial min-norm answer must still explain both bodies and cancel:
    // corrupting the bike measurement or dropping observability yields no
    // sample, never a substituted one.
    {
        auto item = raws[0].attachment_raw.front().second;
        for (auto &v : item.bike_qfrc) v += 10.;
        auto step = raws[0];
        step.attachment_raw = {{"foot_front", std::move(item)}};
        bt::require(runtime::evaluate_attachments(step, workspace).empty(),
                    "a Newton-violating measurement drops the attachment");
    }
    {
        auto item = raws[0].attachment_raw.front().second;
        item.observable = false;
        auto step = raws[0];
        step.attachment_raw = {{"foot_front", std::move(item)}};
        bt::require(runtime::evaluate_attachments(step, workspace).empty(),
                    "an unobservable measurement drops the attachment");
    }
}

// The one-joint model-free effort context: joint 'joint' at qpos/dof/actuator
// 0 on an (nq, nv) = (1, 1) state — the SimpleNamespace controller of the
// Python test.
runtime::EffortContext effort_context(rider::SpindleConfig config, bool strength) {
    config.has_strength = strength;
    if (strength) {
        // Emulates the oracle lambda `('joint',) if q[0] >= 2.`: the positive
        // curve is flat at 10 Nm until 1.5 rad then ramps to zero, so the
        // delivered 2 Nm exceeds capacity only when the anatomical angle —
        // the raw q here — reaches 2.
        config.strength_coordinates["joint"] =
            {.neutral_anatomical_rad = 0., .direction = 1,
             .minimum_anatomical_rad = -4., .maximum_anatomical_rad = 4.,
             .provenance = "test"};
        config.strength["joint"] = rider::DirectionalStrength{
            .positive = {.angles_rad = {0., 1.5, 3.},
                         .torques_nm = {10., 10., 0.},
                         .vmax_rad_s = 10., .hill_c = .3,
                         .eccentric_ratio = 1.5, .source = "test"},
            .negative = {.angles_rad = {0., 1.5, 3.},
                         .torques_nm = {10., 10., 0.},
                         .vmax_rad_s = 10., .hill_c = .3,
                         .eccentric_ratio = 1.5, .source = "test"}};
    }
    return runtime::EffortContext(std::move(config),
        {rider::SpindleController::JointEntry{.name="joint", .qpos_adr=0, .dof_adr=0, .actuator_id=0}}, 1, 1);
}

runtime::RawStep effort_raw(std::int64_t i, double qpos, double speed, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
                            double actuator, double passive) {
    auto step = raw(i);
    step.qpos = {qpos};
    step.qvel = {speed};
    step.actuator_force = {actuator};
    step.qfrc_passive = {passive};
    step.effort_base = {};
    step.effort_state = [] {
        rider::EffortDiagnostics diagnostics;
        diagnostics.present = true;
        return diagnostics;
    }();
    step.rider_control_terms =
        rider::NamedEntries<rider::JointTerms>{{"joint", rider::JointTerms{}}};
    return step;
}

double channel_number(const runtime::WireObject &channels, std::string_view name) {
    return sw::number(sw::required(channels, name));
}
bool channel_bool(const runtime::WireObject &channels, std::string_view name) {
    return sw::boolean(sw::required(channels, name));
}
double channel_named(const runtime::WireObject &channels, std::string_view name,
                     std::string_view joint) {
    return sw::number(sw::required(sw::object(sw::required(channels, name)), joint));
}
std::vector<std::string> channel_strings(const runtime::WireObject &channels,
                                         std::string_view name) {
    std::vector<std::string> out;
    for (const auto &value : sw::array(sw::required(channels, name)))
        out.push_back(sw::string(value));
    return out;
}

// test_held_command_power_and_strength_are_checked_at_each_incoming_state
void test_held_command_power_and_strength_are_checked_at_each_incoming_state() {
    rider::SpindleConfig config;
    config.active_positive_power_limit_w = 10.;
    config.joint_power_limit_w = 250.;
    config.joint_speed_limit_rad_s = 20.;
    const auto context = effort_context(std::move(config), true);
    const std::array<double, 3> speeds{-1., 1., 6.};
    const std::array<double, 3> powers{0., 2., 12.};
    std::vector<std::vector<std::string>> violations;
    for (std::int64_t i = 0; i < 3; ++i) {
        const auto observed = runtime::observe_effort(
            effort_raw(i, static_cast<double>(i), speeds[static_cast<std::size_t>(i)],
                       2., -.5 * speeds[static_cast<std::size_t>(i)]),
            context, .0005);
        bt::require(channel_number(observed.channels, "rider_positive_power_w")
                    == powers[static_cast<std::size_t>(i)],
                    "held active power is torque x incoming speed");
        bt::require(channel_bool(observed.channels, "rider_effort_budget_exceeded")
                    == (i == 2),
                    "the positive-power budget gates at each incoming state");
        violations.push_back(observed.violations);
    }
    bt::require(violations[0].empty() && violations[1].empty(),
                "the first two intervals pass every effort gate");
    bt::require(violations[2] == std::vector<std::string>(
                    {"rider_strength.joint", "rider_power.positive"}),
                "strength and positive-power violations land in oracle order");
    bt::require(std::abs(raw(2).end_time_s - .0015) < 1e-15,
                "first failure time is the third interval's end");
}

// test_held_intervals_reject_joint_power_and_speed_between_control_ticks
void test_held_intervals_reject_joint_power_and_speed_between_control_ticks() {
    rider::SpindleConfig config;
    config.active_positive_power_limit_w = 450.;
    config.joint_power_limit_w = 250.;
    config.joint_speed_limit_rad_s = 20.;
    const auto context = effort_context(std::move(config), false);
    const std::array<std::pair<double, double>, 3> cases{
        {{25., 10.}, {26., 10.}, {-2., 21.}}};
    std::vector<std::vector<std::string>> violations;
    std::vector<runtime::EffortObservation> observed;
    for (std::int64_t i = 0; i < 3; ++i) {
        const auto [torque, speed] = cases[static_cast<std::size_t>(i)];
        observed.push_back(runtime::observe_effort(
            effort_raw(i, 0., speed, torque, 0.), context, .0005));
        violations.push_back(observed.back().violations);
    }
    bt::require(violations[0].empty(), "250 W at the exact limit is admitted");
    bt::require(violations[1] == std::vector<std::string>{"rider_power.joint"},
                "260 W trips the joint power gate");
    bt::require(violations[2] == std::vector<std::string>{"rider_speed.joint"},
                "21 rad/s trips the joint speed gate");
    bt::require(std::abs(raw(1).end_time_s - .001) < 1e-15,
                "first failure lands at the second interval's end");
    bt::require(channel_named(observed[1].channels, "rider_joint_positive_power_w",
                            "joint") == 260.,
                "per-joint positive power is reported by name");
    bt::require(channel_strings(observed[2].channels,
                                "rider_joint_speed_violations")
                == std::vector<std::string>{"joint"},
                "the speed violation names the joint");
}

// ---- runtime scaffolding shared by the three full-episode tests ----------

struct RideRig {
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t period = 0;
    double timestep_s = 0.;
};

RideRig make_ride(double control_period_s, int record_decimation, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
                  double duration_s) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));
    request.setup.model.mode = "ride";
    request.setup.model.physics->timestep_s = .0005;
    // The oracle froze the Python equilibrium; the ported solver converges
    // through a different allocator warm-start iterate, so the test restores
    // the captured PhysicalInitialState document (initial_state_file path)
    // instead of re-solving — the same settled-cache reset the oracle uses.
    // The document's contract hash covers compiled-model fields, so each
    // platform owns its capture under the golden platform key.
    request.setup.initial_state.emplace(bt::text(bt::fixture(
        std::string("initial_state/") + std::string(bt_oracle::platform()) +
        "/probe_welded_0005.json")));
    request.setup.model.rider = bike::configuration::RiderSpecs{};
    request.setup.model.rider->variant = "articulated_planar";
    request.setup.track = bike::terrain::track_from_dict(
        bike::terrain::parse_track_toml(
            "name = \"probe\"\nlength_m = 200.0\nsurface = \"hardpack\"\n"
            "grade_profile = { knots = [[0.0, 0.0], [200.0, 0.0]] }\n"));
    request.experiment.duration_s = duration_s;
    request.experiment.control_period_s = control_period_s;
    request.experiment.record_decimation = record_decimation;
    request.experiment.stop_on_model_violation = false;
    // The oracle run keeps the CLI sensor defaults (latency .01 s, seeded
    // noise) at --sensor-period .005 s: seated_climb feeds measured channels
    // into the rider intent, so the sensor path is physics-visible here.
    request.sensors = research::SensorConfig{};
    request.sensors.sample_period_s = .005;
    const auto prepared = research::prepare_environment(std::move(owner));
    RideRig rig;
    // The period buffer cadence is the physics control_period_s (.005 s -> 10
    // steps), independent of the experiment's policy control period.
    rig.period = prepared->bootstrap.ride.config.control_period_steps;
    rig.timestep_s = prepared->bootstrap.config.timestep_s;
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        std::move(prepared->bootstrap.ride));
    rig.runtime = std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
    // env.reset() replays the stored settled cache: the first cold
    // equilibrium solve leaves a different allocator warm-start iterate than
    // the replayed bootstrap, so the golden path requires the reset state.
    rig.runtime->reset();
    return rig;
}

void drain_pending(runtime::NativeRideRuntime &runtime,
                   std::vector<runtime::SamplePtr> &out) {
    auto const batch = runtime.prepare_samples();
    for (const auto &sample : batch.samples) out.push_back(sample);
    runtime.acknowledge_samples(batch);
}

void step_once(RideRig &rig, const runtime::RideControl &command) {
    const auto result = rig.runtime->advance(rig.runtime->snapshot().step + 1,
                                             command, 0., 0., std::nullopt);
    bt::require(result.reason != "outcome", "the welded flat run stays rideable");
}

const runtime::WireObject *channel_object(const runtime::SamplePtr &sample,
                                          std::string_view name) {
    const auto *wire = sw::find(sample->channels, name);
    return wire ? &sw::object(*wire) : nullptr;
}

std::vector<std::string> channel_violations(const runtime::SamplePtr &sample) {
    const auto *wire = sw::find(sample->channels, "attachment_violations");
    std::vector<std::string> out;
    if (!wire) return out;
    for (const auto &value : sw::array(*wire)) out.push_back(sw::string(value));
    return out;
}

std::vector<double> channel_array(const runtime::WireObject &object,
                                  std::string_view name) {
    std::vector<double> out;
    for (const auto &value : sw::array(sw::required(object, name)))
        out.push_back(sw::number(value));
    return out;
}

// test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms
// (slow): 60 intervals under a .0075 s control period with off-period flushes
// at intervals 15 and 37; every published interval's qpos/qvel and held
// rider_control terms are golden-frozen.
void test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms() {
    bt_oracle::Node node(
        "tests/reference/test_period_buffer.py::test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms");
    auto rig = make_ride(.0075, 1, .03);
    bt::require(rig.period == 10, "the .005 s period buffer is 10 steps");
    std::vector<runtime::SamplePtr> actual;
    for (std::int64_t i = 0; i < 60; ++i) {
        step_once(rig, {.motor_torque_nm = 0., .motor_limit_nm = {},
                        .human_torque_nm = i < 20 ? 0. : 5.,
                        .crank_target_rate_rad_s = {}, .posture = {}});
        drain_pending(*rig.runtime, actual);
        if (i + 1 == 15 || i + 1 == 37) {
            rig.runtime->flush();
            drain_pending(*rig.runtime, actual);
        }
    }
    rig.runtime->flush();
    drain_pending(*rig.runtime, actual);
    bt::require(actual.size() == 60, "the non-aligned flush keeps all 60 intervals");
    std::vector<double> qpos, qvel;
    for (const auto &sample : actual) {
        qpos.insert(qpos.end(), sample->qpos.begin(), sample->qpos.end());
        qvel.insert(qvel.end(), sample->qvel.begin(), sample->qvel.end());
    }
    const std::array<std::size_t, 2> qpos_shape{60, actual.front()->qpos.size()};
    const std::array<std::size_t, 2> qvel_shape{60, actual.front()->qpos.size()};
    node.f64_close(qpos, qpos_shape, "expected/qpos");
    node.f64_close(qvel, qvel_shape, "expected/qvel");
    // The held rider_control terms compare as: the sorted joint -> sorted
    // field key inventory (bitwise JSON), the numeric leaves (1e-9) and the
    // boolean leaves (bitwise), all in sorted order like the oracle walk.
    runtime::WireArray keys;
    std::vector<double> numbers, flags;
    for (const auto &sample : actual) {
        const auto *control = channel_object(sample, "rider_control");
        bt::require(control != nullptr, "every interval publishes rider_control");
        std::vector<std::string> joints;
        for (const auto &[joint, terms] : *control) joints.push_back(joint);
        std::ranges::sort(joints);
        runtime::WireObject bool_keys, num_keys;
        for (const auto &joint : joints) {
            const auto &terms = sw::object(sw::required(*control, joint));
            std::vector<std::string> booleans, numerics;
            for (const auto &[field, value] : terms) {
                if (std::holds_alternative<bool>(value.value))
                    booleans.push_back(field);
                else
                    numerics.push_back(field);
            }
            std::ranges::sort(booleans);
            std::ranges::sort(numerics);
            bool_keys.emplace_back(joint, sw::strings(booleans));
            num_keys.emplace_back(joint, sw::strings(numerics));
            for (const auto &field : numerics)
                numbers.push_back(sw::number(sw::required(terms, field)));
            for (const auto &field : booleans)
                flags.push_back(sw::boolean(sw::required(terms, field)) ? 1. : 0.);
        }
        keys.emplace_back(runtime::WireObject{
            {"bool", runtime::Wire{std::move(bool_keys)}},
            {"num", runtime::Wire{std::move(num_keys)}}});
    }
    const auto keys_json = bike::io::canonical_json(
        bike::research::wire_value(runtime::Wire{std::move(keys)}));
    const auto keys_bytes = std::as_bytes(std::span(keys_json));
    const std::array<std::size_t, 1> keys_shape{keys_bytes.size()};
    node.bytes(keys_bytes, "|u1", keys_shape, "expected/rider_control_keys");
    const std::array<std::size_t, 1> num_shape{numbers.size()};
    const std::array<std::size_t, 1> flag_shape{flags.size()};
    node.f64_close(numbers, num_shape, "expected/rider_control_num");
    node.f64(flags, flag_shape, "expected/rider_control_bool");
    node.done();
}

// test_runtime_batch_matches_preserved_scalar_step_and_flush (slow):
// 2003 intervals at decimation 1 against the frozen scalar-oracle payload,
// then a decimation-80 run whose measured-id and channel-key inventories are
// frozen too.
void test_runtime_batch_matches_preserved_scalar_step_and_flush() {
    bt_oracle::Node node(
        "tests/reference/test_period_buffer.py::test_runtime_batch_matches_preserved_scalar_step_and_flush");
    const runtime::RideControl command{
        .motor_torque_nm = 0., .motor_limit_nm = {}, .human_torque_nm = 0.,
        .crank_target_rate_rad_s = {}, .posture = {}};
    auto rig = make_ride(.005, 1, 1.);
    bt::require(rig.period == 10, "the default control period is 10 steps");
    std::vector<runtime::SamplePtr> actual;
    for (std::int64_t i = 0; i < 2003; ++i) {
        step_once(rig, command);
        drain_pending(*rig.runtime, actual);
    }
    rig.runtime->flush();
    drain_pending(*rig.runtime, actual);
    bt::require(actual.size() == 2003, "the flush publishes the trailing partial period");
    std::vector<double> qpos, qvel, energy, power;
    runtime::WireArray violations;
    for (const auto &sample : actual) {
        qpos.insert(qpos.end(), sample->qpos.begin(), sample->qpos.end());
        qvel.insert(qvel.end(), sample->qvel.begin(), sample->qvel.end());
        const auto *channels = &sample->channels;
        const auto *energy_wire = sw::find(*channels, "energy");
        bt::require(energy_wire != nullptr, "every interval publishes its energy row");
        const auto &energy_object = sw::object(*energy_wire);
        for (const auto *name : {"muscle_positive_j", "constraint_absolute_j",
                               "residual_j"})
            energy.push_back(sw::number(sw::required(energy_object, name)));
        power.push_back(sw::number(sw::required(*channels, "rider_positive_power_w")));
        violations.emplace_back(sw::strings(channel_violations(sample)));
    }
    const std::array<std::size_t, 2> qpos_shape{2003, actual.front()->qpos.size()};
    const std::array<std::size_t, 2> qvel_shape{2003, actual.front()->qvel.size()};
    const std::array<std::size_t, 2> energy_shape{2003, 3};
    const std::array<std::size_t, 1> power_shape{2003};
    const auto violations_json = bike::io::canonical_json(
        bike::research::wire_value(runtime::Wire{std::move(violations)}));
    const auto failure = rig.runtime->snapshot().first_failure;
    const std::string failure_json = failure
        ? bike::io::canonical_json(bike::research::wire_value(
              sw::strings(failure->violations)))
        : "null";
    node.f64_close(qpos, qpos_shape, "expected/qpos");
    node.f64_close(qvel, qvel_shape, "expected/qvel");
    node.f64_close(energy, energy_shape, "expected/energy");
    node.f64_close(power, power_shape, "expected/rider_positive_power_w");
    const auto violations_bytes = std::as_bytes(std::span(violations_json));
    const std::array<std::size_t, 1> violations_shape{violations_bytes.size()};
    node.bytes(violations_bytes, "|u1", violations_shape,
               "expected/attachment_violations");
    const std::vector<double> failure_time =
        failure ? std::vector<double>{failure->time_s} : std::vector<double>{};
    const std::array<std::size_t, 1> failure_shape{failure_time.size()};
    node.f64(failure_time, failure_shape, "monitor/first_failure_time_s");
    const auto failure_bytes = std::as_bytes(std::span(failure_json));
    const std::array<std::size_t, 1> failure_json_shape{failure_bytes.size()};
    node.bytes(failure_bytes, "|u1", failure_json_shape,
               "monitor/first_failure_violations");
    // The decimated phase: a fresh runtime is the native equivalent of
    // env.reset() + set_measurement_decimation(80) — the measurement
    // cadence is a construction-time property of the bootstrap.
    auto decimated_rig = make_ride(.005, 80, 1.);
    std::vector<runtime::SamplePtr> decimated;
    for (std::int64_t i = 0; i < 2003; ++i) {
        step_once(decimated_rig, command);
        drain_pending(*decimated_rig.runtime, decimated);
    }
    decimated_rig.runtime->flush();
    drain_pending(*decimated_rig.runtime, decimated);
    bt::require(decimated.size() == 2003, "decimation keeps every interval row");
    std::vector<std::string> last_keys;
    for (const auto &[key, value] : decimated.back()->channels) last_keys.push_back(key);
    std::ranges::sort(last_keys);
    const auto keys_json = bike::io::canonical_json(
        bike::research::wire_value(sw::strings(last_keys)));
    const auto keys_bytes = std::as_bytes(std::span(keys_json));
    const std::array<std::size_t, 1> keys_shape{keys_bytes.size()};
    node.bytes(keys_bytes, "|u1", keys_shape, "decimated/last_channel_keys");
    std::set<std::int64_t> full_ids;
    for (const auto &sample : decimated)
        if (channel_object(sample, "rider_control") != nullptr)
            full_ids.insert(sample->interval_id);
    std::vector<double> full_ids_doubles(full_ids.begin(), full_ids.end());
    const std::array<std::size_t, 1> ids_shape{full_ids_doubles.size()};
    node.f64(full_ids_doubles, ids_shape, "decimated/full_ids");
    // The frozen id set is the oracle's {i % 80 == 0 or period close or tail}.
    std::set<std::int64_t> expected_ids;
    for (std::int64_t i = 0; i < 2003; ++i)
        if (i % 80 == 0 || (i + 1) % 10 == 0 || i == 2002) expected_ids.insert(i);
    bt::require(full_ids == expected_ids,
                "measured intervals are the decimation and period-closing set");
    bt::require(decimated.back()->channels.size() == actual.back()->channels.size() &&
                [&] {
                    std::set<std::string> a, b;
                    for (const auto &[key, v] : decimated.back()->channels) a.insert(key);
                    for (const auto &[key, v] : actual.back()->channels) b.insert(key);
                    return a == b;
                }(),
                "the decimated tail keeps the full channel inventory");
    for (std::size_t i = 0; i < 2003; ++i) {
        const auto &scalar = actual[i], &sample = decimated[i];
        bt::require(channel_violations(scalar) == channel_violations(sample),
                    "attachment violations are decimation-invariant");
        const auto forwarded = sw::number(sw::required(
            sw::object(sw::required(sample->channels, "energy")), "muscle_positive_j"));
        const auto reference = sw::number(sw::required(
            sw::object(sw::required(scalar->channels, "energy")), "muscle_positive_j"));
        bt::require(std::abs(forwarded - reference) <= 1e-9,
                    "forwarded muscle work matches the measured row");
    }
    node.done();
}

// test_record_decimation_forwards_period_closing_energy (slow): runtime-vs-
// runtime invariants — a decimation-1 baseline and a decimation-80 run over
// period*81+3 intervals, so the unmeasured flush tail exercises the
// previous-period closing-datum forward.
void test_record_decimation_forwards_period_closing_energy() {
    const runtime::RideControl command{
        .motor_torque_nm = 0., .motor_limit_nm = {}, .human_torque_nm = 0.,
        .crank_target_rate_rad_s = {}, .posture = {}};
    auto baseline_rig = make_ride(.005, 1, 1.);
    const std::int64_t period = baseline_rig.period;
    const std::int64_t steps = period * 81 + 3;
    const auto collect = [&](RideRig &rig) {
        std::vector<runtime::SamplePtr> rows;
        for (std::int64_t i = 0; i < steps; ++i) {
            step_once(rig, command);
            drain_pending(*rig.runtime, rows);
        }
        rig.runtime->flush();
        drain_pending(*rig.runtime, rows);
        return rows;
    };
    const auto baseline = collect(baseline_rig);
    auto decimated_rig = make_ride(.005, 80, 1.);
    const auto decimated = collect(decimated_rig);
    bt::require(baseline.size() == static_cast<std::size_t>(steps) &&
                decimated.size() == static_cast<std::size_t>(steps),
                "both runs publish every committed interval");
    std::set<std::int64_t> full_ids;
    for (const auto &sample : decimated)
        if (channel_object(sample, "rider_control") != nullptr)
            full_ids.insert(sample->interval_id);
    std::set<std::int64_t> expected_ids;
    for (std::int64_t i = 0; i < steps; ++i)
        if (i % 80 == 0 || (i + 1) % period == 0 || i == steps - 1)
            expected_ids.insert(i);
    bt::require(full_ids == expected_ids,
                "full intervals are decimation rows plus period closes and the tail");
    const auto closing = [&](std::int64_t interval) {
        const std::int64_t start = interval - interval % period;
        std::optional<std::int64_t> last;
        for (std::int64_t j = start; j < std::min(start + period, steps); ++j)
            if (j % 80 == 0 || (j + 1) % period == 0) last = j;
        if (!last) for (std::int64_t j = start - 1; j >= 0; --j)
            if (j % 80 == 0 || (j + 1) % period == 0) { last = j; break; }
        bt::require(last.has_value(), "a closing measurement exists");
        return *last;
    };
    const std::array<std::string_view, 13> accumulators{
        "active_work_j", "external_work_j", "loss_j", "loss_step_j",
        "muscle_signed_j", "muscle_positive_j", "motor_signed_j",
        "motor_positive_j", "constraint_absolute_j", "constraint_signed_j",
        "solver_constraint_work_j", "electrical_work_j", "source_positive_work_j"};
    const auto elastic_equal = [&](const runtime::WireObject &energy, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
                                   const runtime::WireObject &expected,
                                   std::int64_t i) {
        const auto &actual_elastic = sw::object(sw::required(energy, "elastic_energy_j"));
        std::set<std::string> a_keys, e_keys;
        for (const auto &[key, v] : actual_elastic) a_keys.insert(key);
        for (const auto &[key, v] : expected) e_keys.insert(key);
        bt::require(a_keys == e_keys, "elastic key sets match at interval " + std::to_string(i));
        for (const auto &[name, value] : expected)
            bt::require(std::abs(sw::number(sw::required(actual_elastic, name))
                                 - sw::number(value)) <= 1e-9,
                        "elastic energy " + name + " forwards at interval " + std::to_string(i));
    };
    for (std::size_t i = 0; i < decimated.size(); ++i) {
        const auto &base = baseline[i], &sample = decimated[i];
        bt::require(base->interval_id == sample->interval_id,
                    "interval ids align");
        const auto id = sample->interval_id;
        bt::require(channel_violations(base) == channel_violations(sample),
                    "attachment violations are decimation-invariant");
        const auto &base_energy = sw::object(sw::required(base->channels, "energy"));
        const auto &sample_energy = sw::object(sw::required(sample->channels, "energy"));
        for (const auto name : accumulators)
            bt::require(std::abs(sw::number(sw::required(sample_energy, name))
                                 - sw::number(sw::required(base_energy, name))) <= 1e-9,
                        std::string(name) + " accumulator matches at interval " + std::to_string(id));
        const auto &base_mass = sw::object(sw::required(base->channels, "mass"));
        const auto &sample_mass = sw::object(sw::required(sample->channels, "mass"));
        bt::require(std::abs(sw::number(sw::required(sample_mass, "mass_kg"))
                             - sw::number(sw::required(base_mass, "mass_kg"))) <= 1e-9,
                    "compiled mass matches");
        const auto base_com = channel_array(base_mass, "com_m");
        const auto sample_com = channel_array(sample_mass, "com_m");
        bt::require(base_com.size() == sample_com.size(), "com width matches");
        for (std::size_t c = 0; c < base_com.size(); ++c)
            bt::require(std::abs(sample_com[c] - base_com[c]) <= 1e-12,
                        "compiled CoM matches at interval " + std::to_string(id));
        const bool measured = id % 80 == 0 || (id + 1) % period == 0;
        if (measured) {
            for (const auto *name : {"mechanical_energy_j", "residual_j"})
                bt::require(std::abs(sw::number(sw::required(sample_energy, name))
                                     - sw::number(sw::required(base_energy, name))) <= 1e-9,
                            std::string(name) + " is the interval's own solve");
            elastic_equal(sample_energy, sw::object(sw::required(base_energy, "elastic_energy_j")), id);
            bt::require(sample_mass.size() == base_mass.size(), "measured mass inventory matches");
        } else {
            std::set<std::string> keys;
            for (const auto &[key, v] : sample_mass) keys.insert(key);
            bt::require(keys == std::set<std::string>{"mass_kg", "com_m"},
                        "unmeasured intervals keep only mass/CoM rows");
            const auto &forwarded = sw::object(sw::required(
                baseline[static_cast<std::size_t>(closing(id))]->channels, "energy"));
            bt::require(std::abs(sw::number(sw::required(sample_energy, "mechanical_energy_j"))
                                 - sw::number(sw::required(forwarded, "mechanical_energy_j"))) <= 1e-9,
                        "unmeasured mechanical energy forwards the closing solve");
            elastic_equal(sample_energy, sw::object(sw::required(forwarded, "elastic_energy_j")), id);
            bt::require(std::abs(sw::number(sw::required(sample_energy, "residual_j"))
                        - (sw::number(sw::required(base_energy, "residual_j"))
                           + sw::number(sw::required(forwarded, "mechanical_energy_j"))
                           - sw::number(sw::required(base_energy, "mechanical_energy_j")))) <= 1e-9,
                        "the forwarded residual keeps the interval's own accumulators");
        }
    }
    for (const auto *name : {"model_violation_counts", "model_valid", "numerically_valid"}) {
        const auto &base_status = sw::object(sw::required(baseline.back()->channels, "model_status"));
        const auto &sample_status = sw::object(sw::required(decimated.back()->channels, "model_status"));
        const auto a = sw::required(base_status, name), b = sw::required(sample_status, name);
        bt::require(bike::io::canonical_json(bike::research::wire_value(a))
                    == bike::io::canonical_json(bike::research::wire_value(b)),
                    std::string("model_status.") + std::string(name) + " is decimation-invariant");
    }
    const auto *base_endpoint = sw::find(
        sw::object(sw::required(
            baseline[static_cast<std::size_t>(closing(steps - 1))]->channels,
            "endpoint_mass")), "com_m");
    const auto *tail_endpoint = sw::find(
        sw::object(sw::required(decimated.back()->channels, "endpoint_mass")), "com_m");
    bt::require(tail_endpoint != nullptr && base_endpoint != nullptr,
                "the unmeasured tail reports the last measured endpoint mass");
    const auto tail_com = channel_array(sw::object(sw::required(decimated.back()->channels, "endpoint_mass")), "com_m");
    const auto base_com = channel_array(sw::object(sw::required(
        baseline[static_cast<std::size_t>(closing(steps - 1))]->channels, "endpoint_mass")), "com_m");
    bt::require(tail_com.size() == base_com.size(), "endpoint com width matches");
    for (std::size_t c = 0; c < tail_com.size(); ++c)
        bt::require(std::abs(tail_com[c] - base_com[c]) <= 1e-12,
                    "the tail forwards the closing endpoint CoM");
}

} // namespace

// Ten test bodies plus the pair-model scaffolding inline into this one
// dispatcher; the aggregate frame exceeds the 8 KiB policy even though each
// callee stays within budget, so the suppression wraps main only.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_batched_wrench_matches_scalar_and_first_violation();
        test_missing_wrench_is_not_replaced_by_zero();
        test_buffer_overflow_and_discontinuous_interval_are_rejected();
        test_recording_a_published_period_twice_is_idempotent();
        test_actual_spatial_body_inputs_match_scalar_attachment_measurement();
        test_held_command_power_and_strength_are_checked_at_each_incoming_state();
        test_held_intervals_reject_joint_power_and_speed_between_control_ticks();
        test_non_aligned_flush_keeps_each_intervals_actual_held_command_terms();
        test_runtime_batch_matches_preserved_scalar_step_and_flush();
        test_record_decimation_forwards_period_closing_energy();
    });
}
NATIVE_DIAG_POP
