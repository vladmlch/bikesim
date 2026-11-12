// test_energy_ledger.cpp — native port of tests/reference/test_energy_ledger.py
// (task C323, batch 3.23). Fourteen oracle records in file order:
//
//   1  test_positive_body_work_is_not_net_joint_work
//   2  test_large_constraint_work_cannot_improve_residual_quality
//   3  test_motor_work_reports_signed_and_positive_separately
//   4  test_invalid_work_inputs_are_rejected
//   5  test_constraint_budget_uses_roundoff_floor_when_no_source
//   6  test_runtime_constraint_budget_checks_closing_tail_with_fixed_roundoff
//      (4 parametrized legs)
//   7  test_runtime_zero_source_tolerance_does_not_grow_with_episode_steps
//   8  test_actual_solver_rows_partition_without_missing_or_duplicate_forces
//      (4 parametrized kinds)
//   9  test_cached_equilibrium_restores_the_actual_one_way_constraint_preload
//   10 test_opposite_actual_closure_work_cannot_cancel_inside_one_interval
//      (2 parametrized kinds)
//   11 test_free_fall_keeps_energy_without_constraint_work
//   12 test_closed_ideal_constraint_does_no_work
//   13 test_inelastic_impact_loss_is_physical_not_constraint_credit
//   14 test_friction_loss_is_nonnegative_and_consistent
//   15 test_closure_stored_energy_balances_load_unload_cycle — native-only
//      record (crank-coupling plan, 2026-11-06, task 1): the elastic energy
//      parked in a one-way closure belongs to `total`, so one load/unload
//      cycle of a limited tendon leaves only the share the compliant
//      constraint actually dissipates outside the ledger.
//   16 test_work_ledger_integrates_at_the_interval_midpoint and
//      test_publish_raw_carries_the_interval_midpoint_velocity — native-only
//      record (crank-coupling plan, 2026-11-06): the +48 J rectangle-rule
//      drift of .scratch/devin/2026-11-06-diag-48j/report.md is fixed by
//      charging every integrated F·v·dt channel the interval-midpoint
//      velocity — the same convention records 12-15 apply by hand.
//   17 test_rider_end_stops_are_the_envelope_joints and
//      test_end_stop_work_is_a_loss — native-only records (launch part 4,
//      decision 7): a rider joint's limit row is an anatomical end stop,
//      keyed rider_end_stop:<id> and charged to the loss line — never to
//      numerical constraint work or the closure channel.
//
// The oracle's neutral-runtime seams (`runtime_for_raws`/`neutral_raw` in
// test_monitor_strictness.py, imported by this file) live in
// strictness_reference_support.hpp so both ports execute the same body.
// Record 10 was repaired at the oracle (F1 re-scope: equality rows charge
// the closure channel, never the numerical bound) — the native assert set
// follows the repaired channel split exactly.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "engine_call.hpp"
#include "model_access.hpp"
#include "configuration/load.hpp"
#include "configuration/rider_spec.hpp"
#include "diag.hpp"
#include "drivetrain/transmission.hpp"
#include "model/builder.hpp"
#include "model/rider_geometry.hpp"
#include "research/evidence.hpp"
#include "rider_control_reference_support.hpp"
#include "runtime/accounting.hpp"
#include "runtime/samples.hpp"
#include "runtime/status.hpp"
#include "runtime/step.hpp"
#include "strictness_reference_support.hpp"
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"

namespace {

namespace wire = runtime::sample_wire;
namespace cfg = bike::configuration;
namespace mdl = bike::model;
namespace rr = rider_control_reference;
using strictness_support::neutral_raw;
using strictness_support::runtime_for_raws;

// ---- small numeric helpers --------------------------------------------------

[[nodiscard]] bool approx(double actual, double expected,
                          double rtol = 1e-6, double atol = 0.) {
    return std::abs(actual - expected) <= atol + rtol * std::abs(expected);
}

[[nodiscard]] double dot(std::span<const double> a, std::span<const double> b) {
    double sum = 0.;
    const std::size_t n = std::min(a.size(), b.size());
    for (std::size_t i = 0; i < n; ++i) sum += a[i] * b[i];
    return sum;
}

[[nodiscard]] double norm(std::span<const double> v) {
    return std::sqrt(dot(v, v));
}

[[nodiscard]] bool allclose(std::span<const double> a, std::span<const double> b,
                            double rtol = 1e-5, double atol = 1e-8) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > atol + rtol * std::abs(b[i])) return false;
    return true;
}

[[nodiscard]] std::span<const double> span_of(const std::vector<double> &v) {
    return {v.data(), v.size()};
}

// ---- model construction (mjVFS + mj_loadXML, test_period_buffer pattern) ----

[[nodiscard]] std::unique_ptr<mjModel, decltype(&mj_deleteModel)>
load_xml(std::string_view xml, const char *message) {
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    bt::require(mj_addBufferVFS(&vfs, "model.xml", xml.data(),
                              static_cast<int>(xml.size())) == 0,
                "MJCF mounts into the VFS");
    std::array<char, 1024> error{};
    mjModel *loaded = mj_loadXML("model.xml", &vfs, error.data(),
                                 static_cast<int>(error.size()));
    mj_deleteVFS(&vfs);
    bt::require(loaded != nullptr,
                std::string(message) + " compiles: " + error.data());
    return {loaded, &mj_deleteModel};
}

[[nodiscard]] std::unique_ptr<mjData, decltype(&mj_deleteData)>
data_of(const mjModel *model) {
    return {mj_makeData(model), &mj_deleteData};
}

[[nodiscard]] std::vector<double> copy(std::span<const mjtNum> source) {
    return {source.begin(), source.end()};
}

// ---- records 1/3/4/5: direct ledger arithmetic ------------------------------

void test_positive_body_work_is_not_net_joint_work() {
    const std::array muscle{400., -350.}, constraint{20., -20.};
    const std::span<const double> empty;
    const runtime::StepWork work =
        runtime::step_work(muscle, 100., constraint, empty, .1);
    bt::require(work.muscle_positive_j == 40., "muscle positive work");
    bt::require(work.muscle_signed_j == 5., "muscle signed work");
    bt::require(work.constraint_absolute_j == 4., "constraint absolute work");
    bt::require(work.constraint_signed_j == 0., "constraint signed work");
    bt::require(!runtime::constraint_work_ok(4., 50.),
                "4 J against a 50 J source exceeds the 1 % bound");
}

void test_large_constraint_work_cannot_improve_residual_quality() {
    // energy_quality never reads solver_constraint_work_j: the numerical
    // bucket cannot normalize its own residual away.
    const auto quality = [](double solver) {
        return bike::research::energy_quality(
            runtime::WireObject{
                {"energy_scale_j", runtime::Wire(1.)},
                {"active_work_j", runtime::Wire(100.)},
                {"source_positive_work_j", runtime::Wire(100.)},
                {"external_work_j", runtime::Wire(0.)},
                {"residual_j", runtime::Wire(10.)},
                {"solver_constraint_work_j", runtime::Wire(solver)}},
            .05);
    };
    const auto a = quality(0.), b = quality(100000.);
    bt::require(a.residual_ratio == b.residual_ratio,
                "solver constraint work never enters the residual ratio");
    bt::require(!a.acceptable && !b.acceptable,
                "an over-budget residual stays unacceptable either way");
}

void test_motor_work_reports_signed_and_positive_separately() {
    const std::array forward{50.}, braking{-10.};
    const std::span<const double> empty;
    runtime::StepWork work = runtime::step_work(forward, -80., empty, empty, .01);
    bt::require(work.motor_signed_j == -.8, "negative motor work stays signed");
    bt::require(work.motor_positive_j == 0., "regenerating motor adds no source");
    work = runtime::step_work(braking, 250., empty, empty, .01);
    bt::require(work.motor_signed_j == 2.5, "motor signed work");
    bt::require(work.motor_positive_j == 2.5, "motor positive work");
    bt::require(work.muscle_signed_j == -.1, "muscle signed work");
}

void test_invalid_work_inputs_are_rejected() {
    const std::array nan_muscle{std::numeric_limits<double>::quiet_NaN()};
    const std::array one{0.};
    const std::span<const double> empty;
    bt::invalid([&] { (void)runtime::step_work(nan_muscle, 0., empty, empty, .01); });
    bt::invalid([&] {
        (void)runtime::step_work(one,
                                 std::numeric_limits<double>::infinity(),
                                 empty, empty, .01);
    });
    bt::invalid([&] { (void)runtime::step_work(one, 0., empty, empty, 0.); });
    bt::invalid([&] { (void)runtime::step_work(one, 0., empty, empty, -.1); });
}

void test_constraint_budget_uses_roundoff_floor_when_no_source() {
    bt::require(runtime::constraint_work_ok(1e-9, 0.),
                "sub-roundoff work passes with no source");
    bt::require(!runtime::constraint_work_ok(1e-7, 0.),
                "over-roundoff work fails with no source");
    bt::require(runtime::constraint_work_ok(.009, .9),
                "under 1 % of the source passes");
    bt::require(!runtime::constraint_work_ok(.02, .9),
                "over 1 % of the source fails");
    for (const auto &[absolute, source] :
         std::array{std::pair{-.1, 1.}, std::pair{1., -.1},
                    std::pair{std::numeric_limits<double>::quiet_NaN(), 1.}}) {
        bt::invalid([&] { (void)runtime::constraint_work_ok(absolute, source); });
    }
}

// ---- records 6/7: the closing-tail budget against neutral raws --------------

void test_runtime_constraint_budget_checks_closing_tail_with_fixed_roundoff() {
    for (const auto &[source, absolute, allowed] :
         std::array{std::tuple{1., .0099, true}, std::tuple{1., .0101, false},
                    std::tuple{0., .99e-8, true}, std::tuple{0., 1.01e-8, false}}) {
        const auto runtime = runtime_for_raws();
        // Two opposite-signed closure intervals have zero signed work but
        // the required absolute work. A partial period must pass the same
        // integral gate.
        int i = 0;
        for (const double sign : {1., -1.}) {
            runtime->push(neutral_raw(
                i++, {}, {{"act_rider_probe", {source / .001}},
                          {"closure", {sign * absolute / .001}}},
                std::vector<double>{1.}));
        }
        if (allowed) {
            runtime->flush();
            bt::require(!runtime->state().monitor.first_failure.has_value(),
                        "an allowed tail keeps first_failure clear");
        } else {
            bool rejected = false;
            try {
                runtime->flush();
            } catch (const runtime::InvalidReferenceRun &failure) {
                rejected = std::string(failure.what())
                                   .find("energy.constraint_work") !=
                           std::string::npos;
            }
            bt::require(rejected,
                        "an over-budget tail raises energy.constraint_work");
            const auto &first = runtime->state().monitor.first_failure;
            bt::require(first.has_value() && first->time_s == .001 &&
                            first->violations ==
                                std::vector<std::string>{"energy.constraint_work"},
                        "first_failure pins the closing interval and channel");
            const auto &sample = runtime->pending_samples().front();
            bt::require(wire::array(wire::required(sample->channels,
                                                 "attachment_violations"))
                            .empty(),
                        "an energy rejection is not an attachment violation");
        }
        const auto &energy = runtime->state().energy;
        bt::require(wire::boolean(wire::required(
                        runtime->state().energy_channels, "constraint_work_ok")) == allowed,
                    "the committed flag mirrors the budget verdict");
        bt::require(approx(energy.constraint_absolute_j, absolute),
                    "constraint_absolute_j accumulates absolute row work");
        bt::require(approx(energy.solver_work_j, 0., 1e-6, 1e-15),
                    "opposite rows cancel in the signed accumulator");
        bt::require(approx(energy.muscle_positive_j + energy.motor_positive_j,
                           source),
                    "source_positive_work_j accumulates the probe source");
        bt::require(approx(runtime->state().history.duration_s, .001),
                    "the work history integrates the committed tail");
    }
}

void test_runtime_zero_source_tolerance_does_not_grow_with_episode_steps() {
    // The oracle sets runtime.sim.steps = 100000 to pin that the S8
    // zero-source roundoff budget is absolute and never scales with step
    // count. PeriodAccounting holds no step counter — the bound is a fixed
    // constant — so the reachable contract is the rejection itself plus the
    // accumulated absolute work.
    const auto runtime = runtime_for_raws();
    runtime->push(neutral_raw(0, {}, {{"closure", {1.e-4}}},
                              std::vector<double>{1.}));
    bool rejected = false;
    try {
        runtime->flush();
    } catch (const runtime::InvalidReferenceRun &failure) {
        rejected = std::string(failure.what()).find("energy.constraint_work") !=
                   std::string::npos;
    }
    bt::require(rejected, "zero-source over-roundoff work rejects the flush");
    bt::require(approx(runtime->state().energy.constraint_absolute_j, 5.e-8),
                "the absolute accumulator holds force*qvel*dt exactly");
}

// ---- record 8: solver row partition on synthetic plants ---------------------

void test_actual_solver_rows_partition_without_missing_or_duplicate_forces(
    std::string_view kind) {
    std::string xml;
    if (kind == "contact") {
        xml = R"XML(<worldbody><geom type="plane" size="1 1 .1"/>
<body pos="0 0 .09"><joint type="slide" axis="0 0 1"/>
<geom type="sphere" size=".1" mass="1"/></body></worldbody>)XML";
    } else {
      xml =
          std::string(
              R"(<worldbody><body><joint name="a" type="slide" axis="1 0 0" )") +
          (kind == "joint" ? R"(limited="true" range="-.1 .1")" : "") +
          R"XML(/><inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body>
<body><joint name="b" type="slide" axis="1 0 0"/>
<inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody>)XML";
      if (kind == "tendon")
        xml += R"XML(<tendon><fixed limited="true" range="-100 0">
<joint joint="a" coef="1"/><joint joint="b" coef="-1"/></fixed></tendon>)XML";
      else if (kind == "equality")
        xml += R"XML(<equality><joint joint1="a" joint2="b"/></equality>)XML";
    }
    const auto model =
        load_xml("<mujoco><option gravity=\"0 0 0\"/>" + xml + "</mujoco>",
                 "partition model");
    const auto data = data_of(model.get());
    if (kind != "contact")
        model_access::mutable_buffer(data->qpos, model->nq)[0] = .2;
    engine::forward(model.get(), data.get());
    std::vector<double> weights(static_cast<std::size_t>(data->nefc));
    const auto parts = runtime::constraint_components(model.get(), data.get(), weights);
    const std::string target =
        kind == "contact"     ? "native_contact"
        : kind == "equality"  ? "closure"
                              : "joint_limits";
    const auto qfrc_constraint =
        model_access::readonly_buffer(data->qfrc_constraint, model->nv);
    bt::require(norm(qfrc_constraint) > 1.,
                "the fixture activates a real constraint force");
    std::vector<double> folded(static_cast<std::size_t>(model->nv), 0.);
    bool found = false;
    for (const auto &[name, force] : parts) {
        bt::require(force.size() == static_cast<std::size_t>(model->nv),
                    "every partition is nv-sized");
        for (std::size_t i = 0; i < force.size(); ++i) folded[i] += force[i];
        if (name == target) {
            found = true;
            bt::require(allclose(span_of(force), qfrc_constraint, 1e-12, 1e-12),
                        "the target partition carries the whole constraint force");
        } else {
            bt::require(std::ranges::all_of(force, [](double v) { return v == 0.; }),
                        "non-target partitions are exactly empty");
        }
    }
    bt::require(found, "the partition publishes the target bucket");
    bt::require(allclose(span_of(folded), qfrc_constraint, 1e-12, 1e-12),
                "the partitions fold to qfrc_constraint without loss");
}

// ---- record 9: equilibrium snapshot/restore on the actual constraint --------

void test_cached_equilibrium_restores_the_actual_one_way_constraint_preload() {
    const auto model = load_xml(R"XML(<mujoco><option gravity="0 0 0"/>
<worldbody><body><joint name="a" type="slide" axis="1 0 0"/>
<inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body>
<body><joint name="b" type="slide" axis="1 0 0"/>
<inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody>
<tendon><fixed name="coupling" limited="true" range="-10 0"><joint joint="a" coef="1"/>
<joint joint="b" coef="-1"/></fixed></tendon></mujoco>)XML",
                                "coupling model");
    const auto data = data_of(model.get());
    // IdealFreehubConstraint(model, 1., ...): the oracle's unit ratio —
    // 3/3 keeps the validated minimum teeth with ratio exactly 1.
    drivetrain::Transmission hub(model.get(),
        drivetrain::GearingConfig{.front_teeth = 3, .rear_teeth = 3,
                                  .chain_pitch_m = .0127},
        false, "coupling", "a", "b");
    // hub.boundary=-.001; model.tendon_range[hub.tendon_id,1]=hub.boundary —
    // the two oracle writes map to the snapshot boundary + range lanes.
    auto preload = hub.state();
    preload.boundary = -.001;
    preload.range[1] = -.001;
    hub.restore(std::move(preload));
    engine::forward(model.get(), data.get());
    const std::vector<double> before =
        copy(model_access::readonly_buffer(data->qacc, model->nv));
    // _state_arrays(runtime): only the drive boundaries persist when rider/
    // tire/clutch are absent — the TransmissionSnapshot is the native lane.
    const drivetrain::TransmissionSnapshot saved = hub.state();
    hub.reset(data.get());
    engine::forward(model.get(), data.get());
    const auto qacc = model_access::readonly_buffer(data->qacc, model->nv);
    bt::require(!allclose(qacc, span_of(before)),
                "reset releases the preloaded one-way boundary");
    bt::require(hub.restore(saved), "restore_state returns true");
    engine::forward(model.get(), data.get());
    bt::require(allclose(model_access::readonly_buffer(data->qacc, model->nv),
                         span_of(before), 0., 1e-12),
                "restore replays the preloaded acceleration bitwise-close");
    const auto boundary = hub.state().boundary;
    bt::require(boundary.has_value() && *boundary == -.001,
                "the snapshot boundary survives the round-trip");
    const int tendon_id =
        mj_name2id(model.get(), mjOBJ_TENDON, "coupling");
    const auto range = model_access::readonly_buffer(
        model->tendon_range, 2 * model->ntendon);
    bt::require(tendon_id >= 0 &&
                    range[2 * static_cast<std::size_t>(tendon_id) + 1] == -.001,
                "the live tendon row keeps the restored boundary");
}

// ---- record 10: opposite-signed rows never cancel in the absolute channel ---
// Closure rows (equality welds and tendon-limit one-way closures such as the
// freehub) charge the elastic closure channel; joint-limit rows stay numerical
// constraint work and trip the closing-tail rejection.

void test_opposite_actual_closure_work_cannot_cancel_inside_one_interval(
    std::string_view kind) {
    std::string xml =
        R"XML(<mujoco><option gravity="0 0 0" timestep=".0005"/>
<worldbody><body><joint name="a" type="slide" axis="1 0 0"/><geom size=".1" mass="1" contype="0" conaffinity="0"/></body>
<body><joint name="b" type="slide" axis="1 0 0"/><geom size=".1" mass="1" contype="0" conaffinity="0"/></body></worldbody>
<equality><joint name="one" joint1="a" solref="-100 0"/><joint name="two" joint1="b" solref="-100 0"/></equality></mujoco>)XML";
    if (kind == "tendon") {
        xml = R"XML(<mujoco><option gravity="0 0 0" timestep=".0005"/>
<worldbody>)XML";
        for (const char name : {'a', 'b', 'c', 'd'})
            xml += std::string("<body><joint name=\"") + name +
                   R"XML(" type="slide" axis="1 0 0"/><geom size=".1" mass="1" contype="0" conaffinity="0"/></body>)XML";
        xml += R"XML(</worldbody><tendon>)XML";
        for (const auto &[a, b] : std::array{std::pair{'a', 'b'}, std::pair{'c', 'd'}})
          xml +=
              std::string(
                  R"(<fixed limited="true" range="-10 0" solreflimit="-100 0">)") +
              "<joint joint=\"" + a + R"(" coef="1"/>)" + "<joint joint=\"" +
              b + R"(" coef="-1"/></fixed>)";
        xml += "</tendon></mujoco>";
    } else if (kind == "joint") {
        xml = R"XML(<mujoco><option gravity="0 0 0" timestep=".0005"/>
<worldbody>)XML";
        for (const char name : {'a', 'b'})
            xml += std::string("<body><joint name=\"") + name +
                   R"XML(" type="slide" axis="1 0 0" limited="true" range="-.1 .1" solreflimit="-100 0"/><geom size=".1" mass="1" contype="0" conaffinity="0"/></body>)XML";
        xml += "</worldbody></mujoco>";
    }
    const auto model = load_xml(xml, "opposite-work model");
    const auto data = data_of(model.get());
    const auto qpos = model_access::mutable_buffer(data->qpos, model->nq);
    const auto qvel = model_access::mutable_buffer(data->qvel, model->nv);
    if (kind == "equality") {
        qpos[0] = .1; qpos[1] = -.1;
        qvel[0] = 1.; qvel[1] = 1.;
    } else if (kind == "joint") {
        // Both past the upper limit: one moving into it, one moving out.
        qpos[0] = .15; qpos[1] = .15;
        qvel[0] = 1.; qvel[1] = -1.;
    } else {
        qpos[0] = .1; qpos[1] = 0.; qpos[2] = .1; qpos[3] = 0.;
        qvel[0] = 1.; qvel[1] = 0.; qvel[2] = -1.; qvel[3] = 0.;
    }
    engine::forward(model.get(), data.get());
    std::vector<double> weights(static_cast<std::size_t>(data->nefc));
    const auto parts = runtime::constraint_components(model.get(), data.get(), weights);
    const std::string bucket = kind == "equality" ? "closure" : "joint_limits";
    bool found = false;
    for (const auto &[name, force] : parts)
        if (name == bucket) {
            found = true;
            bt::require(approx(dot(span_of(force),
                                 model_access::readonly_buffer(data->qvel,
                                                               model->nv)),
                               0., 1e-6, 1e-12),
                        "the partition's signed power cancels to zero");
        }
    bt::require(found, "the partition publishes the work bucket");
    std::vector<double> velocity(static_cast<std::size_t>(data->nefc));
    const auto powers = runtime::numerical_constraint_powers(
        model.get(), data.get(),
        model_access::readonly_buffer(data->qvel, model->nv), velocity);
    bt::require(powers.size() == 2, "two independent rows publish powers");
    double minimum = powers.front().second, maximum = minimum;
    for (const auto &[key, power] : powers) {
        minimum = std::min(minimum, power);
        maximum = std::max(maximum, power);
    }
    bt::require(minimum < 0. && maximum > 0.,
                "the two rows carry opposite-signed power");
    const auto runtime = runtime_for_raws();
    auto raw = neutral_raw(0);
    raw.qpos = copy(model_access::readonly_buffer(data->qpos, model->nq));
    raw.qvel = copy(model_access::readonly_buffer(data->qvel, model->nv));
    raw.qvel_mid = raw.qvel;
    raw.components = parts;
    raw.numerical_constraint_power_w = powers;
    raw.constraint_snapshot.qvel_start = raw.qvel;
    runtime->push(std::move(raw));
    double absolute_power = 0.;
    for (const auto &[key, power] : powers) absolute_power += std::abs(power);
    // F1 re-scope: equality and tendon-limit rows charge the elastic closure
    // channel — tracked signed/absolute but outside the numerical bound;
    // joint-limit rows stay numerical constraint work and still trip the
    // closing-tail rejection.
    const auto &energy = runtime->state().energy;
    if (kind != "joint") {
        runtime->flush();
        bt::require(approx(energy.closure_signed_j, 0., 1e-6, 1e-12),
                    "opposite rows cancel in the closure signed channel");
        bt::require(approx(energy.closure_absolute_j, absolute_power * .0005),
                    "closure_absolute_j keeps every row's absolute work");
        bt::require(energy.closure_absolute_j > 1e-8,
                    "the closure magnitude stays physically real");
    } else {
        bool rejected = false;
        try {
            runtime->flush();
        } catch (const runtime::InvalidReferenceRun &failure) {
            rejected = std::string(failure.what())
                               .find("energy.constraint_work") !=
                       std::string::npos;
        }
        bt::require(rejected, "uncancellable joint-limit work rejects the flush");
        bt::require(approx(energy.solver_work_j, 0., 1e-6, 1e-12),
                    "opposite rows cancel in the signed channel");
        bt::require(approx(energy.constraint_absolute_j, absolute_power * .0005),
                    "constraint_absolute_j keeps every row's absolute work");
        bt::require(energy.constraint_absolute_j > 1e-8,
                    "the constraint magnitude stays physically real");
    }
}

// ---- record 17: rider end stops are a loss line (native-only) ---------------

// Launch part 4, decision 7: joint-limit rows of rider joints are the
// anatomical end stops — exactly the limited joints whose name starts with
// "rider_" — and the welded model limits exactly the envelope joints.
void test_rider_end_stops_are_the_envelope_joints() {
    // Compile the welded articulated model from viewer_physics_welded.toml
    // (its envelope map and its lock_ankles value, as the ride does —
    // test_pinned_topology.cpp's compiled() assembly).
    const auto physics = cfg::load_physics(
        rr::repo_file("examples/research/viewer_physics_welded.toml"));
    mdl::BuildRequest request;
    request.mode = "ride";
    request.include_rider = true;
    request.crank_joint = true;
    request.rider = mdl::resolve_rider(
        cfg::RiderSpecs{.variant = "articulated_planar",
                        .inseam_m = std::nullopt},
        true, "lumped");
    request.physics = physics;
    const mdl::ModelOwner model =
        mdl::compile_mjcf(mdl::generate_mjcf(request));
    // From the compiled mjModel collect the joints with jnt_limited set and
    // a name starting with "rider_"; require that set to equal the
    // rider_-prefixed joints of configuration::kRiderJoints present in the
    // model (ankles are absent when lock_ankles holds), and require that no
    // non-rider joint name starts with "rider_" — rider_root_* are the
    // pelvis's free-body surrogate, excluded exactly like the actuator and
    // limit emitters do (articulated_rider.cpp).
    const auto limited = model_access::readonly_buffer(model->jnt_limited,
                                                       model->njnt);
    std::set<std::string> limited_rider, expected;
    for (int j = 0; j < model->njnt; ++j) {
        const char *name = mj_id2name(model.get(), mjOBJ_JOINT, j);
        if (name == nullptr) continue;
        const std::string_view joint(name);
        if (!joint.starts_with("rider_")) continue;
        if (joint.starts_with("rider_root_")) {
            bt::require(limited[static_cast<std::size_t>(j)] == 0,
                        "the free-body root carriers are not end stops");
            continue;
        }
        bt::require(std::ranges::find(cfg::kRiderJoints, joint) !=
                        cfg::kRiderJoints.end(),
                    "no non-rider joint name starts with rider_");
        if (limited[static_cast<std::size_t>(j)] != 0)
            limited_rider.emplace(joint);
    }
    for (const std::string_view joint : cfg::kRiderJoints)
        if (mj_name2id(model.get(), mjOBJ_JOINT,
                       std::string(joint).c_str()) >= 0)
            expected.emplace(joint);
    bt::require(expected == std::set<std::string>{
                                "rider_torso_hinge",
                                "rider_shoulder_left", "rider_shoulder_right",
                                "rider_elbow_left", "rider_elbow_right",
                                "rider_hip_front", "rider_hip_rear",
                                "rider_knee_front", "rider_knee_rear"},
                "the spindle pedal attachment locks both ankles");
    bt::require(limited_rider == expected,
                "the welded model limits exactly the rider envelope joints");
}

// The end-stop rows leave the numerical constraint work and become a loss:
// loss_j and loss_step_j carry -sum(power) * dt of the rider_end_stop rows,
// rider_end_stop_loss_step_j publishes it, and constraint_absolute_j no
// longer counts them; joint_limit rows of bike joints stay numerical.
void test_end_stop_work_is_a_loss() {
    // The plan's dt = .001 — runtime_for_raws is fixed at kDt, so the same
    // detached accounting is assembled with the task's timestep.
    runtime::AccountingConfig config;
    config.timestep_s = .001;
    config.period_steps = 10;
    config.measurement_decimation = 1;
    config.retention = runtime::RetentionMode::legacy;
    runtime::AccountingState initial;
    initial.energy.initial_energy_j = 0.;
    initial.energy.energy_scale_j = 1.;
    initial.monitor.strict = true;
    const auto accounting = std::make_unique<runtime::PeriodAccounting>(
        config, initial, /*generation=*/1,
        /*controller=*/nullptr, /*contacts=*/nullptr);
    auto raw = neutral_raw(0);
    raw.time_s = 0.;
    raw.end_time_s = .001;
    raw.constraint_snapshot.interval_start_s = 0.;
    raw.constraint_snapshot.interval_end_s = .001;
    raw.qvel = {1.};
    raw.qvel_mid = {1.};
    raw.constraint_snapshot.qvel_start = raw.qvel;
    raw.loss_step_j = .001;
    // human_crank feeds source_positive_work_j so the .0005 J joint-limit
    // remainder clears the 1 % strict bound — the end-stop's .004 J would
    // trip the closing-tail rejection if it were still numerical work.
    raw.components = {{"human_crank", {60.}}};
    raw.numerical_constraint_power_w =
        {{"rider_end_stop:12", -4.}, {"joint_limit:3", .5}, {"equality:0", 1.}};
    accounting->push(std::move(raw));
    accounting->flush();
    const auto &energy = accounting->state().energy;
    const auto &channels = accounting->state().energy_channels;
    bt::require(wire::number(wire::required(channels,
                                          "rider_end_stop_loss_step_j")) == .004,
                "the end-stop loss is -sum(power) * dt");
    bt::require(wire::number(wire::required(channels, "loss_step_j")) == .005,
                "loss_step_j carries the raw's loss plus the end-stop loss");
    bt::require(energy.loss_j == .005,
                "loss_j accumulates the same charge");
    bt::require(energy.constraint_absolute_j == .0005,
                "only the bike joint-limit row stays numerical");
    bt::require(energy.closure_signed_j == .001,
                "the equality row still charges the closure channel");
    bt::require(energy.closure_absolute_j == .001,
                "closure_absolute_j keeps the equality row's work");
}

// ---- records 11-14: physical energy on direct MuJoCo plants -----------------

[[nodiscard]] std::unique_ptr<mjModel, decltype(&mj_deleteModel)>
free_body(std::string_view extra) {
  return load_xml(R"(<mujoco><option gravity="0 0 -9.81" timestep="0.0005"/>)" +
                      std::string(extra) + "</mujoco>",
                  "free-body model");
}

// _energy(model, data): kinetic + gravitational potential.
[[nodiscard]] double energy_of(const mjModel *model, const mjData *data) {
    std::vector<double> mv(static_cast<std::size_t>(model->nv), 0.);
    engine::mul_m(model, data, mv.data(), data->qvel);
    const double kinetic = .5 * dot(span_of(mv),
        model_access::readonly_buffer(data->qvel, model->nv));
    const auto masses = model_access::readonly_buffer(model->body_mass,
                                                      model->nbody);
    const auto positions = model_access::readonly_buffer(
        data->xipos, model_access::kXYZ * model->nbody);
    const auto gravity =
        model_access::readonly_buffer(model->opt.gravity, model_access::kXYZ); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
    double potential = 0.;
    for (int body = 0; body < model->nbody; ++body)
        for (std::size_t axis = 0; axis < 3; ++axis)
            potential -= static_cast<double>(masses[static_cast<std::size_t>(body)]) *
                         static_cast<double>(positions[3 * static_cast<std::size_t>(body) + axis]) *
                         static_cast<double>(gravity[axis]);
    return kinetic + potential;
}

// _row_work: work of the selected efc rows over the interval — qfrc.v_mid.dt.
[[nodiscard]] double row_work(const mjModel *model, const mjData *data,
                              std::span<const double> v_mid,
                              const std::function<bool(int)> &keep) {
    const int n = data->nefc;
    const auto types = model_access::readonly_buffer(data->efc_type, n);
    const auto forces = model_access::readonly_buffer(data->efc_force, n);
    std::vector<double> multipliers(static_cast<std::size_t>(n), 0.);
    for (int i = 0; i < n; ++i)
        if (keep(types[static_cast<std::size_t>(i)]))
            multipliers[static_cast<std::size_t>(i)] =
                forces[static_cast<std::size_t>(i)];
    std::vector<double> qfrc(static_cast<std::size_t>(model->nv), 0.);
    engine::mul_jac_t_vec(model, data, qfrc.data(), multipliers.data());
    return dot(span_of(qfrc), v_mid) * model->opt.timestep;
}

[[nodiscard]] double equality_work(const mjModel *model, const mjData *data,
                                   std::span<const double> v_mid) {
    return row_work(model, data, v_mid,
                    [](int type) { return type == mjCNSTR_EQUALITY; });
}

[[nodiscard]] double contact_work(const mjModel *model, const mjData *data,
                                  std::span<const double> v_mid) {
    return row_work(model, data, v_mid, [](int type) {
        return type == mjCNSTR_CONTACT_FRICTIONLESS ||
               type == mjCNSTR_CONTACT_ELLIPTIC ||
               type == mjCNSTR_CONTACT_PYRAMIDAL;
    });
}

void test_free_fall_keeps_energy_without_constraint_work() {
    const auto model = free_body(
        R"XML(<worldbody><body pos="0 0 2"><freejoint/>
<inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
<geom size=".1" contype="0" conaffinity="0"/></body></worldbody>)XML");
    const auto data = data_of(model.get());
    engine::forward(model.get(), data.get());
    const double e0 = energy_of(model.get(), data.get());
    for (int i = 0; i < 200; ++i) {
        engine::step(model.get(), data.get());
        bt::require(data->nefc == 0, "free fall activates no constraint rows");
    }
    bt::require(approx(energy_of(model.get(), data.get()), e0, 1e-3),
                "free fall conserves total energy");
}

void test_closed_ideal_constraint_does_no_work() {
    const auto model = free_body(R"XML(<worldbody>
<body name="a" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
  <geom size=".05" contype="0" conaffinity="0"/></body>
<body name="b" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
  <geom size=".05" contype="0" conaffinity="0"/></body></worldbody>
<equality><weld body1="a" body2="b" solref="0.005 1"/></equality>)XML");
    for (double &axis :
         model_access::mutable_buffer(model->opt.gravity, model_access::kXYZ)) // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        axis = 0.;
    const auto data = data_of(model.get());
    // data.qvel[:6] = data.qvel[6:12] = [.2, 0., .1, 0., .3, 0.]
    const std::array velocity{.2, 0., .1, 0., .3, 0.};
    const auto qvel = model_access::mutable_buffer(data->qvel, model->nv);
    for (std::size_t half = 0; half < 2; ++half)
        for (std::size_t i = 0; i < velocity.size(); ++i)
            qvel[half * 6 + i] = velocity[i];
    engine::forward(model.get(), data.get());
    const double e0 = energy_of(model.get(), data.get());
    double absolute = 0.;
    for (int i = 0; i < 200; ++i) {
        std::vector<double> v_mid =
            copy(model_access::readonly_buffer(data->qvel, model->nv));
        engine::step(model.get(), data.get());
        const auto qvel_after =
            model_access::readonly_buffer(data->qvel, model->nv);
        for (std::size_t j = 0; j < v_mid.size(); ++j)
            v_mid[j] = .5 * (v_mid[j] + qvel_after[j]);
        absolute += std::abs(equality_work(model.get(), data.get(), span_of(v_mid)));
    }
    bt::require(approx(energy_of(model.get(), data.get()), e0, 1e-6),
                "a merged weld pair conserves energy");
    bt::require(absolute < 1e-6 * e0,
                "the closed equality rows do negligible work");
}

void test_inelastic_impact_loss_is_physical_not_constraint_credit() {
    const auto model = free_body(R"XML(<worldbody>
<body name="a" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
  <geom size=".05" contype="0" conaffinity="0"/></body>
<body name="b" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
  <geom size=".05" contype="0" conaffinity="0"/></body></worldbody>
<equality><weld body1="a" body2="b" solref="0.005 1"/></equality>)XML");
    for (double &axis :
         model_access::mutable_buffer(model->opt.gravity, model_access::kXYZ)) // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        axis = 0.;
    const auto data = data_of(model.get());
    const double dv = .8;
    const auto qvel = model_access::mutable_buffer(data->qvel, model->nv);
    qvel[0] = .5; qvel[6] = .5 - dv;
    engine::forward(model.get(), data.get());
    const double mass = 5.;
    const double analytic_loss = .25 * mass * dv * dv;
    double work = 0.;
    for (int i = 0; i < 2000; ++i) {
        std::vector<double> v_mid =
            copy(model_access::readonly_buffer(data->qvel, model->nv));
        engine::step(model.get(), data.get());
        const auto qvel_after =
            model_access::readonly_buffer(data->qvel, model->nv);
        for (std::size_t j = 0; j < v_mid.size(); ++j)
            v_mid[j] = .5 * (v_mid[j] + qvel_after[j]);
        work += equality_work(model.get(), data.get(), span_of(v_mid));
    }
    const double e1 = energy_of(model.get(), data.get());
    bt::require(approx(qvel[0], qvel[6], 1e-6, 1e-3),
                "the inelastic merge converges to one velocity");
    bt::require(approx(-work, analytic_loss, .05),
                "the equality rows carry exactly the engagement loss");
    bt::require(approx(e1, .5 * mass * .5 * .5 + .5 * mass * (.5 - dv) * (.5 - dv) -
                             analytic_loss,
                       .02),
                "total energy falls by the engagement loss alone");
}

void test_friction_loss_is_nonnegative_and_consistent() {
    const auto model = free_body(R"XML(<worldbody>
<geom name="floor" type="plane" size="5 5 1" pos="0 0 0" friction="0.5 0.5 0.5"/>
<body name="block" pos="0 0 0.1"><joint type="slide" axis="1 0 0"/>
  <joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="2" diaginertia="1 1 1"/>
  <geom type="box" size=".1 .1 .1" friction="0.5 0.5 0.5"/></body></worldbody>)XML");
    const auto data = data_of(model.get());
    const auto qvel = model_access::mutable_buffer(data->qvel, model->nv);
    qvel[0] = 2.; qvel[1] = 0.;
    engine::forward(model.get(), data.get());
    const double mass = 2., v0 = 2.;
    const double kinetic0 = .5 * mass * v0 * v0;
    double work = 0.;
    for (int i = 0; i < 400; ++i) {
        std::vector<double> v_mid =
            copy(model_access::readonly_buffer(data->qvel, model->nv));
        engine::step(model.get(), data.get());
        const auto qvel_after =
            model_access::readonly_buffer(data->qvel, model->nv);
        for (std::size_t j = 0; j < v_mid.size(); ++j)
            v_mid[j] = .5 * (v_mid[j] + qvel_after[j]);
        work += contact_work(model.get(), data.get(), span_of(v_mid));
    }
    const double kinetic1 = .5 * mass * qvel[0] * qvel[0];
    const double loss = kinetic0 - kinetic1;
    bt::require(loss > 0., "sliding friction dissipates kinetic energy");
    bt::require(approx(-work, loss, .1),
                "contact rows carry the frictional loss");
    bt::require(kinetic1 < kinetic0, "the model cannot create energy");
}

// test_closure_stored_energy_balances_load_unload_cycle — one load/unload
// cycle of a one-way drivetrain closure: a light driver presses a heavy
// driven side through a limited fixed tendon (the freehub/crank-clutch
// shape). The compliant constraint parks elastic take-up energy while the
// load holds and returns it on release; only the damper's share is lost.
// With the closure term inside the energy, the ledger's uncovered gap
// stays within that dissipated share; without it the whole parked amount
// reads as missing energy.
void test_closure_stored_energy_balances_load_unload_cycle() {
    const auto model = load_xml(
        R"XML(<mujoco><option gravity="0 0 0" timestep=".0005"/>
<worldbody>
<body><joint name="driver" type="slide" axis="1 0 0"/><inertial mass=".005" pos="0 0 0" diaginertia="1 1 1"/></body>
<body><joint name="driven" type="slide" axis="1 0 0"/><inertial mass="5" pos="0 0 0" diaginertia="1 1 1"/></body>
</worldbody>
<tendon><fixed name="freehub_like" limited="true" range="-10 0" solreflimit=".004 1">
<joint joint="driver" coef="1"/><joint joint="driven" coef="-1"/></fixed></tendon>
</mujoco>)XML",
        "closure-cycle model");
    const auto data = data_of(model.get());
    engine::forward(model.get(), data.get());
    const double dt = model->opt.timestep;
    // Ramp up, hold, ramp down: the freehub take-up/release shape.
    const auto force_of = [](double t) {
        if (t < .15) return 60. * t / .15;
        if (t < .30) return 60.;
        if (t < .45) return 60. * (1. - (t - .30) / .15);
        return 0.;
    };
    const auto qvel = model_access::mutable_buffer(data->qvel, model->nv);
    const auto applied =
        model_access::mutable_buffer(data->qfrc_applied, model->nv);
    double work = 0., previous = 0.;
    double peak_stored = 0., gap_with = 0., gap_without = 0.;
    for (int i = 0; i < 1200; ++i) {
        const double force = force_of(data->time);
        std::vector<double> v_mid =
            copy(model_access::readonly_buffer(data->qvel, model->nv));
        applied[0] = force;
        engine::step(model.get(), data.get());
        for (std::size_t j = 0; j < v_mid.size(); ++j)
            v_mid[j] = .5 * (v_mid[j] + qvel[j]);
        work += .5 * (previous + force) * v_mid[0] * dt;
        previous = force;
        const double stored =
            runtime::closure_stored_energy(data.get());
        const double energy = energy_of(model.get(), data.get());
        peak_stored = std::max(peak_stored, stored);
        gap_with =
            std::max(gap_with, std::abs(energy + stored - work));
        gap_without = std::max(gap_without, std::abs(energy - work));
    }
    bt::require(peak_stored > .1,
                "the load phase parks real elastic energy");
    bt::require(gap_without > .5 * peak_stored,
                "without the term the parked energy reads as missing");
    bt::require(gap_with < .4 * peak_stored,
                "with the term only the dissipated share stays open");
}

// ---- record 16: interval-midpoint work quadrature (native-only) --------
// The +48 J ledger drift (.scratch/devin/2026-11-06-diag-48j/report.md):
// every integrated F·v·dt channel charges the interval-midpoint velocity
// .5*(v_start + v_end) — the same convention records 12-15 apply by hand —
// because mechanical_energy_j is the POST-step energy. Start velocity
// under-integrates each channel by .5*F*dv*dt (first order in dt); end
// velocity would over-integrate by the same amount, so the midpoint is
// the only second-order quadrature consistent with the endpoint state.
void test_work_ledger_integrates_at_the_interval_midpoint() {
    const auto accounting = runtime_for_raws();
    // One dof, forces constant over the interval: v_start=0, v_end=2, so
    // the quadrature sees qvel_mid=1. mechanical_energy_j is the post-
    // step total set to exactly the midpoint work sum, so the residual
    // vanishes only under the midpoint convention.
    auto step = neutral_raw(0);
    step.qvel = {0.};
    step.qvel_mid = {1.};
    step.components = {{"external", {100.}},
                       {"human_crank", {200.}},
                       {"mid_drive", {50.}}};
    constexpr double dt = strictness_support::kDt;
    step.mechanical_energy_j = (100. + 200. + 50.) * dt;
    step.constraint_snapshot.qvel_start = step.qvel;
    accounting->push(std::move(step));
    accounting->flush();
    const auto &energy = accounting->state().energy;
    bt::require(energy.external_work_j == 100. * dt,
                "external work integrates the midpoint velocity");
    bt::require(energy.muscle_signed_j == 200. * dt &&
                energy.motor_signed_j == 50. * dt,
                "muscle and motor work integrate the midpoint velocity");
    bt::require(std::abs(energy.residual_j) < 1e-15,
                "midpoint work balances the post-step energy exactly");
    const auto &sample = accounting->pending_samples().front();
    const auto &component_work = wire::object(
        wire::required(sample->channels, "component_work_j"));
    bt::require(wire::number(wire::required(component_work, "external")) ==
                    100. * dt,
                "component_work_j accumulates at the midpoint velocity");
    // A midpoint velocity inconsistent with the interval width is
    // rejected at close — the producer must always stage it.
    const auto broken = runtime_for_raws();
    auto bad = neutral_raw(0);
    bad.qvel = {0., 0.};
    bad.qvel_mid = {0.};
    bad.constraint_snapshot.qvel_start = bad.qvel;
    broken->push(std::move(bad));
    bt::invalid([&] { broken->flush(); });
}

// _environment's flat probe track (test_closed_form_rider.cpp's shape).
bike::terrain::TrackSpec flat_probe_track() {
    return bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = 300.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [300.0, 0.0]] }\n"));
}

// publish_raw stages .5*(v_start + v_end) itself — the production fill
// the fixture leg above bypasses by staging the field directly.
void test_publish_raw_carries_the_interval_midpoint_velocity() {
    const auto rig = rr::pinned_rig({}, flat_probe_track());
    // A few control ticks so the drivetrain and wheels carry speed — the
    // midpoint is only exercised where start and end velocities differ.
    (void)rr::advance_env_steps(*rig->physical, 4);
    const mjModel *m = rig->stepper->model();
    const mjData *d = rig->stepper->data();
    const auto before =
        copy(model_access::readonly_buffer(d->qvel, m->nv));
    const auto raw = rig->physical->advance_physics(
        0., 0., std::nullopt, rr::idle_control());
    const auto after =
        copy(model_access::readonly_buffer(d->qvel, m->nv));
    bt::require(raw.qvel_mid.size() == before.size(),
                "the raw carries an nv-wide midpoint velocity");
    double moved = 0.;
    bool midpoint = true, start = true;
    for (std::size_t i = 0; i < before.size(); ++i) {
        midpoint = midpoint &&
            raw.qvel_mid[i] == .5 * (before[i] + after[i]);
        start = start && raw.qvel[i] == before[i];
        moved = std::max(moved, std::abs(after[i] - before[i]));
    }
    bt::require(moved > 1e-9,
                "the interval actually accelerates some degree of freedom");
    bt::require(start, "raw.qvel stays the interval-start velocity");
    bt::require(midpoint, "raw.qvel_mid is .5*(v_start + v_end)");
}

} // namespace

// The model-compilation test inlines the BuildRequest/physics configuration
// into the bt::run lambda — multi-KiB of automatic storage, over the
// -Wframe-larger-than=8192 gate, exactly like test_pinned_topology.cpp.
// cppcheck-suppress unknownMacro // NATIVE_DIAG_* is the clang-dialect _Pragma wrapper from diag.hpp; cppcheck does not resolve it through -isystem
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_positive_body_work_is_not_net_joint_work();
        test_large_constraint_work_cannot_improve_residual_quality();
        test_motor_work_reports_signed_and_positive_separately();
        test_invalid_work_inputs_are_rejected();
        test_constraint_budget_uses_roundoff_floor_when_no_source();
        test_runtime_constraint_budget_checks_closing_tail_with_fixed_roundoff();
        test_runtime_zero_source_tolerance_does_not_grow_with_episode_steps();
        for (const std::string_view kind : {"tendon", "joint", "contact", "equality"})
            test_actual_solver_rows_partition_without_missing_or_duplicate_forces(kind);
        test_cached_equilibrium_restores_the_actual_one_way_constraint_preload();
        for (const std::string_view kind : {"equality", "tendon", "joint"})
            test_opposite_actual_closure_work_cannot_cancel_inside_one_interval(kind);
        test_rider_end_stops_are_the_envelope_joints();
        test_end_stop_work_is_a_loss();
        test_free_fall_keeps_energy_without_constraint_work();
        test_closed_ideal_constraint_does_no_work();
        test_inelastic_impact_loss_is_physical_not_constraint_credit();
        test_friction_loss_is_nonnegative_and_consistent();
        test_closure_stored_energy_balances_load_unload_cycle();
        test_work_ledger_integrates_at_the_interval_midpoint();
        test_publish_raw_carries_the_interval_midpoint_velocity();
    });
}
