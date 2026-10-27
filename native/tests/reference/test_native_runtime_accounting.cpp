// Completion 3.26 (batch runtime). Source: tests/reference/
// test_native_runtime_accounting.py — A4 independent work/contact gates,
// interval markers, airtime evidence, inter-tick effort observation, and
// complete-period strict publication (11 records, all ported).
//
// Pure-API checks exercise the same production functions the bindings wrap
// (step_work / constraint_work_ok / PeriodBuffer / WorkHistory). The
// inter-tick effort probe replays the welded capture's held rider evidence
// through observe_effort() and asserts the native-vs-native identity with
// SpindleController::solved_effort — the golden-qualified port of the
// oracle's rider_effort fold — so the probe==fold relation carries no
// Python dependency. The strict/diagnostic rides mutate the welded
// documents' weld-foot floor the same way the source mutates the resolved
// config; both comparisons are contracts (publication order, latched
// failure retention, warn-once), not oracle parity.
#include "runtime_reference_support.hpp"

#include "diag.hpp"
#include "rider_control_reference_support.hpp"

#include "rider/spindle_controller.hpp"
#include "rider/state_codec.hpp"
#include "engine_call.hpp"
#include "runtime/accounting.hpp"
#include "runtime/period_buffer.hpp"
#include "runtime/status.hpp"
#include "model_access.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <utility>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace bio = bike::io;
namespace rcr = rider_control_reference;
namespace sw = runtime::sample_wire;
using runtime_reference::compose_runtime;
using runtime_reference::expect_error;
using runtime_reference::expect_invalid;
using runtime_reference::member;
using runtime_reference::num;
using runtime_reference::resolved_docs;
using runtime_reference::set_member;
using runtime_reference::welded_dir;
using runtime_reference::wire_member;

constexpr double kClose = 1e-9;

// _native_runtime_support.assert_tree_close: unordered object members,
// positional arrays, strings/bools exact, numbers within atol/rtol 1e-9.
[[nodiscard]] bool tree_close(const runtime::Wire &actual,
                              const runtime::Wire &expected) {
    if (const auto *a = std::get_if<double>(&actual.value)) {
        const auto *e = std::get_if<double>(&expected.value);
        if (e != nullptr)
            return std::isfinite(*a) && std::isfinite(*e) &&
                   std::abs(*a - *e) <= std::max(kClose, kClose * std::abs(*e));
        if (const auto *i = std::get_if<std::int64_t>(&expected.value))
            return std::isfinite(*a) &&
                   std::abs(*a - static_cast<double>(*i)) <= kClose;
        return false;
    }
    if (const auto *a = std::get_if<std::int64_t>(&actual.value)) {
        if (const auto *e = std::get_if<std::int64_t>(&expected.value))
            return *a == *e;
        if (const auto *e = std::get_if<double>(&expected.value))
            return std::isfinite(*e) &&
                   std::abs(static_cast<double>(*a) - *e) <= kClose;
        return false;
    }
    if (actual.value.index() != expected.value.index()) return false;
    switch (actual.value.index()) {
    case 0: return true;
    case 1: return std::get<bool>(actual.value) == std::get<bool>(expected.value);
    case 4:
        return std::get<std::string>(actual.value) ==
               std::get<std::string>(expected.value);
    case 5: {
        const auto &a = std::get<runtime::WireArray>(actual.value);
        const auto &e = std::get<runtime::WireArray>(expected.value);
        if (a.size() != e.size()) return false;
        for (std::size_t i = 0; i < a.size(); ++i)
            if (!tree_close(a[i], e[i])) return false;
        return true;
    }
    default: {
        const auto &a = std::get<runtime::WireObject>(actual.value);
        const auto &e = std::get<runtime::WireObject>(expected.value);
        if (a.size() != e.size()) return false;
        for (const auto &[name, value] : a) {
            const runtime::Wire *other = sw::find(e, name);
            if (other == nullptr || !tree_close(value, *other)) return false;
        }
        return true;
    }
    }
}

void require_tree_close(const runtime::Wire &actual, const runtime::Wire &expected,
                        std::string_view what) {
    bt::require(tree_close(actual, expected), what);
}
void require_tree_close(const runtime::WireObject &actual,
                        const runtime::WireObject &expected, std::string_view what) {
    require_tree_close(runtime::Wire{runtime::WireObject{actual}},
                       runtime::Wire{runtime::WireObject{expected}}, what);
}

[[nodiscard]] double wire_number(const runtime::WireObject &o, std::string_view key) {
    const runtime::Wire &value = wire_member(o, key);
    if (const auto *d = std::get_if<double>(&value.value)) return *d;
    return static_cast<double>(sw::integer(value));
}
[[nodiscard]] bool wire_bool(const runtime::WireObject &o, std::string_view key) {
    return sw::boolean(wire_member(o, key));
}
[[nodiscard]] std::vector<std::string> wire_strings(const runtime::Wire &value) {
    std::vector<std::string> out;
    for (const auto &item : sw::array(value)) out.push_back(sw::string(item));
    return out;
}

runtime::RuntimeSampleBatch drain(runtime::NativeRideRuntime &rt) {
    runtime::RuntimeSampleBatch batch = rt.prepare_samples();
    rt.acknowledge_samples(batch);
    return batch;
}

// ---------------------------------------------------------------------------
// test_independent_opposing_work_is_not_cancelled — closure powers ride their
// own elastic channel; the numerical constraint bound never sees them.
// ---------------------------------------------------------------------------
void test_independent_opposing_work_is_not_cancelled() {
    const std::array<double, 2> muscle{400., -350.};
    const std::array<double, 2> constraint{20., -20.};
    const std::array<double, 2> closure{7., -3.};
    const runtime::StepWork work =
        runtime::step_work(muscle, 100., constraint, closure, .1);
    bt::require(work.muscle_signed_j == 5. && work.muscle_positive_j == 40.,
                "muscle signed/positive work");
    bt::require(work.motor_signed_j == 10. && work.motor_positive_j == 10.,
                "motor signed/positive work");
    bt::require(work.constraint_signed_j == 0. && work.constraint_absolute_j == 4.,
                "constraint signed cancels while absolute keeps the exchange");
    bt::require(work.closure_signed_j == .4 && work.closure_absolute_j == 1.,
                "closure powers keep their own elastic channel");
    bt::require(!runtime::constraint_work_ok(4., 50.),
                "4 J absolute exceeds the 1% budget on 50 J source work");
    const std::array<double, 2> braking{-40., -60.};
    const runtime::StepWork stopped =
        runtime::step_work(braking, -20., {}, {}, .5);
    bt::require(stopped.muscle_signed_j == -50. && stopped.muscle_positive_j == 0.,
                "braking muscle work is fully signed-negative");
    bt::require(stopped.motor_signed_j == -10. && stopped.motor_positive_j == 0.,
                "braking motor work is fully signed-negative");
    bt::require(stopped.closure_signed_j == 0. && stopped.closure_absolute_j == 0.,
                "empty closure vectors contribute nothing");
}

// ---------------------------------------------------------------------------
// test_zero_source_uses_a_fixed_absolute_roundoff_budget
// ---------------------------------------------------------------------------
void test_zero_source_uses_a_fixed_absolute_roundoff_budget() {
    bt::require(runtime::constraint_work_ok(1e-8, 0.),
                "the fixed 1e-8 J budget admits the boundary value");
    bt::require(!runtime::constraint_work_ok(
                    std::nextafter(1e-8, std::numeric_limits<double>::infinity()), 0.),
                "the next representable double fails the fixed budget");
    bt::require(!runtime::constraint_work_ok(800e-8, 0.),
                "the budget is fixed, not scaled by the zero source");
    bt::require(runtime::constraint_work_ok(1., 100.),
                "1% of 100 J source admits exactly 1 J");
    bt::require(!runtime::constraint_work_ok(1.00001, 100.),
                "the 1% relative budget fails past the boundary");
}

// ---------------------------------------------------------------------------
// test_work_rejects_nonfinite_or_nonpositive_inputs (parametrized x6)
// ---------------------------------------------------------------------------
void test_work_rejects_nonfinite_or_nonpositive_inputs() {
    const std::array<double, 1> nan_lane{std::numeric_limits<double>::quiet_NaN()};
    const std::array<double, 1> inf_lane{std::numeric_limits<double>::infinity()};
    expect_invalid("muscle power",
                   [&] { (void)runtime::step_work(nan_lane, 0., {}, {}, .001); });
    expect_invalid("motor power",
                   [&] { (void)runtime::step_work(
                             std::span<const double>{},
                             std::numeric_limits<double>::infinity(), {}, {}, .001); });
    expect_invalid("constraint power",
                   [&] { (void)runtime::step_work({}, 0., inf_lane, {}, .001); });
    expect_invalid("closure power",
                   [&] { (void)runtime::step_work({}, 0., {}, nan_lane, .001); });
    expect_invalid("timestep",
                   [&] { (void)runtime::step_work({}, 0., {}, {}, 0.); });
    expect_invalid("timestep",
                   [&] { (void)runtime::step_work({}, 0., {}, {}, -.001); });
}

// ---------------------------------------------------------------------------
// test_interval_markers_survive_period_clear (5 interval programs x2
// capacities): push each interval through PeriodBuffer, clearing when full —
// the binding's exact loop; every program must reject.
// ---------------------------------------------------------------------------
void test_interval_markers_survive_period_clear() {
    struct Marker { std::int64_t id; double begin, end; };
    const std::array<std::array<Marker, 2>, 5> programs{{
        {{{0, 0., .1}, {0, .1, .2}}},
        {{{0, 0., .1}, {2, .1, .2}}},
        {{{0, 0., .1}, {1, .11, .2}}},
        {{{0, 0., .1}, {1, .09, .2}}},
        {{{0, 0., .1}, {1, .1, .1}}},
    }};
    for (const auto &program : programs)
        for (const std::size_t capacity : {1U, 4U}) {
            runtime::PeriodBuffer buffer(capacity);
            expect_invalid("", [&] {
                for (const auto &marker : program) {
                    runtime::RawStep raw;
                    raw.interval_id = marker.id;
                    raw.time_s = marker.begin;
                    raw.end_time_s = marker.end;
                    buffer.push(std::move(raw));
                    if (buffer.full()) buffer.clear();
                }
            });
        }
}

// ---------------------------------------------------------------------------
// test_consecutive_intervals_can_cross_multiple_periods
// ---------------------------------------------------------------------------
void test_consecutive_intervals_can_cross_multiple_periods() {
    runtime::PeriodBuffer buffer(2);
    for (std::int64_t i = 0; i < 7; ++i) {
        runtime::RawStep raw;
        raw.interval_id = i;
        raw.time_s = static_cast<double>(i) * .1;
        raw.end_time_s = static_cast<double>(i + 1) * .1;
        buffer.push(std::move(raw));
        if (buffer.full()) buffer.clear();
    }
    bt::require(true, "sequential intervals cross multiple periods");
}

// ---- history rows (the binding's history_sample() document shape) -----------

[[nodiscard]] runtime::PhysicalSampleData
history_row(std::int64_t interval, runtime::WireArray front,
            runtime::WireArray rear) {
    runtime::PhysicalSampleData sample;
    sample.interval_id = interval;
    sample.time_s = static_cast<double>(interval) * .1;
    sample.end_time_s = static_cast<double>(interval + 1) * .1;
    sample.powers_w.emplace_back("muscle", runtime::Wire(2.));
    runtime::WireObject front_evidence{{"patches", runtime::Wire(std::move(front))}};
    runtime::WireObject rear_evidence{{"patches", runtime::Wire(std::move(rear))}};
    runtime::WireObject tires{{"front", runtime::Wire(std::move(front_evidence))},
                              {"rear", runtime::Wire(std::move(rear_evidence))}};
    sample.channels.emplace_back("tires", runtime::Wire(std::move(tires)));
    return sample;
}
[[nodiscard]] runtime::Wire patch(double normal_load_n, std::string_view source) {
    return runtime::Wire(runtime::WireObject{
        {"normal_load_n", runtime::Wire(normal_load_n)},
        {"source_geom", runtime::Wire(std::string(source))}});
}

// ---------------------------------------------------------------------------
// test_airtime_uses_working_road_contact_not_catch_plane
// ---------------------------------------------------------------------------
void test_airtime_uses_working_road_contact_not_catch_plane() {
    runtime::WorkHistory history;
    history.add(history_row(0, {patch(2., "terrain")}, {patch(100., "catch_plane")}));
    history.add(history_row(1, {}, {}));
    const runtime::WireObject wire = history.as_wire();
    bt::require(sw::integer(wire_member(wire, "last_id")) == 1,
                "history tracks the last interval id");
    bt::require(wire_number(wire, "duration_s") == .2,
                "two .1 s intervals accumulate .2 s");
    const auto &work = sw::object(wire_member(wire, "work_j"));
    bt::require(wire_number(work, "muscle") == .4,
                "2 W x .2 s of muscle work accumulates");
    const auto &airtime = sw::object(wire_member(wire, "airtime_s"));
    const auto &front = sw::object(wire_member(airtime, "front"));
    // Front's road load is 2 N: above the 0/1 N buckets, under the 5 N one.
    bt::require(wire_number(front, "0.0") == .1 && wire_number(front, "1.0") == .1 &&
                    wire_number(front, "5.0") == .2,
                "front airtime counts only the working-road load");
    const auto &rear = sw::object(wire_member(airtime, "rear"));
    // Rear's only contact is the catch plane — no working-road load.
    bt::require(wire_number(rear, "0.0") == .2 && wire_number(rear, "1.0") == .2 &&
                    wire_number(rear, "5.0") == .2,
                "catch-plane contact never counts as working load");
}

// ---------------------------------------------------------------------------
// test_malformed_contact_evidence_is_not_airtime (parametrized x4)
// ---------------------------------------------------------------------------
void test_malformed_contact_evidence_is_not_airtime() {
    runtime::WorkHistory history;
    // missing_wheel: the tires object carries no 'rear' member.
    {
        runtime::PhysicalSampleData row = history_row(0, {}, {});
        auto &channels = row.channels.front().second;
        auto &tires = std::get<runtime::WireObject>(channels.value);
        tires.pop_back(); // drop 'rear'
        expect_invalid("", [&] { history.add(row); });
    }
    // missing_patches: rear exists but carries no 'patches' member.
    {
        runtime::PhysicalSampleData row = history_row(0, {}, {});
        auto &channels = row.channels.front().second;
        auto &tires = std::get<runtime::WireObject>(channels.value);
        std::get<runtime::WireObject>(tires.back().second.value).clear();
        expect_invalid("", [&] { history.add(row); });
    }
    // negative_load / unknown_source in the rear patches.
    expect_invalid("",
                   [&] { history.add(history_row(0, {}, {patch(-1., "terrain")})); });
    expect_invalid("", [&] {
        history.add(history_row(0, {}, {patch(1., "unrecognized")}));
    });
}

// ---------------------------------------------------------------------------
// test_history_rejects_duplicate_interval_after_valid_evidence
// ---------------------------------------------------------------------------
void test_history_rejects_duplicate_interval_after_valid_evidence() {
    runtime::WorkHistory history;
    history.add(history_row(0, {}, {}));
    expect_invalid("repeated", [&] { history.add(history_row(0, {}, {})); });
}

// ---------------------------------------------------------------------------
// test_inter_tick_effort_uses_each_captured_velocity_and_strength — the
// welded plant's real SpindleController: observe_effort() on held captures
// must reproduce solved_effort()'s fold for every incoming state, and the
// observation is not a control tick (probe state never moves).
// ---------------------------------------------------------------------------
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_inter_tick_effort_uses_each_captured_velocity_and_strength() {
    rcr::PinnedPlant plant = rcr::pinned_model();
    mjModel *model = plant.model.get();
    mjData *data = plant.data.get();
    const auto nq = static_cast<std::size_t>(model->nq);
    const auto nv = static_cast<std::size_t>(model->nv);
    const auto nu = static_cast<std::size_t>(model->nu);
    const double dt = model->opt.timestep;
    // state.rider_controller is the captured controller's state_dict —
    // the document probe.spindle_restore(controller.state_dict()) feeds.
    const runtime_reference::ResolvedDocs docs = resolved_docs(welded_dir());
    const rider::SpindleController::State held = rider::decode_spindle_state(
        member(docs.state, "rider_controller"), "state.rider_controller");
    // Both controllers carry the capture's rider_controller config — the
    // runtime-side document (delivered_joint_power_budget_w resolves null
    // there while the source TOML declares 375 W, so the plant config
    // would disagree with the held state on the joint-power budget).
    const double crank_m = plant.request.bike.crank_length / 1000.;
    auto probe = std::make_unique<rider::SpindleController>(
        model, plant.pose,
        rider::decode_spindle_config(member(docs.config, "rider_controller")),
        crank_m);
    probe->restore(held);
    // The probe's held state must not move across observations — compare
    // the encoded state before the loop, not the round-tripped document.
    const std::string probe_state_before =
        bio::canonical_json(rider::encode_spindle_state(probe->state()));
    // The oracle-side controller mirrors sim.physical.rider_control.
    auto oracle = std::make_unique<rider::SpindleController>(
        model, plant.pose,
        rider::decode_spindle_config(member(docs.config, "rider_controller")),
        crank_m);
    oracle->restore(held);
    // Settled t=0 qpos — the source's np.array(sim.data.qpos) at capture.
    // mjSTATE_INTEGRATION's lane order is TIME|QPOS|QVEL|ACT|WARMSTART per
    // engine::state_size; qpos begins at offset 1.
    const auto &lanes =
        std::get<bio::Value::Array>(member(docs.state, "integration_state").data);
    const mjtSize width = engine::state_size(
        model, static_cast<int>(mjSTATE_INTEGRATION));
    bt::require(std::cmp_equal(lanes.size(), width),
                "integration_state carries the mjSTATE_INTEGRATION lanes");
    std::vector<double> qpos;
    qpos.reserve(nq);
    for (std::size_t i = 0; i < nq; ++i)
        qpos.push_back(bio::as_double(lanes[1 + i], "integration_state.qpos"));
    // The tested joint — first name containing 'hip' in controller.joints.
    const auto &joints = plant.controller->joints();
    const auto hip = std::ranges::find_if(
        joints, [](const auto &joint) { return joint.name.find("hip") != std::string::npos; });
    bt::require(hip != joints.end(), "the pinned rider has a hip joint");

    constexpr std::array<std::pair<double, double>, 4> cases{
        {{25., -1.}, {26., 10.}, {-2., 21.}, {1e6, 1.}}};
    constexpr std::array<double, 4> powers{0., 260., 0., 1e6};
    std::vector<double> measured;
    for (std::size_t index = 0; index < cases.size(); ++index) {
        const auto [torque, speed] = cases[index];
        std::vector<double> velocity(nv, 0.), forces(nu, 0.), passive(nv, 0.);
        velocity[static_cast<std::size_t>(hip->dof_adr)] = speed;
        forces[static_cast<std::size_t>(hip->actuator_id)] = torque;
        passive[static_cast<std::size_t>(hip->dof_adr)] = -.5 * speed;
        // Oracle: solved_effort(controller, SimpleNamespace(actuator_force=forces,
        // qfrc_passive=passive), (qpos, velocity), dt).
        const auto actuator_span =
            model_access::mutable_buffer(data->actuator_force, model->nu,
                                         "actuator_force");
        const auto passive_span = model_access::mutable_buffer(
            data->qfrc_passive, model->nv, "qfrc_passive");
        std::ranges::fill(actuator_span, 0.);
        std::ranges::fill(passive_span, 0.);
        actuator_span[static_cast<std::size_t>(hip->actuator_id)] = torque;
        passive_span[static_cast<std::size_t>(hip->dof_adr)] = -.5 * speed;
        const rider::EffortDiagnostics expected = oracle->solved_effort(
            data, std::span<const double>(qpos),
            std::span<const double>(velocity), dt);
        // Probe: runtime_observe_effort(qpos, velocity, forces, passive, dt)
        // on the held captures — never a control tick.
        runtime::RawStep raw;
        raw.interval_id = static_cast<std::int64_t>(index);
        raw.qpos = qpos;
        raw.qvel = velocity;
        raw.actuator_force = forces;
        raw.qfrc_passive = passive;
        raw.effort_base = {};
        raw.effort_state = held.effort_diagnostics;
        raw.rider_control_terms = held.last_terms;
        const runtime::EffortObservation observed =
            runtime::observe_effort(raw, *probe, dt);
        // The probe's held state did not move between captures.
        bt::require(bio::canonical_json(rider::encode_spindle_state(
                                          probe->state())) == probe_state_before,
                    "observing an interval is not a control tick");
        // assert_tree_close(actual['effort'], expected) — the 11-key fold.
        runtime::WireObject expected_channels;
        const auto named = [](const rider::NamedEntries<double> &entries) {
            runtime::WireObject object;
            for (const auto &[name, value] : entries)
                object.emplace_back(name, runtime::Wire(value));
            return runtime::Wire(std::move(object));
        };
        const auto names = [](const std::vector<std::string> &items) {
            runtime::WireArray out;
            for (const auto &item : items) out.emplace_back(runtime::Wire(item));
            return runtime::Wire(std::move(out));
        };
        expected_channels.emplace_back("rider_active_delivered_nm",
                                       named(expected.active_delivered_nm));
        expected_channels.emplace_back("rider_joint_positive_power_w",
                                       named(*expected.joint_positive_power_w));
        expected_channels.emplace_back("rider_joint_power_violations",
                                       names(*expected.joint_power_violations));
        expected_channels.emplace_back("rider_joint_speed_violations",
                                       names(*expected.joint_speed_violations));
        expected_channels.emplace_back("rider_positive_power_w",
                                       runtime::Wire(expected.positive_power_w));
        expected_channels.emplace_back("rider_passive_power_w",
                                       runtime::Wire(expected.passive_power_w));
        expected_channels.emplace_back(
            "rider_positive_work_step_j", runtime::Wire(*expected.positive_work_step_j));
        expected_channels.emplace_back(
            "rider_passive_work_step_j", runtime::Wire(*expected.passive_work_step_j));
        expected_channels.emplace_back("rider_strength_violations",
                                       names(*expected.strength_violations));
        expected_channels.emplace_back("rider_effort_budget_exceeded",
                                       runtime::Wire(expected.budget_exceeded));
        expected_channels.emplace_back("rider_effort_observation",
                                       runtime::Wire(expected.observation));
        require_tree_close(observed.channels, expected_channels,
                           "observed effort channels match the solved fold");
        // The violations list is the oracle's ordered fold of the same
        // diagnostics plus the positive-power budget verdict.
        std::vector<std::string> expected_errors;
        for (const auto &name : *expected.strength_violations)
            expected_errors.push_back("rider_strength." + name);
        for (const auto &name : *expected.joint_power_violations)
            expected_errors.push_back("rider_power." + name);
        for (const auto &name : *expected.joint_speed_violations)
            expected_errors.push_back("rider_speed." + name);
        if (expected.budget_exceeded) expected_errors.push_back("rider_power.positive");
        bt::require(observed.violations == expected_errors,
                    "violations fold in oracle order");
        measured.push_back(wire_number(observed.channels, "rider_positive_power_w"));
        if (torque == 1e6) {
            bt::require(std::ranges::find(observed.violations,
                                          "rider_strength." + hip->name) !=
                            observed.violations.end(),
                        "1e6 Nm delivery violates the hip strength curve");
            bt::require(std::ranges::find(observed.violations,
                                          "rider_power.positive") !=
                            observed.violations.end(),
                        "1e6 Nm delivery exceeds the positive-power budget");
        }
    }
    bt::require(measured == std::vector<double>(powers.begin(), powers.end()),
                "per-case positive powers are the captured-torque x speed fold");
}
NATIVE_DIAG_POP

// ---- strict/diagnostic welded rides -----------------------------------------

// weld_foot_min_normal_n=100: the healthy welded plant's every foot sample
// lands below the absurd floor — a deterministic early attachment violation.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[nodiscard]] runtime_reference::ResolvedDocs welded_floor_docs(bool strict,
                                                                int decimation) {
    auto docs = resolved_docs(welded_dir());
    set_member(docs.config, "strict", bio::Value{strict});
    set_member(docs.config, "measurement_decimation", num(decimation));
    set_member(member(docs.config, "rider_controller"), "weld_foot_min_normal_n",
               bio::Value{bio::Number{"100"}});
    set_member(member(member(docs.state, "runtime"), "monitor"), "strict",
               bio::Value{strict});
    set_member(member(docs.state, "runtime"), "measurement_decimation",
               num(decimation));
    return docs;
}
NATIVE_DIAG_POP

// ---------------------------------------------------------------------------
// test_strict_rejection_publishes_the_whole_period_before_raising — the
// contract half: whole completed periods publish before the raise; interval
// structure, attachment evidence, closure keys, counters and the retained
// first failure are the portable invariants.
// ---------------------------------------------------------------------------
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_strict_rejection_publishes_the_whole_period_before_raising() {
    const auto docs = welded_floor_docs(/*strict=*/true, /*decimation=*/7);
    const auto runtime = compose_runtime(docs);
    const runtime::RideControl command;
    bool raised = false;
    std::string message;
    try {
        static_cast<void>(runtime->advance(800, command, 0., 0., std::nullopt));
    } catch (const runtime::InvalidReferenceRun &failure) {
        raised = true;
        message = failure.what();
    }
    bt::require(raised, "strict mode raises InvalidReferenceRun");
    bt::require(message.find("invalid reference step") != std::string::npos,
                "the strict rejection names the invalid reference step");
    const runtime::RuntimeSampleBatch batch = drain(*runtime);
    bt::require(!batch.samples.empty(),
                "the strict rejection publishes completed periods first");
    std::vector<std::int64_t> ids;
    std::optional<std::int64_t> earliest;
    std::vector<std::string> earliest_violations;
    double earliest_end = 0.;
    for (const runtime::SamplePtr &sample : batch.samples) {
        const runtime::WireObject row = sample->as_wire();
        ids.push_back(sw::integer(wire_member(row, "interval_id")));
        const auto violations =
            wire_strings(wire_member(row, "attachment_violations"));
        if (!earliest.has_value() && !violations.empty()) {
            earliest = ids.back();
            earliest_violations = violations;
            earliest_end = wire_number(row, "interval_end_s");
        }
        bt::require(
            runtime_reference::wire_has(sw::object(wire_member(row, "energy")),
                                        "closure_signed_j") &&
                runtime_reference::wire_has(
                    sw::object(wire_member(row, "energy")), "closure_absolute_j"),
            "published rows carry the closure channel structurally");
    }
    bt::require(earliest.has_value(),
                "at least one published row carries attachment violations");
    for (std::size_t i = 0; i < ids.size(); ++i)
        bt::require(ids[i] == static_cast<std::int64_t>(i),
                    "published interval ids are dense from zero");
    const auto snap =
        std::make_unique<const runtime::RuntimeSnapshot>(runtime->snapshot());
    bt::require(snap->step == static_cast<std::int64_t>(ids.size()),
                "snapshot.step equals the published interval count");
    bt::require(snap->first_failure.has_value(),
                "the strict failure is retained as first_failure");
    bt::require(snap->first_failure->time_s == earliest_end &&
                    snap->first_failure->violations == earliest_violations,
                "first_failure is the earliest violating published row");
    const runtime::WireObject accounting = runtime->accounting_state();
    const auto &history = sw::object(wire_member(accounting, "history"));
    bt::require(sw::integer(wire_member(history, "last_id")) ==
                    static_cast<std::int64_t>(ids.size()) - 1,
                "history counted every published interval");
    bt::require(sw::integer(wire_member(accounting, "buffered_intervals")) == 0,
                "the whole closed period was published before the raise");
    bt::require(runtime->prepare_samples().samples.empty(),
                "no further rows remain after the drain");
    runtime->close();
}
NATIVE_DIAG_POP

// ---------------------------------------------------------------------------
// test_diagnostic_warns_once_and_keeps_failure_after_more_periods — the
// pending warning is the adapter's RuntimeWarning surface: it fires once,
// first_failure stays latched, and the model status keeps the failure.
// ---------------------------------------------------------------------------
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_diagnostic_warns_once_and_keeps_failure_after_more_periods() {
    const auto docs = welded_floor_docs(/*strict=*/false, /*decimation=*/1);
    const auto runtime = compose_runtime(docs);
    const runtime::RideControl command;
    static_cast<void>(runtime->advance(41, command, 0., 0., std::nullopt));
    runtime->flush();
    const std::optional<std::string> first_warning = runtime->take_pending_warning();
    bt::require(first_warning.has_value() &&
                    first_warning->find("invalid reference step") != std::string::npos,
                "the first failing interval emits the invalid-step warning once");
    const auto first = runtime->snapshot().first_failure;
    bt::require(first.has_value(), "the diagnostic latches first_failure");
    static_cast<void>(runtime->advance(81, command, 0., 0., std::nullopt));
    runtime->flush();
    bt::require(!runtime->take_pending_warning().has_value(),
                "later failing periods do not re-warn");
    const auto after = runtime->snapshot();
    bt::require(after.first_failure.has_value() &&
                    after.first_failure->time_s == first->time_s &&
                    after.first_failure->violations == first->violations,
                "first_failure stays latched after more periods");
    const auto &status = after.model_status;
    bt::require(!wire_bool(status, "model_valid"),
                "the retained model status keeps the failure");
    const runtime::RuntimeSampleBatch batch = drain(*runtime);
    bt::require(batch.samples.size() == 81,
                "non-strict diagnostics publish all 81 intervals");
    for (std::size_t i = 0; i < batch.samples.size(); ++i)
        bt::require(batch.samples[i]->interval_id == static_cast<std::int64_t>(i),
                    "drained interval ids are dense from zero");
    runtime->close();
}
NATIVE_DIAG_POP

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(test_independent_opposing_work_is_not_cancelled);
        failures += bt::run(test_zero_source_uses_a_fixed_absolute_roundoff_budget);
        failures += bt::run(test_work_rejects_nonfinite_or_nonpositive_inputs);
        failures += bt::run(test_interval_markers_survive_period_clear);
        failures += bt::run(test_consecutive_intervals_can_cross_multiple_periods);
        failures += bt::run(test_airtime_uses_working_road_contact_not_catch_plane);
        failures += bt::run(test_malformed_contact_evidence_is_not_airtime);
        failures += bt::run(test_history_rejects_duplicate_interval_after_valid_evidence);
        failures += bt::run(
            test_inter_tick_effort_uses_each_captured_velocity_and_strength);
        failures += bt::run(
            test_strict_rejection_publishes_the_whole_period_before_raising);
        failures += bt::run(
            test_diagnostic_warns_once_and_keeps_failure_after_more_periods);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
