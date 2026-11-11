// test_static_brake_latch.cpp — launch part 1 task 6: the static-brake
// latch as serialized step state. Review P9 governs the shape: two
// MUTABLE component instances sharing the live brake's model indices and
// spec; a nonzero anchor established by rolling the wheel before the
// hold; a displaced wheel producing a demonstrably nonzero held force;
// the state crossing the production latch codec (JSON text, not a typed
// copy); then both instances continuing through advancing hold, release
// and re-latch inputs compared bitwise on full force vectors AND both
// latch optionals. Non-advancing calls must not move either latch, and a
// rejected second field must leave the whole committed state untouched.
#include "support/fixture_test.hpp"
#include "launch_ride_support.hpp"
#include "io/json.hpp"
#include "model_access.hpp"
#include "runtime/brake_latch_codec.hpp"
#include "runtime/runtime.hpp"
#include "runtime/static_brake.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {
namespace terrain = bike::terrain;
namespace bio = bike::io;

void ride_intervals(launch_ride::PedelecEnv &env, std::int64_t count,
                    const runtime::RideControl &control) {
    const auto committed = env.runtime->snapshot().step;
    const auto result = env.runtime->advance(committed + count, control,
                                             0., 0., std::nullopt);
    bt::require(!result.outcome, "ride did not crash: " +
                                 (result.outcome ? *result.outcome : ""));
}

// The wheel-angle slot hold_qfrc reads for a brake dof
// (qpos[jnt_qposadr[dof_jntid[dof]]]) — same expression, read side.
int wheel_qpos_slot(const mjModel *m, int dof) {
    const auto dof_jntid = model_access::readonly_buffer(
        m->dof_jntid, m->nv, "dof_jntid");
    const auto jnt_qposadr = model_access::readonly_buffer(
        m->jnt_qposadr, m->njnt, "jnt_qposadr");
    const int joint = dof_jntid[static_cast<std::size_t>(dof)];
    return jnt_qposadr[static_cast<std::size_t>(joint)];
}

// One scripted input through both continued instances: the wheel angle
// and speed are staged on the live mjData and the full nv force vector
// plus both latch optionals must come back bitwise identical.
struct StagedWheel {
    double theta;   // angle written into the wheel's qpos slot
    double omega;   // speed written into the wheel's qvel dof
    double front;   // front lever demand
    double rear;    // rear lever demand
    bool advance;   // an advancing evaluation may latch or release
    bool reseat;    // re-anchor a saturated bite at the staged angle
};

void step_both(runtime::StaticBrake &a, runtime::StaticBrake &b,
               const mjModel *m, const mjData *d, int dof,
               const StagedWheel &in) {
    const int slot = wheel_qpos_slot(m, dof);
    const auto qpos =
        model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel =
        model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    qpos[static_cast<std::size_t>(slot)] = in.theta;
    qvel[static_cast<std::size_t>(dof)] = in.omega;
    const std::vector<double> fa =
        a.hold_qfrc(m, d, in.front, in.rear, in.advance, in.reseat);
    const std::vector<double> fb =
        b.hold_qfrc(m, d, in.front, in.rear, in.advance, in.reseat);
    bt::require(fa == fb,
                "restored latch produces the identical held force");
    const auto sa = a.state(), sb = b.state();
    bt::require(sa.latch_front == sb.latch_front &&
                    sa.latch_rear == sb.latch_rear,
                "continued instances keep identical latch state");
    // A non-advancing pass is a pure read — committed state must not move.
    const auto before = a.state();
    (void)a.hold_qfrc(m, d, in.front, in.rear, false);
    const auto after = a.state();
    bt::require(before.latch_front == after.latch_front &&
                    before.latch_rear == after.latch_rear,
                "non-advancing hold_qfrc leaves the latch untouched");
}

void test_latch_state_round_trips_and_continues_bitwise() {
    // Flat ground + pedaling: the wheels genuinely roll before the hold so
    // the captured anchor is a real angle, not the trivial zero.
    const auto track = terrain::load_track(
        launch_ride::repo_file("tests/fixtures/tracks/flat.toml"));
    auto env = launch_ride::environment(track, 6.);
    runtime::RideControl control;
    control.human_torque_nm = 30.;
    ride_intervals(env, 240, control); // 0.3 s of unbraked rolling
    bike::validation::RuntimeRigAccess::set_rider_commands(
        *env.runtime, rider::RiderCommands{.front_brake = 1., .rear_brake = 1.,
            .shift_request = std::nullopt});
    ride_intervals(env, 640, control); // braked to a stop: the bite captures

    double held_force_at_dof = 0.;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &s, const runtime::PhysicalStep &p) {
            const runtime::StaticBrake &held =
                bike::validation::RuntimeRigAccess::static_brake(p);
            const runtime::StaticBrake::State captured = held.state();
            bt::require(captured.latch_rear.has_value(),
                        "the rear bite is latched while holding");
            bt::require(*captured.latch_rear != 0.,
                        "the anchor is a rolled angle, not the trivial zero");
            // The step state carries the same committed latch — this is the
            // member decode_step_state fills on the real transport.
            const auto step_state = p.state();
            bt::require(
                step_state.brake_latch.latch_rear == captured.latch_rear &&
                    step_state.brake_latch.latch_front == captured.latch_front,
                "StepState.brake_latch carries the committed latches");

            const mjModel *m = s.model();
            const mjData *d = s.data();
            const int rear_slot = wheel_qpos_slot(m, held.rear_dof());
            const auto qpos =
                model_access::mutable_buffer(d->qpos, m->nq, "qpos");
            const auto qvel =
                model_access::mutable_buffer(d->qvel, m->nv, "qvel");
            const double saved_theta =
                qpos[static_cast<std::size_t>(rear_slot)];
            const double saved_omega =
                qvel[static_cast<std::size_t>(held.rear_dof())];

            // a restores the typed state directly; b restores through the
            // production codec over serialized JSON text — the same
            // functions capture.cpp emits and decode_step_state reads.
            runtime::StaticBrake a(held), b(held);
            a.reset();
            b.reset();
            a.restore(captured);
            const std::string wire =
                bio::canonical_json(runtime::encode_brake_latch(captured));
            b.restore(runtime::decode_brake_latch(
                bio::parse_json(wire), "state.runtime.brake_latch"));
            bt::require(
                a.state().latch_front == b.state().latch_front &&
                    a.state().latch_rear == b.state().latch_rear,
                "codec round-trip preserves both latches bitwise");

            const double anchor = *captured.latch_rear;
            // Displaced wheel, held lever: the spring term must answer with
            // a nonzero force at the wheel dof.
            qpos[static_cast<std::size_t>(rear_slot)] = anchor + .05;
            qvel[static_cast<std::size_t>(held.rear_dof())] = 0.;
            const auto displaced = a.hold_qfrc(m, d, 1., 1., false);
            held_force_at_dof =
                displaced[static_cast<std::size_t>(held.rear_dof())];
            bt::require(held_force_at_dof != 0.,
                        "a displaced wheel gives a nonzero held force");
            const auto displaced_b = b.hold_qfrc(m, d, 1., 1., false);
            bt::require(displaced == displaced_b,
                        "restored instance answers identically when displaced");

            // Advancing continuation — same staged inputs on both:
            // held spring -> speed release -> re-latch at the new angle ->
            // saturated reseat -> lever-freed release.
            const double release = held.hold_spec().release_radps;
            step_both(a, b, m, d, held.rear_dof(),
                      {.theta = anchor + .02, .omega = 0., .front = 1.,
                       .rear = 1., .advance = true, .reseat = false});
            // Rolls past release: both drop the bite.
            step_both(a, b, m, d, held.rear_dof(),
                      {.theta = anchor + .02, .omega = release + 1.,
                       .front = 1., .rear = 1., .advance = true,
                       .reseat = false});
            // Re-latch at +0.30.
            step_both(a, b, m, d, held.rear_dof(),
                      {.theta = anchor + .30, .omega = 0., .front = 1.,
                       .rear = 1., .advance = true, .reseat = false});
            // Slipped: a saturated bite reseats at +0.80.
            step_both(a, b, m, d, held.rear_dof(),
                      {.theta = anchor + .80, .omega = 0., .front = 1.,
                       .rear = 1., .advance = true, .reseat = true});
            // Lever freed: release.
            step_both(a, b, m, d, held.rear_dof(),
                      {.theta = anchor + .80, .omega = 0., .front = 0.,
                       .rear = 0., .advance = true, .reseat = false});

            qpos[static_cast<std::size_t>(rear_slot)] = saved_theta;
            qvel[static_cast<std::size_t>(held.rear_dof())] = saved_omega;
        });
    bt::require(held_force_at_dof != 0.,
                "the displaced-wheel force was exercised");

    // Fresh episode, fresh bite — reset() replays the sealed bootstrap and
    // PhysicalStep::restore still clears the latch as ruled.
    env.runtime->reset();
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &, const runtime::PhysicalStep &p) {
            const auto st =
                bike::validation::RuntimeRigAccess::static_brake(p).state();
            bt::require(!st.latch_front.has_value() &&
                            !st.latch_rear.has_value(),
                        "a new episode resets the latch");
        });
}

// restore() validates BEFORE committing: a non-finite latch, or a valid
// front followed by an invalid rear, must both throw and leave the whole
// committed pair untouched — the two-field atomicity P9 calls for. The
// codec rejects its own malformed shapes at the wire boundary.
void test_latch_restore_rejects_nonfinite_atomically() {
    runtime::StaticBrake brake(0, 1, 200.,
                             runtime::BrakeHoldSpec{.torque_nm = 150.});
    brake.restore({.latch_front = .25, .latch_rear = -.5});
    const auto before = brake.state();
    bt::invalid([&] {
        brake.restore({.latch_front = .5,
                       .latch_rear = std::numeric_limits<double>::quiet_NaN()});
    });
    const auto after = brake.state();
    bt::require(after.latch_front == before.latch_front &&
                    after.latch_rear == before.latch_rear,
                "a bad rear latch after a valid front commits nothing");
    bt::invalid([&] {
        brake.restore({.latch_front =
                           std::numeric_limits<double>::infinity(),
                       .latch_rear = std::nullopt});
    });
    bt::invalid([&] {
        (void)runtime::decode_brake_latch(
            bio::parse_json(R"({"front":"x","rear":null})"),
            "state.runtime.brake_latch");
    });
    bt::invalid([&] {
        (void)runtime::decode_brake_latch(
            bio::parse_json(R"({"front":1.0})"),
            "state.runtime.brake_latch");
    });
    bt::invalid([&] {
        (void)runtime::decode_brake_latch(
            bio::parse_json(R"({"front":null,"rear":null,"extra":1})"),
            "state.runtime.brake_latch");
    });
    bt::require(brake.state().latch_front == .25 &&
                    brake.state().latch_rear == -.5,
                "the committed pair survives every rejection");
}
} // namespace

int main() {
    return bt::run([] {
        test_latch_state_round_trips_and_continues_bitwise();
        test_latch_restore_rejects_nonfinite_atomically();
    });
}
