// runtime.cpp — NativeRideRuntime: the owned pure-C++ ride runtime.
//
// Construction order (every failure before Stepper construction leaves
// nothing behind; a later failure unwinds the unique_ptr):
//   1. config envelope validation (runtime_schema=1, all required fields)
//      — ran inside compose_bootstrap/read_bootstrap/the adapter before
//      this ctor sees the RuntimeBootstrap
//   2. artifact identity — sha256(model image bytes) == state.model_digest
//   3. Stepper construction over the owned model image (the schema-2
//      writer_config decode happens immediately before it, same position
//      as the nanobind adapter's handle decode)
//   4. the writer-bound bootstrap halves — rider_contacts topology decode
//   5. physical-step assembly — pose/controller/intent decode
//   6. the state.runtime step-state decode
//   7. one ordered restore: reset_data -> model coefficients -> set_const
//      -> integration vector -> writer snapshots -> runtime counters
//      (mj_setConst rewrites qpos, so the integration vector lands after it)
//
// The integration vector is restored verbatim — mj_getState/mj_setState
// round-trip qacc_warmstart exactly; one trailing forward() then
// re-establishes the derived stages the oracle's reset tail leaves
// fresh (the running loop's entry forward is conditional on
// `initializing` and never runs post-bootstrap).
#include "runtime.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <mujoco/mujoco.h>

#include "../diag.hpp"
#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../rider/state_codec.hpp"
#include "../stepper.hpp"
#include "../tyre/profile.hpp"
#include "../writer_config.hpp"
#include "../writers/drivetrain.hpp"
#include "value_readers.hpp"

namespace runtime {
namespace {

namespace bio = bike::io;

// environment.py crash_reason — the latched CrashEvent's outcome label.
[[nodiscard]] std::optional<std::string>
crash_outcome(const std::optional<runtime::CrashEvent> &event) {
    if (!event.has_value()) return std::nullopt;
    if (event->cause == "pitch_over")
        return std::string(event->pitch_rad < 0. ? "crash:loop_out"
                                                 : "crash:endo");
    return "crash:" + event->cause;
}

// Reentrancy guard — advance() and reset() are the two entries that can
// run without caller serialization; a nested or concurrent call fails fast
// instead of observing a half-advanced runtime.
class AdvancementGuard {
public:
    explicit AdvancementGuard(std::atomic<bool> &flag) : flag_(flag) {
        bool expected = false;
        if (!flag_.compare_exchange_strong(expected, true))
            throw std::runtime_error(
                "advance already in progress on this runtime");
    }
    ~AdvancementGuard() { flag_.store(false); }
    AdvancementGuard(const AdvancementGuard &) = delete;
    AdvancementGuard &operator=(const AdvancementGuard &) = delete;
    AdvancementGuard(AdvancementGuard &&) = delete;
    AdvancementGuard &operator=(AdvancementGuard &&) = delete;

private:
    std::atomic<bool> &flag_;
};

// ---- state.runtime section decoders --------------------------------------
// Every state_dict() shape validated key-for-key against setup.py's
// emission; rejected input leaves the live step untouched (the restore
// happens only after the whole StepState decoded).

[[nodiscard]] double real_field(const vread::Dict &d, const char *key) {
    return vread::real(vread::field(d, key), d.child(key));
}

[[nodiscard]] std::optional<double>
optional_field(const vread::Dict &d, const char *key) {
    return vread::optional_real(vread::field(d, key), d.child(key));
}

[[nodiscard]] bool bool_field(const vread::Dict &d, const char *key) {
    return vread::boolean(vread::field(d, key), d.child(key));
}

[[nodiscard]] std::int64_t int_field(const vread::Dict &d, const char *key) {
    return vread::integer(vread::field(d, key), d.child(key));
}

[[nodiscard]] runtime::GroundedFilter::State
parse_grounded(const vread::Dict &d) {
    vread::exact(d, vread::keys("last_time_s", "last_loaded_s", "value"));
    return {.last_time_s = optional_field(d, "last_time_s"),
            .last_loaded_s = optional_field(d, "last_loaded_s"),
            .value = bool_field(d, "value")};
}

[[nodiscard]] runtime::BridgedLoad::State
parse_bridged(const vread::Dict &d) {
    vread::exact(d, vread::keys("held_load_n", "unloaded_steps"));
    return {.held_load_n = real_field(d, "held_load_n"),
            .unloaded_steps = int_field(d, "unloaded_steps")};
}

[[nodiscard]] runtime::RuntimeContactQuery::State
parse_query_state(const vread::Dict &d) {
    vread::exact(d,
                 vread::keys("front_load", "rear_load",
                             "front_controller_grounded",
                             "rear_controller_grounded"));
    return {.front_load = parse_bridged(vread::section(d, "front_load")),
            .rear_load = parse_bridged(vread::section(d, "rear_load")),
            .front_controller = parse_grounded(
                vread::section(d, "front_controller_grounded")),
            .rear_controller = parse_grounded(
                vread::section(d, "rear_controller_grounded"))};
}

[[nodiscard]] runtime::BalanceMonitor::State
parse_balance(const vread::Dict &d) {
    vread::exact(d, vread::keys("event", "current_event", "low_speed_s",
                                "grace_left_s",
                                "started_s", "low_started_s", "last_time_s"));
    runtime::BalanceMonitor::State out;
    const auto parse_event = [&d](const char *key)
        -> std::optional<runtime::BalanceLostEvent> {
        const vread::Node event = vread::field(d, key);
        if (vread::is_none(event)) return std::nullopt;
        const vread::Dict e{
            .value = &vread::mapping(event, d.child(key)),
            .path = d.child(key)};
        vread::exact(e, vread::keys("time_s", "position_m", "speed_mps"));
        return runtime::BalanceLostEvent{
            .time_s = real_field(e, "time_s"),
            .position_m = real_field(e, "position_m"),
            .speed_mps = real_field(e, "speed_mps")};
    };
    out.event = parse_event("event");
    out.current_event = parse_event("current_event");
    out.low_speed_s = real_field(d, "low_speed_s");
    out.grace_left_s = real_field(d, "grace_left_s");
    out.started_s = optional_field(d, "started_s");
    out.low_started_s = optional_field(d, "low_started_s");
    out.last_time_s = optional_field(d, "last_time_s");
    return out;
}

[[nodiscard]] std::optional<runtime::CrashEvent>
parse_crash(const vread::Dict &d) {
    vread::exact(d, vread::keys("event"));
    const vread::Node event = vread::field(d, "event");
    if (vread::is_none(event)) return std::nullopt;
    const vread::Dict e{.value = &vread::mapping(event, d.child("event")),
                        .path = d.child("event")};
    vread::exact(e,
                 vread::keys("cause", "time_s", "position_m", "pitch_rad"));
    return runtime::CrashEvent{
        .cause = vread::string(vread::field(e, "cause"), e.child("cause")),
        .time_s = real_field(e, "time_s"),
        .position_m = real_field(e, "position_m"),
        .pitch_rad = real_field(e, "pitch_rad")};
}

[[nodiscard]] runtime::RuntimeContacts
parse_contacts(const vread::Dict &d) {
    vread::exact(d,
                 vread::keys("front_load_n", "rear_load_n", "front_support_n",
                             "rear_support_n", "handlebar_load_n",
                             "front_controller_grounded",
                             "rear_controller_grounded", "front_slip_mps",
                             "rear_slip_mps"));
    runtime::RuntimeContacts out;
    out.front_load_n = real_field(d, "front_load_n");
    out.rear_load_n = real_field(d, "rear_load_n");
    out.front_support_n = real_field(d, "front_support_n");
    out.rear_support_n = real_field(d, "rear_support_n");
    out.handlebar_load_n = real_field(d, "handlebar_load_n");
    // _contacts_state emits plain bools for the controller-grounded pair.
    out.front_controller_grounded = bool_field(d, "front_controller_grounded");
    out.rear_controller_grounded = bool_field(d, "rear_controller_grounded");
    out.front_slip_mps = optional_field(d, "front_slip_mps");
    out.rear_slip_mps = optional_field(d, "rear_slip_mps");
    return out;
}

[[nodiscard]] runtime::EnergyState
parse_energy(const vread::Dict &d) {
    vread::exact(d, vread::keys(
                        "initial_energy_j", "energy_scale_j", "loss_j",
                        "active_work_j", "external_work_j", "solver_work_j",
                        "muscle_signed_j", "muscle_positive_j",
                        "motor_signed_j", "motor_positive_j",
                        "constraint_absolute_j", "initial_battery_j",
                        "electrical_work_j", "mechanical_energy_j",
                        "elastic_energy_j", "residual_j"));
    runtime::EnergyState out;
    out.initial_energy_j = optional_field(d, "initial_energy_j");
    out.energy_scale_j = real_field(d, "energy_scale_j");
    out.loss_j = real_field(d, "loss_j");
    out.active_work_j = real_field(d, "active_work_j");
    out.external_work_j = real_field(d, "external_work_j");
    out.solver_work_j = real_field(d, "solver_work_j");
    out.muscle_signed_j = real_field(d, "muscle_signed_j");
    out.muscle_positive_j = real_field(d, "muscle_positive_j");
    out.motor_signed_j = real_field(d, "motor_signed_j");
    out.motor_positive_j = real_field(d, "motor_positive_j");
    out.constraint_absolute_j = real_field(d, "constraint_absolute_j");
    out.initial_battery_j = optional_field(d, "initial_battery_j");
    out.electrical_work_j = real_field(d, "electrical_work_j");
    out.mechanical_energy_j = real_field(d, "mechanical_energy_j");
    const runtime::Wire elastic =
        vread::wire_from_value(*vread::field(d, "elastic_energy_j").value,
                               d.child("elastic_energy_j"));
    if (const auto *object = std::get_if<runtime::WireObject>(
            &elastic.value))
        out.elastic_energy_j = *object;
    else
        vread::invalid(d.child("elastic_energy_j"), "expected dict");
    out.residual_j = real_field(d, "residual_j");
    return out;
}

// state.runtime + the sibling top-level sections (signals, rider_intent,
// rider_controller) -> the step's full restore input. history/
// model_status/monitor are decoded separately by initialize_accounting().
[[nodiscard]] runtime::StepState
decode_step_state(const runtime::BootstrapState &bootstrap) {
    const vread::Dict rt{.value = &vread::mapping(
                             vread::Node{&bootstrap.runtime},
                             "state.runtime"),
                         .path = "state.runtime"};
    vread::exact(rt, vread::keys(
                         "step", "generation", "measurement_decimation",
                         "applied_control", "held_control",
                         "held_rider_terms", "rollback_hold",
                         "research_accounting_valid", "initializing",
                         "filters", "balance", "crash", "contacts",
                         "contact_query", "probe_query", "energy", "history",
                         "model_status", "monitor"));
    runtime::StepState out;
    out.step = int_field(rt, "step");
    if (out.step < 0)
        vread::invalid(rt.child("step"), "expected nonnegative integer");
    out.generation =
        vread::integer32(vread::field(rt, "generation"),
                         rt.child("generation"));
    if (out.generation <= 0)
        vread::invalid(rt.child("generation"), "expected positive integer");
    out.measurement_decimation =
        vread::integer32(vread::field(rt, "measurement_decimation"),
                         rt.child("measurement_decimation"));
    if (out.measurement_decimation <= 0)
        vread::invalid(rt.child("measurement_decimation"),
                       "expected positive integer");
    out.applied_control = rider::decode_intent_control(
        *vread::field(rt, "applied_control").value,
        rt.child("applied_control"));
    const vread::Node held = vread::field(rt, "held_control");
    if (!vread::is_none(held))
        out.held_control = rider::decode_named_reals(
            *held.value, rt.child("held_control"));
    const vread::Node terms = vread::field(rt, "held_rider_terms");
    if (!vread::is_none(terms)) {
        const vread::Dict map{
            .value = &vread::mapping(terms, rt.child("held_rider_terms")),
            .path = rt.child("held_rider_terms")};
        rider::NamedEntries<rider::JointTerms> decoded;
        for (const auto &[joint, item] : *map.value) {
            decoded.emplace_back(
                joint,
                rider::decode_joint_terms(item, map.child(joint)));
        }
        out.held_rider_terms = std::move(decoded);
    }
    out.rollback_hold = bool_field(rt, "rollback_hold");
    out.research_accounting_valid =
        bool_field(rt, "research_accounting_valid");
    out.initializing = bool_field(rt, "initializing");
    const vread::Dict filters = vread::section(rt, "filters");
    vread::exact(filters, vread::keys("front", "rear"));
    out.filters = {parse_grounded(vread::section(filters, "front")),
                   parse_grounded(vread::section(filters, "rear"))};
    out.balance = parse_balance(vread::section(rt, "balance"));
    out.crash = parse_crash(vread::section(rt, "crash"));
    out.contacts = parse_contacts(vread::section(rt, "contacts"));
    out.contact_query =
        parse_query_state(vread::section(rt, "contact_query"));
    out.probe_query = parse_query_state(vread::section(rt, "probe_query"));
    out.energy = parse_energy(vread::section(rt, "energy"));
    out.signals =
        rider::decode_intent_signals(bootstrap.signals, "state.signals");
    out.rider_intent =
        rider::decode_intent_state(bootstrap.rider_intent,
                                   "state.rider_intent");
    if (!bio::is_null(bootstrap.rider_controller))
        out.rider_controller = rider::decode_spindle_state(
            bootstrap.rider_controller, "state.rider_controller");
    return out;
}

// ---- PhysicalStep / accounting assembly -----------------------------------

// The step-owned slice of the runtime envelope — physics/drive modes are
// already pinned at decode_runtime_config.
[[nodiscard]] StepConfig step_config(const RuntimeConfig &config) {
    StepConfig out;
    out.timestep_s = config.timestep_s;
    out.control_period_s = config.control_period_s;
    out.measurement_decimation = config.measurement_decimation;
    out.strict = config.strict;
    out.crank_length_m = config.geometry.crank_length_m;
    const vread::Dict rider_cfg{
        .value = &vread::mapping(vread::Node{&config.rider_controller},
                                 "config.rider_controller"),
        .path = "config.rider_controller"};
    out.road_lookahead_m = vread::finite_real(
        vread::field(rider_cfg, "road_lookahead_m"),
        rider_cfg.child("road_lookahead_m"));
    out.rider_present = !bio::is_null(config.geometry.pose);
    out.monitors = config.monitors;
    out.intent = rider::decode_intent_config(config.rider_intent,
                                             "config.rider_intent");
    return out;
}

// StaticBrakeApplier — the geometry section's name-indexed brake refs plus
// the drive config's ceiling, resolved to DOF addresses once.
[[nodiscard]] StaticBrake static_brake(const RuntimeConfig &config,
                                       const mjModel &model,
                                       const Stepper &stepper) {
    const auto resolve = [&](const JointReference &ref, const char *what) {
        const int joint =
            mj_name2id(&model, mjOBJ_JOINT, ref.joint.c_str());
        if (joint < 0)
            throw std::invalid_argument(std::string("unknown ") + what +
                                        " brake joint '" + ref.joint + "'");
        const std::span<const int> joint_types =
            std::views::counted(model.jnt_type, model.njnt);
        const int dof_count = [&] {
            switch (joint_types[static_cast<std::size_t>(joint)]) {
            case mjJNT_FREE: return 6;
            case mjJNT_BALL: return 3;
            default: return 1;
            }
        }();
        if (ref.dof_index < 0 || ref.dof_index >= dof_count)
            throw std::invalid_argument(
                std::string("config.geometry.brake_dofs: ") + what +
                " dof_index out of range");
        const std::span<const int> dofadr =
            std::views::counted(model.jnt_dofadr, model.njnt);
        return dofadr[static_cast<std::size_t>(joint)] + ref.dof_index;
    };
    return {resolve(config.geometry.front_brake, "front"),
            resolve(config.geometry.rear_brake, "rear"),
            stepper.drive().config().brake_ceiling_nm};
}

// The PhysicalStep assembly — pose/config/vertices decode plus the
// constructor — carries a frame too large for the debug stack budget, so
// it owns a separate function scope.
std::unique_ptr<PhysicalStep>
build_physical_step(Stepper &stepper, const RuntimeConfig &config) {
    // The compiled terrain cross-section is constant model geometry — the
    // tire writer's own copy was read from this same post-forward data.
    std::vector<std::array<double, 2>> vertices =
        biketyre::compiled_profile_vertices(stepper.model(),
                                            stepper.data(), "terrain");
    std::optional<rider::SpindlePose> pose;
    std::optional<rider::SpindleConfig> rider_cfg;
    if (!bio::is_null(config.geometry.pose)) {
        pose = rider::decode_spindle_pose(config.geometry.pose,
                                          "config.geometry.pose");
        rider_cfg = rider::decode_spindle_config(
            config.rider_controller);
    }
    return std::make_unique<PhysicalStep>(
        stepper, step_config(config), std::move(vertices),
        static_brake(config, *stepper.model(), stepper), pose, rider_cfg);
}

} // namespace

std::unique_ptr<PhysicalStep>
make_physical_step(Stepper& stepper, const RuntimeConfig& config) {
    return build_physical_step(stepper, config);
}

// One-shot bootstrap decode — like bind_drivetrain, its frame is decode
// temporaries rather than per-step code, so the frame guard is waived.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
NativeRideRuntime::NativeRideRuntime(RuntimeBootstrap bootstrap)
    : config_(std::move(bootstrap.config)),
      bootstrap_(std::move(bootstrap.state)),
      model_image_(std::move(bootstrap.model)) {
    init();
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void NativeRideRuntime::init() {
    // Ownership is already committed; the digest re-verification keeps a
    // fabricated RuntimeBootstrap honest: the image bytes — not the
    // recorded sha field — prove the model identity.
    if (bio::sha256(model_image_.mjb) != bootstrap_.model_digest)
        throw std::invalid_argument(
            "state.model_digest: model artifact bytes do not match");
    // The writer-config decode position: exactly where the nanobind
    // adapter decoded the handle — before the model load.
    const nativecfg::WriterConfig writer_config =
        nativecfg::decode(config_.writer_config);
    stepper_ = std::make_unique<Stepper>(model_image_.mjb, writer_config);
    // Same guarantee the decode gave at compose time — repeated against
    // the live model so a fabricated bootstrap record cannot pass.
    verify_model_dims(bootstrap_.model_dims, *stepper_->model());
    // The ctor-side halves of the staged bootstrap decode — the positions
    // the nanobind build ran them inside init():
    //   rider_contacts decode (needs the live writer topology), then the
    //   physical-step assembly, then the state.runtime step-state decode.
    if (!bio::is_null(bootstrap_.rider_contacts)) {
        try {
            bootstrap_rider_contacts_ =
                rider::decode_rider_contacts_state(
                    bootstrap_.rider_contacts, stepper_->rider_contacts());
        } catch (const std::logic_error &) {
            vread::invalid("state.rider_contacts",
                           "the runtime config has no rider_contacts writer");
        }
    }
    physical_ = make_physical_step(*stepper_, config_);
    bootstrap_step_state_ = decode_step_state(bootstrap_);
    // The captured controller state and the configured rider ensemble
    // arrive together (setup.py emits both or neither).
    if (bootstrap_step_state_.rider_controller.has_value() !=
        physical_->rider_present())
        throw std::invalid_argument(
            "state.rider_controller does not match the rider config");
    apply_bootstrap();
    initialize_accounting();
}
NATIVE_DIAG_POP

void NativeRideRuntime::require_open() const {
    if (closed_) throw std::logic_error("NativeRideRuntime is closed");
}

void NativeRideRuntime::apply_bootstrap() {
    mjModel *const model = stepper_->model();
    mjData *const data = stepper_->data();
    stepper_->mutate([&] {
        engine::reset_data(model, data);
        // reset + model writes (frictionloss, site_pos, mj_setConst,
        // mj_setState, transmission restores): every derived stage is
        // stale until a full forward re-establishes pos/vel caches.
        stepper_->note_posvel_stale();
        const std::span<double> frictionloss =
            std::views::counted(model->dof_frictionloss, model->nv);
        for (const FrictionLossEntry &e :
             bootstrap_.model_mutable.dof_frictionloss)
            frictionloss[static_cast<std::size_t>(e.dof_adr)] = e.value;
        const std::span<double> site_pos =
            std::views::counted(model->site_pos, model->nsite * 3);
        for (const SitePosEntry &e : bootstrap_.model_mutable.site_pos)
            for (std::size_t i = 0; i < 3; ++i)
                site_pos[static_cast<std::size_t>(e.site_id) * 3U + i] =
                    e.value.at(i);
        // mj_setConst derives rest-state constants from qpos0 — it rewrites
        // qpos — so the captured integration vector restores strictly after
        // it, and the writer snapshots (which never touch live mjData) can
        // sit on either side.
        engine::set_const(model, data);
        engine::set_state(model, data, bootstrap_.integration_state.data(),
                          static_cast<int>(mjSTATE_INTEGRATION));
        stepper_->drive().restore(bootstrap_.drive);
        if (bootstrap_rider_contacts_.has_value()) {
            // assign_state moves — hand it a copy so the stored bootstrap
            // stays replayable across reset().
            auto [contact_state, probe] = *bootstrap_rider_contacts_;
            stepper_->rider_contacts().assign_state(std::move(contact_state),
                                                    std::move(probe));
        }
        // A tire writer validates its full canonical column set, so restore
        // whenever one exists; a document carrying tire state against a
        // no-tire Stepper still reports the absent-writer error.
        if (stepper_->has_tire() || !bootstrap_.tire.names.empty())
            stepper_->set_tire_state(bootstrap_.tire.names,
                                     bootstrap_.tire.row);
        // The runtime counters land last (the docstring's ordered restore
        // ends with them): step/generation, clocks, filters, contacts,
        // energy and the controller/intent state faces.
        physical_->restore(bootstrap_step_state_);
        // Oracle: PhysicalRuntime.reset() leaves sim.data fully
        // forwarded (physical_runtime.py:444), so the running loop's
        // apply_forces entry forward is conditional on `initializing`
        // and never runs after bootstrap. mj_setState round-trips only
        // the integration vector; re-establish the derived stages once
        // here so the first interval sees the same fresh caches.
        engine::forward(model, data);
        stepper_->note_posvel_fresh();
    });
}

// Decode the A4-owned portion once; reset replays typed owned values.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void NativeRideRuntime::initialize_accounting() {
    const Wire runtime_tree =
        vread::wire_from_value(bootstrap_.runtime, "state.runtime");
    const auto &runtime = sample_wire::object(runtime_tree);
    bootstrap_accounting_state_.energy = bootstrap_step_state_.energy;
    // _last_endpoint_mass and _last_measurement_source need no explicit
    // init here: the draining buffer always starts at interval 0, and
    // 0 % measurement_decimation == 0 makes that raw measured, so the
    // close scan populates both fields before any unmeasured row can
    // read them. Nullopt is also the oracle's value after restore —
    // Python does not persist them either.
    bootstrap_accounting_state_.last_endpoint_mass = std::nullopt;
    bootstrap_accounting_state_.last_measurement_source = std::nullopt;
    bootstrap_accounting_state_.history = WorkHistory::from_wire(
        sample_wire::object(sample_wire::required(runtime, "history")));
    bootstrap_accounting_state_.model_status = ModelStatus::from_wire(
        sample_wire::object(sample_wire::required(runtime, "model_status")));
    bootstrap_accounting_state_.monitor = ReferenceMonitor::from_wire(
        sample_wire::object(sample_wire::required(runtime, "monitor")));
    if (bootstrap_step_state_.step != 0 || stepper_->data()->time != 0.)
        throw std::invalid_argument("state.runtime.step: bootstrap must describe t=0");
    if (bootstrap_accounting_state_.history.last_id.has_value() ||
        bootstrap_accounting_state_.history.last_end.has_value() ||
        bootstrap_accounting_state_.model_status.last_interval != -1)
        throw std::invalid_argument("state.runtime: t=0 bootstrap contains accounted intervals");
    if (bootstrap_accounting_state_.monitor.strict != config_.strict)
        throw std::invalid_argument("state.runtime.monitor.strict: disagrees with config.strict");
    if (bootstrap_step_state_.measurement_decimation != config_.measurement_decimation)
        throw std::invalid_argument("state.runtime.measurement_decimation: disagrees with config.measurement_decimation");
    const Wire articulated_tree =
        vread::wire_from_value(config_.rider_controller,
                               "config.rider_controller");
    const auto &articulated = sample_wire::object(articulated_tree);
    const auto nonnegative = [&](const char *name) {
        const double value = sample_wire::number(sample_wire::required(articulated, name));
        if (value < 0.)
            throw std::invalid_argument(std::string("config.rider_controller.") + name + ": must be nonnegative");
        return value;
    };
    accounting_config_.attachment_budget = {
        .foot_min_normal_n = nonnegative("pedal_min_normal_n"),
        .foot_mu = nonnegative("foot_mu"), .saddle_mu = nonnegative("saddle_mu"),
        .grip_pull_n = nonnegative("grip_pull_per_hand_n"), .max_gap_m = nonnegative("link_max_gap_m")};
    accounting_config_.timestep_s = config_.timestep_s;
    accounting_config_.period_steps = static_cast<std::size_t>(config_.control_period_steps);
    accounting_config_.measurement_decimation = config_.measurement_decimation;
    accounting_config_.retention = config_.record_retention;
    accounting_config_.async_telemetry = config_.async_telemetry;
    accounting_config_.battery_enabled = stepper_->drive().config().policies.battery.enabled;
    const mjModel *const model = stepper_->model();
    accounting_config_.max_dofs = static_cast<std::size_t>(model->nv);
    accounting_config_.max_qpos = static_cast<std::size_t>(model->nq);
    const int root_x = mj_name2id(model, mjOBJ_JOINT, "root_x");
    const int root_pitch = mj_name2id(model, mjOBJ_JOINT, "root_pitch");
    if (root_x < 0 || root_pitch < 0)
        throw std::invalid_argument("accounting requires root_x and root_pitch joints");
    const auto qpos_addresses = std::views::counted(model->jnt_qposadr, model->njnt);
    const auto dof_addresses = std::views::counted(model->jnt_dofadr, model->njnt);
    accounting_config_.columns = {
        .root_x_qpos = static_cast<std::size_t>(qpos_addresses[static_cast<std::size_t>(root_x)]),
        .root_x_dof = static_cast<std::size_t>(dof_addresses[static_cast<std::size_t>(root_x)]),
        .root_pitch_qpos = static_cast<std::size_t>(qpos_addresses[static_cast<std::size_t>(root_pitch)])};
    accounting_ = std::make_unique<PeriodAccounting>(accounting_config_,
        bootstrap_accounting_state_, physical_->generation(),
        physical_->rider_present() ? &physical_->rider_control() : nullptr,
        bootstrap_rider_contacts_.has_value() ? &stepper_->rider_contacts() : nullptr);
    committed_integration_state_ = bootstrap_.integration_state;
    next_integration_state_ = bootstrap_.integration_state;
    committed_step_ = physical_->step();
    committed_time_s_ = stepper_->data()->time;
    committed_outcome_ = crash_outcome(physical_->crash());
    physical_->initialize_presentation(committed_presentation_);
    physical_->initialize_presentation(next_presentation_);
    cache_research_boundary();
}
NATIVE_DIAG_POP

void NativeRideRuntime::test_fail_at_step(std::int64_t step) {
    const AdvancementGuard guard(advancing_);
    require_open();
    if (step < committed_step_)
        throw std::invalid_argument("test failure step is behind the committed boundary");
    test_failure_step_ = step;
}

void NativeRideRuntime::test_failure_boundary() {
    if (!test_failure_step_.has_value() || committed_step_ != *test_failure_step_) return;
    test_failure_step_.reset();
    // Same isolated fatal probe as binding.cpp::_engine_test_forward_failure.
    stepper_->mutate([&] {
        mjModel *const model = stepper_->model();
        const int previous_solver = model->opt.solver;
        model->opt.solver = 99;
        try { stepper_->forward(); }
        catch (...) { model->opt.solver = previous_solver; throw; }
        model->opt.solver = previous_solver;
    });
    throw std::runtime_error("runtime fatal probe did not trigger an engine error");
}

std::optional<std::string> NativeRideRuntime::outcome_reason() const {
    return committed_outcome_;
}

void NativeRideRuntime::cache_research_boundary() {
    const auto qpos = stepper_->qpos();
    committed_position_m_ = qpos[accounting_config_.columns.root_x_qpos];
    committed_battery_energy_j_ = stepper_->drive().battery_energy_j();
    committed_crash_ = physical_->crash();
    committed_balance_ = physical_->balance_event();
}

void NativeRideRuntime::advance_interval(const RideControl &command, double front, double rear) {
    // Async closes become visible only at a main-thread step boundary. The
    // worker never changes presentation or the live controller/contact state.
    try { accounting_->collect(false); }
    catch (const InvalidReferenceRun&) {
        physical_->capture_accounted_presentation(committed_presentation_);
        throw;
    }
    physical_->capture_accounted_presentation(committed_presentation_);
    test_failure_boundary();
    RawStep raw = physical_->advance_physics(front, rear, std::nullopt, command);
    accounting_->push(std::move(raw));
    engine::get_state(stepper_->model(), stepper_->data(),
        next_integration_state_.data(), static_cast<int>(mjSTATE_INTEGRATION));
    physical_->capture_presentation(next_presentation_);
    committed_integration_state_.swap(next_integration_state_);
    std::swap(committed_presentation_, next_presentation_);
    committed_step_ = physical_->step();
    committed_time_s_ = stepper_->data()->time;
    committed_outcome_ = crash_outcome(physical_->crash());
    cache_research_boundary();
    if (!committed_outcome_ && finish_position_m_ &&
        committed_position_m_ >= *finish_position_m_)
        committed_outcome_ = "end_of_track";
    if (committed_outcome_) flush_accounting();
    else if (accounting_->full()) {
        try { accounting_->close_period(); }
        catch (const InvalidReferenceRun&) {
            physical_->capture_accounted_presentation(committed_presentation_);
            throw;
        }
        physical_->capture_accounted_presentation(committed_presentation_);
    }
}

void NativeRideRuntime::flush_accounting() {
    try {
        accounting_->flush();
    } catch (const InvalidReferenceRun &) {
        physical_->capture_accounted_presentation(committed_presentation_);
        throw;
    }
    physical_->capture_accounted_presentation(committed_presentation_);
}

void NativeRideRuntime::set_finish_position(double position_m) {
    const AdvancementGuard guard(advancing_);
    require_open();
    if (!std::isfinite(position_m) || position_m <= committed_position_m_)
        throw std::invalid_argument("finish position must be finite and ahead of the start");
    if (committed_step_ != 0)
        throw std::invalid_argument("finish position must be configured at t=0");
    finish_position_m_ = position_m;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature is the spec'd API
AdvanceResult
NativeRideRuntime::advance(std::int64_t target_step, const RideControl &command,
                           double front_brake_demand,
                           double rear_brake_demand,
                           std::optional<double> wall_budget_s) {
    require_open();
    // ---- decode + validate every public input first ----
    // An argument rejection never mutates committed state.
    if (!std::isfinite(front_brake_demand))
        throw std::invalid_argument("front_brake_demand must be finite");
    if (!std::isfinite(rear_brake_demand))
        throw std::invalid_argument("rear_brake_demand must be finite");
    if (wall_budget_s.has_value() &&
        (!std::isfinite(*wall_budget_s) || *wall_budget_s < 0.))
        throw std::invalid_argument(
            "wall_budget_s must be finite and nonnegative");
    if (target_step < 0)
        throw std::invalid_argument("target_step must be nonnegative");
    // Reentrancy precedes every state-dependent check: while a concurrent
    // advance is in flight the committed step is a moving target, so
    // reading it (or advancing) must fail as reentrancy first.
    const AdvancementGuard guard(advancing_);
    require_open();
    if (target_step < committed_step_)
        throw std::invalid_argument(
            "target_step is behind the committed step");
    control_validate_for(command, physical_->rider_present(), physical_->drive_mode());
    std::string reason;
    stepper_->refresh_time_callback_policy();
    const auto start_time = std::chrono::steady_clock::now();
    // A previous allocation/evaluation failure may have left a full,
    // unacknowledged raw period. Retry it before accepting new physics.
    if (accounting_->full()) flush_accounting();
    try {
        while (committed_step_ < target_step && !committed_outcome_.has_value()) {
            if (wall_budget_s.has_value() &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time)
                    .count() >= *wall_budget_s) {
                reason = "budget";
                break;
            }
            advance_interval(command, front_brake_demand, rear_brake_demand);
        }
    } catch (const engine::EngineFailure &) {
        // Accounting uses only owned captures, never the poisoned arena.
        // A strict reference rejection cannot replace the fatal engine error.
        const std::exception_ptr failure = std::current_exception();
        // NOLINTNEXTLINE(bugprone-empty-catch) captures remain retryable; the engine failure must not be masked
        try { flush_accounting(); } catch (...) {}
        std::rethrow_exception(failure);
    }
    if (reason.empty()) reason = committed_outcome_.has_value() ? "outcome" : "target";
    return {.step = committed_step_, .time_s = committed_time_s_,
            .reason = std::move(reason), .outcome = outcome_reason()};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature is the spec'd API
PhysicalStep::ProbeResult
NativeRideRuntime::probe_step_inputs(const RideControl &command,
                                     double front_brake_demand,
                                     double rear_brake_demand) {
    require_open();
    if (!std::isfinite(front_brake_demand))
        throw std::invalid_argument("front_brake_demand must be finite");
    if (!std::isfinite(rear_brake_demand))
        throw std::invalid_argument("rear_brake_demand must be finite");
    const AdvancementGuard guard(advancing_);
    require_open();
    return physical_->probe_step_inputs(command, front_brake_demand,
                                        rear_brake_demand);
}

RuntimeSnapshot NativeRideRuntime::snapshot() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    const auto &latest = accounting_->latest_sample();
    return {.generation = physical_->generation(), .step = committed_step_,
            .time_s = committed_time_s_, .integration_state = committed_integration_state_,
            .outcome = outcome_reason(), .latest_sample = latest,
            .view = committed_presentation_.as_wire(),
            .model_status = accounting_->state().model_status.as_wire(),
            .first_failure = accounting_->state().monitor.first_failure};
}

RenderFrame NativeRideRuntime::render_frame() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    // Copy the committed cache under ONE guard: a reset cannot split the
    // integration vector, presentation and generation across two calls.
    RuntimeSnapshot state{.generation = physical_->generation(), .step = committed_step_,
        .time_s = committed_time_s_, .integration_state = committed_integration_state_,
        .outcome = outcome_reason(), .latest_sample = accounting_->latest_sample(),
        .view = committed_presentation_.as_wire(),
        .model_status = accounting_->state().model_status.as_wire(),
        .first_failure = accounting_->state().monitor.first_failure};
    return {.state=std::move(state), .presentation=committed_presentation_, .model_sha256=model_image_.sha256};
}

ModelImage NativeRideRuntime::model_image() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    return model_image_;
}

void NativeRideRuntime::flush() {
    const AdvancementGuard guard(advancing_);
    require_open();
    flush_accounting();
}

RuntimeSampleBatch NativeRideRuntime::prepare_samples() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    return accounting_->prepare_samples();
}

void NativeRideRuntime::acknowledge_samples(const RuntimeSampleBatch &batch) {
    const AdvancementGuard guard(advancing_);
    require_open();
    accounting_->acknowledge_samples(batch);
}

WireObject NativeRideRuntime::accounting_state() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    return accounting_->state_wire();
}

SampleColumns NativeRideRuntime::recorded_columns() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    return accounting_->recorded_columns();
}

std::span<const SamplePtr> NativeRideRuntime::recorded_intervals() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    return accounting_->recorded_samples();
}

OwnedNativeEpisode NativeRideRuntime::seal_episode() {
    const AdvancementGuard guard(advancing_);
    require_open();
    flush_accounting();
    return accounting_->seal_episode();
}

void NativeRideRuntime::release_samples() {
    const AdvancementGuard guard(advancing_);
    require_open();
    if (const auto &latest = accounting_->latest_sample(); latest != nullptr)
        accounting_->acknowledge_through(latest->interval_id);
}

std::optional<std::string> NativeRideRuntime::take_pending_warning() {
    const AdvancementGuard guard(advancing_);
    require_open();
    // An earlier pending warning (e.g. the reset-time flush's) drains
    // first — the adapter emits at most one per call, exactly like the
    // old emit_reference_warning position.
    if (pending_warning_.has_value()) {
        std::optional<std::string> out;
        out.swap(pending_warning_);
        return out;
    }
    return accounting_->take_warning();
}

void NativeRideRuntime::reset() {
    const AdvancementGuard guard(advancing_);
    require_open();
    if (physical_->generation() == std::numeric_limits<int>::max())
        throw std::overflow_error("native runtime generation overflow");
    const int next = physical_->generation() + 1;
    auto accounting = std::make_unique<PeriodAccounting>(accounting_config_,
        bootstrap_accounting_state_, next,
        physical_->rider_present() ? &physical_->rider_control() : nullptr,
        bootstrap_rider_contacts_.has_value() ? &stepper_->rider_contacts() : nullptr);
    // Like PhysicalRuntime.reset(), close the outgoing period first. A
    // strict rejection publishes that period and leaves its generation
    // intact; callers may drain it and retry reset without losing evidence.
    flush_accounting();
    // The outgoing owner is replaced below — lift its pending warning now
    // so the reset cannot silently drop it (the old emit_reference_warning
    // ran at this exact position).
    if (!pending_warning_.has_value())
        pending_warning_ = accounting_->take_warning();
    // Stepper::reset is the supported recovery path for a poisoned owner.
    // The subsequent bootstrap restore replaces all its reset-time solves.
    stepper_->reset();
    apply_bootstrap();
    physical_->set_generation(next);
    accounting_ = std::move(accounting);
    committed_integration_state_ = bootstrap_.integration_state;
    next_integration_state_ = bootstrap_.integration_state;
    committed_step_ = bootstrap_step_state_.step;
    committed_time_s_ = stepper_->data()->time;
    committed_outcome_ = crash_outcome(physical_->crash());
    physical_->capture_presentation(committed_presentation_);
    physical_->capture_presentation(next_presentation_);
    cache_research_boundary();
    test_failure_step_.reset();
}

void NativeRideRuntime::close() {
    // Freeing under an in-flight advance is a UAF — take the guard first.
    // Idempotent (a second close re-acquires and no-ops), like before.
    const AdvancementGuard guard(advancing_);
    closed_.store(true);
    accounting_.reset();
    physical_.reset();
    stepper_.reset();
}

NativeRideRuntime::~NativeRideRuntime() = default;

} // namespace runtime
