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
#include "spatial_timing.hpp"
#include "spatial_tire_factory.hpp"
#include "checkpoint_io.hpp"
#include "presentation_checkpoint.hpp"
#include "spatial_checkpoint_internal.hpp"
#include "../initialization/capture.hpp"
#include "../io/json.hpp"

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
#include <type_traits>
#include <utility>
#include <vector>
#include <mujoco/mujoco.h>

#include "../diag.hpp"
#include "../engine_call.hpp"
#include "../engine_identity.hpp"
#include "../io/hash.hpp"
#include "../platform/provenance.hpp"
#include "../rider/state_codec.hpp"
#include "../spatial/frames.hpp"
#include "../stepper.hpp"
#include "../tyre/profile.hpp"
#include "../writer_config.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/pneumatic_3d_tire.hpp"
#include "../terrain/spec_io.hpp"
#include "value_readers.hpp"

namespace runtime {
namespace {

namespace bio = bike::io;

std::vector<std::byte> tire_state_bytes(const Stepper& stepper) {
    if(stepper.has_spatial_tire())return stepper.spatial_tire_writer().state_bytes();
    if(!stepper.has_tire())return {};
    const auto row=stepper.tire_state();
    const auto bytes=std::as_bytes(std::span(row));
    return {bytes.begin(),bytes.end()};
}

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
    const auto required=vread::keys(
                         "step", "generation", "measurement_decimation",
                         "applied_control", "held_control",
                         "held_rider_terms", "rollback_hold",
                         "research_accounting_valid", "initializing",
                         "filters", "balance", "crash", "contacts",
                         "contact_query", "probe_query", "energy", "history",
                         "model_status", "monitor");
    if(bootstrap.spatial)vread::exact(rt,required,vread::keys("continuation"));
    else vread::exact(rt,required);
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
    out.spatial=bootstrap.spatial;
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
    out.spatial=config.spatial;
    out.terrain3d=config.terrain3d;
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
    const auto &drive = stepper.drive().config();
    return {resolve(config.geometry.front_brake, "front"),
            resolve(config.geometry.rear_brake, "rear"),
            drive.brake_ceiling_nm,
            BrakeHoldSpec{.torque_nm = drive.brake_hold_torque_nm,
                          .stiffness_nm_rad = drive.brake_hold_stiffness_nm_rad,
                          .damping_nms_rad = drive.brake_hold_damping_nms_rad,
                          .engage_radps = drive.brake_hold_engage_radps,
                          .release_radps = drive.brake_hold_release_radps}};
}

// The PhysicalStep assembly — pose/config/vertices decode plus the
// constructor — carries a frame too large for the debug stack budget, so
// it owns a separate function scope.
std::unique_ptr<PhysicalStep>
build_physical_step(Stepper &stepper, const RuntimeConfig &config,
    const bike::rider::SteerProfile* acquired_steer_profile = nullptr) {
    // The compiled terrain cross-section is constant model geometry — the
    // tire writer's own copy was read from this same post-forward data.
    std::vector<std::array<double, 2>> vertices;
    if(config.spatial) {
        if(!config.terrain3d)
            throw std::invalid_argument("config.spatial.terrain: physical source missing");
        // Keep the canonical centreline for initial-state transfer and legacy
        // reporting. Live spatial rider preview queries the physical source
        // at the current wheel positions along the current body heading.
        constexpr std::size_t samples=1025;
        vertices.reserve(samples);
        const auto query=config.terrain3d->query_view();
        const double first=config.terrain3d->x_min_m(),last=config.terrain3d->x_max_m();
        for(std::size_t i=0;i<samples;++i) {
            const double x=i+1==samples?last:first+(last-first)*static_cast<double>(i)/(samples-1);
            const auto sample=query.sample(x,0.);
            if(sample.status!=bike::terrain::QueryStatus3d::ok)
                throw std::invalid_argument("config.spatial.terrain: invalid physical centreline sample");
            vertices.push_back({x,sample.height_m});
        }
    } else {
        vertices=biketyre::compiled_profile_vertices(stepper.model(),stepper.data(),"terrain");
    }
    std::optional<rider::SpindlePose> pose;
    std::optional<rider::SpindleConfig> rider_cfg;
    if (!bio::is_null(config.geometry.pose)) {
        pose = config.spatial?rider::decode_spatial_spindle_pose(config.geometry.pose,"config.geometry.pose"):
            rider::decode_spindle_pose(config.geometry.pose,"config.geometry.pose");
        rider_cfg = config.spatial?rider::decode_spatial_spindle_config(config.rider_controller):
            rider::decode_spindle_config(config.rider_controller);
    }
    return std::make_unique<PhysicalStep>(
        stepper, step_config(config), std::move(vertices),
        static_brake(config, *stepper.model(), stepper), pose, rider_cfg, acquired_steer_profile);
}

} // namespace

std::unique_ptr<PhysicalStep>
make_physical_step(Stepper& stepper, const RuntimeConfig& config) {
    install_spatial_tire(stepper,config);
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
NativeRideRuntime::NativeRideRuntime(RuntimeBootstrap bootstrap, ContinuationTag)
    : config_(std::move(bootstrap.config)), bootstrap_(std::move(bootstrap.state)),
      model_image_(std::move(bootstrap.model)) {
    if(!config_.spatial)throw std::invalid_argument("checkpoint continuation requires spatial runtime2");
    init(true);
}
NativeRideRuntime::NativeRideRuntime(RuntimeBootstrap bootstrap, ProbeTag,
    const bike::rider::SteerProfile* acquired_steer_profile)
    : config_(std::move(bootstrap.config)), bootstrap_(std::move(bootstrap.state)),
      model_image_(std::move(bootstrap.model)) {
    if(!config_.spatial)throw std::invalid_argument("isolated probe requires spatial runtime2");
    init(false,true,acquired_steer_profile);
}
NATIVE_DIAG_POP

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void NativeRideRuntime::init(bool continuation, bool probe,
    const bike::rider::SteerProfile* acquired_steer_profile) {
    if(config_.spatial || bootstrap_.spatial || config_.schema!=kRuntimeSchema || bootstrap_.schema!=kRuntimeSchema)
        validate_schema_family(config_,bootstrap_.schema);
    const bool has_continuation=bio::find_member(bio::as_object(bootstrap_.runtime,"state.runtime"),"continuation")!=nullptr;
    if(continuation!=has_continuation)
        throw std::invalid_argument("state.runtime.continuation: use the complete checkpoint restore boundary");
    // Ownership is already committed; the digest re-verification keeps a
    // fabricated RuntimeBootstrap honest: the image bytes — not the
    // recorded sha field — prove the model identity.
    if (bio::sha256(model_image_.mjb) != bootstrap_.model_digest)
        throw std::invalid_argument(
            "state.model_digest: model artifact bytes do not match");
    model_image_.sha256=bootstrap_.model_digest;
    if(config_.spatial && !config_.terrain3d)
        config_.terrain3d=bike::terrain::detail::load_terrain3d_source(config_.spatial->terrain);
    // The writer-config decode position: exactly where the nanobind
    // adapter decoded the handle — before the model load.
    const nativecfg::WriterConfig writer_config =
        nativecfg::decode(config_.writer_config);
    stepper_ = std::make_unique<Stepper>(model_image_.mjb, writer_config);
    install_spatial_tire(*stepper_,config_);
    // Same guarantee the decode gave at compose time — repeated against
    // the live model so a fabricated bootstrap record cannot pass.
    verify_model_dims(bootstrap_.model_dims, *stepper_->model());
    if(bootstrap_.model_mutable.spatial_arrays) {
        const auto* model=stepper_->model();
        const std::array<mjtSize,7> widths{model->nv,3*model->nsite,model->ntendon,model->ntendon,
            2*model->ntendon,2*model->ntendon,model->nwrap};
        if(!config_.spatial || !bootstrap_.model_mutable.dof_frictionloss.empty() || !bootstrap_.model_mutable.site_pos.empty())
            throw std::invalid_argument("state.model_mutable.spatial_arrays: conflicting owner inventory");
        for(std::size_t i=0;i<widths.size();++i) {
            const auto& row=(*bootstrap_.model_mutable.spatial_arrays)[i];
            if(std::cmp_not_equal(row.size(),widths[i]) || !std::ranges::all_of(row,[](double value){return std::isfinite(value);}))
                throw std::invalid_argument("state.model_mutable.spatial_arrays: invalid model-bound row");
        }
    }
    if(config_.spatial) {
        if(!bootstrap_.spatial || !bootstrap_.tire3d || !stepper_->has_spatial_tire())
            throw std::invalid_argument("bootstrap.spatial.tire: producer state missing");
        const auto topology=bike::spatial::bind_topology(*stepper_->model(),*config_.spatial);
        validate_spatial_identity(config_,*bootstrap_.spatial,topology);
        const auto decoded=bike::pneumatic3d::decode_tyre_state(
            bike::pneumatic3d::encode_tyre_state(*bootstrap_.tire3d),stepper_->spatial_tire_writer().state().identity);
        if(!decoded)throw std::invalid_argument("bootstrap.spatial.tire: "+decoded.error());
    }
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
    physical_ = acquired_steer_profile
        ? build_physical_step(*stepper_, config_, acquired_steer_profile)
        : make_physical_step(*stepper_, config_);
    bootstrap_step_state_ = decode_step_state(bootstrap_);
    // The captured controller state and the configured rider ensemble
    // arrive together (setup.py emits both or neither).
    if (bootstrap_step_state_.rider_controller.has_value() !=
        physical_->rider_present())
        throw std::invalid_argument(
            "state.rider_controller does not match the rider config");
    if(bootstrap_step_state_.rider_controller) {
        const auto& controller=*bootstrap_step_state_.rider_controller;
        if(controller.spatial.has_value()!=config_.spatial.has_value())
            throw std::invalid_argument("state.rider_controller.rider.spatial: runtime family mismatch");
        physical_->rider_control().validate_state(controller);
        if(config_.spatial) {
            const auto* model=stepper_->model();
            std::unique_ptr<mjData,decltype(&mj_deleteData)> staged(engine::make_data(model),&mj_deleteData);
            if(!staged)throw std::runtime_error("bootstrap spatial rider staging allocation failed");
            engine::set_state(model,staged.get(),bootstrap_.integration_state.data(),mjSTATE_INTEGRATION);
            physical_->rider_control().validate_spatial_lanes(controller,
                std::span<const double>(staged->qpos,static_cast<std::size_t>(model->nq)),
                std::span<const double>(staged->qvel,static_cast<std::size_t>(model->nv)));
        }
    }
    apply_bootstrap();
    if(!probe) initialize_accounting(continuation);
}
NATIVE_DIAG_POP

void NativeRideRuntime::require_open() const {
    if (closed_) throw std::logic_error("NativeRideRuntime is closed");
}

void NativeRideRuntime::require_boundary_ready() const {
    if (boundary_recovery_required_)
        throw std::logic_error("runtime boundary capture failed; reset required");
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
        if(bootstrap_.model_mutable.spatial_arrays) {
            const std::array<double*,7> targets{model->dof_frictionloss,model->site_pos,
                model->tendon_stiffness,model->tendon_damping,model->tendon_lengthspring,
                model->tendon_range,model->wrap_prm};
            for(std::size_t i=0;i<targets.size();++i) {
                const auto& values=(*bootstrap_.model_mutable.spatial_arrays)[i];
                std::ranges::copy(values,targets[i]);
            }
        }
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
        if(config_.spatial)
            stepper_->restore_spatial_tire(*bootstrap_.tire3d);
        else if (stepper_->has_tire() || !bootstrap_.tire.names.empty())
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
        physical_->refresh_spatial_observation_cache();
        if(config_.spatial && bike::io::find_member(bike::io::as_object(bootstrap_.runtime,"state.runtime"),"continuation"))
            engine::set_state(model,data,bootstrap_.integration_state.data(),mjSTATE_INTEGRATION);
        stepper_->note_posvel_fresh();
    });
}

// Decode the A4-owned portion once; reset replays typed owned values.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void NativeRideRuntime::initialize_accounting(bool continuation) {
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
    if (!continuation && (bootstrap_step_state_.step != 0 || stepper_->data()->time != 0.))
        throw std::invalid_argument("state.runtime.step: bootstrap must describe t=0");
    if (!continuation && (bootstrap_accounting_state_.history.last_id.has_value() ||
        bootstrap_accounting_state_.history.last_end.has_value() ||
        bootstrap_accounting_state_.model_status.last_interval != -1))
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
    accounting_config_.max_actuators = static_cast<std::size_t>(model->nu);
    if(config_.spatial) {
        accounting_config_.columns.spatial=true;
    } else {
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
    }
    auto accounting = std::make_unique<PeriodAccounting>(accounting_config_,
        bootstrap_accounting_state_, physical_->generation(),
        physical_->rider_present() ? &physical_->rider_control() : nullptr,
        bootstrap_rider_contacts_.has_value() ? &stepper_->rider_contacts() : nullptr);
    auto spare_integration = bootstrap_.integration_state;
    next_integration_state_ = bootstrap_.integration_state;
    auto presentation = std::make_unique<PresentationState>();
    auto spare_presentation = std::make_unique<PresentationState>();
    physical_->initialize_presentation(*presentation);
    physical_->initialize_presentation(*spare_presentation);
    auto boundary = capture_boundary();
    // initialize_accounting is also used by the cold validation rebase path.
    // Its previous readable boundary survives every staging failure above.
    std::swap(next_presentation_, *presentation);
    accounting_.swap(accounting);
    commit_boundary(std::move(boundary));
    next_integration_state_.swap(spare_integration);
    std::swap(next_presentation_, *spare_presentation);
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

NativeRideRuntime::BoundaryState NativeRideRuntime::capture_boundary() const {
    BoundaryState next;
    next.generation = physical_->generation();
    next.step = physical_->step();
    next.time_s = stepper_->data()->time;
    next.position_m = physical_->longitudinal_position_m();
    next.battery_energy_j = stepper_->drive().battery_energy_j();
    next.crash = physical_->crash();
    next.balance = physical_->balance_event();
    next.spatial = physical_->spatial_state();
    if(next.spatial) {
        if(next.spatial->outcome.latched.terminal)
            next.outcome=std::string(bike::spatial::to_string(next.spatial->outcome.latched.reason));
    } else next.outcome = crash_outcome(next.crash);
    next.tire_state_bytes = tire_state_bytes(*stepper_);
    return next;
}

void NativeRideRuntime::commit_boundary(BoundaryState&& next) noexcept {
    static_assert(std::is_nothrow_swappable_v<PresentationState>);
    static_assert(std::is_nothrow_swappable_v<BoundaryState>);
    committed_integration_state_.swap(next_integration_state_);
    std::swap(committed_presentation_, next_presentation_);
    committed_generation_ = next.generation;
    committed_step_ = next.step;
    committed_time_s_ = next.time_s;
    committed_position_m_ = next.position_m;
    committed_battery_energy_j_ = next.battery_energy_j;
    committed_crash_.swap(next.crash);
    committed_balance_.swap(next.balance);
    committed_outcome_.swap(next.outcome);
    committed_spatial_.swap(next.spatial);
    committed_tire_state_bytes_.swap(next.tire_state_bytes);
}

void NativeRideRuntime::advance_interval(const RideControl &command, double front, double rear,
                                        GoldenForceObservation *golden) {
    require_boundary_ready();
    // Async closes become visible only at a main-thread step boundary. The
    // worker never changes presentation or the live controller/contact state.
    {
    const spatial::timing::Scope timing(spatial::timing::Region::accounting);
    try { accounting_->collect(false); }
    catch (const InvalidReferenceRun&) {
        physical_->capture_accounted_presentation(committed_presentation_);
        throw;
    }
    physical_->capture_accounted_presentation(committed_presentation_);
    }
    try { test_failure_boundary(); }
    catch(...) { if(config_.spatial)boundary_recovery_required_=true; throw; }
    RawStep raw = [&] {
        try { return physical_->advance_physics(front, rear, std::nullopt, command, golden); }
        catch (...) {
            const auto failure=std::current_exception();
            if(config_.spatial)boundary_recovery_required_=true;
            // A failed candidate cannot publish an engine state. Its typed
            // invalid outcome remains readable beside the last valid prefix.
            try {
                const auto& causal=physical_->spatial_state();
                if(causal && causal->outcome.latched.terminal && committed_spatial_ &&
                   !committed_spatial_->outcome.latched.terminal) {
                    // All other causal owners belong to the last accepted
                    // engine/tyre generation. A failed candidate contributes
                    // only its terminal record, never an advanced FIFO/latch.
                    boundary_recovery_required_=true;
                    auto staged=*committed_spatial_;
                    staged.outcome=causal->outcome;
                    auto label=std::optional<std::string>(bike::spatial::to_string(staged.outcome.latched.reason));
                    committed_spatial_=std::move(staged);committed_outcome_.swap(label);
                }
            } catch (...) { /* Preserve the original force/engine failure. */ }
            std::rethrow_exception(failure);
        }
    }();
    try {
        const spatial::timing::Scope timing(spatial::timing::Region::accounting);
        engine::get_state(stepper_->model(), stepper_->data(),
            next_integration_state_.data(), static_cast<int>(mjSTATE_INTEGRATION));
        physical_->capture_presentation(next_presentation_);
        auto boundary = capture_boundary();
        if (!boundary.outcome && ((finish_position_m_ && boundary.position_m >= *finish_position_m_) ||
            (boundary.spatial && completion_step_ && boundary.step>=*completion_step_))) {
            if(boundary.spatial) {
                // The step has already evaluated every higher-priority cause.
                // Latch completion directly, without a second dwell tick.
                auto& state=boundary.spatial->outcome;
                const auto pose=bike::spatial::body_pose(*stepper_->model(),*stepper_->data(),physical_->topology()->root);
                state.latched={.reason=bike::spatial::SpatialReason::requested_completion,
                    .terminal=true,.valid=true,.tick=boundary.step-1,.time_s=boundary.time_s,
                    .position_world_m=pose.position_world_m,.diagnostic={}};
                state.valid_mask=1;
                physical_->commit_spatial_state(physical_->prepare_spatial_state(*boundary.spatial));
                boundary.outcome=std::string(bike::spatial::to_string(state.latched.reason));
            } else boundary.outcome = "end_of_track";
        }
        if(boundary.spatial) {
            const auto& outcome=boundary.spatial->outcome.latched;
            raw.channels.emplace_back("spatial_outcome",Wire(WireObject{
                {"spatial_schema",Wire(1)},{"reason",Wire(bike::spatial::to_string(outcome.reason))},
                {"terminal",Wire(outcome.terminal)},{"valid",Wire(outcome.valid)},
                {"tick",Wire(outcome.tick)},{"time_s",Wire(outcome.time_s)},
                {"position_world_m",wire_array(outcome.position_world_m)},
                {"accepted_position_m",Wire(boundary.position_m)}}));
        }
        accounting_->push(std::move(raw));
        commit_boundary(std::move(boundary));
    } catch (...) {
        boundary_recovery_required_ = true;
        throw;
    }
    if (committed_outcome_) flush_accounting();
    else if (accounting_->full()) {
        const spatial::timing::Scope timing(spatial::timing::Region::accounting);
        try { accounting_->close_period(); }
        catch (const InvalidReferenceRun&) {
            physical_->capture_accounted_presentation(committed_presentation_);
            throw;
        }
        physical_->capture_accounted_presentation(committed_presentation_);
    }
}

void NativeRideRuntime::flush_accounting() {
    const spatial::timing::Scope timing(spatial::timing::Region::accounting);
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

void NativeRideRuntime::set_completion_step(std::int64_t step) {
    const AdvancementGuard guard(advancing_);
    require_open();require_boundary_ready();
    if(!config_.spatial || committed_step_!=0 || step<=0)
        throw std::invalid_argument("completion step requires a positive spatial limit configured at t=0");
    completion_step_=step;
}

std::optional<std::string> NativeRideRuntime::checkpoint_unavailable_reason() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    if(!config_.spatial)return "spatial checkpoint requires runtime schema2";
    if(boundary_recovery_required_)
        return "failed physical boundary requires reset; accepted prefix has no complete resumable writer snapshot";
    if(accounting_->asynchronous())return "checkpoint requires synchronous accounting";
    if(physical_->step()!=committed_step_||stepper_->time()!=committed_time_s_)
        return "live engine state is not the accepted checkpoint boundary";
    return {};
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
    require_boundary_ready();
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
    require_boundary_ready();
    if(config_.spatial)
        return spatial::CheckpointAccess::probe_locked(*this,command,front_brake_demand,rear_brake_demand);
    return physical_->probe_step_inputs(command, front_brake_demand,
                                        rear_brake_demand);
}

RuntimeSnapshot NativeRideRuntime::snapshot() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    const auto &latest = accounting_->latest_sample();
    return {.generation = committed_generation_, .step = committed_step_,
            .time_s = committed_time_s_, .integration_state = committed_integration_state_,
            .outcome = outcome_reason(), .latest_sample = latest,
            .view = committed_presentation_.as_wire(),
            .model_status = accounting_->state().model_status.as_wire(),
            .first_failure = accounting_->state().monitor.first_failure,
            .model_sha256=model_image_.sha256, .spatial=committed_spatial_,
            .tire_state_bytes=committed_tire_state_bytes_};
}

RenderFrame NativeRideRuntime::render_frame() const {
    const AdvancementGuard guard(advancing_);
    require_open();
    // Copy the committed cache under ONE guard: a reset cannot split the
    // integration vector, presentation and generation across two calls.
    RuntimeSnapshot state{.generation = committed_generation_, .step = committed_step_,
        .time_s = committed_time_s_, .integration_state = committed_integration_state_,
        .outcome = outcome_reason(), .latest_sample = accounting_->latest_sample(),
        .view = committed_presentation_.as_wire(),
        .model_status = accounting_->state().model_status.as_wire(),
        .first_failure = accounting_->state().monitor.first_failure,
        .model_sha256=model_image_.sha256, .spatial=committed_spatial_,
        .tire_state_bytes=committed_tire_state_bytes_};
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
    if (committed_generation_ == std::numeric_limits<int>::max())
        throw std::overflow_error("native runtime generation overflow");
    const int next = committed_generation_ + 1;
    auto accounting = std::make_unique<PeriodAccounting>(accounting_config_,
        bootstrap_accounting_state_, next,
        physical_->rider_present() ? &physical_->rider_control() : nullptr,
        bootstrap_rider_contacts_.has_value() ? &stepper_->rider_contacts() : nullptr);
    next_integration_state_ = bootstrap_.integration_state;
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
    try {
        stepper_->reset();
        apply_bootstrap();
        physical_->capture_presentation(next_presentation_);
        auto boundary = capture_boundary();
        boundary.generation = next;
        // apply_bootstrap restored the original physical generation. Stage
        // the new lifetime on both published faces before the no-throw commit.
        if (next_presentation_.spatial)
            next_presentation_.spatial->generation = static_cast<std::uint64_t>(next);
        // No owning copy follows publication. A staging failure leaves snapshot
        // generation and all other committed caches at the outgoing boundary.
        physical_->set_generation(next);
        accounting_.swap(accounting);
        commit_boundary(std::move(boundary));
        boundary_recovery_required_ = false;
    } catch (...) {
        boundary_recovery_required_ = true;
        throw;
    }
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

namespace runtime::spatial {
namespace {
using namespace checkpoint_io;
std::string config_digest(const RuntimeConfig& c) {
    if(!c.spatial)throw std::invalid_argument("checkpoint.config: spatial runtime required");
    auto spatial=bike::spatial::spatial_config_to_wire(*c.spatial);
    auto terrain=field(spatial,"terrain");
    // Relocation changes the source pathname, never the acquired physical
    // terrain identity (the latter is pinned by the writer and checkpoint).
    set(terrain,"path",text(""));set(spatial,"terrain",std::move(terrain));
    if(c.spatial->steer.controller!=bike::spatial::SteerController::off &&
       !bike::rider::builtin_steer_profile(c.spatial->steer.profile_id)) {
        auto steer=field(spatial,"steer");set(steer,"profile_id",field(steer,"profile_sha256"));
        set(spatial,"steer",std::move(steer));
    }
    const auto named=[](const std::optional<std::string>& name){return name?text(*name):Value{};};
    const Value document{Object{{"schema",integer(c.schema)},{"dt",real(c.timestep_s)},
        {"period",real(c.control_period_s)},{"period_steps",integer(c.control_period_steps)},
        {"strict",Value{c.strict}},{"async",Value{c.async_telemetry}},{"tyre_threads",integer(c.tyre_threads)},
        {"measurement",integer(c.measurement_decimation)},{"retention",integer(static_cast<int>(c.record_retention))},
        {"writer",c.writer_config},{"rider_controller",c.rider_controller},{"rider_intent",c.rider_intent},
        {"spatial",std::move(spatial)},
        {"monitors",reals(std::array{c.monitors.balance_floor_mps,c.monitors.balance_dwell_s,
            c.monitors.balance_grace_s,c.monitors.grounded_hold_s})},
        {"geometry",Value{Object{{"crank",real(c.geometry.crank_length_m)},
            {"front_joint",text(c.geometry.front_brake.joint)},{"front_dof",integer(c.geometry.front_brake.dof_index)},
            {"rear_joint",text(c.geometry.rear_brake.joint)},{"rear_dof",integer(c.geometry.rear_brake.dof_index)},
            {"front_body",named(c.geometry.front_wheel_body)},{"rear_body",named(c.geometry.rear_wheel_body)},
            {"com_site",named(c.geometry.com_marker_site)},{"pose",c.geometry.pose}}}}}};
    const auto bytes=bike::io::canonical_json(document);
    return bike::io::sha256(std::as_bytes(std::span(bytes.data(),bytes.size())));
}
Value execution(const RuntimeIdentity& identity) {
    return Value{Object{{"engine",text(identity.mujoco_version)},
        {"source",text(identity.native_source_sha256)},{"build",text(identity.build_context_json)}}};
}
template<std::size_t N> std::array<double,N> array(const Value& value) {
    const auto values=numbers(value,N);std::array<double,N> out{};
    std::ranges::copy(values,out.begin());return out;
}
Value rider_kinematics(const std::optional<RiderKinematicState>& state) {
    if(!state)return {};
    Array road;road.reserve(state->road.size());
    for(const auto& point:state->road)road.push_back(reals(std::array{point.x_m,point.height_m,point.grade}));
    return Value{Object{{"qpos",reals(state->qpos)},{"qvel",reals(state->qvel)},
        {"time",real(state->rider_time_s)},{"road",Value{std::move(road)}}}};
}
std::optional<RiderKinematicState> rider_kinematics(const Value& value,const mjModel& model,double time) {
    if(bike::io::is_null(value))return {};
    exact(value,{"qpos","qvel","time","road"});
    RiderKinematicState out{.qpos=numbers(field(value,"qpos"),static_cast<std::size_t>(model.nq)),
        .qvel=numbers(field(value,"qvel"),static_cast<std::size_t>(model.nv)),
        .rider_time_s=number(field(value,"time"))};
    if(out.rider_time_s<0.||out.rider_time_s>time)vread::invalid("checkpoint.rider_kinematics.time","outside accepted prefix");
    // This codec belongs to spatial continuation. Samples follow the saved
    // body heading; their world X can be equal (+Y) or decreasing (-X).
    for(const auto& item:bike::io::as_array(field(value,"road"),"checkpoint.rider_kinematics.road")) {
        const auto point=array<3>(item);
        out.road.push_back(RoadSample{.x_m=point[0],.height_m=point[1],.grade=point[2]});
    }
    return out;
}
Value truth(const SpatialTruth& t) {
    return Value{Object{{"position",reals(t.root.position_world_m)},
        {"quaternion",reals(t.root.quat_world_from_body_wxyz)},
        {"velocity",reals(t.root.linear_velocity_world_mps)},
        {"angular_velocity",reals(t.root.angular_velocity_world_rad_s)},
        {"attitude",reals(std::array{t.attitude.roll_rad,t.attitude.pitch_rad,t.attitude.yaw_rad})},
        {"attitude_valid",Value{t.attitude.valid}},
        {"steer",reals(std::array{t.steer_angle_rad,t.steer_rate_rad_s,t.steer_torque_nm,t.steer_work_j})},
        {"lateral",real(t.lateral_displacement_m)},
        {"contact",Value{Array{Value{t.contact_valid[0]},Value{t.contact_valid[1]}}}},
        {"slip",reals(t.lateral_slip_mps)},{"load",reals(t.normal_load_n)},{"slip_path",reals(t.slip_path_m)},
        {"front_normal",reals(t.support_normal_world[0])},{"rear_normal",reals(t.support_normal_world[1])},
        {"front_unloaded",Value{t.front_unloaded}},{"front_lift",Value{t.front_lift}},{"flight",Value{t.flight}}}};
}
SpatialTruth truth(const Value& value) {
    exact(value,{"position","quaternion","velocity","angular_velocity","attitude","attitude_valid",
        "steer","lateral","contact","slip","load","slip_path","front_normal","rear_normal","front_unloaded","front_lift","flight"});
    SpatialTruth out;
    out.root.position_world_m=array<3>(field(value,"position"));out.root.quat_world_from_body_wxyz=array<4>(field(value,"quaternion"));
    out.root.linear_velocity_world_mps=array<3>(field(value,"velocity"));out.root.angular_velocity_world_rad_s=array<3>(field(value,"angular_velocity"));
    const auto angles=array<3>(field(value,"attitude"));
    out.attitude={.roll_rad=angles[0],.pitch_rad=angles[1],.yaw_rad=angles[2],.valid=boolean(field(value,"attitude_valid"))};
    const auto steer=array<4>(field(value,"steer"));out.steer_angle_rad=steer[0];out.steer_rate_rad_s=steer[1];out.steer_torque_nm=steer[2];out.steer_work_j=steer[3];
    out.lateral_displacement_m=number(field(value,"lateral"));
    const auto& contact=bike::io::as_array(field(value,"contact"),"checkpoint.truth.contact");
    if(contact.size()!=2)vread::invalid("checkpoint.truth.contact","wrong wheel count");
    out.contact_valid={boolean(contact[0]),boolean(contact[1])};
    out.lateral_slip_mps=array<2>(field(value,"slip"));out.normal_load_n=array<2>(field(value,"load"));out.slip_path_m=array<2>(field(value,"slip_path"));
    out.support_normal_world={array<3>(field(value,"front_normal")),array<3>(field(value,"rear_normal"))};
    out.front_unloaded=boolean(field(value,"front_unloaded"));out.front_lift=boolean(field(value,"front_lift"));out.flight=boolean(field(value,"flight"));
    double norm=0.;for(const auto q:out.root.quat_world_from_body_wxyz)norm+=q*q;
    if(std::abs(norm-1.)>2e-9)vread::invalid("checkpoint.truth.quaternion","nonunit orientation");
    return out;
}
struct EngineRows {
    std::array<std::string_view,12> names{"qacc","qacc_smooth","qfrc_constraint","qfrc_smooth","qfrc_inverse",
        "qfrc_actuator","actuator_force","sensordata","cacc","cfrc_int","cfrc_ext","efc_force"};
    std::array<std::span<double>,12> rows;
    EngineRows(const mjModel& m,mjData& d) : rows{
        std::span<double>(d.qacc,static_cast<std::size_t>(m.nv)),
        std::span<double>(d.qacc_smooth,static_cast<std::size_t>(m.nv)),
        std::span<double>(d.qfrc_constraint,static_cast<std::size_t>(m.nv)),
        std::span<double>(d.qfrc_smooth,static_cast<std::size_t>(m.nv)),
        std::span<double>(d.qfrc_inverse,static_cast<std::size_t>(m.nv)),
        std::span<double>(d.qfrc_actuator,static_cast<std::size_t>(m.nv)),
        std::span<double>(d.actuator_force,static_cast<std::size_t>(m.nu)),
        std::span<double>(d.sensordata,static_cast<std::size_t>(m.nsensordata)),
        std::span<double>(d.cacc,static_cast<std::size_t>(6*m.nbody)),
        std::span<double>(d.cfrc_int,static_cast<std::size_t>(6*m.nbody)),
        std::span<double>(d.cfrc_ext,static_cast<std::size_t>(6*m.nbody)),
        std::span<double>(d.efc_force,static_cast<std::size_t>(d.nefc))} {}
};
} // namespace

PhysicalStep::ProbeResult CheckpointAccess::probe_locked(const NativeRideRuntime& source,
    const RideControl& control,double front,double rear) {
    source.require_open();source.require_boundary_ready();
    // Retain the live arena's poison/callback gate without mutating it. Every
    // potentially mutating solve below belongs to a disposable physical copy.
    source.stepper_->mutate([]{});
    const auto document=bike::initialization::capture_current_state(
        *source.stepper_,*source.physical_,source.config_,source.model_image_);
    auto bootstrap=compose_bootstrap(source.config_,source.model_image_,document);
    const auto* profile=source.physical_->steer_balance_
        ? &source.physical_->steer_balance_->profile():nullptr;
    auto candidate=std::unique_ptr<NativeRideRuntime>(new NativeRideRuntime(
        std::move(bootstrap),NativeRideRuntime::ProbeTag{},profile));
    auto& step=*candidate->physical_;
    // Construction forwards the copied engine. Restore the exact incoming
    // integration/warmstart and the same derived rows as a full checkpoint;
    // the observer must see the accepted sensor stage, not that extra solve.
    engine::set_state(candidate->stepper_->model(),candidate->stepper_->data(),
        candidate->bootstrap_.integration_state.data(),mjSTATE_INTEGRATION);
    const EngineRows from(*source.stepper_->model(),*source.stepper_->data());
    const EngineRows to(*candidate->stepper_->model(),*candidate->stepper_->data());
    for(std::size_t i=0;i<from.rows.size();++i) {
        if(from.rows[i].size()!=to.rows[i].size())
            throw std::invalid_argument("spatial probe: copied engine row shape differs");
        std::ranges::copy(from.rows[i],to.rows[i].begin());
    }
    step.acc_inputs_=source.physical_->acc_inputs_;
    step.acc_inputs_valid_=source.physical_->acc_inputs_valid_;
    step.rider_state_=source.physical_->rider_state_;
    step.spatial_brake_force_=source.physical_->spatial_brake_force_;
    step.spatial_brake_last_=source.physical_->spatial_brake_last_;
    step.imu_truth_=source.physical_->imu_truth_;
    step.spatial_truth_=source.physical_->spatial_truth_;
    return step.probe_step_inputs(control,front,rear);
}

bike::io::Value CheckpointAccess::capture_bootstrap(const NativeRideRuntime& runtime) {
    const AdvancementGuard guard(runtime.advancing_);
    runtime.require_open();runtime.require_boundary_ready();
    if(!runtime.config_.spatial || runtime.accounting_->asynchronous())
        throw std::invalid_argument("spatial checkpoint requires a synchronous spatial runtime");
    const auto& step=*runtime.physical_;
    if(step.step()!=runtime.committed_step_ || runtime.stepper_->time()!=runtime.committed_time_s_)
        throw std::logic_error("checkpoint requires an accepted engine boundary");
    auto document=bike::initialization::capture_current_state(*runtime.stepper_,step,runtime.config_,runtime.model_image_);
    auto state=field(document,"runtime");
    const auto& accounted=runtime.accounting_->state();
    set(state,"history",wire(Wire{accounted.history.as_wire()}));
    set(state,"model_status",wire(Wire{accounted.model_status.state_wire()}));
    set(state,"monitor",wire(Wire{accounted.monitor.as_wire()}));
    Object engine_rows;
    const EngineRows engine(*runtime.stepper_->model(),*runtime.stepper_->data());
    for(std::size_t i=0;i<engine.rows.size();++i)engine_rows.emplace_back(engine.names[i],reals(engine.rows[i]));
    Value imu;
    if(step.imu_truth_)imu=Value{Object{{"time",real(step.imu_truth_->time_s)},
        {"acceleration",reals(step.imu_truth_->specific_force_sensor_mps2)},
        {"gyro",reals(step.imu_truth_->angular_rate_sensor_rad_s)}}};
    const auto& brake=step.spatial_brake_last_;
    Value brake_result{Object{{"changed",Value{brake.frictionloss_changed}},
        {"hold",reals(brake.hold_torque_nm)},{"budget",reals(brake.friction_budget_nm)},
        {"energy",reals(std::array{brake.stored_energy_j,brake.damping_heat_j,brake.release_heat_j})}}};
    Value physical{Object{{"acc_inputs",reals(step.acc_inputs_)},{"acc_valid",Value{step.acc_inputs_valid_}},
        {"imu",std::move(imu)},{"truth",step.spatial_truth_?truth(*step.spatial_truth_):Value{}},
        {"rider_kinematics",rider_kinematics(step.rider_state_)},
        {"presentation",encode_presentation_checkpoint(runtime.committed_presentation_)},
        {"engine_rows",Value{std::move(engine_rows)}},{"brake_force",reals(step.spatial_brake_force_)},
        {"brake_result",std::move(brake_result)}}};
    Value continuation{Object{{"schema",integer(1)},{"runtime_config_sha256",text(config_digest(runtime.config_))},
        {"execution",execution(runtime_identity())},{"accounting",runtime.accounting_->checkpoint_state()},
        {"physical",std::move(physical)},{"finish_position",optional(runtime.finish_position_m_)},
        {"completion_step",runtime.completion_step_?integer(*runtime.completion_step_):Value{}},
        {"pending_warning",runtime.pending_warning_?text(*runtime.pending_warning_):Value{}}}};
    set(state,"continuation",std::move(continuation));set(document,"runtime",std::move(state));
    return document;
}

std::unique_ptr<NativeRideRuntime> CheckpointAccess::prepare(const SpatialCheckpoint& checkpoint,
    const NativeRideRuntime& target) {
    const AdvancementGuard guard(target.advancing_);
    return prepare_locked(checkpoint,target);
}
std::unique_ptr<NativeRideRuntime> CheckpointAccess::prepare_locked(const SpatialCheckpoint& checkpoint,
    const NativeRideRuntime& target) {
    target.require_open();target.require_boundary_ready();
    (void)encode_checkpoint(checkpoint);
    if(!target.config_.spatial || !target.committed_spatial_ || target.config_.async_telemetry)
        throw std::invalid_argument("checkpoint.target: synchronous spatial owners required");
    const auto& identity=checkpoint.identity;
    if(string(field(identity,"model_digest"))!=target.model_image_.sha256 ||
       string(field(identity,"terrain_sha256"))!=target.committed_spatial_->terrain_sha256 ||
       string(field(identity,"calibration_sha256"))!=target.committed_spatial_->calibration_sha256)
        throw std::invalid_argument("checkpoint.identity: target model/terrain/calibration differs");
    if(string(field(identity,"solver"))!=bike::platform::execution_identity() ||
       string(field(identity,"provider"))!=engine::identity().numeric_provider)
        throw std::invalid_argument("checkpoint.identity: incompatible-execution");
    const auto& continuation=field(field(checkpoint.bootstrap2,"runtime"),"continuation");
    exact(continuation,{"schema","runtime_config_sha256","execution","accounting","physical","finish_position","completion_step","pending_warning"});
    if(count(field(continuation,"schema"))!=1 || string(field(continuation,"runtime_config_sha256"))!=config_digest(target.config_))
        throw std::invalid_argument("checkpoint.runtime_config_sha256: incompatible runtime configuration");
    if(bike::io::canonical_json(field(continuation,"execution"))!=bike::io::canonical_json(execution(runtime_identity())))
        throw std::invalid_argument("checkpoint.execution: incompatible-execution");
    auto bootstrap=compose_bootstrap(target.config_,target.model_image_,checkpoint.bootstrap2);
    auto candidate=std::unique_ptr<NativeRideRuntime>(new NativeRideRuntime(std::move(bootstrap),NativeRideRuntime::ContinuationTag{}));
    candidate->accounting_->restore_checkpoint_state(field(continuation,"accounting"));
    const auto& accounting=field(continuation,"accounting");
    const auto committed=candidate->accounting_->state().history.last_id.value_or(-1)+1;
    const auto buffered=bike::io::as_array(field(accounting,"buffered"),"checkpoint.buffered").size();
    if(buffered>static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()) ||
       committed>std::numeric_limits<std::int64_t>::max()-static_cast<std::int64_t>(buffered) ||
       committed+static_cast<std::int64_t>(buffered)!=candidate->committed_step_)
        throw std::invalid_argument("checkpoint.accounting: physical/publication interval inventory differs");
    const auto& physical=field(continuation,"physical");
    exact(physical,{"acc_inputs","acc_valid","imu","truth","rider_kinematics","presentation","engine_rows","brake_force","brake_result"});
    auto& step=*candidate->physical_;
    auto acc_inputs=numbers(field(physical,"acc_inputs"),step.acc_inputs_.size());
    auto rider_state=rider_kinematics(field(physical,"rider_kinematics"),*candidate->stepper_->model(),candidate->committed_time_s_);
    if(rider_state&&!step.rider_present())vread::invalid("checkpoint.rider_kinematics","rider absent from model");
    auto presentation=decode_presentation_checkpoint(field(physical,"presentation"),candidate->committed_presentation_);
    if(presentation.time_s!=candidate->committed_time_s_||!presentation.spatial||
       presentation.spatial->generation!=static_cast<std::uint64_t>(candidate->committed_generation_))
        vread::invalid("checkpoint.presentation","clock/generation differs from plant");
    auto brake_force=numbers(field(physical,"brake_force"),static_cast<std::size_t>(candidate->stepper_->model()->nv));
    const auto& result=field(physical,"brake_result");exact(result,{"changed","hold","budget","energy"});
    SpatialBrakeResult brake;
    brake.frictionloss_changed=boolean(field(result,"changed"));brake.hold_torque_nm=array<2>(field(result,"hold"));
    brake.friction_budget_nm=array<2>(field(result,"budget"));const auto energy=array<3>(field(result,"energy"));
    brake.stored_energy_j=energy[0];brake.damping_heat_j=energy[1];brake.release_heat_j=energy[2];
    std::optional<ImuTruth> imu;
    if(!bike::io::is_null(field(physical,"imu"))) {
        const auto& source=field(physical,"imu");exact(source,{"time","acceleration","gyro"});
        imu=ImuTruth{.time_s=number(field(source,"time")),.specific_force_sensor_mps2=array<3>(field(source,"acceleration")),
            .angular_rate_sensor_rad_s=array<3>(field(source,"gyro"))};
        if(imu->time_s<0. || imu->time_s>candidate->committed_time_s_)vread::invalid("checkpoint.imu.time","outside accepted boundary");
    }
    std::optional<SpatialTruth> spatial_truth;
    if(!bike::io::is_null(field(physical,"truth")))spatial_truth=truth(field(physical,"truth"));
    if(!imu || !spatial_truth)vread::invalid("checkpoint.physical","committed spatial observations are required");
    const EngineRows engine(*candidate->stepper_->model(),*candidate->stepper_->data());
    const auto& saved_engine=field(physical,"engine_rows");
    exact(saved_engine,{"qacc","qacc_smooth","qfrc_constraint","qfrc_smooth","qfrc_inverse","qfrc_actuator",
        "actuator_force","sensordata","cacc","cfrc_int","cfrc_ext","efc_force"});
    std::array<std::vector<double>,12> saved;
    for(std::size_t i=0;i<engine.rows.size();++i)saved[i]=numbers(field(saved_engine,engine.names[i]),engine.rows[i].size());
    candidate->finish_position_m_=optional(field(continuation,"finish_position"));
    candidate->completion_step_.reset();
    if(!bike::io::is_null(field(continuation,"completion_step"))) {
        candidate->completion_step_=count(field(continuation,"completion_step"));
        if(*candidate->completion_step_<0)vread::invalid("checkpoint.completion_step","negative limit");
    }
    candidate->pending_warning_.reset();
    if(!bike::io::is_null(field(continuation,"pending_warning")))candidate->pending_warning_=string(field(continuation,"pending_warning"));
    for(std::size_t i=0;i<engine.rows.size();++i)std::ranges::copy(saved[i],engine.rows[i].begin());
    step.acc_inputs_=std::move(acc_inputs);step.acc_inputs_valid_=boolean(field(physical,"acc_valid"));
    step.rider_state_=std::move(rider_state);
    step.spatial_brake_force_=std::move(brake_force);step.spatial_brake_last_=brake;
    step.imu_truth_=std::move(imu);step.spatial_truth_=std::move(spatial_truth);
    candidate->next_presentation_=std::move(presentation);
    candidate->next_integration_state_=candidate->bootstrap_.integration_state;
    candidate->commit_boundary(candidate->capture_boundary());
    // A continuation is not a new reset origin. Copy the original t=0
    // inventories before publication so reset keeps its established meaning.
    candidate->bootstrap_=target.bootstrap_;candidate->bootstrap_step_state_=target.bootstrap_step_state_;
    candidate->bootstrap_rider_contacts_=target.bootstrap_rider_contacts_;
    candidate->bootstrap_accounting_state_=target.bootstrap_accounting_state_;
    return candidate;
}

void CheckpointAccess::commit(CheckpointStage&& staged,NativeRideRuntime& target) {
    if(staged.consumed)throw std::invalid_argument("checkpoint.stage: already consumed");
    if(bike::io::canonical_json(staged.checkpoint.research)!=
           bike::io::canonical_json(inactive_research_checkpoint()) ||
       bike::io::canonical_json(staged.checkpoint.sensor_v2)!=
           bike::io::canonical_json(inactive_sensor_checkpoint()) ||
       boolean(field(staged.checkpoint.policy,"resumable")))
        throw std::invalid_argument("checkpoint.stage: active research/sensor/policy requires its owning research host");
    const auto identity=runtime_identity();
    if(staged.execution.mujoco_version!=identity.mujoco_version ||
       staged.execution.native_source_sha256!=identity.native_source_sha256 ||
       staged.execution.build_context_json!=identity.build_context_json)
        throw std::invalid_argument("checkpoint.stage: incompatible-execution");
    const AdvancementGuard guard(target.advancing_);
    target.require_open();target.require_boundary_ready();
    auto candidate=prepare_locked(staged.checkpoint,target);
    // Stage and live ownership are closed to concurrent advance by the
    // public guard. Every operation after this boundary is nonthrowing.
    target.stepper_.swap(candidate->stepper_);target.physical_.swap(candidate->physical_);
    target.accounting_.swap(candidate->accounting_);
    target.committed_integration_state_.swap(candidate->committed_integration_state_);
    target.next_integration_state_.swap(candidate->next_integration_state_);
    using std::swap;
    swap(target.committed_presentation_,candidate->committed_presentation_);
    swap(target.next_presentation_,candidate->next_presentation_);
    swap(target.committed_generation_,candidate->committed_generation_);swap(target.committed_step_,candidate->committed_step_);
    swap(target.committed_time_s_,candidate->committed_time_s_);swap(target.committed_position_m_,candidate->committed_position_m_);
    swap(target.committed_battery_energy_j_,candidate->committed_battery_energy_j_);
    target.committed_crash_.swap(candidate->committed_crash_);target.committed_balance_.swap(candidate->committed_balance_);
    target.committed_outcome_.swap(candidate->committed_outcome_);target.committed_spatial_.swap(candidate->committed_spatial_);
    target.committed_tire_state_bytes_.swap(candidate->committed_tire_state_bytes_);
    target.finish_position_m_.swap(candidate->finish_position_m_);target.completion_step_.swap(candidate->completion_step_);
    target.pending_warning_.swap(candidate->pending_warning_);target.test_failure_step_.reset();
    staged.consumed=true;
}
} // namespace runtime::spatial
