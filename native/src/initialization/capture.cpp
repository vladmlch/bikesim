#include "capture.hpp"
#include "../diag.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ranges>
#include <type_traits>

#include "../drivetrain/state_codec.hpp"
#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../model/mapping.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/setup_values.hpp"
#include "../stepper.hpp"

namespace bike::initialization {
namespace {
using namespace runtime::setupvalue;

Value optional(std::optional<double> value) { return value ? real(*value) : Value{}; }

Value wire_value(const runtime::Wire& value) {
    return std::visit([](const auto& x) -> Value {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, std::monostate>) return Value{};
        else if constexpr (std::is_same_v<T, double>) return real(x);
        else if constexpr (std::is_same_v<T, std::int64_t>) return integer(x);
        else if constexpr (std::is_same_v<T, runtime::WireArray>) {
            Array out;
            out.reserve(x.size());
            for (const auto& entry : x) out.push_back(wire_value(entry));
            return Value{std::move(out)};
        } else if constexpr (std::is_same_v<T, runtime::WireObject>) {
            Object out;
            out.reserve(x.size());
            for (const auto& [key, child] : x) out.emplace_back(key, wire_value(child));
            return Value{std::move(out)};
        } else return Value{x};
    }, value.value);
}

Value grounded(const runtime::GroundedFilter::State& s) {
    return Value{Object{{"last_time_s", optional(s.last_time_s)},
        {"last_loaded_s", optional(s.last_loaded_s)}, {"value", Value{s.value}}}};
}
Value bridge(const runtime::BridgedLoad::State& s) {
    return Value{Object{{"held_load_n", real(s.held_load_n)}, {"unloaded_steps", integer(s.unloaded_steps)}}};
}
Value query(const runtime::RuntimeContactQuery::State& s) {
    return Value{Object{{"front_load", bridge(s.front_load)}, {"rear_load", bridge(s.rear_load)},
        {"front_controller_grounded", grounded(s.front_controller)},
        {"rear_controller_grounded", grounded(s.rear_controller)}}};
}
Value balance_event(const std::optional<runtime::BalanceLostEvent>& e) {
    if (!e) return Value{};
    return Value{Object{{"time_s", real(e->time_s)}, {"position_m", real(e->position_m)},
        {"speed_mps", real(e->speed_mps)}}};
}
Value balance(const runtime::BalanceMonitor::State& s) {
    return Value{Object{{"event", balance_event(s.event)}, {"current_event", balance_event(s.current_event)},
        {"low_speed_s", real(s.low_speed_s)}, {"grace_left_s", real(s.grace_left_s)},
        {"started_s", optional(s.started_s)}, {"low_started_s", optional(s.low_started_s)},
        {"last_time_s", optional(s.last_time_s)}}};
}
Value contacts(const runtime::RuntimeContacts& s) {
    return Value{Object{{"front_load_n", real(s.front_load_n)}, {"rear_load_n", real(s.rear_load_n)},
        {"front_support_n", real(s.front_support_n)}, {"rear_support_n", real(s.rear_support_n)},
        {"handlebar_load_n", real(s.handlebar_load_n)},
        {"front_controller_grounded", Value{s.front_controller_grounded.value_or(s.front_in_contact())}},
        {"rear_controller_grounded", Value{s.rear_controller_grounded.value_or(s.rear_in_contact())}},
        {"front_slip_mps", optional(s.front_slip_mps)}, {"rear_slip_mps", optional(s.rear_slip_mps)}}};
}
Value energy(const runtime::EnergyState& e) {
    return Value{Object{{"initial_energy_j", optional(e.initial_energy_j)},
        {"energy_scale_j", real(e.energy_scale_j)}, {"loss_j", real(e.loss_j)},
        {"active_work_j", real(e.active_work_j)}, {"external_work_j", real(e.external_work_j)},
        {"solver_work_j", real(e.solver_work_j)}, {"muscle_signed_j", real(e.muscle_signed_j)},
        {"muscle_positive_j", real(e.muscle_positive_j)}, {"motor_signed_j", real(e.motor_signed_j)},
        {"motor_positive_j", real(e.motor_positive_j)}, {"constraint_absolute_j", real(e.constraint_absolute_j)},
        {"initial_battery_j", optional(e.initial_battery_j)}, {"electrical_work_j", real(e.electrical_work_j)},
        {"mechanical_energy_j", real(e.mechanical_energy_j)},
        {"elastic_energy_j", wire_value(runtime::Wire(e.elastic_energy_j))}, {"residual_j", real(e.residual_j)}}};
}
Value runtime_state(const runtime::StepState& s, const runtime::RuntimeConfig& config) {
    Value held;
    if (s.held_control) {
        Object values;
        for (const auto& [key, value] : *s.held_control) values.emplace_back(key, real(value));
        held = Value{std::move(values)};
    }
    Value terms;
    if (s.held_rider_terms) {
        // Reuse the canonical T08 term codec, including optional diagnostic
        // fields, rather than maintaining a second JointTerms field list.
        ::rider::SpindleController::State state;
        state.last_terms = *s.held_rider_terms;
        const auto encoded = ::rider::encode_spindle_state(state);
        terms = field(encoded, "last_terms");
    }
    Value crash;
    if (s.crash) crash = Value{Object{{"cause", text(s.crash->cause)},
        {"time_s", real(s.crash->time_s)}, {"position_m", real(s.crash->position_m)},
        {"pitch_rad", real(s.crash->pitch_rad)}}};
    runtime::ReferenceMonitor monitor;
    monitor.strict = config.strict;
    return Value{Object{{"step", integer(s.step)}, {"generation", integer(s.generation)},
        {"measurement_decimation", integer(s.measurement_decimation)},
        {"applied_control", ::rider::encode_intent_control(s.applied_control)},
        {"held_control", std::move(held)}, {"held_rider_terms", std::move(terms)},
        {"rollback_hold", Value{s.rollback_hold}},
        {"research_accounting_valid", Value{s.research_accounting_valid}},
        {"initializing", Value{s.initializing}},
        {"filters", Value{Object{{"front", grounded(s.filters[0])}, {"rear", grounded(s.filters[1])}}}},
        {"balance", balance(s.balance)}, {"crash", Value{Object{{"event", std::move(crash)}}}},
        {"contacts", contacts(s.contacts)}, {"contact_query", query(s.contact_query)},
        {"probe_query", query(s.probe_query)}, {"energy", energy(s.energy)},
        {"history", wire_value(runtime::Wire(runtime::WorkHistory{}.as_wire()))},
        {"model_status", wire_value(runtime::Wire(runtime::ModelStatus{}.state_wire()))},
        {"monitor", wire_value(runtime::Wire(monitor.as_wire()))}}};
}

} // namespace

runtime::ModelImage capture_model_image(const mjModel& model) {
    struct SizeCall { const mjModel* model{}; mjtSize bytes = 0; } size{.model=&model};
    engine::ErrorBuffer error;
    if (!engine::invoke([](void* raw) noexcept {
        auto& call = *static_cast<SizeCall*>(raw);
        call.bytes = mj_sizeModel(call.model);
    }, &size, error)) engine::throw_failure(error);
    if (size.bytes <= 0 || std::cmp_greater(size.bytes, 512U * 1024U * 1024U))
        throw std::invalid_argument("compiled model image exceeds the 512 MiB setup limit");
    runtime::ModelImage out;
    out.mjb.resize(static_cast<std::size_t>(size.bytes));
    struct SaveCall { const mjModel* model; void* buffer; int bytes; }
        save{.model=&model, .buffer=out.mjb.data(), .bytes=static_cast<int>(size.bytes)};
    if (!engine::invoke([](void* raw) noexcept {
        auto const& call = *static_cast<SaveCall*>(raw);
        mj_saveModel(call.model, nullptr, call.buffer, call.bytes);
    }, &save, error)) engine::throw_failure(error);
    out.sha256 = io::sha256(out.mjb);
    return out;
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
runtime::RuntimeBootstrap capture_initialized(Stepper const& stepper,
    runtime::PhysicalStep const& physical, const runtime::RuntimeConfig& config) {
    const auto state_ptr = std::make_unique<runtime::StepState>(physical.state());
    const auto& state = *state_ptr;
    if (state.step != 0 || stepper.time() != 0.)
        throw std::invalid_argument("bootstrap capture requires t=0");
    if (state.initializing || !state.research_accounting_valid || !state.energy.initial_energy_j)
        throw std::invalid_argument("bootstrap capture requires a completed physical reset");
    const auto& m = *stepper.model();
    const auto& d = *stepper.data();
    auto image = capture_model_image(m);
    const auto size = engine::state_size(&m, mjSTATE_INTEGRATION);
    if (size < 0) throw std::invalid_argument("invalid integration-state size");
    std::vector<double> integration(static_cast<std::size_t>(size));
    engine::get_state(&m, &d, integration.data(), mjSTATE_INTEGRATION);
    if (!std::ranges::all_of(integration, [](double x) { return std::isfinite(x); }))
        throw std::invalid_argument("bootstrap integration state contains non-finite values");
    Value dims{Object{{"nq", integer(m.nq)}, {"nv", integer(m.nv)}, {"nu", integer(m.nu)},
        {"na", integer(m.na)}, {"nbody", integer(m.nbody)}, {"njnt", integer(m.njnt)},
        {"ntendon", integer(m.ntendon)}, {"nwrap", integer(m.nwrap)}, {"nsite", integer(m.nsite)},
        {"ngeom", integer(m.ngeom)}, {"neq", integer(m.neq)}, {"nsensor", integer(m.nsensor)},
        {"nsensordata", integer(m.nsensordata)}}};
    Object friction;
    for (auto side : {"front", "rear"}) {
        const std::string joint = std::string(side) + "_wheel_spin";
        const auto address = model::joint_address(m, joint).second;
        friction.emplace_back(side, Value{Object{{"joint", text(joint)}, {"dof_index", integer(0)},
            {"value", real(std::views::counted(m.dof_frictionloss, m.nv)[static_cast<std::size_t>(address)])}}});
    }
    Object sites;
    if (config.geometry.com_marker_site) {
        const auto id = model::required_id(m, mjOBJ_SITE, *config.geometry.com_marker_site);
        const auto positions = std::views::counted(m.site_pos, m.nsite * 3);
        sites.emplace_back(*config.geometry.com_marker_site,
            reals(positions.subspan(static_cast<std::size_t>(id) * 3, 3)));
    }
    // native_reference builds no tire writer; its bootstrap section is the
    // empty names/row pair the decoder accepts (refine.cpp uses the same
    // convention) rather than the writer schema names.
    Array tire_names;
    Array tire_row;
    if (stepper.has_tire()) {
        for (const auto& name : stepper.tire_state_names()) tire_names.push_back(text(name));
        const auto row = stepper.tire_state();
        tire_row.reserve(row.size());
        for (const double value : row) tire_row.push_back(real(value));
    }
    Value contacts_state;
    if (stepper.has_rider_contacts()) contacts_state = ::rider::encode_rider_contacts_state(
        stepper.rider_contacts().state(), stepper.rider_contacts().probe());
    Object doc_obj;
    doc_obj.emplace_back("schema", integer(runtime::kRuntimeSchema));
    doc_obj.emplace_back("model_digest", text(image.sha256));
    doc_obj.emplace_back("model_dims", std::move(dims));
    doc_obj.emplace_back("integration_state", reals(integration));
    doc_obj.emplace_back("model_mutable", Value{Object{{"dof_frictionloss", Value{std::move(friction)}},
        {"site_pos", Value{std::move(sites)}}}});
    doc_obj.emplace_back("tire", Value{Object{{"names", Value{std::move(tire_names)}}, {"row", Value{std::move(tire_row)}}}});
    doc_obj.emplace_back("drive", drivetrain::encode_drive_snapshot(stepper.drive().state()));
    doc_obj.emplace_back("rider_contacts", std::move(contacts_state));
    doc_obj.emplace_back("rider_controller", state.rider_controller ? ::rider::encode_spindle_state(*state.rider_controller) : Value{});
    doc_obj.emplace_back("rider_intent", ::rider::encode_intent_state(state.rider_intent));
    doc_obj.emplace_back("signals", ::rider::encode_intent_signals(state.signals));
    doc_obj.emplace_back("runtime", runtime_state(state, config));
    Value const document{std::move(doc_obj)};
    // Decode into the exact same owned state used by the captured-file path.
    return runtime::compose_bootstrap(config, std::move(image), document);
}
NATIVE_DIAG_POP

} // namespace bike::initialization
