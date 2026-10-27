#pragma once

#include <algorithm>

// Private test assembly, not a frozen production API. These helpers own the
// real SetupOwner and runtime, including material and causal state inventories.
#include "startup_fixture.hpp"
#if __has_include("fixture_paths.hpp")
#include "fixture_paths.hpp"
#else
// fixture_paths.hpp is configure_file output that only exists inside the
// spatial test graph; standalone header sweeps compile under foreign target
// contexts where it is absent. The fallback only needs to keep this header
// parseable — every consumer that runs these functions has the generated
// values.
#define BIKE_SPATIAL_FIXTURE_ROOT "."
#define BIKE_SPATIAL_SOURCE_ROOT "."
#endif
#include "terrain_fixture.hpp"
#include "support/migration_test.hpp"
#include "initialization/setup_owner.hpp"
#include "io/json.hpp"
#include "model_access.hpp"
#include "drivetrain/state_codec.hpp"
#include "pneumatic3d/state.hpp"
#include "rider/state_codec.hpp"
#include "research/evidence.hpp"
#include "runtime/runtime.hpp"
#include "runtime/setup_values.hpp"
#include "terrain/source3d.hpp"
#include "validation/runtime_access.hpp"

#include <filesystem>
#include <memory>
#include <span>
#include <utility>

namespace spatial_tests {
inline std::unique_ptr<bike::initialization::SetupRequest> make_startup_request(
        bike::spatial::RootKind root,bike::spatial::TyreTier tier,bool articulated,double speed,
        std::shared_ptr<const bike::terrain::Terrain3dSource> terrain={}) {
    auto request=std::make_unique<bike::initialization::SetupRequest>();
    request->asset_root=std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT);
    request->origin="spatial-startup-authored-fixture";request->seed_decimal="73421";request->start_x_m=3.;
    request->track.name="spatial-flat-arena";request->track.length_m=200.;
    request->model.mode="ride";request->model.crank_joint=true;
    request->model.rider.emplace();request->model.rider->variant=articulated?"articulated_planar":"none";
    request->model.physics.emplace();auto& physics=*request->model.physics;
    physics.physics_mode="physical";physics.drive_mode=articulated?"articulated_effort":"crank_effort";
    physics.tires.backend="pneumatic_3d";physics.tires.surface_mode="track";
    physics.timestep_s=tier==bike::spatial::TyreTier::fast?.00125:.0005;
    // configuration/validate.cpp requires closure_time_constant_s >= 2*timestep_s for non-legacy physics.
    physics.closure_time_constant_s=std::max(physics.closure_time_constant_s,2.*physics.timestep_s);
    physics.control_period_s=.005;physics.initial_speed_mps=speed;
    physics.initial_front_brake=physics.initial_rear_brake=0.;
    physics.drive.transmission_model="ideal_mid_drive";physics.drive.shifting.enabled=false;
    physics.drive.human_torque_nm=0.;physics.drive.assist.gain=0.;
    physics.resistance.crr=0.;physics.resistance.cda_m2=0.;
    physics.pitch_assist=false;physics.equilibrium_cache_enabled=false;physics.equilibrium_state_file.reset();
    physics.spatial.emplace();auto& cfg=*physics.spatial;
    cfg.mode=bike::spatial::Mode::spatial3d;cfg.root=root;cfg.tyre.tier=tier;
    cfg.steer.controller=root==bike::spatial::RootKind::free6?bike::spatial::SteerController::roll_heading:
        bike::spatial::SteerController::off;
    cfg.steer.profile_id="roll-heading-v1";cfg.steer.torque_cap_nm=40.;
    cfg.corridor_half_width_m=1.5;
    if(articulated) {
        physics.articulated.pedal_attachment="spindle";physics.articulated.saddle_attachment="pin";
        physics.articulated.grip_attachment="connect";
        physics.articulated.shoulder_profile=(request->asset_root/"examples/research/profiles/spatial-shoulder.json").string();
    }
    request->model.spatial=cfg;
    request->model.terrain3d=terrain?std::move(terrain):
        std::make_shared<const bike::terrain::Terrain3dSource>(plane_grid(0.,0.,0.,401,25,.5,.5));
    request->model.field=bike::terrain::HeightFieldSpec::for_track(request->track);
    request->model.field.nrow=17;request->model.field.ncol=257;
    request->runtime.record_retention=runtime::RetentionMode::full;
    request->runtime.measurement_decimation=1;request->runtime.async_telemetry=false;
    request->runtime.speed_kph=3.6*speed;request->equilibrium.tolerance=.05;
    return request;
}
struct OwnedStartup {
    std::unique_ptr<bike::initialization::SetupOwner> owner;
    bike::initialization::EquilibriumResult equilibrium;
    std::unique_ptr<runtime::RuntimeBootstrap> bootstrap;
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    explicit OwnedStartup(bike::initialization::SetupRequest request) {
        const auto options=request.equilibrium;
        owner=bike::initialization::make_setup(std::move(request));
        equilibrium=bike::initialization::settle(*owner,options);
        bootstrap=std::make_unique<::runtime::RuntimeBootstrap>(owner->seal_bootstrap());
        runtime=std::make_unique<::runtime::NativeRideRuntime>(*bootstrap);
    }
};
inline std::vector<std::byte> physics_state_bytes(const runtime::RuntimeSnapshot& s) {
    const auto numeric=std::as_bytes(std::span{s.integration_state});
    std::vector<std::byte> bytes(numeric.begin(),numeric.end());
    bytes.insert(bytes.end(),s.tire_state_bytes.begin(),s.tire_state_bytes.end());
    if(s.spatial) {
        const auto text=bike::io::canonical_json(runtime::encode_spatial_bootstrap(*s.spatial));
        const auto causal=std::as_bytes(std::span{text});bytes.insert(bytes.end(),causal.begin(),causal.end());
    }
    return bytes;
}
inline std::vector<std::byte> complete_physics_state_bytes(runtime::NativeRideRuntime& ride) {
    using namespace runtime::setupvalue;
    auto bytes=physics_state_bytes(ride.snapshot());
    const auto append=[&](std::span<const std::byte> values) {
        const std::uint64_t count=values.size();
        const auto header=std::as_bytes(std::span{&count,std::size_t{1}});
        bytes.insert(bytes.end(),header.begin(),header.end());
        bytes.insert(bytes.end(),values.begin(),values.end());
    };
    const auto document=[&](const bike::io::Value& value) {
        const auto text=bike::io::recording_json(value);append(std::as_bytes(std::span{text}));
    };
    document(bike::research::wire_value(runtime::Wire(ride.accounting_state())));
    bike::validation::RuntimeRigAccess::inspect(ride,[&](const Stepper& stepper,const runtime::PhysicalStep& physical) {
        const auto* m=stepper.model();
        const auto row=[&](const double* p,mjtSize n){append(std::as_bytes(model_access::readonly_buffer(p,n)));};
        row(m->dof_frictionloss,m->nv);row(m->site_pos,3*m->nsite);
        row(m->tendon_stiffness,m->ntendon);row(m->tendon_damping,m->ntendon);
        row(m->tendon_lengthspring,2*m->ntendon);row(m->tendon_range,2*m->ntendon);row(m->wrap_prm,m->nwrap);
        document(drivetrain::encode_drive_snapshot(stepper.drive().state()));
        if(stepper.has_rider_contacts())document(::rider::encode_rider_contacts_state(
            stepper.rider_contacts().state(),stepper.rider_contacts().probe()));
        // Direct initialization elides the large StepState return into its
        // cold heap owner instead of materializing it in make_unique's caller.
        const std::unique_ptr<runtime::StepState> s{new runtime::StepState(physical.state())};
        if(s->rider_controller)document(::rider::encode_spindle_state(*s->rider_controller));
        document(::rider::encode_intent_state(s->rider_intent));
        document(::rider::encode_intent_signals(s->signals));
        document(::rider::encode_intent_control(s->applied_control));
        Value held;
        if(s->held_control){Object values;for(const auto& [key,value]:*s->held_control)values.emplace_back(key,real(value));held=Value{std::move(values)};}
        document(held);
        Value terms;
        if(s->held_rider_terms){auto controller=std::make_unique<::rider::SpindleController::State>();controller->last_terms=*s->held_rider_terms;
            const auto encoded=::rider::encode_spindle_state(*controller);terms=field(encoded,"last_terms");}
        document(terms);
        const auto optional=[](std::optional<double> x){return x?real(*x):Value{};};
        const auto filter=[&](const runtime::GroundedFilter::State& f){return Value{Object{
            {"last_time",optional(f.last_time_s)},{"last_loaded",optional(f.last_loaded_s)},{"value",Value{f.value}}}};};
        const auto query=[&](const runtime::RuntimeContactQuery::State& q){return Value{Object{
            {"front_load",real(q.front_load.held_load_n)},{"rear_load",real(q.rear_load.held_load_n)},
            {"front_unloaded",integer(q.front_load.unloaded_steps)},{"rear_unloaded",integer(q.rear_load.unloaded_steps)},
            {"front_filter",filter(q.front_controller)},{"rear_filter",filter(q.rear_controller)}}};};
        const auto event=[&](const std::optional<runtime::BalanceLostEvent>& e){return e?Value{Object{
            {"time",real(e->time_s)},{"position",real(e->position_m)},{"speed",real(e->speed_mps)}}}:Value{};};
        const auto& c=s->contacts;const auto& b=s->balance;const auto& e=s->energy;
        document(Value{Object{{"step",integer(s->step)},{"generation",integer(s->generation)},
            {"decimation",integer(s->measurement_decimation)},{"rollback",Value{s->rollback_hold}},
            {"research_valid",Value{s->research_accounting_valid}},{"initializing",Value{s->initializing}},
            {"front_filter",filter(s->filters[0])},{"rear_filter",filter(s->filters[1])},
            {"contact_query",query(s->contact_query)},{"probe_query",query(s->probe_query)},
            {"balance",Value{Object{{"event",event(b.event)},{"current",event(b.current_event)},
                {"low",real(b.low_speed_s)},{"grace",real(b.grace_left_s)},{"started",optional(b.started_s)},
                {"low_started",optional(b.low_started_s)},{"last",optional(b.last_time_s)}}}},
            {"contacts",Value{Object{{"front_load",real(c.front_load_n)},{"rear_load",real(c.rear_load_n)},
                {"front_support",real(c.front_support_n)},{"rear_support",real(c.rear_support_n)},
                {"handlebar",real(c.handlebar_load_n)},
                {"front_grounded",c.front_controller_grounded?Value{*c.front_controller_grounded}:Value{}},
                {"rear_grounded",c.rear_controller_grounded?Value{*c.rear_controller_grounded}:Value{}},
                {"front_slip",optional(c.front_slip_mps)},{"rear_slip",optional(c.rear_slip_mps)}}}},
            {"energy",Value{Object{{"initial",optional(e.initial_energy_j)},{"scale",real(e.energy_scale_j)},
                {"loss",real(e.loss_j)},{"active",real(e.active_work_j)},{"external",real(e.external_work_j)},
                {"solver",real(e.solver_work_j)},{"muscle_signed",real(e.muscle_signed_j)},
                {"muscle_positive",real(e.muscle_positive_j)},{"motor_signed",real(e.motor_signed_j)},
                {"motor_positive",real(e.motor_positive_j)},{"constraint",real(e.constraint_absolute_j)},
                {"battery",optional(e.initial_battery_j)},{"electrical",real(e.electrical_work_j)},
                {"mechanical",real(e.mechanical_energy_j)},{"residual",real(e.residual_j)},
                {"elastic",bike::research::wire_value(runtime::Wire(e.elastic_energy_j))}}}}}});
        if(s->crash)document(Value{Object{{"cause",text(s->crash->cause)},{"time",real(s->crash->time_s)},
            {"position",real(s->crash->position_m)},{"pitch",real(s->crash->pitch_rad)}}});
    });
    // This same-binary comparison contains no origin/cache/source metadata.
    // Every physics field above remains bitwise or losslessly encoded.
    return bytes;
}
} // namespace spatial_tests
