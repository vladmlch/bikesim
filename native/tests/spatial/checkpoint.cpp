// W42 authored continuation controls; execution/qualification belongs to V07.
#include "support/migration_test.hpp"
#include "spatial/support/startup_cases.hpp"
#include "spatial/support/ride_qualification.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/research_io.hpp"
#include "runtime/spatial_checkpoint.hpp"
#include "runtime/spatial_checkpoint_internal.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"
#include "pneumatic3d/state.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace {
using namespace runtime::setupvalue;
namespace io=bike::io;
std::string bytes(const Value& value) { return io::canonical_json(value); }
std::string wire_bytes(const runtime::Wire& value) { return bytes(bike::research::wire_value(value)); }

void same_snapshot(const runtime::RuntimeSnapshot& a,const runtime::RuntimeSnapshot& b) {
    bt::require(a.step==b.step&&a.time_s==b.time_s&&a.generation==b.generation&&a.outcome==b.outcome,
        "restored plant publishes the same committed clock and outcome");
    bt::require(spatial_tests::physics_state_bytes(a)==spatial_tests::physics_state_bytes(b),
        "integration, material, brake, steer, hold and outcome state are bitwise equal");
    bt::require(wire_bytes(a.view)==wire_bytes(b.view)&&wire_bytes(a.model_status)==wire_bytes(b.model_status),
        "render/accounting observations are identical without advancing the plant");
    bt::require(static_cast<bool>(a.latest_sample)==static_cast<bool>(b.latest_sample),"same latest publication presence");
    if(a.latest_sample)bt::require(wire_bytes(a.latest_sample->as_wire())==wire_bytes(b.latest_sample->as_wire()),
        "each accepted measurement row is bitwise equal after restore");
}

void cursor_rejection() {
    runtime::spatial::NoiseTapeV2 tape;tape.noise.resize(2);tape.dropout_uniform.assign(2,.5);
    runtime::spatial::SensorPipelineV2 pipeline({},std::move(tape));
    pipeline.reset(runtime::spatial::SensorObservationV2{});
    const auto before=pipeline.state();auto invalid=before;++invalid.cursor;
    const auto saved=wire_bytes(pipeline.state_wire());
    bt::invalid([&]{pipeline.import_state(invalid);});
    bt::require(pipeline.state().cursor==before.cursor&&wire_bytes(pipeline.state_wire())==saved,
        "an internally inconsistent in-range cursor cannot consume sensor state");
}

void turned_rider_preview() {
    auto request=std::make_unique<bike::research::ResearchRequest>();
    request->setup=std::move(*spatial_tests::make_startup_request(
        bike::spatial::RootKind::free6,bike::spatial::TyreTier::fast,true,0.));
    request->capture_mode=runtime::ResearchCaptureMode::full;
    request->experiment.duration_s=.1;request->experiment.control_period_s=.01;
    auto prepared=bike::research::prepare_environment(std::move(request));
    for(const double yaw:{std::numbers::pi/2.,std::numbers::pi}) {
        auto ride=std::make_unique<runtime::NativeRideRuntime>(prepared->bootstrap.ride);
        bike::validation::SpatialRideAccess::perturb(*ride,{0.,0.,0.},{0.,0.,yaw});
        (void)ride->advance(1,runtime::RideControl{},0.,0.,std::nullopt);
        const auto checkpoint=runtime::spatial::capture_checkpoint(*ride,
            runtime::spatial::inactive_research_checkpoint(),runtime::spatial::inactive_sensor_checkpoint(),
            Value{Object{{"resumable",Value{false}},{"state",Value{}}}},
            runtime::spatial::checkpoint_identity(prepared->metadata));
        const auto& physical=field(field(field(checkpoint.bootstrap2,"runtime"),"continuation"),"physical");
        const auto& road=io::as_array(field(field(physical,"rider_kinematics"),"road"),"road");
        bool nonincreasing_x=false;
        for(std::size_t i=1;i<road.size();++i)
            nonincreasing_x=nonincreasing_x ||
                io::as_double(io::as_array(road[i],"point")[0],"x")<=
                io::as_double(io::as_array(road[i-1],"point")[0],"x");
        bt::require(road.size()>2&&nonincreasing_x,
            "real turned-rider preview is ordered by heading, not increasing world X");
        auto restored=runtime::spatial::CheckpointAccess::prepare(checkpoint,*ride);
        bt::require(bytes(runtime::spatial::CheckpointAccess::capture_bootstrap(*restored))==
                    bytes(checkpoint.bootstrap2),
            "sideways and reversed rider previews survive complete checkpoint restoration");
    }
}

void accounting_power_order() {
    runtime::AccountingConfig config;
    config.timestep_s=.001;config.period_steps=2;config.retention=runtime::RetentionMode::full;
    config.max_qpos=1;config.max_dofs=1;
    auto initial=std::make_unique<runtime::AccountingState>();
    initial->energy.initial_energy_j=100.;initial->energy.energy_scale_j=100.;
    initial->energy.initial_battery_j=0.;initial->energy.mechanical_energy_j=100.;
    auto continuous=std::make_unique<runtime::PeriodAccounting>(config,*initial,1,nullptr,nullptr);
    auto restored=std::make_unique<runtime::PeriodAccounting>(config,*initial,1,nullptr,nullptr);
    auto raw=std::make_unique<runtime::RawStep>();
    raw->interval_id=0;raw->time_s=0.;raw->end_time_s=.001;
    raw->qpos={0.};raw->qvel={1.};raw->qfrc_passive={0.};raw->electrical_power_w=0.;
    raw->numerical_constraint_power_w={{"equality:0",1e16},{"equality:2",-1e16},{"equality:10",1.}};
    rider::AttachmentRaw attachment;
    attachment.rider_jac={.rows=6,.columns=1,.values=std::vector<double>(6,0.)};
    attachment.bike_jac=attachment.rider_jac;attachment.rider_qfrc={0.};attachment.bike_qfrc={0.};
    attachment.normal={0.,0.,1.};attachment.kind="foot";
    raw->attachment_raw={{"saddle",attachment},{"foot_front",attachment}};
    raw->constraint_snapshot={.interval_start_s=0.,.interval_end_s=.001,.qvel_start=raw->qvel,.components={}};
    const runtime::WireObject tire{{"unloaded_radius_m",.35},{"normal_load_n",0.},
        {"penetration_m",0.},{"patches",runtime::WireArray{}}};
    raw->channels={{"tires",runtime::WireObject{{"front",tire},{"rear",tire}}},
        {"suspension",runtime::WireObject{{"linkage_closure_max_m",0.}}},
        {"mass",runtime::WireObject{{"mass_kg",120.},{"com_m",runtime::WireArray{0.,0.,1.}},
            {"kinetic_energy_j",0.},{"gravitational_energy_j",0.}}},
        {"energy",runtime::WireObject{{"mechanical_energy_j",100.},{"residual_j",0.}}}};
    raw->diagnostics={{"rider",runtime::WireObject{}},{"rider_welds",runtime::WireObject{}},
        {"endpoint_mass",runtime::Wire{}},{"rider_intent",runtime::Wire{}},{"inclination_rad",0.},
        {"rider_allocation",runtime::WireObject{}},{"rider_ik_saturation",runtime::WireObject{}},
        {"rider_support_targets",runtime::WireObject{}}};
    continuous->push(std::move(*raw));
    const auto saved=io::parse_json(io::canonical_json(continuous->checkpoint_state()));
    restored->restore_checkpoint_state(saved);
    continuous->flush();restored->flush();
    bt::require(continuous->state().energy.solver_work_j==.001,
        "manufactured first-appearance fold preserves the small constraint contribution");
    bt::require(std::bit_cast<std::uint64_t>(restored->state().energy.solver_work_j)==
        std::bit_cast<std::uint64_t>(continuous->state().energy.solver_work_j),
        "canonical checkpoint roundtrip preserves nonassociative constraint-work reduction");
    bt::require(restored->state().period_violations.at(0).violations==continuous->state().period_violations.at(0).violations &&
        restored->state().period_violations.at(0).violations.at(0)=="saddle:unobservable_attachment_wrench",
        "canonical checkpoint roundtrip preserves attachment failure order");

    auto legacy=saved;auto buffered=std::get<Array>(field(legacy,"buffered").data);
    for(const auto key:{"constraint_power","attachments"}) {
        Object old_entries;
        for(const auto& row:io::as_array(field(buffered.at(0),key),key))
            old_entries.emplace_back(io::as_string(field(row,"name"),"name"),field(row,"value"));
        set(buffered.at(0),key,Value{std::move(old_entries)});
    }
    set(legacy,"buffered",Value{std::move(buffered)});
    auto legacy_owner=std::make_unique<runtime::PeriodAccounting>(config,*initial,1,nullptr,nullptr);
    legacy_owner->restore_checkpoint_state(legacy);legacy_owner->flush();
    bt::require(legacy_owner->state().energy.solver_work_j==.001,
        "earlier checkpoint1 object shape remains readable in its stored order");
}

std::unique_ptr<runtime::ResearchBootstrap> continuation_fixture() {
    auto request=std::make_unique<bike::research::ResearchRequest>();
    request->setup=std::move(*spatial_tests::make_startup_request(
        bike::spatial::RootKind::free6,bike::spatial::TyreTier::detailed,false,0.));
    request->capture_mode=runtime::ResearchCaptureMode::full;
    request->experiment.duration_s=1.;request->experiment.control_period_s=.01;
    request->experiment.actuator_delay_s=.02;request->experiment.record_decimation=1;
    request->experiment.seed_decimal="0";
    request->sensors_v2.latency_s=.02;request->sensors_v2.sample_period_s=.005;
    request->sensors_v2.maximum_read_period_s=.01;
    auto prepared=bike::research::prepare_environment(std::move(request));
    auto bootstrap=std::make_unique<runtime::ResearchBootstrap>(std::move(prepared->bootstrap));
    bt::require(bootstrap->ride.state.tire3d.has_value(),"continuation fixture owns spatial material state");
    // A small explicitly manufactured lateral material prestrain exercises
    // causal history, not a physical reference or a generated golden. The
    // real model steps this state before the checkpoint is captured.
    for(auto& brush:bootstrap->ride.state.tire3d->brush) {
        std::ranges::fill(brush.bristle_uy_m,1e-6);
        std::ranges::fill(brush.bristle_energy_j,1e-12);
        brush.elastic_energy_j=1e-12*static_cast<double>(brush.bristle_energy_j.size());
    }
    (void)bike::pneumatic3d::encode_tyre_state(*bootstrap->ride.state.tire3d);
    return bootstrap;
}

template<class Mutate> void reject_without_change(runtime::NativeResearchRuntime& target,
    const Value& good,Mutate mutate,const std::string& label) {
    const auto before=bytes(target.checkpoint());auto bad=good;mutate(bad);
    bt::invalid([&]{target.restore_checkpoint(bad);});
    bt::require(bytes(target.checkpoint())==before,label+" rejects before changing any causal owner");
}

void continuation() {
    auto bootstrap=continuation_fixture();
    auto copy=std::make_unique<runtime::ResearchBootstrap>(*bootstrap);
    auto continuous=runtime::make_research(std::move(bootstrap));
    auto resumed=runtime::make_research(std::move(copy));
    const auto initial=std::make_unique<runtime::RuntimeSnapshot>(continuous->snapshot());
    runtime::RideControl control;control.human_torque_nm=1.;
    continuous->begin_control(control,.25,.35);
    const auto warmup=continuous->advance_control(std::nullopt);
    bt::require(warmup&&!warmup->terminated&&!warmup->truncated,"first accounting period publishes delayed measurements");
    continuous->acknowledge_control();continuous->begin_control(control,.25,.35);
    const auto partial=continuous->advance_control(std::nullopt,35);
    bt::require(!partial&&continuous->snapshot().step==35,"checkpoint interrupts a live control window");
    const auto checkpoint=continuous->checkpoint();
    const auto& research=field(checkpoint,"research");
    bt::require(!io::is_null(field(research,"window"))&&
        !io::as_array(field(research,"pending_commands"),"pending_commands").empty(),
        "fixture has an active window and delayed actuator commands");
    const auto sensor=runtime::decode_sensor_state_v2(field(field(checkpoint,"sensor_v2"),"state"));
    bt::require(sensor.queue.size()>1,"fixture has undelivered sensor samples");
    const auto& plant=field(checkpoint,"bootstrap2");
    const auto spatial=runtime::decode_spatial_bootstrap(field(plant,"spatial"));
    bt::require(spatial.brake.wheel[0].engaged||spatial.brake.wheel[1].engaged,
        "fixture checkpoint contains an active brake anchor");
    const auto& tire=field(plant,"tire");
    const auto material_bytes=bike::pneumatic3d::base64_decode_strict(io::as_string(field(tire,"payload"),"payload"));
    bt::require(material_bytes.has_value(),"checkpoint material bytes decode");
    const auto material=bike::pneumatic3d::decode_tyre_state(*material_bytes,
        bike::pneumatic3d::tyre_identity_from_wire(field(tire,"identity")));
    bt::require(material.has_value(),"checkpoint material identity matches");
    bool lateral=false;for(const auto& brush:material->brush)
        lateral=lateral||std::ranges::any_of(brush.bristle_uy_m,[](double value){return value!=0.;});
    bt::require(lateral,"fixture checkpoint retains nonzero lateral bristles");

    const auto serialized=io::parse_recording_json(io::recording_json(checkpoint));
    resumed->restore_checkpoint(serialized);
    bt::require(bytes(resumed->checkpoint())==bytes(checkpoint),"cold restore retains every serialized semantic field");
    same_snapshot(continuous->snapshot(),resumed->snapshot());
    const auto restored=resumed->snapshot();
    bt::require(restored.spatial && restored.spatial->brake.interval_index==restored.step-1,
        "restored brake clock carries the last accepted interval");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto identity=field(bad,"identity");set(identity,"terrain_sha256",text(std::string(64,'a')));set(bad,"identity",std::move(identity));
    },"terrain identity");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto identity=field(bad,"identity");set(identity,"solver",text("incompatible-solver"));set(bad,"identity",std::move(identity));
    },"solver identity");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto identity=field(bad,"identity");set(identity,"provider",text("incompatible-provider"));set(bad,"identity",std::move(identity));
    },"numeric provider identity");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto plant=field(bad,"bootstrap2");auto spatial=field(plant,"spatial");
        set(spatial,"topology_fingerprint",text(std::string(64,'b')));set(plant,"spatial",std::move(spatial));set(bad,"bootstrap2",std::move(plant));
    },"topology identity");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto plant=field(bad,"bootstrap2");auto spatial=field(plant,"spatial");auto brake=field(spatial,"brake");
        set(brake,"interval_index",integer(io::as_int64(field(brake,"interval_index"),"interval_index")+1));
        set(spatial,"brake",std::move(brake));set(plant,"spatial",std::move(spatial));set(bad,"bootstrap2",std::move(plant));
    },"scored brake interval index");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto sensor=field(bad,"sensor_v2");auto state=field(sensor,"state");
        set(state,"cursor",integer(io::as_int64(field(state,"cursor"),"cursor")+1));
        set(sensor,"state",std::move(state));set(bad,"sensor_v2",std::move(sensor));
    },"late sensor cursor");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto plant=field(bad,"bootstrap2");auto runtime=field(plant,"runtime");auto continuation=field(runtime,"continuation");
        set(continuation,"pending_warning",integer(7));set(runtime,"continuation",std::move(continuation));
        set(plant,"runtime",std::move(runtime));set(bad,"bootstrap2",std::move(plant));
    },"last continuation field");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto plant=field(bad,"bootstrap2");auto runtime=field(plant,"runtime");auto continuation=field(runtime,"continuation");
        auto accounting=field(continuation,"accounting");auto buffered=std::get<Array>(field(accounting,"buffered").data);
        auto constraints=field(buffered.at(0),"constraints");auto velocity=std::get<Array>(field(constraints,"qvel").data);
        velocity.at(0)=real(io::as_double(velocity.at(0),"qvel")+1.);set(constraints,"qvel",Value{std::move(velocity)});
        set(buffered.at(0),"constraints",constraints);set(accounting,"buffered",Value{std::move(buffered)});
        set(continuation,"accounting",accounting);set(runtime,"continuation",continuation);set(plant,"runtime",runtime);set(bad,"bootstrap2",plant);
    },"buffered constraint velocity");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto plant=field(bad,"bootstrap2");auto runtime=field(plant,"runtime");auto continuation=field(runtime,"continuation");
        auto accounting=field(continuation,"accounting");auto buffered=std::get<Array>(field(accounting,"buffered").data);
        bt::require(io::as_bool(field(buffered.at(0),"full"),"full"),"continuation fixture buffers measured intervals");
        set(buffered.at(0),"elastic",Value{});set(accounting,"buffered",Value{std::move(buffered)});
        set(continuation,"accounting",accounting);set(runtime,"continuation",continuation);set(plant,"runtime",runtime);set(bad,"bootstrap2",plant);
    },"missing buffered measurement owner");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        auto& values=std::get<Object>(bad.data);std::erase_if(values,[](const auto& entry){return entry.first=="policy";});
    },"missing owner");
    reject_without_change(*resumed,checkpoint,[](Value& bad) {
        std::get<Object>(bad.data).emplace_back("sensor_v2",field(bad,"sensor_v2"));
    },"duplicate owner");

    for(std::int64_t next=36;next<=535;++next) {
        const auto a=continuous->advance_control(std::nullopt,next);
        const auto b=resumed->advance_control(std::nullopt,next);
        bt::require(a.has_value()==b.has_value(),"control completion timing is identical");
        same_snapshot(continuous->snapshot(),resumed->snapshot());
        bt::require(continuous->snapshot().step==next,"continuation fixture completes 500 physical intervals");
        if(a) {
            bt::require(wire_bytes(a->as_wire())==wire_bytes(b->as_wire()),"control row and observation are bitwise equal");
            bt::require(!a->terminated&&!a->truncated,"continuation fixture stays nonterminal for the comparison window");
            continuous->acknowledge_control();resumed->acknowledge_control();
            control.human_torque_nm=1.+static_cast<double>((next/20)%3)*.125;
            continuous->begin_control(control,.25,.35);resumed->begin_control(control,.25,.35);
        }
    }
    bt::require(bytes(continuous->checkpoint())==bytes(resumed->checkpoint()),
        "500 intervals preserve all queues, accounting/history and controller owners");
    const auto a_tail=continuous->advance_control(std::nullopt),b_tail=resumed->advance_control(std::nullopt);
    bt::require(a_tail&&b_tail&&wire_bytes(a_tail->as_wire())==wire_bytes(b_tail->as_wire()),
        "the partial final period publishes the same measurement tail");
    bt::require(wire_bytes(continuous->recorded_intervals())==wire_bytes(resumed->recorded_intervals()),
        "all retained measurement rows including the 500-interval continuation are bitwise equal");
    bt::require(wire_bytes(continuous->commands_requested())==wire_bytes(resumed->commands_requested())&&
        wire_bytes(continuous->commands_applied())==wire_bytes(resumed->commands_applied())&&
        wire_bytes(continuous->observations())==wire_bytes(resumed->observations()),"whole control history is identical");
    continuous->acknowledge_control();resumed->acknowledge_control();
    resumed->reset();const auto reset=std::make_unique<runtime::RuntimeSnapshot>(resumed->snapshot());
    bt::require(reset->step==0&&reset->time_s==0.&&reset->generation>initial->generation,
        "reset retains its original t=0 origin after continuation restore");
    bt::require(reset->integration_state==initial->integration_state&&reset->tire_state_bytes==initial->tire_state_bytes,
        "reset restores original integration and material state");
    bt::require(reset->spatial&&!reset->spatial->hold.active,"reset never rearms scored bootstrap hold");
}
} // namespace
int main() { return bt::run([]{cursor_rejection();accounting_power_order();turned_rider_preview();continuation();}); }
