// W52 authored acceptance. Same binary/provider/config/seed, with only the
// explicit tyre worker count varying. Execution and qualification belong to V09.
#include "support/migration_test.hpp"
#include "spatial/support/ride_qualification.hpp"
#include "spatial/support/thread_comparison.hpp"
#include "spatial/support/tyre_fixture.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/research_io.hpp"
#include "runtime/spatial_checkpoint.hpp"
#include "pneumatic3d/state.hpp"
#include "terrain/extrusion3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace runtime::setupvalue;
using Access=bike::validation::SpatialRideAccess;
namespace io=bike::io;
namespace pn=bike::pneumatic3d;
namespace sp=bike::spatial;
namespace terrain=bike::terrain;
constexpr std::array<unsigned,4> counts{1,2,4,8};

std::string wire_bytes(const runtime::Wire& value) {
    return io::canonical_json(bike::research::wire_value(value));
}

std::vector<std::byte> engine_rows(runtime::NativeRideRuntime& ride) {
    const auto& stepper=Access::stepper(ride);
    const auto* model=stepper.model();const auto* data=stepper.data();
    bt::require(model->nsensordata>0,"the model has real engine sensor rows to compare");
    std::vector<std::byte> result;
    const auto append=[&](const double* pointer,mjtSize size) {
        const auto bytes=std::as_bytes(model_access::readonly_buffer(pointer,size));
        result.insert(result.end(),bytes.begin(),bytes.end());
    };
    append(data->qfrc_applied,model->nv);append(data->qfrc_passive,model->nv);
    append(data->qfrc_constraint,model->nv);append(data->qfrc_actuator,model->nv);
    append(data->sensordata,model->nsensordata);
    return result;
}

std::vector<std::uint64_t> contact_keys(runtime::NativeRideRuntime& ride) {
    const auto& writer=Access::stepper(ride).spatial_tire_writer();
    std::vector<std::uint64_t> result;
    for(std::size_t wheel=0;wheel<2;++wheel) {
        const auto patches=writer.model().patches(wheel);
        result.push_back(static_cast<std::uint64_t>(patches.size()));
        for(const auto& patch:patches) {
            result.push_back(patch.id);result.push_back(patch.surface_id);
            result.push_back(static_cast<std::uint64_t>(patch.elements.size()));
            for(const auto element:patch.elements)result.push_back(element);
        }
        const auto& material=writer.state().patches[wheel];
        result.push_back(material.next_id);
        result.insert(result.end(),material.ids.begin(),material.ids.end());
        result.insert(result.end(),material.element_to_patch_slot.begin(),material.element_to_patch_slot.end());
        result.insert(result.end(),material.slot_used.begin(),material.slot_used.end());
    }
    return result;
}

void same_plant(runtime::NativeRideRuntime& expected,runtime::NativeRideRuntime& actual,
                const std::string& digest,bool checkpoint) {
    bt::require(spatial_tests::complete_physics_state_bytes(expected)==
        spatial_tests::complete_physics_state_bytes(actual),
        "thread counts preserve complete integration/material/controller/accounting state bytes");
    bt::require(engine_rows(expected)==engine_rows(actual),
        "all applied/passive/constraint/actuator force rows and engine sensors are byte equal");
    bt::require(contact_keys(expected)==contact_keys(actual),
        "contact IDs, membership, slots and next-ID cursors are equal");
    const auto a=expected.snapshot(),b=actual.snapshot();
    bt::require(a.step==b.step&&a.time_s==b.time_s&&a.outcome==b.outcome,
        "the committed clock and terminal reason are identical");
    bt::require(static_cast<bool>(a.latest_sample)==static_cast<bool>(b.latest_sample),
        "the same accepted sensor/accounting publication exists");
    if(a.latest_sample)bt::require(wire_bytes(a.latest_sample->as_wire())==wire_bytes(b.latest_sample->as_wire()),
        "accepted measurements are byte equal");
    if(checkpoint)bt::require(
        io::canonical_json(spatial_tests::thread_neutral_bootstrap(expected,digest))==
        io::canonical_json(spatial_tests::thread_neutral_bootstrap(actual,digest)),
        "full successive W42 continuations agree except the proved thread-derived config digest");
}

std::shared_ptr<const terrain::Terrain3dSource> material_stripes() {
    // Narrow alternating surface cells split the real contact footprint into
    // multiple patches without introducing an arbitrary patch-count override.
    auto plane=spatial_tests::plane_grid(0.,0.,0.,801,33,.025,.1);
    std::vector<std::uint32_t> materials(800U*32U);
    for(std::size_t y=0;y<32;++y)for(std::size_t x=0;x<800;++x)
        materials[y*800+x]=x%2==0?1U:2U;
    std::vector<terrain::SurfaceSpec3d> surfaces(plane.surfaces().begin(),plane.surfaces().end());
    auto second=surfaces.front();second.id=2;second.name="synthetic-stripe-2";
    second.mu_peak=.65;second.mu_slide=.5;surfaces.push_back(std::move(second));
    auto grid=terrain::Grid3d::create(plane.descriptor(),
        std::vector<double>(plane.elevation().begin(),plane.elevation().end()),
        std::move(materials),std::move(surfaces),plane.metadata());
    if(!grid)throw std::invalid_argument(grid.error());
    return std::make_shared<const terrain::Terrain3dSource>(std::move(*grid));
}

std::shared_ptr<const terrain::Terrain3dSource> rough_bank() {
    auto spec=terrain::detail::load_track3d_spec(std::filesystem::path(BIKE_SPATIAL_FIXTURE_ROOT)/
        "terrain/genspec/constant_bank.toml");
    spec.seed_decimal="73421";spec.roughness={.001,.2};
    spec.banks.front().angle_deg=5.;
    return std::make_shared<const terrain::Terrain3dSource>(terrain::generate_grid3d(spec));
}

std::shared_ptr<const terrain::Terrain3dSource> flat_extrusion() {
    auto flat=spatial_tests::plane_grid(0.,0.,0.,4,4,1.,1.);
    auto source=terrain::ProfileExtrusion3d::create({0.,20.},{0.,0.},{1},
        std::vector<terrain::SurfaceSpec3d>(flat.surfaces().begin(),flat.surfaces().end()),
        1.5,Value{Object{{"fixture",text("W52 analytic flat extrusion")}}});
    if(!source)throw std::invalid_argument(source.error());
    return std::make_shared<const terrain::Terrain3dSource>(std::move(*source));
}

void config_exclusion_controls(const runtime::RuntimeConfig& reference) {
    auto candidate=std::make_unique<runtime::RuntimeConfig>(reference);
    candidate->tyre_threads=8;
    (void)spatial_tests::require_thread_only_difference(reference,*candidate);
    const auto rejected=[&] {
        bool failed=false;
        try{(void)spatial_tests::require_thread_only_difference(reference,*candidate);}
        catch(const std::runtime_error&){failed=true;}
        bt::require(failed,"metadata exclusion cannot hide a causal or policy configuration change");
    };
    candidate->timestep_s=reference.timestep_s*2.;rejected();
    *candidate=reference;candidate->strict=!reference.strict;rejected();
    *candidate=reference;
    candidate->spatial->steer.torque_cap_nm=candidate->spatial->steer.torque_cap_nm.value_or(40.)+1.;rejected();
    *candidate=reference;candidate->terrain3d=std::make_shared<const terrain::Terrain3dSource>(
        spatial_tests::plane_grid(0.,0.,2.,4,4,1.,1.));rejected();
}

void plant_case(sp::TyreTier tier,std::shared_ptr<const terrain::Terrain3dSource> ground,
                bool require_multiple) {
    auto request=spatial_tests::make_startup_request(sp::RootKind::free6,tier,true,3.,std::move(ground));
    request->runtime.tyre_threads=1;
    auto fixture=std::make_unique<spatial_tests::OwnedStartup>(std::move(*request));
    config_exclusion_controls(fixture->bootstrap->config);
    std::array<std::unique_ptr<runtime::NativeRideRuntime>,counts.size()> rides;
    std::string digest;
    for(std::size_t index=0;index<counts.size();++index) {
        auto bootstrap=std::make_unique<runtime::RuntimeBootstrap>(*fixture->bootstrap);
        bootstrap->config.tyre_threads=counts[index];
        digest=spatial_tests::require_thread_only_difference(fixture->bootstrap->config,bootstrap->config);
        rides[index]=std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
        bt::require(Access::stepper(*rides[index]).spatial_tire_writer().model().worker_threads()==counts[index],
            "production runtime factory propagates the typed tyre thread count");
    }
    bool multiple=false,steered=false,braked=false;
    const std::int64_t steps=tier==sp::TyreTier::fast?800:2000; // One physical second.
    for(std::int64_t step=1;step<=steps;++step) {
        runtime::RideControl command;command.human_torque_nm=2.;
        const double brake=step>steps/5&&step<steps/2?.15:0.;
        for(auto& ride:rides) {
            Access::reference_tick(*ride,0.,step<steps/2?.025:-.025,0.);
            const auto result=ride->advance(step,command,brake,brake*.75,std::nullopt);
            bt::require(result.step==step&&!result.outcome,"invariance fixture completes every valid physical interval");
        }
        for(std::size_t index=1;index<counts.size();++index)
            same_plant(*rides.front(),*rides[index],digest,step%100==0||step==steps);
        const auto& causal=Access::causal(*rides.front());
        steered=steered||causal.steer.last_applied_nm!=0.;
        braked=braked||causal.brake.wheel[0].engaged||causal.brake.wheel[1].engaged;
        const auto& model=Access::stepper(*rides.front()).spatial_tire_writer().model();
        multiple=multiple||model.patches(0).size()>1||model.patches(1).size()>1;
    }
    bt::require(steered&&braked,"the comparison exercised steering control and active brake anchors");
    if(require_multiple)bt::require(multiple,"the striped fixture exercised multiple simultaneous patches");
    for(auto& ride:rides)ride->flush();
    for(std::size_t index=1;index<counts.size();++index)
        same_plant(*rides.front(),*rides[index],digest,true);
}

// Failed work may have completed earlier ranges and the first wheel. The
// committed material transaction must still be untouched, and retry must
// agree with an owner which never saw the failed call.
void failed_element_transaction() {
    const terrain::Terrain3dSource ground(spatial_tests::plane_grid(0.,0.,0.,41,17,.5,.2));
    const auto spec=spatial_tests::synthetic_spec(sp::TyreTier::detailed);
    pn::TyreStateIdentity3d id;id.tier=sp::TyreTier::detailed;id.dims=pn::tier_dims(id.tier);
    id.patch_capacity=id.dims.n_theta_fan*id.dims.n_phi;
    id.topology_fingerprint_hex=std::string(64,'a');id.config_fingerprint_hex=std::string(64,'b');
    id.calibration_sha256_hex=std::string(64,'c');id.terrain_sha256_hex=ground.fingerprint_hex();
    std::array<pn::WheelKinematics3d,2> valid;
    for(std::size_t wheel=0;wheel<2;++wheel)valid[wheel]=spatial_tests::wheel_at(
        {4.+static_cast<double>(wheel),0.,spec.outer_radius_m[wheel]-.004},.03,.02,2.,4.);
    for(const auto count:counts) {
        pn::Pneumatic3dModel candidate(spec,ground.query_view(),id),reference(spec,ground.query_view(),id);
        candidate.set_worker_threads(count);reference.set_worker_threads(1);
        bt::invalid([&]{candidate.set_worker_threads(0);});bt::invalid([&]{candidate.set_worker_threads(65);});
        bt::require(candidate.worker_threads()==count,"rejected cold policies preserve the current pool");
        const auto before=pn::encode_tyre_state(candidate.committed_state());
        auto invalid=valid;invalid[1].axle_world={0.,0.,1.};
        bt::require(candidate.evaluate(invalid,{},.0005,true)!=CoreStatus::ok,
            "a second-wheel geometry failure is observed after first-wheel staging");
        bt::require(pn::encode_tyre_state(candidate.committed_state())==before,
            "worker error leaves the entire committed material transaction unchanged");
        bt::require(candidate.evaluate(valid,{},.0005,true)==CoreStatus::ok&&
            reference.evaluate(valid,{},.0005,true)==CoreStatus::ok,"valid retry completes");
        bt::require(pn::encode_tyre_state(candidate.committed_state())==pn::encode_tyre_state(reference.committed_state()),
            "failed worker work cannot change the accepted retry history");
    }
}

Value neutral_research_checkpoint(runtime::NativeResearchRuntime& research,const std::string& digest) {
    auto checkpoint=research.checkpoint();
    auto bootstrap=field(checkpoint,"bootstrap2");
    auto state=field(bootstrap,"runtime");auto continuation=field(state,"continuation");
    (void)io::as_string(field(continuation,"runtime_config_sha256"),"runtime_config_sha256");
    set(continuation,"runtime_config_sha256",text(digest));set(state,"continuation",std::move(continuation));
    set(bootstrap,"runtime",std::move(state));set(checkpoint,"bootstrap2",std::move(bootstrap));
    return checkpoint;
}

std::unique_ptr<runtime::ResearchBootstrap> research_copy(const runtime::ResearchBootstrap& baseline,unsigned count) {
    auto copy=std::make_unique<runtime::ResearchBootstrap>(baseline);copy->ride.config.tyre_threads=count;
    auto execution=field(copy->config.metadata,"execution");
    set(execution,"tyre_threads",integer(count));set(copy->config.metadata,"execution",std::move(execution));
    return copy;
}

void sensor_and_checkpoint_continuation() {
    auto request=std::make_unique<bike::research::ResearchRequest>();
    request->setup=std::move(*spatial_tests::make_startup_request(sp::RootKind::free6,sp::TyreTier::detailed,false,0.));
    request->capture_mode=runtime::ResearchCaptureMode::full;
    request->experiment.duration_s=1.;request->experiment.control_period_s=.01;
    request->experiment.actuator_delay_s=.02;request->experiment.record_decimation=1;request->experiment.seed_decimal="0";
    request->sensors_v2.latency_s=.02;request->sensors_v2.sample_period_s=.005;request->sensors_v2.maximum_read_period_s=.01;
    auto prepared=bike::research::prepare_environment(std::move(request));
    auto baseline=std::make_unique<runtime::ResearchBootstrap>(std::move(prepared->bootstrap));
    bt::require(baseline->ride.state.tire3d.has_value(),"research fixture owns complete detailed tyre material state");
    for(auto& brush:baseline->ride.state.tire3d->brush) {
        std::ranges::fill(brush.bristle_uy_m,1e-6);std::ranges::fill(brush.bristle_energy_j,1e-12);
        brush.elastic_energy_j=1e-12*static_cast<double>(brush.bristle_energy_j.size());
    }
    std::array<std::unique_ptr<runtime::NativeResearchRuntime>,counts.size()> research;
    std::string digest;
    runtime::RideControl control;control.human_torque_nm=1.;
    for(std::size_t index=0;index<counts.size();++index) {
        auto bootstrap=research_copy(*baseline,counts[index]);
        digest=spatial_tests::require_thread_only_difference(baseline->ride.config,bootstrap->ride.config);
        research[index]=runtime::make_research(std::move(bootstrap));
        research[index]->begin_control(control,.25,.35);
    }
    for(std::int64_t next=1;next<=535;++next) {
        std::array<std::optional<runtime::ResearchTransition>,counts.size()> transitions;
        for(std::size_t index=0;index<counts.size();++index) {
            transitions[index]=research[index]->advance_control(std::nullopt,next);
            bt::require(research[index]->snapshot().step==next,"sensor fixture commits every requested interval");
        }
        for(std::size_t index=1;index<counts.size();++index) {
            bt::require(spatial_tests::physics_state_bytes(research.front()->snapshot())==
                spatial_tests::physics_state_bytes(research[index]->snapshot()),
                "sensor-driven runs preserve complete committed physical bytes");
            bt::require(transitions[index].has_value()==transitions.front().has_value(),
                "thread count preserves control delivery timing");
            if(transitions.front())bt::require(wire_bytes(transitions.front()->as_wire())==wire_bytes(transitions[index]->as_wire()),
                "sensor observations, truth and control transitions agree exactly");
            if(next%20==0||next==35||next==535)bt::require(
                io::canonical_json(neutral_research_checkpoint(*research.front(),digest))==
                io::canonical_json(neutral_research_checkpoint(*research[index],digest)),
                "full research continuation, delayed queues, random cursor and all sensor state are equal");
        }
        if(next==35)for(std::size_t index=0;index<counts.size();++index) {
            const auto checkpoint=research[index]->checkpoint();
            const auto sensor=runtime::decode_sensor_state_v2(field(field(checkpoint,"sensor_v2"),"state"));
            bt::require(sensor.queue.size()>1&&sensor.cursor>1,"checkpoint contains undelivered sensor data and an advanced noise cursor");
            const auto& causal=field(checkpoint,"research");
            bt::require(!io::is_null(field(causal,"window"))&&
                !io::as_array(field(causal,"pending_commands"),"pending_commands").empty(),
                "checkpoint interrupts an active control window with delayed actuator commands");
            auto resumed=runtime::make_research(research_copy(*baseline,counts[index]));
            resumed->restore_checkpoint(io::parse_recording_json(io::recording_json(checkpoint)));
            bt::require(io::canonical_json(resumed->checkpoint())==io::canonical_json(checkpoint),
                "each requested pool count survives an exact cold checkpoint restore");
            research[index]=std::move(resumed);
        }
        if(transitions.front()) {
            bt::require(!transitions.front()->terminated&&!transitions.front()->truncated,
                "sensor continuation stays nonterminal throughout the comparison");
            control.human_torque_nm=1.+static_cast<double>((next/20)%3)*.125;
            for(auto& run:research){run->acknowledge_control();run->begin_control(control,.25,.35);}
        }
    }
    for(std::size_t index=1;index<counts.size();++index) {
        bt::require(wire_bytes(research.front()->recorded_intervals())==wire_bytes(research[index]->recorded_intervals()),
            "all retained measurement rows remain identical after checkpoint continuation");
        bt::require(wire_bytes(research.front()->commands_requested())==wire_bytes(research[index]->commands_requested())&&
            wire_bytes(research.front()->commands_applied())==wire_bytes(research[index]->commands_applied())&&
            wire_bytes(research.front()->observations())==wire_bytes(research[index]->observations()),
            "complete command and sensor histories agree across every tyre thread count");
    }
}
} // namespace

int main() {
    return bt::run([] {
        failed_element_transaction();
        plant_case(sp::TyreTier::fast,{},false);
        plant_case(sp::TyreTier::fast,rough_bank(),false);
        plant_case(sp::TyreTier::fast,material_stripes(),true);
        plant_case(sp::TyreTier::detailed,material_stripes(),true);
        sensor_and_checkpoint_continuation();
        // Required W52 gate. The W24/W25 frozen material resolver currently
        // rejects extrusion owners (C10). This case is deliberately neither
        // skipped nor xfailed: V00/V05 must resolve that upstream blocker
        // before a V09 thread-invariance qualification can be claimed.
        plant_case(sp::TyreTier::fast,flat_extrusion(),false);
    });
}
