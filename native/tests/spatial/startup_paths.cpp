#include "support/migration_test.hpp"
#include "support/startup_cases.hpp"
#include "support/ride_qualification.hpp"
#include "initialization/state_file.hpp"
#include "initialization/spatial_equilibrium.hpp"
#include "io/hash.hpp"
#include "model_access.hpp"
#include "model/mapping.hpp"
#include "research/environment.hpp"
#include "runtime/research_io.hpp"
#include "runtime/setup_values.hpp"
#include "spatial/frames.hpp"
#include "writers/pneumatic_3d_tire.hpp"
#include "writers/spatial_tire_adapter.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <unistd.h>

namespace {
class Directory {
public:
    Directory() {
        static std::atomic<unsigned> serial{0};
        for(unsigned attempt=0;attempt<64;++attempt) {
            path=std::filesystem::temp_directory_path()/
                ("bike-spatial-startup-"+std::to_string(::getpid())+"-"+std::to_string(serial.fetch_add(1)));
            if(std::filesystem::create_directory(path))return;
        }
        throw std::runtime_error("unable to allocate exclusive startup-test directory");
    }
    ~Directory(){std::error_code ignored;std::filesystem::remove_all(path,ignored);}
    Directory(const Directory&)=delete;Directory& operator=(const Directory&)=delete;
    std::filesystem::path path;
};
bool same_bits(std::span<const double> a,std::span<const double> b) {
    return std::ranges::equal(std::as_bytes(a),std::as_bytes(b));
}
void same_probe(const runtime::PhysicalStep::ProbeResult& a,const runtime::PhysicalStep::ProbeResult& b) {
    bt::require(a.components.size()==b.components.size(),"startup first force stage has identical components");
    for(std::size_t i=0;i<a.components.size();++i)
        bt::require(a.components[i].first==b.components[i].first && same_bits(a.components[i].second,b.components[i].second),
            "first-stage generalized force components match bitwise");
    bt::require(same_bits(a.ctrl,b.ctrl) &&
        std::bit_cast<std::uint64_t>(a.front_brake_bound_nm)==std::bit_cast<std::uint64_t>(b.front_brake_bound_nm) &&
        std::bit_cast<std::uint64_t>(a.rear_brake_bound_nm)==std::bit_cast<std::uint64_t>(b.rear_brake_bound_nm),
        "first-stage controls and solved brake bounds match bitwise");
}
std::string continuation(runtime::NativeRideRuntime& ride) {
    bike::io::Sha256 digest;
    for(std::int64_t step=1;step<=1000;++step) {
        const auto result=ride.advance(step,runtime::RideControl{},0.,0.,std::nullopt);
        bt::require(result.step==step && !result.outcome,"startup continuation reaches every requested accepted interval");
        const auto state=spatial_tests::complete_physics_state_bytes(ride);digest.update(state);
    }
    const auto bytes=digest.finish();
    return bike::io::sha256(std::as_bytes(std::span{bytes}));
}
void moving_start(const spatial_tests::OwnedStartup& setup,double speed) {
    const auto causal=setup.runtime->snapshot().spatial;
    bt::require(causal && causal->bootstrap_phase==bike::spatial::BootstrapPhase::scored &&
        !causal->hold.active && causal->hold.last_force_world_n==bike::spatial::Vec3{} &&
        causal->hold.last_couple_world_nm==bike::spatial::Vec3{},"scored moving start is released from its bootstrap support");
    bt::require(causal->brake.interval_index==-1 && causal->steer.next_tick==0,
        "scored brake and steering clocks begin before interval zero");
    bike::validation::RuntimeRigAccess::inspect(*setup.runtime,[&](const Stepper& s,const runtime::PhysicalStep& p) {
        const auto& topology=*p.topology();const auto pose=bike::spatial::body_pose(*s.model(),*s.data(),topology.root);
        const auto positions=model_access::readonly_buffer(s.data()->xpos,3*s.model()->nbody);
        bt::require(std::abs(pose.linear_velocity_world_mps[0]-speed)<=1e-12 &&
            std::abs(pose.linear_velocity_world_mps[1])<=1e-12 && std::abs(pose.linear_velocity_world_mps[2])<=1e-12,
            "moving start receives its prescribed world heading velocity without an impulse");
        for(std::size_t side=0;side<2;++side) {
            // On this z=0 plane every contact normal is world Z. The loaded
            // rolling radius is therefore the independently observed hub
            // height, including the tyre's solved vertical compression.
            const double radius=positions[3*static_cast<std::size_t>(topology.wheels[side].wheel_body_id)+2];
            bt::require(radius>0.,"moving start has a physical loaded rolling radius");
            bt::require(std::abs(s.qvel()[static_cast<std::size_t>(topology.wheels[side].spin.dof_adr)]-speed/radius)<=1e-12,
                "each wheel is initialized at v divided by its loaded effective radius");
        }
        bt::require(p.state().energy.initial_energy_j.has_value(),"initial moving kinetic and stored energy are captured");
    });
}
void lifted_initial_state_is_rejected(const spatial_tests::OwnedStartup& source,
        bike::initialization::SetupRequest request,const bike::initialization::PhysicalInitialState& initial,
        const bike::initialization::StateFile& saved) {
    using namespace bike;
    namespace values=runtime::setupvalue;
    auto payload=initial.payload();
    bike::validation::RuntimeRigAccess::inspect(*source.runtime,[&](const Stepper& stepper,const runtime::PhysicalStep& physical) {
        auto integration=initialization::state_array(saved,"integration_state").doubles();
        auto coordinates=initialization::state_array(saved,"qpos").doubles();
        const auto* m=stepper.model();
        const std::unique_ptr<mjData,decltype(&mj_deleteData)> probe(engine::make_data(m),&mj_deleteData);
        bt::require(bool(probe),"lifted import has independent integration storage");
        engine::set_state(m,probe.get(),integration.data(),mjSTATE_INTEGRATION);
        const auto height=static_cast<std::size_t>(physical.topology()->root.free_qpos_adr+2);
        coordinates[height]+=.25;model_access::mutable_buffer(probe->qpos,m->nq)[height]=coordinates[height];
        engine::get_state(m,probe.get(),integration.data(),mjSTATE_INTEGRATION);
        values::set(payload,"qpos",values::reals(coordinates));
        auto engine_doc=values::field(payload,"spatial_engine");
        auto record=values::field(engine_doc,"integration_state");
        values::set(record,"values",values::reals(integration));
        values::set(engine_doc,"integration_state",std::move(record));
        values::set(payload,"spatial_engine",std::move(engine_doc));
    });
    // Keep the real source's successful report and scored phase. Both the
    // coordinate envelope and its integration record agree on the lifted pose.
    request.initial_state.emplace(io::canonical_json(payload));
    const auto options=request.equilibrium;
    auto owner=initialization::make_setup(std::move(request));
    bool rejected=false;
    try {(void)initialization::settle(*owner,options);}
    catch(const std::invalid_argument& error) {
        rejected=std::string_view(error.what())=="spatial equilibrium: missing or invalid wheel contacts";
    }
    bt::require(rejected,"an internally consistent scored import cannot substitute source evidence for current physical support");
    bt::invalid([&]{(void)owner->seal_bootstrap();});
}
void failure_keeps_committed_prefix() {
    using namespace bike;
    const auto request=spatial_tests::make_startup_request(spatial::RootKind::free6,spatial::TyreTier::fast,false,3.);
    spatial_tests::OwnedStartup setup(*request);
    auto& ride=*setup.runtime;
    const auto before=ride.snapshot();
    auto& stepper=bike::validation::SpatialRideAccess::stepper(ride);
    auto& physical=bike::validation::SpatialRideAccess::physical(ride);
    const auto* model=stepper.model();auto* data=stepper.data();
    const auto friction=model_access::readonly_buffer(model->dof_frictionloss,model->nv);
    const auto& wheels=physical.topology()->wheels;
    const std::array old_friction{friction[static_cast<std::size_t>(wheels[0].spin.dof_adr)],
        friction[static_cast<std::size_t>(wheels[1].spin.dof_adr)]};
    bt::require(before.spatial && before.spatial->steer.next_tick==0 && before.spatial->brake.interval_index==-1,
        "failure fixture starts immediately before a live brake and steering control tick");
    stepper.mutate([&] {
        const auto coordinates=model_access::mutable_buffer(data->qpos,model->nq);
        const auto q=static_cast<std::size_t>(physical.topology()->root.free_qpos_adr+3);
        // A vertical axle makes the tyre's gravity projection degenerate.
        // Forward alone does not evaluate the external tyre: rejection occurs
        // inside the next force assembly, after brake/steering tick updates.
        coordinates[q]=std::sqrt(.5);coordinates[q+1]=std::sqrt(.5);
        coordinates[q+2]=coordinates[q+3]=0.;
        stepper.note_posvel_stale();stepper.forward();
    });
    bool rejected=false;
    try {(void)ride.advance(1,runtime::RideControl{},.65,.35,std::nullopt);}
    catch(const pneumatic3d::SpatialTireEvaluationError&) {rejected=true;}
    bt::require(rejected,"real spatial tyre geometry rejects the attempted accepted interval");
    const auto failed=ride.snapshot();
    bt::require(failed.step==before.step && failed.time_s==before.time_s &&
        same_bits(failed.integration_state,before.integration_state) && failed.tire_state_bytes==before.tire_state_bytes,
        "failed force assembly publishes the previous engine and tyre prefix bitwise");
    bt::require(failed.spatial && failed.spatial->outcome.latched.terminal && !failed.spatial->outcome.latched.valid,
        "failed force assembly adds a typed invalid terminal outcome");
    auto normalized=*failed.spatial;normalized.outcome=before.spatial->outcome;
    bt::require(io::canonical_json(runtime::encode_spatial_bootstrap(normalized))==
        io::canonical_json(runtime::encode_spatial_bootstrap(*before.spatial)),
        "the failed committed spatial record differs only in its outcome, never causal generations");
    const std::unique_ptr<runtime::StepState> live{new runtime::StepState(physical.state())};
    const std::array restored_friction{friction[static_cast<std::size_t>(wheels[0].spin.dof_adr)],
        friction[static_cast<std::size_t>(wheels[1].spin.dof_adr)]};
    bt::require(io::canonical_json(spatial::encode_brake(live->spatial->brake))==
        io::canonical_json(spatial::encode_brake(before.spatial->brake)) &&
        io::canonical_json(spatial::encode_steer(live->spatial->steer))==
        io::canonical_json(spatial::encode_steer(before.spatial->steer)) && same_bits(restored_friction,old_friction),
        "live brake/steer owners and brake friction bounds roll back after an unaccepted interval");
    bool needs_reset=false;
    try {(void)ride.advance(1,runtime::RideControl{},0.,0.,std::nullopt);}
    catch(const std::logic_error& error) {needs_reset=std::string_view(error.what()).find("reset")!=std::string_view::npos;}
    bt::require(needs_reset,"an invalid unaccepted boundary cannot be advanced before reset");
    ride.reset();
    const auto reset_frame=ride.render_frame();
    bt::require(reset_frame.presentation.spatial &&
        reset_frame.presentation.spatial->generation==static_cast<std::uint64_t>(reset_frame.state.generation),
        "reset publishes the current spatial presentation generation before the first advance");
    bt::require(!ride.snapshot().outcome && ride.advance(1,runtime::RideControl{},0.,0.,std::nullopt).step==1,
        "reset restores the captured physical bootstrap after rejected geometry");
}
void rider_spring_is_not_material_loss() {
    using namespace bike;
    const auto request=spatial_tests::make_startup_request(
        spatial::RootKind::free6,spatial::TyreTier::fast,true,0.);
    spatial_tests::OwnedStartup setup(*request);
    auto& stepper=bike::validation::SpatialRideAccess::stepper(*setup.runtime);
    auto& physical=bike::validation::SpatialRideAccess::physical(*setup.runtime);
    const auto* model=stepper.model();
    auto* data=stepper.data();
    const auto joint=static_cast<std::size_t>(
        bike::model::required_id(*model,mjOBJ_JOINT,"shoulder_swivel_l"));
    const auto address=bike::model::joint_address(*model,"shoulder_swivel_l");
    const auto qa=static_cast<std::size_t>(address.first);
    const auto va=static_cast<std::size_t>(address.second);
    const auto spring=model_access::readonly_buffer(model->qpos_spring,model->nq);
    const double stiffness=model_access::readonly_buffer(model->jnt_stiffness,model->njnt)[joint];
    const double damping=model_access::readonly_buffer(model->dof_damping,model->nv)[va];
    bt::require(stiffness>0. && damping>0.,"spatial shoulder has spring and damping");
    stepper.mutate([&] {
        model_access::mutable_buffer(data->qpos,model->nq)[qa]=spring[qa]+.01;
        model_access::mutable_buffer(data->qvel,model->nv)[va]=.02;
        stepper.note_posvel_stale();stepper.forward();
    });
    const std::unique_ptr<runtime::RawStep> raw{
        new runtime::RawStep(physical.advance_physics(0.,0.,std::nullopt,{}))};
    const auto component=[&](std::string_view name) -> const std::vector<double>& {
        const auto found=std::ranges::find_if(raw->components,
            [&](const auto& row){return row.first==name;});
        bt::require(found!=raw->components.end(),"solved interval contains "+std::string(name));
        return found->second;
    };
    const double rider=component("rider_passive_damping")[va];
    const double engine=component("engine_passive")[va];
    const double expected_spring=-stiffness*(raw->qpos[qa]-spring[qa]);
    bt::require(std::abs(rider+damping*raw->qvel[va])<=1e-10,
        "rider material-loss row contains damping, not conservative shoulder force");
    bt::require(std::abs(engine-expected_spring)<=1e-10,
        "engine passive row retains the shoulder spring for its stored-energy refund");
    bt::require(std::abs(rider+engine-raw->qfrc_passive[va])<=1e-12,
        "passive component partition preserves the solved generalized force");
}
void research_quality_excludes_invalid_interval(bool completes) {
    using namespace bike;
    namespace values=runtime::setupvalue;
    namespace wire=runtime::sample_wire;
    auto request=std::make_unique<research::ResearchRequest>();
    request->setup=std::move(*spatial_tests::make_startup_request(
        spatial::RootKind::free6,spatial::TyreTier::fast,false,2.));
    const double dt=request->setup.model.physics.value().timestep_s;
    request->capture_mode=runtime::ResearchCaptureMode::full;
    request->experiment.duration_s=completes?dt:2.*dt;
    request->experiment.control_period_s=dt;
    request->experiment.actuator_delay_s=0.;
    request->experiment.record_decimation=1;
    request->experiment.stop_on_model_violation=false;
    request->experiment.async_telemetry=false;
    request->sensors_v2.sample_period_s=dt;
    request->sensors_v2.latency_s=0.;
    auto prepared=research::prepare_environment(std::move(request));
    const double threshold=prepared->bootstrap.config.maximum_energy_residual_ratio;
    const double start_position=prepared->bootstrap.config.start_position_m;
    // Deliberately manufacture a bad accounting baseline without changing
    // the valid plant. The consumer must reject its first accepted sample
    // even when that same interval completes the requested physical ride.
    auto energy=values::field(prepared->bootstrap.ride.state.runtime,"energy");
    values::set(energy,"initial_energy_j",values::real(
        io::as_double(values::field(energy,"initial_energy_j"),"initial_energy_j")+1e6));
    values::set(energy,"energy_scale_j",values::real(1.));
    values::set(prepared->bootstrap.ride.state.runtime,"energy",std::move(energy));
    auto environment=runtime::make_research(
        std::make_unique<runtime::ResearchBootstrap>(std::move(prepared->bootstrap)));
    runtime::RideControl control;control.motor_torque_nm=0.;control.human_torque_nm=0.;
    environment->begin_control(control,0.,0.);
    const auto transition=environment->advance_control(std::nullopt);
    bt::require(transition.has_value(),"one-interval research quality fixture completes its control window");
    const auto& result=transition.value();
    const auto accepted=environment->snapshot();
    bt::require(accepted.step==1 && accepted.latest_sample &&
        accepted.latest_sample->interval_id==0 && accepted.position_m>start_position,
        "quality rejection consumes a real accepted interval with positive distance");
    const auto& source=*accepted.latest_sample;
    const auto& outcome=wire::object(wire::required(source.channels,"spatial_outcome"));
    bt::require(wire::boolean(wire::required(outcome,"valid")) &&
        wire::boolean(wire::required(outcome,"terminal"))==completes,
        "physical validity and completion are independent of the manufactured quality failure");
    const auto quality=runtime::research_quality(
        wire::object(wire::required(source.channels,"energy")),threshold);
    bt::require(!quality.acceptable && quality.residual_ratio>threshold,
        "manufactured baseline produces an observed energy-quality rejection");
    bt::require(!result.numerically_valid && !result.terminated && result.truncated &&
        result.reason==(completes?"completed":"numerical_quality"),
        "quality failure invalidates both terminal and nonterminal research transitions");
    const auto status=environment->status();
    const auto& metrics=wire::object(wire::required(status,"spatial"));
    bt::require(!wire::boolean(wire::required(metrics,"valid")) &&
        wire::number(wire::required(metrics,"valid_duration_s"))==0. &&
        wire::number(wire::required(metrics,"completed_distance_m"))==0.,
        "the bad interval contributes neither valid duration nor completed distance");
    bt::require(wire::number(wire::required(status,"max_energy_residual_ratio"))==quality.residual_ratio,
        "terminal priority does not hide the accepted sample's energy residual");
    bt::require(wire::string(wire::required(metrics,"terminal_reason"))==
        spatial::to_string(completes?spatial::SpatialReason::requested_completion:spatial::SpatialReason::none) &&
        wire::integer(wire::required(metrics,"terminal_tick"))==(completes?0:-1),
        "research quality preserves the physical terminal reason and tick");
}
} // namespace

int main(){return bt::run([] {
    using namespace bike;
    auto shifted=spatial_tests::make_startup_request(spatial::RootKind::free6,spatial::TyreTier::fast,false,0.,
        std::make_shared<const terrain::Terrain3dSource>(spatial_tests::asymmetric_4x4()));
    bool shifted_rejected=false;
    try {(void)initialization::make_setup(std::move(*shifted));}
    catch(const std::invalid_argument& error) {
        shifted_rejected=std::string_view(error.what()).starts_with("setup.spatial.corridor_half_width_m:");
    }
    bt::require(shifted_rejected,"a sufficiently wide shifted grid must cover both sides of the scored corridor");
    Directory temporary;unsigned case_id=0;
    for(auto tier:{spatial::TyreTier::fast,spatial::TyreTier::detailed})
        for(auto root:{spatial::RootKind::constrained4,spatial::RootKind::free6})
            for(bool articulated:{false,true}) {
                auto request=spatial_tests::make_startup_request(root,tier,articulated,3.);
                if(!articulated)request->model.rider->variant="lumped";
                request->cache_directory=temporary.path/("cache-"+std::to_string(case_id++));
                request->model.physics->equilibrium_cache_enabled=true;
                spatial_tests::OwnedStartup fresh(*request);
                bt::require(!fresh.equilibrium.cache_hit,"first native setup follows fresh equilibrium path");
                const auto transferred=fresh.owner->capture_initial_state();
                const auto saved=fresh.owner->capture_state_file();
                if(tier==spatial::TyreTier::fast && root==spatial::RootKind::free6 && !articulated)
                    lifted_initial_state_is_rejected(fresh,*request,transferred,saved);
                spatial_tests::OwnedStartup cached(*request);
                bt::require(cached.equilibrium.cache_hit,"second identical setup restores the native cache");
                auto import_request=*request;import_request.model.physics->equilibrium_cache_enabled=false;
                import_request.initial_state=transferred;
                spatial_tests::OwnedStartup imported(std::move(import_request));
                moving_start(fresh,3.);moving_start(cached,3.);moving_start(imported,3.);
                const auto state=spatial_tests::complete_physics_state_bytes(*fresh.runtime);
                bt::require(state==spatial_tests::complete_physics_state_bytes(*cached.runtime),
                    "cache restores the same complete physics state; no physical lane is normalized away");
                bt::require(state==spatial_tests::complete_physics_state_bytes(*imported.runtime),
                    "explicit captured t=0 import restores all causal owners and mutable model rows");
                const auto view=io::recording_json(research::wire_value(runtime::Wire(fresh.runtime->snapshot().view)));
                bt::require(view==io::recording_json(research::wire_value(runtime::Wire(cached.runtime->snapshot().view))) &&
                    view==io::recording_json(research::wire_value(runtime::Wire(imported.runtime->snapshot().view))),
                    "first physical observation is independent of fresh/cache/import provenance");
                const auto force=fresh.runtime->probe_step_inputs(runtime::RideControl{},0.,0.);
                same_probe(force,cached.runtime->probe_step_inputs(runtime::RideControl{},0.,0.));
                same_probe(force,imported.runtime->probe_step_inputs(runtime::RideControl{},0.,0.));
                bt::require(state==spatial_tests::complete_physics_state_bytes(*fresh.runtime),"the first force probe cannot consume bootstrap state");
                const auto fresh_digest=continuation(*fresh.runtime);
                bt::require(fresh_digest==continuation(*cached.runtime) && fresh_digest==continuation(*imported.runtime),
                    "1000 accepted intervals preserve exact continuation across every startup path");

                auto bad_payload=transferred.payload();
                for(auto& [name,value]:std::get<io::Value::Object>(bad_payload.data))
                    if(name=="contract_sha256")value=io::Value{std::string(64,'f')};
                auto mismatch=*request;mismatch.initial_state.emplace(io::canonical_json(bad_payload));
                bt::invalid([&]{spatial_tests::OwnedStartup rejected(std::move(mismatch));});
                auto planar=*fresh.bootstrap;planar.state.schema=1;planar.state.spatial.reset();planar.state.tire3d.reset();
                bt::invalid([&]{runtime::NativeRideRuntime rejected(std::move(planar));});
                const auto explicit_file=temporary.path/("explicit-"+std::to_string(case_id)+".json");
                auto foreign=saved;
                for(auto& [name,array]:foreign.arrays)if(name=="fingerprint_sha256")
                    array=io::string_array(std::array<std::string,1>{std::string(64,'e')},{});
                initialization::write_state_file(explicit_file,foreign);
                auto explicit_request=*request;explicit_request.model.physics->equilibrium_state_file=explicit_file.string();
                bt::invalid([&]{spatial_tests::OwnedStartup rejected(std::move(explicit_request));});

                std::filesystem::path cache_file;
                for(const auto& entry:std::filesystem::directory_iterator(*request->cache_directory))
                    if(entry.path().extension()==".npz") {bt::require(cache_file.empty(),"each isolated request has one cache key");cache_file=entry.path();}
                bt::require(!cache_file.empty(),"fresh equilibrium publishes its optional cache");
                {
                    std::fstream file(cache_file,std::ios::in|std::ios::out|std::ios::binary);
                    char first{};file.read(&first,1);bt::require(bool(file),"cache corruption fixture opens bytes");
                    first=static_cast<char>(static_cast<unsigned char>(first)^1U);file.seekp(0);file.write(&first,1);
                    bt::require(bool(file),"cache corruption fixture flips one byte");
                }
                spatial_tests::OwnedStartup recovered(*request);
                const auto metadata=recovered.owner->metadata();
                const auto* reason=io::find_member(io::as_object(metadata,"setup"),"cache_rejection_reason");
                bt::require(!recovered.equilibrium.cache_hit && reason && !io::is_null(*reason) &&
                    !io::as_string(*reason,"cache_rejection_reason").empty(),"corrupt optional cache records a reason and takes the fresh path");
                bt::require(state==spatial_tests::complete_physics_state_bytes(*recovered.runtime),"corrupt-cache fallback is the same deterministic fresh setup");
                if(tier==spatial::TyreTier::fast && root==spatial::RootKind::free6 && !articulated) {
                    // A structurally valid cache can still contain an unusable
                    // physical pose. Preserve agreement of its two coordinate
                    // records so rejection requires the real terrain probe.
                    auto semantic=initialization::read_state_file(cache_file);
                    bike::validation::RuntimeRigAccess::inspect(*recovered.runtime,[&](const Stepper& stepper,const runtime::PhysicalStep& physical) {
                        auto integration=initialization::state_array(semantic,"integration_state").doubles();
                        auto coordinates=initialization::state_array(semantic,"qpos").doubles();
                        auto friction=initialization::state_array(semantic,"model_dof_frictionloss").doubles();
                        const auto& binding=physical.topology()->root;
                        const auto station=static_cast<std::size_t>(binding.free_qpos_adr);
                        const auto lateral=static_cast<std::size_t>(binding.free_dof_adr+1);
                        const std::unique_ptr<mjData,decltype(&mj_deleteData)> probe(engine::make_data(stepper.model()),&mj_deleteData);
                        bt::require(bool(probe),"semantic-cache mutation has an independent data owner");
                        engine::set_state(stepper.model(),probe.get(),integration.data(),mjSTATE_INTEGRATION);
                        coordinates[station]=request->model.terrain3d->x_max_m()+10.;
                        model_access::mutable_buffer(probe->qpos,stepper.model()->nq)[station]=coordinates[station];
                        engine::get_state(stepper.model(),probe.get(),integration.data(),mjSTATE_INTEGRATION);
                        friction[lateral]+=7.;
                        for(auto& [name,array]:semantic.arrays) {
                            if(name=="qpos")array=io::double_array(coordinates,{coordinates.size()});
                            else if(name=="integration_state")array=io::double_array(integration,{integration.size()});
                            else if(name=="model_dof_frictionloss")array=io::double_array(friction,{friction.size()});
                        }
                    });
                    initialization::write_state_file(cache_file,semantic);
                    spatial_tests::OwnedStartup semantic_recovered(*request);
                    const auto semantic_metadata=semantic_recovered.owner->metadata();
                    const auto* semantic_reason=io::find_member(io::as_object(semantic_metadata,"setup"),"cache_rejection_reason");
                    bt::require(!semantic_recovered.equilibrium.cache_hit && semantic_reason && !io::is_null(*semantic_reason),
                        "an unusable decoded cache records its failed physical check and falls back");
                    bt::require(state==spatial_tests::complete_physics_state_bytes(*semantic_recovered.runtime),
                        "semantic-cache fallback restores fresh causal owners and mutable model rows exactly");
                    initialization::write_state_file(explicit_file,semantic);
                    auto semantic_explicit=*request;semantic_explicit.model.physics->equilibrium_state_file=explicit_file.string();
                    bt::invalid([&]{spatial_tests::OwnedStartup rejected(std::move(semantic_explicit));});
                }
            }
    failure_keeps_committed_prefix();
    rider_spring_is_not_material_loss();
    research_quality_excludes_invalid_interval(false);
    research_quality_excludes_invalid_interval(true);
});}
