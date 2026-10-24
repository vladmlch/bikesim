#include "support/migration_test.hpp"
#include "support/startup_cases.hpp"
#include "support/ride_qualification.hpp"
#include "initialization/state_file.hpp"
#include "initialization/spatial_equilibrium.hpp"
#include "io/hash.hpp"
#include "model_access.hpp"
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
void failure_keeps_committed_prefix() {
    using namespace bike;
    const auto request=spatial_tests::make_startup_request(spatial::RootKind::free6,spatial::TyreTier::fast,false,3.);
    spatial_tests::OwnedStartup setup(*request);
    auto& ride=*setup.runtime;
    const auto before=ride.snapshot();
    auto& stepper=validation::SpatialRideAccess::stepper(ride);
    auto& physical=validation::SpatialRideAccess::physical(ride);
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
    bt::require(!ride.snapshot().outcome && ride.advance(1,runtime::RideControl{},0.,0.,std::nullopt).step==1,
        "reset restores the captured physical bootstrap after rejected geometry");
}
} // namespace

int main(){return bt::run([] {
    using namespace bike;
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
            }
    failure_keeps_committed_prefix();
});}
