#include "spatial_checkpoint_internal.hpp"
#include "research_io.hpp"
#include "setup_values.hpp"
#include "../io/hash.hpp"
#include "../spatial/bootstrap_state.hpp"
#include <algorithm>
#include <set>
#include <stdexcept>
#include <string_view>

namespace runtime::spatial {
namespace {
using namespace ::runtime::setupvalue;
void keys(const Value& v, std::initializer_list<std::string_view> expected, std::string_view path) {
    const auto& object = bike::io::as_object(v,path); std::set<std::string_view> names;
    for (const auto& [key, ignored] : object) { (void)ignored;
        if (!names.insert(key).second) throw std::invalid_argument(std::string(path)+": duplicate "+key);
    }
    if (names != std::set<std::string_view>(expected)) throw std::invalid_argument(std::string(path)+": exact key set differs");
}
void hash(const Value& v, std::string_view path) {
    const auto& s = bike::io::as_string(v,path);
    if (s.size()!=64 || !std::ranges::all_of(s,[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');}))
        throw std::invalid_argument(std::string(path)+": invalid sha256");
}
void validate(const SpatialCheckpoint& c) {
    if (c.schema != kSpatialCheckpointSchema) throw std::invalid_argument("spatial_checkpoint: unsupported schema");
    keys(c.bootstrap2,{"schema","model_digest","model_dims","integration_state","model_mutable","tire","drive","rider_contacts","rider_controller","rider_intent","signals","runtime","spatial"},"checkpoint.bootstrap2");
    if (bike::io::as_int64(field(c.bootstrap2,"schema"),"checkpoint.bootstrap2.schema") != 2)
        throw std::invalid_argument("checkpoint.bootstrap2.schema: spatial bootstrap2 required");
    keys(c.identity,{"model_digest","config_sha256","terrain_sha256","calibration_sha256","solver","provider","hold_removed"},"checkpoint.identity");
    for (auto name : {"model_digest","config_sha256","terrain_sha256","calibration_sha256"}) hash(field(c.identity,name),std::string("checkpoint.identity.")+name);
    for (auto name : {"solver","provider"})
        if (bike::io::as_string(field(c.identity,name),name).empty()) throw std::invalid_argument(std::string("checkpoint.identity.")+name+": empty");
    if (!bike::io::as_bool(field(c.identity,"hold_removed"),"hold_removed")) throw std::invalid_argument("checkpoint.identity.hold_removed: scored checkpoint must release hold");
    if (bike::io::canonical_json(field(c.bootstrap2,"model_digest")) != bike::io::canonical_json(field(c.identity,"model_digest")))
        throw std::invalid_argument("checkpoint.identity.model_digest: bootstrap differs");
    const auto state = ::runtime::decode_spatial_bootstrap(field(c.bootstrap2,"spatial"));
    if (state.hold.active || state.bootstrap_phase != bike::spatial::BootstrapPhase::scored)
        throw std::invalid_argument("checkpoint.bootstrap2.spatial.hold: scored checkpoint cannot reactivate hold");
    if (state.terrain_sha256 != bike::io::as_string(field(c.identity,"terrain_sha256"),"terrain_sha256") ||
        state.calibration_sha256 != bike::io::as_string(field(c.identity,"calibration_sha256"),"calibration_sha256"))
        throw std::invalid_argument("checkpoint.identity: terrain/calibration differs from bootstrap2");
    keys(c.research,{"clock","pending_commands","actuator_delay","window","ack","metrics_cursors"},"checkpoint.research");
    (void)bike::io::as_object(field(c.research,"clock"),"checkpoint.research.clock");
    (void)bike::io::as_array(field(c.research,"pending_commands"),"checkpoint.research.pending_commands");
    (void)bike::io::as_object(field(c.research,"ack"),"checkpoint.research.ack");
    (void)bike::io::as_object(field(c.research,"metrics_cursors"),"checkpoint.research.metrics_cursors");
    keys(c.sensor_v2,{"schema","config","state"},"checkpoint.sensor_v2");
    if (bike::io::as_int64(field(c.sensor_v2,"schema"),"checkpoint.sensor_v2.schema") != 2)
        throw std::invalid_argument("checkpoint.sensor_v2.schema: unsupported");
    if (bike::io::is_null(field(c.sensor_v2,"config")) != bike::io::is_null(field(c.sensor_v2,"state")))
        throw std::invalid_argument("checkpoint.sensor_v2: inactive sensor must have null config and state");
    if (!bike::io::is_null(field(c.sensor_v2,"state"))) {
        (void)::runtime::decode_sensor_config_v2(field(c.sensor_v2,"config"));
        (void)::runtime::decode_sensor_state_v2(field(c.sensor_v2,"state"));
    }
    keys(c.policy,{"resumable","state"},"checkpoint.policy");
    if (!bike::io::as_bool(field(c.policy,"resumable"),"checkpoint.policy.resumable") && !bike::io::is_null(field(c.policy,"state")))
        throw std::invalid_argument("checkpoint.policy: non-resumable policy cannot claim state");
    // Reject duplicate keys/nonfinite JSON before storing any document owner.
    for (const auto* v : {&c.bootstrap2,&c.research,&c.sensor_v2,&c.policy,&c.identity}) (void)bike::io::canonical_json(*v);
}
} // namespace
bike::io::Value encode_checkpoint(const SpatialCheckpoint& c) {
    validate(c);
    return Value{Object{{"spatial_checkpoint",integer(c.schema)},{"bootstrap2",c.bootstrap2},
        {"research",c.research},{"sensor_v2",c.sensor_v2},{"policy",c.policy},{"identity",c.identity}}};
}
SpatialCheckpoint decode_checkpoint(const bike::io::Value& value) {
    keys(value,{"spatial_checkpoint","bootstrap2","research","sensor_v2","policy","identity"},"spatial_checkpoint");
    const auto version=bike::io::as_int64(field(value,"spatial_checkpoint"),"spatial_checkpoint");
    if(version!=kSpatialCheckpointSchema) throw std::invalid_argument("spatial_checkpoint: unsupported schema");
    SpatialCheckpoint out{.schema=kSpatialCheckpointSchema,.bootstrap2=field(value,"bootstrap2"),
        .research=field(value,"research"),.sensor_v2=field(value,"sensor_v2"),.policy=field(value,"policy"),.identity=field(value,"identity")};
    validate(out); return out;
}
void CheckpointStageDeleter::operator()(CheckpointStage* p) const noexcept { delete p; }
CheckpointStageOwner stage_checkpoint(const SpatialCheckpoint& checkpoint,const RuntimeIdentity& execution) {
    auto validated=decode_checkpoint(encode_checkpoint(checkpoint));
    if(execution.mujoco_version.empty() || execution.native_source_sha256.empty() || execution.build_context_json.empty())
        throw std::invalid_argument("checkpoint: incomplete execution identity");
    (void)bike::io::parse_json(execution.build_context_json);
    return CheckpointStageOwner(new CheckpointStage{.checkpoint=std::move(validated),.execution=execution,.consumed=false});
}
void commit_checkpoint(CheckpointStage&& stage,NativeRideRuntime& runtime) { CheckpointAccess::commit(std::move(stage),runtime); }
SpatialCheckpoint capture_checkpoint(const NativeRideRuntime& runtime,Value research,Value sensor,Value policy,Value identity) {
    SpatialCheckpoint out{.schema=kSpatialCheckpointSchema,.bootstrap2=CheckpointAccess::capture_bootstrap(runtime),
        .research=std::move(research),.sensor_v2=std::move(sensor),.policy=std::move(policy),.identity=std::move(identity)};
    validate(out); return out;
}
Value checkpoint_identity(const Value& metadata) {
    const auto& execution=field(metadata,"execution");
    return Value{Object{{"model_digest",field(metadata,"model_sha256")},
        {"config_sha256",field(metadata,"configuration_sha256")},{"terrain_sha256",field(metadata,"terrain_sha256")},
        {"calibration_sha256",field(metadata,"calibration_sha256")},{"solver",field(execution,"execution_identity")},
        {"provider",field(execution,"numeric_provider")},{"hold_removed",Value{true}}}};
}
Value inactive_research_checkpoint() { return Value{Object{{"clock",Value{Object{}}},{"pending_commands",Value{Array{}}},
    {"actuator_delay",Value{}},{"window",Value{}},{"ack",Value{Object{}}},{"metrics_cursors",Value{Object{}}}}}; }
Value inactive_sensor_checkpoint() { return Value{Object{{"schema",integer(2)},{"config",Value{}},{"state",Value{}}}}; }
Value bootstrap_comparison_value(const Value& bootstrap) {
    auto out=bootstrap;
    const auto& tire=field(bootstrap,"tire");
    const auto bytes=bike::pneumatic3d::base64_decode_strict(bike::io::as_string(field(tire,"payload"),"tire.payload"));
    if(!bytes)throw std::invalid_argument("checkpoint.tire.payload: "+bytes.error());
    const auto decoded=bike::pneumatic3d::decode_tyre_state(*bytes,bike::pneumatic3d::tyre_identity_from_wire(field(tire,"identity")));
    if(!decoded)throw std::invalid_argument("checkpoint.tire.payload: "+decoded.error());
    const auto exact=[](const auto& values) {Array out;for(const auto value:values)out.push_back(Value{bike::io::Number{std::to_string(value)}});return Value{std::move(out)};};
    Array carcass,patches,brush;
    for(std::size_t i=0;i<2;++i) {
        const auto& c=decoded->carcass[i];const auto& p=decoded->patches[i];const auto& b=decoded->brush[i];
        carcass.push_back(Value{Object{{"deflection_m",reals(c.deflection_m)},{"maxwell_q_n_per_m2",reals(c.maxwell_q_n_per_m2)},
            {"previous_penetration_m",reals(c.previous_penetration_m)},{"active",exact(c.active)},
            {"material_phase_rad",real(c.material_phase_rad)},{"stored_energy_j",real(c.stored_energy_j)},{"dissipated_j",real(c.dissipated_j)}}});
        patches.push_back(Value{Object{{"ids",exact(p.ids)},{"element_to_patch_slot",exact(p.element_to_patch_slot)},
            {"slot_used",exact(p.slot_used)},{"next_id",Value{bike::io::Number{std::to_string(p.next_id)}}},{"capacity",integer(p.capacity)}}});
        brush.push_back(Value{Object{{"ux_m",reals(b.ux_m)},{"uy_m",reals(b.uy_m)},{"camber_state",reals(b.camber_state)},
            {"bristle_ux_m",reals(b.bristle_ux_m)},{"bristle_uy_m",reals(b.bristle_uy_m)},{"bristle_energy_j",reals(b.bristle_energy_j)},
            {"elastic_energy_j",real(b.elastic_energy_j)},{"sliding_loss_j",real(b.sliding_loss_j)},{"convective_release_j",real(b.convective_release_j)}}});
    }
    set(out,"tire",Value{Object{{"pneumatic3d_state",integer(decoded->identity.schema)},
        {"identity",bike::pneumatic3d::tyre_identity_to_wire(decoded->identity)},
        {"carcass",Value{std::move(carcass)}},{"patches",Value{std::move(patches)}},{"brush",Value{std::move(brush)}},
        {"accepted_interval",integer(decoded->accepted_interval)}}});
    return out;
}
Value checkpoint_comparison_value(const Value& checkpoint) {
    auto out=checkpoint;set(out,"bootstrap2",bootstrap_comparison_value(field(checkpoint,"bootstrap2")));return out;
}
} // namespace runtime::spatial
