#include "context.hpp"
#include "../diag.hpp"
#include "state_file.hpp"
#include "source_json.hpp"
#include "../drivetrain/state_codec.hpp"
#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"
#include "../writers/drivetrain.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <format>
#include <limits>
#include <memory>
#include <ranges>
#include <set>
#include <stdexcept>
#include <tuple>

namespace bike::initialization {
namespace {
io::Value number(double value){return io::Value{io::Number{configuration::detail::repr_double(value)}};}
const io::Value& field(const io::Value::Object& object,std::string_view key){const auto* value=io::find_member(object,key);if(!value)throw std::invalid_argument("equilibrium identity: missing "+std::string(key));return *value;}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
io::Value plain(io::Value value){
    if(auto* object=std::get_if<io::Value::Object>(&value.data)){
        if(object->size()==1&&object->front().first=="$tuple")return plain(object->front().second);
        for(auto& item:*object)item.second=plain(std::move(item.second));
    }else if(auto* array=std::get_if<io::Value::Array>(&value.data))for(auto& item:*array)item=plain(std::move(item));
    return value;
}
std::string hex(const std::array<std::uint8_t,32>& bytes){constexpr std::string_view chars="0123456789abcdef";std::string out;out.reserve(64);for(auto const c:bytes){out.push_back(chars[std::uint32_t{c}>>4U]);out.push_back(chars[std::uint32_t{c}&15U]);}return out;}
std::string shape_string(std::span<const std::size_t> shape){std::string text="(";for(std::size_t i=0;i<shape.size();++i){if(i)text+=", ";text+=std::to_string(shape[i]);}if(shape.size()==1)text+=',';return text+")";}
void update(io::Sha256& digest,std::string_view name,const io::NpyArray& array,std::string_view dtype_name){digest.update(std::as_bytes(std::span{name}));digest.update(std::as_bytes(std::span{dtype_name}));const std::string shape=shape_string(array.shape);digest.update(std::as_bytes(std::span{shape}));digest.update(array.bytes);}
const io::NpyArray* find(const StateFile& state,std::string_view name){for(const auto& [key,value]:state.arrays)if(key==name)return &value;return nullptr;}
void shape(const io::NpyArray& array,std::initializer_list<std::size_t> expected,std::string_view name){if(array.shape!=std::vector<std::size_t>(expected))throw std::invalid_argument("equilibrium state: wrong shape for "+std::string(name));}
std::vector<double> doubles(const StateFile& state,std::string_view name,std::initializer_list<std::size_t> expected,bool nonfinite=false){const auto& a=state_array(state,name);shape(a,expected,name);return a.doubles(nonfinite);}
std::vector<bool> bools(const StateFile& state,std::string_view name,std::initializer_list<std::size_t> expected){const auto& a=state_array(state,name);shape(a,expected,name);return a.booleans();}
std::string text(const StateFile& state,std::string_view name){const auto& a=state_array(state,name);if(a.elements()!=1)throw std::invalid_argument("equilibrium state: scalar string required");return a.strings().front();}
void put_text(StateFile& state,std::string name,const std::string& text){state.arrays.emplace_back(std::move(name),io::string_array(std::array<std::string,1>{text},{}));}
void put_number(StateFile& state,std::string name,double value){state.arrays.emplace_back(std::move(name),io::double_array(std::array<double,1>{value},{}));}
void settle_metadata(const SetupContext& context,const StateFile& state){
    (void)doubles(state,"qpos",{static_cast<std::size_t>(context.stepper.model()->nq)});
    const auto& steps=state_array(state,"steps");const auto& residual=state_array(state,"residual_qacc");
    if(steps.elements()!=1||residual.elements()!=1||steps.integers().front()<=0||residual.doubles().front()<0.)throw std::invalid_argument("invalid settle metadata");
    if(state.schema_version==1){
        const auto& meta=io::as_object(state.metadata,"state metadata");
        if(io::as_string(field(meta,"provider_identity"),"provider_identity")!=context.provider_identity||io::as_string(field(meta,"model_sha256"),"model_sha256")!=context.model_sha256)
            throw std::invalid_argument("equilibrium state engine/provider/model identity differs");
    }
}
std::vector<std::string> rider_keys(){std::vector<std::string> keys;for(auto const name:writers::kSupportNames)for(int i=0;i<2;++i)keys.push_back(std::string(name)+":"+std::to_string(i));return keys;}

struct Material {
    drivetrain::DriveSnapshot drive;
    std::optional<std::pair<writers::RiderContactsState,std::optional<writers::RiderContactsProbe>>> rider;
    std::optional<::rider::SpindleController::State> controller;
    std::vector<std::string> tire_names;
    std::vector<double> tire_row;
};
std::unique_ptr<Material> current_material(const SetupContext& context){
    auto value=std::make_unique<Material>();value->drive=context.stepper.drive().state();
    if(context.stepper.has_tire()){value->tire_names=context.stepper.tire_state_names();value->tire_row=context.stepper.tire_state();}
    if(context.stepper.has_rider_contacts())value->rider=std::pair{context.stepper.rider_contacts().state(),context.stepper.rider_contacts().probe()};
    if(context.physical.rider_present())value->controller=context.physical.rider_control().state();
    return value;
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::unique_ptr<Material> decode_material(const SetupContext& context,const StateFile& state){
    auto storage=current_material(context);auto& next=*storage;
    if(const auto* encoded=find(state,"state_native_drive")){(void)encoded;next.drive=drivetrain::decode_drive_snapshot(io::parse_recording_json(text(state,"state_native_drive")));}
    else for(const auto& pair:{std::pair{"ideal_hub",&next.drive.ideal_hub},std::pair{"clutch",&next.drive.clutch}}){
        auto& field=*pair.second;if(!field)continue;
        const std::string name="state_drive_"+std::string(pair.first)+"_boundary";const double value=doubles(state,name,{1}).front();
        field->boundary=value;field->range[1]=value;
    }
    if(context.stepper.has_rider_contacts()){
        if(find(state,"state_native_rider_contacts"))next.rider=::rider::decode_rider_contacts_state(io::parse_recording_json(text(state,"state_native_rider_contacts")),context.stepper.rider_contacts());
        else{
            const auto keys=rider_keys();const auto& names=state_array(state,"state_rider_keys");shape(names,{keys.size()},"state_rider_keys");if(names.strings()!=keys)throw std::invalid_argument("equilibrium rider material keys differ");
            const auto xi=doubles(state,"state_rider_xi",{keys.size()}),tangent=doubles(state,"state_rider_tangent",{keys.size(),3},true),grip=doubles(state,"state_rider_grip_xi",{2,3});
            const auto valid=bools(state,"state_rider_tangent_valid",{keys.size()}),enabled=bools(state,"state_rider_enabled",{writers::kContactCount});
            if(!next.rider)throw std::invalid_argument("rider contact material requires an engaged rider");
            auto& r=next.rider->first;
            for(std::size_t i=0;i<keys.size();++i){r.supports[i].xi=xi[i];r.supports[i].tangent.reset();if(valid[i]){::rider::Vec3 v{};for(std::size_t j=0;j<3;++j){v[j]=tangent[3*i+j];if(!std::isfinite(v[j]))throw std::invalid_argument("nonfinite active rider tangent");}r.supports[i].tangent=v;}}
            for(std::size_t i=0;i<writers::kContactCount;++i)r.enabled[i]=enabled[i];
            for(std::size_t i=0;i<2;++i)for(std::size_t j=0;j<3;++j)r.grip_xi_local[i][j]=grip[3*i+j];
            r.pending_release_loss_j=doubles(state,"state_rider_pending_release_loss",{1}).front();
        }
        if(find(state,"state_native_grip_anchors")||find(state,"state_native_grip_anchors_valid")) {
            const auto anchors=doubles(state,"state_native_grip_anchors",{2,3});
            const auto present=bools(state,"state_native_grip_anchors_valid",{2});
            for(std::size_t side=0;side<2;++side){auto& anchor=next.rider->first.grip_anchor_local[side];anchor.reset();if(present[side])anchor=::rider::Vec3{anchors[3*side],anchors[3*side+1],anchors[3*side+2]};}
        }
    }else for(const auto& [name,array]:state.arrays){(void)array;if(name.starts_with("state_rider_")||name=="state_native_rider_contacts")throw std::invalid_argument("unexpected rider material state");}
    if(find(state,"state_native_rider_controller")){
        if(!context.physical.rider_present())throw std::invalid_argument("unexpected rider controller state");
        next.controller=::rider::decode_spindle_state(io::parse_recording_json(text(state,"state_native_rider_controller")),"state.rider_controller");
    }
    if(context.stepper.has_tire()){
        if(find(state,"state_tire_wire_names")){
            next.tire_names=state_array(state,"state_tire_wire_names").strings();shape(state_array(state,"state_tire_wire_names"),{next.tire_names.size()},"state_tire_wire_names");
            next.tire_row=doubles(state,"state_tire_wire_row",{next.tire_names.size()},true);
        }else{
            if(context.stepper.tire_writer().is_pneumatic())throw std::invalid_argument("pneumatic equilibrium state requires canonical wire");
            const auto& sides_array=state_array(state,"state_tire_sides");const auto sides=sides_array.strings();shape(sides_array,{sides.size()},"state_tire_sides");
            const auto xi=doubles(state,"state_tire_xi",{sides.size()}),tangent=doubles(state,"state_tire_tangent",{sides.size(),3},true);const auto valid=bools(state,"state_tire_tangent_valid",{sides.size()});
            const bool distributed=context.stepper.tire_writer().is_distributed();next.tire_row.assign(next.tire_names.size(),std::numeric_limits<double>::quiet_NaN());
            std::vector<double> point,segment,center;std::vector<bool> point_valid,center_valid;
            if(!distributed){if(sides!=std::vector<std::string>{"\"front\"","\"rear\""})throw std::invalid_argument("equilibrium tire side order differs");point=doubles(state,"state_tire_point",{2,3},true);segment=doubles(state,"state_tire_segment",{2});center=doubles(state,"state_tire_center",{2,3},true);point_valid=bools(state,"state_tire_point_valid",{2});center_valid=bools(state,"state_tire_center_valid",{2});}
            std::set<std::string> assigned;
            auto const put=[&](const std::string& key,double value){const auto it=std::ranges::find(next.tire_names,key);if(it==next.tire_names.end())throw std::invalid_argument("foreign tire station/column "+key);next.tire_row[static_cast<std::size_t>(std::distance(next.tire_names.begin(),it))]=value;};
            for(std::size_t i=0;i<sides.size();++i){
                const auto decoded=io::parse_json(sides[i]);std::string prefix;
                if(distributed){const auto& side=io::as_array(decoded,"station side");if(side.size()!=2)throw std::invalid_argument("invalid tire station key");const std::string wheel=io::as_string(side[0],"station wheel");const auto index=io::as_int64(side[1],"station index");if((wheel!="front"&&wheel!="rear")||index<0||index>=context.physics.tires.distributed.station_count)throw std::invalid_argument("foreign tire station");prefix=std::format("{}.s{:03d}",wheel,index);}
                else prefix=io::as_string(decoded,"tire side");
                if(!assigned.insert(prefix).second)throw std::invalid_argument("duplicate tire station");
                put(prefix+".xi",xi[i]);
                for(std::size_t j=0;j<3;++j){const double value=valid[i]?tangent[3*i+j]:std::numeric_limits<double>::quiet_NaN();if(valid[i]&&!std::isfinite(value))throw std::invalid_argument("nonfinite active tire tangent");put(prefix+".tangent."+std::to_string(j),value);}
                if(!distributed){if(std::trunc(segment[i])!=segment[i]||(valid[i]&&(!point_valid[i]||segment[i]<0.)))throw std::invalid_argument("invalid tire branch memory");put(prefix+".segment",segment[i]<0.?std::numeric_limits<double>::quiet_NaN():segment[i]);
                    for(std::size_t j=0;j<3;++j){const double pv=point_valid[i]?point[3*i+j]:std::numeric_limits<double>::quiet_NaN(),cv=center_valid[i]?center[3*i+j]:std::numeric_limits<double>::quiet_NaN();if((point_valid[i]&&!std::isfinite(pv))||(center_valid[i]&&!std::isfinite(cv)))throw std::invalid_argument("nonfinite tire geometry memory");put(prefix+".point."+std::to_string(j),pv);put(prefix+".center."+std::to_string(j),cv);}}
            }
        }
    }else for(const auto& [name,array]:state.arrays){(void)array;if(name.starts_with("state_tire_"))throw std::invalid_argument("unexpected tire material state");}
    return storage;
}
NATIVE_DIAG_POP
void install_material(SetupContext& context,Material& state){
    // These stages leave every live material owner unchanged. The drive's
    // existing restore completes its own validation/allocation before it
    // writes model rows; every following material commit is no-throw.
    if(context.stepper.has_tire())context.stepper.tire_writer().prepare_state(state.tire_names,state.tire_row);
    if(state.controller){state.controller->activation_time_s.reset();context.physical.rider_control().validate_state(*state.controller);}
    context.stepper.mutate([&]{context.stepper.drive().restore(state.drive);});
    if(context.stepper.has_tire())context.stepper.tire_writer().commit_prepared_state();
    if(state.controller)context.physical.rider_control().commit_state(std::move(*state.controller));
    if(state.rider)context.stepper.rider_contacts().assign_state(std::move(state.rider.value().first),std::move(state.rider.value().second));
    context.stepper.drive().restart_clock();
    if(context.stepper.has_rider_contacts())context.stepper.rider_contacts_restart_clock();
    context.stepper.note_posvel_stale();
}
std::string context_fingerprint(const SetupContext& context,bool legacy){
    if(legacy&&(!context.legacy_source_identity||!context.legacy_track_identity))
        throw std::invalid_argument("historical NPZ validation requires explicit legacy source provenance");
    const auto& identity=io::as_object(context.identity,"equilibrium identity");
    io::Value physics=plain(configuration::to_value(context.physics));auto& fields=std::get<io::Value::Object>(physics.data);
    std::erase_if(fields,[](const auto& field){return field.first=="equilibrium_cache_enabled"||field.first=="equilibrium_state_file";});
    io::Value::Object payload{{"version",io::Value{io::Number{"4"}}},{"source",io::Value{legacy?*context.legacy_source_identity:context.source_identity}},{"mujoco",io::Value{std::string(mj_versionString())}},{"physics",std::move(physics)},
        {"specs",plain(field(identity,"specs"))},{"mass_specs",plain(field(identity,"mass_specs"))},{"rider",plain(field(identity,"rider"))},{"track",plain(legacy?*context.legacy_track_identity:field(identity,"track"))},{"start_x_m",number(context.start_x_m)}};
    if(!legacy){payload.emplace_back("native_equilibrium_schema",io::Value{io::Number{"1"}});payload.emplace_back("provider_identity",io::Value{context.provider_identity});payload.emplace_back("model_sha256",io::Value{context.model_sha256});}
    const io::Value identity_payload{std::move(payload)};
    const auto json=legacy?detail::source_json(identity_payload):io::canonical_json(identity_payload);io::Sha256 digest;digest.update(std::as_bytes(std::span{json}));
    const mjModel* m=context.stepper.model();
    auto const add=[&](std::string_view name,const mjtNum* p,mjtSize n,std::vector<std::size_t> shape){const auto values=std::views::counted(p,n);update(digest,name,io::double_array(values,std::move(shape)),"float64");};
    add("qpos0",m->qpos0,m->nq,{static_cast<std::size_t>(m->nq)});
    add("body_mass",m->body_mass,m->nbody,{static_cast<std::size_t>(m->nbody)});
    add("body_ipos",m->body_ipos,3*m->nbody,{static_cast<std::size_t>(m->nbody),3});
    add("body_inertia",m->body_inertia,3*m->nbody,{static_cast<std::size_t>(m->nbody),3});
    add("geom_pos",m->geom_pos,3*m->ngeom,{static_cast<std::size_t>(m->ngeom),3});
    add("geom_size",m->geom_size,3*m->ngeom,{static_cast<std::size_t>(m->ngeom),3});
    add("jnt_range",m->jnt_range,2*m->njnt,{static_cast<std::size_t>(m->njnt),2});
    io::NpyArray hfield{.dtype="<f4",.shape={static_cast<std::size_t>(m->nhfielddata)},.fortran_order=false,.bytes={}};
    for(float const value:std::views::counted(m->hfield_data,m->nhfielddata)){std::uint32_t bits=std::bit_cast<std::uint32_t>(value);for(int i=0;i<4;++i){hfield.bytes.push_back(static_cast<std::byte>(bits&255U));bits>>=8U;}}
    update(digest,"hfield_data",hfield,"float32");update(digest,"timestep",io::double_array(std::array<double,1>{m->opt.timestep},{1}),"float64");return hex(digest.finish());
}
} // namespace

std::string fingerprint(const SetupContext& context){return context_fingerprint(context,false);}
std::string legacy_equilibrium_fingerprint(const SetupContext& context){return context_fingerprint(context,true);}

StateFile capture_state(const SetupContext& context,std::int64_t steps,double residual){
    if(steps<=0||!std::isfinite(residual)||residual<0.)throw std::invalid_argument("invalid equilibrium capture metadata");
    StateFile out;out.metadata=io::Value{io::Value::Object{{"provider_identity",io::Value{context.provider_identity}},{"model_sha256",io::Value{context.model_sha256}},{"source_identity",io::Value{context.source_identity}},{"mujoco",io::Value{std::string(mj_versionString())}},{"inputs",context.identity}}};
    out.arrays.emplace_back("qpos",io::double_array(context.stepper.qpos(),{context.stepper.qpos().size()}));out.arrays.emplace_back("steps",io::integer_array(std::array<std::int64_t,1>{steps},{}));put_number(out,"residual_qacc",residual);put_text(out,"fingerprint_sha256",fingerprint(context));
    const auto storage=current_material(context);const auto& material=*storage;put_text(out,"state_native_drive",io::recording_json(drivetrain::encode_drive_snapshot(material.drive)));
    for(const auto& pair:{std::pair{"ideal_hub",&material.drive.ideal_hub},std::pair{"clutch",&material.drive.clutch}}){auto& slot=*pair.second;if(slot){if(!slot->boundary)throw std::invalid_argument("unprepared drive boundary");out.arrays.emplace_back("state_drive_"+std::string(pair.first)+"_boundary",io::double_array(std::array<double,1>{*slot->boundary},{1}));}}
    if(material.rider){
        const auto& r=material.rider.value().first;put_text(out,"state_native_rider_contacts",io::recording_json(::rider::encode_rider_contacts_state(r,material.rider.value().second)));
        const auto keys=rider_keys();std::vector<double> xi,tangent,grip;std::vector<bool> valid,enabled;
        for(const auto& support:r.supports){xi.push_back(support.xi);valid.push_back(support.tangent.has_value());for(std::size_t j=0;j<3;++j)tangent.push_back(support.tangent?(*support.tangent)[j]:0.);} // NOLINT(bugprone-unchecked-optional-access) the ternary guards engagement
        for(const auto& hand:r.grip_xi_local)for(double const value:hand)grip.push_back(value);
        for(bool const value:r.enabled)enabled.push_back(value);
        out.arrays.emplace_back("state_rider_keys",io::string_array(keys,{keys.size()}));out.arrays.emplace_back("state_rider_xi",io::double_array(xi,{xi.size()}));out.arrays.emplace_back("state_rider_tangent",io::double_array(tangent,{keys.size(),3}));out.arrays.emplace_back("state_rider_tangent_valid",io::bool_array(valid,{valid.size()}));out.arrays.emplace_back("state_rider_grip_xi",io::double_array(grip,{2,3}));out.arrays.emplace_back("state_rider_enabled",io::bool_array(enabled,{enabled.size()}));out.arrays.emplace_back("state_rider_pending_release_loss",io::double_array(std::array<double,1>{r.pending_release_loss_j},{1}));
    }
    if(material.controller)put_text(out,"state_native_rider_controller",io::recording_json(::rider::encode_spindle_state(*material.controller)));
    if(context.stepper.has_tire()){out.arrays.emplace_back("state_tire_wire_names",io::string_array(material.tire_names,{material.tire_names.size()}));out.arrays.emplace_back("state_tire_wire_row",io::double_array(material.tire_row,{material.tire_row.size()}));}
    const mjModel* m=context.stepper.model();const mjData* d=context.stepper.data();
    std::vector<double> integration(static_cast<std::size_t>(engine::state_size(m,mjSTATE_INTEGRATION)));engine::get_state(m,d,integration.data(),mjSTATE_INTEGRATION);out.arrays.emplace_back("integration_state",io::double_array(integration,{integration.size()}));
    for(const auto& item:{std::tuple{"dof_frictionloss",m->dof_frictionloss,m->nv},std::tuple{"site_pos",m->site_pos,3*m->nsite},std::tuple{"tendon_stiffness",m->tendon_stiffness,m->ntendon},std::tuple{"tendon_damping",m->tendon_damping,m->ntendon},std::tuple{"tendon_lengthspring",m->tendon_lengthspring,2*m->ntendon},std::tuple{"tendon_range",m->tendon_range,2*m->ntendon},std::tuple{"wrap_prm",m->wrap_prm,m->nwrap}}){const auto& [name,pointer,width]=item;out.arrays.emplace_back("model_"+std::string(name),io::double_array(std::views::counted(pointer,width),{static_cast<std::size_t>(width)}));}
    return out;
}

std::filesystem::path equilibrium_cache_path(const SetupContext& context){
    // NOLINTNEXTLINE(concurrency-mt-unsafe) matches the single-threaded reference semantics
    std::filesystem::path directory;if(context.cache_directory)directory=*context.cache_directory;else if(const char* value=std::getenv("BIKE_SIM_EQUILIBRIUM_CACHE_DIR");value&&*value)directory=value;else directory=std::filesystem::temp_directory_path()/"bike_sim_equilibrium";
    return directory/(fingerprint(context)+".npz");
}
std::optional<StateFile> load_equilibrium_cache(const SetupContext& context){try{auto state=read_state_file(equilibrium_cache_path(context));settle_metadata(context,state);return state;}catch(const std::bad_alloc&){throw;}catch(const std::exception&){return std::nullopt;}}
StateFile load_equilibrium_state(const SetupContext& context,const std::filesystem::path& file){
    try{auto state=read_state_file(file);const auto expected=state.schema_version==0?legacy_equilibrium_fingerprint(context):fingerprint(context);if(!find(state,"fingerprint_sha256")||fingerprint(state)!=expected)throw std::invalid_argument("fingerprint mismatch: pose belongs to another configuration");settle_metadata(context,state);return state;}
    catch(const std::bad_alloc&){throw;}
    catch(const std::exception& error){throw std::invalid_argument("equilibrium state file "+file.string()+": "+error.what());}
}
// NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
void save_equilibrium_cache(const SetupContext& context,const StateFile& state){try{write_state_file(equilibrium_cache_path(context),state);}catch(const std::bad_alloc&){throw;}catch(const std::exception&){/* Optional cache write failure does not fail a settled run. */}}

void restore_equilibrium_state(SetupContext& context,const StateFile& state){
    require_initializing(context);
    if(context.stepper.time()!=0.)throw std::invalid_argument("equilibrium state restore requires a fresh t=0 setup");
    if(state.schema_version==1)settle_metadata(context,state);
    const auto q=doubles(state,"qpos",{context.stepper.qpos().size()});
    auto next=decode_material(context,state);
    mjModel* m=context.stepper.model();mjData* d=context.stepper.data();
    std::vector<double> integration(static_cast<std::size_t>(engine::state_size(m,mjSTATE_INTEGRATION)));engine::get_state(m,d,integration.data(),mjSTATE_INTEGRATION);
    const auto target_integration=state.schema_version==1?doubles(state,"integration_state",{integration.size()}):integration;
    if(!target_integration.empty()&&target_integration.front()<0.)throw std::invalid_argument("negative equilibrium integration time");
    // All shape/finite/codec checks occur before any mutation. Model values
    // are staged too; callbacks cannot turn an invalid payload into a partial
    // coordinate restore. Engine failures are deliberately not rolled back.
    struct ModelRow{double* pointer;std::vector<double> after;};std::vector<ModelRow> rows;
    if(state.schema_version==1)for(const auto& item:{std::tuple{"dof_frictionloss",m->dof_frictionloss,m->nv},std::tuple{"site_pos",m->site_pos,3*m->nsite},std::tuple{"tendon_stiffness",m->tendon_stiffness,m->ntendon},std::tuple{"tendon_damping",m->tendon_damping,m->ntendon},std::tuple{"tendon_lengthspring",m->tendon_lengthspring,2*m->ntendon},std::tuple{"tendon_range",m->tendon_range,2*m->ntendon},std::tuple{"wrap_prm",m->wrap_prm,m->nwrap}}){const auto& [name,pointer,width]=item;auto values=doubles(state,"model_"+std::string(name),{static_cast<std::size_t>(width)});if((std::string_view(name)=="dof_frictionloss"||std::string_view(name)=="tendon_stiffness"||std::string_view(name)=="tendon_damping")&&std::ranges::any_of(values,[](double v){return v<0.;}))throw std::invalid_argument("negative equilibrium model coefficient");rows.push_back({.pointer=pointer,.after=std::move(values)});}
    install_material(context,*next);
    context.stepper.mutate([&]{
        for(auto& row:rows)std::ranges::copy(row.after,row.pointer);
        engine::set_state(m,d,target_integration.data(),mjSTATE_INTEGRATION);
        std::ranges::copy(q,std::views::counted(d->qpos,m->nq).begin());
        std::ranges::fill(std::views::counted(d->qvel,m->nv),0.);
        d->time=0.;
        context.stepper.note_posvel_stale();
    });
}
} // namespace bike::initialization
