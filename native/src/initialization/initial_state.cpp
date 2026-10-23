#include "context.hpp"
#include "../diag.hpp"
#include "state_file.hpp"
#include "source_json.hpp"
#include "../drivetrain/state_codec.hpp"
#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/step.hpp"
#include "../stepper.hpp"
#include "../tyre/profile.hpp"
#include "../tyre/station.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/suspension.hpp"
#include "../numeric_sincos.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace bike::initialization {
namespace {
namespace bio=bike::io;
bio::Value real(double value){if(!std::isfinite(value))throw std::invalid_argument("nonfinite initial-state value");return bio::Value{bio::Number{configuration::detail::repr_double(value)}};}
bio::Value integer(std::int64_t value){return bio::Value{bio::Number{std::to_string(value)}};}
bio::Value string(std::string value){return bio::Value{std::move(value)};}
const bio::Value& field(const bio::Value::Object& object,std::string_view key){const auto* value=bio::find_member(object,key);if(!value)throw std::invalid_argument("initial state: missing "+std::string(key));return *value;}
void put(bio::Value::Object& object,std::string name,bio::Value value){for(auto& [key,item]:object)if(key==name){item=std::move(value);return;}object.emplace_back(std::move(name),std::move(value));}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
bio::Value plain(bio::Value value){if(auto* o=std::get_if<bio::Value::Object>(&value.data)){if(o->size()==1&&o->front().first=="$tuple")return plain(o->front().second);for(auto& p:*o)p.second=plain(std::move(p.second));}else if(auto* a=std::get_if<bio::Value::Array>(&value.data))for(auto& p:*a)p=plain(std::move(p));return value;}
std::string hash(const bio::Value& value){const auto json=detail::source_json(value);return bio::sha256(std::as_bytes(std::span{json}));}
bio::Value numbers(std::span<const double> data){bio::Value::Array values;values.reserve(data.size());for(double const value:data)values.push_back(real(value));return bio::Value{std::move(values)};}
std::vector<double> vector(const bio::Value& value,std::string_view name,std::optional<std::size_t> size=std::nullopt){const auto& array=bio::as_array(value,name);if(size&&array.size()!=*size)throw std::invalid_argument("invalid initial-state coordinate array "+std::string(name));std::vector<double> result;result.reserve(array.size());for(const auto& item:array)result.push_back(bio::as_double(item,name));return result;}
bio::Value matrix(std::span<const double> data,std::size_t columns){bio::Value::Array result;for(std::size_t row=0;row<data.size()/columns;++row)result.push_back(numbers(data.subspan(row*columns,columns)));return bio::Value{std::move(result)};}
bio::Value optional(std::optional<double> value){return value?real(*value):bio::Value{};}
double height(std::span<const std::array<double,2>> vertices,double x){if(x<=vertices.front()[0])return vertices.front()[1];if(x>=vertices.back()[0])return vertices.back()[1];const auto it=std::ranges::lower_bound(vertices,x,{},[](const auto& a){return a[0];});const auto i=static_cast<std::size_t>(std::ranges::distance(vertices.begin(),it))-1;return biketyre::np_interp(x,std::array{vertices[i][0],vertices[i+1][0]},std::array{vertices[i][1],vertices[i+1][1]});}
void guard(const SetupContext& context){if(context.physical.step()!=0||context.stepper.time()!=0.)throw std::invalid_argument("physical initial states may only be transferred before the first step");}

std::string contract(const SetupContext& context,std::optional<int> station_count=std::nullopt){
    bio::Value physics=plain(configuration::to_value(context.physics));auto& config=std::get<bio::Value::Object>(physics.data);
    if(station_count){auto tires=field(config,"tires");auto distributed=field(std::get<bio::Value::Object>(tires.data),"distributed");put(std::get<bio::Value::Object>(distributed.data),"station_count",integer(*station_count));put(std::get<bio::Value::Object>(tires.data),"distributed",std::move(distributed));put(config,"tires",std::move(tires));}
    std::erase_if(config,[](const auto& p){return p.first=="timestep_s"||p.first=="equilibrium_cache_enabled"||p.first=="equilibrium_state_file"||p.first=="equilibrium_refine_after_s"||p.first=="equilibrium_refine_period_s";});
    const mjModel* m=context.stepper.model();bio::Value::Array joints;for(int i=0;i<m->njnt;++i){const char* name=mj_id2name(m,mjOBJ_JOINT,i);joints.push_back(name?string(name):bio::Value{});}
    bio::Value::Object compiled;
    auto const add=[&](std::string name,const double* pointer,mjtSize count,std::optional<std::size_t> columns=std::nullopt){const auto values=std::span<const double>{std::views::counted(pointer,count)};compiled.emplace_back(std::move(name),columns?matrix(values,*columns):numbers(values));};
    add("qpos0",m->qpos0,m->nq);add("body_mass",m->body_mass,m->nbody);add("body_inertia",m->body_inertia,3*m->nbody,3);add("body_ipos",m->body_ipos,3*m->nbody,3);add("body_pos",m->body_pos,3*m->nbody,3);add("body_quat",m->body_quat,4*m->nbody,4);add("jnt_axis",m->jnt_axis,3*m->njnt,3);add("jnt_pos",m->jnt_pos,3*m->njnt,3);add("jnt_range",m->jnt_range,2*m->njnt,2);add("dof_armature",m->dof_armature,m->nv);add("actuator_gear",m->actuator_gear,6*m->nu,6);add("actuator_ctrlrange",m->actuator_ctrlrange,2*m->nu,2);add("eq_solref",m->eq_solref,2*m->neq,2);add("eq_solimp",m->eq_solimp,5*m->neq,5);
    bio::Value::Array limited;for(auto const value:std::views::counted(m->jnt_limited,m->njnt))limited.push_back(bio::Value{value!=0});compiled.emplace_back("jnt_limited",bio::Value{std::move(limited)});
    const auto& identity=bio::as_object(context.identity,"initial state identity");
    if(context.topology) {
        if(!context.terrain3d)throw std::invalid_argument("initial_state.spatial.terrain: physical source missing");
        compiled.emplace_back("topology_fingerprint",string(context.topology->fingerprint_hex));
        compiled.emplace_back("physical_terrain_sha256",string(context.terrain3d->fingerprint_hex()));
    }
    return hash(bio::Value{bio::Value::Object{{"physics",std::move(physics)},{"specs",plain(field(identity,"specs"))},{"mass",plain(field(identity,"mass_specs"))},{"rider",plain(field(identity,"rider"))},{"start_x_m",real(context.start_x_m)},{"joints",bio::Value{std::move(joints)}},{"compiled",bio::Value{std::move(compiled)}},{"envelope_sha256",context.physics.articulated.joint_envelope_path?string(platform::file_sha256(*context.physics.articulated.joint_envelope_path)):bio::Value{}}}});
}

bio::Value shape_values(const bio::NpyArray& array){
    bio::Value::Array flat;
    if(array.dtype.find('U')!=std::string::npos||array.dtype.find('S')!=std::string::npos)for(const auto& value:array.strings())flat.push_back(string(value));
    else if(array.dtype.size()>1&&array.dtype[1]=='b')for(bool value:array.booleans())flat.emplace_back(value);
    else if(array.dtype.size()>1&&(array.dtype[1]=='i'||array.dtype[1]=='u'))for(auto const value:array.integers())flat.push_back(integer(value));
    else for(double const value:array.doubles(true))flat.push_back(std::isnan(value)?bio::Value{}:real(value));
    std::size_t at=0;
    std::function<bio::Value(std::size_t)> nested=[&](std::size_t depth)->bio::Value{if(depth==array.shape.size()){if(at>=flat.size())throw std::invalid_argument("invalid material array size");return std::move(flat[at++]);}bio::Value::Array values;values.reserve(array.shape[depth]);for(std::size_t i=0;i<array.shape[depth];++i)values.push_back(nested(depth+1));return bio::Value{std::move(values)};};
    return nested(0);
}
bio::Value material_document(const StateFile& state){bio::Value::Object result;for(const auto& [name,array]:state.arrays)if(name.starts_with("state_")){bio::Value::Array shape;for(std::size_t const x:array.shape)shape.push_back(integer(static_cast<std::int64_t>(x)));result.emplace_back(name,bio::Value{bio::Value::Object{{"dtype",string(array.dtype)},{"shape",bio::Value{std::move(shape)}},{"values",shape_values(array)}}});}return bio::Value{std::move(result)};}
bio::NpyArray material_array(const bio::Value& document){
    const auto& data=bio::as_object(document,"material array");const std::string dtype=bio::as_string(field(data,"dtype"),"material dtype");std::vector<std::size_t> shape;std::size_t elements=1;
    for(const auto& entry:bio::as_array(field(data,"shape"),"material shape")){const auto n=bio::as_int64(entry,"material dimension");if(n<0||std::cmp_greater(n,bio::kMaxNumpyBytes)||shape.size()>=16||(n!=0&&elements>bio::kMaxNumpyBytes/static_cast<std::size_t>(n)))throw std::invalid_argument("oversized material shape");shape.push_back(static_cast<std::size_t>(n));elements*=static_cast<std::size_t>(n);}
    bio::Value::Array flat;flat.reserve(elements);std::function<void(const bio::Value&,std::size_t)> flatten=[&](const bio::Value& value,std::size_t depth){if(depth==shape.size()){flat.push_back(value);return;}const auto& array=bio::as_array(value,"material values");if(array.size()!=shape[depth])throw std::invalid_argument("material shape mismatch");for(const auto& item:array)flatten(item,depth+1);};flatten(field(data,"values"),0);
    if(dtype=="bool"||dtype=="|b1"){std::vector<bool> values;for(const auto& item:flat)values.push_back(bio::as_bool(item,"material bool"));return bio::bool_array(values,std::move(shape));}
    if(dtype=="int64"||dtype=="<i8"||dtype==">i8"||dtype=="int32"||dtype=="<i4"){std::vector<std::int64_t> values;for(const auto& item:flat)values.push_back(bio::as_int64(item,"material integer"));return bio::integer_array(values,std::move(shape));}
    if(dtype.find('U')!=std::string::npos||dtype.find('S')!=std::string::npos){std::vector<std::string> values;for(const auto& item:flat)values.push_back(bio::as_string(item,"material string"));return bio::string_array(values,std::move(shape));}
    if(dtype!="float64"&&dtype!="float32"&&dtype!="<f8"&&dtype!=">f8"&&dtype!="<f4"&&dtype!="f8"&&dtype!="f4")throw std::invalid_argument("unsupported initial material dtype");
    std::vector<double> values;for(const auto& item:flat)values.push_back(bio::is_null(item)?std::numeric_limits<double>::quiet_NaN():bio::as_double(item,"material float"));return bio::double_array(values,std::move(shape));
}
StateFile material_state(const bio::Value::Object& payload){StateFile state;state.schema_version=0;const auto q=vector(field(payload,"qpos"),"qpos"),v=vector(field(payload,"qvel"),"qvel");state.arrays.emplace_back("qpos",bio::double_array(q,{q.size()}));state.arrays.emplace_back("qvel",bio::double_array(v,{v.size()}));for(const auto& [name,value]:bio::as_object(field(payload,"material"),"material")){if(!name.starts_with("state_"))throw std::invalid_argument("unknown initial material member");state.arrays.emplace_back(name,material_array(value));}return state;}
void put_array(StateFile& state,std::string name,bio::NpyArray array){for(auto& [key,value]:state.arrays)if(key==name){value=std::move(array);return;}state.arrays.emplace_back(std::move(name),std::move(array));}
std::optional<double> optional_real(const bio::Value& value,std::string_view name){return bio::is_null(value)?std::nullopt:std::optional<double>(bio::as_double(value,name));}

bio::Value equilibrium_value(const EquilibriumResult& r){return bio::Value{bio::Value::Object{{"fork_travel_mm",real(r.fork_travel_mm)},{"shock_stroke_mm",real(r.shock_stroke_mm)},{"rear_travel_mm",real(r.rear_travel_mm)},{"root_z_m",real(r.root_z_m)},{"pitch_rad",real(r.pitch_rad)},{"steps",integer(r.steps)},{"residual_qacc",real(r.residual_qacc)},{"cache_hit",bio::Value{r.cache_hit}},{"initial_crank_phase_rad",real(r.initial_crank_phase_rad)},{"settled_crank_phase_rad",real(r.settled_crank_phase_rad)}}};}
EquilibriumResult equilibrium(const bio::Value& value){const auto& o=bio::as_object(value,"equilibrium");EquilibriumResult r;r.fork_travel_mm=bio::as_double(field(o,"fork_travel_mm"),"fork travel");r.shock_stroke_mm=bio::as_double(field(o,"shock_stroke_mm"),"shock stroke");r.rear_travel_mm=bio::as_double(field(o,"rear_travel_mm"),"rear travel");r.root_z_m=bio::as_double(field(o,"root_z_m"),"root z");r.pitch_rad=bio::as_double(field(o,"pitch_rad"),"pitch");r.steps=bio::as_int64(field(o,"steps"),"steps");r.residual_qacc=bio::as_double(field(o,"residual_qacc"),"residual");if(r.steps<=0||r.residual_qacc<0.)throw std::invalid_argument("invalid transferred equilibrium evidence");if(const auto* p=bio::find_member(o,"initial_crank_phase_rad"))r.initial_crank_phase_rad=bio::as_double(*p,"crank phase");if(const auto* p=bio::find_member(o,"settled_crank_phase_rad"))r.settled_crank_phase_rad=bio::as_double(*p,"settled crank phase");return r;}
StateFile stage_transfer_material(const SetupContext&,const bio::Value::Object&);
void validated(const SetupContext& context,const bio::Value::Object& p){
    guard(context);if(bio::as_int64(field(p,"schema_version"),"schema_version")!=1||bio::as_string(field(p,"contract_sha256"),"contract_sha256")!=contract(context))throw std::invalid_argument("initial-state physical model contract differs (only timestep/road mesh may vary)");
    const auto q=vector(field(p,"qpos"),"qpos",context.stepper.qpos().size());const auto v=vector(field(p,"qvel"),"qvel",context.stepper.qvel().size());(void)vector(field(p,"qacc_warmstart"),"qacc_warmstart",context.stepper.qvel().size());
    if(context.topology) {
        const auto* model=context.stepper.model();
        for(int joint=0;joint<model->njnt;++joint) {
            const int type=model->jnt_type[joint];
            if(type!=mjJNT_FREE && type!=mjJNT_BALL)continue;
            const auto start=static_cast<std::size_t>(model->jnt_qposadr[joint])+(type==mjJNT_FREE?3U:0U);
            if(start>q.size() || q.size()-start<4)throw std::invalid_argument("initial_state.qpos.quaternion: invalid span");
            const double norm=std::hypot(std::hypot(q[start],q[start+1]),std::hypot(q[start+2],q[start+3]));
            if(std::abs(norm-1.)>1e-9)throw std::invalid_argument("initial_state.qpos.quaternion: expected unit quaternion");
        }
        // Validate the final overlaid drive/controller/anchors, not just the
        // inner material document, before prepare resets any live owner.
        validate_equilibrium_material(context,stage_transfer_material(context,p),q,v);
    }
    const auto x=vector(field(p,"road_x"),"road_x"),z=vector(field(p,"road_z"),"road_z",x.size());if(x.empty()||x.size()>1048576)throw std::invalid_argument("invalid initial support road");
    for(std::size_t i=0;i<x.size();++i)if((i>0&&x[i]<=x[i-1])||std::abs(height(context.physical.terrain_vertices(),x[i])-z[i])>2e-7)throw std::invalid_argument("initial support road differs; solve an independent equilibrium");
}
void stage_drive_material(const SetupContext& context,const bio::Value::Object& p,StateFile& material){
    // NOLINTNEXTLINE(modernize-make-unique) make_unique's inlined forwarding path overflows the 8192-byte frame budget here
    auto drive=std::unique_ptr<drivetrain::DriveSnapshot>{new drivetrain::DriveSnapshot(context.stepper.drive().state())};
    for(const auto& [name,array]:material.arrays)if(name=="state_native_drive"){const auto encoded=array.strings();if(encoded.size()!=1)throw std::invalid_argument("invalid initial drive document");*drive=drivetrain::decode_drive_snapshot(bio::parse_recording_json(encoded.front()));}
    const auto& source=bio::as_object(field(p,"drive"),"drive");drive->reference=optional_real(field(source,"reference"),"drive.reference");drive->psi=optional_real(field(source,"psi"),"drive.psi");
    if(bio::is_null(field(source,"angles")))drive->angles.reset();else{const auto angles=vector(field(source,"angles"),"drive.angles");if(angles.empty()||angles.size()>2)throw std::invalid_argument("invalid drive angle count");drive->angles=drivetrain::DriveAngles{};drive->angles.value().count=angles.size();std::ranges::copy(angles,drive->angles.value().values.begin());}
    for(const auto& entry:{std::pair{"ideal_boundary",&drive->ideal_hub},std::pair{"clutch_boundary",&drive->clutch}}){const auto boundary=optional_real(field(p,entry.first),entry.first);if(boundary.has_value()!=entry.second->has_value())throw std::invalid_argument("initial transmission topology differs");if(boundary){(*entry.second)->boundary=boundary;(*entry.second)->range[1]=*boundary;}} // NOLINT(bugprone-unchecked-optional-access) has_value() parity with boundary was enforced two lines up
    if(drive->hub){const auto& hub=bio::as_object(field(p,"hub"),"hub");drive->hub->boundary=optional_real(field(hub,"boundary"),"hub.boundary");drive->hub->energy_j=bio::as_double(field(hub,"energy_j"),"hub.energy_j");drive->hub->torque_nm=bio::as_double(field(hub,"torque_nm"),"hub.torque_nm");}
    put_array(material,"state_native_drive",bio::string_array(std::array<std::string,1>{bio::recording_json(drivetrain::encode_drive_snapshot(*drive))},{}));
}
void stage_controller_material(const SetupContext& context,const bio::Value::Object& p,StateFile& material){
    if(!context.physical.rider_present())return;
    auto controller=std::make_unique<::rider::SpindleController::State>(context.physical.rider_control().state());
    if(context.topology) {
        const auto encoded=state_array(material,"state_native_rider_controller").strings();
        if(encoded.size()!=1)throw std::invalid_argument("initial_state.rider_controller: producer state missing");
        *controller=::rider::decode_spindle_state(bio::parse_recording_json(encoded.front()),"initial_state.rider_controller");
    }
    const auto active=vector(field(p,"active_state"),"active_state",controller->active_state.size());controller->active_state=active;controller->activation_time_s.reset();
    put_array(material,"state_native_rider_controller",bio::string_array(std::array<std::string,1>{bio::recording_json(::rider::encode_spindle_state(*controller))},{}));
}
StateFile stage_transfer_material(const SetupContext& context,const bio::Value::Object& p){
    StateFile material=material_state(p);
    stage_drive_material(context,p,material);
    stage_controller_material(context,p,material);
    if(context.stepper.has_rider_contacts()&&!context.stepper.rider_contacts().welded_grip()){
        std::array<std::optional<::rider::Vec3>,2> anchors;
        const auto& root=bio::as_object(field(p,"grip_anchor_local"),"grip_anchor_local");
        for(std::size_t side=0;side<2;++side){
            const auto& value=field(root,writers::kSideNames[side]);
            if(!bio::is_null(value)){const auto cells=vector(value,"grip anchor",3);anchors[side]=::rider::Vec3{cells[0],cells[1],cells[2]};}
        }
        std::vector<double> values;std::vector<bool> present;
        for(const auto& anchor:anchors){present.push_back(anchor.has_value());for(std::size_t axis=0;axis<3;++axis)values.push_back(anchor?(*anchor)[axis]:0.);}
        put_array(material,"state_native_grip_anchors",bio::double_array(values,{2,3}));
        put_array(material,"state_native_grip_anchors_valid",bio::bool_array(present,{2}));
    }
    return material;
}
} // namespace

PhysicalInitialState::PhysicalInitialState(std::string json):payload_json_(std::move(json)){
    const auto p=bio::parse_json(payload_json_);const auto& o=bio::as_object(p,"physical initial state");if(bio::as_int64(field(o,"schema_version"),"schema_version")!=1)throw std::invalid_argument("unsupported physical initial-state schema");
    // Stored bytes are immutable: preserving the input's canonical/original
    // spelling also preserves the source object's SHA-256 identity.
}
bio::Value PhysicalInitialState::payload()const{return bio::parse_json(payload_json_);}
std::string PhysicalInitialState::sha256()const{return bio::sha256(std::as_bytes(std::span{payload_json_}));}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
PhysicalInitialState PhysicalInitialState::capture(const SetupContext& context){
    guard(context);if(!context.equilibrium)throw std::invalid_argument("physical initial state requires completed equilibrium evidence");
    const auto* m=context.stepper.model();const auto* d=context.stepper.data();const auto positions=std::views::counted(d->geom_xpos,3*m->ngeom);
    const double front=positions[3*static_cast<std::size_t>(context.front_geom_id)],rear=positions[3*static_cast<std::size_t>(context.rear_geom_id)],start=std::min(front,rear)-.5,stop=std::max(front,rear)+.5;
    std::vector<double> road_x(513),road_z(513);for(std::size_t i=0;i<513;++i){road_x[i]=i==512?stop:start+(stop-start)/512.*static_cast<double>(i);road_z[i]=height(context.physical.terrain_vertices(),road_x[i]);}
    const auto state=capture_state(context,context.equilibrium->steps,context.equilibrium->residual_qacc);const auto drive=context.stepper.drive().state();
    bio::Value angles;if(drive.angles)angles=numbers(std::span{drive.angles->values}.first(drive.angles->count));
    bio::Value hub;if(drive.hub)hub=bio::Value{bio::Value::Object{{"boundary",optional(drive.hub.value().boundary)},{"energy_j",real(drive.hub.value().energy_j)},{"torque_nm",real(drive.hub.value().torque_nm)}}};
    bio::Value anchors;if(context.stepper.has_rider_contacts()){bio::Value::Object entries;const auto& rider_state=context.stepper.rider_contacts().state();for(std::size_t side=0;side<2;++side){const auto& anchor=rider_state.grip_anchor_local[side];entries.emplace_back(std::string(writers::kSideNames[side]),anchor?numbers(*anchor):bio::Value{});}anchors=bio::Value{std::move(entries)};}
    const bio::Value active=context.physical.rider_present()?numbers(context.physical.rider_control().state().active_state):bio::Value{};
    bio::Value::Object p{{"schema_version",integer(1)},{"contract_sha256",string(contract(context))},{"qpos",numbers(context.stepper.qpos())},{"qvel",numbers(context.stepper.qvel())},{"qacc_warmstart",numbers(std::views::counted(d->qacc_warmstart,m->nv))},{"road_x",numbers(road_x)},{"road_z",numbers(road_z)},{"material",material_document(state)},{"equilibrium",equilibrium_value(*context.equilibrium)},
        {"drive",bio::Value{bio::Value::Object{{"reference",optional(drive.reference)},{"angles",std::move(angles)},{"psi",optional(drive.psi)}}}},
        {"ideal_boundary",drive.ideal_hub?optional(drive.ideal_hub->boundary):bio::Value{}},{"clutch_boundary",drive.clutch?optional(drive.clutch->boundary):bio::Value{}},{"hub",std::move(hub)},{"grip_anchor_local",std::move(anchors)},{"active_state",active},{"source_timestep_s",real(m->opt.timestep)},{"source_sha256",string(context.source_identity)}};
    return PhysicalInitialState(detail::source_json(bio::Value{std::move(p)}));
}

void PhysicalInitialState::restore(SetupContext& context)const{
    auto const value=payload();const auto& p=bio::as_object(value,"initial state");validated(context,p);require_initializing(context);
    StateFile material=stage_transfer_material(context,p);
    const auto q=vector(field(p,"qpos"),"qpos"),v=vector(field(p,"qvel"),"qvel"),warm=vector(field(p,"qacc_warmstart"),"qacc_warmstart");
    // Decode every field before invoking the shared transactional material
    // restore. No coordinate is written before material validation succeeds.
    restore_equilibrium_state(context,material);
    auto const* m=context.stepper.model();auto const* d=context.stepper.data();
    std::ranges::copy(q,std::views::counted(d->qpos,m->nq).begin());std::ranges::copy(v,std::views::counted(d->qvel,m->nv).begin());std::ranges::copy(warm,std::views::counted(d->qacc_warmstart,m->nv).begin());
    if(context.stepper.has_rider_contacts())context.stepper.rider_contacts_restart_clock();
    if(context.stepper.has_tire())context.stepper.tire_writer().restart_clock();
    context.stepper.drive().restart_clock();
    context.stepper.note_posvel_stale();context.stepper.forward();apply_initial_forces(context,false);context.stepper.forward();
    const double residual=acceleration_residual(context);if(context.equilibrium)context.equilibrium->initial_state_target_residual_qacc=residual;
    context.physical.contacts_=context.physical.refresh_contacts(false,std::nullopt,false);
    std::ranges::copy(warm,std::views::counted(d->qacc_warmstart,m->nv).begin());
}
EquilibriumResult PhysicalInitialState::prepare(SetupContext& context)const{
    require_initializing(context);auto const value=payload();const auto& p=bio::as_object(value,"initial state");validated(context,p);
    const auto q=vector(field(p,"qpos"),"qpos");auto* m=context.stepper.model();auto* d=context.stepper.data();
    EquilibriumResult result=equilibrium(field(p,"equilibrium"));result.cache_hit=false;result.initial_state_sha256=sha256();result.initial_state_source_sha256=bio::as_string(field(p,"source_sha256"),"source_sha256");result.initial_state_source_timestep_s=bio::as_double(field(p,"source_timestep_s"),"source timestep");if(*result.initial_state_source_timestep_s<=0.)throw std::invalid_argument("invalid source timestep");result.source_equilibrium_residual_qacc=result.residual_qacc;
    auto state=std::make_unique<runtime::StepState>(context.physical.state());state->initializing=true;context.physical.restore(*state);
    context.stepper.mutate([&]{engine::reset_data(m,d);});std::ranges::copy(q,std::views::counted(d->qpos,m->nq).begin());context.stepper.note_posvel_stale();context.stepper.forward();
    if(context.physical.rider_present()){context.stepper.mutate([&]{context.physical.rider_control().initialize(d);});std::ranges::copy(q,std::views::counted(d->qpos,m->nq).begin());context.stepper.note_posvel_stale();context.stepper.forward();if(context.stepper.has_rider_contacts())context.stepper.rider_contacts_reset();}
    context.stepper.mutate([&]{context.stepper.drive().reset();});if(context.stepper.has_tire())context.stepper.tire_writer().reset();
    context.equilibrium=result;restore(context);context.completed=true;return *context.equilibrium;
}

PhysicalInitialState PhysicalInitialState::project_stations(const SetupContext& context,int count){
    if(context.physics.tires.backend!="distributed_2d_reference")throw std::invalid_argument("station projection requires the distributed backend");
    const auto target=biketyre::station_angles(count),source_angles=biketyre::station_angles(context.physics.tires.distributed.station_count);const auto source=capture(context);auto value=source.payload();auto& p=std::get<bio::Value::Object>(value.data);
    const auto names=context.stepper.tire_state_names();const auto row=context.stepper.tire_state();
    std::vector<std::string> sides;std::vector<double> xi,tangents;std::vector<bool> valid;
    auto const lookup=[&](const std::string& name){const auto it=std::ranges::find(names,name);if(it==names.end())throw std::invalid_argument("station projection is missing canonical wire columns");const double cell=row[static_cast<std::size_t>(std::distance(names.begin(),it))];return std::isfinite(cell)?cell:0.;};
    auto const interp=[&](double angle,std::span<const double> values){const auto& a=source_angles.angles;const auto it=std::ranges::upper_bound(a,angle);const std::size_t right=it==a.end()?0:static_cast<std::size_t>(std::distance(a.begin(),it)),left=right==0?a.size()-1:right-1;const double ax=a[left],bx=right==0?a.front()+2.*std::numbers::pi:a[right];const double x=angle<ax?angle+2.*std::numbers::pi:angle;return biketyre::np_interp(x,std::array{ax,bx},std::array{values[left],values[right]});};
    const auto* m=context.stepper.model();const auto* d=context.stepper.data();const auto geom_body=std::views::counted(m->geom_bodyid,m->ngeom);const auto centers=std::views::counted(d->geom_xpos,3*m->ngeom);const auto rotations=std::views::counted(d->xmat,9*m->nbody);const auto writer=context.stepper.tire_writer();const auto& radii=writer.radii();
    for(std::size_t side=0;side<2;++side){const std::string wheel=side==0?"front":"rear";const int geom=side==0?context.front_geom_id:context.rear_geom_id;const auto body=static_cast<std::size_t>(geom_body[static_cast<std::size_t>(geom)]),g=static_cast<std::size_t>(geom);
        std::vector<double> old_xi(source_angles.angles.size());std::array<std::vector<double>,3> old_tangent;for(auto& v:old_tangent)v.resize(old_xi.size());
        for(std::size_t index=0;index<old_xi.size();++index){const auto prefix=std::format("{}.s{:03d}",wheel,index);old_xi[index]=lookup(prefix+".xi");for(std::size_t axis=0;axis<3;++axis)old_tangent[axis][index]=lookup(prefix+".tangent."+std::to_string(axis));}
        for(std::size_t index=0;index<target.angles.size();++index){const double angle=target.angles[index],shear=interp(angle,old_xi);std::array<double,3> tangent{};for(std::size_t axis=0;axis<3;++axis)tangent[axis]=interp(angle,old_tangent[axis]);const double norm=std::sqrt(tangent[0]*tangent[0]+tangent[1]*tangent[1]+tangent[2]*tangent[2]);
            const double lx=radii[side]*numeric::np_cos(angle),lz=radii[side]*numeric::np_sin(angle);const double px=centers[3*g]+rotations[9*body]*lx+rotations[9*body+2]*lz,pz=centers[3*g+2]+rotations[9*body+6]*lx+rotations[9*body+8]*lz;
            if(pz>=height(context.physical.terrain_vertices(),px)&&shear==0.)continue;
            sides.push_back("[\""+wheel+"\", "+std::to_string(index)+"]");xi.push_back(shear);valid.push_back(norm>1e-12);for(double const v:tangent)tangents.push_back(norm>1e-12?v/norm:v);
        }
    }
    auto material=material_state(p);std::erase_if(material.arrays,[](const auto& a){return a.first.starts_with("state_tire_");});material.arrays.emplace_back("state_tire_sides",bio::string_array(sides,{sides.size()}));material.arrays.emplace_back("state_tire_xi",bio::double_array(xi,{xi.size()}));material.arrays.emplace_back("state_tire_tangent",bio::double_array(tangents,{xi.size(),3}));material.arrays.emplace_back("state_tire_tangent_valid",bio::bool_array(valid,{valid.size()}));
    put(p,"material",material_document(material));put(p,"contract_sha256",string(contract(context,count)));put(p,"station_projection",bio::Value{bio::Value::Object{{"method",string("periodic piecewise-linear angular shear field; identical qpos/qvel")},{"source_count",integer(context.physics.tires.distributed.station_count)},{"target_count",integer(count)},{"source_initial_state_sha256",string(source.sha256())}}});
    return PhysicalInitialState(detail::source_json(value));
}
NATIVE_DIAG_POP
} // namespace bike::initialization
