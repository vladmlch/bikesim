#include "builder_internal.hpp"
#include "benchmark_whipple_detail.hpp"
#include "../engine_call.hpp"
#include <array>
#include <limits>
#include <stdexcept>

namespace bike::model {
std::string generate_mjcf(const BuildRequest& request) {
    using namespace detail;
    if (request.benchmark_whipple) return generate_whipple_mjcf(*request.benchmark_whipple);
    auto physics=request.physics?
        std::make_unique<configuration::SimulationPhysicsConfig>(*request.physics):nullptr;
    if(request.spatial && physics && physics->spatial &&
       (request.spatial->mode!=physics->spatial->mode ||
        (request.spatial->mode==bike::spatial::Mode::spatial3d &&
         bike::spatial::spatial_config_fingerprint_hex(*request.spatial)!=
         bike::spatial::spatial_config_fingerprint_hex(*physics->spatial))))
        throw std::invalid_argument("model.spatial: model and physics configurations differ");
    if(request.spatial && request.spatial->mode==bike::spatial::Mode::spatial3d) {
        if(!physics) throw std::invalid_argument("spatial_3d: physical configuration required");
        physics->spatial=request.spatial;
    }
    if(physics) configuration::post_init(*physics);
    if(request.terrain3d && (!physics || !physics->spatial ||
       physics->spatial->mode!=bike::spatial::Mode::spatial3d))
        throw std::invalid_argument("model.terrain3d: spatial source requires spatial physics");
    const bool physical=physics && physics->physics_mode=="physical";
    if(physical && request.mode!="ride") throw std::invalid_argument("physical configuration requires ride mode");
    if(physical && request.tyre_model!="sphere") throw std::invalid_argument("legacy pneumatic tyre_model is not a physical backend; configure physics_config.tires");
    const bool crank=request.crank_joint || physical;
    if(crank && request.mode!="ride") throw std::invalid_argument("the pedalled crankset is a ride-mode model; mode '"+request.mode+"' builds a rigid crankset");
    if(request.tyre_model!="sphere" && request.tyre_model!="pneumatic") throw std::invalid_argument("unknown tyre model '"+request.tyre_model+"'; available: sphere, pneumatic");
    if(request.tyre_model!="sphere" && request.mode!="ride") throw std::invalid_argument("tyre model '"+request.tyre_model+"' is ride-mode only");
    auto rider=resolve_rider(request.rider?RiderArg(*request.rider):RiderArg(std::monostate{}),request.include_rider,"lumped");
    if(rider.variant=="seated" && request.mode!="ride") throw std::invalid_argument("the seated rider is a ride-mode model; mode '"+request.mode+"' takes 'none' or 'lumped'");
    std::optional<SeatedPose> seated;
    if(rider.variant=="seated") seated=solve_seated_pose(request.bike,rider);
    std::optional<ArticulatedPose> articulated;
    if(rider.variant=="articulated_planar") {
        if(!physical) throw std::invalid_argument("articulated_planar requires physical ride mode");
        articulated=geometry_pose(rider,request.bike);
    }
    if(physical && physics->drive_mode=="articulated_effort" && !articulated) throw std::invalid_argument("articulated_effort requires articulated_planar rider");
    if(physical && seated && !seated->leg_chains.empty()) throw std::invalid_argument("physical seated rider supports rigid legs; use articulated_planar");
    // Source validation is independent of environment creation; all bad
    // mode/attachment combinations fail before any mjModel or mjData exists.
    if(physics) {
        const auto& a=physics->articulated;
        const bool attachment=a.pedal_attachment=="weld" || a.pedal_attachment=="spindle" || a.saddle_attachment=="weld" || a.saddle_attachment=="pin" || a.grip_attachment=="connect";
        if(attachment && !(physical && articulated)) throw std::invalid_argument("non-flat rider attachments (weld/pin) require physics_mode='physical' and rider='articulated_planar'");
    }
    // Construct only when absent: preserve a caller's fitted linkage solver.
    std::optional<kinematics::HorstLinkageSolver> default_solver;
    if(!request.solver) default_solver.emplace(request.bike);
    const auto& solver=request.solver?*request.solver:*default_solver;
    const auto fixed=geometry::get_fixed_frame_points(request.bike);
    const auto state=solver.solve_state_from_wheel_travel(0.);
    const Vec3 cg=physical?Vec3{.001,0.,0.}:compute_static_system_cg(request.bike,request.masses,rider.present()?&rider:nullptr,&solver).cg_pos_m;
    BuildContext context{.request=request,.solver=solver,.rider=std::move(rider),.seated=std::move(seated),.articulated=articulated,.physics=std::move(physics),.fixed=fixed,.state=state,.cg=cg,.crank_joint=crank,.registry={}};
    Element root("mujoco",{{"model","enduro_bike_horst_yoke"}});
    root.add("compiler",{{"angle","degree"},{"coordinate","local"},{"inertiafromgeom","auto"},{"autolimits","true"}});
    std::string timestep="0.002";
    if(request.mode=="ride") timestep=context.physics?repr(context.physics->timestep_s):"0.0005";
    else if(request.mode=="stand" || request.mode=="playground") timestep="0.001";
    root.add("option",{{"gravity","0 0 -9.81"},{"timestep",timestep},{"integrator","implicitfast"},{"iterations","100"},{"tolerance","1e-10"}});
    build_visual_and_assets(root);
    auto& world=root.add("worldbody");
    build_environment(world,context);
    if(request.mode=="ride") build_terrain(root,world,context);
    auto& frame=build_frame_body(world,context);
    build_steering_and_fork(frame,context);build_rear_linkage(frame,context);
    if(physical) assign_budgets(frame,context.registry,request.masses);
    build_equality_constraints(root,request.mode);
    if(crank && !physical) {
        build_chain_constraint(root,request.gear_ratio);
        if(context.seated) for(const auto& chain:context.seated->leg_chains)
            block(root,"equality").add("weld",{{"name","weld_foot_"+chain.side},{"body1","rider_foot_"+chain.side},{"body2","pedal_"+chain.side},{"solref","0.003 1"}});
    }
    build_contact_exclusions(root);build_actuators(root,context);build_sensors(root,context);
    if(physical) {
        finish_physical_topology(root,context);
        if(context.articulated) {
            for(auto* value:frame.direct("geom")) if(value->get("name").starts_with("geom_rider_")) (void)frame.remove(value);
            std::optional<configuration::JointEnvelopeMap> envelopes;
            if(context.physics->articulated.joint_envelope_path) envelopes=configuration::load_joint_envelopes(*context.physics->articulated.joint_envelope_path);
            build_articulated_rider(world,*context.articulated,split_paired_arm_masses(segment_masses(context.rider.mass_kg,context.rider.helmet_mass_kg)),
                envelopes?&*envelopes:nullptr,context.physics->articulated.pedal_attachment=="spindle");
            add_rider_actuators(root,context.physics->articulated);
            finish_rider_attachments(root,context);
        }
        if(context.spatial()) finish_spatial_topology(root,context);
        finalize_geometric_transmission(root,*context.physics);
    }
    return xml::document(root);
}

void ModelDeleter::operator()(mjModel* model) const noexcept { if(model) mj_deleteModel(model); }
namespace {
struct VirtualFiles {
    mjVFS value{};
    VirtualFiles() { mj_defaultVFS(&value); }
    ~VirtualFiles() { mj_deleteVFS(&value); }
    VirtualFiles(const VirtualFiles&)=delete;
    VirtualFiles& operator=(const VirtualFiles&)=delete;
    VirtualFiles(VirtualFiles&&) = delete;
    VirtualFiles& operator=(VirtualFiles&&) = delete;
};
struct MountOperation {
    mjVFS* vfs{};
    const char* name{};
    const void* bytes{};
    int size{};
    int result=-1;
};
void mount_operation(void* raw) noexcept {
    auto* call=static_cast<MountOperation*>(raw);
    call->result=mj_addBufferVFS(call->vfs,call->name,call->bytes,call->size);
}
struct CompileOperation {
    const char* file{};
    const mjVFS* vfs{};
    char* message{};
    int capacity{};
    mjModel* model=nullptr;
};
void compile_operation(void* raw) noexcept {
    auto* call=static_cast<CompileOperation*>(raw);
    call->model=mj_loadXML(call->file,call->vfs,call->message,call->capacity);
}
}
ModelOwner compile_mjcf(std::string_view xml,const std::filesystem::path& asset_root) {
    if(xml.empty() || xml.find('\0')!=std::string_view::npos) throw std::invalid_argument("compile_mjcf: XML must be a nonempty text document without NUL");
    if(xml.size()>static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::invalid_argument("compile_mjcf: XML exceeds MuJoCo VFS buffer size");
    if(asset_root.native().find('\0')!=std::string::npos) throw std::invalid_argument("compile_mjcf: asset_root contains NUL");
    engine::require_supported_callbacks();engine::require_native_time_policy();
    auto files=std::make_unique<VirtualFiles>();
    // MuJoCo resolves relative assets/includes against the XML resource's
    // directory. Give the in-memory document an absolute source name, with
    // a basename alias for VFS versions that strip source directories.
    const auto filename=(std::filesystem::absolute(asset_root.empty()?std::filesystem::path("."):asset_root)/"__bike_generated_model__.xml").lexically_normal().string();
    engine::ErrorBuffer error;
    MountOperation mount{.vfs=&files->value,.name=filename.c_str(),.bytes=xml.data(),.size=static_cast<int>(xml.size())};
    if(!engine::invoke(mount_operation,&mount,error)) engine::throw_failure(error);
    if(mount.result!=0) throw std::runtime_error("compile_mjcf: cannot mount generated MJCF in VFS");
    MountOperation alias{.vfs=&files->value,.name="__bike_generated_model__.xml",.bytes=xml.data(),.size=static_cast<int>(xml.size())};
    if(!engine::invoke(mount_operation,&alias,error)) engine::throw_failure(error);
    if(alias.result!=0 && alias.result!=2) throw std::runtime_error("compile_mjcf: cannot mount VFS source alias");
    auto message=std::make_unique<std::array<char,4096>>();
    CompileOperation compile{.file=filename.c_str(),.vfs=&files->value,.message=message->data(),.capacity=static_cast<int>(message->size())};
    if(!engine::invoke(compile_operation,&compile,error)) {
        ModelOwner const cleanup(compile.model);engine::throw_failure(error);
    }
    ModelOwner result(compile.model);
    if(!result) throw std::invalid_argument("compile_mjcf: "+std::string(message->data()));
    if(result->nplugin!=0) throw std::invalid_argument("compile_mjcf: unsupported MuJoCo plugin model");
    return result;
}
} // namespace bike::model
