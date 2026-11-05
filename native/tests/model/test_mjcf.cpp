// T16: generation from typed inputs, compared to an independently captured
// Python MJCF through the same pinned native compiler. No fixture is updated.
#include "support/fixture_test.hpp"
#include "configuration/load.hpp"
#include "model/builder.hpp"
#include "model/xml_writer.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>

namespace {
namespace mdl=bike::model;
namespace cfg=bike::configuration;

std::unique_ptr<mdl::BuildRequest> physical(std::string_view profile="viewer_physics_welded.toml") {
    auto request=std::make_unique<mdl::BuildRequest>();
    request->mode="ride";
    request->rider=cfg::RiderSpecs{};request->rider->variant="articulated_planar";
    request->physics=cfg::load_physics(bt::fixture("config/"+std::string(profile)));
    return request;
}
int id(const mjModel& model,mjtObj kind,const char* name) {
    const int value=mj_name2id(&model,kind,name);
    bt::require(value>=0,"required model name: "+std::string(name));return value;
}
template<class T>
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
void equal_array(std::string_view field,const T* actual,const T* expected,std::size_t count) {
    const auto a=std::views::counted(actual,static_cast<std::ptrdiff_t>(count));
    const auto b=std::views::counted(expected,static_cast<std::ptrdiff_t>(count));
    for(std::size_t index=0;index<count;++index) {
        bool equal = false;
        if constexpr(std::is_same_v<T,double>) equal=std::bit_cast<std::uint64_t>(a[index])==std::bit_cast<std::uint64_t>(b[index]);
        else equal=a[index]==b[index];
        bt::require(equal,std::string(field)+"["+std::to_string(index)+"] differs from independently compiled reference XML");
    }
}
void compare(const mjModel& actual,const mjModel& expected) {
#define DIM(field) bt::require(actual.field==expected.field,"compiled dimension " #field)
    DIM(nq);DIM(nv);DIM(nu);DIM(na);DIM(nbody);DIM(njnt);DIM(ngeom);DIM(nsite);
    DIM(neq);DIM(ntendon);DIM(nwrap);DIM(nsensor);DIM(nsensordata);DIM(nhfield);DIM(nhfielddata);
#undef DIM
    struct Named {mjtObj kind;std::size_t count;const char* label;};
    const auto named = std::to_array<Named>({
        {.kind=mjOBJ_BODY,.count=static_cast<std::size_t>(actual.nbody),.label="body"},
        {.kind=mjOBJ_JOINT,.count=static_cast<std::size_t>(actual.njnt),.label="joint"},
        {.kind=mjOBJ_GEOM,.count=static_cast<std::size_t>(actual.ngeom),.label="geom"},
        {.kind=mjOBJ_SITE,.count=static_cast<std::size_t>(actual.nsite),.label="site"},
        {.kind=mjOBJ_ACTUATOR,.count=static_cast<std::size_t>(actual.nu),.label="actuator"},
        {.kind=mjOBJ_EQUALITY,.count=static_cast<std::size_t>(actual.neq),.label="equality"},
        {.kind=mjOBJ_TENDON,.count=static_cast<std::size_t>(actual.ntendon),.label="tendon"},
        {.kind=mjOBJ_SENSOR,.count=static_cast<std::size_t>(actual.nsensor),.label="sensor"},
    });
    for(const auto& group:named) for(std::size_t index=0;index<group.count;++index) {
        const char* a=mj_id2name(&actual,group.kind,static_cast<int>(index));
        const char* b=mj_id2name(&expected,group.kind,static_cast<int>(index));
        bt::require(std::string_view(a?a:"")==std::string_view(b?b:""),std::string(group.label)+" name/order at ID "+std::to_string(index));
    }
#define ARRAY(field,count) equal_array(#field,actual.field,expected.field,static_cast<std::size_t>(count))
    ARRAY(qpos0,actual.nq);
    ARRAY(body_parentid,actual.nbody);ARRAY(body_pos,actual.nbody*3);ARRAY(body_quat,actual.nbody*4);
    ARRAY(body_mass,actual.nbody);ARRAY(body_inertia,actual.nbody*3);ARRAY(body_ipos,actual.nbody*3);ARRAY(body_iquat,actual.nbody*4);
    ARRAY(jnt_type,actual.njnt);ARRAY(jnt_qposadr,actual.njnt);ARRAY(jnt_dofadr,actual.njnt);ARRAY(jnt_bodyid,actual.njnt);
    ARRAY(jnt_pos,actual.njnt*3);ARRAY(jnt_axis,actual.njnt*3);ARRAY(jnt_range,actual.njnt*2);ARRAY(jnt_stiffness,actual.njnt);
    ARRAY(dof_damping,actual.nv);ARRAY(dof_frictionloss,actual.nv);ARRAY(dof_armature,actual.nv);
    ARRAY(dof_solref,actual.nv*mjNREF);ARRAY(dof_solimp,actual.nv*mjNIMP);
    ARRAY(geom_type,actual.ngeom);ARRAY(geom_bodyid,actual.ngeom);ARRAY(geom_pos,actual.ngeom*3);ARRAY(geom_quat,actual.ngeom*4);
    ARRAY(geom_size,actual.ngeom*3);ARRAY(geom_rgba,actual.ngeom*4);ARRAY(geom_contype,actual.ngeom);ARRAY(geom_conaffinity,actual.ngeom);
    ARRAY(geom_friction,actual.ngeom*3);ARRAY(geom_solref,actual.ngeom*mjNREF);ARRAY(geom_solimp,actual.ngeom*mjNIMP);
    ARRAY(site_bodyid,actual.nsite);ARRAY(site_pos,actual.nsite*3);ARRAY(site_quat,actual.nsite*4);ARRAY(site_size,actual.nsite*3);
    ARRAY(eq_type,actual.neq);ARRAY(eq_obj1id,actual.neq);ARRAY(eq_obj2id,actual.neq);ARRAY(eq_active0,actual.neq);ARRAY(eq_data,actual.neq*mjNEQDATA);
    ARRAY(eq_solref,actual.neq*mjNREF);ARRAY(eq_solimp,actual.neq*mjNIMP);
    ARRAY(actuator_trnid,actual.nu*2);ARRAY(actuator_gear,actual.nu*6);ARRAY(actuator_ctrlrange,actual.nu*2);
    ARRAY(actuator_forcerange,actual.nu*2);ARRAY(actuator_gainprm,actual.nu*mjNGAIN);ARRAY(actuator_biasprm,actual.nu*mjNBIAS);
    ARRAY(tendon_range,actual.ntendon*2);ARRAY(tendon_stiffness,actual.ntendon);ARRAY(tendon_damping,actual.ntendon);
    ARRAY(tendon_lengthspring,actual.ntendon*2);ARRAY(tendon_length0,actual.ntendon);
    ARRAY(wrap_type,actual.nwrap);ARRAY(wrap_objid,actual.nwrap);ARRAY(wrap_prm,actual.nwrap);
    ARRAY(sensor_type,actual.nsensor);ARRAY(sensor_objid,actual.nsensor);ARRAY(sensor_adr,actual.nsensor);ARRAY(sensor_dim,actual.nsensor);
    ARRAY(hfield_size,actual.nhfield*4);ARRAY(hfield_nrow,actual.nhfield);ARRAY(hfield_ncol,actual.nhfield);
#undef ARRAY
    bt::require(actual.opt.timestep==expected.opt.timestep,"integration timestep");
    bt::require(actual.opt.integrator==expected.opt.integrator,"integration method");
    bt::require(actual.opt.iterations==expected.opt.iterations,"solver iteration budget");
    bt::require(actual.opt.tolerance==expected.opt.tolerance,"solver tolerance");
}
// No joint in any built model carries damping (the model constraints in
// AGENTS.md; suspension dampers are separate force elements).
void require_no_joint_damping(const mjModel& model,std::string_view label) {
    const auto damping=std::views::counted(model.dof_damping,model.nv);
    for(int dof=0;dof<model.nv;++dof)
        bt::require(damping[static_cast<std::size_t>(dof)]==0.,std::string(label)+": dof "+std::to_string(dof)+" carries joint damping");
}
// Every limited tendon is a one-way drivetrain closure built by
// fixed_tendon (model/physical_topology.cpp). The energy ledger charges all
// 'tendon_limit:' rows to the elastic closure channel (runtime/accounting.cpp
// raw_work), so a new limited tendon must be classified there on purpose.
void require_limited_tendons_are_closures(const mjModel& model,std::string_view label) {
    static constexpr std::array<std::string_view,4> closures{
        "ideal_mid_drive_freehub","geometric_mid_drive_freehub","crank_clutch","motor_freewheel"};
    const auto limited=std::views::counted(model.tendon_limited,model.ntendon);
    for(int tendon=0;tendon<model.ntendon;++tendon) {
        if(limited[static_cast<std::size_t>(tendon)]==0) continue;
        const char* name=mj_id2name(&model,mjOBJ_TENDON,tendon);
        const std::string_view found=name==nullptr?std::string_view{}:std::string_view(name);
        bt::require(std::ranges::find(closures,found)!=closures.end(),
            std::string(label)+": limited tendon '"+std::string(found)+"' is not a drivetrain closure");
    }
}
void captured_welded() {
    auto request=physical();
    // RCP-02 was captured from rough_uphill_savage: 130 m field, 30.1 m
    // elevation, 2.8 m datum; these are reference INPUTS, not output data.
    request->field.ncol=26001;request->field.radius_x_m=65.;request->field.elevation_m=30.1;
    const auto xml=mdl::generate_mjcf(*request);
    bt::require(xml==mdl::generate_mjcf(*request),"MJCF generation is deterministic");
    const auto actual=mdl::compile_mjcf(xml);
    const auto expected=mdl::compile_mjcf(bt::text(bt::fixture("model/welded.xml")),bt::fixture("model/assets"));
    compare(*actual,*expected);
}
void physical_profiles_and_topology() {
    for(const auto* name:{"viewer_physics_welded.toml","viewer_physics_distributed.toml","viewer_physics_pneumatic.toml"}) {
        const auto request=physical(name);
        const auto model=mdl::compile_mjcf(mdl::generate_mjcf(*request));
        require_no_joint_damping(*model,name);
        require_limited_tendons_are_closures(*model,name);
        bt::require(model->nq==26 && model->nv==26 && model->nu==12,"physical spindle profile inventory");
        bt::require(mj_name2id(model.get(),mjOBJ_JOINT,"steer_joint")==-1,"physical steering is welded");
        bt::require(mj_name2id(model.get(),mjOBJ_JOINT,"rider_ankle_front")==-1,"spindle locks ankle hinge");
        id(*model,mjOBJ_EQUALITY,"connect_foot_front");id(*model,mjOBJ_EQUALITY,"connect_saddle");
        id(*model,mjOBJ_EQUALITY,"connect_grip_left");id(*model,mjOBJ_TENDON,"ideal_mid_drive_freehub");
        for(const auto* side:{"front","rear"}) {
            const auto contact=static_cast<std::size_t>(id(*model,mjOBJ_GEOM,("geom_"+std::string(side)+"_contact").c_str()));
            bt::require(std::views::counted(model->geom_contype,model->ngeom)[contact]==0 && std::views::counted(model->geom_conaffinity,model->ngeom)[contact]==0,"native tire backend disables engine wheel contacts");
        }
        const auto cg=static_cast<std::size_t>(id(*model,mjOBJ_SITE,"site_CG"));
        bt::require(std::views::counted(model->site_pos,model->nsite*3)[cg*3]==.001,"COM marker retains initial nonzero offset");
        const auto catch_=static_cast<std::size_t>(id(*model,mjOBJ_GEOM,"catch_plane"));
        const auto terrain=static_cast<std::size_t>(id(*model,mjOBJ_GEOM,"terrain"));
        const auto masks=std::views::counted(model->geom_conaffinity,model->ngeom);
        bt::require(masks[catch_]==13 && masks[terrain]==5,"rider/crash and catch masks");
    }
    auto request=physical();request->physics.value().articulated.joint_envelope_path.reset(); // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    request->physics->articulated.pedal_attachment="weld";request->physics->articulated.saddle_attachment="weld";request->physics->articulated.grip_attachment="spring"; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    const auto welded=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    require_no_joint_damping(*welded,"weld/spring attachments");
    require_limited_tendons_are_closures(*welded,"weld/spring attachments");
    id(*welded,mjOBJ_JOINT,"rider_ankle_front");id(*welded,mjOBJ_EQUALITY,"weld_foot_front");id(*welded,mjOBJ_EQUALITY,"weld_saddle");
    bt::require(welded->nq==28,"welded foot preserves ankle DOFs");
    request->physics.value().drive.transmission_model="elastic_chain"; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    request->physics->drive.shifting.enabled=false; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    const auto elastic=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    require_no_joint_damping(*elastic,"elastic chain");
    require_limited_tendons_are_closures(*elastic,"elastic chain");
    id(*elastic,mjOBJ_BODY,"cassette");
    bt::require(mj_name2id(elastic.get(),mjOBJ_TENDON,"ideal_mid_drive_freehub")==-1,"elastic chain has no ideal freehub constraint");
    const auto wheel_id=static_cast<std::size_t>(id(*elastic,mjOBJ_BODY,"rear_wheel"));
    const auto cassette_id=static_cast<std::size_t>(id(*elastic,mjOBJ_BODY,"cassette"));
    const auto masses=std::views::counted(elastic->body_mass,elastic->nbody);
    bt::require(masses[wheel_id]==request->masses.rear_wheel_mass*.9,"rear wheel split retains 90 percent budget");
    bt::require(masses[cassette_id]==request->masses.rear_wheel_mass*.1,"cassette transfers 10 percent budget");
    request=physical();request->physics->drive.motor_clutch=true; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    const auto clutch=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    require_no_joint_damping(*clutch,"motor clutch");
    require_limited_tendons_are_closures(*clutch,"motor clutch");
    id(*clutch,mjOBJ_JOINT,"drive_shaft_spin");id(*clutch,mjOBJ_TENDON,"crank_clutch");
    request=physical();request->physics->drive.rotor_inertia_kgm2=.2; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    const auto rotor=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    require_no_joint_damping(*rotor,"rotor inertia");
    require_limited_tendons_are_closures(*rotor,"rotor inertia");
    id(*rotor,mjOBJ_JOINT,"rotor_spin");id(*rotor,mjOBJ_TENDON,"motor_freewheel");
    request->physics->drive.motor_clutch=true; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=physical();request->physics->drive.transmission_model="geometric_ideal_mid_drive"; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    const auto geometric=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    require_no_joint_damping(*geometric,"geometric mid drive");
    require_limited_tendons_are_closures(*geometric,"geometric mid drive");
    const auto tendon=static_cast<std::size_t>(id(*geometric,mjOBJ_TENDON,"geometric_mid_drive_freehub"));
    bt::require(std::views::counted(geometric->tendon_num,geometric->ntendon)[tendon]==geometric->njnt,"geometric drive includes all scalar joints and rider zero rows");
    request=physical();request->masses.front_wheel_mass=3.1;request->masses.rear_wheel_mass=3.7;
    request->masses.frame_structure_mass=4.2;request->masses.battery_mass=5.1;
    const auto reweighted=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    const auto weighted_masses=std::views::counted(reweighted->body_mass,reweighted->nbody);
    bt::require(weighted_masses[static_cast<std::size_t>(id(*reweighted,mjOBJ_BODY,"front_wheel"))]==3.1,"front explicit inertial uses requested budget");
    double total=0.;for(double const mass:weighted_masses.subspan(1)) total+=mass;
    bt::require(std::abs(total-(request->masses.total_bike_mass()+request->rider->mass_kg))<1e-11,"physical component and rider mass budgets close"); // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
}
// The non-physical builder modes and rider variants: none emits joint damping.
void no_joint_damping_in_any_mode() {
    struct Case { const char* mode; const char* rider; const char* legs; bool crank; };
    for(const Case c:{Case{.mode="standard",.rider="none",.legs="rigid",.crank=false},
                      Case{.mode="standard",.rider="lumped",.legs="rigid",.crank=false},
                      Case{.mode="ride",.rider="none",.legs="rigid",.crank=false}}) {
        mdl::BuildRequest request;request.mode=c.mode;request.crank_joint=c.crank;
        if(std::string_view(c.rider)!="none") {
            request.include_rider=true;request.rider=cfg::RiderSpecs{};
            request.rider->variant=c.rider;request.rider->legs=c.legs;
        }
        const auto model=mdl::compile_mjcf(mdl::generate_mjcf(request));
        require_no_joint_damping(*model,std::string(c.mode)+"/"+c.rider+"/"+c.legs);
        require_limited_tendons_are_closures(*model,std::string(c.mode)+"/"+c.rider+"/"+c.legs);
    }
}
// The seated rider variant was removed (2026-11-04): it no longer
// resolves, and the declared default rider is lumped.
void seated_variant_is_gone() {
    bt::invalid([] { (void)mdl::rider_from_args("seated"); });
    const cfg::RiderSpecs declared;
    bt::require(declared.variant=="lumped","the declared default rider is lumped");
}
// The seated-only parameters (legs mode, load shares, resonances,
// damping ratio, saddle interface) do not reach any remaining model:
// changing them leaves the MJCF of the lumped and articulated_planar
// riders byte-identical. This is the evidence that lets RiderSpecs drop
// them (configuration/legacy_rider.hpp reads old documents).
[[gnu::noinline]] void seated_parameters_are_inert() {
    for(const auto* variant:{"lumped","articulated_planar"}) {
        mdl::BuildRequest request;request.mode="ride";request.include_rider=true;
        request.rider=cfg::RiderSpecs{};request.rider->variant=variant;
        if(std::string_view(variant)=="articulated_planar")
            request.physics=cfg::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
        const auto before=mdl::generate_mjcf(request);
        auto& rider=*request.rider;
        rider.legs="articulated";
        // Shares still sum to one so post_init accepts the record.
        rider.saddle_share=.50;rider.pedal_share=.35;rider.bar_share=.15;
        rider.torso_resonance_hz=9.;rider.leg_resonance_hz=8.;rider.arm_resonance_hz=7.;
        rider.damping_ratio=.9;rider.saddle_interface_k_n_m=1.;rider.saddle_interface_c_ns_m=1.;
        cfg::detail::post_init(rider);
        bt::require(mdl::generate_mjcf(request)==before,
                    std::string(variant)+": seated-only parameters do not change the model");
    }
}
// Crank qpos 0 is a power position: the two spindles sit level (same
// height) and 180 degrees apart. PedalingPolicy's reposition goal
// (multiples of pi from kPowerPhaseOffsetRad) depends on it.
void crank_zero_is_horizontal() {
    const auto request = physical("viewer_physics_welded.toml");
    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(mj_makeData(model.get()), mj_deleteData);
    const int crank = mj_name2id(model.get(), mjOBJ_JOINT, "crank_spin");
    const int front = mj_name2id(model.get(), mjOBJ_BODY, "pedal_front");
    const int rear = mj_name2id(model.get(), mjOBJ_BODY, "pedal_rear");
    bt::require(crank >= 0 && front >= 0 && rear >= 0, "crank and both pedal bodies exist");
    const auto crank_i = static_cast<std::size_t>(crank);
    const auto front_i = static_cast<std::size_t>(front);
    const auto rear_i = static_cast<std::size_t>(rear);
    const auto qposadr = std::views::counted(model->jnt_qposadr, model->njnt);
    const auto qpos = std::views::counted(data->qpos, model->nq);
    const auto xpos = std::views::counted(data->xpos, model->nbody * 3);
    for (const double angle : {0., std::numbers::pi}) {
        mj_resetData(model.get(), data.get());
        qpos[static_cast<std::size_t>(qposadr[crank_i])] = angle;
        mj_kinematics(model.get(), data.get());
        const double dz = xpos[3 * front_i + 2] - xpos[3 * rear_i + 2];
        const double dx = xpos[3 * front_i + 0] - xpos[3 * rear_i + 0];
        bt::require(std::abs(dz) < 1e-6, "spindles level at a power phase (dz=" + std::to_string(dz) + ")");
        bt::require(std::abs(dx) > .2, "spindles a crank diameter apart along x");
    }
}
void errors_and_ownership() {
    auto request=physical();request->mode="stand";
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=physical();request->mode="standard";
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=physical();request->rider->variant="lumped"; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=physical();request->rider->variant="seated"; // NOLINT(bugprone-unchecked-optional-access) same as above
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=std::make_unique<mdl::BuildRequest>();request->crank_joint=true;
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=physical();request->masses.battery_mass=0.;
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    bt::invalid([]{(void)mdl::compile_mjcf("<mujoco><unknown/></mujoco>");});
    bt::invalid([]{(void)mdl::compile_mjcf(std::string("x\0y",3));});
    const auto xml=mdl::generate_mjcf(*physical());
    auto first=mdl::compile_mjcf(xml),second=mdl::compile_mjcf(xml);
    std::views::counted(first->body_mass,first->nbody)[1]+=1.;
    bt::require(std::views::counted(first->body_mass,first->nbody)[1]!=std::views::counted(second->body_mass,second->nbody)[1],"separate compilation owners do not alias mutable model arrays");
    first.reset();
    bt::require(second->nq==26,"a model remains owned after sibling destruction");
    mdl::xml::Element const escaped("mujoco",{{"model","A & B < \"quoted\"\n"}});
    const auto text=mdl::xml::document(escaped);
    bt::require(text.find("A &amp; B &lt; &quot;quoted&quot;&#10;")!=std::string::npos,"XML attributes escape reserved characters and whitespace");
}
} // namespace
int main() {return bt::run([]{captured_welded();physical_profiles_and_topology();no_joint_damping_in_any_mode();seated_variant_is_gone();seated_parameters_are_inert();crank_zero_is_horizontal();errors_and_ownership();});}
