// T16: generation from typed inputs, compared to an independently captured
// Python MJCF through the same pinned native compiler. No fixture is updated.
#include "support/fixture_test.hpp"
#include "configuration/load.hpp"
#include "engine_call.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model/xml_writer.hpp"
#include "rider/state_codec.hpp"
#include "runtime/project_config.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

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
// bt::fixture("config") is <repo>/tests/fixtures/config — the live example
// document is three levels up plus its repo-relative path.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::fixture("config").parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}
// Same build as physical(), but from the working profile under
// examples/research/: the fixture copies now carry motor_clutch=true like
// the research originals, so the clutch the rider actually gets is proven
// from the documents bike-ride loads.
std::unique_ptr<mdl::BuildRequest> research(std::string_view profile) {
    auto request=std::make_unique<mdl::BuildRequest>();
    request->mode="ride";
    request->rider=cfg::RiderSpecs{};request->rider->variant="articulated_planar";
    request->physics=cfg::load_physics(repo_file("examples/research/"+std::string(profile)));
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
    static constexpr std::array<std::string_view,3> closures{
        "ideal_mid_drive_freehub","geometric_mid_drive_freehub","crank_clutch"};
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
        bt::require(model->nq==27 && model->nv==27 && model->nu==12,"physical spindle profile inventory");
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
    bt::require(welded->nq==29,"welded foot preserves ankle DOFs");
    request=physical();request->physics->drive.motor_clutch=true; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    const auto clutch=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    require_no_joint_damping(*clutch,"motor clutch");
    require_limited_tendons_are_closures(*clutch,"motor clutch");
    id(*clutch,mjOBJ_JOINT,"drive_shaft_spin");id(*clutch,mjOBJ_TENDON,"crank_clutch");
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
    struct Case { const char* mode; const char* rider; bool crank; };
    for(const Case c:{Case{.mode="standard",.rider="none",.crank=false},
                      Case{.mode="standard",.rider="lumped",.crank=false},
                      Case{.mode="ride",.rider="none",.crank=false}}) {
        mdl::BuildRequest request;request.mode=c.mode;request.crank_joint=c.crank;
        if(std::string_view(c.rider)!="none") {
            request.include_rider=true;request.rider=cfg::RiderSpecs{};
            request.rider->variant=c.rider;
        }
        const auto model=mdl::compile_mjcf(mdl::generate_mjcf(request));
        require_no_joint_damping(*model,std::string(c.mode)+"/"+c.rider);
        require_limited_tendons_are_closures(*model,std::string(c.mode)+"/"+c.rider);
    }
}
// The seated rider variant was removed (2026-11-04): it no longer
// resolves, and the declared default rider is lumped.
void seated_variant_is_gone() {
    bt::invalid([] { (void)mdl::rider_from_args("seated"); });
    const cfg::RiderSpecs declared;
    bt::require(declared.variant=="lumped","the declared default rider is lumped");
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
// Two one-way clutches only (user, 2026-11-05): the rear freehub and the
// motor clutch. The rotor freewheel and its body are gone, and an old
// document naming rotor_inertia_kgm2 is rejected.
void drivetrain_has_two_one_way_clutches() {
    auto request = physical("viewer_physics_welded.toml");
    request->physics->drive.motor_clutch = true; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; -> is the loud check
    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    bt::require(mj_name2id(model.get(), mjOBJ_TENDON, "ideal_mid_drive_freehub") >= 0, "freehub present");
    bt::require(mj_name2id(model.get(), mjOBJ_TENDON, "crank_clutch") >= 0, "motor clutch present");
    bt::require(mj_name2id(model.get(), mjOBJ_TENDON, "motor_freewheel") < 0, "no rotor freewheel");
    bt::require(mj_name2id(model.get(), mjOBJ_BODY, "motor_rotor") < 0, "no rotor body");
    bt::require(mj_name2id(model.get(), mjOBJ_JOINT, "rotor_spin") < 0, "no rotor joint");
}
// The 0.2 kg·m² drive-inertia ruling (2026-11-06): a real rotor body welded
// to the motor shaft behind the crank clutch — or to the crank axis on a
// rigid drive — carrying inertia already reflected to the crank coordinate
// (shaft:crank is 1:1). No rotor joint, no second freewheel, no armature.
void rotor_body_welds_to_the_drive_axis() {
    constexpr double inertia = .2;
    const auto dof_inertia = [](const mjModel& model, const char* joint_name) {
        const auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(
            mj_makeData(&model), mj_deleteData);
        const int joint = mj_name2id(&model, mjOBJ_JOINT, joint_name);
        bt::require(joint >= 0, std::string("required joint: ") + joint_name);
        const auto dof = static_cast<std::size_t>(
            std::views::counted(model.jnt_dofadr, model.njnt)[static_cast<std::size_t>(joint)]);
        engine::forward(&model, data.get());
        std::vector<mjtNum> unit(static_cast<std::size_t>(model.nv), 0.), product(unit.size(), 0.);
        unit[dof] = 1.;
        engine::mul_m(&model, data.get(), product.data(), unit.data());
        return static_cast<double>(product[dof]);
    };
    auto request = physical();
    request->physics->drive.motor_clutch = true; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; -> is the loud check
    const auto plain = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    request->physics->drive.rotor_inertia_kgm2 = inertia; // NOLINT(bugprone-unchecked-optional-access) same
    const auto clutched = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    const int rotor = mj_name2id(clutched.get(), mjOBJ_BODY, "motor_rotor");
    bt::require(rotor >= 0, "a positive rotor inertia adds the rotor body");
    bt::require(std::views::counted(clutched->body_parentid, clutched->nbody)[static_cast<std::size_t>(rotor)] ==
                id(*clutched, mjOBJ_BODY, "drive_shaft"),
                "the rotor welds to the motor shaft behind the crank clutch");
    bt::require(mj_name2id(clutched.get(), mjOBJ_JOINT, "rotor_spin") < 0 &&
                mj_name2id(clutched.get(), mjOBJ_TENDON, "motor_freewheel") < 0,
                "the rotor is welded to the shaft, not jointed or clutched");
    bt::require(mj_name2id(plain.get(), mjOBJ_BODY, "motor_rotor") < 0,
                "zero inertia keeps the rotorless shaft");
    bt::require(std::abs(dof_inertia(*clutched, "drive_shaft_spin") -
                       dof_inertia(*plain, "drive_shaft_spin") - inertia) < 1e-12,
                "the welded rotor adds its inertia to the motor-shaft coordinate");
    auto rigid = physical();
    rigid->physics->drive.motor_clutch = false; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; the rigid drive has no clutch shaft
    const auto rigid_plain = mdl::compile_mjcf(mdl::generate_mjcf(*rigid));
    rigid->physics->drive.rotor_inertia_kgm2 = inertia; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; -> is the loud check
    const auto rigid_model = mdl::compile_mjcf(mdl::generate_mjcf(*rigid));
    const int rigid_rotor = mj_name2id(rigid_model.get(), mjOBJ_BODY, "motor_rotor");
    bt::require(rigid_rotor >= 0 &&
                std::views::counted(rigid_model->body_parentid, rigid_model->nbody)[static_cast<std::size_t>(rigid_rotor)] ==
                    id(*rigid_model, mjOBJ_BODY, "crank"),
                "on a rigid drive the rotor welds to the crank axis");
    bt::require(std::abs(dof_inertia(*rigid_model, "crank_spin") -
                       dof_inertia(*rigid_plain, "crank_spin") - inertia) < 1e-12,
                "the crank-axis rotor adds its inertia to the crank coordinate");
}
// Every working profile builds the clutch (user, 2026-11-05): forward the
// crank pulls the motor shaft along; backward (or slower) it is free and
// the shaft is not dragged back by the crank.
void motor_clutch_engages_forward_only() {
    for (const auto* name : {"viewer_physics_welded.toml",
                            "viewer_physics_pneumatic.toml",
                            "viewer_physics_distributed.toml"}) {
        const auto request = research(name);
        const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
        auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(mj_makeData(model.get()), mj_deleteData);
        const int crank = mj_name2id(model.get(), mjOBJ_JOINT, "crank_spin");
        const int shaft = mj_name2id(model.get(), mjOBJ_JOINT, "drive_shaft_spin");
        bt::require(crank >= 0 && shaft >= 0,
                    std::string(name) + ": crank and motor shaft joints exist");
        const auto qvel = std::views::counted(data->qvel, model->nv);
        const auto dofadr = std::views::counted(model->jnt_dofadr, model->njnt);
        const auto crank_dof = static_cast<std::size_t>(dofadr[static_cast<std::size_t>(crank)]);
        const auto shaft_dof = static_cast<std::size_t>(dofadr[static_cast<std::size_t>(shaft)]);
        const auto run = [&](double crank_rate, double shaft_rate) {
            mj_resetData(model.get(), data.get());
            model->opt.gravity[2] = 0.; // isolate the drive from the bike's fall
            qvel[crank_dof] = crank_rate;
            qvel[shaft_dof] = shaft_rate;
            // The engaged clutch is a soft closure spring on an inertia
            // pair; 40 ms covers the exchange on the welded 0.2 kg·m²
            // profiles.
            for (int i = 0; i < 200; ++i) mj_step(model.get(), data.get());
            return std::pair{qvel[crank_dof], qvel[shaft_dof]};
        };
        const auto [c_back, s_back] = run(-3., 5.);
        bt::require(c_back < 0. && s_back >= 0.,
                    std::string(name) + ": backpedalling does not drag the shaft back"
                    + " (crank=" + std::to_string(c_back) + ", shaft=" + std::to_string(s_back) + ")");
        // With the rotor the downstream chain's inertia exceeds the
        // crank's, so a one-shot crank impulse leaves the shaft spinning
        // ahead and the clutch slack — the transmission claim is that the
        // crank's forward motion reaches the shaft, not equal end speeds.
        const auto [c_fwd, s_fwd] = run(8., 0.);
        bt::require(s_fwd > 0. && c_fwd < 8.,
                    std::string(name) + ": a faster forward crank drives the shaft"
                    + " (crank=" + std::to_string(c_fwd) + ", shaft=" + std::to_string(s_fwd) + ")");
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
    bt::require(second->nq==27,"a model remains owned after sibling destruction");
    mdl::xml::Element const escaped("mujoco",{{"model","A & B < \"quoted\"\n"}});
    const auto text=mdl::xml::document(escaped);
    bt::require(text.find("A &amp; B &lt; &quot;quoted&quot;&#10;")!=std::string::npos,"XML attributes escape reserved characters and whitespace");
}
// Leg segment COM and inertia follow de Leva (1996) Table 4 (male):
// thigh COM at 0.4095 of hip-to-knee from hip, radii (0.329, 0.329, 0.149);
// shank COM at 0.4459 of knee-to-ankle from knee, radii (0.251, 0.246, 0.102) for KJC->AJC;
// foot radii (0.257, 0.245, 0.124) against real heel-to-toe length 0.2668 m, COM at midpoint.
void de_leva_leg_segment_com_and_inertia() {
    const auto request = physical("viewer_physics_welded.toml");
    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    const auto body_pos = std::views::counted(model->body_pos, static_cast<std::ptrdiff_t>(model->nbody * 3));
    const auto body_ipos = std::views::counted(model->body_ipos, static_cast<std::ptrdiff_t>(model->nbody * 3));
    const auto body_mass = std::views::counted(model->body_mass, static_cast<std::ptrdiff_t>(model->nbody));
    const auto body_inertia = std::views::counted(model->body_inertia, static_cast<std::ptrdiff_t>(model->nbody * 3));
    for (const auto* side : {"front", "rear"}) {
        const auto thigh_id = static_cast<std::size_t>(id(*model, mjOBJ_BODY, ("rider_thigh_" + std::string(side)).c_str()));
        const auto shank_id = static_cast<std::size_t>(id(*model, mjOBJ_BODY, ("rider_shank_" + std::string(side)).c_str()));
        const auto foot_id = static_cast<std::size_t>(id(*model, mjOBJ_BODY, ("rider_foot_" + std::string(side)).c_str()));

        // Thigh geometry and inertia
        const double shank_x = body_pos[shank_id * 3 + 0];
        const double shank_z = body_pos[shank_id * 3 + 2];
        const double thigh_len = std::hypot(shank_x, shank_z);
        bt::require(thigh_len > 0.4 && thigh_len < 0.5, "thigh length realistic");
        bt::require(std::abs(body_ipos[thigh_id * 3 + 0] - 0.4095 * shank_x) < 1e-9, "thigh COM x matches de Leva 0.4095");
        bt::require(std::abs(body_ipos[thigh_id * 3 + 1]) < 1e-9, "thigh COM y is planar 0");
        bt::require(std::abs(body_ipos[thigh_id * 3 + 2] - 0.4095 * shank_z) < 1e-9, "thigh COM z matches de Leva 0.4095");

        const double thigh_mass = body_mass[thigh_id];
        const double exp_thigh_long = thigh_mass * (0.149 * thigh_len) * (0.149 * thigh_len);
        const double exp_thigh_trans = thigh_mass * (0.329 * thigh_len) * (0.329 * thigh_len);
        const double exp_thigh_sag = thigh_mass * (0.329 * thigh_len) * (0.329 * thigh_len);
        std::array<double, 3> exp_thigh_inertia{exp_thigh_long, exp_thigh_trans, exp_thigh_sag};
        std::ranges::sort(exp_thigh_inertia);
        std::array<double, 3> actual_thigh_inertia{
            body_inertia[thigh_id * 3 + 0],
            body_inertia[thigh_id * 3 + 1],
            body_inertia[thigh_id * 3 + 2]};
        std::ranges::sort(actual_thigh_inertia);
        for (std::size_t i = 0; i < 3; ++i) {
            bt::require(std::abs(actual_thigh_inertia[i] - exp_thigh_inertia[i]) < 1e-8,
                        "thigh principal inertia matches de Leva");
        }

        // Shank geometry and inertia
        const double foot_x = body_pos[foot_id * 3 + 0];
        const double foot_z = body_pos[foot_id * 3 + 2];
        const double shank_len = std::hypot(foot_x, foot_z);
        bt::require(shank_len > 0.4 && shank_len < 0.5, "shank length realistic");
        bt::require(std::abs(body_ipos[shank_id * 3 + 0] - 0.4459 * foot_x) < 1e-9, "shank COM x matches de Leva 0.4459");
        bt::require(std::abs(body_ipos[shank_id * 3 + 1]) < 1e-9, "shank COM y is planar 0");
        bt::require(std::abs(body_ipos[shank_id * 3 + 2] - 0.4459 * foot_z) < 1e-9, "shank COM z matches de Leva 0.4459");

        const double shank_mass = body_mass[shank_id];
        const double exp_shank_long = shank_mass * (0.102 * shank_len) * (0.102 * shank_len);
        const double exp_shank_trans = shank_mass * (0.246 * shank_len) * (0.246 * shank_len);
        const double exp_shank_sag = shank_mass * (0.251 * shank_len) * (0.251 * shank_len);
        std::array<double, 3> exp_shank_inertia{exp_shank_long, exp_shank_trans, exp_shank_sag};
        std::ranges::sort(exp_shank_inertia);
        std::array<double, 3> actual_shank_inertia{
            body_inertia[shank_id * 3 + 0],
            body_inertia[shank_id * 3 + 1],
            body_inertia[shank_id * 3 + 2]};
        std::ranges::sort(actual_shank_inertia);
        for (std::size_t i = 0; i < 3; ++i) {
            bt::require(std::abs(actual_shank_inertia[i] - exp_shank_inertia[i]) < 1e-8,
                        "shank principal inertia matches de Leva");
        }

        // Foot geometry and inertia
        const auto connect_id = static_cast<std::size_t>(id(*model, mjOBJ_EQUALITY, ("connect_foot_" + std::string(side)).c_str()));
        const auto eq_data = std::views::counted(model->eq_data, static_cast<std::ptrdiff_t>(model->neq * mjNEQDATA));
        const double foot_vec_x = eq_data[connect_id * mjNEQDATA + 0];
        const double foot_vec_z = eq_data[connect_id * mjNEQDATA + 2];
        bt::require(std::abs(std::hypot(foot_vec_x, foot_vec_z) - 0.115) < 1e-6, "foot link length is 0.115 m");
        bt::require(std::abs(body_ipos[foot_id * 3 + 0] - 0.5 * foot_vec_x) < 1e-6, "foot COM x is centered on ankle link");
        bt::require(std::abs(body_ipos[foot_id * 3 + 1]) < 1e-9, "foot COM y is planar 0");
        bt::require(std::abs(body_ipos[foot_id * 3 + 2] - 0.5 * foot_vec_z) < 1e-6, "foot COM z remains at midpoint of 0.115 m link");

        constexpr double kRealFootLengthM = 0.2668;
        const double foot_mass = body_mass[foot_id];
        const double exp_foot_long = foot_mass * (0.124 * kRealFootLengthM) * (0.124 * kRealFootLengthM);
        const double exp_foot_trans = foot_mass * (0.245 * kRealFootLengthM) * (0.245 * kRealFootLengthM);
        const double exp_foot_sag = foot_mass * (0.257 * kRealFootLengthM) * (0.257 * kRealFootLengthM);
        std::array<double, 3> exp_foot_inertia{exp_foot_long, exp_foot_trans, exp_foot_sag};
        std::ranges::sort(exp_foot_inertia);
        std::array<double, 3> actual_foot_inertia{
            body_inertia[foot_id * 3 + 0],
            body_inertia[foot_id * 3 + 1],
            body_inertia[foot_id * 3 + 2]};
        std::ranges::sort(actual_foot_inertia);
        for (std::size_t i = 0; i < 3; ++i) {
            bt::require(std::abs(actual_foot_inertia[i] - exp_foot_inertia[i]) < 1e-8,
                        "foot principal inertia matches de Leva from real foot length");
        }
    }
}
// With lock_ankles, both feet are fixed at the same shank-to-foot angle
// (the front leg's build-pose angle), giving equal knee-to-pedal-spindle
// distance L2 on both legs and consistent connect_foot_* anchors.
void symmetric_locked_ankles_equal_knee_to_spindle_distance() {
    const auto request = physical("viewer_physics_welded.toml");
    const auto pose = mdl::geometry_pose(*request->rider, request->bike); // NOLINT(bugprone-unchecked-optional-access)
    const double pose_l2_front = std::hypot(
        pose.pedal_front[0] - pose.knee_front[0],
        pose.pedal_front[2] - pose.knee_front[2]);
    const double pose_l2_rear = std::hypot(
        pose.pedal_rear[0] - pose.knee_rear[0],
        pose.pedal_rear[2] - pose.knee_rear[2]);
    bt::require(std::abs(pose_l2_front - pose_l2_rear) < 1e-9,
                "geometry_pose knee-to-pedal-spindle distance equal on both legs (front=" +
                    std::to_string(pose_l2_front) + ", rear=" + std::to_string(pose_l2_rear) + ")");

    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(mj_makeData(model.get()), mj_deleteData);
    mj_resetData(model.get(), data.get());
    mj_kinematics(model.get(), data.get());
    const auto xpos = std::views::counted(data->xpos, static_cast<std::ptrdiff_t>(model->nbody * 3));
    const auto eq_data = std::views::counted(model->eq_data, static_cast<std::ptrdiff_t>(model->neq * mjNEQDATA));

    const auto leg_metrics = [&](const char* side) {
        const auto shank_id = static_cast<std::size_t>(id(*model, mjOBJ_BODY, ("rider_shank_" + std::string(side)).c_str()));
        const auto foot_id = static_cast<std::size_t>(id(*model, mjOBJ_BODY, ("rider_foot_" + std::string(side)).c_str()));
        const auto pedal_id = static_cast<std::size_t>(id(*model, mjOBJ_BODY, ("pedal_" + std::string(side)).c_str()));
        const auto connect_id = static_cast<std::size_t>(id(*model, mjOBJ_EQUALITY, ("connect_foot_" + std::string(side)).c_str()));

        const double knee_x = xpos[shank_id * 3 + 0];
        const double knee_z = xpos[shank_id * 3 + 2];
        const double ankle_x = xpos[foot_id * 3 + 0];
        const double ankle_z = xpos[foot_id * 3 + 2];
        const double pedal_x = xpos[pedal_id * 3 + 0];
        const double pedal_z = xpos[pedal_id * 3 + 2];

        const double foot_dx = pedal_x - ankle_x;
        const double foot_dz = pedal_z - ankle_z;
        const double anchor_x = eq_data[connect_id * mjNEQDATA + 0];
        const double anchor_y = eq_data[connect_id * mjNEQDATA + 1];
        const double anchor_z = eq_data[connect_id * mjNEQDATA + 2];
        bt::require(std::abs(anchor_x - foot_dx) < 1e-6 &&
                        std::abs(anchor_y) < 1e-9 &&
                        std::abs(anchor_z - foot_dz) < 1e-6,
                    std::string("connect_foot_") + side + " anchor matches pedal-minus-ankle vector");

        const double shank_dx = ankle_x - knee_x;
        const double shank_dz = ankle_z - knee_z;
        const double shank_len = std::hypot(shank_dx, shank_dz);
        const double foot_len = std::hypot(foot_dx, foot_dz);
        bt::require(std::abs(foot_len - 0.115) < 1e-9, std::string(side) + " foot link length is 0.115 m");
        const double cos_bend = (shank_dx * foot_dx + shank_dz * foot_dz) / (shank_len * foot_len);
        const double l2 = std::hypot(pedal_x - knee_x, pedal_z - knee_z);
        return std::pair{l2, cos_bend};
    };

    const auto [l2_front, cos_front] = leg_metrics("front");
    const auto [l2_rear, cos_rear] = leg_metrics("rear");
    bt::require(std::abs(l2_front - l2_rear) < 1e-9,
                "compiled model knee-to-pedal-spindle distance equal on both legs (front=" +
                    std::to_string(l2_front) + ", rear=" + std::to_string(l2_rear) + ")");
    bt::require(std::abs(cos_front - cos_rear) < 1e-9,
                "shank-to-foot angle equal on both legs");
}
// Joint zero reference angles (neutral_anatomical_rad) must be derived from
// the build pose (solve_seated_pose / build_articulated_rider at qpos = 0),
// with no hard-coded neutral_anatomical_rad keys remaining in the JSON profiles.
void build_pose_neutral_angles() {
    const auto request = physical("viewer_physics_welded.toml");
    const auto pose = mdl::geometry_pose(*request->rider, request->bike); // NOLINT(bugprone-unchecked-optional-access)
    const auto& art = request->physics->articulated; // NOLINT(bugprone-unchecked-optional-access)
    const auto env_path = std::filesystem::path(*art.joint_envelope_path); // NOLINT(bugprone-unchecked-optional-access)
    const auto str_path = std::filesystem::path(*art.joint_strength_path); // NOLINT(bugprone-unchecked-optional-access)

    const auto envelopes =
        mdl::with_pose_neutral_angles(cfg::load_joint_envelopes(env_path), pose);
    const auto coordinates = mdl::with_pose_neutral_angles(
        cfg::load_strength_coordinates(str_path, cfg::kRiderJoints), pose);
    const auto rot_y = [](const mdl::Vec3& u, const mdl::Vec3& v) {
        return std::atan2(u[2] * v[0] - u[0] * v[2], u[0] * v[0] + u[2] * v[2]);
    };
    const auto sub3 = [](const mdl::Vec3& a, const mdl::Vec3& b) {
        return mdl::Vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    };
    const mdl::Vec3 v_torso = sub3(pose.shoulder, pose.hip);
    const mdl::Vec3 v_torso_down = sub3(pose.hip, pose.shoulder);
    const mdl::Vec3 v_ua = sub3(pose.elbow, pose.shoulder);
    const mdl::Vec3 v_fa = sub3(pose.grip, pose.elbow);
    const mdl::Vec3 v_tf = sub3(pose.knee_front, pose.hip);
    const mdl::Vec3 v_sf = sub3(pose.ankle_front, pose.knee_front);
    const mdl::Vec3 v_sf_rev = sub3(pose.knee_front, pose.ankle_front);
    const mdl::Vec3 v_ff = sub3(pose.pedal_front, pose.ankle_front);
    const mdl::Vec3 v_tr = sub3(pose.knee_rear, pose.hip);
    const mdl::Vec3 v_sr = sub3(pose.ankle_rear, pose.knee_rear);
    const mdl::Vec3 v_sr_rev = sub3(pose.knee_rear, pose.ankle_rear);
    const mdl::Vec3 v_fr = sub3(pose.pedal_rear, pose.ankle_rear);

    const std::array<std::pair<std::string_view, double>, 11> expected_neutral{{
        {"rider_hip_rear", -rot_y({0., 0., -1.}, v_tr)},
        {"rider_knee_rear", +rot_y(v_tr, v_sr)},
        {"rider_ankle_rear", +rot_y(v_sr_rev, v_fr)},
        {"rider_hip_front", -rot_y({0., 0., -1.}, v_tf)},
        {"rider_knee_front", +rot_y(v_tf, v_sf)},
        {"rider_ankle_front", +rot_y(v_sf_rev, v_ff)},
        {"rider_torso_hinge", +rot_y({0., 0., 1.}, v_torso)},
        {"rider_shoulder_left", -rot_y(v_torso_down, v_ua)},
        {"rider_elbow_left", -rot_y(v_ua, v_fa)},
        {"rider_shoulder_right", -rot_y(v_torso_down, v_ua)},
        {"rider_elbow_right", -rot_y(v_ua, v_fa)},
    }};

    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    const auto jnt_range = std::views::counted(model->jnt_range, static_cast<std::ptrdiff_t>(model->njnt * 2));

    for (const auto& [name_sv, exp_rad] : expected_neutral) {
        const std::string name(name_sv);
        const auto env_it = std::ranges::find_if(envelopes, [&](const auto& p) { return p.first == name; });
        const auto coord_it = std::ranges::find_if(coordinates, [&](const auto& p) { return p.first == name; });
        bt::require(env_it != envelopes.end() && coord_it != coordinates.end(),
                    "envelope and strength coordinate present for " + name);
        bt::require(std::abs(env_it->second.neutral_anatomical_rad - exp_rad) < 1e-12,
                    name + " envelope neutral_anatomical_rad matches build pose (actual=" +
                        std::to_string(env_it->second.neutral_anatomical_rad) +
                        ", expected=" + std::to_string(exp_rad) + ")");
        bt::require(std::abs(coord_it->second.neutral_anatomical_rad - exp_rad) < 1e-12,
                    name + " strength coordinate neutral_anatomical_rad matches build pose");
        bt::require(std::abs(cfg::anatomical_angle(env_it->second, 0.) - exp_rad) < 1e-12,
                    name + " anatomical_angle at qpos=0 equals build-pose angle");
        const int jid = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
        if (jid >= 0) {
            const auto q_range = cfg::joint_q_range(env_it->second);
            const auto base = static_cast<std::size_t>(jid) * 2;
            bt::require(std::abs(jnt_range[base] - q_range[0]) < 1e-12 &&
                            std::abs(jnt_range[base + 1] - q_range[1]) < 1e-12,
                        name + " compiled joint q range matches pose-derived envelope");
        }
    }

    // Raw JSON files must not carry hard-coded neutral_anatomical_rad keys.
    bt::require(!bt::text(env_path).contains("neutral_anatomical_rad"),
                "rider_joint_envelope_anatomical.json does not contain hard-coded neutral_anatomical_rad");
    bt::require(!bt::text(str_path).contains("neutral_anatomical_rad"),
                "rider_strength_reference.json does not contain hard-coded neutral_anatomical_rad");

    // Non-default rider (height 1.70 m, valid inseam 0.78 m) and non-default bike:
    // the runtime controller envelopes and strength_coordinates must match
    // pose_neutral_anatomical_rad of the actual pose the MJCF was built from,
    // to 1e-12, for every leg/arm/torso joint.
    {
        auto custom = physical("viewer_physics_welded.toml");
        if (!custom->rider.has_value() || !custom->physics.has_value())
            throw std::runtime_error("missing rider or physics section");
        auto& custom_rider = *custom->rider;
        auto& custom_physics = *custom->physics;
        custom_rider.height_m = 1.70;
        custom_rider.inseam_m = 0.78;
        custom->bike.reach = 460.0;
        custom_physics.articulated.pedal_attachment = "weld";
        const auto track = bike::terrain::flat();
        custom->field = bike::terrain::HeightFieldSpec::for_track(track);
        auto custom_model = mdl::compile_mjcf(mdl::generate_mjcf(*custom));
        const auto hfield_id = static_cast<std::size_t>(
            mdl::required_id(*custom_model, mjOBJ_HFIELD, "road"));
        const auto hfield_offset = std::views::counted(
            custom_model->hfield_adr, custom_model->nhfield)[hfield_id];
        const auto samples = bike::terrain::build_field_data(track, custom->field);
        const auto target = std::views::counted(
                                custom_model->hfield_data, custom_model->nhfielddata)
                                .subspan(static_cast<std::size_t>(hfield_offset),
                                         samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i)
            target[i] = static_cast<float>(samples[i]);

        const auto custom_pose = mdl::geometry_pose(custom_rider, custom->bike);
        const auto controller_val = runtime::project_rider_controller(
            custom_physics, *custom_model, custom_pose);
        const auto spindle_cfg = std::make_unique<::rider::SpindleConfig>(
            ::rider::decode_spindle_config(controller_val));
        const auto custom_jnt_range = std::views::counted(
            custom_model->jnt_range,
            static_cast<std::ptrdiff_t>(custom_model->njnt * 2));

        for (const auto name_sv : cfg::kRiderJoints) {
            const std::string name(name_sv);
            const double expected_rad =
                mdl::pose_neutral_anatomical_rad(custom_pose, name);
            bt::require(spindle_cfg->envelopes.contains(name),
                        "controller envelope present for " + name);
            bt::require(spindle_cfg->strength_coordinates.contains(name),
                        "controller strength coordinate present for " + name);
            const auto& env = spindle_cfg->envelopes.at(name);
            const auto& coord = spindle_cfg->strength_coordinates.at(name);
            bt::require(
                std::abs(env.neutral_anatomical_rad - expected_rad) < 1e-12,
                "non-default rider controller envelope neutral matches MJCF build pose for " +
                    name + " (actual=" + std::to_string(env.neutral_anatomical_rad) +
                    ", expected=" + std::to_string(expected_rad) + ")");
            bt::require(
                std::abs(coord.neutral_anatomical_rad - expected_rad) < 1e-12,
                "non-default rider controller strength coordinate neutral matches MJCF build pose for " +
                    name + " (actual=" + std::to_string(coord.neutral_anatomical_rad) +
                    ", expected=" + std::to_string(expected_rad) + ")");
            const int jid =
                mj_name2id(custom_model.get(), mjOBJ_JOINT, name.c_str());
            bt::require(jid >= 0, "compiled joint present for " + name);
            const auto q_range = cfg::joint_q_range(env);
            const auto base = static_cast<std::size_t>(jid) * 2;
            bt::require(
                std::abs(custom_jnt_range[base] - q_range[0]) < 1e-12 &&
                    std::abs(custom_jnt_range[base + 1] - q_range[1]) < 1e-12,
                "non-default rider compiled joint q range matches controller envelope for " +
                    name);
        }
    }
}

// Saddle height = 0.8557 * inseam gives 37.5 deg knee flexion at bottom dead
// centre in the build pose (chosen 2026-11-07, docs/agents/simulation-decisions.md #5).
void saddle_height_gives_37_5_deg_bdc_knee_flexion() {
    const cfg::RiderSpecs rider{};
    const mdl::BikeSpecs bike{};
    const auto pose = mdl::solve_seated_pose(bike, rider);
    const double l1 = rider.segment_length("thigh");
    const double l2_front = std::hypot(pose.pedal_front[0] - pose.knee_front[0],
                                       pose.pedal_front[2] - pose.knee_front[2]);
    const double l2_rear = std::hypot(pose.pedal_rear[0] - pose.knee_rear[0],
                                      pose.pedal_rear[2] - pose.knee_rear[2]);
    bt::require(std::abs(l2_front - l2_rear) < 1e-12,
                "equal knee-to-spindle length on both legs");
    const double r_crank = bike.crank_length / 1000.0;
    const double d_bdc = std::hypot(pose.hip[0], pose.hip[2]) + r_crank;
    const double cos_flex = (d_bdc * d_bdc - l1 * l1 - l2_front * l2_front) /
                            (2.0 * l1 * l2_front);
    const double theta_bdc_deg =
        std::acos(std::clamp(cos_flex, -1.0, 1.0)) * 180.0 / std::numbers::pi;
    bt::require(std::abs(theta_bdc_deg - 37.5) < 0.05,
                "build-pose knee flexion at bottom is 37.5 deg within 0.05 deg (actual=" +
                    std::to_string(theta_bdc_deg) + " deg)");
    bt::require(std::abs(rider.saddle_height_m() - 0.8557 * rider.inseam()) < 1e-12,
                "saddle_height_m equals 0.8557 * inseam (actual=" +
                    std::to_string(rider.saddle_height_m()) + ")");
    const auto geom = spindle::leg_loop_geometry(
        {pose.hip[0], pose.hip[2]},
        {pose.knee_front[0], pose.knee_front[2]},
        {pose.pedal_front[0], pose.pedal_front[2]}, true);
    const auto q_bdc = spindle::leg_joint_q(
        geom, {pose.hip[0], pose.hip[2]}, {0.0, -r_crank}, 0.0);
    const double neutral_knee_rad =
        mdl::pose_neutral_anatomical_rad(mdl::geometry_pose(rider, bike),
                                         "rider_knee_front");
    const double controller_bdc_deg =
        (neutral_knee_rad + q_bdc[1]) * 180.0 / std::numbers::pi;
    bt::require(std::abs(pose.knee_flexion_bdc_deg - controller_bdc_deg) < 1e-12,
                "solve_seated_pose BDC knee flexion matches locked-ankle controller geometry (actual=" +
                    std::to_string(pose.knee_flexion_bdc_deg) +
                    " deg, expected=" + std::to_string(controller_bdc_deg) +
                    " deg)");
}
} // namespace
int main(int argc, char* argv[]) {
    try {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
        const std::span<char*> args(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
        if (args.size() > 1 && std::string_view(args[1]) == "de_leva") {
            return bt::run([] { de_leva_leg_segment_com_and_inertia(); });
        }
        if (args.size() > 1 && std::string_view(args[1]) == "symmetric_ankles") {
            return bt::run([] { symmetric_locked_ankles_equal_knee_to_spindle_distance(); });
        }
        if (args.size() > 1 && std::string_view(args[1]) == "build_pose_neutral") {
            return bt::run([] { build_pose_neutral_angles(); });
        }
        if (args.size() > 1 && std::string_view(args[1]) == "saddle_bdc_flexion") {
            return bt::run([] { saddle_height_gives_37_5_deg_bdc_knee_flexion(); });
        }
        return bt::run([]{de_leva_leg_segment_com_and_inertia();symmetric_locked_ankles_equal_knee_to_spindle_distance();build_pose_neutral_angles();saddle_height_gives_37_5_deg_bdc_knee_flexion();captured_welded();physical_profiles_and_topology();no_joint_damping_in_any_mode();seated_variant_is_gone();crank_zero_is_horizontal();drivetrain_has_two_one_way_clutches();rotor_body_welds_to_the_drive_axis();motor_clutch_engages_forward_only();errors_and_ownership();});
    } catch (const std::exception& e) {
        std::cerr << "FAIL " << e.what() << '\n';
        return 1;
    }
}
