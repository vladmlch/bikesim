// T16: generation from typed inputs, compared to an independently captured
// Python MJCF through the same pinned native compiler. No fixture is updated.
#include "support/migration_test.hpp"
#include "configuration/load.hpp"
#include "model/builder.hpp"
#include "model/xml_writer.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
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
void equal_array(std::string_view field,const T* actual,const T* expected,std::size_t count) {
    const auto a=std::views::counted(actual,static_cast<std::ptrdiff_t>(count));
    const auto b=std::views::counted(expected,static_cast<std::ptrdiff_t>(count));
    for(std::size_t index=0;index<count;++index) {
        bool equal;
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
    const Named named[]{
        {mjOBJ_BODY,static_cast<std::size_t>(actual.nbody),"body"},
        {mjOBJ_JOINT,static_cast<std::size_t>(actual.njnt),"joint"},
        {mjOBJ_GEOM,static_cast<std::size_t>(actual.ngeom),"geom"},
        {mjOBJ_SITE,static_cast<std::size_t>(actual.nsite),"site"},
        {mjOBJ_ACTUATOR,static_cast<std::size_t>(actual.nu),"actuator"},
        {mjOBJ_EQUALITY,static_cast<std::size_t>(actual.neq),"equality"},
        {mjOBJ_TENDON,static_cast<std::size_t>(actual.ntendon),"tendon"},
        {mjOBJ_SENSOR,static_cast<std::size_t>(actual.nsensor),"sensor"},
    };
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
void captured_welded() {
    bt::manifest_verify_files(bt::manifest_entry("model/welded.xml"));
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
    auto request=physical();request->physics->articulated.joint_envelope_path.reset();
    request->physics->articulated.pedal_attachment="weld";request->physics->articulated.saddle_attachment="weld";request->physics->articulated.grip_attachment="spring";
    const auto welded=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    id(*welded,mjOBJ_JOINT,"rider_ankle_front");id(*welded,mjOBJ_EQUALITY,"weld_foot_front");id(*welded,mjOBJ_EQUALITY,"weld_saddle");
    bt::require(welded->nq==28,"welded foot preserves ankle DOFs");
    request->physics->drive.transmission_model="elastic_chain";
    request->physics->drive.shifting.enabled=false;
    const auto elastic=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    id(*elastic,mjOBJ_BODY,"cassette");
    bt::require(mj_name2id(elastic.get(),mjOBJ_TENDON,"ideal_mid_drive_freehub")==-1,"elastic chain has no ideal freehub constraint");
    const auto wheel_id=static_cast<std::size_t>(id(*elastic,mjOBJ_BODY,"rear_wheel"));
    const auto cassette_id=static_cast<std::size_t>(id(*elastic,mjOBJ_BODY,"cassette"));
    const auto masses=std::views::counted(elastic->body_mass,elastic->nbody);
    bt::require(masses[wheel_id]==request->masses.rear_wheel_mass*.9,"rear wheel split retains 90 percent budget");
    bt::require(masses[cassette_id]==request->masses.rear_wheel_mass*.1,"cassette transfers 10 percent budget");
    request=physical();request->physics->drive.motor_clutch=true;
    const auto clutch=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    id(*clutch,mjOBJ_JOINT,"drive_shaft_spin");id(*clutch,mjOBJ_TENDON,"crank_clutch");
    request=physical();request->physics->drive.rotor_inertia_kgm2=.2;
    const auto rotor=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    id(*rotor,mjOBJ_JOINT,"rotor_spin");id(*rotor,mjOBJ_TENDON,"motor_freewheel");
    request->physics->drive.motor_clutch=true;
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=physical();request->physics->drive.transmission_model="geometric_ideal_mid_drive";
    const auto geometric=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    const auto tendon=static_cast<std::size_t>(id(*geometric,mjOBJ_TENDON,"geometric_mid_drive_freehub"));
    bt::require(std::views::counted(geometric->tendon_num,geometric->ntendon)[tendon]==geometric->njnt,"geometric drive includes all scalar joints and rider zero rows");
    request=physical();request->masses.front_wheel_mass=3.1;request->masses.rear_wheel_mass=3.7;
    request->masses.frame_structure_mass=4.2;request->masses.battery_mass=5.1;
    const auto reweighted=mdl::compile_mjcf(mdl::generate_mjcf(*request));
    const auto weighted_masses=std::views::counted(reweighted->body_mass,reweighted->nbody);
    bt::require(weighted_masses[static_cast<std::size_t>(id(*reweighted,mjOBJ_BODY,"front_wheel"))]==3.1,"front explicit inertial uses requested budget");
    double total=0.;for(double mass:weighted_masses.subspan(1)) total+=mass;
    bt::require(std::abs(total-(request->masses.total_bike_mass()+request->rider->mass_kg))<1e-11,"physical component and rider mass budgets close");
}
void errors_and_ownership() {
    auto request=physical();request->mode="stand";
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=physical();request->tyre_model="pneumatic";
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=physical();request->rider->variant="lumped";
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=std::make_unique<mdl::BuildRequest>();request->crank_joint=true;
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=std::make_unique<mdl::BuildRequest>();request->tyre_model="unknown";
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
    mdl::xml::Element escaped("mujoco",{{"model","A & B < \"quoted\"\n"}});
    const auto text=mdl::xml::document(escaped);
    bt::require(text.find("A &amp; B &lt; &quot;quoted&quot;&#10;")!=std::string::npos,"XML attributes escape reserved characters and whitespace");
}
} // namespace
int main() {return bt::run([]{captured_welded();physical_profiles_and_topology();errors_and_ownership();});}
