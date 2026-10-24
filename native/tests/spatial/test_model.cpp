#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "support/model_request.hpp"
#include "support/spatial_fixture.hpp"
#include "configuration/load.hpp"
#include "io/value.hpp"
#include "model/mapping.hpp"
#include "model_access.hpp"
#include "spatial/topology.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
namespace mdl=bike::model;
namespace sp=bike::spatial;
struct MassProperties {
    double mass=0.;
    std::array<double,3> centre{};
    std::array<double,9> inertia{};
};
MassProperties properties(const mjModel& model,const mjData& data) {
    const auto masses=model_access::readonly_buffer(model.body_mass,model.nbody);
    const auto positions=model_access::readonly_buffer(data.xipos,3*model.nbody);
    const auto rotations=model_access::readonly_buffer(data.ximat,9*model.nbody);
    const auto inertia=model_access::readonly_buffer(model.body_inertia,3*model.nbody);
    MassProperties result;
    for(std::size_t body=1;body<masses.size();++body) {
        result.mass+=masses[body];
        for(std::size_t k=0;k<3;++k) result.centre[k]+=masses[body]*positions[3*body+k];
    }
    for(auto& coordinate:result.centre) coordinate/=result.mass;
    for(std::size_t body=1;body<masses.size();++body) {
        std::array<double,3> r{};
        double norm2=0.;
        for(std::size_t k=0;k<3;++k) {r[k]=positions[3*body+k]-result.centre[k];norm2+=r[k]*r[k];}
        for(std::size_t row=0;row<3;++row) for(std::size_t col=0;col<3;++col) {
            double rotated=0.;
            for(std::size_t k=0;k<3;++k)
                rotated+=rotations[9*body+3*row+k]*inertia[3*body+k]*rotations[9*body+3*col+k];
            result.inertia[3*row+col]+=rotated+masses[body]*((row==col?norm2:0.)-r[row]*r[col]);
        }
    }
    return result;
}
void inventory_and_mass() {
    auto constrained_request=spatial_tests::model_request(sp::RootKind::constrained4);
    auto free_request=spatial_tests::model_request(sp::RootKind::free6);
    bt::ModelFixture constrained(mdl::generate_mjcf(*constrained_request));
    bt::ModelFixture free(mdl::generate_mjcf(*free_request));
    engine::forward(constrained.model.get(),constrained.data.get());
    engine::forward(free.model.get(),free.data.get());
    const auto ct=sp::bind_topology(*constrained.model,constrained_request->spatial.value());
    const auto ft=sp::bind_topology(*free.model,free_request->spatial.value());
    bt::require(ct.root.kind==sp::RootKind::constrained4 && ft.root.kind==sp::RootKind::free6,"requested root kind is compiled");
    const std::array names{"root_x","root_y","root_z","root_pitch"};
    for(std::size_t k=0;k<names.size();++k) {
        const int joint=mdl::required_id(*constrained.model,mjOBJ_JOINT,names[k]);
        bt::require(joint==ct.root.scalar[0].joint_id+static_cast<int>(k),"constrained roots retain x/y/z/pitch order");
    }
    bt::require(mj_name2id(constrained.model.get(),mjOBJ_JOINT,"steer_joint")==-1,"constrained steering stays welded");
    const auto types=model_access::readonly_buffer(free.model->jnt_type,free.model->njnt);
    bt::require(types[static_cast<std::size_t>(ft.root.free_joint_id)]==mjJNT_FREE,"free root is a true 7/6 joint");
    bt::require(types[static_cast<std::size_t>(ft.steer.joint.joint_id)]==mjJNT_HINGE,"steering is a physical hinge");
    const auto targets=model_access::readonly_buffer(free.model->actuator_trnid,2*free.model->nu);
    const auto ranges=model_access::readonly_buffer(free.model->actuator_ctrlrange,2*free.model->nu);
    const auto actuator=static_cast<std::size_t>(ft.steer.actuator_id);
    bt::require(targets[2*actuator]==ft.steer.joint.joint_id,"steer torque targets physical steering");
    bt::require(ranges[2*actuator]==-12.25 && ranges[2*actuator+1]==12.25,"steering cap is symmetric");
    const auto steering_ranges=model_access::readonly_buffer(free.model->jnt_range,2*free.model->njnt);
    const auto sj=static_cast<std::size_t>(ft.steer.joint.joint_id);
    bt::require(std::abs(steering_ranges[2*sj]+std::numbers::pi/3.)<1e-12 &&
        std::abs(steering_ranges[2*sj+1]-std::numbers::pi/3.)<1e-12,"steering range uses compiler radians");
    const auto a=properties(*constrained.model,*constrained.data),b=properties(*free.model,*free.data);
    bt::require(std::abs(a.mass-b.mass)<1e-9,"root choice preserves total mass");
    for(std::size_t k=0;k<3;++k) bt::require(std::abs(a.centre[k]-b.centre[k])<1e-9,"root choice preserves rest centre of mass");
    for(std::size_t k=0;k<9;++k) bt::require(std::abs(a.inertia[k]-b.inertia[k])<1e-9,"root choice preserves world inertia tensor");
    bt::require(free.model->opt.timestep==constrained.model->opt.timestep &&
        free.model->opt.timestep==free_request->physics->timestep_s,"root choice preserves integration cadence");
    const std::span free_gravity{free.model->opt.gravity},constrained_gravity{constrained.model->opt.gravity};
    for(std::size_t k=0;k<3;++k) bt::require(free_gravity[k]==constrained_gravity[k],"root choice preserves gravity");
    bt::require(ft.imu_site_id>=0,"spatial IMU site is bound");
    (void)mdl::required_id(*free.model,mjOBJ_SENSOR,"imu_acc_spatial");
    (void)mdl::required_id(*free.model,mjOBJ_SENSOR,"imu_gyro_spatial");
    const auto affinity=model_access::readonly_buffer(free.model->geom_conaffinity,free.model->ngeom);
    const auto masks=model_access::readonly_buffer(free.model->geom_contype,free.model->ngeom);
    const auto terrain=static_cast<std::size_t>(mdl::required_id(*free.model,mjOBJ_GEOM,"terrain"));
    for(const char* name:{"geom_front_contact","geom_rear_contact"}) {
        const auto wheel=static_cast<std::size_t>(mdl::required_id(*free.model,mjOBJ_GEOM,name));
        bt::require(masks[wheel]==2 && affinity[wheel]==0 && (masks[wheel]&affinity[terrain])==0,
            "physical tyre excludes engine terrain contacts");
    }
}
void rejections() {
    auto request=spatial_tests::model_request(sp::RootKind::free6);
    request->physics->tires.backend="pneumatic";
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
    request=spatial_tests::model_request(sp::RootKind::constrained4);
    request->spatial->steer.controller=sp::SteerController::roll_heading;
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
}
void spatial_configuration_sources() {
    auto request=spatial_tests::model_request(sp::RootKind::free6);
    const auto reject=[&](std::string_view field) {
        bt::invalid([&] {
            try { (void)mdl::generate_mjcf(*request); }
            catch(const std::invalid_argument& error) {
                bt::require(std::string_view(error.what()).starts_with(field),
                    "configuration rejection identifies the conflicting declaration");
                throw;
            }
        });
    };
    const auto explicit_model=mdl::generate_mjcf(*request);
    request->physics->spatial=request->spatial;
    request->spatial.reset();
    bt::require(mdl::generate_mjcf(*request)==explicit_model,
        "physics-only spatial configuration builds the same free root, steering and terrain");
    request->spatial=request->physics->spatial;
    bt::require(mdl::generate_mjcf(*request)==explicit_model,
        "matching model and physics declarations build the same spatial model");
    request->spatial->root=sp::RootKind::constrained4;
    reject("model.spatial:");
    request->spatial=sp::Config{};
    reject("model.spatial:");
    request->spatial.reset();
    request->physics->spatial.reset();
    request->physics->tires.backend="pneumatic";
    reject("model.terrain3d:");
}
void profile_strength_cap() {
    auto request=spatial_tests::model_request(sp::RootKind::free6);
    request->spatial->steer.torque_cap_nm.reset();
    request->physics->articulated.shoulder_profile=
        spatial_tests::fixture("model/shoulder_strength_17.json").string();
    bt::ModelFixture rig(mdl::generate_mjcf(*request));
    const auto actuator=static_cast<std::size_t>(mdl::required_id(*rig.model,mjOBJ_ACTUATOR,"steer_torque"));
    const auto ranges=model_access::readonly_buffer(rig.model->actuator_ctrlrange,2*rig.model->nu);
    bt::require(ranges[2*actuator]==-17.75 && ranges[2*actuator+1]==17.75,
        "strength steering cap follows the authored profile rather than a fixed fallback");
    request->physics->articulated.shoulder_profile_sha256=std::string(64,'0');
    bt::invalid([&]{(void)mdl::generate_mjcf(*request);});
}
void planar_bytes() {
    // These are exactly the pre-spatial T16 input choices. The historical
    // migration XML is an independently compiled Python reference, not a
    // claim that native text was captured during the source-only W phase.
    auto request=std::make_unique<mdl::BuildRequest>();
    request->mode="ride";
    request->rider=bike::configuration::RiderSpecs{};
    request->rider->variant="articulated_planar";
    request->physics=bike::configuration::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
    request->field.ncol=26001;request->field.radius_x_m=65.;request->field.elevation_m=30.1;
    const auto absent=mdl::generate_mjcf(*request);
    request->spatial=sp::Config{};
    bt::require(absent==mdl::generate_mjcf(*request),"explicit planar extension does not change legacy bytes");
    request->physics->spatial=request->spatial;
    bt::require(absent==mdl::generate_mjcf(*request),"matching explicit planar declarations preserve legacy bytes");
    const auto manifest=spatial_tests::json(spatial_tests::fixture("model/manifest.json"));
    const auto& object=bike::io::as_object(manifest,"model fixture manifest");
    const auto* identity=bike::io::find_member(object,"native_mjcf_sha256_hex");
    bt::require(identity && !bike::io::is_null(*identity),"pre-W13 native planar XML acquisition is pending V03");
    const auto expected=spatial_tests::text(spatial_tests::fixture("model/planar_mjcf_expected.xml"));
    bt::require(spatial_tests::sha256_hex(std::as_bytes(std::span(expected.data(),expected.size())))==
        bike::io::as_string(*identity,"native_mjcf_sha256_hex"),"frozen native planar XML digest");
    bt::require(absent==expected,"native planar XML is byte-exact with pre-W13 capture");
}
} // namespace

int main() {return bt::run([]{inventory_and_mass();rejections();spatial_configuration_sources();profile_strength_cap();planar_bytes();});}
