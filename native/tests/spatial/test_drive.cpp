#include "support/migration_test.hpp"
#include "support/model_request.hpp"
#include "support/startup_fixture.hpp"
#include "../reference/drivetrain_reference_support.hpp"
#include "configuration/load.hpp"
#include "drivetrain/state_codec.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"
#include "model/mapping.hpp"
#include "model_access.hpp"
#include "runtime/project_config.hpp"
#include "spatial/topology.hpp"
#include "writer_config.hpp"
#include "writers/brake.hpp"
#include "writers/drivetrain.hpp"
#include "writers/resistance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>

namespace {
namespace sp=bike::spatial;
struct Scenario {double spin;double speed;bool braking;std::string_view mode;};
struct DriveRig {
    bike::model::ModelOwner model;
    drivetrain::OwnedData data;
    sp::Topology topology;
    std::unique_ptr<drivetrain::DrivetrainWriter> writer;
};
void position(mjModel& model,mjData& data,const char* joint,double value) {
    const auto address=bike::model::joint_address(model,joint).first;
    model_access::mutable_buffer(data.qpos,model.nq)[static_cast<std::size_t>(address)]=value;
}
void velocity(mjModel& model,mjData& data,const char* joint,double value) {
    const auto address=bike::model::joint_address(model,joint).second;
    model_access::mutable_buffer(data.qvel,model.nv)[static_cast<std::size_t>(address)]=value;
}
std::unique_ptr<DriveRig> vehicle(std::string_view family,const Scenario& scenario,double yaw) {
    auto request=spatial_tests::model_request(sp::RootKind::free6);
    request->physics->drive.transmission_model=family;
    request->physics->drive.human_torque_nm=20.;
    request->physics->drive_mode=scenario.mode;
    auto rig=std::make_unique<DriveRig>();
    rig->model=bike::model::compile_mjcf(bike::model::generate_mjcf(*request));
    rig->data.reset(engine::make_data(rig->model.get()));
    bt::require(rig->data!=nullptr,"spatial drivetrain owns data");
    rig->topology=sp::bind_topology(*rig->model,request->spatial.value());
    auto q=model_access::mutable_buffer(rig->data->qpos,rig->model->nq);
    const auto root=static_cast<std::size_t>(rig->topology.root.free_qpos_adr);
    q[root+3]=std::cos(yaw/2.);q[root+4]=0.;q[root+5]=0.;q[root+6]=std::sin(yaw/2.);
    position(*rig->model,*rig->data,"crank_spin",.3);
    position(*rig->model,*rig->data,"rear_wheel_spin",.6);
    velocity(*rig->model,*rig->data,"crank_spin",scenario.spin);
    velocity(*rig->model,*rig->data,"rear_wheel_spin",1.3*scenario.spin);
    velocity(*rig->model,*rig->data,"front_wheel_spin",1.3*scenario.spin);
    if(family=="elastic_chain") {
        position(*rig->model,*rig->data,"cassette_spin",.4);
        velocity(*rig->model,*rig->data,"cassette_spin",1.1*scenario.spin);
    }
    engine::forward(rig->model.get(),rig->data.get());
    using bike::io::Value;
    const Value wire{Value::Object{{"schema",Value{bike::io::Number{"2"}}},
        {"drive",runtime::project_drive_config(request->physics.value())}}};
    const auto config=std::make_unique<nativecfg::WriterConfig>(nativecfg::decode(wire));
    bt::require(config->drive.has_value(),"spatial source projects the native drivetrain");
    rig->writer=std::make_unique<drivetrain::DrivetrainWriter>(rig->model.get(),rig->data.get(),config->drive.value());
    rig->writer->reset();
    if(family=="elastic_chain") {
        // Positive chain stretch after reset exercises a nonzero force;
        // a slack-chain zero row cannot prove rotation covariance.
        position(*rig->model,*rig->data,"crank_spin",.325);
        engine::forward(rig->model.get(),rig->data.get());
    }
    return rig;
}
void drive_covariance() {
    constexpr std::array scenarios{
        Scenario{6.,2.,false,"crank_effort"},Scenario{-6.,-2.,false,"crank_effort"},
        Scenario{6.,2.,false,"coast"},Scenario{6.,2.,true,"crank_effort"},Scenario{0.,0.,true,"coast"}};
    for(const auto family:{"ideal_mid_drive","geometric_ideal_mid_drive","elastic_chain"})
        for(const auto& scenario:scenarios) {
            auto a=vehicle(family,scenario,0.),b=vehicle(family,scenario,std::numbers::pi/2.);
            const auto qa=a->writer->components({},.001,scenario.speed,scenario.braking,true,true,0.,{},true,{});
            const auto qb=b->writer->components({},.001,scenario.speed,scenario.braking,true,true,0.,{},true,{});
            bt::require(qa.size()==qb.size(),"heading preserves drivetrain component inventory");
            for(std::size_t row=0;row<qa.size();++row) {
                bt::require(qa[row].first==qb[row].first && qa[row].second.size()==qb[row].second.size(),"heading preserves force row identities");
                for(std::size_t col=0;col<qa[row].second.size();++col)
                    bt::require(std::abs(qa[row].second[col]-qb[row].second[col])<=1e-12,"local drivetrain torque is heading invariant");
            }
            const auto ca=model_access::readonly_buffer(a->data->ctrl,a->model->nu);
            const auto cb=model_access::readonly_buffer(b->data->ctrl,b->model->nu);
            for(std::size_t k=0;k<ca.size();++k)
                bt::require(std::abs(ca[k]-cb[k])<=1e-12,"heading preserves crank and motor commands");
            if(std::string_view(family)=="geometric_ideal_mid_drive" && scenario.mode=="crank_effort") {
                const auto saved=std::make_unique<drivetrain::DriveSnapshot>(a->writer->state());
                bt::require(saved->ideal_hub && saved->ideal_hub->prepared,"geometric free root has a prepared manifold snapshot");
                const auto& prepared=saved->ideal_hub->prepared.value();
                bt::require(prepared.qpos.size()==static_cast<std::size_t>(a->model->nq) &&
                    prepared.jacobian.size()==static_cast<std::size_t>(a->model->nv),"prepared qpos and Jacobian use distinct widths");
                const auto before=bike::io::canonical_json(drivetrain::encode_drive_snapshot(*saved));
                auto wrong=std::make_unique<drivetrain::DriveSnapshot>(*saved);
                wrong->ideal_hub->prepared->qpos.pop_back();
                bt::invalid([&]{a->writer->restore(*wrong);});
                bt::require(before==bike::io::canonical_json(drivetrain::encode_drive_snapshot(a->writer->state())),"invalid manifold snapshot preserves drive state");
            }
        }
}
void brakes_and_resistance() {
    sp::Config config;config.root=sp::RootKind::free6;
    auto rig=spatial_tests::free_rig(config,false);
    BrakeWriter brakes(rig.model,{.torque_ceiling_nm=40.,.taper_radps=2.});
    auto q=model_access::mutable_buffer(rig.data->qpos,rig.model->nq);
    auto v=model_access::mutable_buffer(rig.data->qvel,rig.model->nv);
    const auto qa=static_cast<std::size_t>(rig.topology.root.free_qpos_adr);
    const auto va=static_cast<std::size_t>(rig.topology.root.free_dof_adr);
    position(*rig.model,*rig.data,"steer_joint",.3);
    velocity(*rig.model,*rig.data,"steer_joint",1.2);
    for(double spin:{-4.,0.,4.}) {
        std::array<std::pair<double,double>,2> torque{};
        std::size_t heading=0;
        for(double yaw:{0.,std::numbers::pi/2.}) {
            q[qa+3]=std::cos(yaw/2.);q[qa+4]=0.;q[qa+5]=0.;q[qa+6]=std::sin(yaw/2.);
            v[va+3]=.7;v[va+4]=5.2;v[va+5]=-.4;
            v[static_cast<std::size_t>(rig.topology.wheels[0].spin.dof_adr)]=spin;
            v[static_cast<std::size_t>(rig.topology.wheels[1].spin.dof_adr)]=-spin;
            engine::forward(rig.model,rig.data);
            torque[heading++]=brakes.torques(rig.data,.5,1.);
        }
        const double sign=spin>0.?1.:spin<0.?-1.:0.;
        bt::require(std::abs(torque[0].first+20.*sign)<1e-12 && std::abs(torque[0].second-40.*sign)<1e-12,
            "braking opposes relative spin even while the frame and steering rotate");
        bt::require(std::abs(torque[0].first-torque[1].first)<1e-12 && std::abs(torque[0].second-torque[1].second)<1e-12,
            "braking torque is invariant under 90 degree heading rotation");
    }
    const nativecfg::ResistanceConfig resistance{.crr=.02,.rolling_taper_rad_s=.2,.rho_kg_m3=1.2,.cda_m2=.5,
        .wind_world_mps={3.,4.,-1.},.point_body_m={.2,.3,.4},.frame_body="frame",
        .front_wheel_body="front_wheel",.rear_wheel_body="rear_wheel"};
    ResistanceWriter writer(rig.model,resistance);
    const std::array loads{400.};const std::array working{true};
    const TireSideInput input{loads,working,.3};
    const auto components=writer.components(rig.data,input,input);
    for(const auto& [name,row]:components) {
        (void)name;
        bt::require(std::ranges::all_of(row,[](double value){return value==0.;}),"C4 tyre owns rolling loss and spatial air drag remains disabled");
    }
}
void resistance_source_boundary() {
    namespace cfg=bike::configuration;
    auto request=spatial_tests::model_request(sp::RootKind::free6);
    auto& config=request->physics.value();
    config.spatial=request->spatial;
    config.resistance.wind_world_mps={3.,4.,-1.};
    config.resistance.point_body_m={.2,.3,.4};
    cfg::post_init(config);
    const auto document=cfg::detail::parse_toml(cfg::write_physics_toml(config),"spatial-resistance");
    auto restored=std::unique_ptr<cfg::SimulationPhysicsConfig>(
        new cfg::SimulationPhysicsConfig(cfg::detail::construct_config(document)));
    bt::require(restored->resistance.wind_world_mps==config.resistance.wind_world_mps &&
        restored->resistance.point_body_m==config.resistance.point_body_m,
        "source TOML preserves finite spatial resistance vectors before writer dispatch");
    restored->spatial.reset();
    restored->tires.backend="pneumatic";
    bt::invalid([&]{cfg::post_init(*restored);});
    const auto planar=cfg::detail::parse_toml(cfg::write_physics_toml(*restored),"planar-resistance");
    bt::invalid([&]{(void)cfg::detail::construct_config(planar);});
}
void planar_oracle() {
    // Reuse the independently recorded pre-spatial force-staging inputs and
    // exact f64 sites. Missing platform captures fail closed in Node; no
    // current implementation output is accepted as its own expected value.
    using namespace drivetrain_reference;
    for(const auto family:kinds) {
        auto golden=node("test_force_staging","crank_effort-"+std::string(family));
        auto owner=pair(family);
        components(golden,components(*owner,.002,2.));
        array(golden,owner->ctrl());tree(golden,diagnostics(*owner));tree(golden,stored_energy(*owner));golden.done();
    }
}
} // namespace

int main() {return bt::run([]{drive_covariance();brakes_and_resistance();resistance_source_boundary();planar_oracle();});}
