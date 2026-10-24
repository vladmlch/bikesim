// W26: hinge and ground stiction have different owners. Numerical gates V04.
#include "support/migration_test.hpp"
#include "spatial/support/startup_fixture.hpp"
#include "spatial/support/terrain_fixture.hpp"
#include "spatial/support/tyre_fixture.hpp"
#include "runtime/static_brake.hpp"
#include "runtime/accounting.hpp"
#include "spatial/brake_state.hpp"
#include "writers/pneumatic_3d_tire.hpp"
#include "model/builder.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

namespace {
namespace sp=bike::spatial;
namespace pn=bike::pneumatic3d;
std::string bytes(const sp::BrakeState& state) {return bike::io::canonical_json(sp::encode_brake(state));}
void hinge_contracts() {
    sp::Config cfg;cfg.mode=sp::Mode::spatial3d;cfg.root=sp::RootKind::constrained4;cfg.steer.controller=sp::SteerController::off;
    auto rig=spatial_tests::constrained_rig(cfg);
    const auto f=rig.topology.wheels[0].spin.dof_adr,r=rig.topology.wheels[1].spin.dof_adr;
    runtime::StaticBrake brake(f,r,40.,{.torque_nm=30.,.stiffness_nm_rad=1000.,.damping_nms_rad=10.});
    std::vector<double> forces(static_cast<std::size_t>(rig.model->nv));
    std::array<double,2> rates{};
    const auto initial=bytes(brake.capture_spatial_brake());
    (void)brake.apply_spatial(rig.model,rates,1.,.5,.001,0,false,forces);
    bt::require(initial==bytes(brake.capture_spatial_brake()),"probe cannot engage a committed latch");
    auto eval=brake.apply_spatial(rig.model,rates,1.,.5,.001,0,true,forces);
    bt::require(brake.capture_spatial_brake().wheel[0].engaged,"standing wheel latches");
    for(std::int64_t i=1;i<201;++i) {
        rates={.2,-.2};std::ranges::fill(forces,0.);
        const double previous_stored=eval.stored_energy_j;
        eval=brake.apply_spatial(rig.model,rates,1.,.5,.001,i,true,forces);
        for(std::size_t w=0;w<2;++w)
            bt::require(std::abs(eval.hold_torque_nm[w])+eval.friction_budget_nm[w]<=40.*(w==0?1.:.5)+1e-12,
                "bite and solver friction share one lever budget in both directions");
        const double pad_work=(eval.hold_torque_nm[0]*rates[0]+eval.hold_torque_nm[1]*rates[1])*.001;
        bt::require(std::abs(pad_work+eval.stored_energy_j-previous_stored+
            eval.damping_heat_j+eval.release_heat_j)<=1e-12,
            "elastic loading and saturated pad yield close the interval work balance");
    }
    bt::require(brake.capture_spatial_brake().wheel[0].unwrapped_angle_rad>0.&&
        brake.capture_spatial_brake().wheel[1].unwrapped_angle_rad<0.,"relative wheel angle integrates signed axis rates");
    const auto checkpoint=brake.capture_spatial_brake();
    runtime::StaticBrake restored(f,r,40.,brake.hold_spec());restored.restore_spatial_brake(checkpoint);
    bt::require(bytes(restored.capture_spatial_brake())==bytes(checkpoint),"latch capture/restore is exact");
    auto invalid=checkpoint;invalid.wheel[0].stored_energy_j=std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&]{restored.restore_spatial_brake(invalid);});
    bt::require(bytes(restored.capture_spatial_brake())==bytes(checkpoint),"invalid restore cannot partially replace latch");
    bt::invalid([&]{(void)brake.apply_spatial(rig.model,rates,1.,.5,.001,200,true,forces);});
    bt::require(bytes(brake.capture_spatial_brake())==bytes(checkpoint),"duplicate interval has no state effect");
    std::ranges::fill(forces,0.);eval=brake.apply_spatial(rig.model,rates,0.,0.,.001,201,true,forces);
    bt::require(!brake.capture_spatial_brake().wheel[0].engaged&&eval.stored_energy_j==0.&&
        eval.hold_torque_nm[0]==0.&&eval.friction_budget_nm[0]==0.,"lever release has no world spring or residual bite");
    bt::require(eval.release_heat_j==checkpoint.wheel[0].stored_energy_j+checkpoint.wheel[1].stored_energy_j,
        "release removes exactly the elastic pad energy");
    const auto energy=runtime::spatial_brake_energy(0.,eval.damping_heat_j,eval.release_heat_j,-5.,.001);
    bt::require(energy.friction_heat_j==.005&&energy.release_work_j==-eval.release_heat_j,
        "hinge friction and pad release are separate energy lanes");
    std::int64_t tick=202;
    for(auto rate:std::array{2.,-2.,.1,-.1}) {
        std::ranges::fill(forces,0.);rates={rate,rate};
        eval=brake.apply_spatial(rig.model,rates,1.,1.,.001,tick++,true,forces);
        bt::require(std::abs(rate)>1. ? !brake.capture_spatial_brake().wheel[0].engaged
                                     : brake.capture_spatial_brake().wheel[0].engaged,
            "stop/start/reverse retain latch hysteresis");
    }
}
// A purpose-built constrained wheel pair. Tilting gravity is the exact rigid
// rotation of a plane-slope problem, keeping the ideal rig's plane at z=0.
// Roll/yaw are constrained; root x/y/z/pitch and both wheel hinges are physical.
spatial_tests::RigModel slope_rig(double slope,double yaw,bool bank) {
    std::ostringstream xml;xml.precision(17);
    const double sx=bank?0.:std::sin(slope),sy=bank?std::sin(slope):0.;
    xml<<"<mujoco><compiler angle='radian'/><option timestep='.0005' gravity='"
       <<9.81*(std::cos(yaw)*sx-std::sin(yaw)*sy)<<' '
       <<9.81*(std::sin(yaw)*sx+std::cos(yaw)*sy)<<' '<<-9.81*std::cos(slope)
       <<"' integrator='Euler'/><default><geom contype='0' conaffinity='0'/></default><worldbody>"
       <<"<body name='frame' pos='4 0 .35' quat='"<<std::cos(yaw/2.)<<" 0 0 "<<std::sin(yaw/2.)<<"'>"
       <<"<joint name='root_x' type='slide' axis='1 0 0'/><joint name='root_y' type='slide' axis='0 1 0'/>"
       <<"<joint name='root_z' type='slide' axis='0 0 1'/><joint name='root_pitch' axis='0 1 0'/>"
       <<"<geom type='box' size='.5 .08 .05' mass='20'/><site name='imu_site_spatial'/>"
       <<"<body name='front_hub' pos='.6 0 0'><body name='front_wheel'><joint name='front_wheel_spin' axis='0 1 0'/>"
       <<"<geom name='geom_front_contact' type='sphere' size='.35' mass='2'/></body></body>"
       <<"<body name='rear_hub' pos='-.6 0 0'><body name='rear_wheel'><joint name='rear_wheel_spin' axis='0 1 0'/>"
       <<"<geom name='geom_rear_contact' type='sphere' size='.35' mass='2'/></body></body></body></worldbody></mujoco>";
    spatial_tests::RigModel rig;rig.model=bike::model::compile_mjcf(xml.str()).release();rig.data=engine::make_data(rig.model);
    sp::Config cfg;cfg.mode=sp::Mode::spatial3d;cfg.root=sp::RootKind::constrained4;cfg.steer.controller=sp::SteerController::off;
    rig.topology=sp::bind_topology(*rig.model,cfg);engine::forward(rig.model,rig.data);return rig;
}
double engine_energy(const spatial_tests::RigModel& rig) {
    struct Call {const mjModel* model;mjData* data;} call{rig.model,rig.data};
    engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw) noexcept {
        const auto& c=*static_cast<Call*>(raw);mj_energyPos(c.model,c.data);mj_energyVel(c.model,c.data);
    },&call,error))engine::throw_failure(error);
    return rig.data->energy[0]+rig.data->energy[1];
}
struct Result {double drift=0.,energy_residual=0.,energy_scale=1.;std::string latch;};
Result ground_case(double slope,double yaw,bool bank) {
    auto rig=slope_rig(slope,yaw,bank);
    bike::terrain::Terrain3dSource terrain(spatial_tests::plane_grid(0.,0.,0.,601,49,.5,.5));
    auto spec=spatial_tests::synthetic_spec(sp::TyreTier::fast);spec.outer_radius_m={.35,.35};
    for(auto& b:spec.brush)b.r_eff_m=.35;
    pn::TyreStateIdentity3d id;id.dims=pn::tier_dims(spec.tier);id.patch_capacity=id.dims.n_theta_fan*id.dims.n_phi;
    id.topology_fingerprint_hex=rig.topology.fingerprint_hex;id.config_fingerprint_hex=std::string(64,'b');
    id.terrain_sha256_hex=terrain.fingerprint_hex();id.calibration_sha256_hex=std::string(64,'c');
    pn::Pneumatic3dTireWriter tyre(*rig.model,rig.topology,spec,terrain,id);
    const int front=rig.topology.wheels[0].spin.dof_adr,rear=rig.topology.wheels[1].spin.dof_adr;
    runtime::StaticBrake brake(front,rear,500.,{.torque_nm=500.,.stiffness_nm_rad=10000.,.damping_nms_rad=30.});
    const auto nv=static_cast<std::size_t>(rig.model->nv);std::vector<double> force(nv);
    double lower=-.03,upper=0.;
    for(int i=0;i<50;++i) {
        const double z=(lower+upper)/2.;rig.data->qpos[rig.topology.root.scalar[2].qpos_adr]=z;
        engine::forward(rig.model,rig.data);std::ranges::fill(force,0.);
        bt::require(tyre.try_compute_into(rig.data,0.,false,force)==CoreStatus::ok,"normal-load setup probe is valid");
        double load=0.;for(std::size_t w=0;w<2;++w)for(const auto& p:tyre.wrenches(w))load+=p.normal_load_n;
        if(load>24.*9.81*std::cos(slope))lower=z;else upper=z;
    }
    double x0=0.,y0=0.,initial_energy=0.,initial_loss=0.,brake_heat=0.,stored_bite=0.;
    for(std::int64_t tick=0;tick<=20000;++tick) {
        engine::forward(rig.model,rig.data);std::ranges::fill(force,0.);
        bt::require(tyre.try_compute_into(rig.data,.0005,true,force)==CoreStatus::ok,"slope contact cannot become an invalid query");
        const std::array rates{rig.data->qvel[front],rig.data->qvel[rear]};
        const auto bite=brake.apply_spatial(rig.model,rates,1.,1.,.0005,tick,true,force);
        for(std::size_t w=0;w<2;++w)bt::require(std::abs(bite.hold_torque_nm[w])+bite.friction_budget_nm[w]<=500.+1e-12,
            "ground slip never enlarges brake torque ceiling");
        std::ranges::copy(force,rig.data->qfrc_applied);engine::step(rig.model,rig.data);
        const auto solved=brake.solved_components(rig.model,rig.data);
        const double friction_power=solved.first[static_cast<std::size_t>(front)]*
                .5*(rates[0]+rig.data->qvel[front])+solved.second[static_cast<std::size_t>(rear)]*
                .5*(rates[1]+rig.data->qvel[rear]);
        if(tick>0)brake_heat+=runtime::spatial_brake_energy(bite.stored_energy_j,bite.damping_heat_j,
            bite.release_heat_j,friction_power,.0005).loss_j;
        stored_bite=bite.stored_energy_j;
        if(tick==0) {
            x0=rig.data->qpos[rig.topology.root.scalar[0].qpos_adr];
            y0=rig.data->qpos[rig.topology.root.scalar[1].qpos_adr];
            initial_energy=engine_energy(rig)+tyre.model().stored_energy_j()+stored_bite;
            initial_loss=tyre.model().dissipated_j();
        }
    }
    const double dx=rig.data->qpos[rig.topology.root.scalar[0].qpos_adr]-x0;
    const double dy=rig.data->qpos[rig.topology.root.scalar[1].qpos_adr]-y0;
    const double end_energy=engine_energy(rig)+tyre.model().stored_energy_j()+stored_bite;
    const double losses=brake_heat+tyre.model().dissipated_j()-initial_loss;
    return {std::hypot(dx,dy),std::abs(end_energy-initial_energy+losses),
        std::max({1.,std::abs(initial_energy),std::abs(end_energy),losses}),bytes(brake.capture_spatial_brake())};
}
}
int main(){return bt::run([]{
    hinge_contracts();const double rad=std::numbers::pi/180.;
    const auto held=ground_case(5.*rad,0.,false);
    bt::require(held.drift<=.001,"five-degree slope drift <= 1 mm over 10 s at dt=.5 ms");
    bt::require(held.energy_residual<=.001*held.energy_scale+1e-6,
        "separate pad/tyre ledger closes within 0.1 percent plus 1e-6 J");
    const auto rotated=ground_case(5.*rad,std::numbers::pi/2.,false);
    bt::require(std::abs(rotated.drift-held.drift)<=1e-12,"yaw rotation preserves wheel-local brake response");
    const auto bank=ground_case(5.*rad,0.,true);bt::require(bank.drift<=.001,"bank load shares brush friction budget");
    const auto slipping=ground_case(std::atan(.8)+.08,0.,false);
    bt::require(slipping.drift>.001,"tan(slope)>mu must slide despite a strong locked brake");
});}
