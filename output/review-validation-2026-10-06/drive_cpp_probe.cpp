#include "native/src/writers/drivetrain.hpp"
#include <cstdlib>
#include <iostream>
#include <new>
#include <string>
#include <type_traits>
static long allocations = 0, fail_at = -1;
void *operator new(std::size_t n) { if (fail_at == allocations++) throw std::bad_alloc(); if(auto *p=std::malloc(n)) return p; throw std::bad_alloc(); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
drivetrain::DriveConfig config() {
    drivetrain::DriveConfig c;
    c.drive_mode="crank_effort";c.transmission_model="ideal_mid_drive";c.human_torque_nm=20.;c.torque_ripple=.35;c.chain_k_n_m=1000.;
    c.policies.gearing={34,24,.0127};
    c.policies.pedaling={false,110.,90.,.3,0.,0.,0.,100.};
    c.policies.shifting={false,{24},60.,100.,.3,.1,.2,0.,1.,"magnitude"};
    c.policies.assist={2.,80.,500.,.05,400.,4.,.1,7.,1.,std::nullopt,std::nullopt,"turbo"};
    c.policies.battery={true,1000.,.01,.001,1.};
    c.policies.hub_stiffness_nm_rad=1000.;return c;
}
int main(int argc,char **argv) {
    std::string arg=argc>1?argv[1]:"normal";
    if(arg=="curve") {
        auto c=config().policies.assist;c.torque_curve.emplace();drivetrain::AssistController a(c);a.ceiling(60.,2.);return 0;
    }
    std::cout<<"Transmission movable "<<std::is_move_constructible_v<drivetrain::Transmission><<" copyable "<<std::is_copy_constructible_v<drivetrain::Transmission><<'\n';
    if(arg=="invalid") {
        drivetrain::PedalingPolicy p({});auto s=p.update(0.,4.,40.,0.,.01);std::cout<<"invalid pedal accepted rate "<<s.target_rate_rad_s<<" phase "<<*s.target_phase_rad<<'\n';
        drivetrain::CadenceShifter sh({},{});std::cout<<"invalid gear accepted ratio "<<sh.gear_ratio()<<'\n';return 0;
    }
    auto *m=mj_loadXML("/private/tmp/drive-ideal.xml",nullptr,nullptr,0);auto *d=mj_makeData(m);
    auto w=drivetrain::DrivetrainWriter(m,d,config());w.reset();
    d->qvel[m->jnt_dofadr[mj_name2id(m,mjOBJ_JOINT,"crank_spin")]]=4.;mj_forward(m,d);
    if(arg=="move") { drivetrain::Transmission a(m,{34,24,.0127}); auto b=std::move(a);a.set_ratio(d,34./28.);return 0; }
    allocations=0;w.components({},.1,2.,false,true,true,0.,std::nullopt,true,std::nullopt);long count=allocations;
    std::cout<<"compute allocations "<<count<<'\n';mj_forward(m,d);w.settle();
    auto initial=w.state();d->time=.1;
    for(long n=0;n<count;n++) {
        w.restore(initial);allocations=0;fail_at=n;
        try { w.components({},.1,2.,false,true,true,0.,std::nullopt,true,std::nullopt); }
        catch(const std::bad_alloc &) { fail_at=-1;auto s=w.state();if(s.pedaling.effort!=initial.pedaling.effort) {std::cout<<"failed compute allocation "<<n<<" changes effort "<<initial.pedaling.effort<<" -> "<<s.pedaling.effort<<" last_time "<<s.last_time_s.value_or(-1.)<<" pending "<<bool(s.pending_actuation)<<'\n';break;} }
        fail_at=-1;
    }
    w.restore(initial);w.components({},.1,2.,false,true,true,0.,std::nullopt,true,std::nullopt);mj_forward(m,d);
    auto pending=w.state();pending.last.clear();std::cout<<"pending torque "<<pending.pending_actuation->requested<<" actual "<<d->actuator_force[mj_name2id(m,mjOBJ_ACTUATOR,"mid_drive")]<<" battery "<<pending.battery.energy_j<<'\n';
    for(long n=0;n<15;n++) {
        w.restore(pending);allocations=0;fail_at=n;
        try {w.settle();}catch(const std::bad_alloc &) { fail_at=-1;auto s=w.state();std::cout<<"settle failed allocation "<<n<<" battery "<<s.battery.energy_j<<'\n';if(s.battery.energy_j!=pending.battery.energy_j) {std::cout<<"failed settle allocation "<<n<<" battery "<<pending.battery.energy_j<<" -> "<<s.battery.energy_j<<" pending "<<bool(s.pending_actuation)<<'\n';w.settle();std::cout<<"retry battery "<<w.state().battery.energy_j<<'\n';break;} }
        fail_at=-1;
    }
    mj_deleteData(d);mj_deleteModel(m);
}
