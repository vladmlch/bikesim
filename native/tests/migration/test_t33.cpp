// T33 controller-only regressions; no window/event-loop dependency.
#include "support/migration_test.hpp"
#include "legacy/playground.hpp"
#include "runtime/setup_values.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

namespace {
void same(std::span<const double> a,std::span<const double> b){bt::require(a.size()==b.size(),"stand state width");for(std::size_t i=0;i<a.size();++i)bt::require(std::bit_cast<std::uint64_t>(a[i])==std::bit_cast<std::uint64_t>(b[i]),"stand state bit pattern");}
double metric(const bike::io::Value& value,std::string_view name){return bike::io::as_double(runtime::setupvalue::field(value,name),name);}
void controllers(){
    bike::geometry::BikeSpecs specs;bike::model::ForkAirSpring air;bike::model::BikeSuspensionSystem suspension({},180.,65.,true);
    bike::legacy::SuspensionController controller(specs,air,suspension);
    const auto rest=controller.compute_shock_force(20.,0.);bt::require(rest.spring_n==2292.&&rest.damper_n==0.&&rest.total_n==2292.,"stand coil law uses shaft metres and authored stiffness");
    const auto compression=controller.compute_fork_force(50.,1.),rebound=controller.compute_fork_force(50.,-1.);bt::require(compression.damper_n>0.&&rebound.damper_n<0.,"compression and rebound damper directions");
}
void lifecycle(){
    auto playground=bike::legacy::make_playground();const auto initial=playground->snapshot();bt::require(initial.time_s==0.&&initial.ctrl.size()==2,"headless playground starts uncompressed with stand actuators");
    const auto stable=playground->telemetry();same(initial.qpos,playground->snapshot().qpos);bt::require(metric(stable,"p3_residual_mm")<1e-5,"initial linkage closes");
    playground->set_travel(90.,90.);playground->step_at(100.);const auto moved=playground->snapshot();bt::require(moved.time_s>0.&&moved.target_rear_travel_mm==90.&&moved.target_fork_travel_mm==90.,"stand targets advance controller simulation");
    const double mass=metric(moved.telemetry,"total_mass_kg");bt::require(playground->toggle_rider(),"rider enables");const auto with_rider=playground->snapshot();same(moved.qpos,with_rider.qpos);same(moved.qvel,with_rider.qvel);bt::require(metric(with_rider.telemetry,"total_mass_kg")>mass,"rider toggle changes compiled mass in place");
    playground->set_fork_clicks(999,-999,999);auto telemetry=playground->telemetry();bt::require(metric(telemetry,"fork_hsc")>=0.&&metric(telemetry,"fork_lsc")==0.,"click settings route through native clamps");
    playground->apply_preset(bike::model::DamperPreset::plush);bt::require(bike::io::as_string(runtime::setupvalue::field(playground->telemetry(),"damper_preset"),"preset")=="PLUSH","factory preset reaches telemetry");
    playground->set_auto_sweep(true,100.,.4);playground->step_at(100.);bt::require(std::abs(playground->snapshot().target_rear_travel_mm)<1e-12,"sweep starts at zero compression");
    playground->step_at(101.25);bt::require(std::abs(playground->snapshot().target_rear_travel_mm-180.)<1e-10,"sweep reaches full compression after half a cycle");
    playground->enqueue_key('r');playground->step_at(102.);bt::require(playground->snapshot().target_rear_travel_mm==0.&&!playground->snapshot().auto_sweep,"queued reset executes before the next step");
    playground->reset();bt::require(playground->snapshot().time_s==0.&&playground->snapshot().generation>initial.generation,"reset clears clock and increments generation");
    const auto before=playground->snapshot();bt::invalid([&]{playground->set_model("invalid");});same(before.qpos,playground->snapshot().qpos);
    bt::invalid([&]{playground->set_travel(std::numeric_limits<double>::quiet_NaN(),0.);});
    for(const char* mode:{"standard","stand","playground"}){playground->set_model(mode);bt::require(playground->snapshot().model_mode==mode&&playground->snapshot().time_s==0.,"model switching installs an independently initialized owner");playground->step_at(1.);}
}
}
int main(){return bt::run([]{controllers();lifecycle();});}
