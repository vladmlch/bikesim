// T32 source-bound behavior tests. Not run in the no-build/no-run session.
#include "support/migration_test.hpp"
#include "legacy/metrics.hpp"
#include "legacy/legs.hpp"
#include "numerics/filter.hpp"
#include "runtime/research_metrics.hpp"
#include "runtime/setup_values.hpp"
#include "terrain/profile.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

namespace {
void filters(){
    const std::array short_signal{0.,1.,-2.,3.,-0.};const auto short_result=bike::numerics::legacy_lowpass(short_signal,.001);
    for(std::size_t i=0;i<short_signal.size();++i)bt::require(std::bit_cast<std::uint64_t>(short_signal[i])==std::bit_cast<std::uint64_t>(short_result[i]),"short filter is a bitwise copy");
    std::vector<double> constant(200,7.);const auto output=bike::numerics::legacy_lowpass(constant,.001);for(double const x:output)bt::require(std::abs(x-7.)<1e-10,"filtfilt preserves a constant including its edges");
    std::vector<double> impulse(401,0.);impulse[200]=1.;const auto response=bike::numerics::legacy_lowpass(impulse,.001);
    bt::require(response[200]>0.&&response[200]<1.,"filter impulse is attenuated");for(std::size_t i=1;i<70;++i)bt::require(std::abs(response[200-i]-response[200+i])<1e-12,"forward/backward filtering has zero phase");
    const auto above=bike::numerics::legacy_lowpass(constant,.01);bt::require(above==constant,"cutoff at or above Nyquist is source passthrough");
}
void events_and_metrics(){
    namespace l=bike::legacy;const std::array<unsigned char,7> flags{1,1,0,1,0,0,1};bt::require(l::count_events(flags)==3,"events count rising edges including the first sample");
    const std::array travel{0.,180.,180.,0.},velocity{0.,2.,-3.,0.};const auto stats=l::travel_stats("fork",travel,velocity,180.,175.);
    bt::require(stats.bottom_outs==1&&stats.top_outs==2&&stats.max_compression_mps==2.&&stats.max_rebound_mps==3.,"travel stops and shaft velocity signs");
    const std::array proper{9.81,9.81,9.81};bt::require(l::accel_stats("bar",proper,.001).peak_raw_mps2==0.,"proper acceleration subtracts gravity once");
    const std::array<l::RimStrikeStats,3> impacts{{{.x_m=10.,.speed_mps=2.,.peak_load_n=600.,.peak_rim_force_n=50.,.absorbed_energy_j=1.,.start_time_s=1.,.end_time_s=1.1},{.x_m=10.005,.speed_mps=2.,.peak_load_n=610.,.peak_rim_force_n=55.,.absorbed_energy_j=1.,.start_time_s=1.1005,.end_time_s=1.2},{.x_m=11.,.speed_mps=2.,.peak_load_n=610.,.peak_rim_force_n=55.,.absorbed_energy_j=1.,.start_time_s=2.,.end_time_s=2.1}}};
    bt::require(l::count_completed_impact_episodes(impacts)==2,"adjacent rim impacts merge by time and distance");
    auto invalid=impacts;invalid[1].start_time_s=1.05;bt::invalid([&]{(void)l::count_completed_impact_episodes(invalid);});
    std::vector<double> sinusoid;sinusoid.reserve(1000);
for(int i=0;i<1000;++i)sinusoid.push_back(20.+3.*std::cos(2.*std::numbers::pi*5.*static_cast<double>(i)*.001));
    bt::require(std::abs(l::sine_amplitude(sinusoid,5.,.001)-3.)<1e-10,"bob fit recovers known amplitude after removing DC");
    l::Channels columns;for(auto const name:l::kLegacyChannels)columns.emplace_back(std::string(name),std::vector<double>{0.,0.,0.});
    auto const set=[&](std::string_view name,std::vector<double> values){for(auto& item:columns)if(item.first==name){item.second=std::move(values);break;}};
    set("x_m",{7.9,8.,8.1});set("speed_mps",{100.,2.,4.});set("front_contact",{1.,0.,0.});set("rear_contact",{1.,0.,1.});set("bar_acc_vert_mps2",{9.81,9.81,9.81});set("saddle_acc_vert_mps2",{9.81,9.81,9.81});set("cadence_rpm",std::vector<double>(3,std::numeric_limits<double>::quiet_NaN()));
    l::SummaryOptions options;options.start_x_m=0.;const l::RunOutcome result{.reason="end of track",.steps=3,.sim_time_s=.003,.wall_clock_s=.01,.position_m=20.,.crash={}};
    const auto summary=l::summarize_ride(columns,.001,bike::terrain::flat(20.),result,options);
    bt::require(summary.window_rows==2&&summary.mean_speed_kmh==10.8&&summary.airborne_events==1&&summary.airborne_time_s==.001,"run-up exclusion, mean speed and airborne timing");
    bt::require(!summary.pedals&&summary.to_json().find("legacy-v1")!=std::string::npos,"missing pedalling remains absent in versioned legacy summary");
    auto unicode=summary;unicode.track="Тест 🚲";
    bt::require(unicode.to_json().find(R"(\u0422\u0435\u0441\u0442 \ud83d\udeb2)")!=std::string::npos,"legacy JSON preserves Python's default ASCII Unicode escapes");
}
void controls(){
    namespace l=bike::legacy;l::DrivetrainSpecs const specs;specs.validate();
    bt::require(std::bit_cast<std::uint64_t>(l::opposing_torque(2.,-0.,.2))==0,"resistance is positive zero at either signed wheel rest");
    bt::require(l::opposing_torque(2.,-.1,.2)==1.&&l::rolling_torque(0.,1.,.35,100.)==0.,"resistance opposes reverse spin and disappears with support");
    bt::require(l::ripple_shape(0.,0.)==1.&&std::abs(l::ripple_shape(std::numbers::pi/2.,1.))<1e-15,"crank ripple shape/phase");
    bt::require(l::limited_rider_torque(100.,10.,specs)==std::pair<double,std::string>{30.,"power"},"rider power limit dominates torque limit at cadence");
    bt::require(l::cutoff_factor(23./3.6,specs)==1.&&l::cutoff_factor(25./3.6,specs)==0.,"assist taper endpoints");
    const auto a=bike::model::crank_torque_share(.2,.85),b=bike::model::crank_torque_share(.2+std::numbers::pi,.85);
    bt::require(std::abs(a+b-l::ripple_shape(.2,.85))<1e-14,"two leg shares reconstruct the crank pulse");
    l::TerrainContacts contacts;const auto moment=l::pitch_step({},true,.2,0.,contacts,.001);bt::require(moment.moment_nm==-80.&&moment.angular_impulse_nms==-.08,"airborne virtual rider saturates and integrates once");
    contacts.front_load_n=10.;bt::require(l::pitch_step(moment,true,.2,0.,contacts,.001).moment_nm==0.,"a loaded wheel disables airborne pitch moment");
    bt::require(!l::detect_crash(0.,1.,std::numbers::pi/3.,contacts),"pitch limit equality is not a crash");
    const auto crash=l::detect_crash(0.,1.,1.1,contacts);bt::require(crash&&crash->cause=="pitch_over","pitch threshold triggers");
    const l::RunLimits limits{.finish_x_m=10.,.max_steps=5,.max_wall_clock_s=1.};bt::require(l::termination_reason(limits,10.,5,1.,crash)=="crash","termination priority retains crash over end/caps");
    bt::require(l::termination_reason(limits,10.,5,1.,{})=="end of track","track end precedes caps");
    l::EnergyLedger ledger(100.);bt::require(ledger.residual(105.,10.,-2.,3.)==0.,"mechanical ledger signs");bt::invalid([&]{(void)ledger.residual(0.,0.,0.,-1.);});
}
void wheelies(){
    bt::require(runtime::classify_contact(0.,0.,.1,.1,.1)=="flight","two-wheel flight is distinct from wheelie");
    runtime::WheelieTracker tracker(.02,2.,.02,.05);runtime::WheelieTruth t;t.rear_load_n=7.;t.front_clearance_m=.04;t.relative_pitch_rad=.1;
    tracker.update(t,.01,{});bt::require(tracker.state()=="wheelie_candidate","wheelie confirmation waits for persistence");
    t.time_s=.01;t.front_load_n=3.;t.front_clearance_m=.015;t.relative_pitch_rad=.03;tracker.update(t,.01,{});bt::require(tracker.state()=="wheelie","exit hysteresis applies only to the front");
    t.time_s=.02;t.rear_load_n=0.;t.rear_clearance_m=.1;tracker.update(t,.01,{});bt::require(tracker.state()!="wheelie","loss of rear support clears a wheelie");
    bt::invalid([&]{tracker.update(t,.01,{});});tracker.reset();bt::require(tracker.state()=="uninitialized","tracker reset clears temporal state");
    bt::require(runtime::quasistatic_front_load(100.,1.,.5,1.,0.,10.)<0.,"quasistatic diagnostic keeps an infeasible signed front load");
}
void oracle(){bt::manifest_verify_files(bt::manifest_entry("metrics/filter"));const auto input=bt::read_f64(bt::fixture("metrics/filter_input.f64"));bt::golden_f64("metrics/filter_expected.f64",bike::numerics::legacy_ride_filter(input,1000.));
    const auto short_input=bt::read_f64(bt::fixture("metrics/filter_input_short.f64"));bt::golden_f64("metrics/filter_short.f64",bike::numerics::legacy_ride_filter(short_input,1000.));}
}
int main(){return bt::run([]{filters();events_and_metrics();controls();wheelies();oracle();});}
