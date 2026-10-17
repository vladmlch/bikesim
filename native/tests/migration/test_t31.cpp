// Authored against T31 source; execution is deferred by user instruction.
#include "support/migration_test.hpp"
#include "legacy/ride.hpp"
#include "legacy/metrics.hpp"
#include "terrain/profile.hpp"
#include "configuration/model_config.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ranges>
#include <span>
#include <string>

namespace {
void same(std::span<const double> a,std::span<const double> b,std::string_view why){
    bt::require(a.size()==b.size(),why);for(std::size_t i=0;i<a.size();++i)bt::require(std::bit_cast<std::uint64_t>(a[i])==std::bit_cast<std::uint64_t>(b[i]),std::string(why)+" at "+std::to_string(i));
}
void tyre_contracts(){
    namespace l=bike::legacy;
    l::TyreConfig config;config.validate();bt::require(!config.pneumatic(),"sphere remains the legacy default");
    const auto authored=l::make_tyre_specs("test",372.,320.,57.,1.5,.011);
    bt::require(authored.area_factor==.46&&authored.tyre_height_mm()==52.&&authored.rim_strike_deflection_mm()==46.&&authored.pressure_pa()==150000.,"tyre defaults and physical unit properties");
    config.model="pneumatic";config=config.with_pressures(1.2,2.1);bt::require(config.front.pressure_bar==1.2&&config.rear.pressure_bar==2.1,"front/rear tyre pressures independent");
    bt::invalid([&]{(void)l::tier_spec("unknown");});bt::invalid([&]{(void)config.with_pressures(0.,1.5);});
    biketyre::RoadProfile road(-1.,.01,std::vector<double>(201,0.));
    const auto crossing=l::first_crossing_reference(0.,0.,.30,.372,road);
    bt::require(crossing&&std::abs(*crossing-.30)<1e-15,"reference ray finds the first visible surface");
    bt::require(!l::first_crossing_reference(0.,0.,1.,.372,road),"reference ray reports no contact beyond radius");
    for(std::string_view name:{"fast","detailed"})for(double pressure:{1.1,2.3}){
        const auto tier=l::tier_spec(name);const auto spec=l::with_pressure(l::front_tyre(),pressure);
        biketyre::PneumaticTyreModel tyre(spec,tier.n_rays,tier.half_angle_deg,tier.discretised_brush);
        const std::array hub{0.,0.,.365},velocity{0.,0.,0.};
        nativecfg::SurfaceMap surface{{"hardpack",.7,.5,18.,4.5},{}};
        const auto& loaded=tyre.evaluate(hub,velocity,0.,road,surface,tier.timestep_s);
        bt::require(loaded.support_n>0.&&!loaded.airborne&&loaded.pressure_bar==pressure,"each legacy tier computes pressure-dependent support");
        tyre.reset();const auto& reset=tyre.last_outputs();bt::require(reset.support_n==0.&&reset.airborne,"tyre reset clears contact/material history");
    }
}
void ride_contracts(){
    namespace l=bike::legacy;
    auto sim=l::make_ride(bt::fixture("requests/legacy_flat.toml"));const auto before=sim->snapshot();
    bt::require(before.time_s==0.&&before.steps==0,"legacy setup ends at the reset time");
    bt::invalid([&]{sim->step(std::numeric_limits<double>::quiet_NaN(),0.);});
    same(before.integration_state,sim->snapshot().integration_state,"invalid demand does not mutate the ride");
    l::RideRecorder recorder(sim->timestep_s(),2);for(int i=0;i<5;++i){sim->step();recorder.record(*sim);}
    bt::require(sim->steps()==5&&recorder.rows()==3,"legacy advances once and decimates first/every nth call");
    const auto columns=recorder.columns();bt::require(columns.size()==l::kLegacyChannels.size(),"legacy CSV schema retains all named channels");
    for(std::size_t i=0;i<columns.size();++i)bt::require(columns[i].first==l::kLegacyChannels[i]&&columns[i].second.size()==3,"column ordering and width retained");
    for(double value:recorder.column("crank_phase_rad"))bt::require(std::isnan(value),"motor mode records missing cranks as NaN");
    for(double value:recorder.column("front_tyre_fz_n"))bt::require(value==0.,"sphere mode keeps the legacy zero tyre channels");
    const auto sampled=sim->snapshot();(void)sim->telemetry();(void)recorder.columns();same(sampled.integration_state,sim->snapshot().integration_state,"legacy metrics leave authoritative stepping unchanged");
    sim->reset();const auto reset=sim->snapshot();same(before.integration_state,reset.integration_state,"repeat native equilibrium restores deterministic legacy state");
    bt::require(reset.generation>before.generation,"reset advances generation");
    bt::manifest_verify_files(bt::manifest_entry("bootstrap/legacy_flat"));
    bt::golden_f64("bootstrap/legacy_flat/integration.f64",before.integration_state);
    sim->step();bt::golden_f64("bootstrap/legacy_flat/step1.f64",sim->snapshot().integration_state);
}
void mode_contracts(){
    for(std::string_view mode:{"pedal","pedelec"}){
        bike::legacy::RideOptions options;options.setup=bike::initialization::read_setup_request(bt::fixture("requests/legacy_flat.toml"));
        options.setup.model.rider->variant="seated";options.drive_mode=mode;options.legs="articulated";options.drivetrain.crank_phase_deg=25.;
        auto ride=bike::legacy::make_ride(std::move(options));ride->step();bike::legacy::RideRecorder recorder(ride->timestep_s());recorder.record(*ride);
        bt::require(std::isfinite(recorder.column("crank_phase_rad").front()),"both pedalled modes expose a real crank phase");
    }
    bike::legacy::RideOptions options;options.setup=bike::initialization::read_setup_request(bt::fixture("requests/legacy_flat.toml"));options.setup.runtime.record_retention=runtime::RetentionMode::full;
    bt::invalid([&]{(void)bike::legacy::make_ride(options);});
}
void csv_bytes(){
    bike::legacy::RideRecorder recorder(.001,2);bike::legacy::TelemetryRow row{};
    row[0]=1.234567890123;row[1]=-0.;row[2]=std::numeric_limits<double>::quiet_NaN();recorder.record(row);
    row[0]=99.;recorder.record(row);
    const auto csv=recorder.to_csv();const auto newline=csv.find('\n');
    bt::require(newline!=std::string::npos&&csv.starts_with("time_s,x_m,speed_mps,fork_travel_mm,"),"legacy CSV begins with the source channel order");
    std::string expected="1.23456789,-0,nan";for(std::size_t i=3;i<bike::legacy::kLegacyChannels.size();++i)expected+=",0";expected+='\n';
    bt::require(csv.substr(newline+1)==expected,"legacy CSV uses ten significant digits, NaN, signed zero and source decimation");
}
}
int main(){return bt::run([]{csv_bytes();tyre_contracts();mode_contracts();ride_contracts();});}
