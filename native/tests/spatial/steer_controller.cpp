#include "support/migration_test.hpp"
#include "fixture_paths.hpp"
#include "rider/steer_balance.hpp"
#include "spatial/bootstrap_state.hpp"
#include "io/json.hpp"

#include <cmath>
#include <filesystem>
#include <limits>

int main(){return bt::run([] {
    using namespace bike;
    const auto document=io::parse_json(bt::text(
        std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"native/tests/spatial/data/steer_profiles.json"));
    const auto profile=rider::read_steer_profile(document);
    for(const auto* key:{"cap_nm","gains","sha256_hex"}) {
        auto duplicate=document;
        std::get<io::Value::Object>(duplicate.data).emplace_back(key,io::Value{});
        bt::invalid([&]{(void)rider::read_steer_profile(duplicate);});
    }
    bt::require(profile.provenance_status=="provisional","authored gains make no calibration claim");
    rider::SteerBalance control(profile,.005);
    bt::require(control.delay_ticks()==20 && control.resolved_delay_s()==.1,"100 ms is twenty 200 Hz ticks");
    rider::SteerInput input;input.speed_signed_mps=3.;
    bt::require(control.tick(input)==0., "zero state produces zero torque");
    control.reset();input.roll_rad=.01;
    std::int64_t first=-1;
    for(std::int64_t tick=0;tick<25;++tick) {
        input.control_tick=tick;
        const double applied=control.tick(input);
        if(applied!=0. && first<0)first=tick;
        bt::require(applied>=0. && std::abs(applied)<=40.+1e-12,"restoring steering sign and strength cap");
    }
    bt::require(first==20,"first command is applied exactly twenty ticks later");
    const auto state=spatial::decode_steer(spatial::encode_steer(control.state()));
    const auto before=io::canonical_json(spatial::encode_steer(control.state()));
    (void)control.tick(input);
    bt::require(before==io::canonical_json(spatial::encode_steer(control.state())),"duplicate control tick cannot advance queue or work");
    rider::SteerBalance restored(profile,.005);
    bt::require(restored.restore(state)==CoreStatus::ok,"active delay queue restores");
    for(std::int64_t tick=25;tick<100;++tick) {
        input.control_tick=tick;input.roll_rad=tick%2==0?10.:-10.;input.steer_rate_rad_s=.2;
        bt::require(restored.tick(input)==control.tick(input),"restored controller continuation is exact");
        bt::require(io::canonical_json(spatial::encode_steer(restored.state()))==
            io::canonical_json(spatial::encode_steer(control.state())),"every state lane continues bitwise through canonical roundtrip");
    }
    input.control_tick=100;input.enabled=false;
    bt::require(control.tick(input)==0.,"disabled controller immediately applies zero while queue receives zeros");
    bt::require(control.state().last_command_nm==0.,"disabled tick advances with a zero command");
    auto bad=state;bad.profile_sha256=std::string(64,'f');
    const auto unchanged=io::canonical_json(spatial::encode_steer(restored.state()));
    bt::require(restored.restore(bad)==CoreStatus::invalid_input,"foreign profile rejected");
    bad=state;bad.delay_fifo_nm[0]=std::numeric_limits<double>::quiet_NaN();
    bt::require(restored.restore(bad)==CoreStatus::invalid_input,"nonfinite queued demand rejected");
    bt::require(unchanged==io::canonical_json(spatial::encode_steer(restored.state())),"failed restores are atomic");
    control.reset();
    bt::require(control.state().controller==spatial::SteerController::roll_heading &&
        control.state().profile_sha256==profile.sha256_hex && control.state().next_tick==0 &&
        control.state().work_j==0.,"reset retains controller identity and clears causal history");
    rider::SteerBalance rounded(profile,.006);
    bt::require(rounded.delay_ticks()==17 && std::abs(rounded.resolved_delay_s()-.102)<=1e-12,
        "nonintegral delay is explicitly resolved to an integer tick count");
});}
