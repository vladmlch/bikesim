#include "support/migration_test.hpp"
#include "spatial/support/ride_qualification.hpp"

namespace {
namespace ride=spatial_tests::ride;
ride::Report episode(bike::spatial::TyreTier tier,double dt,double coarse_dt) {
    const auto manifest=ride::example("research/spatial_flat.toml");
    auto request=spatial_tests::make_startup_request(bike::spatial::RootKind::free6,tier,true,5.);
    auto& physics=*request->model.physics;
    physics.timestep_s=dt;
    // validate.cpp requires closure_time_constant_s >= 2*timestep_s (legacy excepted). Pin one
    // closure constant per tier from the coarse step so refined runs differ only in dt, not in
    // linkage closure stiffness.
    physics.closure_time_constant_s=std::max(physics.closure_time_constant_s,2.*coarse_dt);
    const auto pin=ride::pin_profile(*request,manifest);
    spatial_tests::OwnedStartup fixture(std::move(*request));
    ride::SpatialRideAccess::perturb(*fixture.runtime,{0.,0.,0.},{.01,0.,.01});
    const auto demand=ride::demand_trace(manifest);
    auto result=ride::run(fixture,10.,demand,0.,static_cast<std::int64_t>(std::llround(.005/dt)));
    ride::print_report("timestep",result);
    bt::require(result.profile_digest==pin.tyre && result.steer_digest==pin.steer,
        "timestep trajectory uses the pinned tyre and controller source content");
    bt::require(result.valid_duration==result.requested_duration && result.valid_duration==10. && !result.terminal_reason &&
        result.invalid_ticks==0 && result.hold_ticks==0,"dt comparison uses ten complete valid physical seconds");
    bt::require(!result.points.empty() && result.points.back().energy_error<=.005,"every timestep closes the complete energy ledger within 0.5 percent");
    return result;
}
void compare(const ride::Report& coarse,const ride::Report& fine,double dt) {
    bt::require(coarse.points.size()==fine.points.size(),"refined runs share exact physical observation times");
    bt::require(coarse.events.size()==fine.events.size(),"refinement preserves contact and terminal event inventory");
    double maximum_event_time=0.;
    for(std::size_t i=0;i<coarse.events.size();++i) {
        bt::require(coarse.events[i].kind==fine.events[i].kind,"refinement preserves event order and identity");
        maximum_event_time=std::max(maximum_event_time,std::abs(coarse.events[i].time-fine.events[i].time));
    }
    double angle=0.;for(std::size_t i=0;i<coarse.points.size();++i)
        angle=std::max(angle,bike::spatial::geodesic_angle_rad(coarse.points[i].quaternion,fine.points[i].quaternion));
    const double mean_error=std::abs(coarse.mean_load-fine.mean_load)/std::max(1.,fine.mean_load);
    const double peak_error=std::abs(coarse.peak_load-fine.peak_load)/std::max(1.,fine.peak_load);
    std::cout << "{\"suite\":\"timestep_comparison\",\"coarse_dt_s\":" << dt << ",\"event_time_error_s\":" << maximum_event_time
              << ",\"mean_load_relative_error\":" << mean_error << ",\"peak_load_relative_error\":" << peak_error
              << ",\"orientation_error_rad\":" << angle << "}\n";
    bt::require(maximum_event_time<=dt+1e-9,"physical event times differ by at most one coarse timestep");
    bt::require(mean_error<=.02 && peak_error<=.05,"mean and peak loads converge within two and five percent");
    bt::require(angle<=.5*std::numbers::pi/180.,"orientation converges within half a degree");
}
} // namespace
int main() {return bt::run([] {
    for(const auto tier:{bike::spatial::TyreTier::fast,bike::spatial::TyreTier::detailed}) {
        const double dt=tier==bike::spatial::TyreTier::fast?.00125:.0005;
        const auto coarse=episode(tier,dt,dt),half=episode(tier,.5*dt,dt),quarter=episode(tier,.25*dt,dt);
        compare(coarse,quarter,dt);compare(half,quarter,.5*dt);
    }
});}
