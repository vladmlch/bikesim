// W36: real runtime qualifications, authored but deliberately not executed.
#include "support/migration_test.hpp"
#include "spatial/support/ride_qualification.hpp"

#include <bit>

namespace {
namespace ride=spatial_tests::ride;
namespace sp=bike::spatial;
constexpr double degree=std::numbers::pi/180.;
constexpr std::array tiers{sp::TyreTier::fast,sp::TyreTier::detailed};
void profile_and_physical(const ride::Report& report,const ride::ProfilePin& pin) {
    bt::require(report.profile_digest==pin.tyre && report.steer_digest==pin.steer,
        "qualification cannot drift to different tyre or steering source content");
    bt::require(report.invalid_ticks==0 && report.hold_ticks==0,
        "target ride remains physical throughout every scored interval");
    for(const auto& p:report.points) {
        bt::require(p.valid && !p.hold && p.hold_work==0.,"target ride has no bootstrap hold work or invalid contact");
        bt::require(std::abs(p.steer)<=40.+1e-12 && std::abs(p.motor_torque)<=85.+1e-9,
            "actual steering and motor outputs obey their declared strength ceilings");
        bt::require(p.brake_budget_error<=1e-6,"solved friction plus bite obey one brake torque budget");
        bt::require(p.ellipse<=1.+1e-10,"each actual material patch shares its Stribeck friction ellipse");
    }
}
void full_duration(const ride::Report& report) {
    bt::require(report.valid_duration==report.requested_duration && !report.terminal_reason,
        "qualification requires the entire requested valid episode, not a short surviving prefix");
}
void steering_recovery(const ride::Report& report,double initial_speed) {
    std::int64_t previous_tick=-1,previous_saturated=0,consecutive=0,longest=0;
    const ride::Point* at_three=nullptr;
    for(const auto& p:report.points) {
        if(!at_three || std::abs(p.time-3.)<std::abs(at_three->time-3.))at_three=&p;
        if(p.time>3. || p.steer_tick==previous_tick)continue;
        consecutive=p.saturated_ticks>previous_saturated?consecutive+1:0;
        longest=std::max(longest,consecutive);previous_tick=p.steer_tick;previous_saturated=p.saturated_ticks;
    }
    bt::require(longest<=40,"W32 steering stays below forty consecutive saturated control ticks in recovery");
    if(initial_speed==3.) {
        bt::require(at_three && std::abs(at_three->time-3.)<.006,"W32 recovery includes the three-second physical boundary");
        bt::require(std::abs(at_three->roll)<.5*degree && std::abs(at_three->position[1])<.3,
            "W32 three-metre-per-second recovery reaches half a degree roll and 0.3 m lateral error by three seconds");
    }
}
void flat_matrix() {
    const auto manifest=ride::example("research/spatial_flat.toml");const auto trace=ride::demand_trace(manifest);
    for(auto tier:tiers)for(double speed:{3.,5.,8.})for(double roll:{-3.,3.})for(double heading:{-2.,2.}) {
        auto request=spatial_tests::make_startup_request(sp::RootKind::free6,tier,true,speed);
        const auto pin=ride::pin_profile(*request,manifest);spatial_tests::OwnedStartup fixture(std::move(*request));
        ride::SpatialRideAccess::perturb(*fixture.runtime,{0.,0.,0.},{roll*degree,0.,heading*degree});
        const auto stride=static_cast<std::int64_t>(std::llround(.005/fixture.bootstrap->config.timestep_s));
        const auto report=ride::run(fixture,20.,trace,0.,stride);
        ride::print_report("F",report);profile_and_physical(report,pin);full_duration(report);steering_recovery(report,speed);
        for(const auto& p:report.points)bt::require(std::abs(p.position[1])<=.5,"F lateral position remains inside half a metre");
    }
    auto request=spatial_tests::make_startup_request(sp::RootKind::free6,sp::TyreTier::fast,true,3.);
    request->model.physics->spatial->steer.controller=sp::SteerController::off;request->model.spatial=request->model.physics->spatial;
    spatial_tests::OwnedStartup fixture(std::move(*request));
    ride::SpatialRideAccess::perturb(*fixture.runtime,{0.,0.,0.},{3.*degree,0.,0.});
    const auto diagnostic=ride::run(fixture,20.,trace,0.,4);
    double maximum_roll=0.;for(const auto& p:diagnostic.points)maximum_roll=std::max(maximum_roll,std::abs(p.roll));
    std::cout << "{\"suite\":\"F_controller_off\",\"kind\":\"diagnostic\",\"observed_duration_s\":"
              << (diagnostic.points.empty()?0.:diagnostic.points.back().time) << ",\"maximum_abs_roll_rad\":" << maximum_roll
              << ",\"terminal_reason\":\"" << diagnostic.terminal_reason.value_or("none") << "\"}\n";
    // The diagnostic has no asserted fall time; physical acceptance belongs
    // to the controller-on matrix above.
}
std::shared_ptr<const bike::terrain::Terrain3dSource> bank_terrain(double angle,double mu=.8) {
    return ride::surface_grid([=](double x,double y) {
        const double bank=bike::terrain::quintic_smoothstep((x-12.)/8.)-
                          bike::terrain::quintic_smoothstep((x-60.)/8.);
        return std::tan(angle*degree)*bank*y;
    },[](double,double){return 1u;},ride::dry_and_wet(mu,.3),200.,6.,.25,.1,"bank ramp and return to flat");
}
void bank_matrix() {
    const auto manifest=ride::example("research/spatial_cross_slope.toml");const auto trace=ride::demand_trace(manifest);
    for(auto tier:tiers)for(double magnitude:{5.,10.}) {
        std::array<double,2> lateral{},steer{};
        for(std::size_t sign=0;sign<2;++sign) {
            const double angle=(sign==0?-1.:1.)*magnitude;
            auto request=spatial_tests::make_startup_request(sp::RootKind::free6,tier,true,5.,bank_terrain(angle));
            const auto pin=ride::pin_profile(*request,manifest);spatial_tests::OwnedStartup fixture(std::move(*request));
            const auto report=ride::run(fixture,20.,trace,0.,static_cast<std::int64_t>(std::llround(.005/fixture.bootstrap->config.timestep_s)));
            ride::print_report("B",report);profile_and_physical(report,pin);full_duration(report);
            std::size_t plateau_samples=0;
            for(const auto& p:report.points) {
                bt::require(std::abs(p.position[1])<=.75,"B remains inside the declared lateral corridor");
                if(p.position[0]>22. && p.position[0]<55.){lateral[sign]+=p.lateral_load;steer[sign]+=p.steer;++plateau_samples;}
            }
            bt::require(plateau_samples>0 && report.points.back().position[0]>=68.,"B samples the bank plateau and completes the return to flat");
            lateral[sign]/=static_cast<double>(plateau_samples);steer[sign]/=static_cast<double>(plateau_samples);
            bt::require(angle*lateral[sign]>0.,"cross-slope tangent load points uphill against the downhill normal component");
        }
        bt::require(lateral[0]*lateral[1]<0. && steer[0]*steer[1]<0.,"mirrored bank signs reverse lateral load and steering response");
        std::cout << "{\"suite\":\"B_signs\",\"bank_magnitude_deg\":" << magnitude << ",\"lateral_load_n\":["
                  << lateral[0] << ',' << lateral[1] << "],\"steer_nm\":[" << steer[0] << ',' << steer[1] << "]}\n";
    }
    auto request=spatial_tests::make_startup_request(sp::RootKind::free6,sp::TyreTier::fast,true,5.,bank_terrain(10.,.02));
    const auto pin=ride::pin_profile(*request,manifest);spatial_tests::OwnedStartup fixture(std::move(*request));
    const auto outside=ride::run(fixture,20.,trace,0.,4);ride::print_report("B_outside_mu",outside);profile_and_physical(outside,pin);
    bt::require(outside.terminal_reason.has_value() && !outside.points.empty(),"outside-friction bank ends with a physical terminal event");
    const auto reason=outside.points.back().reason;
    bt::require(reason==sp::SpatialReason::capsize || reason==sp::SpatialReason::off_corridor || reason==sp::SpatialReason::balance_lost,
        "an impossible bank is a physical loss of balance or corridor, never invalid terrain/numerics");
}
void rough_uphill() {
    const auto manifest=ride::example("research/spatial_rough_uphill.toml");const auto trace=ride::demand_trace(manifest);
    auto spec=bike::terrain::detail::load_track3d_spec(ride::example("research/rough_uphill_savage_3d_genspec.toml"));
    bt::require(spec.seed_decimal=="0","rough/uphill uses the authored fixed generator seed");
    std::shared_ptr<const bike::terrain::Terrain3dSource> terrain;
    try {terrain=std::make_shared<const bike::terrain::Terrain3dSource>(bike::terrain::generate_grid3d(spec));}
    catch(const std::invalid_argument&) {
        std::cout << "{\"suite\":\"R\",\"status\":\"blocked\",\"reason\":\"authored savage terrain rejected; resolve W09/V02 recipe before qualification\"}\n";
        throw;
    }
    for(auto tier:tiers) {
        auto request=spatial_tests::make_startup_request(sp::RootKind::free6,tier,true,3.,terrain);
        request->track=bike::terrain::track_from_dict(spec.base_track_spec);
        const auto pin=ride::pin_profile(*request,manifest);spatial_tests::OwnedStartup fixture(std::move(*request));
        const auto report=ride::run(fixture,120.,trace,100.,static_cast<std::int64_t>(std::llround(.005/fixture.bootstrap->config.timestep_s)));
        ride::print_report("R",report);profile_and_physical(report,pin);
        bt::require(report.completed_distance>=report.requested_distance,"rough-uphill acceptance requires the full requested 100 m ride");
        bt::require(!report.points.empty() && report.points.back().energy_error<=.005,"rough-uphill complete ledger closes within 0.5 percent");
        double travel=0.,impulse=0.;double last_time=0.;
        for(const auto& p:report.points) {
            bt::require(p.max_rim==0.,"qualification profile has no rim penetration on the requested rough-uphill course");
            travel=std::max(travel,p.suspension_travel);impulse+=p.load*(p.time-last_time);last_time=p.time;
        }
        std::cout << "{\"suite\":\"R_impulses\",\"load_impulse_ns\":" << impulse << ",\"peak_load_n\":" << report.peak_load
                  << ",\"maximum_suspension_travel_m\":" << travel << ",\"terrain_digest\":\"" << terrain->fingerprint_hex() << "\"}\n";
    }
}
ride::BeforeStep circular_reference(const ride::Vec3& center,double radius) {
    return [=](runtime::NativeRideRuntime& rt,double) {
        auto& physical=ride::SpatialRideAccess::physical(rt);auto& stepper=ride::SpatialRideAccess::stepper(rt);
        const auto p=bike::spatial::body_pose(*stepper.model(),*stepper.data(),physical.topology()->root);
        const auto radial=ride::sub(p.position_world_m,center);
        const double speed=physical.longitudinal_speed_mps();
        ride::SpatialRideAccess::reference_tick(rt,-std::atan(speed*speed/(ride::gravity*radius)),
            std::atan2(radial[1],radial[0])+std::numbers::pi/2.,radius-std::hypot(radial[0],radial[1]),std::atan(1.2/radius));
    };
}
std::shared_ptr<const bike::terrain::Terrain3dSource> split_terrain(bool stripes,double mu=.8) {
    return ride::surface_grid([](double x,double){return std::tan(5.*degree)*x;},
        [=](double,double y){return stripes?(static_cast<std::int64_t>(std::floor((y+6.)/.01))%2==0?1u:2u):(y<0.?1u:2u);},
        ride::dry_and_wet(mu,std::min(.3,mu)),80.,6.,.1,.01,stripes?"5 degree slope with resolved split material strips":"5 degree split-grip slope");
}
void split_grip() {
    const auto manifest=ride::example("research/spatial_split_grip.toml");const auto trace=ride::demand_trace(manifest);
    for(auto tier:tiers) {
        auto request=spatial_tests::make_startup_request(sp::RootKind::free6,tier,true,3.,split_terrain(false));
        // An intentional stopped manoeuvre uses a declared 20 s balance dwell;
        // all geometric capsizing, corridor and invalidity guards remain live.
        request->model.physics->articulated.balance_dwell_s=20.;
        request->model.physics->spatial->corridor_half_width_m=5.;
        const auto pin=ride::pin_profile(*request,manifest);spatial_tests::OwnedStartup fixture(std::move(*request));
        const auto initial=ride::observe(*fixture.runtime);const ride::Vec3 center{initial.position[0],initial.position[1]+20.,0.};
        ride::SpatialRideAccess::perturb(*fixture.runtime,{0.,0.,0.},{-std::atan(9./(ride::gravity*20.)),0.,0.},std::atan(1.2/20.),3./20.);
        const auto report=ride::run(fixture,12.,trace,0.,static_cast<std::int64_t>(std::llround(.005/fixture.bootstrap->config.timestep_s)),circular_reference(center,20.));
        ride::print_report("G",report);profile_and_physical(report,pin);full_duration(report);
        std::vector<ride::Point> held;
        for(const auto& p:report.points)if(p.time>=5.5 && p.time<=7.)held.push_back(p);
        bt::require(!held.empty(),"split-grip ride includes the prescribed stopped hold window");
        for(const auto& p:held)bt::require(std::abs(p.speed)<=.03,"admissible 5 degree slope hold has negligible creep");
        bt::require(ride::norm(ride::sub(held.back().position,held.front().position))<=.01,"brake hold moves less than one centimetre on the admissible slope");
        bt::require(report.points.back().speed>.5,"gradual brake release permits a physical restart");
    }
    auto request=spatial_tests::make_startup_request(sp::RootKind::free6,sp::TyreTier::fast,true,0.,split_terrain(false,.02));
    request->model.physics->initial_front_brake=request->model.physics->initial_rear_brake=1.;
    request->model.physics->articulated.balance_dwell_s=20.;
    const auto pin=ride::pin_profile(*request,manifest);spatial_tests::OwnedStartup fixture(std::move(*request));
    const std::array<ride::Demand,1> brake{{{0.,0.,1.}}};
    const auto start=ride::observe(*fixture.runtime);const auto impossible=ride::run(fixture,3.,brake,0.,4);
    ride::print_report("G_outside_mu",impossible);profile_and_physical(impossible,pin);
    bt::require(!impossible.points.empty() && (std::abs(impossible.points.back().position[0]-start.position[0])>.02 ||
        impossible.points.back().reason==sp::SpatialReason::capsize),"locked wheels cannot provide an impossible slope-hold friction force");
}
void continuation() {
    const auto manifest=ride::example("research/spatial_split_grip.toml");const auto trace=ride::demand_trace(manifest);
    for(auto tier:tiers) {
        auto request=spatial_tests::make_startup_request(sp::RootKind::free6,tier,true,3.,split_terrain(true));
        request->model.physics->articulated.balance_dwell_s=20.;request->model.physics->spatial->corridor_half_width_m=5.;
        const auto pin=ride::pin_profile(*request,manifest);spatial_tests::OwnedStartup fixture(std::move(*request));
        const auto initial=ride::observe(*fixture.runtime);const ride::Vec3 center{initial.position[0],initial.position[1]+20.,0.};
        ride::SpatialRideAccess::perturb(*fixture.runtime,{0.,0.,0.},{-std::atan(9./(ride::gravity*20.)),0.,0.},std::atan(1.2/20.),3./20.);
        const auto reference=circular_reference(center,20.);const double dt=fixture.bootstrap->config.timestep_s;
        // Exactly one physical step after a 200 Hz boundary: accounting has
        // a nonempty unclosed period when the complete checkpoint is taken.
        const auto prefix=ride::run(fixture,2.1+dt,trace,0.,1,reference,false);
        profile_and_physical(prefix,pin);full_duration(prefix);
        const auto point=ride::observe(*fixture.runtime);
        const auto checkpoint=ride::SpatialRideAccess::capture(*fixture.runtime);
        bt::require(checkpoint->buffer && !checkpoint->buffer->empty(),"C captures the in-flight accounting period, not only closed telemetry");
        bt::require(point.patches>=3 && std::abs(point.yaw_rate)>1e-4 && std::abs(point.steer)>1e-9 &&
            checkpoint->step.spatial->brake.wheel[0].previous_demand>0.,"C captures simultaneous turning, braking and a material patch split");
        const auto continuous=ride::run(fixture,.4,trace,0.,1,reference);
        const auto expected=spatial_tests::complete_physics_state_bytes(*fixture.runtime);
        const auto expected_steer=bike::io::canonical_json(sp::encode_steer(ride::SpatialRideAccess::causal(*fixture.runtime).steer));
        ride::SpatialRideAccess::restore(*fixture.runtime,*checkpoint);
        const auto resumed=ride::run(fixture,.4,trace,0.,1,reference);
        const auto actual=spatial_tests::complete_physics_state_bytes(*fixture.runtime);
        const auto actual_steer=bike::io::canonical_json(sp::encode_steer(ride::SpatialRideAccess::causal(*fixture.runtime).steer));
        profile_and_physical(continuous,pin);profile_and_physical(resumed,pin);full_duration(continuous);full_duration(resumed);
        bt::require(actual==expected && actual_steer==expected_steer,
            "same-binary complete material/brake/controller/rider/drive/accounting continuation is bitwise identical after restore");
        bt::require(continuous.points.size()==resumed.points.size(),"C continuation observation inventory is preserved");
        for(std::size_t i=0;i<continuous.points.size();++i) {
            const auto& a=continuous.points[i];const auto& b=resumed.points[i];
            bt::require(a.position==b.position && a.quaternion==b.quaternion &&
                std::bit_cast<std::uint64_t>(a.load)==std::bit_cast<std::uint64_t>(b.load) &&
                std::bit_cast<std::uint64_t>(a.steer_work)==std::bit_cast<std::uint64_t>(b.steer_work),
                "C physical continuation observables remain exact at every accepted interval");
        }
        ride::print_report("C",resumed);
    }
}
} // namespace
int main() {return bt::run([]{flat_matrix();bank_matrix();split_grip();continuation();rough_uphill();});}
