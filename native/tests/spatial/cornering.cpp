// W35: numerical acceptance is pending V06. This executable performs real
// native solves/episodes when explicitly run; no prerecorded result is used.
#include "support/migration_test.hpp"
#include "spatial/support/ideal_cornering.hpp"

#include <deque>

namespace {
namespace ride=spatial_tests::ride;
using ride::SpatialRideAccess;
using ride::Point;
constexpr double degree=std::numbers::pi/180.;
bool steady(const std::deque<Point>& window) {
    if(window.size()<2 || window.back().time-window.front().time<2.-1e-9)return false;
    std::vector<double> yaw,roll,load;
    for(const auto& p:window){if(!p.valid || p.hold || p.load<=0.)return false;yaw.push_back(p.yaw_rate);roll.push_back(p.roll);load.push_back(p.load);}
    return ride::coefficient_of_variation(yaw)<.01 && ride::coefficient_of_variation(roll)<.01 &&
        ride::coefficient_of_variation(load)<.01;
}
void native_mtb(double radius,double target_speed) {
    auto fixture=ride::arena_rig(radius,target_speed,true);
    auto& rt=*fixture->runtime;const double dt=fixture->bootstrap->config.timestep_s;
    const auto first=ride::observe(rt);const ride::Vec3 center{first.position[0],first.position[1]+radius,0.};
    const double lean=-std::atan(target_speed*target_speed/(ride::gravity*radius));
    const double steer_reference=std::atan(1.2/radius);
    const auto observation_stride=static_cast<std::int64_t>(std::llround(.005/dt));
    std::deque<Point> window;std::vector<Point> scored;std::optional<double> score_start;
    double integral=0.,requested_torque=0.;std::optional<std::string> terminal;
    for(std::int64_t step=1;step<=static_cast<std::int64_t>(std::llround(30./dt));++step) {
        auto& physical=SpatialRideAccess::physical(rt);auto& model=SpatialRideAccess::stepper(rt);
        const auto pose=bike::spatial::body_pose(*model.model(),*model.data(),physical.topology()->root);
        const auto radial=ride::sub(pose.position_world_m,center);
        const double actual_radius=std::hypot(radial[0],radial[1]);
        const double heading=std::atan2(radial[1],radial[0])+std::numbers::pi/2.;
        SpatialRideAccess::reference_tick(rt,lean,heading,radius-actual_radius,steer_reference);
        if((step-1)%observation_stride==0) {
            const double error=target_speed-physical.longitudinal_speed_mps();
            integral=std::clamp(integral+.005*error,-3.,3.);
            requested_torque=std::clamp(12.*error+8.*integral,0.,85.);
        }
        runtime::RideControl control;control.motor_torque_nm=requested_torque;control.motor_limit_nm=85.;control.human_torque_nm=0.;
        const auto result=rt.advance(step,control,0.,0.,std::nullopt);
        if(result.outcome){terminal=result.outcome;break;}
        if(step%observation_stride!=0)continue;
        const auto p=ride::observe(rt);
        window.push_back(p);while(window.size()>1 && window.back().time-window[1].time>=2.)window.pop_front();
        if(!score_start && steady(window))score_start=p.time;
        if(score_start && !steady(window)){terminal="steady_window_lost";break;}
        if(score_start)scored.push_back(p);
        if(score_start && p.time-*score_start>=5.-1e-9)break;
    }
    rt.flush();
    if(!score_start || scored.empty() || scored.back().time-*score_start<5.-1e-9) {
        std::cout << "{\"suite\":\"cornering\",\"radius_m\":" << radius << ",\"speed_mps\":" << target_speed
                  << ",\"status\":\"inconclusive\",\"reason\":\"" << (terminal?*terminal:"steady_window_not_reached") << "\"}\n";
        bt::require(false,"inconclusive: full MTB did not reach two seconds of <1% steady variation followed by five valid seconds");
    }
    std::vector<double> speeds;double force_error=0.,momentum_error=0.;
    for(const auto& p:scored) {
        speeds.push_back(std::hypot(p.velocity[0],p.velocity[1]));
        bt::require(p.valid && !p.hold && p.hold_work==0. && p.load>0.,"cornering scores only valid tyre support without bootstrap work");
    }
    for(std::size_t i=1;i+1<scored.size();++i) {
        const double duration=scored[i+1].time-scored[i-1].time;
        const auto acceleration=ride::scale(ride::sub(scored[i+1].velocity,scored[i-1].velocity),1./duration);
        const auto derivative=ride::scale(ride::sub(scored[i+1].momentum,scored[i-1].momentum),1./duration);
        force_error=std::max(force_error,ride::norm(ride::sub(scored[i].force,ride::scale(acceleration,scored[i].mass)))/(scored[i].mass*ride::gravity));
        const double scale=std::max({1.,ride::norm(scored[i].moment),std::abs(scored[i].yaw_rate)*ride::norm(scored[i].momentum)});
        momentum_error=std::max(momentum_error,ride::norm(ride::sub(scored[i].moment,derivative))/scale);
    }
    const double fitted=ride::circle_radius(scored),speed=ride::mean(speeds);
    std::cout << "{\"suite\":\"cornering_mtb\",\"requested_radius_m\":" << radius << ",\"measured_radius_m\":" << fitted
              << ",\"mean_speed_mps\":" << speed << ",\"force_error_mg\":" << force_error
              << ",\"angular_momentum_error\":" << momentum_error << ",\"steady_scored_duration_s\":"
              << scored.back().time-*score_start << ",\"profile_digest\":\""
              << fixture->bootstrap->config.spatial->tyre.lateral_coefficients_sha256 << "\",\"steer_digest\":\""
              << fixture->bootstrap->config.spatial->steer.profile_sha256 << "\"}\n";
    bt::require(std::abs(fitted-radius)<=.05*radius,"full MTB COM circle fit is within five percent of commanded radius");
    bt::require(std::abs(speed-target_speed)<=.02*target_speed,"full MTB steady COM speed is within two percent");
    bt::require(force_error<=1e-3,"full MTB external force closes COM acceleration to 1e-3 mg");
    bt::require(momentum_error<=.01,"full MTB momentum balance includes actual wheel gyroscopic momentum");
}
} // namespace
int main() { return bt::run([] {
    for(double radius:{10.,20.,40.})for(double speed:{3.,5.,8.}) {
        constexpr double mu=.8;
        if(speed*speed/(ride::gravity*radius)>.8*mu) {
            std::cout << "{\"suite\":\"cornering\",\"radius_m\":" << radius << ",\"speed_mps\":" << speed
                      << ",\"status\":\"excluded\",\"reason\":\"lateral demand exceeds 0.8 mu\"}\n";
            continue;
        }
        ride::IdealCorner ideal;const auto reference=ideal.solve(radius,speed);
        const double expected_force=reference.mass*reference.speed*reference.speed/reference.radius;
        bt::require(std::abs(reference.lean_com-std::atan(reference.speed*reference.speed/(ride::gravity*reference.radius)))<=degree,
            "thin-contact rigid-rider COM lean is within one degree of centripetal balance");
        bt::require(std::abs(reference.lateral_force-expected_force)<=.02*expected_force,
            "steady ideal fixture balances measured COM lateral acceleration within two percent");
        std::cout << "{\"suite\":\"cornering_ideal\",\"radius_m\":" << reference.radius << ",\"speed_mps\":" << reference.speed
                  << ",\"lean_com_rad\":" << reference.lean_com << ",\"lateral_force_n\":" << reference.lateral_force << "}\n";
        native_mtb(radius,speed);
    }
}); }
