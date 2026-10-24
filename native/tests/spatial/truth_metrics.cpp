#include "support/migration_test.hpp"
#include "runtime/spatial_truth.hpp"
#include "runtime/research_metrics.hpp"
#include "runtime/setup_values.hpp"
#include "io/json.hpp"
#include <cmath>
#include <limits>

int main(){return bt::run([]{
    using namespace runtime::spatial;
    bt::require(integrate_abs_slip(0.,-2.,.25)==.5,"slip distance integrates absolute velocity");
    bt::invalid([]{(void)integrate_abs_slip(0.,1.,0.);});bt::invalid([]{(void)integrate_abs_slip(-1.,1.,.1);});
    bt::invalid([]{(void)integrate_abs_slip(0.,std::numeric_limits<double>::infinity(),.1);});
    double split=0.;for(int i=0;i<10;++i)split=integrate_abs_slip(split,-2.,.025);
    bt::require(std::abs(split-integrate_abs_slip(0.,2.,.25))<=1e-12,"slip integral is invariant under interval splitting and mirror sign");
    SpatialTruth truth;truth.root.position_world_m={2.,3.,4.};truth.attitude={.roll_rad=.2,.pitch_rad=.1,.yaw_rad=.3,.valid=true};
    truth.contact_valid={true,false};truth.normal_load_n={400.,0.};truth.lateral_slip_mps={-.3,0.};truth.support_normal_world[0]={0.,0.,1.};
    auto value=spatial_truth_to_wire(truth);using namespace runtime::setupvalue;
    bt::require(bike::io::as_array(field(value,"root_position_world_m"),"position").size()==3,"root retains world lateral position");
    bt::require(bike::io::is_null(field(value,"rear_normal_load_n"))&&bike::io::is_null(field(value,"rear_support_normal_world")),"invalid contact emits null evidence");
    truth.attitude.valid=false;value=spatial_truth_to_wire(truth);
    bt::require(bike::io::is_null(field(value,"roll_rad"))&&bike::io::is_null(field(value,"heading_rad"))&&
        bike::io::as_array(field(value,"root_quat_world_from_body_wxyz"),"quaternion").size()==4,"singular Euler display never discards quaternion");
    runtime::SpatialEpisodeMetrics a{.requested_duration_s=1.},b{.requested_duration_s=1.};
    bike::spatial::Outcome outcome;
    for(std::int64_t i=0;i<10;++i){outcome.terminal=i==9;outcome.reason=outcome.terminal?bike::spatial::SpatialReason::requested_completion:bike::spatial::SpatialReason::none;
        outcome.tick=i;outcome.time_s=static_cast<double>(i+1)*.1;
        runtime::update_spatial_metrics(a,i,outcome.time_s,.1,static_cast<double>(i+1),outcome);
        runtime::update_spatial_metrics(b,i,outcome.time_s,.1,static_cast<double>(i+1),outcome);
        runtime::update_spatial_metrics(b,i,outcome.time_s,.1,static_cast<double>(i+1),outcome);}
    bt::require(a.valid_duration_s==b.valid_duration_s&&a.terminal_tick==9&&b.terminal_tick==9&&a.completed_distance_m==b.completed_distance_m,
        "metrics publish the physical terminal tick once regardless of duplicate polling");
});}
