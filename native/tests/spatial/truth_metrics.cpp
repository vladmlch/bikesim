#include "support/migration_test.hpp"
#include "runtime/spatial_truth.hpp"
#include "runtime/research_metrics.hpp"
#include "runtime/setup_values.hpp"
#include "io/value.hpp"
#include "spatial/bootstrap_state.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>

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
    // Key set and order are the channel_spatial / research-transition truth catalog (catalog-v2.json).
    truth.contact_valid={true,true};truth.normal_load_n={400.,380.};truth.support_normal_world[1]={0.,0.,1.};
    constexpr std::array<std::string_view,21> catalog{"root_position_world_m","root_quat_world_from_body_wxyz",
        "root_linear_velocity_world_mps","root_angular_velocity_world_rad_s","attitude_valid","roll_rad","pitch_rad","heading_rad",
        "steer_angle_rad","steer_rate_rad_s","steer_torque_nm","steer_work_j","lateral_displacement_m",
        "front_contact_valid","front_lateral_slip_mps","front_normal_load_n","front_support_normal_world",
        "rear_contact_valid","rear_lateral_slip_mps","rear_normal_load_n","rear_support_normal_world"};
    const auto full=spatial_truth_to_wire(truth);
    const auto& members=bike::io::as_object(full,"truth");
    bool catalog_order=members.size()==catalog.size();
    for(std::size_t i=0;catalog_order&&i<catalog.size();++i)catalog_order=members[i].first==catalog[i];
    bt::require(catalog_order,"spatial truth wire keys equal the recording catalog fields in order");
    truth.contact_valid={true,false};truth.normal_load_n={400.,0.};
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
    // Failed numerical quality is applied before accumulation (research.cpp): the physical reason survives, the endpoint never counts.
    runtime::SpatialEpisodeMetrics rejected{.requested_duration_s=1.};rejected.valid=false;
    bike::spatial::Outcome completion;completion.terminal=true;completion.reason=bike::spatial::SpatialReason::requested_completion;
    completion.tick=0;completion.time_s=.1;
    runtime::update_spatial_metrics(rejected,0,.1,.1,5.,completion);
    bt::require(!rejected.valid&&rejected.valid_duration_s==0.&&rejected.completed_distance_m==0.&&
        rejected.terminal_reason==bike::spatial::SpatialReason::requested_completion&&rejected.terminal_tick==0,
        "a quality-rejected terminal interval keeps its reason but adds no valid duration or distance");
    // as_wire() key order is the published metric order. The continuation reader
    // (research.cpp prepare_continuation, exact()) checks only the twelve-key set plus four cursor keys.
    constexpr std::array<std::string_view,12> metric_keys{"spatial_schema","requested_duration_s","valid_duration_s","completed_distance_m",
        "terminal_reason","terminal_tick","front_lateral_slip_path_m","rear_lateral_slip_path_m","front_unloaded_s","front_lift_s","flight_s","valid"};
    const auto wire=a.as_wire();
    bool metric_order=wire.size()==metric_keys.size();
    for(std::size_t i=0;metric_order&&i<metric_keys.size();++i)metric_order=wire[i].first==metric_keys[i];
    bt::require(metric_order&&runtime::sample_wire::integer(runtime::sample_wire::required(wire,"terminal_tick"))==9,
        "spatial episode metrics publish their wire keys in checkpoint order");
});}
