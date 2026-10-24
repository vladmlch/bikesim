#include "support/migration_test.hpp"
#include "spatial/outcomes.hpp"
#include "io/json.hpp"
#include "runtime/research_metrics.hpp"

#include <cmath>
#include <numbers>

int main(){return bt::run([] {
    using namespace bike::spatial;
    SpatialOutcomeInput in;in.tick=42;in.time_s=.21;in.speed_mps=3.;in.attitude.valid=true;
    OutcomeState state;
    in.lateral_position_m=in.corridor_half_width_m-in.bike_half_width_m;
    bt::require(!evaluate_outcome(in,state).terminal,"corridor boundary is inclusive at the bicycle envelope");
    in.lateral_position_m=std::nextafter(in.lateral_position_m,10.);
    bt::require(evaluate_outcome(in,state).reason==SpatialReason::off_corridor,"the full bicycle width crosses the corridor");
    const auto first=state.latched;in.tick=99;in.time_s=10.;in.engine_finite=false;
    bt::require(evaluate_outcome(in,state).tick==first.tick && state.latched.reason==first.reason,"terminal crossing latches once");
    state={};in={};in.speed_mps=3.;in.attitude.valid=true;in.attitude.roll_rad=in.capsize_roll_rad;
    bt::require(evaluate_outcome(in,state).reason==SpatialReason::capsize,"exact capsize threshold is terminal");
    state={};in.attitude.valid=false;
    bt::require(!evaluate_outcome(in,state).terminal,"Euler singularity alone does not fabricate capsize");
    state={};in.contact_query_valid=false;in.body_ground_contact=true;in.requested_completion=true;
    const auto terrain=evaluate_outcome(in,state);
    bt::require(terrain.reason==SpatialReason::terrain_invalid && !terrain.valid,"query exhaustion precedes crash/completion and never means airborne success");
    state={};in.engine_finite=false;
    bt::require(evaluate_outcome(in,state).reason==SpatialReason::numerical_failure,"nonfinite engine precedes terrain failure");
    state={};in.engine_finite=true;in.contact_query_valid=true;in.inside_terrain_domain=false;
    bt::require(evaluate_outcome(in,state).reason==SpatialReason::terrain_outside,"outside-domain precedes physical crash");
    state={};in={};in.balance_dwell_limit_ticks=3;
    for(int tick=0;tick<2;++tick){in.tick=tick;bt::require(!evaluate_outcome(in,state).terminal,"low-speed dwell has not elapsed");}
    in.tick=2;in.speed_mps=2.;(void)evaluate_outcome(in,state);
    bt::require(state.balance_dwell_ticks==0,"recovered speed resets dwell");
    in.speed_mps=0.;
    for(int tick=3;tick<6;++tick){in.tick=tick;(void)evaluate_outcome(in,state);}
    bt::require(state.latched.reason==SpatialReason::balance_lost && state.latched.valid,"physical low-speed loss remains valid");
    state={};in={};in.speed_mps=2.;in.requested_completion=true;
    bt::require(evaluate_outcome(in,state).reason==SpatialReason::requested_completion,"requested completion has lowest priority");
    bt::require(to_string(state.latched.reason)=="completed","spatial completion has its own closed wire label");
    const auto encoded=bike::io::canonical_json(encode_outcome(state));
    bt::require(encoded==bike::io::canonical_json(encode_outcome(decode_outcome(bike::io::parse_json(encoded)))),"outcome codec preserves terminal latch");
    OutcomeState replay_a,replay_b;
    for(std::int64_t tick=0;tick<20;++tick) {
        SpatialOutcomeInput input;input.tick=tick;input.time_s=.005*static_cast<double>(tick);
        input.attitude.valid=true;input.speed_mps=tick<4?0.:3.;input.balance_dwell_limit_ticks=6;
        input.root.position_world_m={.02*static_cast<double>(tick),0.,1.};
        input.lateral_position_m=tick<12?0.:1.2;
        input.requested_completion=tick>=12;
        const auto a=evaluate_outcome(input,replay_a),b=evaluate_outcome(input,replay_b);
        bt::require(a.reason==b.reason && bike::io::canonical_json(encode_outcome(replay_a))==
            bike::io::canonical_json(encode_outcome(replay_b)),"identical accepted inputs produce identical dwell counters and terminal bytes");
    }
    bt::require(replay_a.latched.reason==SpatialReason::off_corridor && replay_a.latched.tick==12,
        "physical corridor crossing wins over simultaneous requested completion");
    runtime::SpatialEpisodeMetrics metrics;metrics.requested_duration_s=1.;metrics.start_position_m=2.;
    Outcome valid;
    runtime::update_spatial_metrics(metrics,20,.21,.01,2.1,valid);
    bt::require(std::abs(metrics.valid_duration_s-.01)<=1e-15,"metrics count only supplied accepted intervals, excluding an unobserved prefix");
    runtime::update_spatial_metrics(metrics,20,.21,.01,20.,valid);
    bt::require(std::abs(metrics.completed_distance_m-.1)<=1e-15,"duplicate publication cannot increase distance");
    runtime::update_spatial_metrics(metrics,21,.22,.01,2.2,valid);
    Outcome invalid_outcome;invalid_outcome.reason=SpatialReason::terrain_invalid;invalid_outcome.terminal=true;
    invalid_outcome.valid=false;invalid_outcome.tick=22;invalid_outcome.time_s=.23;
    runtime::update_spatial_metrics(metrics,22,.23,.01,100.,invalid_outcome);
    bt::require(!metrics.valid && metrics.terminal_reason==SpatialReason::terrain_invalid &&
        std::abs(metrics.valid_duration_s-.02)<=1e-15 && std::abs(metrics.completed_distance_m-.2)<=1e-15,
        "an invalid endpoint retains only the valid accepted prefix, never its spurious distance");
    runtime::update_spatial_metrics(metrics,23,.24,.01,200.,valid);
    bt::require(std::abs(metrics.completed_distance_m-.2)<=1e-15,"later publication cannot extend a latched terminal episode");
    runtime::SpatialEpisodeMetrics complete;complete.requested_duration_s=.03;
    for(std::int64_t tick=0;tick<3;++tick)runtime::update_spatial_metrics(complete,tick,.01*static_cast<double>(tick+1),.01,
        .1*static_cast<double>(tick+1),valid);
    bt::require(complete.valid_duration_s==complete.requested_duration_s,"a complete accepted episode reaches its exact requested duration");
    runtime::SpatialEpisodeMetrics gap;gap.requested_duration_s=1.;runtime::update_spatial_metrics(gap,0,.01,.01,0.,valid);
    bt::invalid([&]{runtime::update_spatial_metrics(gap,2,.03,.01,0.,valid);});
});}
