#include "outcomes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::spatial {
Outcome evaluate_outcome(const SpatialOutcomeInput& in, OutcomeState& state) noexcept {
    if (state.latched.terminal) return state.latched;
    SpatialReason reason=SpatialReason::none;
    bool valid=true;
    const bool finite=std::isfinite(in.time_s) && in.time_s>=0. && in.tick>=0 &&
        std::isfinite(in.speed_mps) && std::isfinite(in.lateral_position_m) &&
        std::isfinite(in.capsize_roll_rad) && in.capsize_roll_rad>0. &&
        std::isfinite(in.corridor_half_width_m) && in.corridor_half_width_m>0. &&
        std::isfinite(in.bike_half_width_m) && in.bike_half_width_m>=0. &&
        std::isfinite(in.balance_speed_floor_mps) && in.balance_speed_floor_mps>=0. &&
        in.balance_dwell_limit_ticks>0 && (!in.attitude.valid || std::isfinite(in.attitude.roll_rad)) &&
        std::ranges::all_of(in.root.position_world_m,[](double x){return std::isfinite(x);});
    if (!in.engine_finite || !finite) { reason=SpatialReason::numerical_failure;valid=false; }
    else if (!in.contact_query_valid) { reason=SpatialReason::terrain_invalid;valid=false; }
    else if (!in.inside_terrain_domain) { reason=SpatialReason::terrain_outside;valid=false; }
    else if (in.body_ground_contact || (in.attitude.valid && std::abs(in.attitude.roll_rad)>=in.capsize_roll_rad)) {
        reason=SpatialReason::capsize;
        if(state.capsize_dwell_ticks<std::numeric_limits<std::int64_t>::max())++state.capsize_dwell_ticks;
    } else if (std::abs(in.lateral_position_m)+in.bike_half_width_m>in.corridor_half_width_m) {
        reason=SpatialReason::off_corridor;
        if(state.corridor_dwell_ticks<std::numeric_limits<std::int64_t>::max())++state.corridor_dwell_ticks;
    } else {
        if (in.speed_mps<in.balance_speed_floor_mps) {
            if(state.balance_dwell_ticks<std::numeric_limits<std::int64_t>::max())++state.balance_dwell_ticks;
        } else state.balance_dwell_ticks=0;
        if (state.balance_dwell_ticks>=in.balance_dwell_limit_ticks) reason=SpatialReason::balance_lost;
        else if (in.requested_completion) reason=SpatialReason::requested_completion;
    }
    if (reason!=SpatialReason::none) {
        // Keep the hot-path diagnostic empty. The closed reason is emitted by
        // to_string at the cold wire boundary; no formatted string is built here.
        state.latched.reason=reason;state.latched.terminal=true;state.latched.valid=valid;
        state.latched.tick=std::max<std::int64_t>(0,in.tick);
        state.latched.time_s=std::isfinite(in.time_s)?std::max(0.,in.time_s):0.;
        state.latched.position_world_m=in.root.position_world_m;
        for(double& p:state.latched.position_world_m)if(!std::isfinite(p))p=0.;
        state.valid_mask=1;
    }
    return state.latched;
}
} // namespace bike::spatial
