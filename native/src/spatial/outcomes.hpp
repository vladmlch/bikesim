#pragma once

#include "bootstrap_state.hpp"
#include "frames.hpp"
#include "types.hpp"
#include <cstdint>
#include <numbers>

namespace bike::spatial {

struct SpatialOutcomeInput {
    std::int64_t tick = 0;
    double time_s = 0.;
    BodyPose root;
    EulerZyx attitude;
    bool engine_finite = true;
    bool contact_query_valid = true;
    bool body_ground_contact = false;
    double capsize_roll_rad = std::numbers::pi / 3.;
    double corridor_half_width_m = 1.5;
    double bike_half_width_m = .4;
    double lateral_position_m = 0.;
    bool inside_terrain_domain = true;
    bool requested_completion = false;
    double speed_mps = 0.;
    double balance_speed_floor_mps = .5;
    std::int64_t balance_dwell_limit_ticks = 200;
};
// The state owns diagnostic text. Observing a latch must not copy/allocate its
// string inside this noexcept physics boundary.
const Outcome& evaluate_outcome(const SpatialOutcomeInput&, OutcomeState&) noexcept;

} // namespace bike::spatial
