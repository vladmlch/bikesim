// initialization/launch_preparation.hpp — the rider's start preparation for
// one setup request (launch plan, part 2). Before the model exists the
// rider judges the grade ahead of the start point and takes the cog, pedal
// angle and lean the launch table gives for it; the cog and angle become
// the two computed drive members every later stage reads.
#pragma once

#include "../configuration/model_config.hpp"
#include "../rider/launch_config.hpp"
#include "../terrain/types.hpp"
#include "setup_owner.hpp"

namespace bike::initialization {

// The grade the rider sees before the start: the track's grade profile
// lookahead_m ahead of the start point, 0 without a grade profile.
// Obstacles are not part of it — the heightfield does not exist yet.
[[nodiscard]] double rider_start_grade(const terrain::TrackSpec &track,
                                       double start_x_m, double lookahead_m);

// Prepares the start for this physics/track/start point and writes the cog
// and pedal angle into drive.gearing.rear_teeth / drive.crank_phase_rad.
::rider::LaunchPreparation prepare_start(configuration::SimulationPhysicsConfig &physics,
                                         const terrain::TrackSpec &track, double start_x_m);

// prepare_start on the request's own physics, track and start point. Pure
// and idempotent: every path that builds a model or a contract from a
// request calls it first (make_setup, the research environment, the
// validation rig capture), and all of them get the same start.
::rider::LaunchPreparation prepare_request(SetupRequest &request);

} // namespace bike::initialization
