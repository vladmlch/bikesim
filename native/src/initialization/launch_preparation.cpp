#include "launch_preparation.hpp"

#include <cmath>
#include <stdexcept>

namespace bike::initialization {

double rider_start_grade(const terrain::TrackSpec &track, double start_x_m, double lookahead_m) {
    if (!std::isfinite(start_x_m) || !std::isfinite(lookahead_m) || lookahead_m < 0.)
        throw std::invalid_argument("rider start grade needs a finite start and a nonnegative lookahead");
    return track.grade_profile ? track.grade_profile->slope(start_x_m + lookahead_m) : 0.;
}

::rider::LaunchPreparation prepare_start(configuration::SimulationPhysicsConfig &physics,
                                         const terrain::TrackSpec &track, double start_x_m) {
    const auto start = ::rider::prepare_launch(
        physics.rider_launch,
        rider_start_grade(track, start_x_m, physics.articulated.road_lookahead_m));
    physics.drive.gearing.rear_teeth = start.rear_teeth;
    physics.drive.crank_phase_rad = start.crank_phase_rad;
    return start;
}

::rider::LaunchPreparation prepare_request(SetupRequest &request) {
    if (!request.model.physics)
        throw std::invalid_argument("launch preparation requires an explicit physical configuration");
    return prepare_start(*request.model.physics, request.track, request.start_x_m);
}

} // namespace bike::initialization
