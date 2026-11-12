// launch_ride_support.hpp — the shared welded-pedelec ride harness for
// the launch-plan task tests (task 2's rider-senses legs and the tasks
// that follow). The body is the environment()/PedelecEnv/track block of
// test_realistic_pedelec_acceptance.cpp moved verbatim — every field the
// acceptance test uses is retained (review P7): control/max step counts,
// track length, the root slide's dof/qpos adrs, the sorted cassette and
// the shift band/smoothing constants. The acceptance test consumes this
// header unchanged; new tests get the same plant by calling
// launch_ride::environment(track, duration_s).
#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "configuration/load.hpp"
#include "model_access.hpp"
#include "model/rider_geometry.hpp"
#include "research/environment.hpp"
#include "runtime/runtime.hpp"
#include "stepper.hpp"
#include "support/golden.hpp"
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"

namespace launch_ride {

namespace research = bike::research;
namespace terrain = bike::terrain;

// The live example document — bt::oracle::root() is
// <repo>/tests/fixtures/golden; the welded TOML sits beside the tests
// three levels up (same seam as test_seated_pedaling_cycle.cpp).
[[nodiscard]] inline std::filesystem::path
repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

// _environment(tmp_path, track, duration_s): welded physics + the probe
// track + duration + dt .00125 (the TOML's own) + energy-tolerance 1e9 +
// diagnostic model limits + record-decimation 1 + initial-speed 0.
struct PedelecEnv {
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t control_steps = 0, max_steps = 0;
    double track_length_m = 0.;
    int root_x_dof = -1, root_x_qpos = -1;
    std::vector<int> cassette;
    double shift_band_min_rpm = 0., shift_band_max_rpm = 0.,
           shift_cadence_tau_s = 0.;
};

// Launch part 3a: the environment with a physics edit applied right after
// the profile load, an explicit record decimation and capture mode (the
// reaction channel's decimated-ride test: legacy capture keeps only every
// decimation-th interval, accounting.cpp:764-776, so a full capture is
// needed to see unmeasured intervals).
template <typename Edit>
[[nodiscard]] PedelecEnv environment(terrain::TrackSpec track,
                                     double duration_s, Edit &&edit,
                                     int record_decimation,
                                     runtime::ResearchCaptureMode capture_mode) {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    std::forward<Edit>(edit)(*request.setup.model.physics);
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    request.setup.track = std::move(track);
    request.setup.model.physics->initial_speed_mps = 0.;
    request.experiment.duration_s = duration_s;
    request.experiment.record_decimation = record_decimation;
    request.capture_mode = capture_mode;
    // --energy-tolerance 1e9 / --diagnostic-model-limits
    request.experiment.maximum_energy_residual_ratio = 1e9;
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig{};
    // [research] is profile-owned now: copy the loaded physics
    // section across instead of relying on struct defaults.
    request.experiment.control_period_s =
        request.setup.model.physics->research.control_period_s;
    request.experiment.actuator_delay_s =
        request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s =
        request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s =
        request.setup.model.physics->research.sensor_maximum_age_s;
    // The welded .00125 s physics grid: the default .001 s sensor period
    // is not a step multiple — .005 is the 4-step period the other
    // welded fixtures use.
    request.sensors.sample_period_s = .005;
    // prepare_environment consumes the request's physics — copy what the
    // shift check needs before the move.
    const auto cassette =
        request.setup.model.physics->drive.shifting.cassette;
    const auto band_min =
        request.setup.model.physics->drive.shifting.target_cadence_min_rpm;
    const auto band_max =
        request.setup.model.physics->drive.shifting.target_cadence_max_rpm;
    const auto cadence_tau =
        request.setup.model.physics->drive.shifting
            .cadence_smoothing_tau_s;
    const auto prepared = research::prepare_environment(std::move(owner));
    PedelecEnv env;
    env.cassette = cassette;
    env.shift_band_min_rpm = band_min;
    env.shift_band_max_rpm = band_max;
    env.shift_cadence_tau_s = cadence_tau;
    // Position in the scan order is what the nearest-in-band check below
    // reasons about: sort here the same way the shifter constructor
    // normalizes its own copy.
    std::ranges::sort(env.cassette);
    env.control_steps = prepared->bootstrap.config.control_steps;
    env.max_steps = prepared->bootstrap.config.max_steps;
    env.track_length_m = prepared->bootstrap.config.track_length_m;
    env.runtime = std::make_unique<runtime::NativeRideRuntime>(
        std::move(prepared->bootstrap.ride));
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &stepper,
                          const runtime::PhysicalStep &) {
            const int root =
                mj_name2id(stepper.model(), mjOBJ_JOINT, "root_x");
            bt::require(root >= 0, "the welded plant names its root slide");
            env.root_x_dof = model_access::readonly_buffer(
                stepper.model()->jnt_dofadr,
                stepper.model()->njnt)[static_cast<std::size_t>(root)];
            env.root_x_qpos = model_access::readonly_buffer(
                stepper.model()->jnt_qposadr,
                stepper.model()->njnt)[static_cast<std::size_t>(root)];
        });
    return env;
}

[[nodiscard]] inline PedelecEnv environment(terrain::TrackSpec track,
                                            double duration_s) {
    return environment(std::move(track), duration_s, [](auto &) {}, 1,
                       runtime::ResearchCaptureMode::legacy);
}

[[nodiscard]] inline terrain::TrackSpec track(std::string_view knots,
                                              double length) {
    return terrain::track_from_dict(terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = " + std::to_string(length) +
        "\nsurface = \"hardpack\"\ngrade_profile = { knots = " +
        std::string(knots) + " }\n"));
}

} // namespace launch_ride
