#include "launch_ride_support.hpp"

#include <algorithm>
#include <format>
#include <limits>
#include <print>
#include <string>
#include <string_view>

#include "research/evidence.hpp"
#include "runtime/samples.hpp"
#include "support/fixture_test.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"

namespace {

namespace research = bike::research;
namespace sw = runtime::sample_wire;
namespace terrain = bike::terrain;

struct CalibratedTrackMetrics {
    std::string outcome;
    double end_position_m = 0.;
    double end_time_s = 0.;
    double saddle_min_n = std::numeric_limits<double>::infinity();
    double saddle_max_n = -std::numeric_limits<double>::infinity();
    double front_pedal_min_n = std::numeric_limits<double>::infinity();
    double rear_pedal_min_n = std::numeric_limits<double>::infinity();
    double grip_left_max_pull_n = 0.;
    double grip_right_max_pull_n = 0.;
    double max_residual_ratio = 0.;
    double final_residual_j = 0.;
    double peak_soil_elastic_j = 0.;
    double cumulative_ground_plastic_loss_j = 0.;
    std::size_t measured_intervals = 0;
};

[[nodiscard]] CalibratedTrackMetrics run_calibrated_track(
    std::string_view track_relative_path, bool expect_deformable_soil,
    bool allow_roots_single_contact_hazard) {
    const terrain::TrackSpec track =
        terrain::load_track(launch_ride::repo_file(track_relative_path));
    const auto hazards = terrain::single_contact_hazards(track);
    if (allow_roots_single_contact_hazard) {
        bt::require(
            hazards.size() == 1 &&
                hazards.front().find("Roots has faces the 2D tyre cannot ride") !=
                    std::string::npos,
            std::format("{}: expected only Roots in single_contact_hazards, got {}",
                        track_relative_path, hazards.size()));
    } else {
        bt::require(hazards.empty(),
                    std::format("{}: expected zero single_contact_hazards, got {}",
                                track_relative_path, hazards.size()));
    }

    auto env = launch_ride::launch_environment(
        "examples/research/viewer_physics_pneumatic.toml", track, 60.0);
    env.runtime->set_finish_position(env.track_length_m);

    while (!env.runtime->snapshot().outcome.has_value() &&
           env.runtime->snapshot().step < env.max_steps) {
        const auto result = env.runtime->advance(
            env.runtime->snapshot().step + env.control_steps,
            runtime::RideControl{}, 0., 0., std::nullopt);
        if (result.reason == "outcome") break;
    }

    const auto snapshot = env.runtime->snapshot();
    const std::string outcome = snapshot.outcome.value_or("<none>");
    bt::require(snapshot.outcome.has_value(),
                std::format("{}: ride reached a terminal outcome",
                            track_relative_path));
    bt::require(outcome == "end_of_track",
                std::format("{}: expected outcome end_of_track, got {}",
                            track_relative_path, outcome));
    bt::require(!outcome.starts_with("crash:"),
                std::format("{}: unexpected crash {}", track_relative_path,
                            outcome));
    bt::require(!snapshot.first_failure.has_value(),
                std::format("{}: expected zero first_failure",
                            track_relative_path));
    bt::require(sw::boolean(sw::required(snapshot.model_status, "model_valid")),
                std::format("{}: expected model_valid == true",
                            track_relative_path));

    const auto rows = env.runtime->recorded_intervals();
    bt::require(!rows.empty(),
                std::format("{}: expected recorded intervals",
                            track_relative_path));

    CalibratedTrackMetrics metrics;
    metrics.outcome = outcome;
    metrics.end_position_m = rows.back()->qpos[0];
    metrics.end_time_s = rows.back()->end_time_s;

    for (const auto &row : rows) {
        bt::require(research::model_violations(*row).empty(),
                    std::format("{}: interval {} had model violations",
                                track_relative_path, row->interval_id));

        const auto &balance =
            sw::object(sw::required(row->channels, "rider_balance"));
        bt::require(!sw::boolean(sw::required(balance, "balance_lost")),
                    std::format("{}: unexpected balance_lost at t={:.4f}s",
                                track_relative_path, row->end_time_s));

        const auto &attachments =
            sw::object(sw::required(row->channels, "attachment_samples"));
        const double saddle_n = sw::number(sw::required(
            sw::object(sw::required(attachments, "saddle")), "normal_n"));
        const double front_pedal_n = sw::number(sw::required(
            sw::object(sw::required(attachments, "foot_front")), "normal_n"));
        const double rear_pedal_n = sw::number(sw::required(
            sw::object(sw::required(attachments, "foot_rear")), "normal_n"));
        const double grip_left_pull = sw::number(sw::required(
            sw::object(sw::required(attachments, "grip_left")), "pull_n"));
        const double grip_right_pull = sw::number(sw::required(
            sw::object(sw::required(attachments, "grip_right")), "pull_n"));

        metrics.saddle_min_n = std::min(metrics.saddle_min_n, saddle_n);
        metrics.saddle_max_n = std::max(metrics.saddle_max_n, saddle_n);
        metrics.front_pedal_min_n =
            std::min(metrics.front_pedal_min_n, front_pedal_n);
        metrics.rear_pedal_min_n =
            std::min(metrics.rear_pedal_min_n, rear_pedal_n);
        metrics.grip_left_max_pull_n =
            std::max(metrics.grip_left_max_pull_n, grip_left_pull);
        metrics.grip_right_max_pull_n =
            std::max(metrics.grip_right_max_pull_n, grip_right_pull);

        if (const auto *energy_w = sw::find(row->channels, "energy");
            energy_w != nullptr && !sw::is_none(*energy_w)) {
            const auto &energy_obj = sw::object(*energy_w);
            if (sw::boolean(sw::required(energy_obj, "measured"))) {
                const auto eq = research::energy_quality(energy_obj, 0.05);
                bt::require(
                    eq.acceptable && eq.residual_ratio <= 0.05,
                    std::format("{}: energy_quality failed at t={:.4f}s "
                                "(ratio={:.6f})",
                                track_relative_path, row->end_time_s,
                                eq.residual_ratio));
                metrics.max_residual_ratio =
                    std::max(metrics.max_residual_ratio, eq.residual_ratio);
                metrics.final_residual_j =
                    sw::number(sw::required(energy_obj, "residual_j"));
                ++metrics.measured_intervals;
            }
            const double ground_plastic = sw::number(
                sw::required(energy_obj, "ground_plastic_loss_step_j"));
            bt::require(ground_plastic >= 0.0,
                        "nonnegative ground_plastic_loss_step_j");
            metrics.cumulative_ground_plastic_loss_j += ground_plastic;

            const auto &elastic =
                sw::object(sw::required(energy_obj, "elastic_energy_j"));
            if (const auto *soil = sw::find(elastic, "soil");
                soil != nullptr && !sw::is_none(*soil)) {
                const double soil_j = sw::number(*soil);
                bt::require(soil_j >= 0.0, "nonnegative elastic_energy_j.soil");
                metrics.peak_soil_elastic_j =
                    std::max(metrics.peak_soil_elastic_j, soil_j);
            }
        }
    }

    const auto acct = env.runtime->accounting_state();
    const auto &history = sw::object(sw::required(acct, "history"));
    const auto &support =
        sw::object(sw::required(history, "rider_support"));
    const auto support_min = [&](const char *key) {
        return sw::number(sw::required(
            sw::object(sw::required(support, key)), "min_n"));
    };
    metrics.saddle_min_n = std::min(metrics.saddle_min_n, support_min("saddle"));
    metrics.front_pedal_min_n =
        std::min(metrics.front_pedal_min_n, support_min("front_pedal"));
    metrics.rear_pedal_min_n =
        std::min(metrics.rear_pedal_min_n, support_min("rear_pedal"));

    bt::require(metrics.measured_intervals > 0,
                std::format("{}: expected measured energy intervals",
                            track_relative_path));
    bt::require(metrics.saddle_min_n >= -6.0,
                std::format("{}: saddle.min_n={:.3f} < -6.0",
                            track_relative_path, metrics.saddle_min_n));
    bt::require(metrics.front_pedal_min_n >= -6.0,
                std::format("{}: front_pedal.min_n={:.3f} < -6.0",
                            track_relative_path, metrics.front_pedal_min_n));
    bt::require(metrics.rear_pedal_min_n >= -6.0,
                std::format("{}: rear_pedal.min_n={:.3f} < -6.0",
                            track_relative_path, metrics.rear_pedal_min_n));
    bt::require(metrics.saddle_max_n <= 1750.0,
                std::format("{}: saddle.max_n={:.3f} > 1750.0",
                            track_relative_path, metrics.saddle_max_n));
    bt::require(metrics.grip_left_max_pull_n <= 300.0,
                std::format("{}: grip_left max pull={:.3f} > 300.0",
                            track_relative_path, metrics.grip_left_max_pull_n));
    bt::require(metrics.grip_right_max_pull_n <= 300.0,
                std::format("{}: grip_right max pull={:.3f} > 300.0",
                            track_relative_path, metrics.grip_right_max_pull_n));

    if (expect_deformable_soil) {
        bt::require(metrics.cumulative_ground_plastic_loss_j > 0.0,
                    std::format("{}: expected positive cumulative "
                                "ground_plastic_loss_step_j",
                                track_relative_path));
        bt::require(metrics.peak_soil_elastic_j > 0.0,
                    std::format("{}: expected positive peak "
                                "elastic_energy_j.soil",
                                track_relative_path));
    }

    std::println(
        stderr,
        "[calibrated_track] {} outcome={} x={:.4f}m t={:.4f}s "
        "saddle=[{:.2f}, {:.2f}]N pedals=[{:.2f}, {:.2f}]N "
        "grip_pull=[{:.2f}, {:.2f}]N max_ratio={:.6f} final_res={:.3f}J "
        "peak_soil={:.4f}J sum_plastic={:.4f}J",
        track_relative_path, metrics.outcome, metrics.end_position_m,
        metrics.end_time_s, metrics.saddle_min_n, metrics.saddle_max_n,
        metrics.front_pedal_min_n, metrics.rear_pedal_min_n,
        metrics.grip_left_max_pull_n, metrics.grip_right_max_pull_n,
        metrics.max_residual_ratio, metrics.final_residual_j,
        metrics.peak_soil_elastic_j, metrics.cumulative_ground_plastic_loss_j);
    return metrics;
}

void run() {
    (void)run_calibrated_track(
        "examples/research/rough_uphill_trail_calibrated.toml", false, true);
    (void)run_calibrated_track(
        "examples/research/alpine_scree_deformable.toml", true, false);
}

} // namespace

int main() { return bt::run(run); }
