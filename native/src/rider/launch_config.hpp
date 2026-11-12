// rider/launch_config.hpp — the rider's launch experience table (launch
// plan, part 2): the [rider_launch] profile section and the pure start
// preparation it drives. The profile authors only this table; setup
// derives drive.gearing.rear_teeth and drive.crank_phase_rad from it for
// the start grade, and the settle and the first running posture hold its
// lean. Part 3 adds the hold/release/effort/handover fields here.
#pragma once

#include <limits>
#include <vector>

namespace rider {

struct LaunchConfig {
    // Stepwise start cog: start_cog_teeth[i] applies for grades in
    // [start_cog_grade_breaks[i-1], start_cog_grade_breaks[i]); the breaks
    // ascend strictly and number one fewer than the cogs.
    std::vector<double> start_cog_grade_breaks;
    std::vector<int> start_cog_teeth;
    // The crank angle the rider sets the working pedal to, rad — the angle
    // drive.crank_phase_rad carries. NaN until a profile sets it: the
    // record has no code default.
    double start_pedal_angle_rad = std::numeric_limits<double>::quiet_NaN();
    // Torso lean by grade, rad: piecewise linear over strictly ascending
    // knots, held at the end values outside them.
    std::vector<double> start_lean_grades;
    std::vector<double> start_lean_rad;
    // Launch part 3b — the launch plan (rider/launch_plan.hpp). Off in
    // every profile until part 4 switches it on.
    bool plan_enabled = false;
    double standstill_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double standstill_crank_rpm = std::numeric_limits<double>::quiet_NaN();
    double effort_rate_nm_s = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_asphalt_nm = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_hardpack_nm = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_loose_nm = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_wet_nm = std::numeric_limits<double>::quiet_NaN();
    double effort_cap_scree_nm = std::numeric_limits<double>::quiet_NaN();
    // Felt crank torque that releases the brakes, piecewise linear over
    // strictly ascending grade knots, held at the end values outside them.
    std::vector<double> release_torque_grades;
    std::vector<double> release_torque_nm;
    double brake_release_s = std::numeric_limits<double>::quiet_NaN();
    double max_hold_s = std::numeric_limits<double>::quiet_NaN();
    double upshift_crank_rpm = std::numeric_limits<double>::quiet_NaN();
    double end_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double end_crank_rpm = std::numeric_limits<double>::quiet_NaN();
    double end_hold_s = std::numeric_limits<double>::quiet_NaN();
    double max_launch_s = std::numeric_limits<double>::quiet_NaN();
    double handover_s = std::numeric_limits<double>::quiet_NaN();
};

// The start the table gives for one grade.
struct LaunchPreparation {
    int rear_teeth = 0;
    double crank_phase_rad = 0.;
    double lean_rad = 0.;
};

// The table's own shape and domain; throws std::invalid_argument naming the
// rider_launch field. Rules that need other records (cassette, posture
// bounds) live in configuration post_init.
void validate_launch_table(const LaunchConfig &table);

// Validates the table and the grade, then returns the stepwise cog, the
// fixed pedal angle and the interpolated lean for that grade.
[[nodiscard]] LaunchPreparation prepare_launch(const LaunchConfig &table,
                                               double grade);

// The release torque the table gives for one grade (same interpolation as
// the start lean). Validates the table first.
[[nodiscard]] double release_torque_nm(const LaunchConfig &table, double grade);

// The plan times on the intention period grid, at most 2^31 ticks each.
// The profile post_init (always) and RiderIntent (enabled intent) call it.
void validate_launch_plan_context(const LaunchConfig &table, double period_s);

} // namespace rider
