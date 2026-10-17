#pragma once

#include "ride.hpp"
#include "../io/value.hpp"
#include "../terrain/types.hpp"

namespace bike::legacy {
inline constexpr double kRampExclusionM=8., kAccelFilterHz=100., kGravityMps2=9.81;
inline constexpr double kForkBottomMarginMm=5., kTopOutToleranceMm=1.;
struct TravelStats {
    std::string end;
    double travel_limit_mm=0.,max_mm=0.,max_pct=0.,p95_mm=0.,mean_mm=0.,bottom_out_threshold_mm=0.;
    std::size_t bottom_outs=0,top_outs=0;
    double max_compression_mps=0.,max_rebound_mps=0.;
};
struct AccelStats {
    std::string point;
    double filter_hz=kAccelFilterHz,rms_filtered_mps2=0.,peak_filtered_mps2=0.,peak_raw_mps2=0.;
    std::string raw_peak_note="raw peak includes solver contact transients (1-2 ms); compare filtered values";
};
struct RiderStats {
    AccelStats torso,pelvis;
    std::size_t saddle_lift_events=0;
    double saddle_lift_time_s=0.,saddle_gap_max_mm=0.,mean_saddle_share=0.,mean_pedal_share=0.,mean_bar_share=0.;
    double min_saddle_load_n=0.,max_saddle_load_n=0.;
};
struct PotholeReport {
    std::string label,kind;
    double start_m=0.,length_m=0.,declared_depth_mm=0.,effective_drop_front_mm=0.,effective_drop_rear_mm=0.;
};
using RimStrikeStats=RideRecorder::RimEvent;
struct TyreStats {
    std::string wheel;
    double peak_fz_n=0.,max_deflection_mm=0.;
    std::size_t rim_strikes=0;
    std::vector<RimStrikeStats> rim_strike_events;
    double wheelspin_time_s=0.,locked_time_s=0.,mean_dissipated_power_w=0.,crr_equivalent=0.;
    std::size_t completed_rim_impact_episodes=0;
    double rim_episode_gap_s=.001,rim_episode_distance_m=.01;
};
struct PedalBobStats {
    double mean_cadence_rpm=0.,bob_frequency_hz=0.,shock_bob_amplitude_mm=0.,fork_bob_amplitude_mm=0.;
    double crank_torque_mean_nm=0.,crank_torque_peak_nm=0.,assist_torque_mean_nm=0.,wheel_torque_mean_nm=0.;
    double gear_ratio_check=0.,freewheel_fraction=0.,cutoff_fraction=0.,traction_limited_fraction=0.;
    int assist_switches=0;
};
struct SummaryOptions {
    double target_speed_kmh=25.,fork_travel_mm=180.,shock_stroke_mm=65.,shock_bumper_engage_mm=55.,start_x_m=5.;
    std::string rider_variant="lumped";
    double rider_mass_kg=80.;
    io::Value::Object extras;
    std::optional<std::array<RideRecorder::TyreHistory,2>> tyre_history;
    int assist_switches=0;
};
struct RideSummary {
    std::string track;
    double track_length_m=0.;
    std::size_t n_potholes=0,n_bumps=0,n_obstacles=0;
    double target_speed_kmh=0.;
    std::string outcome;
    bool completed=false;
    std::optional<std::string> crash;
    std::int64_t steps=0;
    double sim_time_s=0.,wall_clock_s=0.,sample_interval_s=0.,window_start_m=0.;
    std::size_t window_rows=0;
    double mean_speed_kmh=0.;
    std::size_t airborne_events=0;
    double airborne_time_s=0.;
    TravelStats fork,shock;
    AccelStats bar,saddle;
    std::vector<PotholeReport> potholes;
    io::Value::Object extras;
    std::string rider_variant="lumped";
    std::optional<RiderStats> rider;
    std::array<TyreStats,2> tyres;
    std::optional<PedalBobStats> pedals;
    [[nodiscard]] io::Value to_value() const;
    [[nodiscard]] std::string to_json() const;
    void write_json(const std::filesystem::path&) const;
    [[nodiscard]] std::string format_table() const;
};
[[nodiscard]] std::size_t count_events(std::span<const unsigned char> flags);
[[nodiscard]] TravelStats travel_stats(std::string end,std::span<const double> travel_mm,
    std::span<const double> shaft_mps,double limit_mm,double bottom_threshold_mm);
[[nodiscard]] AccelStats accel_stats(std::string point,std::span<const double> acc_vert_mps2,double dt_s);
[[nodiscard]] RiderStats rider_stats(const Channels& window,double dt_s,double rider_weight_n);
[[nodiscard]] double sine_amplitude(std::span<const double> signal,double frequency_hz,double dt_s);
[[nodiscard]] std::optional<PedalBobStats> pedal_bob_stats(const Channels& window,double dt_s,int assist_switches=0);
[[nodiscard]] std::vector<PotholeReport> pothole_reports(const terrain::TrackSpec&);
[[nodiscard]] std::array<std::size_t,3> count_kinds(std::span<const terrain::Obstacle>);
[[nodiscard]] std::size_t count_completed_impact_episodes(std::span<const RimStrikeStats>,
    double gap_s=.001,double distance_m=.01);
[[nodiscard]] RideSummary summarize_ride(const Channels&,double dt_s,const terrain::TrackSpec&,
    const RunOutcome&,const SummaryOptions&);
} // namespace bike::legacy
