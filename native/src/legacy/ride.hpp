#pragma once

#include <array>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "controls.hpp"
#include "rider.hpp"
#include "tyre.hpp"
#include "../initialization/setup_owner.hpp"

namespace bike::legacy {
inline constexpr auto kLegacyChannels = std::to_array<std::string_view>({
    "time_s","x_m","speed_mps","fork_travel_mm","fork_shaft_mps","shock_stroke_mm",
    "rear_wheel_mm","shock_shaft_mps","fork_spring_n","fork_damper_n","shock_spring_n",
    "shock_bumper_n","shock_damper_n","pitch_rad","front_contact","rear_contact","front_load_n",
    "rear_load_n","drive_torque_nm","wheel_power_w","bar_acc_vert_mps2","bar_acc_long_mps2",
    "saddle_acc_vert_mps2","saddle_acc_long_mps2","rider_moment_nm","rider_impulse_nms","rider_work_j",
    "saddle_load_n","saddle_gap_m","bar_hand_load_n","pedal_load_front_n","pedal_load_rear_n",
    "rider_torso_acc_vert_mps2","rider_torso_acc_long_mps2","rider_pelvis_acc_vert_mps2",
    "rider_pelvis_acc_long_mps2","front_tyre_fz_n","front_tyre_fx_n","front_tyre_deflection_mm",
    "front_patch_length_mm","front_slip_ratio","front_tyre_full_sliding","front_rim_strike",
    "front_tyre_pressure_bar","front_tyre_loss_w","rear_tyre_fz_n","rear_tyre_fx_n","rear_tyre_deflection_mm",
    "rear_patch_length_mm","rear_slip_ratio","rear_tyre_full_sliding","rear_rim_strike","rear_tyre_pressure_bar",
    "rear_tyre_loss_w","crank_phase_rad","cadence_rpm","crank_torque_nm","assist_torque_nm","motor_power_w",
    "support_factor","freewheel","assist_cutoff_active","brake_source","drive_traction_limited","gear_teeth"});
using TelemetryRow = std::array<double,kLegacyChannels.size()>;
using Channels = std::vector<std::pair<std::string,std::vector<double>>>;
struct ForceContribution { std::string name; std::vector<double> force; };
class ForceAccumulator {
public:
    explicit ForceAccumulator(std::size_t width);
    void clear() noexcept;
    void add(std::string_view name, std::span<const double> force);
    [[nodiscard]] std::span<const double> total() const noexcept { return total_; }
    [[nodiscard]] std::vector<ForceContribution> snapshot() const { return components_; }
private:
    std::vector<double> total_;
    std::vector<ForceContribution> components_;
};

struct RideOptions {
    initialization::SetupRequest setup;
    TyreConfig tyre;
    DrivetrainSpecs drivetrain;
    std::string drive_mode = "motor", assist_mode = "tour";
    bool visual_pedalling = false;
    std::optional<std::string> legs;
};
struct RideSnapshot {
    double time_s = 0., position_m = 0., speed_mps = 0., pitch_rad = 0., z_m = 0.;
    std::int64_t steps = 0;
    std::uint64_t generation = 0;
    TerrainContacts contacts;
    std::optional<CrashEvent> crash;
    TelemetryRow telemetry{};
    std::vector<double> qpos,qvel,ctrl,qfrc_applied,xfrc_applied,eq_data,integration_state;
    std::vector<unsigned char> eq_active;
    std::array<std::vector<double>, 6> model_fields;
};
class RideSimulation {
public:
    ~RideSimulation();
    RideSimulation(const RideSimulation&) = delete;
    RideSimulation& operator=(const RideSimulation&) = delete;
    RideSimulation(RideSimulation&&) = delete;
    RideSimulation& operator=(RideSimulation&&) = delete;
    void step(double front_brake = 0., double rear_brake = 0.);
    void reset();
    void set_target_speed_kph(double);
    void set_assist_mode(std::string_view);
    void set_tyre_pressures(double front_bar,double rear_bar);
    // Applies a legacy UI tuning key between physics intervals. The returned
    // message describes the actual accepted value; nullopt means unbound.
    [[nodiscard]] std::optional<std::string> handle_tuning_key(int keycode);
    [[nodiscard]] RideSnapshot snapshot() const;
    [[nodiscard]] TelemetryRow telemetry() const;
    [[nodiscard]] double time_s() const;
    [[nodiscard]] double position_m() const;
    [[nodiscard]] double timestep_s() const;
    [[nodiscard]] double start_x_m() const;
    [[nodiscard]] double shock_bumper_engage_mm() const;
    [[nodiscard]] std::optional<CrashEvent> crash_event() const;
    [[nodiscard]] std::optional<std::array<biketyre::WheelOutputs,2>> tyre_outputs() const;
    [[nodiscard]] std::int64_t steps() const;
    [[nodiscard]] RunLimits default_limits(double max_wall_clock_s = 300.) const;
    [[nodiscard]] RunOutcome run(const RunLimits&, const std::function<void(const RideSimulation&)>& on_step = {},
        double front_brake = 0., double rear_brake = 0.);
    [[nodiscard]] runtime::ModelImage model_image() const;
    [[nodiscard]] io::Value metadata() const;
private:
    struct Impl;
    explicit RideSimulation(std::unique_ptr<Impl>);
    std::unique_ptr<Impl> impl_;
    friend std::unique_ptr<RideSimulation> make_ride(RideOptions);
};
[[nodiscard]] std::unique_ptr<RideSimulation> make_ride(RideOptions);
[[nodiscard]] std::unique_ptr<RideSimulation> make_ride(const std::filesystem::path& request);

// Explicit, opt-in legacy CSV recorder. No growing history exists in the
// RideSimulation itself; --record full recording remains physical-only.
class RideRecorder {
public:
    struct RimEvent {
        double x_m=0., speed_mps=0., peak_load_n=0., peak_rim_force_n=0., absorbed_energy_j=0.;
        std::optional<double> start_time_s, end_time_s;
    };
    struct TyreHistory {
        std::vector<RimEvent> events;
        std::vector<double> starts_m;
        double wheelspin_time_s=0., locked_time_s=0.;
    };
    explicit RideRecorder(double timestep_s, std::size_t decimate = 1);
    void record(const RideSimulation&);
    void record(const TelemetryRow&);
    [[nodiscard]] std::size_t rows() const noexcept { return rows_.size(); }
    [[nodiscard]] double sample_interval_s() const noexcept;
    [[nodiscard]] Channels columns() const;
    [[nodiscard]] std::vector<double> column(std::string_view) const;
    [[nodiscard]] const std::array<TyreHistory,2>& tyre_history() const noexcept { return tyres_; }
    [[nodiscard]] std::string to_csv() const;
    void write_csv(const std::filesystem::path&) const;
private:
    double timestep_;
    std::size_t decimate_, calls_ = 0;
    std::vector<TelemetryRow> rows_;
    std::array<TyreHistory,2> tyres_;
    std::array<bool,2> rim_active_{};
    std::array<double,2> rim_start_time_{};
};
} // namespace bike::legacy
