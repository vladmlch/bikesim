#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <mujoco/mujoco.h>
#include "../runtime/control.hpp"

namespace bike::legacy {
struct WheelSpin {
    std::string name;
    int dofadr=0;
    double radius_m=0.;
    [[nodiscard]] double omega_radps(std::span<const double> qvel) const;
};
[[nodiscard]] WheelSpin resolve_wheel_spin(const mjModel&,std::string_view joint_name,std::string_view contact_geom);
[[nodiscard]] int validate_actuator_target(const mjModel&,std::string_view name,std::string_view joint_name);
[[nodiscard]] std::pair<int,int> resolve_hinge(const mjModel&,std::string_view name);
[[nodiscard]] std::pair<int,int> resolve_scalar_joint(const mjModel&,std::string_view name);
struct DrivetrainSpecs {
    int chainring_teeth = 32, cog_teeth = 14;
    std::vector<int> cassette{10,12,14,16,18,21,24,28,33,39,45,51};
    bool auto_shift = true;
    double target_cadence_min_rpm = 65., target_cadence_max_rpm = 85.;
    double shift_cooldown_s = .40, shift_cut_duration_s = .20;
    double ripple_depth = .85, rider_torque_ceiling_nm = 60., rider_power_ceiling_w = 300.;
    double assist_torque_ceiling_nm = 85., assist_power_ceiling_w = 600., assist_response_s = .30;
    double cutoff_speed_kmh = 25., cutoff_taper_kmh = 2., crank_phase_deg = 0.;
    double crank_hold_stiffness_nm_rad = 30., crank_hold_damping_nms_rad = 3.;
    void validate() const;
    [[nodiscard]] double gear_ratio() const;
    [[nodiscard]] double cadence_rpm_at(double speed_mps, double rear_wheel_radius_m) const;
    [[nodiscard]] double support_factor(std::string_view mode) const;
};
[[nodiscard]] double peak_shape(double depth);
[[nodiscard]] double ripple_shape(double phase_rad, double depth);
[[nodiscard]] std::pair<double,std::string> limited_rider_torque(double demand_nm,
    double crank_radps, const DrivetrainSpecs&);
[[nodiscard]] double cutoff_factor(double speed_mps, const DrivetrainSpecs&);
[[nodiscard]] double assist_target_torque(double rider_nm, double support,
    double crank_radps, const DrivetrainSpecs&);
[[nodiscard]] double first_order_step(double current, double target, double dt_s, double tau_s);

struct CrankCommand {
    double crank_torque_nm = 0., rider_torque_nm = 0., assist_torque_nm = 0.;
    double phase_rad = 0., cadence_rpm = 0., rider_power_w = 0., motor_power_w = 0.;
    double support_factor = 0.;
    bool freewheel = false, cutoff_active = false;
    std::string rider_limit = "none";
    double brake_demand = 0.;
    int gear_teeth = 14;
};

// Borrowed only by one RideSimulation; the owning Stepper outlives this
// controller. Equality datum and ratio are model state, not local caches.
class PedalDrivetrain {
public:
    explicit PedalDrivetrain(mjModel&, DrivetrainSpecs = {}, std::string mode = "pedelec",
                    std::string assist = "tour", bool legs_drive = false);
    [[nodiscard]] const CrankCommand& compute(mjData&, double wheel_demand_nm,
        double speed_mps, bool rear_in_contact, bool traction_limited = false);
    void write(mjData&) const;
    void reset(mjData&, bool initialize_pose = true);
    void set_assist_mode(std::string_view);
    [[nodiscard]] std::string cycle_assist_mode();
    [[nodiscard]] double nominal_support_factor() const;
    [[nodiscard]] double current_gear_ratio() const;
    [[nodiscard]] double chain_residual(const mjData&) const;
    [[nodiscard]] const CrankCommand& command() const noexcept { return command_; }
    [[nodiscard]] const DrivetrainSpecs& specs() const noexcept { return specs_; }
    [[nodiscard]] std::size_t crank_qposadr() const noexcept { return crank_q_; }
private:
    void redatum(const mjData&);
    void engage(mjData&);
    void shift_to(mjData&, int cog);
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    mjModel& model_;
    DrivetrainSpecs specs_;
    std::string mode_, assist_;
    bool legs_drive_, engaged_ = true, traction_limited_ = false;
    std::size_t crank_q_, crank_v_, wheel_q_, wheel_v_, actuator_, equality_;
    int cog_;
    double crank_zero_, wheel_zero_, start_phase_, hold_phase_;
    double assist_nm_ = 0., cooldown_ = 0., cut_ = 0.;
    CrankCommand command_;
};

struct TerrainContacts {
    double front_load_n = 0., rear_load_n = 0.;
    double front_support_n = 0., rear_support_n = 0., handlebar_load_n = 0.;
    std::optional<TireSnapshot> front_snapshot,rear_snapshot;
    std::optional<bool> front_controller_grounded,rear_controller_grounded;
    [[nodiscard]] bool front_in_contact() const noexcept { return front_load_n > 1.; }
    [[nodiscard]] bool rear_in_contact() const noexcept { return rear_load_n > 1.; }
    [[nodiscard]] bool airborne() const noexcept { return !front_in_contact() && !rear_in_contact(); }
    [[nodiscard]] bool handlebar_in_contact() const noexcept { return handlebar_load_n>1.; }
};
class TyreForceApplier;
class TerrainContactQuery {
public:
    explicit TerrainContactQuery(const mjModel&, int dropout_steps = 10);
    [[nodiscard]] TerrainContacts query(const mjData&, const TyreForceApplier* = nullptr,
        std::optional<double> time_s={},std::span<const double> qvel={});
    [[nodiscard]] double handlebar_load(const mjData&) const;
    [[nodiscard]] runtime::RuntimeContactQuery::State state() const noexcept { return query_.state(); }
    void reset() noexcept;
private:
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    const mjModel& model_;
    runtime::RuntimeContactQuery query_;
};
[[nodiscard]] double opposing_torque(double magnitude_nm, double omega_radps, double taper_radps);
[[nodiscard]] double rolling_torque(double support_n, double omega_radps,
    double radius_m, double system_weight_n, double crr = .015,
    double taper_radps = .2, double load_ceiling_weights = 3.);

using CrashEvent=runtime::CrashEvent;
[[nodiscard]] std::optional<CrashEvent> detect_crash(double time_s, double x_m,
    double pitch_rad, const TerrainContacts&, std::optional<CrashEvent> previous = {},
    double limit_deg = 60.);
struct PitchState { double moment_nm = 0., angular_impulse_nms = 0., work_j = 0.; bool active = false; };
[[nodiscard]] PitchState pitch_step(PitchState previous, bool enabled, double pitch_rad,
    double pitch_rate_radps, const TerrainContacts&, double dt_s);

struct RunLimits {
    double finish_x_m{};
    std::int64_t max_steps{};
    double max_wall_clock_s = 300.;
    void validate() const;
    [[nodiscard]] static RunLimits for_track(double length_m, double start_x_m,
        double timestep_s, double max_wall_clock_s = 300., double min_speed_mps = 15. / 3.6);
};
[[nodiscard]] std::optional<std::string> termination_reason(const RunLimits&,
    double position_m, std::int64_t steps, double wall_clock_s,
    const std::optional<CrashEvent>&);
struct RunOutcome {
    std::string reason;
    std::int64_t steps;
    double sim_time_s, wall_clock_s, position_m;
    std::optional<CrashEvent> crash;
    [[nodiscard]] bool completed() const noexcept { return reason == "end of track"; }
};
} // namespace bike::legacy
