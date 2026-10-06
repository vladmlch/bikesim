#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace drivetrain {
    struct GearingConfig {
        int front_teeth{}, rear_teeth{};
        double chain_pitch_m{};
    };

    struct PedalingConfig {
        bool enabled{};
        double coast_above_rpm{}, resume_below_rpm{}, stop_time_s{}, coast_cadence_tau_s{};
        double mash_cadence_rpm{}, mash_torque_nm{}, effort_slew_nm_s{};
    };

    struct ShiftingConfig {
        bool enabled{};
        std::vector<int> cassette;
        double target_cadence_min_rpm{}, target_cadence_max_rpm{}, shift_cooldown_s{};
        double shift_cut_duration_s{}, torque_factor{}, cadence_smoothing_tau_s{}, upshift_slip_limit_mps{};
        std::string upshift_slip_mode;
    };

    struct MotorProfile {
        double eco{}, tour{}, emtb_low{}, emtb_high{}, turbo{}, emtb_full_gain_at_nm{};
    };

    struct AssistConfig {
        double gain{}, max_torque{}, max_power{}, tau{}, slew{}, engage_torque_nm{};
        double gate_min_crank_rad_s{}, cutoff_mps{}, taper_width_mps{};
        std::optional<std::vector<std::array<double, 2> > > torque_curve;
        std::optional<MotorProfile> profile;
        std::string mode;
    };

    struct BatteryConfig {
        bool enabled{};
        double energy_j{}, copper_w_per_nm2{}, speed_w_per_rad_s2{}, idle_w{};
    };

    struct DrivePolicyConfig {
        GearingConfig gearing;
        PedalingConfig pedaling;
        ShiftingConfig shifting;
        AssistConfig assist;
        BatteryConfig battery;
        double hub_stiffness_nm_rad{}, hub_damping_nm_s{};
    };

    struct PedalingSnapshot {
        bool coasting{};
        std::optional<double> target_phase_rad;
        double target_rate_rad_s{}, deceleration_rad_s2{}, effort{};
        std::optional<double> cadence_ema;
    };

    struct ShiftingSnapshot {
        int rear_teeth{}, from_teeth{}, shift_count{};
        double cooldown_s{}, cut_remaining_s{};
        std::string direction = "none";
        std::optional<double> cadence_ema, required_ema;
    };

    struct AssistSnapshot {
        double torque{}, last_gain{};
        bool pedaling{};
    };

    struct BatterySnapshot {
        double initial_energy_j{}, energy_j{}, drawn_energy_j{};
    };

    struct FreehubSnapshot {
        std::optional<double> boundary;
        double energy_j{}, torque_nm{};
    };
    struct DriveConfig {
        DrivePolicyConfig policies;
        std::string drive_mode, transmission_model;
        double human_torque_nm{}, torque_ripple{}, crank_phase_rad{}, chain_k_n_m{},
                chain_c_ns_m{}, bearing_c_nms_rad{}, rotor_inertia_kgm2{};
        bool motor_clutch{};
    };

}

namespace nativecfg {
    // --- typed configs (mirror the Python objects' used attributes) -----------

    // AirSpringSpecs fields the force path reads; the "properties" are the same
    // expressions as physics/air_spring.py so derived values are bitwise-equal.
    struct AirSpringSpec {
        double stanchion_inner_diam_mm;
        double total_travel_mm;
        double pos_chamber_length_mm;
        double neg_chamber_length_mm;
        double token_volume_cm3;
        int max_tokens;
        double gamma;
        double atm_pressure_pa;
    };

    struct AirSpringConfig {
        AirSpringSpec specs;
        int num_tokens;
        double gauge_pressure_psi;
    };

    // BaseDamper resolved fields. `total_travel_mm` is NOT here: Charger 3 reads
    // it, Super Deluxe reads `total_stroke_mm` instead — each subclass section
    // carries the name its methods use, like the Python classes do.
    struct DamperCore {
        int max_hsc, max_lsc, max_reb;
        int hsc_clicks, lsc_clicks, rebound_clicks; // ctor-clamped below
        double c_lsc_min, c_lsc_max;
        double c_hsc_min, c_hsc_max;
        double c_reb_min, c_reb_max;
        double v_knee_comp, v_knee_reb;
    };

    struct Charger3Config {
        DamperCore core;
        double total_travel_mm;
        double hbo_start_mm;
        double c_hbo_base;
        bool legacy_behavior{};
    };

    struct SuperDeluxeConfig {
        DamperCore core;
        double total_stroke_mm;
        int max_hbo, hbo_clicks;
        bool lockout_firm;
        bool legacy_behavior;
        double hbo_start_mm;
        double c_hbo_min, c_hbo_max;
        double lockout_preload_n, lockout_stiffness;
    };

    struct CoilConfig {
        double rate_n_m, preload_mm, stroke_mm;
        double bumper_length_mm, bumper_peak_n;
        bool legacy_behavior;

        [[nodiscard]] double bumper_engage_mm() const {
            return stroke_mm - bumper_length_mm;
        }
    };

    struct EndStopConfig {
        double stiffness_n_m, damping_n_s_m;
    };

    struct SuspensionConfig {
        std::string physics_mode;
        std::string fork_joint, shock_joint;
        AirSpringConfig air_spring{};
        Charger3Config fork_damper{};
        SuperDeluxeConfig shock_damper{};
        CoilConfig coil{};
        EndStopConfig end_stops{};
    };

    // BrakeController fields (braking.py:39-65): ceiling and taper band. The
    // wheel/actuator names it resolves are literals in the Python source, so
    // they live in the writer, not the config.
    struct BrakeConfig {
        double torque_ceiling_nm;
        double taper_radps;
    };

    struct CruiseConfig {
        double target_speed_kmh;
        double kp_nm_per_mps;
        double ki_nm_per_mps_s;
        double torque_ceiling_nm;
    };

    // ResistanceConfig fields ExternalResistanceApplier reads (physical_config.
    // py:266-284), plus the body names its __init__ resolves — literals there
    // too, emitted so a differently-named model stays describable.
    struct ResistanceConfig {
        double crr{};
        double rolling_taper_rad_s{};
        double rho_kg_m3{};
        double cda_m2{};
        std::array<double, 3> wind_world_mps{};
        std::array<double, 3> point_body_m{};
        std::string frame_body, front_wheel_body, rear_wheel_body;
    };

    // TireSpec fields (physics/tire.py:31-50) — the linear radial material the
    // compliant_2d path reads, plus the load-range declaration its validity flag
    // consumes. `provenance`/`pressure_pa_gauge` ride along for completeness; the
    // writer only validates them the way TireSpec.__post_init__ does.
    struct TireMaterial {
        double radial_k_n_m{}, radial_c_ns_m{}, pressure_pa_gauge{};
        std::string provenance;
        std::array<double, 2> valid_load_range_n{};
    };

    // TireParameters (physical_config.py:23-35).
    struct TireParams {
        TireMaterial material;
        double tangent_k_n_m{}, mu{}, relaxation_length_m{};
    };

    // SurfaceSpec (terrain/surface.py:31-88) with the registry name already
    // resolved — the writer only ever calls mu(slip) and reads .name.
    struct SurfaceSpec {
        std::string name;
        double mu_peak{}, mu_slide{}, slip_stiffness_per_load{}, stribeck_speed_mps{};
    };

    // SurfaceSection — a half-open material interval on the track axis.
    struct SurfaceSection {
        double start_m{}, end_m{};
        SurfaceSpec surface;
    };

    // SurfaceMap (terrain/surface.py:160-206): a default material plus sorted
    // non-overlapping sections; `at` scans sections then falls back to default.
    struct SurfaceMap {
        SurfaceSpec surface;
        std::vector<SurfaceSection> sections;
    };

    // TireBackendConfig (physical_config.py:38-63), restricted to the backend the
    // native port implements: 'compliant_2d'.
    struct TireConfig {
        std::string backend;
        std::string surface_mode; // 'configured' | 'track'
        TireParams front, rear;
        double significant_delta_m{}, significance_fraction{}, distinct_normal_deg{};
        std::optional<SurfaceMap> surface_map;
    };

    // RiderForceApplier per-path state (rider_forces.py:35-76): the body's
    // resolved spring params plus the construction-time pedal offset
    // (set_pedal_offsets is applied before project() runs — the config carries
    // the effective offset_m, not the crank derivation). `joint` re-resolves to
    // qposadr/dofadr through _resolve_slide's checks.
    struct RiderPathConfig {
        std::string joint;
        double stiffness_n_m, damping_ns_m;
        double preload_deflection_m, offset_m;
        bool unilateral;
    };

    struct RiderForcesConfig {
        std::vector<RiderPathConfig> paths; // _paths order
    };

    struct NativeConfig {
        std::optional<drivetrain::DriveConfig> drive;
        std::optional<CruiseConfig> cruise;
        std::optional<SuspensionConfig> suspension;
        std::optional<BrakeConfig> brake;
        std::optional<ResistanceConfig> resistance;
        std::optional<TireConfig> tire;
        std::optional<RiderForcesConfig> rider_forces;
    };

}
