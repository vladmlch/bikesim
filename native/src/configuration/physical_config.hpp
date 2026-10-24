// configuration/physical_config.hpp — T12 port of the immutable SI
// parameter records reachable from physical_config.py plus the nested spec
// records its fields type against (tire.py, tire_curve.py, tyre.py,
// chain.py, motor_profile.py, distributed_tire.py, attachment_budget.py).
//
// Member default-initializer lists transcribe the Python declared defaults;
// members with no Python default carry no initializer (the aggregate forces
// them at construction, like the dataclass arity). Every record is
// standard-layout so `detail::kXxxSchema` tables use offsetof().
//
// `detail::post_init(T&)` overloads (validate.cpp) are the __post_init__
// ports: they normalize in place (cassette sort, 'weld'->'connect' grip
// alias) and throw std::invalid_argument carrying the Python message.
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "fields.hpp"

namespace bike::configuration {

// ---- tire.py TireSpec ------------------------------------------------------
// All five members are required in Python; brace-initialize at construction.
struct TireSpec {
    double radial_k_n_m;
    double radial_c_ns_m;
    double pressure_pa_gauge;
    std::string provenance;
    std::array<double, 2> valid_load_range_n;
};

// tire.py _material(): the synthetic default behind TireParameters.material.
inline TireSpec synthetic_tire_material() {
    return TireSpec{.radial_k_n_m=130000., .radial_c_ns_m=800., .pressure_pa_gauge=200000., .provenance="synthetic", .valid_load_range_n={0., 2000.}};
}

// ---- tire_curve.py TabulatedTireSpec ----------------------------------------
struct TabulatedTireSpec {
    std::vector<double> deflection_m;
    std::vector<double> force_n;
    double radial_c_ns_m;
    double pressure_pa_gauge;
    std::string provenance;
    std::array<double, 2> valid_load_range_n;
};

// ---- distributed_tire.py ----------------------------------------------------
struct HingeDensity {
    std::vector<double> knots_m{0., .004, .012};
    std::vector<double> stiffness_n_m{300000., 0., 0.};
    double damping_ns_m = 2000.;
    std::string provenance = "synthetic density; not fitted or measured";
};

struct DistributedWheelMaterial {
    HingeDensity density;
    std::string fitting_dataset_id = "none-synthetic";
    std::array<double, 2> valid_load_range_n{0., 2000.};
    std::array<double, 2> valid_deflection_range_m{0., .04};
    std::string calibration_status = "experimental_unvalidated";
};

struct DistributedTireConfig {
    int station_count = 128;
    HingeDensity density;
    std::string fitting_dataset_id = "none-synthetic";
    std::array<double, 2> valid_load_range_n{0., 2000.};
    std::array<double, 2> valid_deflection_range_m{0., .04};
    std::string calibration_status = "experimental_unvalidated";
    std::optional<DistributedWheelMaterial> front_material;
    std::optional<DistributedWheelMaterial> rear_material;

    // distributed_tire.py for_wheel(side): 'front'/'rear' selects the
    // per-wheel material or builds the shared-law fallback (validated like
    // the Python construction); any other side raises
    // 'wheel side must be front or rear'.
    [[nodiscard]] DistributedWheelMaterial for_wheel(std::string_view side) const;
};

// ---- tyre.py -----------------------------------------------------------------
struct TyreSpecs {
    // First six members are required in Python (no declared defaults).
    std::string name;
    double outer_radius_mm;
    double rim_radius_mm;
    double section_height_mm;
    double pressure_bar;
    double crr_reference;
    double casing_width_mm = 60.;
    double compressed_casing_mm = 6.;
    double area_factor = .46;
    double carcass_stiffness_n_mm2 = .1;
    double contact_length_factor = .86;
    double rate_stiffening = .25;
    double rate_relaxation_s = .2;
    double loss_factor = .07;
    double hysteresis_rate_eps_mps = .01;
    double relaxation_length_mm = 60.;
    double rim_stiffness_n_mm2 = 28.;
    double tread_loss_crr = 0.;

    // Derived properties (tyre.py): geometry-only, no state.
    [[nodiscard]] double tyre_height_mm() const { return outer_radius_mm - rim_radius_mm; }
    [[nodiscard]] double rim_strike_deflection_mm() const { return tyre_height_mm() - compressed_casing_mm; }
    [[nodiscard]] double pressure_pa() const { return pressure_bar * 100000.0; }
    [[nodiscard]] TyreSpecs with_pressure(double pressure_bar_) const;
};

// tyre.py module constants (Plan D5 stock tyres).
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const TyreSpecs kFrontTyre{
    .name="Schwalbe Magic Mary 29x2.4", .outer_radius_mm=372., .rim_radius_mm=320., .section_height_mm=57., .pressure_bar=1.5, .crr_reference=.011,
    .casing_width_mm=60., .compressed_casing_mm=6., .area_factor=.4435, .carcass_stiffness_n_mm2=.1194, .contact_length_factor=.8589, .rate_stiffening=.2484, .rate_relaxation_s=.2, .loss_factor=.0662, .hysteresis_rate_eps_mps=.01, .relaxation_length_mm=60., .rim_stiffness_n_mm2=28., .tread_loss_crr=.00586};
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const TyreSpecs kRearTyre{
    .name="Schwalbe Hans Dampf 27.5x2.4", .outer_radius_mm=352., .rim_radius_mm=300., .section_height_mm=55., .pressure_bar=1.7, .crr_reference=.0103,
    .casing_width_mm=60., .compressed_casing_mm=6., .area_factor=.4576, .carcass_stiffness_n_mm2=.1225, .contact_length_factor=.8801, .rate_stiffening=.2498, .rate_relaxation_s=.2, .loss_factor=.069, .hysteresis_rate_eps_mps=.01, .relaxation_length_mm=60., .rim_stiffness_n_mm2=28., .tread_loss_crr=.00481};

struct TierSpec {
    std::string_view name;
    int n_rays;
    double half_angle_deg;
    double timestep_s;
    bool discretised_brush;
};

// tyre.py TIERS, in dict insertion order ('fast', 'detailed').
inline constexpr std::array<TierSpec, 2> kTireTiers{{
    {.name="fast", .n_rays=64, .half_angle_deg=75.0, .timestep_s=0.0005, .discretised_brush=false},
    {.name="detailed", .n_rays=256, .half_angle_deg=75.0, .timestep_s=0.00025, .discretised_brush=true},
}};
[[nodiscard]] const TierSpec* find_tire_tier(std::string_view name);

// W28 coefficient provenance is independent from execution qualification.
// A digest identifies the authored coefficient content, not measured accuracy.
struct SpatialTyreCoefficientProfile {
    std::string id;
    std::string status = "provisional";
    std::uint64_t source_measurements = 0;
    double kx_n_per_m3 = 5e6, ky_n_per_m3 = 5e6;
    double relaxation_x_m = .3, relaxation_y_m = .3;
    double camber_coefficient_per_rad = .8, pneumatic_trail_m = .02;
};
[[nodiscard]] SpatialTyreCoefficientProfile spatial_tyre_coefficient_profile(std::string_view id);
void validate_spatial_tyre_profile(const SpatialTyreCoefficientProfile&);
[[nodiscard]] bike::io::Value spatial_tyre_profile_registry();

// ---- chain.py DrivetrainSpecs (the mid-drive gearing record) -----------------
struct DrivetrainSpecs {
    int front_teeth = 34;
    int rear_teeth = 24;
    double chain_pitch_m = .0127;

    [[nodiscard]] double front_radius_m() const;
    [[nodiscard]] double rear_radius_m() const;
};

// ---- motor_profile.py --------------------------------------------------------
// mode_gains 'emtb' is a (low, high) tuple; the rest are scalars.
struct MotorModeGains {
    double eco;
    double tour;
    std::array<double, 2> emtb;  // (low, high)
    double turbo;
};

struct MotorProfile {
    std::string name;
    double peak_torque_nm;
    double rated_power_w;
    double peak_power_w;
    double torque_tau_s;
    MotorModeGains mode_gains;
    double emtb_full_gain_at_nm;
    double cadence_support_max_rpm;
    double cutoff_mps;
    double taper_width_mps;
    double gate_min_crank_rad_s;
    std::string provenance;
};

// motor_profile.py PROFILES registry ({'bosch_cx_gen4': BOSCH_CX_GEN4}).
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const MotorProfile kBoschCxGen4{
    .name="bosch_performance_line_cx_gen4", .peak_torque_nm=85., .rated_power_w=250., .peak_power_w=600., .torque_tau_s=.04,
    .mode_gains={.eco=.6, .tour=1.4, .emtb={1.4, 3.4}, .turbo=3.4}, .emtb_full_gain_at_nm=40., .cadence_support_max_rpm=120., .cutoff_mps=25. / 3.6, .taper_width_mps=2. / 3.6,
    .gate_min_crank_rad_s=0.08726646259971647 /* radians(5) */,
    .provenance="Bosch Performance Line CX Gen 4 (no ABS); recalled from memory "
    "2026-10-04, unverified, no web lookup by user instruction"};
[[nodiscard]] const MotorProfile* find_motor_profile(std::string_view name);

// ---- physical_config.py ------------------------------------------------------
struct TireParameters {
    std::variant<TireSpec, TabulatedTireSpec> material = synthetic_tire_material();
    double tangent_k_n_m = 20000.;
    double mu = .8;
    double relaxation_length_m = .2;
};

struct PneumaticTireConfig {
    std::string tier = "fast";
    TyreSpecs front = kFrontTyre;
    TyreSpecs rear = kRearTyre;
    std::optional<std::string> surface;
};

struct TireBackendConfig {
    std::string backend = "native_reference";
    TireParameters front;
    TireParameters rear;
    double significant_delta_m = .0001;
    double significance_fraction = .05;
    double distinct_normal_deg = 20.;
    std::string surface_mode = "configured";
    DistributedTireConfig distributed;
    PneumaticTireConfig pneumatic;
};

struct AssistConfig {
    // A named profile owns torque/power/lag/cutoff/taper/gate; explicit
    // values describe a synthetic motor when no profile is selected.
    std::optional<std::string> profile;
    std::string mode = "turbo";
    double gain = 2.;
    double max_torque = 80.;
    double max_power = 500.;
    double tau = .05;
    double slew = 400.;
    double engage_torque_nm = 4.;
    double gate_min_crank_rad_s = 0.08726646259971647;  // radians(5.)
    double cutoff_mps = 25. / 3.6;
    double taper_width_mps = 2. / 3.6;
    std::optional<std::vector<std::array<double, 2>>> torque_curve;

    // AssistConfig.effective_max_torque: profile peak or the declared value.
    [[nodiscard]] double effective_max_torque() const;
};

struct BatteryConfig {
    bool enabled = true;
    double energy_j = 1800000.;
    double copper_w_per_nm2 = .02;
    double speed_w_per_rad_s2 = 0.;
    double idle_w = 5.;
};

struct PedalingConfig {
    bool enabled = false;
    double coast_above_rpm = 110.;
    double resume_below_rpm = 90.;
    double stop_time_s = .35;
    // Coast on a sustained cadence rise, filtering the two strokes per turn.
    double coast_cadence_tau_s = 0.;
    // Low-cadence effort ramp ceiling inputs.
    double mash_cadence_rpm = 45.;
    double mash_torque_nm = 60.;
    double effort_slew_nm_s = 300.;
    // Hill-hold reflex: rollback grabs the wheel brakes.
    bool rollback_brake = true;
    double rollback_engage_mps = .25;
    double rollback_release_mps = 0.;
    double rollback_demand = 1.;
};

// drivetrain.py CASSETTE_12S_TEETH.
inline constexpr auto kCassette12sTeeth = std::to_array<int>({10, 12, 14, 16, 18, 21,
                                            24, 28, 33, 39, 45, 51});

struct ShiftingConfig {
    bool enabled = false;
    std::vector<int> cassette{std::begin(kCassette12sTeeth),
                              std::end(kCassette12sTeeth)};
    double target_cadence_min_rpm = 65.;
    double target_cadence_max_rpm = 85.;
    double shift_cooldown_s = .4;
    double shift_cut_duration_s = .2;
    double torque_factor = .3;
    double cadence_smoothing_tau_s = .35;
    double upshift_slip_limit_mps = .5;
    std::string upshift_slip_mode = "legacy_signed";
};

struct PhysicalDriveConfig {
    DrivetrainSpecs gearing;
    std::string transmission_model = "elastic_chain";
    double human_torque_nm = 0.;
    double torque_ripple = .35;
    double crank_phase_rad = 0.;
    double chain_k_n_m = 200000.;
    double chain_c_ns_m = 10.;
    double freehub_k_nm_rad = 1000.;
    double freehub_c_nms_rad = .5;
    double bearing_c_nms_rad = .03;
    double brake_ceiling_nm = 200.;
    // Held-brake stiction at standstill (pad bite) — physical_config.py's
    // same-named block: a zero torque or stiffness disables it.
    double brake_hold_torque_nm = 200.;
    double brake_hold_stiffness_nm_rad = 20000.;
    double brake_hold_damping_nms_rad = 10.;
    double brake_hold_engage_radps = .5;
    double brake_hold_release_radps = 1.;
    // Legacy regression topology: crank -[one-way]-> drive_shaft with motor.
    bool motor_clutch = false;
    // Motor inertia reflected to the crank coordinate, kg.m^2.
    double rotor_inertia_kgm2 = 0.;
    AssistConfig assist;
    BatteryConfig battery;
    PedalingConfig pedaling;
    ShiftingConfig shifting;
};

struct ResistanceConfig {
    double crr = .015;
    double rolling_taper_rad_s = .2;
    double rho_kg_m3 = 1.2;
    double cda_m2 = .5;
    std::array<double, 3> wind_world_mps{0., 0., 0.};
    std::array<double, 3> point_body_m{0., 0., .5};
};

struct ArticulatedConfig;
namespace detail {
// __post_init__ bodies (validate.cpp): normalize + domain-check in place,
// throwing std::invalid_argument with the Python messages.
void post_init(TireSpec&);
void post_init(TabulatedTireSpec&);
void post_init(const HingeDensity&);
void post_init(DistributedWheelMaterial&);
void post_init(DistributedTireConfig&);
void post_init(const TyreSpecs&);
void post_init(TireParameters&);
void post_init(PneumaticTireConfig&);
void post_init(TireBackendConfig&);
void post_init(const DrivetrainSpecs&);
void post_init(const MotorProfile&);
void post_init(AssistConfig&);
void post_init(const BatteryConfig&);
void post_init(const PedalingConfig&);
void post_init(ShiftingConfig&);
void post_init(PhysicalDriveConfig&);
void post_init(ResistanceConfig&);
void post_init(ResistanceConfig&, bool spatial_mode);
void post_init(ArticulatedConfig&);
} // namespace detail

// ---- attachment_budget.py AttachmentBudget ------------------------------------
struct AttachmentBudget {
    double foot_min_normal_n;
    double foot_mu;
    double saddle_mu;
    double grip_pull_n;
    double max_gap_m;
};

struct ArticulatedConfig {
    // Desired support distribution belongs to posture control, never anatomy.
    double pedal_support_fraction = .33;
    double bar_support_fraction = .12;
    double posture_sole_depth_m = .003;
    double swing_clearance_m = .003;  // unstrapped return foot is not a brake
    double coasting_brake_d_nm_s_rad = 2.;
    double coasting_brake_limit_nm = 60.;
    double stance_blend_load_n = 50.;
    double posture_pitch_k_nm_rad = 600.;
    double posture_pitch_d_nms_rad = 60.;
    double posture_pitch_limit_nm = 60.;
    // Synthetic internal support feedback, not a force on the floating root.
    double posture_translation_k_n_m = 1500.;
    double posture_translation_d_ns_m = 150.;
    double posture_translation_limit_n = 400.;
    double saddle_patch_half_length_m = .045;
    double pedal_patch_half_length_m = .025;
    double support_pad_radius_m = .020;  // synthetic sole contact shape
    double support_k_n_m = 100000.;
    double support_c_ns_m = 500.;  // saddle interface
    // Pedal sole material kept separate: reusing saddle damping made the
    // explicit edge contact unstable at 0.5 ms.
    double pedal_c_ns_m = 100.;
    double support_tangent_k_n_m = 20000.;
    double support_mu = .8;
    double support_length_m = .1;
    double grip_k_n_m = 4000.;
    double grip_c_ns_m = 150.;
    double grip_release_distance_m = .12;
    std::optional<double> grip_pair_force_limit_n;
    double grip_capture_distance_m = .02;
    double grip_capture_speed_mps = .2;
    double arm_reach_fraction = .92;
    double joint_kp_nm_rad = 600.;
    double joint_kd_nms_rad = 15.;
    // Passive tissue damping of the rider hinges; nullopt keeps the legacy
    // coupling to the controller derivative gain joint_kd_nms_rad.
    std::optional<double> joint_passive_damping_nms_rad;
    // Fully coupled rider (pelvis, soles and hands welded).
    bool coupled_task_control = false;
    double joint_limit_nm = 100.;
    double joint_speed_limit_rad_s = 20.;
    double joint_power_limit_w = 250.;
    std::optional<std::string> joint_envelope_path;
    double joint_envelope_soft_margin_rad = .1;
    double joint_envelope_soft_k_nm_rad = 100.;
    double activation_tau_s = 0.;
    std::optional<double> active_positive_power_limit_w;
    // Reference seated plant budgets (spec S3).
    double foot_mu = .9;
    double saddle_mu = .6;
    double pedal_min_normal_n = 20.;
    double saddle_reserve_weight_fraction = .15;
    double grip_pull_per_hand_n = 300.;
    // Seated pedaling-cycle strategy knobs (R5): ankle rock and return-foot
    // scrape are intents, never physical guarantees.
    double pedal_ankle_amplitude_rad = .1;
    double pedal_scrape_fraction = .5;
    // Two-leg waveform: mean*(1+ripple*cos(2*phase)), an engineering choice.
    double pedal_torque_ripple = .35;
    // Residual recovery-leg load on the rising flat pedal; zero disables it.
    double return_foot_preload_n = 0.;
    // Weld-press servo gain/hold limit and weld-imposed leg-velocity follow
    // damping — physical_config.py's same-named block. Declared and validated
    // here so the mirror stays complete (schema()/to_value()/contract), but
    // the native rider controller has not ported those knobs: the closed
    // config.rider_controller wire drops them in project_rider_controller.
    double weld_servo_gain = .05;
    double weld_servo_limit_nm = 25.;
    double leg_track_damping_nms_rad = 0.;
    double link_max_gap_m = .005;
    // Bounded road preview the rider planner may see ahead of the front wheel.
    double road_lookahead_m = 0.;
    std::optional<std::string> joint_strength_path;
    std::string pedal_attachment = "flat";
    std::string saddle_attachment = "flat";
    // 'spring' is the releasable compliant grip; 'connect' pins the hands.
    // 'weld' is a legacy alias normalized to 'connect' by post_init.
    std::string grip_attachment = "spring";
    double balance_floor_kmh = 4.;
    double balance_dwell_s = .5;
    double balance_grace_s = 3.;

    // Spatial-only source fields are projected separately from the frozen
    // planar dataclass field table, keeping its bytes and schema unchanged.
    std::optional<std::string> shoulder_profile;
    std::optional<std::string> shoulder_profile_sha256;

    [[nodiscard]] AttachmentBudget attachment_budget() const {
        return {.foot_min_normal_n=pedal_min_normal_n, .foot_mu=foot_mu, .saddle_mu=saddle_mu,
                .grip_pull_n=grip_pull_per_hand_n, .max_gap_m=link_max_gap_m};
    }
    // ArticulatedConfig.passive_damping_nms_rad (resolved hinge damping).
    [[nodiscard]] double passive_damping_nms_rad() const {
        return joint_passive_damping_nms_rad.value_or(joint_kd_nms_rad);
    }
};

// ===========================================================================
// Field tables — one FieldDecl per dataclass field in declaration order.
// `type` carries str(f.type) verbatim; `factory` the default_factory
// __qualname__. These tables feed schema(), to_value() and the T13 loader —
// the single shared declaration for the configuration boundary.
// ===========================================================================
namespace detail {

#define BIKE_FIELD(T, f, type_, kind_, req_, fac_, n1_, n2_)               \
    ::bike::configuration::FieldDecl {                                     \
        #f, type_, ::bike::configuration::FieldKind::kind_,                \
        offsetof(T, f), req_, fac_, n1_, n2_                               \
    }
#define REAL(T, f)    BIKE_FIELD(T, f, "<class 'float'>", Real, false, "", nullptr, nullptr)
#define REQ_REAL(T, f) BIKE_FIELD(T, f, "<class 'float'>", Real, true, "", nullptr, nullptr)
#define INT(T, f)     BIKE_FIELD(T, f, "<class 'int'>", Integer, false, "", nullptr, nullptr)
#define REQ_INT(T, f) BIKE_FIELD(T, f, "<class 'int'>", Integer, true, "", nullptr, nullptr)
#define BOOL(T, f)    BIKE_FIELD(T, f, "<class 'bool'>", Boolean, false, "", nullptr, nullptr)
#define STR(T, f)     BIKE_FIELD(T, f, "<class 'str'>", Text, false, "", nullptr, nullptr)
#define REQ_STR(T, f) BIKE_FIELD(T, f, "<class 'str'>", Text, true, "", nullptr, nullptr)

// ---- leaf record schemas -----------------------------------------------------
// TireSpec/TabulatedTireSpec never appear as dataclass fields (only inside
// the material union), so no declared-default instance is needed.
inline constexpr auto kTireSpecFields = std::to_array<FieldDecl>({
    REQ_REAL(TireSpec, radial_k_n_m),
    REQ_REAL(TireSpec, radial_c_ns_m),
    REQ_REAL(TireSpec, pressure_pa_gauge),
    REQ_STR(TireSpec, provenance),
    {.name="valid_load_range_n", .type="tuple[float, float]", .kind=FieldKind::Real2Tuple,
     .offset=offsetof(TireSpec, valid_load_range_n), .required=true, .factory="", .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kTireSpecSchema{
    .path="bike_sim.physics.tire.TireSpec", .fields=kTireSpecFields, .declared_default=nullptr};

inline constexpr auto kTabulatedTireSpecFields = std::to_array<FieldDecl>({
    {.name="deflection_m", .type="tuple[float, ...]", .kind=FieldKind::RealTuple,
     .offset=offsetof(TabulatedTireSpec, deflection_m), .required=true, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="force_n", .type="tuple[float, ...]", .kind=FieldKind::RealTuple,
     .offset=offsetof(TabulatedTireSpec, force_n), .required=true, .factory="", .nested=nullptr, .nested2=nullptr},
    REQ_REAL(TabulatedTireSpec, radial_c_ns_m),
    REQ_REAL(TabulatedTireSpec, pressure_pa_gauge),
    REQ_STR(TabulatedTireSpec, provenance),
    {.name="valid_load_range_n", .type="tuple[float, float]", .kind=FieldKind::Real2Tuple,
     .offset=offsetof(TabulatedTireSpec, valid_load_range_n), .required=true, .factory="", .nested=nullptr,
     .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kTabulatedTireSpecSchema{
    .path="bike_sim.physics.tire_curve.TabulatedTireSpec", .fields=kTabulatedTireSpecFields,
    .declared_default=nullptr};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const HingeDensity kDeclaredHingeDensity{};
inline constexpr auto kHingeDensityFields = std::to_array<FieldDecl>({
    {.name="knots_m", .type="tuple[float, ...]", .kind=FieldKind::RealTuple,
     .offset=offsetof(HingeDensity, knots_m), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="stiffness_n_m", .type="tuple[float, ...]", .kind=FieldKind::RealTuple,
     .offset=offsetof(HingeDensity, stiffness_n_m), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    REAL(HingeDensity, damping_ns_m),
    STR(HingeDensity, provenance),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kHingeDensitySchema{
    .path="bike_sim.physics.distributed_tire.HingeDensity", .fields=kHingeDensityFields,
    .declared_default=&kDeclaredHingeDensity};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const DistributedWheelMaterial kDeclaredDistributedWheelMaterial{};
inline constexpr auto kDistributedWheelMaterialFields = std::to_array<FieldDecl>({
    {.name="density", .type="<class 'bike_sim.physics.distributed_tire.HingeDensity'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(DistributedWheelMaterial, density),
     .required=false, .factory="HingeDensity", .nested=&kHingeDensitySchema, .nested2=nullptr},
    STR(DistributedWheelMaterial, fitting_dataset_id),
    {.name="valid_load_range_n", .type="tuple[float, float]", .kind=FieldKind::Real2Tuple,
     .offset=offsetof(DistributedWheelMaterial, valid_load_range_n), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="valid_deflection_range_m", .type="tuple[float, float]", .kind=FieldKind::Real2Tuple,
     .offset=offsetof(DistributedWheelMaterial, valid_deflection_range_m), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    STR(DistributedWheelMaterial, calibration_status),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kDistributedWheelMaterialSchema{
    .path="bike_sim.physics.distributed_tire.DistributedWheelMaterial",
    .fields=kDistributedWheelMaterialFields, .declared_default=&kDeclaredDistributedWheelMaterial};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const DistributedTireConfig kDeclaredDistributedTireConfig{};
inline constexpr auto kDistributedTireConfigFields = std::to_array<FieldDecl>({
    INT(DistributedTireConfig, station_count),
    {.name="density", .type="<class 'bike_sim.physics.distributed_tire.HingeDensity'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(DistributedTireConfig, density), .required=false,
     .factory="HingeDensity", .nested=&kHingeDensitySchema, .nested2=nullptr},
    STR(DistributedTireConfig, fitting_dataset_id),
    {.name="valid_load_range_n", .type="tuple[float, float]", .kind=FieldKind::Real2Tuple,
     .offset=offsetof(DistributedTireConfig, valid_load_range_n), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="valid_deflection_range_m", .type="tuple[float, float]", .kind=FieldKind::Real2Tuple,
     .offset=offsetof(DistributedTireConfig, valid_deflection_range_m), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    STR(DistributedTireConfig, calibration_status),
    {.name="front_material",
     .type="bike_sim.physics.distributed_tire.DistributedWheelMaterial | None",
     .kind=FieldKind::OptionalDataclass,
     .offset=offsetof(DistributedTireConfig, front_material), .required=false, .factory="",
     .nested=&kDistributedWheelMaterialSchema, .nested2=nullptr,
     .optional_get=&optional_record<DistributedWheelMaterial>, .variant_index=nullptr},
    {.name="rear_material",
     .type="bike_sim.physics.distributed_tire.DistributedWheelMaterial | None",
     .kind=FieldKind::OptionalDataclass,
     .offset=offsetof(DistributedTireConfig, rear_material), .required=false, .factory="",
     .nested=&kDistributedWheelMaterialSchema, .nested2=nullptr,
     .optional_get=&optional_record<DistributedWheelMaterial>, .variant_index=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kDistributedTireConfigSchema{
    .path="bike_sim.physics.distributed_tire.DistributedTireConfig",
    .fields=kDistributedTireConfigFields, .declared_default=&kDeclaredDistributedTireConfig};

// TyreSpecs: the first six members are required (sentinels here are never
// encoded — `required` leaves emit `default: null`).
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const TyreSpecs kDeclaredTyreSpecs{.name="", .outer_radius_mm=0., .rim_radius_mm=0., .section_height_mm=0., .pressure_bar=0., .crr_reference=0.};
inline constexpr auto kTyreSpecsFields = std::to_array<FieldDecl>({
    REQ_STR(TyreSpecs, name),
    REQ_REAL(TyreSpecs, outer_radius_mm),
    REQ_REAL(TyreSpecs, rim_radius_mm),
    REQ_REAL(TyreSpecs, section_height_mm),
    REQ_REAL(TyreSpecs, pressure_bar),
    REQ_REAL(TyreSpecs, crr_reference),
    REAL(TyreSpecs, casing_width_mm),
    REAL(TyreSpecs, compressed_casing_mm),
    REAL(TyreSpecs, area_factor),
    REAL(TyreSpecs, carcass_stiffness_n_mm2),
    REAL(TyreSpecs, contact_length_factor),
    REAL(TyreSpecs, rate_stiffening),
    REAL(TyreSpecs, rate_relaxation_s),
    REAL(TyreSpecs, loss_factor),
    REAL(TyreSpecs, hysteresis_rate_eps_mps),
    REAL(TyreSpecs, relaxation_length_mm),
    REAL(TyreSpecs, rim_stiffness_n_mm2),
    REAL(TyreSpecs, tread_loss_crr),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kTyreSpecsSchema{
    .path="bike_sim.physics.tyre.TyreSpecs", .fields=kTyreSpecsFields, .declared_default=&kDeclaredTyreSpecs};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const TireParameters kDeclaredTireParameters{};
inline constexpr auto kTireParametersFields = std::to_array<FieldDecl>({
    {.name="material",
     .type="bike_sim.physics.tire.TireSpec | "
     "bike_sim.physics.tire_curve.TabulatedTireSpec",
     .kind=FieldKind::UnionMaterial, .offset=offsetof(TireParameters, material), .required=false,
     .factory="_material", .nested=&kTireSpecSchema, .nested2=&kTabulatedTireSpecSchema, .optional_get=nullptr,
     .variant_index=&variant_arm<TireSpec, TabulatedTireSpec>,
     .variant_object=&variant_object<TireSpec, TabulatedTireSpec>},
    REAL(TireParameters, tangent_k_n_m),
    REAL(TireParameters, mu),
    REAL(TireParameters, relaxation_length_m),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kTireParametersSchema{
    .path="bike_sim.physics.physical_config.TireParameters", .fields=kTireParametersFields,
    .declared_default=&kDeclaredTireParameters};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const PneumaticTireConfig kDeclaredPneumaticTireConfig{};
inline constexpr auto kPneumaticTireConfigFields = std::to_array<FieldDecl>({
    STR(PneumaticTireConfig, tier),
    // field(default=FRONT_TYRE/REAR_TYRE): dataclass instance defaults.
    {.name="front", .type="<class 'bike_sim.physics.tyre.TyreSpecs'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(PneumaticTireConfig, front), .required=false, .factory="",
     .nested=&kTyreSpecsSchema, .nested2=nullptr},
    {.name="rear", .type="<class 'bike_sim.physics.tyre.TyreSpecs'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(PneumaticTireConfig, rear), .required=false, .factory="",
     .nested=&kTyreSpecsSchema, .nested2=nullptr},
    {.name="surface", .type="str | None", .kind=FieldKind::OptionalText,
     .offset=offsetof(PneumaticTireConfig, surface), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kPneumaticTireConfigSchema{
    .path="bike_sim.physics.physical_config.PneumaticTireConfig",
    .fields=kPneumaticTireConfigFields, .declared_default=&kDeclaredPneumaticTireConfig};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const TireBackendConfig kDeclaredTireBackendConfig{};
inline constexpr auto kTireBackendConfigFields = std::to_array<FieldDecl>({
    STR(TireBackendConfig, backend),
    {.name="front", .type="<class 'bike_sim.physics.physical_config.TireParameters'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(TireBackendConfig, front), .required=false,
     .factory="TireParameters", .nested=&kTireParametersSchema, .nested2=nullptr},
    {.name="rear", .type="<class 'bike_sim.physics.physical_config.TireParameters'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(TireBackendConfig, rear), .required=false,
     .factory="TireParameters", .nested=&kTireParametersSchema, .nested2=nullptr},
    REAL(TireBackendConfig, significant_delta_m),
    REAL(TireBackendConfig, significance_fraction),
    REAL(TireBackendConfig, distinct_normal_deg),
    STR(TireBackendConfig, surface_mode),
    {.name="distributed",
     .type="<class 'bike_sim.physics.distributed_tire.DistributedTireConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(TireBackendConfig, distributed), .required=false,
     .factory="DistributedTireConfig", .nested=&kDistributedTireConfigSchema, .nested2=nullptr},
    {.name="pneumatic",
     .type="<class 'bike_sim.physics.physical_config.PneumaticTireConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(TireBackendConfig, pneumatic), .required=false,
     .factory="PneumaticTireConfig", .nested=&kPneumaticTireConfigSchema, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kTireBackendConfigSchema{
    .path="bike_sim.physics.physical_config.TireBackendConfig",
    .fields=kTireBackendConfigFields, .declared_default=&kDeclaredTireBackendConfig};

inline const DrivetrainSpecs kDeclaredDrivetrainSpecs{};
inline constexpr auto kDrivetrainSpecsFields = std::to_array<FieldDecl>({
    INT(DrivetrainSpecs, front_teeth),
    INT(DrivetrainSpecs, rear_teeth),
    REAL(DrivetrainSpecs, chain_pitch_m),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kDrivetrainSpecsSchema{
    .path="bike_sim.physics.chain.DrivetrainSpecs", .fields=kDrivetrainSpecsFields,
    .declared_default=&kDeclaredDrivetrainSpecs};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const AssistConfig kDeclaredAssistConfig{};
inline constexpr auto kAssistConfigFields = std::to_array<FieldDecl>({
    {.name="profile", .type="str | None", .kind=FieldKind::OptionalText,
     .offset=offsetof(AssistConfig, profile), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    STR(AssistConfig, mode),
    REAL(AssistConfig, gain),
    REAL(AssistConfig, max_torque),
    REAL(AssistConfig, max_power),
    REAL(AssistConfig, tau),
    REAL(AssistConfig, slew),
    REAL(AssistConfig, engage_torque_nm),
    REAL(AssistConfig, gate_min_crank_rad_s),
    REAL(AssistConfig, cutoff_mps),
    REAL(AssistConfig, taper_width_mps),
    {.name="torque_curve", .type="tuple[tuple[float, float], ...] | None",
     .kind=FieldKind::OptionalPairTuple, .offset=offsetof(AssistConfig, torque_curve),
     .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kAssistConfigSchema{
    .path="bike_sim.physics.physical_config.AssistConfig", .fields=kAssistConfigFields,
    .declared_default=&kDeclaredAssistConfig};

inline const BatteryConfig kDeclaredBatteryConfig{};
inline constexpr auto kBatteryConfigFields = std::to_array<FieldDecl>({
    BOOL(BatteryConfig, enabled),
    REAL(BatteryConfig, energy_j),
    REAL(BatteryConfig, copper_w_per_nm2),
    REAL(BatteryConfig, speed_w_per_rad_s2),
    REAL(BatteryConfig, idle_w),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kBatteryConfigSchema{
    .path="bike_sim.physics.physical_config.BatteryConfig", .fields=kBatteryConfigFields,
    .declared_default=&kDeclaredBatteryConfig};

inline const PedalingConfig kDeclaredPedalingConfig{};
inline constexpr auto kPedalingConfigFields = std::to_array<FieldDecl>({
    BOOL(PedalingConfig, enabled),
    REAL(PedalingConfig, coast_above_rpm),
    REAL(PedalingConfig, resume_below_rpm),
    REAL(PedalingConfig, stop_time_s),
    REAL(PedalingConfig, coast_cadence_tau_s),
    REAL(PedalingConfig, mash_cadence_rpm),
    REAL(PedalingConfig, mash_torque_nm),
    REAL(PedalingConfig, effort_slew_nm_s),
    BOOL(PedalingConfig, rollback_brake),
    REAL(PedalingConfig, rollback_engage_mps),
    REAL(PedalingConfig, rollback_release_mps),
    REAL(PedalingConfig, rollback_demand),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kPedalingConfigSchema{
    .path="bike_sim.physics.physical_config.PedalingConfig", .fields=kPedalingConfigFields,
    .declared_default=&kDeclaredPedalingConfig};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const ShiftingConfig kDeclaredShiftingConfig{};
inline constexpr auto kShiftingConfigFields = std::to_array<FieldDecl>({
    BOOL(ShiftingConfig, enabled),
    {.name="cassette", .type="tuple[int, ...]", .kind=FieldKind::IntegerTuple,
     .offset=offsetof(ShiftingConfig, cassette), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    REAL(ShiftingConfig, target_cadence_min_rpm),
    REAL(ShiftingConfig, target_cadence_max_rpm),
    REAL(ShiftingConfig, shift_cooldown_s),
    REAL(ShiftingConfig, shift_cut_duration_s),
    REAL(ShiftingConfig, torque_factor),
    REAL(ShiftingConfig, cadence_smoothing_tau_s),
    REAL(ShiftingConfig, upshift_slip_limit_mps),
    STR(ShiftingConfig, upshift_slip_mode),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kShiftingConfigSchema{
    .path="bike_sim.physics.physical_config.ShiftingConfig", .fields=kShiftingConfigFields,
    .declared_default=&kDeclaredShiftingConfig};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const PhysicalDriveConfig kDeclaredPhysicalDriveConfig{};
inline constexpr auto kPhysicalDriveConfigFields = std::to_array<FieldDecl>({
    {.name="gearing", .type="<class 'bike_sim.physics.chain.DrivetrainSpecs'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(PhysicalDriveConfig, gearing), .required=false,
     .factory="DrivetrainSpecs", .nested=&kDrivetrainSpecsSchema, .nested2=nullptr},
    STR(PhysicalDriveConfig, transmission_model),
    REAL(PhysicalDriveConfig, human_torque_nm),
    REAL(PhysicalDriveConfig, torque_ripple),
    REAL(PhysicalDriveConfig, crank_phase_rad),
    REAL(PhysicalDriveConfig, chain_k_n_m),
    REAL(PhysicalDriveConfig, chain_c_ns_m),
    REAL(PhysicalDriveConfig, freehub_k_nm_rad),
    REAL(PhysicalDriveConfig, freehub_c_nms_rad),
    REAL(PhysicalDriveConfig, bearing_c_nms_rad),
    REAL(PhysicalDriveConfig, brake_ceiling_nm),
    REAL(PhysicalDriveConfig, brake_hold_torque_nm),
    REAL(PhysicalDriveConfig, brake_hold_stiffness_nm_rad),
    REAL(PhysicalDriveConfig, brake_hold_damping_nms_rad),
    REAL(PhysicalDriveConfig, brake_hold_engage_radps),
    REAL(PhysicalDriveConfig, brake_hold_release_radps),
    BOOL(PhysicalDriveConfig, motor_clutch),
    REAL(PhysicalDriveConfig, rotor_inertia_kgm2),
    {.name="assist", .type="<class 'bike_sim.physics.physical_config.AssistConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(PhysicalDriveConfig, assist), .required=false,
     .factory="AssistConfig", .nested=&kAssistConfigSchema, .nested2=nullptr},
    {.name="battery", .type="<class 'bike_sim.physics.physical_config.BatteryConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(PhysicalDriveConfig, battery), .required=false,
     .factory="BatteryConfig", .nested=&kBatteryConfigSchema, .nested2=nullptr},
    {.name="pedaling",
     .type="<class 'bike_sim.physics.physical_config.PedalingConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(PhysicalDriveConfig, pedaling), .required=false,
     .factory="PedalingConfig", .nested=&kPedalingConfigSchema, .nested2=nullptr},
    {.name="shifting",
     .type="<class 'bike_sim.physics.physical_config.ShiftingConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(PhysicalDriveConfig, shifting), .required=false,
     .factory="ShiftingConfig", .nested=&kShiftingConfigSchema, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kPhysicalDriveConfigSchema{
    .path="bike_sim.physics.physical_config.PhysicalDriveConfig",
    .fields=kPhysicalDriveConfigFields, .declared_default=&kDeclaredPhysicalDriveConfig};

inline const ResistanceConfig kDeclaredResistanceConfig{};
inline constexpr auto kResistanceConfigFields = std::to_array<FieldDecl>({
    REAL(ResistanceConfig, crr),
    REAL(ResistanceConfig, rolling_taper_rad_s),
    REAL(ResistanceConfig, rho_kg_m3),
    REAL(ResistanceConfig, cda_m2),
    {.name="wind_world_mps", .type="tuple[float, float, float]", .kind=FieldKind::Real3Tuple,
     .offset=offsetof(ResistanceConfig, wind_world_mps), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="point_body_m", .type="tuple[float, float, float]", .kind=FieldKind::Real3Tuple,
     .offset=offsetof(ResistanceConfig, point_body_m), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kResistanceConfigSchema{
    .path="bike_sim.physics.physical_config.ResistanceConfig",
    .fields=kResistanceConfigFields, .declared_default=&kDeclaredResistanceConfig};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const ArticulatedConfig kDeclaredArticulatedConfig{};
inline constexpr auto kArticulatedConfigFields = std::to_array<FieldDecl>({
    REAL(ArticulatedConfig, pedal_support_fraction),
    REAL(ArticulatedConfig, bar_support_fraction),
    REAL(ArticulatedConfig, posture_sole_depth_m),
    REAL(ArticulatedConfig, swing_clearance_m),
    REAL(ArticulatedConfig, coasting_brake_d_nm_s_rad),
    REAL(ArticulatedConfig, coasting_brake_limit_nm),
    REAL(ArticulatedConfig, stance_blend_load_n),
    REAL(ArticulatedConfig, posture_pitch_k_nm_rad),
    REAL(ArticulatedConfig, posture_pitch_d_nms_rad),
    REAL(ArticulatedConfig, posture_pitch_limit_nm),
    REAL(ArticulatedConfig, posture_translation_k_n_m),
    REAL(ArticulatedConfig, posture_translation_d_ns_m),
    REAL(ArticulatedConfig, posture_translation_limit_n),
    REAL(ArticulatedConfig, saddle_patch_half_length_m),
    REAL(ArticulatedConfig, pedal_patch_half_length_m),
    REAL(ArticulatedConfig, support_pad_radius_m),
    REAL(ArticulatedConfig, support_k_n_m),
    REAL(ArticulatedConfig, support_c_ns_m),
    REAL(ArticulatedConfig, pedal_c_ns_m),
    REAL(ArticulatedConfig, support_tangent_k_n_m),
    REAL(ArticulatedConfig, support_mu),
    REAL(ArticulatedConfig, support_length_m),
    REAL(ArticulatedConfig, grip_k_n_m),
    REAL(ArticulatedConfig, grip_c_ns_m),
    REAL(ArticulatedConfig, grip_release_distance_m),
    {.name="grip_pair_force_limit_n", .type="float | None", .kind=FieldKind::OptionalReal,
     .offset=offsetof(ArticulatedConfig, grip_pair_force_limit_n), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    REAL(ArticulatedConfig, grip_capture_distance_m),
    REAL(ArticulatedConfig, grip_capture_speed_mps),
    REAL(ArticulatedConfig, arm_reach_fraction),
    REAL(ArticulatedConfig, joint_kp_nm_rad),
    REAL(ArticulatedConfig, joint_kd_nms_rad),
    {.name="joint_passive_damping_nms_rad", .type="float | None", .kind=FieldKind::OptionalReal,
     .offset=offsetof(ArticulatedConfig, joint_passive_damping_nms_rad), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    BOOL(ArticulatedConfig, coupled_task_control),
    REAL(ArticulatedConfig, joint_limit_nm),
    REAL(ArticulatedConfig, joint_speed_limit_rad_s),
    REAL(ArticulatedConfig, joint_power_limit_w),
    {.name="joint_envelope_path", .type="str | None", .kind=FieldKind::OptionalText,
     .offset=offsetof(ArticulatedConfig, joint_envelope_path), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    REAL(ArticulatedConfig, joint_envelope_soft_margin_rad),
    REAL(ArticulatedConfig, joint_envelope_soft_k_nm_rad),
    REAL(ArticulatedConfig, activation_tau_s),
    {.name="active_positive_power_limit_w", .type="float | None",
     .kind=FieldKind::OptionalReal,
     .offset=offsetof(ArticulatedConfig, active_positive_power_limit_w), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    REAL(ArticulatedConfig, foot_mu),
    REAL(ArticulatedConfig, saddle_mu),
    REAL(ArticulatedConfig, pedal_min_normal_n),
    REAL(ArticulatedConfig, saddle_reserve_weight_fraction),
    REAL(ArticulatedConfig, grip_pull_per_hand_n),
    REAL(ArticulatedConfig, pedal_ankle_amplitude_rad),
    REAL(ArticulatedConfig, pedal_scrape_fraction),
    REAL(ArticulatedConfig, pedal_torque_ripple),
    REAL(ArticulatedConfig, return_foot_preload_n),
    REAL(ArticulatedConfig, weld_servo_gain),
    REAL(ArticulatedConfig, weld_servo_limit_nm),
    REAL(ArticulatedConfig, leg_track_damping_nms_rad),
    REAL(ArticulatedConfig, link_max_gap_m),
    REAL(ArticulatedConfig, road_lookahead_m),
    {.name="joint_strength_path", .type="str | None", .kind=FieldKind::OptionalText,
     .offset=offsetof(ArticulatedConfig, joint_strength_path), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    STR(ArticulatedConfig, pedal_attachment),
    STR(ArticulatedConfig, saddle_attachment),
    STR(ArticulatedConfig, grip_attachment),
    REAL(ArticulatedConfig, balance_floor_kmh),
    REAL(ArticulatedConfig, balance_dwell_s),
    REAL(ArticulatedConfig, balance_grace_s),
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kArticulatedConfigSchema{
    .path="bike_sim.physics.physical_config.ArticulatedConfig",
    .fields=kArticulatedConfigFields, .declared_default=&kDeclaredArticulatedConfig};

#undef REAL
#undef REQ_REAL
#undef INT
#undef REQ_INT
#undef BOOL
#undef STR
#undef REQ_STR
#undef BIKE_FIELD

} // namespace detail
} // namespace bike::configuration

