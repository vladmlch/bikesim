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
    return TireSpec{130000., 800., 200000., "synthetic", {0., 2000.}};
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
inline const TyreSpecs kFrontTyre{
    "Schwalbe Magic Mary 29x2.4", 372., 320., 57., 1.5, .011,
    60., 6., .4435, .1194, .8589, .2484, .2, .0662, .01, 60., 28., .00586};
inline const TyreSpecs kRearTyre{
    "Schwalbe Hans Dampf 27.5x2.4", 352., 300., 55., 1.7, .0103,
    60., 6., .4576, .1225, .8801, .2498, .2, .069, .01, 60., 28., .00481};

struct TierSpec {
    std::string name;
    int n_rays;
    double half_angle_deg;
    double timestep_s;
    bool discretised_brush;
};

// tyre.py TIERS, in dict insertion order ('fast', 'detailed').
inline constexpr TierSpec kTireTiers[] = {
    {"fast", 64, 75.0, 0.0005, false},
    {"detailed", 256, 75.0, 0.00025, true},
};
[[nodiscard]] const TierSpec* find_tire_tier(std::string_view name);

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
inline const MotorProfile kBoschCxGen4{
    "bosch_performance_line_cx_gen4", 85., 250., 600., .04,
    {.6, 1.4, {1.4, 3.4}, 3.4}, 40., 120., 25. / 3.6, 2. / 3.6,
    0.08726646259971647 /* radians(5) */,
    "Bosch Performance Line CX Gen 4 (no ABS); recalled from memory "
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
inline constexpr int kCassette12sTeeth[] = {10, 12, 14, 16, 18, 21,
                                            24, 28, 33, 39, 45, 51};

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

    [[nodiscard]] AttachmentBudget attachment_budget() const {
        return {pedal_min_normal_n, foot_mu, saddle_mu,
                grip_pull_per_hand_n, link_max_gap_m};
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
inline constexpr FieldDecl kTireSpecFields[] = {
    REQ_REAL(TireSpec, radial_k_n_m),
    REQ_REAL(TireSpec, radial_c_ns_m),
    REQ_REAL(TireSpec, pressure_pa_gauge),
    REQ_STR(TireSpec, provenance),
    {"valid_load_range_n", "tuple[float, float]", FieldKind::Real2Tuple,
     offsetof(TireSpec, valid_load_range_n), true, "", nullptr, nullptr},
};
inline const RecordSchema kTireSpecSchema{
    "bike_sim.physics.tire.TireSpec", kTireSpecFields, nullptr};

inline constexpr FieldDecl kTabulatedTireSpecFields[] = {
    {"deflection_m", "tuple[float, ...]", FieldKind::RealTuple,
     offsetof(TabulatedTireSpec, deflection_m), true, "", nullptr, nullptr},
    {"force_n", "tuple[float, ...]", FieldKind::RealTuple,
     offsetof(TabulatedTireSpec, force_n), true, "", nullptr, nullptr},
    REQ_REAL(TabulatedTireSpec, radial_c_ns_m),
    REQ_REAL(TabulatedTireSpec, pressure_pa_gauge),
    REQ_STR(TabulatedTireSpec, provenance),
    {"valid_load_range_n", "tuple[float, float]", FieldKind::Real2Tuple,
     offsetof(TabulatedTireSpec, valid_load_range_n), true, "", nullptr,
     nullptr},
};
inline const RecordSchema kTabulatedTireSpecSchema{
    "bike_sim.physics.tire_curve.TabulatedTireSpec", kTabulatedTireSpecFields,
    nullptr};

inline const HingeDensity kDeclaredHingeDensity{};
inline constexpr FieldDecl kHingeDensityFields[] = {
    {"knots_m", "tuple[float, ...]", FieldKind::RealTuple,
     offsetof(HingeDensity, knots_m), false, "", nullptr, nullptr},
    {"stiffness_n_m", "tuple[float, ...]", FieldKind::RealTuple,
     offsetof(HingeDensity, stiffness_n_m), false, "", nullptr, nullptr},
    REAL(HingeDensity, damping_ns_m),
    STR(HingeDensity, provenance),
};
inline const RecordSchema kHingeDensitySchema{
    "bike_sim.physics.distributed_tire.HingeDensity", kHingeDensityFields,
    &kDeclaredHingeDensity};

inline const DistributedWheelMaterial kDeclaredDistributedWheelMaterial{};
inline constexpr FieldDecl kDistributedWheelMaterialFields[] = {
    {"density", "<class 'bike_sim.physics.distributed_tire.HingeDensity'>",
     FieldKind::Dataclass, offsetof(DistributedWheelMaterial, density),
     false, "HingeDensity", &kHingeDensitySchema, nullptr},
    STR(DistributedWheelMaterial, fitting_dataset_id),
    {"valid_load_range_n", "tuple[float, float]", FieldKind::Real2Tuple,
     offsetof(DistributedWheelMaterial, valid_load_range_n), false, "",
     nullptr, nullptr},
    {"valid_deflection_range_m", "tuple[float, float]", FieldKind::Real2Tuple,
     offsetof(DistributedWheelMaterial, valid_deflection_range_m), false, "",
     nullptr, nullptr},
    STR(DistributedWheelMaterial, calibration_status),
};
inline const RecordSchema kDistributedWheelMaterialSchema{
    "bike_sim.physics.distributed_tire.DistributedWheelMaterial",
    kDistributedWheelMaterialFields, &kDeclaredDistributedWheelMaterial};

inline const DistributedTireConfig kDeclaredDistributedTireConfig{};
inline constexpr FieldDecl kDistributedTireConfigFields[] = {
    INT(DistributedTireConfig, station_count),
    {"density", "<class 'bike_sim.physics.distributed_tire.HingeDensity'>",
     FieldKind::Dataclass, offsetof(DistributedTireConfig, density), false,
     "HingeDensity", &kHingeDensitySchema, nullptr},
    STR(DistributedTireConfig, fitting_dataset_id),
    {"valid_load_range_n", "tuple[float, float]", FieldKind::Real2Tuple,
     offsetof(DistributedTireConfig, valid_load_range_n), false, "", nullptr,
     nullptr},
    {"valid_deflection_range_m", "tuple[float, float]", FieldKind::Real2Tuple,
     offsetof(DistributedTireConfig, valid_deflection_range_m), false, "",
     nullptr, nullptr},
    STR(DistributedTireConfig, calibration_status),
    {"front_material",
     "bike_sim.physics.distributed_tire.DistributedWheelMaterial | None",
     FieldKind::OptionalDataclass,
     offsetof(DistributedTireConfig, front_material), false, "",
     &kDistributedWheelMaterialSchema, nullptr,
     &optional_record<DistributedWheelMaterial>, nullptr},
    {"rear_material",
     "bike_sim.physics.distributed_tire.DistributedWheelMaterial | None",
     FieldKind::OptionalDataclass,
     offsetof(DistributedTireConfig, rear_material), false, "",
     &kDistributedWheelMaterialSchema, nullptr,
     &optional_record<DistributedWheelMaterial>, nullptr},
};
inline const RecordSchema kDistributedTireConfigSchema{
    "bike_sim.physics.distributed_tire.DistributedTireConfig",
    kDistributedTireConfigFields, &kDeclaredDistributedTireConfig};

// TyreSpecs: the first six members are required (sentinels here are never
// encoded — `required` leaves emit `default: null`).
inline const TyreSpecs kDeclaredTyreSpecs{"", 0., 0., 0., 0., 0.};
inline constexpr FieldDecl kTyreSpecsFields[] = {
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
};
inline const RecordSchema kTyreSpecsSchema{
    "bike_sim.physics.tyre.TyreSpecs", kTyreSpecsFields, &kDeclaredTyreSpecs};

inline const TireParameters kDeclaredTireParameters{};
inline constexpr FieldDecl kTireParametersFields[] = {
    {"material",
     "bike_sim.physics.tire.TireSpec | "
     "bike_sim.physics.tire_curve.TabulatedTireSpec",
     FieldKind::UnionMaterial, offsetof(TireParameters, material), false,
     "_material", &kTireSpecSchema, &kTabulatedTireSpecSchema, nullptr,
     &variant_arm<TireSpec, TabulatedTireSpec>,
     &variant_object<TireSpec, TabulatedTireSpec>},
    REAL(TireParameters, tangent_k_n_m),
    REAL(TireParameters, mu),
    REAL(TireParameters, relaxation_length_m),
};
inline const RecordSchema kTireParametersSchema{
    "bike_sim.physics.physical_config.TireParameters", kTireParametersFields,
    &kDeclaredTireParameters};

inline const PneumaticTireConfig kDeclaredPneumaticTireConfig{};
inline constexpr FieldDecl kPneumaticTireConfigFields[] = {
    STR(PneumaticTireConfig, tier),
    // field(default=FRONT_TYRE/REAR_TYRE): dataclass instance defaults.
    {"front", "<class 'bike_sim.physics.tyre.TyreSpecs'>",
     FieldKind::Dataclass, offsetof(PneumaticTireConfig, front), false, "",
     &kTyreSpecsSchema, nullptr},
    {"rear", "<class 'bike_sim.physics.tyre.TyreSpecs'>",
     FieldKind::Dataclass, offsetof(PneumaticTireConfig, rear), false, "",
     &kTyreSpecsSchema, nullptr},
    {"surface", "str | None", FieldKind::OptionalText,
     offsetof(PneumaticTireConfig, surface), false, "", nullptr, nullptr},
};
inline const RecordSchema kPneumaticTireConfigSchema{
    "bike_sim.physics.physical_config.PneumaticTireConfig",
    kPneumaticTireConfigFields, &kDeclaredPneumaticTireConfig};

inline const TireBackendConfig kDeclaredTireBackendConfig{};
inline constexpr FieldDecl kTireBackendConfigFields[] = {
    STR(TireBackendConfig, backend),
    {"front", "<class 'bike_sim.physics.physical_config.TireParameters'>",
     FieldKind::Dataclass, offsetof(TireBackendConfig, front), false,
     "TireParameters", &kTireParametersSchema, nullptr},
    {"rear", "<class 'bike_sim.physics.physical_config.TireParameters'>",
     FieldKind::Dataclass, offsetof(TireBackendConfig, rear), false,
     "TireParameters", &kTireParametersSchema, nullptr},
    REAL(TireBackendConfig, significant_delta_m),
    REAL(TireBackendConfig, significance_fraction),
    REAL(TireBackendConfig, distinct_normal_deg),
    STR(TireBackendConfig, surface_mode),
    {"distributed",
     "<class 'bike_sim.physics.distributed_tire.DistributedTireConfig'>",
     FieldKind::Dataclass, offsetof(TireBackendConfig, distributed), false,
     "DistributedTireConfig", &kDistributedTireConfigSchema, nullptr},
    {"pneumatic",
     "<class 'bike_sim.physics.physical_config.PneumaticTireConfig'>",
     FieldKind::Dataclass, offsetof(TireBackendConfig, pneumatic), false,
     "PneumaticTireConfig", &kPneumaticTireConfigSchema, nullptr},
};
inline const RecordSchema kTireBackendConfigSchema{
    "bike_sim.physics.physical_config.TireBackendConfig",
    kTireBackendConfigFields, &kDeclaredTireBackendConfig};

inline const DrivetrainSpecs kDeclaredDrivetrainSpecs{};
inline constexpr FieldDecl kDrivetrainSpecsFields[] = {
    INT(DrivetrainSpecs, front_teeth),
    INT(DrivetrainSpecs, rear_teeth),
    REAL(DrivetrainSpecs, chain_pitch_m),
};
inline const RecordSchema kDrivetrainSpecsSchema{
    "bike_sim.physics.chain.DrivetrainSpecs", kDrivetrainSpecsFields,
    &kDeclaredDrivetrainSpecs};

inline const AssistConfig kDeclaredAssistConfig{};
inline constexpr FieldDecl kAssistConfigFields[] = {
    {"profile", "str | None", FieldKind::OptionalText,
     offsetof(AssistConfig, profile), false, "", nullptr, nullptr},
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
    {"torque_curve", "tuple[tuple[float, float], ...] | None",
     FieldKind::OptionalPairTuple, offsetof(AssistConfig, torque_curve),
     false, "", nullptr, nullptr},
};
inline const RecordSchema kAssistConfigSchema{
    "bike_sim.physics.physical_config.AssistConfig", kAssistConfigFields,
    &kDeclaredAssistConfig};

inline const BatteryConfig kDeclaredBatteryConfig{};
inline constexpr FieldDecl kBatteryConfigFields[] = {
    BOOL(BatteryConfig, enabled),
    REAL(BatteryConfig, energy_j),
    REAL(BatteryConfig, copper_w_per_nm2),
    REAL(BatteryConfig, speed_w_per_rad_s2),
    REAL(BatteryConfig, idle_w),
};
inline const RecordSchema kBatteryConfigSchema{
    "bike_sim.physics.physical_config.BatteryConfig", kBatteryConfigFields,
    &kDeclaredBatteryConfig};

inline const PedalingConfig kDeclaredPedalingConfig{};
inline constexpr FieldDecl kPedalingConfigFields[] = {
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
};
inline const RecordSchema kPedalingConfigSchema{
    "bike_sim.physics.physical_config.PedalingConfig", kPedalingConfigFields,
    &kDeclaredPedalingConfig};

inline const ShiftingConfig kDeclaredShiftingConfig{};
inline constexpr FieldDecl kShiftingConfigFields[] = {
    BOOL(ShiftingConfig, enabled),
    {"cassette", "tuple[int, ...]", FieldKind::IntegerTuple,
     offsetof(ShiftingConfig, cassette), false, "", nullptr, nullptr},
    REAL(ShiftingConfig, target_cadence_min_rpm),
    REAL(ShiftingConfig, target_cadence_max_rpm),
    REAL(ShiftingConfig, shift_cooldown_s),
    REAL(ShiftingConfig, shift_cut_duration_s),
    REAL(ShiftingConfig, torque_factor),
    REAL(ShiftingConfig, cadence_smoothing_tau_s),
    REAL(ShiftingConfig, upshift_slip_limit_mps),
    STR(ShiftingConfig, upshift_slip_mode),
};
inline const RecordSchema kShiftingConfigSchema{
    "bike_sim.physics.physical_config.ShiftingConfig", kShiftingConfigFields,
    &kDeclaredShiftingConfig};

inline const PhysicalDriveConfig kDeclaredPhysicalDriveConfig{};
inline constexpr FieldDecl kPhysicalDriveConfigFields[] = {
    {"gearing", "<class 'bike_sim.physics.chain.DrivetrainSpecs'>",
     FieldKind::Dataclass, offsetof(PhysicalDriveConfig, gearing), false,
     "DrivetrainSpecs", &kDrivetrainSpecsSchema, nullptr},
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
    BOOL(PhysicalDriveConfig, motor_clutch),
    REAL(PhysicalDriveConfig, rotor_inertia_kgm2),
    {"assist", "<class 'bike_sim.physics.physical_config.AssistConfig'>",
     FieldKind::Dataclass, offsetof(PhysicalDriveConfig, assist), false,
     "AssistConfig", &kAssistConfigSchema, nullptr},
    {"battery", "<class 'bike_sim.physics.physical_config.BatteryConfig'>",
     FieldKind::Dataclass, offsetof(PhysicalDriveConfig, battery), false,
     "BatteryConfig", &kBatteryConfigSchema, nullptr},
    {"pedaling",
     "<class 'bike_sim.physics.physical_config.PedalingConfig'>",
     FieldKind::Dataclass, offsetof(PhysicalDriveConfig, pedaling), false,
     "PedalingConfig", &kPedalingConfigSchema, nullptr},
    {"shifting",
     "<class 'bike_sim.physics.physical_config.ShiftingConfig'>",
     FieldKind::Dataclass, offsetof(PhysicalDriveConfig, shifting), false,
     "ShiftingConfig", &kShiftingConfigSchema, nullptr},
};
inline const RecordSchema kPhysicalDriveConfigSchema{
    "bike_sim.physics.physical_config.PhysicalDriveConfig",
    kPhysicalDriveConfigFields, &kDeclaredPhysicalDriveConfig};

inline const ResistanceConfig kDeclaredResistanceConfig{};
inline constexpr FieldDecl kResistanceConfigFields[] = {
    REAL(ResistanceConfig, crr),
    REAL(ResistanceConfig, rolling_taper_rad_s),
    REAL(ResistanceConfig, rho_kg_m3),
    REAL(ResistanceConfig, cda_m2),
    {"wind_world_mps", "tuple[float, float, float]", FieldKind::Real3Tuple,
     offsetof(ResistanceConfig, wind_world_mps), false, "", nullptr, nullptr},
    {"point_body_m", "tuple[float, float, float]", FieldKind::Real3Tuple,
     offsetof(ResistanceConfig, point_body_m), false, "", nullptr, nullptr},
};
inline const RecordSchema kResistanceConfigSchema{
    "bike_sim.physics.physical_config.ResistanceConfig",
    kResistanceConfigFields, &kDeclaredResistanceConfig};

inline const ArticulatedConfig kDeclaredArticulatedConfig{};
inline constexpr FieldDecl kArticulatedConfigFields[] = {
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
    {"grip_pair_force_limit_n", "float | None", FieldKind::OptionalReal,
     offsetof(ArticulatedConfig, grip_pair_force_limit_n), false, "",
     nullptr, nullptr},
    REAL(ArticulatedConfig, grip_capture_distance_m),
    REAL(ArticulatedConfig, grip_capture_speed_mps),
    REAL(ArticulatedConfig, arm_reach_fraction),
    REAL(ArticulatedConfig, joint_kp_nm_rad),
    REAL(ArticulatedConfig, joint_kd_nms_rad),
    {"joint_passive_damping_nms_rad", "float | None", FieldKind::OptionalReal,
     offsetof(ArticulatedConfig, joint_passive_damping_nms_rad), false, "",
     nullptr, nullptr},
    BOOL(ArticulatedConfig, coupled_task_control),
    REAL(ArticulatedConfig, joint_limit_nm),
    REAL(ArticulatedConfig, joint_speed_limit_rad_s),
    REAL(ArticulatedConfig, joint_power_limit_w),
    {"joint_envelope_path", "str | None", FieldKind::OptionalText,
     offsetof(ArticulatedConfig, joint_envelope_path), false, "", nullptr,
     nullptr},
    REAL(ArticulatedConfig, joint_envelope_soft_margin_rad),
    REAL(ArticulatedConfig, joint_envelope_soft_k_nm_rad),
    REAL(ArticulatedConfig, activation_tau_s),
    {"active_positive_power_limit_w", "float | None",
     FieldKind::OptionalReal,
     offsetof(ArticulatedConfig, active_positive_power_limit_w), false, "",
     nullptr, nullptr},
    REAL(ArticulatedConfig, foot_mu),
    REAL(ArticulatedConfig, saddle_mu),
    REAL(ArticulatedConfig, pedal_min_normal_n),
    REAL(ArticulatedConfig, saddle_reserve_weight_fraction),
    REAL(ArticulatedConfig, grip_pull_per_hand_n),
    REAL(ArticulatedConfig, pedal_ankle_amplitude_rad),
    REAL(ArticulatedConfig, pedal_scrape_fraction),
    REAL(ArticulatedConfig, pedal_torque_ripple),
    REAL(ArticulatedConfig, return_foot_preload_n),
    REAL(ArticulatedConfig, link_max_gap_m),
    REAL(ArticulatedConfig, road_lookahead_m),
    {"joint_strength_path", "str | None", FieldKind::OptionalText,
     offsetof(ArticulatedConfig, joint_strength_path), false, "", nullptr,
     nullptr},
    STR(ArticulatedConfig, pedal_attachment),
    STR(ArticulatedConfig, saddle_attachment),
    STR(ArticulatedConfig, grip_attachment),
    REAL(ArticulatedConfig, balance_floor_kmh),
    REAL(ArticulatedConfig, balance_dwell_s),
    REAL(ArticulatedConfig, balance_grace_s),
};
inline const RecordSchema kArticulatedConfigSchema{
    "bike_sim.physics.physical_config.ArticulatedConfig",
    kArticulatedConfigFields, &kDeclaredArticulatedConfig};

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

