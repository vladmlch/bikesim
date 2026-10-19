// configuration/suspension_spec.hpp — the T12 suspension-side input
// records: damper.py's DamperPreset/DamperClickConfig/DAMPER_PRESETS, the
// air_spring.py AirSpringSpecs and coil_shock.py CoilShockSpecs dataclasses,
// and the suspension_config.py build_suspension_components field mapping as
// a resolved spec record.
//
// The force-computing component classes (Charger3Damper, SuperDeluxeDamper,
// ForkAirSpring, CoilShock, BikeSuspensionSystem, SuspensionController) stay
// with their T15 owner; this header carries the input declarations, the
// constructor-level resolved configurations and every validation the Python
// constructors run. The resolved damper members reuse the existing
// nativecfg::Charger3Config/SuperDeluxeConfig wire records, which already
// model exactly the ctor state (core constants + travel + HBO zone).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "fields.hpp"
#include "physical_config.hpp"  // PedalingConfig, ShiftingConfig, DrivetrainSpecs
#include "../config_types.hpp"

namespace bike::configuration {

// ---------------------------------------------------------------------------
// air_spring.py module constants + AirSpringSpecs.
// ---------------------------------------------------------------------------
inline constexpr double kPsiToPa = 6894.757293168;
inline constexpr double kPaToPsi = 1.0 / kPsiToPa;
inline constexpr double kSpringBarToPa = 100000.0;
inline constexpr double kPaToBar = 1.0 / kSpringBarToPa;
inline constexpr double kAtmPa = 101325.0;

struct AirSpringSpecs {
    double stanchion_inner_diam_mm = 24.0;
    double total_travel_mm = 180.0;
    double pos_chamber_length_mm = 280.0;
    double neg_chamber_length_mm = 55.0;
    double token_volume_cm3 = 8.0;
    int max_tokens = 4;
    int default_tokens = 2;
    double gamma = 1.40;
    double atm_pressure_pa = kAtmPa;

    // AirSpringSpecs derived properties (derived() overflow checks in
    // validate.cpp's definitions).
    [[nodiscard]] double piston_area_m2() const;
    [[nodiscard]] double token_volume_m3() const;
    [[nodiscard]] double base_pos_volume_m3() const;
    [[nodiscard]] double base_neg_volume_m3() const;
};

// ---------------------------------------------------------------------------
// coil_shock.py CoilShockSpecs — a plain (mutable) dataclass.
// ---------------------------------------------------------------------------
struct CoilShockSpecs {
    double rate_n_m = 114600.0;
    double preload_mm = 0.0;
    double stroke_mm = 65.0;
    double bumper_length_mm = 10.0;
    double bumper_peak_n = 7000.0;

    // CoilShockSpecs.bumper_engage_mm.
    [[nodiscard]] double bumper_engage_mm() const {
        return stroke_mm - bumper_length_mm;
    }
};

// ---------------------------------------------------------------------------
// damper.py DamperPreset labels + DamperClickConfig + DAMPER_PRESETS.
// ---------------------------------------------------------------------------
// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class DamperPreset : std::uint8_t { kBase, kPlush, kEnduro, kPark, kCustom };

// The enum's .value strings, in declaration order.
[[nodiscard]] std::string_view damper_preset_label(DamperPreset preset);

// damper.py DamperClickConfig — a plain @dataclass (no __post_init__).
struct DamperClickConfig {
    int fork_hsc = 2;      // 0..4 (0=Open, 2=Neutral, 4=Firm)
    int fork_lsc = 7;      // 0..14 (0=Open, 7=Neutral, 14=Firm)
    int fork_rebound = 9;  // 0..17 (0=Fast/Open, 9=Neutral, 17=Slow/Firm)

    int shock_hsc = 2;      // 0..4
    int shock_lsc = 7;      // 0..14
    int shock_rebound = 7;  // 0..14
    int shock_hbo = 2;      // 0..4
    bool shock_lockout = false;  // false=Open, true=Firm Pedal Platform

    std::string preset_name =
        "Base Tune (Factory Neutral)";  // DamperPreset.BASE.value
};

// damper.py DAMPER_PRESETS — the factory map holds BASE/PLUSH/ENDURO/PARK
// only (CUSTOM is never registered).
[[nodiscard]] const DamperClickConfig &damper_preset_row(DamperPreset preset);
[[nodiscard]] bool damper_preset_registered(DamperPreset preset);

// ---------------------------------------------------------------------------
// suspension_config.py build_suspension_components — the resolved setup as a
// spec record. `SuspensionInputs` carries exactly the BikeSpecs fields the
// Python function reads (geometry/specs.py is T14's; this header owns the
// suspension-side assembly and validation).
// ---------------------------------------------------------------------------
struct SuspensionInputs {
    // BikeSpecs click fields -> DamperClickConfig kwargs.
    int fork_hsc = 2;
    int fork_lsc = 7;
    int fork_rebound = 9;
    int shock_hsc = 2;
    int shock_lsc = 7;
    int shock_rebound = 7;
    int shock_hbo = 2;
    bool shock_lockout = false;
    // BikeSpecs geometry/rate fields -> damper travels + spring args.
    double fork_travel = 180.0;
    double shock_stroke = 65.0;
    double fork_initial_psi = 85.2;
    int fork_air_tokens = 2;
    double shock_stiffness = 114600.0;
};

// The resolved suspension setup: what the Python constructors receive after
// the BikeSpecs field mapping. The damper members are the existing
// nativecfg wire records — they carry the ctor constants plus the resolved
// clicks/HBO zone; `clicks` keeps the requested DamperClickConfig.
struct SuspensionSpec {
    DamperClickConfig clicks;
    nativecfg::Charger3Config fork_damper;
    nativecfg::SuperDeluxeConfig shock_damper{};
    AirSpringSpecs air;
    int num_tokens{};
    double gauge_pressure_psi{};
    CoilShockSpecs coil;
};

// build_suspension_components(specs, preload_mm=0.): maps the input fields,
// clamps the click integers like BaseDamper.__init__, resolves the HBO
// zones, and runs every ctor/domain validation (see validate.cpp). Throws
// std::invalid_argument carrying the Python messages.
[[nodiscard]] SuspensionSpec
resolve_suspension(const SuspensionInputs &inputs, double preload_mm = 0.);

// ---------------------------------------------------------------------------
// domain_validation.py — the six free validators. The damper functions take
// the resolved nativecfg records and the Python class name as the error
// label; the config functions take this boundary's input records.
// ---------------------------------------------------------------------------
namespace domain {

// air_specs(specs) — the AirSpringSpecs.__post_init__ body.
void air_specs(const AirSpringSpecs &specs);
// coil_specs(specs) — the CoilShockSpecs.__post_init__ body.
void coil_specs(const CoilShockSpecs &specs);
// pedaling_config(config) — PedalingPolicy's domain gate.
void pedaling_config(const PedalingConfig &config);
// shifting_config(config, gearing) — CadenceShifter's domain gate.
void shifting_config(const ShiftingConfig &config,
                     const DrivetrainSpecs &gearing);

// damper_core(damper) — shared band bounds; `travel` is the damper's
// total_travel_mm (the fork's travel, the shock's stroke).
void damper_core(const nativecfg::DamperCore &core, double total_travel_mm,
                 std::string_view name);
// damper(damper) — damper_core + the subclass fields. The fork record
// carries c_hbo_base (the 'hasattr' branch); the shock record carries
// max_hbo/hbo_clicks/lockout_firm/c_hbo_*/lockout_* (the else branch).
void damper(const nativecfg::Charger3Config &damper,
            std::string_view name = "Charger3Damper");
void damper(const nativecfg::SuperDeluxeConfig &damper,
            std::string_view name = "SuperDeluxeDamper");

} // namespace domain

// ---------------------------------------------------------------------------
// Construction-time validators — __post_init__ bodies (validate.cpp).
// ---------------------------------------------------------------------------
namespace detail {
void post_init(const AirSpringSpecs &);
void post_init(const CoilShockSpecs &);
} // namespace detail

// ---------------------------------------------------------------------------
// Field tables — single declaration shared by schema/provenance/loading.
// ---------------------------------------------------------------------------
namespace detail {

inline const AirSpringSpecs kDeclaredAirSpringSpecs{};
inline constexpr auto kAirSpringSpecsFields = std::to_array<FieldDecl>({
    {.name="stanchion_inner_diam_mm", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(AirSpringSpecs, stanchion_inner_diam_mm), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="total_travel_mm", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(AirSpringSpecs, total_travel_mm), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="pos_chamber_length_mm", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(AirSpringSpecs, pos_chamber_length_mm), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="neg_chamber_length_mm", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(AirSpringSpecs, neg_chamber_length_mm), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="token_volume_cm3", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(AirSpringSpecs, token_volume_cm3), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="max_tokens", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(AirSpringSpecs, max_tokens), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="default_tokens", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(AirSpringSpecs, default_tokens), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="gamma", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(AirSpringSpecs, gamma), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="atm_pressure_pa", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(AirSpringSpecs, atm_pressure_pa), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kAirSpringSpecsSchema{
    .path="bike_sim.physics.air_spring.AirSpringSpecs", .fields=kAirSpringSpecsFields,
    .declared_default=&kDeclaredAirSpringSpecs};

inline const CoilShockSpecs kDeclaredCoilShockSpecs{};
inline constexpr auto kCoilShockSpecsFields = std::to_array<FieldDecl>({
    {.name="rate_n_m", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(CoilShockSpecs, rate_n_m), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="preload_mm", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(CoilShockSpecs, preload_mm), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="stroke_mm", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(CoilShockSpecs, stroke_mm), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="bumper_length_mm", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(CoilShockSpecs, bumper_length_mm), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="bumper_peak_n", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(CoilShockSpecs, bumper_peak_n), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kCoilShockSpecsSchema{
    .path="bike_sim.physics.coil_shock.CoilShockSpecs", .fields=kCoilShockSpecsFields,
    .declared_default=&kDeclaredCoilShockSpecs};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const DamperClickConfig kDeclaredDamperClickConfig{};
inline constexpr auto kDamperClickConfigFields = std::to_array<FieldDecl>({
    {.name="fork_hsc", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(DamperClickConfig, fork_hsc), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="fork_lsc", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(DamperClickConfig, fork_lsc), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="fork_rebound", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(DamperClickConfig, fork_rebound), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="shock_hsc", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(DamperClickConfig, shock_hsc), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="shock_lsc", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(DamperClickConfig, shock_lsc), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="shock_rebound", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(DamperClickConfig, shock_rebound), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="shock_hbo", .type="<class 'int'>", .kind=FieldKind::Integer,
     .offset=offsetof(DamperClickConfig, shock_hbo), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="shock_lockout", .type="<class 'bool'>", .kind=FieldKind::Boolean,
     .offset=offsetof(DamperClickConfig, shock_lockout), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="preset_name", .type="<class 'str'>", .kind=FieldKind::Text,
     .offset=offsetof(DamperClickConfig, preset_name), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kDamperClickConfigSchema{
    .path="bike_sim.physics.damper.DamperClickConfig", .fields=kDamperClickConfigFields,
    .declared_default=&kDeclaredDamperClickConfig};

} // namespace detail
} // namespace bike::configuration
