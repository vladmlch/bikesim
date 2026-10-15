// configuration/model_config.hpp — T12 port of model_config.py
// (EndStopConfig, SimulationPhysicsConfig) plus the public configuration
// boundary (api-index T12):
//
//   SimulationPhysicsConfig default_physics();
//   void validate(const SimulationPhysicsConfig& config);
//   bike::io::Value schema();
//
// `to_value` is the dataclasses.asdict + canonical-codec equivalent used by
// the resolved-profile fixtures and round-trip checks. `post_init` is the
// construction-time normalize+validate the Python frozen dataclasses run in
// __post_init__; `validate` applies it to a copy for a non-mutating check.
#pragma once

#include <optional>
#include <string>

#include "fields.hpp"
#include "physical_config.hpp"
#include "../io/value.hpp"
#include "../rider/intent.hpp"

namespace bike::configuration {

// model_config.py EndStopConfig — frozen record, five declared defaults.
struct EndStopConfig {
    double reference_force_n = 7000.;
    double reference_deflection_m = .010;
    double damping_n_s_m = 500.;
    double overtravel_m = .010;
    std::string provenance = "synthetic";

    // EndStopConfig.stiffness_n_m (derived, overflow-checked).
    [[nodiscard]] double stiffness_n_m() const;
};

// seated_climb.py SeatedClimbConfig — the identical record is already the
// native rider/intent.hpp IntentConfig (same 19 fields, same defaults);
// reuse keeps one declaration for the input and the intent boundary.
using SeatedClimbConfig = ::rider::IntentConfig;

// model_config.py SimulationPhysicsConfig — 19 fields; drive_mode/pitch_assist
// declared defaults are None (nullopt), resolved by post_init.
struct SimulationPhysicsConfig {
    std::string physics_mode = "legacy";
    std::optional<std::string> drive_mode;
    double timestep_s = 0.0005;
    double control_period_s = 0.005;
    std::optional<bool> pitch_assist;
    EndStopConfig end_stops;
    TireBackendConfig tires;
    PhysicalDriveConfig drive;
    ResistanceConfig resistance;
    ArticulatedConfig articulated;
    SeatedClimbConfig seated_climb;
    double initial_speed_mps = 0.;
    double initial_front_brake = 0.;
    double initial_rear_brake = 0.;
    double closure_time_constant_s = .001;
    double equilibrium_refine_after_s = 1.;
    double equilibrium_refine_period_s = 3.;
    bool equilibrium_cache_enabled = false;
    std::optional<std::string> equilibrium_state_file;
};

// ---- public boundary -----------------------------------------------------------

// SimulationPhysicsConfig() — declared defaults after __post_init__ (legacy
// mode resolves drive_mode="ideal_speed_control", pitch_assist=true).
[[nodiscard]] SimulationPhysicsConfig default_physics();

// Python __post_init__: normalize in place (drive_mode/pitch_assist
// resolution, cassette sort, 'weld'->'connect' grip alias) and throw
// std::invalid_argument carrying the source message on any violation.
void post_init(SimulationPhysicsConfig& config);

// Non-mutating domain check: runs post_init on a copy.
void validate(const SimulationPhysicsConfig& config);

// The native/schema.py config_schema() equivalent: the dataclass
// introspection document {config_class, fields:[...]}.
[[nodiscard]] bike::io::Value schema();

// dataclasses.asdict + encode_tree equivalent: nested records become plain
// objects, tuple members carry the {"$tuple": [...]} tag, optionals keep
// null, and number lexemes use Python repr spelling.
[[nodiscard]] bike::io::Value to_value(const SimulationPhysicsConfig& config);

// ---- native/schema.py resolved-wire boundary (schema.cpp) ------------------
// The dynamic validator/coercer over a decoded bike::io::Value document:
// SCHEMAS key sets, integer/bool/sequence/string/scalar dispatch, the
// backend presence rules and post-validation plain() canonicalization.
// Violations throw std::invalid_argument carrying the Python message text.
namespace wire {
void validate_config(const bike::io::Value& config);
void validate_drive_policies(const bike::io::Value& config);
void validate_policy_modes(const bike::io::Value& drive,
                           std::string_view path);
void validate_rider_contacts(
    const bike::io::Value& contacts,
    std::string_view path = "config.rider_contacts");
// plain(value, key) — Python's default key is None; the empty view plays
// that sentinel (it is never a member of the INTEGER/cassette sets).
[[nodiscard]] bike::io::Value plain(const bike::io::Value& value,
                                    std::string_view key = {});
}  // namespace wire

namespace detail {
void post_init(const EndStopConfig&);
void post_init(const SeatedClimbConfig&);
} // namespace detail

// ---- field tables ---------------------------------------------------------------
namespace detail {

inline const EndStopConfig kDeclaredEndStopConfig{};
inline constexpr FieldDecl kEndStopConfigFields[] = {
    {"reference_force_n", "<class 'float'>", FieldKind::Real,
     offsetof(EndStopConfig, reference_force_n), false, "", nullptr,
     nullptr},
    {"reference_deflection_m", "<class 'float'>", FieldKind::Real,
     offsetof(EndStopConfig, reference_deflection_m), false, "", nullptr,
     nullptr},
    {"damping_n_s_m", "<class 'float'>", FieldKind::Real,
     offsetof(EndStopConfig, damping_n_s_m), false, "", nullptr, nullptr},
    {"overtravel_m", "<class 'float'>", FieldKind::Real,
     offsetof(EndStopConfig, overtravel_m), false, "", nullptr, nullptr},
    {"provenance", "<class 'str'>", FieldKind::Text,
     offsetof(EndStopConfig, provenance), false, "", nullptr, nullptr},
};
inline const RecordSchema kEndStopConfigSchema{
    "bike_sim.physics.model_config.EndStopConfig", kEndStopConfigFields,
    &kDeclaredEndStopConfig};

// seated_climb.py SeatedClimbConfig — FieldDecls over the shared
// rider::IntentConfig record (the aliased type above).
inline const SeatedClimbConfig kDeclaredSeatedClimbConfig{};
inline constexpr FieldDecl kSeatedClimbConfigFields[] = {
#define SC_REAL(f)                                                            \
    {#f, "<class 'float'>", FieldKind::Real, offsetof(SeatedClimbConfig, f), \
     false, "", nullptr, nullptr}
    {"enabled", "<class 'bool'>", FieldKind::Boolean,
     offsetof(SeatedClimbConfig, enabled), false, "", nullptr, nullptr},
    SC_REAL(period_s),
    SC_REAL(reaction_delay_s),
    SC_REAL(target_crank_power_w),
    SC_REAL(max_crank_torque_nm),
    SC_REAL(torque_slew_nm_s),
    SC_REAL(lean_gain),
    SC_REAL(max_forward_lean_rad),
    SC_REAL(max_backward_lean_rad),
    SC_REAL(lean_rate_rad_s),
    SC_REAL(orientation_tau_s),
    SC_REAL(surge_power_w),
    SC_REAL(surge_grade),
    SC_REAL(surge_budget_s),
    SC_REAL(surge_recovery_rate),
    SC_REAL(front_load_share_target),
    SC_REAL(lean_trim_gain_rad_s),
    SC_REAL(lean_trim_limit_rad),
    SC_REAL(trim_dead_time_s),
#undef SC_REAL
};
inline const RecordSchema kSeatedClimbConfigSchema{
    "bike_sim.physics.seated_climb.SeatedClimbConfig",
    kSeatedClimbConfigFields, &kDeclaredSeatedClimbConfig};

inline const SimulationPhysicsConfig kDeclaredSimulationPhysicsConfig{};
inline constexpr FieldDecl kSimulationPhysicsConfigFields[] = {
    {"physics_mode", "<class 'str'>", FieldKind::Text,
     offsetof(SimulationPhysicsConfig, physics_mode), false, "", nullptr,
     nullptr},
    {"drive_mode", "str | None", FieldKind::OptionalText,
     offsetof(SimulationPhysicsConfig, drive_mode), false, "", nullptr,
     nullptr},
    {"timestep_s", "<class 'float'>", FieldKind::Real,
     offsetof(SimulationPhysicsConfig, timestep_s), false, "", nullptr,
     nullptr},
    {"control_period_s", "<class 'float'>", FieldKind::Real,
     offsetof(SimulationPhysicsConfig, control_period_s), false, "", nullptr,
     nullptr},
    {"pitch_assist", "bool | None", FieldKind::OptionalBoolean,
     offsetof(SimulationPhysicsConfig, pitch_assist), false, "", nullptr,
     nullptr},
    {"end_stops", "<class 'bike_sim.physics.model_config.EndStopConfig'>",
     FieldKind::Dataclass, offsetof(SimulationPhysicsConfig, end_stops),
     false, "EndStopConfig", &kEndStopConfigSchema, nullptr},
    {"tires",
     "<class 'bike_sim.physics.physical_config.TireBackendConfig'>",
     FieldKind::Dataclass, offsetof(SimulationPhysicsConfig, tires), false,
     "TireBackendConfig", &kTireBackendConfigSchema, nullptr},
    {"drive",
     "<class 'bike_sim.physics.physical_config.PhysicalDriveConfig'>",
     FieldKind::Dataclass, offsetof(SimulationPhysicsConfig, drive), false,
     "PhysicalDriveConfig", &kPhysicalDriveConfigSchema, nullptr},
    {"resistance",
     "<class 'bike_sim.physics.physical_config.ResistanceConfig'>",
     FieldKind::Dataclass, offsetof(SimulationPhysicsConfig, resistance),
     false, "ResistanceConfig", &kResistanceConfigSchema, nullptr},
    {"articulated",
     "<class 'bike_sim.physics.physical_config.ArticulatedConfig'>",
     FieldKind::Dataclass, offsetof(SimulationPhysicsConfig, articulated),
     false, "ArticulatedConfig", &kArticulatedConfigSchema, nullptr},
    {"seated_climb",
     "<class 'bike_sim.physics.seated_climb.SeatedClimbConfig'>",
     FieldKind::Dataclass, offsetof(SimulationPhysicsConfig, seated_climb),
     false, "SeatedClimbConfig", &kSeatedClimbConfigSchema, nullptr},
    {"initial_speed_mps", "<class 'float'>", FieldKind::Real,
     offsetof(SimulationPhysicsConfig, initial_speed_mps), false, "",
     nullptr, nullptr},
    {"initial_front_brake", "<class 'float'>", FieldKind::Real,
     offsetof(SimulationPhysicsConfig, initial_front_brake), false, "",
     nullptr, nullptr},
    {"initial_rear_brake", "<class 'float'>", FieldKind::Real,
     offsetof(SimulationPhysicsConfig, initial_rear_brake), false, "",
     nullptr, nullptr},
    {"closure_time_constant_s", "<class 'float'>", FieldKind::Real,
     offsetof(SimulationPhysicsConfig, closure_time_constant_s), false, "",
     nullptr, nullptr},
    {"equilibrium_refine_after_s", "<class 'float'>", FieldKind::Real,
     offsetof(SimulationPhysicsConfig, equilibrium_refine_after_s), false,
     "", nullptr, nullptr},
    {"equilibrium_refine_period_s", "<class 'float'>", FieldKind::Real,
     offsetof(SimulationPhysicsConfig, equilibrium_refine_period_s), false,
     "", nullptr, nullptr},
    {"equilibrium_cache_enabled", "<class 'bool'>", FieldKind::Boolean,
     offsetof(SimulationPhysicsConfig, equilibrium_cache_enabled), false,
     "", nullptr, nullptr},
    {"equilibrium_state_file", "str | None", FieldKind::OptionalText,
     offsetof(SimulationPhysicsConfig, equilibrium_state_file), false, "",
     nullptr, nullptr},
};
inline const RecordSchema kSimulationPhysicsConfigSchema{
    "bike_sim.physics.model_config.SimulationPhysicsConfig",
    kSimulationPhysicsConfigFields, &kDeclaredSimulationPhysicsConfig};

} // namespace detail
} // namespace bike::configuration
