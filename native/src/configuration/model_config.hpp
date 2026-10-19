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
#include <array>

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

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const EndStopConfig kDeclaredEndStopConfig{};
inline constexpr auto kEndStopConfigFields = std::to_array<FieldDecl>({
    {.name="reference_force_n", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(EndStopConfig, reference_force_n), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="reference_deflection_m", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(EndStopConfig, reference_deflection_m), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="damping_n_s_m", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(EndStopConfig, damping_n_s_m), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="overtravel_m", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(EndStopConfig, overtravel_m), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="provenance", .type="<class 'str'>", .kind=FieldKind::Text,
     .offset=offsetof(EndStopConfig, provenance), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kEndStopConfigSchema{
    .path="bike_sim.physics.model_config.EndStopConfig", .fields=kEndStopConfigFields,
    .declared_default=&kDeclaredEndStopConfig};

// seated_climb.py SeatedClimbConfig — FieldDecls over the shared
// rider::IntentConfig record (the aliased type above).
inline const SeatedClimbConfig kDeclaredSeatedClimbConfig{};
inline constexpr auto kSeatedClimbConfigFields = std::to_array<FieldDecl>({
#define SC_REAL(f)                                                            \
    {#f, "<class 'float'>", FieldKind::Real, offsetof(SeatedClimbConfig, f), \
     false, "", nullptr, nullptr}
    {.name="enabled", .type="<class 'bool'>", .kind=FieldKind::Boolean,
     .offset=offsetof(SeatedClimbConfig, enabled), .required=false, .factory="", .nested=nullptr, .nested2=nullptr},
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
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kSeatedClimbConfigSchema{
    .path="bike_sim.physics.seated_climb.SeatedClimbConfig",
    .fields=kSeatedClimbConfigFields, .declared_default=&kDeclaredSeatedClimbConfig};

// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const SimulationPhysicsConfig kDeclaredSimulationPhysicsConfig{};
inline constexpr auto kSimulationPhysicsConfigFields = std::to_array<FieldDecl>({
    {.name="physics_mode", .type="<class 'str'>", .kind=FieldKind::Text,
     .offset=offsetof(SimulationPhysicsConfig, physics_mode), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="drive_mode", .type="str | None", .kind=FieldKind::OptionalText,
     .offset=offsetof(SimulationPhysicsConfig, drive_mode), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="timestep_s", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(SimulationPhysicsConfig, timestep_s), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="control_period_s", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(SimulationPhysicsConfig, control_period_s), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="pitch_assist", .type="bool | None", .kind=FieldKind::OptionalBoolean,
     .offset=offsetof(SimulationPhysicsConfig, pitch_assist), .required=false, .factory="", .nested=nullptr,
     .nested2=nullptr},
    {.name="end_stops", .type="<class 'bike_sim.physics.model_config.EndStopConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(SimulationPhysicsConfig, end_stops),
     .required=false, .factory="EndStopConfig", .nested=&kEndStopConfigSchema, .nested2=nullptr},
    {.name="tires",
     .type="<class 'bike_sim.physics.physical_config.TireBackendConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(SimulationPhysicsConfig, tires), .required=false,
     .factory="TireBackendConfig", .nested=&kTireBackendConfigSchema, .nested2=nullptr},
    {.name="drive",
     .type="<class 'bike_sim.physics.physical_config.PhysicalDriveConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(SimulationPhysicsConfig, drive), .required=false,
     .factory="PhysicalDriveConfig", .nested=&kPhysicalDriveConfigSchema, .nested2=nullptr},
    {.name="resistance",
     .type="<class 'bike_sim.physics.physical_config.ResistanceConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(SimulationPhysicsConfig, resistance),
     .required=false, .factory="ResistanceConfig", .nested=&kResistanceConfigSchema, .nested2=nullptr},
    {.name="articulated",
     .type="<class 'bike_sim.physics.physical_config.ArticulatedConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(SimulationPhysicsConfig, articulated),
     .required=false, .factory="ArticulatedConfig", .nested=&kArticulatedConfigSchema, .nested2=nullptr},
    {.name="seated_climb",
     .type="<class 'bike_sim.physics.seated_climb.SeatedClimbConfig'>",
     .kind=FieldKind::Dataclass, .offset=offsetof(SimulationPhysicsConfig, seated_climb),
     .required=false, .factory="SeatedClimbConfig", .nested=&kSeatedClimbConfigSchema, .nested2=nullptr},
    {.name="initial_speed_mps", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(SimulationPhysicsConfig, initial_speed_mps), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="initial_front_brake", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(SimulationPhysicsConfig, initial_front_brake), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="initial_rear_brake", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(SimulationPhysicsConfig, initial_rear_brake), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="closure_time_constant_s", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(SimulationPhysicsConfig, closure_time_constant_s), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
    {.name="equilibrium_refine_after_s", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(SimulationPhysicsConfig, equilibrium_refine_after_s), .required=false,
     .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="equilibrium_refine_period_s", .type="<class 'float'>", .kind=FieldKind::Real,
     .offset=offsetof(SimulationPhysicsConfig, equilibrium_refine_period_s), .required=false,
     .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="equilibrium_cache_enabled", .type="<class 'bool'>", .kind=FieldKind::Boolean,
     .offset=offsetof(SimulationPhysicsConfig, equilibrium_cache_enabled), .required=false,
     .factory="", .nested=nullptr, .nested2=nullptr},
    {.name="equilibrium_state_file", .type="str | None", .kind=FieldKind::OptionalText,
     .offset=offsetof(SimulationPhysicsConfig, equilibrium_state_file), .required=false, .factory="",
     .nested=nullptr, .nested2=nullptr},
});
// NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) schema/config tables are built once at startup; a throw is a fatal defect there
inline const RecordSchema kSimulationPhysicsConfigSchema{
    .path="bike_sim.physics.model_config.SimulationPhysicsConfig",
    .fields=kSimulationPhysicsConfigFields, .declared_default=&kDeclaredSimulationPhysicsConfig};

} // namespace detail
} // namespace bike::configuration
