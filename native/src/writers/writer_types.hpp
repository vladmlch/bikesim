// writer_types.hpp — shared types for the owning core's allocation-free
// interfaces (native-safety R1). Writers expose their computed forces through
// caller-owned buffers and typed component views instead of allocating return
// containers; warm-core entry points report failure through CoreStatus rather
// than exceptions so they can run inside BIKE_NONBLOCKING regions.
//
// DriveTelemetry is the fixed-lane replacement for the per-tick
// std::map<std::string, DiagnosticValue>: the serialized schema is a closed
// set of 67 keys, so every field is a compile-time-declared lane — POD
// storage for numeric/flag/closed-label values plus two capacity-managed
// strings for the open-label fields (assist_mode, coasting_reason). Presence
// bits keep the explicit-None semantics of the optional fields, and
// for_each/to_map emit the map's sorted order so the wire never changes.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

// Named producers of generalized force. Order is the stable export order for
// component telemetry at the FFI boundary; extend at the end only.
enum class ForceKind : std::uint8_t {
    human,
    motor,
    transmission,
    bearing,
    suspension,
    tire,
    rider,
    resistance
};

// A read-only view over one writer-owned force component. The pointed-to
// storage lives as long as the owning writer (construction-sized members or
// the active stage bank); views are invalidated only by the writer's next
// compute/commit, never reallocated per tick.
struct ForceComponentView {
    ForceKind kind{};
    std::span<const double> values;
};

// Fixed-width status for the allocation-free core. Convenience wrappers map
// these to the existing exception classes at the boundary; inside
// BIKE_NONBLOCKING regions no exception or string may be constructed.
enum class CoreStatus : std::uint8_t {
    ok,
    invalid_input,
    pending_actuation,
    invalid_geometry,
    engine_failure
};

namespace drivetrain {
    // Wire payload of one diagnostic field — the variant the Python boundary
    // serializes to/from. The map stays the FFI-edge representation
    // (diagnostic_dict/parse and the two-key stored_energy report); typed
    // DriveTelemetry lanes own the core state.
    using DiagnosticValue =
            std::variant<std::monostate, double, int, bool, std::string>;
    using Diagnostics = std::map<std::string, DiagnosticValue>;

    // One enumerator per serialized diagnostic key, in the wire's
    // std::map<std::string,…> iteration order so for_each()/to_map()
    // reproduce the map's emission sequence exactly.
    enum class TelemetryField : std::uint8_t {
        assist_demand_gated,
        assist_gain,
        assist_mode,
        assist_sensor_nm,
        battery_empty,
        battery_energy_j,
        cadence_rpm,
        chain_dissipation_power_w,
        chain_energy_j,
        chain_extension_m,
        chain_extension_rate_mps,
        chain_tension_n,
        coasting_reason,
        crank_clutch_dissipation_power_w,
        crank_clutch_engaged,
        crank_clutch_torque_nm,
        crank_rad_s,
        crank_target_phase_rad,
        crank_target_rate_rad_s,
        drive_shaft_rad_s,
        electrical_power_w,
        energy_limited,
        freehub_deflection_rad,
        freehub_dissipation_power_w,
        freehub_energy_j,
        freehub_engaged,
        freehub_torque_nm,
        gear_front_teeth,
        gear_ratio,
        gear_rear_teeth,
        human_command_nm,
        human_sensor_nm,
        human_setpoint_nm,
        human_torque_nm,
        motor_control_source,
        motor_enabled,
        motor_freewheel_dissipation_power_w,
        motor_freewheel_engaged,
        motor_freewheel_torque_nm,
        motor_limit_nm,
        motor_limited_request_nm,
        motor_request_nm,
        motor_setpoint_nm,
        motor_shaft_power_w,
        motor_torque_nm,
        omits_suspension_coupling,
        required_cadence_rpm,
        rider_mode,
        safety_limited,
        shift_active,
        shift_constraint_work_cumulative_j,
        shift_count,
        shift_direction,
        shift_from_teeth,
        shift_interval_constraint_work_j,
        shift_parameter_work_j,
        shift_time_s,
        shift_torque_factor,
        transmission_boundary_m,
        transmission_constraint_defect_m,
        transmission_gap_m,
        transmission_interval_work_j,
        transmission_model,
        transmission_phi_m,
        transmission_reaction_error_n,
        transmission_reference_status,
        transmission_tension_n,
        _count
    };
    inline constexpr std::size_t telemetry_field_count =
            static_cast<std::size_t>(TelemetryField::_count);

    // Lane families — real and optional share the numeric lanes (presence
    // distinguishes set/absent/explicit-None); open lanes are the two
    // profile-less label fields that accept arbitrary strings.
    enum class TelemetryKind : std::uint8_t {
        real,
        integer,
        flag,
        optional,
        label,
        open
    };

    // Closed label domains — the ordinal stored in the lane indexes this
    // table; the strings are the wire values, in declaration order.
    inline constexpr std::array<std::string_view, 3>
            transmission_model_labels{"elastic_chain", "ideal_mid_drive",
                                      "geometric_ideal_mid_drive"};
    inline constexpr std::array<std::string_view, 3>
            rider_mode_labels{"disabled", "pedaling", "coasting"};
    inline constexpr std::array<std::string_view, 2>
            motor_control_source_labels{"assist", "external_request"};
    inline constexpr std::array<std::string_view, 3>
            shift_direction_labels{"none", "up", "down"};
    inline constexpr std::array<std::string_view, 1>
            transmission_reference_status_labels{
                "experimental_geometric_reduction"};

    // One row of the schema table. `lane` indexes the kind's storage array;
    // `labels` is the closed domain for label-kind fields; `integer_min` is
    // the parse-time floor for integer-kind fields (3 for teeth counts, 0
    // for shift_count).
    struct TelemetrySpec {
        TelemetryField field;
        std::string_view name;
        TelemetryKind kind;
        std::uint8_t lane;
        std::span<const std::string_view> labels{};
        std::int32_t integer_min = 0;
    };

    inline constexpr std::array<TelemetrySpec, telemetry_field_count>
            telemetry_specs{{
                {.field = TelemetryField::assist_demand_gated,
                 .name = "assist_demand_gated",
                 .kind = TelemetryKind::flag,
                 .lane = 0},
                {.field = TelemetryField::assist_gain,
                 .name = "assist_gain",
                 .kind = TelemetryKind::real,
                 .lane = 0},
                {.field = TelemetryField::assist_mode,
                 .name = "assist_mode",
                 .kind = TelemetryKind::open,
                 .lane = 0},
                {.field = TelemetryField::assist_sensor_nm,
                 .name = "assist_sensor_nm",
                 .kind = TelemetryKind::real,
                 .lane = 1},
                {.field = TelemetryField::battery_empty,
                 .name = "battery_empty",
                 .kind = TelemetryKind::flag,
                 .lane = 1},
                {.field = TelemetryField::battery_energy_j,
                 .name = "battery_energy_j",
                 .kind = TelemetryKind::real,
                 .lane = 2},
                {.field = TelemetryField::cadence_rpm,
                 .name = "cadence_rpm",
                 .kind = TelemetryKind::real,
                 .lane = 3},
                {.field = TelemetryField::chain_dissipation_power_w,
                 .name = "chain_dissipation_power_w",
                 .kind = TelemetryKind::real,
                 .lane = 4},
                {.field = TelemetryField::chain_energy_j,
                 .name = "chain_energy_j",
                 .kind = TelemetryKind::real,
                 .lane = 5},
                {.field = TelemetryField::chain_extension_m,
                 .name = "chain_extension_m",
                 .kind = TelemetryKind::real,
                 .lane = 6},
                {.field = TelemetryField::chain_extension_rate_mps,
                 .name = "chain_extension_rate_mps",
                 .kind = TelemetryKind::real,
                 .lane = 7},
                {.field = TelemetryField::chain_tension_n,
                 .name = "chain_tension_n",
                 .kind = TelemetryKind::real,
                 .lane = 8},
                {.field = TelemetryField::coasting_reason,
                 .name = "coasting_reason",
                 .kind = TelemetryKind::open,
                 .lane = 1},
                {.field = TelemetryField::crank_clutch_dissipation_power_w,
                 .name = "crank_clutch_dissipation_power_w",
                 .kind = TelemetryKind::real,
                 .lane = 9},
                {.field = TelemetryField::crank_clutch_engaged,
                 .name = "crank_clutch_engaged",
                 .kind = TelemetryKind::flag,
                 .lane = 2},
                {.field = TelemetryField::crank_clutch_torque_nm,
                 .name = "crank_clutch_torque_nm",
                 .kind = TelemetryKind::real,
                 .lane = 10},
                {.field = TelemetryField::crank_rad_s,
                 .name = "crank_rad_s",
                 .kind = TelemetryKind::real,
                 .lane = 11},
                {.field = TelemetryField::crank_target_phase_rad,
                 .name = "crank_target_phase_rad",
                 .kind = TelemetryKind::optional,
                 .lane = 12},
                {.field = TelemetryField::crank_target_rate_rad_s,
                 .name = "crank_target_rate_rad_s",
                 .kind = TelemetryKind::real,
                 .lane = 13},
                {.field = TelemetryField::drive_shaft_rad_s,
                 .name = "drive_shaft_rad_s",
                 .kind = TelemetryKind::real,
                 .lane = 14},
                {.field = TelemetryField::electrical_power_w,
                 .name = "electrical_power_w",
                 .kind = TelemetryKind::real,
                 .lane = 15},
                {.field = TelemetryField::energy_limited,
                 .name = "energy_limited",
                 .kind = TelemetryKind::flag,
                 .lane = 3},
                {.field = TelemetryField::freehub_deflection_rad,
                 .name = "freehub_deflection_rad",
                 .kind = TelemetryKind::real,
                 .lane = 16},
                {.field = TelemetryField::freehub_dissipation_power_w,
                 .name = "freehub_dissipation_power_w",
                 .kind = TelemetryKind::real,
                 .lane = 17},
                {.field = TelemetryField::freehub_energy_j,
                 .name = "freehub_energy_j",
                 .kind = TelemetryKind::real,
                 .lane = 18},
                {.field = TelemetryField::freehub_engaged,
                 .name = "freehub_engaged",
                 .kind = TelemetryKind::flag,
                 .lane = 4},
                {.field = TelemetryField::freehub_torque_nm,
                 .name = "freehub_torque_nm",
                 .kind = TelemetryKind::real,
                 .lane = 19},
                {.field = TelemetryField::gear_front_teeth,
                 .name = "gear_front_teeth",
                 .kind = TelemetryKind::integer,
                 .lane = 0,
                 .integer_min = 3},
                {.field = TelemetryField::gear_ratio,
                 .name = "gear_ratio",
                 .kind = TelemetryKind::real,
                 .lane = 20},
                {.field = TelemetryField::gear_rear_teeth,
                 .name = "gear_rear_teeth",
                 .kind = TelemetryKind::integer,
                 .lane = 1,
                 .integer_min = 3},
                {.field = TelemetryField::human_command_nm,
                 .name = "human_command_nm",
                 .kind = TelemetryKind::real,
                 .lane = 21},
                {.field = TelemetryField::human_sensor_nm,
                 .name = "human_sensor_nm",
                 .kind = TelemetryKind::real,
                 .lane = 22},
                {.field = TelemetryField::human_setpoint_nm,
                 .name = "human_setpoint_nm",
                 .kind = TelemetryKind::optional,
                 .lane = 23},
                {.field = TelemetryField::human_torque_nm,
                 .name = "human_torque_nm",
                 .kind = TelemetryKind::real,
                 .lane = 24},
                {.field = TelemetryField::motor_control_source,
                 .name = "motor_control_source",
                 .kind = TelemetryKind::label,
                 .lane = 0,
                 .labels = motor_control_source_labels},
                {.field = TelemetryField::motor_enabled,
                 .name = "motor_enabled",
                 .kind = TelemetryKind::flag,
                 .lane = 5},
                {.field = TelemetryField::motor_freewheel_dissipation_power_w,
                 .name = "motor_freewheel_dissipation_power_w",
                 .kind = TelemetryKind::real,
                 .lane = 25},
                {.field = TelemetryField::motor_freewheel_engaged,
                 .name = "motor_freewheel_engaged",
                 .kind = TelemetryKind::flag,
                 .lane = 6},
                {.field = TelemetryField::motor_freewheel_torque_nm,
                 .name = "motor_freewheel_torque_nm",
                 .kind = TelemetryKind::real,
                 .lane = 26},
                {.field = TelemetryField::motor_limit_nm,
                 .name = "motor_limit_nm",
                 .kind = TelemetryKind::optional,
                 .lane = 27},
                {.field = TelemetryField::motor_limited_request_nm,
                 .name = "motor_limited_request_nm",
                 .kind = TelemetryKind::real,
                 .lane = 28},
                {.field = TelemetryField::motor_request_nm,
                 .name = "motor_request_nm",
                 .kind = TelemetryKind::real,
                 .lane = 29},
                {.field = TelemetryField::motor_setpoint_nm,
                 .name = "motor_setpoint_nm",
                 .kind = TelemetryKind::optional,
                 .lane = 30},
                {.field = TelemetryField::motor_shaft_power_w,
                 .name = "motor_shaft_power_w",
                 .kind = TelemetryKind::real,
                 .lane = 31},
                {.field = TelemetryField::motor_torque_nm,
                 .name = "motor_torque_nm",
                 .kind = TelemetryKind::real,
                 .lane = 32},
                {.field = TelemetryField::omits_suspension_coupling,
                 .name = "omits_suspension_coupling",
                 .kind = TelemetryKind::flag,
                 .lane = 7},
                {.field = TelemetryField::required_cadence_rpm,
                 .name = "required_cadence_rpm",
                 .kind = TelemetryKind::real,
                 .lane = 33},
                {.field = TelemetryField::rider_mode,
                 .name = "rider_mode",
                 .kind = TelemetryKind::label,
                 .lane = 1,
                 .labels = rider_mode_labels},
                {.field = TelemetryField::safety_limited,
                 .name = "safety_limited",
                 .kind = TelemetryKind::flag,
                 .lane = 8},
                {.field = TelemetryField::shift_active,
                 .name = "shift_active",
                 .kind = TelemetryKind::flag,
                 .lane = 9},
                {.field = TelemetryField::shift_constraint_work_cumulative_j,
                 .name = "shift_constraint_work_cumulative_j",
                 .kind = TelemetryKind::real,
                 .lane = 34},
                {.field = TelemetryField::shift_count,
                 .name = "shift_count",
                 .kind = TelemetryKind::integer,
                 .lane = 2,
                 .integer_min = 0},
                {.field = TelemetryField::shift_direction,
                 .name = "shift_direction",
                 .kind = TelemetryKind::label,
                 .lane = 2,
                 .labels = shift_direction_labels},
                {.field = TelemetryField::shift_from_teeth,
                 .name = "shift_from_teeth",
                 .kind = TelemetryKind::integer,
                 .lane = 3,
                 .integer_min = 3},
                {.field = TelemetryField::shift_interval_constraint_work_j,
                 .name = "shift_interval_constraint_work_j",
                 .kind = TelemetryKind::real,
                 .lane = 35},
                {.field = TelemetryField::shift_parameter_work_j,
                 .name = "shift_parameter_work_j",
                 .kind = TelemetryKind::real,
                 .lane = 36},
                {.field = TelemetryField::shift_time_s,
                 .name = "shift_time_s",
                 .kind = TelemetryKind::optional,
                 .lane = 37},
                {.field = TelemetryField::shift_torque_factor,
                 .name = "shift_torque_factor",
                 .kind = TelemetryKind::real,
                 .lane = 38},
                {.field = TelemetryField::transmission_boundary_m,
                 .name = "transmission_boundary_m",
                 .kind = TelemetryKind::real,
                 .lane = 39},
                {.field = TelemetryField::transmission_constraint_defect_m,
                 .name = "transmission_constraint_defect_m",
                 .kind = TelemetryKind::real,
                 .lane = 40},
                {.field = TelemetryField::transmission_gap_m,
                 .name = "transmission_gap_m",
                 .kind = TelemetryKind::real,
                 .lane = 41},
                {.field = TelemetryField::transmission_interval_work_j,
                 .name = "transmission_interval_work_j",
                 .kind = TelemetryKind::real,
                 .lane = 42},
                {.field = TelemetryField::transmission_model,
                 .name = "transmission_model",
                 .kind = TelemetryKind::label,
                 .lane = 3,
                 .labels = transmission_model_labels},
                {.field = TelemetryField::transmission_phi_m,
                 .name = "transmission_phi_m",
                 .kind = TelemetryKind::real,
                 .lane = 43},
                {.field = TelemetryField::transmission_reaction_error_n,
                 .name = "transmission_reaction_error_n",
                 .kind = TelemetryKind::real,
                 .lane = 44},
                {.field = TelemetryField::transmission_reference_status,
                 .name = "transmission_reference_status",
                 .kind = TelemetryKind::label,
                 .lane = 4,
                 .labels = transmission_reference_status_labels},
                {.field = TelemetryField::transmission_tension_n,
                 .name = "transmission_tension_n",
                 .kind = TelemetryKind::real,
                 .lane = 45}
            }};

    // Compile-time pin: every schema row must sit at its enum position, so
    // a reordered row is a build failure rather than a silent wire change.
    // The enum member names were transcribed from the wire names — the
    // parse round-trip tests cover the full name mapping.
    consteval bool telemetry_table_ordered() {
        for (std::size_t i = 0; i < telemetry_specs.size(); ++i)
            if (telemetry_specs[i].field != static_cast<TelemetryField>(i))
                return false;
        return true;
    }
    static_assert(telemetry_table_ordered(),
                  "telemetry rows must sit at their enum positions");

    [[nodiscard]] constexpr const TelemetrySpec &
    telemetry_spec(TelemetryField field) noexcept {
        return telemetry_specs[static_cast<std::size_t>(field)];
    }

    [[nodiscard]] constexpr std::optional<TelemetryField>
    telemetry_field(std::string_view name) noexcept {
        for (const TelemetrySpec &spec: telemetry_specs)
            if (spec.name == name)
                return spec.field;
        return std::nullopt;
    }

    // Closed-label lookup: the stored ordinal indexes the field's declared
    // label table. Returns nullopt for a label outside the domain or a
    // non-label field.
    [[nodiscard]] constexpr std::optional<std::uint8_t>
    telemetry_label(TelemetryField field, std::string_view name) noexcept {
        const TelemetrySpec &spec = telemetry_spec(field);
        if (spec.kind != TelemetryKind::label)
            return std::nullopt;
        for (std::size_t i = 0; i < spec.labels.size(); ++i)
            if (spec.labels[i] == name)
                return static_cast<std::uint8_t>(i);
        return std::nullopt;
    }

    [[nodiscard]] constexpr std::string_view
    telemetry_label_name(TelemetryField field, std::uint8_t ordinal) noexcept {
        const TelemetrySpec &spec = telemetry_spec(field);
        if (spec.kind != TelemetryKind::label ||
            ordinal >= spec.labels.size())
            return {};
        return spec.labels[ordinal];
    }

    // Scalar payload of one staged write / for_each emission. string_view
    // points into owning storage (a telemetry lane or the label table) —
    // never detached; uint8_t is a closed-label ordinal.
    using TelemetryScalar =
            std::variant<std::monostate, double, std::int32_t, bool,
                         std::uint8_t, std::string_view>;

    // Fixed-lane diagnostic container: per-field presence bits (0 absent,
    // 1 set, 2 explicit-None) over POD lanes plus two capacity-managed
    // open-label strings. clear()/clear_field() flip presence only — lane
    // contents and string capacity are retained, so a per-tick rewrite is
    // allocation-free after warmup.
    class DriveTelemetry {
    public:
        [[nodiscard]] bool empty() const noexcept {
            for (const std::uint8_t presence: presence_)
                if (presence != 0)
                    return false;
            return true;
        }

        void clear() noexcept { presence_.fill(0); }

        // 0 absent, 1 value present, 2 explicit-None (wire: null).
        [[nodiscard]] std::uint8_t presence(TelemetryField field) const
                noexcept {
            return presence_[static_cast<std::size_t>(field)];
        }

        [[nodiscard]] bool present(TelemetryField field) const noexcept {
            return presence(field) != 0;
        }

        [[nodiscard]] bool present(std::string_view name) const noexcept {
            const std::optional<TelemetryField> field = telemetry_field(name);
            return field && present(*field);
        }

        void clear_field(TelemetryField field) noexcept {
            presence_[static_cast<std::size_t>(field)] = 0;
        }

        // Stage-side writes — checked and throwing like the map assignment
        // they replaced; a kind mismatch is a caller bug reported as
        // std::invalid_argument. set_open may allocate while growing the
        // lane's capacity; steady-state writes under an established
        // capacity allocate nothing.
        void set(TelemetryField field, double value) {
            const TelemetrySpec &spec = checked(field, TelemetryKind::real,
                                                TelemetryKind::optional);
            numeric_[spec.lane] = value;
            presence_[static_cast<std::size_t>(field)] = 1;
        }
        void set(TelemetryField field, std::int32_t value) {
            const TelemetrySpec &spec =
                    checked(field, TelemetryKind::integer);
            integer_[spec.lane] = value;
            presence_[static_cast<std::size_t>(field)] = 1;
        }
        void set(TelemetryField field, bool value) {
            const TelemetrySpec &spec = checked(field, TelemetryKind::flag);
            flag_[spec.lane] = value;
            presence_[static_cast<std::size_t>(field)] = 1;
        }
        void set_label(TelemetryField field, std::uint8_t ordinal) {
            const TelemetrySpec &spec = checked(field, TelemetryKind::label);
            label_[spec.lane] = ordinal;
            presence_[static_cast<std::size_t>(field)] = 1;
        }
        void set_open(TelemetryField field, std::string_view value) {
            const TelemetrySpec &spec = checked(field, TelemetryKind::open);
            open_[spec.lane].assign(value.data(), value.size());
            presence_[static_cast<std::size_t>(field)] = 1;
        }
        void set_none(TelemetryField field) noexcept {
            presence_[static_cast<std::size_t>(field)] = 2;
        }
        void set_optional(TelemetryField field,
                          std::optional<double> value) {
            value ? set(field, *value) : set_none(field);
        }

        // Commit-side preflight for a string_view write: reserves the open
        // lane so the later apply() assigns within existing capacity.
        void ensure_open_capacity(TelemetryField field, std::size_t size) {
            const TelemetrySpec &spec = checked(field, TelemetryKind::open);
            if (open_[spec.lane].capacity() < size)
                open_[spec.lane].reserve(size);
        }

        // Commit-time lane write from a staged scalar — noexcept: the
        // staging contract guarantees the alternative matches the field
        // kind and open-lane capacity was reserved via
        // ensure_open_capacity(). A violation is a programming error.
        void apply(TelemetryField field, // NOLINT(bugprone-exception-escape) open-lane assign() cannot allocate under the capacity ensure_open_capacity() reserved at stage time — a violation is a programming error.
                   const TelemetryScalar &value) noexcept {
            const TelemetrySpec &spec = telemetry_spec(field);
            const auto index = static_cast<std::size_t>(field);
            std::visit(
                [&](const auto &v) {
                    using T = std::decay_t<decltype(v)>;
                    if constexpr (std::is_same_v<T, std::monostate>) {
                        presence_[index] = 2;
                    } else if constexpr (std::is_same_v<T, double>) {
                        numeric_[spec.lane] = v;
                        presence_[index] = 1;
                    } else if constexpr (std::is_same_v<T, std::int32_t>) {
                        integer_[spec.lane] = v;
                        presence_[index] = 1;
                    } else if constexpr (std::is_same_v<T, bool>) {
                        flag_[spec.lane] = v;
                        presence_[index] = 1;
                    } else if constexpr (std::is_same_v<T, std::uint8_t>) {
                        label_[spec.lane] = v;
                        presence_[index] = 1;
                    } else {
                        const auto &view =
                                std::get<std::string_view>(value);
                        open_[spec.lane].assign(view.data(), view.size());
                        presence_[index] = 1;
                    }
                },
                value);
        }

        // Boundary read — materializes the wire value, allocating on
        // string/None materialization. Throws std::out_of_range on an
        // absent field, matching std::map::at.
        [[nodiscard]] DiagnosticValue at(std::string_view name) const {
            const std::optional<TelemetryField> field = telemetry_field(name);
            if (!field)
                throw std::out_of_range("diagnostic key");
            const TelemetrySpec &spec = telemetry_spec(*field);
            switch (presence(*field)) {
            case 0:
                throw std::out_of_range("diagnostic key");
            case 2:
                return std::monostate{};
            default:
                break;
            }
            switch (spec.kind) {
            case TelemetryKind::real:
            case TelemetryKind::optional:
                return numeric_[spec.lane];
            case TelemetryKind::integer:
                return static_cast<int>(integer_[spec.lane]);
            case TelemetryKind::flag:
                return flag_[spec.lane];
            case TelemetryKind::label:
                return std::string(
                    telemetry_label_name(*field, label_[spec.lane]));
            case TelemetryKind::open:
                return open_[spec.lane];
            }
            return std::monostate{};
        }

        // Emit (TelemetryField, TelemetryScalar) for every present field —
        // including explicit-Nones as monostate — in spec order. The scalar
        // borrows lane storage; do not hold it past the next mutation.
        template<class F>
        void for_each(const F &f) const {
            for (const TelemetrySpec &spec: telemetry_specs) {
                const std::uint8_t state =
                        presence_[static_cast<std::size_t>(spec.field)];
                if (state == 0)
                    continue;
                TelemetryScalar value;
                if (state == 2) {
                    value = std::monostate{};
                } else {
                    switch (spec.kind) {
                    case TelemetryKind::real:
                    case TelemetryKind::optional:
                        value = numeric_[spec.lane];
                        break;
                    case TelemetryKind::integer:
                        value = integer_[spec.lane];
                        break;
                    case TelemetryKind::flag:
                        value = flag_[spec.lane];
                        break;
                    case TelemetryKind::label:
                        value = label_[spec.lane];
                        break;
                    case TelemetryKind::open:
                        value = std::string_view(open_[spec.lane]);
                        break;
                    }
                }
                f(spec.field, value);
            }
        }

        // Materialize the wire map — serialization/parse boundary only;
        // allocates like any map build.
        [[nodiscard]] Diagnostics to_map() const {
            Diagnostics out;
            for_each([&](TelemetryField field, const TelemetryScalar &value) {
                const TelemetrySpec &spec = telemetry_spec(field);
                out[std::string(spec.name)] = std::visit(
                    [&](const auto &v) -> DiagnosticValue {
                        using T = std::decay_t<decltype(v)>;
                        if constexpr (std::is_same_v<T, std::monostate>)
                            return std::monostate{};
                        else if constexpr (std::is_same_v<T, std::int32_t>)
                            return static_cast<int>(v);
                        else if constexpr (std::is_same_v<T, std::uint8_t>)
                            return std::string(
                                telemetry_label_name(field, v));
                        else if constexpr (std::is_same_v<T, std::string_view>)
                            return std::string(v);
                        else
                            return v;
                    },
                    value);
            });
            return out;
        }

    private:
        [[nodiscard]] const TelemetrySpec &
        checked(TelemetryField field, TelemetryKind kind) const {
            const TelemetrySpec &spec = telemetry_spec(field);
            if (spec.kind != kind)
                throw std::invalid_argument("telemetry field kind");
            return spec;
        }
        [[nodiscard]] const TelemetrySpec &
        checked(TelemetryField field, TelemetryKind first,
                TelemetryKind second) const {
            const TelemetrySpec &spec = telemetry_spec(field);
            if (spec.kind != first && spec.kind != second)
                throw std::invalid_argument("telemetry field kind");
            return spec;
        }

        std::array<double, 46> numeric_{};
        std::array<std::int32_t, 4> integer_{};
        std::array<bool, 10> flag_{};
        std::array<std::uint8_t, 5> label_{};
        std::array<std::string, 2> open_{};
        std::array<std::uint8_t, telemetry_field_count> presence_{};
    };
    static_assert(std::is_nothrow_move_assignable_v<DriveTelemetry> &&
                  std::is_nothrow_move_constructible_v<DriveTelemetry>,
                  "telemetry swaps into place without allocation");
} // namespace drivetrain
