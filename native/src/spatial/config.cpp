#include "spatial/config.hpp"

#include "io/float_format.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace bike::spatial {
namespace {

using bike::io::Number;
using bike::io::Value;

const Value& require_field(const Value::Object& obj, std::string_view key,
                           const std::string& prefix) {
    const Value* member = bike::io::find_member(obj, key);
    if (member == nullptr) {
        throw std::invalid_argument(prefix + "." + std::string(key) + ": required field missing");
    }
    return *member;
}

void reject_duplicate_and_unknown_keys(const Value::Object& obj,
                                       const std::set<std::string_view>& allowed,
                                       const std::string& prefix) {
    std::set<std::string_view> seen;
    for (const auto& [key, _] : obj) {
        if (!seen.insert(key).second) {
            std::string msg = prefix;
            msg += '.';
            msg += key;
            msg += ": duplicate field";
            throw std::invalid_argument(msg);
        }
        if (!allowed.contains(key)) {
            std::string msg = prefix;
            msg += '.';
            msg += key;
            msg += ": unknown field";
            throw std::invalid_argument(msg);
        }
    }
}

Vec3 parse_vec3_or_default(const Value::Object& obj, std::string_view key, const Vec3& fallback,
                           const std::string& prefix) {
    const Value* member = bike::io::find_member(obj, key);
    if (member == nullptr) {
        return fallback;
    }
    const std::string path = prefix + "." + std::string(key);
    const Value::Array& arr = bike::io::as_array(*member, path);
    if (arr.size() != 3) {
        throw std::invalid_argument(path + ": expected 3-element real array");
    }
    return {
        bike::io::as_double(arr[0], path + "[0]"),
        bike::io::as_double(arr[1], path + "[1]"),
        bike::io::as_double(arr[2], path + "[2]"),
    };
}

Value make_number(double v, const char* path) {
    if (!std::isfinite(v)) {
        throw std::invalid_argument(std::string(path) + ": finite real number required");
    }
    Value out;
    out.data = Number{bike::io::repr_binary64(v)};
    return out;
}

Value make_string(std::string s) {
    Value out;
    out.data = std::move(s);
    return out;
}

Value make_object(Value::Object members) {
    Value out;
    out.data = std::move(members);
    return out;
}

Value vec3_to_value(const Vec3& v, const char* path) {
    Value out;
    out.data = Value::Array{
        make_number(v[0], path),
        make_number(v[1], path),
        make_number(v[2], path),
    };
    return out;
}

std::string_view backend_name(SpatialTireBackend backend) {
    switch (backend) {
    case SpatialTireBackend::compliant_3d: return "compliant_3d";
    case SpatialTireBackend::distributed_3d: return "distributed_3d";
    }
    throw std::invalid_argument("spatial.tire_backend: invalid enum value");
}

std::string_view preset_name(SpatialTirePreset preset) {
    switch (preset) {
    case SpatialTirePreset::mtb_2p8_1p2bar: return "2p8_1p2bar";
    case SpatialTirePreset::mtb_2p4_standard: return "2p4_standard";
    }
    throw std::invalid_argument("spatial.tire_preset: invalid enum value");
}

std::string_view start_mode_name(SpatialStartMode mode) {
    switch (mode) {
    case SpatialStartMode::moving: return "moving";
    case SpatialStartMode::standstill: return "standstill";
    }
    throw std::invalid_argument("spatial.start_mode: invalid enum value");
}

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (duration_s, step_s, min_ticks) matches public require_exact_ticks contract
std::int64_t require_exact_ticks(double duration_s, double step_s, std::int64_t min_ticks,
                                 const char* field_path) {
    if (!std::isfinite(duration_s) || !std::isfinite(step_s) || !(step_s > 0.)) {
        throw std::invalid_argument(std::string(field_path) +
                                    ": finite duration and positive step required");
    }
    const double ratio = duration_s / step_s;
    if (!std::isfinite(ratio) || ratio < static_cast<double>(min_ticks) - 1e-12 || ratio > 1e9) {
        throw std::invalid_argument(std::string(field_path) +
                                    ": duration below minimum ticks or out of range");
    }
    const double nearest = std::nearbyint(ratio);
    const double scale = std::max(1., std::abs(ratio));
    if (std::abs(ratio - nearest) > 1e-12 * scale) {
        throw std::invalid_argument(std::string(field_path) +
                                    ": must be an exact integer multiple of its clock period");
    }
    const auto ticks = static_cast<std::int64_t>(nearest);
    if (ticks < min_ticks) {
        throw std::invalid_argument(std::string(field_path) + ": below minimum tick count");
    }
    return ticks;
}

void validate_spatial_config(const SpatialConfig& config, double timestep_s) {
    if (!std::isfinite(timestep_s) || !(timestep_s > 0.)) {
        throw std::invalid_argument("spatial.timestep_s: positive finite timestep required");
    }
    (void)backend_name(config.tire_backend);
    (void)preset_name(config.tire_preset);
    (void)start_mode_name(config.start_mode);

    if (!std::isfinite(config.corridor_half_width_m) || !(config.corridor_half_width_m >= 0.25)) {
        throw std::invalid_argument("spatial.corridor_half_width_m: must be >= 0.25 m");
    }
    if (!std::isfinite(config.capsize_roll_rad) || !(config.capsize_roll_rad >= 0.1) ||
        !(config.capsize_roll_rad < 0.5 * std::numbers::pi - kDefaultGimbalMarginRad)) {
        throw std::invalid_argument("spatial.capsize_roll_rad: must lie in [0.1, 80 deg)");
    }
    if (!std::isfinite(config.steer.range_deg) || !(config.steer.range_deg > 0.) ||
        !(config.steer.range_deg <= 75.)) {
        throw std::invalid_argument("spatial.steer.range_deg: must lie in (0, 75] deg");
    }
    if (!std::isfinite(config.steer.max_torque_nm) || !(config.steer.max_torque_nm > 0.)) {
        throw std::invalid_argument("spatial.steer.max_torque_nm: positive torque cap required");
    }
    (void)require_exact_ticks(config.steer.control_period_s, timestep_s, 1,
                              "spatial.steer.control_period_s");
    (void)require_exact_ticks(config.steer.reaction_delay_s, config.steer.control_period_s, 0,
                              "spatial.steer.reaction_delay_s");

    for (const double v : config.imu_mount_pos_m) {
        if (!std::isfinite(v)) throw std::invalid_argument("spatial.imu_mount_pos_m: nonfinite");
    }
    for (const double v : config.imu_mount_rpy_rad) {
        if (!std::isfinite(v)) throw std::invalid_argument("spatial.imu_mount_rpy_rad: nonfinite");
    }
    for (const double v : config.imu_misalignment_rad) {
        if (!std::isfinite(v)) {
            throw std::invalid_argument("spatial.imu_misalignment_rad: nonfinite");
        }
    }

    if (config.start_mode == SpatialStartMode::standstill) {
        if (!config.start_support.has_value()) {
            throw std::invalid_argument(
                "spatial.start_support: required when spatial.start_mode == 'standstill'");
        }
        const auto& s = *config.start_support;
        if (!std::isfinite(s.release_speed_mps) || !(s.release_speed_mps >= 0.5) ||
            !(s.release_speed_mps <= 4.0)) {
            throw std::invalid_argument(
                "spatial.start_support.release_speed_mps: must lie in [0.5, 4.0]");
        }
        if (!std::isfinite(s.lateral_stiffness_n_per_m) || !(s.lateral_stiffness_n_per_m > 0.) ||
            !std::isfinite(s.lateral_damping_ns_per_m) || !(s.lateral_damping_ns_per_m >= 0.) ||
            !std::isfinite(s.roll_stiffness_nm_per_rad) || !(s.roll_stiffness_nm_per_rad >= 1500.) ||
            !std::isfinite(s.roll_damping_nms_per_rad) || !(s.roll_damping_nms_per_rad >= 0.)) {
            throw std::invalid_argument(
                "spatial.start_support: positive stiffness (roll >= 1500 N m/rad) and nonnegative damping required");
        }
    } else if (config.start_support.has_value()) {
        throw std::invalid_argument(
            "spatial.start_support: forbidden when spatial.start_mode == 'moving'");
    }
}

std::optional<SpatialConfig> parse_spatial_config(const Value::Object& root_object,
                                                  double timestep_s) {
    std::size_t spatial_key_count = 0;
    const Value* spatial_val = nullptr;
    for (const auto& [k, v] : root_object) {
        if (k == "spatial") {
            ++spatial_key_count;
            spatial_val = &v;
        }
    }
    if (spatial_key_count == 0) {
        return std::nullopt;
    }
    if (spatial_key_count > 1) {
        throw std::invalid_argument("spatial: duplicate root key");
    }

    const std::string prefix = "spatial";
    const Value::Object& obj = bike::io::as_object(*spatial_val, prefix);
    reject_duplicate_and_unknown_keys(
        obj,
        {"tire_backend", "tire_preset", "start_mode", "corridor_half_width_m", "capsize_roll_rad",
         "steer", "start_support", "imu_mount_pos_m", "imu_mount_rpy_rad", "imu_misalignment_rad"},
        prefix);

    SpatialConfig out{};
    const std::string& backend = bike::io::as_string(require_field(obj, "tire_backend", prefix),
                                                     "spatial.tire_backend");
    if (backend == "compliant_3d") {
        out.tire_backend = SpatialTireBackend::compliant_3d;
    } else if (backend == "distributed_3d") {
        out.tire_backend = SpatialTireBackend::distributed_3d;
    } else {
        throw std::invalid_argument(
            "spatial.tire_backend: expected 'compliant_3d' or 'distributed_3d'");
    }

    const std::string& preset = bike::io::as_string(require_field(obj, "tire_preset", prefix),
                                                    "spatial.tire_preset");
    if (preset == "2p8_1p2bar") {
        out.tire_preset = SpatialTirePreset::mtb_2p8_1p2bar;
    } else if (preset == "2p4_standard") {
        out.tire_preset = SpatialTirePreset::mtb_2p4_standard;
    } else {
        throw std::invalid_argument(
            "spatial.tire_preset: expected '2p8_1p2bar' or '2p4_standard'");
    }

    const std::string& mode = bike::io::as_string(require_field(obj, "start_mode", prefix),
                                                  "spatial.start_mode");
    if (mode == "moving") {
        out.start_mode = SpatialStartMode::moving;
    } else if (mode == "standstill") {
        out.start_mode = SpatialStartMode::standstill;
    } else {
        throw std::invalid_argument("spatial.start_mode: expected 'moving' or 'standstill'");
    }

    out.corridor_half_width_m = bike::io::as_double(
        require_field(obj, "corridor_half_width_m", prefix), "spatial.corridor_half_width_m");
    out.capsize_roll_rad = bike::io::as_double(
        require_field(obj, "capsize_roll_rad", prefix), "spatial.capsize_roll_rad");

    const std::string steer_prefix = "spatial.steer";
    const Value::Object& steer_obj = bike::io::as_object(
        require_field(obj, "steer", prefix), steer_prefix);
    reject_duplicate_and_unknown_keys(
        steer_obj, {"range_deg", "control_period_s", "reaction_delay_s", "max_torque_nm"},
        steer_prefix);
    out.steer.range_deg = bike::io::as_double(
        require_field(steer_obj, "range_deg", steer_prefix), "spatial.steer.range_deg");
    out.steer.control_period_s = bike::io::as_double(
        require_field(steer_obj, "control_period_s", steer_prefix),
        "spatial.steer.control_period_s");
    out.steer.reaction_delay_s = bike::io::as_double(
        require_field(steer_obj, "reaction_delay_s", steer_prefix),
        "spatial.steer.reaction_delay_s");
    out.steer.max_torque_nm = bike::io::as_double(
        require_field(steer_obj, "max_torque_nm", steer_prefix), "spatial.steer.max_torque_nm");

    if (const Value* sup_val = bike::io::find_member(obj, "start_support"); sup_val != nullptr) {
        const std::string sup_prefix = "spatial.start_support";
        const Value::Object& sup_obj = bike::io::as_object(*sup_val, sup_prefix);
        reject_duplicate_and_unknown_keys(
            sup_obj,
            {"release_speed_mps", "lateral_stiffness_n_per_m", "lateral_damping_ns_per_m",
             "roll_stiffness_nm_per_rad", "roll_damping_nms_per_rad"},
            sup_prefix);
        SpatialStartSupportSpec sup{};
        sup.release_speed_mps = bike::io::as_double(
            require_field(sup_obj, "release_speed_mps", sup_prefix),
            "spatial.start_support.release_speed_mps");
        sup.lateral_stiffness_n_per_m = bike::io::as_double(
            require_field(sup_obj, "lateral_stiffness_n_per_m", sup_prefix),
            "spatial.start_support.lateral_stiffness_n_per_m");
        sup.lateral_damping_ns_per_m = bike::io::as_double(
            require_field(sup_obj, "lateral_damping_ns_per_m", sup_prefix),
            "spatial.start_support.lateral_damping_ns_per_m");
        sup.roll_stiffness_nm_per_rad = bike::io::as_double(
            require_field(sup_obj, "roll_stiffness_nm_per_rad", sup_prefix),
            "spatial.start_support.roll_stiffness_nm_per_rad");
        sup.roll_damping_nms_per_rad = bike::io::as_double(
            require_field(sup_obj, "roll_damping_nms_per_rad", sup_prefix),
            "spatial.start_support.roll_damping_nms_per_rad");
        out.start_support = sup;
    }

    out.imu_mount_pos_m = parse_vec3_or_default(obj, "imu_mount_pos_m", {0., 0., 0.}, prefix);
    out.imu_mount_rpy_rad = parse_vec3_or_default(obj, "imu_mount_rpy_rad", {0., 0., 0.}, prefix);
    out.imu_misalignment_rad = parse_vec3_or_default(obj, "imu_misalignment_rad", {0., 0., 0.},
                                                     prefix);

    validate_spatial_config(out, timestep_s);
    return out;
}

Value spatial_config_to_value(const SpatialConfig& config) {
    Value::Object steer_obj{{
        {"range_deg", make_number(config.steer.range_deg, "spatial.steer.range_deg")},
        {"control_period_s",
         make_number(config.steer.control_period_s, "spatial.steer.control_period_s")},
        {"reaction_delay_s",
         make_number(config.steer.reaction_delay_s, "spatial.steer.reaction_delay_s")},
        {"max_torque_nm", make_number(config.steer.max_torque_nm, "spatial.steer.max_torque_nm")},
    }};

    Value::Object out{{
        {"tire_backend", make_string(std::string(backend_name(config.tire_backend)))},
        {"tire_preset", make_string(std::string(preset_name(config.tire_preset)))},
        {"start_mode", make_string(std::string(start_mode_name(config.start_mode)))},
        {"corridor_half_width_m",
         make_number(config.corridor_half_width_m, "spatial.corridor_half_width_m")},
        {"capsize_roll_rad", make_number(config.capsize_roll_rad, "spatial.capsize_roll_rad")},
        {"steer", make_object(std::move(steer_obj))},
        {"imu_mount_pos_m", vec3_to_value(config.imu_mount_pos_m, "spatial.imu_mount_pos_m")},
        {"imu_mount_rpy_rad", vec3_to_value(config.imu_mount_rpy_rad, "spatial.imu_mount_rpy_rad")},
        {"imu_misalignment_rad",
         vec3_to_value(config.imu_misalignment_rad, "spatial.imu_misalignment_rad")},
    }};

    if (config.start_support.has_value()) {
        const auto& s = *config.start_support;
        out.emplace_back(
            "start_support",
            make_object(Value::Object{{
                {"release_speed_mps",
                 make_number(s.release_speed_mps, "spatial.start_support.release_speed_mps")},
                {"lateral_stiffness_n_per_m",
                 make_number(s.lateral_stiffness_n_per_m,
                             "spatial.start_support.lateral_stiffness_n_per_m")},
                {"lateral_damping_ns_per_m",
                 make_number(s.lateral_damping_ns_per_m,
                             "spatial.start_support.lateral_damping_ns_per_m")},
                {"roll_stiffness_nm_per_rad",
                 make_number(s.roll_stiffness_nm_per_rad,
                             "spatial.start_support.roll_stiffness_nm_per_rad")},
                {"roll_damping_nms_per_rad",
                 make_number(s.roll_damping_nms_per_rad,
                             "spatial.start_support.roll_damping_nms_per_rad")},
            }}));
    }

    return make_object(std::move(out));
}

} // namespace bike::spatial
