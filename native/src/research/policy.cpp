#include "policy.hpp"
#include "../diag.hpp"
#include "../io/json.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <cmath>
#include <iterator>

namespace bike::research {
PolicyFailure::~PolicyFailure() = default;
io::Value Policy::metadata() const {
    using namespace runtime::setupvalue;
    return Value{Object{{"reference", Value{}}, {"class", text("native::Policy")},
        {"source_file", Value{}}, {"source_sha256", Value{}}}};
}
void validate_policy_input(const runtime::SensorObservation& observation, std::optional<double> demand) {
    observation.validate();
    if (demand && (!std::isfinite(*demand) || *demand < 0.))
        throw std::invalid_argument("policy demand must be finite and nonnegative when present");
}
void validate_motor_policy_control(const runtime::RideControl& control) {
    runtime::control_validate_for(control, true);
    if (control.human_torque_nm || control.posture || control.crank_target_rate_rad_s || !control.rider_enabled)
        throw std::invalid_argument("motor policy cannot own rider inputs");
}
std::unique_ptr<Policy> load_policy(std::string_view specification) {
    return load_policy(specification, io::Value{io::Value::Object{}});
}
std::unique_ptr<Policy> load_policy(std::string_view specification, const io::Value& config) {
    (void)io::as_object(config, "policy configuration");
    if (specification.starts_with("plugin:"))
        return load_plugin_policy(specification.substr(7), config);
    if (auto builtin = make_builtin_policy(specification, config)) return builtin;
    throw std::invalid_argument("unknown native policy '" + std::string(specification) +
        "': use a supported builtin:name or plugin:/absolute/library path; Python module factories require a native port");
}
void validate_policy_abi(const BikePolicyV1* api) {
    if (api == nullptr) throw PolicyFailure("policy entrypoint returned a null ABI descriptor");
    if (api->abi_version != BIKE_POLICY_ABI_VERSION) throw PolicyFailure("unsupported policy ABI version");
    if (api->struct_size < sizeof(BikePolicyV1)) throw PolicyFailure("policy ABI descriptor is too small");
    if (api->required_capabilities != 0) throw PolicyFailure("policy requires unsupported capabilities");
    if (api->sensor_schema_version != BIKE_SENSOR_SCHEMA_VERSION) throw PolicyFailure("unsupported policy sensor schema");
    if (!api->create || !api->reset || !api->act || !api->destroy)
        throw PolicyFailure("policy ABI has a null required callback");
}
runtime::RideControl policy_control_from_abi(const BikeControlV1& value) {
    constexpr std::uint32_t known = BIKE_CONTROL_MOTOR_TORQUE | BIKE_CONTROL_MOTOR_LIMIT | BIKE_CONTROL_HUMAN_TORQUE |
        BIKE_CONTROL_CRANK_RATE | BIKE_CONTROL_POSTURE | BIKE_CONTROL_PELVIS_OFFSET;
    if (value.struct_size != sizeof(BikeControlV1) || (value.present_mask & ~known) != 0)
        throw PolicyFailure("policy returned an invalid control size or presence mask");
    if (value.rider_enabled > 1 || value.use_saddle > 1)
        throw PolicyFailure("policy returned a nonboolean uint8 field");
    if ((value.present_mask & BIKE_CONTROL_PELVIS_OFFSET) != 0 && (value.present_mask & BIKE_CONTROL_POSTURE) == 0)
        throw PolicyFailure("policy pelvis offset requires posture presence");
    runtime::RideControl out;
    if ((value.present_mask & BIKE_CONTROL_MOTOR_TORQUE) != 0) out.motor_torque_nm = value.motor_torque_nm;
    if ((value.present_mask & BIKE_CONTROL_MOTOR_LIMIT) != 0) out.motor_limit_nm = value.motor_limit_nm;
    if ((value.present_mask & BIKE_CONTROL_HUMAN_TORQUE) != 0) out.human_torque_nm = value.human_torque_nm;
    if ((value.present_mask & BIKE_CONTROL_CRANK_RATE) != 0) out.crank_target_rate_rad_s = value.crank_target_rate_rad_s;
    if ((value.present_mask & BIKE_CONTROL_POSTURE) != 0) {
        out.posture = ::rider::RiderPosture{};
        out.posture->torso_lean_rad = value.torso_lean_rad;
        out.posture->pelvis_pitch_rad = value.pelvis_pitch_rad;
        out.posture->use_saddle = value.use_saddle != 0;
        if ((value.present_mask & BIKE_CONTROL_PELVIS_OFFSET) != 0)
            out.posture->pelvis_offset_m = std::array<double, 2>{value.pelvis_offset_m[0], value.pelvis_offset_m[1]};
    }
    out.rider_enabled = value.rider_enabled != 0;
    try { runtime::control_validate_for(out, true); }
    catch (const std::invalid_argument& error) { throw PolicyFailure("policy returned an invalid control: " + std::string(error.what())); }
    return out;
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
BikeObservationV1 policy_observation_to_abi(const runtime::SensorObservation& value) {
    value.validate();
    BikeObservationV1 out{};
    out.struct_size = static_cast<std::uint32_t>(sizeof(out));
    out.sensor_schema_version = BIKE_SENSOR_SCHEMA_VERSION;
    out.valid = value.valid ? 1 : 0;
    out.valid_mask = value.valid ? BIKE_SENSOR_ALL_FIELDS : 0U;
    out.time_s = value.time_s; out.source_time_s = value.source_time_s;
    std::ranges::copy(value.values, std::begin(out.values));
    return out;
}
NATIVE_DIAG_POP
} // namespace bike::research
