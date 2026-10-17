// Standalone trusted policy example. Build as a C++23 shared module using
// native/include, with floating-point contraction and fast-math disabled.
#include <bike/policy_v1.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>
#include <string_view>

namespace {
struct State {
    double cap_nm = 0., last_time_s = 0., standstill_s = 0.;
    bool has_cap = false, has_time = false;
};
int error(BikeErrorV1* out, std::string_view message) noexcept {
    if (out) {
        out->written = out->data ? std::min(out->capacity, message.size()) : 0;
        if (out->written) std::memcpy(out->data, message.data(), out->written);
    }
    return 1;
}
bool empty_object(std::string_view json) noexcept {
    const auto space = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
    std::size_t at = 0;
    while (at < json.size() && space(json[at])) ++at;
    if (at == json.size() || json[at++] != '{') return false;
    while (at < json.size() && space(json[at])) ++at;
    if (at == json.size() || json[at++] != '}') return false;
    while (at < json.size() && space(json[at])) ++at;
    return at == json.size();
}
int create(const char* config, std::size_t count, void** instance, BikeErrorV1* failure) noexcept {
    if (!instance) return error(failure, "missing host instance destination");
    *instance = nullptr;
    if (!config || !empty_object(std::string_view(config, count)))
        return error(failure, "the standalone example accepts an empty JSON object; configurable defaults are in builtin:traction_control");
    auto* state = new (std::nothrow) State{};
    if (!state) return error(failure, "cannot allocate policy state");
    *instance = state;
    return 0;
}
int reset(void* instance, const char* seed, std::size_t count, BikeErrorV1* failure) noexcept {
    if (!instance || !seed || count == 0) return error(failure, "reset requires state and a decimal seed");
    for (const char digit : std::string_view(seed, count))
        if (digit < '0' || digit > '9') return error(failure, "seed must be a nonnegative decimal integer");
    *static_cast<State*>(instance) = State{};
    return 0;
}
int act(void* instance, const BikeObservationV1* observation, const BikeDemandV1* demand,
        BikeControlV1* output, BikeErrorV1* failure) noexcept {
    if (!instance || !observation || !demand || !output)
        return error(failure, "policy act requires host-owned inputs and output");
    if (observation->struct_size != sizeof(BikeObservationV1) || output->struct_size != sizeof(BikeControlV1) ||
        observation->sensor_schema_version != BIKE_SENSOR_SCHEMA_VERSION || observation->valid > 1 || demand->present > 1 ||
        observation->valid_mask != (observation->valid ? BIKE_SENSOR_ALL_FIELDS : 0U))
        return error(failure, "unsupported sensor/control layout or validity bits");
    if (!std::isfinite(observation->time_s) || observation->time_s < 0. || !std::isfinite(observation->source_time_s))
        return error(failure, "nonfinite or negative observation time");
    for (const double value : observation->values)
        if (!std::isfinite(value)) return error(failure, "nonfinite sensor field");
    if (demand->present && (!std::isfinite(demand->torque_nm) || demand->torque_nm < 0.))
        return error(failure, "demand must be finite and nonnegative");
    BikeControlV1 control{};
    control.struct_size = sizeof(BikeControlV1);
    control.use_saddle = control.rider_enabled = 1;
    if (demand->present) {
        control.present_mask = BIKE_CONTROL_MOTOR_TORQUE;
        control.motor_torque_nm = demand->torque_nm;
    }
    auto& state = *static_cast<State*>(instance);
    const double elapsed = state.has_time ? std::max(0., observation->time_s - state.last_time_s) : 0.;
    state.last_time_s = observation->time_s; state.has_time = true;
    if (observation->valid) {
        const double slip = .352 * observation->values[5] - .372 * observation->values[4];
        const double ceiling = std::max(0., demand->present ? demand->torque_nm : observation->values[7]);
        if (!state.has_cap) {
            if (slip > .2) { state.has_cap = true; state.cap_nm = ceiling; }
        } else state.cap_nm = std::min(state.cap_nm, ceiling);
        if (state.has_cap) {
            if (slip > .2) state.cap_nm -= 200. * (slip - .2) * elapsed;
            else state.cap_nm += 25. * elapsed;
            state.cap_nm = std::min(std::max(state.cap_nm, std::min(8., ceiling)), ceiling);
            if (state.cap_nm >= ceiling) state.has_cap = false;
            if (state.has_cap) { control.present_mask |= BIKE_CONTROL_MOTOR_LIMIT; control.motor_limit_nm = state.cap_nm; }
        }
        const bool stopped = std::abs(observation->values[5]) < .3;
        if (stopped && demand->present) state.standstill_s += elapsed;
        else if (!stopped) state.standstill_s = 0.;
        if (stopped && state.standstill_s >= .8) control.present_mask &= ~BIKE_CONTROL_MOTOR_TORQUE;
    }
    *output = control;
    return 0;
}
void destroy(void* instance) noexcept { delete static_cast<State*>(instance); }
const BikePolicyV1 policy{BIKE_POLICY_ABI_VERSION, sizeof(BikePolicyV1), 0, create, reset, act, destroy,
    BIKE_SENSOR_SCHEMA_VERSION};
} // namespace

#if defined(_WIN32)
#define BIKE_EXAMPLE_EXPORT __declspec(dllexport)
#else
#define BIKE_EXAMPLE_EXPORT __attribute__((visibility("default")))
#endif
extern "C" BIKE_EXAMPLE_EXPORT const BikePolicyV1* bike_policy_v1(void) { return &policy; }
