#include "sensors.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace runtime {
namespace {
void nonnegative(double value, std::string_view path) {
    if (!std::isfinite(value) || value < 0.)
        throw std::invalid_argument(std::string(path) + " must be finite and nonnegative");
}
SensorObservation invalid_sample(SensorObservation raw) {
    raw.valid = false;
    raw.values.fill(0.);
    return raw;
}
constexpr std::array<std::string_view, 6> scalar_names{
    "pitch_rate_up_rad_s", "front_wheel_rad_s", "rear_wheel_rad_s",
    "crank_rad_s", "motor_torque_nm", "human_torque_nm"};
}
void SensorConfig::validate() const {
    nonnegative(latency_s, "sensors.latency_s");
    nonnegative(maximum_age_s, "sensors.maximum_age_s");
    nonnegative(dropout_probability, "sensors.dropout_probability");
    if (dropout_probability > 1.)
        throw std::invalid_argument("sensors.dropout_probability must lie in [0, 1]");
    for (const double bias : acceleration_bias)
        if (!std::isfinite(bias)) throw std::invalid_argument("sensors.acceleration_bias must be finite");
    if (!std::isfinite(gyro_bias)) throw std::invalid_argument("sensors.gyro_bias must be finite");
}
void SensorObservation::validate() const {
    nonnegative(time_s, "observation.time_s");
    nonnegative(source_time_s, "observation.source_time_s");
    if (source_time_s > time_s + 1e-12)
        throw std::invalid_argument("sensor observation cannot come from the future");
    for (const double value : values)
        if (!std::isfinite(value)) throw std::invalid_argument("sensor observation must be finite");
}
WireObject SensorObservation::as_wire() const {
    WireObject result{{"time_s", time_s}, {"source_time_s", source_time_s}, {"valid", valid},
        {"specific_force_body_mps2", wire_array(std::span<const double>(values).first<3>())}};
    for (std::size_t i = 0; i < scalar_names.size(); ++i)
        result.emplace_back(std::string(scalar_names[i]), values[i + 3]);
    return result;
}
SensorObservation SensorObservation::from_wire(const WireObject &value) {
    using namespace sample_wire;
    constexpr std::array<std::string_view, 10> keys{"time_s", "source_time_s", "valid",
        "specific_force_body_mps2", "pitch_rate_up_rad_s", "front_wheel_rad_s",
        "rear_wheel_rad_s", "crank_rad_s", "motor_torque_nm", "human_torque_nm"};
    exact(value, keys, "sensor observation");
    SensorObservation result;
    result.time_s = number(required(value, "time_s"));
    result.source_time_s = number(required(value, "source_time_s"));
    result.valid = boolean(required(value, "valid"));
    const auto &acceleration = array(required(value, "specific_force_body_mps2"));
    if (acceleration.size() != 3) throw std::invalid_argument("sensor acceleration must have width 3");
    for (std::size_t i = 0; i < 3; ++i) result.values[i] = number(acceleration[i]);
    for (std::size_t i = 0; i < scalar_names.size(); ++i)
        result.values[i + 3] = number(required(value, scalar_names[i]));
    result.validate();
    return result;
}
void NoiseTape::validate() const {
    if (noise.size() != dropout_uniform.size())
        throw std::invalid_argument("noise tape shapes must be (count, 9) and (count,)");
    for (const auto &row : noise)
        for (const double value : row)
            if (!std::isfinite(value)) throw std::invalid_argument("noise tape must be finite");
    for (const double value : dropout_uniform)
        if (!std::isfinite(value) || value < 0. || value >= 1.)
            throw std::invalid_argument("dropout uniforms must lie in [0, 1)");
}
SensorPipeline::SensorPipeline(SensorConfig config, NoiseTape tape)
    : config_(config), tape_(std::move(tape)) {
    config_.validate();
    tape_.validate();
}
void SensorPipeline::reset(const SensorObservation &initial) {
    initial.validate();
    // Build row zero using a temporary state, retaining the old episode on
    // arithmetic/allocation failure. The tape itself never changes on reset.
    SensorState previous = state_;
    state_ = SensorState{};
    try { push(initial); }
    catch (...) { state_ = std::move(previous); throw; }
}
void SensorPipeline::import_state(SensorState next) {
    if (next.cursor != next.samples_attempted || next.cursor > tape_.noise.size() ||
        next.samples_dropped > next.samples_attempted ||
        next.queue.size() > next.samples_attempted - next.samples_dropped)
        throw std::invalid_argument("sensor startup cursor/counters are inconsistent");
    if (next.cursor == 0) {
        if (next.startup || next.last_time || next.delivery_time || !next.queue.empty())
            throw std::invalid_argument("uninitialized sensor state contains samples");
    } else if (!next.startup || !next.last_time) {
        throw std::invalid_argument("initialized sensor state requires startup and last_time");
    }
    if (next.last_time) nonnegative(*next.last_time, "sensor.last_time");
    if (next.delivery_time) nonnegative(*next.delivery_time, "sensor.delivery_time");
    if (next.startup) {
        next.startup->validate();
        if (next.startup->valid || next.startup->source_time_s > *next.last_time ||
            std::ranges::any_of(next.startup->values, [](double x) { return x != 0.; }))
            throw std::invalid_argument("sensor startup must be a zero invalid sample");
        if (next.delivery_time && next.startup->source_time_s > *next.delivery_time + 1e-12)
            throw std::invalid_argument("sensor startup lies after the delivery clock");
    }
    std::optional<double> last;
    for (const auto &sample : next.queue) {
        sample.validate();
        if ((last && sample.source_time_s <= *last) ||
            !next.last_time || sample.source_time_s > *next.last_time ||
            sample.source_time_s < next.startup->source_time_s)
            throw std::invalid_argument("sensor startup queue timestamps are inconsistent");
        last = sample.source_time_s;
    }
    state_ = std::move(next);
}
void SensorPipeline::push(const SensorObservation &raw) {
    raw.validate();
    if (state_.last_time && raw.source_time_s <= *state_.last_time)
        throw std::invalid_argument("sensor input timestamps must strictly increase");
    if (state_.cursor >= tape_.noise.size())
        throw std::runtime_error("sensor noise tape exhausted; research run is invalid");
    SensorObservation measured = raw;
    const auto &noise = tape_.noise[state_.cursor];
    // Preserve NumPy ufunc association: (raw + bias) + scaled noise.
    for (std::size_t i = 0; i < 3; ++i)
        measured.values[i] = (raw.values[i] + config_.acceleration_bias[i]) + noise[i];
    measured.values[3] = (raw.values[3] + config_.gyro_bias) + noise[3];
    for (std::size_t i = 4; i < noise.size(); ++i) measured.values[i] += noise[i];
    measured.validate();
    if (!config_.imu_enabled) std::fill_n(measured.values.begin(), 4, 0.);
    const bool drop = tape_.dropout_uniform[state_.cursor] < config_.dropout_probability;
    // Only append may allocate; commit scalar clocks/counters afterwards.
    if (!drop) state_.queue.push_back(measured);
    if (!state_.startup) state_.startup = invalid_sample(raw);
    state_.last_time = raw.source_time_s;
    ++state_.cursor;
    ++state_.samples_attempted;
    if (drop) ++state_.samples_dropped;
}
SensorObservation SensorPipeline::read(double now) {
    nonnegative(now, "sensor delivery time");
    if (state_.delivery_time && now < *state_.delivery_time)
        throw std::invalid_argument("sensor delivery clock cannot go backwards");
    if (!state_.startup) throw std::runtime_error("reset the sensor pipeline before reading it");
    const double cutoff = now - config_.latency_s;
    std::size_t selected_index = 0;
    while (selected_index + 1 < state_.queue.size() &&
           state_.queue[selected_index + 1].source_time_s <= cutoff + 1e-12) ++selected_index;
    SensorObservation selected = !state_.queue.empty() &&
        state_.queue[selected_index].source_time_s <= cutoff + 1e-12
        ? state_.queue[selected_index] : *state_.startup;
    if (selected.source_time_s > now + 1e-12)
        throw std::invalid_argument("cannot deliver a future sensor sample");
    selected.time_s = now;
    selected.valid = selected.valid && selected.source_time_s <= cutoff + 1e-12 &&
        now - selected.source_time_s <= config_.maximum_age_s + 1e-12;
    // Invalid reads leave queue and delivery clock untouched.
    for (std::size_t i = 0; i < selected_index; ++i) state_.queue.pop_front();
    state_.delivery_time = now;
    return selected;
}
WireObject SensorPipeline::state_wire() const {
    WireArray queue;
    queue.reserve(state_.queue.size());
    for (const auto &sample : state_.queue) queue.emplace_back(sample.as_wire());
    const auto optional = [](std::optional<double> x) { return x ? Wire(*x) : Wire(nullptr); };
    return {{"queue", std::move(queue)}, {"last_time", optional(state_.last_time)},
        {"delivery_time", optional(state_.delivery_time)},
        {"startup", state_.startup ? Wire(state_.startup->as_wire()) : Wire(nullptr)},
        {"cursor", static_cast<std::int64_t>(state_.cursor)},
        {"samples_attempted", static_cast<std::int64_t>(state_.samples_attempted)},
        {"samples_dropped", static_cast<std::int64_t>(state_.samples_dropped)}};
}
SensorObservation raw_sensor_observation(const PhysicalSampleData &sample) {
    using namespace sample_wire;
    const auto &channels = object(required(sample.channels, "sensors"));
    const auto &force = array(required(channels, "frame_specific_force_body_mps2"));
    const auto &gyro = array(required(channels, "frame_gyro_body_rad_s"));
    const auto &enc = object(required(channels, "encoders_rad_s"));
    if (force.size() != 3 || gyro.size() != 3) throw std::invalid_argument("invalid raw IMU width");
    SensorObservation result;
    result.time_s = result.source_time_s = sample.time_s;
    for (std::size_t i = 0; i < 3; ++i) result.values[i] = number(force[i]);
    result.values[3] = -number(gyro[1]);
    result.values[4] = number(required(enc, "front_wheel"));
    result.values[5] = number(required(enc, "rear_wheel"));
    result.values[6] = number(required(enc, "crank"));
    result.values[7] = number(required(channels, "motor_torque_nm"));
    result.values[8] = number(required(channels, "human_torque_nm"));
    result.validate();
    return result;
}
} // namespace runtime
