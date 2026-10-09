// Deterministic sensor delivery. All state and setup-generated noise are owned.
#pragma once
#include <array>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>
#include "samples.hpp"

namespace runtime {
struct SensorConfig {
    double latency_s = .01;
    std::array<double, 3> acceleration_bias{};
    double gyro_bias = 0.;
    double dropout_probability = 0.;
    double maximum_age_s = .1;
    bool imu_enabled = true;
    void validate() const;
};
struct SensorObservation {
    double time_s = 0., source_time_s = 0.;
    bool valid = true;
    // Acceleration x/y/z, gyro, front/rear/crank encoders, motor/human torque.
    std::array<double, 9> values{};
    void validate() const;
    [[nodiscard]] WireObject as_wire() const;
    [[nodiscard]] static SensorObservation from_wire(const WireObject &value);
};
struct NoiseTape {
    std::vector<std::array<double, 9>> noise;
    std::vector<double> dropout_uniform;
    void validate() const;
};
struct SensorState {
    std::deque<SensorObservation> queue;
    std::optional<double> last_time, delivery_time;
    std::optional<SensorObservation> startup;
    std::size_t cursor = 0, samples_attempted = 0, samples_dropped = 0;
};
class SensorPipeline {
public:
    SensorPipeline(SensorConfig config, NoiseTape tape);
    void reset(const SensorObservation &initial);
    void import_state(SensorState state);
    void push(const SensorObservation &raw);
    [[nodiscard]] SensorObservation read(double time_s);
    [[nodiscard]] const SensorState &state() const noexcept { return state_; }
    [[nodiscard]] WireObject state_wire() const;
private:
    SensorConfig config_;
    NoiseTape tape_;
    SensorState state_;
};
[[nodiscard]] SensorObservation raw_sensor_observation(const PhysicalSampleData &sample);
} // namespace runtime
