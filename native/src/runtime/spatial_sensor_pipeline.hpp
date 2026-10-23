#pragma once

#include "control.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace runtime::spatial {

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>; // Row-major sensor <- nominal frame.

struct ImuTruth {
    double time_s = 0.;
    Vector3 specific_force_sensor_mps2{};
    Vector3 angular_rate_sensor_rad_s{};
};

Vector3 rotate_vector(const Matrix3& sensor_from_nominal, const Vector3& value);

struct SensorObservationV2 {
    double time_s = 0., source_time_s = 0.;
    bool valid = true;
    std::uint16_t valid_mask = 0x7ffU;
    // accel XYZ, gyro XYZ, front/rear/crank rad/s, motor/human N m.
    std::array<double, 11> values{};
    void validate() const;
    WireObject as_wire() const;
    static SensorObservationV2 from_wire(const WireObject&);
};

struct NoiseTapeV2 {
    std::vector<std::array<double, 11>> noise;
    std::vector<double> dropout_uniform;
    void validate() const;
};

struct SensorConfigV2 {
    double latency_s = .01, maximum_age_s = .1, dropout_probability = 0.;
    double sample_period_s = .01, maximum_read_period_s = .02;
    Vector3 acceleration_bias{}, gyro_bias{};
    Vector3 acceleration_std{.05, .05, .05}, gyro_std{.002, .002, .002};
    double encoder_std = .01, torque_std = .05;
    bool imu_enabled = true;
    void validate() const;
};

// Cold snapshot. The live queue is the preallocated ring owned by the pipeline.
struct SensorStateV2 {
    std::deque<SensorObservationV2> queue;
    std::optional<double> last_time, delivery_time;
    std::optional<SensorObservationV2> startup;
    std::size_t cursor = 0, samples_attempted = 0, samples_dropped = 0;
};

class SensorPipelineV2 {
public:
    SensorPipelineV2(SensorConfigV2, NoiseTapeV2);
    void reset(const SensorObservationV2&);
    void import_state(SensorStateV2);
    void push(const SensorObservationV2&);
    SensorObservationV2 read(double time_s);
    SensorStateV2 state() const;
    WireObject state_wire() const;

private:
    SensorConfigV2 config_;
    NoiseTapeV2 tape_;
    std::vector<SensorObservationV2> queue_storage_;
    std::size_t queue_head_ = 0, queue_count_ = 0;
    std::optional<double> last_time_, delivery_time_;
    std::optional<SensorObservationV2> startup_;
    std::size_t cursor_ = 0, samples_attempted_ = 0, samples_dropped_ = 0;
};

} // namespace runtime::spatial
