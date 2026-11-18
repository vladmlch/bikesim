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
    // latency_s and maximum_age_s carry the -1 sentinel: the profile's
    // [research] section is the single place they are declared (validate()
    // rejects the sentinel if any path forgets to copy it across).
    double latency_s = -1.;
    std::array<double, 3> acceleration_bias{};
    double gyro_bias = 0.;
    double dropout_probability = 0.;
    double maximum_age_s = -1.;
    bool imu_enabled = true;
    std::array<double, 3> gyro_bias_3d_rad_s{0., 0., 0.};
    void validate() const;
};
struct SensorObservation {
    double time_s = 0., source_time_s = 0.;
    bool valid = true;
    // Acceleration x/y/z, gyro, front/rear/crank encoders, motor/human torque.
    std::array<double, 9> values{};
    // Engaged iff spatial 6-axis IMU mode is active:
    // [omega_x (roll rate), omega_y (pitch-down rate), omega_z (yaw rate)] in IMU frame (rad/s).
    std::optional<std::array<double, 3>> angular_rate_body_rad_s{};
    void validate() const;
    [[nodiscard]] WireObject as_wire() const;
    [[nodiscard]] static SensorObservation from_wire(const WireObject &value);
};
struct NoiseTape {
    std::vector<std::array<double, 9>> noise;
    std::vector<double> dropout_uniform;
    std::vector<std::array<double, 3>> spatial_gyro_noise{};
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
[[nodiscard]] SensorObservation raw_sensor_observation(const PhysicalSampleData &sample, bool spatial_imu = false);
} // namespace runtime
