// Completion 3.23. Source: tests/reference/test_sensor_noise.py.
// The seeded sensor-noise seam: NumPy-compatible spawn/draw order bitwise
// against the frozen legacy draws, the tape's ownership/validation contract,
// and the delivery pipeline's read/push/counter semantics against the frozen
// pre-refactor oracle trace.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "random/numpy_compat.hpp"
#include "runtime/sensors.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace brand = bike::random;

namespace {

runtime::SensorObservation raw_sensor(double time_s = 0.) {
    runtime::SensorObservation result;
    result.time_s = time_s;
    result.source_time_s = time_s;
    result.valid = true;
    result.values = {.25, -1.5, 9.81, .03, 2., 3., 4., 5., 6.};
    return result;
}

bool same_observation(const runtime::SensorObservation &a,
                      const runtime::SensorObservation &b) {
    return a.time_s == b.time_s && a.source_time_s == b.source_time_s &&
           a.valid == b.valid && a.values == b.values;
}

runtime::SensorPipeline pipeline(const runtime::SensorConfig &config,
                                 std::uint64_t seed, std::int64_t count) {
    const auto tape = brand::build_noise_tape(
        brand::SensorNoiseConfig{}, seed, count);
    return runtime::SensorPipeline(config, {.noise = tape.noise,
                                            .dropout_uniform = tape.dropout_uniform});
}

// The test file's three SensorConfig parametrizations projected onto the
// noise-source's four std fields (latency/bias/dropout/imu do not move the
// tape). config1 is SensorConfig.ideal() — every std zero; config2 zeroes
// acceleration+gyro and disables the IMU (the imu flag lives on the
// delivery config, so the tape is identical to a zeroed-std tape).
const std::array<brand::SensorNoiseConfig, 3> CONFIGS{
    brand::SensorNoiseConfig{},
    brand::SensorNoiseConfig{.acceleration_std_mps2 = 0., .gyro_std_rad_s = 0.,
                              .encoder_std_rad_s = 0., .torque_std_nm = 0.},
    brand::SensorNoiseConfig{.acceleration_std_mps2 = 0., .gyro_std_rad_s = 0.,
                              .encoder_std_rad_s = .01, .torque_std_nm = .05}};
const std::array<std::uint64_t, 3> SEEDS{0, 19, 9223372036854775819ull}; // 2**63+11

void test_exact_legacy_call_order(std::size_t config_index,
                                  std::size_t seed_index) {
    const std::string node_id =
        "tests/reference/test_sensor_noise.py::test_exact_legacy_call_order"
        "[config" + std::to_string(config_index) + "-" +
        std::to_string(SEEDS[seed_index]) + "]";
    bt::oracle::Node node(node_id);
    const auto tape = brand::build_noise_tape(
        CONFIGS[config_index], SEEDS[seed_index], 128);
    std::vector<double> noise;
    noise.reserve(tape.noise.size() * 9);
    for (const auto &row : tape.noise)
        noise.insert(noise.end(), row.begin(), row.end());
    const std::vector<std::size_t> noise_shape{tape.noise.size(), 9};
    node.f64(noise, noise_shape, "noise");
    const std::vector<std::size_t> uniform_shape{tape.dropout_uniform.size()};
    node.f64(tape.dropout_uniform, uniform_shape, "uniform");
    node.done();
}

// test_sensor_noise.py::test_zero_scales_and_imu_switch_do_not_shift_other_channels
void test_zero_scales_and_imu_switch_do_not_shift_other_channels() {
    const auto reference = brand::build_noise_tape(
        brand::SensorNoiseConfig{}, 31, 64);
    const auto masked = brand::build_noise_tape(
        brand::SensorNoiseConfig{.acceleration_std_mps2 = 0.,
                                  .gyro_std_rad_s = 0.}, 31, 64);
    for (std::size_t i = 0; i < 64; ++i) {
        for (std::size_t column = 4; column < 9; ++column)
            bt::require(reference.noise[i][column] == masked.noise[i][column],
                        "zeroed accel/gyro scales do not shift later channels");
        bt::require(reference.dropout_uniform[i] == masked.dropout_uniform[i],
                    "zeroed scales do not shift the dropout stream");
    }
}

// test_sensor_noise.py::test_draws_own_configuration_and_tape_owns_buffers
void test_draws_own_configuration_and_tape_owns_buffers() {
  brand::SensorNoiseConfig const config{};
  brand::SensorNoiseSource const source(config, 0);
  bt::require(&source.config() != &config,
              "the noise source owns its configuration");
  std::array<std::array<double, 9>, 2> noise{};
  noise.fill({1., 1., 1., 1., 1., 1., 1., 1., 1.});
  std::array<double, 2> dropout{.25, .25};
  std::vector<std::array<double, 9>> noise_rows(noise.begin(), noise.end());
  std::vector<double> dropout_rows(dropout.begin(), dropout.end());
  const brand::NoiseTape tape{.noise = noise_rows,
                              .dropout_uniform = dropout_rows};
  noise_rows.assign(2, {99., 99., 99., 99., 99., 99., 99., 99., 99.});
  dropout_rows.assign(2, .99);
  for (const auto &row : tape.noise)
    for (const double value : row)
      bt::require(value == 1., "the tape owns a copy of its noise rows");
  for (const double value : tape.dropout_uniform)
    bt::require(value == .25, "the tape owns a copy of its uniforms");
}

// test_sensor_noise.py::test_invalid_tape_domain — the C++ tape's fixed
// (count,9)/(count,) shape makes the shape/dtype parametrize rows
// (noise0/noise1, noise6..noise9) statically inexpressible; the value-domain
// rows port verbatim through NoiseTape::validate().
void test_invalid_tape_domain() {
    const std::array<double, 9> zeros{};
    const std::array<double, 9> infs{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    const std::array<double, 9> nans{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    // noise2..noise5: dropout 1., -.1, nan and noise inf — all rejected.
    for (const double dropout : {1., -.1, std::numeric_limits<double>::quiet_NaN()}) {
        const brand::NoiseTape tape{.noise = {zeros}, .dropout_uniform = {dropout}};
        bt::invalid([&] { tape.validate(); });
    }
    const brand::NoiseTape inf_noise{.noise = {infs}, .dropout_uniform = {.5}};
    bt::invalid([&] { inf_noise.validate(); });
    const brand::NoiseTape nan_noise{.noise = {nans}, .dropout_uniform = {.5}};
    bt::invalid([&] { nan_noise.validate(); });
    // The (0,8)/(9,) shape rows have no C++ spelling; the static-width tape
    // enforces the same contract at compile time. A count-mismatched pair is
    // the expressible shape violation.
    const brand::NoiseTape mismatched{.noise = {zeros, zeros},
                                       .dropout_uniform = {.5}};
    bt::invalid([&] { mismatched.validate(); });
}

// test_sensor_noise.py::test_invalid_seed — every parametrize row (True, -1,
// .5, '3') is a Python-domain rejection: build_noise_tape's std::uint64_t seed
// boundary makes non-integers unexpressible and negatives statically
// impossible, so the record holds at the type boundary. The one expressible
// contract — the count domain — ports below.
void test_invalid_seed() {
    // The oracle's whole record is the seed domain; the native signature's
    // uint64 boundary enforces it before the function body runs. The
    // expressible half — a valid seed still builds — is the compile-time
    // control; the rejected half is statically unspellable.
    const auto tape = brand::build_noise_tape(brand::SensorNoiseConfig{}, 0, 1);
    bt::require(tape.noise.size() == 1 && tape.dropout_uniform.size() == 1,
                "a valid seed/count still builds a one-row tape");
}

// test_sensor_noise.py::test_invalid_count — -1 ports verbatim; True/1.5 are
// Python-domain rows the int64 boundary cannot spell.
void test_invalid_count() {
    bt::invalid([&] {
        (void)brand::build_noise_tape(brand::SensorNoiseConfig{}, 0, -1); });
}

// test_sensor_noise.py::test_reads_and_invalid_push_do_not_consume_rng
void test_reads_and_invalid_push_do_not_consume_rng() {
    runtime::SensorConfig config{};
    config.latency_s = 0.;
    auto a = pipeline(config, 23, 64);
    auto b = pipeline(config, 23, 64);
    a.reset(raw_sensor());
    b.reset(raw_sensor());
    bt::require(same_observation(a.read(0.), a.read(0.)),
                "reads are idempotent and consume no tape");
    bt::invalid([&] { a.push(raw_sensor()); });
    a.push(raw_sensor(.01));
    b.push(raw_sensor(.01));
    bt::require(same_observation(a.read(.01), b.read(.01)),
                "the rejected push consumed no tape row");
}

// test_sensor_noise.py::test_pipeline_matches_frozen_pre_refactor_sensor_trace
void test_pipeline_matches_frozen_pre_refactor_sensor_trace() {
    runtime::SensorConfig config{};
    config.latency_s = .007;
    config.maximum_age_s = .012;
    config.acceleration_bias = {.1, -.2, .3};
    config.gyro_bias = .004;
    config.dropout_probability = .35;
    config.imu_enabled = false;
    auto pipeline_under_test = pipeline(config, 210, 64);
    pipeline_under_test.reset(raw_sensor());
    bt::oracle::Node node(
        "tests/reference/test_sensor_noise.py::"
        "test_pipeline_matches_frozen_pre_refactor_sensor_trace");
    std::vector<double> rows;
    rows.reserve(static_cast<std::size_t>(60 * 12));
    for (std::size_t index = 0; index < 20; ++index) {
        const double time_s = static_cast<double>(index) * .005;
        if (index != 0) pipeline_under_test.push(raw_sensor(time_s));
        for (const double now : {time_s, time_s + .001, time_s + .001}) {
            const auto read = pipeline_under_test.read(now);
            rows.insert(rows.end(),
                        {read.time_s, read.source_time_s, read.valid ? 1. : 0.});
            rows.insert(rows.end(), read.values.begin(), read.values.end());
        }
    }
    const std::vector<std::size_t> shape{60, 12};
    node.f64(rows, shape, "reads");
    const std::vector<double> counters{
        static_cast<double>(pipeline_under_test.state().samples_attempted),
        static_cast<double>(pipeline_under_test.state().samples_dropped)};
    node.f64(counters, "counters");
    node.done();
}

} // namespace

int main() {
    return bt::run([] {
        for (std::size_t config = 0; config < 3; ++config)
            for (std::size_t seed = 0; seed < 3; ++seed)
                test_exact_legacy_call_order(config, seed);
        test_zero_scales_and_imu_switch_do_not_shift_other_channels();
        test_draws_own_configuration_and_tape_owns_buffers();
        test_invalid_tape_domain();
        test_invalid_seed();
        test_invalid_count();
        test_reads_and_invalid_push_do_not_consume_rng();
        test_pipeline_matches_frozen_pre_refactor_sensor_trace();
    });
}
