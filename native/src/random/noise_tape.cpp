// random/noise_tape.cpp — the T19 source ports riding the NumPy-compat
// seam: sim/research/sensor_noise.py's NoiseTape validation,
// SensorNoiseSource and build_noise_tape, plus sim/research/
// rider_random.py's RiderRandomSpec domain and sample_rider. Draw order,
// stream split (spawned noise/dropout children) and error text are the
// contract; the oracle fixtures pin every produced bit.

#include "random/numpy_compat.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bike::random {
namespace {

// checks.py scalar(value, name, minimum=0.) — the finite/nonnegative leg
// every SensorConfig scale passes through ('invalid {name}').
void check_scale(double value, const char *name) {
    if (!std::isfinite(value) || value < 0.)
        throw std::invalid_argument(std::string("invalid ") + name);
}

// rider_random.py _range(value, name, low_limit, high_limit): the pair leg
// is static (std::array<double, 2>); scalar() runs first on both bounds
// ('invalid {name}' for a non-finite one), then the enclosing chain check
// with the source's formatted limit text.
void check_range_limits(std::array<double, 2> range, const char *name,
                        double low_limit, double high_limit,
                        const char *limits_text) {
    if (!std::isfinite(range.at(0)) || !std::isfinite(range.at(1)))
        throw std::invalid_argument(std::string("invalid ") + name);
    if (!(low_limit <= range.at(0) && range.at(0) <= range.at(1) &&
          range.at(1) <= high_limit))
        throw std::invalid_argument(std::string(name) + " must satisfy " +
                                    limits_text);
}

// sensor_noise.py __init__ — np.random.SeedSequence(seed).spawn(2): one
// parent, children (0,) and (1,).
std::pair<SeedSequence, SeedSequence> spawned_streams(std::uint64_t seed) {
    std::vector<SeedSequence> children = SeedSequence(seed).spawn(2);
    return {std::move(children.at(0)), std::move(children.at(1))};
}

} // namespace

// SensorConfig.__post_init__ — only the four noise scales are carried by
// this record; their scalar(minimum=0.) checks keep the source order
// (acceleration, gyro, encoder, torque).
void SensorNoiseConfig::validate() const {
    check_scale(acceleration_std_mps2, "acceleration_std_mps2");
    check_scale(gyro_std_rad_s, "gyro_std_rad_s");
    check_scale(encoder_std_rad_s, "encoder_std_rad_s");
    check_scale(torque_std_nm, "torque_std_nm");
}

// NoiseTape.__post_init__ — shape first (dropout must track the noise row
// count; the (count, 9) leg is static), then finiteness, then the [0, 1)
// dropout domain, in source order.
void NoiseTape::validate() const {
    if (dropout_uniform.size() != noise.size())
        throw std::invalid_argument(
            "noise tape requires (count, 9) noise and (count,) dropout");
    for (const std::array<double, 9> &row : noise)
        for (const double value : row)
            if (!std::isfinite(value))
                throw std::invalid_argument("noise tape must be finite");
    for (const double value : dropout_uniform)
        if (!std::isfinite(value))
            throw std::invalid_argument("noise tape must be finite");
    for (const double value : dropout_uniform)
        if (value < 0. || value >= 1.)
            throw std::invalid_argument(
                "dropout uniforms must lie in [0, 1)");
}

SensorNoiseSource::SensorNoiseSource(SensorNoiseConfig config,
                                     std::uint64_t seed)
    : SensorNoiseSource(config, seed, spawned_streams(seed)) {}

SensorNoiseSource::SensorNoiseSource(
    SensorNoiseConfig config, std::uint64_t seed,
    const std::pair<SeedSequence, SeedSequence> &children)
    : config_(config), seed_(seed), rng_(children.first),
      dropout_rng_(children.second) {
    // __init__ replaces the incoming config (dataclasses.replace re-runs
    // __post_init__) then validates the seed. The uint64 boundary already
    // enforces _seed()'s nonnegative-integer domain; the two SeedSequence
    // children keep spawn keys (0,) and (1,) — one generator each.
    config_.validate();
}

std::pair<std::array<double, 9>, double> SensorNoiseSource::draw() {
    const SensorNoiseConfig &c = config_;
    // Four separate calls, including the scalar gyro call. The disabled
    // IMU and zero scales do not skip any of them.
    std::array<double, 3> acceleration{};
    rng_.normal(0., c.acceleration_std_mps2, acceleration);
    const double gyro = rng_.normal(0., c.gyro_std_rad_s);
    std::array<double, 3> encoder{};
    rng_.normal(0., c.encoder_std_rad_s, encoder);
    std::array<double, 2> torque{};
    rng_.normal(0., c.torque_std_nm, torque);
    const std::array<double, 9> noise = {
        acceleration.at(0), acceleration.at(1), acceleration.at(2), gyro,
        encoder.at(0), encoder.at(1), encoder.at(2),
        torque.at(0), torque.at(1)};
    return {noise, dropout_rng_.uniform01()};
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters) (seed, count) mirrors the source's build_noise_tape(config, *, seed, count) keywords
NoiseTape build_noise_tape(const SensorNoiseConfig &config,
                           std::uint64_t seed, std::int64_t count) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    if (count < 0)
        throw std::invalid_argument(
            "noise tape count must be a nonnegative integer");
    SensorNoiseSource source(config, seed);
    NoiseTape tape;
    tape.noise.resize(static_cast<std::size_t>(count));
    tape.dropout_uniform.resize(static_cast<std::size_t>(count));
    for (std::int64_t index = 0; index < count; ++index) {
        const std::pair<std::array<double, 9>, double> draw = source.draw();
        tape.noise.at(static_cast<std::size_t>(index)) = draw.first;
        tape.dropout_uniform.at(static_cast<std::size_t>(index)) =
            draw.second;
    }
    tape.validate();
    return tape;
}

// RiderRandomSpec.__post_init__ — the five _range checks in field order
// with the Python-formatted limit text.
void RiderRandomSpec::validate() const {
    check_range_limits(mass_kg, "mass_kg", 20., 200.,
                       "20.0 <= low <= high <= 200.0");
    check_range_limits(height_m, "height_m", 1., 2.3,
                       "1.0 <= low <= high <= 2.3");
    check_range_limits(torso_lean_rad, "torso_lean_rad", -.8, .8,
                       "-0.8 <= low <= high <= 0.8");
    check_range_limits(pelvis_pitch_rad, "pelvis_pitch_rad", -.5, .5,
                       "-0.5 <= low <= high <= 0.5");
    check_range_limits(human_torque_nm, "human_torque_nm", 0., 1000.,
                       "0.0 <= low <= high <= 1000.0");
}

SampledRider sample_rider(const RiderRandomSpec &spec,
                          const SeedSequence &seed) {
    // The dataclass runs __post_init__ at construction; an aggregate spec
    // validates here before any draw, the same observable rejection point.
    spec.validate();
    NumpyRng rng(seed);
    SampledRider sampled;
    sampled.rider.variant = "articulated_planar";
    sampled.rider.mass_kg =
        rng.uniform(spec.mass_kg.at(0), spec.mass_kg.at(1));
    sampled.rider.height_m =
        rng.uniform(spec.height_m.at(0), spec.height_m.at(1));
    ::rider::RiderPosture posture;
    posture.torso_lean_rad =
        rng.uniform(spec.torso_lean_rad.at(0), spec.torso_lean_rad.at(1));
    posture.pelvis_pitch_rad =
        rng.uniform(spec.pelvis_pitch_rad.at(0), spec.pelvis_pitch_rad.at(1));
    const double torque =
        rng.uniform(spec.human_torque_nm.at(0), spec.human_torque_nm.at(1));
    sampled.program.keyframes = {RiderKeyframe{
        .time_s = 0., .posture = posture, .human_torque_nm = torque}};
    sampled.program.reaction_delay_s = 0.;
    return sampled;
}

SampledRider sample_rider(const RiderRandomSpec &spec, std::uint64_t seed) {
    return sample_rider(spec, SeedSequence(seed));
}

} // namespace bike::random
