#include "environment.hpp"
#include <limits>
#include <stdexcept>

namespace bike::research {
void sample_rider(ResearchRequest& request) {
    if (!request.rider_random) return;
    if (!request.programs.rider_frames.empty())
        throw std::invalid_argument("rider randomization and a posture-owning rider program are exclusive");
    const random::SeedSequence seed(validate_seed(request.rider_seed_decimal.value_or(request.experiment.seed_decimal)));
    auto sampled = random::sample_rider(*request.rider_random, seed);
    request.setup.model.rider = std::move(sampled.rider);
    request.programs.reaction_delay_s = sampled.program.reaction_delay_s;
    for (auto const& frame : sampled.program.keyframes)
        request.programs.rider_frames.push_back({.time_s=frame.time_s, .posture=frame.posture, .human_torque_nm=frame.human_torque_nm});
    request.programs.validate();
}

runtime::NoiseTape make_sensor_tape(const SensorConfig& config, std::string_view seed_decimal, std::int64_t count) {
    config.validate();
    if (count < 0 || std::cmp_greater(count, std::numeric_limits<std::size_t>::max()))
        throw std::invalid_argument("noise tape count must be a representable nonnegative integer");
    random::SeedSequence seed(validate_seed(seed_decimal));
    const auto children = seed.spawn(2);
    random::NumpyRng normal(children[0]), dropout(children[1]);
    runtime::NoiseTape tape;
    const auto size = static_cast<std::size_t>(count);
    tape.noise.resize(size); tape.dropout_uniform.resize(size);
    for (std::size_t i = 0; i < size; ++i) {
        auto const row = std::span<double>{tape.noise[i]};
        normal.normal(0., config.noise.acceleration_std_mps2, row.first<3>());
        row[3] = normal.normal(0., config.noise.gyro_std_rad_s);
        normal.normal(0., config.noise.encoder_std_rad_s, row.subspan<4, 3>());
        normal.normal(0., config.noise.torque_std_nm, row.subspan<7, 2>());
        tape.dropout_uniform[i] = dropout.uniform01();
    }
    tape.validate();
    return tape;
}

runtime::SensorState startup_sensor_state(const SensorConfig& config, const runtime::NoiseTape& tape,
                                         const runtime::SensorObservation& raw_initial) {
    config.validate();
    runtime::SensorPipeline pipeline(config.delivery, tape);
    pipeline.reset(raw_initial);
    (void)pipeline.read(0.);
    return pipeline.state();
}
} // namespace bike::research
