// random/numpy_compat.hpp — the T19 seeded-random boundary: a pure-C++
// reproduction of the NumPy 2.5.2 random stack the research/terrain
// generators consume, pinned bitwise to the oracle.
//
// Ported upstream behavior (all draw counts and bit patterns are locked by
// the random/* oracle fixtures):
//   * bit_generator.pyx SeedSequence — entropy coercion to little-endian
//     uint32 words, the INIT_A/MULT_A hashmix entropy pool, get_assembled
//     -entropy's spawn-key padding rule, generate_state(uint32|uint64) and
//     spawn(n_children) bookkeeping.
//   * pcg64.h/pcg64.c PCG64 — the 128-bit XSL-RR generator seeded from four
//     generated uint64 words ({state.high, state.low, inc.high, inc.low}),
//     including the buffered 32-bit half-word lane (has_uint32/uinteger)
//     that next_uint32 consumers share across next64 draws.
//   * distributions.c + _generator.pyx — uint64_to_double, the Ziggurat
//     standard normal (scale=0 still consumes the stream), uniform/normal
//     loc-scale transforms, Lemire bounded integers and choice()'s
//     kahan/cumsum/searchsorted probability path.
//
// Owned sources riding this seam (kept as the Python behavioral oracle
// until T63):
//   * sim/research/sensor_noise.py — SensorNoiseSource's two spawned child
//     streams, the four fixed normal calls per draw() (accel 3, gyro 1,
//     encoder 3, torque 2 — never skipped, even at zero scale or a disabled
//     IMU) plus the one dropout uniform, NoiseTape's validation, and
//     build_noise_tape's count domain.
//   * sim/research/rider_random.py — RiderRandomSpec's (low, high) domain
//     and sample_rider's five uniform draws in source order.
//
// Record reuse notes: bike::configuration::RiderSpecs and
// rider::RiderPosture are the existing native twins of physics/rider.py and
// physics/rider_posture.py and are included — never redeclared. The
// rider_program.py keyframe/program records below mirror the source
// dataclass field layout exactly (runtime::RiderKeyframe carries the
// identical shape through the heavier runtime samples chain, which this
// leaf boundary must not pull in); sensor_noise.py's NoiseTape likewise
// mirrors runtime::NoiseTape's {noise, dropout_uniform} fields.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../configuration/rider_spec.hpp"
#include "../rider/rider_posture.hpp"

namespace bike::random {

// ---------------------------------------------------------------------------
// bit_generator.pyx SeedSequence — pool_size is fixed at DEFAULT_POOL_SIZE=4
// (the only value the consumed NumPy paths use); entropy is a nonnegative
// integer accepted as a uint64 word or a decimal/0x-hex string of arbitrary
// precision, matching _coerce_to_uint32_array's int/str handling.
// ---------------------------------------------------------------------------
class SeedSequence {
public:
    explicit SeedSequence(std::uint64_t entropy);
    explicit SeedSequence(std::string_view entropy);
    SeedSequence(std::string_view entropy,
                 std::span<const std::uint32_t> spawn_key);

    // Python attributes: spawn_key (tuple of ints) and n_children_spawned.
    [[nodiscard]] const std::vector<std::uint32_t> &spawn_key() const noexcept {
        return spawn_key_;
    }
    [[nodiscard]] std::uint32_t n_children_spawned() const noexcept {
        return n_children_spawned_;
    }
    // The mixed entropy pool (post-mix_entropy), exposed for fixture checks.
    [[nodiscard]] const std::array<std::uint32_t, 4> &pool() const noexcept {
        return pool_;
    }

    // generate_state(n_words, dtype=np.uint32|np.uint64).
    [[nodiscard]] std::vector<std::uint32_t>
    generate_state_u32(std::size_t n_words) const;
    [[nodiscard]] std::vector<std::uint64_t>
    generate_state_u64(std::size_t n_words) const;

    // spawn(n_children): each child keeps this sequence's entropy and extends
    // the spawn key with n_children_spawned + i; the counter then advances.
    [[nodiscard]] std::vector<SeedSequence> spawn(std::uint32_t n_children);

private:
    SeedSequence(std::vector<std::uint32_t> entropy_words,
                 std::vector<std::uint32_t> spawn_key);

    std::vector<std::uint32_t> entropy_words_;
    std::vector<std::uint32_t> spawn_key_;
    std::uint32_t n_children_spawned_ = 0;
    std::array<std::uint32_t, 4> pool_{};
};

// ---------------------------------------------------------------------------
// detail — the bitgen state record and NumPy transform declarations shared
// by the random implementation TUs (numpy_compat.cpp owns the PCG64 core;
// distributions.cpp owns the distribution transforms). These names are the
// internal seam the produced API delegates to, not public surface.
// ---------------------------------------------------------------------------
namespace detail {

// pcg64_state: the setseq-128 generator state plus the buffered 32-bit lane.
// next32() draws a fresh uint64 only when no half-word is pending, returns
// the low 32 bits first and buffers the high half — next64() never touches
// the pending half, exactly like pcg64.c.
struct Bitgen {
    std::uint64_t state_high = 0;
    std::uint64_t state_low = 0;
    std::uint64_t inc_high = 0;
    std::uint64_t inc_low = 1;
    bool has_uint32 = false;
    std::uint32_t uinteger = 0;
};

// numpy_compat.cpp — pcg_setseq_128_srandom_r / _xsl_rr_64_random_r /
// next32 / uint64_to_double. `seed_words` is the four uint64 words of
// SeedSequence.generate_state(4, np.uint64): {initstate.high, initstate.low,
// initseq.high, initseq.low}.
void seed_bitgen(Bitgen &bitgen, std::uint64_t seed_high,
                 std::uint64_t seed_low, std::uint64_t inc_high,
                 std::uint64_t inc_low);
[[nodiscard]] std::uint64_t next64(Bitgen &bitgen);
[[nodiscard]] std::uint32_t next32(Bitgen &bitgen);
[[nodiscard]] double next_double(Bitgen &bitgen);

// distributions.cpp — distributions.c / _generator.pyx / _common.pyx ports.
[[nodiscard]] double standard_normal(Bitgen &bitgen);
[[nodiscard]] double normal(Bitgen &bitgen, double loc, double scale);
[[nodiscard]] double uniform(Bitgen &bitgen, double low, double range);
// random_bounded_uint64_fill(cnt=1) with use_masked=False: returns a value
// on the closed interval [off, off+rng].
[[nodiscard]] std::uint64_t bounded_uint64(Bitgen &bitgen, std::uint64_t off,
                                           std::uint64_t rng);
// _common.pyx kahan_sum — the compensated sum choice()'s p check uses.
[[nodiscard]] double kahan_sum(std::span<const double> values);

} // namespace detail

// ---------------------------------------------------------------------------
// NumpyRng — Generator(PCG64) seeded by SeedSequence. Every draw consumes
// the stream in NumPy's order; there is no copy because two generators may
// never alias one stream position.
// ---------------------------------------------------------------------------
class NumpyRng {
public:
    explicit NumpyRng(std::uint64_t seed);
    NumpyRng(std::string_view nonnegative_decimal_entropy,
             std::span<const std::uint32_t> spawn_key);
    explicit NumpyRng(const SeedSequence &sequence);
    ~NumpyRng();
    NumpyRng(NumpyRng &&) noexcept;
    NumpyRng &operator=(NumpyRng &&) noexcept;
    NumpyRng(const NumpyRng &) = delete;
    NumpyRng &operator=(const NumpyRng &) = delete;

    // BitGenerator.random_raw / the next_uint64 primitive.
    std::uint64_t raw64();
    // The buffered next_uint32 primitive (low half of a fresh word first).
    std::uint32_t raw32();
    // Generator.random scalar — uint64_to_double(next64()).
    double uniform01();
    // Generator.standard_normal scalar — the Ziggurat draw.
    double normal01();
    // Generator.random(size) / standard_normal(size): elementwise fills.
    void uniform01(std::span<double> out);
    void normal01(std::span<double> out);
    // Generator.uniform(low, high): low + (high-low)*next_double() with the
    // finite-range/'high - low < 0' checks.
    double uniform(double low, double high);
    void uniform(double low, double high, std::span<double> out);
    // Generator.normal(loc, scale): loc + scale*standard_normal() per
    // element ('scale < 0' check; scale=0 still draws).
    double normal(double loc, double scale);
    void normal(double loc, double scale, std::span<double> out);
    // Generator.integers(low, high, dtype=int64, endpoint=...): the scalar
    // and size-fill paths share one bounded-uint64 fill loop.
    std::int64_t integers(std::int64_t low, std::int64_t high,
                          bool endpoint = false);
    std::int64_t integers(std::int64_t high);
    void integers(std::int64_t low, std::int64_t high,
                  std::span<std::int64_t> out, bool endpoint = false);
    // Generator.choice: the uniform path is integers(0, population); the
    // p path is kahan_sum validation + cdf cumsum + right-side searchsorted
    // on one random() draw per sample.
    std::int64_t choice(std::int64_t population);
    std::int64_t choice(std::span<const double> probabilities);
    void choice(std::span<const double> probabilities,
                std::span<std::int64_t> out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ---------------------------------------------------------------------------
// sensor_noise.py — the seeded sensor-noise seam. SensorNoiseConfig carries
// the four distribution scales SensorConfig declares (the noise source's
// only config reads); NoiseTape and build_noise_tape mirror the source
// dataclass/factory verbatim, and SensorNoiseSource keeps the two spawned
// PCG64 children plus the fixed four-call draw order.
// ---------------------------------------------------------------------------
struct SensorNoiseConfig {
    double acceleration_std_mps2 = .05;
    double gyro_std_rad_s = .002;
    double encoder_std_rad_s = .01;
    double torque_std_nm = .05;
    // SensorConfig.__post_init__'s scalar(..., minimum=0.) checks for the
    // carried fields, in source order ('invalid {name}').
    void validate() const;
};

// sensor_noise.py NoiseTape — noise rows are fixed at the 9-element
// concatenation order (accel xyz, gyro, encoder xyz, torque xy); the dtype/
// shape leg is statically (count, 9) x (count,). Field layout mirrors
// runtime::NoiseTape so the pipeline boundary converts member-wise.
struct NoiseTape {
    std::vector<std::array<double, 9>> noise;
    std::vector<double> dropout_uniform;
    // __post_init__'s shape/finiteness/[0,1) checks in source order.
    void validate() const;
};

class SensorNoiseSource {
public:
    explicit SensorNoiseSource(SensorNoiseConfig config,
                               std::uint64_t seed = 0);
    // One draw(): the four normal calls in source order then the dropout
    // uniform — identical stream consumption every call.
    [[nodiscard]] std::pair<std::array<double, 9>, double> draw();
    [[nodiscard]] const SensorNoiseConfig &config() const noexcept {
        return config_;
    }
    [[nodiscard]] std::uint64_t seed() const noexcept { return seed_; }

private:
    // The __init__ path's two spawned children from one parent sequence.
    SensorNoiseSource(SensorNoiseConfig config, std::uint64_t seed,
                      const std::pair<SeedSequence, SeedSequence> &children);

    SensorNoiseConfig config_;
    std::uint64_t seed_;
    NumpyRng rng_;
    NumpyRng dropout_rng_;
};

// sensor_noise.py build_noise_tape(config, *, seed, count).
[[nodiscard]] NoiseTape build_noise_tape(const SensorNoiseConfig &config,
                                         std::uint64_t seed,
                                         std::int64_t count);

// ---------------------------------------------------------------------------
// rider_random.py — the seeded rider population. The (low, high) pairs are
// std::array<double, 2> (the "must be a (low, high) pair" leg is static);
// validate() applies _range's domain checks in __post_init__'s field order.
// The keyframe/program records mirror rider_program.py's dataclass fields
// exactly; their construction-time validators are owned by T34's program
// boundary, so sample_rider only produces structurally valid records.
// ---------------------------------------------------------------------------
struct RiderRandomSpec {
    std::array<double, 2> mass_kg = {60., 100.};
    std::array<double, 2> height_m = {1.60, 1.85};
    std::array<double, 2> torso_lean_rad = {0., .25};
    std::array<double, 2> pelvis_pitch_rad = {0., .08};
    std::array<double, 2> human_torque_nm = {0., 25.};
    void validate() const;
};

// rider_program.py RiderKeyframe — identical field layout to
// runtime::RiderKeyframe (kept local so this header stays free of the
// mujoco-dependent runtime samples chain).
struct RiderKeyframe {
    double time_s = 0.;
    rider::RiderPosture posture;
    std::optional<double> human_torque_nm;
};

// rider_program.py RiderProgram.
struct RiderProgram {
    std::vector<RiderKeyframe> keyframes;
    double reaction_delay_s = 0.;
};

struct SampledRider {
    configuration::RiderSpecs rider;
    RiderProgram program;
};

// rider_random.py sample_rider(spec, seed): np.random.default_rng(seed)
// then five uniform draws — mass, height, torso lean, pelvis pitch, human
// torque — in source order.
[[nodiscard]] SampledRider sample_rider(const RiderRandomSpec &spec,
                                       std::uint64_t seed);
[[nodiscard]] SampledRider sample_rider(const RiderRandomSpec &spec,
                                       const SeedSequence &seed);

} // namespace bike::random
