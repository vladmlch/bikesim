// Completion 3.33a. Source: tests/reference/test_native_research_sensors.py.
// Sensor-pipeline delivery parity: three oracle-comparison records are frozen
// to per-platform goldens (noise/latency/dropout/stale, imported startup row,
// irregular delivery, same-seed reset), two records are self-contained state
// contracts.
//
// Substitutions:
//  * SensorConfig maps delivery-side only; the four noise scales live in
//    random::SensorNoiseConfig (ideal() == the zero tuple).
//  * `native.push(asdict(raw))` / `native.reset(asdict(raw))` pass dicts
//    through the binding's SensorObservation::from_wire; the C++ port calls
//    the same decoder on hand-built wire dicts, so the strict-bool and
//    timestamp validation legs are exercised verbatim. read(True) and the
//    ndarray dtype legs are nanobind gates with no C++ spelling — the
//    expressible rejections (negative/backwards/NaN delivery clocks,
//    mismatched tape shapes) hit the same validators.
//  * `native.import_state(reference.state_dict())` is replayed literally:
//    the recorded 'input.state' entry freezes the Python-produced state
//    dict, and unflatten_state rebuilds the identical SensorState the
//    binding's decoder would produce.
#include "research_reference_support.hpp"
#include "support/golden.hpp"

namespace {
using namespace research_reference;
namespace oracle = bt::oracle;

constexpr std::array<double, 3> kAccelBias{.5, -.25, .125};

// Inverse of the Python capture site's _flat_obs/_flat_state layout (the
// same order flatten_observation/flatten_state emit on this side).
[[nodiscard]] runtime::SensorObservation
unflatten_observation(std::span<const double> row) {
    runtime::SensorObservation out;
    out.time_s = row[0];
    out.source_time_s = row[1];
    out.valid = row[2] != 0.;
    std::copy_n(row.begin() + 3, 9, out.values.begin());
    return out;
}
[[nodiscard]] runtime::SensorState
unflatten_state(std::span<const double> row) {
    runtime::SensorState out;
    std::size_t offset = 0;
    const auto next = [&row, &offset]() -> double {
        bt::require(offset < row.size(),
                    "the recorded sensor state row is not truncated");
        const double value = row[offset];
        ++offset;
        return value;
    };
    const auto next_optional = [&next]() -> std::optional<double> {
        const bool present = next() != 0.;
        const double value = next();
        return present ? std::optional(value) : std::nullopt;
    };
    out.last_time = next_optional();
    out.delivery_time = next_optional();
    out.cursor = static_cast<std::size_t>(next());
    out.samples_attempted = static_cast<std::size_t>(next());
    out.samples_dropped = static_cast<std::size_t>(next());
    if (next() != 0.) {
        out.startup = unflatten_observation(row.subspan(offset, 12));
        offset += 12;
    }
    const auto count = static_cast<std::size_t>(next());
    for (std::size_t index = 0; index < count; ++index) {
        out.queue.push_back(unflatten_observation(row.subspan(offset, 12)));
        offset += 12;
    }
    bt::require(offset == row.size(),
                "the recorded sensor state row is fully consumed");
    return out;
}

[[nodiscard]] runtime::SensorConfig delivery(
    double latency, double dropout = 0., bool imu = true,
    double maximum_age = .015,
    std::array<double, 3> accel_bias = {0., 0., 0.}, double gyro_bias = 0.) {
    return {.latency_s = latency, .acceleration_bias = accel_bias,
            .gyro_bias = gyro_bias, .dropout_probability = dropout,
            .maximum_age_s = maximum_age, .imu_enabled = imu};
}

[[nodiscard]] runtime::WireObject raw_wire(double time_s = 0.,
                                           bool valid = true,
                                           std::optional<double> source_override = std::nullopt,
                                           bool valid_as_int = false) {
    const auto obs = raw_sensor(time_s);
    return {{"time_s", runtime::Wire{obs.time_s}},
        {"source_time_s", runtime::Wire{source_override.value_or(obs.source_time_s)}},
        {"valid", valid_as_int ? runtime::Wire{std::int64_t{1}} : runtime::Wire{valid}},
        {"specific_force_body_mps2", runtime::wire_array(std::span<const double>(obs.values).first<3>())},
        {"pitch_rate_up_rad_s", runtime::Wire{obs.values[3]}},
        {"front_wheel_rad_s", runtime::Wire{obs.values[4]}},
        {"rear_wheel_rad_s", runtime::Wire{obs.values[5]}},
        {"crank_rad_s", runtime::Wire{obs.values[6]}},
        {"motor_torque_nm", runtime::Wire{obs.values[7]}},
        {"human_torque_nm", runtime::Wire{obs.values[8]}}};
}

// tests/reference/test_native_research_sensors.py::
//   test_noise_latency_dropout_and_stale_match_python
void test_noise_latency_dropout_and_stale_match_python(
    const char *node_suffix, bool imu, double dropout, double latency) {
    oracle::Node node(
        "tests/reference/test_native_research_sensors.py::"
        "test_noise_latency_dropout_and_stale_match_python" +
        std::string(node_suffix));
    auto native = pipeline(delivery(latency, dropout, imu, .015, kAccelBias, .02),
                           default_noise(), 41, 32);
    native.reset(raw_sensor());
    for (std::int64_t index = 0; index < 24; ++index) {
        const double time_s = static_cast<double>(index) * .005;
        if (index) native.push(raw_sensor(time_s));
        const auto delivered = native.read(time_s);
        node.f64(flatten_observation(delivered), "read");
        const auto before = native.state_wire();
        const auto again = flatten_observation(native.read(time_s));
        bt::require(again == flatten_observation(delivered),
                    "a repeated read at the same delivery time returns the same row");
        bt::require(same_state(native.state_wire(), before),
                    "a repeated read leaves the pipeline state untouched");
        node.f64(flatten_state(native.state_wire()), "state");
    }
    const auto stale = native.read(1.);
    bt::require(!stale.valid, "a 1-second-old delivery is stale");
    node.f64(flatten_observation(stale), "read");
    node.f64(flatten_state(native.state_wire()), "state");
    node.done();
}

// tests/reference/test_native_research_sensors.py::
//   test_imported_startup_row_is_not_drawn_twice
void test_imported_startup_row_is_not_drawn_twice() {
    oracle::Node node(
        "tests/reference/test_native_research_sensors.py::"
        "test_imported_startup_row_is_not_drawn_twice");
    // reference.state_dict() after reset+read(0.), recorded on the oracle
    // side — import_state decodes the same state dict binding-side.
    const auto recorded = unflatten_state(node.f64_input("input.state"));
    auto imported = pipeline(delivery(0., 0., true, .1), default_noise(), 41, 32);
    imported.import_state(recorded);
    const auto first = imported.read(0.);
    node.f64(flatten_observation(first), "imported_read");
    bt::require(imported.state().cursor == 1,
                "the imported startup row does not draw the noise tape again");
    imported.push(raw_sensor(.005));
    const auto second = imported.read(.005);
    node.f64(flatten_observation(second), "read");
    bt::require(imported.state().cursor == 2,
                "the next push draws exactly one further noise row");
    node.done();
}

// tests/reference/test_native_research_sensors.py::
//   test_irregular_delivery_schedule_matches_python
void test_irregular_delivery_schedule_matches_python() {
    oracle::Node node(
        "tests/reference/test_native_research_sensors.py::"
        "test_irregular_delivery_schedule_matches_python");
    auto native = pipeline(delivery(.004, .15, true, .02,
                                    std::array<double, 3>{.05, -.01, .02}, .003),
                           default_noise(), 11, 8);
    native.reset(raw_sensor());
    std::vector<double> pushes{.005, .010, .015, .025, .030};
    const std::array<double, 8> reads{.0032, .0067, .0119, .0161, .0204, .0276, .0311, .045};
    for (const double read_at : reads) {
        while (!pushes.empty() && pushes.front() <= read_at) {
            native.push(raw_sensor(pushes.front()));
            pushes.erase(pushes.begin());
        }
        node.f64(flatten_observation(native.read(read_at)), "read");
        node.f64(flatten_state(native.state_wire()), "state");
    }
    node.done();
}

// tests/reference/test_native_research_sensors.py::
//   test_same_seed_reset_repeats_and_different_seeds_differ
void test_same_seed_reset_repeats_and_different_seeds_differ() {
    oracle::Node node(
        "tests/reference/test_native_research_sensors.py::"
        "test_same_seed_reset_repeats_and_different_seeds_differ");
    const auto sequence = [](runtime::SensorPipeline &pipe) {
        pipe.reset(raw_sensor());
        std::vector<double> flat;
        for (std::int64_t index = 1; index < 6; ++index) {
            const double time_s = static_cast<double>(index) * .005;
            pipe.push(raw_sensor(time_s));
            flatten_observation(pipe.read(time_s), flat);
        }
        return flat;
    };
    auto native = pipeline(delivery(0., .3, true, .1), default_noise(), 7, 6);
    const auto first = sequence(native);
    const auto replayed = sequence(native);
    bt::require(replayed == first,
                "reset replays the same seeded tape: identical deliveries");
    node.f64(first, "sequence");
    auto other = pipeline(delivery(0., .3, true, .1), default_noise(), 8, 6);
    const auto different = sequence(other);
    bt::require(different != first,
                "a different seed produces a different delivery sequence");
    node.done();
}

// tests/reference/test_native_research_sensors.py::
//   test_invalid_operations_and_exhaustion_preserve_state
void test_invalid_operations_and_exhaustion_preserve_state() {
    auto native = pipeline(delivery(0., 0., true, .1), ideal_noise(), 41, 2);
    native.reset(raw_sensor());
    (void)native.read(.01);
    auto before = native.state_wire();
    // asdict(raw_sensor()) at the startup timestamp is a duplicate-time push.
    bt::invalid([&] { native.push(raw_sensor(0.)); });
    bt::require(same_state(native.state_wire(), before),
                "a rejected duplicate-time push preserves the pipeline state");
    // dict(..., valid=1): strict bool decode — an integer is not a bool.
    bt::invalid([&] {
        (void)runtime::SensorObservation::from_wire(
            raw_wire(.005, true, std::nullopt, true)); });
    bt::require(same_state(native.state_wire(), before),
                "a rejected non-bool validity preserves the pipeline state");
    // dict(..., source_time_s=.01) against time_s=.005: a future source.
    bt::invalid([&] {
        (void)runtime::SensorObservation::from_wire(raw_wire(.005, true, .01)); });
    bt::require(same_state(native.state_wire(), before),
                "a rejected future-source push preserves the pipeline state");
    for (const double time_s : {-1., .009, std::numeric_limits<double>::quiet_NaN()}) {
        bt::invalid([&] { (void)native.read(time_s); });
        bt::require(same_state(native.state_wire(), before),
                    "a rejected delivery time preserves the pipeline state");
    }
    {
        auto tampered = native.state();
        tampered.cursor += 1;
        bt::invalid([&] { native.import_state(tampered); });
        bt::require(same_state(native.state_wire(), before),
                    "a rejected import preserves the pipeline state");
    }
    native.push(raw_sensor(.005));
    before = native.state_wire();
    throws_runtime([&] { native.push(raw_sensor(.01)); }, "exhausted");
    bt::require(same_state(native.state_wire(), before),
                "an exhausted tape rejects the push without losing state");
}

// Direct-construction helper for the ownership legs (the pipeline() helper
// always builds a fresh seeded tape).
[[nodiscard]] runtime::SensorPipeline pipeline_copy(const runtime::SensorConfig &config,
                                                    runtime::NoiseTape tape) {
    return runtime::SensorPipeline(config, std::move(tape));
}

// tests/reference/test_native_research_sensors.py::
//   test_native_owns_noise_and_validates_empty_shape
void test_native_owns_noise_and_validates_empty_shape() {
    runtime::NoiseTape tape{.noise = std::vector<std::array<double, 9>>(2),
                            .dropout_uniform = std::vector<double>(2, .5)};
    auto owned = tape;
    auto native = pipeline_copy(delivery(0., 0., true, .1), std::move(owned));
    for (auto &row : tape.noise) row.fill(100.);
    std::ranges::fill(tape.dropout_uniform, 0.);
    native.reset(raw_sensor());
    const auto delivered = native.read(0.);
    bt::require(delivered.values[7] == 5.,
                "the pipeline owns its tape copy: source mutation cannot reach it");
    auto empty = pipeline_copy(delivery(0., 0., true, .1), runtime::NoiseTape{});
    throws_runtime([&] { empty.reset(raw_sensor()); }, "exhausted");
    bt::require(empty.state().cursor == 0,
                "an exhausted empty tape never advances the cursor");
    invalid_with([] {
        runtime::NoiseTape bad{.noise = {std::array<double, 9>{}},
                               .dropout_uniform = {}};
        (void)runtime::SensorPipeline(delivery(0., 0., true, .1), std::move(bad)); },
                 "noise tape shapes");
}

} // namespace

int main() {
    return bt::run([] {
        for (const auto &[suffix, imu, dropout, latency] : {
            std::tuple{"[False-0.0-0.0]", false, 0., 0.},
            std::tuple{"[False-0.0-0.003]", false, 0., .003},
            std::tuple{"[False-0.0-0.02]", false, 0., .02},
            std::tuple{"[False-0.4-0.0]", false, .4, 0.},
            std::tuple{"[False-0.4-0.003]", false, .4, .003},
            std::tuple{"[False-0.4-0.02]", false, .4, .02},
            std::tuple{"[False-1.0-0.0]", false, 1., 0.},
            std::tuple{"[False-1.0-0.003]", false, 1., .003},
            std::tuple{"[False-1.0-0.02]", false, 1., .02},
            std::tuple{"[True-0.0-0.0]", true, 0., 0.},
            std::tuple{"[True-0.0-0.003]", true, 0., .003},
            std::tuple{"[True-0.0-0.02]", true, 0., .02},
            std::tuple{"[True-0.4-0.0]", true, .4, 0.},
            std::tuple{"[True-0.4-0.003]", true, .4, .003},
            std::tuple{"[True-0.4-0.02]", true, .4, .02},
            std::tuple{"[True-1.0-0.0]", true, 1., 0.},
            std::tuple{"[True-1.0-0.003]", true, 1., .003},
            std::tuple{"[True-1.0-0.02]", true, 1., .02}})
            test_noise_latency_dropout_and_stale_match_python(suffix, imu, dropout, latency);
        test_imported_startup_row_is_not_drawn_twice();
        test_invalid_operations_and_exhaustion_preserve_state();
        test_irregular_delivery_schedule_matches_python();
        test_same_seed_reset_repeats_and_different_seeds_differ();
        test_native_owns_noise_and_validates_empty_shape();
    });
}
