// ft2_reference_support.hpp — committed-bootstrap plumbing + golden-tree
// consumption for the FT-2 (lane/f2) reference ports.
//
// Fixture story: tests/fixtures/migration/ft2/<platform-key>/ holds three
// oracle captures produced by tools/migration/_ft2_capture.py under the
// reference-test kernel pinning —
//   ride/            make_python_ride(strict=False, decimation=1): welded
//                    physics + rough_uphill_savage (runtime step/commands/
//                    rollout, welded viewer leg).
//   research_flat/   environment_factory() defaults: welded physics, flat,
//                    duration .04, seed 19, decimation 1, delay .005,
//                    diagnostic limits, capture_mode legacy — plus
//                    provenance.json (seed, raw startup observation, noise
//                    stds, metadata, experiment and sensor documents) and
//                    the native-stamped recording assets/.
//   research_savage/ make_python_research(): welded + rough_uphill_savage,
//                    duration .1, seed 210, diagnostic, assist, legacy.
//
// Golden-tree story: tests/reference/_golden.py::record_tree emits two
// entries per call — "<label>.values" (close f8: every numeric leaf in
// sorted-path order) and "<label>.other" (bitwise |u1: canonical JSON of
// the nonnumeric leaves as {path: leaf}, empty containers marked
// {"$empty": "object"|"array"}). consume_tree() below walks a runtime
// Wire tree under the identical path scheme — ".member" for objects,
// "[00000042]" for arrays — and consumes the pair in order.
#pragma once

#include "support/golden.hpp"
#include "support/migration_test.hpp"
#include "runtime_reference_support.hpp"

#include "diag.hpp"

#include "io/json.hpp"
#include "io/value.hpp"
#include "research/environment.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/bootstrap_io.hpp"
#include "runtime/control.hpp"
#include "runtime/research.hpp"
#include "runtime/research_io.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/sensors.hpp"
#include "runtime/value_readers.hpp"

#include <charconv>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ft2_reference {

namespace bio = bike::io;
namespace fs = std::filesystem;

// ---- fixture directories -------------------------------------------------

[[nodiscard]] inline fs::path dir(std::string_view name) {
    return bt::fixture("ft2") / std::string(bt::oracle::platform()) /
           std::string(name);
}

// RuntimeBootstrap is several KB; materializing it by value on the
// caller's frame trips -Wframe-larger-than, so the decode happens on the
// heap like the T09 decode frames.
[[nodiscard]] inline std::unique_ptr<runtime::NativeRideRuntime> ride() {
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        runtime::read_bootstrap(dir("ride")));
    return std::make_unique<runtime::NativeRideRuntime>(
        std::move(*bootstrap));
}

[[nodiscard]] inline bio::Value provenance(std::string_view name) {
    const bio::Value document =
        bio::parse_json(bt::text(dir(name) / "provenance.json"));
    return runtime::resolve_bootstrap_document(document, dir(name));
}

// Loads one research capture and re-seats the cold-path fields the frozen
// research.json deliberately omits: the seed, the raw startup observation
// (reseed provenance), the noise stds (tape regeneration), and the
// configuration metadata.
[[nodiscard]] inline runtime::ResearchBootstrap
research_bootstrap(std::string_view name) {
    runtime::ResearchBootstrap bootstrap =
        runtime::read_research_bootstrap(dir(name));
    const bio::Value meta = provenance(name);
    const auto &object = bio::as_object(meta, "ft2 provenance");
    bootstrap.config.seed_decimal = bio::as_string(
        *bio::find_member(object, "seed_decimal"), "seed_decimal");
    const auto *raw = bio::find_member(object, "raw_initial");
    if (raw != nullptr) {
        bootstrap.config.raw_initial = runtime::SensorObservation::from_wire(
            runtime::sample_wire::object(vread::wire_from_value(
                *raw, "provenance.raw_initial")));
    }
    const auto *stds = bio::find_member(object, "noise_std");
    const auto &scale_list = bio::as_array(*stds, "noise_std");
    if (scale_list.size() != bootstrap.config.noise_std.size())
        throw std::invalid_argument("ft2 provenance noise_std width");
    for (std::size_t index = 0; index < scale_list.size(); ++index)
        bootstrap.config.noise_std[index] =
            bio::as_double(scale_list[index], "noise_std");
    const auto *metadata = bio::find_member(object, "metadata");
    if (metadata != nullptr) bootstrap.config.metadata = *metadata;
    return bootstrap;
}

[[nodiscard]] inline std::unique_ptr<runtime::NativeResearchRuntime>
research(std::string_view name) {
    return runtime::make_research(research_bootstrap(name));
}

// A research capture with test-variant patches applied — the same legs
// environment_factory's CLI overrides run before capture_bootstrap():
// record_decimation forwards to both the research stream decimation and
// the ride's measurement cadence.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[nodiscard]] inline std::unique_ptr<runtime::NativeResearchRuntime>
research_variant(std::string_view name,
                 std::optional<runtime::ResearchCaptureMode> capture =
                     std::nullopt,
                 std::optional<int> decimation = std::nullopt,
                 std::optional<std::int64_t> max_steps = std::nullopt,
                 std::optional<std::int64_t> control_steps = std::nullopt,
                 std::optional<std::int64_t> sensor_steps = std::nullopt,
                 std::optional<std::int64_t> delay_steps = std::nullopt,
                 std::optional<bool> stop_on_model_violation = std::nullopt,
                 std::optional<double> energy_residual = std::nullopt,
                 std::optional<std::string> seed = std::nullopt,
                 std::optional<std::array<double, 3>> accel_bias =
                     std::nullopt,
                 std::optional<double> gyro_bias = std::nullopt,
                 std::optional<double> dropout = std::nullopt) {
    auto bootstrap = research_bootstrap(name);
    if (capture) bootstrap.config.capture_mode = *capture;
    if (accel_bias) bootstrap.config.sensors.acceleration_bias = *accel_bias;
    if (gyro_bias) bootstrap.config.sensors.gyro_bias = *gyro_bias;
    if (dropout) bootstrap.config.sensors.dropout_probability = *dropout;
    if (decimation) {
        bootstrap.config.record_decimation = *decimation;
        bootstrap.ride.config.measurement_decimation = *decimation;
        // The runtime ctor re-verifies the decoded config against the
        // staged state.runtime document — patch both, like ride_decimated
        // does on the unresolved docs.
        runtime_reference::set_member(
            bootstrap.ride.state.runtime, "measurement_decimation",
            runtime_reference::num(*decimation));
    }
    if (max_steps) bootstrap.config.max_steps = *max_steps;
    if (control_steps) bootstrap.config.control_steps = *control_steps;
    if (sensor_steps) bootstrap.config.sensor_steps = *sensor_steps;
    if (delay_steps) bootstrap.config.delay_steps = *delay_steps;
    if (stop_on_model_violation)
        bootstrap.config.stop_on_model_violation = *stop_on_model_violation;
    if (energy_residual)
        bootstrap.config.maximum_energy_residual_ratio = *energy_residual;
    if (seed) bootstrap.config.seed_decimal = *seed;
    // A longer episode, a different sensor cadence or a different seed
    // needs a regenerated tape; a different delivery config (the Python
    // _noise seam's bias/dropout injection) only re-runs the pipeline's
    // delivered startup state — the same legs environment.cpp runs at
    // composition time. The raw startup observation is restored from the
    // provenance the capture froze into config.raw_initial.
    const bool retape = seed || sensor_steps || max_steps;
    if (retape || accel_bias || gyro_bias || dropout) {
        bike::research::SensorConfig sensors;
        sensors.delivery = bootstrap.config.sensors;
        sensors.sample_period_s =
            static_cast<double>(bootstrap.config.sensor_steps) *
            bootstrap.config.timestep_s;
        sensors.noise = bike::random::SensorNoiseConfig{
            .acceleration_std_mps2 = bootstrap.config.noise_std[0],
            .gyro_std_rad_s = bootstrap.config.noise_std[1],
            .encoder_std_rad_s = bootstrap.config.noise_std[2],
            .torque_std_nm = bootstrap.config.noise_std[3]};
        bt::require(bootstrap.config.raw_initial.has_value(),
                    "the ft2 bootstrap carries the raw startup observation");
        if (retape) {
            const std::int64_t count =
                1 + bootstrap.config.max_steps / bootstrap.config.sensor_steps;
            bootstrap.tape = bike::research::make_sensor_tape(
                sensors, bootstrap.config.seed_decimal, count);
        }
        bootstrap.startup = bike::research::startup_sensor_state(
            sensors, bootstrap.tape, *bootstrap.config.raw_initial);
    }
    return runtime::make_research(std::move(bootstrap));
}
NATIVE_DIAG_POP

// The ft2 ride capture with a patched measurement decimation — the same
// set_measurement_decimation() leg create_native_ride(record_decimation=N)
// runs before capture_bootstrap. Kept here so the rollout ports can replay
// decimation-7 oracle rows against the platform capture.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[nodiscard]] inline std::unique_ptr<runtime::NativeRideRuntime>
ride_decimated(int decimation) {
    if (decimation == 1) return ride();
    const fs::path capture = dir("ride");
    auto docs = runtime_reference::resolved_docs(capture);
    runtime_reference::set_member(docs.config, "measurement_decimation",
                                  runtime_reference::num(decimation));
    runtime_reference::set_member(
        runtime_reference::member(docs.state, "runtime"),
        "measurement_decimation", runtime_reference::num(decimation));
    return runtime_reference::compose_runtime(docs.config, docs.state,
                                              capture);
}
NATIVE_DIAG_POP

// ---- golden-tree consumption ----------------------------------------------

struct TreeLeaves {
    // path -> flattened numeric leaf values, iterated in sorted-path order.
    std::map<std::string, std::vector<double>> numeric;
    // path -> nonnumeric leaf (null/bool/string) or {"$empty": kind}.
    std::map<std::string, bio::Value> other;
};

inline void flatten_into(const runtime::Wire &wire, const std::string &path,
                         TreeLeaves &leaves) {
    std::visit(
        [&](const auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, runtime::WireObject>) {
                if (value.empty()) {
                    bio::Value::Object marker;
                    marker.emplace_back("$empty", bio::Value{"object"});
                    leaves.other.emplace(path, bio::Value{std::move(marker)});
                }
                for (const auto &[name, child] : value)
                    flatten_into(child, path + "." + name, leaves);
            } else if constexpr (std::is_same_v<T, runtime::WireArray>) {
                if (value.empty()) {
                    bio::Value::Object marker;
                    marker.emplace_back("$empty", bio::Value{"array"});
                    leaves.other.emplace(path, bio::Value{std::move(marker)});
                }
                for (std::size_t index = 0; index < value.size(); ++index) {
                    char suffix[16];
                    std::snprintf(suffix, sizeof(suffix), "[%08zu]", index);
                    flatten_into(value[index], path + suffix, leaves);
                }
            } else if constexpr (std::is_same_v<T, std::int64_t>) {
                leaves.numeric[path] = {static_cast<double>(value)};
            } else if constexpr (std::is_same_v<T, double>) {
                leaves.numeric[path] = {value};
            } else if constexpr (std::is_same_v<T, std::monostate>) {
                leaves.other.emplace(path, bio::Value{});
            } else if constexpr (std::is_same_v<T, bool> ||
                                 std::is_same_v<T, std::string>) {
                leaves.other.emplace(path, bio::Value{value});
            }
        },
        wire.value);
}

// Consumes the record_tree pair at the cursor: "<label>.values" (close)
// then "<label>.other" (canonical-JSON bytes). The actual side is any
// runtime::Wire-compatible tree — ResearchTransition::as_wire(),
// SensorObservation::as_wire(), asdict-equivalent documents.
inline void consume_tree(bt::oracle::Node &node, const runtime::Wire &actual,
                         std::string_view label) {
    TreeLeaves leaves;
    flatten_into(actual, "", leaves);
    std::vector<double> flat;
    for (const auto &[path, values] : leaves.numeric)
        for (const double value : values) flat.push_back(value);
    node.f64_close(flat, std::string(label) + ".values");
    bio::Value::Object map;
    for (auto &[path, leaf] : leaves.other)
        map.emplace_back(path, std::move(leaf));
    const std::string canonical = bio::canonical_json(bio::Value{std::move(map)});
    const auto bytes = std::as_bytes(std::span<const char>(canonical));
    const std::size_t shape[1] = {canonical.size()};
    node.bytes(bytes, "|u1", shape, std::string(label) + ".other");
}

inline void consume_tree(bt::oracle::Node &node,
                         const runtime::WireObject &object,
                         std::string_view label) {
    consume_tree(node, runtime::Wire{object}, label);
}

// An io::Value document is not a Wire; cold-path oracle surfaces (recording
// exports, provenance docs) compare through canonical bytes instead.
inline void consume_canonical(bt::oracle::Node &node,
                              const bio::Value &actual,
                              std::string_view label) {
    const std::string canonical = bio::canonical_json(actual);
    const auto bytes = std::as_bytes(std::span<const char>(canonical));
    const std::size_t shape[1] = {canonical.size()};
    node.bytes(bytes, "|u1", shape, label);
}

} // namespace ft2_reference
