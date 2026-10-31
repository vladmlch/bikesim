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
