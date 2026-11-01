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

#include "engine_identity.hpp"
#include "io/float_format.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "platform/provenance.hpp"
#include "research/environment.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/bootstrap_io.hpp"
#include "runtime/control.hpp"
#include "runtime/provenance.hpp"
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
    std::string stamped_recipe;
    const auto *metadata = bio::find_member(object, "metadata");
    if (metadata != nullptr) {
        auto restored = *metadata;
        // summarize()/identity_receipt()/the export's summary-vs-recipe
        // consistency gate read the same construction-time stamps
        // configuration_metadata seats in a real native env: the
        // experiment/sensor_config serializations (oracle-identical), the
        // settled owner's setup.equilibrium doc, the building binary's
        // source manifest + configuration digest, and the live execution
        // doc. Nothing changes during the run, so the receipt evaluates
        // all-false — exactly like create_native_research on an unmutated
        // environment.
        for (const auto *key : {"experiment", "sensor_config"}) {
            if (const auto *member = bio::find_member(object, key))
                runtime_reference::set_member(restored, key, *member);
        }
        if (const auto *equilibrium = bio::find_member(object, "equilibrium"))
            runtime_reference::set_member(
                restored, "setup",
                bio::Value{bio::Value::Object{
                    {"equilibrium", *equilibrium}}});
        // physical_configuration_metadata's construction-time stamps.
        const auto identity = runtime::runtime_identity();
        const auto engine = engine::identity();
        const auto build = bike::platform::build_identity();
        const auto text = [](std::string_view value) {
            return bio::Value{std::string(value)};
        };
        runtime_reference::set_member(
            restored, "model_source_sha256",
            text(identity.native_source_sha256));
        // configuration_digest() — metadata.cpp's re-hashing twin: the
        // articulated joint paths substitute their recorded digests before
        // the canonical serialization is hashed.
        {
            auto hashable = runtime_reference::member(restored,
                                                    "resolved_config");
            auto physics = runtime_reference::member(hashable, "physics");
            auto articulated =
                runtime_reference::member(physics, "articulated");
            const auto &resolved_object = bio::as_object(
                hashable, "resolved_config");
            for (const auto *key : {"joint_envelope", "joint_strength"}) {
                const auto *digest = bio::find_member(
                    resolved_object, std::string(key) + "_sha256");
                runtime_reference::set_member(
                    articulated, std::string(key) + "_path",
                    digest ? *digest : bio::Value{});
            }
            runtime_reference::set_member(physics, "articulated",
                                          std::move(articulated));
            runtime_reference::set_member(hashable, "physics",
                                          std::move(physics));
            const auto canonical = bio::canonical_json(hashable);
            runtime_reference::set_member(
                restored, "configuration_sha256",
                text(bio::sha256(std::as_bytes(
                    std::span{canonical.data(), canonical.size()}))));
        }
        bio::Value const execution{bio::Value::Object{
            {"backend", text("native")},
            {"native_source_sha256", text(identity.native_source_sha256)},
            {"execution_identity", text(bike::platform::execution_identity())},
            {"mujoco_library_sha256", text(engine.library_sha256)},
            {"numeric_provider", text(engine.numeric_provider)},
            {"executable_sha256", text(build.executable_sha256)},
            {"source_identity_scope",
             text("compiled first-party source manifest; running "
                  "image hashed separately")}}};
        runtime_reference::set_member(restored, "execution", execution);
        bootstrap.config.metadata = std::move(restored);
        // The env's internal recipe asset stamps the same execution doc —
        // the export's summary-vs-recipe gate compares the two. The
        // stamped bytes are handed to the asset loader below.
        const fs::path recipe_path = dir(name) / "assets" / "recipe.json";
        if (fs::exists(recipe_path)) {
            auto recipe = bio::parse_json(bt::text(recipe_path));
            runtime_reference::set_member(recipe, "execution", execution);
            stamped_recipe = bio::canonical_json(recipe) + '\n';
        }
    }
    // assets.json + assets/ restore the immutable input asset set
    // snapshot_research_assets stamped at capture — legacy/full exports
    // require config.assets; the frozen research.json omits it.
    const fs::path manifest_path = dir(name) / "assets.json";
    if (fs::exists(manifest_path)) {
        auto assets = std::make_shared<std::vector<runtime::ResearchAsset>>();
        const bio::Value manifest = bio::parse_json(bt::text(manifest_path));
        for (const auto &entry :
             bio::as_array(manifest, "assets manifest")) {
            const auto &fields = bio::as_object(entry, "asset entry");
            runtime::ResearchAsset asset;
            asset.name =
                bio::as_string(*bio::find_member(fields, "name"), "name");
            asset.media_type = bio::as_string(
                *bio::find_member(fields, "media_type"), "media_type");
            asset.sha256 = bio::as_string(
                *bio::find_member(fields, "sha256"), "sha256");
            const std::string bytes =
                (asset.name == "recipe.json" && !stamped_recipe.empty())
                    ? stamped_recipe
                    : bt::text(dir(name) / "assets" / asset.name);
            asset.content.reserve(bytes.size());
            for (const char unit : bytes)
                asset.content.push_back(static_cast<std::byte>(unit));
            if (asset.name == "recipe.json" && !stamped_recipe.empty())
                asset.sha256 = bio::sha256(asset.content);
            assets->push_back(std::move(asset));
        }
        bootstrap.config.assets = std::move(assets);
    }
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
                 std::optional<double> dropout = std::nullopt,
                 std::optional<runtime::ResearchPrograms> programs =
                     std::nullopt,
                 std::optional<double> weld_foot_min_normal_n =
                     std::nullopt) {
    auto bootstrap = research_bootstrap(name);
    if (capture) bootstrap.config.capture_mode = *capture;
    if (programs) bootstrap.config.programs = std::move(*programs);
    if (weld_foot_min_normal_n) {
        // violating_physics()'s weld_foot_min_normal_n=100 trigger rides in
        // the rider_controller document — the same doc the runtime ctor
        // decodes into rider::SpindleConfig.
        runtime_reference::set_member(
            bootstrap.ride.config.rider_controller,
            "weld_foot_min_normal_n",
            bio::Value{bio::Number{
                bio::repr_binary64(*weld_foot_min_normal_n)}});
    }
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
    if (stop_on_model_violation) {
        bootstrap.config.stop_on_model_violation = *stop_on_model_violation;
        // The research runtime requires its strictness flag to match the
        // physical runtime's — patch the ride config and the staged
        // state.runtime document together, like the decimation leg.
        bootstrap.ride.config.strict = *stop_on_model_violation;
        auto monitor = runtime_reference::member(
            bootstrap.ride.state.runtime, "monitor");
        runtime_reference::set_member(
            monitor, "strict", bio::Value{*stop_on_model_violation});
        runtime_reference::set_member(
            bootstrap.ride.state.runtime, "monitor", std::move(monitor));
    }
    if (energy_residual)
        bootstrap.config.maximum_energy_residual_ratio = *energy_residual;
    if (seed) bootstrap.config.seed_decimal = *seed;
    // The experiment/sensor_config documents embedded in metadata are
    // construction-time serializations of the request — the same knobs the
    // config patches above change, in document units (seconds, decimals).
    if (std::holds_alternative<bio::Value::Object>(
            bootstrap.config.metadata.data)) {
      const auto patch_member =
          [&](std::string_view member, std::string_view key, bio::Value value) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
            auto doc =
                runtime_reference::member(bootstrap.config.metadata, member);
            runtime_reference::set_member(doc, std::string(key),
                                          std::move(value));
            runtime_reference::set_member(bootstrap.config.metadata,
                                          std::string(member),
                                          std::move(doc));
          };
      const auto real = [](double value) {
        return bio::Value{bio::Number{bio::repr_binary64(value)}};
      };
      if (decimation)
        patch_member("experiment", "record_decimation",
                     runtime_reference::num(*decimation));
      if (control_steps)
        patch_member("experiment", "control_period_s",
                     real(static_cast<double>(*control_steps) *
                          bootstrap.config.timestep_s));
      if (sensor_steps) {
        patch_member("experiment", "sensor_period_s",
                     real(static_cast<double>(*sensor_steps) *
                          bootstrap.config.timestep_s));
        patch_member("sensor_config", "sample_period_s",
                     real(static_cast<double>(*sensor_steps) *
                          bootstrap.config.timestep_s));
      }
        if (max_steps)
            patch_member("experiment", "duration_s",
                         real(static_cast<double>(*max_steps) *
                              bootstrap.config.timestep_s));
        if (delay_steps)
            patch_member("experiment", "actuator_delay_s",
                         real(static_cast<double>(*delay_steps) *
                              bootstrap.config.timestep_s));
        if (stop_on_model_violation)
            patch_member("experiment", "stop_on_model_violation",
                         bio::Value{*stop_on_model_violation});
        if (energy_residual)
            patch_member("experiment", "maximum_energy_residual_ratio",
                         real(*energy_residual));
        if (seed)
            patch_member("experiment", "seed",
                         runtime_reference::num(
                             std::stoll(*seed)));
    }
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
            sensors, bootstrap.tape, (bootstrap.config.raw_initial).value()); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
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
                    flatten_into(child, path + "." + name, leaves); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
            } else if constexpr (std::is_same_v<T, runtime::WireArray>) {
                if (value.empty()) {
                    bio::Value::Object marker;
                    marker.emplace_back("$empty", bio::Value{"array"});
                    leaves.other.emplace(path, bio::Value{std::move(marker)});
                }
                for (std::size_t index = 0; index < value.size(); ++index) {
                    char suffix[16]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
                    std::snprintf(suffix, sizeof(suffix), "[%08zu]", index); // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                    flatten_into(value[index], path + suffix, leaves); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
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
    const std::size_t shape[1] = {canonical.size()}; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
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
    const std::size_t shape[1] = {canonical.size()}; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    node.bytes(bytes, "|u1", shape, label);
}

} // namespace ft2_reference
