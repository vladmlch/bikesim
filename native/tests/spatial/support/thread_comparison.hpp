#pragma once

// W52 comparison-only projection. Persisted checkpoints remain strict about
// runtime execution policy; this helper never enters the production codec or
// restore path. Before masking the count-dependent digest, callers prove that
// the complete input RuntimeConfig differs only in tyre_threads.
#include "support/migration_test.hpp"
#include "runtime/config.hpp"
#include "runtime/setup_values.hpp"
#include "runtime/spatial_checkpoint_internal.hpp"
#include "spatial/config.hpp"
#include "terrain/source3d.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"

#include <algorithm>
#include <array>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace spatial_tests {
inline bike::io::Value thread_neutral_config(const runtime::RuntimeConfig& c) {
    using namespace runtime::setupvalue;
    if (!c.spatial || !c.terrain3d || c.tyre_threads == 0 || c.tyre_threads > 64)
        throw std::invalid_argument("thread comparison requires a spatial config and tyre_threads in [1,64]");
    const auto named = [](const std::optional<std::string>& value) {
        return value ? text(*value) : Value{};
    };
    // Explicit full RuntimeConfig inventory, including flags not belonging to
    // the physics coefficient digest. Keep this in step with config.hpp.
    return Value{Object{
        {"schema", integer(c.schema)}, {"timestep_s", real(c.timestep_s)},
        {"control_period_s", real(c.control_period_s)}, {"control_period_steps", integer(c.control_period_steps)},
        {"strict", Value{c.strict}}, {"async_telemetry", Value{c.async_telemetry}},
        {"tyre_threads", integer(1)}, {"measurement_decimation", integer(c.measurement_decimation)},
        {"record_retention", integer(static_cast<int>(c.record_retention))},
        {"writer_config", c.writer_config}, {"rider_controller", c.rider_controller}, {"rider_intent", c.rider_intent},
        {"spatial", bike::spatial::spatial_config_to_wire(*c.spatial)},
        {"monitors", Value{Object{
            {"balance_floor_mps", real(c.monitors.balance_floor_mps)},
            {"balance_dwell_s", real(c.monitors.balance_dwell_s)},
            {"balance_grace_s", real(c.monitors.balance_grace_s)},
            {"grounded_hold_s", real(c.monitors.grounded_hold_s)}}}},
        {"geometry", Value{Object{
            {"crank_length_m", real(c.geometry.crank_length_m)},
            {"front_brake_joint", text(c.geometry.front_brake.joint)},
            {"front_brake_dof", integer(c.geometry.front_brake.dof_index)},
            {"rear_brake_joint", text(c.geometry.rear_brake.joint)},
            {"rear_brake_dof", integer(c.geometry.rear_brake.dof_index)},
            {"front_wheel_body", named(c.geometry.front_wheel_body)},
            {"rear_wheel_body", named(c.geometry.rear_wheel_body)},
            {"com_marker_site", named(c.geometry.com_marker_site)}, {"pose", c.geometry.pose}}}},
        {"physical_terrain", Value{Object{
            {"kind", integer(static_cast<int>(c.terrain3d->kind()))},
            {"sha256", text(c.terrain3d->fingerprint_hex())},
            {"x_min_m", real(c.terrain3d->x_min_m())}, {"x_max_m", real(c.terrain3d->x_max_m())},
            {"half_width_m", real(c.terrain3d->half_width_m())}}}}}};
}

inline std::string require_thread_only_difference(const runtime::RuntimeConfig& reference,
                                                 const runtime::RuntimeConfig& candidate) {
    const auto expected = bike::io::canonical_json(thread_neutral_config(reference));
    const auto actual = bike::io::canonical_json(thread_neutral_config(candidate));
    bt::require(actual == expected,
        "thread-invariance configurations differ only in the explicitly excluded tyre thread count");
    return bike::io::sha256(std::as_bytes(std::span{expected}));
}

inline bike::io::Value thread_neutral_bootstrap(runtime::NativeRideRuntime& live,
                                               std::string_view neutral_config_sha256) {
    using namespace runtime::setupvalue;
    if (neutral_config_sha256.size() != 64 || !std::ranges::all_of(neutral_config_sha256, [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    })) throw std::invalid_argument("thread comparison requires the verified thread-neutral config digest");
    auto bootstrap = runtime::spatial::CheckpointAccess::capture_bootstrap(live);
    auto state = field(bootstrap, "runtime");
    auto continuation = field(state, "continuation");
    // Resolve the existing field first: never silently add a substitute for
    // a missing checkpoint owner. No state, force, history or sensor is masked.
    (void)bike::io::as_string(field(continuation, "runtime_config_sha256"), "runtime_config_sha256");
    set(continuation, "runtime_config_sha256", text(neutral_config_sha256));
    set(state, "continuation", std::move(continuation));
    set(bootstrap, "runtime", std::move(state));
    return bootstrap;
}
} // namespace spatial_tests
