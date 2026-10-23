#pragma once

#include "provenance.hpp"
#include "runtime.hpp"
#include "../io/value.hpp"

#include <cstdint>
#include <memory>

namespace runtime::spatial {

inline constexpr std::uint32_t kSpatialCheckpointSchema = 1;

struct SpatialCheckpoint {
    std::uint32_t schema = kSpatialCheckpointSchema;
    // bootstrap2 owns engine/tyre/brake/rider/steer/outcome state exactly once.
    bike::io::Value bootstrap2;
    bike::io::Value research;
    bike::io::Value sensor_v2;
    bike::io::Value policy;
    bike::io::Value identity;
};

// Strict exact-key codecs; invalid input is rejected before live mutation.
bike::io::Value encode_checkpoint(const SpatialCheckpoint&);
SpatialCheckpoint decode_checkpoint(const bike::io::Value&);

struct CheckpointStage; // W42 owns the validated, uncommitted implementation.
// Destruction stays in W42's TU, where the opaque staged owners are complete.
struct CheckpointStageDeleter {
    void operator()(CheckpointStage*) const noexcept;
};
using CheckpointStageOwner = std::unique_ptr<CheckpointStage, CheckpointStageDeleter>;
CheckpointStageOwner stage_checkpoint(
    const SpatialCheckpoint&, const ::runtime::RuntimeIdentity&);
// W42 must provide a nonthrowing commit after complete staging; this declaration
// does not assert noexcept until its implementation establishes that guarantee.
void commit_checkpoint(CheckpointStage&&, ::runtime::NativeRideRuntime&);

} // namespace runtime::spatial
