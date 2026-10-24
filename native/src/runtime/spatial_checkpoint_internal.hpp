#pragma once
#include "spatial_checkpoint.hpp"
namespace runtime::spatial {
// Private cold transfer object; public frozen W00 header stays opaque.
struct CheckpointStage {
    SpatialCheckpoint checkpoint;
    RuntimeIdentity execution;
    bool consumed = false;
};
class CheckpointAccess {
public:
    [[nodiscard]] static bike::io::Value capture_bootstrap(const NativeRideRuntime&);
    [[nodiscard]] static std::unique_ptr<NativeRideRuntime> prepare(
        const SpatialCheckpoint&, const NativeRideRuntime&);
    static void commit(CheckpointStage&&, NativeRideRuntime&);
private:
    [[nodiscard]] static std::unique_ptr<NativeRideRuntime> prepare_locked(
        const SpatialCheckpoint&, const NativeRideRuntime&);
};
[[nodiscard]] SpatialCheckpoint capture_checkpoint(const NativeRideRuntime&,
    bike::io::Value research, bike::io::Value sensor_v2,
    bike::io::Value policy, bike::io::Value identity);
[[nodiscard]] bike::io::Value checkpoint_identity(const bike::io::Value& metadata);
[[nodiscard]] bike::io::Value inactive_research_checkpoint();
[[nodiscard]] bike::io::Value inactive_sensor_checkpoint();
// Comparison-only projection of the owned binary tyre image. Persistence
// remains the frozen bootstrap2 codec; no opaque base64 string is compared
// in place of its material-state floating-point and exact integer lanes.
[[nodiscard]] bike::io::Value bootstrap_comparison_value(const bike::io::Value&);
[[nodiscard]] bike::io::Value checkpoint_comparison_value(const bike::io::Value&);
} // namespace runtime::spatial
