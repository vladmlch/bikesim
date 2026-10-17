// config.hpp — the runtime_schema=1 configuration envelope (task T09).
//
// bike_sim.native.setup.capture_bootstrap emits this dict beside the model
// artifact and state inventory; NativeRideRuntime validates every field
// before constructing anything (design doc section 3.1). Nested subtrees
// whose decode positions sit later than the envelope validation
// (writer_config feeds the Stepper construction, rider_controller/
// rider_intent/pose feed the physical-step and accounting assemblies) are
// kept as OWNED bike::io::Value documents — decoded at the same ordinal
// positions the nanobind build ran them, never aliases of caller objects.
#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "../io/value.hpp"

namespace runtime {

constexpr int kRuntimeSchema = 1;

// Recording retention policy, independent of the measurement cadence
// (measurement_decimation): which committed immutable samples the
// accounting layer keeps for episode sealing. `legacy` reproduces the
// historical decimated-recorder projection (interval_id %
// measurement_decimation == 0); `full` retains every committed interval;
// `off` keeps no optional growing history.
enum class RetentionMode : std::uint8_t { off, full, legacy };

struct MonitorConfig {
    double balance_floor_mps = 0.;
    double balance_dwell_s = 0.;
    double balance_grace_s = 0.;
    double grounded_hold_s = 0.;
};

struct JointReference {
    std::string joint;
    int dof_index = 0;
};

struct GeometryConfig {
    double crank_length_m = 0.;
    JointReference front_brake;
    JointReference rear_brake;
    std::optional<std::string> front_wheel_body;
    std::optional<std::string> rear_wheel_body;
    std::optional<std::string> com_marker_site;
    // Owned config.geometry.pose document — a null Value spells the
    // absent-rider None exactly like the nanobind boundary. Decoded into
    // rider::SpindlePose inside the physical-step assembly.
    bike::io::Value pose;
};

struct RuntimeConfig {
    double timestep_s = 0.;
    double control_period_s = 0.;
    int control_period_steps = 0;
    bool strict = false;
    // Owned raw periods are evaluated on a data-only worker. Publication is
    // applied at the following main-thread boundary; explicit flush drains it.
    bool async_telemetry = false;
    // The expensive energy/momentum measurement cadence — historically the
    // `record_decimation` wire/argument value, now named for what it gates.
    int measurement_decimation = 0;
    RetentionMode record_retention = RetentionMode::legacy;
    MonitorConfig monitors;
    GeometryConfig geometry;
    // Owned config.writer_config document — the runtime ctor decodes it
    // into nativecfg::WriterConfig at the same position the nanobind
    // adapter did (immediately before the Stepper's model load).
    bike::io::Value writer_config;
    // Owned config.rider_controller document — decoded into
    // rider::SpindleConfig inside the physical-step assembly and re-read
    // for the accounting attachment budget at accounting init.
    bike::io::Value rider_controller;
    // Owned config.rider_intent document — decoded into
    // rider::IntentConfig inside the physical-step assembly.
    bike::io::Value rider_intent;
};

} // namespace runtime
