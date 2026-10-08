// config.hpp — the runtime_schema=1 configuration envelope.
//
// bike_sim.native.setup.capture_bootstrap emits this dict beside the model
// artifact and state inventory; NativeRideRuntime validates every field
// before constructing anything (design doc section 3.1). Nested subtrees
// without a native consumer yet (writer_config feeds the Stepper directly;
// rider_controller/rider_intent/pose feed the A2/A3 native owners) are kept
// as OWNED deep copies — the runtime never aliases caller objects.
#pragma once
#include <nanobind/nanobind.h>
#include <optional>
#include <string>
#include <string_view>

namespace runtime {

constexpr int kRuntimeSchema = 1;

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
    nanobind::object pose;  // owned dict copy or None
};

struct RuntimeConfig {
    double timestep_s = 0.;
    double control_period_s = 0.;
    int control_period_steps = 0;
    bool strict = false;
    int record_decimation = 0;
    MonitorConfig monitors;
    GeometryConfig geometry;
    nanobind::object writer_config;     // owned dict — the Stepper consumes it once
    nanobind::object rider_controller;  // owned dict — staged for the A2 controller
    nanobind::object rider_intent;      // owned dict — staged for the A2 scheduler
};

// Throws std::invalid_argument naming the full config path on any deviation
// from the runtime_schema=1 envelope.
[[nodiscard]] RuntimeConfig parse_runtime_config(nanobind::handle raw);

// Deep ownership copy of a plain-data tree (dict/list/tuple/str/bool/
// integral/real/ndarray/None). Immutable scalars are shared; containers and
// arrays are rebuilt so the caller cannot mutate the runtime's copy. Any
// other object type is rejected — staged state must never capture arbitrary
// __dict__ payloads.
[[nodiscard]] nanobind::object owned_copy(nanobind::handle value,
                                          std::string_view path);

} // namespace runtime
