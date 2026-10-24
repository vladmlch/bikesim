#pragma once

#include "../runtime/runtime.hpp"

class Stepper;
namespace bike::initialization {

[[nodiscard]] runtime::ModelImage capture_model_image(const mjModel&);
// Called only after initialization has established the t=0 accounting datum.
[[nodiscard]] runtime::RuntimeBootstrap capture_initialized(
    Stepper const&, runtime::PhysicalStep const&, const runtime::RuntimeConfig&,
    const runtime::ModelImage* immutable_spatial_model = nullptr);
// W42 shares the same bootstrap2 encoders at a committed continuation boundary.
// It does not reset clocks, material or controller latches.
[[nodiscard]] bike::io::Value capture_current_state(
    Stepper const&, runtime::PhysicalStep const&, const runtime::RuntimeConfig&,
    const runtime::ModelImage& immutable_spatial_model);

} // namespace bike::initialization
