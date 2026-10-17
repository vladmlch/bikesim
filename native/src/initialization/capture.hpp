#pragma once

#include "../runtime/runtime.hpp"

class Stepper;
namespace bike::initialization {

[[nodiscard]] runtime::ModelImage capture_model_image(const mjModel&);
// Called only after initialization has established the t=0 accounting datum.
[[nodiscard]] runtime::RuntimeBootstrap capture_initialized(
    Stepper&, runtime::PhysicalStep&, const runtime::RuntimeConfig&);

} // namespace bike::initialization
