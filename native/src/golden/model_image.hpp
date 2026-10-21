#pragma once

#include <mujoco/mujoco.h>
#include <cstddef>
#include <vector>

namespace bike::golden::detail {
// Cold serialization of the LIVE model, including mutable coefficients.
// The setup image is not a substitute for Python save(env.sim.model).
[[nodiscard]] std::vector<std::byte> model_image(const mjModel &model);
}
