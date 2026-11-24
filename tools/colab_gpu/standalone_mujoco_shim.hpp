// Standalone MuJoCo wrapper shim used only by tools/colab_gpu Python JIT builds
// (cpu_batch_sweep.py / metal_hybrid_sweep.py) linking upstream PyPI libmujoco
// without CMake's bike_engine / engine_call.cpp custom TLS error-frame objects.
#pragma once

#if !defined(BIKE_GPU_STANDALONE_RUNNER)
#error "standalone_mujoco_shim.hpp is only for standalone Colab/Python builds"
#endif

#include <mujoco/mujoco.h>

namespace engine {

inline mjData *make_data(const mjModel *model) {
    return mj_makeData(model);
}

inline void reset_data(const mjModel *model, mjData *data) {
    mj_resetData(model, data);
}

inline void forward(const mjModel *model, mjData *data) {
    mj_forward(model, data);
}

inline void step(const mjModel *model, mjData *data) {
    mj_step(model, data);
}

inline void kinematics(const mjModel *model, mjData *data) {
    mj_kinematics(model, data);
}

} // namespace engine
