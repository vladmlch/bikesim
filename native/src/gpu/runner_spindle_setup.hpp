// native/src/gpu/runner_spindle_setup.hpp — Shared host-side helper for building
// SpindleM2K2Params<float> from (xml_path, strength_json_path, envelope_json_path)
// inside cpu_batch_runner.cpp, metal_batch_runner.mm, and GPU tests.
#pragma once

#if defined(__METAL_VERSION__) || defined(__CUDA_ARCH__)
#error "runner_spindle_setup.hpp is host-only"
#endif

#include "../physics_core/spindle_m2k2_upload.hpp"

#if defined(BIKE_GPU_STANDALONE_RUNNER)
#include "standalone_mujoco_shim.hpp"
#else
#include "../engine_call.hpp"
#endif

#include <mujoco/mujoco.h>

#include <array>
#include <filesystem>
#include <iostream>
#include <memory>

namespace bike::gpu {

inline int build_runner_spindle_params(
    const char *xml_path,
    const char *strength_json_path,
    const char *envelope_json_path,
    int control_period_steps,
    bike::physics_core::SpindleM2K2Params<float> *out_params
) noexcept {
    if (xml_path == nullptr || strength_json_path == nullptr ||
        envelope_json_path == nullptr || out_params == nullptr) {
        return -1;
    }
    try {
        std::array<char, 1024> err{};
        mjModel *m = mj_loadXML(xml_path, nullptr, err.data(), static_cast<int>(err.size()));
        if (m == nullptr) {
            std::cerr << "[build_runner_spindle_params] mj_loadXML failed: " << err.data() << '\n';
            return -2;
        }
        const auto m_guard = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>(m, &mj_deleteModel);
        mjData *d = engine::make_data(m);
        if (d == nullptr) {
            return -2;
        }
        const auto d_guard = std::unique_ptr<mjData, decltype(&mj_deleteData)>(d, &mj_deleteData);
        engine::forward(m, d);

        bike::physics_core::SpindleM2K2ScalarConfig cfg{};
        if (control_period_steps > 0) {
            cfg.control_period_steps = control_period_steps;
        }
        *out_params = bike::physics_core::build_spindle_m2k2_params_from_json_files<float>(
            m,
            d,
            std::filesystem::path(strength_json_path),
            std::filesystem::path(envelope_json_path),
            cfg);
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[build_runner_spindle_params] " << e.what() << '\n';
        return -1;
    }
}

} // namespace bike::gpu
