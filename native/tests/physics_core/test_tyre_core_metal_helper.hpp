#pragma once

#include <string>

namespace bike::physics_core::test_tyre_metal {

struct MetalTyreCompileResult {
    bool device_available = false;
    bool compiled = false;
    bool executed = false;
    std::string device_name;
    std::string error_message;
    float compliant_xi = 0.0f;
    float compliant_qfrc0 = 0.0f;
    float distributed_xi0 = 0.0f;
    float distributed_qfrc0 = 0.0f;
};

[[nodiscard]] MetalTyreCompileResult
run_metal_tyre_smoke(const std::string &physics_core_dir, bool include_distributed);

} // namespace bike::physics_core::test_tyre_metal
