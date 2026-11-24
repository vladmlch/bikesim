#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace bike::physics_core::test_metal {

struct MetalParityRequest {
    std::string physics_core_dir;
    std::vector<float> exp_inputs;
    std::vector<float> tanh_inputs;
    std::vector<float> hypot_a;
    std::vector<float> hypot_b;
    std::vector<float> interp_s;
    std::vector<float> interp_x;
    std::vector<float> interp_x0;
    std::vector<float> interp_y0;
    std::vector<float> interp_x1;
    std::vector<float> interp_y1;
    std::vector<float> dot3_a;
    std::vector<float> dot3_b;
    std::vector<float> ring_local;
    std::vector<float> ring_rmat;
};

struct MetalParityOutput {
    bool device_available = false;
    std::string device_name;
    std::string error_message;
    std::vector<float> exp_out;
    std::vector<float> tanh_out;
    std::vector<float> hypot_out;
    std::vector<float> interp_out;
    std::vector<float> dot3_out;
    std::vector<float> ring_out;
    float bike_real_1e_12 = 0.0f;
    std::uint32_t sizeof_tyre_model = 0;
    std::uint32_t sizeof_compliant_state = 0;
    std::uint32_t sizeof_station_state = 0;
};

[[nodiscard]] MetalParityOutput run_metal_parity_kernel(const MetalParityRequest &req);

} // namespace bike::physics_core::test_metal
