// native/src/gpu/cuda_kernels.cu — CUDA __global__ kernels and extern "C" host
// entry points for ctypes / mujoco_warp integration on NVIDIA GPUs.

#include "batch_step.hpp"

#if defined(__CUDACC__)
#include <cuda_runtime.h>
#endif

namespace bike::gpu {

#if defined(__CUDACC__)

BIKE_GLOBAL void bike_gpu_init_states_kernel(
    int nworld,
    const GpuModelIndices *d_idx,
    const GpuWorldConfig *d_configs,
    int per_world_config,
    GpuWorldState *d_states,
    const float *d_qpos
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    const GpuWorldConfig &cfg = d_configs[per_world_config ? w : 0];
    const float *qpos_w = (d_qpos != nullptr) ? (d_qpos + w * idx.nq) : nullptr;
    const float crank_qpos = (qpos_w != nullptr) ? qpos_w[idx.crank_spin_qpos] : 0.0f;
    const float rear_wheel_qpos = (qpos_w != nullptr) ? qpos_w[idx.rear_wheel_spin_qpos] : 0.0f;
    gpu_init_world_state(d_states[w], cfg.drivetrain, crank_qpos, rear_wheel_qpos);
}

BIKE_GLOBAL void bike_gpu_writers_kernel(
    int nworld,
    float dt_s,
    const GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    float hf_inv_dx,
    const GpuWorldConfig *d_configs,
    int per_world_config,
    GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    const GpuHeightfield1D hf{
        d_hf_heights,
        hf_num_samples,
        hf_x_min,
        hf_dx,
        hf_inv_dx
    };
    gpu_step_world_writers(
        w,
        dt_s,
        idx,
        hf,
        d_configs,
        per_world_config,
        d_states,
        d_qpos,
        d_qvel,
        d_xpos,
        d_qfrc_applied,
        d_ctrl,
        d_tendon_range,
        tendon_range_world_stride
    );
}

#endif // defined(__CUDACC__)

} // namespace bike::gpu

extern "C" {

int bike_gpu_sizeof_world_config() {
    return static_cast<int>(sizeof(bike::gpu::GpuWorldConfig));
}

int bike_gpu_sizeof_world_state() {
    return static_cast<int>(sizeof(bike::gpu::GpuWorldState));
}

int bike_gpu_sizeof_model_indices() {
    return static_cast<int>(sizeof(bike::gpu::GpuModelIndices));
}

void bike_gpu_init_default_config(bike::gpu::GpuWorldConfig *cfg) {
    if (cfg != nullptr) {
        bike::gpu::gpu_init_default_world_config(*cfg);
    }
}

void bike_gpu_build_sweep_configs(
    bike::gpu::GpuWorldConfig *configs,
    int nworld,
    const float *fork_psi_scale,
    const float *rebound_scale,
    const float *shock_rate_scale,
    const float *tire_stiff_scale,
    int num_stations,
    int num_lateral_ribs
) {
    bike::gpu::gpu_build_sweep_configs(
        configs,
        nworld,
        fork_psi_scale,
        rebound_scale,
        shock_rate_scale,
        tire_stiff_scale,
        num_stations,
        num_lateral_ribs
    );
}

void bike_gpu_launch_init_states(
    void *cuda_stream,
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos
) {
    if (nworld <= 0 || d_configs == nullptr || d_states == nullptr) {
        return;
    }
#if defined(__CUDACC__)
    constexpr int kBlockSize = 128;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    cudaStream_t stream = static_cast<cudaStream_t>(cuda_stream);
    bike::gpu::bike_gpu_init_states_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld,
        d_idx,
        d_configs,
        per_world_config,
        d_states,
        d_qpos
    );
#else
    (void)cuda_stream;
    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : bike::gpu::GpuModelIndices{};
    for (int w = 0; w < nworld; ++w) {
        const bike::gpu::GpuWorldConfig &cfg = d_configs[per_world_config ? w : 0];
        const float *qpos_w = (d_qpos != nullptr) ? (d_qpos + w * idx.nq) : nullptr;
        const float crank_qpos = (qpos_w != nullptr) ? qpos_w[idx.crank_spin_qpos] : 0.0f;
        const float rear_wheel_qpos =
            (qpos_w != nullptr) ? qpos_w[idx.rear_wheel_spin_qpos] : 0.0f;
        bike::gpu::gpu_init_world_state(
            d_states[w], cfg.drivetrain, crank_qpos, rear_wheel_qpos
        );
    }
#endif
}

void bike_gpu_launch_writers(
    void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const float *d_hf_heights,
    int hf_num_samples,
    float hf_x_min,
    float hf_dx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || d_configs == nullptr || d_states == nullptr) {
        return;
    }
    const float hf_inv_dx = (hf_dx > 0.0f) ? (1.0f / hf_dx) : 1.0f;
#if defined(__CUDACC__)
    constexpr int kBlockSize = 128;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    cudaStream_t stream = static_cast<cudaStream_t>(cuda_stream);
    bike::gpu::bike_gpu_writers_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld,
        dt_s,
        d_idx,
        d_hf_heights,
        hf_num_samples,
        hf_x_min,
        hf_dx,
        hf_inv_dx,
        d_configs,
        per_world_config,
        d_states,
        d_qpos,
        d_qvel,
        d_xpos,
        d_qfrc_applied,
        d_ctrl,
        d_tendon_range,
        tendon_range_world_stride
    );
#else
    (void)cuda_stream;
    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : bike::gpu::GpuModelIndices{};
    const bike::gpu::GpuHeightfield1D hf{
        d_hf_heights,
        hf_num_samples,
        hf_x_min,
        hf_dx,
        hf_inv_dx
    };
    for (int w = 0; w < nworld; ++w) {
        bike::gpu::gpu_step_world_writers(
            w,
            dt_s,
            idx,
            hf,
            d_configs,
            per_world_config,
            d_states,
            d_qpos,
            d_qvel,
            d_xpos,
            d_qfrc_applied,
            d_ctrl,
            d_tendon_range,
            tendon_range_world_stride
        );
    }
#endif
}

} // extern "C"
