// native/src/gpu/cuda_kernels.cu — CUDA __global__ kernels and extern "C" host
// entry points for ctypes / mujoco_warp integration on NVIDIA GPUs.

#include "batch_step.hpp"

#include <iostream>

#if defined(__CUDACC__)
#include <cuda_runtime.h>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI; positional float signatures mirror the CUDA/MSL kernels. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

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
    if (idx.nq < 0 || idx.nq > kMaxNq || idx.nv < 0 || idx.nv > kMaxNv ||
        idx.nu < 0 || idx.nu > kMaxNu || idx.nbody < 0 || idx.nbody > kMaxNbody) {
        return;
    }
    const GpuWorldConfig &cfg = d_configs[per_world_config ? w : 0];
    const float *qpos_w = (d_qpos != nullptr) ? (d_qpos + static_cast<long>(w) * idx.nq) : nullptr;
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
    if (idx.nq < 0 || idx.nq > kMaxNq || idx.nv < 0 || idx.nv > kMaxNv ||
        idx.nu < 0 || idx.nu > kMaxNu || idx.nbody < 0 || idx.nbody > kMaxNbody) {
        return;
    }
    const GpuHeightfield1D hf{
        .heights_m = d_hf_heights,
        .num_samples = hf_num_samples,
        .x_min_m = hf_x_min,
        .dx_m = hf_dx,
        .inv_dx_m = hf_inv_dx
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

int bike_gpu_sizeof_world_config();
int bike_gpu_sizeof_world_state();
int bike_gpu_sizeof_model_indices();
void bike_gpu_init_default_config(bike::gpu::GpuWorldConfig *cfg);
void bike_gpu_build_sweep_configs(
    bike::gpu::GpuWorldConfig *configs,
    int nworld,
    const float *fork_psi_scale,
    const float *rebound_scale,
    const float *shock_rate_scale,
    const float *tire_stiff_scale,
    int num_stations,
    int num_lateral_ribs
);
void bike_gpu_launch_init_states(
    const void *cuda_stream,
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos
);
void bike_gpu_launch_writers(
    const void *cuda_stream,
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
);
void bike_gpu_launch_step_writers(
    const void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *idx,
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
);

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
    const void *cuda_stream,
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
    cudaStream_t stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
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
    const bike::gpu::GpuModelIndices *idx_ptr = d_idx;
    if (idx_ptr != nullptr && (idx_ptr->nq > bike::gpu::kMaxNq || idx_ptr->nv > bike::gpu::kMaxNv || idx_ptr->nu > bike::gpu::kMaxNu || idx_ptr->nbody > bike::gpu::kMaxNbody || idx_ptr->nq < 0 || idx_ptr->nv < 0 || idx_ptr->nu < 0 || idx_ptr->nbody < 0)) {
        std::cerr << "bike_gpu_launch_init_states: model dimensions (nq=" << idx_ptr->nq
                  << ", nv=" << idx_ptr->nv << ", nu=" << idx_ptr->nu << ", nbody=" << idx_ptr->nbody
                  << ") exceed kMax bounds (" << bike::gpu::kMaxNq << ", " << bike::gpu::kMaxNv
                  << ", " << bike::gpu::kMaxNu << ", " << bike::gpu::kMaxNbody << ")\n";
        return;
    }
    const bike::gpu::GpuModelIndices idx =
        (d_idx != nullptr) ? *d_idx : bike::gpu::GpuModelIndices{};
    for (int w = 0; w < nworld; ++w) {
        const bike::gpu::GpuWorldConfig &cfg = d_configs[per_world_config ? w : 0];
        const float *qpos_w = (d_qpos != nullptr) ? (d_qpos + static_cast<long>(w) * idx.nq) : nullptr;
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
    const void *cuda_stream,
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
    cudaStream_t stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
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
    const bike::gpu::GpuModelIndices *idx = d_idx;
    if (idx != nullptr && (idx->nq > bike::gpu::kMaxNq || idx->nv > bike::gpu::kMaxNv || idx->nu > bike::gpu::kMaxNu || idx->nbody > bike::gpu::kMaxNbody || idx->nq < 0 || idx->nv < 0 || idx->nu < 0 || idx->nbody < 0)) {
        std::cerr << "bike_gpu_launch_writers: model dimensions (nq=" << idx->nq
                  << ", nv=" << idx->nv << ", nu=" << idx->nu << ", nbody=" << idx->nbody
                  << ") exceed kMax bounds (" << bike::gpu::kMaxNq << ", " << bike::gpu::kMaxNv
                  << ", " << bike::gpu::kMaxNu << ", " << bike::gpu::kMaxNbody << ")\n";
        return;
    }
    const bike::gpu::GpuModelIndices idx_val =
        (d_idx != nullptr) ? *d_idx : bike::gpu::GpuModelIndices{};
    const bike::gpu::GpuHeightfield1D hf{
        .heights_m = d_hf_heights,
        .num_samples = hf_num_samples,
        .x_min_m = hf_x_min,
        .dx_m = hf_dx,
        .inv_dx_m = hf_inv_dx
    };
    for (int w = 0; w < nworld; ++w) {
        bike::gpu::gpu_step_world_writers(
            w,
            dt_s,
            idx_val,
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

void bike_gpu_launch_step_writers(
    const void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *idx,
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
    if (idx != nullptr) {
        bike::gpu::GpuModelIndices host_idx{};
#if defined(__CUDACC__)
        cudaPointerAttributes attr{};
        if (cudaPointerGetAttributes(&attr, idx) == cudaSuccess &&
            attr.type == cudaMemoryTypeDevice) {
            cudaMemcpy(&host_idx, idx, sizeof(host_idx), cudaMemcpyDeviceToHost);
        } else {
            cudaGetLastError();
            host_idx = *idx;
        }
#else
        host_idx = *idx;
#endif
        if (host_idx.nq > bike::gpu::kMaxNq || host_idx.nv > bike::gpu::kMaxNv ||
            host_idx.nu > bike::gpu::kMaxNu || host_idx.nbody > bike::gpu::kMaxNbody ||
            host_idx.nq < 0 || host_idx.nv < 0 || host_idx.nu < 0 || host_idx.nbody < 0) {
            std::cerr << "bike_gpu_launch_step_writers: model dimensions (nq=" << host_idx.nq
                      << ", nv=" << host_idx.nv << ", nu=" << host_idx.nu << ", nbody=" << host_idx.nbody
                      << ") exceed kMax bounds (" << bike::gpu::kMaxNq << ", " << bike::gpu::kMaxNv
                      << ", " << bike::gpu::kMaxNu << ", " << bike::gpu::kMaxNbody << ")\n";
            return;
        }
    }
    bike_gpu_launch_writers(
        cuda_stream,
        nworld,
        dt_s,
        idx,
        d_hf_heights,
        hf_num_samples,
        hf_x_min,
        hf_dx,
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

} // extern "C"

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
