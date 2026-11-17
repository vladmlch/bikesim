// native/src/gpu/cuda_kernels_abi.hpp — the extern "C" host entry points
// cuda_kernels.cu defines (ctypes / mujoco_warp ABI). Host C++ translation
// units declare them from here and link bike_gpu_kernels, the single host
// compile of cuda_kernels.cu; nvcc builds compile cuda_kernels.cu directly.
#pragma once

#include "batch_step.hpp"

// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional float signatures mirror the CUDA/MSL kernels and the ctypes ABI. Register: docs/TESTING.md GPU kernel row.
extern "C" {

int bike_gpu_sizeof_world_config();
int bike_gpu_sizeof_world_state();
int bike_gpu_sizeof_model_indices();
int bike_gpu_sizeof_tyre_model();
int bike_gpu_sizeof_terrain_desc();

int bike_gpu_init_default_config(bike::gpu::GpuWorldConfig *cfg);

int bike_gpu_build_sweep_configs(
    bike::gpu::GpuWorldConfig *configs,
    int nworld,
    const float *fork_psi_scale,
    const float *rebound_scale,
    const float *shock_rate_scale,
    const float *tire_stiff_scale
);

int bike_gpu_launch_init_states(
    const void *cuda_stream,
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos
);

int bike_gpu_launch_writers(
    const void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
);

int bike_gpu_launch_step_writers(
    const void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    float *d_tendon_range,
    int tendon_range_world_stride
);

} // extern "C"
// NOLINTEND(bugprone-easily-swappable-parameters)
