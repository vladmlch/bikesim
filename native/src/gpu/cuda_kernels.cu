// native/src/gpu/cuda_kernels.cu — CUDA __global__ kernels and extern "C" host
// entry points for ctypes / mujoco_warp integration on NVIDIA GPUs (plus Host fallback).

#include "batch_step.hpp"
#include "cuda_kernels_abi.hpp"

#include <cmath>
#include <iostream>
#include <vector>

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
    if (!gpu_validate_model_indices(idx)) {
        return;
    }
    const GpuWorldConfig &cfg = d_configs[per_world_config ? w : 0];
    const float *qpos_w = (d_qpos != nullptr) ? (d_qpos + static_cast<long>(w) * idx.nq) : nullptr;
    const bool has_ds =
        (idx.drive_shaft_spin_qpos >= 0 && idx.drive_shaft_spin_qpos < idx.nq);
    const float crank_qpos =
        (qpos_w != nullptr && idx.crank_spin_qpos >= 0 && idx.crank_spin_qpos < idx.nq)
            ? qpos_w[idx.crank_spin_qpos]
            : 0.0f;
    const float ds_qpos =
        (qpos_w != nullptr && has_ds) ? qpos_w[idx.drive_shaft_spin_qpos] : 0.0f;
    const float rear_wheel_qpos =
        (qpos_w != nullptr && idx.rear_wheel_spin_qpos >= 0 && idx.rear_wheel_spin_qpos < idx.nq)
            ? qpos_w[idx.rear_wheel_spin_qpos]
            : 0.0f;
    gpu_init_world_state(d_states[w], cfg.drivetrain, crank_qpos, rear_wheel_qpos, ds_qpos, has_ds);
}

BIKE_GLOBAL void bike_gpu_writers_kernel(
    int nworld,
    float dt_s,
    const GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const GpuWorldConfig *d_configs,
    int per_world_config,
    GpuWorldState *d_states,
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
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    if (!gpu_validate_model_indices(idx)) {
        return;
    }
    if (d_terrain_desc == nullptr || d_tyre_model == nullptr) {
        return;
    }
    const int max_active_tendon_idx = gpu_max_active_tendon_idx(idx);
    if (d_tendon_range != nullptr && max_active_tendon_idx >= 0 &&
        tendon_range_world_stride > 0 &&
        static_cast<long>(2) * static_cast<long>(max_active_tendon_idx) + 1L >=
            static_cast<long>(tendon_range_world_stride)) {
        return;
    }
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *d_terrain_desc;
    gpu_step_world_writers(
        w,
        dt_s,
        idx,
        d_tyre_model,
        terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        d_configs,
        per_world_config,
        d_states,
        d_qpos,
        d_qvel,
        d_xpos,
        d_xmat,
        d_xanchor,
        d_xaxis,
        d_qfrc_applied,
        d_ctrl,
        d_tendon_range,
        tendon_range_world_stride,
        false
    );
}

#endif // defined(__CUDACC__)

namespace {

#if defined(__CUDACC__)
bool validate_device_monotonic_xz(
    const float *d_terrain_f32,
    int x_offset,
    int z_offset,
    int count
) {
    if (count < 2) {
        return true;
    }
    std::vector<float> hx(static_cast<std::size_t>(count));
    std::vector<float> hz(static_cast<std::size_t>(count));
    if (cudaMemcpy(
            hx.data(),
            d_terrain_f32 + x_offset,
            static_cast<std::size_t>(count) * sizeof(float),
            cudaMemcpyDeviceToHost) != cudaSuccess ||
        cudaMemcpy(
            hz.data(),
            d_terrain_f32 + z_offset,
            static_cast<std::size_t>(count) * sizeof(float),
            cudaMemcpyDeviceToHost) != cudaSuccess) {
        return false;
    }
    if (!std::isfinite(hx[0]) || !std::isfinite(hz[0])) {
        return false;
    }
    for (int i = 0; i < count - 1; ++i) {
        const auto iu = static_cast<std::size_t>(i);
        if (!std::isfinite(hx[iu + 1]) || !std::isfinite(hz[iu + 1]) || !(hx[iu + 1] > hx[iu])) {
            return false;
        }
    }
    return true;
}
#endif

template <class T>
bool copy_maybe_device_struct(const T *ptr, T &out) {
    if (ptr == nullptr) {
        return false;
    }
#if defined(__CUDACC__)
    cudaPointerAttributes attr{};
    if (cudaPointerGetAttributes(&attr, ptr) == cudaSuccess &&
        attr.type == cudaMemoryTypeDevice) {
        return cudaMemcpy(&out, ptr, sizeof(T), cudaMemcpyDeviceToHost) == cudaSuccess;
    }
    cudaGetLastError();
#endif
    out = *ptr;
    return true;
}

inline int validate_writers_scalars_and_ptrs(
    int nworld,
    float dt_s,
    const GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const GpuWorldConfig *d_configs,
    const GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    const float *d_qfrc_applied,
    const float *d_ctrl,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || !bike::physics_core::GpuToolkit::isfinite(dt_s) || dt_s <= 0.0f ||
        tendon_range_world_stride < 0 ||
        d_idx == nullptr || d_tyre_model == nullptr || d_terrain_desc == nullptr ||
        d_configs == nullptr || d_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xpos == nullptr ||
        d_xmat == nullptr || d_xanchor == nullptr || d_xaxis == nullptr ||
        d_qfrc_applied == nullptr || d_ctrl == nullptr) {
        return -1;
    }
    return 0;
}

inline int validate_writers_inputs(
    int nworld,
    float dt_s,
    const GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const GpuWorldConfig *d_configs,
    const GpuWorldState *d_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    const float *d_qfrc_applied,
    const float *d_ctrl,
    const float *d_tendon_range,
    int tendon_range_world_stride,
    GpuModelIndices &host_idx,
    bike::physics_core::GpuTyreModel &host_tyre_model,
    bike::physics_core::GpuTerrainTableDesc &host_terrain_desc
) {
    if (validate_writers_scalars_and_ptrs(
            nworld, dt_s, d_idx, d_tyre_model, d_terrain_desc, d_configs, d_states,
            d_qpos, d_qvel, d_xpos, d_xmat, d_xanchor, d_xaxis, d_qfrc_applied, d_ctrl,
            tendon_range_world_stride) != 0) {
        return -1;
    }
    if (!copy_maybe_device_struct(d_idx, host_idx) ||
        !copy_maybe_device_struct(d_tyre_model, host_tyre_model) ||
        !copy_maybe_device_struct(d_terrain_desc, host_terrain_desc)) {
        return -2;
    }
    if (!gpu_validate_model_indices(host_idx)) {
        std::cerr << "bike_gpu_launch_writers: invalid model indices or kMax bounds exceeded\n";
        return -3;
    }
    const int max_active_tendon_idx = gpu_max_active_tendon_idx(host_idx);
    if (d_tendon_range != nullptr && max_active_tendon_idx >= 0 &&
        tendon_range_world_stride > 0 &&
        static_cast<long>(2) * static_cast<long>(max_active_tendon_idx) + 1L >=
            static_cast<long>(tendon_range_world_stride)) {
        std::cerr << "bike_gpu_launch_writers: tendon_range_world_stride too small for active tendons\n";
        return -4;
    }
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::validate_gpu_tyre_model_device(
            &host_tyre_model, host_idx.nq, host_idx.nv, host_idx.nbody, err) !=
        bike::physics_core::Status::ok) {
        std::cerr << "bike_gpu_launch_writers: invalid GpuTyreModel or dimension mismatch\n";
        return -5;
    }

    const bool need_polyline =
        (host_tyre_model.backend == bike::physics_core::TyreBackendKind::compliant_2d);
    const bool need_signed =
        (host_tyre_model.backend == bike::physics_core::TyreBackendKind::distributed_2d_reference);
#if defined(__CUDACC__)
    bool f32_on_device = false;
    if (d_terrain_f32 != nullptr) {
        cudaPointerAttributes f32_attr{};
        if (cudaPointerGetAttributes(&f32_attr, d_terrain_f32) == cudaSuccess &&
            f32_attr.type == cudaMemoryTypeDevice) {
            f32_on_device = true;
        } else {
            cudaGetLastError();
        }
    }
    const bool can_check_monotonic = !f32_on_device;
#else
    const bool can_check_monotonic = true;
#endif
    if (bike::physics_core::validate_terrain_table_desc(
            host_terrain_desc,
            d_terrain_f32,
            d_terrain_i32,
            need_polyline,
            need_signed,
            can_check_monotonic,
            err) != bike::physics_core::Status::ok) {
        std::cerr << "bike_gpu_launch_writers: invalid GpuTerrainTableDesc or non-monotonic px\n";
        return -6;
    }
#if defined(__CUDACC__)
    if (f32_on_device) {
        if ((host_terrain_desc.polyline_vertex_count > 0 &&
             !validate_device_monotonic_xz(
                 d_terrain_f32,
                 host_terrain_desc.px_offset,
                 host_terrain_desc.pz_offset,
                 host_terrain_desc.polyline_vertex_count)) ||
            (host_terrain_desc.signed_vertex_count > 0 &&
             !validate_device_monotonic_xz(
                 d_terrain_f32,
                 host_terrain_desc.sp_x_offset,
                 host_terrain_desc.sp_z_offset,
                 host_terrain_desc.signed_vertex_count))) {
            std::cerr << "bike_gpu_launch_writers: non-monotonic device terrain px/sp_x\n";
            return -6;
        }
    }
#endif
    return 0;
}

} // namespace

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

int bike_gpu_sizeof_tyre_model() {
    return static_cast<int>(sizeof(bike::physics_core::GpuTyreModel));
}

int bike_gpu_sizeof_terrain_desc() {
    return static_cast<int>(sizeof(bike::physics_core::GpuTerrainTableDesc));
}

int bike_gpu_init_default_config(bike::gpu::GpuWorldConfig *cfg) {
    if (cfg == nullptr) {
        return -1;
    }
    bike::gpu::gpu_init_default_world_config(*cfg);
    return 0;
}

int bike_gpu_build_sweep_configs(
    bike::gpu::GpuWorldConfig *configs,
    int nworld,
    const float *fork_psi_scale,
    const float *rebound_scale,
    const float *shock_rate_scale,
    const float *tire_stiff_scale
) {
    return bike::gpu::gpu_build_sweep_configs(
        configs,
        nworld,
        fork_psi_scale,
        rebound_scale,
        shock_rate_scale,
        tire_stiff_scale
    );
}

int bike_gpu_launch_init_states(
    const void *cuda_stream,
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuWorldConfig *d_configs,
    int per_world_config,
    bike::gpu::GpuWorldState *d_states,
    const float *d_qpos
) {
    if (nworld <= 0 || d_idx == nullptr || d_configs == nullptr || d_states == nullptr) {
        return -1;
    }
#if defined(__CUDACC__)
    constexpr int kBlockSize = 128;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    cudaStream_t stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
    cudaStreamCaptureStatus capture_status = cudaStreamCaptureStatusNone;
    const cudaError_t cap_err = cudaStreamIsCapturing(stream, &capture_status);
    if (cap_err != cudaSuccess || capture_status == cudaStreamCaptureStatusInvalidated) {
        return -2;
    }
    if (capture_status == cudaStreamCaptureStatusActive) {
        bike::gpu::bike_gpu_init_states_kernel<<<grid_size, kBlockSize, 0, stream>>>(
            nworld,
            d_idx,
            d_configs,
            per_world_config,
            d_states,
            d_qpos
        );
        return 0;
    }
    bike::gpu::GpuModelIndices host_idx{};
    if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
        !bike::gpu::gpu_validate_model_indices(host_idx)) {
        return -2;
    }
    bike::gpu::bike_gpu_init_states_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld,
        d_idx,
        d_configs,
        per_world_config,
        d_states,
        d_qpos
    );
    return cudaGetLastError() == cudaSuccess ? 0 : -3;
#else
    (void)cuda_stream;
    bike::gpu::GpuModelIndices host_idx{};
    if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
        !bike::gpu::gpu_validate_model_indices(host_idx)) {
        return -2;
    }
    for (int w = 0; w < nworld; ++w) {
        const bike::gpu::GpuWorldConfig &cfg = d_configs[per_world_config ? w : 0];
        const float *qpos_w =
            (d_qpos != nullptr) ? (d_qpos + static_cast<long>(w) * host_idx.nq) : nullptr;
        const bool has_ds =
            (host_idx.drive_shaft_spin_qpos >= 0 && host_idx.drive_shaft_spin_qpos < host_idx.nq);
        const float crank_qpos =
            (qpos_w != nullptr && host_idx.crank_spin_qpos >= 0 &&
             host_idx.crank_spin_qpos < host_idx.nq)
                ? qpos_w[host_idx.crank_spin_qpos]
                : 0.0f;
        const float ds_qpos =
            (qpos_w != nullptr && has_ds) ? qpos_w[host_idx.drive_shaft_spin_qpos] : 0.0f;
        const float rear_wheel_qpos =
            (qpos_w != nullptr && host_idx.rear_wheel_spin_qpos >= 0 &&
             host_idx.rear_wheel_spin_qpos < host_idx.nq)
                ? qpos_w[host_idx.rear_wheel_spin_qpos]
                : 0.0f;
        bike::gpu::gpu_init_world_state(
            d_states[w], cfg.drivetrain, crank_qpos, rear_wheel_qpos, ds_qpos, has_ds);
    }
    return 0;
#endif
}

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
) {
#if defined(__CUDACC__)
    if (bike::gpu::validate_writers_scalars_and_ptrs(
            nworld, dt_s, d_idx, d_tyre_model, d_terrain_desc, d_configs, d_states,
            d_qpos, d_qvel, d_xpos, d_xmat, d_xanchor, d_xaxis, d_qfrc_applied, d_ctrl,
            tendon_range_world_stride) != 0) {
        return -1;
    }
    constexpr int kBlockSize = 128;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    cudaStream_t stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
    cudaStreamCaptureStatus capture_status = cudaStreamCaptureStatusNone;
    const cudaError_t cap_err = cudaStreamIsCapturing(stream, &capture_status);
    if (cap_err != cudaSuccess || capture_status == cudaStreamCaptureStatusInvalidated) {
        return -7;
    }
    if (capture_status == cudaStreamCaptureStatusActive) {
        bike::gpu::bike_gpu_writers_kernel<<<grid_size, kBlockSize, 0, stream>>>(
            nworld,
            dt_s,
            d_idx,
            d_tyre_model,
            d_terrain_desc,
            d_terrain_f32,
            d_terrain_i32,
            d_configs,
            per_world_config,
            d_states,
            d_qpos,
            d_qvel,
            d_xpos,
            d_xmat,
            d_xanchor,
            d_xaxis,
            d_qfrc_applied,
            d_ctrl,
            d_tendon_range,
            tendon_range_world_stride
        );
        return 0;
    }
    bike::gpu::GpuModelIndices host_idx{};
    bike::physics_core::GpuTyreModel host_tyre_model{};
    bike::physics_core::GpuTerrainTableDesc host_terrain_desc{};
    const int val_rc = bike::gpu::validate_writers_inputs(
        nworld,
        dt_s,
        d_idx,
        d_tyre_model,
        d_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        d_configs,
        d_states,
        d_qpos,
        d_qvel,
        d_xpos,
        d_xmat,
        d_xanchor,
        d_xaxis,
        d_qfrc_applied,
        d_ctrl,
        d_tendon_range,
        tendon_range_world_stride,
        host_idx,
        host_tyre_model,
        host_terrain_desc);
    if (val_rc != 0) {
        return val_rc;
    }
    bike::gpu::bike_gpu_writers_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld,
        dt_s,
        d_idx,
        d_tyre_model,
        d_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        d_configs,
        per_world_config,
        d_states,
        d_qpos,
        d_qvel,
        d_xpos,
        d_xmat,
        d_xanchor,
        d_xaxis,
        d_qfrc_applied,
        d_ctrl,
        d_tendon_range,
        tendon_range_world_stride
    );
    return cudaGetLastError() == cudaSuccess ? 0 : -7;
#else
    (void)cuda_stream;
    bike::gpu::GpuModelIndices host_idx{};
    bike::physics_core::GpuTyreModel host_tyre_model{};
    bike::physics_core::GpuTerrainTableDesc host_terrain_desc{};
    const int val_rc = bike::gpu::validate_writers_inputs(
        nworld,
        dt_s,
        d_idx,
        d_tyre_model,
        d_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        d_configs,
        d_states,
        d_qpos,
        d_qvel,
        d_xpos,
        d_xmat,
        d_xanchor,
        d_xaxis,
        d_qfrc_applied,
        d_ctrl,
        d_tendon_range,
        tendon_range_world_stride,
        host_idx,
        host_tyre_model,
        host_terrain_desc);
    if (val_rc != 0) {
        return val_rc;
    }
    for (int w = 0; w < nworld; ++w) {
        const bike::physics_core::Status st = bike::gpu::gpu_step_world_writers(
            w,
            dt_s,
            host_idx,
            d_tyre_model,
            host_terrain_desc,
            d_terrain_f32,
            d_terrain_i32,
            d_configs,
            per_world_config,
            d_states,
            d_qpos,
            d_qvel,
            d_xpos,
            d_xmat,
            d_xanchor,
            d_xaxis,
            d_qfrc_applied,
            d_ctrl,
            d_tendon_range,
            tendon_range_world_stride,
            false
        );
        if (st != bike::physics_core::Status::ok) {
            return -8;
        }
    }
    return 0;
#endif
}

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
) {
    return bike_gpu_launch_writers(
        cuda_stream,
        nworld,
        dt_s,
        idx,
        d_tyre_model,
        d_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        d_configs,
        per_world_config,
        d_states,
        d_qpos,
        d_qvel,
        d_xpos,
        d_xmat,
        d_xanchor,
        d_xaxis,
        d_qfrc_applied,
        d_ctrl,
        d_tendon_range,
        tendon_range_world_stride
    );
}

} // extern "C"

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
