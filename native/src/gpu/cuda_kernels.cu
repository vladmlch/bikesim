// native/src/gpu/cuda_kernels.cu — CUDA __global__ kernels and extern "C" host
// entry points for ctypes / mujoco_warp integration on NVIDIA GPUs (plus Host fallback).

#include "batch_step.hpp"
#include "sp3_autonomous_step.hpp"
#include "sp4_closed_loop_step.hpp"
#include "../physics_core/tyre_pneumatic_model.hpp"
#include "../physics_core/cruise.hpp"
#include "../physics_core/ride_program.hpp"
#include "../physics_core/energy_ledger.hpp"
#include "../physics_core/spindle_m0.hpp"
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

BIKE_GLOBAL void bike_gpu_sp3_init_kernel(
    int nworld,
    const GpuModelIndices *d_idx,
    const GpuSp3Config *d_sp3_configs,
    int per_world_config,
    GpuSp3WorldState *d_sp3_states,
    GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    if (!gpu_validate_model_indices(idx, ntendon)) {
        return;
    }
    const long w64 = static_cast<long>(w);
    const GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
    const float *qpos_w = (d_qpos != nullptr) ? (d_qpos + w64 * idx.nq) : nullptr;
    GpuWorldState *tyre_w = (d_world_states != nullptr) ? &d_world_states[w] : nullptr;
    float *tr_w = nullptr;
    if (d_tendon_range != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tr_w = d_tendon_range + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tr_w = d_tendon_range;
        } else {
            return;
        }
    }
    gpu_sp3_init_world(idx, cfg_w, &d_sp3_states[w], tyre_w, qpos_w, init_grade, ntendon, tr_w);
}

BIKE_GLOBAL void bike_gpu_sp3_pre_solve_kernel(
    int nworld,
    float dt_s,
    const GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const GpuSp3Config *d_sp3_configs,
    int per_world_config,
    GpuSp3WorldState *d_sp3_states,
    GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    if (!gpu_validate_model_indices(idx, ntendon) ||
        d_terrain_desc == nullptr || d_tyre_model == nullptr || d_world_states == nullptr) {
        return;
    }
    const long w64 = static_cast<long>(w);
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *d_terrain_desc;
    const GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
    float *tr_w = nullptr;
    if (d_tendon_range != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tr_w = d_tendon_range + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tr_w = d_tendon_range;
        } else {
            return;
        }
    }
    float *fl_w = nullptr;
    if (d_dof_frictionloss != nullptr) {
        if (dof_frictionloss_world_stride >= idx.nv) {
            fl_w = d_dof_frictionloss + w64 * dof_frictionloss_world_stride;
        } else if (dof_frictionloss_world_stride == 0 && w == 0) {
            fl_w = d_dof_frictionloss;
        } else {
            return;
        }
    }
    gpu_sp3_pre_solve_world(
        w,
        dt_s,
        idx,
        d_tyre_model,
        terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        cfg_w,
        &d_sp3_states[w],
        d_world_states[w].compliant_wheels,
        &d_world_states[w].distributed_wheels[0][0],
        d_qpos + w64 * idx.nq,
        d_qvel + w64 * idx.nv,
        d_xpos + w64 * idx.nbody * 3,
        d_xmat + w64 * idx.nbody * 9,
        d_xanchor + w64 * d_tyre_model->njnt * 3,
        d_xaxis + w64 * d_tyre_model->njnt * 3,
        d_qfrc_applied + w64 * idx.nv,
        d_ctrl + w64 * idx.nu,
        ntendon,
        tr_w,
        fl_w);
}

BIKE_GLOBAL void bike_gpu_sp3_post_solve_kernel(
    int nworld,
    float dt_s,
    const GpuModelIndices *d_idx,
    const GpuSp3Config *d_sp3_configs,
    int per_world_config,
    GpuSp3WorldState *d_sp3_states,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xmat,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    if (!gpu_validate_model_indices(idx)) {
        return;
    }
    const long w64 = static_cast<long>(w);
    const GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
    const float *sd_w =
        (d_sensordata != nullptr && nsensordata > 0) ? (d_sensordata + w64 * nsensordata) : nullptr;
    int nefc_w = (d_nefc != nullptr && njmax > 0) ? d_nefc[w] : 0;
    if (nefc_w < 0) {
        nefc_w = 0;
    }
    if (nefc_w > njmax) {
        nefc_w = njmax;
    }
    const int *efc_type_w = (d_efc_type != nullptr && njmax > 0) ? (d_efc_type + w64 * njmax) : nullptr;
    const int *efc_id_w = (d_efc_id != nullptr && njmax > 0) ? (d_efc_id + w64 * njmax) : nullptr;
    const float *efc_force_w = (d_efc_force != nullptr && njmax > 0) ? (d_efc_force + w64 * njmax) : nullptr;
    const float *efc_J_w =
        (d_efc_J != nullptr && njmax > 0 && efc_J_row_stride > 0)
            ? (d_efc_J + w64 * static_cast<long>(njmax) * efc_J_row_stride)
            : nullptr;

    int ncon_or_total = 0;
    const int *c_geom = nullptr;
    const int *c_adr = nullptr;
    const int *c_dim = nullptr;
    if (d_nacon != nullptr && nconmax > 0) {
        if (d_contact_worldid != nullptr) {
            ncon_or_total = d_nacon[0];
            if (ncon_or_total < 0) {
                ncon_or_total = 0;
            }
            if (ncon_or_total > nconmax) {
                ncon_or_total = nconmax;
            }
            c_geom = d_contact_geom;
            c_adr = d_contact_efc_address;
            c_dim = d_contact_dim;
        } else {
            ncon_or_total = d_nacon[w];
            if (ncon_or_total < 0) {
                ncon_or_total = 0;
            }
            if (ncon_or_total > nconmax) {
                ncon_or_total = nconmax;
            }
            c_geom = (d_contact_geom != nullptr) ? (d_contact_geom + w64 * 2 * nconmax) : nullptr;
            c_adr = (d_contact_efc_address != nullptr)
                        ? (d_contact_efc_address + w64 * nconmax * contact_efc_stride)
                        : nullptr;
            c_dim = (d_contact_dim != nullptr) ? (d_contact_dim + w64 * nconmax) : nullptr;
        }
    }

    gpu_sp3_post_solve_world(
        w,
        dt_s,
        idx,
        cfg_w,
        &d_sp3_states[w],
        d_qpos + w64 * idx.nq,
        d_qvel + w64 * idx.nv,
        d_xmat + w64 * idx.nbody * 9,
        nsensordata,
        sd_w,
        nefc_w,
        efc_J_row_stride,
        efc_type_w,
        efc_id_w,
        efc_J_w,
        efc_force_w,
        ncon_or_total,
        contact_efc_stride,
        d_contact_worldid,
        c_geom,
        c_adr,
        c_dim);
}

BIKE_GLOBAL void bike_gpu_sp4_init_kernel(
    int nworld,
    const GpuModelIndices *d_idx,
    const GpuSp3Config *d_sp3_configs,
    int per_world_config,
    GpuSp3WorldState *d_sp3_states,
    GpuSp4WorldState *d_sp4_states,
    GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    if (!gpu_validate_model_indices(idx, ntendon)) {
        return;
    }
    const GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
    const float *qpos_w = (d_qpos != nullptr)
                              ? (d_qpos + static_cast<long>(w) * idx.nq)
                              : nullptr;
    GpuWorldState *ws = (d_world_states != nullptr) ? &d_world_states[w] : nullptr;
    float *tr_w = nullptr;
    if (d_tendon_range != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tr_w = d_tendon_range + static_cast<long>(w) * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tr_w = d_tendon_range;
        } else {
            return;
        }
    }
    gpu_sp4_init_world(
        idx, cfg_w, &d_sp3_states[w], &d_sp4_states[w], ws, qpos_w, init_grade, ntendon, tr_w);
}

BIKE_GLOBAL void bike_gpu_sp4_pre_solve_and_pass0_kernel(
    int nworld,
    float dt_s,
    const GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    GpuSp3WorldState *d_sp3_states,
    GpuSp4WorldState *d_sp4_states,
    GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    const float *d_site_xpos,
    const float *d_subtree_com,
    const float *d_body_subtreemass,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride,
    float *d_qacc_warmstart
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld || d_terrain_desc == nullptr) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    if (!gpu_validate_model_indices(idx, ntendon) || d_tyre_model == nullptr ||
        d_spindle_params == nullptr) {
        return;
    }
    const bike::physics_core::GpuTerrainTableDesc terrain_desc = *d_terrain_desc;
    const GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
    const long w64 = static_cast<long>(w);

    float *tr_w = nullptr;
    if (d_tendon_range != nullptr && ntendon > 0) {
        if (tendon_range_world_stride >= 2 * ntendon) {
            tr_w = d_tendon_range + w64 * tendon_range_world_stride;
        } else if (tendon_range_world_stride == 0 && w == 0) {
            tr_w = d_tendon_range;
        }
    }
    float *fl_w = nullptr;
    if (d_dof_frictionloss != nullptr) {
        if (dof_frictionloss_world_stride >= idx.nv) {
            fl_w = d_dof_frictionloss + w64 * dof_frictionloss_world_stride;
        } else if (dof_frictionloss_world_stride == 0 && w == 0) {
            fl_w = d_dof_frictionloss;
        }
    }
    float *ws_w = (d_qacc_warmstart != nullptr) ? (d_qacc_warmstart + w64 * idx.nv) : nullptr;

    gpu_sp4_pre_solve_and_pass0_world(
        w,
        dt_s,
        idx,
        d_tyre_model,
        terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        cfg_w,
        d_spindle_params,
        &d_sp3_states[w],
        &d_sp4_states[w],
        d_world_states[w].compliant_wheels,
        &d_world_states[w].distributed_wheels[0][0],
        d_qpos + w64 * idx.nq,
        d_qvel + w64 * idx.nv,
        d_qfrc_bias + w64 * idx.nv,
        d_xpos + w64 * idx.nbody * 3,
        d_xmat + w64 * idx.nbody * 9,
        d_xanchor + w64 * d_tyre_model->njnt * 3,
        d_xaxis + w64 * d_tyre_model->njnt * 3,
        d_site_xpos + w64 * d_spindle_params->nsite * 3,
        d_subtree_com + w64 * idx.nbody * 3,
        d_body_subtreemass,
        qM_row_stride,
        d_M_rowadr,
        d_M_rownnz,
        d_M_colind,
        d_M_or_qM + w64 * M_world_stride,
        d_qfrc_applied + w64 * idx.nv,
        d_ctrl + w64 * idx.nu,
        ntendon,
        tr_w,
        fl_w,
        ws_w);
}

BIKE_GLOBAL void bike_gpu_sp4_pass_stage_kernel(
    int stage,
    int nworld,
    int nq,
    int nv,
    int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    GpuSp4WorldState *d_sp4_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_qacc,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_ctrl,
    float *d_qacc_warmstart
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const long w64 = static_cast<long>(w);
    float *ws_w = (d_qacc_warmstart != nullptr) ? (d_qacc_warmstart + w64 * nv) : nullptr;
    if (stage == 1) {
        gpu_sp4_pass1_world(
            d_spindle_params, &d_sp4_states[w],
            d_qpos + w64 * nq, d_qvel + w64 * nv, d_qfrc_bias + w64 * nv, d_qacc + w64 * nv,
            qM_row_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
            d_M_or_qM + w64 * M_world_stride, d_ctrl + w64 * nu, ws_w);
    } else if (stage == 2) {
        gpu_sp4_pass2_world(
            d_spindle_params, &d_sp4_states[w],
            d_qpos + w64 * nq, d_qvel + w64 * nv, d_qfrc_bias + w64 * nv, d_qacc + w64 * nv,
            qM_row_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
            d_M_or_qM + w64 * M_world_stride, d_ctrl + w64 * nu, ws_w);
    } else if (stage == 3) {
        gpu_sp4_pass3_world(
            d_spindle_params, &d_sp4_states[w],
            d_qpos + w64 * nq, d_qvel + w64 * nv, d_qfrc_bias + w64 * nv, d_qacc + w64 * nv,
            qM_row_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
            d_M_or_qM + w64 * M_world_stride, d_ctrl + w64 * nu, ws_w);
    }
}

BIKE_GLOBAL void bike_gpu_sp4_post_solve_kernel(
    int nworld,
    float dt_s,
    const GpuModelIndices *d_idx,
    const GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    GpuSp3WorldState *d_sp3_states,
    GpuSp4WorldState *d_sp4_states,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
) {
    const int w = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    if (w >= nworld) {
        return;
    }
    const GpuModelIndices idx = (d_idx != nullptr) ? *d_idx : GpuModelIndices{};
    if (!gpu_validate_model_indices(idx)) {
        return;
    }
    const long w64 = static_cast<long>(w);
    const GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
    const float *sd_w = (d_sensordata != nullptr && nsensordata > 0)
                            ? (d_sensordata + w64 * nsensordata)
                            : nullptr;
    int nefc_w = 0;
    const int *efc_type_w = nullptr;
    const int *efc_id_w = nullptr;
    const float *efc_J_w = nullptr;
    const float *efc_force_w = nullptr;
    if (d_nefc != nullptr && njmax > 0) {
        nefc_w = d_nefc[w];
        if (nefc_w < 0) nefc_w = 0;
        if (nefc_w > njmax) nefc_w = njmax;
        efc_type_w = (d_efc_type != nullptr) ? (d_efc_type + w64 * njmax) : nullptr;
        efc_id_w = (d_efc_id != nullptr) ? (d_efc_id + w64 * njmax) : nullptr;
        efc_force_w = (d_efc_force != nullptr) ? (d_efc_force + w64 * njmax) : nullptr;
        efc_J_w = (d_efc_J != nullptr && efc_J_row_stride > 0)
                      ? (d_efc_J + w64 * static_cast<long>(njmax) * efc_J_row_stride)
                      : nullptr;
    }
    int ncon_or_total = 0;
    const int *c_geom = nullptr;
    const int *c_adr = nullptr;
    const int *c_dim = nullptr;
    if (d_nacon != nullptr && nconmax > 0) {
        if (d_contact_worldid != nullptr) {
            ncon_or_total = d_nacon[0];
            if (ncon_or_total < 0) ncon_or_total = 0;
            if (ncon_or_total > nconmax) ncon_or_total = nconmax;
            c_geom = d_contact_geom;
            c_adr = d_contact_efc_address;
            c_dim = d_contact_dim;
        } else {
            ncon_or_total = d_nacon[w];
            if (ncon_or_total < 0) ncon_or_total = 0;
            if (ncon_or_total > nconmax) ncon_or_total = nconmax;
            c_geom = (d_contact_geom != nullptr) ? (d_contact_geom + w64 * 2 * nconmax) : nullptr;
            c_adr = (d_contact_efc_address != nullptr)
                        ? (d_contact_efc_address + w64 * nconmax * contact_efc_stride)
                        : nullptr;
            c_dim = (d_contact_dim != nullptr) ? (d_contact_dim + w64 * nconmax) : nullptr;
        }
    }
    const int nj = (d_spindle_params != nullptr) ? d_spindle_params->njnt : 0;
    const float *xpos_w = (d_xpos != nullptr) ? (d_xpos + w64 * idx.nbody * 3) : nullptr;
    const float *xanchor_w =
        (d_xanchor != nullptr && nj > 0) ? (d_xanchor + w64 * nj * 3) : nullptr;

    gpu_sp4_post_solve_world(
        w,
        dt_s,
        idx,
        cfg_w,
        d_spindle_params,
        &d_sp3_states[w],
        &d_sp4_states[w],
        d_qpos + w64 * idx.nq,
        d_qvel + w64 * idx.nv,
        xpos_w,
        d_xmat + w64 * idx.nbody * 9,
        xanchor_w,
        nsensordata,
        sd_w,
        nefc_w,
        efc_J_row_stride,
        efc_type_w,
        efc_id_w,
        efc_J_w,
        efc_force_w,
        ncon_or_total,
        contact_efc_stride,
        d_contact_worldid,
        c_geom,
        c_adr,
        c_dim);
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
    if (!bike::physics_core::GpuToolkit::isfinite(hx[0]) ||
        !bike::physics_core::GpuToolkit::isfinite(hz[0])) {
        return false;
    }
    for (int i = 0; i < count - 1; ++i) {
        const auto iu = static_cast<std::size_t>(i);
        if (!bike::physics_core::GpuToolkit::isfinite(hx[iu + 1]) ||
            !bike::physics_core::GpuToolkit::isfinite(hz[iu + 1]) || !(hx[iu + 1] > hx[iu])) {
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

inline int validate_terrain_for_tyre(
    const bike::physics_core::GpuTyreModel &host_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc &host_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const char *caller_tag
) {
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
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::validate_terrain_table_desc(
            host_terrain_desc,
            d_terrain_f32,
            d_terrain_i32,
            need_polyline,
            need_signed,
            can_check_monotonic,
            err) != bike::physics_core::Status::ok) {
        std::cerr << caller_tag << ": invalid GpuTerrainTableDesc or non-monotonic px\n";
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
            std::cerr << caller_tag << ": non-monotonic device terrain px/sp_x\n";
            return -6;
        }
    }
#endif
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

    return validate_terrain_for_tyre(
        host_tyre_model,
        host_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        "bike_gpu_launch_writers");
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

int bike_gpu_sizeof_sp3_config() {
    return static_cast<int>(sizeof(bike::gpu::GpuSp3Config));
}

int bike_gpu_sizeof_sp3_world_state() {
    return static_cast<int>(sizeof(bike::gpu::GpuSp3WorldState));
}

int bike_gpu_init_default_sp3_config(bike::gpu::GpuSp3Config *cfg) {
    if (cfg == nullptr) {
        return -1;
    }
    bike::gpu::gpu_init_default_sp3_config(*cfg);
    return 0;
}

int bike_gpu_build_sp3_sweep_configs(
    bike::gpu::GpuSp3Config *configs,
    int nworld,
    const float *fork_psi_scale,
    const float *rebound_scale,
    const float *shock_rate_scale,
    const float *tire_stiff_scale
) {
    return bike::gpu::gpu_build_sp3_sweep_configs(
        configs, nworld, fork_psi_scale, rebound_scale, shock_rate_scale, tire_stiff_scale);
}

int bike_gpu_launch_sp3_init(
    const void *cuda_stream,
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || !bike::physics_core::GpuToolkit::isfinite(init_grade) ||
        ntendon < 0 || tendon_range_world_stride < 0 ||
        (d_tendon_range != nullptr && ntendon > 0 && tendon_range_world_stride > 0 &&
         tendon_range_world_stride < 2 * ntendon) ||
        (d_tendon_range != nullptr && ntendon > 0 && tendon_range_world_stride == 0 && nworld > 1) ||
        d_idx == nullptr || d_sp3_configs == nullptr || d_sp3_states == nullptr) {
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
    if (capture_status != cudaStreamCaptureStatusActive) {
        bike::gpu::GpuModelIndices host_idx{};
        if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
            !bike::gpu::gpu_validate_model_indices(host_idx, ntendon)) {
            return -2;
        }
    }
    bike::gpu::bike_gpu_sp3_init_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld, d_idx, d_sp3_configs, per_world_config, d_sp3_states, d_world_states,
        d_qpos, init_grade, ntendon, d_tendon_range, tendon_range_world_stride);
    if (capture_status == cudaStreamCaptureStatusActive) {
        return 0;
    }
    return cudaGetLastError() == cudaSuccess ? 0 : -3;
#else
    (void)cuda_stream;
    bike::gpu::GpuModelIndices host_idx{};
    if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
        !bike::gpu::gpu_validate_model_indices(host_idx, ntendon)) {
        return -2;
    }
    for (int w = 0; w < nworld; ++w) {
        const long w64 = static_cast<long>(w);
        const bike::gpu::GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
        const float *qpos_w = (d_qpos != nullptr) ? (d_qpos + w64 * host_idx.nq) : nullptr;
        bike::gpu::GpuWorldState *tyre_w =
            (d_world_states != nullptr) ? &d_world_states[w] : nullptr;
        float *tr_w = nullptr;
        if (d_tendon_range != nullptr && ntendon > 0) {
            if (tendon_range_world_stride >= 2 * ntendon) {
                tr_w = d_tendon_range + w64 * tendon_range_world_stride;
            } else if (tendon_range_world_stride == 0 && w == 0) {
                tr_w = d_tendon_range;
            } else {
                return -1;
            }
        }
        if (bike::gpu::gpu_sp3_init_world(
                host_idx, cfg_w, &d_sp3_states[w], tyre_w, qpos_w, init_grade, ntendon, tr_w) !=
            bike::physics_core::Status::ok) {
            return -3;
        }
    }
    return 0;
#endif
}

int bike_gpu_launch_sp3_pre_solve(
    const void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride
) {
    if (nworld <= 0 || !bike::physics_core::GpuToolkit::isfinite(dt_s) || dt_s <= 0.0f ||
        ntendon < 0 || tendon_range_world_stride < 0 || dof_frictionloss_world_stride < 0 ||
        (d_tendon_range != nullptr && ntendon > 0 && tendon_range_world_stride > 0 &&
         tendon_range_world_stride < 2 * ntendon) ||
        (d_tendon_range != nullptr && ntendon > 0 && tendon_range_world_stride == 0 && nworld > 1) ||
        (d_dof_frictionloss != nullptr && dof_frictionloss_world_stride == 0 && nworld > 1) ||
        d_idx == nullptr || d_tyre_model == nullptr || d_terrain_desc == nullptr ||
        d_sp3_configs == nullptr || d_sp3_states == nullptr || d_world_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xpos == nullptr || d_xmat == nullptr ||
        d_xanchor == nullptr || d_xaxis == nullptr || d_qfrc_applied == nullptr || d_ctrl == nullptr) {
        return -1;
    }
#if defined(__CUDACC__)
    constexpr int kBlockSize = 128;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    cudaStream_t stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
    cudaStreamCaptureStatus capture_status = cudaStreamCaptureStatusNone;
    const cudaError_t cap_err = cudaStreamIsCapturing(stream, &capture_status);
    if (cap_err != cudaSuccess || capture_status == cudaStreamCaptureStatusInvalidated) {
        return -7;
    }
    if (capture_status != cudaStreamCaptureStatusActive) {
        bike::gpu::GpuModelIndices host_idx{};
        bike::physics_core::GpuTyreModel host_tm{};
        bike::physics_core::GpuTerrainTableDesc host_td{};
        if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
            !bike::gpu::copy_maybe_device_struct(d_tyre_model, host_tm) ||
            !bike::gpu::copy_maybe_device_struct(d_terrain_desc, host_td)) {
            return -2;
        }
        if (!bike::gpu::gpu_validate_model_indices(host_idx, ntendon)) {
            return -3;
        }
        if (d_dof_frictionloss != nullptr && dof_frictionloss_world_stride > 0 &&
            dof_frictionloss_world_stride < host_idx.nv) {
            return -4;
        }
        bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
        if (bike::physics_core::validate_gpu_tyre_model_device(
                &host_tm, host_idx.nq, host_idx.nv, host_idx.nbody, err) !=
            bike::physics_core::Status::ok) {
            return -5;
        }
        const int terr_rc = bike::gpu::validate_terrain_for_tyre(
            host_tm,
            host_td,
            d_terrain_f32,
            d_terrain_i32,
            "bike_gpu_launch_sp3_pre_solve");
        if (terr_rc != 0) {
            return terr_rc;
        }
    }
    bike::gpu::bike_gpu_sp3_pre_solve_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld, dt_s, d_idx, d_tyre_model, d_terrain_desc, d_terrain_f32, d_terrain_i32,
        d_sp3_configs, per_world_config, d_sp3_states, d_world_states,
        d_qpos, d_qvel, d_xpos, d_xmat, d_xanchor, d_xaxis, d_qfrc_applied, d_ctrl,
        ntendon, d_tendon_range, tendon_range_world_stride,
        d_dof_frictionloss, dof_frictionloss_world_stride);
    if (capture_status == cudaStreamCaptureStatusActive) {
        return 0;
    }
    return cudaGetLastError() == cudaSuccess ? 0 : -7;
#else
    (void)cuda_stream;
    bike::gpu::GpuModelIndices host_idx{};
    bike::physics_core::GpuTyreModel host_tm{};
    bike::physics_core::GpuTerrainTableDesc host_td{};
    if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
        !bike::gpu::copy_maybe_device_struct(d_tyre_model, host_tm) ||
        !bike::gpu::copy_maybe_device_struct(d_terrain_desc, host_td)) {
        return -2;
    }
    if (!bike::gpu::gpu_validate_model_indices(host_idx, ntendon)) {
        return -3;
    }
    if (d_dof_frictionloss != nullptr && dof_frictionloss_world_stride > 0 &&
        dof_frictionloss_world_stride < host_idx.nv) {
        return -4;
    }
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::validate_gpu_tyre_model_device(
            &host_tm, host_idx.nq, host_idx.nv, host_idx.nbody, err) !=
        bike::physics_core::Status::ok) {
        return -5;
    }
    const int terr_rc = bike::gpu::validate_terrain_for_tyre(
        host_tm,
        host_td,
        d_terrain_f32,
        d_terrain_i32,
        "bike_gpu_launch_sp3_pre_solve");
    if (terr_rc != 0) {
        return terr_rc;
    }
    for (int w = 0; w < nworld; ++w) {
        const long w64 = static_cast<long>(w);
        const bike::gpu::GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
        float *tr_w = nullptr;
        if (d_tendon_range != nullptr && ntendon > 0) {
            if (tendon_range_world_stride >= 2 * ntendon) {
                tr_w = d_tendon_range + w64 * tendon_range_world_stride;
            } else if (tendon_range_world_stride == 0 && w == 0) {
                tr_w = d_tendon_range;
            } else {
                return -1;
            }
        }
        float *fl_w = nullptr;
        if (d_dof_frictionloss != nullptr) {
            if (dof_frictionloss_world_stride >= host_idx.nv) {
                fl_w = d_dof_frictionloss + w64 * dof_frictionloss_world_stride;
            } else if (dof_frictionloss_world_stride == 0 && w == 0) {
                fl_w = d_dof_frictionloss;
            } else {
                return -1;
            }
        }
        if (bike::gpu::gpu_sp3_pre_solve_world(
                w,
                dt_s,
                host_idx,
                &host_tm,
                host_td,
                d_terrain_f32,
                d_terrain_i32,
                cfg_w,
                &d_sp3_states[w],
                d_world_states[w].compliant_wheels,
                &d_world_states[w].distributed_wheels[0][0],
                d_qpos + w64 * host_idx.nq,
                d_qvel + w64 * host_idx.nv,
                d_xpos + w64 * host_idx.nbody * 3,
                d_xmat + w64 * host_idx.nbody * 9,
                d_xanchor + w64 * host_tm.njnt * 3,
                d_xaxis + w64 * host_tm.njnt * 3,
                d_qfrc_applied + w64 * host_idx.nv,
                d_ctrl + w64 * host_idx.nu,
                ntendon,
                tr_w,
                fl_w) != bike::physics_core::Status::ok) {
            return -8;
        }
    }
    return 0;
#endif
}

int bike_gpu_launch_sp3_post_solve(
    const void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xmat,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
) {
    if (nworld <= 0 || !bike::physics_core::GpuToolkit::isfinite(dt_s) || dt_s <= 0.0f ||
        nsensordata < 0 || njmax < 0 || efc_J_row_stride < 0 ||
        nconmax < 0 || contact_efc_stride < 0 ||
        d_idx == nullptr || d_sp3_configs == nullptr || d_sp3_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xmat == nullptr) {
        return -1;
    }
#if defined(__CUDACC__)
    constexpr int kBlockSize = 128;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    cudaStream_t stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
    cudaStreamCaptureStatus capture_status = cudaStreamCaptureStatusNone;
    const cudaError_t cap_err = cudaStreamIsCapturing(stream, &capture_status);
    if (cap_err != cudaSuccess || capture_status == cudaStreamCaptureStatusInvalidated) {
        return -7;
    }
    if (capture_status != cudaStreamCaptureStatusActive) {
        bike::gpu::GpuModelIndices host_idx{};
        if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
            !bike::gpu::gpu_validate_model_indices(host_idx)) {
            return -2;
        }
    }
    bike::gpu::bike_gpu_sp3_post_solve_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld, dt_s, d_idx, d_sp3_configs, per_world_config, d_sp3_states,
        d_qpos, d_qvel, d_xmat, nsensordata, d_sensordata,
        d_nefc, njmax, efc_J_row_stride, d_efc_type, d_efc_id, d_efc_J, d_efc_force,
        d_nacon, nconmax, contact_efc_stride,
        d_contact_worldid, d_contact_geom, d_contact_efc_address, d_contact_dim);
    if (capture_status == cudaStreamCaptureStatusActive) {
        return 0;
    }
    return cudaGetLastError() == cudaSuccess ? 0 : -7;
#else
    (void)cuda_stream;
    bike::gpu::GpuModelIndices host_idx{};
    if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
        !bike::gpu::gpu_validate_model_indices(host_idx)) {
        return -2;
    }
    for (int w = 0; w < nworld; ++w) {
        const long w64 = static_cast<long>(w);
        const bike::gpu::GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
        const float *sd_w =
            (d_sensordata != nullptr && nsensordata > 0) ? (d_sensordata + w64 * nsensordata) : nullptr;
        int nefc_w = (d_nefc != nullptr && njmax > 0) ? d_nefc[w] : 0;
        if (nefc_w < 0) {
            nefc_w = 0;
        }
        if (nefc_w > njmax) {
            nefc_w = njmax;
        }
        const int *efc_type_w = (d_efc_type != nullptr && njmax > 0) ? (d_efc_type + w64 * njmax) : nullptr;
        const int *efc_id_w = (d_efc_id != nullptr && njmax > 0) ? (d_efc_id + w64 * njmax) : nullptr;
        const float *efc_force_w =
            (d_efc_force != nullptr && njmax > 0) ? (d_efc_force + w64 * njmax) : nullptr;
        const float *efc_J_w =
            (d_efc_J != nullptr && njmax > 0 && efc_J_row_stride > 0)
                ? (d_efc_J + w64 * static_cast<long>(njmax) * efc_J_row_stride)
                : nullptr;

        int ncon_or_total = 0;
        const int *c_geom = nullptr;
        const int *c_adr = nullptr;
        const int *c_dim = nullptr;
        if (d_nacon != nullptr && nconmax > 0) {
            if (d_contact_worldid != nullptr) {
                ncon_or_total = d_nacon[0];
                if (ncon_or_total < 0) {
                    ncon_or_total = 0;
                }
                if (ncon_or_total > nconmax) {
                    ncon_or_total = nconmax;
                }
                c_geom = d_contact_geom;
                c_adr = d_contact_efc_address;
                c_dim = d_contact_dim;
            } else {
                ncon_or_total = d_nacon[w];
                if (ncon_or_total < 0) {
                    ncon_or_total = 0;
                }
                if (ncon_or_total > nconmax) {
                    ncon_or_total = nconmax;
                }
                c_geom = (d_contact_geom != nullptr) ? (d_contact_geom + w64 * 2 * nconmax) : nullptr;
                c_adr = (d_contact_efc_address != nullptr)
                            ? (d_contact_efc_address + w64 * nconmax * contact_efc_stride)
                            : nullptr;
                c_dim = (d_contact_dim != nullptr) ? (d_contact_dim + w64 * nconmax) : nullptr;
            }
        }
        if (bike::gpu::gpu_sp3_post_solve_world(
                w,
                dt_s,
                host_idx,
                cfg_w,
                &d_sp3_states[w],
                d_qpos + w64 * host_idx.nq,
                d_qvel + w64 * host_idx.nv,
                d_xmat + w64 * host_idx.nbody * 9,
                nsensordata,
                sd_w,
                nefc_w,
                efc_J_row_stride,
                efc_type_w,
                efc_id_w,
                efc_J_w,
                efc_force_w,
                ncon_or_total,
                contact_efc_stride,
                d_contact_worldid,
                c_geom,
                c_adr,
                c_dim) != bike::physics_core::Status::ok) {
            return -8;
        }
    }
    return 0;
#endif
}

int bike_gpu_sizeof_sp4_world_state() {
    return static_cast<int>(sizeof(bike::gpu::GpuSp4WorldState));
}

int bike_gpu_sizeof_spindle_params() {
    return static_cast<int>(sizeof(bike::physics_core::SpindleM2K2Params<float>));
}

int bike_gpu_launch_sp4_init(
    const void *cuda_stream,
    int nworld,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    float init_grade,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride
) {
    if (nworld <= 0 || ntendon < 0 || tendon_range_world_stride < 0 ||
        d_idx == nullptr || d_sp3_configs == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr ||
        !bike::physics_core::GpuToolkit::isfinite(init_grade)) {
        return -1;
    }
    if (d_tendon_range != nullptr && ntendon > 0 &&
        ((tendon_range_world_stride == 0 && nworld > 1) ||
         (tendon_range_world_stride > 0 && tendon_range_world_stride < 2 * ntendon))) {
        return -1;
    }

#if defined(__CUDACC__)
    const auto stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
    cudaStreamCaptureStatus capture_status = cudaStreamCaptureStatusNone;
    const cudaError_t cap_err = cudaStreamIsCapturing(stream, &capture_status);
    if (cap_err != cudaSuccess || capture_status == cudaStreamCaptureStatusInvalidated) {
        return -7;
    }
    if (capture_status != cudaStreamCaptureStatusActive) {
        bike::gpu::GpuModelIndices host_idx{};
        if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
            !bike::gpu::gpu_validate_model_indices(host_idx, ntendon)) {
            return -2;
        }
    }
    constexpr int kBlockSize = 64;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    bike::gpu::bike_gpu_sp4_init_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld, d_idx, d_sp3_configs, per_world_config, d_sp3_states, d_sp4_states,
        d_world_states, d_qpos, init_grade, ntendon, d_tendon_range, tendon_range_world_stride);
    if (capture_status == cudaStreamCaptureStatusActive) {
        return 0;
    }
    const cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[bike_gpu_launch_sp4_init] CUDA error: " << cudaGetErrorString(err) << '\n';
        return -7;
    }
    return 0;
#else
    (void)cuda_stream;
    const bike::gpu::GpuModelIndices host_idx = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(host_idx, ntendon)) {
        return -1;
    }
    for (int w = 0; w < nworld; ++w) {
        const bike::gpu::GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
        const float *qpos_w = (d_qpos != nullptr)
                                  ? (d_qpos + static_cast<long>(w) * host_idx.nq)
                                  : nullptr;
        bike::gpu::GpuWorldState *ws = (d_world_states != nullptr) ? &d_world_states[w] : nullptr;
        float *tr_w = nullptr;
        if (d_tendon_range != nullptr && ntendon > 0) {
            if (tendon_range_world_stride >= 2 * ntendon) {
                tr_w = d_tendon_range + static_cast<long>(w) * tendon_range_world_stride;
            } else if (tendon_range_world_stride == 0 && w == 0) {
                tr_w = d_tendon_range;
            } else {
                return -1;
            }
        }
        if (bike::gpu::gpu_sp4_init_world(
                host_idx, cfg_w, &d_sp3_states[w], &d_sp4_states[w], ws, qpos_w,
                init_grade, ntendon, tr_w) != bike::physics_core::Status::ok) {
            return -8;
        }
    }
    return 0;
#endif
}

int bike_gpu_launch_sp4_pre_solve_and_pass0(
    const void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::physics_core::GpuTyreModel *d_tyre_model,
    const bike::physics_core::GpuTerrainTableDesc *d_terrain_desc,
    const float *d_terrain_f32,
    const bike::physics_core::i32 *d_terrain_i32,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    bike::gpu::GpuWorldState *d_world_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    const float *d_xaxis,
    const float *d_site_xpos,
    const float *d_subtree_com,
    const float *d_body_subtreemass,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_qfrc_applied,
    float *d_ctrl,
    int ntendon,
    float *d_tendon_range,
    int tendon_range_world_stride,
    float *d_dof_frictionloss,
    int dof_frictionloss_world_stride,
    float *d_qacc_warmstart
) {
    if (nworld <= 0 || dt_s <= 0.0f || !bike::physics_core::GpuToolkit::isfinite(dt_s) ||
        d_idx == nullptr || d_tyre_model == nullptr || d_terrain_desc == nullptr ||
        d_sp3_configs == nullptr || d_spindle_params == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr || d_world_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_qfrc_bias == nullptr ||
        d_xpos == nullptr || d_xmat == nullptr || d_xanchor == nullptr ||
        d_xaxis == nullptr || d_site_xpos == nullptr || d_subtree_com == nullptr ||
        d_M_or_qM == nullptr || d_qfrc_applied == nullptr || d_ctrl == nullptr ||
        M_world_stride <= 0 || ntendon < 0 ||
        tendon_range_world_stride < 0 || dof_frictionloss_world_stride < 0) {
        return -1;
    }

#if defined(__CUDACC__)
    constexpr int kBlockSize = 64;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    cudaStream_t stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
    cudaStreamCaptureStatus capture_status = cudaStreamCaptureStatusNone;
    const cudaError_t cap_err = cudaStreamIsCapturing(stream, &capture_status);
    if (cap_err != cudaSuccess || capture_status == cudaStreamCaptureStatusInvalidated) {
        return -7;
    }
    if (capture_status != cudaStreamCaptureStatusActive) {
        bike::gpu::GpuModelIndices host_idx{};
        bike::physics_core::GpuTyreModel host_tm{};
        bike::physics_core::GpuTerrainTableDesc host_td{};
        std::vector<bike::physics_core::SpindleM2K2Params<float>> host_sp(1);
        if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
            !bike::gpu::copy_maybe_device_struct(d_tyre_model, host_tm) ||
            !bike::gpu::copy_maybe_device_struct(d_terrain_desc, host_td) ||
            !bike::gpu::copy_maybe_device_struct(d_spindle_params, host_sp[0])) {
            return -2;
        }
        if (!bike::gpu::gpu_validate_model_indices(host_idx, ntendon) ||
            !bike::physics_core::validate_spindle_m2k2_params(&host_sp[0])) {
            return -3;
        }
        if (d_tendon_range != nullptr && ntendon > 0 &&
            ((tendon_range_world_stride == 0 && nworld > 1) ||
             (tendon_range_world_stride > 0 && tendon_range_world_stride < 2 * ntendon))) {
            return -4;
        }
        if (d_dof_frictionloss != nullptr &&
            ((dof_frictionloss_world_stride == 0 && nworld > 1) ||
             (dof_frictionloss_world_stride > 0 && dof_frictionloss_world_stride < host_idx.nv))) {
            return -4;
        }
        bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
        if (bike::physics_core::validate_gpu_tyre_model_device(
                &host_tm, host_idx.nq, host_idx.nv, host_idx.nbody, err) !=
            bike::physics_core::Status::ok) {
            return -5;
        }
        const int terr_rc = bike::gpu::validate_terrain_for_tyre(
            host_tm,
            host_td,
            d_terrain_f32,
            d_terrain_i32,
            "bike_gpu_launch_sp4_pre_solve_and_pass0");
        if (terr_rc != 0) {
            return terr_rc;
        }
    }
    bike::gpu::bike_gpu_sp4_pre_solve_and_pass0_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld, dt_s, d_idx, d_tyre_model, d_terrain_desc, d_terrain_f32, d_terrain_i32,
        d_sp3_configs, per_world_config, d_spindle_params, d_sp3_states, d_sp4_states, d_world_states,
        d_qpos, d_qvel, d_qfrc_bias, d_xpos, d_xmat, d_xanchor, d_xaxis,
        d_site_xpos, d_subtree_com, d_body_subtreemass,
        qM_row_stride, M_world_stride, d_M_rowadr, d_M_rownnz, d_M_colind, d_M_or_qM,
        d_qfrc_applied, d_ctrl, ntendon, d_tendon_range, tendon_range_world_stride,
        d_dof_frictionloss, dof_frictionloss_world_stride, d_qacc_warmstart);
    if (capture_status == cudaStreamCaptureStatusActive) {
        return 0;
    }
    return cudaGetLastError() == cudaSuccess ? 0 : -7;
#else
    (void)cuda_stream;
    const bike::gpu::GpuModelIndices host_idx = *d_idx;
    const bike::physics_core::GpuTyreModel host_tm = *d_tyre_model;
    const bike::physics_core::GpuTerrainTableDesc host_terrain_desc = *d_terrain_desc;
    if (!bike::gpu::gpu_validate_model_indices(host_idx, ntendon) ||
        !bike::physics_core::validate_spindle_m2k2_params(d_spindle_params)) {
        return -1;
    }
    if (d_tendon_range != nullptr && ntendon > 0 &&
        ((tendon_range_world_stride == 0 && nworld > 1) ||
         (tendon_range_world_stride > 0 && tendon_range_world_stride < 2 * ntendon))) {
        return -4;
    }
    if (d_dof_frictionloss != nullptr &&
        ((dof_frictionloss_world_stride == 0 && nworld > 1) ||
         (dof_frictionloss_world_stride > 0 && dof_frictionloss_world_stride < host_idx.nv))) {
        return -4;
    }
    bike::physics_core::ErrCode err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::validate_gpu_tyre_model_device(
            &host_tm, host_idx.nq, host_idx.nv, host_idx.nbody, err) !=
        bike::physics_core::Status::ok) {
        return -5;
    }
    const int terr_rc = bike::gpu::validate_terrain_for_tyre(
        host_tm,
        host_terrain_desc,
        d_terrain_f32,
        d_terrain_i32,
        "bike_gpu_launch_sp4_pre_solve_and_pass0");
    if (terr_rc != 0) {
        return terr_rc;
    }
    for (int w = 0; w < nworld; ++w) {
        const long w64 = static_cast<long>(w);
        const bike::gpu::GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
        float *tr_w = nullptr;
        if (d_tendon_range != nullptr && ntendon > 0) {
            if (tendon_range_world_stride >= 2 * ntendon) {
                tr_w = d_tendon_range + w64 * tendon_range_world_stride;
            } else if (tendon_range_world_stride == 0 && w == 0) {
                tr_w = d_tendon_range;
            } else {
                return -1;
            }
        }
        float *fl_w = nullptr;
        if (d_dof_frictionloss != nullptr) {
            if (dof_frictionloss_world_stride >= host_idx.nv) {
                fl_w = d_dof_frictionloss + w64 * dof_frictionloss_world_stride;
            } else if (dof_frictionloss_world_stride == 0 && w == 0) {
                fl_w = d_dof_frictionloss;
            } else {
                return -1;
            }
        }
        float *ws_w = (d_qacc_warmstart != nullptr)
            ? (d_qacc_warmstart + w64 * host_idx.nv)
            : static_cast<float *>(nullptr);
        if (bike::gpu::gpu_sp4_pre_solve_and_pass0_world(
                w, dt_s, host_idx, d_tyre_model, host_terrain_desc, d_terrain_f32, d_terrain_i32,
                cfg_w, d_spindle_params, &d_sp3_states[w], &d_sp4_states[w],
                d_world_states[w].compliant_wheels, &d_world_states[w].distributed_wheels[0][0],
                d_qpos + w64 * host_idx.nq, d_qvel + w64 * host_idx.nv,
                d_qfrc_bias + w64 * host_idx.nv,
                d_xpos + w64 * host_idx.nbody * 3, d_xmat + w64 * host_idx.nbody * 9,
                d_xanchor + w64 * d_tyre_model->njnt * 3, d_xaxis + w64 * d_tyre_model->njnt * 3,
                d_site_xpos + w64 * d_spindle_params->nsite * 3,
                d_subtree_com + w64 * host_idx.nbody * 3,
                d_body_subtreemass, qM_row_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
                d_M_or_qM + w64 * M_world_stride,
                d_qfrc_applied + w64 * host_idx.nv, d_ctrl + w64 * host_idx.nu,
                ntendon, tr_w, fl_w, ws_w) != bike::physics_core::Status::ok) {
            return -8;
        }
    }
    return 0;
#endif
}

namespace {

inline int launch_sp4_pass_stage(
    int stage,
    const void *cuda_stream,
    int nworld,
    int nq,
    int nv,
    int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos,
    const float *d_qvel,
    const float *d_qfrc_bias,
    const float *d_qacc,
    int qM_row_stride,
    int M_world_stride,
    const int *d_M_rowadr,
    const int *d_M_rownnz,
    const int *d_M_colind,
    const float *d_M_or_qM,
    float *d_ctrl,
    float *d_qacc_warmstart
) {
    if (nworld <= 0 || nq <= 0 || nv <= 0 || nu < 0 || M_world_stride <= 0 ||
        d_spindle_params == nullptr || d_sp4_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_qfrc_bias == nullptr ||
        d_qacc == nullptr || d_M_or_qM == nullptr || d_ctrl == nullptr) {
        return -1;
    }

#if defined(__CUDACC__)
    const auto stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
    cudaStreamCaptureStatus capture_status = cudaStreamCaptureStatusNone;
    const cudaError_t cap_err = cudaStreamIsCapturing(stream, &capture_status);
    if (cap_err != cudaSuccess || capture_status == cudaStreamCaptureStatusInvalidated) {
        return -7;
    }
    if (capture_status != cudaStreamCaptureStatusActive) {
        std::vector<bike::physics_core::SpindleM2K2Params<float>> host_sp(1);
        if (!bike::gpu::copy_maybe_device_struct(d_spindle_params, host_sp[0]) ||
            !bike::physics_core::validate_spindle_m2k2_params(&host_sp[0])) {
            return -2;
        }
    }
    constexpr int kBlockSize = 64;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    bike::gpu::bike_gpu_sp4_pass_stage_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        stage, nworld, nq, nv, nu, d_spindle_params, d_sp4_states,
        d_qpos, d_qvel, d_qfrc_bias, d_qacc,
        qM_row_stride, M_world_stride, d_M_rowadr, d_M_rownnz, d_M_colind, d_M_or_qM, d_ctrl,
        d_qacc_warmstart);
    if (capture_status == cudaStreamCaptureStatusActive) {
        return 0;
    }
    const cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[launch_sp4_pass_stage] CUDA error: " << cudaGetErrorString(err) << '\n';
        return -7;
    }
    return 0;
#else
    (void)cuda_stream;
    if (!bike::physics_core::validate_spindle_m2k2_params(d_spindle_params)) {
        return -1;
    }
    for (int w = 0; w < nworld; ++w) {
        const long w64 = static_cast<long>(w);
        float *ws_w = (d_qacc_warmstart != nullptr)
            ? (d_qacc_warmstart + w64 * nv)
            : static_cast<float *>(nullptr);
        bike::physics_core::Status st = bike::physics_core::Status::invalid_argument;
        if (stage == 1) {
            st = bike::gpu::gpu_sp4_pass1_world(
                d_spindle_params, &d_sp4_states[w],
                d_qpos + w64 * nq, d_qvel + w64 * nv, d_qfrc_bias + w64 * nv, d_qacc + w64 * nv,
                qM_row_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
                d_M_or_qM + w64 * M_world_stride, d_ctrl + w64 * nu, ws_w);
        } else if (stage == 2) {
            st = bike::gpu::gpu_sp4_pass2_world(
                d_spindle_params, &d_sp4_states[w],
                d_qpos + w64 * nq, d_qvel + w64 * nv, d_qfrc_bias + w64 * nv, d_qacc + w64 * nv,
                qM_row_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
                d_M_or_qM + w64 * M_world_stride, d_ctrl + w64 * nu, ws_w);
        } else if (stage == 3) {
            st = bike::gpu::gpu_sp4_pass3_world(
                d_spindle_params, &d_sp4_states[w],
                d_qpos + w64 * nq, d_qvel + w64 * nv, d_qfrc_bias + w64 * nv, d_qacc + w64 * nv,
                qM_row_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
                d_M_or_qM + w64 * M_world_stride, d_ctrl + w64 * nu, ws_w);
        }
        if (st != bike::physics_core::Status::ok) {
            return -8;
        }
    }
    return 0;
#endif
}

} // namespace

int bike_gpu_launch_sp4_pass1(
    const void *cuda_stream, int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart
) {
    return launch_sp4_pass_stage(
        1, cuda_stream, nworld, nq, nv, nu, d_spindle_params, d_sp4_states,
        d_qpos, d_qvel, d_qfrc_bias, d_qacc,
        qM_row_stride, M_world_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M_or_qM, d_ctrl, d_qacc_warmstart);
}

int bike_gpu_launch_sp4_pass2(
    const void *cuda_stream, int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart
) {
    return launch_sp4_pass_stage(
        2, cuda_stream, nworld, nq, nv, nu, d_spindle_params, d_sp4_states,
        d_qpos, d_qvel, d_qfrc_bias, d_qacc,
        qM_row_stride, M_world_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M_or_qM, d_ctrl, d_qacc_warmstart);
}

int bike_gpu_launch_sp4_pass3(
    const void *cuda_stream, int nworld, int nq, int nv, int nu,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    const float *d_qpos, const float *d_qvel, const float *d_qfrc_bias, const float *d_qacc,
    int qM_row_stride, int M_world_stride,
    const int *d_M_rowadr, const int *d_M_rownnz, const int *d_M_colind,
    const float *d_M_or_qM, float *d_ctrl, float *d_qacc_warmstart
) {
    return launch_sp4_pass_stage(
        3, cuda_stream, nworld, nq, nv, nu, d_spindle_params, d_sp4_states,
        d_qpos, d_qvel, d_qfrc_bias, d_qacc,
        qM_row_stride, M_world_stride, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M_or_qM, d_ctrl, d_qacc_warmstart);
}

int bike_gpu_launch_sp4_post_solve(
    const void *cuda_stream,
    int nworld,
    float dt_s,
    const bike::gpu::GpuModelIndices *d_idx,
    const bike::gpu::GpuSp3Config *d_sp3_configs,
    int per_world_config,
    const bike::physics_core::SpindleM2K2Params<float> *d_spindle_params,
    bike::gpu::GpuSp3WorldState *d_sp3_states,
    bike::gpu::GpuSp4WorldState *d_sp4_states,
    float *d_qpos,
    const float *d_qvel,
    const float *d_xpos,
    const float *d_xmat,
    const float *d_xanchor,
    int nsensordata,
    const float *d_sensordata,
    const int *d_nefc,
    int njmax,
    int efc_J_row_stride,
    const int *d_efc_type,
    const int *d_efc_id,
    const float *d_efc_J,
    const float *d_efc_force,
    const int *d_nacon,
    int nconmax,
    int contact_efc_stride,
    const int *d_contact_worldid,
    const int *d_contact_geom,
    const int *d_contact_efc_address,
    const int *d_contact_dim
) {
    if (nworld <= 0 || dt_s <= 0.0f || !bike::physics_core::GpuToolkit::isfinite(dt_s) ||
        d_idx == nullptr || d_sp3_configs == nullptr || d_spindle_params == nullptr ||
        d_sp3_states == nullptr || d_sp4_states == nullptr ||
        d_qpos == nullptr || d_qvel == nullptr || d_xmat == nullptr ||
        nsensordata < 0 || njmax < 0 || efc_J_row_stride < 0 || nconmax < 0 ||
        contact_efc_stride < 0 || (nconmax > 0 && contact_efc_stride == 0)) {
        return -1;
    }

#if defined(__CUDACC__)
    const auto stream = static_cast<cudaStream_t>(const_cast<void *>(cuda_stream));
    cudaStreamCaptureStatus capture_status = cudaStreamCaptureStatusNone;
    const cudaError_t cap_err = cudaStreamIsCapturing(stream, &capture_status);
    if (cap_err != cudaSuccess || capture_status == cudaStreamCaptureStatusInvalidated) {
        return -7;
    }
    if (capture_status != cudaStreamCaptureStatusActive) {
        bike::gpu::GpuModelIndices host_idx{};
        std::vector<bike::physics_core::SpindleM2K2Params<float>> host_sp(1);
        if (!bike::gpu::copy_maybe_device_struct(d_idx, host_idx) ||
            !bike::gpu::copy_maybe_device_struct(d_spindle_params, host_sp[0])) {
            return -2;
        }
        if (!bike::gpu::gpu_validate_model_indices(host_idx) ||
            !bike::physics_core::validate_spindle_m2k2_params(&host_sp[0])) {
            return -2;
        }
    }
    constexpr int kBlockSize = 64;
    const int grid_size = (nworld + kBlockSize - 1) / kBlockSize;
    bike::gpu::bike_gpu_sp4_post_solve_kernel<<<grid_size, kBlockSize, 0, stream>>>(
        nworld, dt_s, d_idx, d_sp3_configs, per_world_config, d_spindle_params,
        d_sp3_states, d_sp4_states, d_qpos, d_qvel, d_xpos, d_xmat, d_xanchor,
        nsensordata, d_sensordata, d_nefc, njmax, efc_J_row_stride,
        d_efc_type, d_efc_id, d_efc_J, d_efc_force,
        d_nacon, nconmax, contact_efc_stride, d_contact_worldid,
        d_contact_geom, d_contact_efc_address, d_contact_dim);
    if (capture_status == cudaStreamCaptureStatusActive) {
        return 0;
    }
    const cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[bike_gpu_launch_sp4_post_solve] CUDA error: "
                  << cudaGetErrorString(err) << '\n';
        return -7;
    }
    return 0;
#else
    (void)cuda_stream;
    const bike::gpu::GpuModelIndices host_idx = *d_idx;
    if (!bike::gpu::gpu_validate_model_indices(host_idx) ||
        !bike::physics_core::validate_spindle_m2k2_params(d_spindle_params)) {
        return -1;
    }
    const int nj = d_spindle_params->njnt;
    for (int w = 0; w < nworld; ++w) {
        const long w64 = static_cast<long>(w);
        const bike::gpu::GpuSp3Config *cfg_w = &d_sp3_configs[per_world_config ? w : 0];
        const float *sd_w = (d_sensordata != nullptr && nsensordata > 0)
                                ? (d_sensordata + w64 * nsensordata)
                                : nullptr;
        int nefc_w = 0;
        const int *efc_type_w = nullptr;
        const int *efc_id_w = nullptr;
        const float *efc_J_w = nullptr;
        const float *efc_force_w = nullptr;
        if (d_nefc != nullptr && njmax > 0) {
            nefc_w = d_nefc[w];
            if (nefc_w < 0) nefc_w = 0;
            if (nefc_w > njmax) nefc_w = njmax;
            efc_type_w = (d_efc_type != nullptr) ? (d_efc_type + w64 * njmax) : nullptr;
            efc_id_w = (d_efc_id != nullptr) ? (d_efc_id + w64 * njmax) : nullptr;
            efc_force_w = (d_efc_force != nullptr) ? (d_efc_force + w64 * njmax) : nullptr;
            efc_J_w = (d_efc_J != nullptr && efc_J_row_stride > 0)
                          ? (d_efc_J + w64 * static_cast<long>(njmax) * efc_J_row_stride)
                          : nullptr;
        }
        int ncon_or_total = 0;
        const int *c_geom = nullptr;
        const int *c_adr = nullptr;
        const int *c_dim = nullptr;
        if (d_nacon != nullptr && nconmax > 0) {
            if (d_contact_worldid != nullptr) {
                ncon_or_total = d_nacon[0];
                if (ncon_or_total < 0) ncon_or_total = 0;
                if (ncon_or_total > nconmax) ncon_or_total = nconmax;
                c_geom = d_contact_geom;
                c_adr = d_contact_efc_address;
                c_dim = d_contact_dim;
            } else {
                ncon_or_total = d_nacon[w];
                if (ncon_or_total < 0) ncon_or_total = 0;
                if (ncon_or_total > nconmax) ncon_or_total = nconmax;
                c_geom = (d_contact_geom != nullptr) ? (d_contact_geom + w64 * 2 * nconmax) : nullptr;
                c_adr = (d_contact_efc_address != nullptr)
                            ? (d_contact_efc_address + w64 * nconmax * contact_efc_stride)
                            : nullptr;
                c_dim = (d_contact_dim != nullptr) ? (d_contact_dim + w64 * nconmax) : nullptr;
            }
        }
        const float *xpos_w = (d_xpos != nullptr) ? (d_xpos + w64 * host_idx.nbody * 3) : nullptr;
        const float *xanchor_w =
            (d_xanchor != nullptr && nj > 0) ? (d_xanchor + w64 * nj * 3) : nullptr;

        if (bike::gpu::gpu_sp4_post_solve_world(
                w, dt_s, host_idx, cfg_w, d_spindle_params,
                &d_sp3_states[w], &d_sp4_states[w],
                d_qpos + w64 * host_idx.nq, d_qvel + w64 * host_idx.nv,
                xpos_w, d_xmat + w64 * host_idx.nbody * 9, xanchor_w,
                nsensordata, sd_w, nefc_w, efc_J_row_stride,
                efc_type_w, efc_id_w, efc_J_w, efc_force_w,
                ncon_or_total, contact_efc_stride, d_contact_worldid,
                c_geom, c_adr, c_dim) != bike::physics_core::Status::ok) {
            return -8;
        }
    }
    return 0;
#endif
}

int bike_gpu_sp3_state_has_failure(const bike::gpu::GpuSp3WorldState *st) {
    if (st == nullptr) {
        return -1;
    }
    return (st->sp3.monitors.balance.triggered != 0 ||
            st->sp3.monitors.crash.cause != bike::physics_core::CrashCause::none ||
            st->last_efc.contact_crash_cause != bike::physics_core::CrashCause::none)
               ? 1
               : 0;
}

int bike_gpu_sp3_state_failure_flags(const bike::gpu::GpuSp3WorldState *st) {
    if (st == nullptr) {
        return -1;
    }
    unsigned int flags = 0U;
    if (st->sp3.monitors.balance.triggered != 0) {
        flags |= 1U;
    }
    if (st->sp3.monitors.crash.cause != bike::physics_core::CrashCause::none) {
        flags |= 2U;
    }
    if (st->last_efc.contact_crash_cause != bike::physics_core::CrashCause::none) {
        flags |= 4U;
    }
    return static_cast<int>(flags);
}

} // extern "C"

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
