// native/src/gpu/sp4_closed_loop_step.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// SP4 closed-loop 4-pass M2_K2 step integration uniting SP3 autonomous controllers with
// physics_core::SpindleM2K2 leg/upper-body actuation.
//
// Invariants:
// - Host C++20/23, CUDA, and Apple Metal MSL 2.4+ safe (no std::*, no double literals, no if constexpr).
// - Every pointer parameter uses its own template type so Metal non-unified address spaces
//   accept device/constant/thread pointers without implicit casts.
// - Never copy GpuSp3WorldState, GpuSp4WorldState, or SpindleM2K2Params onto GPU `thread` stack;
//   always mutate via device/host pointers.
#pragma once

#include "../physics_core/spindle_m0.hpp"
#include "../physics_core/spindle_m2k2.hpp"
#include "sp3_autonomous_step.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI; templated pointer parameters carry distinct device/constant/thread address spaces in MSL. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

struct GpuSp4WorldState {
    bike::physics_core::SpindleM2K2State<float> spindle{};
    bike::physics_core::SpindleM2K2StepScratch<float> scratch{};
    bike::physics_core::SpindleCommand<float> last_cmd{};
    float saved_qacc_warmstart[bike::physics_core::kMaxNv]{};
    float last_posture_min_lean_rad = 0.0f;
    float min_press_front_n = 1.0e9f;
    float min_press_rear_n = 1.0e9f;
    float max_rising_front_excess_n = -1.0e9f;
    float max_rising_rear_excess_n = -1.0e9f;
    float min_saddle_force_n = 1.0e9f;
    float max_saddle_force_n = -1.0e9f;
    float sum_delivered_crank_torque_nm = 0.0f;
    float sum_commanded_crank_torque_nm = 0.0f;
    float start_root_x_m = 0.0f;
    float end_root_x_m = 0.0f;
    float end_speed_mps = 0.0f;
    float max_speed_mps = 0.0f;
    float energy_residual_j = 0.0f;
    float energy_scale_j = 1.0f;
    float max_rel_energy_residual = 0.0f;
    bike::physics_core::i32 sample_count = 0;
    bike::physics_core::i32 max_m0_passes_used = 0;
    bike::physics_core::i32 current_m0_passes = 0;
    bike::physics_core::u8 has_posture_min_lean = 0;
    bike::physics_core::u8 has_start_root_x = 0;
    bike::physics_core::u8 active_control_tick = 0;
    bike::physics_core::u8 last_cmd_enabled = 1;
    bike::physics_core::u8 energy_contract_ok = 1;
    bike::physics_core::u8 solver_mode = 0; // 0 = m2k2, 1 = m0
    bike::physics_core::u8 pad_sp4[2]{};
};

static_assert(
    sizeof(GpuSp4WorldState) <= 2560,
    "GpuSp4WorldState should stay compact (<= 2560 bytes per world)");

template <
    class Sp3CfgPtr,
    class Sp3StatePtr,
    class Sp4StatePtr,
    class WsPtr,
    class QposPtr,
    class TendonRangePtr>
BIKE_HD bike::physics_core::Status gpu_sp4_init_world(
    const BIKE_THREAD GpuModelIndices &idx,
    Sp3CfgPtr sp3_cfg,
    Sp3StatePtr sp3_st,
    Sp4StatePtr sp4_st,
    WsPtr ws,
    QposPtr qpos_w,
    float init_grade,
    int ntendon,
    TendonRangePtr tendon_range_w
) noexcept {
    if (sp4_st == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    const bike::physics_core::Status st = gpu_sp3_init_world(
        idx, sp3_cfg, sp3_st, ws, qpos_w, init_grade, ntendon, tendon_range_w);
    if (st != bike::physics_core::Status::ok) {
        return st;
    }

    sp4_st->spindle = bike::physics_core::SpindleM2K2State<float>{};
    sp4_st->scratch = bike::physics_core::SpindleM2K2StepScratch<float>{};
    sp4_st->last_cmd = bike::physics_core::SpindleCommand<float>{};
    for (int i = 0; i < bike::physics_core::kMaxNv; ++i) {
        sp4_st->saved_qacc_warmstart[i] = 0.0f;
    }
    sp4_st->last_posture_min_lean_rad = 0.0f;
    sp4_st->min_press_front_n = 1.0e9f;
    sp4_st->min_press_rear_n = 1.0e9f;
    sp4_st->max_rising_front_excess_n = -1.0e9f;
    sp4_st->max_rising_rear_excess_n = -1.0e9f;
    sp4_st->min_saddle_force_n = 1.0e9f;
    sp4_st->max_saddle_force_n = -1.0e9f;
    sp4_st->sum_delivered_crank_torque_nm = 0.0f;
    sp4_st->sum_commanded_crank_torque_nm = 0.0f;
    if (qpos_w != nullptr && idx.root_x_qpos >= 0 && idx.root_x_qpos < idx.nq) {
        const float x0 = qpos_w[idx.root_x_qpos];
        sp4_st->start_root_x_m = x0;
        sp4_st->end_root_x_m = x0;
        sp4_st->has_start_root_x = 1;
    } else {
        sp4_st->start_root_x_m = 0.0f;
        sp4_st->end_root_x_m = 0.0f;
        sp4_st->has_start_root_x = 0;
    }
    sp4_st->end_speed_mps = 0.0f;
    sp4_st->max_speed_mps = 0.0f;
    sp4_st->energy_residual_j = 0.0f;
    sp4_st->energy_scale_j = 1.0f;
    sp4_st->max_rel_energy_residual = 0.0f;
    sp4_st->sample_count = 0;
    sp4_st->max_m0_passes_used = 0;
    sp4_st->current_m0_passes = 0;
    sp4_st->has_posture_min_lean = 0;
    sp4_st->active_control_tick = 0;
    sp4_st->last_cmd_enabled = 1;
    sp4_st->energy_contract_ok = 1;
    sp4_st->solver_mode = 0;
    sp4_st->pad_sp4[0] = 0;
    sp4_st->pad_sp4[1] = 0;
    return bike::physics_core::Status::ok;
}

template <
    class Sp3CfgPtr,
    class Sp3StatePtr,
    class Sp4StatePtr,
    class M0ScratchPtr,
    class WsPtr,
    class QposPtr,
    class TendonRangePtr>
BIKE_HD bike::physics_core::Status gpu_sp4_m0_init_world(
    const BIKE_THREAD GpuModelIndices &idx,
    Sp3CfgPtr sp3_cfg,
    Sp3StatePtr sp3_st,
    Sp4StatePtr sp4_st,
    M0ScratchPtr m0_scratch_w,
    WsPtr ws,
    QposPtr qpos_w,
    float init_grade,
    int ntendon,
    TendonRangePtr tendon_range_w
) noexcept {
    const bike::physics_core::Status st = gpu_sp4_init_world(
        idx, sp3_cfg, sp3_st, sp4_st, ws, qpos_w, init_grade, ntendon, tendon_range_w);
    if (st != bike::physics_core::Status::ok) {
        return st;
    }
    sp4_st->solver_mode = 1;
    if (m0_scratch_w != nullptr) {
        *m0_scratch_w = bike::physics_core::SpindleM0Scratch<float>{};
    }
    return bike::physics_core::Status::ok;
}

namespace detail {

template <
    class Sp3CfgPtr,
    class SpindleParamsPtr,
    class Sp3StatePtr,
    class Sp4StatePtr,
    class CompliantStatePtr,
    class DistStatePtr,
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class TendonRangePtr,
    class FrictionLossPtr>
BIKE_HD bike::physics_core::Status gpu_sp4_prepare_pre_solve_and_cmd(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_DEVICE const bike::physics_core::GpuTyreModel *tyre_model,
    const BIKE_THREAD bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const bike::physics_core::i32 *terrain_i32,
    Sp3CfgPtr sp3_cfg,
    SpindleParamsPtr spindle_params,
    Sp3StatePtr sp3_st,
    Sp4StatePtr sp4_st,
    CompliantStatePtr compliant_wheels_w,
    DistStatePtr dist_wheels_w,
    const BIKE_THREAD float *qpos_w,
    const BIKE_THREAD float *qvel_w,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    BIKE_THREAD float *qfrc_w,
    BIKE_THREAD float *ctrl_w,
    int ntendon,
    TendonRangePtr tendon_range_w,
    FrictionLossPtr dof_frictionloss_w,
    BIKE_THREAD bike::physics_core::SpindleCommand<float> &out_cmd,
    BIKE_THREAD bool &out_frozen
) noexcept {
    out_cmd = bike::physics_core::SpindleCommand<float>{};
    out_frozen = (sp3_st->frozen != 0 && sp3_cfg->freeze_on_crash != 0);
    if (out_frozen) {
        for (int i = 0; i < idx.nv; ++i) {
            qfrc_w[i] = 0.0f;
        }
        for (int i = 0; i < idx.nu; ++i) {
            ctrl_w[i] = 0.0f;
        }
        return bike::physics_core::Status::ok;
    }

    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const float kPi = 3.14159265358979323846f;
    const float min_elbow_rad =
        sp3_cfg->intent.min_elbow_flexion_deg * (kPi / 180.0f);
    bike::physics_core::u8 has_spindle_min_lean = 0;
    float spindle_min_lean_rad = 0.0f;
    if (sp3_cfg->intent.enabled != 0) {
        bike::physics_core::spindle_m2k2_min_torso_lean_rad<
            float, bike::physics_core::GpuToolkit>(
            spindle_params,
            qpos_w,
            xpos_w,
            xmat_w,
            xanchor_w,
            min_elbow_rad,
            has_spindle_min_lean,
            spindle_min_lean_rad);
    }

    const bool has_lean_limit =
        (sp3_cfg->has_lean_limit != 0 || spindle_params->has_lean_limit != 0);
    float lean_limit_rad = sp3_cfg->intent.max_forward_lean_rad;
    if (sp3_cfg->has_lean_limit != 0 && sp3_cfg->lean_limit_rad < lean_limit_rad) {
        lean_limit_rad = sp3_cfg->lean_limit_rad;
    }
    if (spindle_params->has_lean_limit != 0 && spindle_params->lean_limit_rad < lean_limit_rad) {
        lean_limit_rad = spindle_params->lean_limit_rad;
    }

    bool eff_has_min_lean = false;
    float eff_min_lean_rad = 0.0f;
    if (has_spindle_min_lean != 0 && sp3_cfg->has_min_lean != 0) {
        eff_has_min_lean = true;
        eff_min_lean_rad = gpu_max(spindle_min_lean_rad, sp3_cfg->min_lean_rad);
    } else if (has_spindle_min_lean != 0) {
        eff_has_min_lean = true;
        eff_min_lean_rad = spindle_min_lean_rad;
    } else if (sp3_cfg->has_min_lean != 0) {
        eff_has_min_lean = true;
        eff_min_lean_rad = sp3_cfg->min_lean_rad;
    }

    sp4_st->has_posture_min_lean = has_spindle_min_lean;
    sp4_st->last_posture_min_lean_rad = spindle_min_lean_rad;

    const bike::physics_core::i32 period =
        (spindle_params->control_period_steps > 0)
            ? spindle_params->control_period_steps
            : ((sp3_cfg->control_period_steps > 1) ? sp3_cfg->control_period_steps : 1);
    const bool control_tick =
        (sp4_st->spindle.has_held_torques == 0) || ((sp3_st->step_index % period) == 0);

    bike::physics_core::IntentResolutionCore<float> intent_res{};
    bike::physics_core::PedalingOutput<float> pedal_out{};
    const bike::physics_core::Status sp3_st_code = detail::gpu_sp3_pre_solve_core(
        w,
        dt_s,
        idx,
        tyre_model,
        terrain_desc,
        terrain_f32,
        terrain_i32,
        sp3_cfg,
        sp3_st,
        compliant_wheels_w,
        dist_wheels_w,
        qpos_w,
        qvel_w,
        xpos_w,
        xmat_w,
        xanchor_w,
        xaxis_w,
        qfrc_w,
        ctrl_w,
        ntendon,
        tendon_range_w,
        dof_frictionloss_w,
        true,
        has_lean_limit,
        lean_limit_rad,
        eff_has_min_lean,
        eff_min_lean_rad,
        false,
        control_tick,
        &intent_res,
        &pedal_out);
    if (sp3_st_code != bike::physics_core::Status::ok) {
        return sp3_st_code;
    }

    out_cmd.mean_crank_torque_nm = (sp3_cfg->effort_mode != 0) ? pedal_out.effort_nm : 0.0f;
    out_cmd.enabled = intent_res.control.rider_enabled;
    out_cmd.torso_lean_rad =
        (intent_res.control.has_posture != 0) ? intent_res.control.posture.torso_lean_rad : 0.0f;
    const bool crank_reposition =
        (pedal_out.reason == bike::physics_core::CoastReason::reposition_requested ||
         pedal_out.reason == bike::physics_core::CoastReason::reposition_stall);
    out_cmd.crank_reposition = crank_reposition ? 1u : 0u;
    out_cmd.crank_target_rate_rad_s = crank_reposition
        ? pedal_out.target_rate_rad_s
        : ((intent_res.control.has_crank_target_rate_rad_s != 0)
               ? intent_res.control.crank_target_rate_rad_s
               : pedal_out.target_rate_rad_s);
    out_cmd.weld_crank_torque_nm = sp3_st->last_efc.delivered_crank_torque_nm;
    out_cmd.has_weld_crank_torque = 1;
    out_cmd.steady_state = 0;
    out_cmd.advance = 1;
    return bike::physics_core::Status::ok;
}

} // namespace detail

template <
    class Sp3CfgPtr,
    class SpindleParamsPtr,
    class Sp3StatePtr,
    class Sp4StatePtr,
    class CompliantStatePtr,
    class DistStatePtr,
    class PtrBias,
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class PtrSitePos,
    class PtrSubtreeCom,
    class PtrSubtreeMass,
    class PtrRowAdr,
    class PtrRowNnz,
    class PtrColInd,
    class PtrM,
    class TendonRangePtr,
    class FrictionLossPtr,
    class PtrWarmstart>
BIKE_HD bike::physics_core::Status gpu_sp4_pre_solve_and_pass0_world(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_DEVICE const bike::physics_core::GpuTyreModel *tyre_model,
    const BIKE_THREAD bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const bike::physics_core::i32 *terrain_i32,
    Sp3CfgPtr sp3_cfg,
    SpindleParamsPtr spindle_params,
    Sp3StatePtr sp3_st,
    Sp4StatePtr sp4_st,
    CompliantStatePtr compliant_wheels_w,
    DistStatePtr dist_wheels_w,
    const BIKE_THREAD float *qpos_w,
    const BIKE_THREAD float *qvel_w,
    PtrBias qfrc_bias_w,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    PtrSitePos site_xpos_w,
    PtrSubtreeCom subtree_com_w,
    PtrSubtreeMass body_subtreemass,
    int qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM_w,
    BIKE_THREAD float *qfrc_w,
    BIKE_THREAD float *ctrl_w,
    int ntendon,
    TendonRangePtr tendon_range_w,
    FrictionLossPtr dof_frictionloss_w,
    PtrWarmstart qacc_warmstart_w
) noexcept {
    if (w < 0 || sp3_cfg == nullptr || spindle_params == nullptr ||
        sp3_st == nullptr || sp4_st == nullptr ||
        qpos_w == nullptr || qvel_w == nullptr || qfrc_bias_w == nullptr ||
        xpos_w == nullptr || xmat_w == nullptr || xanchor_w == nullptr ||
        xaxis_w == nullptr || site_xpos_w == nullptr || subtree_com_w == nullptr ||
        d_M_or_qM_w == nullptr || qfrc_w == nullptr || ctrl_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    if (!bike::physics_core::validate_spindle_m2k2_params(spindle_params)) {
        return bike::physics_core::Status::invalid_argument;
    }

    bike::physics_core::SpindleCommand<float> cmd{};
    bool frozen = false;
    const bike::physics_core::Status prep_st = detail::gpu_sp4_prepare_pre_solve_and_cmd(
        w, dt_s, idx, tyre_model, terrain_desc, terrain_f32, terrain_i32,
        sp3_cfg, spindle_params, sp3_st, sp4_st,
        compliant_wheels_w, dist_wheels_w, qpos_w, qvel_w,
        xpos_w, xmat_w, xanchor_w, xaxis_w, qfrc_w, ctrl_w,
        ntendon, tendon_range_w, dof_frictionloss_w, cmd, frozen);
    if (prep_st != bike::physics_core::Status::ok) {
        return prep_st;
    }
    if (frozen) {
        sp4_st->scratch.active_newton = 0;
        sp4_st->active_control_tick = 0;
        return bike::physics_core::Status::ok;
    }

    bike::physics_core::spindle_m2k2_pass0_init_and_base<
        float, bike::physics_core::GpuToolkit>(
        spindle_params,
        cmd,
        sp3_st->step_index,
        qpos_w,
        qvel_w,
        qfrc_bias_w,
        xmat_w,
        xpos_w,
        xanchor_w,
        xaxis_w,
        site_xpos_w,
        subtree_com_w,
        body_subtreemass,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_or_qM_w,
        &sp4_st->spindle,
        &sp4_st->scratch,
        ctrl_w);

    if (qacc_warmstart_w != nullptr && idx.nv > 0) {
        const int ws_cnt = (idx.nv < bike::physics_core::kMaxNv)
            ? idx.nv
            : bike::physics_core::kMaxNv;
        for (int i = 0; i < ws_cnt; ++i) {
            sp4_st->saved_qacc_warmstart[i] = qacc_warmstart_w[i];
        }
    }
    sp4_st->last_cmd = cmd;
    sp4_st->last_cmd_enabled = sp4_st->spindle.command_enabled;
    sp4_st->active_control_tick = sp4_st->scratch.active_newton;
    return bike::physics_core::Status::ok;
}

template <class Sp4StatePtr, class PtrWarmstart>
BIKE_HD void sp4_restore_qacc_warmstart(
    Sp4StatePtr sp4_st,
    int nv,
    PtrWarmstart qacc_warmstart_w
) noexcept {
    if (qacc_warmstart_w != nullptr && nv > 0) {
        const int ws_cnt = (nv < bike::physics_core::kMaxNv)
            ? nv
            : bike::physics_core::kMaxNv;
        for (int i = 0; i < ws_cnt; ++i) {
            qacc_warmstart_w[i] = sp4_st->saved_qacc_warmstart[i];
        }
    }
}

template <
    class SpindleParamsPtr,
    class Sp4StatePtr,
    class PtrQpos,
    class PtrQvel,
    class PtrBias,
    class PtrQacc,
    class PtrRowAdr,
    class PtrRowNnz,
    class PtrColInd,
    class PtrM,
    class PtrCtrl,
    class PtrWarmstart>
BIKE_HD bike::physics_core::Status gpu_sp4_pass1_world(
    SpindleParamsPtr spindle_params,
    Sp4StatePtr sp4_st,
    PtrQpos qpos_w,
    PtrQvel qvel_w,
    PtrBias qfrc_bias_w,
    PtrQacc qacc_w,
    int qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM_w,
    PtrCtrl ctrl_w,
    PtrWarmstart qacc_warmstart_w
) noexcept {
    if (spindle_params == nullptr || sp4_st == nullptr ||
        qpos_w == nullptr || qvel_w == nullptr || qfrc_bias_w == nullptr ||
        qacc_w == nullptr || d_M_or_qM_w == nullptr || ctrl_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    sp4_restore_qacc_warmstart(sp4_st, spindle_params->nv, qacc_warmstart_w);
    bike::physics_core::spindle_m2k2_pass1_perturb0<
        float, bike::physics_core::GpuToolkit>(
        spindle_params,
        qpos_w,
        qvel_w,
        qfrc_bias_w,
        qacc_w,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_or_qM_w,
        &sp4_st->spindle,
        &sp4_st->scratch,
        ctrl_w);
    return bike::physics_core::Status::ok;
}

template <
    class SpindleParamsPtr,
    class Sp4StatePtr,
    class PtrQpos,
    class PtrQvel,
    class PtrBias,
    class PtrQacc,
    class PtrRowAdr,
    class PtrRowNnz,
    class PtrColInd,
    class PtrM,
    class PtrCtrl,
    class PtrWarmstart>
BIKE_HD bike::physics_core::Status gpu_sp4_pass2_world(
    SpindleParamsPtr spindle_params,
    Sp4StatePtr sp4_st,
    PtrQpos qpos_w,
    PtrQvel qvel_w,
    PtrBias qfrc_bias_w,
    PtrQacc qacc_w,
    int qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM_w,
    PtrCtrl ctrl_w,
    PtrWarmstart qacc_warmstart_w
) noexcept {
    if (spindle_params == nullptr || sp4_st == nullptr ||
        qpos_w == nullptr || qvel_w == nullptr || qfrc_bias_w == nullptr ||
        qacc_w == nullptr || d_M_or_qM_w == nullptr || ctrl_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    sp4_restore_qacc_warmstart(sp4_st, spindle_params->nv, qacc_warmstart_w);
    bike::physics_core::spindle_m2k2_pass2_perturb1<
        float, bike::physics_core::GpuToolkit>(
        spindle_params,
        qpos_w,
        qvel_w,
        qfrc_bias_w,
        qacc_w,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_or_qM_w,
        &sp4_st->spindle,
        &sp4_st->scratch,
        ctrl_w);
    return bike::physics_core::Status::ok;
}

template <
    class SpindleParamsPtr,
    class Sp4StatePtr,
    class PtrQpos,
    class PtrQvel,
    class PtrBias,
    class PtrQacc,
    class PtrRowAdr,
    class PtrRowNnz,
    class PtrColInd,
    class PtrM,
    class PtrCtrl,
    class PtrWarmstart>
BIKE_HD bike::physics_core::Status gpu_sp4_pass3_world(
    SpindleParamsPtr spindle_params,
    Sp4StatePtr sp4_st,
    PtrQpos qpos_w,
    PtrQvel qvel_w,
    PtrBias qfrc_bias_w,
    PtrQacc qacc_w,
    int qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM_w,
    PtrCtrl ctrl_w,
    PtrWarmstart qacc_warmstart_w
) noexcept {
    if (spindle_params == nullptr || sp4_st == nullptr ||
        qpos_w == nullptr || qvel_w == nullptr || qfrc_bias_w == nullptr ||
        qacc_w == nullptr || d_M_or_qM_w == nullptr || ctrl_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    sp4_restore_qacc_warmstart(sp4_st, spindle_params->nv, qacc_warmstart_w);
    bike::physics_core::spindle_m2k2_pass3_commit<
        float, bike::physics_core::GpuToolkit>(
        spindle_params,
        qpos_w,
        qvel_w,
        qfrc_bias_w,
        qacc_w,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_or_qM_w,
        &sp4_st->spindle,
        &sp4_st->scratch,
        ctrl_w);
    return bike::physics_core::Status::ok;
}

template <
    class Sp3CfgPtr,
    class SpindleParamsPtr,
    class Sp3StatePtr,
    class Sp4StatePtr,
    class M0ScratchPtr,
    class CompliantStatePtr,
    class DistStatePtr,
    class PtrBias,
    class XposPtr,
    class XmatPtr,
    class XanchorPtr,
    class XaxisPtr,
    class PtrSitePos,
    class PtrSubtreeCom,
    class PtrSubtreeMass,
    class PtrRowAdr,
    class PtrRowNnz,
    class PtrColInd,
    class PtrM,
    class TendonRangePtr,
    class FrictionLossPtr,
    class PtrWarmstart>
BIKE_HD bike::physics_core::Status gpu_sp4_m0_pre_solve_and_begin_world(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    BIKE_DEVICE const bike::physics_core::GpuTyreModel *tyre_model,
    const BIKE_THREAD bike::physics_core::GpuTerrainTableDesc &terrain_desc,
    BIKE_DEVICE const float *terrain_f32,
    BIKE_DEVICE const bike::physics_core::i32 *terrain_i32,
    Sp3CfgPtr sp3_cfg,
    SpindleParamsPtr spindle_params,
    Sp3StatePtr sp3_st,
    Sp4StatePtr sp4_st,
    M0ScratchPtr m0_scratch_w,
    CompliantStatePtr compliant_wheels_w,
    DistStatePtr dist_wheels_w,
    const BIKE_THREAD float *qpos_w,
    const BIKE_THREAD float *qvel_w,
    PtrBias qfrc_bias_w,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    XanchorPtr xanchor_w,
    XaxisPtr xaxis_w,
    PtrSitePos site_xpos_w,
    PtrSubtreeCom subtree_com_w,
    PtrSubtreeMass body_subtreemass,
    int qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM_w,
    BIKE_THREAD float *qfrc_w,
    BIKE_THREAD float *ctrl_w,
    int ntendon,
    TendonRangePtr tendon_range_w,
    FrictionLossPtr dof_frictionloss_w,
    PtrWarmstart qacc_warmstart_w,
    BIKE_THREAD bike::physics_core::u8 &out_needs_solve,
    BIKE_THREAD bike::physics_core::u8 &out_done
) noexcept {
    out_needs_solve = 0;
    out_done = 1;
    if (w < 0 || sp3_cfg == nullptr || spindle_params == nullptr ||
        sp3_st == nullptr || sp4_st == nullptr || m0_scratch_w == nullptr ||
        qpos_w == nullptr || qvel_w == nullptr || qfrc_bias_w == nullptr ||
        xpos_w == nullptr || xmat_w == nullptr || xanchor_w == nullptr ||
        xaxis_w == nullptr || site_xpos_w == nullptr || subtree_com_w == nullptr ||
        d_M_or_qM_w == nullptr || qfrc_w == nullptr || ctrl_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    if (!bike::physics_core::validate_spindle_m2k2_params(spindle_params)) {
        return bike::physics_core::Status::invalid_argument;
    }

    sp4_st->solver_mode = 1;
    bike::physics_core::SpindleCommand<float> cmd{};
    bool frozen = false;
    const bike::physics_core::Status prep_st = detail::gpu_sp4_prepare_pre_solve_and_cmd(
        w, dt_s, idx, tyre_model, terrain_desc, terrain_f32, terrain_i32,
        sp3_cfg, spindle_params, sp3_st, sp4_st,
        compliant_wheels_w, dist_wheels_w, qpos_w, qvel_w,
        xpos_w, xmat_w, xanchor_w, xaxis_w, qfrc_w, ctrl_w,
        ntendon, tendon_range_w, dof_frictionloss_w, cmd, frozen);
    if (prep_st != bike::physics_core::Status::ok) {
        return prep_st;
    }
    if (frozen) {
        m0_scratch_w->done = 1;
        sp4_st->current_m0_passes = 0;
        sp4_st->active_control_tick = 0;
        out_needs_solve = 0;
        out_done = 1;
        return bike::physics_core::Status::ok;
    }

    bike::physics_core::spindle_m0_begin<
        float, bike::physics_core::GpuToolkit>(
        spindle_params,
        cmd,
        sp3_st->step_index,
        qpos_w,
        qvel_w,
        qfrc_bias_w,
        xmat_w,
        xpos_w,
        xanchor_w,
        xaxis_w,
        site_xpos_w,
        subtree_com_w,
        body_subtreemass,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_or_qM_w,
        &sp4_st->spindle,
        m0_scratch_w,
        ctrl_w,
        out_needs_solve,
        out_done);

    if (qacc_warmstart_w != nullptr && idx.nv > 0) {
        const int ws_cnt = (idx.nv < bike::physics_core::kMaxNv)
            ? idx.nv
            : bike::physics_core::kMaxNv;
        for (int i = 0; i < ws_cnt; ++i) {
            sp4_st->saved_qacc_warmstart[i] = qacc_warmstart_w[i];
        }
    }
    sp4_st->last_cmd = cmd;
    sp4_st->last_cmd_enabled = sp4_st->spindle.command_enabled;
    sp4_st->active_control_tick = (out_needs_solve != 0) ? 1u : 0u;
    sp4_st->current_m0_passes = (out_needs_solve != 0) ? 1 : 0;
    if (sp4_st->current_m0_passes > sp4_st->max_m0_passes_used) {
        sp4_st->max_m0_passes_used = sp4_st->current_m0_passes;
    }
    return bike::physics_core::Status::ok;
}

template <
    class Sp3CfgPtr,
    class SpindleParamsPtr,
    class Sp3StatePtr,
    class Sp4StatePtr,
    class M0ScratchPtr,
    class PtrQpos,
    class PtrQvel,
    class PtrBias,
    class PtrQacc,
    class PtrRowAdr,
    class PtrRowNnz,
    class PtrColInd,
    class PtrM,
    class PtrCtrl,
    class PtrWarmstart>
BIKE_HD bike::physics_core::Status gpu_sp4_m0_step_world(
    int w,
    const BIKE_THREAD GpuModelIndices &idx,
    Sp3CfgPtr sp3_cfg,
    SpindleParamsPtr spindle_params,
    Sp3StatePtr sp3_st,
    Sp4StatePtr sp4_st,
    M0ScratchPtr m0_scratch_w,
    PtrQpos qpos_w,
    PtrQvel qvel_w,
    PtrBias qfrc_bias_w,
    PtrQacc qacc_w,
    int qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM_w,
    PtrCtrl ctrl_w,
    PtrWarmstart qacc_warmstart_w,
    BIKE_THREAD bike::physics_core::u8 &out_needs_solve,
    BIKE_THREAD bike::physics_core::u8 &out_done,
    bike::physics_core::SpindleM0Options<float> opts =
        bike::physics_core::SpindleM0Options<float>{}
) noexcept {
    out_needs_solve = 0;
    out_done = 1;
    if (w < 0 || sp3_cfg == nullptr || spindle_params == nullptr ||
        sp3_st == nullptr || sp4_st == nullptr || m0_scratch_w == nullptr ||
        qpos_w == nullptr || qvel_w == nullptr || qfrc_bias_w == nullptr ||
        qacc_w == nullptr || d_M_or_qM_w == nullptr || ctrl_w == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    if ((sp3_st->frozen != 0 && sp3_cfg->freeze_on_crash != 0) ||
        m0_scratch_w->done != 0) {
        out_needs_solve = 0;
        out_done = 1;
        return bike::physics_core::Status::ok;
    }

    sp4_restore_qacc_warmstart(sp4_st, idx.nv, qacc_warmstart_w);
    bike::physics_core::spindle_m0_step<
        float, bike::physics_core::GpuToolkit>(
        spindle_params,
        qpos_w,
        qvel_w,
        qfrc_bias_w,
        qacc_w,
        qM_row_stride,
        M_rowadr,
        M_rownnz,
        M_colind,
        d_M_or_qM_w,
        &sp4_st->spindle,
        m0_scratch_w,
        ctrl_w,
        out_needs_solve,
        out_done,
        opts);

    if (out_needs_solve != 0) {
        sp4_st->current_m0_passes += 1;
        if (sp4_st->current_m0_passes > sp4_st->max_m0_passes_used) {
            sp4_st->max_m0_passes_used = sp4_st->current_m0_passes;
        }
    }
    return bike::physics_core::Status::ok;
}

template <
    class Sp3CfgPtr,
    class SpindleParamsPtr,
    class Sp3StatePtr,
    class Sp4StatePtr,
    class QposPtr,
    class QvelPtr,
    class XposPtr,
    class XmatPtr,
    class AnchorPtr,
    class SensorPtr,
    class PtrType,
    class PtrId,
    class PtrJ,
    class PtrForce,
    class PtrWorldId,
    class PtrGeom,
    class PtrEfcAdr,
    class PtrDim,
    class M0ScratchPtr = const bike::physics_core::SpindleM0Scratch<float> *>
BIKE_HD bike::physics_core::Status gpu_sp4_post_solve_world(
    int w,
    float dt_s,
    const BIKE_THREAD GpuModelIndices &idx,
    Sp3CfgPtr sp3_cfg,
    SpindleParamsPtr spindle_params,
    Sp3StatePtr sp3_st,
    Sp4StatePtr sp4_st,
    QposPtr qpos_w,
    QvelPtr qvel_w,
    XposPtr xpos_w,
    XmatPtr xmat_w,
    AnchorPtr xanchor_w,
    int nsensordata,
    SensorPtr sensordata_w,
    int nefc_w,
    int efc_J_row_stride,
    PtrType efc_type_w,
    PtrId efc_id_w,
    PtrJ efc_J_w,
    PtrForce efc_force_w,
    int ncon_or_total,
    int contact_efc_stride,
    PtrWorldId contact_worldid,
    PtrGeom contact_geom,
    PtrEfcAdr contact_efc_address,
    PtrDim contact_dim,
    M0ScratchPtr m0_scratch_w = static_cast<const bike::physics_core::SpindleM0Scratch<float> *>(nullptr)
) noexcept {
    if (sp4_st == nullptr || sp3_cfg == nullptr || sp3_st == nullptr) {
        return bike::physics_core::Status::invalid_argument;
    }
    if (sp3_st->frozen != 0 && sp3_cfg->freeze_on_crash != 0) {
        return bike::physics_core::Status::ok;
    }
    const bike::physics_core::Status sp3_st_code = gpu_sp3_post_solve_world(
        w,
        dt_s,
        idx,
        sp3_cfg,
        sp3_st,
        qpos_w,
        qvel_w,
        xmat_w,
        nsensordata,
        sensordata_w,
        nefc_w,
        efc_J_row_stride,
        efc_type_w,
        efc_id_w,
        efc_J_w,
        efc_force_w,
        ncon_or_total,
        contact_efc_stride,
        contact_worldid,
        contact_geom,
        contact_efc_address,
        contact_dim);
    if (sp3_st_code != bike::physics_core::Status::ok) {
        return sp3_st_code;
    }

    const float time_s = static_cast<float>(sp3_st->step_index) * dt_s;
    const bool use_m0 = (sp4_st->solver_mode == 1) && (m0_scratch_w != nullptr);
    float phase_rad = use_m0 ? m0_scratch_w->step.phase : sp4_st->scratch.phase;
    const float f_weight_front =
        use_m0 ? m0_scratch_w->step.f_weight[0] : sp4_st->scratch.f_weight[0];
    const float f_weight_rear =
        use_m0 ? m0_scratch_w->step.f_weight[1] : sp4_st->scratch.f_weight[1];
    if (spindle_params != nullptr && xpos_w != nullptr && xanchor_w != nullptr &&
        spindle_params->frame_body >= 0 && spindle_params->pedal_bodies[0] >= 0 &&
        spindle_params->crank_joint >= 0) {
        const int fb9 = 9 * spindle_params->frame_body;
        const int p0 = 3 * spindle_params->pedal_bodies[0];
        const int cj3 = 3 * spindle_params->crank_joint;
        const float dx = xpos_w[p0 + 0] - xanchor_w[cj3 + 0];
        const float dy = xpos_w[p0 + 1] - xanchor_w[cj3 + 1];
        const float dz = xpos_w[p0 + 2] - xanchor_w[cj3 + 2];
        const float sp_x = xmat_w[fb9 + 0] * dx + xmat_w[fb9 + 3] * dy + xmat_w[fb9 + 6] * dz;
        const float sp_z = xmat_w[fb9 + 2] * dx + xmat_w[fb9 + 5] * dy + xmat_w[fb9 + 8] * dz;
        phase_rad = bike::physics_core::SpindlePortableTrig::atan2_val<
            float, bike::physics_core::GpuToolkit>(-sp_z, sp_x);
    }

    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>
    const float kPi = 3.14159265358979323846f;
    const float rad2deg = 180.0f / kPi;

    if (sp3_st->last_efc.has_foot_front != 0) {
        const float f_press = sp3_st->last_efc.foot_front_press_n;
        sp4_st->min_press_front_n = gpu_min(sp4_st->min_press_front_n, f_press);
        if (time_s > 2.0f) {
            float deg_f = phase_rad * rad2deg;
            deg_f = deg_f - 360.0f * gpu_floor(deg_f / 360.0f);
            if (deg_f >= 100.0f && deg_f <= 240.0f) {
                const float ex_f = f_press - f_weight_front;
                sp4_st->max_rising_front_excess_n =
                    gpu_max(sp4_st->max_rising_front_excess_n, ex_f);
            }
        }
    }
    if (sp3_st->last_efc.has_foot_rear != 0) {
        const float r_press = sp3_st->last_efc.foot_rear_press_n;
        sp4_st->min_press_rear_n = gpu_min(sp4_st->min_press_rear_n, r_press);
        if (time_s > 2.0f) {
            float deg_r = (phase_rad + kPi) * rad2deg;
            deg_r = deg_r - 360.0f * gpu_floor(deg_r / 360.0f);
            if (deg_r >= 100.0f && deg_r <= 240.0f) {
                const float ex_r = r_press - f_weight_rear;
                sp4_st->max_rising_rear_excess_n =
                    gpu_max(sp4_st->max_rising_rear_excess_n, ex_r);
            }
        }
    }
    if (sp3_st->last_efc.has_saddle != 0) {
        const float s_press = sp3_st->last_efc.pelvis_saddle_force_n;
        sp4_st->min_saddle_force_n = gpu_min(sp4_st->min_saddle_force_n, s_press);
        sp4_st->max_saddle_force_n = gpu_max(sp4_st->max_saddle_force_n, s_press);
    }

    sp4_st->sum_delivered_crank_torque_nm += sp3_st->last_efc.delivered_crank_torque_nm;
    sp4_st->sum_commanded_crank_torque_nm += sp3_st->last_commanded_crank_torque_nm;
    if (idx.root_x_qpos >= 0 && idx.root_x_qpos < idx.nq) {
        const float rx = qpos_w[idx.root_x_qpos];
        if (sp4_st->has_start_root_x == 0) {
            sp4_st->start_root_x_m = rx;
            sp4_st->has_start_root_x = 1;
        }
        sp4_st->end_root_x_m = rx;
    }
    if (idx.root_x_dof >= 0 && idx.root_x_dof < idx.nv) {
        const float spd = qvel_w[idx.root_x_dof];
        sp4_st->end_speed_mps = spd;
        if (sp4_st->sample_count == 0 || spd > sp4_st->max_speed_mps) {
            sp4_st->max_speed_mps = spd;
        }
    }
    sp4_st->energy_residual_j = sp3_st->energy_ledger.residual_j;
    sp4_st->energy_scale_j = sp3_st->energy_ledger.energy_scale_j;
    sp4_st->max_rel_energy_residual = sp3_st->energy_ledger.max_rel_residual;
    sp4_st->energy_contract_ok =
        (sp3_st->energy_ledger.initialized == 0 ||
         bike::physics_core::energy_ledger_contract_ok<float, bike::physics_core::GpuToolkit>(
             &sp3_st->energy_ledger, 0.05f, 1.0e-3f))
            ? 1u
            : 0u;
    sp4_st->sample_count += 1;
    return bike::physics_core::Status::ok;
}

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
