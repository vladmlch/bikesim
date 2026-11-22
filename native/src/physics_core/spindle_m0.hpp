// native/src/physics_core/spindle_m0.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL 2.4..3.1+)
// exact multi-iteration Armijo Newton SpindleM0 pedal-press solver state machine
// mirroring CPU rider/spindle_controller.cpp (compute() M0) over shared kinematics,
// subtree_com leg weights, CSR/dense M*v, strength/power limits, and upper-body PD.
//
// Strict tri-target hygiene:
// - Zero `double` literals (always `BIKE_REAL(Real, ...)`).
// - Zero `std::` headers/calls inside BIKE_GPU_SHARED_CODE_BEGIN..END.
// - Zero `if constexpr` (MSL 2.4 is C++14; type dispatch uses function overloading).
// - Zero heap allocations, zero lambdas, zero member methods on POD structs.
// - All reference parameters carry explicit BIKE_REF / BIKE_CREF / BIKE_THREAD.
// - Distinct template pointer types for every pointer parameter so `constant`, `device`,
//   and `thread` address spaces never alias the same deduced type on MSL < 3.2.
#pragma once

#include "common.hpp"
#include "gpu_math.hpp"
#include "spindle_m2k2.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: POD structs and templated pointers match the CUDA/MSL ABI without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kM0MaxNewtonIters = 6;
BIKE_CONSTEXPR i32 kM0LineSearchTrials = 5;
BIKE_CONSTEXPR i32 kM0MaxPasses = 1 + kM0MaxNewtonIters * (2 + kM0LineSearchTrials);
BIKE_CONSTEXPR i32 kM0MaxStateTransitions = 128;

template <class Real>
using SpindleM0Params = SpindleM2K2Params<Real>;

template <class Real>
using SpindleM0State = SpindleM2K2State<Real>;

enum class SpindleM0Stage : u8 {
    done = 0,
    wait_initial_eval = 1,
    wait_perturb0 = 2,
    wait_perturb1 = 3,
    wait_line_search = 4
};

template <class Real>
struct SpindleM0Options {
    Real newton_eps = BIKE_REAL(Real, 1e-3);
    Real conv_tol = BIKE_REAL(Real, 1e-6);
    Real det_eps = BIKE_REAL(Real, 1e-12);
};

template <>
struct SpindleM0Options<float> {
    float newton_eps = 1e-2f;
    float conv_tol = 1e-4f;
    float det_eps = 1e-12f;
};

template <class Real>
struct SpindleM0Scratch {
    SpindleM2K2StepScratch<Real> step{};
    PressStateCore<Real> ps{};
    PressStateCore<Real> P{};
    PressStateCore<Real> P0{};
    PressStateCore<Real> P1{};
    PressStateCore<Real> cached_P{};
    PressStateCore<Real> ps_next{};
    Real cached_req[4]{};
    Real cached_prio[4]{};
    Real base_req[4]{};
    Real base_prio[4]{};
    Real pending_req[4]{};
    Real pending_prio[4]{};
    Real r0 = BIKE_REAL(Real, 0.0);
    Real r1 = BIKE_REAL(Real, 0.0);
    Real r2 = BIKE_REAL(Real, 0.0);
    Real err = BIKE_REAL(Real, 0.0);
    Real dp0 = BIKE_REAL(Real, 0.0);
    Real dp1 = BIKE_REAL(Real, 0.0);
    Real dp2 = BIKE_REAL(Real, 0.0);
    Real alpha = BIKE_REAL(Real, 1.0);
    i32 iter = 0;
    i32 ls = 0;
    u8 stage = 0;
    u8 has_cached_eval = 0;
    u8 done = 1;
    u8 pad_m0 = 0;
};

namespace detail_m0 {

enum class StepPhase : u8 {
    check_iter = 0,
    start_perturb0 = 1,
    start_perturb1 = 2,
    compute_step = 3,
    start_line_search = 4,
    check_line_search = 5,
    finalize = 6
};

template <class Real, class RefPtr>
BIKE_HD bool m0_leg_req_matches(
    const BIKE_THREAD Real (&req)[kNumSpindleJoints],
    const BIKE_THREAD Real (&prio)[kNumSpindleJoints],
    RefPtr ref_req,
    RefPtr ref_prio
) noexcept {
    for (i32 j = 0; j < 4; ++j) {
        if (req[5 + j] != ref_req[j] || prio[5 + j] != ref_prio[j]) {
            return false;
        }
    }
    return true;
}

template <class Real, class TK, class Trig,
          class ParamsPtr, class ScratchPtr, class PtrQpos, class PtrQvel, class PtrCtrl>
BIKE_HD void m0_emit_trial_ctrl(
    ParamsPtr params,
    ScratchPtr scratch,
    const BIKE_THREAD Real (&req)[kNumSpindleJoints],
    const BIKE_THREAD Real (&prio)[kNumSpindleJoints],
    PtrQpos qpos,
    PtrQvel qvel,
    PtrCtrl ctrl,
    SpindleM0Stage next_stage,
    BIKE_THREAD u8 &needs_solve_out,
    BIKE_THREAD u8 &done_out
) noexcept {
    for (i32 j = 0; j < 4; ++j) {
        scratch->pending_req[j] = req[5 + j];
        scratch->pending_prio[j] = prio[5 + j];
    }
    Real exc[kNumSpindleJoints]{};
    limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
    write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
    scratch->stage = static_cast<u8>(next_stage);
    scratch->done = 0;
    needs_solve_out = 1;
    done_out = 0;
}

template <class Real, class TK, class Trig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrCtrl>
BIKE_HD void m0_finalize(
    ParamsPtr params,
    StatePtr state,
    ScratchPtr scratch,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrCtrl ctrl,
    BIKE_THREAD u8 &needs_solve_out,
    BIKE_THREAD u8 &done_out
) noexcept {
    Real req[kNumSpindleJoints]{};
    Real prio[kNumSpindleJoints]{};
    Real exc[kNumSpindleJoints]{};
    populate_leg_torques_core<Real, TK, Trig>(
        params, &scratch->step, scratch->ps, scratch->step.advance, qvel, state, req, prio);
    limit_torques_core<Real, TK, Trig>(params, req, prio, qpos, qvel, exc);
    if (scratch->step.advance != 0) {
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            state->held_torques[i] = exc[i];
        }
        state->has_held_torques = 1;
        state->has_prev_ps = 0;
        state->prev_ps = PressStateCore<Real>{};
    }
    write_ctrl_with_speed_gate<Real, TK, Trig>(params, exc, qvel, 1u, ctrl);
    scratch->stage = static_cast<u8>(SpindleM0Stage::done);
    scratch->done = 1;
    needs_solve_out = 0;
    done_out = 1;
}

} // namespace detail_m0

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias,
          class PtrXmat, class PtrPos, class PtrAnchor, class PtrAxis,
          class PtrSitePos, class PtrSubtreeCom, class PtrSubtreeMass,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m0_begin(
    ParamsPtr params,
    SpindleCommand<Real> cmd,
    i32 step_index,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrXmat body_xmat,
    PtrPos body_xpos,
    PtrAnchor jnt_xanchor,
    PtrAxis jnt_xaxis,
    PtrSitePos site_xpos,
    PtrSubtreeCom subtree_com,
    PtrSubtreeMass body_subtreemass,
    i32 qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl,
    BIKE_THREAD u8 &needs_solve_out,
    BIKE_THREAD u8 &done_out
) noexcept {
    needs_solve_out = 0;
    done_out = 1;
    scratch->stage = static_cast<u8>(SpindleM0Stage::done);
    scratch->has_cached_eval = 0;
    scratch->iter = 0;
    scratch->ls = 0;
    scratch->done = 1;
    scratch->step.active_newton = 0;
    scratch->step.converged = 1;

    if (!validate_spindle_m2k2_params(params)) {
        return;
    }

    const u8 cmd_en = (cmd.enabled != 0 && state->enabled != 0) ? 1u : 0u;
    state->command_enabled = cmd_en;
    scratch->step.command_enabled = cmd_en;
    scratch->step.advance = cmd.advance;
    scratch->step.steady_state = cmd.steady_state;

    if (cmd_en == 0) {
        if (cmd.advance != 0) {
            state->coast_hold_nm = BIKE_REAL(Real, 0.0);
            for (i32 i = 0; i < kNumSpindleJoints; ++i) {
                state->held_torques[i] = BIKE_REAL(Real, 0.0);
            }
            state->has_held_torques = 1;
            state->has_prev_ps = 0;
            state->prev_ps = PressStateCore<Real>{};
        }
        for (i32 i = 0; i < kNumSpindleJoints; ++i) {
            const i32 aid = params->joint_actuator_id[i];
            if (aid >= 0) {
                ctrl[aid] = BIKE_REAL(Real, 0.0);
            }
        }
        return;
    }

    const i32 period = (params->control_period_steps > 0) ? params->control_period_steps : 1;
    const u8 is_control_tick =
        (cmd.advance == 0 || cmd.steady_state != 0 || state->has_held_torques == 0 ||
         (step_index % period == 0))
            ? 1u
            : 0u;
    if (is_control_tick == 0) {
        spindle_m2k2_write_held_ctrl<Real, TK, Trig>(
            params, state, qpos, qvel, cmd_en, ctrl);
        return;
    }

    spindle_populate_step_scratch_invariants<Real, TK, Trig>(
        params, cmd, qpos, qvel, qfrc_bias,
        body_xmat, body_xpos, jnt_xanchor, jnt_xaxis,
        site_xpos, subtree_com, body_subtreemass, state, &scratch->step);

    const PressStateCore<Real> ps0 = compute_press_state_zero_qacc_m<Real>(
        params, &scratch->step, qM_row_stride, M_rowadr, M_rownnz, M_colind,
        d_M_or_qM, qfrc_bias);
    scratch->ps = ps0;

    if (cmd.steady_state != 0) {
        detail_m0::m0_finalize<Real, TK, Trig>(
            params, state, scratch, qpos, qvel, ctrl, needs_solve_out, done_out);
        return;
    }

    Real req[kNumSpindleJoints]{};
    Real prio[kNumSpindleJoints]{};
    populate_leg_torques_core<Real, TK, Trig>(
        params, &scratch->step, ps0, 0u, qvel, state, req, prio);
    detail_m0::m0_emit_trial_ctrl<Real, TK, Trig>(
        params, scratch, req, prio, qpos, qvel, ctrl,
        SpindleM0Stage::wait_initial_eval, needs_solve_out, done_out);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias,
          class PtrXmat, class PtrPos, class PtrAnchor, class PtrAxis,
          class PtrSitePos, class PtrSubtreeCom, class PtrSubtreeMass,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m0_begin(
    ParamsPtr params,
    SpindleCommand<Real> cmd,
    i32 step_index,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrXmat body_xmat,
    PtrPos body_xpos,
    PtrAnchor jnt_xanchor,
    PtrAxis jnt_xaxis,
    PtrSitePos site_xpos,
    PtrSubtreeCom subtree_com,
    PtrSubtreeMass body_subtreemass,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl,
    BIKE_THREAD u8 &needs_solve_out,
    BIKE_THREAD u8 &done_out
) noexcept {
    spindle_m0_begin<Real, TK, Trig>(
        params, cmd, step_index, qpos, qvel, qfrc_bias,
        body_xmat, body_xpos, jnt_xanchor, jnt_xaxis,
        site_xpos, subtree_com, body_subtreemass,
        0, M_rowadr, M_rownnz, M_colind, d_M,
        state, scratch, ctrl, needs_solve_out, done_out);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias, class PtrQacc,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m0_step(
    ParamsPtr params,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrQacc qacc,
    i32 qM_row_stride,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M_or_qM,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl,
    BIKE_THREAD u8 &needs_solve_out,
    BIKE_THREAD u8 &done_out,
    SpindleM0Options<Real> opts = SpindleM0Options<Real>{}
) noexcept {
    needs_solve_out = 0;
    if (!validate_spindle_m2k2_params(params) || scratch->done != 0) {
        done_out = 1;
        return;
    }

    const PressStateCore<Real> eval_p = compute_press_state_m<Real>(
        params, &scratch->step, qM_row_stride, M_rowadr, M_rownnz, M_colind,
        d_M_or_qM, qacc, qfrc_bias);
    for (i32 j = 0; j < 4; ++j) {
        scratch->cached_req[j] = scratch->pending_req[j];
        scratch->cached_prio[j] = scratch->pending_prio[j];
    }
    scratch->cached_P = eval_p;
    scratch->has_cached_eval = 1;

    PressStateCore<Real> p_next_eval{};
    detail_m0::StepPhase phase = detail_m0::StepPhase::finalize;
    const SpindleM0Stage cur_stage = static_cast<SpindleM0Stage>(scratch->stage);
    if (cur_stage == SpindleM0Stage::wait_initial_eval) {
        scratch->P = eval_p;
        scratch->iter = 0;
        phase = detail_m0::StepPhase::check_iter;
    } else if (cur_stage == SpindleM0Stage::wait_perturb0) {
        scratch->P0 = eval_p;
        phase = detail_m0::StepPhase::start_perturb1;
    } else if (cur_stage == SpindleM0Stage::wait_perturb1) {
        scratch->P1 = eval_p;
        phase = detail_m0::StepPhase::compute_step;
    } else if (cur_stage == SpindleM0Stage::wait_line_search) {
        p_next_eval = eval_p;
        phase = detail_m0::StepPhase::check_line_search;
    } else {
        phase = detail_m0::StepPhase::finalize;
    }

    for (i32 trans = 0; trans < kM0MaxStateTransitions; ++trans) {
        if (phase == detail_m0::StepPhase::check_iter) {
            if (scratch->iter >= kM0MaxNewtonIters) {
                phase = detail_m0::StepPhase::finalize;
                continue;
            }
            const Real r0 = scratch->P.f_passive[0] - scratch->ps.f_passive[0];
            const Real r1 = scratch->P.f_passive[1] - scratch->ps.f_passive[1];
            const Real r2 = scratch->P.max_total_press_n - scratch->ps.max_total_press_n;
            const Real err = TK::max(TK::abs(r0), TK::max(TK::abs(r1), TK::abs(r2)));
            if (err <= opts.conv_tol) {
                phase = detail_m0::StepPhase::finalize;
                continue;
            }
            scratch->r0 = r0;
            scratch->r1 = r1;
            scratch->r2 = r2;
            scratch->err = err;

            Real req_base[kNumSpindleJoints]{};
            Real prio_base[kNumSpindleJoints]{};
            populate_leg_torques_core<Real, TK, Trig>(
                params, &scratch->step, scratch->ps, 0u, qvel, state, req_base, prio_base);
            for (i32 j = 0; j < 4; ++j) {
                scratch->base_req[j] = req_base[5 + j];
                scratch->base_prio[j] = prio_base[5 + j];
            }
            phase = detail_m0::StepPhase::start_perturb0;
            continue;
        }

        if (phase == detail_m0::StepPhase::start_perturb0) {
            PressStateCore<Real> ps_p0 = scratch->ps;
            ps_p0.f_passive[0] += opts.newton_eps;
            Real req[kNumSpindleJoints]{};
            Real prio[kNumSpindleJoints]{};
            populate_leg_torques_core<Real, TK, Trig>(
                params, &scratch->step, ps_p0, 0u, qvel, state, req, prio);
            if (detail_m0::m0_leg_req_matches<Real>(req, prio, scratch->base_req, scratch->base_prio)) {
                scratch->P0 = scratch->P;
                phase = detail_m0::StepPhase::start_perturb1;
                continue;
            }
            if (scratch->has_cached_eval != 0 &&
                detail_m0::m0_leg_req_matches<Real>(req, prio, scratch->cached_req, scratch->cached_prio)) {
                scratch->P0 = scratch->cached_P;
                phase = detail_m0::StepPhase::start_perturb1;
                continue;
            }
            detail_m0::m0_emit_trial_ctrl<Real, TK, Trig>(
                params, scratch, req, prio, qpos, qvel, ctrl,
                SpindleM0Stage::wait_perturb0, needs_solve_out, done_out);
            return;
        }

        if (phase == detail_m0::StepPhase::start_perturb1) {
            PressStateCore<Real> ps_p1 = scratch->ps;
            ps_p1.f_passive[1] += opts.newton_eps;
            Real req[kNumSpindleJoints]{};
            Real prio[kNumSpindleJoints]{};
            populate_leg_torques_core<Real, TK, Trig>(
                params, &scratch->step, ps_p1, 0u, qvel, state, req, prio);
            if (detail_m0::m0_leg_req_matches<Real>(req, prio, scratch->base_req, scratch->base_prio)) {
                scratch->P1 = scratch->P;
                phase = detail_m0::StepPhase::compute_step;
                continue;
            }
            if (scratch->has_cached_eval != 0 &&
                detail_m0::m0_leg_req_matches<Real>(req, prio, scratch->cached_req, scratch->cached_prio)) {
                scratch->P1 = scratch->cached_P;
                phase = detail_m0::StepPhase::compute_step;
                continue;
            }
            detail_m0::m0_emit_trial_ctrl<Real, TK, Trig>(
                params, scratch, req, prio, qpos, qvel, ctrl,
                SpindleM0Stage::wait_perturb1, needs_solve_out, done_out);
            return;
        }

        if (phase == detail_m0::StepPhase::compute_step) {
            const Real eps = opts.newton_eps;
            const Real J00 = (scratch->P0.f_passive[0] - scratch->P.f_passive[0]) / eps - BIKE_REAL(Real, 1.0);
            const Real J10 = (scratch->P0.f_passive[1] - scratch->P.f_passive[1]) / eps;
            const Real J20 = (scratch->P0.max_total_press_n - scratch->P.max_total_press_n) / eps;
            const Real J01 = (scratch->P1.f_passive[0] - scratch->P.f_passive[0]) / eps;
            const Real J11 = (scratch->P1.f_passive[1] - scratch->P.f_passive[1]) / eps - BIKE_REAL(Real, 1.0);
            const Real J21 = (scratch->P1.max_total_press_n - scratch->P.max_total_press_n) / eps;
            const Real det = J00 * J11 - J01 * J10;
            if (TK::abs(det) < opts.det_eps) {
                phase = detail_m0::StepPhase::finalize;
                continue;
            }
            const Real dp0 = -(J11 * scratch->r0 - J01 * scratch->r1) / det;
            const Real dp1 = -(-J10 * scratch->r0 + J00 * scratch->r1) / det;
            const Real dp2 = scratch->r2 + J20 * dp0 + J21 * dp1;
            scratch->dp0 = dp0;
            scratch->dp1 = dp1;
            scratch->dp2 = dp2;
            scratch->alpha = BIKE_REAL(Real, 1.0);
            scratch->ls = 0;
            phase = detail_m0::StepPhase::start_line_search;
            continue;
        }

        if (phase == detail_m0::StepPhase::start_line_search) {
            scratch->ps_next.f_passive[0] = scratch->ps.f_passive[0] + scratch->alpha * scratch->dp0;
            scratch->ps_next.f_passive[1] = scratch->ps.f_passive[1] + scratch->alpha * scratch->dp1;
            scratch->ps_next.max_total_press_n = scratch->ps.max_total_press_n + scratch->alpha * scratch->dp2;
            Real req[kNumSpindleJoints]{};
            Real prio[kNumSpindleJoints]{};
            populate_leg_torques_core<Real, TK, Trig>(
                params, &scratch->step, scratch->ps_next, 0u, qvel, state, req, prio);
            if (detail_m0::m0_leg_req_matches<Real>(req, prio, scratch->base_req, scratch->base_prio)) {
                p_next_eval = scratch->P;
                phase = detail_m0::StepPhase::check_line_search;
                continue;
            }
            if (scratch->has_cached_eval != 0 &&
                detail_m0::m0_leg_req_matches<Real>(req, prio, scratch->cached_req, scratch->cached_prio)) {
                p_next_eval = scratch->cached_P;
                phase = detail_m0::StepPhase::check_line_search;
                continue;
            }
            detail_m0::m0_emit_trial_ctrl<Real, TK, Trig>(
                params, scratch, req, prio, qpos, qvel, ctrl,
                SpindleM0Stage::wait_line_search, needs_solve_out, done_out);
            return;
        }

        if (phase == detail_m0::StepPhase::check_line_search) {
            const Real next_err = TK::max(
                TK::abs(p_next_eval.f_passive[0] - scratch->ps_next.f_passive[0]),
                TK::max(
                    TK::abs(p_next_eval.f_passive[1] - scratch->ps_next.f_passive[1]),
                    TK::abs(p_next_eval.max_total_press_n - scratch->ps_next.max_total_press_n)));
            if (next_err < scratch->err || (scratch->ls + 1) >= kM0LineSearchTrials) {
                scratch->ps = scratch->ps_next;
                scratch->P = p_next_eval;
                scratch->iter += 1;
                phase = detail_m0::StepPhase::check_iter;
                continue;
            }
            scratch->alpha *= BIKE_REAL(Real, 0.5);
            scratch->ls += 1;
            phase = detail_m0::StepPhase::start_line_search;
            continue;
        }

        break;
    }

    detail_m0::m0_finalize<Real, TK, Trig>(
        params, state, scratch, qpos, qvel, ctrl, needs_solve_out, done_out);
}

template <class Real, class TK, class Trig = SpindlePortableTrig,
          class ParamsPtr, class StatePtr, class ScratchPtr,
          class PtrQpos, class PtrQvel, class PtrBias, class PtrQacc,
          class PtrRowAdr, class PtrRowNnz, class PtrColInd, class PtrM,
          class PtrCtrl>
BIKE_HD void spindle_m0_step(
    ParamsPtr params,
    PtrQpos qpos,
    PtrQvel qvel,
    PtrBias qfrc_bias,
    PtrQacc qacc,
    PtrRowAdr M_rowadr,
    PtrRowNnz M_rownnz,
    PtrColInd M_colind,
    PtrM d_M,
    StatePtr state,
    ScratchPtr scratch,
    PtrCtrl ctrl,
    BIKE_THREAD u8 &needs_solve_out,
    BIKE_THREAD u8 &done_out,
    SpindleM0Options<Real> opts = SpindleM0Options<Real>{}
) noexcept {
    spindle_m0_step<Real, TK, Trig>(
        params, qpos, qvel, qfrc_bias, qacc,
        0, M_rowadr, M_rownnz, M_colind, d_M,
        state, scratch, ctrl, needs_solve_out, done_out, opts);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
