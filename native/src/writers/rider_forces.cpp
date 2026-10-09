// writers/rider_forces.cpp — expression-for-expression port of
// rider_forces.py's _JointPath.compute + RiderForceApplier.apply; comments
// cite file:line at the time of porting. Python max(a, b) maps to std::max —
// same two-argument keep-first-on-tie semantics, including the NaN cases.
#include "rider_forces.hpp"
#include "../config_validation.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"

#include <algorithm>
#include <exception>
#include <new>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
    // rider_forces.py:109-115 — _resolve_slide, same checks and order.
    std::pair<int, int> resolve_slide(const mjModel *m,
                                      const std::string &joint_name) {
        const int jid = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
        if (jid < 0)
            throw std::invalid_argument(
                "model has no joint '" + joint_name +
                "'; the seated rider's forces cannot be applied");
        // Checked extents everywhere — -Wunsafe-buffer-usage rejects raw
        // indexing into mjModel pointer fields (same pattern as suspension.cpp).
        model_access::require_id(jid, m->njnt, "rider slide joint");
        const std::span<const int> jnt_type =
                model_access::readonly_buffer(m->jnt_type, m->njnt);
        const std::span<const mjtBool> limited =
                model_access::readonly_buffer(m->jnt_limited, m->njnt);
        if (jnt_type[static_cast<std::size_t>(jid)] != mjJNT_SLIDE ||
            limited[static_cast<std::size_t>(jid)])
            throw std::invalid_argument(
                "joint '" + joint_name +
                "' must be an unlimited slide for the rider force path");
        const std::span<const int> qposadr =
                model_access::readonly_buffer(m->jnt_qposadr, m->njnt);
        const std::span<const int> dofadr =
                model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
        const int qadr = qposadr[static_cast<std::size_t>(jid)];
        const int vadr = dofadr[static_cast<std::size_t>(jid)];
        // The stored addresses index d->qpos[nq] / d->qvel[nv] on every
        // apply — a corrupt model table is rejected here, not at the read.
        model_access::require_id(qadr, m->nq, "rider slide qpos address");
        model_access::require_id(vadr, m->nv, "rider slide dof address");
        return {qadr, vadr};
    }
} // namespace

RiderForcesWriter::RiderForcesWriter(const mjModel *m,
                                     const nativecfg::RiderForcesConfig &config)
    : nq_(m->nq), nv_(m->nv) {
    nativecfg::validate(config);
    // rider_forces.py:98-106 — one _JointPath per pose body, resolved in
    // order; an empty paths list is the inert pose=None case (qfrc stays
    // all-zero, mirroring apply()'s no-op on a zeroed buffer).
    paths_.reserve(config.paths.size());
    last_.reserve(config.paths.size());
    for (const nativecfg::RiderPathConfig &p: config.paths) {
        const auto [qposadr, dofadr] = resolve_slide(m, p.joint);
        paths_.push_back({
            .name = p.name,
            .qposadr = qposadr, .dofadr = dofadr, .stiffness_n_m = p.stiffness_n_m, .damping_ns_m = p.damping_ns_m,
            .preload_deflection_m = p.preload_deflection_m, .offset_m = p.offset_m, .unilateral = p.unilateral
        });
        last_.push_back({.force_n = 0.0, .gap_m = 0.0});
    }
    // R1: the boxing surface's zeros(nv) buffer is construction-sized —
    // per-tick compute fills this capacity instead of allocating.
    out_.resize(static_cast<std::size_t>(nv_));
}

// Caller precondition (raw boundary): `d` is the live mjData of the model
// this writer was built on, driven by the single owning Stepper — widths
// are validated, model/data pairing is not derivable from them.
void RiderForcesWriter::compute_into(const mjData *d,
                                     std::span<double> out) const {
    if (d == nullptr)
        throw std::invalid_argument(
            "rider compute needs a live mjData");
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()))
        throw std::invalid_argument("rider output width");
    accumulate_validated(d, out);
}

// The warm-core twin of compute_into: the same precondition gates report
// CoreStatus::invalid_input instead of throwing, and residual kernel
// rejections (model-table guards, non-finite derived values) are caught
// and mapped so no exception escapes — noexcept is proved by the
// catch-all. The kernel's own throw construction stays on the failure
// path only; a valid tick allocates nothing.
CoreStatus RiderForcesWriter::try_compute_into(const mjData *d,
                                               std::span<double> out) const noexcept {
    if (d == nullptr || out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()))
        return CoreStatus::invalid_input;
    try {
        accumulate_validated(d, out);
    } catch (const std::bad_alloc &) {
        return CoreStatus::engine_failure;
    } catch (const engine::EngineFailure &) {
        // EngineFailure derives std::exception — it must be caught before
        // the generic mapping classifies an engine error as bad input.
        return CoreStatus::engine_failure;
    } catch (const std::exception &) {
        return CoreStatus::invalid_input;
    } catch (...) {
        return CoreStatus::engine_failure;
    }
    return CoreStatus::ok;
}

std::vector<double> RiderForcesWriter::qfrc(const mjData *d) const {
    compute_into(d, out_);
    return out_;
}

// Extracted unchanged from the original qfrc() body — including the
// model-table require_id guards and the derived() overflow gates, whose
// exception types are part of the observable contract.
void RiderForcesWriter::accumulate_validated(const mjData *d,
                                             std::span<double> out) const {
    // rider_forces.py:151-154 — apply(): reads qpos/qvel only; assign, not
    // accumulate.
    const std::span<const mjtNum> qpos =
            model_access::readonly_buffer(d->qpos, nq_);
    const std::span<const mjtNum> qvel =
            model_access::readonly_buffer(d->qvel, nv_);
    std::ranges::fill(out, 0.0);
    for (std::size_t i = 0; i < paths_.size(); ++i) {
        const Path &p = paths_[i];
        Telemetry &t = last_[i];
        model_access::require_id(p.qposadr, nq_, "rider slide qpos address");
        model_access::require_id(p.dofadr, nv_, "rider slide dof address");
        const double q = qpos[static_cast<std::size_t>(p.qposadr)];
        const double qd = qvel[static_cast<std::size_t>(p.dofadr)];
        // rider_forces.py:64-76 — _JointPath.compute.
        const double deflection = validation::derived(p.preload_deflection_m + p.offset_m - q, "RiderForcesWriter.deflection");
        if (p.unilateral) {
            if (deflection <= 0.0) {
                t.gap_m = -deflection;
                t.force_n = 0.0;
            } else {
                t.gap_m = 0.0;
                // The damper cannot pull the body back onto the saddle or
                // the pedal either (rider_forces.py:71-72).
                t.force_n = std::max(
                    0.0, validation::derived(p.stiffness_n_m * deflection - p.damping_ns_m * qd, "RiderForcesWriter.force"));
            }
        } else {
            t.gap_m = 0.0;
            t.force_n =
                    validation::derived(p.stiffness_n_m * deflection - p.damping_ns_m * qd, "RiderForcesWriter.force");
        }
        out[static_cast<std::size_t>(p.dofadr)] = t.force_n;
    }
}
