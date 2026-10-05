// writers/rider_forces.cpp — expression-for-expression port of
// rider_forces.py's _JointPath.compute + RiderForceApplier.apply; comments
// cite file:line at the time of porting. Python max(a, b) maps to std::max —
// same two-argument keep-first-on-tie semantics, including the NaN cases.
#include "rider_forces.hpp"

#include <algorithm>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

// rider_forces.py:109-115 — _resolve_slide, same checks and order.
std::pair<int, int> resolve_slide(const mjModel* m,
                                  const std::string& joint_name) {
    const int jid = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
    if (jid < 0)
        throw std::invalid_argument(
            "model has no joint '" + joint_name +
            "'; the seated rider's forces cannot be applied");
    // views::counted everywhere — -Wunsafe-buffer-usage rejects raw
    // indexing into mjModel pointer fields (same pattern as suspension.cpp).
    const std::span<const int> jnt_type =
        std::views::counted(m->jnt_type, m->njnt);
    const std::span<const mjtBool> limited =
        std::views::counted(m->jnt_limited, m->njnt);
    if (jnt_type[static_cast<std::size_t>(jid)] != mjJNT_SLIDE ||
        limited[static_cast<std::size_t>(jid)])
        throw std::invalid_argument(
            "joint '" + joint_name +
            "' must be an unlimited slide for the rider force path");
    const std::span<const int> qposadr =
        std::views::counted(m->jnt_qposadr, m->njnt);
    const std::span<const int> dofadr =
        std::views::counted(m->jnt_dofadr, m->njnt);
    return {qposadr[static_cast<std::size_t>(jid)],
            dofadr[static_cast<std::size_t>(jid)]};
}

}  // namespace

RiderForcesWriter::RiderForcesWriter(const mjModel* m,
                                     nativecfg::RiderForcesConfig config)
    : nq_(m->nq), nv_(m->nv) {
    // rider_forces.py:98-106 — one _JointPath per pose body, resolved in
    // order; an empty paths list is the inert pose=None case (qfrc stays
    // all-zero, mirroring apply()'s no-op on a zeroed buffer).
    paths_.reserve(config.paths.size());
    last_.reserve(config.paths.size());
    for (const nativecfg::RiderPathConfig& p : config.paths) {
        const auto [qposadr, dofadr] = resolve_slide(m, p.joint);
        paths_.push_back({qposadr, dofadr, p.stiffness_n_m, p.damping_ns_m,
                          p.preload_deflection_m, p.offset_m, p.unilateral});
        last_.push_back({0.0, 0.0});
    }
}

std::vector<double> RiderForcesWriter::qfrc(const mjData* d) const {
    // rider_forces.py:151-154 — apply(): reads qpos/qvel only; assign, not
    // accumulate.
    const std::span<const mjtNum> qpos = std::views::counted(d->qpos, nq_);
    const std::span<const mjtNum> qvel = std::views::counted(d->qvel, nv_);
    std::vector<double> out(static_cast<std::size_t>(nv_), 0.0);
    for (std::size_t i = 0; i < paths_.size(); ++i) {
        const Path& p = paths_[i];
        Telemetry& t = last_[i];
        const double q = qpos[static_cast<std::size_t>(p.qposadr)];
        const double qd = qvel[static_cast<std::size_t>(p.dofadr)];
        // rider_forces.py:64-76 — _JointPath.compute.
        const double deflection = p.preload_deflection_m + p.offset_m - q;
        if (p.unilateral) {
            if (deflection <= 0.0) {
                t.gap_m = -deflection;
                t.force_n = 0.0;
            } else {
                t.gap_m = 0.0;
                // The damper cannot pull the body back onto the saddle or
                // the pedal either (rider_forces.py:71-72).
                t.force_n = std::max(
                    0.0, p.stiffness_n_m * deflection - p.damping_ns_m * qd);
            }
        } else {
            t.gap_m = 0.0;
            t.force_n =
                p.stiffness_n_m * deflection - p.damping_ns_m * qd;
        }
        out[static_cast<std::size_t>(p.dofadr)] = t.force_n;
    }
    return out;
}
