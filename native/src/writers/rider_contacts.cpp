// writers/rider_contacts.cpp — expression-for-expression port of
// RiderContactApplier (src/bike_sim/sim/ride/rider_contacts.py), writer core.
//
// Engine-call policy: mj_jac, mj_mulJacTVec and mj_name2id are invoked
// directly — like equality_reactions.cpp they have no fatal-error path in
// the engine (pure arithmetic / name-table lookup). mj_kinematics and
// mj_comPos CAN run the fatal machinery (they walk every constraint and
// flexvert path), so the two set_enabled refreshes go through
// engine::invoke — a fatal MuJoCo error surfaces as engine::EngineFailure
// and the owning Stepper's mutate() boundary poisons the context.
//
// numpy/BLAS routing (the bitwise contract):
//   * `a @ b`, `float(x @ y)`, `norm()` → blas::ddot (np.linalg.norm is
//     sqrt(x.dot(x)) for 1-D input — the same ddot + libm sqrt);
//   * `jrel @ qvel`, `R @ v`, `jrel.T @ f` → blas::dgemv;
//   * `math.hypot(*v)` → rider::hypot3 (the CPython port);
//   * `x ** 2`/`x ** y` → pyfloat::pow (CPython float_pow → libm pow);
//   * `np.cross` → the literal component products below;
//   * `min(a,b)`/`max(a,b)` → std::min/std::max — the same two-argument
//     keep-first-on-tie (and NaN) semantics Python's builtins have.
#include "rider_contacts.hpp"

#include "../cblas_abi.hpp"
#include "../contact/laws.hpp"
#include "../diag.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../rider/support_geometry.hpp"
#include "pyfloat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
    using rider::Vec3;
    using rider::Mat3;

    // Element readers over the checked engine spans: xpos/xmat/site_xpos/
    // geom_xpos/geom_xmat are row-major spatial triples (or 3x3 matrices).
    [[nodiscard]] Vec3 vec3_off(std::span<const double> v, std::size_t base) {
        return {v[base], v[base + 1], v[base + 2]};
    }

    [[nodiscard]] Vec3 vec3_at(std::span<const double> v, int id) {
        return vec3_off(v, 3 * static_cast<std::size_t>(id));
    }

    // Returned by value deliberately — same stance as
    // support_geometry.cpp's BoxFace returns; the flag's by-value-parameter
    // half stays enabled elsewhere.
    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
    [[nodiscard]] Mat3 mat3_at(std::span<const double> v, int id) {
        const std::size_t base = 9 * static_cast<std::size_t>(id);
        Mat3 out{};
        for (std::size_t i = 0; i < out.size(); ++i) out[i] = v[base + i];
        return out;
    }
    NATIVE_DIAG_POP

    // `(-x)` elementwise — flips the sign bit of zeros like numpy's unary
    // minus, so (-np.zeros(3)).tolist() parity is -0.0, not +0.0.
    [[nodiscard]] Vec3 negate3(const Vec3 &v) {
        return {-v[0], -v[1], -v[2]};
    }

    // np.cross — the three literal multiply-subtract expressions.
    [[nodiscard]] Vec3 cross_np(const Vec3 &a, const Vec3 &b) {
        return {a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }

    // np.linalg.norm(x) for a 1-D float64 vector: sqrt(x.dot(x)).
    [[nodiscard]] double norm3(const Vec3 &v) {
        return std::sqrt(blas::ddot(3, v.data(), 1, v.data(), 1));
    }

    // engine::invoke operation frames for the two fatal-capable engine
    // calls set_enabled performs — mj_kinematics and mj_comPos.
    struct KinematicsContext {
        const mjModel *model;
        mjData *data;
    };

    // NOLINTNEXTLINE(misc-const-correctness) engine::Operation's ABI fixes the void* signature
    void kinematics_operation(void *raw) noexcept {
        const auto *ctx = static_cast<const KinematicsContext *>(raw);
        mj_kinematics(ctx->model, ctx->data);
    }

    // NOLINTNEXTLINE(misc-const-correctness) engine::Operation's ABI fixes the void* signature
    void compos_operation(void *raw) noexcept {
        const auto *ctx = static_cast<const KinematicsContext *>(raw);
        mj_comPos(ctx->model, ctx->data);
    }

    // nv as a lapack_int for the ctor's workspace bound — 0 on a null
    // model (equality_'s member-init throws before lstsq_ is built).
    [[nodiscard]] rider::lapack_int
    lstsq_rows_bound(const mjModel *model) noexcept {
        return model != nullptr ? model->nv : rider::lapack_int{0};
    }
} // namespace

namespace writers {

    RiderContactWriter::RiderContactWriter(
        const mjModel *model, const rider::RiderContactsConfig &config)
        : model_(model), cfg_(config), equality_(model),
          // The (nv, max(nv,6), 1) bound admits every (k,6) spatial
          // validation solve the attachment measurement runs — the same
          // bound Stepper::rider_attachment_lstsq uses. equality_ throws
          // on a null model before this expression ever dereferences it;
          // the helper keeps the mjtSize read out of a casted ternary
          // (-Wuseless-cast under the second frontend).
          lstsq_(lstsq_rows_bound(model),
                 std::max<rider::lapack_int>(lstsq_rows_bound(model), 6),
                 1) {
        if (model_ == nullptr)
            throw std::invalid_argument("rider contacts need a live model");
        if (model_->nv < 0)
            throw std::invalid_argument(
                "rider contacts need a nonnegative model width");
        // ArticulatedConfig.__post_init__ domain — same rejection the
        // Python ctor's config already carries.
        rider::validate(cfg_);
        // __init__ (rider_contacts.py:60-81): the literal resolution order
        // is part of the error contract — first missing name wins.
        frame_ = model_access::resolve_id(model_, mjOBJ_BODY, "frame");
        steer_ = model_access::resolve_id(model_, mjOBJ_BODY, "steer");
        pelvis_ = model_access::resolve_id(model_, mjOBJ_BODY, "rider_pelvis");
        forearm_[0] =
            model_access::resolve_id(model_, mjOBJ_BODY, "rider_forearm_left");
        forearm_[1] =
            model_access::resolve_id(model_, mjOBJ_BODY, "rider_forearm_right");
        shoulder_[0] =
            model_access::resolve_id(model_, mjOBJ_BODY, "rider_upper_arm_left");
        shoulder_[1] = model_access::resolve_id(model_, mjOBJ_BODY,
                                                "rider_upper_arm_right");
        grip_site_[0] =
            model_access::resolve_id(model_, mjOBJ_SITE, "site_rider_grip_left");
        grip_site_[1] = model_access::resolve_id(model_, mjOBJ_SITE,
                                                 "site_rider_grip_right");
        supports_[0] = {
            .body = pelvis_,
            .site = model_access::resolve_id(model_, mjOBJ_SITE,
                                             "site_rider_saddle"),
            .bike = frame_,
            .geom = model_access::resolve_id(model_, mjOBJ_GEOM,
                                             "geom_saddle")};
        supports_[1] = {
            .body = model_access::resolve_id(model_, mjOBJ_BODY,
                                             "rider_foot_front"),
            .site = model_access::resolve_id(model_, mjOBJ_SITE,
                                             "site_rider_sole_front"),
            .bike = model_access::resolve_id(model_, mjOBJ_BODY,
                                             "pedal_front"),
            .geom = model_access::resolve_id(model_, mjOBJ_GEOM,
                                             "geom_pedal_front")};
        supports_[2] = {
            .body = model_access::resolve_id(model_, mjOBJ_BODY,
                                             "rider_foot_rear"),
            .site = model_access::resolve_id(model_, mjOBJ_SITE,
                                             "site_rider_sole_rear"),
            .bike = model_access::resolve_id(model_, mjOBJ_BODY,
                                             "pedal_rear"),
            .geom = model_access::resolve_id(model_, mjOBJ_GEOM,
                                             "geom_pedal_rear")};
        const int crank =
            model_access::resolve_id(model_, mjOBJ_JOINT, "crank_spin");
        const std::span<const int> dofadr = model_access::readonly_buffer(
            model_->jnt_dofadr, model_->njnt, "jnt_dofadr");
        crank_dof_ = dofadr[static_cast<std::size_t>(crank)];
        jac_a_.assign(3 * static_cast<std::size_t>(model_->nv), 0.);
        jac_b_.assign(3 * static_cast<std::size_t>(model_->nv), 0.);
        qfrc_.assign(static_cast<std::size_t>(model_->nv), 0.);
        measure_qfrc_.assign(static_cast<std::size_t>(model_->nv), 0.);
        // Attachment discriminators (lines 84-95) and the conditional
        // equality resolution each link class performs.
        spindle_pedals_ =
            cfg_.pedal_attachment == rider::PedalAttachment::spindle;
        linked_pedals_ = spindle_pedals_ ||
            cfg_.pedal_attachment == rider::PedalAttachment::weld;
        welded_saddle_ =
            cfg_.saddle_attachment == rider::SaddleAttachment::weld;
        pinned_saddle_ =
            cfg_.saddle_attachment == rider::SaddleAttachment::pin;
        linked_saddle_ = welded_saddle_ || pinned_saddle_;
        welded_grip_ =
            cfg_.grip_attachment == rider::GripAttachment::connect;
        if (linked_pedals_) {
            // PedalWelds/SpindlePins resolve front then rear.
            const char *const prefix =
                spindle_pedals_ ? "connect_foot_" : "weld_foot_";
            pedal_eq_[0] = model_access::resolve_id(
                model_, mjOBJ_EQUALITY,
                (std::string(prefix) + "front").c_str());
            pedal_eq_[1] = model_access::resolve_id(
                model_, mjOBJ_EQUALITY,
                (std::string(prefix) + "rear").c_str());
        }
        if (linked_saddle_)
            saddle_eq_ = model_access::resolve_id(
                model_, mjOBJ_EQUALITY,
                welded_saddle_ ? "weld_saddle" : "connect_saddle");
        if (welded_grip_) {
            grip_eq_[0] = model_access::resolve_id(model_, mjOBJ_EQUALITY,
                                                   "connect_grip_left");
            grip_eq_[1] = model_access::resolve_id(model_, mjOBJ_EQUALITY,
                                                   "connect_grip_right");
        }
        // reset(model, None): the member defaults already encode the
        // fresh-contact state — reset() exists to re-run it against data.
    }

    void RiderContactWriter::reset(mjData *data) {
        // rider_contacts.py:98-123 — the field ordering matters on a throw:
        // an out-of-planar-model rejection leaves the early fields reset
        // and the energy/diagnostic fields untouched, like the oracle.
        seen_ = data;
        state_.enabled = {true, true, true, true};
        state_.supports = {};
        state_.grip_xi_local = {};
        state_.grip_anchor_local = {std::nullopt, std::nullopt};
        if (data != nullptr) {
            const std::array<int, kSupportCount> geoms{
                supports_[0].geom, supports_[1].geom, supports_[2].geom};
            rider::validate_planar_support_model(model_, data, geoms);
            const std::span<const double> xmat =
                model_access::readonly_buffer(
                    data->xmat, 9 * model_->nbody, "xmat");
            const rider::Mat3 steer_R = mat3_at(xmat, steer_);
            for (std::size_t side = 0; side < kSideCount; ++side) {
                if (welded_grip_) {
                    // model.eq_data[eq_id, 3:6] — the body2-local connect
                    // anchor, copied like the oracle's .copy().
                    const std::span<const double> eq_data =
                        model_access::readonly_buffer(
                            model_->eq_data, mjNEQDATA * model_->neq,
                            "eq_data");
                    state_.grip_anchor_local[side] = vec3_off(
                        eq_data,
                        static_cast<std::size_t>(mjNEQDATA) *
                                static_cast<std::size_t>(grip_eq_[side]) +
                            3);
                } else {
                    const std::span<const double> site_xpos =
                        model_access::readonly_buffer(
                            data->site_xpos, 3 * model_->nsite, "site_xpos");
                    const std::span<const double> xpos =
                        model_access::readonly_buffer(
                            data->xpos, 3 * model_->nbody, "xpos");
                    state_.grip_anchor_local[side] = rider::matvec(
                        steer_R,
                        rider::subtract(vec3_at(site_xpos, grip_site_[side]),
                                        vec3_at(xpos, steer_)),
                        blas::Transpose::yes);
                }
            }
        }
        state_.elastic_energy_j = 0.;
        state_.loss_step_j = 0.;
        state_.radial_dissipation_power_w = 0.;
        state_.delivered_crank_torque_nm = 0.;
        state_.diagnostics = RiderContactsDiagnostics{};
        state_.last_time_s.reset();
        state_.pending_release_loss_j = 0.;
        state_.settled_welds.reset();
        state_.attachment_samples.clear();
        state_.attachment_errors.clear();
    }

    void RiderContactWriter::initialize_settled_state(mjData *data) {
        if (state_.last_time_s.has_value() || data->time != 0.)
            throw std::invalid_argument(
                "settled contact initialization requires a fresh clock");
        compute_qfrc_into(data, static_cast<double>(model_->opt.timestep),
                          true, true, qfrc_);
        state_.loss_step_j = 0.;
        state_.radial_dissipation_power_w = 0.;
        restart_clock();
    }

    bool RiderContactWriter::set_enabled(std::string_view name, bool enabled) {
        const auto it = std::ranges::find(kContactNames, name);
        if (it == kContactNames.end())
            throw std::invalid_argument("invalid rider contact enable request");
        const std::size_t index =
            static_cast<std::size_t>(it - kContactNames.begin());
        // rider_contacts.py:128-131 — a weld or pin cannot be released.
        if (!enabled &&
            ((linked_pedals_ && (index == 1 || index == 2)) ||
             (linked_saddle_ && index == 0) ||
             (welded_grip_ && index == kGripIndex)))
            return true;
        if (!enabled && state_.enabled[index]) {
            if (index == kGripIndex) {
                for (std::size_t side = 0; side < kSideCount; ++side) {
                    const Vec3 &xi = state_.grip_xi_local[side];
                    state_.pending_release_loss_j +=
                        .5 * cfg_.grip_k_n_m * rider::dot(xi, xi);
                    state_.grip_xi_local[side] = {};
                }
            } else {
                for (std::size_t i = 0; i < kPadsPerSupport; ++i) {
                    RiderContactSupportState &s =
                        state_.supports[kPadsPerSupport * index + i];
                    state_.pending_release_loss_j +=
                        .25 * cfg_.support_tangent_k_n_m *
                        pyfloat::pow(s.xi, 2.);
                    s = {};
                }
                if (seen_ != nullptr) {
                    // Release can be requested after integration; the pad
                    // gap belongs to the incoming pose — refresh positions
                    // only, do not run a new force solve.
                    refresh_kinematics(seen_);
                    for (std::size_t i = 0; i < kPadsPerSupport; ++i) {
                        PadEval c{};
                        pad_eval(index, i, seen_, c);
                        if (c.inside)
                            state_.pending_release_loss_j +=
                                .25 * cfg_.support_k_n_m *
                                pyfloat::pow(std::max(-c.gap, 0.), 2.);
                    }
                }
            }
        }
        if (index == kGripIndex && enabled && !state_.enabled[kGripIndex]) {
            if (seen_ == nullptr || !state_.grip_anchor_local[0] ||
                !state_.grip_anchor_local[1])
                return false;
            refresh_kinematics_com(seen_);
            const std::span<const double> xmat =
                model_access::readonly_buffer(seen_->xmat,
                                              9 * model_->nbody, "xmat");
            const std::span<const double> xpos =
                model_access::readonly_buffer(seen_->xpos,
                                              3 * model_->nbody, "xpos");
            const std::span<const double> site_xpos =
                model_access::readonly_buffer(seen_->site_xpos,
                                              3 * model_->nsite, "site_xpos");
            const std::span<const double> qvel =
                model_access::readonly_buffer(seen_->qvel, model_->nv, "qvel");
            const Mat3 R = mat3_at(xmat, steer_);
            const Vec3 steer_pos = vec3_at(xpos, steer_);
            const int nv = static_cast<int>(model_->nv);
            for (std::size_t side = 0; side < kSideCount; ++side) {
                // Unreachable once the gate above passed — rechecked so the
                // capture still fails closed if a future edit drops it.
                const auto &anchor = state_.grip_anchor_local[side];
                if (!anchor.has_value()) return false;
                const Vec3 grip =
                    rider::add(steer_pos, rider::matvec(R, *anchor));
                const Vec3 diff =
                    rider::subtract(vec3_at(site_xpos, grip_site_[side]), grip);
                const double gap = norm3(diff);
                const auto &jrel = relative_jacobian(seen_, forearm_[side],
                                                     steer_, grip);
                Vec3 v{};
                blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3, nv,
                            1., jrel.data(), nv, qvel.data(), 1, 0., v.data(),
                            1);
                const double speed = norm3(v);
                if (gap > cfg_.grip_capture_distance_m ||
                    speed > cfg_.grip_capture_speed_mps)
                    return false;
            }
            for (std::size_t side = 0; side < kSideCount; ++side)
                state_.grip_xi_local[side] = {};
        }
        state_.enabled[index] = enabled;
        return state_.enabled[index];
    }

    void RiderContactWriter::release_all() {
        for (std::size_t i = 0; i < kContactCount; ++i)
            static_cast<void>(set_enabled(kContactNames[i], false));
    }

    void RiderContactWriter::compute_qfrc_into(mjData *data, double dt,
                                               bool advance, bool detailed,
                                               std::span<double> out) {
        // scalar(dt, 'rider contact dt', positive=True) — the domain half;
        // the binding already rejected non-real input like scalar()'s
        // type gate.
        if (!std::isfinite(dt) || dt <= 0.)
            throw std::invalid_argument("invalid rider contact dt");
        if (out.size() != static_cast<std::size_t>(model_->nv))
            throw std::invalid_argument(
                "rider contact qfrc destination must be nv elements");
        if (!advance) {
            // compute_qfrc(advance=False): a state copy with a cleared
            // clock runs the full advancing path; only the probe view is
            // published (rider_contacts.py:427-437).
            RiderContactsState probe = state_;
            probe.last_time_s.reset();
            evaluate(data, dt, detailed, probe, out);
            probe_ = RiderContactsProbe{.diagnostics =
                                            std::move(probe.diagnostics),
                                        .enabled = probe.enabled,
                                        .delivered_crank_torque_nm =
                                            probe.delivered_crank_torque_nm};
            return;
        }
        seen_ = data;
        evaluate(data, dt, detailed, state_, out);
    }

    // The whole oracle compute_qfrc evaluation lives in one frame — the
    // staged diagnostics tree, per-pad scratch and both grip branches —
    // so sanitizer redzones push it past the 8 KiB frame budget. The
    // function is depth-bounded (no recursion) and the frame is fixed;
    // suppressing locally like the staged-dict binding paths do.
    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wframe-larger-than")
    void RiderContactWriter::evaluate(mjData *data, double dt, bool detailed,
                                      RiderContactsState &w,
                                      std::span<double> out) {
        const double time = data->time;
        if (w.last_time_s.has_value() && time <= *w.last_time_s)
            throw std::invalid_argument(
                "rider contact state advances only once per timestamp");
        if (!w.grip_anchor_local[0] || !w.grip_anchor_local[1])
            throw std::runtime_error(
                "initialize rider contact anchors before evaluation");
        // The oracle's single equality_rows() map is an allocation saving:
        // EqualityReactions re-derives the same ascending row membership
        // per call, and efc contents do not change mid-evaluation.
        std::ranges::fill(out, 0.);
        const int nv = static_cast<int>(model_->nv);
        double energy = 0.;
        double loss = w.pending_release_loss_j;
        double radial_power = 0.;
        double delivered = 0.;
        std::array<RiderContactSupportState, kPadStateCount> new_states{};
        RiderContactsDiagnostics diagnostics{};
        diagnostics.evaluated = true;
        const std::span<const double> xpos = model_access::readonly_buffer(
            data->xpos, 3 * model_->nbody, "xpos");
        const std::span<const double> xmat = model_access::readonly_buffer(
            data->xmat, 9 * model_->nbody, "xmat");
        const std::span<const double> site_xpos =
            model_access::readonly_buffer(data->site_xpos, 3 * model_->nsite,
                                          "site_xpos");
        const std::span<const double> qvel = model_access::readonly_buffer(
            data->qvel, model_->nv, "qvel");
        for (std::size_t si = 0; si < kSupportCount; ++si) {
            const SupportRef &ref = supports_[si];
            const bool linked =
                (si == 0 && linked_saddle_) || (si > 0 && linked_pedals_);
            if (linked) {
                // Welded/pinned support (lines 458-502): the settled latch
                // supplies force/load/flags — null until settle_welds
                // latches it — and the live translation residual fills gap_m.
                const auto [force_on_rider, normal_load] =
                    settled_force(w, kSupportNames[si]);
                const RiderSettledEntry *const settled =
                    settled_entry(w, kSupportNames[si]);
                RiderContactSupportDiag &d = diagnostics.supports[si];
                d.enabled = true;
                d.in_platform = true;
                d.normal_load_n = normal_load;
                d.welded = true;
                d.would_separate = settled != nullptr && settled->would_separate;
                d.would_slip = settled != nullptr && settled->would_slip;
                d.tangent_n = settled != nullptr ? settled->tangent_n : 0.;
                d.gap_m = equality_.translation_residual(
                    data, si == 0 ? saddle_eq_ : pedal_eq_[si - 1]);
                d.vertical_force_on_rider_n = force_on_rider[2];
                d.detailed = detailed;
                if (detailed) {
                    d.force_on_rider_n = force_on_rider;
                    d.force_on_bike_n = negate3(force_on_rider);
                }
                new_states[kPadsPerSupport * si] = {};
                new_states[kPadsPerSupport * si + 1] = {};
                continue;
            }
            // Unilateral two-pad support (lines 503-556).
            double group_normal = 0., group_tangent = 0., group_radial = 0.;
            double group_shear = 0., group_power = 0., group_vertical = 0.;
            Vec3 group_force{}, group_moment{};
            bool in_platform = false;
            double minimum_gap = std::numeric_limits<double>::infinity();
            std::vector<RiderContactPatchDiag> patches;
            for (std::size_t pad = 0; pad < kPadsPerSupport; ++pad) {
                PadEval c{};
                pad_eval(si, pad, data, c);
                if (std::abs(c.normal[1]) > 1e-9 ||
                    std::abs(c.tangent[1]) > 1e-9)
                    throw std::invalid_argument(
                        "rider support surface is outside the planar model");
                const auto &jrel =
                    relative_jacobian(data, ref.body, ref.bike, c.point);
                Vec3 u{};
                blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3,
                            nv, 1., jrel.data(), nv, qvel.data(), 1, 0.,
                            u.data(), 1);
                const double penetration = -c.gap;
                const double damping =
                    si == 0 ? cfg_.support_c_ns_m : cfg_.pedal_c_ns_m;
                const double k = cfg_.support_k_n_m / 2;
                const double c_damp = damping / 2;
                const double kx = cfg_.support_tangent_k_n_m / 2;
                auto [normal, radial_energy] = contactlaw::normal_contact(
                    penetration,
                    -blas::ddot(3, u.data(), 1, c.normal.data(), 1), k,
                    c_damp);
                const std::size_t key = kPadsPerSupport * si + pad;
                const RiderContactSupportState &old = w.supports[key];
                double xi = old.xi;
                double transport_loss = 0.;
                if (old.tangent.has_value()) {
                    const double alignment = blas::ddot(
                        3, old.tangent->data(), 1, c.tangent.data(), 1);
                    // An opposite face is a new material contact, not a
                    // shear spring carried through the pedal's core.
                    const double transported =
                        alignment >= 0. ? xi * alignment : 0.;
                    transport_loss =
                        .5 * kx * (xi * xi - transported * transported);
                    xi = transported;
                }
                if (!w.enabled[si] || !c.inside) {
                    normal = 0.;
                    radial_energy = 0.;
                }
                auto [new_xi, friction, brush_loss] = contactlaw::brush_step(
                    xi, blas::ddot(3, u.data(), 1, c.tangent.data(), 1), 0.,
                    normal, kx, cfg_.support_mu, cfg_.support_length_m, dt);
                const Vec3 f =
                    rider::add(rider::multiply(c.normal, normal),
                               rider::multiply(c.tangent, friction));
                blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3,
                            nv, 1., jrel.data(), nv, f.data(), 1, 1.,
                            out.data(), 1);
                if (si > 0)
                    delivered += blas::ddot(
                        3, f.data(), 1,
                        std::span{jrel}
                            .subspan(static_cast<std::size_t>(crank_dof_))
                            .data(),
                        nv);
                const double shear = .5 * kx * pyfloat::pow(new_xi, 2.);
                energy += radial_energy + shear;
                loss += std::max(transport_loss, 0.) + brush_loss;
                const double radial_loss =
                    (normal - k * std::max(penetration, 0.)) *
                    (-blas::ddot(3, u.data(), 1, c.normal.data(), 1));
                if (w.enabled[si] && c.inside && penetration > 0.)
                    radial_power += std::max(radial_loss, 0.);
                group_normal += normal;
                group_vertical += f[2];
                in_platform = in_platform || c.inside;
                minimum_gap = std::min(minimum_gap, c.gap);
                if (detailed) {
                    group_tangent += friction;
                    group_force = rider::add(group_force, f);
                    group_moment = rider::add(
                        group_moment,
                        cross_np(rider::subtract(c.point,
                                                 vec3_at(xpos, ref.body)),
                                 f));
                    group_radial += radial_energy;
                    group_shear += shear;
                    group_power +=
                        blas::ddot(3, f.data(), 1, u.data(), 1);
                    patches.push_back({.in_platform = c.inside,
                                       .normal_load_n = normal,
                                       .gap_m = c.gap,
                                       .point_m = c.point,
                                       .force_on_rider_n = f,
                                       .radial_energy_j = radial_energy,
                                       .shear_energy_j = shear});
                }
                new_states[key] = {.xi = new_xi, .tangent = c.tangent};
            }
            RiderContactSupportDiag &d = diagnostics.supports[si];
            d.enabled = w.enabled[si];
            d.in_platform = in_platform;
            d.normal_load_n = group_normal;
            d.gap_m = minimum_gap;
            d.vertical_force_on_rider_n = group_vertical;
            d.welded = false;
            d.detailed = detailed;
            if (detailed) {
                d.tangent_force_scalar = group_tangent;
                d.patches = std::move(patches);
                d.force_on_rider_n = group_force;
                d.force_on_bike_n = negate3(group_force);
                d.moment_nm = group_moment;
                d.radial_energy_j = group_radial;
                d.shear_energy_j = group_shear;
                d.relative_power_w = group_power;
            }
        }
        const Mat3 R = mat3_at(xmat, steer_);
        const Vec3 steer_pos = vec3_at(xpos, steer_);
        if (welded_grip_) {
            // Connect-equality grip (lines 557-600): each hand's reaction
            // comes from the latch; residuals come from the live arena.
            Vec3 total_force{};
            RiderContactGripDiag &aggregate = diagnostics.grip;
            aggregate.welded = true;
            aggregate.enabled = true;
            aggregate.reachable = true;
            for (std::size_t side = 0; side < kSideCount; ++side) {
                const Vec3 grip = rider::add(
                    steer_pos, rider::matvec(R, *w.grip_anchor_local[side]));
                RiderContactGripSideDiag &d = diagnostics.grip_sides[side];
                d.welded = true;
                d.enabled = true;
                d.reachable = true;
                d.hand_gap_m =
                    equality_.translation_residual(data, grip_eq_[side]);
                d.extended = detailed;
                if (detailed) {
                    const auto [force, ignored_normal] = settled_force(
                        w, side == 0 ? "grip_left" : "grip_right");
                    static_cast<void>(ignored_normal);
                    total_force = rider::add(total_force, force);
                    d.overloaded = false;
                    d.trial_pair_force_n = norm3(force);
                    d.pair_force_limit_n = cfg_.grip_pair_force_limit_n;
                    d.release_loss_j = 0.;
                    d.shoulder_distance_m = rider::hypot3(rider::subtract(
                        grip, vec3_at(xpos, shoulder_[side])));
                    d.arm_reach_m = cfg_.arm_reach_m;
                    d.point_m = grip;
                    d.force_on_rider_n = force;
                    d.force_on_bike_n = negate3(force);
                    d.elastic_energy_j = 0.;
                }
            }
            aggregate.hand_gap_m =
                std::max(diagnostics.grip_sides[0].hand_gap_m,
                         diagnostics.grip_sides[1].hand_gap_m);
            aggregate.extended = detailed;
            if (detailed) {
                aggregate.overloaded = false;
                aggregate.trial_pair_force_n = norm3(total_force);
                aggregate.pair_force_limit_n = cfg_.grip_pair_force_limit_n;
                aggregate.release_loss_j = 0.;
                aggregate.shoulder_distance_m =
                    std::max(diagnostics.grip_sides[0].shoulder_distance_m,
                             diagnostics.grip_sides[1].shoulder_distance_m);
                aggregate.arm_reach_m = cfg_.arm_reach_m;
                aggregate.force_on_rider_n = total_force;
                aggregate.force_on_bike_n = negate3(total_force);
                aggregate.elastic_energy_j = 0.;
            }
        } else {
            // Spring grip (lines 601-674): two hands, one 'grip' flag; the
            // pair limit is split evenly between the two springs.
            Vec3 total_force{};
            double pair_energy = 0., pair_loss = 0.;
            bool overloaded = false;
            struct SideEval {
                bool reachable = false, active = false;
                Vec3 grip{}, hand_gap{}, shoulder_gap{}, force{};
                double trial_norm = 0., grip_energy = 0., grip_loss = 0.;
            };
            std::array<SideEval, kSideCount> per_side;
            for (std::size_t side = 0; side < kSideCount; ++side) {
                SideEval &s = per_side[side];
                s.grip = rider::add(
                    steer_pos, rider::matvec(R, *w.grip_anchor_local[side]));
                const Vec3 hand = vec3_at(site_xpos, grip_site_[side]);
                s.shoulder_gap =
                    rider::subtract(s.grip, vec3_at(xpos, shoulder_[side]));
                s.hand_gap = rider::subtract(hand, s.grip);
                s.reachable =
                    rider::hypot3(s.shoulder_gap) <= cfg_.arm_reach_m + 1e-6 &&
                    rider::hypot3(s.hand_gap) <= cfg_.grip_release_distance_m;
                s.active = w.enabled[kGripIndex] && s.reachable;
                const Vec3 old = w.grip_xi_local[side];
                Vec3 fresh{};
                if (s.active) {
                    const auto &jrel = relative_jacobian(
                        data, forearm_[side], steer_, s.grip);
                    Vec3 relative{};
                    blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                                3, nv, 1., jrel.data(), nv, qvel.data(), 1,
                                0., relative.data(), 1);
                    const rider::GripStep step = rider::grip_step(
                        old, rider::matvec(R, relative, blas::Transpose::yes),
                        cfg_.grip_k_n_m, cfg_.grip_c_ns_m, dt);
                    fresh = step.xi;
                    s.force = rider::matvec(R, step.force);
                    s.grip_energy = step.energy_j;
                    s.grip_loss = step.loss_j;
                    s.trial_norm = norm3(s.force);
                    if (cfg_.grip_pair_force_limit_n.has_value()) {
                        const rider::GripRelease release =
                            rider::release_if_overloaded(
                                s.force,
                                .5 * cfg_.grip_k_n_m * rider::dot(old, old),
                                *cfg_.grip_pair_force_limit_n / 2.);
                        s.force = release.force;
                        if (release.overloaded) {
                            overloaded = true;
                            fresh = {};
                            s.grip_energy = 0.;
                            s.force = {};
                            s.grip_loss = release.loss_j;
                        }
                    }
                    blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                                3, nv, 1., jrel.data(), nv, s.force.data(), 1,
                                1., out.data(), 1);
                } else {
                    s.grip_loss = .5 * cfg_.grip_k_n_m * rider::dot(old, old);
                }
                if (!s.reachable) w.enabled[kGripIndex] = false;
                w.grip_xi_local[side] = fresh;
                pair_energy += s.grip_energy;
                pair_loss += s.grip_loss;
                total_force = rider::add(total_force, s.force);
            }
            if (overloaded) w.enabled[kGripIndex] = false;
            energy += pair_energy;
            loss += pair_loss;
            for (std::size_t side = 0; side < kSideCount; ++side) {
                const SideEval &s = per_side[side];
                RiderContactGripSideDiag &d = diagnostics.grip_sides[side];
                d.welded = false;
                d.enabled = s.active;
                d.reachable = s.reachable;
                d.extended = true;
                d.overloaded = overloaded;
                d.trial_pair_force_n = s.trial_norm;
                d.pair_force_limit_n = cfg_.grip_pair_force_limit_n;
                d.release_loss_j = s.active ? 0. : s.grip_loss;
                d.shoulder_distance_m = rider::hypot3(s.shoulder_gap);
                d.arm_reach_m = cfg_.arm_reach_m;
                d.hand_gap_m = rider::hypot3(s.hand_gap);
                d.point_m = s.grip;
                d.force_on_rider_n = s.force;
                d.force_on_bike_n = negate3(s.force);
                d.elastic_energy_j = s.grip_energy;
            }
            RiderContactGripDiag &aggregate = diagnostics.grip;
            aggregate.welded = false;
            aggregate.enabled = per_side[0].active || per_side[1].active;
            aggregate.reachable =
                per_side[0].reachable && per_side[1].reachable;
            aggregate.extended = true;
            aggregate.overloaded = overloaded;
            aggregate.trial_pair_force_n = norm3(total_force);
            aggregate.pair_force_limit_n = cfg_.grip_pair_force_limit_n;
            aggregate.release_loss_j =
                (per_side[0].active && per_side[1].active) ? 0. : pair_loss;
            aggregate.shoulder_distance_m =
                std::max(rider::hypot3(per_side[0].shoulder_gap),
                         rider::hypot3(per_side[1].shoulder_gap));
            aggregate.arm_reach_m = cfg_.arm_reach_m;
            aggregate.hand_gap_m =
                std::max(rider::hypot3(per_side[0].hand_gap),
                         rider::hypot3(per_side[1].hand_gap));
            aggregate.force_on_rider_n = total_force;
            aggregate.force_on_bike_n = negate3(total_force);
            aggregate.elastic_energy_j = pair_energy;
        }
        // Commit (lines 593-599 / 675-682): material state, diagnostics and
        // ledgers swap in one step; the crank read goes through the latch
        // when pedals are linked.
        w.supports = new_states;
        w.diagnostics = std::move(diagnostics);
        w.elastic_energy_j = energy;
        w.loss_step_j = loss;
        w.radial_dissipation_power_w = radial_power;
        if (linked_pedals_)
            delivered = w.settled_welds ? w.settled_welds->crank_torque_nm
                                      : 0.;
        w.delivered_crank_torque_nm = delivered;
        w.pending_release_loss_j = 0.;
        w.last_time_s = time;
    }
    NATIVE_DIAG_POP

    double RiderContactWriter::stored_energy(const mjData *data) const {
        double energy = 0.;
        for (std::size_t side = 0; side < kSideCount; ++side) {
            const Vec3 &xi = state_.grip_xi_local[side];
            energy += .5 * cfg_.grip_k_n_m * rider::dot(xi, xi);
        }
        for (std::size_t si = 0; si < kSupportCount; ++si) {
            if ((linked_pedals_ && si > 0) || (linked_saddle_ && si == 0))
                continue;
            for (std::size_t pad = 0; pad < kPadsPerSupport; ++pad) {
                PadEval c{};
                pad_eval(si, pad, data, c);
                energy += .25 * cfg_.support_tangent_k_n_m *
                          pyfloat::pow(state_.supports[kPadsPerSupport * si +
                                                       pad]
                                           .xi,
                                       2.);
                if (state_.enabled[si] && c.inside)
                    energy += .25 * cfg_.support_k_n_m *
                              pyfloat::pow(std::max(-c.gap, 0.), 2.);
            }
        }
        return energy;
    }

    RiderPreparedAttachments
    RiderContactWriter::prepare_attachment_raw(const mjData *data) const {
        // rider_contacts.py:221-269 — pose-only capture; errors replace
        // entries (unobservable attachments are never zero-substituted).
        RiderPreparedAttachments out;
        const auto run = [this, data, &out](const AttachmentTarget &t) {
            const std::optional<std::vector<double>> pull =
                pull_of(t);
            try {
                out.geometry.emplace_back(
                    t.out, rider::prepare_attachment_geometry(
                               model_, data, t.eq_id, t.rider_body,
                               t.bike_body, t.point, t.normal, t.kind,
                               t.rotational, t.half_patch_m, pull));
            } catch (const std::invalid_argument &) {
                out.errors.push_back(
                    t.out + ":unobservable_attachment_wrench");
            }
        };
        for (std::size_t si = 0; si < kSupportCount; ++si)
            if (const auto t = support_target(si, data)) run(*t);
        if (welded_grip_)
            for (std::size_t side = 0; side < kSideCount; ++side)
                if (const auto t = grip_target(side, data)) run(*t);
        return out;
    }

    RiderAttachmentMeasurement
    RiderContactWriter::attachment_samples(
        mjData *data, const std::optional<RiderIntervalState> &interval,
        bool raw) {
        if (!interval.has_value()) return measure_samples(data, raw);
        const auto &[qpos, qvel] = *interval;
        // The oracle's data.qpos[:]=qpos / data.qvel[:]=qvel broadcast
        // gate, checked before anything is written — plus the port's
        // all-finite domain (set_inputs/set_state's convention; a NaN
        // pose would only ever produce NaN telemetry, never a reading).
        if (qpos.size() != static_cast<std::size_t>(model_->nq))
            throw std::invalid_argument(
                "attachment interval qpos must be nq wide");
        if (qvel.size() != static_cast<std::size_t>(model_->nv))
            throw std::invalid_argument(
                "attachment interval qvel must be nv wide");
        if (!std::ranges::all_of(qpos, [](double v) {
                return std::isfinite(v);
            }))
            throw std::invalid_argument(
                "attachment interval qpos must be finite");
        if (!std::ranges::all_of(qvel, [](double v) {
                return std::isfinite(v);
            }))
            throw std::invalid_argument(
                "attachment interval qvel must be finite");
        const std::span<const double> live_qpos = std::views::counted(
            data->qpos, model_->nq);
        const std::span<const double> live_qvel = std::views::counted(
            data->qvel, model_->nv);
        const std::vector<double> saved_qpos(live_qpos.begin(),
                                             live_qpos.end());
        const std::vector<double> saved_qvel(live_qvel.begin(),
                                             live_qvel.end());
        const auto restore = [&]() {
            std::ranges::copy(saved_qpos, data->qpos);
            std::ranges::copy(saved_qvel, data->qvel);
            refresh_kinematics_com(data);
        };
        // Prologue stays outside the unwind scope like the oracle's:
        // data.qpos[:]=...; data.qvel[:]=...; mj_kinematics; mj_comPos.
        // A kinematics failure here leaves the swap in place and the
        // owning Stepper poisoned (engine::EngineFailure via mutate()).
        std::ranges::copy(qpos, data->qpos);
        std::ranges::copy(qvel, data->qvel);
        refresh_kinematics_com(data);
        RiderAttachmentMeasurement measured;
        try {
            measured = measure_samples(data, raw);
        } catch (...) {
            // try/finally — restore runs for EVERY escape (a collected
            // error is a normal exit; a propagating failure or a restore
            // fault replaces the in-flight exception like Python's
            // finally-raised one does).
            restore();
            throw;
        }
        restore();
        return measured;
    }

    RiderSettleOutcome RiderContactWriter::settle_welds(
        mjData *data, const std::optional<RiderIntervalState> &interval,
        bool raw, const RiderPreparedAttachments *prepared) {
        // rows = equality_rows(data) — ONE pass for the whole settle.
        const rider::EqualityRowMap rows = equality_.equality_rows(data);
        const std::span<const double> xmat =
            model_access::readonly_buffer(data->xmat, 9 * model_->nbody,
                                          "xmat");
        RiderSettleOutcome outcome;
        for (std::size_t si = 0; si < kSupportCount; ++si) {
            const bool linked = si == 0 ? linked_saddle_ : linked_pedals_;
            if (!linked) continue;
            const int eq = si == 0 ? saddle_eq_ : pedal_eq_[si - 1];
            // force_on_rider_n — the lam[:3] slice: zeros(3) when the
            // equality is absent; a shorter vector only when the
            // equality itself owns fewer than three rows (a renamed
            // joint equality) — where the oracle's force @ n matmul
            // raises ValueError, mirrored by the width gate below.
            const std::vector<double> force =
                equality_.force_on_rider(data, eq);
            Vec3 n{};
            Vec3 tangent{};
            if (si > 0 && spindle_pedals_) {
                // Foot z-column (compression toward the ankle) and its
                // in-plane perpendicular — unnormalized, as written.
                const Mat3 foot = mat3_at(xmat, supports_[si].body);
                n = {foot[2], foot[5], foot[8]};
                tangent = {n[2], 0., -n[0]};
            } else {
                PadEval first{};
                PadEval second{};
                pad_eval(si, 0, data, first);
                pad_eval(si, 1, data, second);
                // np.mean(axis=0) over the two pads — the ordered
                // (a+b)/2 pair — then the in-place /= max(norm, 1e-12).
                for (std::size_t i = 0; i < 3; ++i)
                    n[i] = (first.normal[i] + second.normal[i]) / 2.;
                const double n_scale = std::max(norm3(n), 1e-12);
                for (double &v : n) v /= n_scale;
                for (std::size_t i = 0; i < 3; ++i)
                    tangent[i] =
                        (first.tangent[i] + second.tangent[i]) / 2.;
                const double t_scale = std::max(norm3(tangent), 1e-12);
                for (double &v : tangent) v /= t_scale;
            }
            if (force.size() != 3)
                throw std::invalid_argument(
                    "attachment reaction is not a 3-vector");
            // normal = float(force @ n); shear = float(force @ tangent)
            // — np.dot's ddot dispatch.
            const double normal =
                blas::ddot(3, force.data(), 1, n.data(), 1);
            const double shear =
                blas::ddot(3, force.data(), 1, tangent.data(), 1);
            outcome.entries.emplace_back(
                std::string(kSupportNames[si]),
                RiderSettledEntry{
                    .support_fields = true,
                    .force_on_rider_n = {force[0], force[1], force[2]},
                    .normal_n = normal,
                    .tangent_n = shear,
                    .would_separate = normal < 0.,
                    .would_slip = std::abs(shear) >
                                  cfg_.support_mu * std::max(normal, 0.)});
        }
        if (welded_grip_) {
            for (std::size_t side = 0; side < kSideCount; ++side) {
                const std::vector<double> force =
                    equality_.force_on_rider(data, grip_eq_[side]);
                if (force.size() != 3)
                    throw std::invalid_argument(
                        "attachment reaction is not a 3-vector");
                outcome.entries.emplace_back(
                    std::string("grip_") + std::string(kSideNames[side]),
                    RiderSettledEntry{
                        .force_on_rider_n = {force[0], force[1],
                                             force[2]}});
            }
        }
        // delivered_crank_torque_nm over the pedal equalities — the
        // torque sensor's next-step read (0. when pedals are unlinked).
        const double crank =
            linked_pedals_
                ? equality_.equalities_qfrc_at(data, pedal_eq_, crank_dof_)
                : 0.;
        // _settled_welds = (deepcopy(result), crank) — the support/grip
        // entries BEFORE 'crank_torque_nm' joins the emitted dict.
        state_.settled_welds = RiderSettledWelds{.entries = outcome.entries,
                                                 .crank_torque_nm = crank};
        outcome.crank_torque_nm_appended = linked_pedals_;
        outcome.crank_torque_nm = crank;
        if (raw && prepared != nullptr) {
            // The raw period path: pre-step jacobians pair with the
            // CURRENT multipliers; validation stays deferred like the
            // oracle's validate_wrench=False.
            outcome.measurement.raw = true;
            outcome.measurement.errors = prepared->errors;
            for (const auto &[name, geometry] : prepared->geometry) {
                try {
                    outcome.measurement.raws.emplace_back(
                        name, rider::attachment_raw_from_geometry(
                                  data, equality_, lstsq_, measure_qfrc_,
                                  geometry, false, &rows));
                } catch (const std::invalid_argument &) {
                    outcome.measurement.errors.push_back(
                        name + ":unobservable_attachment_wrench");
                }
            }
        } else {
            outcome.measurement =
                attachment_samples(data, interval, raw);
        }
        // last_attachment_samples/last_attachment_errors commit — after
        // measurement, so a propagating failure keeps the previous
        // published pair (and the latch stays written — line 326).
        state_.attachment_samples.clear();
        if (outcome.measurement.raw) {
            for (const auto &[name, r] : outcome.measurement.raws)
                state_.attachment_samples.emplace_back(name, r);
        } else {
            for (const auto &[name, s] : outcome.measurement.samples)
                state_.attachment_samples.emplace_back(name, s);
        }
        state_.attachment_errors = outcome.measurement.errors;
        return outcome;
    }

    std::optional<RiderContactWriter::AttachmentTarget>
    RiderContactWriter::support_target(std::size_t support,
                                       const mjData *data) const {
        // The identical linked/field selection of prepare_attachment_raw
        // (rider_contacts.py:232-249) and _attachment_samples
        // (375-391) — porting them twice would invite drift.
        const SupportRef &ref = supports_[support];
        AttachmentTarget target;
        target.rider_body = ref.body;
        target.bike_body = ref.bike;
        if (support == 0) {
            if (!linked_saddle_) return std::nullopt;
            target.eq_id = saddle_eq_;
            target.rotational = welded_saddle_;
            target.half_patch_m = cfg_.saddle_patch_half_length_m;
            target.out = "saddle";
            target.kind = "saddle";
        } else {
            if (!linked_pedals_) return std::nullopt;
            target.eq_id = pedal_eq_[support - 1];
            target.rotational = !spindle_pedals_;
            target.out = support == 1 ? "foot_front" : "foot_rear";
            target.kind = "foot";
            if (spindle_pedals_) {
                target.half_patch_m = 0.;
            } else {
                const std::span<const double> geom_size =
                    model_access::readonly_buffer(model_->geom_size,
                                                  3 * model_->ngeom,
                                                  "geom_size");
                target.half_patch_m = geom_size[3 *
                    static_cast<std::size_t>(ref.geom)];
            }
        }
        const std::span<const double> site_xpos =
            model_access::readonly_buffer(data->site_xpos,
                                          3 * model_->nsite, "site_xpos");
        // point = np.array(data.site_xpos[site]) — then the spindle
        // pedal moves it to the compiled connect anchor and replaces the
        // normal with the foot's world +z column.
        target.point = vec3_at(site_xpos, ref.site);
        if (support > 0 && spindle_pedals_) {
            const std::span<const double> xmat =
                model_access::readonly_buffer(data->xmat,
                                              9 * model_->nbody, "xmat");
            const std::span<const double> xpos =
                model_access::readonly_buffer(data->xpos,
                                              3 * model_->nbody, "xpos");
            const std::span<const double> eq_data =
                model_access::readonly_buffer(model_->eq_data,
                                              mjNEQDATA * model_->neq,
                                              "eq_data");
            const Mat3 foot = mat3_at(xmat, ref.body);
            target.normal = {foot[2], foot[5], foot[8]};
            const Vec3 anchor = vec3_off(
                eq_data, static_cast<std::size_t>(mjNEQDATA) *
                             static_cast<std::size_t>(target.eq_id));
            target.point = rider::add(vec3_at(xpos, ref.body),
                                      rider::matvec(foot, anchor));
        } else {
            // np.mean([n0, n1], axis=0) — the ordered pair mean,
            // unnormalized (decompose/geometry normalize downstream).
            PadEval first{};
            PadEval second{};
            pad_eval(support, 0, data, first);
            pad_eval(support, 1, data, second);
            for (std::size_t i = 0; i < 3; ++i)
                target.normal[i] =
                    (first.normal[i] + second.normal[i]) / 2.;
        }
        return target;
    }

    std::optional<RiderContactWriter::AttachmentTarget>
    RiderContactWriter::grip_target(std::size_t side,
                                    const mjData *data) const {
        // The welded-grip connect read (lines 257-262/399-404): anchor
        // captured at reset; None skips the side like the oracle.
        const auto &anchor = state_.grip_anchor_local[side];
        if (!anchor.has_value()) return std::nullopt;
        const std::span<const double> xpos =
            model_access::readonly_buffer(data->xpos, 3 * model_->nbody,
                                          "xpos");
        const std::span<const double> xmat =
            model_access::readonly_buffer(data->xmat, 9 * model_->nbody,
                                          "xmat");
        const Mat3 steer = mat3_at(xmat, steer_);
        // grip = xpos[steer] + xmat[steer] @ anchor — R·v.
        const Vec3 grip =
            rider::add(vec3_at(xpos, steer_), rider::matvec(steer, *anchor));
        // pull = xpos[pelvis] - grip — both `normal` and pull_direction.
        const Vec3 pull = rider::subtract(vec3_at(xpos, pelvis_), grip);
        AttachmentTarget target;
        target.eq_id = grip_eq_[side];
        target.rider_body = forearm_[side];
        target.bike_body = steer_;
        target.out =
            std::string("grip_") + std::string(kSideNames[side]);
        target.kind = "grip";
        target.point = grip;
        target.normal = pull;
        target.pull = pull;
        return target;
    }

    RiderAttachmentMeasurement
    RiderContactWriter::measure_samples(const mjData *data, bool raw) {
        // _attachment_samples (rider_contacts.py:359-412): every linked
        // support then the welded grips; a ValueError-in-Python
        // (std::invalid_argument here) collects the
        // ':unobservable_attachment_wrench' string instead of a sample.
        RiderAttachmentMeasurement out;
        out.raw = raw;
        const rider::EqualityRowMap rows = equality_.equality_rows(data);
        const auto run = [this, data, raw, &rows,
                          &out](const AttachmentTarget &t) {
            const std::optional<std::vector<double>> pull = pull_of(t);
            try {
                if (raw) {
                    out.raws.emplace_back(
                        t.out,
                        rider::attachment_raw(
                            model_, data, equality_, measure_qfrc_,
                            t.eq_id, t.rider_body, t.bike_body, t.point,
                            t.normal, t.kind, t.rotational,
                            t.half_patch_m, pull, &rows));
                } else {
                    out.samples.emplace_back(
                        t.out,
                        rider::attachment_sample(
                            model_, data, equality_, lstsq_,
                            measure_qfrc_, t.eq_id, t.rider_body,
                            t.bike_body, t.point, t.normal, t.kind,
                            t.rotational, t.half_patch_m, pull, &rows));
                }
            } catch (const std::invalid_argument &) {
                out.errors.push_back(
                    t.out + ":unobservable_attachment_wrench");
            }
        };
        for (std::size_t si = 0; si < kSupportCount; ++si)
            if (const auto t = support_target(si, data)) run(*t);
        if (welded_grip_)
            for (std::size_t side = 0; side < kSideCount; ++side)
                if (const auto t = grip_target(side, data)) run(*t);
        return out;
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (support, pad) mirrors the oracle's (key, patch) pair
    void RiderContactWriter::pad_eval(std::size_t support, std::size_t pad,
                                      const mjData *data, PadEval &out) const {
        // _pads (rider_contacts.py:192-205): the circular sample sits above
        // the declared sole surface — the FOOT's rotation carries the patch
        // offset, while the contact reads the support geom's own pose.
        const SupportRef &ref = supports_[support];
        const double half =
            support == 0 ? cfg_.saddle_patch_half_length_m
                         : cfg_.pedal_patch_half_length_m;
        const double radius = cfg_.support_pad_radius_m;
        const std::span<const double> site_xpos =
            model_access::readonly_buffer(data->site_xpos,
                                          3 * model_->nsite, "site_xpos");
        const std::span<const double> xmat = model_access::readonly_buffer(
            data->xmat, 9 * model_->nbody, "xmat");
        const std::span<const double> geom_xpos =
            model_access::readonly_buffer(data->geom_xpos, 3 * model_->ngeom,
                                          "geom_xpos");
        const std::span<const double> geom_xmat =
            model_access::readonly_buffer(data->geom_xmat, 9 * model_->ngeom,
                                          "geom_xmat");
        const std::span<const double> geom_size =
            model_access::readonly_buffer(model_->geom_size,
                                          3 * model_->ngeom, "geom_size");
        const Mat3 foot = mat3_at(xmat, ref.body);
        const double sign = pad == 0 ? -1. : 1.;
        const Vec3 center =
            rider::add(vec3_at(site_xpos, ref.site),
                       rider::matvec(foot, {sign * half, 0., radius}));
        const rider::BoxPadContact contact =
            rider::box_pad_contact_unchecked(center, radius,
                                             vec3_at(geom_xpos, ref.geom),
                                             mat3_at(geom_xmat, ref.geom),
                                             vec3_at(geom_size, ref.geom));
        out.point = contact.point_m;
        out.normal = contact.normal;
        out.tangent = contact.tangent();
        out.gap = contact.gap_m;
        out.inside = contact.within_footprint;
    }

    const std::vector<double> &RiderContactWriter::relative_jacobian(
        const mjData *data, int body_a, int body_b, const Vec3 &point) {
        // relative_point_jacobian (physical_mapping.py:40-59): the body
        // gate precedes the engine calls with the oracle's message; the
        // helper's own checks then bound the write.
        if (!(0 < body_a && body_a < model_->nbody &&
              0 < body_b && body_b < model_->nbody) ||
            body_a == body_b)
            throw std::invalid_argument(
                "relative Jacobian requires distinct physical bodies");
        model_access::point_jacobian_into(
            model_, data, body_a, point, jac_a_,
            model_access::JacobianBody::physical, "relative Jacobian body");
        model_access::point_jacobian_into(
            model_, data, body_b, point, jac_b_,
            model_access::JacobianBody::physical, "relative Jacobian body");
        for (std::size_t i = 0; i < jac_a_.size(); ++i) jac_a_[i] -= jac_b_[i];
        return jac_a_;
    }

    std::pair<Vec3, double>
    RiderContactWriter::settled_force(const RiderContactsState &w,
                                      std::string_view name) const {
        const RiderSettledEntry *const entry = settled_entry(w, name);
        if (entry == nullptr) return {Vec3{0., 0., 0.}, 0.};
        return {entry->force_on_rider_n, std::max(entry->normal_n, 0.)};
    }

    const RiderSettledEntry *
    RiderContactWriter::settled_entry(const RiderContactsState &w,
                                      std::string_view name) const {
        if (!w.settled_welds.has_value()) return nullptr;
        for (const auto &[key, entry] : w.settled_welds->entries)
            if (key == name) return &entry;
        return nullptr;
    }

    void RiderContactWriter::refresh_kinematics(mjData *data) const {
        engine::ErrorBuffer error;
        KinematicsContext context{.model = model_, .data = data};
        if (!engine::invoke(kinematics_operation, &context, error))
            engine::throw_failure(error);
    }

    void RiderContactWriter::refresh_kinematics_com(mjData *data) const {
        engine::ErrorBuffer error;
        KinematicsContext context{.model = model_, .data = data};
        if (!engine::invoke(kinematics_operation, &context, error))
            engine::throw_failure(error);
        if (!engine::invoke(compos_operation, &context, error))
            engine::throw_failure(error);
    }
} // namespace writers
