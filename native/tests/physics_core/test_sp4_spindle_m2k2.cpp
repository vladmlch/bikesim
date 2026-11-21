#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/gpu_math.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/joint_envelope.hpp"
#include "physics_core/spindle_m2k2.hpp"
#include "physics_core/spindle_m2k2_rider_bridge.hpp"
#include "physics_core/spindle_m2k2_upload.hpp"

#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/rider_spec.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/spindle_math.hpp"
#include "rider/state_codec.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/config.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "runtime/static_brake.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include "writers/drivetrain.hpp"
#include "writers/resistance.hpp"
#include "writers/rider_contacts.hpp"
#include "writers/suspension.hpp"

#if defined(__APPLE__)
#include "test_sp4_spindle_metal_helper.hpp"
#endif

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <ranges>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace bio = bike::io;
namespace fs = std::filesystem;
namespace pc = bike::physics_core;

using ModelPtr = std::unique_ptr<mjModel, void (*)(mjModel *)>;
using DataPtr = std::unique_ptr<mjData, void (*)(mjData *)>;

void delete_model(mjModel *m) {
    if (m != nullptr) {
        mj_deleteModel(m);
    }
}

void delete_data(mjData *d) {
    if (d != nullptr) {
        mj_deleteData(d);
    }
}

std::size_t to_idx(int v) {
    if (v < 0) {
        throw std::out_of_range("negative index");
    }
    return static_cast<std::size_t>(v);
}

void require(bool cond, std::string_view msg) {
    bt::require(cond, msg);
}

void require_near(double actual, double expected, double tol, std::string_view what) {
    const double diff = std::abs(actual - expected);
    if (!(diff <= tol)) {
        std::ostringstream oss;
        oss << what << ": actual=" << actual << " expected=" << expected
            << " diff=" << diff << " tol=" << tol;
        throw std::runtime_error(oss.str());
    }
}

void require_bits_f32(float actual, float expected, std::string_view what) {
    if (std::isnan(actual) && std::isnan(expected)) {
        return;
    }
    const auto ua = std::bit_cast<std::uint32_t>(actual);
    const auto ue = std::bit_cast<std::uint32_t>(expected);
    if (ua != ue) {
        std::ostringstream oss;
        oss << what << ": 32-bit mismatch (actual=0x" << std::hex << ua
            << ", expected=0x" << ue << ")";
        throw std::runtime_error(oss.str());
    }
}

fs::path repo_file(std::string_view rel) {
    return fs::path(BIKE_PHYSICS_CORE_SRC_DIR).parent_path().parent_path().parent_path() /
           fs::path(rel);
}

ModelPtr load_xml(const fs::path &path) {
    std::array<char, 1024> err_buf{};
    mjModel *raw = mj_loadXML(path.string().c_str(), nullptr, err_buf.data(), static_cast<int>(err_buf.size()));
    if (raw == nullptr) {
        throw std::runtime_error(std::string("mj_loadXML failed: ") + err_buf.data());
    }
    return {raw, &delete_model};
}

struct CpuExactTrig {
    static double flush_subnorm(double x) noexcept {
        return x;
    }
    template <class Real, class TK>
    static Real sin_val(Real x) noexcept {
        return static_cast<Real>(std::sin(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real cos_val(Real x) noexcept {
        return static_cast<Real>(std::cos(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real atan_val(Real x) noexcept {
        return static_cast<Real>(std::atan(static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real atan2_val(Real y, Real x) noexcept {
        return static_cast<Real>(std::atan2(static_cast<double>(y), static_cast<double>(x)));
    }
    template <class Real, class TK>
    static Real acos_val(Real x) noexcept {
        return static_cast<Real>(std::acos(std::clamp(static_cast<double>(x), -1.0, 1.0)));
    }
};

struct MjMulMFn {
    const mjModel *m = nullptr;
    const mjData *d = nullptr;
    template <class PtrIn, class PtrOut>
    void operator()(PtrIn imposed, PtrOut out_inertia) const {
        engine::mul_m(m, d, out_inertia, imposed);
    }
};

template <class Real>
std::vector<Real> to_real_vec(std::span<const double> src) {
    std::vector<Real> dst(src.size());
    for (std::size_t i = 0; i < src.size(); ++i) {
        dst[i] = static_cast<Real>(src[i]);
    }
    return dst;
}

template <class Real>
std::vector<Real> to_real_vec(const double *ptr, mjtSize count) {
    return to_real_vec<Real>(std::views::counted(ptr, count));
}

std::vector<pc::i32> to_i32_vec(const int *ptr, mjtSize count) {
    const auto sp = std::views::counted(ptr, count);
    return {sp.begin(), sp.end()};
}

// ---- Test 1: Analytical 2-link J, dJ/dphi, gammas vs FP64 FD ---------------

void test_analytic_leg_kinematics_accuracy(const pc::SpindleM2K2Params<double> &params64) {
    std::cout << "[SP4-A] 1. Analytical 2-link leg kinematics J, dJ/dphi, gammas...\n";

    double max_j_err_f64 = 0.0;
    double max_dj_err_f64 = 0.0;
    double max_gz_err_f64 = 0.0;
    double max_gx_err_f64 = 0.0;
    double max_j_err_f32 = 0.0;
    double max_dj_err_f32 = 0.0;

    constexpr int kSamples = 129;
    constexpr double kFdH = 1e-6;
    constexpr double kFd4H = 1e-3;
    const double crank_m = params64.crank_length_m;

    // Representative hip position in frame coordinates (matching welded.xml)
    const double hip_x = -0.16874614044463943;
    const double hip_z = 0.76398017851206435;

    for (int s = 0; s < 2; ++s) {
        const std::size_t su = to_idx(s);
        const spindle::LegLoopGeometry geom{
            .thigh_m = std::span{params64.thigh_m}[su],
            .link_m = std::span{params64.link_m}[su],
            .knee_sign = std::span{params64.knee_sign}[su],
            .thigh_angle0_rad = std::span{params64.thigh_angle0_rad}[su],
            .link_angle0_rad = std::span{params64.link_angle0_rad}[su],
            .side_sign = std::span{params64.side_sign}[su],
        };
        const spindle::Vec2 hip_xz{hip_x, hip_z};
        const spindle::Vec2 crank_xz{0.0, 0.0};

        for (int k = 0; k < kSamples; ++k) {
            const double ph = -std::numbers::pi +
                              (2.0 * std::numbers::pi * static_cast<double>(k)) /
                                  static_cast<double>(kSamples - 1);
            const auto q_fn = [&](double p) {
                return spindle::leg_joint_q(
                    geom, hip_xz,
                    spindle::spindle_xz(crank_xz, crank_m, p, geom.side_sign), 0.0);
            };
            const spindle::Vec2 qp = q_fn(ph + kFdH);
            const spindle::Vec2 qm = q_fn(ph - kFdH);
            const std::array<double, 2> j_fd{
                spindle::wrap_angle(qp[0] - qm[0]) / (2.0 * kFdH),
                spindle::wrap_angle(qp[1] - qm[1]) / (2.0 * kFdH),
            };
            const spindle::Vec2 q_m2 = q_fn(ph - 2.0 * kFd4H);
            const spindle::Vec2 q_m1 = q_fn(ph - kFd4H);
            const spindle::Vec2 q_0 = q_fn(ph);
            const spindle::Vec2 q_p1 = q_fn(ph + kFd4H);
            const spindle::Vec2 q_p2 = q_fn(ph + 2.0 * kFd4H);
            const std::array<double, 2> dj_fd4{
                (-spindle::wrap_angle(q_p2[0] - q_0[0]) +
                 16.0 * spindle::wrap_angle(q_p1[0] - q_0[0]) +
                 16.0 * spindle::wrap_angle(q_m1[0] - q_0[0]) -
                 spindle::wrap_angle(q_m2[0] - q_0[0])) /
                    (12.0 * kFd4H * kFd4H),
                (-spindle::wrap_angle(q_p2[1] - q_0[1]) +
                 16.0 * spindle::wrap_angle(q_p1[1] - q_0[1]) +
                 16.0 * spindle::wrap_angle(q_m1[1] - q_0[1]) -
                 spindle::wrap_angle(q_m2[1] - q_0[1])) /
                    (12.0 * kFd4H * kFd4H),
            };

            const spindle::Vec2 sp_xz = spindle::spindle_xz(crank_xz, crank_m, ph, geom.side_sign);
            const spindle::Vec2 gz_ref = spindle::leg_vertical_gamma(geom, hip_xz, sp_xz);
            const spindle::Vec2 gx_ref = spindle::leg_horizontal_gamma(geom, hip_xz, sp_xz);

            const auto an64 = pc::analytic_leg_kinematics<double, pc::CpuExactToolkit, CpuExactTrig>(
                geom.thigh_m, geom.link_m, crank_m, geom.side_sign, geom.knee_sign,
                hip_x, hip_z, ph, 1e-12, 1e-9);
            require(an64.valid != 0, "analytic64 valid");
            for (int i = 0; i < 2; ++i) {
                const std::size_t iu = to_idx(i);
                max_j_err_f64 = std::max(max_j_err_f64, std::abs(std::span{an64.jac}[iu] - j_fd[iu]));
                max_dj_err_f64 = std::max(max_dj_err_f64, std::abs(std::span{an64.djac}[iu] - dj_fd4[iu]));
                max_gz_err_f64 = std::max(max_gz_err_f64, std::abs(std::span{an64.gamma_z}[iu] - gz_ref[iu]));
                max_gx_err_f64 = std::max(max_gx_err_f64, std::abs(std::span{an64.gamma_x}[iu] - gx_ref[iu]));
            }

            const auto an32 = pc::analytic_leg_kinematics<float, pc::GpuToolkit>(
                static_cast<float>(geom.thigh_m), static_cast<float>(geom.link_m),
                static_cast<float>(crank_m), static_cast<float>(geom.side_sign),
                static_cast<float>(geom.knee_sign), static_cast<float>(hip_x),
                static_cast<float>(hip_z), static_cast<float>(ph), 1e-6f, 1e-5f);
            require(an32.valid != 0, "analytic32 valid");
            for (int i = 0; i < 2; ++i) {
                const std::size_t iu = to_idx(i);
                max_j_err_f32 = std::max(
                    max_j_err_f32, std::abs(static_cast<double>(std::span{an32.jac}[iu]) - j_fd[iu]));
                max_dj_err_f32 = std::max(
                    max_dj_err_f32, std::abs(static_cast<double>(std::span{an32.djac}[iu]) - dj_fd4[iu]));
            }
        }
    }

    // Also test knee_sign = -1 branch
    {
        const spindle::LegLoopGeometry geom_neg{
            .thigh_m = 0.42,
            .link_m = 0.45,
            .knee_sign = -1.0,
            .thigh_angle0_rad = 0.2,
            .link_angle0_rad = -0.3,
            .side_sign = 1.0,
        };
        const double ph = 0.4;
        const double h = 1e-6;
        const auto q_fn = [&](double p) {
            return spindle::leg_joint_q(
                geom_neg, {-0.15, 0.70},
                spindle::spindle_xz({0.0, 0.0}, 0.165, p, 1.0), 0.0);
        };
        const auto qp = q_fn(ph + h);
        const auto qm = q_fn(ph - h);
        const auto an = pc::analytic_leg_kinematics<double, pc::CpuExactToolkit, CpuExactTrig>(
            0.42, 0.45, 0.165, 1.0, -1.0, -0.15, 0.70, ph, 1e-12, 1e-9);
        require(an.valid != 0, "knee_sign=-1 valid");
        require_near(an.jac[0], spindle::wrap_angle(qp[0] - qm[0]) / (2.0 * h), 1e-8, "knee_sign=-1 J0");
        require_near(an.jac[1], spindle::wrap_angle(qp[1] - qm[1]) / (2.0 * h), 1e-8, "knee_sign=-1 J1");
    }

    std::cout << "  FP64 max|dJ|=" << max_j_err_f64 << " max|ddJ|=" << max_dj_err_f64
              << " max|dgz|=" << max_gz_err_f64 << " max|dgx|=" << max_gx_err_f64 << "\n"
              << "  FP32 max|dJ|=" << max_j_err_f32 << " max|ddJ|=" << max_dj_err_f32 << "\n";

    require(max_j_err_f64 < 1e-8, "FP64 analytic J vs FD64 < 1e-8");
    require(max_dj_err_f64 < 1e-8, "FP64 analytic dJ/dphi vs 4th-order FD64 < 1e-8");
    require(max_gz_err_f64 < 1e-12, "FP64 gamma_z exact");
    require(max_gx_err_f64 < 1e-12, "FP64 gamma_x exact");
    require(max_j_err_f32 <= 1e-6, "FP32 analytic J vs FD64 <= 1e-6");
    require(max_dj_err_f32 <= 1e-5, "FP32 analytic dJ/dphi vs FD64 <= 1e-5");
}

// ---- Test 2 & 3: subtree_com leg_weights_at_pedal + CSR M*v parity --------

void test_leg_weights_and_csr_matvec_parity(
    const mjModel *m,
    const pc::SpindleM2K2Params<double> &params64,
    const pc::SpindleM2K2Params<float> &params32
) {
    std::cout << "[SP4-A] 2 & 3. subtree_com leg_weights_at_pedal & CSR M*v parity...\n";

    DataPtr d(engine::make_data(m), &delete_data);
    const int crank_qpos = model_access::readonly_buffer(m->jnt_qposadr, m->njnt, "jnt_qposadr")
        [to_idx(params64.crank_joint)];
    const int hip_f_dof = params64.joint_dof_adr[pc::kSpindleSlotHipFront];
    const int knee_f_dof = params64.joint_dof_adr[pc::kSpindleSlotKneeFront];
    const int hip_r_dof = params64.joint_dof_adr[pc::kSpindleSlotHipRear];
    const int knee_r_dof = params64.joint_dof_adr[pc::kSpindleSlotKneeRear];

    std::vector<int> rider_bodies;
    for (int b = 0; b < m->nbody; ++b) {
        const char *name = mj_id2name(m, mjOBJ_BODY, b);
        if (name != nullptr && std::string_view(name).starts_with("rider_")) {
            rider_bodies.push_back(b);
        }
    }

    const auto body_mass = model_access::readonly_buffer(m->body_mass, m->nbody, "body_mass");
    const auto gravity = model_access::readonly_buffer(std::data(m->opt.gravity), 3, "opt.gravity");
    const auto M_rowadr_i32 = to_i32_vec(m->M_rowadr, m->nv);
    const auto M_rownnz_i32 = to_i32_vec(m->M_rownnz, m->nv);
    const auto M_colind_i32 = to_i32_vec(m->M_colind, static_cast<int>(m->nC));
    const auto submass_f32 = to_real_vec<float>(m->body_subtreemass, m->nbody);

    double max_fw_err_f64 = 0.0;
    double max_fw_err_f32 = 0.0;
    double max_csr_err_f64 = 0.0;
    double max_csr_err_f32 = 0.0;
    double max_ps_mulm_err_f64 = 0.0;

    std::vector<double> jac(3 * static_cast<std::size_t>(m->nv), 0.0);
    std::vector<double> imposed(static_cast<std::size_t>(m->nv), 0.0);
    std::vector<double> mulm_ref(static_cast<std::size_t>(m->nv), 0.0);
    std::vector<double> csr_out64(static_cast<std::size_t>(m->nv), 0.0);
    std::vector<float> csr_out32(static_cast<std::size_t>(m->nv), 0.0f);

    auto scratch64 = std::make_unique<pc::SpindleM2K2StepScratch<double>>();

    for (int k = 0; k < 8; ++k) {
        mj_resetData(m, d.get());
        const double ph = -std::numbers::pi + 0.8 * static_cast<double>(k);
        model_access::mutable_buffer(d->qpos, m->nq, "qpos")[to_idx(crank_qpos)] = ph;
        model_access::mutable_buffer(d->qpos, m->nq, "qpos")[to_idx(params64.joint_qpos_adr[0])] =
            0.05 * static_cast<double>(k - 3);
        model_access::mutable_buffer(d->qvel, m->nv, "qvel")[to_idx(params64.crank_spin_dof)] = 8.5;
        engine::forward(m, d.get());

        // Reference mj_jacBodyCom sum over rider_bodies
        const auto xmat = model_access::readonly_buffer(d->xmat, m->nbody * 9, "xmat");
        const auto xpos = model_access::readonly_buffer(d->xpos, m->nbody * 3, "xpos");
        const auto xanchor = model_access::readonly_buffer(d->xanchor, m->njnt * 3, "xanchor");
        const std::size_t fb9 = to_idx(params64.frame_body) * 9;
        const std::size_t cj3 = to_idx(params64.crank_joint) * 3;
        const auto to_fxz = [&](std::span<const double> buf, std::size_t off) -> std::array<double, 2> {
            const double dx = buf[off + 0] - xanchor[cj3 + 0];
            const double dy = buf[off + 1] - xanchor[cj3 + 1];
            const double dz = buf[off + 2] - xanchor[cj3 + 2];
            return {xmat[fb9 + 0] * dx + xmat[fb9 + 3] * dy + xmat[fb9 + 6] * dz,
                    xmat[fb9 + 2] * dx + xmat[fb9 + 5] * dy + xmat[fb9 + 8] * dz};
        };
        const auto sp0 = to_fxz(xpos, to_idx(params64.pedal_bodies[0]) * 3);
        const double phase = std::atan2(-sp0[1], sp0[0]);

        std::array<std::array<double, 2>, 2> gz{};
        for (int s = 0; s < 2; ++s) {
            const std::size_t su = to_idx(s);
            const auto h_xz = to_fxz(xanchor, to_idx(std::span{params64.hip_joints}[su]) * 3);
            const auto an = pc::analytic_leg_kinematics<double, pc::CpuExactToolkit, CpuExactTrig>(
                std::span{params64.thigh_m}[su], std::span{params64.link_m}[su], params64.crank_length_m,
                std::span{params64.side_sign}[su], std::span{params64.knee_sign}[su],
                h_xz[0], h_xz[1], phase, 1e-12, 1e-9);
            gz[su][0] = an.gamma_z[0];
            gz[su][1] = an.gamma_z[1];
            std::span{scratch64->gammas}[su][0] = an.gamma_z[0];
            std::span{scratch64->gammas}[su][1] = an.gamma_z[1];
            std::span{scratch64->gammas_x}[su][0] = an.gamma_x[0];
            std::span{scratch64->gammas_x}[su][1] = an.gamma_x[1];
            std::span{scratch64->jacobians}[su][0] = an.jac[0];
            std::span{scratch64->jacobians}[su][1] = an.jac[1];
            std::span{scratch64->djacs}[su][0] = an.djac[0];
            std::span{scratch64->djacs}[su][1] = an.djac[1];
            for (int c = 0; c < 3; ++c) {
                const std::size_t cu = to_idx(c);
                std::span{std::span{scratch64->hip_world}[su]}[cu] =
                    xanchor[to_idx(std::span{params64.hip_joints}[su]) * 3 + cu];
                std::span{std::span{scratch64->delta_hip}[su]}[cu] =
                    std::span{std::span{scratch64->hip_world}[su]}[cu] - xanchor[cj3 + cu];
            }
        }
        for (int i = 0; i < 9; ++i) {
            std::span{scratch64->frame_r}[to_idx(i)] = xmat[fb9 + to_idx(i)];
        }
        for (int c = 0; c < 3; ++c) {
            const std::size_t cu = to_idx(c);
            std::span{scratch64->crank_world}[cu] = xanchor[cj3 + cu];
            std::span{scratch64->frame_pitch_anchor}[cu] =
                xanchor[to_idx(params64.frame_pitch_joint) * 3 + cu];
            std::span{scratch64->rider_pitch_anchor}[cu] =
                xanchor[to_idx(params64.rider_pitch_joint) * 3 + cu];
        }
        scratch64->rate = 8.5;

        std::array<std::array<double, 2>, 2> leg_g_ref{};
        const std::size_t w = static_cast<std::size_t>(m->nv);
        const std::array<std::array<int, 2>, 2> leg_dofs{{{hip_f_dof, knee_f_dof}, {hip_r_dof, knee_r_dof}}};
        for (const int body : rider_bodies) {
            mj_jacBodyCom(m, d.get(), jac.data(), nullptr, body);
            const double mb = body_mass[to_idx(body)];
            for (int s = 0; s < 2; ++s) {
                const std::size_t su = to_idx(s);
                for (int i = 0; i < 2; ++i) {
                    const std::size_t iu = to_idx(i);
                    const std::size_t dof = to_idx(leg_dofs[su][iu]);
                    leg_g_ref[su][iu] -= mb * (gravity[0] * jac[dof] +
                                               gravity[1] * jac[w + dof] +
                                               gravity[2] * jac[2 * w + dof]);
                }
            }
        }
        const std::array<double, 2> fw_ref{
            gz[0][0] * leg_g_ref[0][0] + gz[0][1] * leg_g_ref[0][1],
            gz[1][0] * leg_g_ref[1][0] + gz[1][1] * leg_g_ref[1][1],
        };

        std::array<double, 2> fw64{};
        pc::leg_weights_at_pedal_core<double>(
            &params64, m->body_subtreemass, d->subtree_com, d->xanchor, d->xaxis,
            gz[0][0], gz[0][1], gz[1][0], gz[1][1], fw64.data());
        max_fw_err_f64 = std::max(max_fw_err_f64, std::abs(fw64[0] - fw_ref[0]));
        max_fw_err_f64 = std::max(max_fw_err_f64, std::abs(fw64[1] - fw_ref[1]));

        std::array<double, 2> fw64_null{};
        pc::leg_weights_at_pedal_core<double>(
            &params64, nullptr, d->subtree_com, d->xanchor, d->xaxis,
            gz[0][0], gz[0][1], gz[1][0], gz[1][1], fw64_null.data());
        require(fw64_null[0] == fw64[0] && fw64_null[1] == fw64[1],
                "leg_weights FP64 nullptr subtreemass identical");

        const auto scom_f32 = to_real_vec<float>(d->subtree_com, m->nbody * 3);
        const auto xanc_f32 = to_real_vec<float>(d->xanchor, m->njnt * 3);
        const auto xax_f32 = to_real_vec<float>(d->xaxis, m->njnt * 3);
        std::array<float, 2> fw32{};
        pc::leg_weights_at_pedal_core<float>(
            &params32, submass_f32.data(), scom_f32.data(), xanc_f32.data(), xax_f32.data(),
            static_cast<float>(gz[0][0]), static_cast<float>(gz[0][1]),
            static_cast<float>(gz[1][0]), static_cast<float>(gz[1][1]), fw32.data());
        max_fw_err_f32 = std::max(max_fw_err_f32, std::abs(static_cast<double>(fw32[0]) - fw_ref[0]));
        max_fw_err_f32 = std::max(max_fw_err_f32, std::abs(static_cast<double>(fw32[1]) - fw_ref[1]));

        std::array<float, 2> fw32_null{};
        pc::leg_weights_at_pedal_core<float>(
            &params32, nullptr, scom_f32.data(), xanc_f32.data(), xax_f32.data(),
            static_cast<float>(gz[0][0]), static_cast<float>(gz[0][1]),
            static_cast<float>(gz[1][0]), static_cast<float>(gz[1][1]), fw32_null.data());
        require_bits_f32(fw32_null[0], fw32[0], "leg_weights FP32 nullptr submass[0]");
        require_bits_f32(fw32_null[1], fw32[1], "leg_weights FP32 nullptr submass[1]");

        // CSR & dense qM matvec parity vs engine::mul_m
        for (int i = 0; i < m->nv; ++i) {
            imposed[to_idx(i)] = std::sin(0.37 * static_cast<double>(i + 1) + 0.2 * k);
        }
        engine::mul_m(m, d.get(), mulm_ref.data(), imposed.data());
        pc::mul_m_csr<double>(
            static_cast<pc::i32>(m->nv), M_rowadr_i32.data(), M_rownnz_i32.data(),
            M_colind_i32.data(), d->M, imposed.data(), csr_out64.data());
        for (int i = 0; i < m->nv; ++i) {
            max_csr_err_f64 = std::max(max_csr_err_f64, std::abs(csr_out64[to_idx(i)] - mulm_ref[to_idx(i)]));
        }

        const auto dM_f32 = to_real_vec<float>(d->M, m->nC);
        const auto imp_f32 = to_real_vec<float>(imposed);
        pc::mul_m_csr<float>(
            static_cast<pc::i32>(m->nv), M_rowadr_i32.data(), M_rownnz_i32.data(),
            M_colind_i32.data(), dM_f32.data(), imp_f32.data(), csr_out32.data());
        for (int i = 0; i < m->nv; ++i) {
            max_csr_err_f32 = std::max(
                max_csr_err_f32, std::abs(static_cast<double>(csr_out32[to_idx(i)]) - mulm_ref[to_idx(i)]));
        }

        // Dense qM parity (strided nv_pad = nv + 4, matching mujoco_warp padded dense matrices)
        const int nv = static_cast<int>(m->nv);
        const int stride = nv + 4;
        std::vector<double> qM_full(static_cast<std::size_t>(nv * nv), 0.0);
        mj_fullM(m, d.get(), qM_full.data());
        std::vector<double> qM_strided64(static_cast<std::size_t>(nv * stride), 123.0);
        std::vector<float> qM_strided32(static_cast<std::size_t>(nv * stride), 123.0f);
        for (int r = 0; r < nv; ++r) {
            for (int c = 0; c < nv; ++c) {
                const double val = qM_full[to_idx(r * nv + c)];
                qM_strided64[to_idx(r * stride + c)] = val;
                qM_strided32[to_idx(r * stride + c)] = static_cast<float>(val);
            }
        }
        std::vector<double> dense_out64(static_cast<std::size_t>(nv), 0.0);
        std::vector<float> dense_out32(static_cast<std::size_t>(nv), 0.0f);
        pc::mul_m_dense<double>(nv, stride, qM_strided64.data(), imposed.data(), dense_out64.data());
        pc::mul_m_dense<float>(nv, stride, qM_strided32.data(), imp_f32.data(), dense_out32.data());
        for (int i = 0; i < nv; ++i) {
            require(std::abs(dense_out64[to_idx(i)] - mulm_ref[to_idx(i)]) <= 1e-12,
                    "mul_m_dense FP64 vs engine::mul_m <= 1e-12");
            require(std::abs(static_cast<double>(dense_out32[to_idx(i)]) - mulm_ref[to_idx(i)]) <= 2e-5,
                    "mul_m_dense FP32 vs engine::mul_m <= 2e-5");
        }
        const auto ps_dense64 = pc::compute_press_state_dense<double>(
            &params64, scratch64.get(), stride, qM_strided64.data(), imposed.data(), d->qfrc_bias);
        const auto ps_csr_imp64 = pc::compute_press_state_csr<double>(
            &params64, scratch64.get(), M_rowadr_i32.data(), M_rownnz_i32.data(),
            M_colind_i32.data(), d->M, imposed.data(), d->qfrc_bias);
        require(std::abs(ps_dense64.f_passive[0] - ps_csr_imp64.f_passive[0]) <= 1e-12 &&
                std::abs(ps_dense64.f_passive[1] - ps_csr_imp64.f_passive[1]) <= 1e-12 &&
                std::abs(ps_dense64.max_total_press_n - ps_csr_imp64.max_total_press_n) <= 1e-12,
                "compute_press_state_dense vs CSR <= 1e-12");

        // Verify compute_press_state(MulMFn) vs compute_press_state_csr
        MjMulMFn mulm_fn{.m = m, .d = d.get()};
        const auto ps_mulm_raw = pc::compute_press_state<double>(
            &params64, scratch64.get(), d->qacc, d->qfrc_bias, mulm_fn);
        const auto ps_csr_raw = pc::compute_press_state_csr<double>(
            &params64, scratch64.get(), M_rowadr_i32.data(), M_rownnz_i32.data(),
            M_colind_i32.data(), d->M, d->qacc, d->qfrc_bias);
        const double raw_scale = std::max({
            1.0, std::abs(ps_csr_raw.f_passive[0]), std::abs(ps_csr_raw.f_passive[1]),
            std::abs(ps_csr_raw.max_total_press_n)});
        require(std::abs(ps_mulm_raw.f_passive[0] - ps_csr_raw.f_passive[0]) <= 1e-13 * raw_scale,
                "raw qacc PressState MulM vs CSR f_passive[0] rel <= 1e-13");
        require(std::abs(ps_mulm_raw.f_passive[1] - ps_csr_raw.f_passive[1]) <= 1e-13 * raw_scale,
                "raw qacc PressState MulM vs CSR f_passive[1] rel <= 1e-13");
        require(std::abs(ps_mulm_raw.max_total_press_n - ps_csr_raw.max_total_press_n) <=
                    1e-13 * raw_scale,
                "raw qacc PressState MulM vs CSR max_total_press rel <= 1e-13");

        const auto ps_mulm = pc::compute_press_state<double>(
            &params64, scratch64.get(), imposed.data(), d->qfrc_bias, mulm_fn);
        const auto ps_csr = pc::compute_press_state_csr<double>(
            &params64, scratch64.get(), M_rowadr_i32.data(), M_rownnz_i32.data(),
            M_colind_i32.data(), d->M, imposed.data(), d->qfrc_bias);
        max_ps_mulm_err_f64 = std::max(
            max_ps_mulm_err_f64, std::abs(ps_mulm.f_passive[0] - ps_csr.f_passive[0]));
        max_ps_mulm_err_f64 = std::max(
            max_ps_mulm_err_f64, std::abs(ps_mulm.f_passive[1] - ps_csr.f_passive[1]));
        max_ps_mulm_err_f64 = std::max(
            max_ps_mulm_err_f64, std::abs(ps_mulm.max_total_press_n - ps_csr.max_total_press_n));
    }

    std::cout << "  leg_weights max|err|: FP64=" << max_fw_err_f64 << " FP32=" << max_fw_err_f32 << "\n"
              << "  CSR M*v max|err|:     FP64=" << max_csr_err_f64 << " FP32=" << max_csr_err_f32 << "\n"
              << "  PressState MulM vs CSR FP64 max|err|=" << max_ps_mulm_err_f64 << "\n";

    require(max_fw_err_f64 <= 1e-12, "leg_weights FP64 <= 1e-12");
    require(max_fw_err_f32 <= 1e-4, "leg_weights FP32 <= 1e-4");
    require(max_csr_err_f64 <= 1e-12, "CSR M*v FP64 <= 1e-12");
    require(max_csr_err_f32 <= 2e-5, "CSR M*v FP32 <= 2e-5");
    require(max_ps_mulm_err_f64 <= 1e-12, "compute_press_state MulM vs CSR <= 1e-12");
}

// ---- Captured state & multi-state M2_K2 parity -----------------------------

struct CapturedTestState {
    rider::RiderCommand cpu_cmd;
    bool steady_state = false;
    bool advance = true;
    bool pedal_recovery = true;
    std::optional<double> weld_crank_torque_nm;
    double initial_coast_hold_nm = 0.0;
    double min_elbow_flexion_rad = 15.0 * (std::numbers::pi / 180.0);
    int step_index = 0;
    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> qacc_warmstart;
    std::vector<double> ctrl;
    std::vector<double> qfrc_applied;
    std::vector<double> dof_frictionloss;
};

struct MultiStateCaptureBundle {
    ModelPtr model{nullptr, &delete_model};
    rider::SpindlePose pose;
    rider::SpindleConfig spindle_cfg;
    double crank_length_m = 0.165;
    int control_period_steps = 4;
    std::vector<CapturedTestState> states;
};

struct RideBootstrapSpec {
    std::string_view track_rel;
    double duration_s = 4.0;
    double initial_speed_mps = 0.0;
};

std::unique_ptr<runtime::RuntimeBootstrap> make_welded_ride_bootstrap(
    const RideBootstrapSpec &spec
) {
    auto owner = std::make_unique<bike::research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.physics->initial_speed_mps = spec.initial_speed_mps;
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider = bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::load_track(repo_file(spec.track_rel));
    request.experiment.duration_s = spec.duration_s;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = false;
    request.sensors = bike::research::SensorConfig{};
    request.experiment.control_period_s =
        request.setup.model.physics->research.control_period_s;
    request.experiment.actuator_delay_s =
        request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s =
        request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s =
        request.setup.model.physics->research.sensor_maximum_age_s;
    request.sensors.sample_period_s = 0.005;
    auto prepared = bike::research::prepare_environment(std::move(owner));
    return std::make_unique<runtime::RuntimeBootstrap>(std::move(prepared->bootstrap.ride));
}

struct CaptureOptions {
    double mean_tq_override = 0.0;
    bool use_override = false;
    bool crank_repos = false;
    double target_rate = 0.0;
    int step_idx = 0;
};

MultiStateCaptureBundle capture_diverse_states_from_runtime() {
    MultiStateCaptureBundle out;
    auto boot_flat = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/flat.toml"});
    out.control_period_steps = boot_flat->config.control_period_steps;
    out.crank_length_m = boot_flat->config.geometry.crank_length_m;
    out.pose = rider::decode_spindle_pose(boot_flat->config.geometry.pose, "config.geometry.pose");
    auto rt_flat = std::make_unique<runtime::NativeRideRuntime>(std::move(*boot_flat));
    boot_flat.reset();

    bike::validation::RuntimeRigAccess::inspect(
        *rt_flat, [&](const Stepper &stepper, const runtime::PhysicalStep &pstep) {
            mjModel *raw_copy = mj_copyModel(nullptr, stepper.model());
            if (raw_copy == nullptr) {
                throw std::runtime_error("mj_copyModel failed");
            }
            out.model = ModelPtr(raw_copy, &delete_model);
            out.spindle_cfg = pstep.rider_control().config();
        });

    const auto capture_from = [&](runtime::NativeRideRuntime &rt, const CaptureOptions &opts) {
        bike::validation::RuntimeRigAccess::inspect(
            rt, [&](const Stepper &stepper, const runtime::PhysicalStep &pstep) {
                const mjModel *m = stepper.model();
                const mjData *d = stepper.data();
                CapturedTestState st;
                const auto qpos_sp = model_access::readonly_buffer(d->qpos, m->nq, "qpos");
                const auto qvel_sp = model_access::readonly_buffer(d->qvel, m->nv, "qvel");
                const auto ws_sp = model_access::readonly_buffer(d->qacc_warmstart, m->nv, "warmstart");
                const auto ctrl_sp = model_access::readonly_buffer(d->ctrl, m->nu, "ctrl");
                const auto qfrc_sp = model_access::readonly_buffer(d->qfrc_applied, m->nv, "qfrc_applied");
                const auto fl_sp = model_access::readonly_buffer(m->dof_frictionloss, m->nv, "dof_frictionloss");

                st.qpos.assign(qpos_sp.begin(), qpos_sp.end());
                st.qvel.assign(qvel_sp.begin(), qvel_sp.end());
                st.qacc_warmstart.assign(ws_sp.begin(), ws_sp.end());
                st.ctrl.assign(ctrl_sp.begin(), ctrl_sp.end());
                st.qfrc_applied.assign(qfrc_sp.begin(), qfrc_sp.end());
                st.dof_frictionloss.assign(fl_sp.begin(), fl_sp.end());

                const auto &rc_state = pstep.rider_control().state();
                st.initial_coast_hold_nm = rc_state.coast_hold_nm.value_or(0.0);
                st.weld_crank_torque_nm = stepper.rider_contacts().state().delivered_crank_torque_nm;
                st.step_index = opts.step_idx;

                const auto &app = pstep.applied_control();
                st.cpu_cmd.enabled = app.rider_enabled;
                st.cpu_cmd.posture = app.posture.value_or(rider::RiderPosture{});
                st.cpu_cmd.crank_target_rate_rad_s =
                    app.crank_target_rate_rad_s.value_or(0.0);
                st.cpu_cmd.crank_reposition = opts.crank_repos;
                if (opts.crank_repos) {
                    st.cpu_cmd.mean_crank_torque_nm = 0.0;
                    st.cpu_cmd.crank_target_rate_rad_s = opts.target_rate;
                } else if (opts.use_override) {
                    st.cpu_cmd.mean_crank_torque_nm = opts.mean_tq_override;
                } else {
                    st.cpu_cmd.mean_crank_torque_nm = app.human_torque_nm.value_or(30.0);
                }
                out.states.push_back(std::move(st));
            });
    };

    runtime::RideControl ctrl_auto{};
    ctrl_auto.rider_enabled = true;

    runtime::RideControl ctrl_coast{};
    ctrl_coast.rider_enabled = true;
    ctrl_coast.human_torque_nm = 0.0;

    // 1. Flat launch & rolling pedaling across crank phases and handover (steps 200..1600)
    (void)rt_flat->advance(200, ctrl_auto, 0.0, 0.0, std::nullopt);
    for (int step = 220; step <= 1200; step += 28) {
        (void)rt_flat->advance(step, ctrl_auto, 0.0, 0.0, std::nullopt);
        capture_from(*rt_flat, {});
    }

    // 2. Explicit handover & coasting / reposition states on flat
    for (int step = 1220; step <= 1340; step += 8) {
        const bool do_coast = (step >= 1296);
        (void)rt_flat->advance(step, do_coast ? ctrl_coast : ctrl_auto, 0.0, 0.0, std::nullopt);
        if (step >= 1316) {
            capture_from(*rt_flat, {.use_override = true,
                                    .crank_repos = (step == 1332),
                                    .target_rate = (step == 1332) ? -3.0 : 0.0});
        } else {
            capture_from(*rt_flat, {});
        }
    }

    // 3. 15% climb states (launch + climb pedaling at auto/60 Nm)
    auto boot_climb = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/climb_15pct.toml"});
    auto rt_climb = std::make_unique<runtime::NativeRideRuntime>(std::move(*boot_climb));
    boot_climb.reset();
    runtime::RideControl ctrl_climb{};
    ctrl_climb.rider_enabled = true;
    ctrl_climb.human_torque_nm = 60.0;
    (void)rt_climb->advance(160, ctrl_climb, 0.0, 0.0, std::nullopt);
    for (int step = 180; step <= 800; step += 40) {
        (void)rt_climb->advance(step, ctrl_climb, 0.0, 0.0, std::nullopt);
        capture_from(*rt_climb, {.mean_tq_override = 60.0, .use_override = true});
    }

    // 4. M1 edge-mode states cloned from captured active rolling state:
    //    - cmd.enabled = false (!rider_enabled)
    //    - cmd.steady_state = true
    //    - cmd.advance = false (non-advancing query: must not mutate coast_hold_nm)
    if (!out.states.empty()) {
        const CapturedTestState base_edge = out.states[out.states.size() / 2u];
        {
            CapturedTestState dis = base_edge;
            dis.cpu_cmd.enabled = false;
            out.states.push_back(std::move(dis));
        }
        {
            CapturedTestState ss = base_edge;
            ss.steady_state = true;
            ss.initial_coast_hold_nm = 3.5;
            out.states.push_back(std::move(ss));
        }
        {
            CapturedTestState noadv = base_edge;
            noadv.advance = false;
            noadv.initial_coast_hold_nm = 4.25;
            out.states.push_back(std::move(noadv));
        }
    }

    return out;
}

double percentile_sorted(const std::vector<double> &sorted, double q) {
    if (sorted.empty()) {
        return 0.0;
    }
    const double idx = q * static_cast<double>(sorted.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(idx));
    const auto hi = static_cast<std::size_t>(std::ceil(idx));
    const double frac = idx - static_cast<double>(lo);
    return sorted[lo] + frac * (sorted[hi] - sorted[lo]);
}

void test_json_upload_welded_xml() {
    const ModelPtr m_w = load_xml(repo_file("tests/fixtures/model/welded.xml"));
    const DataPtr d_w(engine::make_data(m_w.get()), &delete_data);
    engine::reset_data(m_w.get(), d_w.get());
    engine::kinematics(m_w.get(), d_w.get());
    const auto str_path = repo_file("examples/research/rider_strength_reference.json");
    const auto env_path = repo_file("examples/research/rider_joint_envelope_anatomical.json");
    const auto str_doc = bio::parse_json(bt::text(str_path));
    const auto env_doc = bio::parse_json(bt::text(env_path));
    const auto sc_cfg = pc::make_viewer_welded_spindle_scalar_config(1);
    const auto p_json_ptr = std::make_unique<pc::SpindleM2K2Params<float>>(
        pc::build_spindle_m2k2_params_from_json_docs<float>(
            m_w.get(), d_w.get(), str_doc, env_doc, sc_cfg));
    const auto p_files_ptr = std::make_unique<pc::SpindleM2K2Params<float>>(
        pc::build_spindle_m2k2_params_from_json_files<float>(
            m_w.get(), d_w.get(), str_path, env_path, sc_cfg));
    const auto &p_json = *p_json_ptr;
    const auto &p_files = *p_files_ptr;
    require(p_json.nq == 27 && p_json.nv == 27 && p_json.nu == 12, "welded.xml dims");
    require(p_json.has_lean_limit != 0 && p_json.lean_limit_rad > 0.1f, "welded.xml lean_limit");
    require_near(static_cast<double>(p_json.thigh_m[0]), 0.4365, 1e-3, "welded.xml thigh_m");
    require_near(static_cast<double>(p_json.link_m[0]), 0.460, 0.15, "welded.xml link_m");
    require_bits_f32(p_files.lean_limit_rad, p_json.lean_limit_rad, "standalone JSON vs bridge lean_limit");
    require_bits_f32(p_files.thigh_m[0], p_json.thigh_m[0], "standalone JSON vs bridge thigh_m");
    require_bits_f32(p_files.strength[5].positive.torques_nm[1],
                     p_json.strength[5].positive.torques_nm[1],
                     "standalone JSON vs bridge strength");
}

void test_multi_state_m2k2_parity_and_metal(const MultiStateCaptureBundle &bundle) {
    std::cout << "[SP4-A] 4 & 6. Multi-state 1-step M2_K2 (kEps=0.1f) FP32 vs CPU M0 + Metal parity ("
              << bundle.states.size() << " states)...\n";
    require(bundle.states.size() >= 48, "captured >= 48 diverse states from NativeRideRuntime");
    test_json_upload_welded_xml();

    const mjModel *m_ref = bundle.model.get();
    const ModelPtr m_mut(mj_copyModel(nullptr, m_ref), &delete_model);
    mjModel *m = m_mut.get();

    const pc::SpindleUploadTiming timing{
        .crank_length_m = bundle.crank_length_m,
        .control_period_steps = bundle.control_period_steps,
    };
    const auto params64_ptr = std::make_unique<pc::SpindleM2K2Params<double>>(
        pc::build_spindle_m2k2_params_from_spindle_config<double>(
            m, bundle.pose, bundle.spindle_cfg, timing));
    const auto &params64 = *params64_ptr;
    const auto params32_ptr = std::make_unique<pc::SpindleM2K2Params<float>>(
        pc::build_spindle_m2k2_params_from_spindle_config<float>(
            m, bundle.pose, bundle.spindle_cfg, timing));
    const auto &params32 = *params32_ptr;

    auto ctrl_cpu = std::make_unique<rider::SpindleController>(
        m, bundle.pose, bundle.spindle_cfg, bundle.crank_length_m);
    DataPtr d_cpu(engine::make_data(m), &delete_data);
    DataPtr d_gpu(engine::make_data(m), &delete_data);

    const auto M_rowadr_i32 = to_i32_vec(m->M_rowadr, m->nv);
    const auto M_rownnz_i32 = to_i32_vec(m->M_rownnz, m->nv);
    const auto M_colind_i32 = to_i32_vec(m->M_colind, static_cast<int>(m->nC));
    const auto submass_f32 = to_real_vec<float>(m->body_subtreemass, m->nbody);

    std::vector<double> leg_tau_errs;
    std::vector<double> leg_tau_errs_vs_m2k2_f64;
    std::vector<double> leg_tau_errs_m2k2_f64_vs_m0;
    std::vector<double> upper_tau_errs;
    double max_min_lean_err = 0.0;
    double max_upper_target_err = 0.0;
    int handover_count = 0;

    auto scratch32 = std::make_unique<pc::SpindleM2K2StepScratch<float>>();

#if defined(__APPLE__)
    auto metal_req = std::make_unique<pc::test_sp4_metal::MetalSp4Request>();
    metal_req->physics_core_dir = BIKE_PHYSICS_CORE_SRC_DIR;
    metal_req->params = params32;
    metal_req->body_subtreemass = submass_f32;
    metal_req->M_rowadr = M_rowadr_i32;
    metal_req->M_rownnz = M_rownnz_i32;
    metal_req->M_colind = M_colind_i32;
    metal_req->nC = static_cast<pc::i32>(m->nC);
    metal_req->qM_row_stride = static_cast<pc::i32>(m->nv + 4);
    metal_req->leg_hip_x = -0.17590109f;
    metal_req->leg_hip_z = 0.78601288f;
    metal_req->leg_side = 0;
    for (int k = 0; k < 65; ++k) {
        const float ph = -static_cast<float>(std::numbers::pi) +
                         (2.0f * static_cast<float>(std::numbers::pi) * static_cast<float>(k)) / 64.0f;
        metal_req->leg_phases.push_back(ph);
    }
    struct HostStateRef {
        pc::u8 has_min_lean = 0;
        float min_lean_rad = 0.0f;
        std::array<float, pc::kNumUpperSpindleJoints> upper_targets{};
        std::array<float, 2> f_weight{};
        std::vector<float> csr_out;
        std::vector<float> ctrl_p0;
        std::vector<float> ctrl_p1;
        std::vector<float> ctrl_p2;
        std::vector<float> ctrl_p3;
        std::vector<float> ctrl_held;
        float final_coast_hold_nm = 0.0f;
    };
    std::vector<HostStateRef> host_refs;
#endif

    for (std::size_t si = 0; si < bundle.states.size(); ++si) {
        const auto &st = bundle.states[si];

        // 1. Authoritative CPU SpindleController (M0 FP64)
        mj_resetData(m, d_cpu.get());
        std::ranges::copy(st.qpos, model_access::mutable_buffer(d_cpu->qpos, m->nq, "qpos").begin());
        std::ranges::copy(st.qvel, model_access::mutable_buffer(d_cpu->qvel, m->nv, "qvel").begin());
        std::ranges::copy(st.qacc_warmstart,
                          model_access::mutable_buffer(d_cpu->qacc_warmstart, m->nv, "ws").begin());
        std::ranges::copy(st.ctrl, model_access::mutable_buffer(d_cpu->ctrl, m->nu, "ctrl").begin());
        std::ranges::copy(st.qfrc_applied,
                          model_access::mutable_buffer(d_cpu->qfrc_applied, m->nv, "qfrc").begin());
        std::ranges::copy(st.dof_frictionloss,
                          model_access::mutable_buffer(m->dof_frictionloss, m->nv, "fl").begin());
        engine::forward(m, d_cpu.get());

        // Count handover states
        {
            const auto xmat = model_access::readonly_buffer(d_cpu->xmat, m->nbody * 9, "xmat");
            const auto xpos = model_access::readonly_buffer(d_cpu->xpos, m->nbody * 3, "xpos");
            const auto xanchor = model_access::readonly_buffer(d_cpu->xanchor, m->njnt * 3, "xanchor");
            const std::size_t fb9 = to_idx(params64.frame_body) * 9;
            const std::size_t cj3 = to_idx(params64.crank_joint) * 3;
            const std::size_t pb3 = to_idx(params64.pedal_bodies[0]) * 3;
            const double dx = xpos[pb3 + 0] - xanchor[cj3 + 0];
            const double dy = xpos[pb3 + 1] - xanchor[cj3 + 1];
            const double dz = xpos[pb3 + 2] - xanchor[cj3 + 2];
            const double spx = xmat[fb9 + 0] * dx + xmat[fb9 + 3] * dy + xmat[fb9 + 6] * dz;
            const double spz = xmat[fb9 + 2] * dx + xmat[fb9 + 5] * dy + xmat[fb9 + 8] * dz;
            const double phase = std::atan2(-spz, spx);
            if (std::abs(std::cos(phase)) <= 0.25) {
                ++handover_count;
            }
        }

        auto cstate = ctrl_cpu->state();
        cstate.coast_hold_nm = st.initial_coast_hold_nm;
        ctrl_cpu->restore(cstate);

        const auto cpu_min_lean = ctrl_cpu->min_torso_lean_rad(d_cpu.get(), st.min_elbow_flexion_rad);
        const auto cpu_up = ctrl_cpu->upper_targets(d_cpu.get(), &st.cpu_cmd.posture);
        const auto cpu_torques = ctrl_cpu->compute(
            d_cpu.get(), st.cpu_cmd, /*advance=*/st.advance, /*dt_s=*/std::nullopt,
            st.steady_state, std::nullopt, st.pedal_recovery, st.weld_crank_torque_nm);
        ctrl_cpu->write(d_cpu.get(), cpu_torques);
        const double cpu_hold_after = ctrl_cpu->state().coast_hold_nm.value_or(0.0);

        std::array<double, pc::kNumSpindleJoints> cpu_ctrl{};
        {
            const auto d_cpu_ctrl = model_access::readonly_buffer(d_cpu->ctrl, m->nu, "ctrl");
            for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
                const int aid = std::span{params64.joint_actuator_id}[to_idx(i)];
                cpu_ctrl[to_idx(i)] = d_cpu_ctrl[to_idx(aid)];
            }
        }

        // 2. Host FP32 spindle_m2k2.hpp 4-pass M2_K2 (kEps = 0.1f)
        mj_resetData(m, d_gpu.get());
        std::ranges::copy(st.qpos, model_access::mutable_buffer(d_gpu->qpos, m->nq, "qpos").begin());
        std::ranges::copy(st.qvel, model_access::mutable_buffer(d_gpu->qvel, m->nv, "qvel").begin());
        std::ranges::copy(st.qacc_warmstart,
                          model_access::mutable_buffer(d_gpu->qacc_warmstart, m->nv, "ws").begin());
        std::ranges::copy(st.ctrl, model_access::mutable_buffer(d_gpu->ctrl, m->nu, "ctrl").begin());
        std::ranges::copy(st.qfrc_applied,
                          model_access::mutable_buffer(d_gpu->qfrc_applied, m->nv, "qfrc").begin());
        engine::forward(m, d_gpu.get());

        // Snapshot FP32 inputs (as GPU buffers would hold)
        const auto qpos_f32 = to_real_vec<float>(d_gpu->qpos, m->nq);
        const auto qvel_f32 = to_real_vec<float>(d_gpu->qvel, m->nv);
        const auto bias_f32 = to_real_vec<float>(d_gpu->qfrc_bias, m->nv);
        const auto xmat_f32 = to_real_vec<float>(d_gpu->xmat, m->nbody * 9);
        const auto xpos_f32 = to_real_vec<float>(d_gpu->xpos, m->nbody * 3);
        const auto xanc_f32 = to_real_vec<float>(d_gpu->xanchor, m->njnt * 3);
        const auto xax_f32 = to_real_vec<float>(d_gpu->xaxis, m->njnt * 3);
        const auto spos_f32 = to_real_vec<float>(d_gpu->site_xpos, m->nsite * 3);
        const auto scom_f32 = to_real_vec<float>(d_gpu->subtree_com, m->nbody * 3);
        const auto dM_f32 = to_real_vec<float>(d_gpu->M, static_cast<int>(m->nC));

        pc::SpindleCommand<float> cmd32{};
        cmd32.mean_crank_torque_nm = static_cast<float>(st.cpu_cmd.mean_crank_torque_nm);
        cmd32.torso_lean_rad = static_cast<float>(st.cpu_cmd.posture.torso_lean_rad);
        cmd32.crank_target_rate_rad_s = static_cast<float>(st.cpu_cmd.crank_target_rate_rad_s);
        cmd32.weld_crank_torque_nm = static_cast<float>(st.weld_crank_torque_nm.value_or(0.0));
        cmd32.has_weld_crank_torque = st.weld_crank_torque_nm.has_value() ? 1u : 0u;
        cmd32.enabled = st.cpu_cmd.enabled ? 1u : 0u;
        cmd32.crank_reposition = st.cpu_cmd.crank_reposition ? 1u : 0u;
        cmd32.steady_state = st.steady_state ? 1u : 0u;
        cmd32.advance = st.advance ? 1u : 0u;

        // Check min_torso_lean_rad and upper_targets parity
        pc::u8 has_ml = 0;
        float ml = 0.0f;
        pc::spindle_m2k2_min_torso_lean_rad<float, pc::GpuToolkit>(
            &params32, qpos_f32.data(), xpos_f32.data(), xmat_f32.data(), xanc_f32.data(),
            static_cast<float>(st.min_elbow_flexion_rad), has_ml, ml);
        if (cpu_min_lean.has_value()) {
            require(has_ml != 0, "has_min_lean parity");
            max_min_lean_err = std::max(
                max_min_lean_err, std::abs(static_cast<double>(ml) - *cpu_min_lean));
        }

        std::array<float, pc::kNumUpperSpindleJoints> up_t{};
        pc::u8 t_sat = 0;
        pc::u8 a_sat = 0;
        pc::upper_targets_core<float, pc::GpuToolkit>(
            &params32, qpos_f32.data(), xpos_f32.data(), xmat_f32.data(), xanc_f32.data(),
            cmd32.torso_lean_rad, up_t.data(), t_sat, a_sat);
        for (int k = 0; k < pc::kNumUpperSpindleJoints; ++k) {
            max_upper_target_err = std::max(
                max_upper_target_err,
                std::abs(static_cast<double>(up_t[to_idx(k)]) - cpu_up[to_idx(k)].second));
        }

        auto state32_ptr = std::make_unique<pc::SpindleM2K2State<float>>();
        auto &state32 = *state32_ptr;
        state32.coast_hold_nm = static_cast<float>(st.initial_coast_hold_nm);
        *scratch32 = pc::SpindleM2K2StepScratch<float>{};
        auto ctrl32 = to_real_vec<float>(d_gpu->ctrl, m->nu);

        const auto ws_before = std::vector<double>(
            model_access::readonly_buffer(d_gpu->qacc_warmstart, m->nv, "ws").begin(),
            model_access::readonly_buffer(d_gpu->qacc_warmstart, m->nv, "ws").end());

        const auto write_gpu_ctrl = [&](const std::vector<float> &cvec) {
            const auto dst = model_access::mutable_buffer(d_gpu->ctrl, m->nu, "ctrl");
            for (int a = 0; a < m->nu; ++a) {
                dst[to_idx(a)] = static_cast<double>(cvec[to_idx(a)]);
            }
        };

        // Pass 0 -> Solve 1
        pc::spindle_m2k2_pass0_init_and_base<float, pc::GpuToolkit>(
            &params32, cmd32, st.step_index,
            qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
            xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
            spos_f32.data(), scom_f32.data(), submass_f32.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &state32, scratch32.get(), ctrl32.data());
        const auto ctrl_p0_snap = ctrl32;
        write_gpu_ctrl(ctrl32);
        engine::forward_skip(m, d_gpu.get());
        const auto qacc_p0_f32 = to_real_vec<float>(d_gpu->qacc, m->nv);

        // Pass 1 -> Solve 2
        pc::spindle_m2k2_pass1_perturb0<float, pc::GpuToolkit>(
            &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p0_f32.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &state32, scratch32.get(), ctrl32.data());
        const auto ctrl_p1_snap = ctrl32;
        write_gpu_ctrl(ctrl32);
        engine::forward_skip(m, d_gpu.get());
        const auto qacc_p1_f32 = to_real_vec<float>(d_gpu->qacc, m->nv);

        // Pass 2 -> Solve 3
        pc::spindle_m2k2_pass2_perturb1<float, pc::GpuToolkit>(
            &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p1_f32.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &state32, scratch32.get(), ctrl32.data());
        const auto ctrl_p2_snap = ctrl32;
        write_gpu_ctrl(ctrl32);
        engine::forward_skip(m, d_gpu.get());
        const auto qacc_p2_f32 = to_real_vec<float>(d_gpu->qacc, m->nv);

        // Pass 3 -> Final commit
        pc::spindle_m2k2_pass3_commit<float, pc::GpuToolkit>(
            &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p2_f32.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &state32, scratch32.get(), ctrl32.data());

        // Verify ws_trace invariant: qacc_warmstart never mutated across passes
        const auto ws_after = model_access::readonly_buffer(d_gpu->qacc_warmstart, m->nv, "ws");
        for (int i = 0; i < m->nv; ++i) {
            require(std::bit_cast<std::uint64_t>(ws_after[to_idx(i)]) ==
                        std::bit_cast<std::uint64_t>(ws_before[to_idx(i)]),
                    "ws_trace invariant: qacc_warmstart untouched across passes");
        }

        for (int i = 0; i < pc::kNumUpperSpindleJoints; ++i) {
            const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
            upper_tau_errs.push_back(
                std::abs(static_cast<double>(ctrl32[to_idx(aid)]) - cpu_ctrl[to_idx(i)]));
        }
        // Verify FP64 M2_K2 (kEps = 1e-3) on the exact same d_gpu start state
        std::array<double, pc::kNumSpindleJoints> m2k2_f64_leg_ctrl{};
        {
            DataPtr d_64(engine::make_data(m), &delete_data);
            mj_resetData(m, d_64.get());
            std::ranges::copy(st.qpos, model_access::mutable_buffer(d_64->qpos, m->nq, "qpos").begin());
            std::ranges::copy(st.qvel, model_access::mutable_buffer(d_64->qvel, m->nv, "qvel").begin());
            std::ranges::copy(st.qacc_warmstart,
                              model_access::mutable_buffer(d_64->qacc_warmstart, m->nv, "ws").begin());
            std::ranges::copy(st.ctrl, model_access::mutable_buffer(d_64->ctrl, m->nu, "ctrl").begin());
            std::ranges::copy(st.qfrc_applied,
                              model_access::mutable_buffer(d_64->qfrc_applied, m->nv, "qfrc").begin());
            engine::forward(m, d_64.get());
            auto params64_m2 = std::make_unique<pc::SpindleM2K2Params<double>>(params64);
            params64_m2->newton_eps = 0.1;
            params64_m2->conv_eps = 1e-6;
            params64_m2->det_eps = 1e-6;
            params64_m2->ik_reach_eps = 1e-10;
            pc::SpindleCommand<double> cmd64{};
            cmd64.mean_crank_torque_nm = st.cpu_cmd.mean_crank_torque_nm;
            cmd64.torso_lean_rad = st.cpu_cmd.posture.torso_lean_rad;
            cmd64.crank_target_rate_rad_s = st.cpu_cmd.crank_target_rate_rad_s;
            cmd64.weld_crank_torque_nm = st.weld_crank_torque_nm.value_or(0.0);
            cmd64.has_weld_crank_torque = st.weld_crank_torque_nm.has_value() ? 1u : 0u;
            cmd64.enabled = st.cpu_cmd.enabled ? 1u : 0u;
            cmd64.crank_reposition = st.cpu_cmd.crank_reposition ? 1u : 0u;
            cmd64.steady_state = st.steady_state ? 1u : 0u;
            cmd64.advance = st.advance ? 1u : 0u;
            auto state64_ptr = std::make_unique<pc::SpindleM2K2State<double>>();
            auto &state64 = *state64_ptr;
            state64.coast_hold_nm = st.initial_coast_hold_nm;
            const auto scratch64_m2 = std::make_unique<pc::SpindleM2K2StepScratch<double>>();
            std::vector<double> ctrl64(
                model_access::readonly_buffer(d_64->ctrl, m->nu, "c").begin(),
                model_access::readonly_buffer(d_64->ctrl, m->nu, "c").end());
            const auto write_64 = [&]() {
                const auto dst = model_access::mutable_buffer(d_64->ctrl, m->nu, "c");
                for (int a = 0; a < m->nu; ++a) dst[to_idx(a)] = ctrl64[to_idx(a)];
            };
            pc::spindle_m2k2_pass0_init_and_base<double, pc::CpuExactToolkit, CpuExactTrig>(
                params64_m2.get(), cmd64, st.step_index, d_64->qpos, d_64->qvel, d_64->qfrc_bias,
                d_64->xmat, d_64->xpos, d_64->xanchor, d_64->xaxis, d_64->site_xpos, d_64->subtree_com,
                m->body_subtreemass, M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_64->M,
                &state64, scratch64_m2.get(), ctrl64.data());
            write_64(); engine::forward_skip(m, d_64.get());
            pc::spindle_m2k2_pass1_perturb0<double, pc::CpuExactToolkit, CpuExactTrig>(
                params64_m2.get(), d_64->qpos, d_64->qvel, d_64->qfrc_bias, d_64->qacc,
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_64->M,
                &state64, scratch64_m2.get(), ctrl64.data());
            write_64(); engine::forward_skip(m, d_64.get());
            pc::spindle_m2k2_pass2_perturb1<double, pc::CpuExactToolkit, CpuExactTrig>(
                params64_m2.get(), d_64->qpos, d_64->qvel, d_64->qfrc_bias, d_64->qacc,
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_64->M,
                &state64, scratch64_m2.get(), ctrl64.data());
            write_64(); engine::forward_skip(m, d_64.get());
            pc::spindle_m2k2_pass3_commit<double, pc::CpuExactToolkit, CpuExactTrig>(
                params64_m2.get(), d_64->qpos, d_64->qvel, d_64->qfrc_bias, d_64->qacc,
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), d_64->M,
                &state64, scratch64_m2.get(), ctrl64.data());
            for (int i = pc::kSpindleSlotHipFront; i < pc::kNumSpindleJoints; ++i) {
                const int aid = std::span{params64.joint_actuator_id}[to_idx(i)];
                m2k2_f64_leg_ctrl[to_idx(i)] = ctrl64[to_idx(aid)];
                leg_tau_errs_m2k2_f64_vs_m0.push_back(std::abs(ctrl64[to_idx(aid)] - cpu_ctrl[to_idx(i)]));
            }
        }

        if (!st.advance) {
            require(state32.coast_hold_nm == static_cast<float>(st.initial_coast_hold_nm),
                    "advance=0 must not mutate coast_hold_nm");
            require(std::abs(cpu_hold_after - st.initial_coast_hold_nm) <= 1e-12,
                    "CPU advance=0 must not mutate coast_hold_nm");
        }

        // Verify dense strided qM + nullptr subtreemass produces bit-identical pass0..3 ctrl on host
        const int nv_i = static_cast<int>(m->nv);
        const int dense_stride = nv_i + 4;
        std::vector<double> qM_full64(static_cast<std::size_t>(nv_i * nv_i), 0.0);
        mj_fullM(m, d_gpu.get(), qM_full64.data());
        std::vector<float> qM_dense32(static_cast<std::size_t>(nv_i * dense_stride), 77.0f);
        for (int r = 0; r < nv_i; ++r) {
            for (int c = 0; c < nv_i; ++c) {
                qM_dense32[to_idx(r * dense_stride + c)] =
                    static_cast<float>(qM_full64[to_idx(r * nv_i + c)]);
            }
        }
        {
            auto st_d_ptr = std::make_unique<pc::SpindleM2K2State<float>>();
            auto &st_d = *st_d_ptr;
            st_d.coast_hold_nm = static_cast<float>(st.initial_coast_hold_nm);
            const auto scr_d = std::make_unique<pc::SpindleM2K2StepScratch<float>>();
            auto ctrl_d = to_real_vec<float>(st.ctrl);
            pc::spindle_m2k2_pass0_init_and_base<float, pc::GpuToolkit>(
                &params32, cmd32, st.step_index,
                qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
                xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
                spos_f32.data(), scom_f32.data(), nullptr,
                dense_stride, nullptr, nullptr, nullptr, qM_dense32.data(),
                &st_d, scr_d.get(), ctrl_d.data());
            pc::spindle_m2k2_pass1_perturb0<float, pc::GpuToolkit>(
                &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p0_f32.data(),
                dense_stride, nullptr, nullptr, nullptr, qM_dense32.data(),
                &st_d, scr_d.get(), ctrl_d.data());
            pc::spindle_m2k2_pass2_perturb1<float, pc::GpuToolkit>(
                &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p1_f32.data(),
                dense_stride, nullptr, nullptr, nullptr, qM_dense32.data(),
                &st_d, scr_d.get(), ctrl_d.data());
            pc::spindle_m2k2_pass3_commit<float, pc::GpuToolkit>(
                &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p2_f32.data(),
                dense_stride, nullptr, nullptr, nullptr, qM_dense32.data(),
                &st_d, scr_d.get(), ctrl_d.data());
            for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
                const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
                require(std::abs(ctrl_d[to_idx(aid)] - ctrl32[to_idx(aid)]) <= 2e-5f,
                        "dense qM + nullptr submass pass0..3 matches CSR");
            }
        }

        for (int i = pc::kSpindleSlotHipFront; i < pc::kNumSpindleJoints; ++i) {
            const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
            const double err = std::abs(static_cast<double>(ctrl32[to_idx(aid)]) - cpu_ctrl[to_idx(i)]);
            const double err_m2 = std::abs(static_cast<double>(ctrl32[to_idx(aid)]) - m2k2_f64_leg_ctrl[to_idx(i)]);
            leg_tau_errs.push_back(err);
            leg_tau_errs_vs_m2k2_f64.push_back(err_m2);
        }

#if defined(__APPLE__)
        std::vector<float> csr_vec(static_cast<std::size_t>(m->nv), 0.0f);
        for (int i = 0; i < m->nv; ++i) {
            csr_vec[to_idx(i)] = std::sin(0.29f * static_cast<float>(i + 1) + 0.1f * static_cast<float>(si));
        }
        std::vector<float> host_csr_out(static_cast<std::size_t>(m->nv), 0.0f);
        pc::mul_m_csr<float>(
            static_cast<pc::i32>(m->nv), M_rowadr_i32.data(), M_rownnz_i32.data(),
            M_colind_i32.data(), dM_f32.data(), csr_vec.data(), host_csr_out.data());

        std::vector<float> host_ctrl_held(static_cast<std::size_t>(m->nu), 0.0f);
        pc::spindle_m2k2_write_held_ctrl<float, pc::GpuToolkit>(
            &params32, &state32, qpos_f32.data(), qvel_f32.data(),
            state32.command_enabled, host_ctrl_held.data());
        // Verify non-control substep replay (step_index = 1 with 4-step ControlClock) skips Newton & replays held_torques
        if (cmd32.enabled != 0 && cmd32.advance != 0 && cmd32.steady_state == 0) {
            auto params_4step = std::make_unique<pc::SpindleM2K2Params<float>>(params32);
            params_4step->control_period_steps = 4;
            auto state_sub_ptr = std::make_unique<pc::SpindleM2K2State<float>>(state32);
            auto &state_sub = *state_sub_ptr;
            auto scratch_sub = std::make_unique<pc::SpindleM2K2StepScratch<float>>(*scratch32);
            std::vector<float> sub_ctrl(static_cast<std::size_t>(m->nu), 0.0f);
            pc::spindle_m2k2_pass0_init_and_base<float, pc::GpuToolkit>(
                params_4step.get(), cmd32, /*step_index=*/1,
                qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
                xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
                spos_f32.data(), scom_f32.data(), submass_f32.data(),
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
                &state_sub, scratch_sub.get(), sub_ctrl.data());
            require(scratch_sub->active_newton == 0 && scratch_sub->converged == 1,
                    "4-step ControlClock substep skips Newton");
            for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
                const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
                if (aid >= 0) {
                    require(sub_ctrl[to_idx(aid)] == host_ctrl_held[to_idx(aid)],
                            "4-step ControlClock substep matches write_held_ctrl");
                }
            }
        }

        pc::test_sp4_metal::Sp4StateSnapshot snap;
        snap.cmd = cmd32;
        snap.step_index = st.step_index;
        snap.min_elbow_flexion_rad = static_cast<float>(st.min_elbow_flexion_rad);
        snap.initial_coast_hold_nm = static_cast<float>(st.initial_coast_hold_nm);
        snap.qpos = qpos_f32;
        snap.qvel = qvel_f32;
        snap.qfrc_bias = bias_f32;
        snap.body_xmat = xmat_f32;
        snap.body_xpos = xpos_f32;
        snap.jnt_xanchor = xanc_f32;
        snap.jnt_xaxis = xax_f32;
        snap.site_xpos = spos_f32;
        snap.subtree_com = scom_f32;
        snap.d_M = dM_f32;
        snap.qM_dense = std::move(qM_dense32);
        snap.csr_vec = csr_vec;
        snap.qacc_p0 = qacc_p0_f32;
        snap.qacc_p1 = qacc_p1_f32;
        snap.qacc_p2 = qacc_p2_f32;
        metal_req->states.push_back(std::move(snap));

        HostStateRef href;
        href.has_min_lean = has_ml;
        href.min_lean_rad = ml;
        href.upper_targets = up_t;
        href.f_weight[0] = scratch32->f_weight[0];
        href.f_weight[1] = scratch32->f_weight[1];
        href.csr_out = std::move(host_csr_out);
        href.ctrl_p0 = ctrl_p0_snap;
        href.ctrl_p1 = ctrl_p1_snap;
        href.ctrl_p2 = ctrl_p2_snap;
        href.ctrl_p3 = ctrl32;
        href.ctrl_held = std::move(host_ctrl_held);
        href.final_coast_hold_nm = state32.coast_hold_nm;
        host_refs.push_back(std::move(href));
#endif
    }

    std::ranges::sort(leg_tau_errs);
    std::ranges::sort(leg_tau_errs_vs_m2k2_f64);
    std::ranges::sort(leg_tau_errs_m2k2_f64_vs_m0);
    std::ranges::sort(upper_tau_errs);
    const double f32_f64_p50 = percentile_sorted(leg_tau_errs_vs_m2k2_f64, 0.50);
    const double f32_f64_p95 = percentile_sorted(leg_tau_errs_vs_m2k2_f64, 0.95);
    const double f32_f64_max = leg_tau_errs_vs_m2k2_f64.back();
    const double f64_m0_p50 = percentile_sorted(leg_tau_errs_m2k2_f64_vs_m0, 0.50);
    const double f64_m0_p95 = percentile_sorted(leg_tau_errs_m2k2_f64_vs_m0, 0.95);
    const double f64_m0_max = leg_tau_errs_m2k2_f64_vs_m0.back();
    const double leg_p50 = percentile_sorted(leg_tau_errs, 0.50);
    const double leg_p95 = percentile_sorted(leg_tau_errs, 0.95);
    const double leg_max = leg_tau_errs.back();
    const double up_max = upper_tau_errs.back();

    std::cout << "  Handover states=" << handover_count
              << " | M2K2_F32 vs M2K2_F64 |dTau_leg| p50=" << f32_f64_p50
              << " p95=" << f32_f64_p95
              << " max=" << f32_f64_max << "\n"
              << "  M2K2_F32 vs M0 |dTau_leg| p50=" << leg_p50
              << " p95=" << leg_p95
              << " max=" << leg_max
              << " (M2K2_F64 vs M0: " << f64_m0_p50 << "/" << f64_m0_p95 << "/" << f64_m0_max << ")"
              << " | max|dTau_upper|=" << up_max
              << " | max|dMinLean|=" << max_min_lean_err
              << " | max|dUpperTgt|=" << max_upper_target_err << "\n";

    require(handover_count >= 2, "captured handover states");
    require(f32_f64_p50 <= 0.10, "M2_K2 FP32 vs FP64 |dTau_leg| p50 <= 0.10 Nm");
    require(f32_f64_p95 <= 0.50, "M2_K2 FP32 vs FP64 |dTau_leg| p95 <= 0.50 Nm");
    require(f32_f64_max <= 2.00, "M2_K2 FP32 vs FP64 |dTau_leg| max <= 2.00 Nm");
    require(leg_p50 <= 0.10, "M2_K2 FP32 vs M0 |dTau_leg| p50 <= 0.10 Nm");
    require(up_max <= 0.25, "M2_K2 FP32 |dTau_upper| max <= 0.25 Nm");
    require(max_min_lean_err <= 1e-4, "min_torso_lean_rad parity <= 1e-4 rad");
    require(max_upper_target_err <= 1e-4, "upper_targets parity <= 1e-4 rad");

#if defined(__APPLE__)
    const auto metal_out = pc::test_sp4_metal::run_metal_sp4_parity(*metal_req);
    require(metal_out->device_available, "Metal device available on Apple");
    if (!metal_out->error_message.empty()) {
        throw std::runtime_error("Metal SP4 parity failed: " + metal_out->error_message);
    }

    float max_metal_leg_diff = 0.0f;
    for (std::size_t k = 0; k < metal_req->leg_phases.size(); ++k) {
        const auto host_lk = pc::analytic_leg_kinematics<float, pc::GpuToolkit>(
            params32.thigh_m[0], params32.link_m[0], params32.crank_length_m,
            params32.side_sign[0], params32.knee_sign[0],
            metal_req->leg_hip_x, metal_req->leg_hip_z, metal_req->leg_phases[k],
            params32.det_eps, params32.ik_reach_eps * 10.0f);
        const auto &mtl_lk = metal_out->leg_kin_out[k];
        require(mtl_lk.valid == host_lk.valid, "Metal leg valid parity");
        for (int i = 0; i < 2; ++i) {
            const std::size_t iu = to_idx(i);
            max_metal_leg_diff = std::max(
                max_metal_leg_diff, std::abs(std::span{mtl_lk.jac}[iu] - std::span{host_lk.jac}[iu]));
            max_metal_leg_diff = std::max(
                max_metal_leg_diff, std::abs(std::span{mtl_lk.djac}[iu] - std::span{host_lk.djac}[iu]));
            max_metal_leg_diff = std::max(
                max_metal_leg_diff,
                std::abs(std::span{mtl_lk.gamma_z}[iu] - std::span{host_lk.gamma_z}[iu]));
            max_metal_leg_diff = std::max(
                max_metal_leg_diff,
                std::abs(std::span{mtl_lk.gamma_x}[iu] - std::span{host_lk.gamma_x}[iu]));
        }
    }

    float max_metal_ctrl_diff = 0.0f;
    float max_metal_csr_diff = 0.0f;
    float max_metal_fw_diff = 0.0f;
    float max_metal_lean_diff = 0.0f;
    float max_metal_hold_diff = 0.0f;
    for (std::size_t w = 0; w < host_refs.size(); ++w) {
        const auto &hr = host_refs[w];
        const auto &mr = metal_out->state_out[w];
        require(mr.has_min_lean == hr.has_min_lean, "Metal has_min_lean parity");
        max_metal_lean_diff = std::max(max_metal_lean_diff, std::abs(mr.min_lean_rad - hr.min_lean_rad));
        max_metal_fw_diff = std::max(max_metal_fw_diff, std::abs(mr.f_weight[0] - hr.f_weight[0]));
        max_metal_fw_diff = std::max(max_metal_fw_diff, std::abs(mr.f_weight[1] - hr.f_weight[1]));
        max_metal_hold_diff = std::max(
            max_metal_hold_diff,
            std::abs(mr.final_state.coast_hold_nm - hr.final_coast_hold_nm));
        for (int k = 0; k < m->nv; ++k) {
            max_metal_csr_diff = std::max(
                max_metal_csr_diff, std::abs(mr.csr_out[to_idx(k)] - hr.csr_out[to_idx(k)]));
        }
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int k = std::span{params32.joint_actuator_id}[to_idx(i)];
            max_metal_ctrl_diff = std::max(
                max_metal_ctrl_diff, std::abs(mr.ctrl_pass0[to_idx(k)] - hr.ctrl_p0[to_idx(k)]));
            max_metal_ctrl_diff = std::max(
                max_metal_ctrl_diff, std::abs(mr.ctrl_pass1[to_idx(k)] - hr.ctrl_p1[to_idx(k)]));
            max_metal_ctrl_diff = std::max(
                max_metal_ctrl_diff, std::abs(mr.ctrl_pass2[to_idx(k)] - hr.ctrl_p2[to_idx(k)]));
            max_metal_ctrl_diff = std::max(
                max_metal_ctrl_diff, std::abs(mr.ctrl_pass3[to_idx(k)] - hr.ctrl_p3[to_idx(k)]));
            max_metal_ctrl_diff = std::max(
                max_metal_ctrl_diff, std::abs(mr.ctrl_held[to_idx(k)] - hr.ctrl_held[to_idx(k)]));
        }
        require_bits_f32(mr.f_weight[0], hr.f_weight[0], "Metal f_weight[0]");
        require_bits_f32(mr.f_weight[1], hr.f_weight[1], "Metal f_weight[1]");
        require_bits_f32(mr.f_weight_null_subm[0], hr.f_weight[0], "Metal nullptr submass[0]");
        require_bits_f32(mr.f_weight_null_subm[1], hr.f_weight[1], "Metal nullptr submass[1]");
        for (int k = 0; k < m->nv; ++k) {
            require(std::abs(mr.dense_out[to_idx(k)] - mr.csr_out[to_idx(k)]) <= 2e-5f,
                    "Metal mul_m_dense vs mul_m_csr <= 2e-5");
        }
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int k = std::span{params32.joint_actuator_id}[to_idx(i)];
            require(std::abs(mr.ctrl_dense_pass3[to_idx(k)] - mr.ctrl_pass3[to_idx(k)]) <= 5e-5f,
                    "Metal dense qM pass0..3 ctrl parity");
        }
    }

    std::cout << "  Metal (" << metal_out->device_name << ", MSL 2.4/3.0/3.1 safe):"
              << " leg_diff=" << max_metal_leg_diff
              << " csr_diff=" << max_metal_csr_diff
              << " fw_diff=" << max_metal_fw_diff
              << " lean_diff=" << max_metal_lean_diff
              << " ctrl_diff=" << max_metal_ctrl_diff
              << " hold_diff=" << max_metal_hold_diff << "\n";

    require(max_metal_leg_diff <= 1e-5f, "Metal leg kinematics parity <= 1e-5");
    require(max_metal_csr_diff <= 1e-5f, "Metal CSR M*v parity <= 1e-5");
    require(max_metal_fw_diff <= 1e-5f, "Metal f_weight parity <= 1e-5");
    require(max_metal_lean_diff <= 1e-5f, "Metal min_torso_lean parity <= 1e-5");
    require(max_metal_ctrl_diff <= 1e-5f, "Metal M2_K2 4-pass ctrl parity <= 1e-5");
    require(max_metal_hold_diff <= 1e-5f, "Metal coast_hold_nm parity <= 1e-5");
#endif
}

// ---- Test 5: Closed-loop CPU MuJoCo multi-step ride sanity check -----------

void test_closed_loop_cpu_multistep_ride() {
    std::cout << "[SP4-A] 5. Closed-loop CPU MuJoCo multi-step ride sanity check (>= 480 steps = 0.6 s)...\n";

    auto boot_a = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/flat.toml"});
    auto boot_b = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/flat.toml"});
    auto boot_c = make_welded_ride_bootstrap({.track_rel = "tests/fixtures/tracks/flat.toml"});

    // Use the bootstrap's authoritative ControlClock period (viewer_physics_welded.toml)
    const int control_period_steps = boot_a->config.control_period_steps;
    const double crank_length_m = boot_a->config.geometry.crank_length_m;
    const rider::SpindlePose pose = rider::decode_spindle_pose(
        boot_a->config.geometry.pose, "config.geometry.pose");

    auto rt_seed = std::make_unique<runtime::NativeRideRuntime>(std::move(*boot_a));
    boot_a.reset();
    // Advance rt_seed 800 steps (1.0 s) so LaunchPlan finishes brake release and the bike is rolling smoothly
    runtime::RideControl ctrl_init{};
    ctrl_init.rider_enabled = true;
    (void)rt_seed->advance(800, ctrl_init, 0.0, 0.0, std::nullopt);

    // Snapshot the exact rolling state at step 800 for both M0 (Stepper A) and M2_K2 (Stepper B)
    std::vector<double> rolling_state;
    std::vector<double> ctrl_snap;
    std::vector<double> fl_snap;
    std::vector<double> tire_state_snap;
    std::vector<std::string> tire_names_snap;
    auto drive_snap = std::make_unique<drivetrain::DriveSnapshot>();
    auto rc_state_snap = std::make_unique<writers::RiderContactsState>();
    std::optional<writers::RiderContactsProbe> rc_probe_snap;
    double coast_hold_snap = 0.0;
    double delivered_crank_snap = 0.0;
    auto spindle_cfg = std::make_unique<rider::SpindleConfig>();

    bike::validation::RuntimeRigAccess::inspect(
        *rt_seed, [&](const Stepper &stepper, const runtime::PhysicalStep &pstep) {
            const mjModel *m = stepper.model();
            const mjData *d = stepper.data();
            const auto sz = engine::state_size(m, mjSTATE_INTEGRATION);
            rolling_state.resize(static_cast<std::size_t>(sz));
            engine::get_state(m, d, rolling_state.data(), mjSTATE_INTEGRATION);
            const auto ctrl_ro = model_access::readonly_buffer(d->ctrl, m->nu, "ctrl");
            ctrl_snap.assign(ctrl_ro.begin(), ctrl_ro.end());
            const auto fl_sp = model_access::readonly_buffer(m->dof_frictionloss, m->nv, "fl");
            fl_snap.assign(fl_sp.begin(), fl_sp.end());
            tire_names_snap = stepper.tire_writer().state_names();
            tire_state_snap = stepper.tire_writer().state();
            *drive_snap = stepper.drive().state();
            *rc_state_snap = stepper.rider_contacts().state();
            rc_probe_snap = stepper.rider_contacts().probe();
            coast_hold_snap = pstep.rider_control().state().coast_hold_nm.value_or(0.0);
            delivered_crank_snap = stepper.rider_contacts().state().delivered_crank_torque_nm;
            *spindle_cfg = pstep.rider_control().config();
        });
    rt_seed.reset();

    const auto wcfg_a = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(boot_b->config.writer_config));
    const auto wcfg_b = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(boot_c->config.writer_config));
    auto stepper_a = std::make_unique<Stepper>(boot_b->model.mjb, *wcfg_a);
    auto stepper_b = std::make_unique<Stepper>(boot_c->model.mjb, *wcfg_b);
    boot_b.reset();
    boot_c.reset();

    const auto init_stepper_from_snap = [&](Stepper &stp) {
        const mjModel *m = stp.model();
        mjData *d = stp.data();
        std::ranges::copy(fl_snap, model_access::mutable_buffer(m->dof_frictionloss, m->nv, "fl").begin());
        engine::set_state(m, d, rolling_state.data(), mjSTATE_INTEGRATION);
        std::ranges::copy(ctrl_snap, model_access::mutable_buffer(d->ctrl, m->nu, "ctrl").begin());
        stp.tire_writer().set_state(tire_names_snap, tire_state_snap);
        stp.drive().restore(*drive_snap);
        stp.rider_contacts().assign_state(
            writers::RiderContactsState(*rc_state_snap),
            std::optional<writers::RiderContactsProbe>(rc_probe_snap));
        engine::forward(m, d);
    };
    init_stepper_from_snap(*stepper_a);
    init_stepper_from_snap(*stepper_b);

    mjModel *ma = stepper_a->model();
    mjData *da = stepper_a->data();
    mjModel *mb = stepper_b->model();
    mjData *db = stepper_b->data();

    auto ctrl_a = std::make_unique<rider::SpindleController>(
        ma, pose, *spindle_cfg, crank_length_m);
    {
        auto cstate = ctrl_a->state();
        cstate.coast_hold_nm = coast_hold_snap;
        ctrl_a->restore(cstate);
    }

    const auto params32_ptr = std::make_unique<pc::SpindleM2K2Params<float>>(
        pc::build_spindle_m2k2_params_from_spindle_config<float>(
            mb, pose, *spindle_cfg,
            pc::SpindleUploadTiming{
                .crank_length_m = crank_length_m,
                .control_period_steps = control_period_steps,
            }));
    const auto &params32 = *params32_ptr;
    const auto M_rowadr_i32 = to_i32_vec(mb->M_rowadr, mb->nv);
    const auto M_rownnz_i32 = to_i32_vec(mb->M_rownnz, mb->nv);
    const auto M_colind_i32 = to_i32_vec(mb->M_colind, static_cast<int>(mb->nC));
    const auto submass_f32 = to_real_vec<float>(mb->body_subtreemass, mb->nbody);

    auto env_spec_ptr = std::make_unique<pc::RiderJointEnvelopeSpec<double>>();
    auto &env_spec = *env_spec_ptr;
    env_spec.stiffness_nm_rad = spindle_cfg->joint_envelope_soft_k_nm_rad;
    env_spec.margin_rad = spindle_cfg->joint_envelope_soft_margin_rad;
    {
        const auto jnt_limited = model_access::readonly_buffer(mb->jnt_limited, mb->njnt, "jnt_limited");
        const auto jnt_range = model_access::readonly_buffer(mb->jnt_range, mb->njnt * 2, "jnt_range");
        const auto qadr = model_access::readonly_buffer(mb->jnt_qposadr, mb->njnt, "jnt_qposadr");
        const auto dadr = model_access::readonly_buffer(mb->jnt_dofadr, mb->njnt, "jnt_dofadr");
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int jid = mj_name2id(mb, mjOBJ_JOINT, pc::kSpindleJointNames[to_idx(i)]);
            if (jid >= 0 && jnt_limited[to_idx(jid)] != 0) {
                const int idx = env_spec.count++;
                std::span{env_spec.qpos_adrs}[to_idx(idx)] = qadr[to_idx(jid)];
                std::span{env_spec.dof_adrs}[to_idx(idx)] = dadr[to_idx(jid)];
                std::span{env_spec.lower}[to_idx(idx)] = jnt_range[to_idx(jid) * 2 + 0];
                std::span{env_spec.upper}[to_idx(idx)] = jnt_range[to_idx(jid) * 2 + 1];
            }
        }
    }

    pc::SpindleM2K2State<float> sp_state{};
    sp_state.coast_hold_nm = static_cast<float>(coast_hold_snap);
    const auto sp_scratch = std::make_unique<pc::SpindleM2K2StepScratch<float>>();

    // Run both rollouts for 480 steps (0.6 s) with identical explicit human torque & posture
    constexpr int kRolloutSteps = 480;
    const double dt = mb->opt.timestep;
    const double human_torque_nm = 30.0;
    rider::RiderPosture target_posture{};
    target_posture.torso_lean_rad = 0.05;

    const int root_x_qpos = model_access::readonly_buffer(mb->jnt_qposadr, mb->njnt, "qadr")
        [to_idx(mj_name2id(mb, mjOBJ_JOINT, "root_x"))];
    const int root_x_dof = model_access::readonly_buffer(mb->jnt_dofadr, mb->njnt, "dadr")
        [to_idx(mj_name2id(mb, mjOBJ_JOINT, "root_x"))];
    const double start_x =
        model_access::readonly_buffer(db->qpos, mb->nq, "qpos")[to_idx(root_x_qpos)];

    double min_pedal_n = 1e9;
    double min_saddle_n = 1e9;
    double max_saddle_n = -1e9;
    double m0_min_pedal_n = 1e9;
    double prev_delivered_crank_a = delivered_crank_snap;
    double prev_delivered_crank_b = delivered_crank_snap;

    std::array<ForceComponentView, 8> susp_views{};
    std::array<ForceComponentView, 2> res_views{};
    std::vector<double> tire_qfrc_buf(static_cast<std::size_t>(mb->nv), 0.0);
    std::vector<double> rc_qfrc_buf(static_cast<std::size_t>(mb->nv), 0.0);
    std::vector<double> env_qfrc_buf(static_cast<std::size_t>(mb->nv), 0.0);
    std::vector<double> total_qfrc(static_cast<std::size_t>(mb->nv), 0.0);
    std::vector<double> q_start_a(static_cast<std::size_t>(ma->nq), 0.0);
    std::vector<double> v_start_a(static_cast<std::size_t>(ma->nv), 0.0);
    std::vector<double> q_start_b(static_cast<std::size_t>(mb->nq), 0.0);
    std::vector<double> v_start_b(static_cast<std::size_t>(mb->nv), 0.0);
    rider::NamedEntries<double> held_torques_a;

    struct EnvStepRates {
        double effort_nm = 0.0;
        double target_rate_rad_s = 0.0;
    };

    const auto assemble_env_forces = [&](Stepper &stp, double prev_crank_sensed) -> EnvStepRates {
        const mjModel *m = stp.model();
        mjData *d = stp.data();
        std::ranges::fill(model_access::mutable_buffer(d->qfrc_applied, m->nv, "qfrc"), 0.0);
        std::ranges::fill(model_access::mutable_buffer(d->xfrc_applied, m->nbody * 6, "xfrc"), 0.0);
        std::ranges::fill(total_qfrc, 0.0);

        const drivetrain::RideControl d_ctrl{
            .motor_torque_nm = std::nullopt,
            .motor_limit_nm = std::nullopt,
            .human_torque_nm = human_torque_nm,
            .rider_enabled = true,
            .crank_reposition = false,
        };
        const auto ped = stp.drive().prepare(
            d_ctrl, dt, /*braking=*/false, /*active=*/true, /*advance=*/true,
            /*contact=*/true, /*slip=*/std::nullopt, /*ceiling=*/std::nullopt,
            /*shift_blocked=*/false, drivetrain::RiderShift{}, /*cut_after_ceiling=*/false);
        const EnvStepRates rates{
            .effort_nm = ped.effort_nm,
            .target_rate_rad_s = ped.target_rate_rad_s,
        };

        const auto susp_names = stp.suspension().component_names();
        stp.suspension().components_into(d, std::span{susp_views}.first(susp_names.size()));
        for (std::size_t i = 0; i < susp_names.size(); ++i) {
            for (int k = 0; k < m->nv; ++k) {
                total_qfrc[to_idx(k)] += susp_views[i].values[to_idx(k)];
            }
        }

        stp.tire_writer().compute_into(d, dt, tire_qfrc_buf);
        for (int k = 0; k < m->nv; ++k) {
            total_qfrc[to_idx(k)] += tire_qfrc_buf[to_idx(k)];
        }

        const bool committed = stp.tire_writer().snapshots_committed();
        const auto snaps = stp.tire_writer().snapshot_storage();
        const auto r_names = stp.resistance().component_names();
        std::array<std::vector<double>, 2> side_loads;
        std::array<std::array<bool, 64>, 2> side_work{};
        std::array<TireSideInput, 2> side_in{};
        for (std::size_t si = 0; si < 2; ++si) {
            if (committed) {
                const auto &patches = snaps[si].patches;
                const std::size_t np = std::min(patches.size(), side_work[si].size());
                side_loads[si].resize(np);
                for (std::size_t pi = 0; pi < np; ++pi) {
                    side_loads[si][pi] = patches[pi].normal_load_n;
                    side_work[si][pi] = patches[pi].working_surface;
                }
                side_in[si] = TireSideInput{
                    .patch_loads = side_loads[si],
                    .patch_working = std::span{side_work[si]}.first(np),
                    .effective_radius_m = snaps[si].effective_radius_m,
                };
            }
        }
        stp.resistance().components_into(
            d, side_in[0], side_in[1], std::span{res_views}.first(r_names.size()));
        for (std::size_t i = 0; i < r_names.size(); ++i) {
            for (int k = 0; k < m->nv; ++k) {
                total_qfrc[to_idx(k)] += res_views[i].values[to_idx(k)];
            }
        }

        stp.rider_contacts().compute_qfrc_into(d, dt, true, true, rc_qfrc_buf);
        for (int k = 0; k < m->nv; ++k) {
            total_qfrc[to_idx(k)] += rc_qfrc_buf[to_idx(k)];
        }

        const drivetrain::TickInputs t_in{
            .control = d_ctrl,
            .dt = dt,
            .speed = model_access::readonly_buffer(d->qvel, m->nv, "qvel")[to_idx(root_x_dof)],
            .sensed = prev_crank_sensed,
            .braking = false,
            .active = true,
            .advance = true,
            .contact = true,
            .motor_braking = false,
            .shift_blocked = false,
            .pedaling = ped,
            .slip = std::nullopt,
        };
        const auto tick = stp.drive().stage_components(t_in);
        for (const auto &comp : tick.components) {
            for (int k = 0; k < m->nv; ++k) {
                total_qfrc[to_idx(k)] += comp.values[to_idx(k)];
            }
        }
        stp.drive().commit(tick);

        std::ranges::fill(env_qfrc_buf, 0.0);
        double env_j = 0.0;
        pc::ErrCode env_err = pc::ErrCode::none;
        const auto env_st = pc::eval_rider_joint_envelope<double, pc::CpuExactToolkit>(
            &env_spec, static_cast<pc::i32>(m->nq), static_cast<pc::i32>(m->nv),
            d->qpos, env_qfrc_buf.data(), env_j, env_err);
        require(env_st == pc::Status::ok, "eval_rider_joint_envelope ok");
        for (int k = 0; k < m->nv; ++k) {
            total_qfrc[to_idx(k)] += env_qfrc_buf[to_idx(k)];
        }
        std::ranges::copy(total_qfrc, model_access::mutable_buffer(d->qfrc_applied, m->nv, "qfrc").begin());
        return rates;
    };

    for (int step = 0; step < kRolloutSteps; ++step) {
        // ---- Stepper A: M0 reference controller ----
        {
            engine::forward(ma, da);
            const auto rates_a = assemble_env_forces(*stepper_a, prev_delivered_crank_a);
            engine::forward(ma, da);
            if (step % control_period_steps == 0) {
                rider::RiderCommand rcmd{};
                rcmd.mean_crank_torque_nm = rates_a.effort_nm;
                rcmd.enabled = true;
                rcmd.posture = target_posture;
                rcmd.crank_target_rate_rad_s = rates_a.target_rate_rad_s;
                held_torques_a = ctrl_a->compute(
                    da, rcmd, /*advance=*/true,
                    /*dt_s=*/static_cast<double>(control_period_steps) * dt,
                    /*steady_state=*/false, /*support_available=*/std::nullopt,
                    /*pedal_recovery=*/false, prev_delivered_crank_a);
            }
            ctrl_a->write(da, held_torques_a);

            const auto q_ro = model_access::readonly_buffer(da->qpos, ma->nq, "qa");
            const auto v_ro = model_access::readonly_buffer(da->qvel, ma->nv, "va");
            std::ranges::copy(q_ro, q_start_a.begin());
            std::ranges::copy(v_ro, v_start_a.begin());
            const auto prep_a = stepper_a->rider_contacts().prepare_attachment_raw(da);
            engine::step(ma, da);
            const auto settled_a = stepper_a->rider_contacts().settle_welds(
                da, writers::RiderIntervalState{q_start_a, v_start_a}, true, &prep_a);
            (void)stepper_a->drive().settle();
            prev_delivered_crank_a = stepper_a->rider_contacts().state().delivered_crank_torque_nm;
            if (settled_a.front_pedal_reaction.has_value()) {
                m0_min_pedal_n = std::min(m0_min_pedal_n, settled_a.front_pedal_reaction->normal_n);
            }
            if (settled_a.rear_pedal_reaction.has_value()) {
                m0_min_pedal_n = std::min(m0_min_pedal_n, settled_a.rear_pedal_reaction->normal_n);
            }
        }

        // ---- Stepper B: spindle_m2k2.hpp M2_K2 FP32 controller ----
        engine::forward(mb, db);
        const auto rates_b = assemble_env_forces(*stepper_b, prev_delivered_crank_b);
        engine::forward(mb, db);

        const auto qpos_f32 = to_real_vec<float>(db->qpos, mb->nq);
        const auto qvel_f32 = to_real_vec<float>(db->qvel, mb->nv);
        const auto bias_f32 = to_real_vec<float>(db->qfrc_bias, mb->nv);
        const auto xmat_f32 = to_real_vec<float>(db->xmat, mb->nbody * 9);
        const auto xpos_f32 = to_real_vec<float>(db->xpos, mb->nbody * 3);
        const auto xanc_f32 = to_real_vec<float>(db->xanchor, mb->njnt * 3);
        const auto xax_f32 = to_real_vec<float>(db->xaxis, mb->njnt * 3);
        const auto spos_f32 = to_real_vec<float>(db->site_xpos, mb->nsite * 3);
        const auto scom_f32 = to_real_vec<float>(db->subtree_com, mb->nbody * 3);
        const auto dM_f32 = to_real_vec<float>(db->M, static_cast<int>(mb->nC));
        auto ctrl32 = to_real_vec<float>(db->ctrl, mb->nu);

        pc::SpindleCommand<float> cmd32{};
        cmd32.mean_crank_torque_nm = static_cast<float>(rates_b.effort_nm);
        cmd32.torso_lean_rad = static_cast<float>(target_posture.torso_lean_rad);
        cmd32.crank_target_rate_rad_s = static_cast<float>(rates_b.target_rate_rad_s);
        cmd32.weld_crank_torque_nm = static_cast<float>(prev_delivered_crank_b);
        cmd32.has_weld_crank_torque = 1u;
        cmd32.enabled = 1u;
        cmd32.crank_reposition = 0u;
        cmd32.steady_state = 0u;
        cmd32.advance = 1u;

        const auto copy_ctrl_to_db = [&](const std::vector<float> &cvec) {
            const auto dst = model_access::mutable_buffer(db->ctrl, mb->nu, "ctrl");
            for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
                const int aid = std::span{params32.joint_actuator_id}[to_idx(i)];
                if (aid >= 0) {
                    dst[to_idx(aid)] = static_cast<double>(cvec[to_idx(aid)]);
                }
            }
        };

        pc::spindle_m2k2_pass0_init_and_base<float, pc::GpuToolkit>(
            &params32, cmd32, step,
            qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
            xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
            spos_f32.data(), scom_f32.data(), submass_f32.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &sp_state, sp_scratch.get(), ctrl32.data());
        copy_ctrl_to_db(ctrl32);

        if (step % control_period_steps == 0) {
            // 4x forwardSkip on control ticks
            engine::forward_skip(mb, db);
            const auto qacc_p0 = to_real_vec<float>(db->qacc, mb->nv);
            pc::spindle_m2k2_pass1_perturb0<float, pc::GpuToolkit>(
                &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p0.data(),
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
                &sp_state, sp_scratch.get(), ctrl32.data());
            copy_ctrl_to_db(ctrl32);
            engine::forward_skip(mb, db);
            const auto qacc_p1 = to_real_vec<float>(db->qacc, mb->nv);
            pc::spindle_m2k2_pass2_perturb1<float, pc::GpuToolkit>(
                &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p1.data(),
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
                &sp_state, sp_scratch.get(), ctrl32.data());
            copy_ctrl_to_db(ctrl32);
            engine::forward_skip(mb, db);
            const auto qacc_p2 = to_real_vec<float>(db->qacc, mb->nv);
            pc::spindle_m2k2_pass3_commit<float, pc::GpuToolkit>(
                &params32, qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p2.data(),
                M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
                &sp_state, sp_scratch.get(), ctrl32.data());
            copy_ctrl_to_db(ctrl32);
            engine::forward_skip(mb, db);
        }

        // Integrate step & settle welds on interval start/end without intervening engine::forward
        {
            const auto q_ro = model_access::readonly_buffer(db->qpos, mb->nq, "qb");
            const auto v_ro = model_access::readonly_buffer(db->qvel, mb->nv, "vb");
            std::ranges::copy(q_ro, q_start_b.begin());
            std::ranges::copy(v_ro, v_start_b.begin());
        }
        const auto prep_b = stepper_b->rider_contacts().prepare_attachment_raw(db);
        engine::step(mb, db);
        const auto settled = stepper_b->rider_contacts().settle_welds(
            db, writers::RiderIntervalState{q_start_b, v_start_b}, true, &prep_b);
        (void)stepper_b->drive().settle();
        prev_delivered_crank_b = stepper_b->rider_contacts().state().delivered_crank_torque_nm;

        {
            const auto qpos_ro = model_access::readonly_buffer(db->qpos, mb->nq, "qpos");
            const auto qvel_ro = model_access::readonly_buffer(db->qvel, mb->nv, "qvel");
            for (const double v : qpos_ro) {
                require(std::isfinite(v), "closed-loop qpos finite");
            }
            for (const double v : qvel_ro) {
                require(std::isfinite(v), "closed-loop qvel finite");
            }
        }

        if (settled.front_pedal_reaction.has_value()) {
            min_pedal_n = std::min(min_pedal_n, settled.front_pedal_reaction->normal_n);
        }
        if (settled.rear_pedal_reaction.has_value()) {
            min_pedal_n = std::min(min_pedal_n, settled.rear_pedal_reaction->normal_n);
        }
        if (settled.saddle_reaction.has_value()) {
            min_saddle_n = std::min(min_saddle_n, settled.saddle_reaction->normal_n);
            max_saddle_n = std::max(max_saddle_n, settled.saddle_reaction->normal_n);
        }
    }

    const double m0_end_x =
        model_access::readonly_buffer(da->qpos, ma->nq, "qpos")[to_idx(root_x_qpos)];
    const double m2k2_end_x =
        model_access::readonly_buffer(db->qpos, mb->nq, "qpos")[to_idx(root_x_qpos)];
    const double dist_m0 = m0_end_x - start_x;
    const double dist_m2k2 = m2k2_end_x - start_x;
    const double rel_dist_err = std::abs(dist_m2k2 - dist_m0) / std::max(1.0, std::abs(dist_m0));

    std::cout << "  Closed-loop 480 steps (0.6 s): dist_M0=" << dist_m0
              << " m, dist_M2K2=" << dist_m2k2
              << " m, rel_err=" << (rel_dist_err * 100.0) << "%"
              << " | min_pedal_n=" << min_pedal_n << " (M0=" << m0_min_pedal_n << ") N"
              << " | saddle_n in [" << min_saddle_n << ", " << max_saddle_n << "] N\n";

    require(min_pedal_n >= -6.0, "closed-loop push-only pedal normal >= -6 N");
    require(min_saddle_n >= -6.0 && max_saddle_n <= 1750.0,
            "closed-loop saddle normal in [-6, 1750] N");
    require(rel_dist_err <= 0.002, "closed-loop distance within <= 0.2% of M0");
}

void expect_invalid_arg(const auto &fn, std::string_view label) {
    bool threw = false;
    try {
        fn();
    } catch (const std::invalid_argument &) {
        threw = true;
    }
    require(threw, label);
}

void test_host_upload_rejections() {
    const ModelPtr m_w = load_xml(repo_file("tests/fixtures/model/welded.xml"));
    const DataPtr d_w(engine::make_data(m_w.get()), &delete_data);
    engine::reset_data(m_w.get(), d_w.get());
    engine::kinematics(m_w.get(), d_w.get());
    const auto valid_tables_ptr = std::make_unique<pc::SpindleReferenceTables>(
        pc::load_spindle_reference_files(
            repo_file("examples/research/rider_strength_reference.json"),
            repo_file("examples/research/rider_joint_envelope_anatomical.json")));
    const auto &valid_tables = *valid_tables_ptr;
    const auto sc_cfg = pc::make_viewer_welded_spindle_scalar_config(1);

    expect_invalid_arg([&] {
        (void)pc::build_spindle_m2k2_params_from_tables<float>(
            m_w.get(), nullptr, valid_tables, sc_cfg);
    }, "reject null d_qpos0");

    const auto check_bad_table = [&](const auto &mutator, std::string_view label) {
        expect_invalid_arg([&] {
            auto t = std::make_unique<pc::SpindleReferenceTables>(valid_tables);
            mutator(*t);
            (void)pc::build_spindle_m2k2_params_from_tables<float>(
                m_w.get(), d_w.get(), *t, sc_cfg);
        }, label);
    };

    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.strength[0].positive.angles = {0.5, 0.4, 1.3};
    }, "reject non-increasing strength angles");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.strength[0].positive.torques = {100.0, -5.0, 120.0};
    }, "reject negative strength torque");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.strength[0].positive.angles = {0.0};
        t.strength[0].positive.torques = {100.0};
    }, "reject knot_count < 2");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.strength[0].positive.angles.assign(pc::kMaxStrengthKnots + 1, 0.0);
        t.strength[0].positive.torques.assign(pc::kMaxStrengthKnots + 1, 10.0);
        for (std::size_t k = 0; k < t.strength[0].positive.angles.size(); ++k) {
            t.strength[0].positive.angles[k] = static_cast<double>(k) * 0.1;
        }
    }, "reject knot_count > kMaxStrengthKnots");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.strength[0].positive.vmax = 0.0;
    }, "reject vmax <= 0");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.strength[0].positive.hill_c = -0.1;
    }, "reject hill_c <= 0");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.strength[0].positive.eccentric_ratio = 0.9;
    }, "reject eccentric_ratio < 1.0");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.envelopes[0].direction = 0;
    }, "reject invalid envelope direction");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.envelopes[0].minimum_anatomical_rad = 1.0;
        t.envelopes[0].maximum_anatomical_rad = 0.5;
    }, "reject envelope min >= max");
    check_bad_table([](pc::SpindleReferenceTables &t) {
        t.envelopes[0].direction = -t.strength[0].direction;
    }, "reject strength/envelope direction mismatch");

    expect_invalid_arg([&] {
        const ModelPtr m_nolim(mj_copyModel(nullptr, m_w.get()), &delete_model);
        const int jid = mj_name2id(m_nolim.get(), mjOBJ_JOINT, "rider_torso_hinge");
        model_access::mutable_buffer(m_nolim->jnt_limited, m_nolim->njnt, "jl")[to_idx(jid)] =
            static_cast<mjtByte>(0);
        (void)pc::build_spindle_m2k2_params_from_tables<float>(
            m_nolim.get(), d_w.get(), valid_tables, sc_cfg);
    }, "reject unlimited joint with envelope");

    expect_invalid_arg([&] {
        const ModelPtr m_nogrip(mj_copyModel(nullptr, m_w.get()), &delete_model);
        const int eqid = mj_name2id(m_nogrip.get(), mjOBJ_EQUALITY, "connect_grip_left");
        const int adr = model_access::readonly_buffer(m_nogrip->name_eqadr, m_nogrip->neq, "neq")
            [to_idx(eqid)];
        model_access::mutable_buffer(m_nogrip->names, m_nogrip->nnames, "names")[to_idx(adr)] = 'X';
        (void)pc::build_spindle_m2k2_params_from_tables<float>(
            m_nogrip.get(), d_w.get(), valid_tables, sc_cfg);
    }, "reject missing connect_grip_left when !grip_is_spring");
}

void test_device_validation_gates(const pc::SpindleM2K2Params<float> &base_params) {
    require(pc::validate_spindle_m2k2_params(&base_params), "valid params accepted");
    require(!pc::validate_spindle_m2k2_params(static_cast<const pc::SpindleM2K2Params<float> *>(nullptr)),
            "null params rejected");

    auto bad = std::make_unique<pc::SpindleM2K2Params<float>>(base_params);
    bad->nv = 0;
    require(!pc::validate_spindle_m2k2_params(bad.get()), "nv=0 rejected");
    *bad = base_params;
    bad->nv = pc::kMaxNv + 1;
    require(!pc::validate_spindle_m2k2_params(bad.get()), "nv>kMaxNv rejected");
    *bad = base_params;
    bad->frame_body = bad->nbody;
    require(!pc::validate_spindle_m2k2_params(bad.get()), "frame_body>=nbody rejected");
    *bad = base_params;
    bad->grip_sites[1] = -1;
    require(!pc::validate_spindle_m2k2_params(bad.get()), "grip_sites<0 rejected");
    *bad = base_params;
    bad->joint_dof_adr[4] = bad->nv;
    require(!pc::validate_spindle_m2k2_params(bad.get()), "joint_dof_adr>=nv rejected");
    *bad = base_params;
    bad->joint_actuator_id[0] = bad->nu;
    require(!pc::validate_spindle_m2k2_params(bad.get()), "joint_actuator_id>=nu rejected");

    const float *nf = nullptr;
    const pc::i32 *ni = nullptr;
    float *mf = nullptr;

    pc::u8 has_ml = 1;
    float ml = 9.0f;
    pc::spindle_m2k2_min_torso_lean_rad<float, pc::GpuToolkit>(
        bad.get(), nf, nf, nf, nf, 0.25f, has_ml, ml);
    require(has_ml == 0 && ml == 0.0f, "invalid params min_torso_lean safe no-op");

    std::array<float, pc::kNumUpperSpindleJoints> up_tgt{1.f, 2.f, 3.f, 4.f, 5.f};
    pc::u8 t_sat = 1;
    pc::u8 a_sat = 1;
    pc::upper_targets_core<float, pc::GpuToolkit>(
        bad.get(), nf, nf, nf, nf, 0.1f, up_tgt.data(), t_sat, a_sat);
    require(t_sat == 0 && a_sat == 0 && up_tgt[0] == 0.0f,
            "invalid params upper_targets_core safe no-op");

    pc::SpindleM2K2State<float> st_bad{};
    auto scr_bad = std::make_unique<pc::SpindleM2K2StepScratch<float>>();
    scr_bad->active_newton = 1;
    pc::spindle_m2k2_pass0_init_and_base<float, pc::GpuToolkit>(
        bad.get(), pc::SpindleCommand<float>{}, 0, nf, nf, nf,
        nf, nf, nf, nf, nf, nf, nf,
        ni, ni, ni, nf, &st_bad, scr_bad.get(), mf);
    require(scr_bad->active_newton == 0, "invalid params pass0 safe no-op");

    const std::array<pc::i32, 3> radr{0, 2, 4};
    const std::array<pc::i32, 3> rnnz{2, 2, 1};
    const std::array<pc::i32, 5> cind{0, 2, 0, 1, 2};
    const std::array<float, 5> vals{2.0f, 999.0f, 3.0f, 4.0f, 5.0f};
    const std::array<float, 3> vec{1.0f, 2.0f, 3.0f};
    std::array<float, 3> out{-1.0f, -1.0f, -1.0f};
    pc::mul_m_csr<float>(3, radr.data(), rnnz.data(), cind.data(), vals.data(), vec.data(), out.data());
    require_bits_f32(out[0], 2.0f * 1.0f + 3.0f * 2.0f, "mul_m_csr strict lower-tri row 0");
    require_bits_f32(out[1], 3.0f * 1.0f + 4.0f * 2.0f, "mul_m_csr strict lower-tri row 1");
    require_bits_f32(out[2], 5.0f * 3.0f, "mul_m_csr strict lower-tri row 2");
}

void test_validation_power_bisection_and_subnormals(const MultiStateCaptureBundle &bundle) {
    std::cout << "[SP4-A] 7. Device/host validation (B2), power bisection (M1), & subnormal FTZ parity (I2)...\n";
    const mjModel *m_ref = bundle.model.get();
    const pc::SpindleUploadTiming timing{
        .crank_length_m = bundle.crank_length_m,
        .control_period_steps = bundle.control_period_steps,
    };
    const auto base_params = std::make_unique<pc::SpindleM2K2Params<float>>(
        pc::build_spindle_m2k2_params_from_spindle_config<float>(
            m_ref, bundle.pose, bundle.spindle_cfg, timing));
    test_device_validation_gates(*base_params);
    test_host_upload_rejections();

    // M1 + I2: Active power-budget bisection (req > total & prio >= total) + subnormal qvel (1e-40f/-1e-40f)
    const mjModel *m_ref2 = m_ref;
    const ModelPtr m_mut(mj_copyModel(nullptr, m_ref2), &delete_model);
    mjModel *m = m_mut.get();
    DataPtr d_cpu(engine::make_data(m), &delete_data);
    DataPtr d_gpu(engine::make_data(m), &delete_data);
    const auto M_rowadr_i32 = to_i32_vec(m->M_rowadr, m->nv);
    const auto M_rownnz_i32 = to_i32_vec(m->M_rownnz, m->nv);
    const auto M_colind_i32 = to_i32_vec(m->M_colind, static_cast<int>(m->nC));
    const auto submass_f32 = to_real_vec<float>(m->body_subtreemass, m->nbody);

    const CapturedTestState &seed_st = bundle.states[bundle.states.size() / 2u];

    struct EdgeCaseSpec {
        std::string label;
        double active_pos_power_w;
        double jlim_w;
        double spd_lim;
        double mean_tq_nm;
        bool steady_state;
        bool use_subnorm_qvel;
        bool check_cpu_m0;
    };
    const std::array<EdgeCaseSpec, 3> edge_cases{{
        {.label = "power_bisect_req_gt_total",
         .active_pos_power_w = 120.0,
         .jlim_w = 250.0,
         .spd_lim = 20.0,
         .mean_tq_nm = 60.0,
         .steady_state = false,
         .use_subnorm_qvel = false,
         .check_cpu_m0 = true},
        {.label = "power_bisect_prio_ge_total",
         .active_pos_power_w = 6.0,
         .jlim_w = 5.0,
         .spd_lim = 20.0,
         .mean_tq_nm = 55.0,
         .steady_state = false,
         .use_subnorm_qvel = false,
         .check_cpu_m0 = true},
        {.label = "subnormal_qvel_ftz_parity",
         .active_pos_power_w = 0.0,
         .jlim_w = 0.0,
         .spd_lim = 0.0,
         .mean_tq_nm = 35.0,
         .steady_state = false,
         .use_subnorm_qvel = true,
         .check_cpu_m0 = false},
    }};

    for (const auto &ec : edge_cases) {
        auto scfg = bundle.spindle_cfg;
        scfg.active_positive_power_limit_w = ec.active_pos_power_w;
        scfg.joint_power_limit_w = ec.jlim_w;
        scfg.joint_speed_limit_rad_s = ec.spd_lim;
        const auto params32_ec = std::make_unique<pc::SpindleM2K2Params<float>>(
            pc::build_spindle_m2k2_params_from_spindle_config<float>(
                m, bundle.pose, scfg, timing));

        mj_resetData(m, d_gpu.get());
        std::ranges::copy(seed_st.qpos, model_access::mutable_buffer(d_gpu->qpos, m->nq, "q").begin());
        std::ranges::copy(seed_st.qvel, model_access::mutable_buffer(d_gpu->qvel, m->nv, "v").begin());
        std::ranges::copy(seed_st.ctrl, model_access::mutable_buffer(d_gpu->ctrl, m->nu, "c").begin());
        std::ranges::copy(seed_st.qfrc_applied,
                          model_access::mutable_buffer(d_gpu->qfrc_applied, m->nv, "qf").begin());
        if (ec.use_subnorm_qvel) {
            const auto qv_mut = model_access::mutable_buffer(d_gpu->qvel, m->nv, "qv");
            for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
                const int dof = std::span{params32_ec->joint_dof_adr}[to_idx(i)];
                qv_mut[to_idx(dof)] = (i % 2 == 0) ? 1.0e-40 : -1.0e-40;
            }
        }
        engine::forward(m, d_gpu.get());

        auto qpos_f32 = to_real_vec<float>(d_gpu->qpos, m->nq);
        auto qvel_f32 = to_real_vec<float>(d_gpu->qvel, m->nv);
        if (ec.use_subnorm_qvel) {
            for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
                const int dof = std::span{params32_ec->joint_dof_adr}[to_idx(i)];
                qvel_f32[to_idx(dof)] = (i % 2 == 0) ? 1.0e-40f : -1.0e-40f;
            }
        }
        const auto bias_f32 = to_real_vec<float>(d_gpu->qfrc_bias, m->nv);
        const auto xmat_f32 = to_real_vec<float>(d_gpu->xmat, m->nbody * 9);
        const auto xpos_f32 = to_real_vec<float>(d_gpu->xpos, m->nbody * 3);
        const auto xanc_f32 = to_real_vec<float>(d_gpu->xanchor, m->njnt * 3);
        const auto xax_f32 = to_real_vec<float>(d_gpu->xaxis, m->njnt * 3);
        const auto spos_f32 = to_real_vec<float>(d_gpu->site_xpos, m->nsite * 3);
        const auto scom_f32 = to_real_vec<float>(d_gpu->subtree_com, m->nbody * 3);
        const auto dM_f32 = to_real_vec<float>(d_gpu->M, static_cast<int>(m->nC));

        pc::SpindleCommand<float> cmd32{};
        cmd32.mean_crank_torque_nm = static_cast<float>(ec.mean_tq_nm);
        cmd32.torso_lean_rad = static_cast<float>(seed_st.cpu_cmd.posture.torso_lean_rad);
        cmd32.crank_target_rate_rad_s = static_cast<float>(seed_st.cpu_cmd.crank_target_rate_rad_s);
        cmd32.enabled = 1u;
        cmd32.steady_state = ec.steady_state ? 1u : 0u;
        cmd32.advance = 1u;

        pc::SpindleM2K2State<float> st32{};
        const auto scr32 = std::make_unique<pc::SpindleM2K2StepScratch<float>>();
        auto ctrl32 = to_real_vec<float>(d_gpu->ctrl, m->nu);
        const auto write_gpu_ctrl = [&](const std::vector<float> &cvec) {
            const auto dst = model_access::mutable_buffer(d_gpu->ctrl, m->nu, "ctrl");
            for (int a = 0; a < m->nu; ++a) dst[to_idx(a)] = static_cast<double>(cvec[to_idx(a)]);
        };
        pc::spindle_m2k2_pass0_init_and_base<float, pc::GpuToolkit>(
            params32_ec.get(), cmd32, 0, qpos_f32.data(), qvel_f32.data(), bias_f32.data(),
            xmat_f32.data(), xpos_f32.data(), xanc_f32.data(), xax_f32.data(),
            spos_f32.data(), scom_f32.data(), submass_f32.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &st32, scr32.get(), ctrl32.data());
        const auto ctrl_p0 = ctrl32;
        write_gpu_ctrl(ctrl32); engine::forward_skip(m, d_gpu.get());
        const auto qacc_p0 = to_real_vec<float>(d_gpu->qacc, m->nv);
        pc::spindle_m2k2_pass1_perturb0<float, pc::GpuToolkit>(
            params32_ec.get(), qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p0.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &st32, scr32.get(), ctrl32.data());
        const auto ctrl_p1 = ctrl32;
        write_gpu_ctrl(ctrl32); engine::forward_skip(m, d_gpu.get());
        const auto qacc_p1 = to_real_vec<float>(d_gpu->qacc, m->nv);
        pc::spindle_m2k2_pass2_perturb1<float, pc::GpuToolkit>(
            params32_ec.get(), qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p1.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &st32, scr32.get(), ctrl32.data());
        const auto ctrl_p2 = ctrl32;
        write_gpu_ctrl(ctrl32); engine::forward_skip(m, d_gpu.get());
        const auto qacc_p2 = to_real_vec<float>(d_gpu->qacc, m->nv);
        pc::spindle_m2k2_pass3_commit<float, pc::GpuToolkit>(
            params32_ec.get(), qpos_f32.data(), qvel_f32.data(), bias_f32.data(), qacc_p2.data(),
            M_rowadr_i32.data(), M_rownnz_i32.data(), M_colind_i32.data(), dM_f32.data(),
            &st32, scr32.get(), ctrl32.data());
        std::vector<float> ctrl_held(static_cast<std::size_t>(m->nu), 0.0f);
        pc::spindle_m2k2_write_held_ctrl<float, pc::GpuToolkit>(
            params32_ec.get(), &st32, qpos_f32.data(), qvel_f32.data(), 1u, ctrl_held.data());

        double host_pos_power = 0.0;
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int aid = std::span{params32_ec->joint_actuator_id}[to_idx(i)];
            const int dof = std::span{params32_ec->joint_dof_adr}[to_idx(i)];
            host_pos_power += std::max(
                0.0, static_cast<double>(ctrl32[to_idx(aid)]) * static_cast<double>(qvel_f32[to_idx(dof)]));
        }
        if (ec.active_pos_power_w > 0.0) {
            require(host_pos_power <= ec.active_pos_power_w + 1e-2,
                    ec.label + " positive power within active budget");
        }

        if (ec.check_cpu_m0) {
            auto ctrl_cpu = std::make_unique<rider::SpindleController>(
                m, bundle.pose, scfg, bundle.crank_length_m);
            mj_resetData(m, d_cpu.get());
            std::ranges::copy(seed_st.qpos, model_access::mutable_buffer(d_cpu->qpos, m->nq, "q").begin());
            std::ranges::copy(seed_st.qvel, model_access::mutable_buffer(d_cpu->qvel, m->nv, "v").begin());
            std::ranges::copy(seed_st.ctrl, model_access::mutable_buffer(d_cpu->ctrl, m->nu, "c").begin());
            std::ranges::copy(seed_st.qfrc_applied,
                              model_access::mutable_buffer(d_cpu->qfrc_applied, m->nv, "qf").begin());
            engine::forward(m, d_cpu.get());
            auto rcmd = seed_st.cpu_cmd;
            rcmd.mean_crank_torque_nm = ec.mean_tq_nm;
            rcmd.enabled = true;
            const auto cpu_tq = ctrl_cpu->compute(
                d_cpu.get(), rcmd, true, std::nullopt, ec.steady_state, std::nullopt,
                seed_st.pedal_recovery, std::nullopt);
            ctrl_cpu->write(d_cpu.get(), cpu_tq);
            const auto cpu_c = model_access::readonly_buffer(d_cpu->ctrl, m->nu, "cc");
            for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
                const int aid = std::span{params32_ec->joint_actuator_id}[to_idx(i)];
                require(std::abs(static_cast<double>(ctrl32[to_idx(aid)]) - cpu_c[to_idx(aid)]) <= 1.0,
                        ec.label + " Host FP32 vs M0 within 1.0 Nm");
            }
        }

#if defined(__APPLE__)
        auto mreq = std::make_unique<pc::test_sp4_metal::MetalSp4Request>();
        mreq->physics_core_dir = BIKE_PHYSICS_CORE_SRC_DIR;
        mreq->params = *params32_ec;
        mreq->body_subtreemass = submass_f32;
        mreq->M_rowadr = M_rowadr_i32;
        mreq->M_rownnz = M_rownnz_i32;
        mreq->M_colind = M_colind_i32;
        mreq->nC = static_cast<pc::i32>(m->nC);
        pc::test_sp4_metal::Sp4StateSnapshot snap;
        snap.cmd = cmd32;
        snap.step_index = 0;
        snap.min_elbow_flexion_rad = static_cast<float>(seed_st.min_elbow_flexion_rad);
        snap.initial_coast_hold_nm = 0.0f;
        snap.qpos = qpos_f32;
        snap.qvel = qvel_f32;
        snap.qfrc_bias = bias_f32;
        snap.body_xmat = xmat_f32;
        snap.body_xpos = xpos_f32;
        snap.jnt_xanchor = xanc_f32;
        snap.jnt_xaxis = xax_f32;
        snap.site_xpos = spos_f32;
        snap.subtree_com = scom_f32;
        snap.d_M = dM_f32;
        snap.csr_vec = std::vector<float>(static_cast<std::size_t>(m->nv), 0.1f);
        snap.qacc_p0 = qacc_p0;
        snap.qacc_p1 = qacc_p1;
        snap.qacc_p2 = qacc_p2;
        mreq->states.push_back(std::move(snap));

        const auto mout = pc::test_sp4_metal::run_metal_sp4_parity(*mreq);
        if (!mout->error_message.empty()) {
            throw std::runtime_error("Metal edge/subnormal parity failed: " + mout->error_message);
        }
        const auto &mr = mout->state_out[0];
        for (int i = 0; i < pc::kNumSpindleJoints; ++i) {
            const int k = std::span{params32_ec->joint_actuator_id}[to_idx(i)];
            require_bits_f32(mr.ctrl_pass0[to_idx(k)], ctrl_p0[to_idx(k)], ec.label + " Metal pass0");
            require_bits_f32(mr.ctrl_pass1[to_idx(k)], ctrl_p1[to_idx(k)], ec.label + " Metal pass1");
            require_bits_f32(mr.ctrl_pass2[to_idx(k)], ctrl_p2[to_idx(k)], ec.label + " Metal pass2");
            require_bits_f32(mr.ctrl_pass3[to_idx(k)], ctrl32[to_idx(k)], ec.label + " Metal pass3");
            require_bits_f32(mr.ctrl_held[to_idx(k)], ctrl_held[to_idx(k)], ec.label + " Metal held");
        }
#endif
    }
    std::cout << "  Validation gates, power bisection (req & prio), and subnormal FTZ 0-bit Metal parity OK.\n";
}

} // namespace

int main() {
    try {
        const auto bundle = std::make_unique<MultiStateCaptureBundle>(
            capture_diverse_states_from_runtime());
        const pc::SpindleUploadTiming main_timing{
            .crank_length_m = bundle->crank_length_m,
            .control_period_steps = bundle->control_period_steps,
        };
        const auto params64_ptr = std::make_unique<pc::SpindleM2K2Params<double>>(
            pc::build_spindle_m2k2_params_from_spindle_config<double>(
                bundle->model.get(), bundle->pose, bundle->spindle_cfg, main_timing));
        const auto params32_ptr = std::make_unique<pc::SpindleM2K2Params<float>>(
            pc::build_spindle_m2k2_params_from_spindle_config<float>(
                bundle->model.get(), bundle->pose, bundle->spindle_cfg, main_timing));

        test_analytic_leg_kinematics_accuracy(*params64_ptr);
        test_leg_weights_and_csr_matvec_parity(bundle->model.get(), *params64_ptr, *params32_ptr);
        test_multi_state_m2k2_parity_and_metal(*bundle);
        test_validation_power_bisection_and_subnormals(*bundle);
        test_closed_loop_cpu_multistep_ride();

        std::cout << "[SP4-A] ALL TESTS PASSED\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "[SP4-A] FAILED: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[SP4-A] FAILED: unknown exception\n";
        return 1;
    }
}
