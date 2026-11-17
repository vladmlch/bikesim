#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <complex>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>

namespace bike::spatial::whipple {

using Mat2 = std::array<double, 4>;
using Mat4 = std::array<double, 16>;
using State4 = std::array<double, 4>;
using Eigenvalues4 = std::array<std::complex<double>, 4>;

// Exact 25 physical benchmark parameters (+ gravity g) from Meijaard et al.
// (2007) Table 1, in SAE Z-down coordinates (+X forward, +Y right, +Z down,
// origin at the rear wheel ground contact point).
struct WhippleParameters {
    // Geometry & gravity
    double w_m = 1.02;
    double c_m = 0.08;
    double lambda_rad = std::numbers::pi / 10.0;
    double g_mps2 = 9.81;

    // 1. Rear wheel (R)
    double r_r_m = 0.30;
    double m_r_kg = 2.0;
    double i_rxx_kgm2 = 0.0603;
    double i_ryy_kgm2 = 0.12;

    // 2. Rear frame + rider (B)
    double x_b_m = 0.30;
    double z_b_m = -0.90;
    double m_b_kg = 85.0;
    double i_bxx_kgm2 = 9.2;
    double i_byy_kgm2 = 11.0;
    double i_bzz_kgm2 = 2.8;
    double i_bxz_kgm2 = 2.4;

    // 3. Front handlebar/fork (H)
    double x_h_m = 0.90;
    double z_h_m = -0.70;
    double m_h_kg = 4.0;
    double i_hxx_kgm2 = 0.05892;
    double i_hyy_kgm2 = 0.06;
    double i_hzz_kgm2 = 0.00708;
    double i_hxz_kgm2 = -0.00756;

    // 4. Front wheel (F)
    double r_f_m = 0.35;
    double m_f_kg = 3.0;
    double i_fxx_kgm2 = 0.1405;
    double i_fyy_kgm2 = 0.28;
};

// Canonical Meijaard (2007) linearized matrices for q = (phi, delta)^T:
//   M * q_ddot + v * C1 * q_dot + (g * K0 + v^2 * K2) * q = 0.
struct WhippleCanonicalMatrices {
    Mat2 M{};
    Mat2 C1{};
    Mat2 K0{};
    Mat2 K2{};
    double g_mps2 = 9.81;

    [[nodiscard]] static WhippleCanonicalMatrices from_parameters(
        const WhippleParameters& params = {});

    void compute_state_matrix(double speed_mps, Mat4& out) const;
    [[nodiscard]] Eigenvalues4 reference_eigenvalues(double speed_mps) const;
};

struct MjModelDeleter {
    void operator()(mjModel* model) const noexcept;
};

struct MjDataDeleter {
    void operator()(mjData* data) const noexcept;
};

using ModelOwner = std::unique_ptr<mjModel, MjModelDeleter>;
using DataOwner = std::unique_ptr<mjData, MjDataDeleter>;

// Computes all 4 eigenvalues of a general real 4x4 row-major matrix via
// orthogonal Hessenberg reduction and Francis double-shift QR, sorted by
// descending real part then descending imaginary part.
[[nodiscard]] Eigenvalues4 real_4x4_eigenvalues(const Mat4& matrix);

// Generates the 4-body Z-up Whipple MJCF (nbody=5, njnt=4, nq=10, nv=9)
// with zero joint damping, stiffness, frictionloss, armature, and disabled
// native geom contacts.
[[nodiscard]] std::string generate_whipple_mjcf(const WhippleParameters& params = {});

// Compiles a generated Whipple MJCF document via MuJoCo VFS inside engine::invoke.
[[nodiscard]] ModelOwner compile_whipple_mjcf(std::string_view xml);

struct KktLinearizationResult {
    State4 fixed_point_residual{};
    Mat4 jacobian_rich{};
    Eigenvalues4 eigenvalues{};
    double max_manifold_gap_m = 0.0;
};

// Method A: nonholonomic 6-constraint knife-edge KKT linearization on an
// unconstrained Z-up MuJoCo Whipple model at forward speed `speed_mps`.
[[nodiscard]] KktLinearizationResult mujoco_kkt_linearize(
    const mjModel* model,
    double speed_mps);

[[nodiscard]] Eigenvalues4 mujoco_kkt_eigenvalues(
    const mjModel* model,
    double speed_mps);

struct SpeedBracket {
    double low_mps = 0.0;
    double high_mps = 0.0;
};

// Bisects Re(lambda_weave(v)) = 0 on the MuJoCo KKT eigenvalues.
[[nodiscard]] double find_weave_critical_speed_mps(
    const mjModel* model,
    SpeedBracket bracket = {.low_mps = 3.0, .high_mps = 5.5},
    double tol_mps = 1e-9);

// Bisects lambda_capsize(v) = 0 on the MuJoCo KKT eigenvalues.
[[nodiscard]] double find_capsize_critical_speed_mps(
    const mjModel* model,
    SpeedBracket bracket = {.low_mps = 5.0, .high_mps = 7.5},
    double tol_mps = 1e-9);

} // namespace bike::spatial::whipple
