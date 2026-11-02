// native/tests/gpu/test_gpu_rider.cpp — Standalone C++23 unit test for
// native/src/gpu/rider_kernel.hpp and authoritative spindle_math.cpp parity.

#ifndef BIKE_NUMPY_DISTRIBUTIONS_CONTRACTED
#define BIKE_NUMPY_DISTRIBUTIONS_CONTRACTED 1
#endif
#ifndef BIKE_NUMPY_INTERP_CONTRACTED
#define BIKE_NUMPY_INTERP_CONTRACTED 1
#endif
#ifndef BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN
#define BIKE_NUMPY_MUL_BROADCAST_ELEMENT_NAN 0
#endif
#ifndef BIKE_SCIPY_CORRELATE_CONTRACTED
#define BIKE_SCIPY_CORRELATE_CONTRACTED 1
#endif

#include "../../src/cblas_abi.hpp"

#include <iterator>

namespace blas {
    // Sequential stand-in for the seeded blas::ddot that rider::np_norm
    // (spindle_linalg.hpp) calls: this test executable links standalone, so
    // the typed face gets a local definition. std::advance keeps the strided
    // element walk off raw-pointer arithmetic.
    double ddot(int n, const double *x, int incx, const double *y, int incy) noexcept {
        double sum = 0.0;
        const double *xi = x;
        const double *yi = y;
        for (int i = 0; i < n; ++i) {
            sum += *xi * *yi;
            std::advance(xi, incx);
            std::advance(yi, incy);
        }
        return sum;
    }
}

// spindle_math.cpp is included textually on purpose: this test executable
// links standalone (no bike_native_contract_core / bike_rider_math), the same
// arrangement as test_gpu_tire.cpp/station.cpp and
// test_gpu_drivetrain_resistance.cpp/freehub.cpp — see the note above
// bike_add_gpu_test in native/CMakeLists.txt.
// NOLINTNEXTLINE(bugprone-suspicious-include) deliberate standalone-link mechanism: spindle_math.cpp is compiled INTO this TU so the test links without bike_native_contract_core/bike_rider_math and avoids duplicate symbol definitions (native/CMakeLists.txt:543-547). Register: docs/TESTING.md GPU test cpp-include row.
#include "../../src/rider/spindle_math.cpp"
#include "../../src/gpu/rider_kernel.hpp"
#include "../../src/platform/argv.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rider {
namespace spindle = ::spindle;
}

namespace {

using namespace bike::gpu;

using TestFunction = void (*)();

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition)
        throw std::runtime_error(std::string(message));
}

// Numeric helper used instead of printf-style messages. Templated on the
// arithmetic type so float comparisons keep their original float semantics
// instead of silently promoting to double.
template <class T>
void require_near(T actual, T expected, T tol, std::string_view what) {
    if (!(std::abs(actual - expected) <= tol))
        throw std::runtime_error(std::string(what) +
                                 ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected) +
                                 " tol=" + std::to_string(tol));
}

// GpuModelIndices addresses are int for the kernel ABI (a disabled slot is
// -1); container subscripts take size_t, so the explicit conversion lives in
// one place.
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

constexpr double kPi = std::numbers::pi;
constexpr double kDegToRadRef = kPi / 180.0;
// Rider slots 0..3 are legs (passive crank followers, never driven); slots 4..8 are upper body.
constexpr int kFirstUpperBodyJoint = 4;

double relative_error(double actual, double expected) {
    const double denom = std::max(std::abs(expected), 1e-2);
    return std::abs(actual - expected) / denom;
}

void test_anatomical_angle_and_q_range() {
    struct QRangeCase {
        float min_deg;
        float max_deg;
        float ref_deg;
    };
    constexpr std::array<QRangeCase, 4> q_range_cases{{
        {.min_deg = -120.0f, .max_deg = 15.0f, .ref_deg = 10.0f},
        {.min_deg = -135.0f, .max_deg = 0.0f, .ref_deg = -5.0f},
        {.min_deg = -45.0f, .max_deg = 75.0f, .ref_deg = 20.0f},
        {.min_deg = -60.0f, .max_deg = 140.0f, .ref_deg = -15.0f},
    }};

    for (const auto &tc : q_range_cases) {
        for (const int sign_flip : {0, 1}) {
            const rider::spindle::JointEnvelope env{
                .neutral_anatomical_rad = static_cast<double>(tc.ref_deg) * kDegToRadRef,
                .direction = (sign_flip != 0) ? -1 : 1,
                .minimum_anatomical_rad = static_cast<double>(tc.min_deg) * kDegToRadRef,
                .maximum_anatomical_rad = static_cast<double>(tc.max_deg) * kDegToRadRef,
                .provenance = "unit_test",
            };

            const auto ref_range = rider::spindle::joint_q_range(env);
            const GpuVec2 gpu_range =
                gpu_joint_q_range(tc.min_deg, tc.max_deg, tc.ref_deg, sign_flip);

            require(gpu_range.x < gpu_range.y, "q_lo must be strictly below q_hi");
            require(
                relative_error(static_cast<double>(gpu_range.x), ref_range[0]) < 1e-5,
                "gpu_joint_q_range q_lo mismatch vs authoritative spindle_math.cpp");
            require(
                relative_error(static_cast<double>(gpu_range.y), ref_range[1]) < 1e-5,
                "gpu_joint_q_range q_hi mismatch vs authoritative spindle_math.cpp");

            for (const float q_rad : {-2.2f, -1.1f, -0.35f, 0.0f, 0.45f, 1.3f, 2.5f}) {
                const double ref_ang = rider::spindle::anatomical_angle(env, static_cast<double>(q_rad));
                const float gpu_ang = gpu_anatomical_angle(q_rad, tc.ref_deg, sign_flip);
                require(
                    relative_error(static_cast<double>(gpu_ang), ref_ang) < 1e-5,
                    "gpu_anatomical_angle mismatch vs authoritative spindle_math.cpp");
            }
        }
    }
}

void test_soft_edge_torque_and_potential() {
    const float q_lo = -1.2f;
    const float q_hi = 0.8f;
    const float margin = 0.15f;
    const float k_edge = 250.0f;

    // 1. Inside the safe interior [q_lo + margin, q_hi - margin], both torque and
    // potential energy must be identically zero even with non-zero velocity.
    for (int i = 0; i <= 20; ++i) {
        const float alpha = static_cast<float>(i) / 20.0f;
        const float q = (q_lo + margin) + alpha * ((q_hi - margin) - (q_lo + margin));
        const float tau0 = gpu_soft_edge_torque(q, 0.0f, q_lo, q_hi, margin, k_edge, 0.0f);
        const float tau_damped = gpu_soft_edge_torque(q, 1.5f, q_lo, q_hi, margin, k_edge, 8.0f);
        const float pot = gpu_soft_edge_potential(q, q_lo, q_hi, margin, k_edge);
        require(tau0 == 0.0f, "soft edge torque must be zero inside safe interior");
        require(tau_damped == 0.0f, "damped soft edge torque must be zero inside safe interior");
        require(pot == 0.0f, "soft edge potential must be zero inside safe interior");
    }

    // 2. Lower margin and past lower limit: positive restoring torque and growing
    // potential energy matching authoritative spindle_math.cpp within < 1e-5 relative error.
    float prev_tau_lo = 0.0f;
    float prev_pot_lo = 0.0f;
    for (int step = 1; step <= 25; ++step) {
        const float pen = 0.015f * static_cast<float>(step);
        const float q = (q_lo + margin) - pen;
        const float gpu_tau = gpu_soft_edge_torque(q, 0.0f, q_lo, q_hi, margin, k_edge, 0.0f);
        const float gpu_pot = gpu_soft_edge_potential(q, q_lo, q_hi, margin, k_edge);
        const rider::spindle::SoftEdgeRow ref = rider::spindle::soft_edge_scalar(
            static_cast<double>(q),
            static_cast<double>(q_lo),
            static_cast<double>(q_hi),
            static_cast<double>(k_edge),
            static_cast<double>(margin));

        require(gpu_tau > prev_tau_lo, "lower soft edge torque must grow monotonically");
        require(gpu_pot > prev_pot_lo, "lower soft edge potential must grow monotonically");
        require(
            relative_error(static_cast<double>(gpu_tau), ref.torque) < 1e-5,
            "lower soft edge torque relative error >= 1e-5 vs spindle_math.cpp");
        require(
            relative_error(static_cast<double>(gpu_pot), ref.stored_j) < 1e-5,
            "lower soft edge potential relative error >= 1e-5 vs spindle_math.cpp");

        prev_tau_lo = gpu_tau;
        prev_pot_lo = gpu_pot;
    }

    // 3. Upper margin and past upper limit: negative restoring torque and growing
    // potential energy matching authoritative spindle_math.cpp within < 1e-5 relative error.
    float prev_tau_hi = 0.0f;
    float prev_pot_hi = 0.0f;
    for (int step = 1; step <= 25; ++step) {
        const float pen = 0.015f * static_cast<float>(step);
        const float q = (q_hi - margin) + pen;
        const float gpu_tau = gpu_soft_edge_torque(q, 0.0f, q_lo, q_hi, margin, k_edge, 0.0f);
        const float gpu_pot = gpu_soft_edge_potential(q, q_lo, q_hi, margin, k_edge);
        const rider::spindle::SoftEdgeRow ref = rider::spindle::soft_edge_scalar(
            static_cast<double>(q),
            static_cast<double>(q_lo),
            static_cast<double>(q_hi),
            static_cast<double>(k_edge),
            static_cast<double>(margin));

        require(gpu_tau < prev_tau_hi, "upper soft edge torque must grow more negative");
        require(gpu_pot > prev_pot_hi, "upper soft edge potential must grow monotonically");
        require(
            relative_error(static_cast<double>(gpu_tau), ref.torque) < 1e-5,
            "upper soft edge torque relative error >= 1e-5 vs spindle_math.cpp");
        require(
            relative_error(static_cast<double>(gpu_pot), ref.stored_j) < 1e-5,
            "upper soft edge potential relative error >= 1e-5 vs spindle_math.cpp");

        prev_tau_hi = gpu_tau;
        prev_pot_hi = gpu_pot;
    }
}

void test_write_rider_qfrc_and_ctrl() {
    // 1. Verify default GpuModelIndices paired mapping across all 9 rider joints
    const GpuModelIndices idx{};
    const auto rider_ctrl = std::span{idx.rider_ctrl};
    const auto rider_dof = std::span{idx.rider_dof};
    const auto rider_qpos = std::span{idx.rider_qpos};
    const std::array<int, kNumRiderJoints> expected_ctrl = {8, 9, 10, 11, 3, 4, 5, 6, 7};
    const std::array<int, kNumRiderJoints> expected_dof = {22, 23, 24, 25, 17, 18, 19, 20, 21};
    const std::array<int, kNumRiderJoints> expected_qpos = {22, 23, 24, 25, 17, 18, 19, 20, 21};
    for (std::size_t j = 0; j < kNumRiderJoints; ++j) {
        require(rider_ctrl[j] == expected_ctrl[j], "rider_ctrl index must match paired 9-joint mapping");
        require(rider_dof[j] == expected_dof[j], "rider_dof index must match paired 9-joint mapping");
        require(rider_qpos[j] == expected_qpos[j], "rider_qpos index must match paired 9-joint mapping");
    }

    // 2. Verify GpuRiderParams defaults aligned with paired model.xml
    GpuRiderParams rp{};
    const std::span<const GpuJointEnvelopeSpec> joints{rp.joints};
    const std::span<const float> kp_posture{rp.kp_posture};
    const std::span<const float> kd_posture{rp.kd_posture};
    const std::span<const float> target_qpos{rp.target_qpos_rad};
    require(std::abs(joints[1].min_deg - (-139.80f)) < 1e-4f && joints[1].max_deg == 0.0f &&
            std::abs(joints[1].ref_deg - (-76.27f)) < 1e-4f, "joints[1] knee_front ROM default mismatch");
    require(joints[2].min_deg == -120.0f && std::abs(joints[2].max_deg - 35.27f) < 1e-4f &&
            std::abs(joints[2].ref_deg - (-27.10f)) < 1e-4f, "joints[2] hip_rear ROM default mismatch");
    require_near(target_qpos[2], -0.23f, 1e-5f, "joints[2] target_qpos_rad default mismatch");
    require(joints[6].sign_flip == 1, "joints[6] elbow_l sign_flip must be 1");
    require(joints[8].sign_flip == 1, "joints[8] elbow_r sign_flip must be 1");
    require_near(target_qpos[8], -0.14391f, 1e-5f, "joints[8] target_qpos_rad default mismatch");
    // Legs are not driven: zero posture gains. Upper body keeps its posture-holding gains.
    const std::array<float, kNumRiderJoints> expected_kp = {0.0f, 0.0f, 0.0f, 0.0f, 180.0f, 95.0f, 75.0f, 95.0f, 75.0f};
    const std::array<float, kNumRiderJoints> expected_kd = {0.0f, 0.0f, 0.0f, 0.0f, 18.0f, 9.5f, 7.5f, 9.5f, 7.5f};
    for (std::size_t j = 0; j < kNumRiderJoints; ++j) {
        require(kp_posture[j] == expected_kp[j], "kp_posture default mismatch (legs must be 0)");
        require(kd_posture[j] == expected_kd[j], "kd_posture default mismatch (legs must be 0)");
    }

    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    std::array<float, kMaxNv> qfrc{};
    std::array<float, kMaxNu> ctrl{};

    // Pre-populate qfrc on rider DOFs with 12.5f to explicitly verify += accumulation
    for (const int dof : rider_dof) {
        qfrc[to_index(dof)] = 12.5f;
    }

    // Crank phase: legs must not respond to it (see test_leg_joints_do_not_drive_crank).
    qpos[to_index(idx.crank_spin_qpos)] = 0.65f;

    // Place rider joints near/past envelope edges and away from target posture
    // so both envelope_torque_nm and actuator_ctrl_nm are actively exercised.
    for (std::size_t j = 0; j < kNumRiderJoints; ++j) {
        const GpuJointEnvelopeSpec &spec = joints[j];
        const GpuVec2 range =
            gpu_joint_q_range(spec.min_deg, spec.max_deg, spec.ref_deg, spec.sign_flip);
        // Push even-indexed joints below q_lo and odd-indexed joints above q_hi
        qpos[to_index(rider_qpos[j])] = (j % 2 == 0) ? (range.x - 0.05f) : (range.y + 0.05f);
        qvel[to_index(rider_dof[j])] = (j % 2 == 0) ? -0.4f : 0.4f;
    }

    // Sentinel on non-rider ctrl channels [0, 1, 2]
    const std::array<int, 3> non_rider_ctrl = {0, 1, 2};
    for (const int ch : non_rider_ctrl) {
        ctrl[to_index(ch)] = -77.0f;
    }

    gpu_write_rider_qfrc_and_ctrl(
        rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data());

    const GpuRiderOutput step_out =
        gpu_compute_rider_step(rp, idx, qpos.data(), qvel.data());
    const std::span<const float> step_envelope_torque{step_out.envelope_torque_nm};
    require(
        gpu_isfinite(step_out.envelope_energy_j) && step_out.envelope_energy_j > 0.0f,
        "envelope_energy_j must be finite and positive when joints are past limits");

    // Non-rider ctrl channels {0, 1, 2} must remain untouched
    for (const int ch : non_rider_ctrl) {
        require(ctrl[to_index(ch)] == -77.0f, "non-rider ctrl channel was modified");
    }

    // Verify all 9 rider DOFs and all 9 rider actuator channels
    for (std::size_t j = 0; j < kNumRiderJoints; ++j) {
        const int dof = rider_dof[j];
        const int ctrl_id = rider_ctrl[j];
        require(ctrl_id == expected_ctrl[j], "rider_ctrl index must match expected");

        const float qfrc_val = qfrc[to_index(dof)];
        const float ctrl_val = ctrl[to_index(ctrl_id)];

        require(gpu_isfinite(qfrc_val), "qfrc_world rider DOF must be finite");
        require(gpu_isfinite(ctrl_val), "ctrl_world rider channel must be finite");
        require(std::abs(step_envelope_torque[j]) > 0.0f, "step_out envelope torque must be non-zero");
        if (j < kFirstUpperBodyJoint) {
            require(ctrl_val == 0.0f, "leg ctrl_world channel must be exactly zero (legs are not driven)");
        } else {
            require(std::abs(ctrl_val) > 0.0f, "upper-body ctrl_world actuator torque must be non-zero");
        }

        // Verify += accumulation: qfrc was pre-populated with 12.5f
        const float expected_qfrc = 12.5f + step_envelope_torque[j];
        require_near(
            qfrc_val, expected_qfrc, 1e-4f,
            "qfrc[dof] must accumulate via += (pre-populated 12.5f + envelope torque)");

        // Verify restoring direction of envelope torque
        if (j % 2 == 0) {
            require(step_envelope_torque[j] > 0.0f, "lower-limit violation must produce positive torque");
        } else {
            require(step_envelope_torque[j] < 0.0f, "upper-limit violation must produce negative torque");
        }

        // Verify directional torque clamping bounds
        const GpuJointEnvelopeSpec &spec = joints[j];
        const float max_pos = (spec.sign_flip != 0) ? spec.max_flex_torque_nm : spec.max_ext_torque_nm;
        const float max_neg = (spec.sign_flip != 0) ? spec.max_ext_torque_nm : spec.max_flex_torque_nm;
        require(
            ctrl_val >= -max_neg - 1e-5f && ctrl_val <= max_pos + 1e-5f,
            "ctrl_world channel exceeded directional torque bounds");
    }

    // Verify saturation clamping under extreme positive error (qpos = -100.0f -> clamps to +max_pos)
    for (std::size_t j = 0; j < kNumRiderJoints; ++j) {
        qpos[to_index(rider_qpos[j])] = -100.0f;
        qvel[to_index(rider_dof[j])] = 0.0f;
    }
    gpu_write_rider_qfrc_and_ctrl(
        rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data());
    for (std::size_t j = 0; j < kFirstUpperBodyJoint; ++j) {
        require(ctrl[to_index(rider_ctrl[j])] == 0.0f, "leg ctrl_world channel must stay zero under saturation error");
    }
    for (std::size_t j = kFirstUpperBodyJoint; j < kNumRiderJoints; ++j) {
        const GpuJointEnvelopeSpec &spec = joints[j];
        const float max_pos = (spec.sign_flip != 0) ? spec.max_flex_torque_nm : spec.max_ext_torque_nm;
        require_near(
            ctrl[to_index(rider_ctrl[j])], max_pos, 1e-4f,
            "ctrl_world channel must clamp to positive directional limit");
    }

    // Verify saturation clamping under extreme negative error (qpos = +100.0f -> clamps to -max_neg)
    for (std::size_t j = 0; j < kNumRiderJoints; ++j) {
        qpos[to_index(rider_qpos[j])] = 100.0f;
        qvel[to_index(rider_dof[j])] = 0.0f;
    }
    gpu_write_rider_qfrc_and_ctrl(
        rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data());
    for (std::size_t j = 0; j < kFirstUpperBodyJoint; ++j) {
        require(ctrl[to_index(rider_ctrl[j])] == 0.0f, "leg ctrl_world channel must stay zero under saturation error");
    }
    for (std::size_t j = kFirstUpperBodyJoint; j < kNumRiderJoints; ++j) {
        const GpuJointEnvelopeSpec &spec = joints[j];
        const float max_neg = (spec.sign_flip != 0) ? spec.max_ext_torque_nm : spec.max_flex_torque_nm;
        require_near(
            ctrl[to_index(rider_ctrl[j])], -max_neg, 1e-4f,
            "ctrl_world channel must clamp to negative directional limit");
    }

    // Verify negative/out-of-bounds index guards in gpu_compute_rider_step
    GpuModelIndices guard_idx = idx;
    guard_idx.rider_qpos[2] = -1;
    guard_idx.rider_dof[2] = -1;
    const GpuRiderOutput guard_out =
        gpu_compute_rider_step(rp, guard_idx, qpos.data(), qvel.data());
    require(guard_out.envelope_torque_nm[2] == 0.0f, "negative index must yield zero envelope torque");
    require(guard_out.actuator_ctrl_nm[2] == 0.0f, "negative index must yield zero actuator ctrl");

    // Verify rider_enabled == 0 zeroes active ctrl while keeping passive envelope qfrc accumulation
    rp.rider_enabled = 0;
    for (const int dof : rider_dof) {
        qfrc[to_index(dof)] = 5.0f;
    }
    gpu_write_rider_qfrc_and_ctrl(
        rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data());
    for (std::size_t j = 0; j < kNumRiderJoints; ++j) {
        require(ctrl[to_index(rider_ctrl[j])] == 0.0f, "disabled rider must output zero ctrl");
        require(qfrc[to_index(rider_dof[j])] != 5.0f, "passive envelope qfrc must remain active and accumulate");
    }
}

// Legs (slots 0..3) are passive followers of the crank through the connect_foot_* equality
// constraints, so the rider kernel must not drive them: their ctrl is exactly 0 at every crank
// phase. The upper body (slots 4..8) must be independent of crank phase.
void test_leg_joints_do_not_drive_crank() {
    const GpuModelIndices idx{};
    const GpuRiderParams rp{};
    const auto rider_ctrl = std::span{idx.rider_ctrl};
    const auto rider_dof = std::span{idx.rider_dof};
    const auto rider_qpos = std::span{idx.rider_qpos};
    const std::span<const GpuJointEnvelopeSpec> joints{rp.joints};
    const std::span<const float> target_qpos{rp.target_qpos_rad};

    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    std::array<float, kMaxNv> qfrc{};
    std::array<float, kMaxNu> ctrl{};

    // Leg posture at the centre of the ROM interior: envelope torque is zero by construction, so
    // any nonzero leg ctrl could only come from the drive path under test. Non-zero leg velocity
    // exercises velocity-proportional terms as well.
    for (std::size_t j = 0; j < kFirstUpperBodyJoint; ++j) {
        const GpuJointEnvelopeSpec &spec = joints[j];
        const GpuVec2 range =
            gpu_joint_q_range(spec.min_deg, spec.max_deg, spec.ref_deg, spec.sign_flip);
        const float lo = range.x + spec.margin_rad;
        const float hi = range.y - spec.margin_rad;
        require(lo < hi, "leg ROM interior must be non-empty");
        qpos[to_index(rider_qpos[j])] = 0.5f * (lo + hi);
        qvel[to_index(rider_dof[j])] = 2.0f;
    }

    // Upper body 0.1 rad off target so posture PD yields nonzero ctrl (valid-input control).
    for (std::size_t j = kFirstUpperBodyJoint; j < kNumRiderJoints; ++j) {
        qpos[to_index(rider_qpos[j])] = target_qpos[j] + 0.1f;
        qvel[to_index(rider_dof[j])] = 0.0f;
    }

    std::array<float, kNumRiderJoints> upper_ctrl_ref{};
    for (int k = 0; k < 36; ++k) {
        qpos[to_index(idx.crank_spin_qpos)] = static_cast<float>(k) * (2.0f * static_cast<float>(kPi) / 36.0f);

        // Sentinel on rider ctrl channels so a skipped write is detected.
        for (const int ctrl_id : rider_ctrl) {
            ctrl[to_index(ctrl_id)] = 99.0f;
        }
        gpu_write_rider_qfrc_and_ctrl(rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data());
        const GpuRiderOutput out = gpu_compute_rider_step(rp, idx, qpos.data(), qvel.data());
        const std::span<const float> envelope_torque{out.envelope_torque_nm};
        const std::span<const float> actuator_ctrl{out.actuator_ctrl_nm};

        for (std::size_t j = 0; j < kFirstUpperBodyJoint; ++j) {
            require(envelope_torque[j] == 0.0f,
                    "leg envelope torque must stay zero inside the ROM interior");
            require(actuator_ctrl[j] == 0.0f,
                    "leg actuator ctrl must be exactly zero at every crank phase");
            require(!std::signbit(actuator_ctrl[j]),
                    "leg actuator ctrl must be +0.0f, not -0.0f");
            require(ctrl[to_index(rider_ctrl[j])] == 0.0f && !std::signbit(ctrl[to_index(rider_ctrl[j])]),
                    "leg ctrl channel must be written as +0.0f");
        }

        for (std::size_t j = kFirstUpperBodyJoint; j < kNumRiderJoints; ++j) {
            if (k == 0) {
                upper_ctrl_ref[j] = actuator_ctrl[j];
                require(std::abs(upper_ctrl_ref[j]) > 1e-3f,
                        "upper-body posture ctrl must be non-zero for the test posture");
            }
            require(actuator_ctrl[j] == upper_ctrl_ref[j],
                    "upper-body ctrl must be identical across crank phases");
            require(ctrl[to_index(rider_ctrl[j])] == upper_ctrl_ref[j],
                    "upper-body ctrl channel must match kernel output");
        }
    }
}

constexpr std::array<TestCase, 4> cases{{
    {.name = "anatomical_angle_and_q_range",
     .run = test_anatomical_angle_and_q_range},
    {.name = "soft_edge_torque_and_potential",
     .run = test_soft_edge_torque_and_potential},
    {.name = "write_rider_qfrc_and_ctrl", .run = test_write_rider_qfrc_and_ctrl},
    {.name = "leg_joints_do_not_drive_crank",
     .run = test_leg_joints_do_not_drive_crank},
}};

int run_case(const TestCase &test_case) {
    try {
        test_case.run();
        std::cout << "PASS " << test_case.name << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
        return 1;
    }
}
} // namespace

int main(int argc, char **argv) {
    try {
        const std::span<char *> arguments =
            bike::platform::main_arguments(argc, argv);
        if (arguments.size() == 2 &&
            std::string_view(arguments[1]) == "--list") {
            for (const TestCase &test_case: cases)
                std::cout << test_case.name << '\n';
            return 0;
        }
        if (arguments.size() > 2) {
            std::cerr << "usage: bike_test_gpu_rider [case-name|--list]\n";
            return 2;
        }

        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
        bool found = requested == "all";
        int status = 0;
        for (const TestCase &test_case: cases) {
            if (requested == "all" || requested == test_case.name) {
                found = true;
                if (run_case(test_case) != 0)
                    status = 1;
            }
        }
        if (!found) {
            std::cerr << "unknown gpu rider case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        // A throwing argument envelope is a harness failure, not a usage error.
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
