// native/tests/gpu/test_gpu_rider.cpp — Standalone C++23 unit test for
// native/src/gpu/rider_kernel.hpp.
#include "../../src/gpu/rider_kernel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDegToRadRef = kPi / 180.0;

struct RefJointEnvelope {
    double neutral_anatomical_rad;
    int direction;
    double minimum_anatomical_rad;
    double maximum_anatomical_rad;
};

struct RefSoftEdgeRow {
    double torque;
    double stored_j;
};

// Reference port from native/src/rider/spindle_math.cpp:471-479
std::array<double, 2> ref_joint_q_range(const RefJointEnvelope &envelope) {
    const double a = (envelope.minimum_anatomical_rad -
                      envelope.neutral_anatomical_rad) /
                     static_cast<double>(envelope.direction);
    const double b = (envelope.maximum_anatomical_rad -
                      envelope.neutral_anatomical_rad) /
                     static_cast<double>(envelope.direction);
    return {std::min(a, b), std::max(a, b)};
}

// Reference port from native/src/rider/spindle_math.cpp:481-489
double ref_anatomical_angle(const RefJointEnvelope &envelope, double joint_q) {
    const double value =
        envelope.neutral_anatomical_rad +
        static_cast<double>(envelope.direction) * joint_q;
    return std::atan2(std::sin(value), std::cos(value));
}

// Reference port from native/src/rider/spindle_math.cpp:376-391
RefSoftEdgeRow ref_soft_edge_scalar(
    double q,
    double lower,
    double upper,
    double stiffness_nm_rad,
    double margin_rad
) {
    const double margin = std::min(margin_rad, (upper - lower) / 2.0);
    const double left = std::max(lower + margin - q, 0.0);
    const double right = std::max(q - upper + margin, 0.0);
    return {
        .torque = stiffness_nm_rad * (left - right),
        .stored_j = 0.5 * stiffness_nm_rad * (left * left + right * right),
    };
}

double relative_error(double actual, double expected) {
    const double denom = std::max(std::abs(expected), 1e-2);
    return std::abs(actual - expected) / denom;
}

void require(bool cond, const char *msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        std::exit(1);
    }
}

void test_anatomical_angle_and_q_range() {
    using namespace bike::gpu;

    struct TestCase {
        float min_deg;
        float max_deg;
        float ref_deg;
    };
    constexpr std::array<TestCase, 4> cases{{
        {-120.0f, 15.0f, 10.0f},
        {-135.0f, 0.0f, -5.0f},
        {-45.0f, 75.0f, 20.0f},
        {-60.0f, 140.0f, -15.0f},
    }};

    for (const auto &tc : cases) {
        for (int sign_flip : {0, 1}) {
            const RefJointEnvelope env{
                .neutral_anatomical_rad = static_cast<double>(tc.ref_deg) * kDegToRadRef,
                .direction = (sign_flip != 0) ? -1 : 1,
                .minimum_anatomical_rad = static_cast<double>(tc.min_deg) * kDegToRadRef,
                .maximum_anatomical_rad = static_cast<double>(tc.max_deg) * kDegToRadRef,
            };

            const auto ref_range = ref_joint_q_range(env);
            const GpuVec2 gpu_range =
                gpu_joint_q_range(tc.min_deg, tc.max_deg, tc.ref_deg, sign_flip);

            require(gpu_range.x < gpu_range.y, "q_lo must be strictly below q_hi");
            require(
                relative_error(static_cast<double>(gpu_range.x), ref_range[0]) < 1e-5,
                "gpu_joint_q_range q_lo mismatch vs spindle_math.cpp");
            require(
                relative_error(static_cast<double>(gpu_range.y), ref_range[1]) < 1e-5,
                "gpu_joint_q_range q_hi mismatch vs spindle_math.cpp");

            for (float q_rad : {-2.2f, -1.1f, -0.35f, 0.0f, 0.45f, 1.3f, 2.5f}) {
                const double ref_ang = ref_anatomical_angle(env, static_cast<double>(q_rad));
                const float gpu_ang = gpu_anatomical_angle(q_rad, tc.ref_deg, sign_flip);
                require(
                    relative_error(static_cast<double>(gpu_ang), ref_ang) < 1e-5,
                    "gpu_anatomical_angle mismatch vs spindle_math.cpp");
            }
        }
    }
}

void test_soft_edge_torque_and_potential() {
    using namespace bike::gpu;

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
    // potential energy matching spindle_math.cpp within < 1e-5 relative error.
    float prev_tau_lo = 0.0f;
    float prev_pot_lo = 0.0f;
    for (int step = 1; step <= 25; ++step) {
        const float pen = 0.015f * static_cast<float>(step);
        const float q = (q_lo + margin) - pen;
        const float gpu_tau = gpu_soft_edge_torque(q, 0.0f, q_lo, q_hi, margin, k_edge, 0.0f);
        const float gpu_pot = gpu_soft_edge_potential(q, q_lo, q_hi, margin, k_edge);
        const RefSoftEdgeRow ref = ref_soft_edge_scalar(
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
    // potential energy matching spindle_math.cpp within < 1e-5 relative error.
    float prev_tau_hi = 0.0f;
    float prev_pot_hi = 0.0f;
    for (int step = 1; step <= 25; ++step) {
        const float pen = 0.015f * static_cast<float>(step);
        const float q = (q_hi - margin) + pen;
        const float gpu_tau = gpu_soft_edge_torque(q, 0.0f, q_lo, q_hi, margin, k_edge, 0.0f);
        const float gpu_pot = gpu_soft_edge_potential(q, q_lo, q_hi, margin, k_edge);
        const RefSoftEdgeRow ref = ref_soft_edge_scalar(
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
    using namespace bike::gpu;

    GpuRiderParams rp{};
    // Test both sign_flip = 0 and sign_flip = 1 in the 6-DOF table
    rp.joints[1].sign_flip = 1;
    rp.joints[1].min_deg = 0.0f;
    rp.joints[1].max_deg = 135.0f;

    const GpuModelIndices idx{};
    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    std::array<float, kMaxNv> qfrc{};
    std::array<float, kMaxNu> ctrl{};

    // Set crank phase so sin(phase) and cos(phase) are non-zero
    qpos[idx.crank_spin_qpos] = 0.65f;

    // Place rider joints near/past envelope edges and away from target posture
    // so both envelope_torque_nm and actuator_ctrl_nm are actively exercised.
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const auto &spec = rp.joints[j];
        const GpuVec2 range =
            gpu_joint_q_range(spec.min_deg, spec.max_deg, spec.ref_deg, spec.sign_flip);
        // Push even-indexed joints below q_lo and odd-indexed joints above q_hi
        qpos[idx.rider_qpos[j]] = (j % 2 == 0) ? (range.x - 0.05f) : (range.y + 0.05f);
        qvel[idx.rider_dof[j]] = (j % 2 == 0) ? -0.4f : 0.4f;
    }

    // Sentinel on non-rider ctrl channels [0..5]
    for (int i = 0; i < 6; ++i) {
        ctrl[i] = -77.0f;
    }

    gpu_write_rider_qfrc_and_ctrl(
        rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data());

    const GpuRiderOutput step_out =
        gpu_compute_rider_step(rp, idx, qpos.data(), qvel.data());
    require(
        gpu_isfinite(step_out.envelope_energy_j) && step_out.envelope_energy_j > 0.0f,
        "envelope_energy_j must be finite and positive when joints are past limits");

    // Non-rider ctrl channels [0..5] must remain untouched
    for (int i = 0; i < 6; ++i) {
        require(ctrl[i] == -77.0f, "non-rider ctrl channel [0..5] was modified");
    }

    // Verify all 6 rider DOFs and all 6 rider actuator channels [6..11]
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const int dof = idx.rider_dof[j];
        const int ctrl_id = idx.rider_ctrl[j];
        require(ctrl_id == 6 + j, "rider_ctrl index must map to channels 6..11");

        const float qfrc_val = qfrc[dof];
        const float ctrl_val = ctrl[ctrl_id];

        require(gpu_isfinite(qfrc_val), "qfrc_world rider DOF must be finite");
        require(gpu_isfinite(ctrl_val), "ctrl_world rider channel must be finite");
        require(std::abs(qfrc_val) > 0.0f, "qfrc_world envelope torque must be non-zero past limit");
        require(std::abs(ctrl_val) > 0.0f, "ctrl_world actuator torque must be non-zero");

        // Verify restoring direction of envelope torque
        if (j % 2 == 0) {
            require(qfrc_val > 0.0f, "lower-limit violation must produce positive qfrc torque");
        } else {
            require(qfrc_val < 0.0f, "upper-limit violation must produce negative qfrc torque");
        }

        // Verify directional torque clamping bounds
        const auto &spec = rp.joints[j];
        const float max_pos = (spec.sign_flip != 0) ? spec.max_flex_torque_nm : spec.max_ext_torque_nm;
        const float max_neg = (spec.sign_flip != 0) ? spec.max_ext_torque_nm : spec.max_flex_torque_nm;
        require(
            ctrl_val >= -max_neg - 1e-5f && ctrl_val <= max_pos + 1e-5f,
            "ctrl_world channel exceeded directional torque bounds");
    }

    // Verify saturation clamping under extreme posture error
    for (int j = 0; j < kNumRiderJoints; ++j) {
        qpos[idx.rider_qpos[j]] = -100.0f;
        qvel[idx.rider_dof[j]] = 0.0f;
    }
    gpu_write_rider_qfrc_and_ctrl(
        rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data());
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const auto &spec = rp.joints[j];
        const float max_pos = (spec.sign_flip != 0) ? spec.max_flex_torque_nm : spec.max_ext_torque_nm;
        require(
            std::abs(ctrl[idx.rider_ctrl[j]] - max_pos) < 1e-4f,
            "ctrl_world channel must clamp to positive directional limit");
    }

    // Verify rider_enabled == 0 zeroes active ctrl while keeping passive envelope qfrc
    rp.rider_enabled = 0;
    gpu_write_rider_qfrc_and_ctrl(
        rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data());
    for (int j = 0; j < kNumRiderJoints; ++j) {
        require(ctrl[idx.rider_ctrl[j]] == 0.0f, "disabled rider must output zero ctrl");
        require(qfrc[idx.rider_dof[j]] > 0.0f, "passive envelope qfrc must remain active");
    }
}

} // namespace

int main() {
    test_anatomical_angle_and_q_range();
    test_soft_edge_torque_and_potential();
    test_write_rider_qfrc_and_ctrl();
    std::printf("PASS: test_gpu_rider (all checks green)\n");
    return 0;
}
