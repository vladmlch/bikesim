// native/tests/gpu/test_gpu_rider.cpp — Unit and spindle_math parity tests for
// single-source physics_core::eval_rider_joint_envelope wired through batch_step.hpp.
#include "../../src/gpu/batch_step.hpp"
#include "../../src/platform/argv.hpp"
#include "../../src/rider/spindle_math.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using namespace bike::gpu;
using bike::physics_core::Status;
using TestFunction = void (*)();

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

template <class T>
void require_near(T actual, T expected, T tol, std::string_view what) {
    if (!(std::abs(actual - expected) <= tol)) {
        throw std::runtime_error(std::string(what) +
                                 ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected) +
                                 " tol=" + std::to_string(tol));
    }
}

constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

[[gnu::noinline]] void test_soft_edge_parity_and_zero_damping() {
    const GpuRiderParams rp = gpu_make_default_rider_params();
    const GpuModelIndices idx{};

    require(rp.envelope.count == kNumRiderJoints, "9 rider joints populated in default envelope");

    const std::span<const float> lo(rp.envelope.lower);
    const std::span<const float> hi(rp.envelope.upper);
    const std::span<const int> env_qp(rp.envelope.qpos_adrs);
    const std::span<const int> env_dof(rp.envelope.dof_adrs);
    for (int j = 0; j < kNumRiderJoints; ++j) {
        require(env_qp[to_index(j)] == -1 && env_dof[to_index(j)] == -1,
                "default rider envelope uses -1 sentinel for qpos_adrs/dof_adrs");
    }
    const std::span<const int> qp_adr(idx.rider_qpos);
    const std::span<const int> dof_adr(idx.rider_dof);

    std::array<float, kMaxNq> qpos_a{};
    std::array<float, kMaxNv> qvel_a{};
    std::array<float, kMaxNv> qvel_b{};
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const float q = (j % 2 == 0)
            ? (lo[to_index(j)] + 0.03f)
            : (hi[to_index(j)] - 0.025f);
        qpos_a[to_index(qp_adr[to_index(j)])] = q;
        qvel_a[to_index(dof_adr[to_index(j)])] = 0.0f;
        qvel_b[to_index(dof_adr[to_index(j)])] = 2.5f;
    }

    std::array<float, kMaxNv> qfrc_a{};
    std::array<float, kMaxNv> qfrc_b{};
    std::array<float, kMaxNu> ctrl_a{};
    std::array<float, kMaxNu> ctrl_b{};
    float energy_a = 0.0f;
    float energy_b = 0.0f;

    require(gpu_write_rider_qfrc_and_ctrl(
                rp, idx, qpos_a.data(), qvel_a.data(), qfrc_a.data(), ctrl_a.data(), &energy_a) ==
                Status::ok,
            "rider evaluation (stationary) ok");
    require(gpu_write_rider_qfrc_and_ctrl(
                rp, idx, qpos_a.data(), qvel_b.data(), qfrc_b.data(), ctrl_b.data(), &energy_b) ==
                Status::ok,
            "rider evaluation (moving) ok");

    double ref_energy = 0.0;
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const float q = qpos_a[to_index(qp_adr[to_index(j)])];
        const auto [tau_ref, e_ref] = ::spindle::soft_edge_scalar(
            static_cast<double>(q),
            static_cast<double>(lo[to_index(j)]),
            static_cast<double>(hi[to_index(j)]),
            static_cast<double>(rp.envelope.stiffness_nm_rad),
            static_cast<double>(rp.envelope.margin_rad));
        ref_energy += e_ref;

        const float tau_a = qfrc_a[to_index(dof_adr[to_index(j)])];
        const float tau_b = qfrc_b[to_index(dof_adr[to_index(j)])];
        require_near(tau_a, static_cast<float>(tau_ref), 1e-5f,
                     "GPU rider envelope matches spindle::soft_edge_scalar");
        // Ban #26: zero joint damping in soft-edge envelope regardless of velocity
        require_near(tau_a, tau_b, 1e-7f,
                     "rider ROM envelope applies zero joint damping (Ban #26)");
    }
    require_near(energy_a, static_cast<float>(ref_energy), 1e-4f,
                 "stored envelope energy matches spindle::soft_edge_scalar sum");
    require_near(energy_a, energy_b, 1e-7f, "stored energy is velocity-independent");
}

void test_passive_legs_and_upper_body_posture() {
    GpuRiderParams rp = gpu_make_default_rider_params();
    const GpuModelIndices idx{};
    const std::span<const int> qp_adr(idx.rider_qpos);
    const std::span<const int> dof_adr(idx.rider_dof);
    const std::span<const int> ctrl_adr(idx.rider_ctrl);
    const std::span<const float> tgt(rp.target_qpos_rad);
    const std::span<const float> kp(rp.kp_posture);
    const std::span<const float> kd(rp.kd_posture);

    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    std::array<float, kMaxNv> qfrc{};
    std::array<float, kMaxNu> ctrl{};

    for (int j = 0; j < kNumRiderJoints; ++j) {
        qpos[to_index(qp_adr[to_index(j)])] = tgt[to_index(j)] + 0.20f;
        qvel[to_index(dof_adr[to_index(j)])] = -0.35f;
    }

    require(gpu_write_rider_qfrc_and_ctrl(
                rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data()) == Status::ok,
            "rider step ok");

    for (int j = 0; j < 4; ++j) {
        require(ctrl[to_index(ctrl_adr[to_index(j)])] == 0.0f,
                "passive leg follower actuators keep ctrl == 0");
    }
    for (int j = 4; j < kNumRiderJoints; ++j) {
        const float expected =
            kp[to_index(j)] * (tgt[to_index(j)] - qpos[to_index(qp_adr[to_index(j)])]) -
            kd[to_index(j)] * qvel[to_index(dof_adr[to_index(j)])];
        require_near(ctrl[to_index(ctrl_adr[to_index(j)])], expected, 1e-5f,
                     "upper-body posture PD ctrl matches kp*(q*-q) - kd*qd");
    }

    // Disabled rider returns ok without mutating qfrc or ctrl
    rp.rider_enabled = 0;
    std::ranges::fill(qfrc, 1.5f);
    std::ranges::fill(ctrl, 2.5f);
    require(gpu_write_rider_qfrc_and_ctrl(
                rp, idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data()) == Status::ok,
            "disabled rider ok");
    require_near(qfrc[to_index(dof_adr[4])], 1.5f, 1e-7f, "disabled rider leaves qfrc untouched");
    require_near(ctrl[to_index(ctrl_adr[4])], 2.5f, 1e-7f, "disabled rider leaves ctrl untouched");
}

void test_bounds_guards() {
    const GpuRiderParams rp = gpu_make_default_rider_params();
    GpuModelIndices bad_idx{};
    bad_idx.nq = 5;
    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    std::array<float, kMaxNv> qfrc{};
    std::array<float, kMaxNu> ctrl{};
    require(gpu_write_rider_qfrc_and_ctrl(
                rp, bad_idx, qpos.data(), qvel.data(), qfrc.data(), ctrl.data()) ==
                Status::out_of_bounds,
            "envelope rejects out-of-bounds qpos index");
}

[[gnu::noinline]] void test_envelope_sentinel_fallback_and_override() {
    const GpuModelIndices idx{};
    const std::span<const int> idx_qp(idx.rider_qpos);
    const std::span<const int> idx_dof(idx.rider_dof);

    // 1. Default -1 sentinel falls back to idx.rider_qpos[j] / idx.rider_dof[j].
    const GpuRiderParams rp_fallback = gpu_make_default_rider_params();
    const std::span<const float> lo(rp_fallback.envelope.lower);
    std::array<float, kMaxNq> qpos_fb{};
    std::array<float, kMaxNv> qfrc_fb{};
    qpos_fb[to_index(idx_qp[0])] = lo[0] + 0.02f;
    float e_fb = 0.0f;
    require(gpu_write_rider_envelope_qfrc(
                rp_fallback, idx, qpos_fb.data(), qfrc_fb.data(), &e_fb) == Status::ok,
            "envelope -1 sentinel fallback succeeds");
    require(qfrc_fb[to_index(idx_dof[0])] > 0.1f && e_fb > 0.0f,
            "envelope -1 sentinel fallback writes torque at idx.rider_dof[0]");

    // 2. Explicit qpos_adrs >= 0 / dof_adrs >= 0 overrides idx.rider_qpos / idx.rider_dof.
    GpuRiderParams rp_override = rp_fallback;
    const int custom_qp = 2;
    const int custom_dof = 3;
    require(custom_qp != idx_qp[0] && custom_dof != idx_dof[0],
            "custom override indices distinct from idx.rider_qpos/dof[0]");
    std::span<int>(rp_override.envelope.qpos_adrs)[0] = custom_qp;
    std::span<int>(rp_override.envelope.dof_adrs)[0] = custom_dof;

    std::array<float, kMaxNq> qpos_ov{};
    std::array<float, kMaxNv> qfrc_ov{};
    qpos_ov[to_index(custom_qp)] = lo[0] + 0.02f;
    qpos_ov[to_index(idx_qp[0])] = 0.0f;
    float e_ov = 0.0f;
    require(gpu_write_rider_envelope_qfrc(
                rp_override, idx, qpos_ov.data(), qfrc_ov.data(), &e_ov) == Status::ok,
            "envelope explicit override succeeds");
    require_near(qfrc_ov[to_index(custom_dof)], qfrc_fb[to_index(idx_dof[0])], 1e-6f,
                 "explicit qpos_adrs/dof_adrs override writes expected torque to custom dof");
    require_near(qfrc_ov[to_index(idx_dof[0])], 0.0f, 1e-7f,
                 "explicit override leaves default idx.rider_dof[0] untouched");
}

constexpr std::array<TestCase, 4> cases{{
    {.name = "soft_edge_parity_and_zero_damping", .run = test_soft_edge_parity_and_zero_damping},
    {.name = "envelope_sentinel_fallback_and_override", .run = test_envelope_sentinel_fallback_and_override},
    {.name = "passive_legs_and_upper_body_posture", .run = test_passive_legs_and_upper_body_posture},
    {.name = "bounds_guards", .run = test_bounds_guards},
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
        const std::span<char *> arguments = bike::platform::main_arguments(argc, argv);
        if (arguments.size() == 2 && std::string_view(arguments[1]) == "--list") {
            for (const TestCase &test_case : cases) {
                std::cout << test_case.name << '\n';
            }
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
        for (const TestCase &test_case : cases) {
            if (requested == "all" || requested == test_case.name) {
                found = true;
                if (run_case(test_case) != 0) {
                    status = 1;
                }
            }
        }
        if (!found) {
            std::cerr << "unknown gpu rider case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
