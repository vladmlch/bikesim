#pragma once

#include "physics_core/brake_hold.hpp"
#include "physics_core/joint_envelope.hpp"
#include "physics_core/ratchet.hpp"
#include "physics_core/resistance.hpp"
#include "physics_core/suspension.hpp"
#include "physics_core/tyre_types.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace bike::physics_core::test_sp2_metal {

struct MetalSp2Request {
    std::string physics_core_dir;

    // 1. logf / powf
    std::vector<float> log_in;
    std::vector<float> pow_base;
    std::vector<float> pow_exp;

    // 2. Suspension (n_susp cases)
    std::vector<SuspensionParams<float>> susp_params;
    std::vector<float> susp_fork_q;
    std::vector<float> susp_fork_v;
    std::vector<float> susp_shock_q;
    std::vector<float> susp_shock_v;

    // 3. Resistance (1 full welded.xml state with 128 patches)
    i32 res_nv = 0;
    i32 res_njnt = 0;
    ResistanceParams<float> res_cfg{};
    std::array<GpuWheelChain, 2> res_wheel_chains{};
    GpuWheelChain res_frame_chain{};
    std::vector<float> res_body_xpos;
    std::vector<float> res_jnt_xanchor;
    std::vector<float> res_jnt_xaxis;
    std::vector<float> res_qvel;
    std::vector<float> res_front_loads;
    std::vector<u8> res_front_work;
    float res_front_eff_r = 0.0f;
    std::vector<float> res_front_xpos;
    std::vector<float> res_front_xmat;
    std::vector<float> res_rear_loads;
    std::vector<u8> res_rear_work;
    float res_rear_eff_r = 0.0f;
    std::vector<float> res_rear_xpos;
    std::vector<float> res_rear_xmat;
    std::vector<float> res_frame_xpos;
    std::vector<float> res_frame_xmat;

    // 4. Brake hold + apply + rebase
    BrakeHoldParams<float> brake_spec{};
    float brake_ceiling_nm = 0.0f;
    float brake_front_demand = 0.0f;
    float brake_rear_demand = 0.0f;
    float brake_theta_init = 0.0f;
    float brake_omega = 0.0f;
    float brake_latch_init = 0.0f;

    // 5. Rider joint envelope
    RiderJointEnvelopeSpec<float> env_spec{};
    i32 env_nq = 0;
    i32 env_nv = 0;
    std::vector<float> env_qpos;

    // 6. Ratchet + cassette + world rebase
    WorldRotationsRebaseState<float> rebase_in{};
    float ratchet_old_ratio = 0.0f;
    float ratchet_new_ratio = 0.0f;
    i32 active_cog = 0;
};

struct MetalSp2Output {
    bool device_available = false;
    std::string device_name;
    std::string error_message;

    std::vector<float> log_out;
    std::vector<float> pow_out;

    std::vector<SuspensionResult<float>> susp_out;
    std::vector<u8> susp_status;

    std::vector<float> res_rolling_out;
    std::vector<float> res_aero_out;
    u8 res_status = 0;

    float brake_loss_front = 0.0f;
    float brake_loss_rear = 0.0f;
    u8 brake_changed = 0;
    float brake_hold_torque = 0.0f;
    float brake_rebased_theta = 0.0f;
    float brake_rebased_latch = 0.0f;
    u8 brake_has_latch = 0;

    std::vector<float> env_qfrc_out;
    float env_stored_j = 0.0f;
    u8 env_status = 0;

    WorldRotationsRebaseState<float> rebase_out{};
    float ratchet_phi = 0.0f;
    float ratchet_bound_after_reset = 0.0f;
    float ratchet_bound_after_prepare = 0.0f;
    float ratchet_bound_after_shift = 0.0f;
    std::vector<float> cassette_ranges;
    u8 ratchet_status = 0;
};

[[nodiscard]] MetalSp2Output run_metal_sp2_parity(const MetalSp2Request &req);

} // namespace bike::physics_core::test_sp2_metal
