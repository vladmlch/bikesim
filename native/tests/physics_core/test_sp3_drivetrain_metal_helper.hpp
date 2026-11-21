#pragma once

#include "physics_core/assist.hpp"
#include "physics_core/pedaling.hpp"
#include "physics_core/ratchet.hpp"
#include "physics_core/shifting.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace bike::physics_core::test_sp3_metal {

struct ShiftingStepInput {
    float cadence_rpm = 0.0f;
    float required_cadence_rpm = 0.0f;
    float dt = 0.00125f;
    float rear_slip_mps = 0.0f;
    float q_driver = 0.0f;
    float q_driven = 0.0f;
    RiderShiftInput rider{};
    u8 pedaling = 1;
    u8 braking = 0;
    u8 rear_in_contact = 1;
    u8 has_rear_slip = 0;
};

struct ShiftingStepRecord {
    ShiftingState<float> state{};
    RatchetBoundState<float> ratchet{};
    std::array<float, 26> tendon_ranges{};
    float torque_factor = 1.0f;
    u8 shifted = 0;
    u8 status = 0;
    std::array<u8, 2> pad{};
};

struct PedalingStepInput {
    float phase_rad = 0.0f;
    float rate_rad_s = 0.0f;
    float required_cadence_rpm = 0.0f;
    float effort_nm = 0.0f;
    float dt = 0.00125f;
    float shaft_rad_s = 0.0f;
    float ceiling_nm = 0.0f;
    float torque_factor = 1.0f;
    float ripple = 0.35f;
    float dq_crank_rebase = 0.0f;
    float dq_wheel_rebase = 0.0f;
    u8 braking = 0;
    u8 reposition_request = 0;
    u8 has_shaft_rad_s = 0;
    u8 has_ceiling_nm = 0;
    u8 cut_after_ceiling = 0;
    u8 do_rebase = 0;
    std::array<u8, 2> pad{};
};

struct PedalingStepRecord {
    PedalingOutput<float> output{};
    PedalingState<float> state{};
    float human_crank_torque_nm = 0.0f;
    u8 status = 0;
    u8 torque_status = 0;
    std::array<u8, 2> pad{};
};

struct AssistStepInput {
    float human_nm = 0.0f;
    float cadence_rpm = 0.0f;
    float speed_mps = 0.0f;
    float dt = 0.00125f;
    float torque_request_nm = 0.0f;
    float shaft_rpm = 0.0f;
    u8 braking = 0;
    u8 has_torque_request = 0;
    u8 has_shaft_rpm = 0;
    u8 pad = 0;
};

struct AssistStepRecord {
    float torque = 0.0f;
    AssistState<float> state{};
    u8 status = 0;
    std::array<u8, 3> pad{};
};

struct MetalSp3Request {
    std::string physics_core_dir;

    ShiftingParams<float> shift_params{};
    RatchetBoundState<float> shift_init_ratchet{};
    std::vector<ShiftingStepInput> shift_steps;

    PedalingParams<float> pedal_params{};
    std::vector<PedalingStepInput> pedal_steps;

    AssistParams<float> assist_params{};
    std::vector<AssistStepInput> assist_steps;
};

struct MetalSp3Output {
    bool device_available = false;
    std::string device_name;
    std::string error_message;
    bool torture_3_0_passed = false;
    bool torture_3_1_passed = false;

    std::vector<ShiftingStepRecord> shift_records;
    std::vector<PedalingStepRecord> pedal_records;
    std::vector<AssistStepRecord> assist_records;
};

[[nodiscard]] MetalSp3Output run_metal_sp3_parity(const MetalSp3Request &req);

} // namespace bike::physics_core::test_sp3_metal
