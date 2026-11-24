#pragma once

#include "physics_core/common.hpp"
#include "physics_core/efc_monitors.hpp"
#include "physics_core/front_lift.hpp"
#include "physics_core/launch_plan.hpp"
#include "physics_core/rider_intent.hpp"

#include <memory>
#include <string>
#include <vector>

namespace bike::physics_core::test_sp3b_metal {

struct MetalSp3Request {
    std::string physics_core_dir;

    // 1. PortableTrig inputs
    std::vector<float> trig_x;
    std::vector<float> trig_y;

    // 2. LaunchPlan sequence
    LaunchParams<float> launch_cfg{};
    std::vector<u8> launch_has_percepts;
    std::vector<PerceptsCore<float>> launch_percepts;
    std::vector<ShiftFeedbackCore> launch_shift;
    std::vector<u8> launch_eligible;
    std::vector<u8> launch_ext_brake;

    // 3. FrontLiftReaction sequence
    FrontLiftParams<float> lift_cfg{};
    std::vector<u8> lift_has_percepts;
    std::vector<PerceptsCore<float>> lift_percepts;

    // 4. RiderIntent + RoadProfile + SurfaceLabels sequence
    IntentParams<float> intent_cfg{};
    FrontLiftParams<float> intent_lift_cfg{};
    LaunchParams<float> intent_launch_cfg{};
    std::vector<float> road_x;
    std::vector<float> road_z;
    SurfaceLabel default_surface = SurfaceLabel::hardpack;
    std::vector<SurfaceSectionCore<float>> surface_sections;
    float road_lookahead_m = 2.0f;
    float road_spacing_m = 0.05f;
    std::vector<IntentSignalsCore<float>> intent_signals;
    std::vector<float> wheel_front_x;
    std::vector<float> wheel_rear_x;
    std::vector<float> speed_mps;
    std::vector<i32> rear_teeth;
    std::vector<float> hand_press_n;
    std::vector<LaunchContextCore> intent_launch_ctx;

    // 5. Post-solve efc & monitors on welded.xml state
    EfcScanConfig<float> efc_cfg{};
    i32 nefc = 0;
    i32 efc_J_stride = 0;
    i32 nbody = 0;
    i32 ncon = 0;
    std::vector<i32> efc_type;
    std::vector<i32> efc_id;
    std::vector<float> efc_J;
    std::vector<float> efc_force;
    std::vector<float> body_xmat;
    std::vector<i32> contact_geom1;
    std::vector<i32> contact_geom2;
    std::vector<i32> contact_efc_address;
    std::vector<i32> contact_dim;

    GroundedFilterConfig grounded_cfg{};
    BalanceMonitorConfig<float> balance_cfg{};
    RollbackBrakeConfig<float> rollback_cfg{};
    std::vector<u8> mon_raw_grounded;
    std::vector<float> mon_speed_mps;
    std::vector<float> mon_pos_m;
    std::vector<u8> mon_caller_braking;
    std::vector<u8> mon_rider_enabled;
};

struct MetalSp3Output {
    bool device_available = false;
    std::string device_name;
    std::string error_message;

    std::vector<float> atan_out;
    std::vector<float> atan2_out;
    std::vector<float> sin_out;
    std::vector<float> cos_out;

    std::vector<LaunchCommandCore<float>> launch_out;
    LaunchState<float> launch_final_state{};

    std::vector<ReactionCommandCore<float>> lift_out;
    FrontLiftState<float> lift_final_state{};

    std::vector<IntentResolutionCore<float>> intent_out;
    std::vector<float> road_posture_grade;
    std::vector<float> road_preview_grade;
    RiderIntentState<float> intent_final_state{};

    EfcScanResult<float> efc_out{};
    std::vector<u8> mon_grounded_out;
    std::vector<u8> mon_balance_lost_out;
    std::vector<float> mon_rollback_demand_out;
    BalanceMonitorState<float> mon_balance_final{};
};

[[nodiscard]] std::unique_ptr<MetalSp3Output> run_metal_sp3_parity(const MetalSp3Request &req);

} // namespace bike::physics_core::test_sp3b_metal
