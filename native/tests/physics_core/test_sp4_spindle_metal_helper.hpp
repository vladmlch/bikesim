#pragma once

#include "physics_core/common.hpp"
#include "physics_core/spindle_m2k2.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace bike::physics_core::test_sp4_metal {

struct Sp4StateSnapshot {
    SpindleCommand<float> cmd{};
    i32 step_index = 0;
    float min_elbow_flexion_rad = 0.0f;
    float initial_coast_hold_nm = 0.0f;
    std::vector<float> qpos;         // nq
    std::vector<float> qvel;         // nv
    std::vector<float> qfrc_bias;    // nv
    std::vector<float> body_xmat;    // nbody * 9
    std::vector<float> body_xpos;    // nbody * 3
    std::vector<float> jnt_xanchor;  // njnt * 3
    std::vector<float> jnt_xaxis;    // njnt * 3
    std::vector<float> site_xpos;    // nsite * 3
    std::vector<float> subtree_com;  // nbody * 3
    std::vector<float> d_M;          // nC
    std::vector<float> qM_dense;     // optional: nv * qM_row_stride (when qM_row_stride >= nv)
    std::vector<float> csr_vec;      // nv
    std::vector<float> qacc_p0;      // nv (solved qacc after pass0 ctrl)
    std::vector<float> qacc_p1;      // nv (solved qacc after pass1 ctrl)
    std::vector<float> qacc_p2;      // nv (solved qacc after pass2 ctrl)
};

struct MetalSp4Request {
    std::string physics_core_dir;
    SpindleM2K2Params<float> params{};
    std::vector<float> body_subtreemass; // nbody
    std::vector<i32> M_rowadr;           // nv
    std::vector<i32> M_rownnz;           // nv
    std::vector<i32> M_colind;           // nC
    i32 nC = 0;
    i32 qM_row_stride = 0;               // 0 -> CSR d_M; >= nv -> also verify dense qM & nullptr submass

    // 1. Analytic leg kinematics sweep
    std::vector<float> leg_phases;
    float leg_hip_x = 0.0f;
    float leg_hip_z = 0.0f;
    i32 leg_side = 0;

    // 2. Multi-state snapshots
    std::vector<Sp4StateSnapshot> states;
};

struct MetalSp4StateOutput {
    u8 has_min_lean = 0;
    float min_lean_rad = 0.0f;
    std::array<float, kNumUpperSpindleJoints> upper_targets{};
    u8 torso_sat = 0;
    u8 arms_sat = 0;
    std::array<float, 2> f_weight{};
    std::array<float, 2> f_weight_null_subm{};
    std::vector<float> csr_out;     // nv
    std::vector<float> dense_out;   // nv
    std::vector<float> ctrl_pass0;  // nu
    std::vector<float> ctrl_pass1;  // nu
    std::vector<float> ctrl_pass2;  // nu
    std::vector<float> ctrl_pass3;  // nu
    std::vector<float> ctrl_held;   // nu
    std::vector<float> ctrl_dense_pass3; // nu
    SpindleM2K2State<float> final_state{};
    SpindleM2K2StepScratch<float> final_scratch{};
};

struct MetalSp4Output {
    bool device_available = false;
    std::string device_name;
    std::string error_message;

    std::vector<LegKinematicsResult<float>> leg_kin_out;
    std::vector<MetalSp4StateOutput> state_out;
};

[[nodiscard]] std::unique_ptr<MetalSp4Output> run_metal_sp4_parity(const MetalSp4Request &req);

} // namespace bike::physics_core::test_sp4_metal
